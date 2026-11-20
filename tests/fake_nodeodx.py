"""假的 NodeODX（測試用）：依規格 R1 模擬 NodeODX v2.4.0 ＋ ODX v3.8.4 的 REST 行為，在 127.0.0.1 隨機埠上跑真的 HTTP。

不是 FakeSession：請求真的經過 requests → urllib3 → socket，上傳主體由這裡的串流 multipart 解析器邊收邊寫檔
（NodeODX 的 busboy 也是直接寫磁碟），所以分批、串流、斷線、回應遺失都是真的網路行為。

模擬的規格重點（N = NodeODX v2.4.0 原始碼行號，見 survey/odm_client.py 的說明）：
* 權杖只看 query ``token``；錯了回 HTTP 200 ``{"error": "Invalid authentication token: ..."}``。
* init 只解析 multipart（其他 content-type 的欄位被靜默丟掉）；set-uuid 標頭；不認得的參數名稱靜默丟掉。
* upload 欄名只能是 ``images``；同名檔用 'wx' 開，存在就在副檔名前加 ``_``；失敗／中斷的請求刪掉它寫的檔。
* commit 回 uuid 後才初始化：第一次 info 是初始化前（20、processingTime -1、imagesCount = tmp 檔數含 .txt），
  第二次起初始化完成（maxImages 檢查、.txt 搬到 gcp/）；之後排隊 → 執行 → 完成，每次 info 前進一步。
* 未提交的 uuid：/task/cancel、/task/remove 回 ``{"success": false, "error": "<uuid> not found"}``。
* all.zip：odm_report/stats.json（OpenSfM a125a3d 的欄位）、report.pdf、shots.geojson、log.json、images.json、
  orthophoto_tiles/、odm_dem/dsm.tif、task_output.txt。

可單獨當子行程跑（量客戶端記憶體時，伺服器不能跟客戶端在同一個行程）::

    python tests/fake_nodeodx.py --root DIR [--token T]      # stdout 第一行印 "PORT <n>"，stdin 關閉就結束
"""

from __future__ import annotations

import argparse
import io
import json
import re
import shutil
import socket
import sys
import threading
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

QUEUED, RUNNING, FAILED, COMPLETED, CANCELED = 10, 20, 30, 40, 50
_UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[1-7][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$", re.I)
_DISP_RE = re.compile(r'(\w+)="([^"]*)"')


def default_options(cpu: int = 16) -> list:
    """GET /options 的節點參數清單（N odmInfo.js:70-161 的形狀；預設值是字串）。"""
    def o(name, typ, value, domain=""):
        return {"name": name, "type": typ, "value": value, "domain": domain, "help": ""}
    return [o("orthophoto-resolution", "float", "5", "float > 0.0"), o("dsm", "bool", "false"),
            o("tiles", "bool", "false"), o("gps-accuracy", "float", "3", "positive float"),
            o("rolling-shutter", "bool", "false"), o("rolling-shutter-readout", "float", "0", "positive integer"),
            o("split", "int", "999999", "positive integer"), o("split-overlap", "float", "150", "positive float"),
            o("force-gps", "bool", "false"), o("max-concurrency", "int", str(cpu), "positive integer"),
            o("skip-report", "bool", "false"), o("use-exif", "bool", "false"), o("pc-ept", "bool", "false"),
            o("cog", "bool", "false"), o("gltf", "bool", "false")]


class _Abort(Exception):
    """模擬伺服器把連線切掉（不回應）。"""


class MultipartReader:
    """串流 multipart 解析：``feed(bytes)`` 回傳事件 ``("part", headers)``、``("data", bytes)``、``("end",)``、``("done",)``。"""

    def __init__(self, boundary: bytes):
        self.first = b"--" + boundary
        self.delim = b"\r\n--" + boundary
        self.buf = b""
        self.state = "start"

    def feed(self, data: bytes) -> list:
        self.buf += data
        ev = []
        while True:
            if self.state == "start":
                i = self.buf.find(self.first + b"\r\n")
                if i < 0:
                    return ev
                self.buf = self.buf[i + len(self.first) + 2:]
                self.state = "headers"
            elif self.state == "headers":
                i = self.buf.find(b"\r\n\r\n")
                if i < 0:
                    return ev
                head = self.buf[:i].decode("utf-8", "replace")
                self.buf = self.buf[i + 4:]
                hd = {}
                for line in head.split("\r\n"):
                    k, _, v = line.partition(":")
                    hd[k.strip().lower()] = v.strip()
                ev.append(("part", hd))
                self.state = "body"
            elif self.state == "body":
                i = self.buf.find(self.delim)
                if i < 0:
                    keep = len(self.delim)
                    if len(self.buf) > keep:
                        ev.append(("data", self.buf[:-keep]))
                        self.buf = self.buf[-keep:]
                    return ev
                if i:
                    ev.append(("data", self.buf[:i]))
                ev.append(("end",))
                self.buf = self.buf[i + len(self.delim):]
                self.state = "after"
            elif self.state == "after":
                if len(self.buf) < 2:
                    return ev
                if self.buf[:2] == b"--":
                    self.state = "done"
                    ev.append(("done",))
                    return ev
                self.buf = self.buf[2:]
                self.state = "headers"
            else:
                return ev


def parse_multipart(body, content_type: str) -> tuple:
    """把 multipart 主體（bytes 或可迭代的串流）整個解開：回傳 ``(欄位 {name: str}, 檔案 [(欄名, 檔名, bytes)])``。
    給 session 層的假節點用（小資料）；真 HTTP 的假節點邊收邊寫檔，不用這個。"""
    m = re.search(r"boundary=([^;]+)", content_type or "")
    if not m:
        return {}, []
    raw = body if isinstance(body, (bytes, bytearray)) else b"".join(iter(body))
    fields, files, cur = {}, [], None
    for ev in MultipartReader(m.group(1).strip().encode("ascii")).feed(bytes(raw)):
        if ev[0] == "part":
            disp = dict(_DISP_RE.findall(ev[1].get("content-disposition", "")))
            cur = [disp.get("name"), disp.get("filename"), b""]
        elif ev[0] == "data" and cur is not None:
            cur[2] += ev[1]
        elif ev[0] == "end" and cur is not None:
            if cur[1] is None:
                fields[cur[0]] = cur[2].decode("utf-8")
            else:
                files.append((cur[0], cur[1], cur[2]))
            cur = None
    return fields, files


class FakeNodeODX:
    """狀態與故障設定。``faults`` 是計數器：每用掉一次減一。"""

    def __init__(self, root, *, token=None, max_images=None, engine_version="3.8.4", node_max_concurrency=None,
                 options=None, steps=3, fail_task=False, stats="ok", cpu=16, report_pdf=True, shots=True,
                 images_json=True, log_warnings=(), bias_identity=False, empty_result=False):
        self.root = Path(root)
        (self.root / "tmp").mkdir(parents=True, exist_ok=True)
        (self.root / "data").mkdir(parents=True, exist_ok=True)
        self.token, self.max_images, self.engine_version = token, max_images, engine_version
        self.node_max_concurrency = node_max_concurrency
        self.options = options if options is not None else default_options(cpu)
        self.cpu, self.steps, self.fail_task, self.stats = cpu, steps, fail_task, stats
        self.report_pdf, self.shots = report_pdf, shots
        self.images_json, self.log_warnings, self.bias_identity = images_json, list(log_warnings), bias_identity
        #: 真 NodeODX 在 ODX 根本沒啟動時（找不到 winrun.bat）也回報完成，all.zip 只有 task_output.txt（2026-10-20 實測）
        self.empty_result = empty_result
        # 故障計數器（每用掉一次減一）：
        #   upload_500／upload_drop／upload_lose_response：上傳請求（存檔前 500／讀到一半斷線／存好後斷線）
        #   upload_502_after_store：檔案存好之後才回 502（例如反向代理逾時）
        #   info_drop：GET info 斷線；options_502：GET /options 回 502（例如代理擋掉）
        #   commit_drop：commit 讀完就斷線、沒處理；commit_lose_response：commit 處理好才斷線（之後再斷
        #   ``after_commit_drop`` 個任何請求，模擬節點短暫連不上）
        #   drop_any：接下來任何 N 個請求都斷線、不處理
        #   probe_drop：探測未提交上傳的空主體 POST 斷線
        self.faults = {"upload_500": 0, "upload_drop": 0, "upload_lose_response": 0, "info_drop": 0,
                       "upload_502_after_store": 0, "options_502": 0, "commit_drop": 0, "commit_lose_response": 0,
                       "drop_any": 0, "probe_drop": 0}
        self.after_commit_drop = 0
        self.tasks: dict = {}
        self.log: list = []
        self.lock = threading.Lock()
        self.server = None
        self.thread = None

    # ── 生命週期 ────────────────────────────────────────────────────────────
    def start(self) -> "FakeNodeODX":
        node = self

        class Handler(_Handler):
            fake = node
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.server.handle_error = lambda request, client_address: None      # 故障模擬會丟連線錯誤，不要洗版
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        return self

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.server.server_address[1]}"

    def stop(self) -> None:
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None

    # ── 查詢（給測試斷言用） ────────────────────────────────────────────────
    def tmp_files(self, uuid) -> list:
        d = self.root / "tmp" / uuid
        return sorted(p.name for p in d.iterdir() if p.name not in ("body.json", "_body.json")) if d.is_dir() else []

    def requests_to(self, prefix: str) -> list:
        return [r for r in self.log if r["path"].startswith(prefix)]

    # ── 任務狀態機（每次 info 前進一步） ──────────────────────────────────
    def _filtered_options(self, sent: list, skip_pp: bool) -> list:
        known = {o["name"]: o for o in self.options}
        out = []
        for o in sent:
            if o.get("name") not in known:
                continue                                    # N odmInfo.js:290-325：靜默丟掉
            v = o.get("value")
            if o["name"] == "max-concurrency" and self.node_max_concurrency is not None:
                v = min(int(v), self.node_max_concurrency)  # N odmInfo.js:308-315：靜默壓低
            out.append({"name": o["name"], "value": v})
        names = {o["name"] for o in out}
        if self.node_max_concurrency is not None and "max-concurrency" not in names and "max-concurrency" in known:
            out.append({"name": "max-concurrency", "value": self.node_max_concurrency})   # :327-334 自動補上
        if not skip_pp:
            for n in ("pc-ept", "cog", "gltf"):                                          # N Task.js:120-148
                if n in known and n not in names:
                    out.append({"name": n, "value": True})
        return out

    def _init_task(self, t: dict) -> None:
        tmp = self.root / "tmp" / t["uuid"]
        data = self.root / "data" / t["uuid"]
        images = data / "images"
        data.mkdir(parents=True, exist_ok=True)
        shutil.move(str(tmp), str(images))
        # R1 §2.3：張數檢查（taskNew.js:201-210,321-323）在把 *.txt 搬到 gcp/（:325-340）**之前** ⇒ .txt 也算進去
        n_img = len([p for p in images.iterdir() if p.name not in ("body.json", "_body.json")])
        if self.max_images is not None and n_img > self.max_images:
            t.update(status={"code": FAILED,
                             "errorMessage": f"{n_img} images uploaded, but this node can only process up to "
                                             f"{self.max_images}."}, processingTime=-1)
            shutil.rmtree(data, ignore_errors=True)
            return
        gcp_dir = data / "gcp"
        gcp_dir.mkdir()
        for p in sorted(images.iterdir()):
            if p.suffix.lower() == ".txt":
                shutil.move(str(p), str(gcp_dir / p.name))
        txt = sorted(p.name for p in gcp_dir.iterdir())
        t["geo"] = next((n for n in txt if n.lower() == "geo.txt"), None)
        t["gcp"] = next((n for n in txt if n.lower() not in ("geo.txt", "image_groups.txt")), None)
        t["images"] = sorted(p.name for p in images.iterdir())
        t["imagesCount"] = len(t["images"])
        t["status"] = {"code": QUEUED}

    def _start_output(self, t: dict) -> None:
        opts = {o["name"].replace("-", "_"): o["value"] for o in t["options"]}
        base = f"/var/www/data/{t['uuid']}"
        args = {"gcp": f"{base}/gcp/{t['gcp']}" if t.get("gcp") else "None",
                "geo": f"{base}/gcp/{t['geo']}" if t.get("geo") else "None",
                "force_gps": str(bool(opts.get("force_gps", False))),
                "max_concurrency": str(opts.get("max_concurrency", self.cpu)),
                "gps_accuracy": str(opts.get("gps_accuracy", 3.0)), "use_exif": "False",
                "orthophoto_resolution": str(opts.get("orthophoto_resolution", 5.0)),
                "dsm": str(bool(opts.get("dsm", False))), "tiles": str(bool(opts.get("tiles", False)))}
        t["output"] += [f"Initializing ODX {self.engine_version} - Mon Oct 20 10:00:00 2026", "==============="]
        t["output"] += [f"[INFO] {k}: {v}" for k, v in sorted(args.items())]
        t["output"] += ["===============", "[INFO] Running dataset stage", "[INFO] Loading dataset from: "
                        f"{base}/images"]

    def advance(self, t: dict) -> None:
        code = t["status"]["code"]
        if code in (FAILED, COMPLETED, CANCELED):
            return
        t["step"] += 1
        if t["step"] == 1:
            self._init_task(t)
            return
        if t["step"] == 2:
            t["status"] = {"code": RUNNING}
            t["processingTime"] = 1000
            t["progress"] = 9.0
            self._start_output(t)
            return
        t["processingTime"] += 60000
        if self.fail_task and t["step"] >= 3:
            t["output"] += [f"[INFO] stage line {k}" for k in range(55)]
            t["output"] += ["Traceback (most recent call last):", "  File \"/code/run.py\", line 80",
                            "ValueError: not enough images", "===== Dumping Info for Geeks (developers) ====="]
            t["status"] = {"code": FAILED, "errorMessage": "Cannot process dataset"}
            return
        k = t["step"] - 2
        if k >= self.steps:
            t["progress"] = 100.0
            t["status"] = {"code": COMPLETED}
            t["output"] += ["[INFO] ODX app finished - Mon Oct 20 12:00:00 2026"]
            t["zip"] = self.build_zip(t)
        else:
            t["progress"] = round(9.0 + 81.0 * k / self.steps, 1)
            t["output"] += [f"[INFO] progress step {k}"]

    def info(self, t: dict) -> dict:
        return {"uuid": t["uuid"], "name": t["name"], "dateCreated": 1792500000000,
                "processingTime": t["processingTime"], "status": dict(t["status"]), "options": t["options"],
                "imagesCount": t["imagesCount"], "progress": t["progress"]}

    # ── all.zip ─────────────────────────────────────────────────────────────
    def stats_json(self, t: dict) -> dict:
        n = len([x for x in t.get("images", []) if not x.lower().endswith(".txt")])
        has_gcp = bool(t.get("gcp"))
        details = ([{"id": "GCP01", "coordinates": [-412.3, 118.9, 32.1],
                     "observations": [{"shot_id": t["images"][0], "annotated": [0.512, 0.431],
                                       "reprojected": [0.5121, 0.4309]}],
                     "error": {"x": 0.006, "y": -0.004, "z": 0.012}, "n_inliers": 6, "n_total": 6, "role": "gcp",
                     "sigma": {"x": 0.01, "y": 0.01, "z": 0.1}}] if has_gcp else [])
        blk = {"mean": {"x": 0.004, "y": -0.003, "z": 0.011}, "std": {"x": 0.018, "y": 0.016, "z": 0.042},
               "error": {"x": 0.018, "y": 0.016, "z": 0.043}, "average_error": 0.039, "ce90": 0.03, "le90": 0.07}
        return {
            "processing_statistics": {"steps_times": {"Feature Extraction": 812.4, "Features Matching": 1530.2,
                                                      "Tracks Merging": 95.1, "Reconstruction": 4210.7,
                                                      "Statistics": 61.3, "Total Time": 6709.7},
                                      "date": "20/10/2026 at 14:03:12", "start_date": "20/10/2026 at 10:00:00",
                                      "end_date": "20/10/2026 at 12:00:00", "area": 1843200.5},
            "features_statistics": {"detected_features": {"min": 9120, "max": 21544, "mean": 15873, "median": 15990},
                                    "reconstructed_features": {"min": 610, "max": 5120, "mean": 2890,
                                                               "median": 2911}},
            "reconstruction_statistics": {"components": 1, "has_gps": True, "has_gcp": has_gcp,
                                          "initial_points_count": 2412345, "initial_shots_count": n,
                                          "reconstructed_points_count": 1987654,
                                          "reconstructed_shots_count": max(0, n - 1), "observations_count": 9876543,
                                          "average_track_length": 4.97, "average_track_length_over_two": 5.41,
                                          "histogram_track_length": {"2": 512345, "3": 401234},
                                          "reprojection_error_normalized": 0.00031, "reprojection_error_pixels": 0.92,
                                          "reprojection_error_angular": 0.0003,
                                          "reprojection_histogram_pixels": [[1200.0, 980.0], [0.0, 0.13, 0.26]]},
            "camera_errors": {"v2 sony ilce-6100 6000 4000 brown 0.6889": {
                "initial_values": {"focal": 0.6889}, "optimized_values": {"focal": 0.6871},
                "bias": {"rotation": [0.0, 0.0, 0.0],
                         "translation": [0.0, 0.0, 0.0] if self.bias_identity else [0.012, -0.008, 0.041],
                         "scale": 1.0},
                "relative_difference": {"focal": 0.26}}},
            "rig_errors": {},
            "gps_errors": {"mean": {"x": 0.012, "y": -0.008, "z": 0.041}, "std": {"x": 0.021, "y": 0.019, "z": 0.035},
                           "error": {"x": 0.024, "y": 0.021, "z": 0.054}, "average_error": 0.048, "ce90": 0.041,
                           "le90": 0.088, "average_gps_std": {"x": 0.03, "y": 0.03, "z": 0.05}},
            "gcp_errors": ({**blk, "details": details, "gcp_only": blk, "cp_only": {}, "coordinate_system": "WGS84"}
                           if has_gcp else {}),
            "3d_errors": {"mean": {"x": 0.0, "y": 0.0, "z": 0.0}, "error": {"x": 0.011, "y": 0.012, "z": 0.031},
                          "average_error": 0.028, "ce90": 0.019, "le90": 0.044},
            "opk_errors": {}, "output_coordinate_system": "EPSG:32651",
            "point_cloud_statistics": {"stats": {}, "dense": True},
            "odm_processing_statistics": {"total_time": 30512.4, "total_time_human": "8h:28m:32.0s",
                                          "average_gsd": 2.71}}

    def build_zip(self, t: dict) -> bytes:
        opts = {o["name"].replace("-", "_"): o["value"] for o in t["options"]}
        buf = io.BytesIO()
        if self.empty_result:
            with zipfile.ZipFile(buf, "w") as zf:
                zf.writestr("task_output.txt", "'winrun.bat' is not recognized as an internal or external command,\n"
                                               "Compressing all.zip")
            return buf.getvalue()
        with zipfile.ZipFile(buf, "w") as zf:
            zf.writestr("odm_orthophoto/odm_orthophoto.tif", b"II*\x00fake-tif")
            zf.writestr("orthophoto_tiles/tilemapresource.xml", "<TileMap/>")
            zf.writestr("orthophoto_tiles/18/219834/146802.png", b"\x89PNG")
            zf.writestr("odm_dem/dsm.tif", b"II*\x00fake-dsm")
            if self.stats == "ok":
                zf.writestr("odm_report/stats.json", json.dumps(self.stats_json(t)))
            elif self.stats == "malformed":
                zf.writestr("odm_report/stats.json", '{"reconstruction_statistics": {"initial_shots_count": 3,')
            if self.report_pdf:
                zf.writestr("odm_report/report.pdf", b"%PDF-1.4\n%fake\n")
            if self.shots:
                feats = [{"type": "Feature", "properties": {"filename": n}, "geometry": {"type": "Point",
                                                                                        "coordinates": [121.2, 25.1]}}
                         for n in t.get("images", [])[:-1]]
                zf.writestr("odm_report/shots.geojson", json.dumps({"type": "FeatureCollection", "features": feats}))
            zf.writestr("log.json", json.dumps({
                "engine": "ODX", "version": self.engine_version, "odmVersion": "3.5.6", "success": True,
                "options": {"force_gps": bool(opts.get("force_gps", False)),
                            "max_concurrency": opts.get("max_concurrency", self.cpu),
                            "gps_accuracy": opts.get("gps_accuracy", 3.0), "use_exif": False,
                            "gcp": (f"/var/www/data/{t['uuid']}/gcp/{t['gcp']}" if t.get("gcp") else None),
                            "geo": (f"/var/www/data/{t['uuid']}/gcp/{t['geo']}" if t.get("geo") else None)},
                "stages": [{"name": "dataset", "messages": [{"message": m, "type": "warning"}
                                                             for m in self.log_warnings]},
                           {"name": "odm_orthophoto", "messages": [
                    {"message": "Maximum resolution set to 2.44 (2.71 cm / pixel, requested resolution was 2.0)",
                     "type": "warning"}]}]}))
            if self.images_json:
                zf.writestr("images.json", json.dumps([{"filename": n, "gps_xy_stddev": 0.03, "gps_z_stddev": 0.05}
                                                       for n in t.get("images", [])]))
            zf.writestr("task_output.txt", "\n".join(t["output"]))
        return buf.getvalue()


class _Handler(BaseHTTPRequestHandler):
    fake: FakeNodeODX = None
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):                              # noqa: D401 - 安靜
        pass

    def finish(self):
        try:
            super().finish()
        except OSError:
            pass

    # ── 小工具 ──────────────────────────────────────────────────────────────
    def _send(self, obj, status=200, ctype="application/json", body: bytes = None):
        data = body if body is not None else json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        if ctype == "application/zip":
            self.send_header("Content-Disposition", "attachment; filename=all.zip")
        self.end_headers()
        self.wfile.write(data)

    def _cut(self):
        """切斷連線、不回應（模擬節點閒置逾時或網路斷線）。"""
        self.close_connection = True
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.connection.close()
        raise _Abort()

    def _body_iter(self):
        n = int(self.headers.get("Content-Length") or 0)
        while n > 0:
            chunk = self.rfile.read(min(n, 256 * 1024))
            if not chunk:
                return
            n -= len(chunk)
            yield chunk

    def _safe_body_iter(self):
        """同 _body_iter，但客戶端斷線（讀到 EOF 或 socket 錯誤）就靜靜結束——由呼叫端看主體完不完整。"""
        try:
            yield from self._body_iter()
        except OSError:
            return

    def _drain(self):
        for _ in self._body_iter():
            pass

    def _boundary(self):
        ct = self.headers.get("Content-Type") or ""
        if not ct.lower().startswith("multipart/form-data"):
            return None
        m = re.search(r"boundary=([^;]+)", ct)
        return m.group(1).strip().strip('"').encode("ascii") if m else None

    def _route(self):
        u = urlsplit(self.path)
        q = {k: v[-1] for k, v in parse_qs(u.query).items()}
        return u.path, q

    def _auth_ok(self, q) -> bool:
        tok = self.fake.token
        return tok is None or q.get("token") == tok

    def _record(self, path, q, **kw):
        with self.fake.lock:
            self.fake.log.append({"method": self.command, "path": path, "query": dict(q), **kw})

    def _maybe_drop_any(self):
        """``drop_any`` 故障：節點短暫連不上——這個請求不處理、直接斷線。"""
        f = self.fake
        with f.lock:
            hit = f.faults["drop_any"] > 0
            if hit:
                f.faults["drop_any"] -= 1
        if hit:
            path, q = self._route()
            self._record(path, q, result="dropped (drop_any)")
            self._cut()

    # ── GET ─────────────────────────────────────────────────────────────────
    def do_GET(self):
        try:
            self._get()
        except _Abort:
            pass

    def _get(self):
        self._maybe_drop_any()
        path, q = self._route()
        self._record(path, q)
        f = self.fake
        if not self._auth_ok(q):
            return self._send({"error": "Invalid authentication token: token does not match."})
        if path == "/info":
            return self._send({"version": "2.4.0", "taskQueueCount": 0, "totalMemory": 128 * 2**30,
                               "availableMemory": 100 * 2**30, "cpuCores": f.cpu, "maxImages": f.max_images,
                               "maxParallelTasks": 1, "engineVersion": f.engine_version, "engine": "odx"})
        if path == "/options":
            with f.lock:
                bad = f.faults["options_502"] > 0
                if bad:
                    f.faults["options_502"] -= 1
            if bad:
                return self._send(None, 502, "text/html", b"<html>502 Bad Gateway</html>")
            return self._send(f.options)
        m = re.match(r"^/task/([^/]+)/(info|output|download/(.+))$", path)
        if not m:
            return self._send(None, 404, "text/html", b"<html>Cannot GET</html>")
        uid = m.group(1)
        with f.lock:
            t = f.tasks.get(uid)
            if t is None:
                return self._send({"error": f"{uid} not found"})
            if m.group(2) == "info":
                if f.faults["info_drop"] > 0:
                    f.faults["info_drop"] -= 1
                    self._cut()
                info = f.info(t)                            # 先回目前狀態，下一次 info 才看到下一步
                f.advance(t)
                return self._send(info)
            if m.group(2) == "output":
                return self._send(t["output"][int(q.get("line", 0)):])
            if m.group(3) != "all.zip":
                return self._send({"error": "Invalid asset"})
            if t["status"]["code"] != COMPLETED or t.get("zip") is None:
                return self._send({"error": "Asset not ready"})
            return self._send(None, 200, "application/zip", t["zip"])

    # ── POST ────────────────────────────────────────────────────────────────
    def do_POST(self):
        try:
            self._post()
        except _Abort:
            pass

    def _post(self):
        self._maybe_drop_any()
        path, q = self._route()
        f = self.fake
        if not self._auth_ok(q):
            self._drain()
            self._record(path, q, auth="rejected")
            return self._send({"error": "Invalid authentication token: token does not match."})
        if path == "/task/new/init":
            return self._init(path, q)
        m = re.match(r"^/task/new/(upload|commit)/([^/]+)$", path)
        if m and m.group(1) == "upload":
            return self._upload(path, q, m.group(2))
        if m and m.group(1) == "commit":
            self._drain()
            with f.lock:
                fault = next((k for k in ("commit_drop", "commit_lose_response") if f.faults[k] > 0), None)
                if fault:
                    f.faults[fault] -= 1
            self._record(path, q, fault=fault)
            if fault == "commit_drop":
                self._cut()                                 # 沒處理就斷線：commit 沒送達
            if fault == "commit_lose_response":
                self._commit(m.group(2), respond=False)     # 處理好了、回應遺失
                with f.lock:
                    f.faults["drop_any"] += f.after_commit_drop
                self._cut()
            return self._commit(m.group(2))
        if path in ("/task/cancel", "/task/remove"):
            raw = b"".join(self._body_iter()).decode("utf-8")
            self._record(path, q, body=raw)
            ct = (self.headers.get("Content-Type") or "").lower()
            uid = (json.loads(raw).get("uuid") if "json" in ct else parse_qs(raw).get("uuid", [None])[-1])
            with f.lock:
                t = f.tasks.get(uid)
                if t is None:
                    return self._send({"success": False, "error": f"{uid} not found"})
                if path == "/task/cancel":
                    if t["status"]["code"] not in (COMPLETED, FAILED):
                        t["status"] = {"code": CANCELED}
                        t["processingTime"] = -1
                else:
                    f.tasks.pop(uid)
                    shutil.rmtree(f.root / "data" / uid, ignore_errors=True)
            return self._send({"success": True})
        self._drain()
        return self._send(None, 404, "text/html", b"<html>Cannot POST</html>")

    def _read_fields(self):
        """multipart 的文字欄位；遇到檔案欄 ⇒ 'Unexpected field'（init 不收檔案，N storage.js:169-174）。"""
        b = self._boundary()
        if b is None:
            self._drain()
            return {}, None                                 # N storage.js:41：非 multipart 什麼都不解析
        mp = MultipartReader(b)
        fields, cur, err = {}, None, None
        for chunk in self._body_iter():
            for ev in mp.feed(chunk):
                if ev[0] == "part":
                    disp = dict(_DISP_RE.findall(ev[1].get("content-disposition", "")))
                    if "filename" in disp:
                        err = "Unexpected field"
                    cur = [disp.get("name"), b""]
                elif ev[0] == "data" and cur is not None:
                    cur[1] += ev[1]
                elif ev[0] == "end" and cur is not None:
                    fields[cur[0]] = cur[1].decode("utf-8")
                    cur = None
        return fields, err

    def _init(self, path, q):
        f = self.fake
        fields, err = self._read_fields()
        self._record(path, q, fields=sorted(fields), set_uuid=self.headers.get("set-uuid"))
        if err:
            return self._send({"error": err})
        uid = self.headers.get("set-uuid")
        if uid is not None and (not _UUID_RE.match(uid) or uid in f.tasks):
            return self._send({"error": f"Invalid set-uuid: {uid}"})
        if uid is None:
            import uuid as _u
            uid = str(_u.uuid4())
        try:
            opts = json.loads(fields.get("options") or "[]")
        except ValueError:
            return self._send({"error": "Invalid options"})
        for o in opts:
            if o.get("name") == "orthophoto-resolution" and float(o.get("value")) <= 0:
                return self._send({"error": json.dumps([{"name": "orthophoto-resolution",
                                                         "error": f"Invalid value {o.get('value')} (out of range)"}])})
        d = f.root / "tmp" / uid
        d.mkdir(parents=True, exist_ok=True)
        (d / "body.json").write_text(json.dumps({"name": fields.get("name", ""), "options": opts,
                                                 "skipPostProcessing": fields.get("skipPostProcessing")}),
                                     encoding="utf-8")
        return self._send({"uuid": uid})

    def _upload(self, path, q, uid):
        f = self.fake
        d = f.root / "tmp" / uid
        if not ((d / "body.json").exists() or (d / "_body.json").exists()):
            self._drain()
            self._record(path, q, files=[], result="invalid uuid")
            return self._send({"error": "Invalid uuid (not found)"})
        if (d / "body.json").exists():
            (d / "body.json").replace(d / "_body.json")   # N taskNew.js:98-101
        b = self._boundary()
        if b is None:                                       # 探測（空主體、非 multipart）
            with f.lock:
                drop = f.faults["probe_drop"] > 0
                if drop:
                    f.faults["probe_drop"] -= 1
            if drop:
                self._drain()
                self._record(path, q, files=[], result="probe dropped")
                self._cut()
        with f.lock:
            fault = (next((k for k in ("upload_drop", "upload_500", "upload_lose_response", "upload_502_after_store")
                           if f.faults[k] > 0), None) if b is not None else None)
            if fault:
                f.faults[fault] -= 1
        total = int(self.headers.get("Content-Length") or 0)
        written, names, nbytes = [], [], 0
        mp = MultipartReader(b) if b else None
        fh, err = None, None
        try:
            for chunk in self._safe_body_iter():
                nbytes += len(chunk)
                if fault == "upload_drop" and nbytes > total // 2:
                    raise _Abort()
                if mp is None:
                    continue
                for ev in mp.feed(chunk):
                    if ev[0] == "part":
                        disp = dict(_DISP_RE.findall(ev[1].get("content-disposition", "")))
                        if "filename" not in disp:
                            continue
                        if disp.get("name") != "images":
                            err = "Unexpected field"
                            continue
                        name = disp["filename"]
                        stem, dot, ext = name.rpartition(".")
                        target = d / name
                        while True:                                         # 'wx'：存在就加底線（N storage.js:200-208）
                            try:
                                fh = open(target, "xb")
                                break
                            except FileExistsError:
                                stem = stem + "_" if dot else stem
                                name = f"{stem}.{ext}" if dot else name + "_"
                                target = d / name
                        written.append(target)
                        names.append(name)
                    elif ev[0] == "data" and fh is not None:
                        fh.write(ev[1])
                    elif ev[0] == "end" and fh is not None:
                        fh.close()
                        fh = None
        except _Abort:
            if fh is not None:
                fh.close()
            for p in written:
                p.unlink(missing_ok=True)                    # 中斷的請求不留檔（N storage.js:126,145）
            self._record(path, q, files=names, result="dropped", bytes=nbytes)
            self._cut()
        if fh is not None:
            fh.close()
        if mp is not None and mp.state != "done":
            # 客戶端中途斷線（主體不完整）：節點刪掉這個請求寫下的所有檔案（N storage.js:126,145,215-220）
            for p in written:
                p.unlink(missing_ok=True)
            self._record(path, q, files=names, result="client aborted", bytes=nbytes)
            self._cut()
        if err or fault == "upload_500" or not names:
            for p in written:
                p.unlink(missing_ok=True)
            self._record(path, q, files=names, result=err or fault or "no files", bytes=nbytes)
            if fault == "upload_500":
                return self._send({"error": "Upload dropped (test mode)"}, status=500)
            if err:
                return self._send({"error": err})
            return self._send({"error": "Need at least 1 file.", "noRetry": True})
        self._record(path, q, files=names, result="ok", bytes=nbytes)
        if fault == "upload_lose_response":
            self._cut()                                     # 檔案已存好、回應遺失（模稜兩可）
        if fault == "upload_502_after_store":                # 檔案已存好、前面的代理回 502
            return self._send(None, 502, "text/html", b"<html>502 Bad Gateway</html>")
        return self._send({"success": True})

    def _commit(self, uid, respond=True):
        f = self.fake
        d = f.root / "tmp" / uid
        body = d / ("_body.json" if (d / "_body.json").exists() else "body.json")
        if not body.exists():
            return self._send({"error": "Invalid uuid (not found)"}) if respond else None
        b = json.loads(body.read_text(encoding="utf-8"))
        body.unlink()
        files = sorted(p.name for p in d.iterdir())
        if not files:
            return self._send({"error": "Need at least 1 file."}) if respond else None
        skip_pp = b.get("skipPostProcessing") == "true"
        with f.lock:
            f.tasks[uid] = {"uuid": uid, "name": b.get("name") or "Task of 2026-10-20T10:00:00.000Z",
                            "status": {"code": RUNNING}, "processingTime": -1, "progress": 0.0,
                            "imagesCount": len(files), "options": f._filtered_options(b.get("options") or [], skip_pp),
                            "output": [], "step": 0}
        return self._send({"uuid": uid}) if respond else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--token")
    ap.add_argument("--steps", type=int, default=3)
    a = ap.parse_args(argv)
    node = FakeNodeODX(a.root, token=a.token, steps=a.steps).start()
    print(f"PORT {node.server.server_address[1]}", flush=True)
    try:
        sys.stdin.read()
    finally:
        node.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
