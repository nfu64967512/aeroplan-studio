"""最小的 NodeODM／NodeODX REST 客戶端（以獨立服務呼叫，AGPL 義務留在引擎那邊；不是法律意見）。

端點（OpenDroneMap/NodeODM docs/index.adoc，2026-10-14 取得）：
* ``POST /task/new``：multipart。``images`` 欄可重複，影像以外也可附 geo.txt、GCP（*.txt）；
  ``options`` 是 JSON 字串化的陣列 ``[{"name": ..., "value": ...}, ...]``；選填 ``name``。
  回傳 ``{"uuid": ...}``。大型任務官方建議改走 /task/new/init → upload → commit（本模組未實作）。
* ``GET /task/{uuid}/info``：``status.code`` 10 QUEUED、20 RUNNING、30 FAILED、40 COMPLETED、50 CANCELED。
* ``GET /task/{uuid}/download/all.zip``：所有成果。
* 驗證時附 ``?token=``。NodeODM 出錯常常是 HTTP 200 加 ``{"error": ...}``（libs/taskNew.js），兩種都要檢查。

NodeODM 依檔名分類附檔（libs/Task.js:86-91）：剛好叫 ``geo.txt`` 才是地理定位檔，
``image_groups.txt`` 是分組檔，**其他任何 .txt 都當 GCP** ⇒ 上傳清單只收影像、geo.txt 與明確給的 GCP。

ODM 參數名稱已對照 OpenDroneMap/ODM opendm/config.py：rolling-shutter（旗標）、
rolling-shutter-readout（ms；0 = 查內建資料庫，查不到用預設 30 ms ⇒ 這裡不允許 0）、
orthophoto-resolution（cm/px）、dsm、tiles、gps-accuracy（m）、split（平均每子模型張數）、
split-overlap（m）。
"""

from __future__ import annotations

import contextlib
import json
import time
from pathlib import Path
from typing import Optional, Sequence

STATUS_QUEUED, STATUS_RUNNING, STATUS_FAILED, STATUS_COMPLETED, STATUS_CANCELED = 10, 20, 30, 40, 50
_IMAGE_SUFFIXES = (".jpg", ".jpeg", ".tif", ".tiff", ".png", ".dng")


class OdmError(RuntimeError):
    pass


def rolling_shutter_readout(readouts_ms: Sequence[Optional[float]], *, tol_ms: float = 1.0) -> float:
    """一個 ODM 任務只能給一個讀出時間：各相機的實測值必須一致（差 ≤ tol_ms），否則請分任務。"""
    vals = list(readouts_ms)
    if not vals or any(v is None for v in vals):
        raise ValueError("有相機沒有實測讀出時間（readout_ms=null）：不能開滾動快門校正，"
                         "否則 ODM 會靜默用資料庫或預設 30 ms")
    if any(v <= 0 for v in vals):
        raise ValueError(f"讀出時間必須 > 0 ms（0 在 ODM 代表查資料庫），收到 {vals}")
    if max(vals) - min(vals) > tol_ms:
        raise ValueError(f"各相機讀出時間 {vals} ms 相差超過 {tol_ms} ms：ODM 一個任務只吃一個值，請依相機分任務")
    return float(sum(vals) / len(vals))


def build_options(*, orthophoto_resolution_cm: float, dsm: bool = True, tiles: bool = True,
                  gps_accuracy_m: Optional[float] = None, rolling_shutter_readout_ms: Optional[float] = None,
                  split: Optional[int] = None, split_overlap_m: Optional[float] = None,
                  extra: Optional[dict] = None) -> list:
    """組 NodeODM 的 options 陣列。``rolling_shutter_readout_ms`` 給值就同時開 --rolling-shutter。"""
    if orthophoto_resolution_cm <= 0:
        raise ValueError("orthophoto_resolution_cm 必須 > 0")
    opts = [{"name": "orthophoto-resolution", "value": float(orthophoto_resolution_cm)}]
    if dsm:
        opts.append({"name": "dsm", "value": True})
    if tiles:
        opts.append({"name": "tiles", "value": True})
    if gps_accuracy_m is not None:
        if gps_accuracy_m <= 0:
            raise ValueError("gps_accuracy_m 必須 > 0")
        opts.append({"name": "gps-accuracy", "value": float(gps_accuracy_m)})
    if rolling_shutter_readout_ms is not None:
        if rolling_shutter_readout_ms <= 0:
            raise ValueError("rolling_shutter_readout_ms 必須 > 0（0 在 ODM 代表改用資料庫/預設 30 ms）")
        opts.append({"name": "rolling-shutter", "value": True})
        opts.append({"name": "rolling-shutter-readout", "value": float(rolling_shutter_readout_ms)})
    if split is not None:
        if split <= 0:
            raise ValueError("split 必須 > 0")
        opts.append({"name": "split", "value": int(split)})
        if split_overlap_m is not None:
            opts.append({"name": "split-overlap", "value": float(split_overlap_m)})
    elif split_overlap_m is not None:
        raise ValueError("split_overlap_m 只在有 split 時有意義")
    for k, v in (extra or {}).items():
        if any(o["name"] == k for o in opts):
            raise ValueError(f"extra 的 {k!r} 與具名參數重複")
        opts.append({"name": k, "value": v})
    return opts


class NodeOdmClient:
    def __init__(self, base_url: str = "http://127.0.0.1:3000", *, token: Optional[str] = None,
                 session=None, timeout_s: float = 300.0):
        if session is None:
            import requests
            session = requests.Session()
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.session = session
        self.timeout_s = timeout_s

    def _params(self):
        return {"token": self.token} if self.token else None

    @staticmethod
    def _json(resp, what):
        try:
            data = resp.json()
        except ValueError:
            raise OdmError(f"{what}: HTTP {resp.status_code}，回應不是 JSON") from None
        if resp.status_code != 200 or (isinstance(data, dict) and data.get("error")):
            raise OdmError(f"{what}: HTTP {resp.status_code} {data.get('error') if isinstance(data, dict) else data}")
        return data

    def new_task(self, images, *, options: list, geo_txt=None, gcp_txt=None, name: Optional[str] = None) -> str:
        images = [Path(p) for p in images]
        if not images:
            raise ValueError("沒有影像")
        bad = [p.name for p in images if p.suffix.lower() not in _IMAGE_SUFFIXES]
        if bad:
            raise ValueError(f"images 只能放影像（其他 .txt 會被 NodeODM 當成 GCP）：{bad[:5]}")
        extras = []
        if geo_txt is not None:
            if Path(geo_txt).name.lower() != "geo.txt":
                raise ValueError(f"地理定位檔必須叫 geo.txt（NodeODM 依檔名判斷），收到 {Path(geo_txt).name}")
            extras.append(Path(geo_txt))
        if gcp_txt is not None:
            gp = Path(gcp_txt)
            if gp.suffix.lower() != ".txt" or gp.name.lower() in ("geo.txt", "image_groups.txt"):
                raise ValueError(f"GCP 檔必須是 .txt 且不能叫 geo.txt／image_groups.txt：{gp.name}")
            extras.append(gp)
        names = [p.name for p in images + extras]
        if len(set(n.lower() for n in names)) != len(names):
            raise ValueError("上傳檔名重複（NodeODM 以檔名存放）")
        data = {"options": json.dumps(options)}
        if name:
            data["name"] = name
        with contextlib.ExitStack() as stack:
            files = [("images", (p.name, stack.enter_context(open(p, "rb")))) for p in images + extras]
            resp = self.session.post(f"{self.base_url}/task/new", params=self._params(), data=data,
                                     files=files, timeout=self.timeout_s)
        d = self._json(resp, "POST /task/new")
        if not d.get("uuid"):
            raise OdmError(f"POST /task/new 沒有回傳 uuid：{d}")
        return d["uuid"]

    def task_info(self, uuid: str) -> dict:
        resp = self.session.get(f"{self.base_url}/task/{uuid}/info", params=self._params(), timeout=self.timeout_s)
        return self._json(resp, f"GET /task/{uuid}/info")

    def wait(self, uuid: str, *, poll_s: float = 10.0, timeout_s: float = 24 * 3600.0, sleep=time.sleep,
             clock=time.monotonic) -> dict:
        """輪詢到結束；FAILED／CANCELED 丟 OdmError，COMPLETED 回傳最後的 info。"""
        t0 = clock()
        while True:
            info = self.task_info(uuid)
            code = (info.get("status") or {}).get("code")
            if code == STATUS_COMPLETED:
                return info
            if code in (STATUS_FAILED, STATUS_CANCELED):
                raise OdmError(f"任務 {uuid} 狀態 {code}：{info.get('status')}")
            if clock() - t0 > timeout_s:
                raise OdmError(f"任務 {uuid} 等待逾時（{timeout_s} s），最後狀態 {code}")
            sleep(poll_s)

    def download_all(self, uuid: str, dest) -> Path:
        dest = Path(dest)
        dest.parent.mkdir(parents=True, exist_ok=True)
        resp = self.session.get(f"{self.base_url}/task/{uuid}/download/all.zip", params=self._params(),
                                timeout=self.timeout_s, stream=True)
        ctype = (resp.headers or {}).get("Content-Type", "")
        if resp.status_code != 200 or "json" in ctype:
            self._json(resp, f"GET /task/{uuid}/download/all.zip")
            raise OdmError(f"下載 {uuid} 失敗：HTTP {resp.status_code}")
        tmp = dest.with_suffix(dest.suffix + ".part")
        with open(tmp, "wb") as fh:
            for chunk in resp.iter_content(chunk_size=1 << 20):
                if chunk:
                    fh.write(chunk)
        tmp.replace(dest)
        return dest
