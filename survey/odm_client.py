"""NodeODX／NodeODM REST 客戶端（以獨立服務呼叫，AGPL 義務留在引擎那邊；不是法律意見）。

依據：NodeODX v2.4.0（下稱 N）與 ODX v3.8.4（下稱 X）原始碼整理的規格 R1（2026-10-20）。
**沒有對真的節點跑過**；行為以原始碼行號為準，測試用 tests/fake_nodeodx.py 依同一份規格模擬。

傳輸規則（R1 §1）
=================
* 權杖只走 query string ``?token=``（N libs/auth/TokenAuthBase.js:32-41）⇒ 每個 URL 都帶著它；requests／urllib3
  的例外訊息會把 URL 連 query 一起印出（"Max retries exceeded with url: /task/...?token=XXX"）⇒ 本模組所有網路
  呼叫的例外都先遮蔽權杖（:func:`redact`）再丟，**而且在 except 區塊外面丟**（不留原例外鏈）。
* 節點位址不能帶查詢字串／帳密（:func:`sanitize_node_url`）：任務紀錄只存去掉權杖的位址。
* 幾乎所有錯誤都是 HTTP 200 加 ``{"error": ...}``（N index.js:881-884）；cancel／remove／upload 還要
  ``success === true``（N index.js:550-555）。

分段上傳（R1 §2；取代舊版一次把所有影像讀進記憶體的 POST /task/new）
==================================================================
1. ``POST /task/new/init``：**必須 multipart**——非 multipart 的欄位會被靜默丟掉、任務改用節點預設跑
   （N libs/storage.js:41、taskNew.js:149）。本機先產生 UUID 放 ``set-uuid`` 標頭：未提交前 init 可以重送
   （同一個 uuid 只重寫 body.json，已上傳的檔案保留，N taskNew.js:53-70,170-178）。
2. ``POST /task/new/upload/{uuid}``：檔案欄名只能是 ``images``（其他欄名整個請求失敗，N storage.js:169-174）；
   每批檔數與位元組都有上限（:data:`UPLOAD_BATCH_MAX_FILES`／:data:`UPLOAD_BATCH_MAX_BYTES`），**從磁碟串流**
   （:class:`MultipartStream`；記憶體裡只有一個讀取區塊）。
   * 失敗或中斷的請求，伺服器會把該請求寫下的檔案全刪（N storage.js:126,145,215-220）⇒ 重送安全。
   * 「主體已送完、回應遺失」是**模稜兩可**：成功了但回應丟了的話，重送同名檔會被改名成 ``IMG_0001_.JPG``
     （開檔用 'wx'，EEXIST 就在副檔名前加底線，N storage.js:35-38,200-208）⇒ 多一張只有 EXIF GPS 的照片。
     影像批次允許重送但記錄 ``ambiguous_retries``，提交後由 survey.pipeline 用 imagesCount 核對；
     geo.txt／GCP 那批**不允許**（``geo_.txt`` 會被當成 GCP 候選）。
   * 節點 60 s 收不到資料就切斷並刪掉該請求的檔案（N storage.js:87-90,160）：串流本身不會停頓。
3. ``POST /task/new/commit/{uuid}``：只在所有上傳請求都回應後才送（commit 會先中止並刪掉還在傳的請求，
   N taskNew.js:116）。回 uuid 後節點才在背景搬檔、檢查 maxImages——這些錯誤**不在 commit 回應裡**，
   而是讓任務變成 30 FAILED（N taskNew.js:386-391）⇒ 提交後一定要輪詢 info。
* 未提交的上傳**沒有刪除 API**：/task/remove、/task/cancel 回 ``"<uuid> not found"``（N TaskManager.js:283-287），
  暫存 tmp/<uuid> 由節點在 cleanupUploadsAfter（預設 2880 分＝48 h）後自己清掉（N TaskManager.js:130-151）。

依檔名分類附檔（N libs/Task.js:85-96,554-565）：剛好叫 ``geo.txt`` 是地理定位檔、``image_groups.txt`` 是分組檔，
**其他任何 .txt 都是 GCP 候選、只取 readdir 順序第一個** ⇒ 上傳清單只收影像、geo.txt 與一個 GCP；
GCP 一律以 :data:`GCP_UPLOAD_NAME` 這個名字上傳（本機檔名可以是任何 .txt）。
節點會**靜默丟掉**它的引擎不認得的參數名稱（N libs/odmInfo.js:290-325）⇒ 送出前用 GET /options 對照、
提交後再用 info.options 對照（見 survey.pipeline）。

ODX 參數名稱（X modules/config.py）：rolling-shutter（旗標）、rolling-shutter-readout（ms；0 = 查內建資料庫，
查不到用預設 ⇒ 這裡不允許 0）、orthophoto-resolution（cm/px）、dsm、tiles、gps-accuracy（m）、split、
split-overlap（m）、force-gps（旗標，:843-849）、max-concurrency（正整數，:309-316）。
"""

from __future__ import annotations

import json
import mimetypes
import re
import shutil
import threading
import time
import uuid as uuid_mod
from pathlib import Path
from typing import Callable, Optional, Sequence
from urllib.parse import urlsplit, urlunsplit

STATUS_QUEUED, STATUS_RUNNING, STATUS_FAILED, STATUS_COMPLETED, STATUS_CANCELED = 10, 20, 30, 40, 50
STATUS_NAMES = {STATUS_QUEUED: "排隊中", STATUS_RUNNING: "執行中", STATUS_FAILED: "失敗",
                STATUS_COMPLETED: "完成", STATUS_CANCELED: "已取消"}
_IMAGE_SUFFIXES = (".jpg", ".jpeg", ".tif", ".tiff", ".png", ".dng")

# 分段上傳的每批上限（兩個都要滿足；單一檔案超過位元組上限就自己一批）。
# 25 張 × 約 10 MB 的 12.3 MP JPEG ≈ 250 MB：2,500 張約 100 批，失敗只重送一批；
# 記憶體用量與批次大小無關（串流），上限只決定重送的粒度與兩次取消檢查之間的最長時間。
UPLOAD_BATCH_MAX_FILES = 25
UPLOAD_BATCH_MAX_BYTES = 256 * 1024 * 1024
# 每批最多試幾次、每次失敗後等幾秒（PyODM 每檔重試 5 次，pyodm/api.py:297-345）
UPLOAD_ATTEMPTS = 5
UPLOAD_BACKOFF_S = (2.0, 4.0, 8.0, 16.0)
GCP_UPLOAD_NAME = "gcp_list.txt"
CONNECT_TIMEOUT_S = 10.0
_PROGRESS_STEP_BYTES = 8 * 1024 * 1024
_MAX_READ = 1 << 20

# set-uuid 必須符合 v1–7 的 UUID 格式（N taskNew.js:53-70）
_UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[1-7][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$", re.I)
_TOKEN_RE = re.compile(r"(?i)(token=)[^&\s'\"<>)\]]*")
# 權杖「原值」只在夠長時才在自由文字（例外訊息）裡逐字替換：短權杖（"odm"、"run"、"2026"）會撞到路徑、
# 階段名、時間戳，把訊息改得面目全非（2026-10-20 審查 T9）。權杖實際出現的地方是 URL 的 ``token=…``，那由
# _TOKEN_RE 處理，與長度無關。任務紀錄（odm/submission.json）**不做**原值替換（見 survey.pipeline._scrub）。
REDACT_RAW_MIN_LEN = 8


class OdmError(RuntimeError):
    """NodeODX 回報錯誤或回應不合規格（訊息已遮蔽權杖）。"""

    def __init__(self, message: str, *, status: Optional[int] = None, remote_error: Optional[str] = None,
                 no_retry: bool = False):
        super().__init__(message)
        self.status = status
        self.remote_error = remote_error
        self.no_retry = no_retry

    @property
    def not_found(self) -> bool:
        """節點不認得這個 uuid（過期、刪掉、節點當機時遺失、或從沒提交，R1 §3）。"""
        e = (self.remote_error or "").lower()
        return "not found" in e


class OdmTransportError(OdmError):
    """連線層失敗（沒有拿到 HTTP 回應）。``ambiguous`` = 上傳主體已全部交給連線、卻沒有回應。

    ``commit_unknown`` = commit 送出後連線失敗，而且事後用 info／探測也**確認不了**節點有沒有收到（兩邊都連不上）
    ⇒ 呼叫端不能當成「沒提交」（任務可能正在節點上跑）。"""

    def __init__(self, message: str, *, ambiguous: bool = False, commit_unknown: bool = False):
        super().__init__(message)
        self.ambiguous = ambiguous
        self.commit_unknown = commit_unknown


class OdmLocalError(OdmError):
    """本機的錯誤（寫檔失敗、磁碟空間不足），不是連線或節點的問題：遠端成果不受影響。"""


class OdmCanceled(OdmError):
    """使用者在兩批之間取消上傳。``batches`` = 已完成的批次報告。"""

    def __init__(self, message: str, *, batches: Optional[list] = None):
        super().__init__(message)
        self.batches = list(batches or [])


def redact(text, token: Optional[str] = None) -> str:
    """遮蔽自由文字（例外訊息、log）裡的權杖：``token=...`` 一律換成 ``token=***``；權杖原值長度
    ≥ :data:`REDACT_RAW_MIN_LEN` 時，任何地方出現的原值也換掉（短權杖不逐字換，否則會改壞路徑與字詞）。"""
    s = _TOKEN_RE.sub(r"\1***", str(text))
    if token and len(str(token)) >= REDACT_RAW_MIN_LEN:
        s = s.replace(str(token), "***")
    return s


def free_bytes(path) -> Optional[int]:
    """``path``（或它最近的既有上層資料夾）所在磁碟的可用位元組；查不到回 None（測試可換掉這支）。"""
    p = Path(path)
    while not p.exists() and p.parent != p:
        p = p.parent
    try:
        return int(shutil.disk_usage(p).free)
    except OSError:
        return None


def sanitize_node_url(url) -> str:
    """節點位址正規化：只收 http(s)://主機[:埠][/路徑]；帶查詢字串、片段或帳密一律拒絕（訊息不回顯查詢字串）。"""
    u = str(url or "").strip()
    try:
        parts = urlsplit(u)
    except ValueError:
        raise ValueError("NodeODX 位址格式錯誤") from None
    shown = urlunsplit((parts.scheme, parts.netloc.rpartition("@")[2], parts.path, "", ""))
    if parts.scheme.lower() not in ("http", "https") or not parts.netloc:
        raise ValueError(f"NodeODX 位址要是 http(s)://主機:埠，收到 {shown!r}")
    if parts.query or parts.fragment:
        raise ValueError("NodeODX 位址不能帶 ?查詢字串 或 #片段：權杖請填在「權杖」欄（只在連線時使用、不寫進任何檔案），"
                         f"位址只填 {shown!r}")
    if "@" in parts.netloc:
        raise ValueError("NodeODX 位址不能含帳號密碼")
    return urlunsplit((parts.scheme.lower(), parts.netloc, parts.path.rstrip("/"), "", ""))


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
                  force_gps: bool = False, max_concurrency: Optional[int] = None,
                  extra: Optional[dict] = None) -> list:
    """組 NodeODX 的 options 陣列。``rolling_shutter_readout_ms`` 給值就同時開 --rolling-shutter。

    ``force_gps=True`` 才送 --force-gps（布林 false 節點根本不會傳給引擎，N odmRunner.js:36-48，所以「不送」就是 false）；
    ``max_concurrency`` 只有給值才送（沒給 ⇒ 引擎預設＝節點的 CPU 數，或節點自己的上限，N odmInfo.js:327-334）。
    新參數一律接在既有參數後面：沒用到時陣列與加入功能前相同。
    """
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
    if not isinstance(force_gps, bool):
        raise ValueError(f"force_gps 必須是 True/False，收到 {force_gps!r}")
    if force_gps:
        opts.append({"name": "force-gps", "value": True})
    if max_concurrency is not None:
        if isinstance(max_concurrency, bool) or not isinstance(max_concurrency, int) or max_concurrency < 1:
            raise ValueError(f"max_concurrency 必須是 ≥ 1 的整數，收到 {max_concurrency!r}")
        opts.append({"name": "max-concurrency", "value": int(max_concurrency)})
    for k, v in (extra or {}).items():
        if any(o["name"] == k for o in opts):
            raise ValueError(f"extra 的 {k!r} 與具名參數重複")
        opts.append({"name": k, "value": v})
    return opts


# ─────────────────────────────────────────────────────────────────────────────
# 上傳清單與分批
# ─────────────────────────────────────────────────────────────────────────────

def _check_upload_name(name: str) -> None:
    # busboy 0.2 對非 ASCII 檔名的解碼沒驗證過（R1 §2.2）：名字一變就對不上 geo.txt，那張照片會靜默改用 EXIF GPS
    if (not name or not name.isascii() or any(ord(c) < 32 for c in name) or any(c in name for c in '"/\\')
            or name in (".", "..")):
        raise ValueError(f"上傳檔名必須是單純的 ASCII 檔名（NodeODX 對其他檔名的處理未驗證，名字一變就對不上 "
                         f"geo.txt）：{name!r}")


def classify_uploads(images, *, geo_txt=None, gcp_txt=None) -> tuple:
    """上傳清單：回傳 ``(影像, 附檔)``，各是 ``[(本機路徑, 上傳檔名), ...]``。

    規則（N libs/Task.js:85-96）：images 只能放影像（任何 .txt 都會被當 GCP 候選）；地理定位檔必須叫 geo.txt；
    GCP 必須是 .txt 且不能叫 geo.txt／image_groups.txt，一律以 :data:`GCP_UPLOAD_NAME` 上傳；
    檔名（不分大小寫）不能重複、必須是 ASCII。
    """
    images = [Path(p) for p in images]
    if not images:
        raise ValueError("沒有影像")
    bad = [p.name for p in images if p.suffix.lower() not in _IMAGE_SUFFIXES]
    if bad:
        raise ValueError(f"images 只能放影像（其他 .txt 會被 NodeODM 當成 GCP）：{bad[:5]}")
    aux = []
    if geo_txt is not None:
        if Path(geo_txt).name.lower() != "geo.txt":
            raise ValueError(f"地理定位檔必須叫 geo.txt（NodeODM 依檔名判斷），收到 {Path(geo_txt).name}")
        aux.append((Path(geo_txt), "geo.txt"))
    if gcp_txt is not None:
        gp = Path(gcp_txt)
        if gp.suffix.lower() != ".txt" or gp.name.lower() in ("geo.txt", "image_groups.txt"):
            raise ValueError(f"GCP 檔必須是 .txt 且不能叫 geo.txt／image_groups.txt：{gp.name}")
        aux.append((gp, GCP_UPLOAD_NAME))
    imgs = [(p, p.name) for p in images]
    names = [n for _, n in imgs + aux]
    if len(set(n.lower() for n in names)) != len(names):
        raise ValueError("上傳檔名重複（NodeODM 以檔名存放）")
    for n in names:
        _check_upload_name(n)
    return imgs, aux


def plan_batches(items, *, max_files: int = UPLOAD_BATCH_MAX_FILES, max_bytes: int = UPLOAD_BATCH_MAX_BYTES) -> list:
    """依序切批：``items`` 是 ``[(路徑, 上傳檔名, 位元組), ...]``；每批 ≤ max_files 個且檔案位元組 ≤ max_bytes。

    單一檔案超過 max_bytes 就自己一批（不切檔：NodeODX 沒有續傳）。multipart 標頭的幾百位元組不計入上限。
    """
    if int(max_files) < 1 or int(max_bytes) < 1:
        raise ValueError("每批上限必須 ≥ 1")
    batches, cur, cur_b = [], [], 0
    for it in items:
        size = int(it[2])
        if cur and (len(cur) >= max_files or cur_b + size > max_bytes):
            batches.append(cur)
            cur, cur_b = [], 0
        cur.append(it)
        cur_b += size
    if cur:
        batches.append(cur)
    return batches


class MultipartStream:
    """multipart/form-data 主體，檔案邊讀邊送：記憶體裡只有一個讀取區塊（不像 requests 的 ``files=`` 整包組在記憶體）。

    requests 看到有 ``__len__`` 與 ``__iter__`` 的物件就當串流主體、送 Content-Length（requests.models.prepare_body）；
    urllib3 對有 ``read`` 的主體每次讀 blocksize 位元組（urllib3.util.request.body_to_chunks）。
    ``bytes_read`` = 已交給連線的位元組；等於 ``len()`` 表示整個主體（含結尾分隔線）都送出去了。
    檔案在上傳中途變大或變小一律丟 :class:`OdmError`（不默默送出截斷的檔案）。
    ``should_abort()`` 每讀一個區塊檢查一次，為真就丟 :class:`OdmCanceled`：請求中斷，節點會刪掉這個請求寫下的
    所有檔案（N storage.js:126,145），所以中途放棄一批是安全的（關主視窗時不必等一整批傳完）。
    """

    def __init__(self, fields=(), files=(), *, boundary: Optional[str] = None,
                 on_progress: Optional[Callable[[int, Optional[str]], None]] = None,
                 should_abort: Optional[Callable[[], bool]] = None):
        self.boundary = boundary or ("aeroplan" + uuid_mod.uuid4().hex)
        b = self.boundary.encode("ascii")
        segs = []
        for name, value in fields:
            head = (b"--" + b + b"\r\nContent-Disposition: form-data; name=\"" + str(name).encode("ascii")
                    + b"\"\r\n\r\n")
            segs.append(("bytes", head + str(value).encode("utf-8") + b"\r\n"))
        for field, filename, path, size in files:
            _check_upload_name(filename)
            ctype = mimetypes.guess_type(filename)[0] or "application/octet-stream"
            head = (b"--" + b + b"\r\nContent-Disposition: form-data; name=\"" + str(field).encode("ascii")
                    + b"\"; filename=\"" + filename.encode("ascii") + b"\"\r\nContent-Type: "
                    + ctype.encode("ascii") + b"\r\n\r\n")
            segs.append(("bytes", head))
            segs.append(("file", Path(path), int(size), filename))
            segs.append(("bytes", b"\r\n"))
        segs.append(("bytes", b"--" + b + b"--\r\n"))
        self._segs = segs
        self._len = sum(len(s[1]) if s[0] == "bytes" else s[2] for s in segs)
        self._i = 0
        self._off = 0
        self._fh = None
        self._on_progress = on_progress
        self._should_abort = should_abort
        self.bytes_read = 0

    @property
    def content_type(self) -> str:
        return f"multipart/form-data; boundary={self.boundary}"

    def __len__(self) -> int:
        return self._len

    def __iter__(self):
        while True:
            chunk = self.read(64 * 1024)
            if not chunk:
                return
            yield chunk

    @property
    def complete(self) -> bool:
        return self.bytes_read >= self._len

    def _advance(self) -> None:
        self._i += 1
        self._off = 0

    def read(self, n: int = -1) -> bytes:
        # 連線層一定給 blocksize；沒給時也最多 1 MiB（不把整批讀進記憶體——串流的重點）
        n = _MAX_READ if n is None or n < 0 else min(int(n), _MAX_READ)
        if self._should_abort is not None and self._i < len(self._segs) and self._should_abort():
            self.close()
            raise OdmCanceled("上傳途中取消（這個請求中斷，節點會丟掉它寫下的檔案）")
        out = bytearray()
        while len(out) < n and self._i < len(self._segs):
            seg = self._segs[self._i]
            if seg[0] == "bytes":
                data = seg[1][self._off:self._off + (n - len(out))]
                self._off += len(data)
                out += data
                if self._off >= len(seg[1]):
                    self._advance()
                continue
            _, path, size, filename = seg
            if self._fh is None:
                self._fh = open(path, "rb")
            want = min(n - len(out), size - self._off)
            data = self._fh.read(want) if want > 0 else b""
            if len(data) < want:
                self.close()
                raise OdmError(f"{filename} 在上傳途中變短（預期 {size} B，只讀到 {self._off + len(data)} B）")
            self._off += len(data)
            out += data
            done = self._off >= size
            if done:
                extra = self._fh.read(1)
                self.close()
                if extra:
                    raise OdmError(f"{filename} 在上傳途中變大（預期 {size} B）：不送出截斷的檔案")
                self._advance()
            if self._on_progress is not None and (data or done):
                self._on_progress(len(data), filename if done else None)
        self.bytes_read += len(out)
        return bytes(out)

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            finally:
                self._fh = None


# ─────────────────────────────────────────────────────────────────────────────
# 客戶端
# ─────────────────────────────────────────────────────────────────────────────

class _RedactFilter:
    """掛在 urllib3 的 logger 上：它的警告（例如重試）會印出帶 ?token= 的 URL。"""

    def filter(self, record) -> bool:
        try:
            msg = record.getMessage()
        except Exception:                                   # noqa: BLE001
            return True
        if "token=" in msg.lower():
            record.msg, record.args = redact(msg), ()
        return True


_FILTER_LOCK = threading.Lock()
_FILTER_INSTALLED = [False]


def _install_log_filter() -> None:
    import logging
    with _FILTER_LOCK:
        if _FILTER_INSTALLED[0]:
            return
        for name in ("urllib3.connectionpool", "urllib3.util.retry", "urllib3.connection"):
            logging.getLogger(name).addFilter(_RedactFilter())
        _FILTER_INSTALLED[0] = True


class NodeOdmClient:
    """NodeODX 客戶端。``token`` 只存在這個物件裡（送 query string），不寫進任何檔案、訊息或例外。"""

    def __init__(self, base_url: str = "http://127.0.0.1:3000", *, token: Optional[str] = None,
                 session=None, timeout_s: float = 300.0, connect_timeout_s: float = CONNECT_TIMEOUT_S,
                 api_timeout_s: float = 30.0):
        """``timeout_s`` = 上傳／下載的讀取逾時；``api_timeout_s`` = 其他小 JSON 端點（info、output、cancel…）的讀取逾時，
        短一點讓取消與關主視窗不必等一個卡住的輪詢。"""
        if session is None:
            import requests
            session = requests.Session()
        self.base_url = sanitize_node_url(base_url)
        self.token = (str(token).strip() or None) if token is not None else None
        self.session = session
        self.timeout_s = float(timeout_s)
        self.api_timeout_s = min(float(api_timeout_s), self.timeout_s)
        self.connect_timeout_s = float(connect_timeout_s)
        if self.token:
            _install_log_filter()

    @property
    def token_used(self) -> bool:
        return bool(self.token)

    def redact(self, text) -> str:
        return redact(text, self.token)

    def _params(self, extra: Optional[dict] = None) -> Optional[dict]:
        p = dict(extra or {})
        if self.token:
            p["token"] = self.token
        return p or None

    def _timeout(self, read_s: Optional[float] = None):
        return (self.connect_timeout_s, float(read_s if read_s is not None else self.timeout_s))

    def _send(self, method: str, path: str, what: str, *, params: Optional[dict] = None, body=None, **kw):
        """所有網路呼叫的唯一出口：連線層例外 ⇒ :class:`OdmTransportError`（遮蔽權杖、不帶原例外鏈）。"""
        url = self.base_url + path
        kw.setdefault("timeout", self._timeout(self.api_timeout_s))
        err = None
        try:
            return getattr(self.session, method)(url, params=self._params(params), **kw)
        except OdmError:
            raise                                           # 我們自己的（例如檔案在上傳中途變動），訊息不含權杖
        except Exception as e:                              # noqa: BLE001 - requests／urllib3／OSError 都算連線失敗
            ambiguous = bool(body is not None and getattr(body, "complete", False))
            err = OdmTransportError(f"{what}：連線失敗（{type(e).__name__}: {self.redact(e)}）", ambiguous=ambiguous)
        raise err

    def _json(self, resp, what):
        status = getattr(resp, "status_code", None)
        try:
            data = resp.json()
        except ValueError:
            raise OdmError(f"{what}: HTTP {status}，回應不是 JSON", status=status) from None
        if status != 200 or (isinstance(data, dict) and data.get("error")):
            remote = data.get("error") if isinstance(data, dict) else data
            no_retry = bool(isinstance(data, dict) and data.get("noRetry"))
            raise OdmError(f"{what}: HTTP {status} {self.redact(remote)}", status=status,
                           remote_error=self.redact(remote), no_retry=no_retry)
        return data

    def _success(self, resp, what) -> dict:
        d = self._json(resp, what)
        if not (isinstance(d, dict) and d.get("success") is True):
            remote = d.get("error") if isinstance(d, dict) else d
            raise OdmError(f"{what}: 節點沒有回 success（{self.redact(remote)}）", status=getattr(resp, "status_code", None),
                           remote_error=self.redact(remote))
        return d

    # ── 節點 ────────────────────────────────────────────────────────────────
    def node_info(self) -> dict:
        """GET /info：version、engine、engineVersion、maxImages、availableMemory、totalMemory、cpuCores、
        maxParallelTasks、taskQueueCount（N index.js:755-783）。``engine`` 在 ODX 3.8.4 永遠是退回值 "odx"，不是證據。"""
        d = self._json(self._send("get", "/info", "GET /info"), "GET /info")
        if not isinstance(d, dict):
            raise OdmError(f"GET /info 回應不是物件：{type(d).__name__}")
        return d

    def engine_options(self) -> list:
        """GET /options：節點引擎認得的參數 ``[{name, type, value(預設，字串), domain, help}]``（N odmInfo.js:70-161）。"""
        d = self._json(self._send("get", "/options", "GET /options"), "GET /options")
        if not isinstance(d, list):
            raise OdmError(f"GET /options 回應不是陣列：{type(d).__name__}")
        return d

    # ── 分段上傳 ────────────────────────────────────────────────────────────
    def init_task(self, task_uuid: str, *, options: list, name: Optional[str] = None, attempts: int = 3,
                  backoff_s: Sequence[float] = (2.0, 4.0), sleep=time.sleep) -> str:
        """POST /task/new/init（multipart，set-uuid = 本機產生的 uuid；連線失敗可原樣重送，R1 §2.1）。"""
        if not _UUID_RE.match(str(task_uuid)):
            raise ValueError(f"set-uuid 必須是 UUID：{task_uuid!r}")
        fields = [("options", json.dumps(options))]
        if name:
            fields.append(("name", name))
        what = "POST /task/new/init"
        last = None
        for k in range(max(1, int(attempts))):
            body = MultipartStream(fields=fields)
            try:
                resp = self._send("post", "/task/new/init", what, body=body, data=body,
                                  headers={"Content-Type": body.content_type, "set-uuid": task_uuid})
            except OdmTransportError as e:
                last = e
                if k + 1 < attempts:
                    sleep(backoff_s[min(k, len(backoff_s) - 1)] if backoff_s else 0.0)
                continue
            finally:
                body.close()
            d = self._json(resp, what)
            got = d.get("uuid") if isinstance(d, dict) else None
            if got != task_uuid:
                raise OdmError(f"{what} 回傳的 uuid {got!r} 與送出的 set-uuid {task_uuid!r} 不同")
            return got
        raise last

    def upload_files(self, task_uuid: str, files, *, max_files: int = UPLOAD_BATCH_MAX_FILES,
                     max_bytes: int = UPLOAD_BATCH_MAX_BYTES, attempts: int = UPLOAD_ATTEMPTS,
                     backoff_s: Sequence[float] = UPLOAD_BACKOFF_S, allow_ambiguous_retry: bool = True,
                     progress: Optional[Callable[[dict], None]] = None,
                     should_cancel: Optional[Callable[[], bool]] = None, sleep=time.sleep,
                     on_batch: Optional[Callable[[dict], None]] = None, batch_offset: int = 0,
                     n_batches_total: Optional[int] = None, files_offset: int = 0, bytes_offset: int = 0,
                     n_files_total: Optional[int] = None, bytes_total: Optional[int] = None) -> list:
        """分批上傳到 /task/new/upload/{uuid}（欄名 ``images``，從磁碟串流）。回傳每批的報告。

        ``files``：``[(路徑, 上傳檔名), ...]``。``progress(dict)`` 收到 ``files_done, n_files, bytes_done,
        bytes_total, batch, n_batches, file``（同一批重送時 bytes_done 會退回該批開頭）。``should_cancel`` 在每批之前、
        每次重送等待時（每 0.5 s）、以及傳送中每個讀取區塊檢查 ⇒ :class:`OdmCanceled`（傳到一半的那批由節點整批丟掉）。

        重送規則（R1 §2.2）：拿到 HTTP 5xx、或主體沒送完就斷線 ⇒ 節點已刪掉這個請求的檔案，可以重送；
        主體送完才斷線（``ambiguous``）⇒ ``allow_ambiguous_retry`` 才重送並記在 ``ambiguous_retries``；
        HTTP 200 的 ``{"error"}``（例如 uuid 不存在、欄名錯、權杖錯）一律不重送。
        """
        items = []
        for p, name in files:
            p = Path(p)
            items.append((p, name, p.stat().st_size))
        batches = plan_batches(items, max_files=max_files, max_bytes=max_bytes)
        n_files = n_files_total if n_files_total is not None else files_offset + len(items)
        total = bytes_total if bytes_total is not None else bytes_offset + sum(i[2] for i in items)
        n_batches = n_batches_total if n_batches_total is not None else batch_offset + len(batches)
        state = {"files_done": files_offset, "bytes_done": bytes_offset, "last": 0}
        reports = []
        what = f"POST /task/new/upload/{task_uuid}"

        def emit(batch_no, filename, force=False):
            if progress is None:
                return
            if not force and state["bytes_done"] - state["last"] < _PROGRESS_STEP_BYTES and filename is None:
                return
            state["last"] = state["bytes_done"]
            progress({"files_done": state["files_done"], "n_files": n_files, "bytes_done": state["bytes_done"],
                      "bytes_total": total, "batch": batch_no, "n_batches": n_batches, "file": filename})

        def canceled():
            return should_cancel is not None and should_cancel()

        for bi, batch in enumerate(batches):
            batch_no = batch_offset + bi + 1
            if canceled():
                raise OdmCanceled(f"已在第 {batch_no}/{n_batches} 批之前取消上傳", batches=reports)
            start_files, start_bytes = state["files_done"], state["bytes_done"]
            rep = {"index": batch_no, "n_files": len(batch), "bytes": sum(i[2] for i in batch),
                   "first": batch[0][1], "last": batch[-1][1], "attempts": 0, "ambiguous_retries": 0, "errors": []}
            t0 = time.monotonic()
            emit(batch_no, None, force=True)
            while True:
                rep["attempts"] += 1
                state["files_done"], state["bytes_done"] = start_files, start_bytes

                def on_prog(nbytes, done_name, _bn=batch_no):
                    state["bytes_done"] += nbytes
                    if done_name is not None:
                        state["files_done"] += 1
                    emit(_bn, done_name)

                body = MultipartStream(files=[("images", name, p, size) for p, name, size in batch],
                                       on_progress=on_prog, should_abort=canceled)
                retry_reason = None
                try:
                    resp = self._send("post", f"/task/new/upload/{task_uuid}", what, body=body, data=body,
                                      headers={"Content-Type": body.content_type}, timeout=self._timeout())
                except OdmCanceled:
                    raise OdmCanceled(f"第 {batch_no}/{n_batches} 批上傳途中取消（這一批中斷，節點會整批丟掉）",
                                      batches=reports) from None
                except OdmTransportError as e:
                    if e.ambiguous and not allow_ambiguous_retry:
                        rep["errors"].append(str(e))
                        raise OdmTransportError(
                            f"{what}：第 {batch_no} 批主體已送完但沒有回應（{e}）；這批不能重送"
                            "（節點會把同名檔改名成 *_.txt，geo_.txt 會被當成 GCP），已中止", ambiguous=True) from None
                    retry_reason = str(e)
                    if e.ambiguous:
                        rep["ambiguous_retries"] += 1
                else:
                    status = getattr(resp, "status_code", None)
                    if status is not None and status >= 500:
                        try:
                            detail = self.redact((resp.json() or {}).get("error"))
                        except Exception:                   # noqa: BLE001
                            detail = "回應不是 JSON"
                        retry_reason = f"{what}: HTTP {status}（{detail}）"
                        if not allow_ambiguous_retry and body.complete:
                            # 「節點已刪掉檔案」只對 NodeODX 自己在存檔前回的 500 成立；主體送完後才來的 5xx（例如反向
                            # 代理的 502／504）可能是存好之後才壞 ⇒ 重送會變成 geo_.txt／gcp_list_.txt（審查 T8）
                            rep["errors"].append(retry_reason)
                            raise OdmTransportError(
                                f"{what}：第 {batch_no} 批主體已送完卻收到 HTTP {status}（{detail}）；這批不能重送"
                                "（節點可能已存好檔案，重送會變成 *_.txt，geo_.txt 會被當成 GCP），已中止",
                                ambiguous=True)
                    else:
                        self._success(resp, what)       # HTTP 200 {"error"}／4xx ⇒ 直接丟（不重送）
                finally:
                    body.close()
                if retry_reason is None:
                    break
                rep["errors"].append(retry_reason)
                if rep["attempts"] >= attempts:
                    raise OdmTransportError(f"第 {batch_no}/{n_batches} 批上傳失敗（試了 {rep['attempts']} 次）："
                                            f"{retry_reason}", ambiguous=bool(rep["ambiguous_retries"]))
                wait = backoff_s[min(rep["attempts"] - 1, len(backoff_s) - 1)] if backoff_s else 0.0
                t = 0.0
                while True:
                    if canceled():
                        raise OdmCanceled(f"第 {batch_no}/{n_batches} 批重送前取消上傳", batches=reports)
                    if t >= wait:
                        break
                    dt = min(0.5, wait - t)
                    sleep(dt)
                    t += dt
            rep["seconds"] = round(time.monotonic() - t0, 3)
            reports.append(rep)
            emit(batch_no, None, force=True)
            if on_batch is not None:
                on_batch(dict(rep))
        return reports

    def commit_task(self, task_uuid: str) -> str:
        """POST /task/new/commit/{uuid}。連線失敗時用 info 判斷是否其實已提交（第二次 commit 只會回 uuid 不存在）。

        三種結果：確定已提交（回 uuid）；確定**沒**提交（info 說不存在、探測也說上傳不在）⇒ :class:`OdmError`；
        **不確定**（info 或探測連不上）⇒ ``OdmTransportError(commit_unknown=True)``——任務可能正在節點上跑，
        呼叫端要留在「提交中」、之後接回確認，不能當成沒提交（審查 T4）。
        """
        what = f"POST /task/new/commit/{task_uuid}"
        try:
            resp = self._send("post", f"/task/new/commit/{task_uuid}", what, data=b"")
        except OdmTransportError as e:
            info_err = None
            try:
                self.task_info(task_uuid)
                return task_uuid                            # 已提交、只是回應遺失
            except OdmError as ie:
                info_err = ie
            probe = self.probe_upload_session(task_uuid)
            if probe is True:                               # 上傳還在、沒提交 ⇒ 再送一次
                try:
                    resp = self._send("post", f"/task/new/commit/{task_uuid}", what, data=b"")
                except OdmTransportError as e2:
                    raise OdmTransportError(f"{e2}；重送 commit 也沒有回應：提交結果不明", commit_unknown=True) from None
            elif probe is False and info_err.not_found and not isinstance(info_err, OdmTransportError):
                raise OdmError(f"{e}；節點上查不到這個任務、也沒有未提交的上傳（commit 沒有送達）",
                               remote_error=info_err.remote_error) from None
            else:
                raise OdmTransportError(f"{e}；事後查詢也不確定（info：{info_err}；探測上傳：{probe}）：提交結果不明",
                                        commit_unknown=True) from None
        d = self._json(resp, what)
        got = d.get("uuid") if isinstance(d, dict) else None
        if got != task_uuid:
            raise OdmError(f"{what} 回傳的 uuid {got!r} 與送出的不同")
        return got

    def probe_upload_session(self, task_uuid: str) -> Optional[bool]:
        """未提交的上傳是否還在：空主體 POST upload ⇒ "Need at least 1 file." 表示在、"Invalid uuid" 表示不在（R1 §2.2）。"""
        try:
            resp = self._send("post", f"/task/new/upload/{task_uuid}", "probe upload", data=b"")
            d = resp.json()
        except (OdmError, ValueError):
            return None
        err = str(d.get("error") or "") if isinstance(d, dict) else ""
        if "need at least 1 file" in err.lower():
            return True
        if "invalid uuid" in err.lower():
            return False
        return None

    # ── 任務 ────────────────────────────────────────────────────────────────
    def task_info(self, task_uuid: str) -> dict:
        """GET /task/{uuid}/info：status.code、progress（0–100）、processingTime（ms，-1 = 還沒開始）、imagesCount、options。"""
        what = f"GET /task/{task_uuid}/info"
        return self._json(self._send("get", f"/task/{task_uuid}/info", what), what)

    def task_output(self, task_uuid: str, line: int = 0) -> list:
        """GET /task/{uuid}/output?line=N：第 N 行起的主控台輸出（只在節點記憶體裡，節點重開就沒了）。"""
        what = f"GET /task/{task_uuid}/output"
        d = self._json(self._send("get", f"/task/{task_uuid}/output", what, params={"line": int(line)}), what)
        if not isinstance(d, list):
            raise OdmError(f"{what} 回應不是陣列")
        return [str(x) for x in d]

    def cancel_task(self, task_uuid: str) -> dict:
        """POST /task/cancel（表單欄位 uuid）。終止不是立即的；未提交的 uuid 回 not found（R1 §2.4）。"""
        return self._success(self._send("post", "/task/cancel", "POST /task/cancel", data={"uuid": task_uuid}),
                             f"POST /task/cancel {task_uuid}")

    def remove_task(self, task_uuid: str) -> dict:
        """POST /task/remove：取消並刪除節點上的任務資料夾（含 all.zip）。"""
        return self._success(self._send("post", "/task/remove", "POST /task/remove", data={"uuid": task_uuid}),
                             f"POST /task/remove {task_uuid}")

    def new_task(self, images, *, options: list, geo_txt=None, gcp_txt=None, name: Optional[str] = None,
                 task_uuid: Optional[str] = None, progress=None, should_cancel=None, sleep=time.sleep,
                 max_files: int = UPLOAD_BATCH_MAX_FILES, max_bytes: int = UPLOAD_BATCH_MAX_BYTES,
                 force_gps: Optional[bool] = None) -> str:
        """init → 附檔一批（不重送模稜兩可的失敗）→ 影像分批 → commit；回傳 uuid。

        survey.pipeline.run_odm_job 自己逐步呼叫這些步驟以便寫任務紀錄；這個函式給簡單用途。
        失敗或取消時**不**清理節點（未提交的上傳沒有刪除 API，見模組說明）。
        給了 ``gcp_txt`` 就必須明確給 ``force_gps``（True／False），而且要與 ``options`` 裡有沒有 force-gps 一致：
        有 GCP 時 ODX 預設照片 GPS 不進平差（X modules/osfm.py:329-337），沒有預設。
        """
        imgs, aux = classify_uploads(images, geo_txt=geo_txt, gcp_txt=gcp_txt)
        if gcp_txt is not None:
            if not isinstance(force_gps, bool):
                raise ValueError("給了 gcp_txt 就必須明確給 force_gps=True（--force-gps：照片 GPS 也進平差）或 "
                                 "False（只用 GCP）：沒有預設")
            sent = any(isinstance(o, dict) and o.get("name") == "force-gps" and o.get("value") for o in options)
            if sent != force_gps:
                raise ValueError(f"force_gps={force_gps}，但 options {'有' if sent else '沒有'} force-gps：兩者要一致")
        task_uuid = task_uuid or str(uuid_mod.uuid4())
        self.init_task(task_uuid, options=options, name=name, sleep=sleep)
        if aux:
            self.upload_files(task_uuid, aux, allow_ambiguous_retry=False, should_cancel=should_cancel, sleep=sleep,
                              max_files=max_files, max_bytes=max_bytes)
        self.upload_files(task_uuid, imgs, progress=progress, should_cancel=should_cancel, sleep=sleep,
                          max_files=max_files, max_bytes=max_bytes)
        return self.commit_task(task_uuid)

    def wait(self, task_uuid: str, *, poll_s: float = 10.0, timeout_s: float = 24 * 3600.0, sleep=time.sleep,
             clock=time.monotonic, on_info: Optional[Callable[[dict], None]] = None) -> dict:
        """輪詢到結束；FAILED／CANCELED 丟 OdmError，COMPLETED 回傳最後的 info。``on_info`` 每次輪詢都收到 info。"""
        t0 = clock()
        while True:
            info = self.task_info(task_uuid)
            if on_info is not None:
                on_info(info)
            code = (info.get("status") or {}).get("code")
            if code == STATUS_COMPLETED:
                return info
            if code in (STATUS_FAILED, STATUS_CANCELED):
                raise OdmError(f"任務 {task_uuid} 狀態 {code}：{info.get('status')}")
            if clock() - t0 > timeout_s:
                raise OdmError(f"任務 {task_uuid} 等待逾時（{timeout_s} s），最後狀態 {code}")
            sleep(poll_s)

    def download_all(self, task_uuid: str, dest, *, progress: Optional[Callable[[int, Optional[int]], None]] = None,
                     should_cancel: Optional[Callable[[], bool]] = None) -> Path:
        """GET /task/{uuid}/download/all.zip 串流到檔案（先寫 .part，完整才改名）。

        節點不支援 Range（整檔 pipe，R1 §3）⇒ 中斷只能從頭重下；Content-Length 與實際位元組不符 ⇒ 丟錯、刪 .part。
        ``progress(已下載位元組, 總位元組或 None)``；``should_cancel`` 為真 ⇒ :class:`OdmCanceled`（刪 .part）。
        """
        dest = Path(dest)
        dest.parent.mkdir(parents=True, exist_ok=True)
        what = f"GET /task/{task_uuid}/download/all.zip"
        resp = self._send("get", f"/task/{task_uuid}/download/all.zip", what, stream=True, timeout=self._timeout())
        tmp = dest.with_suffix(dest.suffix + ".part")
        try:
            headers = getattr(resp, "headers", None) or {}
            ctype = headers.get("Content-Type", "") or ""
            if resp.status_code != 200 or "json" in ctype.lower():
                self._json(resp, what)
                raise OdmError(f"下載 {task_uuid} 失敗：HTTP {resp.status_code}")
            if "zip" not in ctype.lower() and "octet-stream" not in ctype.lower():
                # NodeODX 成功時是 application/zip（N index.js:504-521）；其他（例如代理伺服器的 HTML 錯誤頁）不當成果存
                raise OdmError(f"{what}：回應的 Content-Type 是 {ctype!r}，不是 application/zip")
            clen = headers.get("Content-Length")
            expected = int(clen) if clen not in (None, "") and str(clen).isdigit() else None
            keep = ("成果仍在節點上（完成後預設保留 48 h），清出空間後用「接回任務」重下")
            if expected is not None:
                free = free_bytes(dest.parent)
                if free is not None and free < expected:
                    raise OdmLocalError(f"{what}：磁碟空間不足——all.zip 有 {expected / 2**30:.2f} GiB，{dest.parent} "
                                        f"只剩 {free / 2**30:.2f} GiB（解壓還要差不多一樣多）；沒有下載。{keep}")
            got = 0
            err = None

            def write_local(fh, data):
                try:
                    fh.write(data)
                except OSError as we:                       # 本機寫檔（ENOSPC 等）≠ 連線中斷
                    raise OdmLocalError(f"{what}：寫入本機 {tmp.name} 失敗（{type(we).__name__}: {we}），已收 {got} B："
                                        f"多半是磁碟空間不足。{keep}") from None
            try:
                try:
                    fh = open(tmp, "wb")
                except OSError as oe:
                    raise OdmLocalError(f"{what}：開不了本機檔案 {tmp}（{type(oe).__name__}: {oe}）。{keep}") from None
                try:
                    for chunk in resp.iter_content(chunk_size=1 << 20):
                        if chunk:
                            write_local(fh, chunk)
                            got += len(chunk)
                            if progress is not None:
                                progress(got, expected)
                        if should_cancel is not None and should_cancel():
                            raise OdmCanceled("已取消下載")
                finally:
                    try:
                        fh.close()                          # 緩衝區最後一次寫入也可能 ENOSPC
                    except OSError as ce:
                        raise OdmLocalError(f"{what}：關閉本機檔案 {tmp.name} 失敗（{type(ce).__name__}: {ce}）。"
                                            f"{keep}") from None
            except OdmError:
                tmp.unlink(missing_ok=True)
                raise
            except Exception as e:                          # noqa: BLE001 - 連線中斷：訊息先遮蔽
                err = OdmTransportError(f"{what}：下載中斷（{type(e).__name__}: {self.redact(e)}），已收 {got} B；"
                                        "節點不支援續傳，只能從頭重下")
            if err is not None:
                tmp.unlink(missing_ok=True)
                raise err
            if expected is not None and got != expected:
                tmp.unlink(missing_ok=True)
                raise OdmError(f"{what}：Content-Length {expected} B，實際收到 {got} B（下載不完整）")
            tmp.replace(dest)
            return dest
        finally:
            close = getattr(resp, "close", None)
            if close is not None:
                close()
