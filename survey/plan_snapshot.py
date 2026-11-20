"""飛行計畫快照（flown-plan snapshot）—— 純資料、不碰 GUI、不連線、不讀設定檔。

為什麼要有這支（2026-10-20 Step 0 第 6 項）
==========================================
航測後處理（survey_process_dialog）今天讀的是**即時**狀態：地圖上的作業區（``MainWindow.survey_area_polygons``）
與 ``gcp_blocks_provider(self)`` 從 HvDccppCoordinator 現場算出的 UAV 區塊。飛完、重開程式、或在地圖上改了作業區
／重新規劃之後，後處理拿到的就不是「真的飛的那一份」。所以在建圖任務**上傳成功**時，把那一份計畫存成一個
帶版本、可驗證、內容決定性的 JSON 檔；後處理可以改讀它，並記錄用的是哪一個來源。

格式（``aeroplan-plan-snapshot/1``）
------------------------------------
::

    {"format": "aeroplan-plan-snapshot/1", "written_by": "survey.plan_snapshot",
     "uploaded_at_utc": "2026-10-20T03:04:05Z",     # 唯一的牆鐘欄位（這批任務送出的時刻）
     "software": {...},                             # aeroplan 的 git describe（不進 plan_sha256）
     "plan_sha256": "...",                          # plan 段的 sha256（同一份計畫＋同一批任務 ⇒ 同值）
     "digest_sha256": "...",                        # 整份（去掉這一欄）的 sha256：竄改／截斷偵測
     "plan": {planner, areas, survey_polygons, fleet, aircraft, mission_uploads, mapping, wind, terrain,
              takeoff_schedule, gcp_blocks, exposure_prediction, params_plan_time, warnings, data_issues},
     "upload": {aircraft: {sysid: {ok, message, ...}}, readback_info, skipped_sysids, not_targeted_sysids, tlog}}

* **決定性**：``plan`` 段只放計畫內容（沒有牆鐘；任何層級叫 ``elapsed_s``／``timing_ms`` 的計算耗時欄位都不收，
  丟掉的路徑列在 ``plan.data_issues.dropped_clock_fields``）；檔案以 sort_keys、ensure_ascii=False、
  LF 寫出 ⇒ 同一份輸入（含同一個 ``uploaded_at_utc``）逐位元相同；換上傳時刻只改 ``uploaded_at_utc`` 與
  ``digest_sha256``，``plan_sha256`` 不變。
* **不是有限數的浮點**（例如風層頂 ``z_top_m = inf``）寫成字串 ``"Infinity"``／``"-Infinity"``／``"NaN"``，
  欄位路徑列在 ``plan.data_issues.nonfinite_as_string``（不默默變 null）；看不懂的物件型別寫成
  ``{"__unserializable__": 型別名}`` 並列在 ``data_issues.unserializable``。
* **任務項目整份內嵌**（不是只放摘要）：後處理要逐條帶的 206 觸發距離與位置核對張數，只放 digest 就又得回頭依賴
  即時規劃器狀態——那正是這個檔要拿掉的東西。大小有上限：飛控任務儲存本身只放得下數百到數千項（ArduPilot
  AP_Mission 依 storage 大小），1000 項約 0.3 MB；超過 :data:`MISSION_ITEMS_INLINE_MAX` 才改成只記 sha256＋項數，
  並寫進 ``warnings``（大聲說）。
* **上傳結果**（每架 ACK）在 ``upload`` 段（不進 ``plan_sha256``）：主視窗在第一個 ok=True 的 ACK 才寫檔
  （:func:`write_snapshot`，永不覆蓋：同名加 ``_2``、``_3``）；之後的 ACK 用 :func:`update_upload_status`
  就地更新同一個檔（只改 upload 段、驗證 plan_sha256 不變、已記錄的結果不可被改寫）。
* **沒有機密**：建構器只從白名單取值；面板參數只收 ``hv_`` 開頭與幾個一般鍵，名稱像 token／key／secret／password
  的一律不收；連線字串去掉 ``?query``、``#fragment`` 與 ``帳號@``。這支從來不讀 config/settings.json。

檔名
----
``<root>/<YYYYMMDD>/flown_plan_<HHMMSS>_<plan_sha256 前 8 碼>.json``（日期時間＝``uploaded_at_utc`` 換成本機時間，
與同一天的 tlog ``runs/survey/<YYYYMMDD>/UAV-n_<HHMMSS>.tlog`` 放在一起）。``root`` 預設
:func:`default_root`：環境變數 ``AEROPLAN_PLAN_SNAPSHOT_DIR``，否則 ``<repo>/runs/survey``；pytest 中沒設環境變數 ⇒
丟例外（測試不可以把假的「已飛計畫」寫進 repo）。

API（給 UI）
------------
* :func:`snapshot_from_hv` ``(hv, swarm, jobs, *, takeoff_offsets, takeoff_source, readback_info, skipped,
  not_targeted, tlog)`` → dict：在 ``_upload_mapping_missions`` 送出後（只傳**真的送出**的 jobs）建好放記憶體。
* :func:`set_upload_status` ``(snap, sysid, ok=, message=)`` → 新 dict（純函式）。
* :func:`write_snapshot` ``(root, snap)`` → Path（原子寫入、永不覆蓋；至少一架 ok=True 才准寫）。
* :func:`update_upload_status` ``(path, sysid, ok=, message=)`` → 更新後的 dict。
* :func:`read_snapshot` ``(path)`` → dict（嚴格驗證：格式、digest、plan_sha256、必要欄位、多邊形、GCP 區塊）。
* :func:`snapshot_polygons`／:func:`snapshot_gcp_blocks`／:func:`polygons_provider`／:func:`blocks_provider`／
  :func:`source_label`／:func:`source_record`／:func:`summary_lines`：給兩個對話框用。
"""
from __future__ import annotations

import copy
import datetime as _dt
import functools
import hashlib
import json
import math
import os
import platform
import re
import subprocess
import uuid
from dataclasses import fields, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Mapping, Optional, Sequence

import numpy as np

FORMAT_FAMILY = "aeroplan-plan-snapshot"
FORMAT = f"{FORMAT_FAMILY}/1"
WRITTEN_BY = "survey.plan_snapshot"
ENV_DIR = "AEROPLAN_PLAN_SNAPSHOT_DIR"
FILENAME_PREFIX = "flown_plan_"
#: 每架任務項目內嵌上限（超過只記 sha256＋項數，並寫進 warnings）。ArduPilot 任務儲存遠小於這個數。
MISSION_ITEMS_INLINE_MAX = 5000
#: 上傳結果訊息的長度上限（字元）
MESSAGE_MAX_CHARS = 2000
#: 同名檔最多試到 _N
MAX_COLLISIONS = 999
CMD_DO_SET_CAM_TRIGG_DIST = 206
UTC_FMT = "%Y-%m-%dT%H:%M:%SZ"

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_TOP_KEYS = frozenset({"format", "written_by", "uploaded_at_utc", "software", "plan_sha256", "digest_sha256",
                       "plan", "upload"})
_PLAN_KEYS = frozenset({"planner", "areas", "survey_polygons", "fleet", "aircraft", "mission_uploads", "mapping",
                        "wind", "terrain", "takeoff_schedule", "gcp_blocks", "exposure_prediction",
                        "params_plan_time", "warnings", "data_issues"})
_UPLOAD_KEYS = frozenset({"aircraft", "readback_info", "skipped_sysids", "not_targeted_sysids", "tlog"})
_HEX64 = re.compile(r"[0-9a-f]{64}")
#: 計畫時的面板參數：只收這些（hv_ 開頭另外收）；名稱像機密的一律不收
_PARAM_KEYS = frozenset({"num_drones", "altitude", "speed", "target_gsd", "overlap", "coverage_width",
                         "scan_angle", "turn_radius"})
_SECRETISH = re.compile(r"token|secret|passw|api_?key|credential|auth|(^|_)keys?($|_)", re.IGNORECASE)
#: 牆鐘／計時欄位：不進快照（決定性）
_CLOCK_KEYS = frozenset({"elapsed_s", "timing_ms"})


class PlanSnapshotError(ValueError):
    """快照建立／驗證／寫入失敗（訊息為繁中，可直接顯示給操作員）。"""


# ─────────────────────────────────────────────────────────────────────────────
# 路徑與版本
# ─────────────────────────────────────────────────────────────────────────────

def default_root() -> Path:
    """快照根目錄：環境變數 ``AEROPLAN_PLAN_SNAPSHOT_DIR``，否則 ``<repo>/runs/survey``（與 tlog 同一處）。

    pytest 執行中（``PYTEST_CURRENT_TEST``）而沒設環境變數 ⇒ 丟例外：測試不可以把假的「已飛計畫」寫進 repo 的 runs/。
    """
    env = (os.environ.get(ENV_DIR) or "").strip()
    if env:
        return Path(env)
    if os.environ.get("PYTEST_CURRENT_TEST"):
        raise PlanSnapshotError(f"測試中沒有設 {ENV_DIR}：飛行計畫快照不可以寫進 repo 的 runs/survey"
                                "（請在 conftest 或測試裡設到暫存目錄）")
    return _PROJECT_ROOT / "runs" / "survey"


@functools.lru_cache(maxsize=1)
def _git_describe() -> tuple:
    try:
        env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")          # 不要順手改寫 .git/index（別的工作階段可能在用）
        kw = {"creationflags": 0x08000000} if os.name == "nt" else {}   # CREATE_NO_WINDOW：不閃主控台
        r = subprocess.run(["git", "-C", str(_PROJECT_ROOT), "describe", "--always", "--dirty", "--tags"],
                           capture_output=True, text=True, timeout=5, env=env, **kw)
        if r.returncode == 0 and r.stdout.strip():
            return r.stdout.strip(), None
        return None, f"git describe 失敗（rc={r.returncode}）：{(r.stderr or '').strip()[:200]}"
    except Exception as e:                                       # noqa: BLE001 - 沒有 git 就照實記下
        return None, f"git describe 失敗：{type(e).__name__}: {e}"


def software_version() -> dict:
    """``{"aeroplan_git_describe", "aeroplan_git_error", "python"}``。git 結果在這個行程裡只取一次（約 0.1 s）。"""
    desc, err = _git_describe()
    return {"aeroplan_git_describe": desc, "aeroplan_git_error": err, "python": platform.python_version()}


def _hv_dccpp_version() -> Optional[str]:
    import sys
    mod = sys.modules.get("hv_dccpp")
    if mod is None:
        try:
            import hv_dccpp as mod                                # noqa: PLC0415 - 只讀版本字串
        except Exception:                                          # noqa: BLE001
            return None
    v = getattr(mod, "__version__", None)
    return str(v) if v is not None else None


# ─────────────────────────────────────────────────────────────────────────────
# 純資料化、雜湊
# ─────────────────────────────────────────────────────────────────────────────

def _new_issues() -> dict:
    return {"nonfinite_as_string": [], "unserializable": [], "dropped_clock_fields": []}


def _key(k, path: str, issues: dict) -> str:
    if isinstance(k, str):
        return k
    if isinstance(k, bool):
        return "true" if k else "false"
    if isinstance(k, (int, np.integer)):
        return str(int(k))
    if isinstance(k, (float, np.floating)) and math.isfinite(float(k)):
        return repr(float(k))
    if isinstance(k, Enum):
        return k.name
    issues["unserializable"].append(f"{path}（鍵 {type(k).__qualname__}）")
    return f"<{type(k).__qualname__}>"


def _plain(obj, path: str, issues: dict):
    """任意物件 → 只含 dict(str 鍵)／list／str／int／float(有限)／bool／None 的資料。看不懂的型別不猜（標記並記路徑）。"""
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        x = float(obj)
        if math.isfinite(x):
            return x
        issues["nonfinite_as_string"].append(path)
        return "NaN" if math.isnan(x) else ("Infinity" if x > 0 else "-Infinity")
    if isinstance(obj, Enum):
        return obj.name
    if isinstance(obj, Path):
        return str(obj)
    if isinstance(obj, np.ndarray):
        return [_plain(v, f"{path}[{i}]", issues) for i, v in enumerate(obj.tolist())]
    if isinstance(obj, Mapping):
        out = {}
        for k in obj:
            sk = _key(k, path, issues)
            if sk in _CLOCK_KEYS and path.startswith("plan"):
                # 計算耗時（elapsed_s／timing_ms）每次都不同 ⇒ plan 段不收（決定性），丟掉的路徑記下來
                issues["dropped_clock_fields"].append(f"{path}.{sk}")
                continue
            if sk in out:
                raise PlanSnapshotError(f"{path} 有兩個鍵轉成字串後相同：{sk!r}")
            out[sk] = _plain(obj[k], f"{path}.{sk}", issues)
        return out
    if isinstance(obj, (list, tuple)):
        return [_plain(v, f"{path}[{i}]", issues) for i, v in enumerate(obj)]
    if isinstance(obj, (set, frozenset)):
        vals = [_plain(v, f"{path}{{}}", issues) for v in obj]
        return sorted(vals, key=lambda v: _canonical(v).decode("utf-8"))
    to_dict = getattr(obj, "to_dict", None)
    if callable(to_dict) and not isinstance(obj, type):
        return _plain(to_dict(), path, issues)
    if is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: _plain(getattr(obj, f.name), f"{path}.{f.name}", issues) for f in fields(obj)}
    geo = getattr(obj, "__geo_interface__", None)
    if isinstance(geo, Mapping):
        return _plain(geo, path, issues)
    issues["unserializable"].append(f"{path}（{type(obj).__qualname__}）")
    return {"__unserializable__": type(obj).__qualname__}


def _canonical(obj) -> bytes:
    """雜湊用的正規 JSON：sort_keys、不轉義非 ASCII、最緊湊分隔、拒絕 NaN/Inf。"""
    return json.dumps(obj, sort_keys=True, ensure_ascii=False, separators=(",", ":"),
                      allow_nan=False).encode("utf-8")


def _sha256(obj) -> str:
    return hashlib.sha256(_canonical(obj)).hexdigest()


def _file_bytes(snap: Mapping) -> bytes:
    return (json.dumps(snap, sort_keys=True, ensure_ascii=False, indent=1, allow_nan=False) + "\n").encode("utf-8")


def plan_sha256(snap: Mapping) -> str:
    """``plan`` 段的 sha256（計畫身分：同一份計畫＋同一批任務 ⇒ 同值，與上傳時刻、ACK 結果無關）。"""
    return _sha256(snap["plan"])


def content_digest(snap: Mapping) -> str:
    """整份快照去掉 ``digest_sha256`` 後的 sha256。"""
    return _sha256({k: v for k, v in snap.items() if k != "digest_sha256"})


def _seal(doc: dict) -> dict:
    doc["plan_sha256"] = _sha256(doc["plan"])
    doc["digest_sha256"] = content_digest(doc)
    return doc


# ─────────────────────────────────────────────────────────────────────────────
# 時間
# ─────────────────────────────────────────────────────────────────────────────

def _utc_text(when) -> str:
    if when is None:
        when = _dt.datetime.now(_dt.timezone.utc)
    if isinstance(when, str):
        _parse_utc(when)
        return when
    if not isinstance(when, _dt.datetime):
        raise PlanSnapshotError(f"uploaded_at_utc 要是 datetime 或 'YYYY-MM-DDTHH:MM:SSZ' 字串，收到 {type(when).__name__}")
    if when.tzinfo is None or when.utcoffset() is None:
        raise PlanSnapshotError("uploaded_at_utc 的 datetime 沒有時區：不猜是本機還是 UTC，請給有時區的時間")
    return when.astimezone(_dt.timezone.utc).strftime(UTC_FMT)


def _parse_utc(text) -> _dt.datetime:
    if not isinstance(text, str):
        raise PlanSnapshotError(f"uploaded_at_utc 要是字串，收到 {type(text).__name__}")
    try:
        return _dt.datetime.strptime(text, UTC_FMT).replace(tzinfo=_dt.timezone.utc)
    except ValueError:
        raise PlanSnapshotError(f"uploaded_at_utc 格式要是 YYYY-MM-DDTHH:MM:SSZ（UTC），收到 {text!r}") from None


# ─────────────────────────────────────────────────────────────────────────────
# 建立（純資料）
# ─────────────────────────────────────────────────────────────────────────────

def _int_id(v, what: str) -> int:
    if isinstance(v, bool) or not isinstance(v, (int, np.integer, str)):
        raise PlanSnapshotError(f"{what} 要是整數，收到 {v!r}")
    try:
        return int(v)
    except ValueError:
        raise PlanSnapshotError(f"{what} 要是整數，收到 {v!r}") from None


def _redact_conn(text) -> Optional[str]:
    """連線字串去掉 ?query、#fragment 與 帳號@（MAVLink 連線字串本來就不該帶機密，但不賭）。"""
    if text is None:
        return None
    s = str(text)
    s = re.sub(r"[?#].*$", "?<已移除>", s)
    s = re.sub(r"(//)[^/@]*@", r"\1<已移除>@", s)
    return s


def _items_record(items, where: str, issues: dict) -> dict:
    if isinstance(items, (str, bytes)) or not isinstance(items, Sequence):
        raise PlanSnapshotError(f"{where} 的任務項目要是 dict 串列，收到 {type(items).__name__}")
    plain = _plain(list(items), f"{where}.mission_items", issues)
    if not plain:
        raise PlanSnapshotError(f"{where} 沒有任何任務項目")
    n_on = n_off = 0
    dists = set()
    for k, it in enumerate(plain):
        if not isinstance(it, dict) or "command" not in it:
            raise PlanSnapshotError(f"{where} 第 {k} 項不是含 command 的 dict：{str(it)[:120]}")
        try:
            cmd = int(it["command"])
            p1 = float(it.get("param1", 0.0) or 0.0)
        except (TypeError, ValueError):
            raise PlanSnapshotError(f"{where} 第 {k} 項的 command／param1 不是數字：{str(it)[:120]}") from None
        if cmd == CMD_DO_SET_CAM_TRIGG_DIST:
            if p1 > 0.0:
                n_on += 1
                dists.add(p1)
            else:
                n_off += 1
    rec = {"n_items": len(plain), "mission_items_sha256": _sha256(plain), "n_206_on": n_on, "n_206_off": n_off,
           "trigger_distances_m": sorted(dists)}
    if len(plain) <= MISSION_ITEMS_INLINE_MAX:
        rec["mission_items"] = plain
        rec["mission_items_omitted"] = None
    else:
        rec["mission_items"] = None
        rec["mission_items_omitted"] = (f"{len(plain)} 項 > 內嵌上限 {MISSION_ITEMS_INLINE_MAX}：只記 sha256 與項數"
                                        "（後處理無法從快照取逐項內容）")
    return rec


def _areas_section(areas, issues: dict) -> dict:
    if not isinstance(areas, Mapping) or "items" not in areas:
        raise PlanSnapshotError("areas 要是 {'source': 來源說明, 'items': [作業區…]}")
    src = areas.get("source")
    if not isinstance(src, str) or not src.strip():
        raise PlanSnapshotError("areas 沒有來源說明（source）：有效值契約要記錄作業區是從哪裡來的")
    raw = areas["items"]
    if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence) or not raw:
        raise PlanSnapshotError("沒有任何作業區：快照至少要有一塊作業區")
    items = []
    for k, a in enumerate(raw, start=1):
        if isinstance(a, Mapping):
            aid = a.get("area_id") or f"area_{k}"
            poly, holes = a.get("polygon"), a.get("holes") or []
        else:
            aid, poly, holes = f"area_{k}", a, []
        items.append({"area_id": str(aid), "polygon": _plain(poly, f"plan.areas.items[{k - 1}].polygon", issues),
                      "holes": _plain(list(holes), f"plan.areas.items[{k - 1}].holes", issues)})
    return {"source": src, "items": items}


def build_snapshot(*, planner: Mapping, areas: Mapping, aircraft: Mapping, mission_uploads: Sequence[Mapping],
                   mapping: Mapping, wind: Mapping, gcp_blocks: Mapping,
                   terrain: Optional[Mapping] = None, survey_polygons: Optional[Mapping] = None,
                   takeoff_schedule: Optional[Mapping] = None, fleet: Optional[Sequence] = None,
                   exposure_prediction: Optional[Mapping] = None, params_plan_time: Optional[Mapping] = None,
                   warnings: Sequence[str] = (), upload: Optional[Mapping] = None,
                   uploaded_at_utc=None, software: Optional[Mapping] = None) -> dict:
    """純資料 → 已封存（plan_sha256、digest_sha256 已算好）的快照 dict。不寫檔、不改輸入。

    必給：``planner``（含 ``name``）、``areas``（``{'source', 'items'}``；item 是 ``{'area_id','polygon','holes'}``
    或單純的 ring）、``aircraft``（``{drone_id: {...}}``，每架至少 ``uav_id``、``cls``、``strips``）、
    ``mission_uploads``（每筆 ``{'sysid','drone_id','items', 選填 'callsign','connection',
    'camera_params_expected','airspeed_expected','airspeed_commands','expected_photos'}``）、``mapping``、
    ``wind``（含 ``source``）、``gcp_blocks``（``{'source','blocks','info'}`` 或 ``{'source','error'}``）。
    選填的段沒給 ⇒ 記成 ``None``（明確的「沒提供」）。``uploaded_at_utc`` 缺省＝現在；``software`` 缺省＝
    :func:`software_version`。結果再經 :func:`validate_snapshot`（不要求已有 ok 的 ACK）。
    """
    issues = _new_issues()
    if not isinstance(planner, Mapping) or not str(planner.get("name") or "").strip():
        raise PlanSnapshotError("planner 要是含 name 的 dict（例如 {'name': 'HARVEST'}）")
    if not isinstance(aircraft, Mapping) or not aircraft:
        raise PlanSnapshotError("沒有任何一架飛機的計畫（aircraft 是空的）")
    if not isinstance(mapping, Mapping) or not mapping:
        raise PlanSnapshotError("沒有建圖／相機設定（mapping）：這不是建圖計畫的快照")
    if not isinstance(wind, Mapping) or not isinstance(wind.get("source"), str):
        raise PlanSnapshotError("wind 要是含 source（來源說明）的 dict")
    if not isinstance(gcp_blocks, Mapping) or not isinstance(gcp_blocks.get("source"), str):
        raise PlanSnapshotError("gcp_blocks 要是含 source 的 dict（{'source','blocks','info'} 或 {'source','error'}）")

    ac = {}
    for did, rec in aircraft.items():
        d = _int_id(did, "aircraft 的機號")
        if not isinstance(rec, Mapping):
            raise PlanSnapshotError(f"機 {d} 的計畫要是 dict")
        r = dict(rec)
        r["drone_id"] = d
        if str(d) in ac:
            raise PlanSnapshotError(f"機號 {d} 重複")
        ac[str(d)] = _plain(r, f"plan.aircraft.{d}", issues)

    if isinstance(mission_uploads, Mapping) or not mission_uploads:
        raise PlanSnapshotError("mission_uploads 要是非空的串列（每筆一架真的送出的任務）")
    mu, conns = {}, {}
    for k, job in enumerate(mission_uploads):
        if not isinstance(job, Mapping):
            raise PlanSnapshotError(f"mission_uploads 第 {k} 筆要是 dict")
        sysid = _int_id(job.get("sysid"), f"mission_uploads 第 {k} 筆的 sysid")
        did = _int_id(job.get("drone_id"), f"mission_uploads 第 {k} 筆的 drone_id")
        if str(sysid) in mu:
            raise PlanSnapshotError(f"UAV-{sysid} 在這批上傳裡出現兩次")
        if str(did) not in ac:
            raise PlanSnapshotError(f"UAV-{sysid} 上傳的是機 {did}，但計畫裡沒有這架")
        where = f"plan.mission_uploads.{sysid}"
        rec = {"sysid": sysid, "drone_id": did, "callsign": str(job.get("callsign") or f"UAV-{sysid}"),
               "uav_id": ac[str(did)].get("uav_id")}
        rec.update(_items_record(job.get("items"), where, issues))
        for key in ("camera_params_expected", "airspeed_expected", "airspeed_commands", "expected_photos"):
            rec[key] = _plain(job.get(key), f"{where}.{key}", issues)
        mu[str(sysid)] = rec
        conns[str(sysid)] = _redact_conn(job.get("connection"))
    for did_s, rec in ac.items():
        rec["uploaded_to_sysids"] = sorted(int(s) for s, j in mu.items() if str(j["drone_id"]) == did_s)

    warn = [str(w) for w in warnings]
    for s, j in sorted(mu.items(), key=lambda kv: int(kv[0])):
        if j["mission_items_omitted"]:
            warn.append(f"UAV-{s} 任務項目沒有內嵌：{j['mission_items_omitted']}")

    plan = {
        "planner": _plain(planner, "plan.planner", issues),
        "areas": _areas_section(areas, issues),
        "survey_polygons": _plain(survey_polygons, "plan.survey_polygons", issues),
        "fleet": _plain(fleet, "plan.fleet", issues),
        "aircraft": ac,
        "mission_uploads": mu,
        "mapping": _plain(mapping, "plan.mapping", issues),
        "wind": _plain(wind, "plan.wind", issues),
        "terrain": _plain(terrain, "plan.terrain", issues),
        "takeoff_schedule": _plain(takeoff_schedule, "plan.takeoff_schedule", issues),
        "gcp_blocks": _plain(gcp_blocks, "plan.gcp_blocks", issues),
        "exposure_prediction": _plain(exposure_prediction, "plan.exposure_prediction", issues),
        "params_plan_time": _plain(params_plan_time, "plan.params_plan_time", issues),
        "warnings": warn,
    }
    for k in issues:
        issues[k] = sorted(set(issues[k]))
    plan["data_issues"] = issues

    up_in = dict(upload or {})
    unknown = set(up_in) - _UPLOAD_KEYS
    if unknown:
        raise PlanSnapshotError(f"upload 有不認得的欄位：{sorted(unknown)}")
    up_issues = _new_issues()
    given = up_in.get("aircraft") or {}
    if not isinstance(given, Mapping):
        raise PlanSnapshotError("upload['aircraft'] 要是 {sysid: {...}}")
    extra = {str(_int_id(s, "upload 的 sysid")) for s in given} - set(mu)
    if extra:
        raise PlanSnapshotError(f"upload 的 sysid {sorted(extra)} 不在這批送出的任務裡")
    up_ac = {}
    for s, j in mu.items():
        g = next((v for kk, v in given.items() if str(kk) == s), None) or {}
        ok = g.get("ok")
        if ok is not None and not isinstance(ok, bool):
            raise PlanSnapshotError(f"UAV-{s} 的上傳結果 ok 要是 true／false／null，收到 {ok!r}")
        msg = g.get("message")
        up_ac[s] = {"sysid": j["sysid"], "callsign": j["callsign"], "drone_id": j["drone_id"], "uav_id": j["uav_id"],
                    "connection": _redact_conn(g.get("connection", conns.get(s))), "ok": ok,
                    "message": None if msg is None else str(msg)[:MESSAGE_MAX_CHARS]}
    up = {"aircraft": up_ac,
          "readback_info": _plain(up_in.get("readback_info"), "upload.readback_info", up_issues),
          "skipped_sysids": sorted(_int_id(s, "skipped sysid") for s in (up_in.get("skipped_sysids") or ())),
          "not_targeted_sysids": sorted(_int_id(s, "not_targeted sysid")
                                        for s in (up_in.get("not_targeted_sysids") or ())),
          "tlog": up_in.get("tlog") if isinstance(up_in.get("tlog"), bool) else None}
    if up_issues["unserializable"]:
        raise PlanSnapshotError(f"upload 段有看不懂的物件：{up_issues['unserializable']}")

    sw = software_version() if software is None else dict(software)
    doc = {"format": FORMAT, "written_by": WRITTEN_BY, "uploaded_at_utc": _utc_text(uploaded_at_utc),
           "software": _plain(sw, "software", _new_issues()), "plan": plan, "upload": up}
    # 走一次 JSON 來回：記憶體裡的 dict 與讀回的逐值相同（int 鍵 → 字串、tuple → list 都已處理）
    doc = json.loads(_canonical(doc).decode("utf-8"))
    _seal(doc)
    validate_snapshot(doc, require_upload_ok=False)
    return doc


# ─────────────────────────────────────────────────────────────────────────────
# 從 HARVEST 規劃取資料（唯讀、duck typing；不改 hv／swarm 的任何狀態）
# ─────────────────────────────────────────────────────────────────────────────

def mapping_plan_problem(hv, swarm) -> Optional[str]:
    """不是可做快照的建圖計畫 ⇒ 繁中原因；可以 ⇒ None（與 survey.preview 的判斷同一組條件）。"""
    if hv is None:
        return "沒有 HARVEST 規劃結果"
    if (getattr(hv, "last_ctx", None) is None or getattr(hv, "last_plan", None) is None
            or getattr(hv, "last_inputs", None) is None):
        return "尚未規劃（或規劃已清除）"
    if getattr(hv, "mapping_mode", None) is None:
        return "這份計畫不是航測建圖模式（沒勾「航測建圖」）"
    if swarm is None or getattr(swarm, "hv_mapping_setup", None) is None:
        return "協調器沒有建圖設定（hv_mapping_setup）⇒ 不是建圖計畫，或計畫已被取代"
    owner = getattr(swarm, "hv_coordinator", None)
    if owner is not None and owner is not hv:
        return "協調器的計畫已換成另一份 ⇒ 快照的輸入對不上"
    if getattr(hv, "camera_cfg", None) is None:
        return "這份計畫沒有相機觸發設定（camera_cfg）"
    return None


def _site_record(sites: Mapping, sid) -> dict:
    s = sites.get(sid) if sid else None
    if s is None:
        return {"site_id": sid, "error": "規劃輸入裡找不到這個站點"}
    return {"site_id": getattr(s, "site_id", sid), "lat": getattr(s, "lat_deg", None),
            "lon": getattr(s, "lon_deg", None), "alt_msl_m": getattr(s, "alt_msl_m", None),
            "alt_agl_m": getattr(s, "alt_agl_m", None), "kind": getattr(s, "kind", None)}


_BUILT_PATH_ALT = ("alt_ref_site", "alt_ref_msl_m", "fc_home_site", "fc_home_msl_m", "fc_alt_offset_m", "land_alt_m")


def _strips_of(up, plan, ctx, warn: list, label: str) -> list:
    from pyproj import Geod
    from hv_dccpp.geometry import is_cross_strip
    geod = Geod(ellps="WGS84")
    rows, pts = [], []
    for k, (sid, d) in enumerate(list(getattr(up, "strips", None) or [])):
        try:
            d = int(d)
        except (TypeError, ValueError):
            d = str(d)
        row = {"order": k, "strip_id": str(sid), "direction": d, "cross": bool(is_cross_strip(str(sid)))}
        st = (getattr(plan, "strips", None) or {}).get(sid)
        if st is None:
            row["error"] = "條帶不在 last_plan.strips 裡"
            warn.append(f"{label} 條帶 {sid} 不在 last_plan.strips 裡（快照沒有它的位置）")
            rows.append(row)
            continue
        if d not in (-1, 1) or isinstance(d, str):
            row["error"] = f"方向 {d!r} 不是 −1／+1"
            warn.append(f"{label} 條帶 {sid} 的方向 {d!r} 不是 −1／+1")
            rows.append(row)
            continue
        pL = np.asarray(st.pL, float).reshape(-1)[:2]
        pR = np.asarray(st.pR, float).reshape(-1)[:2]
        a, b = (pL, pR) if d == -1 else (pR, pL)               # dir −1：L→R；+1：R→L（hv_dccpp Strip.start_state）
        row.update({"cls": str(getattr(st, "cls", "")), "length_m": getattr(st, "length_m", None),
                    "width_m": getattr(st, "width_m", None), "spacing_m": getattr(st, "spacing_m", None),
                    "h_op_m": getattr(st, "h_op_m", None), "z_op_m": getattr(st, "z_op_m", None)})
        pts.append((len(rows), a, b))
        rows.append(row)
    if pts:
        enu = np.array([[*p[1], 0.0] for p in pts] + [[*p[2], 0.0] for p in pts], dtype=float)
        ll = np.asarray(ctx.frame.to_wgs84(enu), float)
        n = len(pts)
        for j, (i, _a, _b) in enumerate(pts):
            s_lat, s_lon = float(ll[j, 0]), float(ll[j, 1])
            e_lat, e_lon = float(ll[n + j, 0]), float(ll[n + j, 1])
            if not all(math.isfinite(v) for v in (s_lat, s_lon, e_lat, e_lon)):
                rows[i]["error"] = "端點換算後不是有限數"
                warn.append(f"{label} 條帶 {rows[i]['strip_id']} 端點換算後不是有限數")
                continue
            az, _baz, dist = geod.inv(s_lon, s_lat, e_lon, e_lat)
            rows[i].update({"start": [s_lat, s_lon], "end": [e_lat, e_lon], "heading_deg": float(az) % 360.0,
                            "endpoint_distance_m": float(dist)})
    return rows


def plan_inputs_from_hv(hv, swarm, jobs: Sequence[Mapping], *, takeoff_offsets: Optional[Mapping] = None,
                        takeoff_source: Optional[str] = None) -> dict:
    """``HvDccppCoordinator``＋``SwarmCoordinator``＋這批真的送出的上傳 jobs → :func:`build_snapshot` 的關鍵字引數
    （``upload``、``uploaded_at_utc``、``software`` 除外）。**唯讀**。

    ``jobs``：``_on_hv_mapping_upload`` 建的 dict（``sysid``、``uav_id``＝機號 drone_id、``items``、``expected``、
    ``airspeed_expected``、``speeds``、``link``）；只傳 ``upload_mission_items`` 沒丟例外的那幾筆。
    必要的東西拿不到 ⇒ :class:`PlanSnapshotError`；選填的拿不到 ⇒ 該段記 ``{'error': …}`` 並寫進 ``warnings``。
    """
    reason = mapping_plan_problem(hv, swarm)
    if reason:
        raise PlanSnapshotError(f"不能建立飛行計畫快照：{reason}")
    plan, ctx, inputs = hv.last_plan, hv.last_ctx, hv.last_inputs
    warn: list = []
    id_map = {int(k): str(v) for k, v in dict(getattr(hv, "id_map", None) or {}).items()}
    plans = dict(getattr(plan, "plans", None) or {})
    if not plans:
        raise PlanSnapshotError("不能建立飛行計畫快照：計畫裡沒有任何一架有任務")
    notes = [str(n) for n in (getattr(hv, "notes", None) or [])]
    harvest_params = dict(getattr(swarm, "harvest_params", None) or {})

    # 規劃器
    feas = getattr(plan, "feasibility", None)
    planner = {"name": "HARVEST", "adapter": type(hv).__name__, "hv_dccpp_version": _hv_dccpp_version(),
               "feasibility_status": getattr(feas, "status", None), "makespan_s": getattr(plan, "makespan_s", None),
               "uncovered": [list(u) for u in (getattr(plan, "uncovered", None) or [])], "notes": notes,
               "validation_reasons": getattr(swarm, "hv_validation_reasons", None),
               "mapping_gate": getattr(swarm, "hv_mapping_gate", None)}

    # 作業區（規劃器真的吃進去的那一份，不是地圖上現在的）
    items = []
    for a in (getattr(inputs, "areas", None) or ()):
        items.append({"area_id": str(getattr(a, "area_id", "")),
                      "polygon": [list(p) for p in getattr(a, "polygon_wgs84", ())],
                      "holes": [[list(p) for p in h] for h in (getattr(a, "holes_wgs84", None) or ())]})
    if not items:
        raise PlanSnapshotError("不能建立飛行計畫快照：規劃輸入裡沒有作業區（last_inputs.areas）")
    areas = {"source": "HARVEST 規劃輸入 last_inputs.areas（上傳時的計畫）", "items": items}

    try:
        from mission.swarm_coordinator import _mapping_survey_polygons
        polys, n_nofly = _mapping_survey_polygons(ctx)
        survey_polygons = {"source": "規劃器要蓋的面（作業區聯集 − 禁航區 ⊕ b_trk；與曝光驗證同一個）",
                           "items": polys, "n_nofly": int(n_nofly)}
    except Exception as e:                                       # noqa: BLE001 - 選填：照實記下
        survey_polygons = {"error": f"{type(e).__name__}: {e}"}
        warn.append(f"規劃器測區（作業區−禁航區）取不到：{type(e).__name__}: {e}")

    # 建圖／相機
    setup = getattr(swarm, "hv_mapping_setup", None) or {}
    cc = getattr(hv, "camera_cfg", None)
    setup_cc = setup.get("camera_cfg") if isinstance(setup, Mapping) else None
    i1, i2 = _new_issues(), _new_issues()
    if setup_cc is not None and _plain(setup_cc, "a", i1) != _plain(cc, "a", i2):
        warn.append("規劃協調器的 camera_cfg 與上傳用的 hv.camera_cfg 不一致（快照記的是上傳用的那一份）")
    mapping = {"setup_effective": setup.get("effective"), "setup_warnings": list(setup.get("warnings") or []),
               "camera_hardware": setup.get("camera"), "camera_trigger": cc,
               "mapping_mode": getattr(hv, "mapping_mode", None),
               "mapping_effective": getattr(hv, "mapping_effective", None),
               "max_wind_mps": getattr(hv, "mapping_max_wind_mps", None),
               "camera_trigger_source": "hv.camera_cfg（上傳任務的 206／CAM1_* 用的同一份）"}

    # 風
    try:
        from mission.swarm_coordinator import _mapping_planning_wind
        (w_spd, w_dir), w_src = _mapping_planning_wind(ctx)
        wind = {"planning_speed_mps": w_spd, "planning_dir_from_deg": w_dir, "source": w_src}
    except Exception as e:                                       # noqa: BLE001
        wind = {"source": "取不到", "error": f"{type(e).__name__}: {e}"}
        warn.append(f"規劃風取不到：{type(e).__name__}: {e}")
    wind["layers"] = list(getattr(inputs, "wind", None) or ())
    wind["panel_hv_wind"] = harvest_params.get("hv_wind")

    # 地形／高程基準
    world = (getattr(swarm, "_world_terrain", None) is not None or bool(harvest_params.get("hv_world_terrain"))
             or any(n.startswith("terrain_source(Cesium World Terrain") for n in notes))
    home = getattr(inputs, "home", None)
    terrain = {"vertical_datum": getattr(hv, "vertical_datum", None),
               "geoid_undulation_m": getattr(hv, "geoid_undulation_m", None),
               "world_terrain": bool(world), "dem_step_m": getattr(hv, "dem_step_m", None),
               "planning_home": _site_record({getattr(home, "site_id", None): home}, getattr(home, "site_id", None))
               if home is not None else None,
               "notes": [n for n in notes if n.startswith(("terrain_source(", "terrain_datum(", "site_alt_vs_dem("))]}

    # 每架
    sites = dict(getattr(inputs, "sites", None) or {})
    specs = dict(getattr(ctx, "specs", None) or {})
    paths = dict((getattr(hv, "last_result", None) or {}).get("assembled_paths") or {})
    rev = {u: d for d, u in id_map.items()}
    fleet = [{"drone_id": d, "uav_id": u, "has_mission": u in plans} for d, u in sorted(id_map.items())]
    aircraft = {}
    for uid in sorted(plans, key=str):
        did = rev.get(str(uid))
        if did is None:
            warn.append(f"計畫裡的 {uid} 沒有對應的機號（id_map）⇒ 快照不收這架")
            continue
        up = plans[uid]
        label = f"機 {did}（{uid}）"
        cls = str(getattr(up, "cls", ""))
        bp = paths.get(did, paths.get(str(did)))
        if bp is None:
            alt = {"error": "last_result['assembled_paths'] 沒有這架"}
            warn.append(f"{label} 沒有組裝路徑 ⇒ 快照沒有它的高度基準（fc_alt_offset_m）")
        else:
            alt = {k: getattr(bp, k, None) for k in _BUILT_PATH_ALT}
            alt["note"] = "送給飛控的相對高度 ＝ alt − fc_alt_offset_m（alt 以規劃 home 為基準）"
        aircraft[did] = {"uav_id": str(uid), "cls": cls, "platform_name": getattr(specs.get(cls), "name", None),
                         "takeoff_site": _site_record(sites, getattr(up, "takeoff_site", None)),
                         "landing_site": _site_record(sites, getattr(up, "landing_site", None)),
                         "t_takeoff_s": getattr(up, "t_takeoff_s", None), "t_land_s": getattr(up, "t_land_s", None),
                         "altitude": alt, "strips": _strips_of(up, plan, ctx, warn, label)}
        for which in ("takeoff_site", "landing_site"):
            if "error" in aircraft[did][which]:
                warn.append(f"{label} 的{('起飛' if which == 'takeoff_site' else '降落')}站 "
                            f"{aircraft[did][which]['site_id']!r} 在規劃輸入裡找不到")

    # 這批上傳
    try:
        from hv_dccpp.integration.aeroplan_adapter import expected_photo_count
    except Exception as e:                                       # noqa: BLE001
        expected_photo_count = None
        warn.append(f"預估張數算不了（expected_photo_count 匯入失敗：{type(e).__name__}: {e}）")
    uploads = []
    for k, job in enumerate(jobs or ()):
        if not isinstance(job, Mapping):
            raise PlanSnapshotError(f"上傳 job 第 {k} 筆不是 dict")
        sysid = _int_id(job.get("sysid"), f"上傳 job 第 {k} 筆的 sysid")
        did = _int_id(job.get("uav_id"), f"UAV-{sysid} 的機號（uav_id）")
        if did not in aircraft:
            raise PlanSnapshotError(f"UAV-{sysid} 上傳的是機 {did}，但這份計畫裡沒有它的任務")
        n_exp = None
        if expected_photo_count is not None:
            try:
                n_exp = int(expected_photo_count(job.get("items") or []))
            except Exception as e:                               # noqa: BLE001
                warn.append(f"UAV-{sysid} 預估張數算不了：{type(e).__name__}: {e}")
        link = job.get("link")
        uploads.append({"sysid": sysid, "drone_id": did, "items": job.get("items"),
                        "camera_params_expected": job.get("expected"),
                        "airspeed_expected": job.get("airspeed_expected"),
                        "airspeed_commands": job.get("speeds"), "expected_photos": n_exp,
                        "connection": getattr(link, "conn_str", None) if link is not None else None})
    if not uploads:
        raise PlanSnapshotError("不能建立飛行計畫快照：這批沒有任何一架真的送出任務")

    # 起飛時程
    if takeoff_offsets is None:
        takeoff = {"error": "呼叫端沒有提供起飛時程（harvest_takeoff_offsets）"}
        warn.append("沒有起飛時程 ⇒ 後處理無法核對錯開起飛")
    else:
        takeoff = {"source": str(takeoff_source or "未說明"),
                   "offsets_s": {int(d): float(s) for d, s in dict(takeoff_offsets).items()}}

    # GCP 區塊（GCP／檢核點規劃對話框的格式）
    try:
        from survey import gcp_plan
        blocks, info = gcp_plan.blocks_from_hv(hv)
        gcp = {"source": "survey.gcp_plan.blocks_from_hv（上傳時的計畫）", "blocks": blocks, "info": info}
    except Exception as e:                                       # noqa: BLE001 - 取不到要照實說，不默默改成不分區塊
        gcp = {"source": "survey.gcp_plan.blocks_from_hv（上傳時的計畫）", "error": f"{type(e).__name__}: {e}"}
        warn.append(f"GCP 區塊取不到：{type(e).__name__}: {e}")

    params = {}
    excluded = []
    for k in sorted(harvest_params, key=str):
        ks = str(k)
        if not (ks.startswith("hv_") or ks in _PARAM_KEYS):
            continue
        if _SECRETISH.search(ks):
            excluded.append(ks)
            continue
        v = harvest_params[k]
        if v is None or isinstance(v, (bool, int, float, str, np.integer, np.floating)) or (
                isinstance(v, (list, tuple)) and all(isinstance(x, (bool, int, float, str, type(None))) for x in v)):
            params[ks] = v
        else:
            excluded.append(f"{ks}（{type(v).__name__}）")
    params_rec = {"source": "SwarmCoordinator.harvest_params（規劃當時的面板值；白名單）", "values": params,
                  "not_recorded": excluded}

    return {"planner": planner, "areas": areas, "aircraft": aircraft, "mission_uploads": uploads,
            "mapping": mapping, "wind": wind, "gcp_blocks": gcp, "terrain": terrain,
            "survey_polygons": survey_polygons, "takeoff_schedule": takeoff, "fleet": fleet,
            "exposure_prediction": getattr(swarm, "hv_exposure_summary", None), "params_plan_time": params_rec,
            "warnings": warn}


def snapshot_from_hv(hv, swarm, jobs: Sequence[Mapping], *, takeoff_offsets: Optional[Mapping] = None,
                     takeoff_source: Optional[str] = None, readback_info: Optional[Mapping] = None,
                     skipped: Sequence = (), not_targeted: Sequence = (), tlog: Optional[bool] = None,
                     uploaded_at_utc=None, software: Optional[Mapping] = None) -> dict:
    """主視窗用：:func:`plan_inputs_from_hv` ＋ :func:`build_snapshot`。所有架的 ACK 結果先是 ``None``（未回報）。"""
    kw = plan_inputs_from_hv(hv, swarm, jobs, takeoff_offsets=takeoff_offsets, takeoff_source=takeoff_source)
    upload = {"readback_info": readback_info, "skipped_sysids": list(skipped or ()),
              "not_targeted_sysids": list(not_targeted or ()), "tlog": tlog}
    return build_snapshot(**kw, upload=upload, uploaded_at_utc=uploaded_at_utc, software=software)


# ─────────────────────────────────────────────────────────────────────────────
# 上傳結果
# ─────────────────────────────────────────────────────────────────────────────

def set_upload_status(snap: Mapping, sysid, *, ok: bool, message: Optional[str] = None) -> dict:
    """某架的 ACK 結果 → 新的快照 dict（原 dict 不動；digest 重算、plan_sha256 不變）。

    同一架已記錄過不同的結果 ⇒ 拒絕（那是另一次上傳，不能改寫這一份的紀錄）；相同結果 ⇒ 原樣回傳。
    """
    if not isinstance(ok, bool):
        raise PlanSnapshotError(f"上傳結果 ok 要是 True／False，收到 {ok!r}")
    s = str(_int_id(sysid, "sysid"))
    new = copy.deepcopy(dict(snap))
    rec = (new.get("upload") or {}).get("aircraft", {}).get(s)
    if rec is None:
        raise PlanSnapshotError(f"UAV-{s} 不在這份快照的上傳清單裡")
    msg = None if message is None else str(message)[:MESSAGE_MAX_CHARS]
    if rec.get("ok") is not None:
        if rec.get("ok") == ok and rec.get("message") == msg:
            return new
        raise PlanSnapshotError(f"UAV-{s} 的上傳結果已記錄為 ok={rec.get('ok')}（{rec.get('message')}），"
                                f"不改寫成 ok={ok}（{msg}）：那是另一次上傳")
    rec["ok"], rec["message"] = ok, msg
    return _seal(new)


def any_upload_ok(snap: Mapping) -> bool:
    return any(r.get("ok") is True for r in ((snap.get("upload") or {}).get("aircraft") or {}).values())


# ─────────────────────────────────────────────────────────────────────────────
# 驗證
# ─────────────────────────────────────────────────────────────────────────────

def _check_ring(ring, where: str) -> None:
    from survey.pipeline import PipelineError, normalize_polygons
    try:
        normalize_polygons(polygons=[ring])
    except PipelineError as e:
        raise PlanSnapshotError(f"{where}不是合法的多邊形：{e}") from None


def validate_snapshot(snap, *, require_upload_ok: bool = True) -> dict:
    """嚴格驗證（不改內容），通過就原樣回傳。任何不符 ⇒ :class:`PlanSnapshotError`（繁中、說哪一欄）。"""
    if not isinstance(snap, dict):
        raise PlanSnapshotError(f"快照要是 JSON 物件，收到 {type(snap).__name__}")
    fmt = snap.get("format")
    if fmt != FORMAT:
        if isinstance(fmt, str) and fmt.startswith(FORMAT_FAMILY + "/"):
            raise PlanSnapshotError(f"快照格式版本 {fmt!r} 不支援（這版只讀 {FORMAT!r}）")
        raise PlanSnapshotError(f"這不是飛行計畫快照（format={fmt!r}，要 {FORMAT!r}）")
    missing = sorted(_TOP_KEYS - set(snap))
    if missing:
        raise PlanSnapshotError(f"快照缺少欄位：{missing}")
    extra = sorted(set(snap) - _TOP_KEYS)
    if extra:
        raise PlanSnapshotError(f"快照有不認得的欄位：{extra}")
    dg = snap.get("digest_sha256")
    if not isinstance(dg, str) or not _HEX64.fullmatch(dg):
        raise PlanSnapshotError("digest_sha256 不是 64 位小寫十六進位")
    try:
        actual = content_digest(snap)
    except (TypeError, ValueError) as e:
        raise PlanSnapshotError(f"快照內容無法正規化（{e}）") from None
    if actual != dg:
        raise PlanSnapshotError(f"快照內容被改過或不完整：digest 不符（檔內 {dg[:12]}…，實算 {actual[:12]}…）")
    ps = snap.get("plan_sha256")
    if not isinstance(ps, str) or ps != _sha256(snap.get("plan")):
        raise PlanSnapshotError("plan_sha256 與 plan 段內容不符")
    if snap.get("written_by") != WRITTEN_BY:
        raise PlanSnapshotError(f"written_by 要是 {WRITTEN_BY!r}，收到 {snap.get('written_by')!r}")
    _parse_utc(snap.get("uploaded_at_utc"))
    if not isinstance(snap.get("software"), dict):
        raise PlanSnapshotError("software 要是物件")

    plan = snap["plan"]
    if not isinstance(plan, dict):
        raise PlanSnapshotError("plan 要是物件")
    missing = sorted(_PLAN_KEYS - set(plan))
    if missing:
        raise PlanSnapshotError(f"plan 缺少欄位：{missing}")
    pl = plan["planner"]
    if not isinstance(pl, dict) or not isinstance(pl.get("name"), str) or not pl["name"].strip():
        raise PlanSnapshotError("plan.planner 沒有規劃器名稱")
    areas = plan["areas"]
    if (not isinstance(areas, dict) or not isinstance(areas.get("source"), str)
            or not isinstance(areas.get("items"), list) or not areas["items"]):
        raise PlanSnapshotError("plan.areas 要有 source 與至少一塊作業區")
    for k, a in enumerate(areas["items"], start=1):
        if not isinstance(a, dict) or not isinstance(a.get("area_id"), str):
            raise PlanSnapshotError(f"第 {k} 塊作業區格式不對（要有 area_id、polygon、holes）")
        _check_ring(a.get("polygon"), f"作業區 {a['area_id']} ")
        holes = a.get("holes")
        if not isinstance(holes, list):
            raise PlanSnapshotError(f"作業區 {a['area_id']} 的 holes 要是串列")
        for j, h in enumerate(holes, start=1):
            _check_ring(h, f"作業區 {a['area_id']} 第 {j} 個內洞")
    ac = plan["aircraft"]
    if not isinstance(ac, dict) or not ac:
        raise PlanSnapshotError("plan.aircraft 沒有任何一架")
    for key, rec in ac.items():
        if not isinstance(rec, dict) or str(rec.get("drone_id")) != key:
            raise PlanSnapshotError(f"plan.aircraft[{key!r}] 的 drone_id 對不上")
        if not isinstance(rec.get("uav_id"), str) or not isinstance(rec.get("cls"), str):
            raise PlanSnapshotError(f"機 {key} 缺少 uav_id／cls")
        if not isinstance(rec.get("strips"), list):
            raise PlanSnapshotError(f"機 {key} 的 strips 要是串列")
    mu = plan["mission_uploads"]
    if not isinstance(mu, dict) or not mu:
        raise PlanSnapshotError("plan.mission_uploads 沒有任何一架")
    for key, rec in mu.items():
        if not isinstance(rec, dict) or str(rec.get("sysid")) != key:
            raise PlanSnapshotError(f"plan.mission_uploads[{key!r}] 的 sysid 對不上")
        if str(rec.get("drone_id")) not in ac:
            raise PlanSnapshotError(f"UAV-{key} 上傳的機 {rec.get('drone_id')} 不在 plan.aircraft 裡")
        n, sha, items = rec.get("n_items"), rec.get("mission_items_sha256"), rec.get("mission_items")
        if isinstance(n, bool) or not isinstance(n, int) or n < 1:
            raise PlanSnapshotError(f"UAV-{key} 的 n_items 不對：{n!r}")
        if not isinstance(sha, str) or not _HEX64.fullmatch(sha):
            raise PlanSnapshotError(f"UAV-{key} 的 mission_items_sha256 不對")
        if items is None:
            if not rec.get("mission_items_omitted"):
                raise PlanSnapshotError(f"UAV-{key} 沒有任務項目，也沒有說明為什麼省略")
        elif not isinstance(items, list) or len(items) != n or _sha256(items) != sha:
            raise PlanSnapshotError(f"UAV-{key} 的任務項目與 n_items／sha256 不符")
    for key in ("mapping", "wind", "gcp_blocks", "data_issues"):
        if not isinstance(plan[key], dict):
            raise PlanSnapshotError(f"plan.{key} 要是物件")
    if not isinstance(plan["wind"].get("source"), str):
        raise PlanSnapshotError("plan.wind 沒有來源說明（source）")
    if not isinstance(plan["warnings"], list) or not all(isinstance(w, str) for w in plan["warnings"]):
        raise PlanSnapshotError("plan.warnings 要是字串串列")
    g = plan["gcp_blocks"]
    if not isinstance(g.get("source"), str):
        raise PlanSnapshotError("plan.gcp_blocks 沒有來源說明（source）")
    if "error" in g:
        if not isinstance(g["error"], str) or "blocks" in g:
            raise PlanSnapshotError("plan.gcp_blocks 的 error 要是字串，且不能同時有 blocks")
    else:
        if not isinstance(g.get("blocks"), dict) or not isinstance(g.get("info"), dict):
            raise PlanSnapshotError("plan.gcp_blocks 要有 blocks 與 info（或 error）")
        from survey.gcp_plan import GcpPlanError, normalize_blocks
        try:
            normalize_blocks(g["blocks"])
        except GcpPlanError as e:
            raise PlanSnapshotError(f"plan.gcp_blocks 的區塊不合法：{e}") from None

    up = snap["upload"]
    if not isinstance(up, dict) or set(up) != _UPLOAD_KEYS:
        raise PlanSnapshotError(f"upload 段的欄位要剛好是 {sorted(_UPLOAD_KEYS)}")
    ua = up["aircraft"]
    if not isinstance(ua, dict) or set(ua) != set(mu):
        raise PlanSnapshotError("upload.aircraft 的 sysid 要與 plan.mission_uploads 完全相同")
    for key, rec in ua.items():
        if not isinstance(rec, dict) or rec.get("ok") not in (None, True, False):
            raise PlanSnapshotError(f"UAV-{key} 的上傳結果 ok 要是 true／false／null")
        if rec.get("message") is not None and not isinstance(rec.get("message"), str):
            raise PlanSnapshotError(f"UAV-{key} 的上傳訊息要是字串")
    if require_upload_ok and not any_upload_ok(snap):
        raise PlanSnapshotError("這份快照沒有任何一架回報上傳成功（ok=true）⇒ 不是「已飛計畫」")
    return snap


# ─────────────────────────────────────────────────────────────────────────────
# 寫檔／讀檔
# ─────────────────────────────────────────────────────────────────────────────

def snapshot_path(root, snap: Mapping) -> Path:
    """第一個候選檔名：``<root>/<YYYYMMDD>/flown_plan_<HHMMSS>_<plan_sha8>.json``（本機時間；同名時 write 會加 _2…）。"""
    t = _parse_utc(snap.get("uploaded_at_utc")).astimezone()
    return Path(root) / t.strftime("%Y%m%d") / f"{FILENAME_PREFIX}{t.strftime('%H%M%S')}_{str(snap['plan_sha256'])[:8]}.json"


def _tmp_write(dest_dir: Path, name: str, data: bytes) -> Path:
    tmp = dest_dir / f".{name}.{os.getpid()}.{uuid.uuid4().hex[:8]}.tmp"
    with open(tmp, "xb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    return tmp


def _place_noreplace(tmp: Path, dest: Path) -> None:
    """原子放置且不覆蓋：Windows 的 rename 遇到已存在就 FileExistsError；POSIX 用 link（同樣不覆蓋）。"""
    if os.name == "nt":
        os.rename(tmp, dest)
    else:
        os.link(tmp, dest)
        os.unlink(tmp)


def write_snapshot(root, snap: Mapping) -> Path:
    """原子寫入、**永不覆蓋**（同名加 ``_2``、``_3``…）。寫之前完整驗證，且至少一架上傳 ok=True。回傳實際路徑。"""
    validate_snapshot(snap, require_upload_ok=True)
    data = _file_bytes(snap)
    first = snapshot_path(root, snap)
    first.parent.mkdir(parents=True, exist_ok=True)
    tmp = _tmp_write(first.parent, first.name, data)
    try:
        for n in range(1, MAX_COLLISIONS + 1):
            dest = first if n == 1 else first.with_name(f"{first.stem}_{n}{first.suffix}")
            try:
                _place_noreplace(tmp, dest)
                return dest
            except FileExistsError:
                continue
        raise PlanSnapshotError(f"{first.parent} 裡同名快照已有 {MAX_COLLISIONS} 個，不再嘗試")
    finally:
        if tmp.exists():
            try:
                tmp.unlink()
            except OSError:
                pass


def _load(path) -> dict:
    p = Path(path)
    try:
        raw = p.read_bytes()
    except OSError as e:
        raise PlanSnapshotError(f"讀不到飛行計畫快照 {p.name}：{e}") from None
    try:
        return json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        raise PlanSnapshotError(f"{p.name} 不是合法的 UTF-8 JSON：{e}") from None


def read_snapshot(path, *, require_upload_ok: bool = True) -> dict:
    """讀＋嚴格驗證（格式版本、digest、plan_sha256、必要欄位、多邊形、GCP 區塊、至少一架上傳成功）。"""
    return validate_snapshot(_load(path), require_upload_ok=require_upload_ok)


def update_upload_status(path, sysid, *, ok: bool, message: Optional[str] = None) -> dict:
    """已寫出的快照收到另一架的 ACK ⇒ 就地更新同一個檔（只改 upload 段；原子取代）。回傳更新後的 dict。"""
    p = Path(path)
    old = read_snapshot(p, require_upload_ok=False)
    new = set_upload_status(old, sysid, ok=ok, message=message)
    if new["plan_sha256"] != old["plan_sha256"]:
        raise PlanSnapshotError("更新上傳結果時 plan 段變了（不該發生）")
    if new == old:
        return new
    validate_snapshot(new, require_upload_ok=True)
    tmp = _tmp_write(p.parent, p.name, _file_bytes(new))
    try:
        os.replace(tmp, p)
    finally:
        if tmp.exists():
            try:
                tmp.unlink()
            except OSError:
                pass
    return new


# ─────────────────────────────────────────────────────────────────────────────
# 給對話框用
# ─────────────────────────────────────────────────────────────────────────────

def source_label(snap: Mapping, path=None) -> str:
    name = Path(path).name if path is not None else "（未存檔）"
    return f"飛行計畫快照 {name}（plan {str(snap.get('plan_sha256'))[:12]}，上傳 {snap.get('uploaded_at_utc')}）"


def snapshot_polygons(snap: Mapping) -> list:
    """作業區外框 → ``[[(lat, lon), …], …]``（``survey.pipeline.normalize_polygons(polygons=…)`` 與
    ``MainWindow.survey_area_polygons()`` 同形狀）。內洞不在這裡（數量見 :func:`snapshot_polygon_holes`）。"""
    return [[(float(p[0]), float(p[1])) for p in a["polygon"]] for a in snap["plan"]["areas"]["items"]]


def snapshot_polygon_holes(snap: Mapping) -> int:
    return sum(len(a.get("holes") or []) for a in snap["plan"]["areas"]["items"])


def polygons_label(snap: Mapping, path=None) -> str:
    n = len(snap["plan"]["areas"]["items"])
    h = snapshot_polygon_holes(snap)
    txt = f"{source_label(snap, path)}：作業區 {n} 塊"
    if h:
        txt += f"（另有 {h} 個內洞沒有納入：這裡只用外框）"
    return txt


def snapshot_gcp_blocks(snap: Mapping, path=None) -> tuple:
    """GCP 區塊 → ``(blocks {標籤: [ring…]}, info, 來源說明)``，與 ``gcp_blocks_provider`` 的回傳同形狀
    （ring 是 (lat, lon) tuple 的串列）。快照記的是錯誤 ⇒ :class:`PlanSnapshotError`（對話框會照實顯示）。"""
    g = snap["plan"]["gcp_blocks"]
    if "error" in g:
        raise PlanSnapshotError(f"快照裡沒有 GCP 區塊（上傳時就取不到：{g['error']}）")
    blocks = {str(k): [[(float(p[0]), float(p[1])) for p in ring] for ring in rings]
              for k, rings in sorted(g["blocks"].items())}
    info = copy.deepcopy(g["info"])
    names = "、".join(f"{k}（#{(v or {}).get('drone_id')}）" for k, v in sorted(info.items()))
    return blocks, info, f"{source_label(snap, path)}：{len(blocks)} 架 {names}"


def polygons_provider(snap: Mapping) -> Callable[[], list]:
    """``SurveyProcessDialog(polygons=…)``／``GcpPlanDialog(polygons=…)`` 吃的可呼叫物件。"""
    frozen = copy.deepcopy(dict(snap))
    return lambda: snapshot_polygons(frozen)


def blocks_provider(snap: Mapping, path=None) -> Callable[[], tuple]:
    """``GcpPlanDialog(blocks=…)`` 吃的可呼叫物件；快照記的是錯誤時呼叫會丟 :class:`PlanSnapshotError`。"""
    frozen = copy.deepcopy(dict(snap))
    return lambda: snapshot_gcp_blocks(frozen, path)


def source_record(snap: Mapping, path=None) -> dict:
    """寫進後處理有效值（input_sources）的來源紀錄：檔名（不放絕對路徑）＋雜湊＋上傳時間。"""
    up = snap["upload"]["aircraft"]
    return {"kind": "plan_snapshot", "format": snap["format"],
            "file": Path(path).name if path is not None else None,
            "plan_sha256": snap["plan_sha256"], "digest_sha256": snap["digest_sha256"],
            "uploaded_at_utc": snap["uploaded_at_utc"],
            "n_areas": len(snap["plan"]["areas"]["items"]), "n_area_holes_ignored": snapshot_polygon_holes(snap),
            "upload_ok_sysids": sorted(int(s) for s, r in up.items() if r.get("ok") is True),
            "upload_failed_sysids": sorted(int(s) for s, r in up.items() if r.get("ok") is False),
            "upload_unreported_sysids": sorted(int(s) for s, r in up.items() if r.get("ok") is None)}


def summary_lines(snap: Mapping, path=None) -> list:
    """對話框標籤用的幾行摘要（繁中）。"""
    plan, up = snap["plan"], snap["upload"]["aircraft"]

    def st(r):
        return "✔" if r.get("ok") is True else ("✘" if r.get("ok") is False else "未回報")
    lines = [source_label(snap, path),
             f"規劃器 {plan['planner'].get('name')}（{plan['planner'].get('feasibility_status')}）；"
             f"作業區 {len(plan['areas']['items'])} 塊；計畫 {len(plan['aircraft'])} 架",
             "上傳：" + "、".join(f"{r.get('callsign')}←機 {r.get('drone_id')} {st(r)}"
                                for _s, r in sorted(up.items(), key=lambda kv: int(kv[0])))]
    g = plan["gcp_blocks"]
    lines.append(f"GCP 區塊：{len(g['blocks'])} 架" if "error" not in g else f"GCP 區塊：✘ {g['error']}")
    if plan["warnings"]:
        lines.append(f"⚠ 快照警告 {len(plan['warnings'])} 條：" + "；".join(plan["warnings"][:3])
                     + ("…" if len(plan["warnings"]) > 3 else ""))
    di = plan["data_issues"]
    if di.get("unserializable"):
        lines.append(f"⚠ 有 {len(di['unserializable'])} 個欄位無法序列化（見 data_issues）")
    return lines
