"""飛控 DataFlash .BIN → 航測 ingest 用的紀錄表。

只用 ``pymavlink.DFReader.DFReader_binary`` 直接讀檔（不走 ``mavutil.mavlink_connection``：
那條路在測試環境被擋，而且讀檔本來就不需要連線物件）。

讀出的訊息與用途
================
* CAM / TRIG：拍照事件。沒設回授腳時 CAM 在 ``trigger_pic`` 之後同一個 tick 寫入，
  時刻 ≈ GPIO 邊緣（µs 級，Plane-4.5.7 AP_Camera_Backend.cpp:141-147）；設了回授腳時
  TRIG 記指令時刻、CAM 記回授中斷時刻（同檔 :399-415）。兩種情況下 **CAM 都是邊緣事件**。
* GPS / GPA：UTC 換算（GWk、GMS）、接收機大地起伏（GPA.Und 或 GPA.AEl，見 datum.py）、精度。
* ATT、POS、XKF1（+ ORGN）：曝光時刻的位置、姿態、速度內插來源。
* MISE / MSG / CMD：任務執行事件 ⇒ 每條帶 206 開／關的時間窗。
  ⚠ 實測本機 SITL 日誌（4.8.0-dev）的 MISE 大量缺漏（i100 log 19 只有 21 筆，GCS 文字
  「Mission: N …」有 82 筆），CMD 只是解鎖時整份任務的傾印（83 筆同一個 TimeUS）。
  所以執行時刻取 MISE ∪ MSG，參數再用 CNum 回查 CMD 傾印。
* MODE、PARM、MSG／VER：模式、參數、韌體字串。
* XKF2／NKF2（＋XKF4／NKF4 的 PI）：EKF3／EKF2 的風估計 VWN／VWE（m/s，空氣往北／東的速度）與主要核心，
  給現場 QA 的風速檢查（survey.qa.wind_report）。2026-10-15 在本機 SITL 日誌 00000021／22.BIN 核對：
  XKF2 格式 'QBccccchhhhhhfff'，VWN／VWE 是 'c'（DFReader 乘 0.01）、FMTU 'm/s'；兩個核心 C=0／1，
  XKF4.PI = 0；SIM_WIND_DIR 270（風從西來）時 VWE ≈ +5 m/s。

角度單位（CAM/TRIG 的 R/P/Y 在 ≤4.6 是 int16 百分度 'c'/'C'，≥4.7 是 float 度 'f'；
4.6.3 vs 4.7.2 libraries/AP_Camera/LogStructure.h 的 'QBHIHLLeeeccC' / 'QBHIHLLeeefff'）：
DFReader 對 'c'/'C' 會自動乘 0.01，因此兩種格式讀出來都是「度」。本模組不靠版本號，
而是逐欄檢查該檔 FMT 的格式字元與 DFReader 的乘數；遇到認不得的格式字元就拒絕。
"""

from __future__ import annotations

import logging
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from pymavlink import DFReader

_log = logging.getLogger(__name__)

_CAM_COLS = ("TimeUS", "I", "Img", "GPSTime", "GPSWeek", "Lat", "Lng", "Alt", "RelAlt", "GPSAlt", "R", "P", "Y")
_CMD_COLS = ("TimeUS", "CTot", "CNum", "CId", "Prm1", "Prm2", "Prm3", "Prm4", "Lat", "Lng", "Alt")
# 每種訊息要取的欄位（該檔 FMT 沒有的欄位就略過；GPA 的 Und／AEl 依版本二擇一）
WANTED_COLUMNS = {
    "CAM": _CAM_COLS,
    "TRIG": _CAM_COLS,
    "GPS": ("TimeUS", "I", "Status", "GMS", "GWk", "NSats", "HDop", "Lat", "Lng", "Alt", "Spd", "GCrs", "VZ"),
    "GPA": ("TimeUS", "I", "VDop", "HAcc", "VAcc", "SAcc", "Und", "AEl"),
    "ATT": ("TimeUS", "Roll", "Pitch", "Yaw"),
    "POS": ("TimeUS", "Lat", "Lng", "Alt"),
    "XKF1": ("TimeUS", "C", "Roll", "Pitch", "Yaw", "VN", "VE", "VD", "PN", "PE", "PD"),
    "ORGN": ("TimeUS", "Type", "Lat", "Lng", "Alt"),
    "MODE": ("TimeUS", "Mode", "ModeNum"),
    "CMD": _CMD_COLS,
    "MISE": _CMD_COLS,
    "XKF2": ("TimeUS", "C", "VWN", "VWE"),
    "XKF4": ("TimeUS", "C", "PI"),
    "NKF2": ("TimeUS", "C", "VWN", "VWE"),
    "NKF4": ("TimeUS", "C", "PI"),
}
# 要做單位檢查的角度欄
ANGLE_COLUMNS = {
    "CAM": ("R", "P", "Y"),
    "TRIG": ("R", "P", "Y"),
    "ATT": ("Roll", "Pitch", "Yaw"),
    "XKF1": ("Roll", "Pitch", "Yaw"),
}
CMD_DO_SET_CAM_TRIGG_DIST = 206

_FW_RE = re.compile(r"^(Ardu\w+)\s+V(\d+)\.(\d+)\.(\d+)")
_MISSION_MSG_RE = re.compile(r"^Mission: (\d+) ")


@dataclass
class DFLog:
    """讀出來的紀錄表。``tables[name][col]`` 是 numpy 陣列（依日誌順序）。"""
    path: str
    tables: dict
    formats: dict                       # name -> (format 字串, 欄位 tuple)
    angle_encoding: dict                # name -> 'int16_centideg' | 'float_deg'
    params: dict                        # 第一次出現的參數值
    param_changes: list                 # [(TimeUS, name, value)]：之後改過的值
    messages: list                      # [(TimeUS, text)]
    firmware: Optional[str]
    # name -> {欄: (FMT 格式字元, DFReader 乘數, FMTU 單位或 None)}：讀出來的欄位怎麼解碼（單位檢查用）
    column_specs: dict = field(default_factory=dict)

    def table(self, name: str) -> dict:
        return self.tables.get(name) or {}

    def count(self, name: str) -> int:
        t = self.tables.get(name)
        return 0 if not t else len(t["TimeUS"])

    @property
    def firmware_version(self):
        """(major, minor, patch) 或 None。"""
        if not self.firmware:
            return None
        m = _FW_RE.match(self.firmware)
        return (int(m.group(2)), int(m.group(3)), int(m.group(4))) if m else None

    @property
    def time_range_us(self):
        lo, hi = None, None
        for t in self.tables.values():
            ts = t.get("TimeUS")
            if ts is not None and len(ts):
                lo = int(ts.min()) if lo is None else min(lo, int(ts.min()))
                hi = int(ts.max()) if hi is None else max(hi, int(ts.max()))
        return lo, hi


def _angle_encoding(fmt, cols) -> str:
    """依 FMT 判斷角度欄的編碼；回傳 'int16_centideg' 或 'float_deg'（兩者經 DFReader 後都是度）。"""
    encs = set()
    for c in cols:
        if c not in fmt.colhash:
            continue
        i = fmt.colhash[c]
        ch = fmt.format[i]
        mult = fmt.msg_mults[i]
        if ch in "cC":
            # DFReader 對 'c'/'C' 乘 0.01（FORMAT_TO_STRUCT）；乘數不對就不知道單位 ⇒ 拒絕
            if mult is None or not math.isclose(mult, 0.01):
                raise ValueError(f"{fmt.name}.{c} 格式 {ch!r} 的 DFReader 乘數是 {mult}，預期 0.01")
            encs.add("int16_centideg")
        elif ch in "fd":
            if mult is not None:
                raise ValueError(f"{fmt.name}.{c} 格式 {ch!r} 不該有乘數，DFReader 給了 {mult}")
            encs.add("float_deg")
        else:
            raise ValueError(f"{fmt.name}.{c} 格式字元 {ch!r} 無法判讀角度單位（預期 c/C 百分度或 f 度）")
        units = getattr(fmt, "units", None)
        if units and units[i] and not units[i].startswith("deg"):
            raise ValueError(f"{fmt.name}.{c} 的 FMTU 單位是 {units[i]!r}，不是度")
    if len(encs) > 1:
        raise ValueError(f"{fmt.name} 的角度欄混用 {sorted(encs)}")
    return encs.pop() if encs else "none"


# 預設不讀 EKF2 的 NKF2／NKF4：EKF3 機（AHRS_EKF_TYPE=3）用不到，而 SITL 兩個都記（00000022.BIN 各 4.7 萬筆，
# 多讀一倍時間）。EKF2 機的風檢查由 survey.qa.wind_report 用 types=EKF2_WIND_TYPES 另讀一次。
EKF2_WIND_TYPES = ("NKF2", "NKF4")
DEFAULT_TYPES = tuple(k for k in WANTED_COLUMNS if k not in EKF2_WIND_TYPES)


def read_dflog(path, types=None) -> DFLog:
    """讀 .BIN。``types`` 預設為 ``DEFAULT_TYPES``（``WANTED_COLUMNS`` 扣掉 NKF2／NKF4）加 MSG、PARM、VER。"""
    path = str(Path(path))
    want = ({t: WANTED_COLUMNS[t] for t in DEFAULT_TYPES} if types is None
            else {t: WANTED_COLUMNS[t] for t in types if t in WANTED_COLUMNS})
    reader = DFReader.DFReader_binary(path, zero_time_base=False)
    try:
        by_name = {f.name: f for f in reader.formats.values()}
        cols = {}
        formats = {}
        specs = {}
        angle_enc = {}
        for name, wanted in want.items():
            fmt = by_name.get(name)
            if fmt is None:
                continue
            cols[name] = tuple(c for c in wanted if c in fmt.colhash)
            formats[name] = (fmt.format, tuple(fmt.columns))
            units = getattr(fmt, "units", None)
            specs[name] = {c: (fmt.format[fmt.colhash[c]], fmt.msg_mults[fmt.colhash[c]],
                               (units[fmt.colhash[c]] or None) if units else None) for c in cols[name]}
            if name in ANGLE_COLUMNS:
                angle_enc[name] = _angle_encoding(fmt, ANGLE_COLUMNS[name])
        rows = {name: [] for name in cols}
        params, param_changes, messages = {}, [], []
        current = {}
        ver_fws = None
        types_to_read = list(cols) + ["MSG", "PARM", "VER"]
        while True:
            m = reader.recv_match(type=types_to_read)
            if m is None:
                break
            t = m.get_type()
            if t in rows:
                rows[t].append(tuple(getattr(m, c) for c in cols[t]))
            elif t == "PARM":
                name, value = m.Name, float(m.Value)
                if name not in params:
                    params[name] = value
                elif value != current[name]:
                    param_changes.append((int(m.TimeUS), name, value))
                current[name] = value
            elif t == "MSG":
                messages.append((int(m.TimeUS), str(m.Message)))
            elif t == "VER" and ver_fws is None:
                ver_fws = str(getattr(m, "FWS", "") or "") or None
    finally:
        try:
            reader.data_map.close()
            reader.filehandle.close()
        except Exception:   # 關檔失敗不影響已讀出的資料
            pass
    tables = {}
    for name, rs in rows.items():
        if not rs:
            continue
        arr = list(zip(*rs))
        tables[name] = {c: np.asarray(v) for c, v in zip(cols[name], arr)}
    firmware = ver_fws
    if firmware is None:
        for _, text in messages:
            if _FW_RE.match(text):
                firmware = text.strip()
                break
    return DFLog(path=path, tables=tables, formats=formats, angle_encoding=angle_enc,
                 params=params, param_changes=param_changes, messages=messages, firmware=firmware,
                 column_specs=specs)


# ── 拍照事件 ────────────────────────────────────────────────────────────

def cam_events(log: DFLog, kind: str = "CAM", instance: Optional[int] = 0) -> dict:
    """CAM（或 TRIG）事件，依 TimeUS 排序。``instance=None`` 表示不篩相機編號。

    同一份日誌出現多個相機編號而呼叫端沒指定時拒絕 —— 不替使用者挑。
    """
    if kind not in ("CAM", "TRIG"):
        raise ValueError(f"kind 必須是 CAM 或 TRIG，收到 {kind!r}")
    t = log.table(kind)
    if not t:
        return {}
    sel = np.ones(len(t["TimeUS"]), bool)
    if "I" in t:
        insts = sorted(set(np.asarray(t["I"]).tolist()))
        if instance is None:
            if len(insts) > 1:
                raise ValueError(f"{kind} 有多個相機編號 {insts}，請指定 instance")
        else:
            sel = np.asarray(t["I"]) == instance
    order = np.argsort(np.asarray(t["TimeUS"])[sel], kind="stable")
    return {c: np.asarray(v)[sel][order] for c, v in t.items()}


# ── 任務執行與作業帶時間窗 ─────────────────────────────────────────────

@dataclass(frozen=True)
class MissionExec:
    t_us: int
    cnum: int
    cid: Optional[int]
    prm1: Optional[float]
    source: str                   # 'MISE' / 'MSG'


@dataclass(frozen=True)
class StripWindow:
    """一條作業帶：206(d>0) 執行起、206(0) 執行止（含起點，不含終點）。"""
    strip: int                    # 1 起算
    t_on_us: int
    t_off_us: int                 # 沒關就到日誌結束（旗標 'unclosed'）
    trigg_dist_m: float
    cnum_on: int
    cnum_off: Optional[int]
    flags: tuple = ()

    def contains(self, t_us) -> bool:
        return self.t_on_us <= t_us < self.t_off_us


def mission_executions(log: DFLog) -> list:
    """任務項目實際開始執行的時刻：MISE ∪ GCS 文字「Mission: N …」，參數用 CMD 傾印補。"""
    cmd = log.table("CMD")
    dump = {}
    if cmd:
        for i in range(len(cmd["TimeUS"])):
            dump.setdefault(int(cmd["CNum"][i]), []).append(
                (int(cmd["TimeUS"][i]), int(cmd["CId"][i]), float(cmd["Prm1"][i])))

    def lookup(cnum, t_us):
        """執行當下有效的任務項目：t 之前最後一次傾印；任務在執行後才傾印的極端情況取最早一筆。"""
        cands = sorted(dump.get(cnum, ()))
        if not cands:
            return None, None
        before = [c for c in cands if c[0] <= t_us]
        _, cid, prm1 = before[-1] if before else cands[0]
        return cid, prm1

    out = []
    mise = log.table("MISE")
    seen = set()
    if mise:
        for i in range(len(mise["TimeUS"])):
            t, cnum = int(mise["TimeUS"][i]), int(mise["CNum"][i])
            out.append(MissionExec(t, cnum, int(mise["CId"][i]), float(mise["Prm1"][i]), "MISE"))
            seen.add((cnum, t // 1000))
    for t, text in log.messages:
        m = _MISSION_MSG_RE.match(text)
        if not m:
            continue
        cnum = int(m.group(1))
        if (cnum, t // 1000) in seen or (cnum, t // 1000 - 1) in seen or (cnum, t // 1000 + 1) in seen:
            continue
        cid, prm1 = lookup(cnum, t)
        out.append(MissionExec(t, cnum, cid, prm1, "MSG"))
        seen.add((cnum, t // 1000))
    out.sort(key=lambda e: (e.t_us, e.cnum))
    return out


def strip_windows(log: DFLog) -> list:
    """由 206 開／關事件組出作業帶時間窗。"""
    _, t_end = log.time_range_us
    windows = []
    open_on = None
    for e in mission_executions(log):
        if e.cid != CMD_DO_SET_CAM_TRIGG_DIST or e.prm1 is None:
            continue
        if e.prm1 > 0:
            if open_on is not None:     # 沒關就又開：前一條在這裡結束
                windows.append(StripWindow(len(windows) + 1, open_on.t_us, e.t_us, open_on.prm1,
                                           open_on.cnum, None, ("reopened",)))
            open_on = e
        elif open_on is not None:
            windows.append(StripWindow(len(windows) + 1, open_on.t_us, e.t_us, open_on.prm1,
                                       open_on.cnum, e.cnum))
            open_on = None
    if open_on is not None:
        windows.append(StripWindow(len(windows) + 1, open_on.t_us, int(t_end) + 1, open_on.prm1,
                                   open_on.cnum, None, ("unclosed",)))
    return windows


def strip_of(windows, t_us) -> Optional[int]:
    for w in windows:
        if w.contains(t_us):
            return w.strip
    return None


# ── 飛控時鐘 → GPST ─────────────────────────────────────────────────────

@dataclass(frozen=True)
class GpsTimeFit:
    """GPST_s = a·(TimeUS·1e-6) + c + lag_s。

    GPS.TimeUS 是飛控「收到並處理」這筆定位的時刻，GMS 是量測 epoch ⇒ 擬合出來的 c 含有
    接收機與驅動延遲（ArduPilot 預設延遲模型 0.1 s，實際值未驗；評估 §14 K13、D05）。
    ``lag_s`` 由呼叫端明確給（預設 0 = 不補），寫進交付物 metadata。
    """
    a: float
    c: float
    lag_s: float
    rms_s: float
    n: int
    n_rejected: int

    def gpst_s(self, t_us):
        return self.a * (np.asarray(t_us, float) * 1e-6) + self.c + self.lag_s


def gps_time_fit(log: DFLog, *, instance: int = 0, min_status: int = 3, lag_s: float = 0.0) -> GpsTimeFit:
    gps = log.table("GPS")
    if not gps or "GWk" not in gps or "GMS" not in gps:
        raise ValueError("日誌沒有 GPS（GWk/GMS）紀錄，無法換算 GPS 時間")
    sel = (np.asarray(gps["Status"]) >= min_status) & (np.asarray(gps["GWk"]) > 0)
    if "I" in gps:
        sel &= np.asarray(gps["I"]) == instance
    x = np.asarray(gps["TimeUS"], float)[sel] * 1e-6
    y = np.asarray(gps["GWk"], float)[sel] * 604800.0 + np.asarray(gps["GMS"], float)[sel] * 1e-3
    if x.size < 2:
        raise ValueError(f"GPS {min_status}D 定位紀錄只有 {x.size} 筆，無法擬合時間")
    x0 = float(x.mean())
    keep = np.ones(x.size, bool)
    for _ in range(5):
        a, c0 = np.polyfit(x[keep] - x0, y[keep], 1)
        r = y - (a * (x - x0) + c0)
        mad = np.median(np.abs(r[keep] - np.median(r[keep])))
        new = np.abs(r) <= max(5.0 * 1.4826 * mad, 0.05)     # 剔除跳秒、重送等離群
        if new.sum() < 2 or (new == keep).all():
            break
        keep = new
    rms = float(np.sqrt(np.mean(r[keep] ** 2)))
    return GpsTimeFit(float(a), float(c0 - a * x0), float(lag_s), rms, int(keep.sum()), int((~keep).sum()))
