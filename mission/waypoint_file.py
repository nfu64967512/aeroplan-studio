# -*- coding: utf-8 -*-
"""航點檔讀寫與驗證 —— Mission Planner 的 Load WP File / Save WP File。

純 Python、不依賴 Qt；pymavlink 只在查「指令名稱」時才惰性匯入（匯不進來就
略過『未知指令』提示，不會拿它當理由讓載入失敗）。

═══ 為什麼是純 dict、不走 mission.waypoint.Waypoint ═══
`Waypoint.__post_init__` 會做 `MAVCommand(command)`，而那是**封閉列舉**，
少了 DO_JUMP(177)、DO_SET_MODE(176)、DO_LAND_START(189)、
NAV_CONTINUE_AND_CHANGE_ALT(30)…… ⇒ 使用者從 Mission Planner 帶來的檔案只要含
其中任何一個，整份就在建構時被 ValueError 吃掉。這裡的項目和
`SITLLink._recv_mission` 吐出的完全同形：

    {"seq","command","frame","lat","lon","alt","param1".."param4",
     "current","autocontinue"}

seq 0 是 Home 槽（Mission Planner 慣例）。

═══ 支援格式 ═══
1. QGC WPL 100/110（.waypoints / .txt）：首行 `QGC WPL <ver>`，之後每行 12 欄
   `INDEX CURRENT FRAME COMMAND P1 P2 P3 P4 LAT LON ALT AUTOCONTINUE`。
   Mission Planner 用 TAB，但任意連續空白都收。容忍 UTF-8 BOM、CRLF/LF 混用、
   空行、整行 `#` 註解、`1.0E-05`、`1e-3`。seq 以**行序重編**（和 Mission
   Planner 一樣），檔內 INDEX 對不上時警告而非默默採用。
2. QGroundControl .plan（JSON）：SimpleItem 直接轉；ComplexItem 若內嵌已解算的
   SimpleItem（TransectStyleComplexItem.Items）就依序展平並警告，沒有內嵌就報
   **錯誤**（不可默默丟掉整片掃描）。`frame` 欄位是高度參考的唯一依據，
   AltitudeMode 對不上只警告。plannedHomePosition 變成合成的 seq 0（cmd 16,
   frame 0, current 1）。QGC 的 DO_JUMP 目標是 doJumpId 而不是 seq，展平後
   會依 doJumpId→seq 對照表改寫 param1。

格式看**內容**判斷（JSON 物件 vs `QGC WPL` 表頭），副檔名不對也讀得進來。

═══ 失敗策略（LOUD）═══
* 讀不到 / 認不出 / 語法錯（欄數不對、不是數字）⇒ 丟 `MissionFileError`，
  訊息帶**行號與該行原文**。永遠不回傳一份「靜靜是空的」任務：空檔、純空白
  檔直接丟例外；只有表頭的檔會回傳一份帶 `empty` 錯誤的任務。
* 語意問題以 `MissionIssue` 收集：error 擋寫入飛控，warning 只提示。

═══ validate_items 的規則碼 ═══
ERROR（擋上傳）
  bad_item       項目缺欄位或欄位不是數字
  empty          沒有任何項目，或只有 seq 0 的 Home（合法但幾乎一定是誤操作）
  too_many       項目數 > MAX_ITEMS(656)。依據：ArduPilot 內建 StorageManager
                 layout 中 StorageMission 最大區塊 9842 位元組（ChibiOS 大板，
                 StorageManager.cpp），每個指令占 AP_MISSION_EEPROM_COMMAND_SIZE
                 =15 位元組 ⇒ 656 個含 Home 槽。用 SD 卡儲存的板子可能更大，
                 但飛控自己也會回 MAV_MISSION_NO_SPACE；這裡取保守值先擋。
  not_finite     lat/lon/alt/param1-4 為 NaN 或 inf
  lat_range      緯度超出 [-90, 90]
  lon_range      經度超出 [-180, 180]
  cmd_range      command 不在 0..65535（MAVLink uint16）
  frame_range    frame 不在 0..255（MAVLink uint8）
  zero_position  帶座標的 NAV 指令 (16,17,18,19,21,22,31,82,84,85) 在全球座標
                 系（frame 0/3/10）位於 lat=lon=0 ⇒ 幾內亞灣上空（幾乎一定是
                 漏填）。例外：NAV_LAND(21)/VTOL_LAND(85) 的 0,0 是官方的
                 「就地降落」，NAV_TAKEOFF(22)/VTOL_TAKEOFF(84) 的 0,0 對稱地
                 是官方的「從目前位置起飛」（兩者飛控端都不採用 mission item
                 座標，見 _HERE_OK 旁的引註）；RTL(20)、NAV_DELAY(93) 不帶
                 座標；seq 0 Home 另有 home_at_zero 警告；frame 2(MISSION)
                 等非全球座標系不檢。
  jump_target    DO_JUMP(177) 的 param1 不是整數，或不在 1..n-1
  jump_self      DO_JUMP 跳到自己
  jump_repeat    DO_JUMP 的 param2 不是整數或 < -1（-1 = 無限重複）
  jump_no_nav    DO_JUMP 重複次數 -1（無限）、且從目標開始模擬接下來的走法
                 （見 `_jump_leads_into_a_nav_free_loop`）永遠碰不到任何 NAV
                 指令 ⇒ 永遠只在 DO 指令間打轉，任務不會前進。不只查「這一個
                 跳自己往回的區間」——往前跳若落在另一個往回跳的無限 DO_JUMP
                 上、兩者互相銜接繞成一圈，一樣算
WARNING（顯示但允許）
  seq_mismatch   項目的 seq 欄和它在清單裡的位置不同（寫入時會以位置重編）
  no_takeoff     第一個 NAV 指令（略過 NAV_DELAY 93）不是起飛(22/84)；從地面切
                 AUTO 會直接去第一個航點
  no_nav_command 整份任務沒有任何 NAV 指令
  after_land     最後一個 LAND(21/85) 之後還有項目（永遠執行不到）
  last_not_terminal 最後一項不是 RTL/LAND/盤旋(17,18,19,31,20,21,85) ⇒ 任務
                 走完後飛控沒有指示
  rel_alt_nonpositive NAV_WAYPOINT 相對高度(frame 3) <= 0
  rel_alt_low    其餘帶座標 NAV（LAND 除外）相對高度 < 1 m
  far_apart      相鄰兩個帶座標 NAV 相距 > 50 km（多半是座標打錯）
  home_at_zero   seq 0 Home 位於 (0,0)
  do_latlon      DO_* 類指令帶非零 lat/lon（多數 DO 指令的飛控端不使用座標，
                 frame 2 下更可能被當整數欄位）；DO_SET_HOME/DO_LAND_START/
                 DO_REPOSITION/DO_SET_ROI* 本來就用座標，不警告
  unknown_command pymavlink 不認得的指令 id（飛控可能仍支援，僅提示）
解析階段另有：bad_version、index_mismatch(_more)、home_missing、
plan_filetype、plan_altmode_mismatch、plan_jump_unmapped、
complex_flattened（警告）與 complex_unresolved、plan_unknown_type、
plan_item、plan_params（錯誤）。
"""
from __future__ import annotations

import json
import math
import os
import re
import tempfile
from dataclasses import dataclass, field

# ── 常數 ────────────────────────────────────────────────────────────────
MAX_ITEMS = 656
MAX_FILE_BYTES = 20 * 1024 * 1024
FAR_APART_M = 50_000.0

CMD_NAV_WAYPOINT = 16
CMD_NAV_LAND = 21
CMD_NAV_TAKEOFF = 22
CMD_NAV_RTL = 20
CMD_NAV_VTOL_TAKEOFF = 84
CMD_NAV_VTOL_LAND = 85
CMD_DO_JUMP = 177

# 帶座標的 NAV 指令
_LOCATION_NAV = frozenset({16, 17, 18, 19, 21, 22, 31, 82, 84, 85})
_TAKEOFF = frozenset({CMD_NAV_TAKEOFF, CMD_NAV_VTOL_TAKEOFF})
_LAND = frozenset({CMD_NAV_LAND, CMD_NAV_VTOL_LAND})
# zero_position 的例外：LAND/VTOL_LAND 的 (0,0) 是官方的「就地降落」；
# TAKEOFF/VTOL_VTAKEOFF 的 (0,0) 與它對稱，是官方的「從目前位置起飛」——
# ArduCopter mode_auto.cpp::takeoff_start() 水平位置一律用 current_loc，
# ArduPlane quadplane.cpp::do_vtol_takeoff() 更直接把 loc.lat/lng 強制清零
# （註解「we always use the current location in XY for takeoff」），兩者
# 都不採用 mission item 裡的座標。Mission Planner / QGC 匯出的標準起飛航點
# 常見這樣寫，擋下它是誤傷，不是保護。
_HERE_OK = frozenset({CMD_NAV_LAND, CMD_NAV_VTOL_LAND}) | _TAKEOFF
_TERMINAL = frozenset({17, 18, 19, 31, CMD_NAV_RTL, CMD_NAV_LAND, CMD_NAV_VTOL_LAND})
_GLOBAL_FRAMES = frozenset({0, 3, 10})
# 本來就使用座標的 DO_* 指令：SET_HOME、LAND_START、REPOSITION、SET_ROI*
_DO_WITH_LOCATION = frozenset({179, 189, 192, 195, 201})
_NAV_LAST = 95  # MAV_CMD_NAV_LAST：id <= 95 皆為 NAV 指令（AP_Mission::is_nav_cmd）

_KEYS_INT = ("command", "frame", "current", "autocontinue")
_KEYS_FLOAT = ("lat", "lon", "alt", "param1", "param2", "param3", "param4")


class MissionFileError(ValueError):
    """檔案讀不到、認不出或語法錯。`line`/`text` 在語法錯時帶行號與原文。"""

    def __init__(self, message, line=None, text=None):
        super().__init__(message)
        self.line = line
        self.text = text


@dataclass(frozen=True)
class MissionIssue:
    level: str          # "error" | "warning"
    seq: "int | None"
    code: str
    message: str        # 繁體中文


@dataclass
class LoadedMission:
    path: str
    fmt: str            # "wpl" | "plan"
    items: list = field(default_factory=list)
    issues: list = field(default_factory=list)

    @property
    def errors(self):
        return [i for i in self.issues if i.level == "error"]

    @property
    def warnings(self):
        return [i for i in self.issues if i.level == "warning"]

    @property
    def has_errors(self):
        return any(i.level == "error" for i in self.issues)

    def summary_text(self) -> str:
        name = {"wpl": "QGC WPL", "plan": "QGC .plan"}.get(self.fmt, self.fmt)
        lines = [f"{os.path.basename(self.path) or self.path}：{len(self.items)} 個項目"
                 f"（{name}），{len(self.errors)} 個錯誤、{len(self.warnings)} 個警告"]
        for i in self.errors:
            lines.append(f"  [錯誤] {_where(i.seq)}{i.message}")
        for i in self.warnings:
            lines.append(f"  [警告] {_where(i.seq)}{i.message}")
        return "\n".join(lines)


def _where(seq):
    return "" if seq is None else f"#{seq} "


# ── 小工具 ──────────────────────────────────────────────────────────────
_CMD_NAMES = None


def _cmd_known(cmd: int):
    """True/False；pymavlink 不可用回 None（= 不下判斷）。"""
    global _CMD_NAMES
    if _CMD_NAMES is None:
        try:
            from pymavlink import mavutil
            _CMD_NAMES = set(mavutil.mavlink.enums["MAV_CMD"].keys())
        except Exception:
            _CMD_NAMES = False
    if _CMD_NAMES is False:
        return None
    return cmd in _CMD_NAMES


def _is_nav(cmd: int) -> bool:
    return 0 < cmd <= _NAV_LAST


def _is_intlike(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x) and float(x) == int(x)


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((p2 - p1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6371000.0 * math.asin(min(1.0, math.sqrt(a)))


def _make_item(seq, current, frame, command, p1, p2, p3, p4, lat, lon, alt, autocontinue):
    return {
        "seq": int(seq), "command": int(command), "frame": int(frame),
        "lat": float(lat), "lon": float(lon), "alt": float(alt),
        "param1": float(p1), "param2": float(p2), "param3": float(p3),
        "param4": float(p4), "current": int(current), "autocontinue": int(autocontinue),
    }


# ── 驗證 ────────────────────────────────────────────────────────────────
def validate_items(items) -> list:
    """回傳 MissionIssue 清單；規則碼見模組 docstring。"""
    issues = []

    def err(seq, code, msg):
        issues.append(MissionIssue("error", seq, code, msg))

    def warn(seq, code, msg):
        issues.append(MissionIssue("warning", seq, code, msg))

    n = len(items)
    if n == 0 or n == 1:
        err(None, "empty", "任務是空的（沒有任何航點；只有 Home 槽也算空）。"
            "寫入空任務會清掉飛控上原有的航線，幾乎一定是誤操作")
    if n > MAX_ITEMS:
        err(None, "too_many", f"項目數 {n} 超過上限 {MAX_ITEMS}"
            "（ArduPilot 最大內建任務儲存 9842 B ÷ 每項 15 B）")

    # 先逐項體檢；壞掉的項目不進入後面的關聯規則
    ok = [False] * n
    for i, it in enumerate(items):
        bad = False
        try:
            for k in _KEYS_INT:
                if not _is_intlike(it[k]):
                    raise ValueError(k)
            for k in _KEYS_FLOAT:
                v = it[k]
                if isinstance(v, bool) or not isinstance(v, (int, float)):
                    raise ValueError(k)
        except (KeyError, ValueError, TypeError) as e:
            err(i, "bad_item", f"項目欄位缺漏或不是數字（{e}）")
            continue
        if not all(math.isfinite(float(it[k])) for k in _KEYS_FLOAT):
            err(i, "not_finite", "lat/lon/alt/param 含 NaN 或 inf")
            bad = True
        cmd, frm = int(it["command"]), int(it["frame"])
        if not 0 <= cmd <= 65535:
            err(i, "cmd_range", f"command {cmd} 超出 MAVLink uint16 範圍 0..65535")
            bad = True
        if not 0 <= frm <= 255:
            err(i, "frame_range", f"frame {frm} 超出 MAVLink uint8 範圍 0..255")
            bad = True
        if not bad:
            ok[i] = True
        if isinstance(it.get("seq"), int) and it["seq"] != i:
            warn(i, "seq_mismatch", f"seq 欄位是 {it['seq']}，但它在清單第 {i} 位；寫入時以位置重編")

    for i, it in enumerate(items):
        if not ok[i]:
            continue
        cmd, frm = int(it["command"]), int(it["frame"])
        lat, lon, alt = float(it["lat"]), float(it["lon"]), float(it["alt"])
        if not -90.0 <= lat <= 90.0:
            err(i, "lat_range", f"緯度 {lat} 超出 [-90, 90]")
        if not -180.0 <= lon <= 180.0:
            err(i, "lon_range", f"經度 {lon} 超出 [-180, 180]")
        if _cmd_known(cmd) is False:
            warn(i, "unknown_command", f"指令 {cmd} 不在 pymavlink 列舉內（飛控可能仍支援）")
        if i == 0:
            if lat == 0.0 and lon == 0.0:
                warn(0, "home_at_zero", "Home（seq 0）位於 (0,0)")
            continue
        if cmd in _LOCATION_NAV and lat == 0.0 and lon == 0.0 \
                and frm in _GLOBAL_FRAMES and cmd not in _HERE_OK:
            err(i, "zero_position", f"NAV 指令 {cmd} 位於 lat=lon=0（幾內亞灣）；"
                "多半是漏填座標")
        if cmd == CMD_NAV_WAYPOINT and frm == 3 and alt <= 0:
            warn(i, "rel_alt_nonpositive", f"NAV_WAYPOINT 相對高度 {alt} m <= 0")
        elif (cmd in _LOCATION_NAV and cmd not in _LAND and frm == 3 and alt < 1):
            # ⚠️ 這裡不排除 (0,0)：那是 zero_position 的例外（見 _HERE_OK），
            # 跟「相對高度是不是低得可疑」是兩件事——TAKEOFF/VTOL_TAKEOFF 在
            # (0,0)（「從目前位置起飛」的官方慣例，Mission Planner/QGC 常見
            # 這樣寫）一樣可能把高度打錯成 0.2 m 而不是 20 m，一樣該提示。
            warn(i, "rel_alt_low", f"相對高度 {alt} m < 1 m")
        if (not _is_nav(cmd) and cmd != CMD_DO_JUMP
                and cmd not in _DO_WITH_LOCATION and (lat != 0.0 or lon != 0.0)):
            warn(i, "do_latlon", f"指令 {cmd} 帶非零 lat/lon，飛控多半不使用")
        if cmd == CMD_DO_JUMP:
            _check_jump(i, it, items, ok, err)

    # 序列規則（只看通過體檢的真實項目）
    real = [i for i in range(1, n) if ok[i]]
    navs = [i for i in real if _is_nav(int(items[i]["command"]))]
    if real and not navs:
        warn(None, "no_nav_command", "任務中沒有任何 NAV 指令，飛控不會去任何地方")
    # NAV_DELAY(93) 是「等到某時刻再開始」，排在起飛前是 HARVEST 多機錯時出發的
    # 標準寫法，不算「第一個動作」（否則 AeroPlan 自己匯出的檔會被誤警告）。
    first_motion = [i for i in navs if int(items[i]["command"]) != 93]
    if first_motion and int(items[first_motion[0]]["command"]) not in _TAKEOFF:
        warn(first_motion[0], "no_takeoff", "第一個 NAV 指令不是起飛（22/84）；"
             "從地面切 AUTO 會直接去第一個航點")
    lands = [i for i in real if int(items[i]["command"]) in _LAND]
    if lands and lands[-1] != n - 1:
        warn(lands[-1] + 1, "after_land", f"最後一個降落（#{lands[-1]}）之後還有項目，"
             "永遠執行不到")
    if real and int(items[real[-1]]["command"]) not in _TERMINAL and real[-1] == n - 1:
        warn(real[-1], "last_not_terminal", "最後一項不是 RTL/降落/盤旋；"
             "任務走完後飛控沒有指示")
    prev = None
    for i in real:
        it = items[i]
        if int(it["command"]) in _LOCATION_NAV and not (it["lat"] == 0 and it["lon"] == 0) \
                and abs(it["lat"]) <= 90 and abs(it["lon"]) <= 180:
            if prev is not None:
                d = _haversine_m(items[prev]["lat"], items[prev]["lon"], it["lat"], it["lon"])
                if d > FAR_APART_M:
                    warn(i, "far_apart", f"與前一個 NAV 點 #{prev} 相距 {d / 1000:.1f} km（> 50 km）")
            prev = i
    return issues


def _jump_next_index(k, items, ok, n):
    """回傳『執行完第 k 項之後，飛控接下來會走到哪一項』的近似值。

    只有「無限重複」的 DO_JUMP 才當成強制轉向 —— 它每次執行到都一定跳，
    永遠不會落到下一項；其餘（含有限次數的 DO_JUMP：用完重複次數後終究
    會落到下一項）一律當成往下一項走。這是保守但足夠判斷「有沒有機會碰到
    NAV」的近似，不是完整的指令模擬器。
    """
    if ok[k]:
        cmd = int(items[k]["command"])
        if cmd == CMD_DO_JUMP:
            tgt, rep = float(items[k]["param1"]), float(items[k]["param2"])
            if (_is_intlike(rep) and rep == -1
                    and _is_intlike(tgt) and 1 <= tgt <= n - 1 and int(tgt) != k):
                return int(tgt)
    return k + 1


def _jump_leads_into_a_nav_free_loop(start, items, ok, n):
    """從 `start` 開始依 `_jump_next_index` 模擬飛控接下來怎麼走。

    在真的碰到 NAV 指令之前，如果又繞回走過的位置，代表這條路徑會永遠
    卡在 DO 指令之間、看不到任何 NAV —— 回傳 True。

    ⚠️ 故意不是只看「這一個 DO_JUMP 自己往回的區間裡有沒有 NAV」：往前跳
    一定繼續往下走**這件事本身沒錯**，但如果它跳到的地方又是另一個無限
    DO_JUMP、繞回來銜接成一圈，整條路徑照樣永遠困住而看不到任何 NAV。
    真正要問的是「模擬走下去，能不能走到 NAV」，不是「往回跳還是往前跳」。
    """
    seen = set()
    k = start
    while 0 <= k < n:
        if k in seen:
            return True  # 繞回走過的位置，沿途都還沒碰到 NAV ⇒ 迴圈
        seen.add(k)
        if ok[k] and _is_nav(int(items[k]["command"])):
            return False
        k = _jump_next_index(k, items, ok, n)
    return False  # 掉出任務尾端＝終究會停下來，不是「永遠困住」


def _check_jump(i, it, items, ok, err):
    n = len(items)
    tgt, rep = float(it["param1"]), float(it["param2"])
    if not _is_intlike(rep) or rep < -1:
        err(i, "jump_repeat", f"DO_JUMP 重複次數 {rep} 必須是 >= -1 的整數（-1 = 無限）")
    if not _is_intlike(tgt) or not 1 <= tgt <= n - 1:
        err(i, "jump_target", f"DO_JUMP 目標 {tgt} 必須是 1..{n - 1} 的整數")
        return
    t = int(tgt)
    if t == i:
        err(i, "jump_self", "DO_JUMP 跳到自己")
        return
    # 無限重複才需要檢查：有限次數用完後終究會落到下一項，不會永遠困住。
    # 兩個方向都要模擬（見 `_jump_leads_into_a_nav_free_loop` 的說明）——
    # 只在「往回跳」才查，擋不住「往前跳落在另一個往回跳的無限 DO_JUMP 上」
    # 這種鏈狀迴圈。
    if _is_intlike(rep) and rep == -1:
        if _jump_leads_into_a_nav_free_loop(t, items, ok, n):
            err(i, "jump_no_nav", f"DO_JUMP 無限重複，從 #{t} 開始永遠碰不到任何 NAV 指令，"
                "任務會永遠卡在 DO 指令裡")


# ── QGC WPL ─────────────────────────────────────────────────────────────
_HEADER_RE = re.compile(r"^QGC\s+WPL\s+(\S+)", re.IGNORECASE)
_NL_RE = re.compile(r"\r\n|\r|\n")


def _tok_int(tok, what, lineno, raw):
    try:
        return int(tok)
    except ValueError:
        pass
    try:
        f = float(tok)
    except ValueError:
        f = None
    if f is None or not math.isfinite(f) or f != int(f):
        raise MissionFileError(f"第 {lineno} 行：{what} 不是整數（{tok!r}）：{raw!r}", lineno, raw)
    return int(f)


def _tok_float(tok, what, lineno, raw):
    try:
        return float(tok)
    except ValueError:
        raise MissionFileError(f"第 {lineno} 行：{what} 不是數字（{tok!r}）：{raw!r}", lineno, raw)


def parse_wpl_text(text: str, source: str = "<text>") -> LoadedMission:
    if text.startswith("﻿"):
        text = text[1:]
    lines = _NL_RE.split(text)
    hdr_i = None
    for k, raw in enumerate(lines):
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        hdr_i = k
        break
    if hdr_i is None:
        raise MissionFileError(f"{source}：檔案是空的（沒有任何內容），不是航點檔")
    m = _HEADER_RE.match(lines[hdr_i].strip())
    if not m:
        raise MissionFileError(
            f"第 {hdr_i + 1} 行：表頭應為 'QGC WPL 110'，實際是 {lines[hdr_i].strip()!r}",
            hdr_i + 1, lines[hdr_i])
    issues = []
    ver = m.group(1)
    if ver not in ("100", "110"):
        issues.append(MissionIssue("warning", None, "bad_version",
                                   f"QGC WPL 版本 {ver} 不是 100/110，仍嘗試依 110 格式解析"))
    items = []
    mism = 0
    for k in range(hdr_i + 1, len(lines)):
        raw = lines[k]
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        lineno = k + 1
        cols = s.split()
        if len(cols) != 12:
            raise MissionFileError(
                f"第 {lineno} 行：需要 12 欄，實際 {len(cols)} 欄：{s!r}", lineno, s)
        idx = _tok_int(cols[0], "INDEX", lineno, s)
        cur = _tok_int(cols[1], "CURRENT", lineno, s)
        frm = _tok_int(cols[2], "COORD_FRAME", lineno, s)
        cmd = _tok_int(cols[3], "COMMAND", lineno, s)
        p = [_tok_float(cols[4 + j], f"P{j + 1}", lineno, s) for j in range(4)]
        lat = _tok_float(cols[8], "LAT", lineno, s)
        lon = _tok_float(cols[9], "LON", lineno, s)
        alt = _tok_float(cols[10], "ALT", lineno, s)
        ac = _tok_int(cols[11], "AUTOCONTINUE", lineno, s)
        seq = len(items)
        if idx != seq:
            mism += 1
            if mism <= 5:
                issues.append(MissionIssue(
                    "warning", seq, "index_mismatch",
                    f"第 {lineno} 行的 INDEX 是 {idx}，但依行序它是第 {seq} 項；以行序重編"))
        items.append(_make_item(seq, cur, frm, cmd, *p, lat, lon, alt, ac))
    if mism > 5:
        issues.append(MissionIssue("warning", None, "index_mismatch_more",
                                   f"另有 {mism - 5} 行 INDEX 與行序不符（未逐一列出）"))
    issues.extend(validate_items(items))
    return LoadedMission(source, "wpl", items, issues)


# ── QGC .plan ───────────────────────────────────────────────────────────
# QGC 4.x AltitudeMode：0 Mixed、1 Relative、2 Absolute、3 CalcAboveTerrain、4 TerrainFrame
_ALTMODE_FRAMES = {1: {3}, 2: {0}, 3: {0}, 4: {10}}


def _plan_number(v, nan_for_null):
    if v is None:
        return math.nan if nan_for_null else 0.0
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise TypeError(repr(v))
    return float(v)


def parse_plan_obj(obj, source: str = "<plan>") -> LoadedMission:
    if not isinstance(obj, dict) or not isinstance(obj.get("mission"), dict):
        raise MissionFileError(f"{source}：不是 QGC .plan（缺少 'mission' 物件）")
    issues = []
    if obj.get("fileType") != "Plan":
        issues.append(MissionIssue("warning", None, "plan_filetype",
                                   f"fileType 是 {obj.get('fileType')!r}，不是 'Plan'"))
    mis = obj["mission"]
    raw_items = mis.get("items")
    if not isinstance(raw_items, list):
        raise MissionFileError(f"{source}：mission.items 不是清單")

    items = []
    home = mis.get("plannedHomePosition")
    if (isinstance(home, list) and len(home) >= 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in home[:3])):
        items.append(_make_item(0, 1, 0, 16, 0, 0, 0, 0, home[0], home[1], home[2], 1))
    else:
        items.append(_make_item(0, 1, 0, 16, 0, 0, 0, 0, 0.0, 0.0, 0.0, 1))
        issues.append(MissionIssue("warning", 0, "home_missing",
                                   "plan 沒有有效的 plannedHomePosition，Home 槽以 (0,0,0) 合成"))
    id_to_seq = {}
    jumps = []   # 展平後的 DO_JUMP 位置

    def add_simple(it, where):
        seq = len(items)
        try:
            cmd = it["command"]
            frm = it["frame"]
            params = it["params"]
            if not isinstance(params, list) or len(params) != 7:
                issues.append(MissionIssue("error", None, "plan_params",
                                           f"{where}：params 必須是 7 個元素"))
                return
            if not _is_intlike(cmd) or not _is_intlike(frm):
                raise TypeError("command/frame")
            cmd, frm = int(cmd), int(frm)
            loc = cmd in _LOCATION_NAV
            p = [_plan_number(params[j], False) for j in range(4)]
            lat, lon, alt = (_plan_number(params[j], loc) for j in (4, 5, 6))
        except KeyError as e:
            issues.append(MissionIssue("error", None, "plan_item", f"{where}：缺少欄位 {e}"))
            return
        except (TypeError, ValueError) as e:
            issues.append(MissionIssue("error", None, "plan_item", f"{where}：欄位型別錯誤（{e}）"))
            return
        ac = it.get("autoContinue", True)
        items.append(_make_item(seq, 0, frm, cmd, *p, lat, lon, alt, 1 if ac else 0))
        djid = it.get("doJumpId")
        if isinstance(djid, int) and not isinstance(djid, bool):
            id_to_seq[djid] = seq
        if cmd == CMD_DO_JUMP:
            jumps.append(seq)
        mode = it.get("AltitudeMode")
        if mode in _ALTMODE_FRAMES and loc and frm not in _ALTMODE_FRAMES[mode]:
            issues.append(MissionIssue(
                "warning", seq, "plan_altmode_mismatch",
                f"{where}：AltitudeMode={mode} 與 frame={frm} 不一致；以 frame 為準"))

    def walk(lst, where, flattened):
        for k, it in enumerate(lst):
            w = f"{where}[{k}]"
            if not isinstance(it, dict):
                issues.append(MissionIssue("error", None, "plan_item", f"{w}：不是物件"))
                continue
            t = it.get("type")
            if t == "SimpleItem":
                add_simple(it, w)
            elif t == "ComplexItem":
                ct = it.get("complexItemType", "?")
                emb = None
                tsc = it.get("TransectStyleComplexItem")
                if isinstance(tsc, dict) and isinstance(tsc.get("Items"), list):
                    emb = tsc["Items"]
                if emb:
                    issues.append(MissionIssue(
                        "warning", None, "complex_flattened",
                        f"{w}：ComplexItem（{ct}）已展平成 {len(emb)} 個 SimpleItem"))
                    walk(emb, w + ".Items", True)
                else:
                    issues.append(MissionIssue(
                        "error", None, "complex_unresolved",
                        f"{w}：ComplexItem（{ct}）沒有內嵌已解算的航點，無法轉成任務項目；"
                        "請在 QGroundControl 中重新儲存或改用 .waypoints"))
            else:
                issues.append(MissionIssue("error", None, "plan_unknown_type",
                                           f"{w}：未知的項目 type {t!r}"))

    walk(raw_items, "mission.items", False)

    for s in jumps:
        t = items[s]["param1"]
        if id_to_seq and _is_intlike(t) and int(t) in id_to_seq:
            items[s]["param1"] = float(id_to_seq[int(t)])
        elif id_to_seq:
            issues.append(MissionIssue("warning", s, "plan_jump_unmapped",
                                       f"DO_JUMP 目標 doJumpId={t} 找不到對應項目，保持原值"))
    issues.extend(validate_items(items))
    return LoadedMission(source, "plan", items, issues)


# ── 入口 ────────────────────────────────────────────────────────────────
def load_mission_file(path) -> LoadedMission:
    p = os.fspath(path)
    try:
        if os.path.getsize(p) > MAX_FILE_BYTES:
            raise MissionFileError(f"{p}：檔案超過 {MAX_FILE_BYTES // 1048576} MB，不像航點檔")
        with open(p, "rb") as f:
            raw = f.read()
    except OSError as e:
        raise MissionFileError(f"{p}：讀不到檔案（{e}）") from e
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as e:
        raise MissionFileError(f"{p}：不是 UTF-8 文字檔（{e}）") from e
    st = text.lstrip()
    if not st:
        raise MissionFileError(f"{p}：檔案是空的（沒有任何內容），不是航點檔")
    if st.startswith("{"):
        try:
            obj = json.loads(st)
        except json.JSONDecodeError as e:
            raise MissionFileError(
                f"{p}：JSON 語法錯（第 {e.lineno} 行第 {e.colno} 欄：{e.msg}）",
                e.lineno) from e
        lm = parse_plan_obj(obj, p)
    elif st.startswith(("QGC", "qgc", "#")) or _HEADER_RE.match(st):
        lm = parse_wpl_text(text, p)
    else:
        raise MissionFileError(
            f"{p}：認不出格式。支援 QGC WPL 110（.waypoints，首行 'QGC WPL 110'）"
            "與 QGroundControl .plan（JSON）")
    return lm


def _fmt(x) -> str:
    return repr(float(x))


def items_to_wpl_text(items) -> str:
    """輸出 QGC WPL 110（TAB 分隔、\\n 結尾）；seq 以位置 0..n-1 重編。"""
    out = ["QGC WPL 110"]
    for i, it in enumerate(items):
        try:
            v = {k: float(it[k]) for k in _KEYS_FLOAT}
            c = {k: int(it[k]) for k in _KEYS_INT}
        except (KeyError, TypeError, ValueError) as e:
            raise MissionFileError(f"第 {i} 項無法輸出（欄位缺漏或非數字：{e}）") from e
        bad = [k for k, x in v.items() if not math.isfinite(x)]
        if bad:
            raise MissionFileError(f"第 {i} 項的 {'/'.join(bad)} 不是有限數字，拒絕寫出")
        out.append("\t".join([
            str(i), str(c["current"]), str(c["frame"]), str(c["command"]),
            _fmt(v["param1"]), _fmt(v["param2"]), _fmt(v["param3"]), _fmt(v["param4"]),
            f"{v['lat']:.8f}", f"{v['lon']:.8f}", _fmt(v["alt"]), str(c["autocontinue"]),
        ]))
    return "\n".join(out) + "\n"


def save_wpl(items, path) -> str:
    """先寫暫存檔再 os.replace，避免寫到一半斷電留下半份航點檔。回傳路徑。"""
    text = items_to_wpl_text(items)
    p = os.fspath(path)
    d = os.path.dirname(os.path.abspath(p))
    tmp = None
    try:
        fd, tmp = tempfile.mkstemp(prefix=".wpl_", suffix=".tmp", dir=d)
        with os.fdopen(fd, "wb") as f:
            f.write(text.encode("utf-8"))
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, p)
        tmp = None
    except OSError as e:
        raise MissionFileError(f"{p}：寫檔失敗（{e}）") from e
    finally:
        if tmp is not None:
            try:
                os.remove(tmp)
            except OSError:
                pass
    return p
