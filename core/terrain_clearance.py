"""上傳任務沿線 AGL 檢查：地形 × **實際上傳、已縮點**的任務項目。純 Python（numpy，不用 Qt）。

規劃器（hv_dccpp）自己的 AGL 驗證（``sim.check_kinematics``）看的是 ``UAVPlan.waypoints`` 的**密集航點**；
真正送上飛控的是 ``ardupilot.reduce_waypoints`` 縮點後的項目（直線段高度剖面以 Douglas–Peucker 縮到
``cfg.delta_prof_m`` = 10 m 容差、弧段換成外切多邊形頂點、整圈盤旋換成 LOITER 指令）。這個模組對**上傳那一份**
重算沿線離地高（2026-10-21，docs/評估_2026-10-21_網格風與地形接入規劃器.md §4.2）。

做法：

* 輸入每架的任務項目（``HvDccppCoordinator.mission_items(did, camera_cfg=…)`` 的 dict，或
  ``hv_dccpp.ardupilot.MissionItem``）：第 0 列 home（這架的起飛坪），導航點 frame 3（相對起飛坪）。
* 地形＝規劃那一次的**自然地形** ``ctx.terrain.raw_view()``（與 ``check_kinematics`` 同一個基準；障礙淨空另有
  keep-out 檢查），座標框架＝``ctx.frame``。
* AGL ＝ home 地面 ＋ 相對高 − 該點地面。home 地面與該點地面取自**同一份 DEM** ⇒ 高程基準（World Terrain 的
  橢球高、N 未知）互相抵銷，只用到相對高度。home 地面取 DEM 在第 0 列經緯度的值（飛控解鎖時 home 重設到飛機
  所在處 ＝ 真實地面），**不是**第 0 列的 alt（那是規劃器自己的坪高估計；兩者差 > 1 m 另列在 caveats）。
* 導航點之間沿直線每 ``sample_m``（25 m，同規劃器 ``d_d_m``）取樣，高度在兩點間線性內插（ArduPlane 的
  glide slope，與規劃器縮點所假設的同一件事）。特例：
  * NAV_VTOL_TAKEOFF／NAV_TAKEOFF：垂直起飛柱不檢查（同 check_kinematics 略過 VTOL 階段），之後那一段（轉換＋加速）
    只檢查 h_clr；
  * NAV_VTOL_LAND：飛向落點那一段**保持前一點高度**（ArduPlane ``QuadPlane::do_vtol_land`` 先把目標高度設成目前
    高度，Plane-4.6.3 quadplane.cpp:3369-3372「initially aim for current altitude」），垂直下降柱不檢查；
  * 盤旋（LOITER_TO_ALT／TURNS／TIME／UNLIM）：整個圓周取樣，高度一律取「抵達高度」與「指令高度」較低者（保守：
    盤旋爬升時第一圈貼著抵達高度）。半徑：TO_ALT 取 |param2|、其餘取 |param3|；0 ⇒ ``loiter_radius_m``，
    也沒給 ⇒ 只取圓心並列入 caveats。
* 走廊：``corridor_half_width_m`` > 0 時另算「走廊內最高地形」的 AGL（``TerrainGrid.corridor_max``，同
  check_kinematics）；**判定**用走廊值（與規劃器 h_clr 的語意一致：飛機不會剛好在中心線上），中心線值一起回報。
* 門檻：每個取樣點 AGL < ``h_clr_m`` − ``tol_m`` ⇒ ``agl_below_h_clr``；固定翼作業階段 < ``h_minfw`` − ``tol_m``
  − ``minfw_slack_m`` ⇒ ``agl_below_h_minfw``。``tol_m`` 預設 1 m（同 check_kinematics 的 ``_AGL_TOL_M``）。
  ``minfw_slack_m``（掛點給 ``cfg.delta_prof_m`` ＝ 10 m）：匯出器縮點本來就允許高度剖面偏離這麼多，實測 CLIMB→ENTRY、
  RTB→DESCENT 交界會因此比 h_minfw 低 1–10 m（每份計畫都有）；容差內的下探列在 ``within_slack``（摘要寫出，不算問題），
  超過才是問題。h_clr 不給這個容差。
* 階段：給了 ``phases``（:func:`nav_phases_from_plan`：把上傳項目對回規劃器的縮點，縮點之間依密集航點的沿線比例
  **逐取樣點**給階段，同 check_kinematics「密集段 j 上的點屬 wps[j].phase」的慣例）⇒ 用規劃器階段，h_minfw 適用
  ORBIT／ENTRY／OPERATE／TRANSFER／RTB／HOLD（ACCEL／CLIMB／DESCENT／DECEL 豁免，D35）。沒給或對不上 ⇒ 依任務
  結構分段（起飛後第一段、相機開著、DO_LAND_START 之後、其他），h_minfw **只**檢查相機開著的段，並列入 caveats
  （檢查變弱要說出來）。

誠實原則（有效值契約、失敗要大聲）：

* DEM 外（地形 nan）的取樣點**不當 0**：計數、列 ``terrain_unknown`` 問題，該點不參與最小值；home 在 DEM 外 ⇒
  整架無法評估，列 ``home_outside_dem``。
* 不支援的高度框架（非 frame 3／6 的導航點）⇒ ``unsupported_frame``，不猜。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

#: 沿線取樣間距（m）：與規劃器 ``PlannerConfig.d_d_m`` 預設相同
DEFAULT_SAMPLE_M = 25.0
#: AGL 判定容差（m）：與 ``hv_dccpp.sim._AGL_TOL_M`` 相同
AGL_TOL_M = 1.0
#: 第 0 列 home 的 alt（規劃器的坪高）與 DEM 在該點的地面差超過這麼多 ⇒ caveats
HOME_GROUND_TOL_M = 1.0
#: 對齊上傳項目與規劃器縮點時的水平容差（m）；兩者由同一個 to_wgs84 算出，實際差 ~1e-9 m
ALIGN_TOL_M = 0.5

# MAVLink 指令／框架（與 pymavlink 列舉同值；不 import pymavlink，保持純函式）
CMD_NAV_WAYPOINT = 16
CMD_NAV_LOITER_UNLIM = 17
CMD_NAV_LOITER_TURNS = 18
CMD_NAV_LOITER_TIME = 19
CMD_NAV_LAND = 21
CMD_NAV_TAKEOFF = 22
CMD_NAV_LOITER_TO_ALT = 31
CMD_NAV_VTOL_TAKEOFF = 84
CMD_NAV_VTOL_LAND = 85
CMD_NAV_DELAY = 93
CMD_DO_LAND_START = 189
CMD_DO_SET_CAM_TRIGG_DIST = 206
_LOITER_CMDS = frozenset({CMD_NAV_LOITER_UNLIM, CMD_NAV_LOITER_TURNS, CMD_NAV_LOITER_TIME, CMD_NAV_LOITER_TO_ALT})
_TAKEOFF_CMDS = frozenset({CMD_NAV_VTOL_TAKEOFF, CMD_NAV_TAKEOFF})
_LAND_CMDS = frozenset({CMD_NAV_VTOL_LAND, CMD_NAV_LAND})
#: 帶座標的導航指令（NAV_DELAY 沒有位置，略過）
_POS_NAV_CMDS = frozenset({CMD_NAV_WAYPOINT}) | _LOITER_CMDS | _TAKEOFF_CMDS | _LAND_CMDS
#: 相對高度框架：GLOBAL_RELATIVE_ALT（3）與其 _INT 版（6）
_REL_FRAMES = frozenset({3, 6})

#: 規劃器階段：h_minfw 適用者（FW_PHASES 去掉 D35 豁免的 ACCEL／CLIMB／DESCENT／DECEL）
MINFW_PHASES = frozenset({"ORBIT", "ENTRY", "OPERATE", "TRANSFER", "RTB", "HOLD"})
#: 垂直段（不檢查）
VERTICAL_STAGES = frozenset({"TAKEOFF", "LAND"})
#: 依任務結構分段時的階段名（沒有規劃器階段可用）
STRUCT_CLIMB_OUT = "climb_out"
STRUCT_OPERATE = "operate"
STRUCT_LANDING = "landing"
STRUCT_ENROUTE = "enroute"
STRUCT_LOITER = "loiter"


class TerrainClearanceError(ValueError):
    """輸入不能評估（例如沒有 home 列）。訊息是給操作員看的繁中說明。"""


# ─────────────────────────────────────────────────────────────────────
#  資料型別
# ─────────────────────────────────────────────────────────────────────
@dataclass
class StageMin:
    """某階段（或全程）最小 AGL 的那個取樣點。``agl_m`` 依判定基準（走廊或中心線）；兩者都附上。"""
    stage: str
    agl_m: float
    agl_centerline_m: float
    agl_corridor_m: float | None
    lat_deg: float
    lon_deg: float
    alt_rel_m: float
    seq: int                       # 該段出發點的任務項目 seq（盤旋 ＝ 該 LOITER 項目）


@dataclass
class ClearanceProblem:
    """一類違規（每架每階段每種一筆）。``kind``：agl_below_h_clr／agl_below_h_minfw／terrain_unknown／
    home_outside_dem／unsupported_frame。"""
    uav: Any
    kind: str
    stage: str | None = None
    agl_m: float | None = None
    limit_m: float | None = None
    lat_deg: float | None = None
    lon_deg: float | None = None
    seq: int | None = None
    n: int = 1
    detail: str = ""

    def text(self) -> str:
        who = f"UAV-{self.uav}"
        where = (f" @({self.lat_deg:.6f},{self.lon_deg:.6f}) seq {self.seq}"
                 if self.lat_deg is not None and self.lon_deg is not None else "")
        if self.kind in ("agl_below_h_clr", "agl_below_h_minfw", "agl_below_h_minfw_within_slack"):
            lim = "h_clr" if self.kind == "agl_below_h_clr" else "h_minfw"
            return (f"{who} {self.stage} 最小 AGL {self.agl_m:.1f} m < {lim} {self.limit_m:.0f} m"
                    f"（{self.n} 個取樣點）{where}")
        if self.kind == "terrain_unknown":
            return f"{who}：{self.n} 個取樣點在 DEM 外 ⇒ 地形未知（不當 0）{where}"
        return f"{who} {self.kind}：{self.detail}{where}"


@dataclass
class DroneClearance:
    uav: Any
    home_lat_deg: float
    home_lon_deg: float
    home_ground_msl_m: float              # DEM 在第 0 列的地面（AGL 的基準）
    home_item_alt_m: float                # 第 0 列的 alt（規劃器的坪高）
    phase_source: str                     # "plan" | "structure"
    n_samples: int = 0
    n_unknown: int = 0
    min_all: StageMin | None = None       # 全程（判定基準）
    min_centerline: StageMin | None = None
    min_corridor: StageMin | None = None
    by_stage: dict[str, StageMin] = field(default_factory=dict)
    problems: list[ClearanceProblem] = field(default_factory=list)
    #: 低於 h_minfw 但在 ``minfw_slack_m`` 內（縮點容差能解釋的下探）：列在摘要，不算問題
    within_slack: list[ClearanceProblem] = field(default_factory=list)
    caveats: list[str] = field(default_factory=list)


@dataclass
class ClearanceReport:
    drones: dict[Any, DroneClearance]
    h_clr_m: float
    h_minfw_m: dict[Any, float]
    sample_m: float
    corridor_half_width_m: float
    tol_m: float
    terrain_label: str = ""
    minfw_slack_m: float = 0.0

    @property
    def problems(self) -> list[ClearanceProblem]:
        return [p for d in self.drones.values() for p in d.problems]

    @property
    def ok(self) -> bool:
        return not self.problems

    @property
    def basis(self) -> str:
        return f"走廊 ±{self.corridor_half_width_m:g} m 最高地形" if self.corridor_half_width_m > 0 else "中心線地面"

    def summary_note(self) -> str:
        """``terrain_clearance(...)``：每架最小 AGL、位置、階段；門檻；地形來源；caveats。"""
        parts = []
        for uav, d in self.drones.items():
            m = d.min_all
            if m is None:
                parts.append(f"UAV-{uav} 無可評估取樣點")
                continue
            c = f"、中心線 {d.min_centerline.agl_m:.1f} m" if (
                self.corridor_half_width_m > 0 and d.min_centerline is not None) else ""
            parts.append(f"UAV-{uav} 最小 {m.agl_m:.1f} m（{m.stage}{c}）@({m.lat_deg:.6f},{m.lon_deg:.6f})")
        cav = [f"UAV-{u}：{c}" for u, d in self.drones.items() for c in d.caveats]
        slk = [p.text() for d in self.drones.values() for p in d.within_slack]
        if slk:
            slk = [f"h_minfw 下探在縮點容差 {self.minfw_slack_m:g} m 內（匯出器高度剖面本來就允許這麼多，不算問題）："
                   + "、".join(slk)]
        minfw = sorted({float(v) for v in self.h_minfw_m.values()})
        head = (f"上傳任務沿線 AGL（每 {self.sample_m:g} m、{self.basis}"
                + (f"、地形 {self.terrain_label}" if self.terrain_label else "") + "）")
        tail = (f"門檻 h_clr {self.h_clr_m:g} m／固定翼 h_minfw {'/'.join(f'{v:g}' for v in minfw)} m"
                + (f"（h_minfw 扣縮點容差 {self.minfw_slack_m:g} m 才算問題）" if self.minfw_slack_m > 0 else ""))
        body = "；".join(parts + [tail] + slk + cav)
        return f"terrain_clearance({head}：{body})"

    def problem_notes(self) -> list[str]:
        """``terrain_clearance_failed(...)``（每架一則；空 ＝ 通過）。"""
        out = []
        for uav, d in self.drones.items():
            if d.problems:
                out.append("terrain_clearance_failed(上傳任務" + "；".join(p.text() for p in d.problems)
                           + f"；判定基準 {self.basis})")
        return out


# ─────────────────────────────────────────────────────────────────────
#  任務項目
# ─────────────────────────────────────────────────────────────────────
@dataclass
class _Item:
    seq: int
    command: int
    frame: int
    lat: float
    lon: float
    alt: float
    p1: float
    p2: float
    p3: float
    p4: float


def _norm_item(it: Any, i: int) -> _Item:
    """dict（mission_item_dicts：lat/lon/alt）或 MissionItem（x/y/z）→ _Item。"""
    if isinstance(it, Mapping):
        g = it.get
        lat = g("lat", g("x"))
        lon = g("lon", g("y"))
        alt = g("alt", g("z"))
        cmd, frm = g("command"), g("frame")
        p = (g("param1", 0.0), g("param2", 0.0), g("param3", 0.0), g("param4", 0.0))
        seq = g("seq", i)
    else:
        lat = getattr(it, "lat", getattr(it, "x", None))
        lon = getattr(it, "lon", getattr(it, "y", None))
        alt = getattr(it, "alt", getattr(it, "z", None))
        cmd, frm = getattr(it, "command", None), getattr(it, "frame", None)
        p = tuple(getattr(it, f"param{k}", 0.0) for k in range(1, 5))
        seq = getattr(it, "seq", i)
    if cmd is None or frm is None or lat is None or lon is None or alt is None:
        raise TerrainClearanceError(f"任務項目 #{i} 缺 command／frame／lat／lon／alt：{it!r}")
    return _Item(int(seq if seq is not None else i), int(cmd), int(frm), float(lat), float(lon), float(alt),
                 *(float(v or 0.0) for v in p))


def _loiter_radius(it: _Item) -> float:
    r = abs(it.p2) if it.command == CMD_NAV_LOITER_TO_ALT else abs(it.p3)
    return float(r)


# ─────────────────────────────────────────────────────────────────────
#  階段對齊（上傳項目 ↔ 規劃器縮點）
# ─────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class LegPhases:
    """一段（某導航列 → 下一個導航列）沿線的規劃器階段：``breaks[i]`` 起（沿線比例 0–1）是 ``names[i]``。

    盤旋列另有 ``circle``（圓周上的階段）。``at(f)`` → 每個取樣點的階段名。"""
    breaks: tuple[float, ...]
    names: tuple[str, ...]
    circle: str | None = None

    @staticmethod
    def single(name: str, circle: str | None = None) -> "LegPhases":
        return LegPhases((0.0,), (str(name),), circle)

    def at(self, f: np.ndarray) -> np.ndarray:
        k = np.searchsorted(np.asarray(self.breaks), np.asarray(f, dtype=float), side="right") - 1
        return np.asarray(self.names, dtype=object)[np.clip(k, 0, len(self.names) - 1)]


def _dense_profile(wps: Sequence[Any], a: int, b: int, fallback: str) -> LegPhases:
    """密集航點 a → b 之間的階段剖面（依沿線弦長比例；VTOL 垂直段權重 0）。check_kinematics 的慣例：
    密集段 j 上的取樣點屬 ``wps[j].phase``。"""
    if b <= a:
        return LegPhases.single(fallback)
    L: list[float] = []
    N: list[str] = []
    for j in range(a, b):
        p, q = wps[j], wps[j + 1]
        if p.phase.name in VERTICAL_STAGES:
            continue
        L.append(math.hypot(q.x_m - p.x_m, q.y_m - p.y_m))
        N.append(p.phase.name)
    tot = float(sum(L))
    if tot <= 1e-9:
        return LegPhases.single(N[0] if N else fallback)
    br: list[float] = []
    nm: list[str] = []
    s = 0.0
    for length, name in zip(L, N):
        if length > 0.0 and (not nm or nm[-1] != name):
            br.append(s / tot)
            nm.append(name)
        s += length
    br[0] = 0.0
    return LegPhases(tuple(br), tuple(nm))


def nav_phases_from_plan(up: Any, ctx: Any, cfg: Any, items: Sequence[Any]) -> list[LegPhases | None] | None:
    """上傳項目 → 每個導航列**出發段**的規劃器階段剖面（非導航列 None）；對不上 ⇒ None（呼叫端退回依結構分段並說出來）。

    重做 ``plan_to_mission_items`` 的縮點（``reduce_waypoints``，接受半徑 min(R_eff(W_max)/2, 30)、z 容差
    ``cfg.delta_prof_m``——與 ``HvDccppCoordinator.mission_items`` 同參數），依序把帶座標的導航列對到縮點
    （水平 ≤ :data:`ALIGN_TOL_M`、種類相符）；對不到的 NAV_WAYPOINT 若緊接在 OPERATE 之後 ⇒ 建圖引入段（同條帶）。
    所有縮點都要用完，否則回 None。

    剖面：縮點 k → k+1 之間的密集航點（``src``）依沿線比例給階段——縮點會把 RTB 的末段與 DESCENT 併成一段，
    整段標出發點的 RTB 會把 DESCENT（h_minfw 豁免）誤判成 RTB 低飛（mini 情境實測 A1 100.9 m）。起飛列之後那段
    ＝ 轉換＋加速（密集的 TRANS_OUT／ACCEL）；盤旋列：圓周 ＝ 該縮點的階段（ORBIT），離開段 ＝ 下一個縮點的階段。
    """
    from hv_dccpp.ardupilot import accept_radius_m, reduce_waypoints
    wps = getattr(up, "waypoints", None) or []
    if not wps:
        return None
    spec = ctx.specs[up.cls]
    w_ref = ctx.wind.max_speed() if getattr(ctx, "wind", None) is not None else 0.0
    pts = reduce_waypoints(wps, spec, cfg, accept_radius_m(spec.r_eff(w_ref)))
    norm = [_norm_item(it, i) for i, it in enumerate(items)]
    want_kind = {CMD_NAV_WAYPOINT: ("wp",), CMD_NAV_VTOL_TAKEOFF: ("takeoff",), CMD_NAV_VTOL_LAND: ("land",),
                 CMD_NAV_LOITER_TO_ALT: ("loiter_alt",), CMD_NAV_LOITER_TURNS: ("loiter_turns",),
                 CMD_NAV_LOITER_TIME: ("loiter_time",)}
    match: list[tuple[int, int | None]] = []          # (item 列, 縮點索引；引入段 None)
    k = 0
    prev_phase: str | None = None
    for j, it in enumerate(norm):
        if j == 0 or it.command not in _POS_NAV_CMDS:
            continue
        if it.command not in want_kind:
            return None
        xy = ctx.frame.to_enu_xy(it.lat, it.lon)[0]
        if k < len(pts):
            p = pts[k]
            if p.kind in want_kind[it.command] and math.hypot(p.x - xy[0], p.y - xy[1]) <= ALIGN_TOL_M:
                match.append((j, k))
                prev_phase = p.phase.name
                k += 1
                continue
        if it.command == CMD_NAV_WAYPOINT and prev_phase == "OPERATE":
            match.append((j, None))       # 建圖 lead-in：與條帶起點共線、同條帶
            continue
        return None
    if k != len(pts):
        return None
    out: list[LegPhases | None] = [None] * len(norm)
    for m, (j, kk) in enumerate(match):
        if kk is None:
            out[j] = LegPhases.single("OPERATE")
            continue
        p = pts[kk]
        nxt = next((k2 for _j2, k2 in match[m + 1:] if k2 is not None), None)
        if p.kind.startswith("loiter"):
            after = pts[nxt].phase.name if nxt is not None else p.phase.name
            out[j] = LegPhases.single(after, circle=p.phase.name)
        elif nxt is None:
            out[j] = LegPhases.single(p.phase.name)
        else:
            fb = "TRANS_OUT" if p.kind == "takeoff" else p.phase.name
            out[j] = _dense_profile(wps, int(p.src), int(pts[nxt].src), fb)
    return out


# ─────────────────────────────────────────────────────────────────────
#  取樣
# ─────────────────────────────────────────────────────────────────────
class _Samples:
    """累積取樣點：ENU xy、相對高、航向（走廊用）、階段（逐點）、出發列 seq。"""

    def __init__(self) -> None:
        self.xy: list[np.ndarray] = []
        self.alt: list[np.ndarray] = []
        self.th: list[np.ndarray] = []
        self.stage: list[np.ndarray] = []
        self.seq: list[np.ndarray] = []

    def add(self, xy: np.ndarray, alt: np.ndarray, th: Any, stage: Any, seq: int) -> None:
        n = len(xy)
        if n == 0:
            return
        self.xy.append(np.asarray(xy, dtype=float).reshape(-1, 2))
        self.alt.append(np.asarray(alt, dtype=float).reshape(-1))
        self.th.append(np.broadcast_to(np.asarray(th, dtype=float), (n,)).astype(float))
        st = np.empty(n, dtype=object)
        st[:] = stage if isinstance(stage, str) else list(stage)
        self.stage.append(st)
        self.seq.append(np.full(n, int(seq), dtype=int))

    def line(self, p0: np.ndarray, a0: float, p1: np.ndarray, a1: float, step: float, stage: str | LegPhases,
             seq: int) -> None:
        d = np.asarray(p1, dtype=float) - np.asarray(p0, dtype=float)
        L = float(math.hypot(d[0], d[1]))
        m = max(1, int(math.ceil(L / step - 1e-9)))
        f = np.linspace(0.0, 1.0, m + 1)
        th = math.atan2(d[0], d[1]) if L > 1e-9 else 0.0
        st = stage.at(f) if isinstance(stage, LegPhases) else stage
        self.add(np.asarray(p0, dtype=float)[None, :] + f[:, None] * d[None, :], a0 + f * (a1 - a0), th, st, seq)

    def circle(self, c: np.ndarray, R: float, alt: float, step: float, stage: str, seq: int) -> None:
        n = max(8, int(math.ceil(2.0 * math.pi * R / step - 1e-9)))
        ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        xy = np.column_stack([c[0] + R * np.cos(ang), c[1] + R * np.sin(ang)])
        th = np.arctan2(-np.sin(ang), np.cos(ang))       # 切線方向（走廊對稱，順逆時針無差）
        self.add(xy, np.full(n, float(alt)), th, stage, seq)


def _stage_struct(cam_on: bool, landing: bool) -> str:
    if landing:
        return STRUCT_LANDING
    if cam_on:
        return STRUCT_OPERATE
    return STRUCT_ENROUTE


def _build_samples(items: Sequence[_Item], phases: Sequence[LegPhases | str | None] | None, frame: Any,
                   home_ground_msl: float,
                   step: float, loiter_radius_m: float | None, caveats: list[str],
                   problems: list[ClearanceProblem], uav: Any) -> _Samples:
    """任務項目 → 沿線取樣（相對高；ENU xy）。

    有規劃器階段 ⇒ 出發導航列的剖面（:class:`LegPhases`，逐取樣點；字串 ＝ 整段同一階段）；依結構 ⇒ 抵達下一個
    導航列時才決定（描述出發段的 DO_ 掛在出發列**之後**，要等它們都讀過）。"""
    S = _Samples()
    home = items[0]

    def xy_of(it: _Item, alt_rel: float) -> np.ndarray:
        return frame.to_enu(it.lat, it.lon, home_ground_msl + alt_rel)[0, :2]

    def lp_of(x: Any) -> LegPhases | None:
        return LegPhases.single(x) if isinstance(x, str) else x

    cur_xy: np.ndarray | None = None       # 目前位置（上一個導航點；盤旋後 ＝ 圓心）
    cur_alt = 0.0
    cur_seq = int(home.seq)
    cur_phase: LegPhases | None = None     # 出發段的規劃器階段剖面（有 phases 時）
    from_takeoff = False                   # 出發列是起飛列
    cur_loiter: tuple[np.ndarray, float] | None = None   # 上一個導航點是盤旋 ⇒ (圓心, 半徑)
    cam_on = False
    landing = False
    for j, it in enumerate(items):
        if j == 0:
            continue
        cmd = it.command
        if cmd == CMD_DO_SET_CAM_TRIGG_DIST:
            cam_on = it.p1 > 0.0
            continue
        if cmd == CMD_DO_LAND_START:
            landing = True
            continue
        if cmd not in _POS_NAV_CMDS:
            continue                                    # 其他 DO_／NAV_DELAY：沒有位置
        if it.frame not in _REL_FRAMES:
            problems.append(ClearanceProblem(uav, "unsupported_frame", seq=it.seq,
                                             detail=f"seq {it.seq} 指令 {cmd} 的高度框架 {it.frame} 不是相對高度（3／6）"
                                                    "⇒ 這架沒評估"))
            return _Samples()
        ph = lp_of(phases[j]) if phases is not None else None
        if cmd in _TAKEOFF_CMDS:
            lat_ok = not (it.lat == 0.0 and it.lon == 0.0)
            cur_xy, cur_alt, cur_seq, cur_loiter = xy_of(it if lat_ok else home, it.alt), it.alt, it.seq, None
            cur_phase, from_takeoff = (ph or LegPhases.single("TRANS_OUT")), True
            continue
        if cur_xy is None and cmd not in _LOITER_CMDS:  # 沒有起飛列：從第一個導航點開始
            cur_xy, cur_alt, cur_seq, cur_phase = xy_of(it, it.alt), it.alt, it.seq, ph
            continue
        if cur_xy is None:
            cur_xy, cur_alt = xy_of(it, it.alt), it.alt
        # 出發段的階段
        if phases is not None:
            leg_stage = cur_phase or STRUCT_ENROUTE
        else:
            leg_stage = STRUCT_CLIMB_OUT if from_takeoff else _stage_struct(cam_on, landing)
        from_takeoff = False
        if cmd in _LAND_CMDS:
            p = xy_of(it, cur_alt)
            start = cur_xy if cur_loiter is None else _loiter_exit(cur_loiter, p)
            S.line(start, cur_alt, p, cur_alt, step, leg_stage, cur_seq)     # 先保持目前高度飛到落點上空
            cur_xy, cur_seq, cur_loiter = p, it.seq, None
            continue
        if cmd in _LOITER_CMDS:
            R = _loiter_radius(it)
            if R <= 0.0:
                if loiter_radius_m is not None and loiter_radius_m > 0.0:
                    R = float(loiter_radius_m)
                else:
                    caveats.append(f"seq {it.seq} 盤旋半徑未知（指令 {cmd} 的半徑參數為 0、也沒給 WP_LOITER_RAD）"
                                   "⇒ 只取圓心")
            c = xy_of(it, it.alt)
            a_c = min(cur_alt, it.alt)
            # 進入：自目前位置飛到圓上最近點，高度往盤旋高度（保守取較低者）
            v = cur_xy - c
            dv = float(math.hypot(v[0], v[1]))
            entry = c + (R / dv) * v if (R > 0.0 and dv > 1e-9) else c
            start = cur_xy if cur_loiter is None else _loiter_exit(cur_loiter, entry)
            S.line(start, cur_alt, entry, a_c, step, leg_stage, cur_seq)
            st = ((ph.circle if ph is not None else None) or "ORBIT") if phases is not None else STRUCT_LOITER
            if R > 0.0:
                S.circle(c, R, a_c, step, st, it.seq)
            else:
                S.add(c[None, :], np.array([a_c]), 0.0, st, it.seq)
            cur_xy, cur_alt, cur_seq, cur_loiter = c, it.alt, it.seq, ((c, R) if R > 0.0 else None)
            cur_phase = ph
            continue
        # NAV_WAYPOINT
        p = xy_of(it, it.alt)
        start = cur_xy if cur_loiter is None else _loiter_exit(cur_loiter, p)
        S.line(start, cur_alt, p, it.alt, step, leg_stage, cur_seq)
        cur_xy, cur_alt, cur_seq, cur_loiter, cur_phase = p, it.alt, it.seq, None, ph
    return S


def _loiter_exit(loiter: tuple[np.ndarray, float], toward: np.ndarray) -> np.ndarray:
    """離開盤旋：圓上朝下一點最近的那一點。"""
    c, R = loiter
    v = np.asarray(toward, dtype=float) - c
    d = float(math.hypot(v[0], v[1]))
    return c + (R / d) * v if d > 1e-9 else c


# ─────────────────────────────────────────────────────────────────────
#  主函式
# ─────────────────────────────────────────────────────────────────────
def uploaded_items_clearance(missions: Mapping[Any, Sequence[Any]], terrain: Any, frame: Any, *,
                             h_clr_m: float, h_minfw_m: float | Mapping[Any, float],
                             phases: Mapping[Any, Sequence[LegPhases | str | None] | None] | None = None,
                             sample_m: float = DEFAULT_SAMPLE_M, corridor_half_width_m: float = 0.0,
                             dv_m: float | None = None, tol_m: float = AGL_TOL_M,
                             loiter_radius_m: float | Mapping[Any, float] | None = None,
                             terrain_label: str = "", minfw_slack_m: float = 0.0) -> ClearanceReport:
    """每架上傳任務的沿線最小 AGL（每階段、全程、位置）＋違規（見模組說明）。

    ``missions``：uav → 任務項目（第 0 列 home）。``terrain``：``TerrainGrid``（**自然地形** ``raw_view()``；ENU z）。
    ``frame``：``GeoFrame``（terrain 的座標框架）。``h_minfw_m``：純量或 uav → 值。``phases``：uav →
    :func:`nav_phases_from_plan` 的結果（None ＝ 依結構分段）。``dv_m``：走廊橫向取樣間距（None ＝ max(step/2, 0.5)，
    同 check_kinematics）。``minfw_slack_m``：h_minfw 下探在這麼多以內 ⇒ 只列 ``within_slack``（摘要），不算問題；
    h_clr 一律不給容差。
    """
    step = float(sample_m)
    if not (math.isfinite(step) and step > 0.0):
        raise TerrainClearanceError(f"sample_m 須 > 0，得 {sample_m!r}")
    slack = float(minfw_slack_m)
    if not (math.isfinite(slack) and slack >= 0.0):
        raise TerrainClearanceError(f"minfw_slack_m 須 ≥ 0，得 {minfw_slack_m!r}")
    hw = float(corridor_half_width_m)
    if not (math.isfinite(hw) and hw >= 0.0):
        raise TerrainClearanceError(f"corridor_half_width_m 須 ≥ 0，得 {corridor_half_width_m!r}")
    dv = float(dv_m) if dv_m is not None else max(0.5 * float(terrain.step_m), 0.5)
    h_clr = float(h_clr_m)
    minfw_of = {u: float(h_minfw_m[u] if isinstance(h_minfw_m, Mapping) else h_minfw_m) for u in missions}
    drones: dict[Any, DroneClearance] = {}
    for uav, raw in missions.items():
        items = [_norm_item(it, i) for i, it in enumerate(raw)]
        if not items:
            raise TerrainClearanceError(f"UAV-{uav}：任務是空的（至少要有第 0 列 home）")
        home = items[0]
        hxy = frame.to_enu_xy(home.lat, home.lon, home.alt)
        hg_enu = terrain.z_at(hxy)
        hg = float(frame.enu_z_to_msl(hxy, hg_enu)[0]) if np.isfinite(hg_enu).all() else math.nan
        ph = None if phases is None else phases.get(uav)
        d = DroneClearance(uav, home.lat, home.lon, hg, home.alt, "plan" if ph is not None else "structure")
        drones[uav] = d
        if not math.isfinite(hg):
            d.problems.append(ClearanceProblem(uav, "home_outside_dem", lat_deg=home.lat, lon_deg=home.lon,
                                               seq=home.seq, detail="起飛坪在 DEM 外 ⇒ 相對高度沒有地面基準，整架無法評估"))
            continue
        if abs(home.alt - hg) > HOME_GROUND_TOL_M:
            d.caveats.append(f"第 0 列 home alt {home.alt:.1f} m 與 DEM 坪地面 {hg:.1f} m 差 {home.alt - hg:+.1f} m"
                             "（飛控 home ＝ 實際地面；任務高度以實際地面為準）")
        if phases is not None and ph is None:
            d.caveats.append("上傳項目對不回規劃器縮點 ⇒ 依任務結構分段，h_minfw 只檢查相機開著的段")
        lr = loiter_radius_m.get(uav) if isinstance(loiter_radius_m, Mapping) else loiter_radius_m
        S = _build_samples(items, ph, frame, hg, step, lr, d.caveats, d.problems, uav)
        if not S.xy:
            continue
        xy = np.vstack(S.xy)
        alt = np.concatenate(S.alt)
        th = np.concatenate(S.th)
        stage = np.concatenate(S.stage)
        seq = np.concatenate(S.seq)
        g_enu = terrain.z_at(xy)
        known = np.isfinite(g_enu)
        g_msl = np.full(len(xy), np.nan)
        if known.any():
            g_msl[known] = frame.enu_z_to_msl(xy[known], g_enu[known])
        agl_c = hg + alt - g_msl
        agl_k: np.ndarray | None = None
        if hw > 0.0:
            zc = terrain.corridor_max(xy, th, hw, dv)
            kc = np.isfinite(zc)
            known &= kc                                 # 走廊任一點在 DEM 外 ⇒ 未知（同 corridor_max 的語意）
            zc_msl = np.full(len(xy), np.nan)
            if kc.any():
                zc_msl[kc] = frame.enu_z_to_msl(xy[kc], zc[kc])
            agl_k = hg + alt - zc_msl
        agl = agl_k if agl_k is not None else agl_c
        d.n_samples = int(len(xy))
        d.n_unknown = int((~known).sum())
        if d.n_unknown:
            k0 = int(np.flatnonzero(~known)[0])
            ll = frame.to_wgs84(np.array([[xy[k0, 0], xy[k0, 1], 0.0]]))[0]
            d.problems.append(ClearanceProblem(uav, "terrain_unknown", stage=str(stage[k0]), lat_deg=float(ll[0]),
                                               lon_deg=float(ll[1]), seq=int(seq[k0]), n=d.n_unknown))

        def rec(k: int, st: str, val: np.ndarray) -> StageMin:
            ll = frame.to_wgs84(np.array([[xy[k, 0], xy[k, 1], 0.0]]))[0]
            return StageMin(st, float(val[k]), float(agl_c[k]),
                            None if agl_k is None else float(agl_k[k]), float(ll[0]), float(ll[1]),
                            float(alt[k]), int(seq[k]))

        checked = known & ~np.isin(stage, list(VERTICAL_STAGES))
        if not checked.any():
            continue
        idx = np.flatnonzero(checked)

        def rec_min(val: np.ndarray) -> StageMin:
            k = int(idx[np.argmin(val[idx])])
            return rec(k, str(stage[k]), val)

        d.min_all = rec_min(agl)
        d.min_centerline = rec_min(agl_c)
        if agl_k is not None:
            d.min_corridor = rec_min(agl_k)
        for st in dict.fromkeys(stage[idx].tolist()):
            sel = idx[stage[idx] == st]
            d.by_stage[st] = rec(int(sel[np.argmin(agl[sel])]), st, agl)
        # 判定
        minfw = minfw_of[uav]
        if ph is not None:
            need_fw = np.isin(stage, list(MINFW_PHASES))
        else:
            need_fw = stage == STRUCT_OPERATE
        below_fw = checked & need_fw & (agl < minfw - tol_m)
        loud_fw = below_fw & (agl < minfw - tol_m - slack)
        if loud_fw.any():                               # 已經是問題的階段不再另列「容差內」
            below_fw &= ~np.isin(stage, list(dict.fromkeys(stage[loud_fw].tolist())))
            below_fw |= loud_fw
        for kind, mask, lim, sink in (("agl_below_h_clr", checked & (agl < h_clr - tol_m), h_clr, d.problems),
                                      ("agl_below_h_minfw", loud_fw, minfw, d.problems),
                                      ("agl_below_h_minfw_within_slack", below_fw & ~loud_fw, minfw, d.within_slack)):
            if not mask.any():
                continue
            for st in dict.fromkeys(stage[mask].tolist()):
                sel = np.flatnonzero(mask & (stage == st))
                k = int(sel[np.argmin(agl[sel])])
                r = rec(k, st, agl)
                sink.append(ClearanceProblem(uav, kind, st, r.agl_m, lim, r.lat_deg, r.lon_deg, r.seq, n=int(len(sel))))
    return ClearanceReport(drones, h_clr, minfw_of, step, hw, float(tol_m), terrain_label, slack)


# ─────────────────────────────────────────────────────────────────────
#  HARVEST 轉接層便利函式（掛點用）
# ─────────────────────────────────────────────────────────────────────
def hv_uploaded_clearance(hv: Any, *, camera_cfg: Any = None, sample_m: float = DEFAULT_SAMPLE_M,
                          corridor_half_width_m: float | None = None, terrain_label: str = "") -> ClearanceReport:
    """最近一次 HARVEST 計畫：每架 ``hv.mission_items(did, camera_cfg=…)`` × ``ctx.terrain.raw_view()``。

    ``camera_cfg`` None ⇒ ``hv.camera_cfg``（建圖模式）或預設 ``CameraTriggerConfig()``（沒開建圖：引入段 0，導航點
    與建圖版相同的縮點）。門檻：``ctx.h_clr_m``、各構型 ``spec.min_fw_operation_agl_m``（生效值，含面板覆寫）。
    走廊半寬預設 ``cfg.b_trk_m``（check_kinematics 非作業段用的同一個）。鍵 ＝ AeroPlan drone_id。

    h_minfw 的容差 ＝ ``cfg.delta_prof_m``（匯出器縮點的高度剖面容差，生效值）：掛點驗的是**同一份地形**上規劃器已經
    驗過的計畫（validate 用密集航點），所以上傳那份比密集航點低的部分只可能來自縮點；在容差內是匯出器本來就允許的
    （實測：CLIMB→ENTRY、RTB→DESCENT 交界提早／延後升降），列在摘要；超過容差才是縮點解釋不了的 ⇒ 問題。
    h_clr 不給容差。
    """
    from hv_dccpp.ardupilot import CameraTriggerConfig
    ctx, plan, cfg = hv.last_ctx, hv.last_plan, hv.cfg
    if ctx is None or plan is None:
        raise TerrainClearanceError("尚未規劃")
    cc = camera_cfg if camera_cfg is not None else (getattr(hv, "camera_cfg", None) or CameraTriggerConfig())
    missions: dict[Any, list] = {}
    phases: dict[Any, list | None] = {}
    minfw: dict[Any, float] = {}
    for did, uid in sorted(hv.id_map.items()):
        up = plan.plans.get(uid)
        if up is None or not up.waypoints:
            continue
        items = hv.mission_items(did, camera_cfg=cc)
        missions[did] = items
        phases[did] = nav_phases_from_plan(up, ctx, cfg, items)
        minfw[did] = float(ctx.specs[up.cls].min_fw_operation_agl_m)
    hw = float(cfg.b_trk_m) if corridor_half_width_m is None else float(corridor_half_width_m)
    # 盤旋半徑一律取任務項目本身（匯出器每個 LOITER 都寫了半徑）；是 0 就列 caveats，不拿別的值猜
    return uploaded_items_clearance(missions, ctx.terrain.raw_view(), ctx.frame, h_clr_m=float(ctx.h_clr_m),
                                    h_minfw_m=minfw, phases=phases, sample_m=sample_m, corridor_half_width_m=hw,
                                    terrain_label=terrain_label, minfw_slack_m=float(cfg.delta_prof_m))
