"""deconflict.py — 4D 分離、起降走廊排程、修復（契約 §13；規格 §13、D18、§11.6–11.7、§12.5）。

層次：第 4 層，依賴 context / wind（以及更低層 mission / geometry / platform）。

規格對應
--------
* §13.2 **同層空間互斥**：同構型團隊之胞多邊形 ⊕ 迴轉帶（D38）互不相交 → 免 4D 檢查
  （:func:`spatially_exclusive`；本實作要求兩帶距離 ≥ ``cfg.d_sep_m``，比「不相交」更保守）。
* §13.3 **4D 分離**：航點依時間戳線性內插（Δt = ``cfg.dt_4d_s``），時間不確定度
  σ_t = min(k_t·(t − t_sync), :data:`SIGMA_T_MAX_S`)（上限為契約偏離，見該常數說明）；
  `cKDTree` 於 (x, y, V_ref·t) 查詢候選對，違規 ⇔ 水平 < d_sep 且垂直 < Δh_sep 且
  |t_a − t_b| < Δt + σ_a + σ_b（:func:`trajectory_samples`、:func:`check_separation`）。
  同胞平行同向作業樣本（皆 OPERATE、航跡差 ≤ 20°）之水平門檻改用 ``cfg.d_sep_lat_m``（D18 側向間距）。
* §13.5 **起降走廊互斥**：:class:`CorridorSchedule` 區間表；起飛 [t0, t0 + Δt_dep]、
  降落 [t_arr(P_tc) − 30, t_land + t_sep,LDG]；同站所有預約互斥（降落優先於起飛由 :func:`repair` 實現）。
* §13.6 **修復（一次算足）**：Δ_req = Δt + σ_a + σ_b − |t_a − t_b|；依序 (i) 起飛延遲／換序（零能量，重排走廊；
  含**提前起飛** :func:`_fit_takeoff_advance` 與**走廊連鎖重排** :func:`_cascade_shift`）
  (ii) 於最近前一轉移插入 n = ⌈Δ_req/T_lap⌉ 圈延遲盤旋（重驗能量；降落時槽優先由連鎖吸收而非加圈數）
  (iii) IDP 重排（本模組僅留 hook：:class:`ResequenceRequest` / :data:`ResequenceHook`）。
  收斂後再做一次**工期壓縮** :func:`_compact_makespan`（把多餘延遲吐回來，不新增任何衝突）。
  不採垂直偏置（D18）：本模組永不改變任何航點之 z。

風中盤旋圈時間 T_lap = 2πR/V_a · circle_time_factor(V_a, W)（規格 §11.6；A、W=5 → 60.9 s，B → 41.9 s）。
"""
from __future__ import annotations

import inspect
import itertools
import math
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Iterable, Literal, Protocol, Sequence

import numpy as np
import shapely
from scipy.spatial import cKDTree

from .context import MissionContext
from .geometry import Cell, turn_band
from .mission import Mode, Phase, PlannerConfig, UAVPlan, Waypoint
from .platform import EnergyModel, PlatformSpec
from .wind import circle_time_factor

__all__ = [
    "LANDING_LEAD_S",
    "GHOST_LANDING_WINDOW_S",
    "HOLD_DISTANCE_M",
    "CODIR_MAX_DIFF_RAD",
    "EVENT_GAP_S",
    "REPAIR_MARGIN_S",
    "SIGMA_T_MAX_S",
    "sigma_t_max_of",
    "INSERTABLE_PHASES",
    "SITE_CORRIDOR_PHASES",
    "SAME_PAD_TOL_M",
    "CASCADE_MAX_UAVS",
    "Conflict",
    "Reservation",
    "CorridorSchedule",
    "ResequenceRequest",
    "ResequenceHook",
    "TeamLike",
    "GhostLike",
    "spatially_exclusive",
    "trajectory_samples",
    "check_separation",
    "repair",
    "orbit_lap_time_s",
    "shift_plan",
    "insert_delay_orbit",
    "orbit_insert_index",
    "default_energy_ok",
    "plan_windows",
]

LANDING_LEAD_S: float = 30.0
"""降落預約起點 = 抵達 P_tc 時刻 − 30 s（規格 §13.5）。"""

GHOST_LANDING_WINDOW_S: tuple[float, float] = (60.0, 180.0)
"""幽靈航跡之降落走廊預約 [ETA − 60, ETA + 180]（規格 §12.5）。"""

HOLD_DISTANCE_M: float = 2000.0
"""走廊衝突之保持點：走廊入口外 2 km（規格 §13.5）。"""

CODIR_MAX_DIFF_RAD: float = math.radians(20.0)
"""同向平行判定之航跡差上限（兩樣本皆 OPERATE 時水平門檻改用 d_sep_lat）。"""

EVENT_GAP_S: float = 10.0
"""同一機對之違規樣本對在 a 之時間軸上間隔超過此值即分為不同事件。"""

REPAIR_MARGIN_S: float = 1.0
"""修復延遲之附加裕度（s）。"""

SIGMA_T_MAX_S: float = 45.0
"""時間不確定度上限：σ_t = min(k_t·(t − t_sync), SIGMA_T_MAX_S)（契約偏離，見 CONTRACT_DEVIATIONS）。

規格 §13.3 寫 σ_t = k_t(t − t_sync)，其中 **t_sync = 最近一次遙測快照時刻**。營運上 t_sync 每個遙測訊框
就被重設；規格 §12.5 更明定失聯逾 ``C2Link.t_ll_s``（預設 60 s）即宣告 LINK_LOSS，該機改以**幽靈航跡**
（自帶 σ_t）處理，不再適用本式。也就是說「連線正常之機」永遠不會累積超過約一分鐘份量的未更新漂移。

但初始規劃取 t_sync = t^0 = 0 並套用到整趟任務，若不設上限，92 min 的山丘情境末段 σ_t 會漲到 ±280 s、
判定視窗 Δt + σ_a + σ_b 達 560 s——此時 4D 檢查已退化成「純空間交叉」測試：該情境 4 筆殘餘「衝突」的
實際時間間隔是 76.6／96.6／219.4／340.0 s（B4-B5、B1-B4 ×2、A2-A3），k_t = 0 時衝突數為 0；
而修復所需延遲發散到 650–2371 s，repair 無法收斂（實測 makespan 7238 → 18571 s）。

取值 45 s：外部審查建議 30–60 s，本實作取其中點。物理意義＝20 m/s 巡航下 ±900 m 的沿航跡不確定度；
兩機通過同一點若相隔 > 2·45 + Δt_4d = 92 s（≈1.84 km 航跡距離）即視為已分離，對「執行已上傳任務、
GPS 導航」之定翼機仍屬非常保守。

山丘情境之敏感度（8 機、repair 後之殘餘衝突／makespan）：
inf → 0 / 6522 s、90 s → 0 / 6415 s、60 s → 0 / 6106 s、**45 s → 0 / 4175 s**、30 s → 0 / 4910 s
（未設上限之原始版本為 4 筆殘餘 / 5552 s）。makespan 對本參數並非單調——修復是貪婪搜尋，
不同的視窗寬度會走到不同的局部解；殘餘衝突數則在任何上限下都是 0。
以 ``cfg.sigma_t_max_s``（若 PlannerConfig 日後提供）覆寫；``math.inf`` = 回到規格原式。"""

SITE_CORRIDOR_PHASES: frozenset[Phase] = frozenset({
    Phase.TAKEOFF, Phase.TRANS_OUT, Phase.TRANS_IN, Phase.LAND,
})
"""起降走廊（旋翼／轉換）階段：此類樣本之互斥由 :class:`CorridorSchedule`（規格 §13.5）負責，
不重複計入 4D 分離（否則同一站點之起降在 σ_t 視窗下必然互相「衝突」，且需求延遲隨 σ_t 發散）。

**豁免三條件**（缺一不可，見 :func:`_track_from_plan`／:func:`_pair_metrics`）：
(1) 兩樣本之航點階段皆屬本集合；(2) 兩樣本之時刻皆落在該機「自己的」走廊預約區間內
（起飛 [t0, t0+Δt_dep]、降落 [t_arr(P_tc)−30, t_land+t_sep,LDG]；:func:`plan_windows`）；
(3) 兩樣本對應之起降**站點身分**相同（起飛段 = ``UAVPlan.takeoff_site``、降落段 =
``UAVPlan.landing_site``；vendored 修改 #8）。如此每一筆被豁免之樣本對，其兩機在該站必各有一筆
預約涵蓋該時刻，即必然落入 :meth:`CorridorSchedule.conflicts` 的檢查範圍。

⚠⚠⚠ 條件 (3) 從前比的是場坪**座標** ≤ :data:`SAME_PAD_TOL_M`，那是**錯的 proxy**：排程以
``site_id`` 字串為鍵，兩個相距 30 m 的**不同**站不會互斥，卻會被豁免（實測數字見該常數）。
計畫未帶 ``takeoff_site``（手搭 UAVPlan）時仍退回座標比較，此時該缺陷依然存在。"""

SAME_PAD_TOL_M: float = 50.0
"""走廊豁免之「同一場坪」判定容差（m）——**僅用於計畫未帶 `takeoff_site` 之退回路徑**。

⚠⚠⚠ 2026-09-09 實測缺陷（vendored 修改 #8 修正）：原本豁免比的是場坪**座標** ≤ 此值，
而 :class:`CorridorSchedule` 的互斥是以 ``site_id`` **字串**為鍵。兩者不一致 ⇒
「兩個**不同**的站相距 < 50 m」時，豁免成立但排程**不會**把它們排進同一條隊列，
於是真違規被靜靜丟掉。4 台、每機一個起飛站的實測（`per_uav_takeoff_sites`）：

===========  ==================  =============  ==============  ==============
起飛站間距   被豁免的真違規      其中真同站     其中**不同站**  最近的一筆
===========  ==================  =============  ==============  ==============
全部同點     0                   0              0               —
200 m        0                   0              0               —
100 m        0                   0              0               —
60 m         496                 **0**          **496**         30.0 m / 6.0 m
30 m         1037                **0**          **1037**        15.0 m / 3.6 m
15 m         1062                **0**          **1062**        7.5 m / 22.3 m
===========  ==================  =============  ==============  ==============

門檻是水平 100 m／垂直 50 m，而 ``validate`` **每一組都回報通過**。失效線正好落在本常數上：
> 100 m 時豁免根本不觸發，所以問題在 50 m 以下才現形。最嚴重一筆是**一機起飛、一機降落**
（B2 於 t=756 s 爬過 20 m，B3 於 t=766 s 降到 14 m，水平 30 m），兩個不同的站。

⚠⚠ 舊 docstring 寫「不同站（相距數 km）」—— 那個假設在每機一個起飛站之前都成立，
所以缺陷潛伏至今。改用身分判定後修復器解得掉：最小水平間距 7.5→101.6 m（15 m 間距），
代價是 makespan +7%～26%。"""

CASCADE_MAX_UAVS: int = 16
"""走廊連鎖重排（:func:`_cascade_shift`）之最大迭代／最大移動機數。"""

INSERTABLE_PHASES: frozenset[Phase] = frozenset({
    Phase.CLIMB, Phase.ORBIT, Phase.ENTRY, Phase.TRANSFER, Phase.RTB, Phase.HOLD, Phase.DESCENT,
})
"""可插入延遲盤旋之航點階段（固定翼、非作業、非加減速／轉換）。"""

_EPS = 1e-9


# ---------------------------------------------------------------------------
# 資料型別
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class Conflict:
    """一筆衝突（契約 §13）。

    契約欄位：``uav_a/uav_b``（4d 依字串排序、ghost 時 uav_b 為幽靈）、``t_s``（a 之時刻）、``d_h_m/d_v_m``
    （最近點水平／垂直距離；走廊衝突為 0）、``idx_a/idx_b``（4d：`trajectory_samples` 樣本索引；corridor：
    `CorridorSchedule.reservations` 索引）、``kind``。

    尾端附加欄位（契約外，皆有預設；記錄於 CONTRACT_DEVIATIONS）：
    ``dt_req_s`` = 事件內最大 Δ_req = Δt + σ_a + σ_b − |t_a − t_b|（走廊：重疊秒數）；
    ``dt_ab_s`` = t_b − t_a（最近點；> 0 → b 較晚）；``delay_a_s/delay_b_s`` = 只延遲 a／只延遲 b 即可消除整個事件之
    最小延遲（含 σ 隨延遲成長 1/(1−k_t) 與 :data:`REPAIR_MARGIN_S`）；``site_id/res_kinds`` 走廊衝突之站點與預約種類；
    ``forbid_a/forbid_b`` = 「只延遲 a／b」之**禁止延遲量開區間**（合併、依左端排序）：延遲量落在其中即仍衝突
    （``delay_a_s`` = 自 0 向上跳出 ``forbid_a`` 之第一點）。修復把起飛延遲擠到更晚之走廊時槽時必須同時避開這些
    區間，否則「延遲後又撞上另一次交會」（:func:`_fit_takeoff_shift`）。
    ``forbid_now_a/forbid_now_b`` = 同上但**只由目前真的違規之樣本對**構成（``forbid_*`` 則由該機對「任意時差下
    空間相近」之全部樣本對構成）。前者 = 「一次解掉現有全部交會」所需之最小延遲；後者 = 再加上「延遲後整趟
    任務都不會出現新交會」的強保證。共用轉場航路之同型機（如山丘之 B1/B2，最近距離 0 m）後者可達 2000 s 以上，
    故修復兩者都當候選，由 :meth:`_Repair.involved_score` 的全域單調性決定採用哪一個。
    """
    uav_a: str
    uav_b: str
    t_s: float
    d_h_m: float
    d_v_m: float
    idx_a: int
    idx_b: int
    kind: Literal["4d", "corridor", "ghost"]
    dt_req_s: float = 0.0
    dt_ab_s: float = 0.0
    delay_a_s: float = 0.0
    delay_b_s: float = 0.0
    site_id: str | None = None
    res_kinds: tuple[str, str] | None = None
    forbid_a: tuple[tuple[float, float], ...] = ()
    forbid_b: tuple[tuple[float, float], ...] = ()
    forbid_now_a: tuple[tuple[float, float], ...] = ()
    forbid_now_b: tuple[tuple[float, float], ...] = ()

    @property
    def pair(self) -> tuple[str, str]:
        return (self.uav_a, self.uav_b)

    def key(self) -> tuple:
        return (self.uav_a, self.uav_b, self.kind, int(self.idx_a), int(self.idx_b))

    def involves(self, uav_id: str) -> bool:
        return uav_id == self.uav_a or uav_id == self.uav_b


@dataclass(slots=True)
class Reservation:
    """走廊預約區間 [t_start, t_end]（同站所有預約互斥）。"""
    site_id: str
    kind: str
    t_start_s: float
    t_end_s: float
    uav_id: str

    def overlaps(self, t_start_s: float, t_end_s: float) -> bool:
        return self.t_start_s < t_end_s - _EPS and t_start_s < self.t_end_s - _EPS


@dataclass(slots=True)
class ResequenceRequest:
    """(iii) IDP 重排（規格 §13.6 步 (iii)）之回呼輸入。

    當 (i) 起飛延遲／換序 與 (ii) 延遲盤旋 皆無法消除某筆衝突時，:func:`repair` 以本物件呼叫
    ``resequence_hook``，請求呼叫端（`planner.py`）對**該衝突涉及之機**重跑 IDP（交換條帶方向／改同向
    轉場為零成本選項），回傳新的 UAVPlan。

    欄位
    ----
    conflict
        待修之 :class:`Conflict`（``kind`` ∈ {"4d", "corridor", "ghost"}；``dt_req_s``、``delay_a_s/b_s``、
        ``forbid_a/forbid_b`` 給出需求延遲與禁止延遲區間）。
    uav_ids
        本次可重排之機（依偏好排序：makespan 貢獻小者在前；不含幽靈、不含 ``plans`` 以外者）。
    plans
        **目前**全隊計畫（含已被本輪修復改過者）；``plans[u]`` 即 ``u`` 的現行 UAVPlan。回呼只讀。
    schedule
        目前之 :class:`CorridorSchedule`（已含所有機之起降預約）。回呼只讀；``repair`` 會在套用後重排。
    ctx / cfg
        :class:`~hv_dccpp.context.MissionContext` 與 :class:`~hv_dccpp.mission.PlannerConfig`。
    models
        ``uav_id -> EnergyModel``（能量重驗用；缺者見 ``ctx.models``）。
    t_now_s / t_sync_s
        現在時刻（已起飛者不可改起飛時刻）與遙測同步時刻（σ_t 基準）。
    takeoff_sites
        ``uav_id -> 起飛站 site_id``。
    energy_check
        ``repair`` 使用之能量重驗函式 ``(UAVPlan, EnergyModel|None) -> bool``；回呼**應**先自行以它驗過再回傳。

    回傳
    ----
    ``dict[str, UAVPlan]``（可一次改多機；鍵須是 ``uav_ids`` 之子集）、單一 ``UAVPlan``（視為改
    ``uav_ids[0]``）或 ``None``（放棄，該衝突留為殘餘）。``repair`` 會對回傳結果重跑該機對之乾跑與
    全域單調性檢查（:meth:`_Repair.involved_score`），不通過即忽略並續留殘餘衝突——因此回呼可以安全地
    回傳「盡力而為」的結果。回呼**不得**改變任何航點之 z（D18）。
    """
    conflict: Conflict
    uav_ids: tuple[str, ...]
    plans: dict[str, UAVPlan]
    schedule: "CorridorSchedule"
    ctx: MissionContext
    cfg: PlannerConfig
    models: dict[str, EnergyModel]
    t_now_s: float
    t_sync_s: float
    takeoff_sites: dict[str, str]
    energy_check: Callable[[UAVPlan, EnergyModel | None], bool]

    def plan_of(self, uav_id: str) -> UAVPlan:
        return self.plans[uav_id]

    def model_of(self, uav_id: str) -> EnergyModel | None:
        return self.models.get(uav_id) or self.ctx.models.get(uav_id)


ResequenceHook = Callable[[ResequenceRequest], "dict[str, UAVPlan] | UAVPlan | None"]
"""(iii) IDP 重排回呼型別：``hook(ResequenceRequest) -> dict[str, UAVPlan] | UAVPlan | None``。

為相容既有呼叫端，:func:`repair` 亦接受舊式三引數形式 ``hook(uav_id, plan, conflict) -> UAVPlan | None``
（以 :func:`inspect.signature` 之位置參數個數判定）。"""


class TeamLike(Protocol):
    """`allocate.Team` 之鴨子型別（本模組只用 cls / cell_ids / uav_ids）。"""
    cls: str
    cell_ids: Sequence[str]
    uav_ids: Sequence[str]


class GhostLike(Protocol):
    """`dynamic.GhostTrack` 之鴨子型別：``samples`` (M,4) [x,y,z,t]、``sigma_t`` 常數。"""
    uav_id: str
    samples: np.ndarray
    sigma_t: float
    landing_site: str
    eta_s: float


# ---------------------------------------------------------------------------
# 走廊排程（規格 §13.5）
# ---------------------------------------------------------------------------

class CorridorSchedule:
    """起降走廊區間表。

    * :meth:`reserve` 一律登記並回傳「登記前該站是否無重疊」（False 仍登記，供 :meth:`conflicts` 列出）。
    * 起飛與降落共用同一走廊：同站任何種類預約互相排斥（規格 §13.5「起降走廊互斥」）。
    * 區間相接（t_end == t_start）不算重疊。
    """

    def __init__(self, cfg: PlannerConfig) -> None:
        self.cfg = cfg
        self.reservations: list[Reservation] = []

    # -- 基本操作 -------------------------------------------------------------
    def reserve(self, site_id: str, kind: Literal["takeoff", "landing"], t_start_s: float, t_end_s: float,
                uav_id: str) -> bool:
        """登記預約；回傳登記前是否與同站既有預約無重疊。

        landing：t_start = 抵達 P_tc 時刻（UAVPlan.t_arr_ptc_s）− 30 s、t_end = 觸地時刻 + cfg.t_sep_ldg_s；
        takeoff：[t0, t0 + dt_dep]（見 :meth:`reserve_plan`）。
        """
        if kind not in ("takeoff", "landing"):
            raise ValueError(f"kind 須為 'takeoff' 或 'landing'，得 {kind!r}")
        t_start_s, t_end_s = float(t_start_s), float(t_end_s)
        if not (t_end_s >= t_start_s):
            raise ValueError("t_end_s 須 ≥ t_start_s")
        free = self.is_free(site_id, t_start_s, t_end_s)
        self.reservations.append(Reservation(str(site_id), str(kind), t_start_s, t_end_s, str(uav_id)))
        return free

    def is_free(self, site_id: str, t_start_s: float, t_end_s: float, exclude_uav: str | None = None) -> bool:
        """該站 [t_start, t_end] 是否與既有預約（可排除某機）無重疊。"""
        for r in self.reservations:
            if r.site_id != site_id or (exclude_uav is not None and r.uav_id == exclude_uav):
                continue
            if r.overlaps(t_start_s, t_end_s):
                return False
        return True

    def release(self, uav_id: str, kind: str | None = None) -> int:
        """移除某機（可限種類）之預約；回傳移除筆數。"""
        before = len(self.reservations)
        self.reservations = [r for r in self.reservations
                             if not (r.uav_id == uav_id and (kind is None or r.kind == kind))]
        return before - len(self.reservations)

    def reservations_of(self, uav_id: str) -> list[Reservation]:
        return [r for r in self.reservations if r.uav_id == uav_id]

    def copy(self) -> "CorridorSchedule":
        other = CorridorSchedule(self.cfg)
        other.reservations = [replace(r) for r in self.reservations]
        return other

    # -- 查詢 -----------------------------------------------------------------
    def conflicts(self) -> list[Conflict]:
        """同站重疊之預約對（kind="corridor"；uav_a/uav_b 依字串排序；t_s = 重疊起點；dt_req_s = 重疊秒數）。"""
        out: list[Conflict] = []
        res = self.reservations
        for i in range(len(res)):
            for j in range(i + 1, len(res)):
                a, b = res[i], res[j]
                if a.site_id != b.site_id or a.uav_id == b.uav_id:
                    continue
                if not a.overlaps(b.t_start_s, b.t_end_s):
                    continue
                ia, ib = (i, j) if a.uav_id <= b.uav_id else (j, i)
                ra, rb = res[ia], res[ib]
                t0 = max(ra.t_start_s, rb.t_start_s)
                t1 = min(ra.t_end_s, rb.t_end_s)
                out.append(Conflict(
                    uav_a=ra.uav_id, uav_b=rb.uav_id, t_s=t0, d_h_m=0.0, d_v_m=0.0, idx_a=ia, idx_b=ib,
                    kind="corridor", dt_req_s=t1 - t0, dt_ab_s=rb.t_start_s - ra.t_start_s,
                    delay_a_s=(rb.t_end_s - ra.t_start_s) + REPAIR_MARGIN_S,
                    delay_b_s=(ra.t_end_s - rb.t_start_s) + REPAIR_MARGIN_S,
                    site_id=ra.site_id, res_kinds=(ra.kind, rb.kind)))
        return out

    def next_free(self, site_id: str, kind: str, t_wanted_s: float, duration_s: float,
                  exclude_uav: str | None = None) -> float:
        """最早之 t ≥ t_wanted 使 [t, t + duration] 於該站無重疊（種類不影響互斥；只作介面對稱）。"""
        t = float(t_wanted_s)
        dur = max(0.0, float(duration_s))
        res = sorted((r for r in self.reservations
                      if r.site_id == site_id and (exclude_uav is None or r.uav_id != exclude_uav)),
                     key=lambda r: (r.t_start_s, r.t_end_s))
        moved = True
        while moved:
            moved = False
            for r in res:
                if r.overlaps(t, t + dur):
                    t = r.t_end_s
                    moved = True
        return t

    def prev_free(self, site_id: str, kind: str, t_wanted_s: float, duration_s: float,
                  exclude_uav: str | None = None) -> float:
        """最晚之 t ≤ t_wanted 使 [t, t + duration] 於該站無重疊（:meth:`next_free` 之反向，供起飛「提前」換序用）。"""
        t = float(t_wanted_s)
        dur = max(0.0, float(duration_s))
        res = sorted((r for r in self.reservations
                      if r.site_id == site_id and (exclude_uav is None or r.uav_id != exclude_uav)),
                     key=lambda r: (r.t_start_s, r.t_end_s), reverse=True)
        moved = True
        while moved:
            moved = False
            for r in res:
                if r.overlaps(t, t + dur):
                    t = r.t_start_s - dur
                    moved = True
        return t

    # -- 便利 -----------------------------------------------------------------
    def reserve_plan(self, plan: UAVPlan, takeoff_site_id: str) -> tuple[bool, bool]:
        """依 :func:`plan_windows` 登記起飛與降落預約；回傳 (takeoff_free, landing_free)。"""
        (ts0, ts1), (ls0, ls1) = plan_windows(plan, self.cfg)
        a = self.reserve(takeoff_site_id, "takeoff", ts0, ts1, plan.uav_id)
        b = self.reserve(plan.landing_site, "landing", ls0, ls1, plan.uav_id)
        return a, b

    def reserve_ghost(self, ghost: GhostLike) -> bool:
        """幽靈航跡之降落走廊預約 [ETA − 60, ETA + 180]（規格 §12.5）。"""
        lead, lag = GHOST_LANDING_WINDOW_S
        return self.reserve(ghost.landing_site, "landing", float(ghost.eta_s) - lead, float(ghost.eta_s) + lag,
                            ghost.uav_id)


def plan_windows(plan: UAVPlan, cfg: PlannerConfig) -> tuple[tuple[float, float], tuple[float, float]]:
    """(起飛區間, 降落區間)：[t0, t0 + Δt_dep]、[t_arr_ptc − 30, t_land + t_sep,LDG]（規格 §13.5）。"""
    to = (float(plan.t_takeoff_s), float(plan.t_takeoff_s) + float(cfg.dt_dep_s))
    ld = (float(plan.t_arr_ptc_s) - LANDING_LEAD_S, float(plan.t_land_s) + float(cfg.t_sep_ldg_s))
    return to, ld


# ---------------------------------------------------------------------------
# 空間互斥（規格 §13.2）
# ---------------------------------------------------------------------------

def _norm_pair(a: str, b: str) -> tuple[str, str]:
    return (a, b) if a <= b else (b, a)


def _team_band(team: TeamLike, cells: dict[str, Cell], r_eff: float | None) -> shapely.Geometry | None:
    """團隊所有胞之迴轉帶聯集（胞多邊形 ⊕ [−d_turn, d_turn] t̂ × [−lateral, lateral] n̂，D38）。"""
    cls = team.cls
    geoms: list[shapely.Geometry] = []
    for cid in team.cell_ids:
        cell = cells[cid]
        alpha = float(cell.alpha_rad.get(cls, cell.min_width_dir_rad))
        d_turn = float(cell.d_turn_m.get(cls, 0.0))
        lateral = 0.0
        if r_eff is not None:
            strips = cell.strips.get(cls) or []
            s_min = min((float(s.spacing_m) for s in strips), default=0.0)
            lateral = max(0.0, float(r_eff) - 0.5 * s_min)
        geoms.append(turn_band(cell.polygon, alpha, d_turn, lateral))
    if not geoms:
        return None
    return shapely.union_all(geoms)


def spatially_exclusive(teams: Sequence[TeamLike], cells: dict[str, Cell], cfg: PlannerConfig, *,
                        r_eff_m: dict[str, float] | float | None = None,
                        margin_m: float | None = None) -> set[tuple[str, str]]:
    """同層（同構型）團隊對：迴轉帶聯集互相距離 ≥ ``margin_m``（預設 ``cfg.d_sep_m``）→ 幾何上不可能 4D 衝突。

    回傳所有此類團隊對之 **uav_id 對**（字串排序之 tuple），可直接作 :func:`check_separation` 之 ``skip_pairs``。
    ``r_eff_m``（cls → R_eff 或單值）給定時側向外延取 max(0, R_eff − s/2)（D38 之 Ω 側向溢出）；None → 0。
    不同構型之團隊由高度分層負責（規格 §13.1），不在此判定。

    .. warning::
       本判定只涵蓋**作業段**（胞內條帶與其迴轉帶）。同一對機仍會在 ENTRY／TRANSFER／RTB 等轉場段相遇，
       故 :func:`check_separation` 只以本函式之結果豁免 OPERATE↔OPERATE 樣本對（見該函式「例外 2」）。
    """
    margin = float(cfg.d_sep_m) if margin_m is None else float(margin_m)
    bands: list[shapely.Geometry | None] = []
    for team in teams:
        r_eff: float | None
        if r_eff_m is None:
            r_eff = None
        elif isinstance(r_eff_m, dict):
            r_eff = r_eff_m.get(team.cls)
        else:
            r_eff = float(r_eff_m)
        bands.append(_team_band(team, cells, r_eff))
    out: set[tuple[str, str]] = set()
    for i in range(len(teams)):
        for j in range(i + 1, len(teams)):
            ti, tj = teams[i], teams[j]
            if ti.cls != tj.cls:
                continue
            bi, bj = bands[i], bands[j]
            if bi is None or bj is None or bi.is_empty or bj.is_empty:
                continue
            if shapely.distance(bi, bj) < margin - 1e-6:
                continue
            for ua in ti.uav_ids:
                for ub in tj.uav_ids:
                    if ua != ub:
                        out.add(_norm_pair(ua, ub))
    return out


# ---------------------------------------------------------------------------
# 4D 取樣與分離檢查（規格 §13.3）
# ---------------------------------------------------------------------------

def _time_grid(t0: float, t1: float, dt: float) -> np.ndarray:
    if not dt > 0.0:
        raise ValueError("dt_s 須 > 0")
    span = t1 - t0
    if span <= 1e-12:
        return np.array([t0], dtype=np.float64)
    n = int(math.floor(span / dt + 1e-9))
    grid = t0 + dt * np.arange(n + 1, dtype=np.float64)
    if grid[-1] < t1 - 1e-9:
        grid = np.append(grid, t1)
    return grid


def _waypoint_arrays(wps: Sequence[Waypoint]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t = np.array([float(w.t_s) for w in wps], dtype=np.float64)
    if t.size > 1 and np.any(np.diff(t) < -1e-9):
        raise ValueError("UAVPlan.waypoints 之 t_s 須非遞減")
    x = np.array([float(w.x_m) for w in wps], dtype=np.float64)
    y = np.array([float(w.y_m) for w in wps], dtype=np.float64)
    z = np.array([float(w.z_m) for w in wps], dtype=np.float64)
    return t, x, y, z


def _interp_plan(wps: Sequence[Waypoint], t_grid: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """線性內插 → (x, y, z, seg)；seg = 每樣本所在航段之起點航點索引。"""
    t, x, y, z = _waypoint_arrays(wps)
    n = t.size
    if n == 1:
        m = t_grid.size
        return np.full(m, x[0]), np.full(m, y[0]), np.full(m, z[0]), np.zeros(m, dtype=int)
    seg = np.clip(np.searchsorted(t, t_grid, side="right") - 1, 0, n - 2)
    dt_seg = t[seg + 1] - t[seg]
    with np.errstate(divide="ignore", invalid="ignore"):
        frac = np.where(dt_seg > 1e-12, (t_grid - t[seg]) / dt_seg, 1.0)
    frac = np.clip(frac, 0.0, 1.0)
    xs = x[seg] + frac * (x[seg + 1] - x[seg])
    ys = y[seg] + frac * (y[seg + 1] - y[seg])
    zs = z[seg] + frac * (z[seg + 1] - z[seg])
    return xs, ys, zs, seg


def sigma_t_max_of(cfg: PlannerConfig) -> float:
    """σ_t 上限：``cfg.sigma_t_max_s``（若有）否則 :data:`SIGMA_T_MAX_S`。"""
    v = getattr(cfg, "sigma_t_max_s", None)
    return SIGMA_T_MAX_S if v is None else float(v)


def _sigma(t_grid: np.ndarray, t_sync_s: float, k_t: float, sigma_max_s: float) -> np.ndarray:
    return np.minimum(float(k_t) * np.maximum(0.0, t_grid - float(t_sync_s)), float(sigma_max_s))


def trajectory_samples(plan: UAVPlan, dt_s: float, t_sync_s: float = 0.0, *, k_t: float = 0.05,
                       sigma_max_s: float = SIGMA_T_MAX_S) -> np.ndarray:
    """航點依時間戳線性內插 → ``(M, 5)`` ``[x, y, z, t, sigma_t]``，σ_t = k_t·max(0, t − t_sync)（規格 §13.3）。

    取樣時刻 t_0, t_0 + Δt, …，末樣本恆為最後航點時刻。弧命令航點（`Waypoint.arc`）以弦近似（build_3d 之
    弧段取樣 ≤ min(d_d, RΔψ)，弦誤差 ≤ e_max）。``k_t`` 預設 = `PlannerConfig.k_t` 預設值；
    :func:`check_separation` 以 ``cfg.k_t`` 呼叫。
    """
    wps = plan.waypoints
    if not wps:
        return np.zeros((0, 5), dtype=np.float64)
    t_grid = _time_grid(float(wps[0].t_s), float(wps[-1].t_s), float(dt_s))
    xs, ys, zs, _ = _interp_plan(wps, t_grid)
    sigma = _sigma(t_grid, t_sync_s, k_t, sigma_max_s)
    return np.column_stack([xs, ys, zs, t_grid, sigma])


@dataclass(slots=True)
class _Track:
    uav_id: str
    samples: np.ndarray            # (M,5) [x,y,z,t,sigma]
    heading: np.ndarray            # (M,) rad（nan = 無航跡方向）
    oper: np.ndarray               # (M,) bool：OPERATE 段
    is_ghost: bool
    corr_pad: np.ndarray | None = None   # (M,2)：該樣本所屬起降場坪之 (x, y)；非走廊樣本為 nan
    corr_site: np.ndarray | None = None  # (M,)：該樣本所屬起降站之 site_id；非走廊樣本／未知為 ""

    def pad(self) -> np.ndarray:
        if self.corr_pad is not None:
            return self.corr_pad
        return np.full((self.samples.shape[0], 2), np.nan)

    def site(self) -> np.ndarray:
        """走廊樣本之站點身分（(M,) 物件陣列）；非走廊樣本或計畫未帶站點時為 ""。"""
        if self.corr_site is not None:
            return self.corr_site
        return np.full(self.samples.shape[0], "", dtype=object)

    @property
    def n(self) -> int:
        return int(self.samples.shape[0])


def _corridor_masks(plan: UAVPlan, cfg: PlannerConfig, t_grid: np.ndarray, seg: np.ndarray
                    ) -> tuple[np.ndarray, np.ndarray]:
    """豁免條件 (1)+(2) 之樣本遮罩 ``(起飛段, 降落段)``。

    ⚠ 場坪座標與站點身分**必須**共用同一組遮罩，分開算會漂移（兩者要指同一批樣本）。
    """
    wps = plan.waypoints
    phase_wp = [w.phase for w in wps]
    dep_wp = np.array([p in (Phase.TAKEOFF, Phase.TRANS_OUT) for p in phase_wp], dtype=bool)
    arr_wp = np.array([p in (Phase.TRANS_IN, Phase.LAND) for p in phase_wp], dtype=bool)
    (ts0, ts1), (ls0, ls1) = plan_windows(plan, cfg)
    dep = dep_wp[seg] & (t_grid >= ts0 - _EPS) & (t_grid <= ts1 + _EPS)
    arr = arr_wp[seg] & (t_grid >= ls0 - _EPS) & (t_grid <= ls1 + _EPS)
    return dep, arr


def _corridor_pads(plan: UAVPlan, cfg: PlannerConfig, t_grid: np.ndarray, seg: np.ndarray) -> np.ndarray:
    """每一樣本之「起降場坪座標」(M,2)：起飛段取首航點、降落段取末航點；非走廊樣本或落在該機自己的
    走廊預約區間之外者為 nan（見 :data:`SITE_CORRIDOR_PHASES` 之豁免三條件）。

    ⚠ 這是**退回路徑**用的：正式判準是 :func:`_corridor_sites` 的站點身分。"""
    wps = plan.waypoints
    pads = np.full((t_grid.size, 2), np.nan)
    dep, arr = _corridor_masks(plan, cfg, t_grid, seg)
    if dep.any():
        pads[dep] = (float(wps[0].x_m), float(wps[0].y_m))
    if arr.any():
        pads[arr] = (float(wps[-1].x_m), float(wps[-1].y_m))
    return pads


def _corridor_sites(plan: UAVPlan, cfg: PlannerConfig, t_grid: np.ndarray, seg: np.ndarray) -> np.ndarray:
    """每一樣本之「起降站 site_id」(M,)：起飛段取 ``takeoff_site``、降落段取 ``landing_site``；
    非走廊樣本、或該欄為空（手搭 UAVPlan）者為 ``""``（⇒ :func:`_pair_metrics` 退回比座標）。"""
    sites = np.full(t_grid.size, "", dtype=object)
    dep, arr = _corridor_masks(plan, cfg, t_grid, seg)
    to_id, ld_id = str(getattr(plan, "takeoff_site", "") or ""), str(plan.landing_site or "")
    if to_id and dep.any():
        sites[dep] = to_id
    if ld_id and arr.any():
        sites[arr] = ld_id
    return sites


def _track_from_plan(plan: UAVPlan, dt_s: float, t_sync_s: float, cfg: PlannerConfig) -> _Track:
    wps = plan.waypoints
    if not wps:
        return _Track(plan.uav_id, np.zeros((0, 5)), np.zeros(0), np.zeros(0, dtype=bool), False)
    t_grid = _time_grid(float(wps[0].t_s), float(wps[-1].t_s), float(dt_s))
    xs, ys, zs, seg = _interp_plan(wps, t_grid)
    sigma = _sigma(t_grid, t_sync_s, cfg.k_t, sigma_t_max_of(cfg))
    samples = np.column_stack([xs, ys, zs, t_grid, sigma])
    trk = np.radians(np.array([float(w.track_deg) for w in wps], dtype=np.float64))
    oper_wp = np.array([w.phase is Phase.OPERATE for w in wps], dtype=bool)
    return _Track(plan.uav_id, samples, trk[seg], oper_wp[seg], False,
                  _corridor_pads(plan, cfg, t_grid, seg), _corridor_sites(plan, cfg, t_grid, seg))


def _track_from_ghost(ghost: GhostLike) -> _Track:
    s = np.asarray(ghost.samples, dtype=np.float64).reshape(-1, 4)
    sig = np.full(s.shape[0], max(0.0, float(ghost.sigma_t)))
    samples = np.column_stack([s[:, 0], s[:, 1], s[:, 2], s[:, 3], sig])
    return _Track(str(ghost.uav_id), samples, np.full(s.shape[0], np.nan), np.zeros(s.shape[0], dtype=bool), True)


def _norm_pairs(pairs: Iterable[tuple[str, str]] | None) -> set[tuple[str, str]]:
    if not pairs:
        return set()
    return {_norm_pair(str(a), str(b)) for a, b in pairs}


def _cross_pairs(pts_a: np.ndarray, pts_b: np.ndarray, r: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """兩點集之所有距離 ≤ r 之跨集合對 (i, j, d)（含 d = 0）。"""
    if pts_a.shape[0] == 0 or pts_b.shape[0] == 0:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int), np.zeros(0)
    m = cKDTree(pts_a).sparse_distance_matrix(cKDTree(pts_b), r, output_type="ndarray")
    return m["i"].astype(int), m["j"].astype(int), m["v"].astype(np.float64)


def _pair_metrics(ta: _Track, tb: _Track, ii: np.ndarray, jj: np.ndarray, cfg: PlannerConfig, *,
                  skip_oper: bool = False
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """樣本對之 (d_h, d_v, Δt = t_b − t_a, win = Δt_4d + σ_a + σ_b, spatial)；spatial = 水平 < 門檻 且 垂直 < Δh_sep。

    水平門檻：兩樣本皆 OPERATE 且航跡差 ≤ 20°（同向平行作業，D18）→ d_sep_lat，否則 d_sep。
    豁免：(a) **同站**起降走廊樣本對（:data:`SITE_CORRIDOR_PHASES`；以 ``site_id`` **身分**判定，
    計畫未帶 ``takeoff_site`` 時才退回比場坪座標 ≤ :data:`SAME_PAD_TOL_M`）；
    (b) ``skip_oper``（空間互斥機對）時之 OPERATE↔OPERATE 樣本對。
    """
    sa, sb = ta.samples, tb.samples
    d_h = np.hypot(sa[ii, 0] - sb[jj, 0], sa[ii, 1] - sb[jj, 1])
    d_v = np.abs(sa[ii, 2] - sb[jj, 2])
    dt_ab = sb[jj, 3] - sa[ii, 3]
    win = float(cfg.dt_4d_s) + sa[ii, 4] + sb[jj, 4]
    with np.errstate(invalid="ignore"):
        dtheta = np.abs((ta.heading[ii] - tb.heading[jj] + math.pi) % (2.0 * math.pi) - math.pi)
        codir = ta.oper[ii] & tb.oper[jj] & (dtheta <= CODIR_MAX_DIFF_RAD)
    thr = np.where(codir, float(cfg.d_sep_lat_m), float(cfg.d_sep_m))
    spatial = (d_h < thr) & (d_v < float(cfg.dh_sep_m))
    # 起降走廊 → 由 CorridorSchedule 管（§13.5）。⚠⚠⚠ 判準是站點**身分**不是座標距離：
    #   排程以 site_id 字串為鍵，兩個相距 30 m 的**不同**站不會互斥，比座標會把真違規豁免掉。
    sa_id, sb_id = ta.site()[ii], tb.site()[jj]
    known = (sa_id != "") & (sb_id != "")
    pa, pb = ta.pad()[ii], tb.pad()[jj]
    with np.errstate(invalid="ignore"):
        near_pad = np.hypot(pa[:, 0] - pb[:, 0], pa[:, 1] - pb[:, 1]) <= SAME_PAD_TOL_M   # nan → False
    same_pad = np.where(known, sa_id == sb_id, near_pad)    # 身分未知（手搭計畫）才退回座標
    spatial = spatial & ~same_pad
    if skip_oper:
        spatial = spatial & ~(ta.oper[ii] & tb.oper[jj])
    return d_h, d_v, dt_ab, win, spatial


def _forbidden_intervals(dt_ab: np.ndarray, win: np.ndarray, k_t: float) -> tuple[tuple[float, float], ...]:
    """「延遲 b 之量 s」之禁止開區間（合併、依左端排序）：s ∈ (L, U) ⇒ 該樣本對仍 |Δt + s| < win + k_t·s。

    每對之禁止區為開區間 (L, U)：U = (win − Δt)/(1 − k_t)（延後至 b 在後且分離）、
    L = (−Δt − win)/(1 + k_t)（b 仍在前且分離）。延遲 a 時以 Δt → −Δt 呼叫。
    """
    if dt_ab.size == 0:
        return ()
    k = float(k_t)
    U = (win - dt_ab) / max(1e-6, 1.0 - k)
    L = (-dt_ab - win) / (1.0 + k)
    order = np.argsort(L)
    L, U = L[order], U[order]
    merged: list[tuple[float, float]] = []
    cur_l, cur_u = float(L[0]), float(U[0])
    for lo, hi in zip(L[1:], U[1:]):
        if lo < cur_u:
            cur_u = max(cur_u, float(hi))
        else:
            merged.append((cur_l, cur_u))
            cur_l, cur_u = float(lo), float(hi)
    merged.append((cur_l, cur_u))
    return tuple(merged)


def _escape(s: float, intervals: Sequence[tuple[float, float]]) -> float:
    """最小 s' ≥ s 且不落在任何禁止開區間內（``intervals`` 須已合併且依左端排序）。"""
    s = float(s)
    for lo, hi in intervals:
        if lo < s < hi:
            s = hi
    return s


def _escape_down(s: float, intervals: Sequence[tuple[float, float]]) -> float:
    """最大 s' ≤ s 且不落在任何禁止開區間內（:func:`_escape` 之反向；起飛「提前」換序用）。"""
    s = float(s)
    for lo, hi in reversed(list(intervals)):
        if lo < s < hi:
            s = lo
    return s


def _min_delay(dt_ab: np.ndarray, win: np.ndarray, k_t: float) -> float:
    """延遲 b 之最小 s ≥ 0 使所有空間相近樣本對皆分離（= 自 0 向上跳出 :func:`_forbidden_intervals`）。"""
    return float(max(0.0, _escape(0.0, _forbidden_intervals(dt_ab, win, k_t))))


def _pair_delays(ta: _Track, tb: _Track, cfg: PlannerConfig,
                 pre: tuple[np.ndarray, np.ndarray] | None = None, *, skip_oper: bool = False
                 ) -> tuple[float, float, tuple[tuple[float, float], ...], tuple[tuple[float, float], ...]]:
    """(delay_a, delay_b, forbid_a, forbid_b)：以 2-D 空間查詢取得任意時差之相近樣本對後，各自單獨延遲即可
    消除整對衝突之最小延遲（含裕度）與對應之禁止延遲區間。

    ``pre`` = 已算好的空間相近對之 (Δt, win)（2-D 候選路徑可直接重用）。
    """
    if pre is None:
        r = max(float(cfg.d_sep_m), float(cfg.d_sep_lat_m))
        ii, jj, _ = _cross_pairs(ta.samples[:, :2], tb.samples[:, :2], r)
        _, _, dt_ab, win, spatial = _pair_metrics(ta, tb, ii, jj, cfg, skip_oper=skip_oper)
        dt_ab, win = dt_ab[spatial], win[spatial]
    else:
        dt_ab, win = pre
    win = win + REPAIR_MARGIN_S
    fa = _forbidden_intervals(-dt_ab, win, cfg.k_t)
    fb = _forbidden_intervals(dt_ab, win, cfg.k_t)
    return float(max(0.0, _escape(0.0, fa))), float(max(0.0, _escape(0.0, fb))), fa, fb


def _events(ii: np.ndarray, jj: np.ndarray, d_h: np.ndarray, d_v: np.ndarray, overlap: np.ndarray,
            dtab: np.ndarray, t_a: np.ndarray, gap_samples: int) -> list[tuple[int, int, float, float, float, float, float]]:
    """把違規樣本對依 a 之索引間隔切成事件；每事件回傳最近點 (ia, ib, d_h, d_v, t_a, Δt) 與事件最大 Δ_req。"""
    order = np.lexsort((jj, ii))
    ii, jj, d_h, d_v, overlap, dtab, t_a = (a[order] for a in (ii, jj, d_h, d_v, overlap, dtab, t_a))
    breaks = np.where(np.diff(ii) > gap_samples)[0] + 1
    out = []
    for g in np.split(np.arange(ii.size), breaks):
        kk = int(g[np.argmin(d_h[g])])
        out.append((int(ii[kk]), int(jj[kk]), float(d_h[kk]), float(d_v[kk]), float(t_a[kk]), float(dtab[kk]),
                    float(np.max(overlap[g]))))
    return out


def _candidate_pairs(ta: _Track, tb: _Track, cfg: PlannerConfig, v_ref_mps: float
                     ) -> tuple[np.ndarray, np.ndarray, bool]:
    """候選樣本對 (i, j, spatial_only)：時間視窗窄時以 (x, y, V_ref·t) 3-D 樹（半徑涵蓋 d_sep 與最大視窗），
    視窗寬（σ 大）時改以 2-D (x, y) 樹（候選 = 全部空間相近對，亦供一次算足之延遲計算）。"""
    r_xy = max(float(cfg.d_sep_m), float(cfg.d_sep_lat_m))
    win_max = float(cfg.dt_4d_s) + float(ta.samples[:, 4].max()) + float(tb.samples[:, 4].max())
    r3 = math.hypot(r_xy, float(v_ref_mps) * win_max)
    if r3 <= 3.0 * r_xy:
        pa = np.column_stack([ta.samples[:, 0], ta.samples[:, 1], float(v_ref_mps) * ta.samples[:, 3]])
        pb = np.column_stack([tb.samples[:, 0], tb.samples[:, 1], float(v_ref_mps) * tb.samples[:, 3]])
        ii, jj, _ = _cross_pairs(pa, pb, r3)
        return ii, jj, False
    ii, jj, _ = _cross_pairs(ta.samples[:, :2], tb.samples[:, :2], r_xy)
    return ii, jj, True


def check_separation(plans: dict[str, UAVPlan], cfg: PlannerConfig, ghosts: Sequence[GhostLike] | None = None,
                     v_ref_mps: float = 20.0, skip_pairs: set[tuple[str, str]] | None = None, *,
                     t_sync_s: float = 0.0, only_uavs: Iterable[str] | None = None) -> list[Conflict]:
    """4D 分離檢查（規格 §13.3）：逐機對以 `cKDTree` 於 (x, y, V_ref·t)（視窗寬時退回 2-D (x, y)）查詢候選對，
    再精確判定。

    違規 ⇔ 水平 < d_sep（同胞同向平行作業樣本：< d_sep_lat）且垂直 < Δh_sep 且 |t_a − t_b| < Δt + σ_a + σ_b。

    **例外 1（起降走廊）**：兩樣本皆屬起降走廊階段（:data:`SITE_CORRIDOR_PHASES`）、皆落在該機自己的走廊
    預約區間內、且屬**同一站點**（``site_id`` 相同；未帶站點之計畫才退回場坪座標 ≤
    :data:`SAME_PAD_TOL_M`）時不計 4D 衝突——該處之互斥由
    :class:`CorridorSchedule`（規格 §13.5：起飛間隔 60 s、降落預約 [t_arr−30, t_land+90]、降落優先）保證；
    若也計入 4D，則同站起降在 σ_t = k_t·t 視窗下恆為衝突，且修復所需延遲隨 t 發散（實測 8 機山丘情境
    makespan 7238 s → 18571 s 仍不收斂）。不同站之走廊樣本、或落在預約區間外之走廊樣本仍照常檢查。

    **例外 2（空間互斥，``skip_pairs``）**：:func:`spatially_exclusive` 之依據是「胞多邊形 ⊕ 迴轉帶」不相交，
    只涵蓋**作業段**；故 ``skip_pairs`` 之機對僅豁免 OPERATE↔OPERATE 樣本對，其 ENTRY/TRANSFER/RTB 等
    轉場段仍照常檢查（幽靈機對則整對豁免——幽靈無階段資訊）。

    每對航跡之連續違規樣本聚為一個事件 → 一筆 :class:`Conflict`（最近點；``dt_req_s`` = 事件內最大 Δ_req；
    ``delay_a_s/delay_b_s`` = 對該機對**所有**空間相近樣本對（任意時差）一次算足之單機最小延遲，使延遲後不再產生
    新的交會——規格 §13.6「一次算足」；``forbid_a/forbid_b`` = 對應之禁止延遲區間）。
    對稱：(a, b) 與 (b, a) 為同一筆（uav_a ≤ uav_b）；幽靈航跡（``ghosts``）為額外航跡，衝突 kind="ghost"
    （uav_a = 計畫機、uav_b = 幽靈）；幽靈之間不檢查。
    ``only_uavs`` 給定時只檢查「至少一方屬於該集合」之機對（修復之局部重算用）。
    """
    tracks: list[_Track] = [_track_from_plan(p, cfg.dt_4d_s, t_sync_s, cfg) for p in plans.values()]
    for g in (ghosts or ()):
        tracks.append(_track_from_ghost(g))
    tracks = [t for t in tracks if t.n > 0]
    nt = len(tracks)
    if nt < 2:
        return []
    skip = _norm_pairs(skip_pairs)
    only = None if only_uavs is None else {str(u) for u in only_uavs}
    gap_samples = max(1, int(math.ceil(EVENT_GAP_S / float(cfg.dt_4d_s))))
    out: list[Conflict] = []
    for a_i in range(nt):
        for b_i in range(a_i + 1, nt):
            tr_a, tr_b = tracks[a_i], tracks[b_i]
            ghost = tr_a.is_ghost or tr_b.is_ghost
            if tr_a.is_ghost and tr_b.is_ghost:
                continue
            if only is not None and tr_a.uav_id not in only and tr_b.uav_id not in only:
                continue
            skipped = _norm_pair(tr_a.uav_id, tr_b.uav_id) in skip
            if skipped and ghost:
                continue                                    # 幽靈：無階段資訊 → 整對豁免
            if (not ghost and tr_a.uav_id > tr_b.uav_id) or (ghost and tr_a.is_ghost):
                tr_a, tr_b = tr_b, tr_a
            li, lj, spatial_only = _candidate_pairs(tr_a, tr_b, cfg, v_ref_mps)
            if li.size == 0:
                continue
            d_h, d_v, dt_ab, win, spatial = _pair_metrics(tr_a, tr_b, li, lj, cfg, skip_oper=skipped)
            flag = spatial & (np.abs(dt_ab) < win)
            if not flag.any():
                continue
            pre = (dt_ab[spatial], win[spatial]) if spatial_only else None
            delay_a, delay_b, forbid_a, forbid_b = _pair_delays(tr_a, tr_b, cfg, pre, skip_oper=skipped)
            w_now = win[flag] + REPAIR_MARGIN_S                  # 只解「目前真的違規」之樣本對
            fn_a = _forbidden_intervals(-dt_ab[flag], w_now, cfg.k_t)
            fn_b = _forbidden_intervals(dt_ab[flag], w_now, cfg.k_t)
            t_a = tr_a.samples[li[flag], 3]
            for (ia, ib, dh, dv, ts, dab, dreq) in _events(li[flag], lj[flag], d_h[flag], d_v[flag],
                                                           (win - np.abs(dt_ab))[flag], dt_ab[flag], t_a, gap_samples):
                out.append(Conflict(uav_a=tr_a.uav_id, uav_b=tr_b.uav_id, t_s=ts, d_h_m=dh, d_v_m=dv, idx_a=ia,
                                    idx_b=ib, kind="ghost" if ghost else "4d", dt_req_s=dreq, dt_ab_s=dab,
                                    delay_a_s=delay_a, delay_b_s=delay_b, forbid_a=forbid_a, forbid_b=forbid_b,
                                    forbid_now_a=fn_a, forbid_now_b=fn_b))
    out.sort(key=lambda c: (c.t_s, c.uav_a, c.uav_b))
    return out


# ---------------------------------------------------------------------------
# 盤旋、平移、能量（修復用工具）
# ---------------------------------------------------------------------------

def _wind_speed_at(wind: Any, z_m: float) -> float:
    if wind is None:
        return 0.0
    if hasattr(wind, "speed_at"):
        return float(wind.speed_at(z_m))
    w = np.asarray(wind, dtype=np.float64).reshape(-1)
    return float(np.hypot(w[0], w[1])) if w.size >= 2 else 0.0


def orbit_lap_time_s(spec: PlatformSpec, wind: Any, z_m: float, r_orb_m: float | None = None) -> tuple[float, float]:
    """(T_lap, R_orb)：T_lap = 2πR/V_a · circle_time_factor(V_a, W(z))，R = r_orb_m 或 R_eff(W)（規格 §11.6）。

    A、W=5 → (60.9 s, 200 m)；B → (41.9 s, 120 m)。
    """
    w = _wind_speed_at(wind, float(z_m))
    r = float(r_orb_m) if r_orb_m is not None else float(spec.r_eff(w))
    va = float(spec.cruise_airspeed_mps)
    return 2.0 * math.pi * r / va * circle_time_factor(va, w), r


def shift_plan(plan: UAVPlan, delay_s: float, from_index: int = 0, energy_wh: float = 0.0) -> UAVPlan:
    """回傳時間戳自 ``from_index`` 起平移 ``delay_s`` 之新 UAVPlan（原物件不變；前段航點物件共用）。

    ``from_index == 0`` 時 t_takeoff_s 亦平移；t_arr_ptc_s / t_land_s 一律平移；
    ``energy_wh`` > 0 時自 from_index 起每航點 energy_remaining_wh 減去該值。
    """
    d = float(delay_s)
    e = float(energy_wh)
    wps = list(plan.waypoints[:from_index])
    for w in plan.waypoints[from_index:]:
        wps.append(replace(w, t_s=w.t_s + d, energy_remaining_wh=w.energy_remaining_wh - e))
    return replace(plan, waypoints=wps,
                   t_takeoff_s=plan.t_takeoff_s + (d if from_index == 0 else 0.0),
                   t_arr_ptc_s=plan.t_arr_ptc_s + d, t_land_s=plan.t_land_s + d,
                   warnings=list(plan.warnings))


def orbit_insert_index(plan: UAVPlan, t_conflict_s: float, *, corridor: bool = False,
                       site_xy: np.ndarray | None = None) -> int | None:
    """延遲盤旋插入航點索引：衝突時刻之「最近前一轉移」（非 OPERATE 航段）起點中第一個可插入階段
    （規格 §13.6 (ii)）；走廊衝突改取保持點（降落前、距站點 ≥ 2 km 之最後可插入航點，規格 §13.5）。無 → None。
    """
    wps = plan.waypoints
    n = len(wps)
    if n == 0:
        return None
    t = np.array([float(w.t_s) for w in wps])
    j = int(np.clip(np.searchsorted(t, float(t_conflict_s), side="right") - 1, 0, n - 1))
    if corridor:
        cands = [i for i in range(0, j + 1) if wps[i].phase in INSERTABLE_PHASES]
        if not cands:
            return None
        if site_xy is not None:
            sx, sy = float(site_xy[0]), float(site_xy[1])
            far = [i for i in cands if math.hypot(wps[i].x_m - sx, wps[i].y_m - sy) >= HOLD_DISTANCE_M]
            if far:
                return far[-1]
        return cands[-1]
    i = j
    while i >= 0 and wps[i].phase is Phase.OPERATE:
        i -= 1
    if i < 0:
        return None
    r = i
    while r - 1 >= 0 and wps[r - 1].phase is not Phase.OPERATE:
        r -= 1
    for k in range(r, i + 1):
        if wps[k].phase in INSERTABLE_PHASES:
            return k
    return None


def insert_delay_orbit(plan: UAVPlan, wp_index: int, n_laps: int, t_lap_s: float, model: EnergyModel | None,
                       r_orb_m: float) -> UAVPlan:
    """於航點 ``wp_index`` 插入 n 圈延遲盤旋（等高、右轉、圓心在航跡右側 R 處），其後航點時間戳 +n·T_lap、
    能量 −e_orbit（e_orbit = P_c·n·T_lap·k_unc，等高巡航功率）；不改變任何 z（D18）。

    插入之 ORBIT 航點為原航點之複本（t 不變、phase=ORBIT、sensor_on=False、arc=(cx, cy, R, +1)）；原航點保留
    其外出航段命令並延後 n·T_lap。
    """
    n_laps = int(n_laps)
    if n_laps < 1:
        return plan
    wps = plan.waypoints
    if not (0 <= wp_index < len(wps)):
        raise IndexError("wp_index 超出範圍")
    base = wps[wp_index]
    shift = n_laps * float(t_lap_s)
    e_orbit = float(model.segment_wh("CRUISE", shift, 0.0)) if model is not None else 0.0
    th = math.radians(float(base.track_deg))
    cx = float(base.x_m) + r_orb_m * math.cos(th)
    cy = float(base.y_m) - r_orb_m * math.sin(th)
    hold = replace(base, phase=Phase.ORBIT, mode=Mode.CRUISE, sensor_on=False, strip_id=None, dir=None,
                   arc=(cx, cy, float(r_orb_m), 1))
    new_wps = list(wps[:wp_index]) + [hold]
    for w in wps[wp_index:]:
        new_wps.append(replace(w, t_s=w.t_s + shift, energy_remaining_wh=w.energy_remaining_wh - e_orbit))
    dist = n_laps * 2.0 * math.pi * float(r_orb_m)
    ceq = float(model.cruise_equiv_km(e_orbit)) if (model is not None and e_orbit > 0) else 0.0
    return replace(plan, waypoints=new_wps, t_arr_ptc_s=plan.t_arr_ptc_s + shift, t_land_s=plan.t_land_s + shift,
                   energy_used_wh=plan.energy_used_wh + e_orbit, energy_margin_wh=plan.energy_margin_wh - e_orbit,
                   distance_m=plan.distance_m + dist, cruise_equiv_km=plan.cruise_equiv_km + ceq,
                   orbits_inserted=int(plan.orbits_inserted) + n_laps,
                   warnings=list(plan.warnings) + [f"delay_orbit(n={n_laps}, T_lap={t_lap_s:.1f}s, wp={wp_index})"])


def default_energy_ok(plan: UAVPlan, model: EnergyModel | None) -> bool:
    """預設能量重驗（規格 §5.5 之簡化；完整 ENERGY_SAFE_CHECK 由 `sim.check_energy` 經 ``energy_check`` 注入）：
    ``energy_margin_wh ≥ 0`` 且末航點 energy_remaining ≥ r·E_nom。"""
    if plan.energy_margin_wh < -1e-9:
        return False
    if model is not None and plan.waypoints:
        return float(plan.waypoints[-1].energy_remaining_wh) >= float(model.reserve_wh()) - 1e-9
    return True


# ---------------------------------------------------------------------------
# 修復（規格 §13.6）
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class _Action:
    kind: str                              # "takeoff" | "orbit" | "hook"
    plans: dict[str, UAVPlan]              # 受影響機之新計畫
    makespan_s: float
    cost: float                            # 次要排序鍵（總平移 / 圈數）


@dataclass(slots=True)
class _Repair:
    """repair 之共用狀態（就地更新 plans / sched）。"""
    plans: dict[str, UAVPlan]
    ctx: MissionContext
    models: dict[str, EnergyModel]
    sched: CorridorSchedule
    cfg: PlannerConfig
    to_site: dict[str, str]
    t_now_s: float
    t_sync_s: float
    v_ref_mps: float
    ghosts: dict[str, GhostLike]
    energy_check: Callable[[UAVPlan, EnergyModel | None], bool]
    hook: Callable[[str, UAVPlan, Conflict], UAVPlan | None] | None
    skip: set[tuple[str, str]] = field(default_factory=set)

    def model_of(self, uav_id: str) -> EnergyModel | None:
        return self.models.get(uav_id) or self.ctx.models.get(uav_id)

    def makespan(self, new: dict[str, UAVPlan]) -> float:
        return max((new.get(k, p).t_land_s for k, p in self.plans.items()), default=0.0)

    def airborne(self, uav_id: str) -> bool:
        return self.plans[uav_id].t_takeoff_s < self.t_now_s

    def involved_score(self, new: dict[str, UAVPlan] | None, uavs: Iterable[str]) -> tuple[int, float]:
        """套用 ``new`` 後、與 ``uavs`` 有關之 (衝突數, Σ Δ_req)（4D + 走廊）——修復候選之全域單調性位能函數。

        只重跑與被改動之機相關的機對（其餘機對不受影響），成本 ≈ n_U 次雙軌檢查（8 機約 10 ms）。
        以字典序嚴格下降為接受條件 ⇒ 修復不會把衝突推給別的機對，且必然終止。
        """
        uavs = {u for u in uavs if u in self.plans}
        if not uavs:
            return (0, 0.0)
        merged = dict(self.plans)
        if new:
            merged.update({u: p for u, p in new.items() if u in merged})
        cs = list(check_separation(merged, self.cfg, list(self.ghosts.values()) or None, self.v_ref_mps, self.skip,
                                   t_sync_s=self.t_sync_s, only_uavs=uavs))
        s2 = self.sched.copy()
        for u in uavs:
            s2.release(u)
            s2.reserve_plan(merged[u], self.to_site[u])
        cs += [x for x in s2.conflicts() if any(x.involves(u) for u in uavs)]
        return (len(cs), float(sum(max(0.0, c.dt_req_s) for c in cs)))

    def pair_clear(self, c: Conflict, new: dict[str, UAVPlan]) -> bool:
        """乾跑驗證：套用候選計畫後該機對（或機–幽靈、走廊）不再衝突。"""
        if c.kind == "corridor":
            s2 = self.sched.copy()
            ids = [u for u in (c.uav_a, c.uav_b) if u in self.plans]
            for u in ids:
                s2.release(u)
            for u in ids:
                s2.reserve_plan(new.get(u, self.plans[u]), self.to_site[u])
            return not any(x.involves(u) for x in s2.conflicts() for u in ids)
        if c.kind == "ghost":
            g = self.ghosts.get(c.uav_b)
            sub = {c.uav_a: new.get(c.uav_a, self.plans[c.uav_a])}
            return not check_separation(sub, self.cfg, [g] if g is not None else None, self.v_ref_mps, None,
                                        t_sync_s=self.t_sync_s)
        sub = {u: new.get(u, self.plans[u]) for u in (c.uav_a, c.uav_b) if u in self.plans}
        return not check_separation(sub, self.cfg, None, self.v_ref_mps, None, t_sync_s=self.t_sync_s)


def _merge_conflicts(*lists: Iterable[Conflict]) -> list[Conflict]:
    seen: set[tuple] = set()
    out: list[Conflict] = []
    for lst in lists:
        for c in lst:
            k = c.key()
            if k in seen:
                continue
            seen.add(k)
            out.append(c)
    out.sort(key=lambda c: (c.t_s, c.uav_a, c.uav_b))
    return out


def _fit_takeoff_shift(sched_wo: CorridorSchedule, plan: UAVPlan, s_req: float, to_site: str, cfg: PlannerConfig,
                       t_now_s: float, forbidden: Sequence[tuple[float, float]] = ()) -> float | None:
    """**最小**起飛延遲 s，同時滿足三個條件（規格 §13.6 (i)）：

    1. ``s ≥ s_req`` 且 t0 + s ≥ max(0, t_now)（已過去之時刻不可排）；
    2. 平移後之起飛區間 [t0+s, t0+Δt_dep+s] 與降落區間 [t_arr−30+s, t_land+t_sep,LDG+s] 於各自站點皆無重疊
       （不動之其他機已在 ``sched_wo`` 中）；
    3. ``s`` 不落在 ``forbidden``（該機對之禁止延遲開區間，:data:`Conflict.forbid_a`／``forbid_b``）之內。

    三個條件各自對應一個「最小的 ≥ s 之可行點」投影運算（`CorridorSchedule.next_free` 與 :func:`_escape` 皆
    傳回其可行集合中 ≥ 輸入之最小元素），故交替投影至不動點即為三集合交集之最小元素。

    舊版只做 (1)+(2)，把 26.7 s 的需求推到下一個空走廊時槽（山丘情境 B4：2083 s），而該時刻常常又落回
    禁止區間 (3)，乾跑失敗 → 整個候選被丟棄。加入 (3) 後同一次求解即得「既排得進走廊、又真的解掉衝突」的最小值。
    """
    (ts0, ts1), (ls0, ls1) = plan_windows(plan, cfg)
    s = max(float(s_req), max(0.0, t_now_s) - ts0)
    s = _escape(s, forbidden)
    for _ in range(200):
        s_prev = s
        s = max(s, sched_wo.next_free(to_site, "takeoff", ts0 + s, ts1 - ts0) - ts0)
        s = max(s, sched_wo.next_free(plan.landing_site, "landing", ls0 + s, ls1 - ls0) - ls0)
        s = _escape(s, forbidden)
        if s <= s_prev + 1e-9:
            return float(s)
    return None


def _fit_takeoff_advance(sched_wo: CorridorSchedule, plan: UAVPlan, s_max: float, to_site: str, cfg: PlannerConfig,
                         t_now_s: float, forbidden: Sequence[tuple[float, float]] = ()) -> float | None:
    """**提前**起飛（規格 §13.6 (i) 之「起飛換序」）：最大之 s ≤ min(0, s_max) 同時滿足

    1. t0 + s ≥ max(0, t_now)（不能排在過去）；2. 平移後起降預約皆無重疊（:meth:`CorridorSchedule.prev_free`）；
    3. s 不在 ``forbidden`` 內。

    repair 原本只會「延遲」，於是同型機共用轉場航路時（山丘 B1/B2，最近距離 0 m）唯一的出路是把某一架推遲
    2000 s 以上、makespan 隨之惡化；往負方向解同一組禁止區間往往只需數百秒且**降低** makespan。
    """
    (ts0, ts1), (ls0, ls1) = plan_windows(plan, cfg)
    lo_bound = max(0.0, float(t_now_s)) - ts0
    s = _escape_down(min(0.0, float(s_max)), forbidden)
    for _ in range(200):
        if s < lo_bound - 1e-9:
            return None
        s_prev = s
        s = min(s, sched_wo.prev_free(to_site, "takeoff", ts0 + s, ts1 - ts0) - ts0)
        s = min(s, sched_wo.prev_free(plan.landing_site, "landing", ls0 + s, ls1 - ls0) - ls0)
        s = _escape_down(s, forbidden)
        if s >= s_prev - 1e-9:
            return float(s) if (s <= -1e-9 and s >= lo_bound - 1e-9) else None
    return None


# -- 走廊連鎖重排（規格 §13.5「起降走廊互斥」＋ §13.6 (i)「起飛換序」）-------------------------------

def _windows_of(plans: dict[str, UAVPlan], to_site: dict[str, str], cfg: PlannerConfig
                ) -> dict[str, tuple[tuple[str, float, float], tuple[str, float, float]]]:
    """``uav -> ((起飛站, t0, t1), (降落站, l0, l1))``（未平移）。"""
    out = {}
    for u, p in plans.items():
        (ts0, ts1), (ls0, ls1) = plan_windows(p, cfg)
        out[u] = ((to_site.get(u, ""), ts0, ts1), (p.landing_site, ls0, ls1))
    return out


def _cascade_shift(R: "_Repair", seed: str | None, s_seed: float, forbidden: Sequence[tuple[float, float]] = (),
                   extra_fixed: Iterable[str] = ()) -> dict[str, float] | None:
    """把 ``seed`` 之整個計畫延後 ``s_seed`` 後，用**連鎖平移**（而非把自己推到下一個空時槽）解走廊重疊。

    回傳 ``{uav_id: 平移量}``（只含 > 0 者，含 seed），或 None（無法解）。

    規則：找出任一組同站重疊之預約 → 推「較晚開始」的一方；若該方不可動（已起飛／幽靈／即 seed 本身且已定量），
    改推另一方；「降落優先於起飛」（規格 §13.5）：起飛 vs 降落重疊時一律推起飛方。全部不可動 → None。
    每機最多被推 :data:`CASCADE_MAX_UAVS` 次，總迭代亦有上限，故必然終止。
    """
    base = _windows_of(R.plans, R.to_site, R.cfg)
    fixed = {u for u in R.plans if R.airborne(u)} | {str(u) for u in extra_fixed}   # 已起飛：起飛時刻不可改
    if seed is not None and (seed in fixed or seed not in R.plans):
        return None
    shift: dict[str, float] = {u: 0.0 for u in R.plans}
    if seed is not None:
        shift[seed] = float(s_seed)
    # 不屬於 plans 之預約（幽靈等）視為固定障礙
    ext = [(r.site_id, r.t_start_s, r.t_end_s) for r in R.sched.reservations if r.uav_id not in R.plans]
    pushes = 0
    for _ in range(CASCADE_MAX_UAVS * CASCADE_MAX_UAVS):
        items: list[tuple[str, str, float, float, str]] = []       # (site, uav, t0, t1, kind)
        for u, ((tsite, ts0, ts1), (lsite, ls0, ls1)) in base.items():
            d = shift[u]
            items.append((tsite, u, ts0 + d, ts1 + d, "takeoff"))
            items.append((lsite, u, ls0 + d, ls1 + d, "landing"))
        clash = None
        for i in range(len(items)):
            si, ui, a0, a1, ki = items[i]
            for sj, uj, b0, b1, kj in items[i + 1:]:
                if si != sj or ui == uj:
                    continue
                if a0 < b1 - _EPS and b0 < a1 - _EPS:
                    clash = ((ui, a0, a1, ki), (uj, b0, b1, kj))
                    break
            if clash is None:                                      # 也要避開固定障礙
                for sj, b0, b1 in ext:
                    if si == sj and a0 < b1 - _EPS and b0 < a1 - _EPS:
                        clash = ((ui, a0, a1, ki), (None, b0, b1, "fixed"))
                        break
            if clash is not None:
                break
        if clash is None:
            return {u: d for u, d in shift.items() if d > 1e-9}
        (ua, a0, a1, ka), (ub, b0, b1, kb) = clash
        push_a, push_b = (ua, b1 - a0), (ub, a1 - b0)
        if ub is None:                                             # 固定障礙：只能推 ua
            order = [push_a]
        elif ka == "takeoff" and kb == "landing":                  # 降落優先於起飛（§13.5）
            order = [push_a, push_b]
        elif kb == "takeoff" and ka == "landing":
            order = [push_b, push_a]
        elif a0 >= b0:                                             # 同種類：推較晚開始者
            order = [push_a, push_b]
        else:
            order = [push_b, push_a]
        moved = False
        for u, extra in order:
            if u is None or u in fixed or extra <= 0.0:
                continue
            shift[u] = shift[u] + float(extra) + REPAIR_MARGIN_S
            if seed is not None and u == seed:
                shift[u] = _escape(shift[u], forbidden)
            moved = True
            break
        pushes += 1
        if not moved or pushes > CASCADE_MAX_UAVS * CASCADE_MAX_UAVS - 2:
            return None
    return None


def _takeoff_movable(c: Conflict, R: _Repair) -> list[str]:
    out: list[str] = []
    for idx, u in enumerate((c.uav_a, c.uav_b)):
        if u in R.ghosts or u not in R.plans or R.airborne(u):
            continue
        if c.kind == "corridor" and c.res_kinds is not None:
            if c.res_kinds[idx] == "landing" and c.res_kinds[1 - idx] == "takeoff":
                continue                                        # 降落優先於起飛
        out.append(u)
    return out


def _forbid_of(c: Conflict, u: str, *, now: bool = False) -> tuple[tuple[float, float], ...]:
    """機 ``u`` 在衝突 ``c`` 中之禁止延遲區間（``now`` → 只由目前違規之樣本對構成；走廊衝突無此資訊 → 空）。"""
    if u == c.uav_a:
        return tuple(c.forbid_now_a if now else c.forbid_a)
    if u == c.uav_b:
        return tuple(c.forbid_now_b if now else c.forbid_b)
    return ()


def _forbid_variants(c: Conflict, u: str, s_req: float) -> list[tuple[float, tuple[tuple[float, float], ...]]]:
    """機 ``u`` 之 (最小延遲, 禁止區間) 候選：先「只解現有交會」再「整趟免疫」（規格 §13.6「一次算足」之兩種強度）。"""
    out: list[tuple[float, tuple[tuple[float, float], ...]]] = []
    for now in (True, False):
        fb = _forbid_of(c, u, now=now)
        s = float(max(0.0, _escape(0.0, fb))) if fb else float(s_req)
        if not any(abs(s - s0) <= 1e-9 for s0, _ in out):
            out.append((s, fb))
    if not out:
        out.append((float(s_req), ()))
    return out


def _takeoff_actions(c: Conflict, R: _Repair) -> list[_Action]:
    """(i) 起飛延遲／換序候選（零能量；重排走廊）。

    每架可動之機產生兩種候選：
    * **推自己**：:func:`_fit_takeoff_shift`（最小 s 同時滿足走廊與禁止區間）；
    * **連鎖重排**：:func:`_cascade_shift`（只延自己所需的 s，被擠到的其他地面機依序讓開）——山丘 8 機共用
      一條走廊時，前者會把 26.7 s 的需求推成 2083 s（要跳過整串既有降落預約），後者只需 26.7 s。
    兩者一併交由 :func:`_repair_one` 依 (makespan, 成本) 擇優。
    """
    movable = _takeoff_movable(c, R)
    actions: list[_Action] = []
    for u in movable:
        s_req0 = c.delay_a_s if u == c.uav_a else c.delay_b_s
        s2 = R.sched.copy()
        s2.release(u)
        for s_req, forbid in _forbid_variants(c, u, s_req0):
            s_fit = _fit_takeoff_shift(s2, R.plans[u], s_req, R.to_site[u], R.cfg, R.t_now_s, forbid)
            if s_fit is not None and s_fit > 0.0:
                new = {u: shift_plan(R.plans[u], s_fit)}
                if R.pair_clear(c, new):
                    actions.append(_Action("takeoff", new, R.makespan(new), abs(s_fit)))
            s_min = _escape(max(float(s_req), max(0.0, R.t_now_s) - R.plans[u].t_takeoff_s), forbid)
            if s_min > 0.0 and (s_fit is None or s_min < s_fit - 1e-9):
                casc = _cascade_shift(R, u, s_min, forbid)
                if casc:
                    new = {v: shift_plan(R.plans[v], d) for v, d in casc.items()}
                    if R.pair_clear(c, new):
                        actions.append(_Action("takeoff", new, R.makespan(new), sum(casc.values())))
            s_adv = _fit_takeoff_advance(s2, R.plans[u], 0.0, R.to_site[u], R.cfg, R.t_now_s, forbid)
            if s_adv is not None and s_adv < 0.0:                # 提前起飛（換序）
                new = {u: shift_plan(R.plans[u], s_adv)}
                if R.pair_clear(c, new):
                    actions.append(_Action("takeoff", new, R.makespan(new), abs(s_adv)))
    if len(movable) == 2:
        a, b = movable
        s_a = R.plans[b].t_takeoff_s - R.plans[a].t_takeoff_s
        if abs(s_a) > 1e-9:
            req = {a: s_a, b: -s_a}
            s2 = R.sched.copy()
            s2.release(a)
            s2.release(b)
            new: dict[str, UAVPlan] = {}
            for u in sorted(req, key=lambda k: req[k]):          # 先安置提前者
                s_fit = _fit_takeoff_shift(s2, R.plans[u], req[u], R.to_site[u], R.cfg, R.t_now_s, _forbid_of(c, u))
                if s_fit is None:
                    new = {}
                    break
                new[u] = shift_plan(R.plans[u], s_fit)
                s2.reserve_plan(new[u], R.to_site[u])
            if len(new) == 2 and R.pair_clear(c, new):
                actions.append(_Action("takeoff", new, R.makespan(new),
                                       sum(abs(p.t_takeoff_s - R.plans[u].t_takeoff_s) for u, p in new.items())))
    return actions


def _orbit_n_for(s_req: float, t_lap: float, forbidden: Sequence[tuple[float, float]], n_max: int = 4096) -> int:
    """最小圈數 n ≥ 1 使 n·T_lap ≥ s_req 且 n·T_lap 不落在禁止延遲區間內。"""
    n = max(1, int(math.ceil(float(s_req) / t_lap - 1e-9)))
    for _ in range(n_max):
        s = n * t_lap
        s2 = _escape(s, forbidden)
        if s2 <= s + 1e-9:
            return n
        n = int(math.ceil(s2 / t_lap - 1e-9))
    return n


def _orbit_actions(c: Conflict, R: _Repair) -> list[_Action]:
    """(ii) 於最近前一轉移插入延遲盤旋（重驗能量）。

    **兩種候選**（規格 §13.6 (ii) 之延遲 vs §13.5 之走廊時槽，分開處理）：

    * ``n_4d`` = 解 4D／走廊衝突本身所需之最小圈數（⌈Δ_req/T_lap⌉，並跳出禁止延遲區間）。若平移後之降落
      預約與他機重疊，改由**走廊連鎖重排**（:func:`_cascade_shift`，讓後面的地面機讓開）吸收，而不是把自己
      的圈數往上加。
    * ``n_slot`` = 舊行為：把圈數一路加到降落走廊有空檔為止（作為連鎖失敗時的後備）。

    舊版只有後者：山丘 8 機共用一條降落走廊時，B4 的 1 圈被拉到 39 圈、B5 到 58 圈（≈1600–2400 s），
    隨後能量重驗必定失敗 → 該衝突完全沒有候選。
    """
    actions: list[_Action] = []
    for idx, u in enumerate((c.uav_a, c.uav_b)):
        if u in R.ghosts or u not in R.plans:
            continue
        if c.kind == "corridor" and (c.res_kinds is None or c.res_kinds[idx] != "landing"):
            continue                                            # 起飛機無盤旋可言
        plan = R.plans[u]
        if not plan.waypoints:
            continue
        s_req0 = c.delay_a_s if idx == 0 else c.delay_b_s
        t_c = c.t_s if idx == 0 else c.t_s + c.dt_ab_s
        site_xy = R.ctx.sites_enu[plan.landing_site][0] if plan.landing_site in R.ctx.sites_enu else None
        k = orbit_insert_index(plan, t_c, corridor=(c.kind == "corridor"), site_xy=site_xy)
        if k is None:
            continue
        spec = R.ctx.specs.get(plan.cls) or R.ctx.spec_of(u)
        model = R.model_of(u)
        t_lap, r_orb = orbit_lap_time_s(spec, R.ctx.wind, float(plan.waypoints[k].z_m))
        if not math.isfinite(t_lap) or t_lap <= 0.0:
            continue
        (_, _), (ls0, ls1) = plan_windows(plan, R.cfg)
        s2 = R.sched.copy()
        s2.release(u)
        cands: list[tuple[int, bool]] = []                       # (圈數, 是否需要走廊連鎖)
        for s_req, forbid in _forbid_variants(c, u, s_req0):
            n_4d = _orbit_n_for(s_req, t_lap, forbid)
            cands.append((n_4d, True))
            n = n_4d
            for _ in range(8):                                  # 後備：圈數增加至降落預約可用
                lsf = s2.next_free(plan.landing_site, "landing", ls0 + n * t_lap, ls1 - ls0)
                if lsf > ls0 + n * t_lap + 1e-9:
                    n = _orbit_n_for((lsf - ls0), t_lap, forbid)
                    continue
                break
            if n != n_4d:
                cands.append((n, False))
        for n_i, need_cascade in dict.fromkeys(cands):
            new_plan = insert_delay_orbit(plan, k, n_i, t_lap, model, r_orb)
            if not R.energy_check(new_plan, model):
                continue
            new = {u: new_plan}
            if need_cascade:
                casc = _cascade_orbit(R, u, new_plan)
                if casc is None:
                    continue
                new.update(casc)
            if R.pair_clear(c, new):
                actions.append(_Action("orbit", new, R.makespan(new), float(n_i)))
    return actions


def _cascade_orbit(R: "_Repair", u: str, new_plan: UAVPlan) -> dict[str, UAVPlan] | None:
    """插入延遲盤旋後，若 ``u`` 之降落預約與他機重疊，讓**其他尚未起飛之機**連鎖讓開（規格 §13.5）。

    回傳 ``{other: 新計畫}``（不含 ``u``；無需讓開時為空 dict），或 None（無法解）。
    """
    tmp = dict(R.plans)
    tmp[u] = new_plan
    R2 = replace(R, plans=tmp)
    casc = _cascade_shift(R2, None, 0.0, extra_fixed=(u,))      # u 已延遲：固定，只讓其他地面機讓開
    if casc is None:
        return None
    return {v: shift_plan(tmp[v], d) for v, d in casc.items() if v != u}


def _apply(action: _Action, R: _Repair) -> set[str]:
    for u, p in action.plans.items():
        R.plans[u] = p
        R.sched.release(u)
        R.sched.reserve_plan(p, R.to_site[u])
    return set(action.plans)


def _dedupe_actions(acts: list[_Action]) -> list[_Action]:
    """去除等價候選（同一組機、同樣的起飛時刻與圈數）——候選產生器會從兩組禁止區間得到相同結果。"""
    seen: set[tuple] = set()
    out: list[_Action] = []
    for a in acts:
        k = tuple(sorted((u, round(p.t_takeoff_s, 3), round(p.t_land_s, 3), int(p.orbits_inserted))
                         for u, p in a.plans.items()))
        if k in seen:
            continue
        seen.add(k)
        out.append(a)
    return out


def _action_key(act: _Action, after: tuple[int, float], ms_now: float) -> tuple:
    """修復候選之排序鍵：**先 makespan、再相關衝突數、再 ΣΔ_req、再成本**。

    makespan 優先是刻意的：所有候選都已通過「相關衝突數不增（且 ΣΔ_req 下降）」的單調性門檻
    （見 :func:`_repair_one`），故任一被採用者都在收斂方向上；在此前提下應挑對總工期傷害最小者。
    反過來（先看衝突數）實測會為了一次多消掉 1–2 筆衝突而把最後一架推遲 2000 s 以上。
    ``max(act.makespan_s, ms_now)`` 使「不動 makespan 的候選」彼此等價，再由衝突數決勝。
    """
    return (max(act.makespan_s, ms_now), after[0], after[1], act.cost)


def _repair_one(c: Conflict, R: _Repair, *, cheap_only: bool = False) -> set[str]:
    """對單筆衝突依序嘗試 (i) 起飛延遲／換序 (ii) 延遲盤旋 (iii) hook；回傳被修改之機。

    所有候選都必須通過**全域單調性**檢查（:meth:`_Repair.involved_score`：相關衝突數下降，或衝突數持平
    但 ΣΔ_req 下降且 makespan 不增）——僅以機對乾跑挑選會讓某些延遲把衝突推到別的機對上（實測山丘 8 機：
    11 → 20 筆）。通過者再以 :func:`_action_key`（makespan 優先）擇優。

    ``cheap_only`` = True 時只接受**不增加 makespan** 的候選（供呼叫端做「先免費、後付費」的兩階段掃描；
    :func:`repair` 目前不用，實測在山丘情境反而較差）。
    """
    ms_now = R.makespan({})
    base_cache: dict[frozenset, tuple[int, float]] = {}
    for finder in (_takeoff_actions, _orbit_actions):
        acts = _dedupe_actions(finder(c, R))
        if not acts:
            continue
        best: tuple[tuple, _Action] | None = None
        for act in acts:
            if cheap_only and act.makespan_s > ms_now + _EPS:
                continue
            uavs = set(act.plans) | {c.uav_a, c.uav_b}
            key = frozenset(uavs)
            if key not in base_cache:
                base_cache[key] = R.involved_score(None, uavs)
            base, after = base_cache[key], R.involved_score(act.plans, uavs)
            better = after[0] < base[0] or (after[0] == base[0] and after[1] < base[1] - _EPS
                                            and act.makespan_s <= ms_now + _EPS)
            if not better:
                continue
            key = _action_key(act, after, ms_now)
            if best is None or key < best[0]:
                best = (key, act)
        if best is not None:
            return _apply(best[1], R)
    return set() if cheap_only else _resequence(c, R)


def _hook_arity(hook: Callable) -> int:
    """回呼之位置參數個數（3 → 舊式 ``(uav_id, plan, conflict)``；其餘 → 新式 ``(ResequenceRequest)``）。"""
    try:
        params = [p for p in inspect.signature(hook).parameters.values()
                  if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
    except (TypeError, ValueError):
        return 1
    return len(params)


def _resequence(c: Conflict, R: _Repair) -> set[str]:
    """(iii) IDP 重排（規格 §13.6 步 (iii)）：以 :class:`ResequenceRequest` 呼叫 ``resequence_hook``。

    回傳之計畫仍須通過該機對之乾跑（:meth:`_Repair.pair_clear`）與全域單調性
    （:meth:`_Repair.involved_score`）；不通過即忽略（該衝突留為殘餘）。
    """
    if R.hook is None:
        return set()
    ids = tuple(u for u in (c.uav_a, c.uav_b) if u not in R.ghosts and u in R.plans)
    if not ids:
        return set()
    ids = tuple(sorted(ids, key=lambda u: R.plans[u].t_land_s))      # makespan 貢獻小者優先（§13.6 (ii)/(iii)）
    if _hook_arity(R.hook) >= 3:                                      # 舊式三引數回呼
        for u in ids:
            out = R.hook(u, R.plans[u], c)
            if out is not None:
                return _apply(_Action("hook", {u: out}, R.makespan({u: out}), 0.0), R)
        return set()
    req = ResequenceRequest(conflict=c, uav_ids=ids, plans=dict(R.plans), schedule=R.sched, ctx=R.ctx, cfg=R.cfg,
                            models=dict(R.models), t_now_s=R.t_now_s, t_sync_s=R.t_sync_s,
                            takeoff_sites=dict(R.to_site), energy_check=R.energy_check)
    out = R.hook(req)
    if out is None:
        return set()
    new = {ids[0]: out} if isinstance(out, UAVPlan) else {str(u): p for u, p in dict(out).items() if u in R.plans}
    if not new:
        return set()
    uavs = set(new) | {c.uav_a, c.uav_b}
    base, after = R.involved_score(None, uavs), R.involved_score(new, uavs)
    if after[0] > base[0] or (after[0] == base[0] and after[1] > base[1] + _EPS):
        return set()
    return _apply(_Action("hook", new, R.makespan(new), 0.0), R)


def _merge_intervals(ivs: Sequence[tuple[float, float]]) -> tuple[tuple[float, float], ...]:
    """合併重疊之開區間並依左端排序。"""
    out: list[tuple[float, float]] = []
    for lo, hi in sorted(ivs):
        if out and lo < out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], hi))
        else:
            out.append((lo, hi))
    return tuple(out)


def _tracks_of(R: _Repair, cache: dict | None) -> dict[str, _Track]:
    """所有實機航跡（可跨呼叫重用；以 UAVPlan 物件同一性判定是否過期）。"""
    out: dict[str, _Track] = {}
    for v, p in R.plans.items():
        hit = None if cache is None else cache.get(v)
        if hit is not None and hit[0] is p:
            out[v] = hit[1]
            continue
        trk = _track_from_plan(p, R.cfg.dt_4d_s, R.t_sync_s, R.cfg)
        out[v] = trk
        if cache is not None:
            cache[v] = (p, trk)
    return out


def _advance_4d_limit(R: _Repair, u: str, cache: dict | None = None) -> tuple[tuple[float, float], ...]:
    """機 ``u`` 之「平移 s」禁止開區間（對**所有**其他航跡與幽靈取聯集；:func:`_forbidden_intervals`）。

    解析解，不需逐點試探：任一不落在這些區間內的 s 都保證不產生 4D／幽靈衝突。
    """
    trks = _tracks_of(R, cache)
    ta = trks[u]
    if ta.n == 0:
        return ()
    r_xy = max(float(R.cfg.d_sep_m), float(R.cfg.d_sep_lat_m))
    lo_all: list[float] = []
    hi_all: list[float] = []
    others: list[_Track] = [t for v, t in trks.items() if v != u]
    others += [_track_from_ghost(g) for g in R.ghosts.values()]
    for tb in others:
        if tb.n == 0:
            continue
        skipped = _norm_pair(ta.uav_id, tb.uav_id) in R.skip
        if skipped and tb.is_ghost:
            continue
        ii, jj, _ = _cross_pairs(ta.samples[:, :2], tb.samples[:, :2], r_xy)
        if ii.size == 0:
            continue
        _, _, dt_ab, win, sp = _pair_metrics(ta, tb, ii, jj, R.cfg, skip_oper=skipped)
        if not sp.any():
            continue
        for lo, hi in _forbidden_intervals(-dt_ab[sp], win[sp] + REPAIR_MARGIN_S, R.cfg.k_t):
            lo_all.append(lo)
            hi_all.append(hi)
    if not lo_all:
        return ()
    return _merge_intervals(list(zip(lo_all, hi_all)))


def _advance_candidates(R: _Repair, u: str, limit: int = 24, cache: dict | None = None) -> list[float]:
    """機 ``u`` 之「提前起飛」候選平移量（負值，最負者在前）。

    下界為 t0 + s ≥ max(0, t_now)（不可排在過去）；候選點取同站既有預約之邊界＋均勻格點，
    再濾掉落在 4D 禁止區間（:func:`_advance_4d_limit`）內者、以及平移後起降預約會重疊者。
    """
    plan = R.plans[u]
    (ts0, ts1), (ls0, ls1) = plan_windows(plan, R.cfg)
    lo = max(max(0.0, R.t_now_s) - ts0, -ts0)                   # t0 + s ≥ max(0, t_now) 且 ≥ 0
    if lo >= -1e-9:
        return []
    forbid = _advance_4d_limit(R, u, cache)
    dur_t, dur_l = ts1 - ts0, ls1 - ls0
    vals = {lo}
    for r in R.sched.reservations:
        if r.uav_id == u:
            continue
        if r.site_id == R.to_site.get(u):
            vals.add(r.t_end_s - ts0)
            vals.add(r.t_start_s - dur_t - ts0)
        if r.site_id == plan.landing_site:
            vals.add(r.t_end_s - ls0)
            vals.add(r.t_start_s - dur_l - ls0)
    step = max(30.0, -lo / 16.0)
    vals.update(lo + step * k for k in range(1, 17))
    vals.update(iv[0] for iv in forbid)                         # 禁止區之下緣本身是可行點
    out = sorted(v for v in vals if lo - 1e-9 <= v <= -1e-9 and _escape_down(v, forbid) >= v - 1e-9)
    ok = [v for v in out if R.sched.is_free(R.to_site.get(u, ""), ts0 + v, ts1 + v, exclude_uav=u)
          and R.sched.is_free(plan.landing_site, ls0 + v, ls1 + v, exclude_uav=u)]
    return ok[:limit]


def _compact_makespan(R: _Repair, rounds: int = 2) -> set[str]:
    """**工期壓縮**（修復後處理）：依落地時刻由晚到早，逐機嘗試把整個計畫**提前**到走廊與 4D 都仍乾淨的最早時刻。

    修復本身只會延遲（規格 §13.6 (i)(ii) 都是加延遲），一連串貪婪延遲會把 makespan 推高且無法回收；
    本步在不新增任何衝突（:meth:`_Repair.involved_score` 不變差）的前提下把多餘的延遲吐回來，
    等價於規格 §13.6 (i) 的「起飛換序」在收斂後再走一次。永不改變航點 z、不改變能量。
    """
    changed: set[str] = set()
    cache: dict = {}
    for _ in range(max(1, int(rounds))):
        moved = False
        for u in sorted(R.plans, key=lambda k: -R.plans[k].t_land_s):
            if R.airborne(u) or not R.plans[u].waypoints:
                continue
            base = R.involved_score(None, {u})
            for s in _advance_candidates(R, u, cache=cache):
                new = {u: shift_plan(R.plans[u], s)}
                after = R.involved_score(new, {u})
                if after[0] <= base[0] and after[1] <= base[1] + _EPS:
                    _apply(_Action("compact", new, R.makespan(new), abs(s)), R)
                    changed.add(u)
                    moved = True
                    break
        if not moved:
            break
    return changed


def scan_done_s(plan: UAVPlan) -> float:
    """該機**掃描完成**時刻＝相機最後一次開著的航點時刻；沒有成像段時退回落地時刻。

    ⚠ 這與 ``t_land_s`` 是兩件事。規格 Eq.(1) 的目標是 min-max **落地**時刻（含返航
    與降落），而「最快掃完區域」關心的是這一個。實測 HV4SITL9：落地 1799.8 s、
    掃描完成 1594.6 s，兩者差 205 s，且**排序不同**（最後降落的不一定最後掃完）。
    """
    ts = [w.t_s for w in plan.waypoints if getattr(w, "sensor_on", False)]
    return max(ts) if ts else float(plan.t_land_s)


def _objective_of(plans: dict[str, UAVPlan], objective: str) -> float:
    if objective == "scan":
        return max((scan_done_s(p) for p in plans.values()), default=0.0)
    return max((float(p.t_land_s) for p in plans.values()), default=0.0)


def _fresh_schedule(R: "_Repair", placed: dict[str, UAVPlan]) -> CorridorSchedule:
    """只含 ``placed``（與幽靈）之走廊時刻表 —— 供逐機插入時做「與已擺放者是否相容」。"""
    s2 = CorridorSchedule(R.cfg)
    for g in R.ghosts.values():
        s2.reserve_ghost(g)
    for k, p in placed.items():
        s2.reserve_plan(p, R.to_site[k])
    return s2


def _global_reschedule(R: "_Repair", *, step_s: float = 10.0, objective: str = "landing",
                       max_evals: int = 20000) -> set[str]:
    """**全隊起飛時刻全域重排**（修復＋工期壓縮之後的最後一手）。

    ⚠⚠⚠ 為什麼需要：``repair`` 是逐衝突貪婪（``_repair_one`` 一有候選通過就 return），
    而 :func:`_compact_makespan` 收尾時**一次只平移一機**（``involved_score(new, {u})``）
    ⇒ 「A 晚一點起飛好讓 B 早很多」這種協同平移**結構上做不到**。實測 HV4SITL9：
    起飛時刻被排成 0 / 289.1 / 623.9 / **1265.6** s，而驅動它的 6 筆衝突**全部**是
    同一個場坪的**降落**走廊排隊 —— 起飛被押後，是為了解 500 s 後才發生的降落衝突。
    在**同一組門檻、同一個可行性模型**下換個順序重排，同一份計畫可到 1461.3 s。

    作法：對可動機的每一種起飛**順序**，逐機插入其「最早可行」時刻（可行＝ 4D 分離
    ＋ 走廊時刻表兩者皆與已擺放者相容），取目標函數最小者。套用前再跑一次**全機對**
    的完整驗證；沒有任何候選全綠就**原樣退回**，所以這一步不可能讓結果變差。

    ⚠⚠ **平移不具不變性，不可以拿「相對時距不變」當可行性的推論**：4D 判準是
    ``|t_a − t_b| < Δt + σ_a + σ_b``，而 ``σ_t = min(k_t·(t − t_sync), SIGMA_T_MAX_S)``
    （見 :func:`check_separation` 與 :data:`SIGMA_T_MAX_S`）是**絕對時間**的函數。
    把全隊同步平移（相對時距一格沒動）實測 Δ=+300 s 就會多出 1 筆衝突。
    ⇒ 因此本函式**不建立任何鬆弛的逐對可行集**，每一個候選都逐機做真檢查；
    也因此提早起飛帶來的改善**有一部分來自 σ 變小**（模型餘裕），不是純排程壓縮。
    要單獨衡量後者，把 ``cfg.k_t`` 設成 0 再比一次。

    永不改變航點 x/y/z、不改變能量（:func:`shift_plan` 在 ``energy_wh=0`` 下只動時間戳）
    ⇒ 條帶、刈幅、側疊、``uncovered`` 一個都不會變。已起飛者（``R.airborne``）不可動。
    """
    movable = [u for u in sorted(R.plans) if not R.airborne(u) and R.plans[u].waypoints]
    if len(movable) < 2:
        return set()
    fixed = {u: p for u, p in R.plans.items() if u not in movable}
    incumbent = _objective_of(R.plans, objective)
    # ⚠ 剪枝的界必須與被比較的量**同單位**。`earliest` 用的是落地時刻
    #   （t_takeoff + dur），所以界要用落地 makespan，不能用 incumbent ——
    #   objective="scan" 時 incumbent 是掃描時刻（比落地早），拿去剪落地時刻會
    #   過度剪枝，實測讓 scan 目標的結果（1251.7）反而**輸給** landing 目標（1216.4）。
    land_cap = _objective_of(R.plans, "landing")
    base_plans = {u: R.plans[u] for u in movable}
    dur = {u: float(p.t_land_s) - float(p.t_takeoff_s) for u, p in base_plans.items()}
    t_floor = max(0.0, R.t_now_s if math.isfinite(R.t_now_s) else 0.0)
    horizon = max(land_cap - t_floor, step_s)
    step = max(1.0, float(step_s))
    evals = 0

    def fits(placed: dict[str, UAVPlan], u: str, cand: UAVPlan) -> bool:
        nonlocal evals
        evals += 1
        merged = dict(placed)
        merged[u] = cand
        if check_separation(merged, R.cfg, list(R.ghosts.values()) or None, R.v_ref_mps, R.skip,
                            t_sync_s=R.t_sync_s, only_uavs={u}):
            return False
        return not any(x.involves(u) for x in _fresh_schedule(R, merged).conflicts())

    def earliest(placed: dict[str, UAVPlan], u: str, cap: float) -> UAVPlan | None:
        """u 在不超過 ``cap``（落地時刻上界）之下的最早可行擺放；找不到回 None。"""
        p0 = base_plans[u]
        lo = t_floor - float(p0.t_takeoff_s)
        s = lo
        hi = lo + horizon
        while s <= hi + 1e-9:
            if float(p0.t_takeoff_s) + s + dur[u] > cap + 1e-9:
                return None                                  # 再往後只會更差 ⇒ 剪枝
            if evals >= max_evals:
                return None
            cand = shift_plan(p0, s)
            if fits(placed, u, cand):
                fine = step / 5.0                            # 往回細找，別停在粗格上
                t = s
                while t - fine >= lo - 1e-9:
                    c2 = shift_plan(p0, t - fine)
                    if not fits(placed, u, c2):
                        break
                    t -= fine
                    cand = c2
                return cand
            s += step
        return None

    # 順序候選：機數少時全枚舉；多時只走幾個啟發序（|G|^(n−1) 不可全枚舉）
    if len(movable) <= 6:
        orders: list[tuple[str, ...]] = sorted(
            itertools.permutations(movable), key=lambda o: [-dur[u] for u in o])
    else:
        orders = [tuple(sorted(movable, key=lambda u: -dur[u])),
                  tuple(sorted(movable, key=lambda u: dur[u])),
                  tuple(sorted(movable, key=lambda u: base_plans[u].t_takeoff_s)),
                  tuple(sorted(movable, key=lambda u: -base_plans[u].t_land_s))]

    best: dict[str, UAVPlan] | None = None
    for order in orders:
        if evals >= max_evals:
            break
        placed = dict(fixed)
        ok = True
        for u in order:
            # 落地 makespan 不得比現況差（無論目標是哪一個）；目標本身另外比。
            cand = earliest(placed, u, land_cap - 1e-9)
            if cand is None:
                ok = False
                break
            placed[u] = cand
        if not ok:
            continue
        score = _objective_of(placed, objective)
        # ⚠ 同樣誠實說明：這個「嚴格改善」條件也**咬不到突變**（改成 `if True`
        #   實測輸出逐位相同 1450.8 / 1216.4）。因為上面的 `earliest` 已經用
        #   `land_cap` 把每個完成的候選都框在「落地 makespan 優於現況」之內。
        #   保留它是為了「目標＝掃描完成」那條路徑：那時 score 與 land_cap 不同軸，
        #   沒有這個比較就會挑到落地較早但**掃描較晚**的解。
        if score < incumbent - 1e-9:
            incumbent = score
            best = {u: placed[u] for u in movable}

    if best is None:
        return set()

    # ── 套用前的**完整**驗證（全機對，不是只看被改動的機）────────────────
    #   ⚠⚠ 誠實說明：這一道在目前的演算法下**可以證明是多餘的**，所以它
    #   **沒有**突變臂能咬到（實測拿掉它，10 條測試全綠）。理由是歸納法：
    #   逐機插入時每擺一台就用 only_uavs={u} 對**所有已擺放者**檢查一次，
    #   而已擺放者之後不再變動 ⇒ 每一對 (a,b) 都在較晚那一台被擺放時檢查過。
    #   保留它是**縱深防禦**：只要有人改動上面的插入迴圈（例如改成先全擺再調整、
    #   或讓已擺放者可回頭平移），那個歸納就不成立，而這一行仍然擋得住。
    #   ⇒ 不可以因為「突變咬不到」就把它刪掉，但也不可以宣稱它被測試涵蓋。
    merged = dict(R.plans)
    merged.update(best)
    if check_separation(merged, R.cfg, list(R.ghosts.values()) or None, R.v_ref_mps, R.skip,
                        t_sync_s=R.t_sync_s):
        return set()
    if _fresh_schedule(R, merged).conflicts():
        return set()

    for u, p in best.items():
        R.plans[u] = p
        R.sched.release(u)
        R.sched.reserve_plan(p, R.to_site[u])
    return set(best)


def _takeoff_sites(plans: dict[str, UAVPlan], ctx: MissionContext, takeoff_site: Any) -> dict[str, str]:
    home = ctx.inputs.home.site_id
    if takeoff_site is None:
        return {u: home for u in plans}
    if isinstance(takeoff_site, str):
        return {u: takeoff_site for u in plans}
    return {u: str(takeoff_site.get(u, home)) for u in plans}


def repair(plans: dict[str, UAVPlan], conflicts: list[Conflict], ctx: MissionContext, models: dict[str, EnergyModel],
           schedule: CorridorSchedule, cfg: PlannerConfig, max_rounds: int = 3, *,
           ghosts: Sequence[GhostLike] | None = None, skip_pairs: set[tuple[str, str]] | None = None,
           t_sync_s: float = 0.0, t_now_s: float = -math.inf, takeoff_site: str | dict[str, str] | None = None,
           v_ref_mps: float = 20.0,
           energy_check: Callable[[UAVPlan, EnergyModel | None], bool] | None = None,
           resequence_hook: ResequenceHook | Callable[[str, UAVPlan, Conflict], UAVPlan | None] | None = None,
           ) -> tuple[dict[str, UAVPlan], list[Conflict]]:
    """修復（規格 §13.6，一次算足）：回傳 (新計畫字典, 殘餘衝突)。

    每輪對每筆衝突（時間序；每機每輪至多修改一次）依序嘗試：

    (i) **起飛延遲／換序**（零能量；於 ``schedule`` 重排走廊，t0 須 ≥ max(0, t_now_s)，已起飛者不可）。
        每機給兩個候選：把自己推到「同時滿足走廊時槽與禁止延遲區間」之最小 s（:func:`_fit_takeoff_shift`），
        以及只延自己所需的 s、被擠到的其他地面機連鎖讓開（:func:`_cascade_shift`，即規格 §13.6 (i) 的「換序」）。
    (ii) **延遲盤旋**：於最近前一轉移插入 n = ⌈Δ_req/T_lap⌉ 圈（``energy_check`` 重驗能量，預設
        :func:`default_energy_ok`；走廊衝突改於保持點盤旋）。降落走廊時槽不足時**優先**以連鎖重排讓開，
        而不是把圈數往上加（後者仍保留為後備候選）。
    (iii) **IDP 重排**：``resequence_hook``。新式簽章 ``hook(ResequenceRequest) -> dict[str, UAVPlan] |
        UAVPlan | None``（見 :class:`ResequenceRequest`）；亦相容舊式 ``hook(uav_id, plan, conflict) ->
        UAVPlan | None``（依位置參數個數自動判定）。本模組不實作重排本身——`planner.py` 可用 `beam_idp`
        對該機所屬團隊重跑後回傳新計畫。

    每個候選以乾跑（該機對重跑 :func:`check_separation`／走廊重排）驗證，取修復後 makespan 最小者
    （tie → 總平移／圈數最小），且須通過全域單調性檢查（:meth:`_Repair.involved_score`）。
    走廊衝突「降落優先於起飛」；幽靈航跡不可修改。每輪後以 :func:`check_separation`
    + ``schedule.conflicts()`` 重驗；``max_rounds`` 輪後仍存在者即為殘餘衝突。輸入 ``plans`` 不被修改（受影響機以新
    物件取代）；``schedule`` 就地更新（尚未登記之計畫先依 :func:`plan_windows` 登記，起飛站由 ``takeoff_site`` 給定，
    預設 home）。永不改變航點 z（D18）。
    """
    plans = dict(plans)
    to_site = _takeoff_sites(plans, ctx, takeoff_site)
    for u, p in plans.items():
        if not schedule.reservations_of(u):
            schedule.reserve_plan(p, to_site[u])
    R = _Repair(plans, ctx, models, schedule, cfg, to_site, float(t_now_s), float(t_sync_s), float(v_ref_mps),
                {str(g.uav_id): g for g in (ghosts or ())}, energy_check or default_energy_ok, resequence_hook,
                _norm_pairs(skip_pairs))
    current = _merge_conflicts(conflicts, schedule.conflicts())
    for _ in range(max(0, int(max_rounds))):
        if not current:
            break
        touched: set[str] = set()
        for c in current:
            touched |= _repair_one(c, R)
        if not touched:
            break
        current = _merge_conflicts(
            check_separation(plans, cfg, ghosts, v_ref_mps, skip_pairs, t_sync_s=t_sync_s), schedule.conflicts())
    if _compact_makespan(R):                                    # 工期壓縮（不新增衝突）
        current = _merge_conflicts(
            check_separation(plans, cfg, ghosts, v_ref_mps, skip_pairs, t_sync_s=t_sync_s), schedule.conflicts())
    # vendored 修改 #3：工期壓縮只能逐機平移，協同平移要靠全域重排（見 _global_reschedule）
    if getattr(cfg, "global_reschedule", False) and not current:
        if _global_reschedule(R, step_s=float(getattr(cfg, "reschedule_step_s", 10.0)),
                              objective=str(getattr(cfg, "reschedule_objective", "landing")),
                              max_evals=int(getattr(cfg, "reschedule_max_evals", 20000))):
            current = _merge_conflicts(
                check_separation(plans, cfg, ghosts, v_ref_mps, skip_pairs, t_sync_s=t_sync_s),
                schedule.conflicts())
    return plans, current
