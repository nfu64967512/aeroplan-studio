"""hv_dccpp/sim.py — 驗證檢查與追蹤模擬（契約 §14；規格 §17.1、§13、§5.5、§11.4）。

* 第 4 層末端：只讀 `MissionPlan`／`UAVPlan`／`MissionContext`／`CostCache`，不改動任何規劃結果。
* `coverage_raster`：每區 `cfg.cov_raster_m` 柵格；每架每 `sensor_on` 作業航段依**實際 AGL**（走廊 w_l/2 內最高地形之保守
  AGL）算感測足跡（LiDAR 線掃描連續；相機另依快門率／前向足跡長 l(h) 檢查曝光間距，間距 > l 時只計曝光矩形），
  覆蓋率 ≥ :data:`COVERAGE_PASS`（0.995）通過（`both` 區 lidar／rgb 各自 ≥ 0.995；規格 §17.1 (1)）。
* `check_kinematics`：固定翼段相鄰航點**地面**坡度 ∈ [−V_a sin(max_descent)/V_g, V_a sin(max_climb)·κ/V_g]（規格 §3.2 風三角；
  **max** 值驗、planning 值規劃；弧段 κ = cos φ_max）、轉換段 ≤ γ_tr,max、離散折線曲率（三點外接圓半徑）≥ R_eff、
  AGL ≥ h_clr（d_d 取樣）、CLIMB/DESCENT/ACCEL/DECEL 以外固定翼段 AGL ≥ h_minfw、OPER 段 AGL ∈ [h_min,l, h_max k_GSD]、
  AGL ≤ h_ceil、Waypoint 模式不變量（規格 §17.1 (3)；A7 無懸停轉向）。
* `check_energy`：逐航點（至 P_tc）E ≥ r·E_nom + min_a budget.point_wh（粗取樣 + Lipschitz 細化，等價於逐航點）；
  E(P_tc) ≥ r·E_nom + e_MA；落地 ≥ r·E_nom；D_plan ≤ D_use；首架次落點 = 回收點（規格 §5.5、§17.1 (4)）。
* `check_link_airspace`：航點折線 ⊕ b_trk 在鏈路覆蓋內、≤ ρ^sym、在空域／區域時間窗內、不入有效禁區／keep-out
  （規格 §17.1 (5)）。
* `validate`：以上全部 + §13 分離檢查重跑（最小水平／垂直距離、4D 與走廊衝突）→ :class:`ValidationReport`。
* `track_simulate`：L1 純追蹤（規格 §17.1 (9)）含風（偏流角修正、傾角限、積分步長 ≤ 0.2 s），回報最大橫向偏差、
  能量積分與計畫偏差、落地時刻。
* `baselines`／`paper_table2_check`：規格 §17.1 (6)(7)。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import unary_union

from .altitude import landing_corridor, takeoff_corridor
from .context import MissionContext, build_context
from .costs import CostCache, EnergyBudgetTable
from .deconflict import Conflict, CorridorSchedule, check_separation, orbit_lap_time_s
from .decompose import resolve_sensor
from .geometry import Cell, Strip
from .mission import (FW_PHASES, MissionInputs, MissionPlan, Mode, Phase, PlannerConfig, SortieEnergy, UAVInstance,
                      UAVPlan, UAVState, Waypoint, mode_phase_violation)
from .planner import plan_mission
from .platform import (G_MPS2, PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, EnergyModel, LidarHardware,
                       PlatformSpec)
from .wind import WindField, r_eff_wind_speed

__all__ = [
    "CoverageReport", "ValidationReport", "TrackResult",
    "coverage_raster", "check_kinematics", "check_energy", "check_link_airspace", "check_mission_integrity", "validate",
    "track_simulate", "baselines", "paper_table2_check",
    "build_validation_cache", "homogeneous_fleet", "min_separation", "subsensors",
    "COVERAGE_PASS", "PAPER_PATH_RANGE_KM",
]

COVERAGE_PASS: float = 0.995
"""覆蓋率門檻（規格 §17.1 (1)）。"""
PAPER_PATH_RANGE_KM: tuple[float, float] = (590.0, 650.0)
"""論文 Table 2 情境路徑長之期望範圍（規格 §17.1 (7)）。"""

_AGL_TOL_M: float = 1.0            # AGL 檢查容差（航點刪減後線性內插 vs 規劃取樣；規劃保證於其 25 m 樣本上成立）
_SLOPE_TOL: float = 1e-6           # 坡度正切容差
_CURV_REL_TOL: float = 0.01        # 曲率半徑相對容差（外接圓半徑 ≥ (1−tol)·R_eff）
_CURV_MIN_TURN_RAD: float = math.radians(0.5)   # 低於此轉角不計曲率
_CURV_MIN_CHORD_M: float = 0.5
_ENERGY_TOL_WH: float = 1e-6
_POINT_WH_REFINE_MIN_WH: float = 10.0   # R-06：舊快取無 point_wh_slack_wh 時之後備門檻
_POINT_WH_REFINE_FRAC: float = 0.05     # 同上（取 5 % 可用能量；正常路徑用 budget.point_wh_slack_wh）
_ENERGY_CHECK_DS_M: float = 500.0  # 逐航點備降檢查之粗取樣間距（Lipschitz 細化保證等價於逐航點；planner 用 200 m）
_MAX_DETAIL_PER_KIND: int = 5      # 每類違規最多列出之明細數
_VTOL_PHASES = {Phase.TAKEOFF, Phase.LAND}
_TRANS_PHASES = {Phase.TRANS_OUT, Phase.TRANS_IN}
_LEVEL_PHASES = {Phase.ACCEL, Phase.DECEL}
_CLIMB_DESC_PHASES = {Phase.CLIMB, Phase.DESCENT}
_HMINFW_EXEMPT = _CLIMB_DESC_PHASES | _LEVEL_PHASES      # D35：H_tc < h_minfw 之爬升／下降段
_MISSED_APPROACH_CLIMB_M: float = 50.0                     # 規格 §5.5 e_MA：沿 θ_app 爬至 z_tc + 50
_TRACK_DT_MAX_S: float = 0.2                               # track_simulate 內部積分步長上限
_KEEPOUT_CHUNK: int = 64                                   # check_link_airspace keep-out 分塊腿數


# ---------------------------------------------------------------------------
# 契約資料型別
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class CoverageReport:
    """單區覆蓋報告。``coverage_by_cls`` 鍵：各構型（"A"/"B"：該構型足跡 union）、感測種類（"lidar"/"rgb"）與 "all"
    （全機隊 union）；``cls_required`` = 必須 ≥ :data:`COVERAGE_PASS` 之鍵（any → ["all"]、lidar → ["lidar"]、
    rgb → ["rgb"]、both → ["lidar", "rgb"]）。``uncovered_cells_xy`` (N,2) = 未被全部必要鍵覆蓋之柵格中心（ENU）。"""
    area_id: str
    cls_required: list[str]
    coverage_by_cls: dict[str, float]
    uncovered_cells_xy: np.ndarray
    raster_m: float

    @property
    def passed(self) -> bool:
        return all(self.coverage_by_cls.get(k, 0.0) >= COVERAGE_PASS for k in self.cls_required)

    @property
    def coverage(self) -> float:
        """必要鍵之最小覆蓋率。"""
        return min((self.coverage_by_cls.get(k, 0.0) for k in self.cls_required), default=0.0)


@dataclass(slots=True)
class ValidationReport:
    """整體驗證報告（契約 §14）。``details``：uav_id → 違規字串；另有 "coverage"、"separation" 鍵。"""
    coverage: list[CoverageReport]
    min_sep_h_m: float
    min_sep_v_m: float
    conflicts: list[Conflict]
    slope_ok: bool
    agl_ok: bool
    curvature_ok: bool
    energy_ok: bool
    link_ok: bool
    airspace_ok: bool
    time_window_ok: bool
    makespan_s: float
    total_distance_m: float
    compute_ms: float
    details: dict[str, list[str]]
    passed: bool
    integrity_ok: bool = True
    """R-14：每個 OPERATE 航點 sensor_on ∧ 每條條帶恰好被飛一次（details["integrity"]）。"""
    separation_ok: bool = True
    """R-13：§13 分離檢查確實跑完（丟例外 → False，且 passed 必為 False）。"""

    @property
    def coverage_ok(self) -> bool:
        return all(c.passed for c in self.coverage)


@dataclass(slots=True)
class TrackResult:
    """追蹤模擬結果：最大橫向偏差（m）、能量偏差比例（|E_sim − E_plan| / E_plan）、模擬落地時刻。"""
    max_cross_track_m: float
    energy_dev_frac: float
    t_land_s: float
    n_steps: int = 0
    energy_sim_wh: float = 0.0
    energy_plan_wh: float = 0.0
    completed: bool = True
    wp_at_max_xt: int = -1


@dataclass(slots=True)
class _BudgetCache:
    """`check_energy` 之最小快取（只需 ``budget``）；由 :func:`build_validation_cache(budget_only=True)` 產生。"""
    budget: EnergyBudgetTable
    R_eff: dict[str, float] = field(default_factory=dict)
    W_R: dict[str, float] = field(default_factory=dict)
    strips: dict[str, dict[str, Strip]] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _wp_arrays(wps: Sequence[Waypoint]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x = np.array([w.x_m for w in wps], dtype=float)
    y = np.array([w.y_m for w in wps], dtype=float)
    z = np.array([w.z_m for w in wps], dtype=float)
    t = np.array([w.t_s for w in wps], dtype=float)
    return x, y, z, t


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _cells_by_id(plan: MissionPlan) -> dict[str, Cell]:
    return {c.cell_id: c for c in plan.cells}


def _sensor_for(ctx: MissionContext, cls: str, req_id: str | None) -> Any | None:
    """(cls, req_id) 之感測器（含 both/any 之 ``_rgb`` 尾碼約定）；req_id 未知 → 該構型任一感測器。"""
    if req_id is not None:
        found = resolve_sensor(ctx, cls, req_id)      # 只讀 ctx.sensors（鴨子型別）
        if found is not None:
            return found[0]
    for (c, _r), s in ctx.sensors.items():
        if c == cls:
            return s
    return None


def _strip_sensor(ctx: MissionContext, strip: Strip, cells: Mapping[str, Cell]) -> Any | None:
    cell = cells.get(strip.cell_id)
    return _sensor_for(ctx, strip.cls, cell.payload_req_id if cell is not None else None)


_RUN_STRIDE: int = 1_000_000
"""同機雙感測時，第 i 個子感測器之 run 編號位移（run 數遠小於此值）。"""


def subsensors(sensor: Any) -> tuple[Any, ...]:
    """感測器之子感測器序列：`platform.CombinedSensor` → 其 ``sensors``；其餘 → ``(sensor,)``（規格 §6.2）。"""
    subs = getattr(sensor, "sensors", None)
    if subs:
        return tuple(subs)
    return (sensor,)


def _shutter_hz(sensor: Any) -> float | None:
    r = getattr(sensor, "r", None)
    v = getattr(r, "shutter_rate_hz", None) if r is not None else getattr(sensor, "shutter_rate_hz", None)
    return None if v is None else float(v)


def _dv_of(ctx: MissionContext) -> float:
    return max(0.5 * float(ctx.terrain.step_m), 0.5)


def _h_ceil_of(ctx: MissionContext, cls: str) -> float:
    if ctx.cfg.h_ceil_override_m is not None:
        return float(ctx.cfg.h_ceil_override_m)
    v = ctx.h_ceil_m.get(cls) if ctx.h_ceil_m else None
    return math.inf if v is None else float(v)


def _r_eff_for_plan(spec: PlatformSpec, ctx: MissionContext, wps: Sequence[Waypoint]) -> float:
    """R_eff 之檢查值：規格 §3.2 W_R 政策（與 costs._set_wind_params 相同：[z_site + H_tc,lo, max z_op]），
    z_op 以 OPER 航點 z 之最大值代表（缺 OPER → 固定翼航點最大 z）。"""
    z_site = float(ctx.z_ground_home_m)
    tc = spec.transition_cmd(transition_profile=ctx.cfg.transition_profile)
    z_op = [w.z_m for w in wps if w.phase is Phase.OPERATE]
    if not z_op:
        z_op = [w.z_m for w in wps if w.phase in FW_PHASES]
    z_top = max(z_op) if z_op else z_site + tc.h_tc_cmd_m
    z_top = max(z_top, z_site + spec.min_fw_operation_agl_m)
    W = float(r_eff_wind_speed(ctx.wind, spec, z_site, z_top, ctx.cfg.r_eff_wind_policy))
    return float(spec.r_eff(W))


def _summarise(kind: str, items: list[str]) -> list[str]:
    """同類違規 → 至多 _MAX_DETAIL_PER_KIND 條明細 + 總數。"""
    if not items:
        return []
    out = [f"{kind}: {s}" for s in items[:_MAX_DETAIL_PER_KIND]]
    if len(items) > _MAX_DETAIL_PER_KIND:
        out.append(f"{kind}: ... (+{len(items) - _MAX_DETAIL_PER_KIND} more, total n={len(items)})")
    return out


# ---------------------------------------------------------------------------
# 17.1 (1) 覆蓋柵格
# ---------------------------------------------------------------------------
def _raster_geom(geom: Any, x0: float, y0: float, r: float, nx: int, ny: int
                 ) -> tuple[slice, slice, np.ndarray] | None:
    """shapely 幾何（ENU）→ 柵格子視窗 (row slice, col slice, bool)；柵格中心 x = x0 + (i+0.5) r。"""
    if geom is None or geom.is_empty:
        return None
    minx, miny, maxx, maxy = geom.bounds
    i0 = max(0, int(math.floor((minx - x0) / r - 0.5)))
    i1 = min(nx - 1, int(math.ceil((maxx - x0) / r - 0.5)))
    j0 = max(0, int(math.floor((miny - y0) / r - 0.5)))
    j1 = min(ny - 1, int(math.ceil((maxy - y0) / r - 0.5)))
    if i1 < i0 or j1 < j0:
        return None
    xs = x0 + (np.arange(i0, i1 + 1) + 0.5) * r
    ys = y0 + (np.arange(j0, j1 + 1) + 0.5) * r
    X, Y = np.meshgrid(xs, ys)
    shapely.prepare(geom)
    inside = shapely.contains_xy(geom, X.ravel(), Y.ravel()).reshape(X.shape)
    return slice(j0, j1 + 1), slice(i0, i1 + 1), inside


def _raster_quad(quad: np.ndarray, x0: float, y0: float, r: float, nx: int, ny: int
                 ) -> tuple[slice, slice, np.ndarray] | None:
    """凸四邊形（ENU）→ 柵格子視窗（半平面測試，向量化）；柵格中心 x = x0 + (i+0.5) r。"""
    q = np.asarray(quad, dtype=float).reshape(-1, 2)
    area2 = 0.0
    for k in range(q.shape[0]):
        p, n_ = q[k], q[(k + 1) % q.shape[0]]
        area2 += p[0] * n_[1] - n_[0] * p[1]
    if abs(area2) < 1e-9:
        return None
    if area2 < 0:
        q = q[::-1]
    i0 = max(0, int(math.floor((q[:, 0].min() - x0) / r - 0.5)))
    i1 = min(nx - 1, int(math.ceil((q[:, 0].max() - x0) / r - 0.5)))
    j0 = max(0, int(math.floor((q[:, 1].min() - y0) / r - 0.5)))
    j1 = min(ny - 1, int(math.ceil((q[:, 1].max() - y0) / r - 0.5)))
    if i1 < i0 or j1 < j0:
        return None
    xs = x0 + (np.arange(i0, i1 + 1) + 0.5) * r
    ys = y0 + (np.arange(j0, j1 + 1) + 0.5) * r
    X, Y = np.meshgrid(xs, ys)
    inside = np.ones(X.shape, dtype=bool)
    for k in range(q.shape[0]):
        p, n_ = q[k], q[(k + 1) % q.shape[0]]
        ex, ey = n_[0] - p[0], n_[1] - p[1]
        inside &= (ex * (Y - p[1]) - ey * (X - p[0])) >= -1e-9
    return slice(j0, j1 + 1), slice(i0, i1 + 1), inside


def _raster_ribbon(quads: Sequence[np.ndarray], x0: float, y0: float, r: float, nx: int, ny: int
                   ) -> tuple[slice, slice, np.ndarray] | None:
    """共線四邊形鏈（直線條帶之足跡帶）→ 柵格子視窗；以 (u, v) 條帶座標作 |v| ≤ hw(u) 之向量化測試
    （hw 沿 u 分段線性）。非共線（弧段）→ None（呼叫端退回多邊形測試）。"""
    q = np.asarray(quads, dtype=float)                    # (m, 4, 2)
    c0 = 0.5 * (q[:, 0] + q[:, 3])                        # 各腿起點（中心線）
    c1 = 0.5 * (q[:, 1] + q[:, 2])                        # 各腿終點
    d = c1[-1] - c0[0]
    L = float(np.hypot(*d))
    if L < 1e-6:
        return None
    t_hat = d / L
    n_hat = np.array([t_hat[1], -t_hat[0]])
    pts = np.vstack([c0, c1]) - c0[0]
    if np.abs(pts @ n_hat).max() > 0.05:                  # 非共線
        return None
    u0 = c0 @ t_hat - c0[0] @ t_hat
    u1 = c1 @ t_hat - c0[0] @ t_hat
    hw0 = 0.5 * np.hypot(*(q[:, 0] - q[:, 3]).T)
    hw1 = 0.5 * np.hypot(*(q[:, 1] - q[:, 2]).T)
    us = np.concatenate([u0, u1[-1:]])
    hws = np.concatenate([hw0, hw1[-1:]])
    order = np.argsort(us)
    us, hws = us[order], hws[order]
    minx, miny = q[:, :, 0].min(), q[:, :, 1].min()
    maxx, maxy = q[:, :, 0].max(), q[:, :, 1].max()
    i0 = max(0, int(math.floor((minx - x0) / r - 0.5)))
    i1 = min(nx - 1, int(math.ceil((maxx - x0) / r - 0.5)))
    j0 = max(0, int(math.floor((miny - y0) / r - 0.5)))
    j1 = min(ny - 1, int(math.ceil((maxy - y0) / r - 0.5)))
    if i1 < i0 or j1 < j0:
        return None
    xs = x0 + (np.arange(i0, i1 + 1) + 0.5) * r - c0[0, 0]
    ys = y0 + (np.arange(j0, j1 + 1) + 0.5) * r - c0[0, 1]
    X, Y = np.meshgrid(xs, ys)
    U = X * t_hat[0] + Y * t_hat[1]
    V = X * n_hat[0] + Y * n_hat[1]
    hw_u = np.interp(U, us, hws)
    inside = (U >= us[0] - 1e-9) & (U <= us[-1] + 1e-9) & (np.abs(V) <= hw_u + 1e-9)
    return slice(j0, j1 + 1), slice(i0, i1 + 1), inside


def _chain_polygon(quads: Sequence[np.ndarray]) -> Any:
    """相鄰共邊之四邊形序列 [(pa+ha n, pb+hb n, pb−hb n, pa−ha n), ...] → 單一多邊形（左緣正走、右緣反走）；
    自交（急彎）時退回 unary_union。"""
    if len(quads) == 1:
        return Polygon(quads[0])
    left = [q[0] for q in quads] + [quads[-1][1]]
    right = [q[3] for q in quads] + [quads[-1][2]]
    poly = Polygon(np.vstack([np.asarray(left), np.asarray(right)[::-1]]))
    if poly.is_valid:
        return poly
    return unary_union([Polygon(q) for q in quads])


def _footprint_quads(plan: MissionPlan, ctx: MissionContext, cells: Mapping[str, Cell]
                     ) -> list[tuple[str | None, str, str, np.ndarray]]:
    """每架每 sensor_on 作業航段（航點 j → j+1，屬性取自出發航點 j）之足跡四邊形：
    (area_id | None, cls, kind, quad (4,2))。寬度 = swath(AGL_cons)，AGL_cons = z − 走廊 (w_l/2) 內最高地形（保守）。
    相機：曝光間距 d_exp = V_g/f_s > l(h) 時只計各曝光矩形（規格 §17.1 (1)「相機另算前向重疊」）。"""
    return [(aid, cls, kind, q) for (aid, cls, kind, _run, q) in _footprint_quads_runs(plan, ctx, cells)]


def _footprint_quads_runs(plan: MissionPlan, ctx: MissionContext, cells: Mapping[str, Cell]
                          ) -> list[tuple[str | None, str, str, int, np.ndarray]]:
    """同 :func:`_footprint_quads`，另附 run 編號（同一機、同一條帶之連續 sensor_on 航段共用一個 run；相機曝光
    矩形以負 run 編號標示，不串接）。"""
    strips = plan.strips
    legs: list[tuple[str | None, str, Any, np.ndarray, np.ndarray, float, float, float, float, int]] = []
    run = 0
    prev_key: tuple[str, str] | None = None
    for uid, p in plan.plans.items():
        wps = p.waypoints
        for j in range(len(wps) - 1):
            w0, w1 = wps[j], wps[j + 1]
            if w0.phase is not Phase.OPERATE or not w0.sensor_on or w0.strip_id is None:
                prev_key = None
                continue
            strip = strips.get(w0.strip_id)
            if strip is None:
                prev_key = None
                continue
            sensor = _strip_sensor(ctx, strip, cells)
            if sensor is None:
                prev_key = None
                continue
            p0 = np.array([w0.x_m, w0.y_m])
            p1 = np.array([w1.x_m, w1.y_m])
            L = float(np.hypot(*(p1 - p0)))
            if L < 1e-6:
                continue
            key = (uid, w0.strip_id)
            if key != prev_key:
                run += 1
                prev_key = key
            cell = cells.get(strip.cell_id)
            aid = cell.area_id if cell is not None else None
            legs.append((aid, strip.cls, sensor, p0, p1, float(w0.z_m), float(w1.z_m), 0.5 * float(strip.width_m),
                         float(w0.ground_speed_mps), run))
    if not legs:
        return []
    pts = np.array([[lg[3][0], lg[3][1]] for lg in legs] + [[lg[4][0], lg[4][1]] for lg in legs], dtype=float)
    th = np.array([math.atan2(lg[4][0] - lg[3][0], lg[4][1] - lg[3][1]) for lg in legs] * 2)
    hw = np.array([lg[7] for lg in legs] * 2)
    nat = ctx.terrain.raw_view()                      # 足跡 AGL 以自然地形量（燒入障礙非被測地表）
    zg = nat.corridor_max(pts, th, hw, _dv_of(ctx))
    zg0 = nat.z_at(pts)
    zg = np.where(np.isfinite(zg), zg, np.where(np.isfinite(zg0), zg0, 0.0))
    n = len(legs)
    out: list[tuple[str | None, str, str, int, np.ndarray]] = []
    next_exp: dict[int, float] = {}                       # run_id → 下一次曝光距本腿起點之距離（跨腿延續曝光相位）
    for k, lg in enumerate(legs):
        aid, cls, sensor, p0, p1, z0, z1, _hw, vg, run_id = lg
        agl0 = max(float(z0 - zg[k]), 0.0)
        agl1 = max(float(z1 - zg[n + k]), 0.0)
        d = p1 - p0
        L = float(np.hypot(*d))
        t_hat = d / L
        n_hat = np.array([t_hat[1], -t_hat[0]])          # 右法向 n̂ = (cos θ, −sin θ)
        # 規格 §6.2：同機雙感測（`platform.CombinedSensor`）一次飛行同時滿足兩種需求 → 每個子感測器各出一套
        # 足跡（各用自己的掃幅／曝光模型），run 編號以 _RUN_STRIDE 錯開，避免不同種類之 run 被串成同一條。
        for si, sub in enumerate(subsensors(sensor)):
            rid = run_id + si * _RUN_STRIDE
            kind = str(getattr(sub, "kind", cls))
            hw0 = 0.5 * float(sub.swath_m(agl0))
            hw1 = 0.5 * float(sub.swath_m(agl1))
            pieces: list[tuple[float, float]] = [(0.0, L)]
            f_s = _shutter_hz(sub) if kind == "rgb" else None
            exposure = False
            if f_s is not None and f_s > 0 and vg > 0:
                l_fp = float(sub.footprint_len_m(0.5 * (agl0 + agl1)))
                d_exp = vg / f_s
                if d_exp > l_fp + 1e-9 and l_fp > 0:
                    exposure = True
                    pieces = []
                    s = next_exp.get(rid, 0.0)
                    while s <= L + 1e-9:
                        pieces.append((s - 0.5 * l_fp, s + 0.5 * l_fp))  # 不裁剪：曝光矩形可越過腿端（同 run 直線延續）
                        s += d_exp
                    next_exp[rid] = s - L
            for (sa, sb) in pieces:
                if sb - sa <= 1e-9:
                    continue
                fa, fb = min(max(sa / L, 0.0), 1.0), min(max(sb / L, 0.0), 1.0)
                pa = p0 + sa * t_hat
                pb = p0 + sb * t_hat
                ha = hw0 + fa * (hw1 - hw0)
                hb = hw0 + fb * (hw1 - hw0)
                quad = np.array([pa + ha * n_hat, pb + hb * n_hat, pb - hb * n_hat, pa - ha * n_hat])
                out.append((aid, cls, kind, -rid if exposure else rid, quad))
    return out


def _footprint_geoms(plan: MissionPlan, ctx: MissionContext, cells: Mapping[str, Cell]
                     ) -> list[tuple[str | None, str, str, Any]]:
    """每個 run（同機同條帶之連續作業航段）之足跡多邊形 (area_id | None, cls, kind, geometry)。"""
    quads = _footprint_quads_runs(plan, ctx, cells)
    groups: dict[int, list[np.ndarray]] = {}
    meta: dict[int, tuple[str | None, str, str]] = {}
    exposures: list[tuple[str | None, str, str, Any]] = []
    for (aid, cls, kind, run_id, q) in quads:
        if run_id < 0:
            exposures.append((aid, cls, kind, Polygon(q)))
            continue
        groups.setdefault(run_id, []).append(q)
        meta.setdefault(run_id, (aid, cls, kind))
    out: list[tuple[str | None, str, str, Any]] = []
    for run_id, qs in groups.items():
        aid, cls, kind = meta[run_id]
        out.append((aid, cls, kind, qs))            # 四邊形鏈（共線 → 向量化帶測試；否則多邊形）
    return out + [(aid, cls, kind, [g]) for (aid, cls, kind, g) in exposures]


def _raster_run(item: Any, x0: float, y0: float, r: float, nx: int, ny: int) -> tuple[slice, slice, np.ndarray] | None:
    """run 足跡（四邊形鏈 list[np.ndarray]、單一 Polygon 之 list、或 shapely 幾何）→ 柵格子視窗。"""
    if isinstance(item, list):
        if len(item) == 1 and not isinstance(item[0], np.ndarray):
            return _raster_geom(item[0], x0, y0, r, nx, ny)
        hit = _raster_ribbon(item, x0, y0, r, nx, ny)
        if hit is not None:
            return hit
        return _raster_geom(_chain_polygon(item), x0, y0, r, nx, ny)
    return _raster_geom(item, x0, y0, r, nx, ny)


def _run_polygon(item: Any) -> Any:
    """run 足跡（四邊形鏈 / 單一多邊形之 list / shapely 幾何）→ shapely 幾何。"""
    if isinstance(item, list):
        if len(item) == 1 and not isinstance(item[0], np.ndarray):
            return item[0]
        return unary_union([Polygon(q) for q in item if len(q) >= 3])
    return item


def _areal_coverage(geoms: Sequence[tuple[str | None, str, str, Any]], aid: str, eff: Any,
                    required: Sequence[str]) -> dict[str, float]:
    """精確面積覆蓋率（R-02）：柵格中心全部落在 eff 之外時（細長附屬區、cov_raster_m 大於區寬）改以面積量測，
    絕不以「無樣本 ⇒ 全覆蓋」放行。"""
    polys: dict[str, list[Any]] = {}
    for (q_aid, cls, kind, geom) in geoms:
        if q_aid is not None and q_aid != aid:
            continue
        g = _run_polygon(geom)
        if g is None or g.is_empty:
            continue
        for key in (cls, kind, "all"):
            polys.setdefault(key, []).append(g)
    denom = float(eff.area)
    by: dict[str, float] = {}
    for key, gs in polys.items():
        try:
            inter = unary_union(gs).intersection(eff)
        except Exception:                                   # pragma: no cover - shapely 拓樸例外
            inter = None
        by[key] = float(inter.area / denom) if (inter is not None and denom > 0.0) else 0.0
    for key in required:
        by.setdefault(key, 0.0)
    return by


def _required_keys(sensor_req: str) -> list[str]:
    if sensor_req == "lidar":
        return ["lidar"]
    if sensor_req == "rgb":
        return ["rgb"]
    if sensor_req == "both":
        return ["lidar", "rgb"]
    return ["all"]


def coverage_raster(plan: MissionPlan, ctx: MissionContext, cfg: PlannerConfig) -> list[CoverageReport]:
    """覆蓋完整性（規格 §17.1 (1)）：每區 `cfg.cov_raster_m` 柵格（分母 = 區域 ∖ (NoFly ⊕ b_trk)），每架每 sensor_on
    作業航段依實際 AGL 之足跡 union；回傳每區 :class:`CoverageReport`（≥ :data:`COVERAGE_PASS` 通過）。"""
    raster = float(cfg.cov_raster_m)
    cells = _cells_by_id(plan)
    geoms = _footprint_geoms(plan, ctx, cells)
    nofly = unary_union(list(ctx.nofly_buffered)) if ctx.nofly_buffered else None
    reports: list[CoverageReport] = []
    for area in ctx.inputs.areas:
        aid = area.area_id
        required = _required_keys(area.sensor_req)
        poly = ctx.areas_enu.get(aid)
        if poly is None or poly.is_empty:
            reports.append(CoverageReport(aid, required, {k: 0.0 for k in required}, np.zeros((0, 2)), raster))
            continue
        eff = poly.difference(nofly) if nofly is not None else poly
        if eff.is_empty:
            reports.append(CoverageReport(aid, required, {k: 1.0 for k in required}, np.zeros((0, 2)), raster))
            continue
        minx, miny, maxx, maxy = eff.bounds
        nx = max(1, int(math.ceil((maxx - minx) / raster)))
        ny = max(1, int(math.ceil((maxy - miny) / raster)))
        xs = minx + (np.arange(nx) + 0.5) * raster
        ys = miny + (np.arange(ny) + 0.5) * raster
        X, Y = np.meshgrid(xs, ys)
        shapely.prepare(eff)
        mask = shapely.contains_xy(eff, X.ravel(), Y.ravel()).reshape(ny, nx)
        n_in = int(mask.sum())
        cov: dict[str, np.ndarray] = {}
        for (q_aid, cls, kind, geom) in geoms:
            if q_aid is not None and q_aid != aid:
                continue
            hit = _raster_run(geom, minx, miny, raster, nx, ny)
            if hit is None:
                continue
            rs, cs, sub = hit
            for key in (cls, kind, "all"):
                if key not in cov:
                    cov[key] = np.zeros((ny, nx), dtype=bool)
                cov[key][rs, cs] |= sub
        if not n_in:
            # eff 非空但無任何柵格中心落入（R-02）：改以精確面積量測，不得回報 1.0 讓覆蓋門檻無聲通過。
            by = _areal_coverage(geoms, aid, eff, required)
            worst = min((by.get(k, 0.0) for k in required), default=0.0)
            pt = eff.representative_point()
            unc_xy = np.zeros((0, 2)) if worst >= COVERAGE_PASS else np.array([[pt.x, pt.y]], dtype=float)
            reports.append(CoverageReport(aid, required, by, unc_xy, raster))
            continue
        by: dict[str, float] = {}
        for key, arr in cov.items():
            by[key] = float((arr & mask).sum() / n_in)
        for key in required:
            by.setdefault(key, 0.0)
        req_all = np.ones((ny, nx), dtype=bool)
        for key in required:
            req_all &= cov.get(key, np.zeros((ny, nx), dtype=bool))
        unc = mask & ~req_all
        unc_xy = np.column_stack([X[unc], Y[unc]]) if unc.any() else np.zeros((0, 2))
        reports.append(CoverageReport(aid, required, by, unc_xy, raster))
    return reports


# ---------------------------------------------------------------------------
# 17.1 (3) 坡度／AGL／曲率／模式
# ---------------------------------------------------------------------------
def check_kinematics(plan: UAVPlan, spec: PlatformSpec, ctx: MissionContext, cfg: PlannerConfig, *,
                     strips: Mapping[str, Strip] | None = None) -> list[str]:
    """規格 §17.1 (3)：回傳違規字串清單（空 = 通過）。前綴分類：``mode_phase``、``time_not_monotone``、
    ``slope_exceeded``、``transition_slope_exceeded``、``curvature_below_R_eff``、``agl_below_h_clr``、
    ``agl_below_h_minfw``、``oper_agl_below_h_min``、``oper_agl_above_h_max``、``above_ceiling``、``terrain_unknown``。

    ``strips``（strip_id → Strip）給定時 OPER 段以 w_l/2 走廊與 h_min,l 檢查；缺省時以 b_trk 走廊、跳過 h_min,l。
    坡度以航點弦長計（弧段弦略短於弧長 → 略保守，max/planning 角度差 0.5°–1° 足以吸收）。"""
    wps = plan.waypoints
    out: list[str] = []
    if not wps:
        return out
    v_min = spec.v_min_mission()
    mp: list[str] = []
    for j, w in enumerate(wps):
        v = mode_phase_violation(w.mode, w.phase, w.airspeed_mps, v_min)
        if v is not None:
            mp.append(f"wp={j} {v}")
    out += _summarise("mode_phase", mp)
    x, y, z, t = _wp_arrays(wps)
    n = len(wps)
    if n > 1 and np.any(np.diff(t) < -1e-9):
        out.append(f"time_not_monotone: first at wp={int(np.argmax(np.diff(t) < -1e-9)) + 1}")
    if n < 2:
        return out
    ph = [w.phase for w in wps]
    dx, dy, dz = np.diff(x), np.diff(y), np.diff(z)
    chord = np.hypot(dx, dy)
    # ---- 坡度（地面座標；規格 §3.2 tanγ^g = V_a sinγ κ / V_g，以 max 角驗、風取腿中點 z）---------------
    sin_up = math.sin(math.radians(spec.max_climb_angle_deg))
    sin_dn = math.sin(math.radians(spec.max_descent_angle_deg))
    tan_tr = math.tan(math.radians(spec.gamma_tr_max_deg))
    kappa_arc = math.cos(math.radians(spec.max_bank_angle_deg))
    theta_leg = np.arctan2(dx, dy)
    va_leg = np.array([w.airspeed_mps if w.airspeed_mps > 0 else spec.cruise_airspeed_mps for w in wps[:-1]], dtype=float)
    z_mid = 0.5 * (z[:-1] + z[1:])
    wind_leg = np.asarray(ctx.wind.wind_vec(z_mid), dtype=float).reshape(-1, 2) if ctx.wind is not None else np.zeros((n - 1, 2))
    with np.errstate(invalid="ignore"):
        w_par = wind_leg[:, 0] * np.sin(theta_leg) + wind_leg[:, 1] * np.cos(theta_leg)
        w_perp = wind_leg[:, 0] * np.cos(theta_leg) - wind_leg[:, 1] * np.sin(theta_leg)
        rad = va_leg * va_leg - w_perp * w_perp
        vg_leg = np.where(rad > 0.0, np.sqrt(np.where(rad > 0.0, rad, 0.0)) + w_par, np.nan)   # = wind.ground_speed
    vg_wp = np.array([w.ground_speed_mps for w in wps[:-1]], dtype=float)
    vg_leg = np.where(np.isfinite(vg_leg) & (vg_leg > 0.0), vg_leg, np.where(vg_wp > 0.0, vg_wp, va_leg))
    sl: list[str] = []
    st: list[str] = []
    for j in range(n - 1):
        if chord[j] < 1e-6:
            continue
        s = dz[j] / chord[j]
        if ph[j] in FW_PHASES:
            kappa = kappa_arc if wps[j].arc is not None else 1.0
            tan_up = va_leg[j] * sin_up * kappa / vg_leg[j]
            tan_dn = va_leg[j] * sin_dn / vg_leg[j]
            if s > tan_up + _SLOPE_TOL or s < -tan_dn - _SLOPE_TOL:
                sl.append(f"wp={j} {ph[j].name} ground slope={math.degrees(math.atan(s)):.2f}deg "
                          f"(limit +{math.degrees(math.atan(tan_up)):.2f}/-{math.degrees(math.atan(tan_dn)):.2f} "
                          f"at V_g={vg_leg[j]:.1f}; air +{spec.max_climb_angle_deg}/-{spec.max_descent_angle_deg})")
        elif ph[j] in _TRANS_PHASES:
            if abs(s) > tan_tr + _SLOPE_TOL:
                st.append(f"wp={j} {ph[j].name} slope={math.degrees(math.atan(s)):.2f}deg (limit {spec.gamma_tr_max_deg})")
    out += _summarise("slope_exceeded", sl)
    out += _summarise("transition_slope_exceeded", st)
    # ---- 曲率（三點外接圓半徑；等於外切／內接多邊形所在圓之半徑，與弦長分布無關）----
    R_eff = _r_eff_for_plan(spec, ctx, wps)
    cv: list[str] = []
    for j in range(1, n - 1):
        if ph[j - 1] not in FW_PHASES or ph[j] not in FW_PHASES:
            continue
        a, b = chord[j - 1], chord[j]
        if a < _CURV_MIN_CHORD_M or b < _CURV_MIN_CHORD_M:
            continue
        h0 = math.atan2(dx[j - 1], dy[j - 1])
        h1 = math.atan2(dx[j], dy[j])
        turn = abs(_wrap_pi(h1 - h0))
        if turn < _CURV_MIN_TURN_RAD:
            continue
        c = math.hypot(x[j + 1] - x[j - 1], y[j + 1] - y[j - 1])
        area2 = abs(dx[j - 1] * dy[j] - dy[j - 1] * dx[j])
        if area2 < 1e-12:
            continue
        r_circ = a * b * c / (2.0 * area2)
        if r_circ < R_eff * (1.0 - _CURV_REL_TOL):
            cv.append(f"wp={j} {ph[j].name} R={r_circ:.1f} < R_eff={R_eff:.1f}")
    out += _summarise("curvature_below_R_eff", cv)
    # ---- AGL（d_d 取樣）--------------------------------------------------------
    d_d = float(cfg.d_d_m)
    P: list[np.ndarray] = []
    Z: list[np.ndarray] = []
    TH: list[float] = []
    HW: list[float] = []
    SEG: list[int] = []
    for j in range(n - 1):
        if ph[j] in _VTOL_PHASES:
            continue
        L = chord[j]
        if L < 1e-6:
            continue
        m = max(1, int(math.ceil(L / d_d)))
        f = np.linspace(0.0, 1.0, m + 1)
        if j < n - 2:
            f = f[:-1]           # 接點只留一次（末腿含終點）
        P.append(np.column_stack([x[j] + f * dx[j], y[j] + f * dy[j]]))
        Z.append(z[j] + f * dz[j])
        th = math.atan2(dx[j], dy[j])
        hw = float(cfg.b_trk_m)
        if ph[j] is Phase.OPERATE and strips is not None and wps[j].strip_id in strips:
            hw = 0.5 * float(strips[wps[j].strip_id].width_m)
        TH += [th] * f.size
        HW += [hw] * f.size
        SEG += [j] * f.size
    if not P:
        return out
    pts = np.vstack(P)
    zz = np.concatenate(Z)
    th_a = np.array(TH)
    hw_a = np.array(HW)
    seg = np.array(SEG, dtype=int)
    nat = ctx.terrain.raw_view()                      # AGL 檢查一律以自然地形（與 altitude.sample_bounds 同基準）；
    zg0 = nat.z_at(pts)                                # 障礙淨空由 check_link_airspace 之 keep-out 2.5D 測試負責
    zg_max = nat.corridor_max(pts, th_a, hw_a, _dv_of(ctx))
    zg_min = nat.corridor_min(pts, th_a, hw_a, _dv_of(ctx))
    unknown = ~np.isfinite(zg_max)
    if unknown.any():
        out.append(f"terrain_unknown: n={int(unknown.sum())} samples outside DEM (first wp={int(seg[np.argmax(unknown)])})")
    known = ~unknown
    ph_s = np.array([ph[j] for j in seg], dtype=object)
    is_fw = np.array([p in FW_PHASES for p in ph_s])
    is_oper = np.array([p is Phase.OPERATE for p in ph_s])
    agl_cons = zz - zg_max            # 走廊內最高地形之 AGL
    agl_ctr = zz - np.where(np.isfinite(zg0), zg0, zg_max)
    agl_max = zz - zg_min
    h_clr = float(ctx.h_clr_m)
    h_minfw = float(spec.min_fw_operation_agl_m)
    h_ceil = _h_ceil_of(ctx, cls=plan.cls)
    bad = known & (agl_cons < h_clr - _AGL_TOL_M)
    out += _summarise("agl_below_h_clr", [f"wp={int(seg[k])} {ph_s[k].name} agl={agl_cons[k]:.1f} < {h_clr:.1f}"
                                          for k in np.flatnonzero(bad)])
    need_fw = np.array([p in FW_PHASES and p not in _HMINFW_EXEMPT for p in ph_s])
    bad = known & need_fw & (agl_cons < h_minfw - _AGL_TOL_M)
    out += _summarise("agl_below_h_minfw", [f"wp={int(seg[k])} {ph_s[k].name} agl={agl_cons[k]:.1f} < {h_minfw:.1f}"
                                            for k in np.flatnonzero(bad)])
    if math.isfinite(h_ceil):
        bad = known & is_fw & (agl_ctr > h_ceil + _AGL_TOL_M)
        out += _summarise("above_ceiling", [f"wp={int(seg[k])} {ph_s[k].name} agl={agl_ctr[k]:.1f} > h_ceil={h_ceil:.1f}"
                                            for k in np.flatnonzero(bad)])
    msl_max = ctx.inputs.airspace.max_msl_m
    if msl_max is not None and cfg.h_ceil_override_m is None:
        z_msl_cap = ctx.frame.msl_to_enu_z(pts, float(msl_max))
        bad = is_fw & (zz > z_msl_cap + _AGL_TOL_M)
        out += _summarise("above_ceiling", [f"wp={int(seg[k])} z exceeds max_msl_m={msl_max}" for k in np.flatnonzero(bad)])
    if strips is not None and is_oper.any():
        by_min: dict[str, float] = {}
        by_max: dict[str, float] = {}
        for k in np.flatnonzero(is_oper & known):
            sid = wps[int(seg[k])].strip_id
            if sid is None or sid not in strips:
                continue
            by_min[sid] = min(by_min.get(sid, math.inf), float(agl_cons[k]))
            by_max[sid] = max(by_max.get(sid, -math.inf), float(agl_max[k]))
        cells_req: dict[str, str | None] = {}
        lo_bad: list[str] = []
        hi_bad: list[str] = []
        for sid, a_min in by_min.items():
            s = strips[sid]
            if a_min < s.h_min_m - _AGL_TOL_M:
                lo_bad.append(f"{sid}: {a_min:.1f} < h_min={s.h_min_m:.1f}")
            sensor = _sensor_for(ctx, s.cls, cells_req.get(s.cell_id))
            h_max = float(sensor.agl_window()[1]) * float(cfg.k_gsd) if sensor is not None else math.inf
            if math.isfinite(h_max) and by_max[sid] > h_max + _AGL_TOL_M:
                hi_bad.append(f"{sid}: {by_max[sid]:.1f} > h_max*k_gsd={h_max:.1f}")
        out += _summarise("oper_agl_below_h_min", lo_bad)
        out += _summarise("oper_agl_above_h_max", hi_bad)
    return out


# ---------------------------------------------------------------------------
# 17.1 (4) 能量與備降
# ---------------------------------------------------------------------------
def _missed_approach_wh(spec: PlatformSpec, model: EnergyModel, wind: Any, z_tc: float, r_eff: float) -> float:
    """e_MA（規格 §5.5）= e(沿 θ_app 以 γ_c 爬至 z_tc + 50) + e(一圈下降盤旋) + e^LDG。"""
    gc = spec.gamma_c_auw(model.auw_kg)
    va = float(spec.cruise_airspeed_mps)
    t_climb = _MISSED_APPROACH_CLIMB_M / max(va * math.sin(gc), 1e-6)
    e = model.segment_wh(Mode.CRUISE, t_climb, gc)
    t_lap, _r = orbit_lap_time_s(spec, wind, z_tc + _MISSED_APPROACH_CLIMB_M, r_eff)
    e += model.segment_wh(Mode.CRUISE, t_lap, -spec.gamma_d())
    return float(e + model.e_landing_wh())


def check_energy(plan: UAVPlan, model: EnergyModel, ctx: MissionContext, cache: CostCache | _BudgetCache | Any
                 ) -> list[str]:
    """規格 §5.5 ENERGY_SAFE_CHECK 之事後驗證（§17.1 (4)）：

    * 自首個固定翼航點至承諾點 P_tc（首個 TRANS_IN 航點）逐航點 E_j ≥ r·E_nom + min_a budget.point_wh(x_j, a)
      （航點以 ≥ 200 m 或相位變換處取樣；point_wh 為上界化）；
    * E(P_tc) ≥ r·E_nom + e_MA；落地餘量（末航點 − 懸停保留）≥ r·E_nom；D_plan ≤ D_use；
    * 首架次落點 = 回收點（除非 warnings 含 planned_landing_at_alternate）。
    `cfg.enforce_energy=False`（legacy）→ 空清單。前綴：``energy_alt_unreachable``、``missed_approach_energy``、
    ``landing_reserve``、``usable_distance_exceeded``、``landing_site_not_recovery``、``energy_budget_unavailable``。"""
    out: list[str] = []
    if not ctx.cfg.enforce_energy:
        return out
    wps = plan.waypoints
    if not wps:
        return out
    cls = plan.cls
    spec = ctx.specs[cls]
    reserve = float(model.reserve_wh())
    budget: EnergyBudgetTable | None = getattr(cache, "budget", None)
    sites = list(getattr(budget, "site_ids", [])) if budget is not None else []
    n = len(wps)
    j_tc = next((j for j, w in enumerate(wps) if w.phase is Phase.TRANS_IN), n)
    j_fw = next((j for j, w in enumerate(wps) if w.phase in FW_PHASES), n)
    # ---- 逐航點備降可達（至 P_tc）----------------------------------------------
    # 粗取樣（_ENERGY_CHECK_DS_M）+ Lipschitz 細化：e_alt 沿路徑之變化率 ≤ L = k_unc·P_c/V_g^worst（point_wh 之直線增量），
    # E 單調遞減，故粗樣本裕度 ≥ L·Δs 之區間內不可能有違規；裕度不足之區間逐航點重驗（等價於全逐航點檢查）。
    if budget is not None and sites and j_fw < j_tc:
        unavailable = False
        viol: list[str] = []
        w_max = float(ctx.wind.max_speed()) if ctx.wind is not None else 0.0
        vg_w = max(float(ctx.cfg.v_g_min_mps), float(spec.cruise_airspeed_mps) - w_max)
        lip = float(model.k_unc) * float(model.p_cruise_w()) / vg_w / 3600.0          # Wh per m
        xy_all = np.array([[w.x_m, w.y_m] for w in wps], dtype=float)
        ds = np.concatenate([[0.0], np.hypot(*np.diff(xy_all, axis=0).T)])
        cum = np.cumsum(ds)

        site_order = ([plan.landing_site] if plan.landing_site in sites else []) + \
                     [s_ for s_ in sites if s_ != plan.landing_site]
        e_ref = float(reserve)

        def _need(j: int) -> float:
            """min_a point_wh(x_j, a)；先試落點／回收點，已達 E_j − r·E_nom 之裕度即可短路（min 只需知道 ≤ 門檻）。"""
            w = wps[j]
            avail = float(w.energy_remaining_wh) - e_ref
            best = math.inf
            for s_ in site_order:
                v = float(budget.point_wh(cls, xy_all[j], float(w.z_m), s_))
                best = min(best, v)
                if best <= avail - 2.0 * lip * _ENERGY_CHECK_DS_M:  # 足夠裕度（≥ 細化門檻）→ 不需再看其他站
                    break
            return best

        # R-06：point_wh 是粗篩，低估量的上界由 point_wh_slack_wh 給出（未知航向之 Dubins 繞行 + keep-out 繞行）。
        # 餘裕落在這個帶內就必須精算，否則「E ≥ r·E_nom + point_wh」可能只是粗篩的假象。
        try:
            slack_wh = float(budget.point_wh_slack_wh(cls))
        except (AttributeError, TypeError):                  # 舊快取（無此方法）→ 用保守常數
            slack_wh = math.inf
        if not math.isfinite(slack_wh):
            slack_wh = max(_POINT_WH_REFINE_MIN_WH, _POINT_WH_REFINE_FRAC * float(model.e_usable_wh()))
        refine_wh = float(slack_wh) + _ENERGY_TOL_WH

        def _exact_margin(j: int, e_ref_: float) -> float:
            """AVOID+ENVELOPE 精算之餘裕（全部候選站點取 max 餘裕）；無法精算 → +inf（不改變粗篩結果）。"""
            w = wps[j]
            k = min(j + 1, n - 1)
            th = math.atan2(wps[k].x_m - w.x_m, wps[k].y_m - w.y_m) if k != j else 0.0
            best = -math.inf
            for s_ in site_order:
                try:
                    v = float(budget.point_wh_exact(cls, xy_all[j], float(w.z_m), th, s_))
                except Exception:                                # pragma: no cover
                    continue
                if math.isfinite(v):
                    best = max(best, float(w.energy_remaining_wh) - (e_ref_ + v))
            return best if math.isfinite(best) else math.inf

        coarse: list[int] = []
        last_s = -math.inf
        for j in range(j_fw, j_tc):
            boundary = (j == j_tc - 1) or (wps[j + 1].phase is not wps[j].phase) or (j == j_fw)
            if boundary or cum[j] - last_s >= _ENERGY_CHECK_DS_M:
                coarse.append(j)
                last_s = cum[j]
        margin: dict[int, float] = {}
        for j in coarse:
            e_alt = _need(j)
            if not math.isfinite(e_alt):
                unavailable = True
                margin[j] = math.inf
                continue
            margin[j] = wps[j].energy_remaining_wh - (reserve + e_alt)
        fine: list[int] = []
        for a_, b_ in zip(coarse, coarse[1:]):
            gap = float(cum[b_] - cum[a_])
            if min(margin[a_], margin[b_]) < lip * gap + _ENERGY_TOL_WH:
                fine.extend(range(a_ + 1, b_))
        for j in sorted(set(coarse) | set(fine)):
            m = margin.get(j)
            if m is None:
                e_alt = _need(j)
                if not math.isfinite(e_alt):
                    unavailable = True
                    continue
                m = wps[j].energy_remaining_wh - (reserve + e_alt)
            if m < refine_wh:
                # R-06：point_wh 是 O(1) 粗篩（且刻意寬放 2πR_eff 以成為上界）；餘裕接近門檻時以
                # AVOID+ENVELOPE 精算取代之——精算值才是該點的真實備降能量。
                m_exact = _exact_margin(j, reserve)
                if math.isfinite(m_exact):
                    m = m_exact
            if m < -_ENERGY_TOL_WH:
                w = wps[j]
                viol.append(f"wp={j} {w.phase.name} E={w.energy_remaining_wh:.1f} < r*E_nom+e_alt={w.energy_remaining_wh - m:.1f}")
        out += _summarise("energy_alt_unreachable", viol)
        if unavailable:
            out.append("energy_budget_unavailable: point_wh returned inf for some waypoints")
    elif budget is None or not sites:
        out.append("energy_budget_unavailable: no EnergyBudgetTable/sites in cache")
    # ---- E(P_tc) ≥ r E_nom + e_MA -------------------------------------------------
    if j_tc < n:
        w_tc = wps[j_tc]
        site = plan.landing_site
        z_site = float(ctx.sites_enu[site][1]) if site in ctx.sites_enu else float(ctx.z_ground_home_m)
        tc = spec.transition_cmd(transition_profile=ctx.cfg.transition_profile)
        r_eff = _r_eff_for_plan(spec, ctx, wps)
        e_ma = _missed_approach_wh(spec, model, ctx.wind, z_site + tc.h_tc_cmd_m, r_eff)
        if w_tc.energy_remaining_wh < reserve + e_ma - _ENERGY_TOL_WH:
            out.append(f"missed_approach_energy: E(P_tc)={w_tc.energy_remaining_wh:.1f} < r*E_nom+e_MA={reserve + e_ma:.1f}")
    # ---- 落地餘量 ---------------------------------------------------------------
    e_land = float(wps[-1].energy_remaining_wh) - float(model.segment_wh(Mode.HOVER, spec.t_hover_reserve_s))
    if e_land < reserve - _ENERGY_TOL_WH:
        out.append(f"landing_reserve: E_land={e_land:.1f} < r*E_nom={reserve:.1f}")
    # ---- D_plan ≤ D_use -----------------------------------------------------------
    d_plan = float(model.cruise_equiv_km(max(float(plan.energy_used_wh), 0.0)))
    d_use = float(model.usable_distance_cap_km())
    if d_plan > d_use + 1e-6:
        out.append(f"usable_distance_exceeded: D_plan={d_plan:.1f} km > D_use={d_use:.1f} km")
    # ---- 首架次落點 ---------------------------------------------------------------
    recovery = ctx.recovery_site_id
    if plan.landing_site != recovery and not any(w.startswith("planned_landing_at_alternate") for w in plan.warnings):
        out.append(f"landing_site_not_recovery: {plan.landing_site} != {recovery} without planned_landing_at_alternate")
    return out


# ---------------------------------------------------------------------------
# 17.1 (5) 鏈路／半徑／時間窗／禁區
# ---------------------------------------------------------------------------
def check_link_airspace(plan: UAVPlan, ctx: MissionContext, *, cells: Mapping[str, Cell] | None = None) -> list[str]:
    """規格 §17.1 (5)：航點折線 ⊕ b_trk ⊆ P_link,c、距回收點／地面站 ≤ ρ^sym_c、時間在空域窗（與區域窗，
    ``cells`` 給定時）內、不入有效禁區（``ctx.keepouts`` 逐腿以 z 提示檢查；缺省時以 ``ctx.nofly_buffered`` 2-D）。
    前綴：``outside_link``、``beyond_rho_sym``、``outside_airspace_window``、``outside_area_window``、
    ``nofly_intersect``、``keepout_intersect``。"""
    out: list[str] = []
    wps = plan.waypoints
    if len(wps) < 2:
        return out
    cfg = ctx.cfg
    cls = plan.cls
    x, y, z, t = _wp_arrays(wps)
    xy = np.column_stack([x, y])
    line = LineString(xy)
    b_trk = float(cfg.b_trk_m)
    if cfg.enforce_link:
        lp = ctx.link_poly.get(cls)
        if lp is not None and not lp.is_empty:
            swept = line.buffer(b_trk)
            if not lp.buffer(1e-6).contains(swept):
                inside = shapely.contains_xy(lp, x, y)
                excess = float(swept.difference(lp).area)
                out.append(f"outside_link: {int((~inside).sum())} waypoints outside P_link[{cls}], swept excess area {excess:.0f} m2")
        rho = float(ctx.rho_sym_m.get(cls, math.inf))
        if math.isfinite(rho):
            d = np.hypot(x - ctx.home_xy[0], y - ctx.home_xy[1]) + b_trk
            if d.max() > rho + 1e-6:
                out.append(f"beyond_rho_sym: max d={d.max():.0f} m > rho_sym={rho:.0f} m (wp={int(np.argmax(d))})")
    if cfg.enforce_time_windows:
        win = ctx.inputs.airspace.time_window_s
        if win is not None:
            lo, hi = float(win[0]), float(win[1])
            if t.min() < lo - 1e-9 or t.max() > hi + 1e-9:
                out.append(f"outside_airspace_window: t in [{t.min():.0f}, {t.max():.0f}] vs window [{lo:.0f}, {hi:.0f}]")
        if cells:
            bad: list[str] = []
            for j, w in enumerate(wps):
                if w.phase is not Phase.OPERATE or w.cell_id is None:
                    continue
                c = cells.get(w.cell_id)
                if c is None or c.time_window_s is None:
                    continue
                lo, hi = float(c.time_window_s[0]), float(c.time_window_s[1])
                if w.t_s < lo - 1e-9 or w.t_s > hi + 1e-9:
                    bad.append(f"wp={j} {c.area_id}/{c.cell_id} t={w.t_s:.0f} not in [{lo:.0f}, {hi:.0f}]")
            out += _summarise("outside_area_window", bad)
    ko = ctx.keepouts
    if ko is not None and len(ko) > 0:
        hits: list[str] = []
        n = len(wps)
        j = 0
        while j < n - 1:                                  # 分塊（≤ _KEEPOUT_CHUNK 腿、z 提示取塊內最小 = 保守）→ 命中時逐腿定位
            if wps[j].phase in _VTOL_PHASES:
                j += 1
                continue
            j_end = j
            while j_end < n - 1 and j_end - j < _KEEPOUT_CHUNK and wps[j_end].phase not in _VTOL_PHASES:
                j_end += 1
            if j_end - j >= 1:
                chunk = LineString(xy[j:j_end + 1])
                if chunk.length > 1e-6 and ko.intersects(chunk, b_trk, float(z[j:j_end + 1].min())):
                    for jj in range(j, j_end):
                        seg = LineString([xy[jj], xy[jj + 1]])
                        if seg.length > 1e-6 and ko.intersects(seg, b_trk, float(min(z[jj], z[jj + 1]))):
                            hits.append(f"wp={jj} {wps[jj].phase.name}")
            j = max(j_end, j + 1)
        out += _summarise("keepout_intersect", hits)
    elif ctx.nofly_buffered:
        hits = []
        for k, nf in enumerate(ctx.nofly_buffered):
            if nf.buffer(-0.5).intersects(line):
                hits.append(f"nofly#{k}")
        out += _summarise("nofly_intersect", hits)
    return out


# ---------------------------------------------------------------------------
# 17.1 (2) 最小分離
# ---------------------------------------------------------------------------
def min_separation(plans: Mapping[str, UAVPlan], cfg: PlannerConfig) -> tuple[float, float, float, tuple[str, str] | None]:
    """所有機對於共同時間格（Δt = cfg.dt_4d_s）之最小水平距離、以及水平 < d_sep 時之最小垂直距離；
    回傳 (min_h, min_v, t_at_min_h, pair)。單機 → (inf, inf, nan, None)。"""
    items = [(u, p) for u, p in plans.items() if len(p.waypoints) >= 1]
    best_h, best_v, t_best, pair = math.inf, math.inf, math.nan, None
    for a in range(len(items)):
        ua, pa = items[a]
        xa, ya, za, ta = _wp_arrays(pa.waypoints)
        for b in range(a + 1, len(items)):
            ub, pb = items[b]
            xb, yb, zb, tb = _wp_arrays(pb.waypoints)
            lo, hi = max(ta[0], tb[0]), min(ta[-1], tb[-1])
            if hi < lo:
                continue
            grid = np.arange(lo, hi + 1e-9, float(cfg.dt_4d_s))
            if grid.size == 0 or grid[-1] < hi - 1e-9:
                grid = np.append(grid, hi)
            xa_i, ya_i, za_i = np.interp(grid, ta, xa), np.interp(grid, ta, ya), np.interp(grid, ta, za)
            xb_i, yb_i, zb_i = np.interp(grid, tb, xb), np.interp(grid, tb, yb), np.interp(grid, tb, zb)
            dh = np.hypot(xa_i - xb_i, ya_i - yb_i)
            dv = np.abs(za_i - zb_i)
            k = int(np.argmin(dh))
            if dh[k] < best_h:
                best_h, t_best, pair = float(dh[k]), float(grid[k]), (ua, ub)
            close = dh < float(cfg.d_sep_m)
            if close.any():
                best_v = min(best_v, float(dv[close].min()))
    return best_h, best_v, t_best, pair


# ---------------------------------------------------------------------------
# validate
# ---------------------------------------------------------------------------
_SLOPE_KEYS = ("slope_exceeded", "transition_slope_exceeded")
_AGL_KEYS = ("agl_below_h_clr", "agl_below_h_minfw", "oper_agl_below_h_min", "oper_agl_above_h_max", "terrain_unknown")
_CURV_KEYS = ("curvature_below_R_eff",)
_ENERGY_KEYS = ("energy_alt_unreachable", "missed_approach_energy", "landing_reserve", "usable_distance_exceeded",
                "landing_site_not_recovery", "energy_budget_unavailable")
_LINK_KEYS = ("outside_link", "beyond_rho_sym")
_AIRSPACE_KEYS = ("above_ceiling", "nofly_intersect", "keepout_intersect")
_WINDOW_KEYS = ("outside_airspace_window", "outside_area_window")
_OTHER_KEYS = ("mode_phase", "time_not_monotone")
_INTEGRITY_KEYS = ("oper_sensor_off", "oper_without_strip", "sensor_on_outside_oper", "strip_unknown",
                   "strip_not_flown", "strip_flown_twice")


def check_mission_integrity(plan: MissionPlan) -> list[str]:
    """任務完整性（規格 §17.1 (1) 之前提；R-14 新增）：回傳違規字串清單（空 = 通過）。

    * ``oper_sensor_off(<uav>@<j>)``：OPERATE 航點未開感測器 —— 該段作業不會產生資料，覆蓋率卻仍以「計畫」為真。
    * ``oper_without_strip(<uav>@<j>)``：OPERATE 航點無 ``strip_id``，無法對應到任何條帶。
    * ``sensor_on_outside_oper(<uav>@<j>)``：非 OPERATE 航點開著感測器。
    * ``strip_unknown(<uav>:<sid>)``：飛到 ``plan.strips`` 沒有的條帶。
    * ``strip_not_flown(<sid>)``：條帶存在但沒有任何一架飛（已列在 ``plan.uncovered`` 者除外，另行報告）。
    * ``strip_flown_twice(<sid>: n=<k>)``：同一條帶被飛了兩次以上（同機同條帶之連續航點段算一次）。
    """
    out: list[str] = []
    flown: dict[str, int] = {}
    for uid, p in plan.plans.items():
        seq: list[str] = []                     # 該機依序作業之條帶（連續同一條帶只記一次；中間插入之延遲盤旋
        for j, w in enumerate(p.waypoints):     # 等非 OPERATE 航點不算切斷）
            if w.phase is Phase.OPERATE:
                if not w.sensor_on:
                    out.append(f"oper_sensor_off({uid}@{j})")
                if w.strip_id is None:
                    out.append(f"oper_without_strip({uid}@{j})")
                    continue
                if not seq or seq[-1] != w.strip_id:
                    seq.append(w.strip_id)
                    if w.strip_id not in plan.strips:
                        out.append(f"strip_unknown({uid}:{w.strip_id})")
            elif w.sensor_on:
                out.append(f"sensor_on_outside_oper({uid}@{j})")
        for sid in seq:
            flown[sid] = flown.get(sid, 0) + 1
    known_uncovered = {a for a, _r in plan.uncovered}
    for sid in plan.strips:
        n = flown.get(sid, 0)
        if n == 0:
            if sid not in known_uncovered:
                out.append(f"strip_not_flown({sid})")
        elif n > 1:
            out.append(f"strip_flown_twice({sid}: n={n})")
    return out


def _has(details: Mapping[str, list[str]], keys: Iterable[str]) -> bool:
    ks = tuple(keys)
    return any(s.startswith(k) for lst in details.values() for s in lst for k in ks)


def validate(plan: MissionPlan, ctx: MissionContext, cache: CostCache | _BudgetCache | Any, cfg: PlannerConfig,
             *, t_sync_s: float = 0.0) -> ValidationReport:
    """整體驗證（契約 §14；規格 §17.1 (1)–(5)）。``cache`` 只需 ``.budget``（:func:`build_validation_cache`）。

    ``passed`` = 覆蓋全部通過 ∧ 任務完整性通過 ∧ 分離檢查「有跑完且無衝突」∧ 所有旗標 True ∧ 無模式／時間單調違規。

    ``t_sync_s``（R-12）：§13.3 之 σ_t = k_t (t − t_sync) 基準時刻 = 最近一次遙測快照。初始計畫用 0；驗證
    ``dynamic.replan`` 產出的中途計畫時**必須**傳入該次事件的 ``t_now``，否則 σ_t 會被灌上 k_t·t_now，
    驗證器會回報重規劃器判定不存在的衝突。
    """
    t0 = time.perf_counter()
    cells = _cells_by_id(plan)
    coverage = coverage_raster(plan, ctx, cfg)
    details: dict[str, list[str]] = {}
    for uid, p in plan.plans.items():
        spec = ctx.specs[p.cls]
        model = ctx.models.get(uid) or ctx.models_rep[p.cls]
        lst = check_kinematics(p, spec, ctx, cfg, strips=plan.strips)
        lst += check_energy(p, model, ctx, cache)
        lst += check_link_airspace(p, ctx, cells=cells)
        details[uid] = lst
    # ---- 分離（§13 重跑）----------------------------------------------------------
    conflicts: list[Conflict] = []
    separation_ok = True
    min_h, min_v, t_min, pair = math.inf, math.inf, math.nan, None
    if plan.plans:
        try:
            conflicts += check_separation(dict(plan.plans), cfg, t_sync_s=float(t_sync_s))
        except ValueError as exc:
            # R-13：分離檢查「跑不起來」不等於「沒有衝突」；必須自己讓 passed=False（舊碼只把訊息塞進
            # details["separation"]，而 _has() 只掃 per_uav，於是 conflicts 空 → passed 條件被滿足）。
            separation_ok = False
            details.setdefault("separation", []).append(f"check_separation_failed: {exc}")
        sched = CorridorSchedule(cfg)
        home = ctx.inputs.home.site_id
        for p in plan.plans.values():
            sched.reserve_plan(p, home)
        conflicts += sched.conflicts()
        min_h, min_v, t_min, pair = min_separation(plan.plans, cfg)
    sep_lines = [f"min_sep_h={min_h:.1f} m at t={t_min:.1f} s pair={pair}", f"min_sep_v(d_h<d_sep)={min_v:.1f} m",
                 f"conflicts={len(conflicts)}"]
    for c in conflicts[:_MAX_DETAIL_PER_KIND]:
        sep_lines.append(f"conflict {c.kind} {c.uav_a}-{c.uav_b} t={c.t_s:.0f} d_h={c.d_h_m:.0f} d_v={c.d_v_m:.0f}")
    details.setdefault("separation", []).extend(sep_lines)
    details["coverage"] = [f"{c.area_id}: " + ", ".join(f"{k}={v:.4f}" for k, v in sorted(c.coverage_by_cls.items()))
                          + f" required={c.cls_required} {'PASS' if c.passed else 'FAIL'}" for c in coverage]
    integrity = check_mission_integrity(plan)
    if integrity:
        details["integrity"] = integrity[:_MAX_DETAIL_PER_KIND * 4] + (
            [f"... (+{len(integrity) - _MAX_DETAIL_PER_KIND * 4} more)"] if len(integrity) > _MAX_DETAIL_PER_KIND * 4 else [])
    per_uav = {u: details[u] for u in plan.plans}
    slope_ok = not _has(per_uav, _SLOPE_KEYS)
    agl_ok = not _has(per_uav, _AGL_KEYS)
    curvature_ok = not _has(per_uav, _CURV_KEYS)
    energy_ok = not _has(per_uav, _ENERGY_KEYS)
    link_ok = not _has(per_uav, _LINK_KEYS)
    airspace_ok = not _has(per_uav, _AIRSPACE_KEYS)
    time_window_ok = not _has(per_uav, _WINDOW_KEYS)
    other_ok = not _has(per_uav, _OTHER_KEYS)
    cov_ok = all(c.passed for c in coverage)
    integrity_ok = not integrity
    passed = bool(cov_ok and integrity_ok and separation_ok and not conflicts and slope_ok and agl_ok
                  and curvature_ok and energy_ok and link_ok and airspace_ok and time_window_ok and other_ok)
    makespan = float(plan.makespan_s) if plan.makespan_s else max((p.t_land_s for p in plan.plans.values()), default=0.0)
    total_d = float(sum(p.distance_m for p in plan.plans.values()))
    return ValidationReport(coverage=coverage, min_sep_h_m=min_h, min_sep_v_m=min_v, conflicts=conflicts,
                            slope_ok=slope_ok, agl_ok=agl_ok, curvature_ok=curvature_ok, energy_ok=energy_ok,
                            link_ok=link_ok, airspace_ok=airspace_ok, time_window_ok=time_window_ok,
                            makespan_s=makespan, total_distance_m=total_d,
                            compute_ms=(time.perf_counter() - t0) * 1e3, details=details, passed=passed,
                            integrity_ok=integrity_ok, separation_ok=separation_ok)


# ---------------------------------------------------------------------------
# 快取重建（plan_mission 不回傳 CostCache）
# ---------------------------------------------------------------------------
def _nearest_entry(strips: Sequence[Strip], xy: np.ndarray) -> tuple[Any, float]:
    best: tuple[float, Strip, int] | None = None
    for s in strips:
        for side, d in ((s.pL, -1), (s.pR, 1)):
            dist = float(math.hypot(side[0] - xy[0], side[1] - xy[1]))
            if best is None or dist < best[0]:
                best = (dist, s, d)
    assert best is not None
    return best[1].start_state(best[2]), float(best[1].z_op_m)


def build_validation_cache(plan: MissionPlan, ctx: MissionContext, *, budget_only: bool = False
                           ) -> CostCache | _BudgetCache:
    """自 `MissionPlan.strips` 重建成本快取（`plan_mission` 不回傳其 CostCache）：條帶依 `Strip.cls` 分組、站點
    S^app 以 `altitude.landing_corridor`（回收點 + preflight 可用備降點）、起飛狀態以 `altitude.takeoff_corridor`
    （最近條帶端點；``budget_only=False`` 時）。``budget_only=True`` → 只建 :class:`EnergyBudgetTable`
    （`check_energy`／`validate` 唯一需要的部分，快得多）。"""
    strips_by: dict[str, dict[str, Strip]] = {}
    for sid, s in plan.strips.items():
        strips_by.setdefault(s.cls, {})[sid] = s
    fleet = [u for u in ctx.inputs.fleet if u.cls in strips_by]
    recovery = ctx.recovery_site_id
    site_ids = [recovery]
    for c in ctx.specs:
        for s in (plan.feasibility.derived.get(c, {}) or {}).get("usable_alternates", []) or []:
            if s in ctx.sites_enu and s not in site_ids:
                site_ids.append(s)
    cfg = ctx.cfg
    sites: dict[str, dict[str, Any]] = {}
    for sid in site_ids:
        sites[sid] = {}
        for c, d in strips_by.items():
            us = [u for u in fleet if u.cls == c]
            if not d or not us:
                continue
            s0 = next(iter(d.values()))
            ld = landing_corridor(sid, us[0], ctx, s0.end_state(-1), float(s0.z_op_m))
            if ld.app_state is None:
                continue
            z_tc = float(ctx.sites_enu[sid][1]) + ctx.specs[c].transition_cmd(transition_profile=cfg.transition_profile).h_tc_cmd_m
            sites[sid][c] = (ld.app_state, z_tc)
    sites = {k: v for k, v in sites.items() if v}
    r_eff: dict[str, float] = {}
    w_r: dict[str, float] = {}
    z_site = float(ctx.z_ground_home_m)
    for c, d in strips_by.items():
        spec = ctx.specs[c]
        zs = [float(s.z_op_m) for s in d.values()]
        z_top = max(max(zs) if zs else z_site + spec.transition_cmd().h_tc_cmd_m, z_site + spec.min_fw_operation_agl_m)
        w_r[c] = float(r_eff_wind_speed(ctx.wind, spec, z_site, z_top, cfg.r_eff_wind_policy))
        r_eff[c] = float(spec.r_eff(w_r[c]))
    if budget_only:
        budget = EnergyBudgetTable(strips_by, sites, ctx.models_rep, ctx.specs, ctx.wind, ctx.keepouts, ctx.terrain, cfg,
                                   h_clr_m=ctx.h_clr_m, r_eff_m=r_eff)
        return _BudgetCache(budget=budget, R_eff=r_eff, W_R=w_r, strips=strips_by)
    home = ctx.inputs.home.site_id
    init_states: dict[str, UAVState] = {}
    init_laps: dict[str, tuple[int, float]] = {}
    for u in fleet:
        cl = list(strips_by.get(u.cls, {}).values())
        if not cl:
            continue
        s_entry, z_entry = _nearest_entry(cl, ctx.home_xy)
        to = takeoff_corridor(home, u, ctx, s_entry, z_entry, 0.0)
        init_states[u.uav_id] = to.exit_state
        init_laps[u.uav_id] = (int(to.n_laps), 0.0)
    cells = _cells_by_id(plan)
    strip_sensors: dict[str, Any] = {}
    for sid, s in plan.strips.items():
        sen = _strip_sensor(ctx, s, cells)
        if sen is not None:
            strip_sensors[sid] = sen
    return CostCache.build(strips_by, fleet, init_states, init_laps, sites, ctx, strip_sensors=strip_sensors)


# ---------------------------------------------------------------------------
# 17.1 (9) L1 追蹤模擬
# ---------------------------------------------------------------------------
def _l1_distance_m(spec: PlatformSpec) -> float:
    """L1 前視距離：3 V_a，夾於 [40 m, 0.9 R_plan]（L1 < 2R 時 L1 法則對圓弧為精確追蹤）。"""
    return float(min(max(3.0 * spec.cruise_airspeed_mps, 40.0), 0.9 * spec.turn_radius_planning_m))


def _wind_at(wind: Any, z: float) -> tuple[float, float]:
    if wind is None:
        return 0.0, 0.0
    if hasattr(wind, "wind_vec"):
        w = np.asarray(wind.wind_vec(float(z)), dtype=float).reshape(-1, 2)[0]
        return float(w[0]), float(w[1])
    w = np.asarray(wind, dtype=float).reshape(2)
    return float(w[0]), float(w[1])


def track_simulate(plan: UAVPlan, spec: PlatformSpec, wind: WindField, model: EnergyModel, cfg: PlannerConfig,
                   dt_s: float = 0.5, *, trace: list[tuple[float, ...]] | None = None) -> TrackResult:
    """L1 純追蹤 + 風（規格 §17.1 (9)）：沿固定翼航點折線（首個固定翼航點 → DECEL 終點）以空速 V_a（航點值）、
    橫向加速度 a = 2V_g² sin η / L1（限 g tan φ_max）飛行；風向量取自 `wind.wind_vec(z)`；z 沿計畫剖面跟隨（無垂直動力學）。
    能量：逐步 P_mode(γ)·dt·k_unc；VTOL／轉換段沿用計畫值。回報最大橫向偏差（對折線）、|E_sim − E_plan|/E_plan、落地時刻。"""
    wps = plan.waypoints
    if not wps:
        return TrackResult(0.0, 0.0, float(plan.t_land_s), 0, 0.0, 0.0, True)
    n = len(wps)
    i0 = next((j for j, w in enumerate(wps) if w.phase in FW_PHASES), None)
    if i0 is None:
        return TrackResult(0.0, 0.0, float(plan.t_land_s), 0, 0.0, float(plan.energy_used_wh), True)
    i1 = max(j for j, w in enumerate(wps) if w.phase in FW_PHASES)
    i_end = min(i1 + 1, n - 1)                       # 末固定翼腿終點（TRANS_IN 起點 = P_tc）
    if i_end <= i0:
        return TrackResult(0.0, 0.0, float(plan.t_land_s), 0, 0.0, float(plan.energy_used_wh), True)
    P = np.array([[w.x_m, w.y_m] for w in wps[i0:i_end + 1]], dtype=float)
    Zr = np.array([w.z_m for w in wps[i0:i_end + 1]], dtype=float)
    K = P.shape[0] - 1
    leg_v = np.array([wps[i0 + k].airspeed_mps if wps[i0 + k].airspeed_mps > 0 else spec.cruise_airspeed_mps
                      for k in range(K)])
    leg_mode = [("ACCEL" if wps[i0 + k].phase in _LEVEL_PHASES else Mode.CRUISE) for k in range(K)]
    d = np.diff(P, axis=0)
    L = np.hypot(d[:, 0], d[:, 1])
    L_safe = np.where(L > 1e-9, L, 1.0)
    cum = np.concatenate([[0.0], np.cumsum(L)])
    total = float(cum[-1])
    L1 = _l1_distance_m(spec)
    # 空氣座標橫向加速度上限：傾角限 g·tanφ_max 與表 1 R_theo 所隱含之 V_a²/R_theo 取大（R_eff 依 D1 由 R_theo 定義）
    a_max = max(G_MPS2 * math.tan(math.radians(spec.max_bank_angle_deg)),
                spec.cruise_airspeed_mps ** 2 / max(float(spec.turn_radius_theoretical_m), 1.0))
    accept = float(getattr(wps[i_end], "accept_radius_m", 30.0))
    # 初始：位置 = 首航點，航向由風三角
    x, y = float(P[0, 0]), float(P[0, 1])
    z = float(Zr[0])
    th0 = math.atan2(d[0, 0], d[0, 1]) if L[0] > 1e-9 else 0.0
    wx, wy = _wind_at(wind, z)
    va0 = float(leg_v[0])
    w_perp = wx * math.cos(th0) - wy * math.sin(th0)
    psi = th0 - math.asin(max(-1.0, min(1.0, w_perp / va0))) if va0 > 0 else th0
    k = 0
    t_sim = 0.0
    e_sim = 0.0
    max_xt = 0.0
    t_plan_fw = float(wps[i_end].t_s - wps[i0].t_s)
    t_cap = max(3.0 * t_plan_fw, 60.0) + 600.0
    completed = False
    steps = 0
    wp_at_max = i0
    dt = min(float(dt_s), _TRACK_DT_MAX_S)       # Euler 積分步長上限（傾角極限附近之弧段對 dt 敏感）
    if dt <= 0.0:
        raise ValueError("dt_s must be > 0")
    while t_sim < t_cap:
        # ---- 投影至現行腿（可跨腿前進）----
        while True:
            dxp, dyp = x - P[k, 0], y - P[k, 1]
            s = (dxp * d[k, 0] + dyp * d[k, 1]) / (L_safe[k] ** 2)
            if s >= 1.0 and k < K - 1:
                k += 1
                continue
            break
        s_cl = min(max(s, 0.0), 1.0)
        # 橫向偏差：對鄰近腿取最小
        xt = math.inf
        for kk in range(max(0, k - 1), min(K, k + 2)):
            ddx, ddy = x - P[kk, 0], y - P[kk, 1]
            ss = min(max((ddx * d[kk, 0] + ddy * d[kk, 1]) / (L_safe[kk] ** 2), 0.0), 1.0)
            px, py = P[kk, 0] + ss * d[kk, 0], P[kk, 1] + ss * d[kk, 1]
            xt = min(xt, math.hypot(x - px, y - py))
        if xt > max_xt:
            max_xt, wp_at_max = xt, i0 + k
        s_along = float(cum[k] + s_cl * L[k])
        if k == K - 1 and (s >= 1.0 or math.hypot(x - P[K, 0], y - P[K, 1]) <= accept):
            completed = True
            break
        # ---- 前視點（沿折線 L1）----
        s_la = min(s_along + L1, total)
        kk = int(np.searchsorted(cum, s_la, side="right") - 1)
        kk = min(max(kk, 0), K - 1)
        fr = (s_la - cum[kk]) / L_safe[kk] if L[kk] > 1e-9 else 1.0
        lx, ly = P[kk, 0] + fr * d[kk, 0], P[kk, 1] + fr * d[kk, 1]
        # ---- 風三角、地速 ----
        va = float(leg_v[k])
        wx, wy = _wind_at(wind, z)
        vx, vy = va * math.sin(psi) + wx, va * math.cos(psi) + wy
        vg = math.hypot(vx, vy)
        if vg < 1e-6:
            vg = 1e-6
        th_g = math.atan2(vx, vy)
        eta = _wrap_pi(math.atan2(lx - x, ly - y) - th_g)
        a_g = 2.0 * vg * vg * math.sin(eta) / L1        # L1 地面航跡橫向加速度指令（圓弧上 η = asin(L1/2R) ⇒ a = V_g²/R，無需前饋）
        cos_crab = max(math.cos(_wrap_pi(psi - th_g)), 0.5)
        psi_dot = a_g / (va * cos_crab)                                               # θ̇_g = V_a ψ̇ cosδ / V_g ⇒ ψ̇ = a_g/(V_a cosδ)
        psi_dot = max(-a_max / va, min(a_max / va, psi_dot))                          # 傾角限：V_a ψ̇ ≤ a_max
        if trace is not None:
            trace.append((t_sim, x, y, z, i0 + k, xt, psi, th_g, vg, psi_dot, a_g))
        psi += psi_dot * dt
        x += vx * dt
        y += vy * dt
        # ---- z 跟隨計畫剖面、能量 ----
        s_new = min(s_along + vg * dt, total)
        z_new = float(np.interp(s_new, cum, Zr))
        gamma = math.asin(max(-1.0, min(1.0, (z_new - z) / (va * dt)))) if va > 0 else 0.0
        e_sim += float(model.segment_wh(leg_mode[k], dt, gamma))
        z = z_new
        t_sim += dt
        steps += 1
    e_plan_fw = float(wps[i0].energy_remaining_wh - wps[i_end].energy_remaining_wh)
    e_pre = float(wps[0].energy_remaining_wh - wps[i0].energy_remaining_wh)
    e_post = float(wps[i_end].energy_remaining_wh - wps[-1].energy_remaining_wh) \
        + float(model.segment_wh(Mode.HOVER, spec.t_hover_reserve_s))
    e_total_sim = e_pre + e_sim + e_post
    e_total_plan = float(plan.energy_used_wh) if plan.energy_used_wh > 0 else (e_pre + e_plan_fw + e_post)
    dev = abs(e_total_sim - e_total_plan) / e_total_plan if e_total_plan > 0 else 0.0
    t_land = float(wps[i0].t_s) + t_sim + float(wps[-1].t_s - wps[i_end].t_s) + float(spec.t_hover_reserve_s)
    if not completed:
        max_xt = math.inf
    return TrackResult(float(max_xt), float(dev), t_land, steps, e_total_sim, e_total_plan, completed, int(wp_at_max))


# ---------------------------------------------------------------------------
# 17.1 (6) 基線比較、(7) 論文情境
# ---------------------------------------------------------------------------
def homogeneous_fleet(inputs: MissionInputs, cls: str) -> MissionInputs:
    """同構機隊：每架換成該構型（機隊中該構型首架為範本；缺則 PLATFORM_PIONEER／HERO + 標稱電池）；A 範本另掛
    RGB 相機（若原機隊含 rgb 硬體且範本無）以承接 rgb 區；uav_id 與 `start_site_id` 不變（C2 策略鍵與起飛站點保留）。"""
    tmpl = next((u for u in inputs.fleet if u.cls == cls), None)
    kinds_fleet = {hw.kind for u in inputs.fleet for hw in u.sensors}
    if tmpl is None:
        spec = PLATFORM_PIONEER if cls == "A" else PLATFORM_HERO
        sensors: tuple[Any, ...] = (LidarHardware(),) if cls == "A" else (CameraHardware(),)
        nominal = float(spec.battery_nominal_wh or (1500.0 if cls == "A" else 420.0))
        sortie = SortieEnergy("tmpl", spec.reference_auw_kg, payload_kg=spec.reference_payload_kg,
                              available_wh=0.9 * nominal, pack_nominal_wh=nominal)
        tmpl = UAVInstance("tmpl", spec, sensors, sortie)
    sensors = tuple(tmpl.sensors)
    if cls == "A" and "rgb" in kinds_fleet and not any(hw.kind == "rgb" for hw in sensors):
        cam = next((hw for u in inputs.fleet for hw in u.sensors if hw.kind == "rgb"), CameraHardware())
        sensors = sensors + (cam,)
    # 起飛站點沿用「原機」而非範本（`UAVInstance.start_site_id` 生效後，範本會把所有機拉回 home，
    # 同構基線就不再與異構任務可比）。
    fleet = tuple(replace(tmpl, uav_id=u.uav_id, sensors=sensors, sortie=replace(tmpl.sortie, uav_id=u.uav_id),
                          start_site_id=u.start_site_id)
                  for u in inputs.fleet)
    return replace(inputs, fleet=fleet)


def _run_makespan(inputs: MissionInputs, cfg: PlannerConfig, terrain_margin_m: float | None) -> dict[str, float]:
    t0 = time.perf_counter()
    kw = {} if terrain_margin_m is None else {"terrain_margin_m": float(terrain_margin_m)}
    try:
        mp = plan_mission(inputs, cfg, **kw)
    except Exception as exc:      # 任何失敗 → inf
        return {"makespan_s": math.inf, "distance_m": math.inf, "compute_ms": (time.perf_counter() - t0) * 1e3,
                "uncovered": math.inf, "error": 1.0, "error_msg": str(exc)}   # type: ignore[dict-item]
    complete = bool(mp.plans) and not mp.uncovered and mp.feasibility.status != "NO_GO"
    cov_min = math.nan
    if complete:
        # R-04：`mp.uncovered` 只反映「未被指派之條帶」，不反映「某一感測需求整套條帶從未鋪設」（both 區由單一
        # 同時掛載雙感測之構型服務時會整套消失）。基線比較必須用實際足跡覆蓋率把關，否則會拿 56 條帶的
        # 不完整任務去和 107 條帶的完整任務比 makespan。
        try:
            ctx_b = build_context(inputs, cfg, **kw)
            reps = coverage_raster(mp, ctx_b, cfg)
            cov_min = min((r.coverage for r in reps), default=1.0)
            complete = bool(reps) and all(r.passed for r in reps)
        except Exception:                                   # pragma: no cover - 覆蓋率無法計算 → 視為不完整
            complete = False
    return {"makespan_s": float(mp.makespan_s) if complete else math.inf,
            "coverage_min": float(cov_min),
            "distance_m": float(sum(p.distance_m for p in mp.plans.values())) if mp.plans else math.inf,
            "compute_ms": (time.perf_counter() - t0) * 1e3,
            "uncovered": float(len(mp.uncovered)), "error": 0.0, "n_plans": float(len(mp.plans)),
            "makespan_partial_s": float(mp.makespan_s)}


def baselines(inputs: MissionInputs, cfg: PlannerConfig, *, terrain_margin_m: float | None = None,
              variants: Sequence[str] = ("hetero", "all_A", "all_B", "legacy")) -> dict[str, float]:
    """規格 §17.1 (6)：異構 vs 同構基線（全 A／全 B，:func:`homogeneous_fleet`）vs `legacy_mode`（同構全 A 機隊 +
    論文退化設定）。回傳 ``{"hetero", "all_A", "all_B", "legacy"} → makespan_s``（失敗、NO_GO 或未完整覆蓋 → inf），
    另附 ``<name>_distance_m``、``<name>_compute_ms``、``<name>_uncovered``。"""
    out: dict[str, float] = {}
    for name in variants:
        if name == "hetero":
            inp, c = inputs, cfg
        elif name == "all_A":
            inp, c = homogeneous_fleet(inputs, "A"), cfg
        elif name == "all_B":
            inp, c = homogeneous_fleet(inputs, "B"), cfg
        elif name == "legacy":
            inp, c = homogeneous_fleet(inputs, "A"), replace(cfg, legacy_mode=True)
        else:
            raise ValueError(f"unknown baseline variant {name!r}")
        try:
            r = _run_makespan(inp, c, terrain_margin_m)
        except Exception:
            r = {"makespan_s": math.inf, "distance_m": math.inf, "compute_ms": 0.0, "uncovered": math.inf, "error": 1.0}
        out[name] = float(r["makespan_s"])
        for k in ("distance_m", "compute_ms", "uncovered", "coverage_min"):
            out[f"{name}_{k}"] = float(r.get(k, math.inf))
    return out


def paper_table2_check(cfg: PlannerConfig, *, d_d_m: float | None = 200.0, terrain_margin_m: float = 5000.0
                       ) -> dict[str, float]:
    """規格 §17.1 (7)：論文 Table 2 情境（`scenario.build_paper_table2_scenario`，3 機、V=230、R=4000、h=1200、平地）
    以 legacy 退化執行 `plan_mission`，回報路徑長（計畫 3D 折線長；3 機自 U1 起飛，論文各機自各自地面站起飛之差異記於
    CONTRACT_DEVIATIONS）。``cfg`` 之 legacy_mode 強制 True、dem_step_m ≥ 情境值（250）、d_d_m 預設放寬為 200 m（70 km 區域）。
    回傳鍵：``total_path_km``、``max_uav_path_km``、``mean_uav_path_km``、``path_km_<uav>``、``n_uav``、``makespan_s``、
    ``compute_ms``、``uncovered``、``expected_lo_km``／``expected_hi_km``（590／650）、``in_range_total``、``in_range_max_uav``。"""
    from .scenario import build_paper_table2_scenario
    inputs, cfg0 = build_paper_table2_scenario()
    kw: dict[str, Any] = {"legacy_mode": True, "dem_step_m": max(float(cfg.dem_step_m), float(cfg0.dem_step_m))}
    if d_d_m is not None:
        kw["d_d_m"] = max(float(cfg.d_d_m), float(d_d_m))
    cfg_used = replace(cfg, **kw)
    t0 = time.perf_counter()
    lo, hi = PAPER_PATH_RANGE_KM
    out: dict[str, float] = {"expected_lo_km": lo, "expected_hi_km": hi, "n_uav": float(len(inputs.fleet))}
    try:
        mp = plan_mission(inputs, cfg_used, terrain_margin_m=terrain_margin_m)
    except Exception:
        out.update({"total_path_km": math.inf, "max_uav_path_km": math.inf, "mean_uav_path_km": math.inf,
                    "makespan_s": math.inf, "uncovered": math.inf, "in_range_total": 0.0, "in_range_max_uav": 0.0,
                    "compute_ms": (time.perf_counter() - t0) * 1e3, "error": 1.0})
        return out
    lens = {u: float(p.distance_m) / 1000.0 for u, p in mp.plans.items()}
    total = sum(lens.values()) if lens else math.inf
    mx = max(lens.values()) if lens else math.inf
    mean = total / len(lens) if lens else math.inf
    out.update({"total_path_km": total, "max_uav_path_km": mx, "mean_uav_path_km": mean,
                "makespan_s": float(mp.makespan_s), "uncovered": float(len(mp.uncovered)),
                "in_range_total": 1.0 if lo <= total <= hi else 0.0,
                "in_range_max_uav": 1.0 if lo <= mx <= hi else 0.0,
                "compute_ms": (time.perf_counter() - t0) * 1e3, "error": 0.0,
                "status_ok": 0.0 if mp.feasibility.status == "NO_GO" else 1.0})
    for u, v in lens.items():
        out[f"path_km_{u}"] = v
    return out
