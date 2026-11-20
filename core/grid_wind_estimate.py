"""網格風重估 HARVEST 計畫的航時與能量（不改計畫）。純 Python（numpy，不用 Qt）。

規劃器（hv_dccpp）只吃**一層水平均勻**的風（``ctx.wind``；AeroPlan 轉接層固定建 0–3000 m 一層）。Open-Meteo 網格／
分層風（``core.wind_field.WindField``／``WindProfile``）原本只拿來顯示，或取向量平均／最接近航高那層寫進規劃風。
這個模組把**同一份計畫**（航點、條帶、順序、起降都不動）的固定翼航段，用網格風重算地速 → 時間 → 能量，
跟規劃器自己的估計並列（2026-10-21，docs/評估_2026-10-21_網格風與地形接入規劃器.md）。

做法（逐段、加法）：

* 只重算規劃器用「弧長 ÷ 地速」計時的階段（``FW_PHASES`` 去掉 ACCEL／DECEL，與 ``altitude.build_3d`` 同一條分支）；
  起飛、降落、轉態、加減速是固定常數，原樣保留。
* 每段（航點 a → b）沿線取樣，用規劃器同一個風三角（``hv_dccpp.wind.ground_speed``）算兩個模型時間：
  ``T_plan`` ＝ 規劃風（``ctx.wind.wind_vec(z)``，生效值），``T_grid`` ＝ 網格風；重估時間 ＝ 規劃器時間 + (T_grid − T_plan)。
  延遲盤旋、起飛延後這類「沒有幾何長度的時間」因此原樣保留（它們是等待時間，不是飛行距離）。
* 網格風取**沿線平均**：條帶（OPERATE）每一趟取整條帶的平均風向量，其他階段每一段連續同階段的航程取一個平均
  （使用者 2026-10-21 定的「每條帶取沿線平均風」）。高度依航點的**離地高**（``Waypoint.agl_m``）選層：
  分層風取最接近的一層（一樣近取低，同 ``profile_wind_at``，但那一層在該點沒有值就算「沒涵蓋」，不換別層）；
  單層網格套用到所有高度（會在 notes 寫明）。
* 能量：規劃器的能量模型沒有直接的風項，風只透過時間影響能量 ⇒ 每段用同一個 ``EnergyModel.segment_wh(CRUISE, dt, γ)``
  （γ 由該段高度差與時間決定，同 build_3d）重算，差值累加。

誠實原則（有效值契約）：

* 網格範圍外、缺值格 ⇒ 該段**沿用規劃風**（重估差 0）並計入 ``uncovered_m``；不外推、不靜默。
* 網格風下地速低於 ``cfg.v_g_min_mps`` 或側風 ≥ 空速 ⇒ 列入 problems（規劃器在這個風下會判那段不可行）。
* 自我檢查：``T_plan``（重建）對規劃器自己的時間差 ``residual``；重建比規劃器**長**超過容差 ⇒ problems
  （幾何重建錯了，重估不可信）。比規劃器短的部分是插入的等待（盤旋／延後），另外統計。
* 重估後的時間偏移對照規劃器 4D 防撞假設的時間不確定度 σ_t ＝ min(k_t·t, 45 s)：超過 ⇒ problems
  （機間時間隔離是用規劃風的時間算的，重估沒有重排降落佇列、沒有重做防撞）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from core.wind_field import WindField, WindProfile, _field_uv_at, uv_to_dir

#: 沿線取樣間距（m）：與規劃器 ``PlannerConfig.d_d_m`` 預設相同
DEFAULT_SAMPLE_M = 25.0
#: 重建時間比規劃器時間**長**超過這麼多（s）⇒ 該段幾何重建錯了（規劃器取樣是起點地速、這裡是中點，直線段完全一致、弧段差 ≪ 0.1 s）
RESIDUAL_TOL_S = 0.5
#: 重建時間比規劃器時間**短**超過這麼多（s）⇒ 視為插入的等待（延遲盤旋、起飛延後），不是誤差
WAIT_MIN_S = 1.0


class GridWindEstimateError(ValueError):
    """輸入不能重估（例如計畫或情境缺欄位）。訊息是給操作員看的繁中說明。"""


def _timed_phases() -> frozenset:
    from hv_dccpp.mission import FW_PHASES, Phase
    return frozenset(FW_PHASES) - {Phase.ACCEL, Phase.DECEL}


def _wrap_pi(a: float) -> float:
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def _mean_dir_text(u: float, v: float) -> str:
    d, s = uv_to_dir(u, v)
    return f"{s:.2f} m/s 來自 {d:.0f}°"


# ─────────────────────────────────────────────────────────────────────
#  網格風來源：(lat, lon, 離地高) → (u, v) 或 None
# ─────────────────────────────────────────────────────────────────────
class GridWindSource:
    """把 ``WindField``（單層網格或均勻場）或 ``WindProfile``（分層）包成取值器。u 東向、v 北向（空氣移動方向），
    與 hv_dccpp 的 ``wind_vec`` 同一個慣例（西風 270° ⇒ u = +s）。"""

    def __init__(self, src: Any) -> None:
        if isinstance(src, WindProfile):
            self.kind = "profile"
            self._layers = tuple(src.layers)
            self.label = src.label
            self.time_utc = src.time_utc
            self.model = src.model
            notes = list(src.notes)
        elif isinstance(src, WindField):
            self.kind = "grid"
            self._layers = (src,)
            self.label = src.label
            self.time_utc = src.time_utc
            self.model = src.source
            notes = list(src.notes)
            notes.append(f"單層 {src.height_m:g} m 網格套用到所有高度（沒有其他高度的資料；"
                         + ("10 m 風通常比作業高度的風弱" if src.height_m < 50 else "爬升、下降段的低空風可能不同") + "）")
        else:
            raise GridWindEstimateError(f"網格風要是 WindField 或 WindProfile，收到 {type(src).__name__}")
        self.heights_m = tuple(float(f.height_m) for f in self._layers)
        self.notes = tuple(notes)

    def layer_index(self, agl_m: float) -> int:
        """最接近的層（一樣近取低）；非有限數 ⇒ 最低層。"""
        if len(self._layers) == 1 or not math.isfinite(agl_m):
            return 0
        return min(range(len(self.heights_m)), key=lambda k: (abs(self.heights_m[k] - agl_m), self.heights_m[k]))

    def uv(self, lat: float, lon: float, agl_m: float) -> Optional[tuple]:
        """該點該高度的 (u, v)；範圍外、四角缺值 ⇒ None（不換層、不外推）。"""
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return None
        return _field_uv_at(self._layers[self.layer_index(agl_m)], float(lat), float(lon))


# ─────────────────────────────────────────────────────────────────────
#  結果
# ─────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RunEstimate:
    """一段連續同階段的航程（條帶一趟、一段轉移……）的重估。"""
    phase: str
    strip_id: Optional[str]
    dir: Optional[int]
    wp_from: int
    wp_to: int
    length_m: float
    dt_plan_s: float
    dt_grid_s: float
    de_wh: float
    covered: bool
    grid_uv: Optional[tuple]          # 沿線平均網格風 (u, v)；沒涵蓋 ⇒ None
    plan_uv: tuple                    # 沿線平均規劃風 (u, v)
    layer_heights_m: tuple            # 用到的網格層高度
    vg_min_grid_mps: float            # 網格風下沿線最低地速（沒涵蓋 ⇒ nan）

    def to_json(self) -> dict:
        def uvj(uv):
            if uv is None:
                return None
            d, s = uv_to_dir(*uv)
            return {"speed_mps": s, "dir_from_deg": d}
        return {"phase": self.phase, "strip_id": self.strip_id, "dir": self.dir, "wp_from": self.wp_from,
                "wp_to": self.wp_to, "length_m": self.length_m, "dt_plan_s": self.dt_plan_s,
                "dt_grid_s": self.dt_grid_s, "de_wh": self.de_wh, "covered": self.covered,
                "grid_wind": uvj(self.grid_uv), "plan_wind": uvj(self.plan_uv),
                "layer_heights_m": list(self.layer_heights_m),
                "vg_min_grid_mps": self.vg_min_grid_mps if math.isfinite(self.vg_min_grid_mps) else None}


@dataclass(frozen=True)
class UAVWindEstimate:
    uav_id: str
    cls: str
    t_takeoff_s: float
    t_land_plan_s: float
    t_land_grid_s: float
    energy_used_plan_wh: float
    energy_used_grid_wh: float
    margin_plan_wh: float
    margin_grid_wh: float
    timed_plan_s: float               # 重估範圍（有涵蓋的計時航段）在規劃器下的時間
    timed_grid_s: float
    timed_m: float                    # 計時航段總長
    uncovered_m: float                # 其中網格沒涵蓋（沿用規劃風）的長度
    wait_s: float                     # 插入的等待（延遲盤旋、起飛延後；原樣保留）
    residual_max_s: float             # 重建時間比規劃器長的最大值（> RESIDUAL_TOL_S ⇒ 重建錯）
    residual_abs_median_s: float
    max_shift_s: float                # 沿途 |重估 − 規劃| 時間偏移的最大值
    max_shift_over_sigma: float       # 上者對規劃器 σ_t 的倍數（> 1 ⇒ 超出 4D 防撞的時間不確定度）
    runs: tuple
    problems: tuple

    @property
    def flight_time_plan_s(self) -> float:
        return self.t_land_plan_s - self.t_takeoff_s

    @property
    def flight_time_grid_s(self) -> float:
        return self.t_land_grid_s - self.t_takeoff_s

    def to_json(self) -> dict:
        return {"uav_id": self.uav_id, "cls": self.cls, "t_takeoff_s": self.t_takeoff_s,
                "t_land_plan_s": self.t_land_plan_s, "t_land_grid_s": self.t_land_grid_s,
                "flight_time_plan_s": self.flight_time_plan_s, "flight_time_grid_s": self.flight_time_grid_s,
                "energy_used_plan_wh": self.energy_used_plan_wh, "energy_used_grid_wh": self.energy_used_grid_wh,
                "margin_plan_wh": self.margin_plan_wh, "margin_grid_wh": self.margin_grid_wh,
                "timed_plan_s": self.timed_plan_s, "timed_grid_s": self.timed_grid_s, "timed_m": self.timed_m,
                "uncovered_m": self.uncovered_m, "wait_s": self.wait_s, "residual_max_s": self.residual_max_s,
                "residual_abs_median_s": self.residual_abs_median_s, "max_shift_s": self.max_shift_s,
                "max_shift_over_sigma": self.max_shift_over_sigma,
                "runs": [r.to_json() for r in self.runs], "problems": list(self.problems)}


@dataclass(frozen=True)
class PlanWindEstimate:
    source_kind: str                  # 'grid' | 'profile'
    source_label: str
    source_time_utc: Optional[str]
    source_heights_m: tuple
    plan_wind_text: str               # 規劃器實際用的風（ctx.wind 生效值）
    makespan_plan_s: float
    makespan_grid_s: float
    uavs: Mapping[str, UAVWindEstimate]
    fingerprint: tuple                # 對應的計畫（plan_fingerprint）；GUI 用來判斷重估是否過時
    notes: tuple = field(default_factory=tuple)

    @property
    def problems(self) -> list:
        return [f"{u}: {p}" for u, e in self.uavs.items() for p in e.problems]

    def summary_lines(self) -> list:
        """給操作員看的幾行（繁中）：來源、規劃風、每機航時／能量前後、問題。"""
        out = [f"網格風重估（計畫不變，只重算固定翼航段的時間與能量）：{self.source_label}",
               f"規劃器用的風：{self.plan_wind_text}"]
        d = self.makespan_grid_s - self.makespan_plan_s
        pct = 100.0 * d / self.makespan_plan_s if self.makespan_plan_s > 0 else float("nan")
        out.append(f"總任務時間 {self.makespan_plan_s:.0f} s → {self.makespan_grid_s:.0f} s（{d:+.1f} s，{pct:+.2f}%；"
                   "未重排降落佇列）")
        for u, e in self.uavs.items():
            dt = e.flight_time_grid_s - e.flight_time_plan_s
            unc = f"，{e.uncovered_m / 1000:.1f} km 不在網格內（沿用規劃風）" if e.uncovered_m > 0 else ""
            out.append(f"{u}：航時 {e.flight_time_plan_s:.0f} → {e.flight_time_grid_s:.0f} s（{dt:+.1f} s）；"
                       f"能量餘裕 {e.margin_plan_wh:.1f} → {e.margin_grid_wh:.1f} Wh{unc}")
        out.extend(f"⚠ {p}" for p in self.problems)
        return out

    def to_json(self) -> dict:
        return {"source_kind": self.source_kind, "source_label": self.source_label,
                "source_time_utc": self.source_time_utc, "source_heights_m": list(self.source_heights_m),
                "plan_wind_text": self.plan_wind_text, "makespan_plan_s": self.makespan_plan_s,
                "makespan_grid_s": self.makespan_grid_s, "uavs": {u: e.to_json() for u, e in self.uavs.items()},
                "fingerprint": [list(x) for x in self.fingerprint], "notes": list(self.notes),
                "problems": self.problems}


def plan_fingerprint(plan: Any) -> tuple:
    """計畫的指紋 ((uav_id, 航點數, 落地時刻), …)：重估結果記這個，計畫換了就看得出來。"""
    return tuple(sorted((str(u), len(p.waypoints), round(float(p.t_land_s), 3)) for u, p in plan.plans.items()))


def estimate_matches(est: PlanWindEstimate, plan: Any) -> bool:
    return est.fingerprint == plan_fingerprint(plan)


def plan_wind_text(wind: Any) -> str:
    """``ctx.wind``（hv_dccpp WindField）各層 → 文字；沒有層 ⇒ 無風。"""
    if wind is None or not getattr(wind, "n_layers", 0):
        return "無風（沒有風層）"
    parts = []
    for k in range(wind.n_layers):
        zb, zt = (float(x) for x in wind.z_bounds_enu_m[k])
        parts.append(f"{float(wind.speeds_mps[k]):.2f} m/s 來自 {math.degrees(float(wind.dir_from_rad[k])) % 360:.0f}°"
                     f"（ENU z {zb:g}–{zt:g} m，全區均勻）")
    return "；".join(parts)


# ─────────────────────────────────────────────────────────────────────
#  重估
# ─────────────────────────────────────────────────────────────────────
def _segment_samples(a: Any, b: Any, ds_m: float) -> tuple:
    """航點 a → b 的取樣（中點法）：(x, y, z, agl, θ, ds) 各 (n,)，以及總長。

    直線：θ 固定；弧（``a.arc`` 有值，弧參數是 a 出發那一段的）：長度 R·|Δθ|、θ 線性變化（規劃器的弧段在縮點時
    幾乎每個取樣點都保留，所以一段弧的 Δθ 很小，取最短的帶號角差）。位置沿弦線性內插（只用來查風，幾公尺誤差對
    13 km 的模式格點沒有影響）。"""
    xa, ya, xb, yb = float(a.x_m), float(a.y_m), float(b.x_m), float(b.y_m)
    dx, dy = xb - xa, yb - ya
    chord = math.hypot(dx, dy)
    if a.arc is not None:
        th_a, th_b = math.radians(float(a.track_deg)), math.radians(float(b.track_deg))
        dth = _wrap_pi(th_b - th_a)
        length = float(a.arc[2]) * abs(dth)
        if length <= 0.0 and chord > 0.0:          # 弧參數與航跡角不一致（不該發生）⇒ 當直線
            length, dth, th_a = chord, 0.0, math.atan2(dx, dy)
    else:
        length = chord
        th_a = math.atan2(dx, dy) if chord > 0 else math.radians(float(a.track_deg))
        dth = 0.0
    if length <= 0.0:
        z = np.zeros(0)
        return z, z, z, z, z, z, 0.0
    n = max(1, int(math.ceil(length / ds_m - 1e-9)))
    f = (np.arange(n, dtype=np.float64) + 0.5) / n
    x = xa + f * dx
    y = ya + f * dy
    z = float(a.z_m) + f * (float(b.z_m) - float(a.z_m))
    agl = float(a.agl_m) + f * (float(b.agl_m) - float(a.agl_m))
    th = th_a + f * dth
    ds = np.full(n, length / n)
    return x, y, z, agl, th, ds, length


def _times(th: np.ndarray, ds: np.ndarray, va: float, wxy: np.ndarray, vg_min: float) -> tuple:
    """Σ ds / max(V_g, V_g,min)（規劃器 build_3d 同一條：不可行或低於下限時以下限計時）＋ 最低地速。"""
    from hv_dccpp.wind import ground_speed
    if th.size == 0:
        return 0.0, math.inf
    vg = ground_speed(th, float(va), wxy)
    vg_c = np.where(np.isfinite(vg) & (vg >= vg_min), vg, vg_min)
    vmin = float(np.nanmin(np.where(np.isfinite(vg), vg, -math.inf)))
    return float(np.sum(ds / vg_c)), vmin


def _cruise_wh(model: Any, va_c: float, dt: float, dz: float) -> float:
    """build_3d 的巡航段能量：segment_wh(CRUISE, dt, asin(dz/(Va·dt)))。"""
    from hv_dccpp.mission import Mode
    if dt <= 0.0:
        return 0.0
    gamma = math.asin(max(-1.0, min(1.0, dz / (va_c * dt))))
    return float(model.segment_wh(Mode.CRUISE, dt, gamma))


def _sigma_t(t: float, cfg: Any) -> float:
    from hv_dccpp.deconflict import sigma_t_max_of
    return min(float(getattr(cfg, "k_t", 0.05)) * max(0.0, float(t)), float(sigma_t_max_of(cfg)))


def estimate_uav(plan: Any, ctx: Any, source: GridWindSource, *, sample_m: float = DEFAULT_SAMPLE_M) -> UAVWindEstimate:
    """單機重估（見模組說明）。``plan`` 是 hv_dccpp ``UAVPlan``；``ctx`` 是同一次規劃的 ``MissionContext``（生效值）。"""
    timed = _timed_phases()
    cfg = ctx.cfg
    spec = ctx.specs[plan.cls]
    model = ctx.models.get(plan.uav_id) or ctx.models_rep[plan.cls]
    va_c = float(spec.cruise_airspeed_mps)
    vg_min = float(cfg.v_g_min_mps)
    wps = list(plan.waypoints)
    problems: list[str] = []

    # ── 1) 每段取樣（計時階段才取）──────────────────────────────────────
    segs = []                                        # (j, x, y, z, agl, th, ds, length)
    for j in range(len(wps) - 1):
        a, b = wps[j], wps[j + 1]
        if a.phase not in timed or not (float(a.airspeed_mps) > 0.0):
            continue
        x, y, z, agl, th, ds, length = _segment_samples(a, b, float(sample_m))
        segs.append((j, x, y, z, agl, th, ds, length))

    # ── 2) 一次把所有取樣點轉經緯度、查兩種風 ─────────────────────────────
    if segs:
        X = np.concatenate([s[1] for s in segs])
        Y = np.concatenate([s[2] for s in segs])
        Z = np.concatenate([s[3] for s in segs])
        AGL = np.concatenate([s[4] for s in segs])
    else:
        X = Y = Z = AGL = np.zeros(0)
    if X.size:
        wgs = ctx.frame.to_wgs84(np.column_stack([X, Y, Z]))
        w_plan = (np.asarray(ctx.wind.wind_vec(Z), dtype=np.float64).reshape(-1, 2) if ctx.wind is not None
                  else np.zeros((X.size, 2)))
    else:
        wgs = np.zeros((0, 3))
        w_plan = np.zeros((0, 2))
    w_grid = np.full((X.size, 2), np.nan)
    layer_h = np.full(X.size, np.nan)
    for i in range(X.size):
        uv = source.uv(float(wgs[i, 0]), float(wgs[i, 1]), float(AGL[i]))
        layer_h[i] = source.heights_m[source.layer_index(float(AGL[i]))]
        if uv is not None:
            w_grid[i] = uv

    # ── 3) 分組：條帶一趟 ＝ 同 strip_id／dir 的連續 OPERATE 段；其他 ＝ 同階段的連續段 ───────────
    groups: list[list[int]] = []                     # segs 的索引
    keys: list[tuple] = []
    prev_j = None
    for k, s in enumerate(segs):
        a = wps[s[0]]
        key = (a.phase, a.strip_id if a.phase.name == "OPERATE" else None,
               a.dir if a.phase.name == "OPERATE" else None)
        if groups and keys[-1] == key and prev_j is not None and s[0] == prev_j + 1:
            groups[-1].append(k)
        else:
            groups.append([k])
            keys.append(key)
        prev_j = s[0]

    offs = np.concatenate([[0], np.cumsum([s[1].size for s in segs])]).astype(int) if segs else np.zeros(1, int)
    seg_delta = {}                                   # 航點 j → (Δt, ΔE)
    runs: list[RunEstimate] = []
    residuals: list[float] = []
    wait_s = 0.0
    timed_m = uncovered_m = timed_plan = timed_grid = 0.0
    for g, key in zip(groups, keys):
        sl = np.concatenate([np.arange(offs[k], offs[k + 1]) for k in g]) if g else np.zeros(0, int)
        ds_all = np.concatenate([segs[k][6] for k in g])
        L = float(np.sum(ds_all))
        covered = bool(sl.size) and bool(np.all(np.isfinite(w_grid[sl])))
        wbar = (np.sum(w_grid[sl] * ds_all[:, None], axis=0) / L) if (covered and L > 0) else None
        pbar = (np.sum(w_plan[sl] * ds_all[:, None], axis=0) / L) if L > 0 else np.zeros(2)
        dtp_run = dtg_run = de_run = 0.0
        vmin_run = math.inf
        for k in g:
            j, _x, _y, _z, _agl, th, ds, length = segs[k]
            a, b = wps[j], wps[j + 1]
            va = float(a.airspeed_mps)
            o0, o1 = offs[k], offs[k + 1]
            t_pm, _ = _times(th, ds, va, w_plan[o0:o1], vg_min)
            dt_p = float(b.t_s) - float(a.t_s)
            res = t_pm - dt_p
            if res < -WAIT_MIN_S:
                wait_s += -res                       # 插入的等待（盤旋／延後）
            else:
                residuals.append(res)
            if wbar is not None:
                t_gm, vmin = _times(th, ds, va, wbar, vg_min)
                vmin_run = min(vmin_run, vmin)
                dt_g = dt_p + (t_gm - t_pm)
            else:
                dt_g = dt_p
            dz = float(b.z_m) - float(a.z_m)
            de = _cruise_wh(model, va_c, dt_g, dz) - _cruise_wh(model, va_c, dt_p, dz)
            seg_delta[j] = (dt_g - dt_p, de)
            dtp_run += dt_p
            dtg_run += dt_g
            de_run += de
        timed_m += L
        if covered:
            timed_plan += dtp_run
            timed_grid += dtg_run
        else:
            uncovered_m += L
        a0 = wps[segs[g[0]][0]]
        hs = tuple(sorted({float(h) for h in layer_h[sl] if math.isfinite(h)}))
        runs.append(RunEstimate(phase=a0.phase.name, strip_id=key[1], dir=key[2], wp_from=segs[g[0]][0],
                                wp_to=segs[g[-1]][0] + 1, length_m=L, dt_plan_s=dtp_run, dt_grid_s=dtg_run,
                                de_wh=de_run, covered=covered,
                                grid_uv=None if wbar is None else (float(wbar[0]), float(wbar[1])),
                                plan_uv=(float(pbar[0]), float(pbar[1])), layer_heights_m=hs,
                                vg_min_grid_mps=vmin_run if math.isfinite(vmin_run) else math.nan))
        if wbar is not None and vmin_run < vg_min:
            what = f"條帶 {key[1]}" if key[1] is not None else a0.phase.name
            problems.append(f"網格風下 {what}（航點 {segs[g[0]][0]}）地速最低 {vmin_run:.1f} m/s < 下限 {vg_min:g} m/s"
                            f"（沿線平均 {_mean_dir_text(*wbar)}）：規劃器在這個風下會判這段不可行")

    # ── 4) 沿途累加：時間偏移、能量、σ_t 對照 ──────────────────────────────
    shift = 0.0
    e_shift = 0.0
    max_shift = 0.0
    max_ratio = 0.0
    for j in range(len(wps) - 1):
        d = seg_delta.get(j)
        if d is not None:
            shift += d[0]
            e_shift += d[1]
        t_here = float(wps[j + 1].t_s) + shift
        if abs(shift) > max_shift:
            max_shift = abs(shift)
        sig = _sigma_t(t_here, cfg)
        if sig > 0:
            max_ratio = max(max_ratio, abs(shift) / sig)
        elif abs(shift) > 1e-9:
            max_ratio = math.inf

    res_arr = np.asarray(residuals, dtype=np.float64)
    res_max = float(res_arr.max()) if res_arr.size else 0.0
    res_med = float(np.median(np.abs(res_arr))) if res_arr.size else 0.0
    if res_max > RESIDUAL_TOL_S:
        problems.append(f"重建的航段時間比規劃器長 {res_max:.2f} s（容差 {RESIDUAL_TOL_S:g} s）：幾何重建與規劃器不一致，"
                        "這份重估不可信")
    margin_grid = float(plan.energy_margin_wh) - e_shift
    if margin_grid < 0.0 <= float(plan.energy_margin_wh):
        problems.append(f"網格風下能量餘裕變負：{float(plan.energy_margin_wh):.1f} → {margin_grid:.1f} Wh"
                        "（已低於保留量；規劃器用的風下是夠的）")
    if max_ratio > 1.0:
        problems.append(f"時間偏移最大 {max_shift:.1f} s，超過規劃器 4D 防撞假設的時間不確定度 σ_t（{max_ratio:.1f} 倍）："
                        "機間時間隔離是用規劃風的時間算的，網格風下不保證")
    return UAVWindEstimate(
        uav_id=str(plan.uav_id), cls=str(plan.cls), t_takeoff_s=float(plan.t_takeoff_s),
        t_land_plan_s=float(plan.t_land_s), t_land_grid_s=float(plan.t_land_s) + shift,
        energy_used_plan_wh=float(plan.energy_used_wh), energy_used_grid_wh=float(plan.energy_used_wh) + e_shift,
        margin_plan_wh=float(plan.energy_margin_wh), margin_grid_wh=margin_grid,
        timed_plan_s=timed_plan, timed_grid_s=timed_grid, timed_m=timed_m, uncovered_m=uncovered_m,
        wait_s=wait_s, residual_max_s=res_max, residual_abs_median_s=res_med, max_shift_s=max_shift,
        max_shift_over_sigma=max_ratio, runs=tuple(runs), problems=tuple(problems))


def estimate_plan(plan: Any, ctx: Any, wind_source: Any, *, sample_m: float = DEFAULT_SAMPLE_M) -> PlanWindEstimate:
    """整份 ``MissionPlan`` 用網格風重估。``wind_source``：``core.wind_field`` 的 ``WindField``／``WindProfile``
    （或已包好的 ``GridWindSource``）。計畫本身不改。"""
    if plan is None or ctx is None:
        raise GridWindEstimateError("沒有計畫或規劃情境（要先完成一次 HARVEST 規劃）")
    if not getattr(plan, "plans", None):
        raise GridWindEstimateError("計畫裡沒有任何一架的航線")
    if not (math.isfinite(float(sample_m)) and float(sample_m) > 0):
        raise GridWindEstimateError(f"取樣間距要 > 0，收到 {sample_m!r}")
    src = wind_source if isinstance(wind_source, GridWindSource) else GridWindSource(wind_source)
    uavs = {str(u): estimate_uav(p, ctx, src, sample_m=float(sample_m)) for u, p in plan.plans.items()}
    # makespan ＝ 各機落地時刻最大值（planner.plan_mission 同一條）
    mk_plan = max(e.t_land_plan_s for e in uavs.values())
    mk_grid = max(e.t_land_grid_s for e in uavs.values())
    notes = list(src.notes)
    if abs(mk_plan - float(plan.makespan_s)) > 1e-6:
        notes.append(f"計畫的 makespan {float(plan.makespan_s):.1f} s 不等於各機落地時刻最大值 {mk_plan:.1f} s；"
                     "這裡用後者")
    return PlanWindEstimate(source_kind=src.kind, source_label=src.label, source_time_utc=src.time_utc,
                            source_heights_m=src.heights_m, plan_wind_text=plan_wind_text(getattr(ctx, "wind", None)),
                            makespan_plan_s=mk_plan, makespan_grid_s=mk_grid, uavs=uavs,
                            fingerprint=plan_fingerprint(plan), notes=tuple(notes))
