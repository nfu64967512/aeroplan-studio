"""hv_dccpp.wind — 風場、風三角、:math:`R_{eff}`、地面坡度、風中時間（契約 §4；規格 §3.2、D1、D14）。

**約定（契約 §0）**

* 航跡方位 ``theta`` 自北順時針（rad）：:math:`\\hat t=(\\sin\\theta,\\cos\\theta)`、右法向
  :math:`\\hat n=(\\cos\\theta,-\\sin\\theta)`。
* ``dir_from_deg`` 為氣象「來向」β；風向量（吹向）:math:`\\vec w=-W(\\sin\\beta,\\cos\\beta)`。
* 高度一律 ENU ``z``（D42）：``WindLayer`` 之 MSL／AGL 層界於建構時以 home 位置換算為 ENU z。
* 風三角（規格 §3.2）：:math:`W_\\parallel=\\vec w\\cdot\\hat t`、:math:`W_\\perp=\\vec w\\cdot\\hat n`、
  :math:`V_g=\\sqrt{V_a^2-W_\\perp^2}+W_\\parallel`、:math:`\\psi=\\theta-\\arcsin(W_\\perp/V_a)`；
  :math:`|W_\\perp|\\ge V_a` → ``nan``（呼叫端視為不可行；``cfg.v_g_min_mps`` 下限亦由呼叫端套用）。
* 弧段時間（D14）：沿弧以 :math:`V_g(\\theta)` 中點 Riemann 和數值積分（Δθ ≤ 5°），
  與 :func:`hv_dccpp.dubins.path_time` 之取點規則完全一致。

本模組為第 1 層：只依賴 ``platform``（``WindLayer`` 定義於 ``mission.py``，此處以鴨子型別接受任何
具 ``z_bottom_m, z_top_m, speed_mps, dir_from_deg, gust_mps, z_ref`` 屬性之物件）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal, Sequence

import numpy as np
from numpy.typing import ArrayLike
from scipy.special import ellipe

from .platform import MissionInputError, PlatformSpec

__all__ = [
    "WindField",
    "wind_vector",
    "ground_speed",
    "heading_from_track",
    "r_eff",
    "r_eff_wind_speed",
    "tan_gamma_ground",
    "worst_case_v_g",
    "time_line",
    "time_arc",
    "circle_time_factor",
    "proxy_turn_time",
    "DEFAULT_ARC_STEP_RAD",
]

TWO_PI: float = 2.0 * math.pi
DEFAULT_ARC_STEP_RAD: float = 0.0873          # 5°（規格 §8.1、D14）
_CURVATURE_NOTE = ("wind layer bounds converted MSL->ENU z at home only; the curvature term "
                   "(~196 m at 50 km) is not applied per position (known approximation)")


# ---------------------------------------------------------------------------
# 風向量
# ---------------------------------------------------------------------------
def wind_vector(speed_mps: ArrayLike, dir_from_deg: ArrayLike) -> np.ndarray:
    """氣象來向 β（deg）與風速 W → 吹向向量 :math:`\\vec w=-W(\\sin\\beta,\\cos\\beta)`，回傳 ``(N, 2)``。

    例：from 270°（西風）→ (+W, 0) 吹向東；from 0°（北風）→ (0, −W) 吹向南。
    """
    w = np.atleast_1d(np.asarray(speed_mps, dtype=np.float64))
    b = np.radians(np.atleast_1d(np.asarray(dir_from_deg, dtype=np.float64)))
    w, b = np.broadcast_arrays(w, b)
    return np.column_stack([-w * np.sin(b), -w * np.cos(b)]).reshape(-1, 2)


# ---------------------------------------------------------------------------
# WindField
# ---------------------------------------------------------------------------
def _parse_layer(i: int, lay: Any, frame: Any, z_ground_home_m: float,
                 notes: list[str]) -> tuple[float, float, float, float, float]:
    """單一 ``WindLayer``（鴨子型別）→ ``(zb_enu, zt_enu, speed, gust, dir_from_rad)``。"""
    try:
        zb = float(getattr(lay, "z_bottom_m"))
        zt = float(getattr(lay, "z_top_m"))
        spd = float(getattr(lay, "speed_mps"))
        d_deg = float(getattr(lay, "dir_from_deg"))
    except (AttributeError, TypeError, ValueError) as e:
        raise MissionInputError(f"wind layer[{i}]: 缺少或無法解析 z_bottom_m/z_top_m/speed_mps/dir_from_deg ({e})") from e
    gust_raw = getattr(lay, "gust_mps", 0.0)
    gust = 0.0 if gust_raw is None else float(gust_raw)
    z_ref = str(getattr(lay, "z_ref", "msl") or "msl").lower()

    if math.isnan(zb) or math.isnan(zt) or not (zt > zb):
        raise MissionInputError(f"wind layer[{i}]: 須 z_top_m > z_bottom_m（得 {zb}, {zt}）")
    if not math.isfinite(spd) or spd < 0.0:
        raise MissionInputError(f"wind layer[{i}]: speed_mps 須為 ≥ 0 之有限值（得 {spd}）")
    if not math.isfinite(gust) or gust < 0.0:
        raise MissionInputError(f"wind layer[{i}]: gust_mps 須為 ≥ 0 之有限值（得 {gust}）")
    if not math.isfinite(d_deg):
        raise MissionInputError(f"wind layer[{i}]: dir_from_deg 須為有限值（得 {d_deg}）")

    if z_ref == "msl":
        if frame is None:
            raise MissionInputError(f"wind layer[{i}]: z_ref='msl' 需要 GeoFrame 以換算 ENU z")
        home_xy = np.zeros((1, 2))

        def conv(v: float) -> float:
            if not math.isfinite(v):
                return v                                   # ±inf 層界原樣保留
            return float(frame.msl_to_enu_z(home_xy, v)[0])

        zb_e, zt_e = conv(zb), conv(zt)
        notes.append(f"wind_layer[{i}]: msl [{zb:g},{zt:g}] -> ENU z [{zb_e:.1f},{zt_e:.1f}] at home")
    elif z_ref == "agl":
        zg = float(z_ground_home_m)
        zb_e = zb if not math.isfinite(zb) else zg + zb
        zt_e = zt if not math.isfinite(zt) else zg + zt
        notes.append(f"wind_layer[{i}]: agl [{zb:g},{zt:g}] -> ENU z [{zb_e:.1f},{zt_e:.1f}] "
                     f"using home ground z={zg:.1f} (assumption: AGL referenced to home ground)")
    else:
        raise MissionInputError(f"wind layer[{i}]: z_ref 須為 'msl' 或 'agl'（得 {z_ref!r}）")
    return zb_e, zt_e, spd, gust, math.radians(d_deg)


@dataclass(slots=True, init=False)
class WindField:
    """高度分層風場（契約 §4；規格 §3.2）。

    * ``layers``：原始輸入（依 ENU z 排序：先 z_bottom、再 z_top）。
    * ``z_bounds_enu_m``：``(L, 2)`` 各層 ``[z_bottom, z_top]`` 之 ENU z（建構時於 home 位置以
      ``frame.msl_to_enu_z`` 換算；``z_ref="agl"`` 層以 ``z_ground_home_m + agl`` 換算）。
    * 層選取規則：**重疊取較低層；範圍外（含層間縫隙）取最近層**——以「z 至各層區間之距離」
      取 argmin 實作（距離 0 者為包含層；並列時取排序在前＝較低者）。
    * 無任何層 → 全域無風（``notes`` 記錄 ``wind_layers_empty_assumed_calm``）。
    * ``version``：風更新序號（``CostCache.wind_key`` 用）。
    """

    layers: list[Any]
    z_bounds_enu_m: np.ndarray
    version: int
    notes: list[str]
    z_ground_home_m: float
    speeds_mps: np.ndarray            # (L,)
    gusts_mps: np.ndarray             # (L,)
    dir_from_rad: np.ndarray          # (L,)

    def __init__(self, layers: Sequence[Any], frame: Any, z_ground_home_m: float,
                 version: int = 0) -> None:
        notes: list[str] = []
        parsed = [_parse_layer(i, lay, frame, z_ground_home_m, notes) for i, lay in enumerate(layers)]
        n = len(parsed)
        if n == 0:
            notes.append("wind_layers_empty_assumed_calm")
            order = np.zeros(0, dtype=int)
        else:
            zb = np.array([p[0] for p in parsed], dtype=np.float64)
            zt = np.array([p[1] for p in parsed], dtype=np.float64)
            order = np.lexsort((zt, zb))                   # 主鍵 z_bottom、次鍵 z_top
            notes.append(_CURVATURE_NOTE)
        self.layers = [layers[int(k)] for k in order]
        self.z_bounds_enu_m = (np.array([[parsed[int(k)][0], parsed[int(k)][1]] for k in order], dtype=np.float64)
                               .reshape(-1, 2))
        self.speeds_mps = np.array([parsed[int(k)][2] for k in order], dtype=np.float64)
        self.gusts_mps = np.array([parsed[int(k)][3] for k in order], dtype=np.float64)
        self.dir_from_rad = np.array([parsed[int(k)][4] for k in order], dtype=np.float64)
        self.version = int(version)
        self.notes = notes
        self.z_ground_home_m = float(z_ground_home_m)

    # ------------------------------------------------------------------ 層選取
    @property
    def n_layers(self) -> int:
        return int(self.speeds_mps.shape[0])

    def layer_index(self, z_m: ArrayLike) -> np.ndarray:
        """各 z（ENU）所用之層索引 ``(N,)`` int；無層時全為 −1；nan z → −1。"""
        z = np.atleast_1d(np.asarray(z_m, dtype=np.float64)).reshape(-1)
        if self.n_layers == 0:
            return np.full(z.shape, -1, dtype=int)
        zb = self.z_bounds_enu_m[:, 0][None, :]
        zt = self.z_bounds_enu_m[:, 1][None, :]
        zc = z[:, None]
        with np.errstate(invalid="ignore"):
            dist = np.maximum(np.maximum(zb - zc, zc - zt), 0.0)     # 至區間之距離（含層 → 0）
        idx = np.argmin(dist, axis=1)                                 # 並列 → 較低層
        idx = np.where(np.isnan(z), -1, idx)
        return idx.astype(int)

    def _per_z(self, arr: np.ndarray, z_m: ArrayLike, fill: float = 0.0) -> np.ndarray:
        idx = self.layer_index(z_m)
        out = np.full(idx.shape, fill, dtype=np.float64)
        ok = idx >= 0
        if ok.any():
            out[ok] = arr[idx[ok]]
        out[np.isnan(np.atleast_1d(np.asarray(z_m, dtype=np.float64)).reshape(-1))] = np.nan
        return out

    # ------------------------------------------------------------------ 查詢
    def wind_vec(self, z_m: ArrayLike) -> np.ndarray:
        """吹向風向量 ``(N, 2)`` ``(wx, wy)``；``z_m`` 為 ENU z（純量 → ``(1, 2)``）。"""
        spd = self._per_z(self.speeds_mps, z_m)
        beta = self._per_z(self.dir_from_rad, z_m)
        return np.column_stack([-spd * np.sin(beta), -spd * np.cos(beta)]).reshape(-1, 2)

    def speeds_at(self, z_m: ArrayLike) -> np.ndarray:
        """向量化風速 ``(N,)``。"""
        return self._per_z(self.speeds_mps, z_m)

    def speed_at(self, z_m: float) -> float:
        """ENU z 處之風速（m/s）。"""
        return float(self._per_z(self.speeds_mps, z_m)[0])

    def gust_at(self, z_m: float) -> float:
        """ENU z 處之陣風（m/s；層未給則 0）。"""
        return float(self._per_z(self.gusts_mps, z_m)[0])

    def dir_from_at(self, z_m: float) -> float:
        """ENU z 處之風來向 β（rad，自北順時針）。"""
        return float(self._per_z(self.dir_from_rad, z_m)[0])

    def max_speed(self) -> float:
        """所有層之最大風速（無層 → 0）。"""
        return float(self.speeds_mps.max()) if self.n_layers else 0.0

    def max_speed_between(self, z_lo_m: float, z_hi_m: float) -> float:
        """:math:`\\max\\{W(z): z\\in[z_{lo},z_{hi}]\\}`（規格 §3.2 :math:`W_{R,c}` 政策用）。

        與 :meth:`speed_at` 完全一致：層選取為分段常數，切換點只可能在層界與「相鄰層界之中點」
        （縫隙內最近層切換處），故於 ``{z_lo, z_hi} ∪ 切換點`` 及其相鄰中點取樣後取最大即為精確值。
        ``z_lo > z_hi`` 時自動對調；無層 → 0。
        """
        if self.n_layers == 0:
            return 0.0
        lo, hi = float(z_lo_m), float(z_hi_m)
        if math.isnan(lo) or math.isnan(hi):
            raise ValueError("max_speed_between: z bounds must not be nan")
        if lo > hi:
            lo, hi = hi, lo
        zb = self.z_bounds_enu_m[:, 0]
        zt = self.z_bounds_enu_m[:, 1]
        finite_b = zb[np.isfinite(zb)]
        finite_t = zt[np.isfinite(zt)]
        # 有限的掃描範圍（±inf 以層界外 1 m 代替；最近層在該處已定）
        z_min_all = float(min(finite_b.min() if finite_b.size else 0.0, finite_t.min() if finite_t.size else 0.0)) - 1.0
        z_max_all = float(max(finite_b.max() if finite_b.size else 0.0, finite_t.max() if finite_t.size else 0.0)) + 1.0
        lo_c = max(lo, z_min_all) if not math.isfinite(lo) else lo
        hi_c = min(hi, z_max_all) if not math.isfinite(hi) else hi
        if lo_c > hi_c:                                   # 兩端皆在同側無窮遠：退回單點
            lo_c = hi_c = min(max(lo, z_min_all), z_max_all)
        pts = [lo_c, hi_c]
        pts.extend(float(v) for v in finite_b if lo_c < v < hi_c)
        pts.extend(float(v) for v in finite_t if lo_c < v < hi_c)
        # 縫隙內最近層切換點：所有 (top_i, bottom_j) 對之中點（top_i < bottom_j）
        if finite_t.size and finite_b.size:
            mids = 0.5 * (finite_t[:, None] + finite_b[None, :])
            mask = finite_t[:, None] < finite_b[None, :]
            for v in mids[mask].ravel():
                if lo_c < v < hi_c:
                    pts.append(float(v))
        s = np.unique(np.asarray(pts, dtype=np.float64))
        if s.size > 1:
            s = np.concatenate([s, 0.5 * (s[:-1] + s[1:])])
        return float(self.speeds_at(s).max())


# ---------------------------------------------------------------------------
# 風三角
# ---------------------------------------------------------------------------
def _components(theta_rad: ArrayLike, wind_xy: ArrayLike) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """回傳 ``(theta (N,), W_par (N,), W_perp (N,))``；``wind_xy`` 為 (2,) 廣播或 (N,2) 逐點。"""
    th = np.atleast_1d(np.asarray(theta_rad, dtype=np.float64))
    w = np.asarray(wind_xy, dtype=np.float64)
    if w.ndim == 1:
        if w.size != 2:
            raise ValueError("wind_xy must be (2,) or (N,2)")
        wx, wy = w[0], w[1]
    elif w.ndim == 2 and w.shape[1] == 2:
        wx, wy = w[:, 0], w[:, 1]
    else:
        raise ValueError("wind_xy must be (2,) or (N,2)")
    st, ct = np.sin(th), np.cos(th)
    w_par = wx * st + wy * ct                      # w · t̂
    w_perp = wx * ct - wy * st                     # w · n̂（右法向）
    th_b = np.broadcast_to(th, w_par.shape)
    return th_b, w_par, w_perp


def ground_speed(theta_rad: ArrayLike, v_air_mps: float, wind_xy: ArrayLike) -> np.ndarray:
    """風三角地速（規格 §3.2）：:math:`V_g=\\sqrt{V_a^2-W_\\perp^2}+W_\\parallel`，回傳 ``(N,)``。

    ``theta_rad`` 為航跡方位（純量或 (N,)）；``wind_xy`` 吹向向量 (2,) 廣播或 (N,2) 逐點。
    :math:`|W_\\perp|\\ge V_a` → ``nan``；:math:`V_g<` ``cfg.v_g_min_mps`` 由呼叫端視為不可行。
    逆風（θ = β）:math:`V_g=V_a-W`、順風（θ = β+π）:math:`V_a+W`、正側風 :math:`\\sqrt{V_a^2-W^2}`。
    """
    if not v_air_mps > 0.0:
        raise ValueError("v_air_mps must be > 0")
    _, w_par, w_perp = _components(theta_rad, wind_xy)
    rad = v_air_mps * v_air_mps - w_perp * w_perp
    with np.errstate(invalid="ignore"):
        out = np.where(rad > 0.0, np.sqrt(np.where(rad > 0.0, rad, 0.0)) + w_par, np.nan)
    return np.atleast_1d(out)


def heading_from_track(theta_rad: ArrayLike, v_air_mps: float, wind_xy: ArrayLike) -> np.ndarray:
    """航向 :math:`\\psi=\\theta-\\arcsin(W_\\perp/V_a)`（規格 §3.2），形狀規則同 :func:`ground_speed`。

    風向右（:math:`W_\\perp>0`）時機頭須向左偏（ψ < θ）。:math:`|W_\\perp|\\ge V_a` → ``nan``。
    恆等式：:math:`V_a(\\sin\\psi,\\cos\\psi)+\\vec w=V_g(\\sin\\theta,\\cos\\theta)`。
    """
    if not v_air_mps > 0.0:
        raise ValueError("v_air_mps must be > 0")
    th, _, w_perp = _components(theta_rad, wind_xy)
    ratio = w_perp / v_air_mps
    with np.errstate(invalid="ignore"):
        out = np.where(np.abs(ratio) < 1.0, th - np.arcsin(np.clip(ratio, -1.0, 1.0)), np.nan)
    return np.atleast_1d(out)


# ---------------------------------------------------------------------------
# R_eff、地面坡度
# ---------------------------------------------------------------------------
def r_eff(spec: PlatformSpec, wind_speed_mps: float) -> float:
    """:math:`R_{eff}=\\max\\{R_{plan},R_{theo}(1+W/V_a)^2\\}`（D1；= ``spec.r_eff`` 薄包裝）。"""
    return spec.r_eff(wind_speed_mps)


def r_eff_wind_speed(wind: WindField, spec: PlatformSpec, z_site_m: float, z_op_max_m: float,
                     policy: str) -> float:
    """:math:`R_{eff}` 所用風速 :math:`W_{R,c}`（規格 §3.2；``cfg.r_eff_wind_policy``）。

    * ``"max_layer"``（保守）：:math:`\\max\\{W(z): z\\in[z_{site}+H_{tc},\\ z_{op,\\max}]\\}`，
      :math:`H_{tc}` 取區間下緣 ``transition_cmd().h_tc_clear_m``（範圍較寬 ⇒ 更保守）；
      ``z_op_max_m`` 應由呼叫端已取 :math:`\\max(z_{op,c}, z_{transit,c})` over cells。
    * ``"op_layer"``：作業層風速 ``wind.speed_at(z_op_max_m)``。
    不變量：``max_layer`` 之值 ≥ 範圍內任一 ``speed_at``（含 ``op_layer`` 值）。
    """
    if policy == "max_layer":
        z_lo = float(z_site_m) + spec.transition_cmd().h_tc_clear_m
        z_hi = max(float(z_op_max_m), z_lo)
        return wind.max_speed_between(z_lo, z_hi)
    if policy == "op_layer":
        return wind.speed_at(float(z_op_max_m))
    raise ValueError(f"r_eff_wind_policy 須為 'max_layer' 或 'op_layer'，得 {policy!r}")


def tan_gamma_ground(spec: PlatformSpec, which: Literal["climb", "descent"], theta_rad: float,
                     wind_xy: np.ndarray, auw_kg: float, arc: bool) -> float:
    """地面坡度正切（規格 §3.2）= ``spec.tan_gamma_ground(which, V_g(θ), auw, arc)`` 薄包裝。

    :math:`\\tan\\gamma^g_c=V_a\\sin\\gamma_c(m)\\kappa/V_g`（弧段 κ = cos φ_max）、
    :math:`\\tan\\gamma^g_d=V_a\\sin\\gamma_d/V_g`；該方向不可行（:math:`V_g\\le0` 或 nan）→ ``inf``。
    A、W=8 順風：:math:`\\tan\\gamma^g_c=0.0381`（2.18°）。
    """
    vg = float(ground_speed(float(theta_rad), spec.cruise_airspeed_mps, wind_xy)[0])
    if not (vg > 0.0):
        return math.inf
    return spec.tan_gamma_ground(which, vg, auw_kg, arc)


def worst_case_v_g(spec: PlatformSpec, wind_speed_mps: float, v_g_min_mps: float = 3.0) -> float:
    """最壞（純逆風）地速 :math:`\\max(V_{g,\\min}, V_a-W)`（能量上界化用，規格 §5.5）。"""
    return max(float(v_g_min_mps), spec.cruise_airspeed_mps - max(0.0, float(wind_speed_mps)))


# ---------------------------------------------------------------------------
# 風中時間（規格 §8.1；D14）
# ---------------------------------------------------------------------------
def _wind_xy_at(wind: Any, z_m: float) -> np.ndarray:
    """``WindField``（或任何具 ``wind_vec`` 者）／(2,) 向量／None → 單一高度之 (2,) 風向量。"""
    if wind is None:
        return np.zeros(2)
    if hasattr(wind, "wind_vec"):
        return np.asarray(wind.wind_vec(np.atleast_1d(float(z_m))), dtype=np.float64).reshape(-1, 2)[0]
    w = np.asarray(wind, dtype=np.float64).reshape(-1)
    if w.size != 2:
        raise TypeError("wind must be a WindField (wind_vec), a (2,) wind vector, or None")
    return w


def time_line(p0: np.ndarray, p1: np.ndarray, v_air_mps: float, wind: WindField, z_m: float) -> float:
    """直線段時間 :math:`L/V_g(\\theta)`（規格 §8.1）；``z_m`` 為 ENU z（風取一次）。

    :math:`V_g\\le0` 或不可行 → ``inf``；:math:`L=0` → 0。
    """
    if not v_air_mps > 0.0:
        raise ValueError("v_air_mps must be > 0")
    a = np.asarray(p0, dtype=np.float64).reshape(-1)[:2]
    b = np.asarray(p1, dtype=np.float64).reshape(-1)[:2]
    dx, dy = float(b[0] - a[0]), float(b[1] - a[1])
    length = math.hypot(dx, dy)
    if length <= 0.0:
        return 0.0
    theta = math.atan2(dx, dy)
    vg = float(ground_speed(theta, v_air_mps, _wind_xy_at(wind, z_m))[0])
    if not (vg > 0.0):
        return math.inf
    return length / vg


def time_arc(center: np.ndarray, R: float, theta0_rad: float, dtheta_rad: float, v_air_mps: float,
             wind: WindField, z_m: float, step_rad: float = DEFAULT_ARC_STEP_RAD) -> float:
    """弧段時間（D14）：中點 Riemann 和 :math:`\\sum_j R\\,\\delta/V_g(\\theta_j)`。

    :math:`n=\\lceil|\\Delta\\theta|/\\text{step}\\rceil`、:math:`\\delta=|\\Delta\\theta|/n`、
    :math:`\\theta_j=\\theta_0+\\mathrm{sign}(\\Delta\\theta)(j+\\tfrac12)\\delta`；取點規則與
    :func:`hv_dccpp.dubins.path_time` 相同（同一路徑兩者相等到 1e-9）。``step_rad → 0`` 收斂至
    :math:`\\int R\\,d\\theta/V_g`。任一步 :math:`V_g\\le0` → ``inf``；:math:`\\Delta\\theta=0` → 0。
    ``center`` 只作形狀檢查（均勻風下弧時間與圓心無關）。
    """
    if not v_air_mps > 0.0:
        raise ValueError("v_air_mps must be > 0")
    if not R > 0.0:
        raise ValueError("R must be > 0")
    if not step_rad > 0.0:
        raise ValueError("step_rad must be > 0")
    c = np.asarray(center, dtype=np.float64).reshape(-1)
    if c.size < 2:
        raise ValueError("center must be (2,)")
    mag = abs(float(dtheta_rad))
    if mag <= 0.0:
        return 0.0
    n = max(1, int(math.ceil(mag / step_rad - 1e-12)))
    delta = mag / n
    j = np.arange(n, dtype=np.float64)
    th = float(theta0_rad) + math.copysign(1.0, float(dtheta_rad)) * delta * (j + 0.5)
    vg = ground_speed(th, v_air_mps, _wind_xy_at(wind, z_m))
    ok = vg > 0.0
    if not ok.all():
        return math.inf
    contrib = R * delta / vg
    return float(contrib.sum())


def circle_time_factor(v_air_mps: float, wind_speed_mps: float) -> float:
    """整圈時間因子 :math:`\\oint d\\theta/V_g \\big/ (2\\pi/V_a)\\ \\ge 1`（D14；Jensen）。

    閉式：:math:`1/V_g=(\\sqrt{V_a^2-W^2\\sin^2\\varphi}-W\\cos\\varphi)/(V_a^2-W^2)`，
    :math:`\\oint\\sqrt{V_a^2-W^2\\sin^2\\varphi}\\,d\\varphi=4V_aE(m)`，:math:`m=(W/V_a)^2` ⇒
    factor :math:`=2V_a^2E(m)/(\\pi(V_a^2-W^2))`。A、W=5：1.043。快速代理 :math:`1+(W/V_a)^2` 在
    :math:`W/V_a\\le0.514`（A 11.0、B 9.8 m/s；含放飛門檻 :math:`W_{go}=8`）內 ≥ 本值，之後本值更大
    （:math:`W\\to V_a` 時發散）。:math:`W\\ge V_a` → ``inf``。
    """
    if not v_air_mps > 0.0:
        raise ValueError("v_air_mps must be > 0")
    w = abs(float(wind_speed_mps))
    if w >= v_air_mps:
        return math.inf
    m = (w / v_air_mps) ** 2
    return float(2.0 * v_air_mps * v_air_mps * ellipe(m) / (math.pi * (v_air_mps * v_air_mps - w * w)))


def proxy_turn_time(length_m: float, v_air_mps: float, wind_speed_mps: float) -> float:
    """快速代理 :math:`L/V_a\\cdot(1+(W/V_a)^2)`（D14；§7.3 :math:`t_{turn}`）。"""
    if not v_air_mps > 0.0:
        raise ValueError("v_air_mps must be > 0")
    ratio = float(wind_speed_mps) / v_air_mps
    return float(length_m) / v_air_mps * (1.0 + ratio * ratio)
