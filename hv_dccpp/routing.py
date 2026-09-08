"""hv_dccpp.routing — keep-out 集合、切線圖／可視圖繞行、AVOID（契約 §6B；規格 §8.5、§8.6；D13、D24、D30、D41）。

**約定（契約 §0）**

* 本地 ENU（m）；航跡方位 ``theta`` 自北順時針（rad）：:math:`\\hat t=(\\sin\\theta,\\cos\\theta)`、
  右法向 :math:`\\hat n=(\\cos\\theta,-\\sin\\theta)`。左轉 L = theta 遞減（逆時針）、右轉 R = theta 遞增（順時針）；
  半徑 r、圓心 c、轉向 s（+1 = R、−1 = L）之圓上方位 θ 處的點為 :math:`p=c-s\\,r\\,\\hat n(\\theta)`
  （與 :mod:`hv_dccpp.dubins` 完全一致）。
* 高度一律 ENU z（D42）：障礙頂高 ``top_msl_m`` 以 :meth:`GeoFrame.msl_to_enu_z` 轉換，``height_agl_m`` 以
  **自然地形**（``TerrainGrid.z_raw``，燒入前）之地面 z 加上高度；禁飛區樓層／天花板同法。
* keep-out（規格 §8.5）：:math:`\\mathcal K_o=(\\text{geom}_o\\oplus\\mathrm{disc}(b_h),\\ z_{top,o}+b_v)`，點→圓、線→膠囊、
  面→圓角偏移；:math:`b_h` = ``Obstacle.keepout_buffer_m`` 或 ``cfg.b_h_m``（D13：非表 1 之
  ``obstacle_horizontal_buffer_m``）。禁飛區依 D41 **不**加水平緩衝（路徑掃掠足跡 ⊕ ``half_width``（= :math:`b_{trk}`）
  即為 NoFly ⊕ :math:`b_{trk}` 之檢查），樓層／天花板各外擴 :math:`b_v`。
* 2.5D（規格 §8.5）：只有 ``z_hint`` 低於頂高（且不低於樓層）之 keep-out 才進入繞行圖；作業路徑不繞行。
* 繞行（規格 §8.6、D30）：主方法為圓角偏移多邊形（:math:`\\rho=\\max(b_h,R_{eff})`）之切線圖 Dijkstra
  （節點 = 切點，邊 = 公切線／弧，邊權由呼叫端回呼——AVOID 以 §8.1 之風中時間積分）；備援為可視圖
  （頂點緩衝 :math:`b_h+R(\\sec(\\Delta/2)-1)`）＋ 圓角（fillet）／Dubins 接合。

**切線圖之幾何實作**：圓角偏移多邊形 :math:`\\text{geom}\\oplus\\mathrm{disc}(\\rho)` 之邊界只含直線與半徑 :math:`\\rho` 之弧
（弧心 = 原始幾何之凸頂點），故最短繞行路徑僅由「頂點圓上的弧」與「圓間公切線」組成；本模組直接以
原始幾何頂點為圓心建立圓（半徑 :math:`R_{path}=\\max(\\rho,R_{eff})`，起訖點之轉彎圓亦同半徑），碰撞檢查則對
``KeepoutSet.polys``（已 ⊕ :math:`b_h`）以淨空 ``clearance_m`` 進行。所有弧半徑一致 ⇒ 回傳之 :class:`DubinsPath`
（``word="AVOID"``、``R_m=R_path``）可直接使用 :func:`dubins.discretize` / :func:`dubins.path_time` / ``sample``。

第 2 層模組：只依賴 ``dubins``、``wind``、``geometry``、``geo``、``mission``（皆較低層）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra as _sp_dijkstra
from shapely.geometry import LineString, Point, Polygon

from .dubins import WORDS, DubinsPath, DubinsSegment, Pose, dubins_all_words, dubins_path
from .geo import GeoFrame, TerrainGrid
from .mission import NoFlyZone, Obstacle, PlannerConfig
from .wind import ground_speed, time_arc

__all__ = [
    "KeepoutSet",
    "RoutingError",
    "TangentGraph",
    "VisibilityGraph",
    "avoid",
    "avoid_rho_m",
    "direct_clear_path",
    "polyline_to_path",
    "segment_time",
    "path_is_clear",
    "path_clearance_m",
    "AVOID_WORD",
    "AVOID_MARGIN_M",
]

TWO_PI: float = 2.0 * math.pi
AVOID_WORD: str = "AVOID"
AVOID_MARGIN_M: float = 0.5           # AVOID 內部淨空 = half_width + 本裕度（吸收弦近似與浮點誤差）
_POLYLINE_DS_M: float = 5.0           # 掃掠足跡取樣間距（契約不變量：polyline(5).buffer(half_width)）
_CHORD_DS_M: float = 5.0              # 弧段碰撞檢查之弦長
_ARC_SAG_TOL_M: float = 0.25          # 弦近似之矢高容差（ds²/(8r) ≤ 0.25 對 r ≥ 12.5 m）
_LINE_TOL_M: float = 1e-3             # 直線段碰撞門檻之浮點容差
_ANGLE_TOL: float = 1e-7              # 同圓上節點合併之角度容差（rad）
_ZERO_LEN: float = 1e-9
_VIRTUAL_W: float = 1e-9              # 虛擬源／匯邊權（避免稀疏矩陣顯式零）
_DSM_SKIP_CATEGORIES: frozenset[str] = frozenset({"tree", "building"})   # D24
_DEFAULT_GEOM_INSET_M: float = 0.5    # geoms 缺省（= polys 簡化）時宣告之內縮量


class RoutingError(RuntimeError):
    """AVOID 無解（切線圖與可視圖備援皆失敗）。"""


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _t_hat(theta: np.ndarray | float) -> np.ndarray:
    th = np.asarray(theta, dtype=float)
    return np.stack([np.sin(th), np.cos(th)], axis=-1)


def _n_hat(theta: np.ndarray | float) -> np.ndarray:
    th = np.asarray(theta, dtype=float)
    return np.stack([np.cos(th), -np.sin(th)], axis=-1)


def _mod2pi(x: np.ndarray | float) -> np.ndarray:
    y = np.mod(np.asarray(x, dtype=float), TWO_PI)
    return np.where(y > TWO_PI - 1e-12, 0.0, y)


def _circle_point(center: np.ndarray, r: float, sign: float, theta: float) -> np.ndarray:
    """轉向 ``sign``（+1 R／−1 L）之圓上方位 θ 處的點 :math:`c-s\\,r\\,\\hat n(\\theta)`。"""
    return np.asarray(center, dtype=float) + np.array([-sign * r * math.cos(theta), sign * r * math.sin(theta)])


def _wind_xy(wind: Any, z_m: float) -> np.ndarray:
    """``WindField``（鴨子型別 ``wind_vec``）／(2,) 向量／None → 單一高度之吹向風向量 (2,)。"""
    if wind is None:
        return np.zeros(2)
    if hasattr(wind, "wind_vec"):
        return np.asarray(wind.wind_vec(np.atleast_1d(float(z_m))), dtype=float).reshape(-1, 2)[0]
    w = np.asarray(wind, dtype=float).reshape(-1)
    if w.size != 2:
        raise TypeError("wind must be a WindField-like object (wind_vec), a (2,) vector or None")
    return w


def _polys_of(geom: shapely.Geometry) -> list[Polygon]:
    if geom is None or geom.is_empty:
        return []
    return [g for g in shapely.get_parts(geom) if isinstance(g, Polygon) and not g.is_empty]


def _largest_polygon(geom: shapely.Geometry) -> Polygon | None:
    parts = _polys_of(geom)
    if not parts:
        return None
    return max(parts, key=lambda p: p.area)


def _ring_vertices(poly: Polygon, tol_m: float) -> list[np.ndarray]:
    """簡化後各環之頂點 (k,2)（去除閉合重複點）；Douglas–Peucker 只保留原頂點子集。"""
    simp = shapely.simplify(poly, tol_m, preserve_topology=True) if tol_m > 0 else poly
    out: list[np.ndarray] = []
    for part in _polys_of(simp):
        rings = [part.exterior] + list(part.interiors)
        for ring in rings:
            xy = np.asarray(ring.coords, dtype=float)[:, :2]
            if xy.shape[0] >= 2 and np.allclose(xy[0], xy[-1]):
                xy = xy[:-1]
            if xy.shape[0] >= 1:
                out.append(xy)
    return out


def _geom_vertices(geom: shapely.Geometry) -> np.ndarray:
    """任意 shapely 幾何之頂點 (k,2)（多邊形含內環；閉合重複點去除；相鄰重複點合併）。"""
    if geom is None or geom.is_empty:
        return np.zeros((0, 2))
    rows: list[np.ndarray] = []
    for part in shapely.get_parts(geom):
        if part.is_empty:
            continue
        if isinstance(part, Polygon):
            rows.extend(_ring_vertices(part, 0.0))
        else:
            xy = np.asarray(shapely.get_coordinates(part), dtype=float).reshape(-1, 2)
            if xy.shape[0] >= 2 and np.allclose(xy[0], xy[-1]):
                xy = xy[:-1]
            rows.append(xy)
    if not rows:
        return np.zeros((0, 2))
    pts = np.vstack(rows)
    if pts.shape[0] > 1:
        keep = np.ones(pts.shape[0], dtype=bool)
        keep[1:] = np.hypot(*(pts[1:] - pts[:-1]).T) > 1e-6
        pts = pts[keep]
    return pts


def _pose(p: Any) -> Pose:
    if isinstance(p, Pose):
        return p
    arr = np.asarray(p, dtype=float).reshape(3)
    return Pose(float(arr[0]), float(arr[1]), float(arr[2]))


# ---------------------------------------------------------------------------
# 風中時間（規格 §8.1；D14）— 單段回呼與向量化版本（取點規則與 dubins.path_time 相同）
# ---------------------------------------------------------------------------
def segment_time(seg: DubinsSegment, v_air_mps: float, wind_xy: np.ndarray | None,
                 step_rad: float = 0.0873) -> float:
    """單段風中時間（規格 §8.1；D14）：直線 :math:`L/V_g(\\theta)`、弧 :math:`\\sum R\\delta/V_g(\\theta_j)`（中點 Riemann 和）。

    ``wind_xy`` 為吹向向量 (2,)（None = 無風）；不可飛（:math:`V_g\\le0`）→ inf。弧半徑由 ``length_m/|dtheta|`` 取得
    （AVOID 路徑之弧半徑 = ``DubinsPath.R_m``）。
    """
    w = np.zeros(2) if wind_xy is None else np.asarray(wind_xy, dtype=float).reshape(2)
    if seg.length_m <= _ZERO_LEN:
        return 0.0
    if seg.kind == "S":
        vg = float(ground_speed(seg.start.theta_rad, v_air_mps, w)[0])
        if not (vg > 0.0):
            return math.inf
        return seg.length_m / vg
    if abs(seg.dtheta_rad) <= _ZERO_LEN:
        return 0.0
    r = seg.length_m / abs(seg.dtheta_rad)
    return time_arc(np.asarray(seg.center, dtype=float), r, seg.start.theta_rad, seg.dtheta_rad,
                    v_air_mps, w, 0.0, step_rad)


def _line_times(theta: np.ndarray, length: np.ndarray, v_air_mps: float, wind_xy: np.ndarray) -> np.ndarray:
    """直線段時間 (M,)：:math:`L/V_g(\\theta)`；:math:`V_g\\le0` → inf；L=0 → 0。"""
    theta = np.asarray(theta, dtype=float).reshape(-1)
    length = np.asarray(length, dtype=float).reshape(-1)
    if theta.size == 0:
        return np.zeros(0)
    vg = ground_speed(theta, v_air_mps, wind_xy)
    ok = vg > 0.0
    out = np.where(ok, length / np.where(ok, vg, 1.0), np.inf)
    return np.where(length <= 0.0, 0.0, out)


def _arc_times(theta0: np.ndarray, dtheta: np.ndarray, radius: np.ndarray, v_air_mps: float,
               wind_xy: np.ndarray, step_rad: float) -> np.ndarray:
    """弧段時間 (M,)（D14 中點 Riemann 和，:math:`n=\\lceil|\\Delta\\theta|/\\text{step}\\rceil`）；任一步 :math:`V_g\\le0` → inf。"""
    theta0 = np.asarray(theta0, dtype=float).reshape(-1)
    dtheta = np.asarray(dtheta, dtype=float).reshape(-1)
    radius = np.asarray(radius, dtype=float).reshape(-1)
    M = theta0.size
    if M == 0:
        return np.zeros(0)
    mag = np.abs(dtheta)
    n = np.where(mag > 0.0, np.maximum(np.ceil(mag / step_rad - 1e-12).astype(int), 1), 0)
    n_max = int(n.max())
    out = np.zeros(M)
    if n_max == 0:
        return out
    delta = np.where(n > 0, mag / np.maximum(n, 1), 0.0)
    j = np.arange(n_max, dtype=float)[None, :]
    th = theta0[:, None] + np.sign(dtheta)[:, None] * delta[:, None] * (j + 0.5)
    vg = ground_speed(th.reshape(-1), v_air_mps, wind_xy).reshape(M, n_max)
    mask = j < n[:, None]
    ok = vg > 0.0
    contrib = np.where(mask & ok, radius[:, None] * delta[:, None] / np.where(ok, vg, 1.0), 0.0)
    out = contrib.sum(axis=1)
    out[(mask & ~ok).any(axis=1)] = np.inf
    return out


# ---------------------------------------------------------------------------
# 掃掠足跡檢查
# ---------------------------------------------------------------------------
def path_is_clear(path: DubinsPath, keepouts: "KeepoutSet | None", half_width_m: float, z_hint_m: float,
                  ds_m: float = _POLYLINE_DS_M) -> bool:
    """契約不變量之檢查：``path.polyline(ds).buffer(half_width)`` 與 ``keepouts.active(z)`` 不相交。"""
    if keepouts is None or not keepouts.active(z_hint_m):
        return True
    return not keepouts.intersects(path.polyline(ds_m), half_width_m, z_hint_m)


def path_clearance_m(path: DubinsPath, keepouts: "KeepoutSet | None", z_hint_m: float,
                     ds_m: float = _POLYLINE_DS_M) -> float:
    """路徑折線至有效 keep-out 之最小距離（無有效者 → inf）。"""
    if keepouts is None:
        return math.inf
    return keepouts.clearance_m(path.polyline(ds_m), z_hint_m)


# ---------------------------------------------------------------------------
# KeepoutSet（規格 §8.5；D13、D24、D41、D42）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class KeepoutSet:
    """keep-out 集合（契約 §6B）。

    * ``polys``：ENU 多邊形。障礙已 ⊕ :math:`b_h`（點→圓、線→膠囊、面→圓角偏移）；禁飛區為原多邊形（D41：
      其緩衝 :math:`b_{trk}` 由路徑掃掠足跡 ``half_width`` 提供）。
    * ``tops_z``：ENU z 頂高 + :math:`b_v`（禁飛區無天花板 → +inf）；``floors_z``：ENU z 樓層 − :math:`b_v`
      （障礙／無樓層 → −inf）。
    * ``active(z)``：``floors_z ≤ z < tops_z`` 者（規格 §8.5 之 2.5D：只有該處 :math:`z<z_{top}+b_v` 才進圖）。
    * ``kinds``：``"obstacle"`` / ``"nofly"``；``categories``：障礙類別（禁飛區為 ``"nofly"``）。
    * ``geoms``：**未緩衝**之原始 ENU 幾何（切線圖以其頂點為弧心）；``buffers_m``：各 keep-out 之水平緩衝
      （``polys[i] ⊆ geoms[i] ⊕ disc(buffers_m[i])``）。缺省（只給契約欄位）時 ``geoms`` = ``polys`` 之 0.5 m 簡化、
      ``buffers_m`` = 0.5（宣告簡化內縮量）。
    """

    polys: list[Polygon]
    tops_z: list[float]
    ids: list[str]
    tree: shapely.STRtree
    floors_z: list[float] = field(default_factory=list)
    kinds: list[str] = field(default_factory=list)
    categories: list[str] = field(default_factory=list)
    b_h_m: float = 50.0
    b_v_m: float = 30.0
    notes: list[str] = field(default_factory=list)
    geoms: list[shapely.Geometry] = field(default_factory=list)
    buffers_m: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        n = len(self.polys)
        if len(self.tops_z) != n or len(self.ids) != n:
            raise ValueError("KeepoutSet: polys / tops_z / ids 長度須一致")
        if not self.floors_z:
            self.floors_z = [-math.inf] * n
        if not self.kinds:
            self.kinds = ["obstacle"] * n
        if not self.categories:
            self.categories = ["other"] * n
        if not self.geoms:
            self.geoms = [shapely.simplify(p, _DEFAULT_GEOM_INSET_M, preserve_topology=True) for p in self.polys]
            self.buffers_m = [_DEFAULT_GEOM_INSET_M] * n
        if not self.buffers_m:
            self.buffers_m = [0.0] * n
        if (len(self.floors_z) != n or len(self.kinds) != n or len(self.categories) != n
                or len(self.geoms) != n or len(self.buffers_m) != n):
            raise ValueError("KeepoutSet: floors_z / kinds / categories / geoms / buffers_m 長度須與 polys 一致")
        self.tops_z = [float(v) for v in self.tops_z]
        self.floors_z = [float(v) for v in self.floors_z]
        self.buffers_m = [float(v) for v in self.buffers_m]

    # -- 查詢 -----------------------------------------------------------------
    def __len__(self) -> int:
        return len(self.polys)

    def active(self, z_hint_m: float) -> list[int]:
        """``floors_z ≤ z_hint < tops_z`` 之索引（升冪）。``z_hint`` 為 nan → 全部視為有效（保守）。"""
        z = float(z_hint_m)
        if math.isnan(z):
            return list(range(len(self.polys)))
        return [i for i in range(len(self.polys)) if self.floors_z[i] <= z < self.tops_z[i]]

    def active_polys(self, z_hint_m: float) -> list[Polygon]:
        return [self.polys[i] for i in self.active(z_hint_m)]

    def intersects(self, line: shapely.LineString, half_width_m: float, z_hint_m: float) -> bool:
        """``line ⊕ disc(half_width)`` 是否碰到任一有效 keep-out（距離 ≤ half_width 即視為碰撞）。"""
        act = self.active(z_hint_m)
        if not act or line is None or line.is_empty:
            return False
        hits = self.tree.query(line, predicate="dwithin", distance=max(float(half_width_m), 0.0))
        if hits.size == 0:
            return False
        act_set = set(act)
        return any(int(h) in act_set for h in np.atleast_1d(hits))

    def clearance_m(self, line: shapely.LineString, z_hint_m: float) -> float:
        """``line`` 至有效 keep-out 之最小距離（無有效者 → inf）。"""
        act = self.active(z_hint_m)
        if not act:
            return math.inf
        return float(min(self.polys[i].distance(line) for i in act))

    def max_buffer_m(self, z_hint_m: float) -> float:
        """有效 keep-out 之最大水平緩衝（無有效者 → 0）。"""
        act = self.active(z_hint_m)
        return max((self.buffers_m[i] for i in act), default=0.0)

    # -- 建構 -----------------------------------------------------------------
    @classmethod
    def build(cls, obstacles: Sequence[Obstacle], nofly: Sequence[NoFlyZone], frame: GeoFrame,
              terrain: TerrainGrid, cfg: PlannerConfig, t_window: tuple[float, float] | None,
              burn: bool = True) -> "KeepoutSet":
        """由表 2 障礙與禁飛區建立 keep-out（規格 §6.1、§8.5；D13、D24、D41、D42）。

        * 障礙：ENU 幾何 ⊕ ``disc(keepout_buffer_m or cfg.b_h_m)``（圓角）；頂高 ``top_msl_m`` →
          ``frame.msl_to_enu_z``（取各頂點之最大值）或 ``height_agl_m`` + 自然地形（``z_raw``）地面 z 之最大值；
          ``tops_z = z_top + cfg.b_v_m``。地形範圍外之 AGL 障礙頂高取 +inf（恆有效）並記錄於 ``notes``。
        * 禁飛區：只納入 ``zone.is_active(t_window)`` 者（D22；``t_window=None`` → 全部）；多邊形不加緩衝（D41）；
          樓層 ``z_floor_msl_m`` → ENU z（頂點最小值）− b_v、天花板 → ENU z（頂點最大值）+ b_v；缺省 −inf／+inf。
        * ``burn=True``：障礙以 ⊕b_h 多邊形與 ``tops_z`` 燒入 ``terrain``（就地、單調、冪等；D24：``terrain.is_dsm``
          時 tree／building 類不燒入，tower／powerline／other 仍燒入）。禁飛區不燒入。
        """
        polys: list[Polygon] = []
        tops: list[float] = []
        floors: list[float] = []
        ids: list[str] = []
        kinds: list[str] = []
        cats: list[str] = []
        geoms: list[shapely.Geometry] = []
        bufs: list[float] = []
        notes: list[str] = []
        b_h = float(cfg.b_h_m)
        b_v = float(cfg.b_v_m)
        raw_grid: TerrainGrid | None = None
        burn_polys: list[Polygon] = []
        burn_tops: list[float] = []

        for obs in obstacles:
            coords = np.asarray(obs.coords_wgs84, dtype=float).reshape(-1, 2)
            xy = frame.to_enu_xy(coords[:, 0], coords[:, 1])
            if obs.geom_type == "point":
                geom: shapely.Geometry = Point(xy[0])
            elif obs.geom_type == "line":
                geom = LineString(xy)
            else:
                geom = shapely.make_valid(Polygon(xy))
                big = _largest_polygon(geom)
                if big is None:
                    geom = Polygon(xy).buffer(0.0)
                else:
                    geom = big
            b = b_h if obs.keepout_buffer_m is None else float(obs.keepout_buffer_m)
            poly = _largest_polygon(geom.buffer(b)) if b > 0.0 else _largest_polygon(geom)
            if poly is None or poly.is_empty:
                notes.append(f"obstacle[{obs.obs_id}]: empty keep-out geometry (buffer {b:g} m) — skipped")
                continue
            if obs.top_msl_m is not None:
                z_top = float(np.max(frame.msl_to_enu_z(xy, float(obs.top_msl_m))))
            else:
                if raw_grid is None:
                    raw_grid = TerrainGrid(terrain.x0_m, terrain.y0_m, terrain.step_m, terrain.z_raw,
                                           z_raw=terrain.z_raw)
                if obs.geom_type == "point":
                    zg = float(raw_grid.z_at(xy[:1])[0])
                else:
                    zg = raw_grid.max_in_polygon(geom.buffer(max(terrain.step_m, 1.0)))
                    if not math.isfinite(zg):
                        zv = raw_grid.z_at(xy)
                        zg = float(np.nanmax(zv)) if np.isfinite(zv).any() else math.nan
                if not math.isfinite(zg):
                    notes.append(f"obstacle[{obs.obs_id}]: ground z unknown (outside terrain) — top set to +inf (always active)")
                    z_top = math.inf
                else:
                    z_top = zg + float(obs.height_agl_m)
            top = z_top + b_v
            polys.append(poly)
            tops.append(top)
            floors.append(-math.inf)
            ids.append(obs.obs_id)
            kinds.append("obstacle")
            cats.append(obs.category)
            geoms.append(geom)
            bufs.append(b)
            if burn and math.isfinite(top):
                if terrain.is_dsm and obs.category in _DSM_SKIP_CATEGORIES:
                    notes.append(f"obstacle[{obs.obs_id}]: DSM — {obs.category} not burned (D24)")
                else:
                    burn_polys.append(poly)
                    burn_tops.append(top)

        for zone in nofly:
            if not zone.is_active(t_window):
                notes.append(f"nofly[{zone.zone_id}]: inactive in window {t_window} (D22) — excluded")
                continue
            coords = np.asarray(zone.polygon_wgs84, dtype=float).reshape(-1, 2)
            xy = frame.to_enu_xy(coords[:, 0], coords[:, 1])
            poly = _largest_polygon(shapely.make_valid(Polygon(xy)))
            if poly is None or poly.is_empty:
                notes.append(f"nofly[{zone.zone_id}]: empty polygon — skipped")
                continue
            floor = -math.inf
            top = math.inf
            if zone.z_floor_msl_m is not None:
                floor = float(np.min(frame.msl_to_enu_z(xy, float(zone.z_floor_msl_m)))) - b_v
            if zone.z_ceiling_msl_m is not None:
                top = float(np.max(frame.msl_to_enu_z(xy, float(zone.z_ceiling_msl_m)))) + b_v
            polys.append(poly)
            tops.append(top)
            floors.append(floor)
            ids.append(zone.zone_id)
            kinds.append("nofly")
            cats.append("nofly")
            geoms.append(poly)
            bufs.append(0.0)

        if burn_polys:
            terrain.burn(burn_polys, burn_tops)
        tree = shapely.STRtree(polys)
        return cls(polys=polys, tops_z=tops, ids=ids, tree=tree, floors_z=floors, kinds=kinds,
                   categories=cats, b_h_m=b_h, b_v_m=b_v, notes=notes, geoms=geoms, buffers_m=bufs)


# ---------------------------------------------------------------------------
# 切線圖（規格 §8.6 主方法；D30）
# ---------------------------------------------------------------------------
def _tangent_arrays(centers: np.ndarray, radii: np.ndarray, i_idx: np.ndarray, j_idx: np.ndarray,
                    s_i: np.ndarray, s_j: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """圓 i（轉向 s_i）→ 圓 j（轉向 s_j）之有向公切線（向量化）。

    切線方位 θ 滿足 :math:`\\vec d\\cdot\\hat n(\\theta)=s_jr_j-s_ir_i`（兩切點沿 θ 前進且各在其圓之對應側），
    即 :math:`\\sin(\\varphi-\\theta)=\\Delta/D`，取 :math:`\\theta=\\varphi-\\arcsin(\\Delta/D)`（前進分量 ≥ 0）；
    長度 :math:`\\sqrt{D^2-\\Delta^2}`。回傳 ``(ok, theta, length, p1, p2)``。
    """
    ci = centers[i_idx]
    cj = centers[j_idx]
    d = cj - ci
    D = np.hypot(d[:, 0], d[:, 1])
    phi = np.arctan2(d[:, 0], d[:, 1])
    ri = radii[i_idx]
    rj = radii[j_idx]
    delta = s_j * rj - s_i * ri
    with np.errstate(invalid="ignore", divide="ignore"):
        ratio = np.where(D > 0.0, delta / np.where(D > 0.0, D, 1.0), 2.0)
    ok = (D > 1e-9) & (np.abs(ratio) <= 1.0)
    ratio = np.clip(ratio, -1.0, 1.0)
    theta = phi - np.arcsin(ratio)
    length = np.sqrt(np.maximum(D * D - delta * delta, 0.0))
    nh = _n_hat(theta)
    p1 = ci - (s_i * ri)[:, None] * nh
    p2 = cj - (s_j * rj)[:, None] * nh
    return ok, theta, length, p1, p2


def _arc_chords(centers: np.ndarray, radii: np.ndarray, signs: np.ndarray, th0: np.ndarray,
                dth: np.ndarray, ds_m: float) -> np.ndarray:
    """M 條弧之弦折線（shapely LineString 陣列），弦長 ≤ ``ds_m``。"""
    M = th0.size
    n = np.maximum(1, np.ceil(radii * np.abs(dth) / ds_m - 1e-12).astype(int))
    counts = n + 1
    offsets = np.concatenate([[0], np.cumsum(counts)[:-1]])
    total = int(counts.sum())
    idx = np.repeat(np.arange(M), counts)
    k = np.arange(total) - offsets[idx]
    frac = k / n[idx]
    th = th0[idx] + dth[idx] * frac
    pts = centers[idx] - (signs[idx] * radii[idx])[:, None] * _n_hat(th)
    return shapely.linestrings(pts, indices=idx)


class TangentGraph:
    """圓角偏移多邊形之切線圖（規格 §8.6；D30）。

    * 障礙：``keepouts.active(z_hint)`` 之原始幾何頂點為圓心、半徑 :math:`R_{path}=\\max(\\rho,R_m)`（``shortest`` 時定）；
      起訖狀態之左右轉彎圓亦為半徑 :math:`R_{path}`。節點 = 切點（含起訖狀態本身），邊 = 圓間有向公切線（直線）與
      同圓同轉向相鄰切點間之弧；Dijkstra 以呼叫端回呼 ``weight(DubinsSegment)`` 為邊權（或向量化風中時間／長度）。
    * 碰撞檢查對 ``keepouts.polys``（已 ⊕ b_h）以淨空 ``clearance_m`` 進行（直線精確、弧以 ≤5 m 弦近似）；
      ``clearance_m`` 缺省 = :math:`\\rho-\\max_i b_{h,i}`（幾何本身提供之淨空）；要求之淨空不得超過此值
      （否則 ``ValueError``——呼叫端應加大 ρ；:func:`avoid_rho_m`）。
    * 回傳之 :class:`DubinsPath`：``word="AVOID"``、``R_m=R_path``、``segments`` 為任意長之 S／L／R 交錯序列，
      所有弧半徑一致（可直接用 ``dubins.discretize / path_time / sample``）；``start``/``end`` 恰為要求之狀態。
    """

    def __init__(self, keepouts: KeepoutSet, rho_m: float, z_hint_m: float,
                 clearance_m: float | None = None, *, chord_ds_m: float = _CHORD_DS_M) -> None:
        if not rho_m > 0.0:
            raise ValueError("rho_m must be > 0")
        self.keepouts = keepouts
        self.rho_m = float(rho_m)
        self.z_hint_m = float(z_hint_m)
        self.chord_ds_m = float(chord_ds_m)
        self.active_idx: list[int] = list(keepouts.active(z_hint_m))
        self.polys: list[Polygon] = [keepouts.polys[i] for i in self.active_idx]
        self.tree: shapely.STRtree | None = shapely.STRtree(self.polys) if self.polys else None
        bufs = [keepouts.buffers_m[i] for i in self.active_idx]
        self.min_offset_m: float = (self.rho_m - max(bufs)) if bufs else math.inf
        if clearance_m is None:
            self.clearance_m: float = max(self.min_offset_m, 0.0) if math.isfinite(self.min_offset_m) else 0.0
        else:
            self.clearance_m = float(clearance_m)
            if self.clearance_m > self.min_offset_m + 1e-9:
                raise ValueError(
                    f"TangentGraph: clearance_m={self.clearance_m:.3f} 超過 rho − max(b_h)={self.min_offset_m:.3f}；"
                    "請加大 rho_m（見 avoid_rho_m）")
        verts = [_geom_vertices(keepouts.geoms[i]) for i in self.active_idx]
        self.obs_centers: np.ndarray = np.vstack(verts) if verts else np.zeros((0, 2))
        self.obs_owner: np.ndarray = (np.concatenate([np.full(v.shape[0], k, dtype=int) for k, v in enumerate(verts)])
                                      if verts else np.zeros(0, dtype=int))
        self.n_nodes_last: int = 0
        self.n_edges_last: int = 0
        self._oo_cache: dict[float, dict[str, Any]] = {}

    # -- 碰撞檢查 -------------------------------------------------------------
    def _blocked(self, lines: np.ndarray, distance_m: float) -> np.ndarray:
        """(M,) bool：各線段距任一有效 keep-out ≤ ``distance_m``。"""
        M = len(lines)
        out = np.zeros(M, dtype=bool)
        if self.tree is None or M == 0:
            return out
        hits = self.tree.query(lines, predicate="dwithin", distance=max(float(distance_m), 0.0))
        if hits.size:
            out[np.unique(hits[0])] = True
        return out

    def path_radius_m(self, R_m: float) -> float:
        """所有弧之半徑 :math:`R_{path}=\\max(\\rho,R_m)`。"""
        return max(self.rho_m, float(R_m))

    # -- 最短路 ---------------------------------------------------------------
    def shortest(self, a: Pose, b: Pose, R_m: float, weight: Callable[[DubinsSegment], float] | None = None,
                 *, v_air_mps: float | None = None, wind_xy: np.ndarray | None = None,
                 step_rad: float = 0.0873) -> DubinsPath | None:
        """自 ``a`` 至 ``b`` 之最小權重繞行路徑；無解 → ``None``。

        ``weight``：逐段回呼（契約簽章）。``weight=None`` 時：給 ``v_air_mps`` → 向量化風中時間（``wind_xy`` (2,)
        吹向向量，None = 無風；規格 §8.1 取點規則），否則 → 路徑長度。權重 inf 之邊視為不可用。
        """
        a = _pose(a)
        b = _pose(b)
        R = self.path_radius_m(R_m)
        if not R > 0.0:
            raise ValueError("R_m must be > 0")
        a_xy = np.array([a.x_m, a.y_m])
        b_xy = np.array([b.x_m, b.y_m])
        na = _n_hat(a.theta_rad)
        nb = _n_hat(b.theta_rad)
        special = np.array([a_xy - R * na, a_xy + R * na, b_xy - R * nb, b_xy + R * nb])
        centers = np.vstack([special, self.obs_centers])
        N = centers.shape[0]
        radii = np.full(N, R)

        # ---- 有向公切線 ----
        # (i) 障礙圓 ↔ 障礙圓：依 R 快取（幾何、淨空、各圓相鄰快取節點間之弧淨空）
        oo = self._oo_cache_for(R)
        # (ii) 含起訖圓之有序對（起訖圓 ↔ 全部圓）：逐次計算
        ii, jj = np.meshgrid(np.arange(N), np.arange(N), indexing="ij")
        mask = (ii != jj) & ((ii < 4) | (jj < 4))
        i_se = np.tile(ii[mask], 4)
        j_se = np.tile(jj[mask], 4)
        P = int(mask.sum())
        si_se = np.repeat(np.array([-1.0, -1.0, 1.0, 1.0]), P)
        sj_se = np.repeat(np.array([-1.0, 1.0, -1.0, 1.0]), P)
        i_se, j_se, si_se, sj_se, th_se, L_se, _, _ = self._clear_tangents(centers, radii, i_se, j_se, si_se, sj_se)
        i_idx = np.concatenate([oo["i"] + 4, i_se])
        j_idx = np.concatenate([oo["j"] + 4, j_se])
        s_i = np.concatenate([oo["s_i"], si_se])
        s_j = np.concatenate([oo["s_j"], sj_se])
        theta_m = np.concatenate([oo["theta"], _mod2pi(th_se)])
        length = np.concatenate([oo["length"], L_se])
        T = theta_m.size

        # ---- 節點（同圓同轉向、方位相同者合併）----
        # 候選：切線兩端 + 起訖狀態（起點於圓 0 (L)/1 (R)，終點於圓 2/3）
        cand_circle = np.concatenate([i_idx, j_idx, [0, 1, 2, 3]])
        cand_sign = np.concatenate([s_i, s_j, [-1.0, 1.0, -1.0, 1.0]])
        cand_theta = np.concatenate([theta_m, theta_m, _mod2pi(np.array([a.theta_rad, a.theta_rad,
                                                                          b.theta_rad, b.theta_rad]))])
        order = np.lexsort((cand_theta, cand_sign, cand_circle))
        cc, cs, ct = cand_circle[order], cand_sign[order], cand_theta[order]
        new = np.ones(cc.size, dtype=bool)
        if cc.size > 1:
            same = (cc[1:] == cc[:-1]) & (cs[1:] == cs[:-1]) & (ct[1:] - ct[:-1] <= _ANGLE_TOL)
            new[1:] = ~same
        cluster = np.cumsum(new) - 1
        node_of_cand = np.empty(cc.size, dtype=int)
        node_of_cand[order] = cluster
        K = int(cluster[-1]) + 1 if cc.size else 0
        node_circle = cc[new]
        node_sign = cs[new]
        node_theta = ct[new]
        node_pos = centers[node_circle] - (node_sign * radii[node_circle])[:, None] * _n_hat(node_theta)
        src_L, src_R, dst_L, dst_R = (int(v) for v in node_of_cand[2 * T: 2 * T + 4])

        # ---- 邊：切線 ----
        e_u = node_of_cand[:T]
        e_v = node_of_cand[T: 2 * T]
        # ---- 邊：弧（同圓同轉向相鄰節點，循環；落在快取「淨空弧」內者免檢查）----
        au, av, adth, unknown = self._group_arcs(node_circle, node_sign, node_theta, K, oo["groups"], 4)
        if unknown.any():
            sel = np.flatnonzero(unknown)
            chords = _arc_chords(centers[node_circle[au[sel]]], radii[node_circle[au[sel]]], node_sign[au[sel]],
                                 node_theta[au[sel]], adth[sel], self.chord_ds_m)
            blocked = self._blocked(chords, self.clearance_m - _ARC_SAG_TOL_M)
            keep = np.ones(au.size, dtype=bool)
            keep[sel[blocked]] = False
            au, av, adth = au[keep], av[keep], adth[keep]
        A = au.size

        # ---- 邊權 ----
        if weight is not None:
            w_line = np.array([weight(self._line_segment(node_pos[e_u[k]], node_pos[e_v[k]], theta_m[k], length[k]))
                               for k in range(T)], dtype=float) if T else np.zeros(0)
            w_arc = np.array([weight(self._arc_segment(centers[node_circle[au[k]]], R, node_sign[au[k]],
                                                       node_theta[au[k]], adth[k]))
                              for k in range(A)], dtype=float) if A else np.zeros(0)
        elif v_air_mps is not None:
            w = np.zeros(2) if wind_xy is None else np.asarray(wind_xy, dtype=float).reshape(2)
            w_line = _line_times(theta_m, length, float(v_air_mps), w)
            w_arc = _arc_times(node_theta[au], adth, np.full(A, R), float(v_air_mps), w, step_rad)
        else:
            w_line = length.copy()
            w_arc = R * np.abs(adth)

        rows = np.concatenate([e_u, au, [K, K], [dst_L, dst_R]]).astype(int)
        cols = np.concatenate([e_v, av, [src_L, src_R], [K + 1, K + 1]]).astype(int)
        wts = np.concatenate([w_line, w_arc, [_VIRTUAL_W] * 4]).astype(float)
        kind = np.concatenate([np.zeros(T, dtype=int), np.ones(A, dtype=int), np.full(4, 2, dtype=int)])
        payload = np.concatenate([np.arange(T), np.arange(A), np.zeros(4, dtype=int)])
        finite = np.isfinite(wts) & (wts >= 0.0)
        rows, cols, wts, kind, payload = rows[finite], cols[finite], wts[finite], kind[finite], payload[finite]
        if rows.size == 0:
            return None
        # 平行邊取最小權重
        order = np.lexsort((wts, cols, rows))
        rows, cols, wts, kind, payload = rows[order], cols[order], wts[order], kind[order], payload[order]
        first = np.ones(rows.size, dtype=bool)
        first[1:] = (rows[1:] != rows[:-1]) | (cols[1:] != cols[:-1])
        rows, cols, wts, kind, payload = rows[first], cols[first], wts[first], kind[first], payload[first]
        self.n_nodes_last = K + 2
        self.n_edges_last = int(rows.size)

        G = csr_matrix((wts, (rows, cols)), shape=(K + 2, K + 2))
        dist, pred = _sp_dijkstra(G, directed=True, indices=K, return_predecessors=True)
        if not np.isfinite(dist[K + 1]):
            return None
        # 回溯
        chain: list[int] = []
        cur = K + 1
        while cur != K:
            chain.append(cur)
            cur = int(pred[cur])
            if cur < 0:
                return None
        chain.reverse()
        nodes = chain[:-1]                       # 去掉匯點；首元素為起點節點
        edge_lookup: dict[tuple[int, int], tuple[int, int]] = {
            (int(r), int(c)): (int(kd), int(pl)) for r, c, kd, pl in zip(rows, cols, kind, payload)}

        # ---- 組裝段（合併同圓連續弧）----
        segs: list[DubinsSegment] = []
        th_cur = a.theta_rad
        pos_cur = a_xy.copy()
        pending_arc: list[float] | None = None      # [circle, sign, dtheta_sum]
        total = 0.0

        def flush_arc() -> None:
            nonlocal th_cur, pos_cur, pending_arc, total
            if pending_arc is None:
                return
            circ, sgn, dth = int(pending_arc[0]), float(pending_arc[1]), float(pending_arc[2])
            if abs(dth) > _ZERO_LEN:
                c = centers[circ]
                seg = self._arc_segment(c, R, sgn, th_cur, dth, start_xy=pos_cur)
                segs.append(seg)
                th_cur = seg.end.theta_rad
                pos_cur = np.array([seg.end.x_m, seg.end.y_m])
                total += seg.length_m
            pending_arc = None

        for u, v in zip(nodes[:-1], nodes[1:]):
            kd, pl = edge_lookup[(u, v)]
            if kd == 1:                                    # 弧
                circ, sgn = int(node_circle[u]), float(node_sign[u])
                if pending_arc is not None and (int(pending_arc[0]) != circ or pending_arc[1] != sgn):
                    flush_arc()
                if pending_arc is None:
                    pending_arc = [circ, sgn, 0.0]
                pending_arc[2] += float(adth[pl])
            else:                                          # 切線
                flush_arc()
                p_end = node_pos[v]
                dx, dy = float(p_end[0] - pos_cur[0]), float(p_end[1] - pos_cur[1])
                L = math.hypot(dx, dy)
                if L > _ZERO_LEN:
                    th_s = math.atan2(dx, dy)
                    th_s += TWO_PI * round((th_cur - th_s) / TWO_PI)      # 展開至 th_cur 附近（差 ≤ 1e-7）
                    seg = self._line_segment(pos_cur, p_end, th_s, L)
                    segs.append(seg)
                    pos_cur = np.array([seg.end.x_m, seg.end.y_m])
                    th_cur = th_s
                    total += L
        flush_arc()
        if not segs:
            return None
        # 終點貼齊為要求之狀態（位置誤差 ≪ 1 mm）
        last = segs[-1]
        if math.hypot(last.end.x_m - b.x_m, last.end.y_m - b.y_m) > 1e-3 + 1e-7 * max(R, 1.0):
            return None
        segs[-1] = DubinsSegment(kind=last.kind, length_m=last.length_m, start=last.start,
                                 end=Pose(b.x_m, b.y_m, b.theta_rad), center=last.center, dtheta_rad=last.dtheta_rad)
        first_seg = segs[0]
        segs[0] = DubinsSegment(kind=first_seg.kind, length_m=first_seg.length_m,
                                start=Pose(a.x_m, a.y_m, a.theta_rad), end=first_seg.end,
                                center=first_seg.center, dtheta_rad=first_seg.dtheta_rad)
        return DubinsPath(word=AVOID_WORD, R_m=float(R), segments=tuple(segs), length_m=float(total))

    # -- 切線／弧之向量化建構與快取 ------------------------------------------
    def _clear_tangents(self, centers: np.ndarray, radii: np.ndarray, i_idx: np.ndarray, j_idx: np.ndarray,
                        s_i: np.ndarray, s_j: np.ndarray) -> tuple[np.ndarray, ...]:
        """有向公切線（過濾不存在、零長與碰撞者）→ ``(i, j, s_i, s_j, theta, length, p1, p2)``。"""
        ok, theta, length, p1, p2 = _tangent_arrays(centers, radii, i_idx, j_idx, s_i, s_j)
        ok &= length > 1e-6
        i_idx, j_idx, s_i, s_j = i_idx[ok], j_idx[ok], s_i[ok], s_j[ok]
        theta, length, p1, p2 = theta[ok], length[ok], p1[ok], p2[ok]
        if theta.size:
            lines = shapely.linestrings(np.stack([p1, p2], axis=1))
            keep = ~self._blocked(lines, self.clearance_m - _LINE_TOL_M)
            i_idx, j_idx, s_i, s_j = i_idx[keep], j_idx[keep], s_i[keep], s_j[keep]
            theta, length, p1, p2 = theta[keep], length[keep], p1[keep], p2[keep]
        return i_idx, j_idx, s_i, s_j, theta, length, p1, p2

    @staticmethod
    def _group_arcs(node_circle: np.ndarray, node_sign: np.ndarray, node_theta: np.ndarray, K: int,
                    groups: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]], offset: int
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """同圓同轉向相鄰節點間之弧（循環）→ ``(u, v, dtheta, unknown)``；``unknown`` False = 已知淨空（快取）。

        節點須依 (circle, sign, theta) 排序。``groups[(circle−offset, sign)] = (angles, clear)``：快取節點角（升冪）與
        「角 k → 角 k+1（循環）」之弧淨空旗標；本次弧恆為某快取弧之子弧（本次節點 ⊇ 快取節點）。
        """
        arc_u: list[np.ndarray] = []
        arc_v: list[np.ndarray] = []
        arc_dth: list[np.ndarray] = []
        arc_unk: list[np.ndarray] = []
        if K:
            grp_start = np.flatnonzero(np.concatenate([[True], (node_circle[1:] != node_circle[:-1])
                                                       | (node_sign[1:] != node_sign[:-1])]))
            grp_end = np.concatenate([grp_start[1:], [K]])
            for g0, g1 in zip(grp_start, grp_end):
                m = g1 - g0
                if m < 2:
                    continue
                ids = np.arange(g0, g1)
                nxt = np.concatenate([ids[1:], ids[:1]])
                th_lo = node_theta[ids]
                gap = _mod2pi(node_theta[nxt] - th_lo)
                gap = np.where(gap <= _ANGLE_TOL, TWO_PI, gap)
                unk = np.ones(m, dtype=bool)
                key = (int(node_circle[g0]) - offset, int(node_sign[g0]))
                cached = groups.get(key)
                if cached is not None:
                    ang, clear = cached
                    idx = np.searchsorted(ang, th_lo + 2.0 * _ANGLE_TOL, side="right") - 1
                    idx = np.where(idx < 0, ang.size - 1, idx)
                    end = ang[(idx + 1) % ang.size]
                    span = _mod2pi(end - ang[idx])
                    span = np.where(span <= _ANGLE_TOL, TWO_PI, span)
                    inside = (_mod2pi(th_lo - ang[idx]) + gap) <= span + 4.0 * _ANGLE_TOL
                    unk = ~(inside & clear[idx])
                if node_sign[g0] > 0:
                    arc_u.append(ids)
                    arc_v.append(nxt)
                    arc_dth.append(gap)
                else:
                    arc_u.append(nxt)
                    arc_v.append(ids)
                    arc_dth.append(-gap)
                arc_unk.append(unk)
        if arc_u:
            return (np.concatenate(arc_u), np.concatenate(arc_v), np.concatenate(arc_dth), np.concatenate(arc_unk))
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int), np.zeros(0), np.zeros(0, dtype=bool)

    def _oo_cache_for(self, R: float) -> dict[str, Any]:
        """障礙圓 ↔ 障礙圓之淨空切線與各圓（含轉向）相鄰切點間弧之淨空旗標（依半徑 R 快取）。"""
        key = float(R)
        hit = self._oo_cache.get(key)
        if hit is not None:
            return hit
        N_o = self.obs_centers.shape[0]
        empty = np.zeros(0)
        out: dict[str, Any] = {"i": np.zeros(0, dtype=int), "j": np.zeros(0, dtype=int), "s_i": empty, "s_j": empty,
                               "theta": empty, "length": empty, "groups": {}}
        if N_o >= 2:
            radii = np.full(N_o, R)
            ii, jj = np.meshgrid(np.arange(N_o), np.arange(N_o), indexing="ij")
            mask = ii != jj
            P = int(mask.sum())
            i_idx = np.tile(ii[mask], 4)
            j_idx = np.tile(jj[mask], 4)
            s_i = np.repeat(np.array([-1.0, -1.0, 1.0, 1.0]), P)
            s_j = np.repeat(np.array([-1.0, 1.0, -1.0, 1.0]), P)
            i_idx, j_idx, s_i, s_j, theta, length, _, _ = self._clear_tangents(self.obs_centers, radii, i_idx,
                                                                                 j_idx, s_i, s_j)
            theta = _mod2pi(theta)
            out.update(i=i_idx, j=j_idx, s_i=s_i, s_j=s_j, theta=theta, length=length)
            # 各 (circle, sign) 之快取節點與弧淨空
            cc = np.concatenate([i_idx, j_idx])
            cs = np.concatenate([s_i, s_j])
            ct = np.concatenate([theta, theta])
            order = np.lexsort((ct, cs, cc))
            cc, cs, ct = cc[order], cs[order], ct[order]
            new = np.ones(cc.size, dtype=bool)
            if cc.size > 1:
                new[1:] = ~((cc[1:] == cc[:-1]) & (cs[1:] == cs[:-1]) & (ct[1:] - ct[:-1] <= _ANGLE_TOL))
            cc, cs, ct = cc[new], cs[new], ct[new]
            K = cc.size
            au, av, adth, _ = self._group_arcs(cc, cs, ct, K, {}, 0)
            if au.size:
                chords = _arc_chords(self.obs_centers[cc[au]], radii[cc[au]], cs[au], ct[au], adth, self.chord_ds_m)
                clear = ~self._blocked(chords, self.clearance_m - _ARC_SAG_TOL_M)
                # 弧 (au→av) 之幾何跨度自低角節點升冪：sign>0 → au 為低角；sign<0 → av 為低角
                lo = np.where(cs[au] > 0, au, av)
                grp_start = np.flatnonzero(np.concatenate([[True], (cc[1:] != cc[:-1]) | (cs[1:] != cs[:-1])]))
                grp_end = np.concatenate([grp_start[1:], [K]])
                g_of = np.zeros(K, dtype=int)
                groups: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}
                for g0, g1 in zip(grp_start, grp_end):
                    g_of[g0:g1] = g0
                    if g1 - g0 >= 2:
                        groups[(int(cc[g0]), int(cs[g0]))] = (ct[g0:g1].copy(), np.ones(g1 - g0, dtype=bool))
                for k in range(au.size):
                    n0 = int(lo[k])
                    groups[(int(cc[n0]), int(cs[n0]))][1][n0 - int(g_of[n0])] = bool(clear[k])
                out["groups"] = groups
        self._oo_cache[key] = out
        return out

    # -- 段建構 ---------------------------------------------------------------
    @staticmethod
    def _line_segment(p1: np.ndarray, p2: np.ndarray, theta: float, length: float) -> DubinsSegment:
        return DubinsSegment(kind="S", length_m=float(length), start=Pose(float(p1[0]), float(p1[1]), float(theta)),
                             end=Pose(float(p2[0]), float(p2[1]), float(theta)), center=None, dtheta_rad=0.0)

    @staticmethod
    def _arc_segment(center: np.ndarray, R: float, sign: float, theta0: float, dtheta: float,
                     start_xy: np.ndarray | None = None) -> DubinsSegment:
        p0 = _circle_point(center, R, sign, theta0) if start_xy is None else np.asarray(start_xy, dtype=float)
        th1 = theta0 + dtheta
        p1 = _circle_point(center, R, sign, th1)
        return DubinsSegment(kind="R" if sign > 0 else "L", length_m=float(R * abs(dtheta)),
                             start=Pose(float(p0[0]), float(p0[1]), float(theta0)),
                             end=Pose(float(p1[0]), float(p1[1]), float(th1)),
                             center=(float(center[0]), float(center[1])), dtheta_rad=float(dtheta))


# ---------------------------------------------------------------------------
# 可視圖備援（規格 §8.6；D30）
# ---------------------------------------------------------------------------
class VisibilityGraph:
    """可視圖備援：keep-out ⊕ (``clearance`` + :math:`R(\\sec(\\Delta_{\\max}/2)-1)`) 之頂點可視圖 + Dijkstra（折線）。

    契約簽章 ``VisibilityGraph(keepouts, R_m, delta_max_rad)``；關鍵字 ``z_hint_m``（None → 全部 keep-out 視為有效）、
    ``clearance_m``（折線至 ``keepouts.polys`` 之最小淨空；AVOID 傳 half_width + 裕度）、``simplify_m``（頂點簡化）。
    頂點緩衝 :math:`R(\\sec(\\Delta/2)-1)` 補償半徑 R 之圓角於轉角 Δ 處向內切入量（``polyline_to_path``）。
    """

    def __init__(self, keepouts: KeepoutSet, R_m: float, delta_max_rad: float = math.pi / 2, *,
                 z_hint_m: float | None = None, clearance_m: float = 0.0, simplify_m: float = 0.5) -> None:
        if not R_m > 0.0:
            raise ValueError("R_m must be > 0")
        if not 0.0 < delta_max_rad < math.pi:
            raise ValueError("delta_max_rad must be in (0, pi)")
        self.keepouts = keepouts
        self.R_m = float(R_m)
        self.delta_max_rad = float(delta_max_rad)
        self.z_hint_m = z_hint_m
        self.clearance_m = float(clearance_m)
        self.fillet_inset_m: float = self.R_m * (1.0 / math.cos(0.5 * self.delta_max_rad) - 1.0)
        act = list(range(len(keepouts))) if z_hint_m is None else keepouts.active(z_hint_m)
        self.active_idx = act
        min_b = min((keepouts.buffers_m[i] for i in act), default=0.0)
        # 頂點緩衝（契約）= clearance + R(sec(Δ/2)−1)；另取大使緩衝後多邊形之凸角局部半徑 ≥ 1.01R + 1 m，
        # 令沿圓角邊界緊貼之折線以半徑 R 圓角時切線長必放得下（見 polyline_to_path）。
        self.buffer_extra_m: float = max(self.clearance_m + self.fillet_inset_m, 1.01 * self.R_m + 1.0 - min_b)
        buffered = [keepouts.polys[i].buffer(self.buffer_extra_m, quad_segs=16) for i in act]
        merged = _polys_of(shapely.unary_union(buffered)) if buffered else []
        self.polys: list[Polygon] = [shapely.simplify(p, simplify_m, preserve_topology=True) for p in merged]
        self.polys = [p for p in self.polys if not p.is_empty]
        self.tree: shapely.STRtree | None = shapely.STRtree(self.polys) if self.polys else None
        self._check_polys: list[Polygon] = [p.buffer(-1e-2) for p in self.polys]
        self._check_tree: shapely.STRtree | None = shapely.STRtree(self._check_polys) if self._check_polys else None
        verts = [v for p in self.polys for v in _ring_vertices(p, 0.0)]
        self.vertices: np.ndarray = np.vstack(verts) if verts else np.zeros((0, 2))

    def _visible(self, lines: np.ndarray) -> np.ndarray:
        M = len(lines)
        vis = np.ones(M, dtype=bool)
        if self._check_tree is None or M == 0:
            return vis
        hits = self._check_tree.query(lines, predicate="intersects")
        if hits.size:
            vis[np.unique(hits[0])] = False
        return vis

    def shortest(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """自點 ``a`` 至點 ``b`` 之最短可視折線 (M,2)（首列 = a、末列 = b）；無解丟 :class:`RoutingError`。"""
        a_xy = np.asarray(a, dtype=float).reshape(-1)[:2]
        b_xy = np.asarray(b, dtype=float).reshape(-1)[:2]
        pts = np.vstack([a_xy[None, :], b_xy[None, :], self.vertices])
        N = pts.shape[0]
        ii, jj = np.triu_indices(N, k=1)
        lines = shapely.linestrings(np.stack([pts[ii], pts[jj]], axis=1))
        vis = self._visible(lines)
        ii, jj = ii[vis], jj[vis]
        if ii.size == 0:
            raise RoutingError("visibility graph: no visible edge (start or goal enclosed)")
        L = np.hypot(*(pts[jj] - pts[ii]).T)
        rows = np.concatenate([ii, jj])
        cols = np.concatenate([jj, ii])
        wts = np.concatenate([L, L])
        wts = np.where(wts <= 0.0, _VIRTUAL_W, wts)
        G = csr_matrix((wts, (rows, cols)), shape=(N, N))
        dist, pred = _sp_dijkstra(G, directed=True, indices=0, return_predecessors=True)
        if not np.isfinite(dist[1]):
            raise RoutingError("visibility graph: goal unreachable")
        chain = [1]
        cur = 1
        while cur != 0:
            cur = int(pred[cur])
            if cur < 0:
                raise RoutingError("visibility graph: broken predecessor chain")
            chain.append(cur)
        chain.reverse()
        return pts[chain].copy()


# ---------------------------------------------------------------------------
# 折線 → 可飛路徑（圓角 fillet + Dubins 接合）
# ---------------------------------------------------------------------------
def _segments_of(path: DubinsPath) -> list[DubinsSegment]:
    return [s for s in path.segments if s.length_m > _ZERO_LEN]


def direct_clear_path(a: Pose, b: Pose, R_m: float, keepouts: KeepoutSet | None, half_width_m: float,
                      z_hint_m: float) -> tuple[DubinsPath, bool]:
    """6-word Dubins 依長度遞增取第一個掃掠足跡淨空者；皆不淨空 → ``(最短路徑, False)``。"""
    a = _pose(a)
    b = _pose(b)
    lengths = dubins_all_words(a.as_array()[None, :], b.as_array()[None, :], R_m)[0]
    order = np.argsort(lengths)
    best: DubinsPath | None = None
    for w in order:
        if not np.isfinite(lengths[w]):
            break
        p = dubins_path(a, b, R_m, WORDS[int(w)])
        if best is None:
            best = p
        if path_is_clear(p, keepouts, half_width_m, z_hint_m):
            return p, True
    if best is None:
        raise RoutingError("no feasible Dubins word")
    return best, False


def _group_corners(pts: np.ndarray, delta_max_rad: float) -> np.ndarray:
    """將折線中同轉向、累計轉角 ≤ ``delta_max`` 之連續內部頂點合併為「虛擬轉角」（首入邊與末出邊之交點）。

    可視折線沿圓角邊界緊貼時由多個小轉角組成，逐點圓角放不下；合併後腿長變長、每個轉角只需一個半徑 R 之圓角
    （切入量 ≤ R(sec(Δ/2)−1)，已由 :class:`VisibilityGraph` 之頂點緩衝補償）。直行（轉角 ≈ 0）頂點直接移除。
    """
    M = pts.shape[0]
    if M < 3:
        return pts
    d = pts[1:] - pts[:-1]
    L = np.hypot(d[:, 0], d[:, 1])
    u = d / L[:, None]
    cross = u[:-1, 0] * u[1:, 1] - u[:-1, 1] * u[1:, 0]
    ang = np.arccos(np.clip((u[:-1] * u[1:]).sum(axis=1), -1.0, 1.0))
    sgn = np.where(cross < 0.0, 1, -1)
    out = [pts[0]]
    k = 0                                  # 內部頂點 k ↔ pts[k+1]，入邊 u[k]、出邊 u[k+1]
    n_int = M - 2
    while k < n_int:
        if ang[k] < 1e-9:
            k += 1
            continue
        s = sgn[k]
        acc = ang[k]
        j = k
        while j + 1 < n_int and sgn[j + 1] == s and ang[j + 1] >= 1e-9 and acc + ang[j + 1] <= delta_max_rad + 1e-9:
            j += 1
            acc += ang[j]
        u_in, u_out = u[k], u[j + 1]
        det = -u_in[0] * u_out[1] + u_out[0] * u_in[1]
        if abs(det) < 1e-12 or acc >= math.pi - 1e-6:
            out.extend(pts[k + 1: j + 2])              # 退化（平行／折返）：保留原頂點
        else:
            rhs = pts[j + 1] - pts[k + 1]
            t = (-u_out[1] * rhs[0] + u_out[0] * rhs[1]) / det
            out.append(pts[k + 1] + t * u_in)
        k = j + 1
    out.append(pts[-1])
    arr = np.asarray(out, dtype=float)
    keep = np.ones(arr.shape[0], dtype=bool)
    keep[1:] = np.hypot(*(arr[1:] - arr[:-1]).T) > 1e-6
    return arr[keep]


def _fillet_plan(pts: np.ndarray, R: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]:
    """折線內部頂點之圓角：回傳 ``(theta_leg (M-1,), turn Δ (M-2,), sign (M-2,), t_len (M-2,), feasible)``。"""
    d = pts[1:] - pts[:-1]
    L = np.hypot(d[:, 0], d[:, 1])
    u = d / L[:, None]
    theta = np.arctan2(u[:, 0], u[:, 1])
    dots = np.clip((u[:-1] * u[1:]).sum(axis=1), -1.0, 1.0)
    delta = np.arccos(dots)
    cross = u[:-1, 0] * u[1:, 1] - u[:-1, 1] * u[1:, 0]
    sign = np.where(cross < 0.0, 1.0, -1.0)           # 順時針（右轉）→ +1
    t_len = np.where(delta > 1e-9, R * np.tan(0.5 * np.minimum(delta, math.pi - 1e-6)), 0.0)
    feasible = bool(np.all(delta < math.pi - 1e-4))
    if delta.size:
        need = np.zeros(L.size)
        need[:-1] += t_len
        need[1:] += t_len
        feasible = feasible and bool(np.all(need <= L + 1e-9))
    return theta, delta, sign, t_len, feasible


def polyline_to_path(poly_xy: np.ndarray, a: Pose, b: Pose, R_m: float, keepouts: KeepoutSet | None = None,
                     half_width_m: float = 0.0, z_hint_m: float = 0.0,
                     delta_max_rad: float = math.pi / 2) -> DubinsPath:
    """可視圖折線 → ``word="AVOID"`` 之 :class:`DubinsPath`（半徑 ``R_m``）。

    先以 :func:`_group_corners` 將同向連續小轉角合併為虛擬轉角（每個 ≤ ``delta_max``），內部頂點以半徑 R 圓角
    （切線長 :math:`R\\tan(\\Delta/2)`），起訖以 Dubins 接合（``a`` → 首圓角入口狀態、末圓角出口狀態 → ``b``；
    各取掃掠足跡淨空之最短 word）。圓角放不下（腿長不足或折返）時改為「頂點狀態 Dubins 接合」：各內部頂點取
    ``(v_k, θ_k)``（出邊方位）為狀態，逐對 Dubins 相接。
    回傳路徑之 ``start``/``end`` 恰為 ``a``/``b``；是否淨空由呼叫端以 :func:`path_is_clear` 驗證。
    """
    a = _pose(a)
    b = _pose(b)
    R = float(R_m)
    pts = np.asarray(poly_xy, dtype=float).reshape(-1, 2)
    if pts.shape[0] > 1:
        keep = np.ones(pts.shape[0], dtype=bool)
        keep[1:] = np.hypot(*(pts[1:] - pts[:-1]).T) > 1e-6
        pts = pts[keep]
    if pts.shape[0] < 2:
        pts = np.array([[a.x_m, a.y_m], [b.x_m, b.y_m]])
    pts = pts.copy()
    pts[0] = (a.x_m, a.y_m)
    pts[-1] = (b.x_m, b.y_m)
    pts = _group_corners(pts, delta_max_rad)
    if pts.shape[0] == 2:
        return direct_clear_path(a, b, R, keepouts, half_width_m, z_hint_m)[0]

    theta, delta, sign, t_len, feasible = _fillet_plan(pts, R)
    segs: list[DubinsSegment] = []
    if feasible:
        n_int = pts.shape[0] - 2
        # 首接合：a → (T_in,1, θ_0)
        t_in = pts[1] - t_len[0] * _t_hat(theta[0])
        segs += _segments_of(direct_clear_path(a, Pose(float(t_in[0]), float(t_in[1]), float(theta[0])), R,
                                               keepouts, half_width_m, z_hint_m)[0])
        for k in range(n_int):
            v = pts[k + 1]
            th_in, th_out = float(theta[k]), float(theta[k + 1])
            p_in = v - t_len[k] * _t_hat(th_in)
            p_out = v + t_len[k] * _t_hat(th_out)
            if t_len[k] > 1e-9:
                s = float(sign[k])
                center = p_in + s * R * _n_hat(th_in)
                dth = s * float(delta[k])
                segs.append(DubinsSegment(kind="R" if s > 0 else "L", length_m=float(R * abs(dth)),
                                          start=Pose(float(p_in[0]), float(p_in[1]), th_in),
                                          end=Pose(float(p_out[0]), float(p_out[1]), th_in + dth),
                                          center=(float(center[0]), float(center[1])), dtheta_rad=dth))
            if k + 1 < n_int:
                nxt_in = pts[k + 2] - t_len[k + 1] * _t_hat(th_out)
                L = float(np.hypot(*(nxt_in - p_out)))
                if L > _ZERO_LEN:
                    segs.append(DubinsSegment(kind="S", length_m=L,
                                              start=Pose(float(p_out[0]), float(p_out[1]), th_out),
                                              end=Pose(float(nxt_in[0]), float(nxt_in[1]), th_out),
                                              center=None, dtheta_rad=0.0))
            else:
                segs += _segments_of(direct_clear_path(Pose(float(p_out[0]), float(p_out[1]), th_out), b, R,
                                                       keepouts, half_width_m, z_hint_m)[0])
    else:
        poses = [a] + [Pose(float(pts[k][0]), float(pts[k][1]), float(theta[k])) for k in range(1, pts.shape[0] - 1)] + [b]
        for p0, p1 in zip(poses[:-1], poses[1:]):
            segs += _segments_of(direct_clear_path(p0, p1, R, keepouts, half_width_m, z_hint_m)[0])
    if not segs:
        return direct_clear_path(a, b, R, keepouts, half_width_m, z_hint_m)[0]
    # 方位連續化（各接合段之 theta 以 2π 為週期一致）、端點貼齊
    out: list[DubinsSegment] = []
    th_cur = a.theta_rad
    for s in segs:
        shift = th_cur - s.start.theta_rad
        shift = TWO_PI * round(shift / TWO_PI)
        st = Pose(s.start.x_m, s.start.y_m, s.start.theta_rad + shift)
        en = Pose(s.end.x_m, s.end.y_m, s.end.theta_rad + shift)
        out.append(DubinsSegment(kind=s.kind, length_m=s.length_m, start=st, end=en, center=s.center,
                                 dtheta_rad=s.dtheta_rad))
        th_cur = en.theta_rad
    first = out[0]
    out[0] = DubinsSegment(kind=first.kind, length_m=first.length_m, start=Pose(a.x_m, a.y_m, a.theta_rad),
                           end=first.end, center=first.center, dtheta_rad=first.dtheta_rad)
    last = out[-1]
    out[-1] = DubinsSegment(kind=last.kind, length_m=last.length_m, start=last.start,
                            end=Pose(b.x_m, b.y_m, b.theta_rad), center=last.center, dtheta_rad=last.dtheta_rad)
    return DubinsPath(word=AVOID_WORD, R_m=R, segments=tuple(out), length_m=float(sum(s.length_m for s in out)))


# ---------------------------------------------------------------------------
# AVOID（規格 §8.6 偽碼）
# ---------------------------------------------------------------------------
def avoid_rho_m(keepouts: KeepoutSet, R_m: float, z_hint_m: float, half_width_m: float,
                margin_m: float = AVOID_MARGIN_M) -> float:
    """切線圖偏移半徑 :math:`\\rho=\\max(b_h,R_{eff})`（規格 §8.6），再與「有效 keep-out 最大緩衝 + half_width + 裕度」取大，
    使 ρ 圓上之弧至 ``keepouts.polys`` 之淨空 ≥ half_width + 裕度（契約不變量）。預設參數下即 :math:`\\max(b_h,R_{eff})`。"""
    return max(float(R_m), float(keepouts.b_h_m), keepouts.max_buffer_m(z_hint_m) + float(half_width_m) + float(margin_m))


def avoid(a: Pose, b: Pose, R_m: float, keepouts: KeepoutSet | None, z_hint_m: float, wind: Any,
          v_air_mps: float, half_width_m: float, tg_cache: dict) -> DubinsPath:
    """AVOID（規格 §8.6；D30）：

    1. 直接 Dubins：6 word 依長度遞增，第一個掃掠足跡（``polyline(5) ⊕ half_width``）與 ``keepouts.active(z_hint)``
       不相交者即回傳（最短 word 淨空時與 :func:`dubins.dubins_path` 完全相同）。
    2. 切線圖（:class:`TangentGraph`，ρ 由 :func:`avoid_rho_m`，快取 ``tg_cache[(rho, z_hint)]``）以 §8.1 風中時間為邊權
       之 Dijkstra；回傳 ``word="AVOID"``、所有弧半徑 :math:`\\max(\\rho,R_m)` 之路徑。
    3. 備援：:class:`VisibilityGraph`（快取 ``tg_cache[("vis", R_m, z_hint, half_width)]``）折線 + 圓角／Dubins 接合。
    三者皆失敗 → :class:`RoutingError`。``wind``：``WindField``／(2,) 向量／None；``z_hint_m`` 為 ENU z。
    """
    a = _pose(a)
    b = _pose(b)
    if not R_m > 0.0:
        raise ValueError("R_m must be > 0")
    if keepouts is None or not keepouts.active(z_hint_m):
        return dubins_path(a, b, R_m)
    path, clear = direct_clear_path(a, b, R_m, keepouts, half_width_m, z_hint_m)
    if clear:
        return path
    wind_xy = _wind_xy(wind, z_hint_m)
    clearance = float(half_width_m) + AVOID_MARGIN_M
    rho = avoid_rho_m(keepouts, R_m, z_hint_m, half_width_m)
    key = (rho, float(z_hint_m))
    tg = tg_cache.get(key) if tg_cache is not None else None
    if tg is None or not isinstance(tg, TangentGraph) or tg.clearance_m + 1e-9 < clearance or tg.keepouts is not keepouts:
        tg = TangentGraph(keepouts, rho, z_hint_m, clearance_m=clearance)
        if tg_cache is not None:
            tg_cache[key] = tg
    tg_path = tg.shortest(a, b, R_m, None, v_air_mps=v_air_mps, wind_xy=wind_xy)
    if tg_path is not None and path_is_clear(tg_path, keepouts, half_width_m, z_hint_m):
        return tg_path
    # 備援：可視圖
    vkey = ("vis", float(R_m), float(z_hint_m), float(half_width_m))
    vg = tg_cache.get(vkey) if tg_cache is not None else None
    if vg is None or not isinstance(vg, VisibilityGraph) or vg.keepouts is not keepouts:
        vg = VisibilityGraph(keepouts, R_m, z_hint_m=z_hint_m, clearance_m=clearance)
        if tg_cache is not None:
            tg_cache[vkey] = vg
    try:
        poly = vg.shortest(np.array([a.x_m, a.y_m]), np.array([b.x_m, b.y_m]))
    except RoutingError as e:
        raise RoutingError(f"AVOID failed: tangent graph and visibility fallback found no path ({e})") from e
    vpath = polyline_to_path(poly, a, b, R_m, keepouts, half_width_m, z_hint_m)
    if path_is_clear(vpath, keepouts, half_width_m, z_hint_m):
        return vpath
    if tg_path is not None:
        # 切線圖路徑幾何上淨空僅差浮點／弦誤差時，回報其淨空以利診斷
        clr = path_clearance_m(tg_path, keepouts, z_hint_m)
        raise RoutingError(f"AVOID failed: tangent-graph path clearance {clr:.3f} m ≤ half_width {half_width_m:.3f} m "
                           "and visibility fallback path not clear")
    raise RoutingError("AVOID failed: visibility fallback path intersects active keep-outs")
