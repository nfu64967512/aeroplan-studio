"""geometry.py — 多邊形簡化、凸分割、掃描座標與條帶（模組契約 §6）。

對應規格：§7.2（只外擴 DP 簡化、反射頂點切割凸分割、MERGE）、§7.4（sensor-driven、
conservative-AGL 條帶產生）、§7.5（殘帶子胞）、§7.7（覆蓋完整性）；裁決 D3（超集帶單調收縮、
整體置中平移）、D17（殘帶）、D31（DP 只外擴）、D38（迴轉帶）、D41（禁區緩衝）。

全域約定（契約 §0）：ENU 公尺；航跡方位 theta 自北順時針，t̂ = (sin θ, cos θ)、右法向
n̂ = (cos θ, −sin θ)；掃描座標 u 沿 t̂、v 沿 n̂，原點為胞在 (t̂, n̂) 投影之 (u_min, v_min) 角點；
條帶方向 dir ∈ {−1, +1}：−1 = 自 p^L 向 p^R（LR）、+1 = 自 p^R 向 p^L（RL）。

本模組為第 0 層，只依賴 numpy / shapely（Pose 取自同層 dubins.py，尚未存在時以契約相同定義後備）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Literal, Protocol

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import split, unary_union

try:  # dubins.py 為同層模組；尚未就位時使用與契約 §5 完全相同的後備定義
    from .dubins import Pose
except ImportError:  # pragma: no cover - 只在 dubins.py 缺席時執行

    @dataclass(frozen=True, slots=True)
    class Pose:  # type: ignore[no-redef]
        """位姿 (x, y, θ)；契約 §5 之後備定義（dubins.py 缺席時）。"""

        x_m: float
        y_m: float
        theta_rad: float

        def as_array(self) -> np.ndarray:
            return np.array([self.x_m, self.y_m, self.theta_rad], dtype=float)


__all__ = [
    "Pose",
    "Strip",
    "Cell",
    "SensorLike",
    "LocalEnvelopeFn",
    "StripInfeasibleError",
    "simplify_outward",
    "concavity",
    "is_convex",
    "convex_partition",
    "merge_shallow",
    "min_width_direction",
    "projection_width",
    "sweep_frame",
    "sweep_origin",
    "to_sweep_coords",
    "from_sweep_coords",
    "clip_band",
    "strip_count",
    "turn_band",
    "lay_strips",
    "residual_subcell",
    "strips_union_footprint",
]

_TWO_PI = 2.0 * math.pi
_CUT_OVERHANG_M = 1e-3        # 切線兩端外延量（保證與邊界確實相交，供 shapely.split 節點化）
_MAX_PARTITION_DEPTH = 256    # 凸分割遞迴深度上限（每次切割至少消解一個反射頂點，實務遠小於此）
_MAX_STRIPS = 50_000          # lay_strips 條帶數上限（防止退化輸入無限迴圈）
_FOOTPRINT_TOL_M = 1e-3       # 後置斷言 footprint ⊇ cell 之容差


# ---------------------------------------------------------------------------
# Protocol / 型別
# ---------------------------------------------------------------------------
class SensorLike(Protocol):
    """lay_strips 所需之感測器介面（duck-typed；platform.Sensor 滿足之）。"""

    def swath_m(self, h_agl_m: float) -> float: ...

    def strip_spacing_m(self, h_agl_m: float) -> float: ...


#: (centerline_xy (N,2), theta (N,), band_v_lo, band_v_hi) → (h_min, agl_max, feasible)
#: 規格 §11.3；decompose 以閉包把 altitude.local_envelope 轉成此簽章。
#: 允許回傳第 4 個元素 z_op_m（走廊地形最高 + h_op 之 ENU z 代表值）；缺省時 Strip.z_op_m = h_op_m。
LocalEnvelopeFn = Callable[[np.ndarray, np.ndarray, float, float], tuple[float, float, bool]]


class StripInfeasibleError(RuntimeError):
    """lay_strips(raise_on_infeasible=True) 時，局部包絡回報 feasible=False 所丟出。"""


# ---------------------------------------------------------------------------
# 資料型別
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class Strip:
    """作業條帶（規格 §7.4 / §8.2）。

    pL / pR：左右端點（ENU，含 e_ext 外延）；theta_LR_rad 自 p^L 指向 p^R；
    v_lo_m / v_hi_m：帶在掃描座標之下／上緣（相對 sweep_origin，v 沿右法向 n̂(theta_LR)）。
    feasible / infeasible_reason：局部包絡（§11.3）回報之可行性（契約外附加欄位，見 CONTRACT_DEVIATIONS）。
    """

    strip_id: str
    cell_id: str
    index: int
    cls: str
    pL: np.ndarray
    pR: np.ndarray
    theta_LR_rad: float
    sweep_origin: np.ndarray
    width_m: float
    spacing_m: float
    h_min_m: float
    h_op_m: float
    agl_max_m: float
    v_lo_m: float
    v_hi_m: float
    length_m: float
    z_op_m: float
    feasible: bool = True
    infeasible_reason: str = ""

    @property
    def theta_RL_rad(self) -> float:
        """θ_RL = θ_LR + π（規格 §8.2），正規化至 [0, 2π)。"""
        return (self.theta_LR_rad + math.pi) % _TWO_PI

    def pose(self, kind: Literal["L", "LR", "R", "RL"]) -> Pose:
        """規格 §8.2 四狀態：L=(p^L, θ_LR)、LR=(p^R, θ_LR)、R=(p^R, θ_RL)、RL=(p^L, θ_RL)。"""
        if kind == "L":
            p, th = self.pL, self.theta_LR_rad
        elif kind == "LR":
            p, th = self.pR, self.theta_LR_rad
        elif kind == "R":
            p, th = self.pR, self.theta_RL_rad
        elif kind == "RL":
            p, th = self.pL, self.theta_RL_rad
        else:
            raise ValueError(f"unknown pose kind {kind!r}")
        return Pose(float(p[0]), float(p[1]), float(th))

    def endpoint(self, side: int) -> np.ndarray:
        """side −1 → p^L，+1 → p^R。"""
        if side == -1:
            return self.pL
        if side == 1:
            return self.pR
        raise ValueError("side must be -1 or +1")

    def end_state(self, dir: int) -> Pose:
        """以 dir 掃完之狀態（D4）：dir −1（LR）→ S^{LR} = (p^R, θ_LR)；+1（RL）→ S^{RL} = (p^L, θ_RL)。"""
        if dir == -1:
            return self.pose("LR")
        if dir == 1:
            return self.pose("RL")
        raise ValueError("dir must be -1 or +1")

    def start_state(self, dir: int) -> Pose:
        """以 dir 開掃之狀態（D4）：dir −1（LR）→ S^L = (p^L, θ_LR)；+1（RL）→ S^R = (p^R, θ_RL)。"""
        if dir == -1:
            return self.pose("L")
        if dir == 1:
            return self.pose("R")
        raise ValueError("dir must be -1 or +1")


@dataclass(slots=True)
class Cell:
    """分解後之胞（契約 §6；由 decompose.decompose_area 建構）。"""

    cell_id: str
    area_id: str
    polygon: Polygon
    sensor_req: str
    payload_req_id: str
    priority: float
    time_window_s: tuple[float, float] | None
    area_m2: float
    min_width_m: float
    min_width_dir_rad: float
    dist_home_m: float
    sweep_origin: dict[str, np.ndarray]
    h_op_m: dict[str, float]
    feasible: dict[str, bool]
    infeasible_reason: dict[str, str]
    alpha_rad: dict[str, float]
    strips: dict[str, list[Strip]]
    alt_dirs: dict[str, list[tuple[float, float]]]
    fit: dict[str, float]
    n_strips: dict[str, int]
    T_single_s: dict[str, float]
    d_turn_m: dict[str, float]
    parent_cell_id: str | None = None
    residual_of: str | None = None
    k_A: int | None = None
    slope_exceeds: dict[str, bool] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# 低階幾何工具
# ---------------------------------------------------------------------------
def _coords(geom: shapely.Geometry) -> np.ndarray:
    """所有頂點 (N,2)（含洞）。"""
    return np.asarray(shapely.get_coordinates(geom), dtype=float)


def _ring_xy(ring: shapely.LinearRing) -> np.ndarray:
    """封閉環之頂點（去掉重複的最後一點）。"""
    c = np.asarray(shapely.get_coordinates(ring), dtype=float)
    if len(c) > 1 and c[0, 0] == c[-1, 0] and c[0, 1] == c[-1, 1]:
        c = c[:-1]
    return c


def _polygons_of(geom: shapely.Geometry, min_area: float = 0.0) -> list[Polygon]:
    """自任意幾何抽出面積 > min_area 之 Polygon 清單。"""
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom] if geom.area > min_area else []
    out: list[Polygon] = []
    for g in getattr(geom, "geoms", []):
        out.extend(_polygons_of(g, min_area))
    return out


def _clean(poly: Polygon) -> Polygon:
    """移除共線／重複頂點、外環逆時針、洞順時針（呼叫端須先保證有效）。"""
    cleaned = shapely.simplify(poly, 1e-6, preserve_topology=True)
    if isinstance(cleaned, Polygon) and not cleaned.is_empty and cleaned.is_valid:
        return orient(cleaned, 1.0)
    return orient(poly, 1.0) if isinstance(poly, Polygon) and poly.is_valid else poly


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.hypot(v[0], v[1]))
    return v / n if n > 0 else v


def _turns(ring_xy: np.ndarray) -> np.ndarray:
    """各頂點之轉向角（逆時針環：左轉為正；反射頂點為負）。"""
    p = np.roll(ring_xy, 1, axis=0)
    q = np.roll(ring_xy, -1, axis=0)
    e1 = ring_xy - p
    e2 = q - ring_xy
    cross = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
    dot = (e1 * e2).sum(axis=1)
    return np.arctan2(cross, dot)


def _densify_closed(coords: np.ndarray, step_m: float) -> np.ndarray:
    """把封閉多段線以 ≤ step_m 間距取樣（向量化；含各頂點）。coords 不含重複末點。"""
    c0 = coords
    c1 = np.roll(coords, -1, axis=0)
    seg = c1 - c0
    L = np.hypot(seg[:, 0], seg[:, 1])
    n = np.maximum(1, np.ceil(L / max(step_m, 1e-9))).astype(int)
    idx = np.repeat(np.arange(len(seg)), n)
    start = np.cumsum(n) - n
    k = np.arange(int(n.sum())) - np.repeat(start, n)
    frac = k / np.repeat(n, n)
    return c0[idx] + seg[idx] * frac[:, None]


# ---------------------------------------------------------------------------
# 簡化與凹度
# ---------------------------------------------------------------------------
def simplify_outward(poly: Polygon, delta_edge_m: float, eps_m: float) -> Polygon:
    """只外擴的 Douglas–Peucker 簡化（規格 §7.2、D31）。

    P = simplify(Z ⊕ disc(δ_edge), ε)；DP 誤差 ≤ ε < δ_edge ⇒ Z ⊆ P。為了在任何 ε 下都保證外擴，
    若簡化結果未包含原多邊形則回傳 simplified ∪ original。洞邊界由 buffer 自動內縮並同樣簡化。
    後置條件：result.buffer(1e-6).contains(poly)。
    """
    if poly is None or poly.is_empty:
        return poly
    grown = poly.buffer(delta_edge_m) if delta_edge_m > 0 else poly
    parts = _polygons_of(grown)
    if len(parts) != 1:  # 正 buffer 之單一 Polygon 必為單一 Polygon；防禦性處理
        grown = max(parts, key=lambda p: p.area) if parts else poly
    else:
        grown = parts[0]
    simp = shapely.simplify(grown, max(float(eps_m), 0.0), preserve_topology=True)
    if simp.is_empty or not simp.is_valid or not isinstance(simp, Polygon):
        simp = grown
    if not simp.buffer(1e-6).contains(poly):
        merged = unary_union([simp, poly])
        merged_parts = _polygons_of(merged)
        simp = merged_parts[0] if len(merged_parts) == 1 else max(merged_parts, key=lambda p: p.area)
        if not simp.buffer(1e-6).contains(poly):  # 極端退化：直接回傳外擴多邊形
            simp = grown
    return simp


def concavity(poly: Polygon) -> float:
    """凸包與多邊形之最大 Hausdorff 深度（m）。

    = max( max_{p ∈ ∂poly 頂點} dist(p, ∂hull), max_{q ∈ ∂hull} dist(q, exterior(poly)) )。
    前者對凸包內點為「到各凸包邊支撐線距離之最小值」（凸性下精確且沿邊為凹函數 ⇒ 極值在頂點）；
    後者以 ≤ step 之取樣近似。凸多邊形為 0；洞頂點一併計入。
    """
    if poly is None or poly.is_empty or poly.area <= 0:
        return 0.0
    hull = poly.convex_hull
    if shapely.get_type_id(hull) != 3 or hull.area <= 0:
        return 0.0
    hc = np.asarray(shapely.get_coordinates(hull), dtype=float)[:-1]
    e = np.roll(hc, -1, axis=0) - hc
    L = np.hypot(e[:, 0], e[:, 1])
    keep = L > 1e-12
    hc, e, L = hc[keep], e[keep], L[keep]
    if len(hc) < 3:
        return 0.0
    nrm = np.stack([e[:, 1], -e[:, 0]], axis=1) / L[:, None]
    pts = _coords(poly)
    # (N,K)：|(p − c_k)·n_k|
    d = np.abs(np.einsum("nkj,kj->nk", pts[:, None, :] - hc[None, :, :], nrm))
    fwd = float(d.min(axis=1).max())
    step = max(0.5, hull.length / 4000.0)
    samp = _densify_closed(hc, step)
    rev = float(np.max(shapely.distance(shapely.points(samp), poly.exterior)))
    return max(fwd, rev)


def is_convex(poly: Polygon, tol_deg: float = 2.0) -> bool:
    """外環是否凸（允許 ≤ tol_deg 之反射轉向）；有洞一律 False。"""
    if poly is None or poly.is_empty:
        return True
    if len(poly.interiors) > 0:
        return False
    ring = _ring_xy(orient(poly, 1.0).exterior)
    if len(ring) < 3:
        return True
    tol = math.radians(tol_deg)
    return bool(np.all(_turns(ring) >= -tol))


# ---------------------------------------------------------------------------
# 凸分割（規格 §7.2）
# ---------------------------------------------------------------------------
def _first_hit_cut(P: Polygon, v: np.ndarray, d: np.ndarray, diam: float) -> LineString | None:
    """自邊界頂點 v 沿方向 d 射入，取到第一次離開多邊形的線段（兩端各外延 _CUT_OVERHANG_M）。"""
    ray = LineString([tuple(v), tuple(v + d * diam)])
    inter = ray.intersection(P)
    if inter.is_empty:
        return None
    pieces = [inter] if isinstance(inter, LineString) else [g for g in getattr(inter, "geoms", []) if isinstance(g, LineString)]
    for g in pieces:
        c = np.asarray(g.coords, dtype=float)
        if len(c) < 2:
            continue
        if np.hypot(*(c[0] - v)) < 1e-6:
            e = c[-1]
        elif np.hypot(*(c[-1] - v)) < 1e-6:
            e = c[0]
        else:
            continue
        if np.hypot(*(e - v)) < 1e-6:
            continue
        return LineString([tuple(v - d * _CUT_OVERHANG_M), tuple(e + d * _CUT_OVERHANG_M)])
    return None


def _full_line(v: np.ndarray, d: np.ndarray, diam: float) -> LineString:
    return LineString([tuple(v - d * diam), tuple(v + d * diam)])


def _segment_cut(v: np.ndarray, u: np.ndarray) -> LineString:
    d = _unit(u - v)
    return LineString([tuple(v - d * _CUT_OVERHANG_M), tuple(u + d * _CUT_OVERHANG_M)])


def _cut_candidates(P: Polygon, tol_rad: float) -> list[LineString]:
    """規格 §7.2：對每個反射頂點 v（洞頂點一律視為反射）產生
    {prev_edge 延長, next_edge 延長, 角平分線} ∪ {v→u : u 自 v 可視} 之切線。
    外環反射頂點用「射到第一次出界」的射線並要求切線消解 v（兩側角 ≤ 180°+tol）；洞頂點用整條直線（單一橋接不會分割）。
    """
    minx, miny, maxx, maxy = P.bounds
    diam = 2.0 * math.hypot(maxx - minx, maxy - miny) + 1.0
    ext = _ring_xy(P.exterior)
    holes = [_ring_xy(r) for r in P.interiors]
    cuts: list[LineString] = []
    n_ext = len(ext)
    if n_ext < 3:
        return cuts

    shapely.prepare(P)
    turns = _turns(ext)
    for i in np.nonzero(turns < -tol_rad)[0]:
        i = int(i)
        v = ext[i]
        p = ext[(i - 1) % n_ext]
        q = ext[(i + 1) % n_ext]
        a = _unit(p - v)
        b = _unit(q - v)
        ang_a = math.atan2(a[1], a[0])
        ang_b = math.atan2(b[1], b[0])
        phi = (ang_a - ang_b) % _TWO_PI  # 內角（反射：> π）
        dirs = [-a, -b]
        s = a + b
        if np.hypot(s[0], s[1]) > 1e-9:
            dirs.append(-_unit(s))
        for d in dirs:
            cut = _first_hit_cut(P, v, d, diam)
            if cut is not None:
                cuts.append(cut)
        # 可視頂點（外環、非鄰接；須消解 v）
        mask = np.ones(n_ext, dtype=bool)
        mask[[i, (i - 1) % n_ext, (i + 1) % n_ext]] = False
        U = ext[mask]
        if len(U) == 0:
            continue
        dv = U - v
        psi = (np.arctan2(dv[:, 1], dv[:, 0]) - ang_b) % _TWO_PI
        ok = (psi >= phi - math.pi - tol_rad) & (psi <= math.pi + tol_rad) & (psi > 1e-6) & (psi < phi - 1e-6)
        U = U[ok]
        if len(U) == 0:
            continue
        segs = shapely.linestrings(np.stack([np.broadcast_to(v, U.shape), U], axis=1))
        vis = shapely.covers(P, segs)
        for u in U[vis]:
            cuts.append(_segment_cut(v, u))

    if holes:
        # 洞頂點一律視為反射：整條直線。可視頂點 u 限縮為「外環反射頂點 ∪ 其他洞頂點 ∪ 同洞非鄰接頂點」
        # （通過外環凸頂點之直線對消解洞無額外助益；見 CONTRACT_DEVIATIONS），並以直線鍵去重。
        ext_reflex = ext[turns < -tol_rad]
        seen: set[tuple[float, float]] = set()
        for hi_, h in enumerate(holes):
            n_h = len(h)
            if n_h < 3:
                continue
            others = [ext_reflex] + [hh for k, hh in enumerate(holes) if k != hi_]
            pool = np.vstack(others) if others else np.zeros((0, 2))
            for i in range(n_h):
                v = h[i]
                p = h[(i - 1) % n_h]
                q = h[(i + 1) % n_h]
                a = _unit(p - v)
                b = _unit(q - v)
                dirs = [a, b]
                s = a + b
                if np.hypot(s[0], s[1]) > 1e-9:
                    dirs.append(_unit(s))
                for d in dirs:
                    key = _line_key(v, v + d)
                    if key not in seen:
                        seen.add(key)
                        cuts.append(_full_line(v, d, diam))
                same = np.array([h[j] for j in range(n_h) if j not in (i, (i - 1) % n_h, (i + 1) % n_h)]).reshape(-1, 2)
                U = np.vstack([pool, same]) if len(same) else pool
                if len(U) == 0:
                    continue
                U = U[np.hypot(U[:, 0] - v[0], U[:, 1] - v[1]) > 1e-9]
                if len(U) == 0:
                    continue
                segs = shapely.linestrings(np.stack([np.broadcast_to(v, U.shape), U], axis=1))
                vis = shapely.covers(P, segs)
                for u in U[vis]:
                    key = _line_key(v, u)
                    if key not in seen:
                        seen.add(key)
                        cuts.append(_full_line(v, _unit(u - v), diam))
    return cuts


def _split_polygon(P: Polygon, cut: LineString) -> list[Polygon]:
    try:
        res = split(P, cut)
    except Exception:  # pragma: no cover - shapely 拓樸例外
        return [P]
    parts = shapely.get_parts(res)
    parts = parts[shapely.get_type_id(parts) == 3]
    tiny = 1e-9 * max(P.area, 1.0)
    return [g for g in parts[shapely.area(parts) > tiny]]


def _line_key(p0: np.ndarray, p1: np.ndarray) -> tuple[float, float]:
    """直線之正規化鍵（方向角 mod π、帶號法向偏移），供候選切線去重。"""
    d = p1 - p0
    ang = math.atan2(d[1], d[0]) % math.pi
    if ang > math.pi - 1e-9:
        ang = 0.0
    n = np.array([-math.sin(ang), math.cos(ang)])
    off = float(p0 @ n)
    return (round(ang, 7), round(off, 3))


def _n_strips_est(poly: Polygon, s_ref_m: float) -> int:
    """n_strips_est = ⌈min_width(p)/s_ref⌉（規格 §7.2 評分）。"""
    w = min_width_direction(poly)[1]
    return max(1, int(math.ceil(w / max(s_ref_m, 1e-9) - 1e-9)))


def _partition_rec(P: Polygon, s_ref_m: float, tau_merge_m: float, lambda_cut: float, lambda_n: float,
                   tol_rad: float, a_min_m2: float, depth: int) -> list[Polygon]:
    if P is None or P.is_empty or P.area <= 0:
        return []
    if not P.is_valid:  # 無效輸入（自觸環等）：make_valid 後逐片遞迴，不丟失面積
        out: list[Polygon] = []
        for g in _polygons_of(shapely.make_valid(P)):
            out.extend(_partition_rec(g, s_ref_m, tau_merge_m, lambda_cut, lambda_n, tol_rad, a_min_m2, depth + 1))
        return out
    P = _clean(P)
    if not isinstance(P, Polygon) or P.is_empty or P.area <= 0:
        return _polygons_of(P)
    if len(P.interiors) == 0 and (is_convex(P, math.degrees(tol_rad)) or concavity(P) <= tau_merge_m):
        return [P]
    if depth > _MAX_PARTITION_DEPTH:
        return [P]
    best_score = math.inf
    best_parts: list[Polygon] | None = None
    for cut in _cut_candidates(P, tol_rad):
        parts = _split_polygon(P, cut)
        if len(parts) < 2:
            continue
        if any(p.area < a_min_m2 for p in parts):
            continue
        score = (sum(_n_strips_est(p, s_ref_m) for p in parts)
                 + lambda_cut * cut.intersection(P).length
                 + lambda_n * (len(parts) - 1))
        if score < best_score:
            best_score, best_parts = score, parts
    if best_parts is None:
        if a_min_m2 > 0:
            return _partition_rec(P, s_ref_m, tau_merge_m, lambda_cut, lambda_n, tol_rad, 0.0, depth)
        return [P]  # near_convex_forced：保證有回傳（§7.4 端點取帶內極值不依賴凸性）
    out: list[Polygon] = []
    for p in best_parts:
        out.extend(_partition_rec(p, s_ref_m, tau_merge_m, lambda_cut, lambda_n, tol_rad, a_min_m2, depth + 1))
    return out


def convex_partition(poly: Polygon | MultiPolygon, s_ref_m: float, tau_merge_m: float, lambda_cut: float = 1e-3,
                     lambda_n: float = 30.0, tol_deg: float = 2.0, a_min_m2: float | None = None) -> list[Polygon]:
    """反射頂點切割之凸分割（規格 §7.2，論文引 [42] 思路），含洞與 MultiPolygon。

    - MultiPolygon 逐一處理；無洞且（凸 tol_deg 或 concavity ≤ tau_merge）→ [poly]（近凸即接受）。
    - 候選切線：反射頂點之前／後邊延長、角平分線、至可視頂點；洞頂點一律視為反射（用整條直線）。
    - 評分 Σ_p ⌈min_width(p)/s_ref⌉ + λ_cut·len(cut ∩ P) + λ_n·(|parts|−1)，拒絕面積 < A_min 之片段；
      無合法切線 → A_min=0 重試 → 仍無 → [poly]（總有回傳）。
    - 最後以 merge_shallow（Lien–Amato 準則）合併凹度 ≤ tau_merge 且不增加條帶數之相鄰片段。
    """
    if poly is None or poly.is_empty:
        return []
    a_min = float(s_ref_m) ** 2 if a_min_m2 is None else float(a_min_m2)
    tol_rad = math.radians(tol_deg)
    if isinstance(poly, Polygon):
        if len(poly.interiors) == 0 and (is_convex(poly, tol_deg) or concavity(poly) <= tau_merge_m):
            return [poly]
        parts = _partition_rec(poly, s_ref_m, tau_merge_m, lambda_cut, lambda_n, tol_rad, a_min, 0)
    else:
        parts = []
        for g in _polygons_of(poly):
            parts.extend(_partition_rec(g, s_ref_m, tau_merge_m, lambda_cut, lambda_n, tol_rad, a_min, 0))
    if not parts:
        return _polygons_of(poly) or [poly]  # type: ignore[list-item]
    return merge_shallow(parts, tau_merge_m, lambda p: _n_strips_est(p, s_ref_m))


def merge_shallow(parts: list[Polygon], tau_m: float, n_strips_fn: Callable[[Polygon], int]) -> list[Polygon]:
    """MERGE（規格 §7.2）：相鄰 (Qa,Qb) 若 concavity(Qa∪Qb) ≤ tau 且 n(Qa∪Qb) ≤ n(Qa)+n(Qb)（合併不增加最佳方向條帶數）
    且並集為無洞單一 Polygon → 合併；貪婪，優先條帶數減少最多、其次凹度最小者，直到無法再合併。
    """
    parts = [p for p in parts if p is not None and not p.is_empty and p.area > 0]
    n_cache: dict[int, int] = {}

    def n_of(k: int) -> int:
        if k not in n_cache:
            n_cache[k] = int(n_strips_fn(parts[k]))
        return n_cache[k]

    while len(parts) > 1:
        best: tuple[tuple[float, float], int, int, Polygon, int] | None = None
        for i in range(len(parts)):
            for j in range(i + 1, len(parts)):
                inter = parts[i].intersection(parts[j])
                if inter.is_empty or inter.length <= 1e-6:
                    continue  # 未共享邊
                U = unary_union([parts[i], parts[j]])
                U = _clean(U) if isinstance(U, Polygon) else U
                if not isinstance(U, Polygon) or len(U.interiors) > 0:
                    continue
                c = concavity(U)
                if c > tau_m:
                    continue
                nU = int(n_strips_fn(U))
                if nU > n_of(i) + n_of(j):
                    continue
                key = (float(nU - n_of(i) - n_of(j)), c)
                if best is None or key < best[0]:
                    best = (key, i, j, U, nU)
        if best is None:
            break
        _, i, j, U, nU = best
        parts = [p for k, p in enumerate(parts) if k not in (i, j)] + [U]
        n_cache = {}
    return parts


# ---------------------------------------------------------------------------
# 最小寬度方向、掃描座標
# ---------------------------------------------------------------------------
def min_width_direction(poly: Polygon) -> tuple[float, float]:
    """論文最小寬度方向（dis_{k,μ}）：對凸包每邊 k 求各頂點到該邊之最遠距離 dis_k，
    w_min = min_k dis_k，方向 α = 該邊之航跡方位（自北順時針，正規化至 [0, π)）。
    以凸包邊計算對任何多邊形皆精確（最小寬度為凸包性質；凸多邊形時即論文原式）。
    """
    if poly is None or poly.is_empty:
        return (0.0, 0.0)
    hull = poly.convex_hull
    if shapely.get_type_id(hull) != 3 or hull.area <= 0:
        return (0.0, 0.0)
    c = np.asarray(shapely.get_coordinates(hull), dtype=float)[:-1]   # 凸包外環（無洞），去掉重複末點
    e = np.roll(c, -1, axis=0) - c
    L = np.hypot(e[:, 0], e[:, 1])
    keep = L > 1e-12
    c, e, L = c[keep], e[keep], L[keep]
    if len(c) < 3:
        return (0.0, 0.0)
    nrm = np.stack([e[:, 1], -e[:, 0]], axis=1) / L[:, None]
    d = np.abs(np.einsum("kuj,kj->ku", c[None, :, :] - c[:, None, :], nrm))  # (K edges, U vertices)
    dis_k = d.max(axis=1)
    k = int(np.argmin(dis_k))
    alpha = math.atan2(float(e[k, 0]), float(e[k, 1])) % math.pi
    return (alpha, float(dis_k[k]))


def projection_width(poly: Polygon, alpha_rad: float) -> float:
    """垂直於 α 之投影寬 W(α) = max v − min v（v 沿右法向 n̂(α)）。"""
    if poly is None or poly.is_empty:
        return 0.0
    _, n_hat = sweep_frame(alpha_rad)
    v = _coords(poly) @ n_hat
    return float(v.max() - v.min())


def sweep_frame(alpha_rad: float) -> tuple[np.ndarray, np.ndarray]:
    """(t̂, n̂)：t̂ = (sin α, cos α)、右法向 n̂ = (cos α, −sin α)（契約 §0）。"""
    t_hat = np.array([math.sin(alpha_rad), math.cos(alpha_rad)], dtype=float)
    n_hat = np.array([math.cos(alpha_rad), -math.sin(alpha_rad)], dtype=float)
    return t_hat, n_hat


def sweep_origin(poly: Polygon, alpha_rad: float) -> np.ndarray:
    """多邊形在 (t̂, n̂) 投影之 (u_min, v_min) 角點（ENU）；使 to_sweep_coords(poly) 之 min u = min v = 0。"""
    t_hat, n_hat = sweep_frame(alpha_rad)
    xy = _coords(poly)
    u_min = float((xy @ t_hat).min())
    v_min = float((xy @ n_hat).min())
    return u_min * t_hat + v_min * n_hat


def to_sweep_coords(xy: np.ndarray, origin: np.ndarray, alpha_rad: float) -> np.ndarray:
    """ENU → 掃描座標 [u, v]：u = (xy−origin)·t̂，v = (xy−origin)·n̂。輸入 (N,2) 或 (2,)，回傳同形狀。"""
    t_hat, n_hat = sweep_frame(alpha_rad)
    M = np.stack([t_hat, n_hat], axis=0)  # rows: t, n
    arr = np.asarray(xy, dtype=float)
    o = np.asarray(origin, dtype=float).reshape(2)
    return (arr - o) @ M.T


def from_sweep_coords(uv: np.ndarray, origin: np.ndarray, alpha_rad: float) -> np.ndarray:
    """掃描座標 → ENU：xy = origin + u t̂ + v n̂。"""
    t_hat, n_hat = sweep_frame(alpha_rad)
    M = np.stack([t_hat, n_hat], axis=0)
    arr = np.asarray(uv, dtype=float)
    o = np.asarray(origin, dtype=float).reshape(2)
    return arr @ M + o


def clip_band(poly: Polygon, origin: np.ndarray, alpha_rad: float, v_lo: float, v_hi: float) -> Polygon | MultiPolygon | None:
    """poly ∩ {v_lo ≤ v ≤ v_hi}（v 沿 n̂(α)、相對 origin）。空 → None；多片 → MultiPolygon（非凸／有洞胞）。"""
    if poly is None or poly.is_empty or v_hi <= v_lo:
        return None
    uv = to_sweep_coords(_coords(poly), origin, alpha_rad)
    u0 = float(uv[:, 0].min()) - 1.0
    u1 = float(uv[:, 0].max()) + 1.0
    band = Polygon(from_sweep_coords(np.array([[u0, v_lo], [u1, v_lo], [u1, v_hi], [u0, v_hi]], dtype=float), origin, alpha_rad))
    inter = poly.intersection(band)
    polys = _polygons_of(inter, min_area=1e-12)
    if not polys:
        return None
    return polys[0] if len(polys) == 1 else MultiPolygon(polys)


def strip_count(W_m: float, w_m: float, s_m: float) -> int:
    """n = ⌈(W − (w − s))/s⌉，≥ 1（規格 §7.4；論文 Eq.(6)(7) 等寬情形）。"""
    if s_m <= 0:
        raise ValueError("strip spacing must be positive")
    return max(1, int(math.ceil((W_m - (w_m - s_m)) / s_m - 1e-9)))


def turn_band(poly: Polygon, alpha_rad: float, d_turn_m: float, lateral_m: float) -> Polygon:
    """迴轉帶（D38）：poly ⊕ 矩形 [−d_turn, d_turn] t̂ × [−lateral, lateral] n̂（Minkowski 和，對任意多邊形精確）。

    poly ⊕ R = poly ∪ ⋃_edges (edge ⊕ R)，edge ⊕ R = conv{端點 + R 之四角}；向量化建構後 union。
    """
    if poly is None or poly.is_empty:
        return poly
    d_turn_m = max(float(d_turn_m), 0.0)
    lateral_m = max(float(lateral_m), 0.0)
    if d_turn_m == 0.0 and lateral_m == 0.0:
        return poly
    t_hat, n_hat = sweep_frame(alpha_rad)
    corners = np.array([su * d_turn_m * t_hat + sv * lateral_m * n_hat for su in (-1.0, 1.0) for sv in (-1.0, 1.0)])
    rings = [_ring_xy(poly.exterior)] + [_ring_xy(r) for r in poly.interiors]
    hulls: list[np.ndarray] = []
    for ring in rings:
        if len(ring) < 2:
            continue
        p0 = ring
        p1 = np.roll(ring, -1, axis=0)
        pts = np.concatenate([p0[:, None, :] + corners[None, :, :], p1[:, None, :] + corners[None, :, :]], axis=1)  # (k,8,2)
        hulls.append(shapely.convex_hull(shapely.multipoints(pts)))
    geoms = [poly] + [g for h in hulls for g in h]
    result = shapely.union_all(geoms)
    parts = _polygons_of(result)
    if len(parts) == 1:
        return parts[0]
    return max(parts, key=lambda p: p.area) if parts else poly


# ---------------------------------------------------------------------------
# 條帶產生（規格 §7.4，D3）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class _StripMeta:
    """lay_strips 內部：每條帶之超集帶 [B_lo, B_hi] 與其高度 h_B（供置中平移後重算）。"""

    B_lo: float
    B_hi: float
    h_B: float


def lay_strips(cell_poly: Polygon, cell_id: str, cls: str, alpha_rad: float, sensor: SensorLike, h_op_m: float,
               e_ext_m: float, local_env: LocalEnvelopeFn, eps_h_m: float = 1.0, max_iter: int = 3,
               widen_on_lift: bool = False, *, raise_on_infeasible: bool = False,
               centerline_ds_m: float = 25.0) -> list[Strip]:
    """LAY_STRIPS（規格 §7.4；D3 超集帶單調收縮 + 整體置中平移；論文 Eq.(6)–(9) 之 sensor-driven 版）。

    迴圈條件 strips==[] or strips[-1].v_hi < v_max；每條帶：
      repeat ≤ max_iter：w = swath(h)、band = [v_base, v_base+w]（第一次記為超集帶 B_prev、h_B）、
      sub = clip(cell, band)、u 範圍 = sub 之 u 極值 ∓/± e_ext、中心線 v = v_base + w/2、
      (h_min, agl_max, feasible) = local_env(centerline, θ, B_prev)；h_min ≥ h − ε_h 即收斂，否則 h ← h_min。
      w = swath(min(h_min, h_B))（寬度永不超過超集帶）；v_base += spacing（= w(1 − o_side)）。
    最後一條溢出 Δ = v_hi − v_max > 0 → 全部 v −= Δ/2 並逐條以平移後走廊重算 h_min；
    若任一條 h_min 下降（或帶已空）→ 自該條起丟棄、以新 v_base 重鋪（至多 1 輪、不再平移）。
    後置斷言 strips_union_footprint(strips) ⊇ cell_poly。

    sensor 只需 swath_m(h)、strip_spacing_m(h)。local_env 可回傳第 4 元素 z_op_m；缺省時 Strip.z_op_m = h_op_m。
    local_env 回報 feasible=False 時仍保守鋪設該條並標記 Strip.feasible=False（raise_on_infeasible=True 則丟 StripInfeasibleError）。
    """
    if cell_poly is None or cell_poly.is_empty or cell_poly.area <= 0:
        return []
    origin = sweep_origin(cell_poly, alpha_rad)
    uv_all = to_sweep_coords(_coords(cell_poly), origin, alpha_rad)
    v_max = float(uv_all[:, 1].max())
    v_vertices = np.unique(uv_all[:, 1])
    tol_v = 1e-9 * max(1.0, abs(v_max))
    h_op_m = float(h_op_m)
    e_ext_m = float(e_ext_m)
    max_iter = max(1, int(max_iter))

    def centerline(u_lo: float, u_hi: float, v_c: float) -> tuple[np.ndarray, np.ndarray]:
        n = max(2, int(math.ceil((u_hi - u_lo) / max(centerline_ds_m, 1e-6))) + 1)
        u = np.linspace(u_lo, u_hi, n)
        uv = np.stack([u, np.full(n, v_c)], axis=1)
        return from_sweep_coords(uv, origin, alpha_rad), np.full(n, float(alpha_rad))

    def u_extent(sub: shapely.Geometry) -> tuple[float, float]:
        uv = to_sweep_coords(_coords(sub), origin, alpha_rad)
        return float(uv[:, 0].min()) - e_ext_m, float(uv[:, 0].max()) + e_ext_m

    def call_env(u_lo: float, u_hi: float, v_c: float, B_lo: float, B_hi: float) -> tuple[float, float, bool, float | None]:
        cl_xy, th = centerline(u_lo, u_hi, v_c)
        out = local_env(cl_xy, th, float(B_lo), float(B_hi))
        z_op = float(out[3]) if len(out) > 3 and out[3] is not None else None  # type: ignore[misc]
        return float(out[0]), float(out[1]), bool(out[2]), z_op

    def make_strip(index: int, u_lo: float, u_hi: float, v_lo: float, w: float, spacing: float, h_min: float,
                   agl_max: float, feas: bool, reason: str, z_op: float | None) -> Strip:
        v_c = v_lo + w / 2.0
        pL = from_sweep_coords(np.array([u_lo, v_c]), origin, alpha_rad)
        pR = from_sweep_coords(np.array([u_hi, v_c]), origin, alpha_rad)
        return Strip(
            strip_id=f"{cell_id}_{cls}_{index}", cell_id=cell_id, index=index, cls=cls,
            pL=np.asarray(pL, dtype=float), pR=np.asarray(pR, dtype=float), theta_LR_rad=float(alpha_rad),
            sweep_origin=np.array(origin, dtype=float, copy=True),
            width_m=float(w), spacing_m=float(spacing), h_min_m=float(h_min), h_op_m=h_op_m,
            agl_max_m=float(agl_max), v_lo_m=float(v_lo), v_hi_m=float(v_lo + w), length_m=float(u_hi - u_lo),
            z_op_m=float(z_op) if z_op is not None else h_op_m, feasible=bool(feas), infeasible_reason=reason,
        )

    def lay_one(v_base: float, index: int) -> tuple[Strip, _StripMeta] | None:
        h = h_op_m
        B_lo = B_hi = None
        h_B = h_op_m
        h_min = agl_max = h_op_m
        feas = True
        reason = ""
        z_op: float | None = None
        u_lo = u_hi = 0.0
        for _ in range(max_iter):
            w = float(sensor.swath_m(h))
            if not (w > 0):
                raise ValueError(f"sensor.swath_m({h}) must be positive, got {w}")
            band_lo, band_hi = v_base, v_base + w
            if B_lo is None:
                B_lo, B_hi, h_B = band_lo, band_hi, h
            sub = clip_band(cell_poly, origin, alpha_rad, band_lo, band_hi)
            if sub is None:
                return None
            u_lo, u_hi = u_extent(sub)
            h_min, agl_max, feas, z_op = call_env(u_lo, u_hi, v_base + w / 2.0, B_lo, B_hi)  # type: ignore[arg-type]
            if not math.isfinite(h_min):  # 不可行時閉包可能回傳 nan/inf：退回超集帶高度（保守）
                h_min = h_B
                feas = False
            if not math.isfinite(agl_max):
                agl_max = h_min
            if not feas:
                reason = "local_envelope_infeasible"
                if raise_on_infeasible:
                    raise StripInfeasibleError(f"{cell_id}/{cls} strip {index}: local envelope infeasible (u∈[{u_lo:.1f},{u_hi:.1f}])")
                break
            if h_min >= h - eps_h_m:
                break
            h = h_min
        assert B_lo is not None and B_hi is not None
        if widen_on_lift and feas and h_min > h_B + eps_h_m:
            # 包絡整段抬高：以更寬的新帶重跑一次 LOCAL_ENVELOPE 確認後才採用
            w2 = float(sensor.swath_m(h_min))
            sub2 = clip_band(cell_poly, origin, alpha_rad, v_base, v_base + w2)
            if sub2 is not None:
                u_lo2, u_hi2 = u_extent(sub2)
                h_min2, agl_max2, feas2, z_op2 = call_env(u_lo2, u_hi2, v_base + w2 / 2.0, v_base, v_base + w2)
                if feas2 and h_min2 >= h_min - eps_h_m:
                    B_lo, B_hi, h_B = v_base, v_base + w2, h_min
                    u_lo, u_hi, h_min, agl_max, z_op = u_lo2, u_hi2, min(h_min, h_min2), agl_max2, z_op2
        h_used = min(h_min, h_B)
        w = float(sensor.swath_m(h_used))
        spacing = float(sensor.strip_spacing_m(h_used))
        if not (w > 0) or not (spacing > 0):
            raise ValueError(f"sensor swath/spacing must be positive at h={h_used}: w={w}, s={spacing}")
        strip = make_strip(index, u_lo, u_hi, v_base, w, spacing, h_min, agl_max, feas, reason, z_op)
        return strip, _StripMeta(B_lo, B_hi, h_B)

    def lay_loop(strips: list[Strip], metas: list[_StripMeta], v_base: float) -> None:
        while not strips or strips[-1].v_hi_m < v_max - tol_v:
            res = lay_one(v_base, len(strips) + 1)
            if res is None:  # 投影間隙（MultiPolygon／退化）：跳到下一個有頂點的 v
                nxt = v_vertices[v_vertices > v_base + tol_v]
                if nxt.size == 0:
                    break
                v_base = float(nxt[0])
                continue
            strip, meta = res
            strips.append(strip)
            metas.append(meta)
            v_base = strip.v_lo_m + strip.spacing_m
            if len(strips) > _MAX_STRIPS:
                raise RuntimeError("lay_strips: too many strips (degenerate sensor/cell)")

    strips: list[Strip] = []
    metas: list[_StripMeta] = []
    lay_loop(strips, metas, 0.0)
    if not strips:
        return []

    # ---- 整體置中平移（D3）：Δ = v_hi_last − v_max > 0 → 全部 v −= Δ/2 ----
    delta = strips[-1].v_hi_m - v_max
    if delta > tol_v:
        shift = delta / 2.0
        relay_from: int | None = None
        for l, (s, m) in enumerate(zip(strips, metas)):
            v_lo = s.v_lo_m - shift
            B_lo, B_hi = m.B_lo - shift, m.B_hi - shift
            sub = clip_band(cell_poly, origin, alpha_rad, v_lo, v_lo + s.width_m)
            if sub is None:
                relay_from = l
                break
            u_lo, u_hi = u_extent(sub)
            h_min2, agl_max2, feas2, z_op2 = call_env(u_lo, u_hi, v_lo + s.width_m / 2.0, B_lo, B_hi)
            if not math.isfinite(h_min2):
                relay_from = l  # 閉包回傳非有限值：保守重鋪
                break
            if not math.isfinite(agl_max2):
                agl_max2 = h_min2
            if feas2 and s.feasible and h_min2 < s.h_min_m - eps_h_m:
                relay_from = l  # h_min 下降 → 自該條起重鋪
                break
            if s.feasible and not feas2:
                relay_from = l
                break
            z_op = z_op2 if z_op2 is not None else (s.z_op_m if s.z_op_m != h_op_m else None)
            strips[l] = make_strip(s.index, u_lo, u_hi, v_lo, s.width_m, s.spacing_m, min(s.h_min_m, h_min2) if s.feasible else s.h_min_m,
                                   agl_max2, s.feasible, s.infeasible_reason, z_op)
            metas[l] = _StripMeta(B_lo, B_hi, m.h_B)
        if relay_from is not None:
            v_base = strips[relay_from].v_lo_m - shift if relay_from < len(strips) else 0.0
            del strips[relay_from:]
            del metas[relay_from:]
            lay_loop(strips, metas, v_base)

    # ---- 重新編號並後置斷言（規格 §7.7） ----
    for k, s in enumerate(strips):
        s.index = k + 1
        s.strip_id = f"{cell_id}_{cls}_{k + 1}"
    fp = strips_union_footprint(strips)
    if not fp.buffer(_FOOTPRINT_TOL_M).contains(cell_poly):
        missing = cell_poly.difference(fp).area
        raise AssertionError(f"lay_strips post-condition violated: footprint does not cover cell {cell_id} (uncovered area {missing:.3f} m²)")
    return strips


def residual_subcell(cell_poly: Polygon, alpha_rad: float, origin: np.ndarray, v_top_m: float,
                     delta_edge_m: float) -> Polygon | MultiPolygon | None:
    """殘帶子胞（規格 §7.5，D17）：clip(cell, v ≥ v_top − δ_edge)；只在 v 方向外擴 δ_edge，u 方向沿用胞邊界。"""
    if cell_poly is None or cell_poly.is_empty:
        return None
    uv = to_sweep_coords(_coords(cell_poly), origin, alpha_rad)
    v_max = float(uv[:, 1].max())
    v_lo = float(v_top_m) - float(delta_edge_m)
    if v_lo >= v_max - 1e-9:
        return None
    return clip_band(cell_poly, origin, alpha_rad, v_lo, v_max + 1.0)


def strips_union_footprint(strips: list[Strip]) -> Polygon | MultiPolygon:
    """以 width_m（以中心線 p^L→p^R 為軸、沿 ±n̂(θ_LR) 各 w/2）建帶之 union（覆蓋驗證用）。"""
    if not strips:
        return Polygon()
    pL = np.array([s.pL for s in strips], dtype=float)
    pR = np.array([s.pR for s in strips], dtype=float)
    th = np.array([s.theta_LR_rad for s in strips], dtype=float)
    hw = np.array([s.width_m for s in strips], dtype=float)[:, None] / 2.0
    n_hat = np.stack([np.cos(th), -np.sin(th)], axis=1)
    corners = np.stack([pL - hw * n_hat, pR - hw * n_hat, pR + hw * n_hat, pL + hw * n_hat], axis=1)  # (k,4,2)
    rects = shapely.polygons(corners)
    result = shapely.union_all(rects)
    parts = _polygons_of(result)
    if len(parts) == 1:
        return parts[0]
    return MultiPolygon(parts) if parts else Polygon()
