"""hv_dccpp/viz.py — 契約 §15：任務計畫視覺化（matplotlib，Agg 後端）。

公開函式（契約簽章）：

* :func:`plot_2d`        平面總覽：地形（山影 + 等高線）、區域、胞、條帶（A／B 兩色）、keep-out／禁飛區、
                         各機 3D 航線之平面投影（依 ``Phase`` 著色）、home／備降點、風向箭頭。
* :func:`plot_profiles`  每機剖面：ENU z 對沿航跡距離、地形、``z_lo``／``z_hi`` 帶（規格 §11.1 之近似，見
                         :func:`profile_bounds`）、階段底色。
* :func:`plot_gantt`     甘特圖：每機階段對時間 ＋ 走廊預約（起飛／降落；`CorridorSchedule` 或由 `plan_windows` 推得）。
* :func:`plot_coverage`  覆蓋柵格（`sim.CoverageReport` 鴨子型別：`area_id / cls_required / coverage_by_cls /
                         uncovered_cells_xy / raster_m`）。
* :func:`save_all`       以上全部（另加 :func:`plot_energy`）→ PNG 路徑清單。

契約外附加（皆記於 CONTRACT_DEVIATIONS.md）：:func:`resolve_coverage`（report → `sim.coverage_raster` →
:func:`coverage_from_plan`）、:func:`coverage_from_plan`（由計畫足跡近似覆蓋率，回傳 :class:`CoverageLite`；sim 不可用
時之後備）、:func:`profile_bounds`、:func:`plot_energy`、:func:`configure_fonts`。

慣例：所有座標為 ENU m（契約 §0）；``Waypoint.track_deg`` 為離開該點航段之航跡方位；圖上文字一律英文，
字型優先 Microsoft JhengHei（若安裝）否則 DejaVu Sans，避免缺字警告。
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPolygon, Polygon

import matplotlib

matplotlib.use("Agg", force=False)
import matplotlib.pyplot as plt  # noqa: E402  (backend must be selected first)
from matplotlib import font_manager, rcParams  # noqa: E402
from matplotlib.collections import LineCollection, PolyCollection  # noqa: E402
from matplotlib.colors import LightSource, ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch, PathPatch  # noqa: E402
from matplotlib.path import Path as MplPath  # noqa: E402

from .context import MissionContext  # noqa: E402
from .deconflict import CorridorSchedule, plan_windows  # noqa: E402
from .geometry import Strip  # noqa: E402
from .mission import FW_PHASES, MissionPlan, Phase, PlannerConfig, UAVPlan, Waypoint  # noqa: E402

__all__ = [
    "plot_2d", "plot_profiles", "plot_gantt", "plot_coverage", "save_all",
    "plot_energy", "coverage_from_plan", "CoverageLite", "resolve_coverage", "profile_bounds", "configure_fonts",
    "PHASE_COLORS", "CLASS_COLORS",
]

# ---------------------------------------------------------------------------
# 顏色 / 字型
# ---------------------------------------------------------------------------

PHASE_COLORS: dict[Phase, str] = {
    Phase.TAKEOFF: "#7f7f7f",
    Phase.TRANS_OUT: "#9467bd",
    Phase.ACCEL: "#8c564b",
    Phase.CLIMB: "#17becf",
    Phase.ORBIT: "#bcbd22",
    Phase.ENTRY: "#1f77b4",
    Phase.OPERATE: "#2ca02c",
    Phase.TRANSFER: "#ff7f0e",
    Phase.RTB: "#d62728",
    Phase.DESCENT: "#0b6f7a",
    Phase.DECEL: "#8c564b",
    Phase.TRANS_IN: "#9467bd",
    Phase.LAND: "#7f7f7f",
    Phase.HOLD: "#e377c2",
}
"""階段 → 顏色（航線投影、剖面、甘特圖共用）。"""

CLASS_COLORS: dict[str, str] = {"A": "#1f4e9c", "B": "#d9822b"}
"""構型 → 顏色（條帶、子胞、機號標籤）。"""

_FONT_PREFERENCE: tuple[str, ...] = ("Microsoft JhengHei", "Noto Sans TC", "Noto Sans CJK TC", "DejaVu Sans")
_FONTS_CONFIGURED = False
_DEFAULT_DPI = 130
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

_CORRIDOR_PHASES: frozenset[Phase] = frozenset({Phase.TAKEOFF, Phase.LAND, Phase.TRANS_OUT, Phase.TRANS_IN})
_CLR_ONLY_PHASES: frozenset[Phase] = frozenset({Phase.CLIMB, Phase.DESCENT, Phase.ACCEL, Phase.DECEL})


def configure_fonts(force: bool = False) -> list[str]:
    """設定 CJK 安全字型後備串（Microsoft JhengHei → Noto Sans TC → DejaVu Sans；只保留已安裝者）。

    回傳實際使用之 ``font.sans-serif`` 串。只執行一次（``force=True`` 重設）。
    """
    global _FONTS_CONFIGURED
    if _FONTS_CONFIGURED and not force:
        return list(rcParams["font.sans-serif"])
    try:
        available = {f.name for f in font_manager.fontManager.ttflist}
    except Exception:  # pragma: no cover - 字型快取損壞時退回預設
        available = set()
    chosen = [name for name in _FONT_PREFERENCE if name in available]
    if "DejaVu Sans" not in chosen:
        chosen.append("DejaVu Sans")
    rcParams["font.family"] = "sans-serif"
    rcParams["font.sans-serif"] = chosen
    rcParams["axes.unicode_minus"] = False
    _FONTS_CONFIGURED = True
    return chosen


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def _ensure_parent(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)


def _save(fig: Any, path: str, dpi: int = _DEFAULT_DPI) -> None:
    _ensure_parent(path)
    fig.savefig(path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def _iter_polys(geom: Any) -> list[Polygon]:
    """shapely 幾何 → Polygon 清單（Multi*／GeometryCollection 展開；空／None → []）。"""
    if geom is None:
        return []
    if getattr(geom, "is_empty", False):
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if hasattr(geom, "geoms"):
        out: list[Polygon] = []
        for g in geom.geoms:
            out.extend(_iter_polys(g))
        return out
    return []


def _poly_patch(poly: Polygon, **kw: Any) -> PathPatch:
    """含洞多邊形 → PathPatch（外環 + 內環皆以 MOVETO…CLOSEPOLY 編碼）。"""
    verts: list[list[float]] = []
    codes: list[int] = []
    for ring in [poly.exterior, *poly.interiors]:
        xy = np.asarray(ring.coords, dtype=float)[:, :2]
        if len(xy) < 3:
            continue
        verts.extend(xy.tolist())
        codes.extend([MplPath.MOVETO] + [MplPath.LINETO] * (len(xy) - 2) + [MplPath.CLOSEPOLY])
    if not verts:
        verts = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        codes = [MplPath.MOVETO, MplPath.LINETO, MplPath.CLOSEPOLY]
    return PathPatch(MplPath(np.asarray(verts), codes), **kw)


def _draw_geom(ax: Any, geom: Any, *, face: str | None = None, edge: str = "k", lw: float = 1.0, ls: str = "-",
               alpha: float = 1.0, hatch: str | None = None, zorder: float = 2.0, label: str | None = None,
               marker: str = "o", ms: float = 5.0) -> None:
    """任意 shapely 幾何 → 面（PathPatch）／線／點；``label`` 只附於第一個元素。"""
    if geom is None or getattr(geom, "is_empty", True):
        return
    polys = _iter_polys(geom)
    if polys:
        for k, p in enumerate(polys):
            ax.add_patch(_poly_patch(p, facecolor=face if face is not None else "none", edgecolor=edge, lw=lw,
                                     ls=ls, alpha=alpha, hatch=hatch, zorder=zorder,
                                     label=label if k == 0 else None))
        return
    gtype = geom.geom_type
    if gtype in ("LineString", "LinearRing"):
        xy = np.asarray(geom.coords, dtype=float)
        ax.plot(xy[:, 0], xy[:, 1], color=edge, lw=lw, ls=ls, alpha=alpha, zorder=zorder, label=label)
    elif gtype == "Point":
        ax.plot([geom.x], [geom.y], marker=marker, ms=ms, color=edge, ls="none", alpha=alpha, zorder=zorder,
                label=label)
    elif hasattr(geom, "geoms"):
        for k, g in enumerate(geom.geoms):
            _draw_geom(ax, g, face=face, edge=edge, lw=lw, ls=ls, alpha=alpha, hatch=hatch, zorder=zorder,
                       label=label if k == 0 else None, marker=marker, ms=ms)


def _wp_xy(wps: Sequence[Waypoint]) -> np.ndarray:
    return np.array([[w.x_m, w.y_m] for w in wps], dtype=float).reshape(-1, 2)


def _phase_runs(wps: Sequence[Waypoint]) -> list[tuple[Phase, int, int]]:
    """相鄰同階段航點之連續區段 ``(phase, i_first, i_last)``（含端點索引）。"""
    runs: list[tuple[Phase, int, int]] = []
    if not wps:
        return runs
    start = 0
    for i in range(1, len(wps) + 1):
        if i == len(wps) or wps[i].phase is not wps[start].phase:
            runs.append((wps[start].phase, start, i - 1))
            start = i
    return runs


def _sorted_plans(plan: MissionPlan) -> list[UAVPlan]:
    return [plan.plans[k] for k in sorted(plan.plans, key=lambda u: (plan.plans[u].cls, u))]


def _home_id(ctx: MissionContext) -> str:
    try:
        return str(ctx.inputs.home.site_id)
    except Exception:  # pragma: no cover - 防禦
        return next(iter(ctx.sites_enu), "home")


def _phase_legend(phases: Sequence[Phase]) -> list[Line2D]:
    order = list(Phase)
    present = sorted(set(phases), key=order.index)
    return [Line2D([0], [0], color=PHASE_COLORS[p], lw=2.5, label=p.name) for p in present]


def _empty_figure(path: str, message: str, title: str) -> None:
    configure_fonts()
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.text(0.5, 0.5, message, ha="center", va="center", fontsize=14, transform=ax.transAxes)
    ax.set_axis_off()
    ax.set_title(title)
    _save(fig, path)


# ---------------------------------------------------------------------------
# 地形背景
# ---------------------------------------------------------------------------

def _roi(plan: MissionPlan, ctx: MissionContext, show_keepouts: bool) -> tuple[float, float, float, float]:
    """感興趣範圍：區域、站點、航點、keep-out 之聯集外框 ⊕ 邊界（≥300 m 或 5%）。"""
    xs: list[float] = []
    ys: list[float] = []
    for poly in ctx.areas_enu.values():
        b = poly.bounds
        xs += [b[0], b[2]]
        ys += [b[1], b[3]]
    for xy, _z in ctx.sites_enu.values():
        xs.append(float(xy[0]))
        ys.append(float(xy[1]))
    for up in plan.plans.values():
        if up.waypoints:
            p = _wp_xy(up.waypoints)
            xs += [float(p[:, 0].min()), float(p[:, 0].max())]
            ys += [float(p[:, 1].min()), float(p[:, 1].max())]
    for s in plan.strips.values():
        xs += [float(s.pL[0]), float(s.pR[0])]
        ys += [float(s.pL[1]), float(s.pR[1])]
    if show_keepouts and ctx.keepouts is not None:
        for poly in getattr(ctx.keepouts, "polys", []):
            b = poly.bounds
            xs += [b[0], b[2]]
            ys += [b[1], b[3]]
    if not xs:
        return (-1000.0, -1000.0, 1000.0, 1000.0)
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    dx, dy = max(xmax - xmin, 1.0), max(ymax - ymin, 1.0)
    m = max(300.0, 0.05 * max(dx, dy))
    return (xmin - m, ymin - m, xmax + m, ymax + m)


def _terrain_window(terrain: Any, bbox: tuple[float, float, float, float]
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """地形節點子陣列 ``(xs, ys, z_raw)``（bbox ⊕ 1 節點）；與網格無交集 → None。"""
    xs_all, ys_all = terrain.xs, terrain.ys
    i0 = int(np.searchsorted(xs_all, bbox[0]) - 1)
    i1 = int(np.searchsorted(xs_all, bbox[2]) + 1)
    j0 = int(np.searchsorted(ys_all, bbox[1]) - 1)
    j1 = int(np.searchsorted(ys_all, bbox[3]) + 1)
    i0, j0 = max(i0, 0), max(j0, 0)
    i1, j1 = min(i1, len(xs_all) - 1), min(j1, len(ys_all) - 1)
    if i1 - i0 < 1 or j1 - j0 < 1:
        return None
    z = np.asarray(terrain.z_raw, dtype=float)[j0:j1 + 1, i0:i1 + 1]
    return xs_all[i0:i1 + 1], ys_all[j0:j1 + 1], z


def _hillshade(z: np.ndarray, step_m: float, vert_exag: float = 3.0, azdeg: float = 315.0, altdeg: float = 45.0
               ) -> np.ndarray:
    """山影強度 ``(ny, nx)`` ∈ [0, 1]；nan 節點以平均值填補後計算並再標回 nan。"""
    z = np.asarray(z, dtype=float)
    mask = ~np.isfinite(z)
    if mask.all():
        return np.full(z.shape, np.nan)
    zf = z.copy()
    if mask.any():
        zf[mask] = float(np.nanmean(z))
    ls = LightSource(azdeg=azdeg, altdeg=altdeg)
    shade = ls.hillshade(zf, vert_exag=vert_exag, dx=step_m, dy=step_m)
    shade = np.asarray(shade, dtype=float)
    shade[mask] = np.nan
    return shade


def _draw_terrain(ax: Any, terrain: Any, bbox: tuple[float, float, float, float]) -> bool:
    """山影 + 等高線背景；回傳是否有可用地形。"""
    win = _terrain_window(terrain, bbox)
    if win is None:
        return False
    xs, ys, z = win
    if not np.isfinite(z).any():
        return False
    step = float(terrain.step_m)
    h = step / 2.0
    extent = (float(xs[0]) - h, float(xs[-1]) + h, float(ys[0]) - h, float(ys[-1]) + h)
    zmin, zmax = float(np.nanmin(z)), float(np.nanmax(z))
    # 高程色階（gist_earth）+ 山影
    img = ax.imshow(np.ma.masked_invalid(z), extent=extent, origin="lower", cmap="gist_earth", alpha=0.55,
                    interpolation="nearest", zorder=0, vmin=zmin - 0.25 * max(zmax - zmin, 1.0), vmax=zmax + 1.0)
    shade = _hillshade(z, step)
    ax.imshow(np.ma.masked_invalid(shade), extent=extent, origin="lower", cmap="gray", vmin=0.0, vmax=1.0,
              alpha=0.35, interpolation="nearest", zorder=0.5)
    if zmax - zmin > 0.5:
        n_levels = 10
        levels = np.linspace(zmin, zmax, n_levels + 2)[1:-1]
        try:
            cs = ax.contour(xs, ys, np.where(np.isfinite(z), z, zmin), levels=levels, colors="k", alpha=0.35,
                            linewidths=0.4, zorder=1)
            ax.clabel(cs, fmt="%.0f", fontsize=6, inline=True)
        except Exception:  # pragma: no cover - 極端資料（全常數等）
            pass
    cbar = ax.figure.colorbar(img, ax=ax, location="bottom", fraction=0.035, pad=0.09, shrink=0.45, aspect=40)
    cbar.set_label("terrain ENU z (m)", fontsize=8)
    cbar.ax.tick_params(labelsize=7)
    return True


# ---------------------------------------------------------------------------
# plot_2d
# ---------------------------------------------------------------------------

def _draw_strips(ax: Any, plan: MissionPlan) -> None:
    used: set[str] = set()
    for up in plan.plans.values():
        used.update(sid for sid, _d in up.strips)
    quads: dict[str, list[np.ndarray]] = {}
    for sid, s in plan.strips.items():
        col = CLASS_COLORS.get(s.cls, "#444444")
        pL = np.asarray(s.pL, dtype=float)
        pR = np.asarray(s.pR, dtype=float)
        th = float(s.theta_LR_rad)
        n_hat = np.array([math.cos(th), -math.sin(th)])
        hw = 0.5 * float(s.width_m)
        quads.setdefault(s.cls, []).append(np.array([pL - hw * n_hat, pR - hw * n_hat, pR + hw * n_hat,
                                                     pL + hw * n_hat]))
        assigned = sid in used
        ax.plot([pL[0], pR[0]], [pL[1], pR[1]], color=col, lw=1.0 if assigned else 0.6,
                ls="-" if assigned else (0, (2, 2)), alpha=0.95 if assigned else 0.5, zorder=3)
        if len(plan.strips) <= 60:
            mid = 0.5 * (pL + pR)
            ax.text(mid[0], mid[1], str(s.index), fontsize=5, color=col, ha="center", va="center", zorder=4,
                    alpha=0.8)
    for cls, qs in quads.items():
        ax.add_collection(PolyCollection(qs, facecolors=CLASS_COLORS.get(cls, "#444444"), edgecolors="none",
                                         alpha=0.07, zorder=2.5))


def _draw_cells(ax: Any, plan: MissionPlan) -> None:
    for cell in plan.cells:
        is_sub = cell.parent_cell_id is not None
        classes = [c for c, st in cell.strips.items() if st]
        col = CLASS_COLORS.get(classes[0], "#333333") if (is_sub and len(classes) == 1) else "#222222"
        _draw_geom(ax, cell.polygon, face=None, edge=col, lw=0.8 if is_sub else 1.2,
                   ls=(0, (1, 2)) if is_sub else "--", alpha=0.9, zorder=3)
        if not is_sub:
            try:
                c = cell.polygon.representative_point()
                ax.text(c.x, c.y, cell.cell_id, fontsize=7, color="#222222", ha="center", va="center",
                        zorder=4, bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.6))
            except Exception:  # pragma: no cover
                pass


def _draw_keepouts(ax: Any, ctx: MissionContext) -> list[Any]:
    handles: list[Any] = []
    drew_obs = drew_nf = False
    ko = ctx.keepouts
    if ko is not None:
        polys = list(getattr(ko, "polys", []))
        kinds = list(getattr(ko, "kinds", [])) or ["obstacle"] * len(polys)
        ids = list(getattr(ko, "ids", [])) or [""] * len(polys)
        for poly, kind, kid in zip(polys, kinds, ids):
            if kind == "nofly":
                _draw_geom(ax, poly, face="#ff00ff", edge="#aa00aa", lw=1.0, alpha=0.18, hatch="//", zorder=2.2)
                drew_nf = True
            else:
                _draw_geom(ax, poly, face="#d62728", edge="#8b0000", lw=0.8, alpha=0.25, zorder=2.2)
                drew_obs = True
            try:
                c = poly.representative_point()
                ax.text(c.x, c.y, str(kid), fontsize=6, color="#8b0000" if kind != "nofly" else "#aa00aa",
                        ha="center", va="center", zorder=4)
            except Exception:  # pragma: no cover
                pass
    for poly in ctx.nofly_buffered:
        _draw_geom(ax, poly, face=None, edge="#aa00aa", lw=0.8, ls=(0, (3, 2)), alpha=0.9, zorder=2.3)
        drew_nf = True
    for ob in ctx.obstacles_enu:
        _draw_geom(ax, ob.geom, face="#8b0000", edge="#8b0000", lw=1.2, alpha=0.9, zorder=2.4, marker="^", ms=4)
        drew_obs = True
    if drew_obs:
        handles.append(Patch(facecolor="#d62728", edgecolor="#8b0000", alpha=0.4, label="keep-out (obstacle + b_h)"))
    if drew_nf:
        handles.append(Patch(facecolor="#ff00ff", edgecolor="#aa00aa", alpha=0.3, hatch="//", label="no-fly (+ b_trk)"))
    return handles


def _draw_sites(ax: Any, ctx: MissionContext) -> list[Any]:
    handles: list[Any] = []
    home = _home_id(ctx)
    for sid, (xy, _z) in ctx.sites_enu.items():
        is_home = sid == home
        is_rec = sid == ctx.recovery_site_id
        marker = "*" if is_home else "^"
        col = "#000000" if is_home else "#2b8a3e"
        ax.plot([xy[0]], [xy[1]], marker=marker, ms=16 if is_home else 11, color=col, mec="white", mew=0.8,
                ls="none", zorder=8)
        tag = sid + (" (home)" if is_home else "") + (" [recovery]" if (is_rec and not is_home) else "")
        ax.annotate(tag, (xy[0], xy[1]), xytext=(6, 6), textcoords="offset points", fontsize=8, zorder=9,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.7))
    gs = np.asarray(ctx.ground_station_xy, dtype=float).reshape(-1)
    if gs.size == 2 and ctx.sites_enu:
        hxy = ctx.sites_enu[home][0] if home in ctx.sites_enu else next(iter(ctx.sites_enu.values()))[0]
        if float(np.hypot(gs[0] - hxy[0], gs[1] - hxy[1])) > 1.0:
            ax.plot([gs[0]], [gs[1]], marker="s", ms=8, color="#555555", ls="none", zorder=8)
            ax.annotate("GCS", (gs[0], gs[1]), xytext=(6, -10), textcoords="offset points", fontsize=8)
            handles.append(Line2D([0], [0], marker="s", color="#555555", ls="none", label="ground station"))
    handles.insert(0, Line2D([0], [0], marker="*", color="#000000", ms=12, ls="none", label="home"))
    if any(sid != home for sid in ctx.sites_enu):
        handles.insert(1, Line2D([0], [0], marker="^", color="#2b8a3e", ms=9, ls="none", label="alternate site"))
    return handles


def _draw_paths(ax: Any, plan: MissionPlan, conflicts: Sequence[Any] | None) -> list[Phase]:
    phases: list[Phase] = []
    for up in _sorted_plans(plan):
        wps = up.waypoints
        if len(wps) < 2:
            continue
        xy = _wp_xy(wps)
        segs = np.stack([xy[:-1], xy[1:]], axis=1)
        cols = [PHASE_COLORS.get(w.phase, "#000000") for w in wps[:-1]]
        ax.add_collection(LineCollection(segs, colors=cols, linewidths=1.4, zorder=5, capstyle="round"))
        phases.extend(w.phase for w in wps)
        # 機號：首個 OPERATE 航點（無則中點）
        idx = next((i for i, w in enumerate(wps) if w.phase is Phase.OPERATE), len(wps) // 2)
        col = CLASS_COLORS.get(up.cls, "#000000")
        ax.annotate(up.uav_id, (xy[idx, 0], xy[idx, 1]), xytext=(4, 4), textcoords="offset points", fontsize=8,
                    color=col, fontweight="bold", zorder=9,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec=col, alpha=0.8))
        ax.plot([xy[0, 0]], [xy[0, 1]], marker="o", ms=4, color=col, ls="none", zorder=7)
        ax.plot([xy[-1, 0]], [xy[-1, 1]], marker="x", ms=5, color=col, ls="none", zorder=7)
    if conflicts:
        pts = []
        for c in conflicts:
            uid = getattr(c, "uav_a", None)
            up = plan.plans.get(uid) if uid is not None else None
            if up is None or len(up.waypoints) < 2:
                continue
            t = np.array([w.t_s for w in up.waypoints])
            xy = _wp_xy(up.waypoints)
            tc = float(getattr(c, "t_s", np.nan))
            if not np.isfinite(tc):
                continue
            pts.append((float(np.interp(tc, t, xy[:, 0])), float(np.interp(tc, t, xy[:, 1]))))
        if pts:
            arr = np.asarray(pts)
            ax.plot(arr[:, 0], arr[:, 1], marker="X", ms=9, color="#ff0000", mec="black", ls="none", zorder=10,
                    label="conflict")
    return phases


def _draw_wind(ax: Any, ctx: MissionContext, bbox: tuple[float, float, float, float], z_ref: float) -> None:
    try:
        w = np.asarray(ctx.wind.wind_vec(float(z_ref)), dtype=float).reshape(-1, 2)[0]
        speed = float(np.hypot(w[0], w[1]))
        dir_from_deg = math.degrees(float(ctx.wind.dir_from_at(float(z_ref)))) % 360.0
    except Exception:  # pragma: no cover - 無風場時
        return
    dx, dy = bbox[2] - bbox[0], bbox[3] - bbox[1]
    L = 0.08 * max(dx, dy)
    ox, oy = bbox[0] + 0.10 * dx, bbox[3] - 0.10 * dy
    if speed > 1e-9:
        u = w / speed
        ax.annotate("", xy=(ox + L * u[0], oy + L * u[1]), xytext=(ox, oy),
                    arrowprops=dict(arrowstyle="-|>", color="#0b3d91", lw=2.0, shrinkA=0, shrinkB=0), zorder=11)
    ax.text(ox, oy - 0.03 * dy, f"wind {speed:.1f} m/s from {dir_from_deg:.0f} deg @ z={z_ref:.0f} m",
            fontsize=8, color="#0b3d91", ha="center", va="top", zorder=11,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#0b3d91", alpha=0.85))


def plot_2d(plan: MissionPlan, ctx: MissionContext, path: str, show_strips: bool = True,
            show_keepouts: bool = True, *, conflicts: Sequence[Any] | None = None, dpi: int = _DEFAULT_DPI) -> None:
    """平面總覽（契約 §15）：地形山影／等高線、區域（含洞）、胞（虛線；殘帶子胞點線）、條帶（A 藍／B 橙，
    已指派實線、未指派虛線，足跡淡色）、keep-out／禁飛區、各機航線投影（依 Phase 著色）、home／備降點、風向箭頭。

    ``conflicts``（契約外關鍵字）：`deconflict.Conflict` 序列 → 以紅色 X 標於 uav_a 在 ``t_s`` 之內插位置。
    """
    configure_fonts()
    bbox = _roi(plan, ctx, show_keepouts)
    fig, ax = plt.subplots(figsize=(12.5, 8.5))
    _draw_terrain(ax, ctx.terrain, bbox)
    handles: list[Any] = []
    # 區域
    for k, (aid, poly) in enumerate(ctx.areas_enu.items()):
        _draw_geom(ax, poly, face="#ffd166", edge="#b8860b", lw=1.6, alpha=0.18, zorder=2)
        _draw_geom(ax, poly, face=None, edge="#b8860b", lw=1.6, zorder=2.1)
        try:
            b = poly.bounds
            ax.text(b[0], b[3], f" {aid}", fontsize=9, color="#7a5c00", ha="left", va="bottom", zorder=4)
        except Exception:  # pragma: no cover
            pass
    handles.append(Patch(facecolor="#ffd166", edgecolor="#b8860b", alpha=0.4, label="survey area"))
    _draw_cells(ax, plan)
    handles.append(Line2D([0], [0], color="#222222", ls="--", label="cell"))
    if show_strips and plan.strips:
        _draw_strips(ax, plan)
        for cls in sorted({s.cls for s in plan.strips.values()}):
            handles.append(Line2D([0], [0], color=CLASS_COLORS.get(cls, "#444444"), lw=1.5, label=f"strips {cls}"))
    if show_keepouts:
        handles.extend(_draw_keepouts(ax, ctx))
    # 鏈路範圍（若有限）
    for cls, lp in ctx.link_poly.items():
        try:
            if lp is not None and not lp.is_empty and lp.area < 1e11:
                _draw_geom(ax, lp, face=None, edge="#1a9850" if cls == "A" else "#91cf60", lw=0.8,
                           ls=(0, (6, 3)), alpha=0.8, zorder=2.6)
                handles.append(Line2D([0], [0], color="#1a9850" if cls == "A" else "#91cf60", ls="--",
                                      label=f"C2 link {cls}"))
        except Exception:  # pragma: no cover
            pass
    phases = _draw_paths(ax, plan, conflicts)
    handles.extend(_draw_sites(ax, ctx))
    if conflicts:
        handles.append(Line2D([0], [0], marker="X", color="#ff0000", mec="black", ms=9, ls="none", label="conflict"))
    # 風向箭頭：參考高度 = OPERATE 航點 z 中位數（無則站高 + 150）
    z_ops = [w.z_m for up in plan.plans.values() for w in up.waypoints if w.phase is Phase.OPERATE]
    if z_ops:
        z_ref = float(np.median(z_ops))
    else:
        z_ref = float(next(iter(ctx.sites_enu.values()))[1]) + 150.0 if ctx.sites_enu else 150.0
    _draw_wind(ax, ctx, bbox, z_ref)
    ax.set_xlim(bbox[0], bbox[2])
    ax.set_ylim(bbox[1], bbox[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(True, alpha=0.2)
    n_uav = len(plan.plans)
    status = getattr(plan.feasibility, "status", "?")
    ax.set_title(f"HV-DCCPP plan overview: {n_uav} UAV, makespan {plan.makespan_s / 60.0:.1f} min, "
                 f"{len(plan.strips)} strips, status {status}")
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8, title="map", frameon=True)
    ph = _phase_legend(phases)
    if ph:
        fig.legend(handles=ph, loc="upper center", bbox_to_anchor=(0.5, 0.02), ncol=min(len(ph), 7), fontsize=8,
                   title="phase", frameon=True)
    _save(fig, path, dpi)


# ---------------------------------------------------------------------------
# plot_profiles
# ---------------------------------------------------------------------------

def _sensor_h_max(ctx: MissionContext, cls: str, req_id: str | None) -> float:
    """(cls, req_id) 感測器之 AGL 上限 ``h_max``（`Sensor.agl_window()[1]`）；找不到 → inf。"""
    if req_id is None:
        return math.inf
    sensors = getattr(ctx, "sensors", {}) or {}
    for key in ((cls, req_id), (cls, req_id + "_rgb")):
        s = sensors.get(key)
        if s is not None:
            try:
                h = float(s.agl_window()[1])
                return h if np.isfinite(h) and h > 0 else math.inf
            except Exception:
                return math.inf
    return math.inf


def profile_bounds(uplan: UAVPlan, ctx: MissionContext, strips: Mapping[str, Strip] | None = None,
                   cells: Sequence[Any] | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """每航點之 ``(z_g, z_lo, z_hi)``（ENU；規格 §11.1 之視覺化近似，契約外附加）。

    * ``z_g``：航點處自然地形（``terrain.raw_view().z_at``；AGL 界線與 §11.1 同基準，障礙淨空另計）。
    * 走廊 ``b``：OPER = ``w_l/2``、其餘 = ``cfg.b_trk_m``；``z_g^max/min`` 以 ``terrain.corridor_max/min``（方位 =
      ``track_deg``）。
    * ``z_lo``：TAKEOFF/LAND/TRANS_OUT/TRANS_IN → ``z_g^max``（旋翼段）；CLIMB/DESCENT/ACCEL/DECEL →
      ``z_g^max + h_clr``（D35）；OPER → ``z_g^max + max(h_clr, h_min,l)``（D36）；其餘固定翼段 →
      ``z_g^max + max(h_clr, h_minfw)``。
    * ``z_hi``：``z_g^min + h_ceil``（OPER 另取 ``min(·, z_g^min + h_max·k_GSD)``，``h_max`` 由該胞
      ``payload_req_id`` 之感測器 `agl_window()` 取得，需給 ``cells``）；``h_ceil`` None → inf。
    地形未知處為 nan。
    """
    wps = uplan.waypoints
    n = len(wps)
    if n == 0:
        e = np.zeros(0)
        return e, e.copy(), e.copy()
    strips = strips if strips is not None else {}
    cell_req: dict[str, str] = {c.cell_id: str(c.payload_req_id) for c in (cells or [])}
    cfg = ctx.cfg
    spec = ctx.specs.get(uplan.cls)
    h_clr = float(ctx.h_clr_m)
    h_minfw = float(getattr(spec, "min_fw_operation_agl_m", 0.0)) if spec is not None else 0.0
    h_ceil = ctx.h_ceil_m.get(uplan.cls) if isinstance(ctx.h_ceil_m, Mapping) else None
    h_ceil_f = math.inf if h_ceil is None else float(h_ceil)
    k_gsd = float(getattr(cfg, "k_gsd", 1.0))
    xy = _wp_xy(wps)
    theta = np.radians(np.array([w.track_deg for w in wps], dtype=float))
    hw = np.full(n, float(cfg.b_trk_m))
    lo_add = np.full(n, h_clr)
    hi_add = np.full(n, h_ceil_f)
    h_max_cache: dict[str, float] = {}
    for i, w in enumerate(wps):
        ph = w.phase
        if ph in _CORRIDOR_PHASES:
            lo_add[i] = 0.0
        elif ph in _CLR_ONLY_PHASES:
            lo_add[i] = h_clr
        elif ph is Phase.OPERATE:
            s = strips.get(w.strip_id) if w.strip_id is not None else None
            if s is not None:
                hw[i] = 0.5 * float(s.width_m)
                lo_add[i] = max(h_clr, float(s.h_min_m))
                req_id = cell_req.get(s.cell_id)
                if req_id is not None:
                    if req_id not in h_max_cache:
                        h_max_cache[req_id] = _sensor_h_max(ctx, uplan.cls, req_id)
                    hi_add[i] = min(hi_add[i], k_gsd * h_max_cache[req_id])
            else:
                lo_add[i] = max(h_clr, h_minfw)
        elif ph in FW_PHASES:
            lo_add[i] = max(h_clr, h_minfw)
    dv = min(float(cfg.dem_step_m), 10.0)
    terrain = ctx.terrain.raw_view()             # AGL 界線以自然地形（同 altitude.sample_bounds / sim.check_kinematics）
    with np.errstate(invalid="ignore"):
        z_g = np.asarray(terrain.z_at(xy), dtype=float)
        z_max = np.asarray(terrain.corridor_max(xy, theta, hw, dv), dtype=float)
        z_min = np.asarray(terrain.corridor_min(xy, theta, hw, dv), dtype=float)
    z_lo = z_max + lo_add
    z_hi = z_min + hi_add
    return z_g, z_lo, z_hi


def _along_track(wps: Sequence[Waypoint]) -> np.ndarray:
    xy = _wp_xy(wps)
    if len(xy) == 0:
        return np.zeros(0)
    d = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return np.concatenate([[0.0], np.cumsum(d)])


def plot_profiles(plan: MissionPlan, ctx: MissionContext, path: str, *, dpi: int = _DEFAULT_DPI) -> None:
    """每機剖面（契約 §15）：ENU z 對沿航跡距離（km）、地形（燒入後）、``z_lo``／``z_hi`` 帶（:func:`profile_bounds`）、
    階段底色與著色航線、OPERATE 段之條帶編號。"""
    configure_fonts()
    plans = _sorted_plans(plan)
    if not plans:
        _empty_figure(path, "no UAV plans", "Altitude profiles")
        return
    n = len(plans)
    fig, axes = plt.subplots(n, 1, figsize=(12.5, 1.2 + 2.6 * n), squeeze=False)
    all_phases: list[Phase] = []
    for ax, up in zip(axes[:, 0], plans):
        wps = up.waypoints
        if len(wps) < 2:
            ax.text(0.5, 0.5, f"{up.uav_id}: no waypoints", ha="center", va="center", transform=ax.transAxes)
            ax.set_axis_off()
            continue
        s_km = _along_track(wps) / 1000.0
        z = np.array([w.z_m for w in wps], dtype=float)
        z_g, z_lo, z_hi = profile_bounds(up, ctx, plan.strips, plan.cells)
        all_phases.extend(w.phase for w in wps)
        # 階段底色
        runs = _phase_runs(wps)
        for ph, i0, i1 in runs:
            x0 = s_km[i0]
            x1 = s_km[i1 + 1] if i1 + 1 < len(s_km) else s_km[i1]
            if x1 > x0:
                ax.axvspan(x0, x1, color=PHASE_COLORS.get(ph, "#000000"), alpha=0.07, lw=0, zorder=0)
            if ph is Phase.OPERATE and wps[i0].strip_id:
                ax.text(0.5 * (x0 + x1), float(np.nanmax(z[i0:i1 + 1])) + 6.0,
                        f"{wps[i0].strip_id} ({'LR' if wps[i0].dir == -1 else 'RL' if wps[i0].dir == 1 else '?'})",
                        fontsize=6, ha="center", va="bottom", color=PHASE_COLORS[Phase.OPERATE], rotation=0, zorder=6)
        # 地形
        zg_plot = np.where(np.isfinite(z_g), z_g, np.nan)
        base = float(np.nanmin(np.concatenate([zg_plot[np.isfinite(zg_plot)], z]))) - 40.0 if np.isfinite(zg_plot).any() \
            else float(np.min(z)) - 40.0
        ax.fill_between(s_km, base, zg_plot, color="#c8a877", alpha=0.85, lw=0, zorder=1, label="terrain")
        ax.plot(s_km, zg_plot, color="#7a5c2e", lw=0.8, zorder=2)
        # z_lo / z_hi 帶
        lo = np.where(np.isfinite(z_lo), z_lo, np.nan)
        hi_f = np.where(np.isfinite(z_hi), z_hi, np.nan)
        top = float(np.nanmax(np.concatenate([z, lo[np.isfinite(lo)] if np.isfinite(lo).any() else z]))) + 40.0
        hi_draw = np.where(np.isfinite(hi_f), np.minimum(hi_f, top + 200.0), top + 200.0)
        ax.fill_between(s_km, lo, hi_draw, where=np.isfinite(lo), color="#9ecae1", alpha=0.25, lw=0, zorder=1.5,
                        label="feasible band [z_lo, z_hi]")
        ax.plot(s_km, lo, color="#3182bd", lw=0.7, ls="--", zorder=3, label="z_lo")
        if np.isfinite(hi_f).any():
            ax.plot(s_km, hi_f, color="#3182bd", lw=0.7, ls=":", zorder=3, label="z_hi")
        # 航線（階段著色）
        pts = np.column_stack([s_km, z])
        segs = np.stack([pts[:-1], pts[1:]], axis=1)
        ax.add_collection(LineCollection(segs, colors=[PHASE_COLORS.get(w.phase, "k") for w in wps[:-1]],
                                         linewidths=1.8, zorder=5))
        ax.plot(s_km, z, color="none", lw=0)  # 更新資料範圍
        ymax = max(top, float(np.nanmax(hi_f)) if np.isfinite(hi_f).any() and np.nanmax(hi_f) < top + 200 else top)
        ax.set_xlim(float(s_km[0]), float(s_km[-1]) if s_km[-1] > s_km[0] else float(s_km[0]) + 1e-3)
        ax.set_ylim(base, ymax)
        ax.set_ylabel("ENU z (m)")
        ax.grid(True, alpha=0.25)
        agl = np.array([w.agl_m for w in wps], dtype=float)
        ax.set_title(f"{up.uav_id} ({up.cls}): dist {up.distance_m / 1000.0:.1f} km, land {up.t_land_s / 60.0:.1f} min "
                     f"@ {up.landing_site}, AGL max {np.nanmax(agl):.0f} m, energy margin {up.energy_margin_wh:.0f} Wh",
                     fontsize=9, loc="left")
        # 時間標註：每個階段起點
        for ph, i0, _i1 in runs:
            if ph in (Phase.TAKEOFF, Phase.ENTRY, Phase.RTB, Phase.LAND) or ph is Phase.OPERATE and i0 == runs[0][1]:
                ax.annotate(f"{ph.name.lower()} t={wps[i0].t_s / 60.0:.1f} min", (s_km[i0], z[i0]),
                            xytext=(2, -12), textcoords="offset points", fontsize=6, color="#333333")
        ax.legend(loc="upper right", fontsize=7, ncol=4, framealpha=0.8)
    axes[-1, 0].set_xlabel("along-track distance (km)")
    ph = _phase_legend(all_phases)
    if ph:
        fig.legend(handles=ph, loc="lower center", ncol=min(len(ph), 7), fontsize=7, bbox_to_anchor=(0.5, -0.01),
                   title="phase")
    fig.suptitle(f"Altitude profiles ({n} UAV), makespan {plan.makespan_s / 60.0:.1f} min", fontsize=11)
    fig.tight_layout(rect=(0, 0.03, 1, 0.98))
    _save(fig, path, dpi)


# ---------------------------------------------------------------------------
# plot_gantt
# ---------------------------------------------------------------------------

def _reservations(plan: MissionPlan, schedule: CorridorSchedule | None, cfg: PlannerConfig | None,
                  takeoff_site_id: str | None) -> list[tuple[str, str, float, float, str]]:
    """走廊預約 ``(site_id, kind, t_start, t_end, uav_id)``；schedule None → 由 `plan_windows` 推得。"""
    out: list[tuple[str, str, float, float, str]] = []
    if schedule is not None:
        for r in getattr(schedule, "reservations", []):
            out.append((str(r.site_id), str(r.kind), float(r.t_start_s), float(r.t_end_s), str(r.uav_id)))
        return out
    cfg = cfg if cfg is not None else PlannerConfig()
    for up in plan.plans.values():
        (a0, a1), (l0, l1) = plan_windows(up, cfg)
        out.append((takeoff_site_id or "home", "takeoff", a0, a1, up.uav_id))
        out.append((str(up.landing_site), "landing", l0, l1, up.uav_id))
    return out


def plot_gantt(plan: MissionPlan, schedule: CorridorSchedule | None, path: str, *, cfg: PlannerConfig | None = None,
               takeoff_site_id: str | None = None, conflicts: Sequence[Any] | None = None,
               dpi: int = _DEFAULT_DPI) -> None:
    """甘特圖（契約 §15）：上半每機階段對時間（航點 ``t_s``；末段延伸至 ``t_land_s``），下半每（站, 種類）之走廊預約
    （同列重疊者以紅框斜線標示）。``schedule`` None → 以 `deconflict.plan_windows(plan, cfg or PlannerConfig())`
    推得（起飛站標籤 = ``takeoff_site_id`` 或 "home"）。``conflicts``：4D 衝突時刻以紅色虛線標示。"""
    configure_fonts()
    plans = _sorted_plans(plan)
    res = _reservations(plan, schedule, cfg, takeoff_site_id)
    if not plans and not res:
        _empty_figure(path, "no UAV plans", "Mission Gantt")
        return
    rows_res = sorted({(s, k) for s, k, *_ in res}, key=lambda t: (t[0], t[1] != "takeoff"))
    n_u, n_r = len(plans), len(rows_res)
    fig, ax = plt.subplots(figsize=(12.5, 1.5 + 0.55 * (n_u + n_r)))
    y = 0
    labels: list[str] = []
    yticks: list[float] = []
    phases_seen: list[Phase] = []
    t_max = float(plan.makespan_s)
    cls_of = {up.uav_id: up.cls for up in plans}
    for up in reversed(plans):  # 由下往上：第一機在最上
        wps = up.waypoints
        yticks.append(y)
        labels.append(f"{up.uav_id} ({up.cls})")
        if wps:
            runs = _phase_runs(wps)
            for ph, i0, i1 in runs:
                t0 = wps[i0].t_s / 60.0
                t1 = (wps[i1 + 1].t_s if i1 + 1 < len(wps) else max(up.t_land_s, wps[i1].t_s)) / 60.0
                ax.broken_barh([(t0, max(t1 - t0, 0.02))], (y - 0.35, 0.7), facecolors=PHASE_COLORS.get(ph, "k"),
                               edgecolors="none", zorder=3)
                phases_seen.append(ph)
            t_max = max(t_max, float(up.t_land_s), float(wps[-1].t_s))
            ax.text(up.t_land_s / 60.0, y, f" land {up.t_land_s / 60.0:.1f} min @ {up.landing_site}", fontsize=7,
                    va="center", ha="left", zorder=4)
            for w in up.warnings:
                if w.startswith("delay_orbit"):
                    ax.text(wps[0].t_s / 60.0, y + 0.36, w, fontsize=5, va="bottom", ha="left", color="#555555")
                    break
        y += 1
    if n_u and n_r:
        ax.axhline(y - 0.5, color="#888888", lw=0.8, ls="--")
    for site, kind in reversed(rows_res):
        yticks.append(y)
        labels.append(f"{site} {kind}")
        items = [(t0, t1, uid) for s, k, t0, t1, uid in res if s == site and k == kind]
        items.sort()
        for j, (t0, t1, uid) in enumerate(items):
            overlap = any(o[0] < t1 - 1e-9 and t0 < o[1] - 1e-9 for i2, o in enumerate(items) if i2 != j)
            col = CLASS_COLORS.get(cls_of.get(uid, ""), "#888888")
            ax.broken_barh([(t0 / 60.0, max(t1 - t0, 1.0) / 60.0)], (y - 0.3, 0.6), facecolors=col, alpha=0.45,
                           edgecolors="#ff0000" if overlap else "none", linewidth=1.5 if overlap else 0,
                           hatch="//" if overlap else None, zorder=3)
            ax.text(0.5 * (t0 + t1) / 60.0, y, uid, fontsize=7, ha="center", va="center", zorder=4)
            t_max = max(t_max, t1)
        y += 1
    if conflicts:
        seen = 0
        for c in conflicts:
            tc = float(getattr(c, "t_s", np.nan))
            if np.isfinite(tc):
                ax.axvline(tc / 60.0, color="#ff0000", lw=0.8, ls=":", zorder=2)
                seen += 1
        if seen:
            ax.plot([], [], color="#ff0000", lw=0.8, ls=":", label=f"conflicts ({seen})")
    if plan.makespan_s > 0:
        ax.axvline(plan.makespan_s / 60.0, color="#000000", lw=1.0, ls="--", zorder=2)
        ax.text(plan.makespan_s / 60.0, y - 0.5, f" makespan {plan.makespan_s / 60.0:.1f} min", fontsize=8,
                va="top", ha="left")
    ax.set_yticks(yticks)
    ax.set_yticklabels(labels, fontsize=8)
    ax.set_ylim(-0.6, max(y - 0.4, 0.6))
    ax.set_xlim(0.0, max(t_max / 60.0 * 1.08, 1.0))
    ax.set_xlabel("mission time (min)")
    ax.grid(True, axis="x", alpha=0.3)
    ax.set_title("Mission Gantt: phases per UAV and corridor reservations")
    handles = _phase_legend(phases_seen)
    handles += [Patch(facecolor=CLASS_COLORS[c], alpha=0.45, label=f"reservation {c}") for c in ("A", "B")
                if c in cls_of.values()]
    handles.append(Patch(facecolor="white", edgecolor="#ff0000", hatch="//", label="reservation overlap"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7)
    _save(fig, path, dpi)


# ---------------------------------------------------------------------------
# coverage
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class CoverageLite:
    """`sim.CoverageReport` 之同形替身（`sim.py` 尚未提供時由 :func:`coverage_from_plan` 產生）。"""
    area_id: str
    cls_required: list[str]
    coverage_by_cls: dict[str, float]
    uncovered_cells_xy: np.ndarray
    raster_m: float


_REQ_TO_CLS: dict[str, list[str]] = {"lidar": ["A"], "rgb": ["B"], "both": ["A", "B"], "any": ["any"]}


def _raster_centres(poly: Any, raster_m: float, max_cells: float = 4.0e6) -> tuple[np.ndarray, np.ndarray, float]:
    """區域外框之柵格中心 ``(X, Y)``（(ny, nx)）與實際步長（超過 max_cells 時放粗）。"""
    b = poly.bounds
    r = float(raster_m)
    span = max((b[2] - b[0]) * (b[3] - b[1]), 1.0)
    if span / (r * r) > max_cells:
        r = math.sqrt(span / max_cells)
    xs = np.arange(b[0] + 0.5 * r, b[2] + 0.5 * r, r)
    ys = np.arange(b[1] + 0.5 * r, b[3] + 0.5 * r, r)
    if xs.size == 0:
        xs = np.array([0.5 * (b[0] + b[2])])
    if ys.size == 0:
        ys = np.array([0.5 * (b[1] + b[3])])
    X, Y = np.meshgrid(xs, ys)
    return X, Y, r


def _sensor_swath(ctx: MissionContext, cls: str, req_id: str, agl: float) -> float | None:
    sensors = getattr(ctx, "sensors", {}) or {}
    for key in ((cls, req_id), (cls, req_id + "_rgb")):
        s = sensors.get(key)
        if s is not None:
            try:
                return float(s.swath_m(float(agl)))
            except Exception:
                return None
    return None


def coverage_from_plan(plan: MissionPlan, ctx: MissionContext, raster_m: float | None = None) -> list[CoverageLite]:
    """由計畫足跡近似覆蓋率（規格 §17.1 之簡化；契約外附加，`sim.coverage_raster` 可用時應改用之）。

    每機 ``sensor_on`` 且同 ``strip_id`` 之連續航點 → 折線 ⊕ ``w/2``（平端帽；``w`` = 該段最小 AGL 之感測器
    刈幅，找不到感測器則用 ``Strip.width_m``）；依構型聯集後與區域柵格（``raster_m`` 或 ``cfg.cov_raster_m``）比對。
    `sensor_req`：lidar → A、rgb → B、both → A 與 B 各算、any → 兩構型聯集（鍵 ``"any"``，另附 A／B 個別值）。
    ``uncovered_cells_xy`` = 任一必要構型未覆蓋之柵格中心。
    """
    r_default = float(raster_m) if raster_m is not None else float(getattr(ctx.cfg, "cov_raster_m", 5.0))
    cell_area: dict[str, str] = {c.cell_id: c.area_id for c in plan.cells}
    area_req: dict[str, tuple[str, str]] = {}
    for a in getattr(ctx.inputs, "areas", ()):
        area_req[a.area_id] = (str(a.sensor_req), str(a.payload_req_id))
    foot: dict[tuple[str, str], list[Any]] = {}
    for up in plan.plans.values():
        wps = up.waypoints
        i = 0
        n = len(wps)
        while i < n:
            if not wps[i].sensor_on or wps[i].strip_id is None:
                i += 1
                continue
            j = i
            while j + 1 < n and wps[j + 1].sensor_on and wps[j + 1].strip_id == wps[i].strip_id:
                j += 1
            sid = wps[i].strip_id
            s = plan.strips.get(sid)
            area_id = cell_area.get(s.cell_id) if s is not None else None
            if area_id is None:
                area_id = next((aid for aid in ctx.areas_enu if sid.startswith(aid)), None)
            if area_id is not None and j > i:
                run = wps[i:j + 1]
                xy = _wp_xy(run)
                req_id = area_req.get(area_id, ("any", "default"))[1]
                agl_min = float(np.nanmin([w.agl_m for w in run]))
                w_m = _sensor_swath(ctx, up.cls, req_id, agl_min)
                if w_m is None or not np.isfinite(w_m) or w_m <= 0:
                    w_m = float(s.width_m) if s is not None else 0.0
                line = LineString(xy)
                if w_m > 0 and line.length > 1e-6:
                    foot.setdefault((area_id, up.cls), []).append(line.buffer(0.5 * w_m, cap_style="flat"))
            i = j + 1
    reports: list[CoverageLite] = []
    for aid, poly in ctx.areas_enu.items():
        sensor_req, _req_id = area_req.get(aid, ("any", "default"))
        required = list(_REQ_TO_CLS.get(sensor_req, ["any"]))
        X, Y, r = _raster_centres(poly, r_default)
        inside = shapely.contains_xy(poly, X, Y)
        n_in = int(inside.sum())
        union_by_cls: dict[str, Any] = {}
        for cls in ("A", "B"):
            parts = foot.get((aid, cls), [])
            union_by_cls[cls] = shapely.union_all(parts) if parts else None
        all_parts = [g for g in union_by_cls.values() if g is not None]
        union_by_cls["any"] = shapely.union_all(all_parts) if all_parts else None
        cov: dict[str, float] = {}
        covered_mask: dict[str, np.ndarray] = {}
        for key in ("any", "A", "B"):
            g = union_by_cls[key]
            m = shapely.contains_xy(g, X, Y) & inside if g is not None else np.zeros_like(inside)
            covered_mask[key] = m
            cov[key] = float(m.sum()) / n_in if n_in > 0 else 0.0
        coverage_by_cls = {k: cov[k] for k in required}
        for k in ("A", "B"):
            if k not in coverage_by_cls and union_by_cls[k] is not None:
                coverage_by_cls[k] = cov[k]
        unc = inside.copy()
        if required:
            unc &= ~np.logical_and.reduce([covered_mask[k] for k in required])
        pts = np.column_stack([X[unc], Y[unc]]) if unc.any() else np.zeros((0, 2))
        reports.append(CoverageLite(aid, required, coverage_by_cls, pts, r))
    return reports


def plot_coverage(report: Sequence[Any], ctx: MissionContext, path: str, *, dpi: int = _DEFAULT_DPI) -> None:
    """覆蓋柵格（契約 §15）：每區一子圖；區域內柵格（``raster_m``）綠 = 已覆蓋、紅 = ``uncovered_cells_xy``、
    區外透明；標題列出各必要構型之覆蓋率。``report`` 為 `sim.CoverageReport`（或 :class:`CoverageLite`）序列。"""
    configure_fonts()
    reports = [r for r in report if getattr(r, "area_id", None) in ctx.areas_enu]
    if not reports:
        _empty_figure(path, "no coverage report", "Coverage raster")
        return
    n = len(reports)
    ncol = min(3, n)
    nrow = int(math.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(6.0 * ncol, 5.0 * nrow), squeeze=False)
    cmap = ListedColormap(["#2ca02c", "#d62728"])
    for k, rep in enumerate(reports):
        ax = axes[k // ncol, k % ncol]
        poly = ctx.areas_enu[rep.area_id]
        r = float(getattr(rep, "raster_m", 5.0) or 5.0)
        X, Y, r_eff = _raster_centres(poly, r)
        inside = shapely.contains_xy(poly, X, Y)
        img = np.full(X.shape, np.nan)
        img[inside] = 0.0
        unc = np.asarray(getattr(rep, "uncovered_cells_xy", np.zeros((0, 2))), dtype=float).reshape(-1, 2)
        if unc.size:
            ix = np.rint((unc[:, 0] - X[0, 0]) / r_eff).astype(int)
            iy = np.rint((unc[:, 1] - Y[0, 0]) / r_eff).astype(int)
            ok = (ix >= 0) & (ix < X.shape[1]) & (iy >= 0) & (iy < X.shape[0])
            img[iy[ok], ix[ok]] = 1.0
        b = poly.bounds
        extent = (X[0, 0] - 0.5 * r_eff, X[0, -1] + 0.5 * r_eff, Y[0, 0] - 0.5 * r_eff, Y[-1, 0] + 0.5 * r_eff)
        ax.imshow(np.ma.masked_invalid(img), extent=extent, origin="lower", cmap=cmap, vmin=0, vmax=1,
                  interpolation="nearest", alpha=0.75, zorder=1)
        _draw_geom(ax, poly, face=None, edge="#b8860b", lw=1.5, zorder=3)
        if unc.size:
            ax.plot(unc[:, 0], unc[:, 1], ls="none", marker="s", ms=2.0, color="#d62728", alpha=0.9, zorder=2)
        cov_txt = ", ".join(f"{c}: {100.0 * v:.1f}%" for c, v in getattr(rep, "coverage_by_cls", {}).items())
        req = ",".join(getattr(rep, "cls_required", []))
        ax.set_title(f"{rep.area_id} (req {req}) raster {r_eff:.0f} m\n{cov_txt}; uncovered cells {len(unc)}",
                     fontsize=9)
        m = 0.05 * max(b[2] - b[0], b[3] - b[1], 1.0)
        ax.set_xlim(b[0] - m, b[2] + m)
        ax.set_ylim(b[1] - m, b[3] + m)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(True, alpha=0.2)
    for k in range(n, nrow * ncol):
        axes[k // ncol, k % ncol].set_axis_off()
    handles = [Patch(facecolor="#2ca02c", alpha=0.75, label="covered"), Patch(facecolor="#d62728", label="uncovered")]
    fig.legend(handles=handles, loc="lower center", ncol=2, fontsize=8, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle("Coverage raster (>= 99.5% required per class)", fontsize=11)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    _save(fig, path, dpi)


# ---------------------------------------------------------------------------
# energy（附加）
# ---------------------------------------------------------------------------

def plot_energy(plan: MissionPlan, ctx: MissionContext, path: str, *, dpi: int = _DEFAULT_DPI) -> None:
    """每機剩餘能量對時間（契約外附加）：``Waypoint.energy_remaining_wh``，保留 ``r·E_nom``（`EnergyModel.reserve_wh`）
    以虛線標示，階段以底色標示。"""
    configure_fonts()
    plans = _sorted_plans(plan)
    if not plans:
        _empty_figure(path, "no UAV plans", "Energy")
        return
    fig, ax = plt.subplots(figsize=(11, 5))
    for up in plans:
        wps = up.waypoints
        if not wps:
            continue
        t = np.array([w.t_s for w in wps]) / 60.0
        e = np.array([w.energy_remaining_wh for w in wps], dtype=float)
        col = CLASS_COLORS.get(up.cls, "#000000")
        ax.plot(t, e, color=col, lw=1.6, label=f"{up.uav_id} ({up.cls}) margin {up.energy_margin_wh:.0f} Wh")
        model = ctx.models.get(up.uav_id)
        if model is not None:
            try:
                res = float(model.reserve_wh())
                ax.axhline(res, color=col, lw=0.8, ls="--", alpha=0.7)
                ax.text(t[-1], res, f" reserve {up.uav_id} {res:.0f} Wh", fontsize=6, color=col, va="bottom")
            except Exception:  # pragma: no cover
                pass
        for ph, i0, i1 in _phase_runs(wps):
            if ph is Phase.OPERATE:
                t1 = wps[i1 + 1].t_s if i1 + 1 < len(wps) else wps[i1].t_s
                ax.axvspan(wps[i0].t_s / 60.0, t1 / 60.0, color=PHASE_COLORS[Phase.OPERATE], alpha=0.06, lw=0)
    ax.set_xlabel("mission time (min)")
    ax.set_ylabel("energy remaining (Wh)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("Energy remaining per UAV (OPERATE phases shaded)")
    _save(fig, path, dpi)


# ---------------------------------------------------------------------------
# save_all
# ---------------------------------------------------------------------------

def resolve_coverage(plan: MissionPlan, ctx: MissionContext, report: Any = None) -> list[Any]:
    """取得覆蓋報告清單：``report.coverage``（非空）→ `sim.coverage_raster(plan, ctx, ctx.cfg)`（延遲 import；
    規格 §17.1 (1) 之實際 AGL 足跡）→ 失敗時 :func:`coverage_from_plan` 近似。"""
    cov = list(getattr(report, "coverage", None) or [])
    if cov:
        return cov
    try:
        from . import sim as _sim  # 延遲 import：避免 viz 之 import 圖拉入 sim/planner 全部
        cov = list(_sim.coverage_raster(plan, ctx, ctx.cfg))
    except Exception:
        cov = []
    if not cov:
        cov = list(coverage_from_plan(plan, ctx))
    return cov


def save_all(plan: MissionPlan, ctx: MissionContext, report: Any, out_dir: str) -> list[str]:
    """全部圖檔 → ``out_dir``（契約 §15）：``plan_2d.png``、``profiles.png``、``gantt.png``、``coverage.png``、
    ``energy.png``。``report`` 為 `sim.ValidationReport`（鴨子型別：``coverage``、``conflicts``）或 None；覆蓋報告由
    :func:`resolve_coverage` 取得（report → `sim.coverage_raster` → :func:`coverage_from_plan`）。甘特圖之走廊預約以
    ``ctx.cfg`` 與 home 站由 `CorridorSchedule.reserve_plan` 重建（含衝突標示）。回傳實際寫出之 PNG 路徑清單（依上列順序）。"""
    os.makedirs(out_dir, exist_ok=True)
    conflicts = list(getattr(report, "conflicts", None) or [])
    paths: list[str] = []
    p = os.path.join(out_dir, "plan_2d.png")
    plot_2d(plan, ctx, p, conflicts=conflicts)
    paths.append(p)
    p = os.path.join(out_dir, "profiles.png")
    plot_profiles(plan, ctx, p)
    paths.append(p)
    schedule: CorridorSchedule | None = None
    try:
        schedule = CorridorSchedule(ctx.cfg)
        home = _home_id(ctx)
        for up in plan.plans.values():
            schedule.reserve_plan(up, home)
    except Exception:
        schedule = None
    p = os.path.join(out_dir, "gantt.png")
    plot_gantt(plan, schedule, p, cfg=ctx.cfg, takeoff_site_id=_home_id(ctx),
               conflicts=[c for c in conflicts if getattr(c, "kind", "4d") != "corridor"])
    paths.append(p)
    coverage = resolve_coverage(plan, ctx, report)
    p = os.path.join(out_dir, "coverage.png")
    plot_coverage(coverage, ctx, p)
    paths.append(p)
    p = os.path.join(out_dir, "energy.png")
    plot_energy(plan, ctx, p)
    paths.append(p)
    return paths
