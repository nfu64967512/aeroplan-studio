"""vendored 修改 #14（context）：網格邊緣有 nan 時，「KeepoutSet 沒燒入 ⇒ build_context 補燒一次」照樣執行。

舊判斷 ``np.array_equal(terrain.z, terrain.z_raw)`` 遇到 nan（DEM 範圍外的節點；地形網格 = 所有幾何 ⊕ 3000 m，
真 DEM 與內建情境 MINI／HILLS 外圈都是 nan）永遠不等 ⇒ 補燒那一支從不執行、障礙沒燒進地形。改問
``TerrainGrid.same_as_raw()``（#13：逐值相等或位元組完全相同、含 nan 位置）。

幾何沿用 test_context（平地 100 m、塔 T1 40 m AGL、輸電線 PL1 頂 130 m MSL）；DEM 只蓋「所有幾何 ⊕ 400 m」。
"""
from __future__ import annotations

import sys

import numpy as np
import pytest

import hv_dccpp.routing  # noqa: F401  先載入，monkeypatch 才能把它換成 None 再還原
from hv_dccpp.context import build_context
from hv_dccpp.geo import DemGrid
from hv_dccpp.mission import Obstacle

from .test_context import CFG, REF, TOWER, Z0, _wgs, make_inputs

BURN_WARN = "obstacles_burned_by_context"


def _nan_edge_dem() -> DemGrid:
    """DEM 只蓋 ENU x∈[-400, 7400]、y∈[-1600, 1900] ⇒ 地形網格外圈是 nan。"""
    sw = REF.to_wgs84(np.array([-400.0, -1600.0, 0.0]))[0]
    ne = REF.to_wgs84(np.array([7400.0, 1900.0, 0.0]))[0]
    step = 0.0005
    nr = int(np.ceil((ne[0] - sw[0]) / step)) + 1
    nc = int(np.ceil((ne[1] - sw[1]) / step)) + 1
    return DemGrid(float(sw[0]), float(sw[1]), step, step, np.full((nr, nc), Z0, dtype=np.float32), sigma_z_m=2.0)


def _no_routing(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "hv_dccpp.routing", None)    # build_context 的延遲 import 丟 ImportError


def _burned(t) -> np.ndarray:
    """有限節點裡被燒入改到的（nan != nan 恆真，要先排除）。"""
    return np.isfinite(t.z_raw) & (t.z != t.z_raw)


def _burn_warnings(ctx) -> list[str]:
    return [w for w in ctx.warnings if w.startswith(BURN_WARN)]


def test_routing_unavailable_nan_edge_burns_exactly_like_full_dem(monkeypatch):
    """routing 不可用（keepouts=None、KeepoutSet 沒燒）：nan 邊緣網格要燒得和全覆蓋 DEM 一模一樣。"""
    _no_routing(monkeypatch)
    edge = build_context(make_inputs(dem=_nan_edge_dem()), CFG)
    full = build_context(make_inputs(), CFG)
    te, tf = edge.terrain, full.terrain
    # 情境有鑑別力：外圈真的是 nan ⇒ 舊判斷（array_equal）在燒入前就已經是 False
    assert edge.keepouts is None and "routing_unavailable_keepouts_none" in edge.warnings
    assert np.isnan(te.z_raw).sum() > 1000 and not np.isnan(tf.z_raw).any()
    assert not np.array_equal(te.z_raw, te.z_raw)
    assert (te.x0_m, te.y0_m, te.ny, te.nx) == (tf.x0_m, tf.y0_m, tf.ny, tf.nx)
    # 有補燒、而且與無 nan 的對照組逐節點相同
    assert _burn_warnings(edge) == _burn_warnings(full) == [f"{BURN_WARN}(T1,PL1)"]
    assert edge.warnings == full.warnings
    assert _burned(te).sum() > 100
    np.testing.assert_array_equal(_burned(te), _burned(tf))
    np.testing.assert_array_equal(te.z[_burned(tf)], tf.z[_burned(tf)])
    np.testing.assert_array_equal(np.isnan(te.z), np.isnan(te.z_raw))                 # nan 位置不變
    for o in edge.obstacles_enu:                                                        # 障礙頂 + b_v 燒進地形
        c = o.geom.interpolate(0.5, normalized=True) if o.geom.geom_type == "LineString" else o.geom
        assert te.z_at(np.array([[c.x, c.y]]))[0] >= o.top_z_m + CFG.b_v_m - 1e-3


def test_keepoutset_burned_nan_edge_is_not_burned_twice():
    """routing 可用：KeepoutSet 已燒入 ⇒ context 不再補燒（「只做一次」不因修正而破功）。"""
    ctx = build_context(make_inputs(dem=_nan_edge_dem()), CFG)
    assert ctx.keepouts is not None and np.isnan(ctx.terrain.z_raw).any()
    assert _burn_warnings(ctx) == []
    assert not ctx.terrain.same_as_raw() and _burned(ctx.terrain).sum() > 100
    t1 = ctx.obstacles_enu[0].geom
    assert ctx.terrain.z_at(np.array([[t1.x, t1.y]]))[0] >= ctx.obstacles_enu[0].top_z_m + CFG.b_v_m - 1e-3


def _edge_tower(dem: DemGrid) -> Obstacle:
    """塔放在「最後一個有限節點」與 DEM 東緣之間：DEM 取得到地面，但雙線性四鄰有 nan ⇒ 網格 z_at = nan。"""
    probe = build_context(make_inputs(dem=dem, obstacles=(TOWER,)), CFG).terrain
    j = int(round((0.0 - probe.y0_m) / probe.step_m))
    x_k = probe.x0_m + int(np.flatnonzero(np.isfinite(probe.z_raw[j]))[-1]) * probe.step_m
    y_j = probe.y0_m + j * probe.step_m
    xs = x_k + np.linspace(0.0, probe.step_m, 501)[1:-1]
    lat, lon = (np.array(v) for v in zip(*(_wgs(float(x), y_j) for x in xs)))
    x_edge = float(xs[np.isfinite(dem.sample_msl(lat, lon))][-1])
    return Obstacle("TE", "point", (_wgs(0.5 * (x_k + x_edge), y_j),), None, 40.0, category="tower")


def test_edge_obstacle_keepoutset_cannot_ground_is_burned_by_context():
    """唯一障礙貼著 DEM 邊緣：KeepoutSet 量不到地面 ⇒ top = +inf、不燒入；context 以頂點 DEM 值得到有限頂高
    ⇒ 補燒（舊判斷因 nan 而跳過，這座塔從沒燒進地形）。"""
    dem = _nan_edge_dem()
    ctx = build_context(make_inputs(dem=dem, obstacles=(_edge_tower(dem),)), CFG)
    t = ctx.terrain
    assert ctx.keepouts is not None and ctx.keepouts.ids[0] == "TE" and ctx.keepouts.tops_z[0] == np.inf
    (te,) = ctx.obstacles_enu
    assert np.isfinite(te.top_z_m) and np.isnan(t.z_at(np.array([[te.geom.x, te.geom.y]]))[0])
    assert _burn_warnings(ctx) == [f"{BURN_WARN}(TE)"]
    m = _burned(t)
    assert m.sum() >= 4
    np.testing.assert_array_equal(t.z[m], np.float32(te.top_z_m + CFG.b_v_m))
