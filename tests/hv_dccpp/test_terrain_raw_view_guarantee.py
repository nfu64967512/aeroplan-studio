"""vendored 修改 #13（geo）：``TerrainGrid.raw_view`` 的保證與 ``same_as_raw``／``corridor_minmax`` 的逐位元等價。

``geo.py`` 的 docstring 寫「`burn()` 不影響其內容」。舊版在 z 與 z_raw 相等時回傳 ``self``，之後燒入就會改到
已拿走的「自然地形」——保證其實不成立（只是所有呼叫點剛好都在函式內現取現用）；網格邊緣有 nan 時
``np.array_equal`` 永遠不等，又白白複製一份，並讓 ``altitude`` 重算同樣的走廊最大值。現在視圖永遠是另一個
物件、與 ``z_raw`` 共用唯讀記憶體；「兩者是否相同」改問 ``same_as_raw()``。
"""
from __future__ import annotations

import dataclasses
import logging
import math

import numpy as np
import pytest
import shapely

from hv_dccpp.geo import TerrainGrid

STEP = 10.0


def _grid(*, nan_edge: bool) -> TerrainGrid:
    rng = np.random.default_rng(13)
    z = rng.uniform(0.0, 30.0, (61, 81)) + np.linspace(0.0, 40.0, 81)[None, :]
    if nan_edge:                                    # DEM 範圍外的節點（真實 DEM 作業點的網格就是這樣）
        z[:, :3] = np.nan
        z[-2:, :] = np.nan
    return TerrainGrid(-400.0, -300.0, STEP, z, sigma_z_m=2.0)


POLY = shapely.Polygon([(-100.0, -100.0), (50.0, -120.0), (60.0, 40.0), (-80.0, 30.0)])
INSIDE = np.array([[0.0, -40.0], [-20.0, 0.0]])


def _old_copy_view(T: TerrainGrid) -> TerrainGrid:
    """舊版在「z ≠ z_raw」時建的視圖（複製 z_raw）——取樣結果的對照組。"""
    return TerrainGrid(T.x0_m, T.y0_m, T.step_m, T.z_raw, z_raw=T.z_raw, sigma_z_m=T.sigma_z_m, is_dsm=T.is_dsm)


@pytest.mark.parametrize("nan_edge", [False, True])
def test_raw_view_is_never_self_and_shares_z_raw_read_only(nan_edge):
    T = _grid(nan_edge=nan_edge)
    v = T.raw_view()
    assert v is not T and T.raw_view() is v and v.raw_view() is v
    assert np.shares_memory(v.z, T.z_raw) and np.shares_memory(v.z_raw, T.z_raw)    # 不複製
    assert not v.z.flags.writeable and T.z_raw.flags.writeable                       # 只有視圖唯讀
    with pytest.raises(ValueError):
        v.z[0, 0] = 1.0


@pytest.mark.parametrize("nan_edge", [False, True])
def test_burn_after_taking_the_view_does_not_change_it(nan_edge):
    """docstring 的保證：先拿視圖、後燒入，視圖仍是自然地形（舊版 nan_edge=False 時這條會紅）。"""
    T = _grid(nan_edge=nan_edge)
    nat = T.raw_view()
    before = nat.z_at(INSIDE).copy()
    T.burn([POLY], [500.0])
    assert np.all(T.z_at(INSIDE) >= 500.0 - 1e-3)                                  # 燒入有作用
    np.testing.assert_array_equal(nat.z_at(INSIDE), before)                          # 舊視圖不受影響
    np.testing.assert_array_equal(T.raw_view().z_at(INSIDE), before)                 # 新視圖也是自然地形


def test_burning_the_view_itself_raises_and_leaves_z_raw_alone():
    T = _grid(nan_edge=False)
    raw = T.z_raw.copy()
    with pytest.raises(ValueError):
        T.raw_view().burn([POLY], [500.0])
    np.testing.assert_array_equal(T.z_raw, raw)


@pytest.mark.parametrize("nan_edge", [False, True])
def test_same_as_raw(nan_edge):
    T = _grid(nan_edge=nan_edge)
    assert T.same_as_raw()                          # 未燒入（nan 邊緣也算相同——舊版的身分判斷在這裡失效）
    T.burn([POLY], [-1000.0])                       # top 在地面下 ⇒ 沒有節點被改
    assert T.same_as_raw()
    T.burn([POLY], [500.0])
    assert not T.same_as_raw()
    T2 = T.copy()
    assert not T2.same_as_raw()


@pytest.mark.parametrize("burned", [False, True])
@pytest.mark.parametrize("nan_edge", [False, True])
def test_raw_view_samples_bitwise_like_the_old_copied_view(nan_edge, burned):
    T = _grid(nan_edge=nan_edge)
    if burned:
        T.burn([POLY], [500.0])
    v, old = T.raw_view(), _old_copy_view(T)
    rng = np.random.default_rng(5)
    xy = np.column_stack([rng.uniform(-450.0, 450.0, 400), rng.uniform(-350.0, 350.0, 400)])  # 含範圍外
    th = rng.uniform(-np.pi, np.pi, 400)
    hw = rng.uniform(0.0, 60.0, 400)
    pairs = [(v.z_at(xy), old.z_at(xy)),
             (v.corridor_max(xy, th, hw, 5.0, offset_m=7.0), old.corridor_max(xy, th, hw, 5.0, offset_m=7.0)),
             (v.corridor_min(xy, th, 30.0, 5.0), old.corridor_min(xy, th, 30.0, 5.0)),
             (v.slope_at(xy), old.slope_at(xy)), (v.slope_at(xy, raw=False), old.slope_at(xy, raw=False)),
             (np.array([v.max_in_polygon(POLY), v.min_in_polygon(POLY)]),
              np.array([old.max_in_polygon(POLY), old.min_in_polygon(POLY)]))]
    for a, b in pairs:
        assert a.dtype == b.dtype and a.tobytes() == b.tobytes()


@pytest.mark.parametrize("nan_edge", [False, True])
def test_same_as_raw_means_sampling_z_equals_sampling_the_view(nan_edge):
    """``altitude`` 的捷徑（same_as_raw ⇒ 燒入後最大值 = 自然地形最大值）所依賴的前提，逐位元。"""
    T = _grid(nan_edge=nan_edge)
    assert T.same_as_raw()
    v = T.raw_view()
    rng = np.random.default_rng(6)
    xy = np.column_stack([rng.uniform(-450.0, 450.0, 300), rng.uniform(-350.0, 350.0, 300)])
    th = rng.uniform(-np.pi, np.pi, 300)
    for f in ("corridor_max", "corridor_min"):
        a, b = getattr(T, f)(xy, th, 40.0, 5.0), getattr(v, f)(xy, th, 40.0, 5.0)
        assert a.tobytes() == b.tobytes()


@pytest.mark.parametrize("nan_edge", [False, True])
def test_corridor_minmax_equals_two_separate_calls(nan_edge):
    T = _grid(nan_edge=nan_edge)
    rng = np.random.default_rng(8)
    xy = np.column_stack([rng.uniform(-450.0, 450.0, 200), rng.uniform(-350.0, 350.0, 200)])
    th = rng.uniform(-np.pi, np.pi, 200)
    for hw, off in ((0.0, 0.0), (25.0, 0.0), (rng.uniform(0.0, 60.0, 200), -12.5)):
        zmax, zmin = T.corridor_minmax(xy, th, hw, 5.0, offset_m=off)
        assert zmax.tobytes() == T.corridor_max(xy, th, hw, 5.0, offset_m=off).tobytes()
        assert zmin.tobytes() == T.corridor_min(xy, th, hw, 5.0, offset_m=off).tobytes()


def _strip_a1():
    """同 tests/hv_dccpp/test_altitude.py 的 §W.3 A 條帶 1（y = 503）。"""
    from hv_dccpp.geometry import Strip
    return Strip("Z1_A_1", "Z1", 1, "A", np.array([3932.0, 503.0]), np.array([7068.0, 503.0]), math.pi / 2,
                 np.array([3985.0, 615.0]), 224.07, 168.05, 160.0, 160.0, 160.0, -112.0, 112.0, 3136.0, 160.0)


def _route_segments(ctx, uav, strip, z_op, h_tc=100.0):
    """同 tests/hv_dccpp/test_altitude.py：起飛走廊 + 進入 + 作業 + 返航 + 降落走廊（單條帶）。"""
    from hv_dccpp import altitude as ALT
    from hv_dccpp.dubins import Pose
    from hv_dccpp.mission import Mode, Phase
    from hv_dccpp.routing import avoid
    va = ctx.specs[uav.cls].cruise_airspeed_mps
    r = ctx.specs[uav.cls].r_eff(5.0)
    s_entry = strip.start_state(-1)
    to = ALT.takeoff_corridor("H", uav, ctx, s_entry, z_op, 0.0)
    s_last = strip.end_state(-1)
    ld = ALT.landing_corridor("H", uav, ctx, s_last, z_op)
    ex = to.exit_state
    entry = avoid(Pose(ex.x_m, ex.y_m, ex.theta_rad), s_entry, r, ctx.keepouts, z_op, ctx.wind, va, 50.0, {})
    rtb = avoid(s_last, ld.app_state, r, ctx.keepouts, z_op, ctx.wind, va, 50.0, {})
    return list(to.segments) + [
        ALT.PathSegment2D(Phase.ENTRY, Mode.CRUISE, entry, None, None, va, False, ex.z_m, z_op),
        ALT.PathSegment2D(Phase.OPERATE, Mode.CRUISE, np.vstack([strip.pL, strip.pR]), strip.strip_id, -1, va, True,
                          z_op, z_op),
        ALT.PathSegment2D(Phase.RTB, Mode.CRUISE, rtb, None, None, va, False, z_op, h_tc),
    ] + list(ld.segments)


def test_burned_obstacle_keeps_clearance_in_both_altitude_bounds():
    """燒入後的地形（障礙頂）仍要保有 h_clr：``local_envelope`` 與 ``sample_bounds`` 的 ``zb_max + h_clr`` 下界。

    障礙只燒進網格、不在 keep-out 裡 ⇒ z_obs 不管它，``zb_max`` 那一項是唯一的保護。``same_as_raw`` 捷徑若在燒入後
    仍共用自然地形的最大值，這兩個下界會掉到障礙頂以下（2026-10-21 突變實測：原本沒有任何測試咬得到這一支）。"""
    from hv_dccpp import altitude as ALT
    from hv_dccpp.context import build_context
    from hv_dccpp.mission import Phase
    from hv_dccpp.platform import PLATFORM_PIONEER
    from hv_dccpp.scenario import build_mini_example

    inputs, cfg = build_mini_example()
    ctx = build_context(inputs, dataclasses.replace(cfg, dem_step_m=25.0), terrain_margin_m=1500.0)
    T = ctx.terrain
    c = np.array([[5500.0, 503.0]])
    zg = float(T.raw_view().z_at(c)[0])
    top = zg + 200.0
    T.burn([shapely.Point(5500.0, 503.0).buffer(60.0)], [top])
    assert not T.same_as_raw() and float(T.z_at(c)[0]) >= top - 1e-3
    h_clr = float(ctx.h_clr_m)

    n = 121
    xy = np.column_stack([np.linspace(4000.0, 7000.0, n), np.full(n, 503.0)])
    near = np.abs(xy[:, 0] - 5500.0) <= 40.0
    prof = ALT.local_envelope(xy, np.full(n, math.pi / 2), np.ones(n, dtype=bool), "A", PLATFORM_PIONEER,
                              ctx.sensors[("A", "default")], T, 112.0, 0.0, h_clr, None, 160.0, ctx.wind, 24.6, ctx.cfg)
    assert near.any() and np.all(prof.z[near] >= top + h_clr - 1e-6)
    zg_nat = T.raw_view().corridor_max(xy, np.full(n, math.pi / 2), 112.0, 12.5)
    assert np.all(zg_nat[near] + max(160.0, h_clr, PLATFORM_PIONEER.min_fw_operation_agl_m) < top + h_clr)
    # ↑ 前置：只看自然地形的目標與下界都在障礙要求之下 ⇒ 上面那條真的是燒入後地形那一項撐起來的

    st = _strip_a1()
    segs = _route_segments(ctx, ctx.uav("A1"), st, 160.0)
    samples = ALT.resample_segments(segs, ctx.cfg.d_d_m, ctx.cfg.e_max_m, ctx.cfg.psi_max_rad)
    j_c, j_d = ALT.phase_boundaries(samples, "A", ctx, 100.0, 100.0, 24.6)
    z_lo = ALT.sample_bounds(samples, "A", ctx, {st.strip_id: st}, 24.6, j_c, j_d)[0]
    op = np.array([p.phase is Phase.OPERATE for p in samples])
    sxy = np.array([[p.x_m, p.y_m] for p in samples])
    hit = op & (np.abs(sxy[:, 0] - 5500.0) <= 40.0) & (np.abs(sxy[:, 1] - 503.0) <= 40.0)
    assert hit.any() and np.all(z_lo[hit] >= top + h_clr - 1e-6)


def test_mini_plan_identical_when_shortcut_is_disabled(monkeypatch):
    """端到端：mini 範例的網格邊緣有 nan、未燒入（same_as_raw 為真 ⇒ 捷徑生效）。強制 same_as_raw → False
    （＝舊版在 nan 網格上的走法：另算一次燒入後的最大值）整份計畫逐欄位相同。"""
    import hv_dccpp
    from hv_dccpp.context import build_context
    from hv_dccpp.scenario import build_mini_example

    logging.disable(logging.WARNING)
    try:
        inputs, cfg = build_mini_example()
        T = build_context(inputs, cfg).terrain
        assert np.isnan(T.z).any() and T.same_as_raw()                                # 前置：確實走捷徑
        a = hv_dccpp.plan_mission(inputs, cfg)
        monkeypatch.setattr(TerrainGrid, "same_as_raw", lambda self: False)
        b = hv_dccpp.plan_mission(inputs, cfg)
    finally:
        logging.disable(logging.NOTSET)
    da, db = dataclasses.asdict(a), dataclasses.asdict(b)
    da.pop("timing_ms", None)
    db.pop("timing_ms", None)
    assert _exact(da) == _exact(db)


def _exact(v):
    """逐位元的比較形式（repr 會把長陣列縮成「...」，不能直接比）。"""
    if isinstance(v, dict):
        return ("D", [(_exact(k), _exact(x)) for k, x in v.items()])
    if isinstance(v, (list, tuple)):
        return (type(v).__name__, [_exact(x) for x in v])
    if isinstance(v, np.ndarray):
        if v.dtype == object:
            return ("ndo", v.shape, [_exact(x) for x in v.ravel().tolist()])
        return ("nd", str(v.dtype), v.shape, np.ascontiguousarray(v).tobytes())
    if isinstance(v, (float, np.floating)):
        return (type(v).__name__, repr(float(v)))
    if hasattr(v, "geom_type"):
        return ("G", shapely.to_wkb(v, hex=True))
    return (type(v).__name__, repr(v))
