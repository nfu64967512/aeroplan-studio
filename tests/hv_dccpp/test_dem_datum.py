"""tests/hv_dccpp/test_dem_datum.py — A6：DEM 高程基準（正高／橢球高、大地起伏 N 已知／未知）的值鎖。

背景：GUI 把 Cesium World Terrain 的**橢球高**餵進 HARVEST，轉接層建 DemGrid 時沒帶基準 ⇒ 被標成 msl、N = 0。
GUI 一律上傳相對起飛坪（frame 3）⇒ N 抵銷、實飛 AGL 正確；但任何絕對海拔的介面（MSL 匯出、坪 alt_msl、禁航區
海拔上下限…）都會差 N。最直接的後果（本檔 T3 重現）：單機、坪高給真海拔 30 m、地形是 48.5 m 橢球高 ⇒ 計畫
120 m AGL 的條帶相對坪 **138.5 m**（> 法規 121.9 m）。

情境：平地（橢球高 48.5 m ＝ 海拔 30 m ＋ N 18.5 m；西南角 3.5 km 外一座 5 m 小丘讓 DEM 不是常數平面）、
單架構型 B、1.2 km × 0.6 km 區域、AGL 上限 120（不給豁免）、作業高度 120。單次規劃 ≈ 0.3 s。
"""
from __future__ import annotations

import json
import math
from dataclasses import replace

import numpy as np
import pytest

from hv_dccpp.ardupilot import CameraTriggerConfig, export_all, fc_home
from hv_dccpp.context import absolute_msl_inputs, build_context
from hv_dccpp.geo import GEOID_UNDULATION_RANGE_M, DemGrid, VerticalDatumError, require_true_msl
from hv_dccpp.integration.aeroplan_adapter import (
    AeroPlanAdapterError, HvDccppCoordinator, terrain_func_to_dem,
)
from hv_dccpp.mission import (
    Airspace, MissionInputError, NoFlyZone, Obstacle, Phase, PlannerConfig, Site, load_mission,
)
from hv_dccpp.scenario import build_mini_example, scenario_frame

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
N_TRUE = 18.5            # 這裡的大地起伏（桃園一帶約 18.5 m）
GROUND_MSL = 30.0        # 地面海拔
GROUND_ELL = GROUND_MSL + N_TRUE
LEGAL_MAX_REL_M = 121.92  # 400 ft


def _ll(e: float, n: float) -> tuple[float, float]:
    return (HOME[0] + n / _M_LAT, HOME[1] + e / _M_LON)


AREAS = [{"area_id": 1, "priority": 1.0,
          "polygon": [_ll(1500, -300), _ll(2700, -300), _ll(2700, 300), _ll(1500, 300)]}]
DRONES = [{"drone_id": 1, "name": "UAV-1", "position": HOME, "speed": 19.0, "heading": 0.0,
           "turn_radius": 120.0, "vehicle_type": "vtol", "fov_width": 110.0}]
_BUMP = _ll(-2500, -2500)


def _z(lat: float, lon: float, base: float) -> float:
    """平地 base ＋ 遠處一座 5 m 小丘（離坪與作業區 > 2 km；只為讓 DEM 不是常數平面）。"""
    d = math.hypot((lat - _BUMP[0]) * _M_LAT, (lon - _BUMP[1]) * _M_LON)
    return base + (5.0 if d < 200.0 else 0.0)


class EllipsoidProvider:
    """自我宣告為橢球高的地形來源（同 CesiumWorldTerrain.vertical_datum）。"""
    vertical_datum = "ellipsoid"

    def get_elevation(self, lat: float, lon: float) -> float:
        return _z(lat, lon, GROUND_ELL)


def plain_ellipsoidal(lat: float, lon: float) -> float:
    """舊行為：同一份橢球高，但是純函式（沒有宣告）⇒ 依 AMSL 舊契約被當正高。"""
    return _z(lat, lon, GROUND_ELL)


def _plan(terrain_func, *, n=None, home=HOME, vertical_datum=None, fence_zones=None) -> HvDccppCoordinator:
    c = HvDccppCoordinator(class_of=lambda d: "B", cfg=PlannerConfig(dem_step_m=25.0, dh_band_min_m=15.0),
                           home=home, waiver_max_agl_m=None, dem_margin_m=3000.0, fence_zones=fence_zones,
                           vertical_datum=vertical_datum, geoid_undulation_m=n)
    c.plan_coverage_dccpp(areas=AREAS, drones=DRONES, coverage_width_m=110.0, overlap_rate=0.2,
                          coverage_altitude=120.0, terrain_func=terrain_func)
    return c


def _uplan(c: HvDccppCoordinator):
    return next(iter(c.last_plan.plans.values()))


def _op_rel(c: HvDccppCoordinator) -> np.ndarray:
    """計畫 OPERATE 航點相對**本機起飛坪**的高度（＝上傳 frame 3 的高度基準）。"""
    up = _uplan(c)
    pad = fc_home(c.last_ctx, up)[3]
    return np.array([w.alt_msl_m - pad for w in up.waypoints if w.phase is Phase.OPERATE])


def _items(c: HvDccppCoordinator, alt_frame: str = "relative") -> list[dict]:
    return c.mission_items(1, camera_cfg=CameraTriggerConfig(), alt_frame=alt_frame)


class FenceZoneStub:
    """AeroPlan FenceZone 的鴨子型別（GUI 對話框預設海拔 0–200 m）；放在作業區北方 1.5 km，不擋航線。"""
    category = "NFZ"
    shape = "POLYGON"
    name = "NFZ_N"
    id = "NFZ_N"
    alt_min_m = 0.0
    alt_max_m = 200.0
    vertices = [_ll(1800, 1500), _ll(2200, 1500), _ll(2200, 1800), _ll(1800, 1800)]


@pytest.fixture(scope="module")
def known():
    """T1：橢球高來源 ＋ 已知 N 18.5。"""
    return _plan(EllipsoidProvider().get_elevation, n=N_TRUE)


@pytest.fixture(scope="module")
def unknown():
    """T2：同一來源、N 未知（GUI 的 World Terrain 現況）。"""
    return _plan(EllipsoidProvider().get_elevation, n=None)


# ===========================================================================
# DemGrid 三態（沒給 ≠ 0 ≠ None）
# ===========================================================================

def _grid(z=GROUND_ELL, **kw) -> DemGrid:
    return DemGrid(23.9, 120.8, 0.01, 0.01, np.full((4, 4), z, np.float32), **kw)


def test_ellipsoid_without_an_explicit_n_is_refused_never_zero():
    with pytest.raises(ValueError, match="不會默默當 N = 0"):
        _grid(vertical_datum="ellipsoid")
    # msl 沒給 ＝ 舊預設 0.0（逐位元不變）
    g = _grid(GROUND_MSL)
    assert g.geoid_undulation_m == 0.0 and g.vertical_datum == "msl" and g.msl_known
    with pytest.raises(ValueError, match="只適用"):
        _grid(GROUND_MSL, geoid_undulation_m=None)                      # 未知 N 只對橢球高有意義
    lo, hi = GEOID_UNDULATION_RANGE_M
    for bad in (lo - 1.0, hi + 1.0, float("nan"), float("inf"), True, "18.5x"):
        with pytest.raises(ValueError):
            _grid(vertical_datum="ellipsoid", geoid_undulation_m=bad)


def test_sample_msl_known_subtracts_n_unknown_returns_native_ellipsoidal():
    k = _grid(vertical_datum="ellipsoid", geoid_undulation_m=N_TRUE)
    u = _grid(vertical_datum="ellipsoid", geoid_undulation_m=None)
    assert k.msl_known and not u.msl_known
    assert float(k.sample_msl(23.91, 120.81)[0]) == pytest.approx(GROUND_MSL, abs=1e-6)
    assert float(u.sample_msl(23.91, 120.81)[0]) == pytest.approx(GROUND_ELL, abs=1e-6)   # 原生橢球高，不是 0、不是海拔
    assert k.frame_undulation_m == N_TRUE and u.frame_undulation_m == 0.0
    assert "N 未知" in u.datum_label() and "18.50" in k.datum_label()


def test_require_true_msl_gate():
    u = _grid(vertical_datum="ellipsoid", geoid_undulation_m=None)
    with pytest.raises(VerticalDatumError, match="大地起伏"):
        require_true_msl(u, "x")
    assert issubclass(VerticalDatumError, ValueError)
    require_true_msl(_grid(vertical_datum="ellipsoid", geoid_undulation_m=N_TRUE), "x")      # 已知 ⇒ 放行
    require_true_msl(_grid(GROUND_MSL), "x")
    require_true_msl(None, "x")                                                              # 拿不到 DEM ⇒ 無從判斷


# ===========================================================================
# T1：已知 N
# ===========================================================================

def test_t1_known_n_converts_ellipsoid_to_msl(known):
    dem = known.last_inputs.dem
    assert dem.vertical_datum == "ellipsoid" and dem.geoid_undulation_m == N_TRUE
    assert float(dem.sample_msl(*HOME)[0]) == pytest.approx(GROUND_MSL, abs=1e-4)
    assert known.last_ctx.frame.geoid_undulation_m == N_TRUE
    assert known.last_ctx.frame.h0_msl_m == pytest.approx(GROUND_MSL, abs=1e-4)
    op = _op_rel(known)
    assert op.size and op.max() <= 120.0 + 1e-3 and op.min() > 100.0
    note = [n for n in known.notes if n.startswith("terrain_datum(")]
    assert len(note) == 1 and "N = 18.50 m" in note[0] and "EllipsoidProvider 宣告" in note[0], known.notes


def test_t1_msl_export_is_true_amsl_not_ellipsoidal(known):
    rel = _items(known)
    msl = _items(known, "msl")
    assert msl[0]["alt"] == pytest.approx(GROUND_MSL, abs=1e-6)          # home 列＝坪的海拔
    nav = [(m, r) for m, r in zip(msl[1:], rel[1:]) if m["frame"] == 0]
    assert nav
    # 每一個絕對高度 − 相對高度 ＝ 坪的海拔 30（不是橢球高 48.5）
    assert {round(m["alt"] - r["alt"], 6) for m, r in nav} == {GROUND_MSL}
    top = max(m["alt"] for m, _ in nav)
    assert top == pytest.approx(GROUND_MSL + 120.0, abs=0.5) and top < GROUND_ELL + 120.0 - 10.0


# ===========================================================================
# T2：N 未知 ⇒ 只准相對高度
# ===========================================================================

def test_t2_relative_outputs_identical_to_known_n(known, unknown):
    a, b = _items(known), _items(unknown)
    assert len(a) == len(b)
    for x, y in zip(a[1:], b[1:]):
        assert (x["command"], x["frame"]) == (y["command"], y["frame"])
        for k in ("lat", "lon", "alt", "param1", "param2", "param3", "param4"):
            assert x[k] == pytest.approx(y[k], abs=1e-6), (k, x, y)
    np.testing.assert_allclose(_op_rel(unknown), _op_rel(known), atol=1e-6)
    assert unknown.last_ctx.frame.geoid_undulation_m == 0.0                  # N 未知 ⇒ 框架 N = 0（幾何精確）
    assert unknown.last_ctx.frame.h0_msl_m == pytest.approx(GROUND_ELL, abs=1e-4)
    assert not unknown.last_inputs.dem.msl_known


def test_t2_unknown_n_refuses_msl_export_and_says_so(unknown, tmp_path):
    with pytest.raises(VerticalDatumError, match="大地起伏"):
        _items(unknown, "msl")
    out = tmp_path / "msl"
    with pytest.raises(VerticalDatumError, match="大地起伏"):
        export_all(unknown.last_plan, unknown.last_ctx, unknown.cfg, out, alt_frame="msl")
    assert not out.exists()                                                   # 擋在建目錄之前
    note = [n for n in unknown.notes if n.startswith("terrain_datum(")]
    assert len(note) == 1 and "N 未知 ⇒ 只准相對高度" in note[0], unknown.notes
    assert any(w.startswith("dem_vertical_datum(ellipsoid") for w in unknown.last_ctx.warnings)


def test_t2_relative_export_labels_heights_as_ellipsoidal(unknown, known, tmp_path):
    s = export_all(unknown.last_plan, unknown.last_ctx, unknown.cfg, tmp_path / "u")
    t = s["terrain"]
    assert (t["vertical_datum"], t["geoid_undulation_m"], t["msl_known"]) == ("ellipsoid", None, False)
    assert "NOT MSL" in t["alt_msl_m_fields_are"]
    assert "N 未知" in s["home"]["alt_datum"]
    on_disk = json.loads((tmp_path / "u" / "mission_summary.json").read_text(encoding="utf-8"))
    assert on_disk["terrain"]["geoid_undulation_m"] is None
    uid = next(iter(s["uavs"]))
    assert "N 未知" in s["uavs"][uid]["fc_home"]["alt_datum"]
    wp = (tmp_path / "u" / f"{uid}.waypoints").read_text(encoding="utf-8")
    assert "geoid N unknown" in wp and "m MSL" not in wp
    k = export_all(known.last_plan, known.last_ctx, known.cfg, tmp_path / "k")
    assert k["terrain"]["geoid_undulation_m"] == N_TRUE and k["terrain"]["msl_known"] is True
    wk = (tmp_path / "k" / f"{uid}.waypoints").read_text(encoding="utf-8")
    assert "m MSL" in wk and "geoid N unknown" not in wk


def test_t2_absolute_site_altitude_is_refused():
    h = Site("H", *HOME, alt_msl_m=GROUND_MSL, kind="home")
    with pytest.raises(AeroPlanAdapterError, match=r"Site\[H\]\.alt_msl_m") as e:
        _plan(EllipsoidProvider().get_elevation, n=None, home=h)
    assert "大地起伏" in str(e.value)


def test_t2_fence_zone_limits_become_full_column_when_n_unknown(unknown):
    c = _plan(EllipsoidProvider().get_elevation, n=None, fence_zones=[FenceZoneStub()])
    (z,) = c.last_inputs.airspace.no_fly
    assert z.z_floor_msl_m is None and z.z_ceiling_msl_m is None
    w = [n for n in c.notes if n.startswith("nfz_vertical_limits_ignored(")]
    assert len(w) == 1 and "NFZ_N[0～200 m]" in w[0] and "整柱" in w[0], c.notes
    assert c.last_plan.plans                                                  # 仍可規劃
    # 已知 N ⇒ 上下限照用、不記警告
    k = _plan(EllipsoidProvider().get_elevation, n=N_TRUE, fence_zones=[FenceZoneStub()])
    (zk,) = k.last_inputs.airspace.no_fly
    assert (zk.z_floor_msl_m, zk.z_ceiling_msl_m) == (0.0, 200.0)
    assert not any(n.startswith("nfz_vertical_limits_ignored(") for n in k.notes)


def test_build_context_lists_every_absolute_msl_key_when_n_unknown():
    inputs, cfg = build_mini_example()
    d = inputs.dem
    ell = DemGrid(d.lat0_deg, d.lon0_deg, d.dlat_deg, d.dlon_deg, np.asarray(d.z_msl) + N_TRUE,
                  vertical_datum="ellipsoid", geoid_undulation_m=None, sigma_z_m=d.sigma_z_m)
    area = inputs.areas[0].polygon_wgs84
    nf = NoFlyZone("NF1", area, z_floor_msl_m=0.0, z_ceiling_msl_m=300.0)
    ob = Obstacle("T1", "point", (area[0],), top_msl_m=200.0, height_agl_m=None)
    bad = replace(inputs, dem=ell, home=replace(inputs.home, alt_msl_m=100.0), obstacles=(ob,),
                  airspace=replace(inputs.airspace, max_msl_m=400.0, no_fly=(nf,)))
    keys = absolute_msl_inputs(bad)
    assert keys == ["Site[H].alt_msl_m", "Obstacle[T1].top_msl_m", "NoFlyZone[NF1].z_floor_msl_m",
                    "NoFlyZone[NF1].z_ceiling_msl_m", "Airspace.max_msl_m"]
    # C2 地面站是另一個 Site 物件（帶海拔）時也要列出
    gs = Site("GS", *area[1], alt_msl_m=120.0)
    assert "c2.ground_station[GS].alt_msl_m" in absolute_msl_inputs(replace(bad, c2=replace(bad.c2, ground_station=gs)))
    with pytest.raises(MissionInputError, match="大地起伏") as e:
        build_context(bad, cfg)
    for k in keys:
        assert k in str(e.value)
    # scenario_frame 與 build_context 同一條規則
    with pytest.raises(MissionInputError, match="大地起伏"):
        scenario_frame(bad)
    # 沒有絕對海拔輸入 ⇒ 照常建情境（風層 z_ref="msl" 刻意不擋）
    ok = replace(inputs, dem=ell)
    assert absolute_msl_inputs(ok) == []
    fr = scenario_frame(ok)
    assert fr.geoid_undulation_m == 0.0 and fr.h0_msl_m == pytest.approx(100.0 + N_TRUE, abs=1e-4)
    fk = scenario_frame(replace(inputs, dem=replace(ell, geoid_undulation_m=N_TRUE)))
    assert fk.geoid_undulation_m == N_TRUE and fk.h0_msl_m == pytest.approx(100.0, abs=1e-4)


# ===========================================================================
# T3：138 m 重現與鎖定（單機：起飛坪＝home Site）
# ===========================================================================

def test_t3_the_138_m_case_is_fixed_by_the_datum():
    h30 = Site("H", *HOME, alt_msl_m=GROUND_MSL, kind="home")
    new = _plan(EllipsoidProvider().get_elevation, n=N_TRUE, home=h30)
    op = _op_rel(new)
    assert op.max() <= 120.0 + 1e-3
    nav = [it["alt"] for it in _items(new)[1:] if it["frame"] == 3]
    assert max(nav) <= LEGAL_MAX_REL_M
    assert not any(n.startswith("site_alt_vs_dem(") for n in new.notes)
    # 舊行為（同一份橢球高、純函式 ⇒ 被當正高、N = 0）：條帶相對坪 138.5 m，超過 121.9 m 法規上限
    old = _plan(plain_ellipsoidal, home=h30)
    assert _op_rel(old).max() == pytest.approx(120.0 + N_TRUE, abs=0.05)
    assert max(it["alt"] for it in _items(old)[1:] if it["frame"] == 3) > LEGAL_MAX_REL_M
    # 這種基準混用現在至少會大聲說出來（坪 30 vs DEM 48.5 ⇒ 高飛）
    w = [n for n in old.notes if n.startswith("site_alt_vs_dem(")]
    assert len(w) == 1 and "差 -18.50 m" in w[0] and "高飛" in w[0], old.notes


def test_t3_reverse_direction_low_flight_is_warned():
    """SRTM 正高 DEM（標 msl）＋ e-GNSS 橢球高坪高：msl_known 擋不到，只能比數值 ⇒ 低飛 18.5 m 要被說出來。"""
    h_ell = Site("H", *HOME, alt_msl_m=GROUND_ELL, kind="home")
    c = _plan(lambda la, lo: _z(la, lo, GROUND_MSL), home=h_ell)
    assert _op_rel(c).max() == pytest.approx(120.0 - N_TRUE, abs=0.05)      # 這就是會低飛的量
    w = [n for n in c.notes if n.startswith("site_alt_vs_dem(")]
    assert len(w) == 1 and "差 +18.50 m" in w[0] and "低飛" in w[0], c.notes
    assert any(x.startswith("preflight:site_alt_vs_dem(") for x in c.notes)


def test_site_alt_check_skips_constant_flat_dem_and_small_offsets():
    """合成平地 DEM 沒有地形資訊（以坪高為準是對的）；差 ≤ 10 m 也不吵（屋頂坪／DEM 誤差）。"""
    h_ell = Site("H", *HOME, alt_msl_m=GROUND_ELL, kind="home")
    flat = _plan(lambda la, lo: GROUND_MSL, home=h_ell)
    assert not any("site_alt_vs_dem" in n for n in flat.notes)
    near = _plan(lambda la, lo: _z(la, lo, GROUND_MSL), home=Site("H", *HOME, alt_msl_m=GROUND_MSL + 9.0, kind="home"))
    assert not any("site_alt_vs_dem" in n for n in near.notes)


def _site_check_inputs():
    """mini 範例 ＋ 5×5 小 DEM（地面 30 m 正高；西南角節點 35 m ⇒ 不是常數平面），home 落在節點 (2, 2)。"""
    inputs, _ = build_mini_example()
    lat, lon = inputs.home.lat_deg, inputs.home.lon_deg
    z = np.full((5, 5), GROUND_MSL, np.float32)
    z[0, 0] = GROUND_MSL + 5.0
    return inputs, DemGrid(lat - 0.02, lon - 0.02, 0.01, 0.01, z)


def test_site_alt_vs_dem_tolerance_boundary_is_exactly_ten_metres():
    """A6 審查 L-c：門檻是「差 > 10 m」——10.0 不警告、10.01 警告（兩個方向）。"""
    from hv_dccpp.context import SITE_ALT_VS_DEM_TOL_M, _site_alt_vs_dem_warnings
    assert SITE_ALT_VS_DEM_TOL_M == 10.0
    inputs, dem = _site_check_inputs()

    def warn(alt):
        h = replace(inputs.home, alt_msl_m=alt)
        return _site_alt_vs_dem_warnings(replace(inputs, home=h, dem=dem, c2=replace(inputs.c2, ground_station=h)), dem)

    assert warn(GROUND_MSL + 10.0) == [] and warn(GROUND_MSL - 10.0) == []
    (hi,) = warn(GROUND_MSL + 10.01)
    assert hi.startswith("site_alt_vs_dem(H:") and "差 +10.01 m" in hi and "低飛" in hi
    (lo,) = warn(GROUND_MSL - 10.01)                       # c2 地面站就是 home（同一物件）⇒ 不重複報
    assert lo.startswith("site_alt_vs_dem(H:") and "差 -10.01 m" in lo and "高飛" in lo


def test_site_alt_vs_dem_checks_a_separate_c2_ground_station():
    """A6 審查 L-c：C2 地面站不是 sites 裡的任何一個（另一個 Site 物件、帶海拔）⇒ 也要比。"""
    from hv_dccpp.context import _site_alt_vs_dem_warnings
    inputs, dem = _site_check_inputs()
    h = replace(inputs.home, alt_msl_m=GROUND_MSL + 10.0)               # home 剛好在門檻上 ⇒ 不報
    gs = Site("GS", inputs.home.lat_deg + 0.01, inputs.home.lon_deg + 0.01, alt_msl_m=GROUND_MSL + 15.0)
    (w,) = _site_alt_vs_dem_warnings(replace(inputs, home=h, dem=dem, c2=replace(inputs.c2, ground_station=gs)), dem)
    assert w.startswith("site_alt_vs_dem(GS:") and "差 +15.00 m" in w
    # 同 id、同內容的地面站 ＝ sites 裡那一個 ⇒ 不重複；同 id 但內容不同（另一個物件）⇒ 照比
    assert _site_alt_vs_dem_warnings(
        replace(inputs, home=h, dem=dem, c2=replace(inputs.c2, ground_station=replace(h))), dem) == []
    (w2,) = _site_alt_vs_dem_warnings(
        replace(inputs, home=h, dem=dem, c2=replace(inputs.c2, ground_station=replace(h, alt_msl_m=GROUND_MSL + 12.0))),
        dem)
    assert w2.startswith("site_alt_vs_dem(H:") and "差 +12.00 m" in w2


def test_terrain_datum_note_carries_the_providers_own_explanation():
    """A6 審查 L-c：來源的 ``vertical_datum_note``（例如 DEMTerrainManager 讀 GeoTIFF 垂直 CRS 的判斷）原文進 note。"""
    class NotedProvider(EllipsoidProvider):
        vertical_datum_note = "DEM CRS 為 3-D 測試標記 Q7（垂直軸 = 橢球高）"

    c = _plan(NotedProvider().get_elevation, n=None)
    (td,) = [n for n in c.notes if n.startswith("terrain_datum(")]
    assert "NotedProvider 宣告" in td and NotedProvider.vertical_datum_note in td and "N 未知" in td
    k = _plan(NotedProvider().get_elevation, n=N_TRUE, vertical_datum="ellipsoid")       # 明確指定、與宣告一致
    (tk,) = [n for n in k.notes if n.startswith("terrain_datum(")]
    assert "明確指定" in tk and NotedProvider.vertical_datum_note in tk and "N = 18.50 m" in tk


# ===========================================================================
# 基準解析：明確 ＞ 來源宣告 ＞ AMSL 舊契約；矛盾 ⇒ 拒絕
# ===========================================================================

_B = (HOME[0] - 0.01, HOME[1] - 0.01, HOME[0] + 0.01, HOME[1] + 0.01)


def test_provider_declaration_explicit_override_and_mismatch():
    prov = EllipsoidProvider().get_elevation
    d = terrain_func_to_dem(prov, _B, 500.0)
    assert d.vertical_datum == "ellipsoid" and d.geoid_undulation_m is None and not d.msl_known
    assert terrain_func_to_dem(prov, _B, 500.0, geoid_undulation_m=N_TRUE).geoid_undulation_m == N_TRUE
    with pytest.raises(AeroPlanAdapterError, match="矛盾"):
        terrain_func_to_dem(prov, _B, 500.0, vertical_datum="msl")
    # 純函式：舊契約 AMSL；明確指定才是橢球高
    assert terrain_func_to_dem(plain_ellipsoidal, _B, 500.0).vertical_datum == "msl"
    e = terrain_func_to_dem(plain_ellipsoidal, _B, 500.0, vertical_datum="ellipsoid", geoid_undulation_m=N_TRUE)
    assert e.vertical_datum == "ellipsoid" and float(e.sample_msl(*HOME)[0]) == pytest.approx(GROUND_MSL, abs=1e-4)
    with pytest.raises(AeroPlanAdapterError):
        terrain_func_to_dem(prov, _B, 500.0, geoid_undulation_m=500.0)      # N 超出合理範圍
    with pytest.raises(AeroPlanAdapterError):
        HvDccppCoordinator(vertical_datum="wgs84")                           # type: ignore[arg-type]
    with pytest.raises(AeroPlanAdapterError, match="矛盾"):
        _plan(prov, vertical_datum="msl")


def test_undeclared_plain_terrain_func_says_it_was_assumed_msl():
    c = _plan(plain_ellipsoidal)
    note = [n for n in c.notes if n.startswith("terrain_datum(")]
    assert len(note) == 1 and "正高（MSL）" in note[0] and "未宣告" in note[0], c.notes
    assert c.last_inputs.dem.vertical_datum == "msl"


# ===========================================================================
# 序列化：N 未知以 JSON null 往返；橢球高缺 N ⇒ 錯誤
# ===========================================================================

def test_json_round_trip_keeps_unknown_n(unknown):
    inp = unknown.last_inputs
    d = json.loads(inp.to_json())
    assert d["dem"]["vertical_datum"] == "ellipsoid" and d["dem"]["geoid_undulation_m"] is None
    back = load_mission(d)
    assert back.dem.vertical_datum == "ellipsoid" and back.dem.geoid_undulation_m is None and not back.dem.msl_known
    assert back.to_dict() == inp.to_dict()
    d2 = dict(d)
    d2["dem"] = {k: v for k, v in d["dem"].items() if k != "geoid_undulation_m"}
    with pytest.raises(MissionInputError, match="不會默默當 N = 0"):
        load_mission(d2)


# ===========================================================================
# 其他絕對海拔出口：mavlink_link 後備匯出器、MQTT 橋接 frame="msl"
# ===========================================================================

def test_mavlink_fallback_global_frame_is_gated(unknown, known):
    pytest.importorskip("pymavlink")
    from hv_dccpp import mavlink_link as ml
    from pymavlink import mavutil
    m = mavutil.mavlink
    up = _uplan(unknown)
    with pytest.raises(VerticalDatumError, match="大地起伏"):
        ml.plan_to_mission_items_fallback(up, unknown.last_ctx, unknown.cfg, frame=m.MAV_FRAME_GLOBAL)
    rel = ml.plan_to_mission_items_fallback(up, unknown.last_ctx, unknown.cfg)
    assert len(rel) > 3
    ok = ml.plan_to_mission_items_fallback(_uplan(known), known.last_ctx, known.cfg, frame=m.MAV_FRAME_GLOBAL)
    assert len(ok) > 3


def test_mqtt_qgc_msl_frame_is_gated(unknown):
    from hv_dccpp.integration import mqtt_bridge as mb
    up = _uplan(unknown)
    ctx = unknown.last_ctx
    spec = ctx.specs[up.cls]
    home = (ctx.inputs.home.lat_deg, ctx.inputs.home.lon_deg, ctx.frame.h0_msl_m)
    with pytest.raises(VerticalDatumError, match="大地起伏"):
        mb.qgc_items_from_plan(up, spec, home, ctx.frame, plan=unknown.last_plan, ctx=ctx,
                               opt=mb.QgcOptions(frame="msl"))
    rel = mb.qgc_items_from_plan(up, spec, home, ctx.frame, plan=unknown.last_plan, ctx=ctx)
    assert len(rel) > 3


# ===========================================================================
# 來源宣告：DEMTerrainManager 讀垂直 CRS／metadata
# ===========================================================================

def test_dem_terrain_manager_declares_datum_from_vertical_crs():
    pytest.importorskip("pyproj")
    from pyproj import CRS
    from pyproj.crs import CompoundCRS
    from sensors.dem_terrain import _vertical_datum_from_wkt
    egm = CompoundCRS("WGS84+EGM96", [CRS.from_epsg(4326), CRS.from_epsg(5773)])
    assert _vertical_datum_from_wkt(egm.to_wkt())[0] == "msl"
    v, note = _vertical_datum_from_wkt(CRS.from_epsg(4326).to_wkt())     # SRTM（taiwan_dem.tif）：沒有垂直分量
    assert v is None and "SRTM" in note
    assert _vertical_datum_from_wkt("")[0] is None


# WKT1：GDAL 舊式「WGS 84 ＋ 橢球高」複合 CRS（VERT_DATUM 型別 2002 = 橢球）；PROJ 會把它升級成 Geographic 3D，
#   sub_crs_list 因此是空的（審查 M1 抓到的漏洞）
_WKT1_COMPD_ELLIPSOID = (
    'COMPD_CS["WGS 84 + Ellipsoid",GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563,'
    'AUTHORITY["EPSG","7030"]],AUTHORITY["EPSG","6326"]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433],'
    'AUTHORITY["EPSG","4326"]],VERT_CS["Ellipsoid (metre)",VERT_DATUM["Ellipsoid",2002],UNIT["metre",1],'
    'AXIS["Up",UP]]]')


def test_three_d_crs_with_ellipsoidal_height_axis_is_ellipsoid():
    """A6 審查 M1：非複合的 3-D CRS（EPSG:4979、投影 3D、WKT1 COMPD_CS＋VERT_CS "Ellipsoid"）宣告的是橢球高；
    之前 sub_crs_list 為空 ⇒ 回 None ⇒ 被當正高、note 還寫「依 SRTM 慣例」。"""
    pytest.importorskip("pyproj")
    from pyproj import CRS
    from pyproj.crs import CompoundCRS
    from sensors.dem_terrain import _vertical_datum_from_wkt
    v, note = _vertical_datum_from_wkt(CRS.from_epsg(4979).to_wkt())
    assert v == "ellipsoid" and "橢球高" in note and "Geographic 3D" in note and "SRTM" not in note
    v, note = _vertical_datum_from_wkt(_WKT1_COMPD_ELLIPSOID)
    assert v == "ellipsoid" and "SRTM" not in note
    # 帶 TOWGS84 的同一份 WKT1 ⇒ PROJ 給 BoundCRS；note 報的是被包住的 source CRS 型別（不是 "Bound CRS"）
    bound = _WKT1_COMPD_ELLIPSOID.replace('AUTHORITY["EPSG","6326"]]', 'TOWGS84[1,2,3,0,0,0,0]]', 1)
    assert CRS.from_user_input(bound).is_bound
    v, note = _vertical_datum_from_wkt(bound)
    assert v == "ellipsoid" and "Geographic 3D CRS" in note and "Bound" not in note
    assert _vertical_datum_from_wkt(CRS.from_epsg(32651).to_3d().to_wkt())[0] == "ellipsoid"   # 投影 3D
    # 對照：正高複合 CRS 仍是 msl；2-D 仍是未宣告；讀不了 ⇒ 未宣告並寫明
    egm = CompoundCRS("WGS84+EGM96", [CRS.from_epsg(4326), CRS.from_epsg(5773)]).to_wkt()
    assert _vertical_datum_from_wkt(egm) == ("msl", "DEM 垂直 CRS = EGM96（正高）")
    assert _vertical_datum_from_wkt(CRS("EPSG:3826+8904").to_wkt()) == ("msl", "DEM 垂直 CRS = TWVD2001（正高）")
    v, note = _vertical_datum_from_wkt(CRS.from_epsg(4326).to_wkt())
    assert v is None and "無垂直 CRS" in note
    v, note = _vertical_datum_from_wkt("這不是 WKT")
    assert v is None and "讀不了" in note


def test_rasterio_loader_reads_the_datum_from_a_geotiff(tmp_path):
    """DEMTerrainManager 的 rasterio 路徑（load_dem 的第一順位）：GeoTIFF 的 CRS 一路變成來源宣告。"""
    pytest.importorskip("pyproj")
    rasterio = pytest.importorskip("rasterio")
    from rasterio.transform import from_origin
    from sensors.dem_terrain import DEMTerrainManager
    z = np.full((40, 40), GROUND_ELL, np.float32)

    def tif(name, crs):
        p = tmp_path / f"{name}.tif"
        with rasterio.open(p, "w", driver="GTiff", height=40, width=40, count=1, dtype="float32",
                           crs=rasterio.crs.CRS.from_user_input(crs),
                           transform=from_origin(HOME[1] - 0.1, HOME[0] + 0.1, 0.005, 0.005)) as dst:
            dst.write(z, 1)
        m = DEMTerrainManager(default_elevation=-1.0)
        assert m.load_dem(str(p)) and m._loaded
        return m

    m = tif("ell3d", "EPSG:4979")
    assert m.vertical_datum == "ellipsoid" and "橢球高" in m.vertical_datum_note
    assert m.get_elevation(*HOME) == pytest.approx(GROUND_ELL, abs=1e-3)
    d = terrain_func_to_dem(m.get_elevation, _B, 500.0)
    assert d.vertical_datum == "ellipsoid" and d.geoid_undulation_m is None and not d.msl_known
    assert tif("egm96", "EPSG:4326+5773").vertical_datum == "msl"
    m2 = tif("srtm", "EPSG:4326")
    assert m2.vertical_datum is None and "SRTM" in m2.vertical_datum_note


def test_dem_terrain_manager_numpy_metadata_reaches_the_adapter(tmp_path):
    from sensors.dem_terrain import DEMTerrainManager
    z = np.full((50, 50), GROUND_ELL)
    np.save(tmp_path / "d.npy", z)
    (tmp_path / "d.json").write_text(json.dumps({"lat_min": HOME[0] - 0.05, "lat_max": HOME[0] + 0.05,
                                                 "lon_min": HOME[1] - 0.05, "lon_max": HOME[1] + 0.05,
                                                 "vertical_datum": "ellipsoidal"}), encoding="utf-8")
    m = DEMTerrainManager(default_elevation=0.0)
    assert m._load_numpy(str(tmp_path / "d.npy"))
    assert m.vertical_datum == "ellipsoid" and "橢球高" in m.vertical_datum_note
    d = terrain_func_to_dem(m.get_elevation, _B, 500.0)
    assert d.vertical_datum == "ellipsoid" and not d.msl_known
    m2 = DEMTerrainManager()
    (tmp_path / "e.json").write_text(json.dumps({"lat_min": 0, "lat_max": 1, "lon_min": 0, "lon_max": 1}),
                                     encoding="utf-8")
    np.save(tmp_path / "e.npy", z)
    assert m2._load_numpy(str(tmp_path / "e.npy")) and m2.vertical_datum is None
    assert "未宣告" in m2.vertical_datum_note
