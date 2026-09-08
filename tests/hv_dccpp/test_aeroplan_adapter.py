"""tests/test_aeroplan_adapter.py — `hv_dccpp.integration.aeroplan_adapter` 之整合測試。

情境：平地、單一 3 km × 1.2 km 矩形任務區、1 架構型 A（Pioneer PNP + LiDAR）＋ 1 架構型 B（HERO PNP + 相機），
home 位於區域西側 4 km。以模組層級 fixture 只跑**一次**真實規劃（≈ 1–2 s），其餘測試共用其結果。

驗證重點（對應 AeroPlan Studio 的消費端契約）
--------------------------------------------
* ``plan_coverage_dccpp`` 的簽章與 REPO/mission/swarm_coordinator.py:975 完全相同，回傳 6 鍵字典
* ``assembled_paths`` 的每機航點非空、``alt`` 為相對 home 的高度且 ≥ 0、
  ``segment_type.name`` 只出現 TAKEOFF/ENTRY/OPERATION/TRANSFER/LANDING
* 五個 ``*_length_m`` 齊全且 ``total_length_m`` 為其和
* ``heading_compass_deg`` 與 ``heading_deg`` 一致且在 ``[0, 360)``
* ``to_upload_tuples`` 的 8-tuple 可被 pymavlink 常數解讀，且 VTOL 起降指令齊全
* ``fence_zones_to_nofly`` 對圓形與多邊形皆正確
* QGC WPL 110 行為 12 欄（格式同 REPO/utils/file_io.py:222 ``create_waypoint_line``）
"""
from __future__ import annotations

import math
import time

import pytest

from hv_dccpp.integration import aeroplan_adapter as ad
from hv_dccpp.integration.aeroplan_adapter import (
    AeroPlanAdapterError, HvBuiltPath, HvBuiltWaypoint, HvDccppCoordinator, HvSegmentLabel, PHASE_TO_SEGMENT,
    SEGMENT_NAMES, fence_zones_to_nofly, plan_to_aeroplan_waypoints, telemetry_frames_to_events,
    terrain_func_to_dem, to_planner_config, to_upload_tuples,
)
from hv_dccpp.mission import Phase, PlannerConfig as HvPlannerConfig

# ---------------------------------------------------------------------------
# 情境（與 scenario.build_mini_example 同尺度：ENU (4000,-600)→(7000,600)）
# ---------------------------------------------------------------------------
HOME_LATLON = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME_LATLON[0]))
AREA_ENU = (4000.0, -600.0, 7000.0, 600.0)
COVERAGE_ALT_M = 160.0
OVERLAP = 0.25
FOV_M = 160.0


def _ll(x_m: float, y_m: float) -> tuple[float, float]:
    """ENU (東, 北) 公尺 → (lat, lon)（近似；測試情境用）。"""
    return (HOME_LATLON[0] + y_m / _M_LAT, HOME_LATLON[1] + x_m / _M_LON)


def _areas() -> list[dict]:
    x0, y0, x1, y1 = AREA_ENU
    return [{"area_id": 1, "polygon": [_ll(x0, y0), _ll(x1, y0), _ll(x1, y1), _ll(x0, y1)], "priority": 1.0}]


def _drones() -> list[dict]:
    """AeroPlan 於 REPO/ui/main_window.py:4721-4744 組裝的 drone dict 形狀。"""
    return [
        {"drone_id": 1, "name": "UAV-1", "position": HOME_LATLON, "speed": 21.5, "heading": 0.0,
         "turn_radius": 200.0, "vehicle_type": "vtol", "fov_width": FOV_M},
        {"drone_id": 2, "name": "UAV-2", "position": HOME_LATLON, "speed": 19.0, "heading": 0.0,
         "turn_radius": 120.0, "vehicle_type": "vtol", "fov_width": FOV_M},
    ]


def _class_of(d: dict) -> str:
    """drone_id 1 → 構型 A（Pioneer + LiDAR）；2 → 構型 B（HERO + 相機）。"""
    return "A" if int(d["drone_id"]) == 1 else "B"


@pytest.fixture(scope="module")
def planned() -> tuple[HvDccppCoordinator, dict, float]:
    """跑一次真實規劃；回傳 (coordinator, 6 鍵結果, 耗時秒)。"""
    prog: list[tuple[float, str]] = []
    reports: list[dict] = []
    coord = HvDccppCoordinator(class_of=_class_of,
                               progress=lambda f, m: prog.append((f, m)),
                               on_report=reports.append)
    t0 = time.perf_counter()
    res = coord.plan_coverage_dccpp(areas=_areas(), drones=_drones(), coverage_width_m=FOV_M,
                                    overlap_rate=OVERLAP, coverage_altitude=COVERAGE_ALT_M)
    dt = time.perf_counter() - t0
    coord._test_progress = prog        # type: ignore[attr-defined]
    coord._test_reports = reports      # type: ignore[attr-defined]
    return coord, res, dt


# ===========================================================================
# 1. 主接縫：簽章與 6 鍵回傳
# ===========================================================================

def test_signature_matches_swarm_coordinator() -> None:
    """簽章必須與 REPO/mission/swarm_coordinator.py:975 逐字相同（參數名與順序、預設值）。"""
    import inspect
    sig = inspect.signature(HvDccppCoordinator.plan_coverage_dccpp)
    names = [p for p in sig.parameters if p != "self"]
    assert names == ["areas", "drones", "coverage_width_m", "overlap_rate", "auto_scan_angle",
                     "coverage_altitude", "terrain_func", "enable_altitude", "mounting_angle_deg",
                     "dem_path", "coordination_mode", "runway_bearing_deg", "runway_length_m",
                     "landing_bearing_deg", "landing_rollout_m"]
    p = sig.parameters
    assert p["overlap_rate"].default == 0.1
    assert p["auto_scan_angle"].default is True
    assert p["coverage_altitude"].default == 100.0
    assert p["terrain_func"].default is None
    assert p["enable_altitude"].default is True
    assert p["mounting_angle_deg"].default == 0.0
    assert p["dem_path"].default == ""
    assert p["coordination_mode"].default == "uncoordinated"
    assert p["runway_bearing_deg"].default is None
    assert p["runway_length_m"].default is None
    assert p["landing_bearing_deg"].default is None
    assert p["landing_rollout_m"].default == 0.0
    # 全部為位置或關鍵字（AeroPlan 以關鍵字呼叫，REPO/ui/main_window.py:4760-4776）
    assert all(p[n].kind is inspect.Parameter.POSITIONAL_OR_KEYWORD for n in names)


def test_returns_six_keys(planned) -> None:
    """6 鍵齊全、型別正確（REPO/mission/swarm_coordinator.py:1291-1298）。"""
    _, res, _ = planned
    assert set(res) == {"results", "allocation", "coverage_paths", "uav_states",
                        "total_makespan", "assembled_paths"}
    assert isinstance(res["results"], dict)
    assert isinstance(res["allocation"], dict)
    assert isinstance(res["coverage_paths"], list)
    assert isinstance(res["uav_states"], list)
    assert isinstance(res["total_makespan"], float)
    assert isinstance(res["assembled_paths"], dict)
    assert res["total_makespan"] > 0.0
    # area_id 為 int（AeroPlan 以整數 area_id 索引）
    assert all(isinstance(k, int) for k in res["results"])
    assert all(isinstance(k, int) for k in res["allocation"])
    assert all(isinstance(k, int) for k in res["assembled_paths"])


def test_planning_is_fast(planned) -> None:
    """單次規劃 < 30 s（GUI 可接受；實測約 1–2 s）。"""
    _, _, dt = planned
    assert dt < 30.0


def test_both_configurations_fly(planned) -> None:
    """異構機隊：A 與 B 都要被分配到工作（回歸測試——payload_reqs 只給一種 kind 時 B 會完全落空）。"""
    _, res, _ = planned
    classes = {getattr(bp, "cls", "") for bp in res["assembled_paths"].values()}
    assert classes == {"A", "B"}
    assert sorted(res["allocation"][1]) == [1, 2]


def test_progress_and_report_callbacks(planned) -> None:
    """progress 單調遞增至 1.0；on_report 恰呼叫一次且含狀態欄位。"""
    coord, _, _ = planned
    prog = coord._test_progress      # type: ignore[attr-defined]
    reports = coord._test_reports    # type: ignore[attr-defined]
    assert prog and prog[-1][0] == pytest.approx(1.0)
    fr = [f for f, _ in prog]
    assert fr == sorted(fr)
    assert len(reports) == 1
    assert reports[0]["status"] in ("GO", "GO_WITH_WARNINGS")
    assert set(reports[0]) >= {"status", "errors", "warnings", "notes", "makespan_s", "timing_ms", "uncovered"}


# ===========================================================================
# 2. assembled_paths（BuiltPath / BuiltWaypoint duck typing）
# ===========================================================================

def test_assembled_paths_waypoints(planned) -> None:
    """每機航點非空；lat/lon 落在合理範圍；alt 為相對 home 高度且 ≥ 0。"""
    _, res, _ = planned
    for did, bp in res["assembled_paths"].items():
        assert bp.uav_id == did
        assert len(bp.waypoints) > 0, f"drone {did} 無航點"
        alts = [w.alt for w in bp.waypoints]
        assert min(alts) >= 0.0, "alt 必須為相對 home 的 AGL（frame 3），不可為負"
        # 平地 DEM 基準 0 m MSL，作業高度約 COVERAGE_ALT_M；上限為豁免天花板 200 m
        assert max(alts) <= 200.0 + 1e-6
        assert max(alts) > 50.0, "應有巡航／作業高度航點"
        for w in bp.waypoints:
            assert 23.0 < w.lat < 25.0 and 120.0 < w.lon < 122.0


def test_segment_labels_only_five_strings(planned) -> None:
    """``segment_type.name`` 只能是那 5 個字串（REPO/ui/main_window.py:6060-6066 以字串比對）。"""
    _, res, _ = planned
    seen: set[str] = set()
    for bp in res["assembled_paths"].values():
        for w in bp.waypoints:
            assert w.segment_type.name in SEGMENT_NAMES
            seen.add(w.segment_type.name)
    assert seen == set(SEGMENT_NAMES), f"五種標籤都應出現，實得 {sorted(seen)}"


def test_phase_map_covers_all_14_phases() -> None:
    """14 種 Phase 全部有對應，且值域只有 5 種標籤。"""
    assert set(PHASE_TO_SEGMENT) == set(Phase)
    assert {v.name for v in PHASE_TO_SEGMENT.values()} == set(SEGMENT_NAMES)
    assert PHASE_TO_SEGMENT[Phase.TRANS_OUT] is HvSegmentLabel.TAKEOFF
    assert PHASE_TO_SEGMENT[Phase.TRANS_IN] is HvSegmentLabel.LANDING
    assert PHASE_TO_SEGMENT[Phase.HOLD] is HvSegmentLabel.TRANSFER
    assert PHASE_TO_SEGMENT[Phase.ORBIT] is HvSegmentLabel.ENTRY


def test_five_lengths_and_total(planned) -> None:
    """五個 ``*_length_m`` 都存在、非負，且 ``total_length_m`` 為其和。"""
    _, res, _ = planned
    keys = ("takeoff_length_m", "entry_length_m", "operation_length_m",
            "transfer_length_m", "landing_length_m")
    for bp in res["assembled_paths"].values():
        parts = [getattr(bp, k) for k in keys]
        assert all(isinstance(v, float) and v >= 0.0 for v in parts)
        assert bp.total_length_m == pytest.approx(sum(parts), rel=1e-12)
        assert bp.operation_length_m > 0.0, "必須有掃描段"
        assert bp.landing_length_m > 0.0, "必須有回收段"
        assert bp.total_length_m > 1000.0


def test_heading_fields_consistent(planned) -> None:
    """``heading_compass_deg`` 與 ``heading_deg`` 一致，且在 [0, 360)。"""
    _, res, _ = planned
    for bp in res["assembled_paths"].values():
        for w in bp.waypoints:
            assert w.heading_deg == w.heading_compass_deg
            assert 0.0 <= w.heading_compass_deg < 360.0


def test_builtpath_helper_methods(planned) -> None:
    """``to_latlon`` / ``to_latlon_list`` / ``to_latlon_3d_list`` 三個消費端方法皆可用。"""
    _, res, _ = planned
    bp = next(iter(res["assembled_paths"].values()))
    ll = bp.to_latlon()
    assert ll == bp.to_latlon_list()
    assert len(ll) == len(bp.waypoints)
    assert all(len(p) == 2 for p in ll)
    ll3 = bp.to_latlon_3d_list()
    assert len(ll3) == len(ll) and all(len(p) == 3 for p in ll3)
    assert bp.waypoints[0].to_tuple() == ll[0]


def test_duck_typing_getattr_pattern(planned) -> None:
    """AeroPlan 消費端的 getattr 寫法（REPO/ui/main_window.py:5308、:5361）都取得到值。"""
    _, res, _ = planned
    for bp in res["assembled_paths"].values():
        assert getattr(bp, "takeoff_length_m", None) is not None
        assert getattr(bp, "landing_length_m", None) is not None
        w = bp.waypoints[0]
        h = getattr(w, "heading_compass_deg", getattr(w, "heading_deg", 0.0))
        assert isinstance(h, float)


# ===========================================================================
# 3. coverage_paths / results / uav_states
# ===========================================================================

def test_coverage_paths(planned) -> None:
    """每區一個 CoveragePath；operations 端點合理、全部被覆蓋。"""
    _, res, _ = planned
    cps = res["coverage_paths"]
    assert len(cps) == 1
    cp = cps[0]
    assert cp.area_id == 1
    assert len(cp.area_polygon) == 4
    assert 0.0 <= cp.scan_angle_deg < 360.0
    assert len(cp.operations) > 0
    assert all(op.covered for op in cp.operations), "本情境應完全覆蓋"
    for op in cp.operations:
        assert op.length_m > 0.0
        assert op.assigned_drone_id in (1, 2)
        lp, rp = op.entry_points()
        assert lp == op.left_point and rp == op.right_point
        assert lp != rp


def test_results_mdtsp(planned) -> None:
    """MDTSPResult 欄位齊全；uav_assignments / total_distances 對得上。"""
    _, res, _ = planned
    r = res["results"][1]
    assert r.area_id == 1
    assert set(r.uav_assignments) == {1, 2}
    assert set(r.total_distances) == {1, 2}
    for did, ops in r.uav_assignments.items():
        assert ops, f"drone {did} 無作業段"
        assert r.total_distances[did] == pytest.approx(sum(o.length_m for o in ops))
    assert r.makespan > 0.0
    assert r.assembled_paths is not None and set(r.assembled_paths) == {1, 2}


def test_uav_states(planned) -> None:
    """UAVState 逐機：轉彎半徑逐構型（A 200 m > B 120 m）、vehicle_type 不得為 MULTIROTOR。"""
    _, res, _ = planned
    st = {s.uav_id: s for s in res["uav_states"]}
    assert set(st) == {1, 2}
    assert st[1].turn_radius > st[2].turn_radius, "A（Pioneer）之 R_eff 應大於 B（HERO）"
    assert st[1].speed == pytest.approx(21.5)
    assert st[2].speed == pytest.approx(19.0)
    for s in st.values():
        assert s.vehicle_type.name in ("FIXED_WING", "VTOL")
        assert 0.0 <= s.heading < 360.0
        assert s.fov_width > 0.0
        assert len(s.position) == 2


# ===========================================================================
# 4. to_upload_tuples（SITLLink.upload_mission 8-tuple）
# ===========================================================================

def test_upload_tuples_shape_and_commands(planned) -> None:
    """8-tuple 形狀；指令碼可被 pymavlink 常數解讀；VTOL 起降指令齊全。"""
    pymavlink = pytest.importorskip("pymavlink")
    from pymavlink import mavutil
    m = mavutil.mavlink
    coord, res, _ = planned
    tuples_by_drone = coord.upload_tuples()
    assert set(tuples_by_drone) == set(res["assembled_paths"])
    for did, tl in tuples_by_drone.items():
        assert tl, f"drone {did} 無上傳項"
        for t in tl:
            assert isinstance(t, tuple) and len(t) == 8, "sitl_link._send_mission 需要 8-tuple"
            lat, lon, alt, cmd, p1, p2, p3, p4 = t
            assert all(isinstance(v, float) for v in (lat, lon, alt, p1, p2, p3, p4))
            assert isinstance(cmd, int)
        cmds = [t[3] for t in tl]
        # pymavlink 常數比對（不手寫數字）
        assert cmds[0] == m.MAV_CMD_DO_CHANGE_SPEED
        assert m.MAV_CMD_NAV_VTOL_TAKEOFF in cmds
        assert m.MAV_CMD_NAV_VTOL_LAND in cmds
        assert cmds.count(m.MAV_CMD_DO_VTOL_TRANSITION) >= 2
        assert cmds[-1] == m.MAV_CMD_NAV_VTOL_LAND
        assert set(cmds) <= {m.MAV_CMD_DO_CHANGE_SPEED, m.MAV_CMD_NAV_VTOL_TAKEOFF,
                             m.MAV_CMD_DO_VTOL_TRANSITION, m.MAV_CMD_NAV_WAYPOINT,
                             m.MAV_CMD_NAV_VTOL_LAND}
        # 轉換方向：起飛後 → FW(4)，降落前 → MC(3)
        trans = [t for t in tl if t[3] == m.MAV_CMD_DO_VTOL_TRANSITION]
        assert trans[0][4] == pytest.approx(float(m.MAV_VTOL_STATE_FW))
        assert trans[-1][4] == pytest.approx(float(m.MAV_VTOL_STATE_MC))
        # 所有指令都在 AeroPlan mission_validator 白名單內（REPO/mission/mission_validator.py:38-51）
        assert set(cmds) <= {16, 21, 22, 84, 85, 178, 3000}


def test_upload_tuples_frame_selection(planned) -> None:
    """DO_* 項的 lat/lon 為 0（sitl_link.py:1270-1274 會給 MAV_FRAME_MISSION）；NAV 項有真座標。"""
    from pymavlink import mavutil
    m = mavutil.mavlink
    coord, _, _ = planned
    mission_frame_cmds = (93, 112, 113, 114, 178, 179, 177, 20, 115, 3000)
    for tl in coord.upload_tuples().values():
        for lat, lon, alt, cmd, *_ in tl:
            if cmd in mission_frame_cmds:
                assert lat == 0.0 and lon == 0.0 and alt == 0.0
            else:
                assert abs(lat) > 1e-6 and abs(lon) > 1e-6
                assert alt >= 0.0
        # 第一個具有效經緯的項會被複製為 Home（sitl_link.py:1237-1245）
        assert any(abs(t[0]) > 1e-9 for t in tl)


def test_upload_tuples_fixed_wing_variant(planned) -> None:
    """``is_vtol=False`` → NAV_TAKEOFF / NAV_LAND（AeroPlan 的固定翼分支）。"""
    from pymavlink import mavutil
    m = mavutil.mavlink
    _, res, _ = planned
    bp = next(iter(res["assembled_paths"].values()))
    tl = to_upload_tuples(bp, is_vtol=False, cruise_speed_mps=20.0, turn_radius_m=150.0)
    cmds = [t[3] for t in tl]
    assert m.MAV_CMD_NAV_TAKEOFF in cmds and cmds[-1] == m.MAV_CMD_NAV_LAND
    assert m.MAV_CMD_NAV_VTOL_TAKEOFF not in cmds


def test_upload_tuples_empty_path() -> None:
    """空路徑 → 空串列（不得丟例外）。"""
    assert to_upload_tuples(HvBuiltPath(uav_id=9)) == []


# ===========================================================================
# 5. fence_zones_to_nofly
# ===========================================================================

class _FakeZone:
    """AeroPlan ``FenceZone``（REPO/mission/fence_zone.py:48）之最小 duck-typed 替身。"""

    def __init__(self, **kw) -> None:
        self.__dict__.update(kw)


def test_fence_polygon_zone() -> None:
    """多邊形 NFZ → NoFlyZone；頂點與樓層／天花板保留。"""
    verts = [_ll(1000.0, 1000.0), _ll(1600.0, 1000.0), _ll(1600.0, 1600.0), _ll(1000.0, 1600.0)]
    z = _FakeZone(id="nfz1", name="N1", category="NFZ", shape="POLYGON", vertices=verts,
                  alt_min_m=10.0, alt_max_m=180.0)
    (nf,) = fence_zones_to_nofly([z])
    assert nf.zone_id == "nfz1"
    assert len(nf.polygon_wgs84) == 4
    assert nf.polygon_wgs84[0] == pytest.approx(verts[0])
    assert nf.z_floor_msl_m == pytest.approx(10.0)
    assert nf.z_ceiling_msl_m == pytest.approx(180.0)


def test_fence_circle_zone() -> None:
    """圓形 NFZ → N 邊形；所有頂點距圓心約等於半徑。"""
    center = _ll(2000.0, 0.0)
    z = _FakeZone(id="nfz2", name="N2", category="NFZ", shape="CIRCLE", center=center, radius_m=300.0,
                  vertices=[], alt_min_m=0.0, alt_max_m=150.0)
    (nf,) = fence_zones_to_nofly([z], circle_segments=24)
    assert len(nf.polygon_wgs84) == 24
    for lat, lon in nf.polygon_wgs84:
        dy = (lat - center[0]) * _M_LAT
        dx = (lon - center[1]) * _M_LON
        assert math.hypot(dx, dy) == pytest.approx(300.0, rel=0.02)


def test_fence_uses_polygon_vertices_method() -> None:
    """有 ``polygon_vertices(circle_segments)`` 方法時優先使用之（原型 :87）。"""
    ring = [_ll(0.0, 0.0), _ll(500.0, 0.0), _ll(500.0, 500.0)]

    class _WithMethod(_FakeZone):
        def polygon_vertices(self, circle_segments: int = 16):
            return list(ring)

    (nf,) = fence_zones_to_nofly([_WithMethod(id="z", category="THREAT", shape="CIRCLE",
                                              center=_ll(0.0, 0.0), radius_m=100.0)])
    assert len(nf.polygon_wgs84) == 3


def test_fence_skips_geofence_and_degenerate() -> None:
    """GEOFENCE（inclusion）與頂點不足者略過；``None`` → 空元組。"""
    good = _FakeZone(id="a", category="NFZ", shape="POLYGON",
                     vertices=[_ll(0, 0), _ll(100, 0), _ll(100, 100)])
    geo = _FakeZone(id="b", category="GEOFENCE", shape="POLYGON",
                    vertices=[_ll(0, 0), _ll(200, 0), _ll(200, 200)])
    bad = _FakeZone(id="c", category="NFZ", shape="POLYGON", vertices=[_ll(0, 0)])
    out = fence_zones_to_nofly([good, geo, bad])
    assert [z.zone_id for z in out] == ["a"]
    assert fence_zones_to_nofly(None) == ()


def test_fence_agl_reference_shifts_by_home_msl() -> None:
    """``alt_ref='agl'`` → 樓層／天花板加上 home MSL（NoFlyZone 一律 MSL）。"""
    z = _FakeZone(id="a", category="NFZ", shape="POLYGON",
                  vertices=[_ll(0, 0), _ll(100, 0), _ll(100, 100)], alt_min_m=0.0, alt_max_m=120.0)
    (nf,) = fence_zones_to_nofly([z], alt_ref="agl", home_msl_m=250.0)
    assert nf.z_floor_msl_m == pytest.approx(250.0)
    assert nf.z_ceiling_msl_m == pytest.approx(370.0)


def test_fence_zones_reach_the_planner() -> None:
    """建構子的 fence_zones 真的進到 MissionInputs.airspace.no_fly。"""
    z = _FakeZone(id="nfz1", category="NFZ", shape="CIRCLE", center=_ll(2000.0, 0.0), radius_m=200.0,
                  vertices=[], alt_min_m=0.0, alt_max_m=200.0)
    coord = HvDccppCoordinator(class_of=_class_of, fence_zones=[z])
    inputs, id_map = ad.to_mission_inputs(_areas(), _drones(), coverage_width_m=FOV_M, overlap_rate=OVERLAP,
                                          coverage_altitude=COVERAGE_ALT_M, fence_zones=coord.fence_zones,
                                          class_of=_class_of)
    assert [n.zone_id for n in inputs.airspace.no_fly] == ["nfz1"]
    assert id_map == {1: "A1", 2: "B2"}


# ===========================================================================
# 6. terrain_func_to_dem
# ===========================================================================

def test_terrain_func_to_dem_flat() -> None:
    """``terrain_func=None`` → 平地 DEM。"""
    dem = terrain_func_to_dem(None, (23.94, 120.89, 23.97, 120.93), step_m=200.0, base_msl_m=42.0)
    assert dem.rows >= 2 and dem.cols >= 2
    assert float(dem.sample_msl(23.95, 120.90)[0]) == pytest.approx(42.0)
    assert dem.sigma_z_m == pytest.approx(2.0)


def test_terrain_func_to_dem_sampled() -> None:
    """回呼被取樣；線性坡面於節點與內插點皆正確；例外／nan 以 base 填補。"""
    calls: list[tuple[float, float]] = []

    def f(lat: float, lon: float) -> float:
        calls.append((lat, lon))
        if lat > 23.965:
            raise RuntimeError("模擬 DEM 讀取失敗")
        return 100.0 + (lat - 23.94) * _M_LAT * 0.01     # 1% 北向坡

    dem = terrain_func_to_dem(f, (23.94, 120.89, 23.97, 120.93), step_m=150.0, base_msl_m=7.0)
    assert calls
    assert float(dem.sample_msl(23.94, 120.90)[0]) == pytest.approx(100.0, abs=1.0)
    assert float(dem.sample_msl(23.95, 120.90)[0]) == pytest.approx(100.0 + 0.01 * _M_LAT * 0.01, abs=2.0)
    assert float(dem.sample_msl(23.9699, 120.90)[0]) == pytest.approx(7.0, abs=1e-3)


def test_terrain_func_to_dem_node_cap() -> None:
    """節點數上限會自動放大步長，不丟例外。"""
    dem = terrain_func_to_dem(None, (23.0, 120.0, 25.0, 122.0), step_m=10.0, max_nodes=10_000)
    assert dem.rows * dem.cols <= 10_000


def test_terrain_func_to_dem_bad_bounds() -> None:
    with pytest.raises(AeroPlanAdapterError):
        terrain_func_to_dem(None, (24.0, 120.0, 23.0, 121.0), step_m=100.0)


def test_terrain_func_reaches_planner() -> None:
    """``plan_coverage_dccpp(terrain_func=...)`` 真的建出非平地 DEM。"""
    # 緩坡起伏（振幅 15 m、波長 ≈ 6 km → 最大坡度 < 1°），確保站點坡度符合 cfg.site_max_slope_deg
    def f(lat: float, lon: float) -> float:
        return 50.0 + 15.0 * math.sin((lon - 120.90) * _M_LON * 2.0 * math.pi / 6000.0)

    coord = HvDccppCoordinator(class_of=_class_of, dem_step_m=100.0)
    coord.plan_coverage_dccpp(areas=_areas(), drones=_drones(), coverage_width_m=FOV_M,
                              overlap_rate=OVERLAP, coverage_altitude=COVERAGE_ALT_M, terrain_func=f)
    dem = coord.last_inputs.dem
    assert float(dem.z_msl.max() - dem.z_msl.min()) > 5.0


# ===========================================================================
# 7. QGC WPL 110
# ===========================================================================

def test_qgc_lines_format(planned) -> None:
    """QGC WPL 110：首行標頭；資料行 12 欄、tab 分隔；lat/lon 6 位、alt 2 位；frame=3。"""
    coord, res, _ = planned
    per_uav = plan_to_aeroplan_waypoints(coord.last_plan, coord.last_ctx, coord.cfg)
    assert set(per_uav) == {"A1", "B2"}
    for uid, lines in per_uav.items():
        assert lines[0] == "QGC WPL 110"
        assert len(lines) > 5
        seqs: list[int] = []
        for ln in lines[1:]:
            cols = ln.split("\t")
            assert len(cols) == 12, f"{uid}: 須 12 欄（同 REPO/utils/file_io.py:222）"
            seq, current, frame, cmd = int(cols[0]), int(cols[1]), int(cols[2]), int(cols[3])
            seqs.append(seq)
            assert current in (0, 1)
            assert frame in (0, 2, 3), "NAV 用 3（GLOBAL_RELATIVE_ALT）、DO_ 用 2（MISSION）、home 用 0"
            assert cmd > 0
            assert len(cols[8].split(".")[1]) == 6      # lat 6 位小數
            assert len(cols[9].split(".")[1]) == 6      # lon 6 位小數
            assert len(cols[10].split(".")[1]) == 2     # alt 2 位小數
            assert int(cols[11]) in (0, 1)
        assert seqs == sorted(seqs)


def test_qgc_lines_have_vtol_commands(planned) -> None:
    """QGC 匯出含 QuadPlane 起降指令（AeroPlan MAVCommand 白名單 REPO/mission/waypoint.py:46-49）。"""
    coord, _, _ = planned
    per_uav = coord.qgc_waypoints(key="drone_id")
    assert set(per_uav) == {"1", "2"}
    for lines in per_uav.values():
        cmds = {int(ln.split("\t")[3]) for ln in lines[1:]}
        assert 84 in cmds and 85 in cmds


# ===========================================================================
# 8. to_planner_config
# ===========================================================================

def test_to_planner_config_maps_two_keys() -> None:
    """``get_dccpp_params()`` 的 22 鍵中只有兩鍵落在 PlannerConfig 上。"""
    params = {
        "vehicle_type": "vtol", "num_drones": 3, "fov_width_m": 100.0, "overlap_rate": 0.1,
        "altitude": 100.0, "speed": 15.0, "turn_radius": 50.0, "auto_scan_angle": True,
        "enable_altitude": True, "auto_landing": True, "mounting_angle_deg": 0.0, "hfov_deg": 60.0,
        "vfov_deg": 45.0, "coordination_mode": "uncoordinated", "dem_path": "",
        "takeoff_bearing": 0.0, "runway_length": 500.0, "landing_bearing": 180.0, "pattern_alt": 80.0,
        "landing_rollout": 0.0, "takeoff_spacing": 50.0, "collision_avoidance": True,
        "min_separation_m": 175.0, "alt_offset_m": 65.0, "avoid_strategy": "altitude",
    }
    cfg = to_planner_config(params)
    assert isinstance(cfg, HvPlannerConfig)
    assert cfg.d_sep_m == pytest.approx(175.0)
    assert cfg.dh_sep_m == pytest.approx(65.0)
    # 其餘欄位維持預設
    base = HvPlannerConfig()
    changed = {k for k in base.to_dict() if getattr(cfg, k) != getattr(base, k)}
    assert changed == {"d_sep_m", "dh_sep_m"}
    # 每個 UI 鍵都必須在說明表中有交代（手冊逐鍵引用）
    assert set(params) <= set(ad.PARAM_KEY_NOTES)


def test_to_planner_config_hv_cfg_override() -> None:
    """``hv_cfg`` 可直接覆寫任意 PlannerConfig 欄位；未知鍵丟 ValueError。"""
    cfg = to_planner_config({"hv_cfg": {"seed": 7, "B": 2, "n_local_search": 20}})
    assert (cfg.seed, cfg.B, cfg.n_local_search) == (7, 2, 20)
    assert to_planner_config(None) == HvPlannerConfig()
    with pytest.raises(ValueError):
        to_planner_config({"hv_cfg": {"no_such_field": 1}})


# ===========================================================================
# 9. telemetry_frames_to_events
# ===========================================================================

class _FakeFrame:
    """AeroPlan ``TelemetryFrame``（REPO/mission/sitl_link.py:30-62）之最小 duck-typed 替身。"""

    def __init__(self, **kw) -> None:
        self.sysid = 1
        self.lat = 0.0
        self.lon = 0.0
        self.alt_msl = 0.0
        self.alt_rel = 0.0
        self.heading = 0.0
        self.ground_speed = 0.0
        self.air_speed = 0.0
        self.battery_pct = -1
        self.mode = "AUTO"
        self.armed = True
        self.last_update = 0.0
        self.__dict__.update(kw)


def test_telemetry_link_loss_and_restore(planned) -> None:
    """last_update 逾時 → LinkLoss；上一輪失聯而本輪正常 → LinkRestored。"""
    from hv_dccpp.dynamic import LinkLoss, LinkRestored
    coord, _, _ = planned
    now = 1000.0
    ok = _FakeFrame(sysid=1, lat=HOME_LATLON[0], lon=HOME_LATLON[1], alt_msl=160.0, alt_rel=160.0,
                    heading=90.0, ground_speed=21.0, air_speed=21.5, battery_pct=80, last_update=now)
    stale = _FakeFrame(sysid=2, lat=HOME_LATLON[0], lon=HOME_LATLON[1], alt_msl=160.0, alt_rel=160.0,
                       heading=90.0, ground_speed=19.0, air_speed=19.0, battery_pct=80,
                       last_update=now - 60.0)
    ev = telemetry_frames_to_events([ok, stale], coord.last_plan, coord.last_ctx, coord.cfg,
                                    id_map=coord.id_map, t0_epoch_s=now, now_s=now)
    assert any(isinstance(e, LinkLoss) and e.uav_id == "B2" for e in ev)
    assert not any(isinstance(e, LinkLoss) and e.uav_id == "A1" for e in ev)
    ev2 = telemetry_frames_to_events([ok], coord.last_plan, coord.last_ctx, coord.cfg,
                                     id_map=coord.id_map, t0_epoch_s=now, now_s=now,
                                     last_link_ok={"A1": False})
    assert any(isinstance(e, LinkRestored) and e.uav_id == "A1" for e in ev2)


def test_telemetry_energy_update(planned) -> None:
    """電量顯著低於計畫 → EnergyUpdate。"""
    from hv_dccpp.dynamic import EnergyUpdate
    coord, _, _ = planned
    now = 2000.0
    low = _FakeFrame(sysid=1, lat=HOME_LATLON[0], lon=HOME_LATLON[1], alt_msl=160.0, alt_rel=160.0,
                     heading=90.0, ground_speed=21.0, air_speed=21.5, battery_pct=5, last_update=now)
    ev = telemetry_frames_to_events([low], coord.last_plan, coord.last_ctx, coord.cfg,
                                    id_map=coord.id_map, t0_epoch_s=now, now_s=now)
    assert any(isinstance(e, EnergyUpdate) and e.uav_id == "A1" for e in ev)


def test_telemetry_returns_planner_telemetry(planned) -> None:
    """``return_telemetry=True`` 回傳可直接餵 replan 的 Telemetry；ENU 換算正確。"""
    from hv_dccpp.dynamic import Telemetry
    coord, _, _ = planned
    now = 3000.0
    f = _FakeFrame(sysid=1, lat=HOME_LATLON[0], lon=HOME_LATLON[1], alt_msl=160.0, alt_rel=160.0,
                   heading=90.0, ground_speed=21.0, air_speed=21.5, battery_pct=80, last_update=now)
    ev, tels = telemetry_frames_to_events([f], coord.last_plan, coord.last_ctx, coord.cfg,
                                          id_map=coord.id_map, t0_epoch_s=now, now_s=now,
                                          return_telemetry=True)
    assert isinstance(ev, list)
    t = tels["A1"]
    assert isinstance(t, Telemetry)
    assert abs(t.x_m) < 5.0 and abs(t.y_m) < 5.0        # 位於 home
    assert t.z_m == pytest.approx(160.0, abs=2.0)
    assert t.link_ok is True
    assert math.degrees(t.theta_rad) == pytest.approx(90.0, abs=1.0)


def test_telemetry_unknown_sysid_ignored(planned) -> None:
    """未對照到的 sysid 直接略過；空輸入回空。"""
    coord, _, _ = planned
    f = _FakeFrame(sysid=99, lat=HOME_LATLON[0], lon=HOME_LATLON[1], last_update=0.0)
    assert telemetry_frames_to_events([f], coord.last_plan, coord.last_ctx, coord.cfg,
                                      id_map=coord.id_map) == []
    assert telemetry_frames_to_events([], coord.last_plan, coord.last_ctx, coord.cfg) == []


# ===========================================================================
# 10. use_native_types / 錯誤處理
# ===========================================================================

def test_use_native_types_swaps_classes(planned) -> None:
    """以替身「原生型別」重建結果；欄位名相同，並且會嘗試補上兩個長度欄位。"""
    from dataclasses import dataclass, field
    from enum import Enum, auto

    class NativeSegmentLabel(Enum):
        TAKEOFF = auto()
        ENTRY = auto()
        OPERATION = auto()
        TRANSFER = auto()
        LANDING = auto()

    @dataclass
    class NativeBuiltWaypoint:                      # 對應 dccpp_path_builder.py:54-62（無 heading_deg）
        lat: float
        lon: float
        alt: float
        heading_compass_deg: float
        segment_type: NativeSegmentLabel

    @dataclass
    class NativeBuiltPath:                          # 對應 :65-78（只有三個長度欄位）
        uav_id: int
        waypoints: list = field(default_factory=list)
        entry_length_m: float = 0.0
        operation_length_m: float = 0.0
        transfer_length_m: float = 0.0

    coord, _, _ = planned
    reg = ad.use_native_types({"SegmentLabel": NativeSegmentLabel, "BuiltWaypoint": NativeBuiltWaypoint,
                               "BuiltPath": NativeBuiltPath}, set_default=False)
    assert reg.built_path is NativeBuiltPath
    res = ad.from_mission_plan(coord.last_plan, coord.last_ctx, areas=_areas(), drones=_drones(),
                               id_map=coord.id_map, types=reg)
    bp = next(iter(res["assembled_paths"].values()))
    assert isinstance(bp, NativeBuiltPath)
    assert isinstance(bp.waypoints[0], NativeBuiltWaypoint)
    assert isinstance(bp.waypoints[0].segment_type, NativeSegmentLabel)
    assert bp.waypoints[0].segment_type.name in SEGMENT_NAMES
    # 未支援的欄位以 setattr 補上（原生 dataclass 無 __slots__）
    assert getattr(bp, "takeoff_length_m", None) is not None
    assert getattr(bp, "landing_length_m", None) is not None
    # 預設註冊表未被更動
    assert ad.default_types.built_path is HvBuiltPath


def test_use_native_types_reset() -> None:
    assert ad.use_native_types(None).built_path is HvBuiltPath
    assert ad.default_types.built_path is HvBuiltPath


def test_default_class_of() -> None:
    """預設構型判定的各條規則。"""
    f = ad.default_class_of
    assert f({"hv_class": "b"}) == "B"
    assert f({"class": "A"}) == "A"
    assert f({"platform": "Pioneer PNP"}) == "A"
    assert f({"platform": "hero_pnp"}) == "B"
    assert f({"payload": "LiDAR"}) == "A"
    assert f({"sensor": "rgb camera"}) == "B"
    assert f({"name": "UAV-B2"}) == "B"
    assert f({"drone_id": 1}) == "A"                # 無資訊 → A（文件所述）


def test_invalid_inputs() -> None:
    """空 areas / 空 drones / 頂點不足 / class_of 回傳非法值 → AeroPlanAdapterError。"""
    coord = HvDccppCoordinator(class_of=_class_of)
    with pytest.raises(AeroPlanAdapterError):
        coord.plan_coverage_dccpp(areas=[], drones=_drones(), coverage_width_m=FOV_M)
    with pytest.raises(AeroPlanAdapterError):
        coord.plan_coverage_dccpp(areas=_areas(), drones=[], coverage_width_m=FOV_M)
    with pytest.raises(AeroPlanAdapterError):
        coord.plan_coverage_dccpp(areas=[{"area_id": 1, "polygon": [HOME_LATLON]}],
                                  drones=_drones(), coverage_width_m=FOV_M)
    bad = HvDccppCoordinator(class_of=lambda d: "C")
    with pytest.raises(AeroPlanAdapterError):
        bad.plan_coverage_dccpp(areas=_areas(), drones=_drones(), coverage_width_m=FOV_M)


def test_helper_methods_require_plan() -> None:
    coord = HvDccppCoordinator(class_of=_class_of)
    for fn in (coord.upload_tuples, coord.qgc_waypoints):
        with pytest.raises(AeroPlanAdapterError):
            fn()


def test_ignored_params_recorded(planned) -> None:
    """被忽略的 AeroPlan 參數會留下 note（手冊要列出）。"""
    coord = HvDccppCoordinator(class_of=_class_of)
    coord.plan_coverage_dccpp(areas=_areas(), drones=_drones(), coverage_width_m=FOV_M,
                              overlap_rate=OVERLAP, coverage_altitude=COVERAGE_ALT_M,
                              enable_altitude=False, auto_scan_angle=False,
                              coordination_mode="coordinated", dem_path="C:/dem.tif",
                              runway_length_m=500.0, landing_rollout_m=30.0)
    joined = " | ".join(coord.notes)
    for key in ("enable_altitude_ignored", "auto_scan_angle_ignored", "coordination_mode_ignored",
                "dem_path_ignored", "runway_length_m_ignored", "landing_rollout_m_ignored"):
        assert key in joined


def test_altitude_clamped_below_min_fw_agl() -> None:
    """coverage_altitude 低於構型 A 的 min_fw_operation_agl_m (120 m) 時會被夾住並記警告。"""
    warns: list[str] = []
    inputs, _ = ad.to_mission_inputs(_areas(), _drones(), coverage_width_m=FOV_M, overlap_rate=OVERLAP,
                                     coverage_altitude=60.0, class_of=_class_of, warnings=warns)
    assert any("coverage_altitude_clamped" in w for w in warns)
    assert inputs.payload_reqs["default"].h_target_agl_m == pytest.approx(120.0)


def test_payload_reqs_cover_every_sensor_kind() -> None:
    """異構機隊必須為每種機上感測種類產生對應 kind 的 PayloadRequirement（否則該構型無 Sensor）。"""
    inputs, _ = ad.to_mission_inputs(_areas(), _drones(), coverage_width_m=FOV_M, overlap_rate=OVERLAP,
                                     coverage_altitude=COVERAGE_ALT_M, class_of=_class_of)
    kinds = {r.kind for r in inputs.payload_reqs.values()}
    assert kinds == {"lidar", "rgb"}
    assert inputs.payload_reqs["default"].side_overlap == pytest.approx(OVERLAP)


def test_waypoint_alt_is_relative_to_home(planned) -> None:
    """``HvBuiltWaypoint.alt`` = ``alt_msl_m − home_msl``（frame 3 慣例），非對地 AGL、非 AMSL。"""
    coord, res, _ = planned
    ctx = coord.last_ctx
    home_msl = ad._home_msl(ctx)
    for uid, up in coord.last_plan.plans.items():
        did = {v: k for k, v in coord.id_map.items()}[uid]
        bp = res["assembled_paths"][did]
        # 首航點為起飛點：alt 應接近 0（相對 home）
        assert bp.waypoints[0].alt == pytest.approx(0.0, abs=2.0)
        # 巡航段的 alt 與計畫的 alt_msl_m − home_msl 同量級
        rel = [w.alt_msl_m - home_msl for w in up.waypoints]
        assert max(w.alt for w in bp.waypoints) == pytest.approx(max(rel), abs=5.0)


def test_reduce_waypoints_shrinks_output(planned) -> None:
    """縮點（預設）大幅少於 build_3d 的密集取樣，但長度統計不變。"""
    coord, res, _ = planned
    full = ad.from_mission_plan(coord.last_plan, coord.last_ctx, areas=_areas(), drones=_drones(),
                               id_map=coord.id_map, reduce=False)
    for did, bp in res["assembled_paths"].items():
        bp_full = full["assembled_paths"][did]
        assert len(bp.waypoints) < len(bp_full.waypoints)
        assert bp.total_length_m == pytest.approx(bp_full.total_length_m, rel=1e-9)
        assert len(bp.waypoints) <= 700, "須低於 ArduPilot 任務項目常見上限"
