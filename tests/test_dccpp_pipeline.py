"""tests/test_dccpp_pipeline.py — DCCPP 覆蓋規劃管線特徵化測試 (characterization)

DCCPP (多無人機協同完全覆蓋路徑規劃) 管線原本 0 pytest 覆蓋，
重構風險極高。本測試以「固定種子輸入」鎖定**當前實作**的數值輸出，
作為後續重構的回歸保險 (regression safety net)。

涵蓋進入點：
    - mission/swarm_coordinator.py::SwarmCoordinator.plan_coverage_dccpp
      （DCCPP 主協調流程：FOV 分解 → 分配 → IDP → Dubins 組裝 → 高度規劃）
    - core/dccpp/task_allocator.py::UAVTaskAllocator        (Algorithm 1)
    - core/dccpp/idp_solver.py::IDP_Solver                  (Algorithm 2)
    - core/dccpp/altitude_planner.py::AltitudePlanner       (Algorithm 3)
    - core/global_planner/mdtsp_solver.py::DCCPPSolver      (整合舊版求解器)

特徵化性質說明：
    下方所有「黃金值 (golden value)」捕捉的是**目前的演算法輸出**，
    並非理論最佳解。若刻意更改演算法行為，必須同步更新這些常數；
    若非預期地變動，則代表重構引入了回歸。

固定輸入（種子）：
    1 塊台中近郊矩形區域 (~600m × 360m) + 3 架無人機，
    混合固定翼 (turn_radius>0, Dubins) 與多旋翼 (turn_radius=0, 直線)。

執行：QT_QPA_PLATFORM=offscreen python -m pytest tests/test_dccpp_pipeline.py -q
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any, Dict, List

import pytest

# 把專案根加入 sys.path（測試從 tests/ 目錄執行）
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from mission.swarm_coordinator import SwarmCoordinator
from core.dccpp.task_allocator import UAVTaskAllocator
from core.dccpp.idp_solver import IDP_Solver
from core.dccpp.altitude_planner import AltitudePlanner
from core.dccpp.area_processor import AreaProcessor
from core.geometry.coordinate import CoordinateTransformer
from core.global_planner.mdtsp_solver import (
    DCCPPSolver, UAVState, VehicleType,
    IDPSolver as LegacyIDPSolver,
    GreedyAllocator,
    AltitudePlanner as LegacyAltitudePlanner,
)


# ═══════════════════════════════════════════════════════════════════════
#  固定種子輸入（所有測試共用同一場景，確保可重現）
# ═══════════════════════════════════════════════════════════════════════

# 台中近郊單一矩形區域（逆時針，~600m 東西 × ~360m 南北）
AREA: Dict[str, Any] = {
    'area_id': 1,
    'priority': 1.0,
    'polygon': [
        (24.1500, 120.6500),
        (24.1500, 120.6560),
        (24.1536, 120.6560),
        (24.1536, 120.6500),
    ],
}

# 3 架無人機：1 固定翼 (Dubins) + 2 多旋翼 (直線)
DRONES: List[Dict[str, Any]] = [
    {'drone_id': 1, 'name': 'FW-1', 'position': (24.1480, 120.6490),
     'heading': 45.0, 'turn_radius': 80.0, 'speed': 22.0,
     'vehicle_type': 'fixed_wing'},
    {'drone_id': 2, 'name': 'MR-1', 'position': (24.1490, 120.6570),
     'heading': 0.0, 'turn_radius': 0.0, 'speed': 12.0,
     'vehicle_type': 'multirotor'},
    {'drone_id': 3, 'name': 'MR-2', 'position': (24.1550, 120.6505),
     'heading': 90.0, 'turn_radius': 0.0, 'speed': 12.0,
     'vehicle_type': 'multirotor'},
]

COVERAGE_WIDTH_M = 120.0
OVERLAP_RATE = 0.1
COVERAGE_ALT_M = 100.0

# 區域 + 起點 lat/lon 的合理外接框（航點有效性驗證用）
_LAT_LO, _LAT_HI = 24.13, 24.17
_LON_LO, _LON_HI = 120.63, 120.67


def _run_pipeline() -> Dict[str, Any]:
    """以固定種子輸入執行完整 DCCPP 管線。"""
    coord = SwarmCoordinator()
    return coord.plan_coverage_dccpp(
        areas=[dict(AREA)],
        drones=[dict(d) for d in DRONES],
        coverage_width_m=COVERAGE_WIDTH_M,
        overlap_rate=OVERLAP_RATE,
        auto_scan_angle=True,
        coverage_altitude=COVERAGE_ALT_M,
        enable_altitude=True,
        coordination_mode='uncoordinated',
    )


def _build_uav_states() -> List[UAVState]:
    """由 DRONES 構造 UAVState 列表（與管線內部一致）。"""
    states: List[UAVState] = []
    for d in DRONES:
        vt = (VehicleType.FIXED_WING if d['vehicle_type'] == 'fixed_wing'
              else VehicleType.MULTIROTOR)
        states.append(UAVState(
            uav_id=d['drone_id'], position=d['position'], heading=d['heading'],
            turn_radius=d['turn_radius'], speed=d['speed'], vehicle_type=vt,
            fov_width=COVERAGE_WIDTH_M,
        ))
    return states


# ═══════════════════════════════════════════════════════════════════════
#  Fixtures
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture(scope='module')
def pipeline_result() -> Dict[str, Any]:
    """整條管線只跑一次（含 Dubins 計算，較耗時），多測試共用。

    下游測試皆**唯讀**此結果，不做變更。
    """
    return _run_pipeline()


@pytest.fixture(scope='module')
def uav_states() -> List[UAVState]:
    return _build_uav_states()


@pytest.fixture(scope='module')
def coverage_paths(pipeline_result):
    """管線產出的 CoveragePath 列表（供分配器/求解器測試重用，唯讀）。"""
    return pipeline_result['coverage_paths']


# 數值容差 helper：相對 0.1% 或絕對 0.5m，兩者取寬鬆者
def _approx_len(value: float):
    return pytest.approx(value, rel=1e-3, abs=0.5)


# ═══════════════════════════════════════════════════════════════════════
#  1. 完整管線 plan_coverage_dccpp — 結構與分配
# ═══════════════════════════════════════════════════════════════════════

class TestPipelineStructure:
    def test_runs_without_error_and_has_keys(self, pipeline_result):
        """管線應正常完成並回傳所有約定鍵。"""
        assert set(pipeline_result.keys()) == {
            'results', 'allocation', 'coverage_paths',
            'uav_states', 'total_makespan', 'assembled_paths',
        }

    def test_v2_passthrough_taken(self, pipeline_result):
        """assembled_paths 非 None 證明走的是 DCCPP v2 直通管線（非退化）。"""
        assert pipeline_result['assembled_paths'] is not None

    def test_uav_states_count(self, pipeline_result):
        """輸入 3 架 → 應有 3 個 UAVState。"""
        assert len(pipeline_result['uav_states']) == 3

    def test_coverage_paths_shape(self, pipeline_result):
        """單一區域 → 1 條 CoveragePath；掃描角與作業段數鎖定為特徵值。"""
        cps = pipeline_result['coverage_paths']
        assert len(cps) == 1
        cp = cps[0]
        assert cp.area_id == 1
        # auto_scan_angle 對此矩形求得 90°（東西向掃描）
        assert cp.scan_angle_deg == pytest.approx(90.0, abs=1e-6)
        # 黃金值：RegionDivider.decompose_by_fov 對此區域產生 4 條作業段
        assert len(cp.operations) == 4

    def test_allocation_assigns_all_three_uavs(self, pipeline_result):
        """3 架 UAV 應全部分配到唯一區域（無漏網）。"""
        allocation = pipeline_result['allocation']
        assert set(allocation.keys()) == {1}
        assigned = allocation[1]
        assert sorted(assigned) == [1, 2, 3]

    def test_allocation_order_golden(self, pipeline_result):
        """鎖定當前分配順序（特徵化）：[2, 3, 1]。"""
        assert pipeline_result['allocation'][1] == [2, 3, 1]


# ═══════════════════════════════════════════════════════════════════════
#  2. 完整管線 — 組裝航點 (assembled_paths)
# ═══════════════════════════════════════════════════════════════════════

class TestAssembledPaths:
    def test_one_path_per_uav(self, pipeline_result):
        ap = pipeline_result['assembled_paths']
        assert set(ap.keys()) == {1, 2, 3}

    def test_waypoints_non_empty(self, pipeline_result):
        """每架 UAV 的航點序列皆非空。"""
        ap = pipeline_result['assembled_paths']
        for uid, bp in ap.items():
            assert len(bp.waypoints) > 0, f'UAV {uid} 航點為空'

    def test_waypoints_have_valid_latlon(self, pipeline_result):
        """所有航點 lat/lon 必為有限值且落在區域合理外接框內。"""
        ap = pipeline_result['assembled_paths']
        for uid, bp in ap.items():
            for w in bp.waypoints:
                assert math.isfinite(w.lat) and math.isfinite(w.lon)
                assert _LAT_LO <= w.lat <= _LAT_HI, f'UAV {uid} lat={w.lat} 越界'
                assert _LON_LO <= w.lon <= _LON_HI, f'UAV {uid} lon={w.lon} 越界'

    def test_waypoint_counts_golden(self, pipeline_result):
        """鎖定各 UAV 航點數（特徵化）。

        固定翼 (uav 1) 因 Dubins 進場+階梯起降而航點較多；
        多旋翼 (uav 2/3) 為簡化直線。
        """
        ap = pipeline_result['assembled_paths']
        assert len(ap[1].waypoints) == 14
        assert len(ap[2].waypoints) == 3
        assert len(ap[3].waypoints) == 3

    def test_path_lengths_golden(self, pipeline_result):
        """鎖定各 UAV 總路徑長 / 進場長 / 作業長（特徵化黃金值）。"""
        ap = pipeline_result['assembled_paths']
        # uav 1（固定翼）
        assert ap[1].total_length_m == _approx_len(1215.228461)
        assert ap[1].entry_length_m == _approx_len(605.436075)
        # uav 2（多旋翼）
        assert ap[2].total_length_m == _approx_len(830.134116)
        assert ap[2].entry_length_m == _approx_len(220.341730)
        # uav 3（多旋翼）
        assert ap[3].total_length_m == _approx_len(1248.694728)
        assert ap[3].entry_length_m == _approx_len(638.902342)

    def test_operation_length_equal_across_uavs(self, pipeline_result):
        """三條平行條帶等寬 → 各 UAV 作業段長相同（≈609.79m）。"""
        ap = pipeline_result['assembled_paths']
        for uid in (1, 2, 3):
            assert ap[uid].operation_length_m == _approx_len(609.792386)

    def test_transfer_length_zero(self, pipeline_result):
        """每架 UAV 僅一條掃描線 → 無轉移段 (transfer=0)。"""
        ap = pipeline_result['assembled_paths']
        for uid in (1, 2, 3):
            assert ap[uid].transfer_length_m == pytest.approx(0.0, abs=1e-6)

    def test_total_length_decomposition(self, pipeline_result):
        """總長 = 進場 + 作業 + 轉移（內部一致性）。"""
        ap = pipeline_result['assembled_paths']
        for uid, bp in ap.items():
            assert bp.total_length_m == pytest.approx(
                bp.entry_length_m + bp.operation_length_m + bp.transfer_length_m,
                rel=1e-9, abs=1e-6,
            )


# ═══════════════════════════════════════════════════════════════════════
#  3. 完整管線 — makespan 與合理界限
# ═══════════════════════════════════════════════════════════════════════

class TestMakespanBounds:
    def test_total_makespan_golden(self, pipeline_result):
        """頂層 total_makespan（= max 路徑長/速度）鎖定為特徵值。"""
        assert pipeline_result['total_makespan'] == pytest.approx(
            104.057894, rel=1e-3,
        )

    def test_total_makespan_within_sane_bounds(self, pipeline_result):
        """makespan 應為正且遠小於 1 小時（防呆界限）。"""
        mk = pipeline_result['total_makespan']
        assert 0.0 < mk < 3600.0

    def test_total_makespan_matches_slowest_uav(self, pipeline_result):
        """total_makespan 應等於最慢 UAV 的 (總路徑長 / 速度)。"""
        ap = pipeline_result['assembled_paths']
        uav_by_id = {u.uav_id: u for u in pipeline_result['uav_states']}
        per_uav_time = [
            bp.total_length_m / max(uav_by_id[uid].speed, 1e-3)
            for uid, bp in ap.items()
        ]
        assert pipeline_result['total_makespan'] == pytest.approx(
            max(per_uav_time), rel=1e-6,
        )

    def test_path_lengths_within_sane_bounds(self, pipeline_result):
        """各路徑長度應為正且在合理範圍（區域對角 < 1km，含進場 < 5km）。"""
        ap = pipeline_result['assembled_paths']
        for uid, bp in ap.items():
            assert 0.0 < bp.total_length_m < 5000.0, f'UAV {uid} 路徑長異常'

    def test_legacy_result_makespan_golden(self, pipeline_result):
        """results[area].makespan（IDP 內部最長路徑長 [m]）鎖定為特徵值。"""
        results = pipeline_result['results']
        assert set(results.keys()) == {1}
        res = results[1]
        # 該欄位語意為「最長 UAV 路徑長度（公尺）」，非時間
        assert res.makespan == _approx_len(1248.694728)
        # 每架 UAV 各分配到 1 條作業段
        assert {k: len(v) for k, v in res.uav_assignments.items()} == {
            2: 1, 3: 1, 1: 1,
        }


# ═══════════════════════════════════════════════════════════════════════
#  4. 完整管線 — 決定性（跨多次執行一致）
# ═══════════════════════════════════════════════════════════════════════

class TestDeterminism:
    def test_allocation_stable_across_runs(self):
        """連跑 3 次，分配結果必須完全一致。"""
        allocs = [_run_pipeline()['allocation'] for _ in range(3)]
        for a in allocs[1:]:
            assert a == allocs[0]

    def test_makespan_and_lengths_stable_across_runs(self):
        """連跑 3 次，makespan 與各 UAV 路徑長必須位元級一致。"""
        sigs = []
        for _ in range(3):
            r = _run_pipeline()
            ap = r['assembled_paths']
            sigs.append((
                round(r['total_makespan'], 9),
                tuple(sorted(
                    (uid, len(bp.waypoints), round(bp.total_length_m, 9))
                    for uid, bp in ap.items()
                )),
            ))
        for s in sigs[1:]:
            assert s == sigs[0]


# ═══════════════════════════════════════════════════════════════════════
#  5. UAVTaskAllocator (Algorithm 1) — 單元特徵化
# ═══════════════════════════════════════════════════════════════════════

class TestUAVTaskAllocator:
    def test_single_area_jk_is_one(self, uav_states, coverage_paths):
        """只有一個區域時，正規化後 J_k 必為 1.0。"""
        alloc = UAVTaskAllocator()
        jk = alloc.compute_jk_values(uav_states, coverage_paths)
        assert len(jk) == 1
        assert float(jk[0]) == pytest.approx(1.0, abs=1e-9)

    def test_lk_assigns_all_uavs_to_single_area(self, uav_states, coverage_paths):
        """單區域 → l_k 應等於 UAV 總數（3）。"""
        alloc = UAVTaskAllocator()
        jk = alloc.compute_jk_values(uav_states, coverage_paths)
        lk = alloc.compute_lk_values(jk, len(uav_states))
        assert [int(x) for x in lk] == [3]

    def test_allocate_result_fields(self, uav_states, coverage_paths):
        """allocate() 回傳之 AllocationResult 各欄位特徵化。"""
        alloc = UAVTaskAllocator()
        res = alloc.allocate(uav_states, coverage_paths)
        assert res.area_assignments == {1: [2, 3, 1]}
        assert res.area_lk_values == {1: 3}
        assert res.area_order == [1]
        assert res.unassigned_uav_ids == []
        assert res.area_jk_values[1] == pytest.approx(1.0, abs=1e-9)

    def test_allocate_is_deterministic(self, uav_states, coverage_paths):
        """同輸入多次分配結果一致。"""
        alloc = UAVTaskAllocator()
        a = alloc.allocate(uav_states, coverage_paths).area_assignments
        b = alloc.allocate(uav_states, coverage_paths).area_assignments
        assert a == b == {1: [2, 3, 1]}

    def test_empty_inputs_return_empty(self):
        """空 UAV 或空區域 → 空結果，不應 crash。"""
        alloc = UAVTaskAllocator()
        assert alloc.allocate([], []).area_assignments == {}


# ═══════════════════════════════════════════════════════════════════════
#  6. IDP_Solver (Algorithm 2) — 單元特徵化
# ═══════════════════════════════════════════════════════════════════════

class _IDPFixture:
    """以固定 uav_inputs 順序 [1,2,3] 跑 IDP，鎖定其黃金輸出。"""


@pytest.fixture(scope='module')
def idp_inputs():
    """區域前處理 + 固定順序 uav_inputs（與管線分開，順序刻意固定）。"""
    proc = AreaProcessor()
    ar = proc.process_latlon(
        AREA['polygon'], COVERAGE_WIDTH_M, COVERAGE_WIDTH_M * OVERLAP_RATE,
    )
    tf = CoordinateTransformer(ar.centroid_latlon[0], ar.centroid_latlon[1])
    uav_inputs = [
        {'uav_id': d['drone_id'], 'lat': d['position'][0],
         'lon': d['position'][1], 'heading': d['heading']}
        for d in DRONES
    ]
    return ar, tf, uav_inputs


@pytest.fixture(scope='module')
def idp_result(idp_inputs):
    ar, tf, uav_inputs = idp_inputs
    solver = IDP_Solver(min_turn_radius=80.0, altitude=COVERAGE_ALT_M)
    return solver.solve(uav_inputs, ar, tf)


class TestIDPSolver:
    def test_area_processed_to_three_scan_lines(self, idp_inputs):
        """AreaProcessor 對此矩形產生 3 條掃描線；最小寬度/掃描角鎖定。"""
        ar, _tf, _ = idp_inputs
        assert len(ar.scan_lines) == 3
        assert ar.min_width_result.min_width == _approx_len(398.7375)
        assert ar.min_width_result.alpha_k_deg == pytest.approx(90.0, abs=1e-3)

    def test_one_plan_per_uav(self, idp_result):
        assert set(idp_result.per_uav.keys()) == {1, 2, 3}

    def test_each_uav_gets_one_scan_line(self, idp_result):
        """3 條掃描線 strip-split 給 3 架 → 各 1 條。"""
        for uid in (1, 2, 3):
            plan = idp_result.per_uav[uid]
            assert len(plan.ordered_scan_lines) == 1
            assert len(plan.directions) == 1
            assert plan.directions[0] in (-1, 1)

    def test_makespan_length_golden(self, idp_result):
        """makespan_length（最長 UAV 路徑長 [m]）鎖定為特徵值。"""
        assert idp_result.makespan_length == _approx_len(1321.500825)

    def test_per_uav_lengths_golden(self, idp_result):
        """各 UAV 子計畫的 total/entry/op 路徑長鎖定為特徵值。"""
        p1 = idp_result.per_uav[1]
        p2 = idp_result.per_uav[2]
        p3 = idp_result.per_uav[3]
        assert p1.total_length_m == _approx_len(979.567517)
        assert p1.entry_length_m == _approx_len(369.775131)
        assert p2.total_length_m == _approx_len(967.007879)
        assert p2.entry_length_m == _approx_len(357.215492)
        assert p3.total_length_m == _approx_len(1321.500825)
        assert p3.entry_length_m == _approx_len(711.708439)
        # 三條等寬條帶 → 作業段長相同
        for p in (p1, p2, p3):
            assert p.operation_length_m == _approx_len(609.792386)

    def test_directions_golden(self, idp_result):
        """鎖定各 UAV 的掃描方向（特徵化）。"""
        assert idp_result.per_uav[1].directions == [1]
        assert idp_result.per_uav[2].directions == [-1]
        assert idp_result.per_uav[3].directions == [1]

    def test_deterministic(self, idp_inputs):
        """同輸入連跑 3 次，makespan_length 位元級一致。"""
        ar, tf, uav_inputs = idp_inputs
        vals = {
            round(
                IDP_Solver(min_turn_radius=80.0, altitude=COVERAGE_ALT_M)
                .solve(uav_inputs, ar, tf).makespan_length, 9,
            )
            for _ in range(3)
        }
        assert len(vals) == 1

    def test_empty_scan_lines_no_crash(self, idp_inputs):
        """掃描線為空時應回傳每架 UAV 的空計畫，不 crash。"""
        ar, tf, uav_inputs = idp_inputs
        # 構造一個 scan_lines 為空的淺複本
        import copy
        empty_ar = copy.copy(ar)
        empty_ar.scan_lines = []
        res = IDP_Solver(min_turn_radius=80.0).solve(uav_inputs, empty_ar, tf)
        assert set(res.per_uav.keys()) == {1, 2, 3}
        for plan in res.per_uav.values():
            assert plan.ordered_scan_lines == []
        assert res.makespan_length == 0.0


# ═══════════════════════════════════════════════════════════════════════
#  7. AltitudePlanner (Algorithm 3) — 單元特徵化
# ═══════════════════════════════════════════════════════════════════════

# 固定 2D 路徑（公尺座標）：開放折線，3 段、2 個轉角
_ALT_PATH_2D = [(0.0, 0.0), (500.0, 0.0), (500.0, 120.0), (0.0, 120.0)]


class TestAltitudePlanner:
    def test_flat_terrain_structure_golden(self):
        """平地：密集插值點數 / 精簡後航點數 鎖定為特徵值。"""
        planner = AltitudePlanner(h_cov=COVERAGE_ALT_M)
        res = planner.plan_altitude(_ALT_PATH_2D, terrain_func=None)
        assert res.n_dense == 24
        # 共線過濾後僅保留 4 個轉角（折線本身的點）
        assert res.n_reduced == 4
        assert len(res.waypoints_3d) == 4

    def test_flat_terrain_altitudes(self):
        """平地：所有航點高度應緊貼 h_cov（曲率修正使遠端略低）。"""
        planner = AltitudePlanner(h_cov=COVERAGE_ALT_M)
        res = planner.plan_altitude(_ALT_PATH_2D, terrain_func=None)
        # 起點 z 精確等於 h_cov
        assert res.waypoints_3d[0][2] == pytest.approx(100.0, abs=1e-6)
        zs = [w[2] for w in res.waypoints_3d]
        assert min(zs) == pytest.approx(99.901554, abs=1e-3)
        assert max(zs) == pytest.approx(100.207744, abs=1e-3)
        # 全程貼地飛行高度應在 h_cov 附近 ±5m
        for z in zs:
            assert 95.0 <= z <= 105.0

    def test_ramp_terrain_follows_ground(self):
        """斜坡地形：高度應隨地面抬升（最大高度顯著高於 h_cov）。"""
        planner = AltitudePlanner(h_cov=COVERAGE_ALT_M)

        def terrain(x, y):
            return 0.05 * x  # 每往東 1m 抬升 0.05m

        res = planner.plan_altitude(_ALT_PATH_2D, terrain_func=terrain)
        zs = [w[2] for w in res.waypoints_3d]
        # 黃金值：斜坡使最高點 ~117.5m，最低點 ~99.9m
        assert max(zs) == pytest.approx(117.512804, abs=1e-2)
        assert min(zs) == pytest.approx(99.901554, abs=1e-2)
        # 共線過濾後航點數鎖定
        assert res.n_reduced == 7

    def test_waypoints_3d_all_finite(self):
        """所有 3D 航點座標皆為有限值。"""
        planner = AltitudePlanner(h_cov=COVERAGE_ALT_M)
        res = planner.plan_altitude(_ALT_PATH_2D, terrain_func=None)
        for x, y, z in res.waypoints_3d:
            assert math.isfinite(x) and math.isfinite(y) and math.isfinite(z)

    def test_single_point_degenerate(self):
        """單點輸入 → 退化為單一 3D 航點（z = h_cov）。"""
        planner = AltitudePlanner(h_cov=COVERAGE_ALT_M)
        res = planner.plan_altitude([(10.0, 20.0)], terrain_func=None)
        assert res.n_reduced == 1
        assert res.waypoints_3d[0][2] == pytest.approx(100.0, abs=1e-6)

    def test_deterministic(self):
        """同輸入連跑 3 次，輸出高度序列位元級一致。"""
        planner = AltitudePlanner(h_cov=COVERAGE_ALT_M)
        sigs = {
            tuple(round(w[2], 9) for w in
                  planner.plan_altitude(_ALT_PATH_2D, terrain_func=None).waypoints_3d)
            for _ in range(3)
        }
        assert len(sigs) == 1

    def test_invalid_h_cov_raises(self):
        """h_cov <= 0 應於建構時拋 ValueError。"""
        with pytest.raises(ValueError):
            AltitudePlanner(h_cov=0.0)


# ═══════════════════════════════════════════════════════════════════════
#  8. DCCPPSolver (整合舊版求解器) — 特徵化
# ═══════════════════════════════════════════════════════════════════════

class TestDCCPPSolver:
    """DCCPPSolver.solve 走的是舊版 IDPSolver（新 IDP_Solver 的 solve 簽章不相容，
    從未接入此整合器）。此處以管線退化分支實際使用的元件組合做特徵化。
    """

    def _make_solver(self, allocator):
        return DCCPPSolver(
            allocator=allocator,
            idp_solver=LegacyIDPSolver(),
            altitude_planner=LegacyAltitudePlanner(coverage_altitude=COVERAGE_ALT_M),
        )

    def test_solve_with_new_allocator_golden(self, uav_states, coverage_paths):
        """UAVTaskAllocator + 舊 IDPSolver：makespan 與各機路徑長鎖定。"""
        solver = self._make_solver(UAVTaskAllocator())
        results = solver.solve(
            uav_states, coverage_paths,
            enable_altitude=True, enable_dubins_assembly=False,
        )
        assert set(results.keys()) == {1}
        res = results[1]
        # makespan = max(總路徑長 / 速度) = 1586.474305 / 12
        assert res.makespan == pytest.approx(132.206192, rel=1e-3)
        # 4 條作業段分給 3 架（其中一架拿 2 條）
        total_ops = sum(len(v) for v in res.uav_assignments.values())
        assert total_ops == 4
        # 各機總路徑長（multiset）鎖定為特徵值
        dists = sorted(round(v, 3) for v in res.total_distances.values())
        assert dists == pytest.approx([824.080, 935.780, 1586.474], rel=1e-3)

    def test_solve_with_greedy_allocator_matches(self, uav_states, coverage_paths):
        """改用 GreedyAllocator，makespan 與路徑長集合應相同（特徵化）。"""
        solver = self._make_solver(GreedyAllocator())
        results = solver.solve(
            uav_states, coverage_paths,
            enable_altitude=True, enable_dubins_assembly=False,
        )
        res = results[1]
        assert res.makespan == pytest.approx(132.206192, rel=1e-3)
        dists = sorted(round(v, 3) for v in res.total_distances.values())
        assert dists == pytest.approx([824.080, 935.780, 1586.474], rel=1e-3)

    def test_solve_makespan_within_sane_bounds(self, uav_states, coverage_paths):
        """makespan 為正且 < 1 小時。"""
        solver = self._make_solver(UAVTaskAllocator())
        results = solver.solve(
            uav_states, coverage_paths,
            enable_altitude=False, enable_dubins_assembly=False,
        )
        assert 0.0 < results[1].makespan < 3600.0

    def test_solve_assignments_cover_all_operations(self, uav_states, coverage_paths):
        """所有作業段都被指派（無漏掃）；各 UAV 的 OperationSegment 有效。"""
        solver = self._make_solver(UAVTaskAllocator())
        results = solver.solve(
            uav_states, coverage_paths,
            enable_altitude=False, enable_dubins_assembly=False,
        )
        res = results[1]
        n_total = len(coverage_paths[0].operations)
        assigned = sum(len(v) for v in res.uav_assignments.values())
        assert assigned == n_total == 4
        # 每個被指派的作業段 lat/lon 端點皆有限且在外接框內
        for ops in res.uav_assignments.values():
            for op in ops:
                for (lat, lon) in (op.left_point, op.right_point):
                    assert math.isfinite(lat) and math.isfinite(lon)
                    assert _LAT_LO <= lat <= _LAT_HI
                    assert _LON_LO <= lon <= _LON_HI

    def test_solve_deterministic(self, uav_states, coverage_paths):
        """同輸入連跑 3 次，makespan 位元級一致。"""
        solver = self._make_solver(UAVTaskAllocator())
        vals = {
            round(solver.solve(
                uav_states, coverage_paths,
                enable_altitude=False, enable_dubins_assembly=False,
            )[1].makespan, 9)
            for _ in range(3)
        }
        assert len(vals) == 1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
