"""core.strike.time_coordination 單元測試

此模組為 5 個 planner 共用的時間協同核心，
一旦 bug 會同時影響 DTOT / VTOL / Recon / Advanced 所有 planner。
"""
import math
import pytest

from core.strike.time_coordination import (
    TimingMode, DtotOrder, FeasibilityStatus,
    TimingSlot, LoiterPlanOutput,
    compute_tot_schedule, fill_loiter_turns,
    compute_loiter_plan, coordinate,
)


# ═══════════════════════════════════════════════════════════════════════
#  STOT 同秒命中
# ═══════════════════════════════════════════════════════════════════════

class TestSTOT:
    def test_all_slots_same_arrival(self):
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[100, 150, 200],
            mode='STOT',
        )
        assert base_tot == 200.0
        assert all(s.arrival_time_sec == 200.0 for s in slots)

    def test_wait_equals_base_minus_nominal(self):
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[100, 150, 200], mode='STOT',
        )
        # slot with nominal=200 → wait=0 (最長者)
        # slot with nominal=150 → wait=50
        # slot with nominal=100 → wait=100
        waits = {round(s.nominal_time_sec): s.wait_time_sec for s in slots}
        assert waits[200] == 0.0
        assert waits[150] == 50.0
        assert waits[100] == 100.0

    def test_empty_input(self):
        base_tot, slots = compute_tot_schedule([])
        assert slots == []

    def test_payloads_preserved(self):
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[100, 200],
            payloads=['uav-a', 'uav-b'],
        )
        assert slots[0].payload == 'uav-a'
        assert slots[1].payload == 'uav-b'


# ═══════════════════════════════════════════════════════════════════════
#  DTOT 間隔命中
# ═══════════════════════════════════════════════════════════════════════

class TestDTOT:
    @pytest.mark.parametrize('interval', [5.0, 10.0, 30.0])
    def test_slot_intervals_exact(self, interval):
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[100, 150, 200],
            mode='DTOT', interval_sec=interval,
        )
        # 排序後的 arrival time 應該是 base_tot, base_tot+Δ, base_tot+2Δ
        arrivals = sorted(s.arrival_time_sec for s in slots)
        for k in range(len(arrivals)):
            expected = base_tot + k * interval
            assert abs(arrivals[k] - expected) < 1e-6

    def test_longest_first_by_default(self):
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[100, 150, 200],
            mode='DTOT', interval_sec=15.0,
        )
        # nominal=200 應為 slot 0 (最先命中，t=base_tot)
        nominal_200 = next(s for s in slots if round(s.nominal_time_sec) == 200)
        assert nominal_200.slot_index == 0
        assert nominal_200.arrival_time_sec == base_tot
        # nominal=100 (最短) 應為 slot 2 (最後命中，補時最多)
        nominal_100 = next(s for s in slots if round(s.nominal_time_sec) == 100)
        assert nominal_100.slot_index == 2

    def test_shortest_first_order(self):
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[100, 150, 200],
            mode='DTOT', interval_sec=10.0,
            dtot_order='shortest_first',
        )
        # 最短 L 入 slot 0
        nominal_100 = next(s for s in slots if round(s.nominal_time_sec) == 100)
        assert nominal_100.slot_index == 0

    def test_index_order_preserves_input(self):
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[100, 200, 150],
            mode='DTOT', interval_sec=10.0,
            dtot_order='index_order',
        )
        # slot_index 對應輸入索引
        for i, s in enumerate(slots):
            assert s.slot_index == i


# ═══════════════════════════════════════════════════════════════════════
#  Loiter 圈數計算
# ═══════════════════════════════════════════════════════════════════════

class TestLoiterTurns:
    def test_loiter_formula(self):
        slots = [TimingSlot(0, 100, 200, 100.0, 0.0, None)]
        V = 25.0
        R = 150.0
        fill_loiter_turns(slots, V, R)
        # T_lap = 2πR/V = 2π·150/25 = 37.7s
        # turns = 100 / 37.7 = 2.65
        expected = 100.0 / (2 * math.pi * R / V)
        assert abs(slots[0].loiter_turns - expected) < 1e-3

    def test_wait_below_threshold_gives_zero(self):
        slots = [TimingSlot(0, 100, 100.2, 0.2, 0.0, None)]
        fill_loiter_turns(slots, 25.0, 150.0, threshold_sec=0.5)
        assert slots[0].loiter_turns == 0.0


# ═══════════════════════════════════════════════════════════════════════
#  coordinate() 一次完成
# ═══════════════════════════════════════════════════════════════════════

class TestCoordinate:
    def test_stot_full_pipeline(self):
        base_tot, slots = coordinate(
            nominal_times_sec=[100, 150, 200],
            mode='STOT',
            cruise_speed_mps=25.0,
            loiter_radius_m=150.0,
        )
        # 所有機同時命中
        assert all(abs(s.arrival_time_sec - base_tot) < 1e-6 for s in slots)
        # 最長者 loiter_turns=0
        longest = max(slots, key=lambda s: s.nominal_time_sec)
        assert longest.loiter_turns == 0.0
        # 最短者 loiter_turns > 0
        shortest = min(slots, key=lambda s: s.nominal_time_sec)
        assert shortest.loiter_turns > 0


# ═══════════════════════════════════════════════════════════════════════
#  compute_loiter_plan (反推空速版本)
# ═══════════════════════════════════════════════════════════════════════

class TestComputeLoiterPlan:
    def test_cruise_speed_ok(self):
        r = compute_loiter_plan(
            path_length_m=2500, arrival_time_sec=100,
            cruise_speed_mps=25, stall_speed_mps=18, max_speed_mps=40,
            loiter_radius_m=150,
        )
        # V_req = 2500/100 = 25.0 == cruise → OK
        assert r.feasibility == FeasibilityStatus.OK
        assert r.loiter_turns == 0.0

    def test_v_req_above_max_infeasible(self):
        r = compute_loiter_plan(
            path_length_m=10000, arrival_time_sec=100,
            cruise_speed_mps=25, stall_speed_mps=18, max_speed_mps=40,
            loiter_radius_m=150,
        )
        # V_req = 100 > max 40 → INFEASIBLE
        assert r.feasibility == FeasibilityStatus.INFEASIBLE

    def test_v_req_below_stall_triggers_loiter(self):
        r = compute_loiter_plan(
            path_length_m=500, arrival_time_sec=100,
            cruise_speed_mps=25, stall_speed_mps=18, max_speed_mps=40,
            loiter_radius_m=150,
        )
        # V_req = 5 < stall 18 → 夾 stall + Loiter
        assert r.feasibility == FeasibilityStatus.LOITER_INSERTED
        assert r.required_speed_mps == 18
        assert r.loiter_turns > 0

    def test_dtot_interval_mode_uses_cruise(self):
        """DTOT 間隔模式：V_stall ≤ V_req < V_cruise*0.95 時用 V_cruise + Loiter"""
        # V_req = 2000/100 = 20 介於 V_stall=18 與 V_cruise*0.95=23.75 之間
        r = compute_loiter_plan(
            path_length_m=2000, arrival_time_sec=100,
            cruise_speed_mps=25, stall_speed_mps=18, max_speed_mps=40,
            loiter_radius_m=150, dtot_interval_mode=True,
        )
        assert r.feasibility == FeasibilityStatus.LOITER_INSERTED
        assert r.required_speed_mps == 25  # 保留 V_cruise (不夾至 V_stall)
        assert r.loiter_turns > 0

    def test_dtot_interval_mode_below_stall_still_clamps(self):
        """DTOT 間隔模式：若 V_req 仍 < V_stall，照樣夾到 V_stall"""
        r = compute_loiter_plan(
            path_length_m=500, arrival_time_sec=100,
            cruise_speed_mps=25, stall_speed_mps=18, max_speed_mps=40,
            loiter_radius_m=150, dtot_interval_mode=True,
        )
        # V_req = 5 < V_stall → 夾 V_stall (DTOT 例外不適用，安全第一)
        assert r.required_speed_mps == 18

    def test_preclimb_fail(self):
        r = compute_loiter_plan(
            path_length_m=1000, arrival_time_sec=10,
            climb_time_sec=15,
            cruise_speed_mps=25, stall_speed_mps=18, max_speed_mps=40,
            loiter_radius_m=150,
        )
        # climb 15 > arrival 10 → PRECLIMB_FAIL
        assert r.feasibility == FeasibilityStatus.PRECLIMB_FAIL

    def test_max_less_than_stall_raises(self):
        with pytest.raises(ValueError):
            compute_loiter_plan(
                path_length_m=1000, arrival_time_sec=50,
                cruise_speed_mps=20, stall_speed_mps=30, max_speed_mps=25,
                loiter_radius_m=150,
            )
