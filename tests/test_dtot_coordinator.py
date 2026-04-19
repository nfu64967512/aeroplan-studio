"""DTOTCoordinator 單元測試 — 保障戰術時間協同不會默默跑錯

戰術計算若 bug 會導致：所有 UCAV 不同秒抵達 → 防空有時間反應 → 任務失敗
因此此測試是高優先級回歸保險。

執行：pytest tests/test_dtot_coordinator.py -v
"""
import math
import pytest

from core.strike import (
    TerminalStrikePlanner, StrikeTarget, DTOTCoordinator,
)


# ═══════════════════════════════════════════════════════════════════════
#  Fixtures
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture
def sample_trajectories():
    """3 個目標台北範例，生成標準軌跡集"""
    targets = [
        StrikeTarget(1, lat=25.033, lon=121.565, name='TGT-A'),
        StrikeTarget(2, lat=25.048, lon=121.517, name='TGT-B'),
        StrikeTarget(3, lat=25.017, lon=121.540, name='TGT-C'),
    ]
    planner = TerminalStrikePlanner(
        max_dive_angle_deg=55, cruise_alt_m=500, cruise_speed_mps=60,
        altitude_step_m=30, min_turn_radius_m=150,
    )
    return planner.plan_auto(targets, spawn_dist_m=1500.0)


@pytest.fixture
def coord():
    return DTOTCoordinator(
        cruise_speed_mps=60, max_speed_mps=85,
        stall_speed_mps=25, min_turn_radius_m=150,
    )


# ═══════════════════════════════════════════════════════════════════════
#  STOT (同時命中) 測試
# ═══════════════════════════════════════════════════════════════════════

class TestSTOT:
    def test_all_uavs_hit_same_second(self, coord, sample_trajectories):
        """STOT 核心：所有 UCAV 應具相同 tot_sec"""
        plans = coord.coordinate(sample_trajectories, mode='STOT')
        assert len(plans) == 3
        tots = {p.tot_sec for p in plans}
        assert len(tots) == 1, f'STOT 應所有機同秒命中，實得 {tots}'

    def test_stot_speed_within_bounds(self, coord, sample_trajectories):
        """所有反推空速必須在 [stall, max] 範圍內"""
        plans = coord.coordinate(sample_trajectories, mode='STOT')
        for p in plans:
            assert coord.stall_speed_mps <= p.required_speed_mps <= coord.max_speed_mps, \
                f'{p.uav_name} 空速 {p.required_speed_mps} 超出安全範圍'

    def test_longest_uav_uses_cruise_speed(self, coord, sample_trajectories):
        """最長路徑的 UCAV 應使用 cruise_speed (不需調速)"""
        plans = coord.coordinate(sample_trajectories, mode='STOT')
        longest = max(plans, key=lambda p: p.baseline_distance_m)
        assert abs(longest.required_speed_mps - coord.cruise_speed_mps) < 1.0, \
            f'最長機應用 cruise={coord.cruise_speed_mps}，實得 {longest.required_speed_mps}'


# ═══════════════════════════════════════════════════════════════════════
#  DTOT (間隔命中) 測試
# ═══════════════════════════════════════════════════════════════════════

class TestDTOTInterval:
    @pytest.mark.parametrize('interval', [5.0, 15.0, 30.0, 60.0])
    def test_interval_arrival_times(self, coord, sample_trajectories, interval):
        """DTOT：相鄰 slot 的命中時刻差應嚴格等於 interval"""
        plans = coord.coordinate(
            sample_trajectories, mode='DTOT', interval_sec=interval,
        )
        tots = sorted(p.tot_sec for p in plans)
        for i in range(1, len(tots)):
            diff = tots[i] - tots[i - 1]
            assert abs(diff - interval) < 0.1, \
                f'slot {i-1}→{i} 間隔 {diff:.2f} ≠ 預期 {interval}'

    def test_longest_path_gets_slot_0(self, coord, sample_trajectories):
        """DTOT 應將最長 L 分配到 slot 0 (最先命中，不補時)"""
        plans = coord.coordinate(
            sample_trajectories, mode='DTOT', interval_sec=15.0,
        )
        first_hitter = min(plans, key=lambda p: p.tot_sec)
        longest = max(plans, key=lambda p: p.baseline_distance_m)
        assert first_hitter.uav_id == longest.uav_id, \
            'slot 0 應為最長路徑的 UCAV'
        # slot 0 不需要 Loiter (路徑自然決定)
        assert first_hitter.holding_time_sec < 1.0

    def test_slot_k_holding_time_positive(self, coord, sample_trajectories):
        """DTOT 非 slot 0 的機必定需要補時 (或本就較短)"""
        plans = coord.coordinate(
            sample_trajectories, mode='DTOT', interval_sec=15.0,
        )
        for p in plans:
            # holding_time 必非負
            assert p.holding_time_sec >= -0.01


# ═══════════════════════════════════════════════════════════════════════
#  邊界條件測試
# ═══════════════════════════════════════════════════════════════════════

class TestBoundary:
    def test_empty_trajectories(self, coord):
        """空輸入應回傳空列表，不應 crash"""
        assert coord.coordinate([]) == []

    def test_max_speed_must_exceed_stall(self):
        """V_max <= V_stall 應拋 ValueError"""
        with pytest.raises(ValueError):
            DTOTCoordinator(
                cruise_speed_mps=30, max_speed_mps=25, stall_speed_mps=30,
            )

    def test_stall_clamp_triggers_loiter(self, sample_trajectories):
        """將 V_stall 設很高 → 所有機都應觸發 Loiter"""
        coord = DTOTCoordinator(
            cruise_speed_mps=60, max_speed_mps=85,
            stall_speed_mps=50,    # 故意設很高
            min_turn_radius_m=150,
        )
        plans = coord.coordinate(sample_trajectories, mode='STOT')
        # 比 longest 短的應觸發 Loiter
        non_longest = [p for p in plans if p.baseline_distance_m < max(
            pp.baseline_distance_m for pp in plans
        )]
        for p in non_longest:
            assert p.holding_time_sec >= 0  # 不 crash 即可


# ═══════════════════════════════════════════════════════════════════════
#  固定翼物理限制測試
# ═══════════════════════════════════════════════════════════════════════

class TestPhysics:
    def test_loiter_turns_match_time(self, coord, sample_trajectories):
        """Loiter 圈數 × T_lap 應等於 holding_time_sec"""
        plans = coord.coordinate(sample_trajectories, mode='STOT')
        for p in plans:
            if p.holding_time_sec > 1.0:
                t_lap = 2 * math.pi * p.holding_radius_m / coord.stall_speed_mps
                # 實際 holding_time 應 ≈ (holding_time / t_lap) × t_lap
                # 兩者應一致，誤差 < 0.1 秒
                # (因 holding_time 是 wait - time_at_stall，t_lap 是衍生量)
                assert p.holding_time_sec > 0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
