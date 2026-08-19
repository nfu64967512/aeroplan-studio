"""sar_swarm.phase_gate 單元測試 —— 爬升/巡航閘門的遲滯與單調展開。

背景:P91 航次的 UAV5 在 200 s 內來回穿越 3000 m 閘門十餘次,每次只低於
閘門 1–23 m,卻每次都把追趕速度(157→96 EAS)與側向展開角(滿角→0°)
一起打掉,形成階段 2 的極限環。詳見 phase_gate.py 的模組說明。

這裡的重點測試是 `test_p91_uav5_altitude_trace_stops_chattering` ——
用該航次真實觀測到的下沉深度重放,證明遲滯確實吃掉抖動。
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / "scripts" / "wsl" / "sar_swarm_patches"))

from phase_gate import LatchedAltGate, MonotonicRamp  # noqa: E402


def _run(gate, samples):
    """samples = [(t, alt)] → 回傳每個 tick 的 climbing 序列。"""
    return [gate.update(a, t)[0] for t, a in samples]


# ═══════════════════════════════════════════════════════════════════════
#  LatchedAltGate —— 基本語意
# ═══════════════════════════════════════════════════════════════════════

class TestGateBasics:
    def test_starts_climbing(self):
        assert LatchedAltGate(3000.0).climbing is True

    def test_enters_cruise_at_threshold_without_delay(self):
        """進入巡航段不帶遲滯:第一次到達門檻就算數。"""
        g = LatchedAltGate(3000.0)
        assert g.update(2999.0, 0.0) == (True, False)
        assert g.update(3000.0, 1.0) == (False, True)
        assert g.flips == 1

    def test_exit_line_is_threshold_minus_hysteresis(self):
        g = LatchedAltGate(3000.0, hysteresis_m=150.0)
        assert g.exit_alt_m == 2850.0

    def test_shallow_sag_never_flips(self):
        """比閘門低 23 m(P91 最深的一次淺下沉)不該有任何作用。"""
        g = LatchedAltGate(3000.0, hysteresis_m=150.0, dwell_s=5.0)
        g.update(3100.0, 0.0)                       # 進巡航
        for i in range(1, 200):
            climbing, _ = g.update(2977.0, float(i))
            assert climbing is False
        assert g.flips == 1                          # 只有最初那次進巡航

    def test_deep_sag_shorter_than_dwell_does_not_flip(self):
        """夠深但撐不滿 dwell 秒 —— 不翻。"""
        g = LatchedAltGate(3000.0, hysteresis_m=150.0, dwell_s=5.0)
        g.update(3100.0, 0.0)
        for t in range(1, 5):                        # 低於 2850 只有 4 s
            assert g.update(2800.0, float(t))[0] is False
        assert g.update(3100.0, 5.0)[0] is False     # 爬回來 → 計時取消
        assert g.flips == 1

    def test_deep_sag_held_for_dwell_does_flip(self):
        """真的掉高度仍然要誠實地退回爬升段 —— 遲滯不是把故障蓋掉。"""
        g = LatchedAltGate(3000.0, hysteresis_m=150.0, dwell_s=5.0)
        g.update(3100.0, 0.0)
        seq = _run(g, [(float(t), 2800.0) for t in range(1, 9)])
        assert seq[-1] is True                       # 最終退回爬升段
        assert g.flips == 2

    def test_dwell_timer_resets_on_recovery(self):
        """爬回遲滯帶內就取消計時,不可以把不連續的秒數累加起來。"""
        g = LatchedAltGate(3000.0, hysteresis_m=150.0, dwell_s=5.0)
        g.update(3100.0, 0.0)
        t = 1.0
        for _ in range(10):                          # 反覆 4 s 深沉 + 1 s 恢復
            for _ in range(4):
                g.update(2800.0, t)
                t += 1.0
            g.update(3100.0, t)
            t += 1.0
        assert g.climbing is False
        assert g.flips == 1

    def test_none_altitude_holds_state(self):
        """沒有定位時維持現狀,不盲翻。"""
        g = LatchedAltGate(3000.0)
        g.update(3100.0, 0.0)
        assert g.update(None, 1.0) == (False, False)
        assert g.climbing is False

    def test_threshold_zero_disables_gate(self):
        """thr <= 0 = 閘門關閉(基準線對照用)⇒ 全程當巡航段。"""
        g = LatchedAltGate(0.0)
        assert g.update(10.0, 0.0)[0] is False
        assert g.enabled is False

    def test_zero_hysteresis_reproduces_bare_comparison(self):
        """遲滯設 0、dwell 設 0 時要退化成原本的裸比較(可做 A/B 對照)。"""
        g = LatchedAltGate(3000.0, hysteresis_m=0.0, dwell_s=0.0)
        g.update(3100.0, 0.0)
        assert g.update(2999.0, 1.0) == (True, True)


# ═══════════════════════════════════════════════════════════════════════
#  P91 真實航次重放
# ═══════════════════════════════════════════════════════════════════════

# P91 節點日誌中 UAV5 每次觸發 CLIMBING 時列印的高度(m,相對 HOME)。
# 全部落在閘門 3000 m 之下 1–23 m —— 這正是裸比較會抖動的區間。
P91_UAV5_SAG_ALTS = [2997, 2983, 2992, 2994, 2977, 2987, 2986, 2999, 2999, 2994]


class TestP91Replay:
    def _trace(self):
        """重建 UAV5 的高度序列:巡航 3100 m,其間十次淺下沉再爬回。"""
        samples, t = [], 0.0
        samples.append((t, 3100.0))                  # 先進巡航段
        t += 1.0
        for sag in P91_UAV5_SAG_ALTS:
            for _ in range(8):                       # 每次下沉持續 8 s
                samples.append((t, float(sag)))
                t += 1.0
            for _ in range(12):                      # 再爬回巡航高度
                samples.append((t, 3100.0))
                t += 1.0
        return samples

    def test_bare_comparison_chatters(self):
        """反證:沒有遲滯時,同一條軌跡翻 20 次(10 下 + 10 上)。"""
        g = LatchedAltGate(3000.0, hysteresis_m=0.0, dwell_s=0.0)
        _run(g, self._trace())
        assert g.flips == 1 + 2 * len(P91_UAV5_SAG_ALTS)

    def test_p91_uav5_altitude_trace_stops_chattering(self):
        """正證:150 m 遲滯 + 5 s 駐留 ⇒ 只剩最初那一次進巡航段。"""
        g = LatchedAltGate(3000.0, hysteresis_m=150.0, dwell_s=5.0)
        seq = _run(g, self._trace())
        assert g.flips == 1
        assert all(c is False for c in seq[1:])

    def test_hysteresis_clears_deepest_observed_sag(self):
        """遲滯要留餘裕:重放 cmdlog 得到的最低點是 2911 m。"""
        g = LatchedAltGate(3000.0, hysteresis_m=150.0)
        assert g.exit_alt_m < 2911.0


# ═══════════════════════════════════════════════════════════════════════
#  MonotonicRamp —— 側向展開只增不減
# ═══════════════════════════════════════════════════════════════════════

class TestMonotonicRamp:
    def test_advances_only_when_allowed(self):
        r = MonotonicRamp(120.0)
        r.update(0.0, advance=True)                  # 第一拍只記時間
        for t in range(1, 61):
            r.update(float(t), advance=True)
        assert r.fraction == pytest.approx(0.5, abs=1e-6)

    def test_freezes_instead_of_collapsing(self):
        """關鍵回歸:回到爬升段時展開角**凍結**,不可以塌回 0。"""
        r = MonotonicRamp(120.0)
        r.update(0.0, advance=True)
        for t in range(1, 61):
            r.update(float(t), advance=True)
        frozen = r.fraction
        for t in range(61, 121):                     # 60 s 爬升段
            assert r.update(float(t), advance=False) == pytest.approx(frozen)
        assert r.fraction == pytest.approx(frozen)

    def test_resumes_after_freeze(self):
        r = MonotonicRamp(100.0)
        r.update(0.0, advance=True)
        for t in range(1, 51):
            r.update(float(t), advance=True)
        for t in range(51, 101):
            r.update(float(t), advance=False)
        for t in range(101, 151):
            r.update(float(t), advance=True)
        assert r.fraction == pytest.approx(1.0)

    def test_never_exceeds_one(self):
        r = MonotonicRamp(10.0)
        for t in range(0, 100):
            assert 0.0 <= r.update(float(t), advance=True) <= 1.0

    def test_large_time_step_is_clamped(self):
        """排程暫停/掛起不該讓展開角一次跳滿。"""
        r = MonotonicRamp(120.0, max_step_s=3.0)
        r.update(0.0, advance=True)
        r.update(600.0, advance=True)                # 一次跳 600 s
        assert r.accumulated_s == pytest.approx(3.0)

    def test_zero_duration_is_fully_expanded(self):
        assert MonotonicRamp(0.0).update(0.0, advance=False) == 1.0
