# -*- coding: utf-8 -*-
"""tests/test_boot_clock.py — 飛控開機時間 → 牆鐘的線上對時（mission/boot_clock.py，2026-10-22）。

為什麼：3D 姿態樣本時間以前是「收到時刻」，SITL 成批送 ⇒ 比飛控取樣晚 0～32 ms 不等；懸停振盪 ~130°/s 時就是幾度。
鎖的是：
① 下包絡：結果落在最小延遲線上（不是平均延遲），而且永遠 ≤ 收到時刻。
② 斜率：SITL --speedup（斜率 1/N）、真飛控晶振 ±100 ppm 都跟得上；跨度不夠時假設 1。
③ 分段：開機時間倒退（重開機）、比線晚到超過 MAX_LATE_S（牆鐘跳／塞住）⇒ 重新開始。
④ 連線那一下的積壓（一口氣到）不會把線拉高。
⑤ 和 tools/attitude_fidelity_probe.py 的事後擬合（各寫各的）在同一組點上給出同一條線。
"""
from __future__ import annotations

import math
import random

import pytest

from mission import boot_clock as bc
from mission.boot_clock import BootClockSync, lower_support_line


def _stream(n, *, hz=50.0, base=1_700_000_000.0, floor=0.004, jitter=0.032, slope=1.0, boot0_ms=2000, seed=1):
    """(boot_ms, 收到時刻, 真正的送出時刻)：飛控每 1/hz 秒（飛控時間）送一則；牆鐘 = base + slope × 開機秒。"""
    rng = random.Random(seed)
    out = []
    for k in range(n):
        boot_ms = boot0_ms + int(round(1000.0 * k / hz))
        sent = base + slope * boot_ms / 1000.0
        out.append((boot_ms, sent + floor + rng.uniform(0.0, jitter), sent + floor))
    return out


def _run(clk, stream, skip=0):
    errs = []
    for i, (bm, t, true_t) in enumerate(stream):
        m = clk.map(bm, t)
        assert m <= t + 1e-12, '取樣時刻不可以晚於收到時刻'
        if i >= skip:
            errs.append(m - true_t)
    return errs


def test_maps_onto_the_minimum_latency_line_not_the_average():
    errs = _run(BootClockSync(), _stream(3000), skip=250)
    assert min(errs) > -0.0005 and max(errs) < 0.002, (min(errs), max(errs))   # 擬合斜率外推：± 不到 1 ms
    errs.sort()
    assert errs[len(errs) // 2] < 0.0007, '中位數貼著最小延遲（收到時刻平均晚 16 ms）'


@pytest.mark.parametrize('slope', [0.5, 0.2], ids=['speedup2', 'speedup5'])
def test_follows_a_sitl_speedup_slope(slope):
    clk = BootClockSync()
    s = _stream(4000, slope=slope, jitter=0.01)      # speedup N：飛控時間 20 ms 一則、牆鐘上只隔 20/N ms
    errs = _run(clk, s, skip=int(2 * bc.MIN_FIT_SPAN_S * 50))
    assert clk.slope == pytest.approx(slope, rel=2e-3)
    assert max(abs(e) for e in errs) < 0.002, max(abs(e) for e in errs)


@pytest.mark.parametrize('ppm', [100.0, -100.0])
def test_follows_crystal_drift_over_ten_minutes(ppm):
    clk = BootClockSync()
    errs = _run(clk, _stream(30000, slope=1.0 + ppm * 1e-6), skip=1000)
    assert clk.slope == pytest.approx(1.0 + ppm * 1e-6, abs=3e-5)
    assert min(errs) > -0.0007 and max(errs) < 0.002, (min(errs), max(errs))


def test_until_the_fit_span_is_reached_the_slope_is_one():
    clk = BootClockSync()
    _run(clk, _stream(int(bc.MIN_FIT_SPAN_S * 50) - 50, slope=0.5, hz=100.0, jitter=0.01))
    assert clk.slope == 1.0


def test_reboot_and_late_arrivals_start_a_new_segment():
    clk = BootClockSync()
    _run(clk, _stream(500, boot0_ms=600000))
    assert clk.segments == 1
    s2 = _stream(500, boot0_ms=1000, base=1_700_000_700.0)                 # 開機時間從 1 s 重來
    errs = _run(clk, s2, skip=100)
    assert clk.reboots == 1 and clk.segments == 2 and max(errs) < 0.03
    s3 = [(bm, t + 10.0, tt + 10.0) for bm, t, tt in _stream(500, boot0_ms=11000, base=1_700_000_700.0)]
    errs = _run(clk, s3, skip=100)                                        # 牆鐘往後跳 10 s
    assert clk.late_resets == 1 and clk.segments == 3 and max(errs) < 0.03


def test_the_connect_backlog_does_not_lift_the_line():
    """SITLLink 連線那一下先同步讀參數 ~1.6 s 才進主迴圈：之前的訊息一口氣到。之後還是落在最小延遲線上。"""
    s = _stream(2000)
    t_drain = s[80][1]
    s = [(bm, max(t, t_drain), tt) if i < 80 else (bm, t, tt) for i, (bm, t, tt) in enumerate(s)]
    errs = _run(BootClockSync(), s, skip=300)
    assert min(errs) > -0.0005 and max(errs) < 0.002, (min(errs), max(errs))


def test_non_finite_inputs_return_the_receipt_time():
    clk = BootClockSync()
    assert clk.map(float('nan'), 5.0) == 5.0 and clk.map(1000, float('inf')) == float('inf')
    assert clk.map(1000, 1_700_000_000.0) == 1_700_000_000.0


def test_support_line_agrees_with_the_probes_independent_fit():
    from tools.attitude_fidelity_probe import fit_lower_envelope
    rng = random.Random(3)
    for _ in range(50):
        n = rng.randint(2, 300)
        xs = sorted(rng.uniform(0, 600) for _ in range(n))
        b0 = rng.uniform(0.2, 2.0)
        ys = [5.0 + b0 * x + rng.uniform(0, 0.03) for x in xs]
        mine = lower_support_line(list(zip(xs, ys)))
        theirs = fit_lower_envelope(xs, ys)
        assert mine == pytest.approx(theirs, rel=1e-12, abs=1e-9)
        a, b = mine
        assert all(y >= a + b * x - 1e-9 for x, y in zip(xs, ys)), '每一點都在線上或上方'
    assert lower_support_line([(1.0, 2.0)]) is None and lower_support_line([(1.0, 2.0), (1.0, 3.0)]) is None


def test_constants_are_the_ones_the_offline_replay_chose():
    """2026-10-22 用真 SITL 的 50 Hz 姿態離線比較：20 s 窗口＋斜率鎖 1 ⇒ 懸停姿態差 p95 0.25°；300 s＋擬合斜率 ⇒ 0.07°。"""
    assert bc.WINDOW_S == 300.0 and bc.BLOCK_S == 0.5 and bc.MIN_FIT_SPAN_S == 10.0
    assert bc.REBOOT_BACK_S == 1.0 and bc.MAX_LATE_S == 3.0
    assert not math.isnan(bc.WINDOW_S)
