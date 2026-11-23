"""survey.clock_fit：逐張邊緣配對（不依序硬配）＋時鐘擬合。"""

from pathlib import Path

import numpy as np
import pytest

from survey import dflog, ingest, pi_session, synth
from survey.clock_fit import ClockFitError, fit_clock

REPO = Path(__file__).resolve().parents[1]


def _events(seed=0, n_strips=4, per_strip=40, base=0.88, wobble=0.02, turn=(25.0, 40.0)):
    """帶內間隔在 base 附近（50 Hz 量化）、帶間轉彎長間隔。"""
    rng = np.random.default_rng(seed)
    t, out = 100.0, []
    for s in range(n_strips):
        for _ in range(per_strip):
            out.append(t)
            t += round((base + rng.uniform(-wobble, wobble)) / 0.02) * 0.02
        t += rng.uniform(*turn)
    return np.asarray(out)


def _pi_from_fc(t_fc, a, b, jitter, seed=1):
    rng = np.random.default_rng(seed)
    return (t_fc - b) / a + rng.normal(0, jitter, len(t_fc))


def test_recovers_offset_drift_and_reports_unmatched_exactly():
    t_fc = _events()
    a, b = 1 - 63e-6, 4321.125
    drop = [3, 77, 150]
    extra_fc_t = t_fc[60] + 0.31                        # 飛控多一筆（Pi 沒看到）
    keep = np.setdiff1d(np.arange(len(t_fc)), drop)
    t_pi = _pi_from_fc(t_fc[keep], a, b, 1e-4)
    spurious = (t_fc[100] + 0.5 - b) / a                # Pi 多一個雜訊邊緣
    t_pi = np.sort(np.append(t_pi, spurious))
    t_fc2 = np.sort(np.append(t_fc, extra_fc_t))
    fit = fit_clock(t_fc2, t_pi)
    exp_fc = sorted([int(np.searchsorted(t_fc2, t_fc[i])) for i in drop] + [int(np.searchsorted(t_fc2, extra_fc_t))])
    assert fit.unmatched_fc.tolist() == exp_fc
    assert fit.unmatched_pi.tolist() == [int(np.searchsorted(t_pi, spurious))]
    assert fit.n_pairs == len(t_fc) - 3
    assert fit.drift_ppm == pytest.approx(-63.0, abs=1.0)
    span = np.linspace(t_pi[0], t_pi[-1], 7)
    assert np.max(np.abs(fit.pi_to_fc(span) - (a * span + b))) < 1e-4
    assert fit.rms_s == pytest.approx(1e-4, rel=0.3)
    assert fit.n_inliers == fit.n_pairs
    # 每一對都對到真正的同一個事件
    t_true = (t_fc2[fit.pairs[:, 0]] - b) / a
    assert np.max(np.abs(t_pi[fit.pairs[:, 1]] - t_true)) < 1e-3


def test_unsorted_inputs_return_original_indices():
    t_fc = _events(n_strips=2, per_strip=30)
    t_pi = _pi_from_fc(t_fc, 1.0, -50.0, 5e-5)
    perm_f = np.random.default_rng(3).permutation(len(t_fc))
    perm_p = np.random.default_rng(4).permutation(len(t_pi))
    fit = fit_clock(t_fc[perm_f], t_pi[perm_p])
    # 原始索引：perm_f[i] 與 perm_p[j] 對到同一個事件
    for i, j in fit.pairs:
        assert perm_f[i] == perm_p[j]
    assert fit.n_pairs == len(t_fc) and fit.unmatched_fc.size == 0


def test_latency_spike_is_paired_but_excluded_from_fit():
    t_fc = _events(n_strips=2, per_strip=30)
    t_pi = _pi_from_fc(t_fc, 1.0, 10.0, 5e-5)
    t_pi[17] += 0.004                                   # 核心排程延遲 4 ms
    fit = fit_clock(t_fc, t_pi)
    assert fit.n_pairs == len(t_fc)
    k = int(np.nonzero(fit.pairs[:, 1] == 17)[0][0])
    assert not fit.inlier[k] and fit.n_inliers == len(t_fc) - 1
    assert fit.residuals_s[k] == pytest.approx(-0.004, abs=3e-4)
    assert fit.rms_s < 1e-4                             # 離群點沒有拉壞擬合


def test_near_duplicate_glitch_edge_is_unmatched():
    t_fc = _events(n_strips=2, per_strip=30)
    t_pi = _pi_from_fc(t_fc, 1.0, 10.0, 5e-5)
    glitch = t_pi[20] + 0.005                           # 同一個脈衝的彈跳
    t_pi2 = np.sort(np.append(t_pi, glitch))
    fit = fit_clock(t_fc, t_pi2)
    assert fit.unmatched_pi.tolist() == [int(np.searchsorted(t_pi2, glitch))]
    assert fit.unmatched_fc.size == 0


def test_perfectly_periodic_sequence_is_rejected_as_ambiguous():
    t_fc = 100.0 + np.arange(60) * 0.88                 # 單條帶、完全等間隔，兩端各少一筆就分不出錯幾張
    t_pi = t_fc[1:-1] - 7.0
    with pytest.raises(ClockFitError, match="歧義"):
        fit_clock(t_fc[1:], t_pi)


def test_single_periodic_strip_with_equal_counts_is_accepted():
    """單帶等間隔、兩邊張數相同：錯一張的解釋要同時假設 Pi 少一個又多一個邊緣 ⇒ 取全配對，不拒絕。"""
    t_fc = 100.0 + np.arange(17) * 0.88
    fit = fit_clock(t_fc, t_fc - 7.0)
    assert fit.n_pairs == 17 and fit.unmatched_fc.size == 0 and fit.unmatched_pi.size == 0
    assert fit.ambiguity == pytest.approx(16 / 17)


def test_long_calm_strips_are_not_rejected():
    """無風長帶：帶內間隔完全相同，錯一張的對齊配對比例 >0.97，但每個轉彎都少配 ≥2 個 ⇒ 仍要接受且對齊正確。"""
    t, fc = 50.0, []
    for gap in (30.37, 41.12, 0.0):
        fc += [t + k * 0.88 for k in range(200)]
        t = fc[-1] + gap
    t_fc = np.asarray(fc)
    keep = np.setdiff1d(np.arange(len(t_fc)), [57, 399])
    t_pi = _pi_from_fc(t_fc[keep], 1 + 11e-6, -321.0, 5e-5)
    fit = fit_clock(t_fc, t_pi)
    assert fit.ambiguity > 0.97
    assert fit.unmatched_fc.tolist() == [57, 399] and fit.unmatched_pi.size == 0
    assert np.array_equal(keep[fit.pairs[:, 1]], fit.pairs[:, 0])


def test_too_few_or_bad_inputs():
    with pytest.raises(ClockFitError, match="至少各 3 筆"):
        fit_clock([1.0, 2.0], [1.0, 2.0, 3.0])
    with pytest.raises(ClockFitError, match="重複"):
        fit_clock([1.0, 2.0, 2.0, 3.0], [1.0, 2.0, 3.0])
    with pytest.raises(ClockFitError, match="配對會混淆"):
        fit_clock([0.0, 0.1, 0.2, 0.3], [0.0, 0.1, 0.2, 0.3], match_tol_s=0.05)
    with pytest.raises(ClockFitError, match="非有限"):
        fit_clock([0.0, np.nan, 2.0], [0.0, 1.0, 2.0])


def test_real_sitl_cam_times_with_synthetic_session(tmp_path):
    """SITL S1：用真實 SITL 日誌的 CAM 時刻合成 Pi 會期，跑 ingest 的配對。"""
    p = REPO / "sitl/vtol/i102/logs/00000019.BIN"
    if not p.exists():
        pytest.skip("本機沒有 SITL 日誌")
    log = dflog.read_dflog(p)
    cam = dflog.cam_events(log)
    truth = synth.synth_session_from_fc(tmp_path / "s", cam["TimeUS"], clock_a=1 + 18e-6, clock_b_s=-2000.5,
                                        boottime_minus_monotonic_ns=7_000_000_000, jitter_s=1e-4,
                                        drop_index=[0, 100, 225], seed=11)
    sess = pi_session.read_pi_session(tmp_path / "s", verify_sha256=True)
    fit, fc = ingest.fit_session(log, sess)
    assert fit.unmatched_fc.tolist() == [0, 100, 225]
    assert fit.unmatched_pi.size == 0 and fit.n_pairs == 223
    assert fit.drift_ppm == pytest.approx(18.0, abs=1.0)
    assert fit.rms_s < 0.5e-3
    for fc_i, pi_j in fit.pairs:
        assert truth["frames"][int(pi_j)]["fc_index"] == int(fc_i)
