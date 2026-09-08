"""tests/test_dubins.py — 契約 §5 不變量與規格 §17.2 之 test_dubins 項目。

six_words_vs_bruteforce；omega_closed_form；loop_closed_form(3πR+2R−s)；uturn_closed_form；
transfer_state_pairing_four_types（只斷言長度不斷言 word）；discretize_circumscribed_error≤e_max；
另含 sample 端點、對稱性、風中時間（規格 §W.5–W.7 數值）、批次時間、離散步角 D27。
"""
from __future__ import annotations

import math

import numpy as np
import pytest
import shapely

from hv_dccpp import dubins as D
from hv_dccpp.dubins import (
    WORDS,
    DubinsPath,
    DubinsSegment,
    Pose,
    discretize,
    discretize_step_rad,
    dubins_all,
    dubins_all_times,
    dubins_all_words,
    dubins_path,
    loop_length,
    omega_length,
    parallel_transfer_pose_pair,
    path_time,
    path_time_and_energy,
    skip_k,
    uturn_length,
)

TWO_PI = 2.0 * math.pi
R_A, R_B = 200.0, 120.0
VA_A, VA_B = 21.5, 19.0
WIND_E5 = np.array([5.0, 0.0])          # 西風 5 m/s（吹向東）


# ---------------------------------------------------------------------------
# 獨立幾何 oracle（θ 自北順時針；直接以切線／切圓構造，不用三角閉式）
# ---------------------------------------------------------------------------
def _t_hat(th: float) -> np.ndarray:
    return np.array([math.sin(th), math.cos(th)])


def _n_hat(th: float) -> np.ndarray:
    return np.array([math.cos(th), -math.sin(th)])


def _center(pose: np.ndarray, turn: str, R: float) -> np.ndarray:
    sign = 1.0 if turn == "R" else -1.0
    return pose[:2] + sign * R * _n_hat(pose[2])


def _arc(th_a: float, th_b: float, turn: str) -> float:
    d = (th_b - th_a) % TWO_PI if turn == "R" else (th_a - th_b) % TWO_PI
    return 0.0 if d > TWO_PI - 1e-9 else d


def _heading_of(vec: np.ndarray) -> float:
    return math.atan2(vec[0], vec[1])          # 自北順時針


def oracle_word_length(start: np.ndarray, end: np.ndarray, R: float, word: str) -> float:
    c1 = _center(start, word[0], R)
    c3 = _center(end, word[2], R)
    v = c3 - c1
    Dc = float(np.hypot(*v))
    if word[1] == "S":
        if Dc < 1e-9 and word[0] == word[2]:
            return R * _arc(start[2], end[2], word[0])
        dirc = _heading_of(v)
        if word[0] == word[2]:
            th_s, p = dirc, Dc
        else:
            if Dc < 2.0 * R:
                return math.inf
            p = math.sqrt(Dc * Dc - 4.0 * R * R)
            th_s = dirc + math.atan2(2.0 * R, p) if word[0] == "R" else dirc - math.atan2(2.0 * R, p)
        return R * (_arc(start[2], th_s, word[0]) + _arc(th_s, end[2], word[2])) + p
    # CCC：兩個候選中圓，取中弧 ≥ π 者（Dubins 條件）
    if Dc > 4.0 * R:
        return math.inf
    dirc = _heading_of(v) if Dc > 0 else 0.0
    th1 = math.acos(min(1.0, Dc / (4.0 * R)))
    best = math.inf
    for side in (+1.0, -1.0):
        th12 = dirc + side * th1
        c2 = c1 + 2.0 * R * _t_hat(th12)
        th32 = _heading_of(c2 - c3)
        if word[0] == "R":
            h1 = th12 + 0.5 * math.pi
            h2 = th32 + 0.5 * math.pi
        else:
            h1 = th12 - 0.5 * math.pi
            h2 = th32 - 0.5 * math.pi
        t = _arc(start[2], h1, word[0])
        p = _arc(h1, h2, word[1])
        q = _arc(h2, end[2], word[2])
        if p < math.pi - 1e-9:
            continue
        best = min(best, R * (t + p + q))
    return best


def _rand_pairs(rng: np.random.Generator, n: int, R: float, span: float):
    s = np.column_stack([rng.uniform(-span, span, n), rng.uniform(-span, span, n), rng.uniform(-TWO_PI, TWO_PI, n)])
    e = np.column_stack([rng.uniform(-span, span, n), rng.uniform(-span, span, n), rng.uniform(-TWO_PI, TWO_PI, n)])
    return s, e


def _segment_end(seg: DubinsSegment, R: float) -> tuple[float, float, float]:
    """由段幾何獨立重算終點（不用 seg.end）。"""
    x0, y0, th0 = seg.start.x_m, seg.start.y_m, seg.start.theta_rad
    if seg.kind == "S":
        return x0 + seg.length_m * math.sin(th0), y0 + seg.length_m * math.cos(th0), th0
    sign = 1.0 if seg.kind == "R" else -1.0
    cx, cy = seg.center
    th = th0 + seg.dtheta_rad
    return cx - sign * R * math.cos(th), cy + sign * R * math.sin(th), th


def _ang_close(a: float, b: float, tol: float = 1e-7) -> bool:
    return abs((a - b + math.pi) % TWO_PI - math.pi) < tol


# ---------------------------------------------------------------------------
# 基本約定
# ---------------------------------------------------------------------------
def test_pose_as_array():
    p = Pose(1.0, 2.0, 0.5)
    assert np.array_equal(p.as_array(), np.array([1.0, 2.0, 0.5]))


def test_heading_convention_quarter_circles():
    # 自北向 (0,0) 右轉 90° → 落於 (R, R) 朝東；左轉 → (−R, R) 朝西。
    L, w = dubins_all(np.array([[0.0, 0.0, 0.0]]), np.array([[R_A, R_A, math.pi / 2]]), R_A)
    assert L[0] == pytest.approx(math.pi / 2 * R_A, rel=1e-12)
    L, w = dubins_all(np.array([[0.0, 0.0, 0.0]]), np.array([[-R_A, R_A, -math.pi / 2]]), R_A)
    assert L[0] == pytest.approx(math.pi / 2 * R_A, rel=1e-12)
    p = dubins_path(Pose(0, 0, 0), Pose(R_A, R_A, math.pi / 2), R_A)
    arcs = [s for s in p.segments if s.length_m > 1e-9]
    assert len(arcs) == 1 and arcs[0].kind == "R" and arcs[0].dtheta_rad > 0
    assert arcs[0].center == pytest.approx((R_A, 0.0))
    p = dubins_path(Pose(0, 0, 0), Pose(-R_A, R_A, -math.pi / 2), R_A)
    arcs = [s for s in p.segments if s.length_m > 1e-9]
    assert len(arcs) == 1 and arcs[0].kind == "L" and arcs[0].dtheta_rad < 0
    assert arcs[0].center == pytest.approx((-R_A, 0.0))


def test_straight_line_and_zero_length():
    L, w = dubins_all([Pose(0, 0, 0)], [Pose(0, 1000, 0)], R_A)
    assert L[0] == pytest.approx(1000.0, abs=1e-9)
    p = dubins_path(Pose(0, 0, 0), Pose(0, 0, 0), R_A)
    assert p.length_m == pytest.approx(0.0, abs=1e-9)
    s = p.sample(5.0)
    assert s.shape[0] >= 2 and np.allclose(s[0], [0, 0, 0]) and np.allclose(s[-1], [0, 0, 0])
    assert len(discretize(p, 3.0, math.radians(30))) >= 1


def test_segment_semantics():
    a, b = parallel_transfer_pose_pair(np.array([7068.0, 503.0]), math.pi / 2, 3 * 168.05)
    p = dubins_path(a, b, R_A)
    assert len(p.segments) == 3 and p.word in WORDS
    total = 0.0
    for seg, kind in zip(p.segments, p.word):
        assert seg.kind == kind
        if seg.kind == "S":
            assert seg.center is None and seg.dtheta_rad == 0.0
        else:
            assert seg.center is not None
            assert seg.length_m == pytest.approx(p.R_m * abs(seg.dtheta_rad), rel=1e-12)
            assert (seg.dtheta_rad > 0) == (seg.kind == "R")
            assert (seg.dtheta_rad < 0) == (seg.kind == "L")
        total += seg.length_m
    assert total == pytest.approx(p.length_m, rel=1e-12)
    assert p.start == a and p.end == b
    # 段間連續
    for s0, s1 in zip(p.segments[:-1], p.segments[1:]):
        assert s0.end == s1.start


# ---------------------------------------------------------------------------
# six_words_vs_bruteforce
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("R,span", [(R_A, 1500.0), (R_B, 600.0), (R_A, 250.0)])
def test_six_words_vs_bruteforce(R, span):
    rng = np.random.default_rng(int(R) + int(span))
    S, E = _rand_pairs(rng, 150, R, span)
    L_all = dubins_all_words(S, E, R)
    best, idx = dubins_all(S, E, R)
    for i in range(S.shape[0]):
        oracle = np.array([oracle_word_length(S[i], E[i], R, w) for w in WORDS])
        for k in range(6):
            if np.isfinite(oracle[k]) or np.isfinite(L_all[i, k]):
                assert np.isfinite(oracle[k]) == np.isfinite(L_all[i, k]), (i, WORDS[k], oracle[k], L_all[i, k])
                if np.isfinite(oracle[k]):
                    assert L_all[i, k] == pytest.approx(oracle[k], rel=1e-9, abs=1e-7), (i, WORDS[k])
        assert best[i] == pytest.approx(oracle.min(), rel=1e-9, abs=1e-7)
        assert best[i] == L_all[i].min()
        assert L_all[i, idx[i]] == best[i]
        # 每個可行 word 之路徑皆閉合於目標（以段幾何獨立重算，不用 seg.end）
        for k in range(6):
            if not np.isfinite(L_all[i, k]):
                continue
            path = dubins_path(Pose(*S[i]), Pose(*E[i]), R, WORDS[k])
            assert path.length_m == pytest.approx(L_all[i, k], rel=1e-12)
            x, y, th = S[i]
            for seg in path.segments:
                assert seg.start.x_m == pytest.approx(x, abs=1e-6)
                assert seg.start.y_m == pytest.approx(y, abs=1e-6)
                x, y, th = _segment_end(seg, R)
            assert math.hypot(x - E[i, 0], y - E[i, 1]) < 1e-6 * R
            assert _ang_close(th, E[i, 2])


def test_dubins_path_equals_dubins_all_and_word_idx_dtype():
    rng = np.random.default_rng(7)
    S, E = _rand_pairs(rng, 60, R_A, 2000.0)
    best, idx = dubins_all(S, E, R_A)
    assert best.shape == (60,) and idx.shape == (60,) and idx.dtype == np.int8
    assert np.all(np.isfinite(best))
    for i in range(60):
        a, b = Pose(*S[i]), Pose(*E[i])
        p = dubins_path(a, b, R_A)
        assert p.length_m == dubins_all([a], [b], R_A)[0][0]
        assert p.word == WORDS[idx[i]]
    with pytest.raises(ValueError):
        dubins_path(Pose(0, 0, 0), Pose(0, 1000, 0), R_A, word="XYZ")
    with pytest.raises(ValueError):
        dubins_all(S, E, 0.0)


# ---------------------------------------------------------------------------
# 閉式（D5、§8.3）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("R,s", [(R_A, 126.0), (R_A, 157.5), (R_A, 168.05), (R_B, 103.0), (R_B, 105.0),
                                 (R_A, 0.0), (R_A, 399.0), (R_B, 50.0)])
@pytest.mark.parametrize("side", [+1.0, -1.0])
@pytest.mark.parametrize("theta_out", [math.pi / 2, 3 * math.pi / 2, 0.3, -2.1])
def test_omega_and_loop_closed_form(R, s, side, theta_out):
    a, b = parallel_transfer_pose_pair(np.array([7068.0, 503.0]), theta_out, side * s)
    L = dubins_all_words([a], [b], R)[0]
    omega = omega_length(s, R)
    ccc = min(L[4], L[5])                     # RLR 或 LRL（依側向方向），只斷言長度
    assert ccc == pytest.approx(omega, rel=1e-9)
    loop = loop_length(s, R)
    csc_same = min(L[0], L[1])                # LSL 或 RSR
    assert csc_same == pytest.approx(loop, rel=1e-9)
    assert loop == pytest.approx(3 * math.pi * R + (2 * R - s), rel=1e-12)
    if s < 2 * R - 1e-9:
        assert ccc <= csc_same


def test_omega_special_values():
    assert omega_length(0.0, R_A) == pytest.approx(R_A * (math.pi + 4 * math.pi / 3), rel=1e-12)   # 7.33R
    assert omega_length(0.0, R_A) / R_A == pytest.approx(7.33, abs=0.01)
    assert loop_length(0.0, R_A) / R_A == pytest.approx(11.42, abs=0.01)
    assert omega_length(2 * R_A, R_A) == pytest.approx(math.pi * R_A, rel=1e-12)
    assert loop_length(2 * R_A, R_A) == pytest.approx(math.pi * R_A, rel=1e-12)
    assert uturn_length(2 * R_A, R_A) == pytest.approx(math.pi * R_A, rel=1e-12)
    # 規格 §8.3 數值：A s=157.5 → Ω 1268 m、U(3s) 701 m；s=168.0 → 1253 / 732；B s=103 → 749 / 445
    assert omega_length(157.5, R_A) == pytest.approx(1268.0, abs=1.0)
    assert uturn_length(3 * 157.5, R_A) == pytest.approx(701.0, abs=1.0)
    assert omega_length(168.0, R_A) == pytest.approx(1253.0, abs=1.0)
    assert uturn_length(3 * 168.0, R_A) == pytest.approx(732.0, abs=1.0)
    assert omega_length(103.0, R_B) == pytest.approx(749.0, abs=1.0)
    assert uturn_length(3 * 103.0, R_B) == pytest.approx(445.0, abs=1.0)
    assert omega_length(126.0, R_A) == pytest.approx(1310.0, abs=2.0)      # "Ω 1.31 km"
    assert uturn_length(4 * 126.0, R_A) == pytest.approx(732.0, abs=1.0)


@pytest.mark.parametrize("R,s", [(R_A, 126.0), (R_A, 157.5), (R_A, 168.05), (R_B, 103.0), (R_B, 105.0),
                                 (R_A, 100.0), (R_B, 60.0), (R_A, 450.0)])
@pytest.mark.parametrize("side", [+1.0, -1.0])
def test_uturn_closed_form_skip_k(R, s, side):
    k = skip_k(s, R)
    assert k == math.ceil(2 * R / s - 1e-9) and k * s >= 2 * R - 1e-6
    a, b = parallel_transfer_pose_pair(np.array([100.0, -50.0]), 1.234, side * k * s)
    best, idx = dubins_all([a], [b], R)
    L = dubins_all_words([a], [b], R)[0]
    u = uturn_length(k * s, R)
    assert u == pytest.approx(math.pi * R + (k * s - 2 * R), rel=1e-12)
    assert min(L[0], L[1]) == pytest.approx(u, rel=1e-9)
    assert best[0] == pytest.approx(u, rel=1e-9)               # skip-k U 型即最短


def test_skip_k_values():
    assert skip_k(168.05, R_A) == 3
    assert skip_k(157.5, R_A) == 3
    assert skip_k(126.0, R_A) == 4
    assert skip_k(102.81, R_B) == 3
    assert skip_k(100.0, R_A) == 4                     # 2R/s 恰為整數
    assert skip_k(400.0, R_A) == 1
    with pytest.raises(ValueError):
        skip_k(0.0, R_A)


def test_transfer_state_pairing_four_types():
    """§8.2 四型：反向 k=1、k=2 皆為 Ω（s<2R）／U（ks≥2R）；同向 racetrack ≈ 2πR + hypot(L, s)。只斷言長度。"""
    s, L_strip, R = 168.05, 3136.0, R_A
    pR = np.array([7068.0, 503.0])
    pL = pR - L_strip * np.array([1.0, 0.0])
    th_LR = math.pi / 2
    # k=1：S^{LR}_l (pR, θ_LR) → S^R_{l'} (pR + s n̂, θ_RL)
    a, b = parallel_transfer_pose_pair(pR, th_LR, s)
    assert dubins_all([a], [b], R)[0][0] == pytest.approx(omega_length(s, R), rel=1e-9)
    # k=2：S^{RL}_l (pL, θ_RL) → S^L_{l'} (pL − s n̂(θ_LR) = pL + s n̂(θ_RL)?, θ_LR)：以 θ_RL 為出發方位，側移在其右法向
    a2, b2 = parallel_transfer_pose_pair(pL, th_LR + math.pi, -s)
    assert b2.x_m == pytest.approx(pL[0]) and b2.y_m == pytest.approx(pL[1] - s)   # 同一條目標帶（南側）
    assert dubins_all([a2], [b2], R)[0][0] == pytest.approx(omega_length(s, R), rel=1e-9)
    # k=0（同向 racetrack）：S^{LR}_l → S^L_{l'}（對側端點，同方位）
    a0, b0 = parallel_transfer_pose_pair(pR, th_LR, s, same_direction=True, strip_len_m=L_strip)
    assert (b0.x_m, b0.y_m, b0.theta_rad) == pytest.approx((pL[0], pL[1] - s, th_LR))
    L0 = dubins_all_words([a0], [b0], R)[0]
    assert min(L0[0], L0[1]) == pytest.approx(2 * math.pi * R + math.hypot(L_strip, s), rel=1e-9)
    assert dubins_all([a0], [b0], R)[0][0] == pytest.approx(2 * math.pi * R + math.hypot(L_strip, s), rel=1e-9)
    # k=3：S^{RL}_l → S^R_{l'}
    a3, b3 = parallel_transfer_pose_pair(pL, th_LR + math.pi, -s, same_direction=True, strip_len_m=L_strip)
    assert (b3.x_m, b3.y_m) == pytest.approx((pR[0], pR[1] - s))
    assert _ang_close(b3.theta_rad, th_LR + math.pi)
    assert dubins_all([a3], [b3], R)[0][0] == pytest.approx(2 * math.pi * R + math.hypot(L_strip, s), rel=1e-9)
    # skip-3 U 型（W.5）：732.5 m
    a4, b4 = parallel_transfer_pose_pair(pR, th_LR, 3 * s)
    assert dubins_all([a4], [b4], R)[0][0] == pytest.approx(732.5, abs=0.1)


def test_parallel_transfer_pose_pair_geometry():
    th = 0.7
    a, b = parallel_transfer_pose_pair(np.array([10.0, 20.0]), th, 50.0)
    assert (a.x_m, a.y_m, a.theta_rad) == (10.0, 20.0, th)
    assert (b.x_m, b.y_m) == pytest.approx((10.0 + 50.0 * math.cos(th), 20.0 - 50.0 * math.sin(th)))
    assert b.theta_rad == pytest.approx(th + math.pi)
    a, b = parallel_transfer_pose_pair(np.array([10.0, 20.0]), th, 50.0, same_direction=True, strip_len_m=300.0)
    assert (b.x_m, b.y_m) == pytest.approx((10.0 + 50.0 * math.cos(th) - 300.0 * math.sin(th),
                                            20.0 - 50.0 * math.sin(th) - 300.0 * math.cos(th)))
    assert b.theta_rad == pytest.approx(th)


# ---------------------------------------------------------------------------
# 對稱性
# ---------------------------------------------------------------------------
def test_symmetry_mirror_and_reverse():
    rng = np.random.default_rng(11)
    S, E = _rand_pairs(rng, 80, R_A, 1200.0)
    L = dubins_all_words(S, E, R_A)
    # 鏡射（x → −x, θ → −θ）：L↔R
    Sm = S * np.array([-1.0, 1.0, -1.0])
    Em = E * np.array([-1.0, 1.0, -1.0])
    Lm = dubins_all_words(Sm, Em, R_A)
    mirror = [1, 0, 3, 2, 5, 4]       # LSL↔RSR, LSR↔RSL, RLR↔LRL
    for k in range(6):
        np.testing.assert_allclose(Lm[:, mirror[k]], L[:, k], rtol=1e-9, atol=1e-7)
    # 反向（起訖對調、方位 +π）：word 倒序且 L↔R → LSL↔RSR, LSR↔LSR, RSL↔RSL, RLR↔LRL
    Sr = E + np.array([0.0, 0.0, math.pi])
    Er = S + np.array([0.0, 0.0, math.pi])
    Lr = dubins_all_words(Sr, Er, R_A)
    rev = [1, 0, 2, 3, 5, 4]
    for k in range(6):
        np.testing.assert_allclose(Lr[:, rev[k]], L[:, k], rtol=1e-9, atol=1e-7)
    # 平移／旋轉不變
    dth = 0.9
    c, s = math.cos(dth), math.sin(dth)
    rot = np.array([[c, s], [-s, c]])         # 方位 +dth（順時針）之座標旋轉
    St = np.column_stack([S[:, :2] @ rot.T + np.array([300.0, -700.0]), S[:, 2] + dth])
    Et = np.column_stack([E[:, :2] @ rot.T + np.array([300.0, -700.0]), E[:, 2] + dth])
    np.testing.assert_allclose(dubins_all_words(St, Et, R_A), L, rtol=1e-9, atol=1e-6)


# ---------------------------------------------------------------------------
# sample / polyline
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("word", WORDS)
def test_sample_endpoints_and_spacing(word):
    rng = np.random.default_rng(100 + WORDS.index(word))
    a = Pose(0.0, 0.0, 0.4)
    # 挑一對使指定 word 可行
    for _ in range(200):
        b = Pose(rng.uniform(-600, 600), rng.uniform(-600, 600), rng.uniform(-math.pi, math.pi))
        L = dubins_all_words([a], [b], R_A)[0][WORDS.index(word)]
        if np.isfinite(L):
            break
    p = dubins_path(a, b, R_A, word)
    ds = 7.0
    s = p.sample(ds)
    assert s.ndim == 2 and s.shape[1] == 3
    np.testing.assert_array_equal(s[0], a.as_array())
    np.testing.assert_array_equal(s[-1], b.as_array())
    assert np.allclose(s[0], [p.start.x_m, p.start.y_m, p.start.theta_rad])
    assert np.allclose(s[-1], [p.end.x_m, p.end.y_m, p.end.theta_rad])
    step = np.hypot(np.diff(s[:, 0]), np.diff(s[:, 1]))
    assert step.max() <= ds + 1e-9
    chord = step.sum()
    assert chord <= p.length_m + 1e-9 and chord >= p.length_m * (1 - (ds / R_A) ** 2 / 24) - 1e-6
    # 取樣點皆在對應段幾何上（弧：距圓心 R；直線：在線段上）
    ok = np.zeros(s.shape[0], bool)
    for seg in p.segments:
        if seg.length_m <= 1e-9:
            continue
        if seg.kind == "S":
            ls = shapely.LineString([(seg.start.x_m, seg.start.y_m), (seg.end.x_m, seg.end.y_m)])
            d = np.array([ls.distance(shapely.Point(x, y)) for x, y in s[:, :2]])
        else:
            cx, cy = seg.center
            d = np.abs(np.hypot(s[:, 0] - cx, s[:, 1] - cy) - R_A)
        ok |= d < 1e-6
    assert ok.all()
    # 中間方位連續（相鄰差 < ds/R + ε）
    dth = np.abs(np.diff(s[:-1, 2]))
    assert dth.max() <= ds / R_A + 1e-9
    line = p.polyline(ds)
    assert isinstance(line, shapely.LineString)
    assert line.length == pytest.approx(chord, rel=1e-12)


# ---------------------------------------------------------------------------
# discretize（D27、§8.5）
# ---------------------------------------------------------------------------
def test_discretize_step_rad_d27():
    # 精確式 2·acos(R/(R+e))：A 19.72°、B 25.36°；規格引述之 19.8°/25.6° 為小角近似 2·sqrt(2e/R)
    assert math.degrees(discretize_step_rad(R_A, 3.0, math.radians(30))) == pytest.approx(2 * math.degrees(math.acos(200 / 203)), rel=1e-12)
    assert math.degrees(discretize_step_rad(R_A, 3.0, math.radians(30))) == pytest.approx(19.8, abs=0.1)
    assert math.degrees(discretize_step_rad(R_B, 3.0, math.radians(30))) == pytest.approx(25.6, abs=0.3)
    assert discretize_step_rad(R_A, 3.0, math.radians(30)) <= 2 * math.sqrt(2 * 3.0 / R_A)   # 較小角近似保守
    assert discretize_step_rad(R_A, 300.0, math.radians(30)) == pytest.approx(math.radians(30))
    with pytest.raises(ValueError):
        discretize_step_rad(R_A, 0.0, 1.0)


@pytest.mark.parametrize("R,e_max,psi_max_deg", [(R_A, 3.0, 30.0), (R_B, 3.0, 30.0), (R_A, 0.5, 30.0), (R_A, 50.0, 30.0), (R_B, 3.0, 10.0)])
def test_discretize_circumscribed_error_le_emax(R, e_max, psi_max_deg):
    rng = np.random.default_rng(int(R * 10 + e_max * 100 + psi_max_deg))
    psi_max = math.radians(psi_max_deg)
    dpsi = discretize_step_rad(R, e_max, psi_max)
    for _ in range(25):
        a = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500), rng.uniform(-math.pi, math.pi))
        b = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500), rng.uniform(-math.pi, math.pi))
        p = dubins_path(a, b, R)
        pts = discretize(p, e_max, psi_max)
        arr = np.array(pts)
        assert arr.shape[1] == 3
        assert arr[0] == pytest.approx(a.as_array())
        assert arr[-1] == pytest.approx(b.as_array())
        # 航點數 = 1 + Σ(直線 1, 弧 n+1)
        n_expect = 1
        for seg in p.segments:
            if seg.length_m <= 1e-9:
                continue
            n_expect += 1 if seg.kind == "S" else int(math.ceil(abs(seg.dtheta_rad) / dpsi - 1e-12)) + 1
        assert len(pts) == n_expect
        # 依段序走訪航點：直線 1 點（終點）、弧 n 個外切頂點 + 弧終點
        # 頂點：在圓外側、徑向偏差 ≤ e_max、方位 = 下一切點之切線方位；整條折線對細取樣之 Hausdorff 距離 ≤ e_max
        k = 1
        for seg in p.segments:
            if seg.length_m <= 1e-9:
                continue
            if seg.kind == "S":
                assert arr[k] == pytest.approx([seg.end.x_m, seg.end.y_m, seg.end.theta_rad])
                k += 1
                continue
            cx, cy = seg.center
            sign = 1.0 if seg.kind == "R" else -1.0
            n = int(math.ceil(abs(seg.dtheta_rad) / dpsi - 1e-12))
            delta = abs(seg.dtheta_rad) / n
            assert delta <= dpsi + 1e-12
            verts = arr[k:k + n]
            rad = np.hypot(verts[:, 0] - cx, verts[:, 1] - cy)
            np.testing.assert_allclose(rad, R / math.cos(delta / 2), rtol=1e-9)
            dev = rad - R
            assert dev.max() <= e_max + 1e-9 and dev.min() >= -1e-9
            th0 = seg.start.theta_rad
            for j, (vx, vy, vth) in enumerate(verts):
                th_mid = math.atan2(sign * (vy - cy), -sign * (vx - cx))     # 頂點徑向對應之弧方位
                assert _ang_close(th_mid, th0 + sign * (j + 0.5) * delta, 1e-9)
                assert _ang_close(vth, th0 + sign * (j + 1.0) * delta, 1e-9)  # 出發航段方位 = 下一切點切線
                # 頂點在切點 j 與 j+1 之切線交點：至兩切點距離相等 = R·tan(δ/2)
                for jj in (j, j + 1):
                    th_c = th0 + sign * jj * delta
                    px, py = cx - sign * R * math.cos(th_c), cy + sign * R * math.sin(th_c)
                    assert math.hypot(vx - px, vy - py) == pytest.approx(R * math.tan(delta / 2), rel=1e-9)
            k += n
            assert arr[k] == pytest.approx([seg.end.x_m, seg.end.y_m, seg.end.theta_rad])
            k += 1
        assert k == len(pts)
        fine = p.polyline(1.0)
        poly = shapely.LineString(arr[:, :2]) if len(pts) > 1 else shapely.Point(arr[0, :2])
        assert poly.hausdorff_distance(fine) <= e_max + 0.01
        # 折線長 ≥ 路徑長（外切）且 ≤ 路徑長·sec(δ/2)
        assert poly.length >= p.length_m - 1e-6
        assert poly.length <= p.length_m / math.cos(dpsi / 2) + 1e-6
    with pytest.raises(ValueError):
        discretize(p, -1.0, psi_max)


def test_discretize_psi_max_cap():
    a, b = parallel_transfer_pose_pair(np.array([0.0, 0.0]), 0.0, 3 * R_A)
    p = dubins_path(a, b, R_A)                                  # U 型：1/4 圓 + 直線（垂直條帶）+ 1/4 圓
    assert [s.kind for s in p.segments] == ["R", "S", "R"] or [s.kind for s in p.segments] == ["L", "S", "L"]
    assert all(abs(abs(s.dtheta_rad) - math.pi / 2) < 1e-9 for s in p.segments if s.kind != "S")
    pts = discretize(p, 1e6, math.radians(30))                  # e_max 極大 → Δψ = 30°
    n_arc = int(math.ceil((math.pi / 2) / math.radians(30) - 1e-12))  # 3
    assert len(pts) == 1 + (n_arc + 1) + 1 + (n_arc + 1)
    fine = discretize(p, 1e6, math.radians(5))
    assert len(fine) == 1 + (18 + 1) + 1 + (18 + 1)


# ---------------------------------------------------------------------------
# 風中時間（§8.1、D14；§W.5–W.7 數值）
# ---------------------------------------------------------------------------
class _FakeWind:
    def __init__(self, wxy):
        self.wxy = np.asarray(wxy, float)

    def wind_vec(self, z_m):
        z = np.atleast_1d(np.asarray(z_m, float))
        return np.tile(self.wxy, (z.shape[0], 1))


class _FakeEnergy:
    def __init__(self, p_w: float = 511.4, k_unc: float = 1.2075):
        self.p_w, self.k_unc, self.calls = p_w, k_unc, []

    def segment_wh(self, mode, duration_s, gamma_rad=0.0):
        self.calls.append((mode, duration_s, gamma_rad))
        return self.p_w * duration_s * self.k_unc / 3600.0


def test_path_time_calm_equals_length_over_va():
    rng = np.random.default_rng(3)
    for _ in range(10):
        a = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500), rng.uniform(-3, 3))
        b = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500), rng.uniform(-3, 3))
        p = dubins_path(a, b, R_A)
        assert path_time(p, VA_A, None, 0.0) == pytest.approx(p.length_m / VA_A, rel=1e-12)
        assert path_time(p, VA_A, _FakeWind([0.0, 0.0]), 250.0) == pytest.approx(p.length_m / VA_A, rel=1e-12)
        assert path_time(p, VA_A, np.zeros(2), 0.0) == pytest.approx(p.length_m / VA_A, rel=1e-12)


def test_path_time_w5_skip3_uturn():
    """§W.5：A、W=5 西風、skip-3 U 型 732.5 m → 35.5 s（半圓 30.5 s、直線 104.1 m 向南 /20.91 = 5.0 s）。"""
    a = Pose(7068.0, 503.0, math.pi / 2)
    b = Pose(7068.0, -1.2, 3 * math.pi / 2)
    p = dubins_path(a, b, R_A)
    assert p.length_m == pytest.approx(732.5, abs=0.1)
    t = path_time(p, VA_A, _FakeWind(WIND_E5), 260.0)
    assert t == pytest.approx(35.5, abs=0.1)
    # 直線向南：V_g = sqrt(21.5² − 5²) = 20.91
    seg_s = [s for s in p.segments if s.kind == "S"][0]
    assert seg_s.length_m / math.sqrt(VA_A**2 - 25.0) == pytest.approx(104.2 / 20.91, abs=0.02)
    arcs = [s for s in p.segments if s.kind != "S"]
    t_arc = sum(path_time(DubinsPath("RSR", R_A, (s,), s.length_m), VA_A, WIND_E5, 0.0) for s in arcs)
    assert t_arc == pytest.approx(30.5, abs=0.1)
    assert t_arc > sum(s.length_m for s in arcs) / VA_A          # Jensen：L/V_a 低估
    assert path_time(p, VA_A, WIND_E5, 0.0) == pytest.approx(t, rel=1e-12)


def test_path_time_spec_w6_w7_numbers():
    w = _FakeWind(WIND_E5)
    # A 進入（左端）：S_acc ((−606,0),270°) → S^L_1 ((3932,503),90°)：RSR 5168 m／201.8 s
    p = dubins_path(Pose(-606, 0, 3 * math.pi / 2), Pose(3932, 503, math.pi / 2), R_A)
    assert p.length_m == pytest.approx(5168.0, abs=1.0)
    assert path_time(p, VA_A, w, 260.0) == pytest.approx(201.8, abs=0.3)
    # A 進入（右端）：→ S^R_1 ((7068,503),270°)：RSL 8937 m／350.8 s
    p = dubins_path(Pose(-606, 0, 3 * math.pi / 2), Pose(7068, 503, 3 * math.pi / 2), R_A)
    assert p.length_m == pytest.approx(8937.0, abs=1.0)
    assert path_time(p, VA_A, w, 260.0) == pytest.approx(350.8, abs=0.3)
    # A RTB：條帶 3 右端 ((7068,166.9),90°) → S^app ((606,0),270°)：RSL 7094 m／422.3 s
    p = dubins_path(Pose(7068, 166.9, math.pi / 2), Pose(606, 0, 3 * math.pi / 2), R_A)
    assert p.length_m == pytest.approx(7094.0, abs=1.0)
    assert path_time(p, VA_A, w, 260.0) == pytest.approx(422.3, abs=0.3)
    # B 進入：((−489,0),270°) → ((3937,−337.8),90°)：LSL 4804 m／205.4 s
    p = dubins_path(Pose(-489, 0, 3 * math.pi / 2), Pose(3937, -337.8, math.pi / 2), R_B)
    assert p.length_m == pytest.approx(4804.0, abs=1.0)
    assert path_time(p, VA_B, w, 200.0) == pytest.approx(205.4, abs=0.3)
    # B RTB：((7063,−440.6),90°) → ((489,0),270°)：LSL 6954 m／490.7 s
    p = dubins_path(Pose(7063, -440.6, math.pi / 2), Pose(489, 0, 3 * math.pi / 2), R_B)
    assert p.length_m == pytest.approx(6954.0, abs=1.0)
    assert path_time(p, VA_B, w, 200.0) == pytest.approx(490.7, abs=0.3)
    # Ω 1→2（A）：1253.3 m／61.1 s；Ω(2s) 4→2：950.2 m／46.3 s
    a, b = parallel_transfer_pose_pair(np.array([7068.0, 503.0]), math.pi / 2, 168.05)
    p = dubins_path(a, b, R_A)
    assert p.length_m == pytest.approx(1253.3, abs=0.1) and path_time(p, VA_A, w, 0) == pytest.approx(61.1, abs=0.1)
    a, b = parallel_transfer_pose_pair(np.array([3932.0, -1.2]), 3 * math.pi / 2, -2 * 168.05)
    p = dubins_path(a, b, R_A)
    assert p.length_m == pytest.approx(950.2, abs=0.1) and path_time(p, VA_A, w, 0) == pytest.approx(46.3, abs=0.1)


def test_circle_time_factor_and_step_convergence():
    seg = DubinsSegment("R", TWO_PI * R_A, Pose(0, 0, 0), Pose(0, 0, TWO_PI), (R_A, 0.0), TWO_PI)
    full = DubinsPath("RSR", R_A, (seg,), TWO_PI * R_A)
    f = path_time(full, VA_A, WIND_E5, 0.0) / (TWO_PI * R_A / VA_A)
    assert f == pytest.approx(1.043, abs=0.002)
    assert f >= 1.0
    # Riemann 和對 step→0 收斂
    t_coarse = path_time(full, VA_A, WIND_E5, 0.0, step_rad=0.0873)
    t_fine = path_time(full, VA_A, WIND_E5, 0.0, step_rad=1e-3)
    t_finer = path_time(full, VA_A, WIND_E5, 0.0, step_rad=1e-4)
    assert abs(t_coarse - t_fine) < 0.05
    assert abs(t_fine - t_finer) < abs(t_coarse - t_finer) + 1e-12
    # 直線順風／逆風
    p = dubins_path(Pose(0, 0, math.pi / 2), Pose(1000, 0, math.pi / 2), R_A)       # 向東
    assert path_time(p, VA_A, WIND_E5, 0.0) == pytest.approx(1000 / 26.5, rel=1e-12)
    p = dubins_path(Pose(1000, 0, 3 * math.pi / 2), Pose(0, 0, 3 * math.pi / 2), R_A)   # 向西
    assert path_time(p, VA_A, WIND_E5, 0.0) == pytest.approx(1000 / 16.5, rel=1e-12)
    p = dubins_path(Pose(0, 0, 0), Pose(0, 1000, 0), R_A)                            # 向北（側風）
    assert path_time(p, VA_A, WIND_E5, 0.0) == pytest.approx(1000 / math.sqrt(VA_A**2 - 25), rel=1e-12)
    with pytest.raises(ValueError):
        path_time(p, VA_A, WIND_E5, 0.0, step_rad=0.0)


def test_path_time_infeasible_wind_is_inf():
    p = dubins_path(Pose(0, 0, 0), Pose(0, 1000, 0), R_A)
    assert path_time(p, VA_A, np.array([VA_A, 0.0]), 0.0) == math.inf          # |W⊥| = Va
    assert path_time(p, VA_A, np.array([0.0, -VA_A - 1.0]), 0.0) == math.inf  # 逆風超過空速
    p = dubins_path(Pose(0, 0, 0), Pose(R_A, R_A, math.pi / 2), R_A)            # 弧
    assert path_time(p, VA_A, np.array([25.0, 0.0]), 0.0) == math.inf
    t, e = path_time_and_energy(p, VA_A, np.array([25.0, 0.0]), 0.0, _FakeEnergy())
    assert t == math.inf and e == math.inf


def test_path_time_and_energy_duck_typed():
    a, b = parallel_transfer_pose_pair(np.array([7068.0, 503.0]), math.pi / 2, 3 * 168.05)
    p = dubins_path(a, b, R_A)
    en = _FakeEnergy()
    t, e = path_time_and_energy(p, VA_A, _FakeWind(WIND_E5), 260.0, en, gamma_rad=0.02)
    assert t == pytest.approx(path_time(p, VA_A, WIND_E5, 260.0), rel=1e-12)
    assert e == pytest.approx(511.4 * t * 1.2075 / 3600.0, rel=1e-12)
    assert len(en.calls) == 1 and en.calls[0][1] == t and en.calls[0][2] == 0.02
    mode = en.calls[0][0]
    assert getattr(mode, "name", mode) == "CRUISE"


def test_dubins_all_times_batch_matches_path_time():
    rng = np.random.default_rng(21)
    S, E = _rand_pairs(rng, 120, R_A, 1500.0)
    times, lengths, idx = dubins_all_times(S, E, R_A, VA_A, WIND_E5)
    best, idx2 = dubins_all(S, E, R_A)
    np.testing.assert_array_equal(idx, idx2)
    np.testing.assert_allclose(lengths, best, rtol=1e-12)
    for i in range(S.shape[0]):
        p = dubins_path(Pose(*S[i]), Pose(*E[i]), R_A)
        assert times[i] == pytest.approx(path_time(p, VA_A, WIND_E5, 0.0), rel=1e-9)
    # 不可飛風 → inf
    times, _, _ = dubins_all_times(S[:5], E[:5], R_A, VA_A, np.array([30.0, 0.0]))
    assert np.all(np.isinf(times))
    # 空輸入
    t0, l0, i0 = dubins_all_times(np.zeros((0, 3)), np.zeros((0, 3)), R_A, VA_A, WIND_E5)
    assert t0.shape == (0,) and l0.shape == (0,) and i0.shape == (0,)


def test_dubins_all_vectorized_large():
    rng = np.random.default_rng(99)
    S, E = _rand_pairs(rng, 20000, R_B, 5000.0)
    best, idx = dubins_all(S, E, R_B)
    assert best.shape == (20000,) and np.all(np.isfinite(best)) and idx.dtype == np.int8
    assert np.all(best >= np.hypot(E[:, 0] - S[:, 0], E[:, 1] - S[:, 1]) - 1e-9)   # 下界：直線距離
    assert set(np.unique(idx)) <= set(range(6))
