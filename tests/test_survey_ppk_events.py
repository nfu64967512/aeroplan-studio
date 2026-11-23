"""survey/ppk/events.py —— TIM-TM2 → GPST、遺失邊緣、兩道數值鎖、與 Pi 邊緣的逐一配對。

合成情境（_scenario）完全以整數奈秒產生真值：
- 4 條帶 × 40 張，帶內依距離觸發（d=18.85 m、地速 18.5–23.5 m/s），觸發時刻落在 ArduPilot
  50 Hz 迴圈的 20 ms 格點上；帶間轉彎 40–70 s。
- F9P：10 Hz 量測 epoch，每個 epoch 依「只報最後一個上升／下降邊緣」產生 TIM-TM2；count 從 65530
  起算（測 65536 繞回），session 前另有 5 個地面試拍邊緣（Pi 尚未開機）。
- 第 2 條帶插入一個與下一張相距 30 ms 的額外邊緣 ⇒ 同一 epoch 兩個上升邊緣，TM2 只報後者（遺失 1）。
- Pi 在第 3 條帶漏掉一個邊緣 ⇒ seq 連續但 count−seq 自此加 1。
- Pi 時鐘：頻差 +37 ppm、任意偏移、20 µs 抖動。
"""
import json

import numpy as np
import pytest

from survey.ppk.events import (GPS_UTC_LEAP_S, PairingError, extract_edges, pair_with_frames,
                               read_frames_jsonl, tm2_time_gpst)
from survey.ppk.ubx import decode_tim_tm2, parse_ubx
from tests.test_survey_ppk_ubx import _TM2_HAND_HEX, pvt_frame, rawx_frame, tm2_frame

W_NS = 604800 * 10**9
WEEK = 2389
T0_NS = (WEEK * 604800 + 180000) * 10**9
EPOCH_NS = 100_000_000
PULSE_NS = 100_000_000


def _fields(t_ns: int):
    wn, rem = divmod(t_ns, W_NS)
    ms, sub = divmod(rem, 1_000_000)
    return wn, ms, sub


def _scenario(seed=7, *, utc=False, c0=65530, pi_rate_ppm=37.0, jitter_ns=20_000,
              extra_edge=True, pi_miss=True, invalid_every=0, base_override=None):
    rng = np.random.default_rng(seed)
    # ---- 實體上升邊緣（整數 ns）----
    edges = [T0_NS - 300 * 10**9 + k * 2 * 10**9 for k in range(5)]      # 地面試拍
    n_pre = len(edges)
    t = 5.0
    phase = 0.0137
    strip_of = []
    for s in range(4):
        v = rng.uniform(18.5, 23.5)
        for _ in range(40):
            ti = t + 18.85 / (v + rng.normal(0, 0.3))
            t = phase + np.ceil((ti - phase) / 0.02) * 0.02
            edges.append(T0_NS + int(round(t * 1e9)))
            strip_of.append(s)
        t += rng.uniform(40, 70)
    extra_idx = None
    if extra_edge:
        # 第 2 條帶裡找一個離上一個 epoch 邊界 ≥35 ms 的邊緣，在它前 30 ms 插一個額外邊緣
        for i in range(n_pre + 45, n_pre + 80):
            if (edges[i] - T0_NS) % EPOCH_NS >= 35_000_000:
                edges.insert(i, edges[i] - 30_000_000)
                strip_of.insert(i - n_pre, 1)
                extra_idx = i
                break
        assert extra_idx is not None
    edges = np.array(edges, dtype=np.int64)
    falls = edges + PULSE_NS
    counts = c0 + np.arange(1, edges.size + 1)       # 第 i 個上升邊緣後的累計數（未繞回）

    # ---- F9P 的 TM2／PVT／RAWX 串流 ----
    first = int((edges[0] - T0_NS) // EPOCH_NS) - 3
    last = int((falls[-1] - T0_NS) // EPOCH_NS) + 3
    stream = []
    last_r = last_f = None
    n_msg = 0
    reported = set()
    for n in range(first, last + 1):
        tau = T0_NS + n * EPOCH_NS
        rmask = (edges > tau - EPOCH_NS) & (edges <= tau)
        fmask = (falls > tau - EPOCH_NS) & (falls <= tau)
        wn_e, ms_e, _ = _fields(tau)
        pvt = pvt_frame(itow_ms=ms_e)
        msg = b""
        if rmask.any() or fmask.any():
            if rmask.any():
                ri = int(np.nonzero(rmask)[0][-1])
                last_r = ri
                reported.add(ri)
            if fmask.any():
                last_f = int(np.nonzero(fmask)[0][-1])
            cnt = int(c0 + np.count_nonzero(edges <= tau))
            tb = 2 if utc else 1
            if base_override is not None:
                tb = base_override
            flags = 0x01 | 0x40 | (tb << 3) | (0x20 if utc else 0)
            flags |= (0x80 if rmask.any() else 0) | (0x04 if fmask.any() else 0)
            n_msg += 1
            if invalid_every and n_msg % invalid_every == 0:
                flags &= ~0x40
            shift = GPS_UTC_LEAP_S * 10**9 if utc else 0
            wr, msr, sr = _fields(int(edges[last_r]) - shift) if last_r is not None else (0, 0, 0)
            wf, msf, sf = _fields(int(falls[last_f]) - shift) if last_f is not None else (0, 0, 0)
            msg = tm2_frame(ch=0, flags=flags, count=cnt, wn_r=wr, tow_ms_r=msr, sub_r=sr,
                            wn_f=wf, tow_ms_f=msf, sub_f=sf, acc=25)
        raw = rawx_frame(ms_e / 1000.0, wn_e, leap=18, nmeas=1)
        stream.append(raw + (pvt + msg if n % 2 == 0 else msg + pvt))
    data = b"".join(stream)

    # ---- Pi frames ----
    seen = list(range(n_pre, edges.size))
    missed = None
    if pi_miss:
        missed = n_pre + 130
        seen.remove(missed)
    pi_off = 123_456_789_000
    frames, truth = [], {}
    for s, i in enumerate(seen, start=1):
        e = pi_off + int(round((edges[i] - T0_NS) * (1 + pi_rate_ppm * 1e-6))) + \
            int(rng.normal(0, jitter_ns))
        frames.append({"seq": s, "edge_ns": e, "file": f"images/{s:06d}.jpg", "status": "ok"})
        truth[s] = i
    return dict(data=data, edges=edges, falls=falls, counts=counts, n_pre=n_pre,
                extra_idx=extra_idx, missed=missed, frames=frames, truth=truth,
                reported=reported, strip_of=strip_of)


@pytest.fixture(scope="module")
def sc():
    return _scenario()


# ---------------------------------------------------------------------------
# 時間換算
# ---------------------------------------------------------------------------
def test_tm2_time_hand_example():
    m = decode_tim_tm2(bytes.fromhex(_TM2_HAND_HEX))
    # 2389·604800 + 183245.678 + 123456e-9
    assert tm2_time_gpst(m) == pytest.approx(1445050445.678123456, abs=1e-6)
    assert tm2_time_gpst(m, "falling") == pytest.approx(1445050445.778654321, abs=1e-6)
    with pytest.raises(ValueError, match="只支援 GPS"):
        tm2_time_gpst(m, gnss_time_base_system="GAL")
    with pytest.raises(ValueError):
        tm2_time_gpst(m, "middle")


def test_tm2_time_utc_base_adds_leap_seconds():
    m = decode_tim_tm2(bytes.fromhex(_TM2_HAND_HEX))
    m_utc = type(m)(**{**m.__dict__, "flags": (m.flags & ~0x18) | 0x10 | 0x20})
    assert m_utc.time_base == 2
    assert tm2_time_gpst(m_utc) - tm2_time_gpst(m) == pytest.approx(18.0, abs=1e-6)
    assert tm2_time_gpst(m_utc, leap_s=17) - tm2_time_gpst(m) == pytest.approx(17.0, abs=1e-6)
    m_rcv = type(m)(**{**m.__dict__, "flags": m.flags & ~0x18})
    with pytest.raises(ValueError, match="timeBase=0"):
        tm2_time_gpst(m_rcv)


# ---------------------------------------------------------------------------
# 邊緣整理
# ---------------------------------------------------------------------------
def test_extract_edges_values_lost_and_wrap(sc):
    ex = extract_edges(parse_ubx(sc["data"]))
    edges, counts = sc["edges"], sc["counts"]
    exp = sorted(sc["reported"])
    assert [e.count for e in ex.edges] == [int(counts[i]) for i in exp]
    assert max(e.count for e in ex.edges) > 65535          # 繞回已解開
    for e, i in zip(ex.edges, exp):
        assert e.t_gpst_s == pytest.approx(edges[i] / 1e9, abs=1e-6)
        assert e.time_base == 1 and e.acc_est_ns == 25 and e.flags == ()
    # 額外邊緣與下一張同 epoch ⇒ 只遺失它一個
    xi = sc["extra_idx"]
    assert [(lo.count_before, lo.count_after, lo.n_lost) for lo in ex.lost] == \
        [(int(counts[xi - 1]), int(counts[xi + 1]), 1)]
    assert ex.lost[0].t_before_gpst_s == pytest.approx(edges[xi - 1] / 1e9, abs=1e-6)
    assert ex.n_lost == 1
    assert ex.invalid == [] and ex.duplicates == [] and ex.other_channel == 0
    assert ex.pvt_checked and ex.pvt_tol_s == pytest.approx(0.25)
    assert ex.rawx_leap_s == 18


def test_extract_falling_edges(sc):
    ex = extract_edges(parse_ubx(sc["data"]), edge="falling")
    falls = sc["falls"]
    # 每個 epoch 只報最後一個下降邊緣 ⇒ 額外邊緣的下降邊緣同樣遺失
    got = [e.t_gpst_s for e in ex.edges]
    assert len(got) == len(falls) - 1
    want = np.delete(falls, sc["extra_idx"]) / 1e9
    assert np.max(np.abs(np.array(got) - want)) < 1e-6
    assert ex.n_lost == 1


def test_utc_base_scenario_matches_gpst_truth():
    s = _scenario(utc=True, extra_edge=False, pi_miss=False)
    ex = extract_edges(parse_ubx(s["data"]))
    assert len(ex.edges) == len(s["edges"])
    got = np.array([e.t_gpst_s for e in ex.edges])
    assert np.max(np.abs(got - s["edges"] / 1e9)) < 1e-6
    assert all(e.flags == ("tm2_utc_base",) for e in ex.edges)


def test_wrong_leap_caught_by_rawx_then_by_pvt():
    s = _scenario(utc=True, extra_edge=False, pi_miss=False)
    log = parse_ubx(s["data"])
    with pytest.raises(ValueError, match="閏秒"):
        extract_edges(log, leap_s=17)
    ex = extract_edges(log, leap_s=17, rawx=[])          # 沒有 RAWX ⇒ 第二道鎖 NAV-PVT
    assert ex.edges == []
    assert len(ex.invalid) == len(s["edges"])
    assert all(r.startswith("pvt_mismatch(-") for _, r in ex.invalid)
    assert ex.rawx_leap_s is None
    # GPS 基準的 TM2 不受閏秒影響 ⇒ 不擋，但照樣回報接收機的閏秒
    g = _scenario(extra_edge=False, pi_miss=False)
    exg = extract_edges(parse_ubx(g["data"]), leap_s=17)
    assert len(exg.edges) == len(g["edges"]) and exg.invalid == [] and exg.rawx_leap_s == 18


def test_gnss_base_other_system_rejected(sc):
    with pytest.raises(ValueError, match="只支援 GPS"):
        extract_edges(parse_ubx(sc["data"]), gnss_time_base_system="BDS")


def test_invalid_flags_are_listed_with_reasons():
    s = _scenario(extra_edge=False, pi_miss=False, invalid_every=7)
    ex = extract_edges(parse_ubx(s["data"]))
    reasons = {r for _, r in ex.invalid}
    assert reasons == {"time_not_valid"}
    assert len(ex.edges) + len(ex.invalid) == len(s["edges"])
    assert len(ex.invalid) >= 20
    s0 = _scenario(extra_edge=False, pi_miss=False, base_override=0)
    ex0 = extract_edges(parse_ubx(s0["data"]))
    assert ex0.edges == [] and {r for _, r in ex0.invalid} == {"time_base_0"}
    su = _scenario(extra_edge=False, pi_miss=False, base_override=2)   # UTC 基準但 utc 位元為 0
    exu = extract_edges(parse_ubx(su["data"]))
    assert exu.edges == [] and {r for _, r in exu.invalid} == {"utc_not_available"}


def test_duplicates_and_other_channel():
    base = dict(flags=0xC9, wn_r=WEEK, tow_ms_r=180000100, acc=5)
    msgs = [decode_tim_tm2(tm2_frame(count=10, **base)[6:-2], offset=0),
            decode_tim_tm2(tm2_frame(count=10, **base)[6:-2], offset=40),
            decode_tim_tm2(tm2_frame(ch=1, count=11, **base)[6:-2], offset=80),
            decode_tim_tm2(tm2_frame(count=13, **{**base, "tow_ms_r": 180001100})[6:-2], offset=120)]
    ex = extract_edges(msgs)
    assert [e.count for e in ex.edges] == [10, 13]
    assert len(ex.duplicates) == 1 and ex.other_channel == 1
    assert [(x.count_before, x.count_after, x.n_lost) for x in ex.lost] == [(10, 13, 2)]
    assert not ex.pvt_checked


# ---------------------------------------------------------------------------
# 配對
# ---------------------------------------------------------------------------
def test_pairing_recovers_truth_not_order(sc):
    ex = extract_edges(parse_ubx(sc["data"]))
    offs = [0.040 + 0.001 * (f["seq"] % 5) for f in sc["frames"]]
    res = pair_with_frames(ex, sc["frames"], exposure_offset_s=offs)
    counts, edges, truth = sc["counts"], sc["edges"], sc["truth"]
    xi = sc["extra_idx"]
    assert len(res.events) == len(sc["frames"])
    by_seq = {e.seq: e for e in res.events}
    for seq, i in truth.items():
        ev = by_seq[seq]
        if i == xi:
            assert ev.tm2_count is None
            assert ev.flags[:1] == ("tm2_missing",)
            assert ev.t_edge_gpst_s == pytest.approx(edges[i] / 1e9, abs=2e-4)
        else:
            assert ev.tm2_count == int(counts[i])
            assert ev.t_edge_gpst_s == pytest.approx(edges[i] / 1e9, abs=1e-6)
            assert abs(ev.residual_s) < 1e-4
        assert ev.file == f"images/{seq:06d}.jpg" and ev.status == "ok"
    # 曝光時刻 = 邊緣 + a·offset
    for ev, o in zip(res.events, offs):
        assert ev.t_exposure_gpst_s == pytest.approx(ev.t_edge_gpst_s + res.fit.slope * o, abs=5e-7)
    # 未配對 TM2：5 個地面試拍（Pi 尚未開機）＋ Pi 漏掉的那一個；未配對 Pi：TM2 遺失的那一個
    want_tm2 = [int(c) for c in counts[:sc["n_pre"]]] + [int(counts[sc["missed"]])]
    assert [e.count for e in res.unmatched_tm2] == want_tm2
    xseq = next(s for s, i in truth.items() if i == xi)
    assert res.unmatched_pi_seq == [xseq]
    assert res.n_paired == len(sc["frames"]) - 1
    assert res.lost == ex.lost
    # Pi 漏掉一個邊緣 ⇒ 兩段 count−seq
    mseq = next(s for s, i in truth.items() if i > sc["missed"])
    k1 = int(counts[truth[1]]) - 1
    assert res.counter_shifts == [(1, k1), (mseq, k1 + 1)]
    seg_a = [s for s in truth if s < mseq and s != xseq]
    seg_b = [s for s in truth if s >= mseq]
    shorter = seg_a if len(seg_a) < len(seg_b) else seg_b
    assert {e.seq for e in res.events if "counter_shift" in e.flags} == set(shorter)
    assert res.fit.counter_offset == (k1 if shorter is seg_b else k1 + 1)
    # 時鐘擬合：Pi 快 37 ppm ⇒ GPST 對 Pi 的斜率 ≈ 1 − 37e-6；殘差 ≈ 20 µs 抖動
    assert res.fit.rate_ppm == pytest.approx(-37.0, abs=0.5)
    assert 10e-6 < res.fit.rms_s < 30e-6
    assert res.fit.n_pairs == len(sc["frames"]) - 1
    # 依順序配對在這個情境下會錯：證明測試有鑑別力
    session = [e for e in extract_edges(parse_ubx(sc["data"])).edges][sc["n_pre"]:]
    naive_wrong = sum(1 for k, f in enumerate(sc["frames"][:len(session)])
                      if session[k].count != int(counts[truth[f["seq"]]]))
    assert naive_wrong > 50


def test_pairing_offsets_none_and_missing():
    s = _scenario(seed=3, extra_edge=False, pi_miss=False)
    ex = extract_edges(parse_ubx(s["data"]))
    res = pair_with_frames(ex, s["frames"])
    assert all("edge_time_only" in e.flags for e in res.events)
    assert all(e.t_exposure_gpst_s == e.t_edge_gpst_s for e in res.events)
    offs = [None if f["seq"] == 3 else float("nan") if f["seq"] == 4 else 0.05 for f in s["frames"]]
    res2 = pair_with_frames(ex, s["frames"], exposure_offset_s=offs)
    ev = {e.seq: e for e in res2.events}
    assert ev[3].t_exposure_gpst_s is None and "no_exposure_offset" in ev[3].flags
    assert ev[4].t_exposure_gpst_s is None and "no_exposure_offset" in ev[4].flags
    # GPST 1.4e9 s 的 float64 解析度約 2.4e-7 s；offset 是 Pi 時鐘秒 ⇒ 乘擬合斜率（37 ppm ⇒ 差 1.85 µs）
    assert ev[5].t_exposure_gpst_s == pytest.approx(ev[5].t_edge_gpst_s + 0.05 * res2.fit.slope, abs=5e-7)
    assert abs(ev[5].t_exposure_gpst_s - (ev[5].t_edge_gpst_s + 0.05)) > 1e-6
    assert [e.count for e in res2.unmatched_tm2] == [int(c) for c in s["counts"][:s["n_pre"]]]
    assert res2.unmatched_pi_seq == []
    assert res2.counter_shifts == [(1, int(s["counts"][s["truth"][1]]) - 1)]
    with pytest.raises(ValueError, match="長度"):
        pair_with_frames(ex, s["frames"], exposure_offset_s=[0.0])


def test_pairing_frames_order_independent():
    s = _scenario(seed=4, extra_edge=False, pi_miss=False)
    ex = extract_edges(parse_ubx(s["data"]))
    fr = list(reversed(s["frames"]))
    offs = [0.01 * f["seq"] for f in fr]
    res = pair_with_frames(ex, fr, exposure_offset_s=offs)
    assert [e.seq for e in res.events] == sorted(f["seq"] for f in fr)
    for e in res.events:
        assert e.t_exposure_gpst_s - e.t_edge_gpst_s == pytest.approx(0.01 * e.seq * res.fit.slope, abs=5e-7)


def test_pairing_fit_extrapolated_flag():
    s = _scenario(seed=5, extra_edge=False, pi_miss=False)
    ex = extract_edges(parse_ubx(s["data"]))
    edges = ex.edges[:-3]                     # 最後三個 TM2 不給 ⇒ 最後三張 Pi 只能外插
    res = pair_with_frames(edges, s["frames"])
    tail = res.events[-3:]
    assert all(e.tm2_count is None and "fit_extrapolated" in e.flags for e in tail)
    for e in tail:
        assert e.t_edge_gpst_s == pytest.approx(s["edges"][s["truth"][e.seq]] / 1e9, abs=2e-4)
    assert res.unmatched_pi_seq == [e.seq for e in tail]


def test_pairing_rejects_unrelated_pi_edges():
    s = _scenario(seed=6, extra_edge=False, pi_miss=False)
    ex = extract_edges(parse_ubx(s["data"]))
    rng = np.random.default_rng(0)
    fake = np.cumsum(rng.uniform(0.3e9, 2.5e9, len(s["frames"]))).astype(np.int64)
    frames = [{"seq": k + 1, "edge_ns": int(e)} for k, e in enumerate(fake)]
    with pytest.raises(PairingError):
        pair_with_frames(ex, frames)


def test_pairing_refuses_perfectly_periodic_triggers():
    """觸發間隔完全相同時錯位一張也對得上 ⇒ 等於按順序配對，必須拒絕。"""
    from survey.ppk.events import Tm2Edge
    t0 = WEEK * 604800 + 180000.0
    tm2 = [Tm2Edge(100 + k, t0 + k * 1.0, 20, 1, k) for k in range(30)]
    frames = [{"seq": k + 1, "edge_ns": 5_000_000_000 + k * 1_000_000_000} for k in range(29)]
    with pytest.raises(PairingError, match="歧義"):
        pair_with_frames(tm2, frames)


def test_pairing_input_validation():
    from survey.ppk.events import Tm2Edge
    tm2 = [Tm2Edge(1 + k, 1e9 + k, 0, 1, k) for k in range(5)]
    with pytest.raises(ValueError, match="seq"):
        pair_with_frames(tm2, [{"seq": 1, "edge_ns": 1}, {"seq": 1, "edge_ns": 2}])
    with pytest.raises(ValueError, match="edge_ns"):
        pair_with_frames(tm2, [{"seq": 1, "edge_ns": 5}, {"seq": 2, "edge_ns": 5}])
    with pytest.raises(PairingError, match="太少"):
        pair_with_frames(tm2[:2], [{"seq": 1, "edge_ns": 5}, {"seq": 2, "edge_ns": 6},
                                   {"seq": 3, "edge_ns": 7}])
    with pytest.raises(ValueError, match="pair_tol_s"):
        pair_with_frames(tm2, [], pair_tol_s=0)


def test_read_frames_jsonl_tolerates_only_truncated_last_line(tmp_path):
    p = tmp_path / "frames.jsonl"
    rows = [{"seq": 1, "edge_ns": 10, "status": "ok"}, {"seq": 2, "edge_ns": 20, "status": "ok"}]
    p.write_text("\n".join(json.dumps(r) for r in rows) + '\n{"seq": 3, "edge_', encoding="utf-8")
    got, bad = read_frames_jsonl(p)
    assert got == rows and bad == [3]
    p.write_text('{"seq": 1}\n{oops\n{"seq": 3}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="第 2 行"):
        read_frames_jsonl(p)
