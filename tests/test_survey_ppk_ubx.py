"""survey/ppk/ubx.py —— UBX 切框、檢查碼、TIM-TM2／NAV-PVT／RAWX 解碼。

測試用的編碼器（_ref_ck、frame、各 payload）**刻意不呼叫被測模組**：檢查碼用規格 §3.4 的
逐位元組迴圈另寫一份，payload 用逐欄 int.to_bytes 依規格表的位移排出（不共用 struct 格式字串），
這樣被測的 struct 格式或位移寫錯時兩邊不會一起錯。
"""
import random

import numpy as np
import pytest

from survey.ppk import ubx
from survey.ppk.ubx import (UBX_NAV_PVT, UBX_RXM_RAWX, UBX_RXM_SFRBX, UBX_TIM_TM2, UbxStreamParser,
                            build_frame, decode_nav_pvt, decode_tim_tm2, parse_ubx, read_ubx_file,
                            ubx_checksum)


# ---------------------------------------------------------------------------
# 獨立編碼器（其他 PPK 測試也匯入這些）
# ---------------------------------------------------------------------------
def _ref_ck(body: bytes) -> bytes:
    a = b = 0
    for x in body:
        a = (a + x) % 256
        b = (b + a) % 256
    return bytes([a, b])


def frame(cls: int, mid: int, payload: bytes) -> bytes:
    body = bytes([cls, mid]) + len(payload).to_bytes(2, "little") + payload
    return b"\xb5\x62" + body + _ref_ck(body)


def _le(fields) -> bytes:
    """[(value, nbytes, signed)] 依序排成小端序。"""
    out = b""
    for f in fields:
        v, n = f[0], f[1]
        signed = f[2] if len(f) > 2 else False
        out += int(v).to_bytes(n, "little", signed=signed)
    return out


def tm2_payload(*, ch=0, flags=0xC8, count=0, wn_r=0, tow_ms_r=0, sub_r=0, wn_f=0, tow_ms_f=0,
                sub_f=0, acc=10) -> bytes:
    # §3.19.1：ch U1、flags X1、count U2、wnR U2、wnF U2、towMsR U4、towSubMsR U4、towMsF U4、towSubMsF U4、accEst U4
    return _le([(ch, 1), (flags, 1), (count % 65536, 2), (wn_r, 2), (wn_f, 2), (tow_ms_r, 4),
                (sub_r, 4), (tow_ms_f, 4), (sub_f, 4), (acc, 4)])


def tm2_frame(**kw) -> bytes:
    return frame(0x0D, 0x03, tm2_payload(**kw))


def pvt_payload(*, itow_ms=0, fix_type=3, flags=0x83, num_sv=24, lat_deg=25.1, lon_deg=121.2,
                height_mm=50000, hmsl_mm=31460, h_acc_mm=14, v_acc_mm=21, vel=(0, 0, 0),
                g_speed=0, head_mot=0, s_acc=50, head_acc=100000, p_dop=123, flags3=0) -> bytes:
    # §3.15.13 位移 0..91
    p = _le([(itow_ms, 4), (2026, 2), (10, 1), (20, 1), (2, 1), (0, 1), (0, 1), (0x07, 1),
             (30, 4), (-12345, 4, True), (fix_type, 1), (flags, 1), (0xE0, 1), (num_sv, 1),
             (round(lon_deg * 1e7), 4, True), (round(lat_deg * 1e7), 4, True),
             (height_mm, 4, True), (hmsl_mm, 4, True), (h_acc_mm, 4), (v_acc_mm, 4),
             (vel[0], 4, True), (vel[1], 4, True), (vel[2], 4, True), (g_speed, 4, True),
             (head_mot, 4, True), (s_acc, 4), (head_acc, 4), (p_dop, 2), (flags3, 2),
             (0, 4), (0, 4, True), (0, 2, True), (0, 2)])
    assert len(p) == 92
    return p


def pvt_frame(**kw) -> bytes:
    return frame(0x01, 0x07, pvt_payload(**kw))


def rawx_frame(rcv_tow: float, week: int, leap: int = 18, nmeas: int = 3, rec_stat: int = 1) -> bytes:
    import struct
    head = struct.pack("<d", rcv_tow) + _le([(week, 2), (leap, 1, True), (nmeas, 1), (rec_stat, 1),
                                             (1, 1), (0, 2)])
    return frame(0x02, 0x15, head + bytes(range(32)) * nmeas)


def sfrbx_frame(nwords: int = 10) -> bytes:
    return frame(0x02, 0x13, _le([(0, 1), (5, 1), (0, 1), (0, 1), (nwords, 1), (3, 1), (2, 1), (0, 1)])
                 + bytes(4 * nwords))


# ---------------------------------------------------------------------------
# 檢查碼
# ---------------------------------------------------------------------------
def test_known_poll_frames_independent_vectors():
    """公開流傳的輪詢框（UBX-CFG-RATE poll、UBX-MON-VER poll），位元組逐一相同。"""
    assert build_frame(0x06, 0x08, b"") == bytes.fromhex("b5 62 06 08 00 00 0e 30")
    assert build_frame(0x0A, 0x04, b"") == bytes.fromhex("b5 62 0a 04 00 00 0e 34")
    log_frames = UbxStreamParser().feed(bytes.fromhex("b5 62 0a 04 00 00 0e 34"))
    assert [(f.cls, f.msg_id, f.payload) for f in log_frames] == [(0x0A, 0x04, b"")]


@pytest.mark.parametrize("n", [0, 1, 2, 63, 64, 65, 66, 1000, 5000])
def test_checksum_matches_reference_loop(n):
    rng = random.Random(n)
    for _ in range(5):
        body = bytes(rng.randrange(256) for _ in range(n))
        assert ubx_checksum(body) == _ref_ck(body)


# ---------------------------------------------------------------------------
# TIM-TM2
# ---------------------------------------------------------------------------
# 手算範例：ch=0、flags=0xCD（newRisingEdge、time valid、timeBase=GNSS、newFallingEdge、mode=running）、
# count=513、wnR=wnF=2389、towMsR=183245678（0x0AEC1B6E）、towSubMsR=123456（0x01E240）、
# towMsF=183245778、towSubMsF=654321（0x09FBF1）、accEst=21。逐位元組小端序：
_TM2_HAND_HEX = ("00 cd 01 02 55 09 55 09 6e 1b ec 0a 40 e2 01 00 "
                 "d2 1b ec 0a f1 fb 09 00 15 00 00 00")


def test_tm2_hand_example_decodes_every_field():
    p = bytes.fromhex(_TM2_HAND_HEX)
    assert p == tm2_payload(flags=0xCD, count=513, wn_r=2389, tow_ms_r=183245678, sub_r=123456,
                            wn_f=2389, tow_ms_f=183245778, sub_f=654321, acc=21)
    m = decode_tim_tm2(p, offset=7)
    assert (m.ch, m.count, m.wn_r, m.wn_f) == (0, 513, 2389, 2389)
    assert (m.tow_ms_r, m.tow_sub_ms_r_ns) == (183245678, 123456)
    assert (m.tow_ms_f, m.tow_sub_ms_f_ns) == (183245778, 654321)
    assert m.acc_est_ns == 21 and m.offset == 7
    assert m.new_rising_edge and m.time_valid and m.new_falling_edge and m.mode_running
    assert not m.stopped and not m.utc_available
    assert m.time_base == 1
    # 完整框的檢查碼（手算 0x47 0xAB）
    assert frame(0x0D, 0x03, p)[-2:] == bytes([0x47, 0xAB])


@pytest.mark.parametrize("flags, expect", [
    (0x01, dict(mode_running=True)),
    (0x02, dict(stopped=True)),
    (0x04, dict(new_falling_edge=True)),
    (0x08, dict(time_base=1)),
    (0x10, dict(time_base=2)),
    (0x18, dict(time_base=3)),
    (0x20, dict(utc_available=True)),
    (0x40, dict(time_valid=True)),
    (0x80, dict(new_rising_edge=True)),
])
def test_tm2_flag_bits_one_at_a_time(flags, expect):
    m = decode_tim_tm2(tm2_payload(flags=flags))
    base = dict(mode_running=False, stopped=False, new_falling_edge=False, time_base=0,
                utc_available=False, time_valid=False, new_rising_edge=False)
    base.update(expect)
    assert {k: getattr(m, k) for k in base} == base


def test_tm2_wrong_length_rejected():
    with pytest.raises(ValueError):
        decode_tim_tm2(tm2_payload()[:27])


# ---------------------------------------------------------------------------
# NAV-PVT
# ---------------------------------------------------------------------------
def test_nav_pvt_scaling_and_offsets():
    p = pvt_payload(itow_ms=180000100, fix_type=3, flags=0x83, num_sv=27, lat_deg=25.1234567,
                    lon_deg=121.7654321, height_mm=50123, hmsl_mm=31583, h_acc_mm=14, v_acc_mm=22,
                    vel=(19500, -2250, 310), g_speed=19629, head_mot=35344567, s_acc=60,
                    head_acc=123456, p_dop=137, flags3=1)
    m = decode_nav_pvt(p, offset=99)
    assert m.itow_ms == 180000100 and m.offset == 99
    assert (m.year, m.month, m.day, m.hour, m.minute, m.second, m.valid) == (2026, 10, 20, 2, 0, 0, 7)
    assert m.t_acc_ns == 30 and m.nano_ns == -12345
    assert m.fix_type == 3 and m.num_sv == 27 and m.flags2 == 0xE0
    assert m.gnss_fix_ok and m.diff_soln and m.carr_soln == 2 and m.invalid_llh
    assert m.lat_deg == pytest.approx(25.1234567, abs=1e-12)
    assert m.lon_deg == pytest.approx(121.7654321, abs=1e-12)
    assert m.height_m == pytest.approx(50.123) and m.h_msl_m == pytest.approx(31.583)
    assert m.h_acc_m == pytest.approx(0.014) and m.v_acc_m == pytest.approx(0.022)
    assert (m.vel_n_mps, m.vel_e_mps, m.vel_d_mps) == pytest.approx((19.5, -2.25, 0.31))
    assert m.g_speed_mps == pytest.approx(19.629)
    assert m.head_mot_deg == pytest.approx(353.44567)
    assert m.s_acc_mps == pytest.approx(0.06) and m.head_acc_deg == pytest.approx(1.23456)
    assert m.p_dop == pytest.approx(1.37)


def test_nav_pvt_carr_soln_float():
    assert decode_nav_pvt(pvt_payload(flags=0x41)).carr_soln == 1
    assert decode_nav_pvt(pvt_payload(flags=0x01)).carr_soln == 0


# ---------------------------------------------------------------------------
# 串流：垃圾、NMEA、假長度、壞檢查碼、截斷尾巴
# ---------------------------------------------------------------------------
def _messy_stream():
    good = [
        tm2_frame(flags=0xC8, count=1, wn_r=2389, tow_ms_r=180000123),
        rawx_frame(180000.0, 2389),
        sfrbx_frame(10),
        pvt_frame(itow_ms=180000100),
        tm2_frame(flags=0xC8, count=2, wn_r=2389, tow_ms_r=180001123),
        rawx_frame(180000.1, 2389),
    ]
    bad_ck = bytearray(tm2_frame(flags=0xC8, count=99))
    bad_ck[-1] ^= 0xFF
    nmea = b"$GNGGA,020000.00,2506.00000,N,12112.00000,E,4,12,0.8,31.5,M,18.5,M,1.0,0000*5B\r\n"
    bogus = b"\xb5\x62\x0d\x03\xff\xff"            # 假長度 65535，後面的真框不可被吞
    tail = pvt_frame(itow_ms=1)[:20]
    parts = [nmea, bogus, good[0], b"\x00\xb5\x13garbage", good[1], good[2], bytes(bad_ck),
             nmea, good[3], b"\xb5", good[4], good[5], tail]
    return b"".join(parts), good, tail


def test_messy_stream_recovers_every_good_frame():
    data, good, tail = _messy_stream()
    log = parse_ubx(data)
    assert [m.count for m in log.tm2] == [1, 2]
    assert [m.tow_ms_r for m in log.tm2] == [180000123, 180001123]
    assert len(log.pvt) == 1 and log.pvt[0].itow_ms == 180000100
    assert log.rawx_count == 2 and log.sfrbx_count == 1
    st = log.stats
    assert st.frames == 6
    assert st.count(*UBX_TIM_TM2) == 2 and st.count(*UBX_NAV_PVT) == 1
    assert st.count(*UBX_RXM_RAWX) == 2 and st.count(*UBX_RXM_SFRBX) == 1
    assert st.truncated_tail_bytes == len(tail)
    assert st.checksum_errors >= 1
    # 每個位元組都被歸類：有效框、垃圾、截斷尾巴
    assert st.garbage_bytes == len(data) - sum(map(len, good)) - len(tail)
    assert st.decode_errors == 0
    # offset 是同步字元在串流中的位置
    for m, g in zip(log.tm2, (good[0], good[4])):
        assert data[m.offset:m.offset + len(g)] == g


@pytest.mark.parametrize("chunk", [1, 7, 36, 64, 1000])
def test_chunked_feed_equals_whole_buffer(chunk):
    data, _, _ = _messy_stream()
    whole = parse_ubx(data)
    part = parse_ubx(data, chunk_size=chunk)
    assert [(m.count, m.offset) for m in part.tm2] == [(m.count, m.offset) for m in whole.tm2]
    assert [p.offset for p in part.pvt] == [p.offset for p in whole.pvt]
    assert (part.stats.frames, part.stats.garbage_bytes, part.stats.truncated_tail_bytes) == \
        (whole.stats.frames, whole.stats.garbage_bytes, whole.stats.truncated_tail_bytes)


@pytest.mark.parametrize("tail, n_tail", [(b"\xb5", 1), (b"\xb5\x62\x0d", 3),
                                          (tm2_frame(count=5)[:-1], 35)])
def test_truncated_tail_variants(tail, n_tail):
    data = tm2_frame(flags=0xC8, count=4) + tail
    log = parse_ubx(data)
    assert [m.count for m in log.tm2] == [4]
    assert log.stats.truncated_tail_bytes == n_tail
    assert log.stats.garbage_bytes == 0


def test_valid_checksum_but_wrong_length_counted_as_decode_error():
    data = (frame(0x0D, 0x03, tm2_payload()[:27])                     # TM2 少一個位元組
            + frame(0x02, 0x15, b"\x00" * 17)                          # RAWX numMeas=0 卻有 17 位元組
            + frame(0x02, 0x13, _le([(0, 1)] * 4 + [(2, 1)] + [(0, 1)] * 3) + bytes(4))  # numWords=2 只帶 1 個字
            + frame(0x01, 0x07, pvt_payload()[:91]))                   # PVT 少一個位元組
    log = parse_ubx(data)
    assert log.stats.frames == 4                 # 檢查碼都對，框都切得出來
    assert log.tm2 == [] and log.rawx == [] and log.pvt == []
    assert log.sfrbx_count == 0
    assert log.stats.decode_errors == 4


def test_rawx_header_fields_and_epoch_interval():
    data = b"".join(rawx_frame(180000.0 + 0.1 * k, 2389, leap=18, nmeas=k % 4, rec_stat=1)
                    for k in range(11))
    log = parse_ubx(data)
    assert log.rawx_count == 11
    r = log.rawx[3]
    assert (r.week, r.leap_s, r.num_meas, r.leap_valid) == (2389, 18, 3, True)
    assert r.rcv_tow_s == pytest.approx(180000.3)
    assert log.rawx_epoch_interval_s() == pytest.approx(0.1)


def test_read_ubx_file_small_chunks(tmp_path):
    data, good, tail = _messy_stream()
    p = tmp_path / "rover.ubx"
    p.write_bytes(data)
    log = read_ubx_file(p, chunk_size=5)
    assert [m.count for m in log.tm2] == [1, 2]
    assert log.stats.truncated_tail_bytes == len(tail)


def test_numpy_branch_used_for_long_frames(monkeypatch):
    """長框走 numpy 展開式，結果仍等於規格迴圈（防止只有短框被測到）。"""
    calls = []
    real = np.frombuffer

    def spy(*a, **k):
        calls.append(1)
        return real(*a, **k)

    monkeypatch.setattr(ubx.np, "frombuffer", spy)
    f = rawx_frame(1.0, 2389, nmeas=10)
    assert parse_ubx(f).rawx_count == 1
    assert calls
