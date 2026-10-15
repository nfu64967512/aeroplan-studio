"""survey/ppk/ubx.py — 最小的串流 UBX 解析器（相機模組 ZED-F9P 的 gnss/rover.ubx）。

只做 PPK 鏈需要的事：
- 切出 UBX 框（同步字元 0xB5 0x62、class/id、小端序長度、8-bit Fletcher 檢查碼），
  框與框之間的垃圾、NMEA、半截框都要能跳過，且**不能因為一個假的長度把後面的真框吞掉**；
- 解 UBX-TIM-TM2（EXTINT 時間標記）與 UBX-NAV-PVT；
- RXM-RAWX 只解表頭（rcvTow、week、leapS、numMeas、recStat）用來數 epoch 與核對閏秒，
  RXM-SFRBX 只計數。完整的 RAWX → RINEX 交給 RTKLIB convbin（見 rtklib.py）。

格式依據（全部對照過原文，pdftotext 轉出後逐欄核對）：
  u-blox F9 HPG 1.32 Interface Description, UBX-22008968 R01（2022-05-02）
  - §3.2 UBX frame structure（p.53）：0xb5 0x62、class、id、U2 小端序 length（只算 payload）
  - §3.4 UBX checksum（p.56）：8-bit Fletcher，範圍「從 class 開始到 checksum 之前」
  - §3.15.13 UBX-NAV-PVT 0x01 0x07，長度 92（p.145–147）
  - §3.17.6 UBX-RXM-RAWX 0x02 0x15，長度 16 + numMeas·32（p.188）
  - §3.17.9 UBX-RXM-SFRBX 0x02 0x13，長度 8 + numWords·4（p.192）
  - §3.19.1 UBX-TIM-TM2 0x0d 0x03，長度 28（p.194–195）
  ZED-F9P Integration Manual UBX-18010802 R12 §3.13.2 Time mark（p.67–68）：
  「Only the last rising and falling edge detected between two epochs is reported」
  ⇒ 兩個邊緣落在同一個量測 epoch 時前一個會遺失，只能靠 count 欄位的跳號偵測（見 events.py）。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

__all__ = [
    "SYNC",
    "UBX_NAV_PVT",
    "UBX_RXM_RAWX",
    "UBX_RXM_SFRBX",
    "UBX_TIM_TM2",
    "UbxFrame",
    "UbxStats",
    "TimTm2",
    "NavPvt",
    "RawxHeader",
    "UbxLog",
    "UbxStreamParser",
    "ubx_checksum",
    "build_frame",
    "decode_tim_tm2",
    "decode_nav_pvt",
    "decode_rawx_header",
    "parse_ubx",
    "read_ubx_file",
]

SYNC = b"\xb5\x62"

# (class, id)
UBX_NAV_PVT = (0x01, 0x07)
UBX_RXM_RAWX = (0x02, 0x15)
UBX_RXM_SFRBX = (0x02, 0x13)
UBX_TIM_TM2 = (0x0D, 0x03)

TIM_TM2_LEN = 28      # §3.19.1
NAV_PVT_LEN = 92      # §3.15.13
RAWX_HEAD_LEN = 16    # §3.17.6：16 + numMeas·32
RAWX_MEAS_LEN = 32
SFRBX_HEAD_LEN = 8    # §3.17.9：8 + numWords·4

_HDR = 6              # sync(2) + class + id + length(2)
_OVERHEAD = _HDR + 2  # + CK_A CK_B


def ubx_checksum(body: bytes) -> bytes:
    """8-bit Fletcher（§3.4），``body`` = class、id、length、payload（不含同步字元）。

    規格的迴圈是 CK_A += x_i、CK_B += CK_A（皆 mod 256），展開即
    CK_A = Σx_i、CK_B = Σ(n−i)·x_i（mod 256）。長框（RAWX 一框 ~1.3 KB、10 Hz 一小時 ~47 MB）
    用 numpy 算展開式，短框照規格逐位元組算；兩條路在測試裡對同一批隨機框比對過。
    """
    n = len(body)
    if n > 64:
        x = np.frombuffer(bytes(body), dtype=np.uint8).astype(np.int64)
        return bytes((int(x.sum()) & 0xFF, int((x * np.arange(n, 0, -1)).sum()) & 0xFF))
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes((a, b))


def build_frame(cls: int, msg_id: int, payload: bytes) -> bytes:
    """組一個完整 UBX 框（測試與台架用；不做任何欄位檢查）。"""
    body = struct.pack("<BBH", cls, msg_id, len(payload)) + bytes(payload)
    return SYNC + body + ubx_checksum(body)


@dataclass(frozen=True)
class UbxFrame:
    cls: int
    msg_id: int
    payload: bytes
    offset: int  # 同步字元在整個串流中的位元組位置（保留串流順序，events.py 拿它對 NAV-PVT）


@dataclass
class UbxStats:
    """解析統計 —— 跳過的東西一律算出來給人看，不靜默吞掉。"""
    frames: int = 0
    checksum_errors: int = 0       # 有同步字元、長度夠，但檢查碼不對（多半是垃圾裡剛好出現 B5 62）
    garbage_bytes: int = 0         # 不屬於任何有效框的位元組（NMEA、雜訊、壞框）
    truncated_tail_bytes: int = 0  # 串流結尾那個不完整框的長度（斷電、檔案還在寫）
    decode_errors: int = 0         # 檢查碼對但長度與規格不符（韌體版本不同？）
    by_type: dict = field(default_factory=dict)  # (cls, id) -> 框數

    def count(self, cls: int, msg_id: int) -> int:
        return self.by_type.get((cls, msg_id), 0)


class UbxStreamParser:
    """可分段餵資料的 UBX 切框器。

    為什麼不能「看到長度超過緩衝區就當成截斷尾巴」：垃圾裡剛好出現 B5 62，後面兩個位元組
    又被讀成一個很大的長度（最大 65535），若直接等資料或直接丟到結尾，後面所有真框都會被吞掉。
    這裡的做法：未收齊就先等（feed 模式），收齊後檢查碼不對就只前進一個位元組重新找同步字元；
    ``close()`` 時剩下收不齊的候選框也只跳過一個位元組繼續掃，真正的截斷尾巴才記成
    ``truncated_tail_bytes``。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._base = 0          # self._buf[0] 在整個串流中的位置
        self.stats = UbxStats()

    def feed(self, data: bytes) -> list[UbxFrame]:
        self._buf += data
        return self._scan(final=False)

    def close(self) -> list[UbxFrame]:
        out = self._scan(final=True)
        return out

    def _scan(self, final: bool) -> list[UbxFrame]:
        buf = self._buf
        n = len(buf)
        i = 0
        out: list[UbxFrame] = []
        st = self.stats
        while i < n:
            j = buf.find(SYNC, i)
            if j < 0:
                # 最後一個位元組是 0xB5 ⇒ 可能是下一框的開頭：feed 模式留著等，close 時算截斷尾巴
                tail = 1 if buf[n - 1] == SYNC[0] else 0
                st.garbage_bytes += (n - tail) - i
                if final:
                    st.truncated_tail_bytes += tail
                    i = n
                else:
                    i = n - tail
                break
            st.garbage_bytes += j - i
            i = j
            if n - i < _HDR:
                if final:
                    st.truncated_tail_bytes += n - i
                    i = n
                break
            length = buf[i + 4] | (buf[i + 5] << 8)
            end = i + _OVERHEAD + length
            if end > n:
                if not final:
                    break               # 等資料
                # 結尾收不齊：可能是真截斷，也可能是垃圾裡的假長度 ⇒ 跳一個位元組繼續找
                nxt = buf.find(SYNC, i + 1)
                if nxt < 0:
                    st.truncated_tail_bytes += n - i
                    i = n
                    break
                st.garbage_bytes += nxt - i
                i = nxt
                continue
            body = bytes(buf[i + 2:end - 2])
            if ubx_checksum(body) != bytes(buf[end - 2:end]):
                st.checksum_errors += 1
                st.garbage_bytes += 1
                i += 1
                continue
            cls, mid = body[0], body[1]
            out.append(UbxFrame(cls, mid, body[4:], self._base + i))
            st.frames += 1
            st.by_type[(cls, mid)] = st.by_type.get((cls, mid), 0) + 1
            i = end
        del buf[:i]
        self._base += i
        return out


# ---------------------------------------------------------------------------
# 解碼
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TimTm2:
    """UBX-TIM-TM2（§3.19.1，p.194–195）。時間欄位保持原始整數，換算見 events.py。"""
    ch: int
    flags: int
    count: int            # Rising edge counter（U2，會在 65536 繞回）
    wn_r: int
    wn_f: int
    tow_ms_r: int
    tow_sub_ms_r_ns: int  # Millisecond fraction of tow of rising edge in nanoseconds
    tow_ms_f: int
    tow_sub_ms_f_ns: int
    acc_est_ns: int
    offset: int = -1

    # flags 位元定義（§3.19.1）
    @property
    def mode_running(self) -> bool:     # bit0：0=single、1=running
        return bool(self.flags & 0x01)

    @property
    def stopped(self) -> bool:          # bit1：0=armed、1=stopped
        return bool(self.flags & 0x02)

    @property
    def new_falling_edge(self) -> bool:  # bit2
        return bool(self.flags & 0x04)

    @property
    def time_base(self) -> int:         # bits4..3：0=receiver、1=GNSS（CFG-TP 指定的系統）、2=UTC
        return (self.flags >> 3) & 0x03

    @property
    def utc_available(self) -> bool:    # bit5
        return bool(self.flags & 0x20)

    @property
    def time_valid(self) -> bool:       # bit6：1 = Time is valid (Valid GNSS fix)
        return bool(self.flags & 0x40)

    @property
    def new_rising_edge(self) -> bool:  # bit7
        return bool(self.flags & 0x80)


_TM2 = struct.Struct("<BBHHHIIIII")
assert _TM2.size == TIM_TM2_LEN


def decode_tim_tm2(payload: bytes, offset: int = -1) -> TimTm2:
    if len(payload) != TIM_TM2_LEN:
        raise ValueError(f"UBX-TIM-TM2 payload 應為 {TIM_TM2_LEN} 位元組，收到 {len(payload)}")
    ch, flags, count, wn_r, wn_f, tms_r, tsub_r, tms_f, tsub_f, acc = _TM2.unpack(payload)
    return TimTm2(ch, flags, count, wn_r, wn_f, tms_r, tsub_r, tms_f, tsub_f, acc, offset)


@dataclass(frozen=True)
class NavPvt:
    """UBX-NAV-PVT（§3.15.13，p.145–147），已換成 SI 單位。height 是**橢球高**、h_msl 是接收機內建大地起伏模型的海拔。"""
    itow_ms: int          # GPS time of week of the navigation epoch
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: int
    valid: int
    t_acc_ns: int
    nano_ns: int
    fix_type: int         # 0 no fix、1 DR、2 2D、3 3D、4 GNSS+DR、5 time only
    flags: int
    flags2: int
    num_sv: int
    lon_deg: float
    lat_deg: float
    height_m: float
    h_msl_m: float
    h_acc_m: float
    v_acc_m: float
    vel_n_mps: float
    vel_e_mps: float
    vel_d_mps: float
    g_speed_mps: float
    head_mot_deg: float
    s_acc_mps: float
    head_acc_deg: float
    p_dop: float
    flags3: int
    offset: int = -1

    @property
    def gnss_fix_ok(self) -> bool:      # flags bit0
        return bool(self.flags & 0x01)

    @property
    def diff_soln(self) -> bool:        # flags bit1
        return bool(self.flags & 0x02)

    @property
    def carr_soln(self) -> int:         # flags bits7..6：0 無、1 float、2 fixed
        return (self.flags >> 6) & 0x03

    @property
    def invalid_llh(self) -> bool:      # flags3 bit0
        return bool(self.flags3 & 0x01)


# offset 0..79 依 §3.15.13；80..91 是 reserved0[4]、headVeh、magDec、magAcc，PPK 用不到
_PVT = struct.Struct("<IHBBBBBBIiBBBBiiiiIIiiiiiIIHH")
assert _PVT.size == 80


def decode_nav_pvt(payload: bytes, offset: int = -1) -> NavPvt:
    if len(payload) != NAV_PVT_LEN:
        raise ValueError(f"UBX-NAV-PVT payload 應為 {NAV_PVT_LEN} 位元組，收到 {len(payload)}")
    (itow, year, month, day, hour, minute, sec, valid, t_acc, nano, fix_type, flags, flags2,
     num_sv, lon, lat, height, hmsl, h_acc, v_acc, vel_n, vel_e, vel_d, g_speed, head_mot,
     s_acc, head_acc, p_dop, flags3) = _PVT.unpack_from(payload, 0)
    return NavPvt(
        itow_ms=itow, year=year, month=month, day=day, hour=hour, minute=minute, second=sec,
        valid=valid, t_acc_ns=t_acc, nano_ns=nano, fix_type=fix_type, flags=flags,
        flags2=flags2, num_sv=num_sv,
        lon_deg=lon * 1e-7, lat_deg=lat * 1e-7,             # scale 1e-7 deg
        height_m=height * 1e-3, h_msl_m=hmsl * 1e-3,         # mm
        h_acc_m=h_acc * 1e-3, v_acc_m=v_acc * 1e-3,          # mm
        vel_n_mps=vel_n * 1e-3, vel_e_mps=vel_e * 1e-3, vel_d_mps=vel_d * 1e-3,  # mm/s
        g_speed_mps=g_speed * 1e-3,
        head_mot_deg=head_mot * 1e-5,                         # scale 1e-5 deg
        s_acc_mps=s_acc * 1e-3,
        head_acc_deg=head_acc * 1e-5,
        p_dop=p_dop * 0.01,
        flags3=flags3,
        offset=offset,
    )


@dataclass(frozen=True)
class RawxHeader:
    """UBX-RXM-RAWX 的表頭（§3.17.6，p.188–189）；量測本體交給 convbin。"""
    rcv_tow_s: float      # 接收機本地時間的週內秒，約略對齊 GPS 時間
    week: int
    leap_s: int           # GPS − UTC 閏秒（接收機目前的認知）
    num_meas: int
    rec_stat: int
    offset: int = -1

    @property
    def leap_valid(self) -> bool:       # recStat bit0：Leap seconds have been determined
        return bool(self.rec_stat & 0x01)


_RAWX_HEAD = struct.Struct("<dHbBBB")


def decode_rawx_header(payload: bytes, offset: int = -1) -> RawxHeader:
    if len(payload) < RAWX_HEAD_LEN:
        raise ValueError(f"UBX-RXM-RAWX payload 太短：{len(payload)}")
    rcv_tow, week, leap_s, num_meas, rec_stat, _ver = _RAWX_HEAD.unpack_from(payload, 0)
    if len(payload) != RAWX_HEAD_LEN + RAWX_MEAS_LEN * num_meas:
        raise ValueError(
            f"UBX-RXM-RAWX 長度 {len(payload)} 與 numMeas={num_meas} 不符"
            f"（應為 {RAWX_HEAD_LEN + RAWX_MEAS_LEN * num_meas}）")
    return RawxHeader(rcv_tow, week, leap_s, num_meas, rec_stat, offset)


@dataclass
class UbxLog:
    """一個 rover.ubx 的解析結果（串流順序保留在各物件的 ``offset``）。"""
    tm2: list[TimTm2]
    pvt: list[NavPvt]
    rawx: list[RawxHeader]
    sfrbx_count: int
    stats: UbxStats

    @property
    def rawx_count(self) -> int:
        return len(self.rawx)

    def rawx_epoch_interval_s(self) -> float | None:
        """RAWX 相鄰 epoch 的中位間隔（判斷是否真的是 10 Hz）；不足兩筆回 None。"""
        if len(self.rawx) < 2:
            return None
        d = sorted((b.week - a.week) * 604800 + (b.rcv_tow_s - a.rcv_tow_s)
                   for a, b in zip(self.rawx, self.rawx[1:]))
        return d[len(d) // 2]


def _decode(frames: Iterable[UbxFrame], log: UbxLog) -> None:
    for f in frames:
        key = (f.cls, f.msg_id)
        try:
            if key == UBX_TIM_TM2:
                log.tm2.append(decode_tim_tm2(f.payload, f.offset))
            elif key == UBX_NAV_PVT:
                log.pvt.append(decode_nav_pvt(f.payload, f.offset))
            elif key == UBX_RXM_RAWX:
                log.rawx.append(decode_rawx_header(f.payload, f.offset))
            elif key == UBX_RXM_SFRBX:
                if len(f.payload) < SFRBX_HEAD_LEN or \
                        len(f.payload) != SFRBX_HEAD_LEN + 4 * f.payload[4]:
                    raise ValueError("UBX-RXM-SFRBX 長度與 numWords 不符")
                log.sfrbx_count += 1
        except ValueError:
            log.stats.decode_errors += 1


def parse_ubx(data: bytes, chunk_size: int = 1 << 20) -> UbxLog:
    """解析整段 UBX 位元組串流。"""
    p = UbxStreamParser()
    log = UbxLog([], [], [], 0, p.stats)
    mv = memoryview(data)
    for k in range(0, len(data), chunk_size):
        _decode(p.feed(bytes(mv[k:k + chunk_size])), log)
    _decode(p.close(), log)
    return log


def read_ubx_file(path: str | Path, chunk_size: int = 1 << 20) -> UbxLog:
    p = UbxStreamParser()
    log = UbxLog([], [], [], 0, p.stats)
    with open(path, "rb") as fh:
        while True:
            chunk = fh.read(chunk_size)
            if not chunk:
                break
            _decode(p.feed(chunk), log)
    _decode(p.close(), log)
    return log
