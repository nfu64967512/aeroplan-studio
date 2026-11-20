"""ZED-F9P 流動站設定（UBX-CFG-VALSET）與 rover.ubx 原始紀錄器 —— 第二階段 PPK

所有 key ID、型別、訊息結構都對照 u-blox「u-blox F9 HPG 1.32 Interface Description」
UBX-22008968 R01（2022-05-02，ZED-F9P 介面版本 27.31）原文核對，頁碼記在 KEYS 表內；
行為說明另參照「ZED-F9P Integration manual」UBX-18010802 R12（下稱 IM）。

- UBX 框架：0xB5 0x62、class、id、長度 U2 小端、payload、CK_A CK_B（§3.4 p.56：8-bit Fletcher，
  涵蓋 class 起到 payload 末，不含同步字元）。
- UBX-CFG-VALSET（class 0x06 id 0x8A，§3.10.26 p.96）：version U1=0x00、layers X1（bit0 RAM、bit1 BBR、
  bit2 Flash）、reserved0 U1[2]、之後是 cfgData；單一訊息最多 64 組 key-value。任何 key 不認得就 ACK-NAK
  且整則不套用。
- Key ID（§6.2 p.223–224）：bits 30…28 是值的儲存大小（0x01 一位元〔存成 1 byte〕、0x02 1 byte、
  0x03 2 bytes、0x04 4 bytes、0x05 8 bytes）；cfgData 是 key(U4 小端)+值 直接串接、不補齊（§6.5 p.225）。
- 圖層（§6.3 p.224）：RAM 立即生效；BBR／Flash 要重開機後才從該層載入。

時間標記（TIM-TM2）：
- IM §3.13.2（p.67–68）：TIM-TM2 在「訊息已啟用」且「上個 epoch 後 EXTINT 有上升/下降緣」時於下個 epoch 輸出；
  **不需要另外的 EXTINT 開關 key**。每個 epoch 只回報最後一個上升與下降緣 ⇒ 量測週期必須短於拍照間隔。
- 時間基準取自 CFG-TP-*（IM 同節；介面描述 p.194 TIM-TM2 flags.timeBase：「GNSS time (the system according
  to the configuration in UBX-CFG-TP5 for tpIdx=0)」，而 UBX-CFG-TP5.gridUtcGnss ↔ CFG-TP-TIMEGRID_TP1，p.279 對照表）。
  CFG-TP-TIMEGRID_TP1 預設 0（UTC，p.300 預設值表），所以這裡設成 1（GPS），讓 TIM-TM2 直接是 GPST（F3）。
  ⚠ TIMEGRID_TP1 的說明寫「Only relevant if CFG-TP-USE_LOCKED_TP1 and ALIGN_TO_TOW_TP1 are set」（指 TP 腳），
  它對 TIM-TM2 的實際效果 UNVERIFIED：地面端必須檢查每筆 TIM-TM2 的 timeBase 位元（bits 4…3 == 1），不符就拒用。
  副作用：TIMEPULSE 腳的時間格也改成 GPS（本案不用 TP 腳）。

鮑率：UART 預設 38400（IM §3.1.3 p.13）；可選 9600…921600（IM Table 28 p.47）。10 Hz RAWX 在 38400 下
塞不下（輸出緩衝溢位就丟訊息；IM p.13 與資料表 p.19 都只提醒「Make sure the UART baud rate used is sufficient」），
所以 UART 埠必須明確給鮑率，且通過下面的頻寬估算。
UART2 要 HPG 1.30 起才能輸出 UBX（IM §3.1.3 p.13、p.47），舊韌體上設定會被接受但不會有資料（rover 紀錄器會報停流）。
"""
from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass

log = logging.getLogger("pi_capture.f9p")

SYNC = b"\xb5\x62"
CLS_ACK, ID_ACK_ACK, ID_ACK_NAK = 0x05, 0x01, 0x00
CLS_CFG, ID_CFG_VALSET = 0x06, 0x8A
LAYER_RAM, LAYER_BBR, LAYER_FLASH = 0x01, 0x02, 0x04
ALL_LAYERS = LAYER_RAM | LAYER_BBR | LAYER_FLASH
MAX_KEYS_PER_VALSET = 64

TIMEREF_GPS = 1                   # CFG-RATE-TIMEREF Table 38（p.262）：UTC 0、GPS 1、GLO 2、BDS 3、GAL 4、NAVIC 5
TIMEGRID_GPS = 1                  # CFG-TP-TIMEGRID_TP1 Table 57（p.269–270）：UTC 0、GPS 1、GLO 2、BDS 3、GAL 4、NAVIC 5
UART_BAUDS = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)   # IM Table 28 p.47
MIN_MEAS_MS = 67                  # 資料表 UBX-21044850 R01 p.4：預設四星系 RAW 最高 15 Hz
MAX_MEAS_MS = 1000
PORTS = ("UART1", "UART2", "USB")

ROVER_UBX = "gnss/rover.ubx"


@dataclass(frozen=True)
class CfgKey:
    name: str
    key_id: int
    type: str                     # L / U1 / U2 / U4 / E1（介面描述 §6.2 的型別名）
    page: int                     # UBX-22008968 R01 頁碼


_SIZE_OF_TYPE = {"L": 1, "U1": 1, "E1": 1, "X1": 1, "I1": 1,
                 "U2": 2, "E2": 2, "X2": 2, "I2": 2,
                 "U4": 4, "E4": 4, "X4": 4, "I4": 4}
_SIZE_CODE_BYTES = {0x01: 1, 0x02: 1, 0x03: 2, 0x04: 4, 0x05: 8}   # Key ID bits 30…28

KEYS = {k.name: k for k in (
    CfgKey("CFG-RATE-MEAS", 0x30210001, "U2", 261),
    CfgKey("CFG-RATE-NAV", 0x30210002, "U2", 262),
    CfgKey("CFG-RATE-TIMEREF", 0x20210003, "E1", 262),
    CfgKey("CFG-TP-TIMEGRID_TP1", 0x2005000C, "E1", 269),
    CfgKey("CFG-MSGOUT-UBX_RXM_RAWX_UART1", 0x209102A5, "U1", 254),
    CfgKey("CFG-MSGOUT-UBX_RXM_RAWX_UART2", 0x209102A6, "U1", 254),
    CfgKey("CFG-MSGOUT-UBX_RXM_RAWX_USB", 0x209102A7, "U1", 254),
    CfgKey("CFG-MSGOUT-UBX_RXM_SFRBX_UART1", 0x20910232, "U1", 254),
    CfgKey("CFG-MSGOUT-UBX_RXM_SFRBX_UART2", 0x20910233, "U1", 254),
    CfgKey("CFG-MSGOUT-UBX_RXM_SFRBX_USB", 0x20910234, "U1", 254),
    CfgKey("CFG-MSGOUT-UBX_TIM_TM2_UART1", 0x20910179, "U1", 255),
    CfgKey("CFG-MSGOUT-UBX_TIM_TM2_UART2", 0x2091017A, "U1", 255),
    CfgKey("CFG-MSGOUT-UBX_TIM_TM2_USB", 0x2091017B, "U1", 255),
    CfgKey("CFG-MSGOUT-UBX_NAV_PVT_UART1", 0x20910007, "U1", 250),
    CfgKey("CFG-MSGOUT-UBX_NAV_PVT_UART2", 0x20910008, "U1", 250),
    CfgKey("CFG-MSGOUT-UBX_NAV_PVT_USB", 0x20910009, "U1", 250),
    CfgKey("CFG-UART1OUTPROT-UBX", 0x10740001, "L", 271),
    CfgKey("CFG-UART1OUTPROT-NMEA", 0x10740002, "L", 271),
    CfgKey("CFG-UART2OUTPROT-UBX", 0x10760001, "L", 273),
    CfgKey("CFG-UART2OUTPROT-NMEA", 0x10760002, "L", 273),
    CfgKey("CFG-USBOUTPROT-UBX", 0x10780001, "L", 274),
    CfgKey("CFG-USBOUTPROT-NMEA", 0x10780002, "L", 274),
    CfgKey("CFG-UART1-BAUDRATE", 0x40520001, "U4", 270),
    CfgKey("CFG-UART2-BAUDRATE", 0x40530001, "U4", 271),
)}

_OUTPROT_PREFIX = {"UART1": "CFG-UART1OUTPROT", "UART2": "CFG-UART2OUTPROT", "USB": "CFG-USBOUTPROT"}

# 頻寬估算（⚠ 估計值，台架 B9 以實際 byte 數驗證）：每則 UBX 額外 8 bytes（同步 2＋class/id 2＋長度 2＋校驗 2）
RAWX_MAX_MEAS = 96                # 四星系 × 雙頻 × 每系約 12 顆的上限假設
SFRBX_BYTES_PER_S = 3000          # 導航電文配額（含北斗 GEO 每 0.6 s 一頁）
UART_MAX_UTILISATION = 0.8


def key_size_bytes(key_id: int) -> int:
    code = (key_id >> 28) & 0x7
    if code not in _SIZE_CODE_BYTES:
        raise ValueError(f"key 0x{key_id:08x} 的大小碼 {code} 不合法")
    return _SIZE_CODE_BYTES[code]


def encode_value(key: CfgKey, value) -> bytes:
    """依型別編碼成小端值，並檢查與 key ID 的大小碼一致、數值在範圍內。"""
    size = _SIZE_OF_TYPE[key.type]
    if size != key_size_bytes(key.key_id):
        raise ValueError(f"{key.name}：型別 {key.type} 與 key ID 大小碼不一致")
    if key.type == "L":
        if value not in (0, 1) or isinstance(value, float):
            raise ValueError(f"{key.name} 是布林，收到 {value!r}")
        return struct.pack("<B", int(value))
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{key.name} 需要整數，收到 {value!r}")
    hi = (1 << (8 * size)) - 1
    if not 0 <= value <= hi:
        raise ValueError(f"{key.name}={value} 超出 {key.type} 範圍 0…{hi}")
    return value.to_bytes(size, "little")


def ubx_checksum(data: bytes) -> bytes:
    """8-bit Fletcher（§3.4 p.56），data 從 class 起算到 payload 末。"""
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes((a, b))


def ubx_frame(cls: int, msg_id: int, payload: bytes) -> bytes:
    body = bytes((cls, msg_id)) + struct.pack("<H", len(payload)) + payload
    return SYNC + body + ubx_checksum(body)


def valset(items, layers: int = ALL_LAYERS) -> bytes:
    """items：[(key 名稱, 值), …] → 一則 UBX-CFG-VALSET（version 0、無交易）。"""
    if not isinstance(layers, int) or layers & ~ALL_LAYERS or layers == 0:
        raise ValueError(f"layers 必須是 RAM/BBR/Flash 位元的非空組合，收到 {layers!r}")
    items = list(items)
    if not 1 <= len(items) <= MAX_KEYS_PER_VALSET:
        raise ValueError(f"一則 VALSET 要 1–{MAX_KEYS_PER_VALSET} 組 key-value，收到 {len(items)}")
    cfg = b""
    for name, value in items:
        key = KEYS[name]
        cfg += struct.pack("<I", key.key_id) + encode_value(key, value)
    return ubx_frame(CLS_CFG, ID_CFG_VALSET, bytes((0x00, layers, 0x00, 0x00)) + cfg)


def estimate_rover_bytes_per_s(meas_rate_ms: int) -> float:
    """RAWX(8+16+32·N) + NAV-PVT(8+92) + TIM-TM2(8+28) 每 epoch，加 SFRBX 固定配額。估計值。"""
    epochs = 1000.0 / meas_rate_ms
    per_epoch = (8 + 16 + 32 * RAWX_MAX_MEAS) + (8 + 92) + (8 + 28)
    return per_epoch * epochs + SFRBX_BYTES_PER_S


@dataclass(frozen=True)
class ConfigMessage:
    description: str
    frame: bytes
    items: tuple
    expect_ack: bool


def build_rover_config(port: str = "USB", *, meas_rate_ms: int = 100, baudrate: int | None = None,
                       layers: int = ALL_LAYERS,
                       min_photo_interval_s: float | None = None) -> list[ConfigMessage]:
    """相機模組 F9P 流動站：量測 meas_rate_ms、該埠只出 UBX（關 NMEA）、每個 epoch 輸出
    RXM-RAWX／RXM-SFRBX／TIM-TM2／NAV-PVT、TIM-TM2 時間基準 GPS，寫進 layers（預設 RAM+BBR+Flash）。

    UART 埠必須給 baudrate（且通過頻寬估算），鮑率放在最後一則、而且不等 ACK（若主機正是經由該 UART
    連線，接收機會用新鮮率回 ACK，主機收不到）——換完要用新鮮率重新開埠。USB 不可給 baudrate。
    min_photo_interval_s：給了就檢查量測週期 < 拍照間隔（TIM-TM2 每 epoch 只回報最後一個邊緣，IM §3.13.2）。
    """
    if port not in PORTS:
        raise ValueError(f"port 必須是 {PORTS} 之一，收到 {port!r}")
    if (not isinstance(meas_rate_ms, int) or isinstance(meas_rate_ms, bool)
            or not MIN_MEAS_MS <= meas_rate_ms <= MAX_MEAS_MS):
        raise ValueError(f"meas_rate_ms 必須是 {MIN_MEAS_MS}–{MAX_MEAS_MS} 的整數"
                         f"（四星系 RAW 上限 15 Hz），收到 {meas_rate_ms!r}")
    if min_photo_interval_s is not None and meas_rate_ms / 1000.0 >= min_photo_interval_s:
        raise ValueError(f"量測週期 {meas_rate_ms} ms 不短於最短拍照間隔 {min_photo_interval_s} s："
                         "同一 epoch 內兩個邊緣只會回報最後一個（IM §3.13.2）")
    if port == "USB":
        if baudrate is not None:
            raise ValueError("USB 沒有鮑率；baudrate 只用於 UART1/UART2")
    else:
        if baudrate not in UART_BAUDS:
            raise ValueError(f"{port} 必須明確指定鮑率，可選 {UART_BAUDS}，收到 {baudrate!r}"
                             "（預設 38400 塞不下 10 Hz RAWX）")
        need = estimate_rover_bytes_per_s(meas_rate_ms)
        cap = baudrate / 10.0 * UART_MAX_UTILISATION
        if need > cap:
            raise ValueError(f"{port} {baudrate} baud 可用 {cap:.0f} B/s（{UART_MAX_UTILISATION:.0%}），"
                             f"估計需要 {need:.0f} B/s：請提高鮑率或改用 USB")

    sfx = port
    items = (
        ("CFG-RATE-MEAS", meas_rate_ms),
        ("CFG-RATE-NAV", 1),
        ("CFG-RATE-TIMEREF", TIMEREF_GPS),
        ("CFG-TP-TIMEGRID_TP1", TIMEGRID_GPS),
        (f"{_OUTPROT_PREFIX[port]}-UBX", 1),
        (f"{_OUTPROT_PREFIX[port]}-NMEA", 0),
        (f"CFG-MSGOUT-UBX_RXM_RAWX_{sfx}", 1),
        (f"CFG-MSGOUT-UBX_RXM_SFRBX_{sfx}", 1),
        (f"CFG-MSGOUT-UBX_TIM_TM2_{sfx}", 1),
        (f"CFG-MSGOUT-UBX_NAV_PVT_{sfx}", 1),
    )
    msgs = [ConfigMessage(f"{port}：{meas_rate_ms} ms、只出 UBX、RAWX/SFRBX/TIM-TM2/NAV-PVT 每 epoch、"
                          f"TIM-TM2 時間基準 GPS", valset(items, layers), items, True)]
    if port != "USB":
        b_items = ((f"CFG-{port}-BAUDRATE", baudrate),)
        msgs.append(ConfigMessage(f"{port} 鮑率 {baudrate}（之後要用新鮮率重新開埠）",
                                  valset(b_items, layers), b_items, False))
    return msgs


class UbxReader:
    """把位元組流切成 UBX 訊息（校驗不符或混在中間的 NMEA 都跳過）。"""

    def __init__(self):
        self._buf = bytearray()
        self.bad_checksums = 0

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self._buf += data
        out = []
        while True:
            i = self._buf.find(SYNC)
            if i < 0:
                del self._buf[:max(0, len(self._buf) - 1)]
                return out
            if i:
                del self._buf[:i]
            if len(self._buf) < 6:
                return out
            n = struct.unpack_from("<H", self._buf, 4)[0]
            if len(self._buf) < 8 + n:
                return out
            body = bytes(self._buf[2:6 + n])
            if ubx_checksum(body) == bytes(self._buf[6 + n:8 + n]):
                out.append((body[0], body[1], body[4:]))
                del self._buf[:8 + n]
            else:
                self.bad_checksums += 1
                del self._buf[:2]


def apply_config(stream, messages, *, ack_timeout_s: float = 2.0, monotonic=time.monotonic) -> list[str]:
    """逐則送出並等 UBX-ACK-ACK（payload = 0x06 0x8A）。NAK 或逾時就丟例外——不能讓設定靜默失效。
    expect_ack=False 的訊息（換鮑率）只送不等，回傳的紀錄裡會註明。"""
    report = []
    for m in messages:
        stream.write(m.frame)
        stream.flush()
        if not m.expect_ack:
            report.append(f"已送出（不等 ACK）：{m.description}")
            continue
        reader = UbxReader()
        deadline = monotonic() + ack_timeout_s
        result = None
        while result is None and monotonic() < deadline:
            for cls, mid, payload in reader.feed(stream.read(256)):
                if cls == CLS_ACK and payload[:2] == bytes((CLS_CFG, ID_CFG_VALSET)):
                    result = mid
                    break
        if result == ID_ACK_ACK:
            report.append(f"ACK：{m.description}")
        elif result == ID_ACK_NAK:
            raise RuntimeError(f"接收機拒絕（ACK-NAK），整則未套用：{m.description}")
        else:
            raise TimeoutError(f"{ack_timeout_s} s 內沒有收到 ACK：{m.description}")
    return report


# ── rover.ubx 原始紀錄器 ──────────────────────────────────────────────────────

class RoverLogger:
    """把接收機輸出的位元組原封不動寫進 gnss/rover.ubx（熱路徑不解析），每 fsync_period_s 秒 fsync。
    超過 stall_warn_s 秒沒有資料就大聲報錯（每次停流報一次）。"""

    def __init__(self, storage, stream, clock, *, fsync_period_s: float = 1.0,
                 stall_warn_s: float = 5.0, chunk: int = 4096):
        if not (isinstance(fsync_period_s, (int, float)) and fsync_period_s > 0):
            raise ValueError(f"fsync_period_s 必須 > 0，收到 {fsync_period_s!r}")
        self.storage = storage
        self.stream = stream
        self.clock = clock
        self.chunk = chunk
        self._fsync_ns = int(round(fsync_period_s * 1e9))
        self._stall_ns = int(round(stall_warn_s * 1e9))
        self._f = None
        self._last_fsync = None
        self._last_data = None
        self._dirty = False
        self.stalled = False
        self.bytes = 0
        self.fsyncs = 0
        self.stall_events = 0

    def open(self) -> None:
        if self.storage.path(ROVER_UBX).exists():
            raise FileExistsError(f"{ROVER_UBX} 已存在：絕不覆寫既有紀錄")
        self.storage.makedirs("gnss")
        self._f = self.storage.open_append(ROVER_UBX)
        now = self.clock.monotonic_ns()
        self._last_fsync = self._last_data = now

    def step(self) -> int:
        data = self.stream.read(self.chunk)
        now = self.clock.monotonic_ns()
        if data:
            self._f.write(data)
            self.bytes += len(data)
            self._dirty = True
            self._last_data = now
            if self.stalled:
                log.warning("GNSS 資料恢復")
            self.stalled = False
        elif not self.stalled and now - self._last_data > self._stall_ns:
            self.stalled = True
            self.stall_events += 1
            log.error("GNSS 已 %.1f s 沒有資料（埠、鮑率或 UART2 韌體 <HPG 1.30？）",
                      (now - self._last_data) / 1e9)
        if self._dirty and now - self._last_fsync >= self._fsync_ns:
            self._f.fsync()
            self.fsyncs += 1
            self._dirty = False
            self._last_fsync = now
        return len(data)

    def run(self, stop_event) -> None:
        while not stop_event.is_set():
            self.step()

    def close(self) -> None:
        if self._f is not None:
            self._f.close()
            self._f = None
        log.info("GNSS：%d bytes、fsync %d 次、停流 %d 次", self.bytes, self.fsyncs, self.stall_events)


class SerialStream:
    """pyserial（serial.Serial(port, baudrate, timeout=…)）。只在實機用；測試用假串流。"""

    def __init__(self, device: str, baudrate: int, timeout_s: float = 0.2):
        self.device, self.baudrate, self.timeout_s = device, baudrate, timeout_s
        self._s = None

    def open(self) -> None:
        import serial                             # 只在需要時 import

        self._s = serial.Serial(self.device, self.baudrate, timeout=self.timeout_s)

    def read(self, n: int) -> bytes:
        return self._s.read(n)

    def write(self, data: bytes) -> None:
        self._s.write(data)

    def flush(self) -> None:
        self._s.flush()

    def close(self) -> None:
        if self._s is not None:
            self._s.close()
            self._s = None
