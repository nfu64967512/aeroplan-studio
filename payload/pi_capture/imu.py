"""原始 IMU 紀錄 imu.csv（F1）

欄位：t_ns,ax,ay,az,gx,gy,gz,temp_c —— t_ns 是 CLOCK_MONOTONIC（與 frames.jsonl 的 edge_ns 同一個時鐘），
加速度 m/s²、角速度 rad/s、溫度 °C。軸向是感測器本體軸，與相機／機身的對應由安裝決定（地面端處理）。

用途只有診斷（振動與模糊、相機對機身的相對運動、日後滾動快門陀螺建模），**不參與地理標記**：
MEMS 姿態誤差 1.5–3.5° 在 100 m 高就是地面 2.6–6.1 m（評估 §3）。

實機後端 Icm42688Spi ⚠ 硬體未驗證：
- 暫存器位址與 WHO_AM_I 值 0x47、SPI 讀取旗標 0x80，與 ArduPilot AP_InertialSensor_Invensensev3.cpp
  （INV3REG_WHOAMI 0x75、PWR_MGMT0 0x4e、GYRO_CONFIG0 0x4f、ACCEL_CONFIG0 0x50、DEVICE_CONFIG 0x11、
  BANK_SEL 0x76、INV3_ID_ICM42688_P 0x47、BIT_READ_FLAG 0x80）交叉核對過；
- 資料暫存器 TEMP_DATA1=0x1D 起連續 14 bytes（溫度、加速度 XYZ、陀螺 XYZ，大端序）、
  溫度 raw/132.48+25、FS 編碼與 ODR 編碼取自 TDK DS-000347 的記憶，**本次沒有對原文核對**。
- BMI088 後端沒有做（兩顆晶片分開 CS，台架選定感測器後再補）。
- 400 Hz 用輪詢＋時間戳（讀取當下的 MONOTONIC），抖動受 Linux 排程影響；要更準應改用 FIFO＋硬體時間戳。
"""
from __future__ import annotations

import logging
import math
import struct
import time
from dataclasses import dataclass

log = logging.getLogger("pi_capture.imu")

IMU_CSV = "imu.csv"
IMU_HEADER = "t_ns,ax,ay,az,gx,gy,gz,temp_c"
G0 = 9.80665                      # 標準重力 m/s²


@dataclass(frozen=True)
class ImuSample:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    temp_c: float | None


def format_row(t_ns: int, s: ImuSample) -> str:
    vals = [s.ax, s.ay, s.az, s.gx, s.gy, s.gz]
    temp = "" if s.temp_c is None else format(float(s.temp_c), ".6g")
    return ",".join([str(int(t_ns))] + [format(float(v), ".9g") for v in vals] + [temp])


class ImuLogger:
    """固定頻率輪詢 backend.read()，每列附讀取當下的 MONOTONIC 時間戳，每 fsync_period_s 秒 fsync 一次。

    step() 是一次取樣（測試直接呼叫）；run() 是實機迴圈。讀取失敗只計數＋日誌，不中斷擷取。
    """

    def __init__(self, storage, backend, clock, *, rate_hz: float = 400.0,
                 fsync_period_s: float = 1.0):
        if not (isinstance(rate_hz, (int, float)) and 0 < rate_hz <= 2000):
            raise ValueError(f"rate_hz 必須在 (0, 2000]，收到 {rate_hz!r}")
        if not (isinstance(fsync_period_s, (int, float)) and fsync_period_s > 0):
            raise ValueError(f"fsync_period_s 必須 > 0，收到 {fsync_period_s!r}")
        self.storage = storage
        self.backend = backend
        self.clock = clock
        self.rate_hz = float(rate_hz)
        self._period_ns = int(round(1e9 / rate_hz))
        self._fsync_ns = int(round(fsync_period_s * 1e9))
        self._f = None
        self._last_fsync = None
        self.samples = 0
        self.read_errors = 0
        self.overruns = 0

    def open(self) -> None:
        if self.storage.path(IMU_CSV).exists():
            raise FileExistsError(f"{IMU_CSV} 已存在：絕不覆寫既有紀錄")
        self._f = self.storage.open_append(IMU_CSV)
        self._f.append_line(IMU_HEADER)
        self._last_fsync = self.clock.monotonic_ns()

    def step(self) -> bool:
        try:
            s = self.backend.read()
        except Exception:                        # noqa: BLE001 —— IMU 壞了不能拖垮相機
            self.read_errors += 1
            if self.read_errors in (1, 10, 100) or self.read_errors % 1000 == 0:
                log.error("IMU 讀取失敗（累計 %d 次）", self.read_errors, exc_info=True)
            return False
        t = self.clock.monotonic_ns()
        self._f.write((format_row(t, s) + "\n").encode("ascii"))
        self.samples += 1
        if t - self._last_fsync >= self._fsync_ns:
            self._f.fsync()
            self._last_fsync = t
        return True

    def run(self, stop_event) -> None:
        nxt = time.monotonic_ns()
        while not stop_event.is_set():
            self.step()
            nxt += self._period_ns
            delay = nxt - time.monotonic_ns()
            if delay > 0:
                time.sleep(delay / 1e9)
            elif -delay > self._period_ns:
                self.overruns += 1
                nxt = time.monotonic_ns()          # 落後超過一個週期就重新對齊，不補讀

    def close(self) -> None:
        if self._f is not None:
            self._f.close()
            self._f = None
        log.info("IMU：%d 筆、讀取失敗 %d、超時 %d", self.samples, self.read_errors, self.overruns)


# ── ICM-42688-P（SPI）⚠ 硬體未驗證 ─────────────────────────────────────────────

ICM_WHO_AM_I = 0x75
ICM_WHO_AM_I_VALUE = 0x47
ICM_DEVICE_CONFIG = 0x11
ICM_PWR_MGMT0 = 0x4E
ICM_GYRO_CONFIG0 = 0x4F
ICM_ACCEL_CONFIG0 = 0x50
ICM_BANK_SEL = 0x76
ICM_TEMP_DATA1 = 0x1D
ICM_READ = 0x80
ICM_GYRO_LSB_PER_DPS = 16.4       # FS ±2000 dps
ICM_ACCEL_LSB_PER_G = 2048.0      # FS ±16 g
ICM_TEMP_LSB_PER_C = 132.48
ICM_TEMP_OFFSET_C = 25.0


def icm42688_convert(raw: bytes) -> ImuSample:
    """TEMP_DATA1 起 14 bytes（大端序 int16 ×7）→ SI 單位。"""
    if len(raw) != 14:
        raise ValueError(f"需要 14 bytes，收到 {len(raw)}")
    t, ax, ay, az, gx, gy, gz = struct.unpack(">7h", raw)
    a = G0 / ICM_ACCEL_LSB_PER_G
    g = math.radians(1.0) / ICM_GYRO_LSB_PER_DPS
    return ImuSample(ax * a, ay * a, az * a, gx * g, gy * g, gz * g,
                     t / ICM_TEMP_LSB_PER_C + ICM_TEMP_OFFSET_C)


class Icm42688Spi:
    """py-spidev（SpiDev.open/xfer2/max_speed_hz/mode）讀 ICM-42688-P。⚠ 硬體未驗證。"""

    def __init__(self, bus: int = 0, device: int = 0, speed_hz: int = 8_000_000):
        self.bus, self.device, self.speed_hz = bus, device, speed_hz
        self._spi = None

    def open(self) -> None:
        import spidev                             # 只在 Pi 上有

        spi = spidev.SpiDev()
        spi.open(self.bus, self.device)
        spi.max_speed_hz = self.speed_hz
        spi.mode = 0b11
        self._spi = spi
        self._write(ICM_BANK_SEL, 0x00)
        self._write(ICM_DEVICE_CONFIG, 0x01)                  # 軟體重置
        time.sleep(0.01)
        who = self._read(ICM_WHO_AM_I, 1)[0]
        if who != ICM_WHO_AM_I_VALUE:
            raise RuntimeError(f"WHO_AM_I=0x{who:02x}，不是 ICM-42688-P（0x47）")
        self._write(ICM_GYRO_CONFIG0, 0x06)                   # ±2000 dps、ODR 1 kHz
        self._write(ICM_ACCEL_CONFIG0, 0x06)                  # ±16 g、ODR 1 kHz
        self._write(ICM_PWR_MGMT0, 0x0F)                      # 陀螺＋加速度計 low-noise 模式
        time.sleep(0.05)

    def _write(self, reg: int, val: int) -> None:
        self._spi.xfer2([reg & 0x7F, val & 0xFF])

    def _read(self, reg: int, n: int) -> bytes:
        return bytes(self._spi.xfer2([reg | ICM_READ] + [0] * n)[1:])

    def read(self) -> ImuSample:
        return icm42688_convert(self._read(ICM_TEMP_DATA1, 14))

    def close(self) -> None:
        if self._spi is not None:
            self._spi.close()
            self._spi = None
