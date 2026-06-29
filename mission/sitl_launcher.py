"""
SITL 本地啟動器
直接執行專案內 sitl/ 資料夾的 ArduPilot SITL binary（從 Mission Planner 複製過來）。

對外提供：
    SITLLauncher.start(vehicle, lat, lon, alt, heading, speedup) → tcp port
    SITLLauncher.stop()
    SITLLauncher.is_running()
"""

from __future__ import annotations

import sys
import socket
import subprocess
import time
from pathlib import Path
from typing import Optional, List

from utils.logger import get_logger

logger = get_logger()

# 專案根目錄下的 sitl 資料夾
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_SITL_DIR     = _PROJECT_ROOT / 'sitl'


# ──────────────────────────────────────────────────────────────────────
# 網路 / 防火牆 helper（嵌入式 fan-out 用）
# ──────────────────────────────────────────────────────────────────────
def _get_lan_ip() -> str:
    """取得 Windows 主機 outbound LAN IPv4。
    用 UDP socket 假裝連往外部位址（不真送封包）讓 OS 自動選擇
    對應的網卡 IP，比列舉 adapter 更可靠。失敗時 fallback 127.0.0.1。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        return '127.0.0.1'
    finally:
        s.close()


def _firewall_rule_exists(rule_name: str) -> bool:
    """檢查 Windows Firewall 是否已有指定名稱的規則。
    用 `netsh advfirewall firewall show rule name=...` 的 returncode 判斷
    （0=存在，1=不存在）。非 Windows / 失敗一律視為「不存在」。
    """
    if sys.platform != 'win32':
        return False
    try:
        result = subprocess.run(
            ['netsh', 'advfirewall', 'firewall', 'show', 'rule', f'name={rule_name}'],
            capture_output=True, text=True, timeout=5,
        )
        return result.returncode == 0
    except Exception:
        return False


def _tcp_port_listener_pid(port: int) -> Optional[int]:
    """回傳目前 LISTEN 該 TCP port 的 PID；無人佔用 → None。

    Windows 用 PowerShell `Get-NetTCPConnection`；非 Windows 一律回 None。
    """
    if sys.platform != 'win32':
        return None
    try:
        result = subprocess.run(
            [
                'powershell', '-NoProfile', '-Command',
                f'(Get-NetTCPConnection -LocalPort {int(port)} '
                f'-State Listen -ErrorAction SilentlyContinue '
                f'| Select-Object -First 1 -ExpandProperty OwningProcess)',
            ],
            capture_output=True, text=True, timeout=4,
        )
        out = (result.stdout or '').strip()
        if not out:
            return None
        return int(out.splitlines()[0])
    except Exception:
        return None


def _process_name_by_pid(pid: int) -> str:
    """回傳 PID 對應的 process 名稱；失敗回 'unknown'。"""
    if sys.platform != 'win32':
        return 'unknown'
    try:
        result = subprocess.run(
            ['powershell', '-NoProfile', '-Command',
             f'(Get-Process -Id {int(pid)} -ErrorAction SilentlyContinue).Name'],
            capture_output=True, text=True, timeout=4,
        )
        return (result.stdout or '').strip() or 'unknown'
    except Exception:
        return 'unknown'


def _find_free_sitl_instance(
    start_instance: int = 0,
    max_instance: int = 16,
    reserved: Optional[set] = None,
) -> int:
    """從 start_instance 起找第一個 TCP port 沒人 LISTEN 且不在 reserved 內的 instance。

    對應 port 公式：5760 + 10 * instance

    reserved: 由 launcher 自己追蹤的「本次批次已配發過」instance 編號集合。
              即使該 port 還沒被 SITL bind（race window），也跳過，避免重派。

    全部都被占用時直接回 start_instance（不阻擋啟動，後續 spawn 自然會失敗
    並由 ArduPilot 印 bind error）。
    """
    reserved = reserved or set()
    for inst in range(start_instance, start_instance + max_instance):
        if inst in reserved:
            continue
        port = SITLLauncher.tcp_port_for(inst)
        if _tcp_port_listener_pid(port) is None:
            return inst
    return start_instance


def _ensure_firewall_rule(port: int, proto: str, rule_name: str) -> bool:
    """確保 Windows Firewall 入站允許規則存在。
    若已存在則跳過；不存在則以 UAC 提權執行 netsh 新增規則。

    參數:
        port: 要開放的埠號
        proto: 'TCP' 或 'UDP'
        rule_name: 規則名稱（用來檢查與識別，建議用 AeroPlanStudio_SITL_<PROTO>_<PORT>）
    回傳:
        True = 規則已存在或成功建立；False = 新增失敗（log 警告，不擋啟動）
    """
    if sys.platform != 'win32':
        return True  # 非 Windows 平台直接放行
    if _firewall_rule_exists(rule_name):
        logger.info(f'[SITL Firewall] 規則已存在，跳過: {rule_name}')
        return True
    try:
        # 用 ShellExecuteW + 'runas' verb 觸發 UAC 提權
        # 參數一次帶完整 netsh advfirewall firewall add rule ...
        import ctypes
        params = (
            f'advfirewall firewall add rule name="{rule_name}" '
            f'dir=in action=allow protocol={proto} localport={port} '
            f'profile=any enable=yes'
        )
        # ShellExecuteW returns > 32 on success
        ret = ctypes.windll.shell32.ShellExecuteW(
            None, 'runas', 'netsh', params, None, 0,  # 0 = SW_HIDE
        )
        if int(ret) > 32:
            logger.info(f'[SITL Firewall] 已新增規則 {rule_name} ({proto}/{port})')
            return True
        logger.warning(
            f'[SITL Firewall] 新增規則失敗 (ShellExecuteW ret={ret})，'
            f'請手動執行：netsh {params}'
        )
        return False
    except Exception as e:
        logger.warning(
            f'[SITL Firewall] 新增規則例外: {e}\n'
            f'    若嵌入式連不上 SITL，請手動在 Windows Defender 新增 {proto}/{port} 入站允許'
        )
        return False

# 飛行器類型對應 binary
_VEHICLE_BINARIES = {
    'PLANE':  'ArduPlane.exe',
    'COPTER': 'ArduCopter.exe',
    'VTOL':   'ArduPlane.exe',       # QuadPlane 基於 ArduPlane 核心
}

# Mission Planner 內建預設參數檔
_DEFAULT_PARAMS = {
    'PLANE':  'default_params/plane.parm',
    'COPTER': 'default_params/copter.parm',
    'VTOL':   'default_params/plane.parm',  # 基礎用 plane，再疊加 VTOL 參數
}


def generate_vtol_params(output_path: Optional[Path] = None) -> Path:
    """產生 VTOL (QuadPlane) 專用參數檔 — 基於 Alti Transition 真機參數。

    包含完整的 QuadPlane 啟用、轉換、SERVO 通道映射、飛行動態參數，
    讓 SITL 模擬行為貼近真機。回傳檔案路徑，供 SITL --defaults 使用。
    """
    if output_path is None:
        output_path = _SITL_DIR / 'default_params' / 'vtol_default.parm'
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 基於 Alti Transition 真機參數檔 (Alti.param) 提取的完整 QuadPlane 參數
    content = """\
# ══════════════════════════════════════════════════════════════════
# VTOL (QuadPlane) 預設參數 — 參考 Alti Transition 真機參數
# 用途：SITL --defaults 疊加於 plane.parm 之上
# ══════════════════════════════════════════════════════════════════

# ── 1. QuadPlane 核心啟用 ──────────────────────────────────────
Q_ENABLE         1          # 啟用 QuadPlane 功能（必須=1，否則純固定翼）
Q_FRAME_CLASS    1          # 1=Quad（四軸）
Q_FRAME_TYPE     1          # 1=X 型佈局

# ── 2. SERVO 通道映射（SITL quadplane 預設通道）──────────────
# SERVO1~4: 固定翼舵面（plane.parm 已定義）
# SERVO5~8: 四軸馬達（QuadPlane 必須正確映射）
SERVO5_FUNCTION  33         # Motor1（右前）
SERVO5_MIN       1000
SERVO5_MAX       2000
SERVO5_TRIM      1500
SERVO6_FUNCTION  34         # Motor2（左後）
SERVO6_MIN       1000
SERVO6_MAX       2000
SERVO6_TRIM      1500
SERVO7_FUNCTION  35         # Motor3（左前）
SERVO7_MIN       1100
SERVO7_MAX       1900
SERVO7_TRIM      1500
SERVO8_FUNCTION  36         # Motor4（右後）
SERVO8_MIN       1100
SERVO8_MAX       1900
SERVO8_TRIM      1500

# ── 3. 轉換 (Transition) 參數 ─────────────────────────────────
Q_TRANSITION_MS  3000       # FW→MC 轉換時間 3 秒
Q_BACKTRANS_MS   3000       # MC→FW 反向轉換時間 3 秒
Q_TRANS_DECEL    2.0        # 轉換減速率 (m/s²)
Q_TRAN_PIT_MAX   3          # 轉換期間最大俯仰角 (度)
Q_TRANS_FAIL     0          # 轉換失敗處理：0=繼續
Q_ASSIST_SPEED   16.0       # 空速低於 16m/s 時四軸馬達自動輔助
Q_ASSIST_ALT     0          # 高度輔助：0=關閉
Q_ASSIST_ANGLE   35         # 傾斜角>35° 時四軸輔助
Q_ASSIST_DELAY   0.5        # 輔助啟動延遲 (秒)

# ── 4. 固定翼空速設定 ─────────────────────────────────────────
AIRSPEED_CRUISE  24         # 巡航空速 24 m/s
AIRSPEED_MIN     20         # 最低空速 20 m/s（低於此→失速）
AIRSPEED_MAX     34         # 最高空速 34 m/s
TRIM_THROTTLE    50         # 平飛油門 50%
STALL_PREVENTION 1          # 啟用失速保護

# ── 5. 多旋翼飛行動態 ─────────────────────────────────────────
Q_ANGLE_MAX      3000       # 最大傾斜角 30°
Q_ACCEL_Z        250        # 垂直加速度 250 cm/s²
Q_VELZ_MAX       250        # 最大垂直速度 250 cm/s (2.5m/s)
Q_VELZ_MAX_DN    0          # 下降速度：0=與上升相同
Q_LOIT_SPEED     250        # QLOITER 水平速度 250 cm/s
Q_LOIT_ACC_MAX   250        # QLOITER 最大加速度
Q_LOIT_ANG_MAX   15         # QLOITER 最大角度 15°
Q_LOIT_BRK_ACCEL 50         # QLOITER 煞車加速度
Q_LOIT_BRK_DELAY 1.0        # QLOITER 煞車延遲
Q_LOIT_BRK_JERK  250        # QLOITER 煞車 jerk
Q_WP_SPEED       500        # 多旋翼航點速度 500 cm/s (5m/s)
Q_WP_SPEED_DN    150        # 下降速度 150 cm/s
Q_WP_SPEED_UP    250        # 上升速度 250 cm/s
Q_WP_ACCEL       100        # 航點加速度
Q_WP_ACCEL_Z     100        # 垂直加速度
Q_WP_RADIUS      200        # 航點到達半徑 200 cm

# ── 6. 降落參數 ───────────────────────────────────────────────
Q_LAND_SPEED     50         # 降落速度 50 cm/s
Q_LAND_FINAL_ALT 5.0        # 最終進場高度 5m（切換為定速下降）
Q_LAND_ICE_CUT   1          # 降落時關閉油引擎

# ── 7. RTL / 安全 ────────────────────────────────────────────
Q_RTL_ALT        25         # QRTL 回航高度 25m
Q_RTL_MODE       3          # QRTL 模式：3=先飛回再垂直降落
Q_OPTIONS        0          # QuadPlane 選項位元遮罩

# ── 8. 馬達參數 ───────────────────────────────────────────────
Q_M_PWM_MIN      1000       # 馬達 PWM 最小值
Q_M_PWM_MAX      2000       # 馬達 PWM 最大值
Q_M_SPIN_ARM     0.1        # ARM 時馬達轉速比例
Q_M_SPIN_MIN     0.15       # 飛行中馬達最低轉速
Q_M_SPIN_MAX     0.95       # 馬達最高轉速
Q_M_THST_HOVER   0.34       # 懸停油門比例（Alti 實測值）
Q_M_THST_EXPO    0.65       # 油門曲線指數
Q_M_SPOOL_TIME   0.25       # 馬達加速時間 (秒)
Q_M_HOVER_LEARN  2          # 自動學習懸停油門
Q_FWD_MANTHR_MAX 30         # QLOITER 前進油門上限 30%

# ── 9. 姿態控制 PID（Alti 調校值）────────────────────────────
Q_A_ANG_PIT_P    4.5
Q_A_ANG_RLL_P    4.5
Q_A_ANG_YAW_P    1.52
Q_A_RAT_PIT_P    0.3
Q_A_RAT_PIT_I    0.3
Q_A_RAT_PIT_D    0.01
Q_A_RAT_RLL_P    0.2
Q_A_RAT_RLL_I    0.2
Q_A_RAT_RLL_D    0.002
Q_A_RAT_YAW_P    2.0
Q_A_RAT_YAW_I    0.18
Q_A_RAT_YAW_D    0.0
Q_A_RATE_FF_ENAB 1          # 啟用前饋控制
Q_A_SLEW_YAW     1500       # 偏航 slew rate

# ── 10. 位置控制 PID ──────────────────────────────────────────
Q_P_POSXY_P      1.0
Q_P_POSZ_P       1.0
Q_P_VELXY_P      1.4
Q_P_VELXY_I      0.7
Q_P_VELXY_D      0.35
Q_P_VELZ_P       5.0
Q_P_ACCZ_P       0.3
Q_P_ACCZ_I       1.0

# ── 11. 固定翼導航（L1）─────────────────────────────────────
NAVL1_PERIOD     17         # L1 導航週期（越大越平滑）
NAVL1_DAMPING    0.75       # L1 阻尼

# ── 12. TECS 能量控制 ────────────────────────────────────────
TECS_CLMB_MAX    5.0        # 最大爬升率 5m/s
TECS_SINK_MAX    5.0        # 最大下沉率 5m/s
TECS_SINK_MIN    2.0        # 最小下沉率 2m/s
TECS_TIME_CONST  5.0        # 時間常數
TECS_PITCH_MAX   15         # 最大俯仰 15°

# ── 13. 航點 / 限制 ──────────────────────────────────────────
WP_RADIUS        100        # 固定翼航點到達半徑 100m
WP_LOITER_RAD    75         # 盤旋半徑 75m
LIM_PITCH_MAX    3000       # 最大俯仰 30°
LIM_PITCH_MIN    -3000      # 最小俯仰 -30°
LIM_ROLL_CD      4000       # 最大滾轉 40°

# ── 14. SITL 專用覆寫 ────────────────────────────────────────
# 以下參數僅用於 SITL 模擬，降低起飛門檻
ARMING_CHECK     0          # 關閉 pre-arm 檢查（SITL 無真實感測器）
FS_GCS_ENABL     0          # 關閉 GCS 失聯保護（SITL 可能心跳延遲）
FS_SHORT_ACTN    0          # 短失聯不動作
FS_LONG_ACTN     0          # 長失聯不動作
THR_FAILSAFE     0          # 關閉油門失聯保護
"""
    try:
        output_path.write_text(content, encoding='utf-8')
        logger.info(f'[SITL] 已產生 VTOL 參數檔（基於 Alti Transition）: {output_path}')
    except Exception as e:
        logger.error(f'[SITL] 寫入 VTOL 參數檔失敗: {e}')
    return output_path


def start_vtol_sitl(launcher: 'SITLLauncher',
                    lat: float = 23.7, lon: float = 121.0,
                    alt: float = 100.0, heading: float = 0.0,
                    speedup: float = 1.0, instance: int = 0) -> str:
    """啟動 VTOL (QuadPlane) SITL 的便捷函式。

    內部流程：
      1. 動態產生 vtol_default.parm（Q_ENABLE=1, Q_FRAME_CLASS=1 ...）
      2. 使用 ArduPlane.exe 核心 + quadplane 物理模型
         ─ 等效於 sim_vehicle.py -v ArduPlane -f quadplane
         ─ SITL 命令列：ArduPlane.exe --model quadplane --home ... --defaults plane.parm,vtol_default.parm,...
      3. 回傳 TCP 連線字串供 MAVLink 建立連線

    ArduPilot VTOL 架構說明：
      ● QuadPlane 基於 ArduPlane 固定翼韌體，不是 ArduCopter
      ● --model quadplane 告訴 SITL 物理引擎模擬「固定翼+四軸馬達」組合
      ● Q_ENABLE=1 → 飛控啟用 QuadPlane 邏輯（QHOVER/QLOITER/QLAND/QRTL）
      ● 飛行模式轉換透過 MAV_CMD_DO_VTOL_TRANSITION (cmd=3000)
        state=3 → 多旋翼 (MC)    state=4 → 固定翼 (FW)
    """
    # 1) 產生 VTOL 參數檔
    vtol_parm = generate_vtol_params()
    logger.info(
        f'[VTOL SITL] 動態產生 vtol_default.parm:\n'
        f'  Q_ENABLE=1        (啟用 QuadPlane)\n'
        f'  Q_FRAME_CLASS=1   (Quad 四軸)\n'
        f'  Q_FRAME_TYPE=1    (X 型佈局)\n'
        f'  Q_ASSIST_SPEED=0  (關閉輔助馬達)\n'
        f'  Q_RTL_MODE=1      (QRTL 先飛回再降落)\n'
        f'  路徑: {vtol_parm}'
    )

    # 2) 呼叫通用 start()，vehicle='VTOL' 會自動選擇：
    #    binary  = ArduPlane.exe（固定翼核心）
    #    model   = quadplane（四軸+固定翼物理引擎）
    #    defaults = plane.parm + vtol_default.parm + identity.parm
    logger.info(
        f'[VTOL SITL] 使用 ArduPlane 核心模擬 VTOL QuadPlane\n'
        f'  Binary: ArduPlane.exe\n'
        f'  Model:  quadplane  (等效 sim_vehicle.py -v ArduPlane -f quadplane)\n'
        f'  Instance: {instance}\n'
        f'  Home: {lat},{lon},{alt},{heading}'
    )
    conn_str = launcher.start(
        vehicle='VTOL', lat=lat, lon=lon, alt=alt,
        heading=heading, speedup=speedup, instance=instance,
    )
    logger.info(f'[VTOL SITL] VTOL SITL 已啟動 → {conn_str}')
    return conn_str


class SITLLauncher:
    """ArduPilot SITL 子行程啟動器"""

    def __init__(self):
        self._procs: list = []      # 多實例 [(proc, vehicle, instance_id, conn_str), ...]
        self._vehicle: str = ''
        # 本 launcher 已派出的 instance 編號（不論 TCP 是否已 bind）。
        # 用來防止 start_multi 連續呼叫 start() 時，前一台 SITL 還沒完成 bind
        # 就被下一輪 port-check 誤判為 free，導致兩台搶同一個 port。
        self._allocated_instances: set[int] = set()

    @property
    def _proc(self):
        return self._procs[0][0] if self._procs else None

    # ── 連接埠 / 實例設定的單一真相來源 (0-8 去重) ──────────────
    @staticmethod
    def tcp_port_for(instance: int) -> int:
        """ArduPilot SITL 慣例：instance N 的 SERIAL0 TCP 埠 = 5760 + 10·N。

        參數
        ----
        instance : int
            SITL 實例編號（0-based）。

        回傳
        ----
        int：對應 TCP 監聽埠（i0→5760, i1→5770, i2→5780, ...）。
        """
        return 5760 + 10 * instance

    @staticmethod
    def default_instance_configs(count: int) -> List[dict]:
        """產生 N 台 SITL 的預設 instance 設定（CLI 與啟動對話框共用）。

        每台：sysid = i+1（對齊 MAVROS tgt_system）、embedded_ip 預設空字串、
        embedded_port = 14550 + 10·i（SITL --serial1=udpin 被動監聽埠）。

        參數
        ----
        count : int
            SITL 台數。

        回傳
        ----
        List[dict]：長度 count 的設定串列。
        """
        return [
            {"sysid": i + 1, "embedded_ip": "", "embedded_port": 14550 + 10 * i}
            for i in range(count)
        ]

    # ── 路徑檢查 ──────────────────────────────────────────────────────
    @classmethod
    def sitl_dir(cls) -> Path:
        return _SITL_DIR

    @classmethod
    def is_available(cls) -> bool:
        """檢查專案內 SITL binary 是否存在"""
        return (
            _SITL_DIR.exists()
            and (_SITL_DIR / 'ArduPlane.exe').exists()
            and (_SITL_DIR / 'ArduCopter.exe').exists()
        )

    @classmethod
    def list_vehicles(cls) -> list:
        return list(_VEHICLE_BINARIES.keys())

    # ── 啟動 ──────────────────────────────────────────────────────────
    def start(self,
              vehicle: str = 'PLANE',
              lat: float = 23.7,
              lon: float = 121.0,
              alt: float = 100.0,
              heading: float = 0.0,
              speedup: float = 1.0,
              instance: int = 0,
              sysid: Optional[int] = None,
              extra_outputs: Optional[List[str]] = None,
              auto_firewall: bool = False) -> str:
        """
        啟動單一 SITL 子行程實例。
        instance N → TCP port 5760+10*N（與 ArduPilot 慣例一致）
        回傳連線字串，例如 'tcp:127.0.0.1:5760'（給 Windows 端 GCS 用）。

        參數:
            sysid:         覆寫 SYSID_THISMAV（給嵌入式 MAVROS tgt_system 對齊用）
                           不指定則沿用舊行為 = instance+1
            extra_outputs: SERIAL1+ 額外 MAVLink 出口 URI 列表，
                           例如 ['udpclient:192.168.1.50:14550']
                           會依序綁到 --serial1 / --serial2 / ...
            auto_firewall: True 時於 Popen 前嘗試新增 Windows Firewall 入站規則
                           （TCP SERIAL0 + 所有 udpclient 目標 port 的 UDP）
        """
        vehicle = vehicle.upper()
        if vehicle not in _VEHICLE_BINARIES:
            raise ValueError(f'未知飛行器類型: {vehicle}')

        if not self.is_available():
            raise FileNotFoundError(
                f'找不到 SITL binary，預期在 {_SITL_DIR}\n'
                f'請從 Mission Planner sitl 資料夾複製過來'
            )

        # ── Port 衝突 / 同 batch 重派檢查 ──────────────────────────────
        # 1) Windows 上常見：VS Code 之類已 LISTEN 127.0.0.1:5760，
        #    SITL bind 0.0.0.0:5760 仍能 listen，但 client 連 127.0.0.1:5760
        #    會走到佔用者而非 SITL → 收不到 heartbeat。
        # 2) start_multi 連續呼叫時，前一台 SITL 還沒完成 bind，下一輪
        #    port-check 可能誤判 free 而重派同一個 instance → bind 撞車。
        #    用 `_allocated_instances` 防止此種 race。
        requested_port = self.tcp_port_for(instance)
        listener_pid = _tcp_port_listener_pid(requested_port)
        in_batch_collision = instance in self._allocated_instances
        if listener_pid is not None or in_batch_collision:
            if in_batch_collision and listener_pid is None:
                reason = (
                    f'本批次已配發 instance {instance}（SITL 尚未完成 bind）'
                )
            else:
                holder = _process_name_by_pid(listener_pid) if listener_pid else 'unknown'
                reason = f'port {requested_port} 已被 {holder} (PID={listener_pid}) 佔用'
            new_inst = _find_free_sitl_instance(
                instance + 1, max_instance=16,
                reserved=self._allocated_instances,
            )
            new_port = self.tcp_port_for(new_inst)
            logger.warning(
                f'[SITL Launcher] {reason}，自動 shift instance '
                f'{instance} → {new_inst} (port {new_port})。'
            )
            instance = new_inst
        self._allocated_instances.add(instance)

        binary = _SITL_DIR / _VEHICLE_BINARIES[vehicle]
        param_file = _SITL_DIR / _DEFAULT_PARAMS[vehicle]

        # 根據載具類型挑選對應的物理模型（這是關鍵！）
        # '+' 是四軸 X 型 → 用在 ArduPlane.exe 會造成物理模擬錯誤 → 一起飛就翻滾
        # VTOL (QuadPlane) 用 'quadplane' 模型 → ArduPlane.exe + 四軸物理
        if vehicle == 'VTOL':
            model = 'quadplane'
        elif vehicle == 'PLANE':
            model = 'plane'
        else:
            model = 'quad'

        # 每個 instance 用獨立工作目錄避免 eeprom 衝突
        work_dir = _SITL_DIR / 'plane' / f'i{instance}'
        work_dir.mkdir(parents=True, exist_ok=True)

        # 生成 identity.parm — 與 Mission Planner 內建 SITL 完全一致
        # 關鍵：SIM_DRIFT_SPEED=0 解除 SITL 預設風漂，避免飛機在地面一直往前滑
        # sysid 若有指定，覆寫 SYSID_THISMAV（嵌入式 MAVROS tgt_system 對齊用）
        effective_sysid = int(sysid) if sysid is not None else (instance + 1)
        identity_file = work_dir / 'identity.parm'
        try:
            identity_file.write_text(
                f'SERIAL0_PROTOCOL=2\n'
                f'SERIAL1_PROTOCOL=2\n'
                f'SYSID_THISMAV={effective_sysid}\n'
                f'SIM_TERRAIN=0\n'
                f'TERRAIN_ENABLE=0\n'
                f'SCHED_LOOP_RATE=50\n'
                f'SIM_RATE_HZ=400\n'
                f'SIM_DRIFT_SPEED=0\n'
                f'SIM_DRIFT_TIME=0\n',
                encoding='utf-8',
            )
        except Exception as _e:
            logger.warning(f'[SITL Launcher] 寫 identity.parm 失敗: {_e}')

        # SITL 命令列：與 Mission Planner sim_vehicle 一致
        cmd = [
            str(binary),
            '--model', model,
            '--home', f'{lat},{lon},{alt},{heading}',
            '--speedup', str(speedup),
            '--instance', str(instance),
        ]
        # sysid 覆寫：--sysid 命令列旗標 + identity.parm 雙保險
        # （SYSID_THISMAV 是 EEPROM 參數，第一次乾淨 boot 才會吃 identity.parm）
        if sysid is not None:
            cmd += ['--sysid', str(int(sysid))]
        # --defaults 用逗號分隔多份檔（與 MP 一致）
        defaults_list = []
        if param_file.exists():
            defaults_list.append(str(param_file))
        else:
            logger.warning(
                f'[SITL Launcher] 找不到預設參數檔 {param_file}'
            )
        # VTOL：額外疊加 QuadPlane 啟用參數
        if vehicle == 'VTOL':
            vtol_parm = generate_vtol_params()
            if vtol_parm.exists():
                defaults_list.append(str(vtol_parm))
        if identity_file.exists():
            defaults_list.append(str(identity_file))
        if defaults_list:
            cmd += ['--defaults', ','.join(defaults_list)]

        # 嵌入式 fan-out：SERIAL1+ 額外 MAVLink 出口
        # SITL 內建 SERIAL0 = tcp:5760+10*instance:wait（Windows GCS 用）
        # 從 SERIAL1 開始疊加 udpclient/tcpclient/udpin/... URI
        fanout_udp_ports: List[int] = []
        if extra_outputs:
            for idx, uri in enumerate(extra_outputs, start=1):
                cmd += [f'--serial{idx}', uri]
                # 解析 udpclient 的目標 port，用來建立防火牆 UDP 入站規則
                # （SITL udpclient socket 同時收嵌入式回送的封包，本端 source port
                #  為 ephemeral，但開放目標 port 的入站允許就足夠雙向通訊）
                if uri.startswith('udpclient:') or uri.startswith('udpin:'):
                    try:
                        port_str = uri.rsplit(':', 1)[1]
                        fanout_udp_ports.append(int(port_str))
                    except (ValueError, IndexError):
                        logger.warning(f'[SITL Launcher] 無法解析 fanout URI port: {uri}')

        # 清除上次的 eeprom.bin / terrain / log，避免殘留壞參數造成 crash-loop
        for junk in ('eeprom.bin', 'mav.parm', 'terrain'):
            p = work_dir / junk
            try:
                if p.is_file():
                    p.unlink()
                elif p.is_dir():
                    import shutil
                    shutil.rmtree(p, ignore_errors=True)
            except Exception:
                pass
        # 注意：不要加 --wipe，它會讓 ArduPlane 反覆 reboot 造成 crash-loop。
        # 前面已刪除 eeprom.bin，SITL 會自動重建乾淨的 EEPROM。

        logger.info(f'[SITL Launcher] 啟動 {vehicle}: {" ".join(cmd)}')
        logger.info(f'[SITL Launcher] 工作目錄: {work_dir}')

        # 在 Popen 之前嘗試新增防火牆規則（首次會跳 UAC，之後存在則略過）
        # 不擋 SITL 啟動：失敗只 log 警告，使用者可手動補規則
        tcp_port_pre = self.tcp_port_for(instance)
        if auto_firewall:
            _ensure_firewall_rule(
                tcp_port_pre, 'TCP',
                f'AeroPlanStudio_SITL_TCP_{tcp_port_pre}',
            )
            for udp_port in fanout_udp_ports:
                _ensure_firewall_rule(
                    udp_port, 'UDP',
                    f'AeroPlanStudio_SITL_UDP_{udp_port}',
                )

        # Windows: 為每個 SITL 實例開獨立 console 視窗
        # （與 Mission Planner SITL 行為一致，方便除錯觀察起飛/墜機原因）
        creationflags = 0
        if sys.platform == 'win32':
            creationflags = (
                subprocess.CREATE_NEW_PROCESS_GROUP
                | subprocess.CREATE_NEW_CONSOLE
            )

        # 注意：使用 CREATE_NEW_CONSOLE 時不要 PIPE stdout，
        # 否則 buffer 滿了會 deadlock，且看不到視窗
        proc = subprocess.Popen(
            cmd,
            cwd=str(work_dir),
            creationflags=creationflags,
        )
        self._vehicle = vehicle

        time.sleep(1.5)

        if proc.poll() is not None:
            raise RuntimeError(
                f'SITL 啟動失敗 (instance {instance}, exit={proc.returncode})\n'
                f'請查看彈出的 console 視窗訊息'
            )

        tcp_port = self.tcp_port_for(instance)
        conn_str = f'tcp:127.0.0.1:{tcp_port}'
        self._procs.append((proc, vehicle, instance, conn_str))
        # 主訊息：簡潔一行給 status bar 用
        logger.info(
            f'[SITL Launcher] {vehicle} 實例 {instance} 已啟動 '
            f'(sysid={effective_sysid}, PID={proc.pid}), {conn_str}'
        )
        # 詳細端點資訊：方便使用者比對嵌入式 MAVROS apm.launch
        if extra_outputs:
            lan_ip = _get_lan_ip()
            for uri in extra_outputs:
                logger.info(f'    嵌入式 fan-out: {uri}')
            logger.info(f'    本機 LAN IP:   {lan_ip}  (apm.launch 對端參考)')
        return conn_str

    def start_multi(self, vehicle: str, count: int,
                    lat: float, lon: float,
                    alt: float = 100.0, heading: float = 0.0,
                    spacing_deg: float = 0.0008,
                    homes: list = None,
                    instance_configs: Optional[List[dict]] = None,
                    auto_firewall: bool = False) -> list:
        """啟動多台 SITL，回傳 [(instance, conn_str), ...]

        參數:
            homes: 可選 [(lat, lon, heading), ...] — 若提供則每架 SITL 依序用
                   該列表指定的座標/航向；否則沿用 lat/lon + spacing_deg 展開
            instance_configs: 來自 SITLLaunchDialog 的每台設定列表，每個 dict 含:
                   sysid / embedded_port
                   （embedded_ip 欄位向下相容保留但不再使用 — SITL 改為 udpin
                    被動模式，嵌入式 MAVROS 為主動 sender）
                   未提供 → 沿用舊行為，無 sysid 覆寫、無 fan-out
            auto_firewall: 是否於啟動前自動新增 Windows Firewall 入站規則
        """
        results = []
        for i in range(count):
            if homes and i < len(homes):
                h = homes[i]
                ilat = float(h[0])
                ilon = float(h[1])
                ihdg = float(h[2]) if len(h) > 2 else heading
            else:
                ilat = lat + i * spacing_deg
                ilon = lon
                ihdg = heading

            # 從 dialog 帶來的 per-instance 設定取出 sysid 與 fan-out port
            # 架構：Windows SITL 被動 listen（udpin），嵌入式 MAVROS 主動 send。
            # 這樣 SITL 啟動時不會卡在嵌入式 ARP/連線，TCP SERIAL0 也不會被拖累。
            sysid_i: Optional[int] = None
            extra_outputs_i: Optional[List[str]] = None
            if instance_configs and i < len(instance_configs):
                cfg = instance_configs[i] or {}
                if cfg.get('sysid') is not None:
                    sysid_i = int(cfg['sysid'])
                port = cfg.get('embedded_port')
                if port:
                    # SITL --serial1=udpin:0.0.0.0:<port> 被動監聽
                    # 對應 MAVROS apm.launch:
                    #   fcu_url:=udp://:<local_port>@<windows_ip>:<port>
                    # MAVROS bind local_port，主動送到 Windows:<port>
                    extra_outputs_i = [f'udpin:0.0.0.0:{int(port)}']

            conn = self.start(
                vehicle=vehicle, lat=ilat, lon=ilon,
                alt=alt, heading=ihdg, instance=i,
                sysid=sysid_i,
                extra_outputs=extra_outputs_i,
                auto_firewall=auto_firewall,
            )
            results.append((i, conn))
            time.sleep(0.5)
        return results

    # ── 停止 ──────────────────────────────────────────────────────────
    def stop(self):
        for proc, vehicle, inst, _ in self._procs:
            if proc.poll() is None:
                logger.info(f'[SITL Launcher] 終止 {vehicle} 實例{inst} PID={proc.pid}')
                try:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=2)
                except Exception as e:
                    logger.error(f'[SITL Launcher] 停止失敗: {e}')
        self._procs = []
        # 同步清空已配發 instance 紀錄，讓下次 start 重新從 0 起算
        self._allocated_instances.clear()

    def is_running(self) -> bool:
        return any(p.poll() is None for p, *_ in self._procs)

    @property
    def vehicle(self) -> str:
        return self._vehicle
