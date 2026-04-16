"""
SITL 本地啟動器
直接執行專案內 sitl/ 資料夾的 ArduPilot SITL binary（從 Mission Planner 複製過來）。

對外提供：
    SITLLauncher.start(vehicle, lat, lon, alt, heading, speedup) → tcp port
    SITLLauncher.stop()
    SITLLauncher.is_running()
"""

from __future__ import annotations

import os
import sys
import subprocess
import time
from pathlib import Path
from typing import Optional

from utils.logger import get_logger

logger = get_logger()

# 專案根目錄下的 sitl 資料夾
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_SITL_DIR     = _PROJECT_ROOT / 'sitl'

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

    @property
    def _proc(self):
        return self._procs[0][0] if self._procs else None

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
              instance: int = 0) -> str:
        """
        啟動單一 SITL 子行程實例。
        instance N → TCP port 5760+10*N（與 ArduPilot 慣例一致）
        回傳連線字串，例如 'tcp:127.0.0.1:5760'
        """
        vehicle = vehicle.upper()
        if vehicle not in _VEHICLE_BINARIES:
            raise ValueError(f'未知飛行器類型: {vehicle}')

        if not self.is_available():
            raise FileNotFoundError(
                f'找不到 SITL binary，預期在 {_SITL_DIR}\n'
                f'請從 Mission Planner sitl 資料夾複製過來'
            )

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
        identity_file = work_dir / 'identity.parm'
        try:
            identity_file.write_text(
                f'SERIAL0_PROTOCOL=2\n'
                f'SERIAL1_PROTOCOL=2\n'
                f'SYSID_THISMAV={instance + 1}\n'
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

        tcp_port = 5760 + 10 * instance
        conn_str = f'tcp:127.0.0.1:{tcp_port}'
        self._procs.append((proc, vehicle, instance, conn_str))
        logger.info(f'[SITL Launcher] {vehicle} 實例 {instance} 已啟動，PID={proc.pid}, {conn_str}')
        return conn_str

    def start_multi(self, vehicle: str, count: int,
                    lat: float, lon: float,
                    alt: float = 100.0, heading: float = 0.0,
                    spacing_deg: float = 0.0008,
                    homes: list = None) -> list:
        """啟動多台 SITL，回傳 [(instance, conn_str), ...]

        homes: 可選 [(lat, lon, heading), ...] — 若提供則每架 SITL 依序用
               該列表指定的座標/航向；否則沿用 lat/lon + spacing_deg 展開
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
            conn = self.start(vehicle=vehicle, lat=ilat, lon=ilon,
                              alt=alt, heading=ihdg, instance=i)
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

    def is_running(self) -> bool:
        return any(p.poll() is None for p, *_ in self._procs)

    @property
    def vehicle(self) -> str:
        return self._vehicle
