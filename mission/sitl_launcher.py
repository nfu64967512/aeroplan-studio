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
}

# Mission Planner 內建預設參數檔
_DEFAULT_PARAMS = {
    'PLANE':  'default_params/plane.parm',
    'COPTER': 'default_params/copter.parm',
}


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
        model = 'plane' if vehicle == 'PLANE' else 'quad'

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
