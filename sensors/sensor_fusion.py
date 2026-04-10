"""
感測器融合引擎
整合 GPS, IMU, VIO (Kimera) 數據，輸出統一狀態估計
"""

import numpy as np
import time
from dataclasses import dataclass
from threading import Lock


@dataclass
class VehicleState:
    """標準化飛行器狀態向量，供規劃器使用"""
    x: float = 0.0          # Local X (meters)
    y: float = 0.0          # Local Y (meters)
    z: float = 0.0          # Local Z (meters) - Height
    yaw: float = 0.0        # Heading (rad)
    v_x: float = 0.0        # Linear velocity X (m/s)
    v_y: float = 0.0        # Linear velocity Y (m/s)
    v_z: float = 0.0        # Linear velocity Z (m/s)
    yaw_rate: float = 0.0   # Angular velocity Z (rad/s)
    timestamp: float = 0.0  # Unix timestamp


class SensorFusionEngine:
    """
    感測器融合引擎
    整合 GPS, IMU, VIO (Kimera) 數據，輸出統一狀態估計
    """

    def __init__(self):
        self._state = VehicleState()
        self._lock = Lock()
        self.use_vio = False  # 標記是否啟用視覺里程計

    def update_from_mavlink(self, msg_dict: dict):
        """
        處理來自 MAVLink (Pixhawk/ArduPilot) 的數據
        例如: GLOBAL_POSITION_INT, ATTITUDE
        """
        with self._lock:
            # 這裡需要接入 coordinate.py 進行 WGS84 -> Local 轉換
            # 暫時僅更新速度與姿態
            if 'vx' in msg_dict:
                self._state.v_x = msg_dict['vx'] / 100.0  # cm/s -> m/s
            if 'vy' in msg_dict:
                self._state.v_y = msg_dict['vy'] / 100.0
            if 'yaw' in msg_dict:
                self._state.yaw = msg_dict['yaw']  # rad
            self._state.timestamp = time.time()

    def update_from_vio(self, pose_matrix: np.ndarray, covariance: np.ndarray = None):
        """
        API 接口：接收來自 Kimera VIO 的位姿更新 (Edge Computing)

        Args:
            pose_matrix: 4x4 變換矩陣 (從 VIO 模組傳入)
            covariance: 協方差矩陣（可選）
        """
        with self._lock:
            self.use_vio = True
            # 解析旋轉矩陣與位移向量
            x, y, z = pose_matrix[0, 3], pose_matrix[1, 3], pose_matrix[2, 3]

            # 從旋轉矩陣提取 Yaw (假設 Z 為上)
            r11, r21 = pose_matrix[0, 0], pose_matrix[1, 0]
            yaw = np.arctan2(r21, r11)

            self._state.x = x
            self._state.y = y
            self._state.z = z
            self._state.yaw = yaw
            self._state.timestamp = time.time()

    def get_state(self) -> VehicleState:
        """獲取當前最新的線程安全狀態 (供 DWA Planner 使用)"""
        with self._lock:
            # 返回副本以防外部修改
            return VehicleState(**self._state.__dict__)

    def predict_state(self, dt: float) -> VehicleState:
        """
        簡單的運動學預測 (若傳感器數據延遲)
        """
        with self._lock:
            s = self._state
            new_x = s.x + s.v_x * dt
            new_y = s.y + s.v_y * dt
            new_yaw = s.yaw + s.yaw_rate * dt

            # 正規化 Yaw
            new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi

            return VehicleState(
                x=new_x, y=new_y, z=s.z, yaw=new_yaw,
                v_x=s.v_x, v_y=s.v_y, v_z=s.v_z, yaw_rate=s.yaw_rate,
                timestamp=time.time()
            )
