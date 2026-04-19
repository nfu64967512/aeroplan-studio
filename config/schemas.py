"""
config.schemas — 強型別參數 Schema (取代散布各處的 dict)

2026 重構：將原本散布在 MainWindow.flight_params / strike_params / coord_plans
等多個 dict 統一為 dataclass + 驗證方法。

為避免增加外部依賴（不引入 pydantic），採用純標準庫 dataclass + __post_init__
驗證。API 近似 pydantic BaseModel：

    * .from_dict(d)    → 從 dict 建立
    * .to_dict()        → 序列化回 dict
    * .validate()       → 主動驗證，失敗 raise ValueError
    * .update(**kwargs) → 部分更新 + 重新驗證

Migration 策略：新程式優先使用這些 dataclass；舊程式透過 `.from_dict`/
`.to_dict` 無痛接軌。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, Optional, Tuple


# ═══════════════════════════════════════════════════════════════════════
#  基礎 Schema 類別
# ═══════════════════════════════════════════════════════════════════════

class _BaseSchema:
    """所有 schema 的基底，提供 dict 互轉與驗證介面"""

    def to_dict(self) -> Dict[str, Any]:
        """序列化為 dict (asdict 會遞迴處理 nested dataclass)"""
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]):
        """從 dict 建立；忽略未知 key (向前相容)"""
        valid_names = {f.name for f in fields(cls)}
        filtered = {k: v for k, v in (d or {}).items() if k in valid_names}
        return cls(**filtered)

    def update(self, **kwargs):
        """部分更新 + 重新驗證"""
        for k, v in kwargs.items():
            if hasattr(self, k):
                setattr(self, k, v)
        self.validate()
        return self

    def validate(self) -> None:
        """子類覆寫；預設 no-op"""
        pass


# ═══════════════════════════════════════════════════════════════════════
#  飛行參數 (取代 MainWindow.flight_params dict)
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class FlightParameters(_BaseSchema):
    """基本飛行參數

    欄位對應原 MainWindow.flight_params 的 key；新增欄位時直接加 dataclass
    field，並在 validate() 補檢查。
    """
    # ── 基礎 ────────────────────────────────────────
    altitude: float = 50.0              # 巡航高度 (m)
    speed: float = 15.0                 # 巡航速度 (m/s)
    angle: float = 0.0                  # 掃描角度 (°)
    spacing: float = 20.0               # 航線間距 (m)
    yaw_speed: float = 60.0             # 偏航速度 (°/s)
    subdivisions: int = 1               # 子區域數

    # ── 飛行器類型 ────────────────────────────────
    vehicle_type: str = 'multirotor'    # 'multirotor' / 'fixed_wing' / 'vtol'

    # ── 重疊與安全 ────────────────────────────────
    overlap_rate: float = 0.1           # 相鄰條帶重疊率 (0.0–0.5)
    reduce_overlap: bool = True
    safety_distance: float = 5.0        # NFZ 安全距離 (m)

    # ── 固定翼 ────────────────────────────────────
    turn_radius: float = 50.0           # 最小轉彎半徑 (m)
    max_bank_angle_deg: float = 35.0    # 最大側傾角
    climb_angle_deg: float = 8.0        # 爬升角

    # ── 圓心/起飛點 ──────────────────────────────
    circle_max_radius: float = 200.0

    def validate(self) -> None:
        if self.altitude < 0 or self.altitude > 5000:
            raise ValueError(f'altitude={self.altitude} 超出 0-5000m 範圍')
        if self.speed <= 0:
            raise ValueError(f'speed={self.speed} 必須 > 0')
        if not (0 <= self.overlap_rate <= 0.5):
            raise ValueError(f'overlap_rate={self.overlap_rate} 需在 [0, 0.5]')
        if self.turn_radius <= 0:
            raise ValueError('turn_radius 必須 > 0')
        if self.vehicle_type not in ('multirotor', 'fixed_wing', 'vtol'):
            raise ValueError(f'vehicle_type={self.vehicle_type} 不在支援清單')


# ═══════════════════════════════════════════════════════════════════════
#  蜂群打擊參數
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class StrikeParameters(_BaseSchema):
    """蜂群打擊完整參數 (取代 strike_params dict)"""

    # ── 發射位置 (launch_mode) + 時間協同 (timing_mode) 正交概念 ──
    # 2026 重構：launch_mode 值由 STOT/DTOT 改為 SAME/DIFF，消除與 timing_mode 歧義
    launch_mode: str = 'DIFF'            # 'SAME' 同地 / 'DIFF' 異地 (發射位置)
    timing_mode: str = 'STOT'            # 'STOT' 同時命中 / 'DTOT' 間隔命中 (時間協同)
    interval_sec: float = 0.0            # DTOT 時間間隔 (sec)

    # ── 基本參數 ─────────────────────────────────
    cruise_alt: float = 500.0
    cruise_speed: float = 60.0
    altitude_step: float = 30.0
    max_dive_angle: float = 45.0
    dive_initiation_dist: float = 800.0
    anim_speed: float = 3.0

    # ── 時空協同邊界 ────────────────────────────
    max_speed: float = 85.0
    stall_speed: float = 25.0
    min_turn_radius: float = 150.0

    # ── VTOL 專屬 ───────────────────────────────
    vtol_enabled: bool = False
    vtol_cruise_kts: float = 50.0
    vtol_terminal_kts: float = 90.0
    vtol_boundary_m: float = 2000.0
    vtol_cep_m: float = 12.0
    vtol_ai_seeker: bool = True

    # ── ReconToStrike ─────────────────────────────
    coalition_size: int = 3
    iapf_safe_dist: float = 1500.0

    def validate(self) -> None:
        # launch_mode: 接受新值 SAME/DIFF 與 legacy STOT/DTOT
        # (STOT→SAME 同地, DTOT→DIFF 異地)
        valid_launch = ('SAME', 'DIFF', 'STOT', 'DTOT')
        if self.launch_mode not in valid_launch:
            raise ValueError(
                f'launch_mode={self.launch_mode} 必須為 SAME/DIFF '
                f'(或 legacy STOT/DTOT)'
            )
        if self.timing_mode not in ('DTOT', 'STOT'):
            raise ValueError(f'timing_mode={self.timing_mode} 必須為 DTOT/STOT')
        if self.timing_mode == 'DTOT' and self.interval_sec <= 0:
            raise ValueError('DTOT 時間協同需 interval_sec > 0')
        if self.max_speed <= self.stall_speed:
            raise ValueError(
                f'max_speed={self.max_speed} 必須 > stall_speed={self.stall_speed}'
            )
        if self.max_dive_angle <= 0 or self.max_dive_angle > 89:
            raise ValueError(f'max_dive_angle={self.max_dive_angle} 需在 (0, 89]°')
        if self.min_turn_radius <= 0:
            raise ValueError('min_turn_radius 必須 > 0')
        if self.coalition_size < 1:
            raise ValueError('coalition_size >= 1')


# ═══════════════════════════════════════════════════════════════════════
#  SITL 連線參數
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class SITLParameters(_BaseSchema):
    """SITL 啟動與連線參數"""

    vehicle: str = 'plane'               # 'plane' / 'copter' / 'vtol'
    instance_count: int = 1
    connection_str: str = 'tcp:127.0.0.1:5760'
    auto_heartbeat: bool = True
    heartbeat_rate_hz: float = 1.0
    param_file: Optional[str] = None     # 選用 .parm 路徑

    def validate(self) -> None:
        if self.vehicle not in ('plane', 'copter', 'vtol'):
            raise ValueError(f'vehicle={self.vehicle} 不在支援清單')
        if self.instance_count < 1 or self.instance_count > 32:
            raise ValueError(f'instance_count 需在 1-32')
        if self.heartbeat_rate_hz <= 0:
            raise ValueError('heartbeat_rate_hz 必須 > 0')


# ═══════════════════════════════════════════════════════════════════════
#  測試 / 示範
# ═══════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    # 從 dict 建立 (典型：讀 YAML 或 UI 送進來的 dict)
    fp = FlightParameters.from_dict({
        'altitude': 100, 'speed': 20, 'vehicle_type': 'fixed_wing',
        'unknown_key': 'ignored',         # 未知 key 自動忽略
    })
    fp.validate()
    print(f'[OK] FlightParameters: {fp}')

    # 驗證邏輯
    try:
        bad = FlightParameters(altitude=-5)
        bad.validate()
    except ValueError as e:
        print(f'[OK] 驗證失敗: {e}')

    # Update + 重新驗證
    sp = StrikeParameters()
    sp.update(timing_mode='DTOT', interval_sec=15.0)
    print(f'[OK] StrikeParameters (DTOT 15s): {sp.timing_mode}, Δ={sp.interval_sec}s')
