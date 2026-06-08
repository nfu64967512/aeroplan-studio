"""mission/fence_zone.py — 統一的 Fence-style 區域資料模型 + 全域註冊器。

設計目標：把 NFZ（禁航區）與動態威脅區的「設定方式」統一成 ArduPilot
fence 模型（多邊形 + 圓形、含 / 拒含、海拔上下限），同時保留「分類」資訊
（NFZ / THREAT / GEOFENCE）用於：
    1. 視覺上以不同顏色區分
    2. 路徑規劃時依分類做不同處理
    3. 都可一鍵以 ArduPilot 4.2+ Polygon Fence 協定上傳飛控 EEPROM
       (MAV_MISSION_TYPE_FENCE + MAV_CMD_NAV_FENCE_*)

ArduPilot 對應：
    inclusion=True  + POLYGON → MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001)
    inclusion=False + POLYGON → MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION (5002)
    inclusion=True  + CIRCLE  → MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION         (5003)
    inclusion=False + CIRCLE  → MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION         (5004)
"""
from __future__ import annotations

import math
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from utils.math_utils import EARTH_RADIUS_M  # 0-2 標準化：WGS84 單一來源
from PyQt6.QtCore import QObject, pyqtSignal


# ──────────────────────────────────────────────────────────────────────
#  類型 enum
# ──────────────────────────────────────────────────────────────────────
class ZoneCategory(Enum):
    """區域用途分類（決定預設顏色與 inclusion 行為）"""

    NFZ = 'NFZ'            # 禁航區（exclusion，UAV 不可進入）
    THREAT = 'THREAT'      # 動態威脅區（exclusion，可能隨時間/情報變動）
    GEOFENCE = 'GEOFENCE'  # 任務作業圍籬（inclusion，UAV 必須留在內）


class ZoneShape(Enum):
    POLYGON = 'POLYGON'    # 任意多邊形（3+ 頂點）
    CIRCLE = 'CIRCLE'      # 圓形（中心 + 半徑）


# ──────────────────────────────────────────────────────────────────────
#  FenceZone 資料類
# ──────────────────────────────────────────────────────────────────────
@dataclass
class FenceZone:
    """單一 fence 區域 — 涵蓋 NFZ / 威脅 / 作業圍籬三類用途。"""

    name: str
    category: ZoneCategory
    shape: ZoneShape
    # POLYGON 用：[(lat, lon), ...] 至少 3 點
    vertices: list[tuple[float, float]] = field(default_factory=list)
    # CIRCLE 用：(lat, lon) 圓心 + 半徑（公尺）
    center: Optional[tuple[float, float]] = None
    radius_m: float = 0.0
    # 海拔範圍（公尺，AMSL 或 AGL 依專案慣例；ArduPilot FENCE_ALT_MAX 用之）
    alt_min_m: float = 0.0
    alt_max_m: float = 200.0
    # None → 依 category 推：GEOFENCE → True（含）；NFZ/THREAT → False（拒含）
    inclusion: Optional[bool] = None
    # 自動生成 8 碼 hex id，用於 Cesium entity 標識與 registry lookup
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])

    def __post_init__(self) -> None:
        if self.inclusion is None:
            self.inclusion = (self.category == ZoneCategory.GEOFENCE)
        # 強制型別（從 JSON / dialog 來的可能是 str）
        if isinstance(self.category, str):
            self.category = ZoneCategory(self.category)
        if isinstance(self.shape, str):
            self.shape = ZoneShape(self.shape)

    # ── 顏色（MIL-STD 警告色帶） ───────────────────────────────────
    @property
    def color_hex(self) -> str:
        return {
            ZoneCategory.NFZ:      '#E53935',   # 紅 — 禁止
            ZoneCategory.THREAT:   '#FF8800',   # 橘 — 警告
            ZoneCategory.GEOFENCE: '#FFB703',   # 琥珀 — 注意
        }[self.category]

    # ── 多邊形展開（給 Cesium / FC 上傳一致使用）───────────────────
    def polygon_vertices(self, circle_segments: int = 16) -> list[tuple[float, float]]:
        """取得頂點列表 — 圓形會展開為 N 邊形（球面 destination 公式）。

        Args:
            circle_segments: CIRCLE 形狀展開的邊數（默認 16 邊，精度 < 2% R）

        Returns:
            [(lat, lon), ...] — POLYGON 直接回傳 vertices；CIRCLE 回傳近似 N 邊形
        """
        if self.shape == ZoneShape.POLYGON:
            return list(self.vertices)
        if self.center is None or self.radius_m <= 0:
            return []
        lat0, lon0 = self.center
        R = EARTH_RADIUS_M
        ang = self.radius_m / R
        lat0r = math.radians(lat0)
        lon0r = math.radians(lon0)
        out = []
        for i in range(circle_segments):
            # 等角分佈，bearing 從正北 0° 順時針
            brg = 2 * math.pi * i / circle_segments
            lat_r = math.asin(
                math.sin(lat0r) * math.cos(ang)
                + math.cos(lat0r) * math.sin(ang) * math.cos(brg)
            )
            lon_r = lon0r + math.atan2(
                math.sin(brg) * math.sin(ang) * math.cos(lat0r),
                math.cos(ang) - math.sin(lat0r) * math.sin(lat_r),
            )
            out.append((math.degrees(lat_r), math.degrees(lon_r)))
        return out

    # ── ArduPilot fence 指令對應 ───────────────────────────────────
    @property
    def mavlink_cmd_id(self) -> int:
        """回傳對應的 MAV_CMD_NAV_FENCE_* 指令 ID（5001-5004）。"""
        if self.shape == ZoneShape.POLYGON:
            return 5001 if self.inclusion else 5002
        else:  # CIRCLE
            return 5003 if self.inclusion else 5004

    # ── 序列化（給 Cesium JSON / persist） ────────────────────────
    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'name': self.name,
            'category': self.category.value,
            'shape': self.shape.value,
            'vertices': list(self.vertices),
            'center': list(self.center) if self.center else None,
            'radius_m': self.radius_m,
            'alt_min_m': self.alt_min_m,
            'alt_max_m': self.alt_max_m,
            'inclusion': self.inclusion,
            'color_hex': self.color_hex,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'FenceZone':
        verts = [tuple(v) for v in d.get('vertices', [])]
        c = d.get('center')
        return cls(
            name=d['name'],
            category=ZoneCategory(d['category']),
            shape=ZoneShape(d['shape']),
            vertices=verts,
            center=tuple(c) if c else None,
            radius_m=float(d.get('radius_m', 0)),
            alt_min_m=float(d.get('alt_min_m', 0)),
            alt_max_m=float(d.get('alt_max_m', 200)),
            inclusion=d.get('inclusion'),
            id=d.get('id') or uuid.uuid4().hex[:8],
        )


# ──────────────────────────────────────────────────────────────────────
#  FenceZoneRegistry — 全域單例
# ──────────────────────────────────────────────────────────────────────
class FenceZoneRegistry(QObject):
    """全域 fence 區域集合（process-wide 單例）。

    UI / planner / SITL 上傳器都從這裡取資料來源。任何增刪改都會
    發 zones_changed signal，讓 Cesium 視覺、planner 約束、FC 上傳
    狀態三方同步。
    """

    zones_changed = pyqtSignal()         # 增 / 刪 / 清空 / 修改 都觸發
    zone_added = pyqtSignal(object)      # FenceZone — 給視覺 add 用
    zone_removed = pyqtSignal(str)       # zone_id — 給視覺 remove 用

    _instance: Optional['FenceZoneRegistry'] = None

    @classmethod
    def instance(cls) -> 'FenceZoneRegistry':
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self) -> None:
        super().__init__()
        self._zones: dict[str, FenceZone] = {}

    # ── CRUD ───────────────────────────────────────────────────────
    def add(self, zone: FenceZone) -> None:
        self._zones[zone.id] = zone
        self.zone_added.emit(zone)
        self.zones_changed.emit()

    def remove(self, zone_id: str) -> bool:
        if zone_id in self._zones:
            del self._zones[zone_id]
            self.zone_removed.emit(zone_id)
            self.zones_changed.emit()
            return True
        return False

    def clear(self) -> None:
        if not self._zones:
            return
        ids = list(self._zones.keys())
        self._zones.clear()
        for zid in ids:
            self.zone_removed.emit(zid)
        self.zones_changed.emit()

    # ── 查詢 ───────────────────────────────────────────────────────
    def all(self) -> list[FenceZone]:
        return list(self._zones.values())

    def get(self, zone_id: str) -> Optional[FenceZone]:
        return self._zones.get(zone_id)

    def by_category(self, category: ZoneCategory) -> list[FenceZone]:
        return [z for z in self._zones.values() if z.category == category]

    def count(self) -> int:
        return len(self._zones)

    # ── ArduPilot FENCE_TYPE 計算 ──────────────────────────────────
    def fence_type_bits(self, include_max_alt: bool = True) -> int:
        """依當前 zone 自動算 FENCE_TYPE bitmask。

        ArduPilot 4.x:
            bit 0 (=1)  Max Altitude
            bit 1 (=2)  Circle
            bit 2 (=4)  Polygon
            bit 3 (=8)  Min Altitude  (only ArduCopter ≥ 4.0)
        """
        has_polygon = any(z.shape == ZoneShape.POLYGON for z in self._zones.values())
        has_circle = any(z.shape == ZoneShape.CIRCLE for z in self._zones.values())
        bits = 0
        if include_max_alt:
            bits |= 1
        if has_circle:
            bits |= 2
        if has_polygon:
            bits |= 4
        return bits if bits else 4  # 預設至少含 polygon

    def max_altitude_m(self, default: float = 200.0) -> float:
        """所有 zone alt_max 的最大值（給 FENCE_ALT_MAX 用）"""
        if not self._zones:
            return default
        return max((z.alt_max_m for z in self._zones.values()), default=default)

    def min_altitude_m(self, default: float = 0.0) -> float:
        if not self._zones:
            return default
        return min((z.alt_min_m for z in self._zones.values()), default=default)


__all__ = [
    'ZoneCategory',
    'ZoneShape',
    'FenceZone',
    'FenceZoneRegistry',
]
