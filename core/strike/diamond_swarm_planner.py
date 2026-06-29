"""
DiamondSwarmStrikePlanner — 菱形編隊 + 末端 STOT 飽和打擊

依《軍工級無人機群飛演算法與地面站架構》戰術規範實作：

    Phase 1  起飛 (NAV_TAKEOFF) ──────► 各機從共用基地直線爬升
    Phase 2  菱形編隊巡航 ────────────► Leader / Rear / Left / Right
                                          四點繞長機航線同步飛行
    Phase 3  決斷邊界 (Decision Ring) ─► 距目標 R_dec (預設 3 km) 處
                                          編隊解散、四散到 4 個方位
    Phase 4  STOT 補時 (NAV_LOITER_TIME)► 在預備點 P_pre_i 盤旋至同秒
    Phase 5  四向同時俯衝 ───────────► 0° / 90° / 180° / 270° 方位
                                          各 UCAV 從 r_dive 處俯衝命中

數學推導
--------

**菱形編隊本地座標** (長機在 (0,0)，+x=右、+y=前)::

    Leader (前)  : (   0,    0)
    Rear   (後)  : (   0, -d_FR)
    Left   (左)  : (-d_LR/2, -d_FR/2)
    Right  (右)  : (+d_LR/2, -d_FR/2)

    d_FR : 前後跨距 ── 約 2 × R_min（避免後機接近長機尾流）
    d_LR : 左右跨距 ── ≥ 2 × safety distance

**本地 → ENU**（航向 ψ：MAVLink 慣例 0°=N 順時針）::

    ΔE =  x·cos(ψ) + y·sin(ψ)
    ΔN = -x·sin(ψ) + y·cos(ψ)

**ENU → WGS84**（小範圍近似，誤差 <1 m / 100 km）::

    Δlat = ΔN / 111320
    Δlon = ΔE / (111320 · cos(lat_deg))

**STOT 時間補時**::

    L_i      = haversine(P_pre_i → P_target) (3D 含俯衝段)
    t_i      = L_i / V_cruise
    t_max    = max(t_i over i = 0..3)
    loiter_i = t_max - t_i              ← 在 P_pre_i 插入 NAV_LOITER_TIME

物理約束
--------
- 固定翼最小轉彎半徑 R_min（預設 150 m）— 編隊解散到預備點用 Dubins 銜接
- 最大俯衝角 θ_max（預設 45°）— 自動驗證 r_dive 是否足夠
- 全部航點維持 cruise_alt，僅最後一段下到 target_alt（俯衝）

匯出格式
--------
QGC WPL 110（與 Mission Planner / QGroundControl 相容）：

    QGC WPL 110
    0  1   0   16  0  0  0  0  HOME_LAT  HOME_LON  0          1
    1  0   3   22  0  0  0  0  HOME_LAT  HOME_LON  TAKEOFF_ALT 1   ← NAV_TAKEOFF
    2  0   3   16  0  0  0  0  WP_LAT    WP_LON    CRUISE_ALT  1   ← NAV_WAYPOINT
    ...
    K  0   3  19  LOITER_T  0  R  0  PRE_LAT  PRE_LON CRUISE_ALT 1 ← NAV_LOITER_TIME
    K+1 0  3  16  0  0  0  0  DIVE_LAT  DIVE_LON  CRUISE_ALT  1   ← 俯衝起點
    K+2 0  3  16  0  0  0  0  TGT_LAT   TGT_LON   TGT_ALT     1   ← 終端目標
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from core.strike.geometry import (
    haversine, bearing_deg, destination, dubins_path_samples,
)
from core.strike.maneuver_path import generate_s_maneuver_path
# 與既有 SwarmStrikePlanner 共用同一個 MissionItem 結構
from core.strike.swarm_strike_planner import MissionItem, FeasibilityStatus


# ══════════════════════════════════════════════════════════════════════
#  ArduPilot MAVLink 命令常數（避免依賴 pymavlink，方便純 Python 使用）
# ══════════════════════════════════════════════════════════════════════
MAV_CMD_NAV_WAYPOINT       = 16
MAV_CMD_NAV_LOITER_UNLIM   = 17        # 無限盤旋（mission 永不結束，避免觸發 RTL）
MAV_CMD_NAV_LOITER_TIME    = 19
MAV_CMD_NAV_TAKEOFF        = 22
MAV_CMD_NAV_DELAY          = 93        # 在當前位置 / 地面等待指定秒數
MAV_CMD_DO_CHANGE_SPEED    = 178


# ══════════════════════════════════════════════════════════════════════
#  資料結構
# ══════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class Target:
    """單一打擊目標（菱形編隊只支援單目標飽和攻擊）。"""
    lat: float
    lon: float
    alt: float = 0.0
    name: str = 'TGT'


@dataclass(frozen=True)
class FormationConfig:
    """菱形/楔形/環形編隊幾何 + 物理參數（支援 2–12 架）。"""

    # ── 編隊規模 ──────────────────────────────────────────────
    n_uavs: int = 4                       # 參戰 UCAV 數量（2–12）

    # ── 幾何尺寸 ──────────────────────────────────────────────
    front_back_spacing_m: float = 300.0   # 列距（每往後一列加一個此距離）
    left_right_half_m:    float = 200.0   # 左右半跨距（左右翼到中軸距離）
    cruise_alt_m:         float = 500.0
    takeoff_alt_m:        float = 80.0    # NAV_TAKEOFF 的目標離地高度（用於 ArduPlane 命令）
    takeoff_pitch_deg:    float = 12.0    # 固定翼起飛仰角（典型 8-15°；用於斜爬升軌跡計算）
    runway_spacing_m:     float = 30.0    # 跑道並排起飛間距 (m)；各機在地面沿 heading
                                          # 的 perpendicular 方向均勻並排（leader 居中），
                                          # 爬升結束後才過渡到菱形編隊位置
    cruise_speed_mps:     float = 60.0

    # ── 戰術 ──────────────────────────────────────────────────
    decision_ring_m:  float = 3000.0
    pre_strike_radius_m: float = 1500.0
    loiter_radius_m:  float = 120.0
    dive_initiation_m: float = 800.0
    max_dive_angle_deg: float = 45.0

    # ── 物理限制 ──────────────────────────────────────────────
    min_turn_radius_m: float = 150.0

    # ── 機間錯開（避免空中碰撞 / 波次攻擊）─────────────────────
    altitude_step_m: float = 0.0     # 各機巡航高度錯開步長 (m)；
                                     # UAV_i 的 cruise_alt = base_cruise + i × step
                                     # 0 = 全部同高度（會增加碰撞風險）
                                     # 建議 ≥ 50m（航空管制典型分層 1000ft ≈ 305m）
    timing_mode: str = 'STOT'        # 'STOT' = 同秒命中（飽和打擊）
                                     # 'DTOT' = 依攻擊方位順序間隔命中（波次攻擊）
    time_interval_s: float = 0.0     # DTOT 模式下機間命中時間間隔 (s)
                                     # 例如 3.0 → UAV_0 命中 t、UAV_1 命中 t+3s、UAV_2 命中 t+6s ...
    delay_strategy: str = 'GROUND'   # 對齊命中時刻的策略（取代或補充空中盤旋）：
                                     # 'GROUND' — 較近 UAV 在地面 NAV_DELAY 後起飛（預設）
                                     #            優點：不空轉燃油、降低空中暴露時間、命中時刻精確
                                     # 'AIR'    — 較近 UAV 在 P_pre 預備點 NAV_LOITER_TIME 盤旋
                                     #            原本行為，仍可用於需要空中集結的場景
                                     # 'SMANEUVER' — 以 S 形蛇行消耗時間（見 maneuver_path）
    launch_stagger_s: float = 0.0    # SMANEUVER 用：依 sysid（長機 UAV1 先）微錯開起飛間隔（s）。
                                     # 跑道並排起飛不再同秒擠在一起；命中時刻仍對齊——
                                     # 每機 S 機動補時自動吸收自己的起飛延遲（base 整體 +最大錯開量）。

    # ── 攻擊弧度（同側半圈扇面攻擊） ─────────────────────────────
    # 攻擊方位以「back_bearing = cruise_heading + 180°」為中心，
    # N 個方位平均分布在 [back - arc/2, back + arc/2] 範圍內（端點含）。
    # 相鄰方位間距 = arc / (N-1)。
    #   180° (預設) — 半圓同側攻擊：N 個方位均分 base 那一側 180°，
    #                 包含兩翼 ±90° 端點。例 N=4 → back ± [90°, 30°]，
    #                 相鄰間距 60°。所有 UAV 從同一側俯衝、不繞過目標、
    #                 不在目標上空交會、最短飛行距離。
    #   360°       — 全方位包圍：360°/N 均分（舊行為），N 機從四面八方撞擊
    #                 ；360° 模式下端點 wrap-around，邏輯改用「步進 360°/N」。
    #   90°-120°   — 窄扇面楔形：所有機從同一窄扇面協同攻擊。
    attack_arc_deg: float = 180.0

    # ── 方位角分配 ─────────────────────────────────────────────
    # None ⇒ 自動依 attack_arc_deg 在 back_bearing 兩側均分；
    # 提供 tuple 則覆蓋（長度需 ≥ n_uavs）
    attack_bearings_deg: Optional[Tuple[float, ...]] = None

    def __post_init__(self):
        """建構即驗證關鍵參數，壞值 fail-fast 而非後面靜默算出 NaN/0。"""
        if not (2 <= int(self.n_uavs) <= 12):
            raise ValueError(f"n_uavs 需在 2..12，收到 {self.n_uavs}")
        if self.cruise_speed_mps <= 0:
            raise ValueError(f"cruise_speed_mps 需 > 0，收到 {self.cruise_speed_mps}")
        if not (0.0 < self.max_dive_angle_deg < 90.0):
            raise ValueError(f"max_dive_angle_deg 需在 (0,90)，收到 {self.max_dive_angle_deg}")
        if self.min_turn_radius_m <= 0:
            raise ValueError(f"min_turn_radius_m 需 > 0，收到 {self.min_turn_radius_m}")
        if (self.attack_bearings_deg is not None
                and len(self.attack_bearings_deg) < int(self.n_uavs)):
            raise ValueError(
                f"attack_bearings_deg 長度 {len(self.attack_bearings_deg)} < n_uavs {self.n_uavs}")


@dataclass
class DiamondSlot:
    """菱形 4 個位置之一（在編隊中的角色）。"""
    role: str          # 'leader' | 'rear' | 'left' | 'right'
    local_x: float     # +x = 右
    local_y: float     # +y = 前
    sysid: int         # 1..4


@dataclass
class UAVStrikePlan:
    """單架 UCAV 從起飛 → 編隊 → 四散 → 俯衝 的完整任務。"""
    sysid: int
    role: str
    base_lat: float
    base_lon: float
    # Slot 起飛點（base + 菱形編隊偏移）— 各機在地面排開的實際位置
    # 用於：① SITL spawn home  ② QGC WPL 中 HOME / NAV_TAKEOFF 的 lat/lon
    # 若為 0.0 → 退回使用 base_lat/base_lon（向後相容）
    slot_lat: float = 0.0
    slot_lon: float = 0.0
    # 該機個體巡航高度（base_cruise + slot_idx × altitude_step）
    # 各機在巡航/編隊/俯衝起點分層，避免空中碰撞
    cruise_alt: float = 0.0

    # ── 規劃結果（3 段視覺化用）────────────────────────────────
    # Phase 1：編隊段（淺綠）— 從基地→決斷邊界
    formation_path: List[Tuple[float, float, float]] = field(default_factory=list)
    # Phase 2：四散段（黃）— 從決斷邊界→預備點
    dispersion_path: List[Tuple[float, float, float]] = field(default_factory=list)
    # Phase 3：俯衝段（血紅）— 從俯衝起點→目標
    dive_path:       List[Tuple[float, float, float]] = field(default_factory=list)

    pre_strike_lat:  float = 0.0
    pre_strike_lon:  float = 0.0
    dive_lat:        float = 0.0
    dive_lon:        float = 0.0
    attack_bearing_deg: float = 0.0

    # ── 時間 ──────────────────────────────────────────────────
    final_leg_length_m: float = 0.0   # P_pre → P_dive → target
    final_leg_time_s:   float = 0.0
    loiter_time_s:      float = 0.0   # 空中補時量（NAV_LOITER_TIME @ P_pre）
    takeoff_delay_s:    float = 0.0   # 地面起飛前等待時間（NAV_DELAY 在 NAV_TAKEOFF 之前）
    feasibility: FeasibilityStatus = FeasibilityStatus.OK
    warning: str = ''

    # ── MAVLink 任務 ─────────────────────────────────────────
    mission: List[MissionItem] = field(default_factory=list)


# ══════════════════════════════════════════════════════════════════════
#  DiamondSwarmStrikePlanner
# ══════════════════════════════════════════════════════════════════════
class DiamondSwarmStrikePlanner:
    """
    菱形編隊蜂群末端打擊規劃器。

    Public API
    ----------
    plan(target, base_latlon)              ── 規劃 4 機打擊任務
    export_qgc_wpl(plans, dir)             ── 每機一個 .waypoints
    """

    # 動態編隊 slot 配置（N 可變 2–12 架）
    @staticmethod
    def _slots(cfg: FormationConfig) -> List[DiamondSlot]:
        """
        產生 N 個編隊位置（本地座標 +x=右、+y=前）。
        各 N 對應的隊形：
            N=2 ─ 雙機並排 (Wingman pair)：Leader + Wing
            N=3 ─ V 字楔形：Leader 居前，左右翼斜後
            N=4 ─ 標準菱形：Leader/Rear/Left/Right（與舊版相容）
            N≥5 ─ 擴展菱形：基本菱形 + 後方梯隊 (Echelon trail)
                  超過 4 架時依序在 Y=-(row+1)*d 列上左右交替排列
        """
        d = cfg.front_back_spacing_m
        w = cfg.left_right_half_m
        n = max(2, min(int(cfg.n_uavs), 12))

        slots: List[DiamondSlot] = []
        slots.append(DiamondSlot(role='leader', local_x=0.0, local_y=0.0, sysid=1))
        if n == 1:
            return slots

        if n == 2:
            slots.append(DiamondSlot(role='wing',  local_x=+w,  local_y=-d/2, sysid=2))
            return slots

        if n == 3:
            slots.append(DiamondSlot(role='left',  local_x=-w,  local_y=-d/2, sysid=2))
            slots.append(DiamondSlot(role='right', local_x=+w,  local_y=-d/2, sysid=3))
            return slots

        if n == 4:
            # 標準菱形（與舊版相容）
            slots.append(DiamondSlot(role='rear',  local_x=0.0, local_y=-d,   sysid=2))
            slots.append(DiamondSlot(role='left',  local_x=-w,  local_y=-d/2, sysid=3))
            slots.append(DiamondSlot(role='right', local_x=+w,  local_y=-d/2, sysid=4))
            return slots

        # N ≥ 5：基本菱形 + 後方梯隊（左右交替）
        # 前 4 個固定為 leader/rear/left/right，從第 5 個起在 row=2,3,... 上排列
        slots.append(DiamondSlot(role='rear',  local_x=0.0, local_y=-d,   sysid=2))
        slots.append(DiamondSlot(role='left',  local_x=-w,  local_y=-d/2, sysid=3))
        slots.append(DiamondSlot(role='right', local_x=+w,  local_y=-d/2, sysid=4))
        for i in range(4, n):
            # 第 i 個（從 0 起）的位置：row = (i-3)//2 + 2，左右交替
            extra_row = ((i - 3) // 2) + 2          # 2, 2, 3, 3, 4, 4, ...
            side = -1 if (i % 2 == 0) else +1
            role = f'echelon_{i+1}'
            slots.append(DiamondSlot(
                role=role,
                local_x=side * w,
                local_y=-extra_row * d,
                sysid=i + 1,
            ))
        return slots

    def __init__(self, cfg: Optional[FormationConfig] = None) -> None:
        self.cfg: FormationConfig = cfg or FormationConfig()

    # ──────────────────────────────────────────────────────────────────
    # 內部數學
    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _local_to_enu(local_x: float, local_y: float,
                       heading_deg: float) -> Tuple[float, float]:
        """
        本地 (右, 前) → 世界 ENU (東, 北)。

        ψ 為 MAVLink 航向（0°=N，順時針增加）。
        """
        psi = math.radians(heading_deg)
        de =  local_x * math.cos(psi) + local_y * math.sin(psi)
        dn = -local_x * math.sin(psi) + local_y * math.cos(psi)
        return de, dn

    @staticmethod
    def _enu_to_latlon(lat0: float, lon0: float,
                        de: float, dn: float) -> Tuple[float, float]:
        """ENU 公尺位移 → WGS84 (小範圍近似，<200 km 誤差 <2 m)。"""
        d_lat = dn / 111320.0
        d_lon = de / (111320.0 * max(math.cos(math.radians(lat0)), 1e-6))
        return lat0 + d_lat, lon0 + d_lon

    def _slot_position(self, leader_lat: float, leader_lon: float,
                        leader_heading_deg: float, slot: DiamondSlot
                        ) -> Tuple[float, float]:
        """從長機航點 + 航向 → 本 slot 的世界 (lat, lon)。"""
        de, dn = self._local_to_enu(slot.local_x, slot.local_y, leader_heading_deg)
        return self._enu_to_latlon(leader_lat, leader_lon, de, dn)

    # ──────────────────────────────────────────────────────────────────
    # 主規劃流程
    # ──────────────────────────────────────────────────────────────────
    def plan(self, target: Target,
              base_latlon: Tuple[float, float],
              cruise_heading_deg: Optional[float] = None,
              ) -> List[UAVStrikePlan]:
        """
        生成 4 機完整打擊任務。

        Parameters
        ----------
        target : Target
            單一打擊目標
        base_latlon : (lat, lon)
            共用發射基地座標（4 機從此點起飛）
        cruise_heading_deg : float, optional
            編隊巡航航向（None = 朝目標方位飛）

        Returns
        -------
        list[UAVStrikePlan]
            4 個 UAV 的任務規劃；mission 欄位即可直接 dump 成 QGC WPL。
        """
        cfg = self.cfg
        b_lat, b_lon = base_latlon

        # ── 1) 巡航航向：基地朝目標 ────────────────────────────
        if cruise_heading_deg is None:
            cruise_heading_deg = bearing_deg(b_lat, b_lon, target.lat, target.lon)

        # ── 2) 決斷邊界：距目標 R_dec、航向回指基地的點 ──────
        # 從目標朝「來向」(基地方向) 退 R_dec 公尺即為解散點
        back_bearing = (cruise_heading_deg + 180.0) % 360.0
        dec_lat, dec_lon = destination(
            target.lat, target.lon, back_bearing, cfg.decision_ring_m
        )

        # ── 3) N 個預備點（同側半圈扇面 / 全方位包圍 / 使用者自訂）──
        # 攻擊方位的物理意義：「P_pre 從 target 看出去的方位角」。
        # P_pre → target 的進攻方向 = (bearing + 180°)。
        #
        # 戰術考量（重要！）：
        #   舊行為：360° 全方位 → 部分 UAV 必須繞過目標飛到對側 P_pre，
        #          多飛 2× pre_strike_radius_m + 飛過目標上空暴露
        #          → 時間落差大、視覺像「鋸齒過頭」、增加被攔截風險
        #   新預設：以 back_bearing（從 target 朝 base 那側）為中心，
        #          arc=180° 範圍內均分 N 個方位（含 ±90° 端點）
        #          → 全部從 base→target 那一側俯衝、不繞目標、最短飛行
        #
        # 新公式（端點含、相鄰等距）：
        #   N=1:  bearings = [back]
        #   N>=2: bearings[i] = back - arc/2 + i × arc/(N-1)
        #         相鄰間距 = arc / (N-1)
        # 範例 N=4, arc=180°, back=42° (cruise=222°) →
        #   [back-90, back-30, back+30, back+90]
        #   = [-48, 12, 72, 132] (mod 360)
        #   相鄰間距 60°，端點剛好 back ± 90° (target 兩翼)
        # 4 個 P_pre 全部位於目標 base 那一側（從 base→target 方向看）。
        n_uavs = max(2, min(int(cfg.n_uavs), 12))
        if cfg.attack_bearings_deg and len(cfg.attack_bearings_deg) >= n_uavs:
            # 使用者指定 → 直接採用
            bearings = list(cfg.attack_bearings_deg[:n_uavs])
        elif cfg.attack_arc_deg >= 359.9:
            # 360° 全方位均分（舊行為，向後相容）— 端點 wrap-around，用步進
            bearings = [(360.0 / n_uavs) * i for i in range(n_uavs)]
        elif n_uavs == 1:
            # 單機 → 對準 back_bearing 正中央俯衝
            bearings = [back_bearing % 360.0]
        else:
            # 同側半圈扇面：N 個方位等距分布 [back - arc/2, back + arc/2]
            # 含端點，相鄰間距 = arc / (N-1)。
            arc = max(float(cfg.attack_arc_deg), 1.0)
            step = arc / float(n_uavs - 1)
            bearings = [
                (back_bearing - arc / 2.0 + i * step) % 360.0
                for i in range(n_uavs)
            ]

        pre_pts: List[Tuple[float, float, float]] = []
        for ang in bearings:
            p_lat, p_lon = destination(
                target.lat, target.lon, ang, cfg.pre_strike_radius_m
            )
            pre_pts.append((p_lat, p_lon, ang))

        # ── 4) N 個俯衝起點（離目標 r_dive，從 P_pre 朝目標方向）
        # 機間高度錯開時，最高那架的 alt_drop 最大 → 用 max alt 算 r_dive
        # 確保所有機都在 max_dive_angle 內俯衝（高機俯衝角偏陡，低機偏緩）。
        n_uavs_pre = max(2, min(int(cfg.n_uavs), 12))
        max_cruise_alt = cfg.cruise_alt_m + (n_uavs_pre - 1) * cfg.altitude_step_m
        alt_drop_max = max(max_cruise_alt - target.alt, 1.0)
        min_dive_dist = alt_drop_max / math.tan(math.radians(cfg.max_dive_angle_deg))
        r_dive = max(cfg.dive_initiation_m, min_dive_dist + 5.0)

        dive_pts: List[Tuple[float, float]] = []
        for (p_lat, p_lon, ang) in pre_pts:
            # 俯衝起點 = 從預備點朝向目標、退 (r_pre - r_dive) 公尺
            # 即：在「預備點與目標連線」上，距目標 r_dive 處
            d_lat, d_lon = destination(target.lat, target.lon,
                                        ang, r_dive)
            dive_pts.append((d_lat, d_lon))

        # ── 5) STOT 時間平衡（各機完整任務時間） ─────────────────
        # 之前只算 P_pre→target → 各機距離全部相同 → loiter_time 全 0 → 沒補時。
        # 現在改算「lineup 起飛點 → 爬升 → 編隊 → 解散 → P_pre → 俯衝 → 目標」
        # 完整任務距離，把 slot 不同造成的編隊段差異與 P_pre 方位差異都納入。
        slots = self._slots(cfg)
        attack_assignment = self._assign_slots_to_bearings(slots, cfg.attack_bearings_deg)
        plans: List[UAVStrikePlan] = []

        # 爬升相關常數（依 takeoff_pitch 計算各機水平爬升距離）
        takeoff_pitch_rad = math.radians(max(cfg.takeoff_pitch_deg, 5.0))

        # leader 爬升結束點（無 slot 偏移） — 用作菱形編隊起點基準。
        # 為避免 V 字回頭轉折，必須同時補償兩個因素：
        #   ① 後排機 slot.local_y = -d 把 formation_start 拉到 climb_end 後方 d 公尺
        #   ② altitude_step > 0 時，高層 UAV 自己的 plan_climb_horiz 比 leader 更遠
        # 所以 leader formation 基準 = max(各機 climb_horiz) + front_back_spacing + 安全緩衝。
        # 這樣保證所有 slot（含最高高度 + 後排 y=-d）的 formation_start
        # 都在自己 climb_end 前方，不會 backward 飛。
        n_uavs_pre = max(2, min(int(cfg.n_uavs), 12))
        all_climb_horiz = [
            (cfg.cruise_alt_m + i * cfg.altitude_step_m) / math.tan(takeoff_pitch_rad)
            for i in range(n_uavs_pre)
        ]
        max_climb_horiz = max(all_climb_horiz)
        # leader_climb_end 距 base 的水平距離（沿 cruise_heading）
        leader_formation_distance_m = (
            max_climb_horiz + float(cfg.front_back_spacing_m) + 100.0
        )
        leader_climb_end_lat, leader_climb_end_lon = destination(
            b_lat, b_lon, cruise_heading_deg, leader_formation_distance_m,
        )

        n_total = len(slots)
        leg_lengths: List[float] = []
        per_uav_cruise_alts: List[float] = []     # 各機個體巡航高度 (m)
        slot_positions: List[Tuple[float, float]] = []   # (1-3) lineup 起飛點，供第二迴圈 reuse
        climb_horiz_list: List[float] = []               # (1-3) 爬升水平距離，供第二迴圈 reuse
        for slot_idx, slot in enumerate(slots):
            # 該機個體巡航高度 — 機間錯開避免空中碰撞
            plan_cruise_alt = cfg.cruise_alt_m + slot_idx * cfg.altitude_step_m
            plan_alt_drop = max(plan_cruise_alt - target.alt, 1.0)
            per_uav_cruise_alts.append(plan_cruise_alt)

            # 該機個體斜爬升 3D 距離（爬到自己的 cruise_alt）
            plan_climb_horiz = plan_cruise_alt / math.tan(takeoff_pitch_rad)
            plan_climb_3d = math.hypot(plan_climb_horiz, plan_cruise_alt)

            bearing_idx = attack_assignment[slot_idx]
            (p_lat, p_lon, _ang) = pre_pts[bearing_idx]
            (d_lat, d_lon) = dive_pts[bearing_idx]

            # ① lineup 起飛點（perpendicular 排開）
            lineup_lx = (slot_idx - (n_total - 1) / 2.0) * cfg.runway_spacing_m
            de_lu, dn_lu = self._local_to_enu(lineup_lx, 0.0, cruise_heading_deg)
            sl_lat, sl_lon = self._enu_to_latlon(b_lat, b_lon, de_lu, dn_lu)
            slot_positions.append((sl_lat, sl_lon))       # (1-3) 供第二迴圈 reuse
            climb_horiz_list.append(plan_climb_horiz)     # (1-3) 供第二迴圈 reuse
            # ② 該機爬升結束點（按該機高度算的水平距離）
            cl_lat, cl_lon = destination(
                sl_lat, sl_lon, cruise_heading_deg, plan_climb_horiz
            )
            # ③ 菱形編隊起點（leader_climb_end + slot 偏移） — 即從 lineup 並排
            #    過渡到菱形所在的 3D 位置
            fs_lat, fs_lon = self._slot_position(
                leader_climb_end_lat, leader_climb_end_lon,
                cruise_heading_deg, slot,
            )
            # ④ 編隊段終點（dec_pt + slot 偏移）
            de_lat_s, de_lon_s = self._slot_position(
                dec_lat, dec_lon, cruise_heading_deg, slot,
            )

            # 累加各段距離
            L = (
                plan_climb_3d                                            # ①→② 斜爬升 3D
                + haversine(cl_lat, cl_lon, fs_lat, fs_lon)              # ②→③ 過渡到菱形
                + haversine(fs_lat, fs_lon, de_lat_s, de_lon_s)          # ③→④ 編隊巡航
                + haversine(de_lat_s, de_lon_s, p_lat, p_lon)            # ④→P_pre 解散段
                + haversine(p_lat, p_lon, d_lat, d_lon)                  # P_pre→P_dive
            )
            # 俯衝段 3D（水平 r_dive + 垂直該機 alt_drop）
            L += math.hypot(haversine(d_lat, d_lon, target.lat, target.lon),
                            plan_alt_drop)
            leg_lengths.append(L)

        leg_times = [L / cfg.cruise_speed_mps for L in leg_lengths]
        # ── 命中時刻平衡：依 timing_mode 決定各機補時 ─────────────
        # STOT — 全部同秒命中（飽和打擊），補時 = t_max - t_i
        # DTOT — 依 attack_bearing 升冪順序間隔命中（波次攻擊）
        #         第 k 架命中時刻 = t_max + k×interval
        if cfg.timing_mode.upper() == 'DTOT' and cfg.time_interval_s > 0.01:
            # 依方位角升冪決定打擊順序：0° 先打、90° 第二、180° 第三...
            ordered = sorted(range(n_total),
                             key=lambda i: pre_pts[attack_assignment[i]][2])
            wave_idx = [0] * n_total
            for k, slot_i in enumerate(ordered):
                wave_idx[slot_i] = k
            t_max = max(leg_times)
            loiter_times = [
                (t_max + wave_idx[i] * cfg.time_interval_s) - leg_times[i]
                for i in range(n_total)
            ]
        else:
            # STOT (預設) — 同秒命中
            t_max = max(leg_times)
            loiter_times = [t_max - t for t in leg_times]

        # 跑道並排起飛 — 各機在地面沿 cruise_heading 的 perpendicular 方向均勻排開
        # 中心對稱排列：N=4 → local_x ∈ {-1.5, -0.5, +0.5, +1.5} × runway_spacing_m
        # 相當於以「leader 居中、其他左右交替」的 lineup，模擬機場跑道並排起飛。
        # 爬升結束後才過渡到菱形編隊位置開始巡航。
        # （leader_climb_end_* 已在 STOT 區段算好）
        for slot_idx, (slot, bearing_idx) in enumerate(zip(slots, attack_assignment)):
            (p_lat, p_lon, ang) = pre_pts[bearing_idx]
            (d_lat, d_lon) = dive_pts[bearing_idx]

            # ── Phase 1：跑道並排起飛點（lineup）─────────────────────────────
            # (1-3 去重) lineup 起飛點 (slot_lat0/lon0) 與爬升水平距離 plan_climb_horiz
            #   已在第一迴圈以相同輸入算過，此處直接 reuse 避免重算（逐位元相同）。
            slot_lat0, slot_lon0 = slot_positions[slot_idx]
            plan_cruise_alt = per_uav_cruise_alts[slot_idx]
            plan_climb_horiz = climb_horiz_list[slot_idx]

            plan = UAVStrikePlan(
                sysid=slot.sysid,
                role=slot.role,
                base_lat=b_lat,
                base_lon=b_lon,
                slot_lat=slot_lat0,        # 並排起飛點 — SITL spawn home / NAV_TAKEOFF 對齊
                slot_lon=slot_lon0,
                cruise_alt=plan_cruise_alt,  # 該機個體巡航高度
                pre_strike_lat=p_lat,
                pre_strike_lon=p_lon,
                dive_lat=d_lat,
                dive_lon=d_lon,
                attack_bearing_deg=ang,
                final_leg_length_m=leg_lengths[slot_idx],
                final_leg_time_s=leg_times[slot_idx],
            )

            # 依 delay_strategy 分配延遲時間到「地面起飛前等待」或「空中盤旋」
            # 兩種策略命中時刻計算相同 (t_max - leg_time_i)，差別僅在物理執行位置：
            #   GROUND — NAV_DELAY 在地面延遲，較近的 UAV 較晚起飛 → 不空轉燃油、隱蔽
            #   AIR    — NAV_LOITER_TIME 在 P_pre 盤旋 → 原本行為，需要空中集結時用
            total_delay = max(loiter_times[slot_idx], 0.0)
            strat = cfg.delay_strategy.upper()
            if strat in ('AIR', 'SMANEUVER'):
                # AIR=盤旋圈補時；SMANEUVER=S 形蛇行補時（皆把待消耗時間放在空中）
                if strat == 'SMANEUVER' and cfg.launch_stagger_s > 0.0:
                    # 跑道安全：依 sysid（長機 UAV1=0）微錯開起飛，避免並排同秒擠在一起。
                    # base 整體 +最大錯開量，每機 S 機動補時 = 原補時 + 最大錯開 - 自己的起飛延遲，
                    # 故 arrival = base + 最大錯開 對所有機相同（命中時刻仍對齊），且補時恆 ≥ 0。
                    max_stag = cfg.launch_stagger_s * (max(cfg.n_uavs, 1) - 1)
                    plan.takeoff_delay_s = cfg.launch_stagger_s * (plan.sysid - 1)
                    plan.loiter_time_s = total_delay + max_stag - plan.takeoff_delay_s
                else:
                    plan.takeoff_delay_s = 0.0
                    plan.loiter_time_s = total_delay
            else:  # 'GROUND' (預設)
                plan.takeoff_delay_s = total_delay
                plan.loiter_time_s = 0.0

            # ── Phase 1：斜爬升軌跡 — 從 lineup 起飛點沿 cruise_heading 斜爬升 6 段
            #   各機平行爬升至各自 cruise_alt（高機爬得更高、更遠）。
            n_climb_samples = 6
            climb_pts: List[Tuple[float, float, float]] = []
            for i in range(n_climb_samples + 1):     # 0, 1, ..., n_climb_samples
                f = i / float(n_climb_samples)       # 高度與水平比例（線性插值）
                lat_i, lon_i = destination(
                    slot_lat0, slot_lon0,
                    cruise_heading_deg,
                    plan_climb_horiz * f,
                )
                alt_i = plan_cruise_alt * f          # 0 → 該機 cruise_alt 線性
                climb_pts.append((lat_i, lon_i, alt_i))

            # ── Phase 2：編隊巡航段（從爬升結束點 → 解散點，全程該機 cruise_alt）
            # n_samples 從 5 減為 2（僅起點 + 終點）— 編隊段是直線，中間 sample
            # 點對 ArduPlane 而言反而造成「飛到 wp 後又轉去下一個 wp」的鋸齒。
            # 兩點直線讓飛機飛得平滑，不再 overshoot/U-turn。
            f_samples = self._sample_formation_path(
                slot,
                leader_climb_end_lat, leader_climb_end_lon,
                dec_lat, dec_lon,
                cruise_heading_deg, n_samples=2,
            )
            # 巡航段全部維持該機 cruise_alt（爬升已在 Phase 1 完成）
            plan.formation_path = climb_pts + [
                (la, lo, plan_cruise_alt) for (la, lo) in f_samples
            ]
            # ── Phase 2：四散段（解散點末端 → 預備點）— Dubins 平滑轉彎
            # 編隊巡航方向是 cruise_heading；P_pre 進場方向應朝 target，
            # 即 (attack_bearing + 180°) — 從 P_pre 朝 target 的航向。
            # 兩個方向通常差 60–180°，直線連接會看起來像折角；
            # 用 Dubins 最短路徑（含圓弧）平滑連接，飛機實際也能依
            # min_turn_radius 飛出該軌跡。
            disp_start = f_samples[-1] if f_samples else (dec_lat, dec_lon)
            disp_start_hdg = cruise_heading_deg
            disp_end_hdg = (ang + 180.0) % 360.0    # P_pre → target 的方位
            disp_xy = dubins_path_samples(
                disp_start[0], disp_start[1], disp_start_hdg,
                p_lat, p_lon, disp_end_hdg,
                cfg.min_turn_radius_m,
                n_samples=20,
            )
            plan.dispersion_path = [
                (la, lo, plan_cruise_alt) for (la, lo) in disp_xy
            ]
            # ── Phase 3：俯衝段（俯衝起點 → 目標）— 從該機 cruise_alt 俯衝
            plan.dive_path = [
                (d_lat, d_lon, plan_cruise_alt),
                (target.lat, target.lon, target.alt),
            ]

            # ── 7) 組裝 MAVLink 任務 ─────────────────────────
            plan.mission = self._build_mission(plan, target, cfg)
            plans.append(plan)

        return plans

    # ──────────────────────────────────────────────────────────────────
    # 編隊軌跡取樣
    # ──────────────────────────────────────────────────────────────────
    def _sample_formation_path(self,
                                slot: DiamondSlot,
                                start_lat: float, start_lon: float,
                                end_lat: float, end_lon: float,
                                heading_deg: float,
                                n_samples: int = 5,
                                ) -> List[Tuple[float, float]]:
        """
        沿 [start → end] 取樣 n 個長機航點，每點按 slot 偏移到本機位置。
        提供給 3D 地圖視覺化（編隊菱形軌跡）。
        """
        pts: List[Tuple[float, float]] = []
        for i in range(n_samples):
            f = i / max(n_samples - 1, 1)
            leader_lat = start_lat + f * (end_lat - start_lat)
            leader_lon = start_lon + f * (end_lon - start_lon)
            slot_lat, slot_lon = self._slot_position(
                leader_lat, leader_lon, heading_deg, slot
            )
            pts.append((slot_lat, slot_lon))
        return pts

    @staticmethod
    def _assign_slots_to_bearings(slots: Sequence[DiamondSlot],
                                    bearings: Sequence[float]) -> List[int]:
        """
        將 N 個 slot 分派到 N 個攻擊方位角索引。

        新策略（同側扇面攻擊適用）：
          依 sysid 順序對應 bearings[0..N-1]，這樣攻擊方位由左至右排列：
            slot[0] (leader, 跑道最左) → bearings[0] (扇面最左)
            slot[1] (rear,   跑道左 2) → bearings[1]
            slot[2] (left,   跑道右 2) → bearings[2]
            slot[3] (right,  跑道最右) → bearings[3] (扇面最右)
          → 解散時各 UAV 向「自己對應的扇面方位」橫向擴散，
             路徑不交叉、Dubins 弧較短。

          舊版 N=4 用「leader→0°/right→90°/rear→180°/left→270°」直觀映射，
          但僅對 360° 全方位包圍有意義；新預設 180° 同側扇面下不再適用。
        """
        return list(range(len(slots)))

    # ──────────────────────────────────────────────────────────────────
    # MAVLink 任務組裝
    # ──────────────────────────────────────────────────────────────────
    def _append_time_consumption(self, m: List[MissionItem], plan: UAVStrikePlan,
                                 cfg: FormationConfig, plan_cruise_alt: float) -> None:
        """補時段 + 俯衝起點：依 delay_strategy 選 S 機動 / 盤旋圈 / 直接過預備點。

        SMANEUVER：預備點→俯衝起點間 S 形 weave，把路徑拉長以消耗 loiter_time_s；
        AIR：NAV_LOITER_TIME 盤旋；其餘(GROUND/基準機)：直接過預備點再進俯衝。
        """
        brg = plan.attack_bearing_deg
        if cfg.delay_strategy.upper() == 'SMANEUVER' and plan.loiter_time_s > 0.5:
            sweave = generate_s_maneuver_path(
                plan.pre_strike_lat, plan.pre_strike_lon,
                plan.dive_lat, plan.dive_lon,
                cfg.cruise_speed_mps, plan.loiter_time_s,
                cfg.min_turn_radius_m, plan_cruise_alt)
            for j, (wla, wlo, wal) in enumerate(sweave[:-1]):
                m.append(MissionItem(
                    cmd=MAV_CMD_NAV_WAYPOINT, lat=wla, lon=wlo, alt=wal,
                    comment=f'Phase 4 S 機動 weave [{j+1}] 補時 {plan.loiter_time_s:.0f}s @bearing {brg:.0f}°'))
            m.append(MissionItem(
                cmd=MAV_CMD_NAV_WAYPOINT, lat=plan.dive_lat, lon=plan.dive_lon, alt=plan_cruise_alt,
                comment=f'Phase 5 俯衝起點 (S 機動後, bearing {brg:.0f}°)'))
            return
        if plan.loiter_time_s > 0.5:        # AIR：盤旋補時
            m.append(MissionItem(
                cmd=MAV_CMD_NAV_LOITER_TIME, lat=plan.pre_strike_lat, lon=plan.pre_strike_lon,
                alt=plan_cruise_alt, param1=float(plan.loiter_time_s), param3=cfg.loiter_radius_m,
                comment=f'NAV_LOITER_TIME {plan.loiter_time_s:.1f}s @bearing {brg:.0f}° ({cfg.timing_mode} 補時)'))
        else:                               # 基準機 / GROUND：直接過預備點
            m.append(MissionItem(
                cmd=MAV_CMD_NAV_WAYPOINT, lat=plan.pre_strike_lat, lon=plan.pre_strike_lon,
                alt=plan_cruise_alt, comment='Phase 4 預備點 (基準機，無需補時)'))
        m.append(MissionItem(
            cmd=MAV_CMD_NAV_WAYPOINT, lat=plan.dive_lat, lon=plan.dive_lon, alt=plan_cruise_alt,
            comment=f'Phase 5 俯衝起點 (bearing {brg:.0f}°, r_dive={cfg.dive_initiation_m:.0f}m, '
                    f'alt={plan_cruise_alt:.0f}m)'))

    def _build_mission(self, plan: UAVStrikePlan,
                        target: Target,
                        cfg: FormationConfig) -> List[MissionItem]:
        """
        組裝單機 QGC WPL 110 任務序列：

            0. HOME (NAV_WAYPOINT @ base, alt=0, current=1)
            1. NAV_TAKEOFF @ base, alt=takeoff_alt
            2. DO_CHANGE_SPEED → cruise_speed
            3..5. 編隊巡航 NAV_WAYPOINT (取樣點)
            6. NAV_LOITER_TIME @ pre_strike (STOT 補時)
            7. 俯衝起點 NAV_WAYPOINT @ dive_pt, alt=cruise
            8. 終端目標 NAV_WAYPOINT @ target, alt=target.alt
        """
        m: List[MissionItem] = []

        # 本機實際起飛位置（slot 偏移後）— 與 SITL spawn home 對齊，避免 ArduPlane
        # 因 home 與 NAV_TAKEOFF 距離過遠而拒絕 AUTO 起飛或試圖飛回 base。
        home_lat = plan.slot_lat if abs(plan.slot_lat) > 1e-9 else plan.base_lat
        home_lon = plan.slot_lon if abs(plan.slot_lon) > 1e-9 else plan.base_lon

        # 0) HOME @ slot 起飛點（QGC WPL 規範要求 seq=0 為 HOME）
        m.append(MissionItem(
            cmd=MAV_CMD_NAV_WAYPOINT,
            lat=home_lat, lon=home_lon, alt=0.0,
            comment=f'HOME (sysid={plan.sysid} {plan.role.upper()})',
        ))
        # ⚠️ 不在 mission 中插入 NAV_DELAY (cmd=93) ：
        #   實測 ArduPlane 在 NAV_TAKEOFF *之前* 的 NAV_DELAY 會被忽略
        #   (官方文件：「需在空中才會 wait at current location」)。
        # 改採 GCS 端排程：strike_controller.launch_kamikaze_synchronized()
        #   會用 QTimer 依 plan.takeoff_delay_s 對各 SITL link 延遲送 AUTO 命令。
        # 這樣才能真正讓較近的 UAV 在跑道上停留再起飛。
        # 1) TAKEOFF @ slot 起飛點 — 起飛仰角採用使用者指定值（典型 8-15°，固定翼不可垂直）
        m.append(MissionItem(
            cmd=MAV_CMD_NAV_TAKEOFF,
            lat=home_lat, lon=home_lon, alt=cfg.takeoff_alt_m,
            param1=float(cfg.takeoff_pitch_deg),    # 起飛俯仰角 (deg)
            comment=f'NAV_TAKEOFF (Phase 1 — 仰角 {cfg.takeoff_pitch_deg:.0f}° 斜爬升)',
        ))
        # 2) 統一巡航空速
        m.append(MissionItem(
            cmd=MAV_CMD_DO_CHANGE_SPEED,
            param1=1.0,                          # type: 1 = airspeed
            param2=cfg.cruise_speed_mps,         # value
            param3=-1.0,                         # throttle %, -1 = no change
            comment=f'DO_CHANGE_SPEED → {cfg.cruise_speed_mps:.1f} m/s',
        ))
        # 該機個體巡航高度（用於 cruise/loiter/dive 起點）
        plan_cruise_alt = plan.cruise_alt if plan.cruise_alt > 1.0 else cfg.cruise_alt_m

        # 3..) 寫入完整爬升軌跡 + 編隊巡航取樣點
        # 之前只寫 cruise_alt 點 → 飛機從 NAV_TAKEOFF (80m) 要直接跳到 500m+ 的
        # waypoint，ArduPlane TECS 來不及爬升，飛機卡在低高度且 4 機橫向只有
        # 30m 並排 → 碰撞危險。改為「跳過 alt=0 的 HOME 點，其餘所有 climb_pts
        # 與 cruise_pts 都寫入 mission」，讓 ArduPlane 沿 12° 斜線階段爬升。
        # （只跳過第一個 alt≈0 的點，因為它與 NAV_TAKEOFF 起點重複）
        traj_pts: List[Tuple[float, float, float]] = []
        for pt in plan.formation_path:
            alt_pt = pt[2] if len(pt) >= 3 else plan_cruise_alt
            # 跳過離地點（alt < 1m，與 HOME / NAV_TAKEOFF 起點重複）
            # 也跳過低於 takeoff_alt 的點（NAV_TAKEOFF 命令本身會處理 0→takeoff_alt）
            if alt_pt < cfg.takeoff_alt_m + 1.0:
                continue
            traj_pts.append(pt)
        if not traj_pts:
            traj_pts = plan.formation_path

        # 區分爬升段與巡航段（僅供 comment 顯示用）
        for i, pt in enumerate(traj_pts):
            lat, lon = pt[0], pt[1]
            alt = pt[2] if len(pt) >= 3 else plan_cruise_alt
            phase = 'CLIMB' if alt < plan_cruise_alt - 1.0 else 'CRUISE'
            m.append(MissionItem(
                cmd=MAV_CMD_NAV_WAYPOINT,
                lat=lat, lon=lon, alt=alt,
                comment=f'Phase 1 {phase} [{i+1}/{len(traj_pts)}] '
                        f'{plan.role} alt={alt:.0f}m',
            ))
        # 4-5) 補時段 + 俯衝起點（S 機動 / 盤旋 / 直接），拆到專責方法（SRP）
        self._append_time_consumption(m, plan, cfg, plan_cruise_alt)
        # 6) 終端目標（俯衝終點 + 撞擊點）
        # param2=30m acceptance — 對固定翼撞擊任務最實用的精度：
        #   太小 (3m)：飛機反覆嘗試精準命中 → S-turn / U-turn 鋸齒
        #   太大 (80m)：飛機在 80m 外就 advance → 沒撞到
        #   30m：飛機接近 30m 才 advance，alt 此時通常 < 50m，仍會撞地
        m.append(MissionItem(
            cmd=MAV_CMD_NAV_WAYPOINT,
            lat=target.lat, lon=target.lon, alt=target.alt,
            param2=30.0,   # 30m acceptance，固定翼撞擊任務實用值
            comment=f'TERMINAL TARGET — {target.name} '
                    f'(θ_max={cfg.max_dive_angle_deg:.0f}°, accept=30m)',
        ))
        # 7) NAV_LOITER_UNLIM 放在「攻擊方向延伸線遠端」 — 移除原本的 OVERSHOOT 點
        # 為什麼移除 OVERSHOOT 200m？實測它造成 V 字鋸齒：
        #   飛機到 target 附近 cross-track 觸發 advance 後，必須朝 attack_dir 200m
        #   位置飛 → 部分 UAV (back ±90° 兩翼) 的 attack_dir 與飛機進場方向差很多 →
        #   被迫做大幅橫向修正 → 看起來像折角鋸齒。
        # 改為直接在 attack_dir 延伸 2000m + alt=200m 放 LOITER：
        #   ① 飛機過 target 後沿同一直線繼續飛 2000m（不需橫向修正）
        #   ② 期間平緩爬升到 200m（自然脫離過程）
        #   ③ 撞地的 UAV 留在地面（CRASH_DETECT=0 不 disarm）
        #   ④ 沒撞中的 UAV 沿原方向飛 2000m 後在 alt=200m 盤旋（不 RTL）
        attack_dir = (plan.attack_bearing_deg + 180.0) % 360.0
        loiter_lat, loiter_lon = destination(
            target.lat, target.lon, attack_dir, 2000.0,
        )
        loiter_alt = float(target.alt) + 200.0
        m.append(MissionItem(
            cmd=MAV_CMD_NAV_LOITER_UNLIM,
            lat=loiter_lat, lon=loiter_lon, alt=loiter_alt,
            param3=cfg.loiter_radius_m,
            comment=f'NAV_LOITER_UNLIM @ {attack_dir:.0f}° +2000m '
                    f'alt={loiter_alt:.0f}m (防 RTL + 撞擊脫離)',
        ))

        return m

    # ──────────────────────────────────────────────────────────────────
    # QGC WPL 110 匯出
    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def to_qgc_wpl(plan: UAVStrikePlan) -> str:
        """單架 UAV 的 QGC WPL 110 字串。"""
        lines = ['QGC WPL 110']
        for seq, it in enumerate(plan.mission):
            current = 1 if seq == 0 else 0      # 第 0 項標記為 current
            frame = 0 if seq == 0 else 3        # HOME=GLOBAL, 其餘=GLOBAL_RELATIVE
            autoc = 1                           # autocontinue
            line = (
                f'{seq}\t{current}\t{frame}\t{it.cmd}\t'
                f'{it.param1:.6f}\t{it.param2:.6f}\t{it.param3:.6f}\t{it.param4:.6f}\t'
                f'{it.lat:.7f}\t{it.lon:.7f}\t{it.alt:.6f}\t{autoc}'
            )
            lines.append(line)
        return '\n'.join(lines) + '\n'

    @classmethod
    def export_qgc_wpl(cls, plans: Sequence[UAVStrikePlan],
                        out_dir: Path,
                        prefix: str = 'diamond_strike_uav') -> List[Path]:
        """
        匯出 4 個 .waypoints + 一份任務簡報 .txt。

        Returns
        -------
        list[Path]
            實際寫入的檔案路徑（含簡報）
        """
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        written: List[Path] = []

        for p in plans:
            fp = out_dir / f'{prefix}_{p.sysid:02d}_{p.role}.waypoints'
            fp.write_text(cls.to_qgc_wpl(p), encoding='utf-8')
            written.append(fp)

        # 任務簡報
        brief_lines = [
            '=== Diamond Swarm Strike — Mission Brief ===',
            '',
            'Phase 1 NAV_TAKEOFF — 共用基地直線爬升（非螺旋）',
            'Phase 2 菱形編隊巡航 — Leader / Rear / Left / Right',
            'Phase 3 決斷邊界（解散）',
            'Phase 4 NAV_LOITER_TIME @ 預備點（STOT 補時）',
            'Phase 5 4 向同時俯衝命中',
            '',
            'UAV 任務摘要：',
        ]
        for p in plans:
            brief_lines.append(
                f'  UAV{p.sysid:02d} [{p.role:<6s}]  '
                f'attack@{p.attack_bearing_deg:5.0f}°  '
                f'leg={p.final_leg_length_m:7.0f}m  '
                f't={p.final_leg_time_s:6.1f}s  '
                f'loiter={p.loiter_time_s:5.1f}s'
            )
        brief = out_dir / f'{prefix}_brief.txt'
        brief.write_text('\n'.join(brief_lines) + '\n', encoding='utf-8')
        written.append(brief)
        return written
