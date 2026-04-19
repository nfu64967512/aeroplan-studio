"""
VTOLSwarmStrikePlanner — VTOL 蜂群末端打擊規劃器
==================================================

針對「VTOL 垂直起降自殺式無人機」(類勁蜂三型 / Switchblade 600 類) 的
蜂群打擊規劃，整合以下核心能力：

    1. 可擴充的 Target 類別 (含速度向量，預留移動目標介面)
    2. VTOL 三階段飛行輪廓 (VTOL 起飛 → Phase 2 巡航 → Phase 3 末端衝刺)
    3. STOT / DTOT 雙模式 — 以「2 km 邊界」為時間協同基準
    4. CEP 末端分佈 — 多機命中點分散涵蓋目標誤差圓
    5. 高度錯層避障 + 360° 全向包圍進場

與 :mod:`core.strike.advanced_swarm_strike_planner` 的核心差異：
    - 計算基準：TOT 以「抵達 2 km 邊界」而非「命中目標」
    - 飛行模式：VTOL (NAV_VTOL_TAKEOFF + DO_VTOL_TRANSITION) 而非固定翼直接起飛
    - 末端觸發：抵達 2 km 時切換高速模式 + 觸發 AI 影像尋標 (IMAGE_START_CAPTURE)

MAVLink 指令對照
----------------
    MAV_CMD_DO_SET_HOME             (179)
    MAV_CMD_NAV_VTOL_TAKEOFF         (84)  — 垂直起飛
    MAV_CMD_DO_VTOL_TRANSITION     (3000)  — MC ↔ FW 轉換 (param1=3:MC / 4:FW)
    MAV_CMD_DO_CHANGE_SPEED         (178)  — 巡航/末端速度切換
    MAV_CMD_NAV_LOITER_TIME          (19)  — 時間協同補時盤旋
    MAV_CMD_NAV_WAYPOINT             (16)  — 巡航/邊界/命中航點
    MAV_CMD_IMAGE_START_CAPTURE    (2000)  — AI 影像辨識/尋標觸發
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

# 復用 swarm_strike_planner 的 Dubins 與地理工具
from core.strike.swarm_strike_planner import (
    MissionItem, dubins_shortest_length,
    _haversine, _bearing_deg, _destination, _angular_diff,
    _MAV_FRAME_REL, _R_EARTH,
)
from utils.file_io import create_waypoint_line, write_waypoints
from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  常數與單位換算
# ═══════════════════════════════════════════════════════════════════════

KNOTS_TO_MPS: float = 1852.0 / 3600.0   # = 0.51444 m/s per knot


def knots_to_mps(kts: float) -> float:
    """節 (knot) → 公尺/秒 (m/s)"""
    return kts * KNOTS_TO_MPS


def mps_to_knots(mps: float) -> float:
    """公尺/秒 (m/s) → 節 (knot)"""
    return mps / KNOTS_TO_MPS


# ── MAVLink 指令 ID ─────────────────────────────────────────────────
class VTOLMAVCmd:
    NAV_WAYPOINT           = 16
    NAV_LOITER_TIME        = 19
    NAV_VTOL_TAKEOFF       = 84
    DO_CHANGE_SPEED        = 178
    DO_SET_HOME            = 179
    DO_VTOL_TRANSITION     = 3000   # param1: 3=MC (多旋翼) / 4=FW (固定翼)
    IMAGE_START_CAPTURE    = 2000


# ═══════════════════════════════════════════════════════════════════════
#  Target 類別 — 可擴充移動目標介面
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class Target:
    """打擊目標 (含速度向量，預留移動目標擴充)

    Attributes
    ----------
    lat, lon, alt :
        目標當前位置 (WGS-84 緯度、經度、海拔公尺)
    vx_mps, vy_mps, vz_mps :
        目標速度向量 (ENU 座標系：x=東, y=北, z=上，單位 m/s)
        靜態目標預設為 0；移動目標由上層填入最新觀測值
    name :
        顯示名稱
    cep_m :
        圓形公算誤差 (Circular Error Probable, m)
        用於多機末端 CEP 分佈；預設 10 m
    """
    lat: float
    lon: float
    alt: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    name: str = 'TGT'
    cep_m: float = 10.0

    # ─── Public API：任何 TOT/IP 計算都必須透過以下方法 ───
    def is_stationary(self) -> bool:
        """目標是否視為靜態 (速度向量各分量 < 0.1 m/s)"""
        return (abs(self.vx_mps) < 0.1 and
                abs(self.vy_mps) < 0.1 and
                abs(self.vz_mps) < 0.1)

    def position_at(self, t_sec: float) -> Tuple[float, float, float]:
        """回傳 t_sec 秒後目標的預測位置 (lat, lon, alt)

        Parameters
        ----------
        t_sec :
            從當前時刻 (t=0) 起算的秒數；0 表示現在

        Returns
        -------
        (lat, lon, alt) :
            目標於 t_sec 秒後的預測位置

        Notes
        -----
        - 靜態目標：直接回傳 (lat, lon, alt)，忽略 t_sec
        - 動態目標：積分 ENU 速度向量 → 增量轉經緯度
          dlat (度) = (vy × t) / R_earth           (線性近似)
          dlon (度) = (vx × t) / (R_earth × cos(lat))
        """
        if self.is_stationary():
            return (self.lat, self.lon, self.alt)

        # 積分速度 → 位置增量 (m)
        de = self.vx_mps * t_sec    # 東向位移
        dn = self.vy_mps * t_sec    # 北向位移
        du = self.vz_mps * t_sec    # 上向位移

        # 公尺 → 度 (線性近似，精度在小範圍足夠)
        coslat = math.cos(math.radians(self.lat))
        dlat_deg = math.degrees(dn / _R_EARTH)
        dlon_deg = math.degrees(de / (_R_EARTH * max(coslat, 1e-9)))

        return (self.lat + dlat_deg,
                self.lon + dlon_deg,
                self.alt + du)

    def intercept_point_at(self, t_sec: float,
                           bearing_deg: float,
                           distance_m: float) -> Tuple[float, float]:
        """回傳目標在 t_sec 時刻、於指定方位/距離的攔截點 (lat, lon)

        用途：為每架 UAV 計算「2 km 邊界進場點」IP_k
        - 靜態目標：等同於 destination(tgt, bearing, distance)
        - 動態目標：先預測目標在 t_sec 的位置，再計算 IP
        """
        tgt_lat, tgt_lon, _ = self.position_at(t_sec)
        return _destination(tgt_lat, tgt_lon, bearing_deg, distance_m)


# ═══════════════════════════════════════════════════════════════════════
#  UAV 與相關列舉
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class VTOLUAV:
    """VTOL 自殺式無人機"""
    uav_id: int
    lat: float
    lon: float
    alt: float = 0.0
    name: str = ''

    def __post_init__(self) -> None:
        if not self.name:
            self.name = f'VTOL-{self.uav_id}'


class AttackMode(Enum):
    STOT = 'STOT'       # Simultaneous — 同秒突破 2 km 邊界
    DTOT = 'DTOT'       # Distributed  — 間隔 Δ 秒依序突破邊界


class FeasibilityStatus(str, Enum):
    OK              = 'OK'
    SPEED_ADJUSTED  = 'SPEED_ADJUSTED'
    LOITER_INSERTED = 'LOITER_INSERTED'
    INFEASIBLE      = 'INFEASIBLE'


# ═══════════════════════════════════════════════════════════════════════
#  規劃結果資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class VTOLPlan:
    """單架 VTOL UCAV 的完整打擊規劃"""
    uav: VTOLUAV
    slot_index: int                 # DTOT 命中順序 (0 = 最先突破邊界)

    # ── 空間協同 ─────────────────────────────────────────────
    attack_heading_deg: float       # 末端進場方位角 ψ_k
    boundary_lat: float             # 2 km 邊界進場點 (IP) 經緯度
    boundary_lon: float
    impact_lat: float               # CEP 微調後的實際命中點
    impact_lon: float
    path_length_m: float            # Dubins 路徑 (起飛 → IP) 長度

    # ── 高度錯層 ─────────────────────────────────────────────
    cruise_alt_m: float             # Phase 2 巡航高度
    terminal_alt_m: float           # Phase 3 衝刺高度 (= cruise)

    # ── 時間協同 (基於 2 km 邊界) ─────────────────────────────
    t_boundary_sec: float           # 突破 2 km 邊界的時刻 (由 t=0 起算)
    t_nominal_sec: float            # 不補時直達邊界的時間 (L/V_cruise + T_vtol)
    t_impact_sec: float             # 預估命中時刻 (邊界時間 + 2km/V_terminal)
    cruise_speed_mps: float         # Phase 2 空速
    terminal_speed_mps: float       # Phase 3 空速

    # ── 補時盤旋 ─────────────────────────────────────────────
    loiter_time_sec: float = 0.0
    loiter_turns: float    = 0.0
    loiter_lat: float      = 0.0
    loiter_lon: float      = 0.0
    loiter_radius_m: float = 0.0

    # ── 狀態與 MAVLink 序列 ──────────────────────────────────
    feasibility: FeasibilityStatus = FeasibilityStatus.OK
    warning: str = ''
    mission: List[MissionItem] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════════════
#  VTOLSwarmStrikePlanner 主類別
# ═══════════════════════════════════════════════════════════════════════

class VTOLSwarmStrikePlanner:
    """VTOL 蜂群協同末端打擊規劃器

    Parameters
    ----------
    target :
        打擊目標 (Target 物件，含速度向量以相容未來移動目標)
    uavs :
        參與打擊的 VTOL UCAV 列表
    mode :
        時間協同模式 — STOT 同時突破邊界 / DTOT 間隔突破邊界
    interval_sec :
        DTOT 模式的相鄰突破時刻間隔 (秒)；STOT 忽略

    cruise_speed_kts :
        Phase 2 巡航空速 (節)；預設 50 kts
    terminal_speed_kts :
        Phase 3 末端衝刺空速 (節)；預設 90 kts
    stall_speed_mps :
        失速速度 (V_req 低於此值時強制 Loiter)

    turn_radius_m :
        最小轉彎半徑 (Dubins + Loiter 用)
    base_cruise_alt_m :
        Phase 2 最底層巡航高度
    altitude_step_m :
        各機高度錯層間距 (≥ 30 m)
    min_alt_separation_m :
        避障檢查的最小允許高度差

    terminal_boundary_m :
        Phase 3 切換邊界 (距目標水平距離，預設 2000 m)
    vtol_transition_alt_m :
        VTOL 垂直起飛後轉換至固定翼的高度 (預設 80 m)
    vtol_climb_rate_mps :
        VTOL 垂直爬升率 (預設 5 m/s，用於 T_vtol 估算)

    approach_offset_deg :
        攻擊方位角全域偏移
    tot_margin_sec :
        TOT 安全邊界 (秒)

    enable_ai_seeker :
        是否在 Phase 3 起點插入 IMAGE_START_CAPTURE 指令 (預設 True)
    cep_spread_scheme :
        'ring' (預設): 圍繞目標均勻分佈；'line': 沿進場軸線分散
    """

    def __init__(self,
                 target: Target,
                 uavs: List[VTOLUAV],
                 mode: AttackMode = AttackMode.STOT,
                 interval_sec: float = 0.0,
                 *,
                 # 速度 (節)
                 cruise_speed_kts: float   = 50.0,
                 terminal_speed_kts: float = 90.0,
                 stall_speed_mps: float    = 18.0,
                 # 幾何
                 turn_radius_m: float      = 150.0,
                 base_cruise_alt_m: float  = 400.0,
                 altitude_step_m: float    = 30.0,
                 min_alt_separation_m: float = 25.0,
                 # 2 km 邊界與 VTOL
                 terminal_boundary_m: float    = 2000.0,
                 vtol_transition_alt_m: float  = 80.0,
                 vtol_climb_rate_mps: float    = 5.0,
                 # 其他
                 approach_offset_deg: float = 0.0,
                 tot_margin_sec: float      = 0.0,
                 enable_ai_seeker: bool     = True,
                 cep_spread_scheme: str     = 'ring'):

        # ── 參數驗證 ───────────────────────────────────────────
        if not uavs:
            raise ValueError('uavs 列表為空')
        if mode == AttackMode.DTOT and interval_sec <= 0:
            raise ValueError(f'DTOT 模式需 interval_sec > 0 (得 {interval_sec})')

        # 速度範圍檢查 (參考戰術規範 40~60 / 80~100 kts)
        if not (40.0 <= cruise_speed_kts <= 60.0):
            logger.warning(f'[VTOL] cruise_speed={cruise_speed_kts} kts 超出戰術區間 40~60')
        if not (80.0 <= terminal_speed_kts <= 100.0):
            logger.warning(f'[VTOL] terminal_speed={terminal_speed_kts} kts 超出戰術區間 80~100')
        if terminal_speed_kts <= cruise_speed_kts:
            raise ValueError(
                f'terminal({terminal_speed_kts}) 必須 > cruise({cruise_speed_kts})'
            )

        self.target   = target
        self.uavs     = list(uavs)
        self.mode     = mode
        self.interval_sec = float(interval_sec)

        # 速度 (kts → mps 轉換後儲存)
        self.cruise_speed_kts   = cruise_speed_kts
        self.terminal_speed_kts = terminal_speed_kts
        self.cruise_speed_mps   = knots_to_mps(cruise_speed_kts)
        self.terminal_speed_mps = knots_to_mps(terminal_speed_kts)
        self.stall_speed_mps    = stall_speed_mps

        # 幾何
        self.turn_radius_m      = turn_radius_m
        self.base_cruise_alt_m  = base_cruise_alt_m
        self.altitude_step_m    = altitude_step_m
        self.min_alt_separation_m = min_alt_separation_m

        # VTOL / 邊界
        self.terminal_boundary_m   = terminal_boundary_m
        self.vtol_transition_alt_m = vtol_transition_alt_m
        self.vtol_climb_rate_mps   = vtol_climb_rate_mps
        self.t_vtol_sec = vtol_transition_alt_m / max(vtol_climb_rate_mps, 0.5)

        self.approach_offset_deg = approach_offset_deg
        self.tot_margin_sec      = tot_margin_sec
        self.enable_ai_seeker    = enable_ai_seeker
        self.cep_spread_scheme   = cep_spread_scheme

        # 結果快取
        self._plans: List[VTOLPlan] = []
        self._base_t_boundary: float = 0.0

    # ═════════════════════════════════════════════════════════════════
    #  公開 API
    # ═════════════════════════════════════════════════════════════════

    def plan(self) -> List[VTOLPlan]:
        """執行完整 VTOL 蜂群規劃"""
        logger.info(
            f'[VTOLSwarm] 模式={self.mode.value}'
            + (f' (Δ={self.interval_sec:.1f}s)' if self.mode == AttackMode.DTOT else '')
            + f', target={self.target.name}, 目標速度='
            + ('STATIC' if self.target.is_stationary() else 'MOVING')
            + f', N={len(self.uavs)}, V_cruise={self.cruise_speed_kts:.0f}kts '
              f'(={self.cruise_speed_mps:.2f}m/s), V_term={self.terminal_speed_kts:.0f}kts '
              f'(={self.terminal_speed_mps:.2f}m/s)'
        )

        # Step 1: 360° 進場向量分配
        assigned = self._assign_omnidirectional_slots()

        # Step 2: CEP 末端分佈 + Phase 3 邊界點
        cep_offsets = self._compute_cep_dispersion(len(self.uavs))

        # Step 3: 逐機生成基礎計畫 (T_vtol + Dubins 到邊界 + CEP 命中點)
        plans: List[VTOLPlan] = []
        for i, (uav, heading) in enumerate(assigned):
            plans.append(self._build_base_plan(uav, heading, cep_offsets[i]))

        # Step 4: STOT / DTOT 時間協同 (以「抵達 2 km 邊界」為基準)
        if self.mode == AttackMode.STOT:
            self._coordinate_stot(plans)
        else:
            plans = self._coordinate_dtot(plans)   # DTOT 會重排

        # Step 5: 高度錯層 (依 slot_index)
        self._assign_altitude_layers(plans)

        # Step 6: 生成 MAVLink 任務序列
        for p in plans:
            p.mission = self._build_mission(p)

        self._plans = plans
        self._log_summary()
        return plans

    @property
    def base_t_boundary_sec(self) -> float:
        """slot 0 抵達 2 km 邊界的時刻"""
        return self._base_t_boundary

    @property
    def plans(self) -> List[VTOLPlan]:
        return list(self._plans)

    def export_qgc_wpl(self, export_dir: str) -> List[str]:
        """匯出每架 UCAV 的 QGC WPL 110 航點檔 + 任務簡報"""
        if not self._plans:
            raise RuntimeError('請先呼叫 plan()')
        os.makedirs(export_dir, exist_ok=True)

        files: List[str] = []
        for p in self._plans:
            fname = (f'VTOL_{self.mode.value}_slot{p.slot_index:02d}_{p.uav.name}'
                     f'_psi{p.attack_heading_deg:03.0f}_cruise{p.cruise_speed_mps:04.1f}'
                     f'_term{p.terminal_speed_mps:04.1f}_t{p.t_boundary_sec:04.0f}s.waypoints')
            fname = fname.replace('/', '-').replace('\\', '-')
            fpath = os.path.join(export_dir, fname)

            lines = ['QGC WPL 110']
            for seq, item in enumerate(p.mission):
                current = 1 if seq == 0 and item.cmd == VTOLMAVCmd.DO_SET_HOME else 0
                lines.append(create_waypoint_line(
                    seq=seq, command=item.cmd,
                    lat=item.lat, lon=item.lon, alt=item.alt,
                    param1=item.param1, param2=item.param2,
                    param3=item.param3, param4=item.param4,
                    frame=_MAV_FRAME_REL,
                    current=current, autocontinue=1,
                ))
            if write_waypoints(fpath, lines):
                files.append(fpath)
        self._write_briefing(export_dir)
        return files

    # ═════════════════════════════════════════════════════════════════
    #  Step 1：360° 全向進場向量分配
    # ═════════════════════════════════════════════════════════════════

    def _assign_omnidirectional_slots(self) -> List[Tuple[VTOLUAV, float]]:
        """依 UAV 相對目標的方位角，分配至 360° 均勻攻擊向量"""
        n = len(self.uavs)
        if n == 1:
            u = self.uavs[0]
            brg = _bearing_deg(u.lat, u.lon, self.target.lat, self.target.lon)
            return [(u, brg)]

        by_bearing = sorted(
            self.uavs,
            key=lambda u: _bearing_deg(
                u.lat, u.lon, self.target.lat, self.target.lon
            ),
        )
        vectors = sorted(
            (self.approach_offset_deg + 360.0 / n * k) % 360.0
            for k in range(n)
        )
        # 圓形旋轉對齊
        best_shift, best_cost = 0, float('inf')
        for shift in range(n):
            cost = sum(
                _angular_diff(
                    _bearing_deg(by_bearing[i].lat, by_bearing[i].lon,
                                 self.target.lat, self.target.lon),
                    vectors[(i + shift) % n],
                )
                for i in range(n)
            )
            if cost < best_cost:
                best_cost, best_shift = cost, shift
        return [(by_bearing[i], vectors[(i + best_shift) % n]) for i in range(n)]

    # ═════════════════════════════════════════════════════════════════
    #  Step 2：CEP 末端分佈
    # ═════════════════════════════════════════════════════════════════

    def _compute_cep_dispersion(self, n: int) -> List[Tuple[float, float]]:
        """為 N 架 UCAV 生成 CEP 末端微調偏移量 (bearing, distance)。

        'ring' 方案 (預設)：
          在以目標為中心、半徑 = cep_m 的圓上均勻分佈 N 個命中點。
          第 i 架 UCAV 的偏移方位 = (360°/N)·i，距離 = cep_m × 0.7
          （若 N=1，距離設 0，直接打中心）

        'line' 方案：
          沿進場軸線分散 [-cep_m, +cep_m]，模擬彈藥散佈。
        """
        cep = max(self.target.cep_m, 0.5)
        offsets: List[Tuple[float, float]] = []

        if n == 1:
            return [(0.0, 0.0)]   # 單機直接打中心

        if self.cep_spread_scheme == 'line':
            # 沿某軸線 (預設 90°) 等距分散
            for i in range(n):
                frac = -1.0 + 2.0 * i / max(n - 1, 1)
                offsets.append((90.0, frac * cep))
            return offsets

        # 'ring' 預設：360° 均勻偏移，距離 = 0.7 × CEP
        for i in range(n):
            ang = (360.0 / n * i) % 360.0
            offsets.append((ang, 0.7 * cep))
        return offsets

    # ═════════════════════════════════════════════════════════════════
    #  Step 3：基礎計畫 (VTOL 時間 + Dubins 到邊界 + CEP 命中點)
    # ═════════════════════════════════════════════════════════════════

    def _build_base_plan(self, uav: VTOLUAV,
                         attack_heading_deg: float,
                         cep_offset: Tuple[float, float]) -> VTOLPlan:
        """建立單架 UCAV 的基礎規劃 (尚未協同時間/高度)。

        關鍵幾何：
          - Boundary point (IP) = 2 km 邊界進場點
              位於目標的 (attack_heading + 180°) 方位，距離 2000 m
          - Impact point = target + CEP offset
              靜態目標直接套用；動態目標需 target.intercept_point_at(t_hit)
              （在 _coordinate_* 確定 t 後再更新）
          - Dubins 路徑: UAV 起飛 → IP
              起始航向 = bearing(UAV → IP)
              終止航向 = attack_heading (已對準目標)
        """
        # ── 2 km 邊界進場點 (先用 t=0 的靜態位置) ─────────────
        bnd_lat, bnd_lon = self.target.intercept_point_at(
            t_sec=0.0,
            bearing_deg=(attack_heading_deg + 180.0) % 360.0,
            distance_m=self.terminal_boundary_m,
        )

        # ── 命中點：目標 + CEP 偏移 ───────────────────────────
        cep_bearing, cep_dist = cep_offset
        tgt_lat, tgt_lon, _ = self.target.position_at(0.0)
        if cep_dist > 0.1:
            impact_lat, impact_lon = _destination(
                tgt_lat, tgt_lon, cep_bearing, cep_dist
            )
        else:
            impact_lat, impact_lon = tgt_lat, tgt_lon

        # ── Dubins 長度: UAV 起飛 → 2 km 邊界 ────────────────
        start_heading = _bearing_deg(uav.lat, uav.lon, bnd_lat, bnd_lon)
        dubins_horiz = dubins_shortest_length(
            uav.lat, uav.lon, start_heading,
            bnd_lat, bnd_lon, attack_heading_deg,
            self.turn_radius_m,
        )

        # ── 名義時間：T_vtol (垂直爬升) + 巡航時間 ─────────────
        t_cruise = dubins_horiz / max(self.cruise_speed_mps, 0.1)
        t_nominal = self.t_vtol_sec + t_cruise

        # ── 預估命中時刻 = 突破邊界 + 2 km / V_terminal ────────
        t_impact = t_nominal + self.terminal_boundary_m / self.terminal_speed_mps

        return VTOLPlan(
            uav=uav,
            slot_index=-1,
            attack_heading_deg=attack_heading_deg,
            boundary_lat=bnd_lat,
            boundary_lon=bnd_lon,
            impact_lat=impact_lat,
            impact_lon=impact_lon,
            path_length_m=dubins_horiz,
            cruise_alt_m=self.base_cruise_alt_m,    # Step 5 重填
            terminal_alt_m=self.base_cruise_alt_m,
            t_boundary_sec=0.0,                      # Step 4 填
            t_nominal_sec=t_nominal,
            t_impact_sec=t_impact,
            cruise_speed_mps=self.cruise_speed_mps,
            terminal_speed_mps=self.terminal_speed_mps,
        )

    # ═════════════════════════════════════════════════════════════════
    #  Step 4a：STOT 時間協同 (以 2 km 邊界為基準)
    # ═════════════════════════════════════════════════════════════════

    def _coordinate_stot(self, plans: List[VTOLPlan]) -> None:
        """STOT：所有 UCAV 在同一秒突破 2 km 邊界後同時加速衝刺。

        2026 重構：TOT 排程委派給 core.strike.time_coordination；
        本方法僅處理 VTOL 特有的 t_impact 計算與動態目標攔截。
        """
        from core.strike.time_coordination import compute_tot_schedule
        base_t, slots = compute_tot_schedule(
            nominal_times_sec=[p.t_nominal_sec for p in plans],
            mode='STOT',
            tot_margin_sec=self.tot_margin_sec,
            payloads=plans,
        )
        self._base_t_boundary = base_t
        logger.info(f'[STOT] 基準突破邊界時刻 = {base_t:.2f} s')

        for slot in slots:
            p = slot.payload
            p.slot_index = slot.slot_index
            p.t_boundary_sec = slot.arrival_time_sec
            self._solve_speed_or_loiter(p, p.t_boundary_sec)
            p.t_impact_sec = (p.t_boundary_sec
                              + self.terminal_boundary_m / p.terminal_speed_mps)
            self._refresh_intercept_if_moving(p)

    # ═════════════════════════════════════════════════════════════════
    #  Step 4b：DTOT 時間協同 (間隔突破邊界)
    # ═════════════════════════════════════════════════════════════════

    def _coordinate_dtot(self, plans: List[VTOLPlan]) -> List[VTOLPlan]:
        """DTOT：slot k 在 Base + k·Δ 秒突破 2 km 邊界；最遠者先。

        2026 重構：TOT 排程委派給共用模組 (longest_first 排序)。
        """
        from core.strike.time_coordination import compute_tot_schedule
        base_t, slots = compute_tot_schedule(
            nominal_times_sec=[p.t_nominal_sec for p in plans],
            mode='DTOT',
            interval_sec=self.interval_sec,
            dtot_order='longest_first',
            tot_margin_sec=self.tot_margin_sec,
            payloads=plans,
        )
        self._base_t_boundary = base_t
        logger.info(
            f'[DTOT] 基準 = {base_t:.2f}s, Δ = {self.interval_sec:.1f}s → '
            + ', '.join(f'{base_t + k * self.interval_sec:.1f}'
                        for k in range(len(plans)))
        )
        for slot in slots:
            p = slot.payload
            p.slot_index = slot.slot_index
            p.t_boundary_sec = slot.arrival_time_sec
            self._solve_speed_or_loiter(p, p.t_boundary_sec)
            p.t_impact_sec = (p.t_boundary_sec
                              + self.terminal_boundary_m / p.terminal_speed_mps)
            self._refresh_intercept_if_moving(p)
        return sorted(plans, key=lambda p: p.slot_index)

    # ═════════════════════════════════════════════════════════════════
    #  Step 4c：速度/盤旋決策 (以邊界時間為目標)
    # ═════════════════════════════════════════════════════════════════

    def _solve_speed_or_loiter(self, p: VTOLPlan, t_target: float) -> None:
        """給定目標突破邊界時刻 t_target，決定 UCAV 的巡航空速與 Loiter。

        注意：t_target 包含 T_vtol，巡航實際可用時間 = t_target − T_vtol。
        """
        L = p.path_length_m
        t_available = t_target - self.t_vtol_sec

        if t_available <= 0.1:
            p.feasibility = FeasibilityStatus.INFEASIBLE
            p.warning = f't_target={t_target:.2f}s 扣 T_vtol 後不夠巡航'
            return

        v_req_ideal = L / t_available

        # ── V_req 超出最大速度 → 不可行 ─────────────────────
        if v_req_ideal > self.terminal_speed_mps + 1e-3:
            p.cruise_speed_mps = self.cruise_speed_mps
            p.feasibility = FeasibilityStatus.INFEASIBLE
            p.warning = (f'V_req={v_req_ideal:.2f} > V_term_max; '
                         f'路徑太長 / 時刻太早')
            return

        # ── V_req < V_stall → 強制 Loiter ─────────────────────
        if v_req_ideal < self.stall_speed_mps - 1e-3:
            p.cruise_speed_mps = self.stall_speed_mps
            p.feasibility = FeasibilityStatus.LOITER_INSERTED
            t_fly_stall = L / self.stall_speed_mps
            p.loiter_time_sec = max(0.0, t_available - t_fly_stall)
            t_lap = 2.0 * math.pi * self.turn_radius_m / self.stall_speed_mps
            p.loiter_turns = p.loiter_time_sec / max(t_lap, 0.1)
            p.loiter_lat = p.uav.lat
            p.loiter_lon = p.uav.lon
            p.loiter_radius_m = self.turn_radius_m
            p.warning = (f'V_req={v_req_ideal:.2f} < V_stall → 夾 V_stall + '
                         f'Loiter {p.loiter_time_sec:.1f}s ({p.loiter_turns:.1f} 圈)')
            return

        # ── DTOT 專屬：保持 V_cruise 以 Loiter 補差 (符合戰術) ─
        if self.mode == AttackMode.DTOT:
            t_at_cruise = L / self.cruise_speed_mps
            if t_at_cruise < t_available - 0.5:
                p.cruise_speed_mps = self.cruise_speed_mps
                p.feasibility = FeasibilityStatus.LOITER_INSERTED
                p.loiter_time_sec = t_available - t_at_cruise
                t_lap = 2.0 * math.pi * self.turn_radius_m / self.cruise_speed_mps
                p.loiter_turns = p.loiter_time_sec / max(t_lap, 0.1)
                p.loiter_lat = p.uav.lat
                p.loiter_lon = p.uav.lon
                p.loiter_radius_m = self.turn_radius_m
                p.warning = (f'DTOT 延遲：V=V_cruise + Loiter '
                             f'{p.loiter_time_sec:.1f}s ({p.loiter_turns:.1f} 圈)')
                return

        # ── 案例 C：V_req 在 [V_stall, cruise] → 調速 ─────────
        # 若 V_req > V_cruise 則 FEASIBLE (會超過戰術設定的巡航上限，但仍在 V_term 內)
        p.cruise_speed_mps = v_req_ideal
        if abs(v_req_ideal - self.cruise_speed_mps) < 0.3:
            p.feasibility = FeasibilityStatus.OK
        else:
            p.feasibility = FeasibilityStatus.SPEED_ADJUSTED

    # ═════════════════════════════════════════════════════════════════
    #  Step 4d：動態目標攔截點刷新 (為未來移動目標預留)
    # ═════════════════════════════════════════════════════════════════

    def _refresh_intercept_if_moving(self, p: VTOLPlan) -> None:
        """若目標為動態，根據 UCAV 的預估突破/命中時刻刷新 IP / impact 點。

        靜態目標下此方法為 no-op。
        """
        if self.target.is_stationary():
            return

        # 更新 2 km 邊界進場點 (於 t_boundary 時目標的預測位置)
        new_bnd_lat, new_bnd_lon = self.target.intercept_point_at(
            t_sec=p.t_boundary_sec,
            bearing_deg=(p.attack_heading_deg + 180.0) % 360.0,
            distance_m=self.terminal_boundary_m,
        )
        p.boundary_lat, p.boundary_lon = new_bnd_lat, new_bnd_lon

        # 更新命中點 (於 t_impact 時目標的預測位置 + CEP 偏移)
        tgt_lat, tgt_lon, _ = self.target.position_at(p.t_impact_sec)
        # CEP offset 需要從原軌跡取 (這裡簡化：以 attack_heading 180 + 0.7CEP)
        cep_ang = (p.attack_heading_deg + 180.0) % 360.0
        p.impact_lat, p.impact_lon = _destination(
            tgt_lat, tgt_lon, cep_ang, 0.7 * self.target.cep_m
        )
        logger.debug(
            f'[VTOL/moving] {p.uav.name}: 邊界點/命中點已依 t={p.t_boundary_sec:.1f}s/'
            f'{p.t_impact_sec:.1f}s 預測目標位置刷新'
        )

    # ═════════════════════════════════════════════════════════════════
    #  Step 5：高度錯層
    # ═════════════════════════════════════════════════════════════════

    def _assign_altitude_layers(self, plans: List[VTOLPlan]) -> None:
        """依 slot_index 分配巡航 / 末端高度層"""
        for p in plans:
            alt = self.base_cruise_alt_m + p.slot_index * self.altitude_step_m
            p.cruise_alt_m = alt
            p.terminal_alt_m = alt

    # ═════════════════════════════════════════════════════════════════
    #  Step 6：MAVLink 任務序列生成
    # ═════════════════════════════════════════════════════════════════

    def _build_mission(self, p: VTOLPlan) -> List[MissionItem]:
        """生成單架 VTOL UCAV 的 MAVLink 任務序列。

        完整序列結構::

            seq  0  DO_SET_HOME              (起飛點錨定)
            seq  1  NAV_VTOL_TAKEOFF         (垂直起飛 → VTOL 轉換高度)
            seq  2  DO_VTOL_TRANSITION p1=4  (MC → FW 轉換)
            seq  3  DO_CHANGE_SPEED V_cruise (Phase 2 巡航空速)
            seq  4  [NAV_LOITER_TIME]        (補時盤旋；STOT/DTOT 用)
            seq  5  NAV_WAYPOINT             (2 km 邊界進場點 IP)
            seq  6  DO_CHANGE_SPEED V_term   (Phase 3 末端衝刺)
            seq  7  [IMAGE_START_CAPTURE]    (AI 影像尋標觸發)
            seq  8  NAV_WAYPOINT             (CEP 調整後的命中點)
        """
        seq: List[MissionItem] = []

        # 0. DO_SET_HOME
        seq.append(MissionItem(
            cmd=VTOLMAVCmd.DO_SET_HOME,
            lat=p.uav.lat, lon=p.uav.lon, alt=p.uav.alt,
            comment=f'Home (slot {p.slot_index})',
        ))

        # 1. NAV_VTOL_TAKEOFF (垂直起飛至轉換高度)
        seq.append(MissionItem(
            cmd=VTOLMAVCmd.NAV_VTOL_TAKEOFF,
            lat=p.uav.lat, lon=p.uav.lon, alt=self.vtol_transition_alt_m,
            comment=f'VTOL takeoff → {self.vtol_transition_alt_m:.0f} m',
        ))

        # 2. DO_VTOL_TRANSITION (param1=4 → FW 固定翼)
        seq.append(MissionItem(
            cmd=VTOLMAVCmd.DO_VTOL_TRANSITION,
            param1=4.0,                           # 4 = MAV_VTOL_STATE_FW
            comment='Transition MC → FW',
        ))

        # 3. DO_CHANGE_SPEED (Phase 2 巡航空速)
        seq.append(MissionItem(
            cmd=VTOLMAVCmd.DO_CHANGE_SPEED,
            param1=0.0,                            # 0 = Airspeed
            param2=float(p.cruise_speed_mps),
            param3=-1.0,                           # 不改油門
            comment=f'Phase 2 cruise V={mps_to_knots(p.cruise_speed_mps):.0f} kts',
        ))

        # 4. [NAV_LOITER_TIME] (若需補時)
        if (p.feasibility == FeasibilityStatus.LOITER_INSERTED
                and p.loiter_time_sec > 0.5):
            seq.append(MissionItem(
                cmd=VTOLMAVCmd.NAV_LOITER_TIME,
                lat=p.loiter_lat, lon=p.loiter_lon, alt=p.cruise_alt_m,
                param1=float(p.loiter_time_sec),
                param2=0.0,
                param3=float(p.loiter_radius_m),
                param4=1.0,
                comment=f'Loiter {p.loiter_time_sec:.1f}s '
                        f'({p.loiter_turns:.1f} turns) for TOT sync',
            ))

        # 5. NAV_WAYPOINT (2 km 邊界 IP)
        seq.append(MissionItem(
            cmd=VTOLMAVCmd.NAV_WAYPOINT,
            lat=p.boundary_lat, lon=p.boundary_lon, alt=p.cruise_alt_m,
            param2=float(self.turn_radius_m),
            comment=f'2km boundary (ψ={p.attack_heading_deg:.1f}°) '
                    f'@ t={p.t_boundary_sec:.1f}s',
        ))

        # 6. DO_CHANGE_SPEED (Phase 3 末端衝刺)
        seq.append(MissionItem(
            cmd=VTOLMAVCmd.DO_CHANGE_SPEED,
            param1=0.0,
            param2=float(p.terminal_speed_mps),
            param3=-1.0,
            comment=f'Phase 3 TERMINAL SPRINT '
                    f'V={mps_to_knots(p.terminal_speed_mps):.0f} kts',
        ))

        # 7. [IMAGE_START_CAPTURE] (AI 影像尋標觸發)
        if self.enable_ai_seeker:
            seq.append(MissionItem(
                cmd=VTOLMAVCmd.IMAGE_START_CAPTURE,
                param1=0.0,       # 相機 ID (0 = 預設/全部)
                param2=0.0,       # 影像間隔 (0 = 一次性)
                param3=1.0,       # 影像總張數 (1 = 觸發一次)
                param4=0.0,
                comment='AI 影像辨識/末端尋標啟動',
            ))

        # 8. NAV_WAYPOINT (CEP 調整後命中點)
        seq.append(MissionItem(
            cmd=VTOLMAVCmd.NAV_WAYPOINT,
            lat=p.impact_lat, lon=p.impact_lon, alt=self.target.alt,
            param2=max(5.0, self.turn_radius_m * 0.1),
            comment=f'IMPACT {self.target.name} @ t={p.t_impact_sec:.1f}s '
                    f'(CEP-offset)',
        ))

        return seq

    # ═════════════════════════════════════════════════════════════════
    #  日誌與簡報
    # ═════════════════════════════════════════════════════════════════

    def _log_summary(self) -> None:
        logger.info(f'[VTOLSwarm] 規劃完成 (基準 t_boundary={self._base_t_boundary:.2f}s)')
        for p in self._plans:
            loi = f' +Loiter {p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.5 else ''
            logger.info(
                f'  slot {p.slot_index} [{p.feasibility.value:<16s}] '
                f'{p.uav.name}  psi={p.attack_heading_deg:5.1f}deg  '
                f'L_dubins={p.path_length_m:7.0f}m  '
                f'V_c={mps_to_knots(p.cruise_speed_mps):4.1f}kts  '
                f'V_t={mps_to_knots(p.terminal_speed_mps):4.1f}kts  '
                f'alt={p.cruise_alt_m:.0f}m  '
                f't_bnd={p.t_boundary_sec:.1f}s  t_hit={p.t_impact_sec:.1f}s{loi}'
                + (f'   ! {p.warning}' if p.warning else '')
            )

    def _write_briefing(self, export_dir: str) -> None:
        path = os.path.join(export_dir, f'VTOL_{self.mode.value}_briefing.txt')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'VTOL {self.mode.value} 蜂群末端打擊任務簡報\n')
            f.write('=' * 70 + '\n\n')
            f.write(f'戰術模式       ：{self.mode.value}'
                    + (f' (Δ={self.interval_sec:.1f}s)' if self.mode == AttackMode.DTOT else '')
                    + '\n')
            f.write(f'目標           ：{self.target.name} '
                    f'({self.target.lat:.6f}, {self.target.lon:.6f})\n')
            f.write(f'目標類型       ：'
                    + ('靜態' if self.target.is_stationary() else
                       f'動態 (v_enu={self.target.vx_mps:.1f},'
                       f'{self.target.vy_mps:.1f},{self.target.vz_mps:.1f} m/s)')
                    + '\n')
            f.write(f'CEP           ：{self.target.cep_m:.1f} m\n')
            f.write(f'UCAV 數量      ：{len(self._plans)}\n\n')

            f.write(f'=== 速度輪廓 ===\n')
            f.write(f'  Phase 2 巡航   : {self.cruise_speed_kts:.0f} kts '
                    f'= {self.cruise_speed_mps:.2f} m/s\n')
            f.write(f'  Phase 3 末端   : {self.terminal_speed_kts:.0f} kts '
                    f'= {self.terminal_speed_mps:.2f} m/s\n')
            f.write(f'  失速下限       : {self.stall_speed_mps:.1f} m/s\n\n')

            f.write(f'=== VTOL / 邊界 ===\n')
            f.write(f'  VTOL 轉換高度  : {self.vtol_transition_alt_m:.0f} m '
                    f'(T_vtol={self.t_vtol_sec:.1f} s)\n')
            f.write(f'  2 km 邊界距離  : {self.terminal_boundary_m:.0f} m\n')
            f.write(f'  高度錯層       : base={self.base_cruise_alt_m:.0f}m, '
                    f'step={self.altitude_step_m:.0f}m\n\n')

            f.write(f'{"slot":>4s} {"UCAV":<10s} {"psi(deg)":>8s} {"L(m)":>7s} '
                    f'{"alt(m)":>6s} {"Vc(kts)":>8s} {"Vt(kts)":>8s} '
                    f'{"t_bnd(s)":>9s} {"t_hit(s)":>9s} {"Loiter":>9s} '
                    f'{"feasibility":<18s}\n')
            f.write('-' * 115 + '\n')
            for p in self._plans:
                loi = f'{p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.1 else '-'
                f.write(
                    f'{p.slot_index:>4d} {p.uav.name:<10s} '
                    f'{p.attack_heading_deg:>8.1f} '
                    f'{p.path_length_m:>7.0f} '
                    f'{p.cruise_alt_m:>6.0f} '
                    f'{mps_to_knots(p.cruise_speed_mps):>8.1f} '
                    f'{mps_to_knots(p.terminal_speed_mps):>8.1f} '
                    f'{p.t_boundary_sec:>9.1f} '
                    f'{p.t_impact_sec:>9.1f} '
                    f'{loi:>9s} '
                    f'{p.feasibility.value:<18s}\n'
                )
            f.write('\n=== 指令對照 ===\n')
            f.write('  179 DO_SET_HOME        185 NAV_LOITER_TURNS     16 NAV_WAYPOINT\n')
            f.write('   84 NAV_VTOL_TAKEOFF   178 DO_CHANGE_SPEED      19 NAV_LOITER_TIME\n')
            f.write(' 3000 DO_VTOL_TRANSITION(param1 4=FW 3=MC)  2000 IMAGE_START_CAPTURE\n')


# ═══════════════════════════════════════════════════════════════════════
#  Main Demo：3 架 VTOL UCAV 對台北 101 執行 STOT 飽和攻擊
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    # ── 目標：靜態高價值目標 (velocity 全 0，預留移動擴充) ─────
    target = Target(
        lat=25.0330, lon=121.5654, alt=0.0,
        vx_mps=0.0, vy_mps=0.0, vz_mps=0.0,
        name='TGT-Taipei101-HQ',
        cep_m=12.0,                  # 假設 GNSS 誤差 ±12m
    )

    # ── 3 架 VTOL UCAV 從不同方向發射 ───────────────────────
    uavs = [
        VTOLUAV(uav_id=1, lat=25.20, lon=121.85, alt=0.0, name='VTOL-NE'),
        VTOLUAV(uav_id=2, lat=25.15, lon=121.30, alt=0.0, name='VTOL-NW'),
        VTOLUAV(uav_id=3, lat=24.88, lon=121.58, alt=0.0, name='VTOL-S'),
    ]

    # ── 建立規劃器 (STOT 飽和攻擊，3 機同時突破 2km 邊界) ────
    planner = VTOLSwarmStrikePlanner(
        target=target,
        uavs=uavs,
        mode=AttackMode.STOT,
        cruise_speed_kts=50.0,           # Phase 2: 50 kts ≈ 25.7 m/s
        terminal_speed_kts=90.0,         # Phase 3: 90 kts ≈ 46.3 m/s
        stall_speed_mps=18.0,
        turn_radius_m=150.0,
        base_cruise_alt_m=400.0,
        altitude_step_m=30.0,
        terminal_boundary_m=2000.0,
        vtol_transition_alt_m=80.0,      # 垂直爬升至 80m 後轉換
        vtol_climb_rate_mps=5.0,
        approach_offset_deg=0.0,
        enable_ai_seeker=True,
        cep_spread_scheme='ring',
    )

    plans = planner.plan()

    # ── 規劃結果輸出 ─────────────────────────────────────────
    print('\n' + '═' * 90)
    print(f' VTOL STOT 飽和攻擊規劃   '
          f'基準突破邊界時刻 = {planner.base_t_boundary_sec:.2f} s')
    print(f' Phase 2 V_cruise  = {planner.cruise_speed_kts:.0f} kts '
          f'({planner.cruise_speed_mps:.2f} m/s)')
    print(f' Phase 3 V_terminal= {planner.terminal_speed_kts:.0f} kts '
          f'({planner.terminal_speed_mps:.2f} m/s)')
    print(f' CEP               = {target.cep_m:.1f} m   (ring 分佈)')
    print('═' * 90)
    print(f'{"slot":>4s} {"UCAV":<10s} {"psi(°)":>7s} {"L(m)":>7s} '
          f'{"alt(m)":>6s} {"Vc(kts)":>8s} {"Vt(kts)":>8s} '
          f'{"t_bnd(s)":>9s} {"t_hit(s)":>9s} {"Loiter":>9s}')
    print('-' * 90)
    for p in plans:
        loi = f'{p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.1 else '-'
        print(f'{p.slot_index:>4d} {p.uav.name:<10s} '
              f'{p.attack_heading_deg:>7.1f} '
              f'{p.path_length_m:>7.0f} '
              f'{p.cruise_alt_m:>6.0f} '
              f'{mps_to_knots(p.cruise_speed_mps):>8.1f} '
              f'{mps_to_knots(p.terminal_speed_mps):>8.1f} '
              f'{p.t_boundary_sec:>9.1f} '
              f'{p.t_impact_sec:>9.1f} '
              f'{loi:>9s}')

    # ── MAVLink 任務序列 (最先突破的 slot 0) ──────────────────
    print('\n' + '═' * 90)
    print(' MAVLink 任務序列 — slot 0 完整 VTOL 飛行輪廓')
    print('═' * 90)
    p0 = plans[0]
    cmd_name = {
        179: 'DO_SET_HOME', 84: 'NAV_VTOL_TAKEOFF', 3000: 'DO_VTOL_TRANSITION',
        178: 'DO_CHANGE_SPEED', 19: 'NAV_LOITER_TIME', 16: 'NAV_WAYPOINT',
        2000: 'IMAGE_START_CAPTURE',
    }
    for i, item in enumerate(p0.mission):
        name = cmd_name.get(item.cmd, f'CMD_{item.cmd}')
        print(f'  seq {i}  {name:<22s} '
              f'lat={item.lat:10.6f} lon={item.lon:11.6f} alt={item.alt:6.1f}  '
              f'p1={item.param1:6.2f} p2={item.param2:6.2f}')
        print(f'          | {item.comment}')

    # ── 匯出 QGC WPL 110 ──────────────────────────────────
    out = os.path.join(os.getcwd(), 'vtol_stot_out')
    files = planner.export_qgc_wpl(out)
    print(f'\n[匯出] {len(files)} 份 .waypoints → {out}')
    for f in files:
        print(f'  • {os.path.basename(f)}')

    print(f'\n[未來移動目標擴充方式]')
    print(f'  只需將 Target 建構時填入 vx_mps/vy_mps/vz_mps != 0')
    print(f'  即啟用 _refresh_intercept_if_moving() 自動預測攔截點。')
