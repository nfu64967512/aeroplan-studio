"""
AdvancedSwarmStrikePlanner — 高階 OWA-UAV 蜂群末端打擊規劃器
================================================================

與 :mod:`core.strike.swarm_strike_planner` 相比，本模組新增了：

    * **DTOT 間隔打擊模式** (Distributed Time On Target)
        使用者輸入 interval_sec，系統為 UCAV 排序使各機分別在
        T, T+Δ, T+2Δ, ... 秒依序命中目標；透過差異化延遲盤旋實現。

    * **嚴格的 3D 機間避障**
        (1) 高度錯層 (Altitude Layering)
        (2) 盤旋圈重疊檢查
        (3) 俯衝軌跡穿透分析 (Dive Path Penetration)

適用機型：Shahed-136 / Geran-2 類 Delta-wing OWA-UAV
戰術目標：針對「單一高價值目標」的全向包圍 + 時間協同打擊

MAVLink 指令對照
----------------
    MAV_CMD_DO_SET_HOME         (179)
    MAV_CMD_DO_CHANGE_SPEED     (178)  ← STOT 速度協同核心
    MAV_CMD_NAV_TAKEOFF          (22)
    MAV_CMD_NAV_LOITER_TIME      (19)  ← DTOT 時間延遲核心
    MAV_CMD_NAV_WAYPOINT         (16)
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

# 復用 swarm_strike_planner 的資料結構與 Dubins/地理工具
from core.strike.swarm_strike_planner import (
    UAV, Target, MissionItem, MAVCmd,
    dubins_shortest_length,
    _haversine, _bearing_deg, _destination, _angular_diff,
    _MAV_FRAME_REL,
)
from utils.file_io import create_waypoint_line, write_waypoints
from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  模式列舉
# ═══════════════════════════════════════════════════════════════════════

class AttackMode(Enum):
    """時間協同模式"""
    STOT = 'STOT'   # Simultaneous TOT — 同時命中 (all hit at Base_TOT)
    DTOT = 'DTOT'   # Distributed TOT — 間隔命中 (T, T+Δ, T+2Δ, ...)


class FeasibilityStatus(str, Enum):
    """單架 UCAV 的可行性"""
    OK              = 'OK'
    SPEED_ADJUSTED  = 'SPEED_ADJUSTED'    # 透過調整空速達成 TOT
    LOITER_INSERTED = 'LOITER_INSERTED'   # 透過 Loiter 延遲達成 TOT
    INFEASIBLE      = 'INFEASIBLE'        # V_req > V_max，無法達成


# ═══════════════════════════════════════════════════════════════════════
#  資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class UAVPlan:
    """單架 UCAV 的完整規劃結果 (空間 + 高度 + 時間 + MAVLink)"""
    uav: UAV
    slot_index: int                # DTOT 模式的命中順序 (0 = 最先命中)

    # ── 空間協同 (360° 全向突防) ──────────────────────────────
    attack_heading_deg: float      # 末端攻擊方位角 ψ_k (0~360°)
    ip_lat: float                  # Initial Point 緯度
    ip_lon: float                  # Initial Point 經度
    path_length_m: float           # Dubins 3D 路徑長度

    # ── 高度錯層 (避障) ──────────────────────────────────────
    cruise_alt_m: float            # 巡航高度 (各機不同)
    loiter_alt_m: float            # 盤旋等待高度 (通常 = cruise_alt)

    # ── 時間協同 ─────────────────────────────────────────────
    arrival_time_sec: float        # 目標命中時刻 (從 T0 起算)
    nominal_time_sec: float        # L_k / V_cruise
    required_speed_mps: float      # 實際採用的巡航空速
    loiter_time_sec: float = 0.0   # Loiter 等待秒數
    loiter_turns: float    = 0.0   # Loiter 圈數 = loiter_time × V_stall / (2πR)
    loiter_lat: float      = 0.0
    loiter_lon: float      = 0.0
    loiter_radius_m: float = 0.0

    # ── 狀態與 MAVLink 序列 ──────────────────────────────────
    feasibility: FeasibilityStatus = FeasibilityStatus.OK
    warning: str = ''
    mission: List[MissionItem] = field(default_factory=list)


@dataclass
class CollisionReport:
    """避障檢查報告"""
    safe: bool
    min_cruise_alt_separation_m: float      # 兩兩巡航高度的最小差值
    min_loiter_distance_m: float            # 兩兩盤旋中心的最小 2D 距離 (-1 代表無共盤旋)
    dive_penetration_warnings: List[str] = field(default_factory=list)
    altitude_violations: List[str] = field(default_factory=list)
    loiter_violations: List[str] = field(default_factory=list)

    @property
    def all_conflicts(self) -> List[str]:
        return (self.altitude_violations + self.loiter_violations
                + self.dive_penetration_warnings)


# ═══════════════════════════════════════════════════════════════════════
#  核心類別
# ═══════════════════════════════════════════════════════════════════════

class AdvancedSwarmStrikePlanner:
    """
    高階 STOT/DTOT 蜂群打擊規劃器

    典型使用流程::

        planner = AdvancedSwarmStrikePlanner(
            target=tgt, uavs=uavs,
            mode=AttackMode.DTOT, interval_sec=10.0,
            cruise_speed_mps=45, max_speed_mps=62, stall_speed_mps=28,
            turn_radius_m=150, base_cruise_alt_m=500, altitude_step_m=40,
            ip_distance_m=2000,
        )
        plans  = planner.plan()
        report = planner.collision_report
        files  = planner.export_qgc_wpl('./out/')

    Parameters
    ----------
    target :
        地面打擊目標 (1 個高價值目標)
    uavs :
        參與打擊的 UCAV 列表
    mode :
        STOT (同時命中) 或 DTOT (間隔命中)
    interval_sec :
        DTOT 模式的相鄰命中間隔 (秒)；STOT 模式忽略
    cruise_speed_mps :
        預設巡航空速
    max_speed_mps :
        飛控最大空速 (ARSPD_FBW_MAX)
    stall_speed_mps :
        失速速度 (ARSPD_FBW_MIN)；V_req 低於此值觸發 Loiter
    turn_radius_m :
        最小轉彎半徑 (用於 Dubins + Loiter)
    base_cruise_alt_m :
        最底層 UCAV 的巡航高度
    altitude_step_m :
        相鄰層 UCAV 的高度間距 (建議 ≥ 30 m)
    ip_distance_m :
        Initial Point 距目標的水平距離
    approach_offset_deg :
        攻擊方位角全域偏移 (預設 0°)
    min_alt_separation_m :
        避障檢查的最小允許高度差
    tot_margin_sec :
        TOT 安全邊界 (預設 0)
    """

    # ─────────────────────────────────────────────────────────────────
    #  初始化
    # ─────────────────────────────────────────────────────────────────
    def __init__(self,
                 target: Target,
                 uavs: List[UAV],
                 mode: AttackMode = AttackMode.STOT,
                 interval_sec: float = 0.0,
                 cruise_speed_mps: float = 45.0,
                 max_speed_mps: float    = 62.0,
                 stall_speed_mps: float  = 28.0,
                 turn_radius_m: float    = 150.0,
                 base_cruise_alt_m: float = 500.0,
                 altitude_step_m: float   = 30.0,
                 ip_distance_m: float     = 2000.0,
                 approach_offset_deg: float = 0.0,
                 min_alt_separation_m: float = 25.0,
                 tot_margin_sec: float    = 0.0):
        # ── 驗證 ───────────────────────────────────────────────
        if not uavs:
            raise ValueError('uavs 列表為空')
        if max_speed_mps <= stall_speed_mps:
            raise ValueError(
                f'max_speed({max_speed_mps}) 必須 > stall_speed({stall_speed_mps})'
            )
        if not (stall_speed_mps <= cruise_speed_mps <= max_speed_mps):
            raise ValueError(
                f'cruise_speed({cruise_speed_mps}) 必須在 '
                f'[{stall_speed_mps}, {max_speed_mps}] 內'
            )
        if mode == AttackMode.DTOT and interval_sec <= 0:
            raise ValueError(f'DTOT 模式必須提供 interval_sec > 0 (得 {interval_sec})')
        if altitude_step_m < min_alt_separation_m:
            logger.warning(
                f'[AdvSwarmStrike] altitude_step({altitude_step_m}) < '
                f'min_alt_separation({min_alt_separation_m}) 可能觸發避障警告'
            )

        self.target             = target
        self.uavs               = list(uavs)
        self.mode               = mode
        self.interval_sec       = float(interval_sec)
        self.cruise_speed_mps   = cruise_speed_mps
        self.max_speed_mps      = max_speed_mps
        self.stall_speed_mps    = stall_speed_mps
        self.turn_radius_m      = turn_radius_m
        self.base_cruise_alt_m  = base_cruise_alt_m
        self.altitude_step_m    = altitude_step_m
        self.ip_distance_m      = ip_distance_m
        self.approach_offset_deg = approach_offset_deg
        self.min_alt_separation_m = min_alt_separation_m
        self.tot_margin_sec     = tot_margin_sec

        # 快取
        self._plans: List[UAVPlan] = []
        self._base_tot: float      = 0.0
        self._collision_report: Optional[CollisionReport] = None

    # ═════════════════════════════════════════════════════════════════
    #  公開 API
    # ═════════════════════════════════════════════════════════════════

    def plan(self) -> List[UAVPlan]:
        """執行完整規劃 (空間 → 高度 → 時間 → 避障 → MAVLink)。

        Returns
        -------
        List[UAVPlan]
            與 `uavs` 相同長度；`slot_index` 指示 DTOT 命中順序
        """
        logger.info(
            f'[AdvSwarmStrike] 模式 {self.mode.value}'
            + (f' (Δ={self.interval_sec:.1f}s)' if self.mode == AttackMode.DTOT else '')
            + f'，目標={self.target.name}，{len(self.uavs)} 架 UCAV'
        )

        # ── Step 1: 360° 全向突防向量分配 ───────────────────
        assigned = self._assign_omnidirectional_slots()

        # ── Step 2: 建立基礎計畫 (Dubins 長度、名義時間) ─────
        plans: List[UAVPlan] = []
        for uav, heading in assigned:
            plans.append(self._build_base_plan(uav, heading))

        # ── Step 3: 時間協同 + slot 排序 ─────────────────────
        if self.mode == AttackMode.STOT:
            self._coordinate_stot(plans)
        else:
            plans = self._coordinate_dtot(plans)  # DTOT 會重排

        # ── Step 4: 高度錯層 (依最終 slot_index 分層) ────────
        self._assign_altitude_layers(plans)

        # ── Step 5: 避障驗證 ────────────────────────────────
        self._collision_report = self._verify_collision_avoidance(plans)

        # ── Step 6: MAVLink 任務序列 ────────────────────────
        for p in plans:
            p.mission = self._build_mission(p)

        self._plans = plans
        self._log_summary()
        return plans

    @property
    def base_tot_sec(self) -> float:
        """基準命中時間 (STOT: 全體命中秒; DTOT: slot 0 命中秒)"""
        return self._base_tot

    @property
    def plans(self) -> List[UAVPlan]:
        return list(self._plans)

    @property
    def collision_report(self) -> Optional[CollisionReport]:
        return self._collision_report

    def export_qgc_wpl(self, export_dir: str,
                       cruise_accept_radius_m: Optional[float] = None,
                       final_accept_radius_m: Optional[float]  = None,
                       ) -> List[str]:
        """匯出每架 UCAV 的 QGC WPL 110 航點檔 + 任務簡報"""
        if not self._plans:
            raise RuntimeError('請先呼叫 plan()')
        os.makedirs(export_dir, exist_ok=True)

        if cruise_accept_radius_m is None:
            cruise_accept_radius_m = self.turn_radius_m
        if final_accept_radius_m is None:
            final_accept_radius_m = max(10.0, self.turn_radius_m * 0.25)

        files: List[str] = []
        for p in self._plans:
            fname = (f'{self.mode.value}_slot{p.slot_index:02d}_{p.uav.name}'
                     f'_psi{p.attack_heading_deg:03.0f}'
                     f'_V{p.required_speed_mps:02.0f}mps'
                     f'_t{p.arrival_time_sec:04.0f}s.waypoints')
            fname = fname.replace('/', '-').replace('\\', '-')
            fpath = os.path.join(export_dir, fname)

            lines = ['QGC WPL 110']
            for seq, item in enumerate(p.mission):
                current = 1 if seq == 0 and item.cmd == MAVCmd.DO_SET_HOME else 0
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
    #  Step 1：360° 全向突防 slot 分配
    # ═════════════════════════════════════════════════════════════════

    def _assign_omnidirectional_slots(self) -> List[Tuple[UAV, float]]:
        """依 UAV 相對目標的方位角，分配至 360° 均勻攻擊向量。

        排序策略：
          - 每架 UAV 到目標的方位角 β_k
          - 攻擊向量 ψ_k = (360°/N)·k + φ_offset, k ∈ [0, N)
          - 升冪排序後成本最小 (以角度圓距離) 的旋轉對齊
        """
        n = len(self.uavs)
        if n == 1:
            u = self.uavs[0]
            brg = _bearing_deg(u.lat, u.lon, self.target.lat, self.target.lon)
            return [(u, brg)]

        # 依 UAV 對目標方位升冪排序
        by_bearing = sorted(
            self.uavs,
            key=lambda u: _bearing_deg(
                u.lat, u.lon, self.target.lat, self.target.lon
            ),
        )
        # 360° 均勻攻擊向量
        attack_vectors = sorted(
            (self.approach_offset_deg + 360.0 / n * k) % 360.0
            for k in range(n)
        )
        # 圓形旋轉對齊 (避免 0°/359° 邊界錯配)
        best_shift, best_cost = 0, float('inf')
        for shift in range(n):
            cost = sum(
                _angular_diff(
                    _bearing_deg(by_bearing[i].lat, by_bearing[i].lon,
                                 self.target.lat, self.target.lon),
                    attack_vectors[(i + shift) % n],
                )
                for i in range(n)
            )
            if cost < best_cost:
                best_cost, best_shift = cost, shift

        return [
            (by_bearing[i], attack_vectors[(i + best_shift) % n])
            for i in range(n)
        ]

    # ═════════════════════════════════════════════════════════════════
    #  Step 2：基礎計畫 (Dubins 長度 + 名義時間)
    # ═════════════════════════════════════════════════════════════════

    def _build_base_plan(self, uav: UAV,
                         attack_heading_deg: float) -> UAVPlan:
        """計算 UAV 的 IP、Dubins 長度、名義時間"""
        # IP 位於目標的 (攻擊方位 + 180°) 方向，距離 ip_distance
        ip_lat, ip_lon = _destination(
            self.target.lat, self.target.lon,
            (attack_heading_deg + 180.0) % 360.0,
            self.ip_distance_m,
        )
        # 起始航向取朝向 IP 以最小化初始轉彎
        start_heading = _bearing_deg(uav.lat, uav.lon, ip_lat, ip_lon)

        # Dubins 水平長度: UAV → IP
        dubins_len = dubins_shortest_length(
            uav.lat, uav.lon, start_heading,
            ip_lat, ip_lon, attack_heading_deg,
            self.turn_radius_m,
        )
        # 末端直線段: IP → target
        final_leg = _haversine(ip_lat, ip_lon,
                               self.target.lat, self.target.lon)
        horiz_total = dubins_len + final_leg

        # 3D 長度 (含爬升斜線)
        climb = max(self.base_cruise_alt_m - uav.alt, 0.0)
        total_3d = math.hypot(horiz_total, climb)

        nominal_time = total_3d / max(self.cruise_speed_mps, 0.1)

        return UAVPlan(
            uav=uav,
            slot_index=-1,                       # 尚未分配
            attack_heading_deg=attack_heading_deg,
            ip_lat=ip_lat, ip_lon=ip_lon,
            path_length_m=total_3d,
            cruise_alt_m=self.base_cruise_alt_m,  # 尚未分層
            loiter_alt_m=self.base_cruise_alt_m,
            arrival_time_sec=0.0,                 # Step 3 填入
            nominal_time_sec=nominal_time,
            required_speed_mps=self.cruise_speed_mps,
        )

    # ═════════════════════════════════════════════════════════════════
    #  Step 3a：STOT 時間協同 (同秒命中)
    # ═════════════════════════════════════════════════════════════════

    def _coordinate_stot(self, plans: List[UAVPlan]) -> None:
        """STOT 模式：所有 UCAV 在同一秒命中目標。

        2026 重構：TOT 排程委派給 core.strike.time_coordination。
        """
        from core.strike.time_coordination import compute_tot_schedule
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[p.nominal_time_sec for p in plans],
            mode='STOT',
            tot_margin_sec=self.tot_margin_sec,
            payloads=plans,
        )
        self._base_tot = base_tot
        logger.info(f'[STOT] Base_TOT = {base_tot:.2f} s (由最長路徑機決定)')

        for slot in slots:
            p = slot.payload
            p.slot_index = slot.slot_index
            p.arrival_time_sec = slot.arrival_time_sec
            self._solve_speed_or_loiter(p, p.arrival_time_sec)

    # ═════════════════════════════════════════════════════════════════
    #  Step 3b：DTOT 時間協同 (間隔命中)
    # ═════════════════════════════════════════════════════════════════

    def _coordinate_dtot(self, plans: List[UAVPlan]) -> List[UAVPlan]:
        """DTOT 模式：T, T+Δ, T+2Δ, ... 依序命中。

        排序策略：最遠 L 入 slot 0 (不補時)，其餘依序延遲。
        2026 重構：TOT 排程委派給 core.strike.time_coordination。
        """
        from core.strike.time_coordination import compute_tot_schedule
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[p.nominal_time_sec for p in plans],
            mode='DTOT',
            interval_sec=self.interval_sec,
            dtot_order='longest_first',
            tot_margin_sec=self.tot_margin_sec,
            payloads=plans,
        )
        self._base_tot = base_tot

        logger.info(
            f'[DTOT] Base_TOT = {base_tot:.2f} s, 間隔 Δ = {self.interval_sec:.1f} s → '
            f'slot 命中時刻：'
            + ', '.join(f'{base_tot + k * self.interval_sec:.1f}'
                        for k in range(len(plans)))
        )

        for slot in slots:
            p = slot.payload
            p.slot_index = slot.slot_index
            p.arrival_time_sec = slot.arrival_time_sec
            self._solve_speed_or_loiter(p, p.arrival_time_sec)

        return sorted(plans, key=lambda p: p.slot_index)

    # ═════════════════════════════════════════════════════════════════
    #  Step 3c：速度/盤旋決策 (共用邏輯)
    # ═════════════════════════════════════════════════════════════════

    def _solve_speed_or_loiter(self, p: UAVPlan, t_arrive: float) -> None:
        """給定目標到達時間 t_arrive，決定 UCAV 的空速與盤旋補時。

        決策優先序 (依戰術規範「V < V_stall 強制使用 Loiter」)：
          1. 若 V_cruise 恰好可在 t_arrive 命中 (誤差 1%)  → OK，V = V_cruise
          2. 若 V_req > V_max                               → INFEASIBLE
          3. 若 V_req 在 [V_stall, V_max]                  → 調速 (SPEED_ADJUSTED)
          4. 若 V_req < V_stall                             → 夾 V_stall + Loiter
        """
        L = p.path_length_m
        if t_arrive <= 0.1:
            p.feasibility = FeasibilityStatus.INFEASIBLE
            p.warning = f't_arrive={t_arrive:.2f} 無效'
            return

        v_req_ideal = L / t_arrive

        # ── 案例 2: V_req > V_max → 無解 ─────────────────────
        if v_req_ideal > self.max_speed_mps + 1e-3:
            p.required_speed_mps = self.max_speed_mps
            p.feasibility = FeasibilityStatus.INFEASIBLE
            p.warning = (f'V_req={v_req_ideal:.2f} > V_max={self.max_speed_mps:.1f} '
                         f'(路徑太長/到達時間太早)')
            return

        # ── 案例 4: V_req < V_stall → 強制 Loiter ────────────
        if v_req_ideal < self.stall_speed_mps - 1e-3:
            p.required_speed_mps = self.stall_speed_mps
            p.feasibility = FeasibilityStatus.LOITER_INSERTED

            t_fly_at_stall = L / self.stall_speed_mps
            p.loiter_time_sec = max(0.0, t_arrive - t_fly_at_stall)
            t_lap = 2.0 * math.pi * self.turn_radius_m / self.stall_speed_mps
            p.loiter_turns = p.loiter_time_sec / max(t_lap, 0.1)
            # 盤旋點：起飛點正上方 (爬升完就地等待)
            p.loiter_lat = p.uav.lat
            p.loiter_lon = p.uav.lon
            p.loiter_radius_m = self.turn_radius_m
            p.warning = (f'V_req={v_req_ideal:.2f} < V_stall → Loiter '
                         f'{p.loiter_time_sec:.1f}s ({p.loiter_turns:.1f} 圈)')
            return

        # ── DTOT 模式特例：路徑較短者以 V_cruise 飛 + Loiter ─
        # (保持巡航速度，僅用盤旋消耗時間差；更符合真實戰術)
        if self.mode == AttackMode.DTOT:
            t_at_cruise = L / self.cruise_speed_mps
            if t_at_cruise < t_arrive - 0.5:
                # 以 V_cruise 飛會早到 → 插入 Loiter 等待
                p.required_speed_mps = self.cruise_speed_mps
                p.feasibility = FeasibilityStatus.LOITER_INSERTED
                p.loiter_time_sec = t_arrive - t_at_cruise
                t_lap = 2.0 * math.pi * self.turn_radius_m / self.cruise_speed_mps
                p.loiter_turns = p.loiter_time_sec / max(t_lap, 0.1)
                p.loiter_lat = p.uav.lat
                p.loiter_lon = p.uav.lon
                p.loiter_radius_m = self.turn_radius_m
                p.warning = (f'DTOT 延遲：V=V_cruise + Loiter {p.loiter_time_sec:.1f}s '
                             f'({p.loiter_turns:.1f} 圈)')
                return

        # ── 案例 3: V_req ∈ [V_stall, V_max] → 調速 ──────────
        # (STOT 模式或 DTOT 路徑最長者)
        p.required_speed_mps = v_req_ideal
        if abs(v_req_ideal - self.cruise_speed_mps) < 0.5:
            p.feasibility = FeasibilityStatus.OK
        else:
            p.feasibility = FeasibilityStatus.SPEED_ADJUSTED

    # ═════════════════════════════════════════════════════════════════
    #  Step 4：高度錯層
    # ═════════════════════════════════════════════════════════════════

    def _assign_altitude_layers(self, plans: List[UAVPlan]) -> None:
        """依 slot_index 分配巡航/盤旋高度層 (STOT: 編號 / DTOT: 命中順序)"""
        for p in plans:
            layer_alt = self.base_cruise_alt_m + p.slot_index * self.altitude_step_m
            p.cruise_alt_m = layer_alt
            p.loiter_alt_m = layer_alt

    # ═════════════════════════════════════════════════════════════════
    #  Step 5：避障驗證 (3D 防撞)
    # ═════════════════════════════════════════════════════════════════

    def _verify_collision_avoidance(self,
                                     plans: List[UAVPlan]) -> CollisionReport:
        """三層避障檢查：高度錯層 + 盤旋隔離 + 俯衝穿透。"""
        alt_viol: List[str] = []
        loi_viol: List[str] = []
        dive_warn: List[str] = []

        min_alt_sep = float('inf')
        min_loi_dist = float('inf')

        # ── 兩兩巡航/盤旋高度差 ──────────────────────────────
        for i, p1 in enumerate(plans):
            for p2 in plans[i + 1:]:
                sep = abs(p1.cruise_alt_m - p2.cruise_alt_m)
                if sep < min_alt_sep:
                    min_alt_sep = sep
                if sep < self.min_alt_separation_m:
                    alt_viol.append(
                        f'{p1.uav.name} (alt={p1.cruise_alt_m:.0f}) vs '
                        f'{p2.uav.name} (alt={p2.cruise_alt_m:.0f}): '
                        f'高度差僅 {sep:.0f}m (< {self.min_alt_separation_m:.0f})'
                    )

        # ── 盤旋中心兩兩 2D 距離 (僅檢查需 Loiter 的機體) ────
        loiter_plans = [p for p in plans if p.loiter_time_sec > 0.5]
        for i, p1 in enumerate(loiter_plans):
            for p2 in loiter_plans[i + 1:]:
                d = _haversine(p1.loiter_lat, p1.loiter_lon,
                               p2.loiter_lat, p2.loiter_lon)
                if d < min_loi_dist:
                    min_loi_dist = d
                # 同高度層 + 盤旋圈重疊 (d < 2R) → 衝突
                alt_sep = abs(p1.cruise_alt_m - p2.cruise_alt_m)
                if (d < 2.0 * self.turn_radius_m
                        and alt_sep < self.min_alt_separation_m):
                    loi_viol.append(
                        f'{p1.uav.name} 與 {p2.uav.name} 盤旋圈重疊 '
                        f'(d={d:.0f}m < 2R={2 * self.turn_radius_m:.0f}m) '
                        f'且高度差 {alt_sep:.0f}m 不足'
                    )

        # ── 俯衝軌跡穿透檢查 ─────────────────────────────────
        # 高層機 H 俯衝時穿過低層機 L 的高度，檢查 2D 位置是否接近
        # 時間條件：僅當 H 俯衝時 L 仍在空域內才算衝突
        for p_hi in plans:
            for p_lo in plans:
                if p_hi.uav.uav_id == p_lo.uav.uav_id:
                    continue
                # 只檢查往下俯衝穿過低層的情況
                if p_hi.cruise_alt_m <= p_lo.cruise_alt_m:
                    continue

                # 俯衝幾何：從 IP_hi(alt=cruise_hi) 直線到 target(alt=0)
                # 穿過高度 alt_lo 的參數 t ∈ [0, 1]
                t_cross = 1.0 - (p_lo.cruise_alt_m / max(p_hi.cruise_alt_m, 0.1))
                t_cross = max(0.0, min(1.0, t_cross))
                cross_lat = p_hi.ip_lat + t_cross * (self.target.lat - p_hi.ip_lat)
                cross_lon = p_hi.ip_lon + t_cross * (self.target.lon - p_hi.ip_lon)

                # 低層機在該時刻的 2D 位置 (簡化為盤旋中心或 IP)
                if p_lo.loiter_time_sec > 0.5:
                    lo_lat, lo_lon = p_lo.loiter_lat, p_lo.loiter_lon
                else:
                    lo_lat, lo_lon = p_lo.ip_lat, p_lo.ip_lon

                d_2d = _haversine(cross_lat, cross_lon, lo_lat, lo_lon)

                # 時間條件檢查：DTOT 模式下低層機若已先命中，不算衝突
                time_overlap = True
                if self.mode == AttackMode.DTOT:
                    # p_hi 俯衝開始時刻 ≈ arrival_hi - dive_time
                    dive_time = self.ip_distance_m / max(p_hi.required_speed_mps, 1.0)
                    t_hi_dive_start = p_hi.arrival_time_sec - dive_time
                    # 若 p_lo 已在 t_hi_dive_start 前命中 (arrival_lo < t_hi_dive_start)
                    # → 空域已淨空，無衝突
                    if p_lo.arrival_time_sec < t_hi_dive_start - 0.5:
                        time_overlap = False

                if time_overlap and d_2d < self.turn_radius_m:
                    dive_warn.append(
                        f'{p_hi.uav.name} 俯衝軌跡在 alt={p_lo.cruise_alt_m:.0f}m '
                        f'穿過 {p_lo.uav.name} 位置 (2D 距離 {d_2d:.0f}m '
                        f'< R_min={self.turn_radius_m:.0f}m)'
                    )

        safe = not (alt_viol or loi_viol or dive_warn)
        return CollisionReport(
            safe=safe,
            min_cruise_alt_separation_m=min_alt_sep if min_alt_sep != float('inf') else -1,
            min_loiter_distance_m=min_loi_dist if min_loi_dist != float('inf') else -1,
            dive_penetration_warnings=dive_warn,
            altitude_violations=alt_viol,
            loiter_violations=loi_viol,
        )

    # ═════════════════════════════════════════════════════════════════
    #  Step 6：MAVLink 任務序列
    # ═════════════════════════════════════════════════════════════════

    def _build_mission(self, p: UAVPlan) -> List[MissionItem]:
        """生成單架 UCAV 的 MAVLink 任務序列 (QGC WPL 110 相容)。

        序列結構::

            seq 0  DO_SET_HOME       (起飛點錨定)
            seq 1  DO_CHANGE_SPEED   (V_req)
            seq 2  NAV_TAKEOFF       (爬升至 cruise_alt)
            seq 3  [NAV_LOITER_TIME] (若需要補時)
            seq 4  NAV_WAYPOINT      (IP — 俯衝起始)
            seq 5  NAV_WAYPOINT      (IMPACT — 目標命中點)
        """
        seq: List[MissionItem] = []

        # 0. DO_SET_HOME
        seq.append(MissionItem(
            cmd=MAVCmd.DO_SET_HOME,
            lat=p.uav.lat, lon=p.uav.lon, alt=p.uav.alt,
            comment=f'Home (slot {p.slot_index})',
        ))
        # 1. DO_CHANGE_SPEED (p1=0 空速, p2=V_req, p3=-1 不改油門)
        seq.append(MissionItem(
            cmd=MAVCmd.DO_CHANGE_SPEED,
            param1=0.0, param2=float(p.required_speed_mps), param3=-1.0,
            comment=f'V={p.required_speed_mps:.2f} m/s',
        ))
        # 2. NAV_TAKEOFF (p1=爬升俯仰 rad=8°; lat/lon=朝 IP 方向 100m 位置)
        brg_to_ip = _bearing_deg(p.uav.lat, p.uav.lon, p.ip_lat, p.ip_lon)
        tk_lat, tk_lon = _destination(p.uav.lat, p.uav.lon, brg_to_ip, 100.0)
        seq.append(MissionItem(
            cmd=MAVCmd.NAV_TAKEOFF,
            lat=tk_lat, lon=tk_lon, alt=p.cruise_alt_m,
            param1=math.radians(8.0),
            comment=f'Takeoff → cruise alt {p.cruise_alt_m:.0f}m',
        ))
        # 3. NAV_LOITER_TIME (若有)
        if p.feasibility == FeasibilityStatus.LOITER_INSERTED and p.loiter_time_sec > 0.5:
            seq.append(MissionItem(
                cmd=MAVCmd.NAV_LOITER_TIME,
                lat=p.loiter_lat, lon=p.loiter_lon, alt=p.loiter_alt_m,
                param1=float(p.loiter_time_sec),
                param2=0.0,
                param3=float(p.loiter_radius_m),
                param4=1.0,            # 達高度方結束
                comment=f'Loiter {p.loiter_time_sec:.1f}s '
                        f'({p.loiter_turns:.1f} turns)',
            ))
        # 4. NAV_WAYPOINT (IP)
        seq.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=p.ip_lat, lon=p.ip_lon, alt=p.cruise_alt_m,
            param2=float(self.turn_radius_m),
            comment=f'IP (ψ={p.attack_heading_deg:.1f}°)',
        ))
        # 5. NAV_WAYPOINT (IMPACT)
        seq.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=self.target.lat, lon=self.target.lon, alt=self.target.alt,
            param2=max(10.0, self.turn_radius_m * 0.25),
            comment=f'IMPACT {self.target.name} @ t={p.arrival_time_sec:.1f}s',
        ))
        return seq

    # ═════════════════════════════════════════════════════════════════
    #  日誌與簡報
    # ═════════════════════════════════════════════════════════════════

    def _log_summary(self) -> None:
        logger.info('[AdvSwarmStrike] 規劃摘要:')
        for p in self._plans:
            loi = f' +Loiter {p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.5 else ''
            logger.info(
                f'  slot {p.slot_index} [{p.feasibility.value:<16s}] '
                f'{p.uav.name}  psi={p.attack_heading_deg:5.1f}deg  '
                f'L={p.path_length_m:7.0f}m  V={p.required_speed_mps:5.2f}m/s  '
                f'alt={p.cruise_alt_m:.0f}m  t_hit={p.arrival_time_sec:.1f}s{loi}'
                + (f'   ! {p.warning}' if p.warning else '')
            )
        if self._collision_report:
            rep = self._collision_report
            logger.info(
                f'[避障] {"SAFE" if rep.safe else "UNSAFE"}  '
                f'min_alt_sep={rep.min_cruise_alt_separation_m:.0f}m  '
                f'min_loi_dist={rep.min_loiter_distance_m:.0f}m  '
                f'conflicts={len(rep.all_conflicts)}'
            )
            for c in rep.all_conflicts:
                logger.warning(f'  ! {c}')

    def _write_briefing(self, export_dir: str) -> None:
        """輸出指揮官任務簡報"""
        path = os.path.join(export_dir, f'{self.mode.value}_briefing.txt')
        rep = self._collision_report
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'{self.mode.value} 蜂群飽和打擊任務簡報\n')
            f.write('=' * 70 + '\n\n')
            f.write(f'戰術模式：{self.mode.value}'
                    + (f' (Δ={self.interval_sec:.1f} s)' if self.mode == AttackMode.DTOT else '')
                    + '\n')
            f.write(f'目標    ：{self.target.name} '
                    f'({self.target.lat:.6f}, {self.target.lon:.6f})\n')
            f.write(f'UCAV 數量：{len(self._plans)}\n')
            f.write(f'基準 TOT ：{self._base_tot:.2f} s  '
                    f'(slot 0 命中時刻)\n')
            f.write(f'速度區間：[{self.stall_speed_mps:.0f}, {self.max_speed_mps:.0f}] m/s '
                    f'(cruise={self.cruise_speed_mps:.0f})\n')
            f.write(f'R_min   ：{self.turn_radius_m:.0f} m\n')
            f.write(f'高度錯層：base={self.base_cruise_alt_m:.0f}m, '
                    f'step={self.altitude_step_m:.0f}m\n\n')

            f.write(f'{"slot":>4s} {"UCAV":<10s} {"psi(deg)":>8s} '
                    f'{"L(m)":>7s} {"alt(m)":>6s} {"V(m/s)":>7s} '
                    f'{"t_hit(s)":>9s} {"Loiter":>9s} {"feasibility":<18s}\n')
            f.write('-' * 100 + '\n')
            for p in self._plans:
                loi = f'{p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.1 else '-'
                f.write(
                    f'{p.slot_index:>4d} {p.uav.name:<10s} '
                    f'{p.attack_heading_deg:>8.1f} '
                    f'{p.path_length_m:>7.0f} '
                    f'{p.cruise_alt_m:>6.0f} '
                    f'{p.required_speed_mps:>7.2f} '
                    f'{p.arrival_time_sec:>9.1f} '
                    f'{loi:>9s} '
                    f'{p.feasibility.value:<18s}\n'
                )
            f.write('\n[避障報告]\n')
            if rep:
                f.write(f'  結果: {"SAFE" if rep.safe else "UNSAFE"}\n')
                f.write(f'  最小巡航高度差: {rep.min_cruise_alt_separation_m:.0f} m\n')
                f.write(f'  最小盤旋距離: {rep.min_loiter_distance_m:.0f} m\n')
                for c in rep.all_conflicts:
                    f.write(f'  ! {c}\n')


# ═══════════════════════════════════════════════════════════════════════
#  Main Demo — 3 架 UCAV 對單一目標執行 DTOT (間隔 10 秒)
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    # ── 目標：台北 101 地面座標 ─────────────────────────────
    target = Target(lat=25.0330, lon=121.5654, alt=0.0, name='TGT-Taipei101')

    # ── 3 架 Shahed-136 類 OWA-UAV 從不同方向發射 ────────────
    uavs = [
        UAV(uav_id=1, lat=25.25, lon=122.05, alt=0.0, name='UCAV-NE'),  # 東北 (最遠)
        UAV(uav_id=2, lat=25.12, lon=121.28, alt=0.0, name='UCAV-NW'),  # 西北 (中等)
        UAV(uav_id=3, lat=24.85, lon=121.60, alt=0.0, name='UCAV-S'),   # 南方 (最近)
    ]

    # ── 建立高階規劃器 (DTOT 間隔打擊，Δ = 10 秒) ───────────
    planner = AdvancedSwarmStrikePlanner(
        target=target,
        uavs=uavs,
        mode=AttackMode.DTOT,
        interval_sec=10.0,           # ← DTOT 間隔
        cruise_speed_mps=45.0,       # Shahed-136 典型
        max_speed_mps=62.0,
        stall_speed_mps=28.0,
        turn_radius_m=150.0,
        base_cruise_alt_m=500.0,     # 最底層 UCAV 巡航高度
        altitude_step_m=40.0,        # 每層差 40 m
        ip_distance_m=2000.0,
        approach_offset_deg=0.0,
        min_alt_separation_m=25.0,
    )

    plans = planner.plan()
    report = planner.collision_report

    # ── 輸出規劃結果 ───────────────────────────────────────
    print('\n' + '═' * 78)
    print(f' DTOT 間隔打擊規劃結果   Base_TOT = {planner.base_tot_sec:.2f} s, '
          f'interval = 10.0 s')
    print('═' * 78)
    print(f'{"slot":>4s} {"UCAV":<10s} {"psi(deg)":>8s} {"L(m)":>7s} '
          f'{"alt(m)":>6s} {"V(m/s)":>7s} {"t_hit(s)":>9s} {"Loiter":>10s} '
          f'{"狀態":<18s}')
    print('-' * 78)
    for p in plans:
        loi = f'{p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.1 else '-'
        print(f'{p.slot_index:>4d} {p.uav.name:<10s} '
              f'{p.attack_heading_deg:>8.1f} '
              f'{p.path_length_m:>7.0f} '
              f'{p.cruise_alt_m:>6.0f} '
              f'{p.required_speed_mps:>7.2f} '
              f'{p.arrival_time_sec:>9.1f} '
              f'{loi:>10s} '
              f'{p.feasibility.value:<18s}')

    # ── 避障報告 ───────────────────────────────────────────
    print('\n' + '═' * 78)
    print(' 避障檢查')
    print('═' * 78)
    print(f'  總體狀態     : {"SAFE" if report.safe else "UNSAFE"}')
    print(f'  最小高度差   : {report.min_cruise_alt_separation_m:.0f} m '
          f'(要求 >= {planner.min_alt_separation_m:.0f})')
    if report.min_loiter_distance_m >= 0:
        print(f'  最小盤旋距離 : {report.min_loiter_distance_m:.0f} m')
    if report.all_conflicts:
        print('  衝突項:')
        for c in report.all_conflicts:
            print(f'    ! {c}')
    else:
        print('  無衝突 (altitude layer + 360° 攻擊角分離已足夠)')

    # ── MAVLink 任務序列範例 (最先命中的 slot 0) ────────────
    print('\n' + '═' * 78)
    print(' MAVLink 任務序列 — slot 0 (最先命中)')
    print('═' * 78)
    p0 = plans[0]
    for i, item in enumerate(p0.mission):
        print(f'  seq {i}  cmd={item.cmd:3d}  '
              f'lat={item.lat:10.6f} lon={item.lon:11.6f} alt={item.alt:6.1f}  '
              f'p1={item.param1:7.2f} p2={item.param2:7.2f} '
              f'p3={item.param3:6.1f} p4={item.param4:5.2f}  '
              f'| {item.comment}')

    # ── 匯出 QGC WPL 110 ──────────────────────────────────
    out = os.path.join(os.getcwd(), 'dtot_interval10_out')
    files = planner.export_qgc_wpl(out)
    print(f'\n[匯出] {len(files)} 份 .waypoints → {out}')
    for f in files:
        print(f'  • {os.path.basename(f)}')
