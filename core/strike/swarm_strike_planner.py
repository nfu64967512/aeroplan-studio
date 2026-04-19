"""
SwarmStrikePlanner — STOT/DTOT 蜂群協同路徑規劃器
==================================================

適用機型：類 Shahed-136 / Geran-2 長程單向攻擊無人機 (OWA-UAV)
         Delta-wing 固定翼，純 GNSS+INS 導航，不可返航。

核心戰術
--------
* **STOT (Simultaneous launch / Time On Target)**：同地發射，同時命中
* **DTOT (Different launch  / Time On Target)**：異地發射，同時命中
    - 不論起飛地點、不論發射時刻差，全部 UCAV 在同一秒抵達目標
    - 360° 均勻分佈的攻擊進場角度，癱瘓敵方防空反應迴路

本模組實作四大核心邏輯：
    1. 全向突防向量分配 (N 架 UAV × 360° / N 攻擊角)
    2. Dubins 曲線平滑軌跡生成 (LSL/RSR/LSR/RSL 四型閉式解)
    3. 時間協同 TOT 演算 (動態速度反推)
    4. 物理極限保護 (V < V_stall → 自動插入 Loiter 補時)

MAVLink 指令對照
----------------
    MAV_CMD_DO_SET_HOME         (179)
    MAV_CMD_DO_CHANGE_SPEED     (178)  ← 核心：TOT 協同的速度指令
    MAV_CMD_NAV_TAKEOFF          (22)
    MAV_CMD_NAV_LOITER_TIME      (19)  ← 核心：V_req < V_stall 時的補時盤旋
    MAV_CMD_NAV_WAYPOINT         (16)
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from utils.file_io import create_waypoint_line, write_waypoints
from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  MAVLink 命令常數
# ═══════════════════════════════════════════════════════════════════════
class MAVCmd:
    """MAVLink 任務指令 ID (與 pymavlink.mavutil.mavlink.MAV_CMD_* 一致)"""
    NAV_WAYPOINT        = 16
    NAV_LOITER_UNLIM    = 17
    NAV_LOITER_TURNS    = 18
    NAV_LOITER_TIME     = 19
    NAV_TAKEOFF         = 22
    DO_CHANGE_SPEED     = 178
    DO_SET_HOME         = 179


# ── 共用幾何常數/函式，統一從 core.strike.geometry 匯入 ────────
# (2026 重構：原本本檔內定義，為消除重複已遷移至 geometry 模組)
from core.strike.geometry import (
    R_EARTH as _R_EARTH,
    MAV_FRAME_REL as _MAV_FRAME_REL,
    haversine as _haversine,
    bearing_deg as _bearing_deg,
    destination as _destination,
    angular_diff as _angular_diff,
    dubins_shortest_length,
)


# ═══════════════════════════════════════════════════════════════════════
#  資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class UAV:
    """參與蜂群打擊的無人機

    Attributes
    ----------
    uav_id :
        機體識別碼
    lat / lon / alt :
        起飛點座標 (WGS-84)
    name :
        顯示用名稱；若留空則自動使用 'UCAV-{id}'
    """
    uav_id: int
    lat: float
    lon: float
    alt: float = 0.0
    name: str = ''

    def __post_init__(self) -> None:
        if not self.name:
            self.name = f'UCAV-{self.uav_id}'


@dataclass
class Target:
    """地面打擊目標"""
    lat: float
    lon: float
    alt: float = 0.0
    name: str = 'TGT'


@dataclass
class MissionItem:
    """MAVLink 任務項 (對應 QGC WPL 110 單行)"""
    cmd: int
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    comment: str = ''


class FeasibilityStatus(str, Enum):
    """單架 UCAV 的 TOT 協同可行性狀態"""
    OK              = 'OK'              # 在速度區間內，直接調速
    LOITER_INSERTED = 'LOITER_INSERTED' # 飛太慢 → 夾 V_stall + 插盤旋
    INFEASIBLE      = 'INFEASIBLE'      # 飛太快 (V_req > V_max) → 不可行


@dataclass
class StrikePlan:
    """單架 UCAV 的完整攻擊規劃 (空間 + 時間 + MAVLink 序列)"""
    uav: UAV

    # ── 空間規劃 ──────────────────────────────────────────────
    attack_heading_deg: float      # 末端攻擊方位角 ψ_i (0~360°)
    ip_lat: float                   # Initial Point 緯度
    ip_lon: float                   # Initial Point 經度

    # ── 軌跡長度與時間 ────────────────────────────────────────
    path_length_m: float            # Dubins 3D 路徑總長度
    nominal_flight_time_sec: float  # = L_i / V_cruise
    required_speed_mps: float       # 反推的最佳巡航空速

    # ── 盤旋補時 (僅 V_req < V_stall 時啟用) ─────────────────
    loiter_time_sec: float = 0.0    # 需要盤旋的時間 (秒)
    loiter_turns: float    = 0.0    # 對應圈數 = time / T_lap
    loiter_lat: float      = 0.0    # 盤旋圓心緯度
    loiter_lon: float      = 0.0    # 盤旋圓心經度
    loiter_radius_m: float = 0.0    # 盤旋半徑

    # ── 狀態旗標 ─────────────────────────────────────────────
    feasibility: FeasibilityStatus = FeasibilityStatus.OK
    warning: str = ''

    # ── MAVLink 任務序列 (QGC WPL 110 格式) ──────────────────
    mission: List[MissionItem] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════════════
#  地理工具 / Dubins / Angular diff
#  ──────────────────────────────────────────────────────────────────────
#  已統一遷移至 core.strike.geometry 模組 (2026 重構，消除 6 個 planner
#  的 2500 行重複程式碼)。本檔保留 `_haversine` / `_bearing_deg` 等
#  私有符號 re-export 僅為向下相容，新程式碼應直接從 `geometry` 匯入。
# ═══════════════════════════════════════════════════════════════════════


# ═══════════════════════════════════════════════════════════════════════
#  核心類別：SwarmStrikePlanner
# ═══════════════════════════════════════════════════════════════════════

class SwarmStrikePlanner:
    """STOT/DTOT 蜂群協同攻擊規劃器

    一次完整的規劃流程由 :meth:`plan` 驅動，回傳每架 UCAV 的 :class:`StrikePlan`
    （含 MAVLink 任務序列）。接著可呼叫 :meth:`export_qgc_wpl` 寫成 `.waypoints`。

    Parameters
    ----------
    target :
        地面打擊目標 (1 個)
    uavs :
        參與打擊的無人機列表 (N 架)
    cruise_speed_mps :
        預設巡航空速 (m/s)；耗時最長機體將以此速度飛行
    max_speed_mps :
        飛控允許最大空速 (ARSPD_FBW_MAX 等價)
    stall_speed_mps :
        失速速度下限 (ARSPD_FBW_MIN 等價)；V_req 低於此值將觸發 Loiter
    turn_radius_m :
        Dubins 最小轉彎半徑 R_min (m)
    cruise_alt_m :
        巡航高度 (m, AGL)；實際各機高度再疊加 altitude_step_m 錯層
    altitude_step_m :
        多機高度錯層間距 (m)；防止空中相撞
    ip_distance_m :
        Initial Point (IP) 距目標的水平距離 (m)；IP 是俯衝前的最後一個巡航點
    approach_offset_deg :
        攻擊方位角全域偏移 (度)；預設 0 = 正北方為第一個 slot
    tot_margin_sec :
        TOT 額外安全邊界 (秒)；預設 0 = 嚴格同秒
    """

    def __init__(self,
                 target: Target,
                 uavs: List[UAV],
                 cruise_speed_mps: float  = 45.0,
                 max_speed_mps: float     = 60.0,
                 stall_speed_mps: float   = 28.0,
                 turn_radius_m: float     = 150.0,
                 cruise_alt_m: float      = 500.0,
                 altitude_step_m: float   = 30.0,
                 ip_distance_m: float     = 1500.0,
                 approach_offset_deg: float = 0.0,
                 tot_margin_sec: float    = 0.0):
        # ── 輸入驗證 ─────────────────────────────────────────
        if not uavs:
            raise ValueError('uavs 列表為空')
        if max_speed_mps <= stall_speed_mps:
            raise ValueError(
                f'max_speed({max_speed_mps}) 必須 > stall_speed({stall_speed_mps})'
            )
        if not (stall_speed_mps <= cruise_speed_mps <= max_speed_mps):
            raise ValueError(
                f'cruise_speed 必須在 [stall, max] 範圍內，'
                f'得 {cruise_speed_mps} (限 {stall_speed_mps}~{max_speed_mps})'
            )

        self.target             = target
        self.uavs               = list(uavs)
        self.cruise_speed_mps   = cruise_speed_mps
        self.max_speed_mps      = max_speed_mps
        self.stall_speed_mps    = stall_speed_mps
        self.turn_radius_m      = turn_radius_m
        self.cruise_alt_m       = cruise_alt_m
        self.altitude_step_m    = altitude_step_m
        self.ip_distance_m      = ip_distance_m
        self.approach_offset_deg = approach_offset_deg
        self.tot_margin_sec     = tot_margin_sec

        # 執行結果的快取 (呼叫 plan() 後填入)
        self._plans: List[StrikePlan] = []
        self._tot_sec: float = 0.0

    # ─────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────
    def plan(self) -> List[StrikePlan]:
        """執行完整 STOT/DTOT 規劃流程。

        流程：
          1. 分配 360° 均勻攻擊向量 (UAV ↔ ψ_i)
          2. 為每機體生成 Dubins 軌跡 + 長度
          3. 計算基準 TOT、反推每機所需空速
          4. 夾限至 [V_stall, V_max]；過慢者插入 Loiter 補時
          5. 產生 MAVLink 任務序列 (內部快取)

        Returns
        -------
        List[StrikePlan]
            與 `uavs` 同長度；`uav_id` 可用於與 SITL link 配對
        """
        # Step 1: 空間規劃
        assignments = self._assign_attack_vectors()

        # Step 2: 每機 Dubins 長度 + 基礎 StrikePlan
        plans: List[StrikePlan] = []
        for slot, (uav, heading) in enumerate(assignments):
            p = self._build_base_plan(uav, heading, slot)
            plans.append(p)
            logger.debug(
                f'[SwarmStrike] {uav.name} → ψ={heading:6.1f}°, '
                f'L={p.path_length_m:.0f}m, T_nom={p.nominal_flight_time_sec:.1f}s'
            )

        # Step 3 + 4: TOT 協同 + 速度/盤旋補償
        self._tot_sec = self._coordinate_tot(plans)

        # Step 5: MAVLink 序列
        for p in plans:
            p.mission = self._build_mission_sequence(p)

        self._plans = plans
        self._log_summary(plans)
        return plans

    def export_qgc_wpl(self, export_dir: str,
                       cruise_accept_radius_m: Optional[float] = None,
                       final_accept_radius_m: Optional[float]  = None,
                       mode_label: str = 'SWARM') -> List[str]:
        """將每架 UCAV 的 mission 寫成 QGC WPL 110 航點檔。

        Returns
        -------
        List[str]
            成功寫出的檔案絕對路徑列表
        """
        if not self._plans:
            raise RuntimeError('必須先呼叫 plan() 再進行匯出')

        os.makedirs(export_dir, exist_ok=True)
        if cruise_accept_radius_m is None:
            cruise_accept_radius_m = self.turn_radius_m
        if final_accept_radius_m is None:
            final_accept_radius_m = max(10.0, self.turn_radius_m * 0.25)

        files: List[str] = []
        for p in self._plans:
            fname = (f'{mode_label}_{p.uav.name}_psi{p.attack_heading_deg:03.0f}'
                     f'_V{p.required_speed_mps:02.0f}mps'
                     f'_TOT{self._tot_sec:04.0f}s.waypoints')
            fname = fname.replace('/', '-').replace('\\', '-')
            fpath = os.path.join(export_dir, fname)

            lines = ['QGC WPL 110']
            for seq, item in enumerate(p.mission):
                # 前面 DO_SET_HOME 的 current=1，其餘 current=0
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
                logger.info(
                    f'[SwarmStrike] 匯出 {p.uav.name}: {fpath} '
                    f'({len(p.mission)} 指令, V={p.required_speed_mps:.1f} m/s, '
                    f'feasibility={p.feasibility.value})'
                )
        self._write_briefing(export_dir, mode_label)
        return files

    @property
    def tot_sec(self) -> float:
        """規劃後的基準命中時間 (Time On Target)"""
        return self._tot_sec

    @property
    def plans(self) -> List[StrikePlan]:
        """規劃結果的複製參照"""
        return list(self._plans)

    # ─────────────────────────────────────────────────────────────────
    #  Step 1：全向突防攻擊向量分配
    # ─────────────────────────────────────────────────────────────────
    def _assign_attack_vectors(self) -> List[Tuple[UAV, float]]:
        """將 N 架 UAV 分配到 360° 均勻分佈的攻擊方位角。

        策略：
          - ψ_k = (360° / N) × k + offset           , k ∈ [0, N)
          - 將 UAV 依「相對目標的方位角 β_i」升冪排序
          - 同樣把 ψ_k 升冪排序後與 UAV 一一對應
          → 北方的 UAV 對應「從北攻擊 slot」，自然最短且不交叉
        """
        n = len(self.uavs)
        if n == 1:
            # 單機：直接使用朝向目標的方位角
            u = self.uavs[0]
            brg = _bearing_deg(u.lat, u.lon, self.target.lat, self.target.lon)
            return [(u, brg)]

        # 計算每架 UAV 到目標的方位角
        uav_bearings = [
            (u, _bearing_deg(u.lat, u.lon, self.target.lat, self.target.lon))
            for u in self.uavs
        ]
        uav_bearings.sort(key=lambda x: x[1])

        # 生成 N 個均勻攻擊向量
        attack_vectors = sorted(
            (self.approach_offset_deg + 360.0 / n * k) % 360.0
            for k in range(n)
        )

        # 圓形指派的一致化：旋轉對齊，使第一個 slot 最接近第一個 UAV 的 β
        # (如此避免排序後「0° slot ↔ 359° UAV」的邊界問題)
        first_beta = uav_bearings[0][1]
        best_shift = 0
        best_cost = float('inf')
        for shift in range(n):
            cost = sum(
                _angular_diff(uav_bearings[i][1],
                              attack_vectors[(i + shift) % n])
                for i in range(n)
            )
            if cost < best_cost:
                best_cost = cost
                best_shift = shift

        return [
            (uav_bearings[i][0], attack_vectors[(i + best_shift) % n])
            for i in range(n)
        ]

    # ─────────────────────────────────────────────────────────────────
    #  Step 2：單機基礎 StrikePlan 構建 (含 Dubins 長度)
    # ─────────────────────────────────────────────────────────────────
    def _build_base_plan(self, uav: UAV,
                         attack_heading_deg: float,
                         slot_index: int) -> StrikePlan:
        """生成 UAV 的空間軌跡參數 (IP、Dubins 長度、時間)"""
        # IP (Initial Point) = 從目標沿「攻擊方位角 + 180°」反推 ip_distance 距離
        ip_lat, ip_lon = _destination(
            self.target.lat, self.target.lon,
            (attack_heading_deg + 180.0) % 360.0,
            self.ip_distance_m,
        )

        # UAV 起飛後的初始航向取「朝向 IP」以最小化初始轉彎
        start_heading = _bearing_deg(uav.lat, uav.lon, ip_lat, ip_lon)

        # 水平 Dubins 路徑長度 (起飛 → IP)
        dubins_len_to_ip = dubins_shortest_length(
            uav.lat, uav.lon, start_heading,
            ip_lat, ip_lon, attack_heading_deg,
            self.turn_radius_m,
        )

        # 最終直線段：IP → 目標
        final_dist = _haversine(ip_lat, ip_lon,
                                self.target.lat, self.target.lon)

        # 總水平距離
        horiz_len = dubins_len_to_ip + final_dist

        # 3D 長度近似：加上爬升段斜線修正 (忽略影響極小)
        climb_h = max(self.cruise_alt_m - uav.alt, 0.0)
        total_len_3d = math.hypot(horiz_len, climb_h)

        nominal_time = total_len_3d / max(self.cruise_speed_mps, 0.1)

        return StrikePlan(
            uav=uav,
            attack_heading_deg=attack_heading_deg,
            ip_lat=ip_lat,
            ip_lon=ip_lon,
            path_length_m=total_len_3d,
            nominal_flight_time_sec=nominal_time,
            required_speed_mps=self.cruise_speed_mps,  # 尚未協同，先填預設
        )

    # ─────────────────────────────────────────────────────────────────
    #  Step 3 + 4：TOT 演算與速度/盤旋補償
    # ─────────────────────────────────────────────────────────────────
    def _coordinate_tot(self, plans: List[StrikePlan]) -> float:
        """計算基準 TOT，反推每機空速，過慢者插入 Loiter 補時。

        Returns
        -------
        float
            基準 Time On Target (秒)，所有 UCAV 將在此時刻同秒命中
        """
        # 找出「名義耗時最長者」→ 其 nominal_time 即基準 TOT
        longest = max(plans, key=lambda p: p.nominal_flight_time_sec)
        tot_base = longest.nominal_flight_time_sec + self.tot_margin_sec

        logger.info(
            f'[SwarmStrike/TOT] 基準 TOT = {tot_base:.2f} s '
            f'(由 {longest.uav.name} 決定, L={longest.path_length_m:.0f} m)'
        )

        for p in plans:
            v_req = p.path_length_m / max(tot_base, 0.1)

            if v_req > self.max_speed_mps + 1e-3:
                # 案例 A：飛不夠快 → 不可行
                p.required_speed_mps = self.max_speed_mps
                p.feasibility = FeasibilityStatus.INFEASIBLE
                p.warning = (
                    f'V_req={v_req:.1f} > V_max={self.max_speed_mps:.1f}，'
                    f'無法達成 TOT'
                )

            elif v_req < self.stall_speed_mps - 1e-3:
                # 案例 B：飛太慢 → 夾至 V_stall + 插入 Loiter
                p.required_speed_mps = self.stall_speed_mps
                p.feasibility = FeasibilityStatus.LOITER_INSERTED

                # 以失速速度飛完路徑所需時間
                t_fly = p.path_length_m / self.stall_speed_mps
                # 需盤旋消耗的時間差
                p.loiter_time_sec = max(0.0, tot_base - t_fly)
                # 對應圈數 (單圈時間 = 2πR / V_stall)
                t_lap = 2.0 * math.pi * self.turn_radius_m / self.stall_speed_mps
                p.loiter_turns = p.loiter_time_sec / max(t_lap, 0.1)

                # 盤旋點：採 UAV 起飛點上方 (水平位置不動，高度=cruise_alt)
                # 這樣 UAV 起飛爬升後就地盤旋，補時後再出發
                p.loiter_lat = p.uav.lat
                p.loiter_lon = p.uav.lon
                p.loiter_radius_m = self.turn_radius_m

                p.warning = (
                    f'V_req={v_req:.2f} < V_stall={self.stall_speed_mps:.1f} '
                    f'→ 夾 V_stall + Loiter {p.loiter_time_sec:.1f}s '
                    f'({p.loiter_turns:.1f} 圈)'
                )

            else:
                # 案例 C：速度在合理區間內 → 直接調速
                p.required_speed_mps = v_req
                p.feasibility = FeasibilityStatus.OK

        return tot_base

    # ─────────────────────────────────────────────────────────────────
    #  Step 5：MAVLink 任務序列生成
    # ─────────────────────────────────────────────────────────────────
    def _build_mission_sequence(self, p: StrikePlan) -> List[MissionItem]:
        """根據 StrikePlan 產生單架 UCAV 的 MAVLink 任務序列。

        序列規範：
          0. DO_SET_HOME       → 起飛點錨定
          1. DO_CHANGE_SPEED   → 每機專屬 V_req (TOT 協同的核心)
          2. NAV_TAKEOFF       → 爬升至巡航高度
          3. NAV_LOITER_TIME   → [可選] Loiter 補時盤旋
          4. NAV_WAYPOINT      → IP (俯衝起始點)
          5. NAV_WAYPOINT      → 目標命中點
        """
        seq: List[MissionItem] = []
        # 各機依 slot index 錯層高度
        cruise_alt = self.cruise_alt_m + self._get_slot_index(p) * self.altitude_step_m

        # 0. DO_SET_HOME
        seq.append(MissionItem(
            cmd=MAVCmd.DO_SET_HOME,
            lat=p.uav.lat, lon=p.uav.lon, alt=p.uav.alt,
            comment='起飛點錨定 (Home)',
        ))

        # 1. DO_CHANGE_SPEED (TOT 協同核心)
        #    param1 = 0 → Airspeed   |   param2 = V_req   |   param3 = -1 不改油門
        seq.append(MissionItem(
            cmd=MAVCmd.DO_CHANGE_SPEED,
            param1=0.0,
            param2=float(p.required_speed_mps),
            param3=-1.0,
            comment=f'TOT 協同空速 V={p.required_speed_mps:.2f} m/s',
        ))

        # 2. NAV_TAKEOFF
        #    param1 = 爬升俯仰 (rad)，固定翼約 8°；  lat/lon 取 IP 方向的一小段距離
        takeoff_brg = _bearing_deg(p.uav.lat, p.uav.lon, p.ip_lat, p.ip_lon)
        tk_lat, tk_lon = _destination(p.uav.lat, p.uav.lon,
                                      takeoff_brg, 100.0)
        seq.append(MissionItem(
            cmd=MAVCmd.NAV_TAKEOFF,
            lat=tk_lat, lon=tk_lon, alt=cruise_alt,
            param1=math.radians(8.0),
            comment=f'爬升至 {cruise_alt:.0f} m',
        ))

        # 3. [可選] NAV_LOITER_TIME 補時盤旋
        if p.feasibility == FeasibilityStatus.LOITER_INSERTED and p.loiter_time_sec > 0.5:
            seq.append(MissionItem(
                cmd=MAVCmd.NAV_LOITER_TIME,
                lat=p.loiter_lat, lon=p.loiter_lon, alt=cruise_alt,
                param1=float(p.loiter_time_sec),   # 盤旋時間 (秒)
                param2=0.0,
                param3=float(p.loiter_radius_m),   # 盤旋半徑 (m)
                param4=1.0,                        # 達高度後結束
                comment=f'Loiter 補時 {p.loiter_time_sec:.1f}s '
                        f'({p.loiter_turns:.1f} 圈)',
            ))

        # 4. NAV_WAYPOINT (IP — 最後一個巡航點，準備進入終端)
        seq.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=p.ip_lat, lon=p.ip_lon, alt=cruise_alt,
            param2=float(self.turn_radius_m),      # acceptance radius
            comment=f'IP (攻擊方位 {p.attack_heading_deg:.1f}°)',
        ))

        # 5. NAV_WAYPOINT (目標命中點 — acceptance 縮小確保貼地)
        final_accept = max(10.0, self.turn_radius_m * 0.25)
        seq.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=self.target.lat, lon=self.target.lon, alt=self.target.alt,
            param2=final_accept,
            comment=f'IMPACT {self.target.name}',
        ))

        return seq

    def _get_slot_index(self, plan: StrikePlan) -> int:
        """以 plan 於 self._plans 中的索引作為 slot (供高度錯層)"""
        # plan() 首次呼叫時 _plans 尚未填入，退化為 uav_id 當 slot
        if self._plans:
            try:
                return self._plans.index(plan)
            except ValueError:
                pass
        return plan.uav.uav_id - 1

    # ─────────────────────────────────────────────────────────────────
    #  輔助：日誌與任務簡報
    # ─────────────────────────────────────────────────────────────────
    def _log_summary(self, plans: List[StrikePlan]) -> None:
        logger.info(f'[SwarmStrike] 規劃完成，TOT = {self._tot_sec:.2f}s')
        for p in plans:
            loiter = f' +Loiter {p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.5 else ''
            logger.info(
                f'  [{p.feasibility.value:<16s}] '
                f'{p.uav.name} → ψ={p.attack_heading_deg:5.1f}°  '
                f'L={p.path_length_m:7.0f} m  '
                f'V={p.required_speed_mps:5.2f} m/s{loiter}'
                + (f'   ! {p.warning}' if p.warning else '')
            )

    def _write_briefing(self, export_dir: str, mode_label: str) -> None:
        """輸出任務簡報 TXT (指揮官版)"""
        path = os.path.join(export_dir, f'{mode_label}_briefing.txt')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(f'{mode_label} 飽和攻擊任務簡報\n')
                f.write('=' * 60 + '\n\n')
                f.write(f'目標：{self.target.name} '
                        f'({self.target.lat:.6f}, {self.target.lon:.6f}, '
                        f'{self.target.alt:.1f} m)\n')
                f.write(f'UCAV 數量：{len(self._plans)}\n')
                f.write(f'基準 TOT ：{self._tot_sec:.2f} s\n')
                f.write(f'巡航速度 ：預設 {self.cruise_speed_mps:.1f} m/s '
                        f'(範圍 {self.stall_speed_mps:.1f}~{self.max_speed_mps:.1f})\n')
                f.write(f'轉彎半徑 ：{self.turn_radius_m:.0f} m\n')
                f.write(f'IP 距離  ：{self.ip_distance_m:.0f} m\n\n')

                f.write(f'{"UCAV":<10s} {"ψ(°)":>7s} {"L(m)":>8s} '
                        f'{"V(m/s)":>8s} {"Loiter":>10s} {"狀態":<18s} 備註\n')
                f.write('-' * 100 + '\n')
                for p in self._plans:
                    loi = f'{p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.1 else '-'
                    f.write(
                        f'{p.uav.name:<10s} '
                        f'{p.attack_heading_deg:>7.1f} '
                        f'{p.path_length_m:>8.0f} '
                        f'{p.required_speed_mps:>8.2f} '
                        f'{loi:>10s} '
                        f'{p.feasibility.value:<18s} '
                        f'{p.warning}\n'
                    )
                f.write('\nMAVLink 任務序列範本:\n')
                f.write('  DO_SET_HOME(179) → DO_CHANGE_SPEED(178) → NAV_TAKEOFF(22)\n')
                f.write('   → [NAV_LOITER_TIME(19)] → NAV_WAYPOINT(16, IP)\n')
                f.write('   → NAV_WAYPOINT(16, IMPACT)\n')
            logger.info(f'[SwarmStrike] 簡報已寫出: {path}')
        except Exception as e:
            logger.error(f'[SwarmStrike] 寫簡報失敗: {e}')


# ═══════════════════════════════════════════════════════════════════════
#  小工具
# ═══════════════════════════════════════════════════════════════════════

def _angular_diff(a_deg: float, b_deg: float) -> float:
    """兩角度的最短差值 (0 ≤ result ≤ 180°)"""
    d = abs((a_deg - b_deg) % 360.0)
    return min(d, 360.0 - d)


# ═══════════════════════════════════════════════════════════════════════
#  Main Demo — 3 架 Shahed-136 類 OWA-UAV 從異地同時命中台北目標
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    # ─── 1. 定義目標 (台北 101 附近地面座標) ─────────────────────
    target = Target(lat=25.0330, lon=121.5654, alt=0.0, name='TGT-Taipei101')

    # ─── 2. 3 架 UCAV 從不同方向發射 (異地 = DTOT 場景) ─────────
    uavs = [
        # 東北外海發射 (最遠)
        UAV(uav_id=1, lat=25.25, lon=122.05, alt=0.0, name='UCAV-NE'),
        # 西北內陸發射
        UAV(uav_id=2, lat=25.12, lon=121.28, alt=0.0, name='UCAV-NW'),
        # 南方近海發射 (最近)
        UAV(uav_id=3, lat=24.85, lon=121.60, alt=0.0, name='UCAV-S'),
    ]

    # ─── 3. 建立規劃器 (Shahed-136 物理參數近似) ────────────────
    #    真機公開資料：巡航 ~45 m/s，失速 ~28 m/s，最大 ~62 m/s
    planner = SwarmStrikePlanner(
        target=target,
        uavs=uavs,
        cruise_speed_mps   = 45.0,    # Shahed-136 典型巡航
        max_speed_mps      = 62.0,    # ARSPD_FBW_MAX
        stall_speed_mps    = 28.0,    # ARSPD_FBW_MIN
        turn_radius_m      = 150.0,   # Delta-wing R_min
        cruise_alt_m       = 500.0,   # 低空突防 AGL
        altitude_step_m    = 40.0,    # 多機高度錯層
        ip_distance_m      = 2000.0,  # IP 距目標 2 km (俯衝起始)
        approach_offset_deg= 0.0,     # 第一個 slot 指向正北
        tot_margin_sec     = 0.0,     # 嚴格同秒命中
    )

    # ─── 4. 執行規劃 (空間 + 時間 + MAVLink 序列) ───────────────
    plans = planner.plan()

    print('\n' + '═' * 70)
    print(f' STOT/DTOT 規劃結果   TOT = {planner.tot_sec:.2f} s')
    print('═' * 70)
    print(f'{"UCAV":<10s} {"ψ(°)":>6s} {"L(m)":>8s} {"V(m/s)":>8s}'
          f' {"Loiter":>10s} {"Feasibility":<18s}')
    print('-' * 70)
    for p in plans:
        loi = f'{p.loiter_time_sec:.1f}s' if p.loiter_time_sec > 0.1 else '-'
        print(f'{p.uav.name:<10s} {p.attack_heading_deg:>6.1f} '
              f'{p.path_length_m:>8.0f} {p.required_speed_mps:>8.2f}'
              f' {loi:>10s} {p.feasibility.value:<18s}')

    # ─── 5. 每架 UCAV 的 MAVLink 序列 (前 8 筆) ─────────────────
    print('\n' + '═' * 70)
    print(' MAVLink 任務序列範例')
    print('═' * 70)
    for p in plans[:1]:  # 只秀第一架為範例
        print(f'\n── {p.uav.name} ────')
        for i, item in enumerate(p.mission):
            print(f'  seq {i}: cmd={item.cmd:3d}  '
                  f'lat={item.lat:10.6f}  lon={item.lon:11.6f}  '
                  f'alt={item.alt:6.1f}  '
                  f'p1={item.param1:7.2f} p2={item.param2:7.2f}  '
                  f'— {item.comment}')

    # ─── 6. 匯出 QGC WPL 110 .waypoints + 任務簡報 ──────────────
    out_dir = os.path.join(os.getcwd(), 'swarm_strike_out')
    files = planner.export_qgc_wpl(out_dir, mode_label='DTOT')
    print(f'\n已匯出 {len(files)} 份航點檔至 {out_dir}')
    for f in files:
        print(f'  • {os.path.basename(f)}')
    print(f'\n[ArduPilot SITL 使用方式]')
    print(f'  Mission Planner → Ctrl+F → Load WP → 選擇對應 .waypoints')
    print(f'  切換 AUTO 模式 → ARM → 自動執行蜂群打擊')
