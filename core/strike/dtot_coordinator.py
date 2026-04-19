"""
DTOT / STOT 飽和攻擊時間空間協同演算法
====================================

戰術背景
--------
飽和攻擊 (Saturation Attack) 透過「多方向同時命中」讓防空系統在極短時間內
同時接戰 N 個來襲目標，達到火力通道飽和、反應時間超限的效果。

    DTOT (Different launch / Time On Target)
        異地發射、同時命中：多架 UCAV 從不同起飛點出發，
        各自規劃不同長度的航線，但在同一 TOT（Time On Target）秒擊中目標。

    STOT (Simultaneous launch / Time On Target)
        同地發射、同時命中：所有 UCAV 同時離地，
        透過不同航線長度錯開飛行時間，最終同秒命中目標。

演算法流程
----------
    1. 取得所有 UCAV 的 3D 軌跡 (StrikeTrajectory) 與基礎飛行距離 L_i
    2. 以最長軌跡計算基準 TOT：TOT_base = max(L_i) / cruise_speed
       └─ STOT 模式再疊加 takeoff_offset（各機起飛時刻差為 0，最長者決定 TOT）
    3. 反推每架 UCAV 應採用的巡航空速：V_i = L_i / (TOT_base - t_climb_i)
    4. 速度邊界檢查：
         ├─ V_i ≤ V_stall → 無法更慢 → 插入 S-turn/Loiter 補時
         ├─ V_i ≥ V_max   → 無法更快 → 回報不可行（或縮短 TOT 重算）
         └─ V_stall < V_i < V_max → 可行，直接套用
    5. 為需要補時的 UCAV 在起飛後巡航段前插入 NAV_LOITER_TIME
       以 R_min 最小轉彎半徑盤旋，補足 Δt = (需求時間) - (以 V_stall 飛完的時間)
    6. 匯出 QGC WPL 110：每機一檔，在 NAV_TAKEOFF 後插入
       MAV_CMD_DO_CHANGE_SPEED(178)，確保 SITL 即時執行指定巡航空速

MAVLink 指令對照
---------------
    MAV_CMD_DO_SET_HOME         (179) — Home 佔位
    MAV_CMD_DO_CHANGE_SPEED     (178) — 時間協同核心：param2=V_i(m/s)
    MAV_CMD_NAV_TAKEOFF          (22) — 固定翼斜坡起飛
    MAV_CMD_NAV_LOITER_TIME      (19) — S-turn 補時盤旋
    MAV_CMD_NAV_WAYPOINT         (16) — 巡航 / 俯衝段航點
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from core.strike.terminal_strike_planner import (
    StrikeTrajectory,
    StrikeWaypoint,
)
from utils.file_io import create_waypoint_line, write_waypoints
from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  MAVLink 命令常數（避免依賴 pymavlink，保持與現有 exporter 風格一致）
# ═══════════════════════════════════════════════════════════════════════
_CMD_NAV_WAYPOINT        = 16
_CMD_NAV_LOITER_TIME     = 19   # param1=loiter_time(sec), param3=radius(m)
_CMD_NAV_TAKEOFF         = 22
_CMD_DO_CHANGE_SPEED     = 178  # param1=speed_type, param2=speed(m/s), param3=throttle(-1=no change)
_CMD_DO_SET_HOME         = 179

# MAV_FRAME
_FRAME_GLOBAL_REL_ALT    = 3    # 相對起飛高度，Plane/Copter 任務最常用

# 地球半徑
_R_EARTH = 6_371_000.0


# ═══════════════════════════════════════════════════════════════════════
#  資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class TimingPlan:
    """單架 UCAV 的時間協同結果

    欄位說明
    --------
    uav_id / uav_name / target_id / target_name
        來源軌跡的識別資訊，供匯出檔名使用
    baseline_distance_m
        原始 3D 軌跡長度（含起飛爬升 + 巡航 + 俯衝）
    required_speed_mps
        為滿足 TOT 所需的巡航空速（經過 stall/max 邊界夾限）
    tot_sec
        預計命中時間（從 t=0 起飛算起）
    holding_time_sec
        為補時而插入的 S-turn 盤旋時間
    holding_center_lat / lon / radius_m
        S-turn 盤旋圓心地理座標與半徑
    feasible
        物理上是否可滿足 TOT 需求
    speed_clamped
        是否觸及 stall 或 max 邊界（僅作警示）
    """
    uav_id: int
    uav_name: str
    target_id: int
    target_name: str

    baseline_distance_m: float = 0.0
    climb_time_sec: float = 0.0          # 起飛爬升段時間（不受 speed 調整）
    cruise_distance_m: float = 0.0       # 巡航+俯衝段距離（受 speed 調整）
    nominal_cruise_time_sec: float = 0.0 # 以 cruise_speed 飛完的時間

    required_speed_mps: float = 0.0
    tot_sec: float = 0.0

    holding_time_sec: float = 0.0
    holding_center_lat: float = 0.0
    holding_center_lon: float = 0.0
    holding_radius_m: float = 0.0

    feasible: bool = True
    speed_clamped: bool = False
    warning: str = ''

    trajectory: Optional[StrikeTrajectory] = None


# ═══════════════════════════════════════════════════════════════════════
#  幾何工具（與 terminal_strike_planner 一致的 Haversine 風格）
# ═══════════════════════════════════════════════════════════════════════

def _haversine_3d(wp1: StrikeWaypoint, wp2: StrikeWaypoint) -> float:
    """兩個 3D 航點的歐氏距離 (m)，水平用 Haversine + 垂直差 sqrt 合成"""
    rlat1, rlon1 = math.radians(wp1.lat), math.radians(wp1.lon)
    rlat2, rlon2 = math.radians(wp2.lat), math.radians(wp2.lon)
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    horiz = 2 * _R_EARTH * math.asin(min(1.0, math.sqrt(a)))
    dh = wp2.alt - wp1.alt
    return math.hypot(horiz, dh)


def _segment_length(waypoints: List[StrikeWaypoint],
                    start_idx: int, end_idx: int) -> float:
    """累計某一段（含頭尾）的 3D 路徑長度"""
    if end_idx <= start_idx or end_idx >= len(waypoints):
        return 0.0
    total = 0.0
    for i in range(start_idx, end_idx):
        total += _haversine_3d(waypoints[i], waypoints[i + 1])
    return total


# ═══════════════════════════════════════════════════════════════════════
#  DTOT / STOT 時空協同演算器
# ═══════════════════════════════════════════════════════════════════════

class DTOTCoordinator:
    """
    飽和攻擊時空協同器

    使用流程（典型）::

        from core.strike.terminal_strike_planner import TerminalStrikePlanner
        from core.strike.dtot_coordinator import DTOTCoordinator

        planner = TerminalStrikePlanner(max_dive_angle_deg=55, cruise_speed_mps=60)
        trajectories = planner.plan(uavs, targets)

        coord = DTOTCoordinator(
            cruise_speed_mps=60.0,
            max_speed_mps=85.0,
            stall_speed_mps=25.0,
            min_turn_radius_m=150.0,
        )
        plans = coord.coordinate(trajectories, mode='DTOT')
        coord.export_qgc_wpl(plans, export_dir='./mission_out/')

    Parameters
    ----------
    cruise_speed_mps : float
        預設巡航空速，用於「最長距離」UCAV 作為 TOT 基準
    max_speed_mps : float
        飛控允許的最大空速上限（ARSPD_FBW_MAX 等價）
    stall_speed_mps : float
        失速速度下限（ARSPD_FBW_MIN 等價）
    min_turn_radius_m : float
        固定翼最小轉彎半徑，用於 S-turn/Loiter 盤旋補時
    tot_margin_sec : float
        額外安全邊界（同秒命中的容差，預設 0.0 = 嚴格同秒）
    """

    def __init__(self,
                 cruise_speed_mps: float = 60.0,
                 max_speed_mps: float = 85.0,
                 stall_speed_mps: float = 22.0,
                 min_turn_radius_m: float = 150.0,
                 tot_margin_sec: float = 0.0):
        if max_speed_mps <= stall_speed_mps:
            raise ValueError(
                f'max_speed ({max_speed_mps}) 必須 > stall_speed ({stall_speed_mps})'
            )
        if not (stall_speed_mps <= cruise_speed_mps <= max_speed_mps):
            logger.warning(
                f'[DTOT] cruise_speed={cruise_speed_mps} 超出 '
                f'[{stall_speed_mps}, {max_speed_mps}] 範圍，已夾限'
            )
            cruise_speed_mps = max(stall_speed_mps,
                                   min(cruise_speed_mps, max_speed_mps))

        self.cruise_speed_mps = cruise_speed_mps
        self.max_speed_mps = max_speed_mps
        self.stall_speed_mps = stall_speed_mps
        self.min_turn_radius_m = min_turn_radius_m
        self.tot_margin_sec = tot_margin_sec

    # ─────────────────────────────────────────────────────────────────
    #  公開 API：協同規劃
    # ─────────────────────────────────────────────────────────────────
    def coordinate(self,
                   trajectories: List[StrikeTrajectory],
                   mode: str = 'STOT',
                   interval_sec: float = 0.0,
                   ) -> List[TimingPlan]:
        """計算所有 UCAV 的動態巡航速度與 S-turn 補時。

        【2026 架構更新】明確區分兩種時間協同模式：

        Parameters
        ----------
        trajectories : List[StrikeTrajectory]
            來自 TerminalStrikePlanner 的 3D 軌跡列表
        mode : {'STOT', 'DTOT'}
            STOT (Simultaneous TOT) — 同時命中：全體 UCAV 同一秒擊中目標。
                基準 TOT = max(L_i / V_cruise)，其餘機調速或補 Loiter。
            DTOT (Distributed TOT) — 間隔命中：slot k 在 base + k·Δ 秒命中。
                最長 L 分到 slot 0（不補時），其餘依序延遲。
        interval_sec : float
            DTOT 模式的相鄰命中時刻間隔（秒）；STOT 忽略。
            範例：3 架 + interval=15 → 命中時刻 T, T+15, T+30。

        Returns
        -------
        List[TimingPlan]
            每架 UCAV 的時空協同結果；`tot_sec` 為該機的實際命中時刻
        """
        if not trajectories:
            logger.warning('[DTOT] 軌跡列表為空')
            return []

        mode = mode.upper()
        if mode not in ('DTOT', 'STOT'):
            raise ValueError(f'不支援的 mode: {mode}（只能是 DTOT 或 STOT）')

        # ── Step 1: 計算每架 UCAV 的分段距離（爬升 / 巡航+俯衝）────
        plans: List[TimingPlan] = []
        for tr in trajectories:
            wps = tr.waypoints
            if len(wps) < 2:
                logger.warning(f'[DTOT] {tr.uav_name} 軌跡航點不足，略過')
                continue

            cruise_start = max(tr.cruise_start_index, 0)
            impact_idx = tr.impact_index if tr.impact_index > 0 else len(wps) - 1

            climb_len = _segment_length(wps, 0, cruise_start)
            cruise_len = _segment_length(wps, cruise_start, impact_idx)
            total_len = climb_len + cruise_len

            # 爬升段時間 = 以軌跡自帶的 time_sec 為準（由 planner 寫入）
            if cruise_start < len(wps):
                climb_time = wps[cruise_start].time_sec
            else:
                climb_time = 0.0

            nominal_cruise_time = cruise_len / max(self.cruise_speed_mps, 0.1)

            plans.append(TimingPlan(
                uav_id=tr.uav_id,
                uav_name=tr.uav_name,
                target_id=tr.target_id,
                target_name=tr.target_name,
                baseline_distance_m=total_len,
                climb_time_sec=climb_time,
                cruise_distance_m=cruise_len,
                nominal_cruise_time_sec=nominal_cruise_time,
                trajectory=tr,
            ))

        if not plans:
            return []

        # ══════════════════════════════════════════════════════════════
        #  2026 重構：Step 2+3 委派給 core.strike.time_coordination 共用模組
        #  (消除與 vtol/recon/advanced 系列 planner 的重複邏輯)
        # ══════════════════════════════════════════════════════════════
        from core.strike.time_coordination import (
            compute_tot_schedule, compute_loiter_plan, FeasibilityStatus,
        )

        # Step 2: STOT/DTOT 命中時刻排程 (含 base_tot 計算 + slot 指派)
        nominal_times = [p.climb_time_sec + p.nominal_cruise_time_sec for p in plans]
        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=nominal_times,
            mode=mode,
            interval_sec=interval_sec,
            dtot_order='longest_first',        # 最長 L → slot 0
            tot_margin_sec=self.tot_margin_sec,
            payloads=plans,
        )
        # 回寫各機命中時刻
        for slot in slots:
            slot.payload.tot_sec = slot.arrival_time_sec

        if mode == 'DTOT' and interval_sec > 0.01:
            logger.info(
                f'[DTOT 間隔] base={base_tot:.2f}s, Δ={interval_sec:.1f}s → '
                f'命中時刻: '
                + ', '.join(f'{base_tot + k * interval_sec:.1f}'
                            for k in range(len(plans)))
            )
        else:
            longest_plan = max(plans, key=lambda p: (p.climb_time_sec
                                                      + p.nominal_cruise_time_sec))
            logger.info(
                f'[STOT 同時] TOT = {base_tot:.2f}s '
                f'(由 {longest_plan.uav_name} 決定, '
                f'L={longest_plan.baseline_distance_m:.0f}m)'
            )

        # Step 3: 逐機反推空速 + Loiter 決策 (dtot_coordinator 特有：支援調速)
        is_dtot_interval = (mode == 'DTOT' and interval_sec > 0.01)
        for p in plans:
            out = compute_loiter_plan(
                path_length_m=p.cruise_distance_m,
                arrival_time_sec=p.tot_sec,
                climb_time_sec=p.climb_time_sec,
                cruise_speed_mps=self.cruise_speed_mps,
                stall_speed_mps=self.stall_speed_mps,
                max_speed_mps=self.max_speed_mps,
                loiter_radius_m=self.min_turn_radius_m,
                dtot_interval_mode=is_dtot_interval,
            )
            p.required_speed_mps = out.required_speed_mps
            p.holding_time_sec = out.wait_time_sec
            p.warning = out.warning

            # 狀態映射
            if out.feasibility in (FeasibilityStatus.INFEASIBLE,
                                    FeasibilityStatus.PRECLIMB_FAIL):
                p.feasible = False
                p.speed_clamped = (out.feasibility == FeasibilityStatus.INFEASIBLE)
            elif out.feasibility == FeasibilityStatus.LOITER_INSERTED:
                p.speed_clamped = (out.required_speed_mps == self.stall_speed_mps)

            # 若需盤旋，設定圓心 (巡航段起點)
            if p.holding_time_sec > 0.5 and p.trajectory is not None:
                cs_idx = p.trajectory.cruise_start_index
                if 0 < cs_idx < len(p.trajectory.waypoints):
                    cs_wp = p.trajectory.waypoints[cs_idx]
                    p.holding_center_lat = cs_wp.lat
                    p.holding_center_lon = cs_wp.lon
                p.holding_radius_m = self.min_turn_radius_m

        # ── Step 4: 日誌輸出 ─────────────────────────────────────
        logger.info(f'[{mode}] 時空協同結果 ({len(plans)} 架):')
        for p in plans:
            flag = 'OK' if p.feasible else 'NG'
            hold = f' +Loiter {p.holding_time_sec:.1f}s' if p.holding_time_sec > 0 else ''
            logger.info(
                f'  {flag} {p.uav_name} → {p.target_name}: '
                f'L={p.baseline_distance_m:.0f}m, V={p.required_speed_mps:.1f}m/s, '
                f'TOT={p.tot_sec:.1f}s{hold}'
                + (f'  ! {p.warning}' if p.warning else '')
            )
        return plans

    # ─────────────────────────────────────────────────────────────────
    #  QGC WPL 110 匯出（每機一檔，含 DO_CHANGE_SPEED + 可選 Loiter）
    # ─────────────────────────────────────────────────────────────────
    def export_qgc_wpl(self,
                       plans: List[TimingPlan],
                       export_dir: str,
                       cruise_accept_radius_m: Optional[float] = None,
                       dive_accept_radius_m: Optional[float] = None,
                       mode: str = 'DTOT',
                       write_briefing: bool = True,
                       ) -> List[str]:
        """將協同結果匯出為 QGC WPL 110 航點檔（每架一檔）。

        Parameters
        ----------
        plans : List[TimingPlan]
            coordinate() 的回傳值
        export_dir : str
            輸出目錄（會自動建立）
        cruise_accept_radius_m : float, optional
            巡航段 acceptance radius（param2），預設 = min_turn_radius_m
        dive_accept_radius_m : float, optional
            俯衝段 acceptance radius，預設 = 0.25 × min_turn_radius_m
        mode : str
            戰術標記（'DTOT' / 'STOT'），僅影響檔名與簡報內容
        write_briefing : bool
            是否一併寫出任務簡報 TXT

        Returns
        -------
        List[str]
            成功寫出的檔案絕對路徑列表
        """
        os.makedirs(export_dir, exist_ok=True)

        if cruise_accept_radius_m is None:
            cruise_accept_radius_m = self.min_turn_radius_m
        if dive_accept_radius_m is None:
            dive_accept_radius_m = max(10.0, self.min_turn_radius_m * 0.25)

        exported_files: List[str] = []
        for p in plans:
            if p.trajectory is None or not p.trajectory.waypoints:
                continue

            lines = ['QGC WPL 110']
            seq = 0
            tr = p.trajectory

            # ── seq 0: DO_SET_HOME (起飛點錨定) ──────────────────
            lines.append(create_waypoint_line(
                seq=seq, command=_CMD_DO_SET_HOME,
                lat=tr.takeoff_lat, lon=tr.takeoff_lon, alt=0.0,
                frame=_FRAME_GLOBAL_REL_ALT,
                current=1, autocontinue=1,
            ))
            seq += 1

            # ── seq 1: DO_CHANGE_SPEED — TIME COORDINATION CORE ─────
            # param1: 0=Airspeed, 1=Groundspeed（Plane 用 Airspeed）
            # param2: 速度 (m/s) ← 每機專屬的 V_i
            # param3: -1 = 油門不變
            lines.append(create_waypoint_line(
                seq=seq, command=_CMD_DO_CHANGE_SPEED,
                param1=0.0,                              # 空速模式
                param2=float(p.required_speed_mps),      # ← 時空協同的核心
                param3=-1.0,
                frame=_FRAME_GLOBAL_REL_ALT,
                current=0, autocontinue=1,
            ))
            seq += 1

            # ── seq 2: NAV_TAKEOFF (爬升至巡航高度) ─────────────
            cs_idx = max(tr.cruise_start_index, 1)
            if cs_idx < len(tr.waypoints):
                t0 = tr.waypoints[cs_idx]
                lines.append(create_waypoint_line(
                    seq=seq, command=_CMD_NAV_TAKEOFF,
                    param1=math.radians(8.0),       # 最小爬升俯仰 (rad)
                    param4=0.0,                     # 使用當前航向
                    lat=t0.lat, lon=t0.lon,
                    alt=tr.cruise_alt_m,
                    frame=_FRAME_GLOBAL_REL_ALT,
                    current=0, autocontinue=1,
                ))
                seq += 1

            # ── 可選 NAV_LOITER_TIME — S-turn 補時盤旋 ──────────
            #   插在 TAKEOFF 之後、巡航段之前，讓慢飛者「消耗時間」。
            if p.holding_time_sec > 0.5:
                # LOITER 中心 = 巡航段起點；高度 = 巡航高度
                lines.append(create_waypoint_line(
                    seq=seq, command=_CMD_NAV_LOITER_TIME,
                    param1=float(p.holding_time_sec),     # 盤旋時間 (sec)
                    param2=0.0,                            # 離場不支援
                    param3=float(p.holding_radius_m),      # 盤旋半徑 (m)
                    param4=1.0,                            # 達高度方結束
                    lat=p.holding_center_lat,
                    lon=p.holding_center_lon,
                    alt=tr.cruise_alt_m,
                    frame=_FRAME_GLOBAL_REL_ALT,
                    current=0, autocontinue=1,
                ))
                seq += 1

            # ── 巡航段 NAV_WAYPOINT ─────────────────────────────
            dive_idx = tr.dive_start_index if tr.dive_start_index > 0 else len(tr.waypoints) - 1
            for wp in tr.waypoints[cs_idx + 1:dive_idx + 1]:
                lines.append(create_waypoint_line(
                    seq=seq, command=_CMD_NAV_WAYPOINT,
                    param2=float(cruise_accept_radius_m),   # acceptance radius
                    lat=wp.lat, lon=wp.lon, alt=wp.alt,
                    frame=_FRAME_GLOBAL_REL_ALT,
                    current=0, autocontinue=1,
                ))
                seq += 1

            # ── 俯衝段 NAV_WAYPOINT（acceptance 縮小確保貼目標）──
            for wp in tr.waypoints[dive_idx + 1:]:
                lines.append(create_waypoint_line(
                    seq=seq, command=_CMD_NAV_WAYPOINT,
                    param2=float(dive_accept_radius_m),
                    lat=wp.lat, lon=wp.lon, alt=wp.alt,
                    frame=_FRAME_GLOBAL_REL_ALT,
                    current=0, autocontinue=1,
                ))
                seq += 1

            # ── 寫檔 ─────────────────────────────────────────────
            safe_name = f'{mode}_{tr.uav_name}_to_{tr.target_name}_V{p.required_speed_mps:.0f}mps_TOT{p.tot_sec:.0f}s'
            safe_name = safe_name.replace('/', '-').replace('\\', '-')
            filepath = os.path.join(export_dir, f'{safe_name}.waypoints')

            if write_waypoints(filepath, lines):
                exported_files.append(filepath)
                logger.info(
                    f'[DTOT] 匯出 {tr.uav_name}: {filepath} '
                    f'({seq} 航點, V={p.required_speed_mps:.1f}m/s)'
                )

        # ── 任務簡報 ────────────────────────────────────────────
        if write_briefing and exported_files:
            self._write_briefing(plans, export_dir, mode)

        return exported_files

    def _write_briefing(self, plans: List[TimingPlan],
                        export_dir: str, mode: str) -> None:
        """寫出指揮官任務簡報 TXT"""
        path = os.path.join(export_dir, f'{mode}_mission_briefing.txt')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(f'{mode} 飽和攻擊協同任務簡報\n')
                f.write('=' * 52 + '\n\n')
                f.write(f'戰術模式：{mode} ')
                if mode == 'DTOT':
                    f.write('(Distributed TOT — 間隔命中)\n')
                else:
                    f.write('(Simultaneous TOT — 同時命中)\n')
                f.write(f'參與 UCAV：{len(plans)} 架\n')
                f.write(f'基準 TOT  ：{plans[0].tot_sec:.2f} s\n')
                f.write(f'巡航速度範圍：{self.stall_speed_mps:.0f} – '
                        f'{self.max_speed_mps:.0f} m/s (預設 {self.cruise_speed_mps:.0f})\n')
                f.write(f'最小轉彎半徑：{self.min_turn_radius_m:.0f} m\n\n')

                f.write(f'{"UCAV":<10} {"TGT":<10} {"L(m)":>8} {"V(m/s)":>8} '
                        f'{"Loiter(s)":>10} {"Status":<10} 備註\n')
                f.write('-' * 90 + '\n')
                for p in plans:
                    status = 'OK' if p.feasible else 'INFEASIBLE'
                    f.write(
                        f'{p.uav_name:<10} {p.target_name:<10} '
                        f'{p.baseline_distance_m:>8.0f} '
                        f'{p.required_speed_mps:>8.1f} '
                        f'{p.holding_time_sec:>10.1f} '
                        f'{status:<10} {p.warning}\n'
                    )
                f.write('\n[核心 MAVLink 指令序列]\n')
                f.write('  seq 0: DO_SET_HOME      (179)\n')
                f.write('  seq 1: DO_CHANGE_SPEED  (178) ← 時空協同關鍵 (param2=V_i)\n')
                f.write('  seq 2: NAV_TAKEOFF       (22)\n')
                f.write('  seq 3: NAV_LOITER_TIME   (19) ← 可選：S-turn 補時\n')
                f.write('  seq N: NAV_WAYPOINT      (16) × 巡航段 + 俯衝段\n')
            logger.info(f'[DTOT] 任務簡報已寫出: {path}')
        except Exception as e:
            logger.error(f'[DTOT] 寫出簡報失敗: {e}')


# ═══════════════════════════════════════════════════════════════════════
#  Example — 獨立執行即可看到 DTOT 協同結果
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    from core.strike.terminal_strike_planner import (
        TerminalStrikePlanner, StrikeTarget, StrikeUAV,
    )

    # 1) 建立 3 個地面目標 (台灣北部範例座標)
    targets = [
        StrikeTarget(target_id=1, lat=25.0330, lon=121.5654, alt=0.0, name='TGT-Taipei101'),
        StrikeTarget(target_id=2, lat=25.0478, lon=121.5170, alt=0.0, name='TGT-Taipei-Main'),
        StrikeTarget(target_id=3, lat=25.0173, lon=121.5397, alt=0.0, name='TGT-Signal-Hill'),
    ]

    # 2) 3 架 UCAV 以不同起飛點 (模擬 DTOT 異地發射場景)
    uavs = [
        StrikeUAV(uav_id=1, lat=25.0100, lon=121.4800, cruise_alt=500.0, speed_mps=60, name='UCAV-A'),
        StrikeUAV(uav_id=2, lat=25.0900, lon=121.4500, cruise_alt=530.0, speed_mps=60, name='UCAV-B'),
        StrikeUAV(uav_id=3, lat=24.9800, lon=121.6100, cruise_alt=560.0, speed_mps=60, name='UCAV-C'),
    ]

    # 3) 產生三段式飛行軌跡 (起飛 → Dubins 巡航 → 末端俯衝)
    planner = TerminalStrikePlanner(
        max_dive_angle_deg=55.0,
        dive_initiation_dist_m=800.0,
        cruise_alt_m=500.0,
        cruise_speed_mps=60.0,
        altitude_step_m=30.0,
        min_turn_radius_m=150.0,
    )
    trajectories = planner.plan(uavs, targets)

    # 4) DTOT 時空協同 → 各機動態空速 + 可選 Loiter 補時
    coord = DTOTCoordinator(
        cruise_speed_mps=60.0,
        max_speed_mps=85.0,
        stall_speed_mps=25.0,
        min_turn_radius_m=150.0,
    )
    plans = coord.coordinate(trajectories, mode='DTOT')

    # 5) 匯出 QGC WPL 110 (可直接載入 Mission Planner / SITL)
    out_dir = os.path.join(os.getcwd(), 'dtot_export_demo')
    files = coord.export_qgc_wpl(plans, export_dir=out_dir, mode='DTOT')
    print(f'\n已匯出 {len(files)} 份航點檔至 {out_dir}\n')
    for f in files:
        print(f'  • {f}')
