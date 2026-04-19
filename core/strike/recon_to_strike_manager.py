"""
ReconToStrikeManager — 偵打一體動態任務切換器
================================================

情境：UAV 群正在執行 DCCPP 協同覆蓋掃描，某架 UAV 的 FOV 偵測到隱藏目標，
系統需「即時中斷」掃描任務並將若干架 UAV 動態重構為「蜂群末端打擊」。

四大核心模組
------------
    1. 偵察觸發與任務重構 (Dynamic Coalition Formation)
       - target_detected_callback() 選出最適合的 N 架 UAV 組成打擊聯盟
       - 凍結當前 pose → 生成平滑過渡航點 → 不讓飛控失控

    2. 宏觀避障：高度錯層 (Altitude Layering)
       - slot k 的 UAV 分配 base_alt + k·30m 專屬高度層
       - Phase 2 (巡航 + Loiter) 絕對不會 2D 相撞

    3. 微觀避障：改進型人工勢場法 (3D IAPF)
       - 固定翼特化：切線斥力為主、徑向斥力為輔
       - 轉為 lookahead 航點，飛控直接追蹤

    4. 時間協同 (STOT)
       - 以最晚抵達的 UAV 時間為 Base_TOT
       - 為提早抵達者在專屬高度層插入 NAV_LOITER_TURNS

MAVLink 任務序列 (每架 UAV)
---------------------------
    seq 0  DO_SET_HOME
    seq 1  NAV_WAYPOINT   Transition (當前航向 + 800m，梯度高度)
    seq 2  DO_CHANGE_SPEED
    seq 3  NAV_WAYPOINT   IAPF 調整後的避障中繼點 (可選)
    seq 4  NAV_WAYPOINT   Loiter 中心 @ 分配高度層
    seq 5  NAV_LOITER_TURNS  (TOT 補時)
    seq 6  NAV_WAYPOINT   2 km 邊界 IP
    seq 7  DO_CHANGE_SPEED  末端加速
    seq 8  NAV_WAYPOINT   IMPACT
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

from core.strike.swarm_strike_planner import (
    MissionItem, dubins_shortest_length,
    _haversine, _bearing_deg, _destination, _angular_diff,
    _MAV_FRAME_REL, _R_EARTH,
)
from utils.file_io import create_waypoint_line, write_waypoints
from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  列舉與 MAVLink 常數
# ═══════════════════════════════════════════════════════════════════════

class TaskMode(Enum):
    """UAV 當前任務狀態"""
    COVERAGE      = 'COVERAGE'        # 執行 DCCPP 覆蓋掃描
    TRANSITIONING = 'TRANSITIONING'   # 切換中 (任務重構期)
    STRIKE        = 'STRIKE'          # 執行蜂群末端打擊
    IDLE          = 'IDLE'            # 閒置 (未分配聯盟)


class MAVCmd:
    NAV_WAYPOINT       = 16
    NAV_LOITER_TURNS   = 18
    NAV_LOITER_TIME    = 19
    NAV_TAKEOFF        = 22
    DO_CHANGE_SPEED    = 178
    DO_SET_HOME        = 179
    DO_PAUSE_CONTINUE  = 193   # 平滑切換必備指令


# ═══════════════════════════════════════════════════════════════════════
#  資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class UAVState:
    """UAV 即時遙測狀態 (通常由 MAVLink 心跳包更新)"""
    uav_id: int
    lat: float
    lon: float
    alt: float
    heading_deg: float                   # 當前航向 (0=北，順時針)
    ground_speed_mps: float = 25.0       # 當前地速
    task_mode: TaskMode = TaskMode.COVERAGE
    name: str = ''

    def __post_init__(self) -> None:
        if not self.name:
            self.name = f'UAV-{self.uav_id}'


@dataclass
class StrikeAssignment:
    """單架 UAV 的打擊任務指派 (偵打切換後)"""
    uav_id: int
    slot_index: int                      # 0 = 聯盟中最先抵達
    approach_heading_deg: float          # 360° 包圍切入角
    transition_alt_m: float              # 專屬過渡/巡航高度

    # 過渡航點 (首要平滑銜接)
    transition_lat: float
    transition_lon: float
    transition_alt_target_m: float

    # 中繼避障點 (IAPF 若偵測衝突才填入，否則為 None)
    avoidance_lat: Optional[float] = None
    avoidance_lon: Optional[float] = None

    # 盤旋等待點 (TOT 同步)
    loiter_lat: float = 0.0
    loiter_lon: float = 0.0
    loiter_turns: float = 0.0
    loiter_radius_m: float = 150.0

    # 2 km 邊界 IP + 命中點
    ip_lat: float = 0.0
    ip_lon: float = 0.0
    impact_lat: float = 0.0
    impact_lon: float = 0.0

    # 時間與速度
    t_arrive_ip_sec: float = 0.0         # 基準 TOT (所有 UAV 同秒抵達)
    cruise_speed_mps: float = 25.0
    terminal_speed_mps: float = 46.0

    # MAVLink 任務序列
    mission: List[MissionItem] = field(default_factory=list)


@dataclass
class CoalitionReport:
    """聯盟構成與避障摘要"""
    target_lat: float
    target_lon: float
    selected_uav_ids: List[int]
    rejected_uav_ids: List[int]
    tot_sec: float
    altitude_layers: Dict[int, float]    # uav_id → altitude
    iapf_conflicts: List[str] = field(default_factory=list)
    iapf_adjustments: List[str] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════════════
#  地理/向量工具 (ENU 區域平面)
# ═══════════════════════════════════════════════════════════════════════

def _latlon_to_enu(lat: float, lon: float, alt: float,
                   ref_lat: float, ref_lon: float,
                   ref_alt: float = 0.0) -> Tuple[float, float, float]:
    """經緯度 → 以 ref 為原點的局部 ENU (公尺)

    使用等距圓柱近似（小範圍 < 20 km 精度足夠）。
    """
    coslat = math.cos(math.radians(ref_lat))
    dx = math.radians(lon - ref_lon) * _R_EARTH * coslat    # 東向
    dy = math.radians(lat - ref_lat) * _R_EARTH             # 北向
    dz = alt - ref_alt                                       # 垂直
    return (dx, dy, dz)


def _enu_to_latlon(dx: float, dy: float, dz: float,
                   ref_lat: float, ref_lon: float,
                   ref_alt: float = 0.0) -> Tuple[float, float, float]:
    """ENU → 經緯度"""
    coslat = math.cos(math.radians(ref_lat))
    lat = ref_lat + math.degrees(dy / _R_EARTH)
    lon = ref_lon + math.degrees(dx / (_R_EARTH * max(coslat, 1e-9)))
    alt = ref_alt + dz
    return (lat, lon, alt)


def _vec3_norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def _vec3_normalize(v: Tuple[float, float, float],
                    eps: float = 1e-9) -> Tuple[float, float, float]:
    n = _vec3_norm(v)
    if n < eps:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _vec3_add(*vs: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (sum(v[0] for v in vs),
            sum(v[1] for v in vs),
            sum(v[2] for v in vs))


def _vec3_scale(v: Tuple[float, float, float],
                s: float) -> Tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


# ═══════════════════════════════════════════════════════════════════════
#  ReconToStrikeManager 核心控制類別
# ═══════════════════════════════════════════════════════════════════════

class ReconToStrikeManager:
    """
    偵察觸發 → 動態任務切換 → 蜂群打擊的總協調器

    使用流程::

        mgr = ReconToStrikeManager(
            uav_states={uav.uav_id: uav for uav in all_uavs},
            coalition_size=3,
        )
        # UAV FOV 偵測到目標時
        report = mgr.target_detected_callback(tgt_lat, tgt_lon)
        # 取得每架 UAV 的新任務序列
        for a in mgr.assignments.values():
            sitl_link.upload_mission(mgr.mission_to_tuples(a))

    Parameters
    ----------
    uav_states :
        所有 UAV 當前遙測狀態 (由 MAVLink 心跳更新)
    coalition_size :
        從覆蓋機群中挑選幾架轉打擊 (預設 3)
    base_strike_alt_m :
        打擊聯盟的最底層高度
    altitude_step_m :
        高度錯層間距
    turn_radius_m :
        固定翼最小轉彎半徑
    terminal_boundary_m :
        末端段起始邊界距離目標 (預設 2 km)
    cruise_speed_mps :
        Phase 2 巡航速度
    terminal_speed_mps :
        Phase 3 末端衝刺速度
    stall_speed_mps :
        失速速度下限
    iapf_min_safe_dist_m :
        IAPF 觸發距離 (UAV 間 3D 距離小於此值啟動斥力)
    iapf_tangent_weight :
        切線斥力佔比 (固定翼建議 0.7，徑向僅 0.3)
    iapf_k_att :
        引力增益
    iapf_k_rep :
        斥力增益
    transition_lookahead_m :
        過渡航點距當前 pose 的 lookahead 距離 (平滑銜接)
    """

    # ─────────────────────────────────────────────────────────────────
    #  初始化
    # ─────────────────────────────────────────────────────────────────
    def __init__(self,
                 uav_states: Dict[int, UAVState],
                 coalition_size: int = 3,
                 base_strike_alt_m: float = 500.0,
                 altitude_step_m: float = 30.0,
                 turn_radius_m: float = 150.0,
                 terminal_boundary_m: float = 2000.0,
                 cruise_speed_mps: float = 25.0,
                 terminal_speed_mps: float = 46.0,
                 stall_speed_mps: float = 18.0,
                 iapf_min_safe_dist_m: float = 150.0,
                 iapf_tangent_weight: float = 0.7,
                 iapf_k_att: float = 1.0,
                 iapf_k_rep: float = 2.0e5,
                 transition_lookahead_m: float = 800.0):

        if coalition_size < 1:
            raise ValueError('coalition_size 必須 ≥ 1')
        if not uav_states:
            raise ValueError('uav_states 為空')

        self.uav_states = dict(uav_states)
        self.coalition_size = coalition_size
        self.base_strike_alt_m = base_strike_alt_m
        self.altitude_step_m = altitude_step_m
        self.turn_radius_m = turn_radius_m
        self.terminal_boundary_m = terminal_boundary_m
        self.cruise_speed_mps = cruise_speed_mps
        self.terminal_speed_mps = terminal_speed_mps
        self.stall_speed_mps = stall_speed_mps

        # IAPF 參數
        self.iapf_min_safe_dist_m = iapf_min_safe_dist_m
        self.iapf_tangent_weight = iapf_tangent_weight
        self.iapf_k_att = iapf_k_att
        self.iapf_k_rep = iapf_k_rep
        self.transition_lookahead_m = transition_lookahead_m

        # 結果快取
        self._assignments: Dict[int, StrikeAssignment] = {}
        self._last_report: Optional[CoalitionReport] = None

    # ═════════════════════════════════════════════════════════════════
    #  1. 偵察觸發與任務重構
    # ═════════════════════════════════════════════════════════════════

    def target_detected_callback(self,
                                  target_lat: float,
                                  target_lon: float,
                                  target_alt: float = 0.0,
                                  ) -> CoalitionReport:
        """偵察到目標時觸發 — 完整任務重構流程。

        Steps
        -----
        1. 篩選出 N 架「最適合」UAV 組成打擊聯盟（距離 + 狀態）
        2. 分配 360° 均勻進場角
        3. 高度錯層 (宏觀避障)
        4. 生成平滑過渡航點 (飛控銜接保護)
        5. IAPF 微觀避障向量修正
        6. STOT 時間協同 + Loiter 圈數
        7. 組合 MAVLink 任務序列
        """
        logger.info(
            f'[ReconToStrike] 偵察觸發! 目標 ({target_lat:.6f}, {target_lon:.6f}), '
            f'當前 UAV 總數 = {len(self.uav_states)}'
        )

        # ── Step 1: 挑選聯盟 ───────────────────────────────
        coalition, rejected = self._select_coalition(
            target_lat, target_lon, target_alt
        )
        if not coalition:
            raise RuntimeError('無可用 UAV 組成打擊聯盟')

        # 將聯盟成員標記為 TRANSITIONING
        for uid in coalition:
            self.uav_states[uid].task_mode = TaskMode.TRANSITIONING

        # ── Step 2: 360° 全向包圍分配 ──────────────────────
        slot_assignments = self._assign_omnidirectional_slots(
            coalition, target_lat, target_lon
        )

        # ── Step 3: 高度錯層分配 + 初始化 StrikeAssignment ──
        assignments: List[StrikeAssignment] = []
        alt_layers: Dict[int, float] = {}
        for slot, (uid, heading) in enumerate(slot_assignments):
            alt_layer = self.base_strike_alt_m + slot * self.altitude_step_m
            alt_layers[uid] = alt_layer

            uav = self.uav_states[uid]
            a = StrikeAssignment(
                uav_id=uid,
                slot_index=slot,
                approach_heading_deg=heading,
                transition_alt_m=alt_layer,
                # 先用 placeholder；下一步計算過渡點
                transition_lat=uav.lat,
                transition_lon=uav.lon,
                transition_alt_target_m=alt_layer,
                loiter_radius_m=self.turn_radius_m,
                cruise_speed_mps=self.cruise_speed_mps,
                terminal_speed_mps=self.terminal_speed_mps,
            )
            # 計算 IP 與命中點 (2 km 邊界)
            a.ip_lat, a.ip_lon = _destination(
                target_lat, target_lon,
                (heading + 180.0) % 360.0, self.terminal_boundary_m,
            )
            a.impact_lat, a.impact_lon = target_lat, target_lon

            # Loiter 中心：IP 與當前位置連線中點偏向 UAV 側，確保高度爬升完成
            mid_lat = (uav.lat + a.ip_lat) / 2.0
            mid_lon = (uav.lon + a.ip_lon) / 2.0
            a.loiter_lat, a.loiter_lon = mid_lat, mid_lon

            assignments.append(a)

        # ── Step 4: 平滑過渡航點 (每架依當前航向生成) ────────
        for a in assignments:
            self._compute_transition_waypoint(a)

        # ── Step 5: IAPF 微觀避障 ─────────────────────────
        iapf_conflicts, iapf_adjustments = self._apply_iapf_avoidance(
            assignments, target_lat, target_lon
        )

        # ── Step 6: STOT 時間協同 + Loiter ────────────────
        tot_sec = self._coordinate_stot(assignments)

        # ── Step 7: 生成 MAVLink 任務序列 ────────────────
        for a in assignments:
            a.mission = self._build_strike_mission(a)
            self.uav_states[a.uav_id].task_mode = TaskMode.STRIKE

        self._assignments = {a.uav_id: a for a in assignments}
        self._last_report = CoalitionReport(
            target_lat=target_lat, target_lon=target_lon,
            selected_uav_ids=[a.uav_id for a in assignments],
            rejected_uav_ids=rejected,
            tot_sec=tot_sec,
            altitude_layers=alt_layers,
            iapf_conflicts=iapf_conflicts,
            iapf_adjustments=iapf_adjustments,
        )
        self._log_report()
        return self._last_report

    def _select_coalition(self, target_lat: float, target_lon: float,
                          target_alt: float) -> Tuple[List[int], List[int]]:
        """挑選 N 架最適合的 UAV 組成打擊聯盟。

        篩選標準（依優先序）：
          1. 任務狀態必須為 COVERAGE (排除已經在 STRIKE 或 IDLE 的)
          2. 對目標的 2D 距離 (越近越優先)
          3. (未來可加) 燃料/電量餘裕、偵搜角度

        Returns
        -------
        (selected, rejected) :
            selected — 入選的 uav_id (按距離升冪)
            rejected — 未入選的 uav_id
        """
        candidates: List[Tuple[int, float]] = []
        rejected: List[int] = []

        for uid, uav in self.uav_states.items():
            if uav.task_mode != TaskMode.COVERAGE:
                rejected.append(uid)
                continue
            d = _haversine(uav.lat, uav.lon, target_lat, target_lon)
            candidates.append((uid, d))

        candidates.sort(key=lambda x: x[1])
        n = min(self.coalition_size, len(candidates))
        selected = [uid for uid, _ in candidates[:n]]
        rejected.extend(uid for uid, _ in candidates[n:])

        logger.info(
            f'[Coalition] 從 {len(self.uav_states)} 架中篩選 {n} 架:'
            + ''.join(f'\n    {self.uav_states[uid].name} d={d:.0f}m'
                      for uid, d in candidates[:n])
        )
        return selected, rejected

    def _assign_omnidirectional_slots(self,
                                       coalition: List[int],
                                       target_lat: float,
                                       target_lon: float,
                                       ) -> List[Tuple[int, float]]:
        """將聯盟 UAV 分配至 360° 均勻進場角 (最小衝突旋轉對齊)"""
        n = len(coalition)
        if n == 1:
            uid = coalition[0]
            uav = self.uav_states[uid]
            brg = _bearing_deg(uav.lat, uav.lon, target_lat, target_lon)
            return [(uid, brg)]

        # 按 UAV 相對目標的方位角排序
        by_bearing = sorted(
            coalition,
            key=lambda uid: _bearing_deg(
                self.uav_states[uid].lat, self.uav_states[uid].lon,
                target_lat, target_lon,
            ),
        )
        vectors = sorted((360.0 / n * k) % 360.0 for k in range(n))
        best_shift, best_cost = 0, float('inf')
        for shift in range(n):
            cost = sum(
                _angular_diff(
                    _bearing_deg(self.uav_states[by_bearing[i]].lat,
                                 self.uav_states[by_bearing[i]].lon,
                                 target_lat, target_lon),
                    vectors[(i + shift) % n],
                )
                for i in range(n)
            )
            if cost < best_cost:
                best_cost, best_shift = cost, shift
        return [(by_bearing[i], vectors[(i + best_shift) % n])
                for i in range(n)]

    # ═════════════════════════════════════════════════════════════════
    #  4. 平滑過渡航點 (飛控銜接保護)
    # ═════════════════════════════════════════════════════════════════

    def _compute_transition_waypoint(self, a: StrikeAssignment) -> None:
        """在當前航向前方插入梯度高度的過渡點。

        關鍵設計：
          - 位置：當前 pose + lookahead·ψ_current → 飛控感受不到急轉
          - 高度：0.5·(當前 alt + 目標層高)    → 漸變避免急爬
        """
        uav = self.uav_states[a.uav_id]
        lookahead = self.transition_lookahead_m
        tlat, tlon = _destination(uav.lat, uav.lon, uav.heading_deg, lookahead)
        mid_alt = uav.alt + 0.5 * (a.transition_alt_m - uav.alt)

        a.transition_lat = tlat
        a.transition_lon = tlon
        a.transition_alt_target_m = mid_alt

    # ═════════════════════════════════════════════════════════════════
    #  3. 微觀避障：改進型人工勢場法 (3D IAPF, 固定翼特化)
    # ═════════════════════════════════════════════════════════════════

    def calculate_avoidance_vector(self,
                                    uav_id: int,
                                    other_uavs: List[UAVState],
                                    goal_lat: float,
                                    goal_lon: float,
                                    goal_alt: float,
                                    min_safe_dist: Optional[float] = None,
                                    ) -> Tuple[float, float, float]:
        """計算單架 UAV 的 3D IAPF 避障向量 (ENU, 公尺)。

        Parameters
        ----------
        uav_id :
            待計算的 UAV
        other_uavs :
            其他僚機當前狀態（用來算斥力）
        goal_lat, goal_lon, goal_alt :
            引力目標（通常是下一個 loiter 中心或 IP）
        min_safe_dist :
            斥力生效距離 (None 使用預設 iapf_min_safe_dist_m)

        Returns
        -------
        (dx, dy, dz) :
            以當前 UAV 位置為原點的 ENU 避障向量 (公尺)；
            將此向量乘上適當 lookahead 即可得到下一個導航點
        """
        if min_safe_dist is None:
            min_safe_dist = self.iapf_min_safe_dist_m

        me = self.uav_states[uav_id]
        # 以自己為 ENU 原點
        ref_lat, ref_lon = me.lat, me.lon
        ref_alt = me.alt

        # ── 引力：指向目標 ─────────────────────────────────
        gx, gy, gz = _latlon_to_enu(goal_lat, goal_lon, goal_alt,
                                     ref_lat, ref_lon, ref_alt)
        F_att = _vec3_scale(_vec3_normalize((gx, gy, gz)), self.iapf_k_att)

        # ── 斥力：來自所有其他 UAV ─────────────────────────
        F_rep_sum = (0.0, 0.0, 0.0)
        for other in other_uavs:
            if other.uav_id == uav_id:
                continue
            ox, oy, oz = _latlon_to_enu(other.lat, other.lon, other.alt,
                                         ref_lat, ref_lon, ref_alt)
            # 從僚機指向自己的向量 (我往外推的方向)
            rx, ry, rz = -ox, -oy, -oz
            d = _vec3_norm((rx, ry, rz))

            if d < 1e-3 or d > min_safe_dist:
                continue   # 完全重疊 (異常) 或太遠不觸發

            # ── 徑向斥力 (標準 APF 項) ─────────────────
            # scale ∝ (1/d - 1/d_safe) / d²
            scale = self.iapf_k_rep * (1.0 / d - 1.0 / min_safe_dist) / (d * d)
            radial = _vec3_scale(_vec3_normalize((rx, ry, rz)), scale)

            # ── 切線斥力 (IAPF 固定翼特化) ─────────────
            # 水平切線：將水平徑向分量逆時針旋轉 90° → 兩機都往「右」偏
            # (等同 ICAO 左右分離規則；避免乒乓震盪)
            # CW rotation 90°: (x, y) → (y, -x)
            tang_x = ry
            tang_y = -rx
            tang_z = rz * 0.3   # 垂直分量保留少量 (輕微爬/降以增加 3D 分離)
            tangential = _vec3_scale(
                _vec3_normalize((tang_x, tang_y, tang_z)),
                scale,
            )

            # 加權合成：切線為主，徑向為輔
            alpha = 1.0 - self.iapf_tangent_weight
            F_rep_this = _vec3_add(
                _vec3_scale(radial,     alpha),
                _vec3_scale(tangential, self.iapf_tangent_weight),
            )
            F_rep_sum = _vec3_add(F_rep_sum, F_rep_this)

        # ── 合成總勢場 ────────────────────────────────────
        return _vec3_add(F_att, F_rep_sum)

    def _apply_iapf_avoidance(self,
                               assignments: List[StrikeAssignment],
                               target_lat: float,
                               target_lon: float,
                               ) -> Tuple[List[str], List[str]]:
        """對所有聯盟成員檢查 3D 衝突 → 生成避障中繼點。

        每架 UAV 的引力目標 = 自己分配到的 loiter 中心；
        斥力來源 = 其他聯盟成員的當前位置。

        若避障向量偏離引力向量超過 15°，視為需要避障 → 插入中繼航點。
        """
        conflicts: List[str] = []
        adjustments: List[str] = []

        uav_states_list = [self.uav_states[a.uav_id] for a in assignments]

        for a in assignments:
            me = self.uav_states[a.uav_id]

            # 先檢查 3D 距離是否有違反
            for other in uav_states_list:
                if other.uav_id == a.uav_id:
                    continue
                d = math.sqrt(
                    _haversine(me.lat, me.lon, other.lat, other.lon) ** 2 +
                    (me.alt - other.alt) ** 2
                )
                if d < self.iapf_min_safe_dist_m:
                    conflicts.append(
                        f'{me.name} ↔ {other.name}: 3D 距離僅 {d:.0f}m '
                        f'< 安全距離 {self.iapf_min_safe_dist_m:.0f}m'
                    )

            # 計算避障向量 (引力目標 = 自己的 loiter 中心)
            F_total = self.calculate_avoidance_vector(
                a.uav_id, uav_states_list,
                a.loiter_lat, a.loiter_lon, a.transition_alt_m,
            )

            # 純引力向量 (只有引力，沒有斥力)
            gx, gy, gz = _latlon_to_enu(
                a.loiter_lat, a.loiter_lon, a.transition_alt_m,
                me.lat, me.lon, me.alt,
            )
            F_att = _vec3_normalize((gx, gy, gz))
            F_total_n = _vec3_normalize(F_total)

            # 計算偏移角度 (arccos of dot product)
            dot = (F_att[0] * F_total_n[0] +
                   F_att[1] * F_total_n[1] +
                   F_att[2] * F_total_n[2])
            dot = max(-1.0, min(1.0, dot))
            deviation_deg = math.degrees(math.acos(dot))

            if deviation_deg > 15.0:
                # 生成中繼避障航點：沿 F_total 方向延伸 lookahead 距離
                d_look = 600.0  # 600m 前方
                dx = F_total_n[0] * d_look
                dy = F_total_n[1] * d_look
                dz = F_total_n[2] * d_look * 0.5   # 高度變化取一半
                av_lat, av_lon, _ = _enu_to_latlon(
                    dx, dy, dz, me.lat, me.lon, me.alt,
                )
                a.avoidance_lat = av_lat
                a.avoidance_lon = av_lon
                adjustments.append(
                    f'{me.name}: 插入 IAPF 避障中繼點 '
                    f'(偏轉 {deviation_deg:.1f}°)'
                )

        return conflicts, adjustments

    # ═════════════════════════════════════════════════════════════════
    #  6. STOT 時間協同 + Loiter 圈數
    # ═════════════════════════════════════════════════════════════════

    def _coordinate_stot(self, assignments: List[StrikeAssignment]) -> float:
        """以最晚抵達 IP 的 UAV 時間為 Base_TOT，其餘補 Loiter 圈數。

        2026 重構：TOT 排程 + Loiter 計算委派給 core.strike.time_coordination；
        本方法僅保留「路徑長度估算」這個 recon 特有邏輯。
        """
        # ── Step 1: 估算每架 UAV 的 nominal 時間 (此為 recon 特有) ──
        t_nominal_list = []
        for a in assignments:
            uav = self.uav_states[a.uav_id]
            d1 = _haversine(uav.lat, uav.lon, a.transition_lat, a.transition_lon)
            climb = abs(a.transition_alt_m - uav.alt)
            d1_3d = math.hypot(d1, climb)

            if a.avoidance_lat is not None:
                d2 = _haversine(a.transition_lat, a.transition_lon,
                                a.avoidance_lat, a.avoidance_lon)
                d3 = _haversine(a.avoidance_lat, a.avoidance_lon,
                                a.loiter_lat, a.loiter_lon)
                d_mid = d2 + d3
            else:
                d_mid = _haversine(a.transition_lat, a.transition_lon,
                                   a.loiter_lat, a.loiter_lon)

            d4 = _haversine(a.loiter_lat, a.loiter_lon, a.ip_lat, a.ip_lon)
            t_nom = (d1_3d + d_mid + d4) / max(a.cruise_speed_mps, 1.0)
            t_nominal_list.append(t_nom)

        # ── Step 2+3: 委派給共用模組 (STOT 排程 + Loiter 圈數) ──
        from core.strike.time_coordination import coordinate as tc_coordinate
        base_tot, slots = tc_coordinate(
            nominal_times_sec=t_nominal_list,
            mode='STOT',
            cruise_speed_mps=max(
                assignments[0].cruise_speed_mps if assignments else 25.0, 1.0
            ),
            loiter_radius_m=self.turn_radius_m,
            payloads=assignments,
        )
        logger.info(f'[STOT] Base_TOT = {base_tot:.2f}s (所有 UAV 同秒抵達 IP)')

        # 回寫結果
        for slot in slots:
            a = slot.payload
            a.t_arrive_ip_sec = slot.arrival_time_sec
            a.loiter_turns = slot.loiter_turns
        return base_tot

    # ═════════════════════════════════════════════════════════════════
    #  7. MAVLink 任務序列生成 (QGC WPL 110)
    # ═════════════════════════════════════════════════════════════════

    def _build_strike_mission(self, a: StrikeAssignment) -> List[MissionItem]:
        """生成單架 UAV 的 MAVLink 任務序列 (平滑銜接 + 打擊)"""
        uav = self.uav_states[a.uav_id]
        mission: List[MissionItem] = []

        # seq 0: DO_SET_HOME (當前位置)
        mission.append(MissionItem(
            cmd=MAVCmd.DO_SET_HOME,
            lat=uav.lat, lon=uav.lon, alt=uav.alt,
            comment=f'Home @ 切換時刻的 pose (slot {a.slot_index})',
        ))

        # seq 1: DO_CHANGE_SPEED (Phase 2 巡航)
        mission.append(MissionItem(
            cmd=MAVCmd.DO_CHANGE_SPEED,
            param1=0.0, param2=float(a.cruise_speed_mps), param3=-1.0,
            comment=f'Phase 2 V={a.cruise_speed_mps:.1f} m/s',
        ))

        # seq 2: NAV_WAYPOINT 過渡航點 (關鍵平滑銜接)
        mission.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=a.transition_lat, lon=a.transition_lon,
            alt=a.transition_alt_target_m,
            param2=float(self.turn_radius_m * 2.0),    # 放寬 acceptance
            comment='Transition WP (當前航向 800m + 梯度高度)',
        ))

        # seq 3: [可選] IAPF 避障中繼航點
        if a.avoidance_lat is not None:
            mission.append(MissionItem(
                cmd=MAVCmd.NAV_WAYPOINT,
                lat=a.avoidance_lat, lon=a.avoidance_lon,
                alt=a.transition_alt_m,
                param2=float(self.turn_radius_m * 1.5),
                comment='IAPF 避障中繼點 (切線斥力修正)',
            ))

        # seq 4: NAV_WAYPOINT 盤旋中心 (爬升至分配高度層)
        mission.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=a.loiter_lat, lon=a.loiter_lon, alt=a.transition_alt_m,
            param2=float(self.turn_radius_m),
            comment=f'Loiter center @ alt {a.transition_alt_m:.0f}m',
        ))

        # seq 5: NAV_LOITER_TURNS (TOT 同步補時)
        if a.loiter_turns > 0.1:
            mission.append(MissionItem(
                cmd=MAVCmd.NAV_LOITER_TURNS,
                lat=a.loiter_lat, lon=a.loiter_lon, alt=a.transition_alt_m,
                param1=float(a.loiter_turns),           # 圈數
                param3=float(a.loiter_radius_m),
                param4=1.0,                              # 達高度方結束
                comment=f'TOT 同步: {a.loiter_turns:.1f} turns',
            ))

        # seq 6: NAV_WAYPOINT 2 km 邊界 IP
        mission.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=a.ip_lat, lon=a.ip_lon, alt=a.transition_alt_m,
            param2=float(self.turn_radius_m),
            comment=f'2km boundary IP (ψ={a.approach_heading_deg:.1f}°) '
                    f'@ t={a.t_arrive_ip_sec:.1f}s',
        ))

        # seq 7: DO_CHANGE_SPEED 末端衝刺
        mission.append(MissionItem(
            cmd=MAVCmd.DO_CHANGE_SPEED,
            param1=0.0, param2=float(a.terminal_speed_mps), param3=-1.0,
            comment=f'Phase 3 末端衝刺 V={a.terminal_speed_mps:.1f} m/s',
        ))

        # seq 8: NAV_WAYPOINT IMPACT
        mission.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=a.impact_lat, lon=a.impact_lon, alt=0.0,
            param2=max(10.0, self.turn_radius_m * 0.2),
            comment='IMPACT',
        ))
        return mission

    # ═════════════════════════════════════════════════════════════════
    #  Utilities
    # ═════════════════════════════════════════════════════════════════

    @property
    def assignments(self) -> Dict[int, StrikeAssignment]:
        return dict(self._assignments)

    @property
    def last_report(self) -> Optional[CoalitionReport]:
        return self._last_report

    def mission_to_tuples(self, a: StrikeAssignment
                          ) -> List[Tuple[float, ...]]:
        """將 StrikeAssignment.mission 轉為 SITLLink.upload_mission() 所需的 8-tuple"""
        return [
            (float(m.lat), float(m.lon), float(m.alt),
             int(m.cmd),
             float(m.param1), float(m.param2), float(m.param3), float(m.param4))
            for m in a.mission
        ]

    def export_qgc_wpl(self, export_dir: str) -> List[str]:
        """將每架 UAV 的 mission 匯出為 QGC WPL 110 檔案"""
        os.makedirs(export_dir, exist_ok=True)
        files: List[str] = []
        for a in self._assignments.values():
            uav = self.uav_states[a.uav_id]
            fname = (f'RECON2STRIKE_slot{a.slot_index:02d}_{uav.name}'
                     f'_psi{a.approach_heading_deg:03.0f}'
                     f'_alt{a.transition_alt_m:04.0f}'
                     f'_t{a.t_arrive_ip_sec:04.0f}s.waypoints')
            fpath = os.path.join(export_dir, fname)
            lines = ['QGC WPL 110']
            for seq, item in enumerate(a.mission):
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
        return files

    # ─── 日誌 ────────────────────────────────────────────
    def _log_report(self) -> None:
        r = self._last_report
        if r is None:
            return
        logger.info(f'[ReconToStrike] 任務重構完成 (TOT={r.tot_sec:.2f}s):')
        for a in self._assignments.values():
            uav = self.uav_states[a.uav_id]
            extra = ''
            if a.avoidance_lat is not None:
                extra = '  [+IAPF 避障中繼]'
            loi = f'  Loiter={a.loiter_turns:.1f}turns' if a.loiter_turns > 0.1 else ''
            logger.info(
                f'  slot {a.slot_index} {uav.name}  '
                f'psi={a.approach_heading_deg:5.1f}deg  '
                f'alt={a.transition_alt_m:.0f}m  '
                f'WPs={len(a.mission)}{loi}{extra}'
            )
        if r.iapf_conflicts:
            logger.warning('[IAPF] 檢測到衝突:')
            for c in r.iapf_conflicts:
                logger.warning(f'  ! {c}')
        if r.iapf_adjustments:
            logger.info('[IAPF] 避障調整:')
            for c in r.iapf_adjustments:
                logger.info(f'  + {c}')


# ═══════════════════════════════════════════════════════════════════════
#  Main Demo：模擬 5 架 DCCPP 掃描中 → 偵測目標 → 3 架轉打擊
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    # ── 情境：5 架 UAV 正在台北市西北方執行 DCCPP 覆蓋掃描 ──
    uav_list = [
        UAVState(uav_id=1, lat=25.12, lon=121.48, alt=300, heading_deg=90,
                 ground_speed_mps=25.0, name='UAV-Alpha'),
        UAVState(uav_id=2, lat=25.10, lon=121.52, alt=300, heading_deg=180,
                 ground_speed_mps=25.0, name='UAV-Bravo'),
        UAVState(uav_id=3, lat=25.08, lon=121.48, alt=300, heading_deg=0,
                 ground_speed_mps=25.0, name='UAV-Charlie'),
        UAVState(uav_id=4, lat=25.14, lon=121.56, alt=300, heading_deg=270,
                 ground_speed_mps=25.0, name='UAV-Delta'),
        UAVState(uav_id=5, lat=25.18, lon=121.62, alt=300, heading_deg=180,
                 ground_speed_mps=25.0, name='UAV-Echo'),
    ]
    # 全部初始為 COVERAGE 狀態 (DCCPP 掃描中)
    for u in uav_list:
        u.task_mode = TaskMode.COVERAGE

    uav_states = {u.uav_id: u for u in uav_list}

    # ── 建立 ReconToStrikeManager ─────────────────────────
    mgr = ReconToStrikeManager(
        uav_states=uav_states,
        coalition_size=3,               # 從 5 架中挑 3 架轉打擊
        base_strike_alt_m=500.0,
        altitude_step_m=30.0,
        turn_radius_m=150.0,
        terminal_boundary_m=2000.0,
        cruise_speed_mps=25.0,
        terminal_speed_mps=46.0,
        stall_speed_mps=18.0,
        iapf_min_safe_dist_m=1500.0,    # 1.5 km 內視為需避障 (聯盟匯聚階段)
        iapf_tangent_weight=0.7,        # 切線斥力佔 70% (固定翼)
        transition_lookahead_m=800.0,
    )

    print('\n' + '═' * 80)
    print(' 初始狀態：5 架 UAV 正在執行 DCCPP 覆蓋掃描')
    print('═' * 80)
    for u in uav_list:
        print(f'  {u.name}  pos=({u.lat:.4f}, {u.lon:.4f})  alt={u.alt}m  '
              f'hdg={u.heading_deg:.0f}°  state={u.task_mode.value}')

    # ── 模擬偵測事件：UAV-Bravo 的 FOV 鎖定到隱藏目標 ────
    print('\n' + '═' * 80)
    print(' 【偵察觸發】UAV-Bravo 捕捉到隱藏目標!')
    target_lat, target_lon = 25.0330, 121.5654   # 台北 101 附近
    print(f'   目標座標: ({target_lat}, {target_lon})')
    print('═' * 80)

    report = mgr.target_detected_callback(target_lat, target_lon, 0.0)

    # ── 顯示聯盟構成 ──────────────────────────────────────
    print('\n' + '═' * 80)
    print(' 動態聯盟構成 (Dynamic Coalition)')
    print('═' * 80)
    print(f'  入選 UAV     : '
          + ', '.join(uav_states[uid].name for uid in report.selected_uav_ids))
    print(f'  未入選 UAV   : '
          + ', '.join(uav_states[uid].name for uid in report.rejected_uav_ids))
    print(f'  基準 TOT     : {report.tot_sec:.2f} s')
    print(f'\n  高度層分配：')
    for uid, alt in report.altitude_layers.items():
        print(f'    {uav_states[uid].name:<12s} → {alt:.0f} m')

    # ── IAPF 避障報告 ───────────────────────────────────
    print('\n' + '═' * 80)
    print(' IAPF 避障檢查')
    print('═' * 80)
    if report.iapf_conflicts:
        print('  衝突偵測：')
        for c in report.iapf_conflicts:
            print(f'    ⚠ {c}')
    else:
        print('  無距離衝突 (所有 UAV 3D 距離 > 安全限)')
    if report.iapf_adjustments:
        print('  避障調整：')
        for a in report.iapf_adjustments:
            print(f'    + {a}')
    else:
        print('  無需插入避障中繼點 (引力/斥力合成後偏轉 < 15°)')

    # ── 打擊航線摘要 ─────────────────────────────────────
    print('\n' + '═' * 80)
    print(' 蜂群打擊航線摘要 (每架 UAV 的 MAVLink 序列)')
    print('═' * 80)
    print(f'{"slot":>4s} {"UAV":<12s} {"psi(°)":>7s} {"alt(m)":>6s} '
          f'{"Loiter(turns)":>14s} {"WP 數":>6s} {"避障":<10s}')
    print('-' * 80)
    for uid in report.selected_uav_ids:
        a = mgr.assignments[uid]
        name = uav_states[uid].name
        ad_flag = 'IAPF 中繼' if a.avoidance_lat is not None else '-'
        print(f'{a.slot_index:>4d} {name:<12s} '
              f'{a.approach_heading_deg:>7.1f} '
              f'{a.transition_alt_m:>6.0f} '
              f'{a.loiter_turns:>14.2f} '
              f'{len(a.mission):>6d} '
              f'{ad_flag:<10s}')

    # ── slot 0 的完整 MAVLink 任務序列 ────────────────────
    print('\n' + '═' * 80)
    print(' slot 0 (最先抵達) 的 MAVLink 任務序列')
    print('═' * 80)
    cmd_name = {179: 'DO_SET_HOME', 178: 'DO_CHANGE_SPEED',
                16: 'NAV_WAYPOINT', 18: 'NAV_LOITER_TURNS'}
    slot0 = [a for a in mgr.assignments.values() if a.slot_index == 0][0]
    for i, item in enumerate(slot0.mission):
        name = cmd_name.get(item.cmd, f'CMD_{item.cmd}')
        print(f'  seq {i}  {name:<20s} '
              f'lat={item.lat:10.6f} lon={item.lon:11.6f} alt={item.alt:7.1f}  '
              f'p1={item.param1:6.2f} p2={item.param2:6.2f}')
        print(f'          | {item.comment}')

    # ── 匯出 QGC WPL 110 ────────────────────────────────
    out_dir = os.path.join(os.getcwd(), 'recon2strike_out')
    files = mgr.export_qgc_wpl(out_dir)
    print(f'\n[匯出] {len(files)} 份 .waypoints → {out_dir}')
    for f in files:
        print(f'  • {os.path.basename(f)}')

    # ── 未被選中的 UAV 仍在執行 DCCPP ────────────────────
    print('\n[任務持續性]')
    for u in uav_list:
        print(f'  {u.name:<12s}  state → {u.task_mode.value}')
