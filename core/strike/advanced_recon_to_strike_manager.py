"""
AdvancedReconToStrikeManager — 高階偵打一體管理器
==================================================

對標已有的 :mod:`core.strike.recon_to_strike_manager`，本模組在四大方面升級：

    1. **Leader/Wingman 角色分派 + Dubins 平滑過渡**
       - 依能力 (距離、速度、高度) 選出臨時長機
       - 透過 Dubins 曲線生成 MAV_CMD_DO_REPOSITION + NAV_WAYPOINT 組合
       - 避免飛控收到突兀指令失速

    2. **改進型 3D IAPF (Ge & Cui 2000 衰減型斥力)**
       - 斥力項加入 ρ_goal^n 因子，UAV 靠近目標時斥力自動消失 → 無 GNRON
       - 附加指數衰減封套 η(ρ_goal)，強化近目標的可達性
       - 局部死鎖偵測 → 生成側向虛擬目標 (ICAO 右手法則) 逃逸
       - 斥力輸出夾限至固定翼最大偏航角速度 ψ̇_max = V/R_min

    3. **STOT/DTOT Time-To-Go 精算 + 專屬高度層盤旋**
       - 每機計算 T_to_go = L/V_cruise
       - wait_i = T_slot_i − T_to_go_i
       - loiter_turns_i = wait_i × V / (2π · R)
       - NAV_LOITER_TIME / NAV_LOITER_TURNS 於對應高度層盤旋

    4. **通訊延遲補償 (Delay-Aware State Estimation)**
       - 鄰機狀態外推：p_now = p_rx + v_rx · Δt
       - 不確定性半徑 σ = V · Δt · α (α≈0.3)
       - 有效安全距離 ρ_safe_eff = ρ_safe + σ

MAVLink 指令對照
----------------
    MAV_CMD_DO_REPOSITION     (192)  — 長機即時重新定位 (GUIDED 模式)
    MAV_CMD_DO_CHANGE_SPEED   (178)
    MAV_CMD_NAV_WAYPOINT       (16)
    MAV_CMD_NAV_LOITER_TIME    (19)
    MAV_CMD_NAV_LOITER_TURNS   (18)
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

from core.strike.swarm_strike_planner import (
    MissionItem, dubins_shortest_length,
    _haversine, _bearing_deg, _destination, _angular_diff,
    _R_EARTH,
)
from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  列舉與常數
# ═══════════════════════════════════════════════════════════════════════

class CoalitionRole(Enum):
    """聯盟角色"""
    LEADER  = 'LEADER'
    WINGMAN = 'WINGMAN'
    IDLE    = 'IDLE'


class TimingMode(Enum):
    """時間協同模式"""
    STOT = 'STOT'   # Simultaneous TOT (同秒命中)
    DTOT = 'DTOT'   # Distributed TOT (間隔命中)


class MAVCmd:
    NAV_WAYPOINT        = 16
    NAV_LOITER_TURNS    = 18
    NAV_LOITER_TIME     = 19
    DO_CHANGE_SPEED     = 178
    DO_SET_HOME         = 179
    DO_REPOSITION       = 192


# MAVLink 座標系
_MAV_FRAME_REL = 3

# 物理常數
_G = 9.80665  # 重力加速度 m/s²


# ═══════════════════════════════════════════════════════════════════════
#  資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class UAVRuntimeState:
    """UAV 即時遙測狀態 (由 MAVLink 心跳更新)

    Attributes
    ----------
    vx_mps, vy_mps, vz_mps :
        速度向量 (ENU, east / north / up)；延遲補償會用到
    last_update_time :
        此狀態的接收時間戳 (Unix sec)；計算 delay 用
    """
    uav_id: int
    lat: float
    lon: float
    alt: float
    heading_deg: float
    ground_speed_mps: float
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    last_update_time: float = 0.0
    role: CoalitionRole = CoalitionRole.IDLE
    name: str = ''

    # 固定翼規格 (可由上層覆寫)
    v_stall_mps: float = 18.0
    v_max_mps:   float = 55.0
    max_bank_deg: float = 35.0      # 最大側傾角 → 用於 R_min 計算
    max_climb_deg: float = 8.0       # 最大爬升 / 下降角

    def __post_init__(self) -> None:
        if not self.name:
            self.name = f'UAV-{self.uav_id}'
        if self.last_update_time <= 0:
            self.last_update_time = time.time()

    # ─── 運動學限制 ────────────────────────────────────────
    def r_min_m(self, V: Optional[float] = None) -> float:
        """最小轉彎半徑 R_min = V² / (g · tan(φ_max))"""
        v = V if V is not None else max(self.ground_speed_mps, self.v_stall_mps)
        return v * v / (_G * math.tan(math.radians(self.max_bank_deg)))

    def max_yaw_rate_rad_s(self, V: Optional[float] = None) -> float:
        """最大偏航角速度 ψ̇_max = V / R_min"""
        v = V if V is not None else max(self.ground_speed_mps, self.v_stall_mps)
        return v / max(self.r_min_m(v), 1.0)


@dataclass
class TargetState:
    """打擊目標狀態 (含移動目標預留欄位)"""
    lat: float
    lon: float
    alt: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    name: str = 'TGT'
    detected_at: float = 0.0


@dataclass
class IAPFDebugRecord:
    """單次 IAPF 計算的除錯資訊 (ENU 向量)"""
    uav_id: int
    f_att: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    f_rep: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    f_total: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    at_local_minimum: bool = False
    virtual_target_used: bool = False
    yaw_rate_clamped: bool = False
    neighbor_inflated_dist: Dict[int, float] = field(default_factory=dict)


@dataclass
class CoalitionAssignment:
    """單架 UAV 於打擊聯盟中的指派結果"""
    uav_id: int
    role: CoalitionRole
    slot_index: int
    name: str

    # 空間規劃
    attack_heading_deg: float
    altitude_layer_m: float
    transition_lat: float
    transition_lon: float
    transition_alt_m: float
    ip_lat: float                   # 2 km 邊界 IP
    ip_lon: float
    impact_lat: float
    impact_lon: float

    # 時間協同
    path_length_m: float
    t_to_go_sec: float
    t_arrival_sec: float             # slot 命中時刻
    wait_time_sec: float = 0.0
    loiter_turns: float  = 0.0
    loiter_lat: float    = 0.0
    loiter_lon: float    = 0.0
    loiter_radius_m: float = 150.0
    cruise_speed_mps: float = 25.0

    # MAVLink 序列
    mission: List[MissionItem] = field(default_factory=list)

    # IAPF 除錯
    iapf_debug: Optional[IAPFDebugRecord] = None


@dataclass
class StrikeResult:
    """一次完整重構的輸出"""
    assignments: List[CoalitionAssignment]
    base_tot_sec: float
    timing_mode: TimingMode
    leader_uav_id: int
    warnings: List[str] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════════════
#  地理 / 向量工具 — 統一匯入自 core.strike.geometry (2026 重構)
# ═══════════════════════════════════════════════════════════════════════
from core.strike.geometry import (
    latlon_to_enu as _ll_to_enu,
    enu_to_latlon as _enu_to_ll,
    v3_norm as _v3_norm,
    v3_normalize as _v3_normalize,
    v3_add as _v3_add,
    v3_scale as _v3_scale,
)


# ═══════════════════════════════════════════════════════════════════════
#  核心類別：AdvancedReconToStrikeManager
# ═══════════════════════════════════════════════════════════════════════

class AdvancedReconToStrikeManager:
    """高階偵打一體管理器 — 含改進型 IAPF + 延遲補償 + Leader/Wingman

    核心設計：將四大模組解耦，每個方法可獨立測試。

    Usage
    -----
    ::
        mgr = AdvancedReconToStrikeManager(
            uav_states={u.uav_id: u for u in uavs},
            coalition_size=3,
            timing_mode=TimingMode.STOT,
            communication_delay_sec=0.5,    # 模擬 500ms 通訊延遲
        )
        result = mgr.on_target_detected(target)
        for a in result.assignments:
            sitl.upload(mgr.mission_to_tuples(a))

    Parameters
    ----------
    uav_states :
        所有 UAV 即時遙測 (含 v_enu + last_update_time)
    coalition_size :
        轉打擊的 UAV 數
    timing_mode :
        STOT 或 DTOT
    interval_sec :
        DTOT 模式的 slot 間隔秒數 (STOT 忽略)
    base_strike_alt_m / altitude_step_m :
        高度錯層參數
    ip_distance_m :
        2 km 邊界 (末端起始) 距目標距離
    cruise_speed_mps / stall_speed_mps / max_speed_mps :
        速度包絡
    turn_radius_m :
        最小轉彎半徑 (僅作為 Loiter 半徑；IAPF 實際使用各機即時 R_min)
    transition_lookahead_m :
        平滑過渡航點距當前 pose 距離

    ─── IAPF 參數 ─────────────────────────────────────
    iapf_k_att :
        引力增益
    iapf_k_rep :
        斥力增益
    iapf_safe_dist_m :
        斥力觸發距離 (ρ₀)
    iapf_n :
        改進型斥力中的 ρ_goal 次方 (Ge & Cui 建議 n = 2)
    iapf_lambda :
        指數衰減封套強度 (0 = 關閉衰減)
    iapf_deadlock_threshold :
        局部極小偵測閾值 (|F_total| < 此值視為死鎖)
    iapf_virtual_target_dist_m :
        虛擬目標的側向距離

    ─── 延遲補償 ──────────────────────────────────────
    communication_delay_sec :
        假定的通訊延遲上界 (秒)
    position_uncertainty_alpha :
        不確定性半徑係數 σ = V · Δt · α
    """

    def __init__(self,
                 uav_states: Dict[int, UAVRuntimeState],
                 coalition_size: int = 3,
                 timing_mode: TimingMode = TimingMode.STOT,
                 interval_sec: float = 0.0,
                 base_strike_alt_m: float = 500.0,
                 altitude_step_m: float = 30.0,
                 ip_distance_m: float = 2000.0,
                 cruise_speed_mps: float = 25.0,
                 stall_speed_mps: float = 18.0,
                 max_speed_mps: float = 55.0,
                 turn_radius_m: float = 150.0,
                 transition_lookahead_m: float = 800.0,
                 # IAPF
                 iapf_k_att: float = 1.0,
                 iapf_k_rep: float = 5.0e4,
                 iapf_safe_dist_m: float = 150.0,
                 iapf_n: int = 2,
                 iapf_lambda: float = 1.5,
                 iapf_deadlock_threshold: float = 0.05,
                 iapf_virtual_target_dist_m: float = 400.0,
                 # Delay
                 communication_delay_sec: float = 0.3,
                 position_uncertainty_alpha: float = 0.3):

        if not uav_states:
            raise ValueError('uav_states 為空')
        if coalition_size < 1:
            raise ValueError('coalition_size 必須 ≥ 1')

        self.uav_states = dict(uav_states)
        self.coalition_size = coalition_size
        self.timing_mode = timing_mode
        self.interval_sec = float(interval_sec)

        # 空間 / 高度
        self.base_strike_alt_m = base_strike_alt_m
        self.altitude_step_m   = altitude_step_m
        self.ip_distance_m     = ip_distance_m
        self.transition_lookahead_m = transition_lookahead_m

        # 速度
        self.cruise_speed_mps = cruise_speed_mps
        self.stall_speed_mps  = stall_speed_mps
        self.max_speed_mps    = max_speed_mps
        self.turn_radius_m    = turn_radius_m

        # IAPF
        self.iapf_k_att   = iapf_k_att
        self.iapf_k_rep   = iapf_k_rep
        self.iapf_safe_dist_m = iapf_safe_dist_m
        self.iapf_n       = iapf_n
        self.iapf_lambda  = iapf_lambda
        self.iapf_deadlock_threshold = iapf_deadlock_threshold
        self.iapf_virtual_target_dist_m = iapf_virtual_target_dist_m

        # Delay
        self.communication_delay_sec = communication_delay_sec
        self.position_uncertainty_alpha = position_uncertainty_alpha

        # 結果快取
        self._last_result: Optional[StrikeResult] = None

    # ═════════════════════════════════════════════════════════════════
    #  主入口
    # ═════════════════════════════════════════════════════════════════

    def on_target_detected(self, target: TargetState) -> StrikeResult:
        """偵察觸發事件 — 完整任務重構流程

        Steps
        -----
        1. 延遲補償所有 UAV 狀態
        2. 挑聯盟 + 分配 Leader / Wingman
        3. 360° 全向進場向量分配
        4. 高度錯層
        5. 平滑過渡航點 (Dubins)
        6. 改進型 3D IAPF 避障檢查
        7. Time-To-Go + STOT/DTOT 盤旋計算
        8. MAVLink 任務序列
        """
        logger.info(
            f'[AdvRecon] 偵測目標 {target.name} @ '
            f'({target.lat:.6f}, {target.lon:.6f})'
        )
        warnings: List[str] = []

        # ── Step 1: 延遲補償 (把過期狀態外推到 t_now) ───────────
        compensated = self._compensate_all_states()

        # ── Step 2: Leader / Wingmen 選擇 ──────────────────────
        coalition_ids, leader_id, rejected_ids = self._select_coalition(
            compensated, target
        )
        if not coalition_ids:
            raise RuntimeError('聯盟組建失敗：無合適 UAV')

        # ── Step 3: 360° 進場向量分配 (Leader = slot 0) ────────
        slot_map = self._assign_slots(coalition_ids, leader_id, target,
                                       compensated)

        # ── Step 3.5: 批次預熱 Dubins cache (2026 重構：item 13) ─
        # 當 N 較大時，先一次性計算所有 UAV→IP 的 Dubins 路徑，
        # 讓後續 _build_assignment 全部命中 LRU cache (無論序列或平行皆受益)。
        self._prewarm_dubins_cache(slot_map, compensated, target)

        # ── Step 4 + 5: 高度層 + 平滑過渡 + 基礎指派 ───────────
        assignments: List[CoalitionAssignment] = []
        for slot_idx, uid in enumerate(slot_map):
            a = self._build_assignment(
                uav_state=compensated[uid],
                slot_index=slot_idx,
                leader_id=leader_id,
                target=target,
                attack_heading=self._slot_heading_deg(slot_idx, len(slot_map)),
            )
            assignments.append(a)

        # ── Step 6: IAPF 避障 (含延遲膨脹 + 死鎖逃逸) ─────────
        self._run_iapf_avoidance(assignments, compensated, target)

        # ── Step 7: Time-To-Go + STOT/DTOT 盤旋 ────────────────
        base_tot = self._coordinate_time(assignments, target, warnings)

        # ── Step 8: 生成 MAVLink 任務序列 ──────────────────────
        for a in assignments:
            a.mission = self._build_mission(a, is_leader=(a.role == CoalitionRole.LEADER))

        # 更新狀態
        for uid in coalition_ids:
            self.uav_states[uid].role = (CoalitionRole.LEADER
                                          if uid == leader_id
                                          else CoalitionRole.WINGMAN)

        result = StrikeResult(
            assignments=assignments,
            base_tot_sec=base_tot,
            timing_mode=self.timing_mode,
            leader_uav_id=leader_id,
            warnings=warnings,
        )
        self._last_result = result
        self._log_summary(result)
        return result

    # ═════════════════════════════════════════════════════════════════
    #  模組 4：延遲感知 — 鄰機狀態外推
    # ═════════════════════════════════════════════════════════════════

    def _compensate_all_states(self) -> Dict[int, UAVRuntimeState]:
        """為所有 UAV 狀態做延遲補償 → 回傳 t_now 的估算狀態。

        公式：
            Δt        = t_now − uav.last_update_time
            p_now     = p_rx + v_rx · Δt              (位置外推)
            σ_pos     = V · Δt · α                     (不確定性半徑)
            儲存至 state.last_update_time = t_now       (下次以此為基準)
        """
        now = time.time()
        compensated: Dict[int, UAVRuntimeState] = {}
        for uid, u in self.uav_states.items():
            dt = max(0.0, min(now - u.last_update_time,
                              self.communication_delay_sec * 2))
            # 淺複製 + 應用外推
            new = UAVRuntimeState(**u.__dict__)
            # ENU 速度 → 經緯度增量
            coslat = math.cos(math.radians(u.lat))
            dlat = math.degrees((u.vy_mps * dt) / _R_EARTH)
            dlon = math.degrees((u.vx_mps * dt) / (_R_EARTH * max(coslat, 1e-9)))
            new.lat = u.lat + dlat
            new.lon = u.lon + dlon
            new.alt = u.alt + u.vz_mps * dt
            new.last_update_time = now
            compensated[uid] = new

            if dt > 0.05:
                logger.debug(
                    f'[Delay] {u.name} 外推 Δt={dt*1000:.0f}ms → '
                    f'偏移 {_v3_norm((u.vx_mps*dt, u.vy_mps*dt, u.vz_mps*dt)):.1f}m'
                )
        return compensated

    def _effective_safe_distance(self, other: UAVRuntimeState) -> float:
        """有效安全距離 = ρ_safe + σ_pos (對鄰機的不確定性膨脹)"""
        V = max(_v3_norm((other.vx_mps, other.vy_mps, other.vz_mps)),
                self.stall_speed_mps)
        sigma = V * self.communication_delay_sec * self.position_uncertainty_alpha
        return self.iapf_safe_dist_m + sigma

    # ═════════════════════════════════════════════════════════════════
    #  模組 1：聯盟構建與角色分派
    # ═════════════════════════════════════════════════════════════════

    def _select_coalition(self,
                          states: Dict[int, UAVRuntimeState],
                          target: TargetState,
                          ) -> Tuple[List[int], int, List[int]]:
        """挑選聯盟成員 + 指派 Leader

        篩選策略：
          1. 僅接受 COVERAGE/IDLE 狀態的 UAV
          2. 依「距離目標」升冪排序
          3. 取前 N 架作為聯盟
          4. **Leader** = 聯盟中距離目標最短且當前速度最貼近 cruise 的
             (避免派正在轉彎中的 UAV 當長機)
        """
        candidates: List[Tuple[int, float, float]] = []
        rejected: List[int] = []

        for uid, u in states.items():
            if u.role not in (CoalitionRole.IDLE, CoalitionRole.WINGMAN):
                # LEADER 本人若在其他聯盟也排除
                rejected.append(uid)
                continue
            d = _haversine(u.lat, u.lon, target.lat, target.lon)
            # 速度穩定性指標 (越接近 cruise 越好)
            v_stability = abs(u.ground_speed_mps - self.cruise_speed_mps)
            candidates.append((uid, d, v_stability))

        candidates.sort(key=lambda x: x[1])
        n = min(self.coalition_size, len(candidates))
        coalition = [c[0] for c in candidates[:n]]
        rejected.extend(c[0] for c in candidates[n:])

        if not coalition:
            return [], -1, rejected

        # Leader: 綜合評分 (距離 0.7 + 速度穩定性 0.3，越低越好)
        scored = [
            (uid, 0.7 * d / 10000.0 + 0.3 * v_stab / 10.0)
            for uid, d, v_stab in candidates[:n]
        ]
        leader_id = min(scored, key=lambda x: x[1])[0]

        logger.info(
            f'[Coalition] 聯盟 ({n} 架): '
            + ', '.join(states[uid].name for uid in coalition)
            + f'  | Leader = {states[leader_id].name}'
        )
        return coalition, leader_id, rejected

    def _assign_slots(self,
                      coalition_ids: List[int],
                      leader_id: int,
                      target: TargetState,
                      states: Dict[int, UAVRuntimeState]
                      ) -> List[int]:
        """將聯盟分配到 slot 0/1/2..., Leader 永遠 = slot 0

        其餘僚機按相對目標的方位角升冪排入 slot 1, 2, ...
        """
        wingmen = [uid for uid in coalition_ids if uid != leader_id]
        wingmen.sort(key=lambda uid: _bearing_deg(
            states[uid].lat, states[uid].lon, target.lat, target.lon
        ))
        return [leader_id] + wingmen

    def _slot_heading_deg(self, slot_idx: int, n_total: int) -> float:
        """slot k 的 360° 進場方位角"""
        return (360.0 / n_total * slot_idx) % 360.0

    # ═════════════════════════════════════════════════════════════════
    #  模組 1 (續)：Leader Dubins 平滑過渡
    # ═════════════════════════════════════════════════════════════════

    def _prewarm_dubins_cache(self,
                                slot_map: List[int],
                                states: Dict[int, UAVRuntimeState],
                                target: TargetState,
                                ) -> None:
        """批次預熱 Dubins LRU cache (item 13: concurrent Dubins 實際呼叫點)

        設計理念：
          - 聯盟組建後，每架 UAV 會各算一次 Dubins (UAV → IP)
          - 單獨呼叫會 miss cache；批次預先算完 → 後續逐機呼叫 100% hit
          - 當 N > 5 時自動用 `dubins_batch_parallel` (雖然多數情況序列更快，
            但 API 已 exercise，未來若規模成長可直接受益)
        """
        from core.strike.geometry import dubins_batch, dubins_batch_parallel

        n = len(slot_map)
        if n <= 0:
            return

        pairs = []
        for slot_idx, uid in enumerate(slot_map):
            u = states[uid]
            heading = self._slot_heading_deg(slot_idx, n)
            ip_lat, ip_lon = _destination(
                target.lat, target.lon,
                (heading + 180.0) % 360.0,
                self.ip_distance_m,
            )
            pairs.append((
                u.lat, u.lon, u.heading_deg,
                ip_lat, ip_lon, heading,
                self.turn_radius_m,
            ))

        # 實際使用 batch API (≥5 架才嘗試平行，其他走序列，讓 cache 填滿)
        if n >= 5:
            _ = dubins_batch_parallel(pairs, min_threshold=5000)
        else:
            _ = dubins_batch(pairs)

        logger.debug(
            f'[AdvRecon] Dubins cache 預熱 {n} 組 UAV→IP pair'
        )

    def _build_assignment(self,
                           uav_state: UAVRuntimeState,
                           slot_index: int,
                           leader_id: int,
                           target: TargetState,
                           attack_heading: float,
                           ) -> CoalitionAssignment:
        """構建單架 UAV 的基礎 Assignment (空間 + 時間估算)"""
        role = (CoalitionRole.LEADER if uav_state.uav_id == leader_id
                else CoalitionRole.WINGMAN)

        # 高度層：Leader 500m, Wingmen 530/560/590...
        alt_layer = self.base_strike_alt_m + slot_index * self.altitude_step_m

        # 2 km 邊界 IP
        ip_lat, ip_lon = _destination(
            target.lat, target.lon,
            (attack_heading + 180.0) % 360.0,
            self.ip_distance_m,
        )

        # 平滑過渡航點 (當前航向 + 800m + 梯度高度)
        t_lat, t_lon = _destination(
            uav_state.lat, uav_state.lon,
            uav_state.heading_deg, self.transition_lookahead_m,
        )
        t_alt = uav_state.alt + 0.5 * (alt_layer - uav_state.alt)

        # 計算 Dubins 路徑長度 (起點 → IP)，用於 T_to_go
        L_dubins = dubins_shortest_length(
            uav_state.lat, uav_state.lon, uav_state.heading_deg,
            ip_lat, ip_lon, attack_heading,
            self.turn_radius_m,
        )
        climb = max(alt_layer - uav_state.alt, 0.0)
        L_3d = math.hypot(L_dubins, climb)
        t_to_go = L_3d / max(self.cruise_speed_mps, 0.1)

        # Loiter 中心 = 過渡航點與 IP 的中點 (Leader) 或僚機自己上空 (Wingman)
        if role == CoalitionRole.LEADER:
            loi_lat = (t_lat + ip_lat) / 2.0
            loi_lon = (t_lon + ip_lon) / 2.0
        else:
            loi_lat = t_lat
            loi_lon = t_lon

        return CoalitionAssignment(
            uav_id=uav_state.uav_id,
            role=role,
            slot_index=slot_index,
            name=uav_state.name,
            attack_heading_deg=attack_heading,
            altitude_layer_m=alt_layer,
            transition_lat=t_lat,
            transition_lon=t_lon,
            transition_alt_m=t_alt,
            ip_lat=ip_lat, ip_lon=ip_lon,
            impact_lat=target.lat, impact_lon=target.lon,
            path_length_m=L_3d,
            t_to_go_sec=t_to_go,
            t_arrival_sec=0.0,                # 由 _coordinate_time 填
            loiter_lat=loi_lat, loiter_lon=loi_lon,
            loiter_radius_m=self.turn_radius_m,
            cruise_speed_mps=self.cruise_speed_mps,
        )

    # ═════════════════════════════════════════════════════════════════
    #  模組 2：改進型 3D IAPF (含虛擬目標 + 運動學限制)
    # ═════════════════════════════════════════════════════════════════

    def compute_iapf_force(self,
                           me: UAVRuntimeState,
                           others: List[UAVRuntimeState],
                           goal_lat: float, goal_lon: float, goal_alt: float,
                           ) -> IAPFDebugRecord:
        """為單架 UAV 計算總勢場 (引力 + 改進斥力 + 運動學夾限)

        依 Ge & Cui 2000 改進公式：
            F_rep_1 = k_rep · (1/ρ − 1/ρ₀) · (ρ_goal^n / ρ²) · r̂
            F_rep_2 = (n/2) · k_rep · (1/ρ − 1/ρ₀)² · ρ_goal^(n−1) · ĝ
        再乘指數衰減封套 η(ρ_goal) = exp(−λ · ρ₀ / max(ρ_goal, ε))
        """
        ref_lat, ref_lon, ref_alt = me.lat, me.lon, me.alt
        # 以自己為 ENU 原點
        gx, gy, gz = _ll_to_enu(goal_lat, goal_lon, goal_alt,
                                 ref_lat, ref_lon, ref_alt)
        rho_goal = _v3_norm((gx, gy, gz))
        goal_dir = _v3_normalize((gx, gy, gz))

        # ── 引力 (指向目標) ──────────────────────────────
        F_att = _v3_scale(goal_dir, self.iapf_k_att * min(rho_goal, 1000.0))

        # ── 斥力 (對每個鄰機累加) ────────────────────────
        F_rep_total = (0.0, 0.0, 0.0)
        inflated: Dict[int, float] = {}

        for o in others:
            if o.uav_id == me.uav_id:
                continue
            ox, oy, oz = _ll_to_enu(o.lat, o.lon, o.alt,
                                     ref_lat, ref_lon, ref_alt)
            # 從鄰機指向自己的向量 r (推離方向)
            rx, ry, rz = -ox, -oy, -oz
            rho = _v3_norm((rx, ry, rz))

            # 延遲感知：有效安全距離
            rho0_eff = self._effective_safe_distance(o)
            inflated[o.uav_id] = rho0_eff

            if rho < 1e-3 or rho > rho0_eff:
                continue  # 太遠不觸發 / 完全重疊 (異常)

            # Ge & Cui 2000 改進版
            inv_diff = (1.0 / rho) - (1.0 / rho0_eff)
            rho_goal_safe = max(rho_goal, 1.0)
            n = self.iapf_n

            # F_rep_1: 徑向推離，但被 ρ_goal^n 壓制 (近目標時趨零)
            scale1 = (self.iapf_k_rep * inv_diff
                      * (rho_goal_safe ** n) / (rho * rho))
            F_rep_1 = _v3_scale(_v3_normalize((rx, ry, rz)), scale1)

            # F_rep_2: 把 UAV 拉向目標 (抵消近目標時的 GNRON)
            scale2 = (n * 0.5 * self.iapf_k_rep * (inv_diff ** 2)
                      * (rho_goal_safe ** (n - 1)))
            F_rep_2 = _v3_scale(goal_dir, scale2)

            # 指數衰減封套
            if self.iapf_lambda > 0:
                eta = math.exp(-self.iapf_lambda * rho0_eff / max(rho_goal, 1e-3))
            else:
                eta = 1.0

            F_rep_i = _v3_scale(_v3_add(F_rep_1, F_rep_2), eta)
            F_rep_total = _v3_add(F_rep_total, F_rep_i)

        # ── 死鎖偵測 + 虛擬目標逃逸 ───────────────────────
        F_total = _v3_add(F_att, F_rep_total)
        mag_total = _v3_norm(F_total)

        at_local_min = (mag_total < self.iapf_deadlock_threshold
                        and rho_goal > self.iapf_safe_dist_m * 2)
        virt_used = False

        if at_local_min:
            # 生成側向虛擬目標 (右手法則)
            psi = math.radians(me.heading_deg)
            # 航向右側 90°
            side_x = math.cos(psi - math.pi / 2) * self.iapf_virtual_target_dist_m
            side_y = math.sin(psi - math.pi / 2) * self.iapf_virtual_target_dist_m
            vdir = _v3_normalize((side_x, side_y, 0.0))
            F_virt = _v3_scale(vdir, self.iapf_k_att * 2.0)
            F_total = _v3_add(F_total, F_virt)
            virt_used = True
            logger.info(f'[IAPF] {me.name} 偵測局部極小 → 插入虛擬目標逃逸')

        # ── 固定翼運動學夾限 ─────────────────────────────
        F_total_clamped, was_clamped = self._apply_kinematic_limits(
            me, F_total,
        )

        return IAPFDebugRecord(
            uav_id=me.uav_id,
            f_att=F_att,
            f_rep=F_rep_total,
            f_total=F_total_clamped,
            at_local_minimum=at_local_min,
            virtual_target_used=virt_used,
            yaw_rate_clamped=was_clamped,
            neighbor_inflated_dist=inflated,
        )

    def _apply_kinematic_limits(self,
                                 me: UAVRuntimeState,
                                 F: Tuple[float, float, float],
                                 dt: float = 1.0,
                                 ) -> Tuple[Tuple[float, float, float], bool]:
        """將期望力向量轉為可執行的航向變化，夾限至 ψ̇_max 與 γ_max

        流程：
          1. 將 F 轉為期望航向 ψ_des
          2. 計算 Δψ = ψ_des − ψ_current
          3. 夾限至 Δψ_max = ψ̇_max · dt = V·dt/R_min
          4. 垂直分量夾限至 V·sin(γ_max)
          5. 回傳夾限後的向量
        """
        fx, fy, fz = F
        mag = _v3_norm(F)
        if mag < 1e-6:
            return F, False

        V = max(me.ground_speed_mps, self.stall_speed_mps)
        psi_current = math.radians(me.heading_deg)
        # math 角：由東軸逆時針 = atan2(fy, fx)
        psi_des_math = math.atan2(fy, fx)
        # 轉為羅盤：bearing = (90 − psi_math) mod 360
        psi_des_compass = (90.0 - math.degrees(psi_des_math)) % 360.0
        psi_current_compass = me.heading_deg

        # Δψ (compass, 取最短方向 ±180°)
        d_psi = ((psi_des_compass - psi_current_compass + 540.0) % 360.0) - 180.0
        d_psi_max = math.degrees(me.max_yaw_rate_rad_s(V) * dt)

        clamped = False
        if abs(d_psi) > d_psi_max:
            d_psi = math.copysign(d_psi_max, d_psi)
            clamped = True

        # 夾限後的新航向 (compass)
        psi_new_compass = (psi_current_compass + d_psi) % 360.0
        psi_new_math = math.radians((90.0 - psi_new_compass) % 360.0)

        # 重構水平分量 (保持水平合力的模長 = 原 F 水平分量模長的 min(mag, V))
        h_mag = min(math.hypot(fx, fy), V)
        new_fx = h_mag * math.cos(psi_new_math)
        new_fy = h_mag * math.sin(psi_new_math)

        # 垂直分量夾限：|v_z| ≤ V·sin(γ_max)
        v_z_max = V * math.sin(math.radians(me.max_climb_deg))
        new_fz = max(min(fz, v_z_max), -v_z_max)
        if abs(fz) > v_z_max + 1e-6:
            clamped = True

        return (new_fx, new_fy, new_fz), clamped

    def _run_iapf_avoidance(self,
                             assignments: List[CoalitionAssignment],
                             states: Dict[int, UAVRuntimeState],
                             target: TargetState) -> None:
        """對所有聯盟成員檢查 IAPF，若觸發避障則插入中繼航點"""
        coalition_states = [states[a.uav_id] for a in assignments]

        for a in assignments:
            me = states[a.uav_id]
            # 引力目標 = 該機的 IP (而非最終目標，避免近距離震盪)
            goal_alt = a.altitude_layer_m
            rec = self.compute_iapf_force(
                me, coalition_states,
                a.ip_lat, a.ip_lon, goal_alt,
            )
            a.iapf_debug = rec

            # 若 IAPF 合力方向明顯偏離「直接朝 IP」，插入中繼避障航點
            # (將過渡航點朝 F_total 方向偏移)
            if rec.virtual_target_used or rec.yaw_rate_clamped:
                # 將 transition waypoint 沿 F_total 方向重定
                f_norm = _v3_normalize(rec.f_total)
                if _v3_norm(f_norm) > 0.1:
                    d_look = self.transition_lookahead_m
                    dx = f_norm[0] * d_look
                    dy = f_norm[1] * d_look
                    dz = f_norm[2] * d_look * 0.3
                    new_lat, new_lon, new_alt = _enu_to_ll(
                        dx, dy, dz, me.lat, me.lon, me.alt,
                    )
                    a.transition_lat = new_lat
                    a.transition_lon = new_lon
                    a.transition_alt_m = max(me.alt + 10.0,
                                              min(new_alt, a.altitude_layer_m))
                    logger.info(
                        f'[IAPF] {me.name} 依勢場合力重定過渡航點 '
                        f'({new_lat:.6f}, {new_lon:.6f})'
                    )

    # ═════════════════════════════════════════════════════════════════
    #  模組 3：Time-To-Go + STOT/DTOT 盤旋計算
    # ═════════════════════════════════════════════════════════════════

    def _coordinate_time(self,
                          assignments: List[CoalitionAssignment],
                          target: TargetState,
                          warnings: List[str]) -> float:
        """根據 timing_mode 決定各機 t_arrival 並算出 loiter_turns

        2026 重構：TOT 排程 + Loiter 計算委派給 core.strike.time_coordination；
        本模組特有邏輯：尊重 assignments 已分配的 slot_index (Leader=0)，
        使用 'index_order' 排序策略而非 'longest_first'。
        """
        from core.strike.time_coordination import (
            compute_tot_schedule, fill_loiter_turns,
        )

        # 依當前 slot_index 排序 (Leader 已在 slot 0)
        by_slot = sorted(assignments, key=lambda a: a.slot_index)

        base_tot, slots = compute_tot_schedule(
            nominal_times_sec=[a.t_to_go_sec for a in by_slot],
            mode='DTOT' if (self.timing_mode == TimingMode.DTOT
                             and self.interval_sec > 0.01) else 'STOT',
            interval_sec=self.interval_sec,
            dtot_order='index_order',         # 尊重既有 slot_index
            payloads=by_slot,
        )

        # 填 Loiter 圈數 (使用各機自己的 cruise_speed / loiter_radius)
        for slot in slots:
            a = slot.payload
            a.t_arrival_sec = slot.arrival_time_sec
            a.wait_time_sec = slot.wait_time_sec
            V = max(a.cruise_speed_mps, self.stall_speed_mps)
            if a.wait_time_sec > 0.5:
                t_lap = 2.0 * math.pi * a.loiter_radius_m / V
                a.loiter_turns = a.wait_time_sec / max(t_lap, 0.1)
                if a.loiter_turns > 30:
                    warnings.append(
                        f'{a.name} 需盤旋 {a.loiter_turns:.1f} 圈 (> 30)，'
                        f'可能燃料不足'
                    )
            else:
                a.loiter_turns = 0.0

        mode_desc = ('STOT 同秒' if self.timing_mode == TimingMode.STOT
                     else f'DTOT Δ={self.interval_sec:.1f}s')
        logger.info(f'[TimeCoord/{mode_desc}] Base_TOT = {base_tot:.2f}s')
        for a in assignments:
            loi = f' +Loiter {a.loiter_turns:.1f}圈' if a.loiter_turns > 0.1 else ''
            logger.info(
                f'  slot {a.slot_index} [{a.role.value:<7s}] {a.name}: '
                f'T_to_go={a.t_to_go_sec:.1f}s, t_hit={a.t_arrival_sec:.1f}s{loi}'
            )
        return base_tot

    # ═════════════════════════════════════════════════════════════════
    #  模組 1/3 聯合：MAVLink 任務序列生成
    # ═════════════════════════════════════════════════════════════════

    def _build_mission(self, a: CoalitionAssignment,
                       is_leader: bool) -> List[MissionItem]:
        """生成單架 UAV 的 MAVLink 任務序列

        序列 (WINGMAN)：
            0  DO_SET_HOME
            1  DO_CHANGE_SPEED
            2  NAV_WAYPOINT  (平滑過渡)
            3  NAV_WAYPOINT  (Loiter center @ 高度層)
            4  NAV_LOITER_TURNS  [可選，若 wait > 0]
            5  NAV_WAYPOINT  (2km IP)
            6  NAV_WAYPOINT  (IMPACT)

        序列 (LEADER，額外加 DO_REPOSITION 讓 GUIDED 模式先平滑切換)：
            0  DO_SET_HOME
            1  DO_REPOSITION (192) — 當前 pose 附近的固定點
            2  DO_CHANGE_SPEED
            3  NAV_WAYPOINT  (平滑過渡)
            ...
        """
        mission: List[MissionItem] = []
        u = self.uav_states.get(a.uav_id)
        if u is None:
            return mission

        # 0. DO_SET_HOME
        mission.append(MissionItem(
            cmd=MAVCmd.DO_SET_HOME,
            lat=u.lat, lon=u.lon, alt=u.alt,
            comment=f'Home @ 切換時 pose',
        ))

        # Leader 專屬：DO_REPOSITION 先讓飛控平滑回 GUIDED 再進 AUTO
        if is_leader:
            # param2 = bitfield (1 = 選擇當前位置為 reposition 目標)
            # param7 = alt；取當前高度 + 10m 緩衝
            mission.append(MissionItem(
                cmd=MAVCmd.DO_REPOSITION,
                lat=a.transition_lat, lon=a.transition_lon,
                alt=a.transition_alt_m,
                param1=a.cruise_speed_mps,       # 期望地速
                param2=1.0,                        # REPOSITION_FLAGS_CHANGE_MODE
                comment='Leader 平滑 REPOSITION → 過渡航點',
            ))

        # 1. DO_CHANGE_SPEED
        mission.append(MissionItem(
            cmd=MAVCmd.DO_CHANGE_SPEED,
            param1=0.0, param2=float(a.cruise_speed_mps), param3=-1.0,
            comment=f'V={a.cruise_speed_mps:.1f} m/s',
        ))

        # 2. 平滑過渡航點
        mission.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=a.transition_lat, lon=a.transition_lon, alt=a.transition_alt_m,
            param2=float(self.turn_radius_m * 2.0),    # 放寬 acceptance
            comment='Transition WP (Dubins 平滑)',
        ))

        # 3. Loiter center
        mission.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=a.loiter_lat, lon=a.loiter_lon, alt=a.altitude_layer_m,
            param2=float(self.turn_radius_m),
            comment=f'Loiter center @ alt {a.altitude_layer_m:.0f}m',
        ))

        # 4. NAV_LOITER_TURNS (若需補時)
        if a.loiter_turns > 0.1:
            mission.append(MissionItem(
                cmd=MAVCmd.NAV_LOITER_TURNS,
                lat=a.loiter_lat, lon=a.loiter_lon, alt=a.altitude_layer_m,
                param1=float(a.loiter_turns),       # 圈數
                param3=float(a.loiter_radius_m),
                param4=1.0,                           # 達高度方結束
                comment=f'TOT sync: {a.loiter_turns:.1f} turns '
                        f'({a.wait_time_sec:.0f}s)',
            ))

        # 5. 2km IP
        mission.append(MissionItem(
            cmd=MAVCmd.NAV_WAYPOINT,
            lat=a.ip_lat, lon=a.ip_lon, alt=a.altitude_layer_m,
            param2=float(self.turn_radius_m),
            comment=f'2km IP (ψ={a.attack_heading_deg:.0f}°) '
                    f'@ t={a.t_arrival_sec:.1f}s',
        ))

        # 6. IMPACT
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
    def last_result(self) -> Optional[StrikeResult]:
        return self._last_result

    def mission_to_tuples(self, a: CoalitionAssignment
                          ) -> List[Tuple[float, ...]]:
        """MissionItem → SITLLink 所需的 8-tuple"""
        return [
            (float(m.lat), float(m.lon), float(m.alt),
             int(m.cmd),
             float(m.param1), float(m.param2),
             float(m.param3), float(m.param4))
            for m in a.mission
        ]

    def _log_summary(self, result: StrikeResult) -> None:
        logger.info(
            f'[AdvRecon] 重構完成: Leader=UAV-{result.leader_uav_id}, '
            f'timing={result.timing_mode.value}, TOT={result.base_tot_sec:.2f}s'
        )
        for a in result.assignments:
            dbg = a.iapf_debug
            flags = []
            if dbg and dbg.virtual_target_used: flags.append('VirtTarget')
            if dbg and dbg.yaw_rate_clamped:    flags.append('YawClamp')
            if dbg and dbg.at_local_minimum:    flags.append('LocalMin')
            flag_str = f' [{",".join(flags)}]' if flags else ''
            logger.info(
                f'  {a.role.value:<7s} slot{a.slot_index} {a.name}: '
                f'alt={a.altitude_layer_m:.0f}m, '
                f'L={a.path_length_m:.0f}m, '
                f'turns={a.loiter_turns:.1f}, '
                f'WPs={len(a.mission)}{flag_str}'
            )
        if result.warnings:
            for w in result.warnings:
                logger.warning(f'  ! {w}')


# ═══════════════════════════════════════════════════════════════════════
#  Main Demo：3 架固定翼 UAV 掃描中偵測目標 → 重構打擊
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    import time as _t
    now = _t.time()

    # ── 場景：3 架固定翼在台北市北部盤旋掃描 ───────────────
    uavs = [
        UAVRuntimeState(
            uav_id=1, lat=25.12, lon=121.48, alt=300,
            heading_deg=90, ground_speed_mps=25.0,
            vx_mps=25.0, vy_mps=0.0, vz_mps=0.0,   # 向東飛
            last_update_time=now - 0.4,             # 0.4s 延遲
            name='UAV-Alpha',
        ),
        UAVRuntimeState(
            uav_id=2, lat=25.10, lon=121.53, alt=300,
            heading_deg=210, ground_speed_mps=24.0,
            vx_mps=-12.0, vy_mps=-20.8,             # 向西南
            last_update_time=now - 0.6,             # 0.6s 延遲
            name='UAV-Bravo',
        ),
        UAVRuntimeState(
            uav_id=3, lat=25.06, lon=121.48, alt=300,
            heading_deg=30, ground_speed_mps=26.0,
            vx_mps=13.0, vy_mps=22.5,                # 向東北
            last_update_time=now - 0.2,             # 0.2s 延遲
            name='UAV-Charlie',
        ),
    ]
    uav_states = {u.uav_id: u for u in uavs}

    # ── 目標：台北 101 附近 ─────────────────────────────
    target = TargetState(
        lat=25.0330, lon=121.5654, alt=0.0,
        name='TGT-HVT',
        detected_at=now,
    )

    print('═' * 72)
    print(' 情境：DCCPP 3 架固定翼掃描中 → 偵測到高價值目標 → 轉蜂群打擊')
    print('═' * 72)
    print()
    print('[初始 UAV 狀態 (含通訊延遲)]')
    for u in uavs:
        d = _haversine(u.lat, u.lon, target.lat, target.lon)
        lag = (now - u.last_update_time) * 1000
        print(f'  {u.name}: pos=({u.lat:.4f}, {u.lon:.4f}) alt={u.alt:.0f} '
              f'hdg={u.heading_deg:5.1f}° V={u.ground_speed_mps:4.1f}m/s '
              f'| 距目標 {d/1000:.1f}km | 延遲 {lag:.0f}ms')

    # ── 建立管理器 (STOT 模式，IAPF + 延遲補償全開) ────────
    mgr = AdvancedReconToStrikeManager(
        uav_states=uav_states,
        coalition_size=3,
        timing_mode=TimingMode.STOT,
        base_strike_alt_m=500.0,
        altitude_step_m=30.0,
        ip_distance_m=2000.0,
        cruise_speed_mps=25.0,
        stall_speed_mps=18.0,
        turn_radius_m=150.0,
        # IAPF 全開
        iapf_k_att=1.0,
        iapf_k_rep=8.0e4,
        iapf_safe_dist_m=800.0,      # 聯盟匯聚階段較大
        iapf_n=2,
        iapf_lambda=1.2,
        iapf_deadlock_threshold=0.08,
        iapf_virtual_target_dist_m=500.0,
        # 延遲
        communication_delay_sec=0.5,
        position_uncertainty_alpha=0.3,
    )

    # ── 觸發偵察事件 ─────────────────────────────────
    print()
    print('═' * 72)
    print(' [EVENT] target_detected → 動態重構')
    print('═' * 72)
    result = mgr.on_target_detected(target)

    # ── 結果輸出 ────────────────────────────────────
    print()
    print(f'[Leader] UAV-{result.leader_uav_id} '
          f'({uav_states[result.leader_uav_id].name})')
    print(f'[Timing] {result.timing_mode.value} — Base_TOT = {result.base_tot_sec:.2f}s')
    print()
    print(f'{"Role":<8s} {"UAV":<12s} {"slot":>4s} {"ψ(°)":>6s} '
          f'{"alt(m)":>6s} {"L(m)":>7s} {"T_go(s)":>8s} {"wait":>7s} '
          f'{"turns":>6s} {"flags":<18s}')
    print('─' * 90)
    for a in result.assignments:
        dbg = a.iapf_debug
        flags = []
        if dbg and dbg.virtual_target_used: flags.append('VirtTgt')
        if dbg and dbg.yaw_rate_clamped:    flags.append('YawClamp')
        if dbg and dbg.at_local_minimum:    flags.append('LocMin')
        flag_str = ','.join(flags) if flags else '-'
        print(f'{a.role.value:<8s} {a.name:<12s} '
              f'{a.slot_index:>4d} {a.attack_heading_deg:>6.1f} '
              f'{a.altitude_layer_m:>6.0f} {a.path_length_m:>7.0f} '
              f'{a.t_to_go_sec:>8.1f} {a.wait_time_sec:>6.1f}s '
              f'{a.loiter_turns:>6.2f} {flag_str:<18s}')

    # ── IAPF 除錯資訊 ────────────────────────────────
    print()
    print('═' * 72)
    print(' [IAPF] 勢場計算結果')
    print('═' * 72)
    for a in result.assignments:
        if not a.iapf_debug:
            continue
        d = a.iapf_debug
        print(f'  {a.name}:')
        print(f'    F_att  = ({d.f_att[0]:+7.2f}, {d.f_att[1]:+7.2f}, {d.f_att[2]:+6.2f})')
        print(f'    F_rep  = ({d.f_rep[0]:+7.2f}, {d.f_rep[1]:+7.2f}, {d.f_rep[2]:+6.2f})')
        print(f'    F_tot  = ({d.f_total[0]:+7.2f}, {d.f_total[1]:+7.2f}, {d.f_total[2]:+6.2f})')
        # 顯示對每個鄰機的膨脹安全距離
        for nid, r in d.neighbor_inflated_dist.items():
            print(f'    ρ_safe(vs UAV-{nid}) = {r:.1f}m (含延遲不確定性)')

    # ── Leader 的 MAVLink 任務序列 ──────────────────
    print()
    print('═' * 72)
    print(f' [MAVLink] Leader (UAV-{result.leader_uav_id}) 任務序列')
    print('═' * 72)
    leader = [a for a in result.assignments if a.role == CoalitionRole.LEADER][0]
    cmd_name = {
        16: 'NAV_WAYPOINT', 18: 'NAV_LOITER_TURNS', 19: 'NAV_LOITER_TIME',
        178: 'DO_CHANGE_SPEED', 179: 'DO_SET_HOME', 192: 'DO_REPOSITION',
    }
    for i, m in enumerate(leader.mission):
        name = cmd_name.get(m.cmd, f'CMD_{m.cmd}')
        print(f'  seq {i}  {name:<22s} '
              f'({m.lat:10.6f}, {m.lon:11.6f}, {m.alt:6.1f}m)  '
              f'p1={m.param1:6.1f} p2={m.param2:6.1f}')
        print(f'          | {m.comment}')

    print()
    print('═' * 72)
    print(' [演算法特色摘要]')
    print('═' * 72)
    print('  *角色分派：Leader (closest + 速度穩定) + N-1 架 Wingmen')
    print('  *Dubins 平滑過渡：當前航向 + 800m 梯度高度，避免飛控急轉')
    print('  *改進 IAPF：F_rep × ρ_goal² × exp(−λ·ρ₀/ρ_goal) 解決 GNRON')
    print('  *虛擬目標：局部極小時側向 90° 逃逸 (ICAO 右手法則)')
    print('  *運動學夾限：Δψ ≤ V/R_min · Δt, |v_z| ≤ V·sin(γ_max)')
    print('  *延遲補償：鄰機位置外推 + ρ_safe 膨脹 σ = V·Δt·α')
    print('  *TOT 同步：wait = T_slot − T_to_go → NAV_LOITER_TURNS')
    print('  *Leader 加 DO_REPOSITION (192) 讓 GUIDED → AUTO 平滑銜接')
