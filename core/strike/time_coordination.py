"""
core.strike.time_coordination — 共用 STOT/DTOT 時間協同核心
============================================================

2026 重構：原本 5 個 planner 各自實作的 `_coordinate_stot` / `_coordinate_dtot`
邏輯 100% 相同，統一抽到此模組：

    * dtot_coordinator.coordinate()
    * vtol_swarm_strike_planner._coordinate_stot/_dtot()
    * recon_to_strike_manager._coordinate_stot()
    * advanced_swarm_strike_planner._coordinate_stot/_dtot()
    * advanced_recon_to_strike_manager._coordinate_time()

共用演算法：
    base_tot = max(nominal_times)
    STOT : t_arrival[k] = base_tot                     ∀k
    DTOT : t_arrival[k] = base_tot + k · interval_sec
    wait[k] = max(0, t_arrival[k] − nominal[k])
    loiter_turns[k] = wait[k] × V_loiter / (2π · R)

每個 planner 只需提供 nominal 時間 + 速度 + 半徑，回傳 slot schedule，
planner 再把結果回寫到自己的 plan 資料類 (TimingPlan / VTOLPlan / UAVPlan 等)。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, List, Optional, Tuple


# ═══════════════════════════════════════════════════════════════════════
#  列舉
# ═══════════════════════════════════════════════════════════════════════

class TimingMode(str, Enum):
    """時間協同模式（何時命中）"""
    STOT = 'STOT'    # Simultaneous TOT — 同秒命中
    DTOT = 'DTOT'    # Distributed TOT — 間隔依序命中


class LaunchMode(str, Enum):
    """發射位置模式（從哪起飛）— 與 TimingMode 正交

    2026 重構：先前兩者都用 STOT/DTOT 字串造成歧義，現在明確拆開：
        SAME = 同地發射 (共用發射基地) → 對應 TerminalStrikePlanner.plan_stot
        DIFF = 異地發射 (各 UCAV 分散)  → 對應 TerminalStrikePlanner.plan_auto
    """
    SAME = 'SAME'    # Same-launch   — 同地
    DIFF = 'DIFF'    # Different-launch — 異地


# ── 舊值 → 新值對應 (向下相容) ──────────────────────────────
# 以前 launch_mode 曾用 'STOT' 表同地、'DTOT' 表異地 (歷史包袱)。
# 讀進來時先 normalize，避免歧義。
_LEGACY_LAUNCH_ALIAS = {
    'STOT': 'SAME',    # 舊：STOT = 同地發射
    'DTOT': 'DIFF',    # 舊：DTOT = 異地發射
    'SAME': 'SAME',
    'DIFF': 'DIFF',
    '同地': 'SAME',
    '異地': 'DIFF',
}


def normalize_launch_mode(value: str) -> str:
    """把任何 launch_mode 值 (含舊版 STOT/DTOT) 標準化為 'SAME' / 'DIFF'

    呼叫範例::

        # 舊程式傳 'STOT' 當同地 → 自動轉換
        normalize_launch_mode('STOT')   # → 'SAME'
        normalize_launch_mode('DIFF')   # → 'DIFF'  (no-op)
        normalize_launch_mode('同地')   # → 'SAME'
        normalize_launch_mode('unknown')  # ValueError
    """
    if not value:
        return 'DIFF'   # 預設異地
    key = value.strip().upper() if isinstance(value, str) else str(value)
    # 先檢查新值
    if key in ('SAME', 'DIFF'):
        return key
    # legacy / 中文別名
    if key in _LEGACY_LAUNCH_ALIAS:
        return _LEGACY_LAUNCH_ALIAS[key]
    # 中文
    if value in _LEGACY_LAUNCH_ALIAS:
        return _LEGACY_LAUNCH_ALIAS[value]
    raise ValueError(f'未知的 launch_mode: {value!r} (允許 SAME/DIFF/STOT/DTOT/同地/異地)')


class DtotOrder(str, Enum):
    """DTOT 的 slot 排序策略"""
    LONGEST_FIRST  = 'longest_first'    # 最長 L → slot 0 (最先命中，不補時)
    SHORTEST_FIRST = 'shortest_first'   # 最短 L → slot 0 (最先命中，需補時最多)
    INDEX_ORDER    = 'index_order'      # 輸入順序即 slot 順序 (不重排)


class FeasibilityStatus(str, Enum):
    OK              = 'OK'
    SPEED_ADJUSTED  = 'SPEED_ADJUSTED'   # V_req 在 [stall, max] 範圍內
    LOITER_INSERTED = 'LOITER_INSERTED'  # V_req < V_stall → 夾 V_stall + Loiter
    INFEASIBLE      = 'INFEASIBLE'       # V_req > V_max
    PRECLIMB_FAIL   = 'PRECLIMB_FAIL'    # climb_time ≥ arrival_time


# ═══════════════════════════════════════════════════════════════════════
#  資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class TimingSlot:
    """單架 UAV 的時間協同結果

    payload: 呼叫端傳入的 plan 物件 (TimingPlan / VTOLPlan / StrikeAssignment 等)，
            方便 caller 直接把計算結果回寫到自己的資料結構。
    """
    slot_index: int
    nominal_time_sec: float         # 該機以 V_cruise 飛完所需時間
    arrival_time_sec: float         # 目標命中時刻 (從 t=0 起算)
    wait_time_sec: float            # 需要 Loiter 盤旋的秒數
    loiter_turns: float             # NAV_LOITER_TURNS param1
    payload: Any = None             # 呼叫端的 plan 物件參考

    # 進階：若呼叫 compute_loiter_plan 會填入
    required_speed_mps: float = 0.0
    feasibility: FeasibilityStatus = FeasibilityStatus.OK
    warning: str = ''


# ═══════════════════════════════════════════════════════════════════════
#  核心 API
# ═══════════════════════════════════════════════════════════════════════

def compute_tot_schedule(
    nominal_times_sec: List[float],
    mode: str = 'STOT',
    interval_sec: float = 0.0,
    dtot_order: str = 'longest_first',
    tot_margin_sec: float = 0.0,
    payloads: Optional[List[Any]] = None,
) -> Tuple[float, List[TimingSlot]]:
    """計算 STOT/DTOT slot 命中時刻 + wait（不含 Loiter 圈數）。

    Parameters
    ----------
    nominal_times_sec : List[float]
        每架 UAV 以 cruise_speed 抵達目標所需時間
    mode : 'STOT' | 'DTOT'
    interval_sec : float
        DTOT 間隔秒數 (STOT 忽略)
    dtot_order : str
        DTOT 下 slot 排序策略：
          'longest_first' — 最長 L 入 slot 0 (推薦；最省 Loiter)
          'shortest_first' — 最短 L 入 slot 0 (近後遠，需大量 Loiter)
          'index_order'    — 不重排 (照輸入順序)
    tot_margin_sec : float
        TOT 安全邊界
    payloads : List[Any], optional
        與 nominal_times 對應的 plan 物件 (slot.payload 會指向這些)

    Returns
    -------
    (base_tot, slots) :
        base_tot: slot 0 的命中時刻
        slots: 依「原始 nominal_times 輸入順序」回傳的 TimingSlot list
               (slot_index 欄位才是排序後的 slot 編號)
    """
    mode = mode.upper()
    if mode not in ('STOT', 'DTOT'):
        raise ValueError(f'mode 必須是 STOT 或 DTOT，得 {mode}')

    n = len(nominal_times_sec)
    if n == 0:
        return 0.0, []

    if payloads is None:
        payloads = [None] * n

    base_tot = max(nominal_times_sec) + tot_margin_sec

    if mode == 'STOT' or interval_sec <= 0.01:
        # STOT 全體同秒命中；順序不重要，slot_index 按輸入順序
        slots = [
            TimingSlot(
                slot_index=i,
                nominal_time_sec=t_nom,
                arrival_time_sec=base_tot,
                wait_time_sec=max(0.0, base_tot - t_nom),
                loiter_turns=0.0,                       # caller 稍後填
                payload=payloads[i],
            )
            for i, t_nom in enumerate(nominal_times_sec)
        ]
        return base_tot, slots

    # ── DTOT: 依 order 策略重排 ────────────────────────────
    indexed = list(enumerate(nominal_times_sec))   # [(orig_idx, t_nom), ...]
    if dtot_order == 'longest_first':
        indexed.sort(key=lambda x: -x[1])
    elif dtot_order == 'shortest_first':
        indexed.sort(key=lambda x: x[1])
    # 'index_order' 不排序

    # slot_k 的命中時刻 = base + k·Δ，套用到排序後的位置
    slot_arrival = {
        orig_idx: base_tot + k * interval_sec
        for k, (orig_idx, _) in enumerate(indexed)
    }
    slot_index_map = {
        orig_idx: k
        for k, (orig_idx, _) in enumerate(indexed)
    }

    slots = [
        TimingSlot(
            slot_index=slot_index_map[i],
            nominal_time_sec=t_nom,
            arrival_time_sec=slot_arrival[i],
            wait_time_sec=max(0.0, slot_arrival[i] - t_nom),
            loiter_turns=0.0,
            payload=payloads[i],
        )
        for i, t_nom in enumerate(nominal_times_sec)
    ]
    return base_tot, slots


def fill_loiter_turns(slots: List[TimingSlot],
                      cruise_speed_mps: float,
                      loiter_radius_m: float,
                      threshold_sec: float = 0.5) -> None:
    """依 wait_time 補上 loiter_turns。

    in-place 修改 slots；若 wait < threshold 則 loiter_turns=0。

    公式：T_lap = 2π · R / V → loiter_turns = wait / T_lap
    """
    if cruise_speed_mps <= 0 or loiter_radius_m <= 0:
        return
    t_lap = 2.0 * math.pi * loiter_radius_m / cruise_speed_mps
    for s in slots:
        if s.wait_time_sec >= threshold_sec:
            s.loiter_turns = s.wait_time_sec / max(t_lap, 0.1)
        else:
            s.loiter_turns = 0.0


# ═══════════════════════════════════════════════════════════════════════
#  進階：反推空速版本 (給 DTOTCoordinator 用)
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class LoiterPlanOutput:
    """compute_loiter_plan 的逐機結果"""
    required_speed_mps: float
    wait_time_sec: float
    loiter_turns: float
    feasibility: FeasibilityStatus
    warning: str = ''


def compute_loiter_plan(
    path_length_m: float,
    arrival_time_sec: float,
    climb_time_sec: float = 0.0,
    *,
    cruise_speed_mps: float,
    stall_speed_mps: float,
    max_speed_mps: float,
    loiter_radius_m: float,
    dtot_interval_mode: bool = False,
) -> LoiterPlanOutput:
    """給定路徑長度 + 目標抵達時刻，決定空速與 Loiter。

    完整決策樹（與原 DTOTCoordinator._solve_speed_or_loiter 邏輯一致）：
      1. available = arrival_time − climb_time；若 ≤ 0.1 → PRECLIMB_FAIL
      2. V_req = L / available
         ├── > V_max                 → INFEASIBLE (夾 V_max，可能無法按時抵達)
         ├── < V_stall               → LOITER_INSERTED (夾 V_stall + Loiter)
         └── 其他                     → OK / SPEED_ADJUSTED
      3. DTOT 間隔模式特殊：V_req 介於 [V_stall, V_cruise·0.95] 時，
         仍可用 V_cruise 飛 + Loiter 補差 (戰術上更省油)

    Returns
    -------
    LoiterPlanOutput with (required_speed, wait, loiter_turns, feasibility, warning)
    """
    if max_speed_mps <= stall_speed_mps:
        raise ValueError('max_speed 必須 > stall_speed')

    t_lap_at = lambda v: 2.0 * math.pi * loiter_radius_m / max(v, 0.1)

    available = arrival_time_sec - climb_time_sec
    if available <= 0.1:
        return LoiterPlanOutput(
            required_speed_mps=cruise_speed_mps,
            wait_time_sec=0.0, loiter_turns=0.0,
            feasibility=FeasibilityStatus.PRECLIMB_FAIL,
            warning=f'climb_time({climb_time_sec:.1f}s) ≥ TOT → 無法協同',
        )

    v_req = path_length_m / available

    # 案例 1: V_req > V_max → 不可行
    if v_req > max_speed_mps + 1e-3:
        return LoiterPlanOutput(
            required_speed_mps=max_speed_mps,
            wait_time_sec=0.0, loiter_turns=0.0,
            feasibility=FeasibilityStatus.INFEASIBLE,
            warning=f'V_req={v_req:.2f} > V_max={max_speed_mps:.1f}',
        )

    # 案例 2: V_req < V_stall → 夾 V_stall + Loiter
    if v_req < stall_speed_mps - 1e-3:
        time_at_stall = path_length_m / stall_speed_mps
        wait = max(0.0, available - time_at_stall)
        return LoiterPlanOutput(
            required_speed_mps=stall_speed_mps,
            wait_time_sec=wait,
            loiter_turns=wait / max(t_lap_at(stall_speed_mps), 0.1),
            feasibility=FeasibilityStatus.LOITER_INSERTED,
            warning=(f'V_req={v_req:.2f} < V_stall → 夾 {stall_speed_mps:.1f} '
                     f'+ Loiter {wait:.1f}s'),
        )

    # 案例 3: DTOT 模式特例：V_cruise + Loiter 更自然
    if dtot_interval_mode and v_req < cruise_speed_mps * 0.95:
        time_at_cruise = path_length_m / cruise_speed_mps
        wait = max(0.0, available - time_at_cruise)
        return LoiterPlanOutput(
            required_speed_mps=cruise_speed_mps,
            wait_time_sec=wait,
            loiter_turns=wait / max(t_lap_at(cruise_speed_mps), 0.1),
            feasibility=FeasibilityStatus.LOITER_INSERTED,
            warning=f'DTOT 延遲：V=V_cruise + Loiter {wait:.1f}s',
        )

    # 案例 4: V_req 在 [stall, cruise·0.95, max] → 調速
    status = (FeasibilityStatus.OK
              if abs(v_req - cruise_speed_mps) < 0.5
              else FeasibilityStatus.SPEED_ADJUSTED)
    return LoiterPlanOutput(
        required_speed_mps=v_req,
        wait_time_sec=0.0, loiter_turns=0.0,
        feasibility=status,
        warning='',
    )


# ═══════════════════════════════════════════════════════════════════════
#  一次性 API：建議 planner 內部呼叫此函式 (最簡路徑)
# ═══════════════════════════════════════════════════════════════════════

def coordinate(
    nominal_times_sec: List[float],
    *,
    mode: str = 'STOT',
    interval_sec: float = 0.0,
    cruise_speed_mps: float,
    loiter_radius_m: float,
    dtot_order: str = 'longest_first',
    tot_margin_sec: float = 0.0,
    payloads: Optional[List[Any]] = None,
) -> Tuple[float, List[TimingSlot]]:
    """一次完成 STOT/DTOT 排程 + Loiter 圈數計算。

    適用於 vtol/recon/advanced_swarm 等「保持 V_cruise + Loiter 補時」的 planner。
    若需要反推空速（如 DTOTCoordinator），請額外呼叫 :func:`compute_loiter_plan`。
    """
    base_tot, slots = compute_tot_schedule(
        nominal_times_sec=nominal_times_sec,
        mode=mode,
        interval_sec=interval_sec,
        dtot_order=dtot_order,
        tot_margin_sec=tot_margin_sec,
        payloads=payloads,
    )
    fill_loiter_turns(slots, cruise_speed_mps, loiter_radius_m)
    return base_tot, slots
