"""
StrikeResultDict — MainWindow._strike_result 的型別定義

先前此欄位為自由 dict，key 散落在程式各處，易出錯。2026 重構改用 TypedDict
提供型別提示與 IDE 自動補全，但保留 dict 行為 (不破壞現有 30+ 存取點)。

使用範例::

    from core.strike.strike_result import StrikeResultDict, make_empty_strike_result

    self._strike_result: Optional[StrikeResultDict] = make_empty_strike_result(
        mode='DTOT',
        timing_mode='STOT',
        trajectories=trajs,
    )
    # 仍可用舊 dict 語法
    print(self._strike_result['mode'])
    speed = self._strike_result.get('coord_plans', [])
"""
from __future__ import annotations

from typing import Any, List, Optional, Tuple, TypedDict


class StrikeResultDict(TypedDict, total=False):
    """蜂群打擊規劃結果 dict schema

    核心欄位 (固定翼):
        mode          : 'DTOT' | 'STOT'  發射位置 (異地 / 同地)
        timing_mode   : 'STOT' | 'DTOT'  時間協同 (同秒 / 間隔)
        interval_sec  : DTOT 間隔秒數 (STOT 為 0)
        launch_base   : (lat, lon) 或 None (STOT 共用基地)
        trajectories  : List[StrikeTrajectory] 固定翼三段軌跡
        targets       : List[StrikeTarget]
        params        : 傳入的原始參數 dict
        coord_plans   : List[TimingPlan] DTOTCoordinator 協同結果 (可選)

    VTOL 專屬:
        is_vtol       : True = VTOL 規劃流程
        vtol_plans    : List[VTOLPlan]
        vtol_planner  : VTOLSwarmStrikePlanner 實例 (供內建匯出使用)
        target        : VTOLTarget (含 CEP 與速度向量)
    """
    # 共通
    mode:         str
    timing_mode:  str
    interval_sec: float
    launch_base:  Optional[Tuple[float, float]]
    params:       dict

    # 固定翼
    trajectories: List[Any]
    targets:      List[Any]
    coord_plans:  List[Any]

    # VTOL
    is_vtol:      bool
    vtol_plans:   List[Any]
    vtol_planner: Any
    target:       Any


def make_empty_strike_result(**overrides) -> StrikeResultDict:
    """建立具備預設值的 StrikeResultDict (便於保證 schema 一致)"""
    default: StrikeResultDict = {
        'mode':         'DTOT',
        'timing_mode':  'STOT',
        'interval_sec': 0.0,
        'launch_base':  None,
        'params':       {},
        'is_vtol':      False,
    }
    default.update(overrides)
    return default
