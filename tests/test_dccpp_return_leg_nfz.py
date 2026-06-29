"""tests/test_dccpp_return_leg_nfz.py — DCCPP 回程穿越 NFZ 修正回歸測試。

情境
----
DCCPP 固定翼覆蓋任務尾端是一串 ``SegmentLabel.LANDING`` 進場航點（場周入口 →
下降階梯 → 觸地 NAV_LAND），全部聚在 home 附近。真正會穿越 NFZ 的是
「最後一個作業點 → 第一個 LANDING 航點（場周入口）」這條長回程巡航段。

修正前：``_apply_fence_avoidance_to_dccpp_result`` 把「整段」尾端 LANDING 一起
豁免 NFZ 避障，連長回程巡航段都跳過 → 回程直穿 NFZ（使用者回報的 bug）。

修正後：``_split_landing_tail`` 把第一個 LANDING 航點（場周入口）併入避障前段，
使回程巡航段被 ``_segment_aware_fence_avoidance`` 的 VG 繞行涵蓋；其餘下降進場
型態仍原樣豁免。

本測試在公尺座標下組合這兩個 static 方法（不需啟動 Qt 事件迴圈），驗證：
  1. _split_landing_tail 確實把場周入口併入 head、其餘下降進場留在 exempt tail。
  2. 修正後回程巡航段不再穿越 NFZ（buffered 多邊形）。
  3. 對照組：若沿用舊行為（整段豁免），該回程段仍會穿越 NFZ。
  4. 下降進場型態（含 alt=0 觸地點）被完整保留、未被避障改動。
"""
from __future__ import annotations

import os

# 匯入 ui.main_window 前先指定 offscreen 平台，避免 headless 環境需要顯示器。
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from typing import List, Tuple

import pytest

from shapely.geometry import LineString

from core.trajectory.dccpp_path_assembler import AssembledWaypoint, SegmentLabel
from core.global_planner.nfz_planner import FixedWingNFZPlanner
from core.base.fixed_wing_constraints import FixedWingConstraints
from ui.main_window import MainWindow


# ──────────────────────────────────────────────────────────────────────
#  測試夾具：公尺座標下的回程穿越 NFZ 場景
# ──────────────────────────────────────────────────────────────────────
# 全部以「home 為原點」的本地公尺座標表示（避免 latlon 投影誤差干擾幾何斷言）。
#   home / 觸地點         : (0, 0)
#   場周入口（回程到達點）: (-300, 0)         ← 近 home
#   最後作業點（掃描出口）: (2500, 2000)      ← 遠離 home，位於 NFZ 另一側
#   NFZ 矩形              : x∈[800,1600], y∈[400,1100]
# 「掃描出口 → 場周入口」直線會穿過此矩形；兩端點皆在矩形外（VG 可繞行）。
_NFZ_VERTS_XY: List[Tuple[float, float]] = [
    (800.0, 400.0), (1600.0, 400.0), (1600.0, 1100.0), (800.0, 1100.0),
]
_OP_PREV_XY = (2200.0, 1800.0)     # 倒數第二個作業點
_OP_LAST_XY = (2500.0, 2000.0)     # 最後作業點（掃描出口）
_LANDING_ENTRY_XY = (-300.0, 0.0)  # 場周入口（第一個 LANDING 航點）
_LANDING_STEP_XY = (-150.0, 0.0)   # 下降階梯
_LANDING_TD_XY = (0.0, 0.0)        # 觸地點 NAV_LAND（alt=0）


def _wp(xy: Tuple[float, float], seg: SegmentLabel, alt: float = 100.0) -> AssembledWaypoint:
    """以本地公尺座標當成 lat/lon 佔位（static 方法只用 segment_type，不用座標）。"""
    return AssembledWaypoint(
        lat=xy[1], lon=xy[0], alt=alt, heading_deg=0.0, segment_type=seg,
    )


def _make_path() -> List[AssembledWaypoint]:
    """組出 [OPERATION, OPERATION, LANDING, LANDING, LANDING] 的完整路徑。"""
    return [
        _wp(_OP_PREV_XY, SegmentLabel.OPERATION),
        _wp(_OP_LAST_XY, SegmentLabel.OPERATION),
        _wp(_LANDING_ENTRY_XY, SegmentLabel.LANDING, alt=80.0),
        _wp(_LANDING_STEP_XY, SegmentLabel.LANDING, alt=40.0),
        _wp(_LANDING_TD_XY, SegmentLabel.LANDING, alt=0.0),
    ]


def _make_planner() -> FixedWingNFZPlanner:
    """建立含單一矩形 NFZ 的固定翼避障規劃器（公尺座標）。"""
    constraints = FixedWingConstraints(
        cruise_airspeed_mps=18.0,
        max_bank_angle_deg=45.0,
    )
    planner = FixedWingNFZPlanner(constraints, buffer_factor=1.5)
    planner.add_polygon_nfz(_NFZ_VERTS_XY, name='NFZ-1', coord_type='metric')
    planner._rebuild_merged()
    return planner


def _buffered_polys(planner: FixedWingNFZPlanner) -> list:
    """取得 planner 已合併且膨脹的禁區多邊形 list。"""
    merged = planner._merged_buffered
    assert merged is not None and not merged.is_empty
    return (list(merged.geoms) if merged.geom_type == 'MultiPolygon' else [merged])


def _penetrates(a, b, polys, tol_m: float = 0.5) -> bool:
    """線段 [a,b] 是否「穿入」任一禁區內部（沿邊滑行/僅觸點不算）。

    與 _apply_fence_avoidance_to_dccpp_result 的 safety-net 採同一判準，
    避免「端點落在 buffered 邊界」造成的觸點誤判。
    """
    seg = LineString([a, b])
    if seg.length < 1e-9:
        return False
    for poly in polys:
        if not seg.intersects(poly):
            continue
        inter = seg.intersection(poly)
        if inter.is_empty or inter.geom_type in ('Point', 'MultiPoint'):
            continue
        if getattr(inter, 'length', 0.0) > tol_m:
            return True
    return False


def _path_penetrates(pts, polys) -> bool:
    """整條折線是否有任一段穿入禁區。"""
    return any(_penetrates(pts[k], pts[k + 1], polys) for k in range(len(pts) - 1))


# ──────────────────────────────────────────────────────────────────────
#  測試
# ──────────────────────────────────────────────────────────────────────
class TestSplitLandingTail:
    """_split_landing_tail：場周入口併入避障前段、其餘下降進場豁免。"""

    def test_entry_moved_into_head(self):
        path = _make_path()
        head, exempt_tail = MainWindow._split_landing_tail(path)

        # head 末端應為第一個 LANDING 航點（場周入口），前面是兩個 OPERATION。
        assert len(head) == 3
        assert head[0].segment_type == SegmentLabel.OPERATION
        assert head[1].segment_type == SegmentLabel.OPERATION
        assert head[2].segment_type == SegmentLabel.LANDING
        assert (head[2].lon, head[2].lat) == _LANDING_ENTRY_XY

        # exempt tail 為其餘下降進場型態（下降階梯 + 觸地點）。
        assert len(exempt_tail) == 2
        assert all(wp.segment_type == SegmentLabel.LANDING for wp in exempt_tail)

    def test_touchdown_preserved_in_exempt_tail(self):
        path = _make_path()
        _, exempt_tail = MainWindow._split_landing_tail(path)
        # 觸地點（alt=0，匯出時的 NAV_LAND）必須完整保留在豁免尾段。
        assert exempt_tail[-1].alt == 0.0
        assert (exempt_tail[-1].lon, exempt_tail[-1].lat) == _LANDING_TD_XY

    def test_no_landing_tail_passthrough(self):
        """無 LANDING 尾段（例如多旋翼 / 關閉 auto-landing）時原樣回傳。"""
        path = [
            _wp(_OP_PREV_XY, SegmentLabel.OPERATION),
            _wp(_OP_LAST_XY, SegmentLabel.OPERATION),
        ]
        head, exempt_tail = MainWindow._split_landing_tail(path)
        assert len(head) == 2
        assert exempt_tail == []


class TestReturnLegAvoidance:
    """回程巡航段（掃描出口 → 場周入口）的 NFZ 避障行為。"""

    def test_scenario_is_valid_straight_return_crosses_nfz(self):
        """前提檢查：未避障時，回程直線確實穿越 NFZ（否則測試無意義）。"""
        planner = _make_planner()
        polys = _buffered_polys(planner)
        assert _penetrates(_OP_LAST_XY, _LANDING_ENTRY_XY, polys) is True

    def test_old_behaviour_would_leave_return_crossing(self):
        """對照組：沿用舊行為（整段 LANDING 豁免）→ 回程段未被避障 → 仍穿越。

        舊行為等價於 head 不含場周入口，因此 head 末端是最後作業點，
        而「最後作業點 → 場周入口」這條接縫從未進入避障輸入。
        """
        planner = _make_planner()
        polys = _buffered_polys(planner)
        old_head_metric = [_OP_PREV_XY, _OP_LAST_XY]  # 不含場周入口
        corrected_old, _ = MainWindow._segment_aware_fence_avoidance(
            [_wp(_OP_PREV_XY, SegmentLabel.OPERATION),
             _wp(_OP_LAST_XY, SegmentLabel.OPERATION)],
            old_head_metric, planner,
        )
        # 舊 head 只到最後作業點；之後直接接 landing tail（場周入口）。
        join_then_return = list(corrected_old) + [_LANDING_ENTRY_XY]
        assert _path_penetrates(join_then_return, polys) is True

    def test_fixed_behaviour_reroutes_return_leg(self):
        """修正後：場周入口併入避障 → 回程巡航段被 VG 繞行，整段不再穿越 NFZ。"""
        planner = _make_planner()
        polys = _buffered_polys(planner)

        path = _make_path()
        head, _exempt = MainWindow._split_landing_tail(path)
        metric_head = [(wp.lon, wp.lat) for wp in head]  # 佔位 lon/lat 即本地公尺

        corrected, refs = MainWindow._segment_aware_fence_avoidance(
            head, metric_head, planner,
        )

        # 避障應插入繞行點（不再是 3 點直連）。
        assert len(corrected) > len(metric_head)
        # 起點與終點（場周入口）保持不變。
        assert corrected[0] == pytest.approx(metric_head[0])
        assert corrected[-1] == pytest.approx(metric_head[-1])
        # 關鍵斷言：修正後整條 head（含回程巡航段）不再穿越 NFZ。
        assert _path_penetrates(corrected, polys) is False
        # ref 索引數與輸出點數一致。
        assert len(refs) == len(corrected)
