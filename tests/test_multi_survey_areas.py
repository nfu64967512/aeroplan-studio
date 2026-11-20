"""模式二：使用者圈好幾塊小區，規劃器一起收。

⚠ 這些測試**不建構 MainWindow** —— 它需要 QApplication 而且 QtWebEngine 在這個
環境會硬當（exit 127）。改成拿未綁定的方法配一個只有必要欄位的替身物件跑，
因為這幾個方法只碰 `survey_areas` 與 `corners`。
"""
from __future__ import annotations

import inspect
import types

import pytest

import ui.main_window as mw


class _Stub:
    """只有 survey_area_polygons 會碰到的欄位。"""

    def __init__(self, committed, drawing):
        self.survey_areas = list(committed)
        self.corners = list(drawing)


def _polys(committed, drawing):
    return mw.MainWindow.survey_area_polygons(_Stub(committed, drawing))


TRI = [(23.95, 120.90), (23.96, 120.90), (23.96, 120.91)]
SQ = [(23.94, 120.88), (23.95, 120.88), (23.95, 120.89), (23.94, 120.89)]


def test_single_area_behaviour_is_unchanged():
    """沒有已完成的區時，行為與改動前相同：只有正在畫的那一塊。"""
    assert _polys([], TRI) == [TRI]
    assert _polys([], []) == []


def test_committed_areas_are_included():
    assert _polys([TRI, SQ], []) == [TRI, SQ]


def test_area_being_drawn_is_appended_last():
    """已完成的區 + 正在畫的那一塊，順序是「先完成的在前」。"""
    out = _polys([SQ], TRI)
    assert out == [SQ, TRI]


def test_incomplete_drawing_is_ignored():
    """正在畫的那一塊不足 3 點就不算 —— 但已完成的區仍然要送出去。"""
    assert _polys([SQ], [(23.95, 120.90), (23.96, 120.90)]) == [SQ]


def test_degenerate_committed_area_is_dropped():
    """已完成清單裡如果混進不足 3 點的東西，要被擋掉而不是送給規劃器。"""
    assert _polys([[(23.95, 120.90)], SQ], []) == [SQ]


# ── 接線（原始碼層，因為建不了視窗）────────────────────────────────
def test_planner_entry_builds_areas_from_the_list():
    """覆蓋規劃入口的 **areas 組裝**要走 `survey_area_polygons()`。

    ⚠⚠ 只斷言 `"survey_area_polygons()" in src` 是**量不到東西的**：我加的那道
    閘門也用同一個字串，所以把組裝改回 `[list(self.corners)]` 時這條照樣會綠。
    實測過那個突變 —— 14 條測試全綠，沒有一條咬。所以要分別釘住
    「`_polys` 從哪來」與「areas 從 `_polys` 長出來」。
    """
    src = inspect.getsource(mw.MainWindow._on_dccpp_coverage_requested)
    assert "_polys = self.survey_area_polygons()" in src, "areas 的來源被換掉了"
    assert "for i, p in enumerate(_polys)" in src, "areas 不是從 _polys 組出來的"
    assert "areas = [{'area_id': 1, 'polygon': list(self.corners)" not in src, \
        "還在寫死單一區"


def test_both_gates_accept_committed_areas():
    """兩道「邊界點不足」的閘門都要認已完成的區，否則按了『完成本區』就規劃不了。"""
    for fn in (mw.MainWindow._on_dccpp_coverage_requested,
               mw.MainWindow._on_vtol_recon_plan):
        src = inspect.getsource(fn)
        assert "survey_area_polygons()" in src, fn.__name__


def test_centroid_uses_every_area():
    """沒設起飛點時的質心要取所有區的角點，不是只取正在畫的那一塊。"""
    src = inspect.getsource(mw.MainWindow._on_dccpp_coverage_requested)
    assert "_all_pts" in src and "for p in _polys for c in p" in src


def test_commit_and_clear_exist_and_are_wired():
    for name in ("commit_current_area", "clear_all_survey_areas",
                 "survey_area_polygons", "_sync_area_counts",
                 "_render_committed_areas"):
        assert isinstance(getattr(mw.MainWindow, name, None), types.FunctionType), name
    src = inspect.getsource(mw.MainWindow)
    assert "commit_area_requested.connect" in src
    assert "clear_areas_requested.connect" in src


def test_render_failure_does_not_break_planning():
    """地圖畫不出來時只能記 log，不可以讓規劃失敗 —— 資料與顯示要分開。"""
    src = inspect.getsource(mw.MainWindow._render_committed_areas)
    assert "except Exception" in src and "logger.warning" in src


# ── 面板 ──────────────────────────────────────────────────────────
@pytest.fixture
def panel(qapp):
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    return VTOLReconPanel()


@pytest.fixture(scope="session")
def qapp():
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_panel_has_the_two_signals(panel):
    assert hasattr(panel, "commit_area_requested")
    assert hasattr(panel, "clear_areas_requested")


def test_panel_area_count_updates_the_label(panel):
    panel.set_area_count(0, 0)
    assert "多區" in panel.areas_label.text()
    panel.set_area_count(3, 2)
    txt = panel.areas_label.text()
    assert "3" in txt and "2" in txt


def test_committed_areas_alone_enable_planning(panel):
    """按了「完成本區」之後畫布是空的，規劃鈕不可以因此變灰。"""
    panel._footprint_ok = True
    panel.set_corner_count(0)
    panel.set_area_count(0, 0)
    assert not panel.btn_plan.isEnabled()
    panel.set_area_count(2, 0)
    assert panel.btn_plan.isEnabled(), "已完成 2 塊區卻不讓規劃"


# ── 地圖顯示（只驗介面存在與失敗不致命，畫面本身在此環境無法驗證）──────
def test_map_widgets_expose_display_survey_areas():
    """兩個地圖都要有這個方法。

    ⚠ `DualMapWidget.__getattr__` 會代理到 2D，所以沒有明確方法時也「會動」——
    但那是碰巧。這條要求它是被寫出來的，這樣「3D 目前不畫」才是有意識的決定。
    """
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.widgets.map_widget import MapWidget
    assert callable(getattr(MapWidget, "display_survey_areas", None))
    assert "display_survey_areas" in DualMapWidget.__dict__, \
        "只靠 __getattr__ 代理 ⇒ 3D 畫不畫是意外不是決定"
