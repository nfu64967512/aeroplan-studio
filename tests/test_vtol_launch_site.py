"""VTOL 偵察的起降場由使用者決定（2026-10-08 使用者定調）。

背景：2026-10-07 的規劃 log 是 ``home=None`` ⇒ 起降坪被默默放在作業區質心，
三台的進場、返場、降落全擠在作業區正中央。起飛點選擇器其實存在，但藏在
「固定翼專用設定」裡、標示為「跑道位置」，VTOL 偵察頁看不到。

決定：
  * VTOL 偵察頁「1. 偵察區域」加「從地圖選起降場」（狀態仍是 MainWindow.home_point，
    與固定翼「起飛點」同一份）；
  * **沒選就擋下**並提示去選，不再退回作業區質心；
  * 降落行為不變（全部回 1 號坪、排隊降落）。

不建 MainWindow（本機 QtWebEngine 一 setHtml 就 abort）：未綁定方法 + 假 self。
"""
from __future__ import annotations

import ast
import inspect
import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from ui.main_window import MainWindow          # 先載入 QtWebEngine
    import ui.main_window as mw
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from PyQt6.QtWidgets import QApplication
    _QT_OK = True
except Exception as _e:                             # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

SITE = (25.0601, 121.2052)
AREA = [(25.0718, 121.2121), (25.0594, 121.2248), (25.0774, 121.2459), (25.0879, 121.2349)]


@pytest.fixture(scope="module")
def qt_app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture()
def panel(qt_app):
    return VTOLReconPanel()


class _Rec:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def _rec(*a, **k):
            self.calls.append((name, a, k))
            return self
        return _rec


class _ReconWin:
    """跑 `_on_vtol_recon_plan` 的假 self。"""

    def __init__(self, panel, home):
        self.vtol_recon_panel = panel
        self.home_point = home
        self.planned = []
        self.survey_areas = []
        self.corners = list(AREA)
        self.survey_area_polygons = types.MethodType(MainWindow.survey_area_polygons, self)

    def _on_dccpp_coverage_requested(self, params, *, run_async=True, on_done=None):
        self.planned.append(params)


@pytest.fixture()
def warnings(monkeypatch):
    seen = []
    monkeypatch.setattr(mw.QMessageBox, "warning",
                        staticmethod(lambda *a, **k: seen.append(a[1:3])))
    return seen


# ── 1. 沒選起降場 ⇒ 擋下並提示 ─────────────────────────────────────────
@pytest.mark.parametrize("planner,hv", [("hv_dccpp", True), ("dccpp", False)])
def test_plan_is_blocked_without_launch_site(panel, warnings, planner, hv):
    win = _ReconWin(panel, home=None)
    MainWindow._on_vtol_recon_plan(win, {"planner": planner, "hv_dccpp": hv,
                                         "fov_width_m": 110.0, "speed": 22.0})
    assert win.planned == [], "沒選起降場卻送去規劃了（會默默用作業區中心）"
    assert warnings and warnings[-1][0] == "尚未選擇起降場"
    assert panel.plan_status.text() == "尚未選擇起降場"


@pytest.mark.parametrize("planner,hv", [("hv_dccpp", True), ("dccpp", False)])
def test_plan_goes_through_with_launch_site(panel, warnings, planner, hv):
    win = _ReconWin(panel, home=SITE)
    MainWindow._on_vtol_recon_plan(win, {"planner": planner, "hv_dccpp": hv,
                                         "fov_width_m": 110.0, "speed": 22.0})
    assert len(win.planned) == 1 and warnings == []


# ── 2. 面板：按鈕發訊號、顯示跟著狀態 ─────────────────────────────────────
def test_panel_buttons_emit_signals(panel):
    got = []
    panel.pick_launch_site_requested.connect(lambda: got.append("pick"))
    panel.clear_launch_site_requested.connect(lambda: got.append("clear"))
    panel.btn_pick_launch_site.click()
    panel.btn_clear_launch_site.click()
    assert got == ["pick", "clear"]


def test_panel_starts_unset_and_shows_the_site(panel):
    assert "尚未選擇" in panel.launch_site_label.text()
    panel.set_launch_site(*SITE)
    assert f"{SITE[0]:.6f}" in panel.launch_site_label.text()
    panel.clear_launch_site()
    assert "尚未選擇" in panel.launch_site_label.text()


def test_main_window_keeps_panel_in_sync(panel):
    """起降場與固定翼「起飛點」是同一份 home_point：任何一邊設定／清除，VTOL 面板都要跟著變。"""
    win = types.SimpleNamespace(home_point=None, picking_home_point=True, vtol_recon_panel=panel,
                                parameter_panel=_Rec(), map_widget=_Rec(), _status=_Rec())
    win.statusBar = lambda: win._status
    win._sync_launch_site_display = types.MethodType(MainWindow._sync_launch_site_display, win)

    MainWindow.on_home_point_set(win, *SITE)
    assert win.home_point == SITE and not win.picking_home_point
    assert f"{SITE[1]:.6f}" in panel.launch_site_label.text()

    MainWindow.on_clear_home_point(win)
    assert win.home_point is None
    assert "尚未選擇" in panel.launch_site_label.text()


def test_clear_area_also_clears_the_displayed_site(panel):
    """「清除」偵察區（on_clear_corners）一向會一併清掉起飛點 ⇒ 面板不可還顯示舊的起降場。"""
    pp = types.SimpleNamespace(update_corner_count=lambda n: None,
                               clear_home_point_display=lambda: None)
    win = types.SimpleNamespace(home_point=SITE, picking_home_point=False, vtol_recon_panel=panel,
                                parameter_panel=pp, map_widget=_Rec(), corners=list(AREA),
                                circle_center=None, picking_circle_center=False)
    win._sync_launch_site_display = types.MethodType(MainWindow._sync_launch_site_display, win)
    panel.set_launch_site(*SITE)

    MainWindow.on_clear_corners(win)
    assert win.home_point is None
    assert "尚未選擇" in panel.launch_site_label.text(), "home_point 清了，面板卻還顯示舊的起降場"


# ── 3. 從 3D 點選起降場：不可在 3D 留下一顆作業區角點 ──────────────────────
def test_pick_in_3d_removes_the_stray_3d_corner(panel):
    class _Map3D:
        def __init__(self):
            self.corners = [AREA[0], SITE]          # 3D 點擊時 Cesium 自己加了一顆

        def remove_last_corner(self):
            return self.corners.pop()

    class _Dual:
        def __init__(self):
            self.corners = list(AREA) + [SITE]    # DualMapWidget 已把 3D 點擊鏡射進 2D
            self.markers = [object()] * len(self.corners)
            self.map_3d = _Map3D()
            self.overlay = None

        def _render_map(self):
            pass

        def set_home_point_overlay(self, lat, lon):
            self.overlay = (lat, lon)

    dual = _Dual()
    win = types.SimpleNamespace(home_point=None, picking_home_point=True, vtol_recon_panel=panel,
                                parameter_panel=_Rec(), map_widget=dual, _status=_Rec())
    win.statusBar = lambda: win._status
    win._sync_launch_site_display = types.MethodType(MainWindow._sync_launch_site_display, win)
    win.on_home_point_set = types.MethodType(MainWindow.on_home_point_set, win)

    MainWindow.on_corner_added(win, *SITE)
    assert win.home_point == SITE
    assert dual.corners == list(AREA), "2D：點選起降場那一下被當成作業區角點留下來了"
    assert dual.map_3d.corners == [AREA[0]], "3D：起降場位置上留了一顆作業區角點"
    assert dual.overlay == SITE


# ── 4. 接線：面板訊號真的接到 home_point 的處理函式 ─────────────────────────
def test_panel_signals_are_wired_to_home_point_handlers():
    tree = ast.parse(inspect.getsource(mw))
    wired = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "connect" and node.args):
            sig = ast.unparse(node.func.value)
            slot = ast.unparse(node.args[0])
            wired.add((sig, slot))
    assert ("self.vtol_recon_panel.pick_launch_site_requested", "self.on_pick_home_point") in wired
    assert ("self.vtol_recon_panel.clear_launch_site_requested", "self.on_clear_home_point") in wired
