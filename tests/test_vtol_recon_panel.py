"""VTOL 群飛偵察面板的行為測試。

重點不是「畫得漂不漂亮」，而是：
  * 送給 DCCPP 的參數是否真的把 vehicle_type 釘成 vtol
  * 感測器足跡換算有沒有真的驅動 fov_width_m（而不是拿到一個常數）
  * 重疊率不會被重複扣兩次
  * 相機幾何無解時按鈕會被鎖住，而不是送出一個垃圾參數
"""

from __future__ import annotations

import os

import pytest

# 無頭環境跑 Qt
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# 與 tests/test_dccpp_coverage_retention.py 相同慣例：
# 必須先匯入 ui.main_window（它會把 QtWebEngine 載進來），才能建 QApplication，
# 否則 PyQt6 會丟 "QtWebEngineWidgets must be imported ... before a QCoreApplication"。
try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from PyQt6.QtWidgets import QApplication
    _QT_OK = True
except Exception as _e:                             # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(
    not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}"
)


@pytest.fixture(scope="module")
def qt_app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture()
def panel(qt_app):
    return VTOLReconPanel()


# ──────────────────────────────────────────────────────────────
#  參數輸出
# ──────────────────────────────────────────────────────────────

def test_vehicle_type_is_pinned_to_vtol(panel):
    """這一頁只規劃 VTOL —— 不論其他分頁設什麼。"""
    assert panel.get_params()["vehicle_type"] == "vtol"


def test_fov_width_tracks_the_footprint_model(panel):
    """改航高就要改變送出去的條帶間距 —— 若是寫死常數這條會紅。"""
    panel.rb_by_alt.setChecked(True)
    panel.altitude_spin.setValue(100.0)
    low = panel.get_params()["fov_width_m"]
    panel.altitude_spin.setValue(400.0)
    high = panel.get_params()["fov_width_m"]
    assert high > low
    assert high == pytest.approx(4.0 * low, rel=1e-6)


def test_overlap_is_applied_exactly_once(panel):
    """重疊已經吃進 fov_width_m，就不可以再讓 DCCPP 扣一次。"""
    panel.altitude_spin.setValue(200.0)
    panel.overlap_spin.setValue(0.0)
    full = panel.get_params()["fov_width_m"]
    panel.overlap_spin.setValue(0.25)
    p = panel.get_params()
    assert p["fov_width_m"] == pytest.approx(full * 0.75, rel=1e-6)
    assert p["overlap_rate"] == 0.0, "overlap_rate 必須歸零，否則會被重複扣"


def test_gsd_mode_drives_altitude(panel):
    """切到「由目標 GSD 決定航高」時，航高要由 GSD 反算而不是沿用輸入框。"""
    panel.rb_by_gsd.setChecked(True)
    panel.target_gsd_spin.setValue(0.02)
    fine = panel.get_params()["altitude"]
    panel.target_gsd_spin.setValue(0.10)
    coarse = panel.get_params()["altitude"]
    assert coarse > fine, "要求更粗的 GSD 應該可以飛更高"
    # 與輸入框的值無關
    panel.altitude_spin.setValue(4321.0)
    assert panel.get_params()["altitude"] == pytest.approx(coarse)


def test_altitude_mode_ignores_target_gsd(panel):
    panel.rb_by_alt.setChecked(True)
    panel.altitude_spin.setValue(250.0)
    panel.target_gsd_spin.setValue(0.001)
    assert panel.get_params()["altitude"] == pytest.approx(250.0)


def test_transition_alt_is_forwarded(panel):
    panel.transition_alt_spin.setValue(75.0)
    assert panel.get_params()["transition_alt"] == pytest.approx(75.0)


# ──────────────────────────────────────────────────────────────
#  閘門
# ──────────────────────────────────────────────────────────────

def test_plan_button_locked_until_area_is_defined(panel):
    assert not panel.btn_plan.isEnabled()
    panel.set_corner_count(2)
    assert not panel.btn_plan.isEnabled(), "2 點不成區域，不應該可以規劃"
    panel.set_corner_count(4)
    assert panel.btn_plan.isEnabled()


def test_impossible_camera_geometry_locks_the_plan_button(panel):
    """掛載角 + 半個 VFOV 逼近 90° ⇒ 足跡發散，必須擋住而不是送出垃圾。"""
    panel.set_corner_count(4)
    assert panel.btn_plan.isEnabled()
    panel.vfov_spin.setValue(60.0)
    panel.mount_spin.setValue(80.0)       # 80 + 30 = 110 > 85
    assert not panel.btn_plan.isEnabled()
    assert "無解" in panel.footprint_label.text()
    # 調回可行值就要恢復
    panel.mount_spin.setValue(10.0)
    assert panel.btn_plan.isEnabled()


def test_verify_and_export_locked_until_a_plan_exists(panel):
    panel.set_corner_count(4)
    assert not panel.btn_verify.isEnabled()
    assert not panel.btn_export.isEnabled()
    panel.set_plan_result([(1, 120, 8.4, 12.0)], "ok")
    assert panel.btn_verify.isEnabled()
    assert panel.btn_export.isEnabled()


def test_show_missed_locked_until_gaps_are_found(panel):
    from core.recon import verify_coverage
    panel.set_corner_count(4)
    panel.set_plan_result([(1, 10, 1.0, 1.0)], "ok")
    assert not panel.btn_show_missed.isEnabled()

    square = [(25.0, 121.0), (25.0, 121.002), (25.001, 121.002), (25.001, 121.0)]
    gappy = verify_coverage({1: []}, square, sweep_width_m=30.0, cell_size_m=20.0)
    panel.set_coverage_result(gappy)
    assert panel.btn_show_missed.isEnabled(), "有漏掃卻不能標示"


def test_full_coverage_leaves_show_missed_disabled(panel):
    from core.recon import verify_coverage
    panel.set_corner_count(4)
    panel.set_plan_result([(1, 10, 1.0, 1.0)], "ok")
    square = [(25.0, 121.0), (25.0, 121.002), (25.001, 121.002), (25.001, 121.0)]
    path = [(25.0, 121.0 + 0.0002 * i) if i % 2 == 0 else (25.001, 121.0 + 0.0002 * i)
            for i in range(11)]
    rep = verify_coverage({1: path}, square, sweep_width_m=300.0, cell_size_m=20.0)
    assert rep.coverage_ratio == pytest.approx(1.0)
    panel.set_coverage_result(rep)
    assert not panel.btn_show_missed.isEnabled()


# ──────────────────────────────────────────────────────────────
#  顯示
# ──────────────────────────────────────────────────────────────

def test_result_table_is_populated(panel):
    rows = [(1, 120, 8.4, 12.0), (2, 98, 6.1, 9.5)]
    panel.set_plan_result(rows, "done")
    assert panel.result_table.rowCount() == 2
    assert panel.result_table.item(0, 0).text() == "UAV-1"
    assert panel.result_table.item(1, 2).text() == "6.10"
    assert panel.plan_status.text() == "done"


def test_footprint_label_shows_real_numbers(panel):
    panel.rb_by_alt.setChecked(True)
    panel.altitude_spin.setValue(120.0)
    panel.hfov_spin.setValue(60.0)
    txt = panel.footprint_label.text()
    assert "掃描寬度" in txt and "GSD" in txt
    # 120 m、60° HFOV ⇒ 2*120*tan(30°) ≈ 138.6 m
    assert "138.6" in txt, txt


def test_corner_count_message_distinguishes_too_few_from_none(panel):
    panel.set_corner_count(0)
    assert "尚未" in panel.area_label.text()
    panel.set_corner_count(2)
    assert "至少需要 3 點" in panel.area_label.text()
    panel.set_corner_count(5)
    assert "5" in panel.area_label.text()


def test_plan_requested_carries_the_params(panel, qt_app):
    got = []
    panel.plan_requested.connect(got.append)
    panel.set_corner_count(4)
    panel.altitude_spin.setValue(180.0)
    panel._emit_plan()
    assert len(got) == 1
    assert got[0]["vehicle_type"] == "vtol"
    assert got[0]["altitude"] == pytest.approx(180.0)
    assert got[0]["fov_width_m"] > 0


def test_launch_signal_carries_the_drone_count(panel):
    got = []
    panel.launch_sitl_requested.connect(got.append)
    panel.num_drones_spin.setValue(5)
    panel.btn_launch.click()
    assert got == [5]


# ──────────────────────────────────────────────────────────────
#  與 MainWindow 的接線
# ──────────────────────────────────────────────────────────────
# 完整建構 MainWindow 在無頭環境太重（Cesium WebEngine），所以改成檢查
# 「接線用到的每個名字都真的存在」。這抓得到最可能的破法 —— 打錯方法名，
# 因為 pyqtSignal.connect 到一個不存在的屬性會在 __init__ 當場 AttributeError。

@pytest.mark.parametrize("attr", [
    "_on_vtol_recon_plan",
    "_on_vtol_recon_verify",
    "_on_vtol_recon_show_missed",
    "_on_vtol_export_requested",
    "on_sitl_upload_mission",
    "on_clear_corners",
    "on_sitl_launch",
    "_polyline_length_m",
    "_get_cesium_widget",
    "_on_dccpp_coverage_requested",
])
def test_mainwindow_exposes_every_wired_handler(attr):
    assert callable(getattr(MainWindow, attr, None)), (
        f"MainWindow 缺少 {attr} —— VTOL 偵察分頁的接線會在 __init__ 就炸掉"
    )


@pytest.mark.parametrize("attr", ["open_click_map_requested", "_make_resizable_tab"])
def test_parameter_panel_exposes_what_the_tab_registration_uses(attr):
    from ui.widgets.parameter_panel import ParameterPanel
    assert hasattr(ParameterPanel, attr), f"ParameterPanel 缺少 {attr}"


def test_cesium_widget_can_draw_coverage_gaps():
    """_on_vtol_recon_show_missed 會呼叫這個方法。"""
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    assert callable(getattr(CesiumMapWidget, "draw_coverage_gaps", None))
    assert callable(getattr(CesiumMapWidget, "clear_coverage_gaps", None))


def test_cesium_template_defines_the_js_side():
    """Python 端呼叫的 JS 函式必須真的存在於樣板裡，否則只會在 console 靜靜失敗。"""
    from pathlib import Path
    html = (Path(__file__).resolve().parent.parent
            / "ui" / "widgets" / "cesium_templates" / "main.html").read_text(
        encoding="utf-8", errors="replace")
    for fn in ("drawCoverageGaps", "clearCoverageGaps",
               "renderCoverageHeatmap", "_sarRampColor"):
        assert f"function {fn}(" in html, f"main.html 缺少 JS 函式 {fn}"


def test_heatmap_button_needs_a_verification_not_just_a_plan(panel):
    """熱力圖要驗算過才能按；但 100% 覆蓋時仍可按（看得到分布）。"""
    from core.recon import verify_coverage
    panel.set_corner_count(4)
    panel.set_plan_result([(1, 10, 1.0, 1.0)], "ok")
    assert not panel.btn_heatmap.isEnabled(), "還沒驗算就能看熱力圖"

    square = [(25.0, 121.0), (25.0, 121.002), (25.001, 121.002), (25.001, 121.0)]
    path = [(25.0, 121.0 + 0.0002 * i) if i % 2 == 0 else (25.001, 121.0 + 0.0002 * i)
            for i in range(11)]
    full = verify_coverage({1: path}, square, sweep_width_m=300.0, cell_size_m=20.0)
    assert full.coverage_ratio == pytest.approx(1.0)
    panel.set_coverage_result(full)
    assert panel.btn_heatmap.isEnabled(), "100% 覆蓋仍應可看熱力圖"
    assert not panel.btn_show_missed.isEnabled(), "沒有漏掃就不該能標紅點"


def test_replanning_invalidates_the_old_coverage(panel):
    """重新規劃後，舊的驗算結果必須失效，不能還讓人按熱力圖看舊資料。"""
    from core.recon import verify_coverage
    square = [(25.0, 121.0), (25.0, 121.002), (25.001, 121.002), (25.001, 121.0)]
    panel.set_corner_count(4)
    panel.set_plan_result([(1, 10, 1.0, 1.0)], "ok")
    panel.set_coverage_result(
        verify_coverage({1: []}, square, sweep_width_m=30.0, cell_size_m=20.0))
    assert panel.btn_heatmap.isEnabled()

    panel.set_plan_result([(1, 12, 1.2, 1.1)], "replanned")
    assert not panel.btn_heatmap.isEnabled(), "重新規劃後仍能看舊熱力圖"
    assert not panel.btn_show_missed.isEnabled()


def test_mainwindow_has_the_heatmap_handler():
    assert callable(getattr(MainWindow, "_on_vtol_recon_heatmap", None))


def test_cesium_widget_can_render_coverage_heatmap():
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    assert callable(getattr(CesiumMapWidget, "render_coverage_heatmap", None))


def test_corner_count_syncs_live_from_parameter_panel(qt_app, panel):
    """在別的分頁 / 地圖上改邊界點時，這一頁要立刻反應，不必等切分頁。"""
    from ui.widgets.parameter_panel import ParameterPanel
    pp = ParameterPanel()
    pp.corner_count_changed.connect(panel.set_corner_count)

    pp.update_corner_count(2)
    assert not panel.btn_plan.isEnabled()
    pp.update_corner_count(5)
    assert panel.btn_plan.isEnabled(), "加到 5 點卻沒有解鎖規劃"
    pp.update_corner_count(0)
    assert not panel.btn_plan.isEnabled(), "清空邊界點卻仍可規劃"


def test_polyline_length_is_sane():
    """~111 m 的一段緯度位移應該量到約 111 m。"""
    d = MainWindow._polyline_length_m([(25.0, 121.0), (25.001, 121.0)])
    assert d == pytest.approx(111.32, rel=0.02)
    assert MainWindow._polyline_length_m([]) == 0.0
    assert MainWindow._polyline_length_m([(25.0, 121.0)]) == 0.0


def test_cruise_speed_stays_inside_fc_airspeed_band(panel):
    """巡航速度預設與範圍必須落在飛控 [AIRSPEED_MIN 20, AIRSPEED_MAX 34] 內。

    2026-08-25 實測：舊預設 18 < AIRSPEED_MIN 20 ⇒ DO_CHANGE_SPEED 被飛控
    夾到 20，面板顯示的速度與實飛速度**悄悄**不一致。範圍下限也要鎖住，
    否則使用者轉一格就掉回失真帶。
    """
    assert panel.speed_spin.minimum() >= 20.0
    assert panel.speed_spin.maximum() <= 34.0
    assert 20.0 <= panel.speed_spin.value() <= 34.0


def test_cruise_speed_default_matches_get_params(panel):
    """對照組：get_params 送出去的就是 spin 的值（防止另一條路徑寫死舊值）。"""
    assert panel.get_params()["speed"] == panel.speed_spin.value()


# ═════════════════════════════════════════════════════════════════════
#  2026-08-25：VTOL 分頁的 Fence 管理（NFZ / 威脅 / 圍籬）
# ═════════════════════════════════════════════════════════════════════
def test_vtol_panel_exposes_fence_signals(panel):
    """VTOL 分頁要能就地管理 Fence —— 信號名必須與 parameter_panel 相同，
    才能接到 main_window 既有的同一組 handler、共用同一個 registry。"""
    for sig in ('fence_zone_create_requested',
                'fence_zone_draw_polygon_requested',
                'fence_zone_draw_circle_requested',
                'fence_zone_upload_requested',
                'fence_zone_clear_all_requested'):
        assert hasattr(panel, sig), f'缺少 {sig}'


def test_vtol_fence_buttons_emit_signals(panel):
    """按鈕真的接到信號（不是擺著好看）。"""
    fired = []
    panel.fence_zone_create_requested.connect(lambda: fired.append('new'))
    panel.fence_zone_draw_polygon_requested.connect(lambda: fired.append('poly'))
    panel.fence_zone_draw_circle_requested.connect(lambda: fired.append('circle'))
    panel.fence_zone_upload_requested.connect(lambda: fired.append('upload'))
    panel.fence_zone_clear_all_requested.connect(lambda: fired.append('clear'))
    panel.btn_fence_new.click()
    panel.btn_fence_poly.click()
    panel.btn_fence_circle.click()
    panel.btn_fence_upload.click()
    panel.btn_fence_clear.click()
    assert fired == ['new', 'poly', 'circle', 'upload', 'clear'], fired


def test_vtol_fence_count_label_updates(panel):
    """update_fence_zone_count 與 parameter_panel 同簽名，讓 main_window
    的 registry 回呼能一視同仁地更新兩個面板。"""
    panel.update_fence_zone_count(0)
    assert '0' in panel.fence_zone_count_label.text()
    panel.update_fence_zone_count(4, 2)
    assert '4' in panel.fence_zone_count_label.text()


def test_vtol_fence_signal_names_match_parameter_panel():
    """對照組：一旦這兩個面板的 fence 信號名不一致，main_window 就得寫兩套
    handler。鎖住它們同名。"""
    from ui.widgets.parameter_panel import ParameterPanel
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    shared = {'fence_zone_create_requested', 'fence_zone_draw_polygon_requested',
              'fence_zone_draw_circle_requested', 'fence_zone_upload_requested',
              'fence_zone_clear_all_requested'}
    for name in shared:
        assert hasattr(ParameterPanel, name), f'ParameterPanel 缺 {name}'
        assert hasattr(VTOLReconPanel, name), f'VTOLReconPanel 缺 {name}'


# ═════════════════════════════════════════════════════════════════════
#  2026-08-25：刪除上一點（誤點救援）—— 2D 與 3D 必須對稱
# ═════════════════════════════════════════════════════════════════════
def test_vtol_panel_has_delete_last_corner_button(panel):
    fired = []
    panel.delete_last_corner_requested.connect(lambda: fired.append('del'))
    panel.btn_del_last.click()
    assert fired == ['del']


def test_cesium_remove_last_corner_pops_and_refreshes():
    """⭐ 這條就是原本的 3D bug：舊版 CesiumMapWidget **沒有** remove_last_corner，
    刪除流程只動 2D ⇒ 3D 那顆被刪的點還在，兩邊索引開始漂移。"""
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    c = CesiumMapWidget.__new__(CesiumMapWidget)
    c.corners = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
    refreshed = []
    c._refresh_corners = lambda: refreshed.append(1)
    removed = CesiumMapWidget.remove_last_corner(c)
    assert removed == (3.0, 3.0)
    assert c.corners == [(1.0, 1.0), (2.0, 2.0)]
    assert refreshed, '刪了卻沒重繪 ⇒ 3D 畫面上那顆點還在'


def test_cesium_remove_last_corner_on_empty_is_safe():
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    c = CesiumMapWidget.__new__(CesiumMapWidget)
    c.corners = []
    c._refresh_corners = lambda: None
    assert CesiumMapWidget.remove_last_corner(c) is None


def test_dual_map_delete_keeps_2d_and_3d_in_sync():
    """add_corner 是 2D+3D 都加，刪除本來就該對稱 —— 這條鎖住兩邊長度一致。"""
    from ui.widgets.dual_map_widget import DualMapWidget

    class _M2:
        def __init__(self):
            self.corners = [(1.0, 1.0), (2.0, 2.0)]
            self.markers = ['a', 'b']

        def remove_last_corner(self):
            self.markers.pop()
            return self.corners.pop()

    class _M3:
        def __init__(self):
            self.corners = [(1.0, 1.0), (2.0, 2.0)]

        def remove_last_corner(self):
            return self.corners.pop()

    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = _M2(), _M3()
    removed = DualMapWidget.remove_last_corner(d)
    assert removed == (2.0, 2.0)
    assert len(d.map_2d.corners) == len(d.map_3d.corners) == 1, \
        f'2D/3D 角點數不一致：{d.map_2d.corners} vs {d.map_3d.corners}'


def test_dual_map_exposes_markers_proxy():
    """main_window 舊刪除流程存取 map_widget.markers —— 沒有這個代理會
    AttributeError，整個刪除在那一行中斷。"""
    from ui.widgets.dual_map_widget import DualMapWidget
    assert isinstance(DualMapWidget.__dict__.get('markers'), property)


def test_all_three_widgets_expose_remove_last_corner():
    """對照組：三層都要有，缺任何一層就會回到「刪了一半」的狀態。"""
    from ui.widgets.map_widget import MapWidget
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    for cls in (MapWidget, CesiumMapWidget, DualMapWidget):
        assert callable(getattr(cls, 'remove_last_corner', None)), \
            f'{cls.__name__} 缺 remove_last_corner'


# ═════════════════════════════════════════════════════════════════════
#  2026-08-25：地圖上有 fence，⑥ 面板卻顯示 0 個
#  真因 —— 專案有兩套圍籬，面板只數了其中一套
# ═════════════════════════════════════════════════════════════════════
def _fake_bundle(nverts=4, amin=100.0, amax=380.0, buf=60.0, method='aabb'):
    from types import SimpleNamespace
    g = SimpleNamespace(vertices=[(0.0, 0.0)] * nverts, method=method,
                        alt_min_m=amin, alt_max_m=amax, buffer_radius_m=buf)
    return SimpleNamespace(geofence=g)


def test_panel_shows_auto_geofence_separately_from_registry(panel):
    """⭐ 使用者回報的那一幕：規劃完地圖上畫著圍籬，⑥ 卻寫「0 個」。

    registry 計數（手動 fence）與自動建構圍籬是**兩套獨立機制**，面板必須
    兩個都顯示，否則就是在對使用者說謊。
    """
    panel.update_fence_zone_count(0)          # registry 真的是 0
    panel.update_fence_status(_fake_bundle())  # 但自動圍籬存在
    assert '0 個' in panel.fence_zone_count_label.text()
    auto = panel.auto_fence_label.text()
    assert '4 頂點' in auto and '尚未規劃' not in auto, auto


def test_auto_geofence_label_reports_altitude_band(panel):
    """高度範圍要顯示出來 —— 圍籬是飛安設定，只講「有」不夠。"""
    panel.update_fence_status(_fake_bundle(amin=120.0, amax=400.0))
    txt = panel.auto_fence_label.text()
    assert '120' in txt and '400' in txt, txt


def test_auto_geofence_label_resets_when_cleared(panel):
    """對照組：bundle=None 要回到「尚未規劃」，否則清掉路徑後標籤會留著假資訊。"""
    panel.update_fence_status(_fake_bundle())
    panel.update_fence_status(None)
    assert '尚未規劃' in panel.auto_fence_label.text()


def test_update_fence_status_signature_matches_parameter_panel():
    """與 parameter_panel 同名同簽名，main_window 的 fence_built 才能一次餵兩個面板。"""
    from ui.widgets.parameter_panel import ParameterPanel
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    import inspect
    a = inspect.signature(ParameterPanel.update_fence_status)
    b = inspect.signature(VTOLReconPanel.update_fence_status)
    assert list(a.parameters) == list(b.parameters), (a, b)


# ═════════════════════════════════════════════════════════════════════
#  2026-08-25：自動圍籬「浮在天上」—— 建構時漏掉會觸地的航段
# ═════════════════════════════════════════════════════════════════════
_VTOL_SWARM = {'drones': [{
    'drone_id': 1,
    'takeoff_paths': [[(25.0779, 121.2320, 0.0), (25.0779, 121.2320, 40.0),
                       (25.0782, 121.2320, 120.0)]],
    'entry_paths': [[(25.0782, 121.2320, 120.0), (25.0790, 121.2303, 120.0)]],
    'operation_paths': [[(25.0790, 121.2303, 120.0), (25.0790, 121.2346, 120.0)]],
    'transfer_paths': [[(25.0790, 121.2346, 120.0), (25.0785, 121.2346, 120.0)]],
    'landing_paths': [[(25.0782, 121.2320, 120.0), (25.0779, 121.2320, 40.0),
                       (25.0779, 121.2320, 0.0)]],
}]}


def _fence_alts(swarm):
    from ui.widgets.dual_map_widget import DualMapWidget
    from mission.geofence_manager import GeofenceConstraintManager
    paths = DualMapWidget._swarm_raw_to_paths(swarm)
    flat = [(p[0], p[1], p[2]) for path in paths for p in path if len(p) >= 3]
    g = GeofenceConstraintManager(buffer_radius_m=60.0).build(waypoints=flat).geofence
    return g.alt_min_m, g.alt_max_m


def test_fence_collects_ground_touching_segments():
    """起降段是**唯一會碰到地面**的航段，漏掉它圍籬地板就浮在半空。"""
    from ui.widgets.dual_map_widget import DualMapWidget
    paths = DualMapWidget._swarm_raw_to_paths(_VTOL_SWARM)
    alts = [p[2] for path in paths for p in path if len(p) >= 3]
    assert min(alts) == 0.0, f'沒收集到觸地航點，最低只有 {min(alts)}'


def test_fence_floor_reaches_the_ground():
    """⭐ 使用者回報「fence 為何飄在天上」的判準。

    這是 **inclusion** 圍籬：地板 100 m 表示飛機在 100 m 以下都算「在圍籬外」，
    上傳飛控後 FENCE_ACTION 會在飛機**還停在地面時**就判定越界。
    """
    alt_min, _ = _fence_alts(_VTOL_SWARM)
    assert alt_min <= 2.0, f'圍籬地板 {alt_min} m —— 起降段會整段落在圍籬外'


def test_old_behaviour_really_floated():
    """對照組：只餵作業段/轉場段（舊行為）確實會得到浮空地板，
    證明上面那條測得到東西、不是恆真。"""
    only_air = {'drones': [{
        'drone_id': 1,
        'operation_paths': _VTOL_SWARM['drones'][0]['operation_paths'],
        'transfer_paths': _VTOL_SWARM['drones'][0]['transfer_paths'],
    }]}
    alt_min, _ = _fence_alts(only_air)
    assert alt_min > 50.0, f'舊行為應該浮空，實際 {alt_min}'


def test_fence_ceiling_still_covers_the_mission():
    """對照組：修完地板不能把天花板弄低 —— 仍要高過巡航高度。"""
    _, alt_max = _fence_alts(_VTOL_SWARM)
    assert alt_max >= 120.0, alt_max


def test_fence_encloses_every_displayed_segment():
    """⭐ 圍籬的**本職**：把畫出來的每一個航點都包進去。

    只驗高度不夠 —— 起飛段與降落段都觸地，漏掉其中一個，地板仍是 2 m，
    高度檢查看不出來。但 VTOL 的家通常離掃描區有一段距離，漏掉那一段
    會讓圍籬的**水平範圍**縮掉，飛機一起飛就在圍籬外。

    這條同時鎖住高度與水平，漏任何一種航段都會紅。
    """
    from ui.widgets.dual_map_widget import DualMapWidget
    from mission.geofence_manager import GeofenceConstraintManager
    from utils.math_utils import point_in_polygon

    # home 離掃描區 ~500 m（遠大於 buffer 60 m），漏掉起降段就會被抓到
    home_lat, home_lon = 25.0740, 121.2280
    swarm = {'drones': [{
        'drone_id': 1,
        'takeoff_paths': [[(home_lat, home_lon, 0.0), (home_lat, home_lon, 40.0)]],
        'entry_paths': [[(home_lat, home_lon, 40.0), (25.0790, 121.2303, 120.0)]],
        'operation_paths': [[(25.0790, 121.2303, 120.0), (25.0790, 121.2346, 120.0)]],
        'transfer_paths': [[(25.0790, 121.2346, 120.0), (25.0785, 121.2346, 120.0)]],
        'landing_paths': [[(25.0785, 121.2346, 120.0), (home_lat, home_lon, 40.0),
                           (home_lat, home_lon, 0.0)]],
    }]}

    paths = DualMapWidget._swarm_raw_to_paths(swarm)
    flat = [(p[0], p[1], p[2]) for path in paths for p in path if len(p) >= 3]
    g = GeofenceConstraintManager(buffer_radius_m=60.0).build(waypoints=flat).geofence
    poly = [(la, lo) for la, lo in g.vertices]

    # 拿**原始資料的全部五種航段**去驗，不是只驗餵進去的那些
    for d in swarm['drones']:
        for key in ('takeoff_paths', 'entry_paths', 'operation_paths',
                    'transfer_paths', 'landing_paths'):
            for path in d.get(key) or []:
                for pt in path:
                    la, lo, alt = pt[0], pt[1], pt[2]
                    assert point_in_polygon(la, lo, poly), \
                        f'{key} 的點 ({la:.5f},{lo:.5f}) 落在圍籬外'
                    # 天花板要蓋過所有航點。地板不在這裡驗 —— manager 刻意
                    # 保留 2 m「地面淨空」，觸地點 (0 m) 本來就略低於地板，
                    # 那個由 test_fence_floor_reaches_the_ground 負責。
                    assert alt <= g.alt_max_m, (
                        f'{key} 的高度 {alt} 超過圍籬天花板 {g.alt_max_m}')


# ═════════════════════════════════════════════════════════════════════
#  2026-08-25：掃描角度可調（慣例 0°=北、順時針為正）
# ═════════════════════════════════════════════════════════════════════
def test_scan_angle_defaults_to_auto(panel):
    d = panel.get_params()
    assert d['auto_scan_angle'] is True
    assert 'scan_angle_deg' in d, 'get_params 沒送出角度 ⇒ 規劃器只能吃到預設 0'


def test_manual_scan_angle_reaches_get_params(panel):
    panel.auto_angle_check.setChecked(False)
    panel.scan_angle_spin.setValue(45.0)
    d = panel.get_params()
    assert d['auto_scan_angle'] is False
    assert d['scan_angle_deg'] == pytest.approx(45.0)


def test_slider_and_spinbox_stay_in_sync(panel):
    """兩個控制項都能改，改哪個另一個都要跟上，否則送出去的值會跟畫面不一致。"""
    panel.scan_angle_slider.setValue(-30)
    assert panel.get_params()['scan_angle_deg'] == pytest.approx(-30.0, abs=0.05)
    panel.scan_angle_spin.setValue(120.0)
    assert panel.scan_angle_slider.value() == 120


def test_manual_angle_controls_disabled_while_auto(panel):
    """自動選角時鎖住手動控制 —— 否則使用者會以為改了其實沒生效。"""
    panel.auto_angle_check.setChecked(True)
    assert not panel.scan_angle_spin.isEnabled()
    assert not panel.scan_angle_slider.isEnabled()
    panel.auto_angle_check.setChecked(False)
    assert panel.scan_angle_spin.isEnabled()


def test_scan_angle_range_covers_full_circle(panel):
    assert panel.scan_angle_spin.minimum() <= -180.0
    assert panel.scan_angle_spin.maximum() >= 180.0


def test_main_window_puts_scan_angle_into_areas():
    """⚠ 源碼層級妥協（規劃流程在 MainWindow 的巨大方法裡）。

    規劃器在 auto_scan_angle=False 時讀的是 ``area_dict['scan_angle_deg']``；
    舊版組 areas 時**沒有這個鍵** ⇒ 手動角度永遠退回 0°，改了完全沒反應。
    """
    from pathlib import Path
    import ui.main_window as mw
    src = Path(mw.__file__).read_text(encoding='utf-8')
    assert "'scan_angle_deg': scan_angle_deg" in src


def test_scan_angle_actually_changes_strip_geometry():
    """⭐ 真正的判準：角度要**改變條帶幾何**，不是只存進 dict。

    正方形區域下，最佳角（條帶最少）應該與 optimal_scan_angle 一致；
    偏離它條帶數就會變多。
    """
    from core.geometry.polygon import PolygonUtils
    from core.geometry.region_divider import RegionDivider
    poly = [(25.0764, 121.2303), (25.0794, 121.2303),
            (25.0794, 121.2346), (25.0764, 121.2346)]
    counts = {a: len(RegionDivider.decompose_by_fov(poly, a, 110.0, 0.0))
              for a in (0.0, 45.0, 90.0, 135.0)}
    assert len(set(counts.values())) > 1, f'角度沒有改變任何幾何：{counts}'
    best = min(counts, key=counts.get)
    opt = PolygonUtils.optimal_scan_angle(poly)
    assert abs(best - opt) < 1e-6 or abs(abs(best - opt) - 180.0) < 1e-6, \
        f'條帶最少的角 {best}° 與 optimal_scan_angle {opt}° 不一致：{counts}'


# ══════════════════════════════════════════════════════════════
# 掃描角：面板顯示的角度必須等於規劃器實際飛的角度
# ══════════════════════════════════════════════════════════════

def test_manual_angle_reaches_get_params(panel):
    """取消自動 + 設定角度 ⇒ get_params 要同時帶出旗標與角度。"""
    panel.auto_angle_check.setChecked(False)
    panel.scan_angle_spin.setValue(47.0)
    p = panel.get_params()
    assert p["auto_scan_angle"] is False
    assert p["scan_angle_deg"] == pytest.approx(47.0)


def test_manual_controls_locked_while_auto(panel):
    """自動模式下手動控制必須是鎖住的——不然使用者會以為改了其實沒生效。"""
    panel.auto_angle_check.setChecked(True)
    assert not panel.scan_angle_slider.isEnabled()
    assert not panel.scan_angle_spin.isEnabled()
    panel.auto_angle_check.setChecked(False)
    assert panel.scan_angle_slider.isEnabled()
    assert panel.scan_angle_spin.isEnabled()


@pytest.mark.parametrize("raw,expect", [
    (0.0, 0.0), (90.0, 90.0), (180.0, -180.0), (270.0, -90.0), (359.0, -1.0),
])
def test_wrap_scan_angle_keeps_the_direction(panel, raw, expect):
    """compass 0..360 → 數字框 [-180,180]，方向不變只換寫法。"""
    assert panel._wrap_scan_angle(raw) == pytest.approx(expect, abs=1e-9)


def test_unchecking_auto_seeds_from_the_effective_angle(panel):
    """取消自動時要從「上一次實際飛的角度」起跳，不是硬跳 0°。"""
    panel.auto_angle_check.setChecked(True)
    panel.set_effective_scan_angle(270.0, auto=True)   # 規劃器回報自動選了 270°
    panel.auto_angle_check.setChecked(False)
    assert panel.scan_angle_spin.value() == pytest.approx(-90.0, abs=1e-6)
    assert panel.get_params()["scan_angle_deg"] == pytest.approx(-90.0, abs=1e-6)


def test_manual_report_does_not_overwrite_the_auto_hint(panel):
    """手動模式的回報不可污染自動起跳值，否則每規劃一次起跳點就漂一次。"""
    panel.auto_angle_check.setChecked(True)
    panel.set_effective_scan_angle(90.0, auto=True)
    panel.set_effective_scan_angle(137.0, auto=False)   # 手動那趟的回報
    panel.auto_angle_check.setChecked(False)
    assert panel.scan_angle_spin.value() == pytest.approx(90.0, abs=1e-6)


def test_no_report_leaves_the_label_alone(panel):
    """規劃器沒回報（例如回退到舊 solver）時不得亂填數字。"""
    before = panel.effective_angle_label.text()
    panel.set_effective_scan_angle(None)
    assert panel.effective_angle_label.text() == before


# ══════════════════════════════════════════════════════════════
# 避撞:高度錯開量必須真的送出去(以前完全沒送 ⇒ 吃寫死預設 30)
# ══════════════════════════════════════════════════════════════

def test_avoidance_params_are_actually_sent(panel):
    """⚠⚠⚠ 本次 bug 的正主守衛。

    以前 get_params() 只送 min_separation_m,`alt_offset_m` **一個字都沒有**
    ⇒ 規劃器吃寫死預設 30,而面板最小間距預設 50 ⇒ 30 < 50 **無條件成立**
    ⇒ 高度錯開永遠拉不到門檻,每輪 +30 一路棘輪(實測 120→360 m)。
    """
    p = panel.get_params()
    assert "alt_offset_m" in p, "沒送 alt_offset_m ⇒ 規劃器會吃寫死預設"
    assert "max_alt_offset_m" in p, "沒送抬升上限 ⇒ 又變成無界棘輪"
    assert p["alt_offset_m"] == pytest.approx(panel.alt_offset_spin.value())
    assert p["max_alt_offset_m"] == pytest.approx(panel.max_alt_offset_spin.value())


def test_default_offset_is_not_below_default_separation(panel):
    """預設值就不該是「保證解不掉」的組合。"""
    p = panel.get_params()
    assert p["alt_offset_m"] >= p["min_separation_m"], (
        f"預設 alt_offset {p['alt_offset_m']} < 最小間距 {p['min_separation_m']}"
        f" ⇒ 完全重疊的航跡永遠收斂不了")


def test_panel_warns_when_offset_below_separation(panel):
    """錯開量小於間距時要當場示警,而不是等規劃完才在 log 裡講。"""
    panel.min_sep_spin.setValue(50.0)
    panel.alt_offset_spin.setValue(30.0)
    assert panel.avoid_warn_label.text() != "", "危險組合卻沒有示警"
    panel.alt_offset_spin.setValue(50.0)
    assert panel.avoid_warn_label.text() == "", "合法組合卻還在示警(假陽性)"


def test_panel_does_not_silently_coerce_the_offset(panel):
    """⚠ 面板顯示的值必須**就是**送出去的值。

    偷偷把 alt_offset 夾到 min_sep 會製造「顯示的值」與「用的值」兩套,
    那正是本 session 反覆踩到的形狀。只警告,不代改。
    """
    panel.min_sep_spin.setValue(80.0)
    panel.alt_offset_spin.setValue(20.0)
    assert panel.get_params()["alt_offset_m"] == pytest.approx(20.0)
    assert panel.alt_offset_spin.value() == pytest.approx(20.0)


# ══════════════════════════════════════════════════════════════
# 版面:無 emoji + 欄位不擁擠
# ══════════════════════════════════════════════════════════════

def _all_group_titles(panel):
    from PyQt6.QtWidgets import QGroupBox
    return [g.title() for g in panel.findChildren(QGroupBox)]


def test_no_emoji_in_any_visible_text(panel):
    """使用者可見文字不得含 emoji / 圈號等裝飾符號。

    判別法用**碼位範圍**而不是列舉字元 —— 列舉一定會漏掉下一個被加進來的,
    而「漏掉」在畫面上沒有任何錯誤,只是又冒出一個圖案。
    """
    from PyQt6.QtWidgets import (QAbstractButton, QCheckBox, QGroupBox,
                                 QLabel)

    def is_decorative(ch):
        o = ord(ch)
        return (
            0x2460 <= o <= 0x24FF      # 圈號 ①②③
            or 0x2600 <= o <= 0x27BF   # 雜項符號與裝飾 ⚠ ✓ ➤
            or 0x1F000 <= o <= 0x1FAFF  # emoji 本體 🚁🛰
            or 0xFE0F == o              # variation selector
        )

    texts = []
    for w in panel.findChildren(QGroupBox):
        texts.append(('QGroupBox', w.title()))
    for w in panel.findChildren(QLabel):
        texts.append(('QLabel', w.text()))
    for w in panel.findChildren((QAbstractButton, QCheckBox)):
        texts.append(('Button', w.text()))

    bad = [(kind, t, ch) for kind, t in texts for ch in t if is_decorative(ch)]
    assert not bad, f"仍有裝飾符號:{bad[:5]}"


def test_group_numbering_is_sequential_and_matches_display_order(panel):
    """群組編號必須依**顯示順序**連號。

    ⚠ 舊版有兩個 ⑥（覆蓋率驗算 與 Fence），而 Fence 其實排在最後 ——
    編號和實際順序對不上,而且沒有任何東西會報錯。
    """
    import re
    titles = _all_group_titles(panel)
    nums = [int(m.group(1)) for t in titles
            if (m := re.match(r'^(\d+)\.', t))]
    assert nums, f"群組標題沒有編號:{titles}"
    assert nums == sorted(nums), f"編號沒有依顯示順序遞增:{nums}"
    assert len(nums) == len(set(nums)), f"編號重複:{nums}"
    assert nums == list(range(1, len(nums) + 1)), f"編號不連續:{nums}"


@pytest.mark.parametrize("attr", ["alt_offset_spin", "min_sep_spin",
                                  "scan_angle_spin", "altitude_spin"])
def test_fields_are_not_cramped(panel, attr):
    """輸入框之間要有足夠留白 —— 這是使用者直接抱怨的那件事。"""
    from PyQt6.QtWidgets import QFormLayout
    w = getattr(panel, attr, None)
    if w is None:
        pytest.skip(f"{attr} 不存在")
    # ⚠ 不能只用 form.indexOf(w):有些欄位是包在 QHBoxLayout 裡再 addRow 進去的
    #   （例如掃描角度 = 滑桿 + 數字框）。只查直屬子項會誤判成「找不到」,
    #   而那個失敗長得像「版面沒設好」—— 其實是**檢查器的圖樣不夠寬**。
    def _contains(layout, target):
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if item is None:
                continue
            if item.widget() is target:
                return True
            sub = item.layout()
            if sub is not None and _contains(sub, target):
                return True
        return False

    form = None
    for lay in panel.findChildren(QFormLayout):
        if _contains(lay, w):
            form = lay
            break
    assert form is not None, f"找不到 {attr} 所在的 QFormLayout"
    assert form.verticalSpacing() >= 10, (
        f"{attr} 所在表單列距只有 {form.verticalSpacing()} px,太擠")
    assert form.horizontalSpacing() >= 14, (
        f"{attr} 標籤與欄位間距只有 {form.horizontalSpacing()} px,太擠")


def test_root_layout_has_breathing_room(panel):
    """整頁四周與群組之間的留白。"""
    lay = panel.layout()
    m = lay.contentsMargins()
    assert lay.spacing() >= 12, f"群組間距只有 {lay.spacing()} px"
    assert min(m.left(), m.right(), m.top(), m.bottom()) >= 12, \
        f"頁面邊界留白不足:{m.left()},{m.top()},{m.right()},{m.bottom()}"


def test_input_fields_still_stretch_full_width(panel):
    """放寬間距不可以讓輸入框縮成一小截。

    ⚠ ExpandingFieldsGrow 只讓 sizePolicy=Expanding 的欄位長,而 spinbox 不是
    ⇒ 輸入框會變窄。使用者要的是「間距鬆」,不是「欄位窄」——兩者在畫面上
    很容易混淆,所以這條把它釘死。
    """
    from PyQt6.QtWidgets import QFormLayout
    forms = panel.findChildren(QFormLayout)
    assert forms, "找不到任何 QFormLayout"
    for f in forms:
        assert f.fieldGrowthPolicy() == \
            QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow, \
            f"欄位成長策略是 {f.fieldGrowthPolicy()},輸入框會縮成一小截"


def test_zero_cap_means_auto_not_zero(panel):
    """抬升上限填 0 ⇒ 送 None(自動),**不是**送 0。

    ⚠ 送 0 會變成「一公尺都不准抬」⇒ 避撞整個失效,而畫面上只會看到
    「衝突沒解掉」,看不出是被自己的上限鎖死的。
    """
    panel.max_alt_offset_spin.setValue(0.0)
    assert panel.get_params()["max_alt_offset_m"] is None
    panel.max_alt_offset_spin.setValue(150.0)
    assert panel.get_params()["max_alt_offset_m"] == pytest.approx(150.0)


def test_zero_shows_auto_text(panel):
    """0 要顯示成「自動」而不是「0.00 m」—— 不然沒人知道那是特殊值。"""
    panel.max_alt_offset_spin.setValue(0.0)
    assert "自動" in panel.max_alt_offset_spin.specialValueText()
    assert panel.max_alt_offset_spin.text().strip() != "0.00 m"
