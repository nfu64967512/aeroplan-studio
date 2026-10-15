"""「清除全部」必須真的把兩張圖清乾淨 —— 值層級的行為鎖。

────────────────────────────────────────────────────────────────────────
案發（2026-08-27）
────────────────────────────────────────────────────────────────────────
使用者按下「清除全部」，log 一路印

    已清除邊界點 / 已清除邊界 / 群飛覆蓋顯示已清除 / 已清除路徑 / 已清除全部

**全部綠燈**，但 2D 還留著琥珀虛線矩形（電子圍籬），3D 還留著黃色圍籬
多邊形 + VTOL 三段軌跡（垂直黃線 + 巡航虛線）。

根因有三層，每一層單獨看都「有實作」：

  1. `MainWindow.on_clear_all_silent()` 只呼叫「清除邊界 + 清除路徑」，
     **從來沒有呼叫過** `map_widget.clear_all()`。
     ⇒「清除全部」在語意上等於「清除邊界」+「清除路徑」。
  2. `DualMapWidget.clear_all()` 第一行是 `self.map_2d.clear_all()`，
     而 2D 的 `MapWidget` **沒有 `clear_all`** ⇒ 就算有人呼叫也是
     AttributeError。這條路徑從來沒被執行過，所以沒人發現。
  3. `CesiumMapWidget.clear_all()` 只清了它自己畫的一部分 entity 陣列；
     `geofenceEntities` / `_vtolEntities` / `_missionEntities` / TOT 閘門 /
     爆炸特效 / 漏掃格心 這幾組**沒有任何 Python 端會去清**。

三條鎖分別對應三層。判準一律是「值」不是「那一行長什麼樣」：
真正去跑那個函式，然後看**輸出**（發出去的 JS、算出來的 folium HTML、
被呼叫到的方法）裡還剩不剩得下那個圖層。

    pytest tests/test_clear_all_wipes_map.py -q
"""

from __future__ import annotations

import os
import re
import types

import pytest

# 無頭環境跑 Qt（沿用 tests/test_vtol_recon_panel.py 的慣例）
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    # 必須先匯入 ui.main_window（它會把 QtWebEngine 載進來），否則 PyQt6 會丟
    # "QtWebEngineWidgets must be imported before a QCoreApplication"。
    from ui.main_window import MainWindow
    from ui.widgets.map_widget import MapWidget
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from mission.geofence_manager import Geofence
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(
    not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}"
)


# ═══════════════════════════════════════════════════════════════════════
#  共用：不進 Qt 建構子的裸實例
#  （建 QWebEngineView 要視窗/GPU，而我們要測的是純邏輯）
# ═══════════════════════════════════════════════════════════════════════
def _bare(cls, **attrs):
    # PyQt6 的 sip 型別不吃 object.__new__（"is not safe"），要用 cls.__new__。
    # 這樣拿到的是「Python 殼建好、C++ 端沒建」的實例：純 Python 的欄位與
    # 方法照跑，只要不去碰 Qt 的東西就好 —— 而我們要測的正好都是純邏輯。
    obj = cls.__new__(cls)
    for k, v in attrs.items():
        setattr(obj, k, v)
    return obj


class _NoDelCesium(CesiumMapWidget):
    """只為了關掉 `__del__`。

    CesiumMapWidget.__del__ 會去讀 `self._cesium_html_file`，而沒跑過 Qt
    建構子的實例一碰就 RuntimeError —— 於是每個測試結束都噴一則
    PytestUnraisableExceptionWarning。噪音會蓋掉真的警告，所以直接關掉。
    被測的仍然是 `CesiumMapWidget.clear_all` 本尊（明確用類別呼叫）。
    """

    def __del__(self):
        pass


def _amber_geofence():
    return Geofence(
        vertices=[(25.070, 121.225), (25.070, 121.240),
                  (25.085, 121.240), (25.085, 121.225)],
        alt_min_m=0.0, alt_max_m=120.0, buffer_radius_m=30.0,
    )


# ═══════════════════════════════════════════════════════════════════════
#  第 3 層：3D —— clear_all 發出去的 JS 必須涵蓋每一組它畫過的 entity
# ═══════════════════════════════════════════════════════════════════════
def _cesium_clear_all_js() -> str:
    """真的跑一次 CesiumMapWidget.clear_all()，把發出去的 JS 全接下來。"""
    sent: list[str] = []
    w = _bare(
        _NoDelCesium,
        corners=[(25.07, 121.23)],
        paths=[[(25.07, 121.23), (25.08, 121.24)]],
        path_colors=['#ff0000'],
        _path_tooltips=['x'],
        transit_paths=[{'path': [(25.07, 121.23), (25.08, 121.24)]}],
        _circle_center=(25.07, 121.23),
        _circle_radius_m=500.0,
        _home_point=(25.07, 121.23),
        _nfz_zones=[{'type': 'circle', 'center': (25.07, 121.23), 'radius': 50}],
        _swarm_data={'drones': []},
        _fw_result={'x': 1},
        _geofence=_amber_geofence(),
        _js=lambda s: sent.append(s),
    )
    CesiumMapWidget.clear_all(w)

    # 順帶鎖住「Python 端的快取也要歸零」——只清畫面不清狀態，
    # 下一次任何重繪都會把同一批東西原地畫回來。
    assert w.corners == [] and w.paths == [] and w.transit_paths == []
    assert w._nfz_zones == []
    assert w._circle_center is None and w._home_point is None
    assert w._swarm_data is None and w._fw_result is None
    assert w._geofence is None, "3D 的 _geofence 沒歸零 ⇒ 圍籬來源還活著"
    return '\n'.join(sent)


# 每一個名字後面都掛著一組**獨立的** entity 陣列；漏一個就是圖上留一層。
_MUST_CLEAR_3D = [
    ('clearCorners',          '邊界角點 cornerEntities / polygonEntity'),
    ('clearPaths',            '飛行路徑 pathEntities / markerEntities'),
    ('clearNFZ',              '禁航區 nfzEntities'),
    ('clearDemOverlay',       'DEM 疊圖 demOverlayEntity'),
    ('clearGeofence',         '電子圍籬 geofenceEntities（2026-08-27 案發主角）'),
    ('vtolClearAll',          'VTOL 三段軌跡 _vtolEntities（2026-08-27 案發主角）'),
    ('clearMissionOverlay',   '飛控讀回的 AUTO 航線 _missionEntities'),
    ('clearTacticalOverlay',  '戰術疊圖 _tacticalEntities'),
    ('clearTotGate',          'TOT 扇形閘門 totGateEntities'),
    ('clearExplosions',       '爆炸特效 explosionEntities'),
    ('clearCoverageGaps',     '漏掃格心 _coverageGapEntities'),
    ('strikeClearAll',        '打擊視覺化 strike*Entities'),
    ('clearAllTerminalDives', '末端俯衝 _terminalDiveEntities'),
]


@pytest.mark.parametrize('fn,layer', _MUST_CLEAR_3D,
                         ids=[f for f, _ in _MUST_CLEAR_3D])
def test_cesium_clear_all_clears_layer(fn, layer):
    js = _cesium_clear_all_js()
    assert re.search(r'\b%s\s*\(' % re.escape(fn), js), (
        f"clear_all 沒清「{layer}」—— 使用者按下清除全部後這層會留在 3D 圖上"
    )


def test_cesium_clear_all_check_can_actually_fail():
    """正對照：這個檢查有鑑別力嗎？

    上面那組 assert 只要「字串出現過」就綠。若 clear_all 發的 JS 裡什麼
    名字都算數，那整組鎖就是永遠不會紅的假綠燈（見 memory:
    instrument-that-cannot-fire）。這裡明確要求一個**不該出現**的名字
    真的不出現。
    """
    js = _cesium_clear_all_js()
    assert not re.search(r'\bclearNoSuchLayer\s*\(', js)


def test_cesium_clear_all_leaves_live_and_user_owned_layers_alone():
    """反向邊界：clear_all **不可以**碰即時 UAV 與使用者建立的圍籬區。

    即時 UAV 有自己的「清除 UAV」按鈕；「管理 Fence」建立的圍籬區是飛安
    設定，有自己的管理介面 —— 被一顆泛用清除鈕靜靜刪掉才是真的災難。
    """
    js = _cesium_clear_all_js()
    for fn in ('clearUAV', 'clearAllFenceZones'):
        assert not re.search(r'\b%s\s*\(' % fn, js), (
            f"clear_all 動了 {fn} —— 這超出「清除標記和路徑」的授權範圍"
        )


# ═══════════════════════════════════════════════════════════════════════
#  第 2 層：2D —— clear_all 存在，而且清完之後 folium 真的不再畫圍籬
# ═══════════════════════════════════════════════════════════════════════
_AMBER = '#FFB703'      # map_widget.py 裡電子圍籬的顏色


def _bare_2d():
    return _bare(
        MapWidget,
        corners=[(25.070, 121.225), (25.070, 121.240), (25.085, 121.240)],
        markers=[1, 2, 3],
        paths=[[(25.07, 121.23), (25.08, 121.24)]],
        path_colors=['#ff0000'],
        _path_tooltips=['x'],
        transit_paths=[{'path': [(25.07, 121.23), (25.08, 121.24)],
                        'altitude': 50.0, 'region_index': 0}],
        _circle_center=(25.075, 121.232),
        _circle_radius_m=500.0,
        _home_point=(25.076, 121.233),
        _nfz_zones=[{'type': 'circle', 'center': (25.07, 121.23),
                     'radius': 50, 'name': 'NFZ_1'}],
        _geofence=_amber_geofence(),
        _swarm_data=None,
        _view_state=None,
        _nfz_poly_draw_mode=False,
        _nfz_poly_vertices=[],
    )


def test_2d_map_widget_has_clear_all_at_all():
    """DualMapWidget.clear_all() 第一行就呼叫它 —— 沒有的話整條路徑是
    AttributeError（而且因為沒人走過，一直沒被發現）。"""
    assert callable(getattr(MapWidget, 'clear_all', None))


def test_2d_render_drops_the_amber_geofence_after_clear_all():
    """值層級：比對**算出來的 folium HTML**，不是比對原始碼。

    先確認清除前真的畫得出琥珀圍籬（否則這條測試量的是空氣），
    再確認清除後那個顏色整個不見了。
    """
    w = _bare_2d()
    before = w._build_map().get_root().render()
    assert _AMBER in before, (
        "前提不成立：清除前就沒畫圍籬 —— 這條測試不可能紅，等於沒鎖"
    )

    rendered = {'n': 0}
    w._render_map = lambda: rendered.__setitem__('n', rendered['n'] + 1)
    w.clear_swarm_display = lambda: None
    w.clear_fc_mission = lambda: None
    MapWidget.clear_all(w)

    after = w._build_map().get_root().render()
    assert _AMBER not in after, "清除全部之後 2D 還畫得出電子圍籬"
    assert rendered['n'] == 1, "clear_all 應該只重繪一次（重繪 = 整頁重載）"


def test_2d_clear_all_drops_every_persistent_field():
    """所有「隨 _render_map 重建」的持久化狀態都要歸零。

    這些欄位的共同特徵是：清了畫面沒用，只要還在欄位裡，下一次任何重繪
    都會把它原封不動畫回來。
    """
    w = _bare_2d()
    w._render_map = lambda: None
    w.clear_swarm_display = lambda: None
    w.clear_fc_mission = lambda: None
    MapWidget.clear_all(w)

    assert w.corners == [] and w.markers == []
    assert w.paths == [] and w.path_colors == [] and w._path_tooltips == []
    assert w.transit_paths == []
    assert w._circle_center is None and w._circle_radius_m == 0.0
    assert w._home_point is None
    assert w._nfz_zones == []
    assert w._geofence is None


def test_2d_clear_all_also_drops_the_js_only_layers():
    """群飛圖層與飛控航線是 JS 直接掛的，不吃 _render_map 的重繪 ——
    必須各自出手清，不能只靠重繪。"""
    called = []
    w = _bare_2d()
    w._render_map = lambda: None
    w.clear_swarm_display = lambda: called.append('swarm')
    w.clear_fc_mission = lambda: called.append('fc')
    MapWidget.clear_all(w)
    assert called == ['swarm', 'fc'] or set(called) == {'swarm', 'fc'}


# ═══════════════════════════════════════════════════════════════════════
#  DualMapWidget —— 兩張圖都要走到，而且圍籬的「來源」要一起斷
# ═══════════════════════════════════════════════════════════════════════
class _FakeMap:
    def __init__(self):
        self.calls = []

    def clear_all(self):
        self.calls.append('clear_all')


def test_dual_clear_all_reaches_both_maps_and_kills_the_fence_source():
    emitted = []
    m2, m3 = _FakeMap(), _FakeMap()
    d = _bare(
        DualMapWidget,
        map_2d=m2, map_3d=m3,
        _last_fence_bundle=object(),        # 圍籬來源還在
        fence_built=types.SimpleNamespace(emit=emitted.append),
    )
    DualMapWidget.clear_all(d)

    assert m2.calls == ['clear_all'], "2D 沒被清"
    assert m3.calls == ['clear_all'], "3D 沒被清"
    assert d._last_fence_bundle is None, (
        "圍籬來源沒斷 —— 一按「顯示圍籬」開關，剛清掉的矩形就原地長回來"
    )
    assert emitted == [None], "面板的圍籬狀態沒被通知 ⇒ 顯示與實際不一致"


# ═══════════════════════════════════════════════════════════════════════
#  第 1 層：MainWindow —— 「清除全部」必須真的走到 map_widget.clear_all()
#  （這是 2026-08-27 那次的直接原因：三行裡沒有這一行）
# ═══════════════════════════════════════════════════════════════════════
class _RecordingMapWidget:
    """只實作 on_clear_* 真的會碰到的方法。

    刻意**不**用萬用 __getattr__：main_window 到處是 `hasattr(...)` 分支，
    萬用代理會讓每個分支都走進去，測出來的路徑跟實機不一樣。
    """

    def __init__(self):
        self.calls = []

    def _rec(self, name):
        self.calls.append(name)

    def clear_corners(self):            self._rec('clear_corners')
    def clear_paths(self):              self._rec('clear_paths')
    def clear_circle_overlay(self):     self._rec('clear_circle_overlay')
    def clear_home_point_overlay(self): self._rec('clear_home_point_overlay')
    def clear_all(self):                self._rec('clear_all')
    def display_nfz_zones(self, zones): self._rec(f'display_nfz_zones:{len(zones)}')


def _bare_main_window(nfz_zones=None):
    panel = types.SimpleNamespace(
        update_corner_count=lambda n: None,
        clear_home_point_display=lambda: None,
    )
    label = types.SimpleNamespace(setText=lambda *_: None)
    return _bare(
        MainWindow,
        map_widget=_RecordingMapWidget(),
        parameter_panel=panel,
        corners=[(25.07, 121.23)],
        waypoints=[(25.07, 121.23, 50.0)],
        spiral_waypoint_altitudes=[50.0],
        waypoint_label=label,
        distance_label=label,
        circle_center=(25.07, 121.23),
        picking_circle_center=True,
        home_point=(25.07, 121.23),
        picking_home_point=True,
        obstacles=[{'x': 1}],
        nfz_zones=list(nfz_zones or []),
        _dccpp_result={'drones': []},
    )


def test_clear_all_silent_actually_calls_map_clear_all():
    mw = _bare_main_window()
    MainWindow.on_clear_all_silent(mw)
    assert 'clear_all' in mw.map_widget.calls, (
        "「清除全部」沒呼叫 map_widget.clear_all() —— 它就只是"
        "「清除邊界 + 清除路徑」，而 log 照樣印「已清除全部」"
    )


def test_clear_all_silent_still_does_what_it_used_to():
    """新增的那一行不可以吃掉原本的行為。"""
    mw = _bare_main_window()
    MainWindow.on_clear_all_silent(mw)
    assert {'clear_corners', 'clear_paths'} <= set(mw.map_widget.calls)
    assert mw.corners == [] and mw.waypoints == [] and mw.obstacles == []
    assert mw.spiral_waypoint_altitudes == []
    assert mw._dccpp_result is None
    assert mw.circle_center is None and mw.home_point is None
    assert mw.picking_circle_center is False and mw.picking_home_point is False


def test_clear_all_silent_keeps_nfz_model_and_view_in_sync():
    """clear_all 會抹掉圖上的 NFZ；模型不同步清，就會留下一份
    畫面上看不見、卻仍參與繞行修正的禁航區。"""
    mw = _bare_main_window(nfz_zones=[{'type': 'circle',
                                       'center': (25.07, 121.23),
                                       'radius': 50, 'name': 'NFZ_1'}])
    MainWindow.on_clear_all_silent(mw)
    assert mw.nfz_zones == [], "模型裡的 NFZ 還在 ⇒ 畫面清了、來源沒清"
    assert 'display_nfz_zones:0' in mw.map_widget.calls


def test_clear_all_silent_does_not_touch_nfz_path_when_empty():
    """沒有 NFZ 時不該白跑一次重繪（on_nfz_changed 會重畫整層）。"""
    mw = _bare_main_window(nfz_zones=[])
    MainWindow.on_clear_all_silent(mw)
    assert not any(c.startswith('display_nfz_zones') for c in mw.map_widget.calls)
