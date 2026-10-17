"""場地預設（config.spawn_presets）：嘉義亞創中心跑道北端／虎尾／桃園機場 —— 值層級的鎖，
以及它接到的三個地方：settings.json 的 map.default_site、VTOL 面板「預設場地」列、MainWindow 的套用處理。

嘉義那一筆的座標來自 OpenStreetMap way 1413073131（aeroway=runway）的北端節點；這裡用跑道**南端**節點
反算長度與方位，鎖住「北端、約 240 m、方位約 189°」這三件事，改錯一個數字就會紅。
"""
from __future__ import annotations

import math
import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from config import spawn_presets as sp

pytest.importorskip("PyQt6")
# ⚠ 要在建 QApplication 之前匯入（map_widget 匯入 QtWebEngineWidgets，之後再匯入會 ImportError）
from ui.main_window import MainWindow                       # noqa: E402
from ui.widgets.vtol_recon_panel import VTOLReconPanel     # noqa: E402

RUNWAY_SOUTH_END = (23.449654, 120.285965)       # OSM way 1413073131 另一端


def _sep_bearing(a, b):
    dN = (b[0] - a[0]) * 111320.0
    dE = (b[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0]))
    return math.hypot(dN, dE), math.degrees(math.atan2(dE, dN)) % 360.0


# ═══════════════════════════════════════════════════════════════════════
#  清單本身
# ═══════════════════════════════════════════════════════════════════════
def test_the_three_sites_in_the_user_order_with_sources():
    assert [p.key for p in sp.SPAWN_PRESETS] == ["CHIAYI_AUIRC", "HUWEI", "TAOYUAN"]
    assert len({p.key for p in sp.SPAWN_PRESETS}) == 3 and len({p.label for p in sp.SPAWN_PRESETS}) == 3
    for p in sp.SPAWN_PRESETS:
        assert p.source.strip(), f"{p.key} 沒有寫座標來源"
        assert 0.0 <= p.heading < 360.0 and -90 <= p.lat <= 90 and -180 <= p.lon <= 180
        assert sp.get_preset_by_key(p.key.lower()) is p and sp.get_preset_by_label(p.label) is p
    assert sp.get_preset_by_key("") is None and sp.get_preset_by_key(None) is None and sp.get_preset_by_key("NOPE") is None
    assert sp.preset_labels()[0].startswith("嘉義 亞創中心跑道北端")


def test_chiayi_point_is_the_north_end_of_the_osm_runway():
    p = sp.get_preset_by_key("CHIAYI_AUIRC")
    assert (p.lat, p.lon) == (23.451785, 120.286324)
    length_m, bearing = _sep_bearing(RUNWAY_SOUTH_END, (p.lat, p.lon))
    assert length_m == pytest.approx(240.0, abs=3.0), "南端→北端應是整條跑道長（OSM 兩端點）"
    assert bearing == pytest.approx(8.8, abs=0.5), "南端→北端方位；北端在北"
    assert p.heading == pytest.approx((bearing + 180.0) % 360.0, abs=0.5), "由北端往南起飛 ⇒ 機頭對準跑道南向"
    assert p.alt_m == 8.0 and "1413073131" in p.source and "亞創" in p.label and "朴子" in p.note


def test_huwei_and_taoyuan_values_are_the_project_defaults_unchanged():
    h, t = sp.get_preset_by_key("HUWEI"), sp.get_preset_by_key("TAOYUAN")
    assert (h.lat, h.lon, h.alt_m, h.heading) == (23.702732, 120.419333, 0.0, 0.0)
    assert (t.lat, t.lon, t.alt_m, t.heading) == (25.0777, 121.2325, 32.0, 52.0)


# ═══════════════════════════════════════════════════════════════════════
#  settings.json 的 map.default_site
# ═══════════════════════════════════════════════════════════════════════
def test_default_site_overrides_map_centre_and_unknown_key_is_loud(tmp_path, capsys):
    from config.settings import GlobalSettings
    f = tmp_path / "settings.json"
    f.write_text('{"map": {"default_site": "chiayi_auirc", "default_lat": 1.0, "default_lon": 2.0}}', encoding="utf-8")
    s = GlobalSettings(config_file=str(f))
    assert (s.map.default_lat, s.map.default_lon) == (23.451785, 120.286324) and s.map_default_site_warning == ""
    f.write_text('{"map": {"default_site": "MARS", "default_lat": 1.0, "default_lon": 2.0}}', encoding="utf-8")
    s = GlobalSettings(config_file=str(f))
    assert (s.map.default_lat, s.map.default_lon) == (1.0, 2.0), "不認得的 key 不可以動座標"
    assert "MARS" in s.map_default_site_warning and "CHIAYI_AUIRC" in s.map_default_site_warning
    assert "MARS" in capsys.readouterr().out
    f.write_text('{"map": {"default_lat": 1.0, "default_lon": 2.0}}', encoding="utf-8")
    s = GlobalSettings(config_file=str(f))
    assert (s.map.default_lat, s.map.default_lon) == (1.0, 2.0) and s.map.default_site == ""
    assert s.map_default_site_warning == ""
    s = GlobalSettings(config_file=str(tmp_path / "none.json"))
    assert s.map.default_site == "" and s.map_default_site_warning == ""
    f.write_text('{"export": {}}', encoding="utf-8")                       # 沒有 map 區塊也要有這個屬性
    assert GlobalSettings(config_file=str(f)).map_default_site_warning == ""


def test_default_site_does_not_eat_the_users_own_coordinates_on_save(tmp_path, capsys):
    """有效值契約：default_site 生效時 save() 寫回的仍是使用者手填的 default_lat/lon，清空 default_site 就拿回原值。"""
    import json
    from config.settings import GlobalSettings
    f = tmp_path / "settings.json"
    f.write_text('{"map": {"default_site": "CHIAYI_AUIRC", "default_lat": 23.702732, "default_lon": 120.419333}}',
                 encoding="utf-8")
    s = GlobalSettings(config_file=str(f))
    assert (s.map.default_lat, s.map.default_lon) == (23.451785, 120.286324)
    assert "保留不改" in capsys.readouterr().out, "取代要說出來，不能靜默"
    assert s.save() is True
    on_disk = json.loads(f.read_text(encoding="utf-8"))["map"]
    assert (on_disk["default_lat"], on_disk["default_lon"], on_disk["default_site"]) == (23.702732, 120.419333, "CHIAYI_AUIRC")
    assert s.get_dict()["map"]["default_lat"] == 23.702732
    on_disk["default_site"] = ""
    f.write_text(json.dumps({"map": on_disk}), encoding="utf-8")
    s2 = GlobalSettings(config_file=str(f))
    assert (s2.map.default_lat, s2.map.default_lon) == (23.702732, 120.419333), "清空 default_site 要拿回手填值"
    s2.save()
    assert json.loads(f.read_text(encoding="utf-8"))["map"]["default_lat"] == 23.702732


# ═══════════════════════════════════════════════════════════════════════
#  面板與 MainWindow
# ═══════════════════════════════════════════════════════════════════════
@pytest.fixture(scope="module")
def qt_app():
    from PyQt6.QtWidgets import QApplication
    yield QApplication.instance() or QApplication([])


def test_panel_lists_the_sites_and_emits_the_key_on_apply(qt_app):
    panel = VTOLReconPanel()
    combo = panel.site_preset_combo
    assert [combo.itemData(i) for i in range(combo.count())] == ["CHIAYI_AUIRC", "HUWEI", "TAOYUAN"]
    assert combo.itemText(0).startswith("嘉義 亞創中心跑道北端") and "23.45179" in combo.itemText(0)
    got = []
    panel.launch_site_preset_requested.connect(got.append)
    panel.btn_apply_site_preset.click()
    combo.setCurrentIndex(2)
    panel.btn_apply_site_preset.click()
    assert got == ["CHIAYI_AUIRC", "TAOYUAN"]


class _Status:
    def __init__(self):
        self.msgs = []

    def showMessage(self, m, *_a):
        self.msgs.append(m)


class _Map:
    def __init__(self):
        self.flights = []

    def fly_to_position(self, lat, lon, alt=0.0, range_m=600.0):
        self.flights.append((lat, lon, alt, range_m))


def _win(monkeypatch, *, map_widget=None):
    w = types.SimpleNamespace()
    w.home_point = None
    w.set_calls = []
    w.on_home_point_set = lambda lat, lon: (w.set_calls.append((lat, lon)), setattr(w, "home_point", (lat, lon)))
    w.map_widget = _Map() if map_widget is None else map_widget
    w._status = _Status()
    w.statusBar = lambda: w._status
    w.SITE_PRESET_VIEW_RANGE_M = MainWindow.SITE_PRESET_VIEW_RANGE_M
    w._on_launch_site_preset = types.MethodType(MainWindow._on_launch_site_preset, w)
    import config.settings as cs
    fake = types.SimpleNamespace(sitl_launch=types.SimpleNamespace(home_preset=""))
    monkeypatch.setattr(cs, "get_settings", lambda: fake)
    w._settings = fake
    return w


def test_applying_a_site_sets_home_flies_the_map_and_syncs_the_sitl_preset(monkeypatch):
    w = _win(monkeypatch)
    w._on_launch_site_preset("CHIAYI_AUIRC")
    assert w.set_calls == [(23.451785, 120.286324)] and w.home_point == (23.451785, 120.286324)
    assert w.map_widget.flights == [(23.451785, 120.286324, 0.0, 1500.0)]
    assert w._settings.sitl_launch.home_preset == "CHIAYI_AUIRC"
    assert "亞創中心" in w._status.msgs[-1] and "23.451785" in w._status.msgs[-1]


def test_unknown_site_changes_nothing_and_says_so(monkeypatch):
    w = _win(monkeypatch)
    w._on_launch_site_preset("MOON")
    assert w.set_calls == [] and w.home_point is None and w.map_widget.flights == []
    assert w._settings.sitl_launch.home_preset == "" and "MOON" in w._status.msgs[-1] and "未變" in w._status.msgs[-1]


def test_map_without_fly_to_or_a_failing_fly_still_sets_the_home_and_says_the_map_did_not_move(monkeypatch):
    w = _win(monkeypatch, map_widget=object())
    w._on_launch_site_preset("HUWEI")
    assert w.home_point == (23.702732, 120.419333) and "地圖未移動" in w._status.msgs[-1]

    class _Boom:
        def fly_to_position(self, *a):
            raise RuntimeError("no page")

    w2 = _win(monkeypatch, map_widget=_Boom())
    w2._on_launch_site_preset("TAOYUAN")
    assert w2.home_point == (25.0777, 121.2325) and w2._settings.sitl_launch.home_preset == "TAOYUAN"
    assert "地圖未移動" in w2._status.msgs[-1]

    class _NoMove:
        def fly_to_position(self, *a):
            return False

    w3 = _win(monkeypatch, map_widget=_NoMove())
    w3._on_launch_site_preset("TAOYUAN")
    assert "地圖未移動" in w3._status.msgs[-1]
    w4 = _win(monkeypatch)                                     # _Map 回 None ⇒ 也算沒動（舊介面）
    w4._on_launch_site_preset("TAOYUAN")
    assert "地圖未移動" in w4._status.msgs[-1]


class _TrueMap(_Map):
    def fly_to_position(self, lat, lon, alt=0.0, range_m=600.0):
        super().fly_to_position(lat, lon, alt, range_m)
        return True


def test_a_map_that_really_moves_is_not_flagged(monkeypatch):
    w = _win(monkeypatch, map_widget=_TrueMap())
    w._on_launch_site_preset("CHIAYI_AUIRC")
    assert "地圖未移動" not in w._status.msgs[-1] and w.map_widget.flights[0][:2] == (23.451785, 120.286324)


# ═══════════════════════════════════════════════════════════════════════
#  DualMapWidget.fly_to_position 的 2D 分支（裸實例）
# ═══════════════════════════════════════════════════════════════════════
def test_dual_map_fly_to_moves_the_2d_map_too():
    from ui.widgets.dual_map_widget import DualMapWidget, _MODE_2D, _MODE_3D

    class _Page:
        def __init__(self):
            self.js = []

        def runJavaScript(self, s, *a):
            self.js.append(s)

    m2 = types.SimpleNamespace(custom_page=_Page(), _view_state=None)
    m3 = types.SimpleNamespace(flights=[])
    m3.fly_to_position = lambda *a: m3.flights.append(a)
    d = DualMapWidget.__new__(DualMapWidget)
    d.__dict__.update(_mode=_MODE_2D, map_2d=m2, map_3d=m3)
    assert DualMapWidget.fly_to_position(d, 23.451785, 120.286324, 0.0, 1500.0) is True
    assert m2._view_state == {"lat": 23.451785, "lon": 120.286324, "zoom": 16}
    assert m2.custom_page.js == ["if (window.currentMap) { window.currentMap.setView([23.451785, 120.286324], 16); }"]
    assert m3.flights == [], "2D 模式不碰 3D"
    assert DualMapWidget.fly_to_position(d, 25.0, 121.0, 0.0, 50000.0) is True and m2._view_state["zoom"] == 12
    m2b = types.SimpleNamespace(_view_state=None)                    # 頁面還沒建：只寫 _view_state
    d.__dict__.update(map_2d=m2b)
    assert DualMapWidget.fly_to_position(d, 1.0, 2.0) is True and m2b._view_state["zoom"] == 16
    d.__dict__.update(_mode=_MODE_3D)
    assert DualMapWidget.fly_to_position(d, 1.0, 2.0, 0.0, 600.0) is True and m3.flights == [(1.0, 2.0, 0.0, 600.0)]


# ═══════════════════════════════════════════════════════════════════════
#  SITL 出生點預設跟著起降場走：地圖點選／清除 ⇒ 回「自動」
# ═══════════════════════════════════════════════════════════════════════
def test_picking_or_clearing_the_home_resets_the_sitl_spawn_preset(monkeypatch):
    w = _win(monkeypatch, map_widget=_TrueMap())
    w.parameter_panel = types.SimpleNamespace(set_home_point_display=lambda *a: None, clear_home_point_display=lambda: None)
    w._sync_launch_site_display = lambda: None
    w.map_widget.set_home_point_overlay = lambda *a: None
    w.map_widget.clear_home_point_overlay = lambda: None
    w.picking_home_point = True
    for name in ("on_home_point_set", "on_clear_home_point", "_reset_sitl_home_preset"):
        setattr(w, name, types.MethodType(getattr(MainWindow, name), w))
    w._on_launch_site_preset("CHIAYI_AUIRC")
    assert w._settings.sitl_launch.home_preset == "CHIAYI_AUIRC" and w.home_point == (23.451785, 120.286324)
    w.on_home_point_set(24.0, 120.0)                             # 使用者在地圖上另點一處
    assert w._settings.sitl_launch.home_preset == "" and w.home_point == (24.0, 120.0) and w.picking_home_point is False
    w._on_launch_site_preset("HUWEI")
    assert w._settings.sitl_launch.home_preset == "HUWEI"
    w.on_clear_home_point()
    assert w._settings.sitl_launch.home_preset == "" and w.home_point is None
