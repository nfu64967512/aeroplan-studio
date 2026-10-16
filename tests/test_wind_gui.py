"""風場顯示的 GUI 接線：VTOLReconPanel 的勾選／抓網格列 ＋ MainWindow 的處理函式（不建 MainWindow）。

MainWindow 的方法用 types.MethodType 綁到 FakeWin（與 tests/test_hdmap_gui_mainwindow.py 同一招）；
地圖是記錄呼叫的假物件；不連網（Open-Meteo 的 worker 不啟動，直接餵 _on_wind_grid_done／_failed）。
"""
from __future__ import annotations

import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

from core import wind_field as wf
from ui.main_window import MainWindow
from ui.widgets.vtol_recon_panel import VTOLReconPanel

AREA = [(24.0, 120.0), (24.01, 120.0), (24.01, 120.01), (24.0, 120.01)]


@pytest.fixture(scope="module")
def qt_app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture
def panel(qt_app):
    return VTOLReconPanel()


# ═══════════════════════════════════════════════════════════════════════
#  面板
# ═══════════════════════════════════════════════════════════════════════
def test_panel_defaults_live_on_plan_off_grid_needs_an_area(panel):
    d = panel.get_wind_display()
    assert d == {"plan": False, "live": True, "speed_mps": 0.0, "dir_from_deg": 0.0, "grid_height": "10m"}
    assert panel.btn_wind_grid.isEnabled() is False and panel.btn_wind_grid_clear.isEnabled() is False
    panel.set_corner_count(3)
    assert panel.btn_wind_grid.isEnabled() is True and panel.btn_wind_grid_clear.isEnabled() is False
    panel.set_wind_grid_loaded(True, "網格")
    assert panel.btn_wind_grid_clear.isEnabled() is True and panel.wind_status_label.text() == "網格"
    panel.set_wind_grid_busy(True, "抓取中")
    assert panel.btn_wind_grid.isEnabled() is False and panel.btn_wind_grid_clear.isEnabled() is False
    panel.set_wind_grid_busy(False)
    assert panel.btn_wind_grid.isEnabled() is True and panel.btn_wind_grid_clear.isEnabled() is True
    panel.set_corner_count(0)
    assert panel.btn_wind_grid.isEnabled() is False, "沒作業區不能抓"


def test_panel_emits_display_changes_for_checks_and_planning_wind_values(panel):
    got = []
    panel.wind_display_changed.connect(got.append)
    panel.wind_show_plan_check.setChecked(True)
    panel.hv_wind_speed_spin.setValue(3.0)
    panel.hv_wind_dir_spin.setValue(270.0)
    panel.wind_show_live_check.setChecked(False)
    assert [(g["plan"], g["live"], g["speed_mps"], g["dir_from_deg"]) for g in got] == [
        (True, True, 0.0, 0.0), (True, True, 3.0, 0.0), (True, True, 3.0, 270.0), (True, False, 3.0, 270.0)]


def test_panel_grid_buttons_emit_height_and_clear(panel):
    reqs, clears = [], []
    panel.wind_grid_requested.connect(reqs.append)
    panel.wind_grid_clear_requested.connect(lambda: clears.append(1))
    panel.set_corner_count(3)
    panel.btn_wind_grid.click()
    panel.wind_grid_height_combo.setCurrentIndex(1)
    panel.btn_wind_grid.click()
    panel.set_wind_grid_loaded(True)
    panel.btn_wind_grid_clear.click()
    assert reqs == [{"height": "10m"}, {"height": "120m"}] and clears == [1]
    assert panel.get_wind_display()["grid_height"] == "120m"


# ═══════════════════════════════════════════════════════════════════════
#  MainWindow 處理函式（FakeWin）
# ═══════════════════════════════════════════════════════════════════════
_BOUND = ("_wind_state", "_wind_display", "_wind_limit_mps", "_wind_area_bbox", "_on_wind", "_wind_arrow_rows",
          "_push_wind_arrows", "_refresh_wind_status", "_refresh_wind_field", "_apply_wind_options",
          "_on_wind_display_changed", "_on_wind_grid_requested", "_on_wind_grid_done", "_on_wind_grid_failed",
          "_on_wind_grid_clear", "survey_area_polygons")


class FakeMap:
    def __init__(self):
        self.calls = []

    def set_wind_field(self, f):
        self.calls.append(("field", f))
        return {}

    def clear_wind_field(self):
        self.calls.append(("clear_field", None))
        return True

    def set_wind_arrows(self, rows):
        self.calls.append(("arrows", [dict(r) for r in rows]))
        return {}

    def clear_wind_arrows(self):
        self.calls.append(("clear_arrows", None))
        return True

    def set_wind_options(self, **o):
        self.calls.append(("options", o))
        return o

    def last(self, kind):
        return [c[1] for c in self.calls if c[0] == kind][-1]

    def kinds(self):
        return [c[0] for c in self.calls]


class _Spin:
    def __init__(self, v):
        self._v = v

    def value(self):
        return self._v


class FakePanel:
    def __init__(self, *, plan=False, live=True, speed=3.0, d=270.0, limit=3.0):
        self.disp = {"plan": plan, "live": live, "speed_mps": speed, "dir_from_deg": d, "grid_height": "10m"}
        self.hv_map_max_wind_spin = _Spin(limit)
        self.status, self.busy, self.loaded = [], [], []

    def get_wind_display(self):
        return dict(self.disp)

    def set_wind_status(self, t):
        self.status.append(t)

    def set_wind_grid_busy(self, b, t=""):
        self.busy.append((b, t))

    def set_wind_grid_loaded(self, b, t=""):
        self.loaded.append((b, t))


class FakeStatus:
    def __init__(self):
        self.msgs = []

    def showMessage(self, m, *_a):
        self.msgs.append(m)


class FakeWin:
    def __init__(self, *, areas=(), corners=(), **panel_kw):
        self.survey_areas, self.corners = [list(a) for a in areas], list(corners)
        self.vtol_recon_panel = FakePanel(**panel_kw)
        self.map_widget = FakeMap()
        self._status = FakeStatus()
        self._sitl_links = []
        for name in _BOUND:
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))
        for k in ("WIND_LIVE_PUSH_INTERVAL_S", "WIND_LIVE_STALE_S", "WIND_PLAN_SYSID", "WIND_GRID_MARGIN_M"):
            setattr(self, k, getattr(MainWindow, k))

    def statusBar(self):
        return self._status


def _live(d=272.0, s=3.2, lat=24.005, lon=120.005, t=100.0):
    return {"dir_from_deg": d, "speed_mps": s, "speed_z_mps": 0.0, "lat": lat, "lon": lon, "alt_rel": 90.0, "t": t}


def test_live_wind_is_throttled_and_pushed_as_arrows_with_a_label(monkeypatch):
    w = FakeWin()
    now = [100.0]
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: now[0])
    w._on_wind(1, _live())
    w._on_wind(2, _live(d=10.0, s=1.0, t=100.0))                 # 0.0 s 後：節流，不推
    assert w.map_widget.kinds() == ["arrows"]
    rows = w.map_widget.last("arrows")
    assert [(r["sysid"], r["dir_from_deg"], r["speed_mps"], r["lat"]) for r in rows] == [(1, 272.0, 3.2, 24.005)]
    assert "EKF" in rows[0]["label"]
    now[0] = 100.6
    w._on_wind(1, _live(s=3.4, t=100.6))
    rows = w.map_widget.last("arrows")
    assert [(r["sysid"], r["speed_mps"]) for r in rows] == [(1, 3.4), (2, 1.0)]
    assert w.vtol_recon_panel.status[-1] == "即時風（EKF 估計）：UAV-1 3.4 m/s 來自 272°；UAV-2 1.0 m/s 來自 10°"
    # 太舊的機不畫（UAV-2 最後一筆 100.0 ⇒ 10.5 s 前；UAV-1 100.6 ⇒ 9.9 s 前還算新）；全部太舊 ⇒ 清掉箭頭、狀態列清空
    now[0] = 110.5
    w._push_wind_arrows()
    assert [r["sysid"] for r in w.map_widget.last("arrows")] == [1]
    now[0] = 120.0
    w._push_wind_arrows()
    assert w.map_widget.kinds()[-1] == "clear_arrows" and w.vtol_recon_panel.status[-1] == ""


def test_live_check_off_hides_aircraft_arrows_but_plan_check_adds_a_centroid_arrow(monkeypatch):
    w = FakeWin(areas=[AREA], live=False, plan=True, speed=2.5, d=90.0)
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: 100.0)
    w._on_wind(1, _live())
    rows = w.map_widget.last("arrows")
    assert len(rows) == 1 and rows[0]["sysid"] == MainWindow.WIND_PLAN_SYSID
    assert rows[0]["lat"] == pytest.approx(24.005, abs=1e-9) and rows[0]["lon"] == pytest.approx(120.005, abs=1e-9)
    assert (rows[0]["dir_from_deg"], rows[0]["speed_mps"]) == (90.0, 2.5) and "假設" in rows[0]["label"]
    assert w.vtol_recon_panel.status[-1] == "", "規劃風不算即時風"
    w2 = FakeWin(plan=True, live=True)                            # 勾了規劃風但沒作業區：沒有中心箭頭、不炸
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: 100.0)
    w2._on_wind(3, _live())
    assert [r["sysid"] for r in w2.map_widget.last("arrows")] == [3]


def test_field_prefers_the_grid_then_the_planning_wind_then_nothing():
    w = FakeWin(areas=[AREA], plan=True, speed=3.0, d=270.0, limit=2.0)
    w._on_wind_display_changed({})
    kinds = w.map_widget.kinds()
    assert kinds[:2] == ["options", "field"] and w.map_widget.last("options") == {"limitMps": 2.0}
    f = w.map_widget.last("field")
    assert isinstance(f, wf.WindField) and f.source == "planning" and f.max_speed() == 3.0
    assert f.bbox[0] < 24.0 and f.bbox[2] > 24.01, "均勻場範圍 = 作業區外擴"
    grid = wf.parse_open_meteo([{"latitude": 24.0, "longitude": 120.0, "current": {
        "time": "2026-10-16T02:00", "wind_speed_10m": 1.0, "wind_direction_10m": 180}}] * 4,
        [24.0, 24.1], [120.0, 120.1], "10m")
    w._on_wind_grid_done(grid)
    assert w.map_widget.last("field") is grid
    assert w.vtol_recon_panel.loaded[-1][0] is True and "Open-Meteo" in w.vtol_recon_panel.loaded[-1][1]
    assert w.vtol_recon_panel.status[-1].startswith("Open-Meteo")
    w._on_wind_grid_clear()
    assert w.vtol_recon_panel.loaded[-1] == (False, "") and isinstance(w.map_widget.last("field"), wf.WindField)
    assert w.map_widget.last("field").source == "planning", "清掉網格 ⇒ 退回規劃風"
    w.vtol_recon_panel.disp["plan"] = False
    w._on_wind_display_changed({})
    assert w.map_widget.kinds()[-2:] == ["clear_field", "clear_arrows"]


def test_planning_field_without_an_area_is_reported_not_drawn():
    w = FakeWin(plan=True)
    w._refresh_wind_field()
    assert w.map_widget.kinds() == ["clear_field"] and "作業區" in w._status.msgs[-1]


def test_grid_request_needs_an_area_and_failure_keeps_the_old_grid(monkeypatch):
    w = FakeWin()
    w._on_wind_grid_requested({"height": "10m"})
    assert w.vtol_recon_panel.busy == [(False, "請先圈好作業區（網格範圍 = 作業區外擴 500 m）")]
    started = []

    class FakeWorker:
        def __init__(self, bbox, *, height, parent=None):
            self.bbox, self.height = bbox, height
            self.finished_ok = types.SimpleNamespace(connect=lambda f: None)
            self.failed = types.SimpleNamespace(connect=lambda f: None)

        def isRunning(self):
            return False

        def start(self):
            started.append((self.bbox, self.height))

    import ui.workers.wind_grid_worker as wgw
    monkeypatch.setattr(wgw, "WindGridWorker", FakeWorker)
    w2 = FakeWin(areas=[AREA])
    w2._on_wind_grid_requested({"height": "120m"})
    assert len(started) == 1 and started[0][1] == "120m"
    b = started[0][0]
    assert b[0] < 24.0 < 24.01 < b[2] and b[1] < 120.0 < 120.01 < b[3]
    assert w2.vtol_recon_panel.busy[-1][0] is True and "120m" in w2.vtol_recon_panel.busy[-1][1]
    w2._on_wind_grid_failed("連不到 Open-Meteo")
    assert w2.vtol_recon_panel.busy[-1] == (False, "") and w2.vtol_recon_panel.loaded[-1] == (False, "⚠ 連不到 Open-Meteo")
    assert "連不到" in w2._status.msgs[-1]


def test_limit_falls_back_to_the_first_flight_default_when_unreadable():
    w = FakeWin(limit=0.0)
    assert w._wind_limit_mps() == 3.0
    w.vtol_recon_panel.hv_map_max_wind_spin = None
    assert w._wind_limit_mps() == 3.0
    w.vtol_recon_panel.hv_map_max_wind_spin = _Spin(4.5)
    assert w._wind_limit_mps() == 4.5
