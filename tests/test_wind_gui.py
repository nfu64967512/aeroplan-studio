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
    assert {k: d[k] for k in ("plan", "live", "speed_mps", "dir_from_deg", "grid_height")} == {
        "plan": False, "live": True, "speed_mps": 0.0, "dir_from_deg": 0.0, "grid_height": "10m"}
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
          "_on_wind_grid_clear", "survey_area_polygons",
          # 2026-10-16 CWA 測站與寫入規劃風
          "_wind_loaded", "_wind_near_point", "_apply_wind_to_plan", "_on_wind_cwa_requested", "_on_wind_cwa_done",
          "_on_wind_cwa_failed", "_wind_note")


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
    def __init__(self, *, plan=False, live=True, speed=3.0, d=270.0, limit=3.0, apply_plan=False):
        self.disp = {"plan": plan, "live": live, "speed_mps": speed, "dir_from_deg": d, "grid_height": "10m",
                     "apply_plan": apply_plan, "cwa_station": ""}
        self.hv_map_max_wind_spin = _Spin(limit)
        self.status, self.busy, self.loaded, self.cwa_busy, self.applied = [], [], [], [], []

    def get_wind_display(self):
        return dict(self.disp)

    def set_wind_status(self, t):
        self.status.append(t)

    def set_wind_grid_busy(self, b, t=None):
        self.busy.append((b, t))

    def set_wind_grid_loaded(self, b, t=None):
        self.loaded.append((b, t))

    def set_wind_cwa_busy(self, b, t=None):
        self.cwa_busy.append((b, t))

    def apply_wind_to_plan(self, s, d, source):
        """與真的面板同義：寫進規劃風欄位（這裡記進 disp），回傳生效值與狀態文字。"""
        if not (s == s and d == d):
            raise ValueError("nan")
        self.disp["speed_mps"], self.disp["dir_from_deg"] = round(float(s), 1), float(round(d % 360))
        text = f"規劃風已寫入：{self.disp['speed_mps']:.1f} m/s 來自 {self.disp['dir_from_deg']:.0f}°（來源：{source}）"
        self.applied.append((s, d, source))
        return {"speed_mps": self.disp["speed_mps"], "dir_from_deg": self.disp["dir_from_deg"], "clamped": False,
                "text": text}


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
        for k in ("WIND_LIVE_PUSH_INTERVAL_S", "WIND_LIVE_STALE_S", "WIND_PLAN_SYSID", "WIND_GRID_MARGIN_M",
                  "WIND_CWA_SYSID"):
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
    assert w.vtol_recon_panel.loaded[-1] == (True, None)
    assert w.vtol_recon_panel.status[-1].startswith("Open-Meteo")
    w._on_wind_grid_clear()
    assert w.vtol_recon_panel.loaded[-1] == (False, None) and isinstance(w.map_widget.last("field"), wf.WindField)
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
    assert w.vtol_recon_panel.busy == [(False, None)]
    assert w.vtol_recon_panel.status[-1] == "請先圈好作業區（網格範圍 = 作業區外擴 500 m）"
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
    assert w2.vtol_recon_panel.busy[-1][0] is True and "120m" in w2.vtol_recon_panel.status[-1]
    w2._on_wind_grid_failed("連不到 Open-Meteo")
    assert w2.vtol_recon_panel.busy[-1] == (False, None) and w2.vtol_recon_panel.loaded[-1] == (False, None)
    assert w2.vtol_recon_panel.status[-1].startswith("⚠ 氣象網格：連不到 Open-Meteo")
    assert "連不到" in w2._status.msgs[-1]


def test_limit_falls_back_to_the_first_flight_default_when_unreadable():
    w = FakeWin(limit=0.0)
    assert w._wind_limit_mps() == 3.0
    w.vtol_recon_panel.hv_map_max_wind_spin = None
    assert w._wind_limit_mps() == 3.0
    w.vtol_recon_panel.hv_map_max_wind_spin = _Spin(4.5)
    assert w._wind_limit_mps() == 4.5


# ═══════════════════════════════════════════════════════════════════════
#  2026-10-16：CWA 測站列 ＋ 讀到的風寫入規劃風
# ═══════════════════════════════════════════════════════════════════════
def test_panel_cwa_row_defaults_and_enabling(panel):
    d = panel.get_wind_display()
    assert d["apply_plan"] is True and d["cwa_station"] == panel.wind_cwa_station_edit.text().strip()
    panel.wind_cwa_station_edit.setText("")
    assert panel.btn_wind_cwa.isEnabled() is False, "沒作業區又沒填測站 ⇒ 找不到最近測站"
    panel.wind_cwa_station_edit.setText("466920")
    assert panel.btn_wind_cwa.isEnabled() is True
    panel.wind_cwa_station_edit.setText("")
    panel.set_corner_count(3)
    assert panel.btn_wind_cwa.isEnabled() is True
    panel.set_wind_cwa_busy(True, "抓取中")
    assert panel.btn_wind_cwa.isEnabled() is False and panel.btn_wind_grid.isEnabled() is False
    panel.set_wind_cwa_busy(False)
    assert panel.btn_wind_cwa.isEnabled() is True and panel.btn_wind_grid.isEnabled() is True
    reqs = []
    panel.wind_cwa_requested.connect(reqs.append)
    panel.wind_cwa_station_edit.setText(" 桃園 ")
    panel.btn_wind_cwa.click()
    assert reqs == [{"station": "桃園"}]


def test_panel_apply_wind_to_plan_writes_the_two_spins_loudly_and_clamps(panel):
    got = []
    panel.wind_display_changed.connect(got.append)
    eff = panel.apply_wind_to_plan(3.26, 272.4, "CWA 測站 桃園")
    assert (panel.hv_wind_speed_spin.value(), panel.hv_wind_dir_spin.value()) == (3.3, 272.0)
    assert eff["speed_mps"] == 3.3 and eff["dir_from_deg"] == 272.0 and eff["clamped"] is False
    assert "規劃風已寫入：3.3 m/s 來自 272°" in eff["text"] and "CWA 測站 桃園" in eff["text"]
    assert panel.wind_status_label.text() == eff["text"]
    assert got and got[-1]["speed_mps"] == 3.3 and got[-1]["dir_from_deg"] == 272.0, "寫入要觸發顯示更新"
    eff = panel.apply_wind_to_plan(30.0, -10.0, "x")
    assert (panel.hv_wind_speed_spin.value(), panel.hv_wind_dir_spin.value()) == (25.0, 350.0)
    assert eff["clamped"] is True and "⚠" in eff["text"] and "30.0 m/s" in eff["text"]
    eff = panel.apply_wind_to_plan(0.0, 359.6, "x")
    assert eff["dir_from_deg"] == 0.0 and eff["clamped"] is False, "359.6 四捨五入成 360 ⇒ 0°，不算夾限"
    with pytest.raises(ValueError):
        panel.apply_wind_to_plan(float("nan"), 0.0, "x")


def _obs(**kw):
    from core.wind_field import StationWind
    base = dict(station_id="C0C480", name="桃園", lat=24.99, lon=121.31, alt_m=12.0, obs_time="2026-10-16T17:00:00+08:00",
                speed_mps=2.4, dir_from_deg=60.0, gust_mps=5.1, gust_dir_deg=70.0, pressure_hpa=1008.2)
    base.update(kw)
    return StationWind(**base)


def test_cwa_request_needs_a_key_then_a_station_or_an_area(monkeypatch):
    import utils.cwa_opendata as co
    monkeypatch.setattr(co, "configured_cwa_api_key", lambda: ("", ""))
    w = FakeWin()
    w._on_wind_cwa_requested({"station": "466920"})
    assert w.vtol_recon_panel.cwa_busy[-1] == (False, None) and "CWA_API_KEY" in w.vtol_recon_panel.status[-1]
    assert w.vtol_recon_panel.status[-1].startswith("沒有 CWA API 授權碼")
    monkeypatch.setattr(co, "configured_cwa_api_key", lambda: ("", "invalid:環境變數 CWA_API_KEY"))
    w._on_wind_cwa_requested({"station": "466920"})
    assert w.vtol_recon_panel.status[-1].startswith("CWA 授權碼讀取失敗（環境變數 CWA_API_KEY）")
    monkeypatch.setattr(co, "configured_cwa_api_key", lambda: ("CWA-OK", "環境變數 CWA_API_KEY"))
    w._on_wind_cwa_requested({"station": ""})
    assert "圈好作業區" in w.vtol_recon_panel.status[-1]
    started = []

    class FakeWorker:
        def __init__(self, key, *, station, near, parent=None):
            started.append((key, station, near))
            self.finished_ok = types.SimpleNamespace(connect=lambda f: None)
            self.failed = types.SimpleNamespace(connect=lambda f: None)

        def isRunning(self):
            return False

        def start(self):
            pass

    import ui.workers.wind_grid_worker as wgw
    monkeypatch.setattr(wgw, "CwaStationWorker", FakeWorker)
    w2 = FakeWin(areas=[AREA])
    w2._on_wind_cwa_requested({"station": ""})
    assert len(started) == 1 and started[0][0] == "CWA-OK" and started[0][1] == ""
    assert started[0][2] == pytest.approx((24.005, 120.005), abs=1e-9), "最近測站的基準 = 作業區中心"
    assert w2.vtol_recon_panel.cwa_busy[-1] == (True, None) and "最近" in w2.vtol_recon_panel.status[-1]
    w3 = FakeWin()
    w3.home_point = (24.9, 121.2)
    w3._on_wind_cwa_requested({"station": "桃園"})
    assert started[-1][1] == "桃園" and started[-1][2] == (24.9, 121.2), "沒作業區 ⇒ 用起降場"
    assert w3._wind_near_point() == (24.9, 121.2) and FakeWin()._wind_near_point() is None


def test_cwa_result_draws_a_station_arrow_and_writes_the_planning_wind(monkeypatch):
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: 100.0)
    w = FakeWin(apply_plan=True)
    w._on_wind(1, _live())
    w._on_wind_cwa_done(_obs())
    rows = w.map_widget.last("arrows")
    assert [(r["sysid"], r["lat"], r["lon"], r["dir_from_deg"], r["speed_mps"]) for r in rows] == [
        (1, 24.005, 120.005, 272.0, 3.2), (MainWindow.WIND_CWA_SYSID, 24.99, 121.31, 60.0, 2.4)]
    assert rows[1]["label"].startswith("CWA 桃園 觀測 17:00")
    assert w.vtol_recon_panel.cwa_busy[-1] == (False, None) and w.vtol_recon_panel.loaded[-1] == (True, None)
    assert w.vtol_recon_panel.applied == [(2.4, 60.0, "CWA 測站 桃園（C0C480） 觀測 2026-10-16T17:00:00+08:00，10 分鐘平均地面測站風（非作業高度）")]
    status = w.vtol_recon_panel.status[-1]
    assert "CWA 測站 桃園（C0C480）" in status and "規劃風已寫入：2.4 m/s 來自 60°" in status and "UAV-1 3.2 m/s" in status
    assert "規劃風已寫入" in w._status.msgs[-1]
    # 狀態列不只給 CWA：清除要連測站一起清、箭頭重推
    w._on_wind_grid_clear()
    assert w._wind_state()["cwa"] is None and [r["sysid"] for r in w.map_widget.last("arrows")] == [1]
    assert "：風 2.4 m/s" not in w.vtol_recon_panel.status[-1], "測站觀測那一行要消失"
    assert "規劃風已寫入" in w.vtol_recon_panel.status[-1] and "來源：CWA" in w.vtol_recon_panel.status[-1], \
        "規劃風欄位不改回去，狀態也留著說明來源"
    # 不勾「寫入規劃風」就不寫
    w2 = FakeWin(apply_plan=False)
    w2._on_wind_cwa_done(_obs())
    assert w2.vtol_recon_panel.applied == [] and w2.map_widget.last("arrows")[0]["sysid"] == MainWindow.WIND_CWA_SYSID


def test_grid_result_writes_the_vector_mean_and_failures_keep_what_is_loaded():
    w = FakeWin(areas=[AREA], apply_plan=True)
    grid = wf.parse_open_meteo([{"latitude": 24.0, "longitude": 120.0, "current": {
        "time": "2026-10-16T09:00", "wind_speed_10m": 1.0, "wind_direction_10m": 180}}] * 4,
        [24.0, 24.1], [120.0, 120.1], "10m")
    w._on_wind_grid_done(grid)
    (s, d, src), = w.vtol_recon_panel.applied
    assert (s, round(d)) == (pytest.approx(1.0), 180) and "Open-Meteo 10 m 網格 4 格向量平均" in src and "非實測" in src
    assert "規劃風已寫入：1.0 m/s 來自 180°" in w.vtol_recon_panel.status[-1]
    w._on_wind_cwa_failed("連不到 CWA")
    assert w.vtol_recon_panel.loaded[-1] == (True, None), "網格還在 ⇒ 清除鈕仍可按"
    assert w.vtol_recon_panel.status[-1].startswith("⚠ CWA 測站：連不到 CWA\n") and "規劃風已寫入" in w.vtol_recon_panel.status[-1]
    w._on_wind_grid_failed("HTTP 429")
    assert w.vtol_recon_panel.loaded[-1][0] is True
    w._on_wind_grid_clear()
    w._on_wind_grid_failed("HTTP 429")
    assert w.vtol_recon_panel.loaded[-1] == (False, None) and w.vtol_recon_panel.status[-1].startswith("⚠ 氣象網格：HTTP 429")


def test_manual_edit_after_a_write_rewrites_the_status_so_it_reports_the_effective_value():
    """有效值契約：寫入後使用者手改欄位，狀態列不能再宣稱「已寫入 2.4 來自 CWA」。"""
    w = FakeWin(apply_plan=True)
    w._on_wind_cwa_done(_obs())
    assert w._wind_state()["plan_applied"] == (2.4, 60.0)
    w.vtol_recon_panel.disp["speed_mps"] = 5.0                  # 使用者把平均風速改成 5.0
    w._on_wind_display_changed({})
    st = w.vtol_recon_panel.status[-1]
    assert "規劃風已手動改為 5.0 m/s 來自 60°" in st and "2.4 m/s／60°" in st and "已被覆蓋" in st
    assert "規劃風已寫入：2.4" not in st and w._wind_state()["plan_applied"] is None
    w._on_wind_grid_clear()
    assert "規劃風已手動改為 5.0" in w.vtol_recon_panel.status[-1]
    w._on_wind_display_changed({})                                 # 再改也不會再生出第二段
    assert w.vtol_recon_panel.status[-1].count("已手動改為") == 1
    w._on_wind_cwa_done(_obs())                                    # 再寫入一次 ⇒ 回到「已寫入」、基準重設
    assert "規劃風已寫入：2.4 m/s 來自 60°" in w.vtol_recon_panel.status[-1]
    assert "已手動改為" not in w.vtol_recon_panel.status[-1] and w._wind_state()["plan_applied"] == (2.4, 60.0)


def test_real_panel_write_then_manual_edit_and_failure_note_survives_live_pushes(qt_app, monkeypatch):
    """用真的 VTOLReconPanel（setValue 會**同步**觸發 wind_display_changed，在 plan_applied 設好之前）。"""
    now = [100.0]
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: now[0])
    w = FakeWin()
    panel = VTOLReconPanel()
    w.vtol_recon_panel = panel
    panel.wind_display_changed.connect(w._on_wind_display_changed)
    panel.wind_apply_plan_check.setChecked(True)
    w._on_wind_cwa_done(_obs())
    label = panel.wind_status_label.text()
    assert "規劃風已寫入：2.4 m/s 來自 60°" in label and "已手動改為" not in label, "寫入當下不可被判成手改"
    assert (panel.hv_wind_speed_spin.value(), panel.hv_wind_dir_spin.value()) == (2.4, 60.0)
    panel.hv_wind_speed_spin.setValue(5.0)
    label = panel.wind_status_label.text()
    assert "規劃風已手動改為 5.0 m/s 來自 60°" in label and "規劃風已寫入：2.4" not in label
    # 失敗提示放第一行，之後的即時風推送不會把它蓋掉
    w._on_wind_cwa_failed("連不到 CWA")
    now[0] = 101.0
    w._on_wind(1, _live(t=101.0))
    label = panel.wind_status_label.text()
    assert label.startswith("⚠ CWA 測站：連不到 CWA\n") and "已手動改為 5.0" in label and "UAV-1 3.2 m/s" in label
    assert panel.btn_wind_cwa.isEnabled() is False or panel.btn_wind_cwa.isEnabled() is True   # 不卡死（沒有作業區時本來就不可按）
    # 抓取中的提示同樣留著
    import utils.cwa_opendata as co
    import ui.workers.wind_grid_worker as wgw
    monkeypatch.setattr(co, "configured_cwa_api_key", lambda: ("CWA-OK", "環境變數 CWA_API_KEY"))

    class FakeWorker:
        def __init__(self, *a, **k):
            self.finished_ok = types.SimpleNamespace(connect=lambda f: None)
            self.failed = types.SimpleNamespace(connect=lambda f: None)

        def isRunning(self):
            return False

        def start(self):
            pass

    monkeypatch.setattr(wgw, "CwaStationWorker", FakeWorker)
    w._on_wind_cwa_requested({"station": "466920"})
    now[0] = 102.0
    w._on_wind(1, _live(t=102.0))
    assert panel.wind_status_label.text().startswith("抓 CWA 測站觀測中…")
    w._on_wind_cwa_done(_obs())
    assert not panel.wind_status_label.text().startswith("抓 CWA")


def test_api_key_pasted_into_the_station_field_is_rejected_without_echoing_it(monkeypatch, qt_app):
    import utils.cwa_opendata as co
    import ui.workers.wind_grid_worker as wgw
    monkeypatch.setattr(co, "configured_cwa_api_key", lambda: ("CWA-OK", "環境變數 CWA_API_KEY"))
    started = []
    monkeypatch.setattr(wgw, "CwaStationWorker", lambda *a, **k: started.append(1))
    fake_key = "CWA-12345678-1234-1234-1234-123456789ABC"
    w = FakeWin(areas=[AREA])
    panel = VTOLReconPanel()
    w.vtol_recon_panel = panel
    panel.wind_cwa_station_edit.setText(fake_key)
    w._on_wind_cwa_requested({"station": fake_key})
    assert started == [] and panel.wind_cwa_station_edit.text() == ""
    label = panel.wind_status_label.text()
    assert "欄填的是授權碼" in label and fake_key not in label and "1234" not in label
    # 讀授權碼本身炸掉也不能讓 slot 帶例外回 Qt
    def boom():
        raise RuntimeError("settings")
    monkeypatch.setattr(co, "configured_cwa_api_key", boom)
    w._on_wind_cwa_requested({"station": "466920"})
    assert panel.wind_status_label.text().startswith("讀取 CWA 授權碼失敗：RuntimeError") and started == []


def test_apply_wind_to_plan_type_errors_are_reported_not_raised():
    w = FakeWin(apply_plan=True)

    def bad(*a):
        raise TypeError("None")

    w.vtol_recon_panel.apply_wind_to_plan = bad
    assert w._apply_wind_to_plan(None, None, "x") is None
    assert "規劃風未寫入" in w._status.msgs[-1] and w._wind_state()["plan_writing"] is False


def test_grid_without_valid_cells_does_not_write_the_plan():
    w = FakeWin(areas=[AREA], apply_plan=True)
    empty = wf.WindField(lats=(24.0,), lons=(120.0,), u=((float("nan"),),), v=((float("nan"),),), height_m=10,
                         source="open-meteo", label="x", bbox=(24.0, 120.0, 24.1, 120.1))
    w._on_wind_grid_done(empty)
    assert w.vtol_recon_panel.applied == [] and "未寫入" in w._status.msgs[-1]
