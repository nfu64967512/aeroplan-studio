"""分層風（Open-Meteo 同一模式、同一逐時時間步的 10／80／120 m）的 GUI 接線（不建 MainWindow、不連網）。

* 面板：「氣象網格」下拉第三項「分層 10／80／120 m（這小時）」、「2D 顯示高度」列（顯示條件、預設＝最接近偵察航高、
  等距取較低層、手動選過就不再自動）、請求 payload、``wind_profile_view_changed``。
* ``WindProfileWorker.run()`` 同步跑：opener 換成回放錄製的 Open-Meteo 回應（tests/fixtures/，2026-10-19 實打錄的），
  ``urllib.request.urlopen`` 一被叫到就失敗。
* MainWindow 處理函式綁到 FakeWin（同 tests/test_wind_gui.py）：優先序（分層 > 網格 > 規劃風 > 無）、清除、
  2D 高度切換只呼叫 set_wind_profile_view、寫入規劃風用「最接近規劃高度的那一層」與照實的理由（等距、超出範圍 ⚠ 不外推、
  中心缺值改用哪層、高度到 0.01 m）、寫不進去的原因留在狀態列第一行、改航高後依新高度重寫或標明過時（真面板＋防抖）、
  缺層時 2D 自動預設只對可用層挑一次、closeEvent 等執行緒。

地圖是記錄呼叫的假物件（真的頁面在 tests/test_wind_profile_maps.py）。
"""
from __future__ import annotations

import dataclasses
import inspect
import json
import math
import os
import pathlib
import types
import urllib.error
import urllib.parse
import urllib.request

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

from core import wind_field as wf
from ui.main_window import MainWindow
from ui.widgets.vtol_recon_panel import VTOLReconPanel, nearest_profile_height

FIX = pathlib.Path(__file__).resolve().parent / "fixtures"
GRID_2X3 = FIX / "open_meteo_profile_icon_global_2x3_20261019T06.json"
#: 錄製資料一帶（嘉義外海附近）的作業區：外擴 500 m 後的 bbox 落在錄製格點範圍附近
AREA = [(23.48, 120.42), (23.52, 120.42), (23.52, 120.48), (23.48, 120.48)]
WHEN = "2026-10-19T06:00"


@pytest.fixture(scope="module")
def qt_app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def _no_network(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("測試不准連網")
    monkeypatch.setattr(urllib.request, "urlopen", boom)


@pytest.fixture
def panel(qt_app):
    p = VTOLReconPanel()
    p.rb_by_gsd.setChecked(False)
    return p


def _payload(drop=(), null_at=()):
    """錄製的 2×3 多點回應（JSON list）；drop 的高度從每一點的 hourly 拿掉（模擬模式不提供那一層）；
    null_at = [(第幾點, 高度)]：那一點那一層的風速設成 null（層還在，只是那一點沒值）。"""
    data = json.loads(GRID_2X3.read_text(encoding="utf-8"))
    assert isinstance(data, list) and len(data) == 6, "錄製資料是 6 點的多點回應（list）"
    for it in data:
        for h in drop:
            it["hourly"].pop(f"wind_speed_{h}m", None)
            it["hourly"].pop(f"wind_direction_{h}m", None)
    for k, h in null_at:
        data[k]["hourly"][f"wind_speed_{h}m"] = [None]
    return data


def _profile(bbox, drop=(), null_at=()):
    """用錄製回應組一份 WindProfile，格點放在 bbox 內（與 fetch_open_meteo_profile 同一套：grid_points 3×2）。"""
    lats, lons = wf.grid_points(bbox, 3, 2)
    p = wf.parse_open_meteo_profile(_payload(drop, null_at), lats, lons, when_utc=WHEN)
    return dataclasses.replace(p, bbox=tuple(float(x) for x in bbox))


class _Resp:
    def __init__(self, b):
        self._b = b

    def read(self):
        return self._b

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


# ═══════════════════════════════════════════════════════════════════════
#  面板
# ═══════════════════════════════════════════════════════════════════════
def test_nearest_profile_height_is_nearest_layer_ties_lower_no_interpolation():
    hs = (10.0, 80.0, 120.0)
    assert nearest_profile_height(100.0, hs) == 80.0, "100 m 與 80／120 等距 ⇒ 較低層"
    assert nearest_profile_height(45.0, hs) == 10.0, "45 m 與 10／80 等距 ⇒ 較低層"
    assert nearest_profile_height(101.0, hs) == 120.0
    assert nearest_profile_height(300.0, hs) == 120.0 and nearest_profile_height(0.0, hs) == 10.0, "範圍外取端點層，不外推"
    assert nearest_profile_height(80.0, (10.0, 120.0)) == 120.0
    for bad in (float("nan"), float("inf"), None, "x"):
        assert nearest_profile_height(bad, hs) is None
    assert nearest_profile_height(50.0, ()) is None


def test_panel_has_the_profile_item_and_a_2d_height_row_only_when_needed(panel):
    combo = panel.wind_grid_height_combo
    assert [combo.itemData(i) for i in range(combo.count())] == ["10m", "120m", "profile"]
    assert combo.itemText(2) == "分層 10／80／120 m（這小時）"
    assert "非實測" in combo.toolTip() and "不內插" in combo.toolTip() and "icon_global" in combo.toolTip()
    h2d = panel.wind_profile_h2d_combo
    assert [h2d.itemData(i) for i in range(h2d.count())] == [10.0, 80.0, 120.0]
    assert [h2d.itemText(i) for i in range(h2d.count())] == ["10 m", "80 m", "120 m"]
    assert panel.wind_profile_h2d_row.isVisibleTo(panel) is False, "預設 10 m 網格：不顯示 2D 顯示高度"
    assert panel.get_wind_display()["profile_height_2d_m"] == 120.0, "預設偵察航高 120 m ⇒ 120 m 那層"
    panel.altitude_spin.setValue(100.0)
    combo.setCurrentIndex(2)
    assert panel.wind_profile_h2d_row.isVisibleTo(panel) is True
    assert panel.get_wind_display()["profile_height_2d_m"] == 80.0, "航高 100 m：與 80／120 等距取較低層"
    combo.setCurrentIndex(0)
    assert panel.wind_profile_h2d_row.isVisibleTo(panel) is False
    panel.set_wind_profile_loaded(True)
    assert panel.wind_profile_h2d_row.isVisibleTo(panel) is True, "地圖上還有分層風 ⇒ 留著讓你換層"
    panel.set_wind_profile_loaded(False)
    assert panel.wind_profile_h2d_row.isVisibleTo(panel) is False


def test_panel_profile_request_payload_and_2d_view_signal(panel):
    reqs, views = [], []
    panel.wind_grid_requested.connect(reqs.append)
    panel.wind_profile_view_changed.connect(views.append)
    panel.set_corner_count(3)
    panel.btn_wind_grid.click()
    panel.altitude_spin.setValue(45.0)
    panel.wind_grid_height_combo.setCurrentIndex(2)            # 預設跟航高：45 m 與 10／80 等距 ⇒ 10 m
    assert views == [{"height_2d_m": 10.0, "user": False}], "自動換層也要通知（帶 user=False：不是使用者選的）"
    assert panel.get_wind_display()["profile_height_2d_user"] is False
    panel.altitude_spin.setValue(60.0)
    panel.btn_wind_grid.click()                                 # 還沒手動選 ⇒ 抓之前再對一次航高（60 m ⇒ 80 m）
    assert reqs == [{"height": "10m"}, {"height": "profile"}], "payload 只有 height，與 10m／120m 同形"
    assert panel.get_wind_display()["profile_height_2d_m"] == 80.0 and views[-1] == {"height_2d_m": 80.0, "user": False}
    panel.wind_profile_h2d_combo.setCurrentIndex(0)             # 使用者手動選 10 m
    assert views[-1] == {"height_2d_m": 10.0, "user": True}
    assert panel.get_wind_display()["profile_height_2d_user"] is True
    panel.altitude_spin.setValue(120.0)
    panel.wind_grid_height_combo.setCurrentIndex(0)
    panel.wind_grid_height_combo.setCurrentIndex(2)
    panel.btn_wind_grid.click()
    assert panel.get_wind_display()["profile_height_2d_m"] == 10.0, "手動選過就不再被航高蓋掉"
    assert reqs[-1] == {"height": "profile"}


def test_panel_default_keeps_the_current_choice_when_the_altitude_cannot_be_read(panel, monkeypatch):
    panel.wind_profile_h2d_combo.setCurrentIndex(1)
    panel._wind_profile_h2d_user = False

    def boom():
        raise ValueError("GSD 反算失敗")

    monkeypatch.setattr(panel, "effective_altitude_m", boom)
    panel.wind_grid_height_combo.setCurrentIndex(2)
    assert panel.get_wind_display()["profile_height_2d_m"] == 80.0


# ═══════════════════════════════════════════════════════════════════════
#  背景執行緒（run() 同步跑、回放錄製回應）
# ═══════════════════════════════════════════════════════════════════════
BBOX = (23.40, 120.30, 23.60, 120.60)


def _run(worker):
    got, err = [], []
    worker.finished_ok.connect(got.append)
    worker.failed.connect(err.append)
    worker.run()
    return got, err


def test_profile_worker_run_fetches_only_the_profile_with_the_recorded_response(qt_app, monkeypatch):
    from ui.workers.wind_grid_worker import WindProfileWorker
    seen = []

    def opener(url, timeout):
        seen.append((url, timeout))
        return _Resp(GRID_2X3.read_bytes())

    monkeypatch.setattr(wf, "fetch_open_meteo", lambda *a, **k: pytest.fail("分層 worker 不該抓單層網格"))
    got, err = _run(WindProfileWorker(BBOX, nx=3, ny=2, opener=opener))
    assert err == [] and len(got) == 1 and isinstance(got[0], wf.WindProfile)
    p = got[0]
    assert p.heights() == [10.0, 80.0, 120.0] and p.model == "icon_global" and p.bbox == BBOX
    assert p.time_utc == "2026-10-19T06:00", "錄製回應只有一筆（forecast_hours=1），三層同一個時間步"
    assert len(seen) == 1
    q = urllib.parse.parse_qs(urllib.parse.urlparse(seen[0][0]).query)
    assert q["models"] == ["icon_global"] and "wind_speed_80m" in q["hourly"][0] and "current" not in q


def test_profile_worker_reports_errors_and_abandoned_results_are_dropped(qt_app):
    from ui.workers.wind_grid_worker import WindProfileWorker

    def no_route(url, timeout):
        raise urllib.error.URLError("no route")

    got, err = _run(WindProfileWorker(BBOX, nx=3, ny=2, opener=no_route))
    assert got == [] and len(err) == 1 and err[0].startswith("連不到 Open-Meteo")
    got, err = _run(WindProfileWorker(BBOX, nx=3, ny=2, opener=lambda url, timeout: object()))
    assert got == [] and len(err) == 1 and err[0].startswith("TypeError: "), "非預期的例外帶型別名"
    w = WindProfileWorker(BBOX, nx=3, ny=2, opener=lambda url, timeout: _Resp(GRID_2X3.read_bytes()))
    w.stop()
    assert _run(w) == ([], []), "放棄的結果不送"
    w = WindProfileWorker(BBOX, nx=3, ny=2, opener=no_route)
    w.stop()
    assert _run(w) == ([], [])


# ═══════════════════════════════════════════════════════════════════════
#  MainWindow 處理函式（FakeWin）
# ═══════════════════════════════════════════════════════════════════════
_BOUND = ("_wind_state", "_wind_display", "_wind_limit_mps", "_wind_area_bbox", "_on_wind", "_wind_arrow_rows",
          "_push_wind_arrows", "_refresh_wind_status", "_refresh_wind_field", "_apply_wind_options",
          "_on_wind_display_changed", "_on_wind_grid_requested", "_on_wind_grid_done", "_on_wind_grid_failed",
          "_on_wind_grid_clear", "survey_area_polygons", "_wind_loaded", "_wind_near_point", "_apply_wind_to_plan",
          "_wind_note",
          # 2026-10-19 分層風
          "_start_wind_profile_fetch", "_wind_profile_heights_of", "_wind_planned_alt_m", "_wind_profile_h2d",
          "_wind_profile_centre", "_wind_profile_payload", "_wind_profile_status", "_push_wind_profile",
          "_on_wind_profile_done", "_on_wind_profile_failed", "_drop_wind_profile", "_on_wind_profile_view_changed",
          "_apply_profile_wind_to_plan", "_profile_plan_not_written", "_on_planned_altitude_changed")


class FakeMap:
    """記錄呼叫的地圖；分層風的 payload 照真的地圖一樣先過 B 的 ``wind_layer.check_wind_profile``（不合法 ⇒ ValueError，
    跟真的 MapWidget／CesiumMapWidget 一樣），所以這裡收下的一定是兩張真地圖也收得下的 payload。"""

    def __init__(self, *, profile_api=True):
        self.calls = []
        self.checked = None                                 # check_wind_profile 的結果（頁面實際拿到的 dict）
        if not profile_api:
            self.set_wind_profile = None                    # 舊版地圖：沒有分層 API

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

    def set_wind_profile(self, payload):
        from ui.widgets.wind_layer import check_wind_profile
        checked = check_wind_profile(payload)               # 真的地圖會做的檢查（失敗就丟、不記）
        self.calls.append(("profile", json.loads(json.dumps(payload, allow_nan=False))))
        self.checked = checked
        return {}

    def clear_wind_profile(self):
        self.calls.append(("clear_profile", None))
        self.checked = None
        return True

    def set_wind_profile_view(self, *, height_2d_m=None, layers_3d=None):
        from ui.widgets.wind_layer import resolve_profile_view
        if self.checked is not None:                        # 真的地圖：最接近的現有層（不內插）
            self.checked.update(resolve_profile_view(self.checked, height_2d_m=height_2d_m, layers_3d=layers_3d))
        self.calls.append(("view", {"height_2d_m": height_2d_m, "layers_3d": layers_3d}))

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
    def __init__(self, *, plan=False, live=True, speed=3.0, d=270.0, limit=3.0, apply_plan=False, h2d=120.0,
                 alt=120.0):
        self.disp = {"plan": plan, "live": live, "speed_mps": speed, "dir_from_deg": d, "grid_height": "profile",
                     "profile_height_2d_m": h2d, "apply_plan": apply_plan, "cwa_station": ""}
        self.alt = alt
        self.hv_map_max_wind_spin = _Spin(limit)
        self.status, self.busy, self.loaded, self.applied, self.profile_loaded = [], [], [], [], []

    def get_wind_display(self):
        return dict(self.disp)

    def effective_altitude_m(self):
        if isinstance(self.alt, Exception):
            raise self.alt
        return self.alt

    def set_wind_status(self, t):
        self.status.append(t)

    def set_wind_grid_busy(self, b, t=None):
        self.busy.append((b, t))

    def set_wind_grid_loaded(self, b, t=None):
        self.loaded.append((b, t))

    def set_wind_profile_loaded(self, b):
        self.profile_loaded.append(b)

    def apply_wind_to_plan(self, s, d, source):
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
    def __init__(self, *, areas=(), profile_api=True, **panel_kw):
        self.survey_areas, self.corners = [list(a) for a in areas], []
        self.vtol_recon_panel = FakePanel(**panel_kw)
        self.map_widget = FakeMap(profile_api=profile_api)
        self._status = FakeStatus()
        self._sitl_links = []
        for name in _BOUND:
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))
        for k in ("WIND_LIVE_PUSH_INTERVAL_S", "WIND_LIVE_STALE_S", "WIND_PLAN_SYSID", "WIND_GRID_MARGIN_M",
                  "WIND_CWA_SYSID", "WIND_PROFILE_PARTICLES_TOTAL"):
            setattr(self, k, getattr(MainWindow, k))

    def statusBar(self):
        return self._status


def _win(**kw):
    w = FakeWin(areas=[AREA], **kw)
    return w, _profile(w._wind_area_bbox())


def _grid():
    return wf.parse_open_meteo([{"latitude": 23.5, "longitude": 120.45, "current": {
        "time": "2026-10-19T06:00", "wind_speed_10m": 1.0, "wind_direction_10m": 180}}] * 4,
        [23.45, 23.55], [120.40, 120.50], "10m")


class FakeWorker:
    started = []

    def __init__(self, bbox, *, parent=None, **kw):
        self.bbox, self.kw, self.running = bbox, kw, False
        self.finished_ok = types.SimpleNamespace(connect=lambda f: None)
        self.failed = types.SimpleNamespace(connect=lambda f: None)

    def isRunning(self):
        return self.running

    def start(self):
        FakeWorker.started.append(self)

    def deleteLater(self):
        pass


def test_profile_request_starts_the_profile_worker_on_the_grid_bbox_and_shares_the_busy_gate(monkeypatch):
    import ui.workers.wind_grid_worker as wgw
    FakeWorker.started = []
    monkeypatch.setattr(wgw, "WindProfileWorker", FakeWorker)
    monkeypatch.setattr(wgw, "WindGridWorker", lambda *a, **k: pytest.fail("分層不該起單層網格 worker"))
    w0 = FakeWin()
    w0._on_wind_grid_requested({"height": "profile"})
    assert FakeWorker.started == [] and w0.vtol_recon_panel.busy == [(False, None)]
    assert w0.vtol_recon_panel.status[-1].startswith("請先圈好作業區")
    w = FakeWin(areas=[AREA])
    w._on_wind_grid_requested({"height": "profile"})
    assert len(FakeWorker.started) == 1
    assert FakeWorker.started[0].bbox == pytest.approx(w._wind_area_bbox()), "範圍與單層網格同一套（作業區外擴 500 m）"
    assert w.vtol_recon_panel.busy[-1] == (True, None) and "分層風" in w.vtol_recon_panel.status[-1]
    assert "10／80／120" in w.vtol_recon_panel.status[-1]
    # 還在抓（不管是分層或單層）⇒ 擋下，不起第二個
    FakeWorker.started[0].running = True
    w._on_wind_grid_requested({"height": "profile"})
    w._on_wind_grid_requested({"height": "10m"})
    assert len(FakeWorker.started) == 1 and w._status.msgs[-2:] == ["氣象網格還在抓，請稍候"] * 2
    FakeWorker.started[0].running = False
    w._wind_worker = FakeWorker(None)
    w._wind_worker.running = True
    w._on_wind_grid_requested({"height": "profile"})
    assert len(FakeWorker.started) == 1, "單層網格還在抓也要擋"


def test_profile_done_pushes_three_layers_a_centre_column_and_the_2d_layer():
    from ui.widgets.wind_layer import check_wind_field
    w, prof = _win(h2d=80.0, limit=4.0)
    w._on_wind_profile_done(prof)
    payload = w.map_widget.last("profile")
    assert [lay["height_m"] for lay in payload["layers"]] == [10.0, 80.0, 120.0]
    for lay in payload["layers"]:
        assert check_wind_field(lay)["height_m"] == lay["height_m"], "每層都是 check_wind_field 收的 dict"
    bb = w._wind_area_bbox()
    col = payload["column"]
    assert (col["lat"], col["lon"]) == pytest.approx((0.5 * (bb[0] + bb[2]), 0.5 * (bb[1] + bb[3])))
    assert col["levels"] == wf.profile_column(prof, col["lat"], col["lon"]) and len(col["levels"]) == 3
    assert payload["height_2d_m"] == 80.0 and payload["particles_total"] == 2400 and payload["limit_mps"] == 4.0
    assert payload["label"] == prof.label and "非實測" in payload["label"] and "icon_global" in payload["label"]
    json.dumps(payload, allow_nan=False)
    # B 的檢查（兩張真地圖都會做）收下了，而且從 A 的層 label 讀出同一個模式與時間（圖例只一個區塊）
    chk = w.map_widget.checked
    assert chk["heights"] == [10.0, 80.0, 120.0] and chk["height_2d_m"] == 80.0 and chk["limit_mps"] == 4.0
    assert chk["legend"] == "風場分層 10／80／120 m（icon_global 預報，非實測，2026-10-19T06:00 UTC）"
    assert [lv["h_m"] for lv in chk["column"]["levels"]] == [10.0, 80.0, 120.0]
    st = w._wind_state()
    assert st["profile"] is prof and st["grid"] is None and w._wind_loaded() is True
    assert w.vtol_recon_panel.busy[-1] == (False, None) and w.vtol_recon_panel.loaded[-1] == (True, None)
    assert w.vtol_recon_panel.profile_loaded == [True]
    first = w.vtol_recon_panel.status[-1].split("\n")[0]
    assert first == ("分層風：Open-Meteo icon_global 預報（模式資料，非實測），2026-10-19T06:00 UTC；"
                     "可用 10／80／120 m（只有這幾層，不內插、不外推）；2D 顯示 80 m")
    assert "所有高度固定用模式 icon_global" in w.vtol_recon_panel.status[-1], "core 的說明放第二行"
    assert w.map_widget.kinds() == ["options", "profile"], "分層風在圖上時不碰單層流動層"


def test_column_stays_at_the_fetched_area_centre_if_the_area_moved_during_the_fetch():
    """剖面柱放在**抓取時**的範圍中心（資料就在那裡）；抓的 1–3 秒間作業區被拖走，柱子不跟著跑到沒資料的地方。"""
    w, prof = _win()
    bb = w._wind_area_bbox()
    w.survey_areas = [[(la + 0.5, lo + 0.5) for la, lo in AREA]]
    w._on_wind_profile_done(prof)
    col = w.map_widget.last("profile")["column"]
    assert (col["lat"], col["lon"]) == pytest.approx((0.5 * (bb[0] + bb[2]), 0.5 * (bb[1] + bb[3])))
    assert len(col["levels"]) == 3


def test_precedence_profile_over_grid_over_planning_wind_and_no_resend_when_nothing_changed(monkeypatch):
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: 100.0)
    w, prof = _win(plan=True, speed=2.0, d=90.0)
    w._on_wind_display_changed({})
    assert w.map_widget.last("field").source == "planning"
    w._on_wind_grid_done(_grid())
    assert w.map_widget.kinds()[-1] == "field" and w.map_widget.last("field").source == "open-meteo"
    w._on_wind_profile_done(prof)
    assert w.map_widget.kinds()[-1] == "profile" and w._wind_state()["grid"] is None, "分層風取代單層網格"
    assert "Open-Meteo 預報（模式資料，非實測）10m" not in w.vtol_recon_panel.status[-1], "舊網格那一行要消失"
    n = len(w.map_widget.calls)
    w.vtol_recon_panel.disp["speed_mps"] = 2.5                   # 規劃風值改了：分層風內容沒變 ⇒ 不重送
    w._on_wind_display_changed({})
    kinds = w.map_widget.kinds()[n:]
    assert "profile" not in kinds and "field" not in kinds and "clear_field" not in kinds and "arrows" in kinds
    assert w.map_widget.last("arrows")[0]["sysid"] == MainWindow.WIND_PLAN_SYSID, "規劃風降為中心箭頭"
    w.vtol_recon_panel.hv_map_max_wind_spin = _Spin(5.0)         # 上色門檻改了 ⇒ 重送（剖面柱顏色跟著變）
    w._on_wind_display_changed({})
    assert w.map_widget.last("profile")["limit_mps"] == 5.0 and w.map_widget.kinds().count("profile") == 2
    # 兩個都有（例如舊狀態）時，分層風優先
    w._wind_state()["grid"] = _grid()
    w._wind_state()["profile_sent"] = None
    w._refresh_wind_field()
    assert w.map_widget.kinds()[-1] == "profile"
    # 地圖沒有分層 API ⇒ 退回網格（不炸）
    w2, prof2 = _win(profile_api=False)
    w2._wind_state()["grid"] = g = _grid()
    w2._wind_state()["profile"] = prof2
    w2._refresh_wind_field()
    assert w2.map_widget.kinds() == ["field"] and w2.map_widget.last("field") is g


def test_live_fc_wind_arrows_are_untouched_by_the_profile(monkeypatch):
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: 100.0)
    w, prof = _win()
    w._on_wind_profile_done(prof)
    w._on_wind(1, {"dir_from_deg": 272.0, "speed_mps": 3.2, "speed_z_mps": 0.0, "lat": 23.5, "lon": 120.45,
                   "alt_rel": 90.0, "t": 100.0})
    rows = w.map_widget.last("arrows")
    assert [(r["sysid"], r["speed_mps"]) for r in rows] == [(1, 3.2)] and "EKF" in rows[0]["label"]
    assert "UAV-1 3.2 m/s" in w.vtol_recon_panel.status[-1] and "分層風" in w.vtol_recon_panel.status[-1]
    assert "clear_profile" not in w.map_widget.kinds()


def test_2d_height_change_only_calls_set_wind_profile_view():
    w, prof = _win(h2d=120.0)
    w._on_wind_profile_view_changed({"height_2d_m": 10.0})        # 還沒有分層風 ⇒ 什麼都不做
    assert w.map_widget.calls == []
    w._on_wind_profile_done(prof)
    n = len(w.map_widget.calls)
    w.vtol_recon_panel.disp["profile_height_2d_m"] = 10.0
    w._on_wind_profile_view_changed({"height_2d_m": 10.0})
    assert w.map_widget.calls[n:] == [("view", {"height_2d_m": 10.0, "layers_3d": None})]
    assert w.map_widget.checked["height_2d_m"] == 10.0 and w.map_widget.checked["layers_3d"] is None
    assert "2D 顯示 10 m" in w.vtol_recon_panel.status[-1]
    w._on_wind_profile_view_changed({"height_2d_m": 10.0})        # 同一層再選一次 ⇒ 不重複呼叫
    w._on_wind_display_changed({})
    assert [c for c in w.map_widget.calls[n:] if c[0] in ("view", "profile")] == [
        ("view", {"height_2d_m": 10.0, "layers_3d": None})]
    w.vtol_recon_panel.disp["profile_height_2d_m"] = 80.0
    w._on_wind_profile_view_changed({"height_2d_m": 80.0})
    assert w.map_widget.last("view")["height_2d_m"] == 80.0 and w.map_widget.kinds().count("profile") == 1


def test_a_dropped_level_is_named_and_the_2d_layer_falls_back_to_the_nearest_available():
    w = FakeWin(areas=[AREA], h2d=80.0)
    prof = _profile(w._wind_area_bbox(), drop=(80,))
    assert prof.heights() == [10.0, 120.0]
    w._on_wind_profile_done(prof)
    payload = w.map_widget.last("profile")
    assert [lay["height_m"] for lay in payload["layers"]] == [10.0, 120.0]
    assert [lv["h_m"] for lv in payload["column"]["levels"]] == [10.0, 120.0], "缺的那層不補、不內插"
    assert payload["height_2d_m"] == 120.0, "80 m 沒資料 ⇒ 最接近的可用層（10 與 120 中 120 較近）"
    assert w.map_widget.checked["legend"] == "風場分層 10／120 m（icon_global 預報，非實測，2026-10-19T06:00 UTC）"
    status = w.vtol_recon_panel.status[-1]
    assert "可用 10／120 m" in status and "缺 80 m（這個模式這小時沒有有效值，已略過）" in status
    assert "2D 顯示 120 m（選的 80 m 沒有資料，改畫最接近的一層）" in status
    assert "80 m：回應沒有這一層" in status, "core 說明缺層原因"
    n = len(w.map_widget.calls)
    w._on_wind_profile_view_changed({"height_2d_m": 80.0})        # 還是 120 m ⇒ 地圖不用動
    assert w.map_widget.calls[n:] == []


def test_clear_removes_the_profile_and_restores_the_previous_single_field():
    w, prof = _win(plan=True)
    w._on_wind_display_changed({})
    w._on_wind_profile_done(prof)
    n = len(w.map_widget.calls)
    w._on_wind_grid_clear()
    kinds = w.map_widget.kinds()[n:]
    assert kinds[:2] == ["clear_profile", "field"] and w.map_widget.last("field").source == "planning"
    st = w._wind_state()
    assert st["profile"] is None and st["profile_text"] == "" and w._wind_loaded() is False
    assert w.vtol_recon_panel.loaded[-1] == (False, None) and w.vtol_recon_panel.profile_loaded[-1] is False
    assert "分層風" not in w.vtol_recon_panel.status[-1]
    w._on_wind_grid_clear()                                         # 再按一次：沒有分層風就不送清除
    assert w.map_widget.kinds().count("clear_profile") == 1
    w2, prof2 = _win(plan=False)
    w2._on_wind_profile_done(prof2)
    w2._on_wind_grid_clear()
    assert w2.map_widget.kinds()[-3:-1] == ["clear_profile", "clear_field"]


def test_a_later_grid_replaces_the_profile():
    w, prof = _win()
    w._on_wind_profile_done(prof)
    g = _grid()
    n = len(w.map_widget.calls)
    w._on_wind_grid_done(g)
    kinds = w.map_widget.kinds()[n:]
    assert kinds.index("clear_profile") < kinds.index("field") and w.map_widget.last("field") is g
    assert w._wind_state()["profile"] is None and w.vtol_recon_panel.profile_loaded[-1] is False
    assert "分層風" not in w.vtol_recon_panel.status[-1] and w.vtol_recon_panel.status[-1].startswith("Open-Meteo")


def test_profile_failure_keeps_what_is_loaded_and_bad_data_is_reported_not_raised():
    w, prof = _win()
    w._on_wind_profile_failed("HTTP 429")
    assert w.vtol_recon_panel.loaded[-1] == (False, None)
    assert w.vtol_recon_panel.status[-1].startswith("⚠ 分層風：HTTP 429") and "HTTP 429" in w._status.msgs[-1]
    w._on_wind_profile_done(prof)
    w._on_wind_profile_failed("連不到 Open-Meteo")
    assert w._wind_state()["profile"] is prof and w.vtol_recon_panel.loaded[-1] == (True, None)
    assert w.vtol_recon_panel.status[-1].startswith("⚠ 分層風：連不到 Open-Meteo\n分層風：Open-Meteo icon_global")
    # 沒有範圍（bbox）也沒有作業區 ⇒ 不知道剖面柱放哪：走失敗路徑，不帶例外回 Qt
    w2 = FakeWin()
    w2._on_wind_profile_done(dataclasses.replace(prof, bbox=None))
    assert w2._wind_state()["profile"] is None and w2.map_widget.calls == []
    assert w2.vtol_recon_panel.status[-1].startswith("⚠ 分層風：資料無法顯示（ValueError")
    # 地圖拒收（頁面端檢查失敗）也一樣只報不炸
    w3, prof3 = _win()

    def reject(_p):
        raise ValueError("風場的緯度要遞增")

    w3.map_widget.set_wind_profile = reject
    w3._on_wind_profile_done(prof3)
    assert w3.vtol_recon_panel.status[-1].startswith("⚠ 分層風畫不出來：ValueError: 風場的緯度要遞增")


#: 作業區中心（錄製資料雙線性到中心）的三層：10 m 2.59 m/s 來自 311°、80 m 2.83 m/s 來自 312°、120 m 2.76 m/s 來自 314°
_NOTE = {
    # 等距：依規則取較低層，而且要說「一樣近」（不是含糊的「最接近」）；較高的 120 m 比 80 m 弱 ⇒ 不加警告
    100.0: (80.0, "已寫入 80 m 的預報風（規劃高度 100 m 與 80／120 m 一樣近，依規則取較低的 80 m）"),
    # 100.4 m 已經離 120 較近：高度要照實印（:.0f 會印成「100 m」，跟上面的等距規則互相矛盾）
    100.4: (120.0, "已寫入 120 m 的預報風（最接近規劃高度 100.4 m 的那一層）"),
    115.0: (120.0, "已寫入 120 m 的預報風（最接近規劃高度 115 m 的那一層）"),
    # 等距且較高的那層比較強 ⇒ 警告寫出較強的值（風速上限要看較強的）
    45.0: (10.0, "已寫入 10 m 的預報風（規劃高度 45 m 與 10／80 m 一樣近，依規則取較低的 10 m）；"
                 "⚠ 80 m 的風較強（2.8 m/s 來自 312°，10 m 是 2.6 m/s）：檢查風速上限請以較強者為準"),
    44.6: (10.0, "已寫入 10 m 的預報風（最接近規劃高度 44.6 m 的那一層）"),
    45.4: (80.0, "已寫入 80 m 的預報風（最接近規劃高度 45.4 m 的那一層）"),
    # 超出最高層：寫的是 120 m 的值，⚠ 明說不代表規劃高度的風（不外推）
    300.0: (120.0, "已寫入 120 m 的預報風（規劃高度 300 m 高於最高層 120 m）；⚠ 這是 120 m 的預報值，不代表 300 m 的風"
                   "（只有 10／80／120 m 這幾層，不外推；高處的風可能更強）"),
    500.0: (120.0, "已寫入 120 m 的預報風（規劃高度 500 m 高於最高層 120 m）；⚠ 這是 120 m 的預報值，不代表 500 m 的風"
                   "（只有 10／80／120 m 這幾層，不外推；高處的風可能更強）"),
    173.21: (120.0, "已寫入 120 m 的預報風（規劃高度 173.21 m 高於最高層 120 m）；⚠ 這是 120 m 的預報值，不代表 173.21 m 的風"
                    "（只有 10／80／120 m 這幾層，不外推；高處的風可能更強）"),
}


@pytest.mark.parametrize("alt", sorted(_NOTE))
def test_profile_writes_the_layer_nearest_the_planned_altitude_and_says_why(alt):
    used, want = _NOTE[alt]
    w, prof = _win(apply_plan=True, alt=alt)
    w._on_wind_profile_done(prof)
    col = w.map_widget.last("profile")["column"]
    s, d, h = wf.profile_wind_at(prof, col["lat"], col["lon"], alt)
    assert h == used
    (s2, d2, src), = w.vtol_recon_panel.applied
    assert (s2, d2) == (pytest.approx(s), pytest.approx(d))
    assert f"Open-Meteo icon_global 分層預報 {used:g} m" in src and "非實測" in src and "2026-10-19T06:00 UTC" in src
    status = w.vtol_recon_panel.status[-1]
    assert status.endswith(want) and w._status.msgs[-1].endswith(want), status
    assert "規劃風已寫入：" in status
    if alt > 120.0:
        assert "最接近" not in status.split("規劃風已寫入：")[1], "超出範圍不可以說成「最接近」的正常情況"
    assert w._wind_state()["plan_applied"] == (round(s, 1), float(round(d % 360)))
    pp = w._wind_state()["plan_profile"]
    assert pp["used_h"] == used and pp["alt"] == alt and pp["profile"] is prof


def test_profile_write_names_a_layer_missing_at_the_centre_instead_of_calling_the_fallback_nearest():
    """80 m 這層存在（heights 還有它），只是作業區中心的一個角點沒值 ⇒ 寫入的是 120 m，理由要說 80 m 在中心沒值。"""
    w = FakeWin(areas=[AREA], apply_plan=True, alt=80.0)
    prof = _profile(w._wind_area_bbox(), null_at=[(1, 80)])
    assert prof.heights() == [10.0, 80.0, 120.0]
    w._on_wind_profile_done(prof)
    col = w.map_widget.last("profile")["column"]
    assert [lv["h_m"] for lv in col["levels"]] == [10.0, 120.0]
    (_s, _d, src), = w.vtol_recon_panel.applied
    assert "分層預報 120 m" in src
    assert w.vtol_recon_panel.status[-1].endswith(
        "已寫入 120 m 的預報風（最接近規劃高度 80 m 的 80 m 在作業區中心沒有值，改用 120 m）")


def test_profile_write_needs_the_option_and_a_readable_altitude():
    w, prof = _win(apply_plan=False)
    w._on_wind_profile_done(prof)
    assert w.vtol_recon_panel.applied == []
    w2, prof2 = _win(apply_plan=True, alt=ValueError("GSD"))
    w2._on_wind_profile_done(prof2)
    assert w2.vtol_recon_panel.applied == [] and "讀不到規劃高度" in w2._status.msgs[-1]
    w3, prof3 = _win(apply_plan=True, alt=float("nan"))
    w3._on_wind_profile_done(prof3)
    assert w3.vtol_recon_panel.applied == [] and "讀不到規劃高度" in w3._status.msgs[-1]
    # 原因要留在面板狀態列的第一行（不是只閃 8 秒），而且說清楚規劃器現在用的是哪個值
    first = w3.vtol_recon_panel.status[-1].split("\n")[0]
    assert first == "⚠ 分層風未寫入規劃風：讀不到規劃高度（偵察航高）；規劃風欄位沒有改（仍是面板上的值）"


def test_a_profile_that_cannot_be_written_says_so_persistently_and_names_the_value_still_in_use(monkeypatch):
    """先由 10 m 網格寫入，再來的分層風寫不進去 ⇒ 第一行寫明未寫入＋原因＋規劃器仍用網格的值（網格已從圖上換掉）。"""
    w, prof = _win(apply_plan=True, alt=float("nan"))
    w.vtol_recon_panel.alt = 120.0
    w._on_wind_grid_done(_grid())
    assert len(w.vtol_recon_panel.applied) == 1
    w.vtol_recon_panel.alt = float("nan")
    w._on_wind_profile_done(prof)
    lines = w.vtol_recon_panel.status[-1].split("\n")
    assert lines[0].startswith("⚠ 分層風未寫入規劃風：讀不到規劃高度（偵察航高）；規劃風仍是先前由 Open-Meteo 10 m 網格")
    assert any(ln.startswith("規劃風已寫入：1.0 m/s 來自 180°") for ln in lines), "規劃器的實際值仍列出"
    assert len(w.vtol_recon_panel.applied) == 1 and w._wind_state()["plan_profile"] is None
    # 中心沒有任何一層有值：同樣留在第一行
    w2, prof2 = _win(apply_plan=True, alt=100.0)
    monkeypatch.setattr(wf, "profile_wind_at", lambda *a, **k: None)
    w2._on_wind_profile_done(prof2)
    assert w2.vtol_recon_panel.status[-1].split("\n")[0] == (
        "⚠ 分層風未寫入規劃風：作業區中心沒有有效的分層預報風；規劃風欄位沒有改（仍是面板上的值）")
    assert w2.vtol_recon_panel.applied == []


# ── 改航高之後：寫入的那一層要跟著重挑，不然要標明過時（有效值契約）──────────────────
def test_altitude_change_rewrites_the_planner_with_the_new_nearest_layer():
    w, prof = _win(apply_plan=True, alt=100.0)
    w._on_wind_profile_done(prof)
    col = w.map_widget.last("profile")["column"]
    assert w.vtol_recon_panel.applied[-1][2].startswith("Open-Meteo icon_global 分層預報 80 m")
    w.vtol_recon_panel.alt = 15.0
    w._on_planned_altitude_changed(15.0)
    s10, d10, h = wf.profile_wind_at(prof, col["lat"], col["lon"], 15.0)
    assert h == 10.0 and len(w.vtol_recon_panel.applied) == 2
    s2, d2, src = w.vtol_recon_panel.applied[-1]
    assert (s2, d2) == (pytest.approx(s10), pytest.approx(d10)) and "分層預報 10 m" in src
    assert w.vtol_recon_panel.status[-1].endswith("已寫入 10 m 的預報風（最接近規劃高度 15 m 的那一層）")
    assert w._wind_state()["plan_profile"]["alt"] == 15.0
    # 高度沒變（防抖後重複送同一值）⇒ 不再寫
    w._on_planned_altitude_changed(15.0)
    assert len(w.vtol_recon_panel.applied) == 2
    # 高過最高層：寫 120 m 並 ⚠ 不外推
    w.vtol_recon_panel.alt = 346.41
    w._on_planned_altitude_changed(346.41)
    assert w.vtol_recon_panel.applied[-1][2].startswith("Open-Meteo icon_global 分層預報 120 m")
    assert "⚠ 這是 120 m 的預報值，不代表 346.41 m 的風" in w.vtol_recon_panel.status[-1]


def test_altitude_change_without_the_write_option_marks_the_planner_wind_as_stale():
    w, prof = _win(apply_plan=True, alt=100.0)
    w._on_wind_profile_done(prof)
    w.vtol_recon_panel.disp["apply_plan"] = False                # 寫入之後把「抓到就寫入」取消
    w.vtol_recon_panel.alt = 120.0
    w._on_planned_altitude_changed(120.0)
    assert len(w.vtol_recon_panel.applied) == 1, "沒勾就不改寫數值"
    plan = w._wind_state()["plan_text"]
    assert plan.startswith("規劃風已寫入：2.8 m/s 來自 312°")
    assert ("⚠ 規劃高度已改為 120 m，依新高度該用 120 m 層（最接近規劃高度 120 m 的那一層）；"
            "規劃風仍是寫入時（規劃高度 100 m）選的 80 m 層的值，沒有改寫（「抓到就寫入規劃風」沒勾）") in plan
    assert "最接近規劃高度 100 m" not in plan and "一樣近" not in plan, "寫入時的舊理由不可以留著"
    assert plan in w.vtol_recon_panel.status[-1]
    # 還是同一層：說明更新成新高度、數值不動
    w.vtol_recon_panel.alt = 90.0
    w._on_planned_altitude_changed(90.0)
    assert len(w.vtol_recon_panel.applied) == 1
    assert "規劃高度已改為 90 m，仍是 80 m 層（最接近規劃高度 90 m 的那一層）" in w._wind_state()["plan_text"]
    # 改回寫入時的高度：說明恢復成寫入當下那一句
    w.vtol_recon_panel.alt = 100.0
    w._on_planned_altitude_changed(100.0)
    assert w._wind_state()["plan_text"].endswith("已寫入 80 m 的預報風（規劃高度 100 m 與 80／120 m 一樣近，依規則取較低的 80 m）")
    assert "已改為" not in w._wind_state()["plan_text"]
    # 分層風清掉之後再改高度：照樣標明（理由換成「已清除」）
    w._on_wind_grid_clear()
    w.vtol_recon_panel.alt = 15.0
    w._on_planned_altitude_changed(15.0)
    assert "依新高度該用 10 m 層" in w._wind_state()["plan_text"]
    assert w._wind_state()["plan_text"].endswith("沒有改寫（分層風已清除）")


def test_altitude_change_after_a_manual_edit_or_another_source_leaves_the_planner_alone(monkeypatch):
    monkeypatch.setattr("ui.main_window.time.monotonic", lambda: 100.0)
    w, prof = _win(apply_plan=True, alt=100.0)
    w._on_wind_profile_done(prof)
    w.vtol_recon_panel.disp["speed_mps"] = 6.0                   # 使用者手改
    w._on_wind_display_changed({})
    assert w._wind_state()["plan_profile"] is None
    before = w._wind_state()["plan_text"]
    w.vtol_recon_panel.alt = 15.0
    w._on_planned_altitude_changed(15.0)
    assert len(w.vtol_recon_panel.applied) == 1 and w._wind_state()["plan_text"] == before
    # 之後由網格寫入：也不跟分層風
    w2, prof2 = _win(apply_plan=True, alt=100.0)
    w2._on_wind_profile_done(prof2)
    w2._on_wind_grid_done(_grid())
    assert w2._wind_state()["plan_profile"] is None
    w2.vtol_recon_panel.alt = 15.0
    w2._on_planned_altitude_changed(15.0)
    assert len(w2.vtol_recon_panel.applied) == 2 and w2.vtol_recon_panel.applied[-1][2].startswith("Open-Meteo 10 m 網格")


def test_real_panel_altitude_change_moves_the_2d_layer_and_rewrites_the_planner(qt_app):
    """真的面板：寫入後把航高改成 15 m ⇒（防抖到期）2D 預設層換成 10 m、地圖只收到 view、規劃風改寫成 10 m 層。"""
    from PyQt6.QtTest import QTest
    w = FakeWin(areas=[AREA])
    panel = VTOLReconPanel()
    panel.rb_by_gsd.setChecked(False)
    panel.altitude_spin.setValue(100.0)
    panel.wind_apply_plan_check.setChecked(True)
    w.vtol_recon_panel = panel
    panel.wind_display_changed.connect(w._on_wind_display_changed)
    panel.wind_profile_view_changed.connect(w._on_wind_profile_view_changed)
    panel.planned_altitude_changed.connect(w._on_planned_altitude_changed)
    alts = []
    panel.planned_altitude_changed.connect(alts.append)
    panel.wind_grid_height_combo.setCurrentIndex(2)
    QTest.qWait(panel.PLANNED_ALT_DEBOUNCE_MS + 150)
    alts.clear()
    prof = _profile(w._wind_area_bbox())
    w._on_wind_profile_done(prof)
    assert w.map_widget.last("profile")["height_2d_m"] == 80.0
    n = len(w.map_widget.calls)
    panel.altitude_spin.setValue(14.0)
    panel.altitude_spin.setValue(15.0)                           # 連打兩次：防抖後只發一次
    assert alts == [], "防抖：還沒到期不發"
    QTest.qWait(panel.PLANNED_ALT_DEBOUNCE_MS + 150)
    assert alts == [15.0]
    assert panel.get_wind_display()["profile_height_2d_m"] == 10.0, "還沒手動選 ⇒ 2D 預設跟著航高"
    assert ("view", {"height_2d_m": 10.0, "layers_3d": None}) in w.map_widget.calls[n:]
    assert w.map_widget.kinds()[n:].count("profile") == 0, "只換 2D 那一層：不重送整份分層風"
    col = w.map_widget.last("profile")["column"]
    s10, d10, _h = wf.profile_wind_at(prof, col["lat"], col["lon"], 15.0)
    assert (panel.hv_wind_speed_spin.value(), panel.hv_wind_dir_spin.value()) == (round(s10, 1), round(d10))
    label = panel.wind_status_label.text()
    assert "已寫入 10 m 的預報風（最接近規劃高度 15 m 的那一層）" in label and "已手動改為" not in label
    assert "2D 顯示 10 m（預設：最接近規劃高度 15 m 的可用層）" in label
    # GSD 模式（預設 0.05 m/px ⇒ 173.21 m）：高過最高層 ⇒ 寫 120 m 並警告
    panel.rb_by_gsd.setChecked(True)
    QTest.qWait(panel.PLANNED_ALT_DEBOUNCE_MS + 150)
    assert alts[-1] == pytest.approx(panel.effective_altitude_m()) and alts[-1] > 120.0
    assert "⚠ 這是 120 m 的預報值，不代表" in panel.wind_status_label.text()


def test_dropped_layer_auto_2d_default_snaps_once_against_the_available_layers(qt_app):
    """80 m 缺層、航高 50 m、沒手動選：2D 要畫 10 m（離 50 m 最近的可用層），不是先挑 80 再挑成 120；
    狀態也不可以說成「選的 80 m」。"""
    w = FakeWin(areas=[AREA])
    panel = VTOLReconPanel()
    panel.rb_by_gsd.setChecked(False)
    panel.altitude_spin.setValue(50.0)
    panel.wind_apply_plan_check.setChecked(True)
    w.vtol_recon_panel = panel
    panel.wind_profile_view_changed.connect(w._on_wind_profile_view_changed)
    panel.wind_grid_height_combo.setCurrentIndex(2)
    assert panel.wind_profile_height_2d_m() == 80.0 and panel.get_wind_display()["profile_height_2d_user"] is False
    prof = _profile(w._wind_area_bbox(), drop=(80,))
    w._on_wind_profile_done(prof)
    assert w.map_widget.last("profile")["height_2d_m"] == 10.0
    first = panel.wind_status_label.text().split("\n")[0]
    assert "2D 顯示 10 m（預設：最接近規劃高度 50 m 的可用層）" in first and "選的" not in first
    assert "已寫入 10 m 的預報風" in panel.wind_status_label.text(), "2D 與寫入規劃器挑的是同一層"
    # 面板自動換層（user=False）送來的 view 也不能被當成「選的」再挑一次
    n = len(w.map_widget.calls)
    w._on_wind_profile_view_changed({"height_2d_m": 80.0, "user": False})
    assert w.map_widget.calls[n:] == [] and "選的" not in panel.wind_status_label.text()
    # 手動選 80 m（沒資料）：這時才說「選的 80 m 沒有資料」
    panel.wind_profile_h2d_combo.setCurrentIndex(2)
    panel.wind_profile_h2d_combo.setCurrentIndex(1)
    assert "2D 顯示 120 m（選的 80 m 沒有資料，改畫最接近的一層）" in panel.wind_status_label.text()


def test_fmt_alt_keeps_the_decimals_that_decide_the_layer():
    from ui.main_window import fmt_alt_m
    assert [fmt_alt_m(x) for x in (100.0, 100.4, 100.04, 173.20508, 45.0, 0.0, 120.999)] == [
        "100", "100.4", "100.04", "173.21", "45", "0", "121"]


def test_real_panel_end_to_end_with_the_fake_map(qt_app):
    """真的面板：done → 「2D 顯示高度」列出現、寫入規劃風（setValue 同步回呼不能被當成手改）、換層只送 view。"""
    w = FakeWin(areas=[AREA])
    panel = VTOLReconPanel()
    panel.rb_by_gsd.setChecked(False)
    panel.altitude_spin.setValue(100.0)
    panel.wind_apply_plan_check.setChecked(True)
    w.vtol_recon_panel = panel
    panel.wind_display_changed.connect(w._on_wind_display_changed)
    panel.wind_profile_view_changed.connect(w._on_wind_profile_view_changed)
    panel.wind_grid_height_combo.setCurrentIndex(2)              # 航高 100 ⇒ 2D 預設 80 m
    prof = _profile(w._wind_area_bbox())
    w._on_wind_profile_done(prof)
    assert w.map_widget.last("profile")["height_2d_m"] == 80.0
    label = panel.wind_status_label.text()
    assert "已寫入 80 m 的預報風（規劃高度 100 m 與 80／120 m 一樣近，依規則取較低的 80 m）" in label
    assert "已手動改為" not in label
    assert w.map_widget.kinds().count("profile") == 1, "寫入規劃風觸發的顯示更新不重送分層風"
    panel.wind_grid_height_combo.setCurrentIndex(0)
    assert panel.wind_profile_h2d_row.isVisibleTo(panel) is True, "分層風還在圖上"
    panel.wind_profile_h2d_combo.setCurrentIndex(0)
    assert w.map_widget.last("view") == {"height_2d_m": 10.0, "layers_3d": None}
    assert "2D 顯示 10 m" in panel.wind_status_label.text()
    w._on_wind_grid_clear()
    assert panel.wind_profile_h2d_row.isVisibleTo(panel) is False and panel.btn_wind_grid_clear.isEnabled() is False


# ═══════════════════════════════════════════════════════════════════════
#  接線與關窗
# ═══════════════════════════════════════════════════════════════════════
def test_close_event_waits_for_a_running_profile_worker():
    class W:
        def __init__(self):
            self.calls = []

        def isRunning(self):
            return True

        def stop(self):
            self.calls.append("stop")

        def wait(self, ms):
            self.calls.append(("wait", ms))
            return True

    class Ev:
        accepted = False

        def accept(self):
            self.accepted = True

        def ignore(self):
            pass

    fake = types.SimpleNamespace(_wind_profile_worker=W(), _sitl_links=[], _sitl_launcher=None, current_mission=None)
    ev = Ev()
    MainWindow.closeEvent(fake, ev)
    assert fake._wind_profile_worker.calls == ["stop", ("wait", 30000)] and ev.accepted


def test_panel_view_signal_is_wired_in_main_window():
    """接線在 MainWindow 建構時做（測試不建 MainWindow）：只能鎖原始碼。"""
    src = inspect.getsource(MainWindow)
    assert "wind_profile_view_changed.connect(self._on_wind_profile_view_changed)" in src
    assert "wind_grid_requested.connect(self._on_wind_grid_requested)" in src
    assert "planned_altitude_changed.connect(self._on_planned_altitude_changed)" in src
