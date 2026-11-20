"""「找平坦位置」的 GUI 這一層：背景執行緒、結果對話框、面板控制項、MainWindow 的平移／復原。

不建 MainWindow（QtWebEngine 在此環境建不起來）：用未綁定方法 + 假 self 跑**真的處理函式**。
"""
from __future__ import annotations

import os
import types

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine 再建 QApplication
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from ui.widgets.flat_candidates_dialog import FlatCandidatesDialog, row_texts, summarize
    from ui.workers.flat_search_worker import FlatSearchWorker
    from PyQt6.QtWidgets import QApplication
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

from tests.test_flat_finder import CENTER, M_LAT, M_LON, Terr, hills, with_flat_block  # noqa: E402

TOKEN = "fake.token.value"


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


def square(c=CENTER, half_m=300.0):
    dn, de = half_m / M_LAT, half_m / M_LON
    return [(c[0] + dn, c[1] - de), (c[0] - dn, c[1] - de), (c[0] - dn, c[1] + de), (c[0] + dn, c[1] + de)]


class FakeWT(Terr):
    attribution_text = "Fake data © Foo Survey"


def payload(*, fits=True, scale=1.0, cands=None):
    need = {"A": 105.0 + 3.0, "B": 95.0 + 3.0} if fits else {"A": 135.0, "B": 125.0}
    cs = cands or [{"rank": 1, "lat": 25.01, "lon": 121.02, "relief_m": 3.0 if fits else 30.0, "z_min": 10.0, "z_max": 13.0,
                    "distance_km": 2.5, "need_m": dict(need), "fits": {k: v <= 120.0 for k, v in need.items()}, "ceil_m": 120.0},
                   {"rank": 2, "lat": 25.02, "lon": 121.03, "relief_m": 9.0, "z_min": 10.0, "z_max": 19.0,
                    "distance_km": 4.0, "need_m": {"A": 114.0, "B": 104.0}, "fits": {"A": True, "B": True}, "ceil_m": 120.0}]
    if not fits and cands is None:
        cs = cs[:1]                                   # 「沒有任何位置容得下」⇒ 只剩那個容不下的
    return {"candidates": cs, "window_h_m": 600.0, "window_w_m": 600.0, "step_m": 30.0, "cells": 1000, "ceil_m": 120.0,
            "here_relief_m": 64.0, "water_fraction": 0.0, "notes": [], "request": {"scale": scale},
            "any_fits": {"A": any(c["fits"]["A"] for c in cs), "B": any(c["fits"]["B"] for c in cs)},
            "attribution": "Fake data © Foo Survey"}


# ═══════════════════════════ 背景執行緒 ═══════════════════════════
def _run_worker(monkeypatch, terr, req_over=None):
    monkeypatch.setattr("core.terrain.CesiumWorldTerrain", lambda token, **k: terr)
    req = {"areas": [square()], "radius_km": 4.0, "scale": 1.0, "min_fw_agl_m": {"A": 100.0, "B": 90.0},
           "dh_band_min_m": 5.0, "clearance_m": 36.0, "ceil_m": 120.0, **(req_over or {})}
    w = FlatSearchWorker(TOKEN, req)
    got = {"ok": [], "fail": [], "prog": []}
    w.finished_ok.connect(got["ok"].append)
    w.failed.connect(got["fail"].append)
    w.progress.connect(got["prog"].append)
    w.run()                                  # 同步呼叫 run()（不開執行緒）：測的是執行緒裡跑的那段邏輯
    return w, got


def test_worker_emits_a_gui_ready_payload(monkeypatch, qt_app):
    _, got = _run_worker(monkeypatch, FakeWT(with_flat_block()))
    assert len(got["ok"]) == 1 and not got["fail"] and got["prog"]
    p = got["ok"][0]
    assert p["any_fits"] == {"A": True, "B": True} and p["candidates"][0]["fits"]["B"]
    assert p["attribution"] == "Fake data © Foo Survey"
    assert p["request"]["radius_km"] == 4.0 and "areas" not in p["request"]
    assert p["window_here_h_m"] == pytest.approx(600.0, rel=1e-3)
    assert TOKEN not in repr(p)


def test_worker_scale_shrinks_the_search_window(monkeypatch, qt_app):
    _, got = _run_worker(monkeypatch, FakeWT(hills), {"scale": 0.5})
    assert got["ok"][0]["window_h_m"] == pytest.approx(300.0, rel=1e-3)


def test_worker_reports_failures_on_the_failed_signal_and_never_leaks_the_token(monkeypatch, qt_app):
    class Boom(FakeWT):
        def prefetch(self, b, **k):
            from core.terrain import WorldTerrainError
            raise WorldTerrainError("Cesium ion 拒絕這個 token（HTTP 401）")
    _, got = _run_worker(monkeypatch, Boom(hills))
    assert not got["ok"] and got["fail"] == ["Cesium ion 拒絕這個 token（HTTP 401）"]
    _, got = _run_worker(monkeypatch, FakeWT(lambda la, lo: np.full(np.shape(la), 18.0)), {"radius_km": 1.0})
    assert not got["ok"] and "陸地窗口" in got["fail"][0]
    _, got = _run_worker(monkeypatch, FakeWT(hills), {"areas": [[(25.0, 121.0), (25.1, 121.1)]]})   # 角點不足
    assert not got["ok"] and got["fail"]
    for msg in got["fail"]:
        assert TOKEN not in msg


def test_an_abandoned_worker_stays_silent(monkeypatch, qt_app):
    monkeypatch.setattr("core.terrain.CesiumWorldTerrain", lambda token, **k: FakeWT(with_flat_block()))
    w = FlatSearchWorker(TOKEN, {"areas": [square()], "radius_km": 4.0})
    got = []
    w.finished_ok.connect(got.append)
    w.failed.connect(got.append)
    w.stop()
    w.run()
    assert got == []


# ═══════════════════════════ 結果對話框 ═══════════════════════════
def test_summary_says_which_case_applies():
    assert "✔" in summarize(payload(fits=True)) and "Pioneer" in summarize(payload(fits=True))
    s = summarize(payload(fits=False))
    assert "⚠" in s and "沒有任何位置能不用豁免" in s and "HERO" in s and "125" in s and "調小" in s
    assert "沒有完整的陸地窗口" in summarize({**payload(), "candidates": [], "any_fits": {}})
    assert "目前位置起伏 64 m" in summarize(payload()) and "50%" in summarize(payload(scale=0.5))


def test_row_texts_show_the_numbers_a_person_needs():
    r = row_texts(payload()["candidates"][0])
    assert r[0] == "1" and r[1] == "25.0100, 121.0200" and r[2] == "2.5 km" and r[3] == "3 m"
    assert "Pioneer 108" in r[4] and "HERO 98" in r[4]
    assert r[5] == "Pioneer、HERO" and r[6] == "+22 m"
    assert row_texts(payload(fits=False)["candidates"][0])[5] == "—"


def test_dialog_selects_the_first_row_and_emits_the_chosen_position(qt_app):
    d = FlatCandidatesDialog(payload(scale=0.5))
    got = []
    d.apply_requested.connect(lambda *a: got.append(a))
    assert d.table.rowCount() == 2 and d.btn_apply.isEnabled()
    d.btn_apply.click()
    assert got == [(25.01, 121.02, 0.5)]
    d.table.selectRow(1)
    d.btn_apply.click()
    assert got[-1] == (25.02, 121.03, 0.5)
    assert "縮成 50%" in d.btn_apply.text()


def test_dialog_without_candidates_cannot_apply_and_undo_follows_state(qt_app):
    d = FlatCandidatesDialog({**payload(), "candidates": [], "any_fits": {}})
    assert not d.btn_apply.isEnabled() and not d.btn_undo.isEnabled()
    undo = []
    d.undo_requested.connect(lambda: undo.append(1))
    d.set_can_undo(True)
    d.btn_undo.click()
    assert undo == [1] and d.btn_undo.isEnabled()


# ═══════════════════════════ 面板 ═══════════════════════════
@pytest.fixture
def panel(qt_app, monkeypatch):
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("a.b.c", "測試"))
    return VTOLReconPanel()


def _arm(p):
    p.set_corner_count(4)
    p._footprint_ok = True
    p._sync_enabled()


def test_find_button_needs_an_area_and_a_token(panel, qt_app, monkeypatch):
    assert not panel.btn_find_flat.isEnabled()            # 還沒圈區
    _arm(panel)
    assert panel.btn_find_flat.isEnabled()
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("", ""))
    q = VTOLReconPanel()
    _arm(q)
    assert not q.btn_find_flat.isEnabled()                # 沒 token


def test_committed_areas_alone_also_enable_the_button(panel, qt_app):
    panel.set_area_count(1, 0)
    panel._footprint_ok = True
    panel._sync_enabled()
    assert panel.btn_find_flat.isEnabled()


def test_the_request_carries_the_panel_settings(panel, qt_app):
    panel.hv_flat_radius_spin.setValue(12.0)
    panel.hv_flat_scale_spin.setValue(70.0)
    panel.hv_minfw_a_spin.setValue(100.0)
    panel.hv_minfw_b_spin.setValue(90.0)
    panel.hv_band_spin.setValue(5.0)
    r = panel.get_flat_request()
    assert r == {"radius_km": 12.0, "scale": 0.7, "min_fw_agl_m": {"A": 100.0, "B": 90.0}, "dh_band_min_m": 5.0,
                 "clearance_m": 36.0, "ceil_m": 120.0}
    panel.hv_clearance_spin.setValue(50.0)
    panel.hv_waiver_check.setChecked(True)
    panel.hv_waiver_spin.setValue(200.0)
    r = panel.get_flat_request()
    assert r["clearance_m"] == 50.0 and r["ceil_m"] == 200.0          # 有豁免就用豁免值；淨空 0=自動 ⇒ 36


def test_clicking_emits_the_request(panel, qt_app):
    _arm(panel)
    got = []
    panel.find_flat_requested.connect(got.append)
    panel.btn_find_flat.click()
    assert got == [panel.get_flat_request()]


def test_busy_state_disables_the_button_and_shows_text(panel, qt_app):
    _arm(panel)
    panel.set_flat_busy(True, "掃描中…")
    assert not panel.btn_find_flat.isEnabled() and panel.flat_status_label.text() == "掃描中…"
    panel._sync_enabled()                                  # 其他事件觸發重新同步時，忙碌中不得被重新啟用
    assert not panel.btn_find_flat.isEnabled()
    panel.set_flat_busy(False, "")
    assert panel.btn_find_flat.isEnabled()


def test_result_dialog_relays_apply_and_undo_through_the_panel(panel, qt_app):
    _arm(panel)
    applied, undone = [], []
    panel.flat_apply_requested.connect(lambda *a: applied.append(a))
    panel.flat_undo_requested.connect(lambda: undone.append(1))
    panel.show_flat_result(payload(), can_undo=True)
    d = panel._flat_dialog
    assert d is not None and d.btn_undo.isEnabled() and panel.btn_find_flat.isEnabled()
    d.btn_apply.click()
    d.btn_undo.click()
    assert applied == [(25.01, 121.02, 1.0)] and undone == [1]
    panel.set_flat_can_undo(False)
    assert not d.btn_undo.isEnabled()
    panel.show_flat_result(payload())                      # 再開一個：舊的被取代、不累積
    assert panel._flat_dialog is not d


# ═══════════════════════════ MainWindow（假 self）═══════════════════════════
class _Rec:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)

        def f(*a, **k):
            self.calls.append((name, a))
        return f


def make_win(*, areas=None, corners=None, home=None, token="a.b.c", worker_running=False):
    w = types.SimpleNamespace()
    w.survey_areas = [list(a) for a in (areas or [])]
    w.corners = list(corners or [])
    w.home_point = home
    w.map_widget, w.parameter_panel, w.vtol_recon_panel, w.msgs = _Rec(), _Rec(), _Rec(), []
    w.statusBar = lambda: types.SimpleNamespace(showMessage=lambda m, *a: w.msgs.append(m))
    w.on_home_point_set = lambda la, lo: (w.map_widget.calls.append(("home_set", (la, lo))), setattr(w, "home_point", (la, lo)))
    w.on_clear_home_point = lambda: (w.map_widget.calls.append(("home_clear", ())), setattr(w, "home_point", None))
    w.on_clear_paths = lambda: w.map_widget.calls.append(("clear_paths", ()))
    w._render_committed_areas = lambda: w.map_widget.calls.append(("render_areas", ()))
    w._sync_area_counts = lambda: w.map_widget.calls.append(("sync_counts", ()))
    w.update_statusbar = lambda: None
    for name in ("survey_area_polygons", "_flat_redraw_areas", "_on_vtol_recon_apply_flat", "_on_vtol_recon_undo_flat",
                 "_on_vtol_recon_find_flat", "_on_flat_search_done", "_on_flat_search_failed"):
        setattr(w, name, types.MethodType(getattr(MainWindow, name), w))
    return w


AREA1 = square((25.0, 121.0), 500.0)
AREA2 = square((25.02, 121.03), 300.0)
CUR = [(25.05, 121.05), (25.05, 121.06), (25.06, 121.06)]


def _center(pts):
    return (np.mean([p[0] for p in pts]), np.mean([p[1] for p in pts]))


def test_apply_moves_every_area_the_drawing_corners_and_the_launch_site_together(qt_app):
    w = make_win(areas=[AREA1, AREA2], corners=CUR, home=(25.001, 121.002))
    allpts_before = [p for a in w.survey_area_polygons() for p in a]
    lo0, lo1 = min(p[1] for p in allpts_before), max(p[1] for p in allpts_before)
    w._on_vtol_recon_apply_flat(24.5, 120.5, 1.0)
    allpts = [p for a in w.survey_area_polygons() for p in a]
    # 聯集外框中心搬到目標
    assert (min(p[0] for p in allpts) + max(p[0] for p in allpts)) / 2 == pytest.approx(24.5)
    assert (min(p[1] for p in allpts) + max(p[1] for p in allpts)) / 2 == pytest.approx(120.5)
    # 形狀不變（縮放 1.0）：總外框寬不變
    assert max(p[1] for p in allpts) - min(p[1] for p in allpts) == pytest.approx(lo1 - lo0)
    # 相對位置不變：第二塊相對第一塊的偏移不變、起降場相對第一塊的偏移不變
    c1, c2 = _center(w.survey_areas[0]), _center(w.survey_areas[1])
    assert (c2[0] - c1[0], c2[1] - c1[1]) == pytest.approx((0.02, 0.03))
    assert w.home_point[0] - c1[0] == pytest.approx(0.001) and w.home_point[1] - c1[1] == pytest.approx(0.002)
    # 地圖與狀態都同步
    names = [c[0] for c in w.map_widget.calls]
    assert names[0] == "clear_corners" or "clear_corners" in [c[0] for c in w.map_widget.calls]
    assert [c for c in w.map_widget.calls if c[0] == "clear_paths"]
    assert [c for c in w.map_widget.calls if c[0] == "home_set"]
    assert len(w.corners) == 3 and w.vtol_recon_panel.calls[-1] == ("set_flat_can_undo", (True,))


def test_apply_redraws_the_map_in_the_right_order(qt_app):
    w = make_win(areas=[AREA1], corners=CUR)
    seq = []
    w.map_widget.clear_corners = lambda: seq.append("clear")
    w.map_widget.add_corner = lambda la, lo: seq.append(("add", round(la, 4), round(lo, 4)))
    w.map_widget.fly_to_position = lambda *a: seq.append("fly")
    w._on_vtol_recon_apply_flat(24.0, 120.0, 1.0)
    assert seq[0] == "clear" and [s[0] for s in seq[1:4]] == ["add"] * 3 and seq[-1] == "fly"
    assert [(s[1], s[2]) for s in seq[1:4]] == [(round(p[0], 4), round(p[1], 4)) for p in w.corners]


def test_apply_with_a_scale_shrinks_the_areas_about_their_centre(qt_app):
    w = make_win(areas=[AREA1])
    before = max(p[0] for p in AREA1) - min(p[0] for p in AREA1)
    w._on_vtol_recon_apply_flat(24.0, 120.0, 0.5)
    after = max(p[0] for p in w.survey_areas[0]) - min(p[0] for p in w.survey_areas[0])
    assert after == pytest.approx(0.5 * before)
    assert any("縮成 50%" in m for m in w.msgs)


def test_undo_restores_everything_exactly_and_forgets_the_snapshot(qt_app):
    w = make_win(areas=[AREA1, AREA2], corners=CUR, home=(25.001, 121.002))
    w._on_vtol_recon_apply_flat(24.5, 120.5, 0.6)
    assert w.survey_areas[0] != AREA1
    w._on_vtol_recon_undo_flat()
    assert w.survey_areas == [AREA1, AREA2] and w.corners == CUR and w.home_point == (25.001, 121.002)
    assert w._flat_undo is None and w.vtol_recon_panel.calls[-1] == ("set_flat_can_undo", (False,))
    w.msgs.clear()
    w._on_vtol_recon_undo_flat()                      # 沒有快照：什麼都不做
    assert w.msgs == [] and w.survey_areas == [AREA1, AREA2]


def test_undo_clears_the_launch_site_when_there_was_none(qt_app):
    w = make_win(areas=[AREA1])
    w._on_vtol_recon_apply_flat(24.0, 120.0, 1.0)
    w._on_vtol_recon_undo_flat()
    assert w.home_point is None and not [c for c in w.map_widget.calls if c[0] == "home_set"]


def test_apply_without_any_area_does_nothing(qt_app):
    w = make_win()
    w._on_vtol_recon_apply_flat(24.0, 120.0, 1.0)
    assert not hasattr(w, "_flat_undo") and w.map_widget.calls == []


def test_a_second_apply_keeps_only_the_latest_snapshot(qt_app):
    w = make_win(areas=[AREA1])
    w._on_vtol_recon_apply_flat(24.0, 120.0, 1.0)
    first = [list(a) for a in w.survey_areas]
    w._on_vtol_recon_apply_flat(23.0, 119.0, 1.0)
    w._on_vtol_recon_undo_flat()
    assert w.survey_areas == first                    # 復原的是「上一次」的狀態


# ── 按下「找平坦位置」──
class FakeWorker:
    instances: list = []

    def __init__(self, token, req, parent=None):
        self.token, self.req, self.parent, self.started = token, req, parent, False
        self.finished_ok = types.SimpleNamespace(connect=lambda f: setattr(self, "ok_cb", f))
        self.failed = types.SimpleNamespace(connect=lambda f: setattr(self, "fail_cb", f))
        FakeWorker.instances.append(self)

    def start(self):
        self.started = True

    def isRunning(self):
        return False

    def deleteLater(self):
        pass


@pytest.fixture
def fake_worker(monkeypatch):
    FakeWorker.instances = []
    monkeypatch.setattr("ui.workers.flat_search_worker.FlatSearchWorker", FakeWorker)
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("a.b.c", "測試"))
    return FakeWorker


def test_find_hands_the_areas_and_the_token_to_a_background_worker(fake_worker, qt_app):
    w = make_win(areas=[AREA1], corners=CUR)
    w._on_vtol_recon_find_flat({"radius_km": 15.0, "scale": 0.7})
    wk, = fake_worker.instances
    assert wk.started and wk.token == "a.b.c" and wk.parent is None
    assert wk.req["areas"] == [AREA1, CUR] and wk.req["radius_km"] == 15.0 and wk.req["scale"] == 0.7
    assert w.vtol_recon_panel.calls[0][0] == "set_flat_busy" and w.vtol_recon_panel.calls[0][1][0] is True


def test_find_without_an_area_or_a_token_explains_instead_of_starting(fake_worker, qt_app, monkeypatch):
    w = make_win()
    w._on_vtol_recon_find_flat({})
    assert not fake_worker.instances and "請先圈好作業區" in w.vtol_recon_panel.calls[-1][1][1]
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("", ""))
    w2 = make_win(areas=[AREA1])
    w2._on_vtol_recon_find_flat({})
    assert not fake_worker.instances and "token" in w2.vtol_recon_panel.calls[-1][1][1]


def test_find_ignores_a_second_press_while_one_is_running(fake_worker, qt_app):
    w = make_win(areas=[AREA1])
    w._flat_worker = types.SimpleNamespace(isRunning=lambda: True)
    w._on_vtol_recon_find_flat({})
    assert not fake_worker.instances and any("還在執行" in m for m in w.msgs)


def test_results_and_failures_reach_the_panel(fake_worker, qt_app):
    w = make_win(areas=[AREA1])
    w._flat_undo = {"x": 1}
    w._on_flat_search_done(payload())
    assert w.vtol_recon_panel.calls[-1][0] == "show_flat_result"
    assert w.vtol_recon_panel.calls[-1][1][0]["any_fits"]["B"] is True
    w._on_flat_search_failed("連線失敗（ConnectionError）")
    assert w.vtol_recon_panel.calls[-1] == ("set_flat_busy", (False, "⚠ 連線失敗（ConnectionError）"))
    assert any("找平坦位置失敗" in m for m in w.msgs)
