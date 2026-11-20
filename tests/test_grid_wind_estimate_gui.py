"""網格風重估接進 GUI（2026-10-21）：抓網格的範圍、規劃完成／網格抵達時重估、過時判斷、問題進橫幅。

不建 MainWindow（QtWebEngine 在這裡建不起來）：MainWindow 的方法用 types.MethodType 綁到 FakeWin（同 tests/test_wind_gui.py、
tests/test_wind_profile_gui.py）。計畫是真規劃器的契約 mini 情境（A1 Pioneer＋B1 HERO、規劃風 5 m/s 來自 270°）；網格是用
**要求的格點**自己組的 Open-Meteo 回應，分層風是 2026-10-19 錄製的 icon_global 回應（放到要求的格點上）。
不連網（worker 換成假的、不啟動）、不寫檔（AEROPLAN_NO_FILE_LOG）、不碰 config/settings.json。
"""
from __future__ import annotations

import copy
import dataclasses
import inspect
import json
import math
import os
import types
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")

from core import grid_wind_estimate as gwe                        # noqa: E402
from core import wind_field as wf                                 # noqa: E402
import ui.main_window as mw                                       # noqa: E402
from ui.main_window import MainWindow                             # noqa: E402

FIX = Path(__file__).resolve().parent / "fixtures"
MARGIN_M = MainWindow.WIND_GRID_MARGIN_M


# ═══════════════════════════════════════════════════════════════════════
#  真計畫（契約 mini 情境）
# ═══════════════════════════════════════════════════════════════════════
@pytest.fixture(scope="module")
def mini():
    from hv_dccpp.context import build_context
    from hv_dccpp.planner import plan_mission
    from hv_dccpp.scenario import build_mini_example
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    mp = plan_mission(inputs, cfg, ctx=ctx)
    assert set(mp.plans) == {"A1", "B1"}
    area = [tuple(p) for p in inputs.areas[0].polygon_wgs84]
    return types.SimpleNamespace(mp=mp, ctx=ctx, area=area)


def _snapshot(plan):
    """計畫的值（不改計畫：前後要逐值相等）。"""
    return {u: ([(w.t_s, w.lat_deg, w.lon_deg, w.z_m, w.airspeed_mps) for w in p.waypoints],
                p.t_takeoff_s, p.t_land_s, p.energy_used_wh, p.energy_margin_wh) for u, p in plan.plans.items()}


# ═══════════════════════════════════════════════════════════════════════
#  假 self
# ═══════════════════════════════════════════════════════════════════════
_BOUND = ("_wind_state", "_wind_display", "_wind_limit_mps", "_wind_area_bbox", "_on_wind", "_wind_arrow_rows",
          "_push_wind_arrows", "_refresh_wind_status", "_refresh_wind_field", "_apply_wind_options",
          "_on_wind_display_changed", "_on_wind_grid_requested", "_on_wind_grid_done", "_on_wind_grid_failed",
          "_on_wind_grid_clear", "survey_area_polygons", "_wind_loaded", "_wind_near_point", "_apply_wind_to_plan",
          "_wind_note", "_start_wind_profile_fetch", "_wind_profile_heights_of", "_wind_planned_alt_m",
          "_wind_profile_h2d", "_wind_profile_centre", "_wind_profile_payload", "_wind_profile_status",
          "_push_wind_profile", "_on_wind_profile_done", "_on_wind_profile_failed", "_drop_wind_profile",
          "_on_wind_profile_view_changed", "_apply_profile_wind_to_plan", "_profile_plan_not_written",
          "_on_planned_altitude_changed",
          # 規劃完成這條路
          "_hv_show_report_banner", "_hv_notify_done", "_on_hv_finished", "on_clear_paths", "_on_vtol_recon_planned")


class FakeMap:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):                       # set_wind_field／clear_wind_arrows／clear_paths …：只記下來
        if name.startswith(("set_", "clear_")):
            return lambda *a, **k: self.calls.append((name, a[0] if a else k))
        raise AttributeError(name)

    def last(self, name):
        return [c[1] for c in self.calls if c[0] == name][-1]


class _Spin:
    def __init__(self, v):
        self._v = v

    def value(self):
        return self._v


class FakePanel:
    def __init__(self, *, apply_plan=False):
        self.disp = {"plan": False, "live": True, "speed_mps": 0.0, "dir_from_deg": 0.0, "grid_height": "10m",
                     "profile_height_2d_m": 120.0, "profile_height_2d_user": True, "apply_plan": apply_plan,
                     "cwa_station": ""}
        self.hv_map_max_wind_spin = _Spin(3.0)
        self.status, self.busy, self.loaded, self.applied, self.plan_results = [], [], [], [], []

    def get_wind_display(self):
        return dict(self.disp)

    def effective_altitude_m(self):
        return 120.0

    def set_wind_status(self, t):
        self.status.append(t)

    def set_wind_grid_busy(self, b, t=None):
        self.busy.append((b, t))

    def set_wind_grid_loaded(self, b, t=None):
        self.loaded.append((b, t))

    def set_wind_profile_loaded(self, b):
        pass

    def set_plan_result(self, rows, summary):
        self.plan_results.append((rows, summary))

    def apply_wind_to_plan(self, s, d, source):
        self.disp["speed_mps"], self.disp["dir_from_deg"] = round(float(s), 1), float(round(d % 360))
        self.applied.append((s, d, source))
        return {"speed_mps": self.disp["speed_mps"], "dir_from_deg": self.disp["dir_from_deg"], "clamped": False,
                "text": f"規劃風已寫入：{self.disp['speed_mps']:.1f} m/s 來自 {self.disp['dir_from_deg']:.0f}°（來源：{source}）"}


class _Label:
    def __init__(self):
        self.text = None

    def setText(self, t):
        self.text = t


class FakeWin:
    def __init__(self, *, areas=(), apply_plan=False):
        self.survey_areas, self.corners = [list(a) for a in areas], []
        self.vtol_recon_panel = FakePanel(apply_plan=apply_plan)
        self.map_widget = FakeMap()
        self.msgs = []
        self._sitl_links = []
        self.in_flight = False
        self._hv_planning_in_flight = lambda: self.in_flight
        # on_clear_paths 會碰的
        self.waypoints, self.spiral_waypoint_altitudes = [1, 2], {1: 2}
        self.waypoint_label, self.distance_label = _Label(), _Label()
        self._get_cesium_widget = lambda: None
        for name in _BOUND:
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))
        for k in ("WIND_LIVE_PUSH_INTERVAL_S", "WIND_LIVE_STALE_S", "WIND_PLAN_SYSID", "WIND_GRID_MARGIN_M",
                  "WIND_CWA_SYSID", "WIND_PROFILE_PARTICLES_TOTAL"):
            setattr(self, k, getattr(MainWindow, k))

    def statusBar(self):
        return types.SimpleNamespace(showMessage=lambda m, *a: self.msgs.append(m), clearMessage=lambda: None)

    @property
    def status(self) -> str:
        return self.vtol_recon_panel.status[-1] if self.vtol_recon_panel.status else ""


def _hv(mini, plan=None):
    """HvDccppCoordinator 的替身：last_result 就是畫面上那份結果（_dccpp_result）。"""
    return types.SimpleNamespace(last_plan=plan or mini.mp, last_ctx=mini.ctx, last_result={"assembled_paths": {}})


def _win(mini, *, plan=True, **kw):
    w = FakeWin(areas=[mini.area], **kw)
    if plan:
        w._hv_coordinator = _hv(mini)
        w._dccpp_result = w._hv_coordinator.last_result
    return w


def _om_field(bbox, speed, dir_from, n=mw.WIND_FETCH_GRID_N):
    """Open-Meteo 10 m 多點回應（每點同一個風），格點＝要求的格點（同 fetch_open_meteo）。"""
    lats, lons = wf.grid_points(bbox, n, n)
    items = [{"latitude": la, "longitude": lo, "current": {"time": "2026-10-23T07:00", "wind_speed_10m": speed,
                                                           "wind_direction_10m": dir_from, "wind_gusts_10m": 1.3 * speed}}
             for la in lats for lo in lons]
    return wf.parse_open_meteo(items, lats, lons, "10m")


def _recorded_profile(bbox):
    """2026-10-19 錄製的 icon_global 2×3 分層回應，放到 bbox 的 3×2 格點上（同 fetch_open_meteo_profile 的 bbox）。"""
    payload = json.loads((FIX / "open_meteo_profile_icon_global_2x3_20261019T06.json").read_text(encoding="utf-8"))
    lats, lons = wf.grid_points(bbox, 3, 2)
    p = wf.parse_open_meteo_profile(payload, lats, lons, when_utc="2026-10-19T06:00")
    return dataclasses.replace(p, bbox=tuple(float(x) for x in bbox))


class FakeWorker:
    started = []

    def __init__(self, bbox, *, parent=None, **kw):
        self.bbox, self.kw = bbox, kw
        self.finished_ok = types.SimpleNamespace(connect=lambda f: None)
        self.failed = types.SimpleNamespace(connect=lambda f: None)

    def isRunning(self):
        return False

    def start(self):
        FakeWorker.started.append(self)

    def deleteLater(self):
        pass


@pytest.fixture
def workers(monkeypatch):
    import ui.workers.wind_grid_worker as wgw
    FakeWorker.started = []
    monkeypatch.setattr(wgw, "WindGridWorker", FakeWorker)
    monkeypatch.setattr(wgw, "WindProfileWorker", FakeWorker)
    return FakeWorker.started


def _fetch_grid(w, workers, speed, dir_from):
    """按「抓氣象網格」（10 m）→ 假 worker 記下範圍 → 用那個範圍組回應 → 結果送回處理函式。"""
    w._on_wind_grid_requested({"height": "10m"})
    bbox = workers[-1].bbox
    field = _om_field(bbox, speed, dir_from)
    w._on_wind_grid_done(field)
    return bbox, field


def _inside(lat, lon, bb, shrink_m=0.0):
    dlat = shrink_m / 111_320.0
    dlon = shrink_m / (111_320.0 * math.cos(math.radians(0.5 * (bb[0] + bb[2]))))
    return bb[0] + dlat <= lat <= bb[2] - dlat and bb[1] + dlon <= lon <= bb[3] - dlon


# ═══════════════════════════════════════════════════════════════════════
#  1. 抓網格的範圍
# ═══════════════════════════════════════════════════════════════════════
def test_request_point_count_is_the_workers_default_and_within_the_open_meteo_cap():
    from ui.workers.wind_grid_worker import WindGridWorker, WindProfileWorker
    for cls in (WindGridWorker, WindProfileWorker):
        ps = inspect.signature(cls.__init__).parameters
        assert ps["nx"].default == ps["ny"].default == mw.WIND_FETCH_GRID_N, cls.__name__
    assert mw.WIND_FETCH_GRID_N ** 2 <= wf.OPEN_METEO_MAX_POINTS


def test_without_a_plan_the_range_is_the_area_margin_as_before(mini):
    w = _win(mini, plan=False)
    sc = mw.wind_fetch_scope(w, MARGIN_M)
    assert sc["kind"] == "area" and sc["bbox"] == pytest.approx(w._wind_area_bbox())
    assert sc["area_bbox"] == pytest.approx(w._wind_area_bbox())
    assert sc["text"].startswith("網格範圍：作業區外擴 500 m（沒有 HARVEST 計畫；規劃完成後再抓會改成涵蓋整條航線與起降場）")
    assert "6×6 點" in sc["text"]
    assert mw.wind_fetch_scope(FakeWin(), MARGIN_M) is None, "沒作業區也沒計畫 ⇒ 沒有範圍"


def test_with_a_plan_the_range_covers_every_waypoint_and_pad_with_margin(mini):
    w = _win(mini)
    area_bb = w._wind_area_bbox()
    wps = [x for p in mini.mp.plans.values() for x in p.waypoints]
    # 前提（就是要修的問題）：只抓作業區外擴 500 m 時，有航點（起降轉移）在範圍外
    assert any(not _inside(x.lat_deg, x.lon_deg, area_bb) for x in wps)
    sc = mw.wind_fetch_scope(w, MARGIN_M)
    assert sc["kind"] == "plan" and sc["area_bbox"] == pytest.approx(area_bb)
    bb = sc["bbox"]
    assert all(_inside(x.lat_deg, x.lon_deg, bb, shrink_m=0.99 * MARGIN_M) for x in wps), "每個航點外擴 500 m 都在範圍內"
    h_xy, h_z = mini.ctx.sites_enu["H"]
    hla, hlo, _ = mini.ctx.frame.to_wgs84([float(h_xy[0]), float(h_xy[1]), float(h_z)])[0]
    assert _inside(hla, hlo, bb, shrink_m=0.99 * MARGIN_M), "起降場 H 在範圍內"
    assert _inside(area_bb[0], area_bb[1], bb) and _inside(area_bb[2], area_bb[3], bb), "作業區範圍也在裡面"
    w_km, h_km = mw._bbox_size_km(bb)
    assert sc["text"] == (f"網格範圍：作業區＋目前 HARVEST 計畫的全部 {len(wps)} 個航點與起降場（H），外擴 500 m；"
                          f"約 {w_km:.1f} × {h_km:.1f} km，6×6 點（點距約 {w_km / 5:.2f} × {h_km / 5:.2f} km）")
    # 沒有圈作業區、只有計畫：照樣涵蓋航線（剖面柱就退回範圍中心）
    w2 = _win(mini)
    w2.survey_areas = []
    sc2 = mw.wind_fetch_scope(w2, MARGIN_M)
    assert sc2["kind"] == "plan" and sc2["area_bbox"] is None
    assert all(_inside(x.lat_deg, x.lon_deg, sc2["bbox"], shrink_m=0.99 * MARGIN_M) for x in wps)


def test_extent_lists_the_pads_each_aircraft_uses_even_off_its_waypoints(mini):
    """起降場不只靠「航點剛好在坪上」：計畫裡寫的起飛／降落站都要進範圍（用 ctx.sites_enu 經 ctx.frame 轉）。"""
    a1 = mini.mp.plans["A1"]
    mid = a1.waypoints[len(a1.waypoints) // 2]
    plan = types.SimpleNamespace(plans={"A1": types.SimpleNamespace(waypoints=[mid], takeoff_site="H",
                                                                   landing_site="ALT1")})
    pts, n_wp, used = mw.harvest_plan_extent_points(plan, mini.ctx)
    assert n_wp == 1 and used == ["ALT1", "H"]
    for sid in used:
        xy, z = mini.ctx.sites_enu[sid]
        la, lo, _ = mini.ctx.frame.to_wgs84([float(xy[0]), float(xy[1]), float(z)])[0]
        assert any(abs(p[0] - la) < 1e-9 and abs(p[1] - lo) < 1e-9 for p in pts), sid
    assert len(pts) == 3


def test_dccpp_plan_or_a_result_that_is_not_the_harvest_one_falls_back_to_the_area(mini):
    w = _win(mini)
    w._dccpp_result = {"assembled_paths": {}}                 # 畫面上換成別的結果（DCCPP）
    assert mw.harvest_display_plan(w) is None
    assert mw.wind_fetch_scope(w, MARGIN_M)["kind"] == "area"


def test_grid_and_profile_requests_fetch_the_plan_range_and_say_so(mini, workers):
    w = _win(mini)
    sc = mw.wind_fetch_scope(w, MARGIN_M)
    w._on_wind_grid_requested({"height": "120m"})
    assert workers[-1].bbox == pytest.approx(sc["bbox"]) and workers[-1].kw == {"height": "120m"}
    assert w.status.startswith("抓取 Open-Meteo 120m 風場中…（要網路，約 1–3 秒）；網格範圍：作業區＋目前 HARVEST 計畫")
    w._on_wind_grid_failed("連不到 Open-Meteo")
    assert "fetch_scope" not in w._wind_state(), "失敗了 ⇒ 範圍不留給之後的結果"
    w._on_wind_grid_requested({"height": "profile"})
    assert workers[-1].bbox == pytest.approx(sc["bbox"])
    assert "分層風（10／80／120 m" in w.status and "網格範圍：作業區＋目前 HARVEST 計畫" in w.status
    w._on_wind_profile_failed("連不到 Open-Meteo")
    assert "fetch_scope" not in w._wind_state()


def test_a_result_over_another_range_is_not_labelled_with_this_request(mini, workers):
    """範圍說明只給「就是那次抓的」結果（格點頭尾＝請求的 bbox）；對不上 ⇒ 不寫範圍（不冒充）。"""
    w = _win(mini)
    w._on_wind_grid_requested({"height": "10m"})
    other = tuple(x + 0.05 for x in workers[-1].bbox)
    w._on_wind_grid_done(_om_field(other, 5.0, 270.0))
    st = w._wind_state()
    assert st["src_scope"] is None and st["scope_text"] == "" and "網格範圍：" not in w.status
    w._on_wind_grid_requested({"height": "profile"})
    w._on_wind_profile_done(_recorded_profile(other))
    assert w._wind_state()["scope_text"] == "" and "網格範圍：" not in w.status


# ═══════════════════════════════════════════════════════════════════════
#  2. 網格抵達 ⇒ 重估（計畫不動）
# ═══════════════════════════════════════════════════════════════════════
def test_grid_equal_to_the_planning_wind_gives_zero_and_shows_range_and_summary(mini, workers):
    before = _snapshot(mini.mp)
    w = _win(mini)
    _bbox, field = _fetch_grid(w, workers, 5.0, 270.0)           # ＝規劃風（mini：5 m/s 來自 270°）
    st = w._wind_state()
    est = st["estimate"]
    assert est is not None and est.source_kind == "grid" and gwe.estimate_matches(est, mini.mp)
    assert est.makespan_grid_s == pytest.approx(est.makespan_plan_s, abs=1e-6)
    assert est.problems == [] and all(e.uncovered_m == 0.0 for e in est.uavs.values()), "範圍涵蓋整條航線 ⇒ 全在網格內"
    lines = est.summary_lines()
    status = w.status
    for ln in lines:
        assert ln in status
    assert "規劃器用的風：5.00 m/s 來自 270°" in status, "規劃器的生效值（ctx.wind），不是面板"
    assert "網格範圍：作業區＋目前 HARVEST 計畫的全部" in status, "抓取結束後範圍來源留在狀態列"
    assert "說明：單層 10 m 網格套用到所有高度" in status, "core 補的說明（網格本身的 notes 沒有）"
    assert "不在網格" not in status and "⚠" not in status.split("網格風重估")[1]
    assert w.msgs == [], "沒有問題就不跳狀態列"
    assert "_hv_last_report" not in w.__dict__, "沒有問題也沒有報告 ⇒ 不憑空建一份"
    # 值層級：跟直接呼叫 core 一樣
    direct = gwe.estimate_plan(mini.mp, mini.ctx, field)
    assert {u: e.t_land_grid_s for u, e in est.uavs.items()} == {u: e.t_land_grid_s for u, e in direct.uavs.items()}
    assert _snapshot(mini.mp) == before, "計畫（航點、時間、能量）一個值都不能動"


def test_area_only_range_leaves_transfers_outside_and_says_how_to_fix_it(mini, workers):
    """沒有計畫時抓的（作業區範圍），之後規劃出來 ⇒ 起降轉移不在網格內：照寫長度＋怎麼補；計畫的範圍重抓 ⇒ 0。"""
    w = _win(mini, plan=False)
    _fetch_grid(w, workers, 9.0, 90.0)
    assert w._wind_state()["estimate"] is None and "網格風重估" not in w.status, "還沒有計畫 ⇒ 不重估"
    assert "網格範圍：作業區外擴 500 m（沒有 HARVEST 計畫" in w.status
    w._hv_coordinator = _hv(mini)                                   # 規劃完成（同步路徑的那一行）
    w._dccpp_result = w._hv_coordinator.last_result
    est = mw.refresh_grid_wind_estimate(w, "規劃完成")
    unc = sum(e.uncovered_m for e in est.uavs.values())
    assert unc > 1000.0
    assert f"⚠ 共 {unc / 1000:.1f} km 航段不在網格範圍內，那幾段沿用規劃風（重估差 0）：" \
           "重抓氣象網格會改用涵蓋整條航線與起降場的範圍" in w.status
    assert "km 不在網格內（沿用規劃風）" in w.status, "每機的長度也照寫"
    _fetch_grid(w, workers, 9.0, 90.0)                              # 現在有計畫 ⇒ 範圍涵蓋整條航線
    est2 = w._wind_state()["estimate"]
    assert est2 is not est and all(e.uncovered_m == 0.0 for e in est2.uavs.values())
    assert "不在網格" not in w.status


def test_written_planning_wind_says_the_mean_covers_the_whole_route(mini, workers):
    w = _win(mini, apply_plan=True)
    _fetch_grid(w, workers, 4.0, 300.0)
    src = w.vtol_recon_panel.applied[-1][2]
    assert src.startswith("Open-Meteo 10 m 網格 36 格向量平均（範圍含整條 HARVEST 航線與起降場，不只作業區），")
    w0 = _win(mini, plan=False, apply_plan=True)
    _fetch_grid(w0, workers, 4.0, 300.0)
    assert w0.vtol_recon_panel.applied[-1][2].startswith("Open-Meteo 10 m 網格 36 格向量平均，2026-10-23T07:00 UTC"), \
        "沒有計畫 ⇒ 原本的說法"
    # 寫進面板的值不影響這份計畫的重估（規劃器的生效值是 ctx.wind）
    assert "規劃器用的風：5.00 m/s 來自 270°" in w.status


def test_profile_arrival_estimates_and_the_column_stays_at_the_area_centre(mini, workers):
    w = _win(mini)
    w._on_wind_grid_requested({"height": "profile"})
    bbox = workers[-1].bbox
    prof = _recorded_profile(bbox)
    w._on_wind_profile_done(prof)
    col = w.map_widget.last("set_wind_profile")["column"]
    ab = w._wind_area_bbox()
    assert (col["lat"], col["lon"]) == pytest.approx((0.5 * (ab[0] + ab[2]), 0.5 * (ab[1] + ab[3])))
    assert abs(col["lon"] - 0.5 * (bbox[1] + bbox[3])) > 0.005, "範圍中心（含起降轉移）不是作業區中心"
    st = w._wind_state()
    est = st["estimate"]
    assert est.source_kind == "profile" and est.source_heights_m == (10.0, 80.0, 120.0)
    direct = gwe.estimate_plan(mini.mp, mini.ctx, prof)
    assert est.makespan_grid_s == pytest.approx(direct.makespan_grid_s, abs=1e-9)
    assert est.summary_lines()[0] in w.status and "網格範圍：作業區＋目前 HARVEST 計畫" in w.status
    assert "說明：" not in w.status.split("網格風重估")[1], "分層風的 notes 已在分層風那一行，不重複"


# ═══════════════════════════════════════════════════════════════════════
#  3. 大聲：問題進橫幅
# ═══════════════════════════════════════════════════════════════════════
def test_problems_go_to_the_banner_report_and_status_and_clear_with_the_grid(mini, workers):
    w = _win(mini)
    w._hv_last_report = {"loud_notes": ["validate_failed(既有的)"], "uncovered": [], "status": "GO"}
    _fetch_grid(w, workers, 12.0, 90.0)                             # 規劃風反向且更強 ⇒ 時間偏移超過 σ_t
    est = w._wind_state()["estimate"]
    probs = est.problems
    assert probs and any("σ_t" in p for p in probs)
    assert w.msgs[-1].startswith("⚠ 網格風重估：" + probs[0])
    notes = w._hv_last_report["loud_notes"]
    assert notes == ["validate_failed(既有的)"] + [f"grid_wind_estimate({p})" for p in probs]
    for p in probs:
        assert f"⚠ {p}" in w.status
    # 橫幅（規劃完成時會叫的那支）也看得到
    w._hv_show_report_banner()
    assert w.msgs[-1].startswith("HARVEST：validate_failed(既有的)；grid_wind_estimate(")
    # 清掉網格 ⇒ 重估與它的 loud note 一起拿掉，別人的 note 留著
    w._on_wind_grid_clear()
    assert w._wind_state()["estimate"] is None and "網格風重估" not in w.status and "網格範圍：" not in w.status
    assert w._hv_last_report["loud_notes"] == ["validate_failed(既有的)"]


def test_report_of_a_planning_in_flight_is_not_touched(mini, workers):
    w = _win(mini)
    w.in_flight = True
    w._hv_last_report = {}                                         # 新的一趟剛開始（_start_hv_planning 清空）
    _fetch_grid(w, workers, 12.0, 90.0)
    assert w._hv_last_report == {}, "那份報告屬於還在跑的新規劃"
    assert w.msgs[-1].startswith("⚠ 網格風重估："), "畫面上那份（舊）計畫的問題照樣跳狀態列"


def test_estimate_failure_is_loud_not_a_silent_zero(mini, workers, monkeypatch):
    def boom(*a, **k):
        raise RuntimeError("boom")
    monkeypatch.setattr(gwe, "estimate_plan", boom)
    w = _win(mini)
    _fetch_grid(w, workers, 5.0, 270.0)
    assert "⚠ 網格風重估失敗（RuntimeError: boom）：這份計畫的航時與能量只有規劃器（規劃風）的估計" in w.status
    assert w.msgs[-1] == "⚠ 網格風重估：重估失敗（RuntimeError: boom）"
    assert w._hv_last_report["loud_notes"] == ["grid_wind_estimate(重估失敗（RuntimeError: boom）)"]
    assert mw.grid_wind_estimate_digest(w) == "⚠ 網格風重估失敗（見風場狀態）"


# ═══════════════════════════════════════════════════════════════════════
#  4. 過時：estimate_matches／換計畫／換網格
# ═══════════════════════════════════════════════════════════════════════
def test_same_plan_and_grid_is_reused_and_a_changed_plan_is_recomputed(mini, workers, monkeypatch):
    calls = []
    real = gwe.estimate_plan
    monkeypatch.setattr(gwe, "estimate_plan", lambda *a, **k: calls.append(1) or real(*a, **k))
    w = _win(mini)
    _fetch_grid(w, workers, 12.0, 90.0)
    est = w._wind_state()["estimate"]
    n_msgs = len(w.msgs)
    assert mw.refresh_grid_wind_estimate(w, "規劃完成") is est and len(calls) == 1, "同一份計畫、同一份網格 ⇒ 沿用"
    assert len(w.msgs) == n_msgs, "沿用不重跳狀態列"
    assert w._hv_last_report["loud_notes"] == [f"grid_wind_estimate({p})" for p in est.problems], "沿用時 loud note 照舊"
    # 新規劃（另一份計畫物件；落地時刻不同）
    a1 = mini.mp.plans["A1"]
    plan2 = dataclasses.replace(mini.mp, plans={**mini.mp.plans, "A1": dataclasses.replace(a1, t_land_s=a1.t_land_s + 7.0)})
    hv2 = _hv(mini, plan2)
    w._hv_coordinator, w._dccpp_result = hv2, hv2.last_result
    assert mw.grid_wind_estimate_status_text(w).startswith("⚠ 網格風重估已作廢"), "沒重算之前：不留舊計畫的數字"
    est2 = mw.refresh_grid_wind_estimate(w, "規劃完成")
    assert est2 is not est and len(calls) == 2 and gwe.estimate_matches(est2, plan2)
    assert est2.uavs["A1"].t_land_plan_s == pytest.approx(a1.t_land_s + 7.0)
    # 另一份計畫物件、指紋剛好一樣（航點數與落地時刻都同）⇒ 也是新計畫，要重算
    plan3 = copy.deepcopy(plan2)
    assert gwe.plan_fingerprint(plan3) == gwe.plan_fingerprint(plan2)
    hv3 = _hv(mini, plan3)
    w._hv_coordinator, w._dccpp_result = hv3, hv3.last_result
    est3 = mw.refresh_grid_wind_estimate(w, "規劃完成")
    assert est3 is not est2 and len(calls) == 3


def test_plan_changed_in_place_is_caught_by_the_fingerprint(mini, workers):
    plan = copy.deepcopy(mini.mp)
    w = _win(mini)
    w._hv_coordinator = _hv(mini, plan)
    w._dccpp_result = w._hv_coordinator.last_result
    _fetch_grid(w, workers, 6.0, 250.0)
    assert mw.grid_wind_estimate_status_text(w).startswith("網格風重估（計畫不變")
    plan.plans["B1"].t_land_s += 3.0                               # 同一個物件就地被改（重規劃寫回）
    assert mw.grid_wind_estimate_status_text(w) == \
        "⚠ 網格風重估已作廢：畫面上的計畫或網格風換過了（規劃完成、重抓或清除氣象網格時會重算）"
    w._refresh_wind_status()
    assert "已作廢" in w.status and "總任務時間" not in w.status
    assert mw.grid_wind_estimate_digest(w) == ""
    mw.refresh_grid_wind_estimate(w, "重算")
    assert gwe.estimate_matches(w._wind_state()["estimate"], plan) and "總任務時間" in w.status


def test_staleness_check_that_breaks_is_loud_and_does_not_raise(mini, workers, monkeypatch):
    """狀態列每次刷新（含即時風的 slot）都會判斷過時：判斷本身壞掉 ⇒ 寫明，不丟例外（slot 丟例外會 qFatal）、不當成還有效。"""
    w = _win(mini)
    _fetch_grid(w, workers, 6.0, 250.0)

    def broken(*_a, **_k):
        raise KeyError("fp")
    monkeypatch.setattr(gwe, "estimate_matches", broken)
    assert mw.grid_wind_estimate_status_text(w) == \
        "⚠ 網格風重估：無法判斷是否還對應目前的計畫（KeyError: 'fp'），請重抓氣象網格"
    w._refresh_wind_status()
    assert "無法判斷" in w.status and "總任務時間" not in w.status
    assert mw.grid_wind_estimate_digest(w) == "⚠ 網格風重估讀不到（KeyError）"


def test_dccpp_plan_or_cleared_paths_drop_the_estimate(mini, workers):
    w = _win(mini)
    _fetch_grid(w, workers, 12.0, 90.0)
    assert "網格風重估" in w.status and w._hv_last_report["loud_notes"]
    w._hv_coordinator, w._dccpp_result = None, {"assembled_paths": {}}   # 改用 DCCPP 規劃
    assert mw.refresh_grid_wind_estimate(w, "規劃完成") is None
    assert w._wind_state()["estimate"] is None and "網格風重估" not in w.status
    assert w._hv_last_report["loud_notes"] == [], "舊計畫的 loud note 不留給新計畫"
    # 清除路徑（on_clear_paths 本體）
    w2 = _win(mini)
    _fetch_grid(w2, workers, 12.0, 90.0)
    w2.on_clear_paths()
    assert w2._dccpp_result is None and w2._wind_state()["estimate"] is None
    assert "網格風重估" not in w2.status and w2.waypoints == []


def test_nothing_happens_without_a_grid(mini):
    w = _win(mini)
    assert mw.refresh_grid_wind_estimate(w, "規劃完成") is None
    assert w.vtol_recon_panel.status == [] and w.msgs == [] and "_hv_last_report" not in w.__dict__
    assert mw.grid_wind_estimate_digest(w) == ""


# ═══════════════════════════════════════════════════════════════════════
#  5. 規劃完成這條路：橫幅之前算好、摘要帶一句
# ═══════════════════════════════════════════════════════════════════════
def test_harvest_finished_estimates_before_the_banner_and_the_recon_summary(mini, workers):
    w = _win(mini, plan=False)
    _fetch_grid(w, workers, 12.0, 90.0)                             # 網格先到、計畫後到
    assert w._wind_state()["estimate"] is None
    hv = _hv(mini)
    order = []

    def display(result, **_kw):                                    # _display_dccpp_result 的替身：最後設 _dccpp_result
        order.append("display")
        w._dccpp_result = result

    w._display_dccpp_result = display
    w._hv_worker = types.SimpleNamespace(coordinator=types.SimpleNamespace(hv_coordinator=hv))
    w._hv_last_report = {"loud_notes": [], "uncovered": [], "status": "GO"}
    w._hv_on_done = lambda ok: (order.append("done"), w._on_vtol_recon_planned({"num_drones": 2}, ok))
    w._on_hv_finished(hv.last_result)
    assert order == ["display", "done"]
    est = w._wind_state()["estimate"]
    assert est is not None and est.problems and gwe.estimate_matches(est, mini.mp)
    assert w.msgs[-1].startswith("HARVEST：grid_wind_estimate("), "橫幅（規劃完成後的狀態列）帶著重估的問題"
    summary = w.vtol_recon_panel.plan_results[-1][1]
    d = est.makespan_grid_s - est.makespan_plan_s
    unc = sum(e.uncovered_m for e in est.uavs.values())
    assert unc > 1000.0, "網格是沒有計畫時抓的（只有作業區）⇒ 起降轉移不在網格內"
    assert summary.endswith(f" ｜ 氣象網格重估 makespan {d:+.1f} s（{100 * d / est.makespan_plan_s:+.2f}%），"
                            f"⚠ {len(est.problems)} 項問題（見風場狀態），{unc / 1000:.1f} km 不在網格內")
    assert "規劃風 5 m/s 來自 270°" in summary


def test_sync_planning_path_and_vehicle_change_call_the_refresh():
    """同步規劃路徑與換載具在 _on_dccpp_coverage_requested／參數處理裡（太大、這裡驅動不起來）⇒ 鎖原始碼的那一行與順序：
    顯示完（_dccpp_result 已換）才重估、on_done（偵察頁摘要）之前；換載具清掉 _dccpp_result 之後。"""
    src = inspect.getsource(MainWindow._on_dccpp_coverage_requested)
    i_disp = src.index("self._display_dccpp_result(dccpp_result, **display_ctx)")
    i_ref = src.index("refresh_grid_wind_estimate(self, '規劃完成')")
    i_done = src.index("on_done(True)", i_disp)
    assert i_disp < i_ref < i_done
    full = inspect.getsource(MainWindow)
    i_veh = full.index("hdmap_preview_reset(self, f\"載具改為 {params['vehicle_type']}\")")
    assert full.index("refresh_grid_wind_estimate(self, '換載具')", i_veh) - i_veh < 200


def test_recon_summary_has_no_digest_without_a_grid(mini):
    w = _win(mini)
    w._on_vtol_recon_planned({"num_drones": 2}, True)
    summary = w.vtol_recon_panel.plan_results[-1][1]
    assert "重估" not in summary and summary.endswith("規劃風 5 m/s 來自 270°")
