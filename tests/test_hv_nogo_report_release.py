"""HARVEST 規劃 NO_GO 時，轉接層那份報告也要送到 `hv_on_report`。

SwarmCoordinator 的 HV 分支用 `_plan_once` 把轉接層的 on_report 換成 `_rep_buf.append`
（自動指派會跑兩次規劃，只能放行勝出那一份）。轉接層在 preflight NO_GO 時是
**先送 report 再丟 AeroPlanAdapterError**（`raise_on_no_go=True`），而緩衝原本只在規劃成功後放行
⇒ NO_GO 時那份（status=NO_GO、coverage_altitude_clamped、alternate_auto_placed…）從不送出，
`MainWindow._on_hv_failed` 的失敗對話框只帶得到規劃前送的 _prenotes。
2026-10-16 實測：NO_GO 情境 `hv_on_report=list.append` 只收到 1 則（前置），轉接層那份卡在緩衝裡。

約束：放行的只能是**這一次規劃**自己的那份；自動指派試跑落選那份不能放。

不建 MainWindow、不起 SITL；每條都跑真的規劃（各約 1–2 s）。
"""
from __future__ import annotations

import math
import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")

import mission.swarm_coordinator as SC                          # noqa: E402

hv = pytest.importorskip("hv_dccpp.integration.aeroplan_adapter")

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))


def _ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def _square(half: float) -> list[tuple[float, float]]:
    return [_ll(-half, -half), _ll(half, -half), _ll(half, half), _ll(-half, half)]


_DRONES = [{"drone_id": 1, "name": "A1", "position": HOME, "speed": 21.5, "heading": 0.0,
            "turn_radius": 200.0, "vehicle_type": "vtol", "fov_width": 150.0},
           {"drone_id": 2, "name": "B1", "position": HOME, "speed": 19.0, "heading": 0.0,
            "turn_radius": 120.0, "vehicle_type": "vtol", "fov_width": 150.0}]


def _coordinator(params: dict, reports: list) -> SC.SwarmCoordinator:
    c = SC.SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: {1: "A", 2: "B"}[int(d["drone_id"])]
    c.hv_home = HOME
    c.hv_wind = (0.0, 0.0)
    c.hv_fence_zones = []
    c.hv_on_report = reports.append
    c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                        "hv_dem_margin_m": 3000.0, "target_gsd": 0.05, **params}
    return c


def _plan_no_go():
    """沒有豁免（AGL 上限 120）＋作業高度 100 ⇒ 轉接層夾到 120（coverage_altitude_clamped）、
    兩構型高度帶都不夠 ⇒ preflight NO_GO。沒給備降點 ⇒ 轉接層自己放一個（alternate_auto_placed）。"""
    reports: list = []
    c = _coordinator({}, reports)
    with pytest.raises(hv.AeroPlanAdapterError) as e:
        c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0, "polygon": _square(782.6)}],
                              drones=[dict(d) for d in _DRONES], coverage_width_m=150.0,
                              overlap_rate=0.30, coverage_altitude=100.0, mounting_angle_deg=0.0)
    return str(e.value), reports


def _adapter_reports(reports):
    """轉接層的 payload 有 `status`；SwarmCoordinator 自己送的前置／尾端只有 validation_note(s)。"""
    return [r for r in reports if "status" in r]


@pytest.fixture(scope="module")
def no_go():
    return _plan_no_go()


# ── 1. NO_GO 時轉接層那份要送出來 ─────────────────────────────────────
def test_no_go_releases_the_adapters_report_exactly_once(no_go):
    msg, reports = no_go
    assert "NO_GO" in msg, msg
    ad = _adapter_reports(reports)
    assert len(ad) == 1, f"NO_GO 時轉接層那份沒送出（或送了不只一次）：收到 {len(reports)} 則 {reports}"
    assert ad[0]["status"] == "NO_GO"
    assert any("no_coverable_area" in e for e in ad[0]["errors"]), ad[0]["errors"]
    notes = ad[0]["notes"]
    assert any(n.startswith("coverage_altitude_clamped(100.0->120.0") for n in notes), notes
    assert any(n.startswith("alternate_auto_placed(") for n in notes), notes


def test_prenotes_still_come_first(no_go):
    """前置 note（規劃前送）仍在、而且在轉接層那份之前（GUI 端依到達順序累加 loud note）。"""
    _msg, reports = no_go
    assert len(reports) == 2, reports
    assert "status" not in reports[0]
    assert any(n.startswith("altitude_band_too_narrow(") for n in reports[0]["validation_notes"]), reports[0]
    assert reports[1]["status"] == "NO_GO"


# ── 2. 端到端：同一批報告餵 MainWindow 的未綁定方法，失敗對話框要帶上轉接層的 loud note ──
def test_failure_dialog_carries_the_adapters_loud_notes(no_go, monkeypatch):
    try:
        from ui.main_window import MainWindow
        from PyQt6 import QtWidgets as _QtW
    except Exception as e:                                      # pragma: no cover
        pytest.skip(f"Qt 匯入失敗：{e}")
    msg, reports = no_go
    crit = []
    monkeypatch.setattr(_QtW.QMessageBox, "critical", staticmethod(lambda *a, **k: crit.append(a)))
    w = types.SimpleNamespace(_HV_LOUD_NOTES=MainWindow._HV_LOUD_NOTES, _hv_last_report={})
    w.statusBar = lambda: types.SimpleNamespace(clearMessage=lambda: None, showMessage=lambda *a, **k: None)
    w._hv_notify_done = lambda ok: None
    for p in reports:                                           # 工作執行緒：report 訊號先於 failed
        MainWindow._on_hv_report(w, p)
    MainWindow._on_hv_failed(w, msg)
    assert len(crit) == 1
    _parent, title, text = crit[0][:3]
    assert title == "HARVEST 失敗"
    for prefix in ("altitude_band_too_narrow(", "coverage_altitude_clamped(", "alternate_auto_placed("):
        assert f"• {prefix}" in text, (prefix, text)
    assert w._hv_last_report["status"] == "NO_GO"


# ── 3. 地形起伏提示那條 re-raise 路徑也要放行 ─────────────────────────
class _FakeWT:
    """假的 World Terrain（形狀同 tests/test_cesium_world_terrain.py::FakeWT）：起伏 77 m。"""
    vertical_datum = "ellipsoid"

    def __init__(self, token, **kw):
        self.failures, self.stats = [], {}
        self.attribution_text = "Fake data"

    def prefetch(self, bounds, **kw):
        return {"level": 13, "tiles": 4, "fetched": 4, "bytes": 4000}

    def relief_m(self, bounds, n=40):
        return (4.0, 81.0)

    def get_elevation(self, lat, lon):
        return 30.0 + (lon - HOME[1]) * _M_LON * 0.01


def test_no_go_with_terrain_hint_also_releases(monkeypatch):
    monkeypatch.setattr("core.terrain.CesiumWorldTerrain", _FakeWT)
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("fake.tok.en", "測試"))
    reports: list = []
    c = _coordinator({"hv_world_terrain": True, "hv_waiver_max_agl_m": None,
                      "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0}, "hv_dh_band_min_m": 15.0}, reports)
    area = [_ll(3000, -450), _ll(4600, -450), _ll(4600, 450), _ll(3000, 450)]
    with pytest.raises(hv.AeroPlanAdapterError) as e:
        c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0, "polygon": area}],
                              drones=[dict(d) for d in _DRONES], coverage_width_m=150.0,
                              overlap_rate=0.30, coverage_altitude=120.0, mounting_angle_deg=0.0)
    assert "提示" in str(e.value), "前提：要走到接上地形提示再 raise 的那條路徑"
    ad = _adapter_reports(reports)
    assert len(ad) == 1 and ad[0]["status"] == "NO_GO", reports


# ── 4. 自動指派試跑那份不能放行 ───────────────────────────────────────
def test_auto_assign_trial_report_is_never_released(monkeypatch):
    """基準規劃成功、自動指派的試跑在轉接層 NO_GO（先送 report 再 raise）⇒ 試跑那份不可以送出去。

    試跑本身用替身（第 2 次呼叫轉接層時送一則帶 TRIAL_LOSER 的報告再丟例外）；
    候選指派用替身強迫它跑試跑。整趟最後成功或失敗都不在這條的範圍。
    """
    calls = []
    orig = hv.HvDccppCoordinator.plan_coverage_dccpp

    def wrapped(self, **kw):
        calls.append(dict(self.recovery_sites or {}))
        if len(calls) == 2:
            if self.on_report is not None:
                self.on_report({"status": "NO_GO", "errors": ["trial"], "warnings": [],
                                "notes": ["coverage_altitude_clamped(TRIAL_LOSER)"], "uncovered": []})
            raise hv.AeroPlanAdapterError("HV-DCCPP preflight NO_GO：trial", ["trial"])
        return orig(self, **kw)

    monkeypatch.setattr(hv.HvDccppCoordinator, "plan_coverage_dccpp", wrapped)
    monkeypatch.setattr(SC, "propose_recovery_assignment", lambda plan, cfg, sites: {"X": sites[-1]})
    reports: list = []
    c = _coordinator({"hv_waiver_max_agl_m": 200.0}, reports)
    c.hv_alternates = [_ll(0.0, 600.0)]
    c.hv_recovery_auto = True
    try:
        c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0, "polygon": _square(600.0)}],
                              drones=[dict(d) for d in _DRONES], coverage_width_m=150.0,
                              overlap_rate=0.30, coverage_altitude=140.0, mounting_angle_deg=0.0)
    except hv.AeroPlanAdapterError:
        pass
    assert len(calls) == 2 and calls[1] == {"X": "ALT1"}, f"前提：試跑要真的跑到（{calls}）"
    leaked = [r for r in reports if "TRIAL_LOSER" in str(r.get("notes"))]
    assert not leaked, f"自動指派試跑落選那份被放行了：{leaked}"


# ── 5. 消費端壞掉不能把 NO_GO 換成別的例外 ─────────────────────────────
def test_a_broken_consumer_does_not_mask_the_no_go():
    """轉接層自己送 report 時會吞掉消費端的例外；改由這裡放行之後也要一樣，否則失敗對話框顯示的
    是消費端的 TypeError 而不是 NO_GO 的原因。"""
    seen = []

    def consumer(payload):
        seen.append(payload)
        if "status" in payload:
            raise RuntimeError("consumer exploded")

    c = _coordinator({}, [])
    c.hv_on_report = consumer
    with pytest.raises(hv.AeroPlanAdapterError) as e:
        c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0, "polygon": _square(782.6)}],
                              drones=[dict(d) for d in _DRONES], coverage_width_m=150.0,
                              overlap_rate=0.30, coverage_altitude=100.0, mounting_angle_deg=0.0)
    assert "NO_GO" in str(e.value)
    assert _adapter_reports(seen), "前提：轉接層那份要真的送到消費端"
