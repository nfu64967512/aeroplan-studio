"""HARVEST 多場坪自動指派（`hv_recovery_auto`）：試跑失敗＝候選落選，不是整趟失敗。

SwarmCoordinator 的 HV 分支在基準規劃 `_plan_once(_rec0)` 成功後，用 `_res2 = _plan_once(_cand)` 試跑候選指派。
那次呼叫外面原本只有 try/finally（還原 progress）⇒ 試跑在轉接層 preflight NO_GO（AeroPlanAdapterError）
或任何其他例外時，例外直接往外丟：
  - 已經成功的基準規劃被丟掉、整趟規劃失敗；
  - `hv_coordinator.recovery_sites` 停在候選（`_hv.recovery_sites = _rec0` 那行跑不到）；
  - 轉接層的 NO_GO 是 plan_mission 跑完、`last_plan = plan` **之後**才 raise ⇒ `last_plan` 也停在候選那份。
2026-10-21 在未修的程式上實測（替身先真的跑完候選的規劃再送報告＋raise）：三件全中，
last_plan 落點 {'A1': 'ALT1', 'B2': 'ALT1'}、基準是 {'A1': 'H', 'B2': 'H'}。

期望：照「落選」那條路跑回基準、發 `recovery_auto_rejected(…試跑失敗：…)`、只放行基準那份報告。

不建 MainWindow、不起 SITL；每條跑 2–3 次真的規劃（約 2–4 s）。
"""
from __future__ import annotations

import math
import os

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


def _progress(*_a, **_k):
    return None


def _run(monkeypatch, *, trial_exc=None, candidate=None):
    """跑一趟自動指派開著的規劃。

    ``trial_exc``：給了就在**第 2 次**呼叫轉接層（＝試跑）時，先真的跑完候選的規劃（last_plan 換成候選，
    同轉接層 NO_GO 的順序）、送一則帶 TRIAL_LOSER 的報告，再丟這個例外。
    ``candidate``：候選指派；預設全員改降最後一個站點（ALT1）。
    回傳 (結果或例外, 協調器, 各次呼叫的 recovery_sites, 各次規劃後的 (plan 物件, 落點), 收到的報告)。
    """
    calls, plans = [], []
    orig = hv.HvDccppCoordinator.plan_coverage_dccpp

    def wrapped(self, **kw):
        calls.append(dict(self.recovery_sites or {}))
        try:
            return orig(self, **kw)
        finally:
            lp = self.last_plan
            plans.append((lp, {u: p.landing_site for u, p in sorted(lp.plans.items())} if lp else None))
            if trial_exc is not None and len(calls) == 2:
                if self.on_report is not None:
                    self.on_report({"status": "NO_GO", "errors": ["trial"], "warnings": [],
                                    "notes": ["coverage_altitude_clamped(TRIAL_LOSER)"], "uncovered": []})
                raise trial_exc

    monkeypatch.setattr(hv.HvDccppCoordinator, "plan_coverage_dccpp", wrapped)
    monkeypatch.setattr(SC, "propose_recovery_assignment",
                        candidate or (lambda plan, cfg, sites: {u: sites[-1] for u in plan.plans}))
    reports: list = []
    c = SC.SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: {1: "A", 2: "B"}[int(d["drone_id"])]
    c.hv_home = HOME
    c.hv_wind = (0.0, 0.0)
    c.hv_fence_zones = []
    c.hv_on_report = reports.append
    c.hv_progress = _progress
    c.hv_alternates = [_ll(0.0, 600.0)]
    c.hv_recovery_auto = True
    c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                        "hv_dem_margin_m": 3000.0, "target_gsd": 0.05, "hv_waiver_max_agl_m": 200.0}
    try:
        out = c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0, "polygon": _square(600.0)}],
                                    drones=[dict(d) for d in _DRONES], coverage_width_m=150.0,
                                    overlap_rate=0.30, coverage_altitude=140.0, mounting_angle_deg=0.0)
    except Exception as e:                                      # noqa: BLE001
        out = e
    return out, c, calls, plans, reports


def _adapter_reports(reports):
    """轉接層的 payload 有 `status`；SwarmCoordinator 自己送的前置／尾端只有 validation_note(s)。"""
    return [r for r in reports if "status" in r]


def _auto_notes(reports):
    return [n for r in reports for n in (r.get("validation_notes") or []) if "recovery_auto" in n]


def _assert_rolled_back(out, c, calls, plans, reports, why_fragment):
    assert not isinstance(out, BaseException), f"試跑失敗讓整趟規劃失敗（基準明明成功了）：{out!r}"
    assert isinstance(out, dict) and out.get("total_makespan", 0) > 0, out
    # 前提：試跑真的跑到、而且候選的落點和基準不同（否則 last_plan 那條驗不出東西）
    assert len(calls) >= 2 and calls[0] == {} and calls[1], f"前提：試跑要真的跑到（{calls}）"
    hvc = c.hv_coordinator
    # 協調器狀態要回到基準：匯出／上傳都讀它
    assert hvc.recovery_sites is None, f"recovery_sites 停在候選：{hvc.recovery_sites}"
    assert hvc.progress is _progress, "試跑後沒把進度回呼換回來"
    base_land = plans[0][1]
    final_land = {u: p.landing_site for u, p in sorted(hvc.last_plan.plans.items())}
    assert final_land == base_land, f"last_plan 停在落選的候選：{final_land}（基準 {base_land}）"
    # 只放行基準那一份轉接層報告，試跑那份不能出去
    ad = _adapter_reports(reports)
    assert not [r for r in reports if "TRIAL_LOSER" in str(r.get("notes"))], "試跑落選那份被放行了"
    assert len(ad) == 1 and ad[0]["status"] != "NO_GO", ad
    assert ad[0]["makespan_s"] == pytest.approx(float(hvc.last_plan.makespan_s)), \
        "放行的報告不是協調器最後那份計畫的"
    # 要留下可讀回的 note（前綴已在 MainWindow._HV_LOUD_NOTES）
    notes = _auto_notes(reports)
    assert len(notes) == 1 and notes[0].startswith("recovery_auto_rejected("), notes
    assert "試跑失敗" in notes[0] and why_fragment in notes[0], notes


# ── 1. 試跑在轉接層 preflight NO_GO ──────────────────────────────────
def test_trial_no_go_falls_back_to_the_baseline(monkeypatch):
    out, c, calls, plans, reports = _run(
        monkeypatch, trial_exc=hv.AeroPlanAdapterError("HV-DCCPP preflight NO_GO：trial", ["trial"]))
    assert plans[1][1] != plans[0][1], f"前提：候選的落點要和基準不同（{plans[1][1]} vs {plans[0][1]}）"
    _assert_rolled_back(out, c, calls, plans, reports, "preflight NO_GO：trial")
    assert c.hv_coordinator.last_plan is not plans[1][0], "last_plan 還是試跑那個物件"


# ── 2. 試跑丟的不是轉接層例外（例如 from_mission_plan／量測崩潰）也一樣是落選 ──────
def test_trial_unexpected_exception_is_also_a_rejection(monkeypatch):
    out, c, calls, plans, reports = _run(monkeypatch, trial_exc=RuntimeError("boom-in-trial"))
    assert plans[1][1] != plans[0][1], "前提：候選的落點要和基準不同"
    _assert_rolled_back(out, c, calls, plans, reports, "RuntimeError: boom-in-trial")


# ── 3. 不用替身：真的轉接層拒絕候選（鍵不在機隊裡 ⇒ AeroPlanAdapterError，在 plan_mission 之前）──
def test_real_adapter_rejecting_the_candidate_is_a_rejection(monkeypatch):
    out, c, calls, plans, reports = _run(monkeypatch, candidate=lambda plan, cfg, sites: {"X": sites[-1]})
    assert calls[1] == {"X": "ALT1"}, calls
    _assert_rolled_back(out, c, calls, plans, reports, "不是機隊裡的載具")


# ── 4. 對照：試跑成功時照舊（守衛擋下也是跑回基準，note 不帶「試跑失敗」）──────────
def test_successful_trial_is_unchanged(monkeypatch):
    out, c, calls, _plans, reports = _run(monkeypatch)
    assert isinstance(out, dict), out
    assert len(calls) == 3 and calls[2] == {}, calls            # 這組情境守衛會擋下（間距變差）⇒ 跑回基準
    notes = _auto_notes(reports)
    assert len(notes) == 1 and notes[0].startswith("recovery_auto_rejected(") and "試跑失敗" not in notes[0], notes
    assert c.hv_coordinator.recovery_sites is None
