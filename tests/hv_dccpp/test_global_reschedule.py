"""vendored 修改 #3：全隊起飛時刻全域重排（`hv_dccpp.deconflict._global_reschedule`）。

⚠⚠⚠ 這個檔的每一條都是**值層級**鎖：鎖「跑出來的 makespan／掃描完成時刻是多少」，
不鎖「那一行程式碼長什麼樣」。源碼層的鎖對「改上游／改下游／繞過」全部免疫，
補鎖是軍備競賽（見專案筆記 value-locks-beat-source-locks）。

⚠⚠ 每一條「開著要多好」的斷言都配一條**前置斷言**「關掉時要多差」。少了它，
這些鎖在任何實作下都會綠 —— 包括在這個功能根本沒接上的實作下。

## 突變實驗的結果（2026-09-08，`scratchpad/reschedule_mutations.py`，**4/6 咬到**）

| 突變臂 | 結果 |
|---|---|
| 不檢查 4D 分離，只看走廊 | ✅ 3 條變紅 |
| 不檢查走廊時刻表，只看 4D | ✅ 3 條變紅 |
| 忽略 `global_reschedule` 開關 | ✅ 5 條變紅 |
| 剪枝界改回用 `incumbent`（單位錯置） | ✅ 1 條變紅 |
| 拿掉套用前的完整驗證 | ❌ 全綠 |
| 拿掉「嚴格改善」的比較 | ❌ 全綠 |

⚠⚠⚠ 後兩條**不是**測試不夠緊，是那兩個突變**在數學上不改變輸出**（已實測：
兩者都逐位給出同樣的 1450.8 / 1216.4）。原因寫在 `deconflict._global_reschedule`
的行內註解裡。**不要**為了讓它們變紅而寫更緊的鎖 —— 那會鎖住與被測機制無關的量。
也**不要**因為咬不到就宣稱那兩行被測試涵蓋。
"""

from __future__ import annotations

import logging

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

from hv_dccpp.deconflict import scan_done_s                     # noqa: E402
from hv_dccpp.mission import PlannerConfig                      # noqa: E402

_SCENARIO = "gui_legal_120_split"

# 上游（關掉全域重排）在這個情境的行為。⚠ 這四個起飛時刻就是已飛的 HV4SITL9。
_OFF_MAKESPAN = 1799.8
_OFF_SCAN = 1594.6
_OFF_TAKEOFF = {"A1": 289.1, "B2": 623.9, "B3": 0.0, "B4": 1265.6}


def _plan(**cfg_over):
    """跑生產 GUI 路徑，回傳 (makespan, 掃描完成, {機: 起飛時刻}, 未覆蓋數)。"""
    scenarios = pytest.importorskip("scripts.hv_sitl_scenarios")
    runner = pytest.importorskip("scripts.hv_sitl_run")
    sc = scenarios.SCENARIOS[_SCENARIO]

    orig = PlannerConfig.__init__

    def patched(self, *a, **kw):
        kw.update(cfg_over)
        orig(self, *a, **kw)

    logging.disable(logging.WARNING)
    PlannerConfig.__init__ = patched                            # type: ignore[method-assign]
    try:
        hv, _report = runner._plan_gui(sc)
    finally:
        PlannerConfig.__init__ = orig                           # type: ignore[method-assign]
        logging.disable(logging.NOTSET)

    plan = hv.last_plan
    return (max(float(p.t_land_s) for p in plan.plans.values()),
            max(scan_done_s(p) for p in plan.plans.values()),
            {u: round(float(p.t_takeoff_s), 1) for u, p in plan.plans.items()},
            len(plan.uncovered or []))


@pytest.fixture(scope="module")
def off():
    return _plan(global_reschedule=False)


@pytest.fixture(scope="module")
def on():
    return _plan(global_reschedule=True, reschedule_objective="landing")


# ── 前置斷言：關掉時**確實**是差的，而且逐位元等於上游 ────────────────────

def test_disabled_reproduces_upstream_bit_for_bit(off):
    """⚠ 刪除實驗：開關關掉 ⇒ 必須完全退回上游行為（含四個起飛時刻）。

    這條同時是所有其他條的前置斷言 —— 沒有它，「開著比較好」可能只是因為
    這個情境本來就好，與功能有沒有生效無關。
    """
    ms, scan, tks, unc = off
    assert ms == pytest.approx(_OFF_MAKESPAN, abs=0.5), ms
    assert scan == pytest.approx(_OFF_SCAN, abs=0.5), scan
    assert unc == 0
    for u, t in _OFF_TAKEOFF.items():
        assert tks[u] == pytest.approx(t, abs=0.5), (u, tks)


def test_disabled_makespan_is_actually_bad(off):
    """前置斷言（獨立寫一條，因為採用門檻掛在它上面）：關掉時 makespan > 1700。"""
    assert off[0] > 1700.0, off[0]


# ── 值層級鎖：開著要好多少 ──────────────────────────────────────────────

def test_reschedule_cuts_makespan(on):
    ms, _scan, _tks, unc = on
    assert ms <= 1500.0, ms
    assert unc == 0


def test_reschedule_cuts_scan_completion(on):
    """使用者要的是「最快掃完」，所以掃描完成時刻要單獨鎖。"""
    assert on[1] <= 1300.0, on[1]


def test_improvement_is_large_not_marginal(on, off):
    """相對改善要 ≥15%，否則就不是這個機制在起作用（貪婪落點的雜訊可達數十秒）。"""
    assert (off[0] - on[0]) / off[0] > 0.15, (off[0], on[0])
    assert (off[1] - on[1]) / off[1] > 0.15, (off[1], on[1])


def test_no_conflicts_remain(on):
    """⭐ 最重要：省下來的時間不可以是拿安全間距換的。"""
    scenarios = pytest.importorskip("scripts.hv_sitl_scenarios")
    runner = pytest.importorskip("scripts.hv_sitl_run")
    import hv_dccpp as hvlib

    orig = PlannerConfig.__init__

    def patched(self, *a, **kw):
        kw.update(global_reschedule=True)
        orig(self, *a, **kw)

    logging.disable(logging.WARNING)
    PlannerConfig.__init__ = patched                            # type: ignore[method-assign]
    try:
        hv, _r = runner._plan_gui(scenarios.SCENARIOS[_SCENARIO])
    finally:
        PlannerConfig.__init__ = orig                           # type: ignore[method-assign]
        logging.disable(logging.NOTSET)

    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    v = hvlib.validate(plan, ctx, hvlib.build_validation_cache(plan, ctx, budget_only=False), cfg)
    assert v.passed, v.conflicts
    assert list(v.conflicts) == [], v.conflicts


def test_geometry_is_untouched(on, off):
    """只動時間戳 ⇒ 航程一公尺都不能變（重排不是拿覆蓋換速度）。"""
    scenarios = pytest.importorskip("scripts.hv_sitl_scenarios")
    runner = pytest.importorskip("scripts.hv_sitl_run")
    sc = scenarios.SCENARIOS[_SCENARIO]
    orig = PlannerConfig.__init__

    def mk(flag):
        def patched(self, *a, **kw):
            kw.update(global_reschedule=flag)
            orig(self, *a, **kw)
        logging.disable(logging.WARNING)
        PlannerConfig.__init__ = patched                        # type: ignore[method-assign]
        try:
            hv, _r = runner._plan_gui(sc)
        finally:
            PlannerConfig.__init__ = orig                       # type: ignore[method-assign]
            logging.disable(logging.NOTSET)
        return {u: (round(p.distance_m, 3), len(p.strips), len(p.waypoints))
                for u, p in hv.last_plan.plans.items()}

    assert mk(True) == mk(False)


# ── 反不可能觸發：把旋鈕轉到無效值必須**退化**，證明它真的被讀 ────────────

def test_absurd_step_degenerates_to_upstream(off):
    """步長 1000 s ⇒ 搜不到任何比現況好的候選 ⇒ 必須原樣退回，不可變差。"""
    ms, scan, _tks, unc = _plan(global_reschedule=True, reschedule_step_s=1000.0)
    assert ms == pytest.approx(off[0], abs=0.5), (ms, off[0])
    assert scan == pytest.approx(off[1], abs=0.5)
    assert unc == 0


def test_zero_eval_budget_degenerates_to_upstream(off):
    """評估次數上限設 0 ⇒ 一個候選都驗不了 ⇒ 原樣退回。"""
    ms, _scan, _tks, unc = _plan(global_reschedule=True, reschedule_max_evals=0)
    assert ms == pytest.approx(off[0], abs=0.5), (ms, off[0])
    assert unc == 0


def test_both_objectives_reach_the_same_optimum(on):
    """⚠ 迴歸鎖：剪枝的界曾經用錯單位（拿掃描時刻去剪落地時刻），
    導致 objective='scan' 的結果反而比 'landing' 差（1251.7 vs 1216.4）。"""
    ms_s, scan_s, _t, unc = _plan(global_reschedule=True, reschedule_objective="scan")
    assert unc == 0
    assert scan_s <= on[1] + 0.5, (scan_s, on[1])
    assert ms_s <= on[0] + 0.5, (ms_s, on[0])
