"""多場坪自動指派**接進 HARVEST 分支**之後的行為（`hv_recovery_auto`）。

⚠ 這個檔每一條都跑真的規劃（每次 ~20–50 s），所以刻意只留最會出錯的四件事：

1. 開關關掉時逐位元等於接線前。
2. 只有一個站點時完全不動（不多跑規劃、不發 note）。
3. 有兩個以上站點時**產物**真的變了，而且**留下可讀回的 note**。
4. ⭐⭐⭐ 守衛擋下候選時，`hv_coordinator.last_plan` 必須**跑回基準** ——
   匯出與上傳都讀它，留著落選的計畫等於飛出去的是沒被採用的那一份。
"""

from __future__ import annotations

import logging

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

import mission.swarm_coordinator as SC                          # noqa: E402

_SCENARIO = "gui_legal_120_split_resched"
_PAD_N600 = 600.0 / 111132.0


def _plan(*, auto: bool, n_sites: int, guard=None):
    """跑一次生產路徑；回傳 (落點, makespan, notes)。

    ``n_sites``：1 = 只有 home（其實是 home + 0 個備降點）；2 = +ALT1；3 = +ALT1 +ALT2。
    ``guard``：給了就用它取代 `recovery_regression_guard`（用來強迫候選落選）。
    """
    runner = pytest.importorskip("scripts.hv_sitl_run")
    scenarios = pytest.importorskip("scripts.hv_sitl_scenarios")
    home = runner.HOME
    alts = [(23.972571, 120.90), (home[0] + _PAD_N600, home[1])][:max(0, n_sites - 1)]

    orig_init = SC.SwarmCoordinator.__init__
    orig_alts = runner.ALTERNATES
    orig_guard = SC.recovery_regression_guard

    def patched(self, *a, **kw):
        orig_init(self, *a, **kw)
        self.hv_recovery_auto = auto

    logging.disable(logging.WARNING)
    SC.SwarmCoordinator.__init__ = patched                      # type: ignore[method-assign]
    runner.ALTERNATES = alts
    if guard is not None:
        SC.recovery_regression_guard = guard
    try:
        hv, report = runner._plan_gui(scenarios.SCENARIOS[_SCENARIO])
    finally:
        SC.SwarmCoordinator.__init__ = orig_init                # type: ignore[method-assign]
        runner.ALTERNATES = orig_alts
        SC.recovery_regression_guard = orig_guard
        logging.disable(logging.NOTSET)

    plan = hv.last_plan
    return ({u: p.landing_site for u, p in sorted(plan.plans.items())},
            round(max(float(p.t_land_s) for p in plan.plans.values()), 1),
            [n for n in (report.get("validation_notes") or []) if "recovery_auto" in n])


@pytest.fixture(scope="module")
def off():
    return _plan(auto=False, n_sites=3)


def test_disabled_lands_everyone_at_home(off):
    """前置斷言：關掉時就是接線前的行為（全員落 H）。"""
    land, ms, notes = off
    assert set(land.values()) == {"H"}, land
    assert notes == [], "關掉了卻還是發了 note"
    assert ms > 1400.0, ms                                      # 前置：基準確實是慢的


def test_no_user_declared_alternate_is_inert(off):
    """使用者**沒給**備降點 ⇒ 完全不動、不多跑規劃、不發 note。

    ⚠⚠⚠ 這條抓到過一個真的缺陷：轉接層在沒給備降點時會**自己合成一個**
    （home 背向區域形心 400 m），而它自己就發 `alternate_auto_placed` 警告說那個點
    「不見得能降落」。自動指派原本會把飛機指到那個系統發明的坪上，實測被 validate
    擋下（conflicts=1）—— 守衛接住了，但白跑一次規劃，而且方向本來就錯。
    修法是只採計使用者宣告過的站點。

    ⚠ 這條同時保證既有的單場坪情境不會因為這個功能多花規劃時間。

    ⚠⚠ 對照組必須是**同一組站點、只切換開關**。不可以拿 `off`（兩個使用者備降點）
    來比：站點集合不同時 makespan 本來就會不一樣（`planner` 的
    `e_alt = min(point_wh … for s in sites)` 對**所有已宣告**站取 min ⇒ 多宣告一個坪
    會放寬全隊的航路能量檢查）。我第一版就是這樣比，量到 1552.7 vs 1450.8 而誤判成缺陷。
    """
    base_land, base_ms, base_notes = _plan(auto=False, n_sites=1)
    land, ms, notes = _plan(auto=True, n_sites=1)

    assert set(base_land.values()) == {"H"} and base_notes == []      # 前置斷言
    assert set(land.values()) == {"H"}, land
    assert notes == [], notes
    assert ms == pytest.approx(base_ms, abs=0.5), (ms, base_ms)


def test_multiple_sites_change_the_landing_points_and_say_so(off):
    """⭐ 有兩個以上站點 ⇒ 產物要變，而且要留下可讀回的痕跡。"""
    land, ms, notes = _plan(auto=True, n_sites=3)
    assert len(set(land.values())) > 1, f"站點有三個卻全落同一處：{land}"
    assert ms < off[1] - 1.0, (ms, off[1])
    assert any(n.startswith("recovery_auto_applied") for n in notes), notes
    # note 要帶得出「改成什麼、換到多少、間距怎麼變」
    n = next(x for x in notes if x.startswith("recovery_auto_applied"))
    assert "makespan" in n and "間距" in n, n


def test_rejected_candidate_rolls_the_coordinator_back_to_baseline(off):
    """⭐⭐⭐ 守衛擋下候選 ⇒ `last_plan` 必須是**基準**那一份，不是落選的那份。

    ⚠⚠ 這是本功能最危險的地方：`hv_coordinator.last_plan` 是匯出／上傳讀的東西，
    留著落選的計畫等於飛出去的是沒被採用的那一份，而且**不會有任何徵狀**。
    """
    land, ms, notes = _plan(auto=True, n_sites=3,
                            guard=lambda base, cand, **kw: (False, "測試強制擋下"))
    assert set(land.values()) == {"H"}, f"候選被擋下卻留著它的落點：{land}"
    assert ms == pytest.approx(off[1], abs=0.5), (ms, off[1])
    assert any(n.startswith("recovery_auto_rejected") for n in notes), notes


def test_note_prefixes_are_registered_as_loud_in_the_gui():
    """⚠ 算得出來但 GUI 不顯示 = 等於沒算。"""
    pytest.importorskip("PyQt6")
    from ui.main_window import MainWindow
    assert "recovery_auto_applied" in MainWindow._HV_LOUD_NOTES
    assert "recovery_auto_rejected" in MainWindow._HV_LOUD_NOTES


def test_defaults():
    c = SC.SwarmCoordinator()
    assert c.hv_recovery_auto is True
    assert c.hv_recovery_sites is None
