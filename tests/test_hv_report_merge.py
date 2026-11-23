"""HARVEST 同一次規劃的多則報告要在 MainWindow 端**累加**，不能後一則整份蓋掉前一則。

SwarmCoordinator 的 HV 分支一次規劃最多送三則（`mission/swarm_coordinator.py`）：
    前置  {'validation_note(s)': _prenotes}               ← World Terrain 路徑必有 terrain_source(...)
    轉接層 {'status', 'notes', 'uncovered', ...}           ← 勝出那次規劃的那一份
    尾端  {'validation_note(s)': _vnotes}                  ← 只有 notes，沒有 uncovered／status
舊的 `_on_hv_report` 每收到一則就把 `_hv_last_report` 整個覆寫 ⇒ 尾端那則把轉接層的 uncovered／status
與 coverage_altitude_clamped 等 loud note 吃掉 ⇒ 作業區真的有未覆蓋時「HARVEST 覆蓋不完整」不跳
（2026-10-15 A6 審查發現的既有缺陷，交接 docs/交接_2026-10-15晚_高精地圖續.md §4）。

不建 MainWindow（house style：未綁定方法 + 假 self，見 tests/test_hdmap_gui_mainwindow.py）、不起 SITL。
"""
from __future__ import annotations

import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")

try:
    from ui.main_window import MainWindow
    from PyQt6 import QtWidgets as _QtW
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")


# ── 假物件 ───────────────────────────────────────────────────────────
class _StatusBar:
    def __init__(self):
        self.messages = []

    def showMessage(self, text, timeout=0):          # noqa: N802 - Qt 介面名稱
        self.messages.append(text)

    def clearMessage(self):                           # noqa: N802
        self.messages.append(None)


def _fake_window():
    """`_on_hv_report`／`_hv_show_report_banner`／`_start_hv_planning` 會碰到的屬性就這些。"""
    w = types.SimpleNamespace()
    w._HV_LOUD_NOTES = MainWindow._HV_LOUD_NOTES
    w._hv_last_report = {}
    w._hv_worker = None
    w._hv_display_ctx = {}
    w._status = _StatusBar()
    w.statusBar = lambda: w._status
    for name in ('_on_hv_report', '_hv_show_report_banner', '_hv_planning_in_flight',
                 '_hv_disconnect_worker', '_start_hv_planning'):
        setattr(w, name, types.MethodType(getattr(MainWindow, name), w))
    for name in ('_on_hv_progress', '_on_hv_failed', '_on_hv_finished', '_on_hv_cancelled'):
        setattr(w, name, lambda *a, **k: None)
    return w


@pytest.fixture
def qmb(monkeypatch):
    QMB = _QtW.QMessageBox
    calls = {"information": [], "warning": [], "critical": []}

    def mk(name):
        def _f(*a, **k):
            calls[name].append(a)
            return QMB.StandardButton.Ok
        return staticmethod(_f)

    for name in calls:
        monkeypatch.setattr(QMB, name, mk(name))
    return calls


# ── 同一次規劃的三則報告（形狀照 swarm_coordinator.py 與 aeroplan_adapter.py 的 payload）──
TERRAIN_SOURCE = 'terrain_source(Cesium World Terrain；個人/非商用)'
TERRAIN_DATUM = 'terrain_datum(DEM=橢球高；N 未知 ⇒ 只准相對高度)'
UNCOVERED = [['AREA_1', 'no_feasible_class'], ['AREA_2:cross', 'cross_energy_infeasible']]

PRE = {'validation_note': TERRAIN_SOURCE, 'validation_notes': [TERRAIN_SOURCE]}
ADAPTER = {'status': 'GO_WITH_WARNINGS', 'errors': [], 'warnings': [],
           'notes': ['coverage_altitude_clamped(100.0->120.0; min=120)',
                     'alternate_auto_placed(ALT1 400 m 背向形心)',
                     'landing_not_at_requested_site(UAV-2 → ALT1)',
                     TERRAIN_DATUM,
                     'provisional_params(A: a_tr_mps2)'],         # 安靜的 note：不該進 loud
           'makespan_s': 812.0, 'timing_ms': {'total': 3.0},
           'uncovered': [list(u) for u in UNCOVERED]}
# 尾端：前置 notes ＋ 原樣轉送的 terrain_datum(…)（HV_FORWARDED_ADAPTER_NOTE_PREFIXES）＋ 收尾的 note
TAIL_NOTES = [TERRAIN_SOURCE, TERRAIN_DATUM, 'partial_fleet_assigned(1/2 台；沒分到工作的構型 [\'A\'])']
TAIL = {'validation_note': ' ; '.join(TAIL_NOTES), 'validation_notes': list(TAIL_NOTES)}


def _feed(w, *payloads):
    for p in payloads:
        MainWindow._on_hv_report(w, p)


def test_tail_report_does_not_erase_the_adapters_uncovered(qmb):
    """核心：轉接層有 uncovered ⇒ 後面再來一則只有 validation_notes 的尾端報告，未覆蓋對話框仍要跳。"""
    w = _fake_window()
    _feed(w, PRE, ADAPTER, TAIL)
    rep = w._hv_last_report
    assert rep['uncovered'] == UNCOVERED
    assert rep['status'] == 'GO_WITH_WARNINGS'          # 尾端沒有 status ⇒ 取最後一個非空值

    MainWindow._hv_show_report_banner(w)
    assert len(qmb['warning']) == 1, '「HARVEST 覆蓋不完整」對話框沒有跳'
    _parent, title, text = qmb['warning'][0][:3]
    assert title == 'HARVEST 覆蓋不完整'
    for key, reason in UNCOVERED:
        assert key in text and reason in text, text
    assert any(m and '未覆蓋' in m for m in w._status.messages)


def test_loud_notes_from_every_report_are_kept_once():
    """轉接層的 loud note 不能被尾端吃掉；尾端原樣轉送的 terrain_datum(…) 不能變兩份；安靜的不進來。"""
    w = _fake_window()
    _feed(w, PRE, ADAPTER, TAIL)
    loud = w._hv_last_report['loud_notes']
    for prefix in ('coverage_altitude_clamped', 'alternate_auto_placed',
                   'landing_not_at_requested_site', 'terrain_source', 'terrain_datum',
                   'partial_fleet_assigned'):
        assert sum(1 for n in loud if n.startswith(prefix)) == 1, (prefix, loud)
    assert not any(n.startswith('provisional_params') for n in loud), loud
    assert len(loud) == len(set(loud)), loud
    # 依到達順序：前置 → 轉接層 → 尾端新增的
    assert loud.index(TERRAIN_SOURCE) < loud.index(TERRAIN_DATUM) < loud.index(TAIL_NOTES[-1])


def test_status_is_the_last_non_empty_one():
    w = _fake_window()
    _feed(w, PRE)
    assert w._hv_last_report['status'] is None
    _feed(w, ADAPTER, TAIL)
    assert w._hv_last_report['status'] == 'GO_WITH_WARNINGS'
    _feed(w, dict(ADAPTER, status='GO'), TAIL)           # 後面真的有新的非空 status ⇒ 用新的
    assert w._hv_last_report['status'] == 'GO'


def test_uncovered_is_deduplicated_by_value():
    """uncovered 的元素是 list（不可雜湊）；同一項重送不能變兩項，新的項要接上。"""
    w = _fake_window()
    _feed(w, ADAPTER, dict(ADAPTER, uncovered=[['AREA_1', 'no_feasible_class'], ['AREA_3', 'x']]), TAIL)
    assert w._hv_last_report['uncovered'] == UNCOVERED + [['AREA_3', 'x']]


def test_fully_covered_plan_shows_loud_notes_not_the_dialog(qmb):
    """對照組：沒有未覆蓋 ⇒ 不跳對話框，狀態列顯示被尾端保留下來的 loud note（不是只有尾端那幾則）。"""
    w = _fake_window()
    _feed(w, PRE, dict(ADAPTER, uncovered=[]), TAIL)
    MainWindow._hv_show_report_banner(w)
    assert qmb['warning'] == []
    assert w._status.messages and 'coverage_altitude_clamped' in w._status.messages[-1]


# ── 新規劃開始時清空（累加的反面：上一趟的未覆蓋不能帶到下一趟）────────────
class _Sig:
    def __init__(self):
        self.slots = []

    def connect(self, slot):
        self.slots.append(slot)

    def disconnect(self):
        self.slots.clear()


class _FakeWorker:
    """HarvestPlanningWorker 的替身：不開 QThread，只記住有沒有被 start。"""

    def __init__(self, coordinator, plan_kwargs, parent=None):
        self.coordinator = coordinator
        for s in ('progress', 'report', 'failed', 'finished_ok', 'cancelled'):
            setattr(self, s, _Sig())
        self.started = False
        self.running = False

    def start(self):
        self.started = True

    def isRunning(self):                              # noqa: N802
        return self.running

    def deleteLater(self):                            # noqa: N802
        pass


@pytest.fixture
def fake_worker(monkeypatch):
    import ui.workers.planning_worker as pw
    monkeypatch.setattr(pw, 'HarvestPlanningWorker', _FakeWorker)
    return _FakeWorker


def test_new_planning_starts_from_an_empty_report(fake_worker, qmb):
    w = _fake_window()
    _feed(w, PRE, ADAPTER, TAIL)                          # 上一趟：有未覆蓋
    assert w._hv_last_report['uncovered']

    MainWindow._start_hv_planning(w, object(), {}, {})
    assert w._hv_worker.started
    assert w._hv_last_report == {}, '新規劃沒清空 ⇒ 上一趟的未覆蓋會帶到這一趟'

    _feed(w, PRE, dict(ADAPTER, uncovered=[], notes=[]), dict(TAIL, validation_notes=[TERRAIN_SOURCE]))
    MainWindow._hv_show_report_banner(w)
    assert qmb['warning'] == [], '這一趟全覆蓋，卻跳了上一趟的未覆蓋對話框'
    assert not any(n.startswith('coverage_altitude_clamped') for n in w._hv_last_report['loud_notes'])


def test_a_rejected_request_does_not_wipe_the_running_plans_report(fake_worker):
    """清空要在「已有規劃在執行」的閘門**之後**：被拒的請求不能把正在跑那一趟已收到的報告清掉。"""
    w = _fake_window()
    MainWindow._start_hv_planning(w, object(), {}, {})
    running = w._hv_worker
    running.running = True
    _feed(w, PRE, ADAPTER)
    MainWindow._start_hv_planning(w, object(), {}, {})   # 被閘門擋下
    assert w._hv_worker is running
    assert w._hv_last_report['uncovered'] == UNCOVERED
