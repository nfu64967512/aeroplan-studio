"""航測建圖修正（F2_gui_gates_release）：R1／R2 審查的 GUI／協調器 finding，每一項一組**值層級**測試。

不建 MainWindow（house style：未綁定方法 + 假 self，見 tests/test_hdmap_gui_mainwindow.py）；規劃是真的
（SwarmCoordinator HARVEST 建圖分支，1.2 × 1.0 km、1 Pioneer + 2 HERO，就是 R1 probe stagger2 的情境）。

(a) R1 #1  起飛時程：上傳報告列每架 T+秒；時程不同 ⇒ 一鍵起飛（auto_start／mission_start）被擋；
           「依時程放行」T+0 立刻、其他用（假）計時器在各自時程送同一支 auto_start，倒數、取消、逐機記錄；
           不放行已解鎖／AUTO／沒連線／寫入失敗的；非建圖 HARVEST 上傳也一樣。
(b) R1 #5  SERVOn_FUNCTION 只有目前是 0／10／-1 才可寫；在用的輸出（馬達、舵面）拒絕並說出會失去什麼。
(c) R1 #6  CAM1_FEEDBAK_PIN（及 POL／COMPID）寫了要重開飛控 ⇒ 不自動上傳。
(d) R2     LOG_BITMASK 要含 bit 12（位元測試，不是值比對）；寫入只用 OR 補這一位；LOG_DISARMED 顯示。
(e) R2     AIRSPEED_MIN／MAX 讀回要等於匯出 .param（AP_Int16 截斷抓得到），且仍包住任務空速；永不自動寫。
(f) R1 #4  建圖後端預設 relay；servo 有大聲警告。
(g) R1 #7  觸發率超過 CAM1_DURATION 允許的上限（0.1 s ⇒ 7.143 Hz）⇒ 擋規劃，訊息給確切上限。
(h) R1 #8  validate 失敗（4D 衝突／能量／沒跑成功）⇒ 建圖閘門擋上傳。
(i) R1 #9  讀回期間重新規劃 ⇒ 不送舊計畫的任務。
(j) R1 #10 只送 HUD 指令對象；已解鎖／AUTO／狀態不明的飛控不換任務。
(l) R2     航測處理對話框收到已完成的測區多邊形。
"""
from __future__ import annotations

import ast
import math
import os
import sys
import types
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine 再建 QApplication
    import ui.main_window as mw
    from PyQt6 import QtWidgets as _QtW
    from PyQt6.QtWidgets import QApplication
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

pytest.importorskip("hv_dccpp")

import hv_dccpp  # noqa: E402
from hv_dccpp.ardupilot import recommended_params  # noqa: E402
from mission import swarm_coordinator as sc_mod  # noqa: E402
from tests.test_hdmap_gui_coordinator import mapping_params, plan  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SRC = (ROOT / "ui" / "main_window.py").read_text(encoding="utf-8")


# ═══════════════════════════ 共用 ═══════════════════════════
@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(scope="module")
def mapped():
    """R1 probe stagger2 的情境（mapping_params() 預設：1.7 Hz、servo）。"""
    return plan(mapping_params())


@pytest.fixture
def qmb(monkeypatch):
    QMB = _QtW.QMessageBox
    calls = {"information": [], "warning": [], "critical": [], "question": []}
    ret = {"value": QMB.StandardButton.Ok}

    def mk(name):
        def _f(*a, **k):
            calls[name].append((a, k))
            return ret["value"]
        return staticmethod(_f)

    for name in calls:
        monkeypatch.setattr(QMB, name, mk(name))
    return {"calls": calls, "ret": ret, "QMB": QMB}


def _texts(qmb, *kinds):
    return [c[0][2] for k in kinds for c in qmb["calls"][k]]


class Link:
    """假 SITLLink：建圖上傳／一鍵起飛／依時程放行會碰到的方法。`fc` ＝ 飛控上的參數表。"""

    def __init__(self, sysid, fc, types_=None, *, armed=False, mode="QLOITER"):
        self.sysid_label = sysid
        self.fc = dict(fc)
        self.types = dict(types_ or {})
        self.read_calls, self.set_calls, self.uploads, self.tuples, self.started = [], [], [], [], []
        self.frame = types.SimpleNamespace(armed=armed, mode=mode)

    def read_params(self, names, timeout_s=3.0, retries=2, types_out=None):
        self.read_calls.append(list(names))
        if types_out is not None:
            for n in names:
                if self.fc.get(n) is not None:
                    types_out[n] = self.types.get(n, 9)
        return {n: self.fc.get(n) for n in names}

    def set_param(self, name, value, ptype="REAL32"):
        self.set_calls.append((name, value, ptype))
        self.fc[name] = float(value)

    def upload_mission_items(self, items, verify=True):
        self.uploads.append((items, verify))

    def upload_mission(self, wps):
        self.tuples.append(list(wps))

    def start_tlog(self, path):
        raise AssertionError("tlog 沒勾不該開")

    def tlog_status(self):
        return {"error": None}

    def get_latest_telemetry(self):
        return self.frame

    def auto_start(self):
        self.started.append("auto_start")

    def mission_start(self):
        self.started.append("mission_start")


class Panel:
    def __init__(self, params):
        self.params = dict(params)
        self.live, self.map_status = [], []

    def get_params(self):
        return dict(self.params)

    def set_mapping_live_status(self, text):
        self.live.append(text)

    def set_mapping_plan_status(self, text, ok=True):
        self.map_status.append((text, ok))


class Status:
    def __init__(self):
        self.msgs = []

    def showMessage(self, m, *a):
        self.msgs.append(m)


_BOUND = ("on_sitl_upload_mission", "_hv_mapping_upload_applies", "_on_hv_mapping_upload",
          "_on_mapping_readback_done", "_write_mapping_params", "_upload_mapping_missions",
          "_refresh_mapping_live_status", "_mapping_upload_stale_reason", "_abort_stale_mapping_upload",
          "_hv_upload_gate", "_refresh_release_countdown", "_sitl_broadcast", "on_release_by_schedule",
          "on_cancel_release", "_release_start_one", "_on_release_changed", "_on_mission_uploaded",
          "survey_area_polygons", "on_survey_process", "_show_mapping_plan_status", "_export_hv_mapping")


class Win:
    """假 self。綁不到的方法（修正前的舊碼沒有）就不綁 ⇒ 舊碼跑這份測試時是**斷言**紅，不是建構失敗。"""

    def __init__(self, coord, links, *, answer=False, panel_over=None):
        self._hv_coordinator = coord.hv_coordinator if coord is not None else None
        self._hv_swarm_coordinator = coord
        self._dccpp_result = coord.hv_coordinator.last_result if coord is not None else None
        self._sitl_links = list(links)
        self._write_pending_sysids = set()
        self._diamond_plans = None
        self.vtol_recon_panel = Panel(mapping_params(**(panel_over or {})))
        self.map_widget = types.SimpleNamespace(paths=[], _swarm_data=None, map_2d=None)
        self._status = Status()
        self.answer = answer
        self.asked = []
        self.readbacks = 0
        self.target_sysid = None
        self._hv_release_schedule = None
        self._release = None
        self._release_timer_factory = None
        self._release_clock = None
        self.survey_areas, self.corners = [], []
        for name in _BOUND:
            if hasattr(MainWindow, name):
                setattr(self, name, types.MethodType(getattr(MainWindow, name), self))

    def statusBar(self):
        return self._status

    def _target_links(self):
        if self.target_sysid is None:
            return list(self._sitl_links)
        return [l for l in self._sitl_links if int(l.sysid_label) == int(self.target_sysid)]

    def _update_write_wp_enabled(self):
        pass

    def _ensure_release_ticker(self):
        pass

    def _ask_write_mapping_params(self, bad):
        self.asked.append(bad)
        self.asked_text = mw.mapping_mismatch_text(bad)
        return self.answer

    def _start_mapping_readback(self, jobs):
        """同步版（真的是 QThread）：同一支 run_mapping_readback，結果交回同一支處理函式。"""
        self.readbacks += 1
        self._on_mapping_readback_done(mw.run_mapping_readback(jobs))


def _export_airspeed(hv, did):
    """匯出 .param 的 AIRSPEED_MIN/MAX（直接用 hv 匯出器的 recommended_params，不經 main_window）。"""
    up = hv.last_plan.plans[hv.id_map[did]]
    p = recommended_params(hv.last_ctx.specs[up.cls], None, hv.cfg, sensor_kinds=("rgb",),
                           camera_cfg=hv.camera_cfg, warnings=[])
    return {k: float(p[k]) for k in ("AIRSPEED_MIN", "AIRSPEED_MAX")}


def _fc(hv, did):
    """設定都對的飛控：相機參數＝匯出值、空速＝匯出值、LOG_BITMASK 含 bit 12、LOG_DISARMED=0。"""
    fc = dict(hv.expected_camera_params(did, camera_cfg=hv.camera_cfg))
    fc.update(_export_airspeed(hv, did))
    fc["LOG_BITMASK"] = float(65790 | 4096)
    fc["LOG_DISARMED"] = 0.0
    return fc


def _links(hv, **kw):
    return [Link(did, _fc(hv, did), **kw) for did in sorted(hv.id_map)]


def _plan_offsets(hv):
    """{drone_id: t_takeoff_s − 全隊最早}（直接從 last_plan 讀，與 export_all 的 delay_s 同一條）。"""
    t = {d: float(hv.last_plan.plans[u].t_takeoff_s) for d, u in hv.id_map.items() if u in hv.last_plan.plans}
    t0 = min(float(p.t_takeoff_s) for p in hv.last_plan.plans.values())
    return {d: v - t0 for d, v in t.items()}


class Timers:
    """假計時器工廠：記下 (延遲, 回呼)；fire() 才觸發；stop() 之後不會觸發。"""

    def __init__(self):
        self.made = []

    def __call__(self, delay_s, cb):
        t = types.SimpleNamespace(delay_s=float(delay_s), cb=cb, stopped=False)
        t.stop = lambda t=t: setattr(t, "stopped", True)
        self.made.append(t)
        return t

    @staticmethod
    def fire(t):
        if not t.stopped:
            t.cb()


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


# ═══════════════════════════ (a) R1 #1 起飛時程 ═══════════════════════════
def test_a_plan_really_staggers_takeoffs(mapped):
    """前提（R1 probe stagger2）：這份計畫的 4D 去衝突把起飛錯開了（不是同時起飛）。"""
    offs = _plan_offsets(mapped.hv_coordinator)
    assert len(offs) == 3 and max(offs.values()) - min(offs.values()) > 1.0, offs


def test_a_takeoff_offsets_helper_matches_the_plan(mapped):
    got, src = mw.harvest_takeoff_offsets(mapped.hv_coordinator)
    assert got == pytest.approx(_plan_offsets(mapped.hv_coordinator), abs=1e-9)
    assert src


def test_a_takeoff_schedule_method_is_preferred_and_garbage_falls_back_loudly():
    hv = types.SimpleNamespace(id_map={1: "A1", 2: "B2"},
                               last_plan=types.SimpleNamespace(plans={
                                   "A1": types.SimpleNamespace(t_takeoff_s=110.0),
                                   "B2": types.SimpleNamespace(t_takeoff_s=10.0)}))
    hv.takeoff_schedule = lambda: {1: {"offset_s": 300.0}, 2: {"offset_s": 0.0}}
    got, src = mw.harvest_takeoff_offsets(hv)
    assert got == {1: 300.0, 2: 0.0} and "takeoff_schedule" in src
    hv.takeoff_schedule = lambda: {"delay_s": {"A1": 7.0, "B2": 2.0}}
    assert mw.harvest_takeoff_offsets(hv)[0] == {1: 5.0, 2: 0.0}
    hv.takeoff_schedule = lambda: "看不懂"
    got, src = mw.harvest_takeoff_offsets(hv)
    assert got == {1: 100.0, 2: 0.0} and "t_takeoff_s" in src          # 退回計畫本身，且來源說出來


def test_a_upload_report_lists_every_takeoff_offset(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    offs = _plan_offsets(hv)
    links = _links(hv)
    win = Win(mapped, links)
    win.on_sitl_upload_mission()
    assert all(len(l.uploads) == 1 for l in links)
    info = _texts(qmb, "information")[-1]
    for did, off in offs.items():
        assert f"UAV-{did} ← 機 {did}（{hv.id_map[did]}）：起飛時程 T+{off:.0f} s" in info, info
    assert "依時程放行" in info and "一鍵起飛" in info
    sched = win._hv_release_schedule
    assert {s: e["offset_s"] for s, e in sched["entries"].items()} == pytest.approx(offs, abs=1e-9)
    assert sched["mapping"] is True


@pytest.mark.parametrize("cmd", ["auto_start", "mission_start"])
def test_a_broadcast_start_is_refused_when_offsets_differ(mapped, qmb, qt_app, cmd):
    hv = mapped.hv_coordinator
    offs = _plan_offsets(hv)
    links = _links(hv)
    win = Win(mapped, links)
    win.on_sitl_upload_mission()
    win._sitl_broadcast(cmd)
    assert not any(l.started for l in links), "一鍵起飛讓時程不同的飛機同時起飛"
    text = _texts(qmb, "critical")[-1]
    assert "依時程放行" in text
    for did, off in offs.items():
        assert f"T+{off:.0f} s" in text
    # 只選一架也不行（單獨提早／延後一樣破壞相對時程）
    win.target_sysid = min(offs, key=offs.get)
    win._sitl_broadcast(cmd)
    assert not any(l.started for l in links)


def test_a_broadcast_start_unchanged_without_staggered_schedule(qmb, qt_app):
    """不是 HARVEST 任務、或時程都相同（差 ≤ 1 s）⇒ 一鍵起飛照舊（行為不變）。"""
    links = [Link(s, {}) for s in (1, 2, 3)]
    win = Win(None, links)
    win._sitl_broadcast("auto_start")
    assert [l.started for l in links] == [["auto_start"]] * 3
    win._hv_release_schedule = {"entries": {s: {"sysid": s, "uav": s, "offset_s": 0.4 * s} for s in (1, 2, 3)},
                                "mapping": True, "upload_ok": {}}
    win._sitl_broadcast("auto_start")
    assert [l.started for l in links] == [["auto_start"] * 2] * 3
    assert not qmb["calls"]["critical"]


def _uploaded_win(mapped, qmb, *, ack=True, **kw):
    hv = mapped.hv_coordinator
    links = _links(hv, **kw)
    win = Win(mapped, links)
    win.on_sitl_upload_mission()
    assert win._write_pending_sysids == {l.sysid_label for l in links}
    if ack:                                    # 飛控回報寫入成功（SITLLink.mission_uploaded）
        for l in links:
            win._on_mission_uploaded(l.sysid_label, True, "ok")
    win._release_timer_factory = Timers()
    win._release_clock = Clock()
    qmb["ret"]["value"] = qmb["QMB"].StandardButton.Yes
    return win, links


def test_a_release_by_schedule_starts_now_and_later_with_countdown(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    offs = _plan_offsets(hv)
    base = min(offs.values())
    win, links = _uploaded_win(mapped, qmb)
    timers, clock = win._release_timer_factory, win._release_clock
    win.on_release_by_schedule()
    by_id = {l.sysid_label: l for l in links}
    now_ids = sorted(d for d, o in offs.items() if o - base <= 1.0)
    later = {d: o - base for d, o in offs.items() if o - base > 1.0}
    assert sorted(d for d, l in by_id.items() if l.started) == now_ids
    assert all(by_id[d].started == ["auto_start"] for d in now_ids)        # 同一支 auto_start
    assert sorted(t.delay_s for t in timers.made) == pytest.approx(sorted(later.values()), abs=1e-9)
    # 倒數（10 s 後）
    clock.now += 10.0
    win._refresh_release_countdown()
    text = win._release_countdown_text
    for d, delay in later.items():
        m, s = divmod(int(math.ceil(delay - 10.0)), 60)
        assert f"UAV-{d}：倒數 {m:d}:{s:02d} 後放行" in text, text
    # 依時間觸發
    for t in sorted(timers.made, key=lambda t: t.delay_s):
        clock.now = 1000.0 + t.delay_s
        timers.fire(t)
    assert all(l.started == ["auto_start"] for l in links)
    ev = {e["sysid"]: e for e in win._release.events}
    for d, o in offs.items():
        assert ev[d]["ok"] is True
        assert ev[d]["planned_s"] == pytest.approx(o - base, abs=1e-9)
        assert ev[d]["actual_s"] == pytest.approx(o - base if o - base > 1.0 else 0.0, abs=1e-9)
    assert not win._release.active


def test_a_cancel_release_stops_pending_aircraft(mapped, qmb, qt_app):
    offs = _plan_offsets(mapped.hv_coordinator)
    base = min(offs.values())
    win, links = _uploaded_win(mapped, qmb)
    win.on_release_by_schedule()
    pending = [d for d, o in offs.items() if o - base > 1.0]
    assert pending
    win.on_cancel_release()
    for t in win._release_timer_factory.made:
        assert t.stopped
        Timers.fire(t)
    by_id = {l.sysid_label: l for l in links}
    assert all(by_id[d].started == [] for d in pending)
    assert {e["sysid"] for e in win._release.events if e["error"] == "已取消"} == set(pending)
    assert not win._release.active


def test_a_release_refuses_whole_batch_when_one_is_armed(mapped, qmb, qt_app):
    win, links = _uploaded_win(mapped, qmb)
    links[1].frame.armed = True
    win.on_release_by_schedule()
    assert not any(l.started for l in links) and not win._release_timer_factory.made
    assert any("已解鎖" in t for t in _texts(qmb, "critical"))
    links[1].frame.armed = False
    links[2].frame.mode = "AUTO"
    win.on_release_by_schedule()
    assert not any(l.started for l in links)
    links[2].frame.mode = "QLOITER"
    links[0].frame = None                                                   # 沒有遙測 = 狀態不明
    win.on_release_by_schedule()
    assert not any(l.started for l in links)


def test_a_release_skips_disconnected_and_failed_write_after_asking(mapped, qmb, qt_app):
    offs = _plan_offsets(mapped.hv_coordinator)
    win, links = _uploaded_win(mapped, qmb)
    late = max(offs, key=offs.get)
    gone = next(l for l in links if l.sysid_label == late)
    win._sitl_links.remove(gone)                                            # 斷線
    failed = sorted(set(offs) - {late})[0]
    win._on_mission_uploaded(failed, False, "逾時")                         # 寫入失敗
    win.on_release_by_schedule()
    q = _texts(qmb, "question")[-1]
    assert f"UAV-{late} 沒有連線" in q and f"UAV-{failed} 任務寫入失敗" in q
    rel_ids = {e["sysid"] for e in win._release.events} | {s for s, *_ in win._release.remaining()}
    assert late not in rel_ids and failed not in rel_ids
    assert gone.started == [] and next(l for l in links if l.sysid_label == failed).started == []


def test_a_release_waits_for_the_write_confirmation(mapped, qmb, qt_app):
    """寫入還沒回報成功（mission_uploaded 還沒來）⇒ 不放行（不知道飛控上是哪一份任務）。"""
    win, links = _uploaded_win(mapped, qmb, ack=False)
    win.on_release_by_schedule()
    assert not any(l.started for l in links) and not win._release_timer_factory.made
    assert all(f"UAV-{l.sysid_label} 任務寫入還沒確認" in _texts(qmb, "critical")[-1] for l in links)


def test_a_release_refuses_at_fire_time_if_state_changed(mapped, qmb, qt_app):
    offs = _plan_offsets(mapped.hv_coordinator)
    base = min(offs.values())
    win, links = _uploaded_win(mapped, qmb)
    win.on_release_by_schedule()
    late = max(offs, key=offs.get)
    assert offs[late] - base > 1.0
    by_id = {l.sysid_label: l for l in links}
    by_id[late].frame.armed = True                                          # 倒數期間有人解鎖
    for t in win._release_timer_factory.made:
        Timers.fire(t)
    assert by_id[late].started == []
    ev = {e["sysid"]: e for e in win._release.events}
    assert ev[late]["ok"] is False and "已解鎖" in ev[late]["error"]
    assert any(f"UAV-{late}" in t and "已解鎖" in t for t in _texts(qmb, "warning"))


def test_a_reupload_or_write_wps_during_countdown_cancels_pending(mapped, qmb, qt_app):
    """倒數中又上傳／Write WPs 換了任務 ⇒ 舊時程的放行全部取消（不會用舊時刻放新任務）。"""
    offs = _plan_offsets(mapped.hv_coordinator)
    base = min(offs.values())
    win, links = _uploaded_win(mapped, qmb)
    win.on_release_by_schedule()
    assert win._release.active
    win.on_sitl_upload_mission()                                            # 重新上傳（同一份計畫）
    assert not win._release.active
    for t in win._release_timer_factory.made:
        Timers.fire(t)
    by_id = {l.sysid_label: l for l in links}
    assert all(by_id[d].started == [] for d, o in offs.items() if o - base > 1.0)
    # Write WPs 換掉其中一架的任務
    for l in links:
        win._on_mission_uploaded(l.sysid_label, True, "ok")
    win._release_timer_factory = Timers()
    win.on_release_by_schedule()
    assert win._release.active
    late = max(offs, key=offs.get)
    mw.forget_release_schedule(win, [late], reason="Write WPs")
    assert not win._release.active and late not in win._hv_release_schedule["entries"]


def test_a_failures_of_a_second_batch_are_still_shown(mapped, qmb, qt_app):
    offs = _plan_offsets(mapped.hv_coordinator)
    late = max(offs, key=offs.get)
    win, links = _uploaded_win(mapped, qmb)
    by_id = {l.sysid_label: l for l in links}
    for batch in range(2):
        win._release_timer_factory = Timers()
        win.on_release_by_schedule()
        by_id[late].frame.armed = True
        n_warn = len(qmb["calls"]["warning"])
        for t in win._release_timer_factory.made:
            Timers.fire(t)
        assert len(qmb["calls"]["warning"]) == n_warn + 1, f"第 {batch + 1} 批的放行失敗沒有顯示"
        by_id[late].frame.armed = False
        for l in links:
            l.started.clear()
            l.frame.armed = False


def test_a_release_without_harvest_upload_explains(qmb, qt_app):
    win = Win(None, [Link(1, {})])
    win.on_release_by_schedule()
    assert _texts(qmb, "information") and win._release is None


def test_a_default_timer_is_precise(qt_app):
    """預設計時器要 PreciseTimer：CoarseTimer 允許 5% 誤差（1516 s ⇒ ±76 s）。"""
    from PyQt6.QtCore import Qt
    t = mw._qt_single_shot(1516.0, lambda: None)
    try:
        assert t.timerType() == Qt.TimerType.PreciseTimer
        assert t.isSingleShot() and t.interval() == 1516000
    finally:
        t.stop()


def test_a_other_upload_paths_forget_the_schedule():
    win = types.SimpleNamespace(_hv_release_schedule={"entries": {1: {}, 2: {}}, "upload_ok": {1: None, 2: None}})
    mw.forget_release_schedule(win, [2], reason="Write WPs")
    assert set(win._hv_release_schedule["entries"]) == {1}
    mw.forget_release_schedule(win, None, reason="x")
    assert win._hv_release_schedule is None
    tree = ast.parse(SRC)
    for name in ("on_sitl_write_wp", "on_sitl_upload_mission"):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        called = {c.func.id for c in ast.walk(fn) if isinstance(c, ast.Call) and isinstance(c.func, ast.Name)}
        assert "forget_release_schedule" in called, name


def test_a_release_controls_sit_under_the_real_hud_start_button(qt_app):
    """真的 SITLHud：「依時程放行／取消放行」＋倒數插在「一鍵起飛」正下方（同一個起飛控制區）。"""
    from ui.widgets.sitl_hud import SITLHud
    hud = SITLHud(None)
    w = types.SimpleNamespace(sitl_hud=hud, on_release_by_schedule=lambda: None, on_cancel_release=lambda: None)
    MainWindow._install_release_controls(w)
    lay = hud.btn_auto_start.parentWidget().layout()
    assert lay.itemAt(lay.indexOf(hud.btn_auto_start) + 1).widget() is w._release_widget
    assert w.btn_release_schedule.text() == "依時程放行" and w.btn_release_cancel.text() == "取消放行"
    assert w.btn_release_cancel.isEnabled() is False                        # 沒有倒數時不能按
    # 建 HUD 的那一支（create_control_panel）真的有呼叫它
    fn = next(n for n in ast.walk(ast.parse(SRC)) if isinstance(n, ast.FunctionDef)
              and "self.sitl_hud = SITLHud(self)" in ast.unparse(n))
    assert "self._install_release_controls()" in ast.unparse(fn)


@pytest.fixture(scope="module")
def unmapped():
    return plan(mapping_params(hv_map_enabled=False))


class _ParamPanel:
    def __init__(self):
        self.vtol_transition_alt_spin = types.SimpleNamespace(value=lambda: 40.0)

    def get_parameters(self):
        return {"altitude": 120.0, "speed": 18.0, "turn_radius": 150.0}


def test_a_non_mapping_harvest_upload_reports_offsets_and_blocks_start(unmapped, qmb, qt_app):
    hv = unmapped.hv_coordinator
    assert hv.mapping_mode is None
    offs = _plan_offsets(hv)
    links = [Link(did, {}) for did in sorted(hv.id_map)]
    win = Win(unmapped, links)
    win.parameter_panel = _ParamPanel()
    win.flight_params = {"vehicle_type": "vtol"}
    win.on_sitl_upload_mission()
    assert all(len(l.tuples) == 1 for l in links)                           # 舊的 upload_tuples 路徑
    info = _texts(qmb, "information")[-1]
    for did, off in offs.items():
        assert f"T+{off:.0f} s" in info
    sched = win._hv_release_schedule
    assert {s: e["offset_s"] for s, e in sched["entries"].items()} == pytest.approx(offs, abs=1e-9)
    assert sched["mapping"] is False
    if max(offs.values()) - min(offs.values()) > 1.0:
        win._sitl_broadcast("auto_start")
        assert not any(l.started for l in links)


# ═══════════════════════════ (b) R1 #5 SERVOn_FUNCTION ═══════════════════════════
@pytest.mark.parametrize("current, label", [
    (36.0, "36 = 馬達 4（Motor4）"), (33.0, "33 = 馬達 1（Motor1）"), (40.0, "40 = 馬達 8（Motor8）"),
    (4.0, "4 = 副翼（Aileron）"), (19.0, "19 = 升降舵（Elevator）"), (21.0, "21 = 方向舵（Rudder）"),
    (70.0, "70 = 油門（Throttle）"), (79.0, "79 = V 尾左（VTailLeft）"), (80.0, "80 = V 尾右（VTailRight）"),
    (77.0, "77 = 左升降副翼（ElevonLeft）"), (99.0, "功能 99"),
])
def test_b_servo_function_in_use_is_not_writable(current, label):
    rows = {r["name"]: r for r in mw.mapping_readback_rows({"SERVO10_FUNCTION": 10.0}, {"SERVO10_FUNCTION": current},
                                                           {}, [19.0])}
    r = rows["SERVO10_FUNCTION"]
    assert r["ok"] is False and r["writable"] is False
    assert label in r["refuse"] and "相機觸發" in r["refuse"]
    assert label in mw.mapping_mismatch_text({1: {"rows": [r]}})


@pytest.mark.parametrize("current, want, writable", [
    (0.0, 10.0, True), (-1.0, 10.0, True), (10.0, -1.0, True), (0.0, -1.0, True), (None, 10.0, False),
])
def test_b_servo_function_writable_only_from_disabled_or_camera(current, want, writable):
    r = {x["name"]: x for x in mw.mapping_readback_rows({"SERVO9_FUNCTION": want}, {"SERVO9_FUNCTION": current},
                                                        {}, [])}["SERVO9_FUNCTION"]
    assert r["ok"] is False and r["writable"] is writable


def test_b_upload_refuses_to_overwrite_a_motor_output(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links(hv)
    assert "SERVO10_FUNCTION" in links[0].fc                                # servo 後端：SERVO10 = 10
    links[0].fc["SERVO10_FUNCTION"] = 36.0                                  # 這個輸出是馬達 4
    win = Win(mapped, links, answer=True)
    win.on_sitl_upload_mission()
    assert "Motor4" in win.asked_text
    assert not any(n == "SERVO10_FUNCTION" for n, *_ in links[0].set_calls)
    assert links[0].fc["SERVO10_FUNCTION"] == 36.0
    assert not any(l.uploads for l in links)


# ═══════════════════════════ (c) R1 #6 重開飛控清單 ═══════════════════════════
@pytest.mark.parametrize("name", ["CAM1_FEEDBAK_PIN", "CAM1_FEEDBAK_POL", "CAM1_COMPID", "CAM1_TYPE",
                                  "RELAY1_PIN", "RELAY1_FUNCTION", "SERVO9_FUNCTION"])
def test_c_reboot_required_names(name):
    bad = {1: {"rows": [{"name": name, "expected": 50.0, "actual": -1.0, "ok": False, "writable": True,
                         "ptype": 2}]}}
    assert mw.mapping_reboot_names(bad) == {1: [name]}


@pytest.mark.parametrize("name", ["CAM1_DURATION", "CAM1_INTRVAL_MIN", "CAM_MAX_ROLL", "CAM1_TRIGG_DIST",
                                  "CAM1_RELAY_ON", "LOG_BITMASK"])
def test_c_runtime_params_do_not_ask_for_reboot(name):
    bad = {1: {"rows": [{"name": name, "expected": 1.0, "actual": 0.0, "ok": False, "writable": True,
                         "ptype": 9}]}}
    assert mw.mapping_reboot_names(bad) == {}


def test_c_feedback_pin_write_is_not_auto_uploaded(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links(hv)
    links[0].fc["CAM1_FEEDBAK_PIN"] = 51.0                                  # 預期 -1
    win = Win(mapped, links, answer=True)
    win.on_sitl_upload_mission()
    assert ("CAM1_FEEDBAK_PIN", -1.0) in [(n, v) for n, v, _t in links[0].set_calls]
    assert not any(l.uploads for l in links)
    assert any("重開" in t and "CAM1_FEEDBAK_PIN" in t for t in _texts(qmb, "warning"))


# ═══════════════════════════ (d) R2 LOG_BITMASK／LOG_DISARMED ═══════════════════════════
@pytest.mark.parametrize("value, ok", [(65790.0, False), (65790.0 + 4096.0, True), (4096.0, True), (-1.0, True),
                                       (0.0, False)])
def test_d_log_bitmask_is_a_bit_test(value, ok):
    r = {x["name"]: x for x in mw.mapping_readback_rows({}, {"LOG_BITMASK": value}, {}, [])}["LOG_BITMASK"]
    assert r["ok"] is ok and r["kind"] == "bit"
    if not ok:
        assert r["writable"] is True and r["write_value"] == float(int(value) | 4096)
        assert f"{int(value)} OR 4096 = {int(value) | 4096}" in r["write_note"]


def test_d_unreadable_log_bitmask_blocks_and_is_not_writable():
    r = {x["name"]: x for x in mw.mapping_readback_rows({}, {"LOG_BITMASK": None}, {}, [])}["LOG_BITMASK"]
    assert r["ok"] is False and r["writable"] is False


def test_d_log_disarmed_is_shown_but_does_not_block():
    rows = {x["name"]: x for x in mw.mapping_readback_rows({}, {"LOG_DISARMED": 0.0, "LOG_BITMASK": 4096.0}, {}, [])}
    assert rows["LOG_DISARMED"]["ok"] is True and "地面測試拍照不會進日誌" in rows["LOG_DISARMED"]["info"]


def test_d_upload_writes_only_the_camera_bit_then_uploads(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links(hv)
    links[0].fc["LOG_BITMASK"] = 65790.0                                    # 沒有 bit 12
    win = Win(mapped, links, answer=True)
    win.on_sitl_upload_mission()
    assert "65790 OR 4096 = 69886" in win.asked_text
    assert [(n, v) for n, v, _t in links[0].set_calls] == [("LOG_BITMASK", 69886.0)]
    assert all(len(l.uploads) == 1 for l in links)                          # 不用重開 ⇒ 重讀相符就上傳
    info = _texts(qmb, "information")[-1]
    assert "LOG_DISARMED=0" in info


def test_d_upload_reads_log_params(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links(hv)
    win = Win(mapped, links)
    win.on_sitl_upload_mission()
    for l in links:
        assert {"LOG_BITMASK", "LOG_DISARMED"} <= set(l.read_calls[0])


# ═══════════════════════════ (e) R2 AIRSPEED 與匯出值比對 ═══════════════════════════
def test_e_airspeed_must_equal_the_exported_param():
    exp = {"AIRSPEED_MIN": 16.0, "AIRSPEED_MAX": 26.6}
    rows = {r["name"]: r for r in mw.mapping_readback_rows(
        {}, {"AIRSPEED_MIN": 16.0, "AIRSPEED_MAX": 26.0}, {"AIRSPEED_MAX": 4}, [19.0], airspeed_expected=exp)}
    assert rows["AIRSPEED_MIN"]["ok"] is True
    r = rows["AIRSPEED_MAX"]
    assert r["ok"] is False and r["writable"] is False                     # AP_Int16 把 26.6 存成 26
    assert "26.6" in r["detail"] and "26" in r["detail"]
    ok = {r["name"]: r for r in mw.mapping_readback_rows(
        {}, {"AIRSPEED_MIN": 16.0, "AIRSPEED_MAX": 26.6}, {}, [19.0], airspeed_expected=exp)}
    assert ok["AIRSPEED_MAX"]["ok"] is True


def test_e_speed_range_check_is_kept():
    exp = {"AIRSPEED_MIN": 18.0, "AIRSPEED_MAX": 26.0}
    rows = {r["name"]: r for r in mw.mapping_readback_rows(
        {}, {"AIRSPEED_MIN": 18.0, "AIRSPEED_MAX": 26.0}, {}, [17.0, 27.0], airspeed_expected=exp)}
    assert rows["AIRSPEED_MIN"]["ok"] is False and "17" in rows["AIRSPEED_MIN"]["detail"]
    assert rows["AIRSPEED_MAX"]["ok"] is False and "27" in rows["AIRSPEED_MAX"]["detail"]


def test_e_upload_blocks_on_airspeed_differing_from_export_and_never_writes(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links(hv)
    did = links[0].sysid_label
    want = _export_airspeed(hv, did)
    links[0].fc["AIRSPEED_MAX"] = want["AIRSPEED_MAX"] + 2.0                # 還包得住任務空速，但不是匯出值
    win = Win(mapped, links, answer=True)
    win.on_sitl_upload_mission()
    assert not any(n.startswith("AIRSPEED") for l in links for n, *_ in l.set_calls)
    assert not any(l.uploads for l in links)
    text = "\n".join(_texts(qmb, "warning", "critical") + [getattr(win, "asked_text", "")])
    assert "AIRSPEED_MAX" in text and f"{want['AIRSPEED_MAX']:g}（匯出 .param）" in text


# ═══════════════════════════ (f) R1 #4 後端預設 relay、servo 警告 ═══════════════════════════
def test_f_servo_backend_warns_loudly():
    s = sc_mod.build_mapping_setup(mapping_params(hv_map_backend="servo"))
    assert "servo 後端會持續輸出 PWM 脈衝，只適用 SITL；實機請用 relay" in s["warnings"]
    r = sc_mod.build_mapping_setup(mapping_params(hv_map_backend="relay", hv_map_firmware="4.5.7",
                                                  hv_map_relay_pin=50, hv_map_relay_aux_servo=9))
    assert not any("servo 後端" in w for w in r["warnings"])


def test_f_panel_defaults_to_relay_and_shows_requirement_and_servo_warning(qt_app):
    from ui.widgets.vtol_recon_panel import MAPPING_BACKENDS, VTOLReconPanel
    assert MAPPING_BACKENDS[0][0] == "relay"
    panel = VTOLReconPanel()
    assert panel.get_params()["hv_map_backend"] == "relay"
    panel.hv_map_check.setChecked(True)
    txt = panel.hv_map_warn_label.text()
    assert "無法規劃" in txt and "firmware" in txt                          # relay 要韌體版本：面板直接說
    panel.hv_map_backend_combo.setCurrentIndex(panel.hv_map_backend_combo.findData("servo"))
    assert "servo 後端會持續輸出 PWM 脈衝，只適用 SITL；實機請用 relay" in panel.hv_map_warn_label.text()


# ═══════════════════════════ (g) R1 #7 觸發率 vs CAM1_DURATION ═══════════════════════════
@pytest.mark.parametrize("duration, spacing", [(0.1, 0.14), (0.2, 0.24), (0.12, 0.16), (0.0, 0.04), (0.13, 0.16)])
def test_g_min_trigger_spacing(duration, spacing):
    assert sc_mod.trigger_min_spacing_s(duration) == pytest.approx(spacing, abs=1e-12)


@pytest.mark.parametrize("backend, extra", [("relay", {"hv_map_firmware": "4.5.7", "hv_map_relay_pin": 50}),
                                            ("servo", {})])
@pytest.mark.parametrize("hz", [12.5, 20.0, 10.0, 8.4])
def test_g_rate_above_the_duration_cap_is_refused(backend, extra, hz):
    with pytest.raises(ValueError) as ei:
        sc_mod.build_mapping_setup(mapping_params(hv_map_backend=backend, hv_map_trigger_hz=hz,
                                                  hv_map_trigger_verified=True, **extra))
    msg = str(ei.value)
    assert "7.143 Hz" in msg and "0.14 s" in msg and "CAM1_DURATION 0.1 s" in msg


@pytest.mark.parametrize("hz, interval", [(7.0, 0.16), (7.14, 0.16), (8.0, 0.14), (8.3, 0.14)])
def test_g_rates_within_the_cap_are_accepted(hz, interval):
    s = sc_mod.build_mapping_setup(mapping_params(hv_map_trigger_hz=hz, hv_map_trigger_verified=True))
    assert s["interval_min_s"] == pytest.approx(interval, abs=1e-12)
    assert s["trigger_rate_planned_hz"] <= 1.0 / 0.14 + 1e-9
    assert s["effective"]["trigger_rate_cap_hz"] == pytest.approx(1.0 / 0.14)


def test_g_mavlink_backend_is_not_duration_limited():
    s = sc_mod.build_mapping_setup(mapping_params(hv_map_backend="mavlink_v2", hv_map_firmware="4.5.7",
                                                  hv_map_trigger_hz=12.5, hv_map_trigger_verified=True))
    assert s["interval_min_s"] == pytest.approx(0.08) and s["effective"]["trigger_rate_cap_hz"] is None


# ═══════════════════════════ (h) R1 #8 validate 失敗擋上傳 ═══════════════════════════
def test_h_gate_combines_validate_reasons():
    hv = types.SimpleNamespace(mapping_gate=lambda: {"uploadable": True, "reasons": []})
    sc = types.SimpleNamespace(hv_coordinator=hv, hv_validation_reasons=[])
    assert mw.hv_upload_gate(hv, sc) == {"uploadable": True, "reasons": []}
    sc.hv_validation_reasons = ["validate_failed(conflicts=2)"]
    assert mw.hv_upload_gate(hv, sc) == {"uploadable": False, "reasons": ["validate_failed(conflicts=2)"]}
    sc.hv_validation_reasons = None
    g = mw.hv_upload_gate(hv, sc)
    assert g["uploadable"] is False and "validate 沒有跑到" in g["reasons"][0]
    g = mw.hv_upload_gate(hv, types.SimpleNamespace(hv_coordinator=object(), hv_validation_reasons=[]))
    assert g["uploadable"] is False and "找不到這份計畫的 validate 結果" in g["reasons"][0]
    assert mw.hv_upload_gate(hv, None)["uploadable"] is False


def test_h_validate_failure_blocks_the_mapping_gate_and_the_upload(monkeypatch, qmb, qt_app):
    fake = types.SimpleNamespace(passed=False, coverage_ok=True, energy_ok=False, conflicts=[1, 2, 3],
                                 min_sep_h_m=12.2)
    monkeypatch.setattr(hv_dccpp, "validate", lambda *a, **k: fake)
    coord = plan(mapping_params())
    want = "validate_failed(energy, conflicts=3)"
    assert coord.hv_validation_reasons == [want]
    assert coord.hv_mapping_gate["uploadable"] is False and want in coord.hv_mapping_gate["reasons"]
    assert coord.hv_coordinator.mapping_gate()["uploadable"] is True        # 轉接層自己看不到 validate
    links = _links(coord.hv_coordinator)
    win = Win(coord, links)
    win.on_sitl_upload_mission()
    assert not any(l.read_calls or l.uploads for l in links)
    assert want in _texts(qmb, "critical")[-1]


def test_h_validate_crash_blocks_too(monkeypatch):
    def boom(*a, **k):
        raise RuntimeError("boom")
    monkeypatch.setattr(hv_dccpp, "validate", boom)
    coord = plan(mapping_params())
    assert coord.hv_validation_reasons and "驗證本身沒跑成功" in coord.hv_validation_reasons[0]
    assert "RuntimeError: boom" in coord.hv_validation_reasons[0]
    assert coord.hv_mapping_gate["uploadable"] is False


def test_h_plan_status_and_export_use_the_combined_gate(mapped, qmb, qt_app, monkeypatch, tmp_path):
    hv = mapped.hv_coordinator
    win = Win(mapped, [])
    win._show_mapping_plan_status(hv)
    text, ok = win.vtol_recon_panel.map_status[-1]
    assert ok is True and "建圖閘門：可上傳" in text
    offs = _plan_offsets(hv)
    assert "起飛時程：" in text and all(f"機 {d} T+{o:.0f} s" in text for d, o in offs.items())
    assert "依時程放行" in text                                              # 時程不同 ⇒ 先告訴操作員
    assert "servo（只適用 SITL；實機請用 relay）" in text
    monkeypatch.setattr(mapped, "hv_validation_reasons", ["validate_failed(conflicts=1)"])
    win._show_mapping_plan_status(hv)
    text, ok = win.vtol_recon_panel.map_status[-1]
    assert ok is False and "validate_failed(conflicts=1)" in text
    win._export_hv_mapping(str(tmp_path))
    assert any("validate_failed(conflicts=1)" in t and "不允許上傳" in t for t in _texts(qmb, "warning"))


# ═══════════════════════════ (i) R1 #9 讀回期間重新規劃 ═══════════════════════════
@pytest.mark.parametrize("how", ["new_result", "new_coordinator", "other_result_on_screen"])
def test_i_replan_during_readback_aborts_the_upload(mapped, qmb, qt_app, monkeypatch, how):
    hv = mapped.hv_coordinator
    links = _links(hv)
    win = Win(mapped, links)

    def _readback(jobs):
        res = mw.run_mapping_readback(jobs)
        if how == "new_result":
            monkeypatch.setattr(hv, "last_result", {"replanned": True})
            win._dccpp_result = hv.last_result
        elif how == "new_coordinator":
            win._hv_coordinator = types.SimpleNamespace(last_result=hv.last_result, mapping_mode=hv.mapping_mode)
        else:
            win._dccpp_result = {"plain_dccpp": True}
        win._on_mapping_readback_done(res)

    win._start_mapping_readback = _readback
    win.on_sitl_upload_mission()
    assert all(l.read_calls for l in links)
    assert not any(l.uploads for l in links), "讀回期間換了計畫，舊計畫的任務仍被送出"
    assert any("取消上傳" in c[0][1] for c in qmb["calls"]["critical"])


def test_i_replan_while_write_dialog_is_open_writes_nothing(mapped, qmb, qt_app, monkeypatch):
    hv = mapped.hv_coordinator
    links = _links(hv)
    links[0].fc["CAM1_INTRVAL_MIN"] = 1.0
    win = Win(mapped, links, answer=True)

    def _ask(bad):
        monkeypatch.setattr(hv, "last_result", {"replanned": True})
        win._dccpp_result = hv.last_result
        return True

    win._ask_write_mapping_params = _ask
    win.on_sitl_upload_mission()
    assert not any(l.set_calls for l in links) and not any(l.uploads for l in links)


# ═══════════════════════════ (j) R1 #10 指令對象與飛控狀態 ═══════════════════════════
def test_j_only_hud_targets_are_read_and_uploaded(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links(hv)
    win = Win(mapped, links)
    tgt = sorted(hv.id_map)[1]
    win.target_sysid = tgt
    win.on_sitl_upload_mission()
    for l in links:
        if l.sysid_label == tgt:
            assert len(l.uploads) == 1 and l.read_calls
            assert l.uploads[0][0] == hv.mission_items(tgt, camera_cfg=hv.camera_cfg)   # 對應不因單選而變
        else:
            assert not l.uploads and not l.read_calls
    others = sorted(d for d in hv.id_map if d != tgt)
    assert f"sysid {others}" in _texts(qmb, "information")[-1]


@pytest.mark.parametrize("state", ["armed", "auto", "no_telemetry", "no_heartbeat"])
def test_j_armed_or_auto_or_unknown_flight_controller_is_refused(mapped, qmb, qt_app, state):
    hv = mapped.hv_coordinator
    links = _links(hv)
    if state == "armed":
        links[1].frame.armed = True
    elif state == "auto":
        links[1].frame.mode = "AUTO"
    elif state == "no_telemetry":
        links[1].frame = None
    else:
        links[1].frame.mode = "---"
    win = Win(mapped, links)
    win.on_sitl_upload_mission()
    assert not any(l.read_calls or l.uploads for l in links)
    assert f"UAV-{links[1].sysid_label}" in _texts(qmb, "critical")[-1]


def test_j_armed_during_readback_cancels_the_upload(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links(hv)
    win = Win(mapped, links)

    def _readback(jobs):
        res = mw.run_mapping_readback(jobs)
        links[2].frame.armed = True
        win._on_mapping_readback_done(res)

    win._start_mapping_readback = _readback
    win.on_sitl_upload_mission()
    assert not any(l.uploads for l in links)


# ═══════════════════════════ (l) R2 航測處理對話框收到測區多邊形 ═══════════════════════════
def test_l_survey_dialog_receives_committed_polygons(qt_app, monkeypatch, qmb):
    made = []

    class Dlg:
        def __init__(self, parent=None, map_widget=None, polygons=None):
            made.append({"parent": parent, "map_widget": map_widget, "polygons": polygons})

        def show(self):
            pass

    fake = types.ModuleType("ui.dialogs.survey_process_dialog")
    fake.SurveyProcessDialog = Dlg
    monkeypatch.setitem(sys.modules, "ui.dialogs.survey_process_dialog", fake)
    win = Win(None, [])
    a1 = [(25.0, 121.0), (25.0, 121.01), (25.01, 121.01)]
    a2 = [(25.1, 121.0), (25.1, 121.01), (25.11, 121.01), (25.11, 121.0)]
    win.survey_areas = [a1, a2]                     # 「完成本區」之後：地圖角點已清空
    win.on_survey_process()
    assert made and made[0]["polygons"] == [a1, a2] and made[0]["map_widget"] is win.map_widget


def test_l_old_dialog_without_polygons_is_explained_not_silently_degraded(qt_app, monkeypatch, qmb):
    made = []

    class Old:
        def __init__(self, parent=None, map_widget=None):
            made.append(1)

        def show(self):
            pass

    fake = types.ModuleType("ui.dialogs.survey_process_dialog")
    fake.SurveyProcessDialog = Old
    monkeypatch.setitem(sys.modules, "ui.dialogs.survey_process_dialog", fake)
    win = Win(None, [])
    win.survey_areas = [[(25.0, 121.0), (25.0, 121.01), (25.01, 121.01)]]
    win.on_survey_process()
    assert made == [] and any("polygons" in t for t in _texts(qmb, "warning"))
