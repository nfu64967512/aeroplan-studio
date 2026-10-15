"""航測建圖：MainWindow 的上傳／讀回閘門／匯出／即時相機回授／tlog／選單／相機配置。

不建 MainWindow（QtWebEngine 在此環境建不起來）：用未綁定方法 + 假 self 跑**真的處理函式**
（house style，見 tests/test_flat_finder_gui.py、tests/test_mission_write.py）。
規劃本身是真的（SwarmCoordinator 的 HARVEST 建圖分支，見 tests/test_hdmap_gui_coordinator.py）。

鎖住的事：
* 建圖上傳走 `upload_mission_items(hv.mission_items(...), verify=True)`，每架送的就是 hv 匯出器的項目；
* 上傳前一定先在**工作執行緒**讀回參數（read_params），比對用 values_match；不符就擋、問要不要寫入，
  寫入走 set_param，寫完重讀重比；CAM1_TYPE／RELAY*／SERVOn_FUNCTION 要重開飛控 ⇒ 寫完不自動上傳；
* 任務裡每個 DO_CHANGE_SPEED 都要落在讀回的 AIRSPEED_MIN..MAX（超出 ArduPlane 會靜默忽略）；
* 建圖閘門（未覆蓋／mapping_incomplete）擋上傳；
* 照片數用 payload['new_photos'] 加總，**不是**訊號次數；gap 是「回授訊息漏收」不是漏拍。
"""
from __future__ import annotations

import ast
import datetime as _dt
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

from hv_dccpp.ardupilot import read_param_file, read_qgc_wpl  # noqa: E402
from hv_dccpp.integration.aeroplan_adapter import expected_photo_count  # noqa: E402
from tests.test_hdmap_gui_coordinator import mapping_params, plan  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SRC = (ROOT / "ui" / "main_window.py").read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(scope="module")
def mapped():
    return plan(mapping_params(hv_map_trigger_hz=2.0, hv_map_trigger_verified=True))


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


# ── 假物件 ───────────────────────────────────────────────────────────
class FakeLink:
    """假 SITLLink：只有建圖上傳會碰到的方法。`fc` 是飛控上的參數表（set_param 會改它）。"""

    def __init__(self, sysid, fc, types_=None, *, tlog_error=None, armed=False, mode="QLOITER"):
        self.sysid_label = sysid
        self.fc = dict(fc)
        self.types = dict(types_ or {})
        self.read_calls, self.set_calls, self.uploads, self.tlog_paths = [], [], [], []
        self.tlog_error = tlog_error
        self.order = []
        self.started = []
        # 假遙測：上傳／放行前要看「沒解鎖、不在 AUTO」（None ＝ 還沒收到遙測）
        self.frame = types.SimpleNamespace(armed=armed, mode=mode)

    def get_latest_telemetry(self):
        return self.frame

    def auto_start(self):
        self.started.append("auto_start")

    def mission_start(self):
        self.started.append("mission_start")

    def read_params(self, names, timeout_s=3.0, retries=2, types_out=None):
        self.read_calls.append((list(names), timeout_s, retries))
        if types_out is not None:
            for n in names:
                if self.fc.get(n) is not None:
                    types_out[n] = self.types.get(n, 9)
        return {n: self.fc.get(n) for n in names}

    def set_param(self, name, value, ptype="REAL32"):
        self.set_calls.append((name, value, ptype))
        self.fc[name] = float(value)

    def upload_mission_items(self, items, verify=True):
        self.order.append("upload")
        self.uploads.append((items, verify))

    def start_tlog(self, path):
        self.order.append("tlog")
        self.tlog_paths.append(str(path))
        return str(path)

    def stop_tlog(self):
        return self.tlog_status()

    def tlog_status(self):
        return {"active": bool(self.tlog_paths), "path": self.tlog_paths[-1] if self.tlog_paths else None,
                "messages": 0, "bytes": 0, "error": self.tlog_error}


class FakePanel:
    def __init__(self, params):
        self.params = dict(params)
        self.live, self.map_status = [], []

    def get_params(self):
        return dict(self.params)

    def set_mapping_live_status(self, text):
        self.live.append(text)

    def set_mapping_plan_status(self, text, ok=True):
        self.map_status.append((text, ok))


class FakeStatus:
    def __init__(self):
        self.msgs = []

    def showMessage(self, m, *a):
        self.msgs.append(m)


_BOUND = ("on_sitl_upload_mission", "_hv_mapping_upload_applies", "_on_hv_mapping_upload",
          "_on_mapping_readback_done", "_write_mapping_params", "_upload_mapping_missions",
          "_on_camera_feedback", "_refresh_mapping_live_status", "_export_hv_mapping",
          "_on_vtol_export_requested", "on_survey_process", "_apply_camera_config_values",
          "_mapping_upload_stale_reason", "_abort_stale_mapping_upload", "_hv_upload_gate",
          "_refresh_release_countdown", "_sitl_broadcast", "on_release_by_schedule", "on_cancel_release",
          "_release_start_one", "_on_release_changed", "_on_mission_uploaded", "survey_area_polygons",
          "_show_mapping_plan_status")


class FakeWin:
    def __init__(self, coord, links, *, panel_over=None, answer=False):
        self._hv_coordinator = coord.hv_coordinator if coord is not None else None
        self._hv_swarm_coordinator = coord          # validate 結果（上傳閘門讀 hv_validation_reasons）
        self._dccpp_result = coord.hv_coordinator.last_result if coord is not None else None
        self._sitl_links = list(links)
        self.target_sysid = None                    # HUD「指令對象」：None ＝ 全部
        self._hv_release_schedule = None
        self._release = None
        self._release_timer_factory = None
        self._release_clock = None
        self.survey_areas, self.corners = [], []
        self._write_pending_sysids = set()
        self._diamond_plans = None
        self.vtol_recon_panel = FakePanel(mapping_params(**(panel_over or {})))
        self.map_widget = object()
        self._status = FakeStatus()
        self.answer = answer
        self.asked = []
        self.readbacks = 0
        self.enabled_updates = 0
        for name in _BOUND:
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))

    def statusBar(self):
        return self._status

    def _target_links(self):
        if self.target_sysid is None:
            return list(self._sitl_links)
        return [l for l in self._sitl_links if int(l.sysid_label) == int(self.target_sysid)]

    def _ensure_release_ticker(self):
        pass

    def _update_write_wp_enabled(self):
        self.enabled_updates += 1

    def _ask_write_mapping_params(self, bad):
        self.asked.append(bad)
        return self.answer

    def _start_mapping_readback(self, jobs):
        """同步版（真的版本是 QThread）：同一支 run_mapping_readback，結果交回同一支處理函式。"""
        self.readbacks += 1
        self._on_mapping_readback_done(mw.run_mapping_readback(jobs))


def _fc_ok(hv, did):
    """一架『設定都對』的飛控：預期的相機參數 + 匯出 .param 的 AIRSPEED_MIN/MAX（包住任務所有速度）
    + LOG_BITMASK 含 bit 12（Camera）+ LOG_DISARMED。"""
    cc = hv.camera_cfg
    fc = dict(hv.expected_camera_params(did, camera_cfg=cc))
    fc.update(mw.mapping_expected_airspeed(hv, did, cc))
    speeds = mw.mapping_airspeed_commands(hv.mission_items(did, camera_cfg=cc))
    assert not speeds or (fc["AIRSPEED_MIN"] <= min(speeds) and fc["AIRSPEED_MAX"] >= max(speeds))
    fc["LOG_BITMASK"] = float(65790 | (1 << 12))
    fc["LOG_DISARMED"] = 0.0
    return fc


def _links_for(hv, **kw):
    return [FakeLink(did, _fc_ok(hv, did), **kw) for did in sorted(hv.id_map)]


# ═══════════════════════════ 純函式 ═══════════════════════════
def test_airspeed_commands_only_counts_airspeed_type():
    items = [{"command": 178, "param1": 0.0, "param2": 21.5}, {"command": 178, "param1": 1.0, "param2": 12.0},
             {"command": 178, "param1": 0.0, "param2": -2.0}, {"command": 178, "param1": 0.0, "param2": 18.0},
             {"command": 16, "param1": 0.0, "param2": 30.0}]
    assert mw.mapping_airspeed_commands(items) == [21.5, 18.0]


def test_readback_rows_use_float32_values_match():
    exp = {"CAM1_DURATION": 0.1, "CAM1_INTRVAL_MIN": 0.6, "CAM1_TYPE": 1.0, "CAM1_TRIGG_DIST": 0.0}
    got = {"CAM1_DURATION": 0.10000000149011612, "CAM1_INTRVAL_MIN": 0.62, "CAM1_TYPE": None,
           "CAM1_TRIGG_DIST": 29.7, "AIRSPEED_MIN": 18.0, "AIRSPEED_MAX": 25.0}
    rows = {r["name"]: r for r in mw.mapping_readback_rows(exp, got, {"CAM1_DURATION": 9}, [18.0, 21.5])}
    assert rows["CAM1_DURATION"]["ok"] is True                 # float32 往返誤差
    assert rows["CAM1_INTRVAL_MIN"]["ok"] is False             # 0.62 ≠ 0.60
    assert rows["CAM1_TYPE"]["ok"] is False                    # 讀不到 = 不符
    assert rows["CAM1_TRIGG_DIST"]["ok"] is False              # 206 寫進 RAM 的殘值
    assert rows["CAM1_TRIGG_DIST"]["writable"] is True
    assert rows["AIRSPEED_MIN"]["ok"] is True and rows["AIRSPEED_MAX"]["ok"] is True
    assert rows["AIRSPEED_MIN"]["writable"] is False and rows["AIRSPEED_MAX"]["writable"] is False


@pytest.mark.parametrize("amin, amax, ok_min, ok_max", [
    (18.0, 21.5, True, True),      # 邊界相等 = ArduPlane 接受（>= 與 <=）
    (18.5, 25.0, False, True),     # 18 < MIN ⇒ 那個 DO_CHANGE_SPEED 被靜默忽略
    (16.0, 21.0, True, False),
    (None, 25.0, False, True),
])
def test_readback_airspeed_range_rows(amin, amax, ok_min, ok_max):
    rows = {r["name"]: r for r in mw.mapping_readback_rows({}, {"AIRSPEED_MIN": amin, "AIRSPEED_MAX": amax},
                                                            {}, [18.0, 21.5, 20.0])}
    assert rows["AIRSPEED_MIN"]["ok"] is ok_min and rows["AIRSPEED_MAX"]["ok"] is ok_max


def test_run_readback_reads_expected_plus_airspeed_and_survives_errors():
    class Boom:
        sysid_label = 3

        def read_params(self, *a, **k):
            raise RuntimeError("link closed")

    ok = FakeLink(1, {"CAM1_TYPE": 1.0, "AIRSPEED_MIN": 10.0, "AIRSPEED_MAX": 30.0, "LOG_BITMASK": 4096.0,
                      "LOG_DISARMED": 1.0})
    res = mw.run_mapping_readback([
        {"sysid": 1, "link": ok, "expected": {"CAM1_TYPE": 1.0}, "speeds": [20.0]},
        {"sysid": 3, "link": Boom(), "expected": {"CAM1_TYPE": 1.0}, "speeds": [20.0]},
    ])
    names, timeout_s, retries = ok.read_calls[0]
    assert names == ["CAM1_TYPE", "AIRSPEED_MIN", "AIRSPEED_MAX", "LOG_BITMASK", "LOG_DISARMED"]
    assert timeout_s > 0 and retries >= 1
    assert res[1]["error"] is None and all(r["ok"] for r in res[1]["rows"])
    assert "link closed" in res[3]["error"] and res[3]["rows"] == []


def test_mismatch_text_lists_name_expected_actual_and_reboot_warning():
    bad = {2: {"error": None, "rows": [
        {"name": "CAM1_TYPE", "expected": 2.0, "actual": 1.0, "ok": False, "writable": True, "ptype": 2},
        {"name": "CAM1_INTRVAL_MIN", "expected": 0.5, "actual": None, "ok": False, "writable": True, "ptype": None},
        {"name": "AIRSPEED_MAX", "expected": "≥ 22.5", "actual": 22.0, "ok": False, "writable": False, "ptype": 9},
        {"name": "CAM_MAX_ROLL", "expected": 15.0, "actual": 15.0, "ok": True, "writable": True, "ptype": 4}]}}
    text = mw.mapping_mismatch_text(bad)
    assert "UAV-2" in text and "CAM1_TYPE" in text and "預期 2" in text and "實際 1" in text
    assert "讀不到" in text
    assert "AIRSPEED_MAX" in text and "不會自動寫入" in text
    assert "CAM_MAX_ROLL" not in text                          # 相符的不列
    assert "重開" in text and "CAM1_TYPE" in mw.mapping_reboot_names(bad)[2]


def test_tlog_path_layout():
    when = _dt.datetime(2026, 10, 14, 9, 5, 7)
    p = mw.mapping_tlog_path("UAV-2", when)
    assert p.name == "UAV-2_090507.tlog"
    assert p.parent.name == "20261014" and p.parent.parent.name == "survey" and p.parent.parent.parent.name == "runs"
    assert p.parent.parent.parent.parent == ROOT


def test_assign_links_matches_sysid_first():
    links = [types.SimpleNamespace(sysid_label=s) for s in (3, 1, 7)]
    pairs, skipped = mw.assign_links_to_uavs(links, [1, 2, 3])
    got = {l.sysid_label: d for l, d in pairs}
    assert got == {3: 3, 1: 1, 7: 2} and skipped == []
    pairs, skipped = mw.assign_links_to_uavs(links, [1])
    assert [(l.sysid_label, d) for l, d in pairs] == [(1, 1)] and sorted(skipped) == [3, 7]


# ═══════════════════════════ 上傳：閘門與讀回 ═══════════════════════════
def test_mapping_upload_uploads_hv_items_after_a_clean_readback(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links_for(hv)
    win = FakeWin(mapped, links)
    assert win._hv_mapping_upload_applies() is True
    win.on_sitl_upload_mission()
    assert win.readbacks == 1 and not win.asked
    for link in links:
        did = link.sysid_label
        want = hv.mission_items(did, camera_cfg=hv.camera_cfg)
        names = link.read_calls[0][0]
        assert set(hv.expected_camera_params(did, camera_cfg=hv.camera_cfg)) | {
            "AIRSPEED_MIN", "AIRSPEED_MAX", "LOG_BITMASK", "LOG_DISARMED"} == set(names)
        assert "CAM1_TRIGG_DIST" in names
        assert link.uploads == [(want, True)]
        assert did in win._write_pending_sysids
        st = win._map_photo_stats[did]
        assert st["expected"] == expected_photo_count(want) and st["taken"] == 0
    info = qmb["calls"]["information"][-1][0][2]
    for link in links:
        n_on = sum(1 for it in link.uploads[0][0] if it["command"] == 206 and it["param1"] > 0)
        assert f"206 開 {n_on} 筆" in info
    assert not any(l.set_calls for l in links)


def test_gate_blocks_upload_without_reading(mapped, qmb, qt_app, monkeypatch):
    hv = mapped.hv_coordinator
    links = _links_for(hv)
    win = FakeWin(mapped, links)
    monkeypatch.setattr(hv, "mapping_gate", lambda: {"uploadable": False, "reasons": ["未覆蓋 1 項：Z2:x"]})
    win.on_sitl_upload_mission()
    assert win.readbacks == 0 and not any(l.uploads or l.read_calls for l in links)
    title, text = qmb["calls"]["critical"][-1][0][1:3]
    assert "Z2:x" in text


def test_mismatch_blocks_and_cancel_does_nothing(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links_for(hv)
    links[0].fc["CAM1_INTRVAL_MIN"] = 1.0
    links[0].fc["CAM1_TRIGG_DIST"] = 29.7
    win = FakeWin(mapped, links, answer=False)
    win.on_sitl_upload_mission()
    assert len(win.asked) == 1
    bad = win.asked[0]
    assert set(bad) == {links[0].sysid_label}
    wrong = {r["name"] for r in bad[links[0].sysid_label]["rows"] if not r["ok"]}
    assert wrong == {"CAM1_INTRVAL_MIN", "CAM1_TRIGG_DIST"}
    assert not any(l.uploads for l in links) and not any(l.set_calls for l in links)


def test_write_then_reread_then_upload(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links_for(hv)
    links[0].fc["CAM1_INTRVAL_MIN"] = 1.0
    links[0].fc["CAM1_TRIGG_DIST"] = 29.7
    win = FakeWin(mapped, links, answer=True)
    win.on_sitl_upload_mission()
    assert win.readbacks == 2                                   # 寫入後重讀重比
    sets = {n: v for n, v, _t in links[0].set_calls}
    assert sets == {"CAM1_INTRVAL_MIN": hv.camera_cfg.interval_min_s, "CAM1_TRIGG_DIST": 0.0}
    assert all(len(l.uploads) == 1 for l in links)
    assert len(links[0].read_calls) == 2


def test_reboot_params_are_written_but_not_auto_uploaded(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links_for(hv)
    links[1].fc["CAM1_TYPE"] = 2.0
    links[1].fc["SERVO10_FUNCTION"] = 0.0
    win = FakeWin(mapped, links, answer=True)
    win.on_sitl_upload_mission()
    assert {n for n, *_ in links[1].set_calls} == {"CAM1_TYPE", "SERVO10_FUNCTION"}
    assert not any(l.uploads for l in links)
    texts = [c[0][2] for c in qmb["calls"]["warning"]]
    assert any("重開" in t and "CAM1_TYPE" in t and "SERVO10_FUNCTION" in t for t in texts)


def test_airspeed_out_of_range_is_not_auto_written(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links_for(hv)
    did = links[0].sysid_label
    speeds = mw.mapping_airspeed_commands(hv.mission_items(did, camera_cfg=hv.camera_cfg))
    assert speeds, "前提：任務要有 DO_CHANGE_SPEED"
    links[0].fc["AIRSPEED_MAX"] = max(speeds) - 0.5
    win = FakeWin(mapped, links, answer=True)
    win.on_sitl_upload_mission()
    assert not links[0].set_calls                               # 飛行包絡參數不自動寫
    assert not any(l.uploads for l in links)
    assert any("AIRSPEED_MAX" in c[0][2] for c in qmb["calls"]["warning"] + qmb["calls"]["critical"])


def test_tlog_starts_before_upload_and_errors_are_shown(mapped, qmb, qt_app):
    hv = mapped.hv_coordinator
    links = _links_for(hv, tlog_error="寫檔失敗：磁碟已滿")
    win = FakeWin(mapped, links, panel_over={"hv_map_tlog": True})
    win.on_sitl_upload_mission()
    for link in links:
        assert link.order == ["tlog", "upload"]
        p = Path(link.tlog_paths[0])
        assert p.name.startswith(f"UAV-{link.sysid_label}_") and p.suffix == ".tlog"
        assert p.parent.parent == ROOT / "runs" / "survey"
    assert any("磁碟已滿" in t for t in win.vtol_recon_panel.live + win._status.msgs
               + [c[0][2] for c in qmb["calls"]["warning"]])


def test_no_tlog_unless_ticked(mapped, qmb, qt_app):
    links = _links_for(mapped.hv_coordinator)
    win = FakeWin(mapped, links, panel_over={"hv_map_tlog": False})
    win.on_sitl_upload_mission()
    assert not any(l.tlog_paths for l in links) and all(l.uploads for l in links)


def test_non_mapping_result_does_not_take_the_mapping_path(qt_app):
    win = FakeWin(None, [])
    assert win._hv_mapping_upload_applies() is False
    hv = types.SimpleNamespace(mapping_mode=None, camera_cfg=None, last_result={"x": 1})
    win._hv_coordinator, win._dccpp_result = hv, hv.last_result
    assert win._hv_mapping_upload_applies() is False
    hv.mapping_mode = object()
    assert win._hv_mapping_upload_applies() is True
    win._dccpp_result = {"other": 1}                            # 結果不是 hv 的 ⇒ 不走
    assert win._hv_mapping_upload_applies() is False


# ═══════════════════════════ 即時相機回授 ═══════════════════════════
def test_feedback_counts_new_photos_not_signals(qt_app):
    win = FakeWin(None, [])
    win._map_photo_stats = {1: {"expected": 120, "taken": 0, "fb_missing": 0, "resets": 0, "uav": 1}}
    win._on_camera_feedback(1, {"new_photos": 3, "gap": (5, 6), "reset": False})
    win._on_camera_feedback(1, {"new_photos": 1, "gap": None, "reset": False})
    win._on_camera_feedback(9, {"new_photos": 50, "gap": None, "reset": False})   # 沒上傳過的機 ⇒ 不算
    st = win._map_photo_stats[1]
    assert st["taken"] == 4 and st["fb_missing"] == 2
    text = win.vtol_recon_panel.live[-1]
    assert "UAV-1" in text and "已拍 4 / 預估 120" in text
    assert "回授訊息漏收 2 則" in text and "不是漏拍" in text
    assert 9 not in win._map_photo_stats


def test_feedback_gap_wraps_and_reset_is_reported(qt_app):
    win = FakeWin(None, [])
    win._map_photo_stats = {2: {"expected": 10, "taken": 0, "fb_missing": 0, "resets": 0, "uav": 2}}
    win._on_camera_feedback(2, {"new_photos": 4, "gap": (65534, 1), "reset": False})
    win._on_camera_feedback(2, {"new_photos": 7, "gap": None, "reset": True})
    st = win._map_photo_stats[2]
    assert st["fb_missing"] == 4 and st["resets"] == 1 and st["taken"] == 11
    assert "重開機" in win.vtol_recon_panel.live[-1]


def test_feedback_without_mapping_upload_is_silent(qt_app):
    win = FakeWin(None, [])
    win._on_camera_feedback(1, {"new_photos": 3, "gap": None, "reset": False})
    assert win.vtol_recon_panel.live == []


def test_camera_feedback_is_connected_in_on_sitl_connect():
    tree = ast.parse(SRC)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "on_sitl_connect")
    calls = [c for c in ast.walk(fn) if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
             and c.func.attr == "connect" and isinstance(c.func.value, ast.Attribute)
             and c.func.value.attr == "camera_feedback"]
    assert len(calls) == 1


def test_tlog_is_stopped_on_every_disconnect_path():
    tree = ast.parse(SRC)
    for name in ("on_sitl_disconnect", "_on_sitl_link_disconnected", "_on_hil_disconnect_all"):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        called = {c.func.id for c in ast.walk(fn) if isinstance(c, ast.Call) and isinstance(c.func, ast.Name)}
        assert "_stop_link_tlog" in called, name


def test_stop_link_tlog_helper():
    class L:
        sysid_label = 4

        def __init__(self):
            self.stopped = 0

        def stop_tlog(self):
            self.stopped += 1
            return {"active": False, "path": "x.tlog", "messages": 3, "bytes": 9, "error": "boom"}

    link = L()
    assert mw._stop_link_tlog(link)["error"] == "boom" and link.stopped == 1
    assert mw._stop_link_tlog(types.SimpleNamespace(sysid_label=1)) is None    # 沒有 tlog 的假鏈路


# ═══════════════════════════ 匯出 ═══════════════════════════
def test_export_in_mapping_mode_uses_export_all_with_camera_cfg(mapped, qmb, qt_app, tmp_path, monkeypatch):
    hv = mapped.hv_coordinator
    monkeypatch.setattr(_QtW.QFileDialog, "getExistingDirectory", staticmethod(lambda *a, **k: str(tmp_path)))
    win = FakeWin(mapped, [])
    win._on_vtol_export_requested({"transition_alt": 40.0})
    cc = hv.camera_cfg
    for did, uid in hv.id_map.items():
        wp = tmp_path / f"{uid}.waypoints"
        if not hv.last_plan.plans[uid].waypoints:
            continue
        items = read_qgc_wpl(wp)
        assert [it.command for it in items] == [it["command"] for it in hv.mission_items(did, camera_cfg=cc)]
        par = read_param_file(tmp_path / f"{uid}.param")
        for k, v in hv.expected_camera_params(did, camera_cfg=cc).items():
            assert par[k] == pytest.approx(v, abs=1e-6), k
    assert (tmp_path / "mission_summary.json").exists()
    text = qmb["calls"]["information"][-1][0][2]
    assert "CAM1_INTRVAL_MIN" in text and "引入段" in text


def test_export_warns_when_the_gate_blocks(mapped, qmb, qt_app, tmp_path, monkeypatch):
    hv = mapped.hv_coordinator
    monkeypatch.setattr(_QtW.QFileDialog, "getExistingDirectory", staticmethod(lambda *a, **k: str(tmp_path)))
    monkeypatch.setattr(hv, "mapping_gate", lambda: {"uploadable": False, "reasons": ["未覆蓋 1 項：Z9:y"]})
    win = FakeWin(mapped, [])
    win._on_vtol_export_requested({"transition_alt": 40.0})
    texts = [c[0][2] for c in qmb["calls"]["warning"] + qmb["calls"]["information"]]
    assert any("Z9:y" in t and "不允許上傳" in t for t in texts)


# ═══════════════════════════ 選單「航測處理…」 ═══════════════════════════
def test_survey_process_menu_entry_exists():
    tree = ast.parse(SRC)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "create_menus")
    texts = [c.args[0].value for c in ast.walk(fn) if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
             and c.func.attr == "addAction" and c.args and isinstance(c.args[0], ast.Constant)]
    assert "航測處理…" in texts
    assert "self.on_survey_process" in ast.unparse(fn)


def test_survey_process_opens_the_dialog_lazily(qt_app, monkeypatch, qmb):
    made = []

    class Dlg:
        def __init__(self, parent=None, map_widget=None, polygons=None):
            made.append((parent, map_widget))
            self.polygons = polygons
            self.shown = False

        def show(self):
            self.shown = True

    fake = types.ModuleType("ui.dialogs.survey_process_dialog")
    fake.SurveyProcessDialog = Dlg
    monkeypatch.setitem(sys.modules, "ui.dialogs.survey_process_dialog", fake)
    win = FakeWin(None, [])
    win.on_survey_process()
    assert made == [(win, win.map_widget)]
    assert win._survey_process_dialog.shown is True


def test_survey_process_missing_module_explains(qt_app, monkeypatch, qmb):
    monkeypatch.setitem(sys.modules, "ui.dialogs.survey_process_dialog", None)     # import ⇒ ImportError
    win = FakeWin(None, [])
    win.on_survey_process()
    assert qmb["calls"]["warning"] or qmb["calls"]["information"]


# ═══════════════════════════ 相機配置 → 航測建圖欄位 ═══════════════════════════
class _ApplyPanel:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.calls = []

    def mapping_enabled(self):
        return self.enabled

    def apply_camera_values(self, values, *, enable_mapping):
        self.calls.append((dict(values), enable_mapping))
        return {"焦距 mm": values["focal_mm"], "建圖模式": enable_mapping}


VALUES = {"camera_name": "DJI Mavic 3", "focal_mm": 24.0, "pixel_um": 3.2765, "width_px": 5280,
          "height_px": 3956, "front_overlap": 0.80, "side_overlap": 0.60,
          "not_applied": {"飛行高度": "50.0 m", "飛行速度": "5.0 m/s"}}


@pytest.mark.parametrize("answer, called, enable", [("Yes", True, True), ("No", True, False),
                                                    ("Cancel", False, None)])
def test_camera_config_apply_asks_before_turning_mapping_on(qt_app, qmb, answer, called, enable):
    QMB = qmb["QMB"]
    qmb["ret"]["value"] = getattr(QMB.StandardButton, answer)
    win = FakeWin(None, [])
    win.vtol_recon_panel = _ApplyPanel(enabled=False)
    win._apply_camera_config_values(dict(VALUES))
    assert len(qmb["calls"]["question"]) == 1
    if called:
        assert win.vtol_recon_panel.calls == [(VALUES, enable)]
        text = qmb["calls"]["information"][-1][0][2]
        assert "飛行高度" in text and "沒有套用" in text
    else:
        assert win.vtol_recon_panel.calls == []


def test_camera_config_apply_when_mapping_already_on_does_not_ask(qt_app, qmb):
    win = FakeWin(None, [])
    win.vtol_recon_panel = _ApplyPanel(enabled=True)
    win._apply_camera_config_values(dict(VALUES))
    assert qmb["calls"]["question"] == []
    assert win.vtol_recon_panel.calls == [(VALUES, True)]


def test_camera_config_apply_out_of_range_is_refused(qt_app, qmb):
    class P(_ApplyPanel):
        def apply_camera_values(self, values, *, enable_mapping):
            raise ValueError("焦距 600 mm 超出面板範圍")

    win = FakeWin(None, [])
    win.vtol_recon_panel = P(enabled=True)
    win._apply_camera_config_values(dict(VALUES))
    assert "超出" in qmb["calls"]["critical"][-1][0][2]
