"""GUI 通用 VTOL 在風中起飛不了（2026-10-22）：角速率增益檔與風速上限警告的值層級鎖。

背景（實飛量到的，docs/修正_2026-10-22_GUI_VTOL有風起飛不了.md）
------------------------------------------------------------------
GUI 的 SITLLauncher().start(vehicle='VTOL', sim_wind=(5, 270)) 開的 VTOL，AUTO 垂直起飛一直報
「Potential VTOL Thrust Loss」、15 分鐘高度 0.1–0.5 m。根因是 vtol_default.parm 的 Alti 真機角速率增益
（俯仰 P 0.3／D 0.01）在 100 Hz 主迴圈下讓 SITL 通用 quadplane 模型俯仰自激振盪、馬達打滿，
推力被姿態吃掉；側風把剩下的餘裕吃光。2、3 m/s 也飛不起來。修法：GUI 這條路在 vtol_default.parm
之後疊 vtol_rate_gains.parm（P 0.1／I 0.1／D 0.002），**不改** vtol_default.parm（hv_sitl_run 200 Hz 也讀它）。

鎖住：
  1. 增益檔內容＝那六個值、名字在韌體參數表、每行 ≤ 98 bytes、寫完讀回（讀回不符不啟動）；
  2. ``--defaults`` 裡緊接 vtol_default.parm 之後、強制檔仍最後 ⇒ 合併後的生效值是新值；
  3. 只對 100 Hz 驗過 ⇒ 迴圈速率與增益一起鎖；迴圈速率被改 ⇒ 啟動時大聲警告；
  4. vtol_default.parm 本身沒動（HV 證據鏈）；
  5. 模擬風超過實測驗證上限 12 m/s ⇒ launcher 記警告、對話框琥珀色警告（不擋，取消勾選就收起）。
不起 SITL、不開 COM 埠、不碰 WSL —— Popen 在來源換成替身。
"""
from __future__ import annotations

import json
import os
import re
import types
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine 再建 QApplication
    from PyQt6.QtWidgets import QApplication
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

import mission.sitl_launcher as sl
from mission.sitl_launcher import SITLLauncher

ROOT = Path(__file__).resolve().parent.parent
_SNAPSHOT = ROOT / "sitl" / "default_params" / "firmware_param_names.json"
_needs_qt = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

_EXPECT = {"Q_A_RAT_RLL_P": 0.1, "Q_A_RAT_RLL_I": 0.1, "Q_A_RAT_RLL_D": 0.002,
           "Q_A_RAT_PIT_P": 0.1, "Q_A_RAT_PIT_I": 0.1, "Q_A_RAT_PIT_D": 0.002}


# ══════════════════════════════════════════════════════════════════════
#  1. 增益檔內容
# ══════════════════════════════════════════════════════════════════════

def test_gains_file_carries_exactly_the_verified_values():
    assert sl.parse_parm_values(sl.vtol_rate_gains_parm_text()) == _EXPECT


def test_gains_names_exist_in_the_vtol_firmware():
    """名字錯了韌體會**靜默略過**（SYSID_THISMAV 那條老陷阱）⇒ 對照韌體參數表快照。"""
    fw = set(json.loads(_SNAPSHOT.read_text(encoding="utf-8"))["vtol"])
    assert len(fw) > 1000
    assert [n for n in _EXPECT if n not in fw] == []


def test_every_physical_line_fits_the_firmware_line_buffer():
    lines = sl.vtol_rate_gains_parm_text().splitlines()
    assert max(len(ln.encode("utf-8")) for ln in lines) <= 98
    for ln in lines:
        if ln.strip() and not ln.lstrip().startswith("#"):
            assert re.fullmatch(r"[A-Z][A-Z0-9_]* -?\d+(\.\d+)?", ln), f"格式不嚴格：{ln!r}"


def test_write_reads_back_and_refuses_a_file_that_does_not_match(tmp_path, monkeypatch):
    p = sl.write_vtol_rate_gains_parm(tmp_path)
    assert p.name == "vtol_rate_gains.parm" and p.parent == tmp_path
    assert sl.parse_parm_values(p.read_text(encoding="utf-8")) == _EXPECT
    monkeypatch.setattr(sl, "vtol_rate_gains_parm_text",
                        lambda: "Q_A_RAT_PIT_P 0.3\nQ_A_RAT_PIT_D 0.01\n")
    with pytest.raises(RuntimeError, match="讀回不符.*Q_A_RAT_PIT_P 期望 0.1"):
        sl.write_vtol_rate_gains_parm(tmp_path)


def test_vtol_default_parm_keeps_the_alti_values(tmp_path):
    """HV 證據鏈（scripts/hv_sitl_run.py，200 Hz）讀的是 vtol_default.parm ⇒ 這次**刻意不改**它。
    同時是對照組：增益檔真的有改到東西（新值 ≠ vtol_default 的值）。"""
    v = sl.parse_parm_values(re.sub(r"#.*", "", sl.generate_vtol_params(tmp_path / "v.parm").read_text(encoding="utf-8")))
    assert (v["Q_A_RAT_PIT_P"], v["Q_A_RAT_PIT_I"], v["Q_A_RAT_PIT_D"]) == (0.3, 0.3, 0.01)
    assert (v["Q_A_RAT_RLL_P"], v["Q_A_RAT_RLL_I"], v["Q_A_RAT_RLL_D"]) == (0.2, 0.2, 0.002)
    assert [k for k in _EXPECT if v[k] != _EXPECT[k]] == ["Q_A_RAT_RLL_P", "Q_A_RAT_RLL_I", "Q_A_RAT_PIT_P",
                                                          "Q_A_RAT_PIT_I", "Q_A_RAT_PIT_D"]


def test_gains_are_locked_to_the_loop_rate_they_were_verified_at():
    """只在 SCHED_LOOP_RATE 100 實飛驗過；改迴圈速率要重測這組（反之亦然）。"""
    assert sl.VTOL_RATE_GAINS_LOOP_HZ == 100
    assert sl._SCHED_LOOP_RATE["VTOL"] == sl.VTOL_RATE_GAINS_LOOP_HZ


# ══════════════════════════════════════════════════════════════════════
#  2. --defaults 順序
# ══════════════════════════════════════════════════════════════════════

def test_defaults_order_puts_gains_right_after_vtol_default():
    assert sl.build_defaults_list("VTOL", "b", "v", "i", "e", wind="w", gains="g") == ["b", "v", "g", "i", "w", "e"]
    assert sl.build_defaults_list("VTOL", "b", "v", "i", "e", gains="g") == ["b", "v", "g", "i", "e"]
    # 不帶增益 ⇒ 與舊版逐項相同；非 VTOL 不收增益檔
    assert sl.build_defaults_list("VTOL", "b", "v", "i", "e") == ["b", "v", "i", "e"]
    assert sl.build_defaults_list("PLANE", "b", "v", "i", "e", gains="g") == ["b", "i"]
    assert sl.build_defaults_list("COPTER", "b", None, "i", None, wind="w", gains="g") == ["b", "i", "w"]


class _Log:
    def __init__(self):
        self.lines = {"info": [], "warning": [], "error": []}

    def __getattr__(self, name):
        return lambda msg, *a, **k: self.lines.setdefault(name, []).append(str(msg))


@pytest.fixture
def fake_sitl(tmp_path, monkeypatch):
    """假 sitl/ 目錄＋Popen 替身（在來源擋 I/O：只准啟動替身目錄裡的空 binary，而且不真的啟動）。"""
    root = tmp_path / "sitl"
    (root / "default_params").mkdir(parents=True)
    for exe in ("ArduPlane.exe", "ArduCopter.exe"):
        (root / exe).write_bytes(b"")
    (root / "default_params" / "plane.parm").write_text("ARMING_REQUIRE 0\n", encoding="utf-8")
    monkeypatch.setattr(sl, "_SITL_DIR", root)
    cmds = []

    class _Proc:
        pid, returncode = 4242, None

        def poll(self):
            return None

    def _popen(cmd, cwd=None, **_kw):
        assert Path(cmd[0]).resolve().parent == root.resolve(), f"想啟動替身以外的程式：{cmd[0]}"
        cmds.append((list(cmd), Path(cwd)))
        return _Proc()

    log = _Log()
    monkeypatch.setattr(sl.subprocess, "Popen", _popen)
    monkeypatch.setattr(sl.time, "sleep", lambda _s: None)
    monkeypatch.setattr(sl, "_tcp_port_listener_pid", lambda _port: None)
    monkeypatch.setattr(sl, "logger", log)
    return root, cmds, log


def _defaults(cmd):
    return [Path(p) for p in cmd[cmd.index("--defaults") + 1].split(",")]


def _merged(paths):
    """依 --defaults 順序合併（後面蓋前面）＝ 韌體開機後的值（eeprom.bin 已刪）。"""
    out = {}
    for p in paths:
        out.update(sl.parse_parm_values(re.sub(r"#.*", "", p.read_text(encoding="utf-8-sig"))))
    return out


def test_start_vtol_layers_the_gains_over_vtol_default(fake_sitl):
    root, cmds, _log = fake_sitl
    SITLLauncher().start(vehicle="VTOL", lat=23.45, lon=120.29, alt=30.0, instance=0, sysid=1,
                         sim_wind=(5.0, 270.0))
    (cmd, cwd), = cmds
    d = _defaults(cmd)
    assert [p.name for p in d] == ["plane.parm", "vtol_default.parm", "vtol_rate_gains.parm", "identity.parm",
                                   "sim_wind.parm", "vtol_enforced.parm"]
    assert d[2].parent == cwd                                   # 每個實例自己的工作目錄
    eff = _merged(d)
    assert {k: eff[k] for k in _EXPECT} == _EXPECT              # 生效值是新增益，不是 Alti 的
    assert eff["Q_A_RAT_YAW_P"] == 2.0                          # 偏航沒動
    assert eff["ARMING_REQUIRE"] == 1.0 and eff["SIM_WIND_SPD"] == 5.0


def test_start_without_gains_would_fly_the_alti_values(fake_sitl, monkeypatch):
    """對照組：拿掉增益檔 ⇒ 合併後是 Alti 的 0.3／0.01（上一條不是恆真）。"""
    _root, cmds, _log = fake_sitl
    real = sl.build_defaults_list
    monkeypatch.setattr(sl, "build_defaults_list", lambda *a, **k: real(*a, **{**k, "gains": None}))
    SITLLauncher().start(vehicle="VTOL", instance=0, sysid=1)
    eff = _merged(_defaults(cmds[0][0]))
    assert (eff["Q_A_RAT_PIT_P"], eff["Q_A_RAT_PIT_D"]) == (0.3, 0.01)


def test_start_refuses_to_launch_when_the_gains_do_not_read_back(fake_sitl, monkeypatch):
    _root, cmds, _log = fake_sitl
    monkeypatch.setattr(sl, "vtol_rate_gains_parm_text", lambda: "Q_A_RAT_PIT_P 0.3\n")
    with pytest.raises(RuntimeError, match="角速率增益"):
        SITLLauncher().start(vehicle="VTOL", instance=0, sysid=1)
    assert cmds == []                                           # 沒有啟動


def test_start_plane_has_no_gains_and_drops_a_stale_file(fake_sitl):
    root, cmds, _log = fake_sitl
    stale = root / "plane" / "i1" / "vtol_rate_gains.parm"
    stale.parent.mkdir(parents=True)
    stale.write_text("Q_A_RAT_PIT_P 0.1\n", encoding="utf-8")
    SITLLauncher().start(vehicle="PLANE", instance=1, sysid=2)
    assert [p.name for p in _defaults(cmds[0][0])] == ["plane.parm", "identity.parm"]
    assert not stale.exists()


def test_start_multi_gives_every_vtol_its_own_gains_file(fake_sitl):
    _root, cmds, _log = fake_sitl
    SITLLauncher().start_multi("VTOL", 3, lat=23.45, lon=120.29, sim_wind=(5.0, 270.0))
    assert len(cmds) == 3
    gains = [_defaults(c)[2] for c, _cwd in cmds]
    assert len({p.parent for p in gains}) == 3 and all(p.name == "vtol_rate_gains.parm" for p in gains)
    for cmd, _cwd in cmds:
        assert {k: _merged(_defaults(cmd))[k] for k in _EXPECT} == _EXPECT


def test_start_warns_loudly_if_the_loop_rate_drifts_from_the_verified_one(fake_sitl, monkeypatch):
    _root, _cmds, log = fake_sitl
    SITLLauncher().start(vehicle="VTOL", instance=0, sysid=1)
    assert not [w for w in log.lines["warning"] if "主迴圈" in w]
    monkeypatch.setitem(sl._SCHED_LOOP_RATE, "VTOL", 200)
    SITLLauncher().start(vehicle="VTOL", instance=1, sysid=2)
    assert [w for w in log.lines["warning"] if "主迴圈 200 Hz" in w and "要重測" in w]


# ══════════════════════════════════════════════════════════════════════
#  3. 風速上限（實測驗證 12 m/s）
# ══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("spd", [0.0, 5.0, 8.0, 12.0])
def test_no_warning_within_the_verified_envelope(spd):
    assert sl.sim_wind_limit_warning("VTOL", spd) is None


def test_warning_tiers_above_the_verified_envelope():
    w13 = sl.sim_wind_limit_warning("vtol", 13.0)
    assert w13.startswith("⚠ ") and "13 m/s" in w13 and "驗證上限 12 m/s" in w13
    assert sl.sim_wind_limit_warning("VTOL", 15.0).startswith("⚠ ")
    w16 = sl.sim_wind_limit_warning("VTOL", 16.0)
    assert w16.startswith("⚠⚠") and "可能飛不起來" in w16


@pytest.mark.parametrize("vehicle", ["PLANE", "COPTER", "", None])
def test_warning_only_speaks_for_the_vtol_it_was_measured_on(vehicle):
    assert sl.sim_wind_limit_warning(vehicle, 30.0) is None
    assert sl.sim_wind_limit_warning("VTOL", None) is None


def test_start_logs_the_wind_warning(fake_sitl):
    _root, _cmds, log = fake_sitl
    SITLLauncher().start(vehicle="VTOL", instance=0, sysid=1, sim_wind=(5.0, 270.0))
    assert not [w for w in log.lines["warning"] if "驗證上限" in w]
    SITLLauncher().start(vehicle="VTOL", instance=1, sysid=2, sim_wind=(14.0, 270.0))
    assert [w for w in log.lines["warning"] if "模擬風 14 m/s" in w and "驗證上限 12 m/s" in w]


@pytest.fixture(scope="module")
def qt_app():
    if not _QT_OK:
        pytest.skip("Qt 不可用")
    return QApplication.instance() or QApplication([])


@pytest.fixture
def dialog_mod(monkeypatch):
    """對話框不讀使用者的 settings.json、不碰網路。"""
    import ui.dialogs.sitl_launch_dialog as dmod
    saved = types.SimpleNamespace(enable_fanout=False, auto_firewall=False, home_preset="", instances=[])
    monkeypatch.setattr(dmod, "get_settings", lambda: types.SimpleNamespace(sitl_launch=saved))
    monkeypatch.setattr(dmod, "_get_lan_ip", lambda: "127.0.0.1")
    return dmod


def _pw(spd):
    return {"speed_mps": spd, "dir_from_deg": 270.0, "source": "目前的 HARVEST 計畫（規劃器生效值）"}


@_needs_qt
def test_dialog_warns_when_the_plan_wind_exceeds_the_vtol_envelope(qt_app, dialog_mod):
    dlg = dialog_mod.SITLLaunchDialog("VTOL", 2, plan_wind=_pw(16.0))
    assert not dlg.sim_wind_warn.isHidden()
    assert dlg.sim_wind_warn.property("role") == "warning"
    assert "16 m/s" in dlg.sim_wind_warn.text() and "驗證上限 12 m/s" in dlg.sim_wind_warn.text()
    assert dlg.collect_result()["sim_wind"] == (16.0, 270.0)   # 不擋：照樣帶
    dlg.chk_sim_wind.setChecked(False)                          # 不帶風 ⇒ 警告收起
    assert dlg.sim_wind_warn.isHidden()
    dlg.chk_sim_wind.setChecked(True)
    assert not dlg.sim_wind_warn.isHidden()
    dlg.deleteLater()


@_needs_qt
@pytest.mark.parametrize("vehicle,spd", [("VTOL", 5.0), ("VTOL", 12.0), ("PLANE", 20.0), ("COPTER", 20.0)])
def test_dialog_is_quiet_inside_the_envelope_or_for_other_vehicles(qt_app, dialog_mod, vehicle, spd):
    dlg = dialog_mod.SITLLaunchDialog(vehicle, 1, plan_wind=_pw(spd))
    assert dlg.sim_wind_warn.isHidden() and dlg.sim_wind_warn.text() == ""
    assert ("實測驗證到 12 m/s" in dlg.sim_wind_hint.text()) == (vehicle == "VTOL")
    dlg.deleteLater()


@_needs_qt
def test_dialog_without_a_wind_has_no_warning(qt_app, dialog_mod):
    dlg = dialog_mod.SITLLaunchDialog("VTOL", 1)
    assert dlg.sim_wind_warn.isHidden()
    dlg.deleteLater()
