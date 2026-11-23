"""規劃風在 GUI 與 SITL 之間的缺口（2026-10-20；docs/評估_2026-10-20_風場是否真的被考慮進去.md §5–6）。

鎖住三件事（值層級；不起 SITL、不開 COM 埠、不碰 WSL —— Popen 在來源就換成替身）：

1. 規劃完成的結果摘要／狀態列寫出「規劃風 X m/s 來自 Y°」（一般模式原本沒有任何一行）；值取**規劃器生效的風**
   （``hv.last_ctx.inputs.wind``，真規劃驗過），0 m/s 寫「無風規劃」；DCCPP 計畫寫「不考慮風」；面板 0 m/s 顯示「無風規劃」。
2. GUI 啟動 SITL：對話框「帶入規劃風」預設勾選、寫明值與方向 → ``start_multi(sim_wind=…)`` → 每台 ``sim_wind.parm``
   （SIM_WIND_SPD／DIR＝來向、T=1、TURB=0、DIR_Z=0）排在 ``--defaults`` 的 identity 之後、強制檔之前，寫完讀回比對。
3. ``scripts/hv_sitl_run.py``：規劃風 ≠ 模擬風 ⇒ 讀回之後大聲警告（不擋）；一致 ⇒ 不警告；
   兩者寫進 scenario.json／wind_readback.json／報告開頭。
"""
from __future__ import annotations

import dataclasses
import json
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
    from PyQt6.QtWidgets import QApplication, QDialog
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

import mission.sitl_launcher as sl
from mission.sitl_launcher import SITLLauncher

pytest.importorskip("hv_dccpp")
from hv_dccpp.mission import WindLayer  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
_SNAPSHOT = ROOT / "sitl" / "default_params" / "firmware_param_names.json"
_needs_qt = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")


def _layer(spd, dfrom):
    return WindLayer(0.0, 3000.0, spd, dfrom, gust_mps=spd * 1.3, z_ref="agl")


def _hv(result, *layers):
    return types.SimpleNamespace(last_result=result, mapping_mode=None,
                                 last_ctx=types.SimpleNamespace(inputs=types.SimpleNamespace(wind=tuple(layers))))


@pytest.fixture(scope="module")
def qt_app():
    if not _QT_OK:
        pytest.skip("Qt 不可用")
    return QApplication.instance() or QApplication([])


@pytest.fixture
def qmb(monkeypatch):
    """所有 QMessageBox 靜態方法換成記錄器（離屏下模態對話框會卡住整個測試）。"""
    QMB = _QtW.QMessageBox
    calls = {"information": [], "warning": [], "critical": [], "question": []}

    def mk(name):
        def _f(*a, **k):
            calls[name].append((a, k))
            return QMB.StandardButton.Ok
        return staticmethod(_f)

    for name in calls:
        monkeypatch.setattr(QMB, name, mk(name))
    return calls


# ══════════════════════════════════════════════════════════════════════
#  2. 模擬風參數檔（mission/sitl_launcher.py）
# ══════════════════════════════════════════════════════════════════════

def test_sim_wind_text_has_the_same_convention_as_hv_sitl_run():
    vals = sl.parse_parm_values(sl.sim_wind_parm_text(5.0, 270.0))
    assert vals == {"SIM_WIND_SPD": 5.0, "SIM_WIND_DIR": 270.0, "SIM_WIND_T": 1.0,
                    "SIM_WIND_TURB": 0.0, "SIM_WIND_DIR_Z": 0.0}
    # 來向正規化到 [0, 360)；小數保留 3 位
    v2 = sl.parse_parm_values(sl.sim_wind_parm_text(4.25, 359.9996))
    assert (v2["SIM_WIND_SPD"], v2["SIM_WIND_DIR"]) == (4.25, 0.0)
    v3 = sl.parse_parm_values(sl.sim_wind_parm_text(3.0, -90.0))
    assert v3["SIM_WIND_DIR"] == 270.0


@pytest.mark.parametrize("bad", [(-1.0, 0.0), (float("nan"), 0.0), (1.0, float("inf"))])
def test_sim_wind_rejects_nonsense(bad):
    with pytest.raises(ValueError):
        sl.sim_wind_params(*bad)


def test_sim_wind_names_exist_in_every_firmware():
    """名字錯了韌體會**靜默略過**（SYSID_THISMAV 那條老陷阱）⇒ 對照韌體參數表快照。"""
    fw = {k: set(v) for k, v in json.loads(_SNAPSHOT.read_text(encoding="utf-8")).items() if not k.startswith("_")}
    names = list(sl.parse_parm_values(sl.sim_wind_parm_text(5.0, 270.0)))
    assert len(names) == 5
    for kind in ("plane", "vtol", "copter"):
        assert len(fw[kind]) > 1000
        assert [n for n in names if n not in fw[kind]] == [], kind


def test_every_physical_line_fits_the_firmware_line_buffer():
    """韌體用 100-byte 緩衝讀 --defaults（見 vtol_enforced_parm_text）⇒ 每行 ≤ 98 bytes。"""
    for spd, d in ((0.0, 0.0), (12.345, 123.456), (25.0, 359.0)):
        for line in sl.sim_wind_parm_text(spd, d).splitlines():
            assert len(line.encode("utf-8")) <= 98, line


def test_write_reads_back_and_refuses_a_file_that_does_not_match(tmp_path, monkeypatch):
    p = sl.write_sim_wind_parm(tmp_path, 6.0, 45.0)
    assert p.name == "sim_wind.parm"
    assert sl.parse_parm_values(p.read_text(encoding="utf-8"))["SIM_WIND_SPD"] == 6.0
    # 讀回這道閘門要響得起來：產生的內容跟要求不同 ⇒ RuntimeError
    monkeypatch.setattr(sl, "sim_wind_parm_text", lambda s, d: "SIM_WIND_SPD 0\nSIM_WIND_DIR 45\n")
    with pytest.raises(RuntimeError, match="讀回不符.*SIM_WIND_SPD 期望 6"):
        sl.write_sim_wind_parm(tmp_path, 6.0, 45.0)


def test_parse_refuses_unparseable_lines():
    with pytest.raises(ValueError, match="第 2 行"):
        sl.parse_parm_values("SIM_WIND_SPD 5\nSIM_WIND_DIR two-seventy\n")


def test_defaults_order_puts_wind_after_identity_and_before_enforced():
    assert sl.build_defaults_list("VTOL", "b", "v", "i", "e", wind="w") == ["b", "v", "i", "w", "e"]
    assert sl.build_defaults_list("PLANE", "b", "v", "i", "e", wind="w") == ["b", "i", "w"]
    assert sl.build_defaults_list("COPTER", "b", None, "i", None, wind="w") == ["b", "i", "w"]
    # 不帶風 ⇒ 與舊版逐項相同
    assert sl.build_defaults_list("VTOL", "b", "v", "i", "e") == ["b", "v", "i", "e"]


@pytest.fixture
def fake_sitl(tmp_path, monkeypatch):
    """假 sitl/ 目錄＋Popen 替身（在來源擋 I/O：只准啟動替身目錄裡的空 binary，而且不真的啟動）。"""
    root = tmp_path / "sitl"
    (root / "default_params").mkdir(parents=True)
    for exe in ("ArduPlane.exe", "ArduCopter.exe"):
        (root / exe).write_bytes(b"")
    # base 檔故意帶一個舊的風：模擬風檔排在後面就要蓋過它（後面蓋前面）
    (root / "default_params" / "plane.parm").write_text("ARMING_REQUIRE 0\nSIM_WIND_SPD 9\n", encoding="utf-8")
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

    monkeypatch.setattr(sl.subprocess, "Popen", _popen)
    monkeypatch.setattr(sl.time, "sleep", lambda _s: None)
    monkeypatch.setattr(sl, "_tcp_port_listener_pid", lambda _port: None)
    return root, cmds


def _defaults(cmd):
    return [Path(p) for p in cmd[cmd.index("--defaults") + 1].split(",")]


def _merged(paths):
    """依 --defaults 順序合併（後面蓋前面）＝ 韌體開機後的值（eeprom.bin 已刪）。"""
    out = {}
    for p in paths:
        out.update(sl.parse_parm_values(p.read_text(encoding="utf-8-sig")))
    return out


def test_start_vtol_with_wind_writes_and_orders_the_file(fake_sitl):
    root, cmds = fake_sitl
    SITLLauncher().start(vehicle="VTOL", lat=23.95, lon=120.9, alt=0.0, instance=0, sysid=1,
                         sim_wind=(5.0, 270.0))
    (cmd, cwd), = cmds
    d = _defaults(cmd)
    # vtol_rate_gains.parm（2026-10-22，tests/test_vtol_sitl_rate_gains.py）緊接 vtol_default 之後
    assert [p.name for p in d] == ["plane.parm", "vtol_default.parm", "vtol_rate_gains.parm", "identity.parm",
                                   "sim_wind.parm", "vtol_enforced.parm"]
    assert d[4].parent == cwd                                   # 每個實例自己的工作目錄
    eff = _merged(d)
    assert (eff["SIM_WIND_SPD"], eff["SIM_WIND_DIR"], eff["SIM_WIND_T"], eff["SIM_WIND_TURB"]) == (5.0, 270.0, 1.0, 0.0)
    assert eff["ARMING_REQUIRE"] == 1.0                         # 強制檔仍然最後


def test_start_plane_with_wind(fake_sitl):
    _root, cmds = fake_sitl
    SITLLauncher().start(vehicle="PLANE", instance=1, sysid=2, sim_wind=(3.0, 0.0))
    d = _defaults(cmds[0][0])
    assert [p.name for p in d] == ["plane.parm", "identity.parm", "sim_wind.parm"]
    assert (_merged(d)["SIM_WIND_SPD"], _merged(d)["SIM_WIND_DIR"]) == (3.0, 0.0)


def test_start_without_wind_is_unchanged_and_drops_a_stale_file(fake_sitl):
    root, cmds = fake_sitl
    stale = root / "plane" / "i0" / "sim_wind.parm"
    stale.parent.mkdir(parents=True)
    stale.write_text("SIM_WIND_SPD 7\n", encoding="utf-8")
    SITLLauncher().start(vehicle="VTOL", instance=0, sysid=1)
    d = _defaults(cmds[0][0])
    assert [p.name for p in d] == ["plane.parm", "vtol_default.parm", "vtol_rate_gains.parm", "identity.parm",
                                   "vtol_enforced.parm"]
    assert not stale.exists()


def test_start_refuses_to_launch_when_the_wind_does_not_read_back(fake_sitl, monkeypatch):
    _root, cmds = fake_sitl
    monkeypatch.setattr(sl, "sim_wind_parm_text", lambda s, d: "SIM_WIND_SPD 0\n")
    with pytest.raises(RuntimeError, match="要求帶入的風沒有生效"):
        SITLLauncher().start(vehicle="VTOL", instance=0, sysid=1, sim_wind=(5.0, 270.0))
    assert cmds == []                                           # 沒有啟動


def test_start_multi_gives_every_instance_the_same_wind(fake_sitl):
    _root, cmds = fake_sitl
    rows = SITLLauncher().start_multi("VTOL", 3, lat=23.95, lon=120.9, sim_wind=(4.5, 300.0))
    assert [r[0] for r in rows] == [0, 1, 2] and len(cmds) == 3
    cwds = set()
    for cmd, cwd in cmds:
        eff = _merged(_defaults(cmd))
        assert (eff["SIM_WIND_SPD"], eff["SIM_WIND_DIR"]) == (4.5, 300.0)
        cwds.add(cwd)
    assert len(cwds) == 3


# ══════════════════════════════════════════════════════════════════════
#  1. 規劃完成的文字（ui/main_window.py）
# ══════════════════════════════════════════════════════════════════════

@_needs_qt
def test_wind_text_reads_the_planner_value():
    res = {}
    assert mw.harvest_wind_text(mw.harvest_plan_wind(_hv(res, _layer(5.0, 270.0)))) == "規劃風 5 m/s 來自 270°"
    assert mw.harvest_wind_text(mw.harvest_plan_wind(_hv(res, _layer(4.5, 359.6)))) == "規劃風 4.5 m/s 來自 0°"
    calm = mw.harvest_wind_text(mw.harvest_plan_wind(_hv(res, _layer(0.0, 270.0))))
    assert "0 m/s＝無風規劃" in calm and "270°" in calm            # 0 m/s 時風向仍影響計畫（§5.2）
    assert "無風規劃" in mw.harvest_wind_text(mw.harvest_plan_wind(_hv(res)))   # 沒有風層
    assert mw.harvest_wind_text(mw.harvest_plan_wind(types.SimpleNamespace(last_ctx=None))) == "規劃風：取不到規劃器生效值"
    two = mw.harvest_plan_wind(_hv(res, _layer(3.0, 90.0), WindLayer(3000.0, 6000.0, 7.0, 100.0)))
    assert (two["speed_mps"], two["dir_from_deg"], two["n_layers"]) == (7.0, 100.0, 2)
    assert mw.harvest_wind_text(two) == "規劃風 7 m/s 來自 100°（2 層取最大）"


class _Panel:
    def __init__(self):
        self.status, self.result = [], None

    def set_plan_status(self, t):
        self.status.append(t)

    def set_plan_result(self, rows, summary):
        self.result = (rows, summary)


def _recon_win(panel, result, hv):
    w = types.SimpleNamespace(vtol_recon_panel=panel, _dccpp_result=result, _hv_coordinator=hv)
    w._get_cesium_widget = lambda: None
    w._polyline_length_m = MainWindow._polyline_length_m
    w._show_mapping_plan_status = lambda h: None
    w._start_hdmap_preview = lambda h: None
    return w


@pytest.fixture(scope="module")
def real_windy_plan():
    """真規劃（一般模式、1 Pioneer + 2 HERO、規劃風 4.5 m/s 來自 300°）：GUI 的 coordinator.hv_wind 那條路。"""
    import tests.test_hdmap_gui_coordinator as g
    from mission.swarm_coordinator import SwarmCoordinator
    coord = SwarmCoordinator()
    coord.use_harvest = True
    coord.hv_class_of = lambda d: g.CLASS_OF[int(d["drone_id"])]
    coord.hv_home = g.ll(0.0, -400.0)
    coord.hv_wind = (4.5, 300.0)
    coord.hv_fence_zones = []
    coord.harvest_params = dict(g.mapping_params(hv_map_enabled=False, hv_wind=(4.5, 300.0)))
    res = coord.plan_coverage_dccpp(areas=g.areas(), drones=g.drones(),
                                    coverage_width_m=g.SWATH * (1.0 - g.SIDE), overlap_rate=0.0,
                                    coverage_altitude=g.ALT, mounting_angle_deg=0.0)
    hv = coord.hv_coordinator
    assert hv is not None and hv.mapping_mode is None
    return hv, res


@_needs_qt
def test_general_mode_summary_states_the_effective_wind(real_windy_plan):
    hv, res = real_windy_plan
    assert hv.last_result is res                                 # GUI 判「這份結果是 HV 規劃的」靠這個
    assert mw.harvest_plan_wind(hv)["speed_mps"] == pytest.approx(4.5)
    panel = _Panel()
    MainWindow._on_vtol_recon_planned(_recon_win(panel, res, hv), {"num_drones": 3, "speed": 18.0}, True)
    rows, summary = panel.result
    assert len(rows) == 3
    assert summary.startswith("規劃完成：3/3 架") and summary.endswith("｜ 規劃風 4.5 m/s 來自 300°")


@_needs_qt
def test_general_mode_summary_says_windless_and_dccpp():
    res = {"assembled_paths": {}, "total_makespan": 0.0}
    panel = _Panel()
    MainWindow._on_vtol_recon_planned(_recon_win(panel, res, _hv(res, _layer(0.0, 0.0))), {"num_drones": 0}, True)
    assert "規劃風 0 m/s＝無風規劃" in panel.result[1]
    # 結果不是這個 hv 的（DCCPP 計畫；上一趟 HV 的轉接器還掛著）⇒ 不能拿 hv 的風來寫
    panel2 = _Panel()
    MainWindow._on_vtol_recon_planned(_recon_win(panel2, res, _hv({}, _layer(5.0, 270.0))), {"num_drones": 0}, True)
    assert panel2.result[1].endswith("｜ 規劃風：DCCPP 不考慮風（無風規劃）")


@_needs_qt
def test_status_bar_after_planning_states_the_wind(real_windy_plan):
    """後處理（_display_dccpp_result，同步與背景兩條路共用）的狀態列也寫規劃風；它在大 try 裡，壞了只會變成一行 log。"""
    import tests.test_hdmap_gui_coordinator as g
    from tests.test_hv_dccpp_adapter import _DisplayWindow
    hv, res = real_windy_plan
    corners = list(g.areas()[0]["polygon"])

    def _show(win):
        MainWindow._display_dccpp_result(
            win, res, params={"collision_avoidance": False, "auto_landing": False, "vehicle_type": "vtol"},
            corners_snapshot=corners, centroid_lat=sum(c[0] for c in corners) / 4,
            centroid_lon=sum(c[1] for c in corners) / 4, altitude=g.ALT, speed=18.0, turn_radius=150.0,
            vehicle_type="vtol", auto_scan=True)
        assert win._dccpp_metrics is not None, "後處理中途丟例外（被大 except 吃掉）"
        return [c[1][0] for c in win._status.calls if c[0] == "showMessage"]

    win = _DisplayWindow(home=g.HOME)
    win._hv_coordinator = hv
    msgs = _show(win)
    assert any(m.startswith("DCCPP 最佳化完成") and m.endswith("| 規劃風 4.5 m/s 來自 300°") for m in msgs), msgs
    win2 = _DisplayWindow(home=g.HOME)                            # 沒有 HV 轉接器 ⇒ DCCPP
    assert any(m.endswith("| 規劃風：DCCPP 不考慮風") for m in _show(win2))


@_needs_qt
def test_panel_shows_zero_as_windless(qt_app):
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    p = VTOLReconPanel()
    assert p.hv_wind_speed_spin.value() == 0.0
    assert p.hv_wind_speed_spin.text() == "0 m/s＝無風規劃"
    assert "0 m/s＝無風規劃" in p.hv_wind_speed_spin.toolTip()
    p.hv_wind_speed_spin.setValue(5.0)
    assert p.hv_wind_speed_spin.text() == "5.0 m/s"
    assert p.get_params()["hv_wind"] == (5.0, 0.0)               # 顯示字不影響送進規劃器的值
    p.deleteLater()


# ══════════════════════════════════════════════════════════════════════
#  2. GUI 啟動 SITL：帶入規劃風
# ══════════════════════════════════════════════════════════════════════

class _Spin:
    def __init__(self, v):
        self._v = v

    def value(self):
        return self._v


@_needs_qt
def test_offer_prefers_the_plan_effective_wind_over_the_panel():
    res = {"assembled_paths": {}}
    panel = types.SimpleNamespace(hv_wind_speed_spin=_Spin(2.0), hv_wind_dir_spin=_Spin(90.0))
    win = types.SimpleNamespace(_hv_coordinator=_hv(res, _layer(5.0, 270.0)), _dccpp_result=res, vtol_recon_panel=panel)
    o = mw.sitl_plan_wind_offer(win)
    assert (o["speed_mps"], o["dir_from_deg"]) == (5.0, 270.0)
    assert o["source"].startswith("目前的 HARVEST 計畫") and "風格子現在是 2 m/s 來自 90°" in o["source"]
    assert o["text"] == "規劃風 5 m/s 來自 270°"
    # 面板與計畫相同 ⇒ 不加註
    win.vtol_recon_panel = types.SimpleNamespace(hv_wind_speed_spin=_Spin(5.0), hv_wind_dir_spin=_Spin(270.0))
    assert mw.sitl_plan_wind_offer(win)["source"] == "目前的 HARVEST 計畫（規劃器生效值）"
    # 沒有 HV 計畫 ⇒ 面板值、來源寫明
    win2 = types.SimpleNamespace(_hv_coordinator=None, _dccpp_result=None, vtol_recon_panel=panel)
    o2 = mw.sitl_plan_wind_offer(win2)
    assert (o2["speed_mps"], o2["dir_from_deg"]) == (2.0, 90.0) and "還沒有計畫" in o2["source"]
    win3 = types.SimpleNamespace(_hv_coordinator=None, _dccpp_result={"x": 1}, vtol_recon_panel=panel)
    assert "DCCPP 計畫（不考慮風）" in mw.sitl_plan_wind_offer(win3)["source"]
    assert mw.sitl_plan_wind_offer(types.SimpleNamespace()) is None


@pytest.fixture
def dialog_mod(monkeypatch):
    """對話框不讀使用者的 settings.json、不碰網路。"""
    import ui.dialogs.sitl_launch_dialog as dmod
    saved = types.SimpleNamespace(enable_fanout=False, auto_firewall=False, home_preset="", instances=[])
    monkeypatch.setattr(dmod, "get_settings", lambda: types.SimpleNamespace(sitl_launch=saved))
    monkeypatch.setattr(dmod, "_get_lan_ip", lambda: "127.0.0.1")
    return dmod


@_needs_qt
def test_dialog_offers_the_wind_checked_with_value_and_direction(qt_app, dialog_mod):
    pw = {"speed_mps": 5.0, "dir_from_deg": 270.0, "source": "目前的 HARVEST 計畫（規劃器生效值）",
          "text": "規劃風 5 m/s 來自 270°"}
    dlg = dialog_mod.SITLLaunchDialog("VTOL", 3, plan_wind=pw)
    assert dlg.chk_sim_wind.isEnabled() and dlg.chk_sim_wind.isChecked()
    assert "5 m/s 來自 270°" in dlg.chk_sim_wind.text() and "SIM_WIND_DIR 270" in dlg.chk_sim_wind.text()
    assert "目前的 HARVEST 計畫" in dlg.sim_wind_hint.text()
    r = dlg.collect_result()
    assert r["sim_wind"] == (5.0, 270.0) and r["sim_wind_source"].startswith("目前的 HARVEST 計畫")
    dlg.chk_sim_wind.setChecked(False)
    assert dlg.collect_result()["sim_wind"] is None and dlg.collect_result()["sim_wind_source"] == ""
    dlg.deleteLater()


@_needs_qt
def test_dialog_without_a_wind_disables_the_option(qt_app, dialog_mod):
    dlg = dialog_mod.SITLLaunchDialog("VTOL", 1)
    assert not dlg.chk_sim_wind.isEnabled() and not dlg.chk_sim_wind.isChecked()
    assert dlg.collect_result()["sim_wind"] is None
    dlg.deleteLater()
    calm = dialog_mod.SITLLaunchDialog("VTOL", 1, plan_wind={"speed_mps": 0.0, "dir_from_deg": 0.0, "source": "s"})
    assert "無風規劃" in calm.chk_sim_wind.text() and calm.collect_result()["sim_wind"] == (0.0, 0.0)
    calm.deleteLater()


@_needs_qt
def test_get_config_passes_the_offer_through(qt_app, dialog_mod, monkeypatch):
    monkeypatch.setattr(dialog_mod.SITLLaunchDialog, "exec", lambda self: QDialog.DialogCode.Accepted)
    r = dialog_mod.SITLLaunchDialog.get_config(
        "VTOL", 2, plan_wind={"speed_mps": 3.0, "dir_from_deg": 0.0, "source": "s"})
    assert r["sim_wind"] == (3.0, 0.0)


class _Hud:
    def __init__(self):
        self.events = []

    def on_sitl_launched(self, *a):
        self.events.append(("launched",) + a)

    def on_sitl_stopped(self):
        self.events.append(("stopped",))

    def set_vehicle(self, v):
        self.events.append(("vehicle", v))


class _StatusBar:
    def __init__(self):
        self.msgs = []

    def showMessage(self, t, *_a):
        self.msgs.append(t)


class _Log:
    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        return lambda msg, *a, **k: self.lines.append((level, str(msg)))


def _launch(monkeypatch, *, dialog_sim_wind, hv_wind=(5.0, 270.0)):
    """跑**真的** MainWindow.on_sitl_launch（假 self、假 launcher）；回傳 (start_multi kwargs, 對話框收到的 plan_wind, log)。"""
    import config.settings as cs
    from ui.dialogs.sitl_launch_dialog import SITLLaunchDialog
    seen = {}

    def _get_config(vehicle, count, parent=None, plan_wind=None):
        seen["plan_wind"] = plan_wind
        return {"enable_fanout": False, "auto_firewall": False, "home_override": None,
                "instances": [], "sim_wind": dialog_sim_wind(plan_wind),
                "sim_wind_source": (plan_wind or {}).get("source", "")}

    class _Launcher:
        def __init__(self):
            self.calls = []

        def start_multi(self, **kw):
            self.calls.append(kw)
            return [(0, "tcp:127.0.0.1:5760", 0)]

        def stop(self):
            self.calls.append("stop")

    monkeypatch.setattr(SITLLauncher, "is_available", classmethod(lambda cls: True))
    monkeypatch.setattr(cs, "get_settings", lambda: types.SimpleNamespace(
        map=types.SimpleNamespace(default_lat=23.95, default_lon=120.9)))
    monkeypatch.setattr(SITLLaunchDialog, "get_config", staticmethod(_get_config))
    monkeypatch.setattr("time.sleep", lambda _s: None)
    log = _Log()
    monkeypatch.setattr(mw, "logger", log)
    res = {"assembled_paths": {}, "vehicle_type": "vtol"}
    sb, conns = _StatusBar(), []
    win = types.SimpleNamespace(
        _sitl_launcher=_Launcher(), _diamond_plans=None, _dccpp_result=res, flight_params={},
        corners=None, home_point=(23.95, 120.9), sitl_hud=_Hud(), vtol_recon_panel=None,
        _hv_coordinator=_hv(res, _layer(*hv_wind)),
        on_sitl_connect=lambda conn, **kw: conns.append((conn, kw)), statusBar=lambda: sb)
    MainWindow.on_sitl_launch(win, "VTOL", 1)
    assert conns and conns[0][0] == "tcp:127.0.0.1:5760", win.sitl_hud.events
    return win._sitl_launcher.calls, seen["plan_wind"], log, sb


@_needs_qt
def test_launch_carries_the_plan_wind_into_start_multi(qt_app, qmb, monkeypatch):
    calls, pw, log, sb = _launch(monkeypatch, dialog_sim_wind=lambda pw: (pw["speed_mps"], pw["dir_from_deg"]))
    assert (pw["speed_mps"], pw["dir_from_deg"]) == (5.0, 270.0) and pw["source"].startswith("目前的 HARVEST 計畫")
    (kw,) = calls
    assert kw["sim_wind"] == (5.0, 270.0) and kw["vehicle"] == "VTOL"
    assert any("模擬風：帶入規劃風 5 m/s 來自 270°" in m for _lv, m in log.lines)
    assert any("模擬風 5 m/s 來自 270°（帶入規劃風）" in m for m in sb.msgs)
    assert qmb["critical"] == []


@_needs_qt
def test_launch_without_wind_warns_when_the_plan_had_wind(qt_app, qmb, monkeypatch):
    calls, _pw, log, sb = _launch(monkeypatch, dialog_sim_wind=lambda pw: None)
    assert calls[0]["sim_wind"] is None
    warns = [m for lv, m in log.lines if lv == "warning" and "模擬風" in m]
    assert len(warns) == 1 and "SITL 無風" in warns[0] and "規劃風 5 m/s 來自 270°" in warns[0]
    assert any(m.endswith("模擬風 無") for m in sb.msgs)
    # 規劃本身就無風 ⇒ 沒帶也不算不一致
    calls2, _pw2, log2, _sb2 = _launch(monkeypatch, dialog_sim_wind=lambda pw: None, hv_wind=(0.0, 270.0))
    assert calls2[0]["sim_wind"] is None
    assert [m for lv, m in log2.lines if lv == "warning" and "模擬風" in m] == []


# ══════════════════════════════════════════════════════════════════════
#  3. scripts/hv_sitl_run.py：規劃風 vs 模擬風
# ══════════════════════════════════════════════════════════════════════

import scripts.hv_sitl_run as runner  # noqa: E402  直接匯入：匯入失敗要紅，不要整檔靜靜跳過


@pytest.mark.parametrize("plan_w,sim,expect", [
    ((5.0, 270.0), (5.0, 270.0, 1.0, 0.0), []),
    ((5.0, 270.0), (5.0, 270.3, 1.0, 0.0), []),                      # 容差內
    ((0.0, 270.0), (0.0, 0.0, 1.0, 0.0), []),                        # 都無風 ⇒ 風向不比
    ((5.0, 0.0), (5.0, 359.8, 1.0, 0.0), []),                        # 0° 與 359.8° 只差 0.2°
    ((0.0, 270.0), (5.0, 270.0, 1.0, 0.0), ["風速"]),                # N_BLIND：規劃 0、飛 5
    ((5.0, 270.0), (0.0, 270.0, 1.0, 0.0), ["風速"]),                # 2026-10-14 那趟
    ((5.0, 270.0), (5.0, 0.0, 1.0, 0.0), ["風向"]),
    ((5.0, 270.0), (5.0, 270.0, 0.0, 0.0), ["隨高度"]),
    ((5.0, 270.0), (5.0, 270.0, 1.0, 0.5), ["湍流"]),
])
def test_plan_sim_wind_problems(plan_w, sim, expect):
    probs = runner.plan_sim_wind_problems(plan_w, *sim)
    assert len(probs) == len(expect)
    for p_, key in zip(probs, expect):
        assert key in p_


class _Link:
    def __init__(self, vals):
        self.vals = vals

    def get_param(self, name, timeout_s=None):
        return self.vals.get(name)


def _set_sim(monkeypatch, spd, d, t=1.0, turb=0.0):
    for k, v in (("SIM_WIND_SPD", spd), ("SIM_WIND_DIR", d), ("SIM_WIND_T", t), ("SIM_WIND_TURB", turb)):
        monkeypatch.setattr(runner, k, v)
    vals = {"SIM_WIND_SPD": spd, "SIM_WIND_DIR": d, "SIM_WIND_T": t, "SIM_WIND_TURB": turb, "MIN_GROUNDSPEED": 0.0}
    return {"A1": _Link(dict(vals)), "B2": _Link(dict(vals))}


def test_readback_warns_loudly_when_plan_and_sim_differ(monkeypatch, tmp_path, capsys):
    links = _set_sim(monkeypatch, 0.0, 270.0)
    monkeypatch.setenv("HV_PLAN_WIND_SPD", "5")
    rec = runner.wind_readback(tmp_path, links, types.SimpleNamespace(wind=(5.0, 270.0)))
    out = capsys.readouterr().out
    assert "10/10 相符" in out
    assert "⚠⚠⚠ 規劃風 ≠ 模擬風（不擋" in out and "風速：規劃 5 m/s ≠ 模擬 0 m/s" in out
    assert out.index("10/10 相符") < out.index("規劃風 ≠ 模擬風")          # 讀回之後才比
    disk = json.loads((tmp_path / "wind_readback.json").read_text(encoding="utf-8"))
    assert disk == rec
    pvs = disk["plan_vs_sim"]
    assert pvs["match"] is False and pvs["from"] == "SITL 讀回值"
    assert pvs["plan"] == {"spd_mps": 5.0, "dir_from_deg": 270.0, "source": "HV_PLAN_WIND_SPD／HV_PLAN_WIND_DIR 覆寫"}
    assert pvs["sim"]["spd_mps"] == 0.0


def test_readback_is_quiet_when_plan_and_sim_match(monkeypatch, tmp_path, capsys):
    links = _set_sim(monkeypatch, 5.0, 270.0)
    monkeypatch.delenv("HV_PLAN_WIND_SPD", raising=False)
    rec = runner.wind_readback(tmp_path, links, types.SimpleNamespace(wind=(5.0, 270.0)))
    out = capsys.readouterr().out
    assert "≠" not in out and "⚠" not in out and "規劃風 = 模擬風：5 m/s 來自 270°" in out
    assert rec["plan_vs_sim"]["match"] is True and rec["plan_vs_sim"]["problems"] == []
    assert rec["plan_vs_sim"]["plan"]["source"] == "情境預設（Scenario.wind）"


def test_readback_failure_still_stops_the_flight(monkeypatch, tmp_path, capsys):
    links = _set_sim(monkeypatch, 5.0, 270.0)
    links["B2"].vals["SIM_WIND_SPD"] = None                       # 韌體沒有這個名字
    with pytest.raises(SystemExit, match="風場參數沒有生效"):
        runner.wind_readback(tmp_path, links, types.SimpleNamespace(wind=(5.0, 270.0)))
    assert "規劃風 ≠ 模擬風" not in capsys.readouterr().out
    assert "plan_vs_sim" not in json.loads((tmp_path / "wind_readback.json").read_text(encoding="utf-8"))


def test_scenario_json_records_plan_and_sim_side_by_side(monkeypatch, tmp_path):
    """plan_and_export 的 scenario.json（規劃換替身，照 tests/test_survey_sitl_s1.py 的作法）。"""
    scenarios = pytest.importorskip("scripts.hv_sitl_scenarios")
    sc = dataclasses.replace(scenarios.SCENARIOS["gui_legal_120_1a2b"], wind=(0.0, 270.0))
    up = dict(strips=[("Z1_1_A_0", 1)], waypoints=[], distance_m=0.0, t_takeoff_s=0.0, t_land_s=0.0,
              energy_used_wh=0.0, energy_margin_wh=0.0, takeoff_site="HOME")
    plan = types.SimpleNamespace(plans={u: types.SimpleNamespace(cls=u[0], **up) for u in ("A1", "B2", "B3")},
                                 makespan_s=0.0, uncovered=[])
    assert len(sc.fleet) == 3
    hv = types.SimpleNamespace(last_plan=plan, last_ctx=types.SimpleNamespace(sensors={}),
                               cfg=types.SimpleNamespace(dh_band_min_m=15.0),
                               last_inputs=types.SimpleNamespace(payload_reqs={}),
                               platform_a=types.SimpleNamespace(min_fw_operation_agl_m=100.0),
                               platform_b=types.SimpleNamespace(min_fw_operation_agl_m=100.0))
    fake_hvlib = types.ModuleType("hv_dccpp")
    fake_hvlib.build_validation_cache = lambda *a, **kw: None
    fake_hvlib.validate = lambda *a, **kw: types.SimpleNamespace(passed=True, conflicts=[], coverage_ok=True,
                                                                 energy_ok=True, min_sep_h_m=200.0, min_sep_v_m=0.0)
    fake_hvlib.export_all = lambda *a, **kw: {"uavs": {}}
    monkeypatch.setitem(sys.modules, "hv_dccpp", fake_hvlib)
    monkeypatch.setattr(runner, "out_dir", lambda sc_: tmp_path)
    monkeypatch.setattr(runner, "_plan_gui", lambda sc_, k_: (hv, {"status": "GO"}))
    monkeypatch.setattr(runner, "_plan_direct", lambda sc_: (hv, {"status": "GO"}))
    monkeypatch.setenv("HV_PLAN_WIND_SPD", "0")
    _set_sim(monkeypatch, 5.0, 270.0)
    runner.plan_and_export(sc, None)
    meta = json.loads((tmp_path / "scenario.json").read_text(encoding="utf-8"))
    assert meta["wind"] == [0.0, 270.0]                                   # 舊欄位不動（分析腳本在讀）
    assert meta["sim_wind"]["spd_mps"] == 5.0
    assert meta["plan_wind"] == {"spd_mps": 0.0, "dir_from_deg": 270.0,
                                 "source": "HV_PLAN_WIND_SPD／HV_PLAN_WIND_DIR 覆寫"}
    pvs = meta["wind_plan_vs_sim"]
    assert pvs["match"] is False and any("風速：規劃 0 m/s ≠ 模擬 5 m/s" in p_ for p_ in pvs["problems"])
    # 報告開頭：沒有讀回檔 ⇒ 用 scenario.json；有讀回 ⇒ 用讀回
    head = runner.wind_header_lines(meta)
    assert head[0].startswith("規劃風 0 m/s 來自 270°") and "scenario.json" in head[0]
    assert head[1].startswith("⚠⚠ 規劃風 ≠ 模擬風")
    rb = {"plan_vs_sim": runner.wind_plan_vs_sim((5.0, 270.0), 5.0, 270.0)}
    assert runner.wind_header_lines(meta, rb)[1] == "規劃風 = 模擬風" and "SITL 讀回" in runner.wind_header_lines(meta, rb)[0]


def test_report_header_handles_old_artifacts():
    old = {"wind": [5.0, 270.0], "sim_wind": {"spd_mps": 5.0, "dir_from_deg": 270.0, "turb": 0.0, "profile_type": 1.0}}
    assert runner.wind_header_lines(old)[1] == "規劃風 = 模擬風"
    old["sim_wind"]["spd_mps"] = 0.0
    assert runner.wind_header_lines(old)[1].startswith("⚠⚠ 規劃風 ≠ 模擬風")
    assert "沒有記錄" in runner.wind_header_lines({"wind": [5.0, 270.0]})[0]     # 09-14 前的趟次：不猜
    assert runner.wind_header_lines({}) == ["規劃風：scenario.json 沒有記錄"]


def test_report_prints_the_wind_header_first():
    """hv_sitl_report.main 在畫圖、算任何東西之前先印風（讀 scenario.json＋wind_readback.json）。"""
    src = (ROOT / "scripts" / "hv_sitl_report.py").read_text(encoding="utf-8")
    body = src[src.index("def main()"):]
    assert body.index("wind_header_lines(") < body.index("plt.subplots(")
