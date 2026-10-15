"""多台內建 SITL 必須各自拿到不同的 MAVLink 機號（2026-10-07 GUI 實際發生的回歸）。

症狀：GUI 啟動 3 台 VTOL SITL，三個行程都起來了（5760/5770/5780），但三條連線
都回報 ``system 1`` ⇒ HUD 併成一張卡、Cesium 一個圖示在三個起飛坪之間跳。

成因有兩半，**要同時成立**才會出事：
  1. SITL 對話框沒勾 fan-out ⇒ ``instance_configs=None`` ⇒ 舊碼不帶 ``--sysid``；
  2. ``identity.parm`` 寫的是 ``SYSID_THISMAV``，V4.8 已改名 ``MAV_SYSID``，
     ``--defaults`` 裡韌體不認識的名字會被**靜默略過**。

``tests/test_sitl_param_names.py`` 鎖住第 2 半（identity.parm 的文字）；這裡鎖住
**真正走過 start_multi → start 的那條路徑**：命令列與寫到磁碟的檔案都要帶對的值。

⚠ 安全：``start()`` 會刪工作目錄裡的 eeprom.bin 並啟動行程。這裡把 ``_SITL_DIR``
整個換成 tmp_path、``Popen`` 換成只記錄參數的替身，絕不碰 repo 的 sitl/plane/i*。
"""

from __future__ import annotations

import logging

import pytest

import mission.sitl_launcher as sl


class _FakeProc:
    _next_pid = 9000

    def __init__(self, cmd, **kwargs):
        self.cmd = list(cmd)
        self.cwd = kwargs.get("cwd")
        _FakeProc._next_pid += 1
        self.pid = _FakeProc._next_pid

    def poll(self):
        return None        # 「還活著」


@pytest.fixture
def launcher(tmp_path, monkeypatch):
    sitl = tmp_path / "sitl"
    (sitl / "default_params").mkdir(parents=True)
    for exe in ("ArduPlane.exe", "ArduCopter.exe"):
        (sitl / exe).write_bytes(b"")
    for parm in ("plane.parm", "copter.parm"):
        # ⚠ 與真的 plane.parm 一樣帶 ARMING_REQUIRE 0：強制檔要蓋得過它才算數
        (sitl / "default_params" / parm).write_text(
            "ARMING_SKIPCHK 0\nARMING_REQUIRE 0\n", encoding="utf-8")

    procs: list[_FakeProc] = []

    def _popen(cmd, **kwargs):
        p = _FakeProc(cmd, **kwargs)
        procs.append(p)
        return p

    # ⚠ launcher 的 logger 掛著寫進 data/logs/UAVPathPlanner_<日期>.log 的 FileHandler；
    #   不換掉的話，每跑一次就在使用者的 GUI 日誌裡留下一串假的「SITL 已啟動 (PID=9001)」。
    _quiet = logging.getLogger("test_sitl_sysid_cmdline.quiet")
    _quiet.handlers[:] = [logging.NullHandler()]
    _quiet.propagate = False
    monkeypatch.setattr(sl, "logger", _quiet)
    monkeypatch.setattr(sl, "_SITL_DIR", sitl)
    monkeypatch.setattr(sl.subprocess, "Popen", _popen)
    monkeypatch.setattr(sl.time, "sleep", lambda *_a, **_k: None)
    monkeypatch.setattr(sl, "_tcp_port_listener_pid", lambda port: None)

    lch = sl.SITLLauncher()
    lch._test_procs = procs
    lch._test_sitl = sitl
    return lch


def _cmd_value(cmd, flag):
    assert cmd.count(flag) == 1, f"{flag} 應恰好出現一次：{cmd}"
    return cmd[cmd.index(flag) + 1]


def _identity_sysid(path):
    lines = [ln for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip()]
    hits = [ln for ln in lines if ln.startswith("MAV_SYSID=")]
    assert len(hits) == 1, f"identity.parm 應恰有一行 MAV_SYSID：{lines}"
    assert not any(ln.startswith("SYSID_THISMAV") for ln in lines), \
        "identity.parm 不可再寫舊名 SYSID_THISMAV（V4.8 會靜默略過）"
    return int(hits[0].split("=", 1)[1])


def test_start_multi_without_configs_gives_distinct_sysids(launcher):
    """GUI 不勾 fan-out 的那條路徑（instance_configs=None）—— 就是這次出事的那條。"""
    homes = [(25.0741, 121.2288, 0.0), (25.0741, 121.2294, 0.0), (25.0741, 121.2300, 0.0)]
    res = launcher.start_multi("VTOL", 3, 25.0741, 121.2288, alt=0.0, homes=homes)

    assert [r[0] for r in res] == [0, 1, 2]
    cmds = [p.cmd for p in launcher._test_procs]
    assert len(cmds) == 3
    sysids_cmd = [int(_cmd_value(c, "--sysid")) for c in cmds]
    assert sysids_cmd == [1, 2, 3], f"命令列 --sysid 應為 1/2/3，實際 {sysids_cmd}"

    sysids_file = [_identity_sysid(launcher._test_sitl / "plane" / f"i{i}" / "identity.parm")
                   for i in range(3)]
    assert sysids_file == [1, 2, 3], f"identity.parm 的 MAV_SYSID 應為 1/2/3，實際 {sysids_file}"


def test_explicit_sysid_from_dialog_wins(launcher):
    """對話框有給 sysid（嵌入式 MAVROS 對齊用）時，命令列與檔案都用它。"""
    cfgs = [{"sysid": 11}, {"sysid": 12}]
    launcher.start_multi("VTOL", 2, 25.0, 121.0, instance_configs=cfgs)
    cmds = [p.cmd for p in launcher._test_procs]
    assert [int(_cmd_value(c, "--sysid")) for c in cmds] == [11, 12]
    assert [_identity_sysid(launcher._test_sitl / "plane" / f"i{i}" / "identity.parm")
            for i in range(2)] == [11, 12]


def test_port_shift_keeps_logical_sysid(launcher, monkeypatch):
    """port 被佔用 ⇒ instance 位移（實測 wslrelay 佔 5760-5790）時，機號仍是邏輯序號+1，
    否則 HUD 標籤（logical+1）會對到別台機體。"""
    busy = {sl.SITLLauncher.tcp_port_for(0)}               # 只有 5760 被佔
    monkeypatch.setattr(sl, "_tcp_port_listener_pid", lambda port: 4242 if port in busy else None)
    monkeypatch.setattr(sl, "_process_name_by_pid", lambda pid: "fake-holder", raising=False)

    res = launcher.start_multi("VTOL", 2, 25.0, 121.0)
    actual = [r[2] for r in res]
    assert actual[0] != 0, f"第一台應被位移離開 instance 0，實際 {actual}"
    cmds = [p.cmd for p in launcher._test_procs]
    assert [int(_cmd_value(c, "--sysid")) for c in cmds] == [1, 2], \
        "位移後機號仍須是邏輯序號+1，不可變成 實際instance+1"


def test_start_alone_port_shift_uses_requested_instance(launcher, monkeypatch):
    """直接呼叫 start()（不經 start_multi、不給 sysid）時，預設機號取**要求的** instance+1。

    start_multi 會明確傳 sysid，所以上一條測試蓋不到 start() 自己的預設值 ——
    兩層防護互相遮蔽，這條單獨鎖 start() 那一層。
    """
    busy = {sl.SITLLauncher.tcp_port_for(0)}
    monkeypatch.setattr(sl, "_tcp_port_listener_pid", lambda port: 4242 if port in busy else None)
    monkeypatch.setattr(sl, "_process_name_by_pid", lambda pid: "fake-holder", raising=False)

    launcher.start("VTOL", 25.0, 121.0, instance=0)
    (cmd,) = [p.cmd for p in launcher._test_procs]
    shifted = int(_cmd_value(cmd, "--instance"))
    assert shifted != 0, "前提：instance 0 的 port 被佔，應該已經位移"
    assert int(_cmd_value(cmd, "--sysid")) == 1, \
        f"要求 instance 0 ⇒ 機號 1；位移到 {shifted} 不可讓機號變成 {shifted + 1}"
    assert _identity_sysid(launcher._test_sitl / "plane" / f"i{shifted}" / "identity.parm") == 1


# ═════════════════════════════════════════════════════════════════════
#  韌體實際看到的**有效值**（--defaults 後面蓋前面）
# ═════════════════════════════════════════════════════════════════════
#   2026-10-08 使用者回報：SITL 一直保持解鎖、無法上鎖；進 AUTO 顯示 VTOLTakeoff 但飛機不起飛。
#   獨立 SITL 實測（同一顆 binary、同一個 home）：
#     目前 --defaults（plane + vtol_default + identity） → ARMING_REQUIRE=0：開機就是已解鎖、
#         DISARM 被拒、四軸馬達恆 1000 µs、最高 0.2 m
#     最後再疊強制檔（ARMING_REQUIRE 1）+ 迴圈 100 Hz → 垂直起飛、轉換、飛到航點 7，最高 106 m
def _effective(cmd) -> dict:
    """依 --defaults 的檔案順序合併（後面蓋前面），回傳 {參數名: 值}。"""
    files = _cmd_value(cmd, "--defaults").split(",")
    out = {}
    for f in files:
        for raw in open(f, encoding="utf-8").read().splitlines():
            line = raw.split("#", 1)[0].strip().replace("=", " ")
            parts = line.split()
            if len(parts) >= 2:
                try:
                    out[parts[0]] = float(parts[1])
                except ValueError:
                    pass
    return out


def test_vtol_launch_firmware_sees_arming_required_and_quadplane_on(launcher):
    """有效值：ARMING_REQUIRE=1（能上鎖、馬達會轉）、Q_ENABLE=1、四軸馬達有對應。"""
    launcher.start("VTOL", 25.0, 121.0, instance=0)
    (cmd,) = [p.cmd for p in launcher._test_procs]
    eff = _effective(cmd)
    assert eff.get("ARMING_REQUIRE") == 1.0, f"有效 ARMING_REQUIRE={eff.get('ARMING_REQUIRE')}（0 = 永遠已解鎖、不起飛）"
    assert eff.get("Q_ENABLE") == 1.0
    assert [eff.get(f"SERVO{n}_FUNCTION") for n in (5, 6, 7, 8)] == [33.0, 34.0, 35.0, 36.0]


def test_vtol_enforced_file_is_the_last_defaults_entry_of_the_real_command(launcher):
    launcher.start("VTOL", 25.0, 121.0, instance=0)
    (cmd,) = [p.cmd for p in launcher._test_procs]
    files = _cmd_value(cmd, "--defaults").split(",")
    assert files[-1].endswith("vtol_enforced.parm"), files
    assert files.count(files[-1]) == 1
    # 強制檔寫在**每台自己的工作目錄**（不共用檔案，多台同時啟動不會互踩）
    assert "plane" in files[-1] and "i0" in files[-1], files[-1]


def test_vtol_launch_loop_rate_meets_the_quadplane_arming_minimum(launcher):
    """有效 SCHED_LOOP_RATE >= 100：韌體對 < 100 的 QuadPlane 回 `Arm: quadplane needs SCHED_LOOP_RATE >= 100`。"""
    launcher.start("VTOL", 25.0, 121.0, instance=0)
    (cmd,) = [p.cmd for p in launcher._test_procs]
    rate = _effective(cmd)["SCHED_LOOP_RATE"]
    # 下限 100：韌體；上限 222：陀螺儀 400 Hz ÷ 1.8（PreArm 要 gyro >= loop×1.8），本機實測也只撐到 267
    assert 100 <= rate <= 222, f"有效 SCHED_LOOP_RATE={rate}，要落在 [100, 222]"


@pytest.mark.parametrize("vehicle", ["PLANE", "COPTER"])
def test_non_vtol_launch_has_no_enforced_file_and_keeps_50_hz(launcher, vehicle):
    launcher.start(vehicle, 25.0, 121.0, instance=0)
    (cmd,) = [p.cmd for p in launcher._test_procs]
    files = _cmd_value(cmd, "--defaults").split(",")
    assert not any(f.endswith("vtol_enforced.parm") for f in files), files
    assert _effective(cmd)["SCHED_LOOP_RATE"] == 50.0, "使用者要的低 Hz：非 VTOL 維持 50"


def test_vtol_launch_refuses_to_start_without_the_enforced_file(launcher, monkeypatch):
    """寫不出強制檔 ⇒ 大聲失敗、不啟動。靜默少了它 = 飛機解得了鎖、進得了 AUTO、就是不起飛。"""
    def _boom(*a, **k):
        raise OSError("disk full")

    monkeypatch.setattr(sl, "write_vtol_enforced_parm", _boom)
    with pytest.raises(RuntimeError, match="強制參數檔"):
        launcher.start("VTOL", 25.0, 121.0, instance=0)
    assert launcher._test_procs == [], "強制檔寫不出來卻還是啟動了 SITL"


def test_vtol_enforced_file_lives_in_each_instances_own_workdir(launcher, monkeypatch):
    """審查（2026-10-08）：每台 VTOL 的強制檔要**各自**在自己的 plane/i{instance}/ 底下（含 port 位移後）。

    舊的檢查只對 instance 0 做「路徑含 plane 與 i0」的子字串比對 ⇒ 所有 instance 共用 i0 的那份
    （蓋掉彼此、同時啟動時互相截斷）完全抓不到，而且 pytest 的 tmp 路徑碰巧含 i0 就會隱藏它。
    """
    from pathlib import Path
    busy = {sl.SITLLauncher.tcp_port_for(0)}               # 順便逼出一次 port 位移
    monkeypatch.setattr(sl, "_tcp_port_listener_pid", lambda port: 4242 if port in busy else None)
    monkeypatch.setattr(sl, "_process_name_by_pid", lambda pid: "fake-holder", raising=False)
    launcher.start_multi("VTOL", 3, 25.0, 121.0)
    lasts = []
    for p in launcher._test_procs:
        inst = int(_cmd_value(p.cmd, "--instance"))
        last = Path(_cmd_value(p.cmd, "--defaults").split(",")[-1])
        assert last == launcher._test_sitl / "plane" / f"i{inst}" / "vtol_enforced.parm", last
        assert last.exists()
        lasts.append(last)
    assert len(set(lasts)) == len(launcher._test_procs) == 3, "有 instance 共用同一份強制檔"
