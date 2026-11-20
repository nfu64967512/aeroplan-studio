"""survey/ppk/rtklib.py —— 命令列、設定檔、找執行檔、執行流程（subprocess 全部 mock；本機沒有 RTKLIB）。"""
import subprocess
from pathlib import Path

import pytest

from survey.ppk import rtklib
from survey.ppk.rtklib import (BaseStation, RtklibError, RtklibNotFound, convbin_command,
                               find_executable, render_config, rnx2rtkp_command, rnx2rtkp_options,
                               run_ppk)

BASE = BaseStation(lat_deg=25.123456789, lon_deg=121.234567891, h_ell_m=45.6789,
                   frame="e-GNSS[2025]", antenna_height_m=1.5, antenna_type="TRM59800.00 NONE")


def test_convbin_command_exact():
    assert convbin_command("in/rover.ubx", "w/rover.obs", "w/rover.nav", exe="C:/rtk/convbin.exe") == [
        "C:/rtk/convbin.exe", "-r", "ubx", "-v", "3.04", "-od", "-os", "-oi", "-ot", "-ol",
        "-o", "w/rover.obs", "-n", "w/rover.nav", "in/rover.ubx"]
    assert convbin_command("a.ubx", "a.obs", "a.nav", rinex_version="3.03", extra_args=["-halfc"])[4:11] == \
        ["3.03", "-od", "-os", "-oi", "-ot", "-ol", "-halfc"]


def test_rnx2rtkp_command_exact():
    assert rnx2rtkp_command("r.obs", "b.obs", ["r.nav", "b.nav"], "r.pos", "c.conf", exe="rk") == [
        "rk", "-k", "c.conf", "-o", "r.pos", "r.obs", "b.obs", "r.nav", "b.nav"]
    with pytest.raises(ValueError, match="導航檔"):
        rnx2rtkp_command("r.obs", "b.obs", [], "r.pos", "c.conf")


def test_default_options_values():
    o = rnx2rtkp_options(BASE)
    expect = {
        "pos1-posmode": "kinematic", "pos1-soltype": "combined", "pos1-frequency": "l1+l2",
        "pos1-elmask": "15", "pos1-navsys": "45", "pos1-dynamics": "on",
        "pos2-armode": "fix-and-hold", "pos2-gloarmode": "off", "pos2-bdsarmode": "off",
        "out-solformat": "llh", "out-outhead": "on", "out-outvel": "on", "out-timesys": "gpst",
        "out-timeform": "tow", "out-degform": "deg", "out-height": "ellipsoidal",
        "misc-timeinterp": "on",
        "ant2-postype": "llh", "ant2-pos1": "25.123456789", "ant2-pos2": "121.234567891",
        "ant2-pos3": "45.6789", "ant2-antdelu": "1.5000", "ant2-anttype": "TRM59800.00 NONE",
    }
    assert {k: o[k] for k in expect} == expect
    # navsys 位元：GPS 1 + GLO 4 + GAL 8 + BDS 32
    assert int(o["pos1-navsys"]) == 1 | 4 | 8 | 32
    assert "ant2-anttype" not in rnx2rtkp_options(BaseStation(25, 121, 40, "own-base"))


def test_overrides_applied_locked_rejected():
    o = rnx2rtkp_options(BASE, {"pos2-armode": "continuous", "pos2-gloarmode": "on",
                                "out-solformat": "xyz"})
    assert (o["pos2-armode"], o["pos2-gloarmode"], o["out-solformat"]) == ("continuous", "on", "xyz")
    for k, v in [("out-timesys", "utc"), ("out-height", "geodetic"), ("out-degform", "dms"),
                 ("out-outhead", "off"), ("out-solformat", "enu"), ("pos1-soltype", "forward")]:
        with pytest.raises(ValueError, match=k):
            rnx2rtkp_options(BASE, {k: v})
    for k in ("ant2-pos1", "ant2-postype"):
        with pytest.raises(ValueError, match="BaseStation"):
            rnx2rtkp_options(BASE, {k: "0"})


def test_render_config_round_trips():
    o = rnx2rtkp_options(BASE)
    txt = render_config(o, "line one\nline two")
    body = [ln for ln in txt.splitlines() if ln and not ln.startswith("#")]
    parsed = {ln.split("=", 1)[0].strip(): ln.split("=", 1)[1] for ln in body}
    assert parsed == o
    assert "pos1-posmode       =kinematic" in txt.splitlines()
    assert txt.splitlines()[1:3] == ["# line one", "# line two"]


def test_base_station_validation():
    with pytest.raises(ValueError, match="框架"):
        BaseStation(25, 121, 40, "  ")
    with pytest.raises(ValueError, match="範圍"):
        BaseStation(95, 121, 40, "x")


def test_find_executable(tmp_path, monkeypatch):
    exe = tmp_path / "convbin.exe"
    exe.write_bytes(b"")
    assert find_executable("convbin", exe) == str(exe)
    with pytest.raises(RtklibNotFound, match="不存在"):
        find_executable("convbin", tmp_path / "nope.exe")
    monkeypatch.setattr(rtklib.shutil, "which", lambda name: None)
    with pytest.raises(RtklibNotFound, match="demo5"):
        find_executable("rnx2rtkp")
    monkeypatch.setattr(rtklib.shutil, "which", lambda name: f"C:/bin/{name}.exe")
    assert find_executable("rnx2rtkp") == "C:/bin/rnx2rtkp.exe"
    assert issubclass(RtklibNotFound, FileNotFoundError)


class _FakeRun:
    """記錄呼叫、依命令產生輸出檔；不會真的啟動任何程式。"""

    def __init__(self, rc=0, write=True, timeout_on=None):
        self.calls, self.rc, self.write, self.timeout_on = [], rc, write, timeout_on

    def __call__(self, cmd, **kw):
        self.calls.append((list(cmd), kw))
        name = cmd[0]
        if self.timeout_on and self.timeout_on in name:
            raise subprocess.TimeoutExpired(cmd, kw.get("timeout"))
        if self.write and self.rc == 0:
            if "convbin" in name:
                for flag in ("-o", "-n"):
                    Path(cmd[cmd.index(flag) + 1]).write_text("RINEX")
            else:
                Path(cmd[cmd.index("-o") + 1]).write_text("% GPST\n")
        return subprocess.CompletedProcess(cmd, self.rc, stdout="out-text", stderr="err-text")


@pytest.fixture
def exes(tmp_path):
    cb, rk = tmp_path / "convbin.exe", tmp_path / "rnx2rtkp.exe"
    cb.write_bytes(b"")
    rk.write_bytes(b"")
    return str(cb), str(rk)


def test_run_ppk_sequence_and_files(tmp_path, monkeypatch, exes):
    fake = _FakeRun()
    monkeypatch.setattr(rtklib.subprocess, "run", fake)
    wd = tmp_path / "work"
    run = run_ppk(tmp_path / "rover.ubx", tmp_path / "base.obs", BASE, wd, extra_nav=["brdc.nav"],
                  overrides={"pos2-armode": "continuous"}, convbin_exe=exes[0], rnx2rtkp_exe=exes[1],
                  timeout_s=123)
    obs, nav, pos, conf = wd / "rover.obs", wd / "rover.nav", wd / "rover.pos", wd / "rnx2rtkp.conf"
    assert [c for c, _ in fake.calls] == [
        convbin_command(tmp_path / "rover.ubx", obs, nav, exe=exes[0]),
        rnx2rtkp_command(obs, tmp_path / "base.obs", [nav, "brdc.nav"], pos, conf, exe=exes[1]),
    ]
    assert all(kw["timeout"] == 123 and kw["capture_output"] for _, kw in fake.calls)
    assert run.pos_path == pos and run.commands == [c for c, _ in fake.calls]
    assert run.options["pos2-armode"] == "continuous"
    assert conf.read_text(encoding="utf-8") == render_config(run.options, "base frame: e-GNSS[2025]")
    assert run.logs == ["out-texterr-text"] * 2


def test_run_ppk_failures(tmp_path, monkeypatch, exes):
    kw = dict(convbin_exe=exes[0], rnx2rtkp_exe=exes[1])
    monkeypatch.setattr(rtklib.subprocess, "run", _FakeRun(rc=3))
    with pytest.raises(RtklibError, match=r"rc=3.*err-text"):
        run_ppk(tmp_path / "r.ubx", tmp_path / "b.obs", BASE, tmp_path / "w1", **kw)
    monkeypatch.setattr(rtklib.subprocess, "run", _FakeRun(write=False))
    with pytest.raises(RtklibError, match="觀測檔"):
        run_ppk(tmp_path / "r.ubx", tmp_path / "b.obs", BASE, tmp_path / "w2", **kw)
    monkeypatch.setattr(rtklib.subprocess, "run", _FakeRun(timeout_on="rnx2rtkp"))
    with pytest.raises(RtklibError, match="未結束"):
        run_ppk(tmp_path / "r.ubx", tmp_path / "b.obs", BASE, tmp_path / "w3", timeout_s=5, **kw)


def test_run_ppk_missing_exe_runs_nothing(tmp_path, monkeypatch):
    fake = _FakeRun()
    monkeypatch.setattr(rtklib.subprocess, "run", fake)
    monkeypatch.setattr(rtklib.shutil, "which", lambda name: None)
    with pytest.raises(RtklibNotFound):
        run_ppk(tmp_path / "r.ubx", tmp_path / "b.obs", BASE, tmp_path / "w")
    assert fake.calls == [] and not (tmp_path / "w").exists()
