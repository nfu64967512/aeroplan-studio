"""survey.dflog：真實 SITL 日誌的數值，以及 ≤4.6 / ≥4.7 格式差異、任務執行事件、GPS 時間。"""

from pathlib import Path

import pytest

from survey import datum, dflog, qa
from tests.test_survey_synthlog import (CAM_COLS, CAM_FMT_CENTIDEG, CAM_FMT_FLOAT, DFWriter, SurveyTruth,
                                        write_survey_log)

REPO = Path(__file__).resolve().parents[1]
REAL = {
    "i100": (REPO / "sitl/vtol/i100/logs/00000019.BIN", 170, 3),
    "i101": (REPO / "sitl/vtol/i101/logs/00000019.BIN", 113, 2),
    "i102": (REPO / "sitl/vtol/i102/logs/00000019.BIN", 226, 4),
}


def _real(key):
    p = REAL[key][0]
    if not p.exists():
        pytest.skip(f"本機沒有 SITL 日誌 {p}")
    return dflog.read_dflog(p)


@pytest.mark.parametrize("key", sorted(REAL))
def test_real_log_cam_counts_and_firmware(key):
    log = _real(key)
    _, n_cam, n_strips = REAL[key]
    cam = dflog.cam_events(log)
    assert len(cam["TimeUS"]) == n_cam
    assert log.count("TRIG") == 0                       # 沒設回授腳：只有 CAM
    assert log.firmware == "ArduPlane V4.8.0-dev (85d417ee)"
    assert log.firmware_version == (4, 8, 0)
    assert log.formats["CAM"][0] == "QBHIHLLeeefff"
    assert log.angle_encoding["CAM"] == "float_deg"
    assert log.angle_encoding["XKF1"] == "int16_centideg"
    w = dflog.strip_windows(log)
    assert len(w) == n_strips
    assert all(x.trigg_dist_m == pytest.approx(29.699, abs=5e-4) for x in w)
    assert all(not x.flags for x in w)
    # 每一筆 CAM 都落在某條帶的 206 開／關時間窗內
    assert all(dflog.strip_of(w, int(t)) is not None for t in cam["TimeUS"])
    assert log.params["CAM1_DURATION"] == 1.0 and log.params["LOG_DISARMED"] == 0.0


def test_real_log_i100_windows_and_spacing():
    log = _real("i100")
    w = dflog.strip_windows(log)
    assert [(x.cnum_on, x.cnum_off) for x in w] == [(23, 25), (39, 41), (60, 63)]
    assert [x.t_on_us for x in w] == [477206578, 589006578, 724506578]
    assert w[-1].t_off_us == 803906578                  # OFF 在 DO_LAND_START(61) 之後（評估 §2 第 6 點）
    # MISE 大量缺漏，執行事件主要來自 GCS 文字
    ex = dflog.mission_executions(log)
    assert sum(1 for e in ex if e.source == "MSG") > sum(1 for e in ex if e.source == "MISE")
    sp = qa.spacing_stats(dflog.cam_events(log), w)
    for s in sp:
        assert s["median_ok"] and s["p99_ok"]
        assert 29.68 <= s["min_m"] and s["max_m"] <= 30.27      # V1 查核：29.68–30.26 m
    cam = dflog.cam_events(log)
    assert cam["R"][0] == pytest.approx(19.947185516357422)       # 第一張（立即拍）在轉彎改出中


def test_real_log_gps_time_and_undulation():
    log = _real("i100")
    g = dflog.gps_time_fit(log)
    assert g.a == pytest.approx(1.0, abs=1e-6)
    assert g.rms_s < 1e-3 and g.n == 1332
    # 第一筆 GPS：TimeUS 124706578 ↔ GWk 2437、GMS 99068600
    assert g.gpst_s(124706578) == pytest.approx(2437 * 604800 + 99068.6, abs=2e-3)
    rx = datum.receiver_undulation_from_log(log)
    assert rx.source == "GPA.AEl" and rx.n_m == 0.0 and rx.n_samples == 1332   # SITL：橢球高 = AMSL


def _cam_log(tmp_path, fmt, r, p, y, units=None):
    w = DFWriter()
    if units is not None:
        w.define("UNIT", "QbZ", ["TimeUS", "Id", "Label"])
        w.define("FMTU", "QBNN", ["TimeUS", "FmtType", "UnitIds", "MultIds"])
    w.define("CAM", fmt, CAM_COLS)
    if units is not None:
        for ch, label in (("d", "deg"), ("r", "rad"), ("m", "m"), ("s", "s")):
            w.write("UNIT", TimeUS=1, Id=ord(ch), Label=label)
        w.write("FMTU", TimeUS=2, FmtType=w.fmts["CAM"][0], UnitIds=units, MultIds="F----GGBBB000")
    for k in range(3):
        w.write("CAM", TimeUS=1_000_000 + k * 1000, I=0, Img=k + 1, Lat=25.1, Lng=121.2, Alt=100.0,
                R=r, P=p, Y=y)
    return w.save(tmp_path / f"cam_{fmt}.BIN")


def test_centidegree_format_reads_degrees(tmp_path):
    """≤4.6 的 'ccC'（int16 百分度）與 ≥4.7 的 'fff'（float 度）讀出同樣的度數。"""
    old = dflog.read_dflog(_cam_log(tmp_path, CAM_FMT_CENTIDEG, -12.34, 5.67, 350.25))
    new = dflog.read_dflog(_cam_log(tmp_path, CAM_FMT_FLOAT, -12.34, 5.67, 350.25))
    assert old.angle_encoding["CAM"] == "int16_centideg" and new.angle_encoding["CAM"] == "float_deg"
    for log in (old, new):
        c = dflog.cam_events(log)
        assert c["R"][0] == pytest.approx(-12.34, abs=1e-4)
        assert c["P"][0] == pytest.approx(5.67, abs=1e-4)
        assert c["Y"][0] == pytest.approx(350.25, abs=1e-4)


def test_unknown_angle_format_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="無法判讀角度單位"):
        dflog.read_dflog(_cam_log(tmp_path, "QBHIHLLeeehhh", 1, 2, 3))


def test_fmtu_unit_not_degrees_is_rejected(tmp_path):
    ok = dflog.read_dflog(_cam_log(tmp_path, CAM_FMT_FLOAT, 1.0, 2.0, 3.0, units="s#---DUmmmddd"))
    assert ok.angle_encoding["CAM"] == "float_deg"
    with pytest.raises(ValueError, match="不是度"):
        dflog.read_dflog(_cam_log(tmp_path, CAM_FMT_FLOAT, 1.0, 2.0, 3.0, units="s#---DUmmmrrr"))


def test_cam_instance_selection(tmp_path):
    w = DFWriter()
    w.define("CAM", CAM_FMT_FLOAT, CAM_COLS)
    for k, inst in enumerate([0, 1, 0, 1, 0]):
        w.write("CAM", TimeUS=1_000_000 + k * 500_000, I=inst, Img=k + 1, Lat=25.1, Lng=121.2)
    log = dflog.read_dflog(w.save(tmp_path / "two.BIN"))
    assert dflog.cam_events(log, instance=0)["TimeUS"].tolist() == [1_000_000, 2_000_000, 3_000_000]
    assert dflog.cam_events(log, instance=1)["Img"].tolist() == [2, 4]
    with pytest.raises(ValueError, match="多個相機編號"):
        dflog.cam_events(log, instance=None)
    with pytest.raises(ValueError):
        dflog.cam_events(log, kind="GPS")


def _mission_log(tmp_path, events, *, dump=None):
    """events: [(t_us, kind, cnum, cid, prm1)]，kind 'MISE' 或 'MSG'；dump: [(cnum, cid, prm1)]。"""
    w = DFWriter()
    w.define("MSG", "QBBZ", ["TimeUS", "ID", "Seq", "Message"])
    cols = ["TimeUS", "CTot", "CNum", "CId", "Prm1", "Prm2", "Prm3", "Prm4", "Lat", "Lng", "Alt", "Frame"]
    w.define("CMD", "QHHHffffLLfB", cols)
    w.define("MISE", "QHHHffffLLfB", cols)
    w.define("POS", "QLLfff", ["TimeUS", "Lat", "Lng", "Alt", "RelHomeAlt", "RelOriginAlt"])
    for cnum, cid, prm1 in dump or []:
        w.write("CMD", TimeUS=1_000_000, CTot=20, CNum=cnum, CId=cid, Prm1=prm1)
    for t, kind, cnum, cid, prm1 in events:
        if kind == "MISE":
            w.write("MISE", TimeUS=t, CTot=20, CNum=cnum, CId=cid, Prm1=prm1)
        else:
            w.write("MSG", TimeUS=t, Message=f"Mission: {cnum} SetCamTrigDst")
    w.write("POS", TimeUS=100_000_000, Lat=25.1, Lng=121.2, Alt=100)
    return dflog.read_dflog(w.save(tmp_path / "mission.BIN"))


def test_windows_from_msg_with_cmd_dump_and_dedupe(tmp_path):
    dump = [(3, 206, 18.85), (5, 206, 0.0), (8, 206, 20.0), (9, 206, 0.0)]
    log = _mission_log(tmp_path, [
        (10_000_000, "MSG", 3, None, None),                 # ON：只有文字（MISE 掉了）
        (20_000_000, "MISE", 5, 206, 0.0), (20_000_000, "MSG", 5, None, None),   # OFF：兩者重複 ⇒ 只算一次
        (30_000_000, "MISE", 8, 206, 20.0), (30_000_000, "MSG", 8, None, None),
        (40_000_000, "MSG", 9, None, None),
    ], dump=dump)
    ex = dflog.mission_executions(log)
    assert [(e.t_us, e.cnum, e.cid, e.prm1, e.source) for e in ex] == [
        (10_000_000, 3, 206, pytest.approx(18.85, abs=1e-5), "MSG"), (20_000_000, 5, 206, 0.0, "MISE"),
        (30_000_000, 8, 206, 20.0, "MISE"), (40_000_000, 9, 206, 0.0, "MSG")]
    w = dflog.strip_windows(log)
    assert [(x.strip, x.t_on_us, x.t_off_us, x.cnum_on, x.cnum_off) for x in w] == [
        (1, 10_000_000, 20_000_000, 3, 5), (2, 30_000_000, 40_000_000, 8, 9)]
    assert w[0].trigg_dist_m == pytest.approx(18.85, abs=1e-5) and w[1].trigg_dist_m == 20.0
    assert w[0].contains(10_000_000) and not w[0].contains(20_000_000)


def test_windows_unclosed_and_reopened(tmp_path):
    log = _mission_log(tmp_path, [
        (10_000_000, "MISE", 3, 206, 18.0), (15_000_000, "MISE", 4, 206, 19.0), (60_000_000, "MSG", 7, None, None),
    ], dump=[(7, 16, 0.0)])
    w = dflog.strip_windows(log)
    assert [(x.t_on_us, x.t_off_us, x.flags) for x in w] == [
        (10_000_000, 15_000_000, ("reopened",)), (15_000_000, 100_000_001, ("unclosed",))]
    assert w[1].trigg_dist_m == 19.0


def test_param_changes_and_gps_time_fit_on_synthetic(tmp_path):
    tr = SurveyTruth(n_strips=1, strip_len=200.0)
    info = write_survey_log(tmp_path / "s.BIN", tr, gps_lag_s=0.08, gpst_at_boot=1_444_000_000.0)
    log = dflog.read_dflog(tmp_path / "s.BIN")
    g = dflog.gps_time_fit(log)
    # TimeUS 是 epoch 之後 80 ms 才寫 ⇒ 不補延遲時 GPST 早 80 ms（評估 §14 D05 的偏差）
    t = 210_000_000
    assert g.gpst_s(t) == pytest.approx(1_444_000_000.0 + 210.0 - 0.08, abs=1e-4)
    g2 = dflog.gps_time_fit(log, lag_s=0.08)
    assert g2.gpst_s(t) == pytest.approx(1_444_000_000.0 + 210.0, abs=1e-4)
    assert log.params["GPS1_POS_X"] == 0.0 and log.param_changes == []
    assert len(info["cam_t_us"]) == log.count("CAM")


def test_param_change_tracking(tmp_path):
    w = DFWriter()
    w.define("PARM", "QNff", ["TimeUS", "Name", "Value", "Default"])
    for t, v in ((1, 0.0), (2, 0.0), (3, 18.85), (4, 18.85), (5, 0.0)):
        w.write("PARM", TimeUS=t, Name="CAM1_TRIGG_DIST", Value=v, Default=0.0)
    log = dflog.read_dflog(w.save(tmp_path / "p.BIN"))
    assert log.params["CAM1_TRIGG_DIST"] == 0.0
    assert [(t, v) for t, _, v in log.param_changes] == [(3, pytest.approx(18.85, abs=1e-5)), (5, 0.0)]
