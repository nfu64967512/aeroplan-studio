"""survey.pose：曝光中心時刻、槓桿臂旋轉、來源選擇、旗標與有效值。"""

import logging

import numpy as np
import pytest
from pyproj import Geod
from scipy.spatial.transform import Rotation

from survey import dflog, ingest, pi_session, pose, synth
from survey.pose import body_to_ned, exposure_center_ns
from tests.test_survey_synthlog import SurveyTruth, write_survey_log

GEOD = Geod(ellps="WGS84")
CAMERA = {"sensor": "imx477", "lens_mm": 6.0, "pixel_um": 1.55, "width": 64, "height": 48, "readout_ms": 86.9,
          "shutter": "rolling", "kernel": "6.6.y", "dtoverlay": "imx477"}


def test_exposure_center_semantics():
    ts = 1_000_000_000_000
    # libcamera：ts = 第一列開始曝光 ⇒ +E/2 +R/2
    assert exposure_center_ns(ts, 1000, 86.9, "exposure_start") == (ts + 500_000 + 43_450_000, [])
    # picamera2：ts = 第一個像素讀出 ⇒ −E/2 +R/2
    assert exposure_center_ns(ts, 1000, 86.9, "readout_start") == (ts - 500_000 + 43_450_000, [])
    assert exposure_center_ns(ts, 1000, None, "exposure_start") == (ts + 500_000, ["readout_unknown"])
    assert exposure_center_ns(ts, None, 10.0, "exposure_start") == (ts + 5_000_000, ["exposure_unknown"])
    with pytest.raises(ValueError, match="sensor_ts_semantics"):
        exposure_center_ns(ts, 1000, 10.0, "middle")


def test_body_to_ned_matches_scipy():
    rng = np.random.default_rng(0)
    v = (0.3, -0.2, 0.5)
    for _ in range(20):
        r, p, y = rng.uniform(-40, 40), rng.uniform(-20, 20), rng.uniform(0, 360)
        exp = Rotation.from_euler("ZYX", [y, p, r], degrees=True).apply(v)
        np.testing.assert_allclose(body_to_ned(r, p, y, v), exp, atol=1e-12)
    # 機頭向東、無姿態：機體 +X（前）= 東，+Y（右）= 南
    np.testing.assert_allclose(body_to_ned(0, 0, 90, (1.0, 0, 0)), (0, 1, 0), atol=1e-12)
    np.testing.assert_allclose(body_to_ned(0, 0, 90, (0, 1.0, 0)), (-1, 0, 0), atol=1e-12)


def test_image_id_is_filename_safe_and_unique():
    assert pose.image_id("hero 2/x", "2026-10-14T03:00Z", 7) == "hero-2-x_2026-10-14T03-00Z_000007"
    assert pose.image_id("a", "s", 1) != pose.image_id("b", "s", 1)


@pytest.fixture(scope="module")
def base(tmp_path_factory):
    d = tmp_path_factory.mktemp("pose")
    tr = SurveyTruth(n_strips=1, strip_len=300.0)
    info = write_survey_log(d / "a.BIN", tr, gps_pos=(0.3, 0.0, -0.2))
    write_survey_log(d / "nogpspos.BIN", tr, gps_pos=None)
    synth.synth_session_from_fc(d / "s", info["cam_t_us"], clock_a=1.0, clock_b_s=-100.0, camera=CAMERA,
                                lever_arm_m=(0.1, 0.0, 0.05), exposure_us=800, no_frame_index=[2], seed=5)
    cam_null = dict(CAMERA, readout_ms=None)
    truth_null = synth.synth_session_from_fc(d / "s_null", info["cam_t_us"], clock_a=1.0, clock_b_s=-100.0,
                                             camera=cam_null, lever_arm_m=(0.1, 0.0, 0.05), exposure_us=800, seed=5)
    log = dflog.read_dflog(d / "a.BIN")
    sess = pi_session.read_pi_session(d / "s")
    fit, fc = ingest.fit_session(log, sess)
    return dict(d=d, tr=tr, log=log, sess=sess, fit=fit, fc=fc, truth_null=truth_null)


def _poses(b, **kw):
    kw.setdefault("sensor_ts_semantics", "exposure_start")
    return pose.compute_photo_poses(b["log"], b["sess"], b["fit"], b["fc"], **kw)


def test_semantics_parameter_is_required_and_validated(base):
    with pytest.raises(TypeError):
        pose.compute_photo_poses(base["log"], base["sess"], base["fit"], base["fc"])
    with pytest.raises(ValueError, match="sensor_ts_semantics"):
        _poses(base, sensor_ts_semantics="center")


def test_semantics_shift_is_exposure_time(base):
    a = {r["seq"]: r for r in _poses(base).rows}
    b = {r["seq"]: r for r in _poses(base, sensor_ts_semantics="readout_start").rows}
    for s in a:
        assert a[s]["t_fc_us"] - b[s]["t_fc_us"] == 800            # 差一個曝光時間（800 µs）
        d = GEOD.inv(a[s]["lon_deg"], a[s]["lat_deg"], b[s]["lon_deg"], b[s]["lat_deg"])[2]
        assert d == pytest.approx(21.5 * 800e-6, abs=2e-3)


def test_readout_unknown_flag_and_override(base, caplog):
    b = dict(base)
    b["sess"] = pi_session.read_pi_session(base["d"] / "s_null")
    b["fit"], b["fc"] = ingest.fit_session(base["log"], b["sess"])
    res = _poses(b)
    assert "readout_unknown" in res.flags and res.effective["readout_source"] == "unknown"
    assert res.effective["readout_ms"] is None
    assert all("readout_unknown" in r["flags"].split(";") for r in res.rows)
    known = {r["seq"]: r for r in _poses(base).rows}
    common = [r for r in res.rows if r["seq"] in known]
    assert len(common) == len(res.rows) - 1                  # base 的 seq 3 是 no_frame
    for r in common:     # 同一組 SensorTimestamp；沒有讀出時間 ⇒ 曝光中心早 R/2 = 43.45 ms
        assert known[r["seq"]]["t_fc_us"] - r["t_fc_us"] == pytest.approx(43450, abs=1)
    with caplog.at_level(logging.WARNING, logger="survey.pose"):
        over = _poses(base, readout_ms=70.0)
    assert over.effective["readout_ms"] == 70.0 and over.effective["readout_source"] == "override"
    assert any("覆寫" in m for m in caplog.messages)


def test_position_sources_agree_and_lever_arm_composition(base):
    pos = _poses(base, position_source="POS")
    xkf = _poses(base, position_source="XKF1")
    assert pos.effective["position_source"] == "POS" and xkf.effective["position_source"] == "XKF1[C=0]"
    for a, b in zip(pos.rows, xkf.rows):
        assert GEOD.inv(a["lon_deg"], a["lat_deg"], b["lon_deg"], b["lat_deg"])[2] < 0.005
    # EKF 輸出是 IMU 位置 ⇒ 有效槓桿臂 = 相機−天線 + 天線−IMU
    assert pos.effective["lever_arm_effective_m"] == pytest.approx([0.4, 0.0, -0.15])
    assert pos.effective["gps_antenna_param"] == "GPS1_POS"
    gps = _poses(base, position_source="GPS")
    assert "pos_from_gps" in gps.flags
    assert gps.effective["lever_arm_effective_m"] == pytest.approx([0.1, 0.0, 0.05])   # 天線本身 ⇒ 不加天線偏移
    with pytest.raises(ValueError):
        _poses(base, position_source="AHR2")


def test_missing_antenna_param_is_flagged(base, tmp_path):
    log = dflog.read_dflog(base["d"] / "nogpspos.BIN")
    res = pose.compute_photo_poses(log, base["sess"], base["fit"], base["fc"], sensor_ts_semantics="exposure_start")
    assert "gps_pos_param_missing" in res.flags
    assert res.effective["gps_antenna_offset_m"] is None
    assert res.effective["lever_arm_effective_m"] == pytest.approx([0.1, 0.0, 0.05])


def test_skipped_frames_have_reasons(base):
    res = _poses(base)
    assert {"seq": 3, "reason": "status:no_frame"} in res.skipped
    assert len(res.rows) + len(res.skipped) == len(base["sess"].frames)
    r0 = res.rows[0]
    assert r0["image_id"] == "uav1_S1_000001" and r0["source"] == "fc_interp" and r0["crs_h"] == "EPSG:4326"
    assert r0["strip"] == 1 and r0["sigma_h_m"] == 0.5 and r0["sigma_v_m"] == 0.8
    # 未提供大地起伏模型 ⇒ 正高留空，只有橢球高（GPA.AEl 給的）
    assert r0["h_orth_m"] is None and r0["crs_v"] == "ellipsoidal"
    assert r0["h_ell_m"] == pytest.approx(150.0 + 18.54, abs=0.5)
