"""survey 端到端：已知真值的合成航線 → .BIN ＋ 合成 Pi 會期 → 配對、擬合、位姿、QA、交付物。

注入：時鐘偏移 −523.25 s ＋ 漂移 +42 ppm ＋ 0.1 ms 抖動、挑幀延遲 0–100 ms、Pi 少 3 個邊緣、
飛控在轉彎中多 1 筆 CAM、Pi 多 1 個雜訊邊緣；飛控 GPS 天線偏移 GPS_POS1（舊參數名）＋相機槓桿臂；
CAM 用 ≤4.6 的 int16 百分度格式、GPA 用 Und（ArduPilot 正負號）。
真值位置用 pyproj／scipy 計算，與 survey 的換算互相獨立。
"""

import math

import numpy as np
import pytest
from pyproj import Geod

from survey import datum, deliverable, exif_write, geotag, ingest, qa, synth
from tests.test_survey_synthlog import CAM_FMT_CENTIDEG, SurveyTruth, strip_intervals, write_survey_log

GEOD = Geod(ellps="WGS84")
A_TRUE, B_TRUE = 1 + 42e-6, -523.25
LEVER = (0.15, -0.05, 0.10)
GPS_POS = (0.30, 0.0, -0.20)
N_SITE = 18.54            # 本場址 EGM96 大地起伏（評估 §8）—— 只用在測試
GPS_LAG = 0.08
CAMERA = {"sensor": "imx477", "lens_mm": 6.0, "pixel_um": 1.55, "width": 64, "height": 48, "readout_ms": 86.9,
          "shutter": "rolling", "kernel": "6.6.y", "dtoverlay": "imx477"}


@pytest.fixture(scope="module")
def sortie(tmp_path_factory):
    d = tmp_path_factory.mktemp("e2e")
    tr = SurveyTruth(n_strips=3, strip_len=500.0, spacing=120.0)
    turn_t = strip_intervals(tr)[0][2] + 2.0
    info = write_survey_log(d / "sortie.BIN", tr, cam_fmt=CAM_FMT_CENTIDEG, gpa_field="Und", undulation_n=N_SITE,
                            gps_pos=GPS_POS, gps_lag_s=GPS_LAG, extra_cam_times=[turn_t], new_param_names=False)
    cam_t = info["cam_t_us"]
    extra_idx = int(np.argmin(np.abs(np.asarray(cam_t) * 1e-6 - turn_t)))
    drops = [5, 33, 61]
    assert extra_idx not in drops
    spurious_fc_s = cam_t[12] * 1e-6 + 0.45
    truth = synth.synth_session_from_fc(
        d / "sess", cam_t, uav="hero2", session_id="20261014T0300Z", unit_model="RPiHQ6-H2", camera=CAMERA,
        lever_arm_m=LEVER, clock_a=A_TRUE, clock_b_s=B_TRUE, boottime_minus_monotonic_ns=1_500_000_000,
        jitter_s=1e-4, frame_period_s=0.1, exposure_us=800, drop_index=drops + [extra_idx],
        extra_edges_fc_s=[spurious_fc_s], seed=7)
    res = ingest.ingest_sortie(d / "sortie.BIN", d / "sess", sensor_ts_semantics="exposure_start",
                               geoid=datum.GeoidModel("EGM96", N_SITE, "test constant"), gps_lag_s=GPS_LAG)
    return dict(dir=d, truth=tr, info=info, cam_t=cam_t, extra_idx=extra_idx, drops=drops, pi=truth, res=res)


def test_fit_recovers_offset_and_drift(sortie):
    fit = sortie["res"].fit
    # 90 s 架次、0.1 ms 抖動、~80 對：斜率的 1σ 約 0.4 ppm
    assert fit.drift_ppm == pytest.approx(42.0, abs=2.0)
    t_pi = np.asarray([f["edge_boot_s"] for f in sortie["pi"]["frames"]])
    err = fit.pi_to_fc(t_pi) - (A_TRUE * t_pi + B_TRUE)
    assert np.max(np.abs(err)) < 1e-4                      # 整個架次 < 0.1 ms
    assert fit.rms_s < 0.5e-3 and fit.rms_s == pytest.approx(1e-4, rel=0.4)


def test_unmatched_events_reported_exactly(sortie):
    fit = sortie["res"].fit
    assert fit.unmatched_fc.tolist() == sorted(sortie["drops"] + [sortie["extra_idx"]])
    spurious = [k for k, f in enumerate(sortie["pi"]["frames"]) if f["fc_index"] is None]
    assert len(spurious) == 1 and fit.unmatched_pi.tolist() == spurious
    # 每一對都是真正同一個邊緣（不是依序硬配）
    for fc_i, pi_j in fit.pairs:
        assert sortie["pi"]["frames"][int(pi_j)]["fc_index"] == int(fc_i)
    assert fit.n_pairs == len(sortie["cam_t"]) - 4


def test_exposure_positions_within_5cm_of_truth(sortie):
    tr = sortie["truth"]
    rows = {r["seq"]: r for r in sortie["res"].poses.rows}
    offset = np.add(GPS_POS, LEVER)
    errs = []
    for f in sortie["pi"]["frames"]:
        r = rows[f["seq"]]
        lat, lon, h = tr.point_llh(f["center_fc_s"], offset)
        _, _, dist = GEOD.inv(lon, lat, r["lon_deg"], r["lat_deg"])
        errs.append(math.hypot(dist, r["h_orth_m"] - h))
    errs = np.asarray(errs)
    assert len(errs) == len(sortie["pi"]["frames"])
    assert np.sqrt(np.mean(errs ** 2)) < 0.05              # 規格：5 cm RMS
    # 實測約 4 mm RMS（0.1 ms 抖動 × 21.5 m/s）；1 cm 上限讓 cm 級的退化（槓桿臂、曝光語意）看得到
    assert errs.max() < 0.01
    # 挑幀延遲 0–100 ms：若直接用 CAM 的經緯度，誤差是公尺級（這正是不用它的理由）
    lat_c = sortie["res"].fc_events["Lat"]
    lon_c = sortie["res"].fc_events["Lng"]
    cam_err = [GEOD.inv(lon_c[fc], lat_c[fc], rows[sortie["res"].session.frames[pi].seq]["lon_deg"],
                        rows[sortie["res"].session.frames[pi].seq]["lat_deg"])[2]
               for fc, pi in sortie["res"].fit.pairs]
    assert np.median(cam_err) > 0.5


def test_heading_course_speed_and_time(sortie):
    tr = sortie["truth"]
    for f in sortie["pi"]["frames"]:
        r = next(x for x in sortie["res"].poses.rows if x["seq"] == f["seq"])
        st = tr.state(f["center_fc_s"])
        assert r["yaw_deg"] == pytest.approx(st["yaw"], abs=0.02)            # 真北航向（機身）
        assert r["cog_deg"] == pytest.approx(st["track"], abs=0.02)          # 對地航跡
        assert ((r["yaw_deg"] - r["cog_deg"] + 180) % 360 - 180) == pytest.approx(5.0, abs=0.05)
        assert r["speed_n_mps"] == pytest.approx(st["vN"], abs=0.01)
        assert r["speed_e_mps"] == pytest.approx(st["vE"], abs=0.01)
        # GPST = 開機起算時間 + gpst_at_boot（已補 GPS 處理延遲）
        assert r["t_gpst_s"] == pytest.approx(sortie["info"]["gpst_at_boot"] + f["center_fc_s"], abs=1e-3)


def test_heights_carry_datum(sortie):
    tr = sortie["truth"]
    r = sortie["res"].poses.rows[10]
    f = next(x for x in sortie["pi"]["frames"] if x["seq"] == r["seq"])
    _, _, h_true = tr.point_llh(f["center_fc_s"], np.add(GPS_POS, LEVER))   # 相機中心的 AMSL
    assert r["crs_v"] == "EGM96"
    assert r["h_orth_m"] == pytest.approx(h_true, abs=0.002)
    assert r["h_ell_m"] == pytest.approx(h_true + N_SITE, abs=0.002)     # 橢球高 = 正高 + N（台灣 N>0）
    eff = sortie["res"].poses.effective
    assert eff["receiver_undulation_source"] == "GPA.Und"
    assert eff["receiver_undulation_m"] == pytest.approx(N_SITE, abs=1e-4)


def test_effective_values_exposed(sortie):
    eff = sortie["res"].poses.effective
    assert eff["position_source"] == "POS"
    assert eff["gps_antenna_param"] == "GPS_POS1"
    assert eff["lever_arm_effective_m"] == pytest.approx(list(np.add(GPS_POS, LEVER)))
    assert eff["readout_ms"] == 86.9 and eff["readout_source"] == "session"
    assert eff["sensor_ts_semantics"] == "exposure_start"
    assert eff["gps_time_fit"]["lag_s"] == GPS_LAG
    assert sortie["res"].log.angle_encoding["CAM"] == "int16_centideg"


def test_flags_and_strips(sortie):
    rows = sortie["res"].poses.rows
    by_seq = {r["seq"]: r for r in rows}
    spurious_seq = next(f["seq"] for f in sortie["pi"]["frames"] if f["fc_index"] is None)
    assert "unpaired_edge" in by_seq[spurious_seq]["flags"].split(";")
    paired = [r for r in rows if r["seq"] != spurious_seq]
    assert all("unpaired_edge" not in r["flags"] for r in paired)
    assert sorted({r["strip"] for r in rows}) == [1, 2, 3]
    assert not any("off_strip" in r["flags"] for r in rows)
    assert not any("readout_unknown" in r["flags"] for r in rows)


def test_qa_conservation_and_report(sortie):
    rep = sortie["res"].qa
    cons = {c["strip"]: c for c in rep["conservation"]}
    fc_per_strip = {}
    for k, ton, toff in sortie["info"]["windows"]:
        fc_per_strip[k] = sum(1 for t in sortie["cam_t"] if ton <= t < toff)
    for k in (1, 2, 3):
        n_drop = sum(1 for i in sortie["drops"] if sortie["info"]["windows"][k - 1][1] <= sortie["cam_t"][i]
                     < sortie["info"]["windows"][k - 1][2])
        extra = 1 if k == 1 else 0
        assert cons[k]["fc_cam"] == fc_per_strip[k]
        assert cons[k]["pi_edges"] == fc_per_strip[k] - n_drop + extra
        assert cons[k]["jpeg_ok"] == cons[k]["pi_edges"]
        assert (cons[k]["unmatched_fc"], cons[k]["unmatched_pi"]) == (n_drop, extra)
        # 第 1 帶：少 1 個邊緣＋多 1 個雜訊邊緣，張數剛好相等 —— 仍必須判為不守恆
        assert cons[k]["conserved"] is (n_drop == 0 and extra == 0)
    assert cons[1]["pi_edges"] == cons[1]["fc_cam"]
    assert cons[None]["fc_cam"] == 1 and cons[None]["pi_edges"] == 0 and cons[None]["conserved"] is False
    assert [u["index"] for u in rep["unmatched_fc"]] == sorted(sortie["drops"] + [sortie["extra_idx"]])
    assert rep["passed"] is False
    for s in rep["spacing"]:
        assert s["median_ok"] and s["p99_ok"]
        assert s["median_m"] == pytest.approx(18.85, rel=0.03)
    assert rep["fit"]["rms_ok"] is True


def test_deliverable_geo_txt_and_exif(sortie):
    d = sortie["dir"]
    rows = sortie["res"].poses.rows
    paths = deliverable.write_i1_deliverable(d / "i1", rows, meta=sortie["res"].poses.effective)
    back = deliverable.read_photos_csv(paths["csv"])
    assert len(back) == len(rows)
    n = geotag.write_geo_txt(d / "odm" / "geo.txt", rows, height="ellipsoidal")
    lines = (d / "odm" / "geo.txt").read_text(encoding="utf-8").splitlines()
    assert n == len(rows) and lines[0] == "EPSG:4326" and len(lines) == len(rows) + 1
    written = exif_write.export_tagged_images(sortie["res"].session, rows, d / "odm" / "images",
                                              make="Raspberry Pi", height="ellipsoidal")
    assert len(written) == len(rows)
    assert {p.name for p in written} == {ln.split()[0] for ln in lines[1:]}


def test_multiplicity_grid_on_strips(sortie):
    rows = sortie["res"].poses.rows
    grid = qa.footprint_multiplicity(rows, focal_mm=6.0, pixel_um=1.55, width_px=4056, height_px=3040,
                                     ground_h_m=sortie["truth"].h - 120.0, height="orthometric", cell_m=2.0)
    assert grid.n_footprints == len(rows) and grid.n_skipped == 0
    # 帶中央：沿航跡足跡 94.24 m ÷ 間距 18.85 m ≈ 5 張
    tr = sortie["truth"]
    strip1 = strip_intervals(tr)[0]
    lat, lon, _ = tr.point_llh((strip1[1] + strip1[2]) / 2)
    m, nr = datum.wgs84_radii(grid.lat0)
    i = int(math.radians(lat - grid.lat0) * m / grid.cell_m)
    j = int(math.radians(lon - grid.lon0) * nr * math.cos(math.radians(grid.lat0)) / grid.cell_m)
    assert grid.counts[i, j] in (4, 5, 6)
    assert grid.counts.max() <= 12
