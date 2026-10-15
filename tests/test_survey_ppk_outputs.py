"""survey/ppk/outputs.py —— photos.csv（F2）相容列、fix 比例報告，以及整條鏈的端到端合成測試。"""
import csv

import numpy as np
import pytest

from survey.ppk.events import PairedEvent, PairingResult, ClockFit, extract_edges, pair_with_frames
from survey.ppk.interp import interpolate_at
from survey.ppk.outputs import (PHOTOS_CSV_FIELDS, fix_ratio_report, fixed_windows, photo_events,
                                ppk_photo_rows, strip_windows, write_photos_csv)
from survey.ppk.pos import parse_pos
from survey.ppk.ubx import parse_ubx
from tests.test_survey_ppk_events import T0_NS, _scenario
from tests.test_survey_ppk_pos import pos_text

# F2 規格原文的欄位順序（獨立打一次，不從被測模組取）
_F2 = ("image_id,file,uav,strip,seq,t_fc_us,t_gpst_s,lat_deg,lon_deg,h_ell_m,h_orth_m,yaw_deg,"
       "pitch_deg,roll_deg,cog_deg,speed_n_mps,speed_e_mps,speed_d_mps,sigma_h_m,sigma_v_m,source,"
       "crs_h,crs_v,flags").split(",")

T0 = 2389 * 604800 + 180000.0


def _m_per_deg_lon(lat=25.1, h=150.0):
    """WGS84 緯圈上每度經度的長度：(N+h)·cosφ·π/180。"""
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    n = a / np.sqrt(1 - e2 * np.sin(np.radians(lat)) ** 2)
    return (n + h) * np.cos(np.radians(lat)) * np.pi / 180


def _east_track(t0, n, q=None):
    """沿 25.1N 緯圈向東 20 m/s 直飛、150 m 橢球高、10 Hz。"""
    t = t0 + 0.1 * np.arange(n)
    lon = 121.2 + 20.0 * (t - t0) / _m_per_deg_lon()
    lat = np.full(n, 25.1)
    h = np.full(n, 150.0)
    return parse_pos(pos_text(t, lat, lon, h, q=q))


def _ev(seq, t, flags=(), status="ok", file=None):
    return PairedEvent(seq=seq, edge_ns=seq * 10**9, tm2_count=100 + seq, t_edge_gpst_s=t,
                       t_exposure_gpst_s=t, residual_s=0.0, acc_est_ns=20, flags=tuple(flags),
                       file=file or f"images/{seq:06d}.jpg", status=status)


def _pairing(events):
    fit = ClockFit(1.0, 0.0, 0, 0, len(events), 0.0, 0.0)
    return PairingResult(events=events, fit=fit, unmatched_tm2=[], unmatched_pi_seq=[])


def test_photos_csv_field_list_is_f2():
    assert list(PHOTOS_CSV_FIELDS) == _F2


def test_photo_events_keeps_only_real_photos():
    evs = [_ev(1, T0 + 1), _ev(2, T0 + 2, status="no_frame"), _ev(3, T0 + 3, status=None),
           PairedEvent(4, 4, None, T0 + 4, None, None, None, ("no_exposure_offset",), "x.jpg", "ok"),
           _ev(5, T0 + 5, status="write_error")]
    keep, skipped = photo_events(_pairing(evs))
    assert [e.seq for e in keep] == [1, 3]
    assert skipped == [(2, "status_no_frame"), (4, "no_exposure_time"), (5, "status_write_error")]


def test_rows_values_and_csv(tmp_path):
    tr = _east_track(T0, 100)
    evs = [_ev(1, T0 + 1.05, flags=("tm2_utc_base",)), _ev(2, T0 + 2.0), _ev(3, T0 + 3.25)]
    att = [[90.0, 2.0, -1.0], [91.0, 0.0, 0.0], [92.0, 1.0, 1.0]]
    res = interpolate_at(tr, [e.t_exposure_gpst_s for e in evs], lever_arm_frd_m=(0.3, 0.0, 0.0),
                         attitude_deg=att)
    rows = ppk_photo_rows(evs, res, crs_h="e-GNSS[2025]", uav="HERO1",
                          strip_of_seq={1: 4, 2: 4}, image_id_of_seq={1: "HERO1_000001"},
                          attitude_deg=att)
    assert [list(r) for r in rows] == [_F2] * 3
    r0 = rows[0]
    assert r0["image_id"] == "HERO1_000001" and rows[1]["image_id"] == ""
    assert (r0["file"], r0["uav"], r0["strip"], r0["seq"]) == ("images/000001.jpg", "HERO1", 4, 1)
    assert rows[2]["strip"] == ""
    assert r0["t_gpst_s"] == pytest.approx(T0 + 1.05)
    assert (r0["lat_deg"], r0["lon_deg"], r0["h_ell_m"]) == (res.lat_deg[0], res.lon_deg[0], res.h_ell_m[0])
    assert r0["h_orth_m"] is None and r0["t_fc_us"] is None
    assert (r0["yaw_deg"], r0["pitch_deg"], r0["roll_deg"]) == (90.0, 2.0, -1.0)
    assert r0["cog_deg"] == pytest.approx(90.0, abs=0.01)
    assert (r0["speed_n_mps"], r0["speed_e_mps"], r0["speed_d_mps"]) == pytest.approx((0, 20, 0), abs=1e-3)
    assert r0["sigma_h_m"] == pytest.approx(0.01, abs=1e-6)
    assert (r0["source"], r0["crs_h"], r0["crs_v"]) == ("ppk", "e-GNSS[2025]", "ellipsoidal")
    assert r0["flags"] == "tm2_utc_base"
    # CSV
    p = write_photos_csv(rows, tmp_path / "photos_ppk.csv")
    with open(p, encoding="utf-8", newline="") as fh:
        got = list(csv.reader(fh))
    assert got[0] == _F2 and len(got) == 4
    d = dict(zip(_F2, got[1]))
    assert d["lat_deg"] == f"{res.lat_deg[0]:.9f}" and len(d["lat_deg"].split(".")[1]) == 9
    assert d["h_ell_m"] == f"{res.h_ell_m[0]:.4f}" and d["t_gpst_s"] == f"{T0 + 1.05:.6f}"
    assert d["h_orth_m"] == "" and d["t_fc_us"] == "" and d["seq"] == "1" and d["strip"] == "4"
    assert d["crs_h"] == "e-GNSS[2025]" and d["flags"] == "tm2_utc_base"


def test_rows_flags_merge_and_nan_blank(tmp_path):
    q = np.ones(100, int)
    q[30] = 2
    tr = _east_track(T0, 100, q=q)
    evs = [_ev(1, T0 + 2.95, flags=("tm2_missing", "fit_extrapolated")), _ev(2, T0 + 50.0)]
    res = interpolate_at(tr, [e.t_exposure_gpst_s for e in evs])
    rows = ppk_photo_rows(evs, res, crs_h="own-base-ITRF2020@2026.8")
    assert rows[0]["flags"] == "tm2_missing;fit_extrapolated;ppk_float;lever_arm_unknown"
    assert rows[1]["flags"] == "ppk_out_of_range;lever_arm_unknown"
    assert rows[1]["lat_deg"] is None and rows[1]["sigma_h_m"] is None
    assert rows[0]["yaw_deg"] is None
    p = write_photos_csv(rows, tmp_path / "x.csv")
    line = dict(zip(_F2, list(csv.reader(open(p, encoding="utf-8")))[2]))
    assert line["lat_deg"] == "" and line["crs_h"] == "own-base-ITRF2020@2026.8"


def test_rows_argument_checks():
    tr = _east_track(T0, 50)
    evs = [_ev(1, T0 + 1.0)]
    res = interpolate_at(tr, [T0 + 1.0])
    for bad in ("", "   ", None):
        with pytest.raises(ValueError, match="crs_h"):
            ppk_photo_rows(evs, res, crs_h=bad)
    with pytest.raises(ValueError, match="不符"):
        ppk_photo_rows(evs + evs, res, crs_h="e-GNSS[2025]")
    with pytest.raises(ValueError, match="不一致"):
        ppk_photo_rows([_ev(1, T0 + 1.5)], res, crs_h="e-GNSS[2025]")
    with pytest.raises(ValueError, match="姿態"):
        ppk_photo_rows(evs, res, crs_h="x", attitude_deg=[[0, 0, 0], [0, 0, 0]])


# ---------------------------------------------------------------------------
# fix 比例
# ---------------------------------------------------------------------------
def test_fix_ratio_report_numbers():
    q = np.ones(200, int)
    q[120:137] = 2          # 第二個視窗 (10–19.95 s) 有 17 個 float、2 個 single
    q[140:142] = 5
    tr = _east_track(T0, 200, q=q)
    wins = [("S1", T0 + 0.0, T0 + 9.95), ("S2", T0 + 10.0, T0 + 19.95), ("S3", T0 + 30.0, T0 + 40.0)]
    pt = [T0 + 1.0, T0 + 5.0, T0 + 12.0, T0 + 13.0, T0 + 35.0]
    pf = [(), ("ppk_float",), ("ppk_float", "lever_arm_unknown"), ("ppk_gap",), ("ppk_out_of_range",)]
    rep = fix_ratio_report(tr, wins, photo_t=pt, photo_flags=pf)
    a, b, c = rep
    assert (a.label, a.n_epochs, a.n_fix, a.n_float, a.n_other, a.fix_ratio) == ("S1", 100, 100, 0, 0, 1.0)
    assert not a.below_threshold and (a.n_photos, a.n_photos_not_fixed) == (2, 1)
    assert (b.n_epochs, b.n_fix, b.n_float, b.n_other) == (100, 81, 17, 2)
    assert b.fix_ratio == pytest.approx(0.81) and b.below_threshold
    assert (b.n_photos, b.n_photos_not_fixed) == (2, 2)
    assert c.n_epochs == 0 and c.fix_ratio is None and c.below_threshold and c.n_photos == 1
    # 門檻是參數：0.8 時 S2 合格
    assert not fix_ratio_report(tr, wins[1:2], threshold=0.8)[0].below_threshold
    with pytest.raises(ValueError):
        fix_ratio_report(tr, wins, threshold=0)


def test_strip_and_fixed_windows():
    t = [5.0, 1.0, 3.0, 9.0, 7.0, 2.0]
    s = ["2", "1", "1", "3", "2", ""]
    assert strip_windows(t, s) == [("1", 1.0, 3.0), ("2", 5.0, 7.0), ("3", 9.0, 9.0)]
    assert fixed_windows(0.0, 10.0, 4.0) == [("0.0", 0.0, 4.0), ("4.0", 4.0, 8.0), ("8.0", 8.0, 10.0)]
    with pytest.raises(ValueError):
        fixed_windows(0, 1, 0)


# ---------------------------------------------------------------------------
# 端到端：TM2＋Pi 會期＋PPK 軌跡 → 每張照片的相機中心
# ---------------------------------------------------------------------------
def test_end_to_end_synthetic_chain():
    sc = _scenario(seed=11)
    ex = extract_edges(parse_ubx(sc["data"]))
    offs = [0.0375 + 0.0002 * (f["seq"] % 7) for f in sc["frames"]]   # 曝光中心相對邊緣（Pi 秒）
    pr = pair_with_frames(ex, sc["frames"], exposure_offset_s=offs)
    keep, skipped = photo_events(pr)
    assert skipped == []
    t_start = T0_NS / 1e9
    tr = _east_track(t_start - 1.0, 5000)                 # 涵蓋整個 session（500 s）
    att = np.tile([90.0, 0.0, 0.0], (len(keep), 1))
    res = interpolate_at(tr, [e.t_exposure_gpst_s for e in keep], lever_arm_frd_m=(0.3, 0.0, 0.0),
                         attitude_deg=att)
    rows = ppk_photo_rows(keep, res, crs_h="e-GNSS[2025]", uav="HERO1", attitude_deg=att)
    # 真值：天線在曝光真時刻的位置 + 東 0.3 m
    m_per_deg_lon = _m_per_deg_lon()
    off_of = {f["seq"]: o for f, o in zip(sc["frames"], offs)}
    for r in rows:
        t_true = sc["edges"][sc["truth"][r["seq"]]] / 1e9 + off_of[r["seq"]] * (1 - 37e-6)
        lon_true = 121.2 + (20.0 * (t_true - (t_start - 1.0)) + 0.3) / m_per_deg_lon
        err_m = abs(r["lon_deg"] - lon_true) * m_per_deg_lon
        assert err_m < (2e-3 if "tm2_missing" in r["flags"] else 5e-4), (r["seq"], err_m)
        assert r["lat_deg"] == pytest.approx(25.1, abs=1e-8)
    assert sum("tm2_missing" in r["flags"] for r in rows) == 1
    assert len(rows) == len(sc["frames"])
