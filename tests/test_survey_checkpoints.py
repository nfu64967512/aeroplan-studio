"""survey.checkpoints：檢核點精度報告 —— 手算例子、DSM 取值（非 WGS84 的 GeoTIFF）、基準不混用。

真值的位移用 pyproj.Geod（與 survey.datum 的局部換算互相獨立）；DSM 用平面 z = a + b·X + c·Y 的合成
GeoTIFF（雙線性內插對平面是精確的 ⇒ 取到的值可以逐位比對到 1e-6 m）。
"""
from __future__ import annotations

import json
import math

import numpy as np
import pytest
from pyproj import CRS, Geod, Transformer

from survey import checkpoints as ck
from survey import datum
from survey import gcp_plan as gp
from survey.checkpoints import CheckpointError

GEOD = Geod(ellps="WGS84")
LAT0, LON0 = 25.07, 121.21
HEADER = "id,type,lat,lon,h,frame,vertical_datum,sigma_h_m,sigma_v_m"


def moved(lat, lon, de, dn):
    lo, la, _ = GEOD.fwd(lon, lat, 90.0, de) if de else (lon, lat, 0)
    lo, la, _ = GEOD.fwd(lo, la, 0.0, dn) if dn else (lo, la, 0)
    return la, lo


def write_csv(path, header, rows):
    path.write_text(header + "\n" + "\n".join(",".join(str(v) for v in r) for r in rows) + "\n", encoding="utf-8")
    return path


# 手算例子（cm）：dE, dN, dZ
ERR = {"C1": (3, -4, 5), "C2": (-3, 4, -1), "C3": (0, 0, 2), "C4": (6, 8, 2)}
POS = {"C1": (0, 0), "C2": (300, 0), "C3": (0, 300), "C4": (300, 300)}


def hand_example(tmp_path, *, sigma_h=0.01, sigma_v=0.02, frame="EPSG:4326", vd="ellipsoidal"):
    srows, mrows = [], []
    for pid, (x, y) in POS.items():
        lat, lon = moved(LAT0, LON0, x, y)
        h = 50.0 + x * 0.01
        srows.append((pid, "CHK", f"{lat:.10f}", f"{lon:.10f}", h, frame, vd, sigma_h, sigma_v))
        de, dn, dz = (v / 100.0 for v in ERR[pid])
        mlat, mlon = moved(lat, lon, de, dn)
        mrows.append((pid, f"{mlat:.11f}", f"{mlon:.11f}", frame, h + dz, vd))
    s = write_csv(tmp_path / "survey.csv", HEADER, srows)
    m = write_csv(tmp_path / "measured.csv", "id,lat,lon,frame,h,vertical_datum", mrows)
    return s, m


def test_rmse_bias_max_and_rss_against_a_hand_example(tmp_path):
    s, m = hand_example(tmp_path)
    rep = ck.accuracy_report(ck.read_surveyed_csv(s), measured=ck.read_measured_csv(m))
    h, v = rep["horizontal"], rep["vertical"]
    assert h["n"] == 4 and v["n"] == 4
    assert h["rmse_e_m"] == pytest.approx(math.sqrt(13.5) / 100, abs=2e-5)
    assert h["rmse_n_m"] == pytest.approx(math.sqrt(24.0) / 100, abs=2e-5)
    assert h["rmse_h_m"] == pytest.approx(math.sqrt(37.5) / 100, abs=2e-5)
    assert (h["mean_e_m"], h["mean_n_m"]) == pytest.approx((0.015, 0.020), abs=2e-5)
    assert h["max_h_m"] == pytest.approx(0.10, abs=2e-5)
    assert h["survey_rmse_h_m"] == pytest.approx(0.01)
    assert h["combined_rmse_h_m"] == pytest.approx(math.sqrt(37.5 + 1.0) / 100, abs=2e-5)    # RSS
    assert v["rmse_v_m"] == pytest.approx(math.sqrt(8.5) / 100, abs=1e-9)
    assert v["mean_v_m"] == pytest.approx(0.02, abs=1e-9) and v["std_v_m"] == pytest.approx(math.sqrt(6) / 100)
    assert v["max_abs_v_m"] == pytest.approx(0.05) and v["survey_rmse_v_m"] == pytest.approx(0.02)
    assert v["combined_rmse_v_m"] == pytest.approx(math.sqrt(12.5) / 100, abs=1e-9)
    a = rep["asprs"]
    assert a["meets_min"] is False and a["statement"].startswith("小專案聲明") and "n = 4" in a["statement"]
    # 沒有目標精度 ⇒ 以實測 RMSE 比：水平 1 cm vs 6.1 cm（✔）、垂直 2 cm vs 2.9 cm（✘ 不到 2 倍）
    assert a["survey_vs_product_h"]["ok"] is True and a["survey_vs_product_h"]["basis"].startswith("實測")
    assert a["survey_vs_product_v"]["ok"] is False
    assert any("垂直" in w and "2 倍" in w for w in rep["warnings"])
    lines = "\n".join(ck.report_lines(rep))
    assert "RMSE_H 6.1 cm" in lines and "合成 RMSE_V 3.5 cm" in lines and "小專案聲明" in lines


def test_targets_drive_the_two_times_rule_and_thirty_points_meet_asprs(tmp_path):
    s, m = hand_example(tmp_path, sigma_v=0.02)
    rep = ck.accuracy_report(ck.read_surveyed_csv(s), measured=ck.read_measured_csv(m), target_rmse_h_m=0.10,
                             target_rmse_v_m=0.05)
    a = rep["asprs"]
    assert a["survey_vs_product_v"] == {"survey_rmse_m": pytest.approx(0.02), "product_rmse_m": 0.05,
                                        "basis": "目標精度", "ratio": pytest.approx(2.5), "ok": True}
    assert a["meets_target_h"] is True and a["meets_target_v"] is True
    rep2 = ck.accuracy_report(ck.read_surveyed_csv(s), measured=ck.read_measured_csv(m), target_rmse_v_m=0.03)
    assert rep2["asprs"]["survey_vs_product_v"]["ok"] is False and rep2["asprs"]["meets_target_v"] is False
    rows = []
    for k in range(30):
        lat, lon = moved(LAT0, LON0, 40.0 * k, 0)
        rows.append((f"K{k}", "CHK", f"{lat:.10f}", f"{lon:.10f}", 10.0, "EPSG:4326", "ellipsoidal", 0.01, 0.01))
    s30 = write_csv(tmp_path / "s30.csv", HEADER, rows)
    m30 = write_csv(tmp_path / "m30.csv", "id,lat,lon,frame", [(r[0], r[2], r[3], "EPSG:4326") for r in rows])
    rep3 = ck.accuracy_report(ck.read_surveyed_csv(s30), measured=ck.read_measured_csv(m30))
    assert rep3["asprs"]["meets_min"] is True and "符合 ASPRS 第 2 版" in rep3["asprs"]["statement"]
    assert rep3["horizontal"]["rmse_h_m"] == pytest.approx(0.0, abs=1e-6) and rep3["vertical"]["n"] == 0


def _dsm(path, crs, origin, *, px=0.5, n=400, plane=(30.0, 0.01, -0.02), nodata_at=None):
    import rasterio
    from rasterio.transform import from_origin
    x0, y0 = origin
    xs = x0 + (np.arange(n) + 0.5) * px
    ys = y0 - (np.arange(n) + 0.5) * px
    X, Y = np.meshgrid(xs, ys)
    a, b, c = plane
    z = (a + b * (X - x0) + c * (Y - y0)).astype("float64")
    if nodata_at is not None:
        z[nodata_at] = -9999.0
    with rasterio.open(path, "w", driver="GTiff", width=n, height=n, count=1, dtype="float64", crs=crs,
                       transform=from_origin(x0, y0, px, px), nodata=-9999.0) as ds:
        ds.write(z, 1)
    return lambda X_, Y_: a + b * (X_ - x0) + c * (Y_ - y0)


def _dsm_case(tmp_path, crs="EPSG:3826", **kw):
    to = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    pts = {pid: moved(LAT0, LON0, 20 + x / 3, 20 + y / 3) for pid, (x, y) in POS.items()}
    xs = [to.transform(lo, la) for la, lo in pts.values()]
    origin = (math.floor(min(x for x, _ in xs)) - 20.0, math.ceil(max(y for _, y in xs)) + 20.0)
    plane = _dsm(tmp_path / "dsm.tif", crs, origin, **kw)
    truth = {pid: plane(*to.transform(lo, la)) for pid, (la, lo) in pts.items()}
    rows = [(pid, "CHK", f"{la:.10f}", f"{lo:.10f}", truth[pid] - ERR[pid][2] / 100.0, "EPSG:4326", "ellipsoidal",
             0.01, 0.01) for pid, (la, lo) in pts.items()]
    return write_csv(tmp_path / "s.csv", HEADER, rows), tmp_path / "dsm.tif", truth, to, origin


def test_dsm_is_sampled_bilinearly_in_its_own_crs(tmp_path):
    s, dsm, truth, _, _ = _dsm_case(tmp_path, crs="EPSG:3826")              # TWD97 / TM2（不是 WGS84）
    rep = ck.accuracy_report(ck.read_surveyed_csv(s), dsm_path=dsm, dsm_vertical="ellipsoidal")
    dz = {r["id"]: r["dZ"] for r in rep["points"]}
    for pid in POS:
        assert dz[pid] == pytest.approx(ERR[pid][2] / 100.0, abs=1e-6), pid
    assert rep["vertical"]["rmse_v_m"] == pytest.approx(math.sqrt(8.5) / 100, abs=1e-6)
    assert rep["effective"]["dsm"]["crs"].startswith("EPSG:3826") and rep["effective"]["dsm"]["pixel"] == [0.5, 0.5]
    assert rep["horizontal"]["n"] == 0
    # 換成 UTM 51N 一樣
    (tmp_path / "u").mkdir()
    s2, dsm2, *_ = _dsm_case(tmp_path / "u", crs="EPSG:32651")
    rep2 = ck.accuracy_report(ck.read_surveyed_csv(s2), dsm_path=dsm2, dsm_vertical="ellipsoidal")
    assert [r["dZ"] for r in rep2["points"]] == pytest.approx([ERR[p][2] / 100.0 for p in POS], abs=1e-6)


def test_dsm_nodata_and_outside_points_are_reported_not_guessed(tmp_path):
    s, dsm, truth, to, origin = _dsm_case(tmp_path)
    rows = ck.read_surveyed_csv(s)
    c1 = next(p for p in rows if p.id == "C1")
    x, y = to.transform(c1.lon, c1.lat)
    col, row = int((x - origin[0]) / 0.5), int((origin[1] - y) / 0.5)
    _dsm(tmp_path / "dsm2.tif", "EPSG:3826", origin, nodata_at=(row, col))
    far = ck.SurveyedPoint("FAR", LAT0 + 0.05, LON0, 10.0, "EPSG:4326", "ellipsoidal", 0.01, 0.01)
    rep = ck.accuracy_report(rows + [far], dsm_path=tmp_path / "dsm2.tif", dsm_vertical="ellipsoidal")
    by = {r["id"]: r for r in rep["points"]}
    assert by["C1"]["dZ"] is None and "nodata" in by["C1"]["reason"]
    assert by["FAR"]["dZ"] is None and "範圍外" in by["FAR"]["reason"]
    assert rep["vertical"]["n"] == 3 and any("沒有成果值" in w for w in rep["warnings"])


def test_vertical_datum_mismatch_is_refused_unless_an_undulation_is_given(tmp_path):
    s, dsm, truth, *_ = _dsm_case(tmp_path)
    pts = ck.read_surveyed_csv(s)
    ortho = [ck.SurveyedPoint(p.id, p.lat, p.lon, p.h - 18.54, p.frame, "EGM96", p.sigma_h_m, p.sigma_v_m)
             for p in pts]
    with pytest.raises(CheckpointError, match="沒有明確給大地起伏"):
        ck.accuracy_report(ortho, dsm_path=dsm, dsm_vertical="ellipsoidal")
    with pytest.raises(CheckpointError, match="明確宣告 DSM"):
        ck.accuracy_report(ortho, dsm_path=dsm)
    rep = ck.accuracy_report(ortho, dsm_path=dsm, dsm_vertical="ellipsoidal", undulation=18.54)
    assert [r["dZ"] for r in rep["points"]] == pytest.approx([ERR[p][2] / 100.0 for p in POS], abs=1e-6)
    assert rep["effective"]["dsm"]["conversion"] == "dsm_ell_to_ortho"
    geo = datum.GeoidModel("EGM96", 18.54, "test")
    rep2 = ck.accuracy_report(ortho, dsm_path=dsm, dsm_vertical="ellipsoidal", undulation=geo)
    assert rep2["vertical"]["rmse_v_m"] == pytest.approx(rep["vertical"]["rmse_v_m"])
    with pytest.raises(CheckpointError, match="TWVD2001"):
        ck.accuracy_report(ortho, dsm_path=dsm, dsm_vertical="ellipsoidal",
                           undulation=datum.GeoidModel("TWVD2001", 18.0, "x"))
    with pytest.raises(CheckpointError, match="兩種正高"):
        ck.accuracy_report(ortho, dsm_path=dsm, dsm_vertical="TWVD2001", undulation=18.54)
    with pytest.raises(CheckpointError, match="不夠明確"):
        ck.normalize_vdatum("orthometric")


def test_raster_crs_vertical_component_must_agree_with_the_declaration():
    comp = CRS.from_user_input("EPSG:3826+5773")                     # TWD97 TM2 + EGM96 height
    assert ck.raster_vertical_datum(comp.to_wkt()) == "EGM96"
    assert ck.raster_vertical_datum(CRS.from_epsg(3826).to_wkt()) is None


def test_compound_crs_dsm_declared_otherwise_is_refused(tmp_path):
    rasterio = pytest.importorskip("rasterio")
    s, dsm, *_ = _dsm_case(tmp_path)
    with rasterio.open(dsm) as src:
        prof, data = src.profile, src.read(1)
    prof["crs"] = rasterio.crs.CRS.from_user_input("EPSG:3826+5773")
    with rasterio.open(tmp_path / "comp.tif", "w", **prof) as dst:
        dst.write(data, 1)
    with rasterio.open(tmp_path / "comp.tif") as chk:
        if ck.raster_vertical_datum(chk.crs.to_wkt()) != "EGM96":
            pytest.skip("這版 GDAL 寫 GeoTIFF 時沒保留垂直分量")
    with pytest.raises(CheckpointError, match="CRS 寫的垂直基準是 EGM96"):
        ck.accuracy_report(ck.read_surveyed_csv(s), dsm_path=tmp_path / "comp.tif", dsm_vertical="ellipsoidal")


def test_frames_are_never_silently_treated_as_wgs84(tmp_path):
    s, dsm, *_ = _dsm_case(tmp_path)
    txt = s.read_text(encoding="utf-8").replace("EPSG:4326", "e-GNSS[2025]")
    s2 = tmp_path / "egnss.csv"
    s2.write_text(txt, encoding="utf-8")
    pts = ck.read_surveyed_csv(s2)
    with pytest.raises(CheckpointError, match="不是 PROJ 可解析"):
        ck.accuracy_report(pts, dsm_path=dsm, dsm_vertical="ellipsoidal")
    rep = ck.accuracy_report(pts, dsm_path=dsm, dsm_vertical="ellipsoidal",
                             frame_crs="+proj=longlat +ellps=GRS80 +no_defs")
    assert rep["effective"]["frame"] == "e-GNSS[2025]" and rep["vertical"]["n"] == 4
    m = write_csv(tmp_path / "m.csv", "id,lat,lon,frame", [(p.id, p.lat, p.lon, "EPSG:4326") for p in pts])
    with pytest.raises(CheckpointError, match="不做框架轉換"):
        ck.accuracy_report(pts, measured=ck.read_measured_csv(m))


def test_bad_survey_csvs_are_refused_with_the_line(tmp_path):
    base = [("A", "CHK", 25.07, 121.21, 10, "EPSG:4326", "ellipsoidal", 0.01, 0.01)]
    for bad, msg in (([("B", "CHK", 25.07, 121.21, 10, "EPSG:4326", "ellipsoidal", "", 0.01)], "σ 水平"),
                     ([("A", "CHK", 25.07, 121.21, 10, "EPSG:4326", "ellipsoidal", 0.01, 0.01)], "重複"),
                     ([("B", "CHK", "", 121.21, 10, "EPSG:4326", "ellipsoidal", 0.01, 0.01)], "緯度沒填"),
                     ([("B", "CHK", 25.07, 121.21, 10, "EPSG:4326", "正高", 0.01, 0.01)], "不夠明確"),
                     ([("B", "CHK", 25.07, 121.21, 10, "EPSG:3826", "ellipsoidal", 0.01, 0.01)], "混用框架")):
        p = write_csv(tmp_path / "b.csv", HEADER, base + bad)
        with pytest.raises(CheckpointError, match=msg):
            ck.read_surveyed_csv(p)
    p = write_csv(tmp_path / "g.csv", HEADER, base + [("G", "GCP", 25.07, 121.21, 10, "EPSG:4326", "ellipsoidal", "",
                                                       "")])
    assert [x.id for x in ck.read_surveyed_csv(p)] == ["A"]               # GCP 列不讀（σ 空白也不報）
    assert ck.checkpoint_ids(p) == ["A"]


def test_checkpoints_used_as_gcps_are_refused(tmp_path):
    s, m = hand_example(tmp_path)
    pts = ck.read_surveyed_csv(s)
    with pytest.raises(CheckpointError, match="不能拿來當控制點"):
        ck.accuracy_report(pts, measured=ck.read_measured_csv(m), gcp_names=["C2", "GCP01"])
    layout = [{"id": "C3", "type": "GCP", "lat": pts[2].lat, "lon": pts[2].lon, "block": "", "purpose": "corner",
               "note": ""}]
    with pytest.raises(CheckpointError, match="是 GCP"):
        ck.accuracy_report(pts, measured=ck.read_measured_csv(m), layout=layout)


def test_per_block_and_per_seam_height_steps_from_a_real_layout(tmp_path):
    from tests.test_survey_gcp_plan import AREA, BLOCKS
    plan = gp.plan_layout([AREA], BLOCKS, gp.LayoutParams())
    bias = {"A1": 0.05, "B2": -0.03, "B3": 0.00}                       # 每台相機自己的高程偏差
    srows, mrows = [], []
    for p in plan.checkpoints():
        b = p.block.split("|")[0]
        lat, lon = moved(p.lat, p.lon, 1.5, -0.7)                        # 外業標靶離規劃點 1.7 m（合理）
        srows.append((p.id, "CHK", f"{lat:.10f}", f"{lon:.10f}", 40.0, "EPSG:4326", "ellipsoidal", 0.01, 0.015))
        mrows.append((p.id, f"{lat:.10f}", f"{lon:.10f}", "EPSG:4326", 40.0 + bias[b], "ellipsoidal"))
    s = write_csv(tmp_path / "s.csv", HEADER, srows)
    m = write_csv(tmp_path / "m.csv", "id,lat,lon,frame,h,vertical_datum", mrows)
    paths = gp.export_layout(plan, tmp_path / "plan", projection="EPSG:4326")
    rep = ck.accuracy_report(ck.read_surveyed_csv(s), measured=ck.read_measured_csv(m),
                             layout=gp.read_layout_csv(paths["csv"]))
    assert rep["per_block"]["A1"]["mean_v_m"] == pytest.approx(0.05) and rep["per_block"]["B2"]["mean_v_m"] == \
        pytest.approx(-0.03)
    seams = {s_["seam"]: s_ for s_ in rep["per_seam"]}
    assert sorted(seams) == ["A1|B2", "B2|B3"]
    assert seams["A1|B2"]["pairs"][0]["step_m"] == pytest.approx(0.08)
    assert seams["B2|B3"]["pairs"][0]["step_m"] == pytest.approx(-0.03)
    assert seams["A1|B2"]["block_mean_step_m"] == pytest.approx(0.08)
    assert not any("編號對錯" in w for w in rep["warnings"])
    # 編號對錯（實測點跑到別的點去）⇒ 警告
    srows[0] = (srows[0][0], "CHK", srows[5][2], srows[5][3]) + srows[0][4:]
    s.write_text(HEADER + "\n" + "\n".join(",".join(str(v) for v in r) for r in srows) + "\n", encoding="utf-8")
    rep2 = ck.accuracy_report(ck.read_surveyed_csv(s), measured=ck.read_measured_csv(m),
                              layout=gp.read_layout_csv(paths["csv"]))
    assert any(srows[0][0] in w and "編號對錯" in w for w in rep2["warnings"])


def test_write_report_json_and_markdown(tmp_path):
    s, m = hand_example(tmp_path)
    rep = ck.accuracy_report(ck.read_surveyed_csv(s), measured=ck.read_measured_csv(m))
    out = ck.write_report(rep, tmp_path / "r")
    j = json.loads(out["json"].read_text(encoding="utf-8"))
    assert j["format"] == ck.FORMAT and j["vertical"]["n"] == 4
    md = out["md"].read_text(encoding="utf-8")
    assert "| C4 |" in md and "小專案聲明" in md
    with pytest.raises(CheckpointError, match="至少要給"):
        ck.accuracy_report(ck.read_surveyed_csv(s))


# ═════════════════════════════════════════════════════════════════════════════
# 現場 QA：EKF 風（survey.qa.wind_report；使用者 2026-10-15：首飛只在 ≤ 3 m/s）
# ═════════════════════════════════════════════════════════════════════════════
from survey import dflog, qa  # noqa: E402
from tests.test_survey_synthlog import CAM_COLS, CAM_FMT_FLOAT, DFWriter  # noqa: E402

LOGS = __import__("pathlib").Path(__file__).resolve().parents[1] / "sitl" / "vtol" / "i100" / "logs"
XKF2_COLS = ["TimeUS", "C", "AX", "AY", "AZ", "VWN", "VWE", "MN", "ME", "MD", "MX", "MY", "MZ", "IDX", "IDY", "IS"]
XKF2_FMT = "QBccccchhhhhhfff"                 # 本機 SITL 4.8.0-dev 日誌的實際格式（VWN／VWE 是 'c'）


NKF2_COLS = ["TimeUS", "C", "AZbias", "GSX", "GSY", "GSZ", "VWN", "VWE", "MN", "ME", "MD", "MX", "MY", "MZ", "MI"]
NKF2_FMT = "QBbccccchhhhhhB"                  # 同一份 SITL 日誌裡 EKF2 的格式


def _wind_log(tmp_path, *, ekf_type=3.0, xkf2_fmt=XKF2_FMT, wind=None, pi_switch_s=25.0, gap=None, name="w.BIN",
              with_xkf2=True, cams=(12.0, 14.0, 16.0, 32.0, 34.0, 50.0), ekf2=False):
    """兩條帶（10–20 s、30–40 s）；核心 0 = 西風 5 m/s、核心 1 = 北風 2 m/s；PI 在 pi_switch_s 由 0 換 1。"""
    wind = wind or {0: (0.0, 5.0), 1: (-2.0, 0.0)}
    w = DFWriter()
    w.define("PARM", "QNff", ["TimeUS", "Name", "Value", "Default"])
    w.define("MSG", "QBBZ", ["TimeUS", "ID", "Seq", "Message"])
    cols = ["TimeUS", "CTot", "CNum", "CId", "Prm1", "Prm2", "Prm3", "Prm4", "Lat", "Lng", "Alt", "Frame"]
    w.define("CMD", "QHHHffffLLfB", cols)
    w.define("CAM", CAM_FMT_FLOAT, CAM_COLS)
    k2, k4 = ("NKF2", "NKF4") if ekf2 else ("XKF2", "XKF4")
    if with_xkf2:
        if ekf2:
            w.define("NKF2", NKF2_FMT, NKF2_COLS)
        else:
            w.define("XKF2", xkf2_fmt, XKF2_COLS)
        w.define(k4, "QBcccccfffHBIHb", ["TimeUS", "C", "SV", "SP", "SH", "SM", "SVT", "errRP", "OFN", "OFE", "FS",
                                         "TS", "SS", "GPS", "PI"])
    if ekf_type is not None:
        w.write("PARM", TimeUS=1000, Name="AHRS_EKF_TYPE", Value=ekf_type, Default=3.0)
    for cn, cid, p1 in ((2, 206, 18.85), (4, 206, 0.0), (6, 206, 18.85), (8, 206, 0.0)):
        w.write("CMD", TimeUS=2000, CTot=10, CNum=cn, CId=cid, Prm1=p1)
    for t, cn in ((10.0, 2), (20.0, 4), (30.0, 6), (40.0, 8)):
        w.write("MSG", TimeUS=int(t * 1e6), Message=f"Mission: {cn} SetCamTrigDst")
    if with_xkf2:
        for k in range(201):
            t = 5.0 + 0.2 * k
            if gap and gap[0] < t < gap[1]:
                continue
            for c in (0, 1):
                vn, ve = wind[c]
                w.write(k2, TimeUS=int(round(t * 1e6)), C=c, VWN=vn, VWE=ve)
        for k in range(41):
            t = 5.0 + k
            for c in (0, 1):
                w.write(k4, TimeUS=int(t * 1e6) + 10, C=c, PI=0 if t < pi_switch_s else 1)
    for i, t in enumerate(cams, start=1):
        w.write("CAM", TimeUS=int(t * 1e6), I=0, Img=i, Lat=25.1, Lng=121.2, Alt=100.0)
    return dflog.read_dflog(w.save(tmp_path / name))


def _wind(log, **kw):
    return qa.wind_report(log, dflog.cam_events(log), dflog.strip_windows(log), **kw)


def test_wind_per_strip_follows_the_primary_core_and_counts_photos_above_the_limit(tmp_path):
    rep = _wind(_wind_log(tmp_path))
    assert rep["available"] and rep["source"] == "XKF2" and rep["lane_switches"] == 1
    assert rep["core_selection"].startswith("XKF4.PI")
    s1, s2 = rep["per_strip"]
    assert (s1["mean_speed_mps"], s1["dir_from_deg"]) == (pytest.approx(5.0), pytest.approx(270.0))
    assert (s2["mean_speed_mps"], s2["dir_from_deg"]) == (pytest.approx(2.0), pytest.approx(0.0, abs=1e-9))
    assert (s1["n_photos"], s1["n_photos_above"], s2["n_photos"], s2["n_photos_above"]) == (3, 3, 2, 0)
    assert (rep["n_photos"], rep["n_photos_with_wind"], rep["n_photos_above"]) == (6, 5, 3)   # 50 s 那張沒有風樣本
    assert rep["share_above"] == pytest.approx(0.6) and rep["flagged"] is True and rep["limit_mps"] == 3.0
    low = _wind(_wind_log(tmp_path, name="b.BIN"), limit_mps=1.5)
    assert low["n_photos_above"] == 5
    calm = _wind(_wind_log(tmp_path, name="c.BIN", wind={0: (0.3, 0.4), 1: (0.0, 0.1)}))
    assert calm["flagged"] is False and calm["share_above"] == 0.0 and calm["photo_speed_max_mps"] == pytest.approx(0.5)
    assert qa.wind_dir_from_deg(0.0, -5.0) == pytest.approx(90.0) and qa.wind_dir_from_deg(5.0, 0.0) == pytest.approx(180.0)


def test_ekf2_logs_are_read_on_demand_with_the_same_rules(tmp_path):
    log = _wind_log(tmp_path, ekf_type=2.0, ekf2=True, name="ekf2.BIN")
    assert "NKF2" not in log.tables and "NKF2" in dflog.EKF2_WIND_TYPES        # 預設不讀（EKF3 機用不到）
    rep = _wind(log)
    assert rep["available"] and rep["source"] == "NKF2" and rep["core_selection"].startswith("NKF4.PI")
    assert [round(s["mean_speed_mps"], 6) for s in rep["per_strip"]] == [5.0, 2.0] and rep["n_photos_above"] == 3
    no = _wind(_wind_log(tmp_path, ekf_type=2.0, ekf2=False, name="ekf2x.BIN"))   # 宣告 EKF2 但只有 XKF2
    assert not no["available"] and "日誌沒有 NKF2" in no["reason"]


def test_wind_gaps_formats_and_missing_estimates_are_explicit(tmp_path):
    g = _wind(_wind_log(tmp_path, gap=(13.0, 15.5), name="g.BIN"))
    assert g["n_photos_with_wind"] == 4                                 # 14 s 那張落在 > 2 s 的缺口
    f = _wind(_wind_log(tmp_path, xkf2_fmt="QBccc" + "ff" + "hhhhhhfff", name="f.BIN"))
    assert f["available"] and f["per_strip"][0]["mean_speed_mps"] == pytest.approx(5.0)
    h = _wind(_wind_log(tmp_path, xkf2_fmt="QBccchchhhhhhfff", name="h.BIN"))
    assert not h["available"] and "無法判讀單位" in h["reason"]
    e = _wind(_wind_log(tmp_path, ekf_type=10.0, name="e.BIN"))
    assert not e["available"] and "不是 EKF2／EKF3" in e["reason"] and e["flagged"] is False
    n = _wind(_wind_log(tmp_path, with_xkf2=False, name="n.BIN"))
    assert not n["available"] and "日誌沒有 XKF2" in n["reason"]
    z = _wind(_wind_log(tmp_path, wind={0: (0.0, 0.0), 1: (0.0, 0.0)}, name="z.BIN"))
    assert not z["available"] and "全為 0" in z["reason"]
    with pytest.raises(ValueError):
        _wind(_wind_log(tmp_path, name="x.BIN"), limit_mps=float("nan"))


@pytest.fixture(scope="module")
def sitl_logs():
    out = {}
    for n in ("00000021.BIN", "00000022.BIN"):
        p = LOGS / n
        if not p.is_file():
            pytest.skip(f"本機沒有 SITL 日誌 {p}")
        out[n] = dflog.read_dflog(p)
    return out


def test_real_sitl_logs_5_mps_flight_reads_about_5_from_the_west_and_calm_flight_about_0(sitl_logs):
    """00000022 是 SIM_WIND_SPD 5、SIM_WIND_DIR 270 飛的；00000021 是 0 m/s（日誌參數裡讀得到）。"""
    w5, w0 = sitl_logs["00000022.BIN"], sitl_logs["00000021.BIN"]
    assert (w5.params["SIM_WIND_SPD"], w5.params["SIM_WIND_DIR"], w0.params["SIM_WIND_SPD"]) == (5.0, 270.0, 0.0)
    r5, r0 = _wind(w5), _wind(w0)
    assert r5["available"] and r5["source"] == "XKF2" and len(r5["per_strip"]) == 7
    for s in r5["per_strip"]:
        assert 4.5 <= s["mean_speed_mps"] <= 5.5 and 265.0 <= s["dir_from_deg"] <= 275.0, s
    assert r5["share_above"] == 1.0 and r5["flagged"] and r5["n_photos"] == 619
    assert sum(s["n_photos"] for s in r5["per_strip"]) == r5["n_photos"]
    assert r0["available"] and not r0["flagged"] and r0["share_above"] == 0.0 and r0["n_photos"] == 637
    assert max(s["max_speed_mps"] for s in r0["per_strip"]) < 0.5
    assert w5.column_specs["XKF2"]["VWN"] == ("c", 0.01, "m/s")
