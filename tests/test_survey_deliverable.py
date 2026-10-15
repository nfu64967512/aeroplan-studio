"""survey.deliverable：F2 photos.csv、GeoJSON、說明檔。"""

import json

import pytest

from survey import datum, deliverable
from survey.deliverable import F2_COLUMNS


def _row(i, **kw):
    r = {c: None for c in F2_COLUMNS}
    r.update(image_id=f"hero2_S_{i:06d}", file=f"images/{i:06d}.jpg", uav="hero2", strip=1, seq=i,
             t_fc_us=400_000_000 + i * 900_000, t_gpst_s=1_444_000_000.123456 + i * 0.9,
             lat_deg=25.1 + i * 1e-4, lon_deg=121.2, h_ell_m=168.5, h_orth_m=149.96, yaw_deg=95.0, pitch_deg=2.0,
             roll_deg=-1.0, cog_deg=90.0, speed_n_mps=0.0, speed_e_mps=21.5, speed_d_mps=-0.1, sigma_h_m=0.5,
             sigma_v_m=0.8, source="fc_interp", crs_h="EPSG:4326", crs_v="EGM96", flags="roll_gt_10;off_strip")
    r.update(kw)
    return r


def test_columns_are_exactly_f2():
    assert F2_COLUMNS == ("image_id", "file", "uav", "strip", "seq", "t_fc_us", "t_gpst_s", "lat_deg", "lon_deg",
                          "h_ell_m", "h_orth_m", "yaw_deg", "pitch_deg", "roll_deg", "cog_deg", "speed_n_mps",
                          "speed_e_mps", "speed_d_mps", "sigma_h_m", "sigma_v_m", "source", "crs_h", "crs_v",
                          "flags")


def test_csv_roundtrip_and_empty_fields(tmp_path):
    rows = [_row(2), _row(1, h_orth_m=None, crs_v="ellipsoidal", strip=None, cog_deg=None, flags="")]
    p = deliverable.write_photos_csv(tmp_path / "photos.csv", rows)
    text = p.read_text(encoding="utf-8").splitlines()
    assert text[0] == ",".join(F2_COLUMNS)
    assert text[1].startswith("hero2_S_000001,images/000001.jpg,hero2,,1,400900000,1444000001.023456,25.100100000,")
    back = deliverable.read_photos_csv(p)
    assert [r["seq"] for r in back] == [1, 2]                       # 依 (uav, t_gpst_s) 排序
    assert back[0]["h_orth_m"] is None and back[0]["strip"] is None and back[0]["cog_deg"] is None
    assert back[0]["flags"] == "" and back[1]["flags"] == "roll_gt_10;off_strip"
    assert back[1]["t_gpst_s"] == pytest.approx(1_444_000_001.923456, abs=1e-6)
    assert back[1]["lat_deg"] == pytest.approx(25.1002, abs=1e-9) and back[1]["t_fc_us"] == 401_800_000
    bad = tmp_path / "bad.csv"
    bad.write_text("a,b\n1,2\n", encoding="utf-8")
    with pytest.raises(ValueError, match="F2"):
        deliverable.read_photos_csv(bad)


def test_validation(tmp_path):
    p = tmp_path / "x.csv"
    with pytest.raises(ValueError, match="source"):
        deliverable.write_photos_csv(p, [_row(1, source="gps")])
    with pytest.raises(ValueError, match="crs_v"):
        deliverable.write_photos_csv(p, [_row(1, crs_v="MSL")])
    with pytest.raises(ValueError, match="不是正高基準"):
        deliverable.write_photos_csv(p, [_row(1, crs_v="ellipsoidal")])        # 有 h_orth_m 卻標橢球
    with pytest.raises(ValueError, match="重複"):
        deliverable.write_photos_csv(p, [_row(1), _row(1)])
    with pytest.raises(ValueError, match="不在 F2"):
        deliverable.write_photos_csv(p, [dict(_row(1), extra=1)])


def test_geojson_and_sidecar(tmp_path):
    rows = [_row(1), _row(2, h_ell_m=None, h_orth_m=150.0, t_gpst_s=None)]
    gj = deliverable.to_geojson(rows)
    f1, f2 = gj["features"]
    assert f1["geometry"]["coordinates"] == [121.2, 25.1001, 168.5]           # 第三維 = 橢球高
    assert f2["geometry"]["coordinates"] == [121.2, 25.1002]                  # 沒有橢球高 ⇒ 二維
    assert f1["properties"]["heading_deg"] == 95.0 and f1["properties"]["cog_deg"] == 90.0
    assert f1["properties"]["t_utc"] == datum.gpst_to_utc(rows[0]["t_gpst_s"]).isoformat(timespec="milliseconds")
    assert f1["properties"]["t_utc"].endswith("+00:00") and f2["properties"]["t_utc"] is None
    sc = deliverable.sidecar(rows, {"frame_epoch": "e-GNSS[2025]", "lever_arm_effective_m": [0.1, 0, 0]})
    assert sc["format"] == "aeroplan-i1/1" and sc["n_photos"] == 2
    assert sc["time_system"]["gpst_minus_utc_s"] == 18
    assert sc["crs_h"] == ["EPSG:4326"] and sc["crs_v"] == ["EGM96"] and sc["frame_epoch"] == "e-GNSS[2025]"
    assert sc["sources"] == {"fc_interp": 2, "ppk": 0, "sfm": 0}
    assert sc["meta"] == {"lever_arm_effective_m": [0.1, 0, 0]}
    a = deliverable.write_i1_deliverable(tmp_path / "a", rows, {"x": 1})
    b = deliverable.write_i1_deliverable(tmp_path / "b", list(reversed(rows)), {"x": 1})
    for k in ("csv", "geojson", "meta"):                                       # 可重現：輸入順序不影響位元組
        assert a[k].read_bytes() == b[k].read_bytes()
        assert b"\r" not in a[k].read_bytes()                                  # 各平台都是 LF
    assert json.loads(a["geojson"].read_text(encoding="utf-8"))["type"] == "FeatureCollection"
