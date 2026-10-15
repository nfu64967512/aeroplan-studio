"""survey.geotag：geo.txt 欄位順序與 ODM GeoFile 的解析一致；不完整就拒絕。"""

import logging
import math

import pytest

from survey import geotag


def odm_parse_geo(text):
    """照 OpenDroneMap/ODM opendm/geo.py GeoFile 的切欄邏輯（EPSG:4326 不做座標轉換）。"""
    lines = [ln.strip() for ln in text.strip().split("\n")]
    out = {}
    for line in lines[1:]:
        parts = line.split()
        x, y = float(parts[1]), float(parts[2])
        z = float(parts[3]) if len(parts) >= 4 else None
        ypr = None
        if len(parts) >= 7:
            ypr = [float(p) for p in parts[4:7]]
            if any(math.isnan(v) for v in ypr):
                ypr = None
        acc = [float(p) for p in parts[7:9]] if len(parts) >= 9 else None
        out[parts[0]] = dict(lon=x, lat=y, z=z, ypr=ypr, acc=acc)
    return lines[0], out


def _rows():
    return [
        {"image_id": "hero2_S_000001", "file": "images/000001.jpg", "lat_deg": 25.123456789, "lon_deg": 121.987654321,
         "h_ell_m": 168.5432, "h_orth_m": 150.0032, "yaw_deg": 95.25, "pitch_deg": 2.0, "roll_deg": -1.5,
         "sigma_h_m": 0.5, "sigma_v_m": 0.8, "crs_h": "EPSG:4326", "crs_v": "EGM96"},
        {"image_id": "hero2_S_000002", "file": "images/000002.JPG", "lat_deg": 25.123556789, "lon_deg": 121.987754321,
         "h_ell_m": 168.6, "h_orth_m": 150.06, "yaw_deg": 359.9, "pitch_deg": 1.0, "roll_deg": 0.0,
         "sigma_h_m": 0.4, "sigma_v_m": 0.7, "crs_h": "EPSG:4326", "crs_v": "EGM96"},
    ]


def test_geo_txt_columns_match_odm_parser(tmp_path):
    p = tmp_path / "geo.txt"
    assert geotag.write_geo_txt(p, _rows(), height="ellipsoidal") == 2
    assert b"\r" not in p.read_bytes()                          # Windows 上也是 LF（輸出可重現）
    text = p.read_text(encoding="utf-8")
    assert text.splitlines()[1] == "hero2_S_000001.jpg 121.987654321 25.123456789 168.543 95.250 2.000 -1.500 0.500 0.800"
    header, ent = odm_parse_geo(text)
    assert header == "EPSG:4326"
    e = ent["hero2_S_000002.JPG"]                               # 副檔名大小寫保留
    assert (e["lon"], e["lat"], e["z"]) == (121.987754321, 25.123556789, 168.6)
    assert e["ypr"] == [359.9, 1.0, 0.0] and e["acc"] == [0.4, 0.7]
    geotag.write_geo_txt(p, _rows(), height="orthometric")
    assert odm_parse_geo(p.read_text(encoding="utf-8"))[1]["hero2_S_000001.jpg"]["z"] == 150.003


def test_angles_can_be_withheld_but_accuracy_kept(tmp_path):
    p = tmp_path / "geo.txt"
    geotag.write_geo_txt(p, _rows(), height="ellipsoidal", include_angles=False)
    _, ent = odm_parse_geo(p.read_text(encoding="utf-8"))
    assert ent["hero2_S_000001.jpg"]["ypr"] is None and ent["hero2_S_000001.jpg"]["acc"] == [0.5, 0.8]
    geotag.write_geo_txt(p, _rows(), height="ellipsoidal", include_accuracy=False)
    assert len(p.read_text(encoding="utf-8").splitlines()[1].split()) == 7
    geotag.write_geo_txt(p, _rows(), height="ellipsoidal", include_accuracy=False, include_angles=False)
    assert len(p.read_text(encoding="utf-8").splitlines()[1].split()) == 4


def test_incomplete_or_mixed_rows_are_rejected(tmp_path):
    p = tmp_path / "geo.txt"
    rows = _rows()
    rows[1]["sigma_v_m"] = None
    with pytest.raises(ValueError, match="缺欄位"):
        geotag.write_geo_txt(p, rows, height="ellipsoidal")
    rows = _rows()
    rows[0]["h_orth_m"] = None
    with pytest.raises(ValueError, match="缺欄位"):
        geotag.write_geo_txt(p, rows, height="orthometric")
    geotag.write_geo_txt(p, rows, height="ellipsoidal")          # 橢球高齊全就可以
    rows = _rows()
    rows[1]["crs_v"] = "TWVD2001"
    with pytest.raises(ValueError, match="正高的基準不一致"):
        geotag.write_geo_txt(p, rows, height="orthometric")
    rows = _rows()
    rows[1]["crs_h"] = "e-GNSS[2025]"
    with pytest.raises(ValueError, match="水平框架不一致"):
        geotag.write_geo_txt(p, rows, height="ellipsoidal")
    with pytest.raises(ValueError, match="height"):
        geotag.write_geo_txt(p, _rows(), height="amsl")
    rows = _rows()
    rows[1]["image_id"] = rows[0]["image_id"]
    rows[1]["file"] = rows[0]["file"]
    with pytest.raises(ValueError, match="檔名重複"):
        geotag.write_geo_txt(p, rows, height="ellipsoidal")
    with pytest.raises(ValueError):
        geotag.write_geo_txt(p, [], height="ellipsoidal")


def test_non_wgs84_frame_needs_explicit_projection(tmp_path, caplog):
    p = tmp_path / "geo.txt"
    rows = _rows()
    for r in rows:
        r["crs_h"] = "e-GNSS[2025]"
    with pytest.raises(ValueError, match="明確給 projection"):
        geotag.write_geo_txt(p, rows, height="ellipsoidal")
    with caplog.at_level(logging.WARNING, logger="survey.geotag"):
        geotag.write_geo_txt(p, rows, height="ellipsoidal", projection="EPSG:4326")
    assert p.read_text(encoding="utf-8").splitlines()[0] == "EPSG:4326"
    assert any("e-GNSS[2025]" in m for m in caplog.messages)


def test_export_filename():
    assert geotag.export_filename({"image_id": "a_b_000001", "file": "images/000001.jpg"}) == "a_b_000001.jpg"
    assert geotag.export_filename({"image_id": "x", "file": None}) == "x.jpg"
