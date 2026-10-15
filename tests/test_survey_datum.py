"""survey.datum：GPST／UTC、ArduPilot 大地起伏正負號、高度基準規則、小位移換算。"""

from datetime import datetime, timedelta, timezone
from types import SimpleNamespace

import numpy as np
import pytest
from pyproj import Geod

from survey import datum
from survey.datum import GeoidModel, ReceiverUndulation, fc_heights

N_SITE = 18.54            # 25.10N 121.20E 的 EGM96 大地起伏（評估 §8，pyproj）—— 只在測試用


def test_gpst_and_utc():
    assert datum.GPST_MINUS_UTC_S == 18
    assert datum.gpst_seconds(2437, 99068.6) == 2437 * 604800 + 99068.6
    utc = datum.gpst_to_utc(2437 * 604800.0)
    assert utc == datetime(1980, 1, 6, tzinfo=timezone.utc) + timedelta(weeks=2437) - timedelta(seconds=18)
    assert utc == datetime(2026, 9, 19, 23, 59, 42, tzinfo=timezone.utc)
    assert datum.utc_to_gpst(utc) == pytest.approx(2437 * 604800.0)
    with pytest.raises(ValueError, match="2017"):
        datum.gpst_to_utc(1100 * 604800.0)
    with pytest.raises(ValueError, match="時區"):
        datum.utc_to_gpst(datetime(2026, 1, 1))


def _log(gpa, gps=None):
    tables = {"GPA": {k: np.asarray(v) for k, v in gpa.items()}}
    if gps:
        tables["GPS"] = {k: np.asarray(v) for k, v in gps.items()}
    return SimpleNamespace(tables=tables)


def test_receiver_undulation_from_und_uses_ardupilot_sign():
    # ArduPilot：Und = AMSL − h_ell = −N；0 表示接收機沒給
    log = _log({"TimeUS": [1, 2, 3, 4], "I": [0, 0, 0, 1], "Und": [-18.53, -18.55, 0.0, -99.0]})
    rx = datum.receiver_undulation_from_log(log)
    assert rx.source == "GPA.Und" and rx.n_samples == 2
    assert rx.n_m == pytest.approx(N_SITE) and rx.spread_m == pytest.approx(0.02)
    none = datum.receiver_undulation_from_log(_log({"TimeUS": [1], "I": [0], "Und": [0.0]}))
    assert none.n_m is None and none.source.startswith("GPA.Und")


def test_receiver_undulation_from_ael_pairs_with_gps():
    gps = {"TimeUS": [10, 20, 30], "I": [0, 0, 0], "Alt": [100.0, 101.0, 102.0]}
    gpa = {"TimeUS": [10, 20, 30], "I": [0, 0, 0], "AEl": [118.54, 119.54, -21474836.48]}   # INT32_MIN = 未知
    rx = datum.receiver_undulation_from_log(_log(gpa, gps))
    assert rx.source == "GPA.AEl" and rx.n_samples == 2 and rx.n_m == pytest.approx(N_SITE)
    assert datum.receiver_undulation_from_log(SimpleNamespace(tables={})).n_m is None


def test_fc_heights_rules():
    rx = ReceiverUndulation(N_SITE, "GPA.Und", 10, 0.0)
    unk = ReceiverUndulation(None, "none")
    # 接收機 N 已知、沒給大地起伏模型 ⇒ 只有橢球高
    assert fc_heights(100.0, 25.1, 121.2, receiver=rx) == (pytest.approx(118.54), None, "ellipsoidal", [])
    # 給了模型 ⇒ 正高 = 橢球高 − N_model
    g = GeoidModel("TWVD2001", lambda lat, lon: 18.40, "test grid")
    h_ell, h_orth, crs_v, fl = fc_heights(100.0, 25.1, 121.2, receiver=rx, geoid=g)
    assert (h_ell, h_orth, crs_v) == (pytest.approx(118.54), pytest.approx(100.14), "TWVD2001") and fl == []
    # 接收機 N 未知 ⇒ 不拿 N=0 硬算
    h_ell, h_orth, crs_v, fl = fc_heights(100.0, 25.1, 121.2, receiver=unk, geoid=g)
    assert h_ell is None and h_orth is None and crs_v == "unknown"
    assert "h_ell_unknown" in fl and "h_orth_needs_h_ell" in fl and "height_unknown" in fl
    # 使用者明確宣告接收機 AMSL 屬 EGM96 ⇒ 正高直接用 AMSL
    assert fc_heights(100.0, 25.1, 121.2, receiver=unk, fc_amsl_datum="EGM96")[1:3] == (100.0, "EGM96")
    with pytest.raises(ValueError, match="不能同時給"):
        fc_heights(100.0, 25.1, 121.2, receiver=rx, geoid=g, fc_amsl_datum="EGM96")
    with pytest.raises(ValueError):
        fc_heights(100.0, 25.1, 121.2, receiver=rx, fc_amsl_datum="ellipsoidal")
    with pytest.raises(ValueError):
        GeoidModel("WGS84", 18.0)
    assert datum.ellipsoidal_heights(118.54, 25.1, 121.2) == (118.54, None, "ellipsoidal", [])
    assert datum.ellipsoidal_heights(118.54, 25.1, 121.2, geoid=GeoidModel("EGM96", N_SITE))[1:3] == (
        pytest.approx(100.0), "EGM96")


def test_ned_offset_matches_geodesic():
    geod = Geod(ellps="WGS84")
    lat, lon, h = datum.ned_offset_to_llh(25.1, 121.2, 0.0, 30.0, 40.0, -2.0)
    lon_g, lat_g, _ = geod.fwd(121.2, 25.1, np.degrees(np.arctan2(40.0, 30.0)), 50.0)
    assert geod.inv(lon_g, lat_g, float(lon), float(lat))[2] < 1e-3      # 與大地線端點差 < 1 mm
    assert float(h) == 2.0
    e, n = datum.local_en_m(25.1, 121.2, float(lat), float(lon))
    assert (float(e), float(n)) == (pytest.approx(40.0, abs=1e-4), pytest.approx(30.0, abs=1e-4))


def test_ap_ned_to_llh_matches_ardupilot_scaling():
    # 1 度緯度 = 111318.845 m（ArduPilot LOCATION_SCALING_FACTOR，球面近似）
    lat, lon, alt = datum.ap_ned_to_llh(25.0, 121.0, 40.0, 111318.84502145034, 0.0, -10.0)
    assert float(lat) == pytest.approx(26.0, abs=1e-12) and float(alt) == 50.0
    lat, lon, _ = datum.ap_ned_to_llh(25.0, 121.0, 40.0, 0.0, 1000.0, 0.0)
    assert float(lon) - 121.0 == pytest.approx(1000.0 / 111318.84502145034 / np.cos(np.radians(25.0)), rel=1e-12)
