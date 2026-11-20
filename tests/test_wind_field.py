"""core.wind_field：風場資料模型、規劃風均勻場、Open-Meteo 網格的請求組裝／回應解析（不連網）。

慣例鎖定：dir_from_deg ＝ 來向（氣象）；西風 270° ⇒ u > 0（往東吹）；與 hv_wind、ArduPilot WIND 相同。
"""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.parse
from io import BytesIO

import pytest

from core import wind_field as wf


# ── 慣例 ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize('d, u, v', [(270.0, 1.0, 0.0), (90.0, -1.0, 0.0), (0.0, 0.0, -1.0), (180.0, 0.0, 1.0)])
def test_dir_to_uv_is_meteorological_from_direction(d, u, v):
    uu, vv = wf.dir_to_uv(d, 1.0)
    assert (round(uu, 9), round(vv, 9)) == (u, v)


@pytest.mark.parametrize('d, s', [(0.0, 1.0), (37.5, 2.5), (270.0, 3.0), (359.0, 0.1)])
def test_uv_to_dir_round_trips(d, s):
    dd, ss = wf.uv_to_dir(*wf.dir_to_uv(d, s))
    assert (round(dd, 6), round(ss, 9)) == (d, s)


def test_uv_to_dir_calm_is_zero():
    assert wf.uv_to_dir(0.0, 0.0) == (0.0, 0.0)


# ── WindField ─────────────────────────────────────────────────────────
def test_field_rejects_bad_shapes_and_unsorted_axes():
    with pytest.raises(ValueError):
        wf.WindField(lats=(1.0, 0.0), lons=(0.0, 1.0), u=((0, 0), (0, 0)), v=((0, 0), (0, 0)),
                     height_m=10, source='x', label='x')
    with pytest.raises(ValueError):
        wf.WindField(lats=(0.0, 1.0), lons=(0.0, 1.0), u=((0, 0),), v=((0, 0), (0, 0)),
                     height_m=10, source='x', label='x')
    with pytest.raises(ValueError):          # 單點沒 bbox
        wf.WindField(lats=(0.0,), lons=(0.0,), u=((1.0,),), v=((0.0,),), height_m=10, source='x', label='x')


def test_uniform_field_is_one_point_with_bbox_and_a_label_that_says_it_is_an_assumption():
    f = wf.uniform_field((24.0, 120.0, 24.02, 120.02), 3.0, 270.0)
    assert f.bbox == (24.0, 120.0, 24.02, 120.02)
    assert (round(f.lats[0], 9), round(f.lons[0], 9)) == (24.01, 120.01)
    assert (round(f.u[0][0], 9), round(f.v[0][0], 9)) == (3.0, 0.0)
    assert f.source == 'planning' and '非實測' in f.label and '3.0 m/s' in f.label and '270°' in f.label
    assert f.max_speed() == 3.0
    js = f.to_js()
    assert js['bbox'] == [24.0, 120.0, 24.02, 120.02] and js['u'] == [[3.0]] and js['mode'] == 'flow'
    json.dumps(js)                            # 必須可序列化


@pytest.mark.parametrize('spd, d', [(-1.0, 0.0), (math.nan, 0.0), (1.0, math.inf), ('3', 0.0)])
def test_uniform_field_rejects_invalid_values(spd, d):
    with pytest.raises(ValueError):
        wf.uniform_field((24.0, 120.0, 24.02, 120.02), spd, d)


def test_area_bbox_expands_by_margin():
    b = wf.area_bbox([[(24.0, 120.0), (24.01, 120.0), (24.01, 120.01)], [(23.995, 120.02)]], margin_m=111.32)
    assert round(b[0], 6) == round(23.995 - 0.001, 6) and round(b[2], 6) == round(24.01 + 0.001, 6)
    assert b[1] < 120.0 and b[3] > 120.02
    with pytest.raises(ValueError):
        wf.area_bbox([])


def test_grid_points_cover_bbox_inclusive_and_bounded():
    lats, lons = wf.grid_points((24.0, 120.0, 24.02, 120.03), 4, 3)
    assert [round(x, 6) for x in lats] == [24.0, 24.01, 24.02]
    assert [round(x, 6) for x in lons] == [120.0, 120.01, 120.02, 120.03]
    with pytest.raises(ValueError):
        wf.grid_points((24.0, 120.0, 24.02, 120.03), 1, 3)
    with pytest.raises(ValueError):
        wf.grid_points((24.0, 120.0, 24.02, 120.03), 11, 11)
    with pytest.raises(ValueError):
        wf.grid_points((24.02, 120.0, 24.0, 120.03), 3, 3)


# ── Open-Meteo ────────────────────────────────────────────────────────
def test_url_has_all_points_in_row_major_order_and_ms_units():
    url = wf.build_open_meteo_url([24.0, 24.1], [120.0, 120.5, 121.0], '10m')
    q = urllib.parse.parse_qs(urllib.parse.urlparse(url).query)
    assert url.startswith(wf.OPEN_METEO_URL)
    assert q['latitude'] == ['24.00000,24.00000,24.00000,24.10000,24.10000,24.10000']
    assert q['longitude'] == ['120.00000,120.50000,121.00000,120.00000,120.50000,121.00000']
    assert q['wind_speed_unit'] == ['ms'] and q['timezone'] == ['UTC']
    assert q['current'] == ['wind_speed_10m,wind_direction_10m,wind_gusts_10m'] and 'hourly' not in q
    url2 = wf.build_open_meteo_url([24.0, 24.1], [120.0, 120.5], '120m')
    q2 = urllib.parse.parse_qs(urllib.parse.urlparse(url2).query)
    assert q2['hourly'] == ['wind_speed_120m,wind_direction_120m'] and q2['forecast_hours'] == ['1']
    with pytest.raises(ValueError):
        wf.build_open_meteo_url([24.0, 24.1], [120.0, 120.5], '50m')


def _item(lat, lon, spd, d, *, kind='current', gust=None, t='2026-10-16T02:00'):
    it = {'latitude': lat, 'longitude': lon}
    if kind == 'current':
        it['current'] = {'time': t, 'wind_speed_10m': spd, 'wind_direction_10m': d}
        if gust is not None:
            it['current']['wind_gusts_10m'] = gust
    else:
        it['hourly'] = {'time': [t], 'wind_speed_120m': [spd], 'wind_direction_120m': [d]}
    return it


def test_parse_current_grid_keeps_requested_coords_and_counts_snapped_cells():
    lats, lons = [24.0, 24.1], [120.0, 120.5]
    payload = [_item(24.02, 120.03, 2.0, 270, gust=4.5), _item(24.02, 120.47, 1.0, 90, gust=2.0),
               _item(24.02, 120.03, None, 270), _item(24.11, 120.47, 0.0, 0)]
    f = wf.parse_open_meteo(payload, lats, lons, '10m')
    assert f.lats == (24.0, 24.1) and f.lons == (120.0, 120.5) and f.height_m == 10.0 and f.source == 'open-meteo'
    assert (round(f.u[0][0], 9), round(f.u[0][1], 9)) == (2.0, -1.0)
    assert math.isnan(f.u[1][0]) and math.isnan(f.v[1][0]) and f.u[1][1] == 0.0
    assert f.time_utc == '2026-10-16T02:00' and f.max_speed() == 2.0
    assert '非實測' in f.label and '3 個模式格點' in f.label and '陣風最大 4.5 m/s' in f.label
    assert any('1 點沒有風值' in n for n in f.notes)
    js = f.to_js(mode='arrows')
    assert js['u'][1][0] is None and js['mode'] == 'arrows' and 'bbox' not in js
    json.dumps(js)


def test_parse_hourly_120m_takes_the_first_hour():
    lats, lons = [24.0, 24.1], [120.0, 120.5]
    payload = [_item(24.0, 120.0, 5.0, 180, kind='hourly', t='2026-10-16T03:00')] * 4
    f = wf.parse_open_meteo(payload, lats, lons, '120m')
    assert f.height_m == 120.0 and round(f.v[1][1], 9) == 5.0 and f.time_utc == '2026-10-16T03:00'
    assert '120m' in f.label and '1 個模式格點' in f.label


def test_parse_rejects_count_mismatch_and_error_items():
    with pytest.raises(wf.WindFetchError, match='回了 1 筆'):
        wf.parse_open_meteo([_item(24.0, 120.0, 1.0, 0)], [24.0, 24.1], [120.0, 120.5], '10m')
    bad = [_item(24.0, 120.0, 1.0, 0)] * 3 + [{'error': True, 'reason': 'rate limited'}]
    with pytest.raises(wf.WindFetchError, match='rate limited'):
        wf.parse_open_meteo(bad, [24.0, 24.1], [120.0, 120.5], '10m')


class _Resp:
    def __init__(self, body):
        self._b = body

    def read(self):
        return self._b

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def test_fetch_uses_opener_and_wraps_network_errors(monkeypatch):
    seen = {}

    def opener(url, timeout):
        seen['url'], seen['timeout'] = url, timeout
        return _Resp(json.dumps([_item(24.0, 120.0, 1.5, 270)] * 4).encode('utf-8'))

    f = wf.fetch_open_meteo((24.0, 120.0, 24.1, 120.5), nx=2, ny=2, opener=opener, timeout_s=7.0)
    assert seen['timeout'] == 7.0 and 'latitude=' in seen['url'] and round(f.u[0][0], 9) == 1.5

    def down(url, timeout):
        raise urllib.error.URLError('no route')

    with pytest.raises(wf.WindFetchError, match='連不到'):
        wf.fetch_open_meteo((24.0, 120.0, 24.1, 120.5), nx=2, ny=2, opener=down)

    def http(url, timeout):
        raise urllib.error.HTTPError(url, 429, 'Too Many Requests', {}, BytesIO(b''))

    with pytest.raises(wf.WindFetchError, match='HTTP 429'):
        wf.fetch_open_meteo((24.0, 120.0, 24.1, 120.5), nx=2, ny=2, opener=http)

    def refused(url, timeout):
        return _Resp(json.dumps({'error': True, 'reason': 'bad params'}).encode('utf-8'))

    with pytest.raises(wf.WindFetchError, match='bad params'):
        wf.fetch_open_meteo((24.0, 120.0, 24.1, 120.5), nx=2, ny=2, opener=refused)

    # 預設 opener 是 urllib（不在測試裡真的連網）：只確認沒有被偷換成別的東西
    assert wf.fetch_open_meteo.__kwdefaults__['opener'] is None


# ═══════════════════════════════════════════════════════════════════════
#  CWA 自動氣象站（O-A0001-001）
# ═══════════════════════════════════════════════════════════════════════
def _cwa_station(sid, name, lat, lon, spd, d, *, gust=None, gust_dir=None, p=1008.2, alt="12.0",
                 t="2026-10-16T17:00:00+08:00", county="桃園市", town="大園區", wgs=True):
    coords = [{"CoordinateName": "TWD67", "CoordinateFormat": "decimal degrees",
               "StationLatitude": lat - 0.0015, "StationLongitude": lon + 0.008}]
    if wgs:
        coords.append({"CoordinateName": "WGS84", "CoordinateFormat": "decimal degrees",
                       "StationLatitude": lat, "StationLongitude": lon})
    gi = {"PeakGustSpeed": -99 if gust is None else gust,
          "Occurred_at": {"WindDirection": -99 if gust_dir is None else gust_dir,
                          "DateTime": "2026-10-16T16:43:00+08:00" if gust is not None else "-99"}}
    return {"StationName": name, "StationId": sid, "ObsTime": {"DateTime": t},
            "GeoInfo": {"Coordinates": coords, "StationAltitude": alt, "CountyName": county, "TownName": town,
                        "CountyCode": "68000", "TownCode": "68000080"},
            "WeatherElement": {"Weather": "晴", "Now": {"Precipitation": 0.0}, "WindDirection": d, "WindSpeed": spd,
                               "AirTemperature": 28.1, "RelativeHumidity": 70, "AirPressure": p, "GustInfo": gi}}


def _cwa_payload(*stations, success="true"):
    return {"success": success, "result": {"resource_id": "O-A0001-001", "fields": []},
            "records": {"Station": list(stations)}}


NEAR = (25.07, 121.22)       # 桃園機場一帶
ST_A = _cwa_station("C0C480", "桃園", 24.99, 121.31, 2.4, 60.0, gust=5.1, gust_dir=70.0)
ST_B = _cwa_station("466920", "臺北", 25.0377, 121.5149, 1.6, 110.0)
ST_C = _cwa_station("C0C700", "大園", 25.06, 121.20, -99, -99)           # 離得最近但沒有風值
ST_D = _cwa_station("C0C590", "新屋", 24.97, 121.05, "3.7", "250", p=-99, alt="-99")
ST_E = _cwa_station("C0A9F0", "大園機場", 25.08, 121.23, 3.0, 270.0, wgs=False)   # 沒有 WGS84 ⇒ 用第一組


def test_parse_takes_wgs84_turns_minus99_into_none_and_keeps_stations_without_wind():
    st = wf.parse_cwa_stations(_cwa_payload(ST_A, ST_B, ST_C, ST_D, ST_E,
                                            _cwa_station("X", "壞座標", -99, 121.0, 1, 1), "not a dict"))
    assert [s.station_id for s in st] == ["C0C480", "466920", "C0C700", "C0C590", "C0A9F0"]
    a = st[0]
    assert (a.lat, a.lon) == (24.99, 121.31) and a.alt_m == 12.0 and a.obs_time == "2026-10-16T17:00:00+08:00"
    assert (a.speed_mps, a.dir_from_deg, a.gust_mps, a.gust_dir_deg) == (2.4, 60.0, 5.1, 70.0)
    assert a.gust_time == "2026-10-16T16:43:00+08:00" and a.pressure_hpa == 1008.2 and a.county == "桃園市"
    assert st[1].gust_mps is None and st[1].gust_dir_deg is None and st[1].gust_time is None
    assert st[2].has_wind() is False and st[2].speed_mps is None
    assert st[3].speed_mps == 3.7 and st[3].dir_from_deg == 250.0 and st[3].pressure_hpa is None and st[3].alt_m is None
    assert (st[4].lat, st[4].lon) == pytest.approx((25.08 - 0.0015, 121.23 + 0.008)), "沒有 WGS84 時退回第一組座標"
    assert st[0].has_wind() and st[0].distance_m is None


@pytest.mark.parametrize("payload, msg", [
    ("[]", "不是 JSON 物件"), ({"success": "false", "message": "Invalid key"}, "Invalid key"),
    ({"success": "true", "records": {"Location": []}}, "records.Station"), ({"success": "true"}, "records.Station"),
])
def test_parse_is_loud_about_the_wrong_shape(payload, msg):
    with pytest.raises(wf.WindFetchError, match=msg):
        wf.parse_cwa_stations(payload)


#: NEAR → 桃園（24.99, 121.31）：球面餘弦定律（獨立於 haversine 的公式）算出 12702.5 m
DIST_NEAR_A = 12702.5


def test_haversine_against_an_independent_formula():
    assert wf.haversine_m(*NEAR, 24.99, 121.31) == pytest.approx(DIST_NEAR_A, abs=1.0)
    assert wf.haversine_m(*NEAR, 25.08, 121.23) == pytest.approx(1500.3, abs=1.0)
    assert wf.haversine_m(0, 0, 0, 1) == pytest.approx(111194.9, abs=1.0)
    assert wf.haversine_m(10, 20, 10, 20) == 0.0


def test_pick_by_id_is_case_insensitive_and_sets_distance_when_near_is_given():
    st = wf.parse_cwa_stations(_cwa_payload(ST_A, ST_B, ST_C, ST_D))
    s = wf.pick_cwa_station(st, station="c0c480", near=NEAR)
    assert s.station_id == "C0C480" and s.distance_m == pytest.approx(DIST_NEAR_A, abs=1.0) and s.matched_by == "id"
    assert wf.pick_cwa_station(st, station="466920").distance_m is None


def test_pick_by_name_exact_beats_substring_and_multiple_hits_need_a_reference_point():
    # 順序刻意：包含命中的「大園機場」離 NEAR 只有 1.5 km、全等的「大園」在 26 km 外且排最後；「園」四站都中
    st = wf.parse_cwa_stations(_cwa_payload(ST_A, ST_B, _cwa_station("Z", "大園國小", 25.00, 121.10, 1.0, 10.0), ST_E,
                                            _cwa_station("Q", "大園", 24.90, 121.40, 2.0, 90.0)))
    assert wf.pick_cwa_station(st, station="桃園").matched_by == "exact"
    q = wf.pick_cwa_station(st, station="大園", near=NEAR)
    assert q.station_id == "Q" and q.matched_by == "exact", "全等贏過 1.5 km 外的包含命中"
    assert wf.pick_cwa_station(st, station="大園").station_id == "Q", "沒有 near 也是全等優先"
    one = wf.pick_cwa_station(st, station="機場")
    assert one.station_id == "C0A9F0" and one.matched_by == "substring" and "名稱以包含比對命中" in one.summary()
    many = wf.pick_cwa_station(st, station="園", near=NEAR)
    assert many.station_id == "C0A9F0" and many.matched_by == "substring+nearest", "四站包含 ⇒ 離 near 最近（不是第一個）"
    assert "多站取最近" in many.summary()
    with pytest.raises(wf.WindFetchError) as ei:                 # 沒有 near ⇒ 不能靜默取第一個
        wf.pick_cwa_station(st, station="園")
    msg = str(ei.value)
    assert "名稱「園」有 4 站符合" in msg and "桃園（C0C480） 桃園市大園區" in msg and "大園（Q）" in msg \
        and "改填測站代碼或全名" in msg


def test_pick_nearest_skips_stations_without_wind_and_never_substitutes_silently():
    st = wf.parse_cwa_stations(_cwa_payload(ST_A, ST_B, ST_C, ST_D))
    s = wf.pick_cwa_station(st, near=NEAR)
    assert s.station_id == "C0C480" and s.matched_by == "nearest", "大園（C0C700）更近但沒有風值"
    assert "離作業區最近且有風值" in s.summary()
    with pytest.raises(wf.WindFetchError, match="沒有風值"):
        wf.pick_cwa_station(st, station="大園", near=NEAR)        # 明確指定的站沒有風值 ⇒ 不拿別站代替
    with pytest.raises(wf.WindFetchError, match="沒給測站也沒有"):
        wf.pick_cwa_station(st)
    with pytest.raises(wf.WindFetchError, match="都沒有風值"):
        wf.pick_cwa_station([st[2]], near=NEAR)
    with pytest.raises(wf.WindFetchError, match="找不到測站名稱「中壢」.*最近有風值的：桃園（C0C480） 12.7 km"):
        wf.pick_cwa_station(st, station="中壢", near=NEAR)
    with pytest.raises(wf.WindFetchError, match="代碼 C9Z999"):
        wf.pick_cwa_station(st, station="c9z999")


def test_long_station_text_is_truncated_in_errors():
    st = wf.parse_cwa_stations(_cwa_payload(ST_A))
    long = "CWA-12345678-1234-1234-1234-123456789ABC"
    with pytest.raises(wf.WindFetchError) as ei:
        wf.pick_cwa_station(st, station=long)
    assert long not in str(ei.value) and "…（共 40 字）" in str(ei.value)
    assert wf.short_text("桃園") == "桃園" and wf.short_text("x" * 13) == "x" * 12 + "…（共 13 字）"


def test_summary_and_arrow_say_what_the_number_is():
    s = wf.parse_cwa_stations(_cwa_payload(ST_A))[0]
    s = wf.pick_cwa_station([s], station="C0C480", near=NEAR)
    txt = s.summary()
    for piece in ("CWA 測站 桃園（C0C480） 觀測 2026-10-16T17:00:00+08:00：", "風 2.4 m/s 來自 60°", "陣風 5.1 m/s 來自 70°",
                  "氣壓 1008.2 hPa", "距作業區中心 12.7 km", "標準觀測高度 10 m", "CWA 未公布該站實際離地高度",
                  "非作業高度"):
        assert piece in txt, piece
    assert "約離地" not in txt, "CWA 沒公布風速計離地高度，不能說該站離地 10 m"
    row = s.to_arrow(-1)
    assert row == {"sysid": -1, "lat": 24.99, "lon": 121.31, "alt_rel": 0.0, "dir_from_deg": 60.0, "speed_mps": 2.4,
                   "label": "CWA 桃園 觀測 17:00"}
    assert "沒有風值" in wf.parse_cwa_stations(_cwa_payload(ST_C))[0].summary()


def test_direction_990_means_variable_not_270_degrees():
    """CWA 資料標準 V1.05：風向 990 ＝ 風向不定。取模會變 270°（西風）被寫進規劃器 —— 必須擋。"""
    st = wf.parse_cwa_stations(_cwa_payload(
        _cwa_station("V1", "不定站", 25.07, 121.22, 1.3, 990, gust=2.0, gust_dir=990),
        _cwa_station("V2", "怪值站", 25.071, 121.221, 1.0, 400.0, gust=1.5, gust_dir=-5.0),
        _cwa_station("V3", "故障站", 25.072, 121.222, "X", "X"),
        ST_A))
    v1, v2, v3 = st[0], st[1], st[2]
    assert v1.dir_from_deg is None and v1.dir_variable is True and v1.speed_mps == 1.3 and v1.has_wind() is False
    assert v1.gust_dir_deg is None and v1.gust_mps == 2.0
    assert "風 1.3 m/s、風向不定（990）" in v1.summary() and "270" not in v1.summary()
    assert v2.dir_from_deg is None and v2.dir_variable is False and v2.gust_dir_deg is None, "超出 0–360 不取模"
    assert v3.speed_mps is None and v3.dir_from_deg is None and "沒有風值（-99／X）" in v3.summary()
    with pytest.raises(wf.WindFetchError, match="風向不定（990）、風速 1.3 m/s"):
        wf.pick_cwa_station(st, station="不定站")
    assert wf.pick_cwa_station(st, near=(25.07, 121.22)).station_id == "C0C480", "最近三站都不能用 ⇒ 取有確定風向的"
    assert wf.parse_cwa_stations(_cwa_payload(_cwa_station("N", "北風站", 25.0, 121.0, 2.0, 360)))[0].dir_from_deg == 360.0


@pytest.mark.parametrize("s, ok",[("466920", True), ("C0C480", True), ("c1f9m1", True), ("桃園", False), ("", False),
                                   ("ABCDEFGH", False), ("12", False), ("C0C480 ", True)])
def test_station_id_detection(s, ok):
    assert wf.looks_like_cwa_station_id(s) is ok


def test_cwa_url_carries_the_key_only_in_the_query_and_redaction_hides_it():
    url = wf.build_cwa_url("CWA-12345678-ABCD", station_id="c0c480")
    q = urllib.parse.parse_qs(urllib.parse.urlparse(url).query)
    assert url.startswith(wf.CWA_STATION_URL) and q["Authorization"] == ["CWA-12345678-ABCD"]
    assert q["StationId"] == ["C0C480"] and q["format"] == ["JSON"]
    assert "StationId" not in wf.build_cwa_url("k")
    red = wf.redact_cwa_url(url)
    assert "CWA-12345678-ABCD" not in red and "Authorization=***" in red and "StationId=C0C480" in red
    with pytest.raises(ValueError):
        wf.build_cwa_url("")


def test_fetch_cwa_filters_by_id_on_the_server_but_picks_names_locally_and_never_leaks_the_key():
    seen = []

    def opener(url, timeout):
        seen.append(url)
        return _Resp(json.dumps(_cwa_payload(ST_A, ST_B)).encode("utf-8"))

    s = wf.fetch_cwa_station("CWA-SECRET-KEY", station="466920", opener=opener)
    assert s.station_id == "466920" and "StationId=466920" in seen[-1]
    s = wf.fetch_cwa_station("CWA-SECRET-KEY", station="桃園", near=NEAR, opener=opener)
    assert s.station_id == "C0C480" and "StationId" not in seen[-1] and s.distance_m is not None
    s = wf.fetch_cwa_station("CWA-SECRET-KEY", near=NEAR, opener=opener)
    assert s.station_id == "C0C480"

    def unauthorized(url, timeout):
        raise urllib.error.HTTPError(url, 401, "Unauthorized", {}, BytesIO(b""))

    with pytest.raises(wf.WindFetchError) as ei:
        wf.fetch_cwa_station("CWA-SECRET-KEY", station="466920", opener=unauthorized)
    assert "HTTP 401" in str(ei.value) and "授權碼" in str(ei.value) and "SECRET" not in str(ei.value)

    def down(url, timeout):
        raise urllib.error.URLError(f"no route to {url}")

    with pytest.raises(wf.WindFetchError) as ei:
        wf.fetch_cwa_station("CWA-SECRET-KEY", station="466920", opener=down)
    assert "連不到 CWA" in str(ei.value) and "SECRET" not in str(ei.value)

    def refused(url, timeout):
        return _Resp(json.dumps({"success": "false", "message": "api key invalid"}).encode("utf-8"))

    with pytest.raises(wf.WindFetchError, match="api key invalid"):
        wf.fetch_cwa_station("CWA-SECRET-KEY", station="466920", opener=refused)
    with pytest.raises(wf.WindFetchError, match="沒有 CWA API 授權碼"):
        wf.fetch_cwa_station("", station="466920", opener=opener)


def test_field_mean_wind_is_a_vector_mean():
    assert wf.field_mean_wind(wf.uniform_field((24.0, 120.0, 24.1, 120.1), 3.0, 270.0)) == pytest.approx((3.0, 270.0, 1))
    u, v = wf.dir_to_uv(270.0, 2.0)
    u2, v2 = wf.dir_to_uv(90.0, 2.0)
    f = wf.WindField(lats=(24.0, 24.1), lons=(120.0, 120.1), u=((u, u2), (math.nan, u)), v=((v, v2), (math.nan, v)),
                     height_m=10, source="x", label="x")
    s, d, n = wf.field_mean_wind(f)
    assert n == 3 and s == pytest.approx(2.0 / 3.0) and d == pytest.approx(270.0)
    empty = wf.WindField(lats=(24.0,), lons=(120.0,), u=((math.nan,),), v=((math.nan,),), height_m=10, source="x",
                         label="x", bbox=(24.0, 120.0, 24.1, 120.1))
    assert wf.field_mean_wind(empty) is None
