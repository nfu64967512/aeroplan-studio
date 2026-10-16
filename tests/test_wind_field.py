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
