"""core.wind_field 分層風（10／80／120 m）：URL 組裝、真實回應解析、同一時間步、缺層、水平雙線性、最接近層（不連網）。

錄製的固定資料（tests/fixtures/，2026-10-19 06:34 UTC 實打 api.open-meteo.com，無任何金鑰）：
* ``open_meteo_profile_icon_global_2x3_20261019T06.json``：2×3 格點的**多點回應（JSON list）**，
  請求參數與 build_open_meteo_profile_url 組出來的完全相同（只差參數順序，見 RECORDED_URL_QUERY）。
* ``open_meteo_profile_icon_global_single_4h_20261019.json``：單點（dict）、past_hours=1＋forecast_hours=3 共 4 個小時，
  含 180 m ⇒ 證明 icon_global 逐時 10／80／120／180 m 都有值，並拿來測「挑哪一個小時」。
* ``open_meteo_profile_two_models_single_20261019T06.json``：models=icon_global,gfs_global ⇒ 鍵帶 ``_<model>`` 後綴。

慣例：dir_from_deg ＝ 來向（氣象）；u ＝ −s·sin(dir)、v ＝ −s·cos(dir)。
"""
from __future__ import annotations

import copy
import datetime
import json
import math
import os
import pathlib
import urllib.error
import urllib.parse
import urllib.request
from io import BytesIO

import pytest

from core import wind_field as wf

FIX = pathlib.Path(__file__).resolve().parent / 'fixtures'
GRID_2X3 = FIX / 'open_meteo_profile_icon_global_2x3_20261019T06.json'
SINGLE_4H = FIX / 'open_meteo_profile_icon_global_single_4h_20261019.json'
TWO_MODELS = FIX / 'open_meteo_profile_two_models_single_20261019T06.json'
LATS, LONS = [23.40, 23.60], [120.30, 120.45, 120.60]
#: 錄製 GRID_2X3 時實際送出的查詢參數（值鎖：URL 組裝改了就要重錄固定資料）
RECORDED_URL_QUERY = (
    'latitude=23.40000%2C23.40000%2C23.40000%2C23.60000%2C23.60000%2C23.60000'
    '&longitude=120.30000%2C120.45000%2C120.60000%2C120.30000%2C120.45000%2C120.60000'
    '&hourly=wind_speed_10m%2Cwind_direction_10m%2Cwind_speed_80m%2Cwind_direction_80m'
    '%2Cwind_speed_120m%2Cwind_direction_120m'
    '&wind_speed_unit=ms&timezone=UTC&models=icon_global&forecast_hours=1')


@pytest.fixture(autouse=True)
def _no_network(monkeypatch):
    """這個檔的測試絕不連網：預設 opener 一被叫到就失敗。"""
    def boom(*a, **k):
        raise AssertionError('測試不准連網')
    monkeypatch.setattr(urllib.request, 'urlopen', boom)


def _load(p):
    return json.loads(p.read_text(encoding='utf-8'))


def _uv(d, s):
    return wf.dir_to_uv(d, s)


def _close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# ── 常數 ──────────────────────────────────────────────────────────────
def test_profile_constants_are_the_three_levels_the_user_chose_and_one_explicit_model():
    assert wf.PROFILE_HEIGHTS_M == (10.0, 80.0, 120.0)
    assert wf.PROFILE_MODEL_DEFAULT == 'icon_global'
    assert wf.build_open_meteo_profile_url.__kwdefaults__ == {'heights': wf.PROFILE_HEIGHTS_M, 'model': 'icon_global'}
    assert wf.parse_open_meteo_profile.__kwdefaults__['model'] == 'icon_global'
    assert wf.fetch_open_meteo_profile.__kwdefaults__['opener'] is None


# ── URL ───────────────────────────────────────────────────────────────
def test_profile_url_matches_the_recorded_request_exactly():
    url = wf.build_open_meteo_profile_url(LATS, LONS)
    assert url.startswith(wf.OPEN_METEO_URL + '?')
    got = urllib.parse.parse_qs(urllib.parse.urlparse(url).query)
    assert got == urllib.parse.parse_qs(RECORDED_URL_QUERY)


def test_profile_url_is_all_hourly_single_model_same_point_layout_as_the_grid_url():
    url = wf.build_open_meteo_profile_url([24.0, 24.1], [120.0, 120.5, 121.0])
    q = urllib.parse.parse_qs(urllib.parse.urlparse(url).query)
    grid = urllib.parse.parse_qs(urllib.parse.urlparse(
        wf.build_open_meteo_url([24.0, 24.1], [120.0, 120.5, 121.0], '10m')).query)
    assert q['latitude'] == grid['latitude'] and q['longitude'] == grid['longitude']
    assert q['hourly'] == ['wind_speed_10m,wind_direction_10m,wind_speed_80m,wind_direction_80m,'
                           'wind_speed_120m,wind_direction_120m']
    assert 'current' not in q                                     # 三層都從同一個逐時時間步拿
    assert q['models'] == ['icon_global'] and q['wind_speed_unit'] == ['ms'] and q['timezone'] == ['UTC']
    assert q['forecast_hours'] == ['1']
    q2 = urllib.parse.parse_qs(urllib.parse.urlparse(wf.build_open_meteo_profile_url(
        [24.0, 24.1], [120.0, 120.5], heights=[180, 10.0], model='gfs_global')).query)
    assert q2['hourly'] == ['wind_speed_10m,wind_direction_10m,wind_speed_180m,wind_direction_180m']
    assert q2['models'] == ['gfs_global']


@pytest.mark.parametrize('heights', [(50.0,), (10.0, 100.0), (), (10.0, 10.0), (True,), ('80',), (math.nan,),
                                     (80.5,), (-10.0,)])
def test_profile_url_rejects_heights_outside_10_80_120_180(heights):
    with pytest.raises(ValueError):
        wf.build_open_meteo_profile_url([24.0, 24.1], [120.0, 120.5], heights=heights)


@pytest.mark.parametrize('model', ['', 'best_match&hourly=x', 'ICON GLOBAL', None, 'a/b'])
def test_profile_url_rejects_model_names_that_are_not_plain_identifiers(model):
    with pytest.raises(ValueError):
        wf.build_open_meteo_profile_url([24.0, 24.1], [120.0, 120.5], model=model)


def test_profile_url_caps_point_count():
    with pytest.raises(ValueError):
        wf.build_open_meteo_profile_url([float(i) for i in range(11)], [float(i) for i in range(10)])
    with pytest.raises(ValueError):
        wf.build_open_meteo_profile_url([], [120.0])


# ── 解析：真實回應（多點 list） ───────────────────────────────────────
def test_parse_recorded_multi_location_list_into_three_layers():
    raw = _load(GRID_2X3)
    assert isinstance(raw, list) and len(raw) == 6              # 多點回應是 JSON list
    p = wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:34:32Z')
    assert p.heights() == [10.0, 80.0, 120.0] and p.model == 'icon_global' and p.time_utc == '2026-10-19T06:00'
    assert p.bbox == (23.40, 120.30, 23.60, 120.60)
    for f in p.layers:
        h = f'{f.height_m:g}'
        assert f.label == f'Open-Meteo 預報（icon_global，非實測）{h} m 風，2026-10-19T06:00 UTC；6 個模式格點'
        assert f.source == 'open-meteo' and f.time_utc == '2026-10-19T06:00'
        assert (f.lats, f.lons) == (tuple(LATS), tuple(LONS))
    # 每一格都是同一個時間步的原值（逐點逐層對照錄製檔）
    for k, it in enumerate(raw):
        i, j = divmod(k, 3)
        hr = it['hourly']
        assert hr['time'] == ['2026-10-19T06:00']
        for f in p.layers:
            h = f'{f.height_m:g}'
            eu, ev = _uv(hr[f'wind_direction_{h}m'][0], hr[f'wind_speed_{h}m'][0])
            assert _close(f.u[i][j], eu) and _close(f.v[i][j], ev)
    # 抽一格寫死的值（不是從同一份資料推回來的）：第一點 10 m 4.06 m/s 來自 308°
    s, d = math.hypot(p.layers[0].u[0][0], p.layers[0].v[0][0]), wf.uv_to_dir(p.layers[0].u[0][0], p.layers[0].v[0][0])[0]
    assert round(s, 6) == 4.06 and round(d, 6) == 308.0
    assert '非實測' in p.label and 'icon_global' in p.label and '2026-10-19T06:00 UTC' in p.label
    assert '10／80／120 m' in p.label and '6 個模式格點' in p.label
    joined = '\n'.join(p.notes)
    assert '不內插' in joined and '不外推' in joined and '離地' in joined and '同一個逐時時間步' in joined
    assert 'best_match' in joined and '要求 6 點，模式實際對應 6 個格點' in joined
    assert '不是本機時鐘' not in joined                           # 06:34 落在 06:00 這個小時，沒有退回


def test_recorded_layers_are_accepted_by_the_page_checker():
    from ui.widgets import wind_layer as wl
    p = wf.parse_open_meteo_profile(_load(GRID_2X3), LATS, LONS, when_utc='2026-10-19T06:00')
    for f in p.layers:
        d = wl.check_wind_field(f.to_js())
        assert d['height_m'] == f.height_m and d['label'] == f.label
        assert f.to_page() == f.to_js()                         # 契約裡的 to_page 是同一個 dict
        json.dumps(d, allow_nan=False)


# ── 時間步：三層一律同一個小時 ─────────────────────────────────────────
def _four(path):
    one = _load(path)
    assert isinstance(one, dict)                               # 單點回應是 dict
    return [copy.deepcopy(one) for _ in range(4)]


@pytest.mark.parametrize('when, idx', [
    ('2026-10-19T05:00', 0),
    ('2026-10-19T07:20:00Z', 2),
    ('2026-10-19T15:59:59+08:00', 2),                          # 有時區 ⇒ 先換成 UTC（07:59Z）
    (datetime.datetime(2026, 10, 19, 8, 1), 3),                # naive ⇒ 當 UTC
    (datetime.datetime(2026, 10, 19, 14, 30, tzinfo=datetime.timezone(datetime.timedelta(hours=8))), 1),
])
def test_hourly_index_follows_the_utc_hour_and_all_levels_share_it(when, idx):
    raw = _four(SINGLE_4H)
    hr = raw[0]['hourly']
    p = wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], when_utc=when)
    assert p.time_utc == hr['time'][idx]
    for f in p.layers:
        h = f'{f.height_m:g}'
        eu, ev = _uv(hr[f'wind_direction_{h}m'][idx], hr[f'wind_speed_{h}m'][idx])
        assert all(_close(f.u[i][j], eu) and _close(f.v[i][j], ev) for i in range(2) for j in range(2))
        assert f.time_utc == hr['time'][idx]
    assert '不是本機時鐘' not in '\n'.join(p.notes)


def test_hour_not_in_response_falls_back_to_the_servers_first_hour_and_says_so():
    raw = _four(SINGLE_4H)
    p = wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], when_utc='2026-10-20T00:10Z')
    assert p.time_utc == '2026-10-19T05:00'
    assert round(math.hypot(p.layers[0].u[0][0], p.layers[0].v[0][0]), 6) == 2.2     # 05:00 的 10 m
    assert any('2026-10-19T05:00 UTC 不是本機時鐘的 2026-10-20T00:00 UTC' in n for n in p.notes)


def test_default_when_is_now_and_the_one_hour_response_is_used():
    before = datetime.datetime.now(datetime.timezone.utc)
    p = wf.parse_open_meteo_profile(_load(GRID_2X3), LATS, LONS)
    after = datetime.datetime.now(datetime.timezone.utc)
    assert p.time_utc == '2026-10-19T06:00'
    if before.replace(minute=0, second=0, microsecond=0) == after.replace(minute=0, second=0, microsecond=0):
        is_that_hour = before.strftime('%Y-%m-%dT%H') == '2026-10-19T06'
        assert any('不是本機時鐘' in n for n in p.notes) is (not is_that_hour)


def test_points_with_different_valid_times_are_refused_not_mixed():
    raw = _four(SINGLE_4H)
    raw[3]['hourly']['time'] = ['2026-10-19T09:00', '2026-10-19T10:00', '2026-10-19T11:00', '2026-10-19T12:00']
    with pytest.raises(wf.WindFetchError, match='預報時間不一致'):
        wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], when_utc='2026-10-19T06:00')


def test_recorded_icon_global_also_has_180m_when_asked():
    raw = _four(SINGLE_4H)
    p = wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], heights=(10, 80, 120, 180),
                                    when_utc='2026-10-19T06:00')
    assert p.heights() == [10.0, 80.0, 120.0, 180.0]
    top = p.layer(180)
    assert round(math.hypot(top.u[0][0], top.v[0][0]), 6) == 3.26
    assert p.label.startswith('Open-Meteo 預報（icon_global，非實測）分層 10／80／120／180 m 風，2026-10-19T06:00 UTC')


# ── 缺層／缺點 ────────────────────────────────────────────────────────
def test_level_missing_from_the_response_is_dropped_with_a_note_and_not_interpolated():
    raw = _load(GRID_2X3)
    for it in raw:
        del it['hourly']['wind_speed_80m'], it['hourly']['wind_direction_80m']
    p = wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')
    assert p.heights() == [10.0, 120.0] and p.layer(80) is None
    assert '分層 10／120 m 風' in p.label and '80 m 無資料已略過' in p.label
    assert any(n.startswith('80 m：回應沒有這一層（模式 icon_global 不提供 wind_speed_80m）') and '不從其他高度內插或外推' in n
               for n in p.notes)
    # 最接近 80 m 的仍是**真的那兩層之一**（一樣近取低的），不是 10／120 的內插
    s, d, used = wf.profile_wind_at(p, 23.4, 120.3, 65.0)
    assert used == 10.0 and round(s, 6) == 4.06 and round(d, 6) == 308.0


def test_level_with_only_nulls_is_dropped_with_a_different_note():
    raw = _load(GRID_2X3)
    for it in raw:
        it['hourly']['wind_speed_120m'] = [None]
    p = wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')
    assert p.heights() == [10.0, 80.0]
    assert any(n.startswith('120 m：模式 icon_global 這個時間步在 6 個點都沒有風值，已略過') for n in p.notes)
    assert '120 m 無資料已略過' in p.label


def test_partly_missing_level_keeps_the_layer_with_nan_cells():
    raw = _load(GRID_2X3)
    raw[4]['hourly']['wind_direction_120m'] = [None]
    raw[5]['hourly']['wind_speed_120m'] = [-1.0]                # 負風速不合法 ⇒ 當缺值
    raw[0]['hourly']['wind_direction_80m'] = [400]             # 0–360 以外 ⇒ 缺值（不取模）
    raw[1]['hourly']['wind_speed_80m'] = [True]                # bool 不是數字
    p = wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')
    top, mid = p.layer(120), p.layer(80)
    assert math.isnan(top.u[1][1]) and math.isnan(top.u[1][2]) and math.isfinite(top.u[0][0])
    assert math.isnan(mid.u[0][0]) and math.isnan(mid.u[0][1])
    assert '120 m：2 點沒有風值（顯示略過）' in p.notes and '80 m：2 點沒有風值（顯示略過）' in p.notes
    assert top.to_js()['u'][1][1] is None
    json.dumps(top.to_js(), allow_nan=False)


def test_zero_layers_is_an_error_in_traditional_chinese():
    raw = _load(GRID_2X3)
    for it in raw:
        for h in (10, 80, 120):
            it['hourly'][f'wind_speed_{h}m'] = [None]
    with pytest.raises(wf.WindFetchError, match=r'Open-Meteo（模式 icon_global）在 10／80／120 m 都沒有風值'):
        wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')


# ── 多模式回應：只認指定的那一個，不拿別的模式頂替 ─────────────────────
def test_suffixed_keys_pick_only_the_requested_model():
    raw = _four(TWO_MODELS)
    hr = raw[0]['hourly']
    ic = wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], when_utc='2026-10-19T06:00')
    gf = wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], model='gfs_global',
                                     when_utc='2026-10-19T06:00')
    assert round(wf.uv_to_dir(ic.layers[0].u[0][0], ic.layers[0].v[0][0])[0], 6) == hr['wind_direction_10m_icon_global'][0] == 305
    assert round(wf.uv_to_dir(gf.layers[0].u[0][0], gf.layers[0].v[0][0])[0], 6) == hr['wind_direction_10m_gfs_global'][0] == 273
    assert 'gfs_global' in gf.label and 'icon_global' not in gf.label
    with pytest.raises(wf.WindFetchError, match='都沒有風值'):
        wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], model='ecmwf_ifs025',
                                    when_utc='2026-10-19T06:00')


def test_suffixed_key_wins_over_an_unsuffixed_one():
    raw = _four(TWO_MODELS)
    for it in raw:
        it['hourly']['wind_speed_10m'] = [99.0]
        it['hourly']['wind_direction_10m'] = [0]
    p = wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], when_utc='2026-10-19T06:00')
    assert round(math.hypot(p.layers[0].u[0][0], p.layers[0].v[0][0]), 6) == 3.16


# ── 不只相信 URL：單位與時差要核對（不換算、不猜） ───────────────────────
def _kmh(raw, model_suffix=''):
    """把風速換成 km/h（值 ×3.6、單位寫 km/h）：Open-Meteo 預設單位就是 km/h，少了 wind_speed_unit=ms 就會這樣回。"""
    for it in raw:
        for h in (10, 80, 120):
            k = f'wind_speed_{h}m{model_suffix}'
            if k in it['hourly']:
                it['hourly'][k] = [None if v is None else v * 3.6 for v in it['hourly'][k]]
                it['hourly_units'][k] = 'km/h'
    return raw


def test_recorded_responses_carry_m_per_s_degrees_and_utc():
    for path in (GRID_2X3, SINGLE_4H, TWO_MODELS):
        raw = _load(path)
        for it in (raw if isinstance(raw, list) else [raw]):
            assert it['utc_offset_seconds'] == 0
            u = it['hourly_units']
            assert {v for k, v in u.items() if k.startswith('wind_speed_')} == {'m/s'}
            assert {v for k, v in u.items() if k.startswith('wind_direction_')} == {'°'}


def test_km_per_hour_payload_is_refused_not_read_as_m_per_s():
    raw = _kmh(_load(GRID_2X3))
    with pytest.raises(wf.WindFetchError, match=r"第 1 點的風速 wind_speed_10m 單位是 'km/h'，不是 'm/s'"):
        wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')


def test_wrong_direction_unit_is_refused():
    raw = _load(GRID_2X3)
    raw[2]['hourly_units']['wind_direction_80m'] = 'rad'
    with pytest.raises(wf.WindFetchError, match=r"第 3 點的風向 wind_direction_80m 單位是 'rad'"):
        wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')


def test_units_are_checked_on_the_requested_models_suffixed_keys_only():
    raw = _kmh(_four(TWO_MODELS), '_gfs_global')                 # 只有 gfs_global 換成 km/h
    p = wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], when_utc='2026-10-19T06:00')
    assert p.heights() == [10.0, 80.0, 120.0], "icon_global 自己的單位是 m/s：照收"
    with pytest.raises(wf.WindFetchError, match='單位是'):
        wf.parse_open_meteo_profile(raw, [23.0, 23.1], [120.0, 120.1], model='gfs_global', when_utc='2026-10-19T06:00')


def test_local_time_payload_is_refused_instead_of_labelled_utc():
    raw = _load(GRID_2X3)
    for it in raw:
        it['utc_offset_seconds'] = 28800
        it['timezone'] = 'Asia/Taipei'
        it['hourly']['time'] = ['2026-10-19T14:00']
    with pytest.raises(wf.WindFetchError, match=r"第 1 點的回應不是 UTC 時間（utc_offset_seconds=28800、timezone='Asia/Taipei'）"):
        wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')
    raw = _load(GRID_2X3)
    raw[0]['utc_offset_seconds'] = 'x'
    with pytest.raises(wf.WindFetchError, match='不是 UTC 時間'):
        wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')
    raw = _load(GRID_2X3)
    raw[0]['hourly_units'] = 'm/s'
    with pytest.raises(wf.WindFetchError, match='hourly_units 不是物件'):
        wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')


def test_missing_units_or_offset_are_read_as_requested_but_said_to_be_unchecked():
    raw = _load(GRID_2X3)
    for it in raw:
        del it['hourly_units'], it['utc_offset_seconds']
    p = wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')
    assert p.heights() == [10.0, 80.0, 120.0]
    assert '6 處回應沒有附單位（hourly_units）：依請求的 wind_speed_unit=ms 解讀，未能核對' in p.notes
    assert '6 點的回應沒有 utc_offset_seconds：時間依請求的 timezone=UTC 解讀，未能核對' in p.notes
    q = wf.parse_open_meteo_profile(_load(GRID_2X3), LATS, LONS, when_utc='2026-10-19T06:00')
    assert not [n for n in q.notes if '未能核對' in n], "錄製回應兩項都有：不寫「未能核對」"


# ── 解析度與地形：說清楚模式格點多粗、高度基準是什麼 ─────────────────────
def test_module_doc_states_the_measured_icon_global_grid_not_1_to_11_km():
    assert '1–11 km' not in wf.__doc__ and '0.125°' in wf.__doc__
    assert wf.PROFILE_MODEL_GRID_DEG == {'icon_global': 0.125}


def test_recorded_coordinates_really_sit_on_a_0125_degree_grid():
    for it in _load(GRID_2X3):
        for k in ('latitude', 'longitude'):
            assert abs(it[k] / 0.125 - round(it[k] / 0.125)) < 1e-9, it[k]


def test_notes_give_the_grid_spacing_cell_count_and_dem_elevation_range():
    p = wf.parse_open_meteo_profile(_load(GRID_2X3), LATS, LONS, when_utc='2026-10-19T06:00')
    km_ew = 0.125 * 111.32 * math.cos(math.radians(23.5))
    assert (f'模式 icon_global 的格點間距 0.125°（這裡約 13.9 km × {km_ew:.1f} km；'
            '2026-10-19 實測回應座標，Open-Meteo 文件標示 ~11 km）') in p.notes
    assert ('回應的 elevation 是要求點的 90 m DEM 高程 8–1003 m（不是模式地表）；'
            '各層的「離地」是相對模式格點地表，與實際地形可能差很多') in p.notes
    assert not [n for n in p.notes if '全部對應同一個模式格點' in n], "6 點對 6 格：沒有「全部同一格」"
    # 4 點吸附到同一格 ⇒ 範圍內各點的風一樣，圖上的差異不是梯度
    one = wf.parse_open_meteo_profile(_four(SINGLE_4H), [23.0, 23.1], [120.0, 120.1], when_utc='2026-10-19T06:00')
    assert ('要求的 4 點全部對應同一個模式格點 ⇒ 範圍內各點的風完全相同；圖上看到的差異不是模式解析出來的風場梯度'
            in one.notes)
    # 沒實測過的模式：不寫數字；量得到才寫回應裡的最小間距
    gf = wf.parse_open_meteo_profile(_four(TWO_MODELS), [23.0, 23.1], [120.0, 120.1], model='gfs_global',
                                     when_utc='2026-10-19T06:00')
    assert '模式 gfs_global 的格點間距沒有實測過（這次只對應到 1 個格點，量不出間距）' in gf.notes
    # 沒實測過的模式名（單一模式回應的鍵不帶後綴 ⇒ 照收）：6 個格點 ⇒ 寫回應裡量到的最小間距
    two = wf.parse_open_meteo_profile(_load(GRID_2X3), LATS, LONS, model='ecmwf_ifs025', when_utc='2026-10-19T06:00')
    assert '模式 ecmwf_ifs025 的格點間距沒有實測過；這次回應吸附到的格點最小間距 0.125°' in two.notes
    partial = _load(GRID_2X3)
    partial[1]['latitude'], partial[1]['longitude'] = partial[0]['latitude'], partial[0]['longitude']
    pp = wf.parse_open_meteo_profile(partial, LATS, LONS, when_utc='2026-10-19T06:00')
    assert ('要求的 6 點只對應 5 個模式格點 ⇒ 範圍內的變化只來自這幾個格點的分界，不是解析出來的風場梯度'
            in pp.notes)


# ── 結構錯誤 ──────────────────────────────────────────────────────────
def _bad(mut):
    raw = _load(GRID_2X3)
    out = mut(raw)
    return raw if out is None else out


@pytest.mark.parametrize('mut, msg', [
    (lambda r: 'oops', '回應格式不對'),
    (lambda r: r[:5], '回了 5 筆，要求的是 6 點'),
    (lambda r: {'error': True, 'reason': 'Cannot initialize WeatherVariable'}, '拒絕請求：Cannot initialize'),
    (lambda r: r[:5] + [{'error': True, 'reason': 'rate limited'}], '第 6 點回應錯誤'),
    (lambda r: r[:5] + ['x'], '第 6 點回應錯誤'),
    (lambda r: r[0].pop('hourly') and None, '第 1 點沒有 hourly 區塊'),
    (lambda r: r[2]['hourly'].update(time='2026-10-19T06:00'), '第 3 點的 hourly.time'),
    (lambda r: r[2]['hourly'].update(time=[]), '第 3 點的 hourly.time'),
    (lambda r: r[1]['hourly'].update(wind_speed_80m=[1.0, 2.0]), '第 2 點的 wind_speed_80m 長度 2 與時間 1 筆不符'),
    (lambda r: r[1]['hourly'].update(wind_direction_10m=5), '第 2 點的 wind_direction_10m 長度 int'),
])
def test_malformed_payloads_raise_wind_fetch_error_in_chinese(mut, msg):
    with pytest.raises(wf.WindFetchError, match=msg):
        wf.parse_open_meteo_profile(_bad(mut), LATS, LONS, when_utc='2026-10-19T06:00')


def test_caller_mistakes_are_value_errors():
    raw = _load(GRID_2X3)
    with pytest.raises(ValueError):
        wf.parse_open_meteo_profile(raw, [23.4], [120.3, 120.45, 120.6, 120.7, 120.8, 120.9])   # 1 列
    with pytest.raises(ValueError):
        wf.parse_open_meteo_profile(raw, [23.6, 23.4], LONS)
    with pytest.raises(ValueError):
        wf.parse_open_meteo_profile(raw, LATS, LONS, heights=(10, 50))
    with pytest.raises(ValueError):
        wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='not a time')


# ── WindProfile 本身 ──────────────────────────────────────────────────
def _grid_field(h, u, v, lats=(0.0, 1.0), lons=(0.0, 1.0)):
    return wf.WindField(lats=tuple(lats), lons=tuple(lons), u=tuple(tuple(r) for r in u),
                        v=tuple(tuple(r) for r in v), height_m=float(h), source='open-meteo', label=f'{h} m')


def _const(h, s, d, lats=(0.0, 1.0), lons=(0.0, 1.0)):
    u, v = _uv(d, s)
    return _grid_field(h, [[u] * len(lons)] * len(lats), [[v] * len(lons)] * len(lats), lats, lons)


def _prof(*layers):
    return wf.WindProfile(layers=tuple(layers), model='icon_global', time_utc='2026-10-19T06:00', label='x',
                          notes=('n',), bbox=(0.0, 0.0, 1.0, 1.0))


def test_wind_profile_accessors_and_json_summary():
    p = _prof(_const(10, 1.0, 270), _const(80, 2.0, 280), _const(120, 3.0, 290))
    assert p.heights() == [10.0, 80.0, 120.0]
    assert p.layer(80).height_m == 80.0 and p.layer(80.0) is p.layer(80)
    assert p.layer(81) is None and p.layer(math.nan) is None and p.layer('80') is None
    js = p.to_json()
    s = json.dumps(js, allow_nan=False, ensure_ascii=False)
    assert js['heights_m'] == [10.0, 80.0, 120.0] and js['model'] == 'icon_global' and js['interpolation'] == 'none'
    assert js['bbox'] == [0.0, 0.0, 1.0, 1.0] and [round(L['max_speed_mps'], 9) for L in js['layers']] == [1.0, 2.0, 3.0]
    assert 'NaN' not in s and '離地' in s
    nan_layer = _grid_field(10, [[math.nan, math.nan], [math.nan, math.nan]], [[math.nan] * 2] * 2)
    js2 = _prof(nan_layer).to_json()
    assert js2['layers'][0]['max_speed_mps'] is None and js2['layers'][0]['n_valid'] == 0
    json.dumps(js2, allow_nan=False)


def test_wind_profile_rejects_empty_or_unsorted_layers():
    with pytest.raises(ValueError):
        _prof()
    with pytest.raises(ValueError):
        _prof(_const(80, 1, 0), _const(10, 1, 0))
    with pytest.raises(ValueError):
        _prof(_const(80, 1, 0), _const(80, 1, 0))
    with pytest.raises(ValueError):
        wf.WindProfile(layers=({'height_m': 10},), model='m', time_utc='t', label='x')


# ── 柱狀剖面：水平雙線性（同引擎） ─────────────────────────────────────
def test_column_is_bilinear_on_u_v_like_the_engine():
    f = _grid_field(10, [[0.0, 2.0], [4.0, 6.0]], [[1.0, 1.0], [-3.0, 5.0]])
    p = _prof(f)
    col = wf.profile_column(p, 0.25, 0.5)
    # 獨立寫法：u = u00 + fi(u10−u00) + fj(u01−u00) + fi·fj(u11−u10−u01+u00)
    fi, fj = 0.25, 0.5
    eu = 0.0 + fi * (4.0 - 0.0) + fj * (2.0 - 0.0) + fi * fj * (6.0 - 4.0 - 2.0 + 0.0)
    ev = 1.0 + fi * (-3.0 - 1.0) + fj * (1.0 - 1.0) + fi * fj * (5.0 - (-3.0) - 1.0 + 1.0)
    d, s = wf.uv_to_dir(eu, ev)
    assert len(col) == 1 and col[0]['h_m'] == 10.0
    assert _close(col[0]['speed_mps'], s) and _close(col[0]['dir_from_deg'], d)
    assert _close(eu, 2.0)


def test_column_uses_vector_interpolation_and_picks_the_right_cell_on_a_3x3_grid():
    # 東半邊西風、西半邊東風 ⇒ 正中間向量相消：0 m/s（不是 2 m/s 的純量平均）
    w = _uv(270, 2.0)
    e = _uv(90, 2.0)
    f = _grid_field(10, [[e[0], w[0]], [e[0], w[0]]], [[e[1], w[1]], [e[1], w[1]]])
    c = wf.profile_column(_prof(f), 0.5, 0.5)
    assert _close(c[0]['speed_mps'], 0.0, 1e-12) and c[0]['dir_from_deg'] == 0.0
    # 3×3：落在右上那一格（i=1, j=1），四角 u = 10, 20, 30, 40
    lats, lons = (0.0, 1.0, 3.0), (0.0, 2.0, 4.0)
    U = [[0, 0, 0], [0, 10, 20], [0, 30, 40]]
    V = [[0] * 3] * 3
    g = _grid_field(80, U, V, lats, lons)
    c2 = wf.profile_column(_prof(g), 2.0, 3.0)           # fi = (2−1)/2 = .5、fj = (3−2)/2 = .5
    assert _close(c2[0]['speed_mps'], 25.0) and _close(c2[0]['dir_from_deg'], 270.0)
    # 正好在格點、上緣、右緣都算在範圍內
    assert _close(wf.profile_column(_prof(g), 1.0, 2.0)[0]['speed_mps'], 10.0)
    assert _close(wf.profile_column(_prof(g), 3.0, 4.0)[0]['speed_mps'], 40.0)


def test_column_on_recorded_data_at_a_grid_node_returns_the_raw_values_per_level():
    p = wf.parse_open_meteo_profile(_load(GRID_2X3), LATS, LONS, when_utc='2026-10-19T06:00')
    col = wf.profile_column(p, 23.6, 120.6)                    # 第 6 點（右上角）
    assert [c['h_m'] for c in col] == [10.0, 80.0, 120.0]
    assert [(round(c['speed_mps'], 6), round(c['dir_from_deg'], 6)) for c in col] == [
        (2.13, 311.0), (2.33, 313.0), (2.19, 313.0)]
    json.dumps(col, allow_nan=False)
    assert set(col[0]) == {'h_m', 'speed_mps', 'dir_from_deg'}


def test_column_omits_layers_without_data_at_that_point():
    a = _const(10, 1.0, 0)
    b = _grid_field(80, [[math.nan, 1.0], [1.0, 1.0]], [[0.0, 0.0], [0.0, 0.0]])   # 左下角缺值
    c = _const(120, 3.0, 90)
    p = _prof(a, b, c)
    assert [x['h_m'] for x in wf.profile_column(p, 0.1, 0.1)] == [10.0, 120.0]
    assert [x['h_m'] for x in wf.profile_column(p, 0.0, 1.0)] == [10.0, 120.0]   # 邊上仍用到同一格的缺角
    assert wf.profile_column(p, 1.5, 0.5) == []                 # 範圍外：不外推
    assert wf.profile_column(p, math.nan, 0.5) == [] and wf.profile_column(p, 0.5, None) == []


# ── 最接近的一層（不內插、一樣近取低的） ──────────────────────────────
@pytest.mark.parametrize('h, used', [
    (-5.0, 10.0), (0.0, 10.0), (10.0, 10.0), (44.0, 10.0), (45.0, 10.0), (45.5, 80.0), (80.0, 80.0),
    (99.9, 80.0), (100.0, 80.0), (100.1, 120.0), (120.0, 120.0), (500.0, 120.0)])
def test_nearest_layer_with_ties_to_the_lower_one(h, used):
    p = _prof(_const(10, 1.0, 270), _const(80, 2.0, 280), _const(120, 3.0, 290))
    s, d, u = wf.profile_wind_at(p, 0.5, 0.5, h)
    assert u == used
    exp_s, exp_d = {10.0: (1.0, 270.0), 80.0: (2.0, 280.0), 120.0: (3.0, 290.0)}[used]
    assert _close(s, exp_s) and _close(d, exp_d)                # 原層的值：不是層間內插、不是外推


def test_nearest_layer_skips_layers_without_data_at_that_point_and_reports_which_one():
    nan80 = _grid_field(80, [[math.nan] * 2] * 2, [[math.nan] * 2] * 2)
    p = _prof(_const(10, 1.0, 270), nan80, _const(120, 3.0, 290))
    assert [x['h_m'] for x in wf.profile_column(p, 0.5, 0.5)] == [10.0, 120.0]
    s, d, used = wf.profile_wind_at(p, 0.5, 0.5, 80.0)          # 80 缺 ⇒ 距 10 是 70、距 120 是 40 ⇒ 用 120
    assert used == 120.0 and _close(s, 3.0) and _close(d, 290.0)
    assert wf.profile_wind_at(p, 0.5, 0.5, 60.0)[2] == 10.0     # 50 vs 60
    assert wf.profile_wind_at(p, 0.5, 0.5, 65.0)[2] == 10.0     # 55 vs 55 一樣近 ⇒ 低的
    assert wf.profile_wind_at(p, 0.5, 0.5, 66.0)[2] == 120.0    # 56 vs 54


def test_nearest_layer_none_cases():
    p = _prof(_const(10, 1.0, 270))
    assert wf.profile_wind_at(p, 0.5, 0.5, math.nan) is None
    assert wf.profile_wind_at(p, 0.5, 0.5, None) is None
    assert wf.profile_wind_at(p, 5.0, 0.5, 10.0) is None        # 範圍外
    r = wf.profile_wind_at(p, 0.5, 0.5, 120)
    assert r[2] == 10.0 and json.dumps(list(r), allow_nan=False)


# ── fetch（假 opener） ────────────────────────────────────────────────
class _Resp:
    def __init__(self, body):
        self._b = body

    def read(self):
        return self._b

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


BBOX = (23.40, 120.30, 23.60, 120.60)


def test_fetch_builds_the_profile_url_and_parses_the_recorded_bytes():
    seen = {}

    def opener(url, timeout):
        seen['url'], seen['timeout'] = url, timeout
        return _Resp(GRID_2X3.read_bytes())

    p = wf.fetch_open_meteo_profile(BBOX, nx=3, ny=2, opener=opener, timeout_s=7.0)
    assert seen['timeout'] == 7.0
    assert urllib.parse.parse_qs(urllib.parse.urlparse(seen['url']).query) == urllib.parse.parse_qs(RECORDED_URL_QUERY)
    assert p.heights() == [10.0, 80.0, 120.0] and p.time_utc == '2026-10-19T06:00' and p.bbox == BBOX
    assert isinstance(p, wf.WindProfile)


def _raise(exc):
    def op(url, timeout):
        raise exc
    return op


@pytest.mark.parametrize('opener, msg', [
    (_raise(urllib.error.URLError('no route')), '連不到 Open-Meteo'),
    (_raise(urllib.error.HTTPError('u', 429, 'Too Many Requests', {}, BytesIO(b''))), 'HTTP 429：Too Many Requests'),
    (_raise(urllib.error.HTTPError('u', 400, 'Bad Request', {}, BytesIO(
        b'{"error":true,"reason":"Data corrupted at path \'\'. Cannot initialize ForecastVariable"}'))),
     r'HTTP 400：Bad Request（Data corrupted'),
    (_raise(TimeoutError('timed out')), '連不到 Open-Meteo'),
    (lambda url, timeout: _Resp(b'<html>'), '不是 JSON'),
    (lambda url, timeout: _Resp(b'{"error": true, "reason": "bad params"}'), '拒絕請求：bad params'),
])
def test_fetch_wraps_errors_in_chinese(opener, msg):
    with pytest.raises(wf.WindFetchError, match=msg):
        wf.fetch_open_meteo_profile(BBOX, nx=3, ny=2, opener=opener)


def test_fetch_validates_arguments_before_any_request():
    calls = []

    def opener(url, timeout):
        calls.append(url)
        return _Resp(b'[]')

    with pytest.raises(ValueError):
        wf.fetch_open_meteo_profile(BBOX, heights=(10, 100), opener=opener)
    with pytest.raises(ValueError):
        wf.fetch_open_meteo_profile(BBOX, model='best_match&x', opener=opener)
    with pytest.raises(ValueError):
        wf.fetch_open_meteo_profile(BBOX, nx=1, opener=opener)
    assert calls == []


# ── 與顯示引擎同一套內插（真的瀏覽器跑 wind_flow.js 的 AeroWind.interp；要 SAFE_ALLOW_BROWSER=1） ──
WIND_JS = pathlib.Path(__file__).resolve().parents[1] / 'ui' / 'widgets' / 'web_shared' / 'wind_flow.js'


@pytest.mark.skipif(os.environ.get('SAFE_ALLOW_BROWSER') != '1', reason='真瀏覽器測試要設 SAFE_ALLOW_BROWSER=1')
def test_column_matches_the_engine_interp_in_a_real_browser(tmp_path):
    sync_api = pytest.importorskip('playwright.sync_api')
    raw = _load(GRID_2X3)
    raw[4]['hourly']['wind_speed_80m'] = [None]                # 讓 80 m 有一個缺角
    p = wf.parse_open_meteo_profile(raw, LATS, LONS, when_utc='2026-10-19T06:00')
    rnd = __import__('random').Random(20261019)
    pts = [(la, lo) for la in LATS for lo in LONS]                                     # 格點
    pts += [(23.4, 120.5), (23.6, 120.37), (23.5, 120.3), (23.5, 120.6), (23.6, 120.6)]   # 邊上、角上
    pts += [(23.39999, 120.4), (23.5, 120.60001), (24.0, 121.0)]                        # 範圍外
    pts += [(rnd.uniform(23.38, 23.62), rnd.uniform(120.28, 120.62)) for _ in range(60)]
    html = tmp_path / 'interp.html'
    html.write_text(f'<!doctype html><html><head><meta charset="utf-8"></head><body>'
                    f'<script src="{WIND_JS.as_uri()}"></script></body></html>', encoding='utf-8')
    with sync_api.sync_playwright() as pw:
        try:
            b = pw.chromium.launch(channel='chrome', headless=True)
        except Exception:
            b = pw.chromium.launch(headless=True)
        try:
            pg = b.new_page()
            errs = []
            pg.on('pageerror', lambda e: errs.append(str(e)))
            pg.route('**/*', lambda r: r.continue_() if r.request.url.startswith('file:') else r.abort())
            pg.goto(html.as_uri())
            pg.wait_for_function("typeof AeroWind === 'object'", timeout=10000)
            for f in p.layers:
                js = pg.evaluate("([f, pts]) => pts.map(q => AeroWind.interp(f, q[0], q[1]))",
                                 [f.to_js(), [list(q) for q in pts]])
                n_null = 0
                for q, got in zip(pts, js):
                    col = {c['h_m']: c for c in wf.profile_column(p, *q)}
                    if got is None:
                        n_null += 1
                        assert f.height_m not in col, (f.height_m, q)
                        continue
                    d, s = wf.uv_to_dir(got['u'], got['v'])
                    c = col[f.height_m]
                    assert abs(c['speed_mps'] - s) < 1e-9 and (abs(c['dir_from_deg'] - d) < 1e-7 or s < 1e-9), (q, c, got)
                assert n_null >= 3                                   # 範圍外三點；80 m 另有缺角那幾格
            assert not errs, errs
        finally:
            b.close()
