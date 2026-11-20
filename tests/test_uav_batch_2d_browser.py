# -*- coding: utf-8 -*-
"""tests/test_uav_batch_2d_browser.py — 2D 批次 UAV 更新在真的 Leaflet 頁面上的行為（無頭 Chrome）。

鎖的是 2026-10-17 效能改法的三個承諾：標記只建一次（第二批不重建 DOM）、位置／轉向／標籤跟著更新、尾跡累積。
要開瀏覽器：用 safe runner 時要設 SAFE_ALLOW_BROWSER=1，否則整檔 skip（同 tests/test_wind_map_pages.py）。
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

from tests.test_wind_map_pages import _bare2d, _folium_page_html, _route_local_only, _QT_OK  # noqa: E402
from tests.test_survey_map_layers import _pw, _launch  # noqa: E402,F401

pytestmark = pytest.mark.skipif(not _QT_OK, reason='Qt 匯入失敗')


def _item(sid, lat, lon, hdg, mode='AUTO', armed=True, vt='VTOL'):
    return {'sysid': sid, 'lat': lat, 'lon': lon, 'alt': 100.0, 'hdg': hdg, 'spd': 18.0,
            'mode': mode, 'armed': armed, 'vt': vt}


@pytest.fixture(scope='module')
def page(_pw, tmp_path_factory):
    w = _bare2d()
    f = tmp_path_factory.mktemp('uav2d') / 'folium.html'
    f.write_text(_folium_page_html(w), encoding='utf-8')
    try:
        b = _launch(_pw, ['--allow-file-access-from-files'])
        pg = b.new_page(viewport={'width': 800, 'height': 600})
        errs = []
        pg.on('pageerror', lambda e: errs.append(str(e)))
        _route_local_only(pg)
        pg.goto(f.as_uri())
        pg.wait_for_function("window.currentMap && typeof window.aeroUavUpdate === 'function'", timeout=30000)
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f'無法啟動無頭瀏覽器：{e}')
    pg._errs = errs
    yield pg
    b.close()


def test_markers_are_created_once_then_moved_rotated_and_relabelled(page):
    lat0, lon0 = 25.07, 121.21
    page.evaluate('(b) => window.aeroUavUpdate(b)', [_item(1, lat0, lon0, 10), _item(2, lat0 + 0.001, lon0, 20, vt='COPTER')])
    info = page.evaluate("""() => {
        // 探針放在**內層** .uav-rot（Leaflet 的 DivIcon.createIcon 會重用外層 DIV、只換 innerHTML，外層探針抓不到重建）
        // 另外把 L.divIcon 包起來數呼叫次數
        window.__divIconCalls = 0; const _orig = L.divIcon; L.divIcon = function(){ window.__divIconCalls++; return _orig.apply(this, arguments); };
        const ms = window._uavMarkers; const out = {};
        for (const k of Object.keys(ms)) { const el = ms[k].getElement(); el.querySelector('.uav-rot').dataset.probe = 'first';
          out[k] = { ll: ms[k].getLatLng(), rot: el.querySelector('.uav-rot').style.transform,
                     lbl: el.querySelector('.uav-lbl').textContent, trail: window._uavTrailPts[k].length,
                     img: el.querySelector('.uav-rot img').src }; }
        return out; }""")
    assert set(info) == {'1', '2'}
    assert abs(info['1']['ll']['lat'] - lat0) < 1e-9 and 'rotate(10deg)' in info['1']['rot']
    assert 'UAV1 [AUTO]' in info['1']['lbl'] and '✈' in info['1']['lbl'] and '\U0001F681' in info['2']['lbl']
    assert info['1']['trail'] == 1
    # 第二批：位置／轉向／標籤變了，但 DOM 節點是同一個（dataset.probe 還在 ⇒ 沒重建 divIcon）
    # 第二批：位置／轉向／標籤變了，機型也從 VTOL 變 COPTER（只換 img.src），但內層節點是同一個、L.divIcon 零呼叫
    page.evaluate('(b) => window.aeroUavUpdate(b)',
                  [_item(1, lat0 + 0.002, lon0 + 0.001, 95, mode='RTL', armed=False, vt='COPTER')])
    after = page.evaluate("""() => { const m = window._uavMarkers[1]; const el = m.getElement();
        return { probe: el.querySelector('.uav-rot').dataset.probe, ll: m.getLatLng(),
                 rot: el.querySelector('.uav-rot').style.transform, lbl: el.querySelector('.uav-lbl').textContent,
                 trail: window._uavTrailPts[1].length, trailLayer: !!window._uavTrails[1],
                 n: Object.keys(window._uavMarkers).length, divIconCalls: window.__divIconCalls,
                 img: el.querySelector('.uav-rot img').src }; }""")
    assert after['probe'] == 'first' and after['divIconCalls'] == 0, '第二批重建了 divIcon'
    assert abs(after['ll']['lat'] - (lat0 + 0.002)) < 1e-9 and 'rotate(95deg)' in after['rot']
    assert 'UAV1 [RTL]' in after['lbl'] and 'ARM' not in after['lbl'] and '\U0001F681' in after['lbl']
    assert after['img'] != info['1']['img'], '機型變了圖示要跟著換'
    assert after['trail'] == 2 and after['trailLayer'] and after['n'] == 2
    # 航跡方向：速度夠時用 crs 轉圖示
    page.evaluate('(b) => window.aeroUavUpdate(b)', [dict(_item(1, lat0, lon0, 10), crs=200.0)])
    assert 'rotate(200deg)' in page.evaluate("window._uavMarkers[1].getElement().querySelector('.uav-rot').style.transform")
    assert not [e for e in page._errs if 'aeroUav' in e or 'uav' in e.lower()], page._errs


def test_trail_is_capped_at_800_points(page):
    page.evaluate("""() => { for (let i = 0; i < 900; i++) window.aeroUavUpdate([{sysid: 3, lat: 25.07 + i * 1e-5, lon: 121.21,
        alt: 100, hdg: 0, spd: 1, mode: 'AUTO', armed: true, vt: 'VTOL'}]); }""")
    assert page.evaluate('window._uavTrailPts[3].length') == 800
