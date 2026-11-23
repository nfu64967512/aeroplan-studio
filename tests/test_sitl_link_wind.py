"""mission.sitl_link：WIND（飛控 EKF 風估計）→ wind 信號／get_wind（地圖風場圖層的「飛機即時風」來源）。

只收**已確認是飛機的自駕儀元件**送的；每一則都送；NaN／inf／負風速整則丟掉（不靜默變 0）；
附上收到當下機卡的位置。訊息用 pymavlink MAVLink2 編碼再解碼（帶真的 sysid/compid），直接餵 _handle_message。
"""
from __future__ import annotations

import math

from tests.test_sitl_link_camera import cam_hb, gpos, hb, make_link, mk


def wind(src, comp, direction, speed, speed_z=0.0):
    return mk(src, comp, 'wind', direction, speed, speed_z)


def _wire(link):
    link._got_wind = []
    link.wind.connect(lambda s, d: link._got_wind.append((s, d)))
    return link


def test_wind_is_emitted_every_message_and_kept_per_sysid_with_position():
    link = _wire(make_link())
    link._handle_message(hb(1, 1))
    link._handle_message(wind(1, 1, 270.0, 3.0))                      # 還沒定位：位置 None
    link._handle_message(gpos(1, 1, 24.1234, 120.5678))
    link._handle_message(wind(1, 1, 272.5, 3.25, -0.5))
    link._handle_message(wind(1, 1, 272.5, 3.25, -0.5))               # 同值也送（風場要持續更新時間戳）
    assert [(s, d['dir_from_deg'], d['speed_mps']) for s, d in link._got_wind] == [
        (1, 270.0, 3.0), (1, 272.5, 3.25), (1, 272.5, 3.25)]
    first, last = link._got_wind[0][1], link._got_wind[-1][1]
    assert (first['lat'], first['lon'], first['alt_rel']) == (None, None, None)
    assert (round(last['lat'], 4), round(last['lon'], 4), last['alt_rel']) == (24.1234, 120.5678, 50.0)
    assert last['speed_z_mps'] == -0.5 and last['t'] > 0.0
    assert link._got_wind[2][1]['t'] >= link._got_wind[1][1]['t']
    got = link.get_wind(1)
    assert got['speed_mps'] == 3.25 and link.get_wind() == got     # 預設 sysid_label
    assert link.get_wind(2) is None
    got['speed_mps'] = 99.0
    assert link.get_wind(1)['speed_mps'] == 3.25                       # 回的是複本


def test_direction_is_normalised_to_0_360():
    link = _wire(make_link())
    link._handle_message(hb(1, 1))
    link._handle_message(wind(1, 1, -90.0, 1.0))
    link._handle_message(wind(1, 1, 450.0, 1.0))
    assert [d['dir_from_deg'] for _s, d in link._got_wind] == [270.0, 90.0]


def test_wind_from_camera_gcs_or_unknown_sysid_is_ignored():
    link = _wire(make_link())
    link._handle_message(hb(1, 1))
    link._handle_message(cam_hb(1, 100))
    link._handle_message(wind(1, 100, 10.0, 5.0))                      # 同 sysid 的相機元件
    link._handle_message(wind(255, 190, 10.0, 5.0))                    # 地面站
    link._handle_message(wind(7, 1, 10.0, 5.0))                        # 沒發過心跳的 sysid
    assert link._got_wind == [] and link.get_wind(1) is None and link.get_wind(7) is None


def test_non_finite_or_negative_wind_is_dropped_not_zeroed():
    link = _wire(make_link())
    link._handle_message(hb(1, 1))
    link._handle_message(wind(1, 1, math.nan, 2.0))
    link._handle_message(wind(1, 1, 10.0, math.inf))
    link._handle_message(wind(1, 1, 10.0, 2.0, math.nan))
    link._handle_message(wind(1, 1, 10.0, -1.0))
    assert link._got_wind == [] and link.get_wind(1) is None
    link._handle_message(wind(1, 1, 10.0, 2.0))
    assert link.get_wind(1)['speed_mps'] == 2.0


def test_sysid_label_is_accepted_before_its_first_heartbeat():
    link = _wire(make_link(sysid_label=3))
    link._handle_message(wind(3, 1, 180.0, 4.0))
    assert [(s, d['dir_from_deg'], d['speed_mps']) for s, d in link._got_wind] == [(3, 180.0, 4.0)]
    assert link.get_wind()['lat'] is None
