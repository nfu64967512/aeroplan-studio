"""投放彈道模組的離線測試（不需要 PyQt / MAVLink / 地圖）。

紀律：每一條都斷言一個**會因為模型寫錯而改變**的量，而不是只斷言「有回傳東西」。
特別驗「真空自由落體的解析解」那一條 —— 那是唯一能證明積分本身沒寫錯的檢查。
"""
import math

import pytest

from mission import ballistics as B


def test_vacuum_freefall_matches_closed_form():
    """k=0 時應該精確等於 h = ½gt²。這條是積分正確性的唯一硬檢查。"""
    h = 500.0
    tr = B.simulate(25.0, 121.0, h, 0.0, 0.0, 0.0, ground_msl=0.0, kind='dumb')
    # 暫時把阻力關掉來比對解析解
    saved = B.STORE_KINDS['dumb']['k']
    B.STORE_KINDS['dumb']['k'] = 0.0
    try:
        tr = B.simulate(25.0, 121.0, h, 0.0, 0.0, 0.0, ground_msl=0.0, kind='dumb')
    finally:
        B.STORE_KINDS['dumb']['k'] = saved
    expected = math.sqrt(2 * h / B.G)
    assert abs(tr.tof - expected) < 0.02, (tr.tof, expected)
    # 靜止投放 ⇒ 落點應在正下方
    assert tr.range_m < 0.5
    assert abs(tr.impact_speed - B.G * expected) < 0.5


def test_drag_slows_the_fall():
    """有阻力一定掉得比真空慢，且高阻力比低阻力更慢。"""
    h = 500.0
    dumb = B.simulate(25.0, 121.0, h, 0, 0, 0, ground_msl=0.0, kind='dumb')
    smoke = B.simulate(25.0, 121.0, h, 0, 0, 0, ground_msl=0.0, kind='smoke')
    vac = math.sqrt(2 * h / B.G)
    assert dumb.tof > vac
    assert smoke.tof > dumb.tof, (smoke.tof, dumb.tof)


def test_terminal_velocity_matches_sqrt_g_over_k():
    """夠高的投放，落地速度應接近終端速度 sqrt(g/k)。"""
    for kind in ('dumb', 'smoke'):
        k = B.STORE_KINDS[kind]['k']
        vt = math.sqrt(B.G / k)
        tr = B.simulate(25.0, 121.0, 6000.0, 0, 0, 0, ground_msl=0.0, kind=kind)
        assert abs(tr.impact_speed - vt) / vt < 0.10, (kind, tr.impact_speed, vt)


def test_forward_throw_scales_with_release_speed():
    """前拋距離要隨投放速度單調增加（速度是它唯一的來源）。"""
    prev = -1.0
    for gs in (0.0, 20.0, 40.0, 80.0):
        tr = B.simulate(25.0, 121.0, 300.0, gs, 0.0, 0.0,
                        ground_msl=0.0, kind='dumb')
        assert tr.range_m > prev, (gs, tr.range_m, prev)
        prev = tr.range_m


def test_release_direction_follows_velocity_vector():
    """往北投 -> 落點在北；往東投 -> 落點在東。方向搞反這條會抓到。"""
    lat, lon, h = 25.0, 121.0, 300.0
    north = B.simulate(lat, lon, h, 60.0, 0.0, 0.0, ground_msl=0.0)
    east = B.simulate(lat, lon, h, 0.0, 60.0, 0.0, ground_msl=0.0)
    assert north.impact_lat > lat and abs(north.impact_lon - lon) < 1e-6
    assert east.impact_lon > lon and abs(east.impact_lat - lat) < 1e-6


def test_downward_release_lands_sooner_than_level():
    """vd 是**向下為正**（與 MAVLink vz 同號）。號搞反這條會抓到。"""
    level = B.simulate(25.0, 121.0, 300.0, 0, 0, 0.0, ground_msl=0.0)
    diving = B.simulate(25.0, 121.0, 300.0, 0, 0, 20.0, ground_msl=0.0)
    climbing = B.simulate(25.0, 121.0, 300.0, 0, 0, -20.0, ground_msl=0.0)
    assert diving.tof < level.tof < climbing.tof


def test_ground_msl_is_respected():
    """地面不在海平面時，落點高度要等於給定的地面高，飛行時間要變短。"""
    low = B.simulate(25.0, 121.0, 1000.0, 0, 0, 0, ground_msl=0.0)
    high = B.simulate(25.0, 121.0, 1000.0, 0, 0, 0, ground_msl=800.0)
    assert abs(high.impact_alt - 800.0) < 1e-6
    assert high.tof < low.tof


def test_release_below_ground_is_a_noop():
    """已經在地面（或參數不合理）不應該跑出一條奇怪的軌跡。"""
    tr = B.simulate(25.0, 121.0, 10.0, 30.0, 0, 0, ground_msl=50.0)
    assert tr.tof == 0.0
    assert tr.impact_alt == 50.0


def test_samples_are_monotonic_and_end_at_impact():
    tr = B.simulate(25.0, 121.0, 400.0, 45.0, 10.0, 0.0, ground_msl=12.0)
    ts = [s[0] for s in tr.samples]
    alts = [s[3] for s in tr.samples]
    assert ts == sorted(ts)
    assert all(alts[i] >= alts[i + 1] - 1e-6 for i in range(len(alts) - 1))
    assert abs(tr.samples[-1][3] - 12.0) < 1e-6
    assert abs(tr.samples[-1][0] - tr.tof) < 1e-6
    assert abs(tr.samples[-1][1] - tr.impact_lat) < 1e-9
    assert len(tr.samples) > 10          # 給地圖內插的點要夠密


def test_as_dict_is_json_safe():
    import json
    d = B.simulate(25.0, 121.0, 300.0, 40.0, 0, 0, ground_msl=0.0).as_dict()
    json.loads(json.dumps(d))
    assert d['kind'] == 'dumb'
    assert len(d['impact']) == 3


def test_simulate_from_frame_uses_home_elevation_by_default():
    """預設地面高 = alt_msl − alt_rel（起飛場高），落點高度要等於它。"""
    class F:
        lat, lon = 25.0, 121.0
        alt_msl, alt_rel = 1200.0, 300.0     # 場高 = 900
        heading, ground_speed, climb = 0.0, 50.0, 0.0
        vel_n, vel_e, vel_d = 50.0, 0.0, 0.0
    tr = B.simulate_from_frame(F())
    assert abs(tr.impact_alt - 900.0) < 1e-6
    assert tr.impact_lat > 25.0          # 往北飛 -> 落點偏北


def test_simulate_from_frame_falls_back_to_heading_when_no_velocity():
    """沒有速度分量時退回 地速×航向；⚠️ 這條退路有風時會偏，但不能整個不動。"""
    class F:
        lat, lon = 25.0, 121.0
        alt_msl, alt_rel = 500.0, 500.0
        heading, ground_speed, climb = 90.0, 60.0, 0.0     # 正東
    tr = B.simulate_from_frame(F())
    assert tr.impact_lon > 121.0
    assert abs(tr.impact_lat - 25.0) < 1e-4


def test_kinds_are_ordered_by_drag():
    """煙霧彈的阻力必須大於傻瓜彈，否則兩個選項沒有區別。"""
    assert B.STORE_KINDS['smoke']['k'] > B.STORE_KINDS['dumb']['k']
