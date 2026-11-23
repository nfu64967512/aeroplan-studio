"""HARVEST：每架預設降回**自己的起飛點**（2026-10-09 使用者：三架 land 會在同一點而不是起飛點，必須修正）。

之前逐機起飛站（vendored #7）只接了起飛側：GUI 3 架（坪相距 60 m）全部降在 home（= 中間那架的坪），
UAV-1／UAV-3 的起飛點與降落點差 60 m，三條最後進場腿疊在同一條線上。降落側的機制早就有
（vendored #4 ``recovery_site_id``），只是沒人預設去用。

全部用真的 HARVEST 計畫斷言**值**；每條先證明情境真的會觸發舊行為（起降點相距、坪不同）。
"""
from __future__ import annotations

import math
import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402
from utils.math_utils import spawn_positions  # noqa: E402

hv = pytest.importorskip('hv_dccpp.integration.aeroplan_adapter')

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
SPACING = 60.0


def _ll(east_m, north_m):
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def _hdist(a, b):
    dy = (b[0] - a[0]) * _M_LAT
    dx = (b[1] - a[1]) * 111412.0 * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)


def _areas():
    return [{'area_id': 1, 'priority': 1.0, 'scan_angle_deg': 0.0,
             'polygon': [_ll(3000, -600), _ll(6000, -600), _ll(6000, 600), _ll(3000, 600)]}]


def _drones(n=3, spacing=SPACING):
    pts = spawn_positions(HOME[0], HOME[1], n, 0.0, spacing)
    return [{'drone_id': i + 1, 'name': f'UAV-{i + 1}', 'position': pts[i], 'speed': 20.0,
             'heading': 0.0, 'turn_radius': 150.0, 'vehicle_type': 'vtol', 'fov_width': 160.0}
            for i in range(n)]


CLASSES = {1: 'A', 2: 'B', 3: 'B'}


@pytest.fixture(scope='module')
def planned():
    """GUI 同一條路：SwarmCoordinator（HARVEST）、1 Pioneer + 2 HERO、坪相距 60 m、home = 中間那架的坪。"""
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: CLASSES.get(int(d['drone_id']), 'A')
    c.hv_home = HOME
    c.harvest_params = {'min_separation_m': 100.0, 'alt_offset_m': 50.0, 'hv_waiver_max_agl_m': 200.0}
    res = c.plan_coverage_dccpp(areas=_areas(), drones=_drones(), coverage_width_m=160.0,
                                overlap_rate=0.25, coverage_altitude=160.0)
    assert res['assembled_paths'] and len(res['assembled_paths']) == 3, '前提：3 架都要有路徑'
    return c, res


def _pad_of(path):
    w = path.waypoints[0]
    return (w.lat, w.lon)


def test_precondition_pads_are_really_apart(planned):
    """前提：三個起飛點兩兩相距 ≈ 60 m 以上 —— 否則「降在自己的坪」與「降在同一點」分不出來。"""
    _, res = planned
    pads = [_pad_of(p) for p in res['assembled_paths'].values()]
    gaps = [_hdist(pads[i], pads[j]) for i in range(3) for j in range(i + 1, 3)]
    assert min(gaps) > 50.0, f'起飛點間距 {gaps}'


def test_each_uav_lands_exactly_where_it_took_off(planned):
    _, res = planned
    for uid, p in res['assembled_paths'].items():
        start, end = p.waypoints[0], p.waypoints[-1]
        d = _hdist((start.lat, start.lon), (end.lat, end.lon))
        assert d < 0.05, f'UAV-{uid} 起飛點與降落點相距 {d:.2f} m（應為 0）'


def test_landing_points_are_not_all_the_same_point(planned):
    """使用者的原話：三架 land 會在同一點。三個降落點必須兩兩分開。"""
    _, res = planned
    ends = [(p.waypoints[-1].lat, p.waypoints[-1].lon) for p in res['assembled_paths'].values()]
    gaps = [_hdist(ends[i], ends[j]) for i in range(3) for j in range(i + 1, 3)]
    assert min(gaps) > 50.0, f'有兩架降在幾乎同一點：兩兩間距 {gaps}'


def test_planner_landing_site_is_the_takeoff_site(planned):
    """讀回規劃器自己的 `UAVPlan.landing_site`（宣告偏好不等於保證）。"""
    c, _ = planned
    plan = c.hv_coordinator.last_plan
    assert plan.plans
    for uid, up in plan.plans.items():
        assert up.landing_site == up.takeoff_site, \
            f'{uid}：起飛站 {up.takeoff_site}、降落站 {up.landing_site}'
        assert str(up.landing_site).startswith('TO_'), f'{uid} 降在 {up.landing_site}（不是自己的起飛站）'


def test_upload_ends_with_vtol_land_on_the_uavs_own_pad(planned):
    """實際會送進 SITL 的任務：最後一項 NAV_VTOL_LAND(85) 的座標 = 該架自己的起飛點。"""
    c, res = planned
    tuples = c.hv_coordinator.upload_tuples(is_vtol=True)
    assert set(tuples) == set(res['assembled_paths'])
    for did, items in tuples.items():
        last = items[-1]
        assert last[3] == 85, f'UAV-{did} 最後一項不是 NAV_VTOL_LAND：{last}'
        pad = _pad_of(res['assembled_paths'][did])
        assert _hdist(pad, (last[0], last[1])) < 0.5, \
            f'UAV-{did} 的 VTOL_LAND 座標離自己的起飛點 {_hdist(pad, (last[0], last[1])):.1f} m'


def test_3d_landing_segment_ends_at_the_uavs_own_pad(planned):
    from mission.vtol_mission_exporter import VTOLMissionExporter
    _, res = planned
    data = {d['uav_id']: d for d in VTOLMissionExporter(transition_alt_m=40.0)
            .to_cesium_data(res['assembled_paths'], 120.0)}
    ends = []
    for uid, p in res['assembled_paths'].items():
        seg = data[uid]['landing_segment']
        assert seg[-1]['alt'] == 0
        assert _hdist(_pad_of(p), (seg[-1]['lat'], seg[-1]['lon'])) < 0.05
        assert _hdist((data[uid]['home']['lat'], data[uid]['home']['lon']),
                      (seg[-1]['lat'], seg[-1]['lon'])) < 0.05, '3D 降落點應等於該架的起飛點（home）'
        ends.append((seg[-1]['lat'], seg[-1]['lon']))
    assert min(_hdist(ends[i], ends[j]) for i in range(3) for j in range(i + 1, 3)) > 50.0


# ── 轉接層（不跑規劃，快）：預設 / 明確指定 / 開關 / no-op ──────────────────────
def _inputs(drones, **kw):
    notes: list = []
    inputs, id_map = hv.to_mission_inputs(
        _areas(), drones, coverage_width_m=160.0, overlap_rate=0.25, coverage_altitude=160.0,
        home=HOME, class_of=lambda d: CLASSES.get(int(d['drone_id']), 'A'), warnings=notes, **kw)
    return inputs, notes, {u.uav_id: u for u in inputs.fleet}


def test_default_recovery_site_is_each_uavs_own_start_site():
    _, notes, fleet = _inputs(_drones())
    assert set(fleet) == {'A1', 'B2', 'B3'}
    for uid, u in fleet.items():
        assert u.start_site_id == f'TO_{uid}'
        assert u.recovery_site_id == u.start_site_id, f'{uid}：recovery={u.recovery_site_id}'
    assert any('landing_at_takeoff_sites' in n for n in notes), '預設生效時要在 notes 說出來'


def test_explicit_recovery_site_wins_over_the_default():
    _, notes, fleet = _inputs(_drones(), recovery_sites={'B2': 'TO_A1', 3: 'H'})
    assert fleet['B2'].recovery_site_id == 'TO_A1'          # 用 uav_id 指定
    assert fleet['B3'].recovery_site_id == 'H'              # 用 drone_id 指定
    assert fleet['A1'].recovery_site_id == 'TO_A1'          # 沒指定者仍是自己的坪
    assert any('landing_at_takeoff_sites(A1' in n for n in notes)
    assert not any('landing_at_takeoff_sites' in n and 'B2' in n for n in notes), \
        '明確指定的機不應被說成「預設降回自己的起飛點」'


def test_unknown_explicit_recovery_site_still_refused_loudly():
    with pytest.raises(hv.AeroPlanAdapterError):
        _inputs(_drones(), recovery_sites={'B2': 'NOPE'})


def test_switch_off_restores_the_old_everyone_to_home_behaviour():
    _, notes, fleet = _inputs(_drones(), land_at_takeoff_site=False)
    assert all(u.recovery_site_id is None for u in fleet.values())
    assert all(u.start_site_id == f'TO_{uid}' for uid, u in fleet.items()), '起飛側不受開關影響'
    assert not any('landing_at_takeoff_sites' in n for n in notes)


def test_no_per_uav_sites_means_no_default_recovery():
    """逐機起飛站沒生效 ⇒ 沒有『自己的坪』可降 ⇒ 預設 no-op（全隊走 home，與舊行為相同）。"""
    # (a) 單機
    _, _, f1 = _inputs(_drones(1))
    assert all(u.recovery_site_id is None for u in f1.values())
    # (b) 多機但座標全相同（腳本情境：每台 position 都是同一個 HOME）
    same = _drones(3)
    for d in same:
        d['position'] = HOME
    _, _, f3 = _inputs(same)
    assert all(u.recovery_site_id is None and u.start_site_id is None for u in f3.values())
    # (c) 關掉逐機起飛站
    _, _, f4 = _inputs(_drones(), per_uav_takeoff_sites=False)
    assert all(u.recovery_site_id is None for u in f4.values())


# ── 讀回：偏好不等於保證 ──────────────────────────────────────────────────────
def test_warns_when_the_planner_could_not_honour_the_own_pad(monkeypatch):
    """規劃器把某架降到別的站（例：能量吃不下）⇒ 要明說，不能讓使用者在 3D 圖上才發現。"""
    import hv_dccpp.integration.aeroplan_adapter as ad

    real = ad.plan_mission
    victims: list = []

    def _sabotaged(*a, **k):
        plan = real(*a, **k)
        victim = sorted(plan.plans)[0]
        victims.append(victim)
        plan.plans[victim].landing_site = 'H'        # 假裝規劃器讓這一架退回 home
        return plan

    monkeypatch.setattr(ad, 'plan_mission', _sabotaged)
    coord = ad.HvDccppCoordinator(class_of=lambda d: CLASSES.get(int(d['drone_id']), 'A'),
                                  home=HOME, wind=(0.0, 0.0), reduce_waypoints=True)
    coord.plan_coverage_dccpp(areas=_areas(), drones=_drones(), coverage_width_m=160.0,
                              overlap_rate=0.25, coverage_altitude=160.0)
    hits = [n for n in coord.notes if 'landing_not_at_requested_site' in n]
    assert victims, '前提：有一架被改成降在 H'
    v = victims[0]
    assert len(hits) == 1 and v in hits[0] and f'TO_{v}' in hits[0], coord.notes


def test_no_warning_when_every_uav_lands_where_requested(planned):
    c, _ = planned
    assert not [n for n in c.hv_coordinator.notes if 'landing_not_at_requested_site' in n]


def test_coordinator_switch_off_makes_everyone_land_at_home():
    """`land_at_takeoff_site=False` 要一路傳到規劃：全隊退回舊行為（降在 home 的同一點）。

    同時證明「預設才是修正」：同一個情境，開關一關，起降點就又不一樣了。
    """
    coord = hv.HvDccppCoordinator(class_of=lambda d: CLASSES.get(int(d['drone_id']), 'A'),
                                  home=HOME, wind=(0.0, 0.0), land_at_takeoff_site=False)
    res = coord.plan_coverage_dccpp(areas=_areas(), drones=_drones(), coverage_width_m=160.0,
                                    overlap_rate=0.25, coverage_altitude=160.0)
    plan = coord.last_plan
    assert plan.plans and all(p.landing_site == 'H' for p in plan.plans.values()),         {u: p.landing_site for u, p in plan.plans.items()}
    ends = [(p.waypoints[-1].lat, p.waypoints[-1].lon) for p in res['assembled_paths'].values()]
    assert all(_hdist(ends[0], e) < 0.5 for e in ends), '關掉之後三架應降在同一點（舊行為）'
    starts = [_pad_of(p) for p in res['assembled_paths'].values()]
    assert max(_hdist(starts[0], s_) for s_ in starts) > 50.0, '前提：起飛點仍是各自分開的'
