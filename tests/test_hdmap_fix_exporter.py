"""航測建圖修正包 F1（匯出器／轉接層）：R1 #2、#3 與 R2 的整數參數、預估張數、206 p1、錯開起飛。

每一條都是**值層級**：先在修正前重現（紅），修正後通過（綠）。

(a) 引入段 NAV 的接受半徑：param2 = r_acc = 30 ⇒ 206 開在引入點前 30 m 就執行（開拍點在帶起點後 20 m，不是 50 m；
    引入段 ≤ 30 m 時首張落在轉彎裡）。修正：引入段 NAV param2 = 1（ArduPlane 取 LOWBYTE(param2) 為接受距離，
    0 會退回 WP_RADIUS／轉彎提前量）。以 core.recon.exposure_verifier.simulate_mission_photos 驗首張位置。
(b) 相對高度基準：ArduPlane 解鎖時把 home 重設成飛機自己的位置（AP_Arming.cpp update_home）⇒ 相對高度必須以
    **這架的起飛坪**為基準，不是規劃 home。斜坡起降坪情境（R1 test_probe_home_alt）＋平地不變。
(c) 整數型參數：AIRSPEED_MIN/MAX、WP_LOITER_RAD、WP_RADIUS、RTL_RADIUS、Q_RTL_ALT 是 AP_Int16，26.6 會被飛控存成 26；
    CAM_MAX_ROLL 也是 AP_Int16 ⇒ CameraTriggerConfig 拒絕小數。
(d) expected_photo_count：考慮 50 Hz AP_Camera tick（實際間距 = ceil(d/(v·Δt))·v·Δt）與各 NAV 的接受半徑。
(e) 206 p1：平地條帶 AGL 因座標換算抖動 µm 級 ⇒ d 差幾個 float32 ulp；AGL 與規劃作業高度差 ≤ 1 cm 時用作業高度。
(f) HvDccppCoordinator.takeoff_schedule()：{drone_id: t_takeoff_s − min}；export_all 摘要帶同樣的錯開；預設不插 NAV_DELAY。
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import sys

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("hv_dccpp")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_hdmap_gui_adapter as T  # noqa: E402

from hv_dccpp import ardupilot as ap  # noqa: E402
from hv_dccpp.ardupilot import CameraTriggerConfig, recommended_params  # noqa: E402
from hv_dccpp.integration import aeroplan_adapter as ad  # noqa: E402
from hv_dccpp.integration.aeroplan_adapter import (  # noqa: E402
    AeroPlanAdapterError, HvDccppCoordinator, expected_photo_count, mission_item_dicts,
)
from hv_dccpp.mission import MappingModeConfig, Mode, Phase, PlannerConfig  # noqa: E402
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER  # noqa: E402

CC = T.CC
LEAD = T.LEAD
NAVS = (16, 17, 18, 19, 21, 22, 31, 84, 85)


def _dist(a, b):
    return ad._ground_dist_m(a, b)


@pytest.fixture(scope="module")
def hv_map():
    return T._plan(T._coord(MappingModeConfig(lead_in_m=LEAD), camera_cfg=CC))


# ── 斜坡起降坪（R1 test_probe_home_alt 的設定）：作業區平地 100 m，起降區東西向約 4° 的坡 ──
def _slope_terrain(lat, lon):
    n = (lat - T.HOME[0]) * T._M_LAT
    e = (lon - T.HOME[1]) * T._M_LON
    w = min(1.0, max(0.0, (-250.0 - n) / 100.0))
    return 100.0 + 0.07 * min(150.0, max(-150.0, e)) * w


def _flat_terrain(lat, lon):
    return 100.0


def _terrain_coord(terrain):
    from hv_dccpp.mission import PayloadRequirement
    req = PayloadRequirement("default", "rgb", gsd_required_m=0.05, side_overlap=0.70, forward_overlap=0.80,
                             h_target_agl_m=120.0)
    hv = HvDccppCoordinator(
        class_of=lambda d: T.CLASSES[int(d["drone_id"])],
        platform_a=dataclasses.replace(PLATFORM_PIONEER, min_fw_operation_agl_m=100.0), platform_b=PLATFORM_HERO,
        sensor_a=(T.HQ6,), sensor_b=(T.HQ6,), payload_reqs={"default": req},
        cfg=PlannerConfig(dem_step_m=25.0, dh_band_min_m=15.0), home=T._ll(0.0, -400.0), wind=(0.0, 0.0),
        waiver_max_agl_m=200.0, dem_margin_m=3000.0, mapping_mode=MappingModeConfig(lead_in_m=LEAD), camera_cfg=CC)
    hv.plan_coverage_dccpp(T._areas(), T._drones(), coverage_width_m=125.7, overlap_rate=0.70,
                           coverage_altitude=120.0, terrain_func=terrain)
    return hv


@pytest.fixture(scope="module")
def hv_slope():
    return _terrain_coord(_slope_terrain)


@pytest.fixture(scope="module")
def hv_flat():
    return _terrain_coord(_flat_terrain)


def _pads(hv):
    """{drone_id: (uav_id, UAVPlan, pad_msl, planning_home_msl)}；只收有任務的。"""
    ctx = hv.last_ctx
    out = {}
    for did in T._flying(hv):
        uid = hv.id_map[did]
        up = hv.last_plan.plans[uid]
        out[did] = (uid, up, ap._site_msl(ctx, up.takeoff_site), ap._home_msl(ctx))
    return out


def _strip_windows(items):
    """每條帶：(條帶起點 NAV, 引入 NAV, 206 開)；206 開 = p1 > 0 且 p3 == 1。"""
    out = []
    for i, it in enumerate(items):
        if it["command"] == 206 and it["param1"] > 0 and it["param3"] == 1.0:
            prev = [x for x in items[:i] if x["command"] in NAVS and x["seq"] > 0]
            out.append((prev[-2], prev[-1], it))
    return out


def _simulate(items, uid, cc=CC):
    from core.recon.exposure_verifier import simulate_mission_photos
    spd = [it["param2"] for it in items if it["command"] == 178 and it["param1"] == 0.0 and it["param2"] > 0]
    return simulate_mission_photos(items, uav=uid, airspeed_mps=spd[0], wind=(0.0, 0.0),
                                   intrval_min_s=cc.interval_min_s, trigger_duration_s=cc.duration_s)


# ═══════════════════════════ (a) 引入段 NAV 接受半徑 ═══════════════════════════
def test_lead_in_nav_accepts_at_one_metre_and_strip_start_keeps_r_acc(hv_map):
    ctx = hv_map.last_ctx
    seen = 0
    for did in T._flying(hv_map):
        up = hv_map.last_plan.plans[hv_map.id_map[did]]
        r_acc = ap.accept_radius_m(ctx.specs[up.cls].r_eff(0.0))
        wins = _strip_windows(hv_map.mission_items(did, camera_cfg=CC))
        assert len(wins) == len(up.strips)
        for start, lead, _on in wins:
            assert lead["command"] == 16 and lead["param2"] == ap.LEAD_IN_ACCEPT_RADIUS_M == 1.0
            assert start["param2"] == pytest.approx(r_acc) and r_acc == 30.0
            assert _dist((start["lat"], start["lon"]), (lead["lat"], lead["lon"])) == pytest.approx(LEAD, abs=0.5)
            seen += 1
    assert seen >= 10


def test_first_photo_lands_at_least_lead_in_after_the_strip_start(hv_map):
    """飛前模擬（exposure_verifier）：每條帶第一張落在帶起點後 ≥ lead_in − 2 m（修正前 ≈ 20 m）。"""
    n = 0
    for did in T._flying(hv_map):
        uid = hv_map.id_map[did]
        items = hv_map.mission_items(did, camera_cfg=CC)
        sim = _simulate(items, uid)
        wins = _strip_windows(items)
        first = {}
        for ph in sim.photos:
            if ph.strip is not None and ph.strip not in first:
                first[ph.strip] = (ph.lat_deg, ph.lon_deg)
        assert len(first) == len(wins), uid
        for k, (start, _lead, _on) in enumerate(wins, start=1):
            off = _dist((start["lat"], start["lon"]), first[str(k)])
            assert LEAD - 2.0 <= off <= LEAD + 3.0, (uid, k, off)
            n += 1
    assert n >= 10


def test_short_lead_in_no_longer_shoots_before_the_strip_start(hv_map):
    """引入段 20 m（≤ 舊的 30 m 接受半徑）：修正前首張在帶起點**之前**（轉彎裡），修正後在帶起點後 ≥ 18 m。"""
    cc = dataclasses.replace(CC, camera_lead_in_m=20.0)
    did = T._flying(hv_map)[0]
    uid = hv_map.id_map[did]
    up = hv_map.last_plan.plans[uid]
    items = mission_item_dicts(ap.plan_to_mission_items(up, hv_map.last_ctx, hv_map.cfg, camera_cfg=cc))
    sim = _simulate(items, uid, cc)
    wins = _strip_windows(items)
    first = {}
    for ph in sim.photos:
        if ph.strip is not None and ph.strip not in first:
            first[ph.strip] = ph
    for k, (start, lead, _on) in enumerate(wins, start=1):
        ph = first[str(k)]
        d_start = _dist((start["lat"], start["lon"]), (ph.lat_deg, ph.lon_deg))
        d_lead = _dist((lead["lat"], lead["lon"]), (ph.lat_deg, ph.lon_deg))
        # 首張在引入點附近（−1 m 接受半徑 … +0.1 s 的 10 Hz 任務更新延遲 ≈ +2 m），不是在帶起點之前的轉彎裡
        assert 18.0 <= d_start <= 23.0 and d_lead <= 3.0, (k, d_start, d_lead)


def test_zero_lead_in_opens_the_camera_at_the_strip_start_not_30_m_before(hv_map):
    """引入段 0 m（有 camera_cfg）：206 開掛在帶起點 NAV 之後 ⇒ 帶起點也要 1 m 接受半徑。
    修正前 r_acc 30 m ⇒ 首張落在帶起點前 28–30 m（轉彎裡，p3=1 那張不受 CAM_MAX_ROLL 擋）。"""
    cc = dataclasses.replace(CC, camera_lead_in_m=0.0)
    n = 0
    for did in T._flying(hv_map):
        uid = hv_map.id_map[did]
        up = hv_map.last_plan.plans[uid]
        items = mission_item_dicts(ap.plan_to_mission_items(up, hv_map.last_ctx, hv_map.cfg, camera_cfg=cc))
        sim = _simulate(items, uid, cc)
        first = {}
        for ph in sim.photos:
            if ph.strip is not None and ph.strip not in first:
                first[ph.strip] = (ph.lat_deg, ph.lon_deg)
        wins = _strip_windows(items)
        assert len(wins) == len(up.strips) and len(first) == len(wins)
        for k, (_before, start, _on) in enumerate(wins, start=1):
            assert start["command"] == 16 and start["param2"] == ap.LEAD_IN_ACCEPT_RADIUS_M, (uid, k)
            assert _dist((start["lat"], start["lon"]), first[str(k)]) <= 3.5, (uid, k)
            n += 1
    assert n >= 10
    # 沒給 camera_cfg（非建圖、舊行為）：帶起點仍是 r_acc
    did = T._flying(hv_map)[0]
    up = hv_map.last_plan.plans[hv_map.id_map[did]]
    old = mission_item_dicts(ap.plan_to_mission_items(up, hv_map.last_ctx, hv_map.cfg))
    assert {w[1]["param2"] for w in _strip_windows(old)} == {30.0}


# ═══════════════════════════ (b) 相對高度以這架的起飛坪為基準 ═══════════════════════════
def test_sloped_pads_reference_every_relative_altitude_to_the_own_pad(hv_slope):
    pads = _pads(hv_slope)
    ctx = hv_slope.last_ctx
    offsets = [pad - home for (_u, _up, pad, home) in pads.values()]
    assert max(abs(o) for o in offsets) > 3.0          # 前提：真的有斜坡（R1：±4.2 m）
    for did, (uid, up, pad, _home) in pads.items():
        rel = hv_slope.mission_items(did, camera_cfg=CC)
        msl = mission_item_dicts(ap.plan_to_mission_items(up, ctx, hv_slope.cfg, alt_frame="msl", camera_cfg=CC))
        site = ctx.inputs.sites[up.takeoff_site]
        # seq 0 = 這架的起飛坪（飛控解鎖時 home 會被重設到這裡）
        assert (rel[0]["lat"], rel[0]["lon"]) == (site.lat_deg, site.lon_deg)
        assert rel[0]["alt"] == pytest.approx(pad, abs=1e-9) and rel[0]["frame"] == 0
        assert msl[0] == rel[0]
        # 起飛：轉換起始高度就是相對起飛坪的 H_ts（修正前 55.80 / 64.20）
        to = next(it for it in rel if it["command"] == 84)
        assert to["alt"] == pytest.approx(ctx.specs[up.cls].transition_start_agl_m, abs=1e-6), uid
        # 逐項：相對高 + 起飛坪 MSL ＝ MSL 框架匯出（MSL 匯出不受影響）
        assert len(rel) == len(msl)
        for a, b in zip(rel[1:], msl[1:]):
            assert (a["command"], a["lat"], a["lon"]) == (b["command"], b["lat"], b["lon"])
            if a["command"] in NAVS and a["frame"] == 3:
                assert a["alt"] + pad == pytest.approx(b["alt"], abs=1e-6), (uid, a["seq"])
        # 降落：降回自己的起飛坪 ⇒ 相對高 0
        land = next(it for it in rel if it["command"] == 85)
        assert up.landing_site == up.takeoff_site
        assert land["alt"] == pytest.approx(0.0, abs=1e-6)


def test_sloped_pads_upload_tuples_use_the_own_pad_but_paths_keep_a_common_reference(hv_slope):
    """非建圖路徑：HvBuiltPath.alt 保留**全機共用**的規劃 home 基準（避撞與顯示跨機比高度），
    to_upload_tuples 送出前減掉 fc_alt_offset_m ⇒ 與 hv 匯出器（相對各自起飛坪）同點同值。"""
    pads = _pads(hv_slope)
    paths = hv_slope.last_result["assembled_paths"]
    ctx = hv_slope.last_ctx
    home_site = ctx.inputs.home.site_id
    for did, (uid, up, pad, home) in pads.items():
        bp = paths[did]
        assert bp.alt_ref_site == home_site and bp.alt_ref_msl_m == pytest.approx(home, abs=1e-9)
        assert bp.fc_home_site == up.takeoff_site and bp.fc_home_msl_m == pytest.approx(pad, abs=1e-9)
        assert bp.fc_alt_offset_m == pytest.approx(pad - home, abs=1e-9)
        rel = {(round(it["lat"], 9), round(it["lon"], 9)): it["alt"]
               for it in hv_slope.mission_items(did, camera_cfg=CC) if it["command"] == 16 and it["seq"] > 0}
        msl = {(round(it.x, 9), round(it.y, 9)): it.z
               for it in ap.plan_to_mission_items(up, ctx, hv_slope.cfg, alt_frame="msl", camera_cfg=CC)
               if it.command == 16 and it.seq > 0}
        tl = hv_slope.upload_tuples()[did]
        up_alt = {(round(t[0], 9), round(t[1], 9)): t[2] for t in tl if t[3] == 16}
        matched = 0
        for w in bp.waypoints:
            key = (round(w.lat, 9), round(w.lon, 9))
            if key in rel and w.alt > 0.0:
                # 共用基準：alt + 規劃 home MSL ＝ 該點的 MSL（兩架都一樣，避撞的 dz 才是真的高度差）
                assert w.alt + home == pytest.approx(msl[key], abs=1e-6), (uid, key)
                # 送飛控：相對自己的起飛坪 ＝ hv 匯出器的值
                assert w.alt - bp.fc_alt_offset_m == pytest.approx(rel[key], abs=1e-6), (uid, key)
                if key in up_alt:
                    assert up_alt[key] == pytest.approx(rel[key], abs=1e-6), (uid, key)
                matched += 1
        assert matched >= 10, (uid, matched)
        land = [t for t in tl if t[3] == 85]
        assert land and land[-1][2] == pytest.approx(bp.land_alt_m) and bp.land_alt_m == pytest.approx(0.0, abs=1e-6)
    # 至少一對 UAV 的坪高不同（前提成立），而兩條路徑的 alt 用同一個基準
    offs = [paths[d].fc_alt_offset_m for d in pads]
    assert max(offs) - min(offs) > 5.0
    assert len({paths[d].alt_ref_msl_m for d in pads}) == 1


def test_sloped_pads_legacy_vtol_exporter_subtracts_the_pad_offset(hv_slope, tmp_path):
    """legacy VTOLMissionExporter（非建圖匯出鈕）同樣要送相對起飛坪的高度。"""
    from mission.vtol_mission_exporter import VTOLMissionExporter
    pads = _pads(hv_slope)
    paths = hv_slope.last_result["assembled_paths"]
    did = max(pads, key=lambda d: abs(paths[d].fc_alt_offset_m))
    bp = paths[did]
    assert abs(bp.fc_alt_offset_m) > 3.0
    ex = VTOLMissionExporter(transition_alt_m=60.0, cruise_speed_ms=19.0, turn_radius_m=100.0)
    with_off = ex.generate_waypoint_lines(bp.waypoints, altitude=120.0, uav_id=did,
                                          alt_offset_m=bp.fc_alt_offset_m)
    no_off = ex.generate_waypoint_lines(bp.waypoints, altitude=120.0, uav_id=did)
    nav = lambda lines: [ln.split("\t") for ln in lines[1:] if ln.split("\t")[3] == "16"]
    a, b = nav(with_off), nav(no_off)
    assert len(a) == len(b) and len(a) >= 10
    for ra, rb in zip(a, b):
        assert float(ra[10]) == pytest.approx(float(rb[10]) - bp.fc_alt_offset_m, abs=0.011)
    files = ex.export_all({did: bp}, str(tmp_path), altitude=120.0, speed=19.0)
    text = (tmp_path / files[0]).read_text(encoding="utf-8").splitlines()
    assert [ln.split("\t")[10] for ln in text[1:] if ln.split("\t")[3] == "16"] == [r[10] for r in a]


def test_landing_at_another_site_is_relative_to_the_own_pad_in_both_exporters(hv_slope):
    """降落站 ≠ 起飛坪（例：能量不夠退回規劃 home）：兩條匯出路徑的降落高度都 = 降落站地面 − 本機起飛坪（不是 0、不是
    相對規劃 home）。"""
    ctx = hv_slope.last_ctx
    did, (uid, up, pad, home) = next(iter(_pads(hv_slope).items()))
    assert abs(pad - home) > 3.0
    other = dataclasses.replace(up, landing_site=ctx.inputs.home.site_id)
    items = ap.plan_to_mission_items(other, ctx, hv_slope.cfg, camera_cfg=CC)
    land = next(it for it in items if it.command == 85)
    assert land.z == pytest.approx(home - pad, abs=1e-6)
    bp = ad._built_path(other, ctx, hv_slope.cfg, did, ad.default_types, reduce=True, wp_radius_m=None,
                        home_msl=home, alt_ref_site=ctx.inputs.home.site_id,
                        fc_home_site=up.takeoff_site, fc_home_msl=pad)
    assert bp.land_alt_m == pytest.approx(home - pad, abs=1e-6)
    tl = ad.to_upload_tuples(bp)
    assert [t for t in tl if t[3] == 85][-1][2] == pytest.approx(home - pad, abs=1e-6)


def test_to_upload_tuples_land_alt_comes_from_the_path():
    import types as _t
    wp = lambda seg: _t.SimpleNamespace(lat=23.9, lon=120.9, alt=120.0, segment_type=seg, heading_compass_deg=0.0)
    bp = _t.SimpleNamespace(waypoints=[wp("TAKEOFF"), wp("OPERATION"), wp("LANDING")], land_alt_m=7.5)
    tl = ad.to_upload_tuples(bp)
    assert [t[2] for t in tl if t[3] == 85] == [7.5]
    bp0 = _t.SimpleNamespace(waypoints=bp.waypoints)                       # 原生 BuiltPath 沒有這欄 ⇒ 0（舊行為）
    assert [t[2] for t in ad.to_upload_tuples(bp0) if t[3] == 85] == [0.0]


def test_sloped_pads_export_all_rally_and_fence_alt_use_the_own_pad(hv_slope, tmp_path):
    pads = _pads(hv_slope)
    ctx = hv_slope.last_ctx
    s = ap.export_all(hv_slope.last_plan, ctx, hv_slope.cfg, tmp_path, camera_cfg=CC)
    h_tc = max(sp.transition_cmd(transition_profile=ctx.cfg.transition_profile).h_tc_cmd_m for sp in ctx.specs.values())
    for did, (uid, up, pad, home) in pads.items():
        u = s["uavs"][uid]
        assert u["fc_home"]["site_id"] == up.takeoff_site
        assert u["fc_home"]["alt_msl_m"] == pytest.approx(pad, abs=1e-9)
        assert u["fc_home"]["offset_from_planning_home_m"] == pytest.approx(pad - home, abs=1e-9)
        items = ap.read_qgc_wpl(u["files"]["waypoints"])
        up_items = hv_slope.mission_items(did, camera_cfg=CC)
        assert [it.command for it in items] == [it["command"] for it in up_items]          # 匯出 ＝ 上傳
        for a, b in zip(items, up_items):
            assert a.z == pytest.approx(b["alt"], abs=2e-6) and a.param2 == pytest.approx(b["param2"], abs=1e-7)
        assert items[0].z == pytest.approx(pad, abs=1e-6)
        rally = ap.read_qgc_wpl(u["files"]["rally"], mission_type=2)
        for r, site in zip(rally, (ctx.inputs.home, *ctx.inputs.alternates)):
            assert r.z == pytest.approx(ap._site_msl(ctx, site.site_id) + h_tc - pad, abs=1e-5)
        rel_max = max(w.alt_msl_m for w in up.waypoints) - pad + hv_slope.cfg.b_v_m
        assert u["params"]["FENCE_ALT_MAX"] >= math.ceil(rel_max)
        assert u["params"]["FENCE_ALT_MAX"] == float(math.ceil(max(rel_max, ctx.h_ceil_m.get(up.cls) or 0.0)))
    # 空域上限不壓過時：FENCE_ALT_MAX（AC_Fence 比的是「相對 home」）＝ 航線最高 MSL − **本機起飛坪** + b_v
    ctx_nc = dataclasses.replace(ctx, h_ceil_m={})
    s2 = ap.export_all(hv_slope.last_plan, ctx_nc, hv_slope.cfg, tmp_path / "nc", camera_cfg=CC)
    for did, (uid, up, pad, home) in pads.items():
        rel_max = max(w.alt_msl_m for w in up.waypoints) - pad + hv_slope.cfg.b_v_m
        assert s2["uavs"][uid]["params"]["FENCE_ALT_MAX"] == float(math.ceil(rel_max)), uid
        assert math.ceil(rel_max) != math.ceil(rel_max + pad - home), uid      # 前提：用規劃 home 會不同


class _RecLink:
    """FleetLink.upload_plan 用的假連線：只記錄送了什麼（不開任何埠）。"""

    def __init__(self):
        self.uav_id = None
        self.sent = {}

    def upload_mission(self, items, mission_type=0, timeout_s=None):
        self.sent["mission"] = items

    def upload_rally(self, items, timeout_s=None, **kw):
        self.sent["rally"] = items

    def upload_fence(self, items, timeout_s=None, **kw):
        self.sent["fence"] = items


def test_fleetlink_uploads_rally_and_mission_relative_to_each_pad(hv_slope):
    """hv_dccpp.mavlink_link.FleetLink.upload_plan：任務與 rally 都以各機起飛坪為 home（斜坡上兩機差 8.4 m）。"""
    from hv_dccpp import mavlink_link as ml
    ctx = hv_slope.last_ctx
    pads = _pads(hv_slope)
    links = {uid: _RecLink() for (uid, _up, _p, _h) in pads.values()}
    fl = ml.FleetLink(links)
    fl.upload_plan(hv_slope.last_plan, ctx, hv_slope.cfg, write_params=False, camera_cfg=CC)
    h_tc = max(sp.transition_cmd(transition_profile=ctx.cfg.transition_profile).h_tc_cmd_m for sp in ctx.specs.values())
    for did, (uid, up, pad, _home) in pads.items():
        sent = links[uid].sent
        assert ml._home_msl(up, ctx)[2] == pytest.approx(pad, abs=1e-9)
        assert sent["mission"][0].z == pytest.approx(pad, abs=1e-9)                       # seq 0 = 起飛坪
        r0 = sent["rally"][0]                                                              # 第 1 個 rally = 規劃 home
        assert r0.z == pytest.approx(ap._home_msl(ctx) + h_tc - pad, abs=1e-5), uid
    zs = [links[u].sent["rally"][0].z for (u, _a, _b, _c) in pads.values()]
    assert max(zs) - min(zs) > 8.0


@pytest.mark.parametrize("which", ["hv_flat", "hv_map"])
def test_flat_terrain_altitudes_unchanged(which, request):
    """平地（100 m DEM／無 DEM）：起飛坪 MSL ＝ 規劃 home MSL（ENU↔MSL 1e-9 m 級雜訊視為同高）⇒ 每個相對高度、
    起飛／降落高度與舊公式（MSL − 規劃 home MSL）逐位元相同。"""
    hv = request.getfixturevalue(which)
    pads = _pads(hv)
    ctx = hv.last_ctx
    for did, (uid, up, _pad, home) in pads.items():
        ref = ap.fc_home(ctx, up)
        assert ref[0] == up.takeoff_site and ref[3] == home
        rel = hv.mission_items(did, camera_cfg=CC)
        msl = mission_item_dicts(ap.plan_to_mission_items(up, ctx, hv.cfg, alt_frame="msl", camera_cfg=CC))
        n = 0
        for a, b in zip(rel[1:], msl[1:]):
            if a["frame"] == 3 and a["command"] in NAVS and a["command"] not in (84, 85):
                assert a["alt"] == b["alt"] - home, (uid, a["seq"])
                n += 1
        assert n > 20
        to = next(it for it in rel if it["command"] == 84)
        assert to["alt"] == next(it for it in msl if it["command"] == 84)["alt"] - home
        assert next(it for it in rel if it["command"] == 85)["alt"] == 0.0


def test_flat_dem_plan_gives_one_trigger_distance_per_height(hv_flat):
    """有 DEM 的平地計畫：帶起點 AGL 因地球曲率／內插高出 4–42 mm ⇒ 修正前每條帶 d 都不同；修正後 ≤ 0.5 m 的都用作業高度。"""
    for did in T._flying(hv_flat):
        uid = hv_flat.id_map[did]
        up = hv_flat.last_plan.plans[uid]
        items = hv_flat.mission_items(did, camera_cfg=CC)
        on = [it["param1"] for it in items if it["command"] == 206 and it["param1"] > 0 and it["param3"] == 1.0]
        starts = []
        for sid, _d in up.strips:
            w = next(w for w in up.waypoints if w.phase is Phase.OPERATE and w.strip_id == sid)
            starts.append(w.agl_m)
        near = [np.float32(p) for p, h in zip(on, starts) if abs(h - 120.0) <= ap.AGL_SNAP_TOL_M]
        assert len(near) >= 3 and len(set(near)) == 1, (uid, sorted(set(near)))


# ═══════════════════════════ (c) 整數型參數 ═══════════════════════════
def test_hero_airspeed_max_is_floored_to_what_the_vehicle_stores():
    spec = PLATFORM_HERO
    w = []
    p = recommended_params(spec, None, PlannerConfig(), warnings=w)
    assert spec.max_airspeed_mps is None or float(spec.max_airspeed_mps).is_integer()
    raw = round(1.4 * spec.cruise_airspeed_mps, 1)
    assert raw == 26.6
    assert p["AIRSPEED_MAX"] == 26.0
    assert any("AIRSPEED_MAX" in s and "26.6" in s and "26" in s for s in w), w


def test_round_int_params_rules_and_effective_value_warnings():
    w = []
    out = ap._round_int_params({"AIRSPEED_MIN": 16.2, "AIRSPEED_MAX": 26.6, "WP_LOITER_RAD": 145.3,
                                "RTL_RADIUS": 145.01, "WP_RADIUS": 27.5, "Q_RTL_ALT": 100.5,
                                "AIRSPEED_CRUISE": 19.3, "Q_ENABLE": 1.0}, w)
    assert out == {"AIRSPEED_MIN": 17.0, "AIRSPEED_MAX": 26.0, "WP_LOITER_RAD": 146.0, "RTL_RADIUS": 146.0,
                   "WP_RADIUS": 27.0, "Q_RTL_ALT": 101.0, "AIRSPEED_CRUISE": 19.3, "Q_ENABLE": 1.0}
    assert len(w) == 6 and all("AP_Int16" in s for s in w)
    assert ap._round_int_params({"WP_LOITER_RAD": -145.3}, [])["WP_LOITER_RAD"] == -146.0   # 方向號保留、半徑往大
    with pytest.raises(ValueError, match="Q_ENABLE"):
        ap._round_int_params({"Q_ENABLE": 1.5}, [])
    with pytest.raises(ValueError, match="CAM_MAX_ROLL"):
        ap._round_int_params({"CAM_MAX_ROLL": 12.5}, [])
    with pytest.raises(ValueError, match="UNKNOWN_PARAM"):
        ap._round_int_params({"UNKNOWN_PARAM": 1.0}, [])


def test_wind_inflated_radii_are_ceiled_never_below_r_eff():
    spec = PLATFORM_HERO
    r_eff = spec.r_eff(7.3)
    assert not float(round(r_eff, 1)).is_integer()                      # 前提：真的有小數
    w = []
    p = recommended_params(spec, None, PlannerConfig(), wind_speed_mps=7.3, warnings=w)
    for k in ("WP_LOITER_RAD", "RTL_RADIUS"):
        assert float(p[k]).is_integer() and p[k] >= r_eff and p[k] - r_eff < 1.0 + 0.05, (k, p[k], r_eff)


@pytest.mark.parametrize("kw", [
    dict(),
    dict(sensor_kinds=("rgb",)),
    dict(sensor_kinds=("rgb",), camera_cfg=CameraTriggerConfig(backend="relay", firmware="4.5.7", relay_pin=50,
                                                               aux_servo_channel_for_relay=9, feedback_pin=51,
                                                               feedback_servo_channel=11, max_roll_deg=15.0)),
    dict(sensor_kinds=("rgb",), camera_cfg=CameraTriggerConfig(backend="relay", firmware="4.4.4", relay_pin=50)),
    dict(sensor_kinds=("lidar",), wind_speed_mps=7.3, alt_frame="terrain", min_nav_spacing_m=33.3),
])
def test_every_emitted_key_is_classified_and_integer_typed_ones_are_integral(kw):
    for spec in (PLATFORM_HERO, PLATFORM_PIONEER):
        p = recommended_params(spec, None, PlannerConfig(), warnings=[], **kw)
        for k, v in p.items():
            t = ap.param_storage_type(k)
            assert t is not None, f"{k} 沒有分類（AP_Int*／AP_Float）"
            if t != "AP_Float":
                assert float(v).is_integer(), (k, v, t)


def test_camera_trigger_config_refuses_fractional_max_roll():
    with pytest.raises(ValueError, match="max_roll_deg"):
        CameraTriggerConfig(max_roll_deg=12.5)
    assert CameraTriggerConfig(max_roll_deg=15.0).max_roll_deg == 15.0
    assert CameraTriggerConfig(max_roll_deg=15).max_roll_deg == 15


def test_commanded_speed_outside_the_stored_range_is_reported(hv_map):
    """取整後若任務的空速指令落在 [AIRSPEED_MIN, AIRSPEED_MAX] 外，飛控會靜默不套用 DO_CHANGE_SPEED ⇒ 要大聲說。
    HERO：AIRSPEED_MAX 26.6 → 26；一個 26.3 m/s 的巡航段在取整前合法、取整後被飛控丟掉。"""
    did = next(d for d in T._flying(hv_map) if hv_map.last_plan.plans[hv_map.id_map[d]].cls == "B")
    up = hv_map.last_plan.plans[hv_map.id_map[did]]
    spec = hv_map.last_ctx.specs["B"]
    k = next(i for i, w in enumerate(up.waypoints) if w.mode is Mode.CRUISE and w.airspeed_mps > 0)
    wps = list(up.waypoints)
    wps[k] = dataclasses.replace(wps[k], airspeed_mps=26.3)
    fast = dataclasses.replace(up, waypoints=wps)
    w = []
    p = recommended_params(spec, None, PlannerConfig(), fast, warnings=w)
    assert p["AIRSPEED_MAX"] == 26.0
    assert any("26.3" in s and "AIRSPEED_MAX" in s and "DO_CHANGE_SPEED" in s for s in w), w
    w2 = []
    recommended_params(spec, None, PlannerConfig(), up, warnings=w2)
    assert not any("DO_CHANGE_SPEED" in s for s in w2), w2


# ═══════════════════════════ (d) 預估張數（50 Hz tick＋接受半徑） ═══════════════════════════
def _syn_items(L, d=18.848, v=19.0, lead_p2=1.0, end_p2=30.0, p3=1.0, speed_item=True):
    """home → NAV(0) → [178 v] → 帶起點 NAV(100) → 引入 NAV(150, lead_p2) → 206(d) → 帶終點 NAV(150+L) → 206(0) → NAV。"""
    def nav(seq, n_m, p2=30.0):
        lat, lon = T.HOME[0] + n_m / T._m_per_deg_lat(T.HOME[0]), T.HOME[1]
        return {"seq": seq, "command": 16, "frame": 3, "lat": lat, "lon": lon, "alt": 120.0, "param1": 0.0,
                "param2": float(p2), "param3": 0.0, "param4": 0.0, "current": 0, "autocontinue": 1}
    home = dict(nav(0, -100.0), frame=0, alt=0.0)
    seq = [home, nav(1, 0.0)]
    if speed_item:
        seq.append({**T._cam(2, 0.0), "command": 178, "param1": 0.0, "param2": float(v), "param3": -1.0})
    seq += [nav(3, 100.0), nav(4, 150.0, lead_p2), T._cam(5, d, p3), nav(6, 150.0 + L, end_p2),
            T._cam(7, 0.0, p3=0.0), nav(8, 150.0 + L + 300.0)]                 # 關 = 匯出器的 206(0, 0, 0, 0)
    for i, it in enumerate(seq):
        it["seq"] = i
    return seq


def test_expected_photo_count_tick_and_acceptance_model_exact_values():
    # L_eff = (150+L−30) − (150−1) = L − 29；間距 q = ceil(d/(v·0.02))·v·0.02
    q19 = math.ceil(18.848 / (19.0 * 0.02)) * 19.0 * 0.02                 # 50 × 0.38 = 19.0
    assert q19 == pytest.approx(19.0)
    for L in (1179.0, 1184.0, 1000.0):
        assert expected_photo_count(_syn_items(L)) == 1 + math.floor((L - 29.0) / q19), L
    # 帶長 1184 m（有效 1155 m）：連續模型 1 + floor(1155/18.848) = 62；tick 量化後 1 + floor(1155/19.0) = 61
    assert expected_photo_count(_syn_items(1184.0)) == 61
    # 速度覆寫：21.5 m/s ⇒ q = 44 × 0.43 = 18.92 ⇒ 1 + floor(1155/18.92) = 62
    assert expected_photo_count(_syn_items(1184.0), speed_mps=21.5) == 62
    # CAM1_INTRVAL_MIN 比距離觸發慢 ⇒ 間距 = ceil(t_min/Δt)·v·Δt（60 × 0.38 = 22.8 m）⇒ 1 + floor(1155/22.8) = 51
    assert expected_photo_count(_syn_items(1184.0), intrval_min_s=1.2) == 51
    # 沒有空速資訊也沒給 speed_mps ⇒ 不量化（連續模型），接受半徑照算 ⇒ 62
    assert expected_photo_count(_syn_items(1184.0, speed_item=False)) == 62
    # 接受半徑：引入 NAV 舊值 30 ⇒ 有效長 = L（兩端各提前 30）⇒ 1 + floor(1184/19.0) = 63
    assert expected_photo_count(_syn_items(1184.0, lead_p2=30.0)) == 63
    # ArduPlane：任何 p3 = 1 的 206 都立即拍一張，包括 p1 = 0 的關（AP_Mission_Commands.cpp 4.5.7:138-142）
    off_p3 = _syn_items(1184.0)
    off_p3[7] = dict(off_p3[7], param3=1.0)
    assert expected_photo_count(off_p3) == 61 + 1
    for bad in (0.0, -1.0, float("nan")):
        with pytest.raises(ValueError):
            expected_photo_count(_syn_items(1184.0), speed_mps=bad)


def test_expected_photo_count_tracks_the_exposure_verifier_without_bias():
    """多種帶長：與 exposure_verifier 的 50 Hz 模擬逐一比，差 ≤ 1 且平均偏差 ≈ 0（舊版偏多 ~0.5 張／帶）。"""
    from core.recon.exposure_verifier import simulate_mission_photos
    diffs_new, diffs_old = [], []
    for L in np.arange(1000.0, 1400.0, 13.0):
        items = _syn_items(float(L))
        sim = len(simulate_mission_photos(items, uav="B", airspeed_mps=19.0, wind=(0.0, 0.0),
                                          intrval_min_s=0.5).photos)
        diffs_new.append(expected_photo_count(items) - sim)
        diffs_old.append((math.floor((float(L) - 29.0) / 18.848 + 1e-9) + 1) - sim)   # 舊公式（同樣的接受半徑）
    assert max(abs(x) for x in diffs_new) <= 1, diffs_new
    assert abs(float(np.mean(diffs_new))) < 0.15, diffs_new
    assert float(np.mean(diffs_old)) > 0.3, diffs_old                    # 前提：舊公式真的系統性偏多


def test_expected_photo_count_equals_the_verifier_on_the_real_plan(hv_map):
    total = 0
    for did in T._flying(hv_map):
        uid = hv_map.id_map[did]
        items = hv_map.mission_items(did, camera_cfg=CC)
        n_sim = len(_simulate(items, uid).photos)
        n = expected_photo_count(items, intrval_min_s=CC.interval_min_s)
        assert abs(n - n_sim) <= 1, (uid, n, n_sim)
        assert expected_photo_count(items) == n                              # 0.5 s 不限速這份計畫
        total += n_sim
    assert total > 100


# ═══════════════════════════ (e) 206 p1 不因 AGL 抖動而不同 ═══════════════════════════
def test_trigger_distance_ignores_micro_agl_jitter_but_follows_real_height_changes(hv_map):
    did = next(d for d in T._flying(hv_map) if len(hv_map.last_plan.plans[hv_map.id_map[d]].strips) >= 3)
    uid = hv_map.id_map[did]
    up = hv_map.last_plan.plans[uid]
    ctx = hv_map.last_ctx
    sids = [s for s, _ in up.strips]

    def bump(dz_by_strip):
        wps, seen = [], set()
        for w in up.waypoints:
            if w.phase is Phase.OPERATE and w.strip_id in dz_by_strip and w.strip_id not in seen:
                seen.add(w.strip_id)
                w = dataclasses.replace(w, agl_m=w.agl_m + dz_by_strip[w.strip_id])
            wps.append(w)
        return dataclasses.replace(up, waypoints=wps)

    def p1s(plan_, cc):
        return [it.param1 for it in ap.plan_to_mission_items(plan_, ctx, hv_map.cfg, camera_cfg=cc)
                if it.command == 206 and it.param1 > 0]

    sensor = ap._sensor_for(up, ctx, sids[0], None)
    d_op = ap.trigger_distance_m(sensor, 120.0)
    # R2 的 5 ulp（−6e-5 m）與有 DEM 時的地球曲率／內插（F1 實測帶起點 AGL 高出 4–42 mm）
    jit = bump({sids[0]: -6e-5, sids[1]: +0.042, sids[2]: -0.4})
    got = p1s(jit, CC)
    assert len(got) == len(sids)
    assert {np.float32(x) for x in got} == {np.float32(d_op)}             # ≤ 0.5 m ⇒ 用作業高度，逐位元相同
    assert ap.AGL_SNAP_TOL_M == 0.5
    # 真的高度差（2 m，超過容差）照舊依該點 AGL
    agl0 = next(w.agl_m for w in up.waypoints if w.phase is Phase.OPERATE and w.strip_id == sids[0])
    real = p1s(bump({sids[0]: 2.0}), CC)
    assert real[0] == pytest.approx(ap.trigger_distance_m(sensor, agl0 + 2.0), rel=1e-12)
    assert real[0] != pytest.approx(d_op, rel=1e-3)
    # 預設路徑（不給 camera_cfg）逐值不變：仍用該點 AGL
    plain = p1s(jit, None)
    assert plain[0] != d_op


# ═══════════════════════════ (f) 錯開起飛 ═══════════════════════════
def test_takeoff_schedule_equals_planned_offsets(hv_map):
    sched = hv_map.takeoff_schedule()
    plans = hv_map.last_plan.plans
    t0 = min(p.t_takeoff_s for p in plans.values())
    want = {d: plans[u].t_takeoff_s - t0 for d, u in hv_map.id_map.items() if u in plans}
    assert sched == want
    assert min(sched.values()) == 0.0
    assert max(sched.values()) > 0.0                                      # 這份計畫真的有錯開（R1：A1 140 s）
    # 預設任務不帶 NAV_DELAY（武裝的飛機在地上等幾十分鐘不安全）
    for did in sched:
        assert all(it["command"] != 93 for it in hv_map.mission_items(did, camera_cfg=CC))
    with pytest.raises(AeroPlanAdapterError, match="尚未規劃"):
        T._coord(None).takeoff_schedule()
    # 全隊整體延後 100.25 s（最小值不是 0）⇒ 錯開不變
    saved = hv_map.last_plan
    try:
        hv_map.last_plan = dataclasses.replace(saved, plans={u: dataclasses.replace(p, t_takeoff_s=p.t_takeoff_s + 100.25)
                                                             for u, p in saved.plans.items()})
        shifted = hv_map.takeoff_schedule()
        assert shifted.keys() == sched.keys()
        for d in sched:
            assert shifted[d] == pytest.approx(sched[d], abs=1e-9)
        assert min(shifted.values()) == 0.0
    finally:
        hv_map.last_plan = saved


def test_export_summary_carries_the_schedule_and_nav_delay_option_still_works(hv_map, tmp_path):
    sched = hv_map.takeoff_schedule()
    by_uid = {hv_map.id_map[d]: v for d, v in sched.items()}
    s = ap.export_all(hv_map.last_plan, hv_map.last_ctx, hv_map.cfg, tmp_path / "op", camera_cfg=CC)
    assert s["takeoff"]["mode"] == "operator"
    assert s["takeoff"]["offset_s"] == by_uid
    for uid, v in by_uid.items():
        assert s["uavs"][uid]["takeoff_offset_s"] == v
    on_disk = json.loads((tmp_path / "op" / "mission_summary.json").read_text(encoding="utf-8"))
    assert on_disk["takeoff"]["offset_s"] == by_uid
    # 時間原點不在整數秒：摘要給未取整的 t_takeoff − t0（delay_s 才是 0.1 s 取整的顯示值）
    t0 = min(p.t_takeoff_s for p in hv_map.last_plan.plans.values()) - 0.37
    s1 = ap.export_all(hv_map.last_plan, hv_map.last_ctx, hv_map.cfg, tmp_path / "t0", camera_cfg=CC, t0_s=t0)
    for uid, v in by_uid.items():
        assert s1["takeoff"]["offset_s"][uid] == pytest.approx(v + 0.37, abs=1e-9)
        assert s1["uavs"][uid]["takeoff_offset_s"] == pytest.approx(v + 0.37, abs=1e-9)
        assert s1["takeoff"]["delay_s"][uid] == round(v + 0.37, 1)
    s2 = ap.export_all(hv_map.last_plan, hv_map.last_ctx, hv_map.cfg, tmp_path / "nd", camera_cfg=CC,
                       takeoff_delay="nav_delay")
    for uid, v in by_uid.items():
        items = ap.read_qgc_wpl(s2["uavs"][uid]["files"]["waypoints"])
        delays = [it for it in items if it.command == 93]
        if v > 0:
            assert len(delays) == 1 and delays[0].param1 == pytest.approx(v, abs=1e-6) and delays[0].seq == 1
        else:
            assert not delays


def test_takeoff_schedule_on_the_gui_coordinator_mapping_plan():
    """R1 的 GUI 路徑情境（test_hdmap_gui_coordinator 的建圖計畫；R1 量到 A1 1516 s、B2 0、B3 122）。"""
    import test_hdmap_gui_coordinator as G
    coord = G.plan(G.mapping_params())
    hv = coord.hv_coordinator
    plans = hv.last_plan.plans
    t0 = min(p.t_takeoff_s for p in plans.values())
    sched = hv.takeoff_schedule()
    assert sched == {d: plans[u].t_takeoff_s - t0 for d, u in hv.id_map.items() if u in plans}
    assert sorted(sched.values())[0] == 0.0 and max(sched.values()) > 0.0
