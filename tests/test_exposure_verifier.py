"""core/recon/exposure_verifier.py —— 曝光層級覆蓋驗證（飛前模擬 ArduPilot 距離觸發、飛後讀 photos.csv）。

全部斷言數值：間距、重疊率、偏流角、張數、欠覆蓋多邊形的位置與面積。任務用 10-tuple
``(seq, command, frame, p1, p2, p3, p4, lat, lon, alt)``，與 hv_dccpp.ardupilot.plan_to_mission_items 的欄位一致。
"""
from __future__ import annotations

import csv
import dataclasses
import math
import time
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from core.recon.exposure_verifier import (
    F2_COLUMNS,
    FrameCamera,
    LocalFrame,
    PhotoPose,
    _footprint_wl,
    exposure_metrics,
    footprint_corners_enu,
    read_photos_csv,
    simulate_mission_photos,
    verify_postflight,
    verify_preflight,
    wind_triangle,
)

LAT0, LON0 = 25.0, 121.0
FR = LocalFrame(LAT0, LON0)
HQ6 = FrameCamera(6.0, 1.55, 4056, 3040)          # 120 m：125.736 × 94.24 m
W120, L120 = HQ6.footprint_m(120.0)
D80 = L120 * (1.0 - 0.8)                            # 前向 0.80 的觸發距離 18.848 m


def ll(x: float, y: float) -> tuple[float, float]:
    la, lo = FR.to_latlon(x, y)
    return float(la), float(lo)


def xy(p: PhotoPose) -> tuple[float, float]:
    x, y = FR.to_xy(p.lat_deg, p.lon_deg)
    return float(x), float(y)


def mission(strips, d=D80, alt=120.0, p3=1.0, r_acc=0.0, home=(0.0, 0.0), frame=3, pre=(), tail=()):
    """home → 每條帶 NAV(起) 206(d,0,p3) NAV(訖) 206(0)。strips = [(x0, y0, x1, y1), ...]（本地 ENU）。"""
    it = [(0, 16, 0, 0, 0, 0, 0, *ll(*home), 0.0)]
    for c in pre:
        it.append((len(it), *c))
    for (x0, y0, x1, y1) in strips:
        it.append((len(it), 16, frame, 0, r_acc, 0, 0, *ll(x0, y0), alt))
        it.append((len(it), 206, 3, d, 0, p3, 0, 0, 0, 0))
        it.append((len(it), 16, frame, 0, r_acc, 0, 0, *ll(x1, y1), alt))
        it.append((len(it), 206, 3, 0, 0, 0, 0, 0, 0, 0))
    for c in tail:
        it.append((len(it), *c))
    return it


def rect(x0, y0, x1, y1):
    return [ll(x0, y0), ll(x1, y0), ll(x1, y1), ll(x0, y1)]


def strip_photos(res, strip="1"):
    return [p for p in res.photos if p.strip == strip]


# ---------------------------------------------------------------------------
# 風三角
# ---------------------------------------------------------------------------
def test_crosswind_5_at_19_gives_15_26_deg_heading_offset():
    s = wind_triangle(0.0, 19.0, 5.0, 270.0)              # 西風（從 270° 來）吹往東
    assert s.crab_deg == pytest.approx(-math.degrees(math.asin(5 / 19)), abs=1e-9)
    assert abs(s.crab_deg) == pytest.approx(15.26, abs=0.005)
    assert s.heading_deg == pytest.approx(344.74, abs=0.005)    # 機頭偏向上風（西）
    assert s.groundspeed_mps == pytest.approx(19 * math.cos(math.asin(5 / 19)), abs=1e-9)
    e = wind_triangle(0.0, 19.0, 5.0, 90.0)                # 東風 → 機頭偏東
    assert e.crab_deg == pytest.approx(15.26, abs=0.005)
    # 順風／逆風：不偏、地速加減
    assert wind_triangle(0.0, 19.0, 5.0, 180.0).groundspeed_mps == pytest.approx(24.0)
    assert wind_triangle(0.0, 19.0, 5.0, 0.0).groundspeed_mps == pytest.approx(14.0)
    # 向東飛、北風（吹往南＝航跡右側）⇒ 機頭偏北（左）
    assert wind_triangle(90.0, 19.0, 5.0, 0.0).crab_deg == pytest.approx(-math.degrees(math.asin(5 / 19)))
    assert wind_triangle(90.0, 19.0, 5.0, 0.0).heading_deg == pytest.approx(90 - 15.2575, abs=1e-3)
    with pytest.raises(ValueError):
        wind_triangle(0.0, 4.0, 5.0, 270.0)                # 側風 ≥ 空速
    with pytest.raises(ValueError):
        wind_triangle(0.0, 4.0, 5.0, 0.0)                  # 逆風 ≥ 空速


def test_preflight_photo_heading_includes_crab_and_cog_is_track():
    res = simulate_mission_photos(mission([(0, 0, 0, 1000)]), uav="H", airspeed_mps=19.0, wind=(5.0, 270.0))
    ph = strip_photos(res)
    assert len(ph) > 40
    for p in ph:
        assert p.cog_deg == pytest.approx(0.0, abs=1e-6)
        assert (p.yaw_deg - p.cog_deg + 180) % 360 - 180 == pytest.approx(-15.26, abs=0.005)
        assert p.groundspeed_mps == pytest.approx(18.330, abs=0.001)


# ---------------------------------------------------------------------------
# ArduPilot 觸發語意
# ---------------------------------------------------------------------------
def test_single_strip_forward_overlap_is_one_minus_d_over_l():
    res = simulate_mission_photos(mission([(0, 0, 0, 2000)]), uav="H", airspeed_mps=19.0, wind=(0.0, 0.0))
    ph = strip_photos(res)
    ys = np.array([xy(p)[1] for p in ph])
    gaps = np.diff(ys)
    # 50 Hz tick 檢查距離：間距 = 第一個 ≥ d 的 tick（每 tick 0.38 m）
    assert gaps.min() >= D80 - 1e-6
    assert gaps.max() < D80 + 19.0 / 50 + 1e-6
    assert gaps == pytest.approx(19.0, abs=1e-6)            # ceil(18.848/0.38)=50 tick
    rep = verify_preflight({"H": mission([(0, 0, 0, 2000)])}, rect(-60, 100, 60, 1900), HQ6,
                           airspeed_mps=19.0, wind=(0.0, 0.0))
    fwd = rep.forward_overlap
    assert fwd.n == len(ph) - 1
    assert fwd.median == pytest.approx(1 - D80 / L120, rel=0.01)
    assert fwd.median == pytest.approx(1 - 19.0 / L120, abs=1e-6)
    assert rep.photos_per_uav == {"H": len(ph)}
    # 單帶：沿帶每點被 4–5 張看到（l/間距 = 4.96）
    assert set(rep.views_histogram) <= {0, 4, 5}
    assert rep.frac_ge[5] == pytest.approx(rep.views_histogram.get(5, 0) / rep.total_cells)


def test_crab_reduces_forward_overlap_by_rotated_footprints():
    rep = verify_preflight({"H": mission([(0, 0, 0, 2000)])}, rect(-60, 100, 60, 1900), HQ6,
                           airspeed_mps=19.0, wind=(5.0, 270.0))
    ph = strip_photos(rep.simulations["H"])
    gap = float(np.median(np.diff([xy(p)[1] for p in ph])))
    c = math.radians(15.26)
    expect = (1 - gap * math.cos(c) / L120) * (1 - gap * math.sin(c) / W120)
    assert rep.forward_overlap.median == pytest.approx(expect, abs=2e-3)
    assert rep.forward_overlap.median < 1 - gap / L120 - 0.02


def test_p3_immediate_shot_counted_and_first_distance_shot_after_d():
    on = simulate_mission_photos(mission([(0, 0, 0, 1000)], p3=1.0), uav="H", airspeed_mps=19.0, wind=(0, 0))
    off = simulate_mission_photos(mission([(0, 0, 0, 1000)], p3=0.0), uav="H", airspeed_mps=19.0, wind=(0, 0))
    a, b = strip_photos(on), strip_photos(off)
    assert a[0].trigger == "immediate"
    assert all(p.trigger == "distance" for p in b)
    assert len(a) == len(b) + 1
    y_on = xy(a[0])[1]
    assert 0.0 <= y_on <= 1.9 + 1e-6                     # 10 Hz 抵達偵測：最多晚 0.1 s
    # 不立刻拍：上一張位置在 206 執行後第一個 tick 才記下 ⇒ 第一張在其後 ≥ d
    assert xy(b[0])[1] - y_on == pytest.approx(19.0, abs=0.38 + 1e-6)
    assert [round(xy(p)[1], 6) for p in a[1:]] == [round(xy(p)[1], 6) for p in b]


def test_206_zero_stops_triggering():
    # 帶 0→600 開、600 關，航線繼續飛到 1500
    it = mission([(0, 0, 0, 600)])
    it.append((len(it), 16, 3, 0, 0, 0, 0, *ll(0, 1500), 120.0))
    res = simulate_mission_photos(it, uav="H", airspeed_mps=19.0, wind=(0, 0))
    ys = [xy(p)[1] for p in res.photos]
    assert np.diff(ys) == pytest.approx(19.0, abs=1e-6)
    assert max(ys) <= 600 + 1.9 + 1e-6
    assert max(ys) > 600 - 19.0 - 1e-6
    # 沒有 206(0)：一路拍到航線結束，且寫 note
    it2 = [t for t in it if not (t[1] == 206 and t[3] == 0)]
    res2 = simulate_mission_photos(it2, uav="H", airspeed_mps=19.0, wind=(0, 0))
    assert max(xy(p)[1] for p in res2.photos) > 1500 - 19.0 - 1e-6
    assert any("206(0)" in n for n in res2.notes)


def test_nav_accept_radius_fires_206_off_early():
    """ArduPlane 的 NAV_WAYPOINT param2 是接受半徑：帶尾 206(0) 在離帶尾 30 m 處就執行。"""
    it = mission([(0, 0, 0, 1000)], r_acc=30.0, home=(0, -300))      # 從南邊 300 m 平飛進場
    r30 = simulate_mission_photos(it, uav="H", airspeed_mps=19.0, wind=(0, 0))
    y30 = [xy(p)[1] for p in r30.photos]
    assert max(y30) <= 1000 - 30 + 1.9 + 1e-6
    assert max(y30) > 1000 - 30 - 19.0
    assert -30 - 1e-6 <= min(y30) <= -30 + 1.9 + 1e-6     # 206 開也提早 30 m（在進場腿上）
    ovr = simulate_mission_photos(it, uav="H", airspeed_mps=19.0, wind=(0, 0), accept_radius=0.0)
    y0 = [xy(p)[1] for p in ovr.photos]
    assert max(y0) > 1000 - 19.0 - 1e-6 and 0 - 1e-6 <= min(y0) <= 1.9 + 1e-6
    assert ovr.effective["accept_radius"] == 0.0
    # param2 = 0 會寫 note（ArduPlane 改用 WP_RADIUS，本模擬以 0 計）
    p0 = simulate_mission_photos(mission([(0, 0, 0, 1000)], r_acc=0.0), uav="H", airspeed_mps=19.0, wind=(0, 0))
    assert p0.effective["nav_p2_zero_count"] == 2
    assert any("WP_RADIUS" in n for n in p0.notes)


def test_intrval_min_postpones_to_v_times_tmin():
    # 20 m/s、d = 10 m：Tmin 0.8 s ⇒ v·Tmin = 16 m > d ⇒ 間距 16 m
    it = mission([(0, 0, 0, 1000)], d=10.0)
    slow = simulate_mission_photos(it, uav="H", airspeed_mps=20.0, wind=(0, 0), intrval_min_s=0.8)
    g = np.diff([xy(p)[1] for p in strip_photos(slow)])
    assert g == pytest.approx(16.0, abs=1e-6)
    assert slow.effective["intrval_min_ms"] == 800
    assert sum(p.trigger == "pending" for p in slow.photos) == len(slow.photos) - 1
    # Tmin 0.3 s ⇒ v·Tmin = 6 m < d ⇒ 間距仍是 d（tick 量化 ≤ 0.4 m）
    fast = simulate_mission_photos(it, uav="H", airspeed_mps=20.0, wind=(0, 0), intrval_min_s=0.3)
    g2 = np.diff([xy(p)[1] for p in strip_photos(fast)])
    assert g2.min() >= 10.0 - 1e-6 and g2.max() <= 10.4 + 1e-6
    assert not any(p.trigger == "pending" for p in fast.photos)
    with pytest.raises(ValueError):
        simulate_mission_photos(it, uav="H", airspeed_mps=20.0, wind=(0, 0), intrval_min_s=11.0)


def test_trigger_duration_blocks_like_a_min_interval():
    """CAM1_DURATION 1.0 s（repo 舊值）：脈衝期間 trigger_pic 失敗 ⇒ 觸發率被壓到 1 Hz。"""
    it = mission([(0, 0, 0, 1000)], d=10.0)
    r = simulate_mission_photos(it, uav="H", airspeed_mps=20.0, wind=(0, 0), trigger_duration_s=1.0)
    g = np.diff([xy(p)[1] for p in strip_photos(r)])
    # 206 p3=1 那張在 mission DO 裡拍、同一 tick 的相機更新就先減 1 ⇒ 第一段 49 tick；之後都是 50 tick
    assert g[0] == pytest.approx(19.6, abs=1e-6)
    assert g[1:] == pytest.approx(20.0, abs=1e-6)            # 50 tick × 0.4 m
    assert r.effective["trigger_duration_ticks"] == 50
    r01 = simulate_mission_photos(it, uav="H", airspeed_mps=20.0, wind=(0, 0), trigger_duration_s=0.1)
    assert np.diff([xy(p)[1] for p in strip_photos(r01)]).max() <= 10.4 + 1e-6


def test_second_do_after_a_nav_runs_one_mission_update_later():
    """同一 NAV 之後的 DO 一次跑一個：DO_SET_SERVO 擋在 206 前面 ⇒ 206 晚 1 次 mission.update（0.1 s）。"""
    base = mission([(0, 0, 0, 800)], home=(0, -300))
    servo = list(base)
    k = next(i for i, t in enumerate(servo) if t[1] == 206)
    servo.insert(k, (k, 183, 3, 9, 1500, 0, 0, 0, 0, 0))   # DO_SET_SERVO（與相機無關）
    servo = [(i, *t[1:]) for i, t in enumerate(servo)]
    a = simulate_mission_photos(base, uav="H", airspeed_mps=19.0, wind=(0, 0))
    b = simulate_mission_photos(servo, uav="H", airspeed_mps=19.0, wind=(0, 0))
    assert a.photos[0].trigger == b.photos[0].trigger == "immediate"
    assert xy(b.photos[0])[1] - xy(a.photos[0])[1] == pytest.approx(0.1 * 19.0, abs=1e-6)
    assert b.photos[0].t_s - a.photos[0].t_s == pytest.approx(0.1, abs=1e-9)
    # 第 3 個 DO 晚 3 次（0.3 s）
    two = list(servo)
    two.insert(k, (k, 183, 3, 10, 1500, 0, 0, 0, 0, 0))
    two = [(i, *t[1:]) for i, t in enumerate(two)]
    c = simulate_mission_photos(two, uav="H", airspeed_mps=19.0, wind=(0, 0))
    assert c.photos[0].t_s - a.photos[0].t_s == pytest.approx(0.3, abs=1e-9)


def test_do_change_speed_applied_or_silently_ignored_out_of_range():
    pre = ((178, 3, 0, 25.0, -1, 0, 0, 0, 0),)              # DO_CHANGE_SPEED 空速 25（param1=0）
    it = mission([(0, 0, 0, 1000)], pre=pre)
    r = simulate_mission_photos(it, uav="H", airspeed_mps=19.0, wind=(0, 0))
    assert {round(p.groundspeed_mps, 6) for p in r.photos} == {25.0}
    assert r.effective["airspeed_changes"] == [(1, 25.0, "assumed")]
    lim = simulate_mission_photos(it, uav="H", airspeed_mps=19.0, wind=(0, 0), airspeed_limits_mps=(15.0, 22.0))
    assert {round(p.groundspeed_mps, 6) for p in lim.photos} == {19.0}
    assert lim.effective["airspeed_changes"] == [(1, 25.0, "ignored_out_of_range")]
    assert any("靜默不套用" in n for n in lim.notes)


def test_altitude_frames_msl_relative_terrain_agree():
    home_msl = 50.0
    rel = mission([(0, 0, 0, 800)], alt=120.0, frame=3)
    msl = mission([(0, 0, 0, 800)], alt=170.0, frame=0)
    ter = mission([(0, 0, 0, 800)], alt=120.0, frame=10)
    kw = dict(uav="H", airspeed_mps=19.0, wind=(0, 0), home_alt_msl=home_msl)
    hs = [{round(p.h_m, 6) for p in simulate_mission_photos(m, terrain=t, **kw).photos}
          for m, t in ((rel, None), (msl, None), (ter, 50.0))]
    assert hs == [{170.0}, {170.0}, {170.0}]
    r = simulate_mission_photos(rel, uav="H", airspeed_mps=19.0, wind=(0, 0))   # home 列 z=0 → 地面 0
    assert r.effective["home_alt_source"] == "mission_item0"
    assert {round(p.h_m, 6) for p in r.photos} == {120.0}
    with pytest.raises(ValueError, match="沒有值"):
        simulate_mission_photos(ter, terrain=lambda la, lo: None, **kw)


def test_unsupported_and_static_mission_checks():
    with pytest.raises(ValueError, match="DO_JUMP"):
        simulate_mission_photos(mission([(0, 0, 0, 500)], tail=((177, 3, 1, 2, 0, 0, 0, 0, 0),)),
                                uav="H", airspeed_mps=19.0, wind=(0, 0))
    with pytest.raises(ValueError, match="2000"):
        simulate_mission_photos(mission([(0, 0, 0, 500)], pre=((2000, 3, 0, 1, 0, 0, 0, 0, 0),)),
                                uav="H", airspeed_mps=19.0, wind=(0, 0))
    # DO_LAND_START 落在 206 開啟區間內（評估 §2 第 6 點）
    it = [(0, 16, 0, 0, 0, 0, 0, *ll(0, 0), 0.0),
          (1, 16, 3, 0, 0, 0, 0, *ll(0, 0), 120.0),
          (2, 206, 3, D80, 0, 1, 0, 0, 0, 0),
          (3, 189, 3, 0, 0, 0, 0, 0, 0, 0),
          (4, 16, 3, 0, 0, 0, 0, *ll(0, 500), 120.0),
          (5, 206, 3, 0, 0, 0, 0, 0, 0, 0)]
    r = simulate_mission_photos(it, uav="H", airspeed_mps=19.0, wind=(0, 0))
    assert any("DO_LAND_START" in n and "seq 3" in n for n in r.notes)
    with pytest.raises(ValueError):
        simulate_mission_photos(it, uav="H", airspeed_mps=19.0, wind=(1.0,))


def test_item_shapes_dict_and_attribute_objects_match_tuples():
    @dataclass
    class MI:                                                # 與 hv_dccpp.ardupilot.MissionItem 同欄位
        seq: int
        frame: int
        command: int
        current: int
        autocontinue: int
        param1: float
        param2: float
        param3: float
        param4: float
        x: float
        y: float
        z: float
    tup = mission([(0, 0, 0, 700)])
    objs = [MI(t[0], t[2], t[1], 0, 1, t[3], t[4], t[5], t[6], t[7], t[8], t[9]) for t in tup]
    dicts = [dict(seq=t[0], command=t[1], frame=t[2], param1=t[3], param2=t[4], param3=t[5], param4=t[6],
                  lat=t[7], lon=t[8], alt=t[9]) for t in tup]
    kw = dict(uav="H", airspeed_mps=19.0, wind=(0, 0))
    ref = [(p.lat_deg, p.lon_deg) for p in simulate_mission_photos(tup, **kw).photos]
    assert len(ref) > 30
    assert [(p.lat_deg, p.lon_deg) for p in simulate_mission_photos(objs, **kw).photos] == ref
    assert [(p.lat_deg, p.lon_deg) for p in simulate_mission_photos(dicts, **kw).photos] == ref
    with pytest.raises(TypeError):
        simulate_mission_photos([(0, 16, 0)], **kw)


# ---------------------------------------------------------------------------
# 足跡、側向、接縫、欠覆蓋
# ---------------------------------------------------------------------------
def test_tilted_footprint_pinhole_projection():
    tw, tl = W120 / 240.0, L120 / 240.0
    one = np.ones(1)
    c0, ok0 = footprint_corners_enu(np.zeros(1), np.zeros(1), 100 * one, tw * one, tl * one,
                                    0 * one, 0 * one, 0 * one)
    assert ok0.all()
    assert sorted(np.round(c0[0, :, 0], 6)) == pytest.approx([-100 * tw] * 2 + [100 * tw] * 2)
    assert sorted(np.round(c0[0, :, 1], 6)) == pytest.approx([-100 * tl] * 2 + [100 * tl] * 2)
    # 右滾 10°：機腹朝左 ⇒ 足跡往西移；左右緣 E = h·tan(±atan(tw) − φ)
    c, ok = footprint_corners_enu(np.zeros(1), np.zeros(1), 100 * one, tw * one, tl * one, 0 * one, 0 * one,
                                  10 * one)
    phi = math.radians(10)
    right = 100 * math.tan(math.atan(tw) - phi)
    left = 100 * math.tan(-math.atan(tw) - phi)
    assert sorted(c[0, :, 0]) == pytest.approx([left, left, right, right], abs=1e-9)
    # 抬頭 10°：足跡往北移；航向 90°：寬邊改沿南北
    cp, _ = footprint_corners_enu(np.zeros(1), np.zeros(1), 100 * one, tw * one, tl * one, 0 * one, 10 * one,
                                  0 * one)
    th = math.radians(10)
    assert sorted(cp[0, :, 1]) == pytest.approx([100 * math.tan(-math.atan(tl) + th)] * 2
                                                + [100 * math.tan(math.atan(tl) + th)] * 2, abs=1e-9)
    cy, _ = footprint_corners_enu(np.zeros(1), np.zeros(1), 100 * one, tw * one, tl * one, 90 * one, 0 * one,
                                  0 * one)
    assert np.ptp(cy[0, :, 1]) == pytest.approx(2 * 100 * tw)
    assert np.ptp(cy[0, :, 0]) == pytest.approx(2 * 100 * tl)
    # 滾到角點視線 > 85° ⇒ 標記不可用
    _, bad = footprint_corners_enu(np.zeros(1), np.zeros(1), 100 * one, tw * one, tl * one, 0 * one, 0 * one,
                                   60 * one)
    assert not bad.any()


@pytest.mark.parametrize("frac", [0.3, 0.5])
def test_two_parallel_strips_side_overlap_is_one_minus_s_over_w(frac):
    s = frac * W120
    # 三條帶：中間那條左右各一個鄰帶；外側兩條只能配最近的那條（隔一條的不算相鄰）
    it = mission([(0, -100, 0, 1100), (s, 1100, s, -100), (2 * s, -100, 2 * s, 1100)])
    rep = verify_preflight({"H": it}, rect(-50, 0, 2 * s + 50, 1000), HQ6, airspeed_mps=19.0, wind=(0, 0))
    so = rep.side_overlap
    m = [sum(1 for p in rep.photos if p.strip == k) for k in ("1", "2", "3")]
    # （照片, 鄰帶）組合：A→B、B→A、B→C、C→B；只取沿帶位置落在鄰帶範圍內的照片（兩端各可能少 1–2 張）
    assert m[0] + 2 * m[1] + m[2] - 8 <= so.n <= m[0] + 2 * m[1] + m[2]
    assert so.median == pytest.approx(1 - s / W120, abs=1e-3)
    assert so.minimum == pytest.approx(1 - s / W120, abs=1e-3)
    assert so.maximum == pytest.approx(1 - s / W120, abs=1e-3)


def test_seam_between_two_uavs_and_gap():
    s = 0.3 * W120                                           # 側向 0.70
    xa = [0.0, s, 2 * s]
    xb = [3 * s, 4 * s, 5 * s]
    poly = rect(-30, 0, 5 * s + 30, 1000)

    def run(xb_):
        ms = {"A": mission([(x, -150, x, 1150) for x in xa], home=(0, -300)),
              "B": mission([(x, -150, x, 1150) for x in xb_], home=(5 * s, -300))}
        return verify_preflight(ms, poly, HQ6, airspeed_mps=19.0, wind=(0, 0), ks=(1, 5), undercover_k=1)
    rep = run(xb)
    (sm,) = rep.seams
    assert (sm.uav_a, sm.uav_b) == ("A", "B")
    assert sm.width_median_m == pytest.approx(W120 - s, abs=1.0)          # 88.0 vs 88.015（格距 2 m）
    assert sm.frac_median == pytest.approx(1 - s / W120, abs=0.01)
    assert sm.width_min_m == pytest.approx(W120 - s, abs=1.0)
    assert sm.cells == pytest.approx((W120 - s) / 2.0 * 500, rel=0.02)      # 寬 44 格 × 長 500 格
    assert rep.seam_overlap_min == sm.frac_min
    assert rep.undercovered == []
    # A 的條帶只飛到測區北緣（覆蓋止於 ~1047），B 飛到 1150：北端「到只有 B 的區」會繞過 A 的端點而偏小，
    # 離「兩架都沒看到」的格不到一個刈幅的帶內格不採計 ⇒ 最小寬度仍是 w − s
    ms = {"A": mission([(x, -150, x, 1000) for x in xa], home=(0, -300)),
          "B": mission([(x, -150, x, 1150) for x in xb], home=(5 * s, -300))}
    st = verify_preflight(ms, poly, HQ6, airspeed_mps=19.0, wind=(0, 0), ks=(1, 5), undercover_k=1)
    assert st.seams[0].width_min_m == pytest.approx(W120 - s, abs=1.0)
    # B 整塊往東移，兩塊之間留 20 m 縫
    shift = W120 - s + 20.0
    rep2 = run([x + shift for x in xb])
    (g,) = rep2.seams
    assert g.cells == 0 and g.frac_min == 0.0
    assert g.gap_m == pytest.approx(20.0, abs=1.0)
    assert rep2.undercovered and rep2.undercovered[0].min_views == 0


def test_undercovered_polygon_detected_when_strip_removed():
    s = 0.7 * W120                                           # 側向 0.30：拿掉中間一條就破洞
    xs = [0.0, s, 2 * s, 3 * s, 4 * s]
    poly = rect(-20, 0, 4 * s + 20, 1000)
    kw = dict(airspeed_mps=19.0, wind=(0, 0), ks=(1, 5), undercover_k=1)
    full = verify_preflight({"H": mission([(x, -150, x, 1150) for x in xs])}, poly, HQ6, **kw)
    assert full.undercovered == [] and full.frac_ge[1] == 1.0
    # k 取直方圖第二小的視數：「< k」恰好是最小那一檔的格（不含剛好 = k 的格），與直方圖一致
    vs = sorted(full.views_histogram)
    assert len(vs) >= 2
    k = vs[1]
    kk = verify_preflight({"H": mission([(x, -150, x, 1150) for x in xs])}, poly, HQ6, airspeed_mps=19.0,
                          wind=(0, 0), ks=(k,), undercover_k=k)
    n_lt = full.views_histogram[vs[0]]
    assert sum(a.cells for a in kk.undercovered) == n_lt
    assert kk.frac_ge[k] == pytest.approx(1 - n_lt / kk.total_cells)
    assert {a.min_views for a in kk.undercovered} == {vs[0]}
    holed = verify_preflight({"H": mission([(x, -150, x, 1150) for x in xs if x != 2 * s])}, poly, HQ6, **kw)
    assert len(holed.undercovered) == 1
    a = holed.undercovered[0]
    gap_w = 2 * s - W120                                     # 兩鄰帶之間沒被看到的寬度
    assert a.min_views == 0
    assert a.area_m2 == pytest.approx(gap_w * 1000.0, rel=0.05)
    assert holed.undercovered_area_m2 == pytest.approx(a.area_m2)
    cx, cy = FR.to_xy(*a.centroid_latlon)
    assert float(cx) == pytest.approx(2 * s, abs=2.0)
    assert float(cy) == pytest.approx(500.0, abs=5.0)
    ex = np.array([FR.to_xy(la, lo) for la, lo in a.exterior_latlon])
    assert ex[:, 0].min() == pytest.approx(2 * s - gap_w / 2, abs=2.5)
    assert ex[:, 0].max() == pytest.approx(2 * s + gap_w / 2, abs=2.5)
    # 格心在測區內的格才算 ⇒ 邊緣最多差一格（2 m），且裁在測區內
    assert ex[:, 1].min() == pytest.approx(0.0, abs=2.0) and ex[:, 1].max() == pytest.approx(1000.0, abs=2.0)
    assert ex[:, 1].min() >= -1e-6 and ex[:, 1].max() <= 1000.0 + 1e-6
    assert holed.frac_ge[1] == pytest.approx(1 - gap_w * 1000 / holed.area_m2, abs=0.01)


def test_views_sum_matches_footprint_area():
    """光柵化守恆：所有格的視數總和 × 格面積 ≈ 每張足跡面積總和。"""
    it = mission([(0, 0, 0, 1000), (60, 1000, 60, 0)])
    rep = verify_preflight({"H": it}, rect(-200, -200, 260, 1200), HQ6, airspeed_mps=19.0, wind=(5.0, 300.0),
                           cell_size_m=1.0)
    n = rep.photos_used
    assert n == sum(rep.photos_per_uav.values()) > 80
    total = rep.grid.views.sum() * rep.cell_size_m ** 2
    assert total == pytest.approx(n * W120 * L120, rel=0.003)


# ---------------------------------------------------------------------------
# 飛後：photos.csv（F2）
# ---------------------------------------------------------------------------
def _pose_dict(p: PhotoPose) -> dict:
    return {k: getattr(p, k) for k in PhotoPose.__slots__}


def _write_f2(path, photos, geoid_n=18.5, blank_orth=False):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(F2_COLUMNS))
        w.writeheader()
        for i, p in enumerate(photos):
            w.writerow({"image_id": f"{p.uav}_{p.seq:06d}", "file": f"images/{p.seq:06d}.jpg", "uav": p.uav,
                        "strip": p.strip or "", "seq": p.seq, "lat_deg": repr(p.lat_deg), "lon_deg": repr(p.lon_deg),
                        "h_ell_m": repr(p.h_m + geoid_n), "h_orth_m": "" if blank_orth and i == 3 else repr(p.h_m),
                        "yaw_deg": repr(p.yaw_deg), "pitch_deg": "0", "roll_deg": "0", "cog_deg": repr(p.cog_deg),
                        "source": "fc_interp", "crs_h": "EPSG:4326", "crs_v": "EGM96", "flags": ""})


def test_postflight_reproduces_preflight_metrics(tmp_path):
    s = 0.3 * W120
    ms = {"A": mission([(0, -150, 0, 1150), (s, 1150, s, -150)]),
          "B": mission([(2 * s, -150, 2 * s, 1150), (3 * s, 1150, 3 * s, -150)], home=(3 * s, 0))}
    poly = rect(-40, 0, 3 * s + 40, 1000)
    kw = dict(cell_size_m=2.0, ks=(3, 5, 9), undercover_k=9)
    pre = verify_preflight(ms, poly, HQ6, airspeed_mps=19.0, wind=(5.0, 250.0), **kw)
    assert pre.photos_used > 200 and pre.seams and pre.undercovered
    f2 = tmp_path / "photos.csv"
    _write_f2(f2, pre.photos)
    post = verify_postflight(str(f2), poly, HQ6, terrain=0.0, height_field="h_orth_m", **kw)
    assert post.photos_per_uav == pre.photos_per_uav
    assert post.views_histogram == pre.views_histogram
    assert post.frac_ge == pre.frac_ge
    for a, b in ((pre.forward_overlap, post.forward_overlap), (pre.side_overlap, post.side_overlap)):
        assert a.n == b.n
        assert (a.minimum, a.median) == pytest.approx((b.minimum, b.median), abs=1e-7)
    assert [(x.uav_a, x.uav_b, x.cells) for x in post.seams] == [(x.uav_a, x.uav_b, x.cells) for x in pre.seams]
    assert post.seam_overlap_min == pytest.approx(pre.seam_overlap_min, abs=1e-9)
    assert [round(u.area_m2, 3) for u in post.undercovered] == [round(u.area_m2, 3) for u in pre.undercovered]
    assert post.effective["crs_v"] == {"EGM96": pre.photos_used}
    # 橢球高 + 橢球高地形 ⇒ 同一結果；橢球高 + 正高地形（基準混用）⇒ 足跡被放大 (120+18.5)/120
    ell = verify_postflight(str(f2), poly, HQ6, terrain=18.5, height_field="h_ell_m", **kw)
    assert ell.views_histogram == pre.views_histogram
    mixed = verify_postflight(str(f2), poly, HQ6, terrain=0.0, height_field="h_ell_m", **kw)
    assert mixed.views_histogram != pre.views_histogram
    assert mixed.side_overlap.median > pre.side_overlap.median + 0.03
    # 單張：足跡面積比例 = ((120+18.5)/120)²
    one = [p for p in pre.photos if p.strip == "1"][10:11]
    big = rect(-400, -400, 400, 1400)
    a = verify_postflight(one, big, HQ6, terrain=0.0)
    b = verify_postflight([PhotoPose(**{**_pose_dict(one[0]), "h_m": one[0].h_m + 18.5})], big, HQ6, terrain=0.0)
    k = (120 + 18.5) / 120
    assert b.grid.views.sum() == pytest.approx(a.grid.views.sum() * k * k, rel=0.01)


def test_postflight_refuses_missing_orthometric_height_and_requires_height_field(tmp_path):
    f2 = tmp_path / "photos.csv"
    ph = [PhotoPose("A", "1", i + 1, *ll(0, 20.0 * i), 120.0, 0.0, cog_deg=0.0) for i in range(10)]
    _write_f2(f2, ph, blank_orth=True)
    with pytest.raises(ValueError, match="N=0"):
        verify_postflight(str(f2), rect(-50, 0, 50, 180), HQ6, terrain=0.0, height_field="h_orth_m")
    with pytest.raises(ValueError, match="height_field"):
        verify_postflight(str(f2), rect(-50, 0, 50, 180), HQ6, terrain=0.0)
    poses, meta = read_photos_csv(f2, height_field="h_ell_m")
    assert [p.h_m for p in poses] == [138.5] * 10 and meta["height_field"] == "h_ell_m"


def test_postflight_actual_yaw_and_roll_change_footprint():
    """飛後用實際 yaw：機頭偏 90° 時寬邊沿航跡；roll 讓足跡側移。"""
    base = [PhotoPose("A", "1", i + 1, *ll(0, 19.0 * i), 120.0, 0.0) for i in range(40)]
    poly = rect(-200, 100, 200, 600)
    r0 = verify_postflight(base, poly, HQ6, terrain=0.0)
    turned = [PhotoPose(p.uav, p.strip, p.seq, p.lat_deg, p.lon_deg, p.h_m, 90.0) for p in base]
    r90 = verify_postflight(turned, poly, HQ6, terrain=0.0)
    assert r0.forward_overlap.median == pytest.approx(1 - 19.0 / L120, abs=1e-3)
    assert r90.forward_overlap.median == pytest.approx(1 - 19.0 / W120, abs=1e-3)
    rolled = [PhotoPose(p.uav, p.strip, p.seq, p.lat_deg, p.lon_deg, p.h_m, 0.0, roll_deg=10.0) for p in base]
    rr = verify_postflight(rolled, poly, HQ6, terrain=0.0)
    flat = verify_postflight(rolled, poly, HQ6, terrain=0.0, tilt=False)

    def x_mid(rep):                                          # 有視數的格，東西範圍中點（本地 x）
        g = rep.grid
        cols = np.nonzero(g.views.any(axis=0))[0]
        x = g.x0 + (cols + 0.5) * g.cell_size_m
        off = FR.to_xy(*g.frame.to_latlon(0.0, 0.0))[0]     # 兩個本地座標系原點的東西差
        return (x.min() + x.max()) / 2 + float(off)
    tw = W120 / 240.0
    expect = 120 * (math.tan(math.atan(tw) - math.radians(10)) + math.tan(-math.atan(tw) - math.radians(10))) / 2
    assert x_mid(flat) == pytest.approx(0.0, abs=2.0)
    assert x_mid(rr) == pytest.approx(expect, abs=2.0)      # 約 −27.2 m（往西）
    assert flat.effective["tilt"] is False


def test_agl_le_zero_photos_excluded_and_counted():
    ph = [PhotoPose("A", "1", i + 1, *ll(0, 19.0 * i), 120.0, 0.0) for i in range(10)]
    ph.append(PhotoPose("A", None, 11, *ll(0, 0), 0.0, 0.0))  # 地面試拍
    rep = verify_postflight(ph, rect(-60, 0, 60, 170), HQ6, terrain=0.0)
    assert rep.photos_used == 10 and rep.photos_excluded == {"agl_le_0": 1}
    assert rep.photos_per_uav == {"A": 11}


def test_metrics_input_validation():
    ph = [PhotoPose("A", "1", 1, *ll(0, 0), 120.0, 0.0)]
    with pytest.raises(ValueError, match="格網"):
        exposure_metrics(ph, rect(-50000, -50000, 50000, 50000), HQ6, terrain=0.0, cell_size_m=1.0)
    with pytest.raises(ValueError):
        exposure_metrics(ph, rect(-50, -50, 50, 50), HQ6, terrain=0.0, cell_size_m=0.0)
    with pytest.raises(ValueError, match="地形"):
        exposure_metrics(ph, rect(-50, -50, 50, 50), HQ6, terrain=None)
    with pytest.raises(ValueError):
        verify_preflight({"A": mission([(0, 0, 0, 100)])}, rect(-50, -50, 50, 50), HQ6,
                         airspeed_mps={"B": 19.0}, wind=(0, 0))


def test_local_frame_error_small_at_2km():
    """切平面用 WGS84 M、N 曲率半徑：與 pyproj aeqd 相比，±1 km 內 < 0.1 m。"""
    pyproj = pytest.importorskip("pyproj")
    lat0, lon0 = 25.10, 121.20
    fr = LocalFrame(lat0, lon0)
    aeqd = pyproj.Proj(proj="aeqd", lat_0=lat0, lon_0=lon0, ellps="WGS84")
    g = np.linspace(-1000, 1000, 9)
    X, Y = np.meshgrid(g, g)
    la, lo = fr.to_latlon(X.ravel(), Y.ravel())
    xa, ya = aeqd(lo, la)
    assert np.hypot(xa - X.ravel(), ya - Y.ravel()).max() < 0.1
    x2, y2 = fr.to_xy(la, lo)
    assert np.abs(x2 - X.ravel()).max() < 1e-6 and np.abs(y2 - Y.ravel()).max() < 1e-6


# ---------------------------------------------------------------------------
# 效能：2 × 2 km、2 m 格、三機 ~5,900 張
# ---------------------------------------------------------------------------
def test_performance_2km_three_uavs_under_30s():
    s, xs = 0.3 * W120, None
    xs = np.arange(-1000 + s / 2, 1000, s)
    ms = {}
    for bi, bx in enumerate(np.array_split(xs, 3)):
        strips = [(x, -1060, x, 1060) if k % 2 == 0 else (x, 1060, x, -1060) for k, x in enumerate(bx)]
        ms[f"U{bi}"] = mission(strips, r_acc=30.0, home=(bx[0], -1100))
    t0 = time.perf_counter()
    rep = verify_preflight(ms, rect(-1000, -1000, 1000, 1000), HQ6, airspeed_mps={"U0": 21.5, "U1": 19.0, "U2": 19.0},
                           wind=(5.0, 270.0), intrval_min_s=0.56, cell_size_m=2.0)
    dt = time.perf_counter() - t0
    assert sum(rep.photos_per_uav.values()) > 5300
    assert rep.total_cells == 1_000_000
    assert dt < 30.0, f"{dt:.1f} s"
    assert rep.frac_ge[5] == 1.0
    assert len(rep.seams) == 2


# ---------------------------------------------------------------------------
# R2 審查修正（2026-10-14）：DO_LAND_START 誤報、接縫只算相鄰兩架、吃規劃器的 CameraHardware、邊緣帶分開
# ---------------------------------------------------------------------------
def _hand(cmds, home=(0.0, 0.0)):
    """手排任務：home 之後依序 cmds；每列 (command, p1, p3, (x, y) 或 None)，NAV 高 120 m（相對 home）。"""
    it = [(0, 16, 0, 0, 0, 0, 0, *ll(*home), 0.0)]
    for c, p1, p3, pos in cmds:
        if pos is None:
            it.append((len(it), c, 3, p1, 0, p3, 0, 0, 0, 0))
        else:
            it.append((len(it), c, 3, p1, 0, p3, 0, *ll(*pos), 120.0))
    return it


def test_land_start_closed_only_when_its_do_run_turns_the_camera_off():
    """跳到 DO_LAND_START 後，它與下一個 NAV 之間的 DO 依序執行（AP_Mission advance_current_do_cmd）：
    這串 DO 跑完相機是關的才算關；仍開著才寫 note。"""
    nav0, nav1, nav2 = (16, 0, 0, (0, 0)), (16, 0, 0, (0, 500)), (16, 0, 0, (0, 900))
    on, off, ls, spd = (206, D80, 1, None), (206, 0, 0, None), (189, 0, 0, None), (178, 0, 0, None)

    def land(cmds):
        r = simulate_mission_photos(_hand(cmds), uav="H", airspeed_mps=19.0, wind=(0, 0))
        return [n for n in r.notes if "DO_LAND_START" in n], r.effective.get("land_start_in_open_206")
    # 匯出器的排法：開 … NAV(條帶終點) → 189 → 206(0) → NAV ⇒ 關、不寫 note（R2：每個建圖任務都誤報）
    assert land([nav0, on, nav1, ls, off, nav2]) == ([], [(4, "closed_by_seq_5")])
    # 189 與 206(0) 之間夾別的 DO（RTB 改空速）：同一串 DO 仍會跑到 206(0) ⇒ 關
    assert land([nav0, on, nav1, ls, spd, off, nav2]) == ([], [(4, "closed_by_seq_6")])
    # 206(0) 是任務最後一列（後面沒有 NAV）⇒ 關
    assert land([nav0, on, nav1, ls, off]) == ([], [(4, "closed_by_seq_5")])
    # 真的開著：189 後面直接是 NAV（206(0) 在那個 NAV 之後才執行）
    n, eff = land([nav0, on, nav1, ls, nav2, off])
    assert len(n) == 1 and "seq 4 DO_LAND_START" in n[0] and eff == [(4, "open")]
    # 206(0) 之後又 206(d) 重新打開 ⇒ 這串 DO 跑完相機開著
    n, eff = land([nav0, on, nav1, ls, off, on, nav2, off])
    assert len(n) == 1 and "seq 4 DO_LAND_START" in n[0] and eff == [(4, "open")]
    # 相機已關時的 189 不在檢查範圍
    assert land([nav0, on, nav1, off, ls, nav2]) == ([], [])


@pytest.fixture(scope="module")
def real_mapping():
    """真的規劃一次建圖模式，再用 plan_to_mission_items 匯出（與上傳飛控同一支函式）。
    1 km² 平地、1 Pioneer + 2 HERO、HQ6（規劃器的 CameraHardware）、側向 0.70／前向 0.80、2 Hz、引入段 50 m。"""
    from hv_dccpp.ardupilot import CameraTriggerConfig, plan_to_mission_items
    from hv_dccpp.context import build_context
    from hv_dccpp.integration.aeroplan_adapter import to_mission_inputs
    from hv_dccpp.mission import MappingModeConfig, PayloadRequirement, PlannerConfig
    from hv_dccpp.planner import plan_mission
    from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware
    home = (23.95, 120.90)
    m_lat, m_lon = 111132.0, 111412.0 * math.cos(math.radians(home[0]))

    def g(e, n):
        return (home[0] + n / m_lat, home[1] + e / m_lon)
    poly = [g(-500, -500), g(500, -500), g(500, 500), g(-500, 500)]
    hw = CameraHardware(focal_length_mm=6.0, sensor_width_mm=4056 * 1.55e-3, sensor_height_mm=3040 * 1.55e-3,
                        pixel_pitch_um=1.55, max_shutter_rate_hz=2.0, is_default_example=False)
    classes = {1: "A", 2: "B", 3: "B"}
    drones = [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": g(-60.0 + 60.0 * i, -900.0), "speed": 20.0,
               "heading": 0.0, "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": 125.7} for i in range(3)]
    req = PayloadRequirement("default", "rgb", gsd_required_m=0.05, side_overlap=0.70, forward_overlap=0.80,
                             h_target_agl_m=120.0)
    cfg = PlannerConfig(dem_step_m=25.0, dh_band_min_m=15.0, d_sep_m=50.0, dh_sep_m=50.0)
    inputs, _ = to_mission_inputs([{"area_id": 1, "priority": 1.0, "polygon": poly}], drones, coverage_width_m=125.7,
                                  overlap_rate=0.70, coverage_altitude=120.0, home=g(0.0, -900.0),
                                  class_of=lambda d: classes[int(d["drone_id"])],
                                  platform_a=dataclasses.replace(PLATFORM_PIONEER, min_fw_operation_agl_m=100.0),
                                  platform_b=PLATFORM_HERO, sensor_a=(hw,), sensor_b=(hw,),
                                  payload_reqs={"default": req}, wind=(0.0, 0.0), waiver_max_agl_m=None,
                                  dem_margin_m=3000.0, mapping_mode=MappingModeConfig(lead_in_m=50.0))
    ctx = build_context(inputs, cfg)
    mp = plan_mission(inputs, cfg, ctx=ctx)
    cc = CameraTriggerConfig(interval_min_s=0.5, camera_lead_in_m=50.0)
    missions = {u: plan_to_mission_items(up, ctx, cfg, camera_cfg=cc) for u, up in mp.plans.items()}
    return SimpleNamespace(poly=poly, hw=hw, missions=missions)


def test_real_mapping_mission_no_land_start_false_alarm_and_planner_camera(real_mapping):
    rm = real_mapping
    assert len(rm.missions) >= 2
    expect = {}
    for u, items in rm.missions.items():
        on = False
        for i, it in enumerate(items):
            if it.command == 206:
                on = it.param1 > 0
            elif it.command == 189 and on:
                # 前提（否則這條測試量不到東西）：照任務順序 189 落在開啟區間內，下一列就是 206(0)
                nxt = items[i + 1]
                assert (nxt.command, nxt.param1) == (206, 0.0), (u, it.seq)
                expect[u] = [(it.seq, f"closed_by_seq_{nxt.seq}")]
    assert expect, "匯出器已不再把 DO_LAND_START 放在開啟區間內：這條測試要改用新的排法"
    kw = dict(airspeed_mps={u: (21.5 if u.startswith("A") else 19.0) for u in rm.missions}, wind=(0.0, 0.0),
              intrval_min_s=0.5, trigger_duration_s=0.1)
    fc = verify_preflight(rm.missions, [rm.poly], HQ6, **kw)
    assert [n for n in fc.notes if "DO_LAND_START" in n] == []
    for u, e in expect.items():
        assert fc.simulations[u].effective["land_start_in_open_206"] == e
    rep = verify_preflight(rm.missions, [rm.poly], rm.hw, **kw)          # 直接吃規劃器的 CameraHardware
    assert "sensor_width_mm" in rep.effective["camera_model"]
    assert [n for n in rep.notes if "DO_LAND_START" in n] == []
    assert rep.views_histogram == fc.views_histogram and rep.frac_ge == fc.frac_ge
    assert [(s.uav_a, s.uav_b, s.cells, s.width_min_m) for s in rep.seams] == \
        [(s.uav_a, s.uav_b, s.cells, s.width_min_m) for s in fc.seams]
    # 邊緣帶 w/2：內部 ≥ 5 視 100%；視數不足（若有）全在邊緣帶，逐格與分量兩種加總都對得起來
    eb, inn = rep.edge_band, rep.interior
    assert rep.edge_band_m == pytest.approx(W120 / 2, abs=0.01)                # AGL 119.9998 m：w 少 0.02 mm 級
    assert eb.cells + inn.cells == rep.total_cells
    assert inn.frac_ge[5] == 1.0 and inn.undercovered_cells == 0
    assert all(a.region == "edge_band" and a.edge_band_cells == a.cells for a in rep.undercovered)
    assert eb.undercovered_cells == sum(c for v, c in rep.views_histogram.items() if v < 5)
    assert rep.undercovered_edge_band_area_m2 + rep.undercovered_interior_area_m2 == \
        pytest.approx(rep.undercovered_area_m2)


def test_seam_only_between_adjacent_uavs_three_interleaved():
    """R2 的排法：B2 … | A1（第一條插在 B2 最後一條旁）| B3 …。B2、B3 的足跡在 A1 底下偶然重疊 w − 2.75s
    （= 0.175·w），舊算法把它當接縫 ⇒ seam_overlap_min 0.175。只算「恰好只有兩架」的格 ⇒ 只剩 A1/B2、A1/B3。"""
    s = 0.3 * W120
    xs = {"B2": [0.0, s, 2 * s, 3 * s], "A1": [2.75 * s, 3.75 * s, 4.75 * s],
          "B3": [5.75 * s, 6.75 * s, 7.75 * s, 8.75 * s]}
    ms = {u: mission([(x, -150, x, 1150) for x in v], home=(v[0], -300)) for u, v in xs.items()}
    rep = verify_preflight(ms, rect(-20, 0, 8.75 * s + 20, 1000), HQ6, airspeed_mps=19.0, wind=(0, 0),
                           ks=(1, 5), undercover_k=1)
    by = {(m.uav_a, m.uav_b): m for m in rep.seams}
    assert set(by) == {("A1", "B2"), ("A1", "B3")}
    # 寬度是兩架之間的重疊寬（與第三架無關）：A1/B2 =（3s + w/2）−（2.75s − w/2）= w + 0.25s；A1/B3 = w − s
    # （光柵量化 ±1 格 = 2 m）
    assert by["A1", "B2"].width_min_m == pytest.approx(W120 + 0.25 * s, abs=2.0)
    assert by["A1", "B3"].width_min_m == pytest.approx(W120 - s, abs=1.0)
    assert by["A1", "B3"].width_median_m == pytest.approx(W120 - s, abs=1.0)
    assert rep.seam_overlap_min == pytest.approx(1 - s / W120, abs=0.01)
    # B2/B3 的重疊（寬 w − 2.75s）全都有 A1 ⇒ 不相鄰、記在 effective（不靜默丟掉）
    ((a, b, shared),) = rep.effective["seam_pairs_not_adjacent"]
    assert (a, b) == ("B2", "B3")
    assert shared == pytest.approx((W120 - 2.75 * s) / 2.0 * 500, rel=0.03)
    # 三架都看到的格不進 A1/B2、A1/B3 的帶，數量記在 shared_cells；A1/B3 帶寬 =（4.75s + w/2）−（3s + w/2）
    assert by["A1", "B2"].shared_cells == by["A1", "B3"].shared_cells == shared
    assert by["A1", "B3"].cells == pytest.approx(1.75 * s / 2.0 * 500, rel=0.03)


def test_gap_seam_only_through_uncovered_cells_between_the_two():
    """A、B 不重疊且相距 < w：(1) 中間整段被 C 蓋住 ⇒ A/B 不是接縫（舊算法報 gap、seam_overlap_min = 0）；
    (2) C 往東移、與 A 之間留 20 m 沒人看到 ⇒ 縫記在 A/C，不在 A/B。"""
    s = 0.3 * W120
    xa = [0.0, s, 2 * s]
    a_end = 2 * s + W120 / 2

    def run(xc, xb0):
        ms = {"A": mission([(x, -150, x, 1150) for x in xa], home=(0, -300)),
              "C": mission([(xc, -150, xc, 1150)], home=(xc, -300)),
              "B": mission([(x, -150, x, 1150) for x in (xb0, xb0 + s, xb0 + 2 * s)], home=(xb0, -300))}
        return verify_preflight(ms, rect(-30, 0, xb0 + 2 * s + 30, 1000), HQ6, airspeed_mps=19.0, wind=(0, 0),
                                ks=(1, 5), undercover_k=1)
    xb0 = a_end + 0.3 * W120 + W120 / 2                         # B 覆蓋從 a_end + 0.3w 開始
    rep = run((a_end + xb0 - W120 / 2) / 2, xb0)                 # C 置中：與兩邊各重疊 0.35w
    assert rep.undercovered == []
    by = {(m.uav_a, m.uav_b): m for m in rep.seams}
    assert set(by) == {("A", "C"), ("B", "C")}
    for m in by.values():
        assert m.gap_m is None and m.width_min_m == pytest.approx(0.35 * W120, abs=1.0)
    assert rep.effective["seam_pairs_not_adjacent"] == []
    assert rep.seam_overlap_min == pytest.approx(0.35, abs=0.01)
    xc2 = a_end + 20.0 + W120 / 2                                # C 覆蓋從 a_end + 20 開始
    xb2 = xc2 + W120 / 2 - 50.0 + W120 / 2                       # B 與 C 重疊 50 m；A–B 相距 95.7 m < w
    rep2 = run(xc2, xb2)
    by2 = {(m.uav_a, m.uav_b): m for m in rep2.seams}
    assert set(by2) == {("A", "C"), ("B", "C")}
    assert by2["A", "C"].cells == 0 and by2["A", "C"].frac_min == 0.0
    assert by2["A", "C"].gap_m == pytest.approx(20.0, abs=1.0)
    assert by2["B", "C"].width_min_m == pytest.approx(50.0, abs=1.0)
    assert rep2.seam_overlap_min == 0.0
    (hole,) = rep2.undercovered                                  # 縫整條南北向 ⇒ 伸入內部 ⇒ 真破洞
    assert hole.min_views == 0 and hole.region == "interior"


def test_touching_blocks_zero_overlap_zero_gap_is_a_seam_of_width_zero():
    """兩架的「只有自己」區直接相接（足跡邊對邊、中間沒有空格也沒有共同格）⇒ 相鄰、重疊 0、縫 0。"""
    ys = [-560.0 + 19.0 * i for i in range(60)]
    ph = [PhotoPose(u, "1", i + 1, *ll(x, y), 120.0, 0.0) for u, x in (("A", -W120 / 2), ("B", W120 / 2))
          for i, y in enumerate(ys)]
    rep = exposure_metrics(ph, rect(-W120 + 5, -500, W120 - 5, 500), HQ6, terrain=0.0, ks=(1,), undercover_k=1)
    g = rep.grid
    va, vb = g.views_by_uav["A"] > 0, g.views_by_uav["B"] > 0
    assert not (va & vb & g.inside).any() and (va | vb)[g.inside].all()      # 前提：真的邊對邊
    (m,) = rep.seams
    assert (m.uav_a, m.uav_b, m.cells, m.width_min_m, m.frac_min, m.gap_m) == ("A", "B", 0, 0.0, 0.0, 0.0)


def test_planner_camera_hardware_footprint_equals_frame_camera():
    from hv_dccpp.platform import CameraHardware
    hw = CameraHardware(focal_length_mm=6.0, sensor_width_mm=4056 * 1.55e-3, sensor_height_mm=3040 * 1.55e-3,
                        pixel_pitch_um=1.55, max_shutter_rate_hz=2.0, is_default_example=False)
    assert _footprint_wl(hw, 120.0) == pytest.approx((125.736, 94.24), rel=1e-12)
    assert _footprint_wl(hw, 120.0) == pytest.approx(HQ6.footprint_m(120.0), rel=1e-12)
    # 鴨子型別：焦距 + 像素間距 + 影像像素寬高（沒有感測器尺寸）
    duck = SimpleNamespace(focal_length_mm=6.0, pixel_pitch_um=1.55, width_px=4056, height_px=3040)
    assert _footprint_wl(duck, 120.0) == pytest.approx((125.736, 94.24), rel=1e-12)
    # 兩種給法都有：一致就接受；不一致就拒絕，不自行挑一個
    both = dict(focal_length_mm=6.0, sensor_width_mm=6.2868, sensor_height_mm=4.712, pixel_pitch_um=1.55,
                image_width_px=4056, image_height_px=3040)
    assert _footprint_wl(SimpleNamespace(**both), 60.0) == pytest.approx((62.868, 47.12), rel=1e-9)
    with pytest.raises(ValueError, match="不一致"):
        _footprint_wl(SimpleNamespace(**{**both, "sensor_width_mm": 7.0}), 120.0)
    with pytest.raises(ValueError, match="不一致"):
        _footprint_wl(SimpleNamespace(**{**both, "image_height_px": 3000}), 120.0)
    with pytest.raises(TypeError, match="pixel_pitch_um"):
        _footprint_wl(SimpleNamespace(focal_length_mm=6.0, pixel_pitch_um=1.55), 120.0)
    with pytest.raises(ValueError, match="focal_length_mm"):
        _footprint_wl(dataclasses.replace(hw, focal_length_mm=0.0), 120.0)
    with pytest.raises(TypeError):
        _footprint_wl(object(), 120.0)
    # 整條驗證：同一任務用 CameraHardware 與 FrameCamera ⇒ 度量相同，effective 寫出用了哪個公式
    s = 0.3 * W120
    ms = {"A": mission([(0, -150, 0, 1150), (s, 1150, s, -150)]),
          "B": mission([(2 * s, -150, 2 * s, 1150), (3 * s, 1150, 3 * s, -150)], home=(3 * s, 0))}
    poly = rect(-40, 0, 3 * s + 40, 1000)
    r_hw = verify_preflight(ms, poly, hw, airspeed_mps=19.0, wind=(5.0, 250.0))
    r_fc = verify_preflight(ms, poly, HQ6, airspeed_mps=19.0, wind=(5.0, 250.0))
    assert r_hw.views_histogram == r_fc.views_histogram
    assert r_hw.forward_overlap.median == pytest.approx(r_fc.forward_overlap.median, abs=1e-12)
    assert r_hw.side_overlap.median == pytest.approx(r_fc.side_overlap.median, abs=1e-12)
    assert [(x.cells, x.width_min_m) for x in r_hw.seams] == [(x.cells, x.width_min_m) for x in r_fc.seams]
    assert r_hw.effective["camera_model"].startswith("CameraHardware: w = AGL·6.2868/6")
    assert r_fc.effective["camera_model"] == "FrameCamera.footprint_m"


def test_edge_band_separates_outer_strip_shortfall_from_interior():
    """R2：最外側條帶外緣只有一條帶蓋到，單帶前向 l/間距 = 94.24/19 = 4.96 ⇒ 4–5 視，k = 5 時邊緣有 4 視的格。
    它們都在離邊界 w/2 內 ⇒ 標 edge_band、有自己的比例；內部 ≥ 5 視 100%；整體數字照舊（不變好看）。"""
    s = 0.3 * W120
    poly = rect(-45.0, 0, 5 * s + 45.0, 1000)                    # 邊界在最外側條帶外 45 m：鄰帶只蓋到 25.2 m
    rep = verify_preflight({"H": mission([(k * s, -150, k * s, 1150) for k in range(6)])}, poly, HQ6,
                           airspeed_mps=19.0, wind=(0, 0), ks=(5, 9))
    eb, inn = rep.edge_band, rep.interior
    assert rep.edge_band_m == pytest.approx(W120 / 2, abs=1e-9) and rep.effective["edge_band_m"] == rep.edge_band_m
    assert "w/2" in rep.effective["edge_band_source"]
    n_lt5 = sum(c for v, c in rep.views_histogram.items() if v < 5)
    assert n_lt5 > 0 and rep.frac_ge[5] == pytest.approx(1 - n_lt5 / rep.total_cells)
    assert inn.frac_ge[5] == 1.0 and inn.undercovered_cells == 0 and inn.min_views >= 5
    assert eb.min_views == 4 and eb.undercovered_cells == n_lt5
    assert eb.undercovered_frac == pytest.approx(n_lt5 / eb.cells) and eb.frac_ge[5] == pytest.approx(1 - n_lt5 / eb.cells)
    assert eb.undercovered_area_m2 == n_lt5 * 4.0 and eb.area_m2 == eb.cells * 4.0
    assert eb.cells + inn.cells == rep.total_cells
    # 邊緣帶 = 測區內、格心不在「內縮 w/2 的矩形」內的格：在報告的格網上獨立數一次
    g = rep.grid
    cx = g.x0 + (np.arange(g.views.shape[1]) + 0.5) * g.cell_size_m
    cy = g.y0 + (np.arange(g.views.shape[0]) + 0.5) * g.cell_size_m
    px_, py_ = g.frame.to_xy(*np.array(poly).T)
    hw2 = W120 / 2
    core = ((cx[None, :] > px_.min() + hw2) & (cx[None, :] < px_.max() - hw2)
            & (cy[:, None] > py_.min() + hw2) & (cy[:, None] < py_.max() - hw2))
    assert eb.cells == pytest.approx(int((g.inside & ~core).sum()), abs=2)
    wp, hp = 5 * s + 90.0, 1000.0                                # 解析值：測區 − 內縮矩形（格心量化 ±周長·格/2）
    assert eb.area_m2 == pytest.approx(wp * hp - (wp - W120) * (hp - W120), abs=(2 * wp + 2 * hp) * 1.0)
    assert rep.undercovered and {a.region for a in rep.undercovered} == {"edge_band"}
    assert all(a.edge_band_cells == a.cells and a.min_views == 4 for a in rep.undercovered)
    assert rep.undercovered_edge_band_area_m2 == pytest.approx(rep.undercovered_area_m2)
    assert rep.undercovered_interior_area_m2 == 0.0
    assert any("邊緣帶" in n and "伸入內部 0 處" in n for n in rep.notes)


def test_hole_reaching_the_interior_is_not_an_edge_effect():
    """拿掉中間一條帶：破洞南北貫穿，兩端落在邊緣帶、中段在內部 ⇒ 整個分量算 interior（真破洞）。"""
    s = 0.7 * W120
    poly = rect(-20, 0, 4 * s + 20, 1000)
    ms = {"H": mission([(x, -150, x, 1150) for x in (0.0, s, 3 * s, 4 * s)])}
    kw = dict(airspeed_mps=19.0, wind=(0, 0), ks=(1, 5), undercover_k=1)
    rep = verify_preflight(ms, poly, HQ6, **kw)
    (a,) = rep.undercovered
    assert a.region == "interior" and 0 < a.edge_band_cells < a.cells and a.min_views == 0
    assert a.edge_band_cells == pytest.approx(2 * (W120 / 2) / 2.0 * (2 * s - W120) / 2.0, rel=0.1)
    assert rep.undercovered_interior_area_m2 == pytest.approx(a.area_m2)
    assert rep.undercovered_edge_band_area_m2 == 0.0
    assert rep.interior.min_views == 0 and rep.interior.undercovered_cells > 0
    # 邊緣帶寬由呼叫端指定：0 ⇒ 沒有邊緣帶；比半個測區還寬 ⇒ 整個測區都是邊緣帶（視數 0 仍會在 note 標出）
    z = verify_preflight(ms, poly, HQ6, edge_band_m=0.0, **kw)
    assert z.edge_band_m == 0.0 and z.effective["edge_band_source"] == "argument"
    assert z.edge_band.cells == 0 and z.interior.cells == z.total_cells
    assert z.undercovered[0].region == "interior" and z.undercovered[0].edge_band_cells == 0
    big = verify_preflight(ms, poly, HQ6, edge_band_m=1000.0, **kw)
    assert big.interior.cells == 0 and big.undercovered[0].region == "edge_band"
    assert any("視數 0" in n for n in big.notes)
    for bad in (-1.0, float("nan"), "w/2"):
        with pytest.raises(ValueError, match="edge_band_m"):
            verify_preflight(ms, poly, HQ6, edge_band_m=bad, **kw)


def test_edge_band_undefined_without_photos():
    """沒有可用照片：w 無從定義 ⇒ 不分邊緣帶（None），視數不足一律當 interior，並寫明原因；給了寬度就照給的算。"""
    rep = exposure_metrics([], rect(-50, 0, 50, 100), HQ6, terrain=0.0)
    assert rep.edge_band is None and rep.interior is None and rep.edge_band_m is None
    assert rep.undercovered and {a.region for a in rep.undercovered} == {"interior"}
    assert "沒有可用照片" in rep.effective["edge_band_source"]
    given = exposure_metrics([], rect(-50, 0, 50, 100), HQ6, terrain=0.0, edge_band_m=10.0)
    assert given.edge_band_m == 10.0 and given.edge_band.cells + given.interior.cells == given.total_cells
    assert given.edge_band.area_m2 == pytest.approx(100 * 100 - 80 * 80, rel=0.02)
