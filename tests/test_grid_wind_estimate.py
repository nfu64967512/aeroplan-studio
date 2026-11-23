"""網格風重估 HARVEST 計畫的航時與能量（core/grid_wind_estimate.py）。

手算情境（自己組 UAVPlan，規劃器時間照規劃風手算）鎖住：順逆風與側風的方向慣例、「每條帶取沿線平均風」的語意
（不是逐點 1/V_g 平均）、分層風依離地高選層（一樣近取低、該層缺值不換層）、網格外不外推、等待時間原樣保留、
能量差 ＝ 同一個能量模型、四種大聲的問題。真規劃器（契約 mini 情境）鎖住：重建時間對規劃器時間的殘差、
網格風＝規劃風時差 0、對獨立的 L1 追蹤模擬（hv_dccpp.sim.track_simulate）換風後的落地時刻差。

一律不連網、不寫檔（AEROPLAN_NO_FILE_LOG 由 conftest 設）。
"""
from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from core import grid_wind_estimate as gwe
from core import wind_field as wf
from hv_dccpp.geo import GeoFrame
from hv_dccpp.mission import MissionPlan, Mode, Phase, PlannerConfig, UAVPlan, Waypoint, WindLayer
from hv_dccpp.platform import PLATFORM_HERO, EnergyModel
from hv_dccpp.wind import WindField as HvWindField
from hv_dccpp.wind import time_arc, wind_vector

FIX = Path(__file__).resolve().parent / "fixtures"
LAT0, LON0 = 23.50, 120.37
VA = 20.0


# ─────────────────────────────────────────────────────────────────────
#  手算情境工具
# ─────────────────────────────────────────────────────────────────────
FRAME = GeoFrame(LAT0, LON0, 0.0)


def hv_wind(speed: float = 0.0, dir_from: float = 0.0) -> HvWindField:
    layers = [WindLayer(0.0, 3000.0, speed, dir_from, gust_mps=speed * 1.3, z_ref="agl")] if speed > 0 else []
    return HvWindField(layers, FRAME, 0.0)


def model() -> EnergyModel:
    return EnergyModel(spec=PLATFORM_HERO, auw_kg=6.65, available_wh=380.0, nominal_wh=420.0, reference_wh=None)


def ctx_of(wind: HvWindField, cfg: PlannerConfig | None = None, em: EnergyModel | None = None):
    em = em or model()
    return SimpleNamespace(cfg=cfg or PlannerConfig(), frame=FRAME, wind=wind, specs={"B": PLATFORM_HERO},
                           models={"B1": em}, models_rep={"B": em})


def wp(x, y, t, phase, *, z=100.0, agl=100.0, va=VA, track=None, arc=None, strip=None, d=None, e=300.0):
    lat, lon, alt = FRAME.to_wgs84(np.array([x, y, z]))[0]
    return Waypoint(x_m=float(x), y_m=float(y), z_m=float(z), lat_deg=float(lat), lon_deg=float(lon),
                    alt_msl_m=float(alt), agl_m=float(agl), t_s=float(t), mode=Mode.CRUISE, phase=phase,
                    sensor_on=phase is Phase.OPERATE, strip_id=strip, cell_id=None, dir=d, airspeed_mps=float(va),
                    ground_speed_mps=float(va), track_deg=float(track if track is not None else 0.0), heading_deg=0.0,
                    energy_remaining_wh=float(e), nearest_alternate="H", alt_reachable=True, lost_link_action="RTB",
                    arc=arc)


def uplan(wps, *, margin=100.0, used=150.0) -> UAVPlan:
    return UAVPlan("B1", "B", wps, [], float(wps[0].t_s), float(wps[-1].t_s), float(wps[-1].t_s), "H", float(used),
                   float(margin), 0.0, 0.0, 0, [], takeoff_site="H")


def mplan(*ups) -> MissionPlan:
    plans = {u.uav_id: u for u in ups}
    return MissionPlan(plans, max(u.t_land_s for u in ups), [], {}, [], None, {}, (0, 0, 0))


def two_strips(L=2000.0, *, agl=100.0, t_east=None, t_west=None, gap=0.0):
    """往東一條、轉回往西一條（同一條線來回，中間 ``gap`` 秒的 0 長度等待）。規劃器時間預設照無風：L/Va。"""
    te = L / VA if t_east is None else t_east
    tw = L / VA if t_west is None else t_west
    w = [wp(0, 0, 0.0, Phase.OPERATE, agl=agl, track=90, strip="S1", d=1),
         wp(L, 0, te, Phase.OPERATE, agl=agl, track=90, strip="S1", d=1)]
    if gap:
        w.append(wp(L, 0, te, Phase.ORBIT, agl=agl, track=270))
    w += [wp(L, 0, te + gap, Phase.OPERATE, agl=agl, track=270, strip="S2", d=-1),
          wp(0, 0, te + gap + tw, Phase.LAND, agl=agl, track=270)]
    return w


def bbox_of(wps, pad=0.02):
    la = [w.lat_deg for w in wps]
    lo = [w.lon_deg for w in wps]
    return (min(la) - pad, min(lo) - pad, max(la) + pad, max(lo) + pad)


def run_of(est, strip):
    return next(r for e in est.uavs.values() for r in e.runs if r.strip_id == strip)


# ─────────────────────────────────────────────────────────────────────
#  方向慣例與解析解
# ─────────────────────────────────────────────────────────────────────
def test_tailwind_headwind_are_the_right_way_round():
    """規劃無風；網格 5 m/s 來自 270°（西風，往東吹）：往東順風 2000/25 = 80 s，往西逆風 2000/15 = 133.33 s。"""
    w = two_strips()
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 5.0, 270.0))
    assert run_of(est, "S1").dt_plan_s == pytest.approx(100.0)
    assert run_of(est, "S1").dt_grid_s == pytest.approx(80.0, abs=1e-9)
    assert run_of(est, "S2").dt_grid_s == pytest.approx(2000.0 / 15.0, abs=1e-9)
    e = est.uavs["B1"]
    assert e.t_land_grid_s - e.t_land_plan_s == pytest.approx(-20.0 + 2000.0 / 15.0 - 100.0, abs=1e-9)
    assert est.makespan_grid_s == pytest.approx(e.t_land_grid_s)


def test_crosswind_is_sqrt_va2_minus_w2():
    w = [wp(0, 0, 0.0, Phase.OPERATE, track=0, strip="N", d=1), wp(0, 2000, 100.0, Phase.OPERATE, track=0, strip="N", d=1),
         wp(0, 2000, 100.0, Phase.LAND)]
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 5.0, 270.0))
    assert run_of(est, "N").dt_grid_s == pytest.approx(2000.0 / math.sqrt(VA ** 2 - 25.0), abs=1e-9)


def test_planner_wind_is_the_effective_ctx_wind_not_zero():
    """規劃器用 5 m/s 西風算好的時間（80／133.3 s）＋同樣的網格風 ⇒ 差 0；拿掉 ctx.wind 才會有差。"""
    w = two_strips(t_east=80.0, t_west=2000.0 / 15.0)
    pl = mplan(uplan(w))
    src = wf.uniform_field(bbox_of(w), 5.0, 270.0)
    est = gwe.estimate_plan(pl, ctx_of(hv_wind(5.0, 270.0)), src)
    e = est.uavs["B1"]
    assert e.t_land_grid_s == pytest.approx(e.t_land_plan_s, abs=1e-9)
    assert e.residual_max_s == pytest.approx(0.0, abs=1e-6)
    assert "5.00 m/s 來自 270°" in est.plan_wind_text
    assert est.problems == []


# ─────────────────────────────────────────────────────────────────────
#  沿線平均風（每條帶一個）
# ─────────────────────────────────────────────────────────────────────
def _east_gradient_field(w0, w1, u0, u1, lat_pad=0.01):
    """經度方向線性的東風分量：西緣 u0、東緣 u1（v = 0）；緯度兩列相同。"""
    lats = (w0.lat_deg - lat_pad, w0.lat_deg + lat_pad)
    lons = (w0.lon_deg, w1.lon_deg)
    return wf.WindField(lats=lats, lons=lons, u=((u0, u1), (u0, u1)), v=((0.0, 0.0), (0.0, 0.0)), height_m=120.0,
                        source="open-meteo", label="合成梯度")


def test_strip_uses_along_line_mean_wind_not_pointwise_average():
    """往東吹的風沿條帶 0 → 10 m/s 線性變化：沿線平均 5 m/s ⇒ 2000/25 = 80.00 s。
    （逐點平均 1/V_g 會是 (2000/10)·ln(30/20) = 81.09 s——使用者定的是「每條帶取沿線平均風」。）"""
    w = [wp(0, 0, 0.0, Phase.OPERATE, track=90, strip="S1", d=1), wp(2000, 0, 100.0, Phase.OPERATE, track=90, strip="S1", d=1),
         wp(2000, 0, 100.0, Phase.LAND, track=90)]
    f = _east_gradient_field(w[0], w[1], 0.0, 10.0)
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), f)
    r = run_of(est, "S1")
    assert r.covered
    assert r.grid_uv[0] == pytest.approx(5.0, abs=0.02)
    assert r.dt_grid_s == pytest.approx(80.0, abs=0.05)
    assert abs(r.dt_grid_s - 200.0 * math.log(1.5)) > 1.0


def test_strip_split_into_several_waypoints_shares_one_mean():
    """規劃器把長條帶切成 ≤ 500 m 的航點：同一條帶（同 strip_id／dir、連續）只取一個平均風，結果與一段相同。"""
    xs = [0, 500, 1000, 1500, 2000]
    w = [wp(x, 0, x / VA, Phase.OPERATE, track=90, strip="S1", d=1) for x in xs] + [wp(2000, 0, 100.0, Phase.LAND, track=90)]
    f = _east_gradient_field(w[0], w[4], 0.0, 10.0)
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), f)
    runs = [r for r in est.uavs["B1"].runs if r.strip_id == "S1"]
    assert len(runs) == 1 and runs[0].wp_from == 0 and runs[0].length_m == pytest.approx(2000.0)
    assert runs[0].dt_grid_s == pytest.approx(80.0, abs=0.05)


# ─────────────────────────────────────────────────────────────────────
#  分層風：依離地高選層
# ─────────────────────────────────────────────────────────────────────
def _profile(bb, w10=(0.0, 0.0), w80=(5.0, 270.0), w120=(10.0, 270.0), nan80=False):
    def layer(h, s, d, nan=False):
        u, v = wf.dir_to_uv(d, s)
        if nan:
            u = v = math.nan
        lats, lons = (bb[0], bb[2]), (bb[1], bb[3])
        return wf.WindField(lats=lats, lons=lons, u=((u, u), (u, u)), v=((v, v), (v, v)), height_m=h,
                            source="open-meteo", label=f"{h:g} m")
    return wf.WindProfile(layers=(layer(10.0, *w10), layer(80.0, *w80, nan=nan80), layer(120.0, *w120)),
                          model="icon_global", time_utc="2026-10-23T07:00", label="合成分層", bbox=bb)


@pytest.mark.parametrize("agl,expect_h,expect_vg", [(115.0, 120.0, 30.0), (95.0, 80.0, 25.0), (100.0, 80.0, 25.0),
                                                     (30.0, 10.0, 20.0), (500.0, 120.0, 30.0)])
def test_profile_picks_nearest_layer_by_agl_ties_go_low(agl, expect_h, expect_vg):
    w = [wp(0, 0, 0.0, Phase.OPERATE, agl=agl, track=90, strip="S1", d=1),
         wp(2000, 0, 100.0, Phase.OPERATE, agl=agl, track=90, strip="S1", d=1), wp(2000, 0, 100.0, Phase.LAND, agl=agl)]
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), _profile(bbox_of(w)))
    r = run_of(est, "S1")
    assert r.layer_heights_m == (expect_h,)
    assert r.dt_grid_s == pytest.approx(2000.0 / expect_vg, abs=1e-9)


def test_profile_nearest_layer_missing_is_uncovered_not_substituted():
    """最接近的那層（80 m）在這裡缺值 ⇒ 該段沒涵蓋（沿用規劃風、差 0、計入 uncovered_m），不偷換 120 m 那層。"""
    w = [wp(0, 0, 0.0, Phase.OPERATE, agl=90, track=90, strip="S1", d=1),
         wp(2000, 0, 100.0, Phase.OPERATE, agl=90, track=90, strip="S1", d=1), wp(2000, 0, 100.0, Phase.LAND, agl=90)]
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), _profile(bbox_of(w), nan80=True))
    e = est.uavs["B1"]
    r = run_of(est, "S1")
    assert not r.covered and r.grid_uv is None
    assert r.dt_grid_s == pytest.approx(r.dt_plan_s)
    assert e.uncovered_m == pytest.approx(2000.0)
    assert any("不在網格內" in ln for ln in est.summary_lines())


def test_single_height_grid_says_it_is_applied_to_all_heights():
    w = two_strips()
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()),
                            dataclasses.replace(wf.uniform_field(bbox_of(w), 3.0, 0.0), height_m=10.0))
    assert est.source_kind == "grid"
    assert any("10 m 網格套用到所有高度" in n for n in est.notes)


# ─────────────────────────────────────────────────────────────────────
#  涵蓋範圍、等待時間、非計時階段
# ─────────────────────────────────────────────────────────────────────
def test_route_outside_grid_is_not_extrapolated():
    w = two_strips()
    w = w[:2] + [wp(2000, 0, 100.0, Phase.TRANSFER, track=90), wp(20000, 0, 1000.0, Phase.TRANSFER, track=90),
                 wp(20000, 0, 1000.0, Phase.LAND)]
    bb = bbox_of(w[:2], pad=0.005)                    # 只涵蓋第一條帶
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bb, 5.0, 270.0))
    e = est.uavs["B1"]
    tr = next(r for r in e.runs if r.phase == "TRANSFER")
    assert not tr.covered and tr.dt_grid_s == pytest.approx(tr.dt_plan_s)
    assert e.uncovered_m == pytest.approx(18000.0)
    assert run_of(est, "S1").covered


def test_inserted_wait_is_preserved_not_scaled():
    """0 長度的延遲盤旋（deconflict.insert_delay_orbit 的形狀：同點複本、之後時間 +shift）60 s 原樣保留。"""
    w = two_strips(gap=60.0)
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 5.0, 270.0))
    e = est.uavs["B1"]
    assert e.t_land_grid_s - e.t_land_plan_s == pytest.approx(-20.0 + 2000.0 / 15.0 - 100.0, abs=1e-9)
    orbit = next(r for r in e.runs if r.phase == "ORBIT")
    assert orbit.length_m == 0.0 and orbit.dt_grid_s == pytest.approx(60.0)


def test_wait_hidden_in_a_segment_is_counted_not_flagged():
    """延後藏在一段裡（shift_plan 的形狀：幾何不變、時間多 45 s）⇒ 算成等待，不當重建誤差，也照樣保留。"""
    w = two_strips(t_east=100.0 + 45.0)
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 5.0, 270.0))
    e = est.uavs["B1"]
    assert e.wait_s == pytest.approx(45.0)
    assert run_of(est, "S1").dt_grid_s == pytest.approx(80.0 + 45.0)
    assert not any("重建" in p for p in e.problems)


def test_fixed_time_phases_are_untouched():
    """起飛、轉態、加減速、降落是固定常數（build_3d 不用地速算）⇒ 任何網格風都不改。"""
    w = [wp(0, 0, 0.0, Phase.TAKEOFF, va=0.0), wp(0, 0, 20.0, Phase.TRANS_OUT, track=90),
         wp(300, 0, 35.0, Phase.ACCEL, track=90), wp(600, 0, 45.0, Phase.DECEL, track=90),
         wp(900, 0, 55.0, Phase.TRANS_IN, track=90), wp(1000, 0, 70.0, Phase.LAND, va=0.0)]
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 8.0, 90.0))
    e = est.uavs["B1"]
    assert e.runs == () and e.t_land_grid_s == e.t_land_plan_s and e.energy_used_grid_wh == e.energy_used_plan_wh


# ─────────────────────────────────────────────────────────────────────
#  能量與大聲的問題
# ─────────────────────────────────────────────────────────────────────
def test_energy_delta_is_the_planner_energy_model():
    em = model()
    w = two_strips()
    est = gwe.estimate_plan(mplan(uplan(w, margin=100.0, used=150.0)), ctx_of(hv_wind(), em=em),
                            wf.uniform_field(bbox_of(w), 5.0, 270.0))
    e = est.uavs["B1"]
    de = (em.segment_wh(Mode.CRUISE, 80.0, 0.0) - em.segment_wh(Mode.CRUISE, 100.0, 0.0)
          + em.segment_wh(Mode.CRUISE, 2000.0 / 15.0, 0.0) - em.segment_wh(Mode.CRUISE, 100.0, 0.0))
    assert de == pytest.approx(em.k_unc * em.p_cruise_w() * (80.0 + 2000.0 / 15.0 - 200.0) / 3600.0)
    assert e.energy_used_grid_wh == pytest.approx(150.0 + de, abs=1e-9)
    assert e.margin_grid_wh == pytest.approx(100.0 - de, abs=1e-9)


def test_climb_energy_keeps_the_potential_energy_term():
    """爬升段：時間變了、高度差不變 ⇒ 能量差只有 k·P_c·Δt（位能項 m g Δz/η 不隨時間變）。"""
    em = model()
    w = [wp(0, 0, 0.0, Phase.CLIMB, z=50, agl=50, track=90), wp(2000, 0, 100.0, Phase.CLIMB, z=150, agl=150, track=90),
         wp(2000, 0, 100.0, Phase.LAND, z=150, agl=150)]
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind(), em=em), wf.uniform_field(bbox_of(w), 5.0, 270.0))
    e = est.uavs["B1"]
    assert e.energy_used_grid_wh - e.energy_used_plan_wh == pytest.approx(em.k_unc * em.p_cruise_w() * -20.0 / 3600.0,
                                                                           rel=1e-9)


def test_headwind_above_ground_speed_floor_is_loud():
    w = two_strips()
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 18.5, 270.0))
    e = est.uavs["B1"]
    assert any("地速最低 1.5 m/s" in p and "不可行" in p and "S2" in p for p in e.problems)
    assert run_of(est, "S2").dt_grid_s == pytest.approx(2000.0 / PlannerConfig().v_g_min_mps)   # 同 build_3d 以下限計時
    assert any(ln.startswith("⚠") for ln in est.summary_lines())


def test_margin_turning_negative_is_loud():
    w = two_strips()
    est = gwe.estimate_plan(mplan(uplan(w, margin=0.5)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 8.0, 270.0))
    assert est.uavs["B1"].margin_grid_wh < 0
    assert any("能量餘裕變負" in p for p in est.uavs["B1"].problems)


def test_time_shift_beyond_planner_sigma_is_loud_and_small_shift_is_not():
    w = two_strips()
    big = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 8.0, 270.0)).uavs["B1"]
    assert big.max_shift_over_sigma > 1.0 and any("σ_t" in p for p in big.problems)
    small = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 0.1, 270.0)).uavs["B1"]
    assert small.max_shift_over_sigma < 1.0 and small.problems == ()


def test_reconstruction_longer_than_planner_is_loud():
    """規劃器時間比幾何可能的還短（例如縮點後的航點跟時間對不上）⇒ 重建不可信要說出來。"""
    w = two_strips(t_east=90.0)
    e = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 1.0, 270.0)).uavs["B1"]
    assert e.residual_max_s == pytest.approx(10.0)
    assert any("重建" in p and "不可信" in p for p in e.problems)


# ─────────────────────────────────────────────────────────────────────
#  弧段（對獨立的 hv_dccpp.wind.time_arc）
# ─────────────────────────────────────────────────────────────────────
def test_arc_time_matches_hv_time_arc():
    """四分之一圓（R = 100 m，右轉、自北轉到東，切成 15° 一段——規劃器弧段取樣 min(d_d 25 m, RΔψ) 在 R = 100 m 時
    ≈ 14°）：網格風 6 m/s 來自 0° 的時間對 time_arc（中點 Riemann、5°）≤ 0.1%。弧長當弦長（差 0.29%）會被抓到。"""
    R, n, step = 100.0, 6, 15.0
    cx, cy = R, 0.0                                   # 自原點往北出發、右轉 ⇒ 圓心在右側 (R, 0)
    w = []
    t = 0.0
    for k in range(n + 1):
        th = math.radians(step * k)
        x, y = cx - R * math.cos(th), cy + R * math.sin(th)
        w.append(wp(x, y, t, Phase.TRANSFER, track=math.degrees(th), arc=(cx, cy, R, 1)))
        t += R * math.radians(step) / VA
    w[-1] = dataclasses.replace(w[-1], arc=None)
    w.append(wp(w[-1].x_m, w[-1].y_m, w[-1].t_s, Phase.LAND, track=90))
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 6.0, 0.0))
    e = est.uavs["B1"]
    ref = time_arc(np.array([cx, cy]), R, 0.0, math.pi / 2, VA, wind_vector(6.0, 0.0)[0], 100.0)
    # 一段 TRANSFER 只取一個平均風 ⇒ 均勻場下就是 time_arc 的風
    assert e.t_land_grid_s - e.t_takeoff_s == pytest.approx(ref, rel=1e-3)
    # 規劃器時間（R·Δθ/Va）與重建時間一致到數值誤差：弧長若被當成弦長，兩邊模型時間一起縮 ⇒ 差值看不出來，殘差看得出來
    assert e.residual_max_s < 1e-6 and e.residual_abs_median_s < 1e-6


# ─────────────────────────────────────────────────────────────────────
#  真回應格式（錄製檔）、指紋、輸入檢查
# ─────────────────────────────────────────────────────────────────────
def test_recorded_open_meteo_profile_drives_the_estimate():
    """2026-10-19 錄製的 icon_global 2×3 分層回應（23.4–23.6°N、120.30–120.60°E）：條帶在一個格點胞內、離地 115 m ⇒
    用 120 m 層；沿線平均 ＝ 條帶中點的雙線性值（胞內沿等緯線是線性）。"""
    payload = json.loads((FIX / "open_meteo_profile_icon_global_2x3_20261019T06.json").read_text(encoding="utf-8"))
    prof = wf.parse_open_meteo_profile(payload, [23.40, 23.60], [120.30, 120.45, 120.60], when_utc="2026-10-19T06:00")
    w = [wp(-1000, 0, 0.0, Phase.OPERATE, agl=115, track=90, strip="S1", d=1),
         wp(1000, 0, 100.0, Phase.OPERATE, agl=115, track=90, strip="S1", d=1), wp(1000, 0, 100.0, Phase.LAND, agl=115)]
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), prof)
    r = run_of(est, "S1")
    assert est.source_kind == "profile" and r.covered and r.layer_heights_m == (120.0,)
    mid = FRAME.to_wgs84(np.array([0.0, 0.0, 100.0]))[0]
    u, v = wf._field_uv_at(prof.layer(120.0), float(mid[0]), float(mid[1]))
    assert r.grid_uv == pytest.approx((u, v), abs=0.02)
    assert "icon_global" in est.source_label and est.source_time_utc


def test_fingerprint_tells_plans_apart():
    a = mplan(uplan(two_strips()))
    b = mplan(uplan(two_strips(gap=60.0)))
    est = gwe.estimate_plan(a, ctx_of(hv_wind()), wf.uniform_field(bbox_of(a.plans["B1"].waypoints), 3.0, 0.0))
    assert gwe.estimate_matches(est, a) and not gwe.estimate_matches(est, b)


def test_bad_inputs_are_refused_in_words():
    w = two_strips()
    with pytest.raises(gwe.GridWindEstimateError, match="WindField 或 WindProfile"):
        gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), (5.0, 270.0))
    with pytest.raises(gwe.GridWindEstimateError, match="先完成一次 HARVEST 規劃"):
        gwe.estimate_plan(None, ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 1.0, 0.0))
    with pytest.raises(gwe.GridWindEstimateError, match="取樣間距"):
        gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind()), wf.uniform_field(bbox_of(w), 1.0, 0.0), sample_m=0)


def test_summary_and_json_carry_source_and_plan_wind():
    w = two_strips()
    est = gwe.estimate_plan(mplan(uplan(w)), ctx_of(hv_wind(5.0, 270.0)), _profile(bbox_of(w)))
    lines = est.summary_lines()
    assert "合成分層" in lines[0] and "計畫不變" in lines[0]
    assert lines[1].startswith("規劃器用的風：5.00 m/s 來自 270°")
    assert "未重排降落佇列" in lines[2]
    j = est.to_json()
    json.dumps(j, allow_nan=False)
    assert j["source_heights_m"] == [10.0, 80.0, 120.0] and j["uavs"]["B1"]["runs"]


# ─────────────────────────────────────────────────────────────────────
#  真規劃器（契約 mini 情境：A1 Pioneer＋B1 HERO、規劃風 5 m/s 來自 270°）
# ─────────────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def mini():
    from hv_dccpp.context import build_context
    from hv_dccpp.planner import plan_mission
    from hv_dccpp.scenario import build_mini_example
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    mp = plan_mission(inputs, cfg, ctx=ctx)
    assert set(mp.plans) == {"A1", "B1"}
    lats = [w.lat_deg for p in mp.plans.values() for w in p.waypoints]
    lons = [w.lon_deg for p in mp.plans.values() for w in p.waypoints]
    bb = (min(lats) - 0.01, min(lons) - 0.01, max(lats) + 0.01, max(lons) + 0.01)
    return SimpleNamespace(cfg=cfg, ctx=ctx, mp=mp, bb=bb)


def test_real_plan_reconstruction_matches_planner_times(mini):
    """重建的「規劃風下的時間」對規劃器 build_3d 自己的時間：每段最多差 0.1 s（實測 0.02–0.04 s），網格風＝規劃風 ⇒ 差 0。"""
    est = gwe.estimate_plan(mini.mp, mini.ctx, wf.uniform_field(mini.bb, 5.0, 270.0))
    for u, e in est.uavs.items():
        assert e.timed_m > 10_000, u                   # 真的有重算到（不是空轉）
        assert e.residual_max_s < 0.1, (u, e.residual_max_s)
        assert e.uncovered_m == 0.0
        assert e.t_land_grid_s == pytest.approx(e.t_land_plan_s, abs=1e-6)
        assert e.margin_grid_wh == pytest.approx(e.margin_plan_wh, abs=1e-6)
        assert e.problems == ()


@pytest.mark.parametrize("spd,dirn", [(0.0, 0.0), (7.0, 270.0), (3.0, 225.0)])
def test_real_plan_shift_agrees_with_independent_track_simulation(mini, spd, dirn):
    """換成另一個均勻風：重估的落地時刻差 vs L1 追蹤模擬（sim.track_simulate，獨立實作）在兩個風下的差。
    2026-10-21 實測差 −2.7～+5.7 s（Δ 50–90 s）；L1 在轉彎切角，所以門檻 0.1·|Δ| + 6 s。符號錯會差 2|Δ|。"""
    from hv_dccpp.sim import track_simulate
    est = gwe.estimate_plan(mini.mp, mini.ctx, wf.uniform_field(mini.bb, spd, dirn))
    for u, p in mini.mp.plans.items():
        spec = mini.ctx.specs[p.cls]
        m = mini.ctx.models.get(u) or mini.ctx.models_rep[p.cls]
        base = track_simulate(p, spec, mini.ctx.wind, m, mini.cfg, dt_s=0.25)
        alt = track_simulate(p, spec, wind_vector(spd, dirn)[0], m, mini.cfg, dt_s=0.25)
        assert base.completed and alt.completed
        d_sim = alt.t_land_s - base.t_land_s
        d_est = est.uavs[u].t_land_grid_s - est.uavs[u].t_land_plan_s
        assert abs(d_sim) > 20.0, (u, d_sim)          # 這個風真的有差
        assert abs(d_est - d_sim) <= 0.1 * abs(d_sim) + 6.0, (u, d_est, d_sim)
