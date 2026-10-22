"""上傳任務沿線 AGL（core/terrain_clearance.py）與 SwarmCoordinator 掛點。

合成山脊 DEM（hv_dccpp.scenario.synthetic_dem 的 ridge：南北向、脊線 x = 1000 m、高 60 m、寬 400 m、升餘弦剖面）
上手算的離地高鎖住：AGL ＝ home 地面 ＋ 相對高 − 該點地面（脊頂 AGL ＝ 相對高 − 60）、走廊最高地形 vs 中心線、
h_clr 容差 1 m 的邊界與取樣點數、h_minfw 與縮點容差、依結構分段時 DO_ 掛在出發列之後、規劃器階段剖面、DEM 外
不當 0、高程基準抵銷（home 地面取 DEM、不取第 0 列 alt）、NAV_VTOL_LAND 保持高度、盤旋圓周取較低高度、半徑來源、
不支援的框架。真規劃器（契約 mini 情境）鎖住：上傳項目對回縮點（含建圖引入段）、RTB→DESCENT 被縮成一段時
逐點階段不誤報、縮點下探落在容差內。掛點：validate 的 AGL 標籤、notes、GUI 大聲清單、SwarmCoordinator 真的跑。

一律不連網、不寫檔（AEROPLAN_NO_FILE_LOG 由 conftest 設）、不建 MainWindow、不開 SITL／COM。
"""
from __future__ import annotations

import dataclasses
import math
from types import SimpleNamespace

import numpy as np
import pytest

from core import terrain_clearance as tc
from hv_dccpp.geo import GeoFrame, TerrainGrid
from hv_dccpp.scenario import synthetic_dem_for_frame

LAT0, LON0 = 23.50, 120.37
BASE = 100.0
FRAME = GeoFrame(LAT0, LON0, BASE)
RIDGE_X, RIDGE_H, RIDGE_W = 1000.0, 60.0, 400.0
GRID = (-1500.0, -1500.0, 3490.0, 1500.0)          # 東緣 3490：不落在任何 −500 + 25k 的取樣點上

WP, LT_TURNS, LT_TIME, LT_ALT, VTO, VLAND, CAM, LAND_START = 16, 18, 19, 31, 84, 85, 206, 189


def ridge_ground(d_perp: float) -> float:
    """脊線橫向距離 d 處的地面（m MSL）：升餘弦剖面。"""
    hw = 0.5 * RIDGE_W
    return BASE + (RIDGE_H * 0.5 * (1.0 + math.cos(math.pi * d_perp / hw)) if abs(d_perp) <= hw else 0.0)


def terrain(shift: float = 0.0, bounds=GRID) -> TerrainGrid:
    c = FRAME.to_wgs84(np.array([[RIDGE_X, 0.0, BASE]]))[0]
    dem = synthetic_dem_for_frame(FRAME, (-2000.0, -2000.0, 4500.0, 2000.0), 10.0, base_msl_m=BASE + shift,
                                  ridge={"center_latlon": (float(c[0]), float(c[1])), "azimuth_deg": 0.0,
                                         "height_m": RIDGE_H, "width_m": RIDGE_W, "length_m": 6000.0})
    return TerrainGrid.from_dem(dem, FRAME, 10.0, bounds)


TERRAIN = terrain()


def ll(x: float, y: float, h: float = BASE) -> tuple[float, float]:
    w = FRAME.to_wgs84(np.array([[x, y, h]]))[0]
    return float(w[0]), float(w[1])


def it(cmd: int, x: float = 0.0, y: float = 0.0, alt: float = 0.0, *, frame: int = 3, p=(0.0, 0.0, 0.0, 0.0)):
    # 經緯度取在**飛行高度**（坪 100 m ＋ 相對高）：ENU 水平座標隨高度微變（2.5 km 外 130 m 高差 ≈ 0.05 m），
    #   被測函式在飛行高度換回 ENU ⇒ 這樣 3000 m 的段剛好 3000 m、每 25 m 一點會落在脊頂上（手算才精確）
    lat, lon = ll(x, y, BASE + (alt if frame == 3 else 0.0))
    return {"command": cmd, "frame": frame, "lat": lat, "lon": lon, "alt": float(alt),
            "param1": p[0], "param2": p[1], "param3": p[2], "param4": p[3]}


def do(cmd: int, p1: float = 0.0):
    return {"command": cmd, "frame": 3, "lat": 0.0, "lon": 0.0, "alt": 0.0, "param1": p1}


HOME = it(WP, 0.0, 0.0, BASE, frame=0)


def run(rows, *, home=HOME, terr=TERRAIN, hw=0.0, phases=None, slack=0.0, minfw=100.0, **kw):
    rep = tc.uploaded_items_clearance({1: [home] + list(rows)}, terr, FRAME, h_clr_m=36.0, h_minfw_m=minfw,
                                      phases=None if phases is None else {1: phases}, corridor_half_width_m=hw,
                                      minfw_slack_m=slack, **kw)
    return rep, rep.drones[1]


def kinds(d) -> list[tuple[str, str]]:
    return [(p.kind, p.stage) for p in d.problems]


def cross(alt: float, *extra_after_wp1):
    """起飛（坪上 90 m）→ (−500, 0) → 東飛越脊線到 (2500, 0)，全程同高 ``alt``。"""
    return [it(VTO, 0, 0, 90.0), it(WP, -500, 0, alt), *extra_after_wp1, it(WP, 2500, 0, alt)]


# ─────────────────────────────────────────────────────────────────────
#  手算：脊頂與走廊
# ─────────────────────────────────────────────────────────────────────
def test_hand_computed_ridge_crest_agl_and_position():
    rep, d = run(cross(130.0))
    assert rep.ok and d.n_unknown == 0
    assert d.home_ground_msl_m == pytest.approx(BASE, abs=0.05)
    m = d.min_all
    assert m.agl_m == pytest.approx(130.0 - RIDGE_H, abs=0.3)          # 100 + 130 − 160
    lat, lon = ll(RIDGE_X, 0.0)
    assert math.hypot((m.lat_deg - lat) * 111e3, (m.lon_deg - lon) * 101e3) < 5.0
    assert m.stage == tc.STRUCT_ENROUTE and m.seq == 2
    assert d.by_stage[tc.STRUCT_CLIMB_OUT].agl_m == pytest.approx(90.0, abs=0.3)   # 起飛後那段在平地上


def test_corridor_max_vs_centerline_parallel_to_ridge():
    # 平行脊線、偏東 40 m：中心線地面 100 + 30(1 + cos(0.2π)) = 154.27；走廊 ±50 m 涵蓋脊頂 160
    #   （起飛後斜切過脊線那段另算 climb_out；這裡只看平行那段 ＝ 相機開著的 operate）
    rows = [it(VTO, 0, 0, 90.0), it(WP, 1040, -1000, 130.0), do(CAM, 50.0), it(WP, 1040, 1000, 130.0)]
    _rep, d = run(rows, hw=50.0, minfw=50.0)
    op = d.by_stage[tc.STRUCT_OPERATE]
    assert op.agl_centerline_m == pytest.approx(BASE + 130.0 - ridge_ground(40.0), abs=0.3)
    assert op.agl_corridor_m == pytest.approx(130.0 - RIDGE_H, abs=0.3)
    assert op.agl_m == op.agl_corridor_m                                  # 判定基準＝走廊
    _rep, d0 = run(rows, hw=0.0, minfw=50.0)
    assert d0.by_stage[tc.STRUCT_OPERATE].agl_m == pytest.approx(op.agl_centerline_m, abs=0.05)
    assert d0.by_stage[tc.STRUCT_OPERATE].agl_corridor_m is None


def test_h_clr_boundary_tolerance_and_sample_count():
    _rep, d = run(cross(95.5))                    # 脊頂 35.5：在 1 m 容差內
    assert d.problems == [] and d.min_all.agl_m == pytest.approx(35.5, abs=0.3)
    rep, d = run(cross(90.0))                     # 脊頂 30
    assert kinds(d) == [("agl_below_h_clr", tc.STRUCT_ENROUTE)]
    p = d.problems[0]
    assert p.agl_m == pytest.approx(30.0, abs=0.3) and p.limit_m == 36.0
    # AGL < 35 ⇔ 地面 > 155 ⇔ |d| < 37.3 m ⇒ x = 975、1000、1025 三個取樣點
    assert p.n == 3
    assert not rep.ok and rep.problem_notes()[0].startswith("terrain_clearance_failed(")
    assert "h_clr 36" in rep.problem_notes()[0]


# ─────────────────────────────────────────────────────────────────────
#  h_minfw：依結構分段、DO_ 掛在出發列之後、縮點容差
# ─────────────────────────────────────────────────────────────────────
def test_minfw_structural_only_on_camera_legs():
    _rep, d = run(cross(150.0))                                  # 沒開相機：脊頂 90 但不檢查 h_minfw
    assert d.problems == []
    _rep, d = run(cross(150.0, do(CAM, 50.0)))                   # 206 掛在 (−500,0) 之後 ⇒ 飛越脊線那段是 operate
    assert kinds(d) == [("agl_below_h_minfw", tc.STRUCT_OPERATE)]
    assert d.problems[0].agl_m == pytest.approx(90.0, abs=0.3)


def test_do_items_describe_the_departure_leg():
    # 206 掛在 (2500,0) 之後：飛越脊線那段相機還沒開 ⇒ 不檢查 h_minfw
    rows = cross(150.0) + [do(CAM, 50.0), it(WP, 2500, 500, 150.0)]
    _rep, d = run(rows)
    assert d.problems == []
    assert d.by_stage[tc.STRUCT_ENROUTE].agl_m == pytest.approx(90.0, abs=0.3)
    assert d.by_stage[tc.STRUCT_OPERATE].agl_m == pytest.approx(150.0, abs=0.3)
    # DO_LAND_START 之後 ⇒ landing
    _rep, d = run(cross(150.0, do(LAND_START)))
    assert tc.STRUCT_LANDING in d.by_stage and d.by_stage[tc.STRUCT_LANDING].agl_m == pytest.approx(90.0, abs=0.3)


def test_minfw_slack_two_tiers():
    rows = cross(150.0, do(CAM, 50.0))                           # 脊頂 90：低 10、在 1 + 10 m 內
    rep, d = run(rows, slack=10.0)
    assert d.problems == [] and [p.kind for p in d.within_slack] == ["agl_below_h_minfw_within_slack"]
    assert "縮點容差 10 m 內" in rep.summary_note() and rep.problem_notes() == []
    rep, d = run(cross(140.0, do(CAM, 50.0)), slack=10.0)        # 脊頂 80：超過容差
    assert kinds(d) == [("agl_below_h_minfw", tc.STRUCT_OPERATE)] and d.within_slack == []
    # h_clr 不給縮點容差
    _rep, d = run(cross(90.0), slack=10.0)
    assert kinds(d) == [("agl_below_h_clr", tc.STRUCT_ENROUTE)]


def test_loud_stage_not_repeated_within_slack():
    # 同一階段：脊頂 75（超過容差）＋脊旁 ~92（容差內）⇒ 只列一筆問題，不另列容差內
    _rep, d = run(cross(135.0, do(CAM, 50.0)), slack=10.0)
    assert kinds(d) == [("agl_below_h_minfw", tc.STRUCT_OPERATE)] and d.within_slack == []


# ─────────────────────────────────────────────────────────────────────
#  規劃器階段剖面
# ─────────────────────────────────────────────────────────────────────
@pytest.mark.parametrize("brk, flagged", [(0.6, True), (0.4, False)])
def test_leg_phase_profile_splits_rtb_and_descent(brk, flagged):
    rows = cross(150.0)                                          # 脊頂在出發段比例 0.5
    phases = [None, "TRANS_OUT", tc.LegPhases((0.0, brk), ("RTB", "DESCENT")), "DECEL"]
    _rep, d = run(rows, phases=phases)
    assert d.phase_source == "plan"
    assert kinds(d) == ([("agl_below_h_minfw", "RTB")] if flagged else [])
    assert set(d.by_stage) == {"TRANS_OUT", "RTB", "DESCENT"}


def test_leg_phases_at():
    lp = tc.LegPhases((0.0, 0.25, 0.75), ("CLIMB", "ENTRY", "OPERATE"))
    assert lp.at(np.array([0.0, 0.2, 0.25, 0.5, 0.75, 1.0])).tolist() == \
        ["CLIMB", "CLIMB", "ENTRY", "ENTRY", "OPERATE", "OPERATE"]


# ─────────────────────────────────────────────────────────────────────
#  DEM 外、高程基準、VTOL 降落、盤旋、框架
# ─────────────────────────────────────────────────────────────────────
def test_outside_dem_is_unknown_not_zero():
    rows = [it(VTO, 0, 0, 90.0), it(WP, -500, 0, 130.0), it(WP, 4000, 0, 130.0)]
    rep, d = run(rows, hw=50.0)
    # (−500 → 4000) 每 25 m：網格東緣 3490 ⇒ x ≥ 3500 的取樣點 k = 160..180 ⇒ 21 個
    assert d.n_unknown == 21
    assert ("terrain_unknown", tc.STRUCT_ENROUTE) in kinds(d)
    assert next(p for p in d.problems if p.kind == "terrain_unknown").n == 21
    assert d.min_all.agl_m == pytest.approx(70.0, abs=0.3)       # 最小值只看已知地形（不是 0 也不是 nan）
    assert "DEM 外" in rep.problem_notes()[0]
    # 只看中心線（沒有走廊的 nan 幫忙）也一樣
    _rep, d0 = run(rows, hw=0.0)
    assert d0.n_unknown == 21 and [p.kind for p in d0.problems] == ["terrain_unknown"]
    assert d0.min_all.agl_m == pytest.approx(70.0, abs=0.3)


def test_corridor_outside_dem_is_unknown():
    # 沿網格北緣（y = 1500）內 30 m 東飛：中心線在 DEM 內，±50 m 走廊伸出去 ⇒ 走廊判定時每個取樣點都未知
    rows = [it(VTO, 0, 0, 90.0), it(WP, -500, 1470, 130.0), it(WP, 0, 1470, 130.0)]
    _rep, d = run(rows, hw=0.0)
    assert d.n_unknown == 0
    _rep, d = run(rows, hw=50.0)
    assert d.n_unknown == 21                                     # 該段 (−500 → 0) 每 25 m 的 21 個點
    assert next(p for p in d.problems if p.kind == "terrain_unknown").n == 21


def test_home_outside_dem():
    home = it(WP, 4000.0, 0.0, BASE, frame=0)
    _rep, d = run([it(VTO, 4000, 0, 90.0), it(WP, 2000, 0, 130.0)], home=home)
    assert [p.kind for p in d.problems] == ["home_outside_dem"] and d.n_samples == 0


def test_datum_cancels_and_home_ground_comes_from_dem():
    # 整份 DEM 抬高 20 m（像 World Terrain 的橢球高、N 未知），第 0 列 alt 給錯：AGL 不變
    shifted = terrain(shift=20.0)
    bad_home = dict(HOME, alt=0.0)
    _rep, d0 = run(cross(130.0))
    rep, d1 = run(cross(130.0), home=bad_home, terr=shifted)
    assert d1.home_ground_msl_m == pytest.approx(BASE + 20.0, abs=0.05)
    assert d1.min_all.agl_m == pytest.approx(d0.min_all.agl_m, abs=1e-3)
    assert any("第 0 列 home alt 0.0 m 與 DEM 坪地面 120.0 m" in c for c in d1.caveats)
    assert "第 0 列 home alt" in rep.summary_note()


def test_vtol_land_approach_holds_altitude():
    # 最後一個導航點 130 m → NAV_VTOL_LAND（alt 0）在 (2500,0)：先保持 130 m 飛到落點上空 ⇒ 脊頂 70，不是 5
    rows = [it(VTO, 0, 0, 90.0), it(WP, -500, 0, 130.0), dict(it(VLAND, 2500, 0, 130.0), alt=0.0)]
    _rep, d = run(rows)
    assert d.problems == []
    assert d.min_all.agl_m == pytest.approx(70.0, abs=0.3)


def test_loiter_circle_at_lower_altitude():
    # LOITER_TO_ALT 圓心在脊頂、R 200（param2）、指令 160、抵達 130 ⇒ 圓周取 130：圓上 (1000, ±200) 在脊線上 ⇒ ≈ 70
    rows = [it(VTO, 0, 0, 90.0), it(WP, -500, 0, 130.0), it(LT_ALT, RIDGE_X, 0, 160.0, p=(0.0, 200.0, 0.0, 1.0)),
            it(WP, 2500, 0, 160.0)]
    _rep, d = run(rows)
    lo = d.by_stage[tc.STRUCT_LOITER]
    assert lo.agl_m == pytest.approx(70.0, abs=0.5) and lo.seq == 3
    assert d.by_stage[tc.STRUCT_ENROUTE].agl_m > 125.0           # 進入段停在圓上 (800, 0)，沒有飛到脊頂
    # LOITER_TURNS 的半徑在 param3
    rows[2] = it(LT_TURNS, RIDGE_X, 0, 130.0, p=(1.0, 0.0, -200.0, 1.0))
    _rep, d = run(rows)
    assert d.by_stage[tc.STRUCT_LOITER].agl_m == pytest.approx(70.0, abs=0.5)
    # 半徑 0 ⇒ 只取圓心（脊頂 70）並列 caveats；給 WP_LOITER_RAD ⇒ 用它
    rows[2] = it(LT_TIME, RIDGE_X + 200.0, 0, 130.0, p=(30.0, 0.0, 0.0, 1.0))
    _rep, d = run(rows)
    assert any("盤旋半徑未知" in c for c in d.caveats)
    assert d.by_stage[tc.STRUCT_LOITER].agl_m == pytest.approx(130.0, abs=0.3)   # 圓心 (1200,0) 在平地
    _rep, d = run(rows, loiter_radius_m=200.0)
    assert d.caveats == [] and d.by_stage[tc.STRUCT_LOITER].agl_m == pytest.approx(70.0, abs=0.5)


def test_unsupported_frame_and_bad_input():
    _rep, d = run([it(VTO, 0, 0, 90.0), it(WP, 500, 0, 230.0, frame=0)])
    assert [p.kind for p in d.problems] == ["unsupported_frame"] and d.n_samples == 0
    with pytest.raises(tc.TerrainClearanceError):
        run([{"command": WP, "frame": 3}])
    with pytest.raises(tc.TerrainClearanceError):
        run(cross(130.0), sample_m=0.0)
    with pytest.raises(tc.TerrainClearanceError):
        tc.uploaded_items_clearance({1: []}, TERRAIN, FRAME, h_clr_m=36.0, h_minfw_m=100.0)


def test_mission_item_objects_are_accepted():
    from hv_dccpp.ardupilot import MissionItem
    rows = [HOME] + cross(90.0)
    objs = [MissionItem(i, r["frame"], r["command"], 0, 1, r.get("param1", 0.0), r.get("param2", 0.0),
                        r.get("param3", 0.0), r.get("param4", 0.0), r["lat"], r["lon"], r["alt"])
            for i, r in enumerate(rows)]
    rep = tc.uploaded_items_clearance({7: objs}, TERRAIN, FRAME, h_clr_m=36.0, h_minfw_m=100.0)
    assert kinds(rep.drones[7]) == [("agl_below_h_clr", tc.STRUCT_ENROUTE)]


# ─────────────────────────────────────────────────────────────────────
#  真規劃器（契約 mini 情境：A1 Pioneer＋B1 HERO、平地 100 m）
# ─────────────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def mini():
    from hv_dccpp.ardupilot import CameraTriggerConfig, plan_to_mission_items
    from hv_dccpp.context import build_context
    from hv_dccpp.integration.aeroplan_adapter import mission_item_dicts
    from hv_dccpp.planner import plan_mission
    from hv_dccpp.scenario import build_mini_example
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    mp = plan_mission(inputs, cfg, ctx=ctx)
    assert set(mp.plans) == {"A1", "B1"}

    def items(uid, lead=0.0):
        return mission_item_dicts(plan_to_mission_items(mp.plans[uid], ctx, cfg, camera_trigger=True,
                                                        camera_cfg=CameraTriggerConfig(camera_lead_in_m=lead)))
    return SimpleNamespace(cfg=cfg, ctx=ctx, mp=mp, items=items)


def _rep(mini, missions, phases, slack=0.0):
    return tc.uploaded_items_clearance(
        missions, mini.ctx.terrain.raw_view(), mini.ctx.frame, h_clr_m=mini.ctx.h_clr_m,
        h_minfw_m={u: mini.ctx.specs[mini.mp.plans[u].cls].min_fw_operation_agl_m for u in missions},
        phases=phases, corridor_half_width_m=mini.cfg.b_trk_m, minfw_slack_m=slack)


def test_real_plan_alignment_and_rtb_descent_profile(mini):
    missions = {u: mini.items(u) for u in mini.mp.plans}
    phases = {u: tc.nav_phases_from_plan(mini.mp.plans[u], mini.ctx, mini.cfg, missions[u]) for u in missions}
    assert all(p is not None for p in phases.values())
    names = {n for lp in phases["A1"] if lp is not None for n in lp.names}
    assert {"TRANS_OUT", "CLIMB", "OPERATE", "RTB"} <= names
    rep = _rep(mini, missions, phases)
    # A1：縮點把 RTB 末段（密集 ≥ 134 m）與 DESCENT 併成一段；整段標出發點的 RTB 會誤報 100.9 m < 120
    assert rep.drones["A1"].problems == []
    assert rep.drones["A1"].by_stage["RTB"].agl_m > 125.0
    # B1：密集 RTB 剛好在 h_minfw 100，縮點（高度剖面容差 10 m）讓上傳那份提早下降 ⇒ 真的低於 h_minfw，但在容差內
    b = rep.drones["B1"]
    assert kinds(b) == [("agl_below_h_minfw", "RTB")]
    assert 100.0 - mini.cfg.delta_prof_m - 1.0 <= b.problems[0].agl_m < 99.0
    rep10 = _rep(mini, missions, phases, slack=float(mini.cfg.delta_prof_m))
    assert rep10.ok and [p.stage for p in rep10.drones["B1"].within_slack] == ["RTB"]


def test_real_plan_uploaded_vs_dense_within_profile_tolerance(mini):
    missions = {u: mini.items(u) for u in mini.mp.plans}
    phases = {u: tc.nav_phases_from_plan(mini.mp.plans[u], mini.ctx, mini.cfg, missions[u]) for u in missions}
    rep = _rep(mini, missions, phases)
    nat = mini.ctx.terrain.raw_view()
    for u, up in mini.mp.plans.items():
        dense: dict[str, float] = {}
        for w in up.waypoints:
            if w.phase.name in tc.VERTICAL_STAGES:
                continue
            g = float(nat.z_at(np.array([[w.x_m, w.y_m]]))[0])
            dense[w.phase.name] = min(dense.get(w.phase.name, math.inf), w.z_m - g)
        d = rep.drones[u]
        assert d.phase_source == "plan" and d.n_unknown == 0
        for st in tc.MINFW_PHASES & set(dense):
            assert d.by_stage[st].agl_centerline_m >= dense[st] - mini.cfg.delta_prof_m - 1.0, (u, st)


def test_real_plan_mapping_lead_in_aligns(mini):
    base = mini.items("B1")
    led = mini.items("B1", lead=50.0)
    n_nav = lambda rows: sum(1 for r in rows if r["command"] == WP)      # noqa: E731
    assert n_nav(led) > n_nav(base)                                       # 有插引入段
    ph = tc.nav_phases_from_plan(mini.mp.plans["B1"], mini.ctx, mini.cfg, led)
    assert ph is not None
    extra = [j for j, r in enumerate(led) if r["command"] == WP and r["param2"] == 1.0 and ph[j] is not None
             and ph[j].names == ("OPERATE",)]
    assert extra


def test_real_plan_alignment_failure_falls_back_loudly(mini):
    items = mini.items("A1")
    k = next(j for j, r in enumerate(items) if j > 3 and r["command"] == WP)
    items[k] = dict(items[k], lat=items[k]["lat"] + 1e-4)                # 偏 ~11 m：對不回縮點
    assert tc.nav_phases_from_plan(mini.mp.plans["A1"], mini.ctx, mini.cfg, items) is None
    rep = _rep(mini, {"A1": items}, {"A1": None})
    d = rep.drones["A1"]
    assert d.phase_source == "structure" and any("對不回規劃器縮點" in c for c in d.caveats)
    assert "對不回規劃器縮點" in rep.summary_note()


# ─────────────────────────────────────────────────────────────────────
#  掛點（mission/swarm_coordinator.py）與 GUI 大聲清單
# ─────────────────────────────────────────────────────────────────────
def test_validate_fail_labels_name_agl():
    from mission.swarm_coordinator import hv_validate_fail_labels
    v = SimpleNamespace(coverage_ok=True, energy_ok=True, conflicts=[], agl_ok=False,
                        details={"A1": ["agl_below_h_minfw: wp=3 RTB agl=92.0 < 100.0",
                                        "agl_below_h_minfw: ... (+3 more, total n=8)"],
                                 "B1": ["agl_below_h_clr: wp=9 OPERATE agl=20.0 < 36.0"],
                                 "coverage": ["1: A=1.0000 required=('A',) PASS"]})
    assert hv_validate_fail_labels(v) == ["agl(agl_below_h_clr、agl_below_h_minfw)"]
    old = SimpleNamespace(coverage_ok=True, energy_ok=False, conflicts=[1, 2, 3])        # 舊寫法逐字不變
    assert hv_validate_fail_labels(old) == ["energy", "conflicts=3"]
    other = SimpleNamespace(coverage_ok=True, energy_ok=True, conflicts=[], slope_ok=False, details={})
    assert hv_validate_fail_labels(other) == ["slope"]


def test_terrain_clearance_notes_helper(mini, monkeypatch):
    from mission.swarm_coordinator import hv_terrain_clearance_notes
    ids = {1: "A1", 2: "B1"}
    calls = []

    def mission_items(did, *, camera_cfg):
        calls.append(camera_cfg)
        return mini.items(ids[did])
    hv = SimpleNamespace(last_ctx=mini.ctx, last_plan=mini.mp, cfg=mini.cfg, id_map=ids, camera_cfg=None,
                         mission_items=mission_items)
    rep, notes = hv_terrain_clearance_notes(hv, "平地 DEM")
    assert isinstance(rep, tc.ClearanceReport) and len(notes) == 1 and len(calls) == 2
    assert notes[0].startswith("terrain_clearance(") and "地形 平地 DEM" in notes[0]
    assert rep.minfw_slack_m == mini.cfg.delta_prof_m and rep.corridor_half_width_m == mini.cfg.b_trk_m
    assert rep.h_minfw_m == {1: 120.0, 2: 100.0} and "UAV-2 RTB" in notes[0]          # B1 縮點下探在容差內
    # 有問題 ⇒ 摘要之後每架一則 terrain_clearance_failed
    fake = SimpleNamespace(summary_note=lambda: "terrain_clearance(x)", problem_notes=lambda: ["terrain_clearance_failed(y)"])
    monkeypatch.setattr(tc, "hv_uploaded_clearance", lambda hv, **kw: fake)
    assert hv_terrain_clearance_notes(hv)[1] == ["terrain_clearance(x)", "terrain_clearance_failed(y)"]
    # 檢查本身壞掉 ⇒ 大聲、不當驗過
    monkeypatch.setattr(tc, "hv_uploaded_clearance", lambda hv, **kw: (_ for _ in ()).throw(RuntimeError("boom")))
    rep, notes = hv_terrain_clearance_notes(hv)
    assert rep is None and len(notes) == 1
    assert notes[0].startswith("terrain_clearance_failed(") and "沒跑成功" in notes[0] and "boom" in notes[0]


def test_gui_loud_prefix():
    import ui.main_window as mw
    loud = mw.MainWindow._HV_LOUD_NOTES
    assert "terrain_clearance_failed(UAV-1 …)".startswith(loud)
    assert not "terrain_clearance(上傳任務沿線 AGL …)".startswith(loud)     # 摘要只寫 log


def _plan_chiayi(monkeypatch=None, wrap=None):
    """SwarmCoordinator 真的規劃一次（嘉義場地幾何、2 HERO、沒有 DEM ⇒ 平地）。回 (協調器, 每則 report)。

    關掉回收站自動指派：這個幾何下它會送一則 recovery_auto_rejected，讓尾端 report 本來就存在；關掉後「沒有別的話
    要說」⇒ 轉接層那份必須是最後一份（swarm_coordinator 的 A6／M2）。``wrap`` 包住掛點的 helper（測 notes 有送到）。"""
    import mission.swarm_coordinator as SC
    from config.spawn_presets import get_preset_by_key
    from utils.math_utils import spawn_positions
    if wrap is not None:
        real = SC.hv_terrain_clearance_notes
        monkeypatch.setattr(SC, "hv_terrain_clearance_notes", lambda hv, label="": wrap(*real(hv, label)))
    p = get_preset_by_key("CHIAYI_AUIRC")
    m_lat, m_lon = 111132.0, 111412.0 * math.cos(math.radians(p.lat))

    def at(e, n):
        return (p.lat + n / m_lat, p.lon + e / m_lon)
    pts = spawn_positions(p.lat, p.lon, 2, 0.0, 10.0)
    drones = [{"drone_id": i + 1, "name": f"B{i + 1}", "hv_class": "B", "position": pts[i], "speed": 19.0,
               "heading": 0.0, "turn_radius": 120.0, "vehicle_type": "vtol", "fov_width": 150.0} for i in range(2)]
    reports: list[dict] = []
    c = SC.SwarmCoordinator()
    c.use_harvest = True
    c.hv_uniform_sensor = True
    c.hv_recovery_auto = False
    c.hv_class_of = lambda d: "B"
    c.hv_home = (p.lat, p.lon)
    c.hv_alternates = [at(0.0, 2500.0)]
    c.hv_wind = (3.0, 270.0)
    c.hv_fence_zones = []
    c.hv_on_report = reports.append
    c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0, "hv_waiver_max_agl_m": 200.0,
                        "hv_dem_margin_m": 3000.0, "hv_terrain_clearance_m": 36.0,
                        "hv_min_fw_agl_m": {"B": 100.0}, "target_gsd": 0.05}
    c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0,
                                  "polygon": [at(-500, -300), at(500, -300), at(500, 300), at(-500, 300)]}],
                          drones=drones, coverage_width_m=150.0, overlap_rate=0.30, coverage_altitude=120.0,
                          mounting_angle_deg=0.0)
    return c, reports


def test_coordinator_hook_runs_on_a_real_plan():
    """validate 之後掛點有跑、報告留在協調器上、地形標籤寫明平地；摘要只是資訊 ⇒ 沒有別的話要說時**不**為它多送一份
    尾端 report（多送會讓轉接層那份不再是最後一份，蓋掉 uncovered／status 與 terrain_datum：A6／M2）。"""
    c, reports = _plan_chiayi()
    rep = c.hv_terrain_clearance
    assert isinstance(rep, tc.ClearanceReport) and set(rep.drones) == set(c.hv_coordinator.id_map)
    assert rep.ok and all(d.phase_source == "plan" for d in rep.drones.values())
    assert "平地 DEM（沒有地形資料：只對平地驗）" in rep.summary_note()
    assert all("validation_notes" not in r for r in reports), [list(r) for r in reports]
    assert "uncovered" in reports[-1] and "status" in reports[-1]


def test_coordinator_hook_sends_failures_and_summary_rides_along(monkeypatch):
    """有違規 ⇒ terrain_clearance_failed 一定送到 GUI（validation_notes），摘要搭同一則尾端 report、只出現一次。"""
    c, reports = _plan_chiayi(monkeypatch, wrap=lambda rep, notes: (rep, notes + ["terrain_clearance_failed(假)"]))
    vn = [n for r in reports for n in (r.get("validation_notes") or [])]
    assert "terrain_clearance_failed(假)" in vn
    summ = [n for n in vn if n.startswith("terrain_clearance(")]
    assert len(summ) == 1 and "平地 DEM（沒有地形資料：只對平地驗）" in summ[0]
    assert isinstance(c.hv_terrain_clearance, tc.ClearanceReport)
