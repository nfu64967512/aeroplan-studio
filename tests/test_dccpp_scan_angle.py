# -*- coding: utf-8 -*-
"""手動掃描角的值層級守衛。

背景（2026-08-25）：面板加了「掃描角」控制項、`main_window` 也把
`scan_angle_deg` 塞進 areas、log 甚至印出「手動掃描角 47.0°」——**但畫面完全沒變**。

真因：`plan_coverage_dccpp` 裡有**兩條**管線。
  * legacy：`RegionDivider.decompose_by_fov(polygon, scan_angle, ...)` → 有吃角度，
    但它產生的 `coverage_paths` **不是飛出來的東西**。
  * DCCPP v2 直通管線（AreaProcessor → Allocator → IDP_Solver → PathBuilder）：
    才是真正產生航點的那條，而它呼叫 `ap.process_latlon(polygon, w_i, w_ovl)`
    ——**沒有角度參數**，方向恆由 `find_min_width` 的幾何最小寬度決定。

所以「參數有送到」不等於「參數有生效」。這裡的守衛全部鎖在**輸出值**上
（條帶數、航向、航點數、makespan），不鎖任何一行長什麼樣：改上游、改下游、
換實作都免疫不了。參見 [[value-locks-beat-source-locks]]。
"""

import math

import pytest

from core.dccpp.area_processor import (
    AreaProcessor,
    convex_hull,
    find_min_width,
    width_along_direction,
)
from mission.swarm_coordinator import SwarmCoordinator


# 長寬明顯不同的矩形：最小寬度方向唯一，換角度一定看得出差別。
# 約 555 m (南北) x 1016 m (東西) @ lat 24。
AREA_POLY = [
    (24.000, 121.000),
    (24.000, 121.010),
    (24.005, 121.010),
    (24.005, 121.000),
]
W_I = 110.0
W_OVL = 11.0


def _hull_xy():
    ap = AreaProcessor()
    return ap.process_latlon(AREA_POLY, W_I, W_OVL).convex_hull_xy


# ══════════════════════════════════════════════════════════════
# A. width_along_direction 的契約
# ══════════════════════════════════════════════════════════════

@pytest.mark.parametrize("ang", [0.0, 30.0, 47.0, 90.0, 137.0, 215.0, 359.0])
def test_scan_lines_fly_the_requested_heading(ang):
    """每一條掃描線的實際航向必須**就是**使用者指定的角度。

    這是使用者唯一看得懂的驗收：設 47° 就要飛 47°。
    """
    ap = AreaProcessor()
    res = ap.process_latlon(AREA_POLY, W_I, W_OVL, scan_angle_deg=ang)
    assert res.scan_lines, "指定角度後不該一條掃描線都生不出來"
    for sl in res.scan_lines:
        # L→R 或 R→L 都算同一條線的方向，取 mod 180 比較
        assert sl.heading_compass_deg % 180.0 == pytest.approx(ang % 180.0, abs=1e-6)
    assert res.min_width_result.alpha_k_deg == pytest.approx(ang % 360.0, abs=1e-9)


def test_direction_vectors_are_orthonormal():
    mwr = width_along_direction(_hull_xy(), 47.0)
    ub, nb = mwr.ub_min, mwr.nb_min
    assert math.hypot(*ub) == pytest.approx(1.0, abs=1e-12)
    assert math.hypot(*nb) == pytest.approx(1.0, abs=1e-12)
    assert ub[0] * nb[0] + ub[1] * nb[1] == pytest.approx(0.0, abs=1e-12)


def test_width_is_the_projection_span_not_the_minimum():
    """指定方向的寬度 = 多邊形在該法向上的投影跨距。

    ⚠ 不是 `find_min_width` 的最小值——那是自動模式。用最小值會讓條帶數
    恆等於自動模式，也就是**角度靜默失效**（正是本次要修的 bug）。
    """
    hull = _hull_xy()
    for ang in (0.0, 47.0, 137.0):
        mwr = width_along_direction(hull, ang)
        proj = [x * mwr.nb_min[0] + y * mwr.nb_min[1] for x, y in hull]
        assert mwr.min_width == pytest.approx(max(proj) - min(proj), abs=1e-9)


def test_opposite_heading_gives_the_same_line_family():
    """θ 與 θ+180 是同一組平行線，只是掃描順序相反。"""
    ap = AreaProcessor()
    a = ap.process_latlon(AREA_POLY, W_I, W_OVL, scan_angle_deg=47.0)
    b = ap.process_latlon(AREA_POLY, W_I, W_OVL, scan_angle_deg=227.0)
    assert len(a.scan_lines) == len(b.scan_lines)
    assert a.min_width_result.min_width == pytest.approx(
        b.min_width_result.min_width, abs=1e-6)


# ══════════════════════════════════════════════════════════════
# B. process_latlon：預設不變 + 角度真的改變條帶
# ══════════════════════════════════════════════════════════════

def test_default_none_is_byte_identical_to_auto():
    """不給角度時行為必須跟原本一模一樣（沒有偷偷改掉自動模式）。"""
    ap = AreaProcessor()
    res = ap.process_latlon(AREA_POLY, W_I, W_OVL)
    hull = res.convex_hull_xy
    expected = find_min_width(convex_hull(list(hull)) if len(hull) < 3 else hull)
    assert res.min_width_result.min_width == pytest.approx(expected.min_width, abs=1e-9)
    assert res.min_width_result.alpha_k_deg == pytest.approx(
        expected.alpha_k_deg, abs=1e-9)


def test_angle_changes_the_strip_count():
    """換角度必須換掃描線數量——這是「有沒有生效」最直接的量。"""
    ap = AreaProcessor()
    auto_n = len(ap.process_latlon(AREA_POLY, W_I, W_OVL).scan_lines)
    n0 = len(ap.process_latlon(AREA_POLY, W_I, W_OVL, scan_angle_deg=0.0).scan_lines)
    n47 = len(ap.process_latlon(AREA_POLY, W_I, W_OVL, scan_angle_deg=47.0).scan_lines)
    # 自動 = 最小寬度方向 ⇒ 條帶數必為最少
    assert auto_n < n0, f"0° 應比自動(最小寬度)多條帶，實際 auto={auto_n} n0={n0}"
    assert auto_n < n47, f"47° 應比自動多條帶，實際 auto={auto_n} n47={n47}"


def test_manual_at_auto_angle_reproduces_auto_exactly():
    """把手動角設成自動算出來的值，必須**逐點**重現自動的結果。

    這條同時擋住兩種錯：角度沒生效（會過但上面那條會掛），
    以及角度慣例接錯（0=北/順時針 vs 數學角），後者只有這條抓得到。
    """
    ap = AreaProcessor()
    auto = ap.process_latlon(AREA_POLY, W_I, W_OVL)
    alpha = auto.min_width_result.alpha_k_deg
    man = ap.process_latlon(AREA_POLY, W_I, W_OVL, scan_angle_deg=alpha)

    assert len(man.scan_lines) == len(auto.scan_lines)
    assert man.min_width_result.min_width == pytest.approx(
        auto.min_width_result.min_width, abs=1e-6)
    for m, a in zip(man.scan_lines, auto.scan_lines):
        assert m.pL_xy[0] == pytest.approx(a.pL_xy[0], abs=1e-6)
        assert m.pL_xy[1] == pytest.approx(a.pL_xy[1], abs=1e-6)
        assert m.pR_xy[0] == pytest.approx(a.pR_xy[0], abs=1e-6)
        assert m.pR_xy[1] == pytest.approx(a.pR_xy[1], abs=1e-6)


# ══════════════════════════════════════════════════════════════
# C. 端到端：真正飛出來的航點會不會變
# ══════════════════════════════════════════════════════════════

def _plan(auto_scan_angle: bool, scan_angle_deg: float):
    drones = [
        {
            'drone_id': i + 1,
            'name': f'UAV-{i + 1}',
            'position': (23.998, 121.0 + i * 0.0005),
            'speed': 22.0,
            'heading': 0.0,
            'turn_radius': 60.0,
            'vehicle_type': 'vtol',
            'fov_width': W_I,
        }
        for i in range(2)
    ]
    res = SwarmCoordinator().plan_coverage_dccpp(
        areas=[{
            'area_id': 1,
            'polygon': list(AREA_POLY),
            'priority': 1.0,
            'scan_angle_deg': scan_angle_deg,
        }],
        drones=drones,
        coverage_width_m=W_I,
        overlap_rate=0.1,
        auto_scan_angle=auto_scan_angle,
        coverage_altitude=120.0,
        vtol_transition_alt_m=50.0,
    )
    built = res['assembled_paths']
    total_wp = sum(len(p.waypoints) for p in built.values())
    return total_wp, float(res['total_makespan'])


def test_manual_angle_changes_the_flown_waypoints():
    """⚠⚠⚠ 本次 bug 的正主守衛。

    以前 DCCPP v2 管線不吃角度，`auto` 與 `手動 0°` 會產生**完全一樣**的航點
    （實測 42 航點 / makespan 259.1 兩邊相同），而 log 還照印「手動掃描角 0°」。
    """
    auto_wp, auto_mk = _plan(True, 0.0)
    man_wp, man_mk = _plan(False, 0.0)
    assert (man_wp, man_mk) != (auto_wp, auto_mk), (
        f"手動角度沒有改變飛出來的路徑：auto=({auto_wp}, {auto_mk:.1f}) "
        f"manual=({man_wp}, {man_mk:.1f})"
    )
    assert man_wp > auto_wp, "非最小寬度方向的航點數應該更多"


def test_end_to_end_manual_at_auto_angle_matches_auto():
    """端到端的身分測試：手動填入自動角 ⇒ 航點數與 makespan 都要一致。"""
    ap = AreaProcessor()
    alpha = ap.process_latlon(AREA_POLY, W_I, W_OVL).min_width_result.alpha_k_deg
    auto_wp, auto_mk = _plan(True, 0.0)
    man_wp, man_mk = _plan(False, alpha)
    assert man_wp == auto_wp
    assert man_mk == pytest.approx(auto_mk, rel=1e-9)


@pytest.mark.parametrize("ang", [0.0, 47.0, 137.0])
def test_every_manual_angle_is_distinguishable_from_auto(ang):
    """三個非最佳角度都必須各自和自動模式不同（不是只有某一個角度會動）。"""
    auto = _plan(True, 0.0)
    assert _plan(False, ang) != auto


# ══════════════════════════════════════════════════════════════
# D. 規劃器要**回報**它用了幾度（顯示值 = 飛的值）
# ══════════════════════════════════════════════════════════════

def _plan_raw(auto_scan_angle: bool, scan_angle_deg: float):
    drones = [{
        'drone_id': 1, 'name': 'UAV-1', 'position': (23.998, 121.0),
        'speed': 22.0, 'heading': 0.0, 'turn_radius': 60.0,
        'vehicle_type': 'vtol', 'fov_width': W_I,
    }]
    return SwarmCoordinator().plan_coverage_dccpp(
        areas=[{'area_id': 1, 'polygon': list(AREA_POLY), 'priority': 1.0,
                'scan_angle_deg': scan_angle_deg}],
        drones=drones, coverage_width_m=W_I, overlap_rate=0.1,
        auto_scan_angle=auto_scan_angle, coverage_altitude=120.0,
        vtol_transition_alt_m=50.0,
    )


@pytest.mark.parametrize("ang", [0.0, 47.0, 137.0, -90.0, -33.5])
def test_planner_reports_the_angle_it_was_given(ang):
    """手動模式下回報值必須**就是**面板送進去的角度（含負角）。

    面板的數字框是 [-180, 180]，規劃器內部是 compass 0..360——這條擋住
    「顯示 47° 但其實飛 -47°」這種只有使用者看得出來的錯。
    """
    eff = _plan_raw(False, ang).get('effective_scan_angles')
    assert eff is not None, "v2 管線必須回報實際生效角"
    assert eff[1] == pytest.approx(ang % 360.0, abs=1e-9)


def test_reported_auto_angle_round_trips_as_manual():
    """把自動回報的角度原封不動當手動送回去，必須得到一模一樣的航線。

    這正是面板「取消自動勾選 → 用回報值當起跳值」的行為；若不成立，
    使用者一取消勾選畫面就會跳掉。
    """
    auto = _plan_raw(True, 0.0)
    reported = auto['effective_scan_angles'][1]
    man = _plan_raw(False, reported)
    assert man['effective_scan_angles'][1] == pytest.approx(reported, abs=1e-9)
    assert man['total_makespan'] == pytest.approx(auto['total_makespan'], rel=1e-9)
    for uid, path in auto['assembled_paths'].items():
        assert len(man['assembled_paths'][uid].waypoints) == len(path.waypoints)


def test_negative_angle_is_the_same_direction_as_its_positive_twin():
    """-90° 與 270° 是同一個方向，不該規劃出不同的東西。"""
    a = _plan_raw(False, -90.0)
    b = _plan_raw(False, 270.0)
    assert a['total_makespan'] == pytest.approx(b['total_makespan'], rel=1e-9)
