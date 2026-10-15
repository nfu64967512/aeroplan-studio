"""HARVEST 的覆蓋率驗算量的是**飛出去的條帶**，不是一條畫歪的線（2026-10-09 使用者問
「為何 HARVEST 覆蓋率驗算都那麼差」）。

GUI 的驗算只取「相機開著的航點」連成折線。HARVEST 的 ``sensor_on`` 屬於**出發腿**（航點 i 描述
wp[i] → wp[i+1]），而縮點後每條條帶只剩起點帶旗標、終點是別的標籤 ⇒ 每條條帶只剩 1 個孤立的點，
驗算再把這些點用斜線連起來 ⇒ 量的是一條跨過整區的鋸齒線。
實測（log 23:28 那份計畫：4 角點、1 Pioneer + 1 HERO）：GUI 75.06%（10 個「成像航點」），
實際飛出去的路徑 100.00%（3585/3585 格）—— 規劃沒有問題，是尺畫歪了。

全部用真的 HARVEST 計畫斷言**值**；每條先證明情境真的會觸發舊量法。
"""
from __future__ import annotations

import math
import os
import types

import numpy as np
import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from core.recon import verify_coverage  # noqa: E402
from mission.path_segments import imaging_path  # noqa: E402
from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402
from utils.math_utils import spawn_positions  # noqa: E402

pytest.importorskip('hv_dccpp.integration.aeroplan_adapter')

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
SWATH = 2 * 120.0 * math.tan(math.radians(60.0) / 2)         # 138.564 m（GUI 驗算用的原始刈幅）
CELL = 20.0


def _ll(east_m, north_m):
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


POLY = [_ll(3000, -450), _ll(4600, -450), _ll(4600, 450), _ll(3000, 450)]      # 1.6 km × 0.9 km
CLASSES = {1: 'A', 2: 'B'}


@pytest.fixture(scope='module')
def planned():
    """GUI 同一條路：SwarmCoordinator（HARVEST，縮點預設開）、1 Pioneer + 1 HERO。"""
    pts = spawn_positions(HOME[0], HOME[1], 2, 0.0, 60.0)
    drones = [{'drone_id': i + 1, 'name': f'UAV-{i + 1}', 'position': pts[i], 'speed': 20.0,
               'heading': 0.0, 'turn_radius': 150.0, 'vehicle_type': 'vtol', 'fov_width': 110.0}
              for i in range(2)]
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: CLASSES.get(int(d['drone_id']), 'A')
    c.hv_home = HOME
    c.harvest_params = {'min_separation_m': 100.0, 'alt_offset_m': 50.0, 'hv_waiver_max_agl_m': 200.0,
                        'hv_reduce_waypoints': True}
    res = c.plan_coverage_dccpp(areas=[{'area_id': 1, 'priority': 1.0, 'scan_angle_deg': 0.0, 'polygon': POLY}],
                                drones=drones, coverage_width_m=110.0, overlap_rate=0.20,
                                coverage_altitude=120.0)
    assert res['assembled_paths'], '前提：要有 assembled_paths'
    return c, res


def _cov(paths):
    return verify_coverage(paths, POLY, SWATH, CELL)


def _old_gui_paths(aps):
    """修正前 `_on_vtol_recon_verify` 的取法：sensor_on 的航點連成一條折線。"""
    return {int(u): [(float(w.lat), float(w.lon)) for w in p.waypoints if bool(getattr(w, 'sensor_on', False))]
            for u, p in aps.items()}


def _native_truth(c):
    """規劃器原生（未縮點）計畫：每個 sensor_on 航點 → 下一個航點是一條成像腿。"""
    plan, ctx = c.hv_coordinator.last_plan, c.hv_coordinator.last_ctx
    paths, k = {}, 0
    for up in plan.plans.values():
        ll = ctx.frame.to_wgs84(np.array([[w.x_m, w.y_m, 0.0] for w in up.waypoints]))
        for i in range(len(up.waypoints) - 1):
            if up.waypoints[i].sensor_on:
                paths[k] = [(float(ll[i, 0]), float(ll[i, 1])), (float(ll[i + 1, 0]), float(ll[i + 1, 1]))]
                k += 1
    assert paths, '前提：原生計畫要有成像腿'
    return _cov(paths)


# ── 前提：舊量法真的會低估，而且不是規劃的問題 ─────────────────────────────────
def test_precondition_old_method_underreports_a_fully_covered_plan(planned):
    c, res = planned
    truth = _native_truth(c)
    assert truth.coverage_pct > 99.5, f'前提：規劃本身要覆蓋完整（{truth.coverage_pct:.2f}%）'
    old = _cov(_old_gui_paths(res['assembled_paths']))
    assert old.coverage_pct < 90.0, f'舊量法沒有低估（{old.coverage_pct:.2f}%）⇒ 情境沒觸發這個缺陷'
    # 縮點後每條條帶只剩 1 個帶旗標的點（所以才會變成孤立點）
    aps = res['assembled_paths']
    n_flagged = sum(1 for p in aps.values() for w in p.waypoints if getattr(w, 'sensor_on', False))
    n_strips = sum(len(up.strips) for up in c.hv_coordinator.last_plan.plans.values())
    assert n_flagged <= n_strips + 1, (n_flagged, n_strips)


# ── 修正後 ───────────────────────────────────────────────────────────────────
def test_leg_rule_reports_the_true_coverage_of_the_flown_path(planned):
    c, res = planned
    truth = _native_truth(c)
    new = _cov({int(u): imaging_path(p)[0] for u, p in res['assembled_paths'].items()})
    assert new.coverage_pct > 99.5, f'修正後仍低估：{new.coverage_pct:.2f}%'
    assert new.coverage_pct == pytest.approx(truth.coverage_pct, abs=0.5), (new.coverage_pct, truth.coverage_pct)


def test_each_strip_is_its_own_imaging_run(planned):
    c, res = planned
    plan = c.hv_coordinator.last_plan
    for did, p in res['assembled_paths'].items():
        path, crit = imaging_path(p)
        assert 'sensor_on' in crit
        runs = 1 + sum(1 for x in path if x is None)
        up = plan.plans[c.hv_coordinator.id_map[did]]
        assert runs == len(up.strips), f'UAV-{did}：{runs} 段成像線 ≠ {len(up.strips)} 條條帶'
        assert path[0] is not None and path[-1] is not None


def test_imaging_legs_have_the_strip_length(planned):
    """成像腿要有條帶的長度（舊量法每條只有一個點 ⇒ 長度 0）。"""
    _, res = planned
    for did, p in res['assembled_paths'].items():
        path, _ = imaging_path(p)
        runs, cur = [], []
        for x in path + [None]:
            if x is None:
                runs.append(cur)
                cur = []
            else:
                cur.append(x)
        for r in runs:
            L = sum(math.hypot((b[0] - a[0]) * _M_LAT, (b[1] - a[1]) * _M_LON) for a, b in zip(r, r[1:]))
            assert L > 500.0, f'UAV-{did} 有成像段只有 {L:.0f} m（條帶應是公里級）'


# ── GUI 真正的處理函式（假 self，不建 MainWindow）────────────────────────────
def test_gui_handler_reports_the_fixed_number(planned):
    """走 `MainWindow._on_vtol_recon_verify` 本人：回報給面板的覆蓋率要是修正後的值。"""
    from ui.main_window import MainWindow
    _, res = planned
    shown = {}
    msgs = []
    fake = types.SimpleNamespace(
        vtol_recon_panel=types.SimpleNamespace(set_coverage_result=lambda r: shown.setdefault('report', r)),
        _dccpp_result=res, corners=list(POLY),
        statusBar=lambda: types.SimpleNamespace(showMessage=lambda *a, **k: msgs.append(a[0])))
    MainWindow._on_vtol_recon_verify(fake, {'swath_width_m': SWATH, 'cell_size_m': CELL})
    rep = shown.get('report')
    assert rep is not None, '面板沒收到驗算結果'
    assert rep.coverage_pct > 99.5, f'GUI 仍回報 {rep.coverage_pct:.2f}%'
    assert fake._vtol_coverage_report is rep


# ── imaging_path 單元 ────────────────────────────────────────────────────────
def _w(i, on, label='TRANSFER'):
    return types.SimpleNamespace(lat=float(i), lon=float(i) * 10.0, sensor_on=on,
                                 segment_type=types.SimpleNamespace(name=label))


def _p(*ws):
    return types.SimpleNamespace(waypoints=list(ws))


def test_imaging_path_takes_the_departing_leg_of_each_flagged_waypoint():
    # 航點：0 關、1 開（腿 1→2）、2 關、3 關、4 開（腿 4→5）、5 關
    ap = _p(*[_w(i, on) for i, on in enumerate([False, True, False, False, True, False])])
    path, _ = imaging_path(ap)
    assert path == [(1.0, 10.0), (2.0, 20.0), None, (4.0, 40.0), (5.0, 50.0)]


def test_consecutive_flagged_waypoints_form_one_run_without_duplicates():
    ap = _p(*[_w(i, on) for i, on in enumerate([False, True, True, True, False, True, False])])
    path, _ = imaging_path(ap)
    assert path == [(1.0, 10.0), (2.0, 20.0), (3.0, 30.0), (4.0, 40.0), None, (5.0, 50.0), (6.0, 60.0)]


def test_flagged_last_waypoint_has_no_leg_but_is_kept_as_a_point():
    path, _ = imaging_path(_p(_w(0, False), _w(1, True)))
    assert path == [(1.0, 10.0)]
    path, _ = imaging_path(_p(_w(0, True), _w(1, True)))
    assert path == [(0.0, 0.0), (1.0, 10.0)]


def test_without_sensor_flag_falls_back_to_operation_labels_unchanged():
    """舊版 DCCPP（不帶 sensor_on）：OPERATION 航點，沒有 None，與修正前相同。"""
    ap = _p(_w(0, False, 'TAKEOFF'), _w(1, False, 'OPERATION'), _w(2, False, 'OPERATION'),
            _w(3, False, 'TRANSFER'), _w(4, False, 'OPERATION'))
    path, crit = imaging_path(ap)
    assert path == [(1.0, 10.0), (2.0, 20.0), (4.0, 40.0)]
    assert None not in path and 'OPERATION' in crit


def test_empty_and_unflagged_paths_give_empty():
    assert imaging_path(_p())[0] == []
    assert imaging_path(_p(_w(0, False), _w(1, False)))[0] == []
