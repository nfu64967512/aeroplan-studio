"""HARVEST 規劃結果在 3D 畫面上要畫出**實際的計畫**（2026-10-07 使用者回報「路徑全擠在同一路線」）。

三個顯示端錯誤疊在一起，讓一份條帶其實分開的計畫看起來全擠在起降坪附近：

  1. 分段不共用邊界點 ⇒ 縮點後每條條帶只剩 1 個 OPERATION 點 ⇒ 繪圖端丟掉 ⇒
     **一條條帶都沒畫**，圖例只剩「進入／轉移／降落」，距離只算到約 1/3。
  2. VTOL 降落段從 LANDING 段的**第一點**（最後一條條帶的終點，離坪 0.6–1.7 km）
     直線拉到降落點 ⇒ 三條實際不會飛的長黃線全部收斂在同一個坪。
  3. 圖例層與條帶虛線層**各自取色** ⇒ 同一架機兩層顏色不同（Pioneer 圖例橘、條帶藍）。

全部用真的 HARVEST 計畫斷言**值**（不是源碼字串），每條都先證明情境真的會觸發該錯誤。
"""
from __future__ import annotations

import json
import math
import os
import re

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mission.path_segments import (  # noqa: E402
    SEGMENT_KEYS, group_waypoints_by_segment, leg_owner_for, segments_for_path)
from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402

hv = pytest.importorskip('hv_dccpp.integration.aeroplan_adapter')

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))


def _ll(east_m, north_m):
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def _hdist(a, b):
    """兩點水平距離（m）；點為 (lat, lon, ...) 或 {'lat','lon'}。"""
    if isinstance(a, dict):
        a = (a['lat'], a['lon'])
    if isinstance(b, dict):
        b = (b['lat'], b['lon'])
    dy = (b[0] - a[0]) * _M_LAT
    dx = (b[1] - a[1]) * 111412.0 * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)


def _poly_len(pts):
    return sum(_hdist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _areas():
    return [{'area_id': 1, 'priority': 1.0, 'scan_angle_deg': 0.0,
             'polygon': [_ll(4000, -600), _ll(7000, -600), _ll(7000, 600), _ll(4000, 600)]}]


@pytest.fixture(scope='module')
def hv_result():
    """1 Pioneer(A) + 1 HERO(B)，GUI 預設的航點縮減開著（條帶因此縮成 1 點）。"""
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: 'A' if int(d['drone_id']) == 1 else 'B'
    c.hv_home = HOME
    c.harvest_params = {'min_separation_m': 100.0, 'alt_offset_m': 50.0,
                        'hv_waiver_max_agl_m': 200.0}
    drones = [
        {'drone_id': i, 'name': f'UAV-{i}', 'position': HOME, 'speed': 20.0,
         'heading': 0.0, 'turn_radius': r, 'vehicle_type': 'vtol', 'fov_width': 160.0}
        for i, r in ((1, 200.0), (2, 120.0))]
    res = c.plan_coverage_dccpp(areas=_areas(), drones=drones, coverage_width_m=160.0,
                                overlap_rate=0.25, coverage_altitude=160.0)
    assert res['assembled_paths'], '前提：HARVEST 要產生 assembled_paths'
    return res


@pytest.fixture(scope='module')
def plan(hv_result):
    return hv_result['assembled_paths']


@pytest.fixture(scope='module', params=['fixed_wing', 'vtol', 'multirotor'])
def legacy_plan(request):
    """舊版 DCCPP（不走 HARVEST）：ENTRY/TRANSFER 航點是 Dubins 腿的**終點**。"""
    vt = request.param
    drones = [{'drone_id': 1, 'name': 'UAV-1', 'position': HOME, 'speed': 18.0,
               'heading': 0.0, 'turn_radius': 0.0 if vt == 'multirotor' else 80.0,
               'vehicle_type': vt, 'fov_width': 160.0}]
    res = SwarmCoordinator().plan_coverage_dccpp(
        areas=_areas(), drones=drones, coverage_width_m=160.0, overlap_rate=0.25,
        coverage_altitude=100.0, vtol_transition_alt_m=30.0)
    ap = res['assembled_paths']
    assert ap, f'前提：舊版 DCCPP（{vt}）要產生 assembled_paths'
    return vt, ap


def _label_runs(wps):
    runs = []
    for wp in wps:
        name = wp.segment_type.name
        if runs and runs[-1][0] == name:
            runs[-1][1] += 1
        else:
            runs.append([name, 1])
    return runs


# ── 1. 條帶要畫得出來 ─────────────────────────────────────────────────────
def test_precondition_strips_are_single_point_runs(plan):
    """前提：縮點後 OPERATION 真的是 1 點一段 —— 否則下面的測試量不到原本的錯誤。"""
    for uid, apath in plan.items():
        op_runs = [n for name, n in _label_runs(apath.waypoints) if name == 'OPERATION']
        assert op_runs, f'UAV{uid} 沒有任何 OPERATION 航點'
        assert min(op_runs) == 1, f'UAV{uid} 的 OPERATION 段長度 {op_runs}，情境沒觸發 1 點段'


def test_every_strip_becomes_a_drawable_operation_path(plan):
    for uid, apath in plan.items():
        segs = segments_for_path(apath, 120.0)
        n_runs = sum(1 for name, _ in _label_runs(apath.waypoints) if name == 'OPERATION')
        ops = segs['operation_paths']
        assert len(ops) == n_runs
        assert all(len(s) >= 2 for s in ops), \
            f'UAV{uid} 有作業段不足 2 點（繪圖端會丟掉）：{[len(s) for s in ops]}'
        # 值鎖：畫出的作業長度 ≈ 規劃器自己依「腿歸出發點」累加的 operation_length_m
        drawn_op = sum(_poly_len(s) for s in ops)
        assert drawn_op == pytest.approx(apath.operation_length_m, rel=0.02), \
            f'UAV{uid}：作業段畫出 {drawn_op:.0f} m，規劃器記 {apath.operation_length_m:.0f} m'


def test_segments_cover_the_whole_polyline_without_gaps(plan):
    """各段長度加總 = 原航點折線長度：沒有漏掉段間的連接腿，也沒有重複計算。"""
    for uid, apath in plan.items():
        full = _poly_len([(w.lat, w.lon) for w in apath.waypoints])
        segs = segments_for_path(apath, 120.0)
        drawn = sum(_poly_len(s) for key in SEGMENT_KEYS.values() for s in segs[key]
                    if len(s) >= 2)
        assert drawn == pytest.approx(full, rel=1e-9), \
            f'UAV{uid}：畫得出來的長度 {drawn:.1f} m ≠ 計畫 {full:.1f} m'


def test_boundary_point_is_shared_between_consecutive_segments():
    """合成資料：A A B B A ⇒ [A0 A1 B2] [B2 B3 A4] [A4]（每條腿歸出發點）。"""
    class L:
        def __init__(self, name):
            self.name = name

    class W:
        def __init__(self, i, lab):
            self.lat, self.lon, self.alt, self.segment_type = float(i), 0.0, 1.0, lab

    op, xf = L('OPERATION'), L('TRANSFER')
    wps = [W(0, op), W(1, op), W(2, xf), W(3, xf), W(4, op)]
    segs = group_waypoints_by_segment(wps, 0.0)
    assert [[p[0] for p in s] for s in segs['operation_paths']] == [[0, 1, 2], [4]]
    assert [[p[0] for p in s] for s in segs['transfer_paths']] == [[2, 3, 4]]
    # 'end'（舊版 DCCPP）：腿歸終點 ⇒ [A0 A1] [A1 B2 B3] [B3 A4]
    segs = group_waypoints_by_segment(wps, 0.0, leg_owner='end')
    assert [[p[0] for p in s] for s in segs['operation_paths']] == [[0, 1], [3, 4]]
    assert [[p[0] for p in s] for s in segs['transfer_paths']] == [[1, 2, 3]]


def test_duplicate_boundary_point_yields_no_zero_length_segment():
    """舊版建構器常把 TRANSFER 終點與下一條帶起點寫成同一座標。

    'end' 慣例下那一段會變成 [b, b]：畫出來零長度、圖例卻多一列 ⇒ 必須濾掉。
    """
    class L:
        def __init__(self, name):
            self.name = name

    class W:
        def __init__(self, x, lab):
            self.lat, self.lon, self.alt, self.segment_type = float(x), 0.0, 1.0, lab

    op, xf = L('OPERATION'), L('TRANSFER')
    wps = [W(0, op), W(1, op), W(1, xf), W(2, op)]
    segs = group_waypoints_by_segment(wps, 0.0, leg_owner='end')
    drawable = [s for k in SEGMENT_KEYS.values() for s in segs[k] if len(s) >= 2]
    assert all(_poly_len(s) > 0 for s in drawable), f'有零長度的可畫段：{drawable}'
    assert segs['transfer_paths'] == []


def test_harvest_paths_use_start_ownership(plan):
    assert {leg_owner_for(ap) for ap in plan.values()} == {'start'}


# ── 1b. 舊版 DCCPP：腿歸終點，U 型轉彎／返場不可被畫成作業 ────────────────
def test_legacy_paths_use_end_ownership(legacy_plan):
    vt, ap = legacy_plan
    assert {leg_owner_for(p) for p in ap.values()} == {'end'}, f'{vt}：舊版路徑不該被當成 HARVEST'


def test_legacy_operation_length_matches_the_builder(legacy_plan):
    """作業段畫出的長度 ≈ 建構器自己記的 operation_length_m（U 型轉彎與返場不可混進來）。

    審查實測（固定翼）：腿歸出發點時 9592 → 12632 m（U 型轉彎與 1 km 返場被畫成作業）。
    """
    vt, ap = legacy_plan
    for uid, p in ap.items():
        runs = _label_runs(p.waypoints)
        assert any(n == 1 for name, n in runs if name == 'TRANSFER'), \
            f'{vt} 前提：舊版 TRANSFER 應是 1 點一段（只有 Dubins 終點）：{runs}'
        segs = segments_for_path(p, 100.0)
        drawn_op = sum(_poly_len(s) for s in segs['operation_paths'])
        assert drawn_op == pytest.approx(p.operation_length_m, rel=0.02), \
            f'{vt} UAV{uid}：作業段畫出 {drawn_op:.0f} m，建構器記 {p.operation_length_m:.0f} m'


def test_legacy_transfers_are_real_legs_and_nothing_is_lost(legacy_plan):
    vt, ap = legacy_plan
    for uid, p in ap.items():
        segs = segments_for_path(p, 100.0)
        for key in ('transfer_paths', 'entry_paths'):
            # 只看繪圖端真的會畫的段（≥2 點）；1 點的段（例：多旋翼以 ENTRY 開頭）本來就被略過
            lens = [_poly_len(s) for s in segs[key] if len(s) >= 2]
            assert all(L > 1.0 for L in lens), f'{vt} UAV{uid}：{key} 有零長度的可畫段 {lens}'
        full = _poly_len([(w.lat, w.lon) for w in p.waypoints])
        drawn = sum(_poly_len(s) for k in SEGMENT_KEYS.values() for s in segs[k] if len(s) >= 2)
        assert drawn == pytest.approx(full, rel=1e-9)


# ── 2. 降落段不可畫出不存在的長弦 ─────────────────────────────────────────
def test_vtol_landing_segment_starts_at_the_final_approach(plan):
    from mission.vtol_mission_exporter import VTOLMissionExporter
    data = {d['uav_id']: d for d in VTOLMissionExporter(transition_alt_m=40.0)
            .to_cesium_data(plan, 120.0)}
    for uid, apath in plan.items():
        wps = apath.waypoints
        land_first = next(w for w in wps if w.segment_type.name == 'LANDING')
        far = _hdist((land_first.lat, land_first.lon), (wps[-1].lat, wps[-1].lon))
        final_leg = _hdist((wps[-2].lat, wps[-2].lon), (wps[-1].lat, wps[-1].lon))
        # 前提：LANDING 段第一點離降落點夠遠，舊寫法才會畫出明顯的假弦
        assert far > final_leg + 50.0, f'UAV{uid} 情境沒觸發長弦（{far:.0f} vs {final_leg:.0f} m）'
        seg = data[uid]['landing_segment']
        assert _hdist(seg[0], (wps[-2].lat, wps[-2].lon)) < 1.0, \
            f'UAV{uid} 降落段應從最後進場腿的起點開始'
        horiz = _poly_len([(p['lat'], p['lon']) for p in seg])
        assert horiz == pytest.approx(final_leg, abs=1.0), \
            f'UAV{uid} 降落段水平長 {horiz:.0f} m，應等於最後一條進場腿 {final_leg:.0f} m'
        assert seg[-1]['alt'] == 0
        assert _hdist(seg[-1], (wps[-1].lat, wps[-1].lon)) < 1e-6


# ── 3. 圖例層與條帶層同一架機同一個顏色 ────────────────────────────────────
def test_strip_layer_and_legend_layer_use_the_same_colour(plan):
    from mission.vtol_mission_exporter import VTOLMissionExporter
    from ui.widgets.cesium_map_widget import CesiumMapWidget, swarm_drone_colors

    colors = swarm_drone_colors((uid, getattr(ap, 'cls', None)) for uid, ap in plan.items())
    assert len(set(colors.values())) == len(plan), '前提：不同機要拿到不同顏色'
    strip = {d['uav_id']: d['color'] for d in VTOLMissionExporter()
             .to_cesium_data(plan, 120.0, colors=colors)}

    # 圖例層：用與 main_window 相同的 drones 形狀餵 display_swarm_raw，攔下送給 JS 的顏色
    drones = []
    for uid, ap in plan.items():
        segs = segments_for_path(ap, 120.0)
        drones.append({'drone_id': uid, 'uav_class': getattr(ap, 'cls', None),
                       'platform': '', **segs})
    calls = []

    class _FakeWidget:                     # 未綁定方法 + 替身（QWidget 不能在此環境建構）
        _js = staticmethod(calls.append)

    CesiumMapWidget.display_swarm_raw(_FakeWidget(), {'drones': drones})

    legend = {}
    for js in calls:
        m = re.match(r'addPath\((".*?"),(".*?"),\d+,(".*?"),false\)$', js)
        if m:
            color, label = json.loads(m.group(2)), json.loads(m.group(3))
            uid = int(re.match(r'D(\d+)', label).group(1))
            legend.setdefault(uid, set()).add(color)
    assert set(legend) == set(plan), f'圖例層沒有畫到每一架：{sorted(legend)}'
    for uid in plan:
        assert legend[uid] == {strip[uid]}, \
            f'UAV{uid}：圖例層 {legend[uid]} ≠ 條帶層 {strip[uid]}'


# ── 4. 端到端：GUI 的兩個呼叫點真的用了上面的修正 ──────────────────────────
def _main_window():
    try:
        from ui.main_window import MainWindow          # 先匯入以載入 QtWebEngine
        from PyQt6.QtWidgets import QApplication
        QApplication.instance() or QApplication([])
        return MainWindow
    except Exception as e:                            # pragma: no cover - 環境相依
        pytest.skip(f'ui.main_window 匯入失敗：{e}')


class _Rec:
    """記錄呼叫的假 widget（不建立真的 MainWindow：本機 QtWebEngine 一 setHtml 就 abort）。"""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def _rec(*a, **k):
            self.calls.append((name, a, k))
            return self
        return _rec


def _run_display(result, vehicle_type):
    """用假 self 跑真的 `MainWindow._display_dccpp_result`，回傳送給地圖的 drones。"""
    MainWindow = _main_window()

    class _Win:
        home_point = HOME
        flight_params = {}

        def __init__(self):
            self.map_widget, self.vtol_recon_panel, self.mission_panel = _Rec(), _Rec(), _Rec()
            self.waypoint_label, self.distance_label, self._status = _Rec(), _Rec(), _Rec()
            self._dccpp_result = self._dccpp_metrics = None

        def statusBar(self):
            return self._status

        def _apply_fence_avoidance_to_dccpp_result(self, *a, **k):
            pass

    win = _Win()
    poly = _areas()[0]['polygon']
    MainWindow._display_dccpp_result(
        win, result, params={'collision_avoidance': False, 'auto_landing': False,
                             'vehicle_type': vehicle_type, 'transition_alt': 100.0},
        corners_snapshot=poly, centroid_lat=sum(c[0] for c in poly) / 4,
        centroid_lon=sum(c[1] for c in poly) / 4, altitude=160.0, speed=20.0,
        turn_radius=200.0, vehicle_type=vehicle_type, auto_scan=True)
    drawn = [c for c in win.map_widget.calls if c[0] == 'display_swarm_raw']
    assert len(drawn) == 1
    return drawn[0][1][0]['drones']


def test_display_dccpp_result_legacy_plan_keeps_turns_out_of_operation():
    """舊版 DCCPP 走同一個呼叫點：作業段長度要 ≈ 建構器的 operation_length_m。

    鎖住 main_window 依來源選慣例 —— 若呼叫點改回一律「腿歸出發點」，U 型轉彎與返場
    會被算進作業段而這條變紅（HARVEST 那條端到端測試抓不到這種回歸）。
    """
    drones = [{'drone_id': 1, 'name': 'UAV-1', 'position': HOME, 'speed': 18.0, 'heading': 0.0,
               'turn_radius': 80.0, 'vehicle_type': 'fixed_wing', 'fov_width': 160.0}]
    res = SwarmCoordinator().plan_coverage_dccpp(
        areas=_areas(), drones=drones, coverage_width_m=160.0, overlap_rate=0.25,
        coverage_altitude=100.0)
    sent = _run_display(res, 'fixed_wing')
    (d,) = sent
    ap = res['assembled_paths'][d['drone_id']]
    drawn_op = sum(_poly_len(s) for s in d['operation_paths'] if len(s) >= 2)
    assert drawn_op == pytest.approx(ap.operation_length_m, rel=0.02),         f'作業段畫出 {drawn_op:.0f} m，建構器記 {ap.operation_length_m:.0f} m'


def test_display_dccpp_result_draws_every_strip(hv_result):
    """`_display_dccpp_result` 送給地圖的 drones 資料：每條條帶都是可畫的作業段。

    鎖住 main_window 那個呼叫點 —— 若又換回內嵌的舊分段迴圈，這條會紅。
    """
    drones = _run_display(hv_result, 'vtol')
    ap = hv_result['assembled_paths']
    for d in drones:
        n_runs = sum(1 for name, _ in _label_runs(ap[d['drone_id']].waypoints) if name == 'OPERATION')
        ops = d['operation_paths']
        assert len(ops) == n_runs and all(len(s) >= 2 for s in ops), \
            f"D{d['drone_id']}：送給地圖的作業段 {[len(s) for s in ops]}（應 {n_runs} 段、每段 ≥2 點）"


def test_recon_planned_strip_layer_gets_the_legend_colours(hv_result):
    """`_on_vtol_recon_planned` 畫 VTOL 條帶層時，顏色必須等於圖例層那一份。

    鎖住 main_window 那個呼叫點 —— 拿掉 `colors=` 時這條會紅（只測匯出器蓋不到）。
    """
    MainWindow = _main_window()
    from ui.widgets.cesium_map_widget import swarm_drone_colors

    cesium = _Rec()

    class _Panel:
        def set_plan_status(self, *_a):
            pass

        def set_plan_result(self, *_a):
            pass

    class _Win:
        vtol_recon_panel = _Panel()
        _dccpp_result = hv_result
        _polyline_length_m = staticmethod(MainWindow._polyline_length_m)

        def _get_cesium_widget(self):
            return cesium

    MainWindow._on_vtol_recon_planned(_Win(), {'altitude': 160.0, 'transition_alt': 40.0}, True)
    sent = [c for c in cesium.calls if c[0] == 'draw_vtol_swarm_paths']
    assert len(sent) == 1, f'條帶層沒有被畫：{[c[0] for c in cesium.calls]}'
    got = {d['uav_id']: d['color'] for d in sent[0][1][0]}
    ap = hv_result['assembled_paths']
    want = swarm_drone_colors((uid, getattr(p, 'cls', None)) for uid, p in ap.items())
    assert got == want, f'條帶層 {got} ≠ 圖例層 {want}'
