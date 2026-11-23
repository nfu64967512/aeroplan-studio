"""群飛（DCCPP / HARVEST）圍籬改成「貼合航線的凸多邊形」（2026-10-08 使用者定調）。

使用者：「Fence 角度不該是寫死的固定角度，應該隨著我的路徑修正，平均覆蓋到全部的飛行路徑。」

舊行為：HARVEST 不回報掃描角 ⇒ 圍籬退成正南北的軸對齊矩形。把矩形轉到條帶方向也不夠——
起降場在作業區外、作業區不是長方形時，任何矩形都在角落留大片空白。使用者那份規劃實測：
軸對齊 6.10 km²、對齊條帶 6.43、最小旋轉矩形 5.53、凸多邊形（12 頂點）4.50 km²。

這裡鎖住（都是量出來的值，不是源碼字串）：
  * 每一段航線離圍籬邊至少 buffer（上傳飛控後不會一起飛就越界）；
  * 圍籬真的貼著航線（面積接近「凸包＋圓角 buffer」的理想值，明顯小於矩形）；
  * 頂點數 ≤ 上限、FENCE_TOTAL = 頂點數、兩種匯出都寫出全部頂點；
  * GUI 群飛顯示那條路徑真的用 'hull'。
"""
from __future__ import annotations

import json
import math
import os

import pytest

pytest.importorskip("shapely")
pytest.importorskip("pyproj")

import pyproj  # noqa: E402
from shapely.geometry import LineString, MultiPoint, Polygon  # noqa: E402

from mission.geofence_manager import GeofenceConstraintManager, GeofenceError  # noqa: E402

BUF = 60.0
ORIGIN = (25.0768, 121.2300)


def _ll(e, n, lat0=ORIGIN[0], lon0=ORIGIN[1]):
    return (lat0 + n / 111320.0, lon0 + e / (111320.0 * math.cos(math.radians(lat0))))


def _diag_mission(heading_deg=57.0, n_strips=8, length=2400.0, spacing=110.0,
                  launch=(-1500.0, -300.0), alt=120.0):
    """斜向來回條帶 + 作業區外的起降場（形狀仿使用者 2026-10-08 那份）。"""
    th = math.radians(heading_deg)
    ux, uy = math.sin(th), math.cos(th)              # 條帶方向（東、北）
    vx, vy = uy, -ux                                  # 垂直方向
    pts = [(*_ll(*launch), 0.0), (*_ll(*launch), alt)]
    for i in range(n_strips):
        off = (i - n_strips / 2) * spacing
        a = (off * vx - length / 2 * ux, off * vy - length / 2 * uy)
        b = (off * vx + length / 2 * ux, off * vy + length / 2 * uy)
        seq = (a, b) if i % 2 == 0 else (b, a)
        pts += [(*_ll(*seq[0]), alt), (*_ll(*seq[1]), alt)]
    pts += [(*_ll(*launch), alt), (*_ll(*launch), 0.0)]
    return pts


def _planar(geofence, pts):
    lat0 = sum(p[0] for p in pts) / len(pts)
    lon0 = sum(p[1] for p in pts) / len(pts)
    fwd = pyproj.Transformer.from_crs(
        "EPSG:4326", pyproj.CRS(proj="aeqd", lat_0=lat0, lon_0=lon0, datum="WGS84"), always_xy=True)
    xy = lambda la, lo: fwd.transform(lo, la)       # noqa: E731
    poly = Polygon([xy(la, lo) for la, lo in geofence.vertices])
    legs = [LineString([xy(a[0], a[1]), xy(b[0], b[1])]) for a, b in zip(pts, pts[1:])
            if (a[0], a[1]) != (b[0], b[1])]
    cloud = MultiPoint([xy(p[0], p[1]) for p in pts])
    return poly, legs, cloud


def _hull(pts, **kw):
    return GeofenceConstraintManager(buffer_radius_m=BUF, method="hull", **kw).build(waypoints=pts)


# ── 1. 安全：每一段航線離圍籬邊至少 buffer ─────────────────────────────────
@pytest.mark.parametrize("heading", [30.0, 57.0, 123.4])
def test_every_leg_is_at_least_buffer_inside(heading):
    pts = _diag_mission(heading)
    poly, legs, _ = _planar(_hull(pts).geofence, pts)
    assert poly.is_valid
    worst = min(poly.exterior.distance(leg) for leg in legs)
    assert all(poly.contains(leg) for leg in legs), "有航線段跑出圍籬"
    assert worst >= BUF - 0.5, f"最近的航線離圍籬邊只有 {worst:.1f} m（緩衝 {BUF} m）"


# ── 2. 平均：貼著航線，而不是一個大矩形 ─────────────────────────────────────
@pytest.mark.parametrize("heading", [30.0, 57.0, 123.4])
def test_fence_hugs_the_paths(heading):
    pts = _diag_mission(heading)
    hull_b = _hull(pts)
    poly, _legs, cloud = _planar(hull_b.geofence, pts)
    ideal = cloud.convex_hull.buffer(BUF, resolution=32)        # 凸包＋圓角 buffer（理想）
    assert poly.area <= 1.12 * ideal.area, \
        f"凸多邊形圍籬 {poly.area/1e6:.3f} km² 比理想 {ideal.area/1e6:.3f} km² 大太多（沒貼著航線）"
    aabb = GeofenceConstraintManager(buffer_radius_m=BUF, method="aabb").build(waypoints=pts).geofence
    rect, _, _ = _planar(aabb, pts)
    assert poly.area < 0.85 * rect.area, \
        f"凸多邊形 {poly.area/1e6:.3f} km² 沒有明顯小於軸對齊矩形 {rect.area/1e6:.3f} km²"


def _ring_mission(n=10, r=900.0):
    """不規則的一圈點（凸包有 n 個頂點），用來測非矩形的匯出。"""
    return [(*_ll(r * math.sin(2 * math.pi * k / n) * (1 + 0.15 * (k % 3)),
                  r * math.cos(2 * math.pi * k / n)), 120.0) for k in range(n)]


# ── 3. 頂點上限、FENCE_TOTAL、標示 ───────────────────────────────────────
@pytest.mark.parametrize("cap", [12, 6, 4])
def test_vertex_cap_and_fence_total(cap):
    # ⚠ 用不規則的一圈點：斜向條帶的凸包剛好是平行四邊形（4 頂點），拿它測
    #   「FENCE_TOTAL == 頂點數」會在寫死 4 的時候照樣通過（突變實測過）。
    pts = _ring_mission()
    b = _hull(pts, max_vertices=cap)
    g = b.geofence
    if cap == 12:
        assert g.vertex_count() > 4, "前提：這個情境要真的是非矩形，否則量不到 FENCE_TOTAL"
    assert 3 <= g.vertex_count() <= cap
    assert b.fence_params["FENCE_TOTAL"] == g.vertex_count()
    assert g.method == "hull" and "HULL" in b.summary()
    poly, legs, _ = _planar(g, pts)
    assert min(poly.exterior.distance(leg) for leg in legs) >= BUF - 0.5, "壓頂點之後包不住了"


def test_rectangle_modes_still_report_four():
    b = GeofenceConstraintManager(buffer_radius_m=BUF, method="aabb").build(waypoints=_diag_mission())
    assert b.geofence.vertex_count() == 4 and b.fence_params["FENCE_TOTAL"] == 4


@pytest.mark.parametrize("pts", [
    [(25.07, 121.22, 50.0), (25.08, 121.23, 60.0)],          # 兩點（共線）
    [(25.07, 121.22, 50.0)],                                  # 單點
])
def test_degenerate_inputs_still_make_a_valid_fence(pts):
    b = _hull(pts)
    poly, _, cloud = _planar(b.geofence, pts)
    assert poly.is_valid and 3 <= b.geofence.vertex_count() <= 12
    assert all(poly.exterior.distance(MultiPoint([p])) >= BUF - 0.5 for p in cloud.geoms)


def test_hull_and_scan_angle_are_mutually_exclusive():
    with pytest.raises(GeofenceError):
        GeofenceConstraintManager(buffer_radius_m=BUF, method="hull").build(
            waypoints=_diag_mission(), scan_angle_deg=57.0)


# ── 4. 匯出：兩種格式都寫出全部頂點 ─────────────────────────────────────────
def test_exports_write_every_vertex(tmp_path):
    b = _hull(_ring_mission())
    n = b.geofence.vertex_count()
    assert n > 4, "前提：要測的是非矩形"
    plan = json.loads(GeofenceConstraintManager.export_qgc_plan(b, tmp_path / "f.plan").read_text("utf-8"))
    assert len(plan["geoFence"]["polygons"][0]["polygon"]) == n
    fen = GeofenceConstraintManager.export_mission_planner_fen(b, tmp_path / "f.fen").read_text("utf-8")
    lines = [ln for ln in fen.splitlines() if ln and not ln.startswith("#")]
    assert len(lines) == 1 + n + 1, "應為 breach 點 + n 個頂點 + 閉合點"


# ── 5. GUI 群飛顯示那條路徑真的用 'hull' ─────────────────────────────────────
def _dual():
    try:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from ui.widgets.dual_map_widget import DualMapWidget
        return DualMapWidget
    except Exception as e:                            # pragma: no cover - 環境相依
        pytest.skip(f"dual_map_widget 匯入失敗：{e}")


class _Sink:
    def __getattr__(self, name):                      # 任何畫圖動作都吞掉
        return lambda *a, **k: None


class _FakeDual:
    def __init__(self, DualMapWidget):
        self.map_2d, self.map_3d = _Sink(), _Sink()
        self._fence_visible = True
        self._last_fence_bundle = "STALE"
        self.built = []
        self.fence_built = type("S", (), {"emit": lambda _s, b: self.built.append(b)})()
        self._auto_build_fence = DualMapWidget._auto_build_fence.__get__(self)
        self._swarm_raw_to_paths = DualMapWidget._swarm_raw_to_paths

    def __getattr__(self, name):                      # 地圖畫圖動作一律吞掉
        return lambda *a, **k: None


@pytest.mark.parametrize("angle", [None, 62.0])
def test_swarm_display_builds_a_hull_fence(angle):
    DualMapWidget = _dual()
    pts = _diag_mission()
    fake = _FakeDual(DualMapWidget)
    swarm = {"drones": [{"drone_id": 1, "takeoff_paths": [pts[:2]], "operation_paths": [pts[2:-2]],
                         "landing_paths": [pts[-2:]]}]}
    DualMapWidget.display_swarm_raw(fake, swarm, scan_angle_deg=angle)
    b = fake._last_fence_bundle
    assert b is not None and b != "STALE", "群飛顯示沒有建出圍籬"
    assert b.geofence.method == "hull", f"群飛圍籬應是貼合航線的多邊形，實際 {b.geofence.method}"
    poly, legs, _ = _planar(b.geofence, pts)
    assert all(poly.contains(leg) for leg in legs)


# ── 6. 使用者那份規劃（起降場在作業區西側、1 Pioneer + 2 HERO）──────────────
def test_users_plan_fence_is_much_smaller_than_the_rectangle():
    hv = pytest.importorskip("hv_dccpp.integration.aeroplan_adapter")  # noqa: F841
    from mission.swarm_coordinator import SwarmCoordinator
    from utils.math_utils import spawn_positions

    corners = [(25.079127, 121.229024), (25.069526, 121.228809),
               (25.079322, 121.244538), (25.084316, 121.241469)]
    home = (25.076814555992296, 121.22078418731691)
    pos = spawn_positions(home[0], home[1], 3, 0.0, 60.0)
    drones = [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": pos[i], "speed": 22.0,
               "heading": 0.0, "turn_radius": 60.0, "vehicle_type": "vtol", "fov_width": 110.85}
              for i in range(3)]
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: "A" if int(d["drone_id"]) == 1 else "B"
    c.hv_home = home
    c.harvest_params = {"min_separation_m": 50.0, "alt_offset_m": 50.0,
                        "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0}, "hv_dh_band_min_m": 15.0}
    res = c.plan_coverage_dccpp(
        areas=[{"area_id": 1, "polygon": corners, "priority": 1.0, "scan_angle_deg": 0.0}],
        drones=drones, coverage_width_m=110.85, overlap_rate=0.0, coverage_altitude=120.0,
        vtol_transition_alt_m=40.0)
    pts = [(w.lat, w.lon, w.alt) for p in res["assembled_paths"].values() for w in p.waypoints]
    assert len(pts) > 20, "前提：要有一份真的計畫"

    hb = _hull(pts)
    poly, legs, _ = _planar(hb.geofence, pts)
    rect, _, _ = _planar(GeofenceConstraintManager(buffer_radius_m=BUF, method="aabb").build(waypoints=pts).geofence, pts)
    assert all(poly.contains(leg) for leg in legs)
    assert min(poly.exterior.distance(leg) for leg in legs) >= BUF - 0.5
    assert poly.area < 0.85 * rect.area, \
        f"使用者那份：凸多邊形 {poly.area/1e6:.2f} km² vs 軸對齊 {rect.area/1e6:.2f} km²"
    need = _max_from_pads(pts) + BUF
    assert hb.fence_params["FENCE_RADIUS"] >= need - 0.5, \
        f"使用者那份：FENCE_RADIUS {hb.fence_params['FENCE_RADIUS']:.0f} m < 起降坪到最遠航點 + 緩衝 {need:.0f} m"


# ── 7. 圓形圍籬半徑：ArduPilot 以 Home 為圓心，不是航點中心 ─────────────────
def _max_from_pads(pts):
    def d(a, b):
        return math.hypot((b[0] - a[0]) * 111320.0,
                          (b[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0])))
    pads = [p for p in pts if p[2] <= 1.0]
    assert pads, "前提：要有貼地的起降坪航點"
    return max(d(h, p) for h in pads for p in pts)


@pytest.mark.parametrize("method", ["hull", "aabb"])
def test_circle_radius_covers_every_waypoint_from_the_pad(method):
    """起降場在作業區外（使用者 2026-10-08 的用法）：FENCE_RADIUS 要涵蓋起降坪到最遠航點。

    舊算法用航點中心 ⇒ 使用者那份只有 1.66 km、實際要 2.92 km ⇒ 任務途中圓形越界 RTL。
    """
    pts = _diag_mission(launch=(-1500.0, -300.0))
    b = GeofenceConstraintManager(buffer_radius_m=BUF, method=method).build(waypoints=pts)
    need = _max_from_pads(pts) + BUF
    assert b.fence_params["FENCE_RADIUS"] >= need - 0.5, \
        f"FENCE_RADIUS {b.fence_params['FENCE_RADIUS']:.0f} m < 起降坪到最遠航點 + 緩衝 {need:.0f} m"


def test_circle_radius_without_ground_points_covers_the_whole_set():
    """沒有貼地航點時不知道 Home 在哪 ⇒ 半徑至少是點集直徑（Home 在任何航點都不誤觸）。"""
    pts = [(p[0], p[1], 100.0) for p in _diag_mission()]
    b = GeofenceConstraintManager(buffer_radius_m=BUF).build(waypoints=pts)
    def d(a, c):
        return math.hypot((c[0] - a[0]) * 111320.0, (c[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0])))
    diam = max(d(a, c) for a in pts for c in pts)
    assert b.fence_params["FENCE_RADIUS"] >= diam + BUF - 1.0


def test_base_layer_default_is_hull():
    """2026-10-08 使用者：「規劃器出來的都要貼合路徑，這是底層規劃的」⇒ 不指定 method 就是 'hull'。"""
    b = GeofenceConstraintManager(buffer_radius_m=BUF).build(waypoints=_ring_mission())
    assert b.geofence.method == "hull"
    assert b.geofence.vertex_count() > 4


@pytest.mark.parametrize("entry,args", [
    ("display_path", lambda p: (p, 120.0)),
    ("display_paths", lambda p: ([p], 120.0)),
    ("display_fw_paths", lambda p: (p[:2], p, p[-2:])),
    ("display_swarm_raw", lambda p: ({"drones": [{"drone_id": 1, "operation_paths": [p]}]},)),
])
def test_every_gui_entry_builds_a_hull_fence(entry, args):
    """所有建圍籬的 GUI 入口（單路徑／多路徑／固定翼／群飛）都產生貼合航線的多邊形，給了掃描角也一樣。"""
    DualMapWidget = _dual()
    pts = _ring_mission()
    fake = _FakeDual(DualMapWidget)
    getattr(DualMapWidget, entry)(fake, *args(pts), scan_angle_deg=30.0)
    b = fake._last_fence_bundle
    assert b is not None and b != "STALE", f"{entry} 沒有建出圍籬"
    assert b.geofence.method == "hull", f"{entry} 的圍籬是 {b.geofence.method}，不是貼合航線"
