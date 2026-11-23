"""覆蓋規劃基準測試台：同一把尺量 HARVEST、DCCPP 與常見演算法。

⚠⚠ 公平性是這支程式唯一的目的。三條規矩：

1. **所有方法都只回傳「每機的航跡折線」**（公尺、以 home 為原點的平面座標），
   之後用同一段程式算指標。誰都不准自己報自己的長度或覆蓋率。
   （舊 DCCPP 的 `total_length_m` 是一本帳不是航跡：`dccpp_path_builder.py:255`
   把轉場簡化成只吐終點卻照記 Dubins 弧長，實測帳面比折線多 13.7%。）
2. **時間一律用 `長度 / 同一個速度`**，不套各自的動力學。這樣掃描率就是純幾何量，
   跨方法可比。各規劃器自己的 makespan 另外報，並標明**不可跨方法比較**
   （HARVEST 的 makespan 含垂直起降轉換與錯開放行，基準演算法沒有這些）。
3. **刈幅兩邊相同**，取 `2·h·tan(HFOV/2)`。問的是「同一台相機飛這個圖案蓋不蓋得到」。

指標
----
``coverage``      覆蓋率 = (掃過 ∩ 作業區) / 作業區。**低於 0.99 時其餘指標無意義**
``overhead``      航程開銷 = 總航程 / (覆蓋面積 / 刈幅)。理想 1.0，越低越好
``redundancy``    重複掃描 = Σ 各機掃過面積 / 聯集面積（≥1，越低越省）
``balance``       負載不均 = 最長 / 平均（1.0 = 完全平均）
``scan_rate``     掃描率 = 覆蓋面積 / (最長航程 / 速度)，km²/h
``waste``         區外浪費 = 掃在作業區外的面積 / 總掃過面積
"""
from __future__ import annotations

import math
import sys

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from shapely.geometry import LineString, Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

HOME_LL = (23.95, 120.90)
M_LAT = 111132.0
M_LON = 111412.0 * math.cos(math.radians(HOME_LL[0]))
SPEED_REF = 20.0            # m/s，所有方法共用，讓掃描率是純幾何量

Track = list                # list[tuple[float, float]]


# ── 座標 ──────────────────────────────────────────────────────────────
def xy(lat: float, lon: float):
    return ((lon - HOME_LL[1]) * M_LON, (lat - HOME_LL[0]) * M_LAT)


def ll(east_m: float, north_m: float):
    return (HOME_LL[0] + north_m / M_LAT, HOME_LL[1] + east_m / M_LON)


def rect_xy(cx, cy, he, hn) -> Polygon:
    return Polygon([(cx - he, cy - hn), (cx + he, cy - hn),
                    (cx + he, cy + hn), (cx - he, cy + hn)])


def polys_to_areas(polys: list[Polygon]) -> list[dict]:
    """平面多邊形 → 規劃器吃的 areas（經緯度）。"""
    return [{"area_id": i + 1, "priority": 1.0,
             "polygon": [ll(x, y) for x, y in p.exterior.coords[:-1]]}
            for i, p in enumerate(polys)]


def _len(t: Track) -> float:
    return sum(math.dist(t[i], t[i - 1]) for i in range(1, len(t)))


# ── 指標（唯一一份）──────────────────────────────────────────────────
def metrics(polys: list[Polygon], tracks: list[Track], swath_m: float,
            speed_mps: float = SPEED_REF) -> dict:
    tracks = [[p for i, p in enumerate(t) if i == 0 or p != t[i - 1]] for t in tracks]
    tracks = [t for t in tracks if len(t) >= 2]
    area = unary_union(polys)
    if not tracks:
        return dict(n_uav=0, total_km=0.0, max_km=0.0, coverage=0.0, overhead=None,
                    redundancy=None, balance=None, scan_rate=0.0, waste=None)
    bufs = [LineString(t).buffer(swath_m / 2.0, quad_segs=8) for t in tracks]
    swept = unary_union(bufs)
    covered = area.intersection(swept).area
    lens = [_len(t) for t in tracks]
    total, mx = sum(lens), max(lens)
    ideal = covered / swath_m if swath_m > 0 else float("nan")
    inside = [b.intersection(area).area for b in bufs]
    return dict(
        n_uav=len(tracks),
        total_km=round(total / 1000.0, 2),
        max_km=round(mx / 1000.0, 2),
        coverage=round(covered / area.area, 4) if area.area else 0.0,
        overhead=round(total / ideal, 3) if ideal > 0 else None,
        redundancy=round(sum(inside) / covered, 3) if covered > 0 else None,
        balance=round(mx / (total / len(lens)), 3),
        scan_rate=round((covered / 1e6) / ((mx / speed_mps) / 3600.0), 3) if mx else 0.0,
        waste=round(1.0 - covered / swept.area, 4) if swept.area else None,
    )


# ── 受測方法：兩個規劃器 ─────────────────────────────────────────────
def _drones(n_a, n_b, width):
    perf = {"A": (21.5, 200.0), "B": (19.0, 120.0)}
    fleet = [("A", *perf["A"])] * n_a + [("B", *perf["B"])] * n_b
    return ([{"drone_id": i + 1, "name": f"{c}{i + 1}", "position": HOME_LL,
              "speed": v, "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
              "fov_width": width} for i, (c, v, r) in enumerate(fleet)],
            {i + 1: c for i, (c, _v, _r) in enumerate(fleet)})


def run_planner(use_hv: bool, polys, n_a, n_b, *, alt, pitch, overlap,
                hfov=60.0, vfov=45.0, wind=(5.0, 270.0)):
    """回傳 (tracks, 自報 makespan 秒)。自報 makespan **不可跨方法比較**。"""
    import logging
    logging.disable(logging.CRITICAL)
    from mission.swarm_coordinator import SwarmCoordinator
    drones, cls = _drones(n_a, n_b, pitch)
    c = SwarmCoordinator()
    if use_hv:
        c.use_harvest = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME_LL
        c.hv_wind = wind
        c.hv_fence_zones = []
        c.harvest_params = {"hfov_deg": hfov, "vfov_deg": vfov, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    try:
        res = c.plan_coverage_dccpp(areas=polys_to_areas(polys), drones=drones,
                                    coverage_width_m=pitch, overlap_rate=overlap,
                                    coverage_altitude=alt, mounting_angle_deg=0.0)
    except Exception as e:                                # noqa: BLE001
        return [], None, f"{type(e).__name__}: {e}"
    tracks = []
    for p in (res.get("assembled_paths") or {}).values():
        t = [xy(w.lat, w.lon) for w in getattr(p, "waypoints", [])]
        if len(t) >= 2:
            tracks.append(t)
    # ⚠⚠⚠ 規劃器**自己**會說哪些胞蓋不到（`plan.uncovered`），而舊版只取
    #   `assembled_paths` 就把那個訊號丟掉了。後果:S1 n=2 / S5 n=8 這種
    #   「機隊吃不下這塊地」的格子，表上看起來是「交出了一份只蓋 52% 的計畫」，
    #   而實際上規劃器有明講 `no_feasible_insertion` 且 `validate` 是 False。
    #   會宣告「我蓋不完」的方法，不該和默默少蓋的方法記成同一件事。
    _self = {"self_uncovered": "", "self_validate": ""}
    try:
        hv = getattr(c, "hv_coordinator", None)
        if hv is not None and getattr(hv, "last_plan", None) is not None:
            _self["self_uncovered"] = len(hv.last_plan.uncovered or ())
        v = getattr(c, "hv_validation", None)
        if v is not None:
            _self["self_validate"] = "pass" if v.passed else "FAIL"
    except Exception:                                     # noqa: BLE001
        pass
    return tracks, res.get("total_makespan"), None, _self


# ── 基準演算法：介面 ─────────────────────────────────────────────────
#   def baseline(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)) -> list[Track]
#   所有基準都必須：起點與終點都在 `start`（來回都算進航程），只回傳折線。
# ---------------------------------------------------------------------
def min_width_angle(poly: Polygon) -> float:
    """最小寬度方向（rad）。條帶垂直於它 ⇒ 條帶數最少，這是教科書做法。"""
    hull = poly.convex_hull
    cs = list(hull.exterior.coords)[:-1]
    best, best_w = 0.0, float("inf")
    for i in range(len(cs)):
        ax, ay = cs[i]
        bx, by = cs[(i + 1) % len(cs)]
        th = math.atan2(by - ay, bx - ax)
        # ⚠⚠⚠ 旋轉 −th 之後的 y 是 **s·x + c·y**。舊版寫 `-s*x + c*y`，
        #   因為 s = −sin(th)，等於量「對 x 軸鏡射後」的寬度 ⇒ 非軸對齊的區塊
        #   會選到錯的條帶方向（旋轉 30/45/60° 的矩形實測寬度多算 95%）。
        #   `mission/swarm_coordinator._min_width_dir` 有同一個錯，同日一起修。
        #   ⚠ 本基準的五個情境**全部軸對齊**（sin 項在 0° 消失），實測 8 個多邊形
        #     的 excess 都是 0.00% ⇒ **已封存的 228 格結果不受影響**。
        #     但加入任何旋轉或斜邊情境之前，這個錯一定要先修掉。
        c, s = math.cos(-th), math.sin(-th)
        ys = [(s * x + c * y) for x, y in cs]       # 旋轉後的 y
        w = max(ys) - min(ys)
        if w < best_w:
            best_w, best = w, th
    return best


def _strips(poly: Polygon, pitch: float, theta: float) -> list[tuple]:
    """沿 theta 方向、間距 pitch 的條帶，裁進多邊形。回傳 [(p0, p1), ...] 由近到遠。"""
    c, s = math.cos(-theta), math.sin(-theta)
    rot = Polygon([(c * x - s * y, s * x + c * y) for x, y in poly.exterior.coords[:-1]])
    x0, y0, x1, y1 = rot.bounds
    out, y = [], y0 + pitch / 2.0
    while y < y1:
        seg = rot.intersection(LineString([(x0 - 10, y), (x1 + 10, y)]))
        for g in getattr(seg, "geoms", [seg]):
            if g.geom_type == "LineString" and g.length > 1e-6:
                out.append((tuple(g.coords[0]), tuple(g.coords[-1]), y))
        y += pitch
    ci, si = math.cos(theta), math.sin(theta)
    back = lambda p: (ci * p[0] - si * p[1], si * p[0] + ci * p[1])   # noqa: E731
    return [(back(a), back(b)) for a, b, _y in sorted(out, key=lambda r: r[2])]


def boustrophedon(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)) -> list[Track]:
    """教科書牛耕式：沿最小寬度方向鋪條帶，依累積長度**平均切成 n 段**給各機。

    這是最常見也最難打敗的基準，不是稻草人：方向取最小寬度（條帶數最少）、
    條帶交替往返（沒有多餘回頭）、分配依長度平衡。
    """
    segs = []
    for p in polys:
        segs += _strips(p, pitch_m, min_width_angle(p))
    if not segs:
        return []
    lens = [math.dist(a, b) for a, b in segs]
    tot, share = sum(lens), sum(lens) / max(n_uav, 1)
    tracks, cur, acc, flip = [], [start], 0.0, False
    for (a, b), L in zip(segs, lens):
        p0, p1 = (b, a) if flip else (a, b)
        cur += [p0, p1]
        flip = not flip
        acc += L
        if acc >= share and len(tracks) < n_uav - 1:
            cur.append(start)
            tracks.append(cur)
            cur, acc, flip = [start], 0.0, False
    cur.append(start)
    tracks.append(cur)
    return [t for t in tracks if len(t) >= 2]


BASELINES = {"boustrophedon": boustrophedon}


# ── 自檢 ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    poly = [rect_xy(0, 0, 2500, 2000)]
    alt, hfov = 140.0, 60.0
    swath = 2 * alt * math.tan(math.radians(hfov) / 2)
    pitch = swath * 0.7
    print(f"作業區 20 km²  刈幅 {swath:.1f} m  條帶間距 {pitch:.1f} m  n=8\n")
    print(f"{'方法':16s}{'台':>3s}{'總 km':>8s}{'最長 km':>9s}{'覆蓋':>8s}"
          f"{'開銷':>7s}{'重複':>7s}{'不均':>7s}{'掃描率 km²/h':>13s}{'區外浪費':>9s}")

    def show(name, tracks):
        m = metrics(poly, tracks, swath)
        print(f"{name:16s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['max_km']:9.1f}"
              f"{m['coverage'] * 100:7.1f}%{m['overhead'] or 0:7.3f}"
              f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
              f"{m['scan_rate']:13.2f}{(m['waste'] or 0) * 100:8.1f}%")

    show("boustrophedon", boustrophedon(poly, 8, swath_m=swath, pitch_m=pitch))
    t, ms, err, _sig = run_planner(False, poly, 8, 0, alt=alt, pitch=pitch, overlap=0.0)
    show("DCCPP", t)
    t, ms, err, _sig = run_planner(True, poly, 8, 0, alt=alt, pitch=pitch, overlap=0.0)
    show("HARVEST", t)


# ══════════════════════════════════════════════════════════════════════
#  禁航區（NFZ）—— 追加，不動上面任何既有介面
# ══════════════════════════════════════════════════════════════════════
#  ⚠⚠⚠ 兩條路徑在**不同層**做避繞，比較時必須講清楚：
#
#    舊 DCCPP  規劃器**完全不知道** NFZ（`core/dccpp/` 全檔 grep 不到
#              nfz / fence / no_fly）。避繞是 GUI 的**事後修正**：
#              `ui/main_window.py:3058 _apply_nfz_correction_multirotor_latlon`
#              → `core/global_planner/nfz_planner.FixedWingNFZPlanner.correct_path`
#              （Visibility Graph + Dijkstra）。
#    HARVEST   NFZ 是**規劃時的約束**（`fence_zones` → `fence_zones_to_nofly`）。
#
#  所以公平的比法是「事後繞行 vs 約束內規劃」，而不是「有沒有避繞」。
#  下面的 `nfz_correct()` 就是把 GUI 那一層抽出來，讓舊 DCCPP 也能在
#  headless 下走完整條管線。

def make_fence_zones(nfz_polys):
    """平面多邊形 → HARVEST 吃的 FenceZone（NFZ 類別）。"""
    from mission.fence_zone import FenceZone, ZoneCategory, ZoneShape
    return [FenceZone(name=f"NFZ{i + 1}", category=ZoneCategory.NFZ,
                      shape=ZoneShape.POLYGON,
                      vertices=[ll(x, y) for x, y in p.exterior.coords[:-1]],
                      alt_min_m=0.0, alt_max_m=500.0)
            for i, p in enumerate(nfz_polys)]


def nfz_correct(tracks, nfz_polys, *, buffer_m: float = 5.0):
    """把 GUI 的事後繞行套到航跡上。回傳 (新航跡, 增加公尺, 被繞航段數)。"""
    from core.base.fixed_wing_constraints import FixedWingConstraints
    from core.global_planner.nfz_planner import FixedWingNFZPlanner
    v = 10.0
    tan_phi = v ** 2 / (9.81 * max(buffer_m, 0.5))
    con = FixedWingConstraints(cruise_airspeed_mps=v,
                               max_bank_angle_deg=min(80.0, math.degrees(math.atan(tan_phi))),
                               stall_speed_mps=4.0, safety_factor=1.0)
    out, added, reroutes = [], 0.0, 0
    for t in tracks:
        pl = FixedWingNFZPlanner(con, buffer_factor=1.0)
        for i, p in enumerate(nfz_polys):
            pl.add_polygon_nfz([(x, y) for x, y in p.exterior.coords[:-1]],
                               f"NFZ{i + 1}", coord_type="metric")
        r = pl.correct_path(t)
        out.append(list(r.corrected_path) if r.corrected_path else list(t))
        added += float(r.added_length or 0.0)
        reroutes += int(r.segments_rerouted or 0)
    return out, added, reroutes


def nfz_metrics(polys, tracks, swath_m, nfz_polys, speed_mps: float = SPEED_REF) -> dict:
    """含禁航區的指標。

    ⚠ 覆蓋率的分母改成 **作業區扣掉 NFZ**（禁航區本來就不該被掃）。
    新增三個欄位：
      ``nfz_track_km``   航跡落在 NFZ 內的長度（>0 就是闖入）
      ``nfz_swept_km2``  掃過的面積落在 NFZ 內
      ``nfz_ok``         航跡完全沒進 NFZ
    """
    tracks = [[p for i, p in enumerate(t) if i == 0 or p != t[i - 1]] for t in tracks]
    tracks = [t for t in tracks if len(t) >= 2]
    nfz = unary_union(nfz_polys) if nfz_polys else None
    target = unary_union(polys)
    if nfz is not None:
        target = target.difference(nfz)
    m = metrics([target] if not target.is_empty else [], tracks, swath_m, speed_mps)
    if nfz is None or not tracks:
        m.update(nfz_track_km=0.0, nfz_swept_km2=0.0, nfz_ok=True)
        return m
    inside = sum(LineString(t).intersection(nfz).length for t in tracks)
    swept = unary_union([LineString(t).buffer(swath_m / 2.0, quad_segs=8) for t in tracks])
    m.update(nfz_track_km=round(inside / 1000.0, 3),
             nfz_swept_km2=round(swept.intersection(nfz).area / 1e6, 4),
             nfz_ok=bool(inside < 1.0))
    return m


def run_planner_nfz(use_hv: bool, polys, n_a, n_b, *, alt, pitch, overlap, nfz_polys,
                    post_correct: bool | None = None, **kw):
    """跑一個規劃器並套上該路徑真正會用的 NFZ 處理。

    `post_correct` 不給時：HARVEST 走約束內規劃（False），舊 DCCPP 走事後繞行（True）。
    """
    if post_correct is None:
        post_correct = not use_hv
    import logging
    logging.disable(logging.CRITICAL)
    from mission.swarm_coordinator import SwarmCoordinator
    drones, cls = _drones(n_a, n_b, pitch)
    c = SwarmCoordinator()
    if use_hv:
        c.use_harvest = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME_LL
        c.hv_wind = kw.get("wind", (5.0, 270.0))
        c.hv_fence_zones = make_fence_zones(nfz_polys)
        c.harvest_params = {"hfov_deg": kw.get("hfov", 60.0), "vfov_deg": kw.get("vfov", 45.0),
                       "hv_c2_range_km": 25.0, "hv_waiver_max_agl_m": 200.0,
                       "hv_dem_margin_m": 3000.0}
    try:
        res = c.plan_coverage_dccpp(areas=polys_to_areas(polys), drones=drones,
                                    coverage_width_m=pitch, overlap_rate=overlap,
                                    coverage_altitude=alt, mounting_angle_deg=0.0)
    except Exception as e:                                # noqa: BLE001
        return [], f"{type(e).__name__}: {e}", 0.0, 0
    tracks = []
    for p in (res.get("assembled_paths") or {}).values():
        t = [xy(w.lat, w.lon) for w in getattr(p, "waypoints", [])]
        if len(t) >= 2:
            tracks.append(t)
    if post_correct and tracks and nfz_polys:
        tracks, added, reroutes = nfz_correct(tracks, nfz_polys)
        return tracks, None, added, reroutes
    return tracks, None, 0.0, 0
