"""(c) 那 0.4% 缺口的最小重現：`python scripts/harvest_c_gap_probe.py`

問的問題
--------
`scripts/harvest_vs_dccpp_figure.py` 的 (c) 格（單一 5×4 km、4 Pioneer + 4 HERO、
140 m AGL、pitch 113.16、overlap 0）HARVEST 量到 **99.56%**。那 0.44% 在哪裡、
是什麼性質？

答案（本腳本會把三件事都印出來）
--------------------------------
1. **不是量測假象。** 三把互相獨立的尺一致：shapely 緩衝 quad_segs=8 → 99.5607%、
   quad_segs=64 → 99.5607%（差 8.3 m²）、格網法 `core/recon/coverage_verifier`
   5 m 格 → 99.5597%。
2. **不是邊界效應、也不是子區接縫。** 缺口 2 塊共 87 859 m²；最大那塊
   82 624 m² 距作業區邊界最近 **320.3 m**、距最近的機隊切區線最近 **110.7 m**。
   兩塊都整個落在子區 id=3（y ∈ [−1500,−1000]）**內部**。
3. **是規劃後的縮點步驟把一條作業條帶的遠端端點弄丟了。**
   `hv_reduce_waypoints=False` ⇒ 覆蓋率 **100.0000%、缺口 0 m²**。

機制（三行程式碼接起來）
------------------------
* `hv_dccpp/deconflict.py:934` 去衝突層插入延遲盤旋時，ORBIT 航點是**條帶終點的
  複本**（`replace(base, phase=Phase.ORBIT, arc=(cx, cy, R, 1))`）。
* `hv_dccpp/ardupilot.py:394` `keep[-1] = False  # 終點由下一 run 首點供給`
  ⇒ 直線 run（作業條帶）刻意不輸出自己的終點，指望上面那個複本補。
* `hv_dccpp/ardupilot.py:361` `p.x, p.y = cx, cy`
  ⇒ 但整圈盤旋被轉成 loiter 指令時，那個複本的座標被**改成圓心**。

於是上傳的任務序列變成
``WP(條帶西端) → NAV_LOITER_TIME(圓心，偏 r_orb) → WP(條帶東端)``：
飛機從條帶起點就朝圓心飛，5 126 m 的條帶被斜掉 **120.00 m**（= r_orb），
缺口寬度由 0 線性長到 ``2s − w = 2(113.54) − 161.66 = 65.42 m``。

⚠ 這條「盤旋補終點」的不變式要用**相鄰性**驗，不能只問「終點座標有沒有出現在
輸出裡」——它確實出現了，但在盤旋**之後**（飛歪再回來），照樣漏掃。第一版守衛就是
這樣印出假綠燈（0 件違規）。

⚠ 劑量反應：四格裡只有帶「作業條帶後緊接延遲盤旋」的兩格掉覆蓋率
（a 0 個盤旋 → 100.00000%；b 0 個 → 100.00000%；c 1 個 → 99.56074%；
d 1 個 → 99.98802%）。(d) 在原疊圖上印成 100.0% 是 ``%5.1f`` 四捨五入的結果，
不是真的 100%。
"""
from __future__ import annotations

import logging
import math
import sys
from pathlib import Path

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
logging.disable(logging.CRITICAL)

import hv_dccpp.ardupilot as _AP                                     # noqa: E402
from hv_dccpp.ardupilot import _runs                                 # noqa: E402

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
PERF = {"A": (21.5, 200.0), "B": (19.0, 120.0)}
SWATH = 2.0 * 140.0 * math.tan(math.radians(60.0) / 2.0)             # 161.66 m

_REC: list[tuple] = []
_ORIG = _AP.reduce_waypoints


def _spy(wps, spec, cfg, r_acc, *a, **k):
    pts = _ORIG(wps, spec, cfg, r_acc, *a, **k)
    _REC.append((wps, pts))
    return pts


def _xy(lat, lon):
    return ((lon - HOME[1]) * _M_LON, (lat - HOME[0]) * _M_LAT)


def _ll(east, north):
    return (HOME[0] + north / _M_LAT, HOME[1] + east / _M_LON)


def _rect(cx, cy, he, hn):
    return [_ll(cx - he, cy - hn), _ll(cx + he, cy - hn),
            _ll(cx + he, cy + hn), _ll(cx - he, cy + hn)]


def _plan(reduce_wp: bool):
    """跑 (c)，回傳 (每機航跡[平面], 覆蓋率, 未覆蓋幾何)。"""
    from mission.swarm_coordinator import SwarmCoordinator
    _REC.clear()
    fleet = [("A", *PERF["A"])] * 4 + [("B", *PERF["B"])] * 4
    drones = [{"drone_id": i + 1, "name": f"{c}{i + 1}", "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 113.16} for i, (c, v, r) in enumerate(fleet)]
    cls = {i + 1: c for i, (c, _v, _r) in enumerate(fleet)}
    co = SwarmCoordinator()
    co.use_harvest = True
    co.hv_class_of = lambda d: cls[int(d["drone_id"])]
    co.hv_home, co.hv_wind, co.hv_fence_zones = HOME, (5.0, 270.0), []
    co.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                         "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0,
                         "hv_reduce_waypoints": reduce_wp}
    areas = [{"area_id": 1, "priority": 1.0, "polygon": _rect(0, 0, 2500, 2000)}]
    res = co.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=113.16,
                                 overlap_rate=0.0, coverage_altitude=140.0,
                                 mounting_angle_deg=0.0)
    tracks = {}
    for k, p in (res.get("assembled_paths") or {}).items():
        pts = [_xy(w.lat, w.lon) for w in getattr(p, "waypoints", [])]
        pts = [q for i, q in enumerate(pts) if i == 0 or q != pts[i - 1]]
        if len(pts) >= 2:
            tracks[k] = pts
    poly = Polygon([_xy(la, lo) for la, lo in areas[0]["polygon"]])
    swept = unary_union([LineString(t).buffer(SWATH / 2.0, quad_segs=64)
                         for t in tracks.values()])
    return tracks, poly.intersection(swept).area / poly.area, poly.difference(swept)


def _invariant_violations() -> list[str]:
    """`ardupilot.py:394` 依賴「直線 run 的終點由下一 run 的**首個輸出點**補上」。

    ⚠ 必須驗相鄰性：只問「終點座標有沒有出現在輸出裡」會得到 0 件違規的假綠燈。
    """
    bad = []
    for ac, (wps, pts) in enumerate(_REC):
        for i0, i1, kind in _runs(wps):
            if kind != "line" or wps[i0].phase.name != "OPERATE":
                continue
            i_end = min(i1 + 1, len(wps) - 1)
            if i_end == i0:
                continue
            js = [j for j, p in enumerate(pts) if i0 <= p.src <= i1]
            nxt = pts[max(js) + 1] if js and max(js) + 1 < len(pts) else None
            ex, ey = wps[i_end].x_m, wps[i_end].y_m
            if nxt is None or math.hypot(nxt.x - ex, nxt.y - ey) >= 1e-6:
                off = "無後續點" if nxt is None else f"{math.hypot(nxt.x - ex, nxt.y - ey):.2f} m"
                got = "(無)" if nxt is None else f"({nxt.x:.2f},{nxt.y:.2f}) kind={nxt.kind}"
                bad.append(f"ac{ac} OPERATE 條帶真終點 ({ex:.2f},{ey:.2f}) "
                           f"→ 下一個輸出點是 {got}，偏移 {off}")
    return bad


def main() -> int:
    _AP.reduce_waypoints = _spy
    import hv_dccpp.integration.aeroplan_adapter as _AD
    _AD.reduce_waypoints = _spy

    print(f"刈幅 2·h·tan(HFOV/2) = {SWATH:.2f} m @ 140 m AGL\n")
    for rw in (True, False):
        tracks, cov, gap = _plan(rw)
        parts = sorted([g for g in getattr(gap, "geoms", [gap]) if not g.is_empty],
                       key=lambda g: -g.area)
        print(f"hv_reduce_waypoints={rw!s:5s}  覆蓋 {cov * 100:9.5f} %   "
              f"缺口 {gap.area:10.1f} m²（{len(parts)} 塊）  "
              f"航點總數 {sum(len(t) for t in tracks.values())}")
        for g in parts:
            x0, y0, x1, y1 = g.bounds
            print(f"      塊 {g.area:9.1f} m²  x[{x0:9.1f},{x1:9.1f}] "
                  f"y[{y0:9.1f},{y1:9.1f}]  高 {y1 - y0:6.2f} m")
        for line in _invariant_violations():
            print(f"      ★ 不變式違反：{line}")
        print()

    print("判讀：兩者只差 `reduce_waypoints`，規劃器產出的覆蓋是完整的；")
    print("      缺口高度 2s − w = 2(113.54) − 161.66 = 65.42 m，偏移 = r_orb = 120.00 m。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
