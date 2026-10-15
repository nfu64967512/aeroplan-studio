"""把舊 DCCPP 當參考實作，與 HARVEST 逐項比較。

⚠⚠ 命名要小心：**兩個規劃器都不是 ground truth**。
    真正的判準有兩個，都不是規劃器自己說的：
      (1) 幾何覆蓋率 —— 拿產出的**作業段**航跡緩衝相機刈幅，與作業區求交集面積比。
      (2) 平台可行性 —— 每機航程與**平台參數表**的可用航程比。
    舊 DCCPP 沒有能量模型也沒有 4D 去衝突，所以它只能當「覆蓋幾何的參考實作」，
    不能當「這份任務飛得起來」的裁判。凡是它贏的地方都要先過 (2)。

用法：
    from harness import compare, rect, HOME
    r = compare(areas, n_a=4, n_b=4, altitude=140.0)
    print(r['legacy']['assigned'], r['hv']['assigned'])
"""
from __future__ import annotations

import logging
import math
import sys

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

logging.disable(logging.CRITICAL)

from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))

# 平台參數表的可用航程（km）。用區間下緣，這是判「飛不飛得完」的硬界。
USABLE_KM = {"A": 133.0, "B": 79.0}
PERF = {"A": (21.5, 200.0), "B": (19.0, 120.0)}      # (巡航 m/s, 規劃轉彎半徑 m)


def ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def rect(cx: float, cy: float, half_e: float, half_n: float):
    return [ll(cx - half_e, cy - half_n), ll(cx + half_e, cy - half_n),
            ll(cx + half_e, cy + half_n), ll(cx - half_e, cy + half_n)]


def _xy(lat: float, lon: float) -> tuple[float, float]:
    return ((lon - HOME[1]) * _M_LON, (lat - HOME[0]) * _M_LAT)


def _drones(n_a: int, n_b: int, width: float):
    fleet = [("A", *PERF["A"])] * n_a + [("B", *PERF["B"])] * n_b
    return ([{"drone_id": i + 1, "name": f"{c}{i + 1}", "position": HOME, "speed": v,
              "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
              "fov_width": width} for i, (c, v, r) in enumerate(fleet)],
            {i + 1: c for i, (c, _v, _r) in enumerate(fleet)})


def _operation_segments(built) -> list[list[tuple[float, float]]]:
    """整條航跡（**不做任何航段篩選**）。

    ⚠⚠⚠ 這裡踩過兩次，兩次都**剛好偏向「HV 比較差」**這個我們想檢定的假說：

    1. 一開始只留 `OPERATION`。但兩個規劃器的標籤慣例不同：同一份 4 區任務，
       舊 DCCPP 把測繪條帶標成 OPERATION，HV 只有 **4–7 個點**是 OPERATION、
       其餘 31–67 個點標成 TRANSFER ⇒ HV 覆蓋率被量成 **0%**。
    2. 改成「排除 TAKEOFF/LANDING」還是錯：HV 每條航路末端有 10–15 個
       `LANDING` 點，而那一段**還在作業區裡**（是最後一條條帶的尾巴）
       ⇒ HV 被量成 75.9–95.9%，但同一份 plan 用全部航點量是 **100%**。

    判別法：拿一份**已知答案**的來對。HV4SITL3 匯出的 `plan/*.waypoints` 與
    實飛航跡兩者都是 100%，任何量法量出來不是 100% 就是量法錯了。
    對舊 DCCPP 而言全算與只算 OPERATION 得到相同的 79.16%，所以不篩選對它無害。
    """
    pts = [_xy(w.lat, w.lon) for w in built.waypoints]
    pts = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
    return [pts] if len(pts) >= 2 else []


def _coverage(areas, paths, swath_m: float) -> list[float]:
    """判準 (1)：作業段航跡緩衝 swath/2 之後，各作業區被蓋掉的面積比。"""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    bufs = []
    for built in paths:
        for run in _operation_segments(built):
            run = [p for i, p in enumerate(run) if i == 0 or p != run[i - 1]]
            if len(run) >= 2:
                bufs.append(LineString(run).buffer(swath_m / 2.0, quad_segs=8))
    swept = unary_union(bufs) if bufs else None
    out = []
    for a in areas:
        poly = Polygon([_xy(la, lo) for la, lo in a["polygon"]])
        if not poly.is_valid:
            poly = poly.buffer(0)
        inter = poly.intersection(swept).area if swept is not None else 0.0
        out.append(round(inter / poly.area, 4) if poly.area else None)
    return out


def _one(use_hv: bool, areas, n_a, n_b, *, altitude, width, overlap, wind,
         hfov, vfov, swath_m):
    drones, cls = _drones(n_a, n_b, width)
    c = SwarmCoordinator()
    if use_hv:
        c.use_harvest = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME
        c.hv_wind = wind
        c.hv_fence_zones = []
        c.harvest_params = {"hfov_deg": hfov, "vfov_deg": vfov, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    err = None
    try:
        res = c.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=width,
                                    overlap_rate=overlap, coverage_altitude=altitude,
                                    mounting_angle_deg=0.0)
    except Exception as e:                       # noqa: BLE001
        return {"error": f"{type(e).__name__}: {e}", "assigned": 0, "total_km": 0.0,
                "makespan_s": None, "coverage": [0.0] * len(areas), "per_uav_km": {},
                "over_range": [], "max_km": 0.0}
    ap = res.get("assembled_paths") or {}
    paths = [p for p in ap.values() if getattr(p, "waypoints", None)]
    per = {}
    for did, p in ap.items():
        km = float(getattr(p, "total_length_m", 0.0)) / 1000.0
        if km > 0:
            per[f"{cls[int(did)]}{did}"] = round(km, 2)
    # 判準 (2)：每機航程 vs 該構型的可用航程
    over = [u for u, km in per.items() if km > USABLE_KM[u[0]]]
    return {"error": err, "assigned": len(per),
            "total_km": round(sum(per.values()), 2),
            "max_km": round(max(per.values()), 2) if per else 0.0,
            "makespan_s": (round(float(res.get("total_makespan") or 0.0), 1)),
            "coverage": _coverage(areas, paths, swath_m),
            "per_uav_km": per, "over_range": over}


def compare(areas, *, n_a=4, n_b=4, altitude=140.0, width=150.0, overlap=0.30,
            wind=(5.0, 270.0), hfov=60.0, vfov=45.0):
    """同一組輸入餵給兩個規劃器，回傳可直接比較的量。

    `swath_m` 兩邊用**同一個物理刈幅** 2·h·tan(HFOV/2)，因為覆蓋要問的是
    「同一台相機飛這個圖案蓋不蓋得到」，不是各自宣稱的條帶間距。
    """
    swath = 2.0 * altitude * math.tan(math.radians(hfov) / 2.0)
    kw = dict(altitude=altitude, width=width, overlap=overlap, wind=wind,
              hfov=hfov, vfov=vfov, swath_m=swath)
    return {"swath_m": round(swath, 1), "n_a": n_a, "n_b": n_b, "n_areas": len(areas),
            "legacy": _one(False, areas, n_a, n_b, **kw),
            "hv": _one(True, areas, n_a, n_b, **kw)}


def line(tag: str, r: dict) -> str:
    lg, hv = r["legacy"], r["hv"]
    f = lambda d: (f"{d['assigned']:2d}台 {d['total_km']:7.1f}km "                # noqa: E731
                   f"cov{min(d['coverage']) * 100 if d['coverage'] else 0:6.1f}% "
                   f"ms{(d['makespan_s'] or 0):7.0f}s"
                   + (f" ✗超程{d['over_range']}" if d["over_range"] else "")
                   + (f" ERR" if d["error"] else ""))
    return f"{tag:30s} 舊 {f(lg)}  |  HV {f(hv)}"
