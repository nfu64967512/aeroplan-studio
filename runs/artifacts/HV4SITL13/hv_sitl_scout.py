"""掃出「四架都有工作、而且 makespan 飛得完」的 SITL 情境幾何。

規劃器**可能只用機隊的一部分，而且完全不會有訊息**（用到幾台是胞的數目決定的，
不是機隊大小）。所以起 SITL 之前先在這裡掃過，不要用真的 SITL 去試錯。

    python scripts/hv_sitl_scout.py            # 全掃
    python scripts/hv_sitl_scout.py 1a3b       # 只掃某一組
"""
from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.hv_sitl_scenarios import HOME, fleet_of, rect  # noqa: E402

MAX_MAKESPAN_S = 2600.0          # speedup 5 ⇒ 牆鐘 8.7 分鐘
logging.disable(logging.CRITICAL)


def plan_gui(n_a: int, n_b: int, areas, alt: float, *, hfov=60.0, vfov=45.0,
             width=150.0, overlap=0.30, uniform=True):
    """走 AeroPlan 介面那一條路徑跑一次規劃，回傳 (hv_coordinator, 秒)。"""
    from mission.swarm_coordinator import SwarmCoordinator
    fleet = fleet_of(n_a, n_b)
    cls_of = {i + 1: c for i, (_n, c, _s, _v, _r) in enumerate(fleet)}
    drones = [{"drone_id": i + 1, "name": n, "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": width}
              for i, (n, _c, _s, v, r) in enumerate(fleet)]
    c = SwarmCoordinator()
    c.use_hv_dccpp = True
    c.hv_uniform_sensor = uniform
    c.hv_class_of = lambda d: cls_of[int(d["drone_id"])]
    c.hv_home = HOME
    c.hv_wind = (5.0, 270.0)
    c.hv_fence_zones = []
    c.hv_params = {"hfov_deg": hfov, "vfov_deg": vfov, "hv_c2_range_km": 25.0,
                   "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    t0 = time.perf_counter()
    c.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=width,
                          overlap_rate=overlap, coverage_altitude=alt,
                          mounting_angle_deg=0.0)
    return c.hv_coordinator, time.perf_counter() - t0


def _grid(n: int, half_e: float, half_n: float, pitch: float, *, row=0.0):
    """n 個並排的矩形，中心間距 pitch。"""
    x0 = -pitch * (n - 1) / 2.0
    return [{"area_id": i + 1, "priority": 1.0,
             "polygon": rect(x0 + i * pitch, row, half_e, half_n)}
            for i in range(n)]


CANDIDATES = {
    # key: (n_a, n_b, areas, altitude)
    "1a3b/4區小": (1, 3, _grid(4, 260, 300, 1500), 140.0),
    "1a3b/4區中": (1, 3, _grid(4, 320, 380, 1700), 140.0),
    "1a3b/3區+1遠": (1, 3, _grid(3, 300, 350, 1600)
                     + [{"area_id": 4, "priority": 1.0, "polygon": rect(0, 2600, 320, 300)}], 140.0),
    "1a3b/2區大": (1, 3, _grid(2, 700, 600, 2200), 140.0),
    "2a2b/4區小": (2, 2, _grid(4, 260, 300, 1500), 150.0),
    "2a2b/4區中": (2, 2, _grid(4, 320, 380, 1700), 150.0),
    "2a2b/6區": (2, 2, _grid(6, 220, 260, 1200), 150.0),
    "2a2b/2x2": (2, 2, _grid(2, 300, 350, 1600) + [
        {"area_id": 3, "priority": 1.0, "polygon": rect(-800, 2000, 300, 350)},
        {"area_id": 4, "priority": 1.0, "polygon": rect(800, 2000, 300, 350)}], 150.0),
    "3a1b/4區中": (3, 1, _grid(4, 320, 380, 1700), 150.0),
}


def main() -> int:
    only = sys.argv[1] if len(sys.argv) > 1 else ""
    print(f"{'情境':18s}{'秒':>6s}{'makespan':>10s}{'胞':>4s}{'未覆蓋':>7s}  分配")
    good = []
    for key, (n_a, n_b, areas, alt) in CANDIDATES.items():
        if only and only not in key:
            continue
        try:
            hv, dt = plan_gui(n_a, n_b, areas, alt)
        except Exception as e:                       # noqa: BLE001
            print(f"{key:18s}  例外 {type(e).__name__}: {e}")
            continue
        plan = hv.last_plan
        got = {u: len(p.strips) for u, p in sorted(plan.plans.items())}
        n_fleet = n_a + n_b
        all_work = len(got) == n_fleet and all(got.values())
        flag = "OK " if all_work and plan.makespan_s <= MAX_MAKESPAN_S else "   "
        print(f"{key:18s}{dt:6.1f}{plan.makespan_s:10.1f}{len(plan.cells):4d}"
              f"{len(plan.uncovered):7d}  {flag}{got}")
        if all_work and plan.makespan_s <= MAX_MAKESPAN_S and not plan.uncovered:
            e = {u: round(p.energy_margin_wh, 1) for u, p in sorted(plan.plans.items())}
            d = {u: round(p.distance_m / 1000, 2) for u, p in sorted(plan.plans.items())}
            print(f"{'':18s}      能量餘裕 {e}")
            print(f"{'':18s}      航程 km   {d}")
            good.append(key)
    print("\n可以拿去飛的:", good or "（沒有）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
