"""航點精簡不可以把作業條帶的終點弄丟（vendored 修改 #6）。

⚠⚠⚠ 2026-09-09 實際缺陷，三行程式碼接起來的：

1. ``hv_dccpp/deconflict.py`` 去衝突插入延遲盤旋，ORBIT 航點是**條帶終點的複本**。
2. ``hv_dccpp/ardupilot.py`` 直線 run 固定 ``keep[-1] = False``（「終點由下一 run
   首點供給」），刻意不輸出自己的終點。
3. 同檔把整圈盤旋轉成 loiter 指令時 ``p.x, p.y = cx, cy`` —— 那個複本的座標被改成
   盤旋**圓心**。

⇒ 上傳序列變成 ``WP(條帶西端) → NAV_LOITER_TIME(圓心) → WP(條帶東端)``，
5 126 m 的作業條帶被斜掉 **r_orb = 120.00 m**，漏掃 **87 851 m²（0.4393%）**。
缺口高度 65.42 m 恰等於 ``2s − w``，也就是「少掃一條」的精確值。

⚠ 這**不只是量測問題**：飛控就是照上傳序列飛，所以飛機真的會飛那條斜線。
關掉 ``reduce_waypoints`` 是精確的 100.0000% ⇒ 規劃沒問題，壞在壓成航點這一步。

⚠⚠ **判準必須驗「相鄰性」**，不是「終點座標有沒有出現在輸出裡」。
原始調查的第一版守衛就是問後者，報 **0 件違規** —— 因為它確實出現了，只是在盤旋
**之後**（先飛歪 5 km 再回來）。那是假綠燈。
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

from hv_dccpp.mission import Phase                                # noqa: E402


def _export(plan, ctx, cfg, uav_id):
    """把一機的計畫壓成 ArduPilot 導航點序列（與轉接層同一組參數）。"""
    from hv_dccpp.ardupilot import accept_radius_m, reduce_waypoints
    up = plan.plans[uav_id]
    spec = ctx.specs[up.cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_acc = accept_radius_m(spec.r_eff(w_ref), None)
    return reduce_waypoints(up.waypoints, spec, cfg, r_acc)


def _orbit_indices(pts):
    return [k for k, p in enumerate(pts) if str(getattr(p, "kind", "")).startswith("loiter")]


def _plan_with_orbits():
    """5×4 km 單一大區 + 4 Pioneer + 4 HERO —— 這組**會**產生延遲盤旋。

    ⚠ 不要用 `gui_legal_120_split_resched`：實測那個情境一個整圈盤旋都沒有，
    盤旋相關的斷言在它上面等於什麼都沒驗（第一版就是這樣紅在前置斷言上）。
    ⚠ 參數必須與 `scripts/harvest_vs_dccpp_figure.py` 的 (c) 格一致 ——
    自己拼一組會踩到 `no_coverable_area` NO_GO（第二版就是這樣紅的）。
    """
    import logging

    from mission.swarm_coordinator import SwarmCoordinator
    lat, lon = 23.95, 120.90
    m_lat = 111132.0
    m_lon = 111412.0 * math.cos(math.radians(lat))
    poly = [(lat - 2000 / m_lat, lon - 2500 / m_lon), (lat - 2000 / m_lat, lon + 2500 / m_lon),
            (lat + 2000 / m_lat, lon + 2500 / m_lon), (lat + 2000 / m_lat, lon - 2500 / m_lon)]
    fleet = ["A"] * 4 + ["B"] * 4
    drones = [{"drone_id": i + 1, "name": f"{c}{i+1}", "position": (lat, lon),
               "speed": 21.5 if c == "A" else 19.0, "heading": 0.0,
               "turn_radius": 200.0 if c == "A" else 120.0, "vehicle_type": "vtol",
               "fov_width": 113.16} for i, c in enumerate(fleet)]
    cls = {i + 1: c for i, c in enumerate(fleet)}
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: cls[int(d["drone_id"])]
    c.hv_home = (lat, lon)
    c.hv_wind = (5.0, 270.0)
    c.hv_fence_zones = []
    c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                        "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    logging.disable(logging.WARNING)
    try:
        c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0, "polygon": poly}],
                              drones=drones, coverage_width_m=113.16, overlap_rate=0.0,
                              coverage_altitude=140.0, mounting_angle_deg=0.0)
    finally:
        logging.disable(logging.NOTSET)
    hv = c.hv_coordinator
    return hv.last_plan, hv.last_ctx, hv.cfg


def _dist_to_polyline(px, py, pts) -> float:
    """點到折線的最短距離。⚠ 不是「到最近頂點」—— 精簡會拿掉**共線**的頂點，
    那在幾何上無損，用頂點距離會誤判成缺陷（第一版就是這樣，量到 24.93 m）。"""
    best = float("inf")
    for a, b in zip(pts, pts[1:]):
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best

def test_a_relocated_loiter_is_preceded_by_its_true_entry_point():
    """⭐ 不變式：每個「座標被改成圓心」的 loiter 點，前一點必須是它**真正的**進入點。

    ⚠ 用相鄰性驗，不是用「座標出現過」驗。
    """
    plan, ctx, cfg = _plan_with_orbits()

    checked = 0
    for uid in plan.plans:
        pts = _export(plan, ctx, cfg, uid)
        for k in _orbit_indices(pts):
            if k == 0:
                continue
            src = getattr(pts[k], "src", None)
            if src is None:
                continue
            w = plan.plans[uid].waypoints[int(src)]
            prev = pts[k - 1]
            d = math.hypot(prev.x - float(w.x_m), prev.y - float(w.y_m))
            assert d < 1.0, (
                f"{uid} 第 {k} 個點是 loiter（座標＝圓心），但前一點離它真正的進入點 "
                f"{d:.2f} m —— 進入點被弄丟了，飛機會直接斜飛過去")
            checked += 1
    # ⚠ 前置斷言：這個情境**真的**有 loiter 點，否則上面等於什麼都沒驗
    assert checked > 0, "這個情境沒有任何整圈盤旋 ⇒ 這條測試量不到東西，要換情境"


def test_operate_strip_endpoint_survives_reduction():
    """⭐⭐ 每一條 OPERATE 條帶的兩個端點，都必須出現在精簡後的序列裡（**位置**要對）。"""
    plan, ctx, cfg = _plan_with_orbits()

    n_ends = 0
    for uid, p in plan.plans.items():
        pts = _export(plan, ctx, cfg, uid)
        xy = [(q.x, q.y) for q in pts]
        # 每段連續的 OPERATE 的頭尾
        runs, cur = [], []
        for w in p.waypoints:
            if w.phase is Phase.OPERATE:
                cur.append(w)
            elif cur:
                runs.append(cur); cur = []
        if cur:
            runs.append(cur)
        for r in runs:
            for w in (r[0], r[-1]):
                d = _dist_to_polyline(float(w.x_m), float(w.y_m), xy)
                assert d < 1.0, (f"{uid} 的作業條帶端點 ({w.x_m:.1f},{w.y_m:.1f}) "
                                 f"離精簡後的**折線** {d:.2f} m —— 條帶被斜掉了")
                n_ends += 1
    assert n_ends > 0, "沒有任何 OPERATE 條帶 ⇒ 這條測試量不到東西"


def test_reduction_does_not_lose_coverage():
    """⭐⭐⭐ 值層級：精簡前後的覆蓋率必須一致。

    ⚠ 這是唯一能抓到「端點被弄丟」造成的**實質**後果的斷言。
    缺陷存在時實測 99.56074% vs 100.00000%（缺口 87 851 m²）。
    """
    import logging

    from core.recon import verify_coverage
    from mission.swarm_coordinator import SwarmCoordinator

    lat, lon = 25.0750, 121.2355
    m_lat = 111132.0
    m_lon = 111412.0 * math.cos(math.radians(lat))
    poly = [(lat - 650 / m_lat, lon - 700 / m_lon), (lat - 650 / m_lat, lon + 700 / m_lon),
            (lat + 650 / m_lat, lon + 700 / m_lon), (lat + 650 / m_lat, lon - 700 / m_lon)]
    drones = [{"drone_id": i + 1, "name": f"U{i+1}", "hv_class": "A" if i == 0 else "B",
               "position": (lat, lon), "speed": 21.5 if i == 0 else 19.0,
               "turn_radius": 200.0 if i == 0 else 120.0, "vehicle_type": "vtol",
               "fov_width": 150.0} for i in range(4)]
    swath = 2 * 120.0 * math.tan(math.radians(60.0) / 2)

    def cov(reduce_wp: bool) -> float:
        c = SwarmCoordinator()
        c.use_harvest = True
        c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_waiver_max_agl_m": None,
                            "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0},
                            "hv_dh_band_min_m": 15.0, "target_gsd": 0.05,
                            "hv_reduce_waypoints": reduce_wp}
        logging.disable(logging.WARNING)
        try:
            r = c.plan_coverage_dccpp(areas=[{"area_id": 1, "priority": 1.0, "polygon": poly}],
                                      drones=drones, coverage_width_m=150.0, overlap_rate=0.30,
                                      coverage_altitude=120.0, mounting_angle_deg=0.0)
        finally:
            logging.disable(logging.NOTSET)
        aps = r.get("assembled_paths") or {}
        paths = {int(u): [(float(w.lat), float(w.lon)) for w in ap.waypoints] for u, ap in aps.items()}
        return verify_coverage(paths, poly, swath, cell_size_m=20.0).coverage_pct

    full, reduced = cov(False), cov(True)
    assert full > 99.9, f"未精簡本身就不完整（{full:.4f}%）⇒ 前提失效，這條測不到精簡的影響"
    assert reduced == pytest.approx(full, abs=0.05), (
        f"精簡讓覆蓋率從 {full:.5f}% 掉到 {reduced:.5f}% —— 端點在精簡時被弄丟了")
