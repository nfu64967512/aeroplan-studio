"""run_mock_autopilot_loop.py — 與 tools/mock_autopilot.py 之閉迴路示範（規劃 → 上傳 → 飛行 → 事件 → 重規劃 → 重上傳）。

    python examples/run_mock_autopilot_loop.py [--time-scale 60] [--ports 14601,14602] [--wall-limit 150]

1. mini example → plan_mission；2. 啟動兩個 mock QuadPlane 子程序（A1、B1，各一 UDP 埠；風 5 m/s from 270 與情境一致）；
3. `mavlink_link.FleetLink.upload_plan`（任務 + rally + fence + 建議參數 PARAM_SET）→ `VehicleLink.start_mission`
   （依計畫起飛時刻錯開）；4. `FleetLink.poll` 輪詢遙測（→ dynamic.Telemetry；`detect_events` 之自動事件只記錄不處理）；
5. A1 進入第 2 條條帶時注入 ENERGY_UPDATE（遙測電量驟降）→ `dynamic.replan` → `FleetLink.replan_and_reupload`
   變更機並 MISSION_SET_CURRENT 至「現行位置對應之新任務 seq」（新舊任務錨點前航點相同）；6. 等待兩機落地；
7. 印出航跡摘要並存 figures/mock_loop_trace.png（+ figures/ardupilot/mock/{before,after}/*、mock_*.log、trace.json）。

需要 pymavlink 與 hv_dccpp.mavlink_link（缺席時腳本印出訊息並結束）。
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import subprocess
import sys
import time
import traceback

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from examples._compat import ARDU_DIR, FIG_DIR, ROOT, export_ardupilot, fmt_table, plan_summary, step  # noqa: E402

import numpy as np  # noqa: E402

from hv_dccpp import dynamic as dyn  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.mission import Phase  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.scenario import build_mini_example  # noqa: E402

DEM_STEP_M = 50.0
TERRAIN_MARGIN_M = 1500.0
MOCK = os.path.join(ROOT, "tools", "mock_autopilot.py")
E_TEL_DROP_WH = 400.0          # 注入之遙測電量（A1；計畫預測約 1200 Wh → κ 上修 → DIVERT）
INJECT_STRIP_INDEX = 1         # A1 之第 2 條條帶
INJECT_MIN_PROGRESS = 0.05


def _wp_index_at(seqmap, seq: int) -> int | None:
    """seq（含 DO_ 指令）→ 最近之前（含）NAV 項目之 waypoint 索引。"""
    if seqmap is None:
        return None
    for s in range(int(seq), -1, -1):
        e = seqmap.entry(s)
        if e is not None and e.wp_index is not None:
            return int(e.wp_index)
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--time-scale", type=float, default=60.0)
    ap.add_argument("--ports", default="14601,14602")
    ap.add_argument("--wall-limit", type=float, default=150.0)
    a = ap.parse_args(argv)
    t_all = time.perf_counter()
    mock_dir = os.path.join(ARDU_DIR, "mock")
    os.makedirs(mock_dir, exist_ok=True)

    try:
        from hv_dccpp import mavlink_link as ml
        if not ml.HAVE_PYMAVLINK:
            raise ImportError("pymavlink not installed")
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil
        mav = mavutil.mavlink
    except ImportError as exc:
        print(f"hv_dccpp.mavlink_link / pymavlink unavailable: {exc}")
        return 1

    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=DEM_STEP_M)
    ctx = step("build_context", lambda: build_context(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M))
    plan0 = step("plan_mission", lambda: plan_mission(inputs, cfg, ctx=ctx)) if ctx is not None else None
    if plan0 is None or not plan0.plans:
        print("no plan; abort")
        return 1
    print(plan_summary(plan0, ctx))
    cache = step("cache_from_plan", lambda: dyn.cache_from_plan(plan0, ctx))
    step("ardupilot.export_all(before)", lambda: export_ardupilot(plan0, ctx, cfg, os.path.join(mock_dir, "before")))

    uav_ids = sorted(plan0.plans)
    ports = dict(zip(uav_ids, [int(p) for p in a.ports.split(",")]))
    home = inputs.home
    home_msl = float(ctx.frame.enu_z_to_msl(np.zeros((1, 2)), 0.0)[0])
    w0 = inputs.wind[0]

    # ---- 2. 啟動 mock 子程序 ------------------------------------------------------
    procs: dict[str, subprocess.Popen] = {}
    logs: dict[str, object] = {}
    for k, uid in enumerate(uav_ids):
        spec = ctx.specs[plan0.plans[uid].cls]
        log = open(os.path.join(mock_dir, f"mock_{uid}.log"), "w", encoding="utf-8")
        # 電池模型改用「這一架的計畫能量剖面」，而不是 mock 的預設 800 Wh／350 W：否則 mock 的電量與計畫天生
        # 不一致，mavlink_link.detect_events 會不停送出 ENERGY_UPDATE（審查前 330 次）。
        prof = os.path.join(mock_dir, f"{uid}_energy.json")
        wps = plan0.plans[uid].waypoints
        with open(prof, "w", encoding="utf-8") as fp:
            json.dump({"t_s": [float(w.t_s) for w in wps],
                       "e_wh": [float(w.energy_remaining_wh) for w in wps]}, fp)
        model = ctx.models[uid]
        cmd = [sys.executable, MOCK, "--listen", f"udpin:127.0.0.1:{ports[uid]}",
               "--home", f"{home.lat_deg},{home.lon_deg},{home_msl},0", "--speed", f"{spec.cruise_airspeed_mps}",
               "--time-scale", f"{a.time_scale}", "--dump", os.path.join(mock_dir, f"{uid}_received.json"),
               "--duration", f"{a.wall_limit + 60}", "--sysid", str(k + 1),
               "--wind", f"{w0.speed_mps},{w0.dir_from_deg}",
               "--battery-wh", f"{model.nominal_wh or model.available_wh}", "--energy-profile", prof]
        procs[uid] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=ROOT,
                                      env={**os.environ, "PYTHONIOENCODING": "utf-8"})
        logs[uid] = log
        print(f"started mock for {uid} on udp {ports[uid]} (pid {procs[uid].pid}, sysid {k + 1}, "
              f"speed {spec.cruise_airspeed_mps} m/s, time-scale {a.time_scale})")

    plan = plan0
    plan_after = plan0
    trace: dict[str, list] = {u: [] for u in uav_ids}
    events_log: list[str] = []
    auto_events: dict[str, int] = {}
    replan_res = None
    landed: set[str] = set()
    fl: ml.FleetLink | None = None
    try:
        # ---- 3. 連線、上傳、參數、起飛 ------------------------------------------------
        def _connect():
            time.sleep(1.5)
            links = {}
            for uid in uav_ids:
                link = ml.VehicleLink(f"udpout:127.0.0.1:{ports[uid]}", timeout_s=10.0, uav_id=uid, time_source="boot")
                hb = link.wait_heartbeat(20.0)
                print(f"  {uid}: heartbeat from sysid {link.target_system} (type {hb.type}, autopilot {hb.autopilot})")
                links[uid] = link
            return ml.FleetLink(links)
        fl = step("connect (VehicleLink.wait_heartbeat)", _connect)
        if fl is None:
            return 1

        def _upload():
            reports = fl.upload_plan(plan0, ctx, cfg, out_dir=os.path.join(mock_dir, "upload"), set_params=True)
            for uid, r in reports.items():
                n_ok = sum(1 for v in (r.params or {}).values() if v)
                print(f"  {uid}: mission {'ACCEPTED' if r.mission.ok else 'FAILED'} ({r.mission.n_items} items, "
                      f"{r.mission.elapsed_s:.1f} s, retries {r.mission.retries_used}); "
                      f"rally {r.rally.n_items if r.rally else '-'} {'OK' if r.rally and r.rally.ok else ''}; "
                      f"fence {r.fence.n_items if r.fence else '-'} {'OK' if r.fence and r.fence.ok else ''}; "
                      f"params {n_ok}/{len(r.params or {})} set; seqmap {len(r.seqmap.items)} entries; "
                      f"files {[os.path.basename(f) for f in r.files]}")
            return reports
        reports = step("FleetLink.upload_plan (mission + rally + fence + PARAM_SET)", _upload)
        if not reports:
            return 1

        order = sorted(uav_ids, key=lambda u: plan0.plans[u].t_takeoff_s)
        wall0 = time.time()
        started: set[str] = set()
        airborne: set[str] = set()
        injected = False
        last_print = -10.0
        print("\n=== flight loop ===")
        while time.time() - wall0 < a.wall_limit:
            now = time.time()
            for uid in order:                      # 錯開起飛（計畫 t_takeoff / time_scale）
                if uid not in started and now - wall0 >= plan0.plans[uid].t_takeoff_s / a.time_scale:
                    link = fl[uid]
                    vt = link.read_telemetry(1.0)
                    ok = link.start_mission(1)
                    tb = (vt.time_boot_ms / 1000.0) if vt is not None else 0.0
                    link.boot_offset_s = tb - plan0.plans[uid].t_takeoff_s      # t_s ≈ 計畫時間軸（模擬秒）
                    started.add(uid)
                    print(f"[{now - wall0:6.1f}s] MISSION_START {uid}: {'ACK' if ok else 'no ack'} "
                          f"(plan t_takeoff={plan0.plans[uid].t_takeoff_s:.0f} s, mock boot t={tb:.0f} s)")
            tels, evs = fl.poll(plan, ctx, cfg, timeout_s=0.25)
            for e in evs:
                auto_events[type(e).__name__] = auto_events.get(type(e).__name__, 0) + 1
            for uid in uav_ids:
                if uid not in started or uid not in tels:
                    continue
                pt = tels[uid]
                vt = fl.last_tel[uid]
                trace[uid].append((round(pt.t_s, 1), pt.x_m, pt.y_m, pt.z_m, int(vt.mission_seq),
                                   -1 if vt.battery_pct is None else int(vt.battery_pct), int(vt.vtol_state),
                                   int(vt.landed_state), int(vt.armed), pt.current_strip_id, round(pt.strip_progress, 2)))
                if vt.armed or vt.landed_state in (mav.MAV_LANDED_STATE_IN_AIR, mav.MAV_LANDED_STATE_TAKEOFF):
                    airborne.add(uid)
                if uid in airborne and not vt.armed and vt.landed_state == mav.MAV_LANDED_STATE_ON_GROUND \
                        and uid not in landed:
                    landed.add(uid)
                    print(f"[{now - wall0:6.1f}s] {uid} LANDED at sim t={pt.t_s:.0f} s (plan t_land "
                          f"{plan.plans[uid].t_land_s if uid in plan.plans else float('nan'):.0f} s), battery {vt.battery_pct} %")
                # ---- 5. 事件注入：A1 進入第 2 條條帶 --------------------------------------
                if not injected and uid == "A1" and pt.current_strip_id is not None:
                    up = plan.plans[uid]
                    strip_ids = [s for s, _d in up.strips]
                    # 進度以位置在條帶上的投影自行計算（seqmap 之「現行 seq」為條帶末端航點，其 phase 為 TRANSFER →
                    # to_planner_telemetry 於條帶中段回報 progress=1.0）
                    prog = pt.strip_progress
                    if pt.current_strip_id in plan.strips and pt.current_strip_id in strip_ids:
                        s_ = plan.strips[pt.current_strip_id]
                        d_ = dict(up.strips)[pt.current_strip_id]
                        p0_, p1_ = (s_.pL, s_.pR) if d_ == -1 else (s_.pR, s_.pL)
                        L_ = float(np.hypot(*(p1_ - p0_)))
                        prog = float(np.dot(np.array([pt.x_m, pt.y_m]) - p0_, (p1_ - p0_) / L_) / L_)
                    if (pt.current_strip_id in strip_ids and strip_ids.index(pt.current_strip_id) >= INJECT_STRIP_INDEX
                            and INJECT_MIN_PROGRESS <= prog < 0.95):
                        injected = True
                        pt = dataclasses.replace(pt, strip_progress=prog)
                        tels[uid] = pt
                        sm_old = fl.seqmaps[uid]
                        wp_i = _wp_index_at(sm_old, vt.mission_seq)
                        w = up.waypoints[wp_i] if wp_i is not None else None
                        t_ev = float(w.t_s) if w is not None else float(pt.t_s)
                        tel = {}
                        for u2, p2 in tels.items():
                            tel[u2] = dataclasses.replace(p2, t_s=t_ev, e_tel_wh=(E_TEL_DROP_WH if u2 == uid else None))
                        msg = (f"[{now - wall0:6.1f}s] INJECT ENERGY_UPDATE {uid}: e_tel={E_TEL_DROP_WH} Wh "
                               f"(plan {w.energy_remaining_wh if w else float('nan'):.0f} Wh, mock battery {vt.battery_pct} %) "
                               f"at strip {pt.current_strip_id} progress {pt.strip_progress:.2f}, seq {vt.mission_seq} -> wp {wp_i}, "
                               f"plan t={t_ev:.0f} s, telemetry t={pt.t_s:.0f} s")
                        print(msg)
                        events_log.append(msg)
                        t0 = time.perf_counter()
                        try:
                            res = dyn.replan(plan, dyn.EnergyUpdate(uid, t_ev, E_TEL_DROP_WH), tel, ctx, cache, cfg, [])
                        except Exception:
                            traceback.print_exc()
                            res = None
                        dt = (time.perf_counter() - t0) * 1e3
                        if res is not None:
                            replan_res = res
                            msg = (f"   replan {dt:.0f} ms: changed={res.changed_uavs} makespan {plan.makespan_s:.0f}->"
                                   f"{res.plan.makespan_s:.0f} s uncovered={res.plan.uncovered} notes={res.notes[:4]}")
                            print(msg)
                            events_log.append(msg)
                            # ---- 重新上傳變更機並切換至現行位置對應之新 seq -------------------------
                            for cu in res.changed_uavs:
                                if cu not in res.plan.plans:
                                    continue
                                newp = res.plan.plans[cu]
                                k = len(newp.waypoints) - len(res.diff.get(cu, []))
                                sm_prev = fl.seqmaps.get(cu)
                                cur_seq = int(fl.last_tel[cu].mission_seq) if cu in fl.last_tel else 0
                                cur_wp = _wp_index_at(sm_prev, cur_seq)
                                t1 = time.perf_counter()
                                rr = fl.replan_and_reupload(cu, res.plan, ctx, cfg, diff_start_wp=k,
                                                            out_dir=os.path.join(mock_dir, "after"), set_current=False)
                                target_wp = k if cur_wp is None else min(cur_wp, k)
                                seq_go = rr.seqmap.seq_of_wp(target_wp) if rr.seqmap is not None else None
                                if seq_go is None:
                                    seq_go = rr.switch_seq if rr.switch_seq is not None else 1
                                ok = fl[cu].set_current(int(seq_go))
                                msg = (f"   re-upload {cu}: {'ACCEPTED' if rr.upload.ok else 'FAILED'} ({rr.upload.n_items} items, "
                                       f"{(time.perf_counter() - t1) * 1e3:.0f} ms); anchor wp {k}, vehicle at seq {cur_seq} (wp {cur_wp}) "
                                       f"-> set_current {seq_go} ({'OK' if ok else 'no MISSION_CURRENT echo'}); "
                                       f"module switch_seq={rr.switch_seq}; new strips {[s for s, _ in newp.strips]} t_land {newp.t_land_s:.0f} s")
                                print(msg)
                                events_log.append(msg)
                            plan = res.plan
                            plan_after = res.plan
            if now - last_print > 5.0:
                last_print = now
                parts = []
                for uid in uav_ids:
                    vt = fl.last_tel.get(uid)
                    if vt is None:
                        parts.append(f"{uid}: (no telemetry)")
                        continue
                    pt = tels.get(uid)
                    parts.append(f"{uid}: t={vt.t_s:.0f}s seq {vt.mission_seq} alt {vt.rel_alt_m:.0f} m bat {vt.battery_pct}% "
                                 f"{vt.mode} strip {pt.current_strip_id if pt else None}")
                print(f"[{now - wall0:6.1f}s] " + " | ".join(parts))
            if landed == set(uav_ids):
                print(f"[{now - wall0:6.1f}s] all landed")
                break
        else:
            print(f"wall limit {a.wall_limit} s reached; landed={sorted(landed)}")
    except Exception:
        traceback.print_exc()
    finally:
        try:
            if fl is not None:
                fl.close()
        except Exception:
            pass
        for uid, pr in procs.items():
            try:
                pr.terminate()
                pr.wait(timeout=5)
            except Exception:
                pr.kill()
            logs[uid].close()

    # ---- 7. 摘要與圖 ---------------------------------------------------------------
    rows = []
    for uid in uav_ids:
        tr = trace[uid]
        if not tr:
            rows.append([uid, 0, "-", "-", "-", "-", "-"])
            continue
        arr = np.array([[r[1], r[2], r[3]] for r in tr])
        ref = plan_after.plans[uid] if uid in plan_after.plans else plan0.plans[uid]
        wps = np.array([[w.x_m, w.y_m] for w in ref.waypoints])
        d = np.array([np.min(np.hypot(wps[:, 0] - x, wps[:, 1] - y)) for x, y in arr[:, :2]])
        rows.append([uid, len(tr), f"{tr[-1][0]:.0f}", f"{ref.t_land_s:.0f}", f"{tr[-1][5]}", f"{np.max(d):.0f}",
                     f"{np.median(d):.0f}", "landed" if uid in landed else f"seq {tr[-1][4]}"])
    print("\n" + fmt_table(rows, ["uav", "samples", "sim_t_end_s", "plan_t_land_s", "bat_%", "max_dev_m", "med_dev_m", "state"]))
    print("  (dev = distance from mock position to nearest planned waypoint; mock flies straight legs at AIRSPEED_CRUISE, "
          "ignores wind and turn dynamics)")
    print(f"auto-detected events from telemetry (logged only): {auto_events}")
    for m in events_log:
        print(m)
    for uid in uav_ids:
        lp = os.path.join(mock_dir, f"mock_{uid}.log")
        if os.path.exists(lp):
            with open(lp, encoding="utf-8", errors="replace") as f:
                lines = [ln.strip().encode("ascii", "replace").decode() for ln in f
                         if "accepted" in ln or "complete" in ln or "AUTO start" in ln]
            print(f"  mock log {uid}: {lines[:6]}")

    def _plot():
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        colors = {"A1": "tab:red", "B1": "tab:blue"}
        for uid in uav_ids:
            c = colors.get(uid, "k")
            p0 = plan0.plans[uid]
            ax1.plot([w.x_m for w in p0.waypoints], [w.y_m for w in p0.waypoints], color=c, lw=0.8, alpha=0.35,
                     label=f"{uid} plan (before)")
            if uid in plan_after.plans and plan_after.plans[uid] is not p0:
                p1 = plan_after.plans[uid]
                ax1.plot([w.x_m for w in p1.waypoints], [w.y_m for w in p1.waypoints], color=c, lw=1.0, ls="--",
                         label=f"{uid} plan (after replan)")
            tr = trace[uid]
            if tr:
                ax1.plot([r[1] for r in tr], [r[2] for r in tr], color=c, lw=1.6, label=f"{uid} mock track")
                ax2.plot([r[0] for r in tr], [r[3] for r in tr], color=c, lw=1.2, label=f"{uid} mock z")
                ax2.plot([w.t_s for w in p0.waypoints], [w.z_m for w in p0.waypoints], color=c, lw=0.8, alpha=0.35,
                         label=f"{uid} plan z")
        for s in plan0.strips.values():
            ax1.plot([s.pL[0], s.pR[0]], [s.pL[1], s.pR[1]], color="0.6", lw=0.5)
        for sid, (xy, _z) in ctx.sites_enu.items():
            ax1.plot(xy[0], xy[1], "k^", ms=8)
            ax1.annotate(sid, xy)
        title = "mock autopilot closed loop (mini example)"
        if events_log:
            title += "\nENERGY_UPDATE injected on A1 -> divert; B1 takes over the lost strips"
        ax1.set_title(title, fontsize=9)
        ax1.set_aspect("equal")
        ax1.set_xlabel("E [m]")
        ax1.set_ylabel("N [m]")
        ax1.legend(fontsize=7, loc="best")
        ax2.set_xlabel("plan time axis [s] (mock boot time - offset)")
        ax2.set_ylabel("ENU z [m]")
        ax2.set_title("altitude: mock vs plan")
        ax2.legend(fontsize=7)
        ax2.grid(alpha=0.3)
        p = os.path.join(FIG_DIR, "mock_loop_trace.png")
        fig.tight_layout()
        fig.savefig(p, dpi=130)
        plt.close(fig)
        return p
    p = step("plot trace", _plot)
    if p:
        print("figure:", os.path.relpath(p, FIG_DIR))
    with open(os.path.join(mock_dir, "trace.json"), "w", encoding="utf-8") as f:
        json.dump({"trace": trace, "events": events_log, "auto_events": auto_events,
                   "replan": None if replan_res is None else {"changed": replan_res.changed_uavs, "notes": replan_res.notes}},
                  f, indent=1, default=str)
    print(f"\nTOTAL {time.perf_counter() - t_all:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
