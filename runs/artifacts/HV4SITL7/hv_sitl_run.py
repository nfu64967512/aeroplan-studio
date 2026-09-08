"""情境驅動的 HV-DCCPP SITL 實跑（規劃 → 匯出 → 起 SITL → 上傳 → 飛 → 收資料）。

    python scripts/hv_sitl_run.py list                 # 列出情境
    python scripts/hv_sitl_run.py plan  gui_uniform_1a3b
    python scripts/hv_sitl_run.py fly   gui_uniform_1a3b

與 `scripts/hv_sitl_flight.py` 的關係
------------------------------------
那一支是 HV4SITL1/2 用的，機隊與作業區寫死在模組層，**刻意不動它**，
好留下一條可重現的基準線。這一支把「情境」抽出來（見 `hv_sitl_scenarios.py`），
並支援兩條規劃路徑；低階的 SITL 參數檔產生器與行程收拾直接沿用那一支。

每一步的守則（都是實測踩出來的，不要省）
--------------------------------------
1. `--defaults` 鏈最後兩份必須是 `vtol_enforced.parm`（ARMING_REQUIRE=1，否則
   四軸馬達停在 PWM 1000）與該機自己的 `<UAV>.param`。
2. **讀回**每一個關鍵參數再比對：名字打錯或韌體改名時 ArduPilot 靜默忽略。
3. 上傳任務後**下載回來**比對項數。
4. ⚠⚠ ArduPlane 4.8 **不執行** NAV_DELAY ⇒ 錯開起飛必須由地面站做：
   任務檔用 `takeoff_delay="operator"`，每架各自等到自己的 t_takeoff_s 才切 AUTO。
5. ⚠⚠ 先解鎖再切 AUTO。在 AUTO 但未解鎖時任務狀態機已經在跑。
6. 時間一律取 `GLOBAL_POSITION_INT.time_boot_ms`（模擬時鐘），不是牆鐘。
"""
from __future__ import annotations

import csv
import json
import logging
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.hv_sitl_flight import (  # noqa: E402
    BINARY, HOME_ALT_MSL, SITL_DIR, _instance_dir, _write_enforced,
    _write_firmware_override, _write_identity,
)
from scripts.hv_sitl_scenarios import HOME, SCENARIOS, Scenario  # noqa: E402

SPEEDUP = float(os.environ.get("HV_SPEEDUP", "5"))
ALTERNATES = [(23.972571, 120.90)]

CHECK_PARAMS = [
    "AIRSPEED_CRUISE", "AIRSPEED_MIN", "AIRSPEED_MAX", "AIRSPEED_STALL",
    "ROLL_LIMIT_DEG", "TECS_CLMB_MAX", "TECS_SINK_MAX", "WP_LOITER_RAD",
    "RTL_RADIUS", "Q_ASSIST_SPEED", "Q_RTL_ALT", "Q_TRANSITION_MS",
    "Q_ENABLE", "ARMING_REQUIRE", "SCHED_LOOP_RATE", "Q_RTL_MODE", "RTL_AUTOLAND",
]
EXPECT_EXTRA = {"Q_ENABLE": 1.0, "ARMING_REQUIRE": 1.0, "SCHED_LOOP_RATE": 200.0,
                "Q_RTL_MODE": 0.0, "RTL_AUTOLAND": 2.0}


def out_dir(sc: Scenario) -> Path:
    return ROOT / "runs" / "artifacts" / os.environ.get("HV_TRIAL", sc.trial)


# ── 只收拾自己的行程 ──────────────────────────────────────────────────
#   ⚠ `hv_sitl_flight.stop_all()` 是 `taskkill` 掉**機器上每一個** ArduPlane.exe。
#     這台機器同時有別的 session 在跑東西，一趟 SITL 不該去殺別人的。
#     所以這裡改成：只殺命令列裡帶著我們 instance 目錄的那幾個。
_INSTANCE_TAG = str((SITL_DIR / "vtol").resolve())


def _sitl_pids(tag: str = _INSTANCE_TAG) -> list[int]:
    """命令列含 `tag` 的 ArduPlane.exe PID。查不到就回空清單（不要猜）。"""
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='ArduPlane.exe'\" | "
          "ForEach-Object { \"$($_.ProcessId)`t$($_.CommandLine)\" }")
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                           capture_output=True, text=True, timeout=30)
    except Exception as e:      # noqa: BLE001
        print(f"  查行程失敗（{e}），不做任何收拾")
        return []
    pids = []
    for line in r.stdout.splitlines():
        pid, _, cmd = line.partition("\t")
        if pid.strip().isdigit() and tag.lower() in cmd.lower():
            pids.append(int(pid))
    return pids


def stop_ours(pids: list[int] | None = None) -> int:
    """終止我們自己起的 SITL。`pids` 不給時用命令列比對找。"""
    before_all = set(_sitl_pids(tag="ArduPlane.exe"))     # 每一個 ArduPlane 的 PID
    targets = pids if pids is not None else _sitl_pids()
    n = 0
    for pid in targets:
        r = subprocess.run(["taskkill", "/PID", str(pid), "/F"], capture_output=True)
        n += (r.returncode == 0)
    left = len(before_all - set(targets))
    print(f"已終止 {n} 個本次的 ArduPlane 行程"
          + (f"（機器上另有 {left} 個不是我們起的，沒有動）" if left else ""))
    return n


# ══════════════════════════════════════════════════════════════════════
#  1. 規劃（兩條路徑）
# ══════════════════════════════════════════════════════════════════════
def _fence_zones(sc: Scenario):
    """情境的禁航區 → FenceZone。空的就回 [] （代表「確定沒有禁航區」而不是「沒查」）。"""
    if not sc.nfz:
        return []
    from mission.fence_zone import FenceZone, ZoneCategory, ZoneShape
    return [FenceZone(name=f"NFZ{i + 1}", category=ZoneCategory.NFZ,
                      shape=ZoneShape.POLYGON, vertices=list(v),
                      alt_min_m=0.0, alt_max_m=500.0)
            for i, v in enumerate(sc.nfz)]


def _plan_gui(sc: Scenario):
    """AeroPlan 介面按下去真正會走的那一條。兩型共用單光相機。"""
    from mission.swarm_coordinator import SwarmCoordinator
    cls_of = {i + 1: c for i, (_n, c, _s, _v, _r) in enumerate(sc.fleet)}
    report: dict = {}
    c = SwarmCoordinator()
    c.use_hv_dccpp = True
    c.hv_uniform_sensor = True
    c.hv_class_of = lambda d: cls_of[int(d["drone_id"])]
    c.hv_home = HOME
    c.hv_alternates = ALTERNATES
    c.hv_wind = sc.wind
    c.hv_fence_zones = _fence_zones(sc)
    c.hv_on_report = report.update
    c.hv_params = {"hfov_deg": sc.hfov_deg, "vfov_deg": sc.vfov_deg,
                   "hv_c2_range_km": 25.0, "hv_waiver_max_agl_m": 200.0,
                   "hv_dem_margin_m": 3000.0, "hv_terrain_clearance_m": 36.0}
    c.plan_coverage_dccpp(areas=sc.areas, drones=sc.drones(),
                          coverage_width_m=sc.coverage_width_m,
                          overlap_rate=sc.overlap_rate,
                          coverage_altitude=sc.coverage_altitude_m,
                          mounting_angle_deg=0.0)
    return c.hv_coordinator, report


def _plan_direct(sc: Scenario):
    """HV4SITL1/2 用的那一條：自己給 payload_reqs。"""
    from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator
    from hv_dccpp.mission import PayloadRequirement, PlannerConfig
    reqs = {rid: PayloadRequirement(rid, **kw) for rid, kw in sc.payload_reqs.items()}
    report: dict = {}
    coord = HvDccppCoordinator(
        class_of=lambda d: str(d["hv_class"]),
        cfg=PlannerConfig(dem_step_m=25.0, t_sep_ldg_s=30.0),
        home=HOME, alternates=ALTERNATES, payload_reqs=reqs,
        wind=sc.wind, c2_range_km=25.0, terrain_clearance_m=36.0,
        waiver_max_agl_m=200.0, dem_margin_m=3000.0,
        on_report=report.update, raise_on_no_go=True)
    coord.plan_coverage_dccpp(areas=sc.areas, drones=sc.drones(),
                              coverage_width_m=sc.coverage_width_m,
                              overlap_rate=sc.overlap_rate,
                              coverage_altitude=sc.coverage_altitude_m)
    return coord, report


def plan_and_export(sc: Scenario):
    import hv_dccpp as hvlib
    out = out_dir(sc)
    out.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    hv, report = (_plan_gui if sc.path == "gui" else _plan_direct)(sc)
    dt = time.perf_counter() - t0
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg

    # ★ 閘門：規劃器可能只用機隊的一部分，而且**不會有任何訊息**。
    working = {u: len(p.strips) for u, p in plan.plans.items()}
    n_fleet = len(sc.fleet)
    if len(plan.plans) != n_fleet or not all(working.values()):
        raise SystemExit(f"不是 {n_fleet} 架都有工作：{working}"
                         f"（用 scripts/hv_sitl_scout.py 換幾何）")

    cache = hvlib.build_validation_cache(plan, ctx, budget_only=False)
    v = hvlib.validate(plan, ctx, cache, cfg)

    print(f"情境 {sc.key}（路徑 {sc.path}）：規劃 {dt:.2f}s  "
          f"makespan {plan.makespan_s:.1f}s  status {report.get('status')}")
    print(f"validate passed={v.passed} conflicts={len(v.conflicts)} "
          f"uncovered={len(plan.uncovered)}")
    kinds = {cls: sen.hw.kind for (cls, _rid), sen in ctx.sensors.items()}
    print(f"感測器 {kinds}   酬載需求 "
          f"{ {r.req_id: r.kind for r in hv.last_inputs.payload_reqs.values()} }")
    for u, p in sorted(plan.plans.items()):
        print(f"  {u} {p.cls} strips={len(p.strips)} wp={len(p.waypoints)} "
              f"{p.distance_m / 1000:.2f}km  t_off={p.t_takeoff_s:.0f}s "
              f"t_land={p.t_land_s:.0f}s  E={p.energy_used_wh:.0f}/{p.energy_margin_wh:.0f}Wh")

    plan_dir = out / "plan"
    if plan_dir.exists():
        shutil.rmtree(plan_dir)
    summary = hvlib.export_all(plan, ctx, cfg, plan_dir, alt_frame="relative",
                               camera_trigger=True, takeoff_delay="operator")
    (out / "validation.json").write_text(json.dumps(
        {"passed": v.passed, "conflicts": len(v.conflicts), "coverage_ok": v.coverage_ok,
         "energy_ok": v.energy_ok, "min_sep_h_m": v.min_sep_h_m,
         "min_sep_v_m": v.min_sep_v_m}, ensure_ascii=False, indent=1), encoding="utf-8")
    (out / "report.json").write_text(json.dumps(
        {k: report.get(k) for k in ("status", "errors", "warnings", "notes",
                                    "uncovered", "timing_ms")},
        ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    (out / "scenario.json").write_text(json.dumps(
        {"key": sc.key, "path": sc.path, "n_a": sc.n_a, "n_b": sc.n_b,
         "altitude_m": sc.coverage_altitude_m, "hfov_deg": sc.hfov_deg,
         "vfov_deg": sc.vfov_deg, "overlap_rate": sc.overlap_rate,
         "wind": list(sc.wind), "areas": sc.areas, "nfz": sc.nfz, "note": sc.note,
         "sensor_kinds": kinds, "speedup": SPEEDUP},
        ensure_ascii=False, indent=1), encoding="utf-8")
    print("匯出:", {u: summary["uavs"][u]["n_items"] for u in summary["uavs"]})
    return hv, plan, ctx, cfg, summary


# ══════════════════════════════════════════════════════════════════════
#  2. 起 SITL
# ══════════════════════════════════════════════════════════════════════
def launch(sc: Scenario, hv) -> list[dict]:
    """★ uav_id → sysid 用 `hv.id_map`（drone_id → uav_id）反查，不要靠排序 zip。"""
    out = out_dir(sc)
    sysid_of = {}
    for drone_id, uav_id in hv.id_map.items():
        _n, cls, sysid, *_ = sc.fleet[int(drone_id) - 1]
        sysid_of[uav_id] = (sysid, cls)
    procs, lines = [], []
    for i, uav in enumerate(sorted(hv.last_plan.plans)):
        sysid, cls = sysid_of[uav]
        work = _instance_dir(i)
        for junk in ("eeprom.bin", "mav.parm"):
            (work / junk).unlink(missing_ok=True)
        defaults = [
            SITL_DIR / "default_params" / "plane.parm",
            SITL_DIR / "default_params" / "vtol_default.parm",
            _write_identity(i, sysid),
            _write_enforced(i),
            out / "plan" / f"{uav}.param",
            _write_firmware_override(i),
        ]
        missing = [str(d) for d in defaults if not d.exists()]
        if missing:
            raise SystemExit(f"缺參數檔：{missing}")
        cmd = [str(BINARY), "--model", "quadplane",
               "--home", f"{HOME[0]},{HOME[1]},{HOME_ALT_MSL},0",
               "--speedup", str(SPEEDUP), "--instance", str(i),
               "--sysid", str(sysid),
               "--defaults", ",".join(str(d) for d in defaults)]
        lines.append(f"[{uav}] " + " ".join(cmd))
        proc = subprocess.Popen(cmd, cwd=str(work),
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append({"uav": uav, "cls": cls, "sysid": sysid, "instance": i,
                      "port": 5760 + 10 * i, "pid": proc.pid})
        time.sleep(1.0)
    (out / "sitl_cmdlines.txt").write_text("\n".join(lines), encoding="utf-8")
    return procs


# ══════════════════════════════════════════════════════════════════════
#  3. 飛
# ══════════════════════════════════════════════════════════════════════
def _param_file_values(sc: Scenario, uav: str) -> dict[str, float]:
    vals = {}
    for ln in (out_dir(sc) / "plan" / f"{uav}.param").read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if ln and not ln.startswith("#") and "," in ln:
            k, v = ln.split(",", 1)
            vals[k.strip()] = float(v)
    return vals


def fly(sc: Scenario) -> int:
    from hv_dccpp import ardupilot as ap
    from hv_dccpp.mavlink_link import VehicleLink

    out = out_dir(sc)
    hv, plan, ctx, cfg, _summary = plan_and_export(sc)
    uavs = sorted(plan.plans)
    print(f"\n機隊 {uavs}  makespan {plan.makespan_s:.0f}s  speedup {SPEEDUP}"
          f"  → 牆鐘約 {plan.makespan_s / SPEEDUP / 60:.1f} 分鐘\n")

    stop_ours()                       # 只收拾上一趟殘留的「我們的」
    procs = launch(sc, hv)
    print("SITL 已啟動:", [(p["uav"], p["port"], p["pid"]) for p in procs])

    links: dict[str, VehicleLink] = {}
    try:
        for p in procs:
            lk = VehicleLink(f"tcp:127.0.0.1:{p['port']}", sysid=p["sysid"],
                             timeout_s=60.0, uav_id=p["uav"], time_source="boot")
            lk.wait_heartbeat(timeout_s=90.0)
            links[p["uav"]] = lk
            print(f"  {p['uav']} 心跳 OK (sysid {p['sysid']}, port {p['port']})")

        # ── 參數讀回 ───────────────────────────────────────────────
        rows, bad = [], []
        for uav, lk in links.items():
            want = _param_file_values(sc, uav) | EXPECT_EXTRA
            for name in CHECK_PARAMS:
                exp = want.get(name)
                got = lk.get_param(name, timeout_s=10.0)
                note = ""
                ok = (got is not None and exp is not None
                      and abs(got - exp) <= max(0.01, abs(exp) * 0.001))
                # 韌體端整數型別會截掉小數（實測 AIRSPEED_MAX 是 INT16 ⇒ 26.6→26.0）。
                if not ok and got is not None and exp is not None and \
                        got in (float(int(exp)), float(round(exp))):
                    ok, note = True, f"韌體整數型別，{exp} → {got}"
                rows.append({"uav": uav, "param": name, "expected": exp,
                             "actual": got, "ok": bool(ok), "note": note})
                if exp is not None and not ok:
                    bad.append(f"{uav}.{name} 期望 {exp} 實得 {got}")
        with (out / "params_readback.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["uav", "param", "expected", "actual", "ok", "note"])
            w.writeheader()
            w.writerows(rows)
        print(f"\n參數讀回：{sum(r['ok'] for r in rows)}/{len(rows)} 相符")
        if bad:
            for b in bad:
                print("   ✗", b)
            raise SystemExit("參數沒有生效，停止起飛（見 params_readback.csv）")

        # ── 上傳 + 下載比對 ────────────────────────────────────────
        upload_rows = []
        for uav, lk in links.items():
            items = ap.plan_to_mission_items(plan.plans[uav], ctx, cfg,
                                             alt_frame="relative", camera_trigger=True)
            lk.upload_mission(items, timeout_s=60.0)
            back = lk.download_mission(timeout_s=60.0)
            same = len(back) == len(items)
            upload_rows.append({"uav": uav, "sent": len(items), "read_back": len(back),
                                "ok": same, "first_cmds": [i.command for i in back[:3]]})
            print(f"  {uav} 上傳 {len(items)} → 讀回 {len(back)} "
                  f"{'OK' if same else '✗ 不一致'}")
            if not same:
                raise SystemExit(f"{uav} 任務讀回不一致")
        (out / "mission_upload.json").write_text(
            json.dumps(upload_rows, ensure_ascii=False, indent=1), encoding="utf-8")

        # ── 解鎖（先解鎖再切 AUTO）─────────────────────────────────
        for uav, lk in links.items():
            armed, reasons, t0 = False, [], time.time()
            while not armed and time.time() - t0 < 120.0:
                armed = lk.arm(True, timeout_s=10.0)
                if armed:
                    break
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    msg = lk.conn.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
                    if msg is not None and msg.text not in reasons:
                        reasons.append(msg.text)
            if not armed:
                print(f"  {uav} 解鎖失敗，飛控說：")
                for r in reasons[-6:]:
                    print("     ", r)
                raise SystemExit(f"{uav} 解鎖失敗")
            print(f"  {uav} armed（等 {time.time() - t0:.0f}s）")

        # ── 依規劃起飛時刻逐機切 AUTO ─────────────────────────────
        t_off = {u: float(plan.plans[u].t_takeoff_s) for u in links}
        t_min = min(t_off.values())
        release = {u: (t_off[u] - t_min) / SPEEDUP for u in links}
        print("\n放行時刻（牆鐘秒）:",
              {u: round(v, 1) for u, v in sorted(release.items(), key=lambda kv: kv[1])})

        writers, files, peak = {}, {}, {u: 0.0 for u in links}
        for uav in links:
            d = out / uav
            d.mkdir(parents=True, exist_ok=True)
            f = (d / "flight.csv").open("w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(["t_s", "lat", "lon", "rel_alt_m", "airspeed",
                        "groundspeed", "wp_seq", "mode", "armed"])
            files[uav], writers[uav] = f, w

        budget_s = plan.makespan_s / SPEEDUP + 420.0
        t_start = time.time()
        done: set[str] = set()
        released: dict[str, float] = {}
        dead: set[str] = set()          # 連續拿不到遙測 ⇒ 不再輪詢（見迴圈內說明）
        rejected: set[str] = set()      # 切 AUTO 三次都被拒 ⇒ 這一架不會起飛
        miss: dict[str, int] = {}
        DEAD_AFTER = 60
        while (time.time() - t_start < budget_s and len(done) < len(links)
               and len(dead) < len(links)):
            el = time.time() - t_start
            for uav, lk in links.items():
                if uav not in released and el >= release[uav] and uav not in dead:
                    # ⚠ 切 AUTO 被拒就是「這一架不會起飛」，而且不會有別的徵狀。
                    #   HV4SITL6（speedup 10）就是這樣：兩架被拒、全程 0.0 m，
                    #   腳本卻照跑到逾時。所以重試，仍失敗就記成 rejected。
                    ok = False
                    for _try in range(3):
                        ok = lk.set_mode("AUTO", timeout_s=10.0)
                        if ok:
                            break
                    released[uav] = time.time() - t_start
                    if not ok:
                        rejected.add(uav)
                    print(f"  > {uav} 放行切 AUTO（牆鐘 {released[uav]:.1f}s，"
                          f"規劃 t_off={t_off[uav]:.0f}s）"
                          f"{'' if ok else ' ✗ 三次都被拒，這一架不會起飛'}")
            for uav, lk in links.items():
                if uav in dead:
                    continue
                tel = lk.read_telemetry(timeout_s=0.25, require_new_position=False)
                if tel is None:
                    # ⚠⚠ 連線斷掉時 pymavlink 會**每次呼叫都印一行** socket 錯誤。
                    #   HV4SITL6 那一趟因此產生 1744 萬行 / 832 MB 的 log。
                    #   所以連續拿不到遙測就把這一架標成 dead，不再輪詢它。
                    miss[uav] = miss.get(uav, 0) + 1
                    if miss[uav] >= DEAD_AFTER and uav not in dead:
                        dead.add(uav)
                        print(f"  ✗ {uav} 連續 {DEAD_AFTER} 次拿不到遙測 ⇒ "
                              f"視為斷線，停止輪詢（SITL 掛了？）")
                        if len(dead) == len(links):
                            print("  ✗ 四架全部斷線，提前結束這一趟")
                    continue
                miss[uav] = 0
                alt = float(getattr(tel, "rel_alt_m", 0.0) or 0.0)
                peak[uav] = max(peak[uav], alt)
                writers[uav].writerow([
                    round(float(getattr(tel, "t_s", 0.0) or 0.0), 2),
                    getattr(tel, "lat_deg", None), getattr(tel, "lon_deg", None),
                    round(alt, 2),
                    round(float(getattr(tel, "airspeed_mps", 0.0) or 0.0), 2),
                    round(float(getattr(tel, "groundspeed_mps", 0.0) or 0.0), 2),
                    getattr(tel, "mission_seq", None), getattr(tel, "mode", ""),
                    getattr(tel, "armed", None)])
                if peak[uav] > 20.0 and alt < 2.0 and not getattr(tel, "armed", True):
                    if uav not in done:
                        done.add(uav)
                        print(f"  {uav} 已降落 (sim t={getattr(tel, 't_s', 0):.0f}s)")
            if int(el) % 20 == 0:
                print(f"    [{el:5.0f}s] " + "  ".join(
                    f"{u}:{peak[u]:5.1f}m" for u in sorted(links)))
                time.sleep(1.0)
        for f in files.values():
            f.close()

        (out / "release_schedule.json").write_text(json.dumps(
            {"speedup": SPEEDUP,
             "planned_t_takeoff_s": {u: round(t_off[u], 1) for u in t_off},
             "release_wall_s": {u: round(release[u], 1) for u in release},
             "actual_release_wall_s": {u: round(v, 1) for u, v in released.items()},
             "mode_change_rejected": sorted(rejected), "link_lost": sorted(dead)},
            ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"\n落地 {len(done)}/{len(links)}：{sorted(done)}")
        if rejected:
            print(f"⚠ 切 AUTO 三次都被拒（沒起飛）：{sorted(rejected)}")
        if dead:
            print(f"⚠ 中途斷線：{sorted(dead)}")
        print("最高高度:", {u: round(v, 1) for u, v in peak.items()})
        return 0 if len(done) == len(links) else 2
    finally:
        for lk in links.values():
            try:
                lk.close()
            except Exception:      # noqa: BLE001
                pass
        stop_ours([p["pid"] for p in procs])


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "list"
    if cmd == "list":
        for k, s in SCENARIOS.items():
            print(f"{k:22s} trial={s.trial:10s} path={s.path:7s} "
                  f"{s.n_a}A+{s.n_b}B alt={s.coverage_altitude_m:.0f}m  {s.note}")
        return 0
    key = sys.argv[2] if len(sys.argv) > 2 else ""
    if key not in SCENARIOS:
        print(f"沒有這個情境：{key!r}。可用：{list(SCENARIOS)}")
        return 1
    sc = SCENARIOS[key]
    logging.basicConfig(level=logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")
    if cmd == "plan":
        plan_and_export(sc)
        return 0
    if cmd == "fly":
        return fly(sc)
    if cmd == "stop":
        stop_ours()
        return 0
    print(f"未知指令 {cmd!r}（list/plan/fly/stop）")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
