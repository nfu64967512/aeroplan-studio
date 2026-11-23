"""情境驅動的 HARVEST SITL 實跑（規劃 → 匯出 → 起 SITL → 上傳 → 飛 → 收資料）。

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
import math
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
#: 每條鏈路每輪的遙測輪詢逾時（s）。這是 flight.csv 取樣率的**硬上限**：
#: 四架時迴圈週期 ≈ 4×_POLL_S，再乘上 SPEEDUP 就是模擬時間的取樣間隔。
#: ⚠ 0.25（舊值）⇒ 5.00 s 模擬 ⇒ HERO 一格飛 95 m，而水平門檻是 100 m
#:   ⇒ 兩機可以從取樣縫隙穿過去，「違規 0 筆」與真的安全長得一樣。
_POLL_S = float(os.environ.get("HV_POLL_S", "0.05"))
# ── SITL 起始埠 ──────────────────────────────────────────────────────
#   ⚠⚠⚠ ArduPlane 的 `--instance N` 是「所有埠都加 10·N」，起始 5760。
#   這台機器上另一個 session 會跑 WSL 的 SITL 群，`wslrelay.exe` 把
#   5760/5770/5780/5790 從 WSL 轉發到 Windows localhost ⇒ 撞到時我們的
#   ArduPlane 起得來，但 client 連到的是**別人的載具**，症狀是
#   「90 s 內無載具心跳」，看起來像我們的 SITL 壞了（HV4SITL10 第一次就這樣）。
#   ⇒ 預設整段錯開到 **6760**（instance 100），與標準埠段完全分離。
_SITL_PORT0 = 5760                       # ArduPlane 的內建起始埠，不可改
BASE_PORT = int(os.environ.get("HV_BASE_PORT", "6760"))
if BASE_PORT < _SITL_PORT0 or (BASE_PORT - _SITL_PORT0) % 10:
    raise SystemExit(f"HV_BASE_PORT 必須 ≥ {_SITL_PORT0} 且為 10 的倍數偏移，收到 {BASE_PORT}")
INSTANCE_OFFSET = int(os.environ.get("HV_INSTANCE_OFFSET",
                                     str((BASE_PORT - _SITL_PORT0) // 10)))
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
    c.use_harvest = True
    c.hv_uniform_sensor = True
    c.hv_class_of = lambda d: cls_of[int(d["drone_id"])]
    c.hv_home = HOME
    c.hv_alternates = ALTERNATES
    c.hv_wind = sc.wind
    c.hv_fence_zones = _fence_zones(sc)
    c.hv_on_report = report.update
    # ⚠⚠⚠ 回收站自動指派會**遮蔽**上游的差異：PAD8 那組 A/B 裡，修好與沒修的
    #   原始計畫最小間距是 51.5 m vs **5.0 m**，但自動指派把 B3/B4 都移到 ALT1
    #   之後兩者都變成同一份計畫（889.5 s、148.2 m）⇒ 實飛完全分辨不出來。
    #   要做「豁免判準」的受控實驗就得關掉它，飛原始計畫。
    c.hv_recovery_auto = os.environ.get("HV_RECOVERY_AUTO", "1") != "0"
    if not c.hv_recovery_auto:
        print("⚠ HV_RECOVERY_AUTO=0：已關閉回收站自動指派，這一趟飛的是原始計畫")
    # ⚠ 這四個是情境自己的，不可以再寫死。`hv_waiver_max_agl_m=None`
    #   ＝ 沒有豁免 ⇒ 上游用民航法規的 120 m 當 h_ceil。
    c.harvest_params = {"hfov_deg": sc.hfov_deg, "vfov_deg": sc.vfov_deg,
                   "hv_c2_range_km": 25.0,
                   "hv_waiver_max_agl_m": sc.waiver_max_agl_m,
                   "hv_dem_margin_m": 3000.0, "hv_terrain_clearance_m": 36.0}
    if sc.min_fw_agl_m:
        c.harvest_params["hv_min_fw_agl_m"] = dict(sc.min_fw_agl_m)
    if sc.dh_band_min_m is not None:
        c.harvest_params["hv_dh_band_min_m"] = float(sc.dh_band_min_m)
    if sc.target_gsd is not None:
        c.harvest_params["target_gsd"] = float(sc.target_gsd)
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
         "sensor_kinds": kinds, "speedup": SPEEDUP,
         # ⚠ 沒有這一欄的話，分散起降點的航次與共點航次在產物上**分不出來**
         #   （唯一的痕跡是 sitl_cmdlines.txt 裡的 --home）。
         "spawn_spread_m": sc.spawn_spread_m,
         "takeoff_sites": {u: getattr(p_, "takeoff_site", "") for u, p_ in plan.plans.items()},
         # ★ 這一趟真正生效的空域／平台條件（讀回，不是抄情境定義）
         "waiver_max_agl_m": sc.waiver_max_agl_m,
         "h_ceil_m": (120.0 if sc.waiver_max_agl_m is None else sc.waiver_max_agl_m),
         "min_fw_agl_m": {"A": hv.platform_a.min_fw_operation_agl_m,
                          "B": hv.platform_b.min_fw_operation_agl_m},
         "dh_band_min_m": cfg.dh_band_min_m,
         "target_gsd": sc.target_gsd},
        ensure_ascii=False, indent=1), encoding="utf-8")
    print("匯出:", {u: summary["uavs"][u]["n_items"] for u in summary["uavs"]})
    return hv, plan, ctx, cfg, summary


# ══════════════════════════════════════════════════════════════════════
#  2. 起 SITL
# ══════════════════════════════════════════════════════════════════════
def _home_of(hv, uav: str) -> tuple[float, float]:
    """該機 SITL 的生成點 = **規劃真正登記的起飛站**之座標。

    來源順序（每一步都讀回，讀不到就往下，全部落空才丟例外）：
    1. ``UAVPlan.takeoff_site`` → ``ctx.inputs.sites[<id>]`` 的 lat/lon（vendored #8）。
    2. ``ctx.inputs.home`` 的 lat/lon。

    ⚠⚠⚠ **不要**在這裡退回模組層的 ``HOME`` 常數。那樣做的話「規劃分了站、
    SITL 沒分」會靜靜通過：任務照上傳、飛機照飛，只有第一段航線歪掉，
    而航線歪掉在報表上跟正常長得一樣。寧可停下來。
    """
    plan = hv.last_plan.plans[uav]
    ctx = hv.last_ctx
    sid = str(getattr(plan, "takeoff_site", "") or "")
    sites = getattr(getattr(ctx, "inputs", None), "sites", None) or {}
    site = sites.get(sid) if sid else None
    if site is None:
        site = getattr(getattr(ctx, "inputs", None), "home", None)
    if site is None or not hasattr(site, "lat_deg"):
        raise SystemExit(f"{uav}：查不到起飛站座標（takeoff_site={sid!r}）⇒ 停止，"
                         f"不要用預設 HOME 蓋過去")
    return (float(site.lat_deg), float(site.lon_deg))


def launch(sc: Scenario, hv) -> list[dict]:
    """★ uav_id → sysid 用 `hv.id_map`（drone_id → uav_id）反查，不要靠排序 zip。"""
    out = out_dir(sc)
    sysid_of = {}
    for drone_id, uav_id in hv.id_map.items():
        _n, cls, sysid, *_ = sc.fleet[int(drone_id) - 1]
        sysid_of[uav_id] = (sysid, cls)
    procs, lines = [], []
    # 埠段錯開的理由見檔頭 BASE_PORT。
    for j, uav in enumerate(sorted(hv.last_plan.plans)):
        i = j + INSTANCE_OFFSET
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
        # ⚠⚠⚠ 起飛點必須**讀回規劃真正用的那個站**，不可以寫死 HOME。
        #   規劃層在 spawn_spread_m > 0 時會生出逐機的 TO_<uav> 站；若 SITL 仍然
        #   全部生在 HOME，就變成「計畫說從 TO_A1 起飛、飛機在別的地方」——
        #   而這個不一致**沒有任何徵狀**：任務照上傳、照飛，只是第一段航線歪掉。
        #   `_home_of` 從 ctx 讀站點座標，讀不到就大聲停下來而不是默默用 HOME。
        hlat, hlon = _home_of(hv, uav)
        cmd = [str(BINARY), "--model", "quadplane",
               "--home", f"{hlat},{hlon},{HOME_ALT_MSL},0",
               "--speedup", str(SPEEDUP), "--instance", str(i),
               "--sysid", str(sysid),
               "--defaults", ",".join(str(d) for d in defaults)]
        lines.append(f"[{uav}] " + " ".join(cmd))
        proc = subprocess.Popen(cmd, cwd=str(work),
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append({"uav": uav, "cls": cls, "sysid": sysid, "instance": i,
                      "port": _SITL_PORT0 + 10 * i, "pid": proc.pid})
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

        # ── 遙測速率：把位置串流拉高，否則間距量測**點不著** ────────
        #   ⚠⚠⚠ 預設 SR*_POSITION = 1 Hz 是**牆鐘**頻率，而 --speedup 5 讓模擬時鐘
        #   跑 5 倍 ⇒ flight.csv 的取樣間隔是 **5.00 s 模擬時間**（實測中位數）。
        #   HERO 19 m/s 在一個間隔飛 95 m，而水平門檻是 100 m ⇒ 兩機可以整段
        #   從彼此的取樣縫隙穿過去，量到的「違規 0 筆」跟真的安全長得一模一樣。
        #   ⇒ 拉到 10 Hz（模擬 0.5 s、HERO 9.5 m）並**讀回**確認真的生效。
        #   ⚠⚠⚠ 參數名是 **MAV1_/MAV2_/MAV3_**，不是 SR0_/SR1_。這版韌體把整組
        #     串流速率參數改名了（見 sitl/default_params/firmware_param_names.json：
        #     只有 MAV1_POSITION…MAV3_POSITION，沒有任何 SR* 串流參數）。
        #     第一版寫 SR0_/SR1_，四架讀回**全是 None**、速率一點都沒動 ——
        #     這正是「ArduPilot 對未知參數名靜默略過」那條老陷阱。
        #     ⇒ 名字錯了唯一的徵狀就是讀回 None，所以**一定要讀回**。
        #   ⚠⚠ 只調 POSITION 還是沒用（實測：參數讀回 10.0，flight.csv 仍是 5.00 s）。
        #     `read_telemetry` 的提前結束條件要**同時**有 EXTENDED_SYS_STATE 與
        #     BATTERY_STATUS/SYS_STATUS 在快取裡；拿不到就每次都跑滿 0.25 s 逾時，
        #     四條鏈路 ⇒ 迴圈固定 ~1 Hz 牆鐘 ⇒ 5 s 模擬。⇒ EXT_STAT／EXTRA2 也要調。
        #     判準是 flight.csv 的**實際中位間隔**，不是參數讀回值。
        want_hz = float(os.environ.get("HV_TELEM_HZ", "10"))
        _sr = {f"MAV{c}_{k}": want_hz for c in (1, 2, 3)
               for k in ("POSITION", "EXTRA1", "EXTRA2", "EXTRA3", "EXT_STAT")}
        for uav, lk in links.items():
            lk.set_params(_sr, timeout_s=5.0)
        got_hz = {u: lk.get_param("MAV1_POSITION", timeout_s=10.0) for u, lk in links.items()}
        print(f"\n位置串流速率讀回: {got_hz}（要求 {want_hz} Hz）")
        if any(v is None or abs(float(v) - want_hz) > 0.51 for v in got_hz.values()):
            print("  ⚠ 有機體沒吃到速率設定 ⇒ 該機的間距量測仍是 5 s 解析度，"
                  "低於數百公尺的結論不可信")

        # ── 生成點讀回：飛機真的生在規劃說的那個起飛站嗎 ────────────
        #   ⚠⚠⚠ 這是 vendored #8 之後必要的一道。規劃層在 spawn_spread_m > 0 時
        #   會逐機生出 TO_<uav> 站；若 SITL 仍全部生在 HOME，任務照上傳、照飛，
        #   只有第一段航線歪掉 —— **完全沒有徵狀**。所以讀回實際位置比對。
        #   ⚠ 起飛前的 GLOBAL_POSITION_INT 就是 `--home`（還沒動）。
        home_rows, home_bad = [], []
        for uav, lk in links.items():
            want = _home_of(hv, uav)
            # ⚠⚠⚠ 判準是**資料穩定**，不是數值範圍。EKF 還在初始化時回報的位置
            #   會從 0 附近往上漂：實測第一架（心跳後立刻被問）先回 (0, 0)，接著
            #   回 (2.975e-4, 2.37e-5)。兩版守衛都被它騙過去 ——
            #     v1「la or lo 非零」→ (0,0) 之後那一筆過關；
            #     v2「兩個分量 > 1e-6」→ 2.975e-4 也過關。
            #   兩次都量出 **12 594 400 m** 的「誤差」，看起來像規劃與 SITL 天差地遠。
            #   ⚠ 不可以改成「離我們送出的 --home 夠近才算數」：那正好把本檢查要抓的
            #     「規劃分了站、SITL 沒分」整個遮掉（拿被測物自己的輸入當判準）。
            #   ⇒ 改成等**連續兩筆相隔 ≥1.5 s 且相距 ≤0.5 m**，收斂了才比對。
            #     真正的不一致是**穩定**的，照樣會被抓出來。
            got, keep, t0_wait, t_end = None, None, time.time(), time.time() + 90.0
            while got is None and time.time() < t_end:
                tel = lk.read_telemetry(timeout_s=1.0, require_new_position=False)
                la, lo = getattr(tel, "lat_deg", None), getattr(tel, "lon_deg", None)
                if la is None or lo is None:
                    continue
                cur = (float(la), float(lo), time.time())
                if keep is not None and cur[2] - keep[2] >= 1.5:
                    dd = math.hypot((cur[0] - keep[0]) * 111132.0,
                                    (cur[1] - keep[1]) * 111412.0
                                    * math.cos(math.radians(cur[0])))
                    if dd <= 0.5:
                        got = (cur[0], cur[1])
                    keep = cur
                elif keep is None:
                    keep = cur
            if got is None:
                raise SystemExit(f"{uav}：起飛前 90 s 內位置沒有收斂 ⇒ 無法確認生成點")
            settle_s = round(time.time() - t0_wait, 1)
            dy = (got[0] - want[0]) * 111132.0
            dx = (got[1] - want[1]) * 111412.0 * math.cos(math.radians(want[0]))
            err = math.hypot(dx, dy)
            site = str(getattr(plan.plans[uav], "takeoff_site", "") or "")
            home_rows.append({"uav": uav, "takeoff_site": site,
                              "want_lat": want[0], "want_lon": want[1],
                              "got_lat": got[0], "got_lon": got[1],
                              "err_m": round(err, 2), "ok": err <= 2.0,
                              "settle_s": settle_s})
            print(f"  {uav} 生成點 {site or '(home)'} 誤差 {err:5.2f} m "
                  f"{'OK' if err <= 2.0 else '✗'}（收斂等了 {settle_s}s）")
            if err > 2.0:
                home_bad.append(f"{uav} 差 {err:.2f} m（站 {site!r}）")
        (out / "spawn_readback.json").write_text(
            json.dumps({"rows": home_rows, "spawn_spread_m": sc.spawn_spread_m,
                        "tol_m": 2.0}, ensure_ascii=False, indent=1), encoding="utf-8")
        if home_bad:
            raise SystemExit("生成點與規劃的起飛站不一致：" + "；".join(home_bad))
        _gaps = [math.hypot((a["got_lat"] - b["got_lat"]) * 111132.0,
                            (a["got_lon"] - b["got_lon"]) * 111412.0
                            * math.cos(math.radians(a["got_lat"])))
                 for i, a in enumerate(home_rows) for b in home_rows[i + 1:]]
        if _gaps:
            print(f"  生成點兩兩最小間距 {min(_gaps):.2f} m"
                  f"（情境設定 {sc.spawn_spread_m:.1f} m）")

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
                # ⚠⚠⚠ 這個逾時**就是**取樣率的上限：`read_telemetry` 的提前結束
                #   條件要 EXTENDED_SYS_STATE 在快取裡，拿不到就每次跑滿逾時
                #   ⇒ 四條鏈路 × 0.25 s = 迴圈固定 1 Hz 牆鐘 = **5 s 模擬**。
                #   三趟不同串流設定都拿到**剛好 164 筆**，就是這裡卡的，不是載具。
                tel = lk.read_telemetry(timeout_s=_POLL_S, require_new_position=False)
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
