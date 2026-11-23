"""HV-DCCPP 四機 SITL 實跑：1 架 Pioneer PNP + 3 架 HERO PNP。

用途
----
把 HV-DCCPP 規劃出來的 4D 任務，用真的 ArduPlane QuadPlane SITL 飛一次，
並且**把 SITL 參數對齊平台參數表的真機值**（每架自己的巡航空速、失速、
滾轉限制、爬升率、轉彎半徑），而不是四架共用一份通用樣板。

為什麼不直接用 mission/sitl_launcher.py
--------------------------------------
工作樹裡那一版的 ``--defaults`` 只掛 ``plane.parm, vtol_default.parm, identity.parm``：
  * ``plane.parm`` 設 ``ARMING_REQUIRE 0`` ⇒ QuadPlane 的四軸馬達永遠停在 PWM 1000，
    看起來有解鎖、實際上飛不起來（實測 0.15 m vs 21.4 m）。修正它的
    ``sitl/vtol/iN/vtol_enforced.parm`` 存在於磁碟，但那一版 launcher 不會掛上去。
  * 它也不支援 per-airframe 參數，四架會共用 ``AIRSPEED_MIN 20``，
    而 HERO 的規劃巡航是 19.0 m/s —— 低於下限，速度指令會被夾掉且不報錯。
  * ``start_multi()`` 沒有 speedup 參數，四機會以 1 倍速跑。
所以這支自己組 ``--defaults`` 鏈，順序是（後蓋前）：
    plane.parm → vtol_default.parm → identity.parm → vtol_enforced.parm → <UAV>.param

用法
----
    python scripts/hv_sitl_flight.py plan     # 只規劃 + 匯出，不起 SITL
    python scripts/hv_sitl_flight.py fly      # 完整：規劃 → 起 SITL → 上傳 → 飛 → 收資料
    python scripts/hv_sitl_flight.py stop     # 收掉殘留的 ArduPlane 行程
"""
from __future__ import annotations

import csv
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

TRIAL = os.environ.get("HV_TRIAL", "HV4SITL1")
OUT = ROOT / "runs" / "artifacts" / TRIAL
SITL_DIR = ROOT / "sitl"
BINARY = SITL_DIR / "ArduPlane.exe"
SPEEDUP = float(os.environ.get("HV_SPEEDUP", "5"))   # 本 repo 唯一有 VTOL AUTO 成功紀錄的倍率

HOME = (23.95, 120.90)
HOME_ALT_MSL = 100.0

# ── 機隊：1 Pioneer(A) + 3 HERO(B) ────────────────────────────────────
FLEET = [
    ("A1", "A", 1, 21.5, 200.0, 224.0),
    ("B1", "B", 2, 19.0, 120.0, 147.0),
    ("B2", "B", 3, 19.0, 120.0, 147.0),
    ("B3", "B", 4, 19.0, 120.0, 147.0),
]

# ── 作業區：LiDAR 區給 Pioneer、RGB 區給三架 HERO ─────────────────────
#   幾何取自 scout 掃描出來、實測 4/4 都有工作且 validate 通過的組合。
AREAS = [
    {"area_id": 1, "priority": 1.0, "sensor_req": "lidar", "payload_req_id": "lidar_req",
     "polygon": [(23.944131, 120.906876), (23.944131, 120.910806),
                 (23.947742, 120.910806), (23.947743, 120.906877)]},
    {"area_id": 2, "priority": 1.0, "sensor_req": "rgb", "payload_req_id": "rgb_req",
     "polygon": [(23.952256, 120.885264), (23.952257, 120.893123),
                 (23.957313, 120.893123), (23.957312, 120.885263)]},
]
ALTERNATES = [(23.972571, 120.90)]


def _drones() -> list[dict]:
    return [{"drone_id": i + 1, "name": n, "hv_class": c, "position": HOME,
             "speed": v, "heading": 0.0, "turn_radius": r,
             "vehicle_type": "vtol", "fov_width": w}
            for i, (n, c, _sysid, v, r, w) in enumerate(FLEET)]


# ══════════════════════════════════════════════════════════════════════
#  1. 規劃 + 匯出
# ══════════════════════════════════════════════════════════════════════
def plan_and_export():
    import hv_dccpp as hv
    from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator
    from hv_dccpp.mission import PayloadRequirement, PlannerConfig

    OUT.mkdir(parents=True, exist_ok=True)
    reqs = {
        "lidar_req": PayloadRequirement("lidar_req", "lidar", density_pt_m2=20.0,
                                        side_overlap=0.25, h_target_agl_m=160.0),
        "rgb_req": PayloadRequirement("rgb_req", "rgb", gsd_required_m=0.05,
                                      side_overlap=0.30, forward_overlap=0.70,
                                      h_target_agl_m=100.0),
    }
    report: dict = {}
    coord = HvDccppCoordinator(
        class_of=lambda d: str(d["hv_class"]),
        cfg=PlannerConfig(dem_step_m=25.0, t_sep_ldg_s=30.0),
        home=HOME, alternates=ALTERNATES, payload_reqs=reqs,
        wind=(5.0, 270.0), c2_range_km=25.0, terrain_clearance_m=36.0,
        waiver_max_agl_m=200.0, dem_margin_m=3000.0,
        on_report=report.update, raise_on_no_go=True)

    t0 = time.perf_counter()
    result = coord.plan_coverage_dccpp(
        areas=AREAS, drones=_drones(), coverage_width_m=224.0,
        overlap_rate=0.25, coverage_altitude=160.0)
    dt = time.perf_counter() - t0

    plan, ctx, cfg = coord.last_plan, coord.last_ctx, coord.cfg

    # ★ 閘門：規劃器可能只用機隊的一部分，而且**不會有任何訊息**。
    working = {u: len(p.strips) for u, p in plan.plans.items()}
    if len(plan.plans) != len(FLEET) or not all(working.values()):
        raise SystemExit(f"不是四架都有工作：{working}（改幾何或機隊再來）")

    cache = hv.build_validation_cache(plan, ctx, budget_only=False)
    v = hv.validate(plan, ctx, cache, cfg)

    print(f"規劃 {dt:.2f}s  makespan {plan.makespan_s:.1f}s  status {report.get('status')}")
    print(f"validate passed={v.passed} conflicts={len(v.conflicts)} uncovered={len(plan.uncovered)}")
    for u, p in sorted(plan.plans.items()):
        print(f"  {u} {p.cls} strips={len(p.strips)} wp={len(p.waypoints)} "
              f"{p.distance_m/1000:.2f}km  t_off={p.t_takeoff_s:.0f}s t_land={p.t_land_s:.0f}s "
              f"E={p.energy_used_wh:.0f}/{p.energy_margin_wh:.0f}Wh")

    plan_dir = OUT / "plan"
    if plan_dir.exists():
        shutil.rmtree(plan_dir)
    # ⚠⚠ 用 'operator' 而不是 'nav_delay'：**實測 ArduPlane 4.8 不執行 NAV_DELAY**。
    #   HV4SITL1 第一趟用 nav_delay，B3 的任務第 1 列是 NAV_DELAY 792 s，
    #   結果它在開機後 53 s 就已經在 21.9 m 高、任務已推進到 seq 2 —— 四架起飛
    #   時間差 3 s，錯開起飛（4D 去衝突的執行機制）完全沒發生，而且沒有任何錯誤。
    #   所以改由地面站控制：任務檔不帶時間，每架各自等到自己的起飛時刻才切 AUTO。
    summary = hv.export_all(plan, ctx, cfg, plan_dir,
                            alt_frame="relative", camera_trigger=True,
                            takeoff_delay="operator")
    (OUT / "validation.json").write_text(
        json.dumps({"passed": v.passed, "conflicts": len(v.conflicts),
                    "coverage_ok": v.coverage_ok, "energy_ok": v.energy_ok,
                    "min_sep_h_m": v.min_sep_h_m, "min_sep_v_m": v.min_sep_v_m},
                   ensure_ascii=False, indent=1), encoding="utf-8")
    (OUT / "report.json").write_text(
        json.dumps({k: report.get(k) for k in
                    ("status", "errors", "warnings", "notes", "uncovered", "timing_ms")},
                   ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    print("匯出:", {u: summary["uavs"][u]["n_items"] for u in summary["uavs"]})
    return plan, ctx, cfg, summary


# ══════════════════════════════════════════════════════════════════════
#  2. 起 SITL（自己組 --defaults，不用 launcher）
# ══════════════════════════════════════════════════════════════════════
def _instance_dir(i: int) -> Path:
    d = SITL_DIR / "vtol" / f"i{i}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _write_identity(i: int, sysid: int) -> Path:
    """身分與模擬器節奏。

    ⚠ MAV_SYSID 才是這版韌體的名字；舊的 SYSID_THISMAV 會被靜默忽略。
    ⚠ SCHED_LOOP_RATE 必須 200：300 會讓 PreArm 擋下（300*1.8 > SITL 陀螺儀 400 Hz）。
    """
    p = _instance_dir(i) / "identity.parm"
    p.write_text(
        "SERIAL0_PROTOCOL=2\nSERIAL1_PROTOCOL=2\n"
        f"MAV_SYSID={sysid}\n"
        "SIM_TERRAIN=0\nTERRAIN_ENABLE=0\n"
        "SCHED_LOOP_RATE=200\nSIM_RATE_HZ=400\n"
        "SIM_DRIFT_SPEED=0\nSIM_DRIFT_TIME=0\n", encoding="utf-8")
    return p


def _write_enforced(i: int) -> Path:
    """保證是 QuadPlane 而且馬達會轉。

    ⚠⚠ ARMING_REQUIRE=1 是關鍵：plane.parm 設 0，那會讓四軸馬達永遠停在 PWM 1000
    —— 畫面上有解鎖、實際上離地 0.15 m。
    """
    p = _instance_dir(i) / "vtol_enforced.parm"
    p.write_text(
        "# 由 scripts/hv_sitl_flight.py 產生。排在 --defaults 的最後之一。\n"
        "Q_ENABLE         1\nQ_FRAME_CLASS    1\nQ_FRAME_TYPE     1\n"
        "SERVO5_FUNCTION  33\nSERVO6_FUNCTION  34\n"
        "SERVO7_FUNCTION  35\nSERVO8_FUNCTION  36\n"
        "ARMING_REQUIRE   1\n", encoding="utf-8")
    return p


def _write_firmware_override(i: int) -> Path:
    """韌體版本相容性覆寫，排在 --defaults 最後。

    ⚠⚠ hv_dccpp 匯出的 ``<UAV>.param`` 同時設 ``RTL_AUTOLAND=2`` 與 ``Q_RTL_MODE=1``，
    但 ArduPlane 4.8.0-dev 認為這兩個互斥，PreArm 直接擋下：
        「PreArm: unset one of RTL_AUTOLAND or Q_RTL_MODE」
    實測兩種解法（同一架、同一份任務）：
        Q_RTL_MODE=1 + RTL_AUTOLAND=0 → 仍被拒：
            「Arm: DO_LAND_START set and RTL_AUTOLAND disabled」（任務裡有 DO_LAND_START）
        RTL_AUTOLAND=2 + Q_RTL_MODE=0 → **解鎖成功**
    所以這裡把 Q_RTL_MODE 關掉。後果：失效保護觸發的 RTL 會走 DO_LAND_START 的
    降落序列（本任務那一段就是垂直降落），而不是 QRTL —— 對這份任務等價。
    匯出端的相容性問題已記在 runs/artifacts/<TRIAL>/README.md。
    """
    p = _instance_dir(i) / "firmware_override.parm"
    p.write_text(
        "# ArduPlane 4.8.0-dev 相容性覆寫（見 scripts/hv_sitl_flight.py 的說明）\n"
        "Q_RTL_MODE 0\n", encoding="utf-8")
    return p


def launch(uavs: list[str]) -> list[dict]:
    procs = []
    lines = []
    for i, (uav, (name, cls, sysid, *_)) in enumerate(zip(uavs, FLEET)):
        work = _instance_dir(i)
        for junk in ("eeprom.bin", "mav.parm"):
            (work / junk).unlink(missing_ok=True)
        defaults = [
            SITL_DIR / "default_params" / "plane.parm",
            SITL_DIR / "default_params" / "vtol_default.parm",
            _write_identity(i, sysid),
            _write_enforced(i),
            OUT / "plan" / f"{uav}.param",          # ★ 每架自己的真機參數
            _write_firmware_override(i),            # 韌體相容性（最後蓋）
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
                      "port": 5760 + 10 * i, "pid": proc.pid, "proc": proc})
        time.sleep(1.0)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "sitl_cmdlines.txt").write_text("\n".join(lines), encoding="utf-8")
    return procs


def stop_all():
    n = 0
    for line in subprocess.run(["tasklist", "/FI", "IMAGENAME eq ArduPlane.exe", "/FO", "CSV"],
                               capture_output=True, text=True).stdout.splitlines():
        m = re.match(r'"ArduPlane\.exe","(\d+)"', line)
        if m:
            subprocess.run(["taskkill", "/PID", m.group(1), "/F"],
                           capture_output=True)
            n += 1
    print(f"已終止 {n} 個 ArduPlane 行程")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "plan"
    if cmd == "plan":
        plan_and_export()
    elif cmd == "stop":
        stop_all()
    else:
        raise SystemExit("用法: plan | fly | stop（fly 由 hv_sitl_fly.py 驅動）")
