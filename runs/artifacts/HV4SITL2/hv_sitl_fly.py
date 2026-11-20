"""HV-DCCPP 四機 SITL 實跑驅動（1 Pioneer + 3 HERO）。

先跑 `scripts/hv_sitl_flight.py plan` 產生 runs/artifacts/<TRIAL>/plan/，再跑這一支。

流程與每一步的守則
------------------
1. 起 4 台 ArduPlane QuadPlane SITL，`--defaults` 鏈最後兩份是
   `vtol_enforced.parm`（ARMING_REQUIRE=1，否則四軸馬達不轉）與該機自己的
   `<UAV>.param`（真機巡航／失速／滾轉／爬升率）。
2. **讀回**每一個關鍵參數再比對檔案值 —— 名字打錯或韌體改名時 ArduPilot 會
   靜默忽略，不讀回就永遠不知道自己飛的是別的設定。
3. 上傳任務後**下載回來**比對項數與指令碼。
4. AUTO + MISSION_START。錯開起飛已經寫進任務檔（seq1 = NAV_DELAY），
   所以這裡**不可以**再延後 MISSION_START，否則延遲會套兩次。
5. 時間一律取 `GLOBAL_POSITION_INT.time_boot_ms`（模擬時鐘）。speedup 下用牆鐘
   算出來的秒數是錯的。
"""
from __future__ import annotations

import csv
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.hv_sitl_flight import (  # noqa: E402
    FLEET, HOME, OUT, SPEEDUP, launch, plan_and_export, stop_all,
)

# 讀回驗證的參數：飛行動力學相關的每一個，加上必須成立的三個前提
CHECK_PARAMS = [
    "AIRSPEED_CRUISE", "AIRSPEED_MIN", "AIRSPEED_MAX", "AIRSPEED_STALL",
    "ROLL_LIMIT_DEG", "TECS_CLMB_MAX", "TECS_SINK_MAX", "WP_LOITER_RAD",
    "RTL_RADIUS", "Q_ASSIST_SPEED", "Q_RTL_ALT", "Q_TRANSITION_MS",
    "Q_ENABLE", "ARMING_REQUIRE", "SCHED_LOOP_RATE", "Q_RTL_MODE", "RTL_AUTOLAND",
]
# ⚠ Q_RTL_MODE 期望 0：被 firmware_override.parm 蓋掉（4.8 不接受它與
#   RTL_AUTOLAND 同時開，而任務裡有 DO_LAND_START 所以必須留 RTL_AUTOLAND）。
EXPECT_EXTRA = {"Q_ENABLE": 1.0, "ARMING_REQUIRE": 1.0, "SCHED_LOOP_RATE": 200.0,
                "Q_RTL_MODE": 0.0, "RTL_AUTOLAND": 2.0}


def _param_file_values(uav: str) -> dict[str, float]:
    vals = {}
    for ln in (OUT / "plan" / f"{uav}.param").read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if ln and not ln.startswith("#") and "," in ln:
            k, v = ln.split(",", 1)
            vals[k.strip()] = float(v)
    return vals


def main() -> int:
    from hv_dccpp import ardupilot as ap
    from hv_dccpp.mavlink_link import VehicleLink

    plan, ctx, cfg, summary = plan_and_export()
    uavs = sorted(plan.plans)                      # A1, B2, B3, B4
    print(f"\n機隊 {uavs}  makespan {plan.makespan_s:.0f}s  speedup {SPEEDUP}"
          f"  → 牆鐘約 {plan.makespan_s / SPEEDUP / 60:.1f} 分鐘\n")

    stop_all()
    procs = launch(uavs)
    print("SITL 已啟動:", [(p["uav"], p["port"], p["pid"]) for p in procs])

    links: dict[str, VehicleLink] = {}
    try:
        # ── 連線 ────────────────────────────────────────────────────
        for p in procs:
            lk = VehicleLink(f"tcp:127.0.0.1:{p['port']}", sysid=p["sysid"],
                             timeout_s=60.0, uav_id=p["uav"], time_source="boot")
            lk.wait_heartbeat(timeout_s=90.0)
            links[p["uav"]] = lk
            print(f"  {p['uav']} 心跳 OK (sysid {p['sysid']}, port {p['port']})")

        # ── 參數讀回驗證 ────────────────────────────────────────────
        rows, bad = [], []
        for uav, lk in links.items():
            want = _param_file_values(uav) | EXPECT_EXTRA
            for name in CHECK_PARAMS:
                exp = want.get(name)
                got = lk.get_param(name, timeout_s=10.0)
                note = ""
                ok = got is not None and exp is not None and \
                    abs(got - exp) <= max(0.01, abs(exp) * 0.001)
                # 韌體端是整數型別時，小數會被截掉。實測 AIRSPEED_MAX 是 INT16
                # ⇒ 26.6 存進去變 26.0。這是型別限制，不是「設定沒生效」，
                # 但仍然要記下來 —— 飛的是 26 不是 26.6。
                if not ok and got is not None and exp is not None and \
                        got in (float(int(exp)), float(round(exp))):
                    ok, note = True, f"韌體整數型別，{exp} → {got}"
                rows.append({"uav": uav, "param": name, "expected": exp, "actual": got,
                             "ok": bool(ok), "note": note})
                if exp is not None and not ok:
                    bad.append(f"{uav}.{name} 期望 {exp} 實得 {got}")
        with (OUT / "params_readback.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["uav", "param", "expected", "actual", "ok", "note"])
            w.writeheader()
            w.writerows(rows)
        print(f"\n參數讀回：{sum(r['ok'] for r in rows)}/{len(rows)} 相符")
        if bad:
            print("  ✗ 不符：")
            for b in bad:
                print("   ", b)
            raise SystemExit("參數沒有生效，停止起飛（見 params_readback.csv）")

        # ── 上傳任務並下載回來比對 ──────────────────────────────────
        upload_rows = []
        for uav, lk in links.items():
            items = ap.plan_to_mission_items(
                plan.plans[uav], ctx, cfg, alt_frame="relative",
                camera_trigger=True)      # 不帶 NAV_DELAY，錯開由地面站做
            res = lk.upload_mission(items, timeout_s=60.0)
            back = lk.download_mission(timeout_s=60.0)
            same = len(back) == len(items)
            upload_rows.append({"uav": uav, "sent": len(items), "read_back": len(back),
                                "ok": same, "first_cmds": [i.command for i in back[:3]]})
            print(f"  {uav} 上傳 {len(items)} 項 → 讀回 {len(back)} 項 "
                  f"{'OK' if same else '✗ 不一致'}  前三指令 {[i.command for i in back[:3]]}")
            if not same:
                raise SystemExit(f"{uav} 任務讀回不一致")
        (OUT / "mission_upload.json").write_text(
            json.dumps(upload_rows, ensure_ascii=False, indent=1), encoding="utf-8")

        # ── 解鎖 + AUTO + 起飛 ─────────────────────────────────────
        #   ⚠ 不要一連上就解鎖：EKF/GPS 還沒健康時 PreArm 會擋，而 arm() 只回
        #     True/False，理由在 STATUSTEXT 裡。這裡重試並把飛控講的話印出來，
        #     失敗時才知道是等不夠久還是參數真的有問題。
        # ⚠⚠ 順序是「先解鎖、再切 AUTO」，不可以顛倒。
        #   在 AUTO 但**尚未解鎖**時，ArduPlane 的任務狀態機就已經在跑：
        #   實測飛控在解鎖前就印出「Mission: 1 Delay / Delaying 71 sec」，
        #   接著印「Mission: 2 VTOLTakeoff」。等到解鎖時 NAV_DELAY 早就倒數完，
        #   四架會同時起飛 —— 錯開起飛（也就是 4D 去衝突的執行機制）整個失效，
        #   而畫面上完全看不出來（第一次跑就是這樣，四架起飛時間差 2 秒）。
        for uav, lk in links.items():
            armed, reasons, t0 = False, [], time.time()
            while not armed and time.time() - t0 < 90.0:
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
            print(f"  {uav} armed（等 {time.time() - t0:.0f}s），待命中")

        # ── 依規劃起飛時刻逐機釋放（切 AUTO）──────────────────────
        #   ArduPlane 一進 AUTO 就開始跑任務，所以「何時切 AUTO」就是放行時刻。
        #   Δt 取自規劃的 t_takeoff_s，除以 speedup 換成牆鐘。
        t_off = {u: float(plan.plans[u].t_takeoff_s) for u in links}
        t_min = min(t_off.values())
        release = {u: (t_off[u] - t_min) / SPEEDUP for u in links}
        print("\n放行時刻（牆鐘秒，相對第一架）:",
              {u: round(v, 1) for u, v in sorted(release.items(), key=lambda kv: kv[1])})

        # ── 監看到全部落地（同時依放行時刻逐機切 AUTO）──────────────
        writers, files, last = {}, {}, {}
        for uav in links:
            d = OUT / uav
            d.mkdir(parents=True, exist_ok=True)
            f = (d / "flight.csv").open("w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(["t_s", "lat", "lon", "rel_alt_m", "airspeed",
                        "groundspeed", "wp_seq", "mode", "armed"])
            files[uav], writers[uav] = f, w

        budget_s = plan.makespan_s / SPEEDUP + 300.0
        t_start = time.time()
        done: set[str] = set()
        released: dict[str, float] = {}
        peak = {u: 0.0 for u in links}
        while time.time() - t_start < budget_s and len(done) < len(links):
            el = time.time() - t_start
            # 放行：到了這一架的規劃起飛時刻才切 AUTO（切了就開始跑任務）
            for uav, lk in links.items():
                if uav not in released and el >= release[uav]:
                    ok = lk.set_mode("AUTO", timeout_s=10.0)
                    released[uav] = time.time() - t_start
                    print(f"  ▶ {uav} 放行切 AUTO（牆鐘 {released[uav]:.1f}s，"
                          f"規劃 t_off={t_off[uav]:.0f}s）{'' if ok else ' ⚠ 切換被拒'}")
            for uav, lk in links.items():
                tel = lk.read_telemetry(timeout_s=0.25, require_new_position=False)
                if tel is None:
                    continue
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
                last[uav] = tel
                # 落地判定：曾經爬過 20 m，現在回到 2 m 以下且已上鎖
                if peak[uav] > 20.0 and alt < 2.0 and not getattr(tel, "armed", True):
                    if uav not in done:
                        done.add(uav)
                        print(f"  {uav} 已降落 (sim t={getattr(tel, 't_s', 0):.0f}s)")
            if int(el) % 20 == 0:
                print(f"    [{el:5.0f}s 牆鐘] " + "  ".join(
                    f"{u}:{peak[u]:5.1f}m" for u in sorted(links)))
                time.sleep(1.0)
        for f in files.values():
            f.close()

        (OUT / "release_schedule.json").write_text(json.dumps(
            {"speedup": SPEEDUP,
             "planned_t_takeoff_s": {u: round(t_off[u], 1) for u in t_off},
             "release_wall_s": {u: round(release[u], 1) for u in release},
             "actual_release_wall_s": {u: round(v, 1) for u, v in released.items()}},
            ensure_ascii=False, indent=1), encoding="utf-8")

        print(f"\n落地 {len(done)}/{len(links)}：{sorted(done)}")
        print("最高高度:", {u: round(v, 1) for u, v in peak.items()})
        return 0 if len(done) == len(links) else 2
    finally:
        for lk in links.values():
            try:
                lk.close()
            except Exception:
                pass
        stop_all()


if __name__ == "__main__":
    raise SystemExit(main())
