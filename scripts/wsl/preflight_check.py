#!/usr/bin/env python3
"""preflight_check.py — 開跑前把「實際生效值」讀回來驗證,不合就擋下航次。

⚠️⚠️ 這支存在的理由:本輪連續踩了 **10 個靜默失敗**,`bash -n` / `ast.parse` /
`colcon build` **一個都抓不到**,因為它們全都是「值合法但語意錯」或
「送出去了但沒生效」。共同結構是:**我們相信自己送出的值,而不去讀回來**。

已知的十個(這支各有對應檢查):
   1. `JET_CDP`/`JET_THRMAX` 為空 ⇒ 高速改裝從未套用,幾十趟跑原廠阻力
   2. `CLIMB_THR_MAX=35` 印了訊息但從未下發(THR_MAX 全程 100)
   3. `catch_speed 157 > AIRSPEED_MAX 150` ⇒ 每發追趕指令被整個拒收
   4. `AIRSPEED_CRUISE` 停在泡棉機的 40 ⇒ 俯衝被 TECS 拉回 40 m/s
   5. `JET_CRUISE=159` 副作用把 `WP_LOITER_RAD` 設成 1488 ⇒ ALIGN 幾何死結
   6. `DIVE_PITCH_MIN=0.0` ⇒ `if < 0.0` 不成立 ⇒ 放開低頭權限的程式碼從未執行
   7. `tot_node` 從未斷言 `THR_MAX` ⇒ 俯衝「全油門」只有巡航的 25%
   8. `CLIMB_TO_ALT` 沒設 ⇒ 只爬到 100 m 就交棒協同層
   9. `timeout` 續行裡夾註解 ⇒ 保護從未生效;修好後參數又過期
  10. `stot_fast.env` 從未被 source ⇒ 混血組態,零警告

⇒ 規則:**每一項都從飛控讀回來比對**,而且**檢查關係不只檢查值** ——
   壞掉的從來不是單一參數,是參數之間的關係(#3 #5 #7 都是關係)。

用法:
    python3 preflight_check.py                    # 讀 env 的期望值
    python3 preflight_check.py --json             # 機器可讀
    STRICT=0 python3 preflight_check.py           # 只警告不擋(不建議)
"""
import json
import math
import os
import sys

BASE = int(os.environ.get("SAR_NODE_CMD_BASE", "5762"))
STRIDE = int(os.environ.get("SAR_NODE_CMD_STRIDE", "10"))
# ⚠️⚠️ 機數。COUNT 優先,其次 SWARM_COUNT(批次鏈用的名字),最後才是 6。
#    2026-08-06 之前只認 COUNT,而 stot_trial.sh 呼叫時沒傳 ⇒ 兩機測試會逐一
#    去連 UAV3~6 的 5782/5792/5802/5812,四個 ConnectionRefused 直接讓
#    「開跑前驗證未通過 —— 中止」。幾何檢查全過卻卡在機數,歸因完全看不出來。
N = int(os.environ.get("COUNT") or os.environ.get("SWARM_COUNT") or "6")
STRICT = os.environ.get("STRICT", "1") != "0"

# 讀回來要比對的參數。None = 只記錄不比對。
WANT = {
    "THR_MAX":          ("JET_THRMAX", 25.0),
    "AIRSPEED_MAX":     ("AIRSPEED_MAX_CAP", 230.0),
    "AIRSPEED_CRUISE":  (None, None),      # 由 JET_CRUISE×0.85 導出,只記錄
    "AIRSPEED_MIN":     (None, 25.0),
    "ARSPD_OPTIONS":    (None, 0.0),       # 0 = 不自動停用空速計
    "WP_LOITER_RAD":    (None, None),      # 關係檢查在下面
    "PTCH_LIM_MIN_DEG": (None, None),      # 巡航段值;終端由 tot_node 放開
    "TECS_CLMB_MAX":    (None, None),
    "ROLL_LIMIT_DEG":   (None, None),
    # ⭐ 實驗 A:速度指令權限的總開關。0 = 感測器被拒收時 TECS 完全不看空速
    #    ⇒ DO_CHANGE_SPEED 零權限(R451 實測 TEC3/TECS = 0/6400)。
    "TECS_SYNAIRSPEED": ("TECS_SYNAIRSPEED", 1.0),
}


def env_f(name, default):
    v = os.environ.get(name, "")
    try:
        return float(v)
    except ValueError:
        return default


def sigma(alt):
    return max(1e-3, (1.0 - 2.25577e-5 * alt) ** 4.25588)


def read_params(uav):
    """從直連飛控鏈路讀回參數。回傳 dict 或 None(連不上)。"""
    from pymavlink import mavutil
    port = BASE + STRIDE * (uav - 1)
    try:
        c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
        c.wait_heartbeat(timeout=8)
    except Exception as e:
        return None, f"連線失敗 {port}: {e!r}"
    out = {}
    for name in WANT:
        try:
            c.mav.param_request_read_send(c.target_system, 1, name.encode(), -1)
            m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
            if m is not None:
                out[m.param_id.strip("\x00")] = float(m.param_value)
        except Exception:
            pass
    try:
        c.close()
    except Exception:
        pass
    return out, None


def main():
    as_json = "--json" in sys.argv
    fails, warns, rows = [], [], []

    # ── 1) 機體改裝檔是否真的存在 ──────────────────────────────────
    mj = os.path.expanduser("~/fixwing_ws/sitl_run/jet_model.json")
    cdp_env = os.environ.get("JET_CDP", "")
    if cdp_env:
        if not os.path.exists(mj):
            fails.append(f"JET_CDP={cdp_env} 但 {mj} 不存在 ⇒ 高速改裝**未套用**"
                         "(swarm_sitl_up.sh 只在 JET_CDP 非空時寫這個檔)")
        else:
            got = json.load(open(mj)).get("c_drag_p")
            if abs(float(got) - float(cdp_env)) > 1e-9:
                fails.append(f"jet_model.json 的 c_drag_p={got} ≠ JET_CDP={cdp_env}")
            else:
                rows.append(("c_drag_p", got, cdp_env, "OK"))
    else:
        warns.append("JET_CDP 未設 ⇒ 跑的是**原廠 plane-jet 阻力**,頂速 ~98.5 EAS")

    # ── 2) 逐機讀回飛控參數 ────────────────────────────────────────
    per = {}
    for i in range(1, N + 1):
        p, err = read_params(i)
        if p is None:
            fails.append(f"UAV{i} {err}")
            continue
        per[i] = p
    if not per:
        print("✗ 一架都連不上 —— SITL 沒起來?")
        return 2

    for name, (env_name, default) in WANT.items():
        vals = {i: per[i].get(name) for i in per if name in per[i]}
        if not vals:
            warns.append(f"{name} 六機都讀不到(此 build 可能沒有這個參數)")
            continue
        uniq = sorted(set(round(v, 4) for v in vals.values()))
        got = uniq[0]
        if len(uniq) > 1:
            fails.append(f"{name} 六機不一致:{uniq}")
        exp = env_f(env_name, default) if env_name else default
        if exp is None:
            rows.append((name, got, "—", "記錄"))
            continue
        ok = abs(got - exp) < 1e-6
        rows.append((name, got, exp, "OK" if ok else "✗"))
        if not ok:
            fails.append(f"{name} 生效值 {got} ≠ 期望 {exp}"
                         f"{f'(env {env_name})' if env_name else ''}")

    ref = per[sorted(per)[0]]

    # ── 3) ⭐ 關係檢查(壞掉的都是關係,不是單一值)────────────────
    a_min = ref.get("AIRSPEED_MIN", 0.0)
    a_max = ref.get("AIRSPEED_MAX", 0.0)

    catch = env_f("CATCH_SPEED", 0.0) or env_f("FORM_CATCH", 0.0) or 157.0
    if not (a_min <= catch <= a_max):
        fails.append(
            f"⚠️ catch_speed={catch:.0f} 不在 [{a_min:.0f}, {a_max:.0f}] ⇒ "
            f"ArduPlane 會**整發拒收** DO_CHANGE_SPEED(MAV_RESULT=4),"
            f"追趕通道等於關閉")

    # ALIGN 死結:WP_LOITER_RAD×eas2tas² 必須小於 reach,否則永遠抵達不了
    wlr = ref.get("WP_LOITER_RAD")
    reach = env_f("REACH", 260.0)
    perch = env_f("PERCH_ALT", 3100.0)
    if wlr:
        eff = wlr / sigma(perch)          # eas2tas² = 1/σ
        rows.append(("繞圈半徑(實效)", round(eff), round(reach),
                     "OK" if eff < reach else "✗"))
        if eff >= reach:
            fails.append(
                f"⚠️⚠️ ALIGN 幾何死結:WP_LOITER_RAD {wlr:.0f} × eas2tas² "
                f"= {eff:.0f} m ≥ reach {reach:.0f} m。ALIGN 的推進條件是 "
                f"`d < reach`,而 param3=0 讓飛機以 {eff:.0f} m 半徑繞圈 ⇒ "
                f"**永遠湊不滿 6 架,沒有 timeout、沒有錯誤訊息**"
                f"(實測 G1/HS1/NEW1/V901 四趟)")

    # 俯衝幾何:pushover_r 要落在「飛得出來」與「扇形內」之間
    ang = env_f("DIVE_ANGLE", 27.0)
    r_fan = env_f("R_FAN", 35000.0)
    if 0.0 < ang < 89.9:
        drop = perch + 47.0               # 實測 z_g ≈ −47 m
        r_po = drop / math.tan(math.radians(ang))
        v_ring = 150.0                    # 環上實測 TAS 量級
        r_min = v_ring ** 2 / (9.80665 * math.tan(math.radians(60.0)))
        rows.append(("pushover_r", round(r_po), f"{r_min:.0f}<r<{r_fan:.0f}",
                     "OK" if r_min < r_po < r_fan else "✗"))
        if r_po <= r_min:
            fails.append(f"⚠️ 俯角 {ang:.0f}° ⇒ pushover_r {r_po:.0f} m "
                         f"< 物理最小迴轉半徑 {r_min:.0f} m(150 m/s @60°)⇒ "
                         f"飛機對不進環,DIVE 永遠不會發生")
    elif ang >= 89.9:
        fails.append(f"⚠️ 俯角 {ang:.0f}° ⇒ tan(γ) 奇異,pushover_r 會塌成 0 "
                     f"並在幾何式裡溢位(實測 V901 印出 −4.2e18)")

    # ⭐⭐ 高度閘門的死區(檔頭第 8 項的續集,而且更隱蔽)。
    #
    # 彈射端(jet_catapult_launch.py)宣告「全員抵巡航高度」的條件是
    #     min_alt >= CLIMB_TO_ALT − CLIMB_TO_TOL
    # 而編隊層(wingman_node / master_node 的 cruise_alt_threshold)判定
    # 「已進巡航」的條件是
    #     alt >= CRUISE_ALT_THRESHOLD
    # 兩個數字先前是**各自獨立寫死**的(3100−200=2900 vs 3000)⇒ 中間 100 m
    # 的區間裡,彈射端說「爬完了、交棒」,編隊層卻仍判定「還在爬升」而不做
    # 協同讓速 —— 而且完全靜默,兩邊各自都覺得自己是對的。
    # ⇒ 這是「關係」而不是「值」,所以只驗單一參數永遠抓不到。
    c_alt = env_f("CLIMB_TO_ALT", 0.0)
    if c_alt > 0.0:
        c_tol = env_f("CLIMB_TO_TOL", 200.0)          # python 端預設,四支腳本都沒設過
        c_thr = env_f("CRUISE_ALT_THRESHOLD", 3000.0)
        floor = c_alt - c_tol                          # 彈射端保證的最低高度
        rows.append(("CRUISE_ALT_THRESHOLD", round(c_thr),
                     f"<= {floor:.0f}(=CLIMB_TO_ALT−TOL)",
                     "OK" if c_thr <= floor else "✗"))
        if c_thr > floor:
            fails.append(
                f"⚠️⚠️ 高度閘門死區 {c_thr - floor:.0f} m:"
                f"CRUISE_ALT_THRESHOLD={c_thr:.0f} > 彈射端保證的下限 {floor:.0f}"
                f"(= CLIMB_TO_ALT {c_alt:.0f} − CLIMB_TO_TOL {c_tol:.0f})。"
                f"彈射端會宣告『全員抵巡航高度』並交棒,而編隊層仍判定在爬升 ⇒ "
                f"協同讓速不啟動,且無任何訊息。"
                f"處置:調低 CRUISE_ALT_THRESHOLD、調小 CLIMB_TO_TOL,"
                f"或提高 CLIMB_TO_ALT。")
        # 遲滯要蓋得住抖動,但不可以大到把閘門整個吃掉
        hyst = env_f("CRUISE_GATE_HYST", 0.0)
        if hyst > 0.0 and c_thr - hyst < 0.0:
            fails.append(f"⚠️ CRUISE_GATE_HYST={hyst:.0f} 大於閘門 {c_thr:.0f} "
                         f"⇒ 退出線為負,閘門一旦進入就永遠不會退出")

    # ⭐ DO_REPOSITION 的 param3:必須 > 0,否則飛控退回 WP_LOITER_RAD 盤旋
    rr = env_f("REPOSITION_RADIUS", 30.0)
    rows.append(("REPOSITION_RADIUS", rr, "> 0", "OK" if rr > 0.0 else "✗"))
    if rr <= 0.0:
        fails.append(
            f"⚠️⚠️ REPOSITION_RADIUS={rr} ⇒ DO_REPOSITION 的 param3 不通過 "
            f"ArduPlane 的 `param3 > 0` 檢查(GCS_MAVLink_Plane.cpp:599)"
            f"⇒ 退回 update_loiter(0) ⇒ WP_LOITER_RAD {wlr or 1488:.0f}"
            f"×eas2tas² ≈ {(wlr or 1488) / sigma(perch):.0f} m 盤旋圈,"
            f"**飛機會繞著目標點轉而不是飛過去**,橫向誤差有硬地板")

    # 低頭權限:DIVE_PITCH_MIN 必須 < 0,否則 tot_node 的放開區塊整段不執行
    dpm = env_f("DIVE_PITCH_MIN", 0.0)
    rows.append(("DIVE_PITCH_MIN", dpm, "< 0", "OK" if dpm < 0.0 else "✗"))
    if dpm >= 0.0:
        fails.append(
            f"⚠️ DIVE_PITCH_MIN={dpm} ⇒ tot_node 的 `if dive_pitch_min < 0.0:` "
            f"不成立 ⇒ **放開低頭/抬頭權限的整段程式碼不會執行**,"
            f"PTCH_LIM_MIN_DEG 會停在巡航值,實際航跡角只有 ~16°"
            f"(實測 DV1:ATT.DesPitch 全程最小 −20.0°)")

    # ── 4) 輸出 ────────────────────────────────────────────────────
    if as_json:
        print(json.dumps({"rows": rows, "fails": fails, "warns": warns},
                         ensure_ascii=False, indent=1))
    else:
        print("══ 開跑前參數驗證(從飛控讀回)══")
        print(f"{'參數':<22}{'生效值':>12}{'期望':>16}  判定")
        for n, g, e, s in rows:
            print(f"{n:<22}{str(g):>12}{str(e):>16}  {s}")
        for w in warns:
            print(f"  ⚠️ {w}")
        for f in fails:
            print(f"  ✗ {f}")
    if fails:
        print(f"\n✗ {len(fails)} 項不合 —— **不要開跑**" if not as_json else "",
              file=sys.stderr)
        return 1 if STRICT else 0
    print("\n✅ 全部通過" if not as_json else "")
    return 0


if __name__ == "__main__":
    sys.exit(main())
