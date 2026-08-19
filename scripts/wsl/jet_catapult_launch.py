#!/usr/bin/env python3
"""jet_catapult_launch.py — 6 架 plane-jet-catapult 彈射起飛。

彈射鏈(SIM_Plane.cpp:410 `launch_triggered = input.servos[6] > 1700`):
  SERVO7_FUNCTION=0(Disabled=手動)→ MAV_CMD_DO_SET_SERVO(183) ch7=2000
  → 伺服輸出 7 拉高 → SITL 施加 launch_accel 15 m/s²(2 秒)
  → 前向加速度超過 TKOFF_THR_MINACC(10)→ ArduPlane TAKEOFF 解除馬達抑制 → 爬升。

⚠️ 不能用 RC_CHANNELS_OVERRIDE:那是灌「RC 輸入」,要 SERVO7 設成 RCIN7 直通才會
   傳到輸出;而且沒有 RC 幀時通道維持 trim。實測 override 期間 servo7 仍是 1000。
   DO_SET_SERVO 直接寫輸出,但通道必須是 Disabled,否則每迴圈被指派功能蓋回去。

用法: python3 jet_catapult_launch.py [stagger_s] [climb_wait_s]
"""
import os, sys, time, math
from pymavlink import mavutil

BASE = 6960          # 走節點分流(AeroPlan 佔 6760 家族)
STRIDE = 10
N = int(os.environ.get("N", "6"))   # 可用 N=2 做低負載對照(見 docs)
TAKEOFF_MODE = 13    # ArduPlane TAKEOFF
GUIDED_MODE = 15


def conn(i):
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{BASE + STRIDE * (i - 1)}",
                                   source_system=180 + i, dialect="ardupilotmega")
    c.wait_heartbeat(timeout=10)
    return c


def setp(c, name, val):
    c.mav.param_set_send(c.target_system, 1, name.encode(), float(val),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def fire_catapult(c, ch=7, pwm=2000):
    """MAV_CMD_DO_SET_SERVO(183):直接把伺服輸出通道 ch 壓到 pwm。"""
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                            float(ch), float(pwm), 0, 0, 0, 0, 0)


def servo7(c):
    """讀回伺服輸出 7,驗證彈射器真的被觸發(>1700)。"""
    t0 = time.time(); last = None
    while time.time() - t0 < 1.5:
        m = c.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
        if m:
            last = m
        else:
            time.sleep(0.02)
    return last.servo7_raw if last else None


def alt_of(c):
    last = None; t0 = time.time()
    while time.time() - t0 < 1.2:
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if m:
            last = m
        else:
            time.sleep(0.02)
    return last.relative_alt / 1000.0 if last else 0.0


def main():
    # ⭐ 幾何推導的彈射間隔:STAGGER = V_SPACING / V_G
    #    六機依序離地後,縱向位置自然吻合 V 隊形的 slot 排布,
    #    編隊層不必在初期「瘋狂追趕缺口」。
    #    V_SPACING=4000 m、巡航地速 157 m/s ⇒ 25.48 s,總起飛時間 ~127 s。
    #    ⚠️ 均勻間隔只能對齊 rank 遞增的排列。本專案的 V 是
    #       UAV2/4 同為 rank1(縱距都是 3277 m)、UAV3/5 同為 rank2(6553 m),
    #       UAV6 在 6000 m —— 均勻間隔會給 UAV6 785×T = 20000 m,
    #       比它的 slot 遠 14 km。使用者 2026-07-31 明確指定沿用均勻間隔。
    #    以環境變數覆寫:STAGGER=<秒>,或 STAGGER_GEOM=1 用上式自動推。
    if os.environ.get("STAGGER_GEOM") == "1":
        v_sp = float(os.environ.get("V_SPACING", "4000.0"))
        v_g = float(os.environ.get("FORM_V_GS", "157.0"))
        stagger = v_sp / max(v_g, 1e-6)
        print("[彈射] 幾何推導間隔 = %.0f / %.0f = %.2f s(總起飛 ~%.0f s)"
              % (v_sp, v_g, stagger, stagger * 5), flush=True)
    else:
        # ⚠️ 3.0 s = 彈射器的物理冷卻極限(使用者 2026-07-31 定調),不是隨意值。
        #    實測反證:均勻 25.48 s 的「幾何推導」間隔讓散布爆到 455 s ——
        #    均勻間隔無法對齊本專案的 V(UAV2/4 同 rank 需同縱距,無解),
        #    而 UAV6 被放到 20 km 後方(slot 只在 6 km),得往前追 14 km。
        stagger = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    climb_wait = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    links = {i: conn(i) for i in range(1, N + 1)}

    # 0) 確保彈射觸發鏈接好(SITL 已在跑時 .parm 不會重載 → 這裡補設)
    for i, c in links.items():
        setp(c, "SERVO7_FUNCTION", 0)
    # ⚠️ 低推重比機體(用 THR_MAX 壓過推重比的高速改裝)彈射起飛會卡在邊緣:
    #    `TKOFF_THR_MAX` **突破不了 THR_MAX** —— 後者是伺服輸出的硬上限,前者只能
    #    往下調。實測 THR_MAX=19 時飛機彈出去只能一路彈地(throttle 讀回就是 19)。
    #    彈射器本身就是模擬用的外部助力,起飛推力放寬屬於同一性質;而且離地後就會
    #    收回設計值,巡航與俯衝的物理完全不受影響。
    tkoff_thr = float(os.environ.get("TKOFF_THR_BOOST", "0"))
    cruise_thr = float(os.environ.get("CRUISE_THR_MAX", "0"))
    # ⭐⭐ 火箭助推:爬升段維持高 THR_MAX,爬完巡航高度才收回設計值。
    #
    # ⚠️⚠️ 依據是飛控內部狀態的實測(CTUN/TECS 分段掃描):
    #    整個爬升段油門在 **81–95%** 之間(THR_MAX=21),1400–2000 m 幾乎貼頂;
    #    而 TECS 在該帶把 UAV5/6 的**速度需求本身**從 111 降到 85 —— 那是
    #    能量受限時犧牲速度的標準行為,不是「追不上」。
    #    理論爬升率(96 EAS、THR_MAX 21)是 25.7 m/s,實測只有 8.3–12.2。
    #    ⇒ 機體在爬升時是**推力受限**的,而 THR_MAX=21 是依「平飛巡航 +
    #      追趕權限」反推的,從未涵蓋「六機同時爬 3100 m」這個工況。
    #    助推同時解兩件事:給更多推力,而且讓飛機遠離飽和點 ——
    #    六機才會表現一致(爬升率差異正是飽和後各自取捨的結果)。
    #    THR_MAX=35 ⇒ T/W 0.67、理論爬升率 51.5 m/s、油門餘裕 80%。
    climb_thr = float(os.environ.get("CLIMB_THR_MAX", "0"))
    if tkoff_thr > 0:
        for i, c in links.items():
            setp(c, "THR_MAX", tkoff_thr)
        print(f"起飛推力暫時開到 THR_MAX={tkoff_thr:.0f}"
              f"(離地後收回 {cruise_thr:.0f})", flush=True)
    time.sleep(2.0)

    # 1) 逐機:設模式 → ARM → 點火彈射
    #
    # LAUNCH_MODE=guided:**直接在 GUIDED 起飛**,不經 TAKEOFF 模式。
    #   SITL 的彈射觸發只看 servo7 的 PWM(SIM_Plane.cpp:`input.servos[6] > 1700`),
    #   和飛行模式無關,所以 GUIDED 下一樣會得到那一下推力;ArduPlane 只要有導引
    #   目標就會照著飛。這樣一次拿掉兩件事:
    #     (a) TKOFF_ALT 的天花板 —— TAKEOFF 模式只爬到那個高度就平飛,
    #         想再爬就得換模式再下航點(實測 R1 因此卡在 300 m 整整 600 s)
    #     (b) 模式轉換的能量暫態 —— 低推重比機體在轉換瞬間很容易拿高度換速度
    # LAUNCH_MODE=takeoff(預設):維持原本的 TAKEOFF → GUIDED 兩段式。
    launch_mode = os.environ.get("LAUNCH_MODE", "takeoff").lower()
    climb_to_env = float(os.environ.get("CLIMB_TO_ALT", "0"))
    guided_launch = (launch_mode == "guided" and climb_to_env > 0)
    if guided_launch:
        print(f"起飛模式:GUIDED 直接爬到 {climb_to_env:.0f} m(不經 TAKEOFF)",
              flush=True)
    # ══ 1) 先把六機**全部**準備好(設模式 / 給目標 / 解鎖)══════════════
    #    這一段花多久都無所謂 —— 它不佔彈射間隔的預算。
    armed = []
    for i in range(1, N + 1):
        c = links[i]
        c.mav.set_mode_send(c.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            GUIDED_MODE if guided_launch else TAKEOFF_MODE)
        if guided_launch:
            # ⚠️ GUIDED 起飛必須**先給目標再解鎖**:沒有目標的 GUIDED 解鎖後
            #    飛控沒有航點可飛,彈射出去只會平飛然後掉下來。
            time.sleep(0.3)
            p = None
            t0 = time.time()
            while p is None and time.time() - t0 < 4.0:
                m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
                if m:
                    p = (m.lat / 1e7, m.lon / 1e7)
                else:
                    time.sleep(0.02)
            if p:
                c.mav.command_long_send(
                    c.target_system, 1,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0,
                    float(os.environ.get("CLIMB_TO_V", "70")), -1, 0, 0, 0, 0)
                c.mav.command_int_send(
                    c.target_system, 1,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
                    float("nan"), int((p[0] + 60000.0 / 111320.0) * 1e7),
                    int(p[1] * 1e7), climb_to_env)
        time.sleep(0.5)
        c.mav.command_long_send(c.target_system, 1,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.8)
        armed.append(i)

    # ══ 2) 點火:準備與點火**分離**,用絕對時刻排程 ══════════════════
    # ⚠️⚠️ 舊寫法把「設模式 → 解鎖 → 壓住 ch7 三秒 → sleep(stagger)」全部串在
    #    同一個迴圈裡,所以實際間隔 = 迴圈本體(~5.4 s)+ stagger。
    #    實測(GPS 共同時鐘,GPS.Spd 首次 >20 m/s):
    #      UAV1..6 相對時刻 0 / 7.60 / 16.40 / 25.40 / 34.40 / 42.00 s
    #      ⇒ 間隔 7.6–9.0、平均 **8.40 s**、總跨距 **42.0 s**,而門檻是 3 s。
    #    光是「壓住 ch7 三秒」就吃掉整個 3 s 預算 —— 序列式流程在結構上不可能達標。
    #
    # ⇒ 改成兩段:上面先把六機**全部**設好模式/目標/解鎖(花多久都無所謂),
    #   這裡才用**絕對時刻** t0 + i×stagger 逐一點火,而且各機的 3 s 伺服壓持
    #   **互相重疊**(用 pending 清單每輪掃過所有「還在壓」的機),
    #   不再互相排隊。這樣間隔就等於 stagger 本身。
    # ⚠️ 用絕對時刻而不是累加 sleep:累加會把每輪的處理時間也加進去而漂移。
    HOLD_S = float(os.environ.get("CATAPULT_HOLD_S", "3.0"))
    t_zero = time.time()
    pending = {}          # uav -> 該收回彈射臂的絕對時刻
    fired = {}            # uav -> 實際點火時刻(相對 t_zero)
    nxt = 0
    while nxt < len(armed) or pending:
        now = time.time()
        # (a) 到點就點火(絕對時刻,不累加)
        if nxt < len(armed) and now >= t_zero + nxt * stagger:
            i = armed[nxt]
            fire_catapult(links[i])
            pending[i] = now + HOLD_S
            fired[i] = now - t_zero
            nxt += 1
            continue
        # (b) 所有還在壓持窗內的機**同時**持續送 ch7=2000(重疊,不排隊)
        for i in list(pending):
            if now >= pending[i]:
                s7 = servo7(links[i])
                fire_catapult(links[i], pwm=1000)      # 收回彈射臂
                print(f"UAV{i} 彈射 @+{fired[i]:.2f}s "
                      f"(servo7={s7} {'✓觸發' if (s7 or 0) > 1700 else '✗未觸發'})",
                      flush=True)
                del pending[i]
            else:
                fire_catapult(links[i])
        time.sleep(0.05)
    if len(fired) > 1:
        seq = [fired[i] for i in sorted(fired)]
        gaps = [seq[k] - seq[k - 1] for k in range(1, len(seq))]
        print(f"彈射間隔 {' / '.join('%.2f' % g for g in gaps)} s"
              f"(目標 {stagger:.1f});總跨距 {seq[-1] - seq[0]:.2f} s", flush=True)

    # 2) 等爬升
    # ⚠️ CLIMB_ALT 是「交接給 GUIDED 的高度」。預設 90 是配原廠 plane-jet(T/W 1.43)
    #    訂的;低推重比的改裝機體(THR_MAX 壓過)在轉 GUIDED 的當下還沒到指令速度,
    #    TECS 會拿高度去換速度,90 m 根本不夠賠 —— 實測從 227 m 都能在 11 秒內撞地。
    #    高速組態請一起把 TKOFF_ALT 拉高,並把這個門檻設到 250 以上。
    climb_alt = float(os.environ.get("CLIMB_ALT", "90"))
    t0 = time.time()
    while time.time() - t0 < climb_wait:
        alts = {i: round(alt_of(links[i])) for i in links}
        print(f"climb… {alts}", flush=True)
        if all(a > climb_alt for a in alts.values()):
            print(f"全員爬升過 {climb_alt:.0f}m", flush=True)
            break
        time.sleep(3)

    # 3) 轉 GUIDED —— ⚠️ 必須在爬升段**之前**做。
    #    DO_REPOSITION 在 TAKEOFF 模式下不起作用:那個模式只爬到 TKOFF_ALT 就平飛。
    #    實測 R1 把爬升放在切 GUIDED 之前,六機整整 600 s 都停在 279–508 m
    #    (≈ TKOFF_ALT 300),完全沒有爬升。
    if tkoff_thr > 0 and cruise_thr > 0:
        for i, c in links.items():
            setp(c, "THR_MAX", climb_thr if climb_thr > 0 else cruise_thr)
        time.sleep(2.0)
        print(f"離地後 THR_MAX="
              f"{climb_thr if climb_thr > 0 else cruise_thr:.0f}"
              f"{'(火箭助推,爬完收回 %.0f)' % cruise_thr if climb_thr > 0 else '(設計值)'}",
              flush=True)
    if not guided_launch:
        for i, c in links.items():
            c.mav.set_mode_send(c.target_system,
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                GUIDED_MODE)
        time.sleep(2.0)
    # ⚠️ 這行是 stot_trial.sh 判斷「起飛成功」的標記(`grep -q "全員 GUIDED"`),
    #    **兩條起飛路徑都必須印**。先前只印在 takeoff 路徑裡,LAUNCH_MODE=guided
    #    時六機明明都爬到 3100 m 了卻被判成「彈射失敗」,整趟中止 —— 而且計分器
    #    接著讀到同名的舊 CSV,報出一個和本趟無關的成績。
    print("全員 GUIDED", flush=True)

    # 3b) 爬到巡航高度(CLIMB_TO_ALT)——**必須在協同層接手之前做完**
    # ⚠️ 這一步不能交給編隊節點:`wingman_node` 每 ~1 s 重發一次導引航點(slot 會動),
    #    而每一發 DO_REPOSITION 都會走 set_target_altitude_current() 把高度目標重錨
    #    在當下高度 ⇒ 爬升需求永遠累積不起來。實測 Q1:長機(不受編隊律管)爬到
    #    3936 m,五架僚機卡在 491–1294 m。編隊 slot 會移動,沒辦法用鎖存繞過,
    #    所以只能**先爬完再交棒** —— 之後每秒重錨的就是正確高度,無害。
    climb_to = float(os.environ.get("CLIMB_TO_ALT", "0"))
    if climb_to > 0:
        print(f"爬升到巡航高度 {climb_to:.0f} m(協同層接手前)", flush=True)
        # 每機一個「正前方很遠」的鎖存航點:目標不動 → 只發一次 → 高度斜坡飛得完
        aim = {}
        for i, c in links.items():
            p = None
            t0 = time.time()
            while p is None and time.time() - t0 < 5.0:
                m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
                if m:
                    p = (m.lat / 1e7, m.lon / 1e7)
                else:
                    time.sleep(0.02)
            if p:
                aim[i] = (p[0] + 60000.0 / 111320.0, p[1])   # 正北 60 km
        v_climb = float(os.environ.get("CLIMB_TO_V", "70"))
        for i, c in links.items():
            if i not in aim:
                continue
            setp(c, "SERVO7_FUNCTION", 0)
            c.mav.command_long_send(c.target_system, 1,
                                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                                    0, v_climb, -1, 0, 0, 0, 0)
            c.mav.command_int_send(
                c.target_system, 1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
                float("nan"), int(aim[i][0] * 1e7), int(aim[i][1] * 1e7),
                float(climb_to))
        t0 = time.time()
        budget = float(os.environ.get("CLIMB_TO_TIMEOUT", "600"))
        # ⚠️⚠️ 這個旗標是 2026-08-06 補的,補的是一個**靜默失敗**:
        #    「全員 GUIDED」印在爬升**之前**(見上面 :259),而這個迴圈預算用完
        #    就直接跳出 —— 沒有錯誤輸出、沒有非零退出、沒有失敗標記。
        #    ⇒ stot_trial.sh 的 `grep -q "全員 GUIDED"` 在「爬升根本沒完成」時
        #      照樣通過,帶著爬到一半的機隊往下跑。這正是 Q1 事故的形狀
        #      (只爬到 ~100 m ⇒ 末端誤差 44 km),而當時補的閘門只檢查
        #      「CLIMB_TO_ALT 這個變數有沒有設」,漏的是「有沒有真的爬到」。
        ok_climb = False
        lo = float("nan")
        tol = float(os.environ.get("CLIMB_TO_TOL", "200"))
        while time.time() - t0 < budget:
            alts = {i: alt_of(links[i]) for i in links}
            lo = min(alts.values())
            print(f"  climb→{climb_to:.0f}  最低 {lo:.0f} m  {alts}", flush=True)
            if lo >= climb_to - tol:
                print(f"  全員抵巡航高度(最低 {lo:.0f} m)", flush=True)
                if climb_thr > 0 and cruise_thr > 0:
                    for c in links.values():
                        setp(c, "THR_MAX", cruise_thr)
                    print(f"  火箭助推結束 —— THR_MAX {climb_thr:.0f} → "
                          f"{cruise_thr:.0f}(巡航設計值)", flush=True)
                # ⚠️⚠️ **速度也要交棒回巡航值。** 上面那一發 DO_CHANGE_SPEED(v_climb)
                #    會一直留在飛控上,而爬升值(70 EAS)遠低於巡航值。
                #    實測 T1:六機整段巡航被釘在 84–95 m/s 地速(≈72 EAS ≈ v_climb),
                #    儘管 wingman_node 每 3 s 就送一次 157 EAS —— 僚機因此比長機還慢,
                #    slot 誤差以 ~30 m/s 線性發散(長機直飛、轉率 0.006°/s,不是轉彎問題)。
                #    交棒的狀態要**直接就是巡航狀態**,不要指望下游把它改回來。
                v_done = float(os.environ.get("CLIMB_DONE_V", "0"))
                if v_done > 0:
                    for c in links.values():
                        c.mav.command_long_send(
                            c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                            0, v_done, -1, 0, 0, 0, 0)
                    print(f"  交棒速度 —— DO_CHANGE_SPEED {v_climb:.0f} → "
                          f"{v_done:.0f} EAS(巡航值)", flush=True)
                ok_climb = True
                break
            time.sleep(10)
        if not ok_climb:
            # ⚠️ 一定要**又出聲又非零退出**:上游 stot_trial.sh 兩種判準都有,
            #    只給其中一種還是會被漏掉。
            print(f"✗ 爬升未完成:預算 {budget:.0f}s 用盡,最低 {lo:.0f} m "
                  f"< 目標 {climb_to:.0f} − 容差 {tol:.0f} = {climb_to - tol:.0f} m。"
                  f"帶著爬到一半的機隊往下跑會讓後面整段失去意義"
                  f"(Q1 實測:只爬到 ~100 m ⇒ 末端誤差 44 km)。"
                  f"要放寬請調 CLIMB_TO_TIMEOUT(現 {budget:.0f}s)"
                  f"或 CLIMB_TO_TOL(現 {tol:.0f}m)。", flush=True)
            sys.exit(2)

    # 4) 交給協同層(GUIDED 與收回起飛推力已在 3) 做完)
    for i, c in links.items():
        print(f"UAV{i} alt={alt_of(c):.0f}m", flush=True)


if __name__ == "__main__":
    main()
