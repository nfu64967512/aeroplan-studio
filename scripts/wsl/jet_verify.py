#!/usr/bin/env python3
"""jet_verify.py — 單機驗證改裝後的 SITL 噴射模型是否落在設計的速度上。

量兩件事,都用**慣性速度**(GLOBAL_POSITION_INT 的 vx/vy/vz),不用空速:
  - 空速表在高空報的是 EAS,和設計用的真空速差 sqrt(ρ0/ρ)(6000 m 差 35%),
    拿它比對會得出錯誤結論。
  - 而且驗收基準本來就是地速,量同一個東西才對得起來。

流程:
  1. 彈射起飛(沿用 jet_catapult_launch.py 的觸發鏈)
  2. 爬到 --alt,平飛全速 → 量**平飛頂速**
  3. 用 SET_ATTITUDE_TARGET 壓一個俯角、油門全開,閉迴路把航路角收到 --angle
     → 量**俯角終端速度**
  4. 兩者都和點質量模型的預測值並排列出(含當地密度修正)

用法:
  python3 jet_verify.py --alt 6000 --angle 20
  python3 jet_verify.py --alt 6000 --angle 20 --link 6960     # 走分流
"""
import argparse
import math
import time

from pymavlink import mavutil

G = 9.80665
MASS = 22.0
HOVER_THROTTLE = 0.7
S = 0.45
B = 1.88
OSWALD = 0.9
W = MASS * G
THRUST_FULL = MASS * G / HOVER_THROTTLE
AR = B * B / S

TAKEOFF_MODE = 13
GUIDED_MODE = 15
EARTH = 111320.0


def rho_at(alt_m):
    return 1.225 * (1.0 - 2.25577e-5 * alt_m) ** 4.2586


def predict(v_guess, gamma_deg, thr_frac, cdp, alt_m):
    """解 T + W·sinγ = D(V) 的平衡速度(牛頓法,D 含誘導阻力)。"""
    rho = rho_at(alt_m)
    F = thr_frac * THRUST_FULL + W * math.sin(math.radians(gamma_deg))
    k_i = W * W / (0.5 * rho * S * math.pi * OSWALD * AR)
    k_p = 0.5 * rho * S * cdp
    v = max(20.0, v_guess)
    for _ in range(200):
        d = k_p * v * v + k_i / (v * v)
        dd = 2 * k_p * v - 2 * k_i / (v ** 3)
        if abs(dd) < 1e-9:
            break
        step = (d - F) / dd
        v -= max(-20.0, min(20.0, step))
        v = max(10.0, v)
    return v


class Air:
    def __init__(self, port):
        self.c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}",
                                            source_system=201,
                                            dialect="ardupilotmega")
        self.c.wait_heartbeat(timeout=15)
        self.c.mav.request_data_stream_send(
            self.c.target_system, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.pos = None      # (lat, lon, rel_alt)
        self.vel = None      # (vn, ve, vd)

    def pump(self, secs=0.0):
        t0 = time.time()
        while True:
            m = self.c.recv_match(type=["GLOBAL_POSITION_INT"], blocking=False)
            if m:
                self.pos = (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)
                self.vel = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
            elif time.time() - t0 >= secs:
                return
            else:
                time.sleep(0.01)

    def speed(self):
        if not self.vel:
            return 0.0, 0.0, 0.0
        vn, ve, vd = self.vel
        vh = math.hypot(vn, ve)
        v = math.hypot(vh, vd)
        gam = math.degrees(math.atan2(-vd, max(vh, 1e-3)))
        return v, vh, gam

    def param(self, name, val):
        self.c.mav.param_set_send(self.c.target_system, 1, name.encode(),
                                  float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def mode(self, m):
        self.c.mav.set_mode_send(
            self.c.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m)

    def arm(self):
        self.c.mav.command_long_send(
            self.c.target_system, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    def is_armed(self, secs=2.0):
        t0 = time.time()
        armed = None
        while time.time() - t0 < secs:
            m = self.c.recv_match(type=["HEARTBEAT", "STATUSTEXT"],
                                  blocking=False)
            if m is None:
                time.sleep(0.02)
                continue
            if m.get_type() == "STATUSTEXT":
                print("    [FC] %s" % m.text, flush=True)
            else:
                armed = bool(m.base_mode
                             & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return armed

    def arm_until(self, mode_id, tries=20):
        """⚠️ 不能盲送一次 ARM 就往下走。

        `swarm_sitl_up.sh` 印出「ports up」只代表 TCP 在監聽,不代表飛控就緒 ——
        EKF/GPS 還沒收斂時 ARM 會被 PreArm 擋掉,而擋掉是**靜默**的:飛機留在地上,
        後面的爬升迴圈就一路讀到 alt=-0、v=0,看起來像「爬不上去」而不是「沒解鎖」。
        所以這裡重試到 HEARTBEAT 真的回報 armed 為止,並把 PreArm 訊息印出來。
        """
        for k in range(tries):
            self.mode(mode_id)
            time.sleep(0.5)
            self.arm()
            if self.is_armed(3.0):
                print("  已解鎖(第 %d 次嘗試)" % (k + 1), flush=True)
                return True
            print("  解鎖未成功,重試 %d/%d…" % (k + 1, tries), flush=True)
            time.sleep(3.0)
        return False

    def servo(self, ch, pwm):
        self.c.mav.command_long_send(
            self.c.target_system, 1, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            float(ch), float(pwm), 0, 0, 0, 0, 0)

    def goto(self, lat, lon, alt):
        self.c.mav.command_int_send(
            self.c.target_system, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
            float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))

    def speed_cmd(self, v):
        self.c.mav.command_long_send(
            self.c.target_system, 1,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, float(v), -1,
            0, 0, 0, 0)

    def attitude(self, roll, pitch, thr):
        """type_mask 132:韌體會先把 mask 反相再解讀,132 → roll/pitch 生效 + 接管油門。"""
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        self.c.mav.set_attitude_target_send(
            0, self.c.target_system, 1, 132,
            [cr * cp, sr * cp, cr * sp, -sr * sp], 0.0, 0.0, 0.0, float(thr))


def dest(lat, lon, brg, d):
    b = math.radians(brg)
    return (lat + d * math.cos(b) / EARTH,
            lon + d * math.sin(b) / (EARTH * math.cos(math.radians(lat))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", type=int, default=6960)
    ap.add_argument("--alt", type=float, default=6000.0, help="測試高度(相對 HOME)")
    ap.add_argument("--angle", type=float, default=20.0, help="俯衝設計角(度)")
    ap.add_argument("--cdp", type=float, default=None,
                    help="模型的 c_drag_p(用來算預測值;不給就只報實測)")
    ap.add_argument("--thrmax", type=float, default=None,
                    help="THR_MAX 百分比(同上)")
    ap.add_argument("--handover-alt", type=float, default=250.0,
                    help="爬到這個高度才收回起飛推力並轉 GUIDED(要 < TKOFF_ALT)")
    ap.add_argument("--tkoff-thr", type=float, default=0.0,
                    help="起飛期間暫時把 THR_MAX 開到這個值(0 = 不動)。"
                         "⚠️ TKOFF_THR_MAX **突破不了 THR_MAX** —— 它是伺服輸出的"
                         "硬上限,TKOFF_THR_MAX 只能往下調。低推重比機體被 THR_MAX "
                         "夾住之後彈射起飛會在邊緣(實測 c_drag_p 0.01150 勉強成功、"
                         "0.01250 就一路彈地起不來)。離地後會自動收回 --thrmax。")
    ap.add_argument("--climb-speed", type=float, default=80.0,
                    help="爬升空速。要用最佳爬升速度,而且**必須是機體做得到的**")
    ap.add_argument("--level-secs", type=float, default=420.0,
                    help="平飛測速的**上限**秒數;實際上收斂就會提早停")
    ap.add_argument("--dive-secs", type=float, default=70.0)
    a = ap.parse_args()

    air = Air(a.link)
    air.pump(1.0)
    print("已連線,開始彈射", flush=True)

    air.param("SERVO7_FUNCTION", 0)
    if a.tkoff_thr > 0:
        air.param("THR_MAX", a.tkoff_thr)
        print("  起飛推力暫時開到 THR_MAX=%.0f(離地後收回 %.0f)"
              % (a.tkoff_thr, a.thrmax or 0), flush=True)
    time.sleep(2.0)
    if not air.arm_until(TAKEOFF_MODE):
        raise SystemExit("解不了鎖(見上面的 PreArm 訊息),不繼續")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        air.servo(7, 2000)
        time.sleep(0.2)
    air.servo(7, 1000)

    # ── 爬到測試高度 ────────────────────────────────────────────────
    # ⚠️ 要等到接近 TKOFF_ALT 才交接。低推重比機體在「收回起飛推力 + 轉 GUIDED」的
    #    當下會有好幾秒的能量暫態,交接高度不夠就直接賠進地面(實測在 99 m 交接 →
    #    99 → 55 → −21)。TKOFF_ALT 設 300,這裡等到 250。
    t0 = time.time()
    while time.time() - t0 < 180.0:
        air.pump(2.0)
        if air.pos and air.pos[2] > a.handover_alt:
            break
    if not air.pos or air.pos[2] < a.handover_alt * 0.7:
        raise SystemExit(
            "彈射後沒爬到交接高度(alt=%.0f m,要 %.0f)。⚠️ TKOFF_THR_MAX 突破不了 "
            "THR_MAX —— 要用 --tkoff-thr 直接把 THR_MAX 開大。"
            % (air.pos[2] if air.pos else -1, a.handover_alt))
    # ⚠️ 收回起飛推力**必須在轉 GUIDED 之前**,否則量到的巡航頂速是假的。
    if a.tkoff_thr > 0 and a.thrmax:
        air.param("THR_MAX", a.thrmax)
        time.sleep(2.0)
        print("  已收回 THR_MAX=%.0f(設計值)" % a.thrmax, flush=True)
    air.mode(GUIDED_MODE)
    air.pump(1.0)
    home = air.pos[:2]
    print("已 GUIDED,alt=%.0f m,爬升到 %.0f m" % (air.pos[2], a.alt), flush=True)

    far = dest(home[0], home[1], 0.0, 200000.0)
    # ⚠️ 爬升時**不可以**下一個達不到的空速。TECS_SPDWEIGHT=1 之下,要求 250 m/s
    #    而機體只做得到 ~140 時,TECS 會拿高度去換速度 —— 實測從 227 m 一路壓到
    #    撞地(alt 227 → −30)。舊組態不會炸是因為 AIRSPEED_MAX 150 把指令夾住了,
    #    高速改裝把上限放寬到 260 之後就沒有東西夾它。
    #    爬升用「最佳爬升速度」:剩餘功率 V·(T−D)/W 在 ~80 m/s 附近最大(~13.8 m/s)。
    # ⚠️ 先「平飛把速度建立起來」再要求爬升。轉 GUIDED 的瞬間飛機通常還沒到指令
    #    速度,而低推重比機體光靠油門補速度要好幾秒 —— 若同時又壓著一個幾千公尺的
    #    高度需求,TECS 會兩件事一起要,結果就是拿高度換速度往下衝。
    v0 = air.pos[2]
    t0 = time.time()
    while time.time() - t0 < 45.0:
        air.goto(far[0], far[1], v0)
        air.speed_cmd(a.climb_speed)
        air.pump(2.0)
        v, vh, gam = air.speed()
        if v >= a.climb_speed - 3.0:
            print("  速度已建立(%.1f m/s @ alt %.0f),開始爬升"
                  % (v, air.pos[2]), flush=True)
            break
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < 900.0:
        air.goto(far[0], far[1], a.alt)
        air.speed_cmd(a.climb_speed)
        air.pump(3.0)
        v, vh, gam = air.speed()
        if time.time() - last > 20.0:
            print("  climb… alt=%.0f v=%.1f" % (air.pos[2], v), flush=True)
            last = time.time()
        if air.pos[2] < 60.0 and time.time() - t0 > 60.0:
            raise SystemExit("爬升中掉高度到 %.0f m —— 檢查空速指令是否超出機體能力"
                             % air.pos[2])
        if air.pos[2] >= a.alt - 60.0:
            break

    # ── 1) 平飛頂速 ────────────────────────────────────────────────
    # ⭐ 用 TECS_SPDWEIGHT=0 逼出全油門:俯仰只追高度、空速完全交給油門,
    #    於是油門會頂在 THR_MAX,飛機自然收斂到「這個推力下的平飛平衡速度」。
    #    這正是我們要量的頂速,而且不會像 SPDWEIGHT=1 那樣把高度換掉。
    air.param("TECS_SPDWEIGHT", 0.0)
    time.sleep(2.0)
    # ⚠️ 固定秒數會量到「加速途中」而不是頂速。實測 90 s 結束時速度還在以每 2 s
    #    約 1 m/s 上升 —— 兩趟不同 c_drag_p 量到 173.5 / 172.7,阻力改了 8.7% 卻
    #    看不出差別,就是因為兩個都沒收斂。改成**跑到收斂為止**:最後 30 s 的
    #    速度變化小於 0.3 m/s 才停。
    print("── 平飛頂速測試(收斂才停,上限 %.0f s,TECS_SPDWEIGHT=0)──"
          % a.level_secs, flush=True)
    t0 = time.time()
    samples = []
    while time.time() - t0 < a.level_secs:
        air.goto(far[0], far[1], a.alt)
        air.speed_cmd(250.0)
        air.pump(2.0)
        v, vh, gam = air.speed()
        samples.append((time.time(), air.pos[2], v))
        print("  alt=%.0f  V=%.1f  γ=%+.1f°" % (air.pos[2], v, gam), flush=True)
        old = [s for s in samples if s[0] <= time.time() - 30.0]
        if old and len(samples) > 20:
            drift = v - old[-1][2]
            if abs(drift) < 0.3:
                print("  已收斂(最後 30 s 只變化 %+.2f m/s)" % drift, flush=True)
                break
    tail = samples[-8:] if len(samples) >= 8 else samples
    v_level = sum(s[2] for s in tail) / len(tail)
    alt_level = sum(s[1] for s in tail) / len(tail)
    if samples and samples[-1][0] - t0 >= a.level_secs - 3:
        print("  ⚠️ 到上限仍未收斂,下面的頂速是低估值", flush=True)

    # ── 2) 俯角終端速度 ────────────────────────────────────────────
    # ⚠️ 低頭上限在巡航段被壓在 −20°(否則 TECS 會拿高度換速度把飛機開進地面),
    #    要測俯衝就得先放開 —— 這正是 tot_node 進 DIVE 時做的事(dive_pitch_min)。
    #    不放開的話姿態指令會被夾住,量到的是 −17° 而不是 −20° 的終端速度。
    for name in ("TECS_PITCH_MIN", "PTCH_LIM_MIN_DEG"):
        air.param(name, -80.0)
    time.sleep(2.0)
    print("── 俯衝終端測試:目標航路角 −%.0f°,油門全開(已放開低頭限制)──"
          % a.angle, flush=True)
    pitch = math.radians(-a.angle)
    t0 = time.time()
    dsamp = []
    while time.time() - t0 < a.dive_secs:
        air.attitude(0.0, pitch, 1.0)
        air.pump(0.1)
        v, vh, gam = air.speed()
        # 閉迴路把航路角收到目標:俯仰 = 航路角 + 攻角,攻角現場補
        err = (-a.angle) - gam
        pitch += math.radians(0.02 * err)
        pitch = max(math.radians(-70.0), min(math.radians(10.0), pitch))
        dsamp.append((air.pos[2], v, gam))
        if len(dsamp) % 40 == 0:
            print("  alt=%.0f  V=%.1f  γ=%+.1f°  pitch=%+.1f°"
                  % (air.pos[2], v, gam, math.degrees(pitch)), flush=True)
        if air.pos[2] < 400.0:
            print("  高度用完,停止", flush=True)
            break
    # 取最後 5 秒且航路角已收斂到目標 ±3° 的樣本
    good = [s for s in dsamp[-120:] if abs(s[2] + a.angle) < 3.0]
    v_dive = (sum(s[1] for s in good) / len(good)) if good else float("nan")
    alt_dive = (sum(s[0] for s in good) / len(good)) if good else float("nan")

    # ── 報告 ──────────────────────────────────────────────────────
    print("\n══════ 結果 ══════")
    print("平飛頂速   實測 %.1f m/s  @ alt %.0f m" % (v_level, alt_level))
    if a.cdp and a.thrmax:
        p = predict(v_level, 0.0, a.thrmax / 100.0, a.cdp, alt_level)
        print("           預測 %.1f m/s   (差 %+.1f)" % (p, v_level - p))
        p0 = predict(136.0, 0.0, a.thrmax / 100.0, a.cdp, 0.0)
        print("           換算海平面 %.1f m/s" % p0)
    if good:
        print("俯衝終端   實測 %.1f m/s  @ alt %.0f m  γ=−%.0f°"
              % (v_dive, alt_dive, a.angle))
        if a.cdp and a.thrmax:
            p = predict(v_dive, -a.angle, a.thrmax / 100.0, a.cdp, alt_dive)
            print("           預測 %.1f m/s   (差 %+.1f)" % (p, v_dive - p))
            print("           地速 = V·cos γ = %.1f m/s"
                  % (v_dive * math.cos(math.radians(a.angle))))
    else:
        print("俯衝終端   航路角沒有收斂到 −%.0f°±3°,無有效樣本" % a.angle)
    print("\n(平飛頂速隨高度上升;設計值是海平面基準,見上面的換算)")


if __name__ == "__main__":
    main()
