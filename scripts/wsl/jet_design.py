#!/usr/bin/env python3
"""jet_design.py — 由「巡航速度 / 終端速度 / 設計俯角」反推 SITL 噴射模型的設定。

SITL 的 plane 模型(`libraries/SITL/SIM_Plane.cpp`)是點質量 + 簡單氣動:

    推力  T   = servo_throttle × thrust_scale,  thrust_scale = mass·g / hover_throttle
    阻力  D   = ½·ρ·s·(c_drag_p + C_L²/(π·e·AR))·V²,   AR = b²/s
    平飛頂速          T = D
    俯角 γ 的終端速度  T + W·sinγ = D

⚠️ 兩個結構性事實(踩過才知道):

1. **`thrust_scale` 與 mass 成正比,所以推重比 T/W ≡ 1/hover_throttle = 1.43 是寫死的,
   把質量加重完全不會改變它。** 而
       (V_終端 / V_巡航)² = 1 + (W/T)·sinγ
   在 T/W=1.43 下,即使垂直俯衝倍率上限也只有 1.30。要更大的倍率就一定要降推重比。

2. 降推重比**不需要改 C++**:`calculate_forces` 用的是 servo 輸出,而 servo 上限就是
   飛控參數 `THR_MAX`。THR_MAX=40 ⇒ 可用推力只剩 40%,等效 T/W = 0.4/0.7 × ... 見下。
   阻力則可以用 `--model <frame>:<file>.json` 從 JSON 覆寫 `c_drag_p`,同樣不用重編譯。
   (JSON 在建構子裡比 `-jet` 區塊**更早**載入,而 `-jet` 只動 mass/thrust_scale,
    所以係數不會被蓋掉。)

用法:
  python3 jet_design.py --cruise 136 --terminal 204
  python3 jet_design.py --cruise 136 --terminal 204 --angle 30 --verbose
"""
import argparse
import math

G = 9.80665
RHO = 1.225           # 海平面;桃園場區 400–1800 m 會低 4–15%,見 --alt
MASS = 22.0           # SIM_Plane.cpp:65  `-jet`
HOVER_THROTTLE = 0.7  # SIM_Plane.h:45
S = 0.45              # 翼面積(預設係數)
B = 1.88
OSWALD = 0.9
C_LIFT_0 = 0.56
C_LIFT_A = 6.9

W = MASS * G
THRUST_FULL = MASS * G / HOVER_THROTTLE      # servo=1.0 時的推力(N)
AR = B * B / S


def rho_at(alt_m):
    """ISA 密度(SITL 用的是同一套指數近似,1 km 內誤差 <1%)。"""
    return RHO * (1.0 - 2.25577e-5 * alt_m) ** 4.2586


def cd(v, rho):
    """含誘導阻力:高速時 C_L 很小,誘導項可忽略,但低速時不行。"""
    q_s = 0.5 * rho * S * v * v
    cl = W / max(q_s, 1e-6)
    return 0.1, cl        # c_drag_p 由呼叫端給,這裡只回傳 C_L


def drag(v, p, rho):
    q_s = 0.5 * rho * S * v * v
    cl = W / max(q_s, 1e-6)
    return q_s * (p + cl * cl / (math.pi * OSWALD * AR))


def solve(cruise, terminal, angle_deg, rho):
    """回傳 (T/W, thr_max_pct, c_drag_p)。"""
    ratio2 = (terminal / cruise) ** 2
    if ratio2 <= 1.0:
        raise SystemExit("終端速度必須大於巡航速度")
    # (V2/V1)² = 1 + (W/T)·sinγ   ⇒  T/W = sinγ / ((V2/V1)² − 1)
    tw = math.sin(math.radians(angle_deg)) / (ratio2 - 1.0)
    T = tw * W
    # 平飛頂速 = cruise ⇒ T = D(cruise)
    q_s = 0.5 * rho * S * cruise * cruise
    cl = W / q_s
    p = T / q_s - cl * cl / (math.pi * OSWALD * AR)
    thr_max = 100.0 * T / THRUST_FULL
    return tw, thr_max, p


def dive(perch_m, angle_deg, v0, p, thr_frac, ground_alt=0.0, dt=0.05):
    """從 perch 高度沿定俯角俯衝,積分到觸地。回傳 (觸地總速, 耗時, 航程)。

    只做縱向能量平衡(這正是決定終端速度的東西);轉彎、導引誤差不在這裡。
    """
    h = perch_m
    v = v0
    s = 0.0
    t = 0.0
    sg = math.sin(math.radians(angle_deg))
    while h > ground_alt and t < 300.0:
        rho = rho_at(h)
        T = thr_frac * THRUST_FULL
        a = (T + W * sg - drag(v, p, rho)) / MASS
        v = max(5.0, v + a * dt)
        s += v * dt
        h -= v * sg * dt
        t += dt
    return v, t, s


def turn_radius(v, bank_deg):
    return v * v / (G * math.tan(math.radians(bank_deg)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cruise", type=float, default=136.0)
    ap.add_argument("--terminal", type=float, default=204.0)
    ap.add_argument("--angle", type=float, default=None,
                    help="只評估這一個俯角;不給就掃一輪")
    ap.add_argument("--bank", type=float, default=60.0, help="環上盤旋的傾角")
    ap.add_argument("--entry", type=float, default=None,
                    help="進入俯衝時的速度(預設 = 0.75×巡航)")
    ap.add_argument("--reach", type=float, default=0.97,
                    help="要求觸地速度達到終端目標的這個比例")
    ap.add_argument("--cal", type=float, default=1.087,
                    help="c_drag_p 的實測校正係數。⚠️ 點質量算式解出來的值會讓飛機"
                         "比目標快:實測 c_drag_p=0.01150 時海平面頂速 141.8 而非 "
                         "136(高 4.3%),阻力要乘 (141.8/136)²=1.087。"
                         "原因是本模型的誘導阻力用 W 算,但俯衝時升力其實是 W·cosγ。"
                         "要看純理論值就傳 --cal 1.0")
    a = ap.parse_args()

    v_entry = a.entry if a.entry else 0.75 * a.cruise
    angles = [a.angle] if a.angle else [15, 20, 25, 30, 35, 45]

    print("目標:平飛頂速 %.0f m/s、俯角終端 %.0f m/s(倍率 %.2f)"
          % (a.cruise, a.terminal, a.terminal / a.cruise))
    print("機體:mass %.0f kg、W %.1f N、servo=1.0 推力 %.1f N(T/W %.2f = 1/hover_throttle)"
          % (MASS, W, THRUST_FULL, THRUST_FULL / W))
    print("進入俯衝速度假設 %.0f m/s\n" % v_entry)

    print("%5s %7s %9s %11s | %9s %9s %9s | %9s %9s" %
          ("俯角", "T/W", "THR_MAX", "c_drag_p",
           "需要perch", "俯衝耗時", "觸地速度", "環半徑", "迴轉半徑"))
    for ang in angles:
        try:
            tw, thr, p = solve(a.cruise, a.terminal, ang, rho_at(0.0))
        except SystemExit:
            raise
        if p <= 0.005:
            print("%5.0f  c_drag_p 解出 %.4f,低於模型合理範圍(<0.005),跳過" % (ang, p))
            continue
        # 找出「觸地速度達到 reach×terminal」所需的 perch
        need = None
        for perch in range(400, 6001, 50):
            vt, tt, ss = dive(perch, ang, v_entry, p, thr / 100.0)
            if vt >= a.reach * a.terminal:
                need = (perch, tt, vt, ss)
                break
        if need is None:
            print("%5.0f %7.3f %9.1f %11.5f   6000 m 以內達不到 %.0f%% 的終端速度"
                  % (ang, tw, thr, p, a.reach * 100))
            continue
        perch, tt, vt, ss = need
        ring = (perch) / math.tan(math.radians(ang))
        tr = turn_radius(a.cruise, a.bank)
        flag = "" if ring > 2.0 * tr else "  ⚠️ 環半徑逼近迴轉半徑"
        # ⚠️ 積分要用**未校正**的 p:校正係數描述的是「模型比真機樂觀多少」,
        #    真機在 c_drag_p = p×cal 之下的行為 ≈ 模型在 p 之下的行為。
        #    要餵給 SITL 的才是 p×cal。
        print("%5.0f %7.3f %9.1f %11.5f | %9.0f %9.1f %9.1f | %9.0f %9.0f%s"
              % (ang, tw, thr, p * a.cal, perch, tt, vt, ring, tr, flag))

    if a.angle:
        tw, thr, p = solve(a.cruise, a.terminal, a.angle, rho_at(0.0))
        print("\n── 設定值(c_drag_p 已乘上實測校正 %.3f)──" % a.cal)
        print("  jet_fast.json :  {\"c_drag_p\": %.5f}" % (p * a.cal))
        print("  jet_fast.parm :  THR_MAX %.0f      (= T/W %.3f)" % (thr, tw))
        print("                   AIRSPEED_MAX / ARSPD_FBW_MAX  >= %.0f"
              % (a.terminal * 1.1))
        print("  平飛驗證      :  全油門定高應收斂到 %.0f m/s" % a.cruise)
        print("\n  ⚠️ 迴轉半徑 ∝ V²:巡航 %.0f m/s / %.0f° 傾角 = %.0f m"
              % (a.cruise, a.bank, turn_radius(a.cruise, a.bank)))
        print("     舊組態是 70 m/s → %.0f m,所以扇形與環的幾何要放大約 %.1f 倍"
              % (turn_radius(70, a.bank),
                 turn_radius(a.cruise, a.bank) / turn_radius(70, a.bank)))


if __name__ == "__main__":
    main()
