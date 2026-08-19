#!/usr/bin/env python3
"""impact_feasibility.py — 「同時命中」的物理可行區間。

回答一個目前沒有答案、但每次調參都預設了答案的問題:
    **給定各機當下的位置、航向、速度與轉彎能力,3 秒散布做不做得到?**

沒有這個答案,調 hold_radius / 環高度 / 釋放順序都是盲的 —— 先前就發生過
「縮小 hold_radius 想壓散布」,算到最後才發現需要 37 m 半徑而物理最小是 206 m。
本工具把那次的個案算術變成通用判據。

── 方法 ──────────────────────────────────────────────────────────────────
文獻(Two-phase guidance law for impact time control under physical constraints,
CJA 2020, doi:10.1016/j.cja.2020.06.007)指出:在加速度與視場限制下,可達的
impact time 是一個**解析可行區間**,不是任意值。同時攻擊要成立,各機的可行區間
必須**有交集**。

⚠️ 誠實聲明:CJA 2020 全文在 ScienceDirect 上對爬取回 403,我只讀到摘要。
   底下的 t_min 是我自己用標準 Dubins「先轉後直」幾何推的,**不是重現該論文的
   公式**。採用的是它的**框架**(可行區間 + 取交集),不是它的閉式解。

t_min(最早可能命中時刻)= 轉向對準的最短路徑 / 速度 + 俯衝段 / 俯衝速度

    最小轉彎半徑  ρ = V² / (g·tanφ)          φ = 傾斜角上限
    水平段        先以 ρ 轉到對準切線,再直線飛到環上(半徑 r_ring)
    俯衝段        路徑長 = Δh / sin γ

t_max(最晚)受限於能量/油料,而不是幾何 —— 飛機可以盤旋等。所以真正的約束是
**t_min 的跨機散布**:最早到的那幾架必須「吸收」掉這段時間差。
    可行的共同命中時刻 T* ≥ max_i t_min_i
    需要吸收的延遲     Δ_i = T* − t_min_i
"""
import argparse
import csv
import math
import os

G = 9.80665


def turn_radius(v, bank_deg):
    """協調轉彎的最小半徑。ρ = V²/(g·tanφ)。"""
    t = math.tan(math.radians(bank_deg))
    if t <= 1e-6:
        return float("inf")
    return v * v / (G * t)


def dubins_len_to_point(de, dn, psi, rho):
    """從（本機在原點、航向 psi）到點 (de, dn) 的「先轉後直」最短路徑長。

    ⚠️ 座標與角度慣例採**航空慣例**,不是數學慣例:
        de, dn = 目標相對本機的（東, 北）位移 (m)
        psi    = 本機航向,**羅盤角**(rad,北為 0,順時針為正)
    寫的時候兩套慣例混用過,結果「正前方的目標」被算成要先轉 180°。
    單元測試 test_target_dead_ahead_is_straight_line 就是為了鎖這件事。

    回傳 (長度, 需要轉的角度 rad)。目標落在轉彎圓內時回傳 (inf, nan) ——
    那代表**當前航向與轉彎能力下根本切不進去**,必須先飛離,是硬性不可行,
    絕不可以悄悄當成一個很大的數字混過去。
    """
    if not math.isfinite(rho):
        return math.hypot(de, dn), 0.0
    sp, cp = math.sin(psi), math.cos(psi)
    # 轉到「+x = 航向、+y = 左」的機體水平框(之後就是標準數學慣例,逆時針為正)
    x = de * sp + dn * cp
    y = dn * sp - de * cp
    if abs(y) < 1e-9:
        if x >= 0:
            return x, 0.0                 # 正前方,直線
        y = 1e-9                          # 正後方:退化,交給下面的轉彎解
    sgn = 1.0 if y > 0 else -1.0          # 往目標那一側轉
    cx, cy = 0.0, rho * sgn               # 轉彎圓心
    d = math.hypot(x - cx, y - cy)
    if d < rho - 1e-9:
        return float("inf"), float("nan")  # 目標在轉彎圓內 → 切不進去
    tangent = math.sqrt(max(0.0, d * d - rho * rho))
    phi_ct = math.atan2(y - cy, x - cx)
    alpha = math.acos(max(-1.0, min(1.0, rho / d)))
    # ⚠️ 切點方向是 φ_CT − sgn·α,不是 +。取錯分支會讓「正前方的目標」算出
    #    要先轉將近 180° —— 寫的時候踩過,靠正前方案例的單元測試抓到。
    phi_tan = phi_ct - sgn * alpha
    phi_p = math.atan2(0.0 - cy, 0.0 - cx)
    dpsi = (phi_tan - phi_p) * sgn
    dpsi = dpsi % (2.0 * math.pi)          # 沿轉向取正
    return rho * dpsi + tangent, dpsi


def t_min_impact(dist, bearing_to_tgt, psi, v, alt_agl, dive_deg, bank_deg,
                 v_dive=None):
    """本機最早可能命中的時間(s)。

    dist            距目標水平距離 (m)
    bearing_to_tgt  目標方位角 (rad, 北為 0)
    psi             本機航向 (rad, 北為 0)
    alt_agl         相對目標地表的高度 (m)
    dive_deg        指令俯衝角 (deg)
    """
    v_dive = v_dive or v
    gam = math.radians(dive_deg)
    if gam <= 0.0:
        raise ValueError("俯衝角必須 > 0")
    r_ring = alt_agl / math.tan(gam)       # 推頭點距目標的水平距離
    rho = turn_radius(v, bank_deg)
    # ⚠️ 先判「已在環內」再判「切不進轉彎圓」。兩者都會回不可行,但原因完全不同,
    #    而使用者要看的是**該修哪裡**:前者是高度/俯衝角的問題,後者是航向的問題。
    if dist < r_ring:
        return float("inf"), {"reason": f"已在環內(距 {dist:.0f}m < 環 {r_ring:.0f}m),"
                                        f"該高度需要更陡的俯衝角",
                              "r_ring": r_ring, "rho": rho}
    de, dn = dist * math.sin(bearing_to_tgt), dist * math.cos(bearing_to_tgt)
    L, dpsi = dubins_len_to_point(de, dn, psi, rho)
    if not math.isfinite(L):
        return float("inf"), {"reason": "目標在最小轉彎圓內,當前航向切不進去",
                              "r_ring": r_ring, "rho": rho}
    L_ring = max(0.0, L - r_ring)          # 最後一段沿徑向,扣掉環半徑
    L_dive = alt_agl / math.sin(gam)
    t = L_ring / v + L_dive / v_dive
    return t, {"r_ring": r_ring, "rho": rho, "L_ring": L_ring, "L_dive": L_dive,
               "turn_deg": math.degrees(dpsi)}


def bearing(a_lat, a_lon, b_lat, b_lon):
    la1, la2 = math.radians(a_lat), math.radians(b_lat)
    dlo = math.radians(b_lon - a_lon)
    y = math.sin(dlo) * math.cos(la2)
    x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlo)
    return math.atan2(y, x) % (2 * math.pi)


def haversine(a_lat, a_lon, b_lat, b_lon):
    R = 6371000.0
    p1, p2 = math.radians(a_lat), math.radians(b_lat)
    dp, dl = math.radians(b_lat - a_lat), math.radians(b_lon - a_lon)
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))


def from_tot_csv(path, geom_path, at_t=None, phase="ALIGN"):
    """從 tot_<T>.csv 取一個時刻的全隊狀態。預設取該相位的第一個樣本。"""
    tgt = None
    with open(geom_path) as f:
        for r in csv.DictReader(f):
            if r["kind"] == "target":
                tgt = (float(r["lat"]), float(r["lon"]))
    if tgt is None:
        raise SystemExit("geom CSV 裡沒有 target 列")
    # ⚠️⚠️ 航向**一定要從實際位移推**,不能因為「取不到前一點」就退回「朝向目標」。
    #    退回樂觀值會讓航向誤差恆為 0 ⇒ 轉彎項歸零 ⇒ 四個傾斜角算出一模一樣的
    #    t_min,而且看起來完全正常。第一版就是這樣,靠「四個傾角結果相同」才發現。
    #    做法:每架取「目標時刻之後」的一小段軌跡,用首尾位移算地面航跡角。
    HDG_WINDOW = 4.0                      # s;夠長才不會被 GPS 抖動主導
    win = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r.get("phase") != phase:
                continue
            i, t = r["sysid"], float(r["t"])
            if at_t is not None and t < at_t:
                continue
            w = win.setdefault(i, [])
            if not w or t - float(w[0]["t"]) <= HDG_WINDOW:
                w.append(r)
    out = []
    for i, w in sorted(win.items(), key=lambda kv: int(kv[0])):
        r = w[0]
        lat, lon = float(r["lat"]), float(r["lon"])
        a, b = w[0], w[-1]
        moved = haversine(float(a["lat"]), float(a["lon"]),
                          float(b["lat"]), float(b["lon"]))
        if moved < 20.0:                  # 位移太小,航跡角沒有意義
            raise SystemExit(
                f"UAV{i} 在 {HDG_WINDOW}s 內只移動 {moved:.0f} m,無法推得航向。"
                "換一個 --at 時刻,或改用帶航向欄位的資料源。")
        psi = bearing(float(a["lat"]), float(a["lon"]),
                      float(b["lat"]), float(b["lon"]))
        out.append({"id": int(i), "lat": lat, "lon": lon,
                    "alt": float(r["alt"]), "gs": max(float(r["gs"]), 1.0),
                    "psi": psi, "t": float(r["t"]),
                    "hdg_err": math.degrees(
                        abs((bearing(lat, lon, *tgt) - psi + math.pi)
                            % (2 * math.pi) - math.pi))})
    return tgt, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trial", nargs="?", help="航次代號(讀 runs/tot_<T>.csv)")
    ap.add_argument("--runs", default=os.path.expanduser("~/fixwing_ws/runs"))
    ap.add_argument("--at", type=float, default=None, help="取這個時刻的狀態 (s)")
    ap.add_argument("--phase", default="ALIGN")
    ap.add_argument("--dive", type=float, default=45.0, help="俯衝角 (deg)")
    ap.add_argument("--bank", type=float, default=None,
                    help="傾斜角上限 (deg);不給就掃描 30/45/60/80")
    ap.add_argument("--v-dive", type=float, default=None)
    ap.add_argument("--target-amsl", type=float, default=0.0,
                    help="目標地表 AMSL;海面 = 0")
    ap.add_argument("--spread-max", type=float, default=3.0)
    a = ap.parse_args()

    if not a.trial:
        ap.error("要給航次代號")
    tgt, air = from_tot_csv(f"{a.runs}/tot_{a.trial}.csv",
                            f"{a.runs}/tot_{a.trial}_geom.csv", a.at, a.phase)
    banks = [a.bank] if a.bank else [30.0, 45.0, 60.0, 80.0]

    print(f"══ {a.trial}:同時命中的物理可行性(相位 {a.phase},"
          f"t={air[0]['t']:.0f}s,俯衝角 {a.dive:.0f}°)══")
    print(f"   目標 {tgt[0]:.5f},{tgt[1]:.5f};目標地表 {a.target_amsl:.0f} m AMSL")
    for bank in banks:
        rows, infeasible = [], []
        for s in air:
            d = haversine(s["lat"], s["lon"], *tgt)
            br = bearing(s["lat"], s["lon"], *tgt)
            # ⚠️ alt 欄是相對 HOME,不是相對目標地表。海面目標時兩者差一個
            #    起飛點海拔 —— 這正是 target_amsl 要處理的事,別忘了換算。
            agl = s["alt"] - a.target_amsl
            t, info = t_min_impact(d, br, s["psi"], s["gs"], agl, a.dive, bank,
                                   a.v_dive)
            rows.append((s["id"], d, agl, s["gs"], t, info))
            if not math.isfinite(t):
                infeasible.append((s["id"], info.get("reason", "?")))
        ts = [r[4] for r in rows if math.isfinite(r[4])]
        rho = turn_radius(sum(s["gs"] for s in air) / len(air), bank)
        print(f"\n── 傾斜上限 {bank:.0f}°(最小轉彎半徑 ≈ {rho:.0f} m,"
              f"負荷 {1/math.cos(math.radians(bank)):.1f} g)──")
        print("   uav    距目標      高度     地速   航向誤差   轉彎    t_min")
        for i, d, agl, v, t, info in rows:
            ts_s = f"{t:7.1f}s" if math.isfinite(t) else "   不可行"
            he = next(x["hdg_err"] for x in air if x["id"] == i)
            tn = info.get("turn_deg")
            tn_s = f"{tn:5.0f}°" if tn is not None and math.isfinite(tn) else "    —"
            print(f"    {i}  {d:8.0f}m {agl:8.0f}m {v:6.1f}  {he:7.0f}°  {tn_s}  {ts_s}")
        if infeasible:
            for i, why in infeasible:
                print(f"   ⚠️ UAV{i} 不可行:{why}")
        if len(ts) < len(rows):
            print("   ⇒ 有機體不可行,同時命中不成立")
            continue
        spread = max(ts) - min(ts)
        T_star = max(ts)
        print(f"   t_min 範圍 {min(ts):.1f} – {max(ts):.1f} s;"
              f"**跨機差 {spread:.1f} s**")
        print(f"   共同命中時刻 T* ≥ {T_star:.1f}s;最早到的要吸收 {spread:.1f}s 延遲")
        # 判定:能不能同時命中,取決於「最早到的能否延遲到 T*」,而不是 t_min 的散布。
        if spread <= a.spread_max:
            print(f"   ✅ 不必額外延遲就已在 {a.spread_max:.0f}s 內")
            continue
        # ⭐ 把「要吸收多少秒」換算成「要多繞多少公尺」—— 這才是可以拿去跟
        #    現行環幾何比對的量。調 hold_radius / 環高度到底夠不夠,看這一行。
        v_mean = sum(r[3] for r in rows) / len(rows)
        need_m = spread * v_mean
        # 現行架構靠「各機環半徑不同」製造路徑差:環半徑 = 高度/tanγ,
        # 而高度差只有 layer_m×(n−1)。能提供的路徑差就是環半徑的跨機差。
        rings = [info["r_ring"] for *_, info in rows if "r_ring" in info]
        have_m = (max(rings) - min(rings)) if rings else 0.0
        print(f"   ⚠️ 需要主動吸收 {spread:.1f}s ≈ **{need_m/1000:.1f} km 的額外路徑**"
              f"(以均速 {v_mean:.0f} m/s 計)")
        print(f"      現行環幾何能提供的路徑差 = 環半徑跨機差 {have_m/1000:.2f} km"
              f" ⇒ {'夠' if have_m >= need_m else f'**差 {(need_m-have_m)/1000:.1f} km**'}")
        if have_m < need_m:
            print(f"      ⇒ 純幾何排程結構性不足。要壓到 {a.spread_max:.0f}s 必須有"
                  f"顯式的延遲吸收機制(盤旋圈/S 彎/減速),量級 {need_m/1000:.0f} km。")
            # 盤旋圈是最省事的延遲吸收器:一圈剛好 2πρ。把缺口換算成圈數,
            # 就是「最早到的那架要在環上繞幾圈」—— 可以直接變成控制參數。
            circ = 2.0 * math.pi * rho
            print(f"      ⇒ 換算成盤旋圈:一圈 = 2πρ = {circ/1000:.1f} km "
                  f"⇒ 最早到的需繞 **{need_m/circ:.2f} 圈**"
                  f"(每圈耗時 {circ/v_mean:.0f}s)")


if __name__ == "__main__":
    main()
