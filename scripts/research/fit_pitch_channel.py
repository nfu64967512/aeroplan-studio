#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""從實測航次擬合**縱向(俯仰)通道**的等效動態。

目的:給 Zarchan 伴隨分析提供**實測**的 τ 與 n_max,而不是用猜的。
    (使用者原提案是假設 τ=2.0 s / n_max=−0.5G;我們有六架次的下沉率
     時間歷程,可以直接擬合出來,結論會硬得多。)

量什麼
------
瞬時航跡角          γ(t)  = atan2(sink, gs)        sink = −d(alt)/dt
一階延遲時間常數    τ     : γ(t) = γ_∞ (1 − e^(−t/τ)) 的最小平方擬合
下壓法向加速度      a_n   ≈ V · dγ/dt              (垂直平面內,單位 g)

⚠️ 為什麼不用平均俯角就好:純運動學模型(定 γ)預測過頭量 1441–6147 m,
   實測只有 62–656 m,高估 3–25 倍 —— 因為航跡在**末端才被拉陡**。
   ⇒ 過頭量由「壓下去的**動態**」決定,不是平均幾何。這支就是量那個動態。

⚠️ alt 取樣 ~20 Hz,直接差分會被雜訊淹掉 ⇒ 用中央差分 + 移動平均。
   窗長是**參數**不是常數:改窗長會改 τ,所以一併印出來,結論要標註窗長。
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

DEFAULT_ART = Path(__file__).resolve().parents[2] / "runs" / "artifacts"
G = 9.80665


def load(csv_path: Path):
    rows: dict[int, list] = {}
    with open(csv_path, encoding="utf-8", errors="replace") as f:
        for r in csv.DictReader(f):
            try:
                sid = int(r["sysid"])
                rows.setdefault(sid, []).append({
                    "t": float(r["t"]), "alt": float(r["alt"]),
                    "gs": float(r["gs"]), "phase": r["phase"], "mode": r["mode"],
                })
            except (ValueError, TypeError, KeyError):
                continue
    return rows


def movavg(xs, w):
    if w <= 1:
        return list(xs)
    out, n = [], len(xs)
    half = w // 2
    for i in range(n):
        a, b = max(0, i - half), min(n, i + half + 1)
        out.append(sum(xs[a:b]) / (b - a))
    return out


def gamma_series(seg, win):
    """回傳 (t, γ度, V) 三串。γ>0 = 下降。"""
    t = [x["t"] for x in seg]
    alt = movavg([x["alt"] for x in seg], win)
    gs = movavg([max(x["gs"], 1e-3) for x in seg], win)
    gam, tt, vv = [], [], []
    for i in range(1, len(seg) - 1):
        dt = t[i + 1] - t[i - 1]
        if dt <= 1e-6:
            continue
        sink = -(alt[i + 1] - alt[i - 1]) / dt
        gam.append(math.degrees(math.atan2(sink, gs[i])))
        tt.append(t[i])
        vv.append(math.hypot(gs[i], sink))
    return tt, movavg(gam, win), vv


def fit_tau(tt, gam, gam_inf):
    """γ(t) = γ_inf (1 − e^(−t/τ)) 的一維最小平方(掃描 τ)。

    ⚠️ 不用線性化取對數:γ 會超過 γ_inf、也會是負的(初期在爬升),
       取對數會直接爆掉或悄悄丟掉那些點 —— 那正是我們最想看的初期行為。
    """
    if not tt or gam_inf <= 0:
        return None, None
    t0 = tt[0]
    best = (None, float("inf"))
    tau = 0.05
    while tau <= 60.0:
        s = 0.0
        for t, g in zip(tt, gam):
            pred = gam_inf * (1.0 - math.exp(-(t - t0) / tau))
            s += (g - pred) ** 2
        if s < best[1]:
            best = (tau, s)
        tau *= 1.06
    rms = math.sqrt(best[1] / len(tt))
    return best[0], rms


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--art", default=str(DEFAULT_ART))
    ap.add_argument("--tags", default="RPK1,RPK2,RPK3")
    ap.add_argument("--cmd-deg", type=float, default=27.0)
    ap.add_argument("--win", type=int, default=21,
                    help="平滑窗(取樣點)。⚠️ 會影響 τ,結論要標註")
    a = ap.parse_args()

    art = Path(a.art)
    print("平滑窗 = %d 點(~%.1f s @20Hz);指令俯角 = %.0f°"
          % (a.win, a.win * 0.05, a.cmd_deg))
    print()
    print("%-6s %-5s | %-7s %-7s %-8s | %-7s %-7s | %-9s %-8s"
          % ("趟", "機", "γ峰值", "γ平均", "達成率", "τ(s)", "擬合RMS",
             "a_n峰(g)", "推頭段"))
    print("-" * 92)

    rows_out = []
    for tag in a.tags.split(","):
        p = art / tag / ("tot_%s.csv" % tag)
        if not p.exists():
            print("%-6s (缺檔)" % tag)
            continue
        data = load(p)
        for sid in sorted(data):
            seg = [x for x in data[sid]
                   if x["phase"] == "DIVE" and x["mode"] == "dive"]
            if len(seg) < 40:
                continue
            # 只取到最低點:之後是命中後的自由飛行
            lo = min(range(len(seg)), key=lambda k: seg[k]["alt"])
            if lo < 20:
                continue
            seg = seg[:lo + 1]
            tt, gam, vv = gamma_series(seg, a.win)
            if len(tt) < 20:
                continue
            gmax = max(gam)
            gavg = sum(gam) / len(gam)
            tau, rms = fit_tau(tt, gam, a.cmd_deg)
            # 法向加速度:a_n = V · dγ/dt(rad/s)
            an_peak, push_s = 0.0, 0.0
            for k in range(1, len(tt)):
                dt = tt[k] - tt[k - 1]
                if dt <= 1e-6:
                    continue
                dg = math.radians(gam[k] - gam[k - 1]) / dt
                an = vv[k] * dg / G
                if an > an_peak:
                    an_peak = an
                if dg > 0:
                    push_s += dt
            rows_out.append((tag, sid, gmax, gavg, tau, an_peak))
            print("%-6s UAV%-2d | %-7.1f %-7.1f %-8.0f%% | %-7.2f %-7.2f | "
                  "%-9.2f %-8.1f"
                  % (tag, sid, gmax, gavg, 100.0 * gavg / a.cmd_deg,
                     tau if tau else float("nan"),
                     rms if rms else float("nan"), an_peak, push_s))

    if rows_out:
        taus = [r[4] for r in rows_out if r[4]]
        ans = [r[5] for r in rows_out]
        gav = [r[3] for r in rows_out]
        print()
        print("── 擬合結果(給伴隨分析用)")
        print("   τ      中位 %.2f s   範圍 %.2f – %.2f"
              % (sorted(taus)[len(taus) // 2], min(taus), max(taus)))
        print("   a_n峰  中位 %.2f g   範圍 %.2f – %.2f"
              % (sorted(ans)[len(ans) // 2], min(ans), max(ans)))
        print("   γ平均  中位 %.1f°    指令 %.0f°  ⇒ 達成率中位 %.0f%%"
              % (sorted(gav)[len(gav) // 2], a.cmd_deg,
                 100.0 * sorted(gav)[len(gav) // 2] / a.cmd_deg))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
