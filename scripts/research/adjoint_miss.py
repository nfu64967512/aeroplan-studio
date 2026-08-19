#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Zarchan 式線性導引迴路的脫靶量分析(縱向平面)。

⚠️⚠️ **先讀這段再看數字。**
本專案的迴路是「航點重發 + PN + TECS」三層混合,不是 Zarchan 假設的
單一線性 PN 迴路;而且實測顯示它對輸入**極度敏感**
(RPK1 兩架 T_d 32.9 vs 34.3、γ_pk 46.2 vs 42.8,落點卻是 62 m vs 499 m)。
⇒ 本模型的絕對值**不可當成預測**,只能當**量級對照**。
   使用者 2026-08-11 明確要求仍然做出來當對照,故保留。

實作方式:直接線性化模擬,而不是伴隨法本身。
    Zarchan 用伴隨法是為了「一次跑完拿到整條曲線」;直接模擬每一步可檢查,
    結果等價。本專案已經被三個「看起來精確但無關」的模型咬過,
    可檢查性優先。

模型
----
    ZEM  = y + ẏ·t_go
    n_c  = N'·ZEM/t_go²                     (比例導引)
    ṅ_L  = (sat(n_c) − n_L)/τ               (一階飛控延遲 + 過載飽和)
    ÿ    = −n_L

初始條件:航跡角虧損 HE(指令 27° − 放行當下實測 γ0)⇒ ẏ(0) = V·HE

反推用法(本檔重點)
------------------
掃描 τ,找出使模擬脫靶量 = 實測脫靶量的那個 τ。
⇒ 得到「實測落點所隱含的等效俯仰通道延遲」,不必信任 fit_pitch_channel 的
  擬合值(那次 RMS 7–17°、訊號僅 14°,擬合失敗)。
"""
from __future__ import annotations

import argparse
import math

G = 9.80665

# 來自 deadtime_miss.py 的實測值:(t_F 秒, V m/s, γ0 度, 觀測沿航跡過頭 m)
CASES = {
    "RPK1-U1": (54.6, 148.6, 7.9, 61.7),
    "RPK1-U2": (69.6, 152.6, 4.4, 499.0),
    "RPK2-U1": (66.0, 158.5, 6.9, 124.5),
    "RPK2-U2": (72.0, 159.7, 8.6, 656.5),
    "RPK3-U2": (68.0, 156.2, 1.8, 147.9),
}


def sim_miss(tF, V, he_rad, tau, n_max_g, Np=4.0, dt=0.005):
    """回傳終端脫靶量 |y(tF)|(m)。"""
    y, yd, nL = 0.0, V * he_rad, 0.0
    n_max = n_max_g * G
    t = 0.0
    while t < tF - 1e-9:
        tgo = max(tF - t, 1e-3)
        zem = y + yd * tgo
        nc = Np * zem / (tgo * tgo)
        nc = max(-n_max, min(n_max, nc))
        nL += (nc - nL) / max(tau, 1e-3) * dt
        y += yd * dt
        yd += -nL * dt
        t += dt
    return abs(y)


def solve_tau(tF, V, he_rad, target_miss, n_max_g, Np):
    """掃 τ,找使模擬脫靶量最接近實測的那個。"""
    best = (None, float("inf"), None)
    tau = 0.02
    while tau <= 40.0:
        m = sim_miss(tF, V, he_rad, tau, n_max_g, Np)
        e = abs(m - target_miss)
        if e < best[1]:
            best = (tau, e, m)
        tau *= 1.04
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--nmax", type=float, default=2.95,
                    help="下壓過載上限(g)。實測峰值中位 2.95")
    ap.add_argument("--np", dest="npn", type=float, default=4.0,
                    help="導航比 N'")
    ap.add_argument("--cmd-deg", type=float, default=27.0)
    a = ap.parse_args()

    print("模型:線性 PN + 一階延遲 + 過載飽和   N'=%.1f  n_max=%.2f g"
          % (a.npn, a.nmax))
    print("HE = 指令 %.0f° − 放行當下實測 γ0" % a.cmd_deg)
    print()
    print("%-9s %-7s %-7s %-7s | %-9s %-9s | %-9s"
          % ("案例", "t_F(s)", "V", "HE(°)", "實測脫靶", "模擬脫靶", "反推τ(s)"))
    print("-" * 78)
    taus = []
    for name, (tF, V, g0, obs) in CASES.items():
        he = math.radians(a.cmd_deg - g0)
        tau, err, m = solve_tau(tF, V, he, obs, a.nmax, a.npn)
        taus.append(tau)
        print("%-9s %-7.1f %-7.1f %-7.1f | %-9.0f %-9.0f | %-9.2f"
              % (name, tF, V, math.degrees(he), obs, m, tau))

    ts = sorted(taus)
    print()
    print("── 反推的等效俯仰延遲 τ:中位 %.2f s  範圍 %.2f – %.2f"
          % (ts[len(ts) // 2], ts[0], ts[-1]))
    print()
    print("── 對照:若 τ 取正常飛控值(0.3 s),脫靶量會是")
    for name, (tF, V, g0, obs) in CASES.items():
        he = math.radians(a.cmd_deg - g0)
        print("     %-9s %6.1f m   (實測 %.0f m)"
              % (name, sim_miss(tF, V, he, 0.3, a.nmax, a.npn), obs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
