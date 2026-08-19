#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""backtest32.py —— C 階段:用 5 維查表對 32 架次放行狀態下**事前閘門**的判定。

⭐⭐ 本檔**只算不調**。閘門是事前寫死的:

    通過:偏差中位絕對值 < 3.0 s **且** 單邊性消失(正負比不得比 24/32 更偏)
    失敗:如實回報並停 —— 不准回頭調參數去湊

慣例:**bias = 預測 − 實測**。負 = 低估(預測比實飛短)。

## 為什麼主判定用 t_wall(牆鐘)

事前登記的基線是「中位 −8.2 s、29/32 單邊、全距 −15.7…+4.3」。A3 已逐位元
重現:那組數字**只有在牆鐘 + 水平距/地速**下才對得上(−8.36/29/32/−15.76…+4.28)。
⇒ 閘門登記時用的真值就是牆鐘。**主判定照登記時的定義走**,不擅自換時鐘。
t_sim(扣掉模擬凍結)一併報成診斷,但**不拿它當判定** —— 換真值定義本身就是調參。
"""
from __future__ import annotations

import math
import os
import statistics as st
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(os.path.join(
    HERE, "..", "..", "wsl", "sar_swarm_patches")))
import build_table                                                  # noqa: E402
import truth32                                                      # noqa: E402
import predictor as _P                                              # noqa: E402

TABLE = os.path.join(HERE, "ditcg_table_ours.npz")


def stats(bias):
    b = [x for x in bias if x is not None and math.isfinite(x)]
    if not b:
        return None
    neg = sum(1 for x in b if x < 0)
    return {"n": len(b), "med": st.median(b), "amed": abs(st.median(b)),
            "lo": min(b), "hi": max(b), "sd": st.pstdev(b),
            "neg": neg, "pos": len(b) - neg,
            "one_sided": max(neg, len(b) - neg)}


def line(name, s):
    if s is None:
        print(f"  {name:<30s}  (no valid sample)")
        return
    print(f"  {name:<30s} med {s['med']:+7.2f}  |med| {s['amed']:5.2f}  "
          f"range {s['lo']:+7.2f}..{s['hi']:+7.2f}  sd {s['sd']:5.2f}  "
          f"neg/pos {s['neg']:2d}/{s['pos']:<2d}  one-sided {s['one_sided']:2d}/{s['n']}")


def pearson(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 3:
        return float("nan")
    return float(np.corrcoef(a[m], b[m])[0, 1])


def gate(s, label):
    """事前閘門,逐字照抄:|中位| < 3.0 且 單邊 <= 24/32。"""
    if s is None:
        return False, f"{label}: no valid sample => FAIL"
    a = s["amed"] < 3.0
    thresh = 24.0 * s["n"] / 32.0
    b = s["one_sided"] <= thresh
    v = "PASS" if (a and b) else "FAIL"
    return (a and b), (f"{label}: |med| {s['amed']:.2f} {'<' if a else '>='} 3.0 "
                       f"[{'PASS' if a else 'FAIL'}]  one-sided {s['one_sided']}/{s['n']} "
                       f"{'<=' if b else '>'} {thresh:.0f} [{'PASS' if b else 'FAIL'}]"
                       f"  => **{v}**")


def main():
    recs = truth32.load()
    n = len(recs)
    tbl = build_table.load(TABLE)
    print(f"table {os.path.basename(TABLE)}  grid "
          f"{'x'.join(str(len(a)) for a in tbl['axes'])} = {tbl['t_go'].size:,}")
    print(f"== data n = {n} ==")
    for k, lab in (("t_wall_s", "t_wall (gate truth)"), ("t_sim_s", "t_sim  (minus freeze)")):
        v = [d[k] for d in recs]
        print(f"  {lab:<24s} med {st.median(v):6.2f}  range {min(v):.1f}..{max(v):.1f}"
              f"  sd {st.pstdev(v):.2f}")

    t_wall = [d["t_wall_s"] for d in recs]
    t_sim = [d["t_sim_s"] for d in recs]

    # 1. 查表(正確 eps = 帶號水平方位差,查表取 |eps|)
    lut, why, lut_lead, lut_spoken = [], [], [], []
    for d in recs:
        t, w = build_table.lookup(tbl, d["r_m"], d["z_m"], d["V_ms"],
                                  d["gamma_deg"], d["eps_deg"])
        lut.append(t)
        why.append(w)
        t2, _ = build_table.lookup(tbl, d["r_m"], d["z_m"], d["V_ms"],
                                   d["gamma_deg"], d["lead_angle_csv_deg"])
        lut_lead.append(t2)
        t3, _ = build_table.lookup(tbl, d["r_m"], d["z_m"], d["V_ms"],
                                   d["gamma_deg"], d["eps_spoken_deg"])
        lut_spoken.append(t3)

    # 2. 對照估計器
    naive_R = [d["R_slant_m"] / d["V_ms"] for d in recs]
    naive_rgs = [d["r_m"] / (d["V_ms"] * math.cos(math.radians(d["gamma_deg"])))
                 for d in recs]
    pred = []
    for d in recs:
        o = _P.predict_terminal(d["z_m"], d["r_m"], d["V_ms"], thr_max_pct=100.0,
                                v_req=210.0, home_amsl=0.0, dt=0.05)
        pred.append(o["t_go"])
    const = [st.median(t_wall)] * n
    const_sim = [st.median(t_sim)] * n

    def bias(p, truth):
        return [(pi - ti) if (pi is not None and math.isfinite(pi)) else float("nan")
                for pi, ti in zip(p, truth)]

    for lab, truth, cst in (("t_wall (registered gate truth)", t_wall, const),
                            ("t_sim (freeze removed; diagnostic)", t_sim, const_sim)):
        print(f"\n{'='*82}\n== three-way vs {lab} ==")
        line("(1) naive r_horiz/gs = BASELINE", stats(bias(naive_rgs, truth)))
        line("(1b) naive R_slant/V_3d", stats(bias(naive_R, truth)))
        line("(2) predictor.py (longitudinal)", stats(bias(pred, truth)))
        line("(3) 5-D TABLE t_go^PNG", stats(bias(lut, truth)))
        line("    (ref) constant = med(truth)", stats(bias(cst, truth)))

    # 4. 閘門判定
    print(f"\n{'='*82}\n== GATE (rule copied verbatim, unmodified) ==")
    print("   rule: |median bias| < 3.0 s AND one-sidedness not worse than 24/32")
    s_main = stats(bias(lut, t_wall))
    ok_main, msg = gate(s_main, "(3) TABLE vs t_wall  [PRIMARY]")
    print("   " + msg)
    for nm, p, tr in (("(3) TABLE vs t_sim   (diag)", lut, t_sim),
                      ("(1) baseline vs t_wall (ref)", naive_rgs, t_wall),
                      ("(2) predictor vs t_wall (ref)", pred, t_wall)):
        print("   " + gate(stats(bias(p, tr)), nm)[1])

    # 5. 診斷
    print(f"\n{'='*82}\n== DIAG A: which dimension does the error track ==")
    e_w = bias(lut, t_wall)
    e_s = bias(lut, t_sim)
    print(f"  {'dim':<8s}{'r(err,dim) t_wall':>20s}{'t_sim':>10s}{'measured range':>26s}")
    for k, lab in (("r_m", "r"), ("z_m", "z"), ("V_ms", "V"),
                   ("gamma_deg", "gamma"), ("eps_deg", "eps")):
        v = [d[k] for d in recs]
        print(f"  {lab:<8s}{pearson(e_w, v):>20.3f}{pearson(e_s, v):>10.3f}"
              f"{min(v):>14.1f}..{max(v):<11.1f}")
    print(f"  r(table_out, truth) = {pearson(lut, t_wall):+.3f} (t_wall) / "
          f"{pearson(lut, t_sim):+.3f} (t_sim)")
    v = [x for x in lut if math.isfinite(x)]
    print(f"  table out sd {st.pstdev(v):.2f} (range {min(v):.1f}..{max(v):.1f}) "
          f"vs measured sd {st.pstdev(t_wall):.2f} (t_wall) / {st.pstdev(t_sim):.2f} (t_sim)")
    for k, lab in (("r_m", "r"), ("z_m", "z"), ("V_ms", "V"),
                   ("gamma_deg", "gamma"), ("eps_deg", "eps")):
        v2 = [d[k] for d in recs]
        print(f"    r(truth_wall, {lab}) = {pearson(t_wall, v2):+.3f}   "
              f"r(table_out, {lab}) = {pearson(lut, v2):+.3f}")

    print(f"\n== DIAG B: sorties landing in the 'undecidable' holes ==")
    c = Counter(why)
    for k, v2 in sorted(c.items()):
        print(f"   {k:<16s} {v2}/{n}")
    for d, w in zip(recs, why):
        if w != "ok":
            print(f"   !! {d['tag']:<9s} {w:<16s} r={d['r_m']:.0f} z={d['z_m']:.0f} "
                  f"V={d['V_ms']:.1f} gam={d['gamma_deg']:+.2f} eps={d['eps_deg']:+.2f}")

    print(f"\n== DIAG C: eps feed variants ==")
    d_lead = [abs(d["eps_deg"]) - abs(d["lead_angle_csv_deg"]) for d in recs]
    d_spk = [abs(d["eps_deg"]) - abs(d["eps_spoken_deg"]) for d in recs]
    print(f"  |eps_correct| - |lead_angle_csv| : med {st.median(d_lead):+.4f} deg  "
          f"range {min(d_lead):+.4f}..{max(d_lead):+.4f}")
    print(f"  |eps_correct| - |eps_spoken(A2)| : med {st.median(d_spk):+.4f} deg  "
          f"range {min(d_spk):+.4f}..{max(d_spk):+.4f}")
    dt_lead = [a - b for a, b in zip(lut, lut_lead)
               if math.isfinite(a) and math.isfinite(b)]
    dt_spk = [a - b for a, b in zip(lut, lut_spoken)
              if math.isfinite(a) and math.isfinite(b)]
    print(f"  => table t_go diff (correct - lead_angle): med {st.median(dt_lead):+.4f} s  "
          f"|max| {max(abs(x) for x in dt_lead):.4f} s  (n={len(dt_lead)})")
    print(f"  => table t_go diff (correct - spoken)    : med {st.median(dt_spk):+.4f} s  "
          f"|max| {max(abs(x) for x in dt_spk):.4f} s  (n={len(dt_spk)})")
    line("   TABLE(fed lead_angle) vs t_wall", stats(bias(lut_lead, t_wall)))
    line("   TABLE(fed spoken eps) vs t_wall", stats(bias(lut_spoken, t_wall)))

    print(f"\n{'='*82}\n== per-sortie ==")
    print(f"{'tag':<10s}{'r':>6s}{'z':>6s}{'V':>7s}{'gam':>7s}{'eps':>7s}"
          f"{'TABLE':>8s}{'t_wall':>8s}{'bias_w':>8s}{'t_sim':>7s}{'bias_s':>8s}"
          f"{'naive':>7s}{'pred':>7s}  why")
    for i, (d, t, w) in enumerate(zip(recs, lut, why)):
        bw = t - d["t_wall_s"] if math.isfinite(t) else float("nan")
        bs = t - d["t_sim_s"] if math.isfinite(t) else float("nan")
        print(f"{d['tag']:<10s}{d['r_m']:6.0f}{d['z_m']:6.0f}{d['V_ms']:7.1f}"
              f"{d['gamma_deg']:+7.2f}{d['eps_deg']:+7.2f}{t:8.2f}"
              f"{d['t_wall_s']:8.2f}{bw:+8.2f}{d['t_sim_s']:7.2f}{bs:+8.2f}"
              f"{naive_rgs[i]:7.1f}{pred[i]:7.1f}  {w}")

    print(f"\n{'='*82}")
    print("FINAL VERDICT (primary = TABLE vs t_wall, registered gate):",
          "PASS" if ok_main else "FAIL")


if __name__ == "__main__":
    main()
