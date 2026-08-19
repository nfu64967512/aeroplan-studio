#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""backtest32_decomp.py —— C 階段的失敗歸因(**判定之後才跑,不回頭改判定**)。

把「查表 vs t_wall 中位 −7.34 s」拆成四段,每一段都要能單獨量出來:

    t_go^表(vmax)  --①內插誤差-->  t_go^sim3d(vmax)
                    --②油門政策-->  t_go^sim3d(clock,重現實測的 v_req)
                    --③模型殘差-->  t_sim(實測扣凍結)
                    --④模擬凍結-->  t_wall(登記閘門用的真值)

⚠️ ② 不是 bug,是**建表時的規格選擇**(Guo §III.B 的 5 維不變性要求 t_go 只能
   是 FSV 的函數,而實測油門律 v_req = clip(45,210,R/t_rem) 引用絕對時鐘)。
   ⇒ t_go^PNG 定義成「最快可達時間」,**結構上就是下界** ⇒ 單邊為負是定義的
   後果,不是校準誤差。這一點決定了閘門的單邊性那一半**在此定義下不可能通過**。
"""
from __future__ import annotations

import math
import os
import statistics as st
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_table                                                  # noqa: E402
import sim3d                                                        # noqa: E402
import truth32                                                      # noqa: E402

TABLE = os.path.join(HERE, "ditcg_table_ours.npz")


def desc(name, v):
    v = [x for x in v if math.isfinite(x)]
    if not v:
        print(f"  {name:<34s} (none)")
        return
    neg = sum(1 for x in v if x < 0)
    print(f"  {name:<34s} med {st.median(v):+7.2f}  range {min(v):+7.2f}..{max(v):+7.2f}"
          f"  sd {st.pstdev(v):5.2f}  neg/pos {neg:2d}/{len(v)-neg:<2d}")


def run(recs, law):
    pn = sim3d.PNParams()
    sp = sim3d.SimParams(throttle_law=law, dt=build_table.TABLE_DT)
    return sim3d.fly(
        np.array([d["r_m"] for d in recs]),
        np.array([d["z_m"] for d in recs]),
        np.array([d["V_ms"] for d in recs]),
        np.radians([d["gamma_deg"] for d in recs]),
        np.radians([d["eps_deg"] for d in recs]),
        pn=pn, sp=sp,
        t_rem0=np.array([d["t_rem0_s"] for d in recs]))


def main():
    recs = truth32.load()
    tbl = build_table.load(TABLE)
    lut = []
    for d in recs:
        t, _w = build_table.lookup(tbl, d["r_m"], d["z_m"], d["V_ms"],
                                   d["gamma_deg"], d["eps_deg"])
        lut.append(t)
    s_vmax = run(recs, "vmax")["t_go"]
    s_clock = run(recs, "clock")["t_go"]
    t_sim = [d["t_sim_s"] for d in recs]
    t_wall = [d["t_wall_s"] for d in recs]

    print("== decomposition of TABLE - t_wall (median -7.34 s) ==")
    desc("(1) interp: table - sim3d(vmax)", [a - b for a, b in zip(lut, s_vmax)])
    desc("(2) policy: sim3d(vmax) - (clock)", [a - b for a, b in zip(s_vmax, s_clock)])
    desc("(3) model : sim3d(clock) - t_sim", [a - b for a, b in zip(s_clock, t_sim)])
    desc("(4) freeze: t_sim - t_wall", [a - b for a, b in zip(t_sim, t_wall)])
    print("  " + "-" * 78)
    desc("SUM  : table - t_wall", [a - b for a, b in zip(lut, t_wall)])

    print("\n== what each stage would score on the gate (vs t_wall) ==")
    for nm, p in (("table (vmax, what we built)", lut),
                  ("sim3d vmax (no interp err)", list(s_vmax)),
                  ("sim3d clock (real throttle)", list(s_clock))):
        b = [a - c for a, c in zip(p, t_wall) if math.isfinite(a)]
        neg = sum(1 for x in b if x < 0)
        one = max(neg, len(b) - neg)
        ok = abs(st.median(b)) < 3.0 and one <= 24.0 * len(b) / 32.0
        print(f"  {nm:<32s} |med| {abs(st.median(b)):5.2f}  one-sided {one}/{len(b)}"
              f"  => {'PASS' if ok else 'FAIL'}")

    print("\n== how much does throttle policy alone cost (per sortie) ==")
    print(f"{'tag':<10s}{'t_rem0':>8s}{'v_req0':>8s}{'V0':>7s}{'thr0':>7s}"
          f"{'vmax':>8s}{'clock':>8s}{'diff':>8s}")
    for d, a, b in zip(recs, s_vmax, s_clock):
        R = math.hypot(d["r_m"], d["z_m"])
        trem = max(d["t_rem0_s"], 1e-6)
        vreq = min(210.0, max(45.0, R / trem))
        thr = min(1.0, max(0.0, 0.36 + 0.030 * (vreq - d["V_ms"])))
        print(f"{d['tag']:<10s}{d['t_rem0_s']:8.2f}{vreq:8.1f}{d['V_ms']:7.1f}"
              f"{thr:7.2f}{a:8.2f}{b:8.2f}{a-b:+8.2f}")

    print("\n== DIAG: does the table have ANY discriminating power ==")
    def pear(a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        m = np.isfinite(a) & np.isfinite(b)
        return float(np.corrcoef(a[m], b[m])[0, 1])
    for nm, p in (("table", lut), ("sim3d vmax", list(s_vmax)),
                  ("sim3d clock", list(s_clock))):
        v = [x for x in p if math.isfinite(x)]
        print(f"  {nm:<14s} sd {st.pstdev(v):5.2f}  r(pred,t_wall) {pear(p, t_wall):+.3f}"
              f"  r(pred,t_sim) {pear(p, t_sim):+.3f}")
    print(f"  measured       sd {st.pstdev(t_wall):5.2f} (t_wall) / "
          f"{st.pstdev(t_sim):5.2f} (t_sim)")

    print("\n== DIAG: residual sd after removing a constant offset ==")
    for nm, p, tr in (("table vs t_wall", lut, t_wall),
                      ("table vs t_sim", lut, t_sim),
                      ("sim3d clock vs t_sim", list(s_clock), t_sim)):
        b = [a - c for a, c in zip(p, tr) if math.isfinite(a)]
        print(f"  {nm:<24s} bias sd {st.pstdev(b):5.2f}   "
              f"(constant-estimator sd = {st.pstdev([c for a, c in zip(p, tr) if math.isfinite(a)]):5.2f})")


if __name__ == "__main__":
    main()
