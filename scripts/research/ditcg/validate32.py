#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""validate32.py —— 用 32 架次實測回測 `sim3d.py`。**建表之前必須先過這一關。**

⭐⭐ 為什麼這一步排在建表前面:**模擬器重現不了實測,表就沒有意義。**
   Guo 的離線階段是「用模擬器掃 10^7 點」;那張表的可信度上限就是模擬器的可信度。
   A3 已經量到現行 `predictor.py`(無側向)對這 32 架次是
   偏差中位 -8.90 s、**32/32 全部低估**、r(pred, 實測) = +0.246
   —— 而 naive `d/V` 是 -8.37 / 29/32,也就是**預測器沒有比一行除法好**,
   而且單邊性更糟。本檔要回答的是:補上側向通道之後,這三個數字動了多少。

## 三組對照(不要只看一個數字)

    A. 時鐘        t_wall(交辦的實測值)vs t_sim(扣掉模擬凍結)
       ⚠️ 32 架次共 108.2 s 凍結(每架中位 3.7 s、最大 7.4 s),
          三條獨立證據證明是**模擬時鐘真的停住**(空檔內位移只有應走的
          0.6-7.1%、空檔後無追趕爆衝、路徑長/積分 = 1.0022)。
          物理模擬器預測的是**機體時間** ⇒ 對 t_wall 比是不公平的,
          但 t_wall 是事前登記閘門用的量 ⇒ **兩個都報,不擅自二選一**。
    B. 油門政策    "clock"(重現實測用,t_rem0 從 CSV 取)
                   vs "vmax"(建表的 PNG 定義,v_req 釘死 210)
    C. 估計器      sim3d / predictor(縱向) / naive r/V / naive R/V / 常數

## 三個要比的量(不是只有耗時)

    1. 俯衝耗時 t_go
    2. 觸地**地速** gs(KPI②;= V cos(gamma))
    3. 落點距紅心 r_touch(KPI③ 的直接量)
"""
from __future__ import annotations

import math
import os
import statistics as st
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim3d                                                        # noqa: E402
import truth32                                                      # noqa: E402
import predictor as _P                                              # noqa: E402


def stats(bias):
    """回報一組偏差(預測 - 實測)的五個數字。負 = 低估。"""
    b = [x for x in bias if x is not None and math.isfinite(x)]
    if not b:
        return None
    neg = sum(1 for x in b if x < 0)
    return {"n": len(b), "med": st.median(b), "amed": abs(st.median(b)),
            "lo": min(b), "hi": max(b), "sd": st.pstdev(b),
            "neg": neg, "one_sided": max(neg, len(b) - neg)}


def line(name, s, extra=""):
    if s is None:
        print(f"  {name:<28s}  (無資料)")
        return
    print(f"  {name:<28s} 中位 {s['med']:+7.2f}  |中位| {s['amed']:6.2f}  "
          f"全距 {s['lo']:+7.2f}…{s['hi']:+7.2f}  sd {s['sd']:5.2f}  "
          f"單邊 {s['one_sided']:2d}/{s['n']:<2d} {extra}")


def pearson(a, b):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 3:
        return float("nan")
    return float(np.corrcoef(a[m], b[m])[0, 1])


def run_sim(recs, *, law, tau_g=None, tau_p=None, dt=0.01, load_cl=True,
            pitch_coupling=True):
    pn = sim3d.PNParams(pitch_min_roll_coupling=pitch_coupling)
    sp = sim3d.SimParams(
        throttle_law=law, dt=dt, use_load_factor_cl=load_cl,
        tau_gamma_s=tau_g if tau_g is not None else sim3d.TAU_GAMMA_S,
        tau_phi_s=tau_p if tau_p is not None else sim3d.TAU_PHI_S)
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
    n = len(recs)
    t_wall = [d["t_wall_s"] for d in recs]
    t_sim = [d["t_sim_s"] for d in recs]
    gs_td = [d["gs_touch_ms"] for d in recs]
    r_td = [d["r_touch_m"] for d in recs]

    print(f"== 資料 n = {n} ==")
    print(f"  t_wall 中位 {st.median(t_wall):.2f} 全距 {min(t_wall):.1f}"
          f"…{max(t_wall):.1f} sd {st.pstdev(t_wall):.2f}")
    print(f"  t_sim  中位 {st.median(t_sim):.2f} 全距 {min(t_sim):.1f}"
          f"…{max(t_sim):.1f} sd {st.pstdev(t_sim):.2f}"
          f"   (凍結總計 {sum(d['t_freeze_s'] for d in recs):.1f} s)")

    # ── 基線估計器 ────────────────────────────────────────────────────
    naive_r = [d["r_m"] / d["V_ms"] for d in recs]
    naive_R = [d["R_slant_m"] / d["V_ms"] for d in recs]
    naive_rgs = [d["r_m"] / (d["V_ms"] * math.cos(math.radians(d["gamma_deg"])))
                 for d in recs]
    pred_lon, pred_lon_gs = [], []
    for d in recs:
        o = _P.predict_terminal(d["z_m"], d["r_m"], d["V_ms"], thr_max_pct=100.0,
                                v_req=210.0, home_amsl=0.0, dt=0.05)
        pred_lon.append(o["t_go"])
        pred_lon_gs.append(o["gs_end"])

    # ── sim3d 兩種油門政策 ────────────────────────────────────────────
    o_clock = run_sim(recs, law="clock")
    o_vmax = run_sim(recs, law="vmax")

    print()
    print("== 逐架次(sim3d, throttle=clock = 實測飛的那條律)==")
    print(f"{'tag':<9s}{'eps':>7s}{'t_wall':>8s}{'t_sim':>7s}{'sim3d':>7s}"
          f"{'d_wall':>8s}{'d_sim':>7s}{'gs_m':>7s}{'gs_p':>7s}"
          f"{'rtd_m':>8s}{'rtd_p':>8s}{'ok':>4s}")
    for k, d in enumerate(recs):
        tg = o_clock["t_go"][k]
        print(f"{d['tag']:<9s}{d['eps_deg']:7.1f}{d['t_wall_s']:8.2f}"
              f"{d['t_sim_s']:7.2f}{tg:7.2f}"
              f"{tg - d['t_wall_s']:+8.2f}{tg - d['t_sim_s']:+7.2f}"
              f"{d['gs_touch_ms']:7.1f}{o_clock['gs_end'][k]:7.1f}"
              f"{d['r_touch_m']:8.1f}{o_clock['r_end'][k]:8.1f}"
              f"{'Y' if o_clock['valid'][k] else 'N':>4s}")

    print()
    print("== ① 俯衝耗時偏差(預測 - 實測);負 = 低估 ==")
    print("-- vs t_wall(牆鐘,= 事前登記閘門用的量)--")
    line("naive r/V(登記基線)", stats([a - b for a, b in zip(naive_r, t_wall)]))
    line("naive r/gs", stats([a - b for a, b in zip(naive_rgs, t_wall)]))
    line("naive R/V(斜距)", stats([a - b for a, b in zip(naive_R, t_wall)]))
    line("predictor.py(縱向)", stats([a - b for a, b in zip(pred_lon, t_wall)]))
    line("常數 = 實測中位", stats([st.median(t_wall) - b for b in t_wall]))
    line("sim3d  throttle=clock",
         stats([a - b for a, b in zip(o_clock["t_go"], t_wall)]))
    line("sim3d  throttle=vmax",
         stats([a - b for a, b in zip(o_vmax["t_go"], t_wall)]))
    print("-- vs t_sim(扣掉模擬凍結;物理模擬器預測的是機體時間)--")
    line("naive r/V", stats([a - b for a, b in zip(naive_r, t_sim)]))
    line("naive R/V(斜距)", stats([a - b for a, b in zip(naive_R, t_sim)]))
    line("predictor.py(縱向)", stats([a - b for a, b in zip(pred_lon, t_sim)]))
    line("常數 = 實測中位", stats([st.median(t_sim) - b for b in t_sim]))
    line("sim3d  throttle=clock",
         stats([a - b for a, b in zip(o_clock["t_go"], t_sim)]))
    line("sim3d  throttle=vmax",
         stats([a - b for a, b in zip(o_vmax["t_go"], t_sim)]))

    print()
    print("== ② 鑑別力(這才是 A3 指出的真問題:偏差可以校,鑑別力不能)==")
    print(f"  實測 t_sim  sd = {st.pstdev(t_sim):.2f} s(要解釋的變異)")
    for nm, v in (("naive r/V", naive_r), ("naive R/V", naive_R),
                  ("predictor.py", pred_lon),
                  ("sim3d clock", list(o_clock["t_go"])),
                  ("sim3d vmax", list(o_vmax["t_go"]))):
        vv = [x for x in v if math.isfinite(x)]
        print(f"  {nm:<14s} 輸出 sd {st.pstdev(vv):5.2f}  全距 {min(vv):5.1f}"
              f"…{max(vv):5.1f}   r(pred, t_sim) = {pearson(v, t_sim):+.3f}"
              f"   r(pred, t_wall) = {pearson(v, t_wall):+.3f}")

    print()
    print("== ③ 觸地地速 gs(KPI②)==")
    line("predictor.py(縱向)", stats([a - b for a, b in zip(pred_lon_gs, gs_td)]))
    line("sim3d clock", stats([a - b for a, b in zip(o_clock["gs_end"], gs_td)]))
    line("sim3d vmax", stats([a - b for a, b in zip(o_vmax["gs_end"], gs_td)]))
    print(f"  實測 gs 中位 {st.median(gs_td):.1f}  全距 {min(gs_td):.1f}"
          f"…{max(gs_td):.1f}")
    print(f"  r(sim3d clock, 實測 gs) = "
          f"{pearson(list(o_clock['gs_end']), gs_td):+.3f}")

    print()
    print("== ④ 落點距紅心 r_touch(KPI③)==")
    print(f"  實測 中位 {st.median(r_td):.1f} m  全距 {min(r_td):.1f}"
          f"…{max(r_td):.1f}  (⚠️ 兩個離群 = 真的脫靶:"
          + ", ".join(f"{d['tag']} {d['r_touch_m']:.0f}m"
                      for d in recs if d["r_touch_m"] > 100) + ")")
    print(f"  sim3d clock 中位 {st.median([abs(x) for x in o_clock['r_end']]):.1f} m"
          f"  全距 {min(o_clock['r_end']):.1f}…{max(o_clock['r_end']):.1f}")
    sub = [(d, k) for k, d in enumerate(recs) if d["r_touch_m"] <= 100]
    line("r_touch(排除 2 個脫靶)",
         stats([abs(o_clock["r_end"][k]) - d["r_touch_m"] for d, k in sub]))

    print()
    print("== ⑤ 靈敏度:哪個旋鈕真的在動偏差 ==")
    base = stats([a - b for a, b in zip(o_clock["t_go"], t_sim)])
    print(f"  基準(clock, tau_g={sim3d.TAU_GAMMA_S}, tau_p={sim3d.TAU_PHI_S}, "
          f"dt=0.01, 載重c_L, pitch耦合開)")
    line("  基準", base)
    for nm, kw in (("tau_gamma 0.5", dict(tau_g=0.5)),
                   ("tau_gamma 2.0", dict(tau_g=2.0)),
                   ("tau_gamma 3.0", dict(tau_g=3.0)),
                   ("tau_phi 0.2", dict(tau_p=0.2)),
                   ("tau_phi 1.5", dict(tau_p=1.5)),
                   ("dt 0.05", dict(dt=0.05)),
                   ("dt 0.002", dict(dt=0.002)),
                   ("c_L 用準平衡(= predictor)", dict(load_cl=False)),
                   ("pitch/roll 耦合關", dict(pitch_coupling=False))):
        o = run_sim(recs, law="clock", **kw)
        line("  " + nm, stats([a - b for a, b in zip(o["t_go"], t_sim)]))

    print()
    print("== ⑥ eps 的貢獻(A1 預測:cos(eps) 項的量級剛好對得上單邊性)==")
    o_noeps = run_sim(recs, law="clock")
    o0 = sim3d.fly(
        np.array([d["r_m"] for d in recs]), np.array([d["z_m"] for d in recs]),
        np.array([d["V_ms"] for d in recs]),
        np.radians([d["gamma_deg"] for d in recs]),
        np.zeros(n),                                   # eps 強制歸零
        sp=sim3d.SimParams(throttle_law="clock", dt=0.01),
        t_rem0=np.array([d["t_rem0_s"] for d in recs]))
    line("eps = 實測", stats([a - b for a, b in zip(o_noeps["t_go"], t_sim)]))
    line("eps = 0(拿掉側向)", stats([a - b for a, b in zip(o0["t_go"], t_sim)]))
    dd = [a - b for a, b in zip(o_noeps["t_go"], o0["t_go"])]
    print(f"  eps 單獨貢獻的耗時:中位 {st.median(dd):+.2f} s  "
          f"全距 {min(dd):+.2f}…{max(dd):+.2f}")

    print()
    print("== ⑦ 終端航跡角 gamma_touch(⚠️ predictor 的 gamma_end 不可用來比對:")
    print("      純視線政策下 h→0 時 gamma_LOS→0,回傳值是收尾步的假象)==")
    gam_td = [d["gamma_touch_deg"] for d in recs]
    line("sim3d clock",
         stats([a - b for a, b in zip(o_clock["gamma_end_deg"], gam_td)]))
    print(f"  實測 中位 {st.median(gam_td):.1f}deg  全距 {min(gam_td):.1f}"
          f"…{max(gam_td):.1f}   sim3d 中位 "
          f"{np.median(o_clock['gamma_end_deg']):.1f}deg")

    print()
    print("== ⑧ 查表 vs 直接模擬(格點解析度夠不夠)==")
    tpath = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "ditcg_table_ours.npz")
    if not os.path.exists(tpath):
        print(f"  (找不到 {tpath};先跑 build_table.py --preset ours)")
    else:
        import build_table as BT
        tbl = BT.load(tpath)
        d_int, oor, inv = [], 0, 0
        for k, d in enumerate(recs):
            g, why = BT.lookup(tbl, d["r_m"], d["z_m"], d["V_ms"],
                               d["gamma_deg"], abs(d["eps_deg"]))
            if why == "out_of_range":
                oor += 1
                continue
            if why == "invalid_corner" or not math.isfinite(g):
                inv += 1
                continue
            d_int.append(g - o_vmax["t_go"][k])
        print(f"  32 個放行狀態:可查 {len(d_int)},格點外 {oor},"
              f"角點含不可判 {inv}")
        line("查表 - 直接模擬(同 vmax 政策)", stats(d_int))
        print("  ⇒ 這一項量的是**內插誤差**,與物理無關。它必須遠小於 3.0 s 的")
        print("     閘門,否則『通過/失敗』都是格點解析度的產物。")

    print()
    print("== ⑨ 健康度 ==")
    for nm, o in (("clock", o_clock), ("vmax", o_vmax)):
        v = o["valid"]
        codes = {}
        for c in o["fail_code"]:
            codes[int(c)] = codes.get(int(c), 0) + 1
        print(f"  {nm:<6s} valid {int(v.sum())}/{n}  fail_code {codes}  "
              f"nz飽和比例中位 {np.median(o['nz_sat_frac']):.3f}  "
              f"phi飽和比例中位 {np.median(o['phi_sat_frac']):.3f}  "
              f"gamma_min 中位 {np.median(o['gamma_min_deg']):.1f}deg  "
              f"n_total_max 最大 {o['n_total_max'].max():.2f}")


if __name__ == "__main__":
    main()
