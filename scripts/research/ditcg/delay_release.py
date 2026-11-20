#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""delay_release.py —— G 階段:測「延後放行到 gamma_IC <= 0」。

⚠️⚠️ 先講清楚本輪的地位:上游保真度閘門(fidelity_long.py)判定 **FAIL**,
   而且失敗理由裡有一條**正中本任務要害**:

       實測 19/32 架「放行後仍在爬升」;sim3d **0/32** ——
       它從放行第一個 tick 就下降,結構上生不出 gamma_IC > 0 這個現象。

   ⇒ 模擬 A/B 對本命題的**鑑別力先驗為零**。本檔仍然跑它(交辦要求),
     但只當「假設層」列印;**主結論一律取實測側**(§1/§2 全部是 CSV 直接量)。

## 段落

  §1 實測判別式重驗:gamma_IC 對觸地誤差到底有沒有鑑別力(RRE16 vs 全 32)
  §2 延後窗口的實測代價:從放行走到首次 gamma<=0,r/z/V/eps/需求俯角各變多少
  §2b 散布代價:各機延後量不同 ⇒ 放行時刻差變大還變小
  §3 模擬 A/B(假設層)+ 與旋鈕組合
  §3b 自我否定的正面檢驗(閉式 need/t_go 兩路)
"""
from __future__ import annotations

import math
import os
import statistics as st
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(os.path.join(
    HERE, "..", "..", "wsl", "sar_swarm_patches")))
import sim3d                                                        # noqa: E402
import truth32                                                      # noqa: E402
import fidelity_long as FL                                          # noqa: E402

G = 9.80665
OUTLIER_M = 100.0
NZ_MIN = -1.0


# ══════════════════════════════════════════════════════════════════════
# 任一列的 5 維狀態(完全沿用 truth32.extract_one 的慣例)
# ══════════════════════════════════════════════════════════════════════
def state_at(rows, k, tlat, tlon):
    row = rows[k]
    lat, lon = truth32._f(row, "lat"), truth32._f(row, "lon")
    dn = (tlat - lat) * truth32.EARTH
    de = (tlon - lon) * truth32._mlon(lat)
    r_h = math.hypot(dn, de)
    z = truth32._f(row, "agl")
    gam = math.radians(truth32._f(row, "gam_deg"))
    gs = truth32._f(row, "gs")
    V = gs / max(math.cos(gam), 1e-6)
    chi, _ = truth32._course_rad(rows, k)
    vn, ve = math.cos(chi), math.sin(chi)
    eps = math.atan2(dn * ve - de * vn, dn * vn + de * ve)
    return dict(idx=k, t=truth32._f(row, "t"), r_m=r_h, z_m=z, V_ms=V,
                gamma_deg=math.degrees(gam), eps_deg=math.degrees(eps),
                gs_ms=gs, mode=row.get("mode", ""),
                t_rem_row=(truth32._f(row, "early_by_s")
                           + truth32._f(row, "secs_to_target")))


def build():
    """32 架次:放行狀態 + 首次 gamma<=0 的延後狀態 + 實測真值。"""
    out = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            i_rel = next((k for k, r in enumerate(rows)
                          if r.get("mode") == "dive"), None)
            if i_rel is None:
                continue
            i_td = next((k for k in range(i_rel, len(rows))
                         if truth32._f(rows[k], "agl") <= 0.0), None)
            if i_td is None:
                continue
            s_rel = state_at(rows, i_rel, tlat, tlon)
            # 首次 gamma <= 0(放行含當拍)
            i_dn = next((k for k in range(i_rel, i_td + 1)
                         if truth32._f(rows[k], "gam_deg") <= 0.0), i_td)
            s_dn = state_at(rows, i_dn, tlat, tlon)
            frz = 0.0
            for a, b in zip(rows[i_rel:i_dn], rows[i_rel + 1:i_dn + 1]):
                dt = truth32._f(b, "t") - truth32._f(a, "t")
                if dt > truth32.GAP_S:
                    frz += dt
            climb = max(0.0, max(truth32._f(r, "agl")
                                 for r in rows[i_rel:i_dn + 1]) - s_rel["z_m"])
            r_last = rows[max(i_rel, i_td - 1)]
            lat_td = truth32._f(rows[i_td], "lat")
            lon_td = truth32._f(rows[i_td], "lon")
            err = math.hypot((tlat - lat_td) * truth32.EARTH,
                             (tlon - lon_td) * truth32._mlon(lat_td))
            r_min, z_rmin = s_rel["r_m"], s_rel["z_m"]
            for b in rows[i_rel:i_td + 1]:
                la, ln = truth32._f(b, "lat"), truth32._f(b, "lon")
                rb = math.hypot((tlat - la) * truth32.EARTH,
                                (tlon - ln) * truth32._mlon(la))
                if rb < r_min:
                    r_min, z_rmin = rb, truth32._f(b, "agl")
            nz = FL.replay_nz(rows, tlat, tlon) or {}
            out.append(dict(
                tag=f"{run}-U{sysid}", run=run, sysid=sysid,
                rel=s_rel, dn=s_dn,
                delay_s=s_dn["t"] - s_rel["t"],
                delay_frz_s=frz,
                delay_fly_s=(s_dn["t"] - s_rel["t"]) - frz,
                climb_m=climb,
                err_m=err, zres_m=z_rmin,
                gam_touch_deg=truth32._f(r_last, "gam_deg"),
                t_wall_s=truth32._f(rows[i_td], "t") - s_rel["t"],
                t_release_s=s_rel["t"], t_touch_s=truth32._f(rows[i_td], "t"),
                nz_clip=nz.get("nz_clip", float("nan")),
                nz_lo=nz.get("nz_lo", float("nan")),
                is_outlier=err > OUTLIER_M,
            ))
    return out


def med(v):
    v = [x for x in v if isinstance(x, float) and math.isfinite(x)]
    return st.median(v) if v else float("nan")


def gam_req_deg(r, z):
    """需求俯角(直指紅心的視線),負值。"""
    return -math.degrees(math.atan2(z, max(r, 1e-3)))


def gdot_avail(V, gam_rad, nz_min=NZ_MIN):
    """可達加陡率 deg/s = (nz_min - cos gamma) g / V。"""
    return math.degrees((nz_min - math.cos(gam_rad)) * G / max(V, 1e-3))


def hdr(s):
    print("\n" + "=" * 104)
    print(s)
    print("=" * 104)


def main():
    D = build()
    keep = [d for d in D if not d["is_outlier"]]
    rre = [d for d in D if d["run"].startswith("RRE")]
    rre_k = [d for d in rre if not d["is_outlier"]]

    # ── §1 判別式重驗 ────────────────────────────────────────────────
    hdr("§1 gamma_IC 判別式重驗")
    for name, S in (("RRE16 全部", rre), ("RRE16 主母體", rre_k),
                    ("全 32", D), ("全 32 主母體 n=30", keep)):
        g = [d["rel"]["gamma_deg"] for d in S]
        e = [d["err_m"] for d in S]
        ea = [abs(d["rel"]["eps_deg"]) for d in S]
        print(f"  {name:<16s} n={len(S):2d}  "
              f"sp(gamma_IC,err)={FL.spearman(g, e):+.3f}   "
              f"sp(|eps0|,err)={FL.spearman(ea, e):+.3f}   "
              f"sp(gamma_IC,|eps0|)={FL.spearman(g, ea):+.3f}")

    print("\n  gamma_IC > 0 vs <= 0(全 32,統計只算主母體):")
    pos = [d for d in D if d["rel"]["gamma_deg"] > 0]
    neg = [d for d in D if d["rel"]["gamma_deg"] <= 0]
    for nm, S in (("gamma_IC>0", pos), ("gamma_IC<=0", neg)):
        ek = [d["err_m"] for d in S if not d["is_outlier"]]
        zk = [d["zres_m"] for d in S if not d["is_outlier"]]
        print(f"    {nm:<12s} n={len(S):2d}(主母體 {len(ek)})  "
              f"err 中位={med(ek):7.2f}  p90={FL.p90(ek):7.2f}  最大={max(ek):8.2f}  "
              f"zres 中位={med(zk):6.2f}  最大={max(zk):6.2f}")

    print("\n  誤差前六大(主母體)是誰:")
    for d in sorted(keep, key=lambda x: -x["err_m"])[:6]:
        print(f"    {d['tag']:<10s} err={d['err_m']:7.2f}  zres={d['zres_m']:6.2f}"
              f"  gamma_IC={d['rel']['gamma_deg']:+7.2f}"
              f"  |eps0|={abs(d['rel']['eps_deg']):6.2f}"
              f"  nz低側={d['nz_lo'] * 100:5.2f}%")

    # ── §1b 分層:gamma_IC 到底預測什麼 ──────────────────────────────
    hdr("§1b gamma_IC 分層 —— 它預測的不是「誤差大小」,而是「整發脫靶」")

    def fisher(a, b, c, d):
        """2x2 右尾 Fisher 精確 p。表 = [[a,b],[c,d]]。"""
        from math import comb
        n = a + b + c + d
        r1, c1 = a + b, a + c
        p = 0.0
        for k in range(max(0, c1 - (n - r1)), min(r1, c1) + 1):
            pk = comb(r1, k) * comb(n - r1, c1 - k) / comb(n, c1)
            if k >= a:
                p += pk
        return p

    for thr in (0.0, 8.0, 12.0):
        hi = [d for d in D if d["rel"]["gamma_deg"] > thr]
        lo = [d for d in D if d["rel"]["gamma_deg"] <= thr]
        a = sum(1 for d in hi if d["is_outlier"])
        b = len(hi) - a
        c = sum(1 for d in lo if d["is_outlier"])
        e = len(lo) - c
        print(f"  gamma_IC > {thr:+5.1f} deg : 真脫靶(>100 m) {a}/{len(hi)}"
              f"   vs  <= 門檻 {c}/{len(lo)}   Fisher 右尾 p={fisher(a, b, c, e):.4f}")
    for thr in (0.0, 12.0):
        hi = [d for d in D if d["rel"]["gamma_deg"] > thr]
        lo = [d for d in D if d["rel"]["gamma_deg"] <= thr]
        a = sum(1 for d in hi if d["err_m"] > 30.0)
        b = len(hi) - a
        c = sum(1 for d in lo if d["err_m"] > 30.0)
        e = len(lo) - c
        print(f"  gamma_IC > {thr:+5.1f} deg : err>30 m       {a}/{len(hi)}"
              f"   vs  <= 門檻 {c}/{len(lo)}   Fisher 右尾 p={fisher(a, b, c, e):.4f}")

    # 偏相關:控制 |eps0| 之後 gamma_IC 還剩多少
    def partial(x, y, z):
        x, y, z = (np.asarray(v, float) for v in (x, y, z))
        rxy, rxz, ryz = (FL.pearson(x, y), FL.pearson(x, z), FL.pearson(y, z))
        den = math.sqrt(max(1e-12, (1 - rxz ** 2) * (1 - ryz ** 2)))
        return (rxy - rxz * ryz) / den

    for nm, S in (("全 32", D), ("主母體 n=30", keep)):
        rk = [FL._rank([d["rel"]["gamma_deg"] for d in S]),
              FL._rank([d["err_m"] for d in S]),
              FL._rank([abs(d["rel"]["eps_deg"]) for d in S])]
        print(f"  {nm:<12s} 偏 spearman(gamma_IC, err | |eps0|) = "
              f"{partial(rk[0], rk[1], rk[2]):+.3f}"
              f"   偏 spearman(|eps0|, err | gamma_IC) = "
              f"{partial(rk[2], rk[1], rk[0]):+.3f}")

    # ── §2 延後窗口的實測代價 ────────────────────────────────────────
    hdr("§2 延後到首次 gamma<=0:實測窗口與幾何代價(逐架,依 gamma_IC 排序)")
    cols = (f"{'tag':<10s}{'gam_IC':>8s}{'delay':>7s}{'fly_s':>7s}"
            f"{'dr_m':>9s}{'dz_m':>8s}{'climb':>7s}{'dV':>7s}"
            f"{'req0':>7s}{'req1':>7s}{'d_req':>7s}"
            f"{'eps0':>7s}{'eps1':>7s}{'gdotA':>7s}{'need':>7s}{'tgo1':>7s}")
    print(cols)
    for d in sorted(D, key=lambda x: -x["rel"]["gamma_deg"]):
        a, b = d["rel"], d["dn"]
        q0 = gam_req_deg(a["r_m"], a["z_m"])
        q1 = gam_req_deg(b["r_m"], b["z_m"])
        ga = gdot_avail(b["V_ms"], math.radians(b["gamma_deg"]))
        need = abs(q1 - b["gamma_deg"]) / max(abs(ga), 1e-6)
        tgo1 = math.hypot(b["r_m"], b["z_m"]) / max(b["V_ms"], 1e-3)
        print(f"{d['tag']:<10s}{a['gamma_deg']:8.2f}{d['delay_s']:7.2f}"
              f"{d['delay_fly_s']:7.2f}"
              f"{b['r_m'] - a['r_m']:9.1f}{b['z_m'] - a['z_m']:8.1f}"
              f"{d['climb_m']:7.1f}{b['V_ms'] - a['V_ms']:7.1f}"
              f"{q0:7.2f}{q1:7.2f}{q1 - q0:7.2f}"
              f"{abs(a['eps_deg']):7.1f}{abs(b['eps_deg']):7.1f}"
              f"{ga:7.2f}{need:7.1f}{tgo1:7.1f}")

    pos_k = pos
    print(f"\n  延後秒數(全 32):中位 {med([d['delay_s'] for d in D]):.2f} s,"
          f"最大 {max(d['delay_s'] for d in D):.2f} s;"
          f"gamma_IC>0 的 {len(pos_k)} 架:"
          f"中位 {med([d['delay_s'] for d in pos_k]):.2f} s,"
          f"最大 {max(d['delay_s'] for d in pos_k):.2f} s")
    print(f"  窗口內 dr 中位 全32 {med([d['dn']['r_m'] - d['rel']['r_m'] for d in D]):+.1f} m /"
          f" gamma_IC>0 {med([d['dn']['r_m'] - d['rel']['r_m'] for d in pos_k]):+.1f} m")
    print(f"  窗口內 dz 中位 全32 {med([d['dn']['z_m'] - d['rel']['z_m'] for d in D]):+.1f} m /"
          f" gamma_IC>0 {med([d['dn']['z_m'] - d['rel']['z_m'] for d in pos_k]):+.1f} m")
    dq = [gam_req_deg(d['dn']['r_m'], d['dn']['z_m'])
          - gam_req_deg(d['rel']['r_m'], d['rel']['z_m']) for d in pos_k]
    print(f"  需求俯角變化(gamma_IC>0):中位 {med(dq):+.2f} deg,最陡 {min(dq):+.2f} deg"
          f"   ⇒ 負值 = 變陡 = 自我否定方向")

    # ── §2b 散布代價 ────────────────────────────────────────────────
    hdr("§2b 散布代價:各機延後量不同 ⇒ 放行時刻差怎麼變")
    print(f"{'run':<8s}{'U1_delay':>10s}{'U2_delay':>10s}{'|d|':>8s}"
          f"{'rel_dt0':>9s}{'rel_dt1':>9s}{'touch_dt':>10s}")
    dd0, dd1 = [], []
    for run in truth32.RUNS:
        p = [d for d in D if d["run"] == run]
        if len(p) != 2:
            continue
        a, b = p
        r0 = abs(a["t_release_s"] - b["t_release_s"])
        r1 = abs((a["t_release_s"] + a["delay_s"])
                 - (b["t_release_s"] + b["delay_s"]))
        t0 = abs(a["t_touch_s"] - b["t_touch_s"])
        dd0.append(r0)
        dd1.append(r1)
        print(f"{run:<8s}{a['delay_s']:10.2f}{b['delay_s']:10.2f}"
              f"{abs(a['delay_s'] - b['delay_s']):8.2f}{r0:9.2f}{r1:9.2f}{t0:10.2f}")
    print(f"\n  雙機放行時刻差:現行中位 {med(dd0):.2f} s → 延後後中位 {med(dd1):.2f} s"
          f"(最大 {max(dd0):.2f} → {max(dd1):.2f});"
          f"惡化 {sum(1 for a, b in zip(dd0, dd1) if b > a)}/{len(dd0)} 趟")

    # ── §3 模擬 A/B(假設層)────────────────────────────────────────
    hdr("§3 模擬 A/B —— ⚠️ 假設層。上游閘門 FAIL,sim3d 生不出「放行後爬升」"
        "(實測 19/32,模擬 0/32)⇒ 本節不得單獨引用")

    def run_sim(states, t_rems, **kw):
        pn = sim3d.PNParams(**kw)
        sp = sim3d.SimParams(throttle_law="clock", dt=0.01)
        o = sim3d.fly(np.array([s["r_m"] for s in states]),
                      np.array([s["z_m"] for s in states]),
                      np.array([s["V_ms"] for s in states]),
                      np.radians([s["gamma_deg"] for s in states]),
                      np.radians([s["eps_deg"] for s in states]),
                      pn=pn, sp=sp, t_rem0=np.array(t_rems))
        zr = np.where(np.isfinite(o["z_at_r0"]), o["z_at_r0"], 0.0)
        zr = np.where(o["fail_code"] == 0, zr, np.nan)
        return dict(z=zr,
                    e=np.where(o["fail_code"] == 0, np.abs(o["r_end"]), np.nan),
                    nz=o["nz_sat_frac"], t=o["t_go"],
                    land=(o["fail_code"] == 0), gam=o["gamma_end_deg"])

    S_rel = [d["rel"] for d in D]
    S_dn = [d["dn"] for d in D]
    T_rel = [d["rel"]["t_rem_row"] for d in D]
    T_dn = [max(1.0, d["rel"]["t_rem_row"] - d["delay_s"]) for d in D]

    KN = {
        "現行": {},
        "nz_min=-2.0": dict(pn_nz_min=-2.0),
        "nz_min=-3.0": dict(pn_nz_min=-3.0),
        "n_vert=7": dict(n_vert=7.0),
        "n_vert7+nz-2": dict(n_vert=7.0, pn_nz_min=-2.0),
    }
    mk = np.array([not d["is_outlier"] for d in D])
    print(f"{'旋鈕':<14s}{'放行點':>12s}{'落地':>6s}{'zres中位':>10s}{'zres_p90':>10s}"
          f"{'err中位':>9s}{'err_p90':>9s}{'err最大':>10s}{'nz夾制%':>9s}{'耗時中位':>10s}")
    RES = {}
    for kname, kw in KN.items():
        for lbl, S, T in (("實際放行", S_rel, T_rel), ("延後gam<=0", S_dn, T_dn)):
            o = run_sim(S, T, **kw)
            RES[(kname, lbl)] = o
            print(f"{kname:<14s}{lbl:>12s}{o['land'][mk].mean() * 100:5.0f}%"
                  f"{FL.med(o['z'][mk]):10.2f}{FL.p90(o['z'][mk]):10.2f}"
                  f"{FL.med(o['e'][mk]):9.2f}{FL.p90(o['e'][mk]):9.2f}"
                  f"{np.nanmax(o['e'][mk]):10.2f}"
                  f"{FL.med(o['nz'][mk]) * 100:9.2f}{FL.med(o['t'][mk]):10.2f}")

    print("\n  ── gamma_IC 最正的 8 架(交辦點名的正組)逐架 ──")
    focus = sorted(range(len(D)), key=lambda i: -D[i]["rel"]["gamma_deg"])[:8]
    print(f"{'tag':<10s}{'gam_IC':>8s}{'實測err':>9s}"
          f"{'A_err':>8s}{'B_err':>8s}{'A_zres':>8s}{'B_zres':>8s}"
          f"{'A_nz%':>8s}{'B_nz%':>8s}{'A_tgo':>8s}{'B_tgo':>8s}")
    A, B = RES[("現行", "實際放行")], RES[("現行", "延後gam<=0")]
    for i in focus:
        d = D[i]
        print(f"{d['tag']:<10s}{d['rel']['gamma_deg']:8.2f}{d['err_m']:9.2f}"
              f"{A['e'][i]:8.2f}{B['e'][i]:8.2f}"
              f"{A['z'][i]:8.2f}{B['z'][i]:8.2f}"
              f"{A['nz'][i] * 100:8.2f}{B['nz'][i] * 100:8.2f}"
              f"{A['t'][i]:8.2f}{B['t'][i]:8.2f}")

    # ── §3b 自我否定的正面檢驗 ──────────────────────────────────────
    hdr("§3b 自我否定檢驗:延後是否讓縱向權限更吃緊")
    a_nz = A["nz"][mk] * 100
    b_nz = B["nz"][mk] * 100
    print(f"  模擬 nz 夾制比例:實際放行 中位 {FL.med(a_nz):.2f}% / p90 {FL.p90(a_nz):.2f}%"
          f"  →  延後 中位 {FL.med(b_nz):.2f}% / p90 {FL.p90(b_nz):.2f}%")
    print("  ⚠️ 模擬 nz 夾制 29/32 架本來就是 0.00%(閘門已揭露)⇒ 這一路解析度不足。\n")
    print("  閉式路:把 gamma 拉到需求視線所需秒數 need = |gam_req - gam| / |gdot_avail|")
    print(f"{'tag':<10s}{'need0':>8s}{'tgo0':>7s}{'ratio0':>8s}"
          f"{'need1':>8s}{'tgo1':>7s}{'ratio1':>8s}{'delta':>8s}")
    rat0, rat1 = [], []
    for d in D:
        a, b = d["rel"], d["dn"]
        for s, acc in ((a, rat0), (b, rat1)):
            q = gam_req_deg(s["r_m"], s["z_m"])
            ga = abs(gdot_avail(s["V_ms"], math.radians(s["gamma_deg"])))
            need = abs(q - s["gamma_deg"]) / max(ga, 1e-6)
            tgo = math.hypot(s["r_m"], s["z_m"]) / max(s["V_ms"], 1e-3)
            acc.append((need, tgo, need / max(tgo, 1e-6)))
        print(f"{d['tag']:<10s}{rat0[-1][0]:8.2f}{rat0[-1][1]:7.1f}{rat0[-1][2]:8.3f}"
              f"{rat1[-1][0]:8.2f}{rat1[-1][1]:7.1f}{rat1[-1][2]:8.3f}"
              f"{rat1[-1][2] - rat0[-1][2]:+8.3f}")
    ip = [i for i in range(len(D)) if D[i]["rel"]["gamma_deg"] > 0]
    print(f"\n  need/t_go 中位:全32 {med([x[2] for x in rat0]):.3f} → "
          f"{med([x[2] for x in rat1]):.3f}"
          f"   gamma_IC>0 的 {len(ip)} 架:"
          f"{med([rat0[i][2] for i in ip]):.3f} → {med([rat1[i][2] for i in ip]):.3f}")
    worse = sum(1 for i in range(len(D)) if rat1[i][2] > rat0[i][2])
    print(f"  ratio 變大(=更吃緊)的架數:{worse}/{len(D)}"
          f"(gamma_IC>0 子群 {sum(1 for i in ip if rat1[i][2] > rat0[i][2])}/{len(ip)})")


if __name__ == "__main__":
    main()
