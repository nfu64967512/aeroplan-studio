#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fidelity_long.py —— F 階段:**縱向權限**掃描的保真度閘門。

⭐⭐ 這一支不掃參數,它回答的是**掃描有沒有資格開始**:
   `sim3d.py` 重現不了「nz 觸夾制比例」這個我們要掃的量,掃描結果就沒有意義。

## 為什麼閘門盯的是 nz 夾制比例,不是終端俯角

32 架次逐拍重放的預測子排名(vs 觸地誤差,n=30):
    nz 觸夾制比例 spearman +0.914 >> ZEM@1.5s +0.561 >> LOS 地板觸發率 +0.404
    >> roll clip 比例 -0.084(29/32 架是 0.000%)
⇒ 瓶頸在**縱向**權限。而 gamma_dot = (nz_min - cos gamma)*g/V 是那個權限的閘。
⚠️ **判準一律是「殘餘高度 + 觸地誤差」,不准用終端俯角** —— 那個判準已經騙過
   兩次(n_vert 5→3 讓俯角變陡但誤差變差)。本檔把終端 gamma 只印成診斷欄。

## 實測側的 nz 夾制比例怎麼來(CSV 沒有這一欄,必須重放)

tot_*.csv 不記 nz。本檔逐列重建 `tot_node._pn_cmd` 2617-2626 的**指令側** nz:

    sig_dot = (hdot*d - h*ddot)/R^2,  ddot = -(dn*vn + de*ve)/d
    gdot    = -n_vert * sig_dot
    nz_raw  = cos(gamma) + V*gdot/g   ⇒ 落在 [pn_nz_min, 3.0] 之外就記一次夾制

這與 `sim3d.fly` 的 `nz_sat_frac` 是**同一條式子、同一個計數點**
(sim3d 導引 tick 的 `nz_sat[idx] += (nz_raw != nz)`)⇒ 兩側同定義,才有資格比。

資料層慣例照 `truth32.py`(它已踩過三個坑):
  * release = 首次 mode=="dive";touchdown = 其後**不限 mode** 的首次 agl<=0
  * 幅值取 `gs` / `gam_deg` 欄(**不自行差分**);vh=gs、hdot=vh*tan(gamma)、
    V=vh/cos(gamma)
  * 航向用 `truth32._course_rad`(只取位移**方向**,免疫 t 的 0.1 s 量化,
    且窗不跨凍結空檔)

⚠️ 已知落差,先講在前面:本檔重建出來的
   spearman(nz夾制比例, 觸地誤差) = +0.581(n=30),而交辦寫的是 **+0.914**。
   機制面完全對上(四架殘餘高度 28.3-40.2 m 的正是低側夾制 10.8-13.4% 的那四架,
   逐架對號入座),但**係數對不上** ⇒ 交辦那個數字用的定義與本檔不同
   (低側夾制比例本檔量到 10.8-13.4%,交辦寫 15.9-19.2%,比值約 1.45)。
   已試過的替代定義:改用**實得** gamma_dot 反推 nz(spearman **-0.399**,方向就錯)
   ⇒ 排除。本檔一律用指令側定義,而且**兩側同定義**,所以閘門 (a) 本身仍成立;
   但「+0.914」這個數字在本輪**沒有被獨立重現**,引用時要標。

## 閘門(事前寫死,不准事後調)

    (a) spearman(sim nz夾制比例, 實測 nz夾制比例) >= 0.60
    (b) 殘餘高度:中位絕對誤差 < 10 m
    (c) 負對照 N1(pn_los_floor=0, pn_nz_min=0.05)**必須崩掉**
    (d) 負對照 N2(n_vert=3)與 N3(n_vert=7)**必須都比現行差**

主母體 n=30(排除 RRD3-U2 / RRE7-U2 兩個真脫靶 >100 m),與交辦排名同母體;
n=32 一併報成次要。任一項不過 ⇒ 本輪掃描結果**不得用於任何飛行決策**。
"""
from __future__ import annotations

import argparse
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

G = 9.80665
OUTLIER_M = 100.0        # 真脫靶門檻(m);主母體排除

# ── 事前寫死的閘門常數 ────────────────────────────────────────────────
GATE_A_SPEARMAN = 0.60
GATE_B_MED_ABS_M = 10.0
#: (c) N1「崩掉」的判準:落地率明顯掉,**或**中位誤差爆 3 倍以上。
#:     實測對照是 RPQ 0/6 落地、RPK/RPL 5/12、誤差 1.5-2423 m。
GATE_C_LAND_DROP = 0.25          # 落地率至少要掉這麼多(絕對比例)
GATE_C_MISS_RATIO = 3.0          # 或中位誤差至少要變這麼多倍
#: (d) N2/N3 只要求**方向**:中位誤差或 p90 誤差不得優於現行。
#:     ⚠️ 實測是別的批次別的組態量的,不要求數值吻合。

CONFIGS = {
    "CUR": dict(label="CUR 現行 (floor=1.0, nz_min=-1.0, n_vert=5)", kw={}),
    "N1": dict(label="N1  pn_los_floor=0.0, pn_nz_min=0.05",
               kw=dict(pn_los_floor=0.0, pn_nz_min=0.05)),
    "N2": dict(label="N2  n_vert=3", kw=dict(n_vert=3.0)),
    "N3": dict(label="N3  n_vert=7", kw=dict(n_vert=7.0)),
}


# ══════════════════════════════════════════════════════════════════════
# 實測側:逐列重放 _pn_cmd 的指令 nz
# ══════════════════════════════════════════════════════════════════════
def replay_nz(rows, tlat, tlon, *, n_vert=5.0, nz_min=-1.0, nz_max=3.0):
    """逐列重放放行→觸地段。回傳 dict;抽不出來回 None。

    附帶量(診斷用,不進閘門):
      climb_m      放行後仍爬升的最大量(m)。⚠️ sim3d 一放行就開始下降,
                   結構上生不出這個 ⇒ 是「缺了什麼物理」的直接指紋。
      gam_min_deg  全段最負的航跡角。
    """
    i_rel = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"), None)
    if i_rel is None:
        return None
    i_td = next((k for k in range(i_rel, len(rows))
                 if truth32._f(rows[k], "agl") <= 0.0), None)
    if i_td is None:
        return None

    seg = rows[i_rel:i_td + 1]
    z_rel = truth32._f(rows[i_rel], "agl")
    climb = max(0.0, max(truth32._f(r, "agl") for r in seg) - z_rel)
    gmin = min(truth32._f(r, "gam_deg") for r in seg)

    n = lo = hi = 0
    for k in range(i_rel, i_td):
        row = rows[k]
        lat, lon = truth32._f(row, "lat"), truth32._f(row, "lon")
        dn = (tlat - lat) * truth32.EARTH
        de = (tlon - lon) * truth32._mlon(lat)
        d = max(1e-3, math.hypot(dn, de))
        h = truth32._f(row, "agl")
        R = max(1e-3, math.hypot(d, h))
        gam = math.radians(truth32._f(row, "gam_deg"))
        vh = truth32._f(row, "gs")                    # gs 是水平地速
        hdot = vh * math.tan(gam)
        V = max(1e-3, math.hypot(vh, hdot))
        chi, _ = truth32._course_rad(rows, k)
        vn, ve = vh * math.cos(chi), vh * math.sin(chi)
        ddot = -(dn * vn + de * ve) / d
        sig_dot = (hdot * d - h * ddot) / (R * R)
        nz_raw = math.cos(gam) - n_vert * sig_dot * V / G
        n += 1
        if nz_raw < nz_min:
            lo += 1
        elif nz_raw > nz_max:
            hi += 1
    t = max(n, 1)
    return dict(nz_clip=(lo + hi) / t, nz_lo=lo / t, nz_hi=hi / t, nz_n=n,
                climb_m=climb, gam_min_meas_deg=gmin)


def measured():
    """32 架次的實測量。truth32 的欄位再加 nz 三欄。"""
    recs = truth32.load()
    by_tag = {}
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            o = replay_nz(rows, tlat, tlon)
            if o is not None:
                by_tag[f"{run}-U{sysid}"] = o
    nan6 = dict(nz_clip=float("nan"), nz_lo=float("nan"), nz_hi=float("nan"),
                nz_n=0, climb_m=float("nan"), gam_min_meas_deg=float("nan"))
    for d in recs:
        d.update(by_tag.get(d["tag"]) or nan6)
        d["is_outlier"] = d["r_touch_m"] > OUTLIER_M
    return recs


# ══════════════════════════════════════════════════════════════════════
# 模擬側
# ══════════════════════════════════════════════════════════════════════
def simulate(recs, *, law="clock", dt=0.01, tau_g=None, **pn_kw):
    pn = sim3d.PNParams(**pn_kw)
    sp = sim3d.SimParams(throttle_law=law, dt=dt,
                         tau_gamma_s=(sim3d.TAU_GAMMA_S if tau_g is None
                                      else tau_g))
    o = sim3d.fly(
        np.array([d["r_m"] for d in recs]),
        np.array([d["z_m"] for d in recs]),
        np.array([d["V_ms"] for d in recs]),
        np.radians([d["gamma_deg"] for d in recs]),
        np.radians([d["eps_deg"] for d in recs]),
        pn=pn, sp=sp,
        t_rem0=np.array([d["t_rem0_s"] for d in recs]))
    # 殘餘高度:r 首次過零時的高度。⚠️ 沒過零 = 在抵達紅心正上方前就觸地
    #   ⇒ 最近水平通過就發生在觸地那一刻,殘餘高度依定義 = 0(不是 NaN)。
    #   與實測的 z_at_rmin_m(最近水平通過點的 agl)同語意。
    z_res = np.where(np.isfinite(o["z_at_r0"]), o["z_at_r0"], 0.0)
    z_res = np.where(o["fail_code"] == 0, z_res, np.nan)   # 沒觸地就沒有殘餘高度
    miss = np.where(o["fail_code"] == 0, np.abs(o["r_end"]), np.nan)
    return {"raw": o, "nz_clip": o["nz_sat_frac"], "z_res": z_res,
            "miss": miss, "t_go": o["t_go"], "gam_end": o["gamma_end_deg"],
            "landed": (o["fail_code"] == 0)}


# ══════════════════════════════════════════════════════════════════════
def _rank(x):
    """平均秩(處理並列)。"""
    x = np.asarray(x, float)
    order = np.argsort(x, kind="mergesort")
    r = np.empty(len(x), float)
    r[order] = np.arange(1, len(x) + 1, dtype=float)
    xs = x[order]
    i = 0
    while i < len(xs):
        j = i
        while j + 1 < len(xs) and xs[j + 1] == xs[i]:
            j += 1
        if j > i:
            r[order[i:j + 1]] = (i + j + 2) / 2.0
        i = j + 1
    return r


def spearman(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 3:
        return float("nan")
    ra, rb = _rank(a[m]), _rank(b[m])
    if np.std(ra) == 0 or np.std(rb) == 0:
        return float("nan")
    return float(np.corrcoef(ra, rb)[0, 1])


def pearson(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 3:
        return float("nan")
    return float(np.corrcoef(a[m], b[m])[0, 1])


def med(v):
    v = [x for x in np.asarray(v, float) if math.isfinite(x)]
    return st.median(v) if v else float("nan")


def p90(v):
    v = [x for x in np.asarray(v, float) if math.isfinite(x)]
    return float(np.percentile(v, 90)) if v else float("nan")


def fnum(x, w=7, p=2):
    if x is None or not math.isfinite(x):
        return "n/a".rjust(w)
    return f"{x:{w}.{p}f}"


# ══════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--law", default="clock", choices=("clock", "vmax"),
                    help="油門政策。clock = 實測飛的那條律(預設,重現用)")
    ap.add_argument("--dt", type=float, default=0.01)
    args = ap.parse_args()

    recs = measured()
    n = len(recs)
    keep = np.array([not d["is_outlier"] for d in recs])

    sims = {k: simulate(recs, law=args.law, dt=args.dt, **v["kw"])
            for k, v in CONFIGS.items()}
    cur = sims["CUR"]

    m_clip = np.array([d["nz_clip"] for d in recs])
    m_zres = np.array([d["z_at_rmin_m"] for d in recs])
    m_miss = np.array([d["r_touch_m"] for d in recs])
    m_twall = np.array([d["t_wall_s"] for d in recs])
    m_tsim = np.array([d["t_sim_s"] for d in recs])
    m_gam = np.array([d["gamma_touch_deg"] for d in recs])

    print("=" * 104)
    print(f"fidelity_long.py   n={n} 架次   油門政策={args.law}   dt={args.dt}")
    print(f"主母體 n={int(keep.sum())}(排除觸地誤差 >{OUTLIER_M:.0f} m 的真脫靶:"
          + ", ".join(f"{d['tag']} {d['r_touch_m']:.0f}m"
                      for d in recs if d["is_outlier"]) + ")")
    print("=" * 104)

    # ── 表 1:正向重現 ────────────────────────────────────────────────
    print("\n【表 1】正向重現(現行組態)—— 逐架次   m=實測 s=模擬")
    print("⚠️ 終端 gamma 只是診斷欄,**不是判準**")
    hdr = ("nz%_m", "nz%_s", "zres_m", "zres_s", "err_m", "err_s",
           "T_wall", "T_sim", "T_s", "gam_m", "gam_s")
    print(f"{'tag':<10s}" + "".join(f"{h:>8s}" for h in hdr) + f"{'ok':>4s}")
    for k, d in enumerate(recs):
        flag = "*" if d["is_outlier"] else " "
        print(f"{d['tag']:<9s}{flag}"
              f"{m_clip[k]*100:8.2f}{cur['nz_clip'][k]*100:8.2f}"
              f"{m_zres[k]:8.1f}{cur['z_res'][k]:8.1f}"
              f"{m_miss[k]:8.1f}{cur['miss'][k]:8.1f}"
              f"{m_twall[k]:8.2f}{m_tsim[k]:8.2f}{cur['t_go'][k]:8.2f}"
              f"{m_gam[k]:8.1f}{cur['gam_end'][k]:8.1f}"
              f"{'Y' if cur['landed'][k] else 'N':>4s}")
    print("  (* = 真脫靶,不進主母體)")

    def blk(mask, tag):
        print(f"\n  ── 匯總 {tag}(n={int(mask.sum())})──")
        rows = (
            ("(a) nz 夾制比例 %", m_clip * 100, cur["nz_clip"] * 100),
            ("(b) 殘餘高度 m", m_zres, cur["z_res"]),
            ("(c) 觸地誤差 m", m_miss, cur["miss"]),
            ("(d) 俯衝耗時 s vs t_wall", m_twall, cur["t_go"]),
            ("(d') 俯衝耗時 s vs t_sim", m_tsim, cur["t_go"]),
            ("(e) 終端 gamma deg [診斷]", m_gam, cur["gam_end"]),
        )
        print(f"  {'量':<26s}{'實測中位':>10s}{'模擬中位':>10s}"
              f"{'中位偏差':>10s}{'|偏差|中位':>11s}{'spearman':>10s}{'pearson':>9s}")
        for name, mv, sv in rows:
            a, b = np.asarray(mv, float)[mask], np.asarray(sv, float)[mask]
            fin = np.isfinite(a) & np.isfinite(b)
            dif = b[fin] - a[fin]
            print(f"  {name:<24s}{fnum(med(a), 10):>10s}{fnum(med(b), 10):>10s}"
                  f"{fnum(med(dif), 10):>10s}{fnum(med(np.abs(dif)), 11):>11s}"
                  f"{spearman(a, b):>10.3f}{pearson(a, b):>9.3f}")

    blk(keep, "主母體")
    blk(np.ones(n, bool), "全部")

    # ── 表 2:負對照 ──────────────────────────────────────────────────
    print("\n" + "=" * 104)
    print("【表 2】負對照 —— 同樣 32 個放行狀態,只換組態(主母體 n=30)")
    print("⚠️ N2/N3 的實測是**別的批次別的組態**量的 ⇒ 只看方向,不比數值")
    print(f"\n{'組態':<44s}{'落地率':>8s}{'zres中位':>10s}{'zres_p90':>10s}"
          f"{'err中位':>9s}{'err_p90':>9s}{'err最大':>10s}{'nz夾制%':>9s}"
          f"{'耗時中位':>10s}")
    summ = {}
    for key in ("CUR", "N1", "N2", "N3"):
        s = sims[key]
        mk = keep
        row = dict(land=float(s["landed"][mk].mean()),
                   zres=med(s["z_res"][mk]), zres90=p90(s["z_res"][mk]),
                   err=med(s["miss"][mk]), err90=p90(s["miss"][mk]),
                   errmax=(float(np.nanmax(s["miss"][mk]))
                           if np.isfinite(s["miss"][mk]).any() else float("nan")),
                   nz=med(s["nz_clip"][mk]) * 100, t=med(s["t_go"][mk]))
        summ[key] = row
        print(f"{CONFIGS[key]['label']:<44s}{row['land']*100:7.0f}%"
              f"{fnum(row['zres'], 10)}{fnum(row['zres90'], 10)}"
              f"{fnum(row['err'], 9)}{fnum(row['err90'], 9)}"
              f"{fnum(row['errmax'], 10)}{fnum(row['nz'], 9)}"
              f"{fnum(row['t'], 10)}")
    ref = summ["CUR"]

    print("\n  失效碼分布(0=觸地 1=逾時 2=打過頭>5km 3=速度掉到地板 4=NaN),全 32:")
    for key in ("CUR", "N1", "N2", "N3"):
        c = {}
        for x in sims[key]["raw"]["fail_code"]:
            c[int(x)] = c.get(int(x), 0) + 1
        print(f"    {key:<4s} {c}")

    print("\n  ── 逐架次負對照(觸地誤差 m;主母體)──")
    print(f"{'tag':<10s}{'實測':>9s}{'CUR':>9s}{'N1':>10s}{'N2':>9s}{'N3':>9s}")
    for k, d in enumerate(recs):
        if d["is_outlier"]:
            continue
        print(f"{d['tag']:<10s}{m_miss[k]:9.1f}{fnum(sims['CUR']['miss'][k], 9)}"
              f"{fnum(sims['N1']['miss'][k], 10)}{fnum(sims['N2']['miss'][k], 9)}"
              f"{fnum(sims['N3']['miss'][k], 9)}")

    # ── 表 3:閘門判定 ────────────────────────────────────────────────
    print("\n" + "=" * 104)
    print("【表 3】閘門判定(規則事前寫死,逐字照抄,未經事後調整)")

    verdicts = []

    sa30 = spearman(cur["nz_clip"][keep], m_clip[keep])
    sa32 = spearman(cur["nz_clip"], m_clip)
    ok_a = math.isfinite(sa30) and sa30 >= GATE_A_SPEARMAN
    verdicts.append(("a", ok_a,
                     f"spearman(sim nz夾制, 實測 nz夾制) = {sa30:+.3f} "
                     f"{'>=' if ok_a else '<'} {GATE_A_SPEARMAN:.2f}"
                     f"   [n=32 時 {sa32:+.3f};pearson n=30 "
                     f"{pearson(cur['nz_clip'][keep], m_clip[keep]):+.3f}]"))

    dz = cur["z_res"][keep] - m_zres[keep]
    b_med = med(np.abs(dz))
    ok_b = math.isfinite(b_med) and b_med < GATE_B_MED_ABS_M
    verdicts.append(("b", ok_b,
                     f"殘餘高度中位絕對誤差 = {b_med:.2f} m "
                     f"{'<' if ok_b else '>='} {GATE_B_MED_ABS_M:.0f} m"
                     f"   [中位帶號偏差 {med(dz):+.2f} m,"
                     f"p90|偏差| {p90(np.abs(dz)):.2f} m]"))

    c_land = ref["land"] - summ["N1"]["land"]
    c_ratio = (summ["N1"]["err"] / ref["err"]
               if (math.isfinite(summ["N1"]["err"]) and ref["err"] > 0)
               else float("inf"))
    ok_c = (c_land >= GATE_C_LAND_DROP) or (c_ratio >= GATE_C_MISS_RATIO)
    verdicts.append(("c", ok_c,
                     f"N1 崩掉?落地率 {ref['land']*100:.0f}% -> "
                     f"{summ['N1']['land']*100:.0f}%(掉 {c_land*100:.0f} 個百分點,"
                     f"門檻 {GATE_C_LAND_DROP*100:.0f});中位誤差 "
                     f"{fnum(ref['err'],1,2).strip()} -> "
                     f"{fnum(summ['N1']['err'],1,2).strip()} m"
                     f"(x{c_ratio:.2f},門檻 x{GATE_C_MISS_RATIO:.0f})"))

    d_parts = []
    ok_d = True
    for key in ("N2", "N3"):
        werr = ((summ[key]["err"] > ref["err"])
                if math.isfinite(summ[key]["err"]) else True)
        w90 = ((summ[key]["err90"] > ref["err90"])
               if math.isfinite(summ[key]["err90"]) else True)
        wz = ((summ[key]["zres"] > ref["zres"])
              if math.isfinite(summ[key]["zres"]) else True)
        good = werr or w90
        ok_d = ok_d and good
        d_parts.append(f"{key} 中位誤差 {fnum(ref['err'],1,2).strip()}->"
                       f"{fnum(summ[key]['err'],1,2).strip()}"
                       f"({'差' if werr else '**沒變差**'})、p90 "
                       f"{fnum(ref['err90'],1,2).strip()}->"
                       f"{fnum(summ[key]['err90'],1,2).strip()}"
                       f"({'差' if w90 else '**沒變差**'})、殘餘高度 "
                       f"{fnum(ref['zres'],1,2).strip()}->"
                       f"{fnum(summ[key]['zres'],1,2).strip()}"
                       f"({'差' if wz else '沒變差'})")
    verdicts.append(("d", ok_d, "N2/N3 方向:  " + "  |  ".join(d_parts)))

    print()
    for tag, ok, msg in verdicts:
        print(f"  ({tag}) [{'PASS' if ok else 'FAIL'}]  {msg}")

    # ⚠️ 空洞旗標:規則照登記的算,但要把「過得沒有意義」講出來。
    #    規則本身**不改**(改了就是事後調參),只加註。
    sim_noise = float(np.nanstd(cur["miss"][keep]))
    spread = max(abs(summ[c]["err"] - ref["err"]) for c in ("N1", "N2", "N3"))
    if spread <= sim_noise:
        print(f"\n  ⚠️⚠️ (d) 的 PASS 是**空洞的**:四個組態的中位誤差彼此只差 "
              f"{spread:.2f} m,而模擬自身的架次間 sd 就有 {sim_noise:.2f} m "
              f"⇒ 方向判定與擲硬幣無異。")
    if abs(med(cur["z_res"][keep])) < 0.1 and np.nanstd(cur["z_res"][keep]) < 0.5:
        print(f"  ⚠️⚠️ (b) 的 PASS 也是**空洞的**:模擬對 32 架全部吐出殘餘高度 "
              f"~0(sd {np.nanstd(cur['z_res'][keep]):.4f} m),"
              f"中位絕對誤差小是因為 26/30 架實測本來就 ~0;"
              f"實測 28.3-40.2 m 的**四架失效,模擬重現了 0 架**"
              f"(p90|偏差| {p90(np.abs(dz)):.1f} m)。")

    # ── 表 4:不過的原因(診斷,不參與判定)────────────────────────────
    print("\n" + "=" * 104)
    print("【表 4】診斷 —— 不過在哪裡、差多少、為什麼")

    print("\n  (4.1) 鑑別力:模擬輸出的離散度 vs 實測(主母體 n=30)")
    print(f"  {'量':<18s}{'實測 sd':>10s}{'實測全距':>18s}{'模擬 sd':>10s}"
          f"{'模擬全距':>18s}{'r(sim,meas)':>12s}")
    for name, a, b in (("殘餘高度 m", m_zres, cur["z_res"]),
                       ("觸地誤差 m", m_miss, cur["miss"]),
                       ("nz 夾制比例 %", m_clip * 100, cur["nz_clip"] * 100)):
        aa, bb = np.asarray(a, float)[keep], np.asarray(b, float)[keep]
        print(f"  {name:<16s}{np.nanstd(aa):10.3f}"
              f"{f'{np.nanmin(aa):.2f}..{np.nanmax(aa):.2f}':>18s}"
              f"{np.nanstd(bb):10.3f}"
              f"{f'{np.nanmin(bb):.3f}..{np.nanmax(bb):.3f}':>18s}"
              f"{pearson(aa, bb):>12.3f}")
    print("  ⇒ 模擬在殘餘高度上的離散度**幾乎是零** ⇒ 不論怎麼掃,輸出都不會動。")

    print("\n  (4.2) 失效母體長什麼樣:實測的殘餘高度全部集中在 |eps0| 最大的四架")
    print(f"  {'tag':<10s}{'|eps0|':>8s}{'nz低側%':>9s}{'zres_m':>8s}{'err_m':>8s}"
          f"{'zres_s':>8s}{'err_s':>8s}{'nz_s%':>8s}")
    for k, d in sorted(enumerate(recs), key=lambda x: -abs(x[1]["eps_deg"]))[:6]:
        print(f"  {d['tag']:<10s}{abs(d['eps_deg']):8.1f}{d['nz_lo']*100:9.2f}"
              f"{d['z_at_rmin_m']:8.1f}{d['r_touch_m']:8.1f}"
              f"{fnum(cur['z_res'][k], 8)}{fnum(cur['miss'][k], 8)}"
              f"{cur['nz_clip'][k]*100:8.2f}")
    ae = np.abs(np.array([d["eps_deg"] for d in recs]))
    print(f"  |eps0| 有乾淨斷層:57.9deg 以上四架 -> 其餘全部 <= 23.8deg。")
    print(f"  spearman(|eps0|, 實測殘餘高度) = "
          f"{spearman(ae[keep], m_zres[keep]):+.3f};"
          f"  但 spearman(|eps0|, 模擬殘餘高度) = "
          f"{spearman(ae[keep], cur['z_res'][keep]):+.3f}"
          f"  ⇒ **同樣的 eps0 餵進去,模擬照樣命中**。")

    print("\n  (4.3) 缺了什麼物理:放行後的爬升(模擬結構上生不出來)")
    climb = np.array([d["climb_m"] for d in recs])
    gmm = np.array([d["gam_min_meas_deg"] for d in recs])
    print(f"  實測:{int((climb > 1.0).sum())}/{n} 架在放行後**仍在爬升**,"
          f"爬升量中位 {med(climb):.1f} m、最大 {np.nanmax(climb):.1f} m")
    print(f"  模擬:0/{n}(sim3d 從放行的第一個 tick 就開始下降)")
    print(f"  實測最陡航跡角中位 {med(gmm):.1f} deg;模擬 "
          f"{med(cur['raw']['gamma_min_deg']):.1f} deg "
          f"⇒ 模擬全程比實測**淺** "
          f"{abs(med(gmm) - med(cur['raw']['gamma_min_deg'])):.1f} deg")
    print(f"  模擬 roll 飽和比例中位 "
          f"{med(cur['raw']['phi_sat_frac'])*100:.2f}% "
          f"(與實測『roll clip 只佔 1.58%、29/32 是 0』同向 ⇒ 橫向沒問題,"
          f"問題在縱向與 plant)")

    print("\n  (4.4) 要多大的遲滯才生得出殘餘高度(tau_gamma 掃描;"
          f"現行擬合值 = {sim3d.TAU_GAMMA_S:.2f} s = pn_lead ⇒ 零額外遲滯)")
    print(f"  {'tau_gamma s':>12s}{'zres中位':>10s}{'zres_p90':>10s}"
          f"{'err中位':>9s}{'err最大':>10s}{'nz夾制%中位':>13s}{'sp(nz)':>9s}")
    for tg in (1.0, 2.0, 3.0, 5.0, 8.0):
        s = simulate(recs, law=args.law, dt=args.dt, tau_g=tg)
        print(f"  {tg:12.1f}{fnum(med(s['z_res'][keep]), 10)}"
              f"{fnum(p90(s['z_res'][keep]), 10)}{fnum(med(s['miss'][keep]), 9)}"
              f"{fnum(float(np.nanmax(s['miss'][keep])), 10)}"
              f"{fnum(med(s['nz_clip'][keep])*100, 13)}"
              f"{spearman(s['nz_clip'][keep], m_clip[keep]):9.3f}")
    print("  ⇒ 要 tau_gamma >= 3 s(擬合值的 3 倍以上)才開始出現殘餘高度,"
          "而那時觸地誤差已經爆掉。")
    print("  ⚠️ 這一列是**診斷**,不是修法建議。把 tau 調到 3-5 去湊 = 事後調參,"
          "本輪明令禁止。")

    all_ok = all(v[1] for v in verdicts)
    print("\n" + "=" * 104)
    print(f"閘門總判定:**{'PASS' if all_ok else 'FAIL'}**")
    if not all_ok:
        print("⚠️⚠️ 未通過 ⇒ 本輪的掃描結果**不得用於任何飛行決策**。")
        print("     ⚠️ 不要回頭調模擬器去湊 —— 誠實的『模擬器還不夠好』"
              "比湊出來的掃描有價值。")
    print("=" * 104)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
