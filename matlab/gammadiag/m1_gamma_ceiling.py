#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1:實得 gamma-dot 貼不貼物理天花板?

只讀 runs/artifacts/{RRD,RRE}{1..8}/tot_*.csv,只寫本目錄。

逐拍量:
  gdot_meas   實得 = 中央差分(gam_deg 欄),跳過凍結空檔
  cap_up/dn   天花板 = (nz_lim - cos gam)*g/V,nz_lim = 3.0 / -1.0,V = gs/cos gam
  gdot_raw    需求(nz 夾制**之前**)= -n_vert*sig_dot
  gdot_nz     需求(nz 夾制**之後**)
  gdot_los    LOS 地板生效**之後**的實效需求 = (gam_cmd - gam)/pn_lead
"""
from __future__ import annotations
import csv, math, os, statistics as st, sys, json

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts", "research", "ditcg"))
import truth32  # noqa: E402

G = 9.80665
N_VERT = 5.0        # pn_n_vert (tot_node.py:701 default; contract does not override)
PN_LEAD = 1.0       # pn_lead (tot_node.py:710)
LOS_FLOOR = 1.0     # contract pn_los_floor = 1.0
NZ_MIN, NZ_MAX = -1.0, 3.0
GAP_S = 0.3
HALF = int(os.environ.get("HALF", "5"))   # central-difference half window (rows)
SPAN_LO = float(os.environ.get("SPAN_LO", "0.20"))
SPAN_HI = float(os.environ.get("SPAN_HI", "0.80"))


def dequant(seg):
    out, run_t, k = [], None, 0
    for r in seg:
        t = truth32._f(r, "t")
        if run_t is not None and abs(t - run_t) < 1e-9:
            k += 1
        else:
            run_t, k = t, 0
        out.append(t + 0.05 * k)
    return out


def per_ac(run, sysid, rows, tlat, tlon):
    i0 = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"), None)
    if i0 is None:
        return None
    i1 = next((k for k in range(i0, len(rows))
               if truth32._f(rows[k], "agl") <= 0.0), None)
    if i1 is None:
        return None
    seg = rows[i0:i1 + 1]
    tq = dequant(seg)
    traw = [truth32._f(r, "t") for r in seg]
    n = len(seg)
    recs = []
    for k in range(n):
        r = seg[k]
        gam = math.radians(truth32._f(r, "gam_deg"))
        vh = truth32._f(r, "gs")
        h = truth32._f(r, "agl")
        lat, lon = truth32._f(r, "lat"), truth32._f(r, "lon")
        if any(math.isnan(x) for x in (gam, vh, h, lat, lon)):
            continue
        cg = math.cos(gam)
        if cg < 0.05:
            continue
        V = max(1e-3, vh / cg)
        dn = (tlat - lat) * truth32.EARTH
        de = (tlon - lon) * truth32._mlon(lat)
        d = max(1e-3, math.hypot(dn, de))
        R = max(1e-3, math.hypot(d, h))
        hdot = vh * math.tan(gam)
        chi, _ = truth32._course_rad(seg, k)
        vn, ve = vh * math.cos(chi), vh * math.sin(chi)
        ddot = -(dn * vn + de * ve) / d
        sig_dot = (hdot * d - h * ddot) / (R * R)
        gdot_raw = -N_VERT * sig_dot
        nz_raw = cg + V * gdot_raw / G
        nz = max(NZ_MIN, min(NZ_MAX, nz_raw))
        gdot_nz = (nz - cg) * G / V
        gam_cmd = gam + gdot_nz * PN_LEAD
        gam_los = -math.atan2(h, d)
        if LOS_FLOOR > 0.0:
            gam_cmd = min(gam_cmd, gam_los * LOS_FLOOR)
        gdot_los = (gam_cmd - gam) / PN_LEAD
        cap_up = (NZ_MAX - cg) * G / V
        cap_dn = (NZ_MIN - cg) * G / V

        gm = float("nan")
        lo, hi = k - HALF, k + HALF
        if lo >= 0 and hi < n:
            ok = all(traw[j + 1] - traw[j] <= GAP_S for j in range(lo, hi))
            span = tq[hi] - tq[lo]
            if ok and SPAN_LO <= span <= SPAN_HI:
                gm = math.radians(truth32._f(seg[hi], "gam_deg")
                                  - truth32._f(seg[lo], "gam_deg")) / span
        recs.append(dict(t=tq[k], frac=(tq[k] - tq[0]) / max(1e-6, tq[-1] - tq[0]),
                         gam=gam, V=V, h=h, d=d, R=R,
                         gdot_raw=gdot_raw, gdot_nz=gdot_nz, gdot_los=gdot_los,
                         nz_raw=nz_raw, cap_up=cap_up, cap_dn=cap_dn,
                         gdot_meas=gm,
                         nz_meas=(cg + V * gm / G) if gm == gm else float("nan")))
    return dict(tag="%s-U%d" % (run, sysid), recs=recs,
                t_dive=tq[-1] - tq[0], n=len(recs))


def deg(x):
    return math.degrees(x)


def q(v, p):
    v = sorted(v)
    if not v:
        return float("nan")
    i = min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))
    return v[i]


def block(name, rs):
    if not rs:
        print("\n== %s: no data" % name)
        return None
    gm = [r["gdot_meas"] for r in rs]
    raw = [r["gdot_raw"] for r in rs]
    nzc = [r["gdot_nz"] for r in rs]
    los = [r["gdot_los"] for r in rs]
    capu = [r["cap_up"] for r in rs]
    capd = [r["cap_dn"] for r in rs]
    occ, ratio_raw, ratio_los = [], [], []
    A = B = C = 0
    for r in rs:
        c = r["cap_up"] if r["gdot_meas"] >= 0 else r["cap_dn"]
        o = abs(r["gdot_meas"]) / max(1e-9, abs(c))
        occ.append(o)
        c1 = r["cap_up"] if r["gdot_raw"] >= 0 else r["cap_dn"]
        ratio_raw.append(abs(r["gdot_raw"]) / max(1e-9, abs(c1)))
        c2 = r["cap_up"] if r["gdot_los"] >= 0 else r["cap_dn"]
        ratio_los.append(abs(r["gdot_los"]) / max(1e-9, abs(c2)))
        if o >= 0.90:
            A += 1
        elif abs(r["gdot_raw"]) > abs(r["gdot_meas"]) + 1e-12:
            B += 1
        else:
            C += 1
    tot = len(rs)
    am = [abs(x) for x in gm]
    print("\n== %s   n=%d" % (name, tot))
    print("  meas gdot (deg/s)   med %+7.2f  p05 %+7.2f  p95 %+7.2f  |.|med %6.2f  |.|p90 %6.2f  |.|max %6.2f"
          % (deg(st.median(gm)), deg(q(gm, .05)), deg(q(gm, .95)),
             deg(st.median(am)), deg(q(am, .90)), deg(max(am))))
    for nm, dd in (("dem PRE-clamp ", raw), ("dem POST-clamp", nzc), ("dem POST-LOS  ", los)):
        ad = [abs(x) for x in dd]
        print("  %s |.|med %8.2f  p90 %9.2f  max %10.2f"
              % (nm, deg(st.median(ad)), deg(q(ad, .90)), deg(max(ad))))
    print("  ceiling (deg/s)     up med %+6.2f   dn med %+6.2f   up p05 %+6.2f  dn p95 %+6.2f"
          % (deg(st.median(capu)), deg(st.median(capd)),
             deg(q(capu, .05)), deg(q(capd, .95))))
    print("  * OCCUPANCY |meas|/ceiling  med %.3f  p90 %.3f  >=0.9 %.1f%%  >=1.0 %.1f%%"
          % (st.median(occ), q(occ, .90),
             sum(1 for x in occ if x >= 0.9) / tot * 100,
             sum(1 for x in occ if x >= 1.0) / tot * 100))
    print("  * DEMAND/ceiling (pre-clamp) med %.2f  p90 %.2f  max %.1f  frac>1 %.1f%%"
          % (st.median(ratio_raw), q(ratio_raw, .90), max(ratio_raw),
             sum(1 for x in ratio_raw if x > 1) / tot * 100))
    print("    DEMAND/ceiling (post-LOS) med %.2f  p90 %.2f"
          % (st.median(ratio_los), q(ratio_los, .90)))
    print("  * CLASS  A(>=90%% ceiling)=%5.1f%%   B(tracking deficit)=%5.1f%%   C(demand<meas)=%5.1f%%"
          % (A / tot * 100, B / tot * 100, C / tot * 100))
    for nm, dd in (("pre-clamp", raw), ("post-clamp", nzc), ("post-LOS", los)):
        sxy = sum(a * b for a, b in zip(dd, gm))
        sxx = sum(a * a for a in dd)
        kk = sxy / sxx if sxx > 0 else float("nan")
        try:
            cc = st.correlation(dd, gm)
        except Exception:
            cc = float("nan")
        print("     gain(meas/dem %-11s) = %7.3f   corr = %+.3f" % (nm, kk, cc))
    return dict(A=A / tot, B=B / tot, C=C / tot, n=tot)


def main():
    acs = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            o = per_ac(run, sysid, rows, tlat, tlon)
            if o:
                acs.append(o)
    allr = [r for a in acs for r in a["recs"] if r["gdot_meas"] == r["gdot_meas"]]
    print("aircraft %d   usable ticks %d / total %d   (HALF=%d, span %.2f-%.2f s)"
          % (len(acs), len(allr), sum(a["n"] for a in acs), HALF, SPAN_LO, SPAN_HI))

    block("ALL (32 aircraft pooled)", allr)
    for lo, hi, nm in ((0.0, 1 / 3, "EARLY 0-33%"), (1 / 3, 2 / 3, "MID 33-67%"),
                       (2 / 3, 1.01, "LATE 67-100%")):
        block(nm, [r for r in allr if lo <= r["frac"] < hi])
    block("LAST 3 s", [r for a in acs for r in a["recs"]
                       if r["gdot_meas"] == r["gdot_meas"]
                       and (a["recs"][-1]["t"] - r["t"]) <= 3.0])

    print("\n== per-aircraft (occ med / dem-pre-clamp per ceiling med / A,B,C %)")
    for a in acs:
        rs = [r for r in a["recs"] if r["gdot_meas"] == r["gdot_meas"]]
        if not rs:
            continue
        occ, rr, A, B, C = [], [], 0, 0, 0
        for r in rs:
            c = r["cap_up"] if r["gdot_meas"] >= 0 else r["cap_dn"]
            o = abs(r["gdot_meas"]) / max(1e-9, abs(c))
            occ.append(o)
            c2 = r["cap_up"] if r["gdot_raw"] >= 0 else r["cap_dn"]
            rr.append(abs(r["gdot_raw"]) / max(1e-9, abs(c2)))
            if o >= 0.9:
                A += 1
            elif abs(r["gdot_raw"]) > abs(r["gdot_meas"]):
                B += 1
            else:
                C += 1
        t = len(rs)
        print("  %10s  n=%4d  T=%5.1fs  occ %.3f  dem/cap %7.2f  A %5.1f  B %5.1f  C %5.1f"
              % (a["tag"], t, a["t_dive"], st.median(occ), st.median(rr),
                 A / t * 100, B / t * 100, C / t * 100))


if __name__ == "__main__":
    main()
