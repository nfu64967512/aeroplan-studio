#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R1 diagnostics (NOT part of the pass/fail verdict).

  A. how far the NEW pitch command sits from the current one
  B. the terminal segment (t_go < 5 s) -- the part that owns the 0.80 m
  C. induced-drag cost of the n_z change  ->  touchdown speed (acceptance #4)
  D. why gate (3) came out the way it did

Read-only.
"""
from __future__ import annotations

import math
import statistics as st

import r1_replay as RP
import r1_gates as GT

G = RP.G
DEG = math.degrees

# ── airframe (predictor.py, calibrated n=1617) ───────────────────────────
import predictor as P                                            # noqa: E402
MASS, S, SPAN, E_OSW, CD0 = P.MASS_KG, P.S_REF_M2, P.SPAN_M, P.OSWALD_E, P.CD0
AR = SPAN * SPAN / S
W = MASS * G
K_IND = 1.0 / (math.pi * AR * E_OSW)


def q(v, p):
    return RP.q(v, p)


def stat(name, v, unit=""):
    a = [abs(x) for x in v]
    print("  %-34s med %8.3f  p90 %9.3f  max %10.3f %s"
          % (name, st.median(a), q(a, .90), max(a), unit))


def main():
    acs, out, aero, late, meas = GT.main()
    n = len(out["OFF"])
    off, new, clamp, cl = out["OFF"], out["NEW"], out["CLAMP"], out["CL"]

    print("\n\n" + "#" * 78)
    print("# DIAGNOSTICS (not part of the verdict)")
    print("#" * 78)

    # ── A. pitch command difference ──────────────────────────────────────
    print("\n== A. pitch command: NEW vs current (deg)")
    dp = [DEG(new[k]["pitch"] - off[k]["pitch"]) for k in range(n)]
    stat("|pitch_NEW - pitch_OFF|", dp, "deg")
    print("     signed: p05 %+.2f  med %+.2f  p95 %+.2f  (+ = NEW commands NOSE UP)"
          % (q(dp, .05), q(dp, .5), q(dp, .95)))
    for nm, cfg in (("clamp-order fix only", clamp), ("closed-loop only", cl)):
        d = [DEG(cfg[k]["pitch"] - off[k]["pitch"]) for k in range(n)]
        stat("  %s" % nm, d, "deg")
    # decomposition: the exact term vs the alpha term
    ex = [DEG(new[k]["gam"] - (off[k]["gam"] + 0.0)) for k in range(n)]  # placeholder
    t1, t2 = [], []
    for k in range(n):
        r_o, r_n = off[k], new[k]
        cosr = max(math.cos(r_n["roll"]), 0.5)
        # pitch_OFF = gam_cmd_OFF + a_est/cosr ; pitch_NEW = gam + a_cmd/cosr
        gcmd_off = r_o["gam"] + (r_o["cmd_nz"] - math.cos(r_o["gam"])) * G / r_o["V"]
        t1.append(DEG(r_n["gam"] - gcmd_off))
        t2.append(DEG((r_n["alpha_cmd"] - r_o["alpha_est"]) / cosr))
    stat("  term1 (gam - gam_cmd_old)", t1, "deg")
    stat("  term2 (a_cmd - a_est)/cos", t2, "deg")
    print("     ==> the split is ~%.0f : %.0f  (term1 : term2, by median |.|)"
          % (st.median([abs(x) for x in t1]), st.median([abs(x) for x in t2])))
    hard = sum(1 for x in dp if abs(x) > 30.0)
    print("     ticks with |dpitch| > 30 deg : %d  (%.2f %%)" % (hard, hard / n * 100))
    # pitch clip occupancy
    for nm, cfg in (("OFF", off), ("NEW", new)):
        lo = sum(1 for r in cfg if r["pitch"] <= math.radians(-80.0) + 1e-12)
        hi = sum(1 for r in cfg if r["pitch"] >= math.radians(45.0) - 1e-12)
        print("     %-4s pitch clipped at -80 deg: %5.2f %%   at +45 deg: %5.2f %%"
              % (nm, lo / n * 100, hi / n * 100))

    # ── B. terminal segment ──────────────────────────────────────────────
    print("\n== B. terminal segment (t_go = R/V < 5 s) -- owns the 0.80 m")
    term = [k for k in range(n) if off[k]["R"] / max(off[k]["V"], 1e-3) < 5.0]
    frz = [k for k in term if off[k]["R"] < 100.0
           or off[k]["R"] / max(off[k]["V"], 1e-3) <= 1.5]
    print("  ticks t_go<5s : %d (%.2f %%)   of those already FROZEN by _pn_step"
          " (t_go<=1.5s or R<=100m) : %d (%.1f %%)"
          % (len(term), len(term) / n * 100, len(frz),
             len(frz) / max(1, len(term)) * 100))
    live = [k for k in term if k not in set(frz)]
    if live:
        d = [DEG(new[k]["pitch"] - off[k]["pitch"]) for k in live]
        stat("  |dpitch| on live terminal ticks", d, "deg")
        print("     signed p05 %+.2f  med %+.2f  p95 %+.2f" % (q(d, .05), q(d, .5), q(d, .95)))
        for key in ("req_nz", "cmd_nz"):
            print("     %-8s OFF med %+7.3f   NEW med %+7.3f"
                  % (key, st.median([off[k][key] for k in live]),
                     st.median([new[k][key] for k in live])))
        dnz = [new[k]["cmd_nz"] - off[k]["cmd_nz"] for k in live]
        stat("  |d cmd_nz| on live terminal", dnz, "g")

    # ── C. induced-drag / speed cost ─────────────────────────────────────
    print("")
    print("== C. induced drag cost  (acceptance #4: touchdown total speed >= 100 m/s)")
    print("  airframe: m=%.1f kg  S=%.3f m2  b=%.2f m  AR=%.3f  e=%.2f  CD0=%.5f"
          % (MASS, S, SPAN, AR, E_OSW, CD0))
    print("  ⚠️ the honest comparison is ACHIEVED-vs-ACHIEVED:")
    print("     NOW  = what the aircraft actually pulled (nz_meas, central diff)")
    print("     PATHA= what it would pull if the loop really closed (cmd_nz of NEW)")
    print("     Using OFF's cmd_nz for 'now' would be wrong -- the whole finding is")
    print("     that the current law commands things the aircraft never delivers.")
    per_ac, prev_tag, prev_t = {}, None, None
    tot = {"NOW": 0.0, "PATHA": 0.0}
    nsk = 0
    for k in range(n):
        r_o, r_n = off[k], new[k]
        tag, t = r_o["tag"], r_o["t"]
        dt = 0.05 if (tag != prev_tag or prev_t is None) else max(0.0, min(0.5, t - prev_t))
        prev_tag, prev_t = tag, t
        if r_o["nz_meas"] != r_o["nz_meas"]:
            nsk += 1
            continue
        qq_ = r_o["q_pa"]
        for cfg, nz in (("NOW", r_o["nz_meas"]), ("PATHA", r_n["cmd_nz"])):
            cl_ = nz * W / max(qq_ * S, 1e-6)
            di = K_IND * cl_ * cl_ * qq_ * S
            tot[cfg] += di * r_o["V"] * dt
            per_ac.setdefault(tag, {}).setdefault(cfg, 0.0)
            per_ac[tag][cfg] += di * r_o["V"] * dt
    de = tot["PATHA"] - tot["NOW"]
    print("  pooled induced-drag work, 32 dives (%d ticks, %d skipped for no nz_meas):"
          % (n - nsk, nsk))
    print("     NOW %.4f MJ   PATH-A %.4f MJ   delta %+.4f MJ (%+.1f %%)"
          % (tot["NOW"] / 1e6, tot["PATHA"] / 1e6, de / 1e6,
             de / max(tot["NOW"], 1e-9) * 100))
    pen = []
    for a in acs:
        tag = a["tag"]
        if tag not in per_ac:
            continue
        d = per_ac[tag]["PATHA"] - per_ac[tag]["NOW"]
        vt = [r["V"] for r in off if r["tag"] == tag][-1]
        pen.append((tag, vt, math.sqrt(max(0.0, vt * vt - 2.0 * d / MASS)) - vt, d))
    dv = [x[2] for x in pen]
    print("  per-sortie touchdown TOTAL-speed change if the whole delta comes out of V:")
    print("     med %+.2f m/s   p10 %+.2f   p90 %+.2f   worst %+.2f m/s"
          % (st.median(dv), q(dv, .10), q(dv, .90), min(dv)))
    print("     end speeds now: min %.1f  med %.1f  max %.1f m/s"
          % (min(x[1] for x in pen), st.median([x[1] for x in pen]),
             max(x[1] for x in pen)))
    for tag, vt, d, e in sorted(pen, key=lambda x: x[2])[:5]:
        print("       %-10s V_end %6.1f -> %+.2f m/s   (dE %+.4f MJ)"
              % (tag, vt, d, e / 1e6))
    print("  ⚠️ this ignores CD0 and the gravity term -- it is only the INDUCED")
    print("     delta, i.e. a lower bound on the cost of pulling harder.")
    # what the |n_z| actually does, per third
    print("  |n_z| per third   (now achieved -> path-A commanded):")
    for lo, hi, nm in ((0, 1 / 3, "EARLY"), (1 / 3, 2 / 3, "MID"), (2 / 3, 1.01, "LATE")):
        ks = [k for k in range(n) if lo <= off[k]["frac"] < hi
              and off[k]["nz_meas"] == off[k]["nz_meas"]]
        print("     %-6s |nz| med  %.3f -> %.3f      signed med  %+.3f -> %+.3f"
              % (nm, st.median([abs(off[k]["nz_meas"]) for k in ks]),
                 st.median([abs(new[k]["cmd_nz"]) for k in ks]),
                 st.median([off[k]["nz_meas"] for k in ks]),
                 st.median([new[k]["cmd_nz"] for k in ks])))

    # ── D. why gate 3 landed where it did ────────────────────────────────
    print("\n== D. gate (3) mechanism")
    print("  path A changes the ATTITUDE that carries a demand; it does not")
    print("  change the demand.  req_nz is pure PN geometry:")
    for nm, ks in (("ALL", list(range(n))), ("LATE third", late)):
        a = [off[k]["req_nz"] for k in ks]
        b = [new[k]["req_nz"] for k in ks]
        print("     %-11s req_nz  OFF med %+.4f   NEW med %+.4f   identical ticks %d/%d"
              % (nm, st.median(a), st.median(b),
                 sum(1 for k in ks if off[k]["req_nz"] == new[k]["req_nz"]), len(ks)))
    lm = [off[k]["nz_meas"] for k in late if off[k]["nz_meas"] == off[k]["nz_meas"]]
    lc = [off[k]["cmd_nz"] for k in late]
    print("  LATE third: commanded %.3f  vs  achieved %.3f  (deficit %.3f g)"
          % (st.median(lc), st.median(lm), st.median(lc) - st.median(lm)))
    ks = [k for k in late if off[k]["nz_meas"] == off[k]["nz_meas"]]
    try:
        c = st.correlation([off[k]["cmd_nz"] for k in ks], [off[k]["nz_meas"] for k in ks])
    except Exception:                                            # noqa: BLE001
        c = float("nan")
    print("  corr(cmd_nz, achieved) on LATE third = %+.3f  (n=%d)" % (c, len(ks)))
    over = sum(1 for k in late if off[k]["req_nz"] > 1.5)
    print("  LATE ticks where PN geometry even ASKS for > 1.5 g: %d / %d = %.1f %%"
          % (over, len(late), over / len(late) * 100))
    nzc = sum(1 for k in late if new[k]["cmd_nz"] >= 3.0 - 1e-9)
    print("  LATE ticks pinned at the nz_max_g=3.0 ceiling: %d (%.1f %%)"
          % (nzc, nzc / len(late) * 100))
    nzf = sum(1 for k in late if new[k]["cmd_nz"] <= -1.0 + 1e-9)
    print("  LATE ticks pinned at pn_nz_min=-1.0        : %d (%.1f %%)"
          % (nzf, nzf / len(late) * 100))
    print("  EARLY third pinned at pn_nz_min=-1.0       : %d / %d (%.1f %%)"
          % (sum(1 for k in range(n) if off[k]["frac"] < 1 / 3
                 and new[k]["cmd_nz"] <= -1.0 + 1e-9),
             sum(1 for k in range(n) if off[k]["frac"] < 1 / 3),
             sum(1 for k in range(n) if off[k]["frac"] < 1 / 3
                 and new[k]["cmd_nz"] <= -1.0 + 1e-9)
             / max(1, sum(1 for k in range(n) if off[k]["frac"] < 1 / 3)) * 100))



    # ── E. what the clamp-after-floor does to the COMMANDED flight path ──
    print("")
    print("== E. the LOS floor vs the clamp-order fix (the load-bearing question)")
    print("  commanded gamma (deg) implied by the sent pitch, per third:")
    print("     %-6s %9s %9s %9s %9s %9s"
          % ("third", "gam_meas", "gam_los", "OFF_cmd", "CLAMP", "NEW"))
    for lo, hi, nm in ((0, 1 / 3, "EARLY"), (1 / 3, 2 / 3, "MID"), (2 / 3, 1.01, "LATE")):
        ks = [k for k in range(n) if lo <= off[k]["frac"] < hi]
        def gc(cfg, k):
            r = cfg[k]
            return DEG(r["gam"] + (r["cmd_nz"] - math.cos(r["gam"])) * G / r["V"])
        gl = [DEG(-math.atan2(off[k]["h"], max(1e-3,
              math.sqrt(max(0.0, off[k]["R"] ** 2 - off[k]["h"] ** 2))))) for k in ks]
        print("     %-6s %+9.2f %+9.2f %+9.2f %+9.2f %+9.2f"
              % (nm, DEG(st.median([off[k]["gam"] for k in ks])), st.median(gl),
                 st.median([gc(off, k) for k in ks]),
                 st.median([gc(clamp, k) for k in ks]),
                 st.median([gc(new, k) for k in ks])))
    bit = sum(1 for k in range(n) if clamp[k]["pitch"] != off[k]["pitch"])
    print("  ticks where the clamp-order fix CHANGES the sent pitch: %d (%.2f %%)"
          % (bit, bit / n * 100))
    d = [DEG(clamp[k]["pitch"] - off[k]["pitch"]) for k in range(n)
         if clamp[k]["pitch"] != off[k]["pitch"]]
    print("     on those ticks: med %+.2f  p90 %+.2f  max %+.2f deg (+ = NOSE UP)"
          % (st.median(d), q(d, .90), max(d)))
    print("  OFF cmd_nz per third (the phantom demand the floor was writing):")
    for lo, hi, nm in ((0, 1 / 3, "EARLY"), (1 / 3, 2 / 3, "MID"), (2 / 3, 1.01, "LATE")):
        ks = [k for k in range(n) if lo <= off[k]["frac"] < hi]
        v = [off[k]["cmd_nz"] for k in ks]
        print("     %-6s med %+8.3f   p05 %+9.3f   min %+10.3f" % (nm, st.median(v), q(v, .05), min(v)))


if __name__ == "__main__":
    main()
