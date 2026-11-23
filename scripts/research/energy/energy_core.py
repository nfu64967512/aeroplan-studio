#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""energy_core.py -- 32 sortie per-tick energy / glide-ratio extraction.

Read-only over runs/artifacts/{RRD,RRE}{1..8}/tot_*.csv .
Segment convention copied verbatim from ditcg/truth32.py (already bit-verified):
  release  = first row with mode == "dive"
  touchdown= first row with agl <= 0 at or after release (NOT filtered by mode)
  V = gs / cos(gamma)          (gs is HORIZONTAL speed)
  z = agl                      (target is sea level, AMSL 0)
"""
from __future__ import annotations
import csv, math, os

G = 9.80665
EARTH = 111320.0
GAP_S = 0.3
BASE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "..", "..", "runs", "artifacts"))
RUNS = [f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]


def _mlon(lat): return EARTH * math.cos(math.radians(lat))


def _f(row, key, default=float("nan")):
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def read_run(run):
    p = os.path.join(BASE, run, f"tot_{run}.csv")
    with open(p, newline="", encoding="utf-8", errors="replace") as f:
        out = {}
        for row in csv.DictReader(f):
            out.setdefault(int(row["sysid"]), []).append(row)
    tlat = tlon = None
    with open(os.path.join(BASE, run, f"tot_{run}_geom.csv"), newline="",
              encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            if row["kind"] == "target":
                tlat, tlon = float(row["lat"]), float(row["lon"])
    return out, tlat, tlon


def segment(rows, tlat, tlon):
    """Return list of per-tick dicts over the dive segment, or None."""
    i_rel = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"), None)
    if i_rel is None:
        return None
    i_td = next((k for k in range(i_rel, len(rows)) if _f(rows[k], "agl") <= 0.0), None)
    if i_td is None:
        return None
    seg = []
    for k in range(i_rel, i_td + 1):
        r = rows[k]
        lat, lon = _f(r, "lat"), _f(r, "lon")
        dn = (tlat - lat) * EARTH
        de = (tlon - lon) * _mlon(lat)
        d = max(1e-3, math.hypot(dn, de))
        h = _f(r, "agl")
        gam = math.radians(_f(r, "gam_deg"))
        gs = _f(r, "gs")
        V = gs / max(math.cos(gam), 1e-6)
        seg.append(dict(
            k=k, t=_f(r, "t"), lat=lat, lon=lon, dn=dn, de=de,
            d=d, h=h, R=math.hypot(d, h), gam=gam, gs=gs, V=V,
            E=h + V * V / (2.0 * G),
            hdot=gs * math.tan(gam),
            secs=_f(r, "secs_to_target"), early=_f(r, "early_by_s"),
        ))
    return seg


def course(seg, k, hw=4):
    """Track course (rad from north) from position displacement only."""
    n = len(seg)
    lo, hi = max(0, k - hw), min(n - 1, k + hw)
    while lo < k and seg[lo + 1]["t"] - seg[lo]["t"] > GAP_S:
        lo += 1
    while hi > k and seg[hi]["t"] - seg[hi - 1]["t"] > GAP_S:
        hi -= 1
    if hi <= lo:
        lo, hi = max(0, k - 1), min(n - 1, k + 1)
    dn = (seg[hi]["lat"] - seg[lo]["lat"]) * EARTH
    de = (seg[hi]["lon"] - seg[lo]["lon"]) * _mlon(seg[lo]["lat"])
    return math.atan2(de, dn)


def load_all(runs=None):
    out = []
    for run in (runs or RUNS):
        by_sys, tlat, tlon = read_run(run)
        for sysid in sorted(by_sys):
            seg = segment(by_sys[sysid], tlat, tlon)
            if seg:
                out.append((f"{run}-U{sysid}", run, sysid, seg, tlat, tlon))
    return out
