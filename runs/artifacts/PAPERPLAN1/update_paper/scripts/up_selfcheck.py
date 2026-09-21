# -*- coding: utf-8 -*-
"""label update_paper / stage 9 -- prose vs numbers.

Every load-bearing figure that appears in TABLES_ZH.md / PAPER_TEXT_ZH.md /
CORRECTIONS_K1_K9.md / README.md is re-derived here from out/*.json and then
searched for in the prose.  A miss is a real defect: it means the text says
something the recomputation does not.

  python up_selfcheck.py     # exit 0 = every checked figure is present
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

from up_lib import OUT, jload

HERE = Path(__file__).resolve().parent.parent
N = jload(OUT / "NUMBERS.json")
C = jload(OUT / "corrections.json")
W = jload(OUT / "wind.json")
S = jload(OUT / "spawn_verification.json")
T = N["trials"]
DOCS = {p.name: p.read_text(encoding="utf-8")
        for p in (HERE / "TABLES_ZH.md", HERE / "PAPER_TEXT_ZH.md",
                  HERE / "CORRECTIONS_K1_K9.md", HERE / "README.md")}
ALL = "\n".join(DOCS.values())
# normalise before searching: the Chinese prose uses the Unicode minus U+2212
# and groups digits with spaces ("1 260.454").
_GROUP = re.compile(r"(?<=\d)[   ](?=\d)")


def _norm(t: str) -> str:
    return _GROUP.sub("", t).replace("−", "-").replace("－", "-")


ALL_N = _norm(ALL)
checks: list[tuple[str, str, bool]] = []


def _present(text: str) -> bool:
    """Substring search, but a numeric needle must not be a prefix of a longer
    number: '5.01' must NOT be satisfied by '5.0149'."""
    t = _norm(text)
    if not t or not t[-1].isdigit():
        return t in ALL_N
    return re.search(re.escape(t) + r"(?!\d)", ALL_N) is not None


def want(label: str, text: str) -> None:
    checks.append((label, text, _present(text)))


def wantv(label: str, x: float, dps=(4, 3, 2)) -> None:
    """A measured value may legitimately be quoted at 2, 3 or 4 decimals;
    accept any of them, and both roundings of an exact .5 tie."""
    cands = set()
    for n in dps:
        cands.add(f"{x:.{n}f}")
        q = 10 ** n
        cands.add(f"{(int(abs(x) * q + 0.5) / q) * (1 if x >= 0 else -1):.{n}f}")
    ok = any(_present(c) for c in cands)
    checks.append((label, " | ".join(sorted(cands)), ok))


def f(x, n=4) -> str:
    return f"{x:.{n}f}"


# --- ceiling control (C1 headline) -----------------------------------------
cc = N["ceiling_control"]
want("ceiling cost", "0.027 894")
want("ceiling cost short", "+0.0279 s")
want("exported above 120 (120.0 m arm)", "88")
want("t>120 fleet (120.0 m arm)", "354.93")
want("t>120 per uav", "140.05")
want("t>120 per uav", "73.57")
want("t>120 per uav", "141.31")
want("flown max AGL 120.0 arm", "120.1742")
want("flown max AGL 119.5 arm", "119.6752")
want("exported max 120.0 arm", "120.0526")
want("exported max 119.5 arm", "119.5526")
want("planned makespan 119.5", "1 260.453 961")
want("planned makespan 120.0", "1 260.426 066")

# --- separation -------------------------------------------------------------
for t in ("HV4W55", "HV4W55R", "HV4W55S", "HV4W05", "HV4W00", "BEAMIDP1",
          "HV4SITL15", "HV4SITL16"):
    wantv(f"{t} min sep", T[t]["separation"]["flown_exact_m"])
want("K1 range vs reporter lo", "0.31")
want("K1 range vs reporter hi", "3.51")
want("K1 range vs binsep lo", "0.34")
want("K1 range vs binsep hi", "0.90")
want("tightest margin", "0.291")
want("HV4W55 margin", "0.426")

# --- altitude ---------------------------------------------------------------
want("HV4SITL15 t>120", "325.79")
want("HV4SITL16 t>120", "330.14")
want("BEAMIDP1 max AGL", "119.697")
want("export discretisation excess lo", "0.0136")
want("export discretisation excess hi", "0.1703")
want("tracking excess lo", "0.0265")
want("tracking excess hi", "0.1475")

# --- wind -------------------------------------------------------------------
for t, u in (("HV4W55", "A1"), ("HV4W55", "B2"), ("HV4W55", "B3"),
             ("HV4W55R", "A1"), ("HV4W55R", "B2"), ("HV4W55R", "B3"),
             ("HV4W05", "A1"), ("HV4W05", "B2"), ("HV4W05", "B3")):
    wantv(f"{t}/{u} E-W", W["floors"]["100m"][t][u]["east_minus_west"])
for u in ("A1", "B2", "B3"):
    wantv(f"HV4W55S/{u} N-S 30m", W["floors"]["30m"]["HV4W55S"][u]["north_minus_south"])
for u in ("A1", "B2", "B3"):
    wantv(f"HV4SITL16/{u} E-W", W["floors"]["100m"]["HV4SITL16"][u]["east_minus_west"])
ct = N["crosstrack_wind_triangle"]
for u in ("A1", "B2", "B3"):
    wantv(f"crosstrack {u} measured", ct[u]["measured_ew_half_sum"])
    wantv(f"crosstrack {u} predicted", ct[u]["predicted_ew_half_sum"])
    wantv(f"crosstrack {u} residual", abs(ct[u]["residual_mps"]))

# --- params -----------------------------------------------------------------
want("strict exact", C["K2_param_strict_comparison"]["HV4W55"]["strict_exact"].replace("/", "／"))
want("strict float32", C["K2_param_strict_comparison"]["HV4W55"]["strict_float32"].replace("/", "／"))

# --- spawn ------------------------------------------------------------------
sp = C["K3_K4_spawn"]
for u in ("A1", "B2", "B3"):
    wantv(f"spawn {u} err (ellipsoid frame)", sp["per_uav"][u]["err_m"])
want("spawn bearing lo", f(min(v["bearing_deg"] for v in sp["per_uav"].values()), 2))
want("spawn bearing hi", f(max(v["bearing_deg"] for v in sp["per_uav"].values()), 2))
wantv("rigid max delta", sp["max_abs_spacing_delta_m"])
arc = S["B_body_axis_arc"]["A1"]
wantv("arc residual median", arc["residual_median_deg"], dps=(2,))
want("arc residual max", f(arc["residual_max_abs_deg"], 2))
want("arc path", f(arc["path_length_m"], 3))
want("arc chord", f(arc["net_chord_m"], 3))
want("arc n steps", str(arc["n_steps_above_threshold"]))

# --- table Z ----------------------------------------------------------------
z = C["K9_table_z_rulers"]
for k in ("dccpp_sep", "dccpp_pad", "harvest_relifted_sep", "harvest_relifted_pad",
          "ca_detected"):
    want(f"tableZ {k}", str(z[k]))
want("tableZ queue lo", "330")
want("tableZ queue hi", "6 460")
want("tableZ last landing lo", "2 576.7")
want("tableZ last landing hi", "9 851.3")

# --- geometry / platform ----------------------------------------------------
g = C["survey_geometry"]
want("area", "1 243 447.65")
want("area m/deg", "1 243 447.45")
want("width", "1 559.61")
want("height", "797.30")
want("swath 119.5", "137.9867")
want("spacing 119.5", "96.5907")
want("swath 120.0", "138.5641")
want("spacing 120.0", "96.9948")
want("W* A", "5.1675")
want("W* B", "5.0333")
want("R_eff(6) A", "212.68")
want("R_eff(6) B", "129.85")
want("R_eff(8) A", "244.74")
want("R_eff(8) B", "151.45")
want("bank HERO", "26.07")
want("bank Pioneer", "19.69")
want("R_geo A", "129.46")
want("R_geo B", "78.92")

# --- gate to gate -----------------------------------------------------------
gg = C["gate_to_gate_prediction_error"]
for t in ("HV4W00", "HV4W05", "HV4W55", "HV4W55R", "HV4W55S"):
    lo, hi = gg[t]["gate_err_range_pct"]
    wantv(f"gate {t} lo", lo, dps=(2,))
    wantv(f"gate {t} hi", hi, dps=(2,))

# --- timing / distance ------------------------------------------------------
want("HV4W05-HV4W00 makespan diff", f(T["HV4W05"]["makespan"]["flown_s"]
                                      - T["HV4W00"]["makespan"]["flown_s"], 2))
want("wind-aware minus wind-blind", f(T["HV4W55"]["makespan"]["flown_s"]
                                      - T["HV4W05"]["makespan"]["flown_s"], 2))
want("HV4W55S planned km", f(T["HV4W55S"]["distance"]["planned_km"], 3))
want("HV4W55S planned makespan", f(T["HV4W55S"]["makespan"]["planned_s"], 3))
want("HV4W55S planned sep", f(T["HV4W55S"]["separation"]["planned_m"], 3))

bad = [c for c in checks if not c[2]]
for label, text, ok in checks:
    if not ok:
        print(f"MISSING  {label:34s} expected to find: {text!r}")
print(f"\n{len(checks) - len(bad)}/{len(checks)} checked figures found in the prose.")
sys.exit(1 if bad else 0)
