"""C2/C6 evidence: R_eff(W) per class, and the wind ceiling imposed by the gust rule."""
from __future__ import annotations
import json, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from hv_dccpp.platform import PLATFORM_PIONEER as PIONEER_PNP, PLATFORM_HERO as HERO_PNP  # type: ignore  # noqa: E402

out = {}
for name, spec in (("A_Pioneer", PIONEER_PNP), ("B_HERO", HERO_PNP)):
    d = {
        "cruise_airspeed_mps": spec.cruise_airspeed_mps,
        "turn_radius_planning_m": spec.turn_radius_planning_m,
        "turn_radius_theoretical_m": spec.turn_radius_theoretical_m,
        "w_star_mps": round(spec.w_star(), 3),
        "max_bank_angle_deg": spec.max_bank_angle_deg,
        "crosswind_site_max_mps": list(getattr(spec, "crosswind_site_max_mps", ()) or ()),
        "gust_site_max_mps": list(spec.gust_site_max_mps),
        "adapter_gust_rule": "gust = 1.3 * wind_speed (hv_dccpp/integration/aeroplan_adapter.py:961)",
        "max_wind_before_gust_NOGO_mps": round(min(spec.gust_site_max_mps) / 1.3, 3),
        "r_eff_by_wind": {str(w): round(spec.r_eff(float(w)), 2)
                          for w in (0, 2, 4, 5, 6, 8, 10, 12, 15)},
    }
    for attr in dir(spec):
        if "gust" in attr.lower() or "crosswind" in attr.lower():
            try:
                d.setdefault("_wind_attrs", {})[attr] = getattr(spec, attr)
            except Exception:
                pass
    out[name] = d
# theoretical radius: r_eff(0) vs r_eff(W) -> solve where wind starts to bind
for name, spec in (("A_Pioneer", PIONEER_PNP), ("B_HERO", HERO_PNP)):
    w = 0.0
    while w < 40.0 and spec.r_eff(w) <= spec.r_eff(0.0) + 1e-9:
        w += 0.1
    out[name]["wind_where_r_eff_starts_growing_mps"] = round(w, 1) if w < 40.0 else None
print(json.dumps(out, ensure_ascii=False, indent=1, default=str))
