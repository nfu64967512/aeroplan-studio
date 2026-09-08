"""preflight.py — 前置檢查 PRECHECK（契約 §3A；規格 §6.2 步 1–13、D7、D8、D11、D12）。

第 4 層模組：依賴 `context`（build_context）、`altitude`（site_check）、`decompose`（altitude_band、
d_turn、感測相容）、`costs`（v_air_star）、`mission`、`platform`、`wind`。

流程（規格 §6.2）
----------------
1. 必填鍵（D11／§5.7）：每構型 P_link,c（`c2.range_km[c]` 或 coverage polygon，其次 spec 覆寫槽）、
   `terrain_clearance_m`（Airspace → spec 槽）、備降點非空（`alternate_landing_sites_required`）、
   E_nom 可解析（Wh/mAh 給定但 nominal 兩層皆 None）→ 缺 → NO_GO 並列鍵名。**輸入層先檢查**，使所有缺鍵一次列出；
   之後才呼叫 `build_context`（其 MissionInputError 亦轉為 NO_GO）。
2. 氣象放飛（D7）：`WeatherThresholds.go_violations(obs, min_c W_go,c)`；陣風 > `BEAUFORT_UPPER_MPS[wind_resistance_public]`
   只警告（能力宣稱非門檻）。
3–4. 座標、地形、淨空（h_clr ≥ 3σ+30）、障礙燒入：由 `context.build_context` 完成，其警告全數併入。
5. 站點：`altitude.site_check`（坡度、淨空圓、轉換／CLIMB／DESCENT 走廊、盤旋碟形區、鏈路、側風／陣風）——
   home／回收點失敗 → 錯誤；備降點失敗 → 該構型不可用（警告），全部不可用且 required → 錯誤。
6. 機體：m ≤ mtow、酬載（A ≤ 5.0、B < 1.0；缺省推估並警告 `payload_estimated`）、E_use > 0、失速裕度、
   V_a,min(m) ≤ V_a,c；轉換段坡度 ≤ γ_tr,max（D37）。
7. 鏈路／半徑裁切（D38）：Z ∩ (P_link ⊖ d_turn) ∩ Disc(H, ρ_sym − d_turn) \\ ∪(NoFly ⊕ b_trk)，逐構型記錄
   可覆蓋比例與 UNCOVERABLE(link/radius/nofly)。
8. 高度帶（§4.6，`decompose.altitude_band`，胞多邊形層級）：Δh_band < 20 → 不可行；A 無豁免 → `A_needs_waiver`；
   相機 GSD 可達性 → `gsd_unattainable`。
9. 感測可行：κ(σ,c)（`decompose.class_can_serve`）∧ 高度帶 ∧ ∃θ 使 V_a* 存在（`costs.v_air_star`，72 個方向）；
   每區至少一構型（`both` 需 lidar 與 rgb 兩半皆有人承接），否則 UNCOVERABLE(sensor)（警告；**所有**區域皆不可覆蓋才 NO_GO）。
10. 時間窗（D22）：區域窗 ∩ 空域窗非空；粗估 makespan **下界**（作業面積／(s·V_g,max)／架數 + 起降固定時間）須 ≤ 窗長。
11. 未驗證航電（D8）：`transition_cmd()` 之 V_tr+ΔV、H_tc = hi；k_bm/k_as 取 cfg；D_use 取下緣——皆列於 `derived`。
12. 警告彙整：`provisional_params`、`reference_battery_assumed`、`battery_nominal_example`、`sensor_spec_default_example(cls)`、
    `airspace_ceiling_defaulted_120`、`endurance_range_inconsistent`（|R_ref/T_ref/V_a − 1| > 10%）、`overridden(...)`。
13. `FeasibilityReport(status, errors, warnings, per_area, per_uav, derived)`；`derived[cls]` 含
    R_eff、V_g_cap、h_op、w、s、P_c_ref、h_ceil_source、A_needs_waiver 等。

`derived` 之鍵只有機隊構型（契約 `derived[cls]`）；任務層數值（W_go、makespan 下界、h_clr…）複製於每一構型字典。
"""
from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np
import shapely
from shapely.geometry import Point

from .altitude import site_check
from .context import (
    MissionContext,
    DecomposeContext,
    apply_platform_overrides,
    base_specs_from_fleet,
    build_context,
)
from .costs import v_air_star
from .decompose import (
    AltitudeBand,
    altitude_band,
    class_can_serve,
    d_turn_m,
    end_extension_for,
    resolve_sensor,
)
from .mission import (
    FeasibilityReport,
    MissionInputError,
    MissionInputs,
    PlannerConfig,
    resolve_weather_go,
)
from .platform import BEAUFORT_UPPER_MPS, PlatformSpec
from .wind import ground_speed, r_eff_wind_speed

__all__ = [
    "preflight",
    "N_DIRECTION_SAMPLES",
    "ENDURANCE_RANGE_TOL",
    "EXAMPLE_BATTERY_NOMINAL_WH",
    "check_required_inputs",
    "check_weather",
    "check_sites",
    "check_airframes",
    "analyse_areas",
    "derive_by_class",
    "check_time_windows",
]

N_DIRECTION_SAMPLES: int = 72
"""步 9「∃θ 使 V_a* 存在」之方位取樣數（5° 間隔）。"""

ENDURANCE_RANGE_TOL: float = 0.10
"""`endurance_range_inconsistent`：|（R_ref/T_ref）/V_a − 1| > 10%（規格 §5.1）。"""

EXAMPLE_BATTERY_NOMINAL_WH: dict[str, float] = {"A": 1500.0, "B": 420.0}
"""規格 §16.2 示例標稱容量；當架次標稱恰為此值時亦警告 `battery_nominal_example`（規格 §5.7）。"""

_AREA_TOL_REL: float = 1e-6      # 裁切損失小於此比例視為 0
_TWO_PI: float = 2.0 * math.pi


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def _f(x: Any) -> float:
    """numpy → Python float（報告 JSON 可序列化）。"""
    return float(x)


def _dedupe(items: Sequence[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for s in items:
        if s not in seen:
            seen.add(s)
            out.append(s)
    return out


def _mod2pi(x: float) -> float:
    return float(x) % _TWO_PI


def _classes(inputs: MissionInputs) -> list[str]:
    return sorted(inputs.fleet_classes)


def _resolved_specs(inputs: MissionInputs) -> tuple[dict[str, PlatformSpec], list[str], list[str]]:
    """輸入層之（已覆寫）spec；覆寫失敗 → 退回基礎 spec 並回報錯誤。回傳 (specs, errors, warnings)。"""
    base, warnings = base_specs_from_fleet(inputs.fleet)
    try:
        specs, w = apply_platform_overrides(base, inputs.platform_overrides)
        warnings = warnings + w
        return specs, [], warnings
    except MissionInputError as e:
        return base, [f"platform_overrides: {e}"], warnings


# ---------------------------------------------------------------------------
# 步 1：必填鍵（D11／§5.7）
# ---------------------------------------------------------------------------

def check_required_inputs(inputs: MissionInputs, cfg: PlannerConfig,
                          specs: dict[str, PlatformSpec]) -> tuple[list[str], list[str]]:
    """規格 §6.2 步 1：缺鍵 → 錯誤（訊息含鍵名）。`cfg.enforce_link=False`（legacy）時略過 C2。"""
    errors: list[str] = []
    warnings: list[str] = []
    for cls in _classes(inputs):
        spec = specs[cls]
        if cfg.enforce_link and inputs.c2.range_km_for(cls, spec) is None and inputs.c2.coverage_for(cls) is None:
            errors.append(f"c2_link_unresolved({cls}): 缺鍵：c2.range_km[{cls}]/c2.coverage_polygon_wgs84[{cls}]/"
                          f"spec.c2_link_range_km")
    if inputs.airspace.terrain_clearance_m is None and all(
            specs[c].terrain_clearance_m is None for c in _classes(inputs)):
        errors.append("terrain_clearance_unresolved: 缺鍵：airspace.terrain_clearance_m/spec.terrain_clearance_m"
                      "（platform_overrides）")
    if not inputs.alternates:
        need = [c for c in _classes(inputs) if specs[c].alternate_landing_sites_required]
        if need:
            errors.append(f"alternates_missing({','.join(need)}): 缺鍵：alternates（alternate_landing_sites_required=true）")
    for u in inputs.fleet:
        spec = specs[u.cls]
        nominal = u.sortie.nominal_wh_resolved(spec)
        try:
            u.sortie.available_wh_resolved(nominal)
        except MissionInputError as e:
            errors.append(f"energy_unresolved({u.uav_id}): {e}")
    return errors, warnings


# ---------------------------------------------------------------------------
# 步 2：氣象放飛（D7、D8）
# ---------------------------------------------------------------------------

def check_weather(inputs: MissionInputs, cfg: PlannerConfig,
                  specs: dict[str, PlatformSpec]) -> tuple[list[str], list[str], dict[str, float]]:
    """規格 §6.2 步 2：`obs.mean_wind ≤ min_c W_go,c`、gust、能見度、降雨、溫度 → 否則錯誤；
    `gust > BEAUFORT_UPPER_MPS[wind_resistance_public]` → 警告。回傳 (errors, warnings, W_go per cls)。"""
    errors: list[str] = []
    warnings: list[str] = []
    th = inputs.weather_thresholds
    obs = inputs.weather_obs
    w_go = {c: _f(resolve_weather_go(specs[c], th)) for c in _classes(inputs)}
    w_go_min = min(w_go.values()) if w_go else 8.0
    errors += [f"weather_go({v})" for v in th.go_violations(obs, w_go_min)]
    for c in _classes(inputs):
        spec = specs[c]
        try:
            upper = spec.wind_limit_public_mps()
        except ValueError:
            continue
        if obs.gust_mps > upper + 1e-9:
            warnings.append(f"gust_above_public_wind_rating({c}: gust {obs.gust_mps:.1f} > Beaufort "
                            f"{spec.wind_resistance_public} upper {upper:.1f} m/s)")
    layer_max = max((float(l.speed_mps) for l in inputs.wind), default=0.0)
    if layer_max > w_go_min + 1e-9:
        warnings.append(f"wind_layer_above_go(max layer {layer_max:.1f} > W_go {w_go_min:.1f} m/s)")
    return errors, warnings, w_go


# ---------------------------------------------------------------------------
# 步 5：站點（規格 §11.8）
# ---------------------------------------------------------------------------

def check_sites(ctx: MissionContext) -> tuple[list[str], list[str], dict[str, list[str]]]:
    """每站點 × 構型 `altitude.site_check`。home／回收點／**任一機之起飛站點**失敗 → 錯誤；其餘備降點失敗 →
    `alternate_unusable`（警告）；required 且無可用備降點 → 錯誤 `no_usable_alternate(cls)`。
    回傳 (errors, warnings, usable_alternates[cls])。

    起飛站點納入 primary 之理由：`UAVInstance.start_site_id` 生效後，planner 真的從該站點起飛（走廊、走廊時隙、
    entry 皆以該站點為準），其站點檢查失敗屬硬錯誤，不能只降級為「備降點不可用」警告。"""
    errors: list[str] = []
    warnings: list[str] = []
    inputs = ctx.inputs
    classes = _classes(inputs)
    usable: dict[str, list[str]] = {c: [] for c in classes}
    obstacles = ctx.obstacle_pairs()
    # primary[site_id] = 「必須通過站點檢查」之構型集合：home／回收點 → 全構型；某機之起飛站點 → 只有該機之構型
    # （逐機起飛站生效後，起飛站失敗屬硬錯誤；但不能因為別的構型用不了該站就整個任務 NO_GO）。
    primary: dict[str, set[str]] = {inputs.home.site_id: set(classes), ctx.recovery_site_id: set(classes)}
    for uid, sid in (getattr(ctx, "start_sites", {}) or {}).items():
        try:
            c_u = inputs.uav_by_id(uid).cls
        except Exception:                                   # pragma: no cover - 手工情境
            continue
        primary.setdefault(sid, set()).add(c_u)
    alt_ids = {s.site_id for s in inputs.alternates}
    for sid in ctx.sites_enu:
        for c in classes:
            issues = site_check(sid, ctx.specs[c], ctx, obstacles)
            must = c in primary.get(sid, ())
            if issues:
                if must:
                    errors.append(f"site_check({sid}, {c}): " + "; ".join(issues))
                else:
                    warnings.append(f"alternate_unusable({sid}, {c}): " + "; ".join(issues))
            elif sid in alt_ids or not must:
                # 通過檢查之備降點一律列為可用（起飛站同時是備降點時亦然）
                usable[c].append(sid)
    for c in classes:
        if ctx.specs[c].alternate_landing_sites_required and inputs.alternates and not usable[c]:
            errors.append(f"no_usable_alternate({c}): 所有備降點皆未通過站點檢查")
    return errors, warnings, usable


# ---------------------------------------------------------------------------
# 步 6：機體（規格 §6.2 步 6、§3.2 D21、§5.3）
# ---------------------------------------------------------------------------

def check_airframes(ctx: MissionContext) -> tuple[dict[str, dict], list[str], list[str]]:
    """逐機：m ≤ mtow、酬載（A ≤ payload_max；B < payload_max，缺省推估 → `payload_estimated`）、E_use > 0、
    失速裕度 V_a,min ≥ 1.2 V_s(m)、V_a,min(m) ≤ V_a,c；m > m_ref → 警告。回傳 (per_uav, errors, warnings)。"""
    errors: list[str] = []
    warnings: list[str] = []
    per_uav: dict[str, dict] = {}
    inputs = ctx.inputs
    for u in inputs.fleet:
        uid = u.uav_id
        spec = ctx.specs[u.cls]
        model = ctx.models[uid]
        s = u.sortie
        auw = _f(s.auw_kg)
        issues: list[str] = []
        if auw > spec.mtow_kg + 1e-9:
            issues.append(f"auw_exceeds_mtow({auw:.2f} > {spec.mtow_kg:.2f} kg)")
        if auw > spec.reference_auw_kg + 1e-9:
            warnings.append(f"auw_above_reference({uid}: {auw:.2f} > {spec.reference_auw_kg:.2f} kg；爬升角依 D21 折減)")
        payload, estimated = s.payload_kg_resolved(spec)
        if estimated:
            warnings.append(f"payload_estimated({uid}: {payload:.2f} kg = auw − m_empty {spec.empty_mass_kg():.2f})")
        pmax = _f(spec.payload_max_kg)
        if u.cls == "B":
            payload_ok = payload < pmax - 1e-9
            rule = f"< {pmax:.2f}"
        else:
            payload_ok = payload <= pmax + 1e-9
            rule = f"≤ {pmax:.2f}"
        if not payload_ok:
            issues.append(f"payload_exceeds_max({payload:.2f} kg，須 {rule} kg)")
        e_use = _f(model.e_usable_wh())
        if not e_use > 0.0:
            issues.append(f"energy_usable_nonpositive(E_use={e_use:.1f})")
        v_s = spec.stall_speed(auw)
        stall_ok = spec.stall_margin_ok(auw)
        if not stall_ok:
            issues.append(f"stall_margin(1.2·V_s(m)={1.2 * v_s:.2f} > V_a,min={spec.v_min_mission():.2f})")
        v_min = max(spec.v_min_mission(), 1.2 * v_s)
        if v_min > spec.cruise_airspeed_mps + 1e-9:
            issues.append(f"v_min_exceeds_cruise(V_a,min(m)={v_min:.2f} > V_a,c={spec.cruise_airspeed_mps:.2f})")
        errors += [f"uav({uid}): {x}" for x in issues]
        nominal = model.nominal_wh
        if nominal is not None and abs(nominal - EXAMPLE_BATTERY_NOMINAL_WH.get(u.cls, math.nan)) < 1e-9:
            warnings.append(f"battery_nominal_example({uid})")
        p_c = _f(model.p_cruise_w())
        per_uav[uid] = {
            "cls": u.cls,
            "auw_kg": auw,
            "mtow_kg": _f(spec.mtow_kg),
            "payload_kg": _f(payload),
            "payload_estimated": bool(estimated),
            "payload_max_kg": pmax,
            "sensor_kinds": sorted(u.available_kinds),
            "base_minute_mode": bool(model.base_minute_mode),
            "E_nom_wh": None if nominal is None else _f(nominal),
            "E_avail_wh": _f(model.available_wh),
            "E_use_wh": e_use,
            "e_TO_wh": _f(model.e_takeoff_wh()),
            "e_LDG_wh": _f(model.e_landing_wh()),
            "reserve_wh": _f(model.reserve_wh()),
            "k_unc": _f(model.k_unc),
            "eta_T": _f(model.eta_T),
            "battery_temp_c": _f(model.battery_temp_c),
            "P_c_w": p_c,
            "P_h_w": _f(model.p_hover_w()),
            "cruise_minutes": _f(e_use / (p_c * model.k_unc) * (1.0 if model.base_minute_mode else 60.0))
            if p_c > 0 else 0.0,
            "cruise_equiv_km": _f(model.cruise_equiv_km(max(e_use, 0.0))),
            "D_use_km": _f(model.usable_distance_cap_km()),
            "v_min_mps": _f(v_min),
            "stall_speed_mps": _f(v_s),
            "stall_margin_ok": bool(stall_ok),
            "lost_link_policy": inputs.c2.policy_for(uid, u.cls),
            "issues": issues,
            "ok": not issues,
        }
    return per_uav, errors, warnings


# ---------------------------------------------------------------------------
# 步 7–9：區域裁切、高度帶、感測可行
# ---------------------------------------------------------------------------

def _area_of(g: shapely.Geometry | None) -> float:
    return 0.0 if g is None or g.is_empty else _f(g.area)


def _theta_grid() -> np.ndarray:
    return np.arange(N_DIRECTION_SAMPLES) * (_TWO_PI / N_DIRECTION_SAMPLES)


def _direction_feasibility(spec: PlatformSpec, auw_kg: float, v_cap: float, wind_xy: np.ndarray,
                           v_g_min: float) -> tuple[bool, bool, int, float]:
    """步 9「∃θ 使 V_a* 存在」（規格 §4.4）：回傳 (any_ok, two_way_ok, n_ok, V_g,max)。"""
    v_min = max(spec.v_min_mission(), 1.2 * spec.stall_speed(auw_kg))
    v_max = spec.cruise_airspeed_mps
    if spec.max_airspeed_mps is not None:
        v_max = min(v_max, float(spec.max_airspeed_mps))
    if v_min > v_max:
        return False, False, 0, 0.0
    va, vg = v_air_star(_theta_grid(), v_min, v_max, v_cap, wind_xy)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(va) & np.isfinite(vg) & (vg >= v_g_min)
    two_way = ok & np.roll(ok, N_DIRECTION_SAMPLES // 2)
    vg_max = _f(np.nanmax(np.where(ok, vg, np.nan))) if ok.any() else 0.0
    return bool(ok.any()), bool(two_way.any()), int(ok.sum()), vg_max


def _band_dict(b: AltitudeBand) -> dict[str, Any]:
    return {"h_op_m": _f(b.h_op_m), "h_lo_m": _f(b.h_lo_m), "h_hi_m": _f(b.h_hi_m), "dh_band_m": _f(b.dh_band_m),
            "feasible": bool(b.feasible), "reason": b.reason}


def analyse_areas(ctx: MissionContext, dc: DecomposeContext) -> tuple[dict[str, dict], list[str], list[str]]:
    """規格 §6.2 步 3（區域在 DEM 內）、步 7（鏈路／半徑／禁區裁切）、步 8（高度帶）、步 9（感測可行）。

    回傳 (per_area, errors, warnings)。`per_area[aid]["classes"][cls]` 含 band、h_op/w/s/e_ext/d_turn、
    coverable_fraction、clip 損失比例、direction_ok、feasible/reason；`per_area[aid]["feasible_classes"]`、
    `coverable`、`uncoverable_reason`。
    """
    errors: list[str] = []
    warnings: list[str] = []
    per_area: dict[str, dict] = {}
    inputs = ctx.inputs
    cfg = ctx.cfg
    classes = _classes(inputs)
    home = Point(np.asarray(ctx.home_xy, dtype=float))
    nofly = shapely.union_all(list(ctx.nofly_buffered)) if ctx.nofly_buffered else None
    z_home = _f(ctx.z_ground_home_m)
    n_uav = {c: len(inputs.uavs_of_class(c)) for c in classes}
    any_coverable = False

    for area in inputs.areas:
        aid = area.area_id
        poly = ctx.areas_enu[aid]
        a_total = _area_of(poly)
        z_max_area = _f(ctx.terrain.max_in_polygon(poly))
        in_dem = math.isfinite(z_max_area)
        if not in_dem:
            errors.append(f"area_outside_dem({aid})")
        rec: dict[str, Any] = {
            "area_m2": a_total,
            "sensor_req": area.sensor_req,
            "payload_req_id": area.payload_req_id,
            "priority": _f(area.priority),
            "time_window_s": None if area.time_window_s is None else [_f(area.time_window_s[0]), _f(area.time_window_s[1])],
            "dist_home_m": _f(poly.distance(home)),
            "in_dem": in_dem,
            "classes": {},
        }
        served_kinds: set[str] = set()
        feasible_classes: list[str] = []
        for c in classes:
            spec = ctx.specs[c]
            kinds = ctx.available_kinds.get(c, frozenset())
            crec: dict[str, Any] = {"n_uav": n_uav[c], "sensor_ok": bool(class_can_serve(area.sensor_req, c, kinds))}
            found = resolve_sensor(dc, c, area.payload_req_id)
            if not crec["sensor_ok"] or found is None:
                crec.update({"feasible": False, "reason": "sensor_incompatible" if not crec["sensor_ok"] else "no_sensor"})
                rec["classes"][c] = crec
                continue
            sensor, rid = found
            crec["req_id_used"] = rid
            crec["sensor_kind"] = sensor.kind
            # --- 步 8 高度帶（胞多邊形層級） ---
            band = altitude_band(poly, c, area.payload_req_id, dc)
            crec["band"] = _band_dict(band)
            h_op = _f(band.h_op_m)
            w = _f(sensor.swath_m(h_op))
            s = _f(sensor.strip_spacing_m(h_op))
            e_ext = _f(end_extension_for(dc, c, sensor, h_op))
            d_turn = _f(d_turn_m(c, s, e_ext, dc))
            crec.update({"h_op_m": h_op, "w_m": w, "s_m": s, "e_ext_m": e_ext, "d_turn_m": d_turn,
                         "V_g_cap_mps": _f(sensor.max_ground_speed_mps(h_op))})
            if band.reason == "A_needs_waiver":
                warnings.append(f"A_needs_waiver({aid}: h_ceil {ctx.h_ceil_m.get(c)} m 無豁免，Δh_band={band.dh_band_m:.1f} < "
                                f"{cfg.dh_band_min_m:g} m → A 不可指派，改由 B 或給 waiver_max_agl_m)")
            elif band.reason == "gsd_unattainable":
                warnings.append(f"gsd_unattainable({aid}, {c}: h_max,B(GSD_req)={sensor.agl_window()[1]:.1f} < "
                                f"h_minfw {spec.min_fw_operation_agl_m:g} + Δh_band,min {cfg.dh_band_min_m:g})")
            elif not band.feasible:
                warnings.append(f"altitude_band_infeasible({aid}, {c}: {band.reason}, Δh_band={band.dh_band_m:.1f})")
            # --- 步 7 裁切 ---
            z = poly
            losses: dict[str, float] = {}
            if cfg.enforce_link:
                lp = ctx.link_poly.get(c)
                if lp is not None and not lp.is_empty:
                    z1 = z.intersection(lp.buffer(-d_turn))
                    losses["link"] = max(0.0, (_area_of(z) - _area_of(z1)) / a_total) if a_total > 0 else 0.0
                    z = z1
                rho = _f(ctx.rho_sym_m.get(c, math.inf))
                if math.isfinite(rho):
                    r = rho - d_turn
                    z1 = z.intersection(home.buffer(r, quad_segs=64)) if r > 0 else z.intersection(Point(0, 0).buffer(0))
                    losses["radius"] = max(0.0, (_area_of(z) - _area_of(z1)) / a_total) if a_total > 0 else 0.0
                    z = z1
            if nofly is not None and not nofly.is_empty:
                z1 = z.difference(nofly)
                losses["nofly"] = max(0.0, (_area_of(z) - _area_of(z1)) / a_total) if a_total > 0 else 0.0
                z = z1
            losses = {k: (0.0 if v < _AREA_TOL_REL else _f(v)) for k, v in losses.items()}
            cov_frac = _f(_area_of(z) / a_total) if a_total > 0 else 0.0
            if cov_frac < _AREA_TOL_REL:
                cov_frac = 0.0
            crec["clip_loss"] = losses
            crec["coverable_fraction"] = cov_frac
            crec["coverable_area_m2"] = _f(_area_of(z))
            lost = {k: v for k, v in losses.items() if v > 0}
            if lost:
                warnings.append(f"uncoverable({aid}/{c}: " + ", ".join(f"{k} {v:.1%}" for k, v in lost.items()) + ")")
            # --- 步 9 方向可行 ---
            z_op = (z_max_area if in_dem else z_home) + h_op
            wind_xy = np.asarray(ctx.wind.wind_vec(z_op), dtype=float).reshape(-1, 2)[0]
            auw = _f(ctx.auw_kg.get(c, spec.reference_auw_kg))
            v_cap = _f(sensor.max_ground_speed_mps(h_op))
            dir_ok, two_way, n_ok, vg_max = _direction_feasibility(spec, auw, v_cap, wind_xy, cfg.v_g_min_mps)
            crec.update({"direction_ok": dir_ok, "two_way_ok": two_way, "n_directions_ok": n_ok,
                         "V_g_max_mps": vg_max, "z_op_m": _f(z_op), "one_way_only": bool(dir_ok and not two_way)})
            if dir_ok and not two_way:
                warnings.append(f"one_way_only({aid}, {c}: 僅單向可行，需同向 racetrack 轉場 D34)")
            # --- 綜合 ---
            reason = ""
            if not in_dem:
                reason = "terrain_unknown"
            elif not band.feasible:
                reason = band.reason or "band_infeasible"
            elif cov_frac <= 0.0:
                reason = "outside_link" if losses.get("link", 0.0) > 0 else (
                    "outside_radius" if losses.get("radius", 0.0) > 0 else "nofly")
            elif not dir_ok:
                reason = "no_feasible_direction"
            feasible = reason == ""
            # 作業時間下界（步 10 用）：可覆蓋面積 / (s · V_g,max)
            t_lb = math.inf
            if feasible and s > 0 and vg_max > 0:
                t_lb = _f(_area_of(z) / (s * vg_max))
            crec.update({"feasible": feasible, "reason": reason, "T_oper_lb_s": t_lb})
            rec["classes"][c] = crec
            if feasible:
                feasible_classes.append(c)
                served_kinds.add(sensor.kind)
                if c == "A" and (c, rid + "_rgb") in ctx.sensors:
                    served_kinds.add("rgb")
        # 每區至少一構型（both 需兩半）
        if area.sensor_req == "both":
            coverable = {"lidar", "rgb"} <= served_kinds
            missing = sorted({"lidar", "rgb"} - served_kinds)
            unc_reason = "" if coverable else f"sensor: both 缺 {','.join(missing)} 半套"
        else:
            coverable = bool(feasible_classes)
            unc_reason = "" if coverable else "sensor: 無可行構型（" + "; ".join(
                f"{c}={rec['classes'][c].get('reason', '?')}" for c in classes) + "）"
        rec["feasible_classes"] = feasible_classes
        rec["served_kinds"] = sorted(served_kinds)
        rec["coverable"] = coverable
        rec["uncoverable_reason"] = unc_reason or None
        rec["T_oper_lb_s"] = min((rec["classes"][c]["T_oper_lb_s"] / max(n_uav[c], 1) for c in feasible_classes),
                                 default=math.inf)
        if not coverable:
            warnings.append(f"uncoverable({aid}: {unc_reason})")
        any_coverable = any_coverable or coverable
        per_area[aid] = rec
    if inputs.areas and not any_coverable:
        errors.append("no_coverable_area: 所有區域皆無可行構型（感測相容／高度帶／鏈路／方向）")
    return per_area, errors, warnings


# ---------------------------------------------------------------------------
# 步 11–12：派生量與警告
# ---------------------------------------------------------------------------

def derive_by_class(ctx: MissionContext, dc: DecomposeContext, per_area: dict[str, dict], w_go: dict[str, float],
                    usable_alternates: dict[str, list[str]]) -> tuple[dict[str, dict], list[str], list[str]]:
    """`derived[cls]`（規格 §6.2 步 11–13）：R_eff（W_R 政策）、V_g（逆／順／側風）、V_g_cap、h_op/w/s/e_ext/d_turn
    （主需求；`*_by_req` 逐需求）、P_c_ref、h_ceil_source、A_needs_waiver、轉換段指令（D8/D29）、k_unc、D_use、
    provisional_params、endurance_range_dev。回傳 (derived, errors, warnings)。"""
    errors: list[str] = []
    warnings: list[str] = []
    derived: dict[str, dict] = {}
    inputs = ctx.inputs
    cfg = ctx.cfg
    z_site = _f(ctx.z_ground_home_m)
    z_terrain_max = _f(np.nanmax(ctx.terrain.z)) if np.isfinite(ctx.terrain.z).any() else z_site
    waiver = inputs.airspace.waiver_max_agl_m
    for c in _classes(inputs):
        spec = ctx.specs[c]
        rep = ctx.models_rep[c]
        tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
        if tc.slope_tr_rad > math.radians(spec.gamma_tr_max_deg) + 1e-9:
            errors.append(f"transition_slope_exceeds({c}: {math.degrees(tc.slope_tr_rad):.2f} > "
                          f"gamma_tr_max {spec.gamma_tr_max_deg:g} deg)")
        h_ceil = ctx.h_ceil_m.get(c)
        h_ceil_f = math.inf if h_ceil is None else _f(h_ceil)
        if cfg.h_ceil_override_m is not None:
            h_ceil_f = _f(cfg.h_ceil_override_m)
        # 主需求：第一個能解析感測器之區域需求
        by_req: dict[str, dict[str, float]] = {}
        primary: dict[str, float] | None = None
        for aid, rec in per_area.items():
            crec = rec["classes"].get(c, {})
            if "h_op_m" not in crec:
                continue
            rid = crec["req_id_used"]
            if rid not in by_req:
                by_req[rid] = {k: crec[k] for k in ("h_op_m", "w_m", "s_m", "e_ext_m", "d_turn_m", "V_g_cap_mps")}
                if primary is None:
                    primary = by_req[rid]
        # R_eff（規格 §3.2 W_R 政策；與 decompose.r_eff_for_class 同一 z_top 規則）
        z_top = z_terrain_max + (h_ceil_f if math.isfinite(h_ceil_f) else 300.0)
        W_R = _f(r_eff_wind_speed(ctx.wind, spec, z_site, z_top, cfg.r_eff_wind_policy))
        R_eff = _f(spec.r_eff(W_R))
        h_ref = primary["h_op_m"] if primary is not None else _f(spec.min_fw_operation_agl_m)
        z_ref = z_site + h_ref
        wind_xy = np.asarray(ctx.wind.wind_vec(z_ref), dtype=float).reshape(-1, 2)[0]
        beta = _f(ctx.wind.dir_from_at(z_ref)) if ctx.wind.n_layers else 0.0
        va = spec.cruise_airspeed_mps

        def _vg(theta: float) -> float:
            v = ground_speed(theta, va, wind_xy)[0]
            return _f(v) if np.isfinite(v) else math.nan

        v_implied = spec.reference_total_range_km * 1000.0 / (spec.reference_endurance_min * 60.0)
        dev = v_implied / va - 1.0
        if abs(dev) > ENDURANCE_RANGE_TOL:
            warnings.append(f"endurance_range_inconsistent({c}: R_ref/T_ref = {v_implied:.2f} m/s vs V_a {va:.2f} → {dev:+.1%})")
        prov = sorted(set(spec.provisional) | set(spec.provisional_supplementary))
        if prov:
            warnings.append(f"provisional_params({c}: {', '.join(prov)})")
        if not (spec.airspeed_sensor_validated and spec.battery_monitor_validated):
            warnings.append(f"avionics_unvalidated({c}: airspeed_sensor_validated={spec.airspeed_sensor_validated}, "
                            f"battery_monitor_validated={spec.battery_monitor_validated} → k_unc={rep.k_unc:.4f}, "
                            f"V_tr,cmd={tc.v_tr_mps:g}, H_tc,cmd={tc.h_tc_cmd_m:g}, D_use={rep.usable_distance_cap_km():g} km)")
        a_needs_waiver = (waiver is None and math.isfinite(h_ceil_f)
                          and h_ceil_f <= spec.min_fw_operation_agl_m + cfg.dh_band_min_m + 1e-9)
        e_use = _f(rep.e_usable_wh())
        t_takeoff = spec.transition_start_agl_m / spec.vtol_climb_rate_mps + spec.t_tr_out_s + tc.t_acc_s
        t_landing = tc.t_acc_s + spec.t_tr_in_s + spec.hover_descent_time_s() + spec.t_hover_reserve_s
        derived[c] = {
            "n_uav": len(inputs.uavs_of_class(c)),
            "uav_ids": [u.uav_id for u in inputs.uavs_of_class(c)],
            # 轉彎／風
            "R_eff": R_eff, "W_R": W_R, "R_plan": _f(spec.turn_radius_planning_m),
            "R_theo": _f(spec.turn_radius_theoretical_m), "W_star": _f(spec.w_star()),
            "V_a": _f(va), "V_a_min": _f(spec.v_min_mission()),
            "wind_speed_ref_mps": _f(np.hypot(wind_xy[0], wind_xy[1])), "wind_dir_from_ref_deg": _f(math.degrees(beta)),
            "V_g_headwind": _vg(beta), "V_g_tailwind": _vg(beta + math.pi), "V_g_crosswind": _vg(beta + 0.5 * math.pi),
            # 感測／作業高度
            "V_g_cap": None if primary is None else primary["V_g_cap_mps"],
            "h_op": None if primary is None else primary["h_op_m"],
            "w": None if primary is None else primary["w_m"],
            "s": None if primary is None else primary["s_m"],
            "e_ext": None if primary is None else primary["e_ext_m"],
            "d_turn": None if primary is None else primary["d_turn_m"],
            "by_req": by_req,
            "h_minfw": _f(spec.min_fw_operation_agl_m), "h_clr": _f(ctx.h_clr_m),
            "h_ceil": None if not math.isfinite(h_ceil_f) else h_ceil_f, "h_ceil_source": ctx.h_ceil_source,
            "A_needs_waiver": bool(a_needs_waiver),
            # 能量（代表模型；規格 §5）
            "P_c_ref": _f(rep.p_cruise_ref_w()), "P_c_rep": _f(rep.p_cruise_w()), "P_h_rep": _f(rep.p_hover_w()),
            "k_bm": _f(rep.k_bm), "k_as": _f(rep.k_as), "k_unc": _f(rep.k_unc), "eta_T_rep": _f(rep.eta_T),
            "e_TO": _f(rep.e_takeoff_wh()), "e_LDG": _f(rep.e_landing_wh()), "reserve_wh": _f(rep.reserve_wh()),
            "E_use_rep": e_use, "base_minute_mode": bool(rep.base_minute_mode),
            "endurance_range_dev": _f(dev), "v_implied_mps": _f(v_implied),
            "D_use_km": _f(rep.usable_distance_cap_km()), "D_model_km": _f(rep.cruise_equiv_km(max(e_use, 0.0))),
            "rho_sym_km": _f(ctx.rho_sym_m.get(c, math.inf)) / 1000.0,
            "link_range_km": inputs.c2.range_km_for(c, spec),
            # 轉換段（D8、D29、D37）
            "v_tr_cmd": _f(tc.v_tr_mps), "h_tc_cmd": _f(tc.h_tc_cmd_m), "h_tc_energy": _f(tc.h_tc_energy_m),
            "h_tc_clear": _f(tc.h_tc_clear_m), "d_tr": _f(tc.d_tr_m), "d_acc": _f(tc.d_acc_m), "t_acc": _f(tc.t_acc_s),
            "slope_tr_deg": _f(math.degrees(tc.slope_tr_rad)), "gamma_tr_max_deg": _f(spec.gamma_tr_max_deg),
            "t_takeoff_s": _f(t_takeoff), "t_landing_s": _f(t_landing),
            "airspeed_sensor_validated": bool(spec.airspeed_sensor_validated),
            "battery_monitor_validated": bool(spec.battery_monitor_validated),
            # 氣象／站點
            "W_go": w_go.get(c), "beaufort_upper_mps": _f(spec.wind_limit_public_mps()),
            "crosswind_site_max_mps": _f(spec.crosswind_site_max_mps[1 if spec.airspeed_sensor_validated else 0]),
            "gust_site_max_mps": _f(spec.gust_site_max_mps[1 if spec.airspeed_sensor_validated else 0]),
            "usable_alternates": list(usable_alternates.get(c, [])),
            "provisional_params": prov,
        }
    return derived, errors, warnings


# ---------------------------------------------------------------------------
# 步 10：時間窗（D22）
# ---------------------------------------------------------------------------

def check_time_windows(ctx: MissionContext, per_area: dict[str, dict],
                       derived: dict[str, dict]) -> tuple[list[str], list[str], float]:
    """區域窗 ∩ 空域窗非空；粗估 makespan **下界**（Σ_k min_c T_oper,kc/n_c + 起降固定時間 + 最近區域往返）須 ≤ 空域窗長；
    每區之作業下界須 ≤ 該區窗長。回傳 (errors, warnings, makespan_lb_s)。`cfg.enforce_time_windows=False` → 只算下界。"""
    errors: list[str] = []
    warnings: list[str] = []
    inputs = ctx.inputs
    cfg = ctx.cfg
    aw = inputs.airspace.time_window_s
    fixed = min((d["t_takeoff_s"] + d["t_landing_s"] for d in derived.values()), default=0.0)
    va_max = max((d["V_a"] for d in derived.values()), default=20.0)
    d_near = min((rec["dist_home_m"] for rec in per_area.values() if rec["coverable"]), default=0.0)
    total_oper = 0.0
    for aid, rec in per_area.items():
        t_lb = rec.get("T_oper_lb_s", math.inf)
        if math.isfinite(t_lb):
            total_oper += t_lb
        tw = rec["time_window_s"]
        if not cfg.enforce_time_windows or tw is None:
            continue
        lo, hi = float(tw[0]), float(tw[1])
        if aw is not None:
            lo, hi = max(lo, float(aw[0])), min(hi, float(aw[1]))
            if lo >= hi:
                errors.append(f"time_window_disjoint({aid}: 區域窗 {tw} ∩ 空域窗 {list(aw)} = ∅)")
                continue
        if math.isfinite(t_lb) and t_lb > (hi - lo) + 1e-9:
            errors.append(f"time_window_insufficient({aid}: 作業下界 {t_lb:.0f} s > 有效窗長 {hi - lo:.0f} s)")
    makespan_lb = fixed + 2.0 * d_near / va_max + total_oper
    if cfg.enforce_time_windows and aw is not None:
        span = float(aw[1]) - float(aw[0])
        if makespan_lb > span + 1e-9:
            errors.append(f"time_window_insufficient(airspace: makespan 下界 {makespan_lb:.0f} s > 窗長 {span:.0f} s)")
    for d in derived.values():
        d["makespan_lb_s"] = _f(makespan_lb)
    return errors, warnings, _f(makespan_lb)


# ---------------------------------------------------------------------------
# 主函式
# ---------------------------------------------------------------------------

def _status(errors: list[str], warnings: list[str]) -> str:
    if errors:
        return "NO_GO"
    return "GO_WITH_WARNINGS" if warnings else "GO"


def preflight(inputs: MissionInputs, cfg: PlannerConfig, ctx: MissionContext | None = None) -> FeasibilityReport:
    """PRECHECK（規格 §6.2 步 1–13；契約 §3A）。`ctx` None → 自行 `build_context(inputs, cfg)`。

    不變量（契約）：任一構型 P_link 未定義、`terrain_clearance_m` 兩層皆 None、`alternates==()`（且 required）、
    「給了 Wh/mAh 但 nominal 兩層皆 None」→ ``NO_GO`` 且 `errors` 含鍵名；`derived["A"]["A_needs_waiver"]` 於
    h_ceil == 120 且無豁免為 True；`warnings` 含 `provisional_params`、`sensor_spec_default_example(cls)`、
    `battery_nominal_example`、`reference_battery_assumed`、`payload_estimated`、`airspace_ceiling_defaulted_120`、
    `endurance_range_inconsistent`、`overridden(...)`；`derived[cls]` 含 `P_c_ref`、`h_ceil_source`。
    """
    errors: list[str] = []
    warnings: list[str] = []

    # 步 1–2（輸入層，不需 context）
    specs, e, w = _resolved_specs(inputs)
    errors += e
    warnings += w
    e, w = check_required_inputs(inputs, cfg, specs)
    errors += e
    warnings += w
    e, w, w_go = check_weather(inputs, cfg, specs)
    errors += e
    warnings += w

    # 步 3–4（context）
    if ctx is None:
        try:
            ctx = build_context(inputs, cfg)
        except MissionInputError as exc:
            msg = f"build_context: {exc}"
            if not any(str(exc) in x for x in errors):
                errors.append(msg)
            return FeasibilityReport(status="NO_GO", errors=_dedupe(errors), warnings=_dedupe(warnings),
                                     per_area={}, per_uav={}, derived={})
    warnings += list(ctx.warnings)
    dc = ctx.decompose_ctx()

    # 步 6 機體
    per_uav, e, w = check_airframes(ctx)
    errors += e
    warnings += w
    # 步 5 站點
    e, w, usable_alt = check_sites(ctx)
    errors += e
    warnings += w
    # 步 7–9 區域
    per_area, e, w = analyse_areas(ctx, dc)
    errors += e
    warnings += w
    # 步 11–12 派生與警告
    derived, e, w = derive_by_class(ctx, dc, per_area, w_go, usable_alt)
    errors += e
    warnings += w
    # 步 10 時間窗
    e, w, _lb = check_time_windows(ctx, per_area, derived)
    errors += e
    warnings += w

    # 契約要求之裸字串警告（provisional_params 逐構型細節已在上方）
    if any(x.startswith("provisional_params(") for x in warnings):
        warnings.insert(0, "provisional_params")
    errors = _dedupe(errors)
    warnings = _dedupe(warnings)
    return FeasibilityReport(status=_status(errors, warnings), errors=errors, warnings=warnings,
                             per_area=per_area, per_uav=per_uav, derived=derived)
