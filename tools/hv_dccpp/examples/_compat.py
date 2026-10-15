"""_compat.py — 範例腳本共用工具（路徑設定、摘要表、ArduPilot 匯出與 MAVLink 機隊鏈路之後備實作）。

`hv_dccpp.ardupilot` / `hv_dccpp.mavlink_link` 若存在則優先使用（`get_ardupilot()` / `get_fleet_link()`）；
本檔案之後備實作（`_ArduPilotFallback`、`FleetLinkFallback`）只在該模組缺席時啟用，並記錄於
hv_dccpp/CONTRACT_DEVIATIONS.md（examples 節）。

後備 ArduPilot 匯出（依 scratchpad/ardupilot_notes.md §1–3）：
- 任務：home(frame 0, NAV_WAYPOINT, current 1) → NAV_VTOL_TAKEOFF(84) → DO_CHANGE_SPEED(178) →
  NAV_WAYPOINT(16)（弧段端點／相位邊界／稀疏化航點；MAV_FRAME_GLOBAL_RELATIVE_ALT，alt = MSL − home MSL）
  ＋ OPERATE 段以 DO_SET_CAM_TRIGG_DIST(206) 包住 → DO_LAND_START(189) → NAV_VTOL_LAND(85)。
- Rally：MAV_CMD_NAV_RALLY_POINT(5100)；Fence：inclusion 5001（任務區域凸包外擴）、exclusion 5002（禁飛區）、
  return point 5000（home）。
- `.param`：AIRSPEED_CRUISE/MIN/MAX/STALL、ROLL_LIMIT_DEG、WP_LOITER_RAD、RTL_RADIUS、TECS_CLMB_MAX、TECS_SINK_MAX、
  Q_ENABLE、FENCE_*（對照表見 ardupilot_notes §3）。
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
import traceback
from dataclasses import asdict, dataclass, field
from typing import Any, Callable, Iterable, Sequence

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

FIG_DIR = os.path.join(ROOT, "figures")
ARDU_DIR = os.path.join(FIG_DIR, "ardupilot")

# Windows 主控台（cp950）遇到非 BMP／特殊符號時不要讓 print 炸掉（模組備註字串含 ×、−、→ 等）
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

import numpy as np  # noqa: E402

from hv_dccpp.mission import MissionPlan, Phase, UAVPlan, Waypoint  # noqa: E402


# ---------------------------------------------------------------------------
# 通用工具
# ---------------------------------------------------------------------------
def step(name: str, fn: Callable[[], Any], *, default: Any = None) -> Any:
    """執行一個步驟：印出耗時；例外時印出 traceback 並回傳 ``default``（腳本繼續）。"""
    t0 = time.perf_counter()
    print(f"\n=== {name} ===", flush=True)
    try:
        out = fn()
        print(f"--- {name}: {time.perf_counter() - t0:.2f} s", flush=True)
        return out
    except Exception:
        print(f"!!! {name} FAILED after {time.perf_counter() - t0:.2f} s", flush=True)
        traceback.print_exc()
        return default


def fmt_table(rows: Sequence[Sequence[Any]], header: Sequence[str]) -> str:
    """簡單等寬表格。"""
    cells = [[str(h) for h in header]] + [[str(c) for c in r] for r in rows]
    w = [max(len(r[i]) for r in cells) for i in range(len(header))]
    lines = ["  ".join(c.ljust(w[i]) for i, c in enumerate(r)) for r in cells]
    lines.insert(1, "  ".join("-" * x for x in w))
    return "\n".join(lines)


def plan_summary(plan: MissionPlan, ctx: Any = None) -> str:
    """每機摘要表：條帶數、距離、時間、能量使用／餘量、落地點；再附 makespan 與未覆蓋。"""
    rows = []
    for uid in sorted(plan.plans):
        p = plan.plans[uid]
        e_avail = ""
        if ctx is not None and uid in ctx.models:
            e_avail = f"/{ctx.models[uid].available_wh:.0f}"
        rows.append([uid, p.cls, len(p.strips), f"{p.distance_m / 1000:.1f}",
                     f"{p.t_takeoff_s:.0f}", f"{p.t_land_s:.0f}", f"{(p.t_land_s - p.t_takeoff_s) / 60:.1f}",
                     f"{p.energy_used_wh:.0f}{e_avail}", f"{p.energy_margin_wh:.0f}", p.landing_site,
                     len(p.waypoints), p.orbits_inserted, len(p.warnings)])
    hdr = ["uav", "cls", "strips", "dist_km", "t_to_s", "t_land_s", "min", "E_used_wh", "E_margin_wh",
           "landing", "n_wp", "orbits", "warn"]
    out = fmt_table(rows, hdr) if rows else "(no UAV plans)"
    out += f"\nmakespan = {plan.makespan_s:.0f} s ({plan.makespan_s / 60:.1f} min); status = {plan.feasibility.status}"
    out += f"; cells = {len(plan.cells)}; strips = {len(plan.strips)}; uncovered = {plan.uncovered}"
    if plan.timing_ms:
        out += "\ntiming_ms: " + ", ".join(f"{k}={v:.0f}" for k, v in plan.timing_ms.items())
    return out


def validation_summary(rep: Any) -> str:
    flags = {k: getattr(rep, k) for k in ("slope_ok", "agl_ok", "curvature_ok", "energy_ok", "link_ok",
                                          "airspace_ok", "time_window_ok")}
    lines = [f"validate: passed={rep.passed}  coverage_ok={rep.coverage_ok}  conflicts={len(rep.conflicts)}  "
             f"min_sep_h={rep.min_sep_h_m:.0f} m  makespan={rep.makespan_s:.0f} s  total_dist={rep.total_distance_m / 1000:.1f} km"
             f"  ({rep.compute_ms:.0f} ms)",
             "flags: " + ", ".join(f"{k}={v}" for k, v in flags.items())]
    for c in rep.coverage:
        lines.append(f"  coverage {c.area_id}: " + ", ".join(f"{k}={v:.4f}" for k, v in sorted(c.coverage_by_cls.items()))
                     + f"  required={c.cls_required} -> {'PASS' if c.passed else 'FAIL'}")
    for uid, det in rep.details.items():
        if uid in ("coverage", "separation"):
            continue
        for d in det[:6]:
            lines.append(f"  {uid}: {d}")
        if len(det) > 6:
            lines.append(f"  {uid}: (+{len(det) - 6} more)")
    for d in rep.details.get("separation", [])[:4]:
        lines.append(f"  separation: {d}")
    return "\n".join(lines)


def telemetry_from_plan(plan: MissionPlan, t_s: float, ctx: Any, *, e_scale: float = 1.0) -> dict[str, Any]:
    """自計畫在 t_s 內插各機遙測（`dynamic.Telemetry`）：位置／高度線性內插、航向取外向腿 track、
    能量取計畫預測 × e_scale（未驗證電量監測時 dynamic 會改採 κ 推估）、現行條帶與進度自 OPERATE 航點推得。"""
    from hv_dccpp import dynamic as dyn
    out: dict[str, Any] = {}
    for uid, p in plan.plans.items():
        wps = p.waypoints
        if not wps:
            continue
        ts = np.array([w.t_s for w in wps])
        if t_s <= ts[0]:
            i0, i1, f = 0, 0, 0.0
        elif t_s >= ts[-1]:
            i0, i1, f = len(wps) - 1, len(wps) - 1, 0.0
        else:
            i1 = int(np.searchsorted(ts, t_s, side="right"))
            i0 = i1 - 1
            dt = ts[i1] - ts[i0]
            f = 0.0 if dt <= 0 else float((t_s - ts[i0]) / dt)
        a, b = wps[i0], wps[i1]
        x = a.x_m + f * (b.x_m - a.x_m)
        y = a.y_m + f * (b.y_m - a.y_m)
        z = a.z_m + f * (b.z_m - a.z_m)
        e = a.energy_remaining_wh + f * (b.energy_remaining_wh - a.energy_remaining_wh)
        strip_id, prog = None, 0.0
        if a.phase is Phase.OPERATE and a.strip_id is not None and a.strip_id in plan.strips:
            s = plan.strips[a.strip_id]
            p0, p1 = (s.pL, s.pR) if a.dir == -1 else (s.pR, s.pL)
            L = float(np.hypot(*(p1 - p0)))
            prog = float(np.dot(np.array([x, y]) - p0, (p1 - p0) / max(L, 1e-9)) / max(L, 1e-9))
            prog = min(1.0, max(0.0, prog))
            strip_id = a.strip_id
        out[uid] = dyn.Telemetry(uid, float(t_s), float(x), float(y), float(z), math.radians(a.track_deg) % (2 * math.pi),
                                 float(a.airspeed_mps), float(e * e_scale), strip_id, prog, True)
    return out


# ---------------------------------------------------------------------------
# ArduPilot 匯出（後備實作）
# ---------------------------------------------------------------------------
def _mav():
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil
    return mavutil.mavlink


@dataclass(slots=True)
class MissionItem:
    """一個 MAVLink 任務項目（MISSION_ITEM_INT 欄位；x/y 為 lat/lon deg、z 為公尺）。``wp_index`` = 對應之
    UAVPlan.waypoints 索引（DO_ 指令為 -1）。"""
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: float
    y: float
    z: float
    wp_index: int = -1
    label: str = ""

    def qgc_line(self) -> str:
        return "\t".join([str(self.seq), str(self.current), str(self.frame), str(self.command),
                          f"{self.param1:.6f}", f"{self.param2:.6f}", f"{self.param3:.6f}", f"{self.param4:.6f}",
                          f"{self.x:.8f}", f"{self.y:.8f}", f"{self.z:.3f}", str(self.autocontinue)])


def _thin_waypoints(wps: Sequence[Waypoint], *, max_gap_m: float = 400.0, heading_tol_deg: float = 4.0,
                    z_tol_m: float = 6.0) -> list[int]:
    """稀疏化航點索引（ardupilot_notes §1：直線只留端點、弧段每 20–30° 一點）。保留：首尾、相位／感測／條帶／弧變化、
    航跡角變化 > tol、高度偏離線性內插 > tol、或距上一保留點 > max_gap。"""
    if not wps:
        return []
    keep = [0]
    n = len(wps)
    for i in range(1, n - 1):
        a, b, c = wps[i - 1], wps[i], wps[i + 1]
        last = wps[keep[-1]]
        dtrack = abs((b.track_deg - last.track_deg + 180.0) % 360.0 - 180.0)
        changed = (b.phase is not a.phase or b.sensor_on != a.sensor_on or b.strip_id != a.strip_id
                   or (b.arc is None) != (a.arc is None) or c.phase is not b.phase or c.sensor_on != b.sensor_on
                   or c.strip_id != b.strip_id or (c.arc is None) != (b.arc is None))
        gap = math.hypot(b.x_m - last.x_m, b.y_m - last.y_m)
        dz = abs((b.z_m - last.z_m) - (c.z_m - last.z_m) * (gap / max(math.hypot(c.x_m - last.x_m, c.y_m - last.y_m), 1e-9)))
        if changed or dtrack > heading_tol_deg or gap > max_gap_m or dz > z_tol_m:
            keep.append(i)
    keep.append(n - 1)
    return keep


def _cam_trigger_dist_m(ctx: Any, uplan: UAVPlan, plan: MissionPlan) -> float:
    """相機等距觸發距離：sensor.footprint_len_m(h_op)·(1 − o_fwd) 之近似——取 strip.h_op 下之足跡長 × 0.3；LiDAR 亦用同一
    指令包住（待驗證，依酬載）。"""
    try:
        sid = next((s for s, _d in uplan.strips), None)
        strip = plan.strips.get(sid) if sid else None
        cells = {c.cell_id: c for c in plan.cells}
        req = cells[strip.cell_id].payload_req_id if strip and strip.cell_id in cells else "default"
        from hv_dccpp.decompose import resolve_sensor
        found = resolve_sensor(ctx.decompose_ctx(), uplan.cls, req)
        if found is not None and strip is not None:
            sensor = found[0]
            fl = float(sensor.footprint_len_m(float(strip.h_op_m)))
            o_fwd = float(getattr(getattr(sensor, "req", None), "forward_overlap", 0.7) or 0.7)
            return max(5.0, fl * (1.0 - o_fwd))
    except Exception:
        pass
    return 50.0


def mission_items(uplan: UAVPlan, ctx: Any, plan: MissionPlan, *, thin: bool = True) -> list[MissionItem]:
    """UAVPlan → ArduPilot QuadPlane 任務項目（後備實作；見模組 docstring）。"""
    mav = _mav()
    spec = ctx.specs[uplan.cls]
    frame_rel = mav.MAV_FRAME_GLOBAL_RELATIVE_ALT
    home = ctx.inputs.home
    home_msl = float(ctx.frame.enu_z_to_msl(np.zeros((1, 2)), 0.0)[0])
    site_xy, site_z = ctx.sites_enu[uplan.landing_site]
    land_ll = ctx.frame.to_wgs84(np.array([site_xy[0], site_xy[1], site_z]))[0]
    items: list[MissionItem] = []

    def add(cmd: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0, p4: float = 0.0, lat: float = 0.0,
            lon: float = 0.0, alt: float = 0.0, *, frame: int = frame_rel, wp_index: int = -1, label: str = "",
            current: int = 0) -> None:
        items.append(MissionItem(len(items), frame, cmd, current, 1, p1, p2, p3, p4, lat, lon, alt, wp_index, label))

    add(mav.MAV_CMD_NAV_WAYPOINT, lat=home.lat_deg, lon=home.lon_deg, alt=home_msl, frame=mav.MAV_FRAME_GLOBAL,
        current=1, label="home")
    wps = uplan.waypoints
    if not wps:
        return items
    to_lat = np.array([[w.lat_deg, w.lon_deg, w.alt_msl_m] for w in wps], dtype=float)
    idx = _thin_waypoints(wps) if thin else list(range(len(wps)))
    # 起飛：垂直爬升至 transition_start_agl（ardupilot_notes §3）
    takeoff_alt = float(spec.transition_start_agl_m)
    add(mav.MAV_CMD_NAV_VTOL_TAKEOFF, p4=float("nan"), lat=wps[0].lat_deg, lon=wps[0].lon_deg, alt=takeoff_alt,
        wp_index=0, label="VTOL_TAKEOFF")
    add(mav.MAV_CMD_DO_CHANGE_SPEED, p1=0.0, p2=float(spec.cruise_airspeed_mps), p3=-1.0, label="cruise speed")
    cam_on = False
    cam_dist = _cam_trigger_dist_m(ctx, uplan, plan)
    last_va = float(spec.cruise_airspeed_mps)
    skip_phases = {Phase.TAKEOFF, Phase.TRANS_OUT, Phase.LAND, Phase.TRANS_IN}
    for i in idx:
        w = wps[i]
        if w.phase in skip_phases:
            continue
        if w.sensor_on and not cam_on:
            add(mav.MAV_CMD_DO_SET_CAM_TRIGG_DIST, p1=cam_dist, p2=0.0, p3=1.0, label=f"cam on ({w.strip_id})")
            cam_on = True
        elif cam_on and not w.sensor_on:
            add(mav.MAV_CMD_DO_SET_CAM_TRIGG_DIST, p1=0.0, label="cam off")
            cam_on = False
        if abs(float(w.airspeed_mps) - last_va) > 0.5 and w.phase in (Phase.OPERATE, Phase.TRANSFER, Phase.ENTRY):
            add(mav.MAV_CMD_DO_CHANGE_SPEED, p1=0.0, p2=float(w.airspeed_mps), p3=-1.0, label="speed")
            last_va = float(w.airspeed_mps)
        alt_rel = float(to_lat[i, 2] - home_msl)
        pass_r = 0.0
        if w.arc is not None:
            pass_r = float(w.arc[2]) * (1.0 if w.arc[3] > 0 else -1.0)
        add(mav.MAV_CMD_NAV_WAYPOINT, p1=0.0, p2=float(w.accept_radius_m), p3=0.0, p4=float("nan"),
            lat=float(to_lat[i, 0]), lon=float(to_lat[i, 1]), alt=alt_rel, wp_index=i,
            label=f"{w.phase.name}{'/' + w.strip_id if w.strip_id else ''}")
    if cam_on:
        add(mav.MAV_CMD_DO_SET_CAM_TRIGG_DIST, p1=0.0, label="cam off")
    add(mav.MAV_CMD_DO_LAND_START, lat=float(land_ll[0]), lon=float(land_ll[1]), label="land start")
    add(mav.MAV_CMD_NAV_VTOL_LAND, p4=float("nan"), lat=float(land_ll[0]), lon=float(land_ll[1]), alt=0.0,
        wp_index=len(wps) - 1, label=f"VTOL_LAND@{uplan.landing_site}")
    return items


def rally_items(ctx: Any, cls: str) -> list[MissionItem]:
    mav = _mav()
    spec = ctx.specs[cls]
    h = float(spec.transition_cmd().h_tc_cmd_m)
    out: list[MissionItem] = []
    for s in ctx.inputs.alternates:
        out.append(MissionItem(len(out), mav.MAV_FRAME_GLOBAL_RELATIVE_ALT, mav.MAV_CMD_NAV_RALLY_POINT, 0, 1,
                               0.0, 0.0, 0.0, 0.0, s.lat_deg, s.lon_deg, h, label=s.site_id))
    return out


def fence_items(ctx: Any, *, buffer_m: float = 1000.0) -> list[MissionItem]:
    """inclusion = (區域 ∪ 站點) 凸包 ⊕ buffer；exclusion = 禁飛區；return point = home。"""
    mav = _mav()
    import shapely
    from shapely.geometry import MultiPoint
    geoms = [g for g in ctx.areas_enu.values()]
    pts = [shapely.Point(xy[0], xy[1]) for xy, _z in ctx.sites_enu.values()]
    hull = shapely.unary_union(geoms + pts).convex_hull.buffer(buffer_m, quad_segs=2)
    hull = hull.simplify(50.0)
    ring = np.asarray(hull.exterior.coords)[:-1]
    ll = ctx.frame.to_wgs84(np.column_stack([ring, np.zeros(len(ring))]))
    out: list[MissionItem] = []
    n = len(ll)
    for lat, lon, _h in ll:
        out.append(MissionItem(len(out), mav.MAV_FRAME_GLOBAL, mav.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, 0, 1,
                               float(n), 0.0, 0.0, 0.0, float(lat), float(lon), 0.0, label="inclusion"))
    for z in ctx.inputs.airspace.no_fly:
        m = len(z.polygon_wgs84)
        for lat, lon in z.polygon_wgs84:
            out.append(MissionItem(len(out), mav.MAV_FRAME_GLOBAL, mav.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, 0, 1,
                                   float(m), 0.0, 0.0, 0.0, float(lat), float(lon), 0.0, label=f"exclusion {z.zone_id}"))
    home = ctx.inputs.home
    out.append(MissionItem(len(out), mav.MAV_FRAME_GLOBAL_RELATIVE_ALT, mav.MAV_CMD_NAV_FENCE_RETURN_POINT, 0, 1,
                           0.0, 0.0, 0.0, 0.0, home.lat_deg, home.lon_deg, 100.0, label="return"))
    return out


def param_lines(ctx: Any, cls: str) -> list[str]:
    """平台參數 → ArduPilot 參數建議（ardupilot_notes §3；未驗證項以註解標示）。"""
    spec = ctx.specs[cls]
    va = float(spec.cruise_airspeed_mps)
    g_c = math.radians(float(spec.planning_climb_angle_deg))
    g_d = math.radians(float(spec.planning_descent_angle_deg))
    h_ceil = ctx.h_ceil_m.get(cls)
    lines = [f"# HARVEST suggested ArduPlane/QuadPlane parameters for class {cls} ({spec.name})",
             "# ArduPlane 4.5+ names; verify against the installed firmware before use.",
             f"AIRSPEED_CRUISE,{va:.1f}", f"AIRSPEED_MIN,{spec.v_min_mission():.1f}"]
    if spec.max_airspeed_mps is not None:
        lines.append(f"AIRSPEED_MAX,{float(spec.max_airspeed_mps):.1f}")
    lines += [f"AIRSPEED_STALL,{float(spec.stall_airspeed_mps):.1f}",
              f"ROLL_LIMIT_DEG,{float(spec.max_bank_angle_deg):.0f}",
              f"WP_LOITER_RAD,{float(spec.turn_radius_planning_m):.0f}",
              f"RTL_RADIUS,{float(spec.turn_radius_planning_m):.0f}",
              f"TECS_CLMB_MAX,{va * math.sin(g_c):.2f}", f"TECS_SINK_MAX,{va * math.sin(g_d):.2f}",
              f"TECS_SINK_MIN,{min(va * math.sin(g_d), float(spec.max_fw_descent_rate_mps)):.2f}",
              "Q_ENABLE,1", "Q_FRAME_CLASS,1",
              f"Q_ASSIST_SPEED,{float(spec.stall_airspeed_mps) + 1.0:.1f}",
              "FENCE_ENABLE,1", "FENCE_TYPE,12", "FENCE_ACTION,1",
              f"FENCE_ALT_MAX,{float(h_ceil) if h_ceil is not None and math.isfinite(h_ceil) else 120.0:.0f}",
              f"FENCE_ALT_MIN,{float(ctx.h_clr_m):.0f}",
              f"# reserve_fraction={spec.reserve_fraction} -> BATT_LOW_MAH/BATT_CRT_MAH = reserve (needs pack mAh)",
              f"# battery_monitor_validated={spec.battery_monitor_validated}; airspeed_sensor_validated={spec.airspeed_sensor_validated}",
              f"# c2_link_range_km={spec.c2_link_range_km} -> FS_LONG_ACTN per lost-link policy"]
    return lines


def write_qgc(path: str, items: Sequence[MissionItem]) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("QGC WPL 110\n")
        for it in items:
            f.write(it.qgc_line() + "\n")
    return path


class _ArduPilotFallback:
    """`hv_dccpp.ardupilot` 缺席時之最小替代（export_all / mission_items）。"""
    name = "examples._compat fallback"

    mission_items = staticmethod(mission_items)
    rally_items = staticmethod(rally_items)
    fence_items = staticmethod(fence_items)
    param_lines = staticmethod(param_lines)

    @staticmethod
    def export_all(plan: MissionPlan, ctx: Any, out_dir: str) -> dict[str, list[str]]:
        os.makedirs(out_dir, exist_ok=True)
        written: dict[str, list[str]] = {}
        for uid, up in plan.plans.items():
            items = mission_items(up, ctx, plan)
            paths = [write_qgc(os.path.join(out_dir, f"{uid}.waypoints"), items)]
            with open(os.path.join(out_dir, f"{uid}_mission.json"), "w", encoding="utf-8") as f:
                json.dump([asdict(it) for it in items], f, indent=1, default=float)
            paths.append(os.path.join(out_dir, f"{uid}_mission.json"))
            paths.append(write_qgc(os.path.join(out_dir, f"{uid}_rally.waypoints"), rally_items(ctx, up.cls)))
            written[uid] = paths
        written["fence"] = [write_qgc(os.path.join(out_dir, "fence.waypoints"), fence_items(ctx))]
        for cls in ctx.specs:
            p = os.path.join(out_dir, f"class_{cls}.param")
            with open(p, "w", encoding="utf-8") as f:
                f.write("\n".join(param_lines(ctx, cls)) + "\n")
            written[f"param_{cls}"] = [p]
        return written


def get_ardupilot() -> Any:
    """回傳 `hv_dccpp.ardupilot`（若存在）否則後備實作。"""
    try:
        from hv_dccpp import ardupilot  # type: ignore
        if hasattr(ardupilot, "export_all"):
            return ardupilot
    except ImportError:
        pass
    return _ArduPilotFallback


def export_ardupilot(plan: MissionPlan, ctx: Any, cfg: Any, out_dir: str) -> tuple[str, dict[str, Any]]:
    """統一入口：`hv_dccpp.ardupilot.export_all(plan, ctx, cfg, out_dir)`（回 summary dict：``uavs[uid][n_items/n_nav/files]``）
    或後備 `_ArduPilotFallback.export_all(plan, ctx, out_dir)`（回 ``{uid: [paths]}``，包成同形狀）。回 (exporter_name, summary)。"""
    ardu = get_ardupilot()
    if ardu is _ArduPilotFallback:
        written = _ArduPilotFallback.export_all(plan, ctx, out_dir)
        summary = {"uavs": {}, "files": {}}
        for k, v in written.items():
            if k in plan.plans:
                summary["uavs"][k] = {"n_items": None, "files": {os.path.basename(p): p for p in v}}
            else:
                summary["files"][k] = v
        return _ArduPilotFallback.name, summary
    return "hv_dccpp.ardupilot", ardu.export_all(plan, ctx, cfg, out_dir)


def export_summary_lines(summary: dict[str, Any], base_dir: str) -> list[str]:
    lines = []
    for uid, d in summary.get("uavs", {}).items():
        files = d.get("files", {})
        names = [os.path.relpath(p, base_dir) for p in files.values()] if isinstance(files, dict) else list(files)
        lines.append(f"  {uid}: n_items={d.get('n_items')} n_nav={d.get('n_nav')} files={names}")
    for k, v in summary.get("files", {}).items():
        lines.append(f"  {k}: {[os.path.relpath(p, base_dir) for p in (v if isinstance(v, list) else [v])]}")
    return lines


# ---------------------------------------------------------------------------
# MAVLink 機隊鏈路（後備實作）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class VehicleState:
    uav_id: str
    t_wall: float = 0.0
    t_boot_s: float = 0.0
    lat_deg: float = float("nan")
    lon_deg: float = float("nan")
    alt_msl_m: float = float("nan")
    alt_rel_m: float = float("nan")
    seq: int = 0
    battery_pct: int = -1
    energy_wh: float = float("nan")
    vtol_state: int = 0
    landed_state: int = 0
    armed: bool = False
    mode: int = -1
    reached: list[int] = field(default_factory=list)
    n_msgs: int = 0


class FleetLinkFallback:
    """`hv_dccpp.mavlink_link.FleetLink` 缺席時之最小替代：每機一條 pymavlink 連線（udpout），任務／rally 上傳、
    參數設定、MISSION_START、MISSION_SET_CURRENT、遙測輪詢。Windows UDP 之 ConnectionResetError 已處理。"""

    def __init__(self, source_system: int = 255) -> None:
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.mav = mavutil.mavlink
        self.source_system = source_system
        self.conns: dict[str, Any] = {}
        self.states: dict[str, VehicleState] = {}

    # -- connection ------------------------------------------------------------
    def connect(self, uav_id: str, url: str, timeout_s: float = 15.0) -> bool:
        c = self.mavutil.mavlink_connection(url, source_system=self.source_system, source_component=190)
        sock = getattr(c, "port", None)
        if sock is not None and hasattr(sock, "ioctl") and hasattr(__import__("socket"), "SIO_UDP_CONNRESET"):
            try:
                import socket as _s
                sock.ioctl(_s.SIO_UDP_CONNRESET, False)
            except OSError:
                pass
        t0 = time.time()
        hb = None
        while time.time() - t0 < timeout_s and hb is None:
            c.mav.heartbeat_send(self.mav.MAV_TYPE_GCS, self.mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            try:
                hb = c.wait_heartbeat(timeout=1.0)
            except ConnectionResetError:
                hb = None
        if hb is None:
            return False
        self.conns[uav_id] = c
        self.states[uav_id] = VehicleState(uav_id)
        return True

    def close(self) -> None:
        for c in self.conns.values():
            try:
                c.close()
            except Exception:
                pass
        self.conns.clear()

    def _recv(self, c: Any, types: Any, timeout: float) -> Any:
        try:
            return c.recv_match(type=types, blocking=True, timeout=timeout)
        except ConnectionResetError:
            return None

    # -- upload ----------------------------------------------------------------
    def upload(self, uav_id: str, items: Sequence[MissionItem], mission_type: int = 0, timeout_s: float = 30.0
               ) -> tuple[bool, int]:
        """MISSION_COUNT → (MISSION_REQUEST_INT → MISSION_ITEM_INT)* → MISSION_ACK；回傳 (accepted, n_sent)。"""
        c = self.conns[uav_id]
        mav = self.mav
        c.mav.mission_count_send(c.target_system, c.target_component, len(items), mission_type)
        sent = 0
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            m = self._recv(c, ["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], 2.0)
            if m is None:
                c.mav.mission_count_send(c.target_system, c.target_component, len(items), mission_type)
                continue
            if m.get_type() == "MISSION_ACK":
                return m.type == mav.MAV_MISSION_ACCEPTED, sent
            it = items[m.seq]
            c.mav.mission_item_int_send(c.target_system, c.target_component, it.seq, it.frame, it.command, it.current,
                                        it.autocontinue, float(it.param1), float(it.param2), float(it.param3),
                                        float(it.param4), int(round(it.x * 1e7)), int(round(it.y * 1e7)), float(it.z),
                                        mission_type)
            sent += 1
        return False, sent

    def upload_mission(self, uav_id: str, items: Sequence[MissionItem]) -> tuple[bool, int]:
        return self.upload(uav_id, items, self.mav.MAV_MISSION_TYPE_MISSION)

    def upload_rally(self, uav_id: str, items: Sequence[MissionItem]) -> tuple[bool, int]:
        return self.upload(uav_id, items, self.mav.MAV_MISSION_TYPE_RALLY)

    def upload_fence(self, uav_id: str, items: Sequence[MissionItem]) -> tuple[bool, int]:
        return self.upload(uav_id, items, self.mav.MAV_MISSION_TYPE_FENCE)

    def set_param(self, uav_id: str, name: str, value: float, timeout_s: float = 3.0) -> bool:
        c = self.conns[uav_id]
        c.mav.param_set_send(c.target_system, c.target_component, name.encode("ascii"), float(value),
                             self.mav.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            m = self._recv(c, "PARAM_VALUE", 1.0)
            if m is None:
                continue
            pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode()
            if pid.rstrip("\x00") == name:
                return abs(float(m.param_value) - float(value)) < 1e-3 * max(1.0, abs(value))
        return False

    def start_mission(self, uav_id: str, first_seq: int = 1) -> bool:
        c = self.conns[uav_id]
        c.mav.command_long_send(c.target_system, c.target_component, self.mav.MAV_CMD_MISSION_START, 0,
                                float(first_seq), 0, 0, 0, 0, 0, 0)
        m = self._recv(c, "COMMAND_ACK", 3.0)
        return m is not None and m.result == self.mav.MAV_RESULT_ACCEPTED

    def set_current(self, uav_id: str, seq: int) -> None:
        c = self.conns[uav_id]
        c.mav.mission_set_current_send(c.target_system, c.target_component, int(seq))

    # -- telemetry -------------------------------------------------------------
    def poll(self, uav_id: str, duration_s: float = 0.3) -> VehicleState:
        """讀取 duration_s 內之遙測並更新 `states[uav_id]`。"""
        c = self.conns[uav_id]
        st = self.states[uav_id]
        mav = self.mav
        t0 = time.time()
        while time.time() - t0 < duration_s:
            m = self._recv(c, ["GLOBAL_POSITION_INT", "MISSION_CURRENT", "BATTERY_STATUS", "SYS_STATUS",
                               "EXTENDED_SYS_STATE", "MISSION_ITEM_REACHED", "HEARTBEAT"], max(0.05, duration_s))
            if m is None:
                break
            st.n_msgs += 1
            st.t_wall = time.time()
            t = m.get_type()
            if t == "GLOBAL_POSITION_INT":
                st.lat_deg, st.lon_deg = m.lat / 1e7, m.lon / 1e7
                st.alt_msl_m, st.alt_rel_m = m.alt / 1000.0, m.relative_alt / 1000.0
                st.t_boot_s = m.time_boot_ms / 1000.0
            elif t == "MISSION_CURRENT":
                st.seq = int(m.seq)
            elif t == "BATTERY_STATUS":
                st.battery_pct = int(m.battery_remaining)
                if m.energy_consumed >= 0:
                    st.energy_wh = m.energy_consumed / 36.0
            elif t == "SYS_STATUS":
                st.battery_pct = int(m.battery_remaining)
            elif t == "EXTENDED_SYS_STATE":
                st.vtol_state, st.landed_state = int(m.vtol_state), int(m.landed_state)
            elif t == "MISSION_ITEM_REACHED":
                st.reached.append(int(m.seq))
            elif t == "HEARTBEAT":
                st.armed = bool(m.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                st.mode = int(m.custom_mode)
        return st


def get_fleet_link() -> Any:
    """回傳 `hv_dccpp.mavlink_link.FleetLink`（若存在）否則 `FleetLinkFallback`。"""
    try:
        from hv_dccpp import mavlink_link  # type: ignore
        if hasattr(mavlink_link, "FleetLink"):
            return mavlink_link.FleetLink
    except ImportError:
        pass
    return FleetLinkFallback


__all__ = ["ROOT", "FIG_DIR", "ARDU_DIR", "step", "fmt_table", "plan_summary", "validation_summary",
           "telemetry_from_plan", "MissionItem", "mission_items", "rally_items", "fence_items", "param_lines",
           "write_qgc", "get_ardupilot", "export_ardupilot", "export_summary_lines", "VehicleState", "FleetLinkFallback", "get_fleet_link"]
