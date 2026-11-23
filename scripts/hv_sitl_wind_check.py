"""SITL wind post-flight check: did the wind really act, how big was it, and did the flight follow the plan.

Offline analysis of ONE trial directory written by ``scripts/hv_sitl_run.py fly``
(``runs/artifacts/<TRIAL>/``).  It starts no SITL, opens no connection and writes only to ``--out``
(default ``runs/artifacts/WINDFLOW_CHECK/<TRIAL>/``; never inside the trial directory, never under ``data/``).

    python scripts/hv_sitl_wind_check.py runs/artifacts/S1_HDMAP_20261015_PW3
    python scripts/hv_sitl_wind_check.py runs/artifacts/HV4SITL15 --no-bin --out <dir>
    python scripts/hv_sitl_wind_check.py <trial> --bin A1=sitl/vtol/i100/logs/00000025.BIN

Inputs actually used (formats learned from the real artifacts, not from a spec)
-------------------------------------------------------------------------------
``<UAV>/flight.csv``  t_s,lat,lon,rel_alt_m,airspeed,groundspeed,wp_seq,mode,armed[,wind_spd_est,wind_dir_from_est]
    * t_s is ``GLOBAL_POSITION_INT.time_boot_ms / 1000`` = the SITL (sim) clock.  Wall clock never enters.
    * There is NO heading and NO velocity column.  Course over ground comes from position differences.
    * Rows are ~1 Hz of sim time (the poll loop), with duplicated rows; duplicates are removed.
    * ``airspeed``/``groundspeed`` are VFR_HUD; ``wind_*_est`` is the MAVLink WIND message (FC estimate).
``plan/<UAV>.waypoints`` (QGC WPL 110): survey legs = NAV_WAYPOINT pairs between DO_SET_CAM_TRIGG_DIST(p1>0)
    and DO_SET_CAM_TRIGG_DIST(0) (the planner's sensor_on); the landing = the NAV_VTOL_LAND / NAV_LAND item.
``plan/mission_summary.json``, ``scenario.json`` (plan wind, sim wind), ``wind_readback.json`` (SIM_WIND_* as
    the FC read them back), ``release_schedule.json``, ``validation.json`` (mapping wind gate), ``report.json``
    (planner crab-margin lines), ``survey_knobs.json`` (.BIN paths), ``s1_analysis*/s1_report.json`` (photo counts;
    reused, never recomputed), ``flight_summary.json`` (old report; used only as a cross-check of my numbers).
Optional ``.BIN`` (survey_knobs.json["sitl_logs"] or ``--bin UAV=PATH``): ATT.Yaw (true heading => crab),
    XKF2 VWN/VWE (EKF wind), BAT (energy), PARM (SIM_WIND_*).  A BIN is only trusted when its POS track matches
    flight.csv (identity check); otherwise it is ignored and the report says so.

Wind estimators (all report standard errors and an identifiability verdict; nothing is guessed)
------------------------------------------------------------------------------------------------
* ``speed_circle``  ground velocity g = air velocity + w with |air| = Va unknown but constant per aircraft:
  least squares on |g_i - w| = Va_k.  Needs ground-velocity directions that are not collinear.  With only
  reciprocal (E-W) strips the along-axis component is found and the cross component is NOT identifiable; the tool
  says so instead of returning a number.  The airspeed column is deliberately NOT used as Va: the SITL pitot
  reads ~2 % low (density corrected twice, see memory sitl-airspeed-double-density), which would bias the
  along-track wind by +/-0.5 m/s with opposite sign on reciprocal legs.
* ``heading``  (needs BIN ATT.Yaw) g_i = w + Va_k * (sin psi_i, cos psi_i): works on strips alone and also
  resolves the sign of the cross component.
* FC's own estimates: WIND columns of flight.csv, and XKF2 VWN/VWE from the BIN.

Wind conventions: (speed, dir_from) with dir_from the meteorological "coming from" bearing (270 = westerly, blows
toward east) - the same as hv_dccpp ``dir_from_deg`` and ArduPilot ``SIM_WIND_DIR``.

Every number in the JSON is either MEASURED (from the files above) or tagged ``model`` (computed from a wind model
and the plan).  The markdown labels the difference.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")     # analysis tools must not write data/logs
os.environ.setdefault("MAVLINK20", "1")                # reading a BIN only; never lock pymavlink to v1

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
FORMAT = "aeroplan-hv-sitl-wind-check/1"

CMD_NAV_WAYPOINT = 16
CMD_NAV_LAND = 21
CMD_NAV_SPLINE = 82
CMD_NAV_VTOL_LAND = 85
CMD_DO_CHANGE_SPEED = 178
CMD_DO_SET_CAM_TRIGG_DIST = 206
LEG_NAV_CMDS = (CMD_NAV_WAYPOINT, CMD_NAV_SPLINE)
LAND_CMDS = (CMD_NAV_LAND, CMD_NAV_VTOL_LAND)

# ── thresholds (all reported in the output so a reader can see what "pass" meant) ─────────────────────────
TRIM_START_M = 150.0          # settle distance after the leg start before samples count as steady
TRIM_END_M = 100.0            # turn-anticipation distance before the leg end
TRIM_MAX_FRAC = 0.2           # never trim more than this fraction of a leg (short legs)
XTE_MAX_M = 30.0              # cross-track gate for "steady straight flight on the planned leg"
MIN_LEG_SAMPLES = 5
MIN_FIT_SAMPLES = 8
CRUISE_AIRSPEED_TOL_MPS = 1.0  # cruise gate: |airspeed_col - Va_ref| (excludes transition / landing decel)
CRUISE_MIN_ALT_M = 30.0
NOISE_FLOOR_MPS = 0.3          # velocity-noise + airspeed-model-error allowance used in the covariance (real cruise residual rms is 0.1-0.16)
IDENT_SE_MAX_MPS = 0.5         # a wind axis is "identified" when its standard error is below this
WIND_AGREE_ABS_MPS = 0.5       # two winds agree when |dw| <= max(abs, rel * |w_ref|)
WIND_AGREE_REL = 0.15
SPEED_RATIO_RANGE = (0.9, 1.1)  # median |v_from_positions| / groundspeed_col; outside => clock suspect
BIN_MATCH_MEDIAN_M = 15.0
CLASS_COS = 0.5               # tail/head/cross split: |cos(angle between track and wind)| >= 0.5
CALM_MPS = 0.3                # below this a wind has no meaningful direction ("calm")
MIN_G_MINOR_MPS = 1.0         # speed_circle: std of the ground velocity across its minor axis must exceed this,
                              # else that axis is declared not identifiable whatever the standard error says
BATCH_GAP_S = 2.5             # central-difference velocity only across <= this span (sim seconds)


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  small maths
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def wrap180(a):
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


def circ_mean_deg(a) -> float:
    a = np.radians(np.asarray(a, dtype=float))
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(math.degrees(math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a)))) % 360.0)


def bearing_deg(dx_east, dy_north):
    return np.degrees(np.arctan2(dx_east, dy_north)) % 360.0


def wind_vec(speed: float, dir_from_deg: float) -> tuple[float, float]:
    """(east, north) velocity of the AIR for a meteorological (speed, dir_from)."""
    d = math.radians(dir_from_deg)
    return (-speed * math.sin(d), -speed * math.cos(d))


def wind_from_vec(we: float, wn: float) -> tuple[float, float]:
    sp = math.hypot(we, wn)
    if sp < 1e-9:
        return 0.0, float("nan")
    return sp, float(math.degrees(math.atan2(-we, -wn)) % 360.0)


def metres_per_degree(lat_deg: float) -> tuple[float, float]:
    """WGS84 series: (metres per degree latitude, per degree longitude)."""
    p = math.radians(lat_deg)
    return (111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p),
            111412.84 * math.cos(p) - 93.5 * math.cos(3 * p))


@dataclass(frozen=True)
class Frame:
    lat0: float
    lon0: float

    def xy(self, lat, lon):
        m_lat, m_lon = metres_per_degree(self.lat0)
        return ((np.asarray(lon, float) - self.lon0) * m_lon, (np.asarray(lat, float) - self.lat0) * m_lat)


def leg_speed_model(length_m: float, track_deg: float, va: float, w_east: float, w_north: float) -> float:
    """Ground speed of an aircraft holding track ``track_deg`` with true airspeed ``va`` in wind (w_east, w_north).
    model: gs = w_along + sqrt(va^2 - w_cross^2).  NaN when the wind exceeds the airspeed cross-track."""
    b = math.radians(track_deg)
    w_along = w_east * math.sin(b) + w_north * math.cos(b)
    w_cross = w_east * math.cos(b) - w_north * math.sin(b)
    if not va > abs(w_cross):
        return float("nan")
    return w_along + math.sqrt(va * va - w_cross * w_cross)


def central_velocity(t, x, y, max_span_s: float = BATCH_GAP_S):
    """Central-difference ground velocity (east, north) m/s; NaN at the ends and across gaps > max_span_s."""
    n = len(t)
    vx = np.full(n, np.nan)
    vy = np.full(n, np.nan)
    if n >= 3:
        dt = t[2:] - t[:-2]
        ok = (dt > 0) & (dt <= max_span_s)
        with np.errstate(invalid="ignore", divide="ignore"):
            vx[1:-1] = np.where(ok, (x[2:] - x[:-2]) / np.where(ok, dt, 1.0), np.nan)
            vy[1:-1] = np.where(ok, (y[2:] - y[:-2]) / np.where(ok, dt, 1.0), np.nan)
    return vx, vy


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  readers
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
@dataclass
class Flight:
    uav: str
    t: np.ndarray
    lat: np.ndarray
    lon: np.ndarray
    alt: np.ndarray
    asp: np.ndarray
    gs: np.ndarray
    seq: np.ndarray
    mode: np.ndarray
    armed: np.ndarray
    wind_spd: np.ndarray
    wind_dir: np.ndarray
    has_wind_cols: bool
    info: dict


_REQ_COLS = ("t_s", "lat", "lon", "rel_alt_m", "airspeed", "groundspeed", "wp_seq", "mode", "armed")


def _f(x, default=float("nan")) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def read_flight_csv(path, uav: str = "") -> Flight:
    """Read a ``flight.csv``.  Duplicated timestamps (the poll loop re-reads the same GLOBAL_POSITION_INT) are
    dropped; a timestamp that goes BACKWARDS is an error (it means the file is not on the sim clock)."""
    path = Path(path)
    rows, n_raw, n_bad, n_dup, n_dup_conflict = [], 0, 0, 0, 0
    last_t = -math.inf
    last_row = None
    with path.open(newline="", encoding="utf-8") as fh:
        rd = csv.DictReader(fh)
        cols = list(rd.fieldnames or [])
        missing = [c for c in _REQ_COLS if c not in cols]
        if missing:
            raise ValueError(f"{path}: missing columns {missing} (have {cols})")
        has_wind = "wind_spd_est" in cols and "wind_dir_from_est" in cols
        for r in rd:
            n_raw += 1
            t, la, lo = _f(r["t_s"]), _f(r["lat"]), _f(r["lon"])
            if not (math.isfinite(t) and math.isfinite(la) and math.isfinite(lo)):
                n_bad += 1
                continue
            if t < last_t:
                raise ValueError(f"{path}: t_s goes backwards at row {n_raw} ({last_t} -> {t}); flight.csv must be on "
                                 f"the sim clock (GLOBAL_POSITION_INT.time_boot_ms), not wall clock")
            if t == last_t:
                n_dup += 1
                if last_row is not None and any(r[c] != last_row[c] for c in _REQ_COLS if c != "t_s"):
                    n_dup_conflict += 1
                continue
            last_t, last_row = t, r
            rows.append(r)
    if not rows:
        raise ValueError(f"{path}: no usable rows")

    def col(name, conv=_f):
        return np.array([conv(r.get(name)) for r in rows])

    def _seq(v):
        try:
            return int(float(v))
        except (TypeError, ValueError):
            return -1

    def _armed(v):
        return str(v).strip().lower() in ("true", "1")

    fl = Flight(
        uav=uav or path.parent.name, t=col("t_s"), lat=col("lat"), lon=col("lon"), alt=col("rel_alt_m"),
        asp=col("airspeed"), gs=col("groundspeed"), seq=np.array([_seq(r["wp_seq"]) for r in rows], dtype=int),
        mode=np.array([str(r["mode"]).strip() for r in rows]),
        armed=np.array([_armed(r["armed"]) for r in rows], dtype=bool),
        wind_spd=col("wind_spd_est") if has_wind else np.full(len(rows), np.nan),
        wind_dir=col("wind_dir_from_est") if has_wind else np.full(len(rows), np.nan),
        has_wind_cols=has_wind,
        info={"n_raw": n_raw, "n_used": len(rows), "n_bad": n_bad, "n_dup": n_dup, "n_dup_conflict": n_dup_conflict,
              "t_first": float(rows and _f(rows[0]["t_s"])), "t_last": float(_f(rows[-1]["t_s"]))})
    return fl


@dataclass
class WP:
    seq: int
    frame: int
    cmd: int
    p1: float
    p2: float
    p3: float
    p4: float
    lat: float
    lon: float
    alt: float


def read_waypoints(path) -> list[WP]:
    """QGC WPL 110 (tab/space separated: seq current frame cmd p1 p2 p3 p4 lat lon alt autocontinue)."""
    out = []
    for ln in Path(path).read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if not ln or ln.startswith("#") or ln.upper().startswith("QGC"):
            continue
        f = ln.split()
        if len(f) < 11:
            continue
        out.append(WP(seq=int(f[0]), frame=int(f[2]), cmd=int(f[3]), p1=_f(f[4]), p2=_f(f[5]), p3=_f(f[6]),
                      p4=_f(f[7]), lat=_f(f[8]), lon=_f(f[9]), alt=_f(f[10])))
    return out


@dataclass
class Leg:
    idx: int
    strip: int
    start_seq: int
    end_seq: int
    on_seq: int
    off_seq: int | None
    lat0: float
    lon0: float
    lat1: float
    lon1: float
    trigger_dist_m: float


def survey_legs(wps: list[WP]) -> list[Leg]:
    """Survey legs = every NAV leg flown while the camera trigger is ON (206 p1>0 ... 206 p1==0).

    The leg starts at the last NAV waypoint BEFORE the 206-on and ends at the NAV waypoint that follows it (a strip
    that has several NAV points inside gives several legs).  The leg's ``end_seq`` is the value of
    MISSION_CURRENT.seq while the aircraft flies it, so flight.csv ``wp_seq == end_seq`` selects its samples."""
    legs: list[Leg] = []
    last_nav: WP | None = None
    on = False
    on_seq = -1
    strip = -1
    trig = 0.0
    open_legs: list[int] = []
    for w in wps:
        if w.cmd in LEG_NAV_CMDS and (w.lat != 0.0 or w.lon != 0.0):
            if on and last_nav is not None:
                legs.append(Leg(idx=len(legs), strip=strip, start_seq=last_nav.seq, end_seq=w.seq, on_seq=on_seq,
                                off_seq=None, lat0=last_nav.lat, lon0=last_nav.lon, lat1=w.lat, lon1=w.lon,
                                trigger_dist_m=trig))
                open_legs.append(len(legs) - 1)
            last_nav = w
        elif w.cmd == CMD_DO_SET_CAM_TRIGG_DIST:
            if w.p1 > 0.0 and not on:
                on, on_seq, trig = True, w.seq, float(w.p1)
                strip += 1
            elif w.p1 > 0.0 and on:
                trig = float(w.p1)
            elif w.p1 == 0.0 and on:
                for i in open_legs:
                    legs[i].off_seq = w.seq
                open_legs = []
                on = False
    return legs


def plan_airspeed(wps: list[WP], param_file: Path | None, summary_params: dict | None) -> dict:
    """The commanded airspeed.  DO_CHANGE_SPEED p1: 0 = airspeed, 1 = groundspeed."""
    for w in wps:
        if w.cmd == CMD_DO_CHANGE_SPEED and w.p2 > 0:
            return {"value_mps": float(w.p2), "kind": "airspeed" if int(w.p1) == 0 else "groundspeed",
                    "source": f"DO_CHANGE_SPEED seq {w.seq}"}
    if summary_params and "AIRSPEED_CRUISE" in summary_params:
        return {"value_mps": float(summary_params["AIRSPEED_CRUISE"]), "kind": "airspeed",
                "source": "mission_summary params AIRSPEED_CRUISE"}
    if param_file is not None and param_file.exists():
        for ln in param_file.read_text(encoding="utf-8").splitlines():
            if ln.startswith("AIRSPEED_CRUISE,"):
                return {"value_mps": float(ln.split(",")[1]), "kind": "airspeed", "source": f"{param_file.name}"}
    return {"value_mps": None, "kind": None, "source": None}


def _read_json(p: Path):
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


@dataclass
class Trial:
    dir: Path
    name: str
    uavs: list[str]
    flights: dict
    waypoints: dict
    scenario: dict
    wind_readback: dict | None
    summary: dict | None
    release: dict | None
    report: dict | None
    validation: dict | None
    knobs: dict | None
    flight_summary: list | None
    sitl_cmdlines: str | None
    notes: list


def load_trial(trial_dir) -> Trial:
    d = Path(trial_dir)
    if not d.is_dir():
        raise SystemExit(f"trial directory {d} not found")
    notes: list[str] = []
    uavs = sorted(p.parent.name for p in d.glob("*/flight.csv"))
    if not uavs:
        raise SystemExit(f"{d}: no <UAV>/flight.csv found (is this a hv_sitl_run.py fly output?)")
    flights, wps = {}, {}
    for u in list(uavs):
        flights[u] = read_flight_csv(d / u / "flight.csv", u)
        wp = d / "plan" / f"{u}.waypoints"
        if wp.exists():
            wps[u] = read_waypoints(wp)
        else:
            notes.append(f"{u}: plan/{u}.waypoints missing - leg, landing-site and commanded-airspeed analysis skipped")
            wps[u] = []
    cmd = d / "sitl_cmdlines.txt"
    return Trial(dir=d, name=d.name, uavs=uavs, flights=flights, waypoints=wps,
                 scenario=_read_json(d / "scenario.json") or {}, wind_readback=_read_json(d / "wind_readback.json"),
                 summary=_read_json(d / "plan" / "mission_summary.json"), release=_read_json(d / "release_schedule.json"),
                 report=_read_json(d / "report.json"), validation=_read_json(d / "validation.json"),
                 knobs=_read_json(d / "survey_knobs.json"), flight_summary=_read_json(d / "flight_summary.json"),
                 sitl_cmdlines=cmd.read_text(encoding="utf-8") if cmd.exists() else None, notes=notes)


def resolve_winds(tr: Trial) -> tuple[dict | None, dict | None]:
    """(plan_wind, sim_wind) as dicts {speed_mps, dir_from_deg, source}.  sim_wind None = not recorded."""
    plan = None
    w = tr.scenario.get("wind")
    if w is None and tr.wind_readback:
        w = tr.wind_readback.get("plan_wind")
    if w is not None and len(w) == 2:
        plan = {"speed_mps": float(w[0]), "dir_from_deg": float(w[1]), "source": "scenario.json wind (planner input)"}
    sim = None
    rb = tr.wind_readback
    if rb and rb.get("rows"):
        def vals(name):
            return [r["actual"] for r in rb["rows"] if r.get("param") == name and r.get("actual") is not None]
        spd, dr = vals("SIM_WIND_SPD"), vals("SIM_WIND_DIR")
        if spd and dr:
            sim = {"speed_mps": float(np.median(spd)), "dir_from_deg": float(np.median(dr)),
                   "source": "wind_readback.json (SIM_WIND_SPD/DIR read back from the FCs)",
                   "readback_all_ok": all(r.get("ok") for r in rb["rows"]),
                   "turb": float(np.median(vals("SIM_WIND_TURB"))) if vals("SIM_WIND_TURB") else None,
                   "profile_T": float(np.median(vals("SIM_WIND_T"))) if vals("SIM_WIND_T") else None}
    if sim is None and isinstance(tr.scenario.get("sim_wind"), dict):
        s = tr.scenario["sim_wind"]
        sim = {"speed_mps": float(s.get("spd_mps", 0.0)), "dir_from_deg": float(s.get("dir_from_deg", 0.0)),
               "source": "scenario.json sim_wind (requested, not read back)"}
    return plan, sim


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  optional .BIN
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def read_bin(path) -> dict:
    """Extract the few series the wind check needs from an ArduPilot DataFlash .BIN (sim clock TimeUS)."""
    from pymavlink import DFReader      # lazy: only when a BIN is really read
    rd = DFReader.DFReader_binary(str(path), zero_time_base=False)
    want = ["ATT", "POS", "XKF2", "NKF2", "BAT", "PARM"]
    att, pos, xkf2, nkf2, bat = [], [], [], [], []
    parm: dict[str, float] = {}
    while True:
        m = rd.recv_match(type=want)
        if m is None:
            break
        ty = m.get_type()
        if ty == "ATT":
            att.append((m.TimeUS, m.Roll, m.Pitch, m.Yaw))
        elif ty == "POS":
            pos.append((m.TimeUS, m.Lat, m.Lng))
        elif ty == "XKF2":
            xkf2.append((m.TimeUS, m.C, m.VWN, m.VWE))
        elif ty == "NKF2":
            nkf2.append((m.TimeUS, m.C, m.VWN, m.VWE))
        elif ty == "BAT":
            if int(m.Inst) == 0:
                bat.append((m.TimeUS, m.Volt, m.Curr, m.CurrTot, m.EnrgTot))
        elif ty == "PARM":
            if m.Name not in parm:               # first value = the value at boot / take-off
                parm[m.Name] = float(m.Value)
    try:
        rd.data_map.close()
        rd.filehandle.close()
    except Exception:       # noqa: BLE001
        pass

    def arr(rows, n):
        return np.array(rows, dtype=float).reshape(-1, n)
    a, p, x, k, b = arr(att, 4), arr(pos, 3), arr(xkf2, 4), arr(nkf2, 4), arr(bat, 5)
    ek = x if len(x) else k
    ek_src = "XKF2" if len(x) else ("NKF2" if len(k) else None)
    if len(ek):                                        # primary EKF core 0 only (both cores agree in SITL)
        c0 = ek[ek[:, 1] == 0]
        ek = c0 if len(c0) else ek
    return {"path": str(path), "att_t": a[:, 0] / 1e6, "att_roll": a[:, 1], "att_pitch": a[:, 2], "att_yaw": a[:, 3],
            "pos_t": p[:, 0] / 1e6, "pos_lat": p[:, 1], "pos_lon": p[:, 2],
            "ekf_t": ek[:, 0] / 1e6 if len(ek) else np.array([]), "ekf_vwn": ek[:, 2] if len(ek) else np.array([]),
            "ekf_vwe": ek[:, 3] if len(ek) else np.array([]), "ekf_src": ek_src,
            "bat_t": b[:, 0] / 1e6, "bat_volt": b[:, 1], "bat_curr": b[:, 2], "bat_currtot_mah": b[:, 3],
            "bat_energy_wh": b[:, 4], "params": parm}


def yaw_at(series: dict, t) -> np.ndarray:
    """ATT.Yaw (deg) interpolated at times ``t`` (unwrapped so 359 -> 1 does not interpolate through 180)."""
    if len(series["att_t"]) < 2:
        return np.full(len(t), np.nan)
    u = np.unwrap(np.radians(series["att_yaw"]))
    y = np.interp(t, series["att_t"], u, left=np.nan, right=np.nan)
    return np.degrees(y) % 360.0


def bin_identity(fl: Flight, series: dict, frame: Frame) -> dict:
    """Does this BIN's POS track match flight.csv?  Guards against a stale/wrong BIN path."""
    sel = (fl.mode == "AUTO") & fl.armed & (fl.alt > 20.0)
    t = fl.t[sel]
    if t.size < 20 or len(series["pos_t"]) < 20:
        return {"ok": False, "reason": "not enough airborne samples to compare", "median_m": None}
    la = np.interp(t, series["pos_t"], series["pos_lat"], left=np.nan, right=np.nan)
    lo = np.interp(t, series["pos_t"], series["pos_lon"], left=np.nan, right=np.nan)
    x, y = frame.xy(fl.lat[sel], fl.lon[sel])
    bx, by = frame.xy(la, lo)
    d = np.hypot(x - bx, y - by)
    d = d[np.isfinite(d)]
    if d.size < 20:
        return {"ok": False, "reason": "BIN time range does not cover the flight", "median_m": None}
    med = float(np.median(d))
    return {"ok": med <= BIN_MATCH_MEDIAN_M, "median_m": med, "p95_m": float(np.percentile(d, 95)),
            "n": int(d.size), "reason": None if med <= BIN_MATCH_MEDIAN_M else
            f"median position difference {med:.1f} m > {BIN_MATCH_MEDIAN_M} m: this BIN is not this flight"}


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  wind estimators
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def _cov_from_jacobian(J: np.ndarray, sigma2: float) -> np.ndarray:
    """sigma2 * (J^T J)^-1 with null directions given (practically) infinite variance instead of zero."""
    lam, V = np.linalg.eigh(J.T @ J)
    lam_c = np.maximum(lam, 1e-12 * max(float(lam.max()), 1e-300))
    return (V / lam_c) @ V.T * sigma2


def _classify_wind(w: np.ndarray, Cw: np.ndarray, se_max: float, forced_dir=None) -> dict:
    """Identifiability verdict from the 2x2 covariance of the wind (east, north).

    ``forced_dir`` (unit vector east,north): the axis closest to it is declared NOT identified regardless of its
    standard error (used when the data are geometrically degenerate, e.g. all ground velocities on one line)."""
    ev, V = np.linalg.eigh(Cw)
    se = np.sqrt(np.maximum(ev, 0.0))
    ident = [bool(s <= se_max) for s in se]
    why = ["" if ok else "standard error" for ok in ident]
    if forced_dir is not None:
        k = int(np.argmax([abs(float(V[:, i] @ forced_dir)) for i in range(2)]))
        ident[k], why[k] = False, "geometry"
    axes = []
    for i in range(2):
        v = V[:, i].copy()
        b = float(bearing_deg(v[0], v[1]))
        if b >= 180.0:
            v, b = -v, b - 180.0
        axes.append({"axis_bearing_deg": b, "component_mps": float(w @ v), "se_mps": float(se[i]),
                     "identified": ident[i], "why_not": why[i] or None})
    if all(ident):
        status = "ok"
    elif any(ident):
        status = "partial"
    else:
        status = "unidentifiable"
    out = {"status": status, "axes": axes, "se_max_mps": se_max}
    if status == "ok":
        sp, dr = wind_from_vec(float(w[0]), float(w[1]))
        out.update(wind_east_mps=float(w[0]), wind_north_mps=float(w[1]), speed_mps=sp, dir_from_deg=dr,
                   se_east_mps=float(math.sqrt(max(Cw[0, 0], 0.0))), se_north_mps=float(math.sqrt(max(Cw[1, 1], 0.0))))
    else:
        out.update(wind_east_mps=None, wind_north_mps=None, speed_mps=None, dir_from_deg=None,
                   se_east_mps=None, se_north_mps=None)

        def lab(a):
            return f"component along bearing {a['axis_bearing_deg']:.0f}/{(a['axis_bearing_deg'] + 180) % 360:.0f}"
        good = [f"{lab(a)} = {a['component_mps']:+.2f} +/- {a['se_mps']:.2f} m/s (identified)" for a in axes if a["identified"]]
        bad = []
        for a in axes:
            if a["identified"]:
                continue
            if a["why_not"] == "geometry":
                bad.append(f"{lab(a)} NOT identified (ground-velocity directions are collinear: single heading or "
                           f"reciprocal strips only, so it cannot be separated from the airspeed)")
            else:
                se_t = f"{a['se_mps']:.2f}" if a["se_mps"] < 1e3 else ">1000"
                bad.append(f"{lab(a)} NOT identified (standard error {se_t} m/s > {se_max:g})")
        out["message"] = "wind NOT fully identifiable from this data: " + "; ".join(good + bad) + "; no value is reported for it"
    return out


def fit_speed_circle(groups: dict, *, noise_floor=NOISE_FLOOR_MPS, se_max=IDENT_SE_MAX_MPS) -> dict:
    """Least-squares wind from ground velocities only.  ``groups[k] = (gx, gy)`` (east/north m/s).

    Model: |g_i - w| = Va_k (one unknown constant true airspeed per group).  Algebraic start
    (|g|^2 = 2 g.w + c_k, linear) then Gauss-Newton on the true residual."""
    keys = [k for k, (gx, gy) in groups.items() if len(gx) >= MIN_FIT_SAMPLES]
    base = {"method": "speed_circle", "groups": keys}
    if not keys:
        return {**base, "status": "insufficient", "n": 0,
                "message": f"fewer than {MIN_FIT_SAMPLES} usable samples in every group"}
    gx = np.concatenate([np.asarray(groups[k][0], float) for k in keys])
    gy = np.concatenate([np.asarray(groups[k][1], float) for k in keys])
    gi = np.concatenate([np.full(len(groups[k][0]), i) for i, k in enumerate(keys)])
    moving = np.hypot(gx, gy) > 3.0                    # a direction needs a moving aircraft
    gx, gy, gi = gx[moving], gy[moving], gi[moving]
    K, n = len(keys), len(gx)
    if n < MIN_FIT_SAMPLES:
        return {**base, "status": "insufficient", "n": int(n), "message": "too few moving samples"}
    onehot = np.eye(K)[gi]
    A = np.column_stack([2 * gx, 2 * gy, onehot])
    sol = np.linalg.lstsq(A, gx ** 2 + gy ** 2, rcond=None)[0]
    w = sol[:2].copy()
    va = np.sqrt(np.maximum(sol[2:] + w @ w, 1.0))
    p = np.concatenate([w, va])
    for _ in range(40):
        dx, dy = gx - p[0], gy - p[1]
        rho = np.maximum(np.hypot(dx, dy), 1e-9)
        r = rho - p[2:][gi]
        J = np.column_stack([-dx / rho, -dy / rho, -onehot])
        step = np.linalg.lstsq(J, -r, rcond=None)[0]
        p = p + step
        if float(np.max(np.abs(step))) < 1e-9:
            break
    dx, dy = gx - p[0], gy - p[1]
    rho = np.maximum(np.hypot(dx, dy), 1e-9)
    r = rho - p[2:][gi]
    J = np.column_stack([-dx / rho, -dy / rho, -onehot])
    dof = max(n - (2 + K), 1)
    rms = float(math.sqrt(float(r @ r) / n))
    sigma2 = max(float(r @ r) / dof, noise_floor ** 2)
    # Identifiability and standard errors come from the LINEARISED design (unit ground-velocity direction u_i:
    # gs_i ~ u_i . w + Va_k to first order in w/Va), NOT from the Jacobian at the solution: when the geometry is
    # degenerate the nonlinear fit can collapse to Va ~ 0 where its own Jacobian looks deceptively well conditioned.
    spd = np.hypot(gx, gy)
    A_lin = np.column_stack([gx / spd, gy / spd, onehot])
    cov = _cov_from_jacobian(A_lin, sigma2)
    out = {**base, "n": int(n), "rms_resid_mps": rms,
           "va_mps": {k: float(p[2 + i]) for i, k in enumerate(keys)},
           "va_se_mps": {k: float(math.sqrt(max(cov[2 + i, 2 + i], 0.0))) for i, k in enumerate(keys)}}
    if float(np.min(p[2:])) < 0.3 * float(np.median(spd)):
        out.update(status="unidentifiable", wind_east_mps=None, wind_north_mps=None, speed_mps=None,
                   dir_from_deg=None, se_east_mps=None, se_north_mps=None, va_mps={k: None for k in keys},
                   message="the fit collapsed to an implausible airspeed (< 30 % of the ground speed): the ground-velocity "
                           "directions do not constrain the wind; no value is reported")
        return out
    # geometric degeneracy: ground velocities (nearly) on one line => the cross axis is not observable
    gc = np.cov(np.vstack([gx, gy])) if n > 2 else np.zeros((2, 2))
    gev, gV = np.linalg.eigh(gc)
    minor_sigma = float(math.sqrt(max(gev[0], 0.0)))
    out["g_minor_axis_std_mps"] = minor_sigma
    out["g_major_axis_std_mps"] = float(math.sqrt(max(gev[1], 0.0)))
    forced = gV[:, 0] if minor_sigma < MIN_G_MINOR_MPS else None
    out.update(_classify_wind(p[:2], cov[:2, :2], se_max, forced_dir=forced))
    if forced is not None:
        out["degenerate_geometry"] = (f"ground-velocity std across its minor axis is {minor_sigma:.2f} m/s < "
                                      f"{MIN_G_MINOR_MPS:g} m/s (single heading or reciprocal strips only)")
    return out


def fit_heading(groups: dict, *, noise_floor=NOISE_FLOOR_MPS, se_max=IDENT_SE_MAX_MPS) -> dict:
    """Least-squares wind from ground velocity AND true heading psi (BIN ATT.Yaw):
    g_i = w + Va_k (sin psi_i, cos psi_i).  ``groups[k] = (gx, gy, psi_deg)``."""
    keys = [k for k, v in groups.items() if len(v[0]) >= MIN_FIT_SAMPLES]
    base = {"method": "heading", "groups": keys}
    if not keys:
        return {**base, "status": "insufficient", "n": 0,
                "message": f"fewer than {MIN_FIT_SAMPLES} usable samples in every group"}
    gx = np.concatenate([np.asarray(groups[k][0], float) for k in keys])
    gy = np.concatenate([np.asarray(groups[k][1], float) for k in keys])
    ps = np.radians(np.concatenate([np.asarray(groups[k][2], float) for k in keys]))
    gi = np.concatenate([np.full(len(groups[k][0]), i) for i, k in enumerate(keys)])
    K, n = len(keys), len(gx)
    onehot = np.eye(K)[gi]
    rows_x = np.column_stack([np.ones(n), np.zeros(n), onehot * np.sin(ps)[:, None]])
    rows_y = np.column_stack([np.zeros(n), np.ones(n), onehot * np.cos(ps)[:, None]])
    J = np.vstack([rows_x, rows_y])
    b = np.concatenate([gx, gy])
    p = np.linalg.lstsq(J, b, rcond=None)[0]
    r = b - J @ p
    dof = max(2 * n - (2 + K), 1)
    sigma2 = max(float(r @ r) / dof, noise_floor ** 2)
    cov = _cov_from_jacobian(J, sigma2)
    out = {**base, "n": int(n), "rms_resid_mps": float(math.sqrt(float(r @ r) / (2 * n))),
           "va_mps": {k: float(p[2 + i]) for i, k in enumerate(keys)},
           "va_se_mps": {k: float(math.sqrt(max(cov[2 + i, 2 + i], 0.0))) for i, k in enumerate(keys)}}
    out.update(_classify_wind(p[:2], cov[:2, :2], se_max))
    return out


def fc_wind_from_columns(fl: Flight, mask: np.ndarray) -> dict | None:
    """Vector mean of the FC's WIND estimate (flight.csv columns) over ``mask``."""
    if not fl.has_wind_cols:
        return None
    m = mask & np.isfinite(fl.wind_spd) & np.isfinite(fl.wind_dir)
    if int(m.sum()) < MIN_FIT_SAMPLES:
        return None
    d = np.radians(fl.wind_dir[m])
    we, wn = -fl.wind_spd[m] * np.sin(d), -fl.wind_spd[m] * np.cos(d)
    sp, dr = wind_from_vec(float(we.mean()), float(wn.mean()))
    return {"n": int(m.sum()), "wind_east_mps": float(we.mean()), "wind_north_mps": float(wn.mean()),
            "speed_mps": sp, "dir_from_deg": dr, "std_east_mps": float(we.std()), "std_north_mps": float(wn.std())}


def fc_wind_from_ekf(series: dict, t: np.ndarray) -> dict | None:
    """XKF2/NKF2 VWN/VWE (EKF wind-velocity states) interpolated at the cruise sample times ``t``."""
    if series is None or len(series["ekf_t"]) < 2 or t.size < MIN_FIT_SAMPLES:
        return None
    wn = np.interp(t, series["ekf_t"], series["ekf_vwn"], left=np.nan, right=np.nan)
    we = np.interp(t, series["ekf_t"], series["ekf_vwe"], left=np.nan, right=np.nan)
    ok = np.isfinite(wn) & np.isfinite(we)
    if int(ok.sum()) < MIN_FIT_SAMPLES:
        return None
    sp, dr = wind_from_vec(float(we[ok].mean()), float(wn[ok].mean()))
    return {"n": int(ok.sum()), "source": series["ekf_src"], "wind_east_mps": float(we[ok].mean()),
            "wind_north_mps": float(wn[ok].mean()), "speed_mps": sp, "dir_from_deg": dr,
            "std_east_mps": float(we[ok].std()), "std_north_mps": float(wn[ok].std())}


def wind_diff(a: dict | None, b: dict | None) -> dict | None:
    """Vector difference of two wind dicts that carry wind_east/north (or speed+dir_from)."""
    def vec(w):
        if w is None:
            return None
        if w.get("wind_east_mps") is not None:
            return (w["wind_east_mps"], w["wind_north_mps"])
        if w.get("speed_mps") is not None and w.get("dir_from_deg") is not None:
            return wind_vec(w["speed_mps"], w["dir_from_deg"])
        return None
    va, vb = vec(a), vec(b)
    if va is None or vb is None:
        return None
    de, dn = va[0] - vb[0], va[1] - vb[1]
    return {"d_east_mps": de, "d_north_mps": dn, "d_vec_mps": math.hypot(de, dn),
            "d_speed_mps": math.hypot(*va) - math.hypot(*vb)}


def winds_agree(a: dict | None, b: dict | None) -> bool | None:
    d = wind_diff(a, b)
    if d is None:
        return None
    ref = b
    vb = (ref.get("wind_east_mps"), ref.get("wind_north_mps")) if ref.get("wind_east_mps") is not None \
        else wind_vec(ref["speed_mps"], ref["dir_from_deg"])
    tol = max(WIND_AGREE_ABS_MPS, WIND_AGREE_REL * math.hypot(*vb))
    return bool(d["d_vec_mps"] <= tol)


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  per-aircraft analysis
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def _leg_geom(leg: Leg, frame: Frame) -> dict:
    x0, y0 = frame.xy(leg.lat0, leg.lon0)
    x1, y1 = frame.xy(leg.lat1, leg.lon1)
    x0, y0, x1, y1 = float(x0), float(y0), float(x1), float(y1)
    L = math.hypot(x1 - x0, y1 - y0)
    ux, uy = ((x1 - x0) / L, (y1 - y0) / L) if L > 0 else (0.0, 1.0)
    return {"x0": x0, "y0": y0, "L": L, "ux": ux, "uy": uy, "bearing": float(bearing_deg(ux, uy))}


def _along_cross(x, y, g: dict):
    dx, dy = x - g["x0"], y - g["y0"]
    return dx * g["ux"] + dy * g["uy"], dx * g["uy"] - dy * g["ux"]       # cross > 0 = right of travel


def _upward_crossing(t, s, level, lo, hi):
    """Time of the last upward crossing of ``level`` in index window [lo, hi) (linear interpolation)."""
    best = None
    for i in range(lo, min(hi, len(s) - 1)):
        if s[i] < level <= s[i + 1] and s[i + 1] > s[i]:
            best = float(t[i] + (level - s[i]) / (s[i + 1] - s[i]) * (t[i + 1] - t[i]))
    return best


def analyse_legs(fl: Flight, legs: list[Leg], frame: Frame, vx, vy, x, y, *, trim_start_m, trim_end_m, xte_max_m):
    """Select steady samples on each survey leg and compute the MEASURED per-leg quantities."""
    out = []
    auto = (fl.mode == "AUTO") & fl.armed
    for leg in legs:
        g = _leg_geom(leg, frame)
        L = g["L"]
        s, e = _along_cross(x, y, g)
        in_leg = auto & (fl.seq == leg.end_seq)
        ts, te = min(trim_start_m, TRIM_MAX_FRAC * L), min(trim_end_m, TRIM_MAX_FRAC * L)
        steady = in_leg & (s >= ts) & (s <= L - te) & (np.abs(e) <= xte_max_m) & np.isfinite(vx)
        idx = np.flatnonzero(steady)
        row = {"uav": fl.uav, "leg": leg.idx, "strip": leg.strip, "start_seq": leg.start_seq, "end_seq": leg.end_seq,
               "bearing_deg": g["bearing"], "length_m": L, "trigger_dist_m": leg.trigger_dist_m,
               "n_in_leg": int(in_leg.sum()), "n_steady": int(idx.size), "steady_idx": idx}
        if idx.size < MIN_LEG_SAMPLES:
            row.update(status="insufficient_samples", gs_col_mps=None, gs_pos_mps=None, airspeed_col_mps=None,
                       course_deg=None, track_err_deg=None, xte_rms_m=None, flown_time_s=None)
            out.append(row)
            continue
        vpos = np.hypot(vx[idx], vy[idx])
        course = circ_mean_deg(bearing_deg(vx[idx], vy[idx]))
        # flown leg time: s(t) crossing 0 and L (interpolated; the legs are entered/left mid-sample)
        w = np.flatnonzero(in_leg)
        lo, hi = max(int(w[0]) - 3, 0), min(int(w[-1]) + 4, len(s))
        t0 = _upward_crossing(fl.t, s, 0.0, lo, int(idx[0]) + 1)
        t1 = _upward_crossing(fl.t, s, L, int(idx[-1]), hi)
        row.update(status="ok", gs_col_mps=float(np.mean(fl.gs[idx])), gs_col_std_mps=float(np.std(fl.gs[idx])),
                   gs_pos_mps=float(np.mean(vpos)), airspeed_col_mps=float(np.mean(fl.asp[idx])),
                   course_deg=course, track_err_deg=float(wrap180(course - g["bearing"])),
                   xte_rms_m=float(np.sqrt(np.mean(e[idx] ** 2))),
                   t_first_s=float(fl.t[idx[0]]), t_last_s=float(fl.t[idx[-1]]),
                   flown_time_s=(t1 - t0) if (t0 is not None and t1 is not None and t1 > t0) else None)
        out.append(row)
    return out


def touchdown_report_def(t, alt, armed) -> float:
    """hv_sitl_report._touchdown_s: first sample after climbing >20 m with alt<2 and disarmed (same definition so my
    numbers can be cross-checked against flight_summary.json)."""
    peak = 0.0
    for ts, a, ar in zip(t, alt, armed):
        peak = max(peak, a)
        if peak > 20.0 and a < 2.0 and not ar:
            return float(ts)
    return float("nan")


def clock_offset(tr: Trial) -> float | None:
    """hv_sitl_report._clock_offset: t_s(first AUTO) - actual_release_wall * speedup, median over aircraft."""
    rel = tr.release
    if not rel:
        return None
    sp = float(rel.get("speedup") or 1.0)
    act = rel.get("actual_release_wall_s") or {}
    offs = []
    for u in tr.uavs:
        if u not in act:
            continue
        auto = np.flatnonzero(tr.flights[u].mode == "AUTO")
        if auto.size:
            offs.append(float(tr.flights[u].t[auto[0]]) - float(act[u]) * sp)
    return float(np.median(offs)) if offs else None


def flown_km_all(fl: Flight, frame: Frame, sel=None) -> float:
    x, y = frame.xy(fl.lat, fl.lon)
    if sel is not None:
        x, y = x[sel], y[sel]
    return float(np.sum(np.hypot(np.diff(x), np.diff(y)))) / 1000.0


def landing_report(fl: Flight, wps: list[WP], frame: Frame, vx, vy, x, y, refs: dict) -> dict:
    """Landed? touchdown position vs the planned landing item, approach course vs wind."""
    out: dict = {"landed": False}
    peak = float(np.max(fl.alt)) if fl.alt.size else 0.0
    td_report = touchdown_report_def(fl.t, fl.alt, fl.armed)
    out["peak_alt_m"] = peak
    out["t_disarm_report_def_s"] = None if math.isnan(td_report) else td_report
    pk = int(np.argmax(fl.alt))
    after = np.flatnonzero((np.arange(len(fl.t)) >= pk) & (fl.alt <= 0.5))
    land_wp = next((w for w in reversed(wps) if w.cmd in LAND_CMDS), None)
    out["planned_landing"] = None if land_wp is None else {"seq": land_wp.seq, "cmd": land_wp.cmd, "lat": land_wp.lat,
                                                          "lon": land_wp.lon}
    out["landed"] = bool(peak > 20.0 and not math.isnan(td_report)) and bool(after.size)
    if after.size:
        i = int(after[0])
        out["t_touch_s"] = float(fl.t[i])
        out["touchdown_lat"], out["touchdown_lon"] = float(fl.lat[i]), float(fl.lon[i])
        if land_wp is not None:
            px, py = frame.xy(land_wp.lat, land_wp.lon)
            dxe, dyn = float(x[i] - px), float(y[i] - py)
            out["touchdown_err_east_m"], out["touchdown_err_north_m"] = dxe, dyn
            out["touchdown_err_m"] = math.hypot(dxe, dyn)
            hov = (fl.t >= fl.t[pk]) & (fl.alt > 5.0) & (fl.alt < 60.0) & (fl.gs < 6.0)
            if int(hov.sum()) >= 5:
                hx, hy = x[hov] - px, y[hov] - py
                out["vtol_descent_mean_offset_east_m"] = float(hx.mean())
                out["vtol_descent_mean_offset_north_m"] = float(hy.mean())
                out["vtol_descent_rms_offset_m"] = float(np.sqrt(np.mean(hx ** 2 + hy ** 2)))
                out["vtol_descent_n"] = int(hov.sum())
    # approach: fixed-wing samples flown toward the landing item (wp_seq == land seq), high enough, still fast
    if land_wp is not None:
        sel = ((fl.mode == "AUTO") & fl.armed & (fl.seq == land_wp.seq) & (fl.alt > 30.0) & (fl.gs >= 12.0)
               & np.isfinite(vx))
        sel &= np.isfinite(vy)
        if int(sel.sum()) >= 5:
            course = circ_mean_deg(bearing_deg(vx[sel], vy[sel]))
            nav = [w for w in wps if w.cmd in LEG_NAV_CMDS and (w.lat or w.lon) and w.seq < land_wp.seq]
            planned = None
            if nav:
                ax_, ay_ = frame.xy(nav[-1].lat, nav[-1].lon)
                bx_, by_ = frame.xy(land_wp.lat, land_wp.lon)
                planned = float(bearing_deg(float(bx_ - ax_), float(by_ - ay_)))
            ap = {"n": int(sel.sum()), "course_deg": course, "planned_bearing_deg": planned,
                  "gs_mean_mps": float(np.mean(fl.gs[sel])), "airspeed_col_mean_mps": float(np.mean(fl.asp[sel]))}
            rel = {}
            for name, wd in refs.items():
                if wd is None or wd.get("speed_mps") is None:
                    continue
                if wd["speed_mps"] < CALM_MPS:
                    rel[name] = {"wind_speed_mps": wd["speed_mps"], "class": "calm"}
                    continue
                if wd.get("dir_from_deg") is None:
                    continue
                ang = float(wrap180(wd["dir_from_deg"] - course))            # 0 = wind from straight ahead
                head = wd["speed_mps"] * math.cos(math.radians(ang))
                cls = "headwind" if math.cos(math.radians(ang)) >= CLASS_COS else \
                    ("tailwind" if math.cos(math.radians(ang)) <= -CLASS_COS else "crosswind")
                rel[name] = {"wind_speed_mps": wd["speed_mps"], "wind_from_deg": wd["dir_from_deg"],
                             "angle_wind_from_minus_course_deg": ang, "headwind_component_mps": head, "class": cls}
            ap["vs_wind"] = rel
            out["approach"] = ap
    return out


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  trial analysis
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def _fmt(v, nd=2, none="n/a"):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return none
    return f"{v:.{nd}f}"


def dirtxt(speed, dir_from, nd=0) -> str:
    """Direction text; a wind below CALM_MPS has no meaningful direction (the angle of noise is shown as nothing)."""
    if speed is None or dir_from is None or not math.isfinite(dir_from):
        return "n/a"
    if speed < CALM_MPS:
        return "calm"
    return f"{dir_from:.{nd}f}"


def wtxt(w) -> str:
    """'3 m/s from 270' / '0 m/s (calm, no direction)' for a wind dict."""
    if w is None:
        return "not recorded"
    return f"{w['speed_mps']:g} m/s {wfrom(w['speed_mps'], w['dir_from_deg'])}" + (" [inferred]" if w.get("inferred") else "")


def wfrom(speed, dir_from, nd=0) -> str:
    """'from 270' or '(calm, no direction)'."""
    t = dirtxt(speed, dir_from, nd)
    return "(calm, no direction)" if t == "calm" else ("n/a" if t == "n/a" else f"from {t}")


def _find_s1_report(d: Path, explicit: str | None):
    if explicit:
        p = Path(explicit)
        return (p, _read_json(p)) if p.exists() else (None, None)
    cands = [p for p in d.glob("s1_analysis*/s1_report.json")]
    if not cands:
        return None, None
    p = max(cands, key=lambda q: q.stat().st_mtime)
    return p, _read_json(p)


def analyze_trial(trial_dir, *, bins: dict | None = None, use_bin: bool = True, s1_report: str | None = None,
                  trim_start_m: float = TRIM_START_M, trim_end_m: float = TRIM_END_M,
                  xte_max_m: float = XTE_MAX_M, bin_series: dict | None = None) -> dict:
    """Run every check on one trial directory and return the report dict (JSON-able after ``_clean``).

    ``bin_series`` (tests): {uav: series-dict as returned by ``read_bin``} bypasses file reading."""
    tr = load_trial(trial_dir)
    notes = list(tr.notes)
    plan_wind, sim_wind = resolve_winds(tr)
    # frame: planning home of the first aircraft with a waypoint file (seq 0 = home)
    f0 = next((w for u in tr.uavs for w in tr.waypoints[u][:1] if (w.lat or w.lon)), None)
    if f0 is not None:
        frame = Frame(f0.lat, f0.lon)
    else:
        fl0 = tr.flights[tr.uavs[0]]
        frame = Frame(float(fl0.lat[0]), float(fl0.lon[0]))

    # ── BINs ───────────────────────────────────────────────────────────────────────────────────────────
    series: dict = {}
    bin_status: dict = {}
    if bin_series is not None:
        series = dict(bin_series)
        bin_status = {u: {"status": "injected"} for u in series}
    elif use_bin:
        logs = {u: r.get("bin") for u, r in (((tr.knobs or {}).get("sitl_logs")) or {}).items()}
        logs.update({u: p for u, p in (bins or {}).items()})
        for u in tr.uavs:
            p = logs.get(u)
            if not p:
                bin_status[u] = {"status": "none", "reason": "no BIN path recorded (survey_knobs.json) or given (--bin)"}
                continue
            if not Path(p).is_file():
                bin_status[u] = {"status": "missing", "path": str(p), "reason": "file does not exist"}
                continue
            try:
                s = read_bin(p)
            except Exception as exc:      # noqa: BLE001 - a bad BIN must not kill the CSV analysis
                bin_status[u] = {"status": "error", "path": str(p), "reason": f"{type(exc).__name__}: {exc}"}
                continue
            idt = bin_identity(tr.flights[u], s, frame)
            if not idt["ok"]:
                bin_status[u] = {"status": "rejected", "path": str(p), "identity": idt, "reason": idt["reason"]}
                continue
            series[u] = s
            bin_status[u] = {"status": "ok", "path": str(p), "identity": idt,
                             "params": {k: s["params"].get(k) for k in
                                        ("SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_TURB", "SIM_WIND_T", "SIM_WIND_T_ALT",
                                         "AIRSPEED_CRUISE", "MIN_GROUNDSPEED") if k in s["params"]}}
    else:
        bin_status = {u: {"status": "disabled"} for u in tr.uavs}

    # BIN parameter readback of the wind actually configured in the FC
    bin_sim = None
    ps = [(u, bin_status[u].get("params")) for u in tr.uavs if bin_status.get(u, {}).get("status") == "ok"]
    ps = [(u, p) for u, p in ps if p and "SIM_WIND_SPD" in p]
    if ps:
        bin_sim = {"speed_mps": float(np.median([p["SIM_WIND_SPD"] for _, p in ps])),
                   "dir_from_deg": float(np.median([p.get("SIM_WIND_DIR", float("nan")) for _, p in ps])),
                   "source": "BIN PARM (first logged value)", "uavs": [u for u, _ in ps]}

    cmd_has_wind_parm = None if tr.sitl_cmdlines is None else ("wind.parm" in tr.sitl_cmdlines)
    if sim_wind is None and bin_sim is not None:
        sim_wind = {"speed_mps": bin_sim["speed_mps"], "dir_from_deg": bin_sim["dir_from_deg"],
                    "source": "BIN PARM SIM_WIND_* (the artifact itself has no wind readback)"}
    elif sim_wind is None and cmd_has_wind_parm is False:
        sim_wind = {"speed_mps": 0.0, "dir_from_deg": 0.0, "inferred": True,
                    "source": "INFERRED: no wind.parm in the SITL --defaults chain (ArduPilot default SIM_WIND_SPD=0); "
                              "confirm with --bin"}

    # ── per-aircraft kinematics and legs ───────────────────────────────────────────────────────────────
    work: dict = {}
    for u in tr.uavs:
        fl = tr.flights[u]
        x, y = frame.xy(fl.lat, fl.lon)
        vx, vy = central_velocity(fl.t, x, y)
        legs = survey_legs(tr.waypoints[u])
        lres = analyse_legs(fl, legs, frame, vx, vy, x, y, trim_start_m=trim_start_m, trim_end_m=trim_end_m,
                            xte_max_m=xte_max_m)
        sidx = np.concatenate([r["steady_idx"] for r in lres]) if lres else np.array([], dtype=int)
        summ_u = ((tr.summary or {}).get("uavs") or {}).get(u, {})
        pa = plan_airspeed(tr.waypoints[u], tr.dir / "plan" / f"{u}.param", summ_u.get("params"))
        auto = (fl.mode == "AUTO") & fl.armed
        if sidx.size >= MIN_LEG_SAMPLES:
            va_ref = float(np.median(fl.asp[sidx]))
        else:
            m = auto & (fl.alt >= CRUISE_MIN_ALT_M) & (fl.gs >= 10.0)
            va_ref = float(np.median(fl.asp[m])) if m.sum() >= MIN_FIT_SAMPLES else float("nan")
        cruise = (auto & (fl.alt >= CRUISE_MIN_ALT_M) & (np.abs(fl.asp - va_ref) <= CRUISE_AIRSPEED_TOL_MPS)
                  & np.isfinite(vx) & np.isfinite(vy)) if math.isfinite(va_ref) else np.zeros(len(fl.t), bool)
        psi = yaw_at(series[u], fl.t) if u in series else None
        work[u] = dict(fl=fl, x=x, y=y, vx=vx, vy=vy, legs=legs, lres=lres, sidx=sidx, va_ref=va_ref, cruise=cruise,
                       psi=psi, plan_air=pa, summ=summ_u)

    # ── time axis sanity: |v from positions| / groundspeed column must be ~1 (it is 5 x off on a wall clock) ──
    ratios = []
    for u, wk in work.items():
        fl = wk["fl"]
        m = wk["cruise"] & (fl.gs > 10.0)
        if m.sum() >= MIN_FIT_SAMPLES:
            ratios.append(np.hypot(wk["vx"][m], wk["vy"][m]) / fl.gs[m])
    sr = float(np.median(np.concatenate(ratios))) if ratios else None
    dts = np.concatenate([np.diff(w["fl"].t) for w in work.values()])
    time_axis = {"source": "flight.csv t_s = GLOBAL_POSITION_INT.time_boot_ms/1000 (sim clock)",
                 "dt_median_s": float(np.median(dts)), "speed_ratio_pos_over_groundspeed_col": sr,
                 "ok": sr is not None and SPEED_RATIO_RANGE[0] <= sr <= SPEED_RATIO_RANGE[1],
                 "duplicates_removed": {u: work[u]["fl"].info["n_dup"] for u in work},
                 "duplicate_conflicts": {u: work[u]["fl"].info["n_dup_conflict"] for u in work}}
    if sr is not None and not time_axis["ok"]:
        notes.append(f"TIME AXIS SUSPECT: median |v from positions|/groundspeed = {sr:.2f} (expected ~1.00); "
                     f"every speed derived from t_s is unreliable")

    # ── wind estimates ─────────────────────────────────────────────────────────────────────────────────
    def groups_for(scope: str, with_psi: bool) -> dict:
        g = {}
        for u, wk in work.items():
            idx = wk["sidx"] if scope == "survey" else np.flatnonzero(wk["cruise"])
            idx = idx[np.isfinite(wk["vx"][idx])] if idx.size else idx
            if with_psi:
                if wk["psi"] is None:
                    continue
                idx = idx[np.isfinite(wk["psi"][idx])] if idx.size else idx
                g[u] = (wk["vx"][idx], wk["vy"][idx], wk["psi"][idx])
            else:
                g[u] = (wk["vx"][idx], wk["vy"][idx])
        return g

    est: dict = {"speed_circle": {}, "heading": {}}
    for scope in ("cruise", "survey"):
        est["speed_circle"][scope] = fit_speed_circle(groups_for(scope, False))
        gh = groups_for(scope, True)
        est["heading"][scope] = fit_heading(gh) if gh else {"method": "heading", "status": "unavailable",
                                                            "message": "no BIN ATT.Yaw (flight.csv has no heading column)"}
    # headline = the 'ok' estimate with the smallest standard error
    cand = []
    for meth in ("heading", "speed_circle"):
        for scope in ("survey", "cruise"):
            e = est[meth][scope]
            if e.get("status") == "ok":
                cand.append((max(e["se_east_mps"], e["se_north_mps"]), meth, scope, e))
    headline = None
    if cand:
        cand.sort(key=lambda c: c[0])
        _, meth, scope, e = cand[0]
        headline = {"method": meth, "scope": scope, **{k: e[k] for k in
                    ("wind_east_mps", "wind_north_mps", "speed_mps", "dir_from_deg", "se_east_mps", "se_north_mps")}}
    # FC's own estimates
    fc_csv, fc_ekf = {}, {}
    for u, wk in work.items():
        fl = wk["fl"]
        fc_csv[u] = fc_wind_from_columns(fl, wk["cruise"])
        fc_ekf[u] = fc_wind_from_ekf(series.get(u), fl.t[wk["cruise"]]) if u in series else None

    def pool(d):
        vals = [v for v in d.values() if v]
        if not vals:
            return None
        we = float(np.mean([v["wind_east_mps"] for v in vals]))
        wn = float(np.mean([v["wind_north_mps"] for v in vals]))
        sp, dr = wind_from_vec(we, wn)
        return {"wind_east_mps": we, "wind_north_mps": wn, "speed_mps": sp, "dir_from_deg": dr,
                "n_uavs": len(vals)}
    fc_csv_pool, fc_ekf_pool = pool(fc_csv), pool(fc_ekf)
    estimates = {"speed_circle": est["speed_circle"], "heading": est["heading"], "headline": headline,
                 "fc_wind_msg_columns": {"per_uav": fc_csv, "pooled": fc_csv_pool},
                 "fc_ekf_xkf2": {"per_uav": fc_ekf, "pooled": fc_ekf_pool}}

    # reference wind used for tail/head/cross classification and per-leg models
    sim_ok = sim_wind is not None
    if headline is not None:
        ref_name, ref = "estimated", {"speed_mps": headline["speed_mps"], "dir_from_deg": headline["dir_from_deg"],
                                      "wind_east_mps": headline["wind_east_mps"], "wind_north_mps": headline["wind_north_mps"]}
    elif sim_ok:
        ref_name, ref = "sim_wind", dict(sim_wind)
    elif plan_wind is not None:
        ref_name, ref = "plan_wind", dict(plan_wind)
    else:
        ref_name, ref = None, None

    def vec_of(w):
        if w is None:
            return None
        if w.get("wind_east_mps") is not None:
            return (w["wind_east_mps"], w["wind_north_mps"])
        return wind_vec(w["speed_mps"], w["dir_from_deg"])
    v_ref, v_sim, v_plan = vec_of(ref), vec_of(sim_wind), vec_of(plan_wind)
    v_est = vec_of(headline)

    # ── per-UAV: legs completed with models, reciprocal check, times, path, energy, landing ──────────────
    off = clock_offset(tr)
    uav_out: dict = {}
    s1p, s1 = _find_s1_report(tr.dir, s1_report)
    for u, wk in work.items():
        fl, lres, pa = wk["fl"], wk["lres"], wk["plan_air"]
        su = wk["summ"]
        va_cmd = pa["value_mps"]
        # true airspeed = fitted Va of the headline method, else of the other 'ok' fits, else leg mean
        tas, tas_src = None, None
        order = ([(headline["method"], headline["scope"])] if headline else []) + [
            ("speed_circle", "cruise"), ("speed_circle", "survey"), ("heading", "survey"), ("heading", "cruise")]
        for m_, sc_ in order:
            e = est[m_][sc_]
            if e.get("status") == "ok" and u in (e.get("va_mps") or {}):
                tas, tas_src = e["va_mps"][u], f"fitted constant airspeed ({m_}/{sc_})"
                break
        okl = [r for r in lres if r["status"] == "ok"]
        if tas is None and okl:
            tas = float(np.mean([r["gs_pos_mps"] for r in okl]))
            tas_src = "mean leg ground speed (fallback; biased if legs are not balanced)"
        for r in lres:
            if r["status"] != "ok":
                continue
            L, b = r["length_m"], r["bearing_deg"]
            r["tas_used_mps"] = tas
            def model_gs(v, va):
                return None if (v is None or va is None) else leg_speed_model(L, b, va, v[0], v[1])
            gm = {"est": model_gs(v_est, tas), "sim": model_gs(v_sim, tas), "plan": model_gs(v_plan, tas)}
            r["gs_model_mps"] = gm                                              # model: TAS_fit + wind
            if v_ref is not None:
                w_along = v_ref[0] * math.sin(math.radians(b)) + v_ref[1] * math.cos(math.radians(b))
                w_cross_r = v_ref[0] * math.cos(math.radians(b)) - v_ref[1] * math.sin(math.radians(b))
                spd = math.hypot(*v_ref)
                r["wind_along_ref_mps"], r["wind_cross_right_ref_mps"] = w_along, w_cross_r
                cosang = (w_along / spd) if spd > 0.3 else 0.0
                r["class_vs_ref"] = ("calm" if spd <= 0.3 else
                                     "tailwind" if cosang >= CLASS_COS else "headwind" if cosang <= -CLASS_COS else "crosswind")
                if tas and abs(w_cross_r) < tas:
                    r["crab_pred_deg"] = float(math.degrees(math.asin(w_cross_r / tas)))
            # plan-time model: commanded airspeed under the PLAN wind (the planner's own per-leg times are not
            # serialised in the artifact, so this is a model of what the plan assumed)
            r["plan_time_model_s"] = (L / leg_speed_model(L, b, va_cmd, v_plan[0], v_plan[1])
                                      if (va_cmd and v_plan is not None and
                                          math.isfinite(leg_speed_model(L, b, va_cmd, v_plan[0], v_plan[1]))) else None)
            gsim = gm["sim"]
            r["sim_time_model_s"] = L / gsim if gsim and math.isfinite(gsim) and gsim > 0 else None
            # measured heading / crab from the BIN
            if wk["psi"] is not None and u in series:
                m = (series[u]["att_t"] >= r["t_first_s"]) & (series[u]["att_t"] <= r["t_last_s"])
                if int(m.sum()) >= 3:
                    hd = circ_mean_deg(series[u]["att_yaw"][m])
                    r["heading_deg"] = hd
                    r["crab_deg"] = float(wrap180(r["course_deg"] - hd))
        # reciprocal-leg (upwind/downwind) check
        recip = reciprocal_check(okl, {"estimated": v_est, "sim": v_sim, "plan": v_plan})
        classes = {}
        for cl in ("tailwind", "headwind", "crosswind", "calm"):
            sel = [r for r in okl if r.get("class_vs_ref") == cl]
            if sel:
                classes[cl] = {"n_legs": len(sel), "gs_pos_mean_mps": float(np.mean([r["gs_pos_mps"] for r in sel])),
                               "gs_col_mean_mps": float(np.mean([r["gs_col_mps"] for r in sel]))}
        gs_rms = {}
        for nm in ("est", "sim", "plan"):
            res = [r["gs_pos_mps"] - r["gs_model_mps"][nm] for r in okl
                   if r.get("gs_model_mps") and r["gs_model_mps"].get(nm) is not None
                   and math.isfinite(r["gs_model_mps"][nm])]
            gs_rms[nm] = float(np.sqrt(np.mean(np.square(res)))) if res else None
        # flight times / path / energy
        auto_i = np.flatnonzero((fl.mode == "AUTO") & fl.armed)
        t_start = float(fl.t[auto_i[0]]) if auto_i.size else None
        air = np.flatnonzero(fl.alt > 2.0)
        t_lift = float(fl.t[air[0]]) if air.size else None
        td = touchdown_report_def(fl.t, fl.alt, fl.armed)
        land = landing_report(fl, tr.waypoints[u], frame, wk["vx"], wk["vy"], wk["x"], wk["y"],
                              {"sim_wind": sim_wind, "estimated": headline or None, "plan_wind": plan_wind,
                               "fc_wind_msg": fc_csv_pool})
        t_touch = land.get("t_touch_s")
        km_all = flown_km_all(fl, frame)
        km_air = None
        if t_lift is not None and t_touch is not None:
            km_air = flown_km_all(fl, frame, (fl.t >= t_lift) & (fl.t <= t_touch))
        plan_t0, plan_t1 = su.get("t_takeoff_s"), su.get("t_land_s")
        times = {"flown_t_first_auto_s": t_start, "flown_t_lift_alt_gt_2m_s": t_lift,
                 "flown_t_touchdown_alt_le_0p5m_s": t_touch,
                 "flown_t_land_report_def_s": None if math.isnan(td) else td,
                 "clock_offset_s": off, "plan_t_takeoff_s": plan_t0, "plan_t_land_s": plan_t1,
                 "plan_t_arr_ptc_s": su.get("t_arr_ptc_s")}
        if t_start is not None and t_touch is not None and plan_t0 is not None and plan_t1 is not None:
            d_flown = t_touch - t_start
            d_plan = plan_t1 - plan_t0
            times.update(duration_flown_s=d_flown, duration_plan_s=d_plan, duration_delta_s=d_flown - d_plan,
                         duration_delta_pct=100.0 * (d_flown - d_plan) / d_plan if d_plan else None)
        if off is not None and plan_t1 is not None and t_touch is not None:
            times["touchdown_minus_plan_t_land_s"] = (t_touch - off) - plan_t1
        if off is not None and not math.isnan(td) and plan_t1 is not None:
            times["report_def_land_minus_plan_s"] = (td - off) - plan_t1
        # cross-check against the old report's numbers (only when that file exists)
        xchk = None
        if tr.flight_summary:
            old = next((r for r in tr.flight_summary if r.get("uav") == u), None)
            if old:
                xchk = {"flight_summary_flown_t_land_s": old.get("flown_t_land_s"),
                        "mine_flown_t_land_s": None if (math.isnan(td) or off is None) else round(td - off, 1),
                        "flight_summary_flown_km": old.get("flown_km"), "mine_flown_km": round(km_all, 2),
                        "flight_summary_clock_offset_s": old.get("clock_offset_s"),
                        "mine_clock_offset_s": None if off is None else round(off, 1)}
                xchk["match"] = bool(
                    xchk["mine_flown_t_land_s"] is not None and old.get("flown_t_land_s") is not None and
                    abs(xchk["mine_flown_t_land_s"] - old["flown_t_land_s"]) <= 0.15 and
                    abs(xchk["mine_flown_km"] - old.get("flown_km", -1)) <= 0.011)
        path = {"plan_distance_km": (su["distance_m"] / 1000.0) if su.get("distance_m") is not None else None,
                "flown_km_all_samples": km_all, "flown_km_airborne": km_air}
        if path["plan_distance_km"]:
            path["delta_pct_all_samples"] = 100.0 * (km_all - path["plan_distance_km"]) / path["plan_distance_km"]
        energy = {"plan_energy_used_wh": su.get("energy_used_wh")}
        if u in series and len(series[u]["bat_t"]):
            bt = series[u]["bat_t"]
            tcut = (t_touch + 5.0) if t_touch is not None else bt[-1]
            j = int(np.searchsorted(bt, tcut, side="right")) - 1
            j = max(j, 0)
            energy.update(bin_energy_used_wh=float(series[u]["bat_energy_wh"][j]),
                          bin_current_total_mah=float(series[u]["bat_currtot_mah"][j]),
                          note="SITL battery model is not calibrated to the planner's energy model; the ratio is "
                               "informational and not a wind effect by itself")
            if su.get("energy_used_wh"):
                energy["bin_over_plan"] = energy["bin_energy_used_wh"] / su["energy_used_wh"]
        leg_rows = [{k: v for k, v in r.items() if k != "steady_idx"} for r in lres]
        sum_flown = sum(r["flown_time_s"] for r in leg_rows if r.get("flown_time_s"))
        sum_plan = sum(r["plan_time_model_s"] for r in leg_rows if r.get("plan_time_model_s") and r.get("flown_time_s"))
        sum_sim = sum(r["sim_time_model_s"] for r in leg_rows if r.get("sim_time_model_s") and r.get("flown_time_s"))
        n_t = sum(1 for r in leg_rows if r.get("flown_time_s"))
        uav_out[u] = {
            "class": su.get("cls"), "plan_airspeed": pa, "tas_fit_mps": tas, "tas_source": tas_src,
            "airspeed_col_survey_mean_mps": None if not okl else float(np.mean([r["airspeed_col_mps"] for r in okl])),
            "airspeed_col_over_tas_fit": (float(np.mean([r["airspeed_col_mps"] for r in okl])) / tas) if (okl and tas) else None,
            "tas_fit_over_commanded": (tas / va_cmd) if (tas and va_cmd) else None,
            "n_survey_legs": len(lres), "n_survey_legs_measured": len(okl), "legs": leg_rows,
            "leg_classes_vs_ref": classes, "reciprocal": recip,
            "gs_model_rms_mps": {**gs_rms, "note": "rms of (measured leg gs - Vg(fitted TAS, wind)) per wind model [model]; "
                                                    "the wind whose model reproduces the flown speeds has the smallest rms"},
            "leg_time_totals": {"n_legs_with_time": n_t, "flown_s": sum_flown, "plan_model_s": sum_plan,
                                "sim_model_s": sum_sim,
                                "note": "plan_model = L/Vg(commanded airspeed, PLAN wind); sim_model = L/Vg(fitted TAS, "
                                        "SIM wind). Both are models; the planner's own per-leg times are not serialised"},
            "times": times, "report_cross_check": xchk, "path": path, "energy": energy, "landing": land,
            "cruise_samples": int(wk["cruise"].sum()), "survey_steady_samples": int(wk["sidx"].size),
            "photos": photo_report(u, tr, s1, s1p, leg_rows)}

    verdicts = build_verdicts(tr, plan_wind, sim_wind, bin_sim, estimates, uav_out, ref_name)
    vmap = {v["key"]: v["status"] for v in verdicts}
    summary = {
        "flown_wind": None if headline is None else {k: headline[k] for k in ("method", "scope", "speed_mps", "dir_from_deg",
                                                                              "wind_east_mps", "wind_north_mps")},
        "plan_wind": plan_wind, "sim_wind": sim_wind,
        "wind_acted_in_flight": None if headline is None else bool(headline["speed_mps"] >= CALM_MPS),
        "flown_wind_matches_sim_wind": vmap.get("kinematic_estimate_matches_sim_wind"),
        "plan_wind_matches_flown_wind": vmap.get("plan_wind_vs_flown_wind"),
        "n_fail": sum(1 for v in verdicts if v["status"] == "FAIL")}
    ratios_tas = [uo["tas_fit_over_commanded"] for uo in uav_out.values() if uo.get("tas_fit_over_commanded")]
    ratios_col = [uo["airspeed_col_over_tas_fit"] for uo in uav_out.values() if uo.get("airspeed_col_over_tas_fit")]
    if ratios_tas and ratios_col:
        notes.append(
            f"MEASURED: fitted true airspeed is {100 * (float(np.mean(ratios_tas)) - 1):+.1f} % vs the commanded airspeed "
            f"(range {min(ratios_tas):.3f}-{max(ratios_tas):.3f}) while the airspeed column reads "
            f"{float(np.mean(ratios_col)):.3f} of the fitted value. INFERENCE: consistent with the SITL pitot "
            f"density-correction defect (airspeed column low, TECS holds the LOW reading, true airspeed high); flown strips "
            f"are therefore faster than any plan-time model that uses the commanded airspeed.")
    # planner's crab-margin lines (what the plan said it assumed) vs measured crab
    crab_plan = []
    for ln in ((tr.report or {}).get("warnings") or []):
        m = re.search(r"mapping_crab_margin\((\w+)/\w+: .*?=([0-9.]+).*?W ([0-9.]+) / V_a ([0-9.]+)", ln)
        if m:
            crab_plan.append({"class": m.group(1), "psi_deg": float(m.group(2)), "wind_mps": float(m.group(3)),
                              "va_mps": float(m.group(4)), "line": ln})
    crab_meas = {}
    for u, uo in uav_out.items():
        cr = [abs(r["crab_deg"]) for r in uo["legs"] if r.get("crab_deg") is not None]
        if cr:
            crab_meas[u] = {"max_abs_crab_deg": float(max(cr)), "n_legs": len(cr)}
    mapping_gate = (tr.validation or {}).get("gui_mapping_gate")
    if not tr.waypoints or not any(tr.waypoints.values()):
        notes.append("no waypoint files: no survey legs")
    rep = {
        "format": FORMAT, "trial": {"dir": str(tr.dir), "name": tr.name, "uavs": tr.uavs},
        "winds": {"plan": plan_wind, "sim": sim_wind, "bin_param_readback": bin_sim,
                  "sitl_cmdlines_has_wind_parm": cmd_has_wind_parm, "reference_used_for_classification": ref_name},
        "plan_wind_handling": {"report_crab_margin_lines": crab_plan, "measured_crab_from_bin": crab_meas,
                               "mapping_wind_gate_validation_json": mapping_gate},
        "summary": summary, "time_axis": time_axis, "bin_status": bin_status, "estimates": estimates,
        "per_uav": uav_out, "verdicts": verdicts,
        "settings": {"trim_start_m": trim_start_m, "trim_end_m": trim_end_m, "xte_max_m": xte_max_m,
                     "cruise_airspeed_tol_mps": CRUISE_AIRSPEED_TOL_MPS, "cruise_min_alt_m": CRUISE_MIN_ALT_M,
                     "ident_se_max_mps": IDENT_SE_MAX_MPS, "noise_floor_mps": NOISE_FLOOR_MPS,
                     "wind_agree_abs_mps": WIND_AGREE_ABS_MPS, "wind_agree_rel": WIND_AGREE_REL,
                     "class_cos": CLASS_COS},
        "notes": notes + [
            "flight.csv has no heading column: crab is only MEASURED when a matching BIN (ATT.Yaw) is available; "
            "otherwise crab_pred_deg is a model value from the fitted wind.",
            "airspeed column = VFR_HUD (SITL pitot); it reads ~2 % low (density corrected twice). The estimators "
            "therefore fit the true airspeed instead of trusting the column."],
    }
    return rep


def reciprocal_check(legs_ok: list[dict], winds: dict) -> list[dict]:
    """Pair legs flown in opposite directions on the same axis; gs_A - gs_B = 2 * (wind along A)."""
    out = []
    used = set()
    legs_ok = [r for r in legs_ok if r.get("gs_pos_mps")]
    for i, a in enumerate(legs_ok):
        if i in used:
            continue
        axis = a["bearing_deg"]
        same = [j for j, b in enumerate(legs_ok) if j not in used and abs(wrap180(b["bearing_deg"] - axis)) <= 15.0]
        opp = [j for j, b in enumerate(legs_ok) if j not in used and abs(wrap180(b["bearing_deg"] - axis - 180.0)) <= 15.0]
        if not same or not opp:
            for j in same:
                used.add(j)
            continue
        for j in same + opp:
            used.add(j)
        gs_a = float(np.mean([legs_ok[j]["gs_pos_mps"] for j in same]))
        gs_b = float(np.mean([legs_ok[j]["gs_pos_mps"] for j in opp]))
        ua = (math.sin(math.radians(axis)), math.cos(math.radians(axis)))
        row = {"axis_bearing_deg": axis, "n_dir_a": len(same), "n_dir_b": len(opp),
               "gs_dir_a_mps": gs_a, "gs_dir_b_mps": gs_b, "delta_gs_a_minus_b_mps": gs_a - gs_b,
               "w_along_a_measured_mps": (gs_a - gs_b) / 2.0, "tas_from_mean_gs_mps": (gs_a + gs_b) / 2.0,
               "airspeed_col_mean_mps": float(np.mean([legs_ok[j]["airspeed_col_mps"] for j in same + opp])),
               "expected_w_along_a_mps": {}}
        for name, v in winds.items():
            if v is not None:
                row["expected_w_along_a_mps"][name] = v[0] * ua[0] + v[1] * ua[1]
        out.append(row)
    return out


def photo_report(u: str, tr: Trial, s1: dict | None, s1p: Path | None, leg_rows: list[dict]) -> dict:
    """Photo count vs plan.  Reuses scripts/survey_sitl_s1.py output (s1_report.json); nothing is recomputed."""
    d = max((r["trigger_dist_m"] for r in leg_rows), default=None) if leg_rows else None
    naive = None
    if leg_rows and all(r["trigger_dist_m"] > 0 for r in leg_rows):
        naive = int(sum(math.floor(r["length_m"] / r["trigger_dist_m"]) + 1 for r in leg_rows))
    out = {"planned_naive_floor_L_over_d_plus_1": naive,
           "naive_note": "reference only: ignores the 50 Hz trigger-tick quantisation (survey_sitl_s1.py docs)",
           "trigger_dist_m": d, "s1_report": str(s1p) if s1p else None}
    if s1 is None:
        out["available"] = False
        out["reason"] = "no s1_analysis*/s1_report.json in the trial (not a mapping trial or analysis not run)"
        return out
    su = (s1.get("uavs") or {}).get(u, {}).get("summary") or {}
    ex = s1.get("exposure") or {}
    pre, post = ex.get("preflight") or {}, ex.get("postflight") or {}
    out.update(available=True, fc_cam_count=su.get("n_cam"), n_strip_windows=su.get("n_windows"),
               exposure_preflight_photos=(pre.get("photos_per_uav") or {}).get(u),
               exposure_postflight_photos=(post.get("photos_per_uav") or {}).get(u),
               fc_vs_simulated=(pre.get("fc_cam_vs_simulated") or {}).get(u),
               exposure_wind_used=pre.get("wind"), exposure_wind_source=pre.get("wind_source"),
               s1_passed=(s1.get("uavs") or {}).get(u, {}).get("passed"))
    return out


def build_verdicts(tr, plan_wind, sim_wind, bin_sim, est, uav_out, ref_name) -> list[dict]:
    V = []

    def add(key, status, detail):
        V.append({"key": key, "status": status, "detail": detail})
    # 1. plan wind vs sim wind
    if plan_wind and sim_wind:
        a = winds_agree({**plan_wind}, {**sim_wind})
        d = wind_diff(plan_wind, sim_wind)
        add("plan_wind_equals_sim_wind", "PASS" if a else "FAIL",
            f"plan {wtxt(plan_wind)} vs sim {wtxt(sim_wind)}"
            f"; |dw| = {d['d_vec_mps']:.2f} m/s")
    elif plan_wind and not sim_wind:
        add("plan_wind_equals_sim_wind", "N/A",
            f"sim wind not recorded in this artifact (no wind_readback.json / sim_wind); plan wind {plan_wind['speed_mps']:g} "
            f"m/s from {plan_wind['dir_from_deg']:g}.  sitl_cmdlines has wind.parm: "
            f"{'yes' if (tr.sitl_cmdlines and 'wind.parm' in tr.sitl_cmdlines) else 'no'}")
    # 2. FC readback of the sim wind: BIN PARM vs requested
    if bin_sim and sim_wind:
        ok = abs(bin_sim["speed_mps"] - sim_wind["speed_mps"]) < 0.01 and (
            sim_wind["speed_mps"] == 0 or abs(float(wrap180(bin_sim["dir_from_deg"] - sim_wind["dir_from_deg"]))) < 0.5)
        add("bin_param_matches_requested_sim_wind", "PASS" if ok else "FAIL",
            f"BIN PARM SIM_WIND_SPD={bin_sim['speed_mps']:g} DIR={bin_sim['dir_from_deg']:g} vs requested "
            f"{sim_wind['speed_mps']:g}/{sim_wind['dir_from_deg']:g}")
    # 3. independent estimate vs sim wind (THE test that the wind acted)
    hl = est.get("headline")
    if hl is not None and sim_wind is not None:
        a = winds_agree(hl, sim_wind)
        d = wind_diff(hl, sim_wind)
        add("kinematic_estimate_matches_sim_wind", "PASS" if a else "FAIL",
            f"kinematic estimate ({hl['method']}/{hl['scope']}) {hl['speed_mps']:.2f} m/s {wfrom(hl['speed_mps'], hl['dir_from_deg'], 0)} "
            f"+/- {max(hl['se_east_mps'], hl['se_north_mps']):.2f} vs sim {wtxt(sim_wind)}; |dw| = {d['d_vec_mps']:.2f} m/s")
    elif hl is not None and sim_wind is None:
        add("kinematic_estimate_matches_sim_wind", "N/A",
            f"no recorded sim wind; kinematic estimate is {hl['speed_mps']:.2f} m/s {wfrom(hl['speed_mps'], hl['dir_from_deg'], 0)} "
            f"({hl['method']}/{hl['scope']}).  Compare with the plan wind in plan_wind_vs_flown_wind")
    else:
        parts = []
        for meth in ("speed_circle", "heading"):
            for sc in ("cruise", "survey"):
                e = est[meth].get(sc, {})
                if e.get("message"):
                    parts.append(f"{meth}/{sc}: {e['message']}")
        add("kinematic_estimate_matches_sim_wind", "N/A", "no identifiable wind estimate: " + " | ".join(parts[:2]))
    # 4. plan wind vs what was actually flown
    if plan_wind and hl is not None:
        a = winds_agree(hl, plan_wind)
        d = wind_diff(hl, plan_wind)
        add("plan_wind_vs_flown_wind", "PASS" if a else "FAIL",
            f"plan assumed {wtxt(plan_wind)}; flown atmosphere "
            f"{hl['speed_mps']:.2f} m/s {wfrom(hl['speed_mps'], hl['dir_from_deg'], 0)}; |dw| = {d['d_vec_mps']:.2f} m/s")
    # 5. FC's own WIND estimate vs sim
    fcp = est["fc_wind_msg_columns"]["pooled"]
    if fcp is not None and sim_wind is not None:
        a = winds_agree(fcp, sim_wind)
        d = wind_diff(fcp, sim_wind)
        add("fc_wind_estimate_matches_sim_wind", "PASS" if a else "FAIL",
            f"FC WIND message {fcp['speed_mps']:.2f} m/s {wfrom(fcp['speed_mps'], fcp['dir_from_deg'], 0)} vs sim "
            f"{wtxt(sim_wind)}; |dw| = {d['d_vec_mps']:.2f} m/s")
    elif fcp is None:
        add("fc_wind_estimate_matches_sim_wind", "N/A", "flight.csv has no wind_*_est columns (older artifact)")
    # 6. FC and kinematic estimates agree with each other
    if fcp is not None and hl is not None:
        a = winds_agree(hl, fcp)
        d = wind_diff(hl, fcp)
        add("kinematic_estimate_matches_fc_estimate", "PASS" if a else "FAIL", f"|dw| = {d['d_vec_mps']:.2f} m/s")
    # 7. reciprocal legs split by 2 x w_along (per aircraft, against sim wind else estimate)
    for u, uo in uav_out.items():
        for r in uo["reciprocal"]:
            ex = r["expected_w_along_a_mps"]
            ref = ex.get("sim") if ex.get("sim") is not None else ex.get("estimated")
            src = "sim" if ex.get("sim") is not None else "estimated"
            if ref is None:
                continue
            res = r["w_along_a_measured_mps"] - ref
            tol = max(WIND_AGREE_ABS_MPS, WIND_AGREE_REL * abs(ref))
            add(f"reciprocal_split_{u}_axis{r['axis_bearing_deg']:.0f}", "PASS" if abs(res) <= tol else "FAIL",
                f"(gs_a - gs_b)/2 = {r['w_along_a_measured_mps']:+.2f} m/s vs expected wind-along ({src}) {ref:+.2f}; "
                f"residual {res:+.2f} (tol {tol:.2f})")
    return V


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  output
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def _clean(o):
    """JSON-safe: numpy -> python, NaN/inf -> None."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    if isinstance(o, (np.floating, float)):
        v = float(o)
        return v if math.isfinite(v) else None
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def render_markdown(rep: dict) -> str:
    L: list[str] = []
    A = L.append
    w = rep["winds"]
    A(f"# SITL wind post-flight check: {rep['trial']['name']}")
    A("")
    A(f"- trial dir: `{rep['trial']['dir']}`; aircraft: {', '.join(rep['trial']['uavs'])}")
    A("- time axis: " + rep["time_axis"]["source"] +
      f"; median dt {rep['time_axis']['dt_median_s']:.2f} s; median |v from positions| / groundspeed column = "
      f"{_fmt(rep['time_axis']['speed_ratio_pos_over_groundspeed_col'], 3)} ({'ok' if rep['time_axis']['ok'] else 'SUSPECT'})")

    def wl(x):
        return "not recorded" if x is None else f"{x['speed_mps']:g} m/s from {x['dir_from_deg']:g} deg ({x['source']})"
    A(f"- PLAN wind (planner input): {wl(w['plan'])}")
    A(f"- SIM wind (SIM_WIND_*): {wl(w['sim'])}")
    if w["bin_param_readback"]:
        A(f"- BIN PARM SIM_WIND readback: {w['bin_param_readback']['speed_mps']:g} m/s from "
          f"{w['bin_param_readback']['dir_from_deg']:g} deg")
    if w["sitl_cmdlines_has_wind_parm"] is not None:
        A(f"- sitl_cmdlines.txt mentions wind.parm: {w['sitl_cmdlines_has_wind_parm']}")
    A("")
    sm = rep["summary"]
    A("## Summary")
    A("")
    fw = sm["flown_wind"]
    A("- wind actually flown (kinematic): " + (
        "no identifiable estimate" if fw is None else
        f"{fw['speed_mps']:.2f} m/s {wfrom(fw['speed_mps'], fw['dir_from_deg'], 0)} ({fw['method']}/{fw['scope']})"))
    A(f"- wind acted in flight: {sm['wind_acted_in_flight']}; flown wind matches SIM wind: {sm['flown_wind_matches_sim_wind']}; "
      f"plan wind matches flown wind: {sm['plan_wind_matches_flown_wind']}; failed verdicts: {sm['n_fail']}")
    A("")
    A("## Verdicts (thresholds: |dw| <= max(%.1f m/s, %.0f %% of the reference wind))" %
      (rep["settings"]["wind_agree_abs_mps"], 100 * rep["settings"]["wind_agree_rel"]))
    A("")
    A("| check | result | detail |")
    A("|---|---|---|")
    for v in rep["verdicts"]:
        A(f"| {v['key']} | **{v['status']}** | {v['detail']} |")
    A("")
    A("## Wind estimates (MEASURED from flight data; SE = standard error)")
    A("")
    A("| method | scope | n | status | wind east/north (m/s) | speed (m/s) | from (deg) | SE (m/s) | residual rms |")
    A("|---|---|---|---|---|---|---|---|---|")
    for meth in ("speed_circle", "heading"):
        for sc in ("cruise", "survey"):
            e = rep["estimates"][meth].get(sc) or {}
            if e.get("status") == "ok":
                A(f"| {meth} | {sc} | {e['n']} | ok | {e['wind_east_mps']:+.2f} / {e['wind_north_mps']:+.2f} | "
                  f"{e['speed_mps']:.2f} | {dirtxt(e['speed_mps'], e['dir_from_deg'], 1)} | {max(e['se_east_mps'], e['se_north_mps']):.2f} | "
                  f"{_fmt(e.get('rms_resid_mps'), 2)} |")
            else:
                A(f"| {meth} | {sc} | {e.get('n', 0)} | **{e.get('status', 'n/a')}** | - | - | - | - | - |")
    for name, key in (("FC WIND msg (flight.csv)", "fc_wind_msg_columns"), ("FC EKF XKF2 (BIN)", "fc_ekf_xkf2")):
        p = rep["estimates"][key]["pooled"]
        if p:
            A(f"| {name} | cruise | - | ok | {p['wind_east_mps']:+.2f} / {p['wind_north_mps']:+.2f} | {p['speed_mps']:.2f} | "
              f"{dirtxt(p['speed_mps'], p['dir_from_deg'], 1)} | - | - |")
        else:
            A(f"| {name} | cruise | - | n/a | - | - | - | - | - |")
    A("")
    for meth in ("speed_circle", "heading"):
        for sc in ("cruise", "survey"):
            e = rep["estimates"][meth].get(sc) or {}
            if e.get("message"):
                A(f"- `{meth}/{sc}`: {e['message']}")
    hl = rep["estimates"]["headline"]
    if hl:
        A(f"- headline = {hl['method']}/{hl['scope']}: {hl['speed_mps']:.2f} m/s {wfrom(hl['speed_mps'], hl['dir_from_deg'], 1)}")
    A("")
    ph = rep["plan_wind_handling"]
    if ph["report_crab_margin_lines"] or ph["mapping_wind_gate_validation_json"]:
        A("## How the plan itself handled wind (from the artifact)")
        A("")
        for c in ph["report_crab_margin_lines"]:
            A(f"- planner crab margin, class {c['class']}: psi = {c['psi_deg']:.2f} deg for W = {c['wind_mps']:g} m/s, "
              f"Va = {c['va_mps']:g} m/s (strip spacing shrinks accordingly)")
        if ph["measured_crab_from_bin"]:
            A("- measured max |crab| on survey legs (BIN yaw): " +
              ", ".join(f"{u} {v['max_abs_crab_deg']:.2f} deg" for u, v in ph["measured_crab_from_bin"].items()))
        if ph["mapping_wind_gate_validation_json"]:
            A(f"- validation.json gui_mapping_gate: {ph['mapping_wind_gate_validation_json']}")
        A("")
    for u, uo in rep["per_uav"].items():
        A(f"## {u} ({uo['class'] or '?'})")
        A("")
        pa = uo["plan_airspeed"]
        A(f"- commanded airspeed {_fmt(pa['value_mps'], 1)} m/s ({pa['source']}); fitted TAS {_fmt(uo['tas_fit_mps'], 2)} m/s "
          f"(x{_fmt(uo['tas_fit_over_commanded'], 3)} of commanded); airspeed column on survey legs "
          f"{_fmt(uo['airspeed_col_survey_mean_mps'], 2)} (x{_fmt(uo['airspeed_col_over_tas_fit'], 3)} of fitted TAS)")
        A(f"- survey legs {uo['n_survey_legs_measured']}/{uo['n_survey_legs']} measured; steady samples {uo['survey_steady_samples']}; "
          f"cruise samples {uo['cruise_samples']}")
        if uo["reciprocal"]:
            A("")
            A("Reciprocal strips (MEASURED gs; expected = model):")
            A("")
            A("| axis (deg) | n a/b | gs a (m/s) | gs b (m/s) | gs a - gs b | w_along(a) = diff/2 | expected sim | expected est | expected plan |")
            A("|---|---|---|---|---|---|---|---|---|")
            for r in uo["reciprocal"]:
                e = r["expected_w_along_a_mps"]
                A(f"| {r['axis_bearing_deg']:.0f} | {r['n_dir_a']}/{r['n_dir_b']} | {r['gs_dir_a_mps']:.2f} | "
                  f"{r['gs_dir_b_mps']:.2f} | {r['delta_gs_a_minus_b_mps']:+.2f} | {r['w_along_a_measured_mps']:+.2f} | "
                  f"{_fmt(e.get('sim'), 2)} | {_fmt(e.get('estimated'), 2)} | {_fmt(e.get('plan'), 2)} |")
        if uo["leg_classes_vs_ref"]:
            A("")
            A("Leg classes vs the reference wind (" + str(rep["winds"]["reference_used_for_classification"]) + "): " +
              "; ".join(f"{k}: {v['n_legs']} legs, mean gs {v['gs_pos_mean_mps']:.2f} m/s"
                        for k, v in uo["leg_classes_vs_ref"].items()))
        A("")
        A("| leg | brg | L (m) | n | gs col | gs pos | asp col | course | trk err | crab (BIN) | crab pred | w_along ref | class | flown t (s) | plan t model | sim t model |")
        A("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for r in uo["legs"]:
            if r["status"] != "ok":
                A(f"| {r['leg']} | {r['bearing_deg']:.0f} | {r['length_m']:.0f} | {r['n_steady']} | insufficient samples ||||||||||||")
                continue
            A(f"| {r['leg']} | {r['bearing_deg']:.0f} | {r['length_m']:.0f} | {r['n_steady']} | {r['gs_col_mps']:.2f} | "
              f"{r['gs_pos_mps']:.2f} | {r['airspeed_col_mps']:.2f} | {r['course_deg']:.1f} | {r['track_err_deg']:+.2f} | "
              f"{_fmt(r.get('crab_deg'), 2)} | {_fmt(r.get('crab_pred_deg'), 2)} | {_fmt(r.get('wind_along_ref_mps'), 2)} | "
              f"{r.get('class_vs_ref', '-')} | "
              f"{_fmt(r.get('flown_time_s'), 1)} | {_fmt(r.get('plan_time_model_s'), 1)} | {_fmt(r.get('sim_time_model_s'), 1)} |")
        t = uo["leg_time_totals"]
        gr = uo["gs_model_rms_mps"]
        A("")
        A(f"- rms of (measured leg gs - model gs with fitted TAS): estimated wind {_fmt(gr.get('est'), 2)}, "
          f"sim wind {_fmt(gr.get('sim'), 2)}, plan wind {_fmt(gr.get('plan'), 2)} m/s [model]")
        A(f"- survey-leg time totals over {t['n_legs_with_time']} legs: flown {t['flown_s']:.1f} s; "
          f"plan-wind model {t['plan_model_s']:.1f} s; sim-wind model {t['sim_model_s']:.1f} s [models]")
        tm = uo["times"]
        A(f"- planned vs flown: plan t_takeoff {_fmt(tm['plan_t_takeoff_s'], 1)} / t_land {_fmt(tm['plan_t_land_s'], 1)} s; "
          f"flown first-AUTO {_fmt(tm['flown_t_first_auto_s'], 1)} lift {_fmt(tm['flown_t_lift_alt_gt_2m_s'], 1)} "
          f"touchdown(alt<=0.5 m) {_fmt(tm['flown_t_touchdown_alt_le_0p5m_s'], 1)} disarm-def {_fmt(tm['flown_t_land_report_def_s'], 1)} "
          f"(sim clock); clock offset {_fmt(tm['clock_offset_s'], 2)} s")
        if "duration_flown_s" in tm:
            A(f"- mission duration (first AUTO -> touchdown): flown {tm['duration_flown_s']:.1f} s vs plan "
              f"{tm['duration_plan_s']:.1f} s => {tm['duration_delta_s']:+.1f} s ({tm['duration_delta_pct']:+.2f} %)")
        if "touchdown_minus_plan_t_land_s" in tm:
            A(f"- touchdown minus plan t_land (offset-corrected): {tm['touchdown_minus_plan_t_land_s']:+.1f} s; "
              f"disarm-definition (old report): {_fmt(tm.get('report_def_land_minus_plan_s'), 1)} s")
        xc = uo["report_cross_check"]
        if xc:
            A(f"- cross-check vs flight_summary.json: t_land {xc['mine_flown_t_land_s']} vs {xc['flight_summary_flown_t_land_s']}, "
              f"km {xc['mine_flown_km']} vs {xc['flight_summary_flown_km']} => {'match' if xc['match'] else 'DIFFERENT'}")
        pth = uo["path"]
        A(f"- path: plan {_fmt(pth['plan_distance_km'], 2)} km, flown {_fmt(pth['flown_km_all_samples'], 2)} km (all samples), "
          f"{_fmt(pth['flown_km_airborne'], 2)} km (airborne); delta {_fmt(pth.get('delta_pct_all_samples'), 1)} %")
        en = uo["energy"]
        A(f"- energy: plan {_fmt(en['plan_energy_used_wh'], 1)} Wh; BIN BAT.EnrgTot {_fmt(en.get('bin_energy_used_wh'), 1)} Wh "
          f"(x{_fmt(en.get('bin_over_plan'), 2)} of plan; informational, SITL battery model is not calibrated to the plan)")
        ld = uo["landing"]
        A(f"- landing: landed={ld['landed']}; touchdown error vs planned landing item "
          f"{_fmt(ld.get('touchdown_err_m'), 2)} m (E {_fmt(ld.get('touchdown_err_east_m'), 2)}, N {_fmt(ld.get('touchdown_err_north_m'), 2)}); "
          f"VTOL descent mean offset E {_fmt(ld.get('vtol_descent_mean_offset_east_m'), 2)} N {_fmt(ld.get('vtol_descent_mean_offset_north_m'), 2)} m, "
          f"rms {_fmt(ld.get('vtol_descent_rms_offset_m'), 2)} m")
        ap = ld.get("approach")
        if ap:
            rel = "; ".join(f"{k}: {v.get('class')} (headwind component {_fmt(v.get('headwind_component_mps'), 2)} m/s)"
                            for k, v in ap["vs_wind"].items())
            A(f"- final approach: flown course {ap['course_deg']:.1f} deg (planned bearing {_fmt(ap['planned_bearing_deg'], 1)}), "
              f"gs {ap['gs_mean_mps']:.2f} m/s; vs wind => {rel}")
        ph_ = uo["photos"]
        if ph_.get("available"):
            A(f"- photos: FC CAM {ph_['fc_cam_count']} over {ph_['n_strip_windows']} strip windows; exposure preflight "
              f"{ph_['exposure_preflight_photos']} / postflight {ph_['exposure_postflight_photos']}; naive floor(L/d)+1 = "
              f"{ph_['planned_naive_floor_L_over_d_plus_1']} (reference); s1 exposure wind {ph_['exposure_wind_used']}")
        else:
            A(f"- photos: n/a ({ph_.get('reason')}); naive floor(L/d)+1 = {ph_['planned_naive_floor_L_over_d_plus_1']} (reference)")
        A("")
    A("## BIN status")
    A("")
    for u, b in rep["bin_status"].items():
        A(f"- {u}: {b['status']}" + (f" ({b.get('reason')})" if b.get("reason") else "") +
          (f"; identity median {b['identity']['median_m']:.2f} m over {b['identity']['n']} samples" if b.get("identity") and b["identity"].get("median_m") is not None else ""))
    A("")
    A("## Notes")
    A("")
    for n in rep["notes"]:
        A(f"- {n}")
    return "\n".join(L) + "\n"


def _check_out(out: Path) -> Path:
    out = Path(out).resolve()
    data = (ROOT / "data").resolve()
    if out == data or data in out.parents:
        raise SystemExit(f"output directory {out} is under data/: this tool never writes the user's data directory")
    return out


def _parse_bins(items) -> dict:
    out = {}
    for it in items or []:
        if "=" not in it:
            raise SystemExit(f"--bin expects UAV=PATH, got {it!r}")
        k, v = it.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SITL wind post-flight check (offline; no SITL, no connections)")
    ap.add_argument("trial_dir", help="runs/artifacts/<TRIAL> written by scripts/hv_sitl_run.py fly")
    ap.add_argument("--out", help="output dir (default runs/artifacts/WINDFLOW_CHECK/<TRIAL>); never under data/")
    ap.add_argument("--bin", action="append", default=[], metavar="UAV=PATH", help="explicit .BIN of one aircraft")
    ap.add_argument("--no-bin", action="store_true", help="flight.csv only (no BIN: no heading/crab/EKF wind)")
    ap.add_argument("--s1-report", help="explicit s1_report.json (default: newest s1_analysis*/ in the trial)")
    ap.add_argument("--trim-start-m", type=float, default=TRIM_START_M)
    ap.add_argument("--trim-end-m", type=float, default=TRIM_END_M)
    ap.add_argument("--xte-max-m", type=float, default=XTE_MAX_M)
    ap.add_argument("--quiet", action="store_true", help="do not print the markdown")
    ap.add_argument("--strict", action="store_true", help="exit 1 when any verdict is FAIL (default: exit 0)")
    args = ap.parse_args(argv)
    for s in (sys.stdout, sys.stderr):
        try:
            s.reconfigure(encoding="utf-8")
        except Exception:       # noqa: BLE001
            pass
    trial = Path(args.trial_dir)
    out = _check_out(args.out or (ROOT / "runs" / "artifacts" / "WINDFLOW_CHECK" / trial.name))
    rep = analyze_trial(trial, bins=_parse_bins(args.bin), use_bin=not args.no_bin, s1_report=args.s1_report,
                        trim_start_m=args.trim_start_m, trim_end_m=args.trim_end_m, xte_max_m=args.xte_max_m)
    out.mkdir(parents=True, exist_ok=True)
    rep_j = _clean(rep)
    (out / "wind_check.json").write_text(json.dumps(rep_j, ensure_ascii=False, indent=1), encoding="utf-8")
    md = render_markdown(rep_j)
    (out / "wind_check.md").write_text(md, encoding="utf-8", newline="\n")
    if not args.quiet:
        print(md)
    print(f"written: {out / 'wind_check.json'}\n         {out / 'wind_check.md'}")
    bad = [v for v in rep_j["verdicts"] if v["status"] == "FAIL"]
    return 1 if (bad and args.strict) else 0


if __name__ == "__main__":
    raise SystemExit(main())
