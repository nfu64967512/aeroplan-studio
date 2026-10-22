"""scripts/hv_sitl_wind_check.py: the SITL wind post-flight check.

No SITL, no connections.  Synthetic trials are generated from a KNOWN wind with a small kinematic model
(constant true airspeed, crab to hold track, coordinated 180-degree turns), written in the exact on-disk format of
``scripts/hv_sitl_run.py fly`` (flight.csv / plan/*.waypoints / scenario.json), and the tool must recover the wind.

The tests are written so that the instrument can FIRE: a flight made with wind W but declared as another wind must
come out FAIL; the estimators must refuse (not guess) when the geometry cannot separate the wind from the airspeed.

The last group runs on the real local artifacts (gitignored) and is skipped when they are absent.
"""
from __future__ import annotations

import csv
import importlib
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")
os.environ.setdefault("MAVLINK20", "1")

wc = importlib.import_module("scripts.hv_sitl_wind_check")

REPO = Path(__file__).resolve().parents[1]
HOME = (23.95, 120.9)
FRAME = wc.Frame(*HOME)


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  synthetic flights from a known wind
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
class Sim:
    """Constant true airspeed ``va`` in a uniform wind blowing TOWARD (we, wn) m/s; dt = 0.05 s."""

    def __init__(self, we, wn, va, x0=-900.0, y0=0.0, dt=0.05, turn_radius=250.0):
        self.we, self.wn, self.va, self.dt, self.R = we, wn, va, dt, turn_radius
        self.x, self.y, self.t = x0, y0, 0.0
        self.rec = []          # (t, x, y, psi_deg, gx, gy, seq)

    def _log(self, psi, gx, gy, seq):
        self.rec.append((self.t, self.x, self.y, math.degrees(psi) % 360.0, gx, gy, seq))

    def heading_for_track(self, bearing_deg):
        b = math.radians(bearing_deg)
        w_r = self.we * math.cos(b) - self.wn * math.sin(b)
        return b - math.asin(w_r / self.va)

    def straight(self, bearing_deg, length_m, seq):
        b = math.radians(bearing_deg)
        w_r = self.we * math.cos(b) - self.wn * math.sin(b)
        w_a = self.we * math.sin(b) + self.wn * math.cos(b)
        gs = w_a + math.sqrt(self.va ** 2 - w_r ** 2)
        psi = self.heading_for_track(bearing_deg)
        n = int(math.ceil(length_m / (gs * self.dt)))
        for _ in range(n):
            self.x += gs * self.dt * math.sin(b)
            self.y += gs * self.dt * math.cos(b)
            self.t += self.dt
            self._log(psi, gs * math.sin(b), gs * math.cos(b), seq)

    def turn_left_to(self, bearing_next_deg, seq):
        """Turn left (heading decreasing) until the heading needed to hold the next track is reached."""
        psi = self.rec[-1][3] * math.pi / 180.0 if self.rec else 0.0
        target = self.heading_for_track(bearing_next_deg)
        rate = self.va / self.R
        # heading must decrease by (psi - target) modulo 2pi
        total = (psi - target) % (2 * math.pi)
        n = int(round(total / (rate * self.dt)))
        for k in range(n):
            psi -= (total / n)
            gx = self.va * math.sin(psi) + self.we
            gy = self.va * math.cos(psi) + self.wn
            self.x += gx * self.dt
            self.y += gy * self.dt
            self.t += self.dt
            self._log(psi, gx, gy, seq)


def _lat_lon(x, y):
    m_lat, m_lon = wc.metres_per_degree(HOME[0])
    return HOME[0] + y / m_lat, HOME[1] + x / m_lon


def make_strip_flight(wind_from=(5.0, 315.0), va=21.5, n_legs=6, *, bearings=None, survey_only_bearing=None,
                      length=1700.0, asp_bias=1.0, noise_m=0.0, seed=1, fc_wind=True, t0=100.04, va_cmd=None):
    """Returns (rows, waypoint_lines, sim).  ``bearings`` overrides the alternating 90/270 legs.
    ``survey_only_bearing``: only legs with this bearing carry the camera (206) => single-heading survey data."""
    we, wn = wc.wind_vec(*wind_from)
    sim = Sim(we, wn, va)
    bearings = bearings or [90.0 if i % 2 == 0 else 270.0 for i in range(n_legs)]
    items = []          # (cmd, p1, p2, lat, lon, alt)

    def add(cmd, p1=0.0, p2=0.0, x=None, y=None, alt=0.0):
        la, lo = (0.0, 0.0) if x is None else _lat_lon(x, y)
        items.append((cmd, p1, p2, la, lo, alt))
        return len(items) - 1                # seq of the item just added (item 0 is home)
    items.append((16, 0, 0, HOME[0], HOME[1], 0.0))                       # seq 0 home
    add(84, x=0.0, y=0.0, alt=60.0)                                       # seq 1 VTOL takeoff
    add(16, p2=30.0, x=-1500.0, y=0.0, alt=100.0)                         # seq 2 transit
    add(178, 0.0, va_cmd if va_cmd is not None else va)                   # seq 3 DO_CHANGE_SPEED airspeed
    seq_of_start, seq_of_end = [], []
    for i, b in enumerate(bearings):
        # first leg starts where the aircraft is; the previous turn ended on the required heading
        xs, ys = sim.x, sim.y
        a = add(16, p2=30.0, x=xs, y=ys, alt=100.0)
        survey = survey_only_bearing is None or abs(wc.wrap180(b - survey_only_bearing)) < 1.0
        if survey:
            add(206, 18.8)
        # need the end position first: simulate, then patch the end waypoint
        sim.straight(b, length, seq=-1)
        xe, ye = sim.x, sim.y
        e = add(16, p2=30.0, x=xe, y=ye, alt=100.0)
        if survey:
            add(206, 0.0)
        # fix the wp_seq of the samples just logged (current target of a straight leg = its end NAV)
        for k in range(len(sim.rec) - 1, -1, -1):
            if sim.rec[k][6] != -1:
                break
            r = sim.rec[k]
            sim.rec[k] = r[:6] + (e,)
        seq_of_start.append(a)
        seq_of_end.append(e)
        if i + 1 < len(bearings):
            sim.turn_left_to(bearings[i + 1], seq=e + (2 if survey else 1))     # target = next leg's start NAV
    # rows at 1 Hz sim time
    rec = np.array(sim.rec)
    t_all = rec[:, 0]
    rng = np.random.default_rng(seed)
    rows = []
    k = 1
    while k * 1.0 <= t_all[-1]:
        i = int(np.argmin(np.abs(t_all - k)))
        x, y = rec[i, 1], rec[i, 2]
        if noise_m:
            x, y = x + rng.normal(0, noise_m), y + rng.normal(0, noise_m)
        la, lo = _lat_lon(x, y)
        gs = math.hypot(rec[i, 4], rec[i, 5])
        wsp, wdir = wc.wind_from_vec(sim.we, sim.wn)
        rows.append({"t_s": f"{t0 + k:.2f}", "lat": f"{la:.7f}", "lon": f"{lo:.7f}", "rel_alt_m": "100.00",
                     "airspeed": f"{va * asp_bias:.2f}", "groundspeed": f"{gs:.2f}", "wp_seq": str(int(rec[i, 6])),
                     "mode": "AUTO", "armed": "True",
                     "wind_spd_est": f"{wsp:.4f}" if fc_wind else "", "wind_dir_from_est": f"{wdir:.2f}" if fc_wind else ""})
        k += 1
    wp_lines = ["QGC WPL 110"]
    for n, (cmd, p1, p2, la, lo, alt) in enumerate(items):
        wp_lines.append(f"{n}\t{1 if n == 0 else 0}\t{0 if n == 0 else 3}\t{cmd}\t{p1}\t{p2}\t0\t0\t{la:.7f}\t{lo:.7f}\t{alt:.6f}\t1")
    return rows, wp_lines, sim


def write_trial(root: Path, name: str, flights: dict, *, plan_wind=(5.0, 270.0), sim_wind=None, readback=False,
                cmdline=None, time_scale=1.0) -> Path:
    """flights = {uav: (rows, wp_lines, sim)}."""
    d = root / name
    (d / "plan").mkdir(parents=True)
    scen = {"key": "synthetic", "wind": list(plan_wind)}
    if sim_wind is not None:
        scen["sim_wind"] = {"spd_mps": sim_wind[0], "dir_from_deg": sim_wind[1]}
    (d / "scenario.json").write_text(json.dumps(scen), encoding="utf-8")
    if sim_wind is not None and readback:
        rows = [{"uav": u, "param": p, "expected": v, "actual": v, "ok": True}
                for u in flights for p, v in (("SIM_WIND_SPD", sim_wind[0]), ("SIM_WIND_DIR", sim_wind[1]),
                                              ("SIM_WIND_TURB", 0.0), ("SIM_WIND_T", 1.0))]
        (d / "wind_readback.json").write_text(json.dumps({"rows": rows, "plan_wind": list(plan_wind)}), encoding="utf-8")
    if cmdline is not None:
        (d / "sitl_cmdlines.txt").write_text(cmdline, encoding="utf-8")
    for u, (rows, wp_lines, _sim) in flights.items():
        (d / u).mkdir()
        with (d / u / "flight.csv").open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            for r in rows:
                r = dict(r)
                if time_scale != 1.0:
                    r["t_s"] = f"{float(r['t_s']) * time_scale:.3f}"
                w.writerow(r)
        (d / "plan" / f"{u}.waypoints").write_text("\n".join(wp_lines) + "\n", encoding="utf-8")
    return d


def bin_series_from(sim: Sim, *, we_est=None, wn_est=None) -> dict:
    rec = np.array(sim.rec)
    n = len(rec)
    we_est = sim.we if we_est is None else we_est
    wn_est = sim.wn if wn_est is None else wn_est
    return {"att_t": rec[:, 0] + 100.04, "att_roll": np.zeros(n), "att_pitch": np.zeros(n), "att_yaw": rec[:, 3],
            "pos_t": np.array([]), "pos_lat": np.array([]), "pos_lon": np.array([]),
            "ekf_t": rec[:, 0] + 100.04, "ekf_vwn": np.full(n, wn_est), "ekf_vwe": np.full(n, we_est), "ekf_src": "XKF2",
            "bat_t": np.array([]), "bat_volt": np.array([]), "bat_curr": np.array([]), "bat_currtot_mah": np.array([]),
            "bat_energy_wh": np.array([]), "params": {}}


def _est(rep, method, scope):
    return rep["estimates"][method][scope]


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  conventions and small pieces
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def test_wind_direction_convention_is_meteorological_from():
    we, wn = wc.wind_vec(3.0, 270.0)           # westerly blows toward the east
    assert we == pytest.approx(3.0) and wn == pytest.approx(0.0, abs=1e-12)
    we, wn = wc.wind_vec(3.0, 0.0)             # northerly blows toward the south
    assert we == pytest.approx(0.0, abs=1e-12) and wn == pytest.approx(-3.0)
    sp, dr = wc.wind_from_vec(3.0, 0.0)
    assert (sp, dr) == (pytest.approx(3.0), pytest.approx(270.0))
    sp, dr = wc.wind_from_vec(0.0, 3.0)        # blows toward the north = comes from the south
    assert dr == pytest.approx(180.0)
    for spd, d in ((4.0, 45.0), (2.0, 315.0), (7.0, 123.0)):
        assert wc.wind_from_vec(*wc.wind_vec(spd, d)) == (pytest.approx(spd), pytest.approx(d))


def test_leg_speed_model_headwind_tailwind_crosswind():
    we, wn = wc.wind_vec(5.0, 270.0)
    assert wc.leg_speed_model(1000, 90.0, 21.5, we, wn) == pytest.approx(26.5)         # downwind
    assert wc.leg_speed_model(1000, 270.0, 21.5, we, wn) == pytest.approx(16.5)        # upwind
    assert wc.leg_speed_model(1000, 0.0, 21.5, we, wn) == pytest.approx(math.sqrt(21.5 ** 2 - 25))
    assert math.isnan(wc.leg_speed_model(1000, 0.0, 4.0, we, wn))                        # wind > airspeed cross-track


def test_flight_csv_dedupes_and_refuses_a_backwards_clock(tmp_path):
    base = "t_s,lat,lon,rel_alt_m,airspeed,groundspeed,wp_seq,mode,armed\n"
    ok = tmp_path / "ok.csv"
    ok.write_text(base + "1.0,23.9,120.9,0,0,0,0,AUTO,True\n1.0,23.9,120.9,0,0,0,0,AUTO,True\n"
                  "2.0,23.9,120.9,0,0,0,0,AUTO,True\n3.0,None,120.9,0,0,0,0,AUTO,True\n", encoding="utf-8")
    fl = wc.read_flight_csv(ok, "A1")
    assert list(fl.t) == [1.0, 2.0] and fl.info["n_dup"] == 1 and fl.info["n_bad"] == 1 and not fl.has_wind_cols
    bad = tmp_path / "bad.csv"
    bad.write_text(base + "5.0,23.9,120.9,0,0,0,0,AUTO,True\n4.0,23.9,120.9,0,0,0,0,AUTO,True\n", encoding="utf-8")
    with pytest.raises(ValueError, match="backwards"):
        wc.read_flight_csv(bad)
    miss = tmp_path / "miss.csv"
    miss.write_text("t_s,lat,lon\n1,2,3\n", encoding="utf-8")
    with pytest.raises(ValueError, match="missing columns"):
        wc.read_flight_csv(miss)


def test_survey_legs_come_from_the_camera_trigger_window(tmp_path):
    lines = ["QGC WPL 110",
             "0\t1\t0\t16\t0\t0\t0\t0\t23.95\t120.9\t0\t1",
             "1\t0\t3\t84\t0\t0\t0\t0\t23.95\t120.9\t60\t1",
             "2\t0\t3\t16\t0\t30\t0\t0\t23.95\t120.89\t100\t1",
             "3\t0\t3\t178\t0\t21.5\t-1\t0\t0\t0\t0\t1",
             "4\t0\t3\t16\t0\t1\t0\t0\t23.95\t120.891\t100\t1",
             "5\t0\t3\t206\t18.8\t0\t0\t0\t0\t0\t0\t1",
             "6\t0\t3\t16\t0\t30\t0\t0\t23.95\t120.909\t100\t1",
             "7\t0\t3\t206\t0\t0\t0\t0\t0\t0\t0\t1",
             "8\t0\t3\t16\t0\t30\t0\t0\t23.951\t120.909\t100\t1",
             "9\t0\t3\t16\t0\t1\t0\t0\t23.951\t120.908\t100\t1",
             "10\t0\t3\t206\t18.8\t0\t0\t0\t0\t0\t0\t1",
             "11\t0\t3\t16\t0\t30\t0\t0\t23.951\t120.89\t100\t1",
             "12\t0\t3\t16\t0\t30\t0\t0\t23.952\t120.89\t100\t1",       # second NAV inside the same strip
             "13\t0\t3\t206\t0\t0\t0\t0\t0\t0\t0\t1",
             "14\t0\t3\t85\t0\t0\t0\t0\t23.95\t120.9\t0\t1"]
    p = tmp_path / "x.waypoints"
    p.write_text("\n".join(lines) + "\n", encoding="utf-8")
    legs = wc.survey_legs(wc.read_waypoints(p))
    assert [(g.start_seq, g.end_seq, g.strip) for g in legs] == [(4, 6, 0), (9, 11, 1), (11, 12, 1)]
    assert all(g.trigger_dist_m == pytest.approx(18.8) for g in legs)
    pa = wc.plan_airspeed(wc.read_waypoints(p), None, None)
    assert pa["value_mps"] == pytest.approx(21.5) and pa["kind"] == "airspeed"


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  estimators on synthetic flights
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def _two_uav_trial(tmp_path, wind_from, *, name="t", plan_wind=(5.0, 270.0), sim_wind=None, readback=False,
                   asp_bias=1.0, noise_m=0.0, with_cmdline=None, time_scale=1.0, **kw):
    f = {"A1": make_strip_flight(wind_from, 21.5, 6, asp_bias=asp_bias, noise_m=noise_m, seed=1, **kw),
         "B2": make_strip_flight(wind_from, 19.0, 6, asp_bias=asp_bias, noise_m=noise_m, seed=2, **kw)}
    return write_trial(tmp_path, name, f, plan_wind=plan_wind, sim_wind=sim_wind, readback=readback,
                       cmdline=with_cmdline, time_scale=time_scale), f


@pytest.mark.parametrize("wind_from", [(5.0, 315.0), (3.0, 270.0), (4.0, 0.0), (6.0, 135.0)])
def test_speed_circle_recovers_a_known_wind_on_all_cruise_samples(tmp_path, wind_from):
    d, _ = _two_uav_trial(tmp_path, wind_from)
    rep = wc.analyze_trial(d, use_bin=False)
    e = _est(rep, "speed_circle", "cruise")
    assert e["status"] == "ok"
    we, wn = wc.wind_vec(*wind_from)
    assert e["wind_east_mps"] == pytest.approx(we, abs=0.05) and e["wind_north_mps"] == pytest.approx(wn, abs=0.05)
    assert e["speed_mps"] == pytest.approx(wind_from[0], abs=0.05)
    assert wc.wrap180(e["dir_from_deg"] - wind_from[1]) == pytest.approx(0.0, abs=1.0)
    # per-aircraft constant airspeeds are recovered too (they differ between the two aircraft)
    assert e["va_mps"]["A1"] == pytest.approx(21.5, abs=0.05) and e["va_mps"]["B2"] == pytest.approx(19.0, abs=0.05)
    assert rep["estimates"]["headline"]["method"] == "speed_circle"


def test_noisy_positions_still_recover_the_wind(tmp_path):
    d, _ = _two_uav_trial(tmp_path, (5.0, 315.0), noise_m=0.3)
    e = _est(wc.analyze_trial(d, use_bin=False), "speed_circle", "cruise")
    we, wn = wc.wind_vec(5.0, 315.0)
    assert e["status"] == "ok"
    assert e["wind_east_mps"] == pytest.approx(we, abs=0.3) and e["wind_north_mps"] == pytest.approx(wn, abs=0.3)


def test_pitot_bias_in_the_airspeed_column_does_not_bias_the_wind(tmp_path):
    """The SITL pitot reads ~2 % low.  The airspeed COLUMN must not be used as Va: the fit finds the true one."""
    d, _ = _two_uav_trial(tmp_path, (5.0, 315.0), asp_bias=0.975)
    rep = wc.analyze_trial(d, use_bin=False)
    e = _est(rep, "speed_circle", "cruise")
    we, wn = wc.wind_vec(5.0, 315.0)
    assert e["wind_east_mps"] == pytest.approx(we, abs=0.05) and e["wind_north_mps"] == pytest.approx(wn, abs=0.05)
    u = rep["per_uav"]["A1"]
    assert u["tas_fit_mps"] == pytest.approx(21.5, abs=0.05)
    assert u["airspeed_col_over_tas_fit"] == pytest.approx(0.975, abs=0.005)
    assert any("pitot" in n for n in rep["notes"])


def test_reciprocal_strips_alone_give_the_along_axis_but_refuse_the_cross_axis(tmp_path):
    """E-W strips only: ground velocities are collinear.  The tool must report the E-W component and say that the
    N-S component cannot be separated from the airspeed - not return a made-up number."""
    d, _ = _two_uav_trial(tmp_path, (5.0, 315.0))                     # wind toward SE: east +3.54, north -3.54
    rep = wc.analyze_trial(d, use_bin=False)
    e = _est(rep, "speed_circle", "survey")
    assert e["status"] == "partial"
    assert e["wind_east_mps"] is None and e["wind_north_mps"] is None and e["speed_mps"] is None
    ident = [a for a in e["axes"] if a["identified"]]
    assert len(ident) == 1
    assert ident[0]["axis_bearing_deg"] == pytest.approx(90.0, abs=2.0)
    assert ident[0]["component_mps"] == pytest.approx(5 * math.sin(math.radians(45)), abs=0.1)   # +3.54 east
    assert "NOT identified" in e["message"] and "collinear" in e["message"]
    assert e["g_minor_axis_std_mps"] < wc.MIN_G_MINOR_MPS


def test_single_heading_only_is_unidentifiable_not_guessed(tmp_path):
    """Only eastbound legs carry the camera: the airspeed and the along-track wind are confounded."""
    f = {"A1": make_strip_flight((5.0, 315.0), 21.5, 6, survey_only_bearing=90.0)}
    d = write_trial(tmp_path, "single", f)
    rep = wc.analyze_trial(d, use_bin=False)
    e = _est(rep, "speed_circle", "survey")
    assert e["status"] == "unidentifiable"
    assert e["speed_mps"] is None and e["dir_from_deg"] is None
    assert "no value is reported" in e["message"]
    assert e["groups"] == ["A1"] and e["n"] > 100                      # plenty of samples: the refusal is geometric
    # all-cruise scope (turn arcs give heading diversity) still works on the same flight
    assert _est(rep, "speed_circle", "cruise")["status"] == "ok"


def test_heading_estimator_resolves_the_cross_axis_that_speeds_cannot(tmp_path):
    d, f = _two_uav_trial(tmp_path, (5.0, 315.0))
    series = {u: bin_series_from(f[u][2]) for u in f}
    rep = wc.analyze_trial(d, use_bin=False, bin_series=series)
    we, wn = wc.wind_vec(5.0, 315.0)
    for scope in ("survey", "cruise"):
        e = _est(rep, "heading", scope)
        assert e["status"] == "ok", e
        assert e["wind_east_mps"] == pytest.approx(we, abs=0.05) and e["wind_north_mps"] == pytest.approx(wn, abs=0.05)
        assert e["va_mps"]["A1"] == pytest.approx(21.5, abs=0.05)
    # the sign of the cross component is right (north component NEGATIVE: wind blows toward the south-east)
    assert _est(rep, "heading", "survey")["wind_north_mps"] < -3.0
    # FC EKF estimate from the injected XKF2 series
    assert rep["estimates"]["fc_ekf_xkf2"]["pooled"]["wind_east_mps"] == pytest.approx(we, abs=1e-6)


def test_heading_estimator_with_one_heading_identifies_only_the_wind_across_the_heading(tmp_path):
    """One heading only: the wind component ACROSS the heading is measurable (it is the crab); the component along
    the heading cannot be separated from the airspeed.  The identified axis must be perpendicular to the heading."""
    f = {"A1": make_strip_flight((5.0, 315.0), 21.5, 6, survey_only_bearing=90.0)}
    d = write_trial(tmp_path, "single_head", f)
    rep = wc.analyze_trial(d, use_bin=False, bin_series={"A1": bin_series_from(f["A1"][2])})
    e = _est(rep, "heading", "survey")
    assert e["status"] == "partial" and e["speed_mps"] is None and e["wind_east_mps"] is None
    ident = [a for a in e["axes"] if a["identified"]]
    assert len(ident) == 1
    psi = math.degrees(f["A1"][2].heading_for_track(90.0)) % 360.0              # the one heading that was flown
    ax = (psi + 90.0) % 180.0
    assert abs(wc.wrap180(ident[0]["axis_bearing_deg"] - ax)) < 1.0 or abs(abs(wc.wrap180(ident[0]["axis_bearing_deg"] - ax)) - 180.0) < 1.0
    we, wn = wc.wind_vec(5.0, 315.0)
    expect = we * math.sin(math.radians(ax)) + wn * math.cos(math.radians(ax))
    assert ident[0]["component_mps"] == pytest.approx(expect, abs=0.1)


def test_crab_is_measured_from_heading_and_has_the_right_sign(tmp_path):
    d, f = _two_uav_trial(tmp_path, (5.0, 315.0))
    rep = wc.analyze_trial(d, use_bin=False, bin_series={u: bin_series_from(f[u][2]) for u in f})
    legs = rep["per_uav"]["A1"]["legs"]
    east = [r for r in legs if abs(r["bearing_deg"] - 90) < 1][0]
    west = [r for r in legs if abs(r["bearing_deg"] - 270) < 1][0]
    # wind toward 135: on an eastbound leg it comes from the left-rear (blows to the right of heading) => crab > 0
    assert east["crab_deg"] == pytest.approx(math.degrees(math.asin(5 * math.sin(math.radians(45)) / 21.5)), abs=0.1)
    assert west["crab_deg"] == pytest.approx(-east["crab_deg"], abs=0.1)
    assert east["crab_pred_deg"] == pytest.approx(east["crab_deg"], abs=0.15)         # model from the fitted wind


def test_upwind_downwind_split_is_twice_the_along_track_wind(tmp_path):
    d, _ = _two_uav_trial(tmp_path, (3.0, 270.0), sim_wind=(3.0, 270.0), readback=True, plan_wind=(3.0, 270.0))
    rep = wc.analyze_trial(d, use_bin=False)
    for u, va in (("A1", 21.5), ("B2", 19.0)):
        r = rep["per_uav"][u]["reciprocal"][0]
        assert r["delta_gs_a_minus_b_mps"] == pytest.approx(6.0, abs=0.05)             # 2 x 3 m/s
        assert r["w_along_a_measured_mps"] == pytest.approx(3.0, abs=0.03)
        assert r["tas_from_mean_gs_mps"] == pytest.approx(va, abs=0.05)
        assert r["expected_w_along_a_mps"]["sim"] == pytest.approx(3.0, abs=1e-9)
        cls = rep["per_uav"][u]["leg_classes_vs_ref"]
        assert cls["tailwind"]["gs_pos_mean_mps"] - cls["headwind"]["gs_pos_mean_mps"] == pytest.approx(6.0, abs=0.05)


def test_yaw_interpolation_goes_through_north_not_through_south():
    ser = {"att_t": np.array([0.0, 1.0]), "att_yaw": np.array([359.0, 1.0])}
    y = wc.yaw_at(ser, np.array([0.0, 0.5, 1.0]))
    assert y[0] == pytest.approx(359.0) and y[2] == pytest.approx(1.0)
    assert min(y[1], 360.0 - y[1]) < 0.01                              # ~0/360, not 180


def test_calm_wind_has_no_direction_text():
    assert wc.dirtxt(0.03, 123.0) == "calm" and wc.wfrom(0.03, 123.0) == "(calm, no direction)"
    assert wc.dirtxt(3.0, 270.4) == "270" and wc.wfrom(3.0, 270.4) == "from 270"
    assert wc.dirtxt(None, None) == "n/a"


def test_flown_leg_time_and_plan_time_model_use_the_right_wind(tmp_path):
    """Middle legs only: the first leg starts at the first sample and the last ends at the last one, so their
    entry/exit crossings are not in the data (the tool then reports None instead of inventing a time)."""
    d, _ = _two_uav_trial(tmp_path, (3.0, 270.0), plan_wind=(3.0, 270.0), sim_wind=(3.0, 270.0), readback=True)
    rep = wc.analyze_trial(d, use_bin=False)
    legs = rep["per_uav"]["A1"]["legs"]
    east = [r for r in legs if abs(r["bearing_deg"] - 90) < 1]
    west = [r for r in legs if abs(r["bearing_deg"] - 270) < 1]
    assert east[0]["flown_time_s"] is None and legs[-1]["flown_time_s"] is None
    L = east[1]["length_m"]
    assert east[1]["flown_time_s"] == pytest.approx(L / 24.5, abs=1.0)           # downwind: 21.5 + 3
    Lw = west[1]["length_m"]
    assert west[1]["flown_time_s"] == pytest.approx(Lw / 18.5, abs=1.0)          # upwind: 21.5 - 3
    assert east[1]["plan_time_model_s"] == pytest.approx(L / 24.5, rel=1e-6)     # commanded 21.5 in the PLAN wind
    assert west[1]["plan_time_model_s"] == pytest.approx(Lw / 18.5, rel=1e-6)
    # with a plan wind that differs from the flown one the plan model changes, the flown time does not
    d2, _ = _two_uav_trial(tmp_path, (3.0, 270.0), name="t2", plan_wind=(0.0, 270.0), sim_wind=(3.0, 270.0), readback=True)
    legs2 = wc.analyze_trial(d2, use_bin=False)["per_uav"]["A1"]["legs"]
    e2 = [r for r in legs2 if abs(r["bearing_deg"] - 90) < 1][1]
    assert e2["plan_time_model_s"] == pytest.approx(L / 21.5, rel=1e-6)
    assert e2["flown_time_s"] == pytest.approx(east[1]["flown_time_s"], abs=0.2)
    assert e2["sim_time_model_s"] == pytest.approx(L / 24.5, rel=0.01)


def test_trial_without_waypoint_files_still_gives_a_cruise_estimate(tmp_path):
    d, _ = _two_uav_trial(tmp_path, (4.0, 225.0))
    for p in (d / "plan").glob("*.waypoints"):
        p.unlink()
    rep = wc.analyze_trial(d, use_bin=False)
    assert any("waypoints missing" in n for n in rep["notes"])
    assert rep["per_uav"]["A1"]["n_survey_legs"] == 0
    e = _est(rep, "speed_circle", "cruise")
    assert e["status"] == "ok" and e["speed_mps"] == pytest.approx(4.0, abs=0.1)
    assert _est(rep, "speed_circle", "survey")["status"] == "insufficient"
    assert wc.render_markdown(wc._clean(rep))                                    # renders without legs


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  the instrument must be able to fire
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def _verdicts(rep):
    return {v["key"]: v["status"] for v in rep["verdicts"]}


def test_flown_wind_equal_to_declared_wind_passes_everything(tmp_path):
    d, _ = _two_uav_trial(tmp_path, (5.0, 270.0), sim_wind=(5.0, 270.0), readback=True, plan_wind=(5.0, 270.0))
    rep = wc.analyze_trial(d, use_bin=False)
    v = _verdicts(rep)
    assert v["plan_wind_equals_sim_wind"] == "PASS"
    assert v["kinematic_estimate_matches_sim_wind"] == "PASS"
    assert v["plan_wind_vs_flown_wind"] == "PASS"
    assert v["fc_wind_estimate_matches_sim_wind"] == "PASS"
    assert v["reciprocal_split_A1_axis90"] == "PASS"
    assert rep["summary"]["wind_acted_in_flight"] is True and rep["summary"]["n_fail"] == 0


def test_wind_flown_but_declared_calm_fails(tmp_path):
    """Flight made in 5 m/s from 270 while the artifact claims SIM wind 0 and plan wind 0 => must FAIL."""
    d, _ = _two_uav_trial(tmp_path, (5.0, 270.0), sim_wind=(0.0, 270.0), readback=True, plan_wind=(0.0, 270.0))
    v = _verdicts(wc.analyze_trial(d, use_bin=False))
    assert v["kinematic_estimate_matches_sim_wind"] == "FAIL"
    assert v["plan_wind_vs_flown_wind"] == "FAIL"
    assert v["reciprocal_split_A1_axis90"] == "FAIL"


def test_calm_flight_with_a_planned_wind_is_reported_as_not_matching(tmp_path):
    """The historical failure: the plan assumed 5 m/s but the SITL atmosphere was calm.  The estimate must come out
    ~0 (it must NOT echo the plan wind) and the plan-vs-flown verdict must FAIL."""
    d, _ = _two_uav_trial(tmp_path, (0.0, 270.0), sim_wind=(0.0, 270.0), readback=True, plan_wind=(5.0, 270.0))
    rep = wc.analyze_trial(d, use_bin=False)
    hl = rep["estimates"]["headline"]
    assert hl["speed_mps"] < 0.2
    v = _verdicts(rep)
    assert v["plan_wind_equals_sim_wind"] == "FAIL"
    assert v["kinematic_estimate_matches_sim_wind"] == "PASS"
    assert v["plan_wind_vs_flown_wind"] == "FAIL"
    assert rep["summary"]["wind_acted_in_flight"] is False
    # and the markdown calls it calm instead of printing the angle of noise
    md = wc.render_markdown(wc._clean(rep))
    assert "(calm, no direction)" in md


def test_estimate_does_not_echo_the_declared_wind(tmp_path):
    d, _ = _two_uav_trial(tmp_path, (2.0, 90.0), sim_wind=(5.0, 270.0), readback=True, plan_wind=(5.0, 270.0))
    e = _est(wc.analyze_trial(d, use_bin=False), "speed_circle", "cruise")
    assert e["wind_east_mps"] == pytest.approx(-2.0, abs=0.05)          # blows toward the WEST
    assert e["wind_north_mps"] == pytest.approx(0.0, abs=0.05)


def test_sim_wind_absent_from_artifact_is_inferred_only_from_the_cmdline(tmp_path):
    d, _ = _two_uav_trial(tmp_path, (0.0, 270.0), with_cmdline="[A1] ArduPlane.exe --model quadplane --defaults a.parm\n")
    rep = wc.analyze_trial(d, use_bin=False)
    assert rep["winds"]["sim"]["inferred"] is True and rep["winds"]["sim"]["speed_mps"] == 0.0
    d2, _ = _two_uav_trial(tmp_path, (0.0, 270.0), name="t2")
    rep2 = wc.analyze_trial(d2, use_bin=False)                          # no cmdline file at all => nothing inferred
    assert rep2["winds"]["sim"] is None
    assert _verdicts(rep2)["kinematic_estimate_matches_sim_wind"] == "N/A"


def test_a_wall_clock_flight_csv_is_flagged(tmp_path):
    """If t_s were wall clock at speedup 5 every speed from t_s would be 5x off.  The tool must notice."""
    d, _ = _two_uav_trial(tmp_path, (3.0, 270.0), time_scale=0.2)
    rep = wc.analyze_trial(d, use_bin=False)
    assert rep["time_axis"]["ok"] is False
    assert rep["time_axis"]["speed_ratio_pos_over_groundspeed_col"] == pytest.approx(5.0, rel=0.05)
    assert any("TIME AXIS SUSPECT" in n for n in rep["notes"])
    ok, _ = _two_uav_trial(tmp_path, (3.0, 270.0), name="ok")
    r_ok = wc.analyze_trial(ok, use_bin=False)
    assert r_ok["time_axis"]["ok"] is True
    assert r_ok["time_axis"]["speed_ratio_pos_over_groundspeed_col"] == pytest.approx(1.0, abs=0.02)


def test_bin_identity_guard_accepts_the_same_flight_and_rejects_another(tmp_path):
    d, f = _two_uav_trial(tmp_path, (3.0, 270.0))
    fl = wc.read_flight_csv(d / "A1" / "flight.csv", "A1")
    rec = np.array(f["A1"][2].rec)
    la, lo = zip(*[_lat_lon(x, y) for x, y in rec[:, 1:3]])
    ser = {"pos_t": rec[:, 0] + 100.04, "pos_lat": np.array(la), "pos_lon": np.array(lo)}
    idt = wc.bin_identity(fl, ser, FRAME)
    assert idt["ok"] and idt["median_m"] < 1.0
    ser_bad = dict(ser, pos_lat=np.array(la) + 0.003)                    # ~330 m north
    idt = wc.bin_identity(fl, ser_bad, FRAME)
    assert not idt["ok"] and "not this flight" in idt["reason"]


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  landing
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def _landing_flight(tmp_path, *, approach_bearing=270.0, wind_from=(3.0, 270.0), offset=(0.0, 0.0)):
    """Fixed-wing approach on ``approach_bearing`` to the pad at (0,0), decelerate, hover-descend, disarm."""
    we, wn = wc.wind_vec(*wind_from)
    va = 19.0
    b = math.radians(approach_bearing)
    rows = []
    n = 0
    # approach: 600 m out, flown at constant TAS with crab to hold the track
    w_r = we * math.cos(b) - wn * math.sin(b)
    w_a = we * math.sin(b) + wn * math.cos(b)
    gs = w_a + math.sqrt(va ** 2 - w_r ** 2)
    x0, y0 = -600 * math.sin(b), -600 * math.cos(b)
    steps = int(600 / gs)
    for k in range(steps):
        x, y = x0 + gs * k * math.sin(b), y0 + gs * k * math.cos(b)
        la, lo = _lat_lon(x, y)
        rows.append((100.04 + n, la, lo, 100.0, 19.0, gs, 30, "AUTO", True)); n += 1
    # hover descent 100 m -> 0 at 1.5 m/s with a small offset from the pad
    for k in range(70):
        alt = max(100.0 - 1.5 * k, 0.0)
        la, lo = _lat_lon(offset[0], offset[1])
        rows.append((100.04 + n, la, lo, alt, 1.0, 0.3, 30, "AUTO", True)); n += 1
    la, lo = _lat_lon(offset[0], offset[1])
    for k in range(4):
        rows.append((100.04 + n, la, lo, 0.0, 0.0, 0.0, 30, "AUTO", k < 2)); n += 1
    d = tmp_path / "land"
    (d / "A1").mkdir(parents=True)
    (d / "plan").mkdir()
    with (d / "A1" / "flight.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "lat", "lon", "rel_alt_m", "airspeed", "groundspeed", "wp_seq", "mode", "armed"])
        for r in rows:
            w.writerow([f"{r[0]:.2f}", f"{r[1]:.7f}", f"{r[2]:.7f}", f"{r[3]:.2f}", f"{r[4]:.2f}", f"{r[5]:.2f}", r[6], r[7], r[8]])
    la0, lo0 = _lat_lon(x0, y0)
    wp = ["QGC WPL 110",
          f"0\t1\t0\t16\t0\t0\t0\t0\t{HOME[0]}\t{HOME[1]}\t0\t1",
          f"29\t0\t3\t16\t0\t30\t0\t0\t{la0:.7f}\t{lo0:.7f}\t100\t1",
          f"30\t0\t3\t85\t0\t0\t0\t0\t{HOME[0]:.7f}\t{HOME[1]:.7f}\t0\t1"]
    (d / "plan" / "A1.waypoints").write_text("\n".join(wp) + "\n", encoding="utf-8")
    return d


def test_landing_touchdown_error_and_headwind_approach(tmp_path):
    d = _landing_flight(tmp_path, approach_bearing=270.0, wind_from=(3.0, 270.0), offset=(2.0, -1.0))
    rep = wc.analyze_trial(d, use_bin=False)
    ld = rep["per_uav"]["A1"]["landing"]
    assert ld["landed"] is True
    assert ld["touchdown_err_east_m"] == pytest.approx(2.0, abs=0.05) and ld["touchdown_err_north_m"] == pytest.approx(-1.0, abs=0.05)
    assert ld["touchdown_err_m"] == pytest.approx(math.hypot(2.0, 1.0), abs=0.05)
    ap = ld["approach"]
    assert ap["course_deg"] == pytest.approx(270.0, abs=0.5) and ap["planned_bearing_deg"] == pytest.approx(270.0, abs=0.5)
    # plan wind defaults to 5 m/s from 270 in write_trial-less trials: here scenario.json is absent => only the
    # estimate-based classification is present
    est_rel = ap["vs_wind"].get("estimated")
    if est_rel is not None:
        assert est_rel["class"] in ("headwind", "calm", "crosswind", "tailwind")


def test_landing_class_against_a_given_wind(tmp_path):
    d = _landing_flight(tmp_path, approach_bearing=90.0, wind_from=(3.0, 270.0))
    fl = wc.read_flight_csv(d / "A1" / "flight.csv", "A1")
    wps = wc.read_waypoints(d / "plan" / "A1.waypoints")
    x, y = FRAME.xy(fl.lat, fl.lon)
    vx, vy = wc.central_velocity(fl.t, x, y)
    ld = wc.landing_report(fl, wps, FRAME, vx, vy, x, y,
                           {"west": {"speed_mps": 3.0, "dir_from_deg": 270.0}, "east": {"speed_mps": 3.0, "dir_from_deg": 90.0},
                            "north": {"speed_mps": 3.0, "dir_from_deg": 0.0}, "calm": {"speed_mps": 0.1, "dir_from_deg": 123.0}})
    rel = ld["approach"]["vs_wind"]
    assert ld["approach"]["course_deg"] == pytest.approx(90.0, abs=0.5)          # flying east
    assert rel["west"]["class"] == "tailwind" and rel["west"]["headwind_component_mps"] == pytest.approx(-3.0, abs=0.05)
    assert rel["east"]["class"] == "headwind" and rel["east"]["headwind_component_mps"] == pytest.approx(3.0, abs=0.05)
    assert rel["north"]["class"] == "crosswind" and abs(rel["north"]["headwind_component_mps"]) < 0.1
    assert rel["calm"]["class"] == "calm"


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  CLI / output
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
def test_cli_writes_json_and_markdown_and_never_into_data(tmp_path, capsys):
    d, _ = _two_uav_trial(tmp_path, (3.0, 270.0), sim_wind=(3.0, 270.0), readback=True, plan_wind=(3.0, 270.0))
    out = tmp_path / "out"
    rc = wc.main([str(d), "--out", str(out), "--no-bin", "--quiet"])
    assert rc == 0
    rep = json.loads((out / "wind_check.json").read_text(encoding="utf-8"))
    assert rep["format"] == wc.FORMAT and rep["summary"]["wind_acted_in_flight"] is True
    md = (out / "wind_check.md").read_text(encoding="utf-8")
    assert "## Verdicts" in md and "## A1" in md and md.isascii()
    with pytest.raises(SystemExit):
        wc.main([str(d), "--out", str(REPO / "data" / "wind_check_must_not_exist"), "--no-bin", "--quiet"])
    assert not (REPO / "data" / "wind_check_must_not_exist").exists()


def test_strict_exit_code_on_a_failed_verdict(tmp_path):
    d, _ = _two_uav_trial(tmp_path, (0.0, 270.0), sim_wind=(0.0, 270.0), readback=True, plan_wind=(5.0, 270.0))
    out = tmp_path / "o"
    assert wc.main([str(d), "--out", str(out), "--no-bin", "--quiet"]) == 0
    assert wc.main([str(d), "--out", str(out), "--no-bin", "--quiet", "--strict"]) == 1


def test_clean_makes_json_strict():
    s = json.dumps(wc._clean({"a": float("nan"), "b": np.float64(1.5), "c": [np.int64(3), float("inf")], "d": np.bool_(True)}),
                   allow_nan=False)
    assert json.loads(s) == {"a": None, "b": 1.5, "c": [3, None], "d": True}


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
#  real local artifacts (gitignored; skipped when absent)
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════
ART = REPO / "runs" / "artifacts"


def _need(*names):
    for n in names:
        if not (ART / n / "A1" / "flight.csv").exists():
            pytest.skip(f"real artifact {n} not present")


def test_real_no_sim_wind_trial_estimates_calm_although_the_plan_assumed_five():
    _need("S1_HDMAP_20261014")
    rep = wc.analyze_trial(ART / "S1_HDMAP_20261014", use_bin=False)
    assert rep["winds"]["plan"]["speed_mps"] == 5.0 and rep["winds"]["sim"]["speed_mps"] == 0.0
    hl = rep["estimates"]["headline"]
    assert hl["speed_mps"] < 0.3, hl
    assert rep["estimates"]["fc_wind_msg_columns"]["pooled"]["speed_mps"] < 0.3
    v = _verdicts(rep)
    assert v["plan_wind_equals_sim_wind"] == "FAIL" and v["plan_wind_vs_flown_wind"] == "FAIL"
    assert v["kinematic_estimate_matches_sim_wind"] == "PASS"


def test_real_sim_wind_three_trial_estimates_three_from_270():
    _need("S1_HDMAP_20261015_PW3")
    rep = wc.analyze_trial(ART / "S1_HDMAP_20261015_PW3", use_bin=False)
    e = _est(rep, "speed_circle", "cruise")
    assert e["status"] == "ok"
    assert e["speed_mps"] == pytest.approx(3.0, abs=0.15) and wc.wrap180(e["dir_from_deg"] - 270.0) == pytest.approx(0.0, abs=3.0)
    assert rep["estimates"]["fc_wind_msg_columns"]["pooled"]["speed_mps"] == pytest.approx(3.0, abs=0.2)
    for u in ("A1", "B2", "B3"):
        r = rep["per_uav"][u]["reciprocal"][0]
        assert r["delta_gs_a_minus_b_mps"] == pytest.approx(6.0, abs=0.1)
        assert rep["per_uav"][u]["landing"]["landed"] is True
        assert rep["per_uav"][u]["landing"]["approach"]["vs_wind"]["sim_wind"]["class"] == "headwind"
    assert all(v["status"] != "FAIL" for v in rep["verdicts"])


def test_real_old_artifact_without_wind_columns_or_bin_still_runs_and_matches_the_old_report():
    _need("HV4SITL15")
    rep = wc.analyze_trial(ART / "HV4SITL15", use_bin=False)
    assert rep["winds"]["sim"]["inferred"] is True
    assert rep["estimates"]["fc_wind_msg_columns"]["pooled"] is None
    assert rep["estimates"]["headline"]["speed_mps"] < 0.3
    for u, uo in rep["per_uav"].items():
        xc = uo["report_cross_check"]
        assert xc is not None and xc["match"], (u, xc)      # same t_land / km as flight_summary.json (hv_sitl_report)


def test_real_bin_heading_estimate_and_identity_check_when_the_bin_is_on_disk():
    _need("S1_HDMAP_20261015_PW3")
    knobs = json.loads((ART / "S1_HDMAP_20261015_PW3" / "survey_knobs.json").read_text(encoding="utf-8"))
    bins = [Path(r["bin"]) for r in knobs["sitl_logs"].values()]
    if not all(b.exists() for b in bins):
        pytest.skip("the SITL .BIN files of that trial are not on this machine")
    pytest.importorskip("pymavlink")
    rep = wc.analyze_trial(ART / "S1_HDMAP_20261015_PW3", use_bin=True)
    assert all(b["status"] == "ok" and b["identity"]["median_m"] < 1.0 for b in rep["bin_status"].values())
    e = _est(rep, "heading", "survey")
    assert e["status"] == "ok" and e["speed_mps"] == pytest.approx(3.0, abs=0.1)
    assert rep["winds"]["bin_param_readback"]["speed_mps"] == 3.0
    assert rep["per_uav"]["A1"]["legs"][0]["crab_deg"] is not None
