#!/usr/bin/env python3
"""M2 -- signal-chain dissection, node by node.

For every one of the 32 RRD/RRE sorties this replays tot_node._pn_cmd against
the flight controller's OWN recorded state (dataflash), then compares the
replay's final output with the command the firmware actually latched
(ATT.DesPitch == nav_pitch_cd/100).  Agreement there is the self-check that
licenses every intermediate value.

PROVENANCE OF EVERY QUANTITY IS TAGGED:
  [M] measured      -- read verbatim out of the dataflash log
  [R] reconstructed -- computed here from measured inputs + the node's source
"""
import csv, math, os, sys
import numpy as np

REPO = "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio"
CACHE = "/home/windows-wsl/gammadiag_cache"
EARTH = 111319.4917
G = 9.80665

TRIALS = [f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]
LOGNUM = {1: dict(zip(TRIALS, [405, 406, 407, 408, 409, 410, 411, 412,
                               413, 414, 415, 416, 417, 418, 419, 420])),
          2: dict(zip(TRIALS, [393, 394, 395, 396, 397, 398, 399, 400,
                               401, 402, 403, 404, 405, 406, 407, 408]))}

# --- node constants (tot_node.py defaults; the contracts leave these unset)
N_VERT = 5.0
N_LAT = 4.0
PN_LEAD = 1.0
PN_LOS_FLOOR = 1.0
PN_NZ_MIN = -1.0          # from contract
PN_NZ_MAX = 3.0           # hard-coded in _pn_cmd
NODE_PITCH_MIN = -80.0    # dive_pitch_min default 0.0 -> falls back to -80
NODE_PITCH_MAX = 45.0
ALPHA_TAU = 1.75
TICK = 0.05
FZ_TGO = 1.5
FZ_R = 100.0
# --- firmware constants (verified in the log PARM stream, dive phase)
FW_PTCH_LIM_MIN = -80.0
FW_PTCH_LIM_MAX = 45.0


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def load_csv(trial, sysid):
    rows = [r for r in csv.DictReader(open(f"{REPO}/runs/artifacts/{trial}/tot_{trial}.csv"))
            if r["sysid"] == str(sysid)]
    t, seen = [], {}
    for r in rows:
        t0 = float(r["t"])
        k = seen.get(t0, 0)
        seen[t0] = k + 1
        t.append(t0 + 0.05 * k)
    return np.array(t), rows


def target(trial):
    for r in csv.DictReader(open(f"{REPO}/runs/artifacts/{trial}/tot_{trial}_geom.csv")):
        if r["kind"] == "target":
            return float(r["lat"]), float(r["lon"])
    raise RuntimeError(trial)


def fit_clock(tcsv, lat, lon, z):
    """Solve t_log = a*t_csv + b.  SITL sim time runs ~1.1x wall time, so a
    constant offset does not fit; regress the nearest-neighbour matches."""
    lt = z["POS.TimeUS"] / 1e6
    lla, llo = z["POS.Lat"], z["POS.Lng"]
    ml = math.cos(math.radians(float(np.mean(lat))))
    idx = np.linspace(0, len(lat) - 1, 300).astype(int)
    ts, dl = [], []
    for i in idx:
        d = np.hypot((lla - lat[i]) * EARTH, (llo - lon[i]) * EARTH * ml)
        j = int(np.argmin(d))
        if d[j] < 25.0:
            ts.append(tcsv[i])
            dl.append(lt[j])
    ts, dl = np.array(ts), np.array(dl)
    a, b = np.polyfit(ts, dl, 1)
    res = dl - (a * ts + b)
    return a, b, float(np.sqrt(np.mean(res ** 2)))


def savgol_rate(t, y, half=6):
    """d y / d t by local linear fit over +-half samples (25 Hz -> +-0.24 s)."""
    out = np.full(len(y), np.nan)
    for i in range(half, len(y) - half):
        s = slice(i - half, i + half + 1)
        tt = t[s] - t[i]
        out[i] = np.polyfit(tt, y[s], 1)[0]
    return out


def q(x, p):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if len(x) else float("nan")


def gate_r_of(trial):
    import json
    v = json.load(open(f"{REPO}/runs/artifacts/{trial}/contract_{trial}.json"))["values"]
    return float(v["gate_r"])


def touchdown_t(trial, sysid):
    for r in csv.DictReader(open(f"{REPO}/matlab/data/truth32.csv")):
        if r["trial"] == trial and r["sysid"] == str(sysid):
            return float(r["t_touch_row_s"])
    return None


def analyse(trial, sysid, dump=None):
    tcsv, rows = load_csv(trial, sysid)
    lat = np.array([float(r["lat"]) for r in rows])
    lon = np.array([float(r["lon"]) for r in rows])
    phase = np.array([r["phase"] for r in rows])
    z = np.load(f"{CACHE}/u{sysid}_{LOGNUM[sysid][trial]}.npz", allow_pickle=True)
    a, b, clkres = fit_clock(tcsv, lat, lon, z)

    dive = phase == "DIVE"
    if dive.sum() < 50:
        return None
    # ---- window = PN ENGAGEMENT ONLY -------------------------------------
    # _pn_step only runs after strike[i] latches, which needs d <= gate_r for
    # gate_hold_n(=3) consecutive ticks (tot_node.py ~4495).  Before that the
    # node is still on the waypoint/TECS path even though phase reads DIVE, so
    # including it would compare the replay against a command PN never issued.
    TLAT0, TLON0 = target(trial)
    dh = np.hypot((TLAT0 - lat) * EARTH,
                  (TLON0 - lon) * EARTH * math.cos(math.radians(float(lat.mean()))))
    gr = gate_r_of(trial)
    hits, i0 = 0, None
    for i in np.where(dive)[0]:
        hits = hits + 1 if dh[i] <= gr else 0
        if hits >= 3:
            i0 = i
            break
    if i0 is None:
        return None
    if os.environ.get("M2_WINDOW") == "phase":
        # counterfactual: the whole DIVE phase, including the ~14 s before PN
        # is handed the aircraft.  Used only to test whether the published
        # 0.063 / +0.217 / 8 s figures are a windowing artefact.
        i0 = int(np.where(dive)[0][0])
    t0c = tcsv[i0]
    t1c = touchdown_t(trial, sysid) or tcsv[dive][-1]
    t1c = min(t1c, tcsv[dive][-1])

    # ---------- measured channels on the ATT grid (25 Hz, sim time) ----------
    tA = z["ATT.TimeUS"] / 1e6
    m = (tA >= a * t0c + b) & (tA <= a * t1c + b)
    tA = tA[m]
    pitch_m = z["ATT.Pitch"][m]                      # [M]
    roll_m = z["ATT.Roll"][m]                        # [M]
    despitch_fw = z["ATT.DesPitch"][m]               # [M] post-firmware clamp

    def at(pref, field, tt=tA):
        return np.interp(tt, z[f"{pref}.TimeUS"] / 1e6, z[f"{pref}.{field}"])

    vn = at("XKF1", "VN"); ve = at("XKF1", "VE"); vd = at("XKF1", "VD")   # [M]
    plat = at("POS", "Lat"); plon = at("POS", "Lng"); palt = at("POS", "Alt")  # [M]
    accz = at("IMU", "AccZ")                                              # [M]
    aoa_fw = at("AOA", "AOA")                                             # [M]
    arsp = at("ARSP", "Airspeed")                                         # [M]
    pid_tar = at("PIDP", "Tar"); pid_act = at("PIDP", "Act")              # [M]
    elev = at("AETR", "Elev")                                             # [M]

    vh = np.hypot(vn, ve)
    V = np.hypot(vh, -vd)
    gam = np.degrees(np.arctan2(-vd, np.maximum(vh, 1e-3)))               # [M]
    gamdot = savgol_rate(tA, gam)                                         # [M]
    nz_meas = -accz / G                                                   # [M]
    alpha_true = pitch_m - gam                                            # [M]

    cg = np.cos(np.radians(gam))
    gdot_ceil_up = np.degrees((PN_NZ_MAX - cg) * G / np.maximum(V, 1))    # [R]
    gdot_ceil_dn = np.degrees((PN_NZ_MIN - cg) * G / np.maximum(V, 1))    # [R]

    # ---------- replay of _pn_cmd on the node's own 20 Hz wall grid ----------
    TLAT, TLON = target(trial)
    tw = np.arange(t0c, t1c, TICK)
    tl = a * tw + b                       # same instants in sim time

    def sam(arr):
        return np.interp(tl, tA, arr)

    r_pitch = sam(pitch_m); r_roll = sam(roll_m)
    r_vn = sam(vn); r_ve = sam(ve); r_vd = sam(vd)
    r_lat = sam(plat); r_lon = sam(plon); r_alt = sam(palt)

    dn = (TLAT - r_lat) * EARTH
    de = (TLON - r_lon) * np.array([mlon(x) for x in r_lat])
    d = np.maximum(1e-3, np.hypot(dn, de))
    h = r_alt                      # target sits at 0 m AMSL (target_amsl=0.0)
    R = np.maximum(1e-3, np.hypot(d, h))
    r_vh = np.hypot(r_vn, r_ve)
    hdot = -r_vd
    r_V = np.maximum(1e-3, np.hypot(r_vh, hdot))
    gamr = np.arctan2(hdot, np.maximum(r_vh, 1e-3))          # rad

    # -- node 1: pure PN longitudinal demand -------------------------------
    sig_dot = (hdot * d - h * (-(dn * r_vn + de * r_ve) / d)) / (R * R)
    gdot_pn = -N_VERT * sig_dot                              # rad/s   [R]
    # -- node 1b: the node's OWN nz clamp ----------------------------------
    nz_raw = np.cos(gamr) + r_V * gdot_pn / G                # [R]
    nz_cl = np.clip(nz_raw, PN_NZ_MIN, PN_NZ_MAX)
    gdot_cl = (nz_cl - np.cos(gamr)) * G / r_V
    gam_cmd_pn = gamr + gdot_cl * PN_LEAD                    # rad     [R]
    # -- node 2: LOS floor --------------------------------------------------
    gam_los = -np.arctan2(h, np.maximum(d, 1e-3))
    gam_cmd = np.minimum(gam_cmd_pn, gam_los * PN_LOS_FLOOR)  # [R]
    los_bites = gam_cmd < gam_cmd_pn - 1e-12
    # -- lateral (needed for the alpha divisor) -----------------------------
    lam_dot = (de * r_vn - dn * r_ve) / (d * d)
    a_h = N_LAT * r_vh * lam_dot
    roll_cmd = np.arctan2(a_h, G * np.maximum(np.cos(gamr), 0.25))        # [R]
    # -- node 3: alpha low-pass and gamma->pitch ---------------------------
    a_raw = np.radians(r_pitch) - gamr           # node sees MAVROS pitch [M]
    alpha_n = np.empty_like(a_raw)
    k = min(1.0, TICK / ALPHA_TAU)
    alpha_n[0] = a_raw[0]
    for i in range(1, len(a_raw)):
        alpha_n[i] = alpha_n[i - 1] + k * (a_raw[i] - alpha_n[i - 1])     # [R]
    pitch_pre = gam_cmd + alpha_n / np.maximum(np.cos(roll_cmd), 0.5)     # [R]
    # -- node 4: node clamp -------------------------------------------------
    pitch_node = np.clip(pitch_pre, math.radians(NODE_PITCH_MIN),
                         math.radians(NODE_PITCH_MAX))
    node_bites = np.abs(pitch_node - pitch_pre) > 1e-9
    # -- node 4b: terminal freeze ------------------------------------------
    tgo = R / np.maximum(r_V, 1e-3)
    frozen = (tgo <= FZ_TGO) | (R <= FZ_R)
    if frozen.any():
        f0 = int(np.argmax(frozen))
        pitch_node = pitch_node.copy()
        pitch_node[f0:] = pitch_node[f0]
    # -- node 5: firmware clamp --------------------------------------------
    lim_min = FW_PTCH_LIM_MIN * np.abs(np.cos(np.radians(r_roll)))        # [R]
    pitch_fw = np.clip(np.degrees(pitch_node), lim_min, FW_PTCH_LIM_MAX)  # [R]
    fw_bites = np.degrees(pitch_node) < lim_min - 1e-9

    # ---------- self-check: replay vs the command the FC actually latched ---
    despitch_at = sam(despitch_fw)
    err = pitch_fw - despitch_at
    ok = np.isfinite(err)

    out = dict(
        trial=trial, sysid=sysid, n_tick=len(tw), n_att=len(tA),
        clk_a=a, clk_res_s=clkres,
        chk_med=q(np.abs(err[ok]), 50), chk_p90=q(np.abs(err[ok]), 90),
        chk_rms=float(np.sqrt(np.nanmean(err[ok] ** 2))),
        chk_frac_1deg=float(np.mean(np.abs(err[ok]) < 1.0)),
        chk_frac_3deg=float(np.mean(np.abs(err[ok]) < 3.0)),
        gdot_pn=q(np.abs(np.degrees(gdot_pn)), 50),
        gdot_pn_p90=q(np.abs(np.degrees(gdot_pn)), 90),
        gdot_nzcl=q(np.abs(np.degrees(gdot_cl)), 50),
        nz_node_bite=float(np.mean((nz_raw < PN_NZ_MIN) | (nz_raw > PN_NZ_MAX))),
        nz_node_lo=float(np.mean(nz_raw < PN_NZ_MIN)),
        nz_node_hi=float(np.mean(nz_raw > PN_NZ_MAX)),
        gdot_cmd_pn=q(np.abs(np.degrees(gam_cmd_pn - gamr) / PN_LEAD), 50),
        gdot_cmd_los=q(np.abs(np.degrees(gam_cmd - gamr) / PN_LEAD), 50),
        gdot_cmd_los_p90=q(np.abs(np.degrees(gam_cmd - gamr) / PN_LEAD), 90),
        los_bite=float(np.mean(los_bites)),
        los_extra_deg=(q(np.degrees(gam_cmd_pn - gam_cmd)[los_bites], 50)
                       if los_bites.any() else 0.0),
        alpha_node=q(np.degrees(alpha_n), 50),
        alpha_true_med=q(alpha_true, 50),
        alpha_true_p90=q(alpha_true, 90),
        alpha_lag_deg=q(np.degrees(alpha_n) - np.degrees(a_raw), 50),
        alpha_lag_abs=q(np.abs(np.degrees(alpha_n - a_raw)), 50),
        alpha_lag_p95=q(np.abs(np.degrees(alpha_n - a_raw)), 95),
        pitch_pre=q(np.degrees(pitch_pre), 50),
        pitch_pre_p05=q(np.degrees(pitch_pre), 5),
        node_bite=float(np.mean(node_bites)),
        node_eat_deg=(q(np.abs(np.degrees(pitch_pre - pitch_node))[node_bites], 50)
                      if node_bites.any() else 0.0),
        fw_bite=float(np.mean(fw_bites)),
        fw_eat_deg=(q((lim_min - np.degrees(pitch_node))[fw_bites], 50)
                    if fw_bites.any() else 0.0),
        fw_limmin=q(lim_min, 50), fw_limmin_p95=q(lim_min, 95),
        roll_abs=q(np.abs(r_roll), 50), roll_abs_p90=q(np.abs(r_roll), 90),
        froze_frac=float(np.mean(frozen)),
        despitch=q(despitch_at, 50), pitch_meas=q(r_pitch, 50),
        track_err_deg=q(np.abs(despitch_at - r_pitch), 50),
        track_err_p90=q(np.abs(despitch_at - r_pitch), 90),
        gam_med=q(gam, 50), gam_min=q(gam, 2), V_med=q(V, 50),
        gdot_act_absmed=q(np.abs(gamdot), 50),
        gdot_act_p90=q(np.abs(gamdot), 90),
        gdot_act_p99=q(np.abs(gamdot), 99),
        gdot_act_max=float(np.nanmax(np.abs(gamdot))),
        ceil_up=q(gdot_ceil_up, 50), ceil_dn=q(gdot_ceil_dn, 50),
        frac_at_ceiling=float(np.nanmean(
            (gamdot > 0.8 * gdot_ceil_up) | (gamdot < 0.8 * gdot_ceil_dn))),
        frac_over_ceiling=float(np.nanmean(
            (gamdot > gdot_ceil_up) | (gamdot < gdot_ceil_dn))),
        nz_meas_med=q(nz_meas, 50), nz_meas_p99=q(nz_meas, 99),
        nz_meas_p01=q(nz_meas, 1), nz_meas_max=float(np.nanmax(nz_meas)),
        nz_meas_min=float(np.nanmin(nz_meas)),
        frac_nz_out=float(np.nanmean((nz_meas > PN_NZ_MAX) | (nz_meas < PN_NZ_MIN))),
        pid_tar_med=q(np.abs(pid_tar), 50), pid_tar_p95=q(np.abs(pid_tar), 95),
        pid_act_med=q(np.abs(pid_act), 50),
        pid_sat=float(np.mean(np.abs(pid_tar) > 89.0)),
        elev_sat=float(np.mean(np.abs(elev) > 4400)),
        aoa_fw_med=q(aoa_fw, 50), aoa_fw_p99=q(aoa_fw, 99),
        arsp_med=q(arsp, 50),
    )
    if dump:
        np.savez_compressed(
            dump, tw=tw, tl=tl, gdot_pn=np.degrees(gdot_pn), gdot_cl=np.degrees(gdot_cl),
            gam_cmd_pn=np.degrees(gam_cmd_pn), gam_cmd=np.degrees(gam_cmd),
            gam_los=np.degrees(gam_los), gamr=np.degrees(gamr),
            alpha_n=np.degrees(alpha_n), a_raw=np.degrees(a_raw),
            pitch_pre=np.degrees(pitch_pre), pitch_node=np.degrees(pitch_node),
            lim_min=lim_min, pitch_fw=pitch_fw, despitch=despitch_at,
            r_pitch=r_pitch, r_roll=r_roll, roll_cmd=np.degrees(roll_cmd),
            nz_raw=nz_raw, R=R, d=d, h=h,
            tA=tA, gam=gam, gamdot=gamdot, nz_meas=nz_meas,
            ceil_up=gdot_ceil_up, ceil_dn=gdot_ceil_dn, V=V,
            pitch_m=pitch_m, despitch_fw=despitch_fw, alpha_true=alpha_true,
            aoa_fw=aoa_fw, pid_tar=pid_tar, pid_act=pid_act, elev=elev)
    return out


if __name__ == "__main__":
    res = []
    for tr in TRIALS:
        for s in (1, 2):
            dp = f"{CACHE}/" + os.environ.get("M2_PFX", "dump") + f"_{tr}_U{s}.npz"
            try:
                r = analyse(tr, s, dump=dp)
            except Exception as e:
                print(tr, s, "FAIL", type(e).__name__, e)
                continue
            if r:
                res.append(r)
    keys = list(res[0])
    with open(f"{REPO}/matlab/gammadiag/chain32_" + os.environ.get("M2_PFX","dump") + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        for r in res:
            w.writerow(r)
    print("wrote", len(res), "sorties")
