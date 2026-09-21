# -*- coding: utf-8 -*-
"""Where does the pre-arm spawn displacement live: in the airframe, or in the estimate?

The gate compares the vehicle's REPORTED position (GLOBAL_POSITION_INT, i.e. the
EKF) against the --home we handed SITL.  That cannot tell a vehicle that moved
from an estimator that drifted.  The dataflash can: SIM.Lat/Lng is the
simulator's TRUTH state, POS.Lat/Lng is the EKF.  Both are logged from the
moment of arming, and in these sorties two of the three aircraft sit armed and
parked for minutes before their release slot, with the wind already blowing.

Also dumps the body-frame specific force (IMU.AccX/AccY) and the attitude
(ATT.Yaw) over that parked window, because the candidate mechanism is that the
horizontal aerodynamic acceleration is NOT removed from accel_body even though
the ground model zeroes the horizontal velocity.

    python spawn_probe.py <TRIAL> <lognum>
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "sorties2"
INST = {"A1": 100, "B2": 101, "B3": 102}
M_LAT = 111132.0


def main():
    trial, lognum = sys.argv[1], int(sys.argv[2])
    home = json.loads((ROOT / "runs" / "artifacts" / trial /
                       "spawn_readback.json").read_text(encoding="utf-8"))
    want = {r["uav"]: (r["want_lat"], r["want_lon"]) for r in home["rows"]}
    res = {"trial": trial, "log": lognum, "per_uav": {}}
    for uav, i in INST.items():
        p = ROOT / f"sitl/vtol/i{i}/logs/{lognum:08d}.BIN"
        m = mavutil.mavlink_connection(str(p), dialect="ardupilotmega")
        sim, pos, imu, att, agl = [], [], [], [], []
        while True:
            msg = m.recv_match(type=["SIM", "POS", "IMU", "ATT", "CTUN"])
            if msg is None:
                break
            t = float(msg.TimeUS) / 1e6
            ty = msg.get_type()
            if ty == "SIM":
                sim.append((t, float(msg.Lat), float(msg.Lng), float(msg.Yaw)))
            elif ty == "POS":
                pos.append((t, float(msg.Lat), float(msg.Lng),
                            float(getattr(msg, "RelHomeAlt", 0.0))))
            elif ty == "IMU" and getattr(msg, "I", 0) == 0:
                imu.append((t, float(msg.AccX), float(msg.AccY), float(msg.AccZ)))
            elif ty == "ATT":
                att.append((t, float(msg.Yaw), float(msg.Roll), float(msg.Pitch)))
        for a in (sim, pos, imu, att):
            a.sort()
        wlat, wlon = want[uav]
        mlon = 111412.0 * math.cos(math.radians(wlat))

        def off(lat, lon):
            return ((lat - wlat) * M_LAT, (lon - wlon) * mlon)

        # the parked window: from the first log sample until the aircraft first
        # climbs above 2 m (POS.RelHomeAlt), i.e. armed but still on the ground
        t_lift = next((t for t, _, _, a in pos if a > 2.0), None)
        t0 = pos[0][0] if pos else sim[0][0]
        tw = t_lift if t_lift is not None else (t0 + 10.0)
        simw = [s for s in sim if s[0] <= tw]
        posw = [q for q in pos if q[0] <= tw]
        imuw = [x for x in imu if x[0] <= tw]
        attw = [x for x in att if x[0] <= tw]

        def stat(rows, kind):
            if not rows:
                return None
            first, last = rows[0], rows[-1]
            dn0, de0 = off(first[1], first[2])
            dn1, de1 = off(last[1], last[2])
            dmax = max(rows, key=lambda r: math.hypot(*off(r[1], r[2])))
            dnm, dem = off(dmax[1], dmax[2])
            return {"kind": kind, "n": len(rows),
                    "first_d_north_m": round(dn0, 3), "first_d_east_m": round(de0, 3),
                    "last_d_north_m": round(dn1, 3), "last_d_east_m": round(de1, 3),
                    "max_offset_m": round(math.hypot(dnm, dem), 3),
                    "max_d_north_m": round(dnm, 3), "max_d_east_m": round(dem, 3),
                    "bearing_at_max_deg": round(math.degrees(math.atan2(dem, dnm)) % 360.0, 2)}

        row = {"log": str(p.relative_to(ROOT)).replace("\\", "/"),
               "parked_window_s": round(tw - t0, 1),
               "t_first_above_2m_s": None if t_lift is None else round(t_lift, 1),
               "truth_SIM": stat(simw, "simulator truth (SIM.Lat/Lng)"),
               "estimate_POS": stat(posw, "EKF estimate (POS.Lat/Lng)")}
        if imuw:
            ax = statistics.median([x[1] for x in imuw])
            ay = statistics.median([x[2] for x in imuw])
            row["imu0_parked_median"] = {
                "n": len(imuw),
                "AccX_mps2": round(ax, 4), "AccY_mps2": round(ay, 4),
                "AccZ_mps2": round(statistics.median([x[3] for x in imuw]), 4),
                "horizontal_mag_mps2": round(math.hypot(ax, ay), 4),
                "horizontal_bearing_deg": round(math.degrees(math.atan2(ay, ax)) % 360.0, 2),
                "note": "body frame; yaw is ~0 so +X is north and +Y is east"}
        if attw:
            row["att_parked"] = {
                "yaw_deg_median": round(statistics.median([x[1] for x in attw]), 3),
                "yaw_deg_min": round(min(x[1] for x in attw), 3),
                "yaw_deg_max": round(max(x[1] for x in attw), 3),
                "roll_deg_median": round(statistics.median([x[2] for x in attw]), 3),
                "pitch_deg_median": round(statistics.median([x[3] for x in attw]), 3)}
        res["per_uav"][uav] = row
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{trial}_spawn_probe.json").write_text(
        json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
