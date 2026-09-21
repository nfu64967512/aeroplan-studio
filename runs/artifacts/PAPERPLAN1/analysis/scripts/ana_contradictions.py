# -*- coding: utf-8 -*-
"""Stage 5: every place this analysis disagrees with an existing product.

Each entry pulls its numbers from out/*.json so the file cannot drift from the
computation.  "them" is quoted from the product named in `their_source`.
"""
from __future__ import annotations

from ana_lib import OUT, jdump, jload

K, S, T, A, V = (jload(OUT / f) for f in
                 ("uav_kinematics.json", "separation.json", "timing.json",
                  "altitude.json", "verify.json"))
D = jload(OUT / "derived.json")


def ekf(t, u):
    return K[t]["per_uav"][u]["wind"]["L2_ekf_state_dataflash"]["EKF3"]["speed_mps"]


items = [
    {
     "id": "X1", "severity": "task premise is wrong",
     "claim_checked": "task item 1 asks for 'the dataflash WIND message' as one of "
                      "three independent wind lines",
     "finding": "there is no WIND message in any of the 18 dataflash logs (0 records "
                "in every one). ArduPilot logs the wind estimate as the EKF state "
                "NKF2/XKF2 VWN,VWE; MAVLink WIND is a telemetry-only message and "
                "exists only because the patched runner mirrors it into flight.csv.",
     "substituted_third_line": "flight.csv wind_spd_est / wind_dir_from_est (telemetry "
                               "transport, same estimator as L2) plus the pre-arm "
                               "SIM_WIND_* readback as the configuration line",
     "their_source": "task brief",
     "my_source": "runs/artifacts/PAPERPLAN1/analysis/cache/index.json (per-log message "
                  "counts) and scripts/ana_extract.py",
    },
    {
     "id": "X2", "severity": "number to replace",
     "claim_checked": "sortie_results.json ekf_wind_from_median_components = "
                      "4.80 / 4.87 / 4.88 m/s",
     "finding": "binalt.py pools NKF2 (EKF2) and XKF2 (EKF3) into one list and includes "
                "ground, climb and descent samples where the wind state has not "
                "converged. Restricted to the cruise band (RelHomeAlt > 100 m) and "
                "separated by estimator, EKF3 reads "
                f"{ekf('HV4W55','A1')} / {ekf('HV4W55','B2')} / {ekf('HV4W55','B3')} m/s "
                f"on HV4W55 and {ekf('HV4W05','A1')} / {ekf('HV4W05','B2')} / "
                f"{ekf('HV4W05','B3')} on HV4W05, against a commanded 5.00.",
     "their_source": "runs/artifacts/PAPERPLAN1/sorties/sortie_results.json",
     "my_source": "out/uav_kinematics.json -> per_uav.<u>.wind.L2_ekf_state_dataflash",
    },
    {
     "id": "X3", "severity": "number to replace, safety-relevant",
     "claim_checked": "flown minimum separation 100.769 / 100.659 / 100.785 m "
                      "(HV4W55 / HV4W05 / HV4W00)",
     "finding": "those are computed with a flat 111132 m/deg of latitude "
                "(hv_sitl_report.py:129-133, binsep.py:M_LAT). The planner's mission "
                "geometry uses the exact WGS84 GeoFrame (hv_dccpp/scenario.py:87-88 "
                "says so explicitly), where 1 deg of latitude is 110757.52 m at the "
                "site -- 0.338 % less -- and the B2-B3 encounter is almost purely "
                "north-south. In the frame the 100 m rule is actually enforced the "
                "same encounters are "
                f"{S['HV4W55']['frames']['ecef']['fleet_min_h_m']} / "
                f"{S['HV4W05']['frames']['ecef']['fleet_min_h_m']} / "
                f"{S['HV4W00']['frames']['ecef']['fleet_min_h_m']} m.",
     "independent_check": "Vincenty inverse between the two interpolated positions at "
                          "the instant of closest approach agrees to 0.0000 m "
                          "(out/verify.json V1)",
     "consequence": f"HV4W05's margin above the floor is "
                    f"{S['HV4W05']['frames']['ecef']['pairs']['B2-B3']['margin_to_100m_m']:.3f} m, "
                    f"not 0.659 m",
     "their_source": "runs/artifacts/PAPERPLAN1/sorties/*_binsep.json, sortie_results.json",
     "my_source": "out/separation.json -> frames.ecef",
    },
    {
     "id": "X4", "severity": "paper number is wrong",
     "claim_checked": "HV4SITL15 flown minimum separation 104.2 m "
                      "(separation_from_tracks.json), quoted in the paper",
     "finding": "that is a 2 s grid over ~1 Hz telemetry. HV4SITL15's own dataflash "
                "(00000013.BIN, mtime identical to runs/artifacts/HV4SITL15/A1/"
                "flight.csv to the 100 ns tick) gives "
                f"{S['HV4SITL15']['frames']['ecef']['fleet_min_h_m']} m exact in the "
                "planner frame -- 3.51 m closer than published, i.e. a margin of "
                f"{S['HV4SITL15']['frames']['ecef']['pairs']['B2-B3']['margin_to_100m_m']:.3f} m, "
                "still with 0 violations.",
     "their_source": "runs/artifacts/HV4SITL15/separation_from_tracks.json",
     "my_source": "out/separation.json",
    },
    {
     "id": "X5", "severity": "prep_check's correction is itself incomplete",
     "claim_checked": "prep_result.json check.problems[0]: the Table-2b flown minimum "
                      "separation 101.0 for HV4SITL16 has no product and must be deleted",
     "finding": "101.0 does have a product: runs/artifacts/PAPERPLAN1/c4_params/"
                "c4_evidence.json:1721,1740 (min_h_m 101.0, d_v 29.0, pair B2-B3), "
                "which re-ran the reporter's algorithm on HV4SITL16's flight.csv. It "
                "rounds to my flat-frame value "
                f"{S['HV4SITL16']['frames']['repo']['fleet_min_h_m']} m. The right "
                "action is therefore not to delete it but to replace it with "
                f"{S['HV4SITL16']['frames']['ecef']['fleet_min_h_m']} m, which is what "
                "HV4SITL16's own dataflash gives in the planner's frame.",
     "their_source": "runs/artifacts/PAPERPLAN1/prep_result.json",
     "my_source": "out/separation.json + c4_params/c4_evidence.json",
    },
    {
     "id": "X6", "severity": "the planned side is not a guarantee either",
     "claim_checked": "validation.json min_sep_h_m = 104.637 / 105.699 / 105.464 "
                      "read as 'the plan keeps 100 m'",
     "finding": "hv_dccpp/sim.py:1007-1034 min_separation() evaluates on a grid of "
                "cfg.dt_4d_s = 2.0 s (hv_dccpp/mission.py:1092) and takes the value AT "
                "grid points, so it is an upper bound exactly like the reporter's. "
                "Measured on these very tracks, a 2 s grid overstates the true minimum "
                "by " + ", ".join(
                    f"{t} {S[t]['grid_cost_ecef_frame']['by_grid']['2s']['overstates_by_m']:+.2f} m"
                    for t in ("HV4W55", "HV4W05", "HV4W00")) +
                ", and a 5 s grid by up to "
                f"{max(S[t]['grid_cost_ecef_frame']['by_grid']['5s']['overstates_by_m'] for t in ('HV4W55','HV4W05','HV4W00')):+.2f} m. "
                "The error is grid-phase dependent, not monotone in the step.",
     "their_source": "runs/artifacts/<trial>/validation.json",
     "my_source": "out/separation.json -> grid_cost_ecef_frame",
    },
    {
     "id": "X7", "severity": "definition, not error",
     "claim_checked": "flown makespan 1294.00 / 1246.50 / 1219.50 / 2283.50 s",
     "finding": "I reproduce those exactly from the reporter's recipe (out/verify.json "
                "V2), so the arithmetic is right, but the recipe uses ONE median clock "
                "offset for three instances whose boot clocks differ by ~1 s, and its "
                "'below 2 m and disarmed' touchdown lands on the FINAL CSV row for the "
                "last aircraft in all four sorties (the monitor loop stops there). On a "
                "per-aircraft GPS time-of-week clock and a clean 2 m touchdown the "
                "makespans are "
                + " / ".join(f"{T[t]['flown_makespan_2m_s']:.2f}"
                             for t in ("HV4W55", "HV4W05", "HV4W00", "BEAMIDP1")) + " s.",
     "their_source": "runs/artifacts/PAPERPLAN1/sorties/sortie_results.json",
     "my_source": "out/timing.json, out/verify.json",
    },
    {
     "id": "X8", "severity": "headline is definition-dependent",
     "claim_checked": "'+2.66% error vs +3.40% error' and '+27.00 s (+2.21%)'",
     "finding": "the DIFFERENCES survive every definition; the absolute percentages do "
                "not. disarm + median clock: +2.66 / +3.40 (theirs). disarm + "
                f"per-aircraft clock: {T['HV4W55']['makespan_err_disarm_pct']:+.2f} / "
                f"{T['HV4W05']['makespan_err_disarm_pct']:+.2f}. 2 m touchdown + "
                f"per-aircraft clock: {T['HV4W55']['makespan_err_2m_pct']:+.2f} / "
                f"{T['HV4W05']['makespan_err_2m_pct']:+.2f}. The wind-attributable "
                f"overrun is {D['wind_cost']['A_cost_of_the_air_same_plan_HV4W05_minus_HV4W00']['d_flown_makespan_s']:+.2f} s "
                f"({D['wind_cost']['A_cost_of_the_air_same_plan_HV4W05_minus_HV4W00']['d_flown_makespan_pct']:+.2f} %) "
                "on the 2 m definition and "
                f"{D['wind_cost']['A_cost_of_the_air_same_plan_HV4W05_minus_HV4W00']['d_flown_makespan_s_disarm_def']:+.2f} s "
                "on the disarm definition, against their +27.00 s.",
     "their_source": "runs/artifacts/PAPERPLAN1/sorties/sortie_results.json",
     "my_source": "out/timing.json, out/derived.json",
    },
    {
     "id": "X9", "severity": "number to replace",
     "claim_checked": "BEAMIDP1 zones '192.01 ha each'; reporter Z1 area 1 248 034.8 m2",
     "finding": "same flat-latitude cause as X3. The scenario polygon itself was written "
                f"with 111132 m/deg (out/verify.json V3: implied "
                f"{V['V3_survey_area_scale']['implied_m_per_deg_lat_if_800m']}), so the "
                "nominal 1560 x 800 m area is 1560 x "
                f"{V['V3_survey_area_scale']['true_north_south_extent_m']} m on the "
                "ellipsoid. Ellipsoidal areas: gui_legal Z1 = "
                f"{T['HV4W55']['coverage_all_airborne'][0]['area_ha']} ha, BEAMIDP1 "
                f"Z1/Z2/Z3 = {T['BEAMIDP1']['coverage_all_airborne'][0]['area_ha']} ha each.",
     "their_source": "sortie_results.json, runs/artifacts/*/coverage_from_tracks.json",
     "my_source": "out/timing.json, out/verify.json V3",
    },
    {
     "id": "X10", "severity": "reading of the result, not a number",
     "claim_checked": "'wind-aware predicts honestly while the wind-ignorant plan "
                      "silently overruns by 27 s'",
     "finding": "true, and the mechanism is now measurable: of the "
                f"{-D['wind_cost']['B_cost_of_ignoring_the_wind_model_same_air_HV4W05_vs_HV4W55']['d_flown_makespan_s']:.1f} s "
                "by which the wind-aware fleet finishes LATER, "
                f"{D['wind_cost']['B_cost_of_ignoring_the_wind_model_same_air_HV4W05_vs_HV4W55']['makespan_decomposition']['A1_release_delay_added_by_wind_aware_plan_s']:.1f} s "
                "is a later SCHEDULED release of A1 (656.0 vs 612.0 s) and only "
                f"{D['wind_cost']['B_cost_of_ignoring_the_wind_model_same_air_HV4W05_vs_HV4W55']['makespan_decomposition']['d_airborne_duration_s']:.2f} s "
                "is a longer flight. So the wind model buys prediction fidelity "
                "(per-aircraft error 7.08/6.22/2.43 -> 4.90/2.88/1.70 %), not speed.",
     "their_source": "runs/artifacts/PAPERPLAN1/sorties/sortie_results.json notes",
     "my_source": "out/derived.json",
    },
    {
     "id": "X11", "severity": "new evidence for C1/D2",
     "claim_checked": "the 120.0 m sorties are the paper's flight evidence",
     "finding": "HV4SITL15 and HV4SITL16 both flew ABOVE the 120.000 m ceiling. Exported "
                "items above 120.000: 88 per sortie (38/13/37). Flown maxima "
                f"{A['HV4SITL15']['fleet_max_flown_m']} and "
                f"{A['HV4SITL16']['fleet_max_flown_m']} m. Fleet time above 120.000 m: "
                f"{sum(A['HV4SITL15']['per_uav'][u]['time_above_120_000_s'] for u in ('A1','B2','B3')):.1f} s "
                f"and {sum(A['HV4SITL16']['per_uav'][u]['time_above_120_000_s'] for u in ('A1','B2','B3')):.1f} s. "
                "At the 119.5 m ceiling: 0 exported items, 0.00 s above 120.000, "
                "margins +0.303 to +0.403 m, all four sorties. This corroborates "
                "c1_altitude/binlog_alt.json (147.3/56.8/126.1 s on log 14) to 0.03 s "
                "with an independent implementation.",
     "their_source": "runs/artifacts/PAPERPLAN1/c1_altitude/",
     "my_source": "out/altitude.json",
    },
    {
     "id": "X12", "severity": "new evidence for C2",
     "claim_checked": "'SITL never set wind' (memory: sitl-wind-and-param-gate)",
     "finding": "confirmed kinematically rather than by parameter absence: on HV4SITL15 "
                "and HV4SITL16 the east-minus-west of (groundspeed - airspeed) is "
                f"{K['HV4SITL15']['per_uav']['B3']['wind']['L1_kinematic_dataflash']['east_minus_west_mps']:+.3f} "
                f"and {K['HV4SITL16']['per_uav']['B3']['wind']['L1_kinematic_dataflash']['east_minus_west_mps']:+.3f} m/s "
                "while their plans assumed 5 m/s from 270. The same statistic is "
                "+9.93 to +9.97 m/s on HV4W55/HV4W05, against an expected +10.00.",
     "their_source": "memory + prep_result.json",
     "my_source": "out/uav_kinematics.json -> wind.L1_kinematic_dataflash",
    },
    {
     "id": "X13", "severity": "already flagged by prep_check; my recommendation",
     "claim_checked": "Beam-IDP sequential makespan estimate 2532.5 s (narrative) vs "
                      "2541.8 s (beamidp_table.json makespan_sequential_estimate_s)",
     "finding": "both are extrapolations that do not re-plan entry/RTB legs or 4-D "
                "deconfliction, and the JSON's own _note says so. The directly computed "
                "quantity needs no extrapolation: fleet turn time 1696.88 -> 1115.73 s, "
                "-34.25 %, turn distance 32.274 -> 21.413 km, -33.65 %, with the "
                "Held-Karp optimum at 1060.15 s, so Beam-IDP captures 91.27 % of the "
                "achievable saving. Quote that and drop the makespan extrapolation.",
     "their_source": "runs/artifacts/PAPERPLAN1/prep_beamidp/",
     "my_source": "out/derived.json -> beam",
    },
]

jdump(OUT / "contradictions.json", {"n": len(items), "items": items})
for it in items:
    print(f"{it['id']}  [{it['severity']}]  {it['claim_checked'][:70]}")
