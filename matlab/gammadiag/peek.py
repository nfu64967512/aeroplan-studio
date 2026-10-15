import numpy as np, sys
tag = sys.argv[1] if len(sys.argv) > 1 else "RRE1_U1"
step = int(sys.argv[2]) if len(sys.argv) > 2 else 40
z = np.load(f"/home/windows-wsl/gammadiag_cache/dump_{tag}.npz")
n = len(z["tw"])
print("n_tick", n)
hdr = ["tw", "gamr", "gam_los", "gcmd_pn", "gam_cmd", "a_n", "a_raw",
       "p_pre", "p_node", "limmin", "p_fw", "DesP", "pitch", "roll", "R"]
print(" ".join(h.rjust(8) for h in hdr))
keys = ["tw", "gamr", "gam_los", "gam_cmd_pn", "gam_cmd", "alpha_n", "a_raw",
        "pitch_pre", "pitch_node", "lim_min", "pitch_fw", "despitch",
        "r_pitch", "r_roll", "R"]
for i in range(0, n, step):
    print(" ".join(f"{float(z[k][i]):8.2f}" for k in keys))
