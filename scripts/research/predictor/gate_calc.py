import csv, math, os, glob

G0 = 9.80665
R_E = 6378137.0

def hav(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat = lat2 - lat1; dlon = lon2 - lon1
    h = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2*R_E*math.asin(min(1.0, math.sqrt(h)))

base = r"C:\Users\joy46\Documents\Projects\aeroplan-studio\runs\artifacts"
trials = ["RPK1","RPK2","RPK3","RPL1","RPL2","RPL3"]

print("=== 實測交棒點狀態(從 CSV 取 d 首次 <= 4800 的那一列)===")
print(f"{'trial':6} {'uav':3} {'d(m)':>7} {'agl(m)':>8} {'gs':>7} {'vz':>7} {'V':>7} {'gam':>6} {'gamma_LOS':>9}")
samples = []
for t in trials:
    tgt = None
    with open(os.path.join(base, t, f"tot_{t}_geom.csv")) as f:
        for row in csv.DictReader(f):
            if row["kind"] == "target":
                tgt = (float(row["lat"]), float(row["lon"])); break
    rows = {}
    with open(os.path.join(base, t, f"tot_{t}.csv")) as f:
        for r in csv.DictReader(f):
            rows.setdefault(int(r["sysid"]), []).append(r)
    for sysid, rs in sorted(rows.items()):
        prev = None
        for r in rs:
            d = hav((float(r["lat"]), float(r["lon"])), tgt)
            if prev is not None and d <= 4800.0 and r["phase"] == "DIVE":
                dt = float(r["t"]) - float(prev["t"])
                agl = float(r["agl"])
                vz = (float(prev["alt"]) - float(r["alt"])) / dt if dt > 0 else 0.0
                gs = float(r["gs"])
                V = math.hypot(gs, vz)
                gam = math.degrees(math.atan2(vz, max(gs, 1e-6)))
                los = math.degrees(math.atan2(agl, max(d, 1.0)))
                print(f"{t:6} {sysid:3d} {d:7.0f} {agl:8.0f} {gs:7.1f} {vz:7.1f} {V:7.1f} {gam:6.1f} {los:9.1f}")
                samples.append((t, sysid, d, agl, gs, vz, V, gam))
                break
            prev = r

def gate_of(h, gs, vz, th_deg, phi=0.80, r_min=2500.0, r_max=6000.0, ring=6084.0,
            nz_min=0.05):
    V = math.hypot(gs, vz)
    gam = math.atan2(vz, max(gs, 1e-6))
    den = math.cos(gam) - nz_min
    th = math.radians(th_deg)
    r_turn = V*V/(G0*den)
    h_arc = r_turn*(1.0 - math.cos(th))
    if h > h_arc:
        d_min = r_turn*math.sin(th) + (h - h_arc)/math.tan(th)
    else:
        d_min = math.sqrt(max(h*(2.0*r_turn - h), 0.0))
    g = d_min/phi
    return d_min, r_turn, h_arc, g, max(r_min, min(g, r_max, ring))

print()
print("=== 試算:perch_alt=3100 dive_angle=27 ring=6084 phi=0.80 gate_r_min=2500 gate_r_max=6000 ===")
for V_label, gs, vz in [("V=111 (vmin95 EAS -> TAS@3100, near level)", 111.0, 0.0),
                        ("V=157 (KPI 巡航地速, near level)", 157.0, 0.0),
                        ("V=157 gs, vz=20 (輕微下降)", 157.0, 20.0),
                        ("V=95  (vmin EAS 名目)", 95.0, 0.0)]:
    print(f"\n-- {V_label} --")
    print(f"{'th_max':>7} {'r_turn':>8} {'h_arc':>8} {'d_min':>9} {'g=d_min/phi':>12} {'clamped':>9} {'binding':>16}")
    for th in (25, 28, 30, 33, 35):
        for h in (3100.0,):
            d_min, r_turn, h_arc, g, gc = gate_of(h, gs, vz, th)
            if gc == 2500.0 and g < 2500.0: b = "gate_r_min"
            elif abs(gc - 6000.0) < 1e-6 and g > 6000.0: b = "gate_r_max"
            elif abs(gc - 6084.0) < 1e-6: b = "ring"
            else: b = "-(未夾,動態)"
            print(f"{th:7d} {r_turn:8.0f} {h_arc:8.0f} {d_min:9.0f} {g:12.0f} {gc:9.0f} {b:>16}")

print()
print("=== 高度散布吸收:同 theta_max、h 從 2286 到 3510 ===")
for gs, vz, lab in [(157.0, 0.0, "V=157"), (111.0, 0.0, "V=111"), (140.0, 30.0, "gs=140 vz=30")]:
    print(f"\n-- {lab} --")
    print(f"{'th':>4} " + " ".join(f"h={h:<6.0f}" for h in (2286, 2600, 2900, 3200, 3510)) + "   spread(clamped)")
    for th in (25, 28, 30, 33, 35):
        raws, cls = [], []
        for h in (2286.0, 2600.0, 2900.0, 3200.0, 3510.0):
            d_min, r_turn, h_arc, g, gc = gate_of(h, gs, vz, th)
            raws.append(g); cls.append(gc)
        print(f"{th:4d} " + " ".join(f"{c:7.0f}" for c in cls) +
              f"   raw {min(raws):.0f}-{max(raws):.0f} (Δ{max(raws)-min(raws):.0f}) "
              f"clamped Δ{max(cls)-min(cls):.0f}")

print()
print("=== 用實測交棒點的真實 (h, gs, vz) 代入 ===")
for th in (25, 28, 30, 33, 35):
    print(f"\n-- theta_max={th} deg --")
    for (t, sysid, d, agl, gs, vz, V, gam) in samples:
        d_min, r_turn, h_arc, g, gc = gate_of(agl, gs, vz, th)
        print(f"  {t} UAV{sysid}: h={agl:5.0f} V={V:6.1f} gam={gam:5.1f} "
              f"r_turn={r_turn:6.0f} d_min={d_min:7.0f} raw={g:7.0f} -> gate={gc:5.0f}")

print()
print("=== 要讓動態交棒不被夾住,ring 需要多大 / 或 gate_r_max 需要多大 ===")
for gs, vz, lab in [(157.0, 0.0, "V=157"), (111.0, 0.0, "V=111")]:
    print(f"-- {lab} --")
    for th in (25, 28, 30, 33, 35):
        d_min, r_turn, h_arc, g, gc = gate_of(3100.0, gs, vz, th)
        # ring needed to be >= g
        need_ang = math.degrees(math.atan2(3100.0, g))
        print(f"  th={th}: raw gate={g:.0f} -> 需要 ring>={g:.0f} "
              f"⇒ dive_angle <= {need_ang:.1f} deg (perch 3100)")
