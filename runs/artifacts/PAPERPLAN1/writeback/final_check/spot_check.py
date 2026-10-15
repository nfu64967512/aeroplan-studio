# Spot-check numbers printed in cowork_brief.md against update_paper/out/NUMBERS.json (+ metrics.json).
# Read-only. Each check: (label, string that must appear in the brief, value recomputed from JSON).
import io
import json
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
R = 'C:/Users/joy46/Documents/Projects/aeroplan-studio/'
brief = open(R + 'docs/hv_dccpp/AASRC2026_abstract/cowork_brief.md', encoding='utf-8').read()
N = json.load(open(R + 'runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json', encoding='utf-8'))
M = json.load(open(R + 'runs/artifacts/PAPERPLAN1/update_paper/out/metrics.json', encoding='utf-8'))
T = N['trials']
C = N['ceiling_control']
G = N['survey_geometry']
w55 = T['HV4W55']
pu = M['HV4W55']['timing']['per_uav']
U = ('A1', 'B2', 'B3')


def f(x, n):
    return f'{x:.{n}f}'


def sig(k, fl='signal_100m'):
    return list(T[k]['wind_test'][fl].values())


ekf270 = [T[k]['wind_test']['ekf_wind'][u]['spd_mps'] for k in ('HV4W05', 'HV4W55', 'HV4W55R') for u in U]
ekf180 = [T['HV4W55S']['wind_test']['ekf_wind'][u]['spd_mps'] for u in U]
dist = [pu[u]['dist_err_pct'] for u in U]
exp_dev = [T[k]['altitude']['exported_max_rel_m'] - T[k]['ceiling_m'] for k in T]
trk = [v for k in T for v in T[k]['altitude']['tracking_excess_m'].values()]
null55s = list(T['HV4W55S']['wind_test']['null_30m'].values())
spawn = N['corrections']['K3_K4_spawn']

checks = [
    ('area km2', '1.2434 km²', f(G['area_km2'], 4)),
    ('swath/spacing @119.5', '刈幅 137.99 m、間距 96.59 m',
     f(G['swath_and_spacing']['119.5']['swath_m'], 2) + '/' + f(G['swath_and_spacing']['119.5']['spacing_m'], 2)),
    ('coverage', '99.97%（上界）', f(100 * w55['coverage']['areas'][0]['ratio'], 2)),
    ('max AGL per uav', '119.622／119.636／119.675',
     '/'.join(f(v, 3) for v in w55['altitude']['flown_max_agl_per_uav'].values())),
    ('abstract max AGL', '119.68 m AGL', f(w55['altitude']['flown_max_agl_m'], 2)),
    ('exported max', '≤119.553', f(w55['altitude']['exported_max_rel_m'], 3)),
    ('flown km', '10.69／8.30／12.91', '/'.join(f(pu[u]['flown_km'], 2) for u in U)),
    ('total dist err', '（+4.0%）', f(w55['distance']['err_pct'], 1)),
    ('per-uav dist err range', '3.7%–4.7%', f(min(dist), 2) + '–' + f(max(dist), 2)),
    ('makespan', '1281.9（+1.70%）',
     f(w55['makespan']['planned_s'], 1) + '→' + f(w55['makespan']['flown_s'], 1) + ' ' + f(w55['makespan']['err_pct'], 2)),
    ('planned sep', '| 105.70 |', f(w55['separation']['planned_m'], 2)),
    ('flown sep/vert/samples', '100.43（垂直 28.4；違規 0/7309）',
     f(w55['separation']['flown_exact_m'], 2) + '/' + f(w55['separation']['vert_at_min_m'], 1) + '/'
     + str(w55['separation']['n_violations']) + '/' + str(w55['separation']['n_paired_samples'])),
    ('params', '91／96、逐位元為 85／96',
     w55['params']['strict_float32'] + ' ' + w55['params']['strict_exact'] + ' ' + str(w55['params']['n_ok']) + '/'
     + str(w55['params']['n_rows'])),
    ('mission items', '82／53／84', '/'.join(str(v) for v in w55['items']['per_uav'].values())),
    ('exported >120 @120.0', '匯出 88 個航點', str(C['deltas']['exported_above_120'])),
    ('HV4W55R max', '120.174 m', f(C['flown']['max_agl_m'][1], 3)),
    ('HV4W55R t>120', '354.93 s', f(C['flown']['t_above_120_s'][1], 2)),
    ('ceiling cost', '+0.0279 s', f(C['planned_makespan_cost_of_1195_s'], 4)),
    ('EKF wind', '4.93–4.97 m/s',
     '270deg trials ' + f(min(ekf270), 2) + '–' + f(max(ekf270), 2) + ' ; HV4W55S ' + f(min(ekf180), 2) + '–'
     + f(max(ekf180), 2)),
    ('W05-W00 flown', '26.90 s', f(T['HV4W05']['makespan']['flown_s'] - T['HV4W00']['makespan']['flown_s'], 2)),
    ('pred err W55 vs W05', '+1.70% 對 +2.43%',
     f(w55['makespan']['err_pct'], 2) + ' / ' + f(T['HV4W05']['makespan']['err_pct'], 2)),
    ('W55-W05 flown', '47.13 s', f(w55['makespan']['flown_s'] - T['HV4W05']['makespan']['flown_s'], 2)),
    ('A1 release delay', '44.0 s',
     f(pu['A1']['plan_t_takeoff_s'] - M['HV4W05']['timing']['per_uav']['A1']['plan_t_takeoff_s'], 1)),
    ('crosswind residual max', '≤ 0.041 m/s',
     f(max(v['residual_mps'] for v in N['crosstrack_wind_triangle'].values()), 3)),
    ('T3 SITL16 t>120', '| 330.14 |', f(T['HV4SITL16']['altitude']['t_above_120_s'], 2)),
    ('T3 W00 makespan', '1205.5→1207.9', f(T['HV4W00']['makespan']['planned_s'], 1) + '→' + f(T['HV4W00']['makespan']['flown_s'], 1)),
    ('T3 W05 makespan', '1205.5→1234.8', f(T['HV4W05']['makespan']['planned_s'], 1) + '→' + f(T['HV4W05']['makespan']['flown_s'], 1)),
    ('T3 W55R makespan', '1260.4→1282.2', f(T['HV4W55R']['makespan']['planned_s'], 1) + '→' + f(T['HV4W55R']['makespan']['flown_s'], 1)),
    ('T3 W55S makespan', '966.2→996.9', f(T['HV4W55S']['makespan']['planned_s'], 1) + '→' + f(T['HV4W55S']['makespan']['flown_s'], 1)),
    ('T3 SITL16 makespan', '1260.4→1255.9', f(T['HV4SITL16']['makespan']['planned_s'], 1) + '→' + f(T['HV4SITL16']['makespan']['flown_s'], 1)),
    ('T3 sep SITL16', '| 100.685 |', f(T['HV4SITL16']['separation']['flown_exact_m'], 3)),
    ('T3 sep W00', '| 100.435 |', f(T['HV4W00']['separation']['flown_exact_m'], 3)),
    ('T3 sep W05', '| 100.291 |', f(T['HV4W05']['separation']['flown_exact_m'], 3)),
    ('T3 sep W55R', '| 100.579 |', f(T['HV4W55R']['separation']['flown_exact_m'], 3)),
    ('T3 sep W55S', '| 266.177 |', f(T['HV4W55S']['separation']['flown_exact_m'], 3)),
    ('T3 E-W SITL16', '−0.019 至 +0.005', f(min(sig('HV4SITL16')), 4) + '..' + f(max(sig('HV4SITL16')), 4)),
    ('T3 E-W W00', '−0.009 至 +0.009', f(min(sig('HV4W00')), 4) + '..' + f(max(sig('HV4W00')), 4)),
    ('T3 E-W W05', '+9.915 至 +9.969', f(min(sig('HV4W05')), 4) + '..' + f(max(sig('HV4W05')), 4)),
    ('T3 E-W W55', '+9.931 至 +9.958', f(min(sig('HV4W55')), 4) + '..' + f(max(sig('HV4W55')), 4)),
    ('T3 E-W W55R', '+9.934 至 +9.960', f(min(sig('HV4W55R')), 4) + '..' + f(max(sig('HV4W55R')), 4)),
    ('T3 N-S W55S (30 m floor)', '+9.512 至 +9.853',
     f(min(sig('HV4W55S', 'signal_30m')), 4) + '..' + f(max(sig('HV4W55S', 'signal_30m')), 4)),
    ('note (a) W55S E-W', '−0.046 至 +0.014', f(min(null55s), 4) + '..' + f(max(null55s), 4)),
    ('note (b) spawn', '5.0–5.1 m',
     '/'.join(f(v['err_m'], 3) for v in spawn['per_uav'].values()) + ' ; spacing delta '
     + f(spawn['max_abs_spacing_delta_m'], 4)),
    ('note (d) SITL15 vs 16', '0.025 s',
     f(T['HV4SITL16']['makespan']['flown_s'] - T['HV4SITL15']['makespan']['flown_s'], 3)),
    ('note (e) plan-flown', '4.20–5.27 m',
     ', '.join(k + ' ' + f(T[k]['separation']['plan_minus_flown_m'], 2)
               for k in ('HV4W00', 'HV4W05', 'HV4W55', 'HV4W55R', 'HV4SITL16', 'HV4SITL15', 'HV4W55S'))),
    ('limit (2) margins', '間距餘裕僅 0.426 m（八趟最緊 0.291 m）',
     f(w55['separation']['margin_to_100_m'], 3) + ' / min ' + f(min(T[k]['separation']['margin_to_100_m'] for k in T), 3)),
    ('limit (7) export dev', '+0.014 至 +0.170', 'trial max range ' + f(min(exp_dev), 4) + '..' + f(max(exp_dev), 4)),
    ('limit (7) tracking', '+0.027 至 +0.148', f(min(trk), 4) + '..' + f(max(trk), 4)),
    ('limit (7) worst margin', '最壞餘裕 0.303 m',
     f(min(T[k]['altitude']['margin_to_120_m'] for k in T if T[k]['ceiling_m'] == 119.5), 4)),
    ('BEAMIDP1 sep', '180.77 m', f(T['BEAMIDP1']['separation']['flown_exact_m'], 2)),
]
ok = 0
for lab, s, v in checks:
    hit = s in brief
    ok += hit
    print(f"{'OK ' if hit else '?? '} {lab:26s} brief: {s:34s} JSON: {v}")
print(f'{ok}/{len(checks)} brief strings located verbatim')
