"""在任一個工作樹裡跑同一組舊 DCCPP 組態，印出可比對的摘要。"""
import hashlib, json, math, sys, logging
sys.path.insert(0, '.')
logging.disable(logging.CRITICAL)
from mission.swarm_coordinator import SwarmCoordinator
HOME = (23.95, 120.90); M_LAT = 111132.0
M_LON = 111412.0 * math.cos(math.radians(23.95))
ll = lambda e, n: (HOME[0] + n / M_LAT, HOME[1] + e / M_LON)
def rect(cx, cy, he, hn):
    return [ll(cx - he, cy - hn), ll(cx + he, cy - hn), ll(cx + he, cy + hn), ll(cx - he, cy + hn)]
def run(n, he, hn, width):
    D = [{'drone_id': i + 1, 'name': f'X{i+1}', 'position': HOME,
          'speed': (21.5 if i < n // 2 else 19.0), 'heading': 0.0,
          'turn_radius': (200.0 if i < n // 2 else 120.0),
          'vehicle_type': 'vtol', 'fov_width': width} for i in range(n)]
    A = [{'area_id': 1, 'priority': 1.0, 'polygon': rect(0, 0, he, hn)}]
    res = SwarmCoordinator().plan_coverage_dccpp(
        areas=A, drones=D, coverage_width_m=width, overlap_rate=0.30,
        coverage_altitude=140.0, mounting_angle_deg=0.0)
    ap = res['assembled_paths']
    ys = [(w.lat - HOME[0]) * M_LAT for p in ap.values() for w in p.waypoints
          if str(w.segment_type).endswith('OPERATION')]
    km = sum(p.total_length_m for p in ap.values()) / 1000.0
    dig = hashlib.md5(json.dumps(
        [[round(w.lat, 7), round(w.lon, 7)] for p in ap.values() for w in p.waypoints],
        sort_keys=True).encode()).hexdigest()[:12]
    return dict(n=n, area=f'{2*he}x{2*hn}', width=width,
                assigned=len([p for p in ap.values() if p.total_length_m > 0]),
                km=round(km, 2), ms=round(res['total_makespan'], 1),
                north_gap=round(hn - max(ys)) if ys else None,
                south_gap=round(min(ys) + hn) if ys else None, digest=dig)
if __name__ == '__main__':
    for cfg in ((8, 2500, 2000, 150.0), (4, 2500, 2000, 150.0), (2, 2500, 2000, 150.0),
                (8, 1500, 1500, 150.0), (8, 2500, 2000, 100.0), (8, 2500, 2000, 250.0),
                (1, 2500, 2000, 150.0), (8, 4000, 500, 150.0)):
        print(json.dumps(run(*cfg), ensure_ascii=False))
