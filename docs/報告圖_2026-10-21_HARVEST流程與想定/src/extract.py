"""Extract HV4W55 plan + SITL data into ENU JSON for the report figures."""
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(sys.argv[1])
OUT = Path(sys.argv[2])
RUN = ROOT / "runs" / "artifacts" / "HV4W55"
H_LAT, H_LON = 23.95, 120.90
M_LAT = 111132.0
M_LON = 111412.0 * math.cos(math.radians(H_LAT))


def enu(lat, lon):
    return ((lon - H_LON) * M_LON, (lat - H_LAT) * M_LAT)


data = {"uavs": {}}
for u in ("A1", "B2", "B3"):
    lines = (RUN / "plan" / f"{u}.waypoints").read_text(encoding="utf-8").splitlines()
    head = lines[1]
    items = []
    for ln in lines[2:]:
        if ln.startswith("#") or not ln.strip():
            continue
        f = ln.split("\t")
        items.append(dict(seq=int(f[0]), cmd=int(f[3]), p1=float(f[4]),
                          lat=float(f[8]), lon=float(f[9]), alt=float(f[10])))
    path = []          # all NAV points in order with tag
    imaging = []       # list of polylines
    cam_on = False
    cur = None
    last = None
    land_start_idx = None
    for it in items:
        if it["cmd"] in (16, 84, 85) or (it["cmd"] == 0 and it["seq"] == 0):
            x, y = enu(it["lat"], it["lon"])
            pt = [round(x, 2), round(y, 2), round(it["alt"], 2), it["cmd"]]
            path.append(pt)
            last = pt
            if cam_on and cur is not None:
                cur.append(pt)
        elif it["cmd"] == 206:
            if it["p1"] > 0:
                cam_on = True
                cur = [last]
            else:
                cam_on = False
                if cur and len(cur) > 1:
                    imaging.append(cur)
                cur = None
        elif it["cmd"] == 189:
            land_start_idx = len(path) - 1
    data["uavs"][u] = dict(head=head, path=path, imaging=imaging, land_start_idx=land_start_idx)

    rows = list(csv.DictReader(open(RUN / u / "flight.csv", encoding="utf-8")))
    trk = []
    for r in rows:
        x, y = enu(float(r["lat"]), float(r["lon"]))
        trk.append([round(float(r["t_s"]) - 74.04, 2), round(x, 1), round(y, 1), round(float(r["rel_alt_m"]), 2), r["mode"]])
    data["uavs"][u]["track"] = trk

sc = json.loads((RUN / "scenario.json").read_text(encoding="utf-8"))
area = [enu(la, lo) for la, lo in sc["areas"][0]["polygon"]]
data["area"] = [[round(x, 2), round(y, 2)] for x, y in area]
data["subzones_lat"] = [23.9464007, 23.9488002, 23.9511998, 23.9535993]
data["subzones_y"] = [round((la - H_LAT) * M_LAT, 2) for la in data["subzones_lat"]]
OUT.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")

for u, d in data["uavs"].items():
    xs = [p[0] for p in d["path"]]
    ys = [p[1] for p in d["path"]]
    print(u, "n", len(d["path"]), "img", len(d["imaging"]), [ (round(s[0][1]),round(s[0][0]),round(s[-1][0])) for s in d["imaging"]],
          "x", round(min(xs)), round(max(xs)), "y", round(min(ys)), round(max(ys)), "landstart", d["land_start_idx"])
print("area", data["area"], "sub_y", data["subzones_y"])
