"""把 HARVEST 四機 SITL 航次整理成圖與說明（runs/artifacts/<TRIAL>/）。

⚠ 本專案的航次資料是**覆寫式**的：下一趟會蓋掉 flight.csv。所以每趟結束要立刻跑這支。
"""
from __future__ import annotations

import csv
import json
import os
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
TRIAL = os.environ.get("HV_TRIAL", "HV4SITL2")
OUT = ROOT / "runs" / "artifacts" / TRIAL

# ⚠ 舊版按**構型**上色 ⇒ 三台 HERO 共用同一個藍色，圖上分不出是哪一台。
#   改成逐機一色、同構型留在同一個色系（暖＝Pioneer、冷＝HERO）。
#   與 scripts/hv_sitl_report.py 的 CLS_PALETTE 同一組值，兩張圖顏色一致。
CLS_PALETTE = {
    "Pioneer": ["#ff6b35", "#e8590c", "#ffa94d", "#c92a2a"],
    "HERO": ["#1c7ed6", "#0ca678", "#7048e8", "#0b7285"],
}


def uav_colors(rows) -> dict:
    """{機號: 顏色}；同構型依機號排序後在自己的色系裡依序取色。"""
    used, out = {}, {}
    for r in sorted(rows, key=lambda x: str(x["uav"])):
        cls = r["class"]
        pal = CLS_PALETTE.get(cls) or ["#888888"]
        i = used.get(cls, 0)
        used[cls] = i + 1
        out[r["uav"]] = pal[i % len(pal)]
    return out
AREAS = {
    "Z1 LiDAR (Pioneer)": [(23.944131, 120.906876), (23.944131, 120.910806),
                           (23.947742, 120.910806), (23.947743, 120.906877)],
    "Z2 RGB (HERO x3)": [(23.952256, 120.885264), (23.952257, 120.893123),
                         (23.957313, 120.893123), (23.957312, 120.885263)],
}


def _track(uav: str):
    rows = [r for r in csv.DictReader((OUT / uav / "flight.csv").open(encoding="utf-8"))
            if r["lat"] and r["t_s"]]
    return ([float(r["lon"]) for r in rows], [float(r["lat"]) for r in rows],
            [float(r["t_s"]) for r in rows], [float(r["rel_alt_m"]) for r in rows])


def _planned(uav: str):
    lon, lat = [], []
    for ln in (OUT / "plan" / f"{uav}.waypoints").read_text(encoding="utf-8").splitlines()[1:]:
        if ln.startswith("#"):
            continue
        f = ln.split("\t")
        if len(f) >= 11 and float(f[8]) != 0.0:
            lat.append(float(f[8]))
            lon.append(float(f[9]))
    return lon, lat


def main() -> int:
    summary = json.loads((OUT / "flight_summary.json").read_text(encoding="utf-8"))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6.5))

    for name, poly in AREAS.items():
        xs = [p[1] for p in poly] + [poly[0][1]]
        ys = [p[0] for p in poly] + [poly[0][0]]
        ax1.plot(xs, ys, "k--", lw=1.0, alpha=0.6)
        ax1.text(sum(xs[:-1]) / 4, max(ys) + 0.0004, name, fontsize=8, ha="center")

    colors = uav_colors(summary)
    for row in summary:
        uav, cls = row["uav"], row["class"]
        c = colors[uav]
        plon, plat = _planned(uav)
        ax1.plot(plon, plat, color=c, lw=0.8, alpha=0.35, ls=":")
        lon, lat, t, alt = _track(uav)
        ax1.plot(lon, lat, color=c, lw=1.6,
                 label=f"{uav} {cls} {row['flown_km']:.2f} km")
        ax2.plot(t, alt, color=c, lw=1.6, label=f"{uav} {cls}")
        ax2.axvline(row["plan_t_land_s"], color=c, ls=":", lw=0.8, alpha=0.5)

    ax1.set_title(f"{TRIAL}: planned (dotted) vs flown (solid)")
    ax1.set_xlabel("lon"); ax1.set_ylabel("lat")
    ax1.legend(fontsize=8); ax1.grid(alpha=0.3); ax1.set_aspect("equal", "datalim")

    ax2.set_title("altitude above home vs simulation time\n"
                  "(dotted vertical = planned landing time)")
    ax2.set_xlabel("sim time [s]  (GLOBAL_POSITION_INT.time_boot_ms)")
    ax2.set_ylabel("rel alt [m]")
    ax2.legend(fontsize=8); ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(OUT / "compare.png", dpi=130)
    print("寫出", OUT / "compare.png")

    try:
        (OUT / "git_state.txt").write_text(
            subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                           capture_output=True, text=True).stdout
            + subprocess.run(["git", "status", "--porcelain"], cwd=ROOT,
                             capture_output=True, text=True).stdout,
            encoding="utf-8")
    except Exception as e:      # noqa: BLE001
        print("git 狀態寫入失敗:", e)
    for name in ("hv_sitl_flight.py", "hv_sitl_fly.py", "hv_sitl_artifacts.py"):
        (OUT / name).write_bytes((ROOT / "scripts" / name).read_bytes())
    print("已複製驅動腳本進 artifacts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
