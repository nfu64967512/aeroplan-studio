"""把一趟 HV-DCCPP SITL 航次整理成圖 + 機器可讀摘要（情境驅動版）。

    python scripts/hv_sitl_report.py gui_uniform_1a3b

作業區幾何直接讀該趟自己寫下的 `scenario.json`，不再寫死在腳本裡。

⚠ 本專案的航次資料是**覆寫式**的：下一趟會蓋掉 flight.csv。每趟結束立刻跑這支。
"""
from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.hv_sitl_run import out_dir  # noqa: E402
from scripts.hv_sitl_scenarios import SCENARIOS  # noqa: E402

CLS_COLOR = {"A": "#ff6b35", "B": "#1c7ed6"}
CLS_NAME = {"A": "Pioneer", "B": "HERO"}


def _track(out: Path, uav: str):
    rows = [r for r in csv.DictReader((out / uav / "flight.csv").open(encoding="utf-8"))
            if r["lat"] and r["t_s"]]
    armed = [(r["armed"].strip().lower() in ("true", "1")) if r.get("armed") else None
             for r in rows]
    return ([float(r["lon"]) for r in rows], [float(r["lat"]) for r in rows],
            [float(r["t_s"]) for r in rows], [float(r["rel_alt_m"]) for r in rows],
            armed)


def _touchdown_s(t, alt, armed) -> float:
    """實際落地的模擬時刻。

    ⚠ **不可以**用 `max(t_s)`：監看迴圈會一直錄到最後一架落地為止，先落地的
    那幾架的最後一列是「停在地上」的時間，不是落地時間。判準與飛行腳本相同：
    曾經爬過 20 m，之後高度 < 2 m 且已上鎖 —— 取第一次成立的那一刻。
    """
    peak = 0.0
    for ts, a, ar in zip(t, alt, armed):
        peak = max(peak, a)
        if peak > 20.0 and a < 2.0 and ar is False:
            return ts
    return float("nan")


def _clock_offset(out: Path, uavs) -> float:
    """`flight.csv` 的 `t_s` 零點與規劃時刻表零點之間的偏移（模擬秒）。

    ⚠⚠ **不校正就會把偏移讀成「每一架都晚 40-60 秒」**。實測：每機第一列都在
    `t_s≈43-46`，而且那時還有飛機停在地面 MANUAL ⇒ `t_s` 的零點不是放行時刻，
    是 `VehicleLink(time_source="boot")` 的時基加上「連線→監看迴圈」之間的模擬時間。

    量法：切 AUTO 的牆鐘時刻是我們自己記的（`release_schedule.json`），
    而 `sim_t(切AUTO) = offset + wall × speedup`，所以
    `offset = t_s(第一次看到 AUTO) − wall × speedup`，逐機算完取中位數。
    八個架次量出來是 +43 ~ +46 s，兩趟的中位數都是 45.0（取樣間隔 ~5.5 s）。
    """
    f = out / "release_schedule.json"
    if not f.exists():
        return 0.0
    rel = json.loads(f.read_text(encoding="utf-8"))
    sp = float(rel.get("speedup") or 1.0)
    act = rel.get("actual_release_wall_s") or {}
    offs = []
    for uav in uavs:
        if uav not in act:
            continue
        rows = [r for r in csv.DictReader((out / uav / "flight.csv").open(encoding="utf-8"))
                if r["t_s"]]
        auto = [r for r in rows if r["mode"] == "AUTO"]
        if auto:
            offs.append(float(auto[0]["t_s"]) - float(act[uav]) * sp)
    if not offs:
        return 0.0
    offs.sort()
    return offs[len(offs) // 2]


def _planned(out: Path, uav: str):
    lon, lat = [], []
    for ln in (out / "plan" / f"{uav}.waypoints").read_text(encoding="utf-8").splitlines()[1:]:
        if ln.startswith("#"):
            continue
        f = ln.split("\t")
        if len(f) >= 11 and float(f[8]) != 0.0:
            lat.append(float(f[8]))
            lon.append(float(f[9]))
    return lon, lat


def _flown_km(lon, lat):
    import math
    d = 0.0
    for i in range(1, len(lat)):
        dy = (lat[i] - lat[i - 1]) * 111132.0
        dx = (lon[i] - lon[i - 1]) * 111412.0 * math.cos(math.radians(lat[i]))
        d += math.hypot(dx, dy)
    return d / 1000.0


def _coverage_from_tracks(meta: dict, tracks: dict, swath_m: float) -> list[dict]:
    """從**實飛航跡**回算每塊作業區的覆蓋率。

    做法：把所有航跡在局部平面上緩衝 `swath/2`，取聯集後與每塊區求交集面積比。

    ⚠ 這是一個**上界**：`flight.csv` 沒有記 `sensor_on`，所以進出場與轉場段
      也被算成有拍到。落在區外的轉場不影響（交集會切掉），但穿越作業區的
      轉場會被高估。所以「≈1.0」只能說「航跡有掃過」，不能說「合格影像」。
    ⚠ 也是一個**下界**的反面：緩衝用的是標稱刈幅，沒有考慮姿態與地形起伏。
    """
    import math

    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union

    lat0 = meta["areas"][0]["polygon"][0][0]
    m_lat, m_lon = 111132.0, 111412.0 * math.cos(math.radians(lat0))

    def xy(lat, lon):
        return ((lon - 120.90) * m_lon, (lat - 23.95) * m_lat)

    lines = []
    for lon, lat in tracks.values():
        pts = [xy(a, o) for a, o in zip(lat, lon)]
        pts = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
        if len(pts) >= 2:
            lines.append(LineString(pts).buffer(swath_m / 2.0, quad_segs=8))
    swept = unary_union(lines) if lines else None

    out = []
    for i, area in enumerate(meta["areas"]):
        poly = Polygon([xy(a, o) for a, o in area["polygon"]])
        if not poly.is_valid:
            poly = poly.buffer(0)
        inter = poly.intersection(swept).area if swept is not None else 0.0
        out.append({"area": f"Z{i + 1}", "area_m2": round(poly.area, 1),
                    "covered_m2": round(inter, 1),
                    "ratio": round(inter / poly.area, 4) if poly.area else None})
    return out


def _min_separation(series: dict, step_s: float = 2.0) -> dict:
    """實飛的最小間距：只在**兩架都在空中**的時刻算。

    `series[uav] = (t, lat, lon, alt)`。取樣間隔約 5.5 s，所以先線性內插到
    `step_s` 的共同時間軸再逐對算水平距離。

    ⚠ 這是**實飛**量測，跟 `validation.json` 的 `min_sep_h_m` 不是同一個東西
      （那個是規劃值）。兩者差很多才值得追。
    ⚠ 內插會低估瞬時最小值（兩點之間可能更近），所以這是一個**上界**。
    """
    import math

    def interp(t, xs, q):
        i = 0
        out = []
        for u in q:
            while i + 2 < len(t) and t[i + 1] < u:
                i += 1
            if u < t[0] or u > t[-1]:
                out.append(None)
                continue
            span = t[i + 1] - t[i]
            f = 0.0 if span <= 0 else (u - t[i]) / span
            out.append(xs[i] + f * (xs[i + 1] - xs[i]))
        return out

    uavs = sorted(series)
    lo = min(min(s[0]) for s in series.values())
    hi = max(max(s[0]) for s in series.values())
    grid = [lo + i * step_s for i in range(int((hi - lo) / step_s) + 1)]
    g = {}
    for u in uavs:
        t, lat, lon, alt = series[u]
        g[u] = (interp(t, lat, grid), interp(t, lon, grid), interp(t, alt, grid))

    best, worst_pair, worst_t, pairs = float("inf"), None, None, {}
    for i, a in enumerate(uavs):
        for b in uavs[i + 1:]:
            pmin = float("inf")
            for k, tk in enumerate(grid):
                la, oa, aa = g[a][0][k], g[a][1][k], g[a][2][k]
                lb, ob, ab = g[b][0][k], g[b][1][k], g[b][2][k]
                if None in (la, oa, aa, lb, ob, ab) or aa <= 2.0 or ab <= 2.0:
                    continue
                dy = (la - lb) * 111132.0
                dx = (oa - ob) * 111412.0 * math.cos(math.radians(la))
                d = math.hypot(dx, dy)
                if d < pmin:
                    pmin = d
                if d < best:
                    best, worst_pair, worst_t = d, (a, b), tk
            pairs[f"{a}-{b}"] = (round(pmin, 1) if pmin < float("inf") else None)
    return {"min_h_m": round(best, 1) if best < float("inf") else None,
            "pair": worst_pair, "t_s": worst_t, "per_pair_min_h_m": pairs,
            "step_s": step_s,
            "caveat": "只在兩架都 >2 m AGL 的時刻計算；線性內插 ⇒ 這是上界"}


def main() -> int:
    key = sys.argv[1] if len(sys.argv) > 1 else ""
    if key not in SCENARIOS:
        print(f"用法：python scripts/hv_sitl_report.py <情境>。可用：{list(SCENARIOS)}")
        return 1
    sc = SCENARIOS[key]
    out = out_dir(sc)
    meta = json.loads((out / "scenario.json").read_text(encoding="utf-8"))
    mission = json.loads((out / "plan" / "mission_summary.json").read_text(encoding="utf-8"))
    uavs = sorted(p for p in mission["uavs"])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15.5, 6.5))
    for i, area in enumerate(meta["areas"]):
        poly = area["polygon"]
        xs = [p[1] for p in poly] + [poly[0][1]]
        ys = [p[0] for p in poly] + [poly[0][0]]
        ax1.plot(xs, ys, "k--", lw=0.9, alpha=0.55)
        ax1.text(sum(xs[:-1]) / len(poly), max(ys) + 0.00035, f"Z{i + 1}",
                 fontsize=8, ha="center")

    offset = _clock_offset(out, uavs)
    print(f"telemetry 時基偏移 {offset:.1f} s（已從所有實飛時刻扣掉）")
    rows, tracks, series = [], {}, {}
    for uav in uavs:
        cls = uav[0]
        c = CLS_COLOR.get(cls, "#888")
        plon, plat = _planned(out, uav)
        ax1.plot(plon, plat, color=c, lw=0.8, alpha=0.35, ls=":")
        lon, lat, t, alt, armed = _track(out, uav)
        t = [x - offset for x in t]          # 扣掉時基偏移，才能跟規劃時刻比
        tracks[uav] = (lon, lat)
        series[uav] = (t, lat, lon, alt)
        km = _flown_km(lon, lat)
        td = _touchdown_s(t, alt, armed)
        u = mission["uavs"][uav]
        ax1.plot(lon, lat, color=c, lw=1.5,
                 label=f"{uav} {CLS_NAME.get(cls, cls)} {km:.2f} km")
        ax2.plot(t, alt, color=c, lw=1.5, label=f"{uav} {CLS_NAME.get(cls, cls)}")
        ax2.axvline(u["t_land_s"], color=c, ls=":", lw=0.8, alpha=0.5)
        air = [x for x, a in zip(t, alt) if a > 2.0]
        rows.append({"uav": uav, "class": CLS_NAME.get(cls, cls),
                     "clock_offset_s": round(offset, 1),
                     "flown_t_takeoff_s": round(air[0], 1) if air else None,
                     "plan_t_takeoff_s": round(u["t_takeoff_s"], 1),
                     "plan_t_land_s": round(u["t_land_s"], 1),
                     "flown_t_land_s": round(td, 1) if td == td else None,
                     "plan_km": round(u["distance_m"] / 1000.0, 2),
                     "flown_km": round(km, 2),
                     "peak_alt_m": round(max(alt) if alt else 0.0, 1),
                     "err_pct": round(100.0 * (km - u["distance_m"] / 1000.0)
                                      / max(u["distance_m"] / 1000.0, 1e-6), 1)})

    ax1.set_title(f"{meta['key']}（{out.name}）: planned (dotted) vs flown (solid)")
    ax1.set_xlabel("lon")
    ax1.set_ylabel("lat")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)
    ax1.set_aspect("equal", "datalim")
    ax2.set_title("altitude above home vs simulation time\n"
                  "(dotted vertical = planned landing time)")
    ax2.set_xlabel("sim time [s]  (GLOBAL_POSITION_INT.time_boot_ms)")
    ax2.set_ylabel("rel alt [m]")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "compare.png", dpi=130)

    (out / "flight_summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    tot_p = sum(r["plan_km"] for r in rows)
    tot_f = sum(r["flown_km"] for r in rows)
    print(f"{out.name}  {len(rows)} 架  規劃 {tot_p:.2f} km  實飛 {tot_f:.2f} km  "
          f"({100 * (tot_f - tot_p) / tot_p:+.1f}%)")
    for r in rows:
        td = r["flown_t_land_s"]
        td_s = f"{td:7.1f}" if td is not None else "  沒落地"
        dt_s = f"{td - r['plan_t_land_s']:+6.1f}" if td is not None else "     ?"
        print(f"  {r['uav']:3s} {r['class']:8s} 落地 規劃 {r['plan_t_land_s']:7.1f}"
              f" / 實飛 {td_s} s ({dt_s})   "
              f"航程 {r['plan_km']:6.2f} / {r['flown_km']:6.2f} km ({r['err_pct']:+.1f}%)"
              f"   最高 {r['peak_alt_m']:6.1f} m")

    # 實飛最小間距（validation.json 那個是規劃值，不是這個）
    try:
        sep = _min_separation(series)
        (out / "separation_from_tracks.json").write_text(
            json.dumps(sep, ensure_ascii=False, indent=1), encoding="utf-8")
        plan_sep = json.loads((out / "validation.json").read_text(encoding="utf-8")
                              ).get("min_sep_h_m")
        print(f"\n實飛最小水平間距 {sep['min_h_m']} m"
              f"（{sep['pair']} 於 t={sep['t_s']:.0f}s；規劃值 {plan_sep:.1f} m）")
        print("  逐對:", sep["per_pair_min_h_m"])
    except Exception as e:      # noqa: BLE001
        print("間距回算失敗:", e)

    # 覆蓋率：從實飛航跡回算（規劃器說 uncovered=0，這裡獨立驗一次）
    import math as _m
    swath = 2.0 * meta["altitude_m"] * _m.tan(_m.radians(meta["hfov_deg"]) / 2.0)
    try:
        cov = _coverage_from_tracks(meta, tracks, swath)
        (out / "coverage_from_tracks.json").write_text(json.dumps(
            {"swath_m": round(swath, 1), "areas": cov,
             "caveat": "航跡緩衝法；flight.csv 沒有 sensor_on，轉場段也被算進去 ⇒ 這是上界"},
            ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"\n實飛航跡覆蓋率（刈幅 {swath:.1f} m 緩衝；上界）:")
        for c in cov:
            print(f"  {c['area']}  {c['ratio'] * 100:6.2f}%  "
                  f"({c['covered_m2'] / 1e4:.2f} / {c['area_m2'] / 1e4:.2f} 公頃)")
    except Exception as e:      # noqa: BLE001
        print("覆蓋率回算失敗:", e)

    try:
        (out / "git_state.txt").write_text(
            subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                           capture_output=True, text=True).stdout
            + subprocess.run(["git", "status", "--porcelain"], cwd=ROOT,
                             capture_output=True, text=True).stdout, encoding="utf-8")
    except Exception as e:      # noqa: BLE001
        print("git 狀態寫入失敗:", e)
    for name in ("hv_sitl_run.py", "hv_sitl_scenarios.py", "hv_sitl_scout.py",
                 "hv_sitl_report.py"):
        (out / name).write_bytes((ROOT / "scripts" / name).read_bytes())
    print("寫出", out / "compare.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
