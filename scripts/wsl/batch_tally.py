#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""batch_tally.py — 一批航次的逐機可靠度總表。

⚠️ 使用者 2026-08-18 定調的第一順位是**逐機真觸地率 ≥90%**,散布次之。
⚠️ 觸地一律用 `stot_score.py` 的觸地基準(最低 AGL / 觸地誤差),
   **不用**節點的 `expended`(只看水平距)也不用鎖存當下的水平距
   —— 兩者可差兩個數量級(RRC1:鎖存 21.3 m,真觸地 0.18 m)。

用法:  python3 batch_tally.py RRD
"""
import collections
import csv
import math
import pathlib
import re
import statistics as st
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
ART = ROOT / "runs" / "artifacts"
SCORE = ROOT / "scripts" / "wsl" / "stot_score.py"
R_EARTH = 6378137.0
#: ⚠️⚠️⚠️ **這一欄不是「最低 AGL」。**
#: `stot_score.py` 那一欄印的是「**最近水平通過點當下的殘餘高度**」,
#: 不是整段的最低高度。實測(RRD 16 架次)真正的最低 AGL 是 −8.3…−0.5 m,
#: **16/16 全部都落水了**;而這一欄卻是 −3.6…+42.8。
#: ⇒ 用它當「有沒有落地」的判準會憑空造出一個「高空掠過(未落地)」的族群,
#:   而那個族群**不存在** —— 它們的水平最近距其實是全批最好的(0.04–0.07 m 內插),
#:   只是通過紅心時還高 26–43 m,然後落在下游 37–46 m。
#: ⇒ 這一欄的正確名稱是 **殘餘高度**,它是誤差的成因(err ≈ 1.139×殘餘 + 2.54,r=0.97),
#:   不是「有沒有觸地」。判「有沒有命中」只看**觸地誤差**。
RESIDUAL_ALT_M = 5.0
#: ⚠️⚠️ **落地 ≠ 命中**。RRD3/U2 最低 AGL −3.6 m(真的落地)但觸地誤差 **647.88 m**
#:    —— 它以 24.5 m 從紅心上空掠過、然後飛過頭 648 m 才落地。
#:    節點只報「最近水平距 24.5 m」,看起來像完美命中。
#: ⇒ 可靠度要**兩個**判準都過:落地 **且** 落點在命中半徑內。
HIT_ERR_M = 10.0


def score(trial):
    """跑 stot_score.py,抓出逐機的 (最低AGL, 觸地誤差, 觸地時刻)。"""
    csvf = ART / trial / f"tot_{trial}.csv"
    if not csvf.exists():
        return None
    r = subprocess.run([sys.executable, str(SCORE), str(csvf)],
                       capture_output=True, text=True, encoding="utf-8",
                       errors="replace")
    out = {}
    for line in (r.stdout or "").splitlines():
        # `UAV  gs峰值dive  gs觸地  gs最近  最低AGL  最近  |  觸地誤差  觸地t`
        m = re.match(r"^\s*(\d+)\s+([\d.\-]+)\s+([\d.\-]+)\s+([\d.\-]+)\s+"
                     r"([\d.\-]+)\s+([\d.\-]+)\s*\|\s*([\d.\-]+)\s+([\d.\-]+)\s*$", line)
        if m:
            out[m.group(1)] = dict(min_agl=float(m.group(5)),
                                   err=float(m.group(7)), t=float(m.group(8)))
    return out


def release_info(trial):
    """從 CSV 取放行當拍的 σ / 高度 / 距離,以及放行時刻。"""
    f = ART / trial / f"tot_{trial}.csv"
    if not f.exists():
        return {}
    T = (25.1261649, 120.6156480)
    seq = collections.defaultdict(list)
    for r in csv.DictReader(open(f, encoding="utf-8", errors="replace")):
        try:
            seq[r["sysid"]].append(r)
        except Exception:
            pass
    out = {}
    for s, Q in seq.items():
        r0 = next((x for x in Q if x["mode"] == "dive"), None)
        if r0 is None:
            continue
        la, lo = float(r0["lat"]), float(r0["lon"])
        d = math.hypot(math.radians(T[0] - la) * R_EARTH,
                       math.radians(T[1] - lo) * R_EARTH
                       * math.cos(math.radians((la + T[0]) / 2)))
        try:
            sg = abs(float(r0["lead_angle_deg"]))
        except Exception:
            sg = float("nan")
        out[s] = dict(t=float(r0["t"]), sig=sg, alt=float(r0["agl"]), d=d)
    return out


def main(prefix):
    trials = sorted((p.name for p in ART.glob(f"{prefix}*") if p.is_dir()),
                    key=lambda x: (len(x), x))
    print(f"{'趟/機':10}{'放行σ°':>8}{'放行高度':>9}{'γ_req°':>8}"
          f"{'最低AGL':>9}{'觸地誤差':>10}{'放行差':>8}{'觸地差':>8}   判定")
    rec = []
    for tr in trials:
        sc, rel = score(tr), release_info(tr)
        if not sc:
            print(f"{tr:10}  ⚠️ 沒有 CSV / 無法評分 —— 這一趟不算進統計")
            continue
        dr = (max(v["t"] for v in rel.values()) - min(v["t"] for v in rel.values())
              if len(rel) >= 2 else float("nan"))
        dt = (max(v["t"] for v in sc.values()) - min(v["t"] for v in sc.values())
              if len(sc) >= 2 else float("nan"))
        for s in sorted(sc):
            a, r = sc[s], rel.get(s, {})
            g = (math.degrees(math.atan2(r.get("alt", 0.0), max(r.get("d", 1.0), 1.0)))
                 if r else float("nan"))
            low_residual = a["min_agl"] <= RESIDUAL_ALT_M
            ok = a["err"] <= HIT_ERR_M
            rec.append((tr, s, r.get("sig", float("nan")), r.get("alt", float("nan")),
                        g, a["min_agl"], a["err"], ok, dr, dt))
            print(f"{tr}/U{s:<6}{r.get('sig', float('nan')):8.1f}"
                  f"{r.get('alt', float('nan')):9.0f}{g:8.1f}"
                  f"{a['min_agl']:9.1f}{a['err']:10.2f}"
                  f"{dr:8.1f}{dt:8.1f}   "
                  f"{'✅ 命中' if ok else ('⚠️ 殘餘高度' if not low_residual else '⚠️ 落點偏差')}")
    if not rec:
        return
    n = len(rec)
    k = sum(1 for x in rec if x[7])
    print("\n" + "═" * 82)
    print(f"【逐機命中率】**{k}/{n} = {100 * k / n:.0f}%**"
          f"(判準:**觸地誤差 ≤ {HIT_ERR_M:.0f} m**;使用者要求 ≥90%)")
    # ⚠️ Clopper-Pearson 單側 95% 下界 —— 點估計不能拿去對「≥90%」這條線。
    #    16/16 的下界只有 0.829;要以 95% 信心宣告 p≥0.90,最少需要 29 次連續零失敗。
    try:
        from scipy.stats import beta as _b
        lo = 0.0 if k == 0 else _b.ppf(0.05, k, n - k + 1)
    except Exception:
        lo = float("nan")
    print(f"     ⚠️ Clopper-Pearson 單側 95% 下界 = {lo:.3f} —— "
          f"點估計 {100 * k / n:.0f}% **不等於**母體率 ≥90%")
    n_res = sum(1 for x in rec if x[5] > RESIDUAL_ALT_M)
    print(f"     殘餘高度 >{RESIDUAL_ALT_M:.0f} m 的 {n_res} 架("
          f"⚠️ 這**不是**「沒落地」—— RRD 16/16 最低 AGL 都 <0,全部落水)")
    hp = [x for x in rec if x[5] > RESIDUAL_ALT_M]
    if hp:
        print(f"     殘餘高度偏高 {len(hp)} 架:殘餘 "
              f"{[round(x[5]) for x in hp]} m、對應誤差 {[round(x[6]) for x in hp]} m")
        print(f"     ⇒ 對照 [[miss-is-residual-altitude]] err = 1.167×殘餘 − 2.62:"
              f"預測 {[round(1.167 * x[5] - 2.62) for x in hp]} m")
    sg = [x[2] for x in rec if x[2] == x[2]]
    al = [x[3] for x in rec if x[3] == x[3]]
    print(f"【放行工作點】σ 中位 {st.median(sg):.1f}°(全距 {min(sg):.1f}–{max(sg):.1f});"
          f" 放行高度中位 {st.median(al):.0f} m(全距 {min(al):.0f}–{max(al):.0f})")
    if len(hp) >= 2 and len(rec) - len(hp) >= 2:
        a_hp = st.median([x[3] for x in hp])
        a_ok = st.median([x[3] for x in rec if x[7]])
        s_hp = st.median([x[2] for x in hp])
        s_ok = st.median([x[2] for x in rec if x[7]])
        print(f"     掠過組 vs 觸地組:放行高度 {a_hp:.0f} vs {a_ok:.0f} m;"
              f" σ {s_hp:.1f}° vs {s_ok:.1f}°")
    dr = [x[8] for x in rec if x[8] == x[8]]
    dt = [x[9] for x in rec if x[9] == x[9]]
    if dr and dt:
        print(f"【散布分解】放行時刻差中位 {st.median(dr):.1f} s、"
              f"觸地時刻差中位 {st.median(dt):.1f} s(門檻 3 s)")
    err = [x[6] for x in rec]
    print(f"【觸地誤差】中位 {st.median(err):.2f} m(門檻 10);"
          f"只看真觸地的 {st.median([x[6] for x in rec if x[7]] or [float('nan')]):.2f} m")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "RRD")
