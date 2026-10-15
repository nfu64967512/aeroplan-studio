#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""matlab/export_truth32.py — 把 RRD1-8 + RRE1-8(32 架次)的真值匯出成 CSV。

只讀 `runs/artifacts/`,只寫 `matlab/data/`。**不改任何既有 .py / .sh / tests/。**

    PYTHONIOENCODING=utf-8 python matlab/export_truth32.py

輸出
----
    matlab/data/truth32.csv                每架一列的放行狀態 + 真值
    matlab/data/traj/<trial>_U<sid>.csv     放行後的逐拍軌跡(逃逸半徑 20 km 收窗)
    matlab/data/truth32_README.md           欄位定義(手寫,不由本檔生成)

三個來源,各自負責它已經被驗證過的那一段
------------------------------------------
1. `scripts/research/ditcg/truth32.py`  —— 放行狀態(r/z/V/gamma/eps)、
   兩個時鐘(t_wall / t_sim)。已對 `runs/_rrd_rows.json` 逐位元驗證。
2. `scripts/wsl/stot_score.py --json`   —— **觸地量**(觸地誤差 / 殘餘高度 /
   觸地速度 / 判定)。⚠️ 這一段**不能**用 truth32 的取樣值:
   truth32 的 `r_touch_m` 是「首次 agl<=0 那一列的水平距」,沒有內插
   ⇒ 155 m/s × 0.05 s tick = 7.8 m 的量化殘渣,RRE 中位吐 4.79 m;
   stot_score 在 AGL=0 內插出穿越點,同一批吐 **0.80 m**(= batch_tally
   印的那個數字,也是交辦要求對上的那個)。**兩個都匯出,前綴分開。**
3. `scripts/research/ditcg/fidelity_long.py::replay_nz` —— nz 夾制比例
   (CSV 沒有這一欄,逐列重放 `tot_node._pn_cmd` 的指令側 nz)。

自我驗證(每次執行都會印,對不上會 exit 1)
------------------------------------------
    RRE 命中率(觸地誤差 <= 10 m)     必須 = 15/16
    RRE 觸地誤差中位                   必須落在 0.79-0.80 m
"""
from __future__ import annotations

import csv
import json
import math
import os
import statistics as st
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts", "research", "ditcg"))
sys.path.insert(0, os.path.join(ROOT, "scripts", "wsl", "sar_swarm_patches"))

import truth32                                                      # noqa: E402
import fidelity_long as FL                                          # noqa: E402

ART = os.path.join(ROOT, "runs", "artifacts")
OUT = os.path.join(HERE, "data")
TRAJ = os.path.join(OUT, "traj")
SCORE = os.path.join(ROOT, "scripts", "wsl", "stot_score.py")

ESCAPE_R_M = 20000.0     # 逃逸半徑;與 stot_score.py --escape-r 預設同值
HIT_ERR_M = 10.0         # 與 batch_tally.py::HIT_ERR_M 同值
SIZE_BUDGET_B = 5 * 1024 * 1024
DECIMATE_S = 0.2

COLS = [
    # ── 身分 ──
    "trial", "sysid", "tag",
    # ── 放行狀態(Guo 5 維 FSV + 時刻表)──
    "t_release_s", "r_rel_m", "z_rel_m", "V_rel_ms", "gs_rel_ms",
    "gamma_rel_deg", "eps_rel_deg", "eps_spoken_rel_deg",
    "lead_angle_csv_deg", "R_slant_rel_m", "dive_T_s", "t_rem0_s",
    # ── 真值:時間 ──
    "t_touch_row_s", "t_dive_wall_s", "t_freeze_s", "t_dive_sim_s",
    "t_touch_interp_s", "t_dive_interp_s",
    # ── 真值:幾何(canonical = stot_score 內插)──
    "err_touch_m", "z_res_m", "d1_min_m", "agl_min_m",
    # ── 真值:幾何(取樣值,無內插;來自 truth32)──
    "row_r_touch_m", "row_r_min_m", "row_z_at_rmin_m", "path_len_m",
    # ── 真值:速度 ──
    "V_touch_ms", "gs_touch_ms", "row_gs_touch_ms", "row_V_touch_ms",
    "row_gamma_touch_deg", "gs_peak_dive_ms",
    # ── 真值:重放重建 ──
    "nz_clip_frac", "nz_lo_frac", "nz_hi_frac", "nz_n",
    "climb_after_rel_m", "gam_min_deg",
    # ── 判定 ──
    "status", "hit_10m", "escaped", "closed_by", "n_rows_dive",
]


def _score_json(trial):
    """跑 stot_score.py --json,回傳 {sysid:int -> per_uav dict}(含視窗欄位)。"""
    csvf = os.path.join(ART, trial, f"tot_{trial}.csv")
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    r = subprocess.run([sys.executable, SCORE, csvf, "--json"],
                       capture_output=True, text=True, encoding="utf-8",
                       errors="replace", env=env)
    if r.returncode != 0:
        raise RuntimeError(f"{trial}: stot_score 失敗\n{r.stderr[-2000:]}")
    o = json.loads(r.stdout)
    win = o.get("window", {}).get("per_uav", {})
    out = {}
    for k, v in o.get("per_uav", {}).items():
        v = dict(v)
        v["closed_by"] = (win.get(k) or {}).get("closed_by", "")
        out[int(k)] = v
    return out


def _seg_idx(rows):
    """(i_rel, i_td) —— 與 truth32.extract_one 逐字同一組判斷。"""
    i_rel = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"), None)
    if i_rel is None:
        return None, None
    i_td = next((k for k in range(i_rel, len(rows))
                 if truth32._f(rows[k], "agl") <= 0.0), None)
    return i_rel, i_td


def _dequant(seg):
    """量測紀律:t 量化 0.1 s、同戳最多 2 筆 ⇒ 同戳第 k 筆補 +0.05k。"""
    out, run_t, k = [], None, 0
    for r in seg:
        t = truth32._f(r, "t")
        if run_t is not None and abs(t - run_t) < 1e-9:
            k += 1
        else:
            run_t, k = t, 0
        out.append(t + 0.05 * k)
    return out


def build():
    recs = truth32.load()                       # 32 列,已含放行狀態與兩個時鐘
    by_tag = {d["tag"]: d for d in recs}
    rows_out, traj = [], []

    for trial in truth32.RUNS:
        tlat, tlon = truth32._read_target(trial)
        sc = _score_json(trial)
        for sysid, rows in sorted(truth32._rows(trial).items()):
            tag = f"{trial}-U{sysid}"
            d = by_tag.get(tag)
            if d is None:
                continue
            i_rel, i_td = _seg_idx(rows)
            s = sc.get(sysid, {})
            nz = FL.replay_nz(rows, tlat, tlon) or {}

            seg = rows[i_rel:i_td + 1]
            agl_min = min(truth32._f(r, "agl") for r in seg)
            t_touch_i = s.get("impact_t_s")
            err = s.get("impact_miss_m")

            rows_out.append({
                "trial": trial, "sysid": sysid, "tag": tag,
                "t_release_s": d["t_release_s"],
                "r_rel_m": d["r_m"], "z_rel_m": d["z_m"], "V_rel_ms": d["V_ms"],
                "gs_rel_ms": truth32._f(rows[i_rel], "gs"),
                "gamma_rel_deg": d["gamma_deg"], "eps_rel_deg": d["eps_deg"],
                "eps_spoken_rel_deg": d["eps_spoken_deg"],
                "lead_angle_csv_deg": d["lead_angle_csv_deg"],
                "R_slant_rel_m": d["R_slant_m"],
                "dive_T_s": d["t_release_s"] + d["t_rem0_s"],
                "t_rem0_s": d["t_rem0_s"],
                "t_touch_row_s": d["t_touch_s"],
                "t_dive_wall_s": d["t_wall_s"], "t_freeze_s": d["t_freeze_s"],
                "t_dive_sim_s": d["t_sim_s"],
                "t_touch_interp_s": t_touch_i,
                "t_dive_interp_s": (None if t_touch_i is None
                                    else t_touch_i - d["t_release_s"]),
                "err_touch_m": err, "z_res_m": s.get("alt_agl_m"),
                "d1_min_m": s.get("d1_m"), "agl_min_m": agl_min,
                "row_r_touch_m": d["r_touch_m"], "row_r_min_m": d["r_min_m"],
                "row_z_at_rmin_m": d["z_at_rmin_m"], "path_len_m": d["path_len_m"],
                "V_touch_ms": s.get("impact_v_ms"),
                "gs_touch_ms": s.get("impact_gs_ms"),
                "row_gs_touch_ms": d["gs_touch_ms"],
                "row_V_touch_ms": d["V_touch_ms"],
                "row_gamma_touch_deg": d["gamma_touch_deg"],
                "gs_peak_dive_ms": s.get("gs_peak_ms"),
                "nz_clip_frac": nz.get("nz_clip"), "nz_lo_frac": nz.get("nz_lo"),
                "nz_hi_frac": nz.get("nz_hi"), "nz_n": nz.get("nz_n"),
                "climb_after_rel_m": nz.get("climb_m"),
                "gam_min_deg": nz.get("gam_min_meas_deg"),
                "status": s.get("status"),
                "hit_10m": (None if err is None else int(err <= HIT_ERR_M)),
                "escaped": int(bool(s.get("escaped"))),
                "closed_by": s.get("closed_by"),
                "n_rows_dive": d["n_rows"],
            })
            traj.append((tag, rows, i_rel, i_td, tlat, tlon))
    return rows_out, traj


def _traj_rows(rows, i_rel, i_td, tlat, tlon):
    """放行 → 逃逸半徑 20 km(或資料末端)。回傳 list[dict]。"""
    seg, k = [], i_rel
    while k < len(rows):
        r = rows[k]
        lat, lon = truth32._f(r, "lat"), truth32._f(r, "lon")
        rr = math.hypot((tlat - lat) * truth32.EARTH,
                        (tlon - lon) * truth32._mlon(lat))
        if rr > ESCAPE_R_M:
            break
        seg.append((k, r, rr))
        k += 1
    tdq = _dequant([x[1] for x in seg])
    t0 = truth32._f(rows[i_rel], "t")
    out = []
    for (k, r, rr), td in zip(seg, tdq):
        gam = math.radians(truth32._f(r, "gam_deg"))
        gs = truth32._f(r, "gs")
        out.append({
            "t": truth32._f(r, "t"), "t_dq": round(td, 4),
            "t_rel": round(truth32._f(r, "t") - t0, 4),
            "r": round(rr, 3), "z": truth32._f(r, "agl"),
            "V": round(gs / max(math.cos(gam), 1e-6), 4),
            "gamma_deg": truth32._f(r, "gam_deg"), "gs": gs,
            "mode": r.get("mode", ""),
            "lat": truth32._f(r, "lat"), "lon": truth32._f(r, "lon"),
            "post_touch": int(i_td is not None and k > i_td),
        })
    return out


def _decimate(rows, step_s):
    keep, last = [], None
    for i, r in enumerate(rows):
        first_post = (r["post_touch"] == 1 and (i == 0 or rows[i - 1]["post_touch"] == 0))
        if last is None or r["t_dq"] - last >= step_s - 1e-9 or first_post \
                or i == len(rows) - 1:
            keep.append(r)
            last = r["t_dq"]
    return keep


def main():
    os.makedirs(TRAJ, exist_ok=True)
    rows_out, traj = build()
    if len(rows_out) != 32:
        sys.exit(f"⚠️ 只抽到 {len(rows_out)} 架次(應為 32)—— 停下")

    p = os.path.join(OUT, "truth32.csv")
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLS, lineterminator="\n")
        w.writeheader()
        for d in rows_out:
            w.writerow({k: ("" if d.get(k) is None else d[k]) for k in COLS})
    print(f"寫出 {p}  ({len(rows_out)} 列、{len(COLS)} 欄、"
          f"{os.path.getsize(p) / 1024:.1f} KB)")

    # ── 逐拍軌跡 ──────────────────────────────────────────────────────
    built = [(tag, _traj_rows(rows, i_rel, i_td, tlat, tlon))
             for tag, rows, i_rel, i_td, tlat, tlon in traj]
    tcols = ["t", "t_dq", "t_rel", "r", "z", "V", "gamma_deg", "gs", "mode",
             "lat", "lon", "post_touch"]
    est = sum(len(t) for _, t in built) * 95
    dec = est > SIZE_BUDGET_B
    if dec:
        built = [(tag, _decimate(t, DECIMATE_S)) for tag, t in built]
    tot = 0
    for tag, t in built:
        trial, sid = tag.split("-U")
        fp = os.path.join(TRAJ, f"{trial}_U{sid}.csv")
        with open(fp, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=tcols, lineterminator="\n")
            w.writeheader()
            w.writerows(t)
        tot += os.path.getsize(fp)
    print(f"寫出 {len(built)} 個軌跡檔到 {TRAJ}  "
          f"(共 {sum(len(t) for _, t in built)} 列、{tot / 1024 / 1024:.2f} MB、"
          f"{'0.2 s 抽樣' if dec else '原始 ~20 Hz 全取'})")

    # ── 自我驗證 ──────────────────────────────────────────────────────
    rre = [d for d in rows_out if d["trial"].startswith("RRE")]
    err = [d["err_touch_m"] for d in rre if d["err_touch_m"] is not None]
    nhit = sum(d["hit_10m"] or 0 for d in rre)
    med = st.median(err)
    med_hit = st.median([e for e in err if e <= HIT_ERR_M])
    print(f"\n[自我驗證] RRE n={len(rre)}  觸地誤差可得 {len(err)} 架")
    print(f"  命中率(err <= {HIT_ERR_M:.0f} m) = {nhit}/{len(rre)}"
          f"   期望 15/16   {'✅' if nhit == 15 else '❌'}")
    print(f"  觸地誤差中位 = {med:.2f} m(只看命中 {med_hit:.2f} m)"
          f"   期望 0.79-0.80   {'✅' if 0.785 <= med <= 0.805 else '❌'}")
    rrd = [d for d in rows_out if d["trial"].startswith("RRD")]
    print(f"  [對照] RRD 命中率 = {sum(d['hit_10m'] or 0 for d in rrd)}/{len(rrd)}"
          f"、中位 {st.median([d['err_touch_m'] for d in rrd]):.2f} m")
    print(f"  [對照] 取樣值 row_r_touch_m 的 RRE 中位 = "
          f"{st.median([d['row_r_touch_m'] for d in rre]):.2f} m(未內插,勿用)")
    nzv = [d["nz_clip_frac"] * 100 for d in rows_out
           if d["nz_clip_frac"] is not None]
    print(f"  nz 夾制比例 {min(nzv):.2f}–{max(nzv):.2f} %"
          f"(中位 {st.median(nzv):.2f} %)、全為 0 的架數 "
          f"{sum(1 for v in nzv if v == 0)}/32")
    print(f"  真最低 AGL {min(d['agl_min_m'] for d in rows_out):.1f} … "
          f"{max(d['agl_min_m'] for d in rows_out):.1f} m、"
          f"<=0 的架數 {sum(1 for d in rows_out if d['agl_min_m'] <= 0)}/32")
    if nhit != 15 or not (0.785 <= med <= 0.805):
        sys.exit("❌ 自我驗證未通過")


if __name__ == "__main__":
    main()
