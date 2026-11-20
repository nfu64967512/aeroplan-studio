#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""truth32.py —— 從 RRD1-8 + RRE1-8(共 32 架次)抽出放行狀態與回測真值。

只讀 `runs/artifacts/{RRD,RRE}{1..8}/tot_*.csv`,不寫任何東西進 repo。

## eps 的定義:**取 sigma(純水平方位差),不取 A2 的 arcsin(cos gamma sin sigma)**

A2 用了 Guo §II.A 的**口語**定義(速度向量與 LOS 鉛垂平面的夾角)算出
`eps = arcsin(cos(gamma) * sin(sigma))`;A1 主張該用 Eq.(27)-(29) 的運動學版
`eps = sigma`。**兩者不能都對,而自洽性可以直接判決:**

    rdot = d/dt(水平距) = -(水平速度在 LOS 方向的分量) = -V cos(gamma) cos(sigma)

Guo Eq.(29) 寫的是 `epsdot = psidot + V cos(gamma) sin(eps)/r`,而 Eq.(18) 的 r
是水平距 ⇒ 與它成對的 rdot 必然是 `-V cos(gamma) cos(eps)`。
把 `eps = arcsin(cos(gamma) sin(sigma))` 代進去,`cos(eps) != cos(sigma)`
(gamma=-49°、sigma=45° 時 cos 差 0.18)⇒ 運動學不自洽。
**⇒ 本檔與 sim3d.py 一律用 `eps = sigma`(帶號)。**
差別的量級:放行點 |eps - eps_A2| 中位 0.069°、最壞 6.28°(RRD1-U2,gamma=-21°);
俯衝中後段(gamma 到 -52°)可到 17°。

## 三個資料層陷阱(A2 已實地驗過,照做)

1. **「首次 AGL <= 0」不能在 `mode == "dive"` 的列裡找。** 放行後飛機會在
   AGL 10-70 m 轉成 `mode == "expended"`,過零發生在那之後 ⇒ 照字面過濾
   32 架次只有 2 架找得到觸地。正確作法:release = 首次 `mode=="dive"`;
   touchdown = 從該索引起、**不限 mode** 的首次 `agl <= 0`。
2. **必須取首次過零。** 觸地後 CSV 還會記錄十幾秒(SITL 不終結落海機體),
   取 min(agl) 或最後一列會撞上 second-pass-contamination。
3. `gs` 是**水平**速度 ⇒ V = gs/cos(gamma);`alt` 是相對 HOME ⇒ z 要用 `agl`。

## 兩個時鐘

CSV 的 `t` 是 `time.time()`(牆鐘)。每趟俯衝夾著 1-2 次 dt > 0.5 s 的空檔,
空檔內機體只移動 3.4-24.2 m(若真飛了 3.7 s @190 m/s 應該是 ~700 m)
⇒ **模擬時鐘實質停住**。三條獨立證據見 A2 的 `a2_gap.py`。
本檔兩個都輸出:`t_wall`(= 交辦的實測值,中位 41.20 s)與
`t_sim = t_wall - 凍結總長`(中位 37.90 s)。**回報時必須說明用哪一個。**
"""
from __future__ import annotations

import csv
import math
import os

BASE = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..",
    "runs", "artifacts"))
RUNS = [f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]
EARTH = 111320.0        # ⚠️ 與 tot_node.py:51 逐位元相同,否則換算對不上
GAP_S = 0.3             # 相鄰列時距超過此值 = 節點凍結空檔
FD_HALF = 4             # 航向有限差分的窗半寬(列)。A2 已掃過敏感度:
                        # |hw2-hw4| 中位 0.033°、|hw4-hw8| 中位 0.123°


def _mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def _read_target(run):
    p = os.path.join(BASE, run, f"tot_{run}_geom.csv")
    with open(p, newline="", encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            if row["kind"] == "target":
                return float(row["lat"]), float(row["lon"])
    raise RuntimeError(f"{run}: geom.csv 沒有 target 列")


def _rows(run):
    p = os.path.join(BASE, run, f"tot_{run}.csv")
    with open(p, newline="", encoding="utf-8", errors="replace") as f:
        out = {}
        for row in csv.DictReader(f):
            out.setdefault(int(row["sysid"]), []).append(row)
    return out


def _f(row, key, default=float("nan")):
    v = row.get(key, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _course_rad(rows, k):
    """航跡方位角(rad,由北順時針)。⚠️ 只用位移**方向**,不除 dt
    ⇒ 完全免疫 t 的 0.1 s 量化。窗不得跨越凍結空檔(跨了就往內縮)。"""
    n = len(rows)
    lo, hi = max(0, k - FD_HALF), min(n - 1, k + FD_HALF)
    while lo < k and _f(rows[lo + 1], "t") - _f(rows[lo], "t") > GAP_S:
        lo += 1
    while hi > k and _f(rows[hi], "t") - _f(rows[hi - 1], "t") > GAP_S:
        hi -= 1
    if hi <= lo:
        lo, hi = max(0, k - 1), min(n - 1, k + 1)
    la0, lo0 = _f(rows[lo], "lat"), _f(rows[lo], "lon")
    la1, lo1 = _f(rows[hi], "lat"), _f(rows[hi], "lon")
    dn = (la1 - la0) * EARTH
    de = (lo1 - lo0) * _mlon(0.5 * (la0 + la1))
    return math.atan2(de, dn), math.hypot(dn, de)


def extract_one(run, sysid, rows, tlat, tlon):
    """回傳單機的放行狀態 + 真值 dict;抽不出來回 None。"""
    i_rel = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"), None)
    if i_rel is None:
        return None
    i_td = next((k for k in range(i_rel, len(rows)) if _f(rows[k], "agl") <= 0.0),
                None)
    if i_td is None:
        return None

    r0 = rows[i_rel]
    lat, lon = _f(r0, "lat"), _f(r0, "lon")
    dn = (tlat - lat) * EARTH
    de = (tlon - lon) * _mlon(lat)
    r_h = math.hypot(dn, de)
    z0 = _f(r0, "agl")
    gam0 = math.radians(_f(r0, "gam_deg"))
    gs0 = _f(r0, "gs")
    V0 = gs0 / max(math.cos(gam0), 1e-6)

    # eps(帶號):速度水平投影 與 水平 LOS 的夾角。>0 = 速度在 LOS 右側。
    # 與 tot_node._lead_angle_deg 同定義,只是不取 abs。
    chi, _span = _course_rad(rows, i_rel)
    vn, ve = math.cos(chi), math.sin(chi)
    cross = dn * ve - de * vn
    dot = dn * vn + de * ve
    eps0 = math.atan2(cross, dot)
    # A2 用的口語版,留著當對照(見檔頭:自洽性判決取 sigma)
    eps0_spoken = math.asin(max(-1.0, min(1.0, math.cos(gam0) * math.sin(eps0))))

    # 油門律用的 t_rem:early_by_s = (dive_T - now) - secs_to_target
    #                ⇒ dive_T - now = early_by_s + secs_to_target
    t_rem0 = _f(r0, "early_by_s") + _f(r0, "secs_to_target")

    seg = rows[i_rel:i_td + 1]
    t_rel, t_td = _f(r0, "t"), _f(rows[i_td], "t")
    t_wall = t_td - t_rel
    freeze = 0.0
    for a, b in zip(seg, seg[1:]):
        dt = _f(b, "t") - _f(a, "t")
        if dt > GAP_S:
            freeze += dt

    # 觸地狀態:取過零**前**最後一列(過零那列 agl 已為負,位置外插過頭)
    r_last = rows[max(i_rel, i_td - 1)]
    gam_td = math.radians(_f(r_last, "gam_deg"))
    gs_td = _f(r_last, "gs")
    V_td = gs_td / max(math.cos(gam_td), 1e-6)
    lat_td, lon_td = _f(rows[i_td], "lat"), _f(rows[i_td], "lon")
    r_td = math.hypot((tlat - lat_td) * EARTH, (tlon - lon_td) * _mlon(lat_td))

    # 航跡長度(位置積分)+ 最近通過
    path = 0.0
    r_min, z_at_rmin = r_h, z0
    for a, b in zip(seg, seg[1:]):
        la0, ln0 = _f(a, "lat"), _f(a, "lon")
        la1, ln1 = _f(b, "lat"), _f(b, "lon")
        dh = math.hypot((la1 - la0) * EARTH, (ln1 - ln0) * _mlon(la0))
        path += math.hypot(dh, _f(b, "agl") - _f(a, "agl"))
        rb = math.hypot((tlat - la1) * EARTH, (tlon - ln1) * _mlon(la1))
        if rb < r_min:
            r_min, z_at_rmin = rb, _f(b, "agl")

    return {
        "run": run, "sysid": sysid, "tag": f"{run}-U{sysid}",
        # ── 放行狀態(Guo 5 維 FSV)──
        "r_m": r_h, "z_m": z0, "V_ms": V0,
        "gamma_deg": math.degrees(gam0), "eps_deg": math.degrees(eps0),
        "eps_spoken_deg": math.degrees(eps0_spoken),
        "lead_angle_csv_deg": _f(r0, "lead_angle_deg"),
        "R_slant_m": math.hypot(r_h, z0),
        "t_rem0_s": t_rem0,
        # ── 真值 ──
        "t_release_s": t_rel, "t_touch_s": t_td,
        "t_wall_s": t_wall, "t_freeze_s": freeze, "t_sim_s": t_wall - freeze,
        "gs_touch_ms": gs_td, "V_touch_ms": V_td,
        "gamma_touch_deg": math.degrees(gam_td),
        "r_touch_m": r_td, "r_min_m": r_min, "z_at_rmin_m": z_at_rmin,
        "path_len_m": path, "n_rows": len(seg),
    }


def load(runs=None):
    """回傳 32 個 dict 的 list(按 run / sysid 排序)。"""
    out = []
    for run in (runs or RUNS):
        tlat, tlon = _read_target(run)
        by_sys = _rows(run)
        for sysid in sorted(by_sys):
            rec = extract_one(run, sysid, by_sys[sysid], tlat, tlon)
            if rec is not None:
                out.append(rec)
    return out


if __name__ == "__main__":
    import statistics as st
    recs = load()
    print(f"n = {len(recs)}")
    hdr = ("tag", "r_m", "z_m", "V_ms", "gam", "eps", "t_rem0",
           "t_wall", "frz", "t_sim", "gs_td", "r_td")
    print("{:<9s}{:>7s}{:>7s}{:>7s}{:>7s}{:>7s}{:>8s}{:>8s}{:>6s}{:>8s}"
          "{:>7s}{:>8s}".format(*hdr))
    for d in recs:
        print(f"{d['tag']:<9s}{d['r_m']:7.0f}{d['z_m']:7.0f}{d['V_ms']:7.1f}"
              f"{d['gamma_deg']:7.2f}{d['eps_deg']:7.2f}{d['t_rem0_s']:8.2f}"
              f"{d['t_wall_s']:8.2f}{d['t_freeze_s']:6.1f}{d['t_sim_s']:8.2f}"
              f"{d['gs_touch_ms']:7.1f}{d['r_touch_m']:8.1f}")
    for k in ("t_wall_s", "t_sim_s", "gs_touch_ms", "r_touch_m", "t_rem0_s"):
        v = [d[k] for d in recs]
        print(f"  {k:<12s} median {st.median(v):8.2f}  min {min(v):8.2f}  "
              f"max {max(v):8.2f}  sd {st.pstdev(v):6.2f}")
