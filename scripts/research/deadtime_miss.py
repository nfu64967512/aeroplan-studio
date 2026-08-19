#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""縱向通道的**純延遲**模型:能不能解釋實測的沿航跡過頭量?

為什麼是純延遲而不是一階遲滯
--------------------------
`fit_pitch_channel.py` 的擬合 RMS 是 7–17°,而訊號平均只有 14° ——
一階遲滯**擬合不上**。原因在數據裡:γ 峰值 29.8–46.2°(**每架都超過指令 27°**)
而平均只有 9.9–18.3° ⇒ 響應是「前段拖著不下去、後段猛壓過頭」,
不是單調趨近。⇒ 正確的形式是 **dead time**,不是 lag。

兩段式模型(參數全部實測,零假設)
--------------------------------
    第一段(延遲 T_d):以實測的初期航跡角 γ0 飛
        水平走 V·cos(γ0)·T_d,高度只掉 V·sin(γ0)·T_d
    第二段:以實測的**峰值**航跡角 γ_pk 把剩餘高度卸完
        水平需要 h_rem / tan(γ_pk)

    預測落點水平距 = V·cos(γ0)·T_d + h_rem/tan(γ_pk)
    預測過頭量     = 上式 − d0(放行當下距紅心)

T_d 的定義:從放行(mode 首次為 dive)到 γ **首次穿越指令角** 的時間。
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

DEFAULT_ART = Path(__file__).resolve().parents[2] / "runs" / "artifacts"
T_TGT = (25.1261649, 120.6156480)
# 觀測值來自 err_axis 的沿航跡分解(負 = 飛過頭 ⇒ 這裡取絕對值)
OBS = {("RPK1", 1): 61.7, ("RPK1", 2): 499.0, ("RPK2", 1): 124.5,
       ("RPK2", 2): 656.5, ("RPK3", 2): 147.9}


def ml(lat):
    return 111320.0 * math.cos(math.radians(lat))


def movavg(xs, w):
    if w <= 1:
        return list(xs)
    out, n, half = [], len(xs), w // 2
    for i in range(n):
        a, b = max(0, i - half), min(n, i + half + 1)
        out.append(sum(xs[a:b]) / (b - a))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--art", default=str(DEFAULT_ART))
    ap.add_argument("--tags", default="RPK1,RPK2,RPK3")
    ap.add_argument("--cmd-deg", type=float, default=27.0)
    ap.add_argument("--win", type=int, default=21)
    a = ap.parse_args()

    print("%-6s %-5s | %-7s %-7s %-7s %-7s | %-9s %-9s %-7s"
          % ("趟", "機", "T_d(s)", "γ0(°)", "γ峰(°)", "d0(m)",
             "預測過頭", "觀測過頭", "比值"))
    print("-" * 94)
    pairs = []
    for tag in a.tags.split(","):
        p = Path(a.art) / tag / ("tot_%s.csv" % tag)
        if not p.exists():
            continue
        rows = {}
        with open(p, encoding="utf-8", errors="replace") as f:
            for r in csv.DictReader(f):
                try:
                    rows.setdefault(int(r["sysid"]), []).append(r)
                except (ValueError, TypeError):
                    continue
        for sid in sorted(rows):
            seg = [r for r in rows[sid]
                   if r["phase"] == "DIVE" and r["mode"] == "dive"]
            if len(seg) < 40:
                continue
            lo = min(range(len(seg)), key=lambda k: float(seg[k]["alt"]))
            if lo < 20:
                continue
            seg = seg[:lo + 1]
            t = [float(x["t"]) for x in seg]
            alt = movavg([float(x["alt"]) for x in seg], a.win)
            gs = movavg([max(float(x["gs"]), 1e-3) for x in seg], a.win)
            gam, tt, vv = [], [], []
            for i in range(1, len(seg) - 1):
                dt = t[i + 1] - t[i - 1]
                if dt <= 1e-6:
                    continue
                sink = -(alt[i + 1] - alt[i - 1]) / dt
                gam.append(math.degrees(math.atan2(sink, gs[i])))
                tt.append(t[i]); vv.append(math.hypot(gs[i], sink))
            gam = movavg(gam, a.win)
            if len(tt) < 20:
                continue
            # T_d:γ 首次穿越指令角
            td, i_cross = None, None
            for k, g in enumerate(gam):
                if g >= a.cmd_deg:
                    td, i_cross = tt[k] - tt[0], k
                    break
            if td is None:
                td, i_cross = tt[-1] - tt[0], len(tt) - 1
            g0 = sum(gam[:max(1, i_cross)]) / max(1, i_cross)   # 延遲段平均角
            gpk = max(gam)
            V0 = sum(vv[:max(1, i_cross)]) / max(1, i_cross)
            h0 = alt[0]
            d0 = math.hypot((float(seg[0]["lon"]) - T_TGT[1]) * ml(T_TGT[0]),
                            (float(seg[0]["lat"]) - T_TGT[0]) * 111320.0)
            drop = V0 * math.sin(math.radians(max(g0, 0.0))) * td
            h_rem = max(h0 - drop, 0.0)
            x1 = V0 * math.cos(math.radians(max(g0, 0.0))) * td
            x2 = h_rem / math.tan(math.radians(max(gpk, 1.0)))
            pred = (x1 + x2) - d0
            obs = OBS.get((tag, sid))
            ratio = ("%.2f" % (pred / obs)) if (obs and obs > 1) else "—"
            if obs:
                pairs.append((pred, obs))
            print("%-6s UAV%-2d | %-7.1f %-7.1f %-7.1f %-7.0f | %-9.0f %-9s %-7s"
                  % (tag, sid, td, g0, gpk, d0, pred,
                     ("%.0f" % obs) if obs else "—", ratio))

    if pairs:
        rs = sorted(p / o for p, o in pairs if o > 1)
        print()
        print("── 預測/觀測 比值:中位 %.2f  範圍 %.2f – %.2f"
              % (rs[len(rs) // 2], rs[0], rs[-1]))
        print("   (1.0 附近 ⇒ 純延遲模型足以解釋過頭量 ⇒ 物理必然,不是導引律算錯)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
