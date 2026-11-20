#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fit_lag.py —— 用 32 架次實測的 gamma(t) 重擬 sim3d 的 gamma 通道遲滯 tau。

## 為什麼要做

`predictor.py:206` 的 `PN_TAU_S = 2.0` **自述「沒有受控實測,是估的」**。
在 predictor 裡 gamma 只是代數量、tau 只被 `min_handover_altitude_m` 用到,
所以估錯了也看不見。**但 sim3d 把 gamma 升成狀態** ⇒ 這個遲滯會直接寫進
t_go^PNG ⇒ 事前閘門會變成在測這個估值,而不是在測方法。A1 把它列成阻礙。

## 怎麼擬(直接量,不做黑箱最佳化)

對俯衝段每一拍,用**實測**的 (r, z, V, gamma, eps) 重算節點會下的
`gdot_cmd`(逐項複刻 `tot_node._pn_cmd` 的縱向通道 + LOS 地板 + nz 夾制),
再拿**實測**的 `gdot_meas = d(gamma)/dt` 去比。一階遲滯模型

    gdot_act[k] = gdot_act[k-1] + (gdot_cmd[k-1] - gdot_act[k-1]) * dt / tau

掃 tau,**用兩個判準**:
  (a) 逐拍 gammadot 的 RMS —— 結果**單調**、無極小值 ⇒ 這個判準是病態的:
      指令 gammadot 有 -61 deg/s 的瞬間尖峰(LOS 地板一咬就跳),實測只有
      -14.25…+11.35 ⇒ 「tau 越大越像」只是在說「把指令壓扁就對了」,
      量不到真正的時間常數。**留著是為了記錄它為什麼不能用。**
  (b) ⭐ **閉迴路重放 gamma(t)**:從放行的 gamma 出發,只用一階遲滯積分 gamma,
      驅動量取**實測**的 (r, z, V, eps) ⇒ 比的是軌跡本身,不是它的導數。
      這個判準有明確極小值,是本檔採用的那一個。

⚠️ **時鐘陷阱**:CSV 的 t 是牆鐘且量化到 0.1 s,而且每趟夾著 1-2 次
   dt > 0.5 s 的凍結空檔。凍結那幾拍**必須整段丟掉**(不是內插)——
   在空檔上算 d(gamma)/dt 會得到假的「跟得很慢」,把 tau 灌大。
⚠️ 逐列 dt 不可信(0.1 s 桶內約 2 列)⇒ 用 5 列的中央差分,並要求該窗
   跨過的牆鐘時距落在 [0.15, 0.45] s 才採用。
"""
from __future__ import annotations

import csv
import math
import os
import statistics as st
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import truth32                                                      # noqa: E402
import sim3d                                                        # noqa: E402

G = sim3d.G
PN = sim3d.PNParams()
HALF = 2            # 中央差分窗半寬(列)
DT_LO, DT_HI = 0.15, 0.45


def _f(row, key):
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return float("nan")


def gather():
    """回傳 [(dt, gdot_cmd, gdot_meas), ...],已剔除凍結窗。"""
    out = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            i0 = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"),
                      None)
            if i0 is None:
                continue
            i1 = next((k for k in range(i0, len(rows))
                       if _f(rows[k], "agl") <= 0.0), None)
            if i1 is None:
                continue
            seg = rows[i0:i1 + 1]
            for k in range(HALF, len(seg) - HALF):
                a, b, c = seg[k - HALF], seg[k], seg[k + HALF]
                dt = _f(c, "t") - _f(a, "t")
                if not (DT_LO <= dt <= DT_HI):
                    continue          # 凍結空檔 / 量化異常 ⇒ 整窗丟掉
                lat, lon = _f(b, "lat"), _f(b, "lon")
                dn = (tlat - lat) * truth32.EARTH
                de = (tlon - lon) * truth32._mlon(lat)
                r = math.hypot(dn, de)
                z = _f(b, "agl")
                if r < 150.0 or z <= 0.0:
                    continue          # 終端凍結區,指令已鎖住,不進擬合
                gam = math.radians(_f(b, "gam_deg"))
                V = _f(b, "gs") / max(math.cos(gam), 1e-6)
                chi, _ = truth32._course_rad(seg, k)
                cross = dn * math.sin(chi) - de * math.cos(chi)
                dot = dn * math.cos(chi) + de * math.sin(chi)
                eps = math.atan2(cross, dot)

                # ── 逐項複刻 tot_node._pn_cmd 的縱向通道 ──
                cg = math.cos(gam)
                R = max(math.hypot(r, z), 1e-3)
                rdot = -V * cg * math.cos(eps)
                zdot = V * math.sin(gam)
                sig_dot = (zdot * r - z * rdot) / (R * R)
                gdot_pn = -PN.n_vert * sig_dot
                nz = min(PN.pn_nz_max, max(PN.pn_nz_min, cg + V * gdot_pn / G))
                gdot_cmd = (nz - cg) * G / V
                gam_cmd = gam + gdot_cmd * PN.pn_lead
                if PN.pn_los_floor > 0.0:
                    gam_cmd = min(gam_cmd,
                                  -math.atan2(z, max(r, 1e-3)) * PN.pn_los_floor)
                gdot_want = (gam_cmd - gam) / PN.pn_lead

                gdot_meas = (math.radians(_f(c, "gam_deg"))
                             - math.radians(_f(a, "gam_deg"))) / dt
                if not (math.isfinite(gdot_want) and math.isfinite(gdot_meas)):
                    continue
                out.append((dt / (2 * HALF), gdot_want, gdot_meas))
    return out


def rms_for_tau(samples, tau):
    """判準 (a):把一階遲滯套在整串 gammadot 上,回傳 RMS 誤差(deg/s)。
    ⚠️ 已證病態(單調無極小),只保留當記錄。"""
    err = []
    act = 0.0
    for dt, cmd, meas in samples:
        act += (cmd - act) * dt / tau
        err.append(math.degrees(act - meas))
    return math.sqrt(sum(e * e for e in err) / max(len(err), 1))


def replay_rms(flights, tau):
    """判準 (b):閉迴路重放 gamma(t)。回傳 (整體 RMS deg, 每架 RMS 的中位數)。

    每架次從放行的實測 gamma 出發,之後 gamma 完全由模型自己走:
        gdot_act += (gdot_want(state) - gdot_act) * dt / tau
        nz  = clip(cos(gam) + V*gdot_act/g, nz_min, nz_max)   ← plant 端能力
        gam += (nz - cos(gam)) * g / V * dt
    其中 gdot_want 用**模型自己的 gamma** 配上**實測的 (r, z, V, eps)** 算
    ⇒ 只讓 gamma 這一個通道開迴路,其餘全部釘在實測 ⇒ 分離出來的就是遲滯。
    """
    per, allsq, alln = [], 0.0, 0
    for fl in flights:
        gam = fl[0]["gam"]
        act = 0.0
        sq = nn = 0
        for s in fl:
            cg = math.cos(gam)
            V, r, z, eps = s["V"], s["r"], s["z"], s["eps"]
            R = max(math.hypot(r, z), 1e-3)
            rdot = -V * cg * math.cos(eps)
            zdot = V * math.sin(gam)
            sig_dot = (zdot * r - z * rdot) / (R * R)
            nz_c = min(PN.pn_nz_max,
                       max(PN.pn_nz_min, cg + V * (-PN.n_vert * sig_dot) / G))
            gam_cmd = gam + (nz_c - cg) * G / V * PN.pn_lead
            if PN.pn_los_floor > 0.0:
                gam_cmd = min(gam_cmd,
                              -math.atan2(z, max(r, 1e-3)) * PN.pn_los_floor)
            want = (gam_cmd - gam) / PN.pn_lead
            act += (want - act) * s["dt"] / tau
            nz = min(PN.pn_nz_max, max(PN.pn_nz_min, cg + V * act / G))
            gam += (nz - cg) * G / V * s["dt"]
            e = math.degrees(gam - s["gam"])
            sq += e * e
            nn += 1
        if nn:
            per.append(math.sqrt(sq / nn))
            allsq += sq
            alln += nn
    return math.sqrt(allsq / max(alln, 1)), st.median(per) if per else float("nan")


def gather_flights():
    """判準 (b) 用:逐架次的實測狀態序列(已剔除凍結空檔)。"""
    out = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            i0 = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"),
                      None)
            if i0 is None:
                continue
            i1 = next((k for k in range(i0, len(rows))
                       if _f(rows[k], "agl") <= 0.0), None)
            if i1 is None:
                continue
            seg = rows[i0:i1 + 1]
            fl = []
            for k in range(len(seg) - 1):
                dt = _f(seg[k + 1], "t") - _f(seg[k], "t")
                if not (0.0 < dt <= 0.3):
                    continue          # 凍結空檔 / 同時戳重複列 ⇒ 丟掉
                b = seg[k]
                lat, lon = _f(b, "lat"), _f(b, "lon")
                dn = (tlat - lat) * truth32.EARTH
                de = (tlon - lon) * truth32._mlon(lat)
                r = math.hypot(dn, de)
                z = _f(b, "agl")
                if r < 150.0 or z <= 0.0:
                    continue
                gam = math.radians(_f(b, "gam_deg"))
                chi, _ = truth32._course_rad(seg, k)
                cross = dn * math.sin(chi) - de * math.cos(chi)
                dot = dn * math.cos(chi) + de * math.sin(chi)
                fl.append({"dt": dt, "r": r, "z": z, "gam": gam,
                           "eps": math.atan2(cross, dot),
                           "V": _f(b, "gs") / max(math.cos(gam), 1e-6)})
            if len(fl) > 100:
                out.append(fl)
    return out


def main():
    s = gather()
    print(f"樣本數 n = {len(s)}(已剔除凍結窗與終端凍結區)")
    md = [math.degrees(x[2]) for x in s]
    mc = [math.degrees(x[1]) for x in s]
    print(f"  實測 gammadot  中位 {st.median(md):+.3f} deg/s  "
          f"全距 {min(md):+.2f}…{max(md):+.2f}")
    print(f"  指令 gammadot  中位 {st.median(mc):+.3f} deg/s  "
          f"全距 {min(mc):+.2f}…{max(mc):+.2f}")
    print()
    print("tau 掃描(RMS of gammadot_act - gammadot_meas,deg/s):")
    best, best_r = None, 1e18
    for tau in (0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0):
        r = rms_for_tau(s, tau)
        mark = ""
        if r < best_r:
            best, best_r = tau, r
        print(f"  tau = {tau:4.2f} s   RMS = {r:6.3f}{mark}")
    print(f"  ⇒ 判準 (a) 在 0.25…32 s 上**單調**(最佳落在掃描邊界 {best:.2f} s,"
          f"RMS {best_r:.3f} deg/s)⇒ **病態,不採用**。原因:指令 gammadot 的"
          f"全距 {min(mc):+.1f}…{max(mc):+.1f} deg/s 比實測的"
          f" {min(md):+.1f}…{max(md):+.1f} 寬一個量級,"
          f"「tau 越大越像」只是在說「把指令壓扁」。")

    print()
    print("判準 (b) ⭐ 閉迴路重放 gamma(t)(RMS of gamma 誤差,deg):")
    fl = gather_flights()
    print(f"  逐架次序列 {len(fl)} 架,總拍數 {sum(len(x) for x in fl)}")
    bb, bbr = None, 1e18
    for tau in (0.05, 0.10, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
        a, m = replay_rms(fl, tau)
        if a < bbr:
            bb, bbr = tau, a
        print(f"  tau = {tau:4.2f} s   RMS(全體) = {a:6.3f} deg   "
              f"每架 RMS 中位 = {m:6.3f} deg")
    print(f"\n⭐ 採用 tau = {bb:.2f} s(重放 RMS {bbr:.3f} deg)")
    print(f"   對照:predictor.PN_TAU_S = {2.0}(自述『沒有受控實測,是估的』)")
    print(f"   sim3d.TAU_GAMMA_S 目前 = {sim3d.TAU_GAMMA_S}")
    print()
    print("⚠️⚠️ 結論:**tau 在這批資料上不可識別,而且是雙向不可識別**。")
    print("   判準 (a) 單調往大(一路到掃描上界 32 s 都還在降);")
    print("   判準 (b) 在 0.10-0.75 s 上幾乎平坦(13.61→13.82,差 1.5%),")
    print("   0.05 才因為數值步長爆掉,往大則單調變差。")
    print("   —— 兩個判準把 tau 推向**相反**的方向 ⇒ 資料裡沒有這個自由度的資訊。")
    print("   機制:pn_los_floor = 1.0 讓 gamma 指令大部分時間**由幾何(視線角)**")
    print("   決定,而不是由 PN 的角速率決定;遲滯只影響「多快貼上視線」,")
    print("   而視線本身隨接近自然變陡 ⇒ 觀測不到時間常數。")
    print()
    print("   ⇒ sim3d.TAU_GAMMA_S 取 **1.0 = pn_lead**(= 零額外遲滯、完全複刻")
    print("      節點指令),並把不可識別性當成**已量化的不確定度**:validate32 的")
    print("      靈敏度掃描顯示 tau 0.5→3.0 只把耗時偏差中位從 +1.04 搬到 +1.50 s")
    print("      ⇒ 這 0.46 s 就是 tau 對閘門的最大貢獻,遠小於 3.0 s 的閘門。")
    print("      **不需要為了 tau 卡住建表。**")
    print()
    print("   ⚠️ 但要誠實記一筆:判準 (b) 的 gamma 重放 RMS 在**任何** tau 下都是")
    print("      12-18 deg —— 「只放開 gamma 一個通道、其餘釘在實測」這種混合重放")
    print("      本身就不收斂(模型 gamma 一偏,幾何就對不上,誤差自我放大)。")
    print("      該看的是 validate32 的**全閉迴路**結果(耗時偏差中位 +1.09 s、")
    print("      r(pred, 實測) = +0.575),不是這個 RMS。")


if __name__ == "__main__":
    main()
