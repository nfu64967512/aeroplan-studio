#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s1_readout.py — 抵環即放行(S1)的判讀。

⚠️⚠️ **觸地一律用嚴格判準**:放行後首次 `agl ≤ 30 m` **且** 平距 ≤ 200 m。

   節點的 `expended` **只看水平距、不看高度**([[hit-criterion-ignores-altitude]]),
   RRB 七趟 14 架次裡有 4 架是高空掠過(最近點高度 407 / 585 / 109 / 88 m)
   卻被記成命中。

⚠️⚠️ **反方向一樣會錯**:掃描窗**不可以**收在 `expended`。
   那只是水平距的鎖存,鎖存當下飛機通常還在空中(RRC1 是 +16 / +15 m),
   之後才真的落地(AGL 0.14 / 0.01 m)。收在 expended 會把真觸地誤判成高空掠過
   —— 第一版就是這樣,把 RRB 算成 3/14,而正確是 7/14。
   ⇒ 掃到 AGL ≤ 0 為止,但用首航次遲滯擋掉第二趟。

⚠️ **精度一律以 `stot_score.py` 的觸地誤差為準**,不是節點鎖存的水平距 ——
   兩者可差兩個數量級(RRC1:鎖存 21.3 / 20.1 m,真觸地 **0.18 / 0.03 m**)。

⚠️ 使用者 2026-08-18 定調:**先把逐機真觸地率做到 ≥90%,再談散布**;
   而散布的驗收範圍是**單群 8 架**的 max−min ≤ 3 s
   ⇒ 逐機時序 σ 必須 ≤ 3/d2(8) = 1.05 s。

用法:
  python3 s1_readout.py RRC1 [RRC2 ...]            # 指定趟次
  python3 s1_readout.py --glob 'RRB?'              # 當基準對照
"""
import argparse
import collections
import csv
import glob as globmod
import math
import pathlib
import statistics as st

ROOT = pathlib.Path(__file__).resolve().parents[2]
ART = ROOT / "runs" / "artifacts"
R_EARTH = 6378137.0

#: 嚴格觸地判準。⚠️ 兩個條件都要,缺一不可。
TOUCH_AGL_M = 30.0
TOUCH_D_M = 200.0

#: 逃逸半徑:飛出這個距離就收窗,不採計之後的任何接近。
#: ⚠️ SITL 不終結落海機體 —— 沉水再爬起會重新穿越並被記成命中
#:    ([[second-pass-contamination]]:SW1 的 22.6/67.1 是污染值,真值 32.6/217.9)。
#: ⚠️ 與 `stot_score.py` 的首航次窗用同一個數,兩支才可以互相對照。
ESCAPE_R_M = 20000.0

#: 期望全距係數 d2(n)(常態母體的 range 期望 / σ)。
D2 = {2: 1.128, 3: 1.693, 4: 2.059, 6: 2.534, 8: 2.847, 16: 3.532}


def dist_m(a, T):
    la = math.radians((a[0] + T[0]) / 2)
    return math.hypot(math.radians(T[0] - a[0]) * R_EARTH,
                      math.radians(T[1] - a[1]) * R_EARTH * math.cos(la))


def target_of(trial):
    """紅心座標。優先讀 geom.csv,讀不到就用互動路徑的固定 payload。"""
    g = ART / trial / f"tot_{trial}_geom.csv"
    if g.exists():
        try:
            for r in csv.DictReader(open(g, encoding="utf-8", errors="replace")):
                if r.get("tgt_lat"):
                    return (float(r["tgt_lat"]), float(r["tgt_lon"]))
        except Exception:
            pass
    return (25.1261649, 120.6156480)


def load(trial):
    f = ART / trial / f"tot_{trial}.csv"
    if not f.exists():
        return None
    T = target_of(trial)
    seq = collections.defaultdict(list)
    for r in csv.DictReader(open(f, encoding="utf-8", errors="replace")):
        try:
            seq[r["sysid"]].append({
                "t": float(r["t"]),
                "d": dist_m((float(r["lat"]), float(r["lon"])), T),
                "lat": float(r["lat"]), "lon": float(r["lon"]),
                "agl": float(r["agl"]) if r.get("agl") not in (None, "") else float("nan"),
                "mode": r["mode"], "phase": r["phase"],
                "gs": float(r["gs"]) if r.get("gs") else float("nan"),
                "sig": (abs(float(r["lead_angle_deg"]))
                        if r.get("lead_angle_deg") not in (None, "") else float("nan")),
            })
        except Exception:
            pass
    return T, seq


def per_aircraft(Q):
    """回傳單機的判讀:放行點、航跡比、嚴格觸地。"""
    k = next((i for i, x in enumerate(Q) if x["mode"] == "dive"), None)
    out = {"released": k is not None, "hold_ring_rows":
           sum(1 for x in Q if x["mode"] == "hold_ring")}
    # 抵環(第一次 d <= 6084 附近用實際環半徑不可得 ⇒ 用 mode 轉換佐證)
    out["t_ring"] = next((x["t"] for x in Q if x["d"] <= 6084.0), None)
    if k is None:
        out["closest_d"] = min((x["d"] for x in Q), default=float("nan"))
        return out
    r0 = Q[k]
    out.update(t_rel=r0["t"], R_rel=r0["d"], sig_rel=r0["sig"], alt_rel=r0["agl"])
    path = 0.0
    touch_t = None
    closest = (float("inf"), None)
    for a, b in zip(Q[k:], Q[k + 1:]):
        path += math.hypot(
            math.radians(b["lat"] - a["lat"]) * R_EARTH,
            math.radians(b["lon"] - a["lon"]) * R_EARTH * math.cos(math.radians(a["lat"])))
        if b["d"] < closest[0]:
            closest = (b["d"], b["agl"])
        # ⚠️ 嚴格觸地:高度**與**距離都要滿足
        if touch_t is None and b["agl"] == b["agl"] \
                and b["agl"] <= TOUCH_AGL_M and b["d"] <= TOUCH_D_M:
            touch_t = b["t"]
        # ⚠️⚠️ **不可以**在 `expended` 就收窗。
        #    `expended` 是節點的 impact 鎖存(只看水平距 ≤ impact_reach),
        #    而鎖存當下飛機通常還在空中 —— RRC1 鎖存時是 +16 / +15 m,
        #    之後才真的落到 AGL 0.14 / 0.01 m(stot_score.py 的觸地誤差
        #    是 0.18 / 0.03 m,不是鎖存的 21.3 / 20.1 m)。
        #    收在 expended 會把**真觸地**誤判成「高空掠過」。
        # ⇒ 掃到真的落地(AGL ≤ 0)為止。
        # ⚠️ 但要防 [[second-pass-contamination]]:SITL 不終結落海機體,
        #    沉水再爬起會做第二趟並重新穿越 ⇒ 用**逃逸半徑**收窗。
        # ⚠️⚠️ 收窗判準**不可以**是「飛出放行距離 + 遲滯」——
        #    σ>90° 放行的機體一開始就是**在拉開距離**的(ṙ = −V·cos σ > 0),
        #    那樣寫會在放行後幾秒就把窗關掉,航跡比變成 0.19–0.56 的垃圾,
        #    而且真觸地會全部漏掉。第一版就是這樣錯的。
        #    ⇒ 用固定的逃逸半徑(與 stot_score.py 同一個數)。
        if b["d"] > ESCAPE_R_M:
            break
        if b["agl"] == b["agl"] and b["agl"] <= 0.0:
            break
    out.update(path=path, ratio=path / max(r0["d"], 1.0),
               t_touch=touch_t, closest_d=closest[0], closest_agl=closest[1],
               node_expended=any(x["mode"] in ("expended", "impact") for x in Q))
    return out


def report(trials):
    print(f"{'趟/機':10}{'hold_ring':>10}{'放行R':>8}{'放行σ':>8}"
          f"{'航跡':>8}{'**比值**':>9}{'最近d':>8}{'最近agl':>9}   判定")
    rows = []
    for tr in trials:
        got = load(tr)
        if got is None:
            print(f"{tr:10}  ⚠️ 沒有 CSV —— 這一趟**不存在**,不可以算進統計")
            continue
        _, seq = got
        for s, Q in sorted(seq.items()):
            a = per_aircraft(Q)
            rows.append((tr, s, a))
            if not a["released"]:
                print(f"{tr}/U{s:<6}{a['hold_ring_rows']:>10}"
                      f"{'—':>8}{'—':>8}{'—':>8}{'—':>9}"
                      f"{a['closest_d']:8.0f}{'—':>9}   ❌ 從未放行")
                continue
            touched = a["t_touch"] is not None
            verdict = ("✅ 真觸地" if touched
                       else ("⚠️ 高空掠過(節點記命中)" if a["node_expended"]
                             else "❌ 未觸地"))
            print(f"{tr}/U{s:<6}{a['hold_ring_rows']:>10}{a['R_rel']:8.0f}"
                  f"{a['sig_rel']:8.1f}{a['path']:8.0f}{a['ratio']:9.2f}"
                  f"{a['closest_d']:8.0f}{a['closest_agl']:9.0f}   {verdict}")

    ok = [a for _, _, a in rows if a["released"]]
    if not ok:
        print("\n⚠️ 沒有任何一架放行 —— 無法判讀")
        return
    n_touch = sum(1 for a in ok if a.get("t_touch") is not None)
    print("\n" + "═" * 78)
    print(f"【G1 機制有沒有生效】hold_ring 列數合計 = "
          f"{sum(a['hold_ring_rows'] for _, _, a in rows)}   "
          f"(必須 **0**;非 0 表示 dive_from_runup 沒生效)")
    sg = [a["sig_rel"] for a in ok if a["sig_rel"] == a["sig_rel"]]
    rr = [a["R_rel"] for a in ok]
    print(f"【G2 放行工作點】σ 中位 {st.median(sg):.1f}°(基準 130.2°,目標 ≤30°);"
          f" R 中位 {st.median(rr):.0f} m(環 6084,基準 3417)")
    ra = [a["ratio"] for a in ok]
    print(f"【G3 ★決定性讀數★】航跡比中位 **{st.median(ra):.2f}**"
          f"(全距 {min(ra):.2f}–{max(ra):.2f});基準 14 架次 1.60–4.49、中位 2.33")
    print(f"     ⇒ {'✅ 通過(≤1.30)—— σ 是迂迴主因,值得往下做' if st.median(ra) <= 1.30 else '❌ 未過(>1.30)'}")
    if st.median(ra) > 1.30 and st.median(sg) <= 30.0:
        print("     ⚠️⚠️ σ 已經壓下去了但比值沒降 ⇒ 迂迴**不是**放行航向造成的,"
              "而是 PN 側向加速度飽和。\n"
              "        整條同步路線要停下來改查 PN,不要再往同步層加東西。")
    print(f"【G4 逐機真觸地率】**{n_touch}/{len(ok)}**"
          f"(嚴格判準 agl≤{TOUCH_AGL_M:.0f} m 且 d≤{TOUCH_D_M:.0f} m;"
          f"RRB 基準 7/14 = 50%,使用者要求 ≥90%)")
    fake = [a for a in ok if a.get("t_touch") is None and a.get("node_expended")]
    if fake:
        print(f"     ⚠️ 其中 {len(fake)} 架被節點記成命中但其實是高空掠過,"
              f"最近點高度 {[round(a['closest_agl']) for a in fake]} m")
    tt = [a["t_touch"] for a in ok if a.get("t_touch") is not None]
    if len(tt) >= 2:
        rng = max(tt) - min(tt)
        sig = st.pstdev(tt)
        print(f"【G5 散布】真觸地時刻全距 {rng:.1f} s、σ {sig:.2f} s(n={len(tt)})")
        print(f"     ⇒ 外推單群 8 架的期望全距 = d2(8)×σ = {D2[8] * sig:.1f} s"
              f"(門檻 3 s ⇒ 逐機 σ 必須 ≤ {3.0 / D2[8]:.2f} s)")
    else:
        print("【G5 散布】真觸地不足 2 架 ⇒ **散布在這一趟沒有定義**"
              "(這不是缺樣本,是無限大)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("trials", nargs="*")
    ap.add_argument("--glob", default=None)
    a = ap.parse_args()
    tr = list(a.trials)
    if a.glob:
        tr += sorted(p.name for p in ART.glob(a.glob) if p.is_dir())
    if not tr:
        ap.error("要給趟次名稱或 --glob")
    report(tr)
