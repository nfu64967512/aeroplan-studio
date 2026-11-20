#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nzc_readout.py — `nz_clamp_after_floor` 受控對照的判讀。

處理組 NZC(開關 True)vs 對照組 RRE(開關 False,已飛,15/16)。
對照組合法性:matlab/gammadiag/n1_identity.py 證實工作區的律與 RRE 當時的律
在 24102/24102 拍**逐位元相同**。

判準與功效分析在 docs/nzc_decision_rule.md,**資料到達前**定稿,這裡照抄不改。

⚠️⚠️ 本腳本的第一段是**機制生效證明**,而且它**擋住**後面的結果判讀。
   理由:[[release-sigma-has-no-power]] —— 拿一個看起來對的量當「生效證據」
   會產生假綠燈。這裡要的是直接證據:`cmd_nz` 這一欄。

   對照組 RRE(重放):cmd_nz 最低 −18.28、35.05% 的拍 < −1.0
   處理組 NZC 若生效:cmd_nz 最低 **恰好 −1.0**、原本越界的那些拍**貼在 −1.0**
   處理組若沒生效:cmd_nz 仍然跌到 −18 ⇒ 整份判讀作廢,不准往下讀
"""
from __future__ import annotations

import collections
import csv
import math
import os
import pathlib
import re
import statistics as st
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ART = pathlib.Path(os.environ.get(
    "ART", "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio/runs/artifacts"))
if not ART.exists():                       # Windows 端直接跑
    ART = HERE.parents[1] / "runs" / "artifacts"
SCORE = HERE / "stot_score.py"

HIT_M = 10.0            # 與 s2_readout.py:103 同一個判準
NZ_MIN = -1.0           # 契約值,也是本次夾制的下緣
EPS = 1e-3

# ── 對照組的既定事實(來自已飛的 RRE 與 32 架次重放)────────────────────
RRE_HITS, RRE_N = 15, 16
RRE_CMD_NZ_MIN = -18.2792
RRE_FRAC_BELOW = 0.3505
RRE_ERR_MED = 0.80
RRE_UNDERSHOOT = 1
RRE_SPREAD_S = 7.6


def fisher_2x2(a, b, c, d):
    """two-sided Fisher exact p."""
    from math import comb
    tot, r1, r2, c1 = a + b + c + d, a + b, c + d, a + c

    def p(x):
        return comb(r1, x) * comb(r2, c1 - x) / comb(tot, c1)

    p0 = p(a)
    return sum(p(x) for x in range(max(0, c1 - r2), min(r1, c1) + 1)
               if p(x) <= p0 + 1e-12)


def trials_of(prefix):
    out = []
    for p in ART.glob(f"{prefix}*"):
        if not p.is_dir():
            continue
        tail = p.name[len(prefix):]
        if tail.isdigit():
            out.append((int(tail), p.name))
    return [n for _, n in sorted(out)]


def rows_of(trial):
    f = ART / trial / f"tot_{trial}.csv"
    if not f.exists():
        return {}
    by = collections.defaultdict(list)
    with open(f, newline="", encoding="utf-8", errors="replace") as fh:
        for r in csv.DictReader(fh):
            by[r.get("sysid")].append(r)
    return by


def _f(r, k):
    try:
        return float(r.get(k, ""))
    except (TypeError, ValueError):
        return float("nan")


# ══════════════════════════════════════════════════════════════════════════
#  1) 機制生效證明(擋門)
# ══════════════════════════════════════════════════════════════════════════
def effect_proof(prefix):
    trials = trials_of(prefix)
    if not trials:
        return None, [f"✗ {ART} 底下找不到任何 {prefix}* 目錄"]
    ticks = at_floor = below = 0
    worst = float("inf")
    for t in trials:
        for _s, Q in rows_of(t).items():
            for r in Q:
                if r.get("mode") != "dive" or "cmd_nz" not in r:
                    continue
                v = _f(r, "cmd_nz")
                if v != v:
                    continue
                ticks += 1
                worst = min(worst, v)
                if abs(v - NZ_MIN) < EPS:
                    at_floor += 1
                elif v < NZ_MIN - EPS:
                    below += 1
    msgs = []
    if not ticks:
        msgs.append("✗ 俯衝段一拍都沒有 `cmd_nz` 欄 ⇒ **無法證明開關生效**。")
        msgs.append("  這不是「沒有越界」,是「沒有量到」—— 兩者長得一模一樣。")
        return None, msgs
    msgs.append(f"俯衝段有 cmd_nz 的拍數 : {ticks}")
    msgs.append(f"cmd_nz 最低            : {worst:.4f}"
                f"      (對照組 RRE 重放 {RRE_CMD_NZ_MIN:.4f})")
    msgs.append(f"貼在下緣 −1.0 的拍     : {at_floor} = {100 * at_floor / ticks:.2f}%"
                f"   (RRE 有 {100 * RRE_FRAC_BELOW:.2f}% 的拍越過 −1.0)")
    msgs.append(f"跌破 −1.0 的拍         : {below} = {100 * below / ticks:.3f}%")
    ok = True
    if below:
        ok = False
        msgs.append(f"✗ 有 {below} 拍跌破契約下緣 ⇒ 夾制**沒有**全程生效")
    if at_floor == 0:
        ok = False
        msgs.append("✗ 沒有任何一拍貼在下緣 ⇒ 夾制**從未咬過**,這一批等於沒動到"
                    "自變數(見 [[itcg-lateral-authority-ceiling]] 陷阱 #1)")
    if ok:
        msgs.append("✅ 夾制全程生效且確實咬住 ⇒ 可以往下讀結果")
    return (ticks, at_floor, below, worst), msgs


# ══════════════════════════════════════════════════════════════════════════
#  2) 結果
# ══════════════════════════════════════════════════════════════════════════
# ⚠️⚠️ `stot_score.py` 有**兩種**逐機列,而且第二種是判定的重點:
#
#   真落地 : `1   218.2   146.9   146.9     0.08    0.00 |     0.12   567.90`
#   未觸地 : `2   217.2     n/a   154.0  1589.76   14.61 |   未觸地        -`
#
# ⚠️ 先前這裡用一條「八個數字欄」的正則 ⇒ **未觸地那一列不match、被靜靜丟掉**
#    ⇒ 分母從 16 縮成 15、掠過的那架從報表上消失 = 命中率灌水。
#    RRE 正對照抓不到這個 bug,因為 RRE 十六架**全部**真的落地 ——
#    正對照沒有走到這條路徑。這正是 [[hit-criterion-ignores-altitude]] 那一條:
#    「全員命中」只看水平距,RD1 其實 6/6 高空掠過。
#
# ⇒ 現在改成:分成 `|` 左右兩半、逐欄容忍 `n/a` / `未觸地` / `-`,
#   而且**未觸地一律記成 miss,不是缺值**。
_NUM = re.compile(r"^-?[\d.]+$")


def _num(tok):
    return float(tok) if _NUM.match(tok) else None


def score(trial):
    csvf = ART / trial / f"tot_{trial}.csv"
    if not csvf.exists():
        return {}
    r = subprocess.run([sys.executable, str(SCORE), str(csvf)],
                       capture_output=True, text=True, encoding="utf-8",
                       errors="replace")
    out = {}
    for line in (r.stdout or "").splitlines():
        if "|" not in line:
            continue
        left, right = line.split("|", 1)
        lt, rt = left.split(), right.split()
        if len(lt) != 6 or not lt[0].isdigit() or len(rt) < 2:
            continue
        min_agl = _num(lt[4])
        if min_agl is None:
            continue
        err, tt = _num(rt[0]), _num(rt[1])
        out[lt[0]] = dict(min_agl=min_agl, err=err, t=tt,
                          near=_num(lt[5]), touched=err is not None)
    return out


def target_of(trial):
    f = ART / trial / f"tot_{trial}_geom.csv"
    if not f.exists():
        return None
    with open(f, newline="", encoding="utf-8", errors="replace") as fh:
        for r in csv.DictReader(fh):
            if r.get("kind") == "target":
                return float(r["lat"]), float(r["lon"])
    return None


def undershoot(tgt, Q):
    """打短判準([[undershoot-is-the-remaining-mode]]):過 r=2000 m 時 AGL<900。"""
    if tgt is None:
        return None
    tl, tn = tgt
    E = 111320.0
    prev = None
    for r in Q:
        if r.get("mode") != "dive":
            continue
        la, lo, ag = _f(r, "lat"), _f(r, "lon"), _f(r, "agl")
        if la != la or lo != lo:
            continue
        d = math.hypot((la - tl) * E,
                       (lo - tn) * E * math.cos(math.radians(la)))
        if prev is not None and prev > 2000.0 >= d:
            return ag < 900.0
        prev = d
    return None


def report(prefix):
    print("=" * 74)
    print(f"  NZC 判讀 — 處理組 {prefix}(nz_clamp_after_floor=True)"
          f" vs 對照組 RRE({RRE_HITS}/{RRE_N})")
    print("=" * 74)

    print("\n【1) 機制生效證明 — 這一段不過就不准讀結果】")
    proof, msgs = effect_proof(prefix)
    for m in msgs:
        print("   " + m)
    if proof is None or proof[2] or proof[1] == 0:
        print("\n⛔ 生效證明未過 ⇒ **停在這裡**。"
              "把沒生效的批次拿去比對結果,就是製造假綠燈。")
        return

    print("\n【2) 逐機結果】")
    print(f"   {'趟次':<9}{'UAV':>4}{'最低AGL':>10}{'觸地誤差':>11}"
          f"{'觸地t':>10}{'打短':>6}{'命中':>6}")
    hits = tot = us = no_touch = 0
    errs = []
    touch_t = collections.defaultdict(dict)
    missing = []
    for t in trials_of(prefix):
        sc, by, tgt = score(t), rows_of(t), target_of(t)
        # ⚠️ 分母以**實際飛了俯衝的機體**為準,不是以計分器吐得出幾列為準。
        flew = {s for s, Q in by.items()
                if any(r.get("mode") == "dive" for r in Q)}
        for s in sorted(flew, key=int):
            if s not in sc:
                missing.append(f"{t}-U{s}")
                tot += 1
                print(f"   {t:<9}{s:>4}{'—':>10}{'計分器無此列':>11}"
                      f"{'—':>10}{'—':>6}{'❌':>6}")
                continue
            a = sc[s]
            u = undershoot(tgt, by.get(s, []))
            # ⚠️ 未觸地 = miss。它不是缺值,是「從紅心上空飛過去了」。
            h = a["touched"] and a["err"] <= HIT_M
            tot += 1
            hits += bool(h)
            us += bool(u)
            if a["touched"]:
                errs.append(a["err"])
                touch_t[t][s] = a["t"]
            else:
                no_touch += 1
            e_s = ("%.2f" % a["err"]) if a["touched"] else "未觸地"
            t_s = ("%.1f" % a["t"]) if a["touched"] else "—"
            u_s = "是" if u else ("否" if u is not None else "—")
            print(f"   {t:<9}{s:>4}{a['min_agl']:10.1f}{e_s:>11}{t_s:>10}"
                  f"{u_s:>6}{('✅' if h else '❌'):>6}")
    if not tot:
        print("   (沒有可判讀的架次)")
        return
    if missing:
        print(f"\n   ⚠️ {len(missing)} 架飛了俯衝但計分器沒有輸出列:"
              f"{', '.join(missing)} ⇒ 一律記 miss,不從分母移除。")
    if no_touch:
        print(f"\n   ⚠️ **{no_touch}/{tot} 架未觸地(高空掠過)** —— "
              f"這是 miss,不是缺值([[hit-criterion-ignores-altitude]])。")

    print("\n【2b) 劑量反應 — 夾制**否決掉多少需求**,對上結果】")
    print("   ⭐ 這一段比命中率有用:`req_nz` 是夾制**前**的需求,`cmd_nz` 是送出去的。")
    print("      兩者的落差就是「這一架被夾制拿走了多少」。若失效只落在落差大的那些,")
    print("      就證明契約外的推頭**是在撐著結果的**,而不是可有可無的雜訊。")
    print(f"   {'趟次-UAV':<12}{'req_nz最低':>11}{'被否決':>9}"
          f"{'γ中位':>8}{'殘餘AGL':>9}{'結果':>6}")
    dose = []
    for t in trials_of(prefix):
        sc, by, tgt = score(t), rows_of(t), target_of(t)
        for s in sorted(sc, key=int):
            Q = [r for r in by.get(s, []) if r.get("mode") == "dive"]
            rn = [_f(r, "req_nz") for r in Q if "req_nz" in r]
            rn = [v for v in rn if v == v]
            gm = [_f(r, "gam_deg") for r in Q]
            gm = [v for v in gm if v == v]
            if not rn:
                continue
            a = sc[s]
            h = a["touched"] and a["err"] <= HIT_M
            denied = max(0.0, NZ_MIN - min(rn))     # 夾制吃掉的 g 數
            dose.append((f"{t}-U{s}", min(rn), denied,
                         st.median(gm) if gm else float("nan"),
                         a["min_agl"], h))
    for tag, rmin, den, gmed, agl, h in sorted(dose, key=lambda x: x[2]):
        print(f"   {tag:<12}{rmin:11.2f}{den:9.2f}{gmed:8.1f}{agl:9.0f}"
              f"{('✅' if h else '❌'):>6}")
    if len(dose) >= 4:
        den_ok = [x[2] for x in dose if x[5]]
        den_no = [x[2] for x in dose if not x[5]]
        if den_ok and den_no:
            print(f"\n   被否決的 g 數:命中組 中位 {st.median(den_ok):.2f}"
                  f"   失效組 中位 {st.median(den_no):.2f}")
            # ⚠️⚠️ **中位數比較會騙人**,不要拿它下結論:多數命中者的否決量
            #    恆為 0(夾制對它們惰性),中位自然是 0.00 —— 那反映的是
            #    「大部分架次沒被咬」,不是「被咬就會死」。
            #    唯一算數的是**重疊區**:大額否決卻乾淨命中的架次。
            clean_big = [x[0] for x in dose if x[2] > 5.0 and x[5]]
            small_bad = [x[0] for x in dose if 0.0 < x[2] <= 1.0 and not x[5]]
            if clean_big:
                print(f"   ⚠️ 否證條件 2 觸發:被否決 > 5 g 卻乾淨命中 —— "
                      f"{', '.join(clean_big)}")
            if small_bad:
                print(f"   ⚠️ 咬得很淺(≤1 g)卻失效 —— {', '.join(small_bad)}")
            if clean_big or small_bad:
                print("   ⇒ **單調劑量反應不成立**。夾制的深淺不是充分也不是必要條件,"
                      "\n      不可以宣稱「否決越多越會死」。真正的機制要另外找。")
            else:
                print("   ⇒ 重疊區是空的:大額否決全部失效、淺咬全部存活"
                      " ⇒ 劑量反應成立。")

    print("\n【3) 主判準 — 真觸地命中(誤差 ≤ 10 m)】")
    p = fisher_2x2(RRE_HITS, RRE_N - RRE_HITS, hits, tot - hits)
    print(f"   對照 RRE   {RRE_HITS}/{RRE_N} = {RRE_HITS / RRE_N:.4f}")
    print(f"   處理 {prefix:<6} {hits}/{tot} = {hits / tot:.4f}")
    print(f"   Fisher 兩尾 p = {p:.4f}   ⇒ "
          f"{'顯著差異' if p < 0.05 else '**統計上無法區分**'}")

    print("\n【4) 次判準(只報告,不當閘門)】")
    # ⚠️ 分母要印出來。誤差中位只涵蓋**有觸地**的架次 —— 沒觸地的沒有誤差可言,
    #    但它們正是最壞的那些 ⇒ 不寫分母就是把最壞的樣本靜靜藏起來。
    print(f"   觸地誤差 中位 {st.median(errs):8.2f} m   最大 {max(errs):9.2f} m"
          f"   (n={len(errs)}/{tot})   (RRE 中位 {RRE_ERR_MED:.2f} m, n=16/16)")
    # ⚠️⚠️ 打短計數在 n=32、事件數 1 的情況下**構造上零資訊**(Fisher p=1.000,
    #    虛無分布只有兩格各 0.5)。不可以拿 0/16 當改善。真正有力的是它二值化掉
    #    的連續量:過 r=2000 m 時的 AGL(RRE 中位 1314.6 / NZC 1550.6,p=2.4e-5)。
    print(f"   打短          {us:>8}/{tot}"
          f"                       (RRE {RRE_UNDERSHOOT}/{RRE_N};⚠️ 此計數零鑑別力)")

    # ── 散布:設限,不是缺值 ──────────────────────────────────────────
    # ⚠️⚠️ 先前這裡是 `if len(v) > 1` 直接跳過 ⇒ 沒觸地的那些趟**靜靜消失**,
    #    而它們正好是最壞的兩趟。實測:丟掉版中位 20.96 s(n=6),
    #    但那兩趟的散布是**右設限**、下界 ≥30.00 s 與 ≥47.60 s,兩個都超過 20.96
    #    ⇒ 丟掉的做法系統性低估處理組。設限感知後中位 ≥28.13 s、p 從 0.108 掉到 0.038。
    full, cens = [], []
    for t, v in touch_t.items():
        n_flew = len([1 for s, Q in rows_of(t).items()
                      if any(r.get("mode") == "dive" for r in Q)])
        if len(v) >= n_flew and len(v) > 1:
            full.append(max(v.values()) - min(v.values()))
        elif v:
            # 有機體沒觸地 ⇒ 散布至少是「最後一筆記錄時刻 − 已觸地的最早時刻」
            last = max(_f(r, "t") for _s, Q in rows_of(t).items() for r in Q
                       if _f(r, "t") == _f(r, "t"))
            cens.append(last - min(v.values()))
    if full or cens:
        allv = full + cens
        print(f"   逐趟觸地散布 中位 {st.median(allv):.2f} s   最大 {max(allv):.2f} s"
              f"   (RRE {RRE_SPREAD_S} s,門檻 3 s)")
        if cens:
            print(f"   ⚠️ 其中 {len(cens)} 趟是**右設限**(有機體未觸地),下界 "
                  f"{', '.join(f'≥{c:.2f}s' for c in sorted(cens))}"
                  f" ⇒ 這個中位是**下界**,真值更差。")
            print(f"      只算完整趟次會得到 {st.median(full):.2f} s(n={len(full)})"
                  f" —— 那會把最壞的兩趟靜靜刪掉。")

    print("\n【5) 判定(照 docs/nzc_decision_rule.md,不事後調整)】")
    # ⚠️ 判準是對 **16 架次** 定的。批次沒跑完就套用等於偷看期中結果,
    #    而期中結果的分母小、Fisher 檢定力更低 ⇒ 只會給出一個看起來很確定
    #    的錯結論。跑完之前只報進度。
    if tot < RRE_N:
        print(f"   ⏳ 只有 {tot}/{RRE_N} 架次,**判準尚不適用**。")
        print("      判準是對 16 架次定的;期中就套用等於偷看,不做。")
        return
    if hits >= 10:
        print(f"   {hits}/{tot} ≥ 10 ⇒ **未觀察到災難性衰退**。")
        print("   ⚠️ 這**不等於**等效:功效分析說 14/16 與 15/16 的 p = 1.000,")
        print("      16 架次區分不出來。可以把系統收進契約內,")
        print("      但不可以宣稱「沒有代價」。")
        print("   下一步:pn_nz_min −1.0 → −2.0 的第二個單變數實驗。")
    else:
        print(f"   {hits}/{tot} ≤ 9 ⇒ **契約外的無上界推頭是在撐著結果的**。")
        print("   ⇒ 不採用這顆開關;改成把契約放寬到符合實測,")
        print("      並把理由寫進 contract_table 的 evidence 欄。")
        print("      誠實宣告我們需要 −18 g,好過假裝 −1 g 夠用。")


if __name__ == "__main__":
    if os.name == "nt":
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    report(sys.argv[1] if len(sys.argv) > 1 else "NZC")
