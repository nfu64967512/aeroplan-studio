#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S 型時間同步(T3b)的航次判讀。

用法:
    python weave_readout.py runs/artifacts/WV1/tot_WV1.csv [更多趟次…]

═══════════════════════════════════════════════════════════════════════════
 這支在回答什麼
═══════════════════════════════════════════════════════════════════════════
⭐⭐ **主判準是「實得迂迴倍率」,不是指令角度。**

蛇形吸收時間的機制是把航跡拉長 1/cosθ 倍。但指令 θ 與實得 θ 之間有落差
(換向率 ω = g·tanφ/V 有限,而 LM 查證的文獻只給 |σ̇| ≤ σ̇_max 這個約束式,
**沒有**給落差的解析式 —— 那是我們自己要量的)。

⇒ 只看 `weave_yaw_deg` 非零就宣稱「蛇形有作用」是**假綠燈**:
   指令 45° 對應迂迴 1.41,實飛可能只有 1.02 ⇒ 吸收力差 20 倍,
   而 log 一切正常。所以本檔一律同時報:

       指令迂迴 = mean(1/cos θ_cmd)     ← 律以為自己在做什麼
       實得迂迴 = 航跡長 / 直線縮短量   ← 飛機實際做了什麼
       達成率  = (實得−1)/(指令−1)      ← 兩者的比

⚠️ 分母用「直線**縮短量**」(起點到終點的距目標距離差)而不是兩點直線距離:
   蛇形段的起終點都在朝目標飛,兩點直線距離會把「本來就要飛的那一段」
   算成迂迴的一部分,把達成率灌水。

═══════════════════════════════════════════════════════════════════════════
 已知的 schema 差異
═══════════════════════════════════════════════════════════════════════════
⚠️ `weave_early_rel_s` 欄是 2026-08-23 才加的。在那之前跑的趟次沒有這一欄,
   而且那些趟次的 `early_by_s` 是**兩個量的混合**
   (`_observe_timing` 寫絕對式、`_weave_theta_of` 寫相對式,同一格)。
   ⇒ 舊趟次一律不報 e_t,只報角度與迂迴。本檔會明說是哪一種。
"""
import csv
import math
import os
import sys
from collections import defaultdict

if os.name == "nt":                      # cp950 主控台會在 ⇒/⚠️ 上炸掉
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass

EARTH = 111320.0


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def dist_m(a, b):
    return math.hypot((b[0] - a[0]) * EARTH, (b[1] - a[1]) * mlon(a[0]))


def _f(v):
    """空字串 → None。⚠️ 不可以回 0.0:0 是**合法的**偏航角(沒在擺),
    拿它當「沒資料」會把「蛇形關著」與「蛇形開著但角度是 0」混為一談。"""
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path):
    rows = defaultdict(list)
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        rd = csv.DictReader(f)
        has_rel = "weave_early_rel_s" in (rd.fieldnames or [])
        for r in rd:
            i = _f(r.get("sysid"))
            t = _f(r.get("t"))
            if i is None or t is None:
                continue
            rows[int(i)].append(r)
    for i in rows:
        rows[i].sort(key=lambda r: float(r["t"]))
    return rows, has_rel


def track_len(seq):
    """實飛航跡長(m)。"""
    tot = 0.0
    for a, b in zip(seq, seq[1:]):
        tot += dist_m((float(a["lat"]), float(a["lon"])),
                      (float(b["lat"]), float(b["lon"])))
    return tot


def report(path):
    rows, has_rel = load(path)
    tag = os.path.basename(path)
    print(f"\n{'=' * 74}\n{tag}   ({len(rows)} 架)")
    if not has_rel:
        print("  ⚠️ 這一趟沒有 weave_early_rel_s 欄(2026-08-23 之前的 schema)"
              " ⇒ 不報 e_t:那時 early_by_s 是絕對式與相對式的混合值。")

    print(f"\n  {'機':>3} {'wait 段':>9} {'擺動占比':>9} {'θ中位':>7} {'θ最大':>7} "
          f"{'指令迂迴':>9} {'實得迂迴':>9} {'達成率':>7}")
    print("  " + "-" * 72)
    rel = {}
    for i in sorted(rows):
        seq = rows[i]
        wait = [r for r in seq if r.get("mode") == "wait"]
        if len(wait) < 3:
            print(f"  {i:>3}   —  沒有 wait 段(共 {len(seq)} 拍)")
            continue
        th = [_f(r.get("weave_yaw_deg")) for r in wait]
        th = [x for x in th if x is not None]
        on = [x for x in th if x > 0.05]
        dur = float(wait[-1]["t"]) - float(wait[0]["t"])
        # 直線**縮短量**:這一段本來就必須飛的距離
        d0 = _f(wait[0].get("d_slot"))
        d1 = _f(wait[-1].get("d_slot"))
        straight = None if (d0 is None or d1 is None) else abs(d0 - d1)
        flown = track_len(wait)
        got = (flown / straight) if straight and straight > 50.0 else None
        cmd = (sum(1.0 / math.cos(math.radians(min(x, 89.0))) for x in on) / len(on)
               if on else None)
        # 只在擺動占比夠高時才算達成率 —— 占比低時分母被沒擺的那段稀釋
        rate = None
        if cmd and got and cmd > 1.0005 and on and len(on) / len(th) > 0.3:
            rate = (got - 1.0) / (cmd - 1.0)
        thm = sorted(on)[len(on) // 2] if on else 0.0
        print(f"  {i:>3} {dur:>8.1f}s {len(on) / max(len(th), 1) * 100:>8.1f}% "
              f"{thm:>6.1f}° {max(th or [0]):>6.1f}° "
              f"{'—' if cmd is None else f'{cmd:>8.3f}'} "
              f"{'—' if got is None else f'{got:>8.3f}'} "
              f"{'—' if rate is None else f'{rate * 100:>6.0f}%'}")
        rel[i] = dict(dur=dur, on=len(on) / max(len(th), 1), cmd=cmd, got=got)

    # ── 放行時刻與觸地 ──
    print(f"\n  {'機':>3} {'放行 t':>9} {'觸地 t':>9} {'俯衝耗時':>9} "
          f"{'觸地地速':>9} {'最低AGL':>9}")
    print("  " + "-" * 60)
    rls, tds = {}, {}
    for i in sorted(rows):
        seq = rows[i]
        dive = [r for r in seq if r.get("mode") in ("dive", "impact")]
        if not dive:
            print(f"  {i:>3}   —  沒有進 dive")
            continue
        t0 = float(dive[0]["t"])
        t1 = float(dive[-1]["t"])
        agl = [_f(r.get("agl")) for r in dive]
        agl = [a for a in agl if a is not None]
        gs = _f(dive[-1].get("gs"))
        rls[i], tds[i] = t0, t1
        print(f"  {i:>3} {t0:>8.1f}s {t1:>8.1f}s {t1 - t0:>8.1f}s "
              f"{'—' if gs is None else f'{gs:>8.1f}'} "
              f"{'—' if not agl else f'{min(agl):>8.1f}'}")

    if len(rls) >= 2:
        dr = max(rls.values()) - min(rls.values())
        dt = max(tds.values()) - min(tds.values())
        dd = max(tds[i] - rls[i] for i in rls) - min(tds[i] - rls[i] for i in rls)
        print(f"\n  ⭐ 散布分解(memory/spread-decomposition):")
        print(f"     放行時刻差 {dr:>6.2f}s  +  俯衝耗時差 {dd:>6.2f}s "
              f"→  觸地散布 {dt:>6.2f}s")
        print(f"     ⇒ {'上游(放行)' if dr > dd else '下游(俯衝)'}是主導項")
    return rel


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    for p in sys.argv[1:]:
        if os.path.exists(p):
            report(p)
        else:
            print(f"⚠️ 找不到 {p}")
