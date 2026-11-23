#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""編隊巡航段判讀:雁型出來了嗎?巡航到 157 了嗎?

    python3 formation_readout.py runs/artifacts/FLOG1/formation

═══════════════════════════════════════════════════════════════════════════
 這支要回答的兩個問題,以及它們各自的陷阱
═══════════════════════════════════════════════════════════════════════════
① **巡航地速 157** —— 指令是 EAS,幾何是地速,3100 m 差 1.167 倍。
   ⚠️ 只看實得地速看不出「指令被飛控整發拒收」(飛機仍在飛,只是吃著
      別的速度)。判準是 `gs_err = 實得 − 指令換算的地速`。

② **雁型間隔 2.5 km** —— 有兩個不同的量,都要看:
     `d_slot`     僚機**跟不跟得上** slot
     `sep_along/cross`  對長機的**實際**相對位置 = 雁型有沒有真的出來
   ⚠️ 只看前者的話,一個「完美跟上一個錯的 slot」的航次看起來是滿分。
      而 2026-08-24 之前 slot 方位就是寫死正後方的。

⚠️⚠️ **恆空的欄位會被讀成「量到零」。** 本檔在報任何數字之前先印填充率,
   一欄全空就明說「沒量到」而不是報 0([[instrument-that-cannot-fire]])。
"""
import csv
import glob
import io
import math
import os
import statistics as st
import sys

sys.stdout.reconfigure(encoding="utf-8")


def num(x):
    try:
        v = float(x)
        return v if v == v else None          # NaN -> None
    except (TypeError, ValueError):
        return None


def med(xs):
    xs = [x for x in xs if x is not None]
    return st.median(xs) if xs else None


def fmt(v, unit="", nd=1):
    return "—" if v is None else ("%.*f%s" % (nd, v, unit))


def load(path):
    with io.open(path, encoding="utf-8", errors="replace", newline="") as fh:
        return list(csv.DictReader(fh))


def report(d):
    files = sorted(glob.glob(os.path.join(d, "formation_*.csv")))
    if not files:
        print("找不到 %s/formation_*.csv" % d)
        print("⚠️ 這**不是**「編隊沒問題」—— 是這一段沒有留下任何資料。")
        return 1
    for p in files:
        rows = load(p)
        role = os.path.basename(p).split("_")[1]
        print("═" * 74)
        print("%s  (%d 列)" % (os.path.basename(p), len(rows)))
        if not rows:
            print("  ⚠️⚠️ 檔案是空的 —— 記錄器開了但一列都沒寫。")
            continue

        # ── 儀表自檢:先講哪些欄位根本沒資料 ──────────────────────────
        cols = [c for c in rows[0] if c not in ("role",)]
        dead = [c for c in cols
                if all((r.get(c) or "") == "" for r in rows)]
        if dead:
            print("  ⚠️⚠️ **整欄空白**(下游會讀成「量到零」):%s"
                  % ", ".join(dead))
        else:
            print("  ✅ 沒有整欄空白的欄位")

        # ⭐⭐⭐ **先切時間段，不要把一個檔當作一趢航次。**
        #
        # FLOG1 實測：一個 CSV 跨 **7265 s**（2 小時），中間有一個
        # **660 s 的空隙恰好涵蓋整趢打擊**（編隊層讓權給 tot_node）。
        # 而空隙兩側的列看起來完全一樣 —— 我自己把空隙**之後**
        # 兩具已 disarm 殘骸的數字（cmd 90 / gs 25.9 / worst_err 11954）
        # 讀成「編隊在發散」，並且報告了出去。
        # ⇒ 空隙是一個**看不見的訊號**；不主動指出來的話，
        #   任何統計量都會把不同階段的資料混在一起。
        ts = [num(r.get("t")) for r in rows]
        ts = [t for t in ts if t is not None]
        if len(ts) > 2:
            gaps = [(ts[i + 1] - ts[i], ts[i] - ts[0])
                    for i in range(len(ts) - 1) if ts[i + 1] - ts[i] > 30.0]
            print("  總跨度 %.0f s" % (ts[-1] - ts[0]))
            if gaps:
                print("  ⚠️⚠️ 有 %d 個 >30 s 的空隙 ⇒ **這不是一段連續的資料**:"
                      % len(gaps))
                for g, w in sorted(gaps, reverse=True)[:3]:
                    print("       %.0f s 的空隙 @+%.0f s" % (g, w))
                print("       編隊層在打擊期間會**讓權**給 tot_node 而停寫 ——")
                print("       空隙之後的列很可能是**航次結束後**的殘骸資料。")
                print("       ⇒ 下面的統計只取**第一段**（空隙之前）。")
                cut = ts[0] + sorted(gaps, key=lambda g: g[1])[0][1] + 0.001
                rows = [r for r in rows
                        if (num(r.get("t")) or 0) <= cut]

        # 讓權邊界標記（節點主動寫的）
        marks = [r for r in rows if r.get("phase") in ("YIELD_TO_TOT", "RESUMED")]
        if marks:
            print("  ⭐ 讓權標記:%s"
                  % ", ".join("%s@+%.0fs" % (r["phase"], num(r["t"]) - ts[0])
                              for r in marks[:4]))

        ph = {}
        for r in rows:
            ph[r.get("phase") or "?"] = ph.get(r.get("phase") or "?", 0) + 1
        print("  相位:%s" % ", ".join("%s %d" % kv for kv in sorted(ph.items())))

        cruise = [r for r in rows if r.get("phase") == "CRUISE"]
        seg = cruise if cruise else rows
        tag = "巡航段" if cruise else "全段(沒有 CRUISE 拍)"

        # ── ① 速度 ──────────────────────────────────────────────────
        cmd = med([num(r.get("cmd_speed_eas")) for r in seg])
        cgs = med([num(r.get("cmd_gs_equiv")) for r in seg])
        gs = med([num(r.get("gs")) for r in seg])
        err = [num(r.get("gs_err")) for r in seg]
        err = [e for e in err if e is not None]
        print("\n  ── ① 速度(%s)──" % tag)
        print("     指令 EAS        %s" % fmt(cmd))
        print("     指令換算地速    %s   ← 「本來應該飛多快」" % fmt(cgs))
        print("     實得地速        %s" % fmt(gs))
        # ⚠️⚠️ **飛機還在地上時不可以宣布「指令沒生效」。**
        #    FLOG1 實測：航次在起飛前就被契約閘門中止，所以
        #    gs 恒為 0、alt 恒為 0 —— 而我的第一版據此報「100% 的拍
        #    偏離指令 >15 m/s ⇒ 指令很可能沒生效」。那是**假陽性**，
        #    而且它指向的刚好是一個真實存在的失效，最容易被信。
        #    ⇒ 先問「這些拍里飛機有在飛嗎」再談指令有沒有生效。
        # ⚠️⚠️⚠️ **逐拍**判斷飛機有沒有在飛，不是拿整段的 max。
        #    第一版寫 `airborne = max(gs) > 20`，而飛機在這一段**末尾**
        #    才開始滑行 ⇒ max 跨過 20、整段被判成「在飛」，
        #    於是地面上的 gs=0 又被拿去算 gs_err，又報一次
        #    「100% 的拍指令沒生效」。同一個假陽性在同一支工具裡
        #    用兩種寫法各發作一次。
        FLY_GS = 20.0
        n_fly = sum(1 for r in seg if (num(r.get("gs")) or 0.0) > FLY_GS)
        err = [num(r.get("gs_err")) for r in seg
               if (num(r.get("gs")) or 0.0) > FLY_GS]
        err = [e for e in err if e is not None]
        print("     在飛的拍        %d/%d (%.0f%%)   ← 下面只算這些拍"
              % (n_fly, len(seg), 100.0 * n_fly / max(len(seg), 1)))
        if n_fly < 20:
            print("     ⚠️ 在飛的拍太少 ⇒ **不宣布「指令有沒有生效」**。")
            print("        地面上 gs=0 是**對的**，拿它算偏差只會得到假陽性。")
            err = []
        # ── ② 幾何(只有僚機/備長有)────────────────────────────────
        sep = [num(r.get("sep_m")) for r in seg]
        if any(s is not None for s in sep):
            along = med([num(r.get("sep_along")) for r in seg])
            cross = med([num(r.get("sep_cross")) for r in seg])
            dsl = med([num(r.get("d_slot")) for r in seg])
            brg = med([num(r.get("tail_brg")) for r in seg])
            eff = med([num(r.get("eff_angle")) for r in seg])
            print("\n  ── ② 幾何(%s)──" % tag)
            print("     對長機實際間距  %s m" % fmt(med(sep), nd=0))
            print("       沿航向        %s m   (負 = 在後方)" % fmt(along, nd=0))
            print("       垂航向        %s m   (正 = 在左)" % fmt(cross, nd=0))
            print("     對 slot 的誤差  %s m   ← 跟不跟得上(與上面是兩件事)"
                  % fmt(dsl, nd=0))
            print("     指令方位偏置    %s°  (180 = 正後方)" % fmt(brg))
            print("     展開後偏角      %s°" % fmt(eff))
            if brg is not None and abs(brg - 180.0) < 0.5:
                print("     ⚠️⚠️ 指令方位是**正後方** ⇒ 這是縱列,不是雁型。")
            elif eff and cross is not None and along is not None:
                # 實得偏角 = atan2(|cross|, |along|)
                got = math.degrees(math.atan2(abs(cross), abs(along)))
                print("     ⭐ 實得偏角      %.1f°  ⇒ 達成率 %.2f"
                      % (got, got / eff if eff else float("nan")))

        # ── ③ 成形判定 ──────────────────────────────────────────────
        fm = [r.get("formed") for r in seg if (r.get("formed") or "") != ""]
        if fm:
            wo = [num(r.get("worst_err")) for r in seg]
            wo = [w for w in wo if w is not None]
            frac = sum(1 for f in fm if f == "1") / len(fm)
            print("\n  ── ③ 成形判定 ──")
            print("     formed=1 的拍  %.0f%%" % (100 * frac))
            print("     worst_err 中位 %s m" % fmt(med(wo), nd=0))
            if frac > 0.9 and wo and med(wo) is not None and med(wo) > 400:
                print("     ⚠️⚠️ 幾乎恆為「已成形」而 worst 很大 ⇒ **判準沒有鑑別力**")
                print("        (formed_radius >= 間距時,容差球會把長機本身包進去)")
    return 0


if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    sys.exit(report(d))
