#!/usr/bin/env python3
"""stot_rescore_all.py — 用**現在**的 stot_score.py 重算 runs/ 裡的每一趟。

為什麼需要這支:
  `stot_repeat.csv` 是各趟當下寫進去的,而計分器後來修過好幾次。最嚴重的一次是
  「取第一個地面穿越點」→「取離紅心最近的那個穿越點」(commit e8b775d):在那之前
  寫進總表的列,只要飛機在遠處先掠過一次較高地形,就會被記成幾百甚至幾千公尺的
  誤差。實測 T1 記成 2083.59 m、B21 記成 2554.75 m,重算後其實是 0.39 m 和 2.71 m
  —— 兩趟好航次被當成失敗,而且被寫進了成功率。
  另外 v1 的總表沒有速度欄,而速度是 2026-07-28 之後的第一優先。

  ⚠️ 所以:**引用歷史成功率之前先跑這支**,不要直接讀 stot_repeat.csv 的舊列。

⚠️⚠️ 計分失敗的航次**留在總表裡**(2026-08-05 blocker)。上一版是
  `except Exception: return None` + 呼叫端 `if s is None: continue`,把該趟整個
  從總表移除;而被移除的一律是「CSV 壞掉 / 空檔 / 逾時報廢 / 目標座標找不到」
  這一類**失敗**航次。批次統計 `sum(1 for _, ok in g if ok), len(g)` 的分母
  只算得分成功的那些 ⇒ 成功率被系統性**灌高**,方向永遠樂觀;結尾那句
  「重算 N 趟」印的也是存活數,讀者看不出少了誰,連「有東西不見了」都看不出來。
  這支正是「引用歷史成功率之前先跑」的工具,它報出來的比例會直接進交接文檔;
  128-146 行的「速度紀錄」也是從同一份 rows 挑的。
  現在:失敗列照樣佔一行(pass=0 + 失敗原因),分母含失敗列,結尾明確印
  「N 趟成功計分 / M 趟計分失敗(原因分類)」,--csv 另存的總表多一個 reason 欄。

用法:
  python3 stot_rescore_all.py                 # 全部,依批次彙總
  python3 stot_rescore_all.py --csv out.csv   # 另存成一份乾淨的總表
  python3 stot_rescore_all.py --batch K L N   # 只看這幾個批次
"""
import argparse
import ast
import collections
import csv
import json
import os
import re
import subprocess
import sys

RUNS = os.path.expanduser("~/fixwing_ws/runs")
HERE = os.path.dirname(os.path.abspath(__file__))


# ── 計分失敗的分類 ────────────────────────────────────────────────────────
# ⚠️ 原因要**分類**而不是只丟一句 traceback:結尾要按類別統計,讀者才分得出
#    「少掉的 12 趟是同一個原因(CSV 全空)」還是「12 個各自不同的偶發錯」。
#    每一類都對應一種實際遇過的失敗:
#      no-csv        tot_<trial>.csv 不見了(只剩 df_/show_)
#      no-target     *_geom.csv 不在或沒有 target 列 ⇒ stot_score 回 2
#      no-phase-data CSV 在,但一列 DIVE 都沒有(航次在 DIVE 之前就報廢)
#      score-failed  stot_score 非零離開,但不是上面兩種
#      bad-json      stdout 不是合法 JSON(被 kill -KILL 截斷、空檔最常見)
#      version-bad   是合法 JSON dict,但缺下游會下標的欄位 ⇒ **計分器版本不符**
#      timeout       計分逾時(SW1 那種 130 MB 的 CSV)
#      spawn-fail    連子行程都起不來
R_NO_CSV = "no-csv"
R_NO_TARGET = "no-target"
R_NO_PHASE = "no-phase-data"
R_SCORE_FAILED = "score-failed"
R_BAD_JSON = "bad-json"
R_VERSION_BAD = "version-mismatch"
R_TIMEOUT = "timeout"
R_SPAWN = "spawn-fail"


def _required_summary_keys(path=None):
    """把「下游一定會下標的 summary 欄位」**從本檔原始碼推導出來**。

    ⚠️⚠️ 2026-08-05 blocker:score() 的守衛以前只檢查 `"n_touch" not in s`,
       但 verdict() 另外下標 touch_max_miss_m / touch_spread_s / max_abs_alt_m /
       touch_gs_mean_ms,表格與 --csv 又再下標 n / touch_gs_min_ms /
       touch_gs_max_ms —— **六分之一的覆蓋率**。docstring 自己寫著「半個 JSON
       混進總表,後面每一個取欄位的地方都會變成 KeyError」,守衛卻放它進來。
       實際踩得到:HERE 目錄裡的 stot_score.py 若是 2026-07-28 之前的版本
       (沒有速度欄),第一趟就 KeyError 炸掉**整份**總表 —— 那是最極端的
       「資料全部消失」,連一列都印不出來。

    ⚠️ 為什麼用 AST 推導而不是手抄一份清單:手抄的清單就是這次漏掉五個鍵的
       原因。下次誰在表格裡多印一欄 `s["新欄位"]`,守衛會**自動**跟著涵蓋;
       抄的話只會再漏一次,而且一樣要等到炸掉才知道。

    ⚠️ 只收 `x["字面字串"]` 這種**下標**;`s.get("verdict")` 刻意不算 ——
       缺 verdict 走的是另一條路(verdict() 判不通過 + 吼版本不符,航次留在
       總表),不是把整趟打成計分失敗。兩者都不許靜默,但形狀不同。
    """
    src = open(path or os.path.abspath(__file__), encoding="utf-8").read()
    keys = set()
    for node in ast.walk(ast.parse(src)):
        if not isinstance(node, ast.Subscript):
            continue
        sl = node.slice
        if isinstance(sl, ast.Index):           # py<3.9
            sl = sl.value
        if isinstance(sl, ast.Constant) and isinstance(sl.value, str):
            keys.add(sl.value)
    # 錨點:推導壞掉(語法變動、被打包成 zipapp 讀不到原始碼)必須**吼**,
    # 不可以安靜地退化成空集合 —— 空集合等於守衛整個消失。
    if "n_touch" not in keys:
        raise AssertionError(
            "從 %s 推導不出 summary 欄位清單(連 n_touch 都沒有)—— 守衛失效,"
            "不要讓它安靜地放行" % (path or os.path.abspath(__file__)))
    return keys


REQUIRED_SUMMARY_KEYS = _required_summary_keys()


def _tail(s, n=140):
    s = " ".join((s or "").split())
    return s if len(s) <= n else "…" + s[-n:]


def score(trial, speed_min, runs=None):
    """跑一次 stot_score.py --json。

    回傳 **(summary, None)** 或 **(None, (分類, 細節))** —— 兩者剛好互斥。

    ⚠️⚠️ 2026-08-05 blocker:這裡以前是 `except Exception: return None`,把逾時、
       非零離開、JSON 壞掉、檔案不見全部壓成同一個 None,呼叫端再 `continue`
       掉整趟。失敗的形狀因此完全看不見,而且**只有失敗會消失** ⇒ 成功率灌高,
       方向永遠樂觀。現在每一種失敗都帶著自己的分類與細節回去,由呼叫端把它
       印成一列 pass=0 —— 這支不吞任何一種失敗。
    ⚠️ 也不要在這裡 raise:一趟壞檔不該讓整份總表算不出來(那是另一種資料消失)。
    """
    runs = RUNS if runs is None else runs
    csvp = os.path.join(runs, "tot_%s.csv" % trial)
    logp = os.path.join(runs, "trial_%s.log" % trial)
    if not os.path.exists(csvp):
        return None, (R_NO_CSV, "找不到 %s" % csvp)
    cmd = [sys.executable, os.path.join(HERE, "stot_score.py"), csvp]
    if os.path.exists(logp):
        cmd += ["--log", logp]
    cmd += ["--speed-min", str(speed_min), "--json"]
    try:
        # ⚠️⚠️ encoding 一定要**明講**。text=True 不指定時走的是主機 locale,
        #    在中文 Windows 上是 cp950,而 stot_score.py 的輸出是 UTF-8 中文
        #    ⇒ subprocess 的 reader thread 拋 UnicodeDecodeError 而死,
        #    `p.stdout` 變成 **None**(不是空字串、也不是例外),
        #    然後 json.loads(None) 才炸,錯誤訊息指向 JSON 而不是編碼 ——
        #    在同一台機器上「從 Git Bash 跑會過、從 PowerShell 跑會紅」。
        #    errors="replace" 保證壞位元組不會再讓整份總表算不出來。
        p = subprocess.run(cmd, capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=180)
    except subprocess.TimeoutExpired:
        return None, (R_TIMEOUT, "stot_score.py 超過 180 s 沒跑完")
    except Exception as e:                      # OSError:python 不見了之類
        return None, (R_SPAWN, "%s: %s" % (type(e).__name__, e))
    if p.returncode != 0:
        err = _tail(p.stderr)
        if "找不到目標座標" in (p.stderr or ""):
            return None, (R_NO_TARGET, err)
        if "階段的資料" in (p.stderr or ""):
            return None, (R_NO_PHASE, err)
        return None, (R_SCORE_FAILED, "rc=%d %s" % (p.returncode, err))
    try:
        s = json.loads(p.stdout)
    except ValueError as e:
        return None, (R_BAD_JSON, "%s(stdout %d B)" % (e, len(p.stdout or "")))
    # ⚠️ 只要不是「能拿來判定的 summary」就算失敗 —— 半個 JSON 混進總表,後面
    #    每一個取欄位的地方都會變成 KeyError 或更糟的靜默 None。
    if not isinstance(s, dict):
        return None, (R_BAD_JSON,
                      "JSON 不是物件而是 %s" % type(s).__name__)
    # ⚠️⚠️ 守衛要涵蓋**下游每一個會下標的鍵**,不是只有 n_touch(舊版只擋
    #    六分之一)。清單由 _required_summary_keys() 從本檔原始碼推導,新增欄位
    #    會自動被涵蓋。缺鍵幾乎只有一個成因:部署版 stot_score.py 太舊
    #    (2026-07-28 之前沒有速度欄)⇒ 歸類成「版本不符」而不是「壞 JSON」,
    #    因為修法完全不同:一個是重新部署工具,一個是這趟資料真的壞了。
    missing = sorted(k for k in REQUIRED_SUMMARY_KEYS if k not in s)
    if missing:
        return None, (R_VERSION_BAD,
                      "JSON 缺 %d 個下游要用的欄位:%s(部署版 %s 太舊?"
                      "2026-07-28 之前的版本沒有速度欄)"
                      % (len(missing), "、".join(missing),
                         os.path.join(HERE, "stot_score.py")))
    return s, None


def verdict(s, speed_min, alt_max, trial=None):
    """和 stot_score.py 的 `ok` 同一套判定,集中在這裡免得兩邊漂移。

    ⚠️⚠️ 2026-08-04 補上「首航次全員命中」。stot_score:648 加了 first_pass_ok
       之後這裡沒有同步,結果是同一份 JSON、兩支工具給出**不同**結論:
       一趟「兩架脫靶飛到 200 km 外、幾小時後隨機重穿紅心」的航次在觸地基準上
       看起來很正常,stot_score 判 ❌ 而總表判 ✅。而總表才是拿來排序、
       挑「最好的一趟」的那一份。

    ⚠️⚠️ 2026-08-05 blocker C:**缺 verdict 區塊一律 fail-closed。**
       舊版寫的是 `True if not v else …`,理由是「--no-window 產的舊 JSON 沒有
       verdict,跟工具端一樣回 True」。這條理由現在兩邊都不成立:
         ① score() **從來不傳 --no-window**(見上面的 cmd),所以「沒有 verdict」
            在這裡的唯一成因是**計分器版本不符** —— 部署版 stot_score.py 沒跟
            repo 同步(實測 ~/fixwing_ws/sitl_run/stot_score.py 只有 15,140 B,
            grep 不到 verdict/window/all_hit,但仍接受 --json 且回 0)。
            「工具太舊、量不出這一項」的正確結論是**不予通過並吼一聲**,
            不是預設通過 —— 否則首航次閘門在版本不符時整個消失,而總表正是
            拿來排序、挑「最好的一趟」的依據。
         ② stot_score.py 那一處也同步改成 fail-closed(--no-window 不予通過)。
       三處保持逐字同一個結論:有 verdict ⇒ 看 all_hit;沒有 ⇒ False + 吼。

    ⚠️ 計分**失敗**的航次根本走不到這裡(沒有 summary 可判):它一律算不通過
       (pass=0)、帶著失敗原因留在總表,而不是從總表消失 —— 見 score()。
    """
    v = s.get("verdict")
    if isinstance(v, dict):
        first_pass_ok = bool(v.get("all_hit"))
    else:
        first_pass_ok = False
        _shout_version_mismatch(trial)
    return (s["n_touch"] == 6
            and s["touch_max_miss_m"] is not None and s["touch_max_miss_m"] <= 5.0
            and s["touch_spread_s"] <= 3.0
            and s["max_abs_alt_m"] is not None and s["max_abs_alt_m"] <= alt_max
            and s["touch_gs_mean_ms"] is not None
            and s["touch_gs_mean_ms"] >= speed_min
            and first_pass_ok)


#: 版本不符只吼一次全文,之後每趟只補一個航次編號 —— 掃 100 趟不會被洗版,
#: 但也絕不靜默(靜默失敗是本專案的頭號死因)。
_MISMATCH = []


def _shout_version_mismatch(trial):
    _MISMATCH.append(trial)
    if len(_MISMATCH) == 1:
        print("⚠️⚠️ JSON 沒有 verdict 區塊 ⇒ 首航次判定**量不出來**。"
              "本工具從不傳 --no-window,所以唯一成因是 %s 版本不符"
              "(舊版沒有首航次窗)。這些航次一律判**不通過**(未判定 ≠ 通過)"
              " —— 請把 repo 版 stot_score.py 部署過去再重算。"
              % os.path.join(HERE, "stot_score.py"), file=sys.stderr)
    print("⚠️ 缺 verdict(判不通過):%s" % trial, file=sys.stderr)


def _batch_of(trial):
    """批次前綴。⚠️ 不要直接 .group(0):數字開頭的 trial 會 AttributeError,
    整份總表跟著死掉 —— 那也是一種「資料全部消失」。"""
    m = re.match(r"[A-Za-z]+", trial)
    return m.group(0) if m else "?"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed-min", type=float, default=100.0)
    ap.add_argument("--alt-max", type=float, default=30.0)
    ap.add_argument("--batch", nargs="*", default=None,
                    help="只算這些批次前綴(例:--batch K L N)")
    ap.add_argument("--csv", default=None, help="另存一份重算後的總表")
    ap.add_argument("--runs", default=RUNS,
                    help="航次目錄(預設 ~/fixwing_ws/runs)")
    a = ap.parse_args(argv)

    if not os.path.isdir(a.runs):
        print("找不到航次目錄 %s" % a.runs, file=sys.stderr)
        return 2
    trials = sorted(
        m.group(1) for m in
        (re.match(r"tot_([A-Za-z0-9]+)\.csv$", f) for f in os.listdir(a.runs))
        if m)
    if a.batch:
        pref = tuple(a.batch)
        trials = [t for t in trials if t.startswith(pref)]

    # rows 的每一列是 (trial, summary|None, ok, reason|None)。
    # ⚠️⚠️ 計分失敗的列 summary=None、ok=False、reason=(分類, 細節),但它**還在
    #    rows 裡**。上一版是 `if s is None: continue`,於是失敗航次從表、從分母、
    #    從結尾那句「重算 N 趟」裡同時消失,只剩成功的那些去算成功率。
    rows = []
    for t in trials:
        s, why = score(t, a.speed_min, a.runs)
        ok = False if s is None else verdict(s, a.speed_min, a.alt_max, trial=t)
        rows.append((t, s, ok, why))

    print("%-6s %5s %8s %8s %9s %9s %8s %6s" %
          ("trial", "觸地", "地速平均", "最慢", "誤差(m)", "AGL(m)",
           "散布(s)", "判定"))
    for t, s, ok, why in rows:
        if s is None:
            # ⚠️ 失敗列照樣佔一行,原因印在最右邊 —— 讀者要看得出「這一趟不是
            #    沒跑,是**算不出來**」,而不是讓它從畫面上消失。
            print("%-6s %5s %8s %8s %9s %9s %8s %6s  計分失敗:%s(%s)"
                  % (t, "-", "-", "-", "-", "-", "-", "❌", why[0], why[1]))
            continue
        print("%-6s %5s %8s %8s %9s %9s %8s %6s" % (
            t, "%d/%d" % (s["n_touch"], s["n"]),
            "-" if s["touch_gs_mean_ms"] is None else "%.1f" % s["touch_gs_mean_ms"],
            "-" if s["touch_gs_min_ms"] is None else "%.1f" % s["touch_gs_min_ms"],
            "-" if s["touch_max_miss_m"] is None else "%.2f" % s["touch_max_miss_m"],
            "-" if s["max_abs_alt_m"] is None else "%.2f" % s["max_abs_alt_m"],
            "%.2f" % s["touch_spread_s"],
            "✅" if ok else "❌"))

    print()
    print("%-6s %4s %8s %8s %12s %12s" %
          ("批次", "n", "全項過", "計分失敗", "地速平均", "散布 中位"))
    by = collections.defaultdict(list)
    for t, s, ok, why in rows:
        by[_batch_of(t)].append((s, ok))

    def med(xs):
        xs = sorted(x for x in xs if x is not None)
        return None if not xs else xs[len(xs) // 2]

    for b in sorted(by):
        g = by[b]
        # ⚠️⚠️ 分母 len(g) 含**計分失敗**的列。上一版的分母只有得分成功的那些,
        #    而消失的一律是失敗航次 ⇒ 成功率系統性灌高。中位數只能用成功的列算
        #    (失敗列沒有數字),但那**不影響分母**——「算不出來」不等於「不存在」。
        n_fail = sum(1 for s, _ in g if s is None)
        gm = [s["touch_gs_mean_ms"] for s, _ in g if s is not None]
        sp = [s["touch_spread_s"] for s, _ in g if s is not None]
        print("%-6s %4d %8s %8s %12s %12s" % (
            b, len(g), "%d/%d" % (sum(1 for _, ok in g if ok), len(g)),
            "-" if not n_fail else "%d" % n_fail,
            "-" if med(gm) is None else "%.1f" % med(gm),
            "-" if med(sp) is None else "%.2f" % med(sp)))

    npass = sum(1 for _, _, ok, _ in rows if ok)
    failed = [(t, why) for t, s, _ok, why in rows if s is None]
    print("\n重算 %d 趟:%d 趟成功計分、%d 趟**計分失敗**;"
          "以「地速平均 ≥ %.0f、散布 ≤3 s、誤差 ≤5 m、"
          "AGL ≤%.0f m、六機到齊、首航次全員命中」全項判定:%d/%d 趟通過"
          "(分母含那 %d 趟計分失敗)。"
          % (len(rows), len(rows) - len(failed), len(failed),
             a.speed_min, a.alt_max, npass, len(rows), len(failed)))
    if failed:
        # ⚠️ 原因要分類印出來:少掉的那幾趟是不是同一個病,決定了要回頭修資料
        #    還是修飛控;而「有幾趟算不出來」本身就是一個結論。
        kinds = collections.Counter(why[0] for _t, why in failed)
        print("   計分失敗分類:%s"
              % "、".join("%s×%d" % (k, n) for k, n in sorted(kinds.items())))
        for t, why in failed:
            print("     %-6s %-14s %s" % (t, why[0], why[1]))
        print("   ⚠️ 這 %d 趟一律記**不通過**(pass=0)並留在分母 —— 被移除的"
              "一律是失敗航次,分母少掉它們就是把成功率灌高。" % len(failed))
        # ⚠️⚠️ 版本不符要單獨再講一次:它**不是航次結果**,而且修法是「重新部署
        #    計分器再重算」,跟「這趟資料壞了」完全不同。舊守衛只擋 n_touch,
        #    缺速度欄的舊版 JSON 會整份混進總表,第一個 KeyError 就炸掉全表。
        vbad = [t for t, why in failed if why[0] == R_VERSION_BAD]
        if vbad:
            print("   ⚠️⚠️ 其中 %d 趟是**計分器版本不符**(JSON 缺欄位,不是航次"
                  "結果):%s —— 先把 repo 版 scripts/wsl/stot_score.py 部署到 %s"
                  " 再重算,這幾趟的判定在那之前都無效。"
                  % (len(vbad), "、".join(vbad[:8])
                     + ("…" if len(vbad) > 8 else ""), HERE))
    # ⚠️ 版本不符的話,「%d 趟通過」這個數字本身是無效的(首航次那一項量不出來,
    #    全部被判不通過)—— 必須印在同一個畫面上,不能只留在 stderr。
    if _MISMATCH:
        print("⚠️⚠️ 其中 %d 趟的 JSON 沒有 verdict 區塊(%s)⇒ 首航次判定量不出來,"
              "一律記不通過。這是**計分器版本不符**,不是航次結果:先部署 repo 版 "
              "stot_score.py 再重算,否則上面的通過數沒有意義。"
              % (len(_MISMATCH), "、".join(str(t) for t in _MISMATCH[:8])
                 + ("…" if len(_MISMATCH) > 8 else "")))
    # ⚠️ 速度紀錄只在**成功計分**的列裡挑(失敗列沒有數字可比);不要因此忘了
    #    它們仍在分母裡 —— 這一行講的是「最快的一趟」,不是成功率。
    best = [r for r in rows
            if r[1] is not None and r[1]["touch_gs_mean_ms"] is not None]
    if best:
        # 只在「其他三項都過」的航次裡找速度紀錄,否則會拿失敗航次的高速去誤導
        okish = [r for r in best
                 if r[1]["n_touch"] == 6
                 and r[1]["touch_max_miss_m"] is not None
                 and r[1]["touch_max_miss_m"] <= 5.0
                 and r[1]["touch_spread_s"] <= 3.0
                 and r[1]["max_abs_alt_m"] is not None
                 and r[1]["max_abs_alt_m"] <= a.alt_max]
        if okish:
            top = max(okish, key=lambda r: r[1]["touch_gs_mean_ms"])
            print("速度紀錄(其餘各項都達標的航次裡):%s,平均 %.1f m/s、"
                  "散布 %.2f s、誤差 %.2f m"
                  % (top[0], top[1]["touch_gs_mean_ms"],
                     top[1]["touch_spread_s"], top[1]["touch_max_miss_m"]))

    if a.csv:
        # ⚠️ 同上:不指定 encoding 就是主機 locale。中文 Windows 會寫出 cp950,
        #    而讀端(含這支自己、stot_extract_df.py、pandas)一律當 UTF-8 讀
        #    ⇒ 「寫得出來、讀不回去」。理由欄位是中文,一定會踩到。
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            # ⚠️ 多一個 reason 欄:另存的總表也必須帶得走「哪幾趟算不出來」,
            #    否則下游只讀得到成功的列,又是同一種灌高 —— 而 --csv 產出的
            #    這一份正是拿去排序、挑「最好的一趟」的東西。
            w.writerow(["trial", "n_touch", "gs_touch_mean", "gs_touch_min",
                        "gs_touch_max", "max_impact_miss_m", "max_abs_alt_m",
                        "touch_spread_s", "pass", "reason"])
            for t, s, ok, why in rows:
                if s is None:
                    w.writerow([t, "", "", "", "", "", "", "", 0,
                                "%s: %s" % (why[0], why[1])])
                    continue
                w.writerow([t, s["n_touch"], s["touch_gs_mean_ms"],
                            s["touch_gs_min_ms"], s["touch_gs_max_ms"],
                            s["touch_max_miss_m"], s["max_abs_alt_m"],
                            s["touch_spread_s"], int(ok), ""])
        print("已寫出 %s" % a.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
