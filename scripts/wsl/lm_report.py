#!/usr/bin/env python3
"""lm_report.py — 把一批航次整理成可直接餵給 NotebookLM 的 Markdown。

⚠️⚠️ 為什麼要有一支專門的產生器,而不是手工貼數字:

  1. **手工挑數字會挑到支持自己假設的那些。** 這一批的價值在於它是同組態
     對照,任何一次「只貼有趣的那趟」都會毀掉對照。所以這裡的規則是
     **全部航次都出,包含失敗的**,失敗原因照實寫。
  2. 決定性的量(放行瞬間的 γ_req / d / h / 徑向速率)在 RPY3 之前
     **一個都沒被記錄**,三個診斷代理是從 .BIN 反查的。這支把它們算出來
     並固定成表,下次不用再反查。

⚠️⚠️ 上傳到 NotebookLM 之前必讀(memory/notebooklm-cites-our-own-docs):
  NotebookLM 會把**我們自己上傳的文件**當外部文獻引用,並憑空生成本系統的
  假數字。所以本檔開頭固定放一段「這是實測資料不是文獻」的標頭,
  之後在回答裡看到這些數字被當成「文獻說」就知道是污染。

用法:
  python3 lm_report.py --trials RPZ1 RPZ2 RPZ3 --baseline RPY1 RPY3 \
      --out docs/lm_rpz.md
"""
import argparse
import collections
import csv
import json
import math
import os
import pathlib
import statistics as st

ROOT = pathlib.Path(__file__).resolve().parents[2]
ART = ROOT / "runs" / "artifacts"
EARTH = 111320.0


def _f(x, d=None):
    try:
        v = float(x)
        return None if math.isnan(v) else v
    except (TypeError, ValueError):
        return d


def dist_m(a, b):
    return math.hypot((a[0] - b[0]) * EARTH,
                      (a[1] - b[1]) * EARTH * math.cos(math.radians(a[0])))


def target_of(trial):
    p = ART / trial / f"tot_{trial}_geom.csv"
    if not p.exists():
        return None
    for r in csv.DictReader(p.open(encoding="utf-8", errors="replace")):
        if r.get("kind") == "target":
            return (float(r["lat"]), float(r["lon"]))
    return None


def load(trial):
    p = ART / trial / f"tot_{trial}.csv"
    if not p.exists():
        return None
    rows = list(csv.DictReader(p.open(encoding="utf-8", errors="replace")))
    by = collections.defaultdict(list)
    for r in rows:
        by[r["sysid"]].append(r)
    return by


def per_uav(trial):
    """回傳 {uav: {...}} —— 放行幾何、俯衝耗時、速度指令、觸地。"""
    by = load(trial)
    T = target_of(trial)
    if not by or not T:
        return {}
    out = {}
    for u, rs in sorted(by.items()):
        rel = next((r for r in rs if r["mode"] == "dive"), None)
        dv = [r for r in rs if r["mode"] == "dive"]
        touch = next((r for r in rs if (_f(r["agl"]) or 9e9) < 3.0), None)
        d = dict(uav=u, n=len(rs))
        if rel:
            dr = dist_m((float(rel["lat"]), float(rel["lon"])), T)
            h = _f(rel["agl"]) or 0.0
            # 徑向速率:放行前 10 s 的距離變化率(> 0 = 外擺,< 0 = 內衝)
            t0 = float(rel["t"])
            prev = [r for r in rs if t0 - 10.5 <= float(r["t"]) < t0 - 0.5]
            rate = None
            if prev:
                d0 = dist_m((float(prev[0]["lat"]), float(prev[0]["lon"])), T)
                dt = t0 - float(prev[0]["t"])
                rate = (dr - d0) / dt if dt > 0.5 else None
            d.update(rel_t=t0, rel_d=dr, rel_h=h,
                     gam_req=math.degrees(math.atan2(h, max(dr, 1.0))),
                     radial=rate)
        if dv:
            # ⚠️ 兩個門檻都從**該趟契約現場讀回**,不寫死 —— 寫死的話換組態
            #    這一欄就會靜靜地量錯東西。
            _cv = (contract_of(trial).get("values") or {})
            vmin = _f(_cv.get("vmin"), 95.0)
            dive_sp = _f(_cv.get("dive_speed"), 210.0)
            v = [_f(r["v_cmd"]) for r in dv if _f(r["v_cmd"]) is not None]
            g = [_f(r["gs"]) for r in dv if _f(r["gs"]) is not None]
            d.update(v_cmd_min=min(v) if v else None,
                     v_cmd_max=max(v) if v else None,
                     v_cmd_med=st.median(v) if v else None,
                     # ⭐ 主要判讀量:對 **vmin** 比,不是對「這架自己的最小值」比。
                     # ⚠️ 一開始寫成 `abs(x - min(v))`,結果 RPZ1-UAV2 全程恆定
                     #    210(= dive_speed **上限**飽和)也被算成「釘下限 100%」——
                     #    那與 RPY 的「釘 vmin」意義完全相反,一個是權限用滿、
                     #    一個是權限歸零,混在同一欄會把結論讀反。
                     v_pinned=(100.0 * sum(1 for x in v if abs(x - vmin) < 0.51)
                               / len(v)) if v else None,
                     v_at_cap=(100.0 * sum(1 for x in v if abs(x - dive_sp) < 0.51)
                               / len(v)) if v else None,
                     gs_peak=max(g) if g else None)
        if touch and rel:
            d.update(touch_t=float(touch["t"]),
                     dive_s=float(touch["t"]) - float(rel["t"]))
        # 預測器欄位有沒有真的產出(RPY 全空)
        for col in ("pred_residual_alt_m", "pred_h_cross", "pred_over_limit_frac",
                    "pred_sat", "secs_to_target", "tgo_s", "early_by_s", "e_t_s"):
            if col in rs[0]:
                n = sum(1 for r in rs if r.get(col) not in (None, "", "nan"))
                if n:
                    d.setdefault("cols", {})[col] = n
        out[u] = d
    return out


def contract_of(trial):
    p = ART / trial / f"contract_{trial}.json"
    if not p.exists():
        return {}
    try:
        return json.load(p.open(encoding="utf-8"))
    except Exception:                                    # noqa: BLE001
        return {}


def verdict_lines(prefix):
    """從 batchlog 撈每趟的判定行 —— ⚠️ 包含失敗的,不過濾。"""
    p = ROOT / "runs" / "batchlog" / f"{prefix}_console.log"
    if not p.exists():
        return []
    keep = []
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        if any(k in s for k in ("判定:", "觸地時間散布", "觸地水平誤差",
                                "終端總速", "沒起飛", "逾時,沒有結案",
                                "★★ 俯衝結案", "拒收共同命中時刻",
                                "重排共同命中時刻")):
            if s not in keep:
                keep.append(s)
    return keep


HEADER = """# STOT 飛測結果:{tag}

> ## ⚠️ 這份文件是**我們自己 SITL 實測的資料**,不是文獻
>
> 若你在回答裡把下面任何數字當成「文獻指出」「研究顯示」,那是錯的 ——
> 它們來自本專案的 ArduPlane SITL 航次,沒有任何論文知道這些值。
> 請在回答時明確區分:
>   * 「**文獻逐字**」= 來自你資料庫裡的論文,附篇名
>   * 「**實測**」= 來自本文件
>   * 「**推論**」= 你自己的推理
>
> 系統:定翼噴射無人機群、同時到達目標(STOT)、巡航 157 m/s、
> 終端導引用比例導引(PN)經 SET_ATTITUDE_TARGET 繞過 TECS。
> 驗收:散布 ≤3 s、命中誤差 ≤10 m、終端總速 204 m/s。

"""


def render(tag, trials, baseline, prefix, base_prefix):
    L = [HEADER.format(tag=tag)]

    L.append("## 本批要驗證什麼\n")
    L.append("上一批(基準)發現兩個問題,本批是修好之後的對照:\n")
    L.append("1. **`dive_T` 時基污染** —— 剩餘時間被算成 17.8 億秒 ⇒ "
             "速度需求 `v_req = 距離/剩餘時間 ≈ 0` ⇒ 俯衝段速度指令"
             "**全程釘在下限 vmin=95 m/s**,`dive_speed=210` 一次都沒生效。\n"
             "   ⚠️ 注意:放行時刻表**沒有**壞掉 —— 比較式兩側帶同一個偏移,"
             "正好抵銷。死的只有速度通道。\n")
    L.append("2. **線上預測器從未執行** —— `predict_hz` 預設 0 且從未接線,"
             "預測欄位整批空白,而且是 early-return、**零告警**。\n")
    L.append("\n**主要判讀量:`v_cmd 釘在下限的比例`。基準批四架次全是 100%。**\n")

    for label, group, pre in (("本批", trials, prefix),
                              ("基準(修復前)", baseline, base_prefix)):
        if not group:
            continue
        L.append(f"\n## {label}:逐架次資料\n")
        L.append("| 航次 | UAV | 放行時所需俯角 | 放行時水平距 | 放行時高度 | "
                 "徑向速率 | 俯衝耗時 | v_cmd 中位 | **釘 vmin %** | 頂到上限 % | 峰值地速 |")
        L.append("|---|---|---|---|---|---|---|---|---|---|---|")
        for t in group:
            d = per_uav(t)
            if not d:
                L.append(f"| {t} | — | — | — | — | — | — | — | — | — |")
                continue
            for u, x in sorted(d.items()):
                def g(k, f="{:.1f}", suf=""):
                    v = x.get(k)
                    return "—" if v is None else (f.format(v) + suf)
                L.append(
                    f"| {t} | {u} | {g('gam_req','{:.1f}','°')} | "
                    f"{g('rel_d','{:.0f}',' m')} | {g('rel_h','{:.0f}',' m')} | "
                    f"{g('radial','{:+.0f}',' m/s')} | {g('dive_s','{:.1f}',' s')} | "
                    f"{g('v_cmd_med','{:.1f}')} | **{g('v_pinned','{:.1f}','%')}** | "
                    f"{g('v_at_cap','{:.1f}','%')} | {g('gs_peak','{:.0f}')} |")

    L.append("\n### 欄位定義(⚠️ 帶方向)\n")
    L.append("* **所需俯角** = `atan(高度 / 到目標水平距)`,**正值**,"
             "指「若走直線打到目標,航跡必須向下傾多少度」。\n")
    L.append("* **徑向速率**:**正 = 正在遠離目標(外擺)**,負 = 正在逼近(內衝)。\n")
    L.append("* **釘 vmin %**:速度指令 = vmin 的比例。**100% = 同步迴路在俯衝段"
             "完全沒有權限**(基準批四架次皆是)。\n")
    L.append("* **頂到上限 %**:速度指令 = dive_speed 的比例。⚠️ 與上一欄意義**相反** —— "
             "一個是權限歸零、一個是權限用滿。混看會把結論讀反。\n")
    L.append("* **俯衝耗時** = 從 `mode` 轉入 `dive` 到首次 AGL < 3 m。\n")

    # ── 散布分解:這是本批最重要的一張表 ────────────────────────────────
    L.append("\n## ⭐ 散布分解:放行時刻差 vs 俯衝耗時差\n")
    L.append("觸地時間差 = **放行時刻差** + **俯衝耗時差**。"
             "把散布拆成這兩項,才知道下一步該動上游還是下游。\n")
    L.append("| 航次 | 放行時刻差 | 俯衝耗時差 | 觸地散布 | 主導項 |")
    L.append("|---|---|---|---|---|")
    for t in list(trials) + list(baseline):
        d = per_uav(t)
        us = [u for u in sorted(d) if d[u].get("rel_t") and d[u].get("touch_t")]
        if len(us) < 2:
            L.append(f"| {t} | — | — | — | (資料不足:航次失敗或未結案) |")
            continue
        a, b = d[us[0]], d[us[1]]
        dr = b["rel_t"] - a["rel_t"]
        dd = b["dive_s"] - a["dive_s"]
        L.append(f"| {t} | {dr:+.1f} s | {dd:+.1f} s | {abs(dr + dd):.1f} s | "
                 f"{'放行時刻' if abs(dr) > abs(dd) else '俯衝耗時'} |")
    L.append("\n(差值皆為 UAV2 − UAV1。)\n")

    L.append("\n## 預測器欄位是否真的有輸出\n")
    L.append("| 航次 | UAV | 有值的欄位(欄名: 筆數) |")
    L.append("|---|---|---|")
    for t in list(trials) + list(baseline):
        d = per_uav(t)
        for u, x in sorted(d.items()):
            c = x.get("cols") or {}
            L.append(f"| {t} | {u} | " +
                     (", ".join(f"{k}: {v}" for k, v in sorted(c.items()))
                      if c else "**(全空)**") + " |")

    L.append("\n## 生效值契約(每趟現場讀回,非抄自啟動指令)\n")
    for t in list(trials) + list(baseline):
        c = contract_of(t)
        v = c.get("values") or {}
        if not v:
            continue
        keys = ["vmin", "vmax", "dive_speed", "dive_pace", "gate_r", "perch_alt",
                "dive_angle", "dive_angle_max", "pn_los_floor", "pn_nz_min",
                "hold_radius", "predict_hz", "tgo_from_predictor"]
        L.append(f"\n**{t}**  " + " / ".join(
            f"{k}={v[k]}" for k in keys if k in v))
        if c.get("violations"):
            L.append(f"\n  ⚠️ 違規:{c['violations']}")

    L.append("\n## 批次判定原文(⚠️ 含失敗航次,未過濾)\n```")
    for pre in (prefix, base_prefix):
        if not pre:
            continue
        L.append(f"── {pre} ──")
        L.extend(verdict_lines(pre) or ["(無 batchlog)"])
    L.append("```")

    L.append("\n## ⚠️ 本批已經自行查明的機制(請據此回答,不要重述)\n")
    L.append("放行前有一道**空間**閘門:必須先飛進 pushover 環才有資格放行。\n"
             "環半徑 6084 m × 寬容倍率 1.10 = **6692 m**。\n\n"
             "| 航次 | UAV | 第一架放行當下的距離 | 是否在環內 | 結果 |\n"
             "|---|---|---|---|---|\n"
             "| RPZ1 | 1 | 4964 m | 在環內 | 立刻放行 |\n"
             "| RPZ1 | 2 | **7543 m** | **環外 851 m** | 等 37 s 飛進來才放行 |\n"
             "| RPZ2 | 1 | 5360 m | 在環內 | 放行 |\n"
             "| RPZ2 | 2 | 5615 m | 在環內 | 放行(差 2.5 s)|\n\n"
             "而共同命中時刻的錯開機制本身是**時間**的(t_go 大的先放行)。"
             "兩者疊在一起時,空間閘門會**壓過**時間錯開:環外的機體即使時刻表"
             "已經輪到它,也不能放行。RPZ1 的 +37.2 s 就是這樣來的。\n\n"
             "⚠️ 諷刺的是,程式碼自己的註解就寫著這個設計是為了取代空間閘門:"
             "「舊版用統一的空間座標解脫……抵達同一個圓的時刻天然就差 51 s」。"
             "空間閘門並沒有被取代,只是被移到了時間錯開的**前面**。\n")

    L.append("\n## 想請教的問題\n")
    L.append("1. **時間協調**與**空間可行域**這兩個條件,文獻怎麼安排先後?"
             "是把「還要多久才進得了可行域」併進共同命中時刻的可行窗計算,"
             "還是直接放寬/取消空間閘門?兩種做法的代價分別是什麼?\n")
    L.append("2. 我們的共同命中時刻取的是 `max_j(t_go_j) + 2 s`。"
             "若某架**當下還進不了可行域**,這個 max 就低估了它真正需要的時間。"
             "文獻有沒有處理「可行時間窗下界不只由航程決定」的情形?\n")
    L.append("3. 俯衝耗時差已經收到 −1.0 / +6.4 s。若把放行時刻差也壓到個位數,"
             "**剩下的誤差來源會是什麼**?請按預期量級排序,並說明各自用什麼量觀測。\n")
    L.append("4. 本批三趟有一趟(RPZ3)完全沒有結案(1500 s 逾時)。"
             "同組態三趟出現一趟完全不同的行為,在實驗設計上該怎麼處理 ——"
             "當離群值剔除,還是視為必須解釋的失效模式?\n")
    L.append("\n⚠️ 回答時請對每一條標明「文獻逐字 / 實測 / 推論」。"
             "上表所有數字都是**實測**,不是文獻。\n")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", nargs="+", required=True)
    ap.add_argument("--baseline", nargs="*", default=[])
    ap.add_argument("--prefix", default=None, help="本批 batchlog 前綴")
    ap.add_argument("--baseline-prefix", default=None)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    pre = a.prefix or a.trials[0].rstrip("0123456789")
    bpre = a.baseline_prefix or (a.baseline[0].rstrip("0123456789")
                                 if a.baseline else None)
    tag = a.tag or pre
    txt = render(tag, a.trials, a.baseline, pre, bpre)
    out = pathlib.Path(a.out)
    os.makedirs(out.parent, exist_ok=True)
    out.write_bytes(txt.encode("utf-8"))       # ⚠️ write_bytes:避免 CRLF
    print(f"[ok] {out}  ({len(txt)} chars)")


if __name__ == "__main__":
    main()
