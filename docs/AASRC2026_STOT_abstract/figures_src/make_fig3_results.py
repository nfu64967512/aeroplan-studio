#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AASRC 2026 STOT 中文加長型摘要 圖3：終端命中與觸地時刻差分解（ArduPilot SITL 飛行，每趟 2 架）。

用法（任何工作目錄皆可）::

    python docs/AASRC2026_STOT_abstract/figures_src/make_fig3_results.py
    python docs/AASRC2026_STOT_abstract/figures_src/make_fig3_results.py --selftest

輸出（只寫這三個檔）：
    docs/AASRC2026_STOT_abstract/fig3_results.png / .svg   17.0 cm × 6.2 cm、300 dpi（原寸插入）
    docs/AASRC2026_STOT_abstract/figures_src/fig3_data.json 圖中每個數值、原始檔路徑、欄位、判準、sha256

資料原則
- 每個數值都由本腳本直接讀 runs/artifacts/<ID>/ 下的原始檔重算，不手抄：
    tot_<ID>.csv（t, sysid, phase, mode 欄）、tot_<ID>_geom.csv（目標）、tot_standby.log（z_g）、
    contract_<ID>.json（values.* 只用來決定標籤，並做數值鎖）。
- 觸地判準：in-process 呼叫 scripts/wsl/stot_score.py 的 main([... "--json", "--log", log])，
  取 per_uav[*].impact_miss_m / impact_t_s（首航次窗 :223-231、首次 AGL<=0 收窗 :295-297、
  AGL 穿越 0 線性內插 :548-558）。stdout/stderr 導入記憶體；sys.dont_write_bytecode=True，
  不在 repo 產生 __pycache__；stot_score.main 只讀檔。
- 命中：impact_miss_m <= 10 m（batch_tally.py HIT_ERR_M，行號由本腳本搜尋原始碼取得）。
- 放行時刻：該架 phase=="DIVE" 且 mode=="dive" 的第一筆 t（同 batch_tally.py:75 的 mode 判準；
  本腳本另算不篩 phase 的版本並比對，不一致會列入 json）。
- 分解：Δ = 第 2 架 − 第 1 架（sysid 小者為第 1 架）；
    放行時刻差 = t_rel2 − t_rel1；俯衝耗時差 = (t_imp2 − t_rel2) − (t_imp1 − t_rel1)；
    觸地時刻差 = t_imp2 − t_imp1 = 兩者之和（恆等式，程式有斷言）。
- 分類（|·| 以 1e-6 容差比較 3 s）：
    兩分量皆 <=3 s（受控）；觸地差 <=3 s 但分量未皆 <=3 s（抵銷）；觸地差 >3 s（未達）。
- 母體篩選依序：無 geom → 計分器無輸出 → 非 2 架 → 有機未放行 → 有機首航次未觸地。
- 數值鎖：與已核對素材（86／15／6／9、RRB 2/14、RRD 10/16、RRE 15/16、代表航次分量）
  逐項比對，任一不符即不存檔、非零結束（--no-lock 可略過，僅供除錯）。

版面檢查（每次產圖都跑；任一項不過就不存檔、非零結束）
- 畫布 17.0 cm 寬、高 <= 6.2 cm；所有文字字級 >= 8 pt 且完全落在畫布內；
- 文字彼此不重疊；軸內文字與圖例不壓到資料點或長條、不跨出軸框；引線不穿過其他文字；
- 每個字元都有字型，英數必須由 Times New Roman 提供、中文由標楷體提供；
- 投稿用字：不得有底線字元與程式識別字（見 FORBIDDEN）。
``--selftest`` 會故意製造字太小／出畫布／重疊／壓點／缺字／英數字型錯／底線／尺寸超規，
確認每一項檢查真的會報錯。

決定性：PNG 去 Software 欄；SVG 去 Date、固定 hashsalt、字形轉路徑；json sort_keys、無時間戳。
主控台只印 ASCII 狀態記號，stdout 錯誤處理設 replace（cp950 主控台不中斷）。
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import io
import json
import math
import re
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent                 # .../AASRC2026_STOT_abstract/figures_src
OUT_DIR = HERE.parent                                  # .../AASRC2026_STOT_abstract
ROOT = HERE.parents[2]                                 # repo root
ART = ROOT / "runs" / "artifacts"
WSL = ROOT / "scripts" / "wsl"
SCORE_PY = WSL / "stot_score.py"
TALLY_PY = WSL / "batch_tally.py"
TOT_NODE_PY = WSL / "sar_swarm_patches" / "tot_node.py"

CM = 1.0 / 2.54
DPI = 300
FIG_W, FIG_H = 17.0, 6.2
FIG_H_MAX = 6.2
MIN_PT = 8.0
SPREAD_TOL_S = 3.0
EPS = 1e-6
L_AX = 40.0                   # (b) 軸範圍 ±40 s，界外點以邊緣三角形標示

FONT_KAI = Path("C:/Windows/Fonts/kaiu.ttf")
FONT_TNR = Path("C:/Windows/Fonts/times.ttf")

FORBIDDEN = ("ring_pace", "release_at_ring", "wait_goto_radius", "hold_ring", "stot_score",
             "batch_tally", "impact_miss", "impact_t", "tot_node", "sysid", ".py", ".csv",
             "S2", "phase", "mode")

# 已核對素材（僅作數值鎖；圖中數值一律由原始檔重算，不取自此處）
EXPECT = {
    "n_csv": 132, "n_unscorable": 13, "n_not2": 3, "n_unreleased": 3, "n_notouch": 27,
    "n_usable": 86, "n_pass": 15, "n_controlled": 6, "n_cancel": 9,
    "controlled_ids": ["RPK1", "RQA3", "RQD3", "RRD5", "RRE1", "WX2"],
    "cancel_ids": ["FORMB1", "FORMC1", "FORMD1", "RB1", "RPK2", "RPV1", "RPX3", "RQB2", "RRB5"],
    "out_of_range_ids": ["NZC7", "RPY3", "RQF1", "RQF2", "RQF4", "RQH1"],
    "batch": {"RRB": (14, 9, 2), "RRD": (16, 16, 10), "RRE": (16, 16, 15)},
    "chain": {"LAT1": (19.10, -2.30, 16.80), "RA1": (-33.90, 0.60, -33.30),
              "RB1": (-5.60, 3.86, -1.74), "SB1": (1.50, -7.05, -5.55),
              "SB2": (-1.10, 10.45, 9.35)},
}
BATCHES = ("RRB", "RRD", "RRE")
CHAIN = ("LAT1", "RA1", "RB1", "SB1", "SB2")


def rel(p: Path) -> str:
    return p.resolve().relative_to(ROOT).as_posix()


def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def find_line(path: Path, pattern: str) -> tuple[int, str]:
    """回傳原始碼中第一個符合 pattern 的（行號, 該行去空白）；找不到就丟例外（不靜默）。"""
    rx = re.compile(pattern)
    for i, ln in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if rx.search(ln):
            return i, ln.strip()
    raise RuntimeError(f"{rel(path)} 找不到 {pattern!r}")


# ════════════════════════════════════════════════════════════════════
#  資料（worker：每趟一個行程，只讀檔）
# ════════════════════════════════════════════════════════════════════
CONTRACT_KEYS = ("dive_angle", "ring_pace", "release_at_ring", "wait_goto_radius",
                 "tgo_from_predictor")


def score_run(rid: str) -> dict:
    sys.dont_write_bytecode = True
    if str(WSL) not in sys.path:
        sys.path.insert(0, str(WSL))
    import stot_score  # noqa: E402  （只讀檔；main 以 argv 呼叫）

    d = ART / rid
    csvp, geom, log, con = (d / f"tot_{rid}.csv", d / f"tot_{rid}_geom.csv",
                            d / "tot_standby.log", d / f"contract_{rid}.json")
    rec = {"id": rid, "inputs": {}, "has_geom": geom.exists(), "has_log": log.exists(),
           "has_contract": con.exists()}
    for p in (csvp, geom, log, con):
        if p.exists():
            rec["inputs"][rel(p)] = sha256(p)
    # ── 自行讀 CSV：放行時刻（第一筆 mode=="dive"）與機數 ──
    sysids, rel_dive, rel_any = set(), {}, {}
    with open(csvp, encoding="utf-8", errors="replace", newline="") as f:
        rd = csv.DictReader(f)
        if rd.fieldnames:
            for r in rd:
                try:
                    s = int(r["sysid"])
                except (TypeError, ValueError):
                    continue
                sysids.add(s)
                if r.get("mode") == "dive":
                    t = float(r["t"])
                    rel_any.setdefault(s, t)
                    if r.get("phase") == "DIVE":
                        rel_dive.setdefault(s, t)
    rec["n_sys_csv"] = len(sysids)
    rec["t_rel"] = {str(k): v for k, v in sorted(rel_dive.items())}
    rec["t_rel_no_phase_filter"] = {str(k): v for k, v in sorted(rel_any.items())}
    # ── 契約（只取標籤用的鍵）──
    if con.exists():
        vals = json.loads(con.read_text(encoding="utf-8")).get("values", {})
        rec["contract"] = {k: vals.get(k) for k in CONTRACT_KEYS}
    # ── stot_score 觸地判準 ──
    rec["score"] = None
    if geom.exists():
        argv = [str(csvp), "--json"] + (["--log", str(log)] if log.exists() else [])
        out, err = io.StringIO(), io.StringIO()
        try:
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                rc = stot_score.main(argv)
        except Exception as e:                                    # noqa: BLE001
            rc = f"exception {type(e).__name__}"
        rec["score_rc"] = rc
        if rc == 0:
            sc = json.loads(out.getvalue())
            rec["score"] = {
                "touch_datum": sc["window"]["touch_datum"],
                "per_uav": {k: {f: v.get(f) for f in ("impact_miss_m", "impact_t_s", "status",
                                                      "alt_agl_m", "window_start")}
                            for k, v in sorted(sc["per_uav"].items(), key=lambda kv: int(kv[0]))},
            }
        else:
            rec["score_stderr_head"] = err.getvalue().strip().splitlines()[:1]
    return rec


def collect(workers: int) -> dict:
    ids = sorted(p.name for p in ART.iterdir() if p.is_dir() and (p / f"tot_{p.name}.csv").exists())
    with ProcessPoolExecutor(max_workers=workers) as ex:
        recs = list(ex.map(score_run, ids))
    return {r["id"]: r for r in recs}


def cp_lower(k: int, n: int) -> float:
    """Clopper-Pearson 單側 95% 下限（同 batch_tally.py 的 beta.ppf(0.05, k, n-k+1)）。"""
    from scipy.stats import beta
    return 0.0 if k == 0 else float(beta.ppf(0.05, k, n - k + 1))


def analyse(R: dict) -> dict:
    hit_line, hit_src = find_line(TALLY_PY, r"^HIT_ERR_M\s*=")
    hit_err = float(hit_src.split("=")[1])
    rel_line, _ = find_line(TALLY_PY, r'x\["mode"\] == "dive"')
    spread_line, spread_src = find_line(SCORE_PY, r'touch_spread_s"\]\s*<=\s*3\.0')
    rp_line, _ = find_line(TOT_NODE_PY, r'g\("ring_pace",\s*False\)')

    # ── 母體（所有 n=2 航次）──
    excl = {"無幾何檔": [], "計分器無輸出（無 DIVE 資料或空檔）": [], "非 2 架": [],
            "有機未放行": [], "有機首航次未觸地": []}
    rows = []
    phase_filter_diff = []
    for rid, r in sorted(R.items()):
        if r["t_rel"] != r["t_rel_no_phase_filter"]:
            phase_filter_diff.append(rid)
        if not r["has_geom"]:
            excl["無幾何檔"].append(rid)
            continue
        sc = r["score"]
        if not sc:
            excl["計分器無輸出（無 DIVE 資料或空檔）"].append(rid)
            continue
        pu = sc["per_uav"]
        if len(pu) != 2:
            excl["非 2 架"].append(rid)
            continue
        s1, s2 = sorted(pu, key=int)
        if s1 not in r["t_rel"] or s2 not in r["t_rel"]:
            excl["有機未放行"].append(rid)
            continue
        i1, i2 = pu[s1]["impact_t_s"], pu[s2]["impact_t_s"]
        if i1 is None or i2 is None:
            excl["有機首航次未觸地"].append(rid)
            continue
        tr1, tr2 = r["t_rel"][s1], r["t_rel"][s2]
        d_rel = tr2 - tr1
        d_dive = (i2 - tr2) - (i1 - tr1)
        d_t = i2 - i1
        assert abs(d_rel + d_dive - d_t) < 1e-9, rid              # 恆等式
        both = abs(d_rel) <= SPREAD_TOL_S + EPS and abs(d_dive) <= SPREAD_TOL_S + EPS
        ok = abs(d_t) <= SPREAD_TOL_S + EPS
        cls = "controlled" if (ok and both) else ("cancel" if ok else "fail")
        raw_cls = ("controlled" if abs(d_t) <= 3 and abs(d_rel) <= 3 and abs(d_dive) <= 3
                   else "cancel" if abs(d_t) <= 3 else "fail")
        rows.append({
            "id": rid, "uav1": int(s1), "uav2": int(s2),
            "t_rel1_s": tr1, "t_rel2_s": tr2, "t_imp1_s": i1, "t_imp2_s": i2,
            "d_rel_s": round(d_rel, 6), "d_dive_s": round(d_dive, 6), "d_touch_s": round(d_t, 6),
            "err1_m": pu[s1]["impact_miss_m"], "err2_m": pu[s2]["impact_miss_m"],
            "class": cls, "class_without_eps": raw_cls, "both_components_le3": both,
            "out_of_range": abs(d_rel) > L_AX or abs(d_dive) > L_AX,
            "n_sys_csv": r["n_sys_csv"],
            "contract": r.get("contract"),
            "touch_datum": sc["touch_datum"],
        })
    cnt = {c: sum(1 for x in rows if x["class"] == c) for c in ("controlled", "cancel", "fail")}

    # ── (a) 三批逐機 ──
    batches = {}
    for b in BATCHES:
        ids = sorted((k for k in R if re.fullmatch(rf"{b}\d+", k)), key=lambda s: int(s[len(b):]))
        ac = []
        for rid in ids:
            sc = R[rid]["score"]
            if not sc:
                continue
            for s, v in sc["per_uav"].items():
                e = v["impact_miss_m"]
                ac.append({"run": rid, "uav": int(s), "impact_miss_m": e,
                           "status_first_pass": v["status"], "touch_datum": sc["touch_datum"],
                           "released": s in R[rid]["t_rel"],
                           "class": ("no_touch" if e is None else
                                     "hit" if e <= hit_err else "touch_miss")})
        n = len(ac)
        k = sum(1 for a in ac if a["class"] == "hit")
        nt = sum(1 for a in ac if a["class"] != "no_touch")
        contracts = {rid: R[rid].get("contract") for rid in ids}
        missing_csv = sorted({f"{b}{i}" for i in range(1, 9)} - set(ids))
        batches[b] = {"runs": ids, "runs_without_csv_1_to_8": missing_csv, "aircraft": ac,
                      "n": n, "n_touch": nt, "n_hit": k, "cp95_lower": cp_lower(k, n),
                      "contracts": contracts}
    # 標籤的數值鎖：放行方式由契約讀回決定
    lab_ok = (all((c or {}).get("release_at_ring") in (None, "0.0") for c in batches["RRB"]["contracts"].values())
              and all((c or {}).get("release_at_ring") == "1.0" and (c or {}).get("wait_goto_radius") in (None, "0.0")
                      for c in batches["RRD"]["contracts"].values())
              and all((c or {}).get("release_at_ring") == "1.0" and (c or {}).get("wait_goto_radius") == "1.0"
                      for c in batches["RRE"]["contracts"].values()))

    chain = {}
    for rid in CHAIN:
        x = next(q for q in rows if q["id"] == rid)
        c = x["contract"] or {}
        chain[rid] = {**{k: x[k] for k in ("d_rel_s", "d_dive_s", "d_touch_s", "err1_m", "err2_m",
                                           "class", "t_rel1_s", "t_rel2_s", "t_imp1_s", "t_imp2_s")},
                      "dive_angle_deg": float(c["dive_angle"]),
                      "ring_pace_on": c.get("ring_pace") == "1.0",
                      "ring_pace_raw": c.get("ring_pace")}
    return {"rows": rows, "excl": excl, "cnt": cnt, "batches": batches, "chain": chain,
            "labels_match_contracts": lab_ok, "phase_filter_diff": phase_filter_diff,
            "hit_err_m": hit_err, "hit_line": hit_line, "rel_line": rel_line,
            "spread_line": spread_line, "spread_src": spread_src, "ring_pace_default_line": rp_line}


def value_lock(R: dict, A: dict) -> list[str]:
    probs = []

    def eq(name, got, exp):
        if got != exp:
            probs.append(f"{name}: got {got} expected {exp}")

    eq("n_csv", len(R), EXPECT["n_csv"])
    eq("n_unscorable", len(A["excl"]["無幾何檔"]) + len(A["excl"]["計分器無輸出（無 DIVE 資料或空檔）"]),
       EXPECT["n_unscorable"])
    eq("n_not2", len(A["excl"]["非 2 架"]), EXPECT["n_not2"])
    eq("n_unreleased", len(A["excl"]["有機未放行"]), EXPECT["n_unreleased"])
    eq("n_notouch", len(A["excl"]["有機首航次未觸地"]), EXPECT["n_notouch"])
    eq("n_usable", len(A["rows"]), EXPECT["n_usable"])
    eq("n_pass", A["cnt"]["controlled"] + A["cnt"]["cancel"], EXPECT["n_pass"])
    eq("n_controlled", A["cnt"]["controlled"], EXPECT["n_controlled"])
    eq("n_cancel", A["cnt"]["cancel"], EXPECT["n_cancel"])
    eq("controlled_ids", sorted(x["id"] for x in A["rows"] if x["class"] == "controlled"),
       EXPECT["controlled_ids"])
    eq("cancel_ids", sorted(x["id"] for x in A["rows"] if x["class"] == "cancel"), EXPECT["cancel_ids"])
    eq("out_of_range_ids", sorted(x["id"] for x in A["rows"] if x["out_of_range"]),
       EXPECT["out_of_range_ids"])
    for b, (n, nt, k) in EXPECT["batch"].items():
        B = A["batches"][b]
        eq(f"{b} n/touch/hit", (B["n"], B["n_touch"], B["n_hit"]), (n, nt, k))
    for rid, exp in EXPECT["chain"].items():
        c = A["chain"][rid]
        got = (c["d_rel_s"], c["d_dive_s"], c["d_touch_s"])
        if any(abs(g - e) > 0.005 for g, e in zip(got, exp)):
            probs.append(f"chain {rid}: got {got} expected {exp}")
    if not A["labels_match_contracts"]:
        probs.append("RRB/RRD/RRE 契約開關與圖中放行方式標籤不符")
    if any(x["class"] != x["class_without_eps"] for x in A["rows"]):
        probs.append("容差 1e-6 改變了分類：" + str([x["id"] for x in A["rows"]
                                                  if x["class"] != x["class_without_eps"]]))
    return probs


# ════════════════════════════════════════════════════════════════════
#  作圖（主行程才 import matplotlib）
# ════════════════════════════════════════════════════════════════════
def setup_fonts():
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager as fm
    for f in (FONT_TNR, FONT_KAI):
        fm.fontManager.addfont(str(f))
    tnr = fm.FontProperties(fname=str(FONT_TNR)).get_name()
    kai = fm.FontProperties(fname=str(FONT_KAI)).get_name()
    matplotlib.rcParams.update({
        "font.family": [tnr, kai], "font.size": 8.0, "axes.titlesize": 8.5,
        "axes.labelsize": 8.0, "xtick.labelsize": 8.0, "ytick.labelsize": 8.0,
        "legend.fontsize": 8.0, "axes.unicode_minus": True, "axes.linewidth": 0.6,
        "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.minor.width": 0.4,
        "ytick.minor.width": 0.4, "xtick.major.size": 2.5, "ytick.major.size": 2.5,
        "xtick.minor.size": 1.4, "ytick.minor.size": 1.4,
        "xtick.direction": "in", "ytick.direction": "in", "hatch.linewidth": 0.5,
        "svg.fonttype": "path", "svg.hashsalt": "aasrc2026-stot-fig3", "pdf.fonttype": 42,
    })
    from fontTools.ttLib import TTFont
    cmaps = []
    for p in (FONT_TNR, FONT_KAI):
        with TTFont(str(p)) as ft:                 # 用 with 關檔（lazy 模式會留下 ResourceWarning）
            cmaps.append((p.name, dict(ft.getBestCmap())))
    return cmaps


def beeswarm(vals_cm: list[float], step_cm: float, unit_cm: float, max_off: float) -> list[float]:
    """一欄內的點依 y（cm）由小到大放置，水平偏移取最小可行值，點距 >= step_cm。"""
    order = sorted(range(len(vals_cm)), key=lambda i: (vals_cm[i], i))
    cands = [0.0]
    k = 1
    while k * step_cm * 0.55 / unit_cm <= max_off + 1e-9:
        o = k * step_cm * 0.55 / unit_cm
        cands += [o, -o]
        k += 1
    placed, off = [], [0.0] * len(vals_cm)
    for i in order:
        for o in cands:
            if all(math.hypot((o - po) * unit_cm, vals_cm[i] - py) >= step_cm for po, py in placed):
                off[i] = o
                placed.append((o, vals_cm[i]))
                break
        else:
            raise RuntimeError("beeswarm 放不下，請加寬欄位")
    return off


class Layout:
    def __init__(self, fig):
        self.fig = fig
        self.obstacles = []          # (ax, [x0,y0,x1,y1] cm, 名稱)
        self.leaders = []            # (Annotation, 目標點 cm)
        self.points = []             # (ax, cx, cy) cm，資料點中心

    def px2cm(self, bb):
        s = 2.54 / DPI
        return (bb.x0 * s, bb.y0 * s, bb.x1 * s, bb.y1 * s)

    def data2cm(self, ax, x, y):
        X, Y = ax.transData.transform((x, y))
        s = 2.54 / DPI
        return X * s, Y * s


def ax_cm(fig, x0, y0, w, h):
    W_cm, H_cm = (v * 2.54 for v in fig.get_size_inches())
    return fig.add_axes((x0 / W_cm, y0 / H_cm, w / W_cm, h / H_cm))


def text_bb(t, renderer):
    """文字本身的外框（Annotation.get_window_extent 會併入引線，這裡只取文字）。"""
    from matplotlib.text import Text
    return Text.get_window_extent(t, renderer=renderer)


def build(A: dict, height_cm: float = FIG_H):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Polygon, Rectangle
    import matplotlib.ticker as mt

    fig = plt.figure(figsize=(FIG_W * CM, height_cm * CM), dpi=DPI)
    lay = Layout(fig)
    AX_B, AX_T = 1.66, 5.66
    AX_H = AX_T - AX_B
    PITCH_PT = 10.0

    def table_row(ax, xs, labels, k, header=None, header_dx_cm=0.10):
        """軸下方第 k 列文字（與刻度標籤同一格線）；header 置於軸左緣外、靠右對齊。"""
        for x, s in zip(xs, labels):
            if not s:
                continue
            t = ax.annotate(s, (x, 0), xycoords=("data", "axes fraction"),
                            xytext=(0, -3.5 - k * PITCH_PT), textcoords="offset points",
                            ha="center", va="top", fontsize=8, annotation_clip=False)
            t.set_gid("outside")
        if header:
            t = ax.annotate(header, (0, 0), xycoords="axes fraction",
                            xytext=(-header_dx_cm / 2.54 * 72.0, -3.5 - k * PITCH_PT),
                            textcoords="offset points", ha="right", va="top", fontsize=8,
                            annotation_clip=False)
            t.set_gid("outside")

    # ─────────────────────────── (a) 逐機觸地誤差 ───────────────────────────
    axA = ax_cm(fig, 1.50, AX_B, 4.35, AX_H)
    Y_LO, Y_HI, Y_SEP, Y_NT = 0.03, 1.6e5, 2.4e4, 6.0e4
    axA.set_yscale("log")
    axA.set_ylim(Y_LO, Y_HI)
    axA.set_xlim(-0.5, 2.5)
    fig.canvas.draw()
    axA.axhspan(Y_SEP, Y_HI, color="0.90", lw=0, zorder=0)
    axA.axhline(Y_SEP, color="black", lw=0.6, zorder=1)
    axA.axhline(A["hit_err_m"], color="black", lw=0.8, ls=(0, (4, 2)), zorder=1)
    unit_cm = 4.35 / 3.0
    dec_cm = AX_H / math.log10(Y_HI / Y_LO)
    for j, b in enumerate(BATCHES):
        B = A["batches"][b]
        ac = B["aircraft"]
        ycm = [(math.log10(a["impact_miss_m"] if a["impact_miss_m"] is not None else Y_NT)) * dec_cm
               for a in ac]
        offs = beeswarm(ycm, step_cm=0.115, unit_cm=unit_cm, max_off=0.36)
        for a, o in zip(ac, offs):
            y = a["impact_miss_m"] if a["impact_miss_m"] is not None else Y_NT
            a["plot_x"], a["plot_y"] = j + o, y
            if a["class"] == "hit":
                kw = dict(marker="o", ms=3.0, mfc="black", mec="black", mew=0.5)
            elif a["class"] == "touch_miss":
                kw = dict(marker="o", ms=3.0, mfc="white", mec="black", mew=0.7)
            else:
                kw = dict(marker="x", ms=3.2, mec="black", mew=0.8)
            axA.plot([j + o], [y], ls="none", zorder=3, clip_on=False, **kw)
            lay.points.append((axA, j + o, y))
    axA.set_xticks([0, 1, 2])
    axA.set_xticklabels(["", "", ""])
    axA.tick_params(axis="x", length=0)
    axA.set_yticks([0.1, 1, 10, 100, 1000, 10000, Y_NT])
    axA.yaxis.set_major_formatter(mt.FuncFormatter(lambda v, _: "未觸地" if v == Y_NT else f"{v:g}"))
    axA.yaxis.set_minor_locator(mt.LogLocator(base=10, subs=range(2, 10), numticks=20))
    axA.yaxis.set_minor_formatter(mt.NullFormatter())
    axA.set_ylabel("觸地水平誤差 (m，對數軸)", labelpad=1.5)
    axA.set_title("(a) 逐機觸地誤差", loc="left", pad=3)
    t10 = axA.annotate(f"{A['hit_err_m']:g} m", (2.5, A["hit_err_m"]), xytext=(-2, 1.5),
                       textcoords="offset points", ha="right", va="bottom", fontsize=8)
    Bs = [A["batches"][b] for b in BATCHES]
    table_row(axA, [0, 1, 2], list(BATCHES), 0)
    table_row(axA, [0, 1, 2], ["環外放行", "抵環即放行", "＋等待修正"], 1)   # 審查 B21：同正文用詞
    table_row(axA, [0, 1, 2], [f"{B['n_hit']}/{B['n']}" for B in Bs], 2, header="命中")
    table_row(axA, [0, 1, 2], [f"{B['cp95_lower']:.3f}" for B in Bs], 3, header="95%下限")

    # ─────────────────────────── (b) 散布分解散點 ───────────────────────────
    BX0, BW = 6.95, 4.25
    axB = ax_cm(fig, BX0, AX_B, BW, AX_H)
    L = L_AX
    axB.set_xlim(-L, L)
    axB.set_ylim(-L, L)
    axB.add_patch(Polygon([(-3 * L, 3 * L + 3), (3 * L, -3 * L + 3), (3 * L, -3 * L - 3),
                           (-3 * L, 3 * L - 3)], closed=True, fc="0.82", ec="none", zorder=0))
    axB.add_patch(Rectangle((-3, -3), 6, 6, fc="none", ec="black", lw=0.8, zorder=2))
    axB.axhline(0, color="0.55", lw=0.4, zorder=1)
    axB.axvline(0, color="0.55", lw=0.4, zorder=1)
    STY = {"fail": dict(marker="o", ms=2.8, mfc="white", mec="0.35", mew=0.55),
           "cancel": dict(marker="^", ms=3.4, mfc="black", mec="black", mew=0.4),
           "controlled": dict(marker="s", ms=3.0, mfc="black", mec="black", mew=0.4)}
    for x in sorted(A["rows"], key=lambda q: ({"fail": 0, "cancel": 1, "controlled": 2}[q["class"]], q["id"])):
        px, py = x["d_rel_s"], x["d_dive_s"]
        if x["out_of_range"]:
            m = "<" if px < -L else ">" if px > L else "v" if py < -L else "^"
            # 內縮 2.4 s（≈0.13 cm）：第二輪目視 1.2 s 時三角形被軸框切掉一半
            cx, cy = max(-L + 2.4, min(L - 2.4, px)), max(-L + 2.4, min(L - 2.4, py))
            axB.plot([cx], [cy], ls="none", marker=m, ms=3.2, mfc="white", mec="black", mew=0.6, zorder=3)
            x["plot_xy"] = (cx, cy)
        else:
            axB.plot([px], [py], ls="none", zorder=3, **STY[x["class"]])
            x["plot_xy"] = (px, py)
        lay.points.append((axB, *x["plot_xy"]))
    axB.set_xticks([-30, -15, 0, 15, 30])
    axB.set_yticks([-30, -15, 0, 15, 30])
    axB.set_xlabel("放行時刻差 (s)", labelpad=1.5)
    axB.set_ylabel("俯衝耗時差 (s)", labelpad=1.5)
    n_all = len(A["rows"])
    axB.set_title(f"(b) 觸地時刻差分解（{n_all} 趟）", loc="left", pad=3)
    cnt = A["cnt"]
    n_oor = sum(1 for x in A["rows"] if x["out_of_range"])
    # 圖例放在 (b) 軸下方的空白（(a)(c) 在同高度是表列）
    hb = [Line2D([], [], ls="none", label=f"兩分量皆≤3 s：{cnt['controlled']}", **STY["controlled"]),
          Line2D([], [], ls="none", label=f"觸地差>3 s：{cnt['fail']}", **STY["fail"]),
          Line2D([], [], ls="none", label=f"靠正負抵銷：{cnt['cancel']}", **STY["cancel"]),
          Line2D([], [], ls="none", marker="<", ms=3.2, mfc="white", mec="black", mew=0.6,
                 label=f"超出±{L:g} s：{n_oor}")]
    lgb = axB.legend(handles=hb, ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.215),
                     frameon=False, borderpad=0.0, borderaxespad=0.0, handlelength=0.8,
                     handletextpad=0.35, columnspacing=0.9, labelspacing=0.12)
    lgb.set_gid("outside")
    # 第二輪目視：放右下角時字壓在灰帶上 ⇒ 移到右上角無資料點處（壓點由 check() 把關）
    band = axB.annotate("灰帶：\n觸地差≤3 s", (L - 1.0, L - 1.5), ha="right", va="top",
                        fontsize=8, zorder=6, linespacing=1.1)
    band.set_gid("inaxes")
    # 代表航次標號（引線指向該點；位置選在無資料點處，由 check() 把關）
    # 審查 B18：RA1 標號原在右下 (−25,−15)，引線沿途貼過 RRD2 (−32.6,−2.25)，終點看似指向 RRD2；
    # 改由右上方引入：搜尋候選位置（標號框離所有點 ≥2.3 s、引線離其他點 ≥3.1 s、避開 y 軸內向刻度），
    # 第一次試的正上方 (−34.5, 14.5) 壓到 15 s 刻度線
    for rid, tx, ty in (("LAT1", 25.0, 8.0), ("RA1", -24.0, 8.0)):
        x = next(q for q in A["rows"] if q["id"] == rid)
        px, py = x["plot_xy"]
        axB.annotate(rid, (px, py), xytext=(tx, ty), textcoords="data", ha="center", va="center",
                     fontsize=8, zorder=6,
                     arrowprops=dict(arrowstyle="-", lw=0.5, color="black", shrinkA=1.0, shrinkB=2.0))

    # ─────────────────────────── (c) 代表航次分量 ───────────────────────────
    CX0, CW = 12.35, 4.50
    axC = ax_cm(fig, CX0, AX_B, CW, AX_H)
    axC.set_xlim(-0.6, len(CHAIN) - 0.4)
    axC.set_ylim(-40, 25)
    axC.axhspan(-SPREAD_TOL_S, SPREAD_TOL_S, color="0.85", lw=0, zorder=0)
    axC.axhline(0, color="black", lw=0.5, zorder=1)
    BWID = 0.25
    kinds = (("d_rel_s", dict(fc="0.45", ec="black", lw=0.4), "放行時刻差"),
             ("d_dive_s", dict(fc="white", ec="black", lw=0.5, hatch="//////"), "俯衝耗時差"),
             ("d_touch_s", dict(fc="black", ec="black", lw=0.4), "觸地時刻差"))
    for j, rid in enumerate(CHAIN):
        c = A["chain"][rid]
        for m, (key, sty, _) in enumerate(kinds):
            v = c[key]
            xb = j + (m - 1) * (BWID + 0.02)
            r = Rectangle((xb - BWID / 2, min(0.0, v)), BWID, abs(v), zorder=3, **sty)
            axC.add_patch(r)
    axC.set_xticks(range(len(CHAIN)))
    axC.set_xticklabels([""] * len(CHAIN))
    axC.tick_params(axis="x", length=0)
    axC.set_yticks([-40, -30, -20, -10, 0, 10, 20])
    axC.set_ylabel("時刻差 (s)", labelpad=1.0)
    axC.set_title("(c) 代表航次的兩分量", loc="left", pad=3)
    hand = [Patch(label=lab, **sty) for _, sty, lab in kinds] + [Patch(fc="0.85", ec="none", label="±3 s")]
    lg = axC.legend(handles=hand, loc="lower right", frameon=False, borderpad=0.2, handlelength=1.3,
                    handleheight=0.8, handletextpad=0.4, labelspacing=0.15, borderaxespad=0.15)
    table_row(axC, range(len(CHAIN)), list(CHAIN), 0)
    table_row(axC, range(len(CHAIN)), [f"{A['chain'][r]['dive_angle_deg']:g}°" for r in CHAIN], 1,
              header="俯角")
    table_row(axC, range(len(CHAIN)), ["開" if A["chain"][r]["ring_pace_on"] else "關" for r in CHAIN], 2,
              header="配速")
    j_rb1 = CHAIN.index("RB1")
    c_rb1 = A["chain"]["RB1"]
    axC.annotate("抵銷", (j_rb1 + (BWID + 0.02), c_rb1["d_touch_s"]), xytext=(j_rb1 + 0.55, 11.0),
                 textcoords="data", ha="center", va="bottom", fontsize=8, zorder=6,
                 arrowprops=dict(arrowstyle="-", lw=0.5, color="black", shrinkA=0.5, shrinkB=1.0))

    for ax in (axA, axB, axC):
        ax.tick_params(pad=1.8)
    fig.canvas.draw()
    # 障礙物：長條
    for p in axC.patches:
        if p.get_zorder() == 3:
            lay.obstacles.append((axC, lay.px2cm(p.get_window_extent()), "bar"))
    return fig, lay, (axA, axB, axC)


# ════════════════════════════════════════════════════════════════════
#  版面檢查
# ════════════════════════════════════════════════════════════════════
def _ov(a, b, m=0.0):
    return not (a[2] + m <= b[0] or b[2] + m <= a[0] or a[3] + m <= b[1] or b[3] + m <= a[1])


def all_texts(fig):
    out = []
    for ax in fig.axes:
        xl, yl = sorted(ax.get_xlim()), sorted(ax.get_ylim())
        cands = [(ax.title, "title"), (ax._left_title, "title"), (ax._right_title, "title"),
                 (ax.xaxis.label, "label"), (ax.yaxis.label, "label")]
        for tk in ax.xaxis.get_major_ticks():
            if xl[0] - 1e-9 <= tk.get_loc() <= xl[1] + 1e-9:
                cands.append((tk.label1, "tick"))
        for tk in ax.yaxis.get_major_ticks():
            if yl[0] - 1e-9 <= tk.get_loc() <= yl[1] + 1e-9:
                cands.append((tk.label1, "tick"))
        for t in ax.texts:
            cands.append((t, "outside" if t.get_gid() == "outside" else "inaxes"))
        lg = ax.get_legend()
        if lg is not None:
            for t in lg.get_texts():
                cands.append((t, "legend"))
        for t, kind in cands:
            if t.get_visible() and t.get_text().strip():
                out.append((ax, t, kind))
    for t in fig.texts:
        if t.get_visible() and t.get_text().strip():
            out.append((None, t, "fig"))
    return out


def check(fig, lay, cmaps, height_limit=FIG_H_MAX) -> list[str]:
    probs = []
    fig.canvas.draw()
    r = fig.canvas.get_renderer()
    W_cm, H_cm = (v / CM for v in fig.get_size_inches())
    if abs(W_cm - FIG_W) > 1e-6:
        probs.append(f"圖寬 {W_cm:.3f} cm ≠ {FIG_W}")
    if H_cm > height_limit + 1e-6:
        probs.append(f"圖高 {H_cm:.3f} cm > {height_limit}")
    tnr = FONT_TNR.name
    recs = []
    for ax, t, kind in all_texts(fig):
        s = t.get_text()
        bb = lay.px2cm(text_bb(t, r))
        recs.append((ax, t, kind, bb))
        if t.get_fontsize() < MIN_PT - 1e-6:
            probs.append(f"字級 {t.get_fontsize():.2f} pt < {MIN_PT}：{s!r}")
        if bb[0] < 0.02 or bb[1] < 0.02 or bb[2] > W_cm - 0.02 or bb[3] > H_cm - 0.02:
            probs.append(f"文字超出畫布：{s!r} {tuple(round(v, 2) for v in bb)}")
        if "_" in s:
            probs.append(f"出現底線字元：{s!r}")
        for w in FORBIDDEN:
            if w.lower() in s.lower():
                probs.append(f"出現程式識別字 {w!r}：{s!r}")
        for ch in s:
            if ch in "\n ":
                continue
            owner = next((nm for nm, cm in cmaps if ord(ch) in cm), None)
            if owner is None:
                probs.append(f"沒有字型有這個字：{ch!r} in {s!r}")
            elif ch.isascii() and ch.isalnum() and owner != tnr:
                probs.append(f"英數沒走 Times New Roman：{ch!r}")
            elif not ch.isascii() and ord(ch) > 0x2E80 and owner != FONT_KAI.name:
                probs.append(f"中文沒走標楷體：{ch!r}")
    # 文字兩兩不重疊
    for i, (_, ta, _, a) in enumerate(recs):
        for _, tb, _, b in recs[i + 1:]:
            if _ov(a, b, 0.02):
                probs.append(f"文字重疊：{ta.get_text()!r} × {tb.get_text()!r}")
    # 軸內文字與圖例：不跨出軸框、不壓資料點與長條
    boxes = [(ax, t.get_text(), bb) for ax, t, kind, bb in recs if kind == "inaxes"]
    for ax in fig.axes:
        lg = ax.get_legend()
        if lg is not None and lg.get_gid() != "outside":
            boxes.append((ax, "圖例", lay.px2cm(lg.get_window_extent(renderer=r))))
    PR = 0.06                                    # 資料點半徑 cm（ms 3 pt ≈ 0.053 cm）
    for ax, s, bb in boxes:
        ab = lay.px2cm(ax.get_window_extent(renderer=r))
        if bb[0] < ab[0] + 0.02 or bb[1] < ab[1] + 0.02 or bb[2] > ab[2] - 0.02 or bb[3] > ab[3] - 0.02:
            probs.append(f"軸內文字跨出軸框：{s!r}")
        for pax, px, py in lay.points:
            if pax is not ax:
                continue
            cx, cy = lay.data2cm(ax, px, py)
            if _ov(bb, (cx - PR, cy - PR, cx + PR, cy + PR), 0.02):
                probs.append(f"文字壓到資料點：{s!r} @ ({px:.2f}, {py:.2f})")
        for oax, ob, name in lay.obstacles:
            if oax is ax and _ov(bb, ob, 0.03):
                probs.append(f"文字壓到{name}：{s!r}")
    # 引線不穿過其他文字
    for ax in fig.axes:
        for t in ax.texts:
            ap = getattr(t, "arrow_patch", None)
            if ap is None:
                continue
            verts = ap.get_transform().transform_path(ap.get_path()).vertices
            pts = []
            for (x0, y0), (x1, y1) in zip(verts, verts[1:]):
                for k in range(21):
                    X, Y = x0 + (x1 - x0) * k / 20, y0 + (y1 - y0) * k / 20
                    pts.append((X * 2.54 / DPI, Y * 2.54 / DPI))
            for _, tb, _, b in recs:
                if tb is t:
                    continue
                if any(b[0] - 0.02 <= x <= b[2] + 0.02 and b[1] - 0.02 <= y <= b[3] + 0.02 for x, y in pts):
                    probs.append(f"引線穿過文字：{t.get_text()!r} → {tb.get_text()!r}")
    return probs


def selftest(A, cmaps) -> int:
    import matplotlib.pyplot as plt
    cases = []
    fig, lay, (axA, axB, axC) = build(A)
    cases.append(("基準圖應無問題", check(fig, lay, cmaps) == []))
    t = axB.xaxis.label
    old = t.get_text()
    t.set_fontsize(6.5)
    cases.append(("字級過小", any("字級" in p for p in check(fig, lay, cmaps))))
    t.set_fontsize(8)
    t.set_text("放行時刻差 d_rel")
    got = check(fig, lay, cmaps)
    cases.append(("底線", any("底線" in p for p in got)))
    t.set_text("放行時刻差 \U0001F600")
    cases.append(("缺字", any("沒有字型" in p for p in check(fig, lay, cmaps))))
    t.set_text(old)
    saved = list(cmaps)
    cmaps.reverse()
    cases.append(("英數沒走 TNR", any("Times New Roman" in p for p in check(fig, lay, cmaps))))
    cmaps[:] = saved
    lab = next(q for q in axB.texts if q.get_text() == "LAT1")
    pos = lab.get_position()
    x = next(q for q in A["rows"] if q["id"] == "LAT1")
    lab.set_position(x["plot_xy"])
    cases.append(("軸內文字壓點", any("壓到資料點" in p for p in check(fig, lay, cmaps))))
    lab.set_position((pos[0], 38.5))
    cases.append(("軸內文字跨出軸框", any("跨出軸框" in p for p in check(fig, lay, cmaps))))
    lab.set_position(pos)
    ttl = axA._left_title
    ttl.set_x(-0.5)
    cases.append(("文字超出畫布", any("超出畫布" in p for p in check(fig, lay, cmaps))))
    ttl.set_x(0.0)
    cases.append(("尺寸超規", any("圖高" in p for p in check(fig, lay, cmaps, height_limit=6.0))))
    cases.append(("還原後應無問題", check(fig, lay, cmaps) == []))
    axB.xaxis.set_label_coords(0.5, -0.01)       # 壓到刻度標籤（之後整張重建）
    cases.append(("文字重疊", any("文字重疊" in p for p in check(fig, lay, cmaps))))
    plt.close(fig)
    fig, lay, _ = build(A)
    cases.append(("重建後應無問題", check(fig, lay, cmaps) == []))
    plt.close(fig)
    fails = 0
    for name, ok in cases:
        print(f"  [{'OK ' if ok else 'BAD'}] {name}")
        fails += 0 if ok else 1
    return fails


# ════════════════════════════════════════════════════════════════════
#  fig3_data.json
# ════════════════════════════════════════════════════════════════════
def write_json(R, A, png, svg):
    art = "runs/artifacts/<ID>/"
    score_rel = rel(SCORE_PY)
    crit = {
        "touchdown": (f"{score_rel} main(--json --log tot_standby.log)：首航次窗起點＝第一筆 "
                      "mode∈{dive,impact,expended}（:223-231）；首次 AGL<=0 無條件收窗（:295-297）；"
                      "AGL 穿越 0 線性內插（:548-558）；觸地誤差＝per_uav[*].impact_miss_m、"
                      "觸地時刻＝per_uav[*].impact_t_s；null＝窗內未觸地"),
        "hit": f"impact_miss_m <= {A['hit_err_m']:g} m（{rel(TALLY_PY)}:{A['hit_line']} HIT_ERR_M）",
        "release_time": (f"tot_<ID>.csv 該 sysid 第一筆 phase==\"DIVE\" 且 mode==\"dive\" 的 t"
                         f"（mode 判準同 {rel(TALLY_PY)}:{A['rel_line']}）"),
        "decomposition": ("Δ＝第2架−第1架（sysid 由小到大）；放行時刻差＝t_rel2−t_rel1；"
                          "俯衝耗時差＝(t_imp2−t_rel2)−(t_imp1−t_rel1)；觸地時刻差＝t_imp2−t_imp1＝兩者和"),
        "spread_threshold": (f"3.0 s（{score_rel}:{A['spread_line']} {A['spread_src']}）；"
                             "分類以 |·|<=3+1e-6 比較"),
        "classes": {"controlled": "|放行時刻差|<=3 且 |俯衝耗時差|<=3（圖(b)實心方塊）",
                    "cancel": "|觸地時刻差|<=3 但兩分量未皆<=3（實心三角）",
                    "fail": "|觸地時刻差|>3（空心圓）"},
        "cp95_lower": "Clopper-Pearson 單側 95% 下限 scipy.stats.beta.ppf(0.05, k, n-k+1)（同 batch_tally.py）",
        "population_filter_order": list(A["excl"].keys()),
    }
    items = []
    for b in BATCHES:
        B = A["batches"][b]
        for a in B["aircraft"]:
            items.append({"panel": "a", "batch": b, "run": a["run"], "uav": a["uav"],
                          "value_m": a["impact_miss_m"], "class": a["class"],
                          "shown_as": ("實心圓" if a["class"] == "hit" else "空心圓" if a["class"] == "touch_miss"
                                       else "× 於「未觸地」列"),
                          "plot_y": a["plot_y"], "first_pass_status": a["status_first_pass"],
                          "touch_datum": a["touch_datum"], "released": a["released"],
                          "source": {"file": f"runs/artifacts/{a['run']}/tot_{a['run']}.csv",
                                     "field": "stot_score per_uav[uav].impact_miss_m", "criterion": "touchdown"}})
        items.append({"panel": "a", "batch": b, "label": f"{b} 命中數/架次（表列）",
                      "value": f"{B['n_hit']}/{B['n']}", "n_touch": B["n_touch"],
                      "shown_as": f"{B['n_hit']}/{B['n']}", "criterion": "hit",
                      "runs": B["runs"], "runs_without_csv_1_to_8": B["runs_without_csv_1_to_8"]})
        items.append({"panel": "a", "batch": b, "label": f"{b} 命中率 Clopper-Pearson 單側 95% 下限（表列）",
                      "value": round(B["cp95_lower"], 6), "shown_as": f"{B['cp95_lower']:.3f}",
                      "criterion": "cp95_lower"})
        items.append({"panel": "a", "batch": b, "label": f"{b} 放行方式標籤依據（契約讀回）",
                      "value": {r: {k: (c or {}).get(k) for k in ("release_at_ring", "wait_goto_radius")}
                                for r, c in B["contracts"].items()},
                      "shown_as": {"RRB": "環外放行", "RRD": "抵環即放行", "RRE": "＋等待修正"}[b],
                      "source": {"file": f"{art}contract_<ID>.json", "field": "values.release_at_ring, "
                                 "values.wait_goto_radius（缺鍵＝預設 False，"
                                 f"{rel(TOT_NODE_PY)}:{A['ring_pace_default_line']}-"
                                 f"{A['ring_pace_default_line'] + 2}）"}})
    items.append({"panel": "a", "label": "命中門檻線", "value_m": A["hit_err_m"], "shown_as": "虛線與「10 m」",
                  "criterion": "hit"})
    for x in A["rows"]:
        items.append({"panel": "b", "run": x["id"],
                      **{k: x[k] for k in ("uav1", "uav2", "t_rel1_s", "t_rel2_s", "t_imp1_s", "t_imp2_s",
                                           "d_rel_s", "d_dive_s", "d_touch_s", "err1_m", "err2_m", "class",
                                           "out_of_range", "touch_datum", "n_sys_csv")},
                      "plot_xy": [round(v, 6) for v in x["plot_xy"]],
                      "contract_switches": x["contract"],
                      "source": {"file": f"runs/artifacts/{x['id']}/tot_{x['id']}.csv",
                                 "fields": "t, sysid, phase, mode（放行）；stot_score impact_t_s, impact_miss_m",
                                 "criterion": "release_time, touchdown, decomposition, classes"}})
    cnt = A["cnt"]
    items += [
        {"panel": "b", "label": "可用趟數（標題）", "value": len(A["rows"]), "shown_as": str(len(A["rows"]))},
        {"panel": "b", "label": "兩分量皆<=3 s 趟數", "value": cnt["controlled"],
         "ids": sorted(x["id"] for x in A["rows"] if x["class"] == "controlled")},
        {"panel": "b", "label": "靠正負抵銷趟數", "value": cnt["cancel"],
         "ids": sorted(x["id"] for x in A["rows"] if x["class"] == "cancel")},
        {"panel": "b", "label": "觸地差>3 s 趟數", "value": cnt["fail"]},
        {"panel": "b", "label": "超出 ±40 s 趟數（邊緣三角形）",
         "value": sum(1 for x in A["rows"] if x["out_of_range"]),
         "ids": sorted(x["id"] for x in A["rows"] if x["out_of_range"])},
        {"panel": "b", "label": "兩分量皆<=3 s 但觸地差>3 s 的趟數（應為 0）",
         "value": sum(1 for x in A["rows"] if x["both_components_le3"] and x["class"] == "fail")},
        {"panel": "b", "label": "排除（依篩選順序）", "value": {k: len(v) for k, v in A["excl"].items()},
         "ids": A["excl"]},
    ]
    for rid in CHAIN:
        c = A["chain"][rid]
        items.append({"panel": "c", "run": rid,
                      **{k: c[k] for k in ("d_rel_s", "d_dive_s", "d_touch_s", "err1_m", "err2_m", "class",
                                           "t_rel1_s", "t_rel2_s", "t_imp1_s", "t_imp2_s")},
                      "dive_angle_deg": c["dive_angle_deg"], "ring_pace_on": c["ring_pace_on"],
                      "shown_as": {"bars": "放行時刻差（灰）、俯衝耗時差（斜線）、觸地時刻差（黑）",
                                   "row_angle": f"{c['dive_angle_deg']:g}°",
                                   "row_pace": "開" if c["ring_pace_on"] else "關"},
                      "source": {"file": f"runs/artifacts/{rid}/tot_{rid}.csv；contract_{rid}.json",
                                 "fields": "同 (b)；values.dive_angle、values.ring_pace（缺鍵＝預設 False，"
                                           f"{rel(TOT_NODE_PY)}:{A['ring_pace_default_line']}）",
                                 "criterion": "decomposition"}})
    items.append({"panel": "c", "label": "±3 s 帶", "value_s": SPREAD_TOL_S, "criterion": "spread_threshold"})
    items.append({"panel": "c", "label": "「抵銷」標註", "run": "RB1",
                  "value": "放行時刻差與俯衝耗時差反號，觸地時刻差 |·|<=3 s 但分量未皆<=3 s"})
    inputs = {}
    for r in R.values():
        inputs.update(r["inputs"])
    for p in (SCORE_PY, TALLY_PY, TOT_NODE_PY):
        inputs[rel(p)] = sha256(p)
    sha_by_csv = {}
    for rid, r in R.items():
        h = r["inputs"].get(f"runs/artifacts/{rid}/tot_{rid}.csv")
        sha_by_csv.setdefault(h, []).append(rid)
    data = {
        "figure": {"png": rel(png), "svg": rel(svg), "width_cm": FIG_W, "height_cm": FIG_H, "dpi": DPI,
                   "script": rel(Path(__file__))},
        "criteria": crit,
        "items": items,
        "checks": {
            "n_tot_csv": len(R),
            "duplicate_csv_sha256": [v for v in sha_by_csv.values() if len(v) > 1],
            "release_time_phase_filter_changes_runs": A["phase_filter_diff"],
            "usable_touch_datums": sorted({x["touch_datum"] for x in A["rows"]}),
            "usable_runs_with_n_sys_csv_not_2": [x["id"] for x in A["rows"] if x["n_sys_csv"] != 2],
            "value_lock_against_verified_material": EXPECT,
        },
        "caveats": [
            "全部為 ArduPilot SITL 飛行（非實飛）；每趟 2 架；plane-jet 低阻改裝（JET_CDP=0.01642）舊剖面基線；"
            "契約讀回 THR_MAX=100（設計值 25）；未設 SIM_WIND（推定無風，無讀回值）。",
            "終端導引為帶視線地板的雙通道純比例導引（契約 pn_los_floor=1.0、pn_nz_min=-1.0；縱向 N=5、橫向 N=4）。",
            "環上配速（ring_pace）、抵環即放行（release_at_ring）、等待段修正（wait_goto_radius）"
            "在 tot_node.py 預設皆為 False，屬非預設實驗開關；以契約讀回為準。",
            "(b) 母體 86 趟高度異質（27° 72 趟、20° 10 趟、未知 4 趟；多種開關組合），不是單一配方的重複；"
            "Δ 的正負取決於機號順序，(x,y) 與 (−x,−y) 同義。",
            "(b) 超出 ±40 s 的 6 趟畫在軸框內側邊緣（三角形指向界外方向），原值見 items。",
            "(c) LAT1 與 RA1 契約只差 dive_angle，RA1 與 RB1 為環上配速開關；兩趟之間的俯衝耗時差差異小於"
            "逐機俯衝耗時 σ≈3.3 s，只作示意，不作機制證據；SB1、SB2 契約另多 6 個鍵（近同組態重複）。",
            "單趟觸地時刻差雜訊約 ±10 s；RRB7 目錄無 tot CSV（僅契約）。",
            "runs/artifacts 被 .gitignore:109 排除、不在版控；inputs_sha256 供核對。",
        ],
        "inputs_sha256": dict(sorted(inputs.items())),
    }
    (HERE / "fig3_data.json").write_text(json.dumps(data, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
                                         encoding="utf-8")


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--no-lock", action="store_true", help="略過數值鎖（僅供除錯，不應用於投稿圖）")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--cache", type=Path, default=None,
                    help="（僅供版面迭代）逐趟重算結果的暫存 json，應放在 repo 外；投稿圖不要用")
    ap.add_argument("--debug-png", type=Path, default=None,
                    help="版面檢查失敗時仍把圖存到這裡（應放在 repo 外）以便目視")
    args = ap.parse_args()

    if args.cache is not None and args.cache.exists():
        R = json.loads(args.cache.read_text(encoding="utf-8"))
        print("WARN using cache", args.cache.name)
    else:
        R = collect(args.workers)
        if args.cache is not None:
            args.cache.write_text(json.dumps(R, ensure_ascii=False, sort_keys=True), encoding="utf-8")
    A = analyse(R)
    probs = value_lock(R, A)
    print(f"runs={len(R)} usable={len(A['rows'])} cnt={A['cnt']} "
          f"batches={[(b, A['batches'][b]['n_hit'], A['batches'][b]['n']) for b in BATCHES]}")
    if probs:
        print("FAIL value lock:")
        for p in probs:
            print("   -", p)
        if not args.no_lock:
            return 1
    else:
        print("OK   value lock (%d checks vs verified material)" % 14)

    import matplotlib.pyplot as plt
    cmaps = setup_fonts()
    if args.selftest:
        return 1 if selftest(A, cmaps) else 0
    fig, lay, _ = build(A)
    lp = check(fig, lay, cmaps)
    if lp:
        print(f"FAIL layout check {len(lp)}:")
        for p in lp:
            print("   -", p)
        if args.debug_png is not None:
            fig.savefig(args.debug_png, dpi=DPI, facecolor="white")
            print("WARN debug png written (layout FAILED):", args.debug_png.name)
        plt.close(fig)
        return 1
    png, svg = OUT_DIR / "fig3_results.png", OUT_DIR / "fig3_results.svg"
    fig.savefig(png, dpi=DPI, facecolor="white", metadata={"Software": None})
    fig.savefig(svg, facecolor="white", metadata={"Date": None, "Creator": None})
    plt.close(fig)
    write_json(R, A, png, svg)
    for p in (png, svg, HERE / "fig3_data.json"):
        print(f"OK   {p.name}  sha256:{sha256(p)[:16]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
