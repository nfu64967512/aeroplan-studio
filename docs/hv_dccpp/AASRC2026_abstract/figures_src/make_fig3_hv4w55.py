"""AASRC 2026 論文 圖3（重畫）：HV4W55（1 Pioneer + 2 HERO，天花板 119.5 m，模擬風 5 m/s 西風）。

    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig3_hv4w55.py [--layout abc|bc] [--bin-cache DIR]

版面
    abc（預設）：(a) 左上、(c) 左下、(b) 右欄；17.0 × 12.6 cm
    bc         ：(b)(c) 橫排、無 (a)，風場寫進圖說（fig3_hv4w55_bc_caption.txt）；17.0 × 9.6 cm
    wide       ：(b)(c) 橫排、長寬比 2.764（≥ 2.75）；17.0 × 6.15 cm。(b) 剖面內嵌放大框，圖例／逾越累計／逐機表
                 放在 (b) 右側文字欄；(c) 機對一行圖例，違規區定義與 A1–B2 註記移到圖說（fig3_hv4w55_wide_caption.txt）
                 ⚠ 只有兩格，圖上與圖說的編號印成 (a) 高度剖面、(b) 機間間距（PANEL_TAG）
    3col       ：(a) 平面航跡／(b) 高度剖面／(c) 機間間距三欄並排，2007 × 620 px（16.997 × 5.254 cm）。頂列一行
                 各機航程（規劃→飛行，兼作三機線型圖例）；(b) 頂部兩列參考線圖例、放大框右側為逾越註記；
                 圖內不出現架次代號（本架次／對照架次）。輸出 fig3_results_hv4w55_3col.png／.svg
用詞（2026-09-25）：圖上一律「規劃上限」（不用「天花板」）；HV4W55R 顯示為「對照架次」（R_NAME）
三格
    (a) 平面航跡（寬淡線：規劃航點連線；細線：實飛），標註各機航程 規劃→飛行、風（規劃＝模擬）箭頭
    (b) 高度剖面（POS.RelHomeAlt 原生取樣率）。主圖**只畫航段輪廓**：120.000 m 作業上限（實線）與 119.5 m
        規劃天花板（虛線）相距 0.5 m，在 0–125 m 主圖上會黏成一條、讀成「貼齊 120 m」⇒ 兩條參考線與刻度
        只放在 119.4–120.3 m 放大內插框；框內疊 HV4W55R（天花板 120.0 m）單變因對照，框旁標逾越 120.000 m
        累計時間（HV4W55R vs 本架次）
    (c) 同時在空中的機間間距（水平 × 垂直）

資料與數字的分工（**不手 key 任何數字**）
    - 圖上**顯示的每個數字**一律讀 runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json 與 metrics.json。
    - **幾何**（航跡、高度曲線、間距路徑）直接讀 sitl/vtol/i10{0,1,2}/logs/000000NN.BIN，用產生那些數字的
      **同一支程式**：update_paper/scripts/up_extract.extract()（BIN→陣列）、up_lib.enu()（WGS84 ECEF→ENU）、
      up_metrics._track(frame="ecef") + up_metrics._pair_min()（飛控原生 POS 5.1–6.4 Hz、兩機原生時刻聯集、
      連續時間最小值）。不用 binsep.py、separation_from_tracks.json、111132 m/度平面近似或 2 s 網格。
    - 畫圖前先驗證：BIN 重抽的陣列與 update_paper/cache/*.npz 逐元素相同；以 BIN 重算的 altitude()、timing()、
      _pair_min() 與 metrics.json 逐欄相同。任何一項不符就中止，不出圖。
    - 最後印出「圖上每個數字 ↔ NUMBERS.json／metrics.json 欄位」逐項比對表（也寫進 fig3_hv4w55_numbers.tsv）。

輸出（17.0 cm 寬、300 dpi；圖內最小字 9 pt，程式在存檔前逐一檢查所有文字物件）
    docs/hv_dccpp/AASRC2026_abstract/fig3_results_hv4w55{,_bc}.png / .svg
    docs/hv_dccpp/AASRC2026_abstract/figures_src/fig3_hv4w55{,_bc}_data.json（圖中每個數值 + 出處 + 輸入 sha256）
    docs/hv_dccpp/AASRC2026_abstract/figures_src/fig3_hv4w55{,_bc}_numbers.tsv
    docs/hv_dccpp/AASRC2026_abstract/figures_src/fig3_hv4w55_bc_caption.txt（僅 bc）

唯讀：只寫上面這些檔（--bin-cache 另存 BIN 重抽結果以加速重跑，預設不存）。不開任何 COM 埠。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import pickle
import sys
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path

sys.dont_write_bytecode = True
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

import numpy as np                                                  # noqa: E402
import matplotlib                                                   # noqa: E402

matplotlib.use("Agg")
from matplotlib import font_manager as fm                           # noqa: E402
import matplotlib.pyplot as plt                                     # noqa: E402
import matplotlib.ticker as mticker                                 # noqa: E402
from matplotlib.lines import Line2D                                 # noqa: E402
from matplotlib.patches import Rectangle                            # noqa: E402
from matplotlib.text import Text                                    # noqa: E402

HERE = Path(__file__).resolve().parent
OUT_DIR = HERE.parent
ROOT = HERE.parents[3]
UP = ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "update_paper"
sys.path.insert(0, str(UP / "scripts"))
import up_lib                                                       # noqa: E402
import up_metrics                                                   # noqa: E402
import up_extract                                                   # noqa: E402

TRIAL, TRIAL_R = "HV4W55", "HV4W55R"
RUN = ROOT / "runs" / "artifacts" / TRIAL
UAVS = up_lib.UAVS
N_F, M_F = UP / "out" / "NUMBERS.json", UP / "out" / "metrics.json"
NUM, MET = up_lib.jload(N_F), up_lib.jload(M_F)
SCEN = up_lib.jload(RUN / "scenario.json")
MS = up_lib.jload(RUN / "plan" / "mission_summary.json")
CLS_NAME = {"A": "Pioneer", "B": "HERO"}


def rel(p: Path) -> str:
    return Path(p).resolve().relative_to(ROOT).as_posix()


def sha256(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


# ------------------------------------------------------------------------------------------- 顯示用數字
SHOWN: list[dict] = []          # 逐項比對表：每個出現在圖上的數字


def jget(doc: dict, path: str):
    cur = doc
    for k in path.split("."):
        cur = cur[int(k)] if isinstance(cur, list) else cur[k]
    return cur


def num(file_tag: str, path: str, nd: int, label: str, panel: str, fmt: str | None = None,
        tiebreak: tuple[float, str] | None = None) -> str:
    """讀 NUMBERS.json／metrics.json 的欄位 → 依 nd 位小數（半數進位，以 JSON 內存的十進位值為準）格式化，
    並登記到逐項比對表。回傳圖上實際印出的字串。

    ⚠ 雙重捨入：JSON 已存為 4 位小數，若第 nd+1 位起**恰好是 5**（例如 7.9265 → 3 位），真值可能在任一側，
      單憑 JSON 判不出來。此時必須給 tiebreak = (未捨入的原始值, 出處)，改以原始值半數進位；沒給就中止。"""
    doc = NUM if file_tag == "NUMBERS" else MET
    v = jget(doc, path)
    step = Decimal(1).scaleb(-nd) if nd > 0 else Decimal(1)
    d = Decimal(str(v))
    note = ""
    if (d - d.quantize(step, rounding="ROUND_DOWN")).copy_abs() == step / 2:
        if tiebreak is None:
            raise SystemExit(f"{path}={v}：{nd} 位捨入恰逢 .5，需要原始值判定（tiebreak）")
        d = Decimal(repr(float(tiebreak[0])))
        note = f"JSON 值恰為 .5，依原始值 {tiebreak[0]!r}（{tiebreak[1]}）判定"
    q = d.quantize(step, rounding=ROUND_HALF_UP)
    s = f"{q:f}" if fmt is None else fmt.format(q)
    SHOWN.append(dict(panel=panel, label=label, shown=s, file=f"{file_tag}.json", field=path, json_value=v,
                      note=note))
    return s


def txt(file_tag: str, path: str, label: str, panel: str, transform=lambda v: v) -> str:
    doc = NUM if file_tag == "NUMBERS" else MET
    v = jget(doc, path)
    s = str(transform(v))
    SHOWN.append(dict(panel=panel, label=label, shown=s, file=f"{file_tag}.json", field=path, json_value=v))
    return s


# ------------------------------------------------------------------------------------------- BIN → 陣列
def bin_arrays(args) -> dict:
    """{(trial, uav): arrays}：用 up_extract.extract() 直接讀 BIN（與建 cache 的程式同一支）。"""
    want = {}
    for trial in (TRIAL, TRIAL_R):
        lognum = up_extract.TRIALS[trial]
        for u in UAVS:
            want[(trial, u)] = ROOT / f"sitl/vtol/i{up_extract.INST[u]}/logs/{lognum:08d}.BIN"
    key = {f"{t}_{u}": [rel(p), p.stat().st_size, p.stat().st_mtime] for (t, u), p in want.items()}
    cache_f = Path(args.bin_cache) / "fig3_hv4w55_bin.pkl" if args.bin_cache else None
    if cache_f and cache_f.exists():
        blob = pickle.loads(cache_f.read_bytes())
        if blob.get("key") == key:
            print("BIN 重抽結果沿用", cache_f)
            return blob["arrays"], want
    arrays = {}
    for (trial, u), p in want.items():
        print(f"讀 BIN {rel(p)} …", flush=True)
        arrays[(trial, u)], _ = up_extract.extract(up_extract.TRIALS[trial], u)
    if cache_f:
        cache_f.parent.mkdir(parents=True, exist_ok=True)
        cache_f.write_bytes(pickle.dumps(dict(key=key, arrays=arrays)))
    return arrays, want


def deep_equal(a, b, path="") -> list[str]:
    if isinstance(a, dict) and isinstance(b, dict):
        bad = []
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                bad.append(f"{path}.{k}: 只有一邊有")
            else:
                bad += deep_equal(a[k], b[k], f"{path}.{k}")
        return bad
    if isinstance(a, float) or isinstance(b, float):
        if a is None or b is None or not math.isclose(float(a), float(b), rel_tol=0, abs_tol=1e-9):
            return [f"{path}: 重算 {a!r} ≠ JSON {b!r}"]
        return []
    return [] if a == b else [f"{path}: 重算 {a!r} ≠ JSON {b!r}"]


def verify_pipeline(arrays) -> dict:
    """BIN ↔ cache、重算 ↔ metrics.json。全部相同才回傳；任何不符 raise。"""
    report = {"bin_vs_cache": {}, "recomputed_vs_metrics": {}}
    for (trial, u), arr in arrays.items():
        cache = up_lib.load(trial, u)
        diff = [k for k in sorted(set(arr) | set(cache))
                if k not in arr or k not in cache or not np.array_equal(arr[k], cache[k])]
        report["bin_vs_cache"][f"{trial}_{u}"] = "identical" if not diff else f"DIFF {diff}"
        if diff:
            raise SystemExit(f"BIN 重抽 ≠ cache：{trial} {u} {diff}")
    # 之後所有 up_metrics 計算都吃 BIN 重抽的陣列
    up_metrics.load = lambda trial, uav: dict(arrays[(trial, uav)])
    for trial in (TRIAL, TRIAL_R):
        for part, fn in (("altitude", up_metrics.altitude), ("timing", up_metrics.timing)):
            bad = deep_equal(fn(trial), MET[trial][part], f"{trial}.{part}")
            report["recomputed_vs_metrics"][f"{trial}.{part}"] = "identical" if not bad else bad
            if bad:
                raise SystemExit("重算 ≠ metrics.json：\n  " + "\n  ".join(bad[:20]))
    lat0, lon0 = float(MS["home"]["lat_deg"]), float(MS["home"]["lon_deg"])
    tr = {u: up_metrics._track(TRIAL, u, lat0, lon0, "ecef") for u in UAVS}
    for a, b in (("A1", "B2"), ("A1", "B3"), ("B2", "B3")):
        got = up_metrics._pair_min(*tr[a], *tr[b])
        pub = MET[TRIAL]["separation"]["frames"]["ecef"]["pairs"][f"{a}-{b}"]
        bad = deep_equal(got, pub, f"{TRIAL}.separation.ecef.{a}-{b}")
        report["recomputed_vs_metrics"][f"separation.ecef.{a}-{b}"] = "identical" if not bad else bad
        if bad:
            raise SystemExit("間距重算 ≠ metrics.json：\n  " + "\n  ".join(bad))
    return report


# ------------------------------------------------------------------------------------------- 幾何
def mission_clock(trial: str) -> dict:
    """同 up_metrics.timing：t0 = 規劃延遲為 0 之機的 AUTO 進入時刻 − 該延遲（GPS 週內秒）。回傳各機 (off, t0)。"""
    ms = up_lib.jload(ROOT / "runs" / "artifacts" / trial / "plan" / "mission_summary.json")
    delays = {u: float(ms["uavs"][u]["t_takeoff_s"]) for u in UAVS}
    zero = min(delays, key=lambda u: delays[u])
    off = {u: up_lib.gps_offset(up_metrics.load(trial, u)) for u in UAVS}
    t0 = up_lib.first_auto_t(up_metrics.load(trial, zero)) + off[zero] - delays[zero]
    return {u: (off[u], t0) for u in UAVS}


def alt_series(trial: str, u: str, clock: dict):
    """POS.RelHomeAlt 原生取樣（airborne_window 內，另各多留一筆），x = 任務時間。"""
    p = up_metrics.load(trial, u)["pos"]
    t, relh = p[:, 0], p[:, 4]
    ta, tb = up_lib.airborne_window(t, relh)
    i0 = max(0, int(np.searchsorted(t, ta)) - 1)
    i1 = min(len(t) - 1, int(np.searchsorted(t, tb)) + 1)
    off, t0 = clock[u]
    return t[i0:i1 + 1] + off - t0, relh[i0:i1 + 1]


def pair_series(ta, xa, ya, za, tb, xb, yb, zb):
    """與 up_metrics._pair_min **同一套**時刻聯集與線性內插（逐行對照 up_metrics.py:135-145），回傳
    兩機皆在空（>2 m）的逐點 (水平, 垂直) 間距，並以 NaN 斷開不在空的段落（作圖用）。"""
    lo, hi = max(ta[0], tb[0]), min(ta[-1], tb[-1])
    if hi <= lo:
        return None
    tu = np.unique(np.concatenate([[lo], ta[(ta >= lo) & (ta <= hi)], tb[(tb >= lo) & (tb <= hi)], [hi]]))
    a = [np.interp(tu, ta, v) for v in (xa, ya, za)]
    b = [np.interp(tu, tb, v) for v in (xb, yb, zb)]
    air = (a[2] > up_lib.AIRBORNE_M) & (b[2] > up_lib.AIRBORNE_M)
    if air.sum() < 2:
        return None
    h = np.hypot(b[0] - a[0], b[1] - a[1])
    v = np.abs(b[2] - a[2])
    h[~air], v[~air] = np.nan, np.nan
    return h, v, int(air.sum())


# ------------------------------------------------------------------------------------------- 字型與樣式
KAIU, TIMES = Path("C:/Windows/Fonts/kaiu.ttf"), Path("C:/Windows/Fonts/times.ttf")
for f in (KAIU, TIMES):
    fm.fontManager.addfont(str(f))
KAIU_NAME = fm.FontProperties(fname=str(KAIU)).get_name()
TIMES_NAME = fm.FontProperties(fname=str(TIMES)).get_name()
FS = 9.0                                   # 圖內最小字級（使用者要求 ≥ 9 pt）
matplotlib.rcParams.update({
    "font.family": [TIMES_NAME, KAIU_NAME], "font.size": FS,
    "axes.titlesize": FS, "axes.labelsize": FS, "xtick.labelsize": FS, "ytick.labelsize": FS,
    "legend.fontsize": FS, "legend.title_fontsize": FS,
    "axes.unicode_minus": True, "axes.linewidth": 0.6,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.major.size": 2.5, "ytick.major.size": 2.5,
    "xtick.minor.size": 1.5, "ytick.minor.size": 1.5, "xtick.minor.width": 0.5, "ytick.minor.width": 0.5,
    "xtick.direction": "in", "ytick.direction": "in",
    "svg.fonttype": "path", "svg.hashsalt": "fig3_hv4w55", "pdf.fonttype": 42,
})
CM = 1 / 2.54
STYLE = {                                   # 與舊版圖3 相同：Pioneer 暖色、HERO 冷色；黑白列印靠線型
    "A1": {"color": "#B5470B", "ls": "-"},
    "B2": {"color": "#1B4F91", "ls": (0, (3.2, 1.4))},
    "B3": {"color": "#127A5C", "ls": (0, (5.0, 1.3, 1.1, 1.3))},
}
R_COLOR = "0.55"                            # HV4W55R 對照：單一中性灰，不借用各機身分色
R_NAME = "對照架次"                          # HV4W55R 在圖上的顯示名（資料鍵仍是 TRIAL_R）
PANEL_TAG = {"abc": {"b": "b", "c": "c"}, "bc": {"b": "b", "c": "c"}, "3col": {"b": "b", "c": "c"},
             "wide": {"b": "a", "c": "b"}}   # wide 版只有兩格，編號改為 (a)(b)


def fmt_fixed(v: float, nd: int) -> str:
    return f"{Decimal(str(v)).quantize(Decimal(1).scaleb(-nd), ROUND_HALF_UP):f}"


# ------------------------------------------------------------------------------------------- (a) 平面航跡
def panel_a(axA, ctx, layout: str = "abc"):
    lat0, lon0 = ctx["lat0"], ctx["lon0"]
    poly = SCEN["areas"][0]["polygon"]
    pe, pn = up_lib.enu([p[0] for p in poly] + [poly[0][0]], [p[1] for p in poly] + [poly[0][1]], lat0, lon0)
    axA.plot(pe / 1000, pn / 1000, color="0.25", lw=0.7, ls=(0, (2.0, 1.2)), zorder=1)
    all_e, all_n = [], []
    for u in UAVS:
        wp = [w for w in up_lib.read_waypoints(RUN / "plan" / f"{u}.waypoints") if w["lat"] != 0.0]
        e, n = up_lib.enu([w["lat"] for w in wp], [w["lon"] for w in wp], lat0, lon0)
        axA.plot(e / 1000, n / 1000, color=STYLE[u]["color"], lw=2.6, alpha=0.22, solid_capstyle="round",
                 solid_joinstyle="round", zorder=2)
    for u in UAVS:
        p = up_metrics.load(TRIAL, u)["pos"]
        ta, tb = up_lib.airborne_window(p[:, 0], p[:, 4])
        m = (p[:, 0] >= ta) & (p[:, 0] <= tb)
        e, n = up_lib.enu(p[m, 1], p[m, 2], lat0, lon0)
        axA.plot(e / 1000, n / 1000, color=STYLE[u]["color"], lw=0.7, ls=STYLE[u]["ls"], zorder=3)
        all_e += list(e / 1000)
        all_n += list(n / 1000)
    axA.plot([0], [0], marker="s", ms=3.8, mfc="white", mec="black", mew=0.8, ls="none", zorder=5)
    if layout == "3col":
        return _panel_a_3col(axA, pe / 1000, pn / 1000, all_e, all_n, ctx)
    axA.annotate("H 起降點", (0, 0), xytext=(min(all_e) + 0.01, min(all_n) - 0.12), ha="left", va="top",
                 zorder=6, arrowprops=dict(arrowstyle="-", lw=0.5, color="black", shrinkA=1, shrinkB=3))
    axA.text(max(pe) / 1000, min(all_n) - 0.015, "作業區（虛線框）", ha="right", va="top", color="0.15",
             zorder=6)
    # 風：規劃風＝模擬風（NUMBERS.json plan_wind、sim_wind；不一致即中止）。箭頭指向風吹往的方向。
    spd = num("NUMBERS", f"trials.{TRIAL}.sim_wind.spd_mps", 0, "模擬風速 (m/s)", "a")
    dfr = num("NUMBERS", f"trials.{TRIAL}.sim_wind.dir_from_deg", 0, "模擬風向（來向，°）", "a")
    pw = NUM["trials"][TRIAL]["plan_wind"]
    sw = NUM["trials"][TRIAL]["sim_wind"]
    assert [float(pw[0]), float(pw[1])] == [float(sw["spd_mps"]), float(sw["dir_from_deg"])], (pw, sw)
    to = math.radians(float(sw["dir_from_deg"]) + 180.0)          # 羅盤角：0 = 北、90 = 東
    ux, uy = math.sin(to), math.cos(to)
    wy = min(all_n) - 0.93
    tw = axA.text(min(all_e) + 0.01, wy, f"風 {spd} m/s（自 {dfr}°；規劃＝模擬）", ha="left", va="center",
                  zorder=6)
    axA.figure.canvas.draw()
    bb = tw.get_window_extent().transformed(axA.transData.inverted())
    ax0 = bb.x1 + 0.08
    axA.annotate("", xy=(ax0 + 0.14 + 0.14 * ux, wy + 0.14 * uy), xytext=(ax0 + 0.14 - 0.14 * ux, wy - 0.14 * uy),
                 arrowprops=dict(arrowstyle="-|>", lw=1.0, color="black", mutation_scale=8), zorder=6)
    hands = []
    for u in UAVS:
        cls = MET[TRIAL]["timing"]["per_uav"][u]["cls"]
        # plan_km 是 up_metrics.timing 由 plan/mission_summary.json uavs.<u>.distance_m ÷ 1000 捨入到 4 位而來
        pk = num("metrics", f"{TRIAL}.timing.per_uav.{u}.plan_km", 3, f"{u} 規劃航程 (km)", "a",
                 tiebreak=(float(MS["uavs"][u]["distance_m"]) / 1000.0,
                           f"runs/artifacts/{TRIAL}/plan/mission_summary.json uavs.{u}.distance_m ÷ 1000"))
        fk = num("metrics", f"{TRIAL}.timing.per_uav.{u}.flown_km", 3, f"{u} 飛行航程 (km)", "a")
        hands.append(Line2D([], [], color=STYLE[u]["color"], lw=1.0, ls=STYLE[u]["ls"],
                            label=f"{u} {CLS_NAME[cls]}  {pk} → {fk} km"))
    lgA = axA.legend(handles=hands, loc="lower right", bbox_to_anchor=(1.0, 0.115), frameon=False,
                     handlelength=2.3, handletextpad=0.45, borderaxespad=0.1, labelspacing=0.2,
                     title="各機航程（規劃 → 飛行）")
    lgA._legend_box.align = "left"
    axA.set_xlim(min(all_e) - 0.05, max(all_e) + 0.05)
    axA.set_ylim(min(all_n) - 1.02, max(all_n) + 0.06)
    axA.set_aspect("equal", adjustable="box")
    axA.set_yticks([t for t in np.arange(-1.0, 1.01, 0.25) if min(all_n) - 0.05 <= t <= max(all_n) + 0.05])
    axA.set_xlabel("東 E (km)", labelpad=1.5)
    axA.set_ylabel("北 N (km)", labelpad=1.5)
    axA.set_title("(a) 平面航跡（寬淡線：規劃；細線：飛行）", loc="left", pad=3)


def _axes_cm(ax) -> tuple[float, float]:
    """軸框實際寬、高（cm）。"""
    pos, fig = ax.get_position(), ax.figure
    return pos.width * fig.get_figwidth() * 2.54, pos.height * fig.get_figheight() * 2.54


def _panel_a_3col(axA, pe, pn, all_e, all_n, ctx):
    """3col 版 (a)：各機航程移到頂列、不畫風；軸框維持整欄高度（等比例 ⇒ N 範圍由框的長寬比決定）。
    由上而下：「作業區（虛線框）」→ 航跡 → H 起降點標註 → 線型圖例（寬淡線＝規劃、細線＝飛行），整組垂直置中。"""
    w_cm, h_cm = _axes_cm(axA)
    e0, e1 = min(all_e) - 0.05, max(all_e) + 0.05
    s = w_cm / (e1 - e0)                                  # cm／km
    box_top, n_lo = float(max(pn)), min(all_n)
    top, bot = box_top + 0.40 / s, n_lo - 0.80 / s        # 內容上下緣：上一行標籤、下兩行（H 標註＋圖例）
    slack = (e1 - e0) * h_cm / w_cm - (top - bot)
    n1 = top + slack / 2
    n0 = n1 - (e1 - e0) * h_cm / w_cm
    axA.set_xlim(e0, e1)
    axA.set_ylim(n0, n1)
    axA.set_aspect("equal", adjustable="box")
    axA.text(float(max(pe)), box_top + 0.06 / s, "作業區（虛線框）", ha="right", va="bottom", color="0.15",
             zorder=6)
    axA.annotate("H 起降點", (0, 0), xytext=(min(all_e) + 0.02, n_lo - 0.06 / s), ha="left", va="top",
                 zorder=6, arrowprops=dict(arrowstyle="-", lw=0.5, color="black", shrinkA=1, shrinkB=3))
    hands = [Line2D([], [], color="0.45", lw=2.6, alpha=0.35, solid_capstyle="butt", label="規劃"),
             Line2D([], [], color="0.15", lw=0.7, label="飛行")]
    axA.legend(handles=hands, loc="upper left", bbox_to_anchor=(min(all_e) - 0.02, n_lo - 0.42 / s),
               bbox_transform=axA.transData, ncol=2, frameon=False, handlelength=1.6, handletextpad=0.35,
               columnspacing=0.9, borderaxespad=0.0, borderpad=0.0)
    axA.set_yticks([t for t in (-0.4, 0.0, 0.4) if n_lo - 0.05 <= t <= max(all_n) + 0.05])
    axA.set_xticks([-1.0, 0.0, 1.0])
    axA.set_xticks([-0.5, 0.5], minor=True)
    axA.xaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:.1f}".replace("-", "\N{MINUS SIGN}")))
    axA.set_xlabel("東 E (km)", labelpad=1.5)
    axA.set_ylabel("北 N (km)", labelpad=1.5)
    # 縱軸標題對齊有刻度的航跡段（而非整個框）；x 等畫完後沿用自動位置（main 裡處理）
    ctx.setdefault("ylabel_y", []).append((axA, (0.0 - n0) / (n1 - n0)))
    axA.set_title("(a) 平面航跡", loc="left", pad=3)


def header_3col(fig, x0: float, x1: float, yc: float):
    """3col 版頂列（cm 座標）：「各機航程（規劃→飛行）」＋三機線型與 規劃→飛行 航程，左右撐滿 x0–x1。
    數字與 abc 版 (a) 圖例同一來源（metrics.json plan_km／flown_km，3 位；7.9265 依原始 distance_m 判定）。"""
    w_cm, h_cm = fig.get_figwidth() * 2.54, fig.get_figheight() * 2.54
    r = fig.canvas.get_renderer()
    title = "各機航程（規劃→飛行）"
    ents = []
    for u in UAVS:
        cls = MET[TRIAL]["timing"]["per_uav"][u]["cls"]
        pk = num("metrics", f"{TRIAL}.timing.per_uav.{u}.plan_km", 3, f"{u} 規劃航程 (km)", "頂列",
                 tiebreak=(float(MS["uavs"][u]["distance_m"]) / 1000.0,
                           f"runs/artifacts/{TRIAL}/plan/mission_summary.json uavs.{u}.distance_m ÷ 1000"))
        fk = num("metrics", f"{TRIAL}.timing.per_uav.{u}.flown_km", 3, f"{u} 飛行航程 (km)", "頂列")
        ents.append((u, f"{u} {CLS_NAME[cls]} {pk}→{fk} km"))

    def width(s):
        t = fig.text(0, 0, s)
        w = t.get_window_extent(r).width / fig.dpi * 2.54
        t.remove()
        return w

    HL, PAD = 0.32, 0.06                                  # 線段長、線段到字距（cm）；點劃線一個週期約 0.31 cm
    wt, we = width(title), [width(s) for _, s in ents]
    gap = (x1 - x0 - wt - sum(we) - 3 * (HL + PAD)) / 3   # 標題→A1、A1→B2、B2→B3 三個間隙均分剩餘寬度
    if gap < 0.15:
        raise SystemExit(f"頂列放不下：間隙只剩 {gap:.3f} cm")
    x = x0
    fig.text(x / w_cm, yc / h_cm, title, ha="left", va="center")
    x += wt + gap
    for (u, s), w in zip(ents, we):
        fig.add_artist(Line2D([x / w_cm, (x + HL) / w_cm], [yc / h_cm] * 2, transform=fig.transFigure,
                              color=STYLE[u]["color"], ls=STYLE[u]["ls"], lw=1.0))
        x += HL + PAD
        fig.text(x / w_cm, yc / h_cm, s, ha="left", va="center")
        x += w + gap


# ------------------------------------------------------------------------------------------- (b) 高度剖面
def panel_b(axB, ctx, layout: str):
    """主圖只畫航段輪廓（**不畫**參考線、不標 120／119.5 刻度——兩線相距 0.5 m，在 0–125 m 的主圖上會黏成
    一條，看起來像「貼齊 120 m」）；參考線與刻度全部移到 119.4–120.3 m 放大內插框。"""
    ceil_ops, h_ceil = ctx["ceil_ops"], ctx["h_ceil"]
    axS = ctx.get("axS")                         # wide：圖例／比較框／表格放在 (b) 右側的文字欄
    if layout == "wide":
        F_DATA, F_TAB1, F_INS0, F_INS1 = 0.470, None, 0.525, 0.985
        INS_X0, INS_X1, SIDE_X = 0.175, 0.985, None
        cols_x, row_h = None, None
    elif layout == "bc":
        F_DATA, F_TAB1, F_INS0, F_INS1 = 0.335, 0.585, 0.615, 0.985
        INS_X0, INS_X1, SIDE_X = 0.125, 0.555, 0.578
        cols_x, row_h = (0.66, 0.825, 0.985), 0.047
    elif layout == "3col":
        # 以 cm 配置（沒有側欄）：頂部兩列圖例 → 放大框（左）＋逾越註記（右）→ 剖面
        # note_w：註記框外寬（最長一行「逾 120.000 m 累計：」2.80 cm＋左右 pad）；放大框右緣再空 0.18 cm
        b_w, b_h = _axes_cm(axB)
        B3 = dict(w=b_w, h=b_h, leg_h=0.80, band_h=1.14, note_w=2.98, note_r=0.04)
        F_INS1 = 1 - (B3["leg_h"] + 0.04) / b_h
        F_INS0 = F_INS1 - B3["band_h"] / b_h
        F_DATA = F_INS0 - 0.16 / b_h
        F_TAB1 = SIDE_X = cols_x = row_h = None
        INS_X0, INS_X1 = 0.80 / b_w, 1 - (B3["note_r"] + B3["note_w"] + 0.18) / b_w
    else:
        F_DATA, F_TAB1, F_INS0, F_INS1 = 0.330, 0.555, 0.590, 0.985
        INS_X0, INS_X1, SIDE_X = 0.135, 0.470, 0.488
        cols_x, row_h = (0.66, 0.825, 0.985), 0.0335
    Y_LO, Y_DATA = -8.0, 125.0
    y_top = Y_LO + (Y_DATA - Y_LO) / F_DATA
    series = {u: alt_series(TRIAL, u, ctx["clock"]) for u in UAVS}
    series_r = {u: alt_series(TRIAL_R, u, ctx["clock_r"]) for u in UAVS}
    for u in UAVS:
        t, a = series[u]
        axB.plot(t, a, color=STYLE[u]["color"], lw=0.75, ls=STYLE[u]["ls"], zorder=3)
    for u in UAVS:
        tt = MET[TRIAL]["timing"]["per_uav"][u]["plan_t_takeoff_s"]
        axB.plot([tt], [0], marker="^", ms=4.4, mfc="white", mec=STYLE[u]["color"], mew=0.9, ls="none",
                 zorder=5, clip_on=False)
    tmax = max(series[u][0][-1] for u in UAVS)
    xl = (-40, tmax + 40)
    axB.set_xlim(xl)
    axB.set_ylim(Y_LO, y_top)
    axB.set_yticks([0, 50, 100])
    axB.set_ylabel("相對高度 (m)", labelpad=1.5)
    if layout in ("wide", "3col"):
        axB.set_xticks([0, 400, 800, 1200])
        axB.set_xlabel("任務時間 (s)；△ 規劃放行", labelpad=1.5)
        if layout == "wide":
            axB.yaxis.set_label_coords(-0.135, F_DATA / 2)
            axB.axhline(Y_LO + (y_top - Y_LO) * (F_DATA + 0.015), color="0.6", lw=0.5, zorder=1)
        else:
            ctx.setdefault("ylabel_y", []).append((axB, F_DATA / 2))
            axB.axhline(Y_LO + (y_top - Y_LO) * (F_DATA + 0.07 / B3["h"]), color="0.6", lw=0.5, zorder=1)
    else:
        axB.set_xlabel("任務時間 (s)；△：規劃放行時刻", labelpad=1.5)
        axB.yaxis.set_label_coords(-0.075 if layout == "bc" else -0.105, F_DATA / 2)
        axB.axhline(Y_LO + (y_top - Y_LO) * (F_DATA + 0.010), color="0.6", lw=0.5, zorder=1)  # 剖面／表格分隔
    axB.set_title("(b) 高度剖面 AGL" if layout == "3col" else
                  f"({PANEL_TAG[layout]['b']}) 高度剖面（POS.RelHomeAlt 原生取樣）", loc="left", pad=3)

    # ---- 放大內插框 119.4–120.3 m：兩條參考線與刻度只在這裡
    ins = axB.inset_axes([INS_X0, F_INS0, INS_X1 - INS_X0, F_INS1 - F_INS0])
    for u in UAVS:
        t, a = series_r[u]
        ins.plot(t, a, color=R_COLOR, lw=0.6, ls=STYLE[u]["ls"], zorder=2)
    for u in UAVS:
        t, a = series[u]
        ins.plot(t, a, color=STYLE[u]["color"], lw=0.7, ls=STYLE[u]["ls"], zorder=3)
    ins.axhline(ceil_ops, color="black", lw=0.8, zorder=4)
    ins.axhline(h_ceil, color="black", lw=0.8, ls=(0, (3.0, 2.0)), zorder=4)
    if layout == "3col":
        # 放大框較窄 ⇒ 只取有軌跡落在 119.4–120.3 m 的時段（兩架次聯集，前後各留 25 s）
        tin = [t[(a >= 119.4) & (a <= 120.3)] for s in (series, series_r) for t, a in s.values()]
        ins.set_xlim(min(float(x.min()) for x in tin) - 25, max(float(x.max()) for x in tin) + 25)
        ins.set_xticks([400, 800])
    else:
        ins.set_xlim(xl)
    ins.set_ylim(119.4, 120.3)
    ins.set_yticks([119.4, 119.5, 119.6, 119.7, 119.8, 119.9, 120.0, 120.1, 120.2, 120.3])
    ins.yaxis.set_major_formatter(mticker.FuncFormatter(
        lambda v, _: f"{v:.1f}" if round(v * 10) % 5 == 0 else ""))
    ins.set_xticklabels([])
    ins.tick_params(axis="x", length=2)
    for sp in ins.spines.values():
        sp.set_linewidth(0.7)
    s_ops = fmt_fixed(ceil_ops, 3)
    SHOWN.append(dict(panel="b", label="作業上限線 (m)", shown=s_ops, file="NUMBERS.json",
                      field=f"trials.{TRIAL}.altitude.flown_max_agl_m + margin_to_120_m",
                      json_value=ceil_ops, note="＝ up_lib.CEIL_OPS_M（已斷言相等）"))
    s_hc = num("NUMBERS", f"trials.{TRIAL}.ceiling_m", 1, "規劃上限線 (m)", "b")
    s_r_ceil = num("NUMBERS", f"trials.{TRIAL_R}.ceiling_m", 1, f"{R_NAME}規劃上限 (m)", "b")
    # 本架次 vs HV4W55R 逾越 120.000 m 累計時間（NUMBERS.json altitude.t_above_120_s；354.925 捨到 2 位恰為 .5，
    # 以未捨入的逐機 time_above 加總判定——與 up_metrics.altitude 同一函式，只是不先捨入）
    raw_r = 0.0
    for u in UAVS:
        p = up_metrics.load(TRIAL_R, u)["pos"]
        ta, tb = up_lib.airborne_window(p[:, 0], p[:, 4])
        m = (p[:, 0] >= ta) & (p[:, 0] <= tb)
        raw_r += up_lib.time_above(p[m, 0], p[m, 4], up_lib.CEIL_OPS_M)
    s_t_self = num("NUMBERS", f"trials.{TRIAL}.altitude.t_above_120_s", 2, f"本架次逾越 {s_ops} m 累計 (s)", "b")
    s_t_r = num("NUMBERS", f"trials.{TRIAL_R}.altitude.t_above_120_s", 2, f"{R_NAME}逾越 {s_ops} m 累計 (s)", "b",
                tiebreak=(raw_r, "up_lib.time_above 逐機未捨入加總（BIN 重抽，與 up_metrics.altitude 同一函式）"))
    leg_h = [Line2D([], [], color="black", lw=0.8, label=f"作業上限 {s_ops} m"),
             Line2D([], [], color="black", lw=0.8, ls=(0, (3.0, 2.0)), label=f"規劃上限 {s_hc} m"),
             Line2D([], [], color=R_COLOR, lw=0.9, label=f"{R_NAME}（規劃上限 {s_r_ceil} m）")]
    cmp_txt = f"逾越 {s_ops} m 累計\n{R_NAME}：{s_t_r} s\n本架次：{s_t_self} s"
    if layout == "wide":
        return _side_column_wide(axS, leg_h, cmp_txt, ins)
    if layout == "3col":
        # 註記用字依使用者指定：「逾 120.000 m 累計：本架次 0.00 s／對照架次 354.93 s」（框窄，斷成三行、字不變）
        note = f"逾 {s_ops} m 累計：\n本架次 {s_t_self} s／\n{R_NAME} {s_t_r} s"
        kw = dict(loc="upper left", frameon=False, handlelength=1.3, handletextpad=0.35, borderaxespad=0.12,
                  borderpad=0.1)
        lg1 = axB.legend(handles=leg_h[:2], bbox_to_anchor=(0.0, 1.0), ncol=2, columnspacing=0.8, **kw)
        axB.add_artist(lg1)
        fig = axB.figure
        fig.canvas.draw()
        y1 = lg1.get_window_extent().transformed(axB.transAxes.inverted()).y0
        lg2 = axB.legend(handles=leg_h[2:], bbox_to_anchor=(0.0, y1 + 0.012), **kw)
        pad = 0.25 * FS / 72 * 2.54                     # bbox pad（字級倍數）換成 cm
        tn = axB.text(1 - (B3["note_r"] + pad) / B3["w"], F_INS1 - pad / B3["h"], note, transform=axB.transAxes,
                      ha="right", va="top", multialignment="left", linespacing=1.05,
                      bbox=dict(boxstyle="square,pad=0.25", fc="white", ec="black", lw=0.6))
        # 版面自檢：註記框不得碰到放大框或第二列圖例、不得超出 (b) 框
        fig.canvas.draw()
        to_ax = axB.transAxes.inverted()
        nb = tn.get_bbox_patch().get_window_extent().transformed(to_ax)
        ib = ins.get_tightbbox().transformed(to_ax)
        lb = lg2.get_window_extent().transformed(to_ax)
        assert nb.x0 > ib.x1 and nb.y1 < lb.y0 and nb.x1 < 1 and nb.y0 > F_DATA, (nb, ib, lb)
        assert all(v == 0.0 for v in NUM["trials"][TRIAL]["altitude"]["t_above_120_per_uav"].values())
        return ins
    if True:                                     # 兩種版面都用右側欄：圖例 + 比較框
        leg_h[2].set_label(f"{R_NAME}\n（規劃上限 {s_r_ceil} m）")
        axB.legend(handles=leg_h, loc="upper left", bbox_to_anchor=(SIDE_X, F_INS1 + 0.004), ncol=1,
                   frameon=False, handlelength=1.6 if layout == "bc" else 1.3, handletextpad=0.35,
                   borderaxespad=0.0, labelspacing=0.15,
                   title=("放大框 119.4–120.3 m" if layout == "bc" else "放大框\n119.4–120.3 m"),
                   alignment="left")
        axB.text(SIDE_X + 0.008, F_INS0 + 0.010, cmp_txt, transform=axB.transAxes, ha="left", va="bottom",
                 linespacing=1.2,
                 bbox=dict(boxstyle="square,pad=0.3", fc="white", ec="black", lw=0.6))

    # ---- 逐機數值表
    rows_y = [F_TAB1 - 0.006 - k * row_h for k in range(5)]
    for u, cx in zip(UAVS, cols_x):
        axB.plot([cx - 0.135, cx - 0.075], [rows_y[0] - row_h * 0.35] * 2, transform=axB.transAxes,
                 color=STYLE[u]["color"], ls=STYLE[u]["ls"], lw=1.0, clip_on=False)
        axB.text(cx, rows_y[0], u, transform=axB.transAxes, ha="right", va="top")
    for k, lab in enumerate(["實飛最高 (m)", "匯出最高 (m)", "規劃放行時刻 (s)", f"{R_NAME}實飛最高 (m)"], start=1):
        axB.text(0.02, rows_y[k], lab, transform=axB.transAxes, ha="left", va="top")
    for u, cx in zip(UAVS, cols_x):
        vals = [
            num("NUMBERS", f"trials.{TRIAL}.altitude.flown_max_agl_per_uav.{u}", 3, f"{u} 實飛最高 (m)", "b"),
            num("metrics", f"{TRIAL}.altitude.per_uav.{u}.exported_max_rel_m", 3, f"{u} 匯出最高 (m)", "b"),
            num("metrics", f"{TRIAL}.timing.per_uav.{u}.plan_t_takeoff_s", 0, f"{u} 放行時刻 (s)", "b"),
            num("NUMBERS", f"trials.{TRIAL_R}.altitude.flown_max_agl_per_uav.{u}", 3,
                f"{R_NAME} {u} 實飛最高 (m)", "b"),
        ]
        for k, v in enumerate(vals, start=1):
            axB.text(cx, rows_y[k], v, transform=axB.transAxes, ha="right", va="top")
    # 逐機逾越 120.000 m 時間全為 0（本架次累計 0.00 s 即其總和）——斷言，不另佔一列
    assert all(v == 0.0 for v in NUM["trials"][TRIAL]["altitude"]["t_above_120_per_uav"].values())
    return ins


# ------------------------------------------------------------------------------------------- (c) 機間間距
def panel_c(axC, ctx, layout: str):
    lat0, lon0, d_sep, dh_sep = ctx["lat0"], ctx["lon0"], ctx["d_sep"], ctx["dh_sep"]
    tr = {u: up_metrics._track(TRIAL, u, lat0, lon0, "ecef") for u in UAVS}
    PAIR_STYLE = {"A1-B3": dict(color="0.60", lw=0.7, ls="-"),
                  "B2-B3": dict(color="0.15", lw=0.7, ls=(0, (2.6, 1.2)))}
    pair_data = {f"{a}-{b}": pair_series(*tr[a], *tr[b]) for a, b in (("A1", "B2"), ("A1", "B3"), ("B2", "B3"))}
    assert pair_data["A1-B2"] is None, "A1–B2 應從未同時在空中（metrics.json 如此記載）"
    hmax = max(float(np.nanmax(v[0])) for v in pair_data.values() if v)
    vmax = max(float(np.nanmax(v[1])) for v in pair_data.values() if v)
    X_LO, X_HI = d_sep / 2, hmax * 1.25            # 對數軸：左緣只是軸下限，違規區實際自 0 m 起
    axC.set_xscale("log")
    axC.add_patch(Rectangle((X_LO, 0), d_sep - X_LO, dh_sep, fc="0.90", ec="black", lw=0.6, hatch="////",
                            zorder=1))
    s_dsep = num("metrics", f"{TRIAL}.separation.reporter_raw.thresholds.d_sep_m", 0, "違規門檻 水平 (m)", "c")
    s_dhsep = num("metrics", f"{TRIAL}.separation.reporter_raw.thresholds.dh_sep_m", 0, "違規門檻 垂直 (m)", "c")
    handles_c = []
    for key, st in PAIR_STYLE.items():
        h, v, _n = pair_data[key]
        axC.plot(h, v, zorder=3, **st)
        handles_c.append(Line2D([], [], label=key.replace("-", "–"), **{**st, "lw": 1.0}))
    sep = NUM["trials"][TRIAL]["separation"]
    s_min = num("NUMBERS", f"trials.{TRIAL}.separation.flown_exact_m", 3, "實飛最小水平間距 (m)", "c")
    s_pair = txt("NUMBERS", f"trials.{TRIAL}.separation.binding_pair", "最小間距機對", "c",
                 lambda v: v.replace("-", "–"))
    s_vert = num("NUMBERS", f"trials.{TRIAL}.separation.vert_at_min_m", 3, "最小點垂直差 (m)", "c")
    # 分子分母同一範圍：B2–B3 這一對自己的違規數／取樣數（全機隊 n_violations 亦為 0）
    assert NUM["trials"][TRIAL]["separation"]["n_violations"] == 0
    s_nv = num("metrics", f"{TRIAL}.separation.frames.ecef.pairs.B2-B3.n_violation_intervals", 0,
               "B2–B3 違規數", "c")
    s_ns = num("metrics", f"{TRIAL}.separation.frames.ecef.pairs.B2-B3.n_samples", 0, "B2–B3 對取樣數", "c")
    axC.plot([sep["flown_exact_m"]], [sep["vert_at_min_m"]], marker="o", ms=4.4, mfc="black", mec="white",
             mew=0.6, ls="none", zorder=6)
    last = f"{s_nv}／{s_ns} 對取樣違反門檻" if layout == "3col" else f"違規 {s_nv}／{s_ns} 對取樣"
    axC.annotate(f"最小 {s_min} m（{s_pair}）\n該時刻垂直差 {s_vert} m\n{last}",
                 (sep["flown_exact_m"], sep["vert_at_min_m"]),
                 xytext=(0.015 if layout == "3col" else 0.03, 0.975), textcoords="axes fraction",
                 ha="left", va="top", zorder=7, linespacing=1.08 if layout == "3col" else 1.2,
                 arrowprops=dict(arrowstyle="-", lw=0.6, color="black", shrinkA=1, shrinkB=2.5,
                                 relpos=(0.06, 0.0)),
                 bbox=dict(boxstyle="square,pad=0.12", fc="white", ec="none", alpha=0.95))
    axC.set_xlim(X_LO, X_HI)
    # 3col 欄窄：100／200／500／1000／2000 的字會黏在一起 ⇒ 只標 100／300／1000
    ticks_c = (100, 300, 1000) if layout == "3col" else (100, 200, 500, 1000, 2000)
    axC.set_xticks([t for t in ticks_c if X_LO < t <= X_HI])
    axC.xaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:g}"))
    axC.xaxis.set_minor_formatter(mticker.NullFormatter())
    axC.set_ylabel("垂直間距 (m)", labelpad=1.5)
    axC.set_yticks([0, 50, 100])
    if layout == "3col":
        c_w, c_h = _axes_cm(axC)
        TOP = 1.42                                       # 頂部保留（cm）：三行註記＋一列機對圖例
        axC.set_ylim(0, vmax / (1 - TOP / c_h))
        axC.legend(handles=handles_c, loc="upper right", bbox_to_anchor=(1.0, 1 - 1.06 / c_h), ncol=2,
                   frameon=True, fancybox=False, framealpha=0.95, edgecolor="none", handlelength=1.4,
                   handletextpad=0.3, columnspacing=0.6, borderaxespad=0.0, borderpad=0.1)
        axC.text(math.sqrt(X_LO * d_sep), dh_sep / 2, "違規區", rotation=90, ha="center", va="center",
                 zorder=6, bbox=dict(boxstyle="square,pad=0.03", fc="white", ec="none"))
        axC.set_xlabel("水平間距 (m，對數軸)", labelpad=1.5)
        axC.set_title("(c) 同時在空中的機間間距", loc="left", pad=3)
        return
    if layout == "wide":
        axC.set_ylim(0, vmax + 68)
        # 機對圖例一行放在標註下方（右靠，避開引線）；A1–B2 與違規區定義寫進圖說
        lgC = axC.legend(handles=handles_c, loc="upper right", bbox_to_anchor=(0.995, 0.735), ncol=2,
                         frameon=True, fancybox=False, framealpha=0.95, edgecolor="none", handlelength=1.8,
                         handletextpad=0.35, columnspacing=0.8, borderaxespad=0.0, borderpad=0.15)
        axC.text(math.sqrt(X_LO * d_sep), dh_sep / 2, "違規區", rotation=90, ha="center", va="center",
                 zorder=6, bbox=dict(boxstyle="square,pad=0.08", fc="white", ec="none"))
        axC.set_xlabel("水平間距 (m，對數軸)", labelpad=1.5)
        axC.set_title(f"({PANEL_TAG[layout]['c']}) 同時在空中的機間間距", loc="left", pad=3)
        return
    if layout == "bc":
        handles_c.append(Line2D([], [], ls="none", label="A1–B2：未同時在空中"))
        handles_c.append(Rectangle((0, 0), 1, 1, fc="0.90", ec="black", lw=0.6, hatch="////",
                                   label=f"違規區：水平 < {s_dsep} m\n且垂直 < {s_dhsep} m"))
        lgC = axC.legend(handles=handles_c, loc="upper right", bbox_to_anchor=(0.995, 0.80), frameon=True,
                         fancybox=False, framealpha=0.95, edgecolor="none", handlelength=2.0,
                         borderaxespad=0.0, labelspacing=0.2)
        lgC._legend_box.align = "left"
        axC.set_ylim(0, vmax + 118)
        axC.set_xlabel("水平間距 (m，對數軸)", labelpad=1.5)
        axC.set_title("(c) 同時在空中的機間間距", loc="left", pad=3)
    else:
        axC.legend(handles=handles_c, loc="upper right", frameon=True, fancybox=False, framealpha=0.95,
                   edgecolor="none", handlelength=2.2, borderaxespad=0.25, labelspacing=0.2)
        axC.set_ylim(0, vmax + 62)
        axC.text(X_LO * 1.05, dh_sep + 3, "違規區", ha="left", va="bottom", zorder=6)
        axC.set_xlabel(f"水平間距 (m，對數軸)；違規＝水平 < {s_dsep} m 且垂直 < {s_dhsep} m", labelpad=1.5)
        axC.set_title("(c) 同時在空中的機間間距（A1–B2 從未同時在空中）", loc="left", pad=3)


def _side_column_wide(axS, leg_h, cmp_txt, ins):
    """wide 版 (b) 右側文字欄：放大框圖例 → 逾越累計比較框 → 逐機數值表（上到下）。"""
    axS.axis("off")
    axS.set_xlim(0, 1)
    axS.set_ylim(0, 1)
    lg = axS.legend(handles=leg_h, loc="upper left", bbox_to_anchor=(0.0, 1.0), ncol=1, frameon=False,
                    handlelength=1.6, handletextpad=0.35, borderaxespad=0.0, labelspacing=0.1,
                    title="放大框 119.4–120.3 m", alignment="left")
    axS.figure.canvas.draw()
    lg_bb = lg.get_window_extent().transformed(axS.transAxes.inverted())
    tb = axS.text(0.02, lg_bb.y0 - 0.035, cmp_txt, transform=axS.transAxes, ha="left", va="top",
                  linespacing=1.12, bbox=dict(boxstyle="square,pad=0.28", fc="white", ec="black", lw=0.6))
    axS.figure.canvas.draw()
    tb_bb = tb.get_bbox_patch().get_window_extent().transformed(axS.transAxes.inverted())
    cols_x = (0.535, 0.7675, 1.00)
    n_rows = 5
    top = tb_bb.y0 - 0.03
    row_h = top / n_rows
    rows_y = [top - k * row_h for k in range(n_rows)]
    for u, cx in zip(UAVS, cols_x):
        axS.plot([cx - 0.215, cx - 0.105], [rows_y[0] - row_h * 0.42] * 2, transform=axS.transAxes,
                 color=STYLE[u]["color"], ls=STYLE[u]["ls"], lw=1.0, clip_on=False)
        axS.text(cx, rows_y[0], u, transform=axS.transAxes, ha="right", va="top")
    axS.text(0.0, rows_y[0], "最高 (m)", transform=axS.transAxes, ha="left", va="top")
    for k, lab in enumerate(["實飛", "匯出", R_NAME, "放行 (s)"], start=1):
        axS.text(0.0, rows_y[k], lab, transform=axS.transAxes, ha="left", va="top")
    for u, cx in zip(UAVS, cols_x):
        vals = [
            num("NUMBERS", f"trials.{TRIAL}.altitude.flown_max_agl_per_uav.{u}", 3, f"{u} 實飛最高 (m)", "b"),
            num("metrics", f"{TRIAL}.altitude.per_uav.{u}.exported_max_rel_m", 3, f"{u} 匯出最高 (m)", "b"),
            num("NUMBERS", f"trials.{TRIAL_R}.altitude.flown_max_agl_per_uav.{u}", 3,
                f"{R_NAME} {u} 實飛最高 (m)", "b"),
            num("metrics", f"{TRIAL}.timing.per_uav.{u}.plan_t_takeoff_s", 0, f"{u} 規劃放行時刻 (s)", "b"),
        ]
        for k, v in enumerate(vals, start=1):
            axS.text(cx, rows_y[k], v, transform=axS.transAxes, ha="right", va="top")
    assert all(v == 0.0 for v in NUM["trials"][TRIAL]["altitude"]["t_above_120_per_uav"].values())
    return ins


def caption_bc(layout: str = "bc") -> str:
    """(b)(c) 版沒有 (a)，風場寫進圖說；數字同樣讀 NUMBERS.json 並登記到比對表。"""
    pw = NUM["trials"][TRIAL]["plan_wind"]
    sw = NUM["trials"][TRIAL]["sim_wind"]
    assert [float(pw[0]), float(pw[1])] == [float(sw["spd_mps"]), float(sw["dir_from_deg"])], (pw, sw)
    spd = num("NUMBERS", f"trials.{TRIAL}.sim_wind.spd_mps", 0, "模擬風速 (m/s)", "圖說")
    dfr = num("NUMBERS", f"trials.{TRIAL}.sim_wind.dir_from_deg", 0, "模擬風向（來向，°）", "圖說")
    pws = num("NUMBERS", f"trials.{TRIAL}.plan_wind.0", 0, "規劃風速 (m/s)", "圖說")
    pwd = num("NUMBERS", f"trials.{TRIAL}.plan_wind.1", 0, "規劃風向（來向，°）", "圖說")
    hc = num("NUMBERS", f"trials.{TRIAL}.ceiling_m", 1, "規劃上限 (m)", "圖說")
    hc_r = num("NUMBERS", f"trials.{TRIAL_R}.ceiling_m", 1, f"{R_NAME}規劃上限 (m)", "圖說")
    c_extra = ""
    if layout == "wide":
        ds = num("metrics", f"{TRIAL}.separation.reporter_raw.thresholds.d_sep_m", 0, "違規門檻 水平 (m)", "圖說")
        dh = num("metrics", f"{TRIAL}.separation.reporter_raw.thresholds.dh_sep_m", 0, "違規門檻 垂直 (m)", "圖說")
        assert MET[TRIAL]["separation"]["frames"]["ecef"]["pairs"]["A1-B2"]["min_h_m"] is None
        c_extra = f"，斜線區為違規區（水平 < {ds} m 且垂直 < {dh} m）；A1–B2 從未同時在空中"
    pb, pc = PANEL_TAG[layout]["b"], PANEL_TAG[layout]["c"]
    return (f"圖3　HV4W55（1 Pioneer＋2 HERO，規劃上限 {hc} m）"
            f"SITL 實飛：({pb}) 高度剖面，放大框為 119.4–120.3 m，灰線為{R_NAME}"
            f"（規劃上限 {hc_r} m，單變因對照）；"
            f"({pc}) 同時在空中的機間水平×垂直間距{c_extra}。規劃風＝模擬風＝{spd} m/s／{dfr}°"
            f"（規劃 {pws} m/s、自 {pwd}°；模擬 {spd} m/s、自 {dfr}°）。")


def caption_3col() -> str:
    """3col 版圖說（圖內不寫架次代號，圖說也用「本架次／對照架次」）；風場只在圖說。"""
    pw = NUM["trials"][TRIAL]["plan_wind"]
    sw = NUM["trials"][TRIAL]["sim_wind"]
    assert [float(pw[0]), float(pw[1])] == [float(sw["spd_mps"]), float(sw["dir_from_deg"])], (pw, sw)
    spd = num("NUMBERS", f"trials.{TRIAL}.sim_wind.spd_mps", 0, "模擬風速 (m/s)", "圖說")
    dfr = num("NUMBERS", f"trials.{TRIAL}.sim_wind.dir_from_deg", 0, "模擬風向（來向，°）", "圖說")
    hc = num("NUMBERS", f"trials.{TRIAL}.ceiling_m", 1, "規劃上限 (m)", "圖說")
    hc_r = num("NUMBERS", f"trials.{TRIAL_R}.ceiling_m", 1, f"{R_NAME}規劃上限 (m)", "圖說")
    ds = num("metrics", f"{TRIAL}.separation.reporter_raw.thresholds.d_sep_m", 0, "違規門檻 水平 (m)", "圖說")
    dh = num("metrics", f"{TRIAL}.separation.reporter_raw.thresholds.dh_sep_m", 0, "違規門檻 垂直 (m)", "圖說")
    assert MET[TRIAL]["separation"]["frames"]["ecef"]["pairs"]["A1-B2"]["min_h_m"] is None
    return (f"圖3　主結果架次（1 Pioneer＋2 HERO，規劃上限 {hc} m；規劃風＝模擬風＝{spd} m/s／{dfr}°）SITL 實飛："
            f"(a) 平面航跡，寬淡線為規劃、細線為飛行，虛線框為作業區，H 為共用起降點；"
            f"(b) 高度剖面，放大框為 119.4–120.3 m，灰線為{R_NAME}（規劃上限改設 {hc_r} m，單變因對照）；"
            f"(c) 同時在空中的機間水平×垂直間距，斜線區為違規區（水平 < {ds} m 且垂直 < {dh} m）；"
            f"A1–B2 從未同時在空中。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin-cache", default=None, help="BIN 重抽結果暫存目錄（加速重跑；預設不存）")
    ap.add_argument("--layout", choices=("abc", "bc", "wide", "3col"), default="abc",
                    help="abc：三格（(a) 左上、(c) 左下、(b) 右欄）；bc：(b)(c) 橫排，風場寫進圖說；"
                         "wide：(b)(c) 橫排、長寬比 ≥ 2.75（17.0 × 6.15 cm）；"
                         "3col：(a)(b)(c) 三欄並排、頂列各機航程，2007 × 620 px")
    args = ap.parse_args()

    arrays, bins = bin_arrays(args)
    check = verify_pipeline(arrays)
    print("驗證：BIN↔cache 與 重算↔metrics.json 全部相同")

    ctx = dict(lat0=float(MS["home"]["lat_deg"]), lon0=float(MS["home"]["lon_deg"]),
               clock=mission_clock(TRIAL), clock_r=mission_clock(TRIAL_R))
    # 門檻與上限：由 JSON 導出並與產生數字的程式常數交叉核對（兩者不符即中止）
    ctx["ceil_ops"] = (NUM["trials"][TRIAL]["altitude"]["flown_max_agl_m"]
                       + NUM["trials"][TRIAL]["altitude"]["margin_to_120_m"])
    ctx["d_sep"] = MET[TRIAL]["separation"]["reporter_raw"]["thresholds"]["d_sep_m"]
    ctx["dh_sep"] = MET[TRIAL]["separation"]["reporter_raw"]["thresholds"]["dh_sep_m"]
    ctx["h_ceil"] = NUM["trials"][TRIAL]["ceiling_m"]
    assert math.isclose(ctx["ceil_ops"], up_lib.CEIL_OPS_M, abs_tol=1e-9), ctx["ceil_ops"]
    # margin_to_100_m = 最小間距 − 100（間距越大越好）；margin_to_120_m = 120 − 最高高度（越低越好）
    assert math.isclose(NUM["trials"][TRIAL]["separation"]["flown_exact_m"]
                        - NUM["trials"][TRIAL]["separation"]["margin_to_100_m"], ctx["d_sep"], abs_tol=1e-9)
    assert ctx["d_sep"] == up_lib.D_SEP_H and ctx["dh_sep"] == up_lib.D_SEP_V

    caption = None
    if args.layout == "abc":
        fig = plt.figure(figsize=(17.0 * CM, 12.6 * CM))
        gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.0], height_ratios=[1.3, 1.0],
                              wspace=0.19, hspace=0.36, left=0.068, right=0.985, bottom=0.075, top=0.955)
        axA, axC, axB = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[:, 1])
        panel_a(axA, ctx)
        axes = [axA]
        stem = "fig3_results_hv4w55"
    elif args.layout == "bc":
        fig = plt.figure(figsize=(17.0 * CM, 9.6 * CM))
        gs = fig.add_gridspec(1, 2, width_ratios=[1.62, 1.0], wspace=0.17,
                              left=0.062, right=0.985, bottom=0.105, top=0.935)
        axB, axC = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
        axes = []
        stem = "fig3_results_hv4w55_bc"
    elif args.layout == "3col":
        # 使用者指定 2007 × 620 px @ 300 dpi；Agg 取 int(寬 × dpi) ⇒ 各加半個像素，避免浮點誤差少一像素
        W_CM, H_CM = 2007.5 / 300 * 2.54, 620.5 / 300 * 2.54
        fig = plt.figure(figsize=(W_CM * CM, H_CM * CM))
        fx = lambda cm: cm / W_CM                  # noqa: E731
        fy = lambda cm: cm / H_CM                  # noqa: E731
        LM, G_AB, G_BC, RM = 1.05, 1.02, 1.00, 0.10   # 左緣、(a)(b) 間、(b)(c) 間、右緣（放刻度字與軸標題）
        A_W, C_W = 3.45, 3.77
        B_W = W_CM - LM - G_AB - G_BC - RM - A_W - C_W
        bot, top = 0.76, H_CM - 0.86               # 下：刻度字＋軸標題；上：頂列＋子圖標題
        axA = fig.add_axes([fx(LM), fy(bot), fx(A_W), fy(top - bot)])
        axB = fig.add_axes([fx(LM + A_W + G_AB), fy(bot), fx(B_W), fy(top - bot)])
        axC = fig.add_axes([fx(W_CM - RM - C_W), fy(bot), fx(C_W), fy(top - bot)])
        header_3col(fig, 0.06, W_CM - 0.06, H_CM - 0.20)
        panel_a(axA, ctx, "3col")
        axes = [axA]
        stem = "fig3_results_hv4w55_3col"
    else:
        W_CM, H_CM = 17.0, 6.15                    # 長寬比 2.764 ≥ 2.75
        fig = plt.figure(figsize=(W_CM * CM, H_CM * CM))
        fx = lambda cm: cm / W_CM                  # noqa: E731
        fy = lambda cm: cm / H_CM                  # noqa: E731
        bot, top = fy(0.95), fy(5.62)
        axB = fig.add_axes([fx(1.12), bot, fx(5.10), top - bot])
        ctx["axS"] = fig.add_axes([fx(6.45), fy(0.10), fx(4.95), top - fy(0.10)])
        axC = fig.add_axes([fx(12.55), bot, fx(4.40), top - bot])
        axes = []
        stem = "fig3_results_hv4w55_wide"
    ins = panel_b(axB, ctx, args.layout)
    panel_c(axC, ctx, args.layout)
    if args.layout in ("bc", "wide"):
        caption = caption_bc(args.layout)
    elif args.layout == "3col":
        caption = caption_3col()
    for ax in axes + [axB, axC, ins]:
        ax.tick_params(pad=1.8)
    if ctx.get("ylabel_y"):
        # 縱軸標題：x 沿用畫完後的自動位置（緊貼刻度字），y 改對齊有資料的那一段
        fig.canvas.draw()
        for ax, yf in ctx["ylabel_y"]:
            x_disp = ax.yaxis.label.get_position()[0]
            ax.yaxis.set_label_coords(ax.transAxes.inverted().transform((x_disp, 0))[0], yf)

    # ------------------------------------------------------------------ 最小字級檢查（存檔前）
    fig.canvas.draw()
    small = []
    for t in fig.findobj(Text):
        if t.get_visible() and t.get_text().strip() and t.get_fontsize() < FS - 1e-9:
            small.append((t.get_text()[:30], t.get_fontsize()))
    if small:
        raise SystemExit(f"有文字小於 {FS} pt：{small}")

    png, svg = OUT_DIR / f"{stem}.png", OUT_DIR / f"{stem}.svg"
    fig.savefig(png, dpi=300, metadata={"Software": None})
    fig.savefig(svg, metadata={"Date": None, "Creator": None})
    w_in, h_in = fig.get_size_inches()
    tag = {"abc": "", "bc": "_bc", "wide": "_wide", "3col": "_3col"}[args.layout]
    if caption:
        (HERE / f"fig3_hv4w55{tag}_caption.txt").write_text(caption + "\n", encoding="utf-8")

    # ------------------------------------------------------------------ 逐項比對表
    for s in SHOWN:
        s["panel"] = PANEL_TAG[args.layout].get(s["panel"], s["panel"])
    rows = []
    for s in SHOWN:
        jv = s["json_value"]
        if isinstance(jv, (int, float)) and not isinstance(jv, bool):
            # 圖上數字與 JSON 值之差 ≤ 顯示精度的一半 ⇒ 一致（恰為 .5 的情形兩側皆算一致，由備註說明取哪側）
            nd = len(s["shown"].split(".")[1]) if "." in s["shown"] else 0
            ok = abs(Decimal(s["shown"]) - Decimal(str(jv))) <= Decimal(1).scaleb(-nd) / 2
        else:
            ok = s["shown"] == str(jv).replace("-", "–")
        rows.append((s["panel"], s["label"], s["shown"], s["file"], s["field"], jv, "OK" if ok else "✗",
                     s.get("note", "")))
    hdr = ("格", "圖上項目", "圖上數字", "檔案", "欄位", "JSON 值", "一致", "備註")
    tsv = HERE / f"fig3_hv4w55{tag}_numbers.tsv"
    tsv.write_text("\t".join(hdr) + "\n" + "\n".join("\t".join(map(str, r)) for r in rows) + "\n",
                   encoding="utf-8")
    data = {
        "figure": {"png": rel(png), "svg": rel(svg), "width_cm": round(w_in * 2.54, 3),
                   "height_cm": round(h_in * 2.54, 3), "dpi": 300, "min_font_pt": FS, "layout": args.layout,
                   "script": rel(Path(__file__))},
        "caption": caption,
        "trial": TRIAL, "control_trial": TRIAL_R,
        "shown_numbers": SHOWN,
        "pipeline_check": check,
        "geometry_sources": {
            "bins": {f"{t}_{u}": rel(p) for (t, u), p in bins.items()},
            "reader": "update_paper/scripts/up_extract.py extract()",
            "frame": "update_paper/scripts/up_lib.py enu()（WGS84 geodetic→ECEF→ENU，原點 plan/mission_summary.json home）",
            "separation": "update_paper/scripts/up_metrics.py _track(frame='ecef') + _pair_min()（原生 POS 時刻聯集、連續時間最小值）",
            "altitude": "POS.RelHomeAlt 原生取樣（airborne_window 內），x = 任務時間（同 up_metrics.timing 的 t0）",
        },
        "inputs_sha256": {rel(p): sha256(p) for p in sorted(
            [N_F, M_F, RUN / "scenario.json", RUN / "plan" / "mission_summary.json",
             UP / "scripts" / "up_lib.py", UP / "scripts" / "up_metrics.py", UP / "scripts" / "up_extract.py"]
            + [RUN / "plan" / f"{u}.waypoints" for u in UAVS] + list(bins.values()))},
    }
    (HERE / f"fig3_hv4w55{tag}_data.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")

    print(f"\n輸出 {rel(png)}（{w_in * 2.54:.2f} × {h_in * 2.54:.2f} cm，300 dpi）與 {rel(svg)}")
    if caption:
        print("\n圖說（風場）：" + caption)
    print("\n圖上數字 ↔ NUMBERS.json／metrics.json 逐項比對")
    widths = [max(len(str(r[i])) for r in rows + [hdr]) for i in range(len(hdr))]
    print("  ".join(str(h).ljust(w) for h, w in zip(hdr, widths)))
    for r in rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))
    bad = [r for r in rows if r[6] != "OK"]
    print(f"\n共 {len(rows)} 項，不一致 {len(bad)} 項")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
