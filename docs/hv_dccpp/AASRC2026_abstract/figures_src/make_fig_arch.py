#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AASRC 2026 中文加長型摘要用圖（研討會論文版 v2）：圖1 系統架構、圖2 規劃演算法流程。

用法（在 repo 根目錄或任何地方皆可）::

    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig_arch.py
    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig_arch.py --selftest
    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig_arch.py --compact   # 只產扁平版

輸出（與本腳本上一層目錄）：
    fig1_architecture.png / .svg   17 cm × 6.8 cm、300 dpi
    fig2_flowchart.png    / .svg   17 cm × 7.6 cm、300 dpi
    fig1_architecture_compact.png / .svg   17 cm × 5.33 cm、300 dpi（2007×629 px）
        圖1 扁平版，Word 第 1 頁高度不足時原寸替換用；內容與圖1 同義，
        另檢查寬 17.0 cm、高 ≤ 5.6 cm（見 build_fig1_compact）。最後產生，不影響前兩張圖位元組。
    17 cm = A4 扣左右各 2 cm 的版心寬，插入時不縮放，字級即印刷字級。
    v1（內部工程文件風格：函式名、GUI 路徑缺陷）保留為 *_v1_internal.png/svg，
    本腳本不再產生、也不覆寫它們。

版面檢查（每次產圖都跑；有任一項不過就不存檔、非零結束）：
    * 方塊文字的外框完全落在方塊（矩形／菱形／圖2 圖例框）內；
    * 箭頭折線不穿過任何文字外框、也不穿過非端點的方塊；箭頭彼此不交叉；
    * 文字與文字不重疊（混排文字塊跨行也檢查）、標籤不壓到方塊、全部落在畫布內；
    * 字級 ≥ 8 pt，mathtext 下標以 0.7 倍計仍須 ≥ 8 pt；
    * 每個字元都有字型可用，英數必須由 Times New Roman 提供，中文標楷體；
    * 投稿用字：非數學文字不得含底線字元，也不得出現內部用語
      （函式名、「保守」「下界」「例外」等，見 FORBIDDEN）。
``--selftest`` 會故意製造溢出／穿字／交叉／出畫布／字太小／缺字／英數字型錯／
底線／禁用字，確認每一項檢查真的會報錯（檢查器本身也要被驗證）。

主控台輸出只用 ASCII 狀態記號（OK／FAIL／WARN），並把 stdout 的錯誤處理設為
replace：本機預設 cp950 主控台下直接執行也不會因編碼中斷（v2 審查意見）。

v3 用詞更正（2026-09-21，word_layout）：
  「含風（Dubins）時間成本」一律改為論文定稿用詞「考慮風場之飛行時間成本」（C7）；
  圖1 執行層末格原寫「覆蓋率與機間間距（兩者皆為上界）」，與 3.1 節量尺不符
  （間距已改為 WGS84 橢球座標之連續時間最小值，只有覆蓋率仍為上界）⇒ 改為
  「覆蓋率（上界）與連續時間最小間距」（扁平版「航跡回算覆蓋率（上界）與連續時間最小間距」）；圖2 B6 方塊加寬 3.40 → 4.00 cm 容納較長用詞。
  修改前的圖檔與腳本備份於 runs/artifacts/PAPERPLAN1/word_layout/figures/before/。

v2 審查後的版面決定（2026-09-14）：
  圖1  輸入資料移到入口左側（全圖左→右讀向）；圖例移到層帶外最下一列；
       內容區左右留白一致（各 0.15 cm）；共用輸入條拉滿六個模組；
       規劃層六個模組與圖2 主流程一一對應，統一用「四維」「區域分解與高度分層」。
  圖2  λ 為 9 pt 斜體、floor 為 8 pt 正體說明性下標（手動下移基線），行距均勻；
       分岔兩路都標條件；左下加圖例；「4D」改「四維」與圖1、正文一致。

內容依據（2026-09-14 工作樹，唯讀核對；行號可能隨他人修改漂移）：
  圖1
    情境腳本入口          scripts/hv_sitl_run.py:191-229
    AeroPlan VTOL 偵察頁  ui/main_window.py:993-995, 7780-7812（論文圖只畫到協調介面）
    協調介面（切區、空域） mission/swarm_coordinator.py:1653-2183（切區 :1707-1744）
    規劃器模組            hv_dccpp/context.py:658；hv_dccpp/planner.py:1280-1593；
                          hv_dccpp/sim.py:1098-1176（整體驗證）
    匯出任務與參數檔      hv_dccpp/ardupilot.py:1087-1187；scripts/hv_sitl_run.py:287-288
    QuadPlane SITL ×3     scripts/hv_sitl_run.py:346-390
    參數逐項讀回          scripts/hv_sitl_run.py:520-552（不符 SystemExit :549-552）
    任務讀回（項數）      scripts/hv_sitl_run.py:554-569
    依起飛時刻放行        scripts/hv_sitl_run.py:590-633
    航跡回算              覆蓋率為上界（scripts/hv_sitl_report.py:138-262）；間距為
                          WGS84 連續時間最小值（runs/artifacts/PAPERPLAN1/update_paper，
                          論文 3.1 節之量尺）
  圖2
    切區                  mission/swarm_coordinator.py:1707-1744
    轉成規劃輸入          hv_dccpp/integration/aeroplan_adapter.py:1905-1916
    情境建構＋前置檢查    hv_dccpp/context.py:658-814；hv_dccpp/planner.py:1309-1313
    回報不可行            aeroplan_adapter.py:1923-1924, 1942-1944
    回報未覆蓋區          hv_dccpp/planner.py:1387-1389
    分層→時間分離         hv_dccpp/altitude.py:1516-1572；planner.py:1348-1367
    配置與 λ_floor        hv_dccpp/allocate.py:693-748（決策級配額）
    Beam-IDP              hv_dccpp/planner.py:1491-1492
    返場能量修復 3 輪     hv_dccpp/planner.py:1493-1509
    剩餘條帶再拍賣        hv_dccpp/planner.py:1512-1530
    走廊與三維組裝        hv_dccpp/planner.py:1533-1566
    衝突偵測／修復        hv_dccpp/planner.py:1568-1585；hv_dccpp/deconflict.py:1465-1499, 1841-1901
    整體驗證報告          mission/swarm_coordinator.py:2162-2183；scripts/hv_sitl_run.py:269-293
    匯出                  hv_dccpp/ardupilot.py:1087-1187
"""
from __future__ import annotations

import argparse
import hashlib
import math
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager as fm  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent.parent
CM = 1.0 / 2.54
PT_CM = 2.54 / 72.0
DPI = 300
MIN_PT = 8.0
SUB_SHRINK = 0.7          # matplotlib mathtext 的下標縮放（_mathtext.SHRINK_FACTOR）
LAMBDA_PT = 9.0           # 圖2 λ 的字級（與 8 pt 中文同行，略大以補斜體希臘字的視覺小）
SUB_DROP = 0.06           # 手動下標的基線下移量 cm（≈ 1.7 pt ≈ 0.19 em @ 9 pt）
MATH_EXTRA = 0.025        # 與數學行相鄰時行距多加的量 cm

FONT_KAI = Path("C:/Windows/Fonts/kaiu.ttf")      # 標楷體
FONT_TNR = Path("C:/Windows/Fonts/times.ttf")     # Times New Roman
FONT_TNR_I = Path("C:/Windows/Fonts/timesi.ttf")  # Times New Roman Italic（mathtext）
FONT_JH = Path("C:/Windows/Fonts/msjh.ttc")       # 退路：微軟正黑體

# 投稿版不得出現的內部用語（非數學文字另外禁止底線字元）
FORBIDDEN = ("保守", "下界", "例外", "不阻擋", "只記", "僅腳本", "元組", "未載入",
             "validate", "MissionInputs", "Exporter", "dccpp", "adapter",
             "build_context", "plan_mission", "plan_coverage", ".py", "NO_GO")
MATH_CMDS = {r"\lambda": "\u03bb"}
MATH_FORMAT_CMDS = (r"\mathrm",)   # \u53ea\u6539\u5b57\u5f62\u3001\u4e0d\u7522\u751f\u5b57\u5143\u7684 mathtext \u6307\u4ee4

# ── 黑白可辨識的樣式（只用灰階與線型）──────────────────────────────
DASH = (0, (4.0, 2.0))
DASHDOT = (0, (5.0, 1.6, 1.2, 1.6))
LW_THIN = 0.9
LW_THICK = 1.8


# ════════════════════════════════════════════════════════════════════
#  字型
# ════════════════════════════════════════════════════════════════════
class Fonts:
    def __init__(self) -> None:
        self.notes: list[str] = []
        cjk = FONT_KAI
        if not cjk.exists():
            cjk = FONT_JH
            self.notes.append(f"找不到 {FONT_KAI}，中文退回 {FONT_JH}")
        latin = FONT_TNR if FONT_TNR.exists() else None
        if latin is None:
            self.notes.append(f"找不到 {FONT_TNR}，英數改用中文字型")
        self.paths = [p for p in (latin, cjk) if p is not None]
        self.names = []
        for p in self.paths:
            fm.fontManager.addfont(str(p))
            # 以 FontProperties(fname=...) 取得字型名稱，再組成
            # 「英數 → 中文」的字型後備鏈（matplotlib ≥3.6 逐字元後備）。
            self.names.append(fm.FontProperties(fname=str(p)).get_name())
        # mathtext 不走後備鏈：數學片段只放拉丁／希臘字，中文另成片段（見 Canvas.rich）
        self.math_paths = [p for p in (FONT_TNR, FONT_TNR_I) if p.exists()]
        if latin is not None and FONT_TNR_I.exists():
            fm.fontManager.addfont(str(FONT_TNR_I))
            tnr = self.names[0]
            plt.rcParams.update({"mathtext.fontset": "custom", "mathtext.rm": tnr,
                                 "mathtext.it": f"{tnr}:italic", "mathtext.bf": f"{tnr}:bold"})
        else:
            plt.rcParams["mathtext.fontset"] = "stix"
            self.notes.append("找不到 Times New Roman（含斜體），mathtext 改用 STIX")
        self._cmaps = None
        self._math_cmaps = None

    def prop(self, size: float) -> fm.FontProperties:
        return fm.FontProperties(family=list(self.names), size=size)

    @staticmethod
    def _load(paths):
        from fontTools.ttLib import TTFont
        out = []
        for p in paths:
            kw = {"fontNumber": 0} if p.suffix.lower() == ".ttc" else {}
            out.append((p.name, TTFont(str(p), lazy=True, **kw).getBestCmap()))
        return out

    def cmaps(self):
        if self._cmaps is None:
            self._cmaps = self._load(self.paths)
        return self._cmaps

    def math_cmaps(self):
        if self._math_cmaps is None:
            self._math_cmaps = self._load(self.math_paths)
        return self._math_cmaps


def is_math(s: str) -> bool:
    return s.count("$") >= 2


def math_glyphs(s: str) -> tuple[str, list[str]]:
    """把 mathtext 字串還原成實際會畫出的字元；回傳（字元, 未知指令）。"""
    body = s.replace("$", "")
    for cmd, ch in MATH_CMDS.items():
        body = body.replace(cmd, ch)
    for cmd in MATH_FORMAT_CMDS:
        body = body.replace(cmd, "")
    unknown = re.findall(r"\\[A-Za-z]+", body)
    return re.sub(r"[_^{}\s]", "", body), unknown


# ════════════════════════════════════════════════════════════════════
#  畫布：以公分為資料座標，並登記所有物件供版面檢查
# ════════════════════════════════════════════════════════════════════
class Canvas:
    def __init__(self, w_cm: float, h_cm: float, fonts: Fonts) -> None:
        self.W, self.H, self.fonts = w_cm, h_cm, fonts
        self.fig = plt.figure(figsize=(w_cm * CM, h_cm * CM), dpi=DPI)
        self.ax = self.fig.add_axes((0, 0, 1, 1))
        self.ax.set_xlim(0, w_cm)
        self.ax.set_ylim(0, h_cm)
        self.ax.set_axis_off()
        self.nodes: dict[str, dict] = {}
        self.edges: list[dict] = []
        self.texts: list[dict] = []
        self._ngroup = 0

    # ── 量測 ────────────────────────────────────────────────────────
    def extent(self, t) -> tuple[float, float, float, float]:
        r = self.fig.canvas.get_renderer()
        bb = t.get_window_extent(renderer=r)
        inv = self.ax.transData.inverted()
        (x0, y0), (x1, y1) = inv.transform([(bb.x0, bb.y0), (bb.x1, bb.y1)])
        return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)

    def measure(self, s, size=8.0, linespacing=1.12) -> tuple[float, float]:
        t = self.ax.text(0, 0, s, fontproperties=self.fonts.prop(size), linespacing=linespacing)
        b = self.extent(t)
        t.remove()
        return b[2] - b[0], b[3] - b[1]

    # ── 文字 ────────────────────────────────────────────────────────
    def text(self, x, y, s, size=8.0, ha="center", va="center", kind="label",
             owner=None, rotation=0, linespacing=1.12, group=None, row=None):
        t = self.ax.text(x, y, s, ha=ha, va=va, rotation=rotation, multialignment=ha
                         if ha in ("left", "right") else "center",
                         fontproperties=self.fonts.prop(size),
                         linespacing=linespacing, zorder=6, color="black")
        self.texts.append({"artist": t, "kind": kind, "owner": owner, "group": group,
                           "row": row})
        return t

    def rich(self, cx, cy, lines, kind="node", owner=None, gap=0.05):
        """多行、可混排 mathtext 的文字塊（中文與數學分成相鄰片段）。

        lines: 每行為 [(字串, 字級[, 前距 cm[, 基線偏移 cm]]), ...]。matplotlib 的
        mathtext 不走字型後備鏈，中文若放進 $...$ 同一個 Text 會變成缺字方框，
        所以必須拆片段排版。前距以「版面框」計；斜體 λ 的墨跡會伸出版面框左緣，
        所以數學片段要自己給前距（依放大裁切目視調定）。

        說明性下標（如 floor）不用 mathtext 的 _{}：那會縮成 0.7 倍，為了 ≥ 8 pt
        只好把 λ 撐到 11.4 pt，λ 比同行中文大一截、行距也被撐開（審查意見）。
        改成另一個 8 pt 正體片段，以「基線偏移」手動下移。

        行距一律用一般多行文字（linespacing 1.12）的基線間距（量「兩行高 − 一行高」），
        不依各行版面框往下疊，否則含數學的行會被撐開、行距不均。上下行片段不重疊
        由 check() 的「同文字塊跨行重疊」檢查把關。
        """
        self._ngroup += 1
        grp = f"rich{self._ngroup}"
        base_size = lines[0][0][1]
        one = self.measure("情", base_size)[1]
        pitch = self.measure("情\n情", base_size)[1] - one   # 基線到基線
        rows = []
        for r, parts in enumerate(lines):
            arts, gaps, dys = [], [], []
            for j, part in enumerate(parts):
                s, size = part[0], part[1]
                gaps.append(0.0 if j == 0 else (part[2] if len(part) > 2 else gap))
                dys.append(part[3] if len(part) > 3 else 0.0)
                t = self.text(0, 0, s, size=size, ha="left", va="baseline",
                              kind=kind, owner=owner, group=grp, row=r)
                arts.append(t)
            exts = [self.extent(t) for t in arts]
            asc = max(e[3] + dy for e, dy in zip(exts, dys))
            desc = -min(e[1] + dy for e, dy in zip(exts, dys))
            rows.append((arts, gaps, dys, exts, asc, desc))
        # 中文行距本身只有約 0.024 cm 空隙；9 pt 的 λ 版面框比中文高 0.04 cm，
        # 所以與數學行相鄰的行距只多加 MATH_EXTRA（約 7%，不會出現舊版 1.6 倍的不均）
        has_math = [any(is_math(t.get_text()) for t in row[0]) for row in rows]
        offs = [0.0]
        for k in range(1, len(rows)):
            extra = MATH_EXTRA if has_math[k] or has_math[k - 1] else 0.0
            offs.append(offs[-1] + pitch + extra)
        rel_top = max(row[4] - offs[k] for k, row in enumerate(rows))
        rel_bot = min(-offs[k] - row[5] for k, row in enumerate(rows))
        base0 = cy - (rel_top + rel_bot) / 2
        for k, (arts, gaps, dys, exts, _, _) in enumerate(rows):
            base = base0 - offs[k]
            widths = [e[2] - e[0] for e in exts]
            x = cx - (sum(widths) + sum(gaps)) / 2
            for t, e, w, g, dy in zip(arts, exts, widths, gaps, dys):
                x += g
                t.set_position((x - e[0], base + dy))
                x += w

    # ── 方塊 ────────────────────────────────────────────────────────
    def band(self, x0, y0, x1, y1, fc, ec="#7f7f7f", lw=0.5, ls="-", z=0):
        self.ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=fc,
                                    edgecolor=ec, lw=lw, ls=ls, zorder=z))

    def box(self, nid, cx, cy, w, h, label, fill="white", ls="-", lw=LW_THIN,
            rounding=0.06, size=8.0, align="center"):
        p = FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                           boxstyle=f"round,pad=0,rounding_size={rounding}",
                           facecolor=fill, edgecolor="black", lw=lw, ls=ls, zorder=3)
        self.ax.add_patch(p)
        self.nodes[nid] = {"shape": "rect", "cx": cx, "cy": cy, "w": w, "h": h}
        if isinstance(label, list):
            self.rich(cx, cy, label, owner=nid)
        elif align == "left":
            self.text(cx - w / 2 + 0.20, cy, label, size=size, ha="left", kind="node", owner=nid)
        else:
            self.text(cx, cy, label, size=size, kind="node", owner=nid)

    def diamond(self, nid, cx, cy, w, h, label, fill="#d9d9d9", lw=LW_THIN, size=8.0):
        pts = [(cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2), (cx - w / 2, cy)]
        self.ax.add_patch(Polygon(pts, closed=True, facecolor=fill, edgecolor="black",
                                  lw=lw, zorder=3, joinstyle="miter"))
        self.nodes[nid] = {"shape": "diamond", "cx": cx, "cy": cy, "w": w, "h": h}
        self.text(cx, cy, label, size=size, kind="node", owner=nid)

    def at(self, nid, side, off=0.0):
        n = self.nodes[nid]
        cx, cy, w, h = n["cx"], n["cy"], n["w"], n["h"]
        if n["shape"] == "diamond":
            off = 0.0
        return {"top": (cx + off, cy + h / 2), "bottom": (cx + off, cy - h / 2),
                "left": (cx - w / 2, cy + off), "right": (cx + w / 2, cy + off)}[side]

    # ── 箭頭（正交折線 + 實心箭頭）───────────────────────────────────
    def arrow(self, pts, ls="-", lw=LW_THIN, head=True, name=""):
        pts = [tuple(map(float, p)) for p in pts]
        L, HW = (0.20, 0.085) if lw >= LW_THICK else (0.17, 0.068)
        draw = list(pts)
        if head:
            (x0, y0), (x1, y1) = pts[-2], pts[-1]
            d = math.hypot(x1 - x0, y1 - y0)
            ux, uy = (x1 - x0) / d, (y1 - y0) / d
            draw[-1] = (x1 - ux * L * 0.85, y1 - uy * L * 0.85)
            bx, by = x1 - ux * L, y1 - uy * L
            tri = [(x1, y1), (bx - uy * HW, by + ux * HW), (bx + uy * HW, by - ux * HW)]
            self.ax.add_patch(Polygon(tri, closed=True, facecolor="black",
                                      edgecolor="black", lw=0.3, zorder=5))
        xs, ys = zip(*draw)
        self.ax.plot(xs, ys, color="black", lw=lw, ls=ls, zorder=4,
                     solid_capstyle="butt", dash_capstyle="butt",
                     solid_joinstyle="miter")
        self.edges.append({"pts": pts, "name": name, "head_len": L if head else 0.0})

    def dot(self, x, y):
        self.ax.plot([x], [y], marker="o", ms=2.6, color="black", zorder=5)


# ════════════════════════════════════════════════════════════════════
#  版面檢查
# ════════════════════════════════════════════════════════════════════
def _inside_node(n, x, y, shrink=0.0) -> bool:
    cx, cy, w, h = n["cx"], n["cy"], n["w"] / 2 - shrink, n["h"] / 2 - shrink
    if w <= 0 or h <= 0:
        return False
    if n["shape"] == "rect":
        return abs(x - cx) < w and abs(y - cy) < h
    return abs(x - cx) / w + abs(y - cy) / h < 1.0


def _rects_overlap(a, b, m=0.0) -> bool:
    return not (a[2] + m <= b[0] or b[2] + m <= a[0] or a[3] + m <= b[1] or b[3] + m <= a[1])


def _grid(b, n=12):
    for i in range(n + 1):
        for j in range(n + 1):
            yield b[0] + (b[2] - b[0]) * i / n, b[1] + (b[3] - b[1]) * j / n


def _samples(pts, step=0.01):
    """折線上的取樣點，排除距起點／終點 0.05 cm 內（那裡本來就貼著端點方塊）。"""
    start, end = pts[0], pts[-1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        k = max(1, int(d / step))
        for i in range(k + 1):
            x, y = x0 + (x1 - x0) * i / k, y0 + (y1 - y0) * i / k
            if math.hypot(x - start[0], y - start[1]) < 0.05:
                continue
            if math.hypot(x - end[0], y - end[1]) < 0.05:
                continue
            yield x, y


def _seg_cross(p1, p2, q1, q2) -> bool:
    def orient(a, b, c):
        v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        return 0 if abs(v) < 1e-9 else (1 if v > 0 else -1)
    o1, o2 = orient(p1, p2, q1), orient(p1, p2, q2)
    o3, o4 = orient(q1, q2, p1), orient(q1, q2, p2)
    return o1 * o2 < 0 and o3 * o4 < 0


def check(cv: Canvas, allowed_crossings=()) -> list[str]:
    probs: list[str] = []
    boxes = []
    cmaps = cv.fonts.cmaps()
    # 英數必須由 Times New Roman 提供：以檔名比對，不以「後備鏈第一個」比對
    # （v2 自我測試發現：舊寫法在鏈順序錯時拿錯的第一個來比，永遠不會報錯）
    tnr = FONT_TNR.name if FONT_TNR in cv.fonts.paths else None
    for rec in cv.texts:
        t = rec["artist"]
        s = t.get_text()
        if not s:
            continue
        ismath = is_math(s)
        eff = t.get_fontsize() * (SUB_SHRINK if ismath and ("_" in s or "^" in s) else 1.0)
        if eff < MIN_PT - 1e-6:
            probs.append(f"字級 {eff:.2f} pt < {MIN_PT}（含下標縮放）：{s!r}")
        b = cv.extent(t)
        boxes.append((rec, b))
        if b[0] < 0.02 or b[1] < 0.02 or b[2] > cv.W - 0.02 or b[3] > cv.H - 0.02:
            probs.append(f"文字超出畫布：{s!r} {tuple(round(v, 2) for v in b)}")
        # 投稿用字
        if not ismath and "_" in s:
            probs.append(f"出現底線字元（識別字或未轉下標）：{s!r}")
        for word in FORBIDDEN:
            if word.lower() in s.lower():
                probs.append(f"出現內部用語 {word!r}：{s!r}")
        # 字元涵蓋
        if ismath:
            glyphs, unknown = math_glyphs(s)
            for cmd in unknown:
                probs.append(f"未登記的 mathtext 指令 {cmd}：{s!r}")
            for nm, cm in cv.fonts.math_cmaps():
                for ch in glyphs:
                    if ord(ch) not in cm:
                        probs.append(f"數學字型 {nm} 沒有 {ch!r}：{s!r}")
            continue
        for ch in s:
            if ch in "\n ":
                continue
            owner = next((nm for nm, cm in cmaps if ord(ch) in cm), None)
            if owner is None:
                probs.append(f"沒有字型有這個字：{ch!r} in {s!r}")
            elif ch.isascii() and ch.isalnum() and tnr is not None and owner != tnr:
                probs.append(f"英數沒走 Times New Roman：{ch!r}")

    # 1) 方塊文字在方塊內
    for rec, b in boxes:
        if rec["kind"] != "node":
            continue
        n = cv.nodes[rec["owner"]]
        # 0.12 cm ≈ 1.2 mm 留白：v1 第一輪目視發現 0.07 cm 時字貼框
        pad = 0.12 if n["shape"] == "rect" else 0.0
        ok = all(_inside_node(n, x, y, shrink=pad) for x, y in
                 ((b[0], b[1]), (b[0], b[3]), (b[2], b[1]), (b[2], b[3])))
        if n["shape"] == "diamond":
            cx, cy, hw, hh = n["cx"], n["cy"], n["w"] / 2, n["h"] / 2
            worst = max(abs(x - cx) / hw + abs(y - cy) / hh for x, y in
                        ((b[0], b[1]), (b[0], b[3]), (b[2], b[1]), (b[2], b[3])))
            ok = worst <= 0.97
        if not ok:
            probs.append(f"文字溢出方塊 {rec['owner']}：{rec['artist'].get_text()!r}")

    # 2) 箭頭不穿過文字、不穿過非端點方塊；最後一段要放得下箭頭
    for e in cv.edges:
        samp = list(_samples(e["pts"]))
        for rec, b in boxes:
            bb = (b[0] - 0.03, b[1] - 0.03, b[2] + 0.03, b[3] + 0.03)
            if any(bb[0] <= x <= bb[2] and bb[1] <= y <= bb[3] for x, y in samp):
                probs.append(f"箭頭 {e['name']} 穿過文字 {rec['artist'].get_text()!r}")
        for nid, n in cv.nodes.items():
            if any(_inside_node(n, x, y, shrink=0.01) for x, y in samp):
                probs.append(f"箭頭 {e['name']} 穿過方塊 {nid}")
        (x0, y0), (x1, y1) = e["pts"][-2], e["pts"][-1]
        if math.hypot(x1 - x0, y1 - y0) < e["head_len"] + 0.04:
            probs.append(f"箭頭 {e['name']} 最後一段太短，放不下箭頭")

    # 3) 箭頭彼此交叉
    for i, a in enumerate(cv.edges):
        for bE in cv.edges[i + 1:]:
            if (a["name"], bE["name"]) in allowed_crossings:
                continue
            for p1, p2 in zip(a["pts"], a["pts"][1:]):
                for q1, q2 in zip(bE["pts"], bE["pts"][1:]):
                    if _seg_cross(p1, p2, q1, q2):
                        probs.append(f"箭頭交叉：{a['name']} × {bE['name']}")

    # 4) 文字彼此不重疊（同一個混排文字塊「同一行」的相鄰片段除外；跨行仍檢查，
    #    以 m=0 計，因為行距本身就只有約 0.04 cm）；非方塊文字不壓到方塊
    for i, (ra, a) in enumerate(boxes):
        for rb, b in boxes[i + 1:]:
            same_group = ra["group"] is not None and ra["group"] == rb["group"]
            if same_group and ra["row"] == rb["row"]:
                continue
            if _rects_overlap(a, b, m=0.0 if same_group else 0.02):
                probs.append(f"文字重疊：{ra['artist'].get_text()!r} × {rb['artist'].get_text()!r}")
        if ra["kind"] == "node":
            continue
        for nid, n in cv.nodes.items():
            if any(_inside_node(n, x, y, shrink=-0.03) for x, y in _grid(a)):
                probs.append(f"標籤 {ra['artist'].get_text()!r} 壓到方塊 {nid}")

    # 5) 方塊彼此不重疊，且至少留 CLEAR cm（v2 第二輪目視：圖例框角離菱形僅 0.08 cm，
    #    幾何上沒重疊所以沒被抓到）；取樣含邊界點（shrink=-1e-6）
    CLEAR = 0.10
    ids = list(cv.nodes)
    for i, u in enumerate(ids):
        nu = cv.nodes[u]
        bu = (nu["cx"] - nu["w"] / 2, nu["cy"] - nu["h"] / 2, nu["cx"] + nu["w"] / 2, nu["cy"] + nu["h"] / 2)
        if bu[0] < 0.02 or bu[1] < 0.02 or bu[2] > cv.W - 0.02 or bu[3] > cv.H - 0.02:
            probs.append(f"方塊超出畫布：{u}")
        for v in ids[i + 1:]:
            nv = cv.nodes[v]
            pts_u = [(x, y) for x, y in _grid(bu, 40) if _inside_node(nu, x, y, shrink=-1e-6)]
            if any(_inside_node(nv, x, y) for x, y in pts_u):
                probs.append(f"方塊重疊：{u} × {v}")
            elif any(_inside_node(nv, x, y, shrink=-CLEAR) for x, y in pts_u):
                probs.append(f"方塊間距 < {CLEAR} cm：{u} × {v}")
    return probs


def _row(cv: Canvas, x0, x1, labels, min_w, pad=0.50):
    """一列方塊：寬度依文字量測（≥ min_w），等間距鋪滿 [x0, x1]；回傳 [(cx, w)]。"""
    ws = [max(min_w, cv.measure(s)[0] + pad) for s in labels]
    gap = (x1 - x0 - sum(ws)) / (len(ws) - 1)
    out, x = [], x0
    for w in ws:
        out.append((x + w / 2, w))
        x += w + gap
    return out, gap


# ════════════════════════════════════════════════════════════════════
#  圖1　系統架構（三層水平帶）
# ════════════════════════════════════════════════════════════════════
def build_fig1(fonts: Fonts) -> Canvas:
    W, H = 17.0, 6.8
    cv = Canvas(W, H, fonts)
    A = cv.at
    TH = {"lw": LW_THICK}
    # 內容區左右界：層名欄右緣 1.75、層帶右緣 W−0.05，兩側留白皆 0.15 cm
    # （v2 審查：XR = W−0.10 時右側只剩 0.05 cm，方塊框線貼著層帶邊線）
    XL, XR = 1.90, W - 0.20

    # ── 三層色帶與左側層名；最下方一列是層帶外的圖例（v2 審查：圖例不可放在層內）──
    bands = [("輸入與\n介面層", 5.00, H - 0.05),
             ("HARVEST\n規劃層", 2.50, 4.95),
             ("ArduPilot\n執行與\n驗證層", 0.45, 2.45)]
    for name, y0, y1 in bands:
        cv.band(0.05, y0, W - 0.05, y1, "#f2f2f2")
        cv.band(0.05, y0, 1.75, y1, "#d4d4d4")
        cv.text(0.90, (y0 + y1) / 2, name, size=9, kind="band")

    # ── 第 1 層（左→右）：輸入資料 → 兩個入口 → 協調介面 ──
    # v2 審查：輸入框原在協調介面右側、箭頭由右往左，與全圖讀向相反
    yc1 = 5.875
    ye1, ye2 = yc1 + 0.43, yc1 - 0.43
    d_txt = "輸入：作業區多邊形、\n機隊構型、高度上限 120 m、\n風、起降場坪"
    dw = cv.measure(d_txt)[0] + 0.45
    e_lab = ("AeroPlan 地面站（VTOL 偵察頁）", "情境腳本（驗證實驗）")
    ew = max(cv.measure(s)[0] for s in e_lab) + 0.50
    c_txt = "協調介面\n機隊切區、空域與法規參數"
    cw = cv.measure(c_txt)[0] + 0.50
    g1 = (XR - XL - dw - ew - cw) / 2          # 兩段水平間距均分剩餘寬度
    dcx = XL + dw / 2
    ecx = XL + dw + g1 + ew / 2
    ccx = XR - cw / 2
    cv.box("D", dcx, yc1, dw, 1.45, d_txt, ls=DASH, align="left")
    cv.box("E1", ecx, ye1, ew, 0.58, e_lab[0])
    cv.box("E2", ecx, ye2, ew, 0.58, e_lab[1])
    cv.box("C", ccx, yc1, cw, 1.45, c_txt)
    cv.arrow([A("D", "right", off=ye1 - yc1), A("E1", "left")], name="D-E1")
    cv.arrow([A("D", "right", off=ye2 - yc1), A("E2", "left")], name="D-E2")
    cv.arrow([A("E1", "right"), A("C", "left", off=ye1 - yc1)], name="E1-C")
    cv.arrow([A("E2", "right"), A("C", "left", off=ye2 - yc1)], name="E2-C", **TH)

    # ── 第 2 層：HARVEST 規劃器大框 ──
    # 模組切分與圖2 主流程一一對應（v2 審查：兩圖用詞與切分不一致）
    fy0, fy1 = 2.56, 4.89
    cv.band(XL, fy0, XR, fy1, "#e6e6e6", ec="black", lw=LW_THIN, z=1)
    mods = ["情境建構\n與前置檢查",
            "區域分解\n與高度分層",
            "考慮風場之\n飛行時間成本、\n構型與架數配置",
            "Beam-IDP\n條帶序列與\n返場能量修復",
            "起降段組裝\n與四維去衝突",
            "整體驗證"]
    x_down = XR - 0.25                          # M6 → 第 3 層的下行線（在模組列右側）
    row, _ = _row(cv, XL + 0.15, x_down - 0.25, mods, min_w=1.75, pad=0.42)
    my, mh = 3.77, 1.15
    for k, ((cx, w), lab) in enumerate(zip(row, mods)):
        cv.box(f"M{k + 1}", cx, my, w, mh, lab)
    for k in range(1, len(mods)):
        cv.arrow([A(f"M{k}", "right"), A(f"M{k + 1}", "left")], name=f"M{k}-M{k + 1}", **TH)
    y_run = 4.62
    x_c = ccx - cw / 2 + 0.60                   # 由協調介面左下方落下，讓出右側標題
    cv.arrow([A("C", "bottom", off=x_c - ccx), (x_c, y_run), (row[0][0], y_run), A("M1", "top")],
             name="C-M1", **TH)
    cv.text(XR - 0.18, y_run, "HARVEST 規劃器", size=9, ha="right", kind="title")
    # 共用輸入條拉滿六個模組（v2 審查：原本只橫跨 2–5，像是前置檢查與驗證用不到能量模型）
    sx0 = row[0][0] - row[0][1] / 2
    sx1 = row[-1][0] + row[-1][1] / 2
    cv.box("S", (sx0 + sx1) / 2, 2.83, sx1 - sx0, 0.50,
           "各模組共用輸入：平台與能量模型、風場與地形", ls=DASH)

    # ── 第 3 層：ArduPilot 執行與驗證 ──
    exe = ["匯出 QuadPlane\n任務與參數檔",
           "MAVLink 上傳至\nArduPlane QuadPlane\nSITL（×3）",
           "讀回檢核\n參數逐項、任務項數\n不符即停飛",
           "依規劃起飛\n時刻放行",
           "飛行紀錄回算\n覆蓋率（上界）與\n連續時間最小間距"]
    row3, _ = _row(cv, XL, XR, exe, min_w=2.20)
    y3, h3 = 1.10, 1.15
    for k, ((cx, w), lab) in enumerate(zip(row3, exe)):
        cv.box(f"X{k + 1}", cx, y3, w, h3, lab)
    for k in range(1, len(exe)):
        cv.arrow([A(f"X{k}", "right"), A(f"X{k + 1}", "left")], name=f"X{k}-X{k + 1}", **TH)
    yc = 1.95
    cv.arrow([A("M6", "right"), (x_down, my), (x_down, yc), (row3[0][0], yc), A("X1", "top")],
             name="M6-X1", **TH)
    cv.text((row3[1][0] + row3[3][0]) / 2, yc + 0.24, "任務計畫與驗證報告", kind="label")

    # ── 圖例（層帶外，單行）──
    yl = 0.225
    lx = XL
    for i, (lw, txt) in enumerate(((LW_THIN, "資料流"), (LW_THICK, "本文驗證路徑"))):
        cv.arrow([(lx, yl), (lx + 0.75, yl)], lw=lw, name=f"legend{i}")
        t = cv.text(lx + 0.90, yl, txt, ha="left", kind="legend")
        lx = cv.extent(t)[2] + 0.60
    return cv


# ════════════════════════════════════════════════════════════════════
#  圖1 扁平版（Word 第 1 頁只剩約 5.8 cm 高：17.0 cm × ≤5.6 cm，原寸插入）
# ════════════════════════════════════════════════════════════════════
COMPACT_W, COMPACT_H_MAX = 17.0, 5.6


def build_fig1_compact(fonts: Fonts) -> Canvas:
    """與 build_fig1 同義的扁平版；不改動 build_fig1 本身（原圖輸出位元組不變）。

    壓高度的取捨（實測：單行字高 0.26 cm、兩行 0.53–0.57 cm；check 要求方塊內留白 0.12）：
      * 方塊文字最多兩行（高 0.85）；入口方塊單行（高 0.52）上下疊放；
      * 層名欄 1.45 cm 寬、8.5 pt；層帶之間仍留 0.05 cm 白縫；
      * 「任務計畫與驗證報告」改為嵌在通道線上（線在字的兩側斷開，
        登記成兩段箭頭，所以「箭頭不穿字」照樣由 check() 把關），省掉一列標籤；
      * 共用輸入條單行細條（高 0.50）；圖例仍在層帶外最下一列。
    高度由下往上逐項累加，最後一個數字就是畫布高，避免手調常數互相打架。
    """
    W = COMPACT_W
    A = None
    TH = {"lw": LW_THICK}
    GAP_BAND = 0.05                     # 層帶之間的白縫
    # 層名欄右緣：第一輪目視 1.50 時「HARVEST」(8.5 pt 寬 1.36) 左右只剩 0.045 cm、貼欄邊
    COL_R = 1.56
    XL, XR = COL_R + 0.12, W - 0.05 - 0.12
    H2, H1 = 0.85, 0.52                 # 兩行方塊高、單行方塊高
    RUN = 0.26                          # 箭頭最後一段（≥ 粗箭頭頭長 0.20 + 0.04）

    # ── 由下往上排定各列 y ──
    y_leg = 0.19                        # 圖例列（層帶外）
    b3_0 = 0.36
    y3 = b3_0 + 0.07 + H2 / 2           # 執行層方塊中心
    yc = y3 + H2 / 2 + RUN              # M6 → X1 的通道線
    b3_1 = yc + 0.13 + 0.06             # 通道上嵌字半高 0.13 + 留白
    b2_0 = b3_1 + GAP_BAND
    fy0 = b2_0 + 0.05                   # 規劃器大框下緣
    ys = fy0 + 0.07 + 0.25              # 共用輸入條中心（高 0.50）
    my = ys + 0.25 + 0.13 + H2 / 2      # 模組列中心（與共用條留 0.13 > CLEAR）
    y_run = my + H2 / 2 + RUN           # 協調介面 → M1 的橫線
    # 9 pt 標題半高 0.15 + 留白；第一輪目視 0.05 時標題字頂貼大框上緣 ⇒ 0.09
    fy1 = y_run + 0.15 + 0.09
    b2_1 = fy1 + 0.05
    b1_0 = b2_1 + GAP_BAND
    yc1 = b1_0 + 0.06 + (2 * H1 + 0.14) / 2
    b1_1 = yc1 + (2 * H1 + 0.14) / 2 + 0.06
    H = round(b1_1 + 0.05, 2)

    cv = Canvas(W, H, fonts)
    A = cv.at

    bands = [("輸入與\n介面層", b1_0, b1_1),
             ("HARVEST\n規劃層", b2_0, b2_1),
             ("ArduPilot\n執行與\n驗證層", b3_0, b3_1)]
    for name, y0, y1 in bands:
        cv.band(0.05, y0, W - 0.05, y1, "#f2f2f2")
        cv.band(0.05, y0, COL_R, y1, "#d4d4d4")
        cv.text((0.05 + COL_R) / 2, (y0 + y1) / 2, name, size=8.5, kind="band")

    # ── 第 1 層（左→右）：輸入資料 → 兩個入口（上下疊）→ 協調介面 ──
    hh = 2 * H1 + 0.14
    ye1, ye2 = yc1 + (H1 + 0.14) / 2, yc1 - (H1 + 0.14) / 2
    d_txt = "輸入：作業區多邊形、機隊構型、\n高度上限 120 m、風、起降場坪"
    dw = cv.measure(d_txt)[0] + 0.20 + 0.25        # 靠左排：左距 0.20（box 內定）
    e_lab = ("AeroPlan 地面站（VTOL 偵察頁）", "情境腳本（驗證實驗）")
    ew = max(cv.measure(s)[0] for s in e_lab) + 0.40
    c_txt = "協調介面：機隊切區、\n空域與法規參數"
    cw = cv.measure(c_txt)[0] + 0.40
    g1 = (XR - XL - dw - ew - cw) / 2
    dcx = XL + dw / 2
    ecx = XL + dw + g1 + ew / 2
    ccx = XR - cw / 2
    cv.box("D", dcx, yc1, dw, hh, d_txt, ls=DASH, align="left")
    cv.box("E1", ecx, ye1, ew, H1, e_lab[0])
    cv.box("E2", ecx, ye2, ew, H1, e_lab[1])
    cv.box("C", ccx, yc1, cw, hh, c_txt)
    cv.arrow([A("D", "right", off=ye1 - yc1), A("E1", "left")], name="D-E1")
    cv.arrow([A("D", "right", off=ye2 - yc1), A("E2", "left")], name="D-E2")
    cv.arrow([A("E1", "right"), A("C", "left", off=ye1 - yc1)], name="E1-C")
    cv.arrow([A("E2", "right"), A("C", "left", off=ye2 - yc1)], name="E2-C", **TH)

    # ── 第 2 層：HARVEST 規劃器大框（六模組與圖2 主流程一一對應）──
    cv.band(XL, fy0, XR, fy1, "#e6e6e6", ec="black", lw=LW_THIN, z=1)
    mods = ["情境建構\n與前置檢查",
            "區域分解\n與高度分層",
            "考慮風場之飛行時間成本、\n構型與架數配置",
            "Beam-IDP 條帶序列\n與返場能量修復",
            "起降段組裝\n與四維去衝突",
            "整體驗證"]
    x_down = XR - 0.20
    row, _ = _row(cv, XL + 0.12, x_down - 0.20, mods, min_w=1.45, pad=0.30)
    for k, ((cx, w), lab) in enumerate(zip(row, mods)):
        cv.box(f"M{k + 1}", cx, my, w, H2, lab)
    for k in range(1, len(mods)):
        cv.arrow([A(f"M{k}", "right"), A(f"M{k + 1}", "left")], name=f"M{k}-M{k + 1}", **TH)
    x_c = ccx - cw / 2 + 0.30
    cv.arrow([A("C", "bottom", off=x_c - ccx), (x_c, y_run), (row[0][0], y_run), A("M1", "top")],
             name="C-M1", **TH)
    cv.text(XR - 0.12, y_run, "HARVEST 規劃器", size=9, ha="right", kind="title")
    sx0 = row[0][0] - row[0][1] / 2
    sx1 = row[-1][0] + row[-1][1] / 2
    cv.box("S", (sx0 + sx1) / 2, ys, sx1 - sx0, 0.50,
           "各模組共用輸入：平台與能量模型、風場與地形", ls=DASH)

    # ── 第 3 層：ArduPilot 執行與驗證 ──
    exe = ["匯出 QuadPlane\n任務與參數檔",
           "MAVLink 上傳至 ArduPlane\nQuadPlane SITL（×3）",
           "讀回檢核：參數逐項、\n任務項數；不符即停飛",
           "依規劃起飛\n時刻放行",
           "航跡回算覆蓋率（上界）\n與連續時間最小間距"]
    row3, _ = _row(cv, XL, XR, exe, min_w=1.45, pad=0.30)
    for k, ((cx, w), lab) in enumerate(zip(row3, exe)):
        cv.box(f"X{k + 1}", cx, y3, w, H2, lab)
    for k in range(1, len(exe)):
        cv.arrow([A(f"X{k}", "right"), A(f"X{k + 1}", "left")], name=f"X{k}-X{k + 1}", **TH)
    # 通道標籤嵌在線上：線在字的左右各斷開 LAB_GAP（> 取樣端點排除 0.05 + 字框外擴 0.03）
    LAB_GAP = 0.12
    t = cv.text((row3[1][0] + row3[3][0]) / 2, yc, "任務計畫與驗證報告", kind="label")
    lx0, _, lx1, _ = cv.extent(t)
    cv.arrow([A("M6", "right"), (x_down, my), (x_down, yc), (lx1 + LAB_GAP, yc)],
             head=False, name="M6-X1a", **TH)
    cv.arrow([(lx0 - LAB_GAP, yc), (row3[0][0], yc), A("X1", "top")], name="M6-X1b", **TH)

    # ── 圖例（層帶外，單行）──
    lx = XL
    for i, (lw, txt) in enumerate(((LW_THIN, "資料流"), (LW_THICK, "本文驗證路徑"))):
        cv.arrow([(lx, y_leg), (lx + 0.75, y_leg)], lw=lw, name=f"legend{i}")
        t = cv.text(lx + 0.90, y_leg, txt, ha="left", kind="legend")
        lx = cv.extent(t)[2] + 0.60
    return cv


def check_compact_size(cv: Canvas) -> list[str]:
    """扁平版的尺寸規格：寬 17.0 cm、高 ≤ 5.6 cm。"""
    probs = []
    if abs(cv.W - COMPACT_W) > 1e-9:
        probs.append(f"扁平版寬 {cv.W} cm ≠ {COMPACT_W} cm")
    if cv.H > COMPACT_H_MAX + 1e-9:
        probs.append(f"扁平版高 {cv.H} cm > {COMPACT_H_MAX} cm")
    return probs


# ════════════════════════════════════════════════════════════════════
#  圖2　規劃演算法流程（S 型三列）
# ════════════════════════════════════════════════════════════════════
def build_fig2(fonts: Fonts) -> Canvas:
    W, H = 17.0, 7.6
    cv = Canvas(W, H, fonts)
    ABN = {"fill": "#bfbfbf", "ls": DASHDOT, "lw": LW_THIN}
    TERM = {"fill": "white", "ls": "-", "lw": LW_THICK, "rounding": 0.30}
    A = cv.at

    # ── 第 1 列（左→右）──
    y1 = 6.88
    cv.box("B1", 1.575, y1, 2.95, 1.20, "原始輸入\n作業區、機隊、空域\n禁航區、風、起降場", **TERM)
    cv.box("B2", 4.705, y1, 1.95, 0.90, "機隊與禁航區\n切區")
    cv.box("B2b", 7.485, y1, 2.10, 0.90, "轉成規劃輸入")
    cv.box("B3", 10.265, y1, 1.95, 0.90, "情境建構與\n前置檢查")
    cv.diamond("B4", 13.07, y1, 2.00, 1.10, "可規劃？")
    cv.box("XE", 15.875, y1, 2.10, 1.00, "回報不可行\n（NO-GO）", **ABN)
    for a, b in (("B1", "B2"), ("B2", "B2b"), ("B2b", "B3"), ("B3", "B4")):
        cv.arrow([A(a, "right"), A(b, "left")], name=f"{a}-{b}")
    cv.arrow([A("B4", "right"), A("XE", "left")], ls=DASHDOT, name="B4-XE")
    cv.text((14.07 + 14.825) / 2, y1 + 0.22, "否", kind="label")

    # ── 第 2 列（右→左）──
    y2 = 4.30
    cv.box("XP", 15.90, y2, 2.05, 0.90, "回報未覆蓋區", **ABN)
    cv.box("B5", 12.60, y2, 2.55, 1.20, "凸分解、鋪條帶、\n高度分層（不足時\n改時間分離）")
    # λ 用 mathtext 斜體 9 pt；說明性下標 floor 為 8 pt 正體（ISO 80000-2），手動下移基線
    cv.box("B6", 8.641, y2, 4.00, 1.40,
           [[("考慮風場之飛行時間成本、", 8.0)],
            [("構型與架數配置", 8.0)],
            [("（決策級配額", 8.0), (r"$\lambda$", LAMBDA_PT, 0.05),
             ("floor", MIN_PT, 0.005, -SUB_DROP), ("）", 8.0, 0.02)]])
    cv.box("B7", 4.983, y2, 1.95, 0.90, "Beam-IDP\n條帶序列")
    cv.diamond("B8", 1.45, y2, 2.50, 1.50, "返場能量\n可行？")
    cv.arrow([A("B4", "bottom"), (13.07, y2 + 0.60)], name="B4-B5")
    cv.text(13.22, 5.60, "是", ha="left", kind="label")
    cv.arrow([A("B5", "right"), A("XP", "left")], ls=DASHDOT, name="B5-XP")
    cv.text((13.875 + 14.875) / 2, y2 + 0.24, "無條帶", kind="label")
    cv.arrow([A("B5", "left"), A("B6", "right")], name="B5-B6")
    cv.arrow([A("B6", "left"), A("B7", "right")], name="B6-B7")
    cv.arrow([A("B7", "left"), A("B8", "right")], name="B7-B8")
    y_loop = 5.35
    cv.arrow([A("B8", "top"), (1.45, y_loop), (4.983, y_loop), A("B7", "top")], name="B8-B7")
    cv.text(3.20, y_loop + 0.07, "否：能量修復（≤3 輪）\n仍不可行者標為未覆蓋", va="bottom", kind="label")

    # ── 第 3 列（左→右）──
    y3 = 1.45
    cv.box("B9", 1.15, y3, 2.00, 0.90, "起降段與\n三維剖面組裝")
    # 「四維」與同圖「三維剖面」、圖1「四維去衝突」及正文一致（v2 審查：4D／四維混用）
    cv.diamond("B10", 4.15, y3, 2.90, 1.60, "四維或時窗\n衝突？")
    # 每行結尾都帶 →（v2 審查：斷行後箭頭位置不一致）
    cv.box("B11", 7.725, y3, 3.20, 1.30, "延遲／換序→盤旋→\n重排序（≤3 輪）→\n工期壓縮→全域重排")
    cv.box("B12", 11.65, y3, 1.60, 0.90, "整體驗證")
    cv.box("B13", 15.75, y3, 2.35, 1.00, "匯出 QuadPlane\n任務與參數檔", **TERM)

    y_branch = 3.10
    cv.arrow([A("B8", "bottom"), A("B9", "top", off=0.30)], name="B8-B9")
    cv.text(1.33, 3.32, "是", ha="right", kind="label")
    cv.dot(1.45, y_branch)
    cv.arrow([(1.45, y_branch), (8.641, y_branch), A("B6", "bottom")], ls=DASH, name="B8-B6")
    # 分岔兩路都標條件（v2 審查：看起來像兩條同時執行）
    cv.text(5.0, y_branch - 0.07, "有剩餘條帶：再拍賣", va="top", kind="label")
    cv.text(1.33, 2.50, "無剩餘", ha="right", kind="label")
    cv.arrow([A("B9", "right"), A("B10", "left")], name="B9-B10")
    cv.arrow([A("B10", "right"), A("B11", "left")], name="B10-B11")
    cv.text((5.60 + 6.125) / 2, y3 + 0.07, "是", va="bottom", kind="label")
    cv.arrow([A("B11", "right"), A("B12", "left")], name="B11-B12")
    cv.text((9.325 + 10.85) / 2, y3 + 0.07, "殘餘衝突\n列入報告", va="bottom", kind="label")
    cv.arrow([A("B12", "right"), A("B13", "left")], name="B12-B13")
    cv.text((12.45 + 14.575) / 2, y3 + 0.07, "驗證報告\n隨計畫輸出", va="bottom", kind="label")
    y_by = 0.25
    cv.arrow([A("B10", "bottom"), (4.15, y_by), (11.65, y_by), A("B12", "bottom")], name="B10-B12")
    cv.text(4.00, 0.42, "否", ha="right", kind="label")

    # ── 圖例（左下空白處；v2 審查：線型意義沒有交代）──
    # 圖例框登記成方塊、文字歸它所有：「文字在框內」「方塊不重疊（與菱形 B10）」
    # 「標籤不壓框（否）」都由 check() 把關（第一輪目視：未登記時字貼框、框貼菱形）。
    # 線型樣本不登記為箭頭（否則會被判穿過圖例框），位置固定在文字左側。
    # 第二輪目視：「不可行／未覆蓋出口」讓框角離菱形 B10 只剩約 0.08 cm ⇒ 縮短字、收窄樣本
    legend = ["不可行／未覆蓋", "再分配回圈"]
    tw = max(cv.measure(s)[0] for s in legend)
    lx0, sw, sg = 0.08, 0.45, 0.10              # 框左緣、樣本寬、樣本與文字間距
    lw_box = 0.14 + sw + sg + tw + 0.16
    # 由實測字高推出列位置與框高：方塊內留白 0.12（check 的 pad）再多 0.015，兩列間距 0.30
    th = max(cv.measure(s)[1] for s in legend)
    yb = 0.05 + 0.135 + th / 2
    ya = yb + 0.30
    lh = (ya + th / 2 + 0.135) - 0.05
    cv.ax.add_patch(Rectangle((lx0, 0.05), lw_box, lh, facecolor="white",
                              edgecolor="#555555", lw=0.5, zorder=3))
    cv.nodes["LEG"] = {"shape": "rect", "cx": lx0 + lw_box / 2, "cy": 0.05 + lh / 2,
                       "w": lw_box, "h": lh}
    sx = lx0 + 0.14
    cv.ax.add_patch(Rectangle((sx, ya - 0.11), sw, 0.22, facecolor="#bfbfbf",
                              edgecolor="black", lw=LW_THIN, ls=DASHDOT, zorder=4))
    cv.ax.plot([sx, sx + sw], [yb, yb], color="black", lw=LW_THIN, ls=DASH, zorder=4,
               dash_capstyle="butt")
    for y, s in ((ya, legend[0]), (yb, legend[1])):
        cv.text(sx + sw + sg, y, s, ha="left", kind="node", owner="LEG")
    return cv


# ════════════════════════════════════════════════════════════════════
def save(cv: Canvas, stem: str) -> list[Path]:
    png, svg = OUT_DIR / f"{stem}.png", OUT_DIR / f"{stem}.svg"
    cv.fig.savefig(png, dpi=DPI, facecolor="white", metadata={"Software": None})
    cv.fig.savefig(svg, facecolor="white", metadata={"Date": None, "Creator": None})
    return [png, svg]


def selftest(fonts: Fonts) -> int:
    """故意弄壞版面，確認檢查器真的會報錯（檢查器本身也要被驗證）。"""
    fails = 0
    cases = []

    def node_text(cv, nid):
        return next(r["artist"] for r in cv.texts if r["owner"] == nid)

    cv = build_fig1(fonts)
    cases.append(("基準圖1應無問題", check(cv) == []))
    t_node = node_text(cv, "E2")
    old = t_node.get_text()
    t_node.set_text("情境腳本（驗證實驗很長很長很長）")
    cases.append(("方塊文字溢出", any("溢出" in p for p in check(cv))))
    t_node.set_text("情境腳本\nhv_sitl_run.py")
    got = check(cv)
    cases.append(("底線識別字", any("底線" in p for p in got)))
    cases.append(("禁用內部用語", any("內部用語" in p for p in got)))
    t_node.set_text(old)
    # 字型覆蓋：kaiu 與 TNR 都沒有的字（U+1F600）
    t_node.set_text("情境腳本\U0001F600")
    cases.append(("沒有字型的字元", any("沒有字型有這個字" in p for p in check(cv))))
    t_node.set_text(old)
    # 字型覆蓋：英數沒走 Times New Roman（模擬後備鏈順序錯：中文字型排在前面）
    saved = cv.fonts.cmaps()
    cv.fonts._cmaps = list(reversed(saved))
    cases.append(("英數沒走 TNR", any("Times New Roman" in p for p in check(cv))))
    cv.fonts._cmaps = saved
    t_lab = next(r["artist"] for r in cv.texts if r["artist"].get_text() == "任務計畫與驗證報告")
    pos = t_lab.get_position()
    corridor_y = next(e for e in cv.edges if e["name"] == "M6-X1")["pts"][-2][1]
    t_lab.set_position((pos[0], corridor_y))
    cases.append(("箭頭穿過文字", any("穿過文字" in p for p in check(cv))))
    t_lab.set_position((cv.W + 0.5, pos[1]))
    cases.append(("文字超出畫布", any("超出畫布" in p for p in check(cv))))
    t_lab.set_position(pos)
    t_lab.set_fontsize(6.5)
    cases.append(("字級過小", any("字級" in p for p in check(cv))))
    t_lab.set_fontsize(8)
    cases.append(("還原後圖1應無問題", check(cv) == []))
    cv.edges.append({"pts": [(0.3, 0.1), (0.3, 0.4)], "name": "直線甲", "head_len": 0.0})
    cv.edges.append({"pts": [(0.15, 0.25), (0.45, 0.25)], "name": "直線乙", "head_len": 0.0})
    cases.append(("箭頭交叉", any("箭頭交叉" in p and "直線甲" in p for p in check(cv))))
    del cv.edges[-2:]
    # E1、E2 原間距 0.28 cm；E1 下移 0.23 ⇒ 間距 0.05：沒重疊但低於 CLEAR
    cv.nodes["E1"]["cy"] -= 0.23
    got = check(cv)
    cases.append(("方塊間距不足（未重疊）",
                  any("方塊間距" in p for p in got) and not any("方塊重疊" in p for p in got)))
    cv.nodes["E1"]["cy"] += 0.23
    cv.nodes["E1"]["cy"] -= 0.60
    cases.append(("方塊重疊", any("重疊" in p for p in check(cv))))
    plt.close(cv.fig)

    cv2 = build_fig2(fonts)
    cases.append(("基準圖2應無問題", check(cv2) == []))
    b6 = [r for r in cv2.texts if r["owner"] == "B6"]
    t_math = next(r["artist"] for r in b6 if is_math(r["artist"].get_text()))
    t_sub = next(r["artist"] for r in b6 if r["artist"].get_text() == "floor")
    t_math.set_text(r"$\lambda_{\mathrm{floor}}$")
    cases.append(("mathtext 下標縮放後字太小",
                  any("含下標縮放" in p and "floor" in p for p in check(cv2))))
    t_math.set_text(r"$\mu$")
    cases.append(("未登記 mathtext 指令", any("mathtext 指令" in p for p in check(cv2))))
    t_math.set_text(r"$\lambda$")
    t_sub.set_fontsize(7.0)
    cases.append(("手動下標字太小", any("字級" in p and "floor" in p for p in check(cv2))))
    t_sub.set_fontsize(MIN_PT)
    r0 = next(r["artist"] for r in b6 if r["row"] == 0)
    p0 = r0.get_position()
    r0.set_position((p0[0], p0[1] - 0.20))
    cases.append(("同文字塊跨行重疊", any("文字重疊" in p for p in check(cv2))))
    r0.set_position(p0)
    t_b12 = node_text(cv2, "B12")
    t_b12.set_text("整體驗證 validate")
    cases.append(("圖2 禁用字 validate", any("validate" in p for p in check(cv2))))
    t_b12.set_text("整體驗證")
    cases.append(("還原後圖2應無問題", check(cv2) == []))
    cv2.edges.append({"pts": [(0.5, 7.2), (16.5, 0.5)], "name": "斜線", "head_len": 0.0})
    cases.append(("箭頭穿過方塊", any("穿過方塊" in p for p in check(cv2))))
    plt.close(cv2.fig)

    cv3 = build_fig1_compact(fonts)
    cases.append(("基準扁平圖1應無問題", check(cv3) == [] and check_compact_size(cv3) == []))
    # 嵌在通道線上的標籤：兩段線各自斷開；把標籤往左移到線段上必須被抓到
    t_lab = next(r["artist"] for r in cv3.texts if r["artist"].get_text() == "任務計畫與驗證報告")
    pos = t_lab.get_position()
    t_lab.set_position((pos[0] - 1.0, pos[1]))
    cases.append(("扁平版：嵌線標籤壓到線段", any("穿過文字" in p for p in check(cv3))))
    t_lab.set_position(pos)
    t_node = node_text(cv3, "X2")
    t_node.set_text("MAVLink 上傳至 ArduPlane\nQuadPlane SITL（×3）\n第三行")
    cases.append(("扁平版：三行溢出兩行高方塊", any("溢出" in p for p in check(cv3))))
    t_node.set_text("MAVLink 上傳至 ArduPlane\nQuadPlane SITL（×3）")
    cv3.H = COMPACT_H_MAX + 0.1
    cases.append(("扁平版：高度超過規格", any("扁平版高" in p for p in check_compact_size(cv3))))
    cv3.H = cv3.ax.get_ylim()[1]
    cases.append(("還原後扁平圖1應無問題", check(cv3) == [] and check_compact_size(cv3) == []))
    plt.close(cv3.fig)
    for name, ok in cases:
        print(f"  [{'OK ' if ok else 'BAD'}] {name}")
        fails += 0 if ok else 1
    return fails


def main() -> int:
    # v2 審查：本機主控台是 cp950，印 ✓／✗ 會 UnicodeEncodeError 而中斷產圖。
    # 狀態記號改用 ASCII；問題訊息可能含 cp950 沒有的字（如 U+1F600），
    # 保留主控台編碼、只把錯誤處理改成 replace，中文照常顯示、缺字印成 ?。
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--compact", action="store_true",
                    help="只產生圖1 扁平版 fig1_architecture_compact.png/svg")
    args = ap.parse_args()
    fonts = Fonts()
    for n in fonts.notes:
        print("WARN", n)
    print("字型鏈:", fonts.names)
    if args.selftest:
        return 1 if selftest(fonts) else 0
    rc = 0
    # 扁平版排在最後產生：不影響前兩張圖的建構順序與輸出位元組
    jobs = [("fig1_architecture", build_fig1), ("fig2_flowchart", build_fig2),
            ("fig1_architecture_compact", build_fig1_compact)]
    if args.compact:
        jobs = jobs[-1:]
    for stem, build in jobs:
        cv = build(fonts)
        probs = check(cv)
        if build is build_fig1_compact:
            probs += check_compact_size(cv)
        if probs:
            rc = 1
            print(f"FAIL {stem} 版面檢查 {len(probs)} 項：")
            for p in probs:
                print("   -", p)
            plt.close(cv.fig)
            continue
        for p in save(cv, stem):
            h = hashlib.sha256(p.read_bytes()).hexdigest()[:16]
            print(f"OK   {p.name}  sha256:{h}")
        plt.close(cv.fig)
    return rc


if __name__ == "__main__":
    plt.rcParams["svg.hashsalt"] = "aasrc2026"
    plt.rcParams["svg.fonttype"] = "path"
    plt.rcParams["axes.unicode_minus"] = False
    sys.exit(main())
