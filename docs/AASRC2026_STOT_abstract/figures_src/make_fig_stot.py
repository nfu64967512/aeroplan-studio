#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AASRC 2026 STOT 中文加長型摘要用圖：圖1 系統架構（扁平）、圖2 任務階段流程（S 型）。

用法::

    python docs/AASRC2026_STOT_abstract/figures_src/make_fig_stot.py
    python docs/AASRC2026_STOT_abstract/figures_src/make_fig_stot.py --selftest

輸出（本腳本上一層目錄）：
    fig1_architecture.png / .svg   17.0 cm × ≤5.4 cm、300 dpi
    fig2_flowchart.png    / .svg   17.0 cm × ≤7.0 cm、300 dpi
    17.0 cm = A4 版心寬，以 100% 原寸插入，字級即印刷字級。

版面檢查（每次產圖都跑；任一項不過就不存檔、非零結束）：
    * 方塊文字外框落在方塊（矩形留白 0.12 cm／菱形正規化 ≤0.97）內；
    * 箭頭折線不穿過任何文字外框、不穿過非端點方塊；箭頭彼此不交叉；
    * 文字彼此不重疊、標籤不壓方塊、全部落在畫布內；方塊間距 ≥0.10 cm；
    * 字級 ≥ 8 pt；每個字元都有字型，英數必須由 Times New Roman 提供，中文標楷體；
    * 投稿用字：不得有底線字元、程式識別字、錯誤或過度主張的用語、真實地名（見 FORBIDDEN）；
    * 圖面尺寸：寬 17.0 cm，高分別 ≤5.4 / ≤7.0 cm。
``--selftest`` 故意弄壞版面，確認每一項檢查真的會報錯。

版面引擎（Fonts／Canvas／check）複製改寫自
docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig_arch.py（未 import、未修改該檔）。

內容依據：STOT 已核對素材（審查紀錄 A1–A91、素材 B §0–§2），重點：
  圖1
    權限邊界在 PN 交接、不在放行：交接條件 d ≤ 交接距離連 3 拍（tot_node.py repo 4484-4508）；
      放行後、交接前是鎖存航點仍由 L1/TECS 執行（repo 4796-4797, 3360-3377）；
      交接後直連 5762+10i 送姿態目標（type_mask 132，repo 2345-2347），ArduPilot 姿態環與限幅仍在。
    MAVProxy 扇出 4 路（swarm_sitl_up.sh:431-442）；MAVROS 5860（swarm_ros2_up.sh:726）。
    長機最大值共識只在批次巡航段（master_node.py:89-91, 229-276；審查 A41）。
    讓權：相位 ∈ {ALIGN, PERCH, DIVE} 時編隊層停手（strike_authority.py:57,65,93-106）；
      批次路徑改為收掉編隊層（stot_trial.sh:460）。
    生效值契約讀回不符即中止（rep_interactive.sh:335-349；stot_trial.sh:241-244）。
    評分以首次觸地為基準（stot_score.py:234-315, 624-628）。
  圖2
    批次：彈射、爬升、編隊巡航（jet_catapult_launch.py；stot_trial.sh:306-461），巡航編隊未通過階段 2。
    互動：跑道起飛、待命節點收目標（rep_interactive.sh:352-371；tot_node.py repo 1649-1730）。
    扇形對齊（repo 3423-3437, 3936-3992）；自動航線分支預設關閉且從未飛過
      （部署版 tot_node.py:1117；handoff_2026-08-25_auto_mission.md:81）。
    爬升對正就緒 0.9（stot_trial.sh:500；swarm_ros2_up.sh:809）；同步起算 25 s（repo 4100-4154）。
    放行：預設 _tgo_of ≥ dive_T − now（repo 4378-4379）；抵環即放行為非預設（repo 638、4377），
      表1 之 RRD、RRE 與表2 代表航次採用（n=2）；RRB 為環外放行，86 趟母體僅 34 趟開啟。
    PN 交接 d ≤ gate 連 3 拍（repo 4484-4508）；命中鎖存／MISS（repo 4548-4585, 3625-3685）；
      觸地 disarm（repo 3736-3798）；中止（rep_interactive.sh、stot_trial.sh；部署版 5905-5952）。
"""
from __future__ import annotations

import argparse
import hashlib
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager as fm  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent.parent
CM = 1.0 / 2.54
DPI = 300
MIN_PT = 8.0
FIG_W = 17.0
FIG1_H_MAX = 5.4
FIG2_H_MAX = 7.0

FONT_KAI = Path("C:/Windows/Fonts/kaiu.ttf")      # 標楷體
FONT_TNR = Path("C:/Windows/Fonts/times.ttf")     # Times New Roman
FONT_JH = Path("C:/Windows/Fonts/msjh.ttc")       # 退路：微軟正黑體

# 投稿版不得出現的字（不分大小寫；另外一律禁止底線字元）
FORBIDDEN = (
    # 程式識別字與相位字串
    "tot", "node", "master", "wingman", "backup", "release", "ring", "dive", "perch",
    "runup", "runin", "align", "standby", "gate", "auto", "mission", "mavros interface",
    ".py", ".sh", ".json", "type mask", "attitude target", "reposition", "change speed",
    # 素材 §0.5 要避開的寫法
    "PPN", "純 PN", "純比例", "放行後即", "放行後直", "實飛", "已驗證", "部署版",
    # 真實地名與武器用語
    "虎尾", "雲林", "臺灣", "台灣", "Taiwan", "澎湖", "金門", "馬祖", "彈頭", "投彈",
    "武器", "酬載", "炸",
)

DASH = (0, (2.4, 1.4))    # 第一輪目視：(4, 2) 在 0.3 cm 短箭頭上看起來像實線
DASHDOT = (0, (5.0, 1.6, 1.2, 1.6))
LW_THIN = 0.9
LW_THICK = 1.8
GRAY_OFF = "#c8c8c8"      # 從未飛過（灰底）
BAND = "#f2f2f2"
BAND_COL = "#d4d4d4"


# ════════════════════════════════════════════════════════════════════
#  字型
# ════════════════════════════════════════════════════════════════════
class Fonts:
    def __init__(self) -> None:
        self.notes: list[str] = []
        cjk = FONT_KAI
        if not cjk.exists():
            cjk = FONT_JH
            self.notes.append(f"kaiu.ttf not found, CJK falls back to {FONT_JH.name}")
        latin = FONT_TNR if FONT_TNR.exists() else None
        if latin is None:
            self.notes.append("times.ttf not found, latin uses CJK font")
        self.paths = [p for p in (latin, cjk) if p is not None]
        self.names = []
        for p in self.paths:
            fm.fontManager.addfont(str(p))
            self.names.append(fm.FontProperties(fname=str(p)).get_name())
        self._cmaps = None

    def prop(self, size: float) -> fm.FontProperties:
        return fm.FontProperties(family=list(self.names), size=size)

    def cmaps(self):
        if self._cmaps is None:
            from fontTools.ttLib import TTFont
            out = []
            for p in self.paths:
                kw = {"fontNumber": 0} if p.suffix.lower() == ".ttc" else {}
                out.append((p.name, TTFont(str(p), lazy=True, **kw).getBestCmap()))
            self._cmaps = out
        return self._cmaps


# ════════════════════════════════════════════════════════════════════
#  畫布：以公分為資料座標，並登記所有物件供版面檢查
# ════════════════════════════════════════════════════════════════════
class Canvas:
    def __init__(self, w_cm: float, h_cm: float, fonts: Fonts, h_max: float) -> None:
        self.W, self.H, self.H_MAX, self.fonts = w_cm, h_cm, h_max, fonts
        self.fig = plt.figure(figsize=(w_cm * CM, h_cm * CM), dpi=DPI)
        self.ax = self.fig.add_axes((0, 0, 1, 1))
        self.ax.set_xlim(0, w_cm)
        self.ax.set_ylim(0, h_cm)
        self.ax.set_axis_off()
        self.nodes: dict[str, dict] = {}
        self.edges: list[dict] = []
        self.texts: list[dict] = []
        self.extra_probs: list[str] = []

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

    def text(self, x, y, s, size=8.0, ha="center", va="center", kind="label",
             owner=None, linespacing=1.12):
        t = self.ax.text(x, y, s, ha=ha, va=va,
                         multialignment=ha if ha in ("left", "right") else "center",
                         fontproperties=self.fonts.prop(size),
                         linespacing=linespacing, zorder=6, color="black")
        self.texts.append({"artist": t, "kind": kind, "owner": owner})
        return t

    def band(self, x0, y0, x1, y1, fc, ec="#7f7f7f", lw=0.5, ls="-", z=0):
        self.ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=fc,
                                    edgecolor=ec, lw=lw, ls=ls, zorder=z))

    def box(self, nid, cx, cy, w, h, label, fill="white", ls="-", lw=LW_THIN,
            rounding=0.06, size=8.0):
        p = FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                           boxstyle=f"round,pad=0,rounding_size={rounding}",
                           facecolor=fill, edgecolor="black", lw=lw, ls=ls, zorder=3)
        self.ax.add_patch(p)
        self.nodes[nid] = {"shape": "rect", "cx": cx, "cy": cy, "w": w, "h": h}
        self.text(cx, cy, label, size=size, kind="node", owner=nid)

    def boxlr(self, nid, x0, x1, cy, h, label, **kw):
        self.box(nid, (x0 + x1) / 2, cy, x1 - x0, h, label, **kw)

    def diamond(self, nid, cx, cy, w, h, label, fill="white", lw=LW_THIN, size=8.0):
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

    def L(self, nid):
        n = self.nodes[nid]
        return n["cx"] - n["w"] / 2

    def R(self, nid):
        n = self.nodes[nid]
        return n["cx"] + n["w"] / 2

    def T(self, nid):
        n = self.nodes[nid]
        return n["cy"] + n["h"] / 2

    def B(self, nid):
        n = self.nodes[nid]
        return n["cy"] - n["h"] / 2

    def arrow(self, pts, ls="-", lw=LW_THIN, head=True, name=""):
        pts = [tuple(map(float, p)) for p in pts]
        Lh, HW = (0.20, 0.085) if lw >= LW_THICK else (0.17, 0.068)
        draw = list(pts)
        if head:
            (x0, y0), (x1, y1) = pts[-2], pts[-1]
            d = math.hypot(x1 - x0, y1 - y0)
            ux, uy = (x1 - x0) / d, (y1 - y0) / d
            draw[-1] = (x1 - ux * Lh * 0.85, y1 - uy * Lh * 0.85)
            bx, by = x1 - ux * Lh, y1 - uy * Lh
            tri = [(x1, y1), (bx - uy * HW, by + ux * HW), (bx + uy * HW, by - ux * HW)]
            self.ax.add_patch(Polygon(tri, closed=True, facecolor="black",
                                      edgecolor="black", lw=0.3, zorder=5))
        xs, ys = zip(*draw)
        self.ax.plot(xs, ys, color="black", lw=lw, ls=ls, zorder=4,
                     solid_capstyle="butt", dash_capstyle="butt", solid_joinstyle="miter")
        self.edges.append({"pts": pts, "name": name, "head_len": Lh if head else 0.0})


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


def check(cv: Canvas) -> list[str]:
    probs: list[str] = list(cv.extra_probs)
    # 尺寸規格
    if abs(cv.W - FIG_W) > 1e-9:
        probs.append(f"圖寬 {cv.W} cm != {FIG_W} cm")
    if cv.H > cv.H_MAX + 1e-9:
        probs.append(f"圖高 {cv.H:.2f} cm > {cv.H_MAX} cm")
    px_w, px_h = cv.fig.get_size_inches() * DPI
    if abs(px_w / DPI * 2.54 - FIG_W) > 0.01:
        probs.append(f"輸出寬度換算 {px_w / DPI * 2.54:.3f} cm 不符")

    boxes = []
    cmaps = cv.fonts.cmaps()
    tnr = FONT_TNR.name if FONT_TNR in cv.fonts.paths else None
    for rec in cv.texts:
        t = rec["artist"]
        s = t.get_text()
        if not s:
            continue
        if t.get_fontsize() < MIN_PT - 1e-6:
            probs.append(f"字級 {t.get_fontsize():.2f} pt < {MIN_PT}：{s!r}")
        b = cv.extent(t)
        boxes.append((rec, b))
        if b[0] < 0.02 or b[1] < 0.02 or b[2] > cv.W - 0.02 or b[3] > cv.H - 0.02:
            probs.append(f"文字超出畫布：{s!r} {tuple(round(v, 2) for v in b)}")
        if "_" in s:
            probs.append(f"出現底線字元（識別字）：{s!r}")
        for word in FORBIDDEN:
            if word.lower() in s.lower():
                probs.append(f"出現禁用字 {word!r}：{s!r}")
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
        corners = ((b[0], b[1]), (b[0], b[3]), (b[2], b[1]), (b[2], b[3]))
        if n["shape"] == "rect":
            ok = all(_inside_node(n, x, y, shrink=0.12) for x, y in corners)
        else:
            cx, cy, hw, hh = n["cx"], n["cy"], n["w"] / 2, n["h"] / 2
            ok = max(abs(x - cx) / hw + abs(y - cy) / hh for x, y in corners) <= 0.97
        if not ok:
            probs.append(f"文字溢出方塊 {rec['owner']}：{rec['artist'].get_text()!r}")

    # 2) 箭頭不穿過文字、不穿過非端點方塊；最後一段放得下箭頭
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
        for x, y in e["pts"]:
            if x < 0.02 or y < 0.02 or x > cv.W - 0.02 or y > cv.H - 0.02:
                probs.append(f"箭頭 {e['name']} 超出畫布")

    # 3) 箭頭彼此交叉
    for i, a in enumerate(cv.edges):
        for bE in cv.edges[i + 1:]:
            for p1, p2 in zip(a["pts"], a["pts"][1:]):
                for q1, q2 in zip(bE["pts"], bE["pts"][1:]):
                    if _seg_cross(p1, p2, q1, q2):
                        probs.append(f"箭頭交叉：{a['name']} x {bE['name']}")

    # 4) 文字彼此不重疊；非方塊文字不壓到方塊
    for i, (ra, a) in enumerate(boxes):
        for rb, b in boxes[i + 1:]:
            if _rects_overlap(a, b, m=0.02):
                probs.append(f"文字重疊：{ra['artist'].get_text()!r} x {rb['artist'].get_text()!r}")
        if ra["kind"] == "node":
            continue
        for nid, n in cv.nodes.items():
            if any(_inside_node(n, x, y, shrink=-0.03) for x, y in _grid(a)):
                probs.append(f"標籤 {ra['artist'].get_text()!r} 壓到方塊 {nid}")

    # 5) 方塊不重疊、間距 ≥ CLEAR、落在畫布內
    CLEAR = 0.10
    ids = list(cv.nodes)
    for i, u in enumerate(ids):
        nu = cv.nodes[u]
        bu = (nu["cx"] - nu["w"] / 2, nu["cy"] - nu["h"] / 2,
              nu["cx"] + nu["w"] / 2, nu["cy"] + nu["h"] / 2)
        if bu[0] < 0.02 or bu[1] < 0.02 or bu[2] > cv.W - 0.02 or bu[3] > cv.H - 0.02:
            probs.append(f"方塊超出畫布：{u}")
        for v in ids[i + 1:]:
            nv = cv.nodes[v]
            pts_u = [(x, y) for x, y in _grid(bu, 40) if _inside_node(nu, x, y, shrink=-1e-6)]
            if any(_inside_node(nv, x, y) for x, y in pts_u):
                probs.append(f"方塊重疊：{u} x {v}")
            elif any(_inside_node(nv, x, y, shrink=-CLEAR) for x, y in pts_u):
                probs.append(f"方塊間距 < {CLEAR} cm：{u} x {v}")
    return probs


def wbox(cv: Canvas, s: str, pad=0.40) -> float:
    return cv.measure(s)[0] + pad


# ════════════════════════════════════════════════════════════════════
#  圖1　系統架構（四層水平帶，扁平版）
# ════════════════════════════════════════════════════════════════════
def build_fig1(fonts: Fonts) -> Canvas:
    W = FIG_W
    H2 = 0.85            # 兩行方塊高（實測兩行字高約 0.55，上下留白各 0.15）
    PADV = 0.175         # 方塊到層帶上下緣（第一輪目視：0.14 時層間虛線短到像實線）
    SEAM = 0.05          # 層帶之間白縫
    # 由下往上：圖例列、SITL、扇出、協同、地面站
    y_leg = 0.19
    rows = []
    b = 0.36
    for _ in range(4):
        yc = b + PADV + H2 / 2
        rows.append((b, yc, yc + H2 / 2 + PADV))
        b = rows[-1][2] + SEAM
    H = round(rows[-1][2] + 0.05, 3)
    (s0, y4, s1), (m0, y3, m1), (c0, y2, c1), (g0, y1, g1) = rows

    cv = Canvas(W, H, fonts, FIG1_H_MAX)
    A = cv.at
    TH = {"lw": LW_THICK}

    band_names = [("地面站\n與腳本", g0, g1), ("ROS 2\n協同層", c0, c1),
                  ("MAVLink\n扇出橋接", m0, m1), ("ArduPilot\nSITL", s0, s1)]
    col_w = max(cv.measure(n)[0] for n, _, _ in band_names)
    COL_R = 0.05 + col_w + 0.22
    XL, XR = COL_R + 0.12, W - 0.05 - 0.12
    for name, y0, y1b in band_names:
        cv.band(0.05, y0, W - 0.05, y1b, BAND)
        cv.band(0.05, y0, COL_R, y1b, BAND_COL)
        cv.text((0.05 + COL_R) / 2, (y0 + y1b) / 2, name, kind="band")

    # ── 協同層（決定全圖欄位）──
    t_mst = "長機速度最大值共識\n（僅批次巡航段）"
    t_wb = "僚機與\n備援長機"
    t_tot = "協同終端節點：時序排程、放行閘門、\n三維 PN（縱向含視線角下限）"
    t_yield = "讓權"
    mst_l, mst_r = XL, XL + wbox(cv, t_mst)
    wb_l = mst_r + 0.55
    wb_r = wb_l + wbox(cv, t_wb, pad=0.50)
    gy = max(1.00, cv.measure(t_yield)[0] + 0.50)
    tot_l = wb_r + gy
    tot_r = tot_l + wbox(cv, t_tot)
    cv.boxlr("MST", mst_l, mst_r, y2, H2, t_mst)
    cv.boxlr("WB", wb_l, wb_r, y2, H2, t_wb)
    cv.boxlr("TOT", tot_l, tot_r, y2, H2, t_tot)
    OFF = 0.18
    cv.arrow([A("MST", "right", OFF), A("WB", "left", OFF)], name="MST-WB")
    cv.arrow([A("WB", "left", -OFF), A("MST", "right", -OFF)], name="WB-MST")
    cv.arrow([A("TOT", "left"), A("WB", "right")], ls=DASH, name="TOT-WB")
    cv.text((wb_r + tot_l) / 2, y2 + 0.07, t_yield, va="bottom")

    # ── 權限邊界：PN 交接後直連姿態環（繞過扇出與 L1/TECS）──
    x_b = tot_r - 0.45
    t_by = "PN 交接後直連送姿態指令\n（放行至交接仍經 L1/TECS）"
    by_w = cv.measure(t_by)[0]
    if x_b + 0.15 + by_w > XR:
        cv.extra_probs.append(f"邊界標籤放不下：需 {x_b + 0.15 + by_w:.2f} > {XR:.2f}")

    # ── 扇出與橋接層 ──
    mp_r = x_b - 0.85
    cv.boxlr("MR", mst_l, mst_r, y3, H2, "MAVROS 橋接\n（位姿）")
    cv.boxlr("MP", wb_l, mp_r, y3, H2,
             "MAVProxy 各機 MAVLink 扇出：\n遙測上行；航點、速度、參數下行")
    cv.text(x_b + 0.15, y3, t_by, ha="left")

    # ── ArduPilot SITL 層 ──
    t_att = "姿態環\n（限幅仍在）"
    att_l = mp_r + 0.55
    att_r = max(att_l + wbox(cv, t_att), x_b + 0.45)
    cv.boxlr("MDL", mst_l, mst_r, y4, H2, "機體模型\n參數檔")
    cv.boxlr("FC", wb_l, mp_r, y4, H2, "物理模擬、EKF 狀態估計、\nL1/TECS 位置與速度環")
    cv.boxlr("ATT", att_l, att_r, y4, H2, t_att)
    cv.arrow([A("MDL", "right"), A("FC", "left")], name="MDL-FC")
    cv.arrow([A("FC", "right", OFF), A("ATT", "left", OFF)], name="FC-ATT")
    cv.arrow([A("ATT", "left", -OFF), A("FC", "right", -OFF)], name="ATT-FC")

    # ── 地面站與腳本層 ──
    t_sh = "啟動腳本：批次（彈射、編隊巡航）\n互動（跑道起飛、待命收目標）"
    t_gcs = "地面站：指定\n海上靜止目標"
    t_ct = "生效值契約讀回\n（不符即中止）"
    # 審查 B19：正文與表1 只報地速（總速依估計法差可達 15 m/s）
    t_sc = "離線評分：首次觸地的\n誤差、散布、地速"
    cv.boxlr("SH", mst_l, wb_r, y1, H2, t_sh)
    if wbox(cv, t_sh) > wb_r - mst_l:
        cv.extra_probs.append("啟動腳本方塊寬度不足")
    gcs_r = tot_l + wbox(cv, t_gcs)
    cv.boxlr("GCS", tot_l, gcs_r, y1, H2, t_gcs)
    ct_l = gcs_r + 0.45
    ct_r = ct_l + wbox(cv, t_ct)
    cv.boxlr("CT", ct_l, ct_r, y1, H2, t_ct)
    sc_r = XR
    sc_l = sc_r - wbox(cv, t_sc)
    cv.boxlr("SC", sc_l, sc_r, y1, H2, t_sc)
    if ct_l > tot_r - 0.5 or sc_l < tot_r + 0.3 or sc_l < ct_r + 0.3:
        cv.extra_probs.append("地面站層欄位與終端節點對不上")

    # ── 垂直連線 ──
    for nid in ("MST", "WB"):
        cx = cv.nodes[nid]["cx"]
        cv.arrow([(cx, cv.B("SH")), (cx, cv.T(nid))], ls=DASH, name=f"SH-{nid}")
    gcs_cx = cv.nodes["GCS"]["cx"]
    cv.arrow([(gcs_cx - 0.35, cv.B("GCS")), (gcs_cx - 0.35, cv.T("TOT"))], name="GCS-TOT")
    cv.arrow([(gcs_cx + 0.35, cv.T("TOT")), (gcs_cx + 0.35, cv.B("GCS"))], name="TOT-GCS")
    x_ct = (ct_l + min(ct_r, tot_r)) / 2
    cv.arrow([(x_ct, cv.T("TOT")), (x_ct, cv.B("CT"))], name="TOT-CT")
    x_sc = max(sc_l + 0.45, tot_r + 0.45)
    cv.arrow([A("TOT", "right", -OFF), (x_sc, y2 - OFF), (x_sc, cv.B("SC"))], name="TOT-SC")

    wb_cx = cv.nodes["WB"]["cx"]
    cv.arrow([(wb_cx, cv.B("WB")), (wb_cx, cv.T("MP"))], name="WB-MP")
    cv.arrow([A("MP", "left"), A("MR", "right")], name="MP-MR")
    mr_cx = cv.nodes["MR"]["cx"]
    cv.arrow([(mr_cx, cv.T("MR")), (mr_cx, cv.B("MST"))], name="MR-MST")
    tx1, tx2 = tot_l + 0.45, tot_l + 1.10
    if tx2 > mp_r - 0.25:
        cv.extra_probs.append("扇出方塊太窄，終端節點連線落不下")
    cv.arrow([(tx1, cv.B("TOT")), (tx1, cv.T("MP"))], name="TOT-MP")
    cv.arrow([(tx2, cv.T("MP")), (tx2, cv.B("TOT"))], name="MP-TOT")
    cv.arrow([(x_b, cv.B("TOT")), (x_b, cv.T("ATT"))], name="TOT-ATT", **TH)
    cv.arrow([(tx1, cv.B("MP")), (tx1, cv.T("FC"))], name="MP-FC")
    cv.arrow([(wb_cx, cv.T("FC")), (wb_cx, cv.B("MP"))], name="FC-MP")

    # ── 圖例（層帶外，單行）──
    lx = XL
    for i, (lw, ls, txt) in enumerate(((LW_THIN, "-", "指令與資料流"),
                                       (LW_THICK, "-", "權限邊界：PN 交接後直連姿態指令"),
                                       (LW_THIN, DASH, "讓權或收掉編隊層"))):
        cv.arrow([(lx, y_leg), (lx + 0.75, y_leg)], lw=lw, ls=ls, name=f"legend{i}")
        t = cv.text(lx + 0.90, y_leg, txt, ha="left", kind="legend")
        lx = cv.extent(t)[2] + 0.60
    return cv


# ════════════════════════════════════════════════════════════════════
#  圖2　任務階段流程（S 型三列＋頂部支線）
# ════════════════════════════════════════════════════════════════════
def build_fig2(fonts: Fonts) -> Canvas:
    W, H = FIG_W, 7.0
    cv = Canvas(W, H, fonts, FIG2_H_MAX)
    A = cv.at
    H2 = 0.85
    XL, XR = 0.10, W - 0.10
    TERM = {"lw": LW_THICK, "rounding": 0.28}
    OFFB = {"fill": GRAY_OFF, "ls": DASH}           # 預設關閉且從未飛過
    y_sub, y1, y2, y3, y_leg = 6.47, 5.45, 3.55, 1.72, 0.20

    # ── 第 1 列（左→右）：兩條啟動路徑 → 扇形對齊 → 自動航線？ → 爬升對正 ──
    t_bat = "批次路徑：彈射、爬升、\n編隊巡航（未通過驗收）"
    t_int = "互動路徑：跑道起飛，\n待命節點接收目標"
    st_w = max(wbox(cv, t_bat), wbox(cv, t_int))
    cv.boxlr("BAT", XL, XL + st_w, y_sub, H2, t_bat, **TERM)
    cv.boxlr("INT", XL, XL + st_w, y1, H2, t_int, **TERM)
    t_pos = "全員定位"
    al_l = XL + st_w + cv.measure(t_pos)[0] + 0.50
    # 審查 M9：蛇形在 v_req < v_min（時間有餘）時才進入（repo 3953-3975）；B21：用語同正文
    t_al = "扇形對齊：共同抵達槽位\n時刻（時間有餘則蛇形）"
    al_r = al_l + wbox(cv, t_al)
    cv.boxlr("ALIGN", al_l, al_r, y1, H2, t_al)
    cv.arrow([A("INT", "right"), A("ALIGN", "left")], name="INT-ALIGN")
    cv.text((cv.R("INT") + al_l) / 2, y1 + 0.07, t_pos, va="bottom")
    x_drop = al_r - 0.50
    cv.arrow([A("BAT", "right"), (x_drop, y_sub), (x_drop, cv.T("ALIGN"))], name="BAT-ALIGN")
    # 審查 M10：巡航段的交棒距離（HANDOFF_D）稱「移交距離」，與 PN「交接距離」區分
    cv.text((cv.R("BAT") + x_drop) / 2, y_sub + 0.07, "抵移交距離，收掉編隊層", va="bottom")

    t_slot = "全員\n抵槽位"
    DW, DH = 2.40, 1.20
    d_cx = al_r + cv.measure(t_slot)[0] + 0.50 + DW / 2
    cv.diamond("DAUTO", d_cx, y1, DW, DH, "自動航線\n開啟？")
    cv.arrow([A("ALIGN", "right"), A("DAUTO", "left")], name="ALIGN-DAUTO")
    cv.text((al_r + d_cx - DW / 2) / 2, y1 + 0.07, t_slot, va="bottom")

    t_pe = "爬升對正：爬至\n俯衝起始高度"
    pe_l = XR - wbox(cv, t_pe)
    cv.boxlr("PERCH", pe_l, XR, y1, H2, t_pe)
    cv.arrow([A("DAUTO", "right"), A("PERCH", "left")], name="DAUTO-PERCH")
    cv.text((d_cx + DW / 2 + pe_l) / 2, y1 + 0.07, "否（預設）", va="bottom")

    t_auto = "自動航線任務\n（預設關閉、未飛過）"
    au_l = d_cx + 0.80
    au_r = au_l + wbox(cv, t_auto)
    cv.boxlr("AUTO", au_l, au_r, y_sub, H2, t_auto, **OFFB)
    cv.arrow([A("DAUTO", "top"), (d_cx, y_sub), A("AUTO", "left")], ls=DASH, name="DAUTO-AUTO")
    cv.text((d_cx + au_l) / 2, y_sub + 0.07, "是", va="bottom")
    x_p = max(au_r + 0.40, pe_l + 0.40)
    if x_p > XR - 0.30:
        cv.extra_probs.append("自動航線方塊與爬升對正方塊對不上")
    cv.arrow([A("AUTO", "right"), (x_p, y_sub), (x_p, cv.T("PERCH"))], ls=DASH, name="AUTO-PERCH")

    # ── 第 2 列（右→左）：同步起算 → 內切等待 → 放行閘門 → 放行後航點俯衝 ──
    t_ru = "同步起算內切，\n送俯衝飛控參數"
    ru_l = XR - wbox(cv, t_ru)
    cv.boxlr("RUNUP", ru_l, XR, y2, H2, t_ru)
    x_pr = max(pe_l, ru_l) + 0.45
    cv.arrow([(x_pr, cv.B("PERCH")), (x_pr, cv.T("RUNUP"))], name="PERCH-RUNUP")
    cv.text(x_pr - 0.12, (cv.B("PERCH") + cv.T("RUNUP")) / 2, "全員就緒", ha="right")

    t_wait = "等滿 25 s\n或高度收斂"
    # 審查 M5：配速（ring_pace）預設關閉，RRD、RRE、LAT1、RA1 皆未開
    t_ri = "內切等待至抵環\n（配速為選項）"
    ri_r = ru_l - (cv.measure(t_wait)[0] + 0.50)
    ri_l = ri_r - wbox(cv, t_ri)
    cv.boxlr("RUNIN", ri_l, ri_r, y2, H2, t_ri)
    cv.arrow([A("RUNUP", "left"), A("RUNIN", "right")], name="RUNUP-RUNIN")
    cv.text((ri_r + ru_l) / 2, y2 + 0.07, t_wait, va="bottom")

    GW, GH = 2.90, 1.30
    g_cx = ri_l - 0.70 - GW / 2
    cv.diamond("GATE", g_cx, y2, GW, GH, "抵放行環且\n可放行？")
    cv.arrow([A("RUNIN", "left"), A("GATE", "right")], name="RUNIN-GATE")
    y_loop = y2 + GH / 2 + 0.35
    x_ri = ri_l + 0.50
    cv.arrow([A("GATE", "top"), (g_cx, y_loop), (x_ri, y_loop), (x_ri, cv.T("RUNIN"))],
             name="GATE-RUNIN")
    cv.text((g_cx + x_ri) / 2, y_loop + 0.07, "否：未抵環或未到放行時機", va="bottom")

    # 第二輪目視：「≥ 至排定命中剩餘」缺受詞；對應 repo 4378-4379 的 _tgo_of ≥ dive_T − now
    t_def = "預設：預測俯衝耗時\n≥ 距排定命中時刻"
    t_wp = "放行後鎖存航點俯衝\n（仍由 L1/TECS 執行）"
    wp_r = g_cx - GW / 2 - (cv.measure(t_def)[0] + 0.50)
    wp_l = wp_r - wbox(cv, t_wp)
    if wp_l < XL - 1e-9:
        cv.extra_probs.append(f"第 2 列放不下：航點俯衝方塊左緣 {wp_l:.2f} < {XL}")
    cv.boxlr("WPDIVE", wp_l, wp_r, y2, H2, t_wp)
    cv.arrow([A("GATE", "left"), A("WPDIVE", "right")], name="GATE-WPDIVE")
    cv.text((wp_r + g_cx - GW / 2) / 2, y2 + 0.07, t_def, va="bottom")
    # 抵環即放行：非預設 ⇒ 虛線。表1 之 RRD、RRE 與表2 代表航次採用；RRB（表1、圖3(a)）
    # 未開 release_at_ring（改開 tgo_from_predictor），86 趟母體僅 34 趟開啟（審查 M4）
    y_d = y2 - GH / 2 - 0.30
    x_dv = wp_r + 0.45
    cv.arrow([A("GATE", "bottom"), (g_cx, y_d), (x_dv, y_d), (x_dv, y2 - 0.20),
              A("WPDIVE", "right", -0.20)], ls=DASH, name="GATE-WPDIVE-ring")
    cv.text((x_dv + g_cx) / 2, y_d - 0.07, "抵環即放行（預設關閉，\n本文主要航次採用）", va="top")

    # ── 第 3 列（左→右）：三維 PN → 撞擊或未命中？ → 結案；中止 ──
    t_pn = "三維 PN 姿態導引\n（縱向含視線角下限）"
    pn_r = XL + wbox(cv, t_pn)
    cv.boxlr("PN", XL, pn_r, y3, H2, t_pn)
    x_h = (max(wp_l, XL) + min(wp_r, pn_r)) / 2
    cv.arrow([(x_h, cv.B("WPDIVE")), (x_h, cv.T("PN"))], name="WPDIVE-PN")
    cv.text(x_h + 0.12, (cv.B("WPDIVE") + cv.T("PN")) / 2, "交接距離內\n連續 3 拍", ha="left")

    TW, THd = 2.40, 1.20
    t_cx = 8.50
    # 審查 B20：正文「命中」＝觸地誤差 ≤10 m；節點鎖存（d < impact_reach 或過最近點）不是驗收判準
    cv.diamond("TD", t_cx, y3, TW, THd, "撞擊或\n未命中？")
    cv.arrow([A("PN", "right"), A("TD", "left")], name="PN-TD")
    # 第二輪目視：「結案；互動／路徑回待命」斷詞難讀 ⇒ 收窄標籤，讓結案方塊放得下完整片語
    t_hit = "撞擊鎖存（非\n驗收判準）；\n觸地即上鎖"
    dn_l = t_cx + TW / 2 + cv.measure(t_hit)[0] + 0.50
    t_dn = "結案\n（互動路徑回待命）"
    dn_r = dn_l + wbox(cv, t_dn)
    cv.boxlr("DONE", dn_l, dn_r, y3, H2, t_dn, **TERM)
    cv.arrow([A("TD", "right"), A("DONE", "left")], name="TD-DONE")
    cv.text((t_cx + TW / 2 + dn_l) / 2, y3 + 0.07, t_hit, va="bottom")
    y_m = y3 - THd / 2 - 0.30
    dn_cx = cv.nodes["DONE"]["cx"]
    cv.arrow([A("TD", "bottom"), (t_cx, y_m), (dn_cx, y_m), A("DONE", "bottom")], name="TD-DONE-miss")
    cv.text((t_cx + dn_cx) / 2, y_m - 0.07, "未命中：逃逸、停滯或逾時，原地盤旋", va="top")

    t_ab = "中止：記錄\n失敗原因"
    ab_l = XR - wbox(cv, t_ab)
    cv.boxlr("ABORT", ab_l, XR, y3, H2, t_ab, ls=DASHDOT)
    x_ab = max(ab_l, ru_l) + 0.45
    cv.arrow([(x_ab, cv.B("RUNUP")), (x_ab, cv.T("ABORT"))], ls=DASHDOT, name="RUNUP-ABORT")
    cv.text(x_ab - 0.12, (cv.B("RUNUP") + cv.T("ABORT")) / 2, "任一階段逾時\n或存活架數不足", ha="right")

    # ── 圖例（最下一列）：樣本不登記為方塊／箭頭，文字登記為 legend 標籤 ──
    lx = XL
    for kind, txt in (("box", "灰底虛框：預設關閉且從未 SITL 飛行"),
                      ("dash", "虛線：預設關閉的分支"),
                      ("dashdot", "點劃線：中止路徑")):
        if kind == "box":
            cv.ax.add_patch(Rectangle((lx, y_leg - 0.12), 0.60, 0.24, facecolor=GRAY_OFF,
                                      edgecolor="black", lw=LW_THIN, ls=DASH, zorder=4))
        else:
            cv.ax.plot([lx, lx + 0.60], [y_leg, y_leg], color="black", lw=LW_THIN,
                       ls=DASH if kind == "dash" else DASHDOT, zorder=4, dash_capstyle="butt")
        t = cv.text(lx + 0.75, y_leg, txt, ha="left", kind="legend")
        lx = cv.extent(t)[2] + 0.60
    return cv


# ════════════════════════════════════════════════════════════════════
def save(cv: Canvas, stem: str) -> list[Path]:
    png, svg = OUT_DIR / f"{stem}.png", OUT_DIR / f"{stem}.svg"
    cv.fig.savefig(png, dpi=DPI, facecolor="white", metadata={"Software": None})
    cv.fig.savefig(svg, facecolor="white", metadata={"Date": None, "Creator": None})
    return [png, svg]


def selftest(fonts: Fonts) -> int:
    """故意弄壞版面，確認檢查器真的會報錯。"""
    cases = []

    def node_text(cv, nid):
        return next(r["artist"] for r in cv.texts if r["owner"] == nid)

    def has(cv, key):
        return any(key in p for p in check(cv))

    cv = build_fig1(fonts)
    cases.append(("基準圖1應無問題", check(cv) == []))
    t = node_text(cv, "WB")
    old = t.get_text()
    t.set_text("僚機與備援長機很長很長")
    cases.append(("方塊文字溢出", has(cv, "溢出")))
    t.set_text("tot_node")
    got = check(cv)
    cases.append(("底線識別字", any("底線" in p for p in got)))
    cases.append(("禁用識別字", any("禁用字" in p for p in got)))
    t.set_text("放行後即\n姿態控制")
    cases.append(("禁用錯誤寫法", has(cv, "禁用字")))
    t.set_text("僚機\U0001F600")
    cases.append(("沒有字型的字元", has(cv, "沒有字型有這個字")))
    t.set_text(old)
    saved = cv.fonts.cmaps()
    cv.fonts._cmaps = list(reversed(saved))
    cases.append(("英數沒走 TNR", has(cv, "Times New Roman")))
    cv.fonts._cmaps = saved
    t_lab = next(r["artist"] for r in cv.texts if r["artist"].get_text() == "讓權")
    pos = t_lab.get_position()
    t_lab.set_position((pos[0], cv.nodes["TOT"]["cy"]))
    cases.append(("箭頭穿過文字", has(cv, "穿過文字")))
    t_lab.set_position((cv.W + 0.5, pos[1]))
    cases.append(("文字超出畫布", has(cv, "超出畫布")))
    t_lab.set_position(pos)
    t_lab.set_fontsize(7.5)
    cases.append(("字級過小", has(cv, "字級")))
    t_lab.set_fontsize(8)
    cv.edges.append({"pts": [(0.3, 0.1), (0.3, 0.3)], "name": "直線甲", "head_len": 0.0})
    cv.edges.append({"pts": [(0.2, 0.2), (0.4, 0.2)], "name": "直線乙", "head_len": 0.0})
    cases.append(("箭頭交叉", has(cv, "直線甲 x 直線乙")))
    del cv.edges[-2:]
    cv.nodes["MST"]["cy"] -= 0.40
    cases.append(("方塊重疊", has(cv, "方塊重疊")))
    cv.nodes["MST"]["cy"] += 0.40
    cv.H_MAX = cv.H - 0.1
    cases.append(("圖高超過規格", has(cv, "圖高")))
    cv.H_MAX = FIG1_H_MAX
    cases.append(("還原後圖1應無問題", check(cv) == []))
    plt.close(cv.fig)

    cv2 = build_fig2(fonts)
    cases.append(("基準圖2應無問題", check(cv2) == []))
    t = node_text(cv2, "GATE")
    old = t.get_text()
    t.set_text("抵放行環且可放行？")
    cases.append(("菱形文字溢出", has(cv2, "溢出方塊 GATE")))
    t.set_text(old)
    cv2.edges.append({"pts": [(0.5, 6.9), (16.5, 0.5)], "name": "斜線", "head_len": 0.0})
    cases.append(("箭頭穿過方塊", has(cv2, "穿過方塊")))
    del cv2.edges[-1]
    lab = next(r["artist"] for r in cv2.texts if r["artist"].get_text().startswith("否："))
    p = lab.get_position()
    lab.set_position((p[0], p[1] + 0.35))
    cases.append(("標籤壓到方塊或文字", has(cv2, "壓到方塊") or has(cv2, "文字重疊")))
    lab.set_position(p)
    cases.append(("還原後圖2應無問題", check(cv2) == []))
    plt.close(cv2.fig)

    fails = 0
    for name, ok in cases:
        print(f"  [{'OK ' if ok else 'BAD'}] {name}")
        fails += 0 if ok else 1
    return fails


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    fonts = Fonts()
    for n in fonts.notes:
        print("WARN", n)
    print("fonts:", fonts.names)
    if args.selftest:
        return 1 if selftest(fonts) else 0
    rc = 0
    for stem, build in (("fig1_architecture", build_fig1), ("fig2_flowchart", build_fig2)):
        cv = build(fonts)
        probs = check(cv)
        if probs:
            rc = 1
            print(f"FAIL {stem}: {len(probs)} layout problems")
            for p in probs:
                print("   -", p)
            plt.close(cv.fig)
            continue
        for p in save(cv, stem):
            h = hashlib.sha256(p.read_bytes()).hexdigest()
            print(f"OK   {p.name}  {cv.W:.2f}x{cv.H:.2f} cm  sha256:{h}")
        plt.close(cv.fig)
    return rc


if __name__ == "__main__":
    plt.rcParams["svg.hashsalt"] = "aasrc2026-stot"
    plt.rcParams["svg.fonttype"] = "path"
    plt.rcParams["axes.unicode_minus"] = False
    sys.exit(main())
