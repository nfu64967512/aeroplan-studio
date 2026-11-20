#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AASRC 2026：圖2 HARVEST 規劃演算法流程（研討會正式版）。

依使用者 2026-09-25 提供之草圖重排（兩列 S 型；粗框為本文提出之模組），
內容逐字沿用草圖，只改版面與字型：

    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig2_harvest_flow.py
    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig2_harvest_flow.py --selftest

輸出（本腳本上一層目錄）：
    fig2_harvest_flow.pdf   向量、字型內嵌（Type 42 子集），建議優先使用
    fig2_harvest_flow.svg   向量、文字轉外框（不依賴對方電腦字型）
    fig2_harvest_flow.png   600 dpi 點陣，Word「插入圖片」用
    寬 17.0 cm = A4 扣左右各 2 cm 的版心寬（投稿格式檔規定），插入時不縮放，字級即印刷字級。

字型（投稿格式：中文一律標楷體、英文 Times New Roman）：
    中文 kaiu.ttf；英數 times.ttf／timesi.ttf（變數斜體）。
    ⌈ ⌉ 兩套字型都沒有，不引入第三套字型：取 Times New Roman 的「[」「]」字形，
    裁掉下方橫筆即得 ⌈ ⌉（筆畫粗細、高度與同行 TNR 一致）。
    公式寫法與正文 2.3 節相同（render_word 的行內數學：拉丁字母斜體、下標 eff 斜體、
    「=」兩側留空）。

版面檢查沿用 make_fig_arch.check()（該檢查器已有 --selftest 逐項驗證會報錯），
另加本圖自己的三項：⌈⌉ 落在方塊內且不壓到相鄰字、公式各片段互不重疊、
全圖只用到 kaiu／times／timesi 三個字型檔。任一項不過就不存檔、非零結束。
"""
from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager as fm  # noqa: E402
from matplotlib.patches import FancyBboxPatch, PathPatch, Polygon, Rectangle  # noqa: E402
from matplotlib.textpath import TextPath  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_fig_arch as fa  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent.parent
STEM = "fig2_harvest_flow"
W = 17.0
PNG_DPI = 600

FS = 9.0            # 方塊內文字
FS_LAB = 8.5        # 箭頭旁條件字、圖例、底部說明
FS_MATH = 10.0      # 公式拉丁字母（斜體 TNR 與 9 pt 楷書同行時，視覺大小相當）
FS_SUB = 8.0        # 下標 eff（≥ 8 pt 的全圖下限；0.8 倍）
SUB_DROP = 0.065    # 下標基線下移 cm（≈ 1.8 pt）

PAD_X = 0.26        # 文字到框的左右留白 cm
PAD_Y = 0.17        # 上下留白 cm（check 要求 ≥ 0.12）
MARGIN = 0.10       # 畫布四周留白
GAP_UP = 0.50       # 列 1 → 上方出口框的箭頭長
GAP_ROWS = 0.72     # 列 1 底與列 2 最高物件（菱形頂點）之間：放再分配回圈線與標籤
D2_H = 1.55         # 「返場能量／可行？」菱形高
GAP_NOTE = 0.22     # 列 2 與底部說明之間
LOOP_OFF = 0.55     # 列 1 右端方塊底部：回圈進入點與往下出口各偏離中心的量

GREY = "#d9d9d9"
THICK = {"lw": fa.LW_THICK}
TERM = {"rounding": 0.28}
EXIT = {"fill": GREY, "ls": fa.DASHDOT}

TXT = {
    "N1": "原始輸入\n作業區、機隊、風、\n禁航區、起降場",
    "N2": "情境建構與\n前置檢查",
    "D1": "可規劃？",
    "N3": "區域分解、鋪條帶\n與高度分層",
    "N4": "含風 Dubins\n時間成本\n決策級機數配額",
    "E1": "回報不可行（NO-GO）",
    "E2": "回報未覆蓋區",
    "N5a": "Beam-IDP 條帶序列",
    "N5b": "跳帶數",
    "D2": "返場能量\n可行？",
    "N6": "起降時窗互斥與\n四維隔離去衝突",
    "N7": "整體驗證",
    "N8": "匯出 QuadPlane\n任務與參數檔",
    "NOTE": "再分配回圈：能量修復（≤3 輪）、剩餘條帶再拍賣；"
            "衝突消解序：延遲→換序→盤旋→重排序→工期壓縮→全域重排",
    "LEG": ("本文提出之模組", "終止出口", "再分配回圈"),
}

TNR_I = fm.FontProperties(fname=str(fa.FONT_TNR_I))
TNR_R = fm.FontProperties(fname=str(fa.FONT_TNR))
ALLOWED_FONT_FILES = {fa.FONT_KAI.name.lower(), fa.FONT_TNR.name.lower(),
                      fa.FONT_TNR_I.name.lower()}


# ════════════════════════════════════════════════════════════════════
#  量測
# ════════════════════════════════════════════════════════════════════
def text_wh(cv: fa.Canvas, s: str, size: float) -> tuple[float, float]:
    return cv.measure(s, size)


def rect_size(cv, s, size=FS):
    w, h = text_wh(cv, s, size)
    return w + 2 * PAD_X, h + 2 * PAD_Y


def diamond_size(cv, s, height, size=FS, fill=0.80):
    """給定菱形高，求讓文字外框四角落在 |x|/a+|y|/b ≤ fill 的最小寬。"""
    w, h = text_wh(cv, s, size)
    return w / (fill - h / height), height


# ════════════════════════════════════════════════════════════════════
#  公式列：跳帶數 k = ⌈2R_eff/s⌉
# ════════════════════════════════════════════════════════════════════
def _bracket_path(ch: str, size: float):
    """TNR 的「[」或「]」字形，回傳（Path 以 cm 計、以基線為 y=0, 裁切高度 y_cut）。"""
    tp = TextPath((0, 0), ch, size=size, prop=TNR_R)
    path = tp.transformed(Affine2D().scale(fa.PT_CM))
    v = path.vertices
    ymin = v[:, 1].min()
    ys = sorted({round(y, 5) for y in v[:, 1] if y < ymin + 0.25 * (v[:, 1].max() - ymin)})
    bar = ys[1] - ys[0]                       # 下方橫筆厚度
    return path, ymin + bar


class Formula:
    """公式片段與括號，由 lay() 以基線座標排版；片段也登記進 cv.texts 供 check()。"""

    def __init__(self, cv: fa.Canvas, owner: str, group: str, row: int):
        self.cv, self.owner, self.group, self.row = cv, owner, group, row
        self.items = []      # (kind, obj, gap_before, dy)

    def text(self, s, prop=None, size=FS_MATH, gap=0.0, dy=0.0):
        kw = {"fontproperties": fa.Fonts.prop(self.cv.fonts, size)} if prop is None else \
             {"fontproperties": prop.copy()}
        if prop is not None:
            kw["fontproperties"].set_size(size)
        t = self.cv.ax.text(0, 0, s, ha="left", va="baseline", zorder=6, color="black", **kw)
        self.cv.texts.append({"artist": t, "kind": "node", "owner": self.owner,
                              "group": self.group, "row": self.row})
        self.items.append(("t", t, gap, dy))

    def bracket(self, ch, gap=0.0):
        path, y_cut = _bracket_path(ch, FS_MATH)
        self.items.append(("b", (path, y_cut), gap, 0.0))

    def extents(self):
        out = []
        for kind, obj, gap, dy in self.items:
            if kind == "t":
                x0, y0, x1, y1 = self.cv.extent(obj)
                bx, by = obj.get_position()
                out.append((x0 - bx, y0 - by + dy, x1 - bx, y1 - by + dy))
            else:
                path, y_cut = obj
                v = path.vertices
                out.append((v[:, 0].min(), y_cut, v[:, 0].max(), v[:, 1].max()))
        return out

    def width(self):
        ex = self.extents()
        return sum(e[2] - e[0] for e in ex) + sum(it[2] for it in self.items)

    def lay(self, x_left, base):
        """把片段從 x_left 起、以 base 為基線依序排開；回傳括號的實際外框清單。"""
        x = x_left
        self.bboxes = []
        for (kind, obj, gap, dy), e in zip(self.items, self.extents()):
            x += gap
            if kind == "t":
                obj.set_position((x - e[0], base + dy))
            else:
                path, y_cut = obj
                ox = x - e[0]
                pp = PathPatch(path.transformed(Affine2D().translate(ox, base)),
                               facecolor="black", edgecolor="none", lw=0, zorder=6)
                self.cv.ax.add_patch(pp)
                clip = Rectangle((ox + e[0] - 0.05, base + y_cut), e[2] - e[0] + 0.10,
                                 (e[3] - y_cut) + 0.05, transform=self.cv.ax.transData)
                pp.set_clip_path(clip)
                self.bboxes.append((ox + e[0], base + y_cut, ox + e[2], base + e[3]))
            x += e[2] - e[0]
        return self.bboxes


def build_formula(cv, owner, group, row):
    f = Formula(cv, owner, group, row)
    f.text(TXT["N5b"], prop=None, size=FS)
    f.text("k", TNR_I, gap=0.10)
    f.text("=", TNR_R, gap=0.07)
    f.bracket("[", gap=0.07)
    f.text("2", TNR_R, gap=0.015)
    f.text("R", TNR_I, gap=0.0)
    f.text("eff", TNR_I, size=FS_SUB, gap=0.005, dy=-SUB_DROP)
    f.text("/", TNR_R, gap=0.05)       # 斜體 f 的右上懸出會碰到「/」（第一版目視）
    f.text("s", TNR_I, gap=0.0)
    f.bracket("]", gap=0.025)
    return f


# ════════════════════════════════════════════════════════════════════
#  圖
# ════════════════════════════════════════════════════════════════════
def build(fonts: fa.Fonts) -> tuple[fa.Canvas, dict]:
    # ── 1) 先用量測畫布算尺寸（文字 cm 大小與畫布高無關）──
    m = fa.Canvas(W, 5.0, fonts)
    sz = {k: rect_size(m, TXT[k]) for k in ("N1", "N2", "N3", "N4", "E1", "E2", "N6", "N7", "N8")}
    fm_ = build_formula(m, "N5", "tmp", 1)
    wa, ha = text_wh(m, TXT["N5a"], FS)
    one = text_wh(m, "情", FS)[1]
    pitch = text_wh(m, "情\n情", FS)[1] - one
    n5_text_w = max(wa, fm_.width())
    sz["N5"] = (n5_text_w + 2 * PAD_X, pitch + one + 0.05 + 2 * PAD_Y)
    plt.close(m.fig)

    h1 = max(sz[k][1] for k in ("N1", "N2", "N3", "N4"))
    h2 = max(sz[k][1] for k in ("N5", "N6", "N7", "N8"))
    m = fa.Canvas(W, 5.0, fonts)
    d1 = diamond_size(m, TXT["D1"], h1 * 0.92)
    # 兩行字的菱形：寬 = w/(fill − h/H)，H 太小寬度會暴增（H = 1.19 時 4.5 cm）⇒ 取較高的菱形
    d2 = diamond_size(m, TXT["D2"], D2_H, fill=0.86)
    th_lab = max(text_wh(m, s, FS_LAB)[1] for s in TXT["LEG"])
    plt.close(m.fig)
    # 左右兩端各自對齊成一欄（兩個端點框、兩個粗框同寬同中心）
    wl = max(sz["N1"][0], sz["N8"][0])
    wr = max(sz["N4"][0], sz["N5"][0])
    for k in ("N1", "N8"):
        sz[k] = (wl, sz[k][1])
    for k in ("N4", "N5"):
        sz[k] = (wr, sz[k][1])
    h_top = max(sz["E1"][1], sz["E2"][1])
    for k in ("E1", "E2"):
        sz[k] = (sz[k][0], h_top)

    # 垂直：由上往下。頂帶放出口框與兩列圖例，取兩者較高者
    band = max(h_top, 2 * th_lab + 0.14)
    note_h = th_lab + 0.06
    H = (MARGIN + band + GAP_UP + h1 + GAP_ROWS + max(h2, d2[1]) + GAP_NOTE
         + note_h + MARGIN)
    H = round(H + 1e-9, 2)
    cv = fa.Canvas(W, H, fonts)
    A = cv.at

    y_top = H - MARGIN - band / 2                      # 出口框／圖例中心
    y1 = H - MARGIN - band - GAP_UP - h1 / 2
    y2 = y1 - h1 / 2 - GAP_ROWS - max(h2, d2[1]) / 2
    y_note = MARGIN + note_h / 2

    # 水平：每列兩端固定，中間三個等間距
    xl = MARGIN + wl / 2
    xr = W - MARGIN - wr / 2

    def spread(widths):
        inner = (W - 2 * MARGIN) - sum(widths)
        gap = inner / (len(widths) - 1)
        xs, x = [], MARGIN
        for w in widths:
            xs.append(x + w / 2)
            x += w + gap
        return xs, gap

    r1 = [wl, sz["N2"][0], d1[0], sz["N3"][0], wr]
    xs1, gap1 = spread(r1)
    r2 = [wl, sz["N7"][0], sz["N6"][0], d2[0], wr]
    xs2, gap2 = spread(r2)

    # ── 列 1 ──
    cv.box("N1", xs1[0], y1, wl, h1, TXT["N1"], size=FS, **TERM)
    cv.box("N2", xs1[1], y1, sz["N2"][0], h1, TXT["N2"], size=FS)
    cv.diamond("D1", xs1[2], y1, d1[0], d1[1], TXT["D1"], fill="white", size=FS)
    cv.box("N3", xs1[3], y1, sz["N3"][0], h1, TXT["N3"], size=FS)
    cv.box("N4", xs1[4], y1, wr, h1, TXT["N4"], size=FS, **THICK)
    for a, b in (("N1", "N2"), ("N2", "D1"), ("D1", "N3"), ("N3", "N4")):
        cv.arrow([A(a, "right"), A(b, "left")], name=f"{a}-{b}")
    xa, xb = A("D1", "right")[0], A("N3", "left")[0]
    cv.text((xa + xb) / 2, y1 + 0.05, "是", size=FS_LAB, va="bottom")

    # 上方出口
    cv.box("E1", xs1[2], y_top, sz["E1"][0], h_top, TXT["E1"], size=FS, **EXIT)
    cv.box("E2", xs1[3], y_top, sz["E2"][0], h_top, TXT["E2"], size=FS, **EXIT)
    cv.arrow([A("D1", "top"), A("E1", "bottom")], name="D1-E1")
    cv.arrow([A("N3", "top"), A("E2", "bottom")], name="N3-E2")
    y_mid_up = (A("D1", "top")[1] + A("E1", "bottom")[1]) / 2
    cv.text(xs1[2] + 0.09, y_mid_up, "否", size=FS_LAB, ha="left")
    y_mid_up2 = (A("N3", "top")[1] + A("E2", "bottom")[1]) / 2
    cv.text(xs1[3] + 0.09, y_mid_up2, "無條帶", size=FS_LAB, ha="left")

    # ── 列 2（右→左）──
    # N5 的公式列要自己排版（⌈⌉ 為裁切字形），方塊不走 cv.box()
    p = FancyBboxPatch((xr - wr / 2, y2 - h2 / 2), wr, h2,
                       boxstyle="round,pad=0,rounding_size=0.06",
                       facecolor="white", edgecolor="black", lw=fa.LW_THICK, zorder=3)
    cv.ax.add_patch(p)
    cv.nodes["N5"] = {"shape": "rect", "cx": xr, "cy": y2, "w": wr, "h": h2}
    # N5 兩行：第一行一般文字、第二行公式；以一般多行文字的基線間距排
    grp = "N5rich"
    t_a = cv.ax.text(0, 0, TXT["N5a"], ha="left", va="baseline", zorder=6,
                     fontproperties=fonts.prop(FS), color="black")
    cv.texts.append({"artist": t_a, "kind": "node", "owner": "N5", "group": grp, "row": 0})
    f = build_formula(cv, "N5", grp, 1)
    ea = cv.extent(t_a)
    pa = t_a.get_position()
    ea = (ea[0] - pa[0], ea[1] - pa[1], ea[2] - pa[0], ea[3] - pa[1])
    desc_b = min(e[1] for e in f.extents())
    gap_lines = pitch + 0.05                 # 公式列比一般行多 0.05 cm（10 pt 拉丁字母較高）
    top_rel = ea[3]
    bot_rel = -gap_lines + desc_b
    base_a = y2 - (top_rel + bot_rel) / 2
    t_a.set_position((xr - (ea[2] - ea[0]) / 2 - ea[0], base_a))
    f.lay(xr - f.width() / 2, base_a - gap_lines)

    cv.diamond("D2", xs2[3], y2, d2[0], d2[1], TXT["D2"], fill="white", size=FS)
    cv.box("N6", xs2[2], y2, sz["N6"][0], h2, TXT["N6"], size=FS, **THICK)
    cv.box("N7", xs2[1], y2, sz["N7"][0], h2, TXT["N7"], size=FS)
    cv.box("N8", xs2[0], y2, wl, h2, TXT["N8"], size=FS, **TERM)

    x_down = xr + LOOP_OFF
    x_loop = xr - LOOP_OFF
    cv.arrow([(x_down, y1 - h1 / 2), (x_down, y2 + h2 / 2)], name="N4-N5")
    cv.arrow([A("N5", "left"), A("D2", "right")], name="N5-D2")
    cv.arrow([A("D2", "left"), A("N6", "right")], name="D2-N6")
    xa, xb = A("N6", "right")[0], A("D2", "left")[0]
    cv.text((xa + xb) / 2, y2 + 0.05, "是", size=FS_LAB, va="bottom")
    cv.arrow([A("N6", "left"), A("N7", "right")], name="N6-N7")
    cv.arrow([A("N7", "left"), A("N8", "right")], name="N7-N8")

    # 再分配回圈（虛線）：D2 頂 → 上 → 右 → 進入 N4 底部左側
    d2_top = A("D2", "top")[1]
    row1_bot = y1 - h1 / 2
    lab_h = text_wh(cv, "再分配回圈", FS_LAB)[1]
    y_loop = d2_top + 0.30
    assert y_loop + 0.06 + lab_h + 0.06 <= row1_bot + 1e-9, "列間距放不下回圈標籤"
    cv.arrow([A("D2", "top"), (xs2[3], y_loop), (x_loop, y_loop), (x_loop, row1_bot)],
             ls=fa.DASH, name="D2-N4")
    cv.text(xs2[3] - 0.09, y_loop, "否", size=FS_LAB, ha="right")
    cv.text((xs2[3] + x_loop) / 2, y_loop + 0.06, "再分配回圈", size=FS_LAB, va="bottom")

    # ── 底部說明 ──
    cv.text(W / 2, y_note, TXT["NOTE"], size=FS_LAB)

    # ── 圖例（左上；登記成方塊，文字歸它所有 ⇒ check() 會檢查字在框內、框不碰別的方塊）──
    leg = legend(cv, y_top, band)
    geo = {"H": H, "gap1": gap1, "gap2": gap2, "formula": f, "xs1": xs1, "xs2": xs2,
           "legend": leg}
    return cv, geo


def legend(cv: fa.Canvas, yc: float, hb: float) -> dict:
    """左上無框圖例（加框會像流程裡多一個方塊）。

    兩列：「粗框＝本文提出之模組」一列；「灰底點劃＝終止出口」「虛線＝再分配回圈」一列。
    文字登記為 label ⇒ check() 會查它不壓方塊、不被箭頭穿過；樣本外框另回傳給
    extra_checks() 查不碰出口框。
    """
    labs = TXT["LEG"]
    sw, sh, sg, col_gap = 0.50, 0.22, 0.10, 0.32     # 樣本寬高、樣本與字距、兩欄距
    tws = [text_wh(cv, s, FS_LAB)[0] for s in labs]
    th = max(text_wh(cv, s, FS_LAB)[1] for s in labs)
    ya = yc + hb / 2 - th / 2 - 0.02
    yb = yc - hb / 2 + th / 2 + 0.02
    sx = MARGIN
    # 粗框樣本（線寬與正文粗框相同）
    cv.ax.add_patch(FancyBboxPatch((sx, ya - sh / 2), sw, sh,
                                   boxstyle="round,pad=0,rounding_size=0.04",
                                   facecolor="white", edgecolor="black", lw=fa.LW_THICK, zorder=4))
    cv.text(sx + sw + sg, ya, labs[0], size=FS_LAB, ha="left")
    # 灰底點劃樣本
    cv.ax.add_patch(Rectangle((sx, yb - sh / 2), sw, sh, facecolor=GREY, edgecolor="black",
                              lw=fa.LW_THIN, ls=fa.DASHDOT, zorder=4))
    cv.text(sx + sw + sg, yb, labs[1], size=FS_LAB, ha="left")
    # 虛線箭頭樣本
    sx2 = sx + sw + sg + tws[1] + col_gap
    L, HW = 0.17, 0.068
    cv.ax.plot([sx2, sx2 + sw - L * 0.85], [yb, yb], color="black", lw=fa.LW_THIN, ls=fa.DASH,
               zorder=4, dash_capstyle="butt")
    cv.ax.add_patch(Polygon([(sx2 + sw, yb), (sx2 + sw - L, yb + HW), (sx2 + sw - L, yb - HW)],
                            closed=True, facecolor="black", edgecolor="black", lw=0.3, zorder=5))
    cv.text(sx2 + sw + sg, yb, labs[2], size=FS_LAB, ha="left")
    right = max(sx + sw + sg + tws[0], sx2 + sw + sg + tws[2])
    return {"bbox": (sx, yc - hb / 2, right, yc + hb / 2), "row_gap": (ya - yb) - th}


# ════════════════════════════════════════════════════════════════════
#  本圖額外檢查
# ════════════════════════════════════════════════════════════════════
def extra_checks(cv: fa.Canvas, geo: dict) -> list[str]:
    probs = []
    f = geo["formula"]
    n = cv.nodes["N5"]
    x0, y0 = n["cx"] - n["w"] / 2 + 0.12, n["cy"] - n["h"] / 2 + 0.12
    x1, y1 = n["cx"] + n["w"] / 2 - 0.12, n["cy"] + n["h"] / 2 - 0.12
    seg = [cv.extent(it[1]) for it in f.items if it[0] == "t"]
    for b in f.bboxes:
        if not (x0 <= b[0] and b[2] <= x1 and y0 <= b[1] and b[3] <= y1):
            probs.append(f"括號超出 N5 內緣：{tuple(round(v, 3) for v in b)}")
        for s in seg:
            if fa._rects_overlap(b, s, m=0.0):
                probs.append("括號壓到相鄰公式片段")
    for i, a in enumerate(seg):
        for b in seg[i + 1:]:
            # 下標 eff 與 R 的外框允許貼齊（R 的版面框含右側留白），不允許實際交疊超過 0.01 cm
            ov = min(a[2], b[2]) - max(a[0], b[0])
            if ov > 0.01 and min(a[3], b[3]) - max(a[1], b[1]) > 0:
                probs.append("公式片段互相重疊")
    if geo["gap1"] < 0.45 or geo["gap2"] < 0.45:
        probs.append(f"列內間距太小：{geo['gap1']:.2f}/{geo['gap2']:.2f} cm")
    lb = geo["legend"]["bbox"]
    if geo["legend"]["row_gap"] < 0.04:
        probs.append(f"圖例兩列間距 {geo['legend']['row_gap']:.3f} cm < 0.04")
    for nid, nd in cv.nodes.items():
        nb = (nd["cx"] - nd["w"] / 2, nd["cy"] - nd["h"] / 2,
              nd["cx"] + nd["w"] / 2, nd["cy"] + nd["h"] / 2)
        if fa._rects_overlap(lb, nb, m=0.25):
            probs.append(f"圖例離方塊 {nid} 不到 0.25 cm")
    return probs


def fonts_used(cv: fa.Canvas) -> set[str]:
    """實際用到的字型檔（matplotlib 逐字元後備鏈的解析結果）。"""
    used = set()
    for rec in cv.texts:
        t = rec["artist"]
        fp = t.get_fontproperties()
        for fname in fm.fontManager._find_fonts_by_props(fp) if hasattr(
                fm.fontManager, "_find_fonts_by_props") else [fm.findfont(fp)]:
            used.add(Path(fname).name.lower())
    return used


def save(cv: fa.Canvas) -> list[Path]:
    out = []
    for ext, kw in (("pdf", {}), ("svg", {}), ("png", {"dpi": PNG_DPI})):
        p = OUT_DIR / f"{STEM}.{ext}"
        meta = {"png": {"Software": None}, "svg": {"Date": None, "Creator": None},
                "pdf": {"Creator": None, "Producer": None, "CreationDate": None}}[ext]
        cv.fig.savefig(p, facecolor="white", metadata=meta, **kw)
        out.append(p)
    return out


def selftest(fonts) -> int:
    cases = []
    cv, geo = build(fonts)
    cases.append(("基準圖應無問題", fa.check(cv) == [] and extra_checks(cv, geo) == []))
    f = geo["formula"]
    b = f.bboxes[0]
    f.bboxes[0] = (b[0] - 5, b[1], b[2] - 5, b[3])
    cases.append(("括號超出方塊", any("括號超出" in p for p in extra_checks(cv, geo))))
    f.bboxes[0] = b
    seg = [it[1] for it in f.items if it[0] == "t"]
    pos = seg[2].get_position()
    seg[2].set_position((pos[0] - 0.15, pos[1]))
    cases.append(("公式片段重疊", any("重疊" in p for p in extra_checks(cv, geo))))
    seg[2].set_position(pos)
    t = next(r["artist"] for r in cv.texts if r["artist"].get_text() == "再分配回圈"
             and r["kind"] == "label")
    p0 = t.get_position()
    t.set_position((p0[0], p0[1] - 0.10))
    cases.append(("回圈標籤壓線", any("穿過文字" in p for p in fa.check(cv))))
    t.set_position(p0)
    cases.append(("還原後應無問題", fa.check(cv) == [] and extra_checks(cv, geo) == []))
    plt.close(cv.fig)
    bad = 0
    for name, ok in cases:
        print(f"  [{'OK ' if ok else 'BAD'}] {name}")
        bad += 0 if ok else 1
    return bad


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    fonts = fa.Fonts()
    for n in fonts.notes:
        print("WARN", n)
    if args.selftest:
        return 1 if selftest(fonts) else 0
    cv, geo = build(fonts)
    probs = fa.check(cv) + extra_checks(cv, geo)
    used = fonts_used(cv)
    if not used <= ALLOWED_FONT_FILES:
        probs.append(f"用到規定以外的字型：{sorted(used - ALLOWED_FONT_FILES)}")
    print(f"畫布 {W} x {geo['H']} cm；列內間距 {geo['gap1']:.2f} / {geo['gap2']:.2f} cm；"
          f"字型檔 {sorted(used)}")
    if probs:
        print(f"FAIL 版面檢查 {len(probs)} 項：")
        for p in probs:
            print("   -", p)
        return 1
    for p in save(cv):
        h = hashlib.sha256(p.read_bytes()).hexdigest()[:16]
        print(f"OK   {p.name}  sha256:{h}")
    plt.close(cv.fig)
    return 0


if __name__ == "__main__":
    plt.rcParams["svg.hashsalt"] = "aasrc2026"
    plt.rcParams["svg.fonttype"] = "path"
    plt.rcParams["pdf.fonttype"] = 42
    plt.rcParams["axes.unicode_minus"] = False
    sys.exit(main())
