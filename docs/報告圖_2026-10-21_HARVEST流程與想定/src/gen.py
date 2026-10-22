"""Generate report figures (SVG) for HARVEST flow + scenario.

mode 'css'  -> class attributes (page CSS resolves tokens; theme-aware)
mode 'lit'  -> literal light-theme attributes (for Word/PowerPoint and PNG export)
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).parent
DATA = json.loads((HERE / "data.json").read_text(encoding="utf-8"))

LIGHT = {
    "surface": "#FFFFFF", "ink": "#18222D", "ink2": "#52606D", "ink3": "#8794A0",
    "line": "#C3CCD4", "grid": "#E6EAEE", "accent": "#0E6378", "accent_soft": "#E1EFF3",
    "exit": "#A2412F", "exit_soft": "#F8E7E2", "area": "#EEF2F4",
    "a1": "#C2601A", "b2": "#11806A", "b3": "#3B5FC4", "warn": "#B4231B", "opt": "#7A6A2E",
}
FONT = "'Noto Sans TC','Microsoft JhengHei','PingFang TC',sans-serif"
MONO = "'IBM Plex Mono','Consolas',monospace"

# class -> {svg attr: token-or-literal}
STYLE = {
    "card":      {"fill": "surface", "stroke": "line", "stroke-width": "1.3"},
    "card-hl":   {"fill": "accent_soft", "stroke": "accent", "stroke-width": "2.2"},
    "card-opt":  {"fill": "surface", "stroke": "ink3", "stroke-width": "1.3", "stroke-dasharray": "6 4"},
    "card-exit": {"fill": "exit_soft", "stroke": "exit", "stroke-width": "1.3", "stroke-dasharray": "7 3 2 3"},
    "dia":       {"fill": "surface", "stroke": "ink2", "stroke-width": "1.4"},
    "badge":     {"fill": "ink"},
    "badge-hl":  {"fill": "accent"},
    "badge-t":   {"fill": "surface", "font-weight": "700", "font-size": "14"},
    "t-title":   {"fill": "ink", "font-weight": "700", "font-size": "16.5"},
    "t-h":       {"fill": "ink", "font-weight": "700", "font-size": "14.5"},
    "t":         {"fill": "ink", "font-size": "13.5"},
    "t-s":       {"fill": "ink2", "font-size": "12"},
    "t-xs":      {"fill": "ink2", "font-size": "11"},
    "t-tag":     {"fill": "ink2", "font-size": "11.5"},
    "t-acc":     {"fill": "accent", "font-weight": "700", "font-size": "13"},
    "t-exit":    {"fill": "exit", "font-weight": "700", "font-size": "13"},
    "t-lab":     {"fill": "ink2", "font-size": "12", "font-weight": "700"},
    "t-mono":    {"fill": "ink2", "font-size": "11.5", "font-family": "mono"},
    "rule":      {"stroke": "line", "stroke-width": "1"},
    "arrow":     {"stroke": "ink2", "stroke-width": "1.6", "fill": "none"},
    "arrow-acc": {"stroke": "accent", "stroke-width": "2", "fill": "none"},
    "arrow-loop": {"stroke": "ink2", "stroke-width": "1.5", "fill": "none", "stroke-dasharray": "6 4"},
    "arrow-exit": {"stroke": "exit", "stroke-width": "1.5", "fill": "none"},
    "mk":        {"fill": "ink2"},
    "mk-acc":    {"fill": "accent"},
    "mk-exit":   {"fill": "exit"},
    "bullet":    {"fill": "accent"},
    "bullet-o":  {"fill": "none", "stroke": "ink3", "stroke-width": "1.2"},
    "grid":      {"stroke": "grid", "stroke-width": "1"},
    "area":      {"fill": "area", "stroke": "ink", "stroke-width": "1.6"},
    "subz":      {"stroke": "ink2", "stroke-width": "1.2", "stroke-dasharray": "8 5", "fill": "none"},
    "ink-line":  {"stroke": "ink", "stroke-width": "1.4", "fill": "none"},
    "ink-fill":  {"fill": "ink"},
    "surf-fill": {"fill": "surface", "stroke": "ink", "stroke-width": "1.2"},
    "lim":       {"stroke": "warn", "stroke-width": "1.4", "stroke-dasharray": "7 4", "fill": "none"},
    "lim-t":     {"fill": "warn", "font-size": "11.5", "font-weight": "700"},
    "ceil":      {"stroke": "accent", "stroke-width": "1.3", "stroke-dasharray": "3 3", "fill": "none"},
    "minfw":     {"stroke": "ink3", "stroke-width": "1.2", "stroke-dasharray": "2 4", "fill": "none"},
    "axis":      {"stroke": "ink2", "stroke-width": "1.1", "fill": "none"},
    "plan-bar":  {"fill": "none", "stroke": "ink2", "stroke-width": "1.2", "stroke-dasharray": "4 3"},
    "wait-bar":  {"fill": "grid"},
    "halo":      {"paint-order": "stroke", "stroke": "surface", "stroke-width": "4", "stroke-linejoin": "round"},
    "t-in":      {"fill": "surface", "font-size": "11.5", "font-weight": "700"},
}
for u in ("a1", "b2", "b3"):
    STYLE[f"{u}-img"] = {"stroke": u, "stroke-width": "3.2", "fill": "none", "stroke-linecap": "round"}
    STYLE[f"{u}-path"] = {"stroke": u, "stroke-width": "1.5", "fill": "none", "stroke-linejoin": "round"}
    STYLE[f"{u}-trk"] = {"stroke": u, "stroke-width": "1.6", "fill": "none", "stroke-linejoin": "round"}
    STYLE[f"{u}-sw"] = {"fill": u, "fill-opacity": "0.10"}
    STYLE[f"{u}-fill"] = {"fill": u}
    STYLE[f"{u}-t"] = {"fill": u, "font-weight": "700", "font-size": "13"}


def css_rules() -> str:
    out = []
    for cls, attrs in STYLE.items():
        decl = []
        for k, v in attrs.items():
            if k == "font-family":
                decl.append(f"font-family:{MONO}")
                continue
            val = f"var(--{v.replace('_', '-')})" if v in LIGHT else v
            if k == "font-size":
                val = f"{v}px"
            decl.append(f"{k}:{val}")
        out.append(f".fig .{cls}{{{';'.join(decl)}}}")
    return "\n".join(out)


class Svg:
    def __init__(self, mode: str, w: int, h: int, label: str):
        self.mode, self.w, self.h, self.label = mode, w, h, label
        self.parts: list[str] = []

    def a(self, cls):
        if cls is None:
            return ""
        if self.mode == "css":
            return f' class="{cls}"'
        out = []
        for c in cls.split():
            for k, v in STYLE[c].items():
                if k == "font-family":
                    v = MONO.replace("'", "")
                elif v in LIGHT:
                    v = LIGHT[v]
                out.append(f'{k}="{v}"')
        return " " + " ".join(out)

    def add(self, s):
        self.parts.append(s)

    def rect(self, x, y, w, h, cls, rx=8, extra=""):
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}"{self.a(cls)}{extra}/>')

    def line(self, x1, y1, x2, y2, cls, extra=""):
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}"{self.a(cls)}{extra}/>')

    def poly(self, pts, cls, marker=None, extra=""):
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        m = f' marker-end="url(#{marker})"' if marker else ""
        self.add(f'<polyline points="{d}"{self.a(cls)}{m}{extra}/>')

    def polygon(self, pts, cls, extra=""):
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        self.add(f'<polygon points="{d}"{self.a(cls)}{extra}/>')

    def path(self, d, cls, marker=None, extra=""):
        m = f' marker-end="url(#{marker})"' if marker else ""
        self.add(f'<path d="{d}"{self.a(cls)}{m}{extra}/>')

    def circle(self, x, y, r, cls, extra=""):
        self.add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}"{self.a(cls)}{extra}/>')

    def text(self, x, y, s, cls, anchor="start", extra=""):
        s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        anc = "" if anchor == "start" else f' text-anchor="{anchor}"'
        self.add(f'<text x="{x:.1f}" y="{y:.1f}"{anc}{self.a(cls)}{extra}>{s}</text>')

    def markers(self):
        defs = []
        for mid, cls in (("ah", "mk"), ("ah-acc", "mk-acc"), ("ah-exit", "mk-exit"),
                         ("ah-a1", "a1-fill"), ("ah-b2", "b2-fill"), ("ah-b3", "b3-fill")):
            defs.append(f'<marker id="{self.label}-{mid}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
                        f'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z"{self.a(cls)}/></marker>')
        return "<defs>" + "".join(defs) + "</defs>"

    def mk(self, name):
        return f"{self.label}-{name}"

    def render(self, title: str) -> str:
        font = f' font-family="{FONT.replace(chr(39), "")}"' if self.mode == "lit" else ""
        bg = f'<rect width="{self.w}" height="{self.h}" fill="{LIGHT["surface"]}"/>' if self.mode == "lit" else ""
        cls = ' class="fig"' if self.mode == "css" else ""
        xmlns = ' xmlns="http://www.w3.org/2000/svg"'
        return (f'<svg{xmlns}{cls} viewBox="0 0 {self.w} {self.h}" width="{self.w}" height="{self.h}"{font} '
                f'role="img" aria-label="{title}"><title>{title}</title>{self.markers()}{bg}'
                + "".join(self.parts) + "</svg>")


# ───────────────────────── Figure 1: end-to-end flow ─────────────────────────
def fig_flow(mode):
    s = Svg(mode, 1280, 880, "f1")
    W, H = 392, 330
    xs = [24, 444, 864]
    ys = [20, 418]
    cards = [
        ("1", "任務設定", "AeroPlan VTOL 偵察頁", "card", [
            ("作業區多邊形（可多區）與起降場", None),
            ("機隊：Pioneer（A）×1、HERO（B）×2", None),
            ("相機足跡：視角、重疊率、目標 GSD → 航高", None),
            ("空域：AGL 上限 120 m、最低定翼作業高度", None),
            ("規劃風：手填／CWA 測站／Open-Meteo 網格", None),
            ("地形：DEM 檔或 Cesium World Terrain", None),
            ("禁航區、威脅區（Fence 管理）", None),
            ("選用：航測建圖（真相機、交叉航帶、GCP）", "opt"),
        ]),
        ("2", "協調前處理", "SwarmCoordinator＋轉接層", "card", [
            ("機隊切區：1 塊作業區切成 n 塊子區", None),
            ("混編機隊配單一大區會排不出工作", "sub"),
            ("禁航區切區：子區繞開禁航區", None),
            ("航高、高度帶、地形起伏預檢", None),
            ("空間不足時提示豁免或調低定翼高度", "sub"),
            ("轉成規劃輸入 MissionInputs", None),
            ("逐機起飛坪、降落坪（預設回自己的坪）、備降點", "sub"),
        ]),
        ("3", "HARVEST 規劃", "hv_dccpp.plan_mission", "card-hl", None),
        ("4", "飛前檢核", "AeroPlan", "card", [
            ("回收站自動指派試跑（有宣告備降點時）", None),
            ("工期或最小間距變差就不採用", "sub"),
            ("validate()：四維間隔、能量、AGL", None),
            ("上傳版（縮點後）航點沿線地形 AGL", None),
            ("覆蓋率格網驗算、2D／3D 檢視", None),
            ("網格風事後重估航時與能量", None),
            ("選用：建圖上傳閘門（風速、曝光）", "opt"),
        ]),
        ("5", "匯出・上傳・放行", "AeroPlan → 飛控", "card", [
            ("匯出 QuadPlane 任務與 .param", None),
            ("圍籬（貼合航線凸包）、集結點", "sub"),
            ("條帶上以距離觸發拍照（206）", None),
            ("MAVLink 上傳後讀回比對", None),
            ("參數逐項、任務項數；不符即停飛", "sub"),
            ("依規劃起飛時刻逐機放行", None),
            ("韌體不執行 NAV_DELAY，由地面站計時", "sub"),
        ]),
        ("6", "執行與飛後分析", "SITL／HIL／分析工具", "card", [
            ("ArduPlane QuadPlane SITL ×3（含模擬風）", None),
            ("HIL：Cube 飛控板上模擬", None),
            ("多機即時監控（2D／3D、飛控風估計）", None),
            ("下載每台 .bin 日誌", None),
            ("航跡回算覆蓋率（上界）、最小間距、最高 AGL", None),
            ("規劃與飛行對照（工期、航程）", None),
            ("選用：PPK → 照片定位 → ODX 拼接", "opt"),
        ]),
    ]
    pos = [(xs[0], ys[0]), (xs[1], ys[0]), (xs[2], ys[0]), (xs[0], ys[1]), (xs[1], ys[1]), (xs[2], ys[1])]
    for (num, title, tag, cls, lines), (x, y) in zip(cards, pos):
        s.rect(x, y, W, H, cls, rx=10)
        s.circle(x + 28, y + 30, 14, "badge-hl" if cls == "card-hl" else "badge")
        s.text(x + 28, y + 35, num, "badge-t", anchor="middle")
        s.text(x + 52, y + 36, title, "t-title")
        s.text(x + W - 16, y + 35, tag, "t-mono" if cls == "card-hl" else "t-tag", anchor="end")
        s.line(x + 16, y + 54, x + W - 16, y + 54, "rule")
        if lines is None:
            continue
        cy = y + 84
        for txt, kind in lines:
            if kind == "sub":
                s.text(x + 38, cy - 4, txt, "t-s")
                cy += 22
                continue
            if kind == "opt":
                s.rect(x + 20, cy - 10, 8, 8, "bullet-o", rx=1)
            else:
                s.rect(x + 20, cy - 10, 8, 8, "bullet", rx=1)
            s.text(x + 38, cy, txt, "t" if kind != "opt" else "t-s")
            cy += 30
    # HARVEST card content
    x, y = xs[2], ys[0]
    s.text(x + 20, y + 82, "輸入：子區、機隊、相機、空域、風、地形", "t")
    steps = ["① 情境建構與前置檢查", "② 區域分解與鋪條帶", "③ 高度分層", "④ 暫定起降走廊",
             "⑤ 風場時間成本與機數配額", "⑥ Beam-IDP 排序＋能量修復", "⑦ 走廊精算與 3D 組裝", "⑧ 四維去衝突"]
    for i, st in enumerate(steps):
        cx = x + 20 + (i // 4) * 186
        cy = y + 116 + (i % 4) * 30
        s.text(cx, cy, st, "t-acc" if i in (4, 5, 7) else "t")
    s.line(x + 16, y + 238, x + W - 16, y + 238, "rule")
    s.text(x + 20, y + 264, "輸出：每機四維航點、起飛時刻、能量餘裕", "t")
    s.text(x + 20, y + 292, "不可行回報 NO_GO；排不進的條帶回報未覆蓋", "t-s")
    s.text(x + 20, y + 314, "八個步驟的分支與回圈見圖 2", "t-s")
    # arrows
    ym = ys[0] + H / 2
    s.poly([(xs[0] + W + 2, ym), (xs[1] - 4, ym)], "arrow-acc", s.mk("ah-acc"))
    s.poly([(xs[1] + W + 2, ym), (xs[2] - 4, ym)], "arrow-acc", s.mk("ah-acc"))
    ym2 = ys[1] + H / 2
    s.poly([(xs[0] + W + 2, ym2), (xs[1] - 4, ym2)], "arrow-acc", s.mk("ah-acc"))
    s.poly([(xs[1] + W + 2, ym2), (xs[2] - 4, ym2)], "arrow-acc", s.mk("ah-acc"))
    yb = ys[0] + H
    s.poly([(xs[2] + W / 2, yb + 2), (xs[2] + W / 2, yb + 34), (xs[0] + W / 2, yb + 34), (xs[0] + W / 2, ys[1] - 4)],
           "arrow-acc", s.mk("ah-acc"))
    s.text(xs[1] + W / 2, yb + 28, "MissionPlan（每機四維航點＋起飛時刻）", "t-s", anchor="middle")
    # NO-GO feedback from 3 and 4 back to 1 (user edits inputs)
    # status strip
    y0 = 774
    s.text(24, y0 + 6, "驗證現況", "t-lab")
    chips = [
        ("規劃層", "封閉基準、突變測試", True),
        ("SITL 閉迴路", "論文 8 趟，主結果 HV4W55", True),
        ("HIL", "3 片 Cube 板，流程與桌面一致", True),
        ("實機", "尚未飛", False),
    ]
    cw, gap, x0 = 296, 18, 24
    for i, (h, d, ok) in enumerate(chips):
        cx = x0 + i * (cw + gap)
        cls = "card" if ok else "card-opt"
        s.rect(cx, y0 + 20, cw, 62, cls, rx=8)
        if ok:
            s.circle(cx + 22, y0 + 51, 9, "badge-hl")
            s.path(f"M{cx+17:.1f},{y0+51:.1f} l3.5,4 l7,-8", None,
                   extra=f' fill="none" stroke="{LIGHT["surface"] if mode=="lit" else "currentColor"}" stroke-width="2"'
                         + ('' if mode == 'lit' else ' class="tick"'))
        else:
            s.circle(cx + 22, y0 + 51, 9, "bullet-o")
        s.text(cx + 40, y0 + 46, h, "t-h")
        s.text(cx + 40, y0 + 68, d, "t-s")
        if i < 3:
            s.poly([(cx + cw + 2, y0 + 51), (cx + cw + gap - 3, y0 + 51)], "arrow", s.mk("ah"))
    return s.render("HARVEST 整體流程")


# ───────────────────────── Figure 2: HARVEST internals ─────────────────────────
def fig_harvest(mode):
    s = Svg(mode, 1280, 760, "f2")
    BH = 84

    def box(x, y, w, title, lines, cls="card", h=BH):
        s.rect(x, y, w, h, cls, rx=8)
        s.text(x + w / 2, y + 27, title, "t-h", anchor="middle")
        for i, ln in enumerate(lines):
            s.text(x + w / 2, y + 50 + i * 19, ln, "t-s", anchor="middle")

    def dia(cx, cy, hw, hh, lines):
        s.polygon([(cx, cy - hh), (cx + hw, cy), (cx, cy + hh), (cx - hw, cy)], "dia")
        for i, ln in enumerate(lines):
            s.text(cx, cy + 5 - (len(lines) - 1) * 9 + i * 18, ln, "t-h", anchor="middle")

    def arr(pts, cls="arrow", mk="ah"):
        s.poly(pts, cls, s.mk(mk))

    r1, r2, r3 = 104, 312, 540          # row tops
    # row 1 (L→R)
    box(24, r1, 150, "規劃輸入", ["MissionInputs", "（轉接層產生）"], "card")
    box(210, r1, 214, "① 情境建構與前置檢查", ["build_context・preflight", "平台、能量、風、高度帶"], "card")
    dia(498, r1 + BH / 2, 58, 40, ["可規劃？"])
    box(592, r1, 234, "② 區域分解與鋪條帶", ["凸胞分解；刈幅 w、間距 s", "殘帶候選（A／B 分帶）"], "card")
    dia(884, r1 + BH / 2, 52, 40, ["有條帶？"])
    box(972, r1, 284, "③ 高度分層", ["兩構型分高度帶；不足改時間分離", "建圖模式一律同高（保 GSD）"], "card")
    c1 = r1 + BH / 2
    arr([(176, c1), (206, c1)])
    arr([(426, c1), (438, c1)])
    arr([(558, c1), (588, c1)]); s.text(573, c1 - 8, "是", "t-s", anchor="middle")
    arr([(828, c1), (830, c1)])
    arr([(938, c1), (968, c1)]); s.text(953, c1 - 8, "是", "t-s", anchor="middle")
    # exits row 1
    box(388, 14, 220, "回報 NO_GO", ["空計畫＋逐條原因"], "card-exit", h=58)
    box(780, 14, 208, "回報未覆蓋", ["逐條帶列出原因"], "card-exit", h=58)
    arr([(498, c1 - 42), (498, 76)], "arrow-exit", "ah-exit"); s.text(508, 92, "否", "t-s")
    arr([(884, c1 - 42), (884, 76)], "arrow-exit", "ah-exit"); s.text(894, 92, "否", "t-s")
    # row1 → row2
    arr([(1146, r1 + BH + 2), (1146, r2 - 4)])
    # row 2 (R→L)
    box(1036, r2, 220, "④ 暫定起降走廊", ["VTOL 爬升 → 轉換 → 進場", "降落走廊（各回收坪）"], "card")
    box(690, r2, 310, "⑤ 時間成本與機數配額", ["考慮風場地速、R_eff Dubins 成本表", "決策級配額 λ_floor、起飛時程"], "card-hl")
    box(412, r2, 244, "⑥ Beam-IDP 條帶排序", ["跳帶 k = ⌈2R_eff / s⌉", "每機接上返場段（RTB）"], "card-hl")
    dia(316, r2 + BH / 2, 62, 42, ["返場能量", "可行？"])
    box(24, r2, 196, "⑥b 交叉航帶", ["建圖模式才執行"], "card-opt")
    c2 = r2 + BH / 2
    arr([(1034, c2), (1004, c2)])
    arr([(688, c2), (660, c2)])
    arr([(410, c2), (380, c2)])
    arr([(252, c2), (224, c2)]); s.text(238, c2 - 8, "是", "t-s", anchor="middle")
    # energy loop
    s.rect(262, r2 + BH + 36, 400, 52, "card", rx=8)
    s.text(462, r2 + BH + 58, "能量修復（≤3 輪）、剩餘條帶再拍賣", "t-h", anchor="middle")
    s.text(462, r2 + BH + 78, "同構型只允許尾端追加；仍不可行 → 計入未覆蓋", "t-s", anchor="middle")
    arr([(316, c2 + 44), (316, r2 + BH + 32)], "arrow-loop"); s.text(326, r2 + BH + 22, "否", "t-s")
    arr([(600, r2 + BH + 34), (600, r2 + BH + 4)], "arrow-loop")
    # row2 → row3
    arr([(122, r2 + BH + 2), (122, r3 - 4)])
    # row 3 (L→R)
    box(24, r3, 250, "⑦ 走廊精算與組裝", ["以首條帶入口重算起飛走廊", "2D 路徑 → 3D 高度剖面"], "card")
    box(310, r3, 300, "⑧ 四維去衝突", ["場坪預約時窗互斥（以場坪身分）", "空間互斥免檢＋四維間隔檢查"], "card-hl")
    dia(702, r3 + BH / 2, 56, 40, ["有衝突？"])
    box(796, r3, 460, "輸出 MissionPlan", ["每機四維航點、起飛時刻、makespan、能量餘裕",
                                          "連同未覆蓋清單與警告 → 飛前檢核（圖 1 階段 4）"], "card")
    c3 = r3 + BH / 2
    arr([(276, c3), (306, c3)])
    arr([(612, c3), (642, c3)])
    arr([(760, c3), (792, c3)]); s.text(776, c3 - 8, "否", "t-s", anchor="middle")
    # repair loop
    rx0, ry0 = 420, r3 + BH + 34
    s.rect(rx0, ry0, 620, 70, "card", rx=8)
    s.text(rx0 + 310, ry0 + 24, "修復序（逐輪、乾跑驗證後取工期最小者）", "t-h", anchor="middle")
    s.text(rx0 + 310, ry0 + 46, "起飛延遲／換序 → 延遲盤旋 → IDP 重排 → 工期壓縮 → 全域重排", "t", anchor="middle")
    s.text(rx0 + 310, ry0 + 64, "仍解不掉者列為殘餘衝突並回報（航點高度永不更動）", "t-s", anchor="middle")
    arr([(702, c3 + 42), (702, ry0 - 4)], "arrow-loop"); s.text(712, ry0 - 12, "是", "t-s")
    arr([(500, ry0 - 2), (500, r3 + BH + 4)], "arrow-loop")
    # legend
    lx, ly = 24, 668
    s.rect(lx, ly, 30, 18, "card", rx=4); s.text(lx + 38, ly + 14, "步驟", "t-s")
    s.rect(lx + 90, ly, 30, 18, "card-hl", rx=4); s.text(lx + 128, ly + 14, "HARVEST 主要機制", "t-s")
    s.rect(lx, ly + 30, 30, 18, "card-opt", rx=4); s.text(lx + 38, ly + 44, "選用步驟", "t-s")
    s.rect(lx + 90, ly + 30, 30, 18, "card-exit", rx=4); s.text(lx + 128, ly + 44, "終止出口", "t-s")
    s.poly([(lx, ly + 72), (lx + 30, ly + 72)], "arrow-loop", s.mk("ah")); s.text(lx + 38, ly + 76, "回圈", "t-s")
    s.text(lx + 90, ly + 76, "列 1、3 左→右；列 2 右→左", "t-s")
    return s.render("HARVEST 規劃器八步驟")


# ───────────────────────── Figure 3: scenario map ─────────────────────────
def fig_scenario(mode):
    W, Hh = 1280, 700
    s = Svg(mode, W, Hh, "f3")
    SC = 0.47
    X0, X1, Y0, Y1 = -1330.0, 1250.0, -500.0, 470.0
    ox, oy = 34.0, 22.0

    def P(x, y):
        return (ox + (x - X0) * SC, oy + (Y1 - y) * SC)

    # map frame
    mx0, my0 = P(X0, Y1)
    mx1, my1 = P(X1, Y0)
    s.rect(mx0, my0, mx1 - mx0, my1 - my0, "card", rx=6)
    # grid every 200 m
    for gx in range(-1200, 1250, 200):
        a, b = P(gx, Y1), P(gx, Y0)
        s.line(a[0], a[1] + 1, b[0], b[1] - 1, "grid")
    for gy in range(-400, 470, 200):
        a, b = P(X0, gy), P(X1, gy)
        s.line(a[0] + 1, a[1], b[0] - 1, b[1], "grid")
    # area
    area = [P(x, y) for x, y in DATA["area"]]
    s.polygon(area, "area")
    # clip for swaths
    ax0, ay0 = P(-780, 400)
    ax1, ay1 = P(780, -400)
    clip = f"{s.label}-clip"
    s.add(f'<clipPath id="{clip}"><rect x="{ax0:.1f}" y="{ay0:.1f}" width="{ax1-ax0:.1f}" height="{ay1-ay0:.1f}"/></clipPath>')
    sw = 137.99 / 2
    s.add(f'<g clip-path="url(#{clip})">')
    for u in ("A1", "B2", "B3"):
        for seg in DATA["uavs"][u]["imaging"]:
            y = seg[0][1]
            a, b = P(-900, y + sw), P(900, y - sw)
            s.rect(a[0], a[1], b[0] - a[0], b[1] - a[1], f"{u.lower()}-sw", rx=0)
    s.add("</g>")
    # subzone lines
    for y in DATA["subzones_y"][1:-1]:
        a, b = P(-780, y), P(780, y)
        s.line(a[0], a[1], b[0], b[1], "subz")
    s.polygon(area, "ink-line")
    # paths
    for u in ("A1", "B2", "B3"):
        d = DATA["uavs"][u]
        pts = [P(p[0], p[1]) for p in d["path"]]
        s.poly(pts, f"{u.lower()}-path")
    for u in ("A1", "B2", "B3"):
        for seg in DATA["uavs"][u]["imaging"]:
            a, b = P(seg[0][0], seg[0][1]), P(seg[-1][0], seg[-1][1])
            s.poly([a, b], f"{u.lower()}-img")
            # direction arrow at 30 % along
            t = 0.5
            mx, my = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            dx = 1 if b[0] > a[0] else -1
            s.poly([(mx - 10 * dx, my), (mx + 8 * dx, my)], f"{u.lower()}-img", s.mk(f"ah-{u.lower()}"))
    # subzone labels (left inside area, between strips)
    for name, ym in (("Z4_1（北）", 313.0), ("Z3_1", 46.5), ("Z2_1（南）", -220.5)):
        px, py = P(-770, ym)
        s.text(px, py + 4, "子區 " + name, "t-lab halo")
    # dims
    a, b = P(-780, -400), P(780, -400)
    s.text((a[0] + b[0]) / 2, a[1] + 17, "1560 m × 800 m（WGS84 1.2434 km²）", "t-lab", anchor="middle")
    a2 = P(780, 0)
    # pads: H marker + callout
    hx, hy = P(0, 0)
    s.circle(hx, hy, 6.5, "surf-fill")
    s.circle(hx, hy, 2.2, "ink-fill")
    s.poly([(hx + 6, hy + 5), (hx + 26, hy + 26), (hx + 70, hy + 26)], "ink-line")
    s.text(hx + 74, hy + 30, "起降場（3 坪，見右下放大）", "t-s halo")
    # UAV labels at first imaging strip start
    for u, txt, x, y in (("A1", "A1 Pioneer：每子區 1 條 A 帶", -430, 400 + 6 / SC), ("B3", "B3 HERO：Z4_1、Z3_1 各 2 條", -430, 262),
                         ("B2", "B2 HERO：Z2_1 2 條", 300, -368)):
        px, py = P(x, y)
        s.text(px, py - 8, txt, f"{u.lower()}-t halo")
    # wind arrow (top-left)
    wx, wy = P(-1300, 430)
    s.poly([(wx, wy + 10), (wx + 74, wy + 10)], "arrow-acc", s.mk("ah-acc"))
    s.text(wx, wy + 30, "風 5 m/s，來自 270°", "t-acc")
    # north arrow + scale bar (bottom-left)
    nx, ny = P(1200, 440)
    s.polygon([(nx, ny), (nx - 7, ny + 20), (nx, ny + 15), (nx + 7, ny + 20)], "ink-fill")
    s.text(nx, ny + 34, "N", "t-lab", anchor="middle")
    bx, by = P(-1300, -470)
    s.rect(bx, by - 6, 200 * SC, 6, "ink-fill", rx=0)
    s.rect(bx + 200 * SC, by - 6, 300 * SC, 6, "surf-fill", rx=0)
    for v, lbl in ((0, "0"), (200, "200"), (500, "500 m")):
        s.text(bx + v * SC, by + 10, lbl, "t-xs", anchor="middle" if v else "start")
    s.text(P(1240, -470)[0], P(1240, -470)[1], "格線 200 m｜平面近似 ENU，原點＝起降場 H", "t-xs", anchor="end")

    # ── legend row ──
    ly = my1 + 22
    # legend items
    s.text(34, ly + 14, "圖例", "t-lab")
    items = [
        ("img", "拍照條帶（206 觸發段）與飛行方向"),
        ("path", "過境、迴轉、進場與返場"),
        ("sw", "相機刈幅 138.0 m（119.5 m、HFOV 60°）"),
        ("subz", "機隊切區界線（3 塊子區）"),
        ("area", "作業區 1560 × 800 m"),
    ]
    for i, (k, t) in enumerate(items):
        yy = ly + 38 + i * 26
        if k == "img":
            s.poly([(34, yy - 4), (70, yy - 4)], "a1-img")
        elif k == "path":
            s.poly([(34, yy - 4), (70, yy - 4)], "b3-path")
        elif k == "sw":
            s.rect(34, yy - 12, 36, 16, "b2-sw", rx=0)
        elif k == "subz":
            s.line(34, yy - 4, 70, yy - 4, "subz")
        else:
            s.rect(34, yy - 12, 36, 16, "area", rx=0)
        s.text(80, yy, t, "t-s")
    # fleet table
    tx = 400
    cols = [(0, "機號"), (52, "機型（構型）"), (158, "空速"), (226, "轉彎半徑"), (296, "條帶（子區）"), (430, "航程"), (500, "起飛 → 落地")]
    s.text(tx, ly + 14, "機隊與分工（規劃值）", "t-lab")
    for cx, h in cols:
        s.text(tx + cx, ly + 40, h, "t-xs")
    s.line(tx, ly + 47, tx + 620, ly + 47, "rule")
    rows = [
        ("A1", "Pioneer（A）", "21.5 m/s", "200 m", "3（三子區各 1）", "10.29 km", "656 → 1260.5 s"),
        ("B2", "HERO（B）", "19.0 m/s", "120 m", "2（Z2_1）", "7.93 km", "0 → 533.9 s"),
        ("B3", "HERO（B）", "19.0 m/s", "120 m", "4（Z3_1、Z4_1）", "12.44 km", "106 → 894.5 s"),
    ]
    for i, r in enumerate(rows):
        yy = ly + 70 + i * 27
        s.rect(tx, yy - 11, 10, 10, f"{r[0].lower()}-fill", rx=2)
        for j, (cx, _h) in enumerate(cols):
            s.text(tx + cx + (16 if j == 0 else 0), yy, r[j], f"{r[0].lower()}-t" if j == 0 else "t-s")
    s.text(tx, ly + 160, "A 帶與 B 帶幾何相同（兩型共用單光相機）；B 帶間距 97 m＜水平門檻 100 m", "t-xs")
    # pad inset
    ix, iy, iw, ih = 1056, ly + 4, 190, 150
    s.rect(ix, iy, iw, ih, "card", rx=6)
    s.text(ix + 10, iy + 18, "起降場放大（坪距 10 m）", "t-lab")
    k = 5.0   # px per m
    cxp, cyp = ix + iw / 2, iy + 78
    for px_m in (-20, -10, 0, 10, 20):
        s.line(cxp + px_m * k, iy + 30, cxp + px_m * k, iy + 120, "grid")
    pads = [("TO_A1", -10, "a1"), ("H＝TO_B2", 0, "b2"), ("TO_B3", 10, "b3")]
    for name, xm, u in pads:
        s.circle(cxp + xm * k, cyp, 9, f"{u}-fill")
        s.circle(cxp + xm * k, cyp, 3, "surf-fill")
    s.text(cxp - 10 * k, cyp - 16, "A1", "a1-t", anchor="middle")
    s.text(cxp, cyp - 16, "B2", "b2-t", anchor="middle")
    s.text(cxp + 10 * k, cyp - 16, "B3", "b3-t", anchor="middle")
    s.text(cxp, cyp + 28, "TO_A1｜H（＝TO_B2）｜TO_B3", "t-xs", anchor="middle")
    s.text(cxp, cyp + 46, "本架次三機皆降回 H", "t-xs", anchor="middle")
    s.text(cxp, cyp + 62, "10-09 起預設降回各自起飛坪", "t-xs", anchor="middle")
    return s.render("HARVEST 想定圖（HV4W55）")


# ───────────────────────── Figure 4: timeline + altitude (SITL) ─────────────────────────
def fig_timeline(mode):
    W, Hh = 1280, 640
    s = Svg(mode, W, Hh, "f4")
    X0p, X1p = 96, 1250
    T1 = 1300.0

    def tx(t):
        return X0p + t / T1 * (X1p - X0p)

    plan = {"A1": (656.0, 1260.5), "B2": (0.0, 533.9), "B3": (106.0, 894.5)}
    act = {}
    for u in ("A1", "B2", "B3"):
        trk = DATA["uavs"][u]["track"]
        up = [p[0] for p in trk if p[3] > 2.0]
        act[u] = (up[0], up[-1])
    # ── Gantt ──
    s.text(24, 22, "起飛時程（規劃 vs. SITL）", "t-lab")
    for i, u in enumerate(("A1", "B2", "B3")):
        y = 40 + i * 34
        s.text(24, y + 16, u, f"{u.lower()}-t")
        s.rect(tx(0), y + 2, tx(T1) - tx(0), 20, "wait-bar", rx=3)
        p0, p1 = plan[u]
        a0, a1 = act[u]
        s.rect(tx(a0), y + 4, tx(a1) - tx(a0), 16, f"{u.lower()}-fill", rx=3)
        s.rect(tx(p0), y + 1, tx(p1) - tx(p0), 22, "plan-bar", rx=3)
        s.text(tx(a1) - 8, y + 16, f"SITL {a0:.0f} → {a1:.0f} s", "t-in", anchor="end")
    s.text(tx(330), 40 + 0 * 34 + 17, "A1 地面等待 656 s（規劃）", "t-s", anchor="middle")
    yl = 40 + 3 * 34 + 2
    s.rect(tx(0), yl, 26, 12, "a1-fill", rx=2); s.text(tx(0) + 32, yl + 11, "SITL 在空（相對高度 > 2 m）", "t-xs")
    s.rect(tx(240), yl - 1, 26, 14, "plan-bar", rx=2); s.text(tx(240) + 32, yl + 11, "規劃起飛 → 落地", "t-xs")
    s.text(tx(470), yl + 11, "SITL makespan 1281.9 s（規劃 1260.5 s，+1.70%）", "t-xs")

    # ── altitude chart ──
    cy0, cy1 = 186, 446
    A1m = 130.0

    def ay(h):
        return cy1 - h / A1m * (cy1 - cy0)

    s.text(24, cy0 - 14, "相對高度（SITL 飛控紀錄，約 1 Hz）", "t-lab")
    for h in (0, 20, 40, 60, 80, 100, 120):
        s.line(X0p, ay(h), X1p, ay(h), "grid")
        s.text(X0p - 8, ay(h) + 4, f"{h}", "t-xs", anchor="end")
    s.text(X0p - 8, cy0 - 2, "m", "t-xs", anchor="end")
    s.line(X0p, ay(120), X1p, ay(120), "lim")
    s.text(X1p - 4, ay(120) - 6, "作業上限 120 m（法規 400 ft＝121.92 m）", "lim-t", anchor="end")
    s.line(X0p, ay(100), X1p, ay(100), "minfw")
    s.text(tx(568), ay(100) + 15, "最低定翼作業高度 100 m", "t-xs")
    for u in ("B2", "B3", "A1"):
        pts = [(tx(p[0]), ay(max(p[3], 0))) for p in DATA["uavs"][u]["track"] if 0 <= p[0] <= T1]
        s.poly(pts, f"{u.lower()}-trk")
    # phase annotation on B2
    s.text(tx(8), ay(56), "VTOL 垂直起飛", "t-xs")
    s.text(tx(60), ay(126), "轉換後爬升 → 掃描 119.5 m", "t-xs")
    s.text(tx(536), ay(48), "返場下降 → VTOL 垂直降落", "t-xs")
    s.axis = None
    s.line(X0p, cy1, X1p, cy1, "axis")
    for t in range(0, 1301, 100):
        s.line(tx(t), cy1, tx(t), cy1 + 5, "axis")
        s.text(tx(t), cy1 + 19, f"{t}", "t-xs", anchor="middle")
    s.text(X1p, cy1 + 36, "任務時間 t（s，自第一架放行起算）", "t-xs", anchor="end")

    # ── zoom band 117–121 m ──
    zy0, zy1 = 512, 616
    Z0, Z1 = 117.0, 121.0

    def zy(h):
        return zy1 - (h - Z0) / (Z1 - Z0) * (zy1 - zy0)

    s.text(24, zy0 - 12, "巡航段放大（117–121 m）", "t-lab")
    s.rect(X0p, zy0, X1p - X0p, zy1 - zy0, "card", rx=2)
    for h in (118, 119, 120):
        s.line(X0p, zy(h), X1p, zy(h), "grid")
        s.text(X0p - 8, zy(h) + 4, f"{h}", "t-xs", anchor="end")
    s.line(X0p, zy(120), X1p, zy(120), "lim")
    s.line(X0p, zy(119.5), X1p, zy(119.5), "ceil")
    s.text(X0p + 6, zy(119.5) + 14, "規劃天花板 119.5 m", "t-acc")
    for u in ("B2", "B3", "A1"):
        seg = []
        for p in DATA["uavs"][u]["track"]:
            if 0 <= p[0] <= T1 and Z0 <= p[3] <= Z1:
                seg.append((tx(p[0]), zy(p[3])))
            else:
                if len(seg) > 1:
                    s.poly(seg, f"{u.lower()}-trk")
                seg = []
        if len(seg) > 1:
            s.poly(seg, f"{u.lower()}-trk")
    s.text(X1p - 6, zy(120) - 6, "高於 120 m 時間 0 s；最高 119.68 m（B3）", "lim-t", anchor="end")
    return s.render("HARVEST 想定時程與高度（HV4W55 SITL）")


if __name__ == "__main__":
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    figs = {"fig1_flow": fig_flow, "fig2_harvest": fig_harvest, "fig3_scenario": fig_scenario,
            "fig4_timeline": fig_timeline}
    for name, fn in figs.items():
        (out / f"{name}.css.svg").write_text(fn("css"), encoding="utf-8")
        (out / f"{name}.svg").write_text(fn("lit"), encoding="utf-8")
    (out / "fig.css").write_text(css_rules(), encoding="utf-8")
    # B2 phase timing probe
    trk = DATA["uavs"]["B2"]["track"]
    for th in (45, 99, 119):
        tt = [p[0] for p in trk if p[3] >= th]
        print("B2 reach", th, tt[0] if tt else None, "last", tt[-1] if tt else None)
    print("done")
