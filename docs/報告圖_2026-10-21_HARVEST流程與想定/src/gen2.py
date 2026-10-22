"""16:9 slide figures from TODAY's HARVEST plan (today.json, dumped by dump_today.py).

slide1_birdseye : oblique 3-D scenario view + 8-phase VTOL storyboard
slide2_timeline : phase-coded Gantt + planned altitude–time
slide3_plan     : to-scale plan view + locator for ALT1
"""
import json
import math
import sys
from pathlib import Path

from gen import LIGHT, STYLE, Svg, css_rules

HERE = Path(__file__).parent
T = json.loads((HERE / "today.json").read_text(encoding="utf-8"))
# 高度一律用 AGL（z_m 是局部 ENU，含地球曲率，條帶兩端會低 0.1 m）
for _d in T["uavs"].values():
    for _w in _d["wp"]:
        _w[2] = _w[3]
UAVS = ("A1", "B2", "B3")
NAME = {"A1": "A1 Pioneer", "B2": "B2 HERO", "B3": "B3 HERO"}
W, H = 1920, 1080
SW_HALF = 137.99 / 2

STYLE.update({
    "s-title": {"fill": "ink", "font-weight": "700", "font-size": "38"},
    "s-sub":   {"fill": "ink2", "font-size": "21"},
    "s-tag":   {"fill": "accent", "font-weight": "700", "font-size": "19"},
    "s-h":     {"fill": "ink", "font-weight": "700", "font-size": "23"},
    "s-t":     {"fill": "ink", "font-size": "19"},
    "s-s":     {"fill": "ink2", "font-size": "17"},
    "s-xs":    {"fill": "ink2", "font-size": "15"},
    "s-lab":   {"fill": "ink2", "font-size": "18", "font-weight": "700"},
    "s-bt":    {"fill": "surface", "font-weight": "700", "font-size": "19"},
    "s-a1":    {"fill": "a1", "font-weight": "700", "font-size": "20"},
    "s-b2":    {"fill": "b2", "font-weight": "700", "font-size": "20"},
    "s-b3":    {"fill": "b3", "font-weight": "700", "font-size": "20"},
    "s-lim":   {"fill": "warn", "font-size": "17", "font-weight": "700"},
    "s-acc":   {"fill": "accent", "font-size": "18", "font-weight": "700"},
    "ground":  {"fill": "area", "fill-opacity": "0.55", "stroke": "line", "stroke-width": "1"},
    "trk":     {"stroke": "ink3", "stroke-width": "1.1", "fill": "none", "stroke-opacity": "0.55"},
    "drop":    {"stroke": "ink3", "stroke-width": "1", "stroke-dasharray": "2 4", "fill": "none"},
    "icon":    {"fill": "ink"},
    "icon-l":  {"stroke": "ink", "stroke-width": "2", "fill": "none", "stroke-linecap": "round"},
    "rotor":   {"fill": "ink", "fill-opacity": "0.28"},
    "hover-l": {"stroke": "ink", "stroke-width": "3.2", "fill": "none", "stroke-dasharray": "1 5", "stroke-linecap": "round"},
    "sk":      {"stroke": "ink2", "stroke-width": "2.4", "fill": "none", "stroke-linecap": "round", "stroke-linejoin": "round"},
    "sk-d":    {"stroke": "ink2", "stroke-width": "2.4", "fill": "none", "stroke-dasharray": "7 5"},
    "sk-g":    {"stroke": "ink3", "stroke-width": "1.6", "fill": "none"},
    "sk-lvl":  {"stroke": "accent", "stroke-width": "1.3", "fill": "none", "stroke-dasharray": "3 3"},
    "cone":    {"fill": "a1", "fill-opacity": "0.16", "stroke": "a1", "stroke-width": "1.2"},
    "cone-k":  {"fill": "accent", "fill-opacity": "0.14", "stroke": "accent", "stroke-width": "1.2"},
    "pad":     {"fill": "surface", "stroke": "ink", "stroke-width": "2"},
    "panel":   {"fill": "surface", "stroke": "line", "stroke-width": "1.3"},
    "gwait":   {"fill": "grid"},
    "hover-f": {"fill": "ink"},
    "mspan":   {"stroke": "ink", "stroke-width": "1.6", "stroke-dasharray": "6 4", "fill": "none"},
})
for u in ("a1", "b2", "b3"):
    STYLE[f"{u}-op"] = {"stroke": u, "stroke-width": "5", "fill": "none", "stroke-linecap": "round"}
    STYLE[f"{u}-fw"] = {"stroke": u, "stroke-width": "2.2", "fill": "none", "stroke-linejoin": "round"}
    STYLE[f"{u}-tr"] = {"stroke": u, "stroke-width": "2.6", "fill": "none", "stroke-dasharray": "9 5"}
    STYLE[f"{u}-vt"] = {"stroke": u, "stroke-width": "3", "fill": "none", "stroke-dasharray": "2 4", "stroke-linecap": "round"}
    STYLE[f"{u}-tint"] = {"fill": u, "fill-opacity": "0.30"}
    STYLE[f"{u}-ln"] = {"stroke": u, "stroke-width": "2.6", "fill": "none", "stroke-linejoin": "round"}
    STYLE[f"hl-{u}"] = {"stroke": u, "stroke-width": "3.2"}
    STYLE[f"s-v{u}"] = {"fill": u, "font-weight": "700", "font-size": "17"}
STYLE["s-v"] = {"fill": "ink", "font-size": "17"}

PH_KIND = {"TAKEOFF": "vt", "LAND": "vt", "TRANS_OUT": "tr", "TRANS_IN": "tr", "OPERATE": "op"}


def kind(ph):
    return PH_KIND.get(ph, "fw")


def runs(wp):
    """split waypoint list into consecutive runs of the same drawing kind (by segment start phase)."""
    out = []
    for i in range(1, len(wp)):
        k = kind(wp[i - 1][5])
        if wp[i - 1][7] and wp[i][7]:
            k = "op"
        elif k == "op":
            k = "fw"
        if out and out[-1][0] == k:
            out[-1][1].append(wp[i])
        else:
            out.append([k, [wp[i - 1], wp[i]]])
    return out


def phase_stats(u):
    """per storyboard phase: (duration s, z0, z1, count)."""
    wp = T["uavs"][u]["wp"]
    groups = {"1": ["TAKEOFF"], "2": ["TRANS_OUT"], "3": ["ACCEL", "CLIMB", "ENTRY"], "4": ["OPERATE"],
              "5": ["TRANSFER"], "6": ["RTB", "DESCENT"], "7": ["DECEL", "TRANS_IN"], "8": ["LAND"]}
    segs = []
    for i in range(1, len(wp)):
        p = wp[i - 1][5]
        if segs and segs[-1][0] == p:
            segs[-1][2] = wp[i][4]
            segs[-1][4] = wp[i][2]
        else:
            segs.append([p, wp[i - 1][4], wp[i][4], wp[i - 1][2], wp[i][2]])
    res = {}
    for g, phs in groups.items():
        ss = [x for x in segs if x[0] in phs]
        res[g] = (sum(x[2] - x[1] for x in ss), ss[0][3], ss[-1][4], len(ss))
    return res


def header(s, title, tag):
    s.text(48, 66, title, "s-title")
    s.text(W - 48, 64, tag, "s-tag", anchor="end")
    s.text(48, 104, "1 架 Pioneer＋2 架 HERO｜作業區 1560 × 800 m｜作業上限 120 m AGL（規劃天花板 119.5 m）｜規劃風 5 m/s，來自 270°",
           "s-sub")


def footer(s, txt):
    s.text(48, H - 22, txt, "s-xs")


def icon(s, x, y, sc=1.0, hover=False, face=1, cls_body="icon", cls_line="icon-l", cls_rotor="rotor"):
    """side-view QuadPlane pictogram; (x,y) = fuselage centre; face=+1 nose right."""
    f = face * sc

    def X(v):
        return x + v * f

    def Y(v):
        return y + v * sc
    s.add(f'<g>')
    s.path(f"M{X(-24):.1f},{Y(-3.5):.1f} L{X(18):.1f},{Y(-4.5):.1f} Q{X(28):.1f},{Y(-3):.1f} {X(29):.1f},{Y(0):.1f} "
           f"Q{X(28):.1f},{Y(3):.1f} {X(18):.1f},{Y(4):.1f} L{X(-24):.1f},{Y(2.5):.1f} Z", cls_body)
    s.polygon([(X(-18), Y(-3)), (X(-27), Y(-16)), (X(-21), Y(-16)), (X(-11), Y(-3))], cls_body)
    s.line(X(-15), Y(-7), X(17), Y(-7), cls_line)
    for mx in (-13, 15):
        s.line(X(mx), Y(-7), X(mx), Y(-11), cls_line)
        if hover:
            s.add(f'<ellipse cx="{X(mx):.1f}" cy="{Y(-12):.1f}" rx="{12*sc:.1f}" ry="{2.6*sc:.1f}"{s.a(cls_rotor)}/>')
            s.line(X(mx) - 12 * sc, Y(-12), X(mx) + 12 * sc, Y(-12), cls_line)
        else:
            s.line(X(mx) - 5 * sc, Y(-11.5), X(mx) + 5 * sc, Y(-11.5), cls_line)
    if not hover:
        s.add(f'<ellipse cx="{X(-26):.1f}" cy="{Y(0):.1f}" rx="{1.6*sc:.1f}" ry="{8*sc:.1f}"{s.a(cls_rotor)}/>')
    s.add('</g>')


def badge(s, x, y, n, cls="badge-hl", r=16):
    s.circle(x, y, r + 3, "surf-fill")
    s.circle(x, y, r, cls)
    s.text(x, y + 7, n, "s-bt", anchor="middle")


def hatch_defs(s):
    d = []
    for u in ("a1", "b2", "b3"):
        d.append(f'<pattern id="{s.label}-h-{u}" width="7" height="7" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
                 f'<rect width="7" height="7"{s.a(u + "-tint")}/><line x1="0" y1="0" x2="0" y2="7"{s.a("hl-" + u)}/></pattern>')
    s.add("<defs>" + "".join(d) + "</defs>")


# ───────────────────────── Slide 1: bird's-eye + storyboard ─────────────────────────
def slide1(mode):
    s = Svg(mode, W, H, "s1")
    header(s, "想定圖：異質 VTOL 群飛偵察任務", "HARVEST 規劃結果｜2026-10-21｜規劃值")
    D, B, ZEX = 1.0, math.radians(50), 1.8
    YCLIP = 640.0
    gx0, gx1, gy0, gy1 = -1320.0, 1320.0, -520.0, 660.0

    def raw(x, y, z):
        return (x + y * D * math.cos(B), -(y * D * math.sin(B) + z * ZEX))

    pts = [raw(x, y, z) for x in (gx0, gx1) for y in (gy0, gy1) for z in (0, 125)]
    rx0, rx1 = min(p[0] for p in pts), max(p[0] for p in pts)
    ry0, ry1 = min(p[1] for p in pts), max(p[1] for p in pts)
    bx0, bx1, by0, by1 = 48, W - 48, 132, 748
    S = min((bx1 - bx0) / (rx1 - rx0), (by1 - by0) / (ry1 - ry0))
    ox = bx0 + ((bx1 - bx0) - (rx1 - rx0) * S) / 2 - rx0 * S
    oy = by0 + ((by1 - by0) - (ry1 - ry0) * S) / 2 - ry0 * S

    def P(x, y, z=0.0):
        a, b = raw(x, y, z)
        return (ox + a * S, oy + b * S)

    # ground plane + grid
    s.polygon([P(gx0, gy0), P(gx1, gy0), P(gx1, gy1), P(gx0, gy1)], "ground")
    for gx in range(-1200, 1300, 200):
        a, b = P(gx, gy0), P(gx, gy1)
        s.line(*a, *b, "grid")
    for gy in range(-400, 660, 200):
        a, b = P(gx0, gy), P(gx1, gy)
        s.line(*a, *b, "grid")
    area = [P(-780, -400), P(780, -400), P(780, 400), P(-780, 400)]
    s.polygon(area, "area")
    clip = f"{s.label}-clip"
    s.add(f'<clipPath id="{clip}"><polygon points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in area)}"/></clipPath>')
    s.add(f'<g clip-path="url(#{clip})">')
    for u in UAVS:
        for y in sorted({round(w[1], 1) for w in T["uavs"][u]["wp"] if w[7]}):
            s.polygon([P(-900, y - SW_HALF), P(900, y - SW_HALF), P(900, y + SW_HALF), P(-900, y + SW_HALF)], f"{u.lower()}-sw")
    s.add("</g>")
    for y in (-133.34, 133.34):
        s.line(*P(-780, y), *P(780, y), "subz")
    s.polygon(area, "ink-line")

    def clipped(wp):
        out = []
        for w in wp:
            if w[1] > YCLIP:
                break
            out.append(w)
        return out

    # ground tracks
    for u in UAVS:
        wp = clipped(T["uavs"][u]["wp"])
        s.poly([P(w[0], w[1], 0) for w in wp], "trk")
    # drop lines at strip ends
    for u in UAVS:
        for y in sorted({round(w[1], 1) for w in T["uavs"][u]["wp"] if w[7]}):
            for x in (-848, 848):
                s.line(*P(x, y, 119.5), *P(x, y, 0), "drop")
    # height ruler (front-left corner of area)
    rx_, ry_ = -1180.0, -480.0
    s.line(*P(rx_, ry_, 0), *P(rx_, ry_, 120), "axis")
    for h, lab in ((0, "0"), (50, "50"), (100, "100"), (120, "120 m")):
        a = P(rx_, ry_, h)
        s.line(a[0] - 7, a[1], a[0] + 7, a[1], "axis")
        s.text(a[0] - 12, a[1] + 6, lab, "s-xs", anchor="end")
    a = P(rx_, ry_, 0)
    s.text(a[0], a[1] + 26, "高度放大 1.8 倍", "s-xs", anchor="middle")
    # paths (B3, B2 first, A1 on top)
    for u in ("B3", "B2", "A1"):
        wp = clipped(T["uavs"][u]["wp"])
        for k, seg in runs(wp):
            s.poly([P(w[0], w[1], w[2]) for w in seg], f"{u.lower()}-{k}")
    # B3 exit to ALT1
    b3 = T["uavs"]["B3"]["wp"]
    last = clipped(b3)[-1]
    nxt = next(w for w in b3 if w[1] > YCLIP)
    a, b = P(last[0], last[1], last[2]), P(nxt[0], nxt[1], nxt[2])
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    e = (a[0] + 46 * math.cos(ang), a[1] + 46 * math.sin(ang))
    s.poly([a, e], "b3-fw", s.mk("ah-b3"))
    s.text(e[0] - 14, e[1] + 4, "B3 返場 → 備降點 ALT1", "s-b3 halo", anchor="end")
    s.text(e[0] - 14, e[1] + 28, "（H 北方 2.5 km，降落程序同 ⑥–⑧）", "s-xs halo", anchor="end")
    # camera cone on A1 Z2 strip
    cx_, cy_ = 620.0, -169.4
    s.polygon([P(cx_, cy_, 119.5), P(cx_, cy_ - SW_HALF, 0), P(cx_, cy_ + SW_HALF, 0)], "cone")
    icon(s, *P(cx_, cy_, 119.5), sc=1.25, hover=False, face=1, cls_body="a1-fill", cls_line="a1-fw", cls_rotor="a1-tint")
    # pads
    for sid in ("TO_A1", "TO_B2", "TO_B3"):
        x, y, _z = T["sites"][sid]
        s.add(f'<ellipse cx="{P(x, y)[0]:.1f}" cy="{P(x, y)[1]:.1f}" rx="6" ry="3.4"{s.a("pad")}/>')
    # hover icon above the pad cluster
    icon(s, *P(0, 0, 60), sc=1.15, hover=True, face=-1)
    # phase badges on A1
    wa = T["uavs"]["A1"]["wp"]

    def mid(phase, nth=0):
        idx = [i for i, w in enumerate(wa) if w[5] == phase]
        groups, cur = [], [idx[0]]
        for i in idx[1:]:
            if i == cur[-1] + 1:
                cur.append(i)
            else:
                groups.append(cur)
                cur = [i]
        groups.append(cur)
        g = groups[nth]
        w = wa[g[len(g) // 2]]
        return P(w[0], w[1], w[2])

    badges = [
        ("1", P(-10, 0, 28), (-34, 0), "垂直起飛", "end"),
        ("2", mid("TRANS_OUT"), (0, -34), "轉換", "middle"),
        ("3", mid("ENTRY"), (-30, -26), "爬升進場", "end"),
        ("4", P(-300, -169.4, 119.5), (0, -34), "條帶作業", "middle"),
        ("5", mid("TRANSFER", 0), (34, 0), "迴轉", "start"),
        ("6", mid("RTB"), (0, -34), "返場", "middle"),
        ("7", mid("TRANS_IN"), (0, -34), "回轉換", "middle"),
        ("8", P(0, 0, 28), (34, 0), "垂直降落", "start"),
    ]
    for n, (x, y), (dx, dy), lab, anc in badges:
        bx, by = x + dx, y + dy
        s.line(x, y, bx, by, "ink-line")
        s.circle(x, y, 3.5, "ink-fill")
        badge(s, bx, by, n)
        if anc == "end":
            s.text(bx - 24, by + 7, lab, "s-h halo", anchor="end")
        elif anc == "start":
            s.text(bx + 24, by + 7, lab, "s-h halo")
        else:
            s.text(bx, by - 26, lab, "s-h halo", anchor="middle")
    # pad callout
    p0 = P(10, 0, 0)
    s.poly([p0, (p0[0] + 150, p0[1] + 150), (p0[0] + 190, p0[1] + 150)], "ink-line")
    s.text(p0[0] + 196, p0[1] + 157, "起降場：TO_A1｜H（＝TO_B2）｜TO_B3，坪距 10 m", "s-s halo")
    # wind + north
    wx, wy = 60, 396
    s.poly([(wx, wy), (wx + 96, wy)], "arrow-acc", s.mk("ah-acc"))
    s.text(wx, wy + 30, "風 5 m/s，來自 270°（西風）", "s-acc")
    q0, q1 = P(0, 0, 0), P(0, 100, 0)
    L_ = math.hypot(q1[0] - q0[0], q1[1] - q0[1])
    ux, uy = (q1[0] - q0[0]) / L_, (q1[1] - q0[1]) / L_
    n0 = (1790, 470)
    n1 = (n0[0] + 80 * ux, n0[1] + 80 * uy)
    s.poly([n0, n1], "ink-line", s.mk("ah"))
    s.text(n1[0] + 8, n1[1] + 4, "N", "s-lab")
    # legend (top-right)
    lx, ly = 60, 160
    for i, u in enumerate(UAVS):
        yy = ly + i * 30
        s.line(lx, yy, lx + 44, yy, f"{u.lower()}-op")
        s.text(lx + 56, yy + 7, NAME[u], f"s-{u.lower()}")
    for i, (c, t) in enumerate((("a1-op", "條帶作業（相機拍照）"), ("a1-fw", "定翼飛行（進場、迴轉、返場）"),
                                ("a1-tr", "轉換（旋翼 ⇄ 定翼）"), ("a1-vt", "垂直起降（旋翼）"))):
        yy = ly + 100 + i * 28
        s.line(lx, yy, lx + 44, yy, c)
        s.text(lx + 56, yy + 6, t, "s-s")
    # ── storyboard ──
    st = {u: phase_stats(u) for u in ("A1", "B2")}
    cells = [
        ("1", "垂直起飛", "旋翼模式垂直爬升", "up"),
        ("2", "轉換", "迎風轉為定翼模式", "trout"),
        ("3", "爬升進場", "加速、爬升、轉入條帶", "climb"),
        ("4", "條帶作業", "定高掃描、距離觸發拍照", "op"),
        ("5", "條帶間迴轉", "依轉彎半徑迴轉", "turn"),
        ("6", "返場", "飛回回收坪並下降", "rtb"),
        ("7", "回轉換", "迎風減速轉回旋翼", "trin"),
        ("8", "垂直降落", "旋翼模式降落於坪", "down"),
    ]
    cw, gap, cx0, cy0, ch = 222, 8, 48, 772, 250
    for i, (n, name, desc, sk) in enumerate(cells):
        x = cx0 + i * (cw + gap)
        s.rect(x, cy0, cw, ch, "panel", rx=10)
        badge(s, x + 26, cy0 + 28, n, r=14)
        s.text(x + 48, cy0 + 36, name, "s-h")
        draw_sketch(s, sk, x + 14, cy0 + 56, cw - 28, 82)
        s.text(x + 14, cy0 + 164, desc, "s-s")
        for j, u in enumerate(("A1", "B2")):
            dur, z0, z1, cnt = st[u][n]
            if n == "4":
                val = f"{cnt} 條，共 {dur:.0f} s"
            elif n == "5":
                val = f"{cnt} 次，共 {dur:.0f} s"
            else:
                val = f"{dur:.0f} s｜{fz(z0)}→{fz(z1)} m"
            s.text(x + 14, cy0 + 198 + j * 27, u, f"s-v{u.lower()}")
            s.text(x + 46, cy0 + 198 + j * 27, val, "s-v")
    footer(s, "資料：以 2026-10-21 工作樹的 HARVEST 重規劃情境 gui_legal_120_1a2b（天花板 119.5 m），validate 通過，"
              "尚未飛驗。分鏡數值 A1＝Pioneer、B2＝HERO；B3 流程同 B2（4 條帶，降於 ALT1）。")
    return s.render("想定圖：異質 VTOL 群飛偵察任務（鳥瞰）")


def fz(z):
    return f"{z:.0f}" if abs(z - round(z)) < 0.05 else f"{z:.1f}"


def draw_sketch(s, kind_, x, y, w, h):
    gy = y + h - 6
    if kind_ != "turn":
        s.line(x, gy, x + w, gy, "sk-g")
    top = y + 14
    if kind_ == "up":
        s.rect(x + w / 2 - 16, gy - 3, 32, 5, "ink-fill", rx=2)
        s.poly([(x + w / 2, gy - 8), (x + w / 2, top + 26)], "sk", s.mk("ah"))
        icon(s, x + w / 2, top + 10, sc=0.95, hover=True)
    elif kind_ == "down":
        s.rect(x + w / 2 - 16, gy - 3, 32, 5, "ink-fill", rx=2)
        s.poly([(x + w / 2, top + 22), (x + w / 2, gy - 10)], "sk", s.mk("ah"))
        icon(s, x + w / 2, top + 10, sc=0.95, hover=True, face=-1)
    elif kind_ == "trout":
        s.poly([(x + 10, gy - 24), (x + w - 50, top + 12)], "sk-d", s.mk("ah"))
        icon(s, x + w - 30, top + 8, sc=0.85)
    elif kind_ == "trin":
        s.poly([(x + 44, top + 8), (x + w - 14, gy - 22)], "sk-d", s.mk("ah"))
        icon(s, x + 26, top + 6, sc=0.85, hover=True)
    elif kind_ == "climb":
        s.line(x, top, x + w, top, "sk-lvl")
        s.path(f"M{x+6:.1f},{gy-18:.1f} C{x+w*0.45:.1f},{gy-18:.1f} {x+w*0.5:.1f},{top:.1f} {x+w-48:.1f},{top:.1f}", "sk", s.mk("ah"))
        icon(s, x + w - 26, top - 2, sc=0.85)
    elif kind_ == "op":
        cx = x + w / 2
        s.polygon([(cx, top + 4), (cx - 46, gy - 2), (cx + 46, gy - 2)], "cone-k")
        s.poly([(x + 6, top + 4), (x + w - 6, top + 4)], "sk", s.mk("ah"))
        icon(s, cx, top, sc=0.85)
    elif kind_ == "turn":
        cy = y + h / 2 - 2
        s.path(f"M{x+20:.1f},{cy-22:.1f} L{x+w-60:.1f},{cy-22:.1f} A22,22 0 0 1 {x+w-60:.1f},{cy+22:.1f} L{x+44:.1f},{cy+22:.1f}",
               "sk", s.mk("ah"))
        s.text(x + w - 30, cy + 6, "R", "s-lab")
    elif kind_ == "rtb":
        s.rect(x + w - 30, gy - 3, 24, 5, "ink-fill", rx=2)
        s.path(f"M{x+6:.1f},{top+4:.1f} L{x+w*0.45:.1f},{top+4:.1f} L{x+w-40:.1f},{top+26:.1f}", "sk", s.mk("ah"))
        icon(s, x + 36, top, sc=0.85)


# ───────────────────────── Slide 2: timeline + altitude ─────────────────────────
def slide2(mode):
    s = Svg(mode, W, H, "s2")
    hatch_defs(s)
    header(s, "任務時程與高度剖面", "HARVEST 規劃結果｜2026-10-21｜規劃值")
    X0, X1, T1 = 250, 1860, 920.0

    def tx(t):
        return X0 + t / T1 * (X1 - X0)

    # Gantt
    s.text(48, 162, "各機時程（依航段）", "s-lab")
    gy0 = 182
    for i, u in enumerate(UAVS):
        d = T["uavs"][u]
        y = gy0 + i * 64
        s.text(48, y + 26, NAME[u], f"s-{u.lower()}")
        s.text(48, y + 50, f"{d['takeoff_site']} → {d['landing_site']}｜{d['t_takeoff_s']:.0f}→{d['t_land_s']:.1f} s", "s-xs")
        s.rect(tx(0), y + 6, tx(T1) - tx(0), 36, "gwait", rx=4)
        wp = d["wp"]
        for k, seg in runs(wp):
            t0, t1 = seg[0][4], seg[-1][4]
            if t1 - t0 <= 0:
                continue
            cls = {"vt": "hover-f", "op": f"{u.lower()}-fill", "fw": f"{u.lower()}-tint"}.get(k)
            if k == "tr":
                s.add(f'<rect x="{tx(t0):.1f}" y="{y+6}" width="{tx(t1)-tx(t0):.1f}" height="36" fill="url(#{s.label}-h-{u.lower()})"/>')
            else:
                s.rect(tx(t0), y + 6, tx(t1) - tx(t0), 36, cls, rx=0)
        tl = d["t_land_s"]
        last_t = wp[-1][4]
        if tl > last_t:
            s.rect(tx(last_t), y + 6, tx(tl) - tx(last_t), 36, "hover-f", rx=0)
        if d["t_takeoff_s"] > 60:
            s.text(tx(d["t_takeoff_s"] / 2), y + 31, f"地面等待 {d['t_takeoff_s']:.0f} s", "s-s", anchor="middle")
    ms = T["makespan_s"]
    s.line(tx(ms), gy0 - 4, tx(ms), gy0 + 3 * 64 + 4, "mspan")
    s.text(tx(ms) - 8, gy0 - 10, f"makespan {ms:.1f} s", "s-lab", anchor="end")
    # legend for Gantt
    ly = gy0 + 3 * 64 + 22
    items = [("hover-f", "垂直起降（旋翼）"), ("hatch", "轉換"), ("a1-tint", "定翼飛行（進場、迴轉、返場）"), ("a1-fill", "條帶作業（拍照）"),
             ("gwait", "地面待命")]
    xx = X0
    for c, t in items:
        if c == "hatch":
            s.add(f'<rect x="{xx}" y="{ly}" width="34" height="20" fill="url(#{s.label}-h-a1)"/>')
        else:
            s.rect(xx, ly, 34, 20, c, rx=2)
        s.text(xx + 44, ly + 17, t, "s-s")
        xx += 44 + len(t) * 17 + 46
    s.text(xx, ly + 17, "（色彩依機號）", "s-xs")
    # altitude chart
    cy0, cy1 = 520, 960
    A1m = 130.0

    def ay(h):
        return cy1 - h / A1m * (cy1 - cy0)

    s.text(48, cy0 - 22, "相對高度（規劃航點）", "s-lab")
    for h in (0, 20, 40, 60, 80, 100, 120):
        s.line(X0, ay(h), X1, ay(h), "grid")
        s.text(X0 - 12, ay(h) + 6, f"{h} m", "s-xs", anchor="end")
    s.line(X0, ay(120), X1, ay(120), "lim")
    s.text(X1, ay(120) - 8, "作業上限 120 m", "s-lim", anchor="end")
    s.line(X0, ay(100), X1, ay(100), "minfw")
    for u in UAVS:
        wp = T["uavs"][u]["wp"]
        for k, seg in runs(wp):
            pts = [(tx(w[4]), ay(max(w[2], 0))) for w in seg]
            s.poly(pts, {"vt": f"{u.lower()}-vt", "tr": f"{u.lower()}-tr", "op": f"{u.lower()}-op",
                         "fw": f"{u.lower()}-ln"}[k])
    # phase badges on A1 curve
    wa = T["uavs"]["A1"]["wp"]

    def first(phase):
        idx = [i for i, w in enumerate(wa) if w[5] == phase]
        return idx

    marks = [("1", wa[0][4] + 15, 30), ("2", None, "TRANS_OUT"), ("3", None, "ENTRY"), ("4", None, "OPERATE"),
             ("5", None, "TRANSFER"), ("6", None, "RTB"), ("7", None, "TRANS_IN"), ("8", wa[-1][4] - 35, 30)]
    for n, t, ph in marks:
        if t is None:
            idx = first(ph)
            w = wa[idx[len(idx) // 6] if ph in ("OPERATE", "TRANSFER") else idx[len(idx) // 2]]
            t, z = w[4], w[2]
        else:
            z = ph
        x, y = tx(t), ay(z)
        dy = -34 if z > 90 else 0
        dx = 0 if z > 90 else (-30 if n == "1" else 30)
        if z > 110:
            dy = 40
        s.line(x, y, x + dx, y + dy, "ink-line")
        badge(s, x + dx, y + dy, n, r=13)
    s.text(tx(575), ay(40), "圓圈標號＝A1 的航段，對應鳥瞰圖分鏡 ①–⑧", "s-s")
    s.line(X0, cy1, X1, cy1, "axis")
    for t in range(0, 901, 100):
        s.line(tx(t), cy1, tx(t), cy1 + 7, "axis")
        s.text(tx(t), cy1 + 30, f"{t}", "s-xs", anchor="middle")
    s.text(X1, cy1 + 58, "任務時間（s，自第一架起飛起算）", "s-xs", anchor="end")
    footer(s, "資料：以 2026-10-21 工作樹的 HARVEST 重規劃 gui_legal_120_1a2b；validate 通過、四維衝突 0、規劃最小水平間距 "
              f"{T['validate']['min_sep_h_m']:.1f} m；尚未飛驗。")
    return s.render("任務時程與高度剖面（規劃值）")


# ───────────────────────── Slide 3: to-scale plan view ─────────────────────────
def slide3(mode):
    s = Svg(mode, W, H, "s3")
    header(s, "想定圖：平面（真實比例）", "HARVEST 規劃結果｜2026-10-21｜規劃值")
    X0, X1, Y0, Y1 = -1320.0, 1300.0, -500.0, 470.0
    mx0, my0, mw = 48, 130, W - 96
    S = mw / (X1 - X0)
    mh = (Y1 - Y0) * S

    def P(x, y):
        return (mx0 + (x - X0) * S, my0 + (Y1 - y) * S)

    s.rect(mx0, my0, mw, mh, "panel", rx=8)
    for gx in range(-1200, 1300, 200):
        s.line(P(gx, Y1)[0], my0 + 1, P(gx, Y0)[0], my0 + mh - 1, "grid")
    for gy in range(-400, 470, 200):
        s.line(mx0 + 1, P(X0, gy)[1], mx0 + mw - 1, P(X1, gy)[1], "grid")
    area = [P(-780, -400), P(780, -400), P(780, 400), P(-780, 400)]
    s.polygon(area, "area")
    clip = f"{s.label}-clip"
    ax0, ay0 = P(-780, 400)
    ax1, ay1 = P(780, -400)
    s.add(f'<clipPath id="{clip}"><rect x="{ax0:.1f}" y="{ay0:.1f}" width="{ax1-ax0:.1f}" height="{ay1-ay0:.1f}"/></clipPath>')
    s.add(f'<g clip-path="url(#{clip})">')
    for u in UAVS:
        for y in sorted({round(w[1], 1) for w in T["uavs"][u]["wp"] if w[7]}):
            a, b = P(-900, y + SW_HALF), P(900, y - SW_HALF)
            s.rect(a[0], a[1], b[0] - a[0], b[1] - a[1], f"{u.lower()}-sw", rx=0)
    s.add("</g>")
    for y in (-133.34, 133.34):
        s.line(*P(-780, y), *P(780, y), "subz")
    s.polygon(area, "ink-line")
    mclip = f"{s.label}-mclip"
    s.add(f'<clipPath id="{mclip}"><rect x="{mx0}" y="{my0}" width="{mw}" height="{mh:.1f}"/></clipPath>')
    s.add(f'<g clip-path="url(#{mclip})">')
    for u in ("B3", "B2", "A1"):
        for k, seg in runs(T["uavs"][u]["wp"]):
            s.poly([P(w[0], w[1]) for w in seg], f"{u.lower()}-{k}" if k != "vt" else f"{u.lower()}-fw")
    s.add("</g>")
    # direction arrows on strips
    for u in UAVS:
        wp = T["uavs"][u]["wp"]
        ys = sorted({round(w[1], 1) for w in wp if w[7]})
        for y in ys:
            seg = [w for w in wp if w[7] and abs(w[1] - y) < 1]
            d = 1 if seg[-1][0] > seg[0][0] else -1
            mx, my = P(0, y)
            s.poly([(mx - 12 * d, my), (mx + 10 * d, my)], f"{u.lower()}-op", s.mk(f"ah-{u.lower()}"))
    # labels
    for name, ym in (("子區 Z4_1", 313.0), ("子區 Z3_1", 46.5), ("子區 Z2_1", -220.5)):
        px, py = P(-770, ym)
        s.text(px, py + 7, name, "s-lab halo")
    for u, txt, x, y in (("A1", "A1 Pioneer：每子區 1 條 A 帶", -420, 400), ("B3", "B3 HERO：Z3_1、Z4_1 各 2 條", -420, 261),
                         ("B2", "B2 HERO：Z2_1 2 條", 260, -367)):
        px, py = P(x, y)
        s.text(px, py - 10, txt, f"s-{u.lower()} halo")
    hx, hy = P(0, 0)
    s.circle(hx, hy, 8, "pad")
    s.circle(hx, hy, 3, "ink-fill")
    s.poly([(hx + 8, hy + 6), (hx + 30, hy + 30), (hx + 80, hy + 30)], "ink-line")
    s.text(hx + 86, hy + 37, "起降場 H（3 坪，坪距 10 m）", "s-s halo")
    # B3 exit to ALT1
    b3 = T["uavs"]["B3"]["wp"]
    w0 = next(w for w in b3 if w[1] > Y1 - 30)
    ex, ey = P(w0[0], Y1)
    s.poly([(ex, ey + 40), (ex, ey + 4)], "b3-fw", s.mk("ah-b3"))
    s.text(ex - 12, ey + 28, "B3 → 備降點 ALT1（北 2.5 km）", "s-b3 halo", anchor="end")
    # wind, north, scale
    wx, wy = P(-1290, 430)
    s.poly([(wx, wy), (wx + 90, wy)], "arrow-acc", s.mk("ah-acc"))
    s.text(wx, wy + 30, "風 5 m/s，來自 270°", "s-acc")
    nx, ny = P(1260, 440)
    s.polygon([(nx, ny), (nx - 9, ny + 26), (nx, ny + 20), (nx + 9, ny + 26)], "ink-fill")
    s.text(nx, ny + 48, "N", "s-lab", anchor="middle")
    bx, by = P(-1290, -460)
    s.rect(bx, by - 8, 200 * S, 8, "ink-fill", rx=0)
    s.rect(bx + 200 * S, by - 8, 300 * S, 8, "surf-fill", rx=0)
    for v, lbl in ((0, "0"), (200, "200"), (500, "500 m")):
        s.text(bx + v * S, by + 18, lbl, "s-xs", anchor="middle" if v else "start")
    # bottom panel: legend | fleet table | locator
    py0 = my0 + mh + 22
    s.text(48, py0 + 20, "圖例", "s-lab")
    for i, (c, t) in enumerate((("a1-op", "條帶作業（拍照）"), ("a1-fw", "定翼飛行"), ("a1-tr", "轉換（旋翼 ⇄ 定翼）"),
                                ("a1-sw", "相機刈幅 138.0 m"), ("subz", "子區界線"))):
        yy = py0 + 50 + i * 30
        if c.endswith("-sw"):
            s.rect(48, yy - 14, 44, 20, c, rx=0)
        else:
            s.line(48, yy - 4, 92, yy - 4, c)
        s.text(104, yy + 2, t, "s-s")
    tx0 = 440
    cols = [(0, "機號"), (150, "起飛坪 → 降落坪"), (380, "條帶"), (470, "航程"), (580, "起飛 → 落地（s）"), (800, "能量使用／餘裕")]
    s.text(tx0, py0 + 20, "機隊與分工（規劃值）", "s-lab")
    for cx, h in cols:
        s.text(tx0 + cx, py0 + 52, h, "s-xs")
    s.line(tx0, py0 + 62, tx0 + 1000, py0 + 62, "rule")
    for i, u in enumerate(UAVS):
        d = T["uavs"][u]
        yy = py0 + 94 + i * 34
        row = [NAME[u], f"{d['takeoff_site']} → {d['landing_site']}", f"{len(d['strips'])}", f"{d['distance_m']/1000:.2f} km",
               f"{d['t_takeoff_s']:.0f} → {d['t_land_s']:.1f}", f"{d['energy_used_wh']:.0f}／{d['energy_margin_wh']:.0f} Wh"]
        for j, (cx, _h) in enumerate(cols):
            s.text(tx0 + cx, yy, row[j], f"s-{u.lower()}" if j == 0 else "s-t")
    s.text(tx0, py0 + 200, f"makespan {T['makespan_s']:.1f} s｜validate 通過、四維衝突 0｜B3 由回收站自動指派改降 ALT1",
           "s-s")
    # locator
    lx0, ly0, lw, lh = W - 330, py0 - 4, 282, 236
    s.rect(lx0, ly0, lw, lh, "panel", rx=8)
    s.text(lx0 + 14, ly0 + 28, "位置圖（含 ALT1）", "s-lab")
    LS = (lh - 56) / 3150.0
    lcx = lx0 + lw / 2

    def L(x, y):
        return (lcx + x * LS, ly0 + lh - 18 - (y + 500) * LS)

    s.polygon([L(-780, -400), L(780, -400), L(780, 400), L(-780, 400)], "area")
    s.poly([L(w[0], w[1]) for w in T["uavs"]["B3"]["wp"]], "b3-fw")
    s.circle(*L(0, 0), 4, "ink-fill")
    s.circle(*L(0, 2500), 5, "pad")
    s.text(L(0, 2500)[0] + 12, L(0, 2500)[1] + 6, "ALT1", "s-lab")
    s.text(L(-780, 0)[0] - 8, L(0, -400)[1] + 4, "作業區", "s-xs", anchor="end")
    a, b = L(-1100, 0), L(-1100, 2500)
    s.line(*a, *b, "axis")
    s.text(a[0] - 6, (a[1] + b[1]) / 2, "2.5 km", "s-xs", anchor="end")
    footer(s, "資料：以 2026-10-21 工作樹的 HARVEST 重規劃 gui_legal_120_1a2b（天花板 119.5 m）；航線為規劃航點的平面近似 ENU，"
              "原點＝起降場 H；格線 200 m；尚未飛驗。")
    return s.render("想定圖：平面（真實比例）")


if __name__ == "__main__":
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for name, fn in (("slide1_birdseye", slide1), ("slide2_timeline", slide2), ("slide3_plan", slide3)):
        (out / f"{name}.css.svg").write_text(fn("css"), encoding="utf-8")
        (out / f"{name}.svg").write_text(fn("lit"), encoding="utf-8")
    (out / "fig.css").write_text(css_rules(), encoding="utf-8")
    for u in ("A1", "B2"):
        print(u, {k: (round(v[0], 1), round(v[1], 1), round(v[2], 1), v[3]) for k, v in phase_stats(u).items()})
