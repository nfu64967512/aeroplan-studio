# -*- coding: utf-8 -*-
"""build_pioneer_glb.py — MFE Pioneer（开拓者）4+1 VTOL 的 3D 模型 → ui/resources/pioneer.glb（CesiumJS 用，實際尺寸）。

來源：原廠在 gitee 開源的 Pioneer CAD（makeflyeasy/MFE_ArduPlane，Pioneer/飞机图纸Aircraft drawings，commit 9ecb2615）。
  那份資料夾只有 5 個 STEP：機頭（掛載型）、載荷艙、機身尾部（含背鰭、水平尾座、腹鰭座、尾推座）、腹鰭（「垂尾」）、
  翼尖航燈外殼。**沒有**主翼、水平尾、機身中段（x 350→850 的 EPO 左右機身）、機頭蓋、座艙／電池艙蓋、機臂、
  馬達、槳、起落架。⇒ 有 STEP 的照 STEP（只平移不改形），沒有的依原廠尺寸參數化補上：
    * 尺寸主要依據：原廠貼紙示意圖（endoc.makeflyeasy.com/Pioneer/Pioneer_sticker_illustration.jpg，上／下／側三視、
      0.8454 mm/px：翼展 3200、機身長 1678、機臂 1081、旋翼間距 998.5，CAD 剪影誤差中位 0.8 mm）、手冊 V1.0 第 5 頁俯視圖
      （3.79 mm/px）、經銷商三視圖（3200/1680/416）、旋翼／巡航電機座 PDF。
    * STEP 已在組裝位置（同一個 CAD 座標）；只有航燈外殼被排在前面 ⇒ x +100.5（燈罩前緣對齊示意圖翼尖前緣，最小平方 100.35、
      殘差 0.64 mm；早先 +89 是對整個外殼剪影擬合的，會讓燈罩凸出前緣 11–15 mm，審查 2026-10-23）。
    * 主翼翼型：載荷艙機翼接合面（|y|=168 的平面）剖面＝翼根外形加 7 mm 整流餘隙 ⇒ 以弦中點縮到 331.5，前緣落在 x≈0、
      後緣 331（示意圖 330.9）；安裝角照 CAD（3.27°，手冊寫 2.9°）。等弦到 |y|≈580，外段後緣前掠到翼尖 288.6，
      前緣在 |y| 1440 之後圓角（示意圖逐點）。
    * 「垂尾」STEP 是**機腹垂尾**（兼尾橇），在位置上、沒有翻：根部榫頭與機身尾部的榫槽面面相合（4085 mm²）。
  量測與來源細節：docs/實作_2026-10-23_Pioneer3D模型.md。共用工具：tools/glb_kit.py。

座標：同 Hero（CAD mm：x 往後、y 往右翼、z 往上；x=0 ≈ 翼根前緣）→ glTF 公尺、+X 機鼻、+Y 上、+Z 右翼，原點＝重心
  （手冊第 5 頁重心掛環：弦長 28.2% ≈ 前緣後 93.3 mm）⇒ main.html 校正 0/0/0、scale 1。

用法（只有建模時需要 cascadio、trimesh、shapely、numpy）：
  python -I tools/build_pioneer_glb.py fetch [--dst runs/artifacts/PIONEER_MODEL/step]
  python -I tools/build_pioneer_glb.py build [--src ...] [--out ui/resources/pioneer.glb] [--report 報告.json] [--check] [--cache 目錄]
  ⚠ 原廠資料夾沒有授權檔 ⇒ STEP 不進 git，pioneer.glb 跟其他 GLB 一樣不追蹤。
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import glb_kit as gk                                                # noqa: E402
from glb_kit import (Part, box, cylinder, loft, merged, mirror_y, naca4, offset_profile,   # noqa: E402,F401
                     propeller, resample_loop_polar, resample_profile, rounded_box, section_outline)

ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_STEP_DIR = ROOT / 'runs' / 'artifacts' / 'PIONEER_MODEL' / 'step'
DEFAULT_OUT = ROOT / 'ui' / 'resources' / 'pioneer.glb'

GITEE_REPO = 'makeflyeasy/MFE_ArduPlane'
GITEE_COMMIT = '9ecb2615840d69a0243f7628a0fed3c34343cc66'     # Pioneer 資料夾最後一次變更（2026-01-21）
GITEE_DIR = 'Pioneer/飞机图纸Aircraft drawings'
SOURCES = {
    'nose': ('开拓者机头（Pioneer head）.step', '73998dc7d2254e42fee078a6608fea60f0968d3f'),
    'payload_bay': ('开拓者载荷舱（Pioneer payload compartment）.step', '534c651ee415ced0c746307b90cf8e60dc901953'),
    'fuselage_rear': ('开拓者机身尾部（Pioneer fuselage rear）.step', 'd34ac03cf83adbc3e6aac2ee1c543f6f90cdc2e5'),
    'tail': ('开拓者垂尾（Pioneer tail）.step', '7ff66f180860c88fab736ed97db91fb1c632c1a9'),
    'nav_light': ('开拓者航灯外壳（Pioneer navigation light housing）.stp', '0d75fb9ce88d1708c6e4ac093d05ba12038f0565'),
}
STEP_TOL = (0.6, 0.5)

# ── 幾何常數（mm，CAD 座標）──────────────────────────────────────────────────
NAV_LIGHT_SHIFT_X = 100.5          # 航燈外殼唯一被排開的零件（y／z 已對）；燈罩前緣對齊翼尖前緣；左翼那個是鏡像

# 主翼
WING_ROOT_Y = 168.0                # 載荷艙機翼接合面（平面、法向 ±y）
WING_ROOT_SECTION_Y = 167.5        # 在接合面內側一點剖 ⇒ 接合面外形
WING_CHORD = 331.5                 # 示意圖：前緣 −0.5、後緣 330.9（手冊俯視 331.3）
WING_TE = (330.9, 356.6, 0.0424)   # 後緣 x = min(330.9, 356.6 − 0.0424·|y|)（外段前掠 2.4°；翼尖 288.6）
WING_TIP_DROOP = (1545.0, 18.0)    # 翼尖下垂：從 |y| 1545 起、到翼尖 18 mm（示意圖／經銷商前視 12–19），前緣側多、後緣側少
# 翼尖前緣圓角（示意圖俯視，|y| → 前緣 x）
WING_TIP_LE = [(1440.0, 0.0), (1468.9, 1.2), (1485.8, 2.9), (1502.7, 4.6), (1519.6, 8.8), (1536.5, 12.2),
               (1553.4, 18.9), (1564.4, 24.9), (1571.2, 30.8), (1577.9, 37.5), (1584.7, 48.5), (1589.8, 60.4),
               (1593.2, 72.2), (1596.5, 89.1), (1598.2, 105.2), (1599.4, 120.0)]   # 末點＝翼尖（翼展 3198.8；示意圖 3200）
WING_ROOT_STRAP = (178.0, 210.0, 0.6)   # 翼根黑色束帶（示意圖／渲染 |y| 178–210，從前緣到後緣）

# 水平尾（中置在尾錐兩側，z≈46）
HTAIL_ROOT_Y, HTAIL_TIP_Y = 63.0, 456.1
HTAIL_LE = [(63.0, 990.3), (99.3, 991.2), (116.2, 992.8), (167.0, 1000.5), (234.6, 1008.9), (302.2, 1018.2),
            (369.9, 1027.5), (420.6, 1034.3), (437.5, 1036.8), (442.6, 1039.3), (447.6, 1045.3), (451.0, 1052.9),
            (452.7, 1060.5), (454.4, 1069.8), (455.6, 1080.0)]
HTAIL_TE_X = 1207.6
HTAIL_LE_Z, HTAIL_TE_Z = 46.7, 45.0     # 水平尾座：安裝角 +0.44°
HTAIL_AIRFOIL = '0010'                  # 座面厚 22.7／弦 219.8 ≈ 10.3%
HTAIL_ROOT_STRAP = (63.0, 93.0, 0.6)    # 根部黑色套座（渲染 |y| 62–93）

# 機身中段（EPO 左右機身，x 349.6 → 849.6 沒有 STEP）：示意圖俯視半寬／側視上下（兩視圖互差 ≤0.8 mm）
MID_FUSE = [  # x, 半寬, 頂, 底
    (349.0, 118.0, 124.0, -100.5), (400.0, 112.0, 120.2, -96.2), (440.0, 108.6, 117.7, -92.0),
    (500.0, 103.6, 114.3, -86.9), (560.0, 98.5, 110.1, -82.7), (620.0, 93.4, 105.9, -76.8),
    (680.0, 88.3, 101.6, -72.5), (740.0, 83.3, 96.6, -67.5), (800.0, 79.0, 92.3, -63.2), (856.0, 75.2, 88.5, -59.2)]
REAR_SHAPE_X = 856.0               # 中段剖面形狀＝機身尾部在這裡的剖面（正規化），兩端接得起來

# 座艙蓋／電池艙蓋（STEP 頂部開口）：示意圖側視頂線
HATCH_TOP = [(-380.0, 71.2), (-360.0, 72.0), (-340.0, 79.6), (-320.0, 87.3), (-300.0, 93.2), (-280.0, 99.1),
             (-260.0, 104.2), (-240.0, 107.5), (-220.0, 110.9), (-200.0, 114.3), (-180.0, 116.8), (-160.0, 119.4),
             (-140.0, 121.1), (-120.0, 122.8), (-100.0, 124.5), (-80.0, 125.3), (-60.0, 127.0), (-40.0, 127.8),
             (-20.0, 128.7), (0.0, 129.5), (20.0, 130.4), (60.0, 131.2), (160.0, 131.2), (200.0, 130.4),
             (240.0, 128.7), (280.0, 127.0), (320.0, 125.3), (349.0, 124.0)]
CANOPY_X = (-364.0, -75.0)         # 座艙蓋：示意圖俯視接縫 -364.5..-76.2（蓋子比開口大，蓋住前後的過渡）
CANOPY_RIM_X = (-358.0, -86.0)     # 量開口邊緣只在 STEP 真的開著的範圍（-362 以前開口還很窄；-84 之後機頭後環收上來）
BATTERY_HATCH_X = (25.0, 349.0)    # 載荷艙電池艙開口（示意圖 23.6..349.9；x≈11..19 是一根橫樑）
FRONT_HATCH = (-27.0, 7.0, 72.0, 121.0)   # 載荷艙前段窄開口（|y|≈70、z≈122）：x 範圍、半寬、邊緣 z
RIM_SMOOTH = 5                     # 開口邊緣沿 x 移動平均的點數（STEP 內部結構會讓個別站位跳）
HEAD_CONNECTOR_X = (-64.0, -27.0)  # 機頭與載荷艙之間的塑料接頭框（手冊 p9 #3）：接到前段小艙蓋，載荷艙前端斜面才不會露洞
NOSE_CAP = (-428.0, -450.0, (0.0, -20.0))   # 機頭前面的蓋：CAD 前端開口在 -428，蓋尖 (-450, y0, z-20)
NOSE_CAP_EXPO = 1.6               # 蓋的輪廓 (1−t^e)^(1/e)：1.5 尖端太尖、餘弦太胖（審查對示意圖量）

# 垂起機臂、馬達、槳
BOOM_Y, BOOM_Z, BOOM_D = 499.3, 60.2, 28.0          # 示意圖：機臂 ±499.3、軸 z 60.2、Ø28（電機座 PDF 套筒 28.1）
LIFT_HUB_X = (-400.1, 598.4)                        # 旋翼間距 998.5（MFE：1 m）
BOOM_END_X = (-441.4, 639.5)                        # 含電機座 1081（手冊 1082）
MOUNT_L, MOUNT_W, MOUNT_Z = 80.2, 47.6, (44.1, 69.1)  # 旋翼電機座 PDF 頭部 47.6×80.2；示意圖側視頂 69.1／底 44.1
BOOM_BLOCKS_X = ((-79.0, -11.0), (346.0, 403.0))    # 機臂在前緣前、後緣後的固定座（俯視）
LIFT_MOTOR_D, LIFT_MOTOR_H = 72.8, 42.4             # M6518（手冊 72.8×42.4）
LIFT_PROP_D = 558.8                                  # 22 吋（官方俯視渲染 559.6）

# 尾推
PUSHER_FACE_X, PUSHER_Z = 1230.0, 11.08             # 機身尾部後端面（法向正 +x、無上翹）、74.2 方孔中心
PUSHER_MOTOR_D, PUSHER_MOTOR_L = 72.8, 50.4         # X6525
PUSHER_PROP_X, PUSHER_SPINNER_TIP_X = 1287.0, 1318.0  # 官方俯視渲染（低信心 ±8）
PUSHER_PROP_D = 533.4                               # 21 吋（淺灰）

# 起落架（兩支折疊扁片腳，前視外撇 ≈35°）＋腹鰭下的玻纖加強片
GEAR_ROOT = (-29.0, 118.0, -81.0)                   # 側視：前緣 -46.6、後緣 -11.5；根部在載荷艙外皮內 ~1 mm、腳座盒裡
GEAR_FOOT = (-32.5, 257.0, -269.0)                  # 前視腳距 ±257；地面 z -269 ⇒ 外撇 atan(139/188)=36.5°（經銷商前視 36.1–36.9°）
GEAR_W, GEAR_T = (35.0, 27.4), 6.0
FIN_BRACE = (1066.0, 1146.0, -269.5, -264.6, 9.0)   # 腹鰭下玻纖片：x 範圍、z 範圍、半寬

CG = (93.3, 0.0, 40.0)             # 手冊第 5 頁重心（弦長 28.2%）；高度取在推力線與翼弦之間

MATERIALS = [
    ('epo_white', [0.80, 0.80, 0.80, 1.0], 0.0, 0.85),
    ('black', [0.012, 0.012, 0.012, 1.0], 0.0, 0.55),
    ('motor', [0.02, 0.02, 0.022, 1.0], 0.7, 0.35),
    ('prop_carbon', [0.028, 0.028, 0.028, 1.0], 0.1, 0.35),
    ('prop_grey', [0.40, 0.40, 0.41, 1.0], 0.0, 0.5),         # 尾推槳淺灰（渲染 #A8A8A8）
]


def fetch(dst: pathlib.Path) -> None:
    gk.fetch_blobs(SOURCES, GITEE_REPO, dst)


def interp(table, x):
    a = np.asarray(table, float)
    return float(np.interp(x, a[:, 0], a[:, 1]))


def wing_le_x(y):
    return 0.0 if y <= WING_TIP_LE[0][0] else interp(WING_TIP_LE, y)


def wing_te_x(y):
    a, b, k = WING_TE
    return min(a, b - k * y)


def shell_rim(V, F, x):
    """開口剖面（頂部開的 U）兩側最高點＝開口邊緣 (|y|, z)；兩側取平均（左右對稱）。"""
    seg = gk.section_segments(V, F, (x, 0, 0), (1, 0, 0))
    P = seg.reshape(-1, 3)
    L, R = P[P[:, 1] < -15], P[P[:, 1] > 15]
    pl, pr = L[np.argmax(L[:, 2])], R[np.argmax(R[:, 2])]
    return (abs(pl[1]) + abs(pr[1])) / 2, (pl[2] + pr[2]) / 2


def smoothed(a, k):
    """移動平均（兩端用邊值補齊，長度不變）。"""
    a = np.asarray(a, float)
    if k <= 1 or len(a) < k:
        return a
    p = np.r_[np.full(k // 2, a[0]), a, np.full(k // 2, a[-1])]
    return np.convolve(p, np.ones(k) / k, mode='valid')


def polar_envelope(P, center, n=180):
    """剖面點雲（y, z）→ 從 center 看每個方位的最遠點（外表面）；空的方位用相鄰的內插（環狀）。"""
    c = np.asarray(center, float)
    d = P - c
    ang = np.arctan2(d[:, 1], d[:, 0])
    r = np.linalg.norm(d, axis=1)
    b = np.floor((ang + math.pi) / (2 * math.pi) * n).astype(int) % n
    rmax = np.full(n, np.nan)
    np.fmax.at(rmax, b, r)
    t = -math.pi + (np.arange(n) + 0.5) * 2 * math.pi / n
    ok = ~np.isnan(rmax)
    if not ok.any():
        raise RuntimeError('polar_envelope：沒有任何剖面點')
    tt = np.r_[t[ok] - 2 * math.pi, t[ok], t[ok] + 2 * math.pi]
    rr = np.interp(t, tt, np.r_[rmax[ok], rmax[ok], rmax[ok]])
    return c + np.c_[rr * np.cos(t), rr * np.sin(t)]


def lid_section(x, w, z_rim, z_top, n=48, thick=3.0, expo=2.6):
    """艙蓋剖面：從一側開口邊緣 (−w, z_rim) 拱到頂 (0, z_top) 再到另一側，厚 thick（封閉的薄殼剖面）。"""
    t = np.linspace(-1, 1, n)
    y = w * t
    z = z_rim + (z_top - z_rim) * (1 - np.abs(t) ** expo) ** (1 / expo)
    outer = np.c_[y, z]
    inner = np.c_[y[::-1] * 0.985, z[::-1] - thick]
    loop = np.vstack([outer, inner])
    return np.c_[np.full(len(loop), x), loop]


# ═════════════════════════════════════════════════════════════════════════════
#  組裝
# ═════════════════════════════════════════════════════════════════════════════
def assemble(src: pathlib.Path, cache: pathlib.Path | None = None):
    cache = cache or (src / '_mesh')
    parts: list[Part] = []

    def step(key):
        return gk.load_step(src, key, SOURCES[key][1], cache, STEP_TOL)

    def add(name, mat, VF, smooth):
        parts.append(Part(name, mat, VF[0], VF[1], smooth))

    def add_lr(name, mat, VF, smooth):
        add(name + '_R', mat, VF, smooth)
        add(name + '_L', mat, mirror_y(*VF), smooth)

    # ── STEP：機頭、載荷艙、機身尾部、腹鰭（都在位置上）、航燈外殼（x +89，右翼；左翼鏡像）─────────────
    nose_V, nose_F = merged([(V, F) for V, F, _ in step('nose')])
    bay_V, bay_F = merged([(V, F) for V, F, _ in step('payload_bay')])
    rear_V, rear_F = merged([(V, F) for V, F, _ in step('fuselage_rear')])
    fin_V, fin_F = merged([(V, F) for V, F, _ in step('tail')])
    add('nose', 'epo_white', (nose_V, nose_F), False)
    add('payload_bay', 'epo_white', (bay_V, bay_F), False)
    add('fuselage_rear', 'epo_white', (rear_V, rear_F), False)
    add('ventral_fin', 'epo_white', (fin_V, fin_F), False)
    nl_V, nl_F = merged([(V, F) for V, F, _ in step('nav_light')])
    add_lr('nav_light', 'epo_white', (nl_V + (NAV_LIGHT_SHIFT_X, 0, 0), nl_F), False)     # 官方渲染是白色

    # ── 機頭蓋（CAD 前端在 -428 開口，示意圖蓋深 22 mm、蓋尖 (-450, 0, -20)）─────────────────────
    # 底圈取外皮（剖面的凸包）：那一站的 polygonize 只會給相機窗孔那一圈（外皮留一圈 8 mm 台階，審查 2026-10-23）
    x0, x1, (cy, cz) = NOSE_CAP
    nseg = gk.section_segments(nose_V, nose_F, (x0 + 0.5, 0, 0), (1, 0, 0)).reshape(-1, 3)[:, 1:]
    c0 = np.array([0.0, (nseg[:, 1].min() + nseg[:, 1].max()) / 2])
    base = resample_loop_polar(gk.outer_hull(nseg), 72, c0)
    secs = []
    for t in np.r_[np.linspace(0, 0.9, 7), 0.96, 0.99]:
        c = c0 + (np.array([cy, cz]) - c0) * t
        loop = c + (base - c0) * (1 - t ** NOSE_CAP_EXPO) ** (1 / NOSE_CAP_EXPO)
        secs.append(np.c_[np.full(len(loop), x0 + 0.5 + (x1 - x0 - 0.5) * t), loop])
    add('nose_cap', 'epo_white', loft(secs), True)

    # ── 機頭／載荷艙之間的塑料接頭框（黑）：機頭後環的外形，前後一樣 ──────────────────────────────
    # 外形取機頭後端剖面的凸包（外皮那圈在這一站不閉合，polygonize 只給內圈 ⇒ 以前縮進 15–36 mm、側面看得穿）
    hx0, hx1 = HEAD_CONNECTOR_X
    hseg = gk.section_segments(nose_V, nose_F, (hx0 - 0.6, 0, 0), (1, 0, 0)).reshape(-1, 3)[:, 1:]
    hc = np.array([0.0, (hseg[:, 1].min() + hseg[:, 1].max()) / 2])
    hring = resample_loop_polar(gk.outer_hull(hseg), 96, hc)
    hring = hc + (hring - hc) * 0.997                          # 機頭／載荷艙外皮在的地方略收在裡面
    add('head_connector', 'black', loft([np.c_[np.full(96, hx0 - 1.0), hring], np.c_[np.full(96, hx1 + 1.0), hring]]), True)

    # ── 座艙蓋、電池艙蓋（開口兩側邊緣 → 示意圖頂線；邊緣沿 x 平滑）──────────────────────────────
    def hatch(V, F, xa, xb, name, rim_x=None):
        """rim_x：只在這個 x 範圍量開口邊緣（蓋子比開口長的那一段沿用端點的邊緣）。"""
        xs = np.linspace(xa, xb, int((xb - xa) / 8) + 1)
        rim = np.array([shell_rim(V, F, x) for x in (np.clip(xs, *rim_x) if rim_x else xs)])
        W, Z = smoothed(rim[:, 0], RIM_SMOOTH), smoothed(rim[:, 1], RIM_SMOOTH)
        secs = [lid_section(x, w + 1.5, zr - 1.0, max(interp(HATCH_TOP, x), zr + 4.0) + 0.5)
                for x, w, zr in zip(xs, W, Z)]
        add(name, 'epo_white', loft(secs), True)
        return xs, W, Z
    hatch(nose_V, nose_F, *CANOPY_X, 'canopy', CANOPY_RIM_X)
    _, bW, bZ = hatch(bay_V, bay_F, *BATTERY_HATCH_X, 'battery_hatch')
    fa, fb, fw, fz = FRONT_HATCH
    add('front_hatch', 'epo_white',
        loft([lid_section(x, fw, fz, interp(HATCH_TOP, x) + 0.5) for x in np.linspace(fa, fb, 6)]), True)

    # ── 機身中段：從載荷艙尾端的實際外形（含翼根整流、頂部補艙蓋）漸變到機身尾部前端剖面的形狀
    #    （正規化）× 示意圖的半寬／頂／底 ───────────────────────────────────────────────────────
    shape = resample_loop_polar(section_outline(rear_V, rear_F, (REAR_SHAPE_X, 0, 0), (1, 0, 0), [1, 2],
                                                expect_full=True), 96)
    sy0, sz0, sz1 = np.abs(shape[:, 0]).max(), shape[:, 1].min(), shape[:, 1].max()
    unit = np.c_[shape[:, 0] / sy0, (shape[:, 1] - (sz0 + sz1) / 2) / ((sz1 - sz0) / 2)]
    mf = np.asarray(MID_FUSE, float)

    def body(x):
        w, zt, zb = (np.interp(x, mf[:, 0], mf[:, k]) for k in (1, 2, 3))
        return np.c_[unit[:, 0] * w, (zt + zb) / 2 + unit[:, 1] * (zt - zb) / 2]
    xb0 = MID_FUSE[0][0]
    # 包絡前先把線段加密：只拿線段端點，空的方位會被內部結構的頂點搶走 ⇒ 兩側各刻出 4 mm 的凹槽（審查 2026-10-23）
    seg = gk.densify_segments(gk.section_segments(bay_V, bay_F, (xb0 - 1.0, 0, 0), (1, 0, 0))[:, :, 1:])
    lid = gk.densify(lid_section(xb0, bW[-1] + 1.5, bZ[-1] - 1.0, interp(HATCH_TOP, xb0) + 0.5)[:, 1:])
    centre = (0.0, (mf[0, 2] + mf[0, 3]) / 2)
    # 兩種剖面都是 resample_loop_polar 取樣（近似同一組方位角、從 -y 方向起；形狀中心不同 ⇒ 偏差 ≤1.6°，
    # 小於半個取樣間隔）⇒ 逐點對應，可以直接混
    bay_loop = resample_loop_polar(polar_envelope(np.vstack([seg, lid]), centre), 96, centre)
    secs = [np.c_[np.full(96, xb0 - 1.0), bay_loop]]
    for x, wgt in ((362.0, 0.45), (380.0, 0.8)):                   # 翼根整流在 50 mm 內收回機身（示意圖 164→112）
        secs.append(np.c_[np.full(96, x), bay_loop * (1 - wgt) + body(x) * wgt])
    for x, *_ in MID_FUSE[1:]:
        secs.append(np.c_[np.full(96, x), body(x)])
    add('mid_fuselage', 'epo_white', loft(secs), True)

    # ── 主翼（翼型＝載荷艙接合面外形以弦中點縮到 331.5）＋翼根黑色束帶 ──────────────────────────────
    NP = 120
    face = resample_profile(section_outline(bay_V, bay_F, (0, WING_ROOT_SECTION_Y, 0), (0, 1, 0), [0, 2],
                                            expect_full=True), NP)
    mid = (face[np.argmin(face[:, 0])] + face[np.argmax(face[:, 0])]) / 2
    face_chord = face[:, 0].max() - face[:, 0].min()
    root = mid + (face - mid) * (WING_CHORD / face_chord)
    le = root[np.argmin(root[:, 0])].copy()

    def wing_sec(y):
        c = wing_te_x(y) - wing_le_x(y)
        p = le + (root - le) * (c / WING_CHORD)
        p[:, 0] += wing_le_x(y) - le[0]
        y0, dz = WING_TIP_DROOP                                    # 翼尖下垂：前緣側全量、後緣側 60%
        k = max(0.0, (y - y0) / (WING_TIP_LE[-1][0] - y0)) ** 1.5
        p[:, 1] -= dz * k * (1 - 0.4 * (p[:, 0] - p[:, 0].min()) / c)
        return np.c_[p[:, 0], np.full(len(p), y), p[:, 1]]
    ys = sorted({WING_ROOT_Y, 400.0, 580.0, 900.0, 1200.0, *[y for y, _ in WING_TIP_LE]})
    add_lr('wing', 'epo_white', loft([wing_sec(y) for y in ys]), True)
    sa, sb, st = WING_ROOT_STRAP
    strap = offset_profile(root, st)
    add_lr('wing_root_strap', 'black', loft([np.c_[strap[:, 0], np.full(NP, y), strap[:, 1]] for y in (sa, sb)]), True)

    # ── 水平尾（NACA 0010、+0.44°、翼尖前緣圓角）＋根部黑色套座 ─────────────────────────────────────
    af = resample_profile(naca4(HTAIL_AIRFOIL), 64)
    inc = math.atan2(HTAIL_LE_Z - HTAIL_TE_Z, HTAIL_TE_X - HTAIL_LE[0][1])

    def htail_sec(y, grow=0.0, thick=1.0):
        le_x = interp(HTAIL_LE, y) - grow
        c = HTAIL_TE_X + grow - le_x
        u, v = af[:, 0] * c, af[:, 1] * c * thick
        x = le_x + u * math.cos(inc) + v * math.sin(inc)
        z = HTAIL_LE_Z - u * math.sin(inc) + v * math.cos(inc)
        return np.c_[x, np.full(len(x), y), z]
    hys = sorted({HTAIL_ROOT_Y, 150.0, 250.0, 350.0, *[y for y, _ in HTAIL_LE[8:]]})
    add_lr('htail', 'epo_white', loft([htail_sec(y) for y in hys]), True)
    ha, hb, _ = HTAIL_ROOT_STRAP
    add_lr('htail_socket', 'black', loft([htail_sec(ha, 2.0, 1.12), htail_sec(hb, 2.0, 1.12)]), True)

    # ── 垂起機臂、電機座、固定座、升力馬達、升力槳（Quad-X：右前／左後 CCW、左前／右後 CW）──────────────
    for side in (-1, 1):
        y = side * BOOM_Y
        add(f'boom_{side}', 'black', cylinder((BOOM_END_X[0] + 40, y, BOOM_Z), (BOOM_END_X[1] - 40, y, BOOM_Z), BOOM_D / 2, 24), True)
        for (xa, xb) in BOOM_BLOCKS_X:
            add(f'boom_block_{side}_{xa}', 'black', box((xa, y - 20, BOOM_Z - 18), (xb, y + 20, BOOM_Z + 16)), True)
        for x_h, x_end in zip(LIFT_HUB_X, BOOM_END_X):
            xc = x_end + math.copysign(MOUNT_L / 2, x_h - x_end)
            add(f'mount_head_{side}_{x_h}', 'black',
                rounded_box((xc, y, sum(MOUNT_Z) / 2), (MOUNT_L, MOUNT_W, MOUNT_Z[1] - MOUNT_Z[0]), 9.0), True)
            base = np.array([x_h, y, MOUNT_Z[1]])
            add(f'motor_{side}_{x_h}', 'motor', cylinder(base, base + (0, 0, LIFT_MOTOR_H), LIFT_MOTOR_D / 2, 32), True)
            add(f'motor_cap_{side}_{x_h}', 'black',
                cylinder(base + (0, 0, LIFT_MOTOR_H), base + (0, 0, LIFT_MOTOR_H + 4), LIFT_MOTOR_D * 0.3, 20), True)
            front = x_h < 0
            ccw = (side > 0) == front
            radial = (math.cos(math.radians(45)), side * math.sin(math.radians(45)) * (1 if front else -1), 0)
            add(f'prop_{side}_{x_h}', 'prop_carbon',
                propeller(base + (0, 0, LIFT_MOTOR_H + 8), (0, 0, 1), radial, LIFT_PROP_D, handed=1 if ccw else -1,
                          chord_root=44.0, chord_tip=17.0, thick=4.5), True)

    # ── 尾推：座板、X6525、21 吋淺灰槳（停成水平）、黑色整流罩 ────────────────────────────────────
    ax = np.array([1.0, 0.0, 0.0])
    f0 = np.array([PUSHER_FACE_X, 0.0, PUSHER_Z])
    add('pusher_plate', 'black', box(f0 + (0, -37.1, -37.1), f0 + (4.0, 37.1, 37.1)), True)
    add('pusher_motor', 'motor', cylinder(f0 + ax * 4, f0 + ax * (4 + PUSHER_MOTOR_L), PUSHER_MOTOR_D / 2, 32), True)
    hub = np.array([PUSHER_PROP_X, 0.0, PUSHER_Z])
    add('pusher_prop', 'prop_grey', propeller(hub, ax, (0, 1, 0), PUSHER_PROP_D, handed=1, chord_root=42.0,
                                             chord_tip=16.0, thick=4.5, hub_d=30.0), True)
    add('spinner', 'black', cylinder(hub - ax * 6, (PUSHER_SPINNER_TIP_X, 0, PUSHER_Z), 19.0, 24, r1=0.0), True)

    # ── 起落架（兩支扁片腳，外撇 36.5°）、腳座、腹鰭下玻纖片 ────────────────────────────────────────
    for side in (-1, 1):
        r0 = np.array([GEAR_ROOT[0], side * GEAR_ROOT[1], GEAR_ROOT[2]])
        r1 = np.array([GEAR_FOOT[0], side * GEAR_FOOT[1], GEAR_FOOT[2]])
        d = r1 - r0
        nrm = np.cross(d, [1.0, 0, 0]); nrm /= np.linalg.norm(nrm)          # 片的厚度方向（在 y-z 平面內）
        secs = []
        for t, w in ((0.0, GEAR_W[0]), (1.0, GEAR_W[1])):
            c = r0 + d * t
            secs.append(np.array([c + (-w / 2, 0, 0) + nrm * GEAR_T / 2, c + (w / 2, 0, 0) + nrm * GEAR_T / 2,
                                  c + (w / 2, 0, 0) - nrm * GEAR_T / 2, c + (-w / 2, 0, 0) - nrm * GEAR_T / 2]))
        add(f'gear_leg_{side}', 'black', loft(secs), True)
        add(f'gear_mount_{side}', 'black',
            box((GEAR_ROOT[0] - 18, side * 100 - 14, -104), (GEAR_ROOT[0] + 18, side * 100 + 14, -72)), True)
    xa, xb, za, zb, hw = FIN_BRACE
    add('fin_brace', 'black', box((xa, -hw, za), (xb, hw, zb + 0.6)), True)

    info = dict(wing_face_chord=round(float(face_chord), 2), wing_root_le_z=round(float(le[1]), 2),
                wing_root_te_z=round(float(root[np.argmax(root[:, 0])][1]), 2))
    return parts, info


def build(src: pathlib.Path, out: pathlib.Path, cache: pathlib.Path | None = None):
    parts, info = assemble(src, cache)
    groups = gk.build_groups(parts, MATERIALS, CG)
    extras = {
        'source': f'https://gitee.com/{GITEE_REPO}/tree/{GITEE_COMMIT}/{GITEE_DIR}',
        'source_note': 'STEP parts from MFE (no license file; repo describes itself as open-source CAD drawings); '
                       'wing, horizontal tail, middle fuselage, hatches, nose cap, booms, motors, props, landing gear '
                       'are parametric from the MFE Pioneer sticker illustration, the Pioneer PNP manual V1.0 and the '
                       'motor-mount drawings',
        'frame': f'metres; +X nose, +Y up, +Z right wing; origin at CG ({CG[0]} mm aft of wing root LE)',
        'step_tolerance': list(STEP_TOL), 'step_sha1': {k: v[1] for k, v in SOURCES.items()},
        'tools': gk.tool_versions(),
    }
    data = gk.write_glb(groups, out, extras, MATERIALS, name='Pioneer')
    return data, gk.report_of(data, groups, out, len(parts), **info)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    f = sub.add_parser('fetch')
    f.add_argument('--dst', type=pathlib.Path, default=DEFAULT_STEP_DIR)
    b = sub.add_parser('build')
    b.add_argument('--src', type=pathlib.Path, default=DEFAULT_STEP_DIR)
    b.add_argument('--out', type=pathlib.Path, default=DEFAULT_OUT)
    b.add_argument('--cache', type=pathlib.Path, default=None)
    b.add_argument('--report', type=pathlib.Path)
    b.add_argument('--check', action='store_true', help='另開行程、全新快取再建一次，逐位元組比對（決定性）')
    a = ap.parse_args(argv)
    if a.cmd == 'fetch':
        fetch(a.dst)
        return 0
    data, rep = build(a.src, a.out, a.cache)
    if a.check:
        rep['deterministic_fresh_tessellation'] = gk.check_rebuild_identical(
            pathlib.Path(__file__).resolve(), data, ['--src', str(a.src)])
        if not rep['deterministic_fresh_tessellation']:
            print('❌ 全新快取重建出來不一樣', file=sys.stderr)
    txt = json.dumps(rep, ensure_ascii=False, indent=1)
    print(txt)
    if a.report:
        a.report.write_text(txt, encoding='utf-8')
    return 0 if rep.get('deterministic_fresh_tessellation', True) else 1


if __name__ == '__main__':
    sys.exit(main())
