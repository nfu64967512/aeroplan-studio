# -*- coding: utf-8 -*-
"""build_hero_glb.py — MFE Hero（逆行者）4+1 VTOL 的 3D 模型 → ui/resources/hero.glb（CesiumJS 用，實際尺寸）。

來源：原廠在 gitee 開源的 Hero CAD（makeflyeasy/MFE_ArduPlane，Hero/飞机图纸Aircraft drawings，commit 50d1034d）。
  那份資料夾只有機身與塑料件的 STEP：航測／監測機頭、載荷艙、機身尾部（含腹鰭、V 尾座、尾推座）、兩個艙蓋、
  翼根塑料件、翼尖、分段塑料件、副翼連接件。**沒有**主翼 EPO 段、V 尾、垂起機臂、馬達、槳、起落架。
  ⇒ 有 STEP 的照 STEP（逐面三角化、只平移不改形），沒有的依原廠尺寸參數化補上：
    * 主翼：翼根塑料件在 |y|=117.5 的剖面＝翼型（t/c 11.8%、安裝角 2.97°），翼根段等弦 258.7 到 |y|=428，
      外段由 255.2 線性收到翼尖件內端（|y|=940.33，直接取翼尖件的剖面）；外段內端沿分段塑料件的斜面放樣 ⇒ 不留縫。
    * 機臂／馬達／槳、V 尾、尾推、起落架：手冊第 5 頁俯視圖（2.389 mm/px，翼展／槳盤／機身長交叉驗證 ±0.8%）、
      ArduPilot 論壇原廠三視圖（2.248 mm/px）、起落架尺寸圖、手冊規格表；V 尾上反角 35° 由機身尾部 STEP 的
      V 尾座面量出（35.0°），與三視圖前視（35.1°）一致。
  量測與來源細節、審查結果：docs/實作_2026-10-23_Hero3D模型.md。共用工具：tools/glb_kit.py。

座標：建模用 CAD 座標（mm；x 往後、y 往右翼、z 往上；原點＝翼根前緣），匯出時換成 glTF：
  公尺、+X＝機鼻、+Y＝上、+Z＝右翼（與 barracuda500.glb 同慣例 ⇒ main.html 的校正偏移 0/0/0、scale 1），
  原點＝重心（手冊第 5 頁：翼根塑料件下凸起處，弦長 28.6% ≈ 前緣後 74 mm）。
輸出：一個 mesh、每種材質一個 primitive（5 個，< glb_merge.MIN_PRIMITIVES_TO_MERGE ⇒ 不需要合併快取）、
  只有 POSITION／NORMAL＋baseColorFactor，沒有貼圖／擴充。

用法（只有建模時需要 cascadio、trimesh、shapely、numpy；app 執行期不需要）：
  python -I tools/build_hero_glb.py fetch [--dst runs/artifacts/HERO_MODEL/step]
  python -I tools/build_hero_glb.py build [--src runs/artifacts/HERO_MODEL/step] [--out ui/resources/hero.glb]
                                          [--nose survey|monitor] [--report 報告.json] [--check] [--cache 目錄]
  --check：另開一個行程、用全新的空快取（重新三角化 STEP）再建一次，逐位元組比對。
  ⚠ OpenCASCADE 讀不了中文路徑 ⇒ fetch 存成英文檔名（SOURCES 的鍵）。
  ⚠ 原廠資料夾沒有授權檔（repo 描述「飞机模型的CAD图纸开源」）⇒ STEP 不進 git，hero.glb 跟其他 GLB 一樣不追蹤。
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
from glb_kit import (Part, box, cylinder, git_blob_sha1, keep_faces, loft, merged, mirror_y,   # noqa: E402,F401
                     naca4, offset_profile, propeller, resample_loop_polar, resample_profile, rounded_box,
                     section_outline)

ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_STEP_DIR = ROOT / 'runs' / 'artifacts' / 'HERO_MODEL' / 'step'
DEFAULT_OUT = ROOT / 'ui' / 'resources' / 'hero.glb'

GITEE_REPO = 'makeflyeasy/MFE_ArduPlane'
GITEE_COMMIT = '50d1034d86c9ef1b88a0cb3320a48270f765878d'     # Hero 資料夾最後一次變更（2026-01-05）
GITEE_DIR = 'Hero/飞机图纸Aircraft drawings'
# 英文檔名 → (原廠檔名, git blob sha1)。fetch 走 blob API（大檔 raw 連結要登入），下載後驗 sha1。
SOURCES = {
    'aileron_conn_R': ('逆行者右副翼连接件.stp', '6572152614fda5f2b74adfb803c6ed3a42650b6b'),
    'aileron_conn_L': ('逆行者左副翼连接件.stp', '802a92d67016010451f90b9532fda7d64b4cbffe'),
    'wing_root_plastic_L': ('逆行者左机翼塑料件(Hero left wing plastic parts).stp', '27f36bde72ab0d3c20277ee319206a97e1cf02e1'),
    'fuselage_rear': ('逆行者机身尾部（Hero fuselage rear）.stp', 'f6feaf419a958f1952fddc7a9ad19895def43cc7'),
    'fuselage_payload_bay': ('逆行者机身载荷舱（Hero fuselage load compartment）.stp', 'df4ccd5d22b963c93c883f7976f0102915e5a638'),
    'nose_monitor': ('逆行者监测机头(Hero monitors the nose).stp', '4446f092dd4a9b7ce5aa12de2e37bba1a0db1b6d'),
    'wingtip': ('逆行者翼尖(Hero wingtip).stp', '529566ab12b3ba46e36719376488d214c896d71d'),
    'wingtip_plastic': ('逆行者翼尖塑料件(Hero wingtip plastic parts).stp', '23f6242b3d630b5f432106a18b781d4d7c350448'),
    'nose_survey': ('逆行者航测机头(Hero aerial survey nose).stp', '1bf2f6a6c1170a672f89b0c9c0ba88a0a233bea0'),
    'payload_hatch': ('逆行者载荷舱盖（Hero load hatch cover）.stp', '318dd929711af1dc806b61315f7f8c77cc6b6b35'),
    'fc_hatch': ('逆行者飞控舱盖（Hero flight control hatch）.stp', '1e085bf956d79b7af77722f76777bf78dc346af3'),
}

# STEP 三角化容差（mm／rad）：0.6／0.5 ⇒ CAD 部分約 13 萬面；0.1／0.3 是 33 萬面，遠看分不出來
STEP_TOL = (0.6, 0.5)

# ── 幾何常數（mm，CAD 座標）──────────────────────────────────────────────────
# 原廠排版把部分零件沿 y 排開（同一份 CAD 座標、只平移）：要「加」到頂點上的平移
SHIFT_NOSE_SURVEY = 220.0          # 接環與監測機頭的接環逐點相同
SHIFT_FUSELAGE_REAR = 400.0        # 移回後左右鏡像中位差 0.13 mm、與載荷艙在 x 159..183 外形重合
SHIFT_RIB_B = 27.0                 # 翼根第二片塑料件：背靠背接在第一片後面（外壁 y 131..142，下方小舌片伸到 149.4）
SHIFT_RIB_R2 = 770.0               # 分段塑料件的右翼那片
SHIFT_CAP_W2 = 1830.66             # 右翼尖（左右兩片並排放）
BAY_CUT_X = 159.5                  # 載荷艙與機身尾部在 x 159..183 重疊（同一個外表面）⇒ 載荷艙切到這裡，免得 z-fighting

WING_RIB_A_SECTION_Y = -117.5      # 翼根塑料件 A：這個剖面就是翼型（翼根弦 258.7、安裝角 2.97°）
WING_STUB_Y = 117.0                # 機身側（|y| 86.8）到翼根件 A 邊緣（117）的肩部：翼型穿過機身補滿
ROOT_BAND_Y = 137.0                # 翼根黑色束帶外緣：照片 ≈19 mm（翼根件 A 117..131 ＋ B 131..137）；B 超出的部分不畫
WING_ROOT_PANEL = (ROOT_BAND_Y, 428.0)   # 翼根 EPO 段 → 分段塑料件內側面（後緣 428..430.5 是副翼端的真縫）
WING_TIP_PANEL_Y0 = 440.0          # 分段塑料件外側面在副翼前（x ≳ 112）的位置；更前面是斜面（|y| 433.5 → 440），照 CAD 量
WING_TIP_PANEL_CHORD0 = 255.2      # 外段內端弦長（＝副翼連接件後緣 x）
WING_TIP_PANEL_LE_Z0 = 15.30       # 分段塑料件的前緣 z（比翼根高 0.31 mm）
WING_CAP_SECTION_Y = -940.5        # 翼尖件內端（平面在 -940.33）：外段直接接這個剖面
JOINT_STRAP = (424.0, 444.0, 0.5)  # 分段處黑色束帶（照片 13–24 mm，取 20）：|y| 範圍、比翼面外推的厚度

BOOM_Y, BOOM_Z, BOOM_D = 347.5, -7.2, 20.0          # 碳管 Ø20：管頂 2.8 ＝ 鈍後緣上表面（2.91）以下；前面的空隙由夾具填
LIFT_HUB_X = (-260.7, 426.3)                         # 手冊俯視圖：前後馬達軸心（相距 687）
MOUNT_W, MOUNT_L, MOUNT_Z = 40.0, 67.0, (-16.0, 8.6)  # 馬達座頭（三視圖前視 z -16..+8.6）
MOUNT_SLEEVE_D, MOUNT_SLEEVE_L = 26.0, 61.6          # 夾管段（從軸心往機臂內側）
WING_CLAMP_X, WING_CLAMP_TOP = (41.0, 172.0), 8.0    # 機臂夾在主翼下、對齊兩根翼管；夾具頂埋進翼裡（下表面 ≤ 6.8）
LIFT_MOTOR_D, LIFT_MOTOR_H = 49.8, 27.0              # M4112
LIFT_PROP_D = 381.0                                   # 15 吋（MFE 1555 碳纖槳）

VTAIL_DIHEDRAL = 35.0              # 機身尾部 V 尾座面 35.0°；三視圖前視 35.1°
VTAIL_AXIS_Z = -10.2               # 兩片延長線交會在 y=0 的高度（座面形心 (677.95, ±44.47, 20.95) 推回）
VTAIL_Y = (38.0, 326.0)            # 投影 |y|：根部插進整流座內、翼尖外緣
VTAIL_LE = (591.4, 17.27)          # 投影：|y|=49 處前緣 x、前緣後掠角
VTAIL_TE = (789.5, 4.78)           # 投影：|y|=49 處後緣 x、後緣後掠角
VTAIL_TIP_ROUND = 22.0             # 翼尖前緣圓角
VTAIL_AIRFOIL = '0009'             # 座面厚 18 mm／弦 194 ≈ 9%

PUSHER_FACE = (794.96, 0.0, 0.03)                    # 機身尾端方框（60×60）中心
PUSHER_AXIS = (0.99917, 0.0, 0.0407)                 # 往後、後端上翹 2.33°（三個後向面法向一致）
PUSHER_MOTOR_D, PUSHER_MOTOR_L = 42.5, 45.0          # X3520
PUSHER_PROP_D = 381.0                                 # APC 15×10（灰）
PUSHER_HUB_AT, PUSHER_SPINNER_TIP_AT = 57.0, 86.0    # 沿軸距尾端面：槳盤（x≈852）、整流罩尖（x≈880.7，手冊俯視圖）

GEAR_X, GEAR_TRACK, GEAR_H = -81.0, 313.6, 110.0     # 起落架尺寸圖＋手冊俯視圖
GEAR_FLAT, GEAR_W_TOP, GEAR_W_FOOT, GEAR_T = 144.0, 25.0, 15.0, 4.5
GPS_X, GPS_D, GPS_H = 360.0, 65.0, 12.0
GIMBAL_C, GIMBAL_D = (-285.0, 0.0, -92.0), 70.0      # 監測機頭才有
NOSE_BAND_X = (-168.5, -155.0, -147.8)               # 機頭接環後的過渡：黑色到 -155（照片束帶 ≈15 mm）、之後白色接到載荷艙

CG = (74.0, 0.0, 10.0)             # 模型原點（Cesium 繞這點轉）：弦長 28.6%，高度取在翼弦平面與推力線之間

# 材質：baseColorFactor 是線性色彩空間（sRGB #EEEEEE ≈ 0.85）
MATERIALS = [
    ('epo_white', [0.80, 0.80, 0.80, 1.0], 0.0, 0.85),     # 機身／主翼／V 尾（白色 EPO）
    ('black', [0.012, 0.012, 0.012, 1.0], 0.0, 0.55),      # 塑料件、束帶、碳管、馬達座、起落架、GPS、整流罩
    ('motor', [0.02, 0.02, 0.022, 1.0], 0.7, 0.35),        # 馬達外殼（金屬）
    ('prop_carbon', [0.028, 0.028, 0.028, 1.0], 0.1, 0.35),  # 升力槳（碳纖，略亮）
    ('prop_grey', [0.25, 0.25, 0.26, 1.0], 0.0, 0.5),       # 尾推槳 APC（灰）
]


def to_gltf(V):
    return gk.to_gltf(V, CG)


def build_groups(parts):
    return gk.build_groups(parts, MATERIALS, CG)


def write_glb(groups, path, extras):
    return gk.write_glb(groups, path, extras, MATERIALS, name='Hero')


def fetch(dst: pathlib.Path) -> None:
    gk.fetch_blobs(SOURCES, GITEE_REPO, dst)


def bow_gear(x_c, top_z):
    """碳纖弓形起落架（尺寸圖）：平頂 144、兩腳略外凸往下 110 到腳距 313.6；葉寬 25→15、厚 4.5。"""
    half_flat, foot = GEAR_FLAT / 2, GEAR_TRACK / 2
    pts = [(y, 0.0, 0.0) for y in np.linspace(-half_flat, half_flat, 9)]      # (y, z, 腿長比例)
    legs = []
    for t in np.linspace(0, 1, 15)[1:]:
        y = half_flat + (foot - half_flat) * t + 5.0 * math.sin(math.pi * t)   # 略往外凸
        legs.append((y, -GEAR_H * (t ** 1.08), t))
    cl = np.array([(-y, z, t) for (y, z, t) in reversed(legs)] + pts + legs)
    secs = []
    for i in range(len(cl)):
        j0, j1 = max(i - 1, 0), min(i + 1, len(cl) - 1)
        tg = cl[j1, :2] - cl[j0, :2]
        tg /= np.linalg.norm(tg)
        nrm = np.array([-tg[1], tg[0]])
        w = GEAR_W_TOP + (GEAR_W_FOOT - GEAR_W_TOP) * cl[i, 2]
        q0, q1 = cl[i, :2] + nrm * GEAR_T / 2, cl[i, :2] - nrm * GEAR_T / 2
        secs.append(np.array([[x_c - w / 2, q0[0], top_z + q0[1]], [x_c + w / 2, q0[0], top_z + q0[1]],
                              [x_c + w / 2, q1[0], top_z + q1[1]], [x_c - w / 2, q1[0], top_z + q1[1]]]))
    return loft(secs)


def rib_outboard_face(rib_V, xs, default):
    """分段塑料件（左翼）外側面在每個 x 的 y（最外側的頂點）：前緣附近是斜的（|y| 433.5 → 440），
    副翼段沒有塑料件 ⇒ default。外段放樣的內端照這條線，才不會在前緣留下三角洞（2026-10-23 審查 G2）。"""
    out = []
    for x in xs:
        sel = np.abs(rib_V[:, 0] - x) < 3.0
        out.append(max(rib_V[sel, 1].min(), default) if sel.any() else default)
    return np.array(out)


# ═════════════════════════════════════════════════════════════════════════════
#  組裝
# ═════════════════════════════════════════════════════════════════════════════
def assemble(src: pathlib.Path, nose: str = 'survey', cache: pathlib.Path | None = None):
    cache = cache or (src / '_mesh')
    parts: list[Part] = []

    def step(key):
        return gk.load_step(src, key, SOURCES[key][1], cache, STEP_TOL)

    def add(name, mat, VF, smooth):
        parts.append(Part(name, mat, VF[0], VF[1], smooth))

    def add_lr(name, mat, VF, smooth):
        """左翼那份＋鏡像到右翼。"""
        add(name + '_L', mat, VF, smooth)
        add(name + '_R', mat, mirror_y(*VF), smooth)

    def shifted(V, dy):
        V = V.copy(); V[:, 1] += dy
        return V

    def cy_of(V, F):
        return gk.face_centroids(V, F)[:, 1]

    # ── 機頭：殼白、接環黑（原檔顏色：殼 255,255,0、環 18,18,18）──────────────────
    nose_bodies = step('nose_survey' if nose == 'survey' else 'nose_monitor')
    dy = SHIFT_NOSE_SURVEY if nose == 'survey' else 0.0
    for k, (V, F, col) in enumerate(nose_bodies):
        dark = max(col[:3]) < 64
        orange = col[:3] == (255, 81, 0)                     # 監測機頭的雲台底板
        add(f'nose_{k}', 'black' if (dark or orange) else 'epo_white', (shifted(V, dy), F), False)
    ring = [b for b in nose_bodies if max(b[2][:3]) < 64][0]
    ring_V = shifted(ring[0], dy)

    # ── 載荷艙（切掉和機身尾部重疊的 x>159.5）、機身尾部、兩個艙蓋 ───────────────
    bay_V, bay_F = merged([(V, F) for V, F, _ in step('fuselage_payload_bay')])
    cent_x = gk.face_centroids(bay_V, bay_F)[:, 0]
    add('payload_bay', 'epo_white', keep_faces(bay_V, bay_F, cent_x <= BAY_CUT_X), False)
    rear_V, rear_F = merged([(V, F) for V, F, _ in step('fuselage_rear')])
    rear_V = shifted(rear_V, SHIFT_FUSELAGE_REAR)
    add('fuselage_rear', 'epo_white', (rear_V, rear_F), False)
    for key in ('payload_hatch', 'fc_hatch'):
        add(key, 'epo_white', merged([(V, F) for V, F, _ in step(key)]), False)

    # 機頭接環後緣（x=-167）到載荷艙前緣（-148.8）的 18 mm：照片是一圈 ≈15 mm 的黑色束帶、接著白色機身。
    # 接環在 x=-175 剖（-167.5 只剖到半圈 ⇒ 環下方會有台階，審查 G4）。
    ring_loop = section_outline(ring_V, ring[1], (-175.0, 0, 0), (1, 0, 0), [1, 2], expect_full=True)
    bay_loop = section_outline(bay_V, bay_F, (-148.3, 0, 0), (1, 0, 0), [1, 2], expect_full=True)
    c = (ring_loop.mean(axis=0) + bay_loop.mean(axis=0)) / 2
    l0, l1 = resample_loop_polar(ring_loop, 96, c), resample_loop_polar(bay_loop, 96, c)
    xa, xm, xb = NOSE_BAND_X
    lm = l0 + (l1 - l0) * (xm - xa) / (xb - xa)
    add('nose_band', 'black', loft([np.c_[np.full(96, xa), l0], np.c_[np.full(96, xm), lm]]), True)
    add('nose_fairing', 'epo_white', loft([np.c_[np.full(96, xm), lm], np.c_[np.full(96, xb), l1]]), True)

    # ── 主翼 CAD 件：翼根塑料件 A／B、分段塑料件、副翼連接件、翼尖 ──────────────────
    (rV, rF, _), = step('wing_root_plastic_L')
    cy = cy_of(rV, rF)
    ribA = keep_faces(rV, rF, cy > -140.0)
    ribB = keep_faces(rV, rF, cy <= -140.0)
    ribB = (shifted(ribB[0], SHIFT_RIB_B), ribB[1])
    ribB = gk.slice_keep(*ribB, (0, -ROOT_BAND_Y, 0), (0, 1, 0))           # 束帶外緣以外不畫（被翼根段蓋住）
    add_lr('wing_root_rib_A', 'black', ribA, False)
    add_lr('wing_root_rib_B', 'black', ribB, False)
    wp = step('wingtip_plastic')
    jV, jF, _ = max(wp, key=lambda b: len(b[1]))                   # 另一個小幾何是內部舵機支架，不畫
    cy = cy_of(jV, jF)
    R1 = keep_faces(jV, jF, cy < -400.0)
    add('joint_rib_L', 'black', R1, False)
    R2 = keep_faces(jV, jF, (cy > -350.0))
    add('joint_rib_R', 'black', (shifted(R2[0], SHIFT_RIB_R2), R2[1]), False)
    for key in ('aileron_conn_L', 'aileron_conn_R'):
        V, F = merged([(V, F) for V, F, _ in step(key)])
        # 外段開始以外的部分在副翼泡棉裡面（同一個外表面 ⇒ z-fighting，審查 G5）
        add(key, 'black', keep_faces(V, F, np.abs(cy_of(V, F)) <= WING_TIP_PANEL_Y0 + 0.5), False)
    (cV, cF, _), = step('wingtip')
    cy = cy_of(cV, cF)
    W1 = keep_faces(cV, cF, cy < -915.33)
    W2 = keep_faces(cV, cF, cy >= -915.33)
    add('wingtip_L', 'epo_white', W1, False)
    add('wingtip_R', 'epo_white', (shifted(W2[0], SHIFT_CAP_W2), W2[1]), False)

    # ── 主翼 EPO 段（參數化放樣，兩端剖面取自 CAD）──────────────────────────────────
    NP = 120
    root = resample_profile(section_outline(ribA[0], ribA[1], (0, WING_RIB_A_SECTION_Y, 0), (0, 1, 0), [0, 2],
                                            expect_full=True), NP)
    cap_in = resample_profile(section_outline(W1[0], W1[1], (0, WING_CAP_SECTION_Y, 0), (0, 1, 0), [0, 2],
                                              expect_full=True), NP)
    le = root[np.argmin(root[:, 0])]
    chord_root = root[:, 0].max() - root[:, 0].min()
    tip0 = le + (root - le) * (WING_TIP_PANEL_CHORD0 / chord_root)
    tip0[:, 1] += WING_TIP_PANEL_LE_Z0 - le[1]

    def at_y(prof, y):
        y = np.broadcast_to(np.asarray(y, float), (len(prof),))
        return np.c_[prof[:, 0], y, prof[:, 1]]
    add('wing_stub', 'epo_white', loft([at_y(root, -WING_STUB_Y), at_y(root, WING_STUB_Y)]), True)
    y0, y1 = WING_ROOT_PANEL
    add_lr('wing_root_panel', 'epo_white', loft([at_y(root, -y0), at_y(root, -y1)]), True)
    y_in = rib_outboard_face(R1[0], tip0[:, 0], -WING_TIP_PANEL_Y0)          # 內端沿分段塑料件的斜面
    add_lr('wing_tip_panel', 'epo_white', loft([at_y(tip0, y_in), at_y(cap_in, WING_CAP_SECTION_Y)]), True)
    sa, sb, st = JOINT_STRAP
    add_lr('joint_strap', 'black', loft([at_y(offset_profile(root, st), -sa), at_y(offset_profile(tip0, st), -sb)]), True)

    # ── 垂起機臂、馬達座、夾具、升力馬達、升力槳 ─────────────────────────────────
    # 槳停在對角（看起來像照片）；Quad-X：右前／左後 CCW（正槳 handed=+1）、左前／右後 CW（反槳）
    for side in (-1, 1):
        y = side * BOOM_Y
        x_f, x_r = LIFT_HUB_X
        add(f'boom_{side}', 'black', cylinder((x_f - 12, y, BOOM_Z), (x_r + 12, y, BOOM_Z), BOOM_D / 2, 20), True)
        for x_h, inward in ((x_f, 1), (x_r, -1)):
            add(f'mount_head_{side}_{x_h}', 'black',
                rounded_box((x_h, y, sum(MOUNT_Z) / 2), (MOUNT_L, MOUNT_W, MOUNT_Z[1] - MOUNT_Z[0]), 8.0), True)
            add(f'mount_sleeve_{side}_{x_h}', 'black',
                cylinder((x_h, y, BOOM_Z), (x_h + inward * MOUNT_SLEEVE_L, y, BOOM_Z), MOUNT_SLEEVE_D / 2, 20), True)
            base = np.array([x_h, y, MOUNT_Z[1]])
            add(f'motor_{side}_{x_h}', 'motor', cylinder(base, base + (0, 0, LIFT_MOTOR_H), LIFT_MOTOR_D / 2, 28), True)
            add(f'motor_cap_{side}_{x_h}', 'black',
                cylinder(base + (0, 0, LIFT_MOTOR_H), base + (0, 0, LIFT_MOTOR_H + 3), LIFT_MOTOR_D * 0.3, 20), True)
            front = x_h < 0
            ccw = (side > 0) == front
            radial = (math.cos(math.radians(45)), side * math.sin(math.radians(45)) * (1 if front else -1), 0)
            add(f'prop_{side}_{x_h}', 'prop_carbon',
                propeller(base + (0, 0, LIFT_MOTOR_H + 6), (0, 0, 1), radial, LIFT_PROP_D, handed=1 if ccw else -1), True)
        for xc in WING_CLAMP_X:
            add(f'wing_clamp_{side}_{xc}', 'black', box((xc - 12, y - 15, BOOM_Z - 7), (xc + 12, y + 15, WING_CLAMP_TOP)), True)

    # ── V 尾（上反 35°、NACA 0009、翼尖前緣圓角）＋根部黑色接頭 ─────────────────────
    af = resample_profile(naca4(VTAIL_AIRFOIL), 64)
    d = math.radians(VTAIL_DIHEDRAL)
    ys = np.r_[np.linspace(VTAIL_Y[0], VTAIL_Y[1] - VTAIL_TIP_ROUND, 10),
               VTAIL_Y[1] - VTAIL_TIP_ROUND * (1 - np.sin(np.linspace(0, math.pi / 2, 7)[1:]))]

    def vtail_section(side, Y, grow=0.0, thick=1.0):
        """投影 |y|=Y 處的剖面；grow＝前後緣各加長（mm）、thick＝厚度倍數（根部套筒用）。"""
        e_n = np.array([0.0, -side * math.sin(d), math.cos(d)])      # 剖面厚度方向（垂直於面板、在 y-z 平面內）
        le_x = VTAIL_LE[0] + (Y - 49.0) * math.tan(math.radians(VTAIL_LE[1]))
        te_x = VTAIL_TE[0] + (Y - 49.0) * math.tan(math.radians(VTAIL_TE[1]))
        dtip = Y - (VTAIL_Y[1] - VTAIL_TIP_ROUND)
        if dtip > 0:
            le_x += VTAIL_TIP_ROUND - math.sqrt(max(VTAIL_TIP_ROUND ** 2 - dtip ** 2, 0.0))
        le_x -= grow
        ch = te_x + grow - le_x
        base = np.array([le_x, side * Y, VTAIL_AXIS_Z + Y * math.tan(d)])
        return base + np.outer(af[:, 0] * ch, [1.0, 0, 0]) + np.outer(af[:, 1] * ch * thick, e_n)

    for side in (-1, 1):
        add(f'vtail_{side}', 'epo_white', loft([vtail_section(side, Y) for Y in ys]), True)
        # 根部黑色接頭（照片）：機身尾部的 V 尾座面（194×18、前緣 |y| 50.6／後緣 37.4，斜的）比尾翼厚，
        # 不包起來會看到座面凹槽 ⇒ 從座面內側包到外側 56，再收回尾翼本身的剖面
        add(f'vtail_collar_{side}', 'black',
            loft([vtail_section(side, 32.0, 4.0, 1.5), vtail_section(side, 52.0, 4.0, 1.5),
                  vtail_section(side, 57.0, 0.5, 1.03)]), True)

    # ── 尾推：座板、X3520、APC 15×10（停成水平，照片）、黑色整流罩 ───────────────────
    ax = np.asarray(PUSHER_AXIS, float); ax /= np.linalg.norm(ax)
    f0 = np.asarray(PUSHER_FACE, float)
    R = gk.frame_from_axis(ax, (0, 1, 0))
    plate = rounded_box((0, 0, 3.0), (60.0, 60.0, 6.0), 8.0)
    add('pusher_plate', 'black', (plate[0] @ R.T + f0, plate[1]), True)
    add('pusher_motor', 'motor', cylinder(f0 + ax * 6, f0 + ax * (6 + PUSHER_MOTOR_L), PUSHER_MOTOR_D / 2, 28), True)
    add('pusher_prop', 'prop_grey', propeller(f0 + ax * PUSHER_HUB_AT, ax, (0, 1, 0), PUSHER_PROP_D, handed=1,
                                             chord_root=32.0, chord_tip=14.0, hub_d=24.0), True)
    add('spinner', 'black', cylinder(f0 + ax * (PUSHER_HUB_AT - 4), f0 + ax * PUSHER_SPINNER_TIP_AT, 17.0, 24, r1=0.0), True)

    # ── 起落架（頂面貼載荷艙底）、GPS 定位器（機身尾段頂）、監測機頭雲台 ─────────────
    P = gk.section_segments(bay_V, bay_F, (GEAR_X, 0, 0), (1, 0, 0)).reshape(-1, 3)    # 這一站有兩圈 ⇒ 直接看線段
    bottom_z = P[np.abs(P[:, 1]) < 40][:, 2].min()
    add('landing_gear', 'black', bow_gear(GEAR_X, bottom_z - GEAR_T / 2), True)
    P = gk.section_segments(rear_V, rear_F, (GPS_X, 0, 0), (1, 0, 0)).reshape(-1, 3)
    top_z = P[np.abs(P[:, 1]) < 5][:, 2].max()
    add('gps', 'black', cylinder((GPS_X, 0, top_z - 2), (GPS_X, 0, top_z + GPS_H), GPS_D / 2, 32), True)
    if nose == 'monitor':
        import trimesh
        ball = trimesh.creation.icosphere(subdivisions=3, radius=GIMBAL_D / 2)
        add('gimbal', 'black', (np.asarray(ball.vertices) + GIMBAL_C, np.asarray(ball.faces)), True)
        add('gimbal_bracket', 'black', box((-298, -12, -60), (-272, 12, -40)), True)

    info = dict(gear_top_z=round(float(bottom_z), 2), gps_base_z=round(float(top_z), 2),
                root_chord=round(float(chord_root), 2), nose=nose)
    return parts, info


def build(src: pathlib.Path, out: pathlib.Path, nose: str, cache: pathlib.Path | None = None):
    parts, info = assemble(src, nose, cache)
    groups = build_groups(parts)
    extras = {
        'source': f'https://gitee.com/{GITEE_REPO}/tree/{GITEE_COMMIT}/{GITEE_DIR}',
        'source_note': 'STEP parts from MFE (no license file; repo describes itself as open-source CAD drawings); '
                       'wing EPO panels, V-tail, booms, motors, props, landing gear are parametric from the '
                       'Hero PNP manual V4.0 and the MFE 3-view',
        'frame': 'metres; +X nose, +Y up, +Z right wing; origin at CG (74 mm aft of wing root LE)',
        'nose': nose, 'step_tolerance': list(STEP_TOL),
        'step_sha1': {k: v[1] for k, v in SOURCES.items()}, 'tools': gk.tool_versions(),
    }
    data = write_glb(groups, out, extras)
    return data, gk.report_of(data, groups, out, len(parts), **info)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    f = sub.add_parser('fetch')
    f.add_argument('--dst', type=pathlib.Path, default=DEFAULT_STEP_DIR)
    b = sub.add_parser('build')
    b.add_argument('--src', type=pathlib.Path, default=DEFAULT_STEP_DIR)
    b.add_argument('--out', type=pathlib.Path, default=DEFAULT_OUT)
    b.add_argument('--nose', choices=('survey', 'monitor'), default='survey')
    b.add_argument('--cache', type=pathlib.Path, default=None, help='STEP 三角化快取目錄（預設 <src>/_mesh）')
    b.add_argument('--report', type=pathlib.Path)
    b.add_argument('--check', action='store_true', help='另開行程、全新快取再建一次，逐位元組比對（決定性）')
    a = ap.parse_args(argv)
    if a.cmd == 'fetch':
        fetch(a.dst)
        return 0
    data, rep = build(a.src, a.out, a.nose, a.cache)
    if a.check:
        rep['deterministic_fresh_tessellation'] = gk.check_rebuild_identical(
            pathlib.Path(__file__).resolve(), data, ['--src', str(a.src), '--nose', a.nose])
        if not rep['deterministic_fresh_tessellation']:
            print('❌ 全新快取重建出來不一樣', file=sys.stderr)
    txt = json.dumps(rep, ensure_ascii=False, indent=1)
    print(txt)
    if a.report:
        a.report.write_text(txt, encoding='utf-8')
    return 0 if rep.get('deterministic_fresh_tessellation', True) else 1


if __name__ == '__main__':
    sys.exit(main())
