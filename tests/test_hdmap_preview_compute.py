"""survey/preview.py —— 航測建圖「預計成果」預覽（規劃預測，非實際成果）的值測試。

不建 MainWindow、不起 SITL、不開 COM 埠、不寫 data/。分兩組：

* **合成任務**（快）：自己組 N–S 割草機任務 → 真的 ``verify_preflight`` → :func:`build_hdmap_preview`。
  鎖住：分級上色逐格對（north-up、格邊 bounds）、block-MIN 縮圖不吃掉破洞、可成圖外框面積、GSD／正射／DSM／
  資料量／ODM 級距的數值、JSON 安全、World Terrain 不出山影、靜態圖只有 ASCII（沒有缺字警告）。
  另有 12 km² 全覆蓋的端到端計時（走 :func:`compute_hdmap_preview`，只把規劃器那一層換成合成任務）。
* **真規劃**（慢，約 20 s）：tests/test_hdmap_gui_coordinator 的 1.2 × 1.0 km、1 Pioneer + 2 HERO 建圖計畫。
  鎖住：預覽與上傳閘門餵給驗證器的輸入**逐項相同**（只有格距不同）、照片數與閘門一致、內部 ⊂ 可成圖範圍、
  挖掉一架相機 ⇒ 內部破洞出現在圖上、World Terrain 一律不出山影。
"""
from __future__ import annotations

import base64
import dataclasses
import io
import json
import math
import time
import types
import warnings

import numpy as np
import pytest
import shapely
from PIL import Image

import core.recon.exposure_verifier as ev
from core.recon.exposure_verifier import FrameCamera, LocalFrame, verify_preflight
from mission import swarm_coordinator as SC
from survey import preview as PV
from survey.preview import (
    DATA_MB_PER_PHOTO, HQ_VIEWS, K_PRODUCT, PREVIEW_MAX_POINTS, QUALITY_CLASSES, HdmapPreview,
    build_hdmap_preview, choose_cell_size_m, compute_hdmap_preview, odm_ram_tier, render_preview_map_png,
)

LAT0, LON0 = 23.95, 120.90
FR = LocalFrame(LAT0, LON0)
HQ6 = FrameCamera(6.0, 1.55, 4056, 3040)              # 120 m：125.736 × 94.24 m，GSD 3.1 cm
CAM_EFF = {'preset': 'rpi_hq_6mm', 'focal_mm': 6.0, 'pixel_um': 1.55, 'width_px': 4056, 'height_px': 3040}
ALT = 120.0
SWATH, FOOT_L = HQ6.footprint_m(ALT)
SPACING = SWATH * (1.0 - 0.70)                         # 側向 0.70 ⇒ 37.72 m
TRIG_D = FOOT_L * (1.0 - 0.80)                         # 前向 0.80 ⇒ 18.85 m
GSD_CM = ALT * 1.55 / 6.0 / 10.0                       # 3.1 cm
KX, KY = PV._m_per_deg(LAT0)


def ll(x: float, y: float) -> tuple[float, float]:
    la, lo = FR.to_latlon(x, y)
    return float(la), float(lo)


def rect(x0, y0, x1, y1) -> dict:
    return {'exterior': [ll(x0, y0), ll(x1, y0), ll(x1, y1), ll(x0, y1)], 'holes': []}


def lawnmower(x0: float, x1: float, y0: float, y1: float, *, skip=(), home=(0.0, -200.0)) -> list:
    """home → N–S 條帶（間距 SPACING），每帶 NAV(起) 206(d) NAV(訖) 206(0)。skip 的帶 206 給 0（不拍）。"""
    it = [(0, 16, 0, 0, 0, 0, 0, *ll(*home), 0.0)]
    for k, x in enumerate(np.arange(x0, x1 + 1e-6, SPACING)):
        ya, yb = (y0, y1) if k % 2 == 0 else (y1, y0)
        it.append((len(it), 16, 3, 0, 0, 0, 0, *ll(x, ya), ALT))
        it.append((len(it), 206, 3, 0.0 if k in skip else TRIG_D, 0, 0, 0, 0, 0, 0))
        it.append((len(it), 16, 3, 0, 0, 0, 0, *ll(x, yb), ALT))
        it.append((len(it), 206, 3, 0, 0, 0, 0, 0, 0, 0))
    return it


def run_verifier(missions: dict, polygons: list, cell: float):
    return verify_preflight(missions, polygons, HQ6, airspeed_mps=20.0, wind=(0.0, 0.0), terrain=0.0,
                            intrval_min_s=0.6, trigger_duration_s=0.1, accept_radius='mission',
                            cell_size_m=cell, ks=(5, 9), undercover_k=5)


def decode(png: bytes) -> np.ndarray:
    with Image.open(io.BytesIO(png)) as im:
        assert im.mode == 'RGBA'
        return np.asarray(im)


def pixel_at(pv: HdmapPreview, img: np.ndarray, lat: float, lon: float) -> np.ndarray:
    W, S, E, N = pv.bounds_wsen
    w, h = pv.image_px
    col = int(math.floor((lon - W) / (E - W) * w))
    row = int(math.floor((N - lat) / (N - S) * h))
    return img[row, col]


def class_rgba(i: int) -> tuple:
    col = QUALITY_CLASSES[i][2]
    if col is None:
        return None
    return tuple(int(col[k:k + 2], 16) for k in (1, 3, 5)) + (255,)


def ring_to_xy_poly(ring) -> shapely.Polygon:
    a = np.asarray(ring, dtype=float)
    x, y = FR.to_xy(a[:, 0], a[:, 1])
    return shapely.Polygon(np.column_stack([x, y]))


SMALL_POLY = [rect(0.0, 0.0, 600.0, 400.0)]


@pytest.fixture(scope='module')
def small():
    """600 × 400 m、1 架、全覆蓋（條帶超出測區 20 m／60 m）；格 2 m。"""
    rep = run_verifier({'H1': lawnmower(-20.0, 620.0, -60.0, 460.0)}, SMALL_POLY, 2.0)
    pv = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                             drone_of={'H1': 7}, wind=(0.0, 0.0), wind_source='規劃沒有風層 ⇒ 無風',
                             cam_max_roll_deg=15.0, datum='測試（平地 0 m）')
    return rep, pv


@pytest.fixture(scope='module')
def holed():
    """同上，但中間連續 4 條帶不拍 ⇒ 內部一條視數 0 的破洞。"""
    rep = run_verifier({'H1': lawnmower(-20.0, 620.0, -60.0, 460.0, skip=(7, 8, 9, 10))}, SMALL_POLY, 2.0)
    return rep


# ═══════════════════════════ 常數與分級 ═══════════════════════════
def test_k_product_is_the_upload_gate_threshold_and_classes_are_contiguous():
    assert K_PRODUCT == SC.MAPPING_EXPOSURE_MIN_VIEWS == 5
    assert HQ_VIEWS == 9
    lo_prev = -1
    for lo, hi, col, lab in QUALITY_CLASSES:
        assert lo == lo_prev + 1                                  # 0、1、2、5、9：無縫、不重疊
        lo_prev = lo if hi is None else hi
        assert (col is None) == (lo == 0) and isinstance(lab, str) and lab
        if col:
            assert len(col) == 7 and col.startswith('#') and int(col[1:], 16) >= 0
    assert QUALITY_CLASSES[-1][1] is None
    assert [c[0] for c in QUALITY_CLASSES if c[0] >= K_PRODUCT][0] == K_PRODUCT     # 「可成圖」那級從門檻起
    assert [c[0] for c in QUALITY_CLASSES][-1] == HQ_VIEWS
    v = np.array([0, 1, 2, 4, 5, 8, 9, 30])
    assert PV._class_index(v).tolist() == [0, 1, 2, 2, 3, 3, 4, 4]


def test_block_min_keeps_a_single_cell_hole_and_pads_conservatively():
    v = np.full((100, 100), 12, dtype=np.int32)
    v[37, 58] = 0
    d = PV._block_min(v, 4)
    assert d.shape == (25, 25)
    assert d[37 // 4, 58 // 4] == 0 and int((d == 0).sum()) == 1 and int((d == 12).sum()) == 624
    # 平均或抽樣都會把這格吃掉；block-MIN 不會
    assert v.reshape(25, 4, 25, 4).mean(axis=(1, 3))[9, 14] > 11.0
    # 尾端不足 f 的區塊補 0（保守：不把沒驗證過的格說成有覆蓋）
    d2 = PV._block_min(np.full((10, 10), 7, dtype=np.int32), 4)
    assert d2.shape == (3, 3) and d2[:2, :2].tolist() == [[7, 7], [7, 7]] and (d2[2, :] == 0).all()
    assert PV._block_min(v, 1) is v


def test_views_png_is_north_up_with_transparent_zero_views():
    v = np.array([[0, 1, 3], [5, 8, 9], [9, 9, 0]], dtype=np.int32)       # 第 0 列在南
    img = decode(PV._views_png(v))
    assert img.shape == (3, 3, 4)
    # 圖的第 0 列＝北＝v 的最後一列
    assert tuple(img[0, 0]) == class_rgba(4) and img[0, 2, 3] == 0
    assert tuple(img[1, 0]) == class_rgba(3) and tuple(img[1, 2]) == class_rgba(4)
    assert img[2, 0, 3] == 0 and tuple(img[2, 1]) == class_rgba(1) and tuple(img[2, 2]) == class_rgba(2)


def test_default_cell_size_follows_the_area_rule_and_cell_cap():
    assert choose_cell_size_m([rect(0, 0, 1000, 1000)]) == 2.0
    assert choose_cell_size_m([rect(0, 0, 2000, 1999)]) == 2.0                 # ≤ 4 km²
    assert choose_cell_size_m([rect(0, 0, 3000, 2000)]) == 4.0
    big = [rect(0, 0, 40000, 40000)]
    c = choose_cell_size_m(big)
    assert c > 4.0 and (40600.0 ** 2) / (c * c) <= PV.PREVIEW_MAX_CELLS
    assert (40600.0 ** 2) / ((c - 0.5) ** 2) > PV.PREVIEW_MAX_CELLS             # 取到 0.5 m 的最小值


@pytest.mark.parametrize('n, frag', [(0, '沒有照片'), (400, '≤ 32 GB'), (1000, '約 64 GB'), (1942, '約 128 GB'),
                                     (5300, '--split')])
def test_odm_ram_tier(n, frag):
    assert frag in odm_ram_tier(n)


def test_json_safe_maps_inf_nan_numpy_and_tuples():
    out = PV._json_safe({1: float('inf'), 'b': np.float32(1.5), 'c': (1, np.int64(2)), 'd': float('nan'),
                         'e': np.bool_(True), 'f': np.array([1.0, np.inf])})
    assert out == {'1': None, 'b': 1.5, 'c': [1, 2], 'd': None, 'e': True, 'f': [1.0, None]}
    json.dumps(out, allow_nan=False)


# ═══════════════════════════ 合成任務：數值 ═══════════════════════════
def test_raster_matches_the_verifier_grid_cell_by_cell(small):
    rep, pv = small
    assert pv.ok and pv.reason is None and pv.cell_size_m == 2.0
    W, S, E, N = pv.bounds_wsen
    w, h = pv.image_px
    # bounds 是格邊：寬高（公尺，驗證器本地切平面的尺度）＝像素數 × 格距
    kx, ky = PV._m_per_deg(rep.grid.frame.lat0)
    assert (E - W) * kx == pytest.approx(w * 2.0, rel=1e-9) and (N - S) * ky == pytest.approx(h * 2.0, rel=1e-9)
    img = decode(pv.image_png)
    assert img.shape == (h, w, 4) and w <= PV.PREVIEW_MAX_PX and h <= PV.PREVIEW_MAX_PX
    g = rep.grid
    rng = np.random.default_rng(3)
    ys, xs = np.nonzero(g.views >= 0)
    checked = 0
    for k in rng.choice(len(ys), 4000, replace=False):
        iy, ix = int(ys[k]), int(xs[k])
        lat, lon = g.frame.to_latlon(g.x0 + (ix + 0.5) * g.cell_size_m, g.y0 + (iy + 0.5) * g.cell_size_m)
        if not (S < lat < N and W < lon < E):
            assert g.views[iy, ix] == 0                          # 裁掉的部分都沒有照片
            continue
        want = class_rgba(int(PV._class_index(g.views[iy, ix])))
        got = pixel_at(pv, img, float(lat), float(lon))
        assert (got[3] == 0) if want is None else (tuple(got) == want), (iy, ix, g.views[iy, ix], got)
        checked += 1
    assert checked > 1000
    # 只有分級顏色或全透明
    colors = {tuple(c) for c in img.reshape(-1, 4)}
    allowed = {class_rgba(i) for i in range(1, len(QUALITY_CLASSES))}
    assert all(c[3] == 0 or c in allowed for c in colors)


def test_small_full_coverage_stats_are_the_expected_numbers(small):
    rep, pv = small
    st = pv.stats
    assert set(PV._STATS_KEYS) <= set(st)
    json.dumps(st, allow_nan=False, ensure_ascii=False)
    n = sum(rep.photos_per_uav.values())
    assert st['uav_labels'] == ['H1'] and st['drone_ids'] == [7] and st['photos_per_uav'] == {'H1': n}
    assert st['photos_total'] == n > 300
    assert st['agl_m'] == {'min': 120.0, 'median': 120.0, 'max': 120.0}
    assert st['gsd_cm']['median'] == pytest.approx(GSD_CM, abs=1e-9) == 3.1
    assert st['ortho_resolution_cm'] == 3.1 and st['dsm_resolution_cm_est'] == 6.2
    assert st['data_gb_range'] == [round(n * 7.4 / 1000, 2), round(n * 12.3 / 1000, 2)]
    assert DATA_MB_PER_PHOTO == (7.4, 12.3) and any('7.4–12.3 MB' in a for a in st['assumptions'])
    assert st['camera'] == CAM_EFF
    assert st['k_product'] == 5 and st['cell_size_m'] == 2.0 and st['gate_cell_size_m'] == 4.0
    assert st['interior_hole_count'] == 0 and st['interior_hole_area_m2'] == 0.0
    # 沒給閘門格的報告也沒有閘門摘要（2 m 格）⇒ 依據是「預覽格、閘門結果未知」
    assert st['interior_hole_basis'] == 'preview' and st['interior_hole_cell_m'] == 2.0
    assert st['gate_disagrees'] is None and st['preview_grid_hole_count'] == 0
    assert st['data_mb_per_photo'] == [7.4, 12.3] and st['data_bytes_per_px'] == [0.6, 1.0]
    assert st['photos_shown'] == len(pv.photos) == n
    assert st['frac_ge']['interior'] == {'5': 1.0, '9': 1.0}
    assert st['area_km2']['survey'] == pytest.approx(0.24, rel=1e-4) == rep.area_m2 / 1e6
    assert st['area_km2']['product_in_survey'] <= st['area_km2']['survey'] + 1e-12
    assert st['area_km2']['product'] >= st['area_km2']['high_quality'] > 0.2
    assert 0.75 < st['forward_overlap_p05'] <= 0.81 and 0.65 < st['side_overlap_p05'] <= 0.71
    vf = {f['feature']: f for f in st['visible_features']}
    assert vf['車道線（寬）']['pixels'] == round(15 / 3.1, 1) and vf['小客車（長）']['size_cm'] == 450
    # 誠實規則：預測標籤、樂觀模型、精度只有文獻＋條件、我們的精度未知
    assert st['banner'] == '規劃預測，非實際成果'
    assert SC.MAPPING_EXPOSURE_SEAM_CAVEAT in st['assumptions']
    assert any(a.startswith('CAM_MAX_ROLL 15° 未建模') for a in st['assumptions'])
    assert st['accuracy_note'].startswith('我們自己的精度目前未知') and '≥30 個檢核點' in st['accuracy_note']
    for row in st['accuracy_literature']:
        assert set(row) == {'case', 'horizontal', 'vertical', 'condition'} and row['condition']
    assert any('16.5 → 2.4 cm' in (r['vertical'] or '') and '平地' in r['condition'] for r in st['accuracy_literature'])
    assert pv.hillshade_png is None and st['terrain_context'].startswith('沒有山影')
    assert pv.legend['title'].endswith('規劃預測，非實際成果')
    assert [c['color'] for c in pv.legend['classes']] == [c[2] for c in QUALITY_CLASSES if c[2]]


def test_extent_outline_encloses_exactly_the_cells_with_enough_views(small):
    rep, pv = small
    assert len(pv.extent_rings) == 1 and pv.extent_holes == []
    ext = ring_to_xy_poly(pv.extent_rings[0])
    assert ext.is_valid
    # 面積＝視數 ≥5 的格數 × 4 m²（簡化 1 格，容許 1%）
    assert ext.area == pytest.approx(pv.stats['area_km2']['product'] * 1e6, rel=0.01)
    # 測區內縮半刈幅（內部）一定整個在可成圖範圍內
    survey = ring_to_xy_poly(SMALL_POLY[0]['exterior'])
    assert ext.contains(survey.buffer(-pv.stats['edge_band_m']))
    # 測區 ring 原樣帶出
    assert pv.survey_rings == [[[round(a, 7), round(b, 7)] for a, b in SMALL_POLY[0]['exterior']]]


def test_payload_is_json_safe_and_self_consistent(small):
    rep, pv = small
    p = pv.to_js_payload(opacity=0.5, mask_alpha=0.6, colors=['#112233', '#445566'])
    s = json.dumps(p, allow_nan=False, ensure_ascii=False)
    assert '</' not in s
    assert set(p) >= {'id', 'label', 'bounds', 'image', 'image_px', 'opacity', 'mask_alpha', 'extent_rings',
                      'extent_holes', 'holes', 'edge_short', 'survey_rings', 'points', 'point_colors', 'legend',
                      'stats_line'}
    assert p['id'] == 'hdmap_plan' and p['label'] == '預計成果（規劃預測）'
    assert p['bounds'] == list(pv.bounds_wsen) and p['image_px'] == list(pv.image_px)
    assert p['opacity'] == 0.5 and p['mask_alpha'] == 0.6 and p['point_colors'] == ['#112233', '#445566']
    assert p['image'].startswith('data:image/png;base64,')
    assert base64.b64decode(p['image'].split(',', 1)[1]) == pv.image_png
    assert len(p['points']) == len(pv.photos) == pv.stats['photos_total'] <= PREVIEW_MAX_POINTS
    assert all(pt[2] == 0 for pt in p['points'])
    assert p['legend']['points'] == [{'label': 'H1', 'color': '#112233'}]
    assert '規劃預測，非實際成果' in p['stats_line']
    assert '內部視數不足 0 處（預覽 2 m 格；上傳閘門結果未知，以閘門為準）' in p['stats_line']
    assert all(e['label'].endswith('不擋上傳）') for e in p['edge_short'])
    with pytest.raises(ValueError):
        pv.to_js_payload(opacity=1.5)
    with pytest.raises(ValueError):
        pv.to_js_payload(mask_alpha=float('nan'))


def test_photos_are_decimated_evenly_to_the_cap(small):
    rep, _pv = small
    pv = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                             max_points=50)
    n = len(rep.photos)
    assert len(pv.photos) == 50 and pv.stats['photos_total'] == n            # 統計用全部，點只抽樣
    assert pv.photos[0][:2] == [round(rep.photos[0].lat_deg, 7), round(rep.photos[0].lon_deg, 7)]
    assert pv.photos[-1][:2] == [round(rep.photos[-1].lat_deg, 7), round(rep.photos[-1].lon_deg, 7)]


def test_interior_hole_is_reported_drawn_and_survives_downsampling(holed):
    rep = holed
    pv = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0)
    st = pv.stats
    assert st['interior_hole_count'] >= 1 and st['interior_hole_area_m2'] > 2.0e4
    assert st['interior_hole_area_m2'] == pytest.approx(rep.undercovered_interior_area_m2)
    big = pv.interior_holes[0]
    assert big['min_views'] == 0 and big['area_m2'] == max(h['area_m2'] for h in pv.interior_holes)
    hole_poly = ring_to_xy_poly(big['ring'])
    assert hole_poly.area == pytest.approx(big['area_m2'], rel=0.05)
    # 破洞在 x ≈ 264–377 m 那幾條帶之間
    assert 240.0 < hole_poly.centroid.x < 400.0
    rp = hole_poly.representative_point()
    lat, lon = (float(v) for v in FR.to_latlon(rp.x, rp.y))
    assert pixel_at(pv, decode(pv.image_png), lat, lon)[3] == 0               # 圖上是透明（未拍到）
    # 可成圖範圍被破洞切開（洞跨到測區上下緣 ⇒ 兩塊，或一塊帶內環）
    assert len(pv.extent_rings) + len(pv.extent_holes) >= 2
    p = pv.to_js_payload()
    # 沒有閘門格的結果 ⇒ 標籤不可以說「會擋上傳」（那是閘門的判定，不是 2 m 預覽格的）
    assert p['holes'][0]['label'].startswith('內部視數不足 ') and '會擋上傳' not in p['holes'][0]['label']
    assert '上傳閘門結果未知' in p['holes'][0]['label'] and p['holes'][0]['kind'] == 'hole'
    assert '內部視數不足 ' in p['stats_line'] and '會擋上傳' not in ''.join(p['legend']['notes'])
    # 給了閘門格（4 m）的報告 ⇒ 用它的破洞說「會擋上傳」
    g4 = run_verifier({'H1': lawnmower(-20.0, 620.0, -60.0, 460.0, skip=(7, 8, 9, 10))}, SMALL_POLY, 4.0)
    pg = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0, gate_report=g4)
    gp = pg.to_js_payload()
    assert pg.stats['interior_hole_basis'] == 'gate' and pg.stats['interior_hole_count'] == len(
        [a for a in g4.undercovered if a.region == 'interior']) >= 1
    assert pg.stats['interior_hole_area_m2'] == pytest.approx(g4.undercovered_interior_area_m2)
    assert gp['holes'][0]['label'].startswith('內部破洞 ') and '上傳閘門 4 m 格' in gp['holes'][0]['label']
    assert '會擋上傳' in gp['holes'][0]['label'] and pg.interior_holes[0]['source'] == 'gate'
    assert pg.stats['gate_disagrees'] is False, '大破洞兩個格距都看得到'
    # 縮成每邊 ≤ 40 px（block-MIN）：破洞照樣在
    small_pv = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                                   max_px=40)
    assert max(small_pv.image_px) <= 40 and small_pv.cell_size_m > 2.0
    assert small_pv.stats['raster_cell_m'] == small_pv.cell_size_m and small_pv.stats['cell_size_m'] == 2.0
    assert pixel_at(small_pv, decode(small_pv.image_png), lat, lon)[3] == 0
    W, S, E, N = small_pv.bounds_wsen
    kx, ky = PV._m_per_deg(rep.grid.frame.lat0)
    assert (E - W) * kx == pytest.approx(small_pv.image_px[0] * small_pv.cell_size_m, rel=1e-9)
    assert (N - S) * ky == pytest.approx(small_pv.image_px[1] * small_pv.cell_size_m, rel=1e-9)


def _hill_dem(amp: float):
    from hv_dccpp.geo import DemGrid
    lat0, lon0, d = LAT0 - 0.02, LON0 - 0.02, 0.0003
    n = 160
    la = lat0 + np.arange(n) * d
    lo = lon0 + np.arange(n) * d
    LO, LA = np.meshgrid(lo, la)
    c_lat, c_lon = ll(300.0, 200.0)
    z = 50.0 + amp * np.exp(-(((LA - c_lat) * KY) ** 2 + ((LO - c_lon) * KX) ** 2) / (2 * 250.0 ** 2))
    return DemGrid(lat0, lon0, d, d, z.astype(np.float32))


@pytest.mark.parametrize('world', [False, True])
def test_hillshade_only_when_dem_is_not_world_terrain(world):
    dem = _hill_dem(30.0)
    flat_home = float(dem.sample_msl(*ll(0.0, -200.0))[0])
    items = lawnmower(-20.0, 620.0, -60.0, 460.0)
    items[0] = items[0][:9] + (flat_home,)                     # home 高＝DEM 在 home 的地面
    rep = verify_preflight({'H1': items}, SMALL_POLY, HQ6, airspeed_mps=20.0, wind=(0.0, 0.0),
                           terrain=lambda a, b: float(dem.sample_msl(a, b)[0]), intrval_min_s=0.6,
                           cell_size_m=2.0, ks=(5, 9), undercover_k=5)
    pv = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF,
                             terrain=lambda a, b: float(dem.sample_msl(a, b)[0]), dem=dem, is_world_terrain=world,
                             datum='x')
    assert pv.ok
    # AGL 隨地形變（山頂比 home 高 ⇒ AGL 小於 120），GSD 跟著變
    assert pv.stats['agl_m']['min'] < 100.0 < pv.stats['agl_m']['max'] <= 120.5
    assert pv.stats['gsd_cm']['min'] == pytest.approx(pv.stats['agl_m']['min'] * 1.55 / 6.0 / 10.0, abs=0.02)
    assert pv.stats['terrain_is_world_terrain'] is world
    if world:
        assert pv.hillshade_png is None and pv.hillshade_bounds_wsen is None
        assert any('World Terrain' in a and '不產生' in a for a in pv.stats['assumptions'])
        assert pv.stats['terrain_context'].startswith('不產生')
    else:
        hs = decode(pv.hillshade_png)
        hW, hS, hE, hN = pv.hillshade_bounds_wsen
        W, S, E, N = pv.bounds_wsen
        assert hW <= W and hS <= S and hE >= E and hN >= N               # 山影蓋住整張上色圖
        assert hs.shape[2] == 4 and (hs[..., 3] == 255).all() and np.ptp(hs[..., 0]) > 40
        # north-up：山的西北坡（迎光，azimuth 315°）比東南坡亮
        lat_c, lon_c = ll(300.0, 200.0)
        h, w = hs.shape[:2]

        def px(lat, lon):
            return hs[int((hN - lat) / (hN - hS) * h), int((lon - hW) / (hE - hW) * w), 0]
        nw = px(*ll(300.0 - 180.0, 200.0 + 180.0))
        se = px(*ll(300.0 + 180.0, 200.0 - 180.0))
        assert int(nw) > int(se) + 30


def test_render_png_is_aspect_correct_ascii_only_and_handles_failures(small, holed):
    _rep, pv = small
    with warnings.catch_warnings():
        warnings.simplefilter('error')                       # 缺字（CJK）會發 UserWarning ⇒ 這裡變成失敗
        png = render_preview_map_png(pv, width_px=900)
        png_h = render_preview_map_png(build_hdmap_preview(holed, survey_polygons=SMALL_POLY,
                                                           camera_effective=CAM_EFF, terrain=0.0))
        png_f = render_preview_map_png(HdmapPreview.failed('原因'))
    with Image.open(io.BytesIO(png)) as im:
        w, h = im.size
    assert w == 900
    W, S, E, N = pv.bounds_wsen
    want = ((N - S) * KY) / ((E - W) * KX)
    assert h / w == pytest.approx(want, rel=0.05)
    for b in (png_h, png_f):
        assert b[:8] == b'\x89PNG\r\n\x1a\n'


# ═══════════════════════════ compute_hdmap_preview：不丟例外、只吃建圖計畫 ═══════════════════════════
def _fake(**over):
    hv = types.SimpleNamespace(last_ctx=object(), last_plan=object(), mapping_mode=object(), camera_cfg=object(),
                               notes=[])
    sw = types.SimpleNamespace(hv_mapping_setup={'camera': HQ6, 'effective': dict(CAM_EFF)}, hv_coordinator=hv,
                               hv_exposure_summary=None, hv_mapping_gate=None, harvest_params={})
    for k, v in over.items():
        tgt, name = k.split('__')
        setattr(hv if tgt == 'hv' else sw, name, v)
    return hv, sw


@pytest.mark.parametrize('over, frag', [
    ({'hv__last_plan': None}, '尚未規劃'),
    ({'hv__mapping_mode': None}, '不是航測建圖模式'),
    ({'sw__hv_mapping_setup': None}, '沒有建圖設定'),
    ({'sw__hv_coordinator': object()}, '換成另一份'),
    ({'hv__camera_cfg': None}, 'camera_cfg'),
    ({'sw__hv_mapping_setup': {'camera': HQ6, 'effective': dict(CAM_EFF, pixel_um=None)}}, '像素尺寸'),
])
def test_not_a_usable_mapping_plan_gives_a_reason_not_an_exception(over, frag, monkeypatch):
    def boom(*a, **k):
        raise AssertionError('不該跑到驗證器')
    monkeypatch.setattr(SC, 'mapping_exposure_report', boom)
    hv, sw = _fake(**over)
    pv = compute_hdmap_preview(hv, sw)
    assert pv.ok is False and frag in pv.reason
    assert set(PV._STATS_KEYS) <= set(pv.stats) and pv.stats['k_product'] == 5
    json.dumps(pv.to_js_payload(), allow_nan=False)
    assert compute_hdmap_preview(None, sw).ok is False


def test_verifier_failure_becomes_a_reason(monkeypatch):
    def boom(hv, camera, *, cell_size_m):
        raise ValueError('boom-verifier')
    monkeypatch.setattr(SC, '_mapping_survey_polygons', lambda ctx: (SMALL_POLY, 0))
    monkeypatch.setattr(SC, 'mapping_exposure_report', boom)
    hv, sw = _fake()
    pv = compute_hdmap_preview(hv, sw)
    assert pv.ok is False and 'boom-verifier' in pv.reason and pv.reason.startswith('預覽計算失敗')
    assert compute_hdmap_preview(hv, sw, cell_size_m=-1.0).reason.startswith('格距須為正數')


def _synthetic_report_patch(monkeypatch, missions, polys, seen):
    def report(hv, camera, *, cell_size_m):
        seen.append(cell_size_m)
        rep = run_verifier(missions, polys, cell_size_m)
        return rep, {'polygons': polys, 'n_nofly': 0, 'dem': None, 'terrain': 0.0,
                     'drone_of': {u: i + 1 for i, u in enumerate(sorted(missions))}, 'wind': (0.0, 0.0),
                     'wind_source': '規劃沒有風層 ⇒ 無風', 'cam_max_roll_deg': 15.0}
    monkeypatch.setattr(SC, '_mapping_survey_polygons', lambda ctx: (polys, 0))
    monkeypatch.setattr(SC, 'mapping_exposure_report', report)


def test_compute_uses_2m_for_small_areas_and_reports_gate_and_datum(monkeypatch):
    seen: list = []
    _synthetic_report_patch(monkeypatch, {'H1': lawnmower(-20.0, 620.0, -60.0, 460.0)}, SMALL_POLY, seen)
    hv, sw = _fake(sw__hv_exposure_summary={'ok': True, 'cell_size_m': 4.0, 'interior_hole_count': 0,
                                            'undercovered_interior_area_m2': 0.0},
                   sw__hv_mapping_gate={'uploadable': True, 'reasons': []})
    pv = compute_hdmap_preview(hv, sw)
    assert pv.ok and seen == [2.0, 4.0], '預覽格 2 m ＋ 閘門格 4 m 另跑一次（內部破洞以閘門格為準）'
    assert pv.stats['gate'] == {'cell_size_m': 4.0, 'interior_hole_count': 0, 'undercovered_interior_area_m2': 0.0,
                                'uploadable': True}
    assert pv.stats['interior_hole_basis'] == 'gate' and pv.stats['interior_hole_cell_m'] == 4.0
    assert pv.stats['datum'] == '規劃 DEM（未知基準）' and pv.stats['terrain_is_world_terrain'] is False
    assert any('上傳閘門格 4 m' in a and '閘門格另算一次' in a for a in pv.stats['assumptions'])
    # World Terrain 的三種認法：協調器的取樣器、面板勾選、轉接層的 terrain_source note
    hv_a, sw_a = _fake()
    sw_a._world_terrain = object()
    cases = [(hv_a, sw_a), _fake(sw__harvest_params={'hv_world_terrain': True}),
             _fake(hv__notes=['terrain_source(Cesium World Terrain，橢球高…)'])]
    for hv2, sw2 in cases:
        pv2 = compute_hdmap_preview(hv2, sw2)
        assert pv2.ok and pv2.stats['terrain_is_world_terrain'] is True and pv2.hillshade_png is None
        assert pv2.stats['datum'].startswith('Cesium World Terrain')


def test_synthetic_12km2_full_coverage_end_to_end_timing(monkeypatch, capsys):
    """4 × 3 km、3 架各一塊、全覆蓋：走 compute_hdmap_preview（只把規劃器那層換成合成任務）。"""
    polys = [rect(0.0, 0.0, 4000.0, 3000.0)]
    third = 4000.0 / 3.0
    missions = {f'U{i + 1}': lawnmower(i * third - 20.0, (i + 1) * third + 20.0, -60.0, 3060.0,
                                       home=(i * third, -300.0)) for i in range(3)}
    seen: list = []
    _synthetic_report_patch(monkeypatch, missions, polys, seen)
    hv, sw = _fake()
    t = time.perf_counter()
    pv = compute_hdmap_preview(hv, sw)
    dt = time.perf_counter() - t
    with capsys.disabled():
        print(f"\n[timing] synthetic 12 km² full coverage: compute_hdmap_preview {dt:.2f} s "
              f"(stats elapsed {pv.stats['elapsed_s']} s, cell {seen}, photos {pv.stats['photos_total']}, "
              f"image {pv.image_px}, png {len(pv.image_png)} B)")
    assert pv.ok, pv.reason
    assert seen == [4.0]                                          # > 4 km² ⇒ 4 m
    st = pv.stats
    assert st['area_km2']['survey'] == pytest.approx(12.0, rel=1e-3)          # 驗證器以測區中心為切平面原點
    assert st['interior_hole_count'] == 0 and st['frac_ge']['interior']['5'] == 1.0
    assert st['uav_labels'] == ['U1', 'U2', 'U3'] and st['drone_ids'] == [1, 2, 3]
    assert st['photos_total'] > 15000 and len(pv.photos) <= PREVIEW_MAX_POINTS
    assert '--split' in st['odm_ram_tier']
    assert {p[2] for p in pv.photos} == {0, 1, 2}
    s = json.dumps(pv.to_js_payload(), allow_nan=False)
    assert len(s) < 3_000_000
    assert dt < 120.0


# ═══════════════════════════ 真規劃（約 20 s）═══════════════════════════
@pytest.fixture(scope='module')
def mapped():
    pytest.importorskip('hv_dccpp')
    from tests.test_hdmap_gui_coordinator import mapping_params, plan
    return plan(mapping_params())


def test_real_plan_preview_feeds_the_verifier_exactly_the_gate_inputs(mapped, monkeypatch):
    hv = mapped.hv_coordinator
    cam = mapped.hv_mapping_setup['camera']
    calls: list = []
    real = ev.verify_preflight

    def spy(missions, polygons, camera, **kw):
        calls.append(dict(kw, missions=missions, polygons=polygons, camera=camera))
        return real(missions, polygons, camera, **kw)
    monkeypatch.setattr(ev, 'verify_preflight', spy)
    gate = SC.mapping_exposure_summary(hv, cam)
    pv = compute_hdmap_preview(hv, mapped)
    assert pv.ok, pv.reason
    assert len(calls) == 3
    g, p, g2 = calls                                                        # 閘門、預覽（2 m）、預覽的閘門格重跑
    assert g['cell_size_m'] == 4.0 and p['cell_size_m'] == 2.0 and g2['cell_size_m'] == 4.0   # 只有格距不同
    for key in ('missions', 'polygons', 'airspeed_mps', 'airspeed_limits_mps', 'wind', 'intrval_min_s',
                'trigger_duration_s', 'accept_radius', 'ks', 'undercover_k'):
        assert p[key] == g[key] == g2[key], key
    assert pv.stats['interior_hole_count'] == gate['interior_hole_count']
    assert pv.stats['interior_hole_area_m2'] == pytest.approx(gate['undercovered_interior_area_m2'])
    assert p['camera'] is g['camera'] is cam
    for row in g['missions'][sorted(g['missions'])[0]][1:40]:
        if row['lat'] or row['lon']:
            assert p['terrain'](row['lat'], row['lon']) == g['terrain'](row['lat'], row['lon'])
    # 同一份模擬 ⇒ 照片數與閘門摘要一樣（與格距無關）
    assert pv.stats['photos_per_uav'] == gate['photos_per_uav'] == mapped.hv_exposure_summary['photos_per_uav']
    assert pv.stats['area_km2']['survey'] == pytest.approx(gate['area_m2'] / 1e6)


def test_real_plan_preview_values(mapped):
    hv = mapped.hv_coordinator
    t = time.perf_counter()
    pv = compute_hdmap_preview(hv, mapped)
    dt = time.perf_counter() - t
    print(f"\n[timing] fixture plan (1.2 × 1.0 km, 3 架): compute_hdmap_preview {dt:.2f} s")
    assert pv.ok, pv.reason
    st = pv.stats
    gate = mapped.hv_exposure_summary
    assert st['cell_size_m'] == 2.0 and pv.cell_size_m == 2.0
    assert st['uav_labels'] == sorted(hv.last_plan.plans) and st['drone_ids'] == [
        {u: d for d, u in hv.id_map.items()}[u] for u in st['uav_labels']]
    assert st['interior_hole_count'] == gate['interior_hole_count'] == 0
    assert st['interior_hole_basis'] == 'gate' and st['gate_disagrees'] is False
    assert st['frac_ge']['interior']['5'] == 1.0
    assert st['gsd_cm']['median'] == pytest.approx(3.1, abs=0.01) and st['ortho_resolution_cm'] == 3.1
    n = st['photos_total']
    assert n == sum(gate['photos_per_uav'].values())
    assert st['data_gb_range'] == [round(n * 7.4 / 1000, 2), round(n * 12.3 / 1000, 2)]
    assert st['odm_ram_tier'] == odm_ram_tier(n)
    assert st['gate']['uploadable'] is True and st['gate']['cell_size_m'] == 4.0
    assert st['datum'] == f"規劃 DEM（{hv.last_ctx.inputs.dem.datum_label()}）"
    assert st['wind'] == [0.0, 0.0]
    assert pv.hillshade_png is None                                         # 夾具是平地
    json.dumps(pv.to_js_payload(), allow_nan=False)
    # 測區內縮邊緣帶（內部）⊂ 可成圖範圍
    ext = shapely.union_all([ring_to_xy_poly(r) for r in pv.extent_rings])
    for ring in pv.survey_rings:
        inner = ring_to_xy_poly(ring).buffer(-st['edge_band_m'])
        assert ext.buffer(1.0).contains(inner)
    assert dt < 30.0


def test_real_plan_dark_camera_hole_is_on_the_map(mapped, monkeypatch):
    hv = mapped.hv_coordinator
    real = hv.mission_items
    dark = sorted(hv.id_map)[0]

    def items(did, **kw):
        out = real(did, **kw)
        if int(did) == dark:
            out = [dict(it, param1=0.0) if it['command'] == 206 else it for it in out]
        return out
    monkeypatch.setattr(hv, 'mission_items', items)
    pv = compute_hdmap_preview(hv, mapped)
    assert pv.ok, pv.reason
    uid = hv.id_map[dark]
    assert uid in pv.stats['uav_labels'] and pv.stats['photos_per_uav'][uid] == 0
    assert pv.stats['interior_hole_count'] >= 1 and pv.stats['interior_hole_area_m2'] > 5.0e4
    big = pv.interior_holes[0]
    assert big['min_views'] == 0
    rp = ring_to_xy_poly(big['ring']).representative_point()
    lat, lon = (float(v) for v in FR.to_latlon(rp.x, rp.y))
    W, S, E, N = pv.bounds_wsen
    if S < lat < N and W < lon < E:
        assert pixel_at(pv, decode(pv.image_png), lat, lon)[3] == 0
    lab = pv.to_js_payload()['holes'][0]['label']
    assert lab.startswith('內部破洞 ') and '上傳閘門 4 m 格' in lab and '會擋上傳' in lab
    assert pv.stats['interior_hole_count'] >= 1 and pv.stats['interior_hole_basis'] == 'gate'


class _Proxy:
    def __init__(self, base, **over):
        self._b, self._o = base, over

    def __getattr__(self, n):
        return self._o[n] if n in self._o else getattr(self._b, n)


def test_real_plan_world_terrain_never_gets_a_hillshade(mapped, monkeypatch):
    hv = mapped.hv_coordinator
    ctx = hv.last_ctx
    dem = ctx.inputs.dem
    la = dem.lat0_deg + np.arange(dem.rows) * dem.dlat_deg
    lo = dem.lon0_deg + np.arange(dem.cols) * dem.dlon_deg
    LO, LA = np.meshgrid(lo, la)
    c_lat, c_lon = 23.95 + 100.0 / 111132.0, 120.90
    hill = 15.0 * np.exp(-(((LA - c_lat) * 111132.0) ** 2 + ((LO - c_lon) * 101800.0) ** 2) / (2 * 300.0 ** 2))
    hilly = dataclasses.replace(dem, z_msl=np.asarray(dem.z_msl, dtype=np.float32) + hill.astype(np.float32))
    monkeypatch.setattr(hv, 'last_ctx', _Proxy(ctx, inputs=_Proxy(ctx.inputs, dem=hilly)))
    pv = compute_hdmap_preview(hv, mapped)
    assert pv.ok and pv.hillshade_png is not None and pv.stats['terrain_is_world_terrain'] is False
    monkeypatch.setattr(mapped, '_world_terrain', object(), raising=False)
    pv_w = compute_hdmap_preview(hv, mapped)
    assert pv_w.ok and pv_w.hillshade_png is None and pv_w.hillshade_bounds_wsen is None
    assert pv_w.stats['terrain_is_world_terrain'] is True and pv_w.stats['datum'].startswith('Cesium World Terrain')
    # 規劃的數值本身不受「是不是 World Terrain」影響
    assert pv_w.stats['photos_per_uav'] == pv.stats['photos_per_uav'] and pv_w.image_png == pv.image_png


def test_report_and_summary_share_inputs(mapped):
    hv = mapped.hv_coordinator
    cam = mapped.hv_mapping_setup['camera']
    rep, inp = SC.mapping_exposure_report(hv, cam)
    s = SC.mapping_exposure_summary(hv, cam)
    assert s['photos_per_uav'] == dict(rep.photos_per_uav) and s['drone_of'] == inp['drone_of']
    assert s['assumptions'] == SC.mapping_exposure_assumptions(inp['wind'], inp['wind_source'], 4.0,
                                                               inp['cam_max_roll_deg'])
    assert s['effective']['airspeed_initial_mps'] == inp['airspeed_mps']
    assert s['effective']['intrval_min_s'] == inp['intrval_min_s'] == float(hv.camera_cfg.interval_min_s)
    assert inp['cam_max_roll_deg'] == hv.camera_cfg.max_roll_deg and inp['k'] == 5 and inp['ks'] == (5, 9)
    assert rep.grid is not None and inp['cell_size_m'] == 4.0


# ═══════════════════════════ 審查修正（2026-10-19）：閘門格、資料量、抽樣揭露、文獻、格數上限 ═══════════════════════════
def _seam_missions(gap: float, off: float) -> dict:
    """兩架各一塊 N–S 割草機（每帶 10 視），兩塊之間留 gap 公尺的「零視數」接縫、整組平移 off 公尺。
    接縫比格子窄 ⇒ 2 m 預覽格與 4 m 閘門格的格心落點不同，兩邊可能看到不同的破洞（審查重現用的同一組幾何）。"""
    d = FOOT_L * 0.10

    def mower(xs, home):
        it = [(0, 16, 0, 0, 0, 0, 0, *ll(*home), 0.0)]
        for k, x in enumerate(xs):
            ya, yb = (-60.0, 460.0) if k % 2 == 0 else (460.0, -60.0)
            it.append((len(it), 16, 3, 0, 0, 0, 0, *ll(x, ya), ALT))
            it.append((len(it), 206, 3, d, 0, 0, 0, 0, 0, 0))
            it.append((len(it), 16, 3, 0, 0, 0, 0, *ll(x, yb), ALT))
            it.append((len(it), 206, 3, 0, 0, 0, 0, 0, 0, 0))
        return it
    sp_ = SWATH * 0.30
    xa = list(np.arange(-20.0, 250.0, sp_) + off)
    xb = list(np.arange(xa[-1] + SWATH + gap, 640.0, sp_))
    return {'U1': mower(xa, (0.0, -200.0)), 'U2': mower(xb, (300.0, -200.0))}


def _seam_reports(gap: float, off: float):
    ms = _seam_missions(gap, off)
    kw = dict(airspeed_mps=20.0, wind=(0.0, 0.0), terrain=0.0, intrval_min_s=0.0, trigger_duration_s=0.1,
              accept_radius='mission', ks=(5, 9), undercover_k=5)
    return (verify_preflight(ms, SMALL_POLY, HQ6, cell_size_m=2.0, **kw),
            verify_preflight(ms, SMALL_POLY, HQ6, cell_size_m=4.0, **kw))


@pytest.mark.parametrize('gap, off, gate_holes, preview_holes', [(3.0, 0.5, 0, 1), (1.5, 3.5, 1, 0)])
def test_upload_wording_follows_the_gate_grid_not_the_preview_grid(gap, off, gate_holes, preview_holes):
    """2 m 預覽格與 4 m 閘門格看到的接縫破洞不同（兩個方向都有）：「會擋上傳」、破洞數、摘要行只跟閘門格走，
    只在預覽格看到的標「閘門沒擋」，閘門有而預覽格沒有的照樣畫出來，gate_disagrees＝True。"""
    r2, r4 = _seam_reports(gap, off)
    h2 = [a for a in r2.undercovered if a.region == 'interior']
    h4 = [a for a in r4.undercovered if a.region == 'interior']
    assert (len(h4), len(h2)) == (gate_holes, preview_holes), '前提：兩個格距看到的不同（否則這條量不到東西）'
    pv = build_hdmap_preview(r2, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                             drone_of={'U1': 1, 'U2': 2}, wind=(0.0, 0.0), wind_source='無風', cam_max_roll_deg=15.0,
                             gate_report=r4,
                             gate={'cell_size_m': 4.0, 'interior_hole_count': len(h4),
                                   'undercovered_interior_area_m2': r4.undercovered_interior_area_m2,
                                   'uploadable': not h4})
    st, p = pv.stats, pv.to_js_payload()
    assert st['interior_hole_basis'] == 'gate' and st['interior_hole_count'] == gate_holes
    assert st['interior_hole_area_m2'] == pytest.approx(r4.undercovered_interior_area_m2)
    assert st['preview_grid_hole_count'] == preview_holes and st['gate_disagrees'] is True
    gate_labels = [h['label'] for h in p['holes'] if h['kind'] == 'hole']
    fine_labels = [h['label'] for h in p['holes'] if h['kind'] == 'preview_only']
    assert len(gate_labels) == gate_holes and all('會擋上傳' in t and '上傳閘門 4 m 格' in t for t in gate_labels)
    assert len(fine_labels) == st['preview_only_hole_count'] == (1 if preview_holes and not gate_holes else 0)
    assert all('會擋上傳' not in t and '沒擋' in t and '預覽 2 m 格' in t for t in fine_labels)
    assert f'內部破洞 {gate_holes} 處（上傳閘門 4 m 格）' in p['stats_line']
    if fine_labels:
        assert '預覽 2 m 格另見 1 處（閘門沒擋' in p['stats_line']
        assert any(n.startswith('紅點框＝只在預覽 2 m 細格看到') for n in p['legend']['notes'])
    else:                                                       # 閘門有、預覽格沒有 ⇒ 閘門那一處照樣畫
        a = np.asarray(pv.interior_holes[0]['ring'], dtype=float)           # 驗證器的切平面（以測區中心為原點）
        ring = shapely.Polygon(np.column_stack(r4.grid.frame.to_xy(a[:, 0], a[:, 1])))
        assert ring.distance(h4[0].geometry_enu) < 1.0 and pv.interior_holes[0]['blocks_upload'] is True
        assert ring.area == pytest.approx(h4[0].area_m2, rel=0.1)
    assert any(n.startswith('⚠ 預覽格與上傳閘門格看到的內部破洞不同') for n in p['legend']['notes'])
    assert any('會擋上傳' in n and '上傳閘門 4 m 格判定' in n for n in p['legend']['notes'])
    render_preview_map_png(pv, width_px=400)                     # 兩種破洞畫法都畫得出來


def test_gate_summary_fallback_marks_the_gate_centroid_when_the_preview_grid_misses_it():
    """閘門格沒重跑（格數超過上限）時：個數用規劃時的閘門摘要；閘門的破洞中心不在預覽格破洞附近 ⇒ 畫圓圈標位置。"""
    r2, r4 = _seam_reports(1.5, 3.5)
    h4 = [a for a in r4.undercovered if a.region == 'interior']
    summ = [{'area_m2': float(a.area_m2), 'cells': int(a.cells), 'min_views': int(a.min_views),
             'centroid_latlon': tuple(a.centroid_latlon)} for a in h4]
    pv = build_hdmap_preview(r2, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                             gate={'cell_size_m': 4.0, 'interior_hole_count': len(h4),
                                   'undercovered_interior_area_m2': r4.undercovered_interior_area_m2,
                                   'uploadable': False},
                             gate_holes_summary=summ, gate_note='閘門格 4 m 約 9,999,999 格，超過預覽格數上限')
    st, p = pv.stats, pv.to_js_payload()
    assert st['interior_hole_basis'] == 'gate_summary' and st['interior_hole_count'] == 1
    assert st['gate_marker_count'] == 1 and st['gate_disagrees'] is True
    mk = [h for h in p['holes'] if h['kind'] == 'marker']
    assert len(mk) == 1 and '會擋上傳' in mk[0]['label'] and '圓圈只標中心位置' in mk[0]['label']
    c = ring_to_xy_poly(pv.interior_holes[-1]['ring']).centroid
    gx, gy = FR.to_xy(*h4[0].centroid_latlon)
    assert math.hypot(c.x - float(gx), c.y - float(gy)) < 1.0
    assert any('這次沒有重算閘門格' in a and '超過預覽格數上限' in a for a in st['assumptions'])
    assert '內部破洞 1 處（上傳閘門 4 m 格）' in p['stats_line']


def test_compute_falls_back_to_the_gate_summary_when_the_gate_grid_is_too_big(monkeypatch):
    seen: list = []
    _synthetic_report_patch(monkeypatch, {'H1': lawnmower(-20.0, 620.0, -60.0, 460.0)}, SMALL_POLY, seen)
    monkeypatch.setattr(PV, 'PREVIEW_MAX_CELLS', 20_000)
    hv, sw = _fake(sw__hv_exposure_summary={'ok': True, 'cell_size_m': 4.0, 'interior_hole_count': 0,
                                            'undercovered_interior_area_m2': 0.0, 'interior_holes': []},
                   sw__hv_mapping_gate={'uploadable': True})
    pv = compute_hdmap_preview(hv, sw, cell_size_m=8.0)          # 8 m 格估 1.9 萬格（實際 6,650）；閘門 4 m 格 ×4 ⇒ 超過
    assert pv.ok, pv.reason
    assert seen == [8.0], '閘門格估出來超過上限就不重跑'
    assert pv.stats['interior_hole_basis'] == 'gate_summary' and pv.stats['interior_hole_count'] == 0
    assert any('超過預覽格數上限' in a for a in pv.stats['assumptions'])


def test_explicit_cell_size_is_held_to_the_cell_cap_before_simulating(monkeypatch):
    def boom(*a, **k):
        raise AssertionError('格距太細應該在模擬前就擋下')
    big = [rect(0.0, 0.0, 4000.0, 3000.0)]
    monkeypatch.setattr(SC, '_mapping_survey_polygons', lambda ctx: (big, 0))
    monkeypatch.setattr(SC, 'mapping_exposure_report', boom)
    hv, sw = _fake()
    pv = compute_hdmap_preview(hv, sw, cell_size_m=0.5)
    est = PV.estimate_cells(big, 0.5)
    assert est == pytest.approx(4600.0 * 3600.0 / 0.25, rel=1e-3)
    assert pv.ok is False and pv.reason.startswith('格距太細') and f'{est:,.0f}' in pv.reason
    need = float(pv.reason.split('≥ ')[1].split(' m')[0])
    assert PV.estimate_cells(big, need) <= PV.PREVIEW_MAX_CELLS < PV.estimate_cells(big, need - 0.5)


@pytest.mark.parametrize('w, h, lo, hi', [(4056, 3040, 7.4, 12.3),       # RPi HQ（IMX477）：維持 7.4–12.3 MB
                                          (1456, 1088, 0.95, 1.58),      # RPi Global Shutter（IMX296）1.58 MP
                                          (4608, 2592, 7.17, 11.9)])     # RPi CM3（IMX708）
def test_data_volume_scales_with_the_camera_pixel_count(w, h, lo, hi):
    assert PV.data_mb_per_photo({'width_px': w, 'height_px': h})[:2] == (lo, hi)
    rep = run_verifier({'H1': lawnmower(-20.0, 620.0, -60.0, 460.0)}, SMALL_POLY, 4.0)
    cam = dict(CAM_EFF, width_px=w, height_px=h)
    st = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=cam, terrain=0.0).stats
    n = st['photos_total']
    assert st['data_mb_per_photo'] == [lo, hi] and st['data_gb_range'] == [round(n * lo / 1000, 2),
                                                                          round(n * hi / 1000, 2)]
    txt = next(a for a in st['assumptions'] if a.startswith('資料量假設'))
    assert f'每張 {lo:g}–{hi:g} MB' in txt and f'{w * h / 1e6:.2f} MP' in txt and '0.6–1.0 B/px' in txt
    assert '未經實測' in txt and 'phase2_verify.md' in txt
    if (w, h) == (1456, 1088):                                   # 比 8-bit RGB 不壓縮（4.75 MB）還大是不可能的
        assert hi < w * h * 3 / 1e6


def test_data_volume_falls_back_and_says_so_when_pixels_are_unknown():
    assert PV.data_mb_per_photo({'width_px': None, 'height_px': 3040}) == (7.4, 12.3, None)
    rep = run_verifier({'H1': lawnmower(-20.0, 620.0, -60.0, 460.0)}, SMALL_POLY, 4.0)
    st = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=dict(CAM_EFF, width_px=None),
                             terrain=0.0).stats
    txt = next(a for a in st['assumptions'] if a.startswith('資料量假設'))
    assert '相機像素數未知' in txt and '7.4–12.3 MB' in txt and '未經實測' in txt
    assert st['data_bytes_per_px'] is None and '相機像素數未知' in st['data_mb_basis']


def test_stats_line_uses_the_in_survey_area_and_qualifies_the_whole_grid_value(small):
    _rep, pv = small
    a = pv.stats['area_km2']
    assert a['product'] > a['survey'] * 1.2, '前提：足跡伸出測區（整張格網的值比測區大）'
    line = pv.stats_line()
    assert line.startswith(f"預計成果（規劃預測）：測區內可成圖 {a['product_in_survey']:.2f} km²（測區 {a['survey']:.2f} km²；"
                           f"白框含測區外足跡 {a['product']:.2f} km²），≥9 張 {a['high_quality_in_survey']:.2f} km²")
    # 只有整張格網的值（舊／合成的統計）⇒ 明寫「含測區外足跡」
    old = HdmapPreview(ok=True, stats=dict(pv.stats, area_km2={'survey': 0.24, 'product': 0.37, 'high_quality': 0.33}))
    assert '可成圖 0.37 km²（含測區外足跡）' in old.stats_line()


def test_decimated_points_and_block_min_raster_are_disclosed(small):
    rep, _pv = small
    pv = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                             max_points=50, max_px=40)
    n = len(rep.photos)
    fac = int(round(pv.stats['raster_cell_m'] / 2.0))
    assert fac > 1 and pv.stats['photos_shown'] == 50 < n
    notes = pv.legend['notes']
    assert f'拍照點只畫 50／{n:,} 張（均勻抽樣；張數統計用全部）' in notes
    want = f'上色圖每像素＝{fac}×{fac} 格取最少視數（{2.0 * fac:g} m，保守顯示；統計仍以 2 m 格計）'
    assert want in notes and want in pv.stats['assumptions']
    assert any(a.startswith('地圖上的拍照點只畫 50／') for a in pv.stats['assumptions'])
    # 沒抽樣、沒縮圖 ⇒ 不寫（不然每張圖都有一句沒意義的話）
    _r, full = small
    assert not any('拍照點只畫' in x or '上色圖每像素' in x for x in full.legend['notes'] + full.stats['assumptions'])


def test_accuracy_table_has_the_same_altitude_cross_strip_row_and_the_plan_condition_is_named(small):
    rep, pv = small
    rows = {r['case']: r for r in pv.stats['accuracy_literature']}
    st_row = rows['同高度垂直交叉（Štroner 2021）']
    assert st_row['vertical'] == '只有小幅改善' and '不能套用' in st_row['condition'] and '不換高度' in st_row['condition']
    same = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                               cross={'cross_strips': True, 'cross_alt_offset_m': 0.0})
    assert any('同高度（高度差 0 m）' in a and 'Štroner 2021' in a and '不適用' in a for a in same.stats['assumptions'])
    assert same.stats['cross_strips'] == {'on': True, 'alt_offset_m': 0.0}
    lower = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                                cross={'cross_strips': True, 'cross_alt_offset_m': -20.0})
    assert any(a.startswith('交叉航帶高度差 -20 m') for a in lower.stats['assumptions'])
    off = build_hdmap_preview(rep, survey_polygons=SMALL_POLY, camera_effective=CAM_EFF, terrain=0.0,
                              cross={'cross_strips': False, 'cross_alt_offset_m': 0.0})
    assert not any('交叉航帶' in a for a in off.stats['assumptions'])


def test_compute_passes_the_cross_strip_setting_through(monkeypatch):
    seen: list = []
    _synthetic_report_patch(monkeypatch, {'H1': lawnmower(-20.0, 620.0, -60.0, 460.0)}, SMALL_POLY, seen)
    hv, sw = _fake(sw__hv_mapping_setup={'camera': HQ6, 'effective': dict(CAM_EFF, cross_strips=True,
                                                                          cross_alt_offset_m=0.0)})
    pv = compute_hdmap_preview(hv, sw)
    assert pv.ok and pv.stats['cross_strips'] == {'on': True, 'alt_offset_m': 0.0}
    assert any('Štroner 2021' in a for a in pv.stats['assumptions'])
