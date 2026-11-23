"""航測建圖「預計成果」預覽 —— **規劃預測，非實際成果**。

由上傳那份任務（與上傳閘門同一組生效值，見 :func:`mission.swarm_coordinator.mapping_exposure_report`）模擬
ArduPilot 的距離觸發，得到每張照片的足跡，再算「每格被幾張照片看到」（視數）。本模組把那張視數格網整理成：

* 依視數分級的上色圖（north-up RGBA PNG；視數 0 的格透明；縮圖用 **block-MIN**，破洞不會被縮掉）；
* 預計可成圖範圍（視數 ≥ :data:`K_PRODUCT`，與上傳閘門同一門檻）的外框與其中的洞；
* 內部破洞／邊緣帶視數不足（與驗證器的 ``region`` 同一分法）。**「會不會擋上傳」只用上傳閘門那個格距
  （:data:`mission.swarm_coordinator.MAPPING_EXPOSURE_CELL_M`）的結果說**：預覽格較細時另跑一次閘門格，
  閘門格的破洞才標「會擋上傳」；只在預覽細格看到的另標「閘門沒擋，仍建議檢查」（格點量化讓兩邊可能不同）；
* 模擬拍照點（抽樣 ≤ :data:`PREVIEW_MAX_POINTS`；抽樣時圖例寫出「只畫 N／M 張」）；
* 給人看的統計：GSD、預估正射／DSM 解析度、張數與資料量（範圍＋假設）、ODM RAM 級距、面積、重疊、
  文獻精度（**只列文獻數字與其條件**；我們自己的精度要等實飛＋檢核點）、看得見哪些地物。

誠實規則（使用者決策層級，不可違反）：

* 不重抽、不抓 Google／底圖瓦片去假裝「預測正射影像」：上色圖只有視數，沒有影像內容；
* 一律標「規劃預測，非實際成果」；驗證器是樂觀模型（不建模 CAM_MAX_ROLL 與轉彎動態，見
  :data:`mission.swarm_coordinator.MAPPING_EXPOSURE_SEAM_CAVEAT`），統計的 ``assumptions`` 一定寫出來；
* 資料量只給範圍，並寫明每張幾 MB 的假設：本相機像素數 × 0.6–1.0 B/px（未經實測；IMX477 12.33 MP 即
  7.4–12.3 MB），像素數未知時退回 7.4–12.3 MB；
* 規劃 DEM 是 Cesium World Terrain 時**不產生**山影（授權：只放記憶體、不可存檔）。

不 import Qt；只用 numpy／shapely／rasterio／PIL／matplotlib（Agg）。
"""
from __future__ import annotations

import base64
import io
import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from mission.swarm_coordinator import (
    MAPPING_EXPOSURE_CELL_M,
    MAPPING_EXPOSURE_MIN_VIEWS,
    MAPPING_EXPOSURE_SEAM_CAVEAT,
)

__all__ = [
    "K_PRODUCT",
    "HQ_VIEWS",
    "QUALITY_CLASSES",
    "PREVIEW_MAX_POINTS",
    "PREVIEW_MAX_PX",
    "PREVIEW_MAX_CELLS",
    "DATA_MB_PER_PHOTO",
    "DATA_BYTES_PER_PX",
    "POINT_COLORS",
    "VISIBLE_FEATURES",
    "PREVIEW_LABEL",
    "PREVIEW_BANNER",
    "HdmapPreview",
    "choose_cell_size_m",
    "estimate_cells",
    "data_mb_per_photo",
    "hole_count_text",
    "odm_ram_tier",
    "build_hdmap_preview",
    "compute_hdmap_preview",
    "render_preview_map_png",
]

_log = logging.getLogger(__name__)

#: 可成圖門檻（視數）——與上傳閘門同一個常數（k=5）；改那邊這裡跟著變
K_PRODUCT = MAPPING_EXPOSURE_MIN_VIEWS
#: 高品質門檻（視數）——與閘門摘要的 ks=(5, 9) 同一個 9
HQ_VIEWS = 9
#: (最少視數, 最多視數或 None, 顏色或 None, 圖例文字)
QUALITY_CLASSES = [
    (0, 0, None, '未拍到'),
    (1, 1, '#e53935', '只有 1 張（不會成圖）'),
    (2, 4, '#fb8c00', '2–4 張（邊緣，可能缺漏或扭曲）'),
    (5, 8, '#c0ca33', '5–8 張（可成圖）'),
    (9, None, '#43a047', '≥9 張（高品質）'),
]
#: 模擬拍照點抽樣上限（全隊合計）
PREVIEW_MAX_POINTS = 5000
#: 上色圖每邊最多像素（超過就 block-MIN 縮圖）
PREVIEW_MAX_PX = 2048
#: 預覽格網格數上限（約；超過就把格距放大）
PREVIEW_MAX_CELLS = 4_000_000
#: JPEG 每像素位元組的假設（B/px）。來源 runs/artifacts/HDMAP_FEASIBILITY_2026-10-14/phase2_verify.md R5：
#: 12.33 MP × 0.6／1.0 B/px，**未經實測**（與別處用的「5 MB/張」也不一致）。每張 MB＝本相機像素數 × 這個範圍
DATA_BYTES_PER_PX = (0.6, 1.0)
#: 相機像素數未知時的退路（MB／張）＝ IMX477 12.33 MP × :data:`DATA_BYTES_PER_PX`
DATA_MB_PER_PHOTO = (7.4, 12.3)
DATA_SIZE_SOURCE = 'runs/artifacts/HDMAP_FEASIBILITY_2026-10-14/phase2_verify.md（R5）'
#: 模擬拍照點的各機顏色（避開分級用的紅／橙／黃綠／綠）
POINT_COLORS = ['#1e88e5', '#8e24aa', '#00acc1', '#d81b60', '#5e35b1', '#6d4c41', '#3949ab', '#546e7a']
#: 看得見哪些地物（名稱, 尺寸 cm）：跨幾個像素 = 尺寸 ÷ 中位 GSD
VISIBLE_FEATURES = [('車道線（寬）', 15), ('交通錐（底座）', 30), ('人孔蓋', 60), ('機車（長）', 180),
                    ('小客車（長）', 450)]
PREVIEW_LABEL = '預計成果（規劃預測）'
PREVIEW_BANNER = '規劃預測，非實際成果'

#: 未知值時也要有的統計鍵（全部存在；不知道就 None）
_STATS_KEYS = (
    'uav_labels', 'drone_ids', 'camera', 'agl_m', 'gsd_cm', 'ortho_resolution_cm', 'dsm_resolution_cm_est',
    'photos_per_uav', 'photos_total', 'data_gb_range', 'odm_ram_tier', 'area_km2', 'frac_ge',
    'forward_overlap_p05', 'side_overlap_p05', 'seam_overlap_min', 'interior_hole_count', 'interior_hole_area_m2',
    'edge_band_m', 'edge_fringe_note', 'cell_size_m', 'k_product', 'assumptions', 'accuracy_literature',
    'accuracy_note', 'visible_features', 'datum', 'elapsed_s',
)

#: 文獻精度：**只列文獻數字與其條件**（docs/評估_2026-10-14_群飛航測高精地圖.md §7、
#: docs/比較_2026-10-15_交叉航帶vs加密GCP.md §6）；不是本計畫的預測
ACCURACY_LITERATURE = [
    {'case': '相對精度（模型內部）', 'horizontal': '1–3×GSD', 'vertical': None,
     'condition': '一般文獻值；只代表模型內部一致性，不是絕對位置'},
    {'case': '絕對精度，只用飛控 GPS', 'horizontal': '2–6 m', 'vertical': None,
     'condition': '沒有地面控制點、沒有 PPK；飛控 GPS 本身是公尺級'},
    {'case': '絕對精度，加地面控制點', 'horizontal': '約 2.5×GSD', 'vertical': '依地形與航線（見下兩列）',
     'condition': '有地面控制點；高程受地形起伏與航線設計影響大'},
    {'case': 'Gerke & Przybilla 2016（平地）', 'horizontal': None, 'vertical': 'Z RMSE 16.5 → 2.4 cm',
     'condition': '平地、4 個 GCP、無 RTK；加一組換方向又換高度（75 對 105 m）、約 20% 面積的交叉航帶；GSD 約 2.7 cm'},
    {'case': 'Gerke & Przybilla 2016（起伏大的料堆區）', 'horizontal': None,
     'vertical': '4 個 GCP 約 20 cm；18 個 GCP 約 8 cm',
     'condition': '起伏大的料堆區；同一組交叉航帶在這裡沒有顯著效果'},
    {'case': '同高度垂直交叉（Štroner 2021）', 'horizontal': None, 'vertical': '只有小幅改善',
     'condition': '只換方向、不換高度（同高度垂直雙網格）；上面 Gerke 的 16.5 → 2.4 cm 是換方向＋換高度，'
                  '不能套用到同高度交叉；沒有文獻支持同高度交叉能取代加密 GCP'},
    {'case': 'PPK＋1 個控制點＋交叉航帶', 'horizontal': '2–5 cm', 'vertical': '2–10 cm',
     'condition': '文獻值；我們的硬體未驗證'},
]
ACCURACY_NOTE = ('我們自己的精度目前未知：要等實飛並以獨立檢核點量測後才能宣告（ASPRS 第 2 版：≥30 個檢核點，'
                 '檢核點本身至少比產品精確 2 倍）。上表是文獻在其各自條件下量到的數字，不是本計畫的預測'
                 '（docs/比較_2026-10-15_交叉航帶vs加密GCP.md §6、docs/評估_2026-10-14_群飛航測高精地圖.md §7）。')

# WGS84（與 core.recon.exposure_verifier.LocalFrame 同式）
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563


def _m_per_deg(lat_deg: float) -> tuple[float, float]:
    """(每經度幾公尺, 每緯度幾公尺)，WGS84 卯酉圈／子午圈曲率半徑（與驗證器的本地切平面同式）。"""
    e2 = _WGS84_F * (2.0 - _WGS84_F)
    s = math.sin(math.radians(lat_deg))
    w = 1.0 - e2 * s * s
    m = _WGS84_A * (1.0 - e2) / w ** 1.5
    n = _WGS84_A / math.sqrt(w)
    return n * math.cos(math.radians(lat_deg)) * math.pi / 180.0, m * math.pi / 180.0


# ---------------------------------------------------------------------------
# 資料類
# ---------------------------------------------------------------------------
def _empty_stats() -> dict:
    return {k: None for k in _STATS_KEYS}


def _raster_note(cell_m: float, fac: int) -> str:
    return (f'上色圖每像素＝{int(fac)}×{int(fac)} 格取最少視數（{cell_m * fac:g} m，保守顯示；'
            f'統計仍以 {cell_m:g} m 格計）')


def hole_count_text(stats: Mapping | None) -> str:
    """「內部破洞 N 處（依據）」——地圖圖例的摘要行與 MainWindow 狀態字共用（依據＝哪個格距判定）。

    ``interior_hole_basis``：``'gate'``（上傳閘門那個格距：與閘門同一判定）、``'gate_summary'``（只有規劃時的閘門摘要）、
    ``'preview'``（閘門結果未知，只有預覽格）；沒有這個鍵（舊／合成的統計）⇒ 只寫個數。
    """
    s = stats or {}
    n = s.get('interior_hole_count')
    n_txt = '—' if n is None else f'{int(n)}'
    basis, gc, pc = s.get('interior_hole_basis'), s.get('gate_cell_size_m'), s.get('cell_size_m')
    po = int(s.get('preview_only_hole_count') or 0)
    if basis in ('gate', 'gate_summary') and gc is not None:
        out = f'內部破洞 {n_txt} 處（上傳閘門 {float(gc):g} m 格）'
        if po and pc is not None:
            out += f'，預覽 {float(pc):g} m 格另見 {po} 處（閘門沒擋，建議檢查）'
        return out
    if basis == 'preview' and pc is not None:
        return f'內部視數不足 {n_txt} 處（預覽 {float(pc):g} m 格；上傳閘門結果未知，以閘門為準）'
    return f'內部破洞 {n_txt} 處'


def _default_legend(cell_m: float | None = None, *, gate_cell_m: float | None = None, gate_known: bool = False,
                    preview_only: int = 0, gate_markers: int = 0, disagrees: bool = False, raster_fac: int = 1,
                    photos_shown: int | None = None, photos_all: int | None = None) -> dict:
    notes = [f'{PREVIEW_BANNER}：由模擬拍照位置推算每格被幾張照片看到（視數），沒有影像內容',
             f'未上色＝未拍到；白框＝預計可成圖範圍（≥{K_PRODUCT} 張，含足跡伸出測區外的部分）']
    if gate_known and gate_cell_m is not None:
        notes.append(f'紅框＝內部破洞（<{K_PRODUCT} 張；上傳閘門 {gate_cell_m:g} m 格判定，會擋上傳）')
        if preview_only:
            notes.append(f'紅點框＝只在預覽 {cell_m:g} m 細格看到的內部視數不足（上傳閘門沒擋，仍建議檢查）'
                         if cell_m is not None else '紅點框＝只在預覽細格看到的內部視數不足（上傳閘門沒擋，仍建議檢查）')
        if gate_markers:
            notes.append('紅圓圈＝上傳閘門回報的內部破洞中心（形狀以閘門為準）')
    else:
        notes.append(f'紅框＝內部視數不足（<{K_PRODUCT} 張）；能不能上傳以上傳閘門'
                     + (f'（{gate_cell_m:g} m 格）' if gate_cell_m is not None else '') + '為準')
    notes.append('橘框＝邊緣帶視數不足（不擋上傳）')
    if disagrees:
        notes.append('⚠ 預覽格與上傳閘門格看到的內部破洞不同（格點量化）：能不能上傳以上傳閘門為準')
    notes.append('樂觀模型：未建模 CAM_MAX_ROLL 側傾門檻與轉彎動態，有風時實際可能較差')
    if cell_m is not None:
        notes.append(f'格 {cell_m:g} m（視數以格心計，量化約 ±1 格）')
        if raster_fac > 1:
            notes.append(_raster_note(cell_m, raster_fac))
    if photos_shown is not None and photos_all is not None and photos_shown < photos_all:
        notes.append(f'拍照點只畫 {photos_shown:,}／{photos_all:,} 張（均勻抽樣；張數統計用全部）')
    # 地圖圖例（ui/widgets/hdmap_preview_layer）上面已固定印「預計成果（規劃預測）」與橫幅 ⇒ 標題改說顏色是什麼，
    # 不再把那兩行重複一次；結尾仍是橫幅（只拿標題去用的地方也看得到「規劃預測」）
    return {'title': f'顏色＝每格預計被幾張照片拍到（視數）——{PREVIEW_BANNER}',
            'classes': [{'label': lab, 'color': col} for _lo, _hi, col, lab in QUALITY_CLASSES if col],
            'notes': notes}


@dataclass
class HdmapPreview:
    """預計成果（規劃預測）。``ok=False`` 時只有 ``reason``（繁中）有意義，其餘是空值。"""
    ok: bool
    reason: str | None = None
    bounds_wsen: tuple = (0.0, 0.0, 0.0, 0.0)       # 上色圖的**格邊**（度）：(W, S, E, N)
    image_png: bytes = b''                          # north-up RGBA PNG；視數 0 透明
    image_px: tuple = (0, 0)                        # (寬, 高)
    cell_size_m: float = 0.0                        # 上色圖一格（縮圖後）
    extent_rings: list = field(default_factory=list)
    extent_holes: list = field(default_factory=list)
    interior_holes: list = field(default_factory=list)
    edge_short: list = field(default_factory=list)
    survey_rings: list = field(default_factory=list)
    photos: list = field(default_factory=list)      # [[lat, lon, uav_index], ...]
    stats: dict = field(default_factory=_empty_stats)
    legend: dict = field(default_factory=_default_legend)
    hillshade_png: bytes | None = None
    hillshade_bounds_wsen: tuple | None = None

    @classmethod
    def failed(cls, reason: str, *, elapsed_s: float | None = None) -> "HdmapPreview":
        st = _empty_stats()
        st['k_product'] = K_PRODUCT
        st['elapsed_s'] = None if elapsed_s is None else round(float(elapsed_s), 3)
        return cls(ok=False, reason=str(reason), stats=st)

    def stats_line(self) -> str:
        """一行摘要（地圖圖例用）。面積用**測區內**的值（與狀態字同一個數）；整張格網的值（含足跡伸出測區外、
        就是白框圍的範圍）另外標明，免得讀成「覆蓋率 >100%」。"""
        if not self.ok:
            return f'{PREVIEW_LABEL}：無法預覽（{self.reason}）'
        s = self.stats
        a = s.get('area_km2') or {}
        g = s.get('gsd_cm') or {}
        d = s.get('data_gb_range') or [None, None]
        if a.get('product_in_survey') is not None:
            area_txt = (f"測區內可成圖 {_fmt(a.get('product_in_survey'), '.2f')} km²"
                        f"（測區 {_fmt(a.get('survey'), '.2f')} km²；白框含測區外足跡 {_fmt(a.get('product'), '.2f')} km²）"
                        f"，≥{HQ_VIEWS} 張 {_fmt(a.get('high_quality_in_survey'), '.2f')} km²")
        else:
            area_txt = (f"可成圖 {_fmt(a.get('product'), '.2f')} km²（含測區外足跡）"
                        f"，≥{HQ_VIEWS} 張 {_fmt(a.get('high_quality'), '.2f')} km²")
        mb = s.get('data_mb_per_photo')
        mb_txt = (f"，每張 {_fmt(mb[0], 'g')}–{_fmt(mb[1], 'g')} MB 的假設"
                  if isinstance(mb, (list, tuple)) and len(mb) == 2 else '')
        parts = [f"{PREVIEW_LABEL}：{area_txt}",
                 f"照片 {s.get('photos_total') or 0:,} 張（約 {_fmt(d[0], '.1f')}–{_fmt(d[1], '.1f')} GB{mb_txt}）",
                 f"GSD 中位 {_fmt(g.get('median'), '.1f')} cm",
                 hole_count_text(s)]
        return '，'.join(parts) + f'（{PREVIEW_BANNER}）'

    def to_js_payload(self, *, opacity: float = 0.45, mask_alpha: float = 0.55, colors: list[str] | None = None,
                      layer_id: str = 'hdmap_plan') -> dict:
        """地圖圖層用的純 JSON 資料（``json.dumps(..., allow_nan=False)`` 一定成功）。"""
        op = float(opacity)
        ma = float(mask_alpha)
        if not (math.isfinite(op) and 0.0 <= op <= 1.0):
            raise ValueError(f'opacity 須在 0–1，得 {opacity!r}')
        if not (math.isfinite(ma) and 0.0 <= ma <= 1.0):
            raise ValueError(f'mask_alpha 須在 0–1，得 {mask_alpha!r}')
        cols = list(colors) if colors else list(POINT_COLORS)
        if not cols:
            raise ValueError('colors 不可為空')
        labels = list((self.stats or {}).get('uav_labels') or [])
        legend = dict(self.legend or _default_legend())
        legend['points'] = [{'label': lab, 'color': cols[i % len(cols)]} for i, lab in enumerate(labels)]
        image = ('data:image/png;base64,' + base64.b64encode(self.image_png).decode('ascii')) if self.image_png else ''
        out = {
            'id': str(layer_id),
            'label': PREVIEW_LABEL,
            'bounds': [float(v) for v in self.bounds_wsen],
            'image': image,
            'image_px': [int(self.image_px[0]), int(self.image_px[1])],
            'opacity': op,
            'mask_alpha': ma,
            'extent_rings': self.extent_rings,
            'extent_holes': self.extent_holes,
            'holes': [{'ring': h['ring'], 'label': h.get('label') or _hole_label(h, 'interior'),
                       'kind': _HOLE_KIND.get(h.get('source'), 'hole')} for h in self.interior_holes],
            'edge_short': [{'ring': h['ring'], 'label': h.get('label') or _hole_label(h, 'edge_band')}
                           for h in self.edge_short],
            'survey_rings': self.survey_rings,
            'points': [[p[0], p[1], int(p[2]) % len(cols)] for p in self.photos],
            'point_colors': cols,
            'legend': legend,
            'stats_line': self.stats_line(),
        }
        return _json_safe(out)


def _fmt(v, spec: str) -> str:
    return '—' if v is None else format(float(v), spec)


#: interior_holes 的 ``source`` → payload 的 ``kind``（地圖用的畫法；ui.widgets.hdmap_preview_layer.HOLE_KINDS）
_HOLE_KIND = {'gate': 'hole', 'preview_grid': 'hole', None: 'hole', 'preview_only': 'preview_only',
              'gate_centroid': 'marker'}


def _hole_label(h: Mapping, region: str, *, cell_m: float | None = None, gate_cell_m: float | None = None,
                gate_txt: str | None = None) -> str:
    """破洞／邊緣帶的說明字。``h['source']``（build_hdmap_preview 填）決定「會不會擋上傳」怎麼說：
    只有上傳閘門那個格距的結果（'gate'／'gate_centroid'）才說「會擋上傳」。"""
    a, mv = float(h.get('area_m2') or 0.0), h.get('min_views')
    if region != 'interior':
        return f'邊緣帶視數不足 {a:,.0f} m²（最少 {mv} 張，不擋上傳）'
    src = h.get('source')
    gc = f'{float(gate_cell_m):g} m 格' if gate_cell_m is not None else '格'
    pc = f'{float(cell_m):g} m 格' if cell_m is not None else '細格'
    if src == 'gate':
        return f'內部破洞 {a:,.0f} m²（上傳閘門 {gc}：最少 {mv} 張，會擋上傳）'
    if src == 'gate_centroid':
        return (f'上傳閘門（{gc}）回報的內部破洞約 {a:,.0f} m²（最少 {mv} 張，會擋上傳）；'
                '圓圈只標中心位置，形狀以閘門為準')
    if src == 'preview_only':
        return (f'內部視數不足 {a:,.0f} m²（最少 {mv} 張）：只在預覽 {pc}看到，上傳閘門（{gc}）沒擋——仍建議檢查')
    if src == 'preview_grid':
        return (f'內部視數不足 {a:,.0f} m²（預覽 {pc}，最少 {mv} 張）；'
                + (gate_txt or '上傳閘門結果未知') + '——能不能上傳以閘門為準')
    return f'內部破洞 {a:,.0f} m²（最少 {mv} 張；能不能上傳以上傳閘門為準）'


def _json_safe(v):
    """numpy 純量 → Python；非有限浮點 → None；tuple → list；dict 鍵 → str。"""
    if isinstance(v, dict):
        return {str(k): _json_safe(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_json_safe(x) for x in v]
    if isinstance(v, np.ndarray):
        return [_json_safe(x) for x in v.tolist()]
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else None
    return v


# ---------------------------------------------------------------------------
# 小工具（可單獨測）
# ---------------------------------------------------------------------------
def _poly_bbox_m(polygons: Sequence[Mapping]) -> tuple[float, float]:
    """測區外接框（寬 m, 高 m）。polygons：verify_preflight 吃的 [{'exterior': [(lat, lon), …], …}, …]。"""
    pts = np.array([p for poly in polygons for p in poly['exterior']], dtype=float)
    if pts.size == 0:
        raise ValueError('沒有測區多邊形')
    kx, ky = _m_per_deg(float((pts[:, 0].min() + pts[:, 0].max()) / 2.0))
    return float(np.ptp(pts[:, 1]) * kx), float(np.ptp(pts[:, 0]) * ky)


def estimate_cells(polygons: Sequence[Mapping], cell_size_m: float, *, margin_m: float = 300.0) -> float:
    """格距 ``cell_size_m`` 時驗證器格網約幾格：測區外接框外擴 ``margin_m``（足跡伸出測區）÷ 格面積
    （與 :func:`choose_cell_size_m` 同一個估法）。"""
    w, h = _poly_bbox_m(polygons)
    c = float(cell_size_m)
    return (w + 2.0 * margin_m) * (h + 2.0 * margin_m) / (c * c)


def _sig3(x: float) -> float:
    return float(f'{float(x):.3g}')


def data_mb_per_photo(camera_effective: Mapping | None) -> tuple[float, float, float | None]:
    """每張照片的資料量範圍（MB）＝本相機像素數 × :data:`DATA_BYTES_PER_PX`（取 3 位有效數字）→ (低, 高, MP)。
    像素數未知 ⇒ 退回 :data:`DATA_MB_PER_PHOTO`、MP 給 None。IMX477（4056×3040＝12.33 MP）正好是 7.4–12.3 MB。"""
    cam = camera_effective or {}
    w, h = cam.get('width_px'), cam.get('height_px')
    ok = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v)) and float(v) > 0
             for v in (w, h))
    if not ok:
        return float(DATA_MB_PER_PHOTO[0]), float(DATA_MB_PER_PHOTO[1]), None
    mp = float(w) * float(h) / 1e6
    return _sig3(DATA_BYTES_PER_PX[0] * mp), _sig3(DATA_BYTES_PER_PX[1] * mp), mp


def choose_cell_size_m(polygons: Sequence[Mapping], *, margin_m: float = 300.0,
                       max_cells: int = PREVIEW_MAX_CELLS) -> float:
    """預設格距：測區外接框 ≤ 4 km² → 2 m，否則 4 m；連外擴 ``margin_m``（足跡伸出測區）估算的格數超過
    ``max_cells`` 時再放大（取到 0.5 m）。"""
    w, h = _poly_bbox_m(polygons)
    cell = 2.0 if w * h <= 4.0e6 else 4.0
    full = (w + 2.0 * margin_m) * (h + 2.0 * margin_m)
    if full / (cell * cell) > max_cells:
        cell = math.ceil(math.sqrt(full / max_cells) * 2.0) / 2.0
    return float(cell)


def odm_ram_tier(n_photos: int) -> str:
    """依 ODM 的 RAM 表（評估 §8 引用：500 張 32 GB、1,500 張 64 GB、2,500 張 128 GB）給級距。"""
    n = int(n_photos or 0)
    if n <= 0:
        return '沒有照片'
    if n <= 500:
        return '≤ 32 GB（ODM RAM 表：500 張 32 GB）'
    if n <= 1500:
        return '約 64 GB（ODM RAM 表：500–1,500 張的級距）'
    if n <= 2500:
        return '約 128 GB（ODM RAM 表：1,500–2,500 張的級距）'
    return (f'超過 ODM RAM 表最大級距（2,500 張 128 GB；本計畫 {n:,} 張）⇒ 需要 >128 GB 記憶體，'
            '或用 --split 分塊處理')


def _class_index(views: np.ndarray) -> np.ndarray:
    """視數 → QUALITY_CLASSES 的索引（uint8）。"""
    v = np.asarray(views)
    out = np.zeros(v.shape, dtype=np.uint8)
    for i, (lo, hi, _c, _l) in enumerate(QUALITY_CLASSES):
        sel = v >= lo if hi is None else (v >= lo) & (v <= hi)
        out[sel] = i
    return out


def _hex_rgb(c: str) -> tuple[int, int, int]:
    c = c.lstrip('#')
    return int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16)


def _block_min(views: np.ndarray, f: int) -> np.ndarray:
    """f×f 區塊取最小值（縮圖時破洞不會被平均掉）。尾端不足 f 的部分補 0（保守：不會把沒驗證過的格說成有覆蓋）。"""
    f = int(f)
    if f <= 1:
        return np.asarray(views)
    v = np.asarray(views)
    ny, nx = v.shape
    py, px = (-ny) % f, (-nx) % f
    if py or px:
        v = np.pad(v, ((0, py), (0, px)), mode='constant', constant_values=0)
    ny2, nx2 = v.shape
    return v.reshape(ny2 // f, f, nx2 // f, f).min(axis=(1, 3))


def _views_png(views_south_up: np.ndarray) -> bytes:
    """視數格網（第 0 列在南）→ north-up RGBA PNG；視數 0 透明。"""
    from PIL import Image
    lut = np.zeros((len(QUALITY_CLASSES), 4), dtype=np.uint8)
    for i, (_lo, _hi, col, _lab) in enumerate(QUALITY_CLASSES):
        if col:
            lut[i, :3] = _hex_rgb(col)
            lut[i, 3] = 255
    rgba = lut[_class_index(views_south_up)[::-1]]
    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgba)).save(buf, format='PNG')   # (h, w, 4) uint8 ⇒ RGBA
    return buf.getvalue()


def _ll_ring(frame, xs, ys) -> list:
    lat, lon = frame.to_latlon(np.asarray(xs, dtype=float), np.asarray(ys, dtype=float))
    return [[round(float(a), 7), round(float(b), 7)] for a, b in zip(np.atleast_1d(lat), np.atleast_1d(lon))]


def _extent_polygons(mask: np.ndarray, x0: float, y0: float, cell: float, frame, simplify_m: float
                     ) -> tuple[list, list]:
    """bool 格網（第 0 列在南，原點 (x0, y0) 是格邊）→ (外環 [[lat, lon], …] 的串列, 內環的串列)。"""
    if not mask.any():
        return [], []
    import shapely
    import shapely.geometry
    from rasterio import features as rio_features
    from rasterio.transform import Affine
    tr = Affine(cell, 0.0, x0, 0.0, cell, y0)
    outers, inners = [], []
    for geo, _val in rio_features.shapes(mask.astype(np.uint8), mask=mask, transform=tr, connectivity=4):
        g = shapely.geometry.shape(geo)
        if simplify_m > 0:
            g = g.simplify(simplify_m, preserve_topology=True)
        for part in getattr(g, 'geoms', [g]):
            if part.is_empty or part.geom_type != 'Polygon' or part.area <= 0:
                continue
            ex = np.asarray(part.exterior.coords)
            outers.append(_ll_ring(frame, ex[:, 0], ex[:, 1]))
            for h in part.interiors:
                hc = np.asarray(h.coords)
                inners.append(_ll_ring(frame, hc[:, 0], hc[:, 1]))
    return outers, inners


def _area_rings(areas, frame, simplify_m: float, limit: int, *, source: str | None = None,
                cell_m: float | None = None, label: Callable[[dict], str] | None = None) -> list:
    """UnderCoveredArea 的串列 → [{'ring', 'area_m2', 'min_views'(, 'source', 'cell_m', 'blocks_upload', 'label')}]
    （依面積大到小，最多 limit 個）。``source``：這一處是哪個格網判定的（見 :func:`_hole_label`）。"""
    out = []
    for a in list(areas)[:limit]:
        ring = None
        g = getattr(a, 'geometry_enu', None)
        if g is not None and frame is not None:
            gs = g.simplify(simplify_m, preserve_topology=True) if simplify_m > 0 else g
            if not gs.is_empty and gs.geom_type == 'Polygon':
                ex = np.asarray(gs.exterior.coords)
                ring = _ll_ring(frame, ex[:, 0], ex[:, 1])
        if ring is None:
            ring = [[round(float(la), 7), round(float(lo), 7)] for la, lo in a.exterior_latlon]
        h = {'ring': ring, 'area_m2': round(float(a.area_m2), 1), 'min_views': int(a.min_views)}
        if source is not None:
            h.update(source=source, cell_m=(None if cell_m is None else float(cell_m)),
                     blocks_upload={'gate': True, 'preview_only': False}.get(source))
        if label is not None:
            h['label'] = label(h)
        out.append(h)
    return out


def _hole_geom(a, frame):
    """UnderCoveredArea → 本地 ENU 的 shapely 幾何（驗證器有給 geometry_enu 就用它；兩個格距的報告用同一個切平面）。"""
    import shapely
    g = getattr(a, 'geometry_enu', None)
    if g is not None and not g.is_empty:
        return g
    ll = np.asarray(getattr(a, 'exterior_latlon', None) or [], dtype=float).reshape(-1, 2)
    if len(ll) >= 3:
        x, y = frame.to_xy(ll[:, 0], ll[:, 1])
        return shapely.Polygon(np.column_stack([x, y])).buffer(0)
    x, y = frame.to_xy(*a.centroid_latlon)
    return shapely.Point(float(x), float(y))


def _match_holes(fine, coarse, frame, dist_m: float) -> tuple[set, set]:
    """兩個格距的內部破洞互相對應（距離 ≤ ``dist_m`` 算同一處）→ (fine 裡沒對應的索引, coarse 裡沒對應的索引)。"""
    import shapely
    if not fine or not coarse:
        return set(range(len(fine))), set(range(len(coarse)))
    fg = [_hole_geom(a, frame).buffer(float(dist_m)) for a in fine]
    cg = [_hole_geom(a, frame) for a in coarse]
    pairs = shapely.STRtree(cg).query(fg, predicate='intersects')
    return (set(range(len(fine))) - {int(i) for i in pairs[0]},
            set(range(len(coarse))) - {int(j) for j in pairs[1]})


def _marker_ring(frame, lat: float, lon: float, radius_m: float, n: int = 16) -> list:
    """中心 (lat, lon)、半徑 radius_m 的圓（只標位置用）→ [[lat, lon], …]。"""
    cx, cy = frame.to_xy(float(lat), float(lon))
    t = np.linspace(0.0, 2.0 * math.pi, int(n), endpoint=False)
    return _ll_ring(frame, float(cx) + radius_m * np.cos(t), float(cy) + radius_m * np.sin(t))


def _ground_heights(lat: np.ndarray, lon: np.ndarray, terrain: Any, dem: Any) -> np.ndarray:
    """照片天底點地面高（與驗證器同一個地面）。有 DEM 就向量化取樣（與閘門的 ``_ground`` 同式），否則逐點呼叫。"""
    if dem is not None and hasattr(dem, 'sample_msl'):
        return np.asarray(dem.sample_msl(lat, lon), dtype=float).reshape(lat.shape)
    if isinstance(terrain, (int, float)):
        return np.full(lat.shape, float(terrain))
    if callable(terrain):
        return np.array([float(terrain(a, b)) for a, b in zip(lat, lon)], dtype=float)
    return np.full(lat.shape, np.nan)


def _stat3(v: np.ndarray, nd: int) -> dict:
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {'min': None, 'median': None, 'max': None}
    return {'min': round(float(v.min()), nd), 'median': round(float(np.median(v)), nd), 'max': round(float(v.max()), nd)}


def _hillshade(dem: Any, bounds_wsen: tuple, *, max_px: int = 512) -> tuple[bytes, tuple] | None:
    """規劃 DEM（DemGrid：經緯規則網格，第 0 列在南）在 bounds 範圍的粗山影 → (north-up RGBA PNG, 格邊 bounds)。
    範圍外、沒有有限值，或起伏 < 0.5 m（平地：山影沒有資訊）⇒ None。"""
    need = ('z_msl', 'lat0_deg', 'lon0_deg', 'dlat_deg', 'dlon_deg')
    if dem is None or not all(hasattr(dem, k) for k in need):
        return None
    from matplotlib.colors import LightSource
    from PIL import Image
    W, S, E, N = (float(v) for v in bounds_wsen)
    z = np.asarray(dem.z_msl, dtype=float)
    lat0, lon0, dlat, dlon = float(dem.lat0_deg), float(dem.lon0_deg), float(dem.dlat_deg), float(dem.dlon_deg)
    i0 = max(int(math.floor((S - lat0) / dlat)) - 1, 0)
    i1 = min(int(math.ceil((N - lat0) / dlat)) + 1, z.shape[0] - 1)
    j0 = max(int(math.floor((W - lon0) / dlon)) - 1, 0)
    j1 = min(int(math.ceil((E - lon0) / dlon)) + 1, z.shape[1] - 1)
    if i1 - i0 < 2 or j1 - j0 < 2:
        return None
    stride = max(1, int(math.ceil(max(i1 - i0 + 1, j1 - j0 + 1) / max_px)))
    sub = z[i0:i1 + 1:stride, j0:j1 + 1:stride]
    fin = np.isfinite(sub)
    if sub.shape[0] < 2 or sub.shape[1] < 2 or not fin.any():
        return None
    if float(np.nanmax(sub) - np.nanmin(sub)) < 0.5:
        return None
    nr, nc = sub.shape
    lat_last = lat0 + (i0 + (nr - 1) * stride) * dlat
    lon_last = lon0 + (j0 + (nc - 1) * stride) * dlon
    kx, ky = _m_per_deg((lat0 + i0 * dlat + lat_last) / 2.0)
    zf = np.where(fin, sub, float(np.nanmean(sub)))[::-1]          # north-up（matplotlib 影像慣例：第 0 列在上）
    shade = LightSource(azdeg=315.0, altdeg=45.0).hillshade(zf, vert_exag=3.0, dx=dlon * stride * kx,
                                                            dy=dlat * stride * ky)
    g = np.clip(np.asarray(shade, dtype=float) * 255.0, 0, 255).astype(np.uint8)
    rgba = np.empty(g.shape + (4,), dtype=np.uint8)
    rgba[..., 0] = rgba[..., 1] = rgba[..., 2] = g
    rgba[..., 3] = np.where(fin[::-1], 255, 0).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(rgba).save(buf, format='PNG')
    hd, hw = dlat * stride / 2.0, dlon * stride / 2.0
    bounds = (lon0 + j0 * dlon - hw, lat0 + i0 * dlat - hd, lon_last + hw, lat_last + hd)
    return buf.getvalue(), tuple(float(v) for v in bounds)


# ---------------------------------------------------------------------------
# 主體：報告 → 預覽
# ---------------------------------------------------------------------------
def build_hdmap_preview(rep, *, survey_polygons: Sequence[Mapping], camera_effective: Mapping,
                        terrain: Any = None, dem: Any = None, drone_of: Mapping[str, int] | None = None,
                        wind: tuple | None = None, wind_source: str | None = None,
                        cam_max_roll_deg: float | None = None, datum: str | None = None,
                        is_world_terrain: bool = False, gate: Mapping | None = None,
                        gate_report: Any = None, gate_holes_summary: Sequence[Mapping] | None = None,
                        gate_note: str | None = None, cross: Mapping | None = None,
                        max_px: int = PREVIEW_MAX_PX, max_points: int = PREVIEW_MAX_POINTS,
                        t_start: float | None = None) -> HdmapPreview:
    """曝光報告（:func:`core.recon.exposure_verifier.verify_preflight` 的輸出，要帶 ``grid``）→ :class:`HdmapPreview`。

    ``camera_effective``：``build_mapping_setup()['effective']``（焦距、像素間距、影像寬高、preset）。
    ``terrain``／``dem``：與驗證器同一個地面（算 AGL／GSD）；``dem`` 是 DemGrid 時順便做粗山影
    （``is_world_terrain=True`` 時一律不做：授權只准放記憶體）。

    內部破洞「會不會擋上傳」只用上傳閘門那個格距（:data:`MAPPING_EXPOSURE_CELL_M`）的結果：
    ``rep`` 本身就是閘門格 ⇒ 用它；否則 ``gate_report``（同一組輸入、閘門格重跑的報告）；都沒有 ⇒ ``gate``
    （規劃時閘門摘要的個數／面積）＋ ``gate_holes_summary``（摘要的前 5 大中心），再沒有就標「閘門結果未知」。
    ``gate_note``：為什麼沒有 ``gate_report``（寫進假設）。``cross``：建圖生效值（``cross_strips``／
    ``cross_alt_offset_m``；精度文獻要挑對條件）。
    """
    from mission.swarm_coordinator import mapping_exposure_assumptions
    t0 = time.perf_counter() if t_start is None else float(t_start)
    grid = getattr(rep, 'grid', None)
    if grid is None:
        return HdmapPreview.failed('曝光報告沒有格網（grid）⇒ 畫不出預計成果', elapsed_s=time.perf_counter() - t0)
    f_mm = camera_effective.get('focal_mm')
    pix_um = camera_effective.get('pixel_um')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v)) and float(v) > 0
               for v in (f_mm, pix_um)):
        return HdmapPreview.failed('沒有相機像素尺寸／焦距 ⇒ 算不出 GSD（請在「航測建圖」填相機）',
                                   elapsed_s=time.perf_counter() - t0)
    f_mm, pix_um = float(f_mm), float(pix_um)
    views = np.asarray(grid.views)
    inside = np.asarray(grid.inside)
    cell = float(grid.cell_size_m)
    frame = grid.frame
    if not (views > 0).any():
        return HdmapPreview.failed('模擬不到任何一張可用的照片（所有拍照距離都是 0，或照片全被排除）⇒ 沒有預計成果',
                                   elapsed_s=time.perf_counter() - t0)

    # ---- 裁到有照片的範圍（外加 1 格）----
    rows = np.nonzero(views.any(axis=1))[0]
    cols = np.nonzero(views.any(axis=0))[0]
    r0, r1 = max(int(rows[0]) - 1, 0), min(int(rows[-1]) + 2, views.shape[0])
    c0, c1 = max(int(cols[0]) - 1, 0), min(int(cols[-1]) + 2, views.shape[1])
    v = views[r0:r1, c0:c1]
    ins = inside[r0:r1, c0:c1]
    x0, y0 = float(grid.x0) + c0 * cell, float(grid.y0) + r0 * cell
    cell_a = cell * cell

    # ---- 面積 ----
    #   product／high_quality：整張格網（含伸出測區的部分）視數 ≥ 門檻的格數 × 格面積；
    #   *_in_survey：測區內（格心在測區內）的格比例 × 測區多邊形面積（不會因格量化超過測區面積）
    prod = v >= K_PRODUCT
    hq = v >= HQ_VIEWS
    n_in = int(inside.sum())
    survey_km2 = float(rep.area_m2) / 1e6
    area = {'survey': survey_km2,
            'product': float(prod.sum()) * cell_a / 1e6,
            'high_quality': float(hq.sum()) * cell_a / 1e6,
            'product_in_survey': (float((prod & ins).sum()) / n_in * survey_km2) if n_in else 0.0,
            'high_quality_in_survey': (float((hq & ins).sum()) / n_in * survey_km2) if n_in else 0.0}
    area = {k: round(x, 6) for k, x in area.items()}          # km²，到 1 m²

    # ---- 可成圖範圍（全解析度多邊形化，約 1 格簡化）----
    ext_rings, ext_holes = _extent_polygons(prod, x0, y0, cell, frame, simplify_m=cell)

    # ---- 上色圖（block-MIN 縮圖）----
    fac = max(1, int(math.ceil(max(v.shape) / float(max_px))))
    vd = _block_min(v, fac)
    png = _views_png(vd)
    rcell = cell * fac
    lat_s, lon_w = frame.to_latlon(x0, y0)
    lat_n, lon_e = frame.to_latlon(x0 + vd.shape[1] * rcell, y0 + vd.shape[0] * rcell)
    bounds = (float(lon_w), float(lat_s), float(lon_e), float(lat_n))

    # ---- 破洞／邊緣帶 ----
    #   「會擋上傳」只用上傳閘門那個格距的結果說（兩個格距的格點量化不同：細破洞可能只有一邊看得到）
    gcell = float(MAPPING_EXPOSURE_CELL_M)
    under = list(getattr(rep, 'undercovered', None) or [])
    pv_holes_all = [a for a in under if a.region == 'interior']
    edge_all = [a for a in under if a.region == 'edge_band']
    edge = _area_rings(edge_all, frame, cell * 0.5, 500)
    if gate_report is None and abs(cell - gcell) <= 1e-9:
        gate_report = rep
    gate = dict(gate) if gate else None
    preview_only_n = gate_only_n = disagrees = None
    markers = 0
    gate_txt = None

    def _lab(h):
        return _hole_label(h, 'interior', cell_m=cell, gate_cell_m=gcell, gate_txt=gate_txt)
    if gate_report is not None:
        basis = 'gate'
        g_holes_all = [a for a in (getattr(gate_report, 'undercovered', None) or []) if a.region == 'interior']
        hole_n = len(g_holes_all)
        hole_area = float(getattr(gate_report, 'undercovered_interior_area_m2', 0.0) or 0.0)
        if gate_report is rep:
            po_idx, go_idx = set(), set()
        else:      # 同一個切平面（驗證器以測區定原點）⇒ 幾何直接比；距離 ≤ 一個閘門格算同一處
            po_idx, go_idx = _match_holes(pv_holes_all, g_holes_all, frame, gcell)
        holes = _area_rings(g_holes_all, frame, gcell * 0.5, 500, source='gate', cell_m=gcell, label=_lab)
        holes += _area_rings([a for i, a in enumerate(pv_holes_all) if i in po_idx], frame, cell * 0.5, 500,
                             source='preview_only', cell_m=cell, label=_lab)
        preview_only_n, gate_only_n = len(po_idx), len(go_idx)
        disagrees = bool(po_idx or go_idx)
    elif gate is not None and gate.get('interior_hole_count') is not None:
        basis = 'gate_summary'
        hole_n = int(gate['interior_hole_count'])
        hole_area = float(gate.get('undercovered_interior_area_m2') or 0.0)
        gate_txt = f'上傳閘門（{gcell:g} m 格）共 {hole_n} 處 {hole_area:,.0f} m²'
        holes = _area_rings(pv_holes_all, frame, cell * 0.5, 500, source='preview_grid', cell_m=cell, label=_lab)
        # 閘門摘要只有前 5 大的中心：沒落在預覽格破洞附近的，畫圓圈標位置（形狀以閘門為準）
        import shapely
        near = [_hole_geom(a, frame).buffer(gcell) for a in pv_holes_all]
        for gh in list(gate_holes_summary or []):
            c = gh.get('centroid_latlon') if isinstance(gh, Mapping) else None
            if not c:
                continue
            px, py = frame.to_xy(float(c[0]), float(c[1]))
            if any(g.contains(shapely.Point(float(px), float(py))) for g in near):
                continue
            a_m2 = max(float(gh.get('area_m2') or 0.0), 0.0)
            h = {'ring': _marker_ring(frame, c[0], c[1], max(3.0 * gcell, min(math.sqrt(a_m2 / math.pi), 200.0))),
                 'area_m2': round(a_m2, 1), 'min_views': int(gh.get('min_views') or 0), 'source': 'gate_centroid',
                 'cell_m': gcell, 'blocks_upload': True}
            h['label'] = _lab(h)
            holes.append(h)
            markers += 1
        disagrees = bool(markers) or ((hole_n > 0) != (len(pv_holes_all) > 0))
    else:
        basis = 'preview'
        hole_n = len(pv_holes_all)
        hole_area = float(getattr(rep, 'undercovered_interior_area_m2', 0.0) or 0.0)
        holes = _area_rings(pv_holes_all, frame, cell * 0.5, 500, source='preview_grid', cell_m=cell, label=_lab)

    # ---- 測區 ----
    survey_rings = [[[round(float(a), 7), round(float(b), 7)] for a, b in poly['exterior']]
                    for poly in survey_polygons]

    # ---- 照片 ----
    drone_of = dict(drone_of or {})
    ppu = dict(getattr(rep, 'photos_per_uav', None) or {})
    labels = sorted(set(ppu) | set(drone_of))
    photos_all = list(getattr(rep, 'photos', None) or [])
    lab_idx = {u: i for i, u in enumerate(labels)}
    n_ph = len(photos_all)
    if n_ph > max_points:
        sel = np.unique(np.linspace(0, n_ph - 1, int(max_points)).round().astype(int))
    else:
        sel = np.arange(n_ph)
    photos = [[round(float(photos_all[i].lat_deg), 7), round(float(photos_all[i].lon_deg), 7),
               lab_idx[photos_all[i].uav]] for i in sel]

    # ---- AGL／GSD（與驗證器同一個地面）----
    if n_ph:
        plat = np.array([p.lat_deg for p in photos_all], dtype=float)
        plon = np.array([p.lon_deg for p in photos_all], dtype=float)
        ph = np.array([p.h_m for p in photos_all], dtype=float)
        agl = ph - _ground_heights(plat, plon, terrain, dem)
        good = np.isfinite(agl) & (agl > 0)
    else:
        agl, good = np.zeros(0), np.zeros(0, dtype=bool)
    gsd_cm = agl * pix_um / f_mm / 10.0                 # AGL·像素間距/焦距（m）×100
    per_uav_med = []
    uav_arr = np.array([p.uav for p in photos_all], dtype=object)
    for u in labels:
        m = good & (uav_arr == u)
        if m.any():
            per_uav_med.append(float(np.median(gsd_cm[m])))
    ortho = round(max(per_uav_med), 1) if per_uav_med else None
    gsd_stats = _stat3(gsd_cm[good], 2)
    gsd_med = gsd_stats['median']

    # ---- 張數／資料量／RAM ----
    per_uav_n = {u: int(ppu.get(u, 0)) for u in labels}
    n_total = int(sum(per_uav_n.values()))
    lo_mb, hi_mb, mp = data_mb_per_photo(camera_effective)       # 本相機像素數 × 0.6–1.0 B/px（未經實測）
    data_gb = [round(n_total * lo_mb / 1000.0, 2), round(n_total * hi_mb / 1000.0, 2)]

    # ---- 視數比例 ----
    def _fr(r):
        if r is None:
            return None
        return {str(int(k)): float(x) for k, x in r.frac_ge.items()}
    frac = {'interior': _fr(getattr(rep, 'interior', None)), 'edge_band': _fr(getattr(rep, 'edge_band', None)),
            'all': {str(int(k)): float(x) for k, x in (getattr(rep, 'frac_ge', None) or {}).items()}}

    eb = getattr(rep, 'edge_band_m', None)
    a_edge = float(getattr(rep, 'undercovered_edge_band_area_m2', 0.0) or 0.0)
    if eb is None:
        edge_note = '沒有可用照片 ⇒ 不分邊緣帶，視數不足全部算內部'
    else:
        edge_note = (f'邊緣帶＝離測區邊界 ≤ {float(eb):.0f} m（半刈幅）：最外側條帶只蓋一次、觸發間距又被 50 Hz 量化，'
                     f'視數規律地少——這裡視數 <{K_PRODUCT} 共 {len(edge_all)} 處 {a_edge:,.0f} m²，不擋上傳。'
                     '成果的邊緣品質通常較差；需要邊緣也成圖時，請把測區外擴約半個刈幅')

    # ---- 可見地物 ----
    feats = [{'feature': name, 'size_cm': size, 'pixels': (round(size / gsd_med, 1) if gsd_med else None)}
             for name, size in VISIBLE_FEATURES]

    # ---- 假設（與閘門摘要同一份文字＋預覽自己的）----
    assumptions = [f'{PREVIEW_BANNER}：由上傳那份任務模擬距離觸發的拍照位置，推算每格被幾張照片看到；'
                   '圖上只有視數，沒有任何影像內容（不重抽底圖、不預測正射影像的樣子）']
    if wind is not None:
        assumptions += mapping_exposure_assumptions(wind, wind_source or '—', cell, cam_max_roll_deg)
    assumptions.append(MAPPING_EXPOSURE_SEAM_CAVEAT)
    if basis == 'gate' and gate_report is not rep:
        assumptions.append(f'預覽格 {cell:g} m、上傳閘門格 {gcell:g} m：內部破洞與「會不會擋上傳」用閘門格另算一次'
                           f'（與上傳閘門同一判定）；預覽格只用來上色，並另標閘門格看不到的細破洞（格點量化，可能差 ±1 格）')
    elif basis == 'gate_summary':
        assumptions.append(f'預覽格 {cell:g} m、上傳閘門格 {gcell:g} m，這次沒有重算閘門格'
                           + (f'（{gate_note}）' if gate_note else '')
                           + '：破洞位置用預覽格畫，個數與能不能上傳以規劃時的閘門摘要為準')
    elif basis == 'preview':
        assumptions.append('上傳閘門結果未知' + (f'（{gate_note}）' if gate_note else '')
                           + f'：破洞用預覽 {cell:g} m 格畫，與上傳閘門（{gcell:g} m 格）的判定可能差 ±1 格，'
                             '能不能上傳以閘門為準')
    if fac > 1:
        assumptions.append(_raster_note(cell, fac))
    if n_ph > len(sel):
        assumptions.append(f'地圖上的拍照點只畫 {len(sel):,}／{n_ph:,} 張（均勻抽樣；張數與資料量用全部）')
    assumptions.append(f'可成圖＝視數 ≥{K_PRODUCT}（與上傳閘門同一門檻）、高品質＝≥{HQ_VIEWS}：常用經驗門檻，不是成圖保證'
                       '（紋理少、水面、移動物、曝光不良都會讓 SfM 失敗）')
    assumptions.append('GSD＝AGL × 像素間距 ÷ 焦距（正下視、平飛；不含傾斜與鏡頭畸變）；AGL 用規劃 DEM')
    bpx = f'{DATA_BYTES_PER_PX[0]:.1f}–{DATA_BYTES_PER_PX[1]:.1f} B/px'
    if mp is not None:
        assumptions.append(f'資料量假設每張 {lo_mb:g}–{hi_mb:g} MB＝本相機 {mp:.2f} MP × {bpx}（JPEG 壓縮率的假設，'
                           f'未經實測；來源 {DATA_SIZE_SOURCE}，與別處用的「5 MB/張」也不一致）；1 GB＝1000 MB')
    else:
        assumptions.append(f'資料量假設每張 {lo_mb:g}–{hi_mb:g} MB（相機像素數未知 ⇒ 用 12.33 MP × {bpx} 的範圍，'
                           f'未經實測；來源 {DATA_SIZE_SOURCE}）；1 GB＝1000 MB')
    cross = dict(cross or {})
    if cross.get('cross_strips'):
        off = cross.get('cross_alt_offset_m')
        try:
            off = float(off)
        except (TypeError, ValueError):
            off = None
        if off is not None and abs(off) < 1e-9:
            assumptions.append('交叉航帶與主航帶同高度（高度差 0 m）⇒ 精度表最接近的文獻是 Štroner 2021（只有小幅改善）；'
                               'Gerke 2016 的 16.5 → 2.4 cm 是換方向＋換高度，不適用')
        elif off is not None:
            assumptions.append(f'交叉航帶高度差 {off:+g} m：Gerke 2016 的條件是 75 對 105 m（高度比 1.4），'
                               '其他高度比沒有文獻數字')
    assumptions.append('正射解析度≈各機中位 GSD 中最粗的那個（與航測處理對話框的預設同規則）；'
                       'DSM 解析度約 2×GSD 是估計（ODM 預設行為的經驗值），實際以 ODM 輸出為準')
    assumptions.append('ODM 記憶體級距引自評估 §8 的 ODM RAM 表（500 張 32 GB、1,500 張 64 GB、2,500 張 128 GB）')
    assumptions.append('可見地物以「尺寸 ÷ GSD＝跨幾個像素」估；經驗上約 3–5 像素以上才看得出形狀，不是保證')
    if is_world_terrain:
        assumptions.append('規劃 DEM 是 Cesium World Terrain：授權只准放記憶體 ⇒ 不產生、不輸出地形山影')

    st = _empty_stats()
    st.update({
        'uav_labels': labels,
        'drone_ids': [drone_of.get(u) for u in labels],
        'camera': {'preset': camera_effective.get('preset'), 'focal_mm': f_mm, 'pixel_um': pix_um,
                   'width_px': camera_effective.get('width_px'), 'height_px': camera_effective.get('height_px')},
        'agl_m': _stat3(agl[good], 1),
        'gsd_cm': gsd_stats,
        'ortho_resolution_cm': ortho,
        'dsm_resolution_cm_est': (round(2.0 * ortho, 1) if ortho is not None else None),
        'dsm_resolution_note': '估計值（約 2×GSD；ODM 預設行為的經驗值），實際以 ODM 輸出為準',
        'photos_per_uav': per_uav_n,
        'photos_total': n_total,
        'photos_excluded': dict(getattr(rep, 'photos_excluded', None) or {}),
        'data_gb_range': data_gb,
        'data_mb_per_photo': [lo_mb, hi_mb],
        'data_bytes_per_px': (list(DATA_BYTES_PER_PX) if mp is not None else None),
        'data_mb_basis': (f'本相機 {mp:.2f} MP × {bpx}（未經實測）' if mp is not None
                          else f'相機像素數未知 ⇒ 12.33 MP × {bpx}（未經實測）'),
        'photos_shown': int(len(sel)),
        'odm_ram_tier': odm_ram_tier(n_total),
        'area_km2': area,
        'frac_ge': frac,
        'forward_overlap_p05': getattr(getattr(rep, 'forward_overlap', None), 'p05', None),
        'side_overlap_p05': getattr(getattr(rep, 'side_overlap', None), 'p05', None),
        'seam_overlap_min': getattr(rep, 'seam_overlap_min', None),
        # 內部破洞：依據 basis（'gate'＝上傳閘門格、與閘門同一判定；'gate_summary'＝規劃時的閘門摘要；
        # 'preview'＝閘門結果未知、只有預覽格）。預覽格自己的數字另列（preview_grid_*）
        'interior_hole_count': int(hole_n),
        'interior_hole_area_m2': float(hole_area),
        'interior_hole_basis': basis,
        'interior_hole_cell_m': (cell if basis == 'preview' else gcell),
        'preview_grid_hole_count': len(pv_holes_all),
        'preview_grid_hole_area_m2': float(getattr(rep, 'undercovered_interior_area_m2', 0.0) or 0.0),
        'preview_only_hole_count': preview_only_n,
        'gate_only_hole_count': gate_only_n,
        'gate_marker_count': markers,
        'gate_disagrees': disagrees,
        'edge_short_count': len(edge_all),
        'edge_short_area_m2': a_edge,
        'edge_band_m': (None if eb is None else float(eb)),
        'edge_fringe_note': edge_note,
        'cell_size_m': cell,
        'raster_cell_m': rcell,
        'gate_cell_size_m': MAPPING_EXPOSURE_CELL_M,
        'k_product': K_PRODUCT,
        'hq_views': HQ_VIEWS,
        'assumptions': assumptions,
        'accuracy_literature': [dict(r) for r in ACCURACY_LITERATURE],
        'accuracy_note': ACCURACY_NOTE,
        'visible_features': feats,
        'datum': datum,
        'terrain_is_world_terrain': bool(is_world_terrain),
        'wind': (None if wind is None else [float(wind[0]), float(wind[1])]),
        'wind_source': wind_source,
        'gate': gate,
        'cross_strips': ({'on': bool(cross.get('cross_strips')), 'alt_offset_m': cross.get('cross_alt_offset_m')}
                         if cross else None),
        'banner': PREVIEW_BANNER,
    })

    # ---- 粗山影（World Terrain 一律不做）----
    hs = None if is_world_terrain else _hillshade(dem, bounds)
    st['terrain_context'] = ('不產生（規劃 DEM 是 Cesium World Terrain，授權只准放記憶體）' if is_world_terrain
                             else ('規劃 DEM 山影（粗）' if hs is not None else '沒有山影（DEM 平坦或範圍外）'))
    st['elapsed_s'] = round(time.perf_counter() - t0, 3)
    st = _json_safe(st)

    return HdmapPreview(
        ok=True, reason=None, bounds_wsen=bounds, image_png=png, image_px=(int(vd.shape[1]), int(vd.shape[0])),
        cell_size_m=float(rcell), extent_rings=ext_rings, extent_holes=ext_holes, interior_holes=holes,
        edge_short=edge, survey_rings=survey_rings, photos=photos, stats=st,
        legend=_default_legend(cell, gate_cell_m=gcell, gate_known=basis in ('gate', 'gate_summary'),
                               preview_only=int(preview_only_n or 0), gate_markers=markers,
                               disagrees=bool(disagrees), raster_fac=fac, photos_shown=int(len(sel)),
                               photos_all=n_ph),
        hillshade_png=(hs[0] if hs else None), hillshade_bounds_wsen=(hs[1] if hs else None))


def _is_world_terrain(hv, swarm) -> bool:
    if getattr(swarm, '_world_terrain', None) is not None:
        return True
    if bool((getattr(swarm, 'harvest_params', None) or {}).get('hv_world_terrain')):
        return True
    return any(str(n).startswith('terrain_source(Cesium World Terrain') for n in (getattr(hv, 'notes', None) or []))


def _not_mapping_reason(hv, swarm) -> str | None:
    if hv is None:
        return '沒有 HARVEST 規劃結果'
    if getattr(hv, 'last_ctx', None) is None or getattr(hv, 'last_plan', None) is None:
        return '尚未規劃（或規劃已清除）'
    if getattr(hv, 'mapping_mode', None) is None:
        return '這份計畫不是航測建圖模式（沒勾「航測建圖」）⇒ 沒有可預覽的成果'
    if swarm is None or getattr(swarm, 'hv_mapping_setup', None) is None:
        return '協調器沒有建圖設定（hv_mapping_setup）⇒ 不是建圖計畫，或計畫已被取代'
    owner = getattr(swarm, 'hv_coordinator', None)
    if owner is not None and owner is not hv:
        return '協調器的計畫已換成另一份 ⇒ 這份預覽的輸入對不上，請重新規劃後再預覽'
    if getattr(hv, 'camera_cfg', None) is None:
        return '這份計畫沒有相機觸發設定（camera_cfg）⇒ 沒有拍照指令'
    return None


def compute_hdmap_preview(hv, swarm, *, cell_size_m: float | None = None) -> HdmapPreview:
    """建圖計畫 → 預計成果（規劃預測）。**不丟例外**：任何失敗都回 ``ok=False`` 與繁中原因。

    ``hv``：規劃後的 HvDccppCoordinator；``swarm``：規劃它的 SwarmCoordinator（``hv_mapping_setup`` 給相機）。
    輸入與上傳閘門的 :func:`mission.swarm_coordinator.mapping_exposure_summary` **同一支**
    （:func:`mission.swarm_coordinator.mapping_exposure_report`），只有格距不同：預設外接框 ≤ 4 km² 用 2 m、否則 4 m，
    格數不超過約 :data:`PREVIEW_MAX_CELLS`（明給的 ``cell_size_m`` 估出來超過也一樣拒絕）。

    預覽格與閘門格（:data:`MAPPING_EXPOSURE_CELL_M`）不同時，**再用閘門格跑一次**（格數較少；同一組輸入），
    內部破洞與「會不會擋上傳」以它為準（見 :func:`build_hdmap_preview`）；閘門格估出來超過格數上限就不重跑，
    改用規劃時的閘門摘要（``swarm.hv_exposure_summary``）。
    """
    t0 = time.perf_counter()
    try:
        from mission import swarm_coordinator as SC
        reason = _not_mapping_reason(hv, swarm)
        if reason:
            return HdmapPreview.failed(reason, elapsed_s=time.perf_counter() - t0)
        setup = swarm.hv_mapping_setup
        eff = dict(setup.get('effective') or {})
        camera = setup.get('camera')
        if camera is None:
            return HdmapPreview.failed('建圖設定沒有相機（camera）', elapsed_s=time.perf_counter() - t0)
        for key in ('pixel_um', 'focal_mm'):
            val = eff.get(key)
            if not (isinstance(val, (int, float)) and not isinstance(val, bool) and math.isfinite(float(val))
                    and float(val) > 0):
                return HdmapPreview.failed('沒有相機像素尺寸／焦距 ⇒ 算不出 GSD（請在「航測建圖」填相機）',
                                           elapsed_s=time.perf_counter() - t0)
        explicit = cell_size_m is not None
        if explicit:
            cell = float(cell_size_m)
            if not (math.isfinite(cell) and cell > 0):
                return HdmapPreview.failed(f'格距須為正數，得 {cell_size_m!r}', elapsed_s=time.perf_counter() - t0)
        polys, _n = SC._mapping_survey_polygons(hv.last_ctx)
        if not polys:
            return HdmapPreview.failed('測區（作業區 − 禁航區）是空的', elapsed_s=time.perf_counter() - t0)
        if explicit:
            # 明給的格距也要守格數上限（views_by_uav 每架一張 int32 全格網；先擋，免得白模擬一輪拍照）
            est = estimate_cells(polys, cell)
            if est > PREVIEW_MAX_CELLS * 1.1:
                need = math.ceil(math.sqrt(est * cell * cell / PREVIEW_MAX_CELLS) * 2.0) / 2.0
                return HdmapPreview.failed(f'格距太細：{cell:g} m 約 {est:,.0f} 格，超過預覽上限約 {PREVIEW_MAX_CELLS:,} 格'
                                           f'（請把格距調大到 ≥ {need:g} m，或不指定格距用預設）',
                                           elapsed_s=time.perf_counter() - t0)
        else:
            cell = choose_cell_size_m(polys)
        rep, inp = SC.mapping_exposure_report(hv, camera, cell_size_m=cell)
        n_cells = int(rep.grid.views.size) if rep.grid is not None else 0
        if not explicit and n_cells > PREVIEW_MAX_CELLS * 1.1:
            # 足跡伸出測區比估的多（高 AGL／廣角）：放大格距重算一次
            cell = math.ceil(cell * math.sqrt(n_cells / PREVIEW_MAX_CELLS) * 2.0) / 2.0
            del rep
            rep, inp = SC.mapping_exposure_report(hv, camera, cell_size_m=cell)
            n_cells = int(rep.grid.views.size) if rep.grid is not None else 0
        # 閘門格（4 m）另跑一次：內部破洞「會不會擋上傳」以它為準（預覽格較細時兩邊的格點量化不同）
        gcell = float(SC.MAPPING_EXPOSURE_CELL_M)
        gate_rep, gate_note = None, None
        if abs(cell - gcell) > 1e-9:
            est_g = n_cells * (cell / gcell) ** 2
            if est_g > PREVIEW_MAX_CELLS * 1.1:
                gate_note = f'閘門格 {gcell:g} m 約 {est_g:,.0f} 格，超過預覽格數上限'
            else:
                try:
                    gate_rep, _inp_g = SC.mapping_exposure_report(hv, camera, cell_size_m=gcell)
                except Exception as e:      # noqa: BLE001 — 閘門格重算失敗：退回規劃時的閘門摘要，原因寫進假設
                    _log.warning(f'預覽：閘門格重算失敗：{type(e).__name__}: {e}')
                    gate_note = f'閘門格重算失敗：{type(e).__name__}: {e}'
        world = _is_world_terrain(hv, swarm)
        dem = inp.get('dem')
        dl = dem.datum_label() if hasattr(dem, 'datum_label') else '未知基準'
        datum = (f'Cesium World Terrain（{dl}；只放記憶體）' if world else f'規劃 DEM（{dl}）')
        gs = getattr(swarm, 'hv_exposure_summary', None)
        gate_d = getattr(swarm, 'hv_mapping_gate', None)
        gate, gate_holes = None, None
        if isinstance(gs, Mapping) and gs.get('ok') is True:
            gate = {'cell_size_m': gs.get('cell_size_m'), 'interior_hole_count': gs.get('interior_hole_count'),
                    'undercovered_interior_area_m2': gs.get('undercovered_interior_area_m2'),
                    'uploadable': (gate_d or {}).get('uploadable') if isinstance(gate_d, Mapping) else None}
            gate_holes = list(gs.get('interior_holes') or [])
        out = build_hdmap_preview(
            rep, survey_polygons=inp['polygons'], camera_effective=eff, terrain=inp.get('terrain'), dem=dem,
            drone_of=inp.get('drone_of'), wind=inp.get('wind'), wind_source=inp.get('wind_source'),
            cam_max_roll_deg=inp.get('cam_max_roll_deg'), datum=datum, is_world_terrain=world, gate=gate,
            gate_report=gate_rep, gate_holes_summary=gate_holes, gate_note=gate_note,
            cross={k: eff.get(k) for k in ('cross_strips', 'cross_alt_offset_m')}, t_start=t0)
        del rep, gate_rep
        return out
    except Exception as e:      # noqa: BLE001 — 預覽失敗不能讓背景執行緒崩潰；原因要讓操作員看得到
        _log.warning(f'預覽高精地圖成果失敗：{type(e).__name__}: {e}', exc_info=True)
        return HdmapPreview.failed(f'預覽計算失敗（{type(e).__name__}: {e}）', elapsed_s=time.perf_counter() - t0)


# ---------------------------------------------------------------------------
# 靜態圖（對話框／匯出用）：matplotlib Agg，只用 ASCII／數字（不依賴中文字型）
# ---------------------------------------------------------------------------
_NICE_LENGTHS_M = (5, 10, 20, 50, 100, 200, 250, 500, 1000, 2000, 2500, 5000, 10000, 20000)


def _nice_scale_m(width_m: float) -> float:
    target = 0.2 * float(width_m)
    best = _NICE_LENGTHS_M[0]
    for L in _NICE_LENGTHS_M:
        if L <= target:
            best = L
    return float(best)


def _png_to_array(png: bytes) -> np.ndarray:
    from PIL import Image
    with Image.open(io.BytesIO(png)) as im:
        return np.asarray(im.convert('RGBA'))


def render_preview_map_png(preview: HdmapPreview, *, width_px: int = 1100) -> bytes:
    """預計成果的靜態地圖（PNG bytes）：山影（若有）＋分級上色＋可成圖外框＋破洞（紅）＋拍照點＋比例尺＋指北。
    north-up、經緯比例依 cos(緯度) 校正。圖上文字只有 ASCII／數字（不需要中文字型）。"""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.patches import Patch
    from matplotlib.patches import Polygon as MplPolygon

    width_px = int(max(200, min(int(width_px), 4000)))
    w_in = width_px / 100.0
    if not preview.ok:
        fig = Figure(figsize=(w_in, w_in * 0.4), dpi=100)
        FigureCanvasAgg(fig)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_axis_off()
        ax.text(0.5, 0.5, 'No preview (see reason in the dialog)', ha='center', va='center', fontsize=14)
        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=100)
        return buf.getvalue()

    W, S, E, N = (float(v) for v in preview.bounds_wsen)
    if preview.hillshade_bounds_wsen:
        hW, hS, hE, hN = (float(v) for v in preview.hillshade_bounds_wsen)
    lat_c = (S + N) / 2.0
    kx, ky = _m_per_deg(lat_c)
    width_m, height_m = (E - W) * kx, (N - S) * ky
    pad_x, pad_y = 0.03 * (E - W), 0.03 * (N - S)
    xl, xr, yb, yt = W - pad_x, E + pad_x, S - pad_y, N + pad_y
    ratio = ((yt - yb) * ky) / max((xr - xl) * kx, 1e-9)
    h_in = float(min(max(w_in * ratio, 2.0), w_in * 2.5))
    fig = Figure(figsize=(w_in, h_in), dpi=100, facecolor='white')
    FigureCanvasAgg(fig)
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.98])
    ax.set_facecolor('#f2f2f2')
    ax.set_xlim(xl, xr)
    ax.set_ylim(yb, yt)
    ax.set_aspect(ky / kx, adjustable='datalim')
    ax.set_xticks([])
    ax.set_yticks([])

    if preview.hillshade_png and preview.hillshade_bounds_wsen:
        ax.imshow(_png_to_array(preview.hillshade_png), extent=(hW, hE, hS, hN), origin='upper',
                  interpolation='bilinear', alpha=0.55, zorder=0)
    if preview.image_png:
        ax.imshow(_png_to_array(preview.image_png), extent=(W, E, S, N), origin='upper', interpolation='nearest',
                  alpha=0.85, zorder=1)
    for ring in preview.survey_rings:
        r = np.asarray(ring, dtype=float)
        if len(r):
            r = np.vstack([r, r[:1]])
            ax.plot(r[:, 1], r[:, 0], color='#212121', lw=0.9, ls=(0, (2, 2)), zorder=2)
    for ring in list(preview.extent_rings) + list(preview.extent_holes):
        r = np.asarray(ring, dtype=float)
        if len(r):
            ax.plot(r[:, 1], r[:, 0], color='black', lw=2.4, zorder=3)
            ax.plot(r[:, 1], r[:, 0], color='white', lw=1.1, zorder=3.1)
    for h in preview.edge_short:
        r = np.asarray(h['ring'], dtype=float)
        if len(r) >= 3:
            ax.add_patch(MplPolygon(r[:, ::-1], closed=True, facecolor='none', edgecolor='#ff6f00', lw=0.8,
                                    zorder=3.5))
    n_fine = 0
    for h in preview.interior_holes:
        r = np.asarray(h['ring'], dtype=float)
        if len(r) < 3:
            continue
        src = h.get('source') if isinstance(h, Mapping) else None
        if src == 'preview_only':          # 只在預覽細格看到（閘門沒擋）：點線、不填色
            n_fine += 1
            ax.add_patch(MplPolygon(r[:, ::-1], closed=True, facecolor='none', edgecolor='#d50000', lw=1.2,
                                    ls=(0, (1, 2)), zorder=4))
        elif src == 'gate_centroid':       # 閘門摘要只有中心：圓圈標位置
            ax.add_patch(MplPolygon(r[:, ::-1], closed=True, facecolor='none', edgecolor='#d50000', lw=1.6,
                                    zorder=4))
        else:
            ax.add_patch(MplPolygon(r[:, ::-1], closed=True, facecolor=(1.0, 0.0, 0.0, 0.35), edgecolor='#d50000',
                                    lw=1.6, zorder=4))
    if preview.photos:
        p = np.asarray(preview.photos, dtype=float)
        cols = np.array(POINT_COLORS, dtype=object)[p[:, 2].astype(int) % len(POINT_COLORS)]
        ax.scatter(p[:, 1], p[:, 0], s=1.5, c=list(cols), linewidths=0, zorder=5)

    # 比例尺（左下）
    L = _nice_scale_m(width_m)
    x0 = xl + 0.04 * (xr - xl)
    y0 = yb + 0.05 * (yt - yb)
    x1 = x0 + L / kx
    tick = 0.012 * (yt - yb)
    ax.plot([x0, x1], [y0, y0], color='black', lw=3, solid_capstyle='butt', zorder=6)
    ax.plot([x0, x0], [y0 - tick, y0 + tick], color='black', lw=1.5, zorder=6)
    ax.plot([x1, x1], [y0 - tick, y0 + tick], color='black', lw=1.5, zorder=6)
    ax.text((x0 + x1) / 2.0, y0 + 2.0 * tick, (f'{L / 1000:g} km' if L >= 1000 else f'{L:g} m'), ha='center',
            va='bottom', fontsize=10, zorder=6,
            bbox=dict(boxstyle='round,pad=0.15', facecolor='white', edgecolor='none', alpha=0.7))
    # 指北（右上）
    ax.annotate('N', xy=(0.95, 0.96), xytext=(0.95, 0.86), xycoords='axes fraction', textcoords='axes fraction',
                ha='center', va='center', fontsize=12, fontweight='bold', zorder=7,
                arrowprops=dict(arrowstyle='-|>', color='black', lw=1.6))
    # 圖例（只用 ASCII）
    handles = []
    ascii_labels = {1: '1 view', 2: '2-4 views', 3: f'{K_PRODUCT}-8 views', 4: f'>={HQ_VIEWS} views'}
    for i, (_lo, _hi, col, _lab) in enumerate(QUALITY_CLASSES):
        if col:
            handles.append(Patch(facecolor=col, edgecolor='none', label=ascii_labels.get(i, str(i))))
    handles.append(Patch(facecolor='none', edgecolor='black', label=f'extent (>={K_PRODUCT} views)'))
    st = preview.stats or {}
    gc = st.get('gate_cell_size_m')
    if st.get('interior_hole_basis') in ('gate', 'gate_summary') and gc is not None:
        hole_lab = f'interior hole (<{K_PRODUCT}, upload gate {float(gc):g} m grid: blocks upload)'
    else:
        hole_lab = f'interior hole (<{K_PRODUCT}; upload gate decides)'
    handles.append(Patch(facecolor=(1.0, 0.0, 0.0, 0.35), edgecolor='#d50000', label=hole_lab))
    if n_fine:
        handles.append(Patch(facecolor='none', edgecolor='#d50000', linestyle=':',
                             label='finer preview grid only (gate does not block)'))
    ax.legend(handles=handles, loc='lower right', fontsize=8, framealpha=0.85)
    ax.text(0.01, 0.99, 'PLANNING PREDICTION - simulated views per cell, not actual imagery',
            transform=ax.transAxes, ha='left', va='top', fontsize=9, zorder=7,
            bbox=dict(boxstyle='round,pad=0.25', facecolor='white', edgecolor='#999999', alpha=0.85))
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=100, facecolor='white')
    return buf.getvalue()
