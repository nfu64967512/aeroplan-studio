"""高精地圖「預計成果」對話框（偵察頁「預覽高精地圖成果…」或 工具 → 預覽高精地圖成果…；非模態）。

``HdmapPreviewDialog(preview, parent=None, *, map_png=None)``

* ``preview``：``survey.preview.HdmapPreview``（``compute_hdmap_preview`` 的 ``ok=True`` 結果）。
* ``map_png``：左邊那張圖的 PNG；不給就呼叫 ``survey.preview.render_preview_map_png(preview)``（延遲匯入）。

左邊是成果分級圖（只有數字／ASCII，不需要中文字型），右邊是統計卡。全部都是**規劃預測，非實際成果**：
* 不拿 Google／底圖瓦片重採樣假裝成「預測正射」；
* 精度只列文獻值與其條件，外加「我們自己的精度要等實飛＋檢核點才知道」；
* 資料量是範圍（每張 MB＝相機像素數 × 0.6–1.0 B/px 的假設寫出來，未經實測；IMX477 即 7.4–12.3 MB）；
* 與上傳閘門同一句樂觀模型提醒（CAM_MAX_ROLL 與轉彎動態沒建模）。
匯出 = 地圖 PNG ＋ 同名 JSON（統計、圖例、外框／破洞環）；Cesium World Terrain 衍生的資料不落地（授權只准記憶體內）。
"""
from __future__ import annotations

import datetime as _dt
import html
import json
import math
from pathlib import Path
from typing import Any, List, Optional, Tuple

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (QDialog, QFileDialog, QHBoxLayout, QLabel, QMessageBox, QPushButton, QSizePolicy,
                             QSplitter, QTextBrowser, QVBoxLayout, QWidget)

from utils.logger import get_logger

logger = get_logger()

#: 標題／橫幅：每個出口都要看得到「規劃預測，非實際成果」
PREDICTION_BANNER = '規劃預測，非實際成果'
DIALOG_TITLE = f'高精地圖預計成果（{PREDICTION_BANNER}）'
#: stats 沒帶精度說明時的保底句（docs/比較_2026-10-15_交叉航帶vs加密GCP.md §6）
ACCURACY_UNKNOWN_FALLBACK = '我們自己的精度目前未知：要等實飛加獨立檢核點才能宣告。'
#: 資料量假設（stats 沒帶範圍時也要說清楚是怎麼估的）
PHOTO_MB_RANGE = (7.4, 12.3)


# ═════════════════════════════════════════════════════════════════════
#  純函式（不碰 widget ⇒ 值層級測試不必開對話框）
# ═════════════════════════════════════════════════════════════════════
def _esc(v: Any) -> str:
    return html.escape('' if v is None else str(v))


def _num(v: Any) -> Optional[float]:
    if v is None or isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _fmt(v: Any, spec: str = 'g', unit: str = '') -> str:
    f = _num(v)
    if f is None:
        return '—'
    return f'{f:{spec}}{unit}'


def _pct(v: Any) -> str:
    f = _num(v)
    return '—' if f is None else f'{f:.2%}'


def _mmm(d: Any, spec: str, unit: str) -> str:
    """{'min','median','max'} → 「最少 / 中位 / 最多」。"""
    d = d if isinstance(d, dict) else {}
    return (f'最少 {_fmt(d.get("min"), spec, unit)}／中位 {_fmt(d.get("median"), spec, unit)}／'
            f'最多 {_fmt(d.get("max"), spec, unit)}')


def _frac_get(d: Any, k: int) -> Any:
    if not isinstance(d, dict):
        return None
    if str(k) in d:
        return d[str(k)]
    return d.get(k)


def _stats(preview) -> dict:
    s = getattr(preview, 'stats', None)
    return s if isinstance(s, dict) else {}


def _mb_range(s: dict) -> Tuple[float, float]:
    """每張 MB 的假設：survey.preview 依相機像素數算的（data_mb_per_photo）；沒有才用固定的退路。"""
    mb = s.get('data_mb_per_photo')
    if isinstance(mb, (list, tuple)) and len(mb) == 2 and _num(mb[0]) is not None and _num(mb[1]) is not None:
        return float(mb[0]), float(mb[1])
    return PHOTO_MB_RANGE


def _legend(preview) -> dict:
    lg = getattr(preview, 'legend', None)
    return lg if isinstance(lg, dict) else {}


def preview_uses_world_terrain(preview) -> bool:
    """這份預覽的地形是不是 Cesium World Terrain（stats['terrain_is_world_terrain']，或基準標籤提到它）。"""
    s = _stats(preview)
    if s.get('terrain_is_world_terrain') is True:
        return True
    return 'world terrain' in str(s.get('datum') or '').lower()


def _cell(v: Any) -> str:
    """表格格子：None／空字串 ⇒ —。"""
    return '—' if v is None or str(v).strip() == '' else _esc(v)


def _hole_rows(s: dict, k_txt: str) -> List[Tuple[str, str]]:
    """內部破洞的列：**「會不會擋上傳」只用上傳閘門那個格距的結果說**（stats['interior_hole_basis']）；
    預覽格自己的數字另列並註明「以閘門為準」；兩邊不同時用橘字說出來。⚠ 原樣的 '<' 會被 Qt 當標籤開頭 ⇒ &lt;。"""
    rows: List[Tuple[str, str]] = []
    basis = s.get('interior_hole_basis')
    gc = _fmt(s.get('gate_cell_size_m'), 'g', ' m')
    pc = _fmt(s.get('cell_size_m'), 'g', ' m')
    n, a = s.get('interior_hole_count'), s.get('interior_hole_area_m2')
    gate = s.get('gate') if isinstance(s.get('gate'), dict) else {}
    up = gate.get('uploadable')
    up_txt = ('' if up is None else ('；建圖閘門目前：可上傳' if up else '；建圖閘門目前：不可上傳（原因見偵察頁）'))
    if basis in ('gate', 'gate_summary'):
        same_grid = (_num(s.get('cell_size_m')) is not None
                     and _num(s.get('cell_size_m')) == _num(s.get('gate_cell_size_m')))
        src = ('規劃時的閘門摘要' if basis == 'gate_summary' else
               ('預覽格就是閘門格，與上傳閘門同一判定' if same_grid else '閘門格另算一次，與上傳閘門同一判定'))
        blocks = ('有就擋上傳' if _num(n) is None else ('會擋上傳' if float(n) > 0 else '不擋上傳'))
        rows.append((f'內部破洞（上傳閘門，格 {gc}）',
                     f"{_fmt(n, '.0f', ' 處')}、{_fmt(a, ',.0f', ' m²')}（視數 &lt; {k_txt}；{blocks}；{_esc(src)}）"
                     + _esc(up_txt)))
        po = s.get('preview_only_hole_count')
        if not same_grid:                           # 預覽格就是閘門格（大面積 4 m）⇒ 上一列已是同一份
            rows.append((f'預覽格 {pc} 的內部視數不足',
                         f"{_fmt(s.get('preview_grid_hole_count'), '.0f', ' 處')}、"
                         f"{_fmt(s.get('preview_grid_hole_area_m2'), ',.0f', ' m²')}"
                         + (f"；其中只在預覽格看到 {_fmt(po, '.0f', ' 處')}（閘門沒擋，仍建議檢查）" if _num(po) else '')
                         + '（能不能上傳以上一列的閘門為準）'))
        if s.get('gate_disagrees'):
            rows.append(('⚠ 兩個格距不一致',
                         "<span style='color:#e65100;'><b>預覽格與上傳閘門格看到的內部破洞不同（格點量化）："
                         "地圖上紅框＝閘門判定、紅點框＝只在預覽格看到；能不能上傳以閘門為準。</b></span>"))
    elif basis == 'preview':
        rows.append((f'內部視數不足（預覽格 {pc}）',
                     f"{_fmt(n, '.0f', ' 處')}、{_fmt(a, ',.0f', ' m²')}（視數 &lt; {k_txt}；"
                     f"上傳閘門（格 {gc}）結果未知，能不能上傳以閘門為準）" + _esc(up_txt)))
    else:
        rows.append(('內部破洞', f"{_fmt(n, '.0f', ' 處')}、{_fmt(a, ',.0f', ' m²')}"
                                 f"（視數 &lt; {k_txt}；能不能上傳以上傳閘門為準）" + _esc(up_txt)))
    return rows


def preview_html(preview) -> str:
    """統計卡（zh-TW HTML）：橫幅、摘要、圖例、假設、文獻精度（附條件）、看得見的地物、破洞、基準。"""
    s = _stats(preview)
    k = s.get('k_product')
    k_txt = _fmt(k, 'g')
    out: List[str] = []
    out.append(f"<p style='background:#fdecea;color:#b71c1c;padding:6px;border:1px solid #e57373;'>"
               f"<b>⚠ {PREDICTION_BANNER}</b>：下列數字由規劃航線、相機與規劃 DEM 模擬推得，"
               f"不是飛完的成果；圖上的顏色是「每格預計被幾張照片拍到」，不是正射影像。</p>")

    # ── 摘要 ──
    cam = s.get('camera') if isinstance(s.get('camera'), dict) else {}
    rows: List[Tuple[str, str]] = []
    rows.append(('相機', f"{_esc(cam.get('preset') or '自訂')}：焦距 {_fmt(cam.get('focal_mm'), 'g', ' mm')}、"
                         f"像素 {_fmt(cam.get('pixel_um'), 'g', ' µm')}、"
                         f"{_fmt(cam.get('width_px'), '.0f')}×{_fmt(cam.get('height_px'), '.0f')} px"))
    rows.append(('對地高度（AGL）', _mmm(s.get('agl_m'), '.0f', ' m')))
    rows.append(('地面解析度（GSD）', _mmm(s.get('gsd_cm'), '.2f', ' cm')))
    rows.append(('正射解析度（預估）', _fmt(s.get('ortho_resolution_cm'), '.2f', ' cm')))
    # survey.preview 有自己的說明（dsm_resolution_note）就用它；沒有才用固定的一句（兩句一起放是同一件事講兩次）
    rows.append(('DSM 解析度（預估）', _fmt(s.get('dsm_resolution_cm_est'), '.1f', ' cm')
                 + (f"（{_esc(s.get('dsm_resolution_note'))}）" if s.get('dsm_resolution_note')
                    else '（約 2×GSD，ODM 預設值推估）')))
    labels = s.get('uav_labels') or []
    ids = s.get('drone_ids') or []
    ppu = s.get('photos_per_uav') if isinstance(s.get('photos_per_uav'), dict) else {}
    if ppu:
        parts = []
        for lab, n in ppu.items():
            did = None
            if lab in labels:
                i = labels.index(lab)
                did = ids[i] if i < len(ids) else None
            who = f'機 {did}（{lab}）' if did is not None else str(lab)
            parts.append(f'{_esc(who)} {_fmt(n, ".0f")} 張')
        rows.append(('模擬張數', '、'.join(parts)))
    rows.append(('總張數', _fmt(s.get('photos_total'), '.0f', ' 張')))
    excl = s.get('photos_excluded') if isinstance(s.get('photos_excluded'), dict) else {}
    if any(_num(v) for v in excl.values()):
        why = {'agl_le_0': 'AGL ≤ 0', 'horizon': '傾斜到角點視線 > 85°'}
        rows.append(('不進度量的照片', '、'.join(f'{_esc(why.get(r, r))} {_fmt(n, ".0f")} 張'
                                             for r, n in excl.items() if _num(n))))
    rng = s.get('data_gb_range')
    lo, hi = (rng[0], rng[1]) if isinstance(rng, (list, tuple)) and len(rng) == 2 else (None, None)
    mb_lo, mb_hi = _mb_range(s)
    basis = s.get('data_mb_basis')
    rows.append(('原始照片資料量', f'{_fmt(lo, ".1f")}–{_fmt(hi, ".1f")} GB'
                 f'（假設每張 {float(mb_lo):g}–{float(mb_hi):g} MB'
                 + (f'＝{_esc(basis)}' if basis else '') + '，依壓縮與場景而變）'))
    rows.append(('ODM 記憶體需求', _esc(s.get('odm_ram_tier') or '—')))
    area = s.get('area_km2') if isinstance(s.get('area_km2'), dict) else {}
    # product／high_quality 是整張格網（足跡伸出測區的部分也算）⇒ 可能比測區還大；
    # survey.preview 另給 *_in_survey（測區內）——有就一起寫出來，免得讀成「覆蓋率 >100%」
    def _in_survey(key):
        v = area.get(f'{key}_in_survey')
        return f"（測區內 {_fmt(v, '.3f', ' km²')}）" if _num(v) is not None else ''
    rows.append(('面積', f"測區 {_fmt(area.get('survey'), '.3f', ' km²')}、"
                         f"可成圖（視數 ≥{k_txt}）{_fmt(area.get('product'), '.3f', ' km²')}{_in_survey('product')}、"
                         f"高品質（≥9）{_fmt(area.get('high_quality'), '.3f', ' km²')}{_in_survey('high_quality')}"
                         + ('；含足跡伸出測區外的部分' if _in_survey('product') else '')))
    fg = s.get('frac_ge') if isinstance(s.get('frac_ge'), dict) else {}
    eb = s.get('edge_band_m')
    cell_txt = (f'（預覽格 {_fmt(s.get("cell_size_m"), "g", " m")}）' if s.get('interior_hole_basis') else '')
    for key, name in (('interior', f'內部（離邊界 > {_fmt(eb, ".1f", " m")}）{cell_txt}'),
                      ('edge_band', f'邊緣帶（≤ {_fmt(eb, ".1f", " m")}）{cell_txt}'),
                      ('all', f'全測區{cell_txt}')):
        d = fg.get(key)
        if key == 'all' and not d:
            continue
        rows.append((name, f'視數 ≥{k_txt}：{_pct(_frac_get(d, int(k) if _num(k) is not None else 5))}、'
                           f'≥9：{_pct(_frac_get(d, 9))}'))
    rows.append(('重疊（第 5 百分位）', f"前向 {_fmt(s.get('forward_overlap_p05'), '.2f')}、"
                                     f"側向 {_fmt(s.get('side_overlap_p05'), '.2f')}"))
    rows.append(('跨機接縫最窄', _fmt(s.get('seam_overlap_min'), '.2f', ' 刈幅')
                 if _num(s.get('seam_overlap_min')) is not None else '—（單機或區塊不相接）'))
    rows.extend(_hole_rows(s, k_txt))
    if s.get('edge_short_count') is not None:
        rows.append(('邊緣帶視數不足', f"{_fmt(s.get('edge_short_count'), '.0f', ' 處')}、"
                                    f"{_fmt(s.get('edge_short_area_m2'), ',.0f', ' m²')}（只顯示不擋上傳）"))
    if s.get('edge_fringe_note'):
        rows.append(('測區邊緣', _esc(s.get('edge_fringe_note'))))
    gate_cell = s.get('gate_cell_size_m')
    rows.append(('格網', f"{_fmt(s.get('cell_size_m'), 'g', ' m')}（成圖門檻 k = {k_txt}，與上傳閘門相同"
                         + (f"；閘門格 {_fmt(gate_cell, 'g', ' m')}" if _num(gate_cell) is not None else '') + '）'))
    wind = s.get('wind')
    if isinstance(wind, (list, tuple)) and len(wind) == 2:
        rows.append(('規劃風', f"{_fmt(wind[0], 'g', ' m/s')} 來自 {_fmt(wind[1], 'g', '°')}"
                              f"（{_esc(s.get('wind_source') or '來源未記錄')}；不是現場實測風）"))
    rows.append(('高程基準', _esc(s.get('datum') or '—')))
    if s.get('terrain_context'):
        rows.append(('地形底圖', _esc(s.get('terrain_context'))))
    rows.append(('計算時間', _fmt(s.get('elapsed_s'), '.1f', ' s')))
    out.append('<h3>摘要</h3><table cellspacing=0 cellpadding=3 border=1>'
               + ''.join(f'<tr><td><b>{_esc(a)}</b></td><td>{b}</td></tr>' for a, b in rows) + '</table>')

    # ── 圖例 ──
    lg = _legend(preview)
    cls = lg.get('classes') or []
    if cls:
        out.append(f"<h3>{_esc(lg.get('title') or '圖例')}</h3><table cellspacing=0 cellpadding=3>")
        for c in cls:
            col = str(c.get('color') or '')
            sw = (f"<span style='background:{_esc(col)};'>&nbsp;&nbsp;&nbsp;&nbsp;</span>" if col.startswith('#')
                  else '&nbsp;&nbsp;&nbsp;&nbsp;')
            out.append(f'<tr><td>{sw}</td><td>{_esc(c.get("label"))}</td></tr>')
        out.append('</table>')
        notes = lg.get('notes') or []
        if notes:
            out.append('<ul>' + ''.join(f'<li>{_esc(n)}</li>' for n in notes) + '</ul>')

    # ── 假設與限制 ──
    assumptions = [str(a) for a in (s.get('assumptions') or [])]
    out.append('<h3>假設與限制（模型偏樂觀）</h3>')
    if assumptions:
        out.append('<ul>' + ''.join(f'<li>{_esc(a)}</li>' for a in assumptions) + '</ul>')
    else:
        out.append('<p>（預覽沒有附上假設清單）</p>')

    # ── 精度：文獻值與條件 ──
    out.append('<h3>預期精度（只列文獻值與其條件）</h3>')
    acc = [a for a in (s.get('accuracy_literature') or []) if isinstance(a, dict)]
    if acc:
        out.append('<table cellspacing=0 cellpadding=3 border=1><tr><th>情境</th><th>平面</th><th>高程</th>'
                   '<th>條件</th></tr>'
                   + ''.join(f"<tr><td>{_cell(a.get('case'))}</td><td>{_cell(a.get('horizontal'))}</td>"
                             f"<td>{_cell(a.get('vertical'))}</td><td>{_cell(a.get('condition'))}</td></tr>"
                             for a in acc) + '</table>')
    note = str(s.get('accuracy_note') or '').strip() or ACCURACY_UNKNOWN_FALLBACK
    out.append(f"<p style='color:#b71c1c;'><b>{_esc(note)}</b></p>")

    # ── 看得見的地物 ──
    feats = [f for f in (s.get('visible_features') or []) if isinstance(f, dict)]
    if feats:
        out.append('<h3>地物在影像上約幾個像素（以中位 GSD 推算）</h3>'
                   '<table cellspacing=0 cellpadding=3 border=1><tr><th>地物</th><th>尺寸</th><th>像素</th></tr>'
                   + ''.join(f"<tr><td>{_esc(f.get('feature'))}</td><td>{_fmt(f.get('size_cm'), 'g', ' cm')}</td>"
                             f"<td>{_fmt(f.get('pixels'), '.1f', ' px')}</td></tr>" for f in feats) + '</table>')

    # ── 破洞清單 ──
    holes = list(getattr(preview, 'interior_holes', None) or [])
    edge = list(getattr(preview, 'edge_short', None) or [])
    if holes:
        src_txt = {'gate': '（上傳閘門格判定，會擋上傳）', 'gate_centroid': '（上傳閘門摘要：只有中心位置，會擋上傳）',
                   'preview_only': '（只在預覽細格看到，閘門沒擋，建議檢查）',
                   'preview_grid': '（預覽格；能不能上傳以閘門為準）'}
        out.append(f'<h3>內部破洞（地圖紅框）{len(holes)} 處</h3><ul>'
                   + ''.join(f"<li>{_fmt(h.get('area_m2'), ',.0f', ' m²')}，最少 {_fmt(h.get('min_views'), '.0f')} 視"
                             f"{_esc(src_txt.get(h.get('source'), ''))}</li>"
                             for h in holes[:20])
                   + ('<li>…</li>' if len(holes) > 20 else '') + '</ul>')
    if edge:
        out.append(f'<p>邊緣帶不足 {len(edge)} 處（只顯示不擋上傳）。</p>')
    return ''.join(out)


def _jsonable(v: Any) -> Any:
    """轉成 JSON 安全的值：tuple → list、numpy 純量 → Python、非有限浮點 → None、bytes 一律丟掉。"""
    if isinstance(v, (bytes, bytearray, memoryview)):
        return None
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    if isinstance(v, int):
        return int(v)
    if hasattr(v, 'item') and callable(getattr(v, 'item')):      # numpy 純量
        try:
            return _jsonable(v.item())
        except Exception:                                       # noqa: BLE001
            return None
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    return f if math.isfinite(f) else None


#: 匯出的 PNG 是什麼（JSON 裡寫明；raster_* 欄位描述的不是它）
PNG_NOTE = ('同名 PNG 是示意圖（四邊留白、圖例、比例尺、指北、橫幅，經緯比例依 cos(緯度) 校正），'
            '不是地理參照影像；raster_bounds_wsen／raster_px／raster_cell_m 描述的是內部的分級格網（沒有匯出）。')


def preview_export_dict(preview, *, stale_reason: Optional[str] = None) -> dict:
    """匯出 JSON 的內容：統計、圖例、外框／破洞環（不含任何影像位元組、不含 DEM 衍生的地形陰影）。

    ``stale_reason``：這份預覽已不對應目前的計畫（對話框被標成過期）⇒ ``stale``／``stale_reason`` 與一句聲明。"""
    s = _stats(preview)
    mb_lo, mb_hi = _mb_range(s)
    disclaimer = [
        f'{PREDICTION_BANNER}：由規劃航線、相機與規劃 DEM 模擬每格被拍到的張數，不是飛完的成果。',
        str(s.get('accuracy_note') or '').strip() or ACCURACY_UNKNOWN_FALLBACK,
        f'資料量為範圍：假設每張 {mb_lo:g}–{mb_hi:g} MB'
        + (f'（{s.get("data_mb_basis")}）' if s.get('data_mb_basis') else '') + '。',
    ]
    if stale_reason:
        disclaimer.append(f'⚠ 匯出時這份預覽已不對應目前的計畫（{stale_reason}）：只供對照舊計畫。')
    return _jsonable({
        'kind': 'hdmap_preview',
        'label': f'預計成果（{PREDICTION_BANNER}）',
        'exported_at': _dt.datetime.now().isoformat(timespec='seconds'),
        'stale': bool(stale_reason),
        'stale_reason': (str(stale_reason) if stale_reason else None),
        'disclaimer': disclaimer,
        'png_note': PNG_NOTE,
        'raster_bounds_wsen': list(getattr(preview, 'bounds_wsen', None) or []),
        'raster_px': list(getattr(preview, 'image_px', None) or []),
        'raster_cell_m': getattr(preview, 'cell_size_m', None),
        'stats': s,
        'legend': _legend(preview),
        'extent_rings': getattr(preview, 'extent_rings', None) or [],
        'extent_holes': getattr(preview, 'extent_holes', None) or [],
        'interior_holes': getattr(preview, 'interior_holes', None) or [],
        'edge_short': getattr(preview, 'edge_short', None) or [],
        'survey_rings': getattr(preview, 'survey_rings', None) or [],
        'photos_count': len(getattr(preview, 'photos', None) or []),
        'photos_note': '拍照點是均勻抽樣（張數見 stats.photos_total）' if (
            _num(s.get('photos_total')) is not None
            and len(getattr(preview, 'photos', None) or []) < float(s.get('photos_total'))) else None,
        'terrain_shading_in_png': getattr(preview, 'hillshade_png', None) is not None,
    })


def export_target_paths(path) -> Tuple[Path, Path]:
    """使用者選的檔名 → (PNG 路徑, 同名 JSON 路徑)。``.png`` 原樣；``.json`` 換成 ``.png``（不會變成
    x.json.png／x.json.json）；其他（含沒有副檔名）一律補 ``.png``。"""
    p = Path(path)
    suf = p.suffix.lower()
    if suf == '.png':
        png = p
    elif suf == '.json':
        png = p.with_suffix('.png')
    else:
        png = p.with_name(p.name + '.png')
    return png, png.with_suffix('.json')


def export_preview_files(preview, png_bytes: bytes, png_path, *, overwrite: bool = False,
                         stale_reason: Optional[str] = None) -> Tuple[Path, Path]:
    """把地圖 PNG 與同名 .json 寫出去；回傳 (png 路徑, json 路徑)。

    ``overwrite=False``：兩個目標任一已存在就丟 FileExistsError、什麼都不寫（存檔對話框只問過使用者選的那一個；
    對話框會先問過再傳 True）。
    ⚠ 地形是 Cesium World Terrain 卻帶著地形陰影 ⇒ 拒絕（ValueError）：授權只准記憶體內使用，
    而 PNG 底圖就畫著那份陰影（照契約這時 hillshade 應為 None；這裡是第二道防線）。
    """
    if not png_bytes:
        raise ValueError('沒有地圖影像可以存（預覽圖產生失敗）')
    if getattr(preview, 'hillshade_png', None) is not None and preview_uses_world_terrain(preview):
        raise ValueError('這份預覽的地形陰影來自 Cesium World Terrain（授權只准在記憶體內使用），不能存檔')
    png, js = export_target_paths(png_path)
    if not overwrite:
        exist = [str(p) for p in (png, js) if p.exists()]
        if exist:
            raise FileExistsError('已有同名檔案（沒有覆寫）：' + '、'.join(exist))
    data = json.dumps(preview_export_dict(preview, stale_reason=stale_reason), ensure_ascii=False, indent=1,
                      allow_nan=False)
    png.parent.mkdir(parents=True, exist_ok=True)
    png.write_bytes(bytes(png_bytes))
    js.write_text(data, encoding='utf-8')
    logger.info(f'[高精地圖預覽] 已匯出 {png}（{len(png_bytes):,} B）與 {js}')
    return png, js


# ═════════════════════════════════════════════════════════════════════
#  對話框
# ═════════════════════════════════════════════════════════════════════
class _ScaledImage(QLabel):
    """等比例縮放顯示一張 PNG（視窗拉大縮小都跟著縮放，不裁切）。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pm: Optional[QPixmap] = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_png(self, data: bytes) -> bool:
        pm = QPixmap()
        if not data or not pm.loadFromData(bytes(data), 'PNG'):
            self._pm = None
            return False
        self._pm = pm
        self._rescale()
        return True

    def source_size(self) -> Tuple[int, int]:
        return (0, 0) if self._pm is None else (self._pm.width(), self._pm.height())

    def _rescale(self) -> None:
        if self._pm is None:
            return
        self.setPixmap(self._pm.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                       Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, ev):          # noqa: N802 - Qt 介面
        super().resizeEvent(ev)
        self._rescale()


class HdmapPreviewDialog(QDialog):
    """非模態：左＝成果分級圖，右＝統計卡；匯出 PNG＋JSON。"""

    def __init__(self, preview, parent=None, *, map_png: Optional[bytes] = None):
        super().__init__(parent)
        self.preview = preview
        self._stale_reason: Optional[str] = None
        self.setWindowTitle(DIALOG_TITLE)
        self.setModal(False)
        self.resize(1280, 760)
        self._map_png: bytes = b''
        self._map_error: Optional[str] = None
        if map_png is None:
            try:
                from survey.preview import render_preview_map_png
                map_png = render_preview_map_png(preview)
            except Exception as e:                                # noqa: BLE001 - 畫圖失敗也要能看統計
                self._map_error = f'{type(e).__name__}: {e}'
                logger.error(f'[高精地圖預覽] 地圖影像產生失敗：{self._map_error}', exc_info=True)
                map_png = b''
        self._map_png = bytes(map_png or b'')

        lay = QVBoxLayout(self)
        self.banner = QLabel(f'⚠ {PREDICTION_BANNER}：顏色＝每格預計被幾張照片拍到（模擬），不是正射影像；'
                             '模型沒有建模 CAM_MAX_ROLL 側傾門檻與轉彎動態，結果偏樂觀。')
        self.banner.setWordWrap(True)
        self.banner.setStyleSheet('color:#b71c1c;font-weight:bold;')
        lay.addWidget(self.banner)
        self.stale_label = QLabel('')
        self.stale_label.setWordWrap(True)
        self.stale_label.setStyleSheet('color:#e65100;font-weight:bold;')
        self.stale_label.setVisible(False)
        lay.addWidget(self.stale_label)

        split = QSplitter(Qt.Orientation.Horizontal, self)
        left = QWidget(split)
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        self.map_label = _ScaledImage(left)
        if not self.map_label.set_png(self._map_png):
            self.map_label.setText('地圖影像無法顯示' + (f'：{self._map_error}' if self._map_error else ''))
            self.map_label.setWordWrap(True)
        ll.addWidget(self.map_label, 1)
        cap = QLabel('北朝上；紅框＝內部破洞（上傳閘門判定）；紅點框＝只在預覽細格看到（閘門沒擋）；'
                     '白線＝預計可成圖範圍；點＝模擬拍照位置（依機色，可能是抽樣）。')
        cap.setWordWrap(True)
        ll.addWidget(cap)
        self.stats_browser = QTextBrowser(split)
        self.stats_browser.setOpenExternalLinks(False)
        self.stats_browser.setHtml(preview_html(preview))
        split.addWidget(left)
        split.addWidget(self.stats_browser)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        lay.addWidget(split, 1)

        row = QHBoxLayout()
        row.addStretch(1)
        self.btn_export = QPushButton('匯出 PNG＋JSON…')
        self.btn_export.setToolTip('存出左邊這張圖（PNG）與同名 JSON（統計、圖例、範圍與破洞環）；兩者都標明是規劃預測')
        self.btn_export.setEnabled(bool(self._map_png))
        self.btn_export.clicked.connect(self._on_export)
        row.addWidget(self.btn_export)
        self.btn_close = QPushButton('關閉')
        self.btn_close.clicked.connect(self.close)
        row.addWidget(self.btn_close)
        lay.addLayout(row)

    @property
    def map_png(self) -> bytes:
        return self._map_png

    def mark_stale(self, text: str) -> None:
        """MainWindow 呼叫：計畫已變更／清除 ⇒ 這份預覽不再對應目前的計畫（留著看，但說出來；匯出時也說）。"""
        self._stale_reason = str(text)
        self.stale_label.setText(f'⚠ {text}')
        self.stale_label.setVisible(True)
        self.btn_export.setToolTip('這份預覽已不對應目前的計畫：仍可匯出，但 JSON 會標明 stale（過期），只供對照舊計畫')

    @property
    def stale_reason(self) -> Optional[str]:
        return self.__dict__.get('_stale_reason')

    def _default_export_name(self) -> str:
        return (f'hdmap_preview_{_dt.datetime.now():%Y%m%d_%H%M%S}'
                + ('_stale' if self.stale_reason else '') + '.png')

    def _ask_export_path(self) -> str:
        path, _f = QFileDialog.getSaveFileName(self, '匯出預計成果（PNG＋同名 JSON）', self._default_export_name(),
                                               'PNG 圖檔 (*.png)')
        return path

    def _ask_yes(self, title: str, text: str) -> bool:
        ans = QMessageBox.question(self, title, text, QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                   QMessageBox.StandardButton.No)
        return ans == QMessageBox.StandardButton.Yes

    def _on_export(self) -> None:
        path = self._ask_export_path()
        if not path:
            return
        stale = self.stale_reason
        if stale and not self._ask_yes('匯出預計成果',
                                       f'這份預覽已不對應目前的計畫（{stale}）。\n\n仍要匯出嗎？'
                                       '匯出的 JSON 會標明 stale（過期），只供對照舊計畫。'):
            return
        png, js = export_target_paths(path)
        # 存檔對話框只問過使用者選的那一個檔名；另一個（同名 JSON，或改過副檔名的 PNG）已存在要另外問
        chosen = Path(path)
        others = [p for p in (png, js) if p.exists() and p != chosen]
        if others and not self._ask_yes('匯出預計成果', '下列檔案已存在，要覆寫嗎？\n\n' + '\n'.join(map(str, others))):
            return
        try:
            png, js = export_preview_files(self.preview, self._map_png, path, overwrite=True, stale_reason=stale)
        except (OSError, ValueError) as e:
            logger.error(f'[高精地圖預覽] 匯出失敗：{e}')
            QMessageBox.warning(self, '匯出預計成果', f'沒有存檔：{e}')
            return
        QMessageBox.information(self, '匯出預計成果', f'已存：\n{png}\n{js}\n\n（兩者都是{PREDICTION_BANNER}'
                                + ('；已標明過期' if stale else '') + '）')
