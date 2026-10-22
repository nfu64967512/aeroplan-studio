"""預計成果圖層（高精地圖「規劃預測」）—— 2D Leaflet 與 3D Cesium 共用的 Python 端：payload 檢查、頁面膠水 JS、重建時套回。

資料來源是 ``survey.preview.HdmapPreview.to_js_payload()``（規劃預測，**不是**實際成果）；這裡只負責「畫」：

    setHdmapPreview(id, p)   clearHdmapPreview(id|null)   setHdmapPreviewVisible(id, bool)   hdmapPreviewInfo()

Python 端（MapWidget／CesiumMapWidget／DualMapWidget）只呼叫這四個頁面函式；payload 格式（全部 JSON-safe）：

    {'id', 'label', 'bounds': [W, S, E, N]（格子邊緣，度）, 'image': 'data:image/png;base64,…', 'image_px': [w, h],
     'opacity', 'mask_alpha', 'extent_rings', 'extent_holes', 'holes': [{'ring', 'label', 'kind'（可省）}],
     'edge_short': [{'ring', 'label'}], 'survey_rings', 'points': [[lat, lon, color_index]], 'point_colors': ['#RRGGBB'],
     'legend': {'title', 'classes': [{'label', 'color'|None}], 'notes': [str], 'points'（可省）: [{'label', 'color'}]},
     'stats_line'}

每個 ring 是 ``[[lat, lon], …]``（可閉合可不閉合）。數字收 numpy 純量（轉成 Python float／int 再送出）。
``holes[].kind``：``'hole'``（預設；上傳閘門判定的內部破洞）、``'preview_only'``（只在預覽細格看到、閘門沒擋）、
``'marker'``（閘門摘要只給中心：圓圈標位置）。

畫法（兩邊同一套語意）：
  * 品質分級點陣圖：2D ``L.imageOverlay``（最近鄰放大）／3D ``SingleTileImageryProvider`` 的 ImageryLayer（NEAREST），
    疊在底圖上、路徑之下；3D 圖層帶 ``_hdmapPreview`` 旗標 ⇒ 圖層面板的「地圖」開關不會把它一起關掉。
  * 預計成圖範圍外變暗：一個多邊形（外框＝很大的框，範圍外框＝洞、範圍內的洞＝島，奇偶填色）。
    **不吃滑鼠**：2D interactive:false；3D 是 allowPicking:false 的 GroundPrimitive（entity 會被 Viewer 預設的
    點選／雙擊拿去開資訊框、飛到 49 km 高）。
  * 外框白線、內部漏洞紅虛線（只在預覽細格看到的紅點線）、邊緣不足橘虛線、規劃作業區藍虛線、拍照點小圓點。
    2D 的漏洞說明：**不放任何吃滑鼠的圖層**（隱形命中多邊形會搶走規劃路徑／衝突點／禁航區的懸停與點擊），
    改由地圖的 mousemove 自己判斷點在哪個多邊形內；游標下已有別的互動圖層時不顯示。3D 的說明在 entity 名稱。
  * 圖例：標題寫明「規劃預測，非實際成果」（固定字樣，不靠 payload）；說明預設收起（點標題列展開）；
    高度上限＝可用空間（2D 不蓋左上縮放鈕、3D 不蓋圖層面板），放不下才可捲動。

⚠ JSON 會被放進行內 <script>（2D 整頁重建、3D 範本填值）⇒ ``<``／``>``／``&`` 一律轉成 \\uXXXX，``</script>`` 不可能出現。
⚠ 膠水 JS 不可以含 ``__大寫__`` 形式的字（3D 範本的佔位字檢查會把它當成沒換掉的佔位）。
"""
from __future__ import annotations

import json
import math
import numbers
import re
from typing import Any, Iterable

PAGE_FUNCTIONS = ('setHdmapPreview', 'clearHdmapPreview', 'setHdmapPreviewVisible', 'hdmapPreviewInfo')
#: 與 survey.preview.PREVIEW_MAX_POINTS 同一個上限（payload 由 A 端先抽稀；這裡只擋超量，不偷偷截斷）
PREVIEW_MAX_POINTS = 5000
#: data URL 的長度上限（2048×2048 RGBA PNG 的最壞情況約 22 MB base64）
IMAGE_MAX_CHARS = 32 * 1024 * 1024
LABEL_DEFAULT = '預計成果（規劃預測）'
#: 圖例上一定會出現的固定字樣（使用者決策：一切標成規劃預測）
PREDICTION_BANNER = '規劃預測，非實際成果'

#: 向量圖層的顏色（2D/3D 同一份；圖例也從這裡拿）
STYLE = {
    'extent': '#FFFFFF',        # 預計成圖範圍外框
    'hole': '#FF1744',          # 內部漏洞
    'edge': '#FF9100',          # 邊緣不足
    'survey': '#90CAF9',        # 規劃作業區
}
#: 圖例裡向量圖層的說明（只列出這次真的有畫的）
STYLE_LABELS = {
    'extent': '白線：預計成圖範圍外框',
    'hole': '紅虛線：內部漏洞（預計不會成圖）',
    'hole_preview': '紅點線：只在預覽細格看到的視數不足（閘門沒擋，建議檢查）',
    'hole_marker': '紅圓圈：上傳閘門回報的破洞中心（形狀以閘門為準）',
    'edge': '橘虛線：邊緣張數不足',
    'survey': '藍虛線：規劃作業區',
    'points': '小圓點：預計拍照位置（依機上色）',
}
#: holes[].kind 可用的值（其他值整份拒絕；沒給 ＝ 'hole'）
HOLE_KINDS = ('hole', 'preview_only', 'marker')

_HEX = re.compile(r'^#[0-9A-Fa-f]{6}$')
_DATA_PNG = 'data:image/png;base64,'
_B64 = re.compile(r'^[A-Za-z0-9+/]*={0,2}$')


# ─────────────────────────────────────────────────────────────────────
#  payload 檢查（寧可整份拒絕，也不畫出一半讓人以為那就是全部）
# ─────────────────────────────────────────────────────────────────────
def _num(x: Any, what: str) -> float:
    # numbers.Real：numpy 的 float32／int64 也收（轉成 Python float 再出去）；bool 不算數字
    if isinstance(x, bool) or not isinstance(x, numbers.Real):
        raise ValueError(f'{what} 不是數字：{x!r}')
    v = float(x)
    if not math.isfinite(v):
        raise ValueError(f'{what} 不是有限數：{x!r}')
    return v


def _unit(x: Any, what: str) -> float:
    v = _num(x, what)
    if not 0.0 <= v <= 1.0:
        raise ValueError(f'{what} 要在 0–1：{x!r}')
    return v


def _color(c: Any, what: str, *, allow_none: bool = False):
    if c is None and allow_none:
        return None
    if not isinstance(c, str) or not _HEX.match(c):
        raise ValueError(f"{what} 要是 '#RRGGBB'：{c!r}")
    return c.upper()


def _text(s: Any, what: str, *, default: str | None = None) -> str:
    if s is None and default is not None:
        return default
    if not isinstance(s, str):
        raise ValueError(f'{what} 要是字串：{s!r}')
    return s


def _ring(r: Any, what: str) -> list:
    """[[lat, lon], …] → 檢查過的 float 列（四捨五入到 1e-7 度 ≈ 1 cm，縮小 JSON）；少於 3 個不同點就拒絕。"""
    if not isinstance(r, (list, tuple)):
        raise ValueError(f'{what} 要是 [[lat, lon], …]')
    out = []
    for i, p in enumerate(r):
        if not isinstance(p, (list, tuple)) or len(p) != 2:
            raise ValueError(f'{what} 第 {i} 點不是 [lat, lon]：{p!r}')
        lat, lon = _num(p[0], f'{what} 第 {i} 點緯度'), _num(p[1], f'{what} 第 {i} 點經度')
        if abs(lat) > 90 or abs(lon) > 180:
            raise ValueError(f'{what} 第 {i} 點超出經緯度範圍：{p!r}')
        out.append([round(lat, 7), round(lon, 7)])
    if len({(a, b) for a, b in out}) < 3:
        raise ValueError(f'{what} 少於 3 個不同的點')
    return out


def _rings(rs: Any, what: str) -> list:
    if rs is None:
        return []
    if not isinstance(rs, (list, tuple)):
        raise ValueError(f'{what} 要是 ring 的串列')
    return [_ring(r, f'{what}[{i}]') for i, r in enumerate(rs)]


def _labelled_rings(items: Any, what: str, *, kinds: tuple | None = None) -> list:
    if items is None:
        return []
    if not isinstance(items, (list, tuple)):
        raise ValueError(f'{what} 要是 [{{ring, label}}, …]')
    out = []
    for i, it in enumerate(items):
        if not isinstance(it, dict):
            raise ValueError(f'{what}[{i}] 要是 {{ring, label}}：{it!r}')
        row = {'ring': _ring(it.get('ring'), f'{what}[{i}].ring'),
               'label': _text(it.get('label'), f'{what}[{i}].label', default='')}
        if kinds is not None:
            k = it.get('kind')
            k = kinds[0] if k is None else k
            if k not in kinds:
                raise ValueError(f'{what}[{i}].kind 只能是 {list(kinds)}：{k!r}')
            row['kind'] = k
        out.append(row)
    return out


def check_bounds_wsen(b: Any) -> list:
    if not isinstance(b, (list, tuple)) or len(b) != 4:
        raise ValueError(f'bounds 要是 [西, 南, 東, 北]：{b!r}')
    w, s, e, n = (_num(x, 'bounds') for x in b)
    if not (-180 <= w < e <= 180 and -90 <= s < n <= 90):
        raise ValueError(f'bounds 不合法（要 西<東、南<北、在經緯度範圍內）：{b!r}')
    return [w, s, e, n]


def check_layer_id(layer_id: Any) -> str:
    if not isinstance(layer_id, str) or not layer_id.strip():
        raise ValueError(f'圖層代號要是非空字串，收到 {layer_id!r}')
    return layer_id


def check_payload(payload: Any, layer_id: str | None = None) -> dict:
    """檢查 ``HdmapPreview.to_js_payload()`` 的輸出，回傳可直接 JSON 化（allow_nan=False）的新 dict。

    ``layer_id`` 給了就以它為準（覆寫 payload 的 'id'）。任何一欄不合法都丟 ValueError（訊息是中文、說哪一欄）。
    """
    if not isinstance(payload, dict):
        raise ValueError('預計成果 payload 要是 dict（HdmapPreview.to_js_payload() 的輸出）')
    lid = check_layer_id(layer_id if layer_id is not None else payload.get('id'))
    img = payload.get('image')
    if not isinstance(img, str) or not img.startswith(_DATA_PNG):
        raise ValueError("image 要是 'data:image/png;base64,…'（品質分級 PNG）")
    if len(img) > IMAGE_MAX_CHARS:
        raise ValueError(f'image 太大（{len(img)} 字元 > {IMAGE_MAX_CHARS}）；點陣圖每邊上限 2048 px')
    if not _B64.match(img[len(_DATA_PNG):]) or len(img) == len(_DATA_PNG):
        raise ValueError('image 的 base64 內容不合法或是空的')
    px = payload.get('image_px')
    if px is not None:
        if not isinstance(px, (list, tuple)) or len(px) != 2:
            raise ValueError(f'image_px 要是 [寬, 高]：{px!r}')
        pw, ph = (_num(v, 'image_px') for v in px)
        if pw < 1 or ph < 1 or pw != int(pw) or ph != int(ph):
            raise ValueError(f'image_px 要是正整數：{px!r}')
        px = [int(pw), int(ph)]
    colors = payload.get('point_colors') or []
    if not isinstance(colors, (list, tuple)):
        raise ValueError('point_colors 要是 [#RRGGBB, …]')
    colors = [_color(c, f'point_colors[{i}]') for i, c in enumerate(colors)]
    pts_in = payload.get('points') or []
    if not isinstance(pts_in, (list, tuple)):
        raise ValueError('points 要是 [[lat, lon, color_index], …]')
    if len(pts_in) > PREVIEW_MAX_POINTS:
        raise ValueError(f'points 有 {len(pts_in)} 點，超過上限 {PREVIEW_MAX_POINTS}（應由 survey.preview 先均勻抽稀）')
    pts = []
    for i, p in enumerate(pts_in):
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            raise ValueError(f'points[{i}] 不是 [lat, lon, color_index]：{p!r}')
        lat, lon = _num(p[0], f'points[{i}] 緯度'), _num(p[1], f'points[{i}] 經度')
        if abs(lat) > 90 or abs(lon) > 180:
            raise ValueError(f'points[{i}] 超出經緯度範圍：{p!r}')
        ci = p[2]
        if isinstance(ci, bool) or not isinstance(ci, numbers.Integral) or not 0 <= int(ci) < len(colors):
            raise ValueError(f'points[{i}] 的 color_index {ci!r} 不在 point_colors（{len(colors)} 色）範圍內')
        pts.append([round(lat, 7), round(lon, 7), int(ci)])
    leg = payload.get('legend') or {}
    if not isinstance(leg, dict):
        raise ValueError('legend 要是 {title, classes, notes}')
    classes = []
    for i, c in enumerate(leg.get('classes') or []):
        if not isinstance(c, dict):
            raise ValueError(f'legend.classes[{i}] 要是 {{label, color}}')
        classes.append({'label': _text(c.get('label'), f'legend.classes[{i}].label'),
                        'color': _color(c.get('color'), f'legend.classes[{i}].color', allow_none=True)})
    notes = [_text(n, f'legend.notes[{i}]') for i, n in enumerate(leg.get('notes') or [])]
    # 可選：拍照點每架一色的圖例（survey.preview 會放 [{label: 'UAV-1', color}]）
    lpts = []
    for i, c in enumerate(leg.get('points') or []):
        if not isinstance(c, dict):
            raise ValueError(f'legend.points[{i}] 要是 {{label, color}}')
        lpts.append({'label': _text(c.get('label'), f'legend.points[{i}].label'),
                     'color': _color(c.get('color'), f'legend.points[{i}].color')})
    out = {
        'id': lid,
        'label': _text(payload.get('label'), 'label', default=LABEL_DEFAULT) or LABEL_DEFAULT,
        'bounds': check_bounds_wsen(payload.get('bounds')),
        'image': img,
        'image_px': px,
        'opacity': _unit(payload.get('opacity', 0.45), 'opacity'),
        'mask_alpha': _unit(payload.get('mask_alpha', 0.55), 'mask_alpha'),
        'extent_rings': _rings(payload.get('extent_rings'), 'extent_rings'),
        'extent_holes': _rings(payload.get('extent_holes'), 'extent_holes'),
        'holes': _labelled_rings(payload.get('holes'), 'holes', kinds=HOLE_KINDS),
        'edge_short': _labelled_rings(payload.get('edge_short'), 'edge_short'),
        'survey_rings': _rings(payload.get('survey_rings'), 'survey_rings'),
        'points': pts,
        'point_colors': colors,
        'legend': {'title': _text(leg.get('title'), 'legend.title', default=''), 'classes': classes, 'notes': notes,
                   'points': lpts},
        'stats_line': _text(payload.get('stats_line'), 'stats_line', default=''),
    }
    json.dumps(out, allow_nan=False)            # 防呆：一定是合法 JSON
    return out


def summary(p: dict) -> dict:
    """檢查過的 payload → 給呼叫端／log 的計數（不含影像本身）。"""
    return {'id': p['id'], 'bounds': list(p['bounds']), 'image_px': p['image_px'],
            'extent_rings': len(p['extent_rings']), 'extent_holes': len(p['extent_holes']),
            'holes': len(p['holes']), 'edge_short': len(p['edge_short']),
            'holes_preview_only': sum(1 for h in p['holes'] if h.get('kind') == 'preview_only'),
            'hole_markers': sum(1 for h in p['holes'] if h.get('kind') == 'marker'),
            'survey_rings': len(p['survey_rings']), 'points': len(p['points']),
            'image_bytes': len(p['image'])}


# ─────────────────────────────────────────────────────────────────────
#  JS 呼叫與套回
# ─────────────────────────────────────────────────────────────────────
def js_json(obj: Any) -> str:
    """可以安全放進行內 <script> 的 JSON（NaN 直接丟錯；< > & 與 U+2028/2029 轉成 \\uXXXX）。"""
    s = json.dumps(obj, allow_nan=False, ensure_ascii=False, separators=(',', ':'))
    return (s.replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
             .replace('\u2028', '\\u2028').replace('\u2029', '\\u2029'))


def js_call(fn: str, *args: Any) -> str:
    if fn not in PAGE_FUNCTIONS:
        raise ValueError(f'不是預計成果圖層的頁面函式：{fn}')
    return f'{fn}({", ".join(js_json(a) for a in args)})'


def restore_js(store: dict | None, hidden: Iterable[str] | None = None) -> str:
    """頁面重建後（2D 每次 folium 重繪、3D 重新載入）把 Python 端記住的圖層套回去的 JS（沒有就空字串）。"""
    hidden = set(hidden or ())
    parts = []
    for lid, p in (store or {}).items():
        parts.append(js_call('setHdmapPreview', lid, p) + ';')
        if lid in hidden:
            parts.append(js_call('setHdmapPreviewVisible', lid, False) + ';')
    return '\n'.join(parts)


# ─────────────────────────────────────────────────────────────────────
#  頁面膠水
# ─────────────────────────────────────────────────────────────────────
_GLUE_COMMON = r"""
var _HDMAP_STYLE = %STYLE%;
var _HDMAP_STYLE_LABELS = %STYLE_LABELS%;
var _HDMAP_BANNER = %BANNER%;
var _hdmapRecs = {};
// 圖例的「說明」預設收起（全部也在預覽對話框裡）；點圖例標題列展開／收起（所有圖層一起）
var _hdmapNotesOpen = false;
function _hdmapStop(ev) { ev.stopPropagation(); }
// 圖例：全部用 DOM＋textContent 組（標籤是資料，不可以當 HTML）；只用行內樣式
function _hdmapLegendEl(id, p, counts) {
  var box = document.createElement('div');
  box.className = 'hdmap-preview-legend';
  box.setAttribute('data-hdmap-id', id);
  box.style.cssText = 'background:rgba(13,17,23,0.86);color:#e8eef5;padding:6px 9px;border-radius:6px;'
    + 'font:11px "Microsoft JhengHei","Segoe UI",sans-serif;line-height:1.35;max-width:280px;'
    + 'box-shadow:0 1px 6px rgba(0,0,0,0.45);pointer-events:none;text-align:left;box-sizing:border-box;';
  // 放不下而可以捲動時圖例會吃滑鼠：點擊／滾輪／拖曳不可以穿到地圖（加角點、縮放、平移）
  ['click', 'dblclick', 'mousedown', 'pointerdown', 'touchstart', 'wheel', 'contextmenu'].forEach(function (e) {
    box.addEventListener(e, _hdmapStop); });
  function line(text, css, parent) {
    var d = document.createElement('div'); d.textContent = text; if (css) d.style.cssText = css;
    (parent || box).appendChild(d); return d; }
  var lg = p.legend || {}, notes = lg.notes || [];
  // 標題列：圖層名＋說明開關（只有這一列吃滑鼠）
  var head = document.createElement('div');
  head.className = 'hdmap-legend-head';
  head.style.cssText = 'display:flex;align-items:baseline;justify-content:space-between;gap:8px;'
    + (notes.length ? 'cursor:pointer;pointer-events:auto;' : '');
  var ttl = document.createElement('span'); ttl.textContent = p.label || '預計成果';
  ttl.style.cssText = 'font-weight:bold;font-size:12px;color:#ffffff;';
  var tog = document.createElement('span'); tog.className = 'hdmap-legend-toggle';
  tog.style.cssText = 'color:#90caf9;font-size:10px;white-space:nowrap;';
  head.appendChild(ttl); head.appendChild(tog); box.appendChild(head);
  if (notes.length) {
    head.addEventListener('click', function (ev) {
      ev.stopPropagation(); ev.preventDefault();
      _hdmapNotesOpen = !_hdmapNotesOpen;
      _hdmapSyncLegends();
    });
  }
  line('⚠ ' + _HDMAP_BANNER, 'color:#ffcc80;font-weight:bold;');
  if (lg.title) line(lg.title, 'margin-top:3px;color:#b0bec5;');
  (lg.classes || []).forEach(function (c) {
    var row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;gap:6px;margin:1px 0;';
    var sw = document.createElement('span');
    sw.style.cssText = 'display:inline-block;width:12px;height:12px;flex:0 0 12px;border:1px solid rgba(255,255,255,0.6);'
      + 'background:' + (c.color || 'transparent') + ';';
    var tx = document.createElement('span'); tx.textContent = c.label;
    row.appendChild(sw); row.appendChild(tx); box.appendChild(row);
  });
  function styleRow(key, color, dash) {
    var row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;gap:6px;margin:1px 0;';
    var sw = document.createElement('span');
    sw.style.cssText = 'display:inline-block;width:12px;height:0;flex:0 0 12px;border-top:2px '
      + dash + ' ' + color + ';';
    var tx = document.createElement('span'); tx.textContent = _HDMAP_STYLE_LABELS[key];
    row.appendChild(sw); row.appendChild(tx); box.appendChild(row);
  }
  if (counts.extentRings) styleRow('extent', _HDMAP_STYLE.extent, 'solid');
  if (counts.holesGate) styleRow('hole', _HDMAP_STYLE.hole, 'dashed');
  if (counts.holesFine) styleRow('hole_preview', _HDMAP_STYLE.hole, 'dotted');
  if (counts.markers) styleRow('hole_marker', _HDMAP_STYLE.hole, 'solid');
  if (counts.edgeShort) styleRow('edge', _HDMAP_STYLE.edge, 'dashed');
  if (counts.surveyRings) styleRow('survey', _HDMAP_STYLE.survey, 'dashed');
  if (counts.points) {
    line(_HDMAP_STYLE_LABELS.points, 'margin-top:1px;');
    (lg.points || []).forEach(function (c) {
      var row = document.createElement('div');
      row.style.cssText = 'display:flex;align-items:center;gap:6px;margin:1px 0 1px 8px;';
      var sw = document.createElement('span');
      sw.style.cssText = 'display:inline-block;width:7px;height:7px;flex:0 0 7px;border-radius:50%;background:' + c.color + ';';
      var tx = document.createElement('span'); tx.textContent = c.label;
      row.appendChild(sw); row.appendChild(tx); box.appendChild(row);
    });
  }
  if (p.stats_line) line(p.stats_line, 'margin-top:3px;color:#cfd8dc;');
  var nb = document.createElement('div');
  nb.className = 'hdmap-legend-notes';
  notes.forEach(function (n) { line(n, 'margin-top:2px;color:#90a4ae;font-size:10px;', nb); });
  box.appendChild(nb);
  box._hdmapSync = function () {
    nb.style.display = (_hdmapNotesOpen && notes.length) ? 'block' : 'none';
    tog.textContent = notes.length ? (_hdmapNotesOpen ? '收起說明 ▴' : '說明 ' + notes.length + ' 條 ▾') : '';
  };
  box._hdmapSync();
  return box;
}
// 高度上限＝可用空間；放不下才可捲動（這時才吃滑鼠），放得下就不吃滑鼠（點擊穿過到地圖）
function _hdmapFitBox(el, avail) {
  el.style.maxHeight = Math.max(48, Math.floor(avail)) + 'px';
  el.style.overflowY = 'auto';
  el.style.pointerEvents = (el.scrollHeight > el.clientHeight + 1) ? 'auto' : 'none';
}
function _hdmapSyncLegends() {
  Object.keys(_hdmapRecs).forEach(function (k) { var r = _hdmapRecs[k]; if (r.legendEl && r.legendEl._hdmapSync) r.legendEl._hdmapSync(); });
  _hdmapFitLegends();
}
function _hdmapCounts(p) {
  var hs = p.holes || [];
  function kindOf(h) { return h.kind || 'hole'; }
  return { extentRings: (p.extent_rings || []).length, extentHoles: (p.extent_holes || []).length,
           holes: hs.length, holesGate: hs.filter(function (h) { return kindOf(h) === 'hole'; }).length,
           holesFine: hs.filter(function (h) { return kindOf(h) === 'preview_only'; }).length,
           markers: hs.filter(function (h) { return kindOf(h) === 'marker'; }).length,
           edgeShort: (p.edge_short || []).length,
           surveyRings: (p.survey_rings || []).length, points: (p.points || []).length };
}
// 漏洞的畫法（2D／3D 同一套）：[線寬, 虛線樣式（2D dashArray；null＝實線）, 填色透明度]
function _hdmapHoleStyle(kind) {
  if (kind === 'preview_only') return [1.5, '1,4', 0.04];
  if (kind === 'marker') return [2, null, 0.06];
  return [2, '4,3', 0.12];
}
function _hdmapClosed(ring) {
  var r = ring.slice();
  if (r.length && (r[0][0] !== r[r.length - 1][0] || r[0][1] !== r[r.length - 1][1])) r.push(r[0]);
  return r;
}
// 點在多邊形內（射線法；ring = [[lat, lon], …]）
function _hdmapInside(lat, lon, ring) {
  var c = false;
  for (var i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    var yi = ring[i][0], xi = ring[i][1], yj = ring[j][0], xj = ring[j][1];
    if (((yi > lat) !== (yj > lat)) && (lon < (xj - xi) * (lat - yi) / ((yj - yi) || 1e-300) + xi)) c = !c;
  }
  return c;
}
"""

_GLUE_LEAFLET = r"""
var _hdmapCanvasR = null, _hdmapTip = null;
function _hdmapMap() { return window.currentMap || null; }
// 自己的窗格：z-index 350（圖磚 200 之上、路徑 overlayPane 400 之下）、不吃滑鼠；底下再分「影像」「向量」兩層確保影像永遠在遮罩之下。
// ⚠ 不放任何吃滑鼠的圖層（以前在 z 450 放的隱形命中多邊形會搶走規劃路徑 tooltip、衝突點、禁航區 popup）：
//   漏洞說明由地圖的 mousemove 自己判斷（_hdmapOnMove）。
function _hdmapEnsurePanes(map) {
  if (!map.getPane('hdmapPreview')) {
    var root = map.createPane('hdmapPreview');
    root.style.zIndex = '350'; root.style.pointerEvents = 'none';
    var a = map.createPane('hdmapPreviewImg', root); a.style.zIndex = '1'; a.style.pointerEvents = 'none';
    var b = map.createPane('hdmapPreviewVec', root); b.style.zIndex = '2'; b.style.pointerEvents = 'none';
  }
  if (!_hdmapCanvasR) _hdmapCanvasR = L.canvas({pane: 'hdmapPreviewVec', padding: 0.5});
  if (!map._hdmapHoverBound) {
    map.on('mousemove', _hdmapOnMove);
    map.on('mouseout', _hdmapHideTip);
    map.on('resize', _hdmapFitLegends);
    map._hdmapHoverBound = true;
  }
}
function _hdmapBox(ring) {
  var s = 90, w = 180, n = -90, e = -180;
  ring.forEach(function (q) { if (q[0] < s) s = q[0]; if (q[0] > n) n = q[0]; if (q[1] < w) w = q[1]; if (q[1] > e) e = q[1]; });
  return [s, w, n, e];
}
// 看得見的外框在 350 窗格（不吃滑鼠）；說明只登記成懸停目標（不建任何吃滑鼠的圖層）
function _hdmapOutlined(rec, bucket, ring, color, style, label, opt) {
  var l = L.polygon(ring, opt({renderer: _hdmapCanvasR, pane: 'hdmapPreviewVec', color: color, weight: style[0],
    opacity: 1, dashArray: style[1], fill: true, fillColor: color, fillOpacity: style[2], interactive: false}));
  bucket.push(l); rec.group.addLayer(l);
  if (label) rec.hover.push({ ring: ring, box: _hdmapBox(ring), label: label });
}
function _hdmapTipEl(text) { var s = document.createElement('span'); s.textContent = text; return s; }
function _hdmapHideTip() { var map = _hdmapMap(); if (map && _hdmapTip && map.hasLayer(_hdmapTip)) map.closeTooltip(_hdmapTip); }
function _hdmapHoverAt(lat, lon) {
  var map = _hdmapMap(), ids = Object.keys(_hdmapRecs);
  for (var k = 0; k < ids.length; k++) {
    var r = _hdmapRecs[ids[k]];
    if (!r.visible || !map || !map.hasLayer(r.group)) continue;
    for (var i = 0; i < r.hover.length; i++) {
      var t = r.hover[i], b = t.box;
      if (lat >= b[0] && lat <= b[2] && lon >= b[1] && lon <= b[3] && _hdmapInside(lat, lon, t.ring)) return t.label;
    }
  }
  return null;
}
// 游標下已有別的互動圖層（路徑 tooltip、衝突點、禁航區 popup、標記）⇒ 讓它的說明出來，不蓋上漏洞說明。
// Leaflet 的 canvas 繪圖器在滑過互動圖層時會在 canvas 上加 leaflet-interactive（SVG 路徑／標記本身就帶）。
function _hdmapOtherUnder(ev) {
  var t = ev && ev.originalEvent && ev.originalEvent.target;
  if (!t || !t.classList) return false;
  if (t.classList.contains('leaflet-interactive')) return true;
  return !!(t.closest && t.closest('.leaflet-control, .leaflet-popup, .leaflet-tooltip'));
}
function _hdmapOnMove(ev) {
  var map = _hdmapMap();
  if (!map || !ev || !ev.latlng) return;
  var label = _hdmapOtherUnder(ev) ? null : _hdmapHoverAt(ev.latlng.lat, ev.latlng.lng);
  if (!label) { _hdmapHideTip(); return; }
  if (!_hdmapTip) _hdmapTip = L.tooltip({direction: 'top', offset: [0, -10], opacity: 0.95, className: 'hdmap-preview-tip'});
  if (_hdmapTip._hdmapText !== label) { _hdmapTip.setContent(_hdmapTipEl(label)); _hdmapTip._hdmapText = label; }
  _hdmapTip.setLatLng(ev.latlng);
  if (!map.hasLayer(_hdmapTip)) map.openTooltip(_hdmapTip);
}
// 圖例放左下角、在角點計數框（bottom:30px）之上：量計數框實際的上緣，把圖例的 margin-bottom 墊到它上面；
// 高度上限到左上角控制項（縮放鈕）下緣（不蓋住它們）
function _hdmapLiftLegend(rec) {
  var map = _hdmapMap(), el = rec.legendEl;
  if (!map || !el || !el.parentNode) return;
  el.style.marginBottom = '10px';
  var cnt = document.getElementById('corner-counter');
  var mr = map._container.getBoundingClientRect(), lr = el.getBoundingClientRect();
  var need = 96;                                     // 量不到（頁面還沒排版）時的保守值
  if (cnt && mr.height > 0) {
    var cr = cnt.getBoundingClientRect();
    if (cr.height > 0) need = (mr.bottom - cr.top) + 8;
  }
  if (mr.height > 0 && lr.height > 0) {
    var cur = mr.bottom - lr.bottom;
    if (cur < need) el.style.marginBottom = (10 + need - cur) + 'px';
  } else {
    el.style.marginBottom = need + 'px';
  }
  if (mr.height > 0) {
    var top = mr.top + 10, tl = map._container.querySelector('.leaflet-top.leaflet-left');
    if (tl) { var tr = tl.getBoundingClientRect(); if (tr.height > 0) top = Math.max(top, tr.bottom + 8); }
    var lr2 = el.getBoundingClientRect();
    if (lr2.height > 0) _hdmapFitBox(el, lr2.bottom - top);
  }
}
function _hdmapFitLegends() {
  Object.keys(_hdmapRecs).forEach(function (k) { var r = _hdmapRecs[k]; if (r.visible) _hdmapLiftLegend(r); });
}
function _hdmapApply(rec) {
  var map = _hdmapMap();
  if (!map) return;
  if (rec.visible) {
    if (!map.hasLayer(rec.group)) map.addLayer(rec.group);
    if (rec.imageEl()) rec.imageEl().style.imageRendering = 'pixelated';
    if (rec.legendEl) { rec.legendEl.style.display = 'block'; _hdmapLiftLegend(rec); }
  } else {
    if (map.hasLayer(rec.group)) map.removeLayer(rec.group);
    if (rec.legendEl) rec.legendEl.style.display = 'none';
    _hdmapHideTip();
  }
}
function setHdmapPreview(id, p) {
  var map = _hdmapMap();
  if (!map || !p) { console.error('[HdmapPreview] 地圖未就緒或沒有資料，預計成果圖層沒畫'); return {ok: false}; }
  clearHdmapPreview(id);
  _hdmapEnsurePanes(map);
  function opt(o) { o.hdmapPreview = true; o.hdmapId = id; return o; }   // clear_all_display_layers_js 靠這個旗標跳過
  var b = p.bounds, counts = _hdmapCounts(p);
  var rec = { id: id, visible: true, group: L.layerGroup([], opt({})), image: null, mask: null, legend: null,
              legendEl: null, counts: counts, bounds: b, hover: [],
              layers: { extent: [], extentHoles: [], holes: [], edge: [], survey: [], points: [] } };
  rec.image = L.imageOverlay(p.image, [[b[1], b[0]], [b[3], b[2]]],
                             opt({pane: 'hdmapPreviewImg', opacity: p.opacity, interactive: false, className: 'hdmap-preview-img'}));
  rec.imageEl = function () { return rec.image.getElement ? rec.image.getElement() : null; };
  rec.group.addLayer(rec.image);
  var ext = (p.extent_rings || []).concat(p.extent_holes || []);
  if (p.mask_alpha > 0) {
    // 外框取以影像中心為準的整個世界：奇偶填色 ⇒ 範圍外框是洞、範圍內的洞又被填回（變暗）
    var cx = (b[0] + b[2]) / 2;
    var world = [[-85, cx - 179.9], [85, cx - 179.9], [85, cx + 179.9], [-85, cx + 179.9]];
    rec.mask = L.polygon([world].concat(ext), opt({renderer: _hdmapCanvasR, pane: 'hdmapPreviewVec', stroke: false,
      fill: true, fillColor: '#000000', fillOpacity: p.mask_alpha, fillRule: 'evenodd', interactive: false}));
    rec.group.addLayer(rec.mask);
  }
  (p.survey_rings || []).forEach(function (r) {
    var l = L.polyline(_hdmapClosed(r), opt({renderer: _hdmapCanvasR, pane: 'hdmapPreviewVec', color: _HDMAP_STYLE.survey,
      weight: 1.5, opacity: 0.9, dashArray: '6,4', interactive: false}));
    rec.layers.survey.push(l); rec.group.addLayer(l);
  });
  (p.extent_rings || []).forEach(function (r) {
    var l = L.polyline(_hdmapClosed(r), opt({renderer: _hdmapCanvasR, pane: 'hdmapPreviewVec', color: _HDMAP_STYLE.extent,
      weight: 2, opacity: 0.95, interactive: false}));
    rec.layers.extent.push(l); rec.group.addLayer(l);
  });
  (p.extent_holes || []).forEach(function (r) {
    var l = L.polyline(_hdmapClosed(r), opt({renderer: _hdmapCanvasR, pane: 'hdmapPreviewVec', color: _HDMAP_STYLE.extent,
      weight: 1, opacity: 0.8, interactive: false}));
    rec.layers.extentHoles.push(l); rec.group.addLayer(l);
  });
  (p.points || []).forEach(function (q) {
    var c = (p.point_colors || [])[q[2]] || '#FFFFFF';
    var m = L.circleMarker([q[0], q[1]], opt({renderer: _hdmapCanvasR, pane: 'hdmapPreviewVec', radius: 2, stroke: false,
      fill: true, fillColor: c, fillOpacity: 0.9, interactive: false}));
    rec.layers.points.push(m); rec.group.addLayer(m);
  });
  // 懸停時先比對漏洞（紅）再比對邊緣不足（橘）
  (p.holes || []).forEach(function (h) {
    _hdmapOutlined(rec, rec.layers.holes, h.ring, _HDMAP_STYLE.hole, _hdmapHoleStyle(h.kind || 'hole'), h.label, opt);
  });
  (p.edge_short || []).forEach(function (h) {
    _hdmapOutlined(rec, rec.layers.edge, h.ring, _HDMAP_STYLE.edge, [1.5, '2,4', 0.08], h.label, opt);
  });
  var ctl = L.control({position: 'bottomleft'});
  ctl.onAdd = function () { var el = _hdmapLegendEl(id, p, counts); rec.legendEl = el; return el; };
  rec.legend = ctl;
  ctl.addTo(map);
  _hdmapRecs[id] = rec;
  _hdmapApply(rec);
  return { ok: true, id: id, counts: counts };
}
function clearHdmapPreview(id) {
  var map = _hdmapMap(), ids = (id === null || id === undefined) ? Object.keys(_hdmapRecs) : [id], any = false;
  ids.forEach(function (k) {
    var rec = _hdmapRecs[k];
    if (!rec) return;
    any = true;
    if (map) {
      try { map.removeLayer(rec.group); } catch (_) {}
      try { rec.legend.remove(); } catch (_) {}
    }
    delete _hdmapRecs[k];
  });
  if (any) _hdmapHideTip();
  return any;
}
function setHdmapPreviewVisible(id, visible) {
  var rec = _hdmapRecs[id];
  if (!rec) return false;
  rec.visible = !!visible;
  _hdmapApply(rec);
  return true;
}
function hdmapPreviewInfo() {
  var map = _hdmapMap(), out = { ids: Object.keys(_hdmapRecs), layers: {} };
  out.ids.forEach(function (k) {
    var r = _hdmapRecs[k], ib = r.image.getBounds();
    var img = r.imageEl();
    function on(arr) { return arr.filter(function (l) { return map && map.hasLayer(l); }).length; }
    var onMap = !!(map && map.hasLayer(r.group));
    out.layers[k] = {
      visible: r.visible, onMap: onMap,
      image: !!(map && map.hasLayer(r.image)), imageBounds: [ib.getWest(), ib.getSouth(), ib.getEast(), ib.getNorth()],
      imageOpacity: r.image.options.opacity, imageRendering: img ? img.style.imageRendering : null,
      imagePane: r.image.options.pane, mask: !!(r.mask && map && map.hasLayer(r.mask)),
      maskRings: r.mask ? r.mask.getLatLngs().length : 0, maskAlpha: r.mask ? r.mask.options.fillOpacity : null,
      extent: on(r.layers.extent), extentHoles: on(r.layers.extentHoles), holes: on(r.layers.holes),
      edgeShort: on(r.layers.edge), survey: on(r.layers.survey), points: on(r.layers.points),
      hits: onMap ? r.hover.length : 0,
      interactive: r.group.getLayers().filter(function (l) { return l.options && l.options.interactive !== false; }).length,
      legend: !!(r.legendEl && r.legendEl.parentNode), legendShown: !!(r.legendEl && r.legendEl.style.display !== 'none'),
      notesOpen: _hdmapNotesOpen,
      counts: r.counts
    };
  });
  return out;
}
"""

_GLUE_CESIUM = r"""
var _hdmapLegendBox = null;
function _hdmapRender() { if (typeof _requestRender === 'function') _requestRender(); }
function _hdmapPanelOn() { var cb = document.getElementById('lyr_hdmap'); return !cb || cb.checked; }
function _hdmapLegendHost() {
  if (_hdmapLegendBox && _hdmapLegendBox.parentNode) return _hdmapLegendBox;
  _hdmapLegendBox = document.createElement('div');
  _hdmapLegendBox.id = 'hdmapLegendBox';
  // 左下（#sitlStatus 在 bottom:36px 之上）；只用行內樣式。高度上限＝到圖層面板下緣（不蓋住它），見 _hdmapFitLegends
  _hdmapLegendBox.style.cssText = 'position:absolute;left:8px;bottom:66px;z-index:999;display:flex;'
    + 'flex-direction:column;gap:6px;pointer-events:none;box-sizing:border-box;';
  ['click', 'dblclick', 'mousedown', 'pointerdown', 'touchstart', 'wheel', 'contextmenu'].forEach(function (e) {
    _hdmapLegendBox.addEventListener(e, _hdmapStop); });
  document.body.appendChild(_hdmapLegendBox);
  if (!window._hdmapResizeBound) { window.addEventListener('resize', _hdmapFitLegends); window._hdmapResizeBound = true; }
  return _hdmapLegendBox;
}
function _hdmapFitLegends() {
  var host = _hdmapLegendBox;
  if (!host || !host.parentNode) return;
  var top = 8, lp = document.getElementById('layerPanel');
  if (lp) { var r = lp.getBoundingClientRect(); if (r.height > 0) top = r.bottom + 8; }
  var hr = host.getBoundingClientRect();
  var bottom = hr.height > 0 ? hr.bottom : window.innerHeight - 66;
  _hdmapFitBox(host, bottom - top);
}
function _hdmapPos(ring) {
  var out = [];
  _hdmapClosed(ring).forEach(function (q) { out.push(Cesium.Cartesian3.fromDegrees(q[1], q[0])); });
  return out;
}
function _hdmapArea(ring) {
  var a = 0;
  for (var i = 0, j = ring.length - 1; i < ring.length; j = i++) a += (ring[j][1] + ring[i][1]) * (ring[j][0] - ring[i][0]);
  return Math.abs(a / 2);
}
// 範圍外框與範圍內的洞互不相交（shapely 有效多邊形）⇒ 用「最小的包含者」排出巢狀樹，交給 PolygonHierarchy（洞裡的島照樣變暗）
function _hdmapNest(rings) {
  var n = rings.length, parent = [], area = rings.map(_hdmapArea), kids = [];
  for (var i = 0; i < n; i++) {
    parent.push(-1); kids.push([]);
    var q = rings[i][0];
    for (var j = 0; j < n; j++) {
      if (j === i || area[j] <= area[i]) continue;
      if (_hdmapInside(q[0], q[1], rings[j]) && (parent[i] < 0 || area[j] < area[parent[i]])) parent[i] = j;
    }
  }
  var roots = [];
  for (var k = 0; k < n; k++) { if (parent[k] < 0) roots.push(k); else kids[parent[k]].push(k); }
  function H(idx) { return new Cesium.PolygonHierarchy(_hdmapPos(rings[idx]), kids[idx].map(H)); }
  return roots.map(H);
}
function _hdmapApply(rec) {
  var on = rec.visible && _hdmapPanelOn();
  if (rec.layer) rec.layer.show = on;
  if (rec.mask && !rec.mask.isDestroyed()) rec.mask.show = on;
  rec.entities.forEach(function (e) { e.show = on; });
  if (rec.points) rec.points.show = on;
  if (rec.legendEl) rec.legendEl.style.display = on ? 'block' : 'none';
  rec.shown = on;
}
// 圖層面板的「預計成果」勾選框（toggleLayer('hdmap') 呼叫）
function _hdmapApplyAll() { Object.keys(_hdmapRecs).forEach(function (k) { _hdmapApply(_hdmapRecs[k]); }); _hdmapFitLegends(); _hdmapRender(); }
// 非同步幾何（GroundPrimitive 在 web worker 裡建）好了才畫得出來：依需求算圖時要一直催到它 ready
function _hdmapRenderUntilReady(prim, tries) {
  if (!prim || prim.isDestroyed()) return;
  _hdmapRender();
  if (prim.ready || tries <= 0) return;
  setTimeout(function () { _hdmapRenderUntilReady(prim, tries - 1); }, 100);
}
function setHdmapPreview(id, p) {
  if (!p) { console.error('[HdmapPreview] 沒有資料，預計成果圖層沒畫'); return {ok: false}; }
  clearHdmapPreview(id);
  var b = p.bounds, counts = _hdmapCounts(p);
  var rec = { id: id, visible: true, shown: false, layer: null, state: 'loading', entities: [], mask: null, points: null,
              legendEl: null, counts: counts, bounds: b, ents: { extent: [], extentHoles: [], holes: [], edge: [], survey: [] } };
  // 品質分級點陣圖：單張影像的 ImageryLayer（最近鄰、不做 mipmap）；_hdmapPreview 旗標讓「地圖」開關跳過它
  var rect = Cesium.Rectangle.fromDegrees(b[0], b[1], b[2], b[3]);
  var layer = Cesium.ImageryLayer.fromProviderAsync(Cesium.SingleTileImageryProvider.fromUrl(p.image, {rectangle: rect}), {
    alpha: p.opacity, magnificationFilter: Cesium.TextureMagnificationFilter.NEAREST,
    minificationFilter: Cesium.TextureMinificationFilter.NEAREST });
  layer._hdmapPreview = true;
  layer.readyEvent.addEventListener(function () { rec.state = 'ready'; _hdmapRender(); });
  layer.errorEvent.addEventListener(function (e) {
    rec.state = 'error';
    console.error('[HdmapPreview] ' + id + ' 點陣圖載入失敗：' + (e && e.message ? e.message : e));
  });
  viewer.imageryLayers.add(layer);
  rec.layer = layer;
  function ent(o, bucket) { var e = viewer.entities.add(o); e._hdmapPreview = true; rec.entities.push(e); if (bucket) bucket.push(e); return e; }
  if (p.mask_alpha > 0) {
    // 範圍外變暗：外框 = 影像範圍外擴約 20 km 的框；範圍外框是洞、範圍內的洞是島（巢狀 PolygonHierarchy）。
    // ⚠ 用 allowPicking:false 的 GroundPrimitive，不用 entity：entity 會被 Viewer 預設的左鍵點選（開資訊框）與
    //   雙擊（zoomTo ⇒ 飛到約 49 km 高）拿去用 —— 加角點、畫禁航區時每點一下都會中。
    var cy = (b[1] + b[3]) / 2, dLat = 20000 / 111320, dLon = 20000 / (111320 * Math.max(0.05, Math.cos(cy * Math.PI / 180)));
    var W = b[0] - dLon, E = b[2] + dLon, S = Math.max(-89, b[1] - dLat), N = Math.min(89, b[3] + dLat);
    var outer = [Cesium.Cartesian3.fromDegrees(W, S), Cesium.Cartesian3.fromDegrees(E, S),
                 Cesium.Cartesian3.fromDegrees(E, N), Cesium.Cartesian3.fromDegrees(W, N)];
    var nested = _hdmapNest((p.extent_rings || []).concat(p.extent_holes || []));
    rec.mask = new Cesium.GroundPrimitive({
      geometryInstances: new Cesium.GeometryInstance({
        geometry: new Cesium.PolygonGeometry({ polygonHierarchy: new Cesium.PolygonHierarchy(outer, nested) }),
        attributes: { color: Cesium.ColorGeometryInstanceAttribute.fromColor(Cesium.Color.BLACK.withAlpha(p.mask_alpha)) } }),
      appearance: new Cesium.PerInstanceColorAppearance({ flat: true, translucent: true }),
      classificationType: Cesium.ClassificationType.BOTH, allowPicking: false, asynchronous: true });
    rec.mask._hdmapPreview = true;
    viewer.scene.groundPrimitives.add(rec.mask);
    rec.maskBox = [W, S, E, N];
    rec.maskTopHoles = nested.length;
  }
  function outline(ring, color, width, dash, name, bucket) {
    var mat = dash ? new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString(color), dashLength: dash })
                   : Cesium.Color.fromCssColorString(color).withAlpha(0.95);
    return ent({ name: name, polyline: { positions: _hdmapPos(ring), width: width, material: mat, clampToGround: true } }, bucket);
  }
  (p.survey_rings || []).forEach(function (r) { outline(r, _HDMAP_STYLE.survey, 1.5, 12, '規劃作業區', rec.ents.survey); });
  (p.extent_rings || []).forEach(function (r) { outline(r, _HDMAP_STYLE.extent, 2, 0, '預計成圖範圍', rec.ents.extent); });
  (p.extent_holes || []).forEach(function (r) { outline(r, _HDMAP_STYLE.extent, 1, 0, '預計成圖範圍（內緣）', rec.ents.extentHoles); });
  (p.edge_short || []).forEach(function (h) { outline(h.ring, _HDMAP_STYLE.edge, 1.5, 12, h.label || '邊緣張數不足', rec.ents.edge); });
  (p.holes || []).forEach(function (h) {
    var k = h.kind || 'hole';
    outline(h.ring, _HDMAP_STYLE.hole, k === 'preview_only' ? 1.5 : 2.5, k === 'preview_only' ? 4 : (k === 'marker' ? 0 : 12),
            h.label || '內部漏洞', rec.ents.holes);
  });
  if ((p.points || []).length) {
    var coll = new Cesium.PointPrimitiveCollection(), cache = {};
    p.points.forEach(function (q) {
      var c = (p.point_colors || [])[q[2]] || '#FFFFFF';
      var col = cache[c] || (cache[c] = Cesium.Color.fromCssColorString(c));
      coll.add({ position: Cesium.Cartesian3.fromDegrees(q[1], q[0], 0), color: col, pixelSize: 4,
                 outlineColor: Cesium.Color.BLACK, outlineWidth: 0.5, disableDepthTestDistance: Number.POSITIVE_INFINITY });
    });
    coll._hdmapPreview = true;
    viewer.scene.primitives.add(coll);
    rec.points = coll;
  }
  rec.legendEl = _hdmapLegendEl(id, p, counts);
  _hdmapLegendHost().appendChild(rec.legendEl);
  _hdmapRecs[id] = rec;
  _hdmapApply(rec);
  _hdmapFitLegends();
  _hdmapRender();
  if (rec.mask) _hdmapRenderUntilReady(rec.mask, 300);
  return { ok: true, id: id, counts: counts };
}
function clearHdmapPreview(id) {
  var ids = (id === null || id === undefined) ? Object.keys(_hdmapRecs) : [id], any = false;
  ids.forEach(function (k) {
    var rec = _hdmapRecs[k];
    if (!rec) return;
    any = true;
    try { viewer.imageryLayers.remove(rec.layer, true); } catch (_) {}
    if (rec.mask) { try { viewer.scene.groundPrimitives.remove(rec.mask); } catch (_) {} }
    rec.entities.forEach(function (e) { try { viewer.entities.remove(e); } catch (_) {} });
    if (rec.points) { try { viewer.scene.primitives.remove(rec.points); } catch (_) {} }
    if (rec.legendEl && rec.legendEl.parentNode) rec.legendEl.parentNode.removeChild(rec.legendEl);
    delete _hdmapRecs[k];
  });
  _hdmapFitLegends();
  _hdmapRender();
  return any;
}
function setHdmapPreviewVisible(id, visible) {
  var rec = _hdmapRecs[id];
  if (!rec) return false;
  rec.visible = !!visible;
  _hdmapApply(rec);
  _hdmapFitLegends();
  _hdmapRender();
  return true;
}
// 預計成果自己的拾取物（拍照點、變暗遮罩）—— 給 main.html 的 photoPoseAt 判斷要不要往下鑽
function hdmapPreviewOwnsPick(picked) {
  if (!picked) return false;
  return !!((picked.collection && picked.collection._hdmapPreview) || (picked.primitive && picked.primitive._hdmapPreview));
}
function hdmapPreviewInfo() {
  var out = { ids: Object.keys(_hdmapRecs), panel: _hdmapPanelOn(), notesOpen: _hdmapNotesOpen, layers: {} };
  out.ids.forEach(function (k) {
    var r = _hdmapRecs[k], L_ = r.layer, rc = null;
    try { var pr = L_.imageryProvider; if (pr && pr.rectangle) rc = [pr.rectangle.west, pr.rectangle.south, pr.rectangle.east, pr.rectangle.north].map(Cesium.Math.toDegrees); } catch (_) {}
    function shown(arr) { return arr.filter(function (e) { return e.show && viewer.entities.contains(e); }).length; }
    var m = (r.mask && !r.mask.isDestroyed()) ? r.mask : null;
    out.layers[k] = {
      visible: r.visible, shown: r.shown, state: r.state,
      imagery: viewer.imageryLayers.contains(L_), imageryShow: L_.show, imageryFlag: L_._hdmapPreview === true,
      alpha: L_.alpha, nearest: L_.magnificationFilter === Cesium.TextureMagnificationFilter.NEAREST,
      rect: rc, mask: !!(m && viewer.scene.groundPrimitives.contains(m)), maskShown: !!(m && m.show),
      maskReady: !!(m && m.ready), maskPickable: m ? m.allowPicking : null, maskBox: r.maskBox || null,
      maskTopHoles: r.maskTopHoles || 0,
      extent: shown(r.ents.extent), extentHoles: shown(r.ents.extentHoles), holes: shown(r.ents.holes),
      edgeShort: shown(r.ents.edge), survey: shown(r.ents.survey),
      points: r.points ? r.points.length : 0, pointsShown: !!(r.points && r.points.show),
      pointsInScene: !!(r.points && viewer.scene.primitives.contains(r.points)),
      legend: !!(r.legendEl && r.legendEl.parentNode), legendShown: !!(r.legendEl && r.legendEl.style.display !== 'none'),
      counts: r.counts
    };
  });
  return out;
}
"""


def glue_js(kind: str) -> str:
    """頁面膠水 JS。kind = 'leaflet' | 'cesium'。"""
    if kind == 'leaflet':
        adapter = _GLUE_LEAFLET
    elif kind == 'cesium':
        adapter = _GLUE_CESIUM
    else:
        raise ValueError(f"kind 只能是 leaflet/cesium：{kind!r}")
    common = (_GLUE_COMMON.replace('%STYLE%', js_json(STYLE))
              .replace('%STYLE_LABELS%', js_json(STYLE_LABELS))
              .replace('%BANNER%', js_json(PREDICTION_BANNER)))
    return common + adapter
