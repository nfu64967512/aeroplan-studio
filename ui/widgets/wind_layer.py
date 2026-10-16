"""風場圖層（2D Leaflet 與 3D Cesium 共用的 Python 端）：資料檢查、頁面膠水 JS、引擎檔位置。

畫圖引擎是 ``ui/widgets/web_shared/wind_flow.js``（純瀏覽器端：流動粒子＋箭頭＋圖例；``window.AeroWind``），
兩個地圖頁面都用 ``<script src="file://…/wind_flow.js">`` 載入，再各自用一小段膠水 JS 提供同名的頁面函式：

    setWindField(field)   clearWindField()   setWindArrows(rows)   clearWindArrows()
    setWindOptions(opts)  windInfo()

Python 端（MapWidget／CesiumMapWidget／DualMapWidget）只呼叫這六個函式；資料格式：
  * field：core.wind_field.WindField.to_js() 的 dict（u 東向、v 北向 m/s；缺值 null；單點＋bbox＝均勻場）
  * rows：每架一列 ``[sysid, lat, lon, alt_rel, dir_from_deg, speed_mps, label]``（飛控 WIND 估計）

⚠ 三種風的意義不同（規劃假設／模式預報／飛控估計），圖例文字由資料的 label 帶進來，這裡不改寫。
"""
from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Any, Iterable

WIND_FLOW_JS = Path(__file__).resolve().parent / 'web_shared' / 'wind_flow.js'
#: 首飛風速上限（使用者 2026-10-15 定調 ≤3 m/s）：圖例與箭頭上色的門檻預設值；面板可改
WIND_LIMIT_MPS_DEFAULT = 3.0
#: 一次最多幾架的箭頭（每架一列；蜂群通常 ≤ 10）
WIND_ARROWS_MAX = 64
PAGE_FUNCTIONS = ('setWindField', 'clearWindField', 'setWindArrows', 'clearWindArrows', 'setWindOptions', 'windInfo')


def wind_flow_js_uri() -> str:
    """引擎檔的 file:// URL（兩個頁面的 <script src>）。檔案不在就丟錯，不會靜默少一個圖層。"""
    if not WIND_FLOW_JS.is_file():
        raise FileNotFoundError(f'風場引擎 {WIND_FLOW_JS} 不存在：風場圖層無法顯示')
    return WIND_FLOW_JS.as_uri()


@lru_cache(maxsize=1)
def wind_flow_js_source() -> str:
    """引擎原始碼（測試與無頭驗證用；頁面本身用 URL 載入）。"""
    return WIND_FLOW_JS.read_text(encoding='utf-8')


def _num(x: Any) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        raise ValueError(f'不是數字：{x!r}')
    return float(x)


def check_wind_field(field: Any) -> dict:
    """接受 WindField 或其 to_js() dict；檢查形狀／遞增／bbox／有限值，回傳可直接 JSON 化的 dict。"""
    if hasattr(field, 'to_js'):
        field = field.to_js()
    if not isinstance(field, dict):
        raise ValueError('風場要是 WindField 或 to_js() 的 dict')
    lats = [_num(x) for x in field.get('lats') or []]
    lons = [_num(x) for x in field.get('lons') or []]
    if not lats or not lons:
        raise ValueError('風場至少要一個格點')
    if lats != sorted(lats) or lons != sorted(lons):
        raise ValueError('風場的緯度、經度要遞增')
    if any(abs(a) > 90 for a in lats) or any(abs(o) > 180 for o in lons):
        raise ValueError('風場格點經緯度超出範圍')
    u, v = field.get('u'), field.get('v')
    for name, g in (('u', u), ('v', v)):
        if not isinstance(g, (list, tuple)) or len(g) != len(lats) or any(
                not isinstance(r, (list, tuple)) or len(r) != len(lons) for r in g):
            raise ValueError(f'風場 {name} 的形狀要是 {len(lats)}×{len(lons)}')
        for r in g:
            for x in r:
                if x is not None and not math.isfinite(_num(x)):
                    raise ValueError(f'風場 {name} 有非有限值（缺值請用 None）')
    bbox = field.get('bbox')
    if bbox is not None:
        bbox = [_num(x) for x in bbox]
        if len(bbox) != 4 or not (bbox[2] > bbox[0] and bbox[3] > bbox[1]):
            raise ValueError(f'bbox 不合法：{bbox!r}（lat_min, lon_min, lat_max, lon_max）')
    if (len(lats) == 1 or len(lons) == 1) and bbox is None:
        raise ValueError('單列／單行的風場要給 bbox')
    mode = str(field.get('mode') or 'flow')
    if mode not in ('flow', 'arrows', 'both'):
        raise ValueError(f'mode 只能是 flow/arrows/both：{mode!r}')
    label = str(field.get('label') or '')
    if not label.strip():
        raise ValueError('風場要有 label（來源與限制要寫在圖例上，不能空白）')
    out = {'lats': lats, 'lons': lons,
           'u': [[None if x is None else float(x) for x in r] for r in u],
           'v': [[None if x is None else float(x) for x in r] for r in v],
           'height_m': _num(field.get('height_m', 0.0)), 'label': label, 'mode': mode}
    if bbox is not None:
        out['bbox'] = bbox
    return out


def wind_arrows_to_compact(arrows: Iterable[Any]) -> tuple[list, dict]:
    """每架的風估計 dict → 精簡列；沒位置／非有限值的**略過並計數**（不放到 (0, 0)）。

    輸入 dict 欄位：sysid, lat, lon, alt_rel, dir_from_deg, speed_mps, label（可缺；預設 ``UAV-<sysid>``）。
    """
    rows, skipped = [], 0
    for a in arrows:
        try:
            sysid = int(a['sysid'])
            lat, lon = _num(a['lat']), _num(a['lon'])
            d, s = _num(a['dir_from_deg']), _num(a['speed_mps'])
            alt = a.get('alt_rel')
            alt = 0.0 if alt is None else _num(alt)
            if not all(math.isfinite(x) for x in (lat, lon, d, s, alt)) or abs(lat) > 90 or abs(lon) > 180 or s < 0:
                raise ValueError('非有限或超出範圍')
        except (KeyError, TypeError, ValueError):
            skipped += 1
            continue
        rows.append([sysid, lat, lon, alt, d % 360.0, s, str(a.get('label') or f'UAV-{sysid}')])
    if len(rows) > WIND_ARROWS_MAX:
        raise ValueError(f'{len(rows)} 架超過風箭頭上限 {WIND_ARROWS_MAX}')
    return rows, {'rows': len(rows) + skipped, 'drawn': len(rows), 'skipped': skipped}


def js_call(fn: str, *args: Any) -> str:
    """組一句頁面函式呼叫（JSON 引數；NaN 直接丟錯而不是寫出不合法的 JSON）。"""
    if fn not in PAGE_FUNCTIONS:
        raise ValueError(f'不是風場頁面函式：{fn}')
    return f'{fn}({", ".join(json.dumps(a, allow_nan=False, ensure_ascii=False) for a in args)})'


# ─────────────────────────────────────────────────────────────────────
#  頁面膠水：兩個頁面各自提供 adapter（投影＋視角鍵），其餘交給引擎
# ─────────────────────────────────────────────────────────────────────
_GLUE_COMMON = r"""
var _windLayer = null, _windField = null, _windArrows = [], _windOpts = {limitMps: __LIMIT__};
function _windEnsure() {
  if (_windLayer) return _windLayer;
  if (!window.AeroWind || typeof window.AeroWind.create !== 'function') {
    console.error('[Wind] 引擎 wind_flow.js 沒載入，風場圖層無法顯示');
    return null;
  }
  _windLayer = window.AeroWind.create(_windHost(), _windAdapter());
  _windLayer.setOptions(_windOpts);
  return _windLayer;
}
function setWindField(field) {
  var L_ = _windEnsure();
  _windField = field || null;
  if (!L_) return false;
  if (_windField) L_.setField(_windField); else L_.clearField();
  return true;
}
function clearWindField() { _windField = null; if (_windLayer) _windLayer.clearField(); return true; }
function setWindArrows(rows) {
  var L_ = _windEnsure();
  _windArrows = Array.isArray(rows) ? rows : [];
  if (!L_) return false;
  if (_windArrows.length) L_.setArrows(_windArrows); else L_.clearArrows();
  return true;
}
function clearWindArrows() { _windArrows = []; if (_windLayer) _windLayer.clearArrows(); return true; }
function setWindOptions(opts) {
  Object.keys(opts || {}).forEach(function (k) { _windOpts[k] = opts[k]; });
  if (_windLayer) _windLayer.setOptions(_windOpts);
  return true;
}
function windInfo() {
  var st = _windLayer ? _windLayer.state() : null;
  return { engine: !!(window.AeroWind), layer: !!_windLayer, field: !!_windField, arrows: _windArrows.length,
           options: _windOpts, state: st };
}
"""

_GLUE_LEAFLET = r"""
function _windHost() { return window.currentMap._container; }
function _windAdapter() {
  var map = window.currentMap;
  return {
    project: function (lat, lon, h) {
      var p = map.latLngToContainerPoint([lat, lon]);
      return (isFinite(p.x) && isFinite(p.y)) ? { x: p.x, y: p.y } : null;
    },
    viewKey: function () {
      var c = map.getCenter(), s = map.getSize();
      return map.getZoom() + '|' + c.lat.toFixed(7) + ',' + c.lng.toFixed(7) + '|' + s.x + 'x' + s.y;
    }
  };
}
"""

_GLUE_CESIUM = r"""
function _windHost() { return viewer.container; }
function _windAdapter() {
  var scratch = new Cesium.Cartesian2();
  return {
    project: function (lat, lon, h) {
      var c = Cesium.Cartesian3.fromDegrees(lon, lat, h || 0);
      var p = Cesium.SceneTransforms.wgs84ToWindowCoordinates(viewer.scene, c, scratch);
      if (!p || !isFinite(p.x) || !isFinite(p.y)) return null;
      // 被地球擋住（背面）的點不畫：相機到點的向量與地表法線同向 ⇒ 在背面
      var cam = viewer.camera.positionWC;
      var n = Cesium.Cartesian3.normalize(c, new Cesium.Cartesian3());
      var toCam = Cesium.Cartesian3.subtract(cam, c, new Cesium.Cartesian3());
      if (Cesium.Cartesian3.dot(n, toCam) < 0) return null;
      return { x: p.x, y: p.y };
    },
    viewKey: function () {
      var cam = viewer.camera, p = cam.positionWC, cv = viewer.canvas;
      return p.x.toFixed(1) + ',' + p.y.toFixed(1) + ',' + p.z.toFixed(1) + '|' + cam.heading.toFixed(4) + ','
           + cam.pitch.toFixed(4) + ',' + cam.roll.toFixed(4) + '|' + cv.clientWidth + 'x' + cv.clientHeight;
    }
  };
}
"""


def glue_js(kind: str, *, limit_mps: float = WIND_LIMIT_MPS_DEFAULT) -> str:
    """頁面膠水 JS（不含引擎）。kind = 'leaflet' | 'cesium'。"""
    if kind == 'leaflet':
        adapter = _GLUE_LEAFLET
    elif kind == 'cesium':
        adapter = _GLUE_CESIUM
    else:
        raise ValueError(f"kind 只能是 leaflet/cesium：{kind!r}")
    lim = float(limit_mps)
    if not (math.isfinite(lim) and lim > 0):
        raise ValueError(f'風速上限不合法：{limit_mps!r}')
    return adapter + _GLUE_COMMON.replace('__LIMIT__', repr(lim))


def restore_js(field: dict | None, rows: list | None, options: dict | None) -> str:
    """頁面重建後（2D 每次 folium 重繪、3D 重新載入）把 Python 端記住的狀態套回去的 JS。"""
    parts = []
    if options:
        parts.append(js_call('setWindOptions', options) + ';')
    if field:
        parts.append(js_call('setWindField', field) + ';')
    if rows:
        parts.append(js_call('setWindArrows', rows) + ';')
    return '\n'.join(parts)
