"""風場圖層（2D Leaflet 與 3D Cesium 共用的 Python 端）：資料檢查、頁面膠水 JS、引擎檔位置。

畫圖引擎是 ``ui/widgets/web_shared/wind_flow.js``（純瀏覽器端：流動粒子＋箭頭＋圖例；``window.AeroWind``），
兩個地圖頁面都用 ``<script src="file://…/wind_flow.js">`` 載入，再各自用一小段膠水 JS 提供同名的頁面函式：

    setWindField(field)   clearWindField()   setWindArrows(rows)   clearWindArrows()
    setWindOptions(opts)  windInfo()

Python 端（MapWidget／CesiumMapWidget／DualMapWidget）只呼叫這六個函式；資料格式：
  * field：core.wind_field.WindField.to_js() 的 dict（u 東向、v 北向 m/s；缺值 null；單點＋bbox＝均勻場）
  * rows：每架一列 ``[sysid, lat, lon, alt_rel, dir_from_deg, speed_mps, label]``（飛控 WIND 估計）

分層風場（2026-10-19；使用者定調只做 10／80／120 m 三層）另有三個頁面函式：

    setWindProfile(p)   clearWindProfile()   setWindProfileView(view)      （狀態在 windInfo().profile）

  * p：``check_wind_profile(payload)`` 的結果（各層＝檢查過的 field、剖面柱、圖例字、2D 高度、3D 子集、粒子預算）
  * 設著分層風場時它就是那張圖的**流動層**：setWindField 的單一風場先記著不畫，clearWindProfile 後畫回來；
    飛控即時風箭頭（setWindArrows）不受影響。
  * 3D：每層一個引擎風場（id ``prof:<h>``，畫在自己的高度）。粒子數：各層平分「單一風場依可見面積的自動粒子數」
    （引擎 particleShare＝1／顯示層數，每層至少 80），``max(80, 總數 // 顯示層數)`` 是**上限**——總工作量約等於
    原本的單一風場，拉遠時也守引擎的面積密度上限。height_2d_m 那一層畫實線、其餘層淡化（俯視疊在一起才分得出），
    換這一層只換樣式、粒子不重生、剖面柱不重畫。圖例合成一個區塊；剖面點畫一根垂直柱（Cesium entity）＋柱頂標題
    （模式／預報非實測／時間）＋每層一支順風箭頭＋文字「<h> m  <s> m/s  來自 <dir>°」。
  * 2D（俯視各層重疊）：只畫 height_2d_m 那一層（圖例兩行：層＋模式時間）＋左上角一張剖面表（行內樣式，Leaflet
    控制項）；地圖窄到風場圖例會壓到剖面表時，圖例往下移到表格下方。
  * 各層之間**不內插、不外推**：只畫拿到的那幾層。

⚠ 三種風的意義不同（規劃假設／模式預報／飛控估計），圖例文字由資料的 label 帶進來，這裡不改寫
（分層風場的區塊標題由各層 label 裡的模式名與時間組成，見 profile_legend）。
"""
from __future__ import annotations

import json
import math
import re
from functools import lru_cache
from pathlib import Path
from typing import Any, Iterable, Optional

WIND_FLOW_JS = Path(__file__).resolve().parent / 'web_shared' / 'wind_flow.js'
#: 首飛風速上限（使用者 2026-10-15 定調 ≤3 m/s）：圖例與箭頭上色的門檻預設值；面板可改
WIND_LIMIT_MPS_DEFAULT = 3.0
#: 一次最多幾架的箭頭（每架一列；蜂群通常 ≤ 10）
WIND_ARROWS_MAX = 64
PAGE_FUNCTIONS = ('setWindField', 'clearWindField', 'setWindArrows', 'clearWindArrows', 'setWindOptions', 'windInfo',
                  'setWindProfile', 'clearWindProfile', 'setWindProfileView')

#: 分層風場：3D 全部層加起來的粒子預算（每層 max(80, 總數 // 顯示層數)）；2D 只畫一層、沿用單一風場的自動粒子數
PROFILE_PARTICLES_DEFAULT = 2400
PROFILE_PARTICLES_MIN_PER_LAYER = 80
#: 引擎每個風場的粒子上限（wind_flow.js 的 MAX_PARTICLES）
PROFILE_PARTICLES_MAX = 20000
#: 一份分層風場最多幾層（10／80／120 m 是 3 層；上限只為了把工作量框住）
PROFILE_LAYERS_MAX = 8
#: 引擎裡各層的風場 id 前綴（``prof:10``、``prof:80``、``prof:120``）與圖例分組名
PROFILE_FIELD_PREFIX = 'prof:'
PROFILE_GROUP = 'profile'
#: 3D 剖面柱的樣式：箭頭長度 = m_per_sqrt_mps × √風速（公尺，夾在 min_m–max_m；25 m/s 才到上限——舊的線性
#:   15 m/(m/s) 夾 120 m 在 8 m/s 就飽和，強風的幾層長度一樣）；風速 < calm_mps 只畫點不畫箭頭。
#:   文字一律淺色（深底上對比 ≥ 4.5:1）、風速顏色只放在點與箭頭上（紅字在深底上只有 ~3:1 讀不清楚）；
#:   文字放在箭頭的**另一側**（每次算圖前依螢幕上箭頭往哪邊決定），文字底下挖空粒子（引擎 setMask）。
PROFILE_COLUMN_STYLE = {'m_per_sqrt_mps': 30.0, 'min_m': 10.0, 'max_m': 150.0, 'calm_mps': 0.05,
                        'arrow_width_px': 12, 'line_width_px': 2, 'label_step_px': 20, 'label_gap_px': 14,
                        'label_font_px': 13, 'label_pad_px': [5, 2], 'point_px': 9,
                        'text_color': '#e8eef5', 'label_bg': [0.06, 0.08, 0.11, 0.85]}
#: 剖面表／剖面柱文字顏色（淺色；風速顏色另外用色塊）
PROFILE_TEXT_COLOR = '#e8eef5'


def profile_arrow_length_m(speed_mps: float) -> float:
    """3D 剖面柱上一層的順風箭頭長度（公尺）：m_per_sqrt_mps × √風速，夾在 min_m–max_m；靜風（< calm_mps）＝ 0。"""
    s = float(speed_mps)
    S = PROFILE_COLUMN_STYLE
    if not math.isfinite(s) or s < S['calm_mps']:
        return 0.0
    return min(S['max_m'], max(S['min_m'], S['m_per_sqrt_mps'] * math.sqrt(s)))


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


# ─────────────────────────────────────────────────────────────────────
#  分層風場（10／80／120 m；同一模式、同一小時）
# ─────────────────────────────────────────────────────────────────────
#: core.wind_field 各層 label 的固定格式：'Open-Meteo 預報（<model>，非實測）<h> m 風，<time> UTC；<n> 個模式格點'
_LAYER_MODEL_RE = re.compile(r'預報（([^，（）]+)，非實測）')
_LAYER_TIME_RE = re.compile(r'風，([^；，]+?) UTC')


def fmt_height(h: float) -> str:
    """高度字串（與頁面 JS 的 String(h) 一致：10.0 → '10'、12.5 → '12.5'）。"""
    return f'{float(h):g}'


def profile_field_id(h: float) -> str:
    """某一層在引擎裡的風場 id（``prof:80``）。"""
    return PROFILE_FIELD_PREFIX + fmt_height(h)


def profile_particles_per_layer(total: int, n_layers: int) -> int:
    """3D 每層粒子數：總預算平均分給顯示中的層，每層至少 PROFILE_PARTICLES_MIN_PER_LAYER。"""
    return max(PROFILE_PARTICLES_MIN_PER_LAYER, int(total) // max(1, int(n_layers)))


def nearest_height(heights: Iterable[float], h: float) -> float:
    """最接近 h 的那一層（**不內插**）；一樣近取低的那層。"""
    hs = [float(x) for x in heights]
    if not hs:
        raise ValueError('沒有任何一層')
    return min(hs, key=lambda x: (abs(x - float(h)), x))


def _match_height(heights: list, h: float) -> Optional[float]:
    for x in heights:
        if abs(x - h) < 1e-6:
            return x
    return None


def profile_legend(heights: list, label: str, layer_labels: Iterable[str], *, model: Optional[str] = None,
                   time_utc: Optional[str] = None) -> tuple[str, str]:
    """分層風場的圖例區塊（兩行）：('風場分層 10／80／120 m', '（icon_global 預報，非實測，2026-10-19T08:00 UTC）')。

    模式名與時間：payload 有給 model／time_utc 就用；否則從各層 label（core.wind_field 的固定格式）取，
    **全部層一致**才用；都不行就整段用 payload 的 label（沒寫「非實測」就補上——這一定是模式預報）。
    """
    head = f'風場分層 {"／".join(fmt_height(h) for h in heights)} m'
    if not (model and time_utc):
        labels = list(layer_labels)
        ms = [m.group(1) for m in (_LAYER_MODEL_RE.search(s) for s in labels) if m]
        ts = [m.group(1) for m in (_LAYER_TIME_RE.search(s) for s in labels) if m]
        if labels and len(ms) == len(labels) == len(ts) and len(set(ms)) == 1 and len(set(ts)) == 1:
            model, time_utc = ms[0], ts[0]
    if model and time_utc:
        t = str(time_utc).strip()
        desc = f'{str(model).strip()} 預報，非實測，{t if t.upper().endswith("UTC") else t + " UTC"}'
    else:
        desc = label if '非實測' in label else f'{label}；預報，非實測'
    return head, f'（{desc}）'


def _check_column(col: Any, heights: list) -> Optional[dict]:
    if col is None:
        return None
    if not isinstance(col, dict):
        raise ValueError('剖面柱 column 要是 {lat, lon, levels}')
    lat, lon = _num(col.get('lat')), _num(col.get('lon'))
    if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90 and abs(lon) <= 180):
        raise ValueError(f'剖面柱位置不合法：{lat!r}, {lon!r}')
    levels = col.get('levels') or []
    if not isinstance(levels, (list, tuple)):
        raise ValueError('剖面柱 levels 要是串列')
    out, seen = [], set()
    for i, lv in enumerate(levels):
        if not isinstance(lv, dict):
            raise ValueError(f'剖面柱第 {i + 1} 層要是 {{h_m, speed_mps, dir_from_deg}}')
        try:
            h, s, d = _num(lv.get('h_m')), _num(lv.get('speed_mps')), _num(lv.get('dir_from_deg'))
        except ValueError as e:
            raise ValueError(f'剖面柱第 {i + 1} 層：{e}（缺值請整層省略）') from e
        if not all(math.isfinite(x) for x in (h, s, d)) or s < 0:
            raise ValueError(f'剖面柱第 {i + 1} 層有非有限或負值（缺值請整層省略）')
        hm = _match_height(heights, h)
        if hm is None:
            raise ValueError(f'剖面柱的 {fmt_height(h)} m 不是分層裡的任何一層（{"／".join(map(fmt_height, heights))} m）：'
                             '各層之間不內插')
        if hm in seen:
            raise ValueError(f'剖面柱的 {fmt_height(h)} m 重複')
        seen.add(hm)
        out.append({'h_m': hm, 'speed_mps': s, 'dir_from_deg': d % 360.0})
    out.sort(key=lambda r: r['h_m'])
    return {'lat': lat, 'lon': lon, 'levels': out}


def resolve_profile_view(profile: dict, *, height_2d_m: Any = None, layers_3d: Any = None) -> dict:
    """把「2D 顯示高度／3D 顯示哪幾層」對到真的有的層（**不內插**）。

    height_2d_m：None ⇒ 沿用 profile 目前的；否則取最接近的一層（一樣近取低的），要求值另記在 height_2d_requested_m。
    layers_3d：None ⇒ 全部；否則只留真的有的層（其餘列在 layers_3d_ignored）；空串列 ⇒ 3D 只剩剖面柱。
    """
    heights = list(profile['heights'])
    if height_2d_m is None:
        h2, req = profile['height_2d_m'], profile.get('height_2d_requested_m')
    else:
        req = _num(height_2d_m)
        if not math.isfinite(req):
            raise ValueError(f'2D 顯示高度不合法：{height_2d_m!r}')
        h2 = nearest_height(heights, req)
    if layers_3d is None:
        l3, ignored = None, []
    else:
        if isinstance(layers_3d, (str, bytes, dict)) or not isinstance(layers_3d, Iterable):
            raise ValueError(f'layers_3d 要是高度串列或 None：{layers_3d!r}')
        l3, ignored = [], []
        for x in layers_3d:
            v = _num(x)
            m = _match_height(heights, v) if math.isfinite(v) else None
            if m is None:
                ignored.append(v if math.isfinite(v) else None)
            elif m not in l3:
                l3.append(m)
        l3.sort()
    return {'height_2d_m': h2, 'height_2d_requested_m': req, 'layers_3d': l3, 'layers_3d_ignored': ignored}


def check_wind_profile(payload: Any) -> dict:
    """分層風場 payload → 頁面用的 dict（可直接 JSON 化；不合法 ⇒ ValueError，訊息是中文）。

    payload = {'layers': [field（含 height_m）, …], 'column': {'lat', 'lon', 'levels': [{'h_m', 'speed_mps',
    'dir_from_deg'}]}, 'label': 來源／模式／時間, 'height_2d_m': 2D 顯示高度, 'particles_total': 3D 粒子總數（預設
    2400）, 'limit_mps': 上色門檻（可省）, 'layers_3d': 3D 顯示哪幾層（可省＝全部）, 'model'／'time_utc'（可省）}
    """
    if not isinstance(payload, dict):
        raise ValueError('分層風場要是 dict（layers／column／label／height_2d_m…）')
    raw = payload.get('layers')
    if not isinstance(raw, (list, tuple)) or not raw:
        raise ValueError('分層風場至少要一層（layers）')
    if len(raw) > PROFILE_LAYERS_MAX:
        raise ValueError(f'分層風場最多 {PROFILE_LAYERS_MAX} 層：{len(raw)}')
    layers = []
    for i, lay in enumerate(raw):
        has_h = (lay.get('height_m') if isinstance(lay, dict) else getattr(lay, 'height_m', None)) is not None
        if not has_h:
            raise ValueError(f'分層風場第 {i + 1} 層沒有 height_m')
        try:
            f = check_wind_field(lay)
        except ValueError as e:
            raise ValueError(f'分層風場第 {i + 1} 層：{e}') from e
        if not (math.isfinite(f['height_m']) and 0.0 <= f['height_m'] <= 10000.0):
            raise ValueError(f'分層風場第 {i + 1} 層的高度不合法：{f["height_m"]!r}')
        if f['mode'] != 'flow':
            f['mode'] = 'flow'                      # 分層風場一律是流動層（箭頭在剖面柱上）
        layers.append(f)
    layers.sort(key=lambda f: f['height_m'])
    heights = [f['height_m'] for f in layers]
    if any(b - a < 1e-6 for a, b in zip(heights, heights[1:])):
        raise ValueError(f'分層風場的高度重複：{"／".join(map(fmt_height, heights))} m')
    label = str(payload.get('label') or '').strip()
    if not label:
        raise ValueError('分層風場要有 label（來源、模式、時間要寫在圖例上，不能空白）')
    column = _check_column(payload.get('column'), heights)
    total = payload.get('particles_total', PROFILE_PARTICLES_DEFAULT)
    if total is None:
        total = PROFILE_PARTICLES_DEFAULT
    if isinstance(total, bool) or not isinstance(total, (int, float)) or not math.isfinite(total) \
            or total != int(total) or not (1 <= int(total) <= PROFILE_PARTICLES_MAX):
        raise ValueError(f'particles_total 要是 1–{PROFILE_PARTICLES_MAX} 的整數：{total!r}')
    lim = payload.get('limit_mps')
    if lim is not None:
        lim = _num(lim)
        if not (math.isfinite(lim) and lim > 0):
            raise ValueError(f'limit_mps 要是正數：{payload.get("limit_mps")!r}')
    model, time_utc = payload.get('model'), payload.get('time_utc')
    head, desc = profile_legend(heights, label, [f['label'] for f in layers],
                                model=str(model) if model else None, time_utc=str(time_utc) if time_utc else None)
    p = {'layers': layers, 'heights': heights, 'column': column, 'label': label,
         'legend_head': head, 'legend_desc': desc, 'legend': head + desc,
         'height_2d_m': heights[0], 'height_2d_requested_m': None, 'layers_3d': None, 'layers_3d_ignored': [],
         'particles_total': int(total), 'particles_min_per_layer': PROFILE_PARTICLES_MIN_PER_LAYER,
         'limit_mps': lim}
    h2 = payload.get('height_2d_m')
    p.update(resolve_profile_view(p, height_2d_m=h2, layers_3d=payload.get('layers_3d')))
    return p


def profile_summary(p: dict, kind: str) -> dict:
    """set_wind_profile 的回傳值（實際生效的值，不是要求值）。kind = '2d' | '3d'。"""
    if kind == '2d':
        drawn, per = [p['height_2d_m']], None          # 2D 只畫一層、粒子數用引擎的自動值
    elif kind == '3d':
        drawn = list(p['heights']) if p['layers_3d'] is None else list(p['layers_3d'])
        per = profile_particles_per_layer(p['particles_total'], len(drawn)) if drawn else 0
    else:
        raise ValueError(f"kind 只能是 2d/3d：{kind!r}")
    col = p.get('column')
    emph = profile_emphasis_3d(p) if kind == '3d' else None
    return {'heights': list(p['heights']), 'drawn': drawn, 'field_ids': [profile_field_id(h) for h in drawn],
            'height_2d_m': p['height_2d_m'], 'height_2d_requested_m': p['height_2d_requested_m'],
            'layers_3d': None if p['layers_3d'] is None else list(p['layers_3d']),
            'layers_3d_ignored': list(p['layers_3d_ignored']),
            # 3D：每層的粒子數**上限**（實際數由引擎依可見面積算：自動值 ÷ 顯示層數，至少 80）；2D：None（自動值）
            'particles_per_layer': per, 'particles_cap_per_layer': per,
            'particles_rule': (None if per is None else
                               f'每層＝依可見面積的自動粒子數 ÷ {len(drawn)}（至少 {PROFILE_PARTICLES_MIN_PER_LAYER}），'
                               f'上限 {per}'),
            'emphasis_3d_m': emph,
            'particles_total': p['particles_total'], 'column_levels': len(col['levels']) if col else 0,
            'legend': p['legend'], 'label': p['label']}


def profile_emphasis_3d(p: dict) -> Optional[float]:
    """3D 畫實線（其餘層淡化）的那一層 ＝ height_2d_m（面板「2D 顯示高度」，預設最接近航高）；只有一層要畫、
    或那一層不在 3D 顯示的子集裡 ⇒ None（全部照常畫）。"""
    drawn = list(p['heights']) if p.get('layers_3d') is None else list(p['layers_3d'])
    h = p.get('height_2d_m')
    if len(drawn) < 2 or h is None or _match_height(drawn, float(h)) is None:
        return None
    return _match_height(drawn, float(h))


def profile_resync_js(profile: Optional[dict]) -> str:
    """頁面載入期間分層風場有變動時，載入完成後補送的 JS（有 ⇒ setWindProfile；沒有 ⇒ clearWindProfile）。"""
    return js_call('setWindProfile', profile) if profile else js_call('clearWindProfile')


def _page_json(a: Any) -> str:
    # 也會被內嵌進 <script>（2D 整頁重繪、3D 範本）：'</' 會提早結束 script ⇒ 寫成 '<\/'；
    # 3D 範本是逐一 .replace 佔位字（__XXX__）⇒ 字串裡的 '__' 寫成 '__'。兩者都是合法的 JSON 跳脫，值不變。
    return json.dumps(a, allow_nan=False, ensure_ascii=False).replace('</', '<\\/').replace('__', '_\\u005f')


def js_call(fn: str, *args: Any) -> str:
    """組一句頁面函式呼叫（JSON 引數；NaN 直接丟錯而不是寫出不合法的 JSON）。"""
    if fn not in PAGE_FUNCTIONS:
        raise ValueError(f'不是風場頁面函式：{fn}')
    return f'{fn}({", ".join(_page_json(a) for a in args)})'


# ─────────────────────────────────────────────────────────────────────
#  頁面膠水：兩個頁面各自提供 adapter（投影＋視角鍵），其餘交給引擎
# ─────────────────────────────────────────────────────────────────────
_GLUE_COMMON = r"""
var _windLayer = null, _windField = null, _windArrows = [], _windOpts = {limitMps: __LIMIT__, legendMargin: __LEGEND__,
                                                                        trailsFollowView: __TRAILS_FOLLOW__};
function _windEnsure() {
  if (_windLayer) return _windLayer;
  if (!window.AeroWind || typeof window.AeroWind.create !== 'function') {
    console.error('[Wind] 引擎 wind_flow.js 沒載入，風場圖層無法顯示');
    return null;
  }
  _windLayer = window.AeroWind.create(_windHost(), _windAdapter());
  _windLayer.setOptions(_windOpts);
  if (typeof _windAfterCreate === 'function') { try { _windAfterCreate(_windLayer); } catch (e) { console.error('[Wind] 頁面掛勾失敗', e); } }
  return _windLayer;
}
function setWindField(field) {
  var L_ = _windEnsure();
  _windField = field || null;
  if (!L_) return false;
  if (_windProfile) return true;          // 分層風場是這張圖的流動層：單一風場先記著，clearWindProfile 後畫回來
  if (_windField) L_.setField(_windField); else L_.clearField('default');
  _windAfterLegendChange();
  return true;
}
function clearWindField() { _windField = null; if (_windLayer) { _windLayer.clearField('default'); _windAfterLegendChange(); } return true; }
function setWindArrows(rows) {
  var L_ = _windEnsure();
  _windArrows = Array.isArray(rows) ? rows : [];
  if (!L_) return false;
  if (_windArrows.length) L_.setArrows(_windArrows); else L_.clearArrows();
  _windAfterLegendChange();
  return true;
}
function clearWindArrows() { _windArrows = []; if (_windLayer) { _windLayer.clearArrows(); _windAfterLegendChange(); } return true; }
function setWindOptions(opts) {
  Object.keys(opts || {}).forEach(function (k) { _windOpts[k] = opts[k]; });
  if (_windLayer) _windLayer.setOptions(_windOpts);
  if (_windProfile && _windLayer) _windProfileDecorSafe(_windProfile, _windProfileDrawn(_windProfile));   // 門檻變 ⇒ 重新上色
  _windAfterLegendChange();
  return true;
}
function windInfo() {
  var st = _windLayer ? _windLayer.state() : null;
  return { engine: !!(window.AeroWind), layer: !!_windLayer, field: !!_windField, arrows: _windArrows.length,
           options: _windOpts, state: st, profile: _windProfileInfo(st) };
}

// ── 分層風場（Python：wind_layer.check_wind_profile 的結果）──
//   各層＝一個引擎風場（id 'prof:<h>'）；顯示哪幾層由 _windProfileDrawn 決定（3D：layers_3d 子集或全部；2D：height_2d_m 那一層）。
//   各層之間不內插：只畫拿到的層。剖面柱（3D）／剖面表（2D）由各頁的 _windProfileDecor 畫。
//   同 id 再 setField 時引擎沿用粒子陣列 ⇒ 換顯示高度／實線那層時粒子不重生、軌跡不清。
var _windProfile = null, _windProfileIds = [], _windProfileErrLogged = false;
var _WIND_PROF = __PROFILE_CONST__;
function _windFmtH(h) { return String(Math.round(Number(h) * 1000) / 1000); }
function _windSameList(a, b) {
  if (a.length !== b.length) return false;
  for (var i = 0; i < a.length; i++) { if (a[i] !== b[i]) return false; }
  return true;
}
function _windProfileDrawn(p) {
  if (!p) return [];
  if (_WIND_PROFILE_MULTI) {
    if (p.layers_3d === null || p.layers_3d === undefined) return p.heights.slice();
    return p.heights.filter(function (h) { return p.layers_3d.indexOf(h) >= 0; });
  }
  return p.heights.indexOf(p.height_2d_m) >= 0 ? [p.height_2d_m] : [];
}
function _windProfileEmph(p) {          // 3D 畫實線的那一層（＝2D 顯示高度）；只畫一層或那層不在子集裡 ⇒ null（全部照常）
  if (!p || !_WIND_PROFILE_MULTI) return null;
  var d = _windProfileDrawn(p);
  return (d.length > 1 && d.indexOf(p.height_2d_m) >= 0) ? p.height_2d_m : null;
}
function _windProfilePerLayer(p, drawn) {   // 3D 每層粒子數的上限（實際數由引擎依可見面積平分）
  return Math.max(_WIND_PROF.minPerLayer, Math.floor(p.particles_total / Math.max(1, drawn.length)));
}
function _windProfileGroupLabel(p, drawn) {      // 圖例區塊：每行都要放得下（引擎超寬會截斷）⇒ 說明各自另起一行
  if (!_WIND_PROFILE_MULTI) {                    // 2D 只畫一層：寫那一層（短，後面放得下最大風速）＋模式／非實測／時間
    return '風場分層 ' + (drawn.length ? _windFmtH(drawn[0]) + ' m' : '—') + '（2D 只畫這層）\n' + p.legend_desc;
  }
  var s = p.legend_head + '\n' + p.legend_desc;
  if (drawn.length !== p.heights.length) {
    s += '\n3D 只顯示 ' + (drawn.length ? drawn.map(_windFmtH).join('／') + ' m' : '剖面柱') + '（其餘層隱藏）';
  }
  var e = _windProfileEmph(p);
  if (e !== null) s += '\n實線＝' + _windFmtH(e) + ' m 層，其餘層淡化（各層數值見剖面柱）';
  return s;
}
function _windProfileDecorSafe(p, drawn) {
  try { _windProfileDecor(p, drawn); }
  catch (e) { if (!_windProfileErrLogged) { _windProfileErrLogged = true; console.error('[Wind] 分層風場剖面畫不出來', e); } }
}
function _windProfileField(p, lay, drawn) {
  var f = {};
  Object.keys(lay).forEach(function (k) { f[k] = lay[k]; });
  f.group = _WIND_PROF.group;
  f.groupLabel = _windProfileGroupLabel(p, drawn);
  if (_WIND_PROFILE_MULTI) {
    var e = _windProfileEmph(p);
    f.particles = _windProfilePerLayer(p, drawn);          // 上限
    f.particleShare = 1 / Math.max(1, drawn.length);      // 各層平分單一風場依可見面積的自動預算
    f.particlesMin = _WIND_PROF.minPerLayer;
    f.faint = e !== null && lay.height_m !== e;
  }
  return f;
}
function _windProfileSetFields(L_, p, drawn) {
  var ids = [];
  p.layers.forEach(function (lay) {
    if (drawn.indexOf(lay.height_m) < 0) return;
    var id = _WIND_PROF.prefix + _windFmtH(lay.height_m);
    if (L_.setField(id, _windProfileField(p, lay, drawn))) ids.push(id);
  });
  return ids;
}
function _windProfileApply() {
  var L_ = _windEnsure();
  if (!L_) return false;
  var p = _windProfile, drawn = p ? _windProfileDrawn(p) : [];
  var want = drawn.map(function (h) { return _WIND_PROF.prefix + _windFmtH(h); });
  for (var i = 0; i < _windProfileIds.length; i++) { if (want.indexOf(_windProfileIds[i]) < 0) L_.clearField(_windProfileIds[i]); }
  _windProfileIds = [];
  if (!p) {
    if (_windField) L_.setField(_windField);                  // 單一風場畫回來
    _windProfileDecorSafe(null, []);
    _windAfterLegendChange();
    return true;
  }
  L_.clearField('default');
  _windProfileIds = _windProfileSetFields(L_, p, drawn);
  _windProfileDecorSafe(p, drawn);
  _windAfterLegendChange();
  return true;
}
function setWindProfile(p) { _windProfile = p || null; return _windProfileApply(); }
function clearWindProfile() {
  var had = !!_windProfile;
  _windProfile = null;
  if (_windLayer) _windProfileApply(); else _windProfileDecorSafe(null, []);
  return had;
}
function setWindProfileView(v) {
  if (!_windProfile) return false;
  var p = _windProfile, before = _windProfileDrawn(p), emph0 = _windProfileEmph(p);
  v = v || {};
  ['height_2d_m', 'height_2d_requested_m', 'layers_3d', 'layers_3d_ignored'].forEach(function (k) {
    if (Object.prototype.hasOwnProperty.call(v, k)) p[k] = v[k];
  });
  var after = _windProfileDrawn(p);
  if (!_windSameList(before, after)) return _windProfileApply();
  var L_ = _windEnsure();
  if (!L_) return false;
  if (_WIND_PROFILE_MULTI) {
    // 3D：顯示的層沒變 ⇒ 粒子不重生、剖面柱不重畫；實線那層換了才重設樣式（同 id，粒子陣列沿用）
    if (_windProfileEmph(p) !== emph0) _windProfileIds = _windProfileSetFields(L_, p, after);
    return true;
  }
  _windProfileDecorSafe(p, after);              // 2D：還是同一層（要求值可能不同）⇒ 只更新表格
  return true;
}
function _windProfileInfo(st) {
  var p = _windProfile;
  if (!p) return null;
  var drawn = _windProfileDrawn(p), d = null, actual = 0;
  try { d = _windProfileDecorInfo(); } catch (e) { d = { error: String(e) }; }
  if (st && st.fields) _windProfileIds.forEach(function (id) { if (st.fields[id]) actual += st.fields[id].n_particles; });
  var cap = _WIND_PROFILE_MULTI ? _windProfilePerLayer(p, drawn) : null;
  return { heights: p.heights.slice(), drawn: drawn, fieldIds: _windProfileIds.slice(), height_2d_m: p.height_2d_m,
           height_2d_requested_m: p.height_2d_requested_m, layers_3d: p.layers_3d, particles_total: p.particles_total,
           particles_per_layer: cap, particles_cap_per_layer: cap, particles_actual: actual,
           emphasis_3d_m: _windProfileEmph(p), legend: p.legend, label: p.label, decor: d };
}
"""

_GLUE_LEAFLET = r"""
function _windHost() { return window.currentMap._container; }
function _windAdapter() {
  var map = window.currentMap;
  return {
    project: function (lat, lon, h) {
      var p = map.latLngToContainerPoint([lat, lon]);          // 畫面外照樣回座標（引擎要靠它找可見區）
      return (isFinite(p.x) && isFinite(p.y)) ? { x: p.x, y: p.y } : null;
    },
    viewKey: function () {
      var c = map.getCenter(), s = map.getSize();
      return map.getZoom() + '|' + c.lat.toFixed(7) + ',' + c.lng.toFixed(7) + '|' + s.x + 'x' + s.y;
    }
  };
}
function _windAfterCreate(L_) {
  // Leaflet 縮放動畫（約 250 ms）用 CSS 縮放圖磚窗格、引擎的 canvas 不會跟著縮 ⇒ 粒子錯位；動畫期間先藏起來
  var map = window.currentMap;
  map.on('zoomstart', function () { L_.setOptions({visible: false}); });
  map.on('zoomend', function () { L_.setOptions({visible: true}); });
  map.on('resize', function () { _windAfterLegendChange(); });
}
// 地圖窄的時候，右上的風場圖例（引擎畫在 canvas 上）會伸到左上的剖面表 ⇒ 圖例往下移到表格下方。只動引擎的
//   legendMargin；_windOpts 裡的基準不改（setWindOptions 會先套回基準、再重量一次）。圖例寬要等引擎畫過一幀才量得到
//   ⇒ 兩個 requestAnimationFrame 之後再量。
var _windFitPending = false, _windFitTries = 0, _windLegendShift = null;
function _windAfterLegendChange() {
  if (_windFitPending) return;
  _windFitPending = true;
  var raf = window.requestAnimationFrame ? function (f) { window.requestAnimationFrame(f); } : function (f) { setTimeout(f, 16); };
  raf(function () { raf(function () { _windFitPending = false; try { _windFitLegend(); } catch (e) {} }); });
}
function _windFitLegend() {
  var L_ = _windLayer, map = window.currentMap;
  if (!L_ || !map) return null;
  var base = _windOpts.legendMargin || [10, 60], m = [base[0], base[1]], again = false;
  var el = _windProfCtl ? _windProfCtl.getContainer() : null;
  if (el && el.parentNode) {
    var mc = map._container.getBoundingClientRect(), tr = el.getBoundingClientRect(), lb = L_.state().legendBox;
    if (!lb && _windFitTries < 20) { _windFitTries++; again = true; }
    var legW = lb ? lb[2] : 380;                               // 還沒畫過：用引擎的最寬值估
    if (mc.width - base[0] - legW < tr.right - mc.left + 8) {
      m = [base[0], Math.max(base[1], Math.ceil(tr.bottom - mc.top + 8))];
    }
  }
  var cur = L_.options().legendMargin;
  if (cur[0] !== m[0] || cur[1] !== m[1]) L_.setOptions({legendMargin: m});
  _windLegendShift = (m[1] !== base[1]) ? m : null;
  if (again) _windAfterLegendChange(); else _windFitTries = 0;
  return m;
}
// 分層風場（2D 俯視：各層重疊 ⇒ 只畫 height_2d_m 那一層）＋左上角剖面表（Leaflet 控制項，疊在縮放／全螢幕鈕下面）。
//   只用行內樣式（不加全域 CSS）；左下的角點計數框、預計成果圖例，右上的風場圖例、右下的群飛圖例都不在這一角。
//   預計成果圖例的高度上限量的是左上控制項的下緣 ⇒ 表格出現／消失後請它重新量（_hdmapFitLegends）。
//   文字一律淺色（紅字在深底上對比不到 3:1）；風速顏色放在色塊上；方向寫「來自 X°」，旁邊的箭頭指順風方向。
var _WIND_PROFILE_MULTI = false, _windProfCtl = null;
function _windRefitHdmap() { if (typeof _hdmapFitLegends === 'function') { try { _hdmapFitLegends(); } catch (e) {} } }
function _windSpan(text, css) { var s = document.createElement('span'); s.textContent = text; if (css) s.style.cssText = css; return s; }
function _windProfileDecor(p, drawn) {
  var map = window.currentMap;
  if (!p) {
    if (_windProfCtl) { try { _windProfCtl.remove(); } catch (e) {} _windProfCtl = null; _windRefitHdmap(); }
    return;
  }
  if (!map) return;
  if (!_windProfCtl) {
    _windProfCtl = L.control({position: 'topleft'});
    _windProfCtl.onAdd = function () {
      var el = L.DomUtil.create('div', 'aerowind-profile-table');
      L.DomEvent.disableClickPropagation(el);
      L.DomEvent.disableScrollPropagation(el);
      return el;
    };
    _windProfCtl.addTo(map);
  }
  var el = _windProfCtl.getContainer(), lim = _windOpts.limitMps, col = p.column, i, TXT = _WIND_PROF.column.text_color;
  el.style.cssText = 'background:rgba(15,20,28,0.85);color:' + TXT + ';padding:6px 8px;border-radius:6px;'
    + 'font:12px "Microsoft JhengHei","PingFang TC","Noto Sans TC",sans-serif;line-height:1.35;max-width:240px;'
    + 'box-shadow:0 1px 6px rgba(0,0,0,0.45);';
  while (el.firstChild) el.removeChild(el.firstChild);
  var title = document.createElement('div');
  title.style.cssText = 'font-weight:bold;margin-bottom:3px;';
  title.textContent = '風場分層（預報，非實測）';
  el.appendChild(title);
  var hs = p.heights.slice().sort(function (a, b) { return b - a; });      // 高的在上面
  for (i = 0; i < hs.length; i++) {
    var h = hs[i], lv = null, on = drawn.indexOf(h) >= 0, row = document.createElement('div');
    if (col && col.levels) col.levels.forEach(function (x) { if (x.h_m === h) lv = x; });
    row.setAttribute('data-h', _windFmtH(h));
    row.setAttribute('data-drawn', on ? '1' : '0');
    row.style.cssText = 'display:flex;align-items:center;gap:5px;padding:1px 4px;border-radius:3px;white-space:nowrap;'
      + (on ? 'background:rgba(255,255,255,0.10);box-shadow:inset 3px 0 0 #90caf9;font-weight:bold;' : '');
    row.appendChild(_windSpan(on ? '▶' : '', 'display:inline-block;width:10px;color:#90caf9;'));
    row.appendChild(_windSpan(_windFmtH(h) + ' m', 'display:inline-block;min-width:40px;text-align:right;'));
    if (lv) {
      var c = window.AeroWind ? window.AeroWind.speedColor(lv.speed_mps, lim) : '#9e9e9e';
      var sw = _windSpan('', 'display:inline-block;width:10px;height:10px;border-radius:2px;background:' + c
        + ';box-shadow:0 0 0 1px rgba(0,0,0,0.7);');
      sw.setAttribute('data-color', c);
      sw.title = '風速顏色（綠 < 0.7×上限、琥珀 < 上限、紅 ≥ 上限；上限 ' + lim + ' m/s）';
      row.appendChild(sw);
      var sp = _windSpan(lv.speed_mps.toFixed(1) + ' m/s', 'display:inline-block;min-width:52px;text-align:right;color:' + TXT + ';');
      sp.setAttribute('data-speed', '1');
      row.appendChild(sp);
      var arr = _windSpan('↑', 'display:inline-block;transform:rotate(' + ((lv.dir_from_deg + 180) % 360).toFixed(1)
        + 'deg);color:' + TXT + ';');
      arr.title = '順風方向（風往這邊吹）';
      row.appendChild(arr);
      row.appendChild(_windSpan('來自 ' + String(Math.round(lv.dir_from_deg) % 360) + '°', 'display:inline-block;min-width:62px;'));
    } else {
      row.appendChild(_windSpan('—（剖面點無資料）', 'color:#b0bec5;'));
    }
    el.appendChild(row);
  }
  var foot = function (t, css) {
    var d = document.createElement('div');
    d.style.cssText = 'margin-top:3px;font-size:11px;color:#b0bec5;white-space:normal;' + (css || '');
    d.textContent = t;
    el.appendChild(d);
  };
  foot(p.legend_desc);
  if (col) foot('剖面點 ' + col.lat.toFixed(5) + ', ' + col.lon.toFixed(5) + '（各層之間不內插；方向＝風的來向）');
  foot(drawn.length ? '2D 只畫 ' + _windFmtH(drawn[0]) + ' m 這一層（俯視時各層重疊）' : '2D 沒有可畫的層', 'color:#90caf9;');
  if (drawn.length && p.height_2d_requested_m !== null && p.height_2d_requested_m !== undefined
      && Math.abs(p.height_2d_requested_m - drawn[0]) > 1e-6) {
    foot('要求的 ' + _windFmtH(p.height_2d_requested_m) + ' m 沒有資料，改畫最接近的 ' + _windFmtH(drawn[0]) + ' m',
         'color:#ffb74d;');
  }
  _windRefitHdmap();
  _windAfterLegendChange();
}
function _windProfileDecorInfo() {
  if (!_windProfCtl) return { kind: 'table', present: false, rows: [] };
  var el = _windProfCtl.getContainer(), r = el.getBoundingClientRect(), rows = [];
  Array.prototype.forEach.call(el.querySelectorAll('[data-h]'), function (x) {
    var sw = x.querySelector('[data-color]'), sp = x.querySelector('[data-speed]');
    rows.push({ h: x.getAttribute('data-h'), drawn: x.getAttribute('data-drawn') === '1', text: x.textContent,
                color: sw ? sw.getAttribute('data-color') : null,
                text_color: sp ? getComputedStyle(sp).color : null, row_bg: getComputedStyle(x).backgroundColor });
  });
  return { kind: 'table', present: !!el.parentNode, rows: rows, text: el.textContent,
           rect: [r.left, r.top, r.width, r.height], bg: getComputedStyle(el).backgroundColor,
           legend_shift: _windLegendShift };
}
"""

_GLUE_CESIUM = r"""
function _windHost() { return viewer.container; }
function _windAdapter() {
  var scene = viewer.scene, camera = scene.camera, ell = scene.globe.ellipsoid, ST = Cesium.SceneTransforms;
  var toWin = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;   // 1.115 是前者，新版改名
  var occ = new Cesium.EllipsoidalOccluder(ell, Cesium.Cartesian3.ZERO);
  var cart = new Cesium.Cartesian3(), win = new Cesium.Cartesian2();
  // 快速投影（2026-10-19）：追機時鏡頭每幀都在動 ⇒ 引擎每幀把每一層的經緯度格網重投影一次；逐點走
  // SceneTransforms.wgs84ToWindowCoordinates 很貴（三層分層風在追機下佔一幀 58% 的時間、掉到 44 FPS）。
  // 3D＋透視鏡頭時：鏡頭沒變就沿用同一個「投影×視圖」矩陣，每點只做一次 4×4 乘法 → 螢幕 CSS 像素；
  // 其他模式（2D／哥倫布／正交）照舊走 Cesium。地球背面（遮蔽）與相機後方（w ≤ 0）一樣回 null。
  // ⚠ 這個函式每幀被呼叫數千次（單層 ~1800、三層 ~5200）：每次只讀一個 getter（viewMatrix）、比 16＋3 個數字、
  //   不配置物件；畫布尺寸與投影矩陣只在要重算時才讀（讀 clientWidth 可能觸發版面重算，逐點讀反而比原本慢）。
  var vp = new Cesium.Matrix4(), vmLast = new Float64Array(16), vpW = 0, vpH = 0, vpFov = NaN, vpAsp = NaN, vpOk = false;
  function vpFresh() {
    var f = camera.frustum;
    if (!(f instanceof Cesium.PerspectiveFrustum)) return false;
    var vm = camera.viewMatrix, i, same = vpOk && f.fov === vpFov && f.aspectRatio === vpAsp;
    for (i = 0; same && i < 16; i++) { if (vm[i] !== vmLast[i]) same = false; }
    if (!same) {
      var P = f.projectionMatrix, cv = scene.canvas;
      if (!P || !cv) return false;
      Cesium.Matrix4.multiply(P, vm, vp);
      occ.cameraPosition = camera.positionWC;               // 遮蔽器也只在鏡頭變時更新（setter 會重算地平線量）
      for (i = 0; i < 16; i++) vmLast[i] = vm[i];
      vpFov = f.fov; vpAsp = f.aspectRatio; vpW = cv.clientWidth; vpH = cv.clientHeight; vpOk = vpW > 0 && vpH > 0;
    }
    return vpOk;
  }
  return {
    project: function (lat, lon, h) {
      var p = Cesium.Cartesian3.fromDegrees(lon, lat, h || 0, ell, cart);
      if (scene.mode === Cesium.SceneMode.SCENE3D) {
        var fast = vpFresh();
        if (!fast) occ.cameraPosition = camera.positionWC;
        if (!occ.isPointVisible(p)) return null;             // 地球背面
        if (fast) {                                           // Matrix4 是 column-major：第 r 列第 c 行 = m[c*4 + r]
          var m = vp, x = p.x, y = p.y, z = p.z;
          var cw = m[3] * x + m[7] * y + m[11] * z + m[15];
          if (!(cw > 1e-9)) return null;                      // 相機後方
          var cx = m[0] * x + m[4] * y + m[8] * z + m[12], cy = m[1] * x + m[5] * y + m[9] * z + m[13];
          var sx = (cx / cw + 1) * 0.5 * vpW, sy = (1 - cy / cw) * 0.5 * vpH;
          // w：引擎的可見區格網在齊次座標內插（追機時靠近鏡頭一格跨幾百公尺，只內插 x、y 會差幾十 px）
          return (isFinite(sx) && isFinite(sy)) ? { x: sx, y: sy, w: cw } : null;
        }
      }
      var w = toWin.call(ST, scene, p, win);                 // 相機後方 ⇒ undefined
      return (w && isFinite(w.x) && isFinite(w.y)) ? { x: w.x, y: w.y } : null;   // 畫面外照樣回座標
    },
    viewKey: function () {
      var c = camera.positionWC, d = camera.directionWC, u = camera.upWC, cv = scene.canvas;
      return [c.x.toFixed(2), c.y.toFixed(2), c.z.toFixed(2), d.x.toFixed(6), d.y.toFixed(6), d.z.toFixed(6),
              u.x.toFixed(6), u.y.toFixed(6), u.z.toFixed(6), cv.clientWidth + 'x' + cv.clientHeight, scene.mode].join('|');
    }
  };
}
function _windAfterCreate(L_) {}
function _windAfterLegendChange() {}
// 分層風場（3D：每層畫在自己的高度）＋剖面柱：剖面點一根 0 m 到最高層的垂直線＋柱頂標題（模式、預報非實測、時間），
//   每層一支順風箭頭（dir_from + 180°，長度 30·√風速、夾 10–150 m）＋點（風速顏色＝引擎的 speedColor，同一個上限）＋
//   淺色文字「<h> m  <s> m/s  來自 <dir>°」（深底；紅字在深底上對比不足）。
//   文字放在箭頭的另一側：每次算圖前（preRender）把各層箭頭投影到螢幕，往右伸得多 ⇒ 文字放左邊，反之放右邊；
//   文字框底下請引擎挖空粒子（setMask），粒子 canvas 疊在 Cesium 之上，不挖空的話軌跡會蓋過字。
//   高度慣例與路徑／起飛柱／SITL 飛機相同：相對起降場、以橢球面為 0（Cartesian3.fromDegrees(lon, lat, h)）。
//   每次變動都要 _requestRender()（requestRenderMode：entity 增刪不會自己算圖）。
var _WIND_PROFILE_MULTI = true, _windColEnts = [], _windColInfo = null, _windColLabels = [], _windColSide = 1;
var _windColHooked = false, _windColMaskOn = false, _windTextCtx = null;
function _windRR() {
  if (typeof _requestRender === 'function') { _requestRender(); return; }
  try { viewer.scene.requestRender(); } catch (e) {}
}
function _windColFont() { return _WIND_PROF.column.label_font_px + 'px "Microsoft JhengHei", "PingFang TC", "Noto Sans TC", sans-serif'; }
function _windTextW(text) {
  try {
    if (!_windTextCtx) _windTextCtx = document.createElement('canvas').getContext('2d');
    _windTextCtx.font = _windColFont();
    return _windTextCtx.measureText(text).width;
  } catch (e) { return 8 * String(text).length; }
}
function _windToWin(lon, lat, h) {
  var ST = Cesium.SceneTransforms, f = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
  var w = f.call(ST, viewer.scene, Cesium.Cartesian3.fromDegrees(lon, lat, h));
  return (w && isFinite(w.x) && isFinite(w.y)) ? w : null;
}
function _windColPickSide() {          // 文字放哪邊：箭頭在螢幕上往右伸的總長 vs 往左；差不多（< 4 px）時不換邊、不閃
  var I = _windColInfo, gap = _WIND_PROF.column.label_gap_px, right = 0, left = 0;
  if (!I) return _windColSide;
  I.levels.forEach(function (lv) {
    if (!lv.tip) return;
    var b = _windToWin(I.lon, I.lat, lv.h_m), t = _windToWin(lv.tip[1], lv.tip[0], lv.tip[2]);
    if (!b || !t) return;
    var dx = t.x - b.x;
    if (dx > 0) right += Math.max(0, dx - gap / 2); else left += Math.max(0, -dx - gap / 2);
  });
  I.arrow_extent_px = { left: left, right: right };
  if (_windColSide > 0 && right > left + 4) return -1;
  if (_windColSide < 0 && left > right + 4) return 1;
  return _windColSide;
}
function _windColLayout() {
  if (!_windColInfo || !_windColLabels.length) {
    if (_windColMaskOn && _windLayer && typeof _windLayer.setMask === 'function') _windLayer.setMask([]);
    _windColMaskOn = false;
    return;
  }
  var S = _WIND_PROF.column, C = Cesium, side = _windColPickSide();
  if (side !== _windColSide) {
    _windColSide = side;
    _windColLabels.forEach(function (L) {
      L.ent.label.horizontalOrigin = side < 0 ? C.HorizontalOrigin.RIGHT : C.HorizontalOrigin.LEFT;
      L.ent.label.pixelOffset = new C.Cartesian2(side * S.label_gap_px, L.dy);
    });
    _windRR();
  }
  var rects = [], ph = S.label_font_px + 2 * S.label_pad_px[1] + 8;
  _windColLabels.forEach(function (L) {
    var w = _windToWin(L.lon, L.lat, L.h);
    if (!w) return;
    var bw = L.tw + 2 * S.label_pad_px[0] + 8;
    var x = side < 0 ? w.x - S.label_gap_px - bw + 4 : w.x + S.label_gap_px - 4;
    rects.push([x, w.y + L.dy - ph / 2, bw, ph]);
  });
  _windColInfo.label_side = side < 0 ? 'left' : 'right';
  _windColInfo.mask = rects;
  if (_windLayer && typeof _windLayer.setMask === 'function') { _windLayer.setMask(rects); _windColMaskOn = true; }
}
function _windColHook() {
  if (_windColHooked) return;
  _windColHooked = true;
  viewer.scene.preRender.addEventListener(function () { try { _windColLayout(); } catch (e) {} });
}
function _windProfileDecor(p, drawn) {
  var C = Cesium, i;
  for (i = 0; i < _windColEnts.length; i++) { try { viewer.entities.remove(_windColEnts[i]); } catch (e) {} }
  _windColEnts = [];
  _windColInfo = null;
  _windColLabels = [];
  var col = p && p.column;
  if (col && col.levels && col.levels.length) {
    var S = _WIND_PROF.column, lim = _windOpts.limitMps, lat = col.lat, lon = col.lon, top = 0, levels = [];
    var cosLat = Math.max(1e-6, Math.cos(lat * Math.PI / 180)), side = _windColSide, n = col.levels.length;
    var textC = C.Color.fromCssColorString(S.text_color), bg = new C.Color(S.label_bg[0], S.label_bg[1], S.label_bg[2], S.label_bg[3]);
    var desc = p.legend_desc || '（預報，非實測）';
    var lab = function (text, dy) {
      return { text: text, font: _windColFont(), fillColor: textC, outlineColor: C.Color.BLACK, outlineWidth: 2,
               style: C.LabelStyle.FILL_AND_OUTLINE, verticalOrigin: C.VerticalOrigin.CENTER,
               horizontalOrigin: side < 0 ? C.HorizontalOrigin.RIGHT : C.HorizontalOrigin.LEFT,
               showBackground: true, backgroundColor: bg, backgroundPadding: new C.Cartesian2(S.label_pad_px[0], S.label_pad_px[1]),
               pixelOffset: new C.Cartesian2(side * S.label_gap_px, dy), disableDepthTestDistance: Number.POSITIVE_INFINITY };
    };
    col.levels.forEach(function (lv) { if (lv.h_m > top) top = lv.h_m; });
    _windColEnts.push(viewer.entities.add({
      name: '風場剖面柱' + desc,
      polyline: { positions: [C.Cartesian3.fromDegrees(lon, lat, 0), C.Cartesian3.fromDegrees(lon, lat, top)],
                  width: S.line_width_px, material: C.Color.WHITE.withAlpha(0.8), arcType: C.ArcType.NONE }
    }));
    // 正上方俯視時各層的點投影到同一處 ⇒ 文字依層序上下錯開 label_step_px（高的往上）；柱頂標題在最上面一行
    var mid = (n - 1) / 2, hdy = -(n - 1 - mid) * S.label_step_px - S.label_step_px, htext = '風場剖面' + desc;
    var hent = viewer.entities.add({ name: '風場剖面標題' + desc, position: C.Cartesian3.fromDegrees(lon, lat, top),
                                     label: lab(htext, hdy) });
    _windColEnts.push(hent);
    _windColLabels.push({ ent: hent, dy: hdy, lon: lon, lat: lat, h: top, tw: _windTextW(htext) });
    col.levels.forEach(function (lv, k) {
      var dy = -(k - mid) * S.label_step_px;
      var css = window.AeroWind ? window.AeroWind.speedColor(lv.speed_mps, lim) : '#9e9e9e';
      var c = C.Color.fromCssColorString(css), calm = lv.speed_mps < S.calm_mps;
      var brg = (lv.dir_from_deg + 180) % 360, b = brg * Math.PI / 180;
      var len = calm ? 0 : Math.min(S.max_m, Math.max(S.min_m, S.m_per_sqrt_mps * Math.sqrt(lv.speed_mps)));
      var lat1 = lat + len * Math.cos(b) / 111320, lon1 = lon + len * Math.sin(b) / (111320 * cosLat);
      var text = _windFmtH(lv.h_m) + ' m  ' + lv.speed_mps.toFixed(1) + ' m/s  來自 ' + (Math.round(lv.dir_from_deg) % 360) + '°';
      var e = {
        name: '風場剖面 ' + _windFmtH(lv.h_m) + ' m' + desc,
        position: C.Cartesian3.fromDegrees(lon, lat, lv.h_m),
        point: { pixelSize: S.point_px, color: c, outlineColor: C.Color.BLACK, outlineWidth: 1.5,
                 disableDepthTestDistance: Number.POSITIVE_INFINITY },
        label: lab(text, dy)
      };
      if (!calm) {
        e.polyline = { positions: [C.Cartesian3.fromDegrees(lon, lat, lv.h_m), C.Cartesian3.fromDegrees(lon1, lat1, lv.h_m)],
                       width: S.arrow_width_px, material: new C.PolylineArrowMaterialProperty(c), arcType: C.ArcType.NONE };
      }
      var ent = viewer.entities.add(e);
      _windColEnts.push(ent);
      _windColLabels.push({ ent: ent, dy: dy, lon: lon, lat: lat, h: lv.h_m, tw: _windTextW(text) });
      levels.push({ h_m: lv.h_m, speed_mps: lv.speed_mps, dir_from_deg: lv.dir_from_deg, bearing_deg: brg,
                    length_m: len, color: css, label: text, label_dy_px: dy, calm: calm, tip: calm ? null : [lat1, lon1, lv.h_m] });
    });
    _windColInfo = { lat: lat, lon: lon, top_m: top, levels: levels, header: htext, header_dy_px: hdy,
                     label_side: side < 0 ? 'left' : 'right', mask: [], text_color: S.text_color, label_bg: S.label_bg.slice() };
    _windColHook();
    try { _windColLayout(); } catch (e) {}
  } else {
    try { _windColLayout(); } catch (e) {}
  }
  _windRR();
}
function _windProfileDecorInfo() {
  return { kind: 'column', present: _windColEnts.length > 0, entities: _windColEnts.length, column: _windColInfo };
}
"""

#: 圖例位置（[離角落水平 px, 垂直 px]）：2D 右上有 Leaflet 圖層控制、3D 右上有 #toolbar，各自讓開
LEGEND_MARGIN = {'leaflet': [10, 60], 'cesium': [140, 8]}
#: 視角變時軌跡怎麼辦（引擎選項 trailsFollowView）：3D 追機／旋轉時鏡頭每幀都在動，舊做法（清掉）每幀都清 ⇒ 只剩點，
#: 改成用粒子的位置歷史以新視角重畫（2026-10-21）；2D 維持舊行為（拖曳時清掉、放開後重新累積）。
TRAILS_FOLLOW_VIEW = {'leaflet': False, 'cesium': True}


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
    prof = {'minPerLayer': PROFILE_PARTICLES_MIN_PER_LAYER, 'group': PROFILE_GROUP, 'prefix': PROFILE_FIELD_PREFIX,
            'column': PROFILE_COLUMN_STYLE}
    return adapter + (_GLUE_COMMON.replace('__LIMIT__', repr(lim)).replace('__LEGEND__', json.dumps(LEGEND_MARGIN[kind]))
                      .replace('__TRAILS_FOLLOW__', json.dumps(TRAILS_FOLLOW_VIEW[kind]))
                      .replace('__PROFILE_CONST__', json.dumps(prof)))


def restore_js(field: dict | None, rows: list | None, options: dict | None, profile: dict | None = None) -> str:
    """頁面重建後（2D 每次 folium 重繪、3D 重新載入）把 Python 端記住的狀態套回去的 JS。

    順序：選項 → 單一風場 → 箭頭 → 分層風場（最後套：它會把單一風場藏起來，清掉分層時再畫回來）。
    profile 是 check_wind_profile 的結果（含目前的 2D 高度／3D 子集）。
    """
    parts = []
    if options:
        parts.append(js_call('setWindOptions', options) + ';')
    if field:
        parts.append(js_call('setWindField', field) + ';')
    if rows:
        parts.append(js_call('setWindArrows', rows) + ';')
    if profile:
        parts.append(js_call('setWindProfile', profile) + ';')
    return '\n'.join(parts)
