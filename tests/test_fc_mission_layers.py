# -*- coding: utf-8 -*-
"""任務疊圖（讀回／匯入預覽）在 2D、3D、DualMapWidget 的值層級鎖 —— 每個 sysid 一層。

不開瀏覽器、不建 QWebEngineView：2D 用裸 MapWidget＋假 page 收 JS，3D 收 `_js` 字串，
Dual 用假的 2D/3D 記呼叫（house style，見 tests/test_wind_layer.py）。

防的是：
① 2D 根本沒有 display_fc_mission（以前靠 hasattr 守著 ⇒ 載入後 2D 看不到也不報錯）。
② 2D 收到三元組（folium 那條路會讓整張圖靜靜消失）⇒ 這裡整份拒絕。
③ 同 sysid 再畫沒有換掉舊的、不同 sysid 互相蓋掉（3D 以前只有一份疊圖）。
④ 整頁重繪（_render_map）後 JS 圖層消失 ⇒ 要有 replay。
⑤ 預覽偷換圍籬來源（display_path* / _auto_build_fence）。
⑥ 圖層鍵互撞：同一架的讀回 (sysid=n) 與匯入預覽共用 'uav<n>' 互相蓋掉；單檔載入走 (None, None)
   把每一層都洗掉 ⇒ 加 `kind`，鍵的規則只有 `mission_view.fc_layer_key` 一份，2D/3D 不分岔。
⑦ 規劃前的清場 `clear_all_display_layers_js()` 掃掉任務圖層但狀態還在 ⇒ 預覽消失又回來。
⑧ 2D tooltip 是 innerHTML：檔名帶 `<b>` 會被渲染／執行 ⇒ Python 端先 html.escape。
⑨ main.html 三個 add* 都要把 layerId 傳進 `_missionTrack`（只數呼叫次數抓不到漏傳）。
"""
from __future__ import annotations

import ast
import json
import os
import pathlib
import re

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

REPO = pathlib.Path(__file__).resolve().parents[1]
MAIN_HTML = REPO / 'ui' / 'widgets' / 'cesium_templates' / 'main.html'

try:
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.widgets.map_widget import MapWidget
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f'Qt 匯入失敗：{globals().get("_QT_ERR", "")}')

from mission import mission_view as mv  # noqa: E402

PATH2D = [(25.001, 121.001), (25.002, 121.002), (25.003, 121.003)]
PATH3D = [(25.001, 121.001, 50.0), (25.002, 121.002, 50.0), (25.003, 121.003, 0.0)]
LOITERS = [{'seq': 2, 'lat': 25.002, 'lon': 121.002, 'alt': 60.0, 'radius_m': 80.0}]
HOME = {'lat': 25.0, 'lon': 121.0, 'alt': 0.0}


class _FakePage:
    def __init__(self):
        self.sent = []

    def runJavaScript(self, js, cb=None):
        self.sent.append(js)
        if cb:
            cb(None)


def _bare2d():
    w = MapWidget.__new__(MapWidget)
    w.custom_page = _FakePage()
    return w


class _NoDelCesium(CesiumMapWidget if _QT_OK else object):
    def __del__(self):
        pass


def _bare3d():
    w = _NoDelCesium.__new__(_NoDelCesium)
    w._sent = []
    w._js = w._sent.append
    return w


def _js_value(js: str, var: str):
    """從注入的 JS 裡把 `var <var> = <json>;` 的值讀回來。"""
    m = re.search(r'var %s = (.*?);\n' % re.escape(var), js, flags=re.S)
    assert m, f'JS 裡沒有 var {var}'
    return json.loads(m.group(1))


def _call(js: str, fn: str):
    m = re.fullmatch(re.escape(fn) + r'\((.*)\)', js, flags=re.S)
    assert m, f'不是 {fn}(...)：{js[:120]}'
    return json.loads('[' + m.group(1) + ']') if m.group(1) else []


# ═══════════════════════════════════════════════════════════════════════
#  2D
# ═══════════════════════════════════════════════════════════════════════
def test_2d_draws_one_layer_per_sysid_and_sends_two_tuples():
    w = _bare2d()
    r1 = MapWidget.display_fc_mission(w, PATH2D, LOITERS, None, home=HOME, sysid=1)
    r2 = MapWidget.display_fc_mission(w, PATH2D[:2], None, None, sysid=2)
    assert r1['layer'] == 'uav1' and r2['layer'] == 'uav2'
    assert r1 == {'layer': 'uav1', 'points': 3, 'loiters': 1, 'home': True,
                  'color': mv.sysid_color(1), 'label': 'UAV-1 任務'}
    assert set(w._fc_missions) == {'uav1', 'uav2'}, '每個 sysid 一層，第二架不可以蓋掉第一架'
    js1, js2 = w.custom_page.sent
    assert _js_value(js1, 'key') == 'uav1' and _js_value(js2, 'key') == 'uav2'
    pts = _js_value(js1, 'pts')
    assert pts == [[25.001, 121.001], [25.002, 121.002], [25.003, 121.003]]
    assert all(len(p) == 2 for p in pts), '2D 一律二元組'
    assert _js_value(js1, 'color') == mv.sysid_color(1) and _js_value(js2, 'color') == mv.sysid_color(2)
    assert _js_value(js1, 'label') == 'UAV-1 任務'
    assert _js_value(js1, 'home') == [25.0, 121.0]
    assert _js_value(js2, 'home') is None
    assert '[[25.002, 121.002, 80.0]].forEach' in js1
    assert 'window._fcMissionLayers' in js1 and 'store[key] = layers' in js1
    # 有 sysid ⇒ 只換自己那層（replace_all=false）
    assert 'if(false){ Object.keys(store).forEach(drop); } else { drop(key); }' in js1
    assert 'L.polyline(pts' in js1 and 'U1\\u25b6' in js1 and 'U1\\u25a0' in js1


def test_2d_same_sysid_replaces_and_other_layers_survive():
    w = _bare2d()
    MapWidget.display_fc_mission(w, PATH2D, sysid=3)
    MapWidget.display_fc_mission(w, PATH2D, sysid=4)
    MapWidget.display_fc_mission(w, PATH2D[:1], sysid=3)
    assert set(w._fc_missions) == {'uav3', 'uav4'}
    assert w._fc_missions['uav3']['path'] == [[25.001, 121.001]], '同 sysid 再畫 = 換掉'
    assert len(w._fc_missions['uav4']['path']) == 3, '別架的層不動'
    assert _js_value(w.custom_page.sent[-1], 'key') == 'uav3'


def test_2d_without_sysid_keeps_the_old_single_overlay_semantics():
    """沒有 sysid（「讀回飛控」那條路目前的呼叫法）⇒ 清掉全部層、畫成單一 'fc' 層。"""
    w = _bare2d()
    MapWidget.display_fc_mission(w, PATH2D, sysid=1)
    MapWidget.display_fc_mission(w, PATH2D, sysid=2)
    r = MapWidget.display_fc_mission(w, PATH2D, LOITERS, '#00E5FF', home=HOME)
    assert r['layer'] == 'fc' and r['color'] == '#00E5FF' and r['label'] == 'FC Mission'
    assert set(w._fc_missions) == {'fc'}
    js = w.custom_page.sent[-1]
    assert 'if(true){ Object.keys(store).forEach(drop); }' in js
    assert '>\\u25b6<' in js, '沒有 sysid 就沒有 U<n> 徽章'


def test_2d_rejects_three_tuples_loudly_and_leaves_no_state():
    """⚠️⚠️ 三元組在 folium 那條路會讓整張 2D 圖靜靜消失；這裡整份拒絕，不靜默修正。"""
    w = _bare2d()
    with pytest.raises(ValueError, match='二元組'):
        MapWidget.display_fc_mission(w, PATH3D, sysid=1)
    with pytest.raises(ValueError, match='二元組'):
        MapWidget.display_fc_mission(w, [(25.0,)], sysid=1)
    with pytest.raises(ValueError, match='有限數'):
        MapWidget.display_fc_mission(w, [(float('nan'), 121.0)], sysid=1)
    assert w.custom_page.sent == [], '拒絕時什麼都不送'
    assert not w.__dict__.get('_fc_missions'), '拒絕時不留狀態（否則整頁重繪會畫出半份）'


def test_2d_empty_path_still_draws_home_and_loiters():
    w = _bare2d()
    r = MapWidget.display_fc_mission(w, [], LOITERS, home=HOME, sysid=5)
    assert r['points'] == 0 and r['home'] and r['loiters'] == 1
    js = w.custom_page.sent[-1]
    assert _js_value(js, 'pts') == [] and _js_value(js, 'home') == [25.0, 121.0]


def test_2d_fit_only_when_asked():
    w = _bare2d()
    MapWidget.display_fc_mission(w, PATH2D, sysid=1)
    assert 'if(false && (pts.length || home))' in w.custom_page.sent[-1]
    MapWidget.display_fc_mission(w, PATH2D, sysid=1, fit=True)
    assert 'if(true && (pts.length || home))' in w.custom_page.sent[-1]
    assert 'map.fitBounds(b' in w.custom_page.sent[-1]


def test_2d_clear_one_layer_or_all():
    w = _bare2d()
    MapWidget.display_fc_mission(w, PATH2D, sysid=1)
    MapWidget.display_fc_mission(w, PATH2D, sysid=2)
    assert MapWidget.clear_fc_mission(w, 1) is True
    assert set(w._fc_missions) == {'uav2'}
    js = w.custom_page.sent[-1]
    assert _js_value(js, 'key') == 'uav1' and 'delete store[key]' in js
    assert MapWidget.clear_fc_mission(w, 1) is False, '已經沒有了'
    assert MapWidget.clear_fc_mission(w) is True
    assert w._fc_missions == {}
    assert 'window._fcMissionLayers = {}' in w.custom_page.sent[-1]
    assert MapWidget.clear_fc_mission(w) is False


def test_2d_replays_layers_after_a_full_page_rerender():
    """⚠️ folium 每次 _render_map 都整頁重載，JS 圖層會消失 ⇒ 注入的 HTML 要把記住的層畫回來。"""
    w = _bare2d()
    assert MapWidget._fc_mission_js_block(w) == '', '沒有東西就不注入'
    MapWidget.display_fc_mission(w, PATH2D, sysid=1)
    MapWidget.display_fc_mission(w, PATH2D[:2], sysid=2)
    block = MapWidget._fc_mission_js_block(w)
    assert block.count('var key = ') == 2
    assert '"uav1"' in block and '"uav2"' in block
    assert 'if(false){ Object.keys(store).forEach(drop); }' in block, 'replay 不可以互相清掉'
    assert block.count('<script>') == 1
    MapWidget.clear_fc_mission(w)
    assert MapWidget._fc_mission_js_block(w) == ''


def test_2d_inject_javascript_actually_appends_the_replay_block():
    """宣告了 _fc_mission_js_block 不等於有人呼叫它 —— AST 確認 _inject_javascript 真的接了，
    而且是 `js_code += self._fc_mission_js_block()`（呼叫了卻丟掉回傳值一樣是 no-op）。"""
    src = (REPO / 'ui' / 'widgets' / 'map_widget.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_inject_javascript')
    calls = {c.func.attr for c in ast.walk(fn) if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}
    assert '_fc_mission_js_block' in calls and '_wind_js_block' in calls
    aug = [n for n in ast.walk(fn)
           if isinstance(n, ast.AugAssign) and isinstance(n.target, ast.Name) and n.target.id == 'js_code'
           and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Attribute)
           and n.value.func.attr == '_fc_mission_js_block']
    assert len(aug) == 1, '_fc_mission_js_block() 的回傳值必須 += 進 js_code'


def test_2d_inject_javascript_output_really_contains_the_replay():
    """值層級：裸 MapWidget 記著兩層，跑真的 _inject_javascript ⇒ 輸出 HTML 裡有把它們畫回來的 JS。"""
    w = _bare2d()
    w.corners = []
    MapWidget.display_fc_mission(w, PATH2D, sysid=1)
    MapWidget.display_fc_mission(w, PATH2D[:2], sysid=1, kind='import')
    out = MapWidget._inject_javascript(w, '<html><body></body></html>', 'map_x')
    assert 'window._fcMissionLayers' in out and '"uav1"' in out and '"uav1-import"' in out
    assert out.index('window._fcMissionLayers') < out.rindex('</html>')
    assert 'if(true){ Object.keys(store).forEach(drop); }' not in out, 'replay 不可以互相清掉'


# ═══════════════════════════════════════════════════════════════════════
#  圖層鍵（kind）：讀回 / 匯入預覽 / 單檔載入 不互撞
# ═══════════════════════════════════════════════════════════════════════
def test_layer_key_scheme_is_one_function_shared_by_both_maps():
    assert mv.fc_layer_key() == 'fc' and mv.fc_layer_key(None, None) == 'fc'
    assert mv.fc_layer_key(3) == 'uav3'
    assert mv.fc_layer_key(3, 'import') == 'uav3-import'
    assert mv.fc_layer_key(None, 'loaded') == 'loaded'
    assert mv.fc_layer_key('3', ' import ') == 'uav3-import'
    with pytest.raises(ValueError):
        mv.fc_layer_key(3, '')
    with pytest.raises(ValueError):
        mv.fc_layer_key(None, '  ')
    assert mv.fc_layer_selector() == (None, None)
    assert mv.fc_layer_selector(3) == ('uav3', 'uav3-')
    assert mv.fc_layer_selector(3, 'import') == ('uav3-import', None)
    assert mv.fc_layer_selector(None, 'loaded') == ('loaded', None)
    assert all(mv.fc_layer_matches(k) for k in ('fc', 'uav3', 'uav3-import', 'loaded'))
    assert [k for k in ('fc', 'uav3', 'uav3-import', 'uav30', 'uav30-import', 'loaded')
            if mv.fc_layer_matches(k, 3)] == ['uav3', 'uav3-import']
    assert [k for k in ('uav3', 'uav3-import', 'uav3-loaded') if mv.fc_layer_matches(k, 3, 'import')] == ['uav3-import']
    assert [k for k in ('fc', 'loaded', 'uav3-loaded') if mv.fc_layer_matches(k, None, 'loaded')] == ['loaded']
    # 2D 與 3D 都從這一份拿，不各自發明
    assert MapWidget._fc_layer_key(3, 'import') == mv.fc_layer_key(3, 'import')


def test_2d_readback_import_preview_and_single_load_get_their_own_layers():
    """⚠️ 以前讀回 (sysid=1) 與匯入預覽 (sysid=1) 都是 'uav1' 互相蓋掉；單檔載入 (None, None) 把每層洗掉。"""
    w = _bare2d()
    r_rb = MapWidget.display_fc_mission(w, PATH2D, sysid=1)
    r_im = MapWidget.display_fc_mission(w, PATH2D[:2], sysid=1, kind='import')
    r_ld = MapWidget.display_fc_mission(w, PATH2D[:1], kind='loaded')
    assert (r_rb['layer'], r_im['layer'], r_ld['layer']) == ('uav1', 'uav1-import', 'loaded')
    assert set(w._fc_missions) == {'uav1', 'uav1-import', 'loaded'}, '三者並存，沒有誰蓋掉誰'
    assert len(w._fc_missions['uav1']['path']) == 3 and len(w._fc_missions['uav1-import']['path']) == 2
    assert w._fc_missions['uav1-import']['kind'] == 'import' and w._fc_missions['uav1']['kind'] is None
    js_rb, js_im, js_ld = w.custom_page.sent
    assert _js_value(js_im, 'key') == 'uav1-import' and _js_value(js_ld, 'key') == 'loaded'
    for js in (js_rb, js_im, js_ld):
        assert 'if(false){ Object.keys(store).forEach(drop); } else { drop(key); }' in js, '有鍵就只換自己那層'
    # kind 有 sysid 的照樣有 U<n> 徽章與該機顏色；單檔載入沒有徽章、預設青色
    assert 'U1\\u25b6' in js_im and _js_value(js_im, 'color') == mv.sysid_color(1)
    assert '>\\u25b6<' in js_ld and _js_value(js_ld, 'color') == '#00E5FF' and r_ld['label'] == 'FC Mission'
    # 同一把鍵再畫 = 換掉自己
    MapWidget.display_fc_mission(w, PATH2D, sysid=1, kind='import')
    assert len(w._fc_missions['uav1-import']['path']) == 3 and len(w._fc_missions) == 3
    # 舊語意 (None, None) 仍然清光
    MapWidget.display_fc_mission(w, PATH2D)
    assert set(w._fc_missions) == {'fc'}
    assert 'if(true){ Object.keys(store).forEach(drop); }' in w.custom_page.sent[-1]
    # 空的 kind 在算鍵時就炸、不留狀態
    with pytest.raises(ValueError):
        MapWidget.display_fc_mission(w, PATH2D, sysid=2, kind='')
    assert set(w._fc_missions) == {'fc'}


def test_2d_clear_by_sysid_takes_the_whole_aircraft_but_not_others():
    w = _bare2d()
    MapWidget.display_fc_mission(w, PATH2D, sysid=1)
    MapWidget.display_fc_mission(w, PATH2D, sysid=1, kind='import')
    MapWidget.display_fc_mission(w, PATH2D, sysid=10, kind='import')     # 'uav10-import'：前綴 'uav1' 不可以誤殺
    MapWidget.display_fc_mission(w, PATH2D, sysid=2)
    MapWidget.display_fc_mission(w, PATH2D, kind='loaded')
    assert MapWidget.clear_fc_mission(w, 1) is True
    assert set(w._fc_missions) == {'uav10-import', 'uav2', 'loaded'}
    js = w.custom_page.sent[-1]
    assert _js_value(js, 'key') == 'uav1' and _js_value(js, 'prefix') == 'uav1-'
    assert 'delete store[key]' in js and 'k.indexOf(prefix) === 0' in js
    assert MapWidget.clear_fc_mission(w, 1) is False
    # sysid+kind ⇒ 恰好那一層
    MapWidget.display_fc_mission(w, PATH2D, sysid=2, kind='import')
    assert MapWidget.clear_fc_mission(w, 2, kind='import') is True
    assert set(w._fc_missions) == {'uav10-import', 'uav2', 'loaded'}
    js = w.custom_page.sent[-1]
    assert _js_value(js, 'key') == 'uav2-import' and _js_value(js, 'prefix') is None
    # kind 單獨 ⇒ 恰好那一層
    assert MapWidget.clear_fc_mission(w, kind='loaded') is True
    assert set(w._fc_missions) == {'uav10-import', 'uav2'}
    assert _js_value(w.custom_page.sent[-1], 'key') == 'loaded'
    assert MapWidget.clear_fc_mission(w, kind='loaded') is False
    # 都 None ⇒ 全部
    assert MapWidget.clear_fc_mission(w) is True and w._fc_missions == {}


def test_2d_label_is_html_escaped_before_it_reaches_the_tooltip():
    """⚠️ bindTooltip 是 innerHTML：檔名裡的標籤滑過去就會被渲染／執行。#k、起點、終點、Home、盤旋
    的尾綴全都接在同一個 JS 變數 label 後面，所以轉義一次就全覆蓋；replay 走同一個產生器。"""
    w = _bare2d()
    evil = '<b onmouseover="alert(1)">x</b> & "y"'
    r = MapWidget.display_fc_mission(w, PATH2D, LOITERS, home=HOME, sysid=1, label=evil)
    js = w.custom_page.sent[-1]
    assert _js_value(js, 'label') == '&lt;b onmouseover=&quot;alert(1)&quot;&gt;x&lt;/b&gt; &amp; &quot;y&quot;'
    assert '<b ' not in js and 'onmouseover=\\"' not in js
    assert "bindTooltip(label + ' #' + (i + 1))" in js, '#k 尾綴接在已轉義的 label 後面'
    assert r['label'] == evil and w._fc_missions['uav1']['label'] == evil, '狀態與回傳保留原文，只在進 JS 時轉義'
    block = MapWidget._fc_mission_js_block(w)
    assert '&lt;b onmouseover' in block and '<b ' not in block


def _sweep_js() -> str:
    """clear_all_display_layers_js() 送出的那段 JS（AST 取 runJavaScript 的字串常數）。"""
    src = (REPO / 'ui' / 'widgets' / 'map_widget.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'clear_all_display_layers_js')
    consts = [c.args[0].value for c in ast.walk(fn)
              if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute) and c.func.attr == 'runJavaScript'
              and c.args and isinstance(c.args[0], ast.Constant) and isinstance(c.args[0].value, str)]
    assert len(consts) == 1
    return consts[0]


def test_2d_planning_sweep_skips_fc_mission_layers_and_leaves_state_alone():
    """⚠️ 規劃前的清場（map.eachLayer 掃 polyline/marker）以前連任務圖層一起掃掉，但 _fc_missions /
    window._fcMissionLayers 還在 ⇒ 預覽消失、下一次整頁重繪又回來。現在：圖層打 options.fcMission=true，
    掃描跳過；狀態與畫面一致（都留著）。"""
    sweep = _sweep_js()
    i_each, i_skip = sweep.index('map.eachLayer('), sweep.index('layer.options.fcMission')
    i_first_remove = sweep.index('map.removeLayer(layer)')
    assert i_each < i_skip < i_first_remove, '跳過要在 eachLayer 回呼裡、第一個 removeLayer 之前'
    assert 'if(layer.options && layer.options.fcMission) return;' in sweep
    # 建圖層那邊：每一個 L.polyline / circleMarker / marker / circle 都打了旗標
    w = _bare2d()
    w.paths, w.transit_paths, w._swarm_data = [], [], None
    MapWidget.display_fc_mission(w, PATH2D, LOITERS, home=HOME, sysid=1)
    js = w.custom_page.sent[-1]
    n_ctor = len(re.findall(r'\bL\.(?:polyline|circleMarker|marker|circle)\(', js))
    # polyline、circleMarker、起點／終點／Home 三個 marker、盤旋 circle ＝ 6 個建構子
    assert n_ctor == 6 and js.count('fcMission:true') == n_ctor, '每個 Leaflet 圖層都要帶 fcMission:true'
    # 值層級：真的跑清場，狀態不動、送出的就是那段 JS
    MapWidget.clear_all_display_layers_js(w)
    assert w.custom_page.sent[-1] == sweep
    assert set(w._fc_missions) == {'uav1'}, '清場不碰任務疊圖的狀態（畫面也沒掃掉，兩邊一致）'
    assert w.paths == [] and w._swarm_data is None


def test_2d_display_does_not_touch_corners_paths_or_fence():
    src = (REPO / 'ui' / 'widgets' / 'map_widget.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for name in ('display_fc_mission', 'clear_fc_mission', '_fc_mission_js'):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        calls = {c.func.attr for c in ast.walk(fn) if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}
        assert not calls & {'_render_map', 'display_path', 'display_paths', 'display_fw_paths',
                            '_auto_build_fence', 'clear_paths', 'clear_corners'}, f'{name} 有副作用：{calls}'


# ═══════════════════════════════════════════════════════════════════════
#  3D
# ═══════════════════════════════════════════════════════════════════════
def test_3d_with_sysid_uses_its_own_layer_and_threads_the_id_through():
    w = _bare3d()
    CesiumMapWidget.display_fc_mission(w, PATH3D, LOITERS, None, home=HOME, sysid=3)
    sent = w._sent
    assert sent[0] == 'clearMissionLayer("uav3")', '有 sysid 只清自己那層'
    pts, color, label, layer = _call(sent[1], 'addMissionRoute')
    assert pts == [[25.001, 121.001, 50.0], [25.002, 121.002, 50.0], [25.003, 121.003, 0.0]]
    assert color == mv.sysid_color(3) and label == 'UAV-3 任務' and layer == 'uav3'
    lat, lon, alt, layer, text = _call(sent[2], 'addMissionHome')
    assert (lat, lon, alt, layer, text) == (25.0, 121.0, 0.0, 'uav3', 'H3')
    lat, lon, r, alt, color, layer = _call(sent[3], 'addMissionLoiter')
    assert (lat, lon, r, alt, layer) == (25.002, 121.002, 80.0, 60.0, 'uav3')
    assert color == mv.sysid_color(3)
    assert len(sent) == 4


def test_3d_without_sysid_keeps_clear_all_semantics():
    w = _bare3d()
    CesiumMapWidget.display_fc_mission(w, PATH3D, home=HOME)
    assert w._sent[0] == 'clearMissionOverlay()'
    _pts, color, label, layer = _call(w._sent[1], 'addMissionRoute')
    assert (color, label, layer) == ('#00E5FF', 'FC Mission', None)
    assert _call(w._sent[2], 'addMissionHome')[3:] == [None, 'H']


def test_3d_clear_one_or_all():
    w = _bare3d()
    CesiumMapWidget.clear_fc_mission(w, 2)
    CesiumMapWidget.clear_fc_mission(w)
    assert w._sent == ['clearMissionLayerFamily("uav2")', 'clearMissionOverlay()'], \
        '只給 sysid ⇒ 整架（uav2 與 uav2-*），與 2D 同一條規則'


def test_3d_kind_keys_match_2d_and_clear_selects_the_same_layers():
    w = _bare3d()
    CesiumMapWidget.display_fc_mission(w, PATH3D, LOITERS, None, home=HOME, sysid=3, kind='import')
    assert w._sent[0] == 'clearMissionLayer("uav3-import")'
    _pts, color, label, layer = _call(w._sent[1], 'addMissionRoute')
    assert (color, label, layer) == (mv.sysid_color(3), 'UAV-3 任務', 'uav3-import')
    assert _call(w._sent[2], 'addMissionHome')[3:] == ['uav3-import', 'H3']
    assert _call(w._sent[3], 'addMissionLoiter')[5] == 'uav3-import'
    assert layer == mv.fc_layer_key(3, 'import'), '3D 的 layerId 就是 2D 的鍵'
    w._sent.clear()
    CesiumMapWidget.display_fc_mission(w, PATH3D, home=HOME, kind='loaded')
    assert w._sent[0] == 'clearMissionLayer("loaded")', 'kind 單獨 ⇒ 只換自己這層，不是 clearMissionOverlay'
    assert _call(w._sent[1], 'addMissionRoute')[1:] == ['#00E5FF', 'FC Mission', 'loaded']
    assert _call(w._sent[2], 'addMissionHome')[3:] == ['loaded', 'H']
    w._sent.clear()
    CesiumMapWidget.clear_fc_mission(w, 3, kind='import')
    CesiumMapWidget.clear_fc_mission(w, kind='loaded')
    CesiumMapWidget.clear_fc_mission(w, 3)
    assert w._sent == ['clearMissionLayer("uav3-import")', 'clearMissionLayer("loaded")',
                       'clearMissionLayerFamily("uav3")']
    with pytest.raises(ValueError):
        CesiumMapWidget.display_fc_mission(w, PATH3D, sysid=3, kind='')


class TestMainHtmlMissionLayers:
    HTML = MAIN_HTML.read_text(encoding='utf-8')

    def _fn(self, name):
        m = re.search(r'function %s\(.*?\n\}' % re.escape(name), self.HTML, flags=re.S)
        assert m, f'main.html 找不到 {name}'
        return m.group(0)

    def test_layer_table_and_per_layer_clear_exist(self):
        assert 'let _missionLayers = {}' in self.HTML
        assert 'function clearMissionLayer(layerId)' in self.HTML
        assert 'function clearMissionOverlay()' in self.HTML, '舊的「清全部」要留著'
        body = self._fn('clearMissionOverlay')
        assert '_missionLayers = {}' in body, '清全部也要清圖層表，否則下次 clearMissionLayer 會去 remove 已經沒了的 entity'
        body = self._fn('clearMissionLayer')
        assert 'viewer.entities.remove' in body and '_missionEntities = _missionEntities.filter' in body

    @pytest.mark.parametrize('fn, nparams', [('addMissionRoute', 4), ('addMissionHome', 5), ('addMissionLoiter', 6)])
    def test_add_functions_take_a_layer_id_and_record_it(self, fn, nparams):
        m = re.search(r'function %s\(([^)]*)\)' % fn, self.HTML)
        params = [p.strip() for p in m.group(1).split(',')]
        assert len(params) == nparams and 'layerId' in params
        body = self._fn(fn)
        n_track = body.count('_missionTrack(')
        assert n_track >= 1 and n_track == body.count('_missionEntities.push(')
        # ⚠ 只數呼叫次數抓不到「_missionTrack(e) 少傳 layerId」—— 每一個 _missionTrack(viewer.entities.add({...})
        #   都要以 `}), layerId))` 收尾，entity 才進得了那一層、clearMissionLayer 才清得掉。
        assert n_track == len(re.findall(r'\),\s*layerId\)\)', body)), f'{fn}：有 _missionTrack 沒把 layerId 傳進去'
        assert n_track == body.count('_missionTrack(viewer.entities.add(')
        assert '_tacticalEntities' not in body

    def test_track_helper_only_records_when_a_layer_is_given(self):
        body = self._fn('_missionTrack')
        assert 'layerId !== undefined && layerId !== null' in body
        assert 'return e' in body

    def test_family_clear_takes_the_aircraft_and_all_its_kinds(self):
        """`clearMissionLayerFamily('uav3')` = 'uav3' 本身 + 'uav3-*'（與 2D clear_fc_mission(sysid) 同規則）。"""
        assert 'function clearMissionLayerFamily(layerId)' in self.HTML
        body = self._fn('clearMissionLayerFamily')
        assert "String(layerId) + '-'" in body and 'clearMissionLayer(k)' in body
        assert 'k === layerId || k.indexOf(prefix) === 0' in body
        assert 'Object.keys(_missionLayers)' in body

    def test_point_number_prefix_still_recognises_the_aircraft_with_a_kind_suffix(self):
        """逐點編號前的 '3:' 由 layerId 的正則取機號：'uav3' 與 'uav3-import' 都要是 3，'loaded' 沒有。"""
        body = self._fn('addMissionRoute')
        m = re.search(r'/(\^uav[^/]*)/\.exec\(', body)
        assert m, 'addMissionRoute 裡找不到取機號的正則'
        pat = re.compile(m.group(1))
        assert pat.match('uav3').group(1) == '3' and pat.match('uav3-import').group(1) == '3'
        assert pat.match('uav12-loaded').group(1) == '12'
        assert pat.match('loaded') is None and pat.match('uav3x') is None and pat.match('xuav3') is None


# ═══════════════════════════════════════════════════════════════════════
#  DualMapWidget
# ═══════════════════════════════════════════════════════════════════════
class _M:
    def __init__(self, tag, calls):
        self.tag, self.calls = tag, calls

    def display_fc_mission(self, *a, **k):
        self.calls.append((self.tag, 'display_fc_mission', a, k))
        return {'layer': k.get('sysid') and f"uav{k['sysid']}", 'points': len(a[0] or [])}

    def clear_fc_mission(self, *a, **k):
        self.calls.append((self.tag, 'clear_fc_mission', a, k))
        return True

    def fly_to_position(self, *a, **k):
        self.calls.append((self.tag, 'fly_to_position', a, k))


def _dual():
    calls = []
    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = _M('2d', calls), _M('3d', calls)
    return d, calls


def test_dual_calls_both_maps_directly_with_the_right_shapes_and_sysid():
    """⚠️ 以前用 hasattr 守著 ⇒ 2D 沒有方法就靜靜跳過。現在是直接呼叫：假的 2D 沒方法會炸。"""
    d, calls = _dual()
    out = DualMapWidget.display_fc_mission(d, PATH2D, PATH3D, LOITERS, home=HOME, sysid=2)
    assert [c[:2] for c in calls] == [('2d', 'display_fc_mission'), ('3d', 'display_fc_mission')]
    (_t, _n, a2, k2), (_t3, _n3, a3, k3) = calls
    assert a2[0] == PATH2D and a3[0] == PATH3D, '2D 收二元組、3D 收三元組，不是同一份'
    assert a2[1] is LOITERS and a3[1] is LOITERS
    assert a2[2] == a3[2] == mv.sysid_color(2)
    assert k2['home'] is HOME and k3['home'] is HOME
    assert k2['sysid'] == 2 and k3['sysid'] == 2
    assert k2['label'] == k3['label'] == 'UAV-2 任務'
    assert k2['fit'] is False
    assert out['layer'] == 'uav2' and out['color'] == mv.sysid_color(2) and out['2d']['points'] == 3
    assert not any(n == 'fly_to_position' for _t, n, _a, _k in calls)


def test_dual_without_sysid_passes_none_and_default_colour():
    d, calls = _dual()
    DualMapWidget.display_fc_mission(d, PATH2D, PATH3D, home=None)
    # ⚠ 先確認兩張圖都被呼叫到：沒有呼叫的話底下的 for 一次都不跑，整條測試是空的
    assert [c[:2] for c in calls] == [('2d', 'display_fc_mission'), ('3d', 'display_fc_mission')]
    for _t, _n, a, k in calls:
        assert k['sysid'] is None and k['kind'] is None and a[2] == '#00E5FF' and k['label'] == 'FC Mission'


def test_dual_forwards_kind_to_both_maps_for_display_and_clear():
    """kind 在 Dual 這層只是轉發；兩張圖拿到同一組 (sysid, kind) 才會算出同一把鍵。"""
    d, calls = _dual()
    out = DualMapWidget.display_fc_mission(d, PATH2D, PATH3D, LOITERS, home=HOME, sysid=2, kind='import')
    assert [c[:2] for c in calls] == [('2d', 'display_fc_mission'), ('3d', 'display_fc_mission')]
    for _t, _n, _a, k in calls:
        assert k['sysid'] == 2 and k['kind'] == 'import' and k['label'] == 'UAV-2 任務'
    assert out['color'] == mv.sysid_color(2)
    calls.clear()
    DualMapWidget.display_fc_mission(d, PATH2D, PATH3D, kind='loaded')
    for _t, _n, _a, k in calls:
        assert k['sysid'] is None and k['kind'] == 'loaded'
    calls.clear()
    assert DualMapWidget.clear_fc_mission(d, 2, kind='import') == {'2d': True, '3d': None}
    DualMapWidget.clear_fc_mission(d, kind='loaded')
    DualMapWidget.clear_fc_mission(d, 2)
    assert [(c[0], c[2], c[3]) for c in calls] == [
        ('2d', (2,), {'kind': 'import'}), ('3d', (2,), {'kind': 'import'}),
        ('2d', (None,), {'kind': 'loaded'}), ('3d', (None,), {'kind': 'loaded'}),
        ('2d', (2,), {'kind': None}), ('3d', (2,), {'kind': None}),
    ]


def test_dual_fit_flies_3d_to_the_mission_centre():
    d, calls = _dual()
    DualMapWidget.display_fc_mission(d, PATH2D, PATH3D, home=HOME, sysid=1, fit=True)
    k2 = calls[0][3]
    assert k2['fit'] is True
    fly = [c for c in calls if c[1] == 'fly_to_position']
    assert len(fly) == 1 and fly[0][0] == '3d'
    lat, lon, alt, rng = fly[0][2]
    assert lat == pytest.approx((25.001 + 25.002 + 25.003 + 25.0) / 4)
    assert lon == pytest.approx((121.001 + 121.002 + 121.003 + 121.0) / 4)
    assert rng >= 600.0


def test_dual_fit_with_nothing_to_show_does_not_move_the_camera():
    d, calls = _dual()
    DualMapWidget.display_fc_mission(d, [], [], fit=True)
    assert not any(c[1] == 'fly_to_position' for c in calls)


def test_dual_clear_forwards_sysid_to_both():
    d, calls = _dual()
    assert DualMapWidget.clear_fc_mission(d, 4) == {'2d': True, '3d': None}
    DualMapWidget.clear_fc_mission(d)
    assert [(c[0], c[2]) for c in calls] == [('2d', (4,)), ('3d', (4,)), ('2d', (None,)), ('3d', (None,))]
    assert all(c[3] == {'kind': None} for c in calls)


def test_dual_has_no_hasattr_guard_and_no_fence_side_effect():
    src = (REPO / 'ui' / 'widgets' / 'dual_map_widget.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for name in ('display_fc_mission', 'clear_fc_mission'):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        names = set()
        for c in ast.walk(fn):
            if isinstance(c, ast.Call):
                names.add(c.func.attr if isinstance(c.func, ast.Attribute) else getattr(c.func, 'id', None))
        assert 'hasattr' not in names, f'{name} 還在用 hasattr 守 ⇒ 方法不存在會靜靜跳過'
        assert not names & {'_auto_build_fence', 'display_path', 'display_paths', 'display_fw_paths'}
