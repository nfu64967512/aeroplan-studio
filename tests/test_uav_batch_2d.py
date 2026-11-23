# -*- coding: utf-8 -*-
"""tests/test_uav_batch_2d.py — 2D 與 Dual 地圖的 UAV 批次更新（2026-10-17 效能）。

以前 `MapWidget.update_uav_position` 每架每幀送兩段 ~2 KB JS（SVG data URL 每次都送、每幀重建 L.divIcon）。
現在頁面注入時定義 `window.aeroUavUpdate(batch)`，之後只送一小段 JSON；Dual 層負責「3D 可見才推 3D、
2D 藏在後面時每 N 批推一次」。
"""
from __future__ import annotations

import json
import os
import re

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

try:
    import ui.main_window  # noqa: F401
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    from ui.widgets.map_widget import MapWidget
    from ui.widgets import dual_map_widget as dmw
    from ui.widgets.dual_map_widget import DualMapWidget
    _QT_OK = True
except Exception as _e:                       # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')


class _Page:
    def __init__(self):
        self.js = []

    def runJavaScript(self, s, *a, **k):
        self.js.append(s)


def _bare2d(page):
    w = MapWidget.__new__(MapWidget)
    w.custom_page = page
    return w


def _item(sid, lat=23.45):
    return {'sysid': sid, 'lat': lat, 'lon': 120.28, 'alt': 100.0, 'hdg': 45.0, 'spd': 18.0,
            'mode': 'AUTO', 'armed': True, 'vt': 'VTOL', 'pitch': 1.0, 'roll': 2.0}


@qt_only
class Test2D:
    def test_batch_is_one_small_js_call_without_svg(self):
        p = _Page()
        w = _bare2d(p)
        assert w.update_uav_positions([_item(1), _item(2), _item(3)]) == 3
        assert len(p.js) == 1, '一個批次只能一次 runJavaScript'
        js = p.js[0]
        assert js.startswith('if(window.aeroUavUpdate){window.aeroUavUpdate(')
        assert '<svg' not in js and 'data:image' not in js, 'SVG 不可以每次都送'
        payload = json.loads(re.search(r'aeroUavUpdate\((.*)\);\}$', js).group(1))
        assert [u['sysid'] for u in payload] == [1, 2, 3]
        assert set(payload[0]) == {'sysid', 'lat', 'lon', 'alt', 'hdg', 'spd', 'mode', 'armed', 'vt'}, '多餘鍵要剝掉'
        assert len(js) < 600, f'三架的批次 {len(js)} B 太大'

    def test_course_is_passed_through_when_given(self):
        p = _Page()
        w = _bare2d(p)
        it = dict(_item(1)); it['crs'] = 123.456
        w.update_uav_positions([it])
        payload = json.loads(re.search(r'aeroUavUpdate\((.*)\);\}$', p.js[0]).group(1))
        assert payload[0]['crs'] == 123.5 and payload[0]['hdg'] == 45.0
        block = w._uav_js_block()
        assert "(u.crs !== undefined && u.spd > 2) ? u.crs : u.hdg" in block, '速度夠時要用航跡方向轉圖示'
        assert 'transition: transform' in block, '批次之間要 CSS 補間'

    def test_single_api_wraps_the_batch(self):
        p = _Page()
        w = _bare2d(p)
        w.update_uav_position(23.45, 120.28, 100.0, 45.0, 18.0, sysid=4, mode='LOITER', armed=False, vehicle_type='COPTER')
        assert len(p.js) == 1 and '"sysid":4' in p.js[0] and '"vt":"COPTER"' in p.js[0]

    def test_empty_or_bad_items_do_not_send(self):
        p = _Page()
        w = _bare2d(p)
        assert w.update_uav_positions([]) == 0 and w.update_uav_positions([{'nope': 1}]) == 0
        assert p.js == []

    def test_page_block_defines_the_updater_and_reuses_markers(self):
        w = _bare2d(_Page())
        block = w._uav_js_block()
        assert 'window.aeroUavUpdate = function(batch)' in block
        assert 'window._uavMarkers' in block and 'window._uavTrails' in block and 'window._uavTrailPts' in block, \
            '狀態名要跟 clear_uav 共用'
        assert 'm.setLatLng(' in block and "querySelector('.uav-rot')" in block, '要重用標記：setLatLng ＋ 轉向'
        assert 'L.divIcon(' in block and block.count('L.divIcon(') == 1, 'divIcon 只在建立時做一次'
        assert 'pts.length > 800' in block
        assert 'if (!window.currentMap || !batch) return;' in block

    def test_icon_glyph_follows_vehicle_type_without_rebuilding_the_marker(self):
        """機型變了（心跳晚到／VTOL 轉換）只換 <img> 的 src；不重建 divIcon（審查 2026-10-17）。"""
        block = _bare2d(_Page())._uav_js_block()
        else_part = block.split('} else {', 1)[1]
        assert 'meta.plane !== isPlane' in else_part and "img.src = 'data:image/svg+xml;utf8,'" in else_part
        assert 'L.divIcon(' not in else_part

    def test_inject_javascript_includes_the_block(self):
        import ast, inspect, pathlib
        src = pathlib.Path(inspect.getsourcefile(MapWidget)).read_text(encoding='utf-8', errors='replace')
        fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == '_inject_javascript')
        aug = [n for n in ast.walk(fn) if isinstance(n, ast.AugAssign) and isinstance(n.target, ast.Name)
               and n.target.id == 'js_code' and '_uav_js_block' in ast.unparse(n.value)]
        assert aug, '_inject_javascript 沒有 js_code += self._uav_js_block()'


class _Map2:
    def __init__(self):
        self.batches = []

    def update_uav_positions(self, batch):
        self.batches.append(list(batch))
        return len(batch)


class _Map3:
    def __init__(self):
        self.batches = []

    def update_uav_positions(self, batch):
        self.batches.append(list(batch))


class _OldMap3:
    def __init__(self):
        self.calls = []

    def update_uav_position(self, *a, **k):
        self.calls.append(k)


def _dual(mode, m2, m3, settling=False):
    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = m2, m3
    d._mode = mode
    d._map3d_settling = settling
    return d


@qt_only
class TestDual:
    def test_3d_visible_pushes_3d_every_batch_and_2d_every_nth(self):
        m2, m3 = _Map2(), _Map3()
        d = _dual(dmw._MODE_3D, m2, m3)
        n = DualMapWidget.HIDDEN_2D_EVERY_N_BATCHES
        for i in range(2 * n):
            r = d.update_uav_positions([_item(1), _item(2)])
            assert r['3d'] == 2
        assert len(m3.batches) == 2 * n
        assert len(m2.batches) == 2, '2D 藏在 3D 後面時每 N 批推一次'
        assert d._active_sysids == {1, 2}

    def test_2d_visible_pushes_2d_every_batch_and_never_3d(self):
        m2, m3 = _Map2(), _Map3()
        d = _dual('2d', m2, m3)
        for _ in range(3):
            r = d.update_uav_positions([_item(5)])
            assert r == {'3d': 0, '2d': 1}
        assert len(m2.batches) == 3 and m3.batches == []

    def test_settling_window_skips_3d(self):
        m2, m3 = _Map2(), _Map3()
        d = _dual(dmw._MODE_3D, m2, m3, settling=True)
        assert d.update_uav_positions([_item(1)])['3d'] == 0 and m3.batches == []

    def test_old_3d_api_fallback_and_single_wrapper(self):
        m2, m3 = _Map2(), _OldMap3()
        d = _dual(dmw._MODE_3D, m2, m3)
        d.update_uav_position(23.45, 120.28, 100.0, 45.0, 18.0, sysid=7, mode='AUTO', armed=True,
                              vehicle_type='VTOL', pitch_deg=1.5, roll_deg=-2.0)
        assert len(m3.calls) == 1 and m3.calls[0]['sysid'] == 7 and m3.calls[0]['pitch_deg'] == 1.5

    def test_empty_batch_is_a_noop(self):
        m2, m3 = _Map2(), _Map3()
        d = _dual(dmw._MODE_3D, m2, m3)
        assert d.update_uav_positions([]) == {'3d': 0, '2d': 0} and m2.batches == [] and m3.batches == []
