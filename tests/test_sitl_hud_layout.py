# -*- coding: utf-8 -*-
"""tests/test_sitl_hud_layout.py — SITL 控制頁 2026-10-17 重排（精簡、戰術風格）的版面鎖。

使用者定調：SITL 控制頁元素太多要精簡、整體更現代／戰術；蜂群攻擊（投放／嵌入式蜂群）先從 UI 拿掉。

鎖的是：
① 五個區段（LINK / FLIGHT / MISSION / LOG / PARAMS）存在、順序正確，PARAMS 預設摺起。
② 所有既有屬性名、信號、工具提示仍在（main_window 與其他測試靠這些）。
③ 「一鍵起飛」必須是某個 QVBoxLayout 的**直接**子項（main_window 用 indexOf+insertWidget 把
   「依時程放行」插在它正下方；放進 QHBoxLayout 或 QGridLayout 會讓那個插入靜默失敗）。
④ 攻擊那一系（投放列、嵌入式蜂群鈕）預設不在版面、也不可見；功能開關打開時才進版面**且看得見**。
⑤ 指令鈕不再用 emoji（戰術風：SVG 圖示＋語意色條）；高度 24 px；全寬填色長條**只剩**一鍵起飛。

⚠ 開關值在每個測試裡都用 autouse fixture 釘住（預設 False）：不然環境有 AEROPLAN_STRIKE_UI=1 時
  五區段順序會多一個 RELEASE 而整檔紅（審查 2026-10-17 抓到）。
"""
from __future__ import annotations

import ast
import os
import re

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

try:
    import ui.main_window  # noqa: F401  QtWebEngine 匯入順序
    from PyQt6.QtWidgets import QApplication, QVBoxLayout, QPushButton
    _APP = QApplication.instance() or QApplication([])
    from ui.widgets.sitl_hud import SITLHud
    from config import features
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')

_EMOJI = re.compile('[\U0001F300-\U0001FAFF☀-➿⬀-⯿️]')

PANELS_IN_ORDER = ['LINK', 'FLIGHT', 'MISSION', 'LOG']


@pytest.fixture(autouse=True)
def _strike_off(monkeypatch):
    """預設把開關釘在 False；要測「開」的那幾條自己再 setattr True。"""
    if _QT_OK:
        monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
    yield


def _panel_titles(hud) -> list:
    """依版面順序列出 hudPanel 的標頭代碼（LINK / FLIGHT / …）；藏起來的不算。"""
    out = []
    root = hud.layout()
    for i in range(root.count()):
        w = root.itemAt(i).widget()
        if w is None or w.objectName() != 'hudPanel' or w.isHidden():
            continue
        head = w.layout().itemAt(0).layout()
        for j in range(head.count()):
            lw = head.itemAt(j).widget()
            if lw is not None and hasattr(lw, 'text') and lw.text():
                out.append(lw.text().split('  ')[0].strip())
                break
    return out


def _layout_widgets(layout) -> list:
    """一個 layout（含巢狀子 layout）裡所有直接放進去的 widget。"""
    out = []
    for i in range(layout.count()):
        it = layout.itemAt(i)
        if it.widget() is not None:
            out.append(it.widget())
        elif it.layout() is not None:
            out.extend(_layout_widgets(it.layout()))
    return out


@qt_only
class TestSections:
    def test_five_sections_in_order_and_params_collapsed(self):
        h = SITLHud()
        titles = _panel_titles(h)
        assert titles == PANELS_IN_ORDER, titles
        assert h._params_frame.isHidden() and not h._params_visible, 'PARAMS 要預設摺起'
        h._btn_params_toggle.click()
        assert h._params_frame.isHidden() is False and h._btn_params_toggle.text().startswith('▼')
        assert h.servo_panel.isHidden(), 'SERVO OUTPUT 照舊預設摺起'

    def test_every_legacy_attribute_and_signal_still_exists(self):
        h = SITLHud()
        for name in ('preset_combo', 'btn_connect', 'btn_embedded', 'vehicle_combo', 'count_spin', 'btn_launch',
                     'servo_panel', '_btn_servo_toggle', 'lbl_cmd_target', 'btn_target_all',
                     'btn_arm', 'btn_disarm', 'btn_takeoff', 'btn_guided_takeoff', 'btn_guided_takeoff_pt',
                     'btn_guided_goto', 'btn_vtol_mc', 'btn_vtol_fw', 'btn_upload', 'btn_download',
                     'btn_load_wp', 'btn_import_wps', 'btn_write_wp', 'btn_write_imported', 'btn_save_wp',
                     'btn_log_download_all', 'chk_auto_logdl', 'btn_upload_fence', 'btn_auto_start',
                     'btn_release_safety', 'btn_release_1', 'btn_release_2', 'spin_release_ch1',
                     'spin_release_ch2', 'spin_release_pwm', 'combo_store_kind',
                     'param_name_edit', 'param_value_edit', 'param_type_combo', 'common_param_combo',
                     'log_view', 'lbl_status', 'lbl_mode', 'lbl_armed', 'lbl_gps', 'lbl_bat', 'lbl_pos'):
            assert hasattr(h, name), f'缺少屬性 {name}'
        for sig in ('cmd_arm', 'cmd_disarm', 'cmd_set_mode', 'cmd_takeoff', 'cmd_upload', 'cmd_download',
                    'cmd_load_wp', 'cmd_write_wp', 'cmd_save_wp', 'cmd_import_wps', 'cmd_write_imported',
                    'cmd_log_download', 'cmd_log_download_all', 'cmd_log_cancel', 'cmd_upload_fence',
                    'cmd_auto_start', 'cmd_param_set', 'cmd_param_get', 'cmd_params_batch',
                    'cmd_vtol_transition', 'cmd_release_store', 'cmd_guided_takeoff',
                    'cmd_guided_goto_mode', 'cmd_guided_takeoff_mode', 'connect_embedded_requested'):
            assert hasattr(h, sig), f'缺少信號 {sig}'
        assert len(h._mode_buttons) == 10 and set(h._mode_buttons) >= {'GUIDED', 'AUTO', 'QRTL'}
        assert h.btn_write_wp.isEnabled() is False and h.btn_write_imported.isEnabled() is False
        assert h.btn_release_1.isEnabled() is False and h.btn_release_2.isEnabled() is False

    def test_tooltips_that_other_tests_depend_on_are_verbatim_and_name_current_labels(self):
        h = SITLHud()
        assert '不需要連線' in h.btn_load_wp.toolTip() and '確認對話框' in h.btn_write_wp.toolTip()
        assert '讀回' in h.btn_save_wp.toolTip() and '對應 sysid' in h.btn_write_imported.toolTip()
        assert 'MAVFTP' in h.btn_log_download_all.toolTip()
        assert '投放保險' in h.btn_release_safety.toolTip()
        # 提示裡引用的按鈕名要是**現在**的標籤（審查 2026-10-17：舊名「✍️ 寫入飛控」已經不存在）
        for b in (h.btn_load_wp, h.btn_import_wps, h.btn_write_imported, h.btn_save_wp):
            assert '✍️' not in b.toolTip() and '📥' not in b.toolTip(), b.toolTip()
        assert '「寫入飛控」' in h.btn_load_wp.toolTip() and '「寫入每架匯入」' in h.btn_import_wps.toolTip()
        assert '「讀回任務」' in h.btn_save_wp.toolTip()

    def test_auto_start_is_a_direct_child_of_a_vbox_so_release_widget_can_be_inserted_after_it(self):
        h = SITLHud()
        lay = h.btn_auto_start.parentWidget().layout()
        assert isinstance(lay, QVBoxLayout), type(lay)
        idx = lay.indexOf(h.btn_auto_start)
        assert idx >= 0, 'btn_auto_start 不是該 QVBoxLayout 的直接子項 ⇒ 「依時程放行」插不進去'
        probe = QPushButton('probe')
        lay.insertWidget(idx + 1, probe)
        assert lay.itemAt(idx + 1).widget() is probe

    def test_mode_buttons_emit_the_mode_name(self):
        h = SITLHud()
        got = []
        h.cmd_set_mode.connect(got.append)
        h._mode_buttons['QLOITER'].click()
        h._mode_buttons['AUTO'].click()
        assert got == ['QLOITER', 'AUTO']
        vt = []
        h.cmd_vtol_transition.connect(vt.append)
        h.btn_vtol_mc.click(); h.btn_vtol_fw.click()
        assert vt == [3, 4]
        alts = []
        h.cmd_takeoff.connect(alts.append); h.cmd_guided_takeoff.connect(alts.append)
        h.btn_takeoff.click(); h.btn_guided_takeoff.click()
        assert alts == [30.0, 60.0]


@qt_only
class TestTacticalStyle:
    def test_command_buttons_have_no_emoji_and_are_24px(self):
        h = SITLHud()
        for b in (h.btn_arm, h.btn_disarm, h.btn_takeoff, h.btn_guided_takeoff, h.btn_upload, h.btn_download,
                  h.btn_load_wp, h.btn_import_wps, h.btn_write_wp, h.btn_write_imported, h.btn_save_wp,
                  h.btn_log_download_all, h.btn_upload_fence, h.btn_auto_start, h.btn_vtol_mc, h.btn_vtol_fw,
                  *h._mode_buttons.values()):
            assert not _EMOJI.search(b.text()), f'{b.text()!r} 還有 emoji'
            assert b.height() == 24 or b.maximumHeight() == 24, (b.text(), b.height())
        assert not h.btn_arm.icon().isNull() and not h.btn_auto_start.icon().isNull(), '要有 SVG 圖示'

    def test_only_the_hero_button_is_filled(self):
        """全寬飽和色長條**只剩**「一鍵起飛」：掃全部 24 px chip，沒有左側色條的（＝填色）只能是它。"""
        h = SITLHud()
        chips = [b for b in h.findChildren(QPushButton) if b.maximumHeight() == 24]
        assert len(chips) >= 20, len(chips)
        filled = [b for b in chips if 'border-left:3px solid' not in b.styleSheet()]
        assert filled == [h.btn_auto_start], [b.text() for b in filled]
        assert f'background:{h._tone_color("warn")}' in h.btn_auto_start.styleSheet()

    def test_checkable_chips_still_toggle_their_modes(self):
        h = SITLHud()
        got = []
        h.cmd_guided_goto_mode.connect(got.append)
        h.btn_guided_goto.click()
        assert got == [True] and h.btn_guided_goto.isChecked()


@qt_only
class TestStrikeGate:
    def test_release_row_and_embedded_button_are_off_the_layout_by_default(self):
        h = SITLHud()
        assert h._release_group.isHidden() and h.btn_embedded.isHidden()
        root = h.layout()
        assert all(root.itemAt(i).widget() is not h._release_group for i in range(root.count()))
        assert 'RELEASE' not in _panel_titles(h)
        link_frame = root.itemAt(0).widget()
        assert h.btn_embedded not in _layout_widgets(link_frame.layout()), '關著還在連線列裡'
        # 投放保險的既有行為照舊（tests/test_store_release.py 直接操作這些 widget）
        h.btn_release_safety.setChecked(True)
        assert h.btn_release_1.isEnabled()

    def test_flag_on_puts_them_back_and_visible(self, monkeypatch):
        monkeypatch.setattr(features, 'STRIKE_ENABLED', True)
        h = SITLHud()
        root = h.layout()
        assert any(root.itemAt(i).widget() is h._release_group for i in range(root.count()))
        assert not h._release_group.isHidden()
        titles = _panel_titles(h)
        assert titles.index('RELEASE') == titles.index('FLIGHT') + 1
        link_frame = root.itemAt(0).widget()
        assert h.btn_embedded.parentWidget() is link_frame
        assert not h.btn_embedded.isHidden(), '開關開了鈕卻是藏著的'
        assert h.btn_embedded in _layout_widgets(link_frame.layout()), '沒真的放進連線列'

    def test_flag_is_read_live_not_captured_at_import(self, monkeypatch):
        monkeypatch.setattr(features, 'STRIKE_ENABLED', True)
        assert features.strike_enabled() is True
        monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
        assert features.strike_enabled() is False

    def test_feature_module_only_reads_the_environment(self):
        """結構鎖（AST）：config/features.py 只 import os／__future__，沒有任何 get_settings／open／json 呼叫。
        以前用子字串鎖 'settings.json'，結果一個真的去讀 settings.json 的實作反而更容易通過（審查 2026-10-17）。"""
        tree = ast.parse(open(features.__file__, encoding='utf-8').read())
        mods = set()
        for n in ast.walk(tree):
            if isinstance(n, ast.Import):
                mods |= {a.name for a in n.names}
            elif isinstance(n, ast.ImportFrom):
                mods.add(n.module or '')
        assert mods <= {'os', '__future__'}, mods
        calls = {ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)}
        assert not any(c in calls or c in x for x in calls for c in ('get_settings', 'open', 'json')), calls
        assert any('AEROPLAN_STRIKE_UI' == n.value for n in ast.walk(tree) if isinstance(n, ast.Constant))
