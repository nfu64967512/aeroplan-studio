# -*- coding: utf-8 -*-
"""tests/test_sitl_hud_modes.py — SITL 頁「群控／單機」兩種模式（2026-10-22）。

使用者：「servo output 跟 message 應該是我選哪一台飛機就列出哪一台的；SITL 操作面板應該要有兩個模式，
一種是群控，一種是單獨看某一台飛機的詳細資訊。」

鎖的是（值層級，真 SITLHud、離屏）：
  H1 預設群控、單機區藏著、區段順序不變；H2 切換只發一次信號、chip 互斥、SERVO 只自動展開一次；
  H3 群控訊息每行標 UAVn、依嚴重度上色；H4 單機只列選定那台（＋本機訊息），換台立刻列出那台的歷史；
  H5 緩衝上限、PARAM 洪水擠不掉飛控訊息；H6/H7 SERVO 依 sysid；H8/H9 指令對象；H10 看不到的元件零更新；
  H11 切換成本有上限；H12 斷線；H13 HTML 跳脫；H14 第一台自動選；H15 別台嚴重訊息標未讀。
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

try:
    import ui.main_window  # noqa: F401  QtWebEngine 匯入順序
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    from ui.widgets.sitl_hud import SITLHud
    from ui.widgets.sitl_messages import SitlMessageLog
    from ui.resources.tactical_theme import TacticalColors as TC
    from mission.sitl_link import TelemetryFrame, servo_function_name
    from config import features
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')


@pytest.fixture(autouse=True)
def _strike_off(monkeypatch):
    if _QT_OK:
        monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
    yield


def _f(sid, mode='AUTO', **kw):
    kw.setdefault('lat', 23.4 + sid * 0.001)
    kw.setdefault('lon', 120.2)
    kw.setdefault('servo_pwm', [1000 + sid * 100 + i for i in range(16)])
    return TelemetryFrame(sysid=sid, mode=mode, vehicle_type='PLANE', system_status='ACTIVE', **kw)


def _hud(*sysids):
    h = SITLHud()
    for s in sysids:
        h.on_telemetry(_f(s))
    return h


def _panel_titles(hud) -> list:
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
    out = []
    for i in range(layout.count()):
        it = layout.itemAt(i)
        if it.widget() is not None:
            out.append(it.widget())
        elif it.layout() is not None:
            out.extend(_layout_widgets(it.layout()))
    return out


class _Rec:
    """每個實例自己的記錄器（不改類別）：包住方法、記下參數、照樣呼叫原方法。"""

    def __init__(self):
        self.calls = {}

    def wrap(self, obj, name, key, pick=lambda a: a):
        orig = getattr(obj, name)
        self.calls.setdefault(key, [])

        def w(*a, **k):
            self.calls[key].append(pick(a))
            return orig(*a, **k)
        setattr(obj, name, w)

    def n(self, key):
        return len(self.calls.get(key, []))

    def clear(self):
        for v in self.calls.values():
            v.clear()


# ══════════════════════════════════════════════════════════════════════
@qt_only
def test_H1_default_is_swarm_and_single_widgets_are_hidden():
    h = SITLHud()
    assert h.view_mode() == 'swarm' and h.btn_mode_swarm.isChecked() and not h.btn_mode_single.isChecked()
    assert not h._cards_scroll.isHidden() and not h.log_view.isHidden()
    for w in (h._single_frame, h._single_msg_frame, h._btn_servo_toggle, h.servo_panel):
        assert w.isHidden(), w
    assert _panel_titles(h) == ['LINK', 'FLIGHT', 'MISSION', 'LOG']
    link = h.layout().itemAt(0).widget()
    ws = _layout_widgets(link.layout())
    assert h.btn_mode_swarm in ws and h.btn_mode_single in ws, '模式 chip 要在 LINK 區段裡'
    assert h.get_target_sysid() is None
    assert '群控' in h.btn_mode_swarm.text() and '單機' in h.btn_mode_single.text()


@qt_only
def test_H2_mode_switch_flips_visibility_emits_once_and_chips_are_exclusive():
    h = SITLHud()
    got = []
    h.view_mode_changed.connect(got.append)
    assert h.set_view_mode('single') is True
    assert h.set_view_mode('single') is False and h.set_view_mode('bogus') is False
    assert got == ['single']
    assert h._cards_scroll.isHidden() and h.log_view.isHidden()
    assert not h._single_frame.isHidden() and not h._single_msg_frame.isHidden()
    assert not h._btn_servo_toggle.isHidden() and not h.servo_panel.isHidden(), '第一次進單機自動展開 SERVO'
    assert h.btn_mode_single.isChecked() and not h.btn_mode_swarm.isChecked()
    # 單機：FLIGHT 緊接 UAV（HUD）之後、MSG 在 FLIGHT 下面（DISARM／RTL 不被擠到畫面外；2026-10-22 審查）；
    # 沒選定飛機 ⇒ DETAIL 不顯示
    assert _panel_titles(h) == ['LINK', 'UAV', 'FLIGHT', 'MSG', 'MISSION', 'LOG']
    h.btn_mode_swarm.click()
    assert got == ['single', 'swarm'] and h.view_mode() == 'swarm'
    assert h.btn_mode_swarm.isChecked() and not h.btn_mode_single.isChecked()
    h.btn_mode_swarm.click()                                  # 點已勾的那顆：不切、不發
    assert got == ['single', 'swarm'] and h.btn_mode_swarm.isChecked()
    h.btn_mode_single.click()
    assert got[-1] == 'single'
    h._toggle_servo_panel()                                   # 使用者收起
    h.set_view_mode('swarm'); h.set_view_mode('single')
    assert h.servo_panel.isHidden() and not h._servo_visible, '自動展開只做一次，不能違背使用者收起'


@qt_only
def test_H3_swarm_log_tags_each_line_with_its_uav_and_colours_by_severity():
    h = SITLHud()
    h.on_status_text_sysid(1, 6, 'alpha', 'fc')
    h.on_status_text_sysid(2, 3, 'bravo', 'fc')
    lines = h.log_view.toPlainText().splitlines()
    assert any('UAV1' in ln and 'alpha' in ln and '[FC]' in ln for ln in lines), lines
    assert any('UAV2' in ln and 'bravo' in ln for ln in lines), lines
    ents = [e for e in h._msgs.merged() if e.kind == 'fc']
    assert [(e.sysid, e.severity, e.text) for e in ents] == [(1, 6, 'alpha'), (2, 3, 'bravo')]
    assert TC.HOSTILE in h._msg_html(ents[1], True) and TC.HOSTILE not in h._msg_html(ents[0], True)
    assert TC.NEUTRAL in h._msg_html(ents[0], True)


@qt_only
def test_H4_single_view_lists_only_the_selected_uav_and_switching_shows_its_history():
    h = _hud(1, 2)
    h.on_status_text_sysid(1, 6, 'u1-a', 'fc')                # 群控時收到的也要算進歷史
    h.on_status_text_sysid(2, 6, 'u2-a', 'fc')
    h.lbl_status.setText('LOCAL-LINE')
    h.set_view_mode('single')
    assert h.single_sysid() == 1
    h.on_status_text_sysid(1, 6, 'u1-b', 'gcs')
    h.on_status_text_sysid(2, 6, 'u2-b', 'fc')
    t = h.single_log_view.toPlainText()
    assert 'u1-a' in t and 'u1-b' in t and 'LOCAL-LINE' in t
    assert 'u2-' not in t, '單機不能列別台的訊息'
    assert '[UAV' not in t, '單機區不標 UAVn（整區就是那一台）'
    assert h.set_single_sysid(2) is True
    t = h.single_log_view.toPlainText()
    assert 'u1-' not in t and 'LOCAL-LINE' in t
    assert 0 <= t.index('u2-a') < t.index('u2-b'), '換台立刻依序列出那台的歷史'


@qt_only
def test_H5_buffers_are_bounded_and_a_param_flood_cannot_evict_fc_history():
    log = SitlMessageLog()
    for i in range(1000):
        log.add(1, 6, f'fc{i}', 'fc')
    c = log.counts()
    assert c['per'][1] == 300 and c['merged'] == 500
    m = log.merged()
    assert m[-1].text == 'fc999' and all(a.seq < b.seq for a, b in zip(m, m[1:]))
    u = log.for_uav(1)
    assert len(u) == 300 and u[0].text == 'fc700' and u[-1].text == 'fc999'
    for i in range(1000):
        log.add(1, 6, f'PARAM P{i} = 1', 'param')
    u2 = log.for_uav(1)
    assert [e.text for e in u2 if e.kind == 'fc'] == [e.text for e in u], 'PARAM 洪水把飛控訊息擠掉了'
    assert sum(e.kind == 'param' for e in u2) == 50 and len(log.merged()) == 550
    assert log.add(0, 6, 'x', 'fc').sysid is None and log.add(-1, 6, 'y', 'fc').sysid is None
    # HUD 端：即時只列前 PARAM_LIVE_PER_S 則，其餘補一行摘要
    h = SITLHud()
    for i in range(100):
        h.on_status_text_sysid(1, 6, f'PARAM P{i} = 1', 'param')
    t = h.log_view.toPlainText()
    assert t.count('PARAM P') == SITLHud.PARAM_LIVE_PER_S, t.count('PARAM P')
    h._flush_param_summaries()
    assert '[UAV1] （參數回報 90 筆未逐筆列出）' in h.log_view.toPlainText()


@qt_only
def test_H6_servo_output_shows_the_selected_uavs_params_and_pwm():
    h = _hud(1, 2, 3)
    h.on_servo_param_sysid(1, 5, 'FUNCTION', 33.0)
    h.on_servo_param_sysid(2, 5, 'FUNCTION', 4.0)
    h.on_servo_param_sysid(2, 5, 'REVERSED', 1.0)
    h.set_view_mode('single')
    row = h.servo_panel._rows[4]
    assert h.single_sysid() == 1 and row.lbl_func.text() == servo_function_name(33)
    assert row.bar._pwm == 1000 + 100 + 4
    h.set_single_sysid(2)
    assert row.lbl_func.text() == servo_function_name(4) and row.lbl_rev.text() == '☑'
    assert row.bar._pwm == 1000 + 200 + 4
    title = h.servo_panel.lbl_title.text()
    assert 'UAV2' in title and '2/80' in title, title
    assert h.servo_panel._rows[0].lbl_func.text() == '—', '沒讀回的通道不能顯示預設值'
    assert 'UAV2' in h._btn_servo_toggle.text()
    h.set_single_sysid(3)
    assert row.lbl_func.text() == '—' and row.lbl_rev.text() == '☐', '上一台的值不能留著'
    assert 'UAV3' in h.servo_panel.lbl_title.text() and '0/80' in h.servo_panel.lbl_title.text()
    h.on_servo_param_sysid(1, 5, 'FUNCTION', 70.0)            # 不是正在看的那台：只進快取
    assert row.lbl_func.text() == '—'
    h.on_servo_param_sysid(3, 5, 'FUNCTION', 19.0)            # 正在看的那台：即時寫面板
    assert row.lbl_func.text() == servo_function_name(19) and '1/80' in h.servo_panel.lbl_title.text()
    h.set_single_sysid(1)
    assert row.lbl_func.text() == servo_function_name(70)


@qt_only
def test_H6b_switching_shows_the_new_uavs_slow_readouts_immediately():
    """大 HUD 的精讀數值（GS/VS/BAT/GPS）1 Hz 才刷新：換台時要整幀寫完再刷新，不能留上一台的電量／衛星數。
    （2026-10-22 截圖查到：換到 UAV2 後 BAT 還是 UAV1 的 24.6V 78%、17sat。）"""
    h = SITLHud()
    h.on_telemetry(_f(1, battery_v=24.6, battery_pct=78, gps_sats=17, ground_speed=19.6))
    h.on_telemetry(_f(2, battery_v=23.9, battery_pct=64, gps_sats=15, ground_speed=17.2))
    h.set_view_mode('single')
    hud = h.single_hud
    assert (hud._d_bat_v, hud._d_bat_pct, hud._d_gps_sats) == (24.6, 78, 17)
    h.set_single_sysid(2)
    assert (hud._d_bat_v, hud._d_bat_pct, hud._d_gps_sats, hud._d_gs) == (23.9, 64, 15, 17.2)
    assert hud._d_mode == 'AUTO'
    h.set_single_sysid(1)
    assert (hud._d_bat_v, hud._d_bat_pct, hud._d_gps_sats, hud._d_gs) == (24.6, 78, 17, 19.6)


@qt_only
def test_H7_pwm_from_other_uavs_never_reaches_the_servo_panel():
    h = _hud(1)
    h.set_view_mode('single')
    seen = []
    h.servo_panel.on_telemetry = lambda fr: seen.append(fr.sysid)
    for s in (1, 2, 3, 1, 2):
        h.on_telemetry(_f(s))
    assert seen == [1, 1]
    h._toggle_servo_panel()                                   # 收起 ⇒ 不更新
    seen.clear()
    h.on_telemetry(_f(1))
    assert seen == []


@qt_only
def test_H8_command_target_follows_the_mode():
    h = _hud(1, 2, 3)
    h._uav_cards[2].selected.emit(2)                          # 群控點機卡 2
    assert h.get_target_sysid() == 2 and h.single_sysid() == 2
    h._uav_cards[2].selected.emit(2)                          # 再點一次 ⇒ 全部
    assert h.get_target_sysid() is None
    h._uav_cards[2].selected.emit(2)
    assert h.get_target_sysid() == 2
    h.set_view_mode('single')
    assert h.get_target_sysid() == 2 and h.lbl_cmd_target.text() == 'UAV2（單機模式）'
    assert not h.btn_target_all.isEnabled() and '群控' in h.btn_target_all.toolTip()
    h._single_chips[3].click()
    assert h.get_target_sysid() == 3 and h.lbl_cmd_target.text() == 'UAV3（單機模式）'
    h.set_view_mode('swarm')
    assert h.get_target_sysid() == 2, '回群控：群控的指令對象原封不動'
    assert h.btn_target_all.isEnabled() and h.lbl_cmd_target.text() == 'UAV2（單獨）'
    empty = SITLHud()
    empty.set_view_mode('single')
    assert empty.get_target_sysid() == SITLHud.NO_TARGET == -1
    assert empty.lbl_cmd_target.text() == '未選定（單機模式）' and not empty.btn_target_all.isEnabled()


@qt_only
def test_H9_asking_for_all_while_single_switches_back_to_swarm_visibly():
    h = _hud(1, 2)
    h.set_view_mode('single')
    h._set_target(None)                                       # main_window「改成全部 UAV」
    assert h.view_mode() == 'swarm' and h.get_target_sysid() is None
    assert h.btn_mode_swarm.isChecked() and not h._cards_scroll.isHidden()


@qt_only
def test_H10_hidden_widgets_get_zero_updates():
    h = _hud(1, 2)
    h.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    h.resize(440, 1400)
    h.show()
    _APP.processEvents()
    r = _Rec()
    r.wrap(h.single_hud, 'update_from_frame', 'single_hud', lambda a: a[0].sysid)
    r.wrap(h.servo_panel, 'on_telemetry', 'servo_tel', lambda a: a[0].sysid)
    r.wrap(h.servo_panel, 'on_servo_param', 'servo_param')
    r.wrap(h.single_log_view, 'appendHtml', 'single_log')
    r.wrap(h.log_view, 'appendHtml', 'log')
    r.wrap(h.lbl_mode, 'setText', 'legacy', lambda a: a[0])
    for s, card in h._uav_cards.items():
        r.wrap(card, 'update_from_frame', f'card{s}')
    # ── 群控 ──
    for i in range(20):
        h.on_telemetry(_f(1, 'AUTO'))
        h.on_telemetry(_f(2, 'RTL'))
    for i in range(5):
        h.on_status_text_sysid(1, 6, f'm1-{i}', 'fc')
        h.on_status_text_sysid(2, 6, f'm2-{i}', 'fc')
    h.on_servo_param_sysid(1, 1, 'FUNCTION', 4.0)
    for k in ('single_hud', 'servo_tel', 'servo_param', 'single_log', 'legacy'):
        assert r.n(k) == 0, f'群控時 {k} 被更新了 {r.n(k)} 次'
    assert r.n('card1') == 20 and r.n('card2') == 20 and r.n('log') == 10
    # ── 單機（看 UAV1）──
    h.set_view_mode('single')
    _APP.processEvents()
    assert h.single_sysid() == 1 and h._legacy_hud_frame.isVisible()
    r.clear()
    for i in range(20):
        h.on_telemetry(_f(1, 'AUTO'))
        h.on_telemetry(_f(2, 'RTL'))
    for i in range(5):
        h.on_status_text_sysid(2, 6, f'm2b-{i}', 'fc')
    assert r.n('card1') == 0 and r.n('card2') == 0, '單機時機卡不更新'
    assert r.n('log') == 0, '單機時合併訊息區不 append'
    assert r.calls['single_hud'] == [1] * 20 and r.calls['servo_tel'] == [1] * 20
    assert r.n('single_log') == 0, '別台的訊息不進單機區'
    assert r.calls['legacy'] == ['AUTO'] * 20 and h.lbl_mode.text() == 'AUTO', 'DETAIL 只吃選定那台'
    # ── 可觸發：把模式硬改錯（群控畫面、狀態說單機）⇒ 計數馬上不是 0 ──
    h.set_view_mode('swarm')
    r.clear()
    h.__dict__['_mode'] = 'single'
    h.on_telemetry(_f(1))
    assert r.n('single_hud') == 1
    h.__dict__['_mode'] = 'swarm'
    h.hide()


@qt_only
def test_H11_switch_cost_is_bounded():
    h = _hud(1, 2)
    for i in range(500):
        h.on_status_text_sysid(1, 6, f'a{i}', 'fc')
        h.on_status_text_sysid(2, 6, f'b{i}', 'fc')
    h.set_view_mode('single')
    r = _Rec()
    r.wrap(h.single_log_view, 'appendHtml', 'single_log')
    r.wrap(h.log_view, 'appendHtml', 'log')
    h.set_single_sysid(2)
    assert 0 < r.n('single_log') <= SITLHud.SINGLE_LOG_MAX == 450
    assert h.single_log_view.toPlainText().splitlines()[-1].endswith('b499')
    h.set_view_mode('swarm')
    assert 0 < r.n('log') <= 450


@qt_only
def test_H12_removal_in_swarm_auto_adopts_but_in_single_holds_until_the_user_picks():
    h = _hud(1, 2)
    assert h.single_sysid() == 1
    h.remove_card(1)
    assert h.single_sysid() is None and h._single_hold is False
    h.on_telemetry(_f(2))
    assert h.single_sysid() == 2, '群控：下一幀自動接手'

    s = _hud(1, 2, 3)
    s.on_servo_param_sysid(1, 5, 'FUNCTION', 33.0)
    s.set_view_mode('single')
    got = []
    s.single_uav_changed.connect(got.append)
    s.remove_card(1)
    assert s.single_sysid() is None and got == [SITLHud.NO_TARGET]
    assert s.get_target_sysid() == -1 and s._single_hold is True
    assert 'UAV1 已斷線' in s.lbl_single_empty.text() and not s.lbl_single_empty.isHidden()
    assert s.single_hud.isHidden()
    s.on_telemetry(_f(3))
    assert s.single_sysid() is None and got == [-1], '單機：選定那台斷線不能自動改看別台'
    assert 1 not in s._servo_cache and 1 not in s._single_chips and 1 not in s._last_frame
    assert 'UAV1' not in s.servo_panel.lbl_title.text()
    s._single_chips[3].click()
    assert s.single_sysid() == 3 and s._single_hold is False and got == [-1, 3]


@qt_only
def test_H13_message_text_is_html_escaped():
    h = SITLHud()
    h.on_status_text_sysid(1, 6, '<b>x</b>', 'fc')
    h.lbl_status.setText('<i>y</i> & z')
    t = h.log_view.toPlainText()
    assert '<b>x</b>' in t and '<i>y</i> & z' in t


@qt_only
def test_H14_first_telemetry_auto_adopts_exactly_once():
    h = SITLHud()
    got = []
    h.single_uav_changed.connect(got.append)
    h.on_telemetry(_f(3))
    h.on_telemetry(_f(1))
    h.on_telemetry(_f(3))
    assert h.single_sysid() == 3 and got == [3]
    assert h.get_target_sysid() is None, '群控的指令對象不受影響'


@qt_only
def test_H15_critical_message_from_another_uav_marks_its_chip_unread():
    h = _hud(1, 2)
    h.set_view_mode('single')
    chip = h._single_chips[2]
    h.on_status_text_sysid(2, 4, 'warn-only', 'fc')
    assert chip.text() == 'UAV2' and chip.property('chipTone') == 'info'
    h.on_status_text_sysid(2, 2, 'boom', 'fc')
    assert chip.text() == 'UAV2 ●1' and chip.property('chipTone') == 'stop'
    assert f'border-left:3px solid {TC.HOSTILE}' in chip.styleSheet()
    h.on_status_text_sysid(1, 2, 'own-crit', 'fc')            # 正在看的那台：不標
    assert h._single_chips[1].text() == 'UAV1'
    chip.click()
    assert h.single_sysid() == 2 and chip.text() == 'UAV2' and chip.property('chipTone') == 'info'
    assert 'boom' in h.single_log_view.toPlainText()


@qt_only
def test_link_freshness_line_turns_stale():
    h = _hud(1)
    h.set_view_mode('single')
    assert h.lbl_single_link.text().startswith('UAV1 · 遙測 <1 s 前 · ACTIVE · PLANE')
    h._last_frame_t[1] -= 12.0
    h._refresh_single_link_line()
    assert 'STALE' in h.lbl_single_link.text() and TC.HOSTILE in h.lbl_single_link.styleSheet()
    assert h._single_age_timer.isActive()
    h.set_view_mode('swarm')
    assert not h._single_age_timer.isActive()
