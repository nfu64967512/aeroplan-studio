"""SITL 面板「一號彈 / 二號彈」投放鈕的接線測試。

驗的是**行為**不是「屬性存在」：
  * 保險沒開時兩顆鈕按不下去（這是唯一擋住誤觸的機制）
  * 按下去真的送出 MAV_CMD_DO_SET_SERVO(183)，通道與 PWM 對得上面板設定
  * 送出後保險自動復位（一次開保險只放一發）
  * 彈種只影響地圖模擬，不影響送出的指令

⚠️ Qt 部分的兩個環境陷阱（都實際踩到）：
  1. `QtWebEngineWidgets` 必須在 `QApplication` 之前 import，否則
     `import ui.widgets` 會炸 "must be imported ... before a QCoreApplication"。
     `ui/__init__.py` 會拉進整個 MainWindow → map_widget → QWebEngineView，
     所以只要碰到 ui 套件就會踩到。
  2. 純文字檢查（例如「main.html 有沒有那個函式」）不要用 import 去做，
     讀檔就好 —— 免得為了一條字串斷言把整個 Qt 拉起來。
"""
import queue
from pathlib import Path

import pytest

from mission import ballistics

REPO = Path(__file__).resolve().parents[1]


# ── SITLLink：投放指令真的變成 MAV_CMD_DO_SET_SERVO ─────────────────
# 用「未繫結呼叫 + 假 self」而不是建真的 SITLLink：後者是 QThread，
# __init__ 會起執行緒、pyqtSignal 也需要 super().__init__ 跑過。
class _FakeMavChannel:
    def __init__(self):
        self.sent = []

    def command_long_send(self, target_system, target_component, command,
                          confirmation, p1, p2, p3, p4, p5, p6, p7):
        self.sent.append(dict(cmd=command, p1=p1, p2=p2))


class _FakeSignal:
    def __init__(self):
        self.msgs = []

    def emit(self, *a):
        self.msgs.append(a)


class _FakeSelf:
    def __init__(self):
        self._mav = type('M', (), {})()
        self._mav.mav = _FakeMavChannel()
        self._mav.target_system = 1
        self._mav.target_component = 1
        self.status_text = _FakeSignal()
        self._cmd_queue = queue.Queue()


def test_set_servo_emits_do_set_servo_183():
    from mission.sitl_link import SITLLink
    me = _FakeSelf()
    SITLLink._send_set_servo(me, 9, 2000)
    sent = me._mav.mav.sent
    assert len(sent) == 1
    assert sent[0]['cmd'] == 183, '必須是 MAV_CMD_DO_SET_SERVO'
    assert sent[0]['p1'] == 9.0, 'param1 = 伺服通道'
    assert sent[0]['p2'] == 2000.0, 'param2 = PWM'
    assert me.status_text.msgs, '要留下可稽核的狀態訊息'


def test_set_servo_queues_channel_and_pwm_as_ints():
    from mission.sitl_link import SITLLink
    me = _FakeSelf()
    SITLLink.set_servo(me, '10', '1000')
    cmd, arg = me._cmd_queue.get_nowait()
    assert cmd == 'set_servo'
    assert arg == (10, 1000)


def test_dispatch_routes_set_servo():
    """dispatch 少一條分支的話，指令會被安靜吞掉（佇列進得去、什麼都沒送）。"""
    src = (REPO / 'mission' / 'sitl_link.py').read_text(encoding='utf-8')
    assert "elif cmd == 'set_servo':" in src
    assert '_send_set_servo(*arg)' in src


def test_telemetry_frame_carries_ned_velocity():
    """彈道要的是速度**向量**；只有地速+航向在有風時方向會錯。"""
    from mission.sitl_link import TelemetryFrame
    f = TelemetryFrame()
    for attr in ('vel_n', 'vel_e', 'vel_d'):
        assert hasattr(f, attr), f'TelemetryFrame 少了 {attr}'
    src = (REPO / 'mission' / 'sitl_link.py').read_text(encoding='utf-8')
    assert 'f.vel_n, f.vel_e, f.vel_d' in src, 'GLOBAL_POSITION_INT 要真的填進去'


# ── 地圖端：純文字檢查，不 import Qt ────────────────────────────────
def test_cesium_widget_exposes_drop_store():
    src = (REPO / 'ui' / 'widgets' / 'cesium_map_widget.py').read_text(encoding='utf-8')
    assert 'def drop_store' in src
    assert 'dropStore(' in src, 'Python 端要呼叫 JS 的 dropStore'
    assert 'def clear_stores' in src


def test_main_html_defines_the_js_side():
    html = (REPO / 'ui' / 'widgets' / 'cesium_templates' / 'main.html').read_text(
        encoding='utf-8')
    for fn in ('function dropStore', 'function _impactMark',
               'function clearStores', 'function clearImpacts'):
        assert fn in html, f'main.html 少了 {fn}'
    # 落地改成單純的紅色圓形標示影響範圍，不再用爆炸火球
    assert '_impactMark(' in html
    # 但 explodeAt 本身必須留著 —— 蜂群打擊視覺化與 check_bullseye_hit 在用
    assert 'function explodeAt' in html, 'explodeAt 是別的功能在用的，不可移除'
    # dropStore 不可以再呼叫 explodeAt（那是 140 m 火球，尺度不對）。
    # 只取 dropStore 這一個函式，而且濾掉註解 —— 註解裡本來就會提到 explodeAt
    # 解釋「為什麼不用它」，拿整段字串比對會誤判。
    i0 = html.index('function dropStore')
    i1 = html.index(chr(10) + 'function ', i0 + 10)
    body = chr(10).join(
        ln for ln in html[i0:i1].splitlines()
        if not ln.strip().startswith('//'))
    assert 'explodeAt' not in body, 'dropStore 不該再叫 explodeAt'
    assert '_impactMark(' in body, 'dropStore 落地時要畫落點標記'


def test_blast_radius_is_a_simple_small_circle():
    """影響範圍是示意用的小圓（使用者要 5~10 m），不是爆炸半徑。"""
    for kind in ('dumb', 'smoke'):
        r = ballistics.STORE_KINDS[kind]['blast_radius_m']
        assert 5.0 <= r <= 10.0, (kind, r)
    d = ballistics.simulate(25.0, 121.0, 300.0, 30, 0, 0,
                            ground_msl=0.0, kind='smoke').as_dict()
    assert d['blast_radius_m'] == ballistics.STORE_KINDS['smoke']['blast_radius_m']


def test_main_window_wires_the_signal():
    src = (REPO / 'ui' / 'main_window.py').read_text(encoding='utf-8')
    assert 'cmd_release_store.connect(self.on_release_store)' in src
    assert 'def on_release_store' in src
    assert 'link.set_servo(ch, pwm)' in src


# ── HUD：保險與按鈕（需要 Qt）────────────────────────────────────────
@pytest.fixture(scope="module")
def qapp():
    # ⚠️ 順序不可調換：WebEngine 一定要在 QApplication 之前 import
    try:
        import PyQt6.QtWebEngineWidgets  # noqa: F401
    except Exception:
        pass
    from PyQt6.QtWidgets import QApplication
    yield QApplication.instance() or QApplication([])


@pytest.fixture
def hud(qapp):
    from ui.widgets.sitl_hud import SITLHud
    return SITLHud()


def test_release_buttons_exist_and_start_disabled(hud):
    assert hud.btn_release_1.isEnabled() is False
    assert hud.btn_release_2.isEnabled() is False
    assert hud.btn_release_safety.isChecked() is False


def test_safety_gates_the_release_buttons(hud):
    hud.btn_release_safety.setChecked(True)
    assert hud.btn_release_1.isEnabled() and hud.btn_release_2.isEnabled()
    hud.btn_release_safety.setChecked(False)
    assert not hud.btn_release_1.isEnabled()
    assert not hud.btn_release_2.isEnabled()


def test_clicking_release_emits_index_and_kind_then_resafes(hud):
    got = []
    hud.cmd_release_store.connect(lambda i, k: got.append((i, k)))
    hud.btn_release_safety.setChecked(True)
    hud.btn_release_1.click()
    assert got == [(1, 'dumb')]
    # 一次開保險只放一發
    assert hud.btn_release_safety.isChecked() is False
    assert hud.btn_release_1.isEnabled() is False

    hud.combo_store_kind.setCurrentIndex(1)          # 煙霧彈
    hud.btn_release_safety.setChecked(True)
    hud.btn_release_2.click()
    assert got[-1] == (2, 'smoke')


def test_disabled_button_cannot_fire(hud):
    """保險沒開時就算程式去 click() 也不能送出 —— 這是擋誤觸的唯一機制。"""
    got = []
    hud.cmd_release_store.connect(lambda i, k: got.append((i, k)))
    assert hud.btn_release_safety.isChecked() is False
    hud.btn_release_1.click()
    hud.btn_release_2.click()
    assert got == []


def test_default_channels_match_swarm_strike_convention(hud):
    """預設 ch9/ch10 與 swarm_strike 的左右掛架一致，換場景時少一個對照表。"""
    assert hud.release_channel(1) == 9
    assert hud.release_channel(2) == 10
    assert hud.release_pwm() == 2000


def test_channel_and_pwm_are_configurable(hud):
    hud.spin_release_ch1.setValue(5)
    hud.spin_release_pwm.setValue(1800)
    assert hud.release_channel(1) == 5
    assert hud.release_pwm() == 1800


def test_store_kind_only_changes_ballistics_not_the_command(hud):
    """彈種是地圖模擬用的，不可以影響送出的通道/PWM。"""
    before = (hud.release_channel(1), hud.release_channel(2), hud.release_pwm())
    hud.combo_store_kind.setCurrentIndex(1)
    after = (hud.release_channel(1), hud.release_channel(2), hud.release_pwm())
    assert before == after
    # 但兩種彈的彈道必須真的不同，否則這個選項沒有意義
    a = ballistics.simulate(25.0, 121.0, 400.0, 50, 0, 0, ground_msl=0, kind='dumb')
    b = ballistics.simulate(25.0, 121.0, 400.0, 50, 0, 0, ground_msl=0, kind='smoke')
    assert b.tof > a.tof
    assert a.range_m > b.range_m, '傻瓜彈的前拋距離應該比煙霧彈遠'


# ── 讀回確認：動畫只在 PWM 真的動了之後才播 ─────────────────────────
# 用「未繫結呼叫 + 假 self」直接測 MainWindow 的三個方法，不建整個視窗。
class _FakeStatusBar:
    def __init__(self):
        self.msgs = []

    def showMessage(self, text, msec=0):
        self.msgs.append(text)


class _FakeCesium:
    def __init__(self):
        self.played = []

    def drop_store(self, traj, label):
        self.played.append((traj, label))


class _FakeFrame:
    def __init__(self, sysid=1, servo=None):
        self.sysid = sysid
        self.servo_pwm = servo if servo is not None else [0] * 16
        self.lat, self.lon = 25.0, 121.0
        self.alt_msl, self.alt_rel = 400.0, 400.0
        self.heading, self.ground_speed, self.climb = 0.0, 30.0, 0.0
        self.vel_n, self.vel_e, self.vel_d = 30.0, 0.0, 0.0
        self.gps_fix, self.hdop = 3, 1.0

    def is_valid_gps(self):
        return True


class _FakeWin:
    """只帶 _confirm_pending_release / _play_store_drop 需要的東西。

    常數與 _play_store_drop 都**從真的 MainWindow 取**，不要在測試裡另抄一份 ——
    否則改了逾時或容差，測試還在驗舊值（就是這個專案記過的「兩份複本會漂移」）。
    """

    def __init__(self, cesium=None):
        from ui.main_window import MainWindow
        self._pending_releases = []
        self._cesium = cesium if cesium is not None else _FakeCesium()
        self.map_widget = type('MW', (), {'_mode': 1})()   # 1 = _MODE_3D
        self._status = _FakeStatusBar()
        self._RELEASE_PWM_TOL = MainWindow._RELEASE_PWM_TOL
        self._RELEASE_CONFIRM_TIMEOUT_S = MainWindow._RELEASE_CONFIRM_TIMEOUT_S
        self._play_store_drop = (
            lambda frame, pr: MainWindow._play_store_drop(self, frame, pr))

    def statusBar(self):
        return self._status

    def _get_cesium_widget(self):
        return self._cesium


def _mw():
    from ui.main_window import MainWindow
    return MainWindow


def _pending(ch=9, target=2000, before=1000, sysid=1, kind='dumb'):
    return dict(sysid=sysid, ch=ch, target=target, before=before,
                kind=kind, label='1 號彈', t0=0.0, has_stream=True)


def test_no_animation_until_pwm_actually_moves(qapp):
    """PWM 還沒動 -> 一定不能播動畫。這是這次改動的核心要求。"""
    MW = _mw()
    win = _FakeWin()
    win._pending_releases = [_pending()]
    servo = [0] * 16
    servo[8] = 1000                      # ch9 還停在鎖住位置
    MW._confirm_pending_release(win, _FakeFrame(servo=servo))
    assert win._cesium.played == [], 'PWM 沒動就播動畫 = 假綠燈'
    assert len(win._pending_releases) == 1, '還沒確認，不可以把 pending 清掉'


def test_animation_plays_once_pwm_reaches_target(qapp):
    MW = _mw()
    win = _FakeWin()
    win._pending_releases = [_pending()]
    servo = [0] * 16
    servo[8] = 2000                      # ch9 真的跳到 HI
    MW._confirm_pending_release(win, _FakeFrame(servo=servo))
    assert len(win._cesium.played) == 1
    traj, label = win._cesium.played[0]
    assert traj.tof > 0 and traj.range_m > 0
    assert '1 號彈' in label
    assert win._pending_releases == [], '確認後要把 pending 清掉，避免重播'


def test_wrong_channel_moving_does_not_trigger(qapp):
    """別的通道動了不算 —— 否則任何舵面動作都會觸發動畫。"""
    MW = _mw()
    win = _FakeWin()
    win._pending_releases = [_pending(ch=9)]
    servo = [0] * 16
    servo[9] = 2000                      # 動的是 ch10
    servo[8] = 1000
    MW._confirm_pending_release(win, _FakeFrame(servo=servo))
    assert win._cesium.played == []


def test_other_aircraft_pwm_does_not_trigger(qapp):
    """多機時，別台的 PWM 不能觸發這台的動畫。"""
    MW = _mw()
    win = _FakeWin()
    win._pending_releases = [_pending(sysid=2)]
    servo = [0] * 16
    servo[8] = 2000
    MW._confirm_pending_release(win, _FakeFrame(sysid=1, servo=servo))
    assert win._cesium.played == []
    assert len(win._pending_releases) == 1


def test_already_at_target_is_not_a_release(qapp):
    """按下前就停在 2000（上一發沒歸位）-> 讀到 2000 也不算動過。"""
    MW = _mw()
    win = _FakeWin()
    win._pending_releases = [_pending(before=2000)]
    servo = [0] * 16
    servo[8] = 2000
    MW._confirm_pending_release(win, _FakeFrame(servo=servo))
    assert win._cesium.played == []


def test_timeout_reports_the_right_reason(qapp):
    """逾時要大聲說沒動，而且要分辨「沒有遙測」與「有遙測但沒變」。"""
    MW = _mw()
    win = _FakeWin()
    pr_no_stream = _pending(sysid=1)
    pr_no_stream['has_stream'] = False
    pr_no_change = _pending(sysid=2)
    win._pending_releases = [pr_no_stream, pr_no_change]
    win._release_timer = type('T', (), {'isActive': lambda s: True,
                                        'stop': lambda s: None})()
    MW._sweep_pending_releases(win)
    msgs = ' | '.join(win._status.msgs)
    assert '未確認投放' in msgs
    assert 'SERVO_OUTPUT_RAW' in msgs, '沒有遙測的那台要講清楚是收不到'
    assert 'FUNCTION' in msgs, '有遙測但沒變的那台要指向 SERVOn_FUNCTION'
    assert win._pending_releases == []


def test_2d_mode_confirms_but_does_not_draw_into_hidden_pane(qapp):
    """2D 模式下照樣確認投放，但不畫進看不見的圖台。"""
    MW = _mw()
    win = _FakeWin()
    win.map_widget._mode = 0             # 0 = _MODE_2D
    MW._play_store_drop(win, _FakeFrame(), _pending())
    assert win._cesium.played == []
    assert any('3D' in m for m in win._status.msgs)
