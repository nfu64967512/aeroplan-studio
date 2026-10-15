# -*- coding: utf-8 -*-
"""tests/test_mission_write.py — AeroPlan 航點檔 Load / Write / Save WPs（UI 層）。

被測的是 **UI 這一側的接線與守衛**，不是 mission/waypoint_file.py 或
mission/sitl_link.py 本身（那兩個核心已經各自有自己的測試檔，這裡只用真貨）。

═══ 這個檔在防哪些已經在別處發生過、這裡刻意重演一次的事 ═══

① **兩架飛機拿到同一份任務**（on_sitl_upload_mission 的 per_uav_wps 註解記過
   一次）⇒ `on_sitl_write_wp` 把「目標必須恰好一台」做成硬性拒絕，這裡鎖住
   0 台 / 1 台 / >1 台三種情況都對。

② **確認對話框顯示的東西跟實際送出的不是同一份**（effective-value contract）
   ⇒ 用「對話框開著時使用者又載入了別的檔案」這個具體情境，鎖住快照必須在
   確認「當下」鎖定，而不是送出前才重讀 `self._loaded_mission`。

③ **失敗看起來像成功**（本 repo 的核心紀律）⇒ 寫入失敗、載入失敗都必須有
   `QMessageBox.critical`，不能只是一行 log。

④ **讀回任務重建圍籬**（`tests/test_mission_download.py::
   TestDrawingDoesNotRebuildTheGeofence` 記過的同型坑）⇒ 載入預覽也必須走
   `display_fc_mission`，不能用 `display_path`/`display_paths`/
   `display_fw_paths`。

⑤ **GUI 在本環境不能真的 New 出來**（QtWebEngineWidgets 必須在 QApplication
   建立前匯入，否則直接崩潰不留 traceback）⇒ 全部走 unbound-method + 假 self
   的寫法（house style，見 tests/test_hv_dccpp_adapter.py::_DisplayWindow、
   tests/test_dccpp_reroute_altitude.py 的 QMessageBox 整體替換測法）。
   `SITLHud`/`DroneDetailPanel` 這種輕量 QWidget 本身可以真的建出來，用來測
   訊號/按鈕存在與否；只有 `MainWindow` 不行。
"""
from __future__ import annotations

import ast
import os
import pathlib
import re
import types
import uuid

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

ROOT = pathlib.Path(__file__).resolve().parents[1]

# ── Qt / 主視窗匯入（headless）── 見檔頭 ⑤：MainWindow 必須先匯入（連帶載入
#    QtWebEngineWidgets），QApplication 才能在之後建立，順序反了會直接崩潰。
try:
    from ui.main_window import MainWindow, looks_like_real_board_connection
    import PyQt6.QtWidgets as _QtW
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    from ui.widgets.sitl_hud import SITLHud
    from ui.widgets.drone_detail_panel import DroneDetailPanel
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'ui.main_window / PyQt6 匯入失敗: {globals().get("_QT_ERR")}')

from mission.waypoint_file import (  # noqa: E402
    MissionFileError, load_mission_file, parse_wpl_text, save_wpl,
)
from mission.sitl_link import SITLLink  # noqa: E402


SAMPLE_WPL = (
    "QGC WPL 110\n"
    "0\t1\t0\t16\t0\t0\t0\t0\t25.0000000\t121.0000000\t0.0\t1\n"
    "1\t0\t3\t22\t0\t0\t0\t0\t25.0010000\t121.0010000\t50.0\t1\n"
    "2\t0\t3\t16\t0\t0\t0\t0\t25.0020000\t121.0020000\t50.0\t1\n"
    "3\t0\t3\t21\t0\t0\t0\t0\t25.0030000\t121.0030000\t0.0\t1\n"
)


def _good_mission():
    lm = parse_wpl_text(SAMPLE_WPL, source='C:/tmp/sample.waypoints')
    assert not lm.has_errors, 'fixture 本身就有錯誤，下面的測試全部測不了正常路徑'
    return lm


def _bad_mission():
    """驗證會擋下的任務：只有 Home 槽，觸發 `empty` 錯誤。"""
    lm = parse_wpl_text(
        "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t0.0\t0.0\t0.0\t1\n",
        source='C:/tmp/empty.waypoints')
    assert lm.has_errors, 'fixture 本身沒有錯誤，測不了「有錯誤就拒絕」那條'
    return lm


# ══════════════════════════════════════════════════════════════════════
#  假 self / 假替身（house style：只給真的會被碰到的屬性）
# ══════════════════════════════════════════════════════════════════════
class _Button:
    def __init__(self):
        self.enabled = True
        self.history = []

    def setEnabled(self, v):
        self.enabled = bool(v)
        self.history.append(bool(v))

    def isEnabled(self):
        return self.enabled


class _StatusBar:
    def __init__(self):
        self.messages = []

    def showMessage(self, text, timeout=0):
        self.messages.append(text)


class _Label:
    def __init__(self):
        self.text_ = ''

    def setText(self, t):
        self.text_ = t


class _Hud:
    def __init__(self):
        self.btn_write_wp = _Button()
        self.lbl_status = _Label()
        self.on_disconnected_calls = []
        self.remove_card_calls = []

    def on_disconnected(self, reason):
        self.on_disconnected_calls.append(reason)

    def remove_card(self, sysid):
        self.remove_card_calls.append(sysid)


class _MapWidget:
    def __init__(self):
        self.calls = []

    def display_fc_mission(self, *a, **k):
        self.calls.append(('display_fc_mission', a, k))

    def display_path(self, *a, **k):
        self.calls.append(('display_path', a, k))

    def display_paths(self, *a, **k):
        self.calls.append(('display_paths', a, k))

    def display_fw_paths(self, *a, **k):
        self.calls.append(('display_fw_paths', a, k))


class _Frame:
    def __init__(self, armed=False, mode='MANUAL'):
        self.armed = armed
        self.mode = mode


class _FakeLink:
    """假 SITLLink：只做 upload_mission_items 記錄，不碰任何 socket。"""

    def __init__(self, sysid=1, conn_str='udpin:0.0.0.0:14550', frame=None):
        self.sysid_label = sysid
        self.conn_str = conn_str
        self.uploaded = []          # [(items, verify), ...]
        self._frame = frame

    def get_latest_telemetry(self):
        return self._frame

    def upload_mission_items(self, items, verify=True):
        self.uploaded.append((items, verify))


_BOUND_METHODS = (
    'on_sitl_load_wp_file', 'on_sitl_write_wp', '_on_mission_uploaded',
    'on_sitl_save_wp_file', '_update_write_wp_enabled',
    '_on_detail_panel_write_wp', '_on_hil_disconnect_all',
    '_on_sitl_link_disconnected',
)


class _HilPanel:
    """假 HILConnectPanel：只給 `_on_hil_disconnect_all` 真的會呼叫到的方法。"""

    def __init__(self):
        self.disconnected_calls = 0

    def on_disconnected(self):
        self.disconnected_calls += 1


class _HilLink(_FakeLink):
    """假 HIL 鏈路：疊在 `_FakeLink` 上，多補 `_on_hil_disconnect_all` 會碰到的
    `disconnected`（可 `.disconnect()` 的假訊號）/`stop()`/`wait()`。"""

    class _Signal:
        def disconnect(self):
            pass

    def __init__(self, sysid=1, conn_str='COM7,115200'):
        super().__init__(sysid=sysid, conn_str=conn_str)
        self.disconnected = self._Signal()
        self.stopped = False

    def stop(self):
        self.stopped = True

    def wait(self, ms):
        pass


class _FakeWin:
    """跑 Write WPs 一族方法用的假 self —— 只給它們真的會碰的屬性（house style，
    見 tests/test_hv_dccpp_adapter.py::_JoinWindow 用 types.MethodType 綁真方法
    的寫法）。"""

    def __init__(self, target_links=None, sitl_links=None, hil_links=None):
        self.map_widget = _MapWidget()
        self.sitl_hud = _Hud()
        self._status = _StatusBar()
        self._loaded_mission = None
        self._fc_mission = None
        self._write_pending_sysids = set()
        self._last_wp_dir = ''
        self._sitl_links = list(sitl_links) if sitl_links is not None else []
        self._target = list(target_links) if target_links is not None else []
        # ── _on_hil_disconnect_all 專用的假屬性（其餘方法不會碰到） ──
        self._hil_links = list(hil_links) if hil_links is not None else []
        self._sitl_link = self._sitl_links[0] if self._sitl_links else None
        self.hil_panel = _HilPanel()
        for name in _BOUND_METHODS:
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))

    def statusBar(self):
        return self._status

    def _target_links(self):
        return list(self._target)


@pytest.fixture
def qmb_stub(monkeypatch):
    """把 QMessageBox 的幾個類別方法換成記錄用的假函式（house style，見
    tests/test_dccpp_reroute_altitude.py 的 `_QtW.QMessageBox.warning =
    staticmethod(...)`）。回傳一個 dict：呼叫記錄 + 設定回傳值的方法。"""
    QMB = _QtW.QMessageBox
    calls = {'information': [], 'warning': [], 'critical': [], 'question': []}
    ret = {'value': QMB.StandardButton.Ok}

    def mk(name):
        def _f(*a, **k):
            calls[name].append((a, k))
            return ret['value']
        return staticmethod(_f)

    for name in ('information', 'warning', 'critical', 'question'):
        monkeypatch.setattr(QMB, name, mk(name))
    return {'calls': calls, 'ret': ret, 'QMB': QMB}


@pytest.fixture
def filedialog_stub(monkeypatch):
    """把 QFileDialog 的 getOpenFileName/getSaveFileName 換成可控制回傳值的假函式。"""
    state = {'open': ('', ''), 'save': ('', ''), 'open_calls': [], 'save_calls': []}

    def _open(*a, **k):
        state['open_calls'].append((a, k))
        return state['open']

    def _save(*a, **k):
        state['save_calls'].append((a, k))
        return state['save']

    monkeypatch.setattr(_QtW.QFileDialog, 'getOpenFileName', staticmethod(_open))
    monkeypatch.setattr(_QtW.QFileDialog, 'getSaveFileName', staticmethod(_save))
    return state


# ══════════════════════════════════════════════════════════════════════
#  A. 真板子偵測（純函式，不需要 Qt）
# ══════════════════════════════════════════════════════════════════════
class TestRealBoardDetection:
    @pytest.mark.parametrize('conn,expected', [
        ('COM3,57600', True),
        ('COM10,115200', True),
        ('/dev/ttyACM0', True),
        ('/dev/ttyUSB0', True),
        ('serial:/dev/foo:115200', True),
        ('udpin:0.0.0.0:14550', False),
        ('udp:127.0.0.1:14550', False),
        ('tcp:127.0.0.1:5760', False),
        ('', False),
        ('command_center', False),  # 'com' 開頭但第 4 碼不是數字，不可誤判
    ])
    @qt_only
    def test_classification(self, conn, expected):
        assert looks_like_real_board_connection(conn) is expected


# ══════════════════════════════════════════════════════════════════════
#  B. HUD：三顆新按鈕 + 三個新信號
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestHudButtonsAndSignals:
    def test_signals_exist(self):
        h = SITLHud()
        for sig in ('cmd_load_wp', 'cmd_write_wp', 'cmd_save_wp'):
            assert hasattr(h, sig), f'SITLHud 缺少信號 {sig}'

    def test_buttons_exist_right_after_download(self):
        h = SITLHud()
        assert hasattr(h, 'btn_load_wp')
        assert hasattr(h, 'btn_write_wp')
        assert hasattr(h, 'btn_save_wp')

    def test_write_button_starts_disabled(self):
        """⚠️ 沒有連線就不可以按得下去 Write WPs —— 這是安全關鍵鈕的預設狀態。"""
        h = SITLHud()
        assert h.btn_write_wp.isEnabled() is False

    def test_load_and_save_buttons_start_enabled(self):
        """載入不需要連線；存檔鈕本身不鎖（真正的「沒東西可存」由 handler 說明）。"""
        h = SITLHud()
        assert h.btn_load_wp.isEnabled() is True
        assert h.btn_save_wp.isEnabled() is True

    def test_tooltips_are_non_empty_and_chinese_ui(self):
        """粗略把關：三顆鈕都要有說明文字，且是繁中（含常見繁中安全用語）。"""
        h = SITLHud()
        for btn, must_contain in (
            (h.btn_load_wp, '不需要連線'),
            (h.btn_write_wp, '確認對話框'),
            (h.btn_save_wp, '讀回'),
        ):
            tip = btn.toolTip()
            assert tip, f'{btn.text()} 沒有 tooltip'
            assert must_contain in tip, f'{btn.text()} 的 tooltip 少了關鍵字 {must_contain!r}'

    def test_write_button_tooltip_does_not_overpromise_target_gating(self):
        """⚠️ finding（major，ui_safety）：舊版 tooltip 宣稱『指令對象是「全部」
        或沒有連線時無法按』，但 `MainWindow._update_write_wp_enabled()` 其實
        只依「有沒有連線」「有沒有寫入還在等回報」兩個條件決定 enabled，從不
        看指令對象——鈕的實際可按狀態跟自己 tooltip 講的不一致。這裡鎖住
        新文字**不再**做那個承諾，並且明確講清楚「對象」是送出時才擋。"""
        h = SITLHud()
        tip = h.btn_write_wp.toolTip()
        assert '指令對象' in tip and '送出' in tip, 'tooltip 應該講清楚對象是送出時才擋，不是鈕本身的 enabled 條件'
        assert '無法按' not in tip, (
            '不可以再宣稱「指令對象是全部就無法按」——_update_write_wp_enabled() '
            '沒有這個邏輯，講了會跟實際行為不一致')

    def test_clicking_buttons_emits_signals(self, qtbot=None):
        h = SITLHud()
        seen = []
        h.cmd_load_wp.connect(lambda: seen.append('load'))
        h.cmd_save_wp.connect(lambda: seen.append('save'))
        h.btn_load_wp.click()
        h.btn_save_wp.click()
        assert seen == ['load', 'save']
        # write 按鈕預設停用，click() 對停用元件是 no-op —— 這裡改用手動 enable
        # 後點擊，確認訊號真的接到 cmd_write_wp.emit（而不只是外觀接好）。
        h.btn_write_wp.setEnabled(True)
        h.cmd_write_wp.connect(lambda: seen.append('write'))
        h.btn_write_wp.click()
        assert seen == ['load', 'save', 'write']


# ══════════════════════════════════════════════════════════════════════
#  C. DroneDetailPanel：「↥」鈕不再是死的
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestDroneDetailPanelUploadButton:
    def test_write_wp_requested_signal_exists(self):
        d = DroneDetailPanel()
        assert hasattr(d, 'write_wp_requested')

    def test_click_emits_with_current_callsign(self):
        d = DroneDetailPanel()
        d._callsign = 'UAV-3'
        seen = []
        d.write_wp_requested.connect(lambda cs: seen.append(cs))
        d._btn_upload.click()
        assert seen == ['UAV-3']

    def test_click_with_no_callsign_is_a_noop(self):
        """沒有選定機的時候點「↥」不可以送出一個空字串當 callsign。"""
        d = DroneDetailPanel()
        d._callsign = None
        seen = []
        d.write_wp_requested.connect(lambda cs: seen.append(cs))
        d._btn_upload.click()
        assert seen == []

    def test_upload_button_is_wired_via_ast(self):
        """AST 而非字串比對：確認 `_btn_upload.clicked.connect(` 這個呼叫真的存在
        （字串比對法會被本檔開頭那行『dead 的 "↥" upload button』的說明文字誤判，
        見本 repo 記錄在案的第六次同型誤判，test_mission_download.py 開頭引用過）。
        """
        src = (ROOT / 'ui' / 'widgets' / 'drone_detail_panel.py').read_text(encoding='utf-8')
        tree = ast.parse(src)
        found = False
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'connect'
                    and isinstance(node.func.value, ast.Attribute)
                    and node.func.value.attr == 'clicked'
                    and isinstance(node.func.value.value, ast.Attribute)
                    and node.func.value.value.attr == '_btn_upload'):
                found = True
        assert found, '_btn_upload.clicked.connect(...) 沒有被接上'


# ══════════════════════════════════════════════════════════════════════
#  D. main_window 接線（AST / regex 交叉比對，house style 同
#     test_mission_download.py::TestEveryLinkSignalMainWindowConnectsExists）
# ══════════════════════════════════════════════════════════════════════
class TestMainWindowWiring:
    SRC = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8')

    @pytest.mark.parametrize('snippet', [
        'self.sitl_hud.cmd_load_wp.connect(self.on_sitl_load_wp_file)',
        'self.sitl_hud.cmd_write_wp.connect(self.on_sitl_write_wp)',
        'self.sitl_hud.cmd_save_wp.connect(self.on_sitl_save_wp_file)',
    ])
    def test_hud_signals_connected(self, snippet):
        assert snippet in self.SRC, f'HUD 訊號沒有接：{snippet}'

    def test_mission_uploaded_signal_exists_on_sitl_link(self):
        """交叉比對：main_window 接的 signal 必須真的存在於 SITLLink 上
        （ParamService 那一類「送得出去、收不回來」的形狀）。"""
        assert hasattr(SITLLink, 'mission_uploaded')

    def test_mission_uploaded_connected_exactly_once_inside_on_sitl_connect(self):
        """⭐ 用 AST 數 `on_sitl_connect` 函式體內 `link.mission_uploaded.connect(`
        的呼叫次數，必須恰好 1 次——零次 = 沒人聽 mission_uploaded；
        接在別的地方（例如按鈕處理函式）則不會落在這個函式體內，同樣會被算成 0，
        逼著新接線也要接在 on_sitl_connect 裡（跟 mission_downloaded 相同理由：
        不論觸發來源是 HUD / DroneDetailPanel / 程式自己呼叫，都要聽得到，而且
        每條 link 只接一次，不會「按第 N 次連線接 N 條」）。
        """
        tree = ast.parse(self.SRC)
        fn = next((n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == 'on_sitl_connect'), None)
        assert fn is not None, '找不到 on_sitl_connect'
        calls = [c for c in ast.walk(fn)
                 if isinstance(c, ast.Call)
                 and isinstance(c.func, ast.Attribute)
                 and c.func.attr == 'connect'
                 and isinstance(c.func.value, ast.Attribute)
                 and c.func.value.attr == 'mission_uploaded']
        assert len(calls) == 1, (
            f'on_sitl_connect 裡 mission_uploaded.connect( 出現 {len(calls)} 次，應該恰好 1 次')

    def test_hil_connect_all_routes_through_on_sitl_connect(self):
        """HIL（實體飛控）多機連線必須走同一條 on_sitl_connect，
        否則真板子不會有一樣的 mission_uploaded 接線。"""
        tree = ast.parse(self.SRC)
        fn = next((n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == '_on_hil_connect_all'), None)
        assert fn is not None
        calls = [c for c in ast.walk(fn)
                 if isinstance(c, ast.Call)
                 and isinstance(c.func, ast.Attribute)
                 and c.func.attr == 'on_sitl_connect']
        assert calls, '_on_hil_connect_all 沒有呼叫 self.on_sitl_connect(...)'

    def test_detail_panel_write_wp_requested_connected(self):
        assert 'self.sitl_command_panel.detail.write_wp_requested.connect(' in self.SRC
        assert '_on_detail_panel_write_wp' in self.SRC

    def test_on_sitl_upload_mission_machinery_not_reused(self):
        """⚠️ Write WPs 不可以重用 on_sitl_upload_mission 的機制——語意不同
        （當前規劃路徑 vs 已載入航點檔），混用會兩邊都難懂難測。"""
        tree = ast.parse(self.SRC)
        fn = next((n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == 'on_sitl_write_wp'), None)
        assert fn is not None
        calls = {c.func.attr for c in ast.walk(fn)
                 if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}
        assert 'on_sitl_upload_mission' not in calls


# ══════════════════════════════════════════════════════════════════════
#  E. on_sitl_load_wp_file
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestLoadWpFile:
    def test_success_stores_and_previews(self, tmp_path, qmb_stub, filedialog_stub):
        p = tmp_path / 'sample.waypoints'
        p.write_text(SAMPLE_WPL, encoding='utf-8')
        filedialog_stub['open'] = (str(p), '')
        win = _FakeWin()
        win.on_sitl_load_wp_file()
        assert win._loaded_mission is not None
        assert len(win._loaded_mission.items) == 4
        names = [c[0] for c in win.map_widget.calls]
        assert 'display_fc_mission' in names

    def test_success_does_not_use_fence_rebuilding_methods(self, tmp_path, qmb_stub,
                                                            filedialog_stub):
        p = tmp_path / 'sample.waypoints'
        p.write_text(SAMPLE_WPL, encoding='utf-8')
        filedialog_stub['open'] = (str(p), '')
        win = _FakeWin()
        win.on_sitl_load_wp_file()
        names = [c[0] for c in win.map_widget.calls]
        for bad in ('display_path', 'display_paths', 'display_fw_paths'):
            assert bad not in names, f'載入預覽呼叫了 {bad}（會重建圍籬）'

    def test_preview_uses_display_fc_mission_via_ast(self):
        """AST 交叉比對：on_sitl_load_wp_file 真的呼叫了 display_fc_mission，
        且完全沒有呼叫 display_path/display_paths/display_fw_paths。"""
        src = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8')
        tree = ast.parse(src)
        fn = next((n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == 'on_sitl_load_wp_file'), None)
        assert fn is not None
        attrs = {c.func.attr for c in ast.walk(fn)
                 if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}
        assert 'display_fc_mission' in attrs
        assert not ({'display_path', 'display_paths', 'display_fw_paths'} & attrs)

    def test_error_shows_critical_never_silent(self, tmp_path, qmb_stub, filedialog_stub):
        p = tmp_path / 'bad.waypoints'
        p.write_text('not a waypoints file\n', encoding='utf-8')
        filedialog_stub['open'] = (str(p), '')
        win = _FakeWin()
        win.on_sitl_load_wp_file()
        assert qmb_stub['calls']['critical'], '載入失敗沒有跳出 critical（不可以靜默 return）'

    def test_error_leaves_previous_loaded_mission_intact(self, tmp_path, qmb_stub,
                                                          filedialog_stub):
        p = tmp_path / 'bad.waypoints'
        p.write_text('not a waypoints file\n', encoding='utf-8')
        filedialog_stub['open'] = (str(p), '')
        win = _FakeWin()
        prev = _good_mission()
        win._loaded_mission = prev
        win.on_sitl_load_wp_file()
        assert win._loaded_mission is prev, '載入失敗卻覆蓋掉了先前已載入的任務'

    def test_mission_with_validation_errors_is_refused(self, tmp_path, qmb_stub,
                                                        filedialog_stub):
        """語法沒錯但語意錯誤（例如空任務）一樣要擋，不能進 _loaded_mission。"""
        p = tmp_path / 'empty.waypoints'
        p.write_text("QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t0.0\t0.0\t0.0\t1\n",
                     encoding='utf-8')
        filedialog_stub['open'] = (str(p), '')
        win = _FakeWin()
        win.on_sitl_load_wp_file()
        assert win._loaded_mission is None
        assert qmb_stub['calls']['critical']

    def test_cancel_dialog_is_a_noop(self, qmb_stub, filedialog_stub):
        filedialog_stub['open'] = ('', '')
        win = _FakeWin()
        win.on_sitl_load_wp_file()
        assert win._loaded_mission is None
        assert not qmb_stub['calls']['critical']
        assert not qmb_stub['calls']['information']

    def test_does_not_require_a_connection(self, tmp_path, qmb_stub, filedialog_stub):
        """B 條款：載入不需要連線 —— _sitl_links 空的也要能載入成功。"""
        p = tmp_path / 'sample.waypoints'
        p.write_text(SAMPLE_WPL, encoding='utf-8')
        filedialog_stub['open'] = (str(p), '')
        win = _FakeWin(sitl_links=[])
        win.on_sitl_load_wp_file()
        assert win._loaded_mission is not None

    def test_remembers_last_directory(self, tmp_path, qmb_stub, filedialog_stub):
        p = tmp_path / 'sample.waypoints'
        p.write_text(SAMPLE_WPL, encoding='utf-8')
        filedialog_stub['open'] = (str(p), '')
        win = _FakeWin()
        assert win._last_wp_dir == ''
        win.on_sitl_load_wp_file()
        assert win._last_wp_dir == str(tmp_path)


# ══════════════════════════════════════════════════════════════════════
#  F. on_sitl_write_wp（安全關鍵）
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestWriteWpGuards:
    def test_no_loaded_mission_refused(self, qmb_stub):
        link = _FakeLink(sysid=1)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win.on_sitl_write_wp()
        assert link.uploaded == []
        assert qmb_stub['calls']['information']

    def test_mission_with_errors_refused(self, qmb_stub):
        link = _FakeLink(sysid=1)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _bad_mission()
        win.on_sitl_write_wp()
        assert link.uploaded == []
        assert qmb_stub['calls']['critical']

    def test_multi_target_refused(self, qmb_stub):
        l1, l2 = _FakeLink(sysid=1), _FakeLink(sysid=2)
        win = _FakeWin(target_links=[l1, l2], sitl_links=[l1, l2])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win.on_sitl_write_wp()
        assert l1.uploaded == [] and l2.uploaded == []
        assert qmb_stub['calls']['warning']

    def test_zero_target_refused(self, qmb_stub):
        win = _FakeWin(target_links=[], sitl_links=[])
        win._loaded_mission = _good_mission()
        win.on_sitl_write_wp()
        assert qmb_stub['calls']['information']
        assert win._write_pending_sysids == set()

    def test_cancel_sends_nothing(self, qmb_stub):
        link = _FakeLink(sysid=3)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win.on_sitl_write_wp()
        assert link.uploaded == [], 'Cancel 之後仍然送出了東西'
        assert win._write_pending_sysids == set()

    def test_confirmed_sends_exact_snapshot_with_verify_true(self, qmb_stub):
        link = _FakeLink(sysid=5)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        lm = _good_mission()
        win._loaded_mission = lm
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win.on_sitl_write_wp()
        assert len(link.uploaded) == 1
        items, verify = link.uploaded[0]
        assert verify is True
        assert [i['seq'] for i in items] == [i['seq'] for i in lm.items]
        assert [i['lat'] for i in items] == [i['lat'] for i in lm.items]

    def test_snapshot_is_a_copy_not_a_live_reference(self, qmb_stub):
        """快照必須是複製，不是同一個 list 物件——之後修改 lm.items
        不可以改到已經送出去的內容。"""
        link = _FakeLink(sysid=5)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        lm = _good_mission()
        win._loaded_mission = lm
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win.on_sitl_write_wp()
        items, _ = link.uploaded[0]
        assert items is not lm.items

    def test_snapshot_vs_reload_race(self, qmb_stub):
        """⚠️⚠️ 對話框開著時使用者又載入了別的檔案 —— 已經送出去的必須是
        確認「當下」鎖定的那份，不是後來重新載入的那份。用 QMessageBox 的
        stub 在『對話框正在顯示』的那一刻模擬使用者觸發了重新載入。"""
        link = _FakeLink(sysid=5)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        lm = _good_mission()
        other = parse_wpl_text(
            "QGC WPL 110\n"
            "0\t1\t0\t16\t0\t0\t0\t0\t9.0\t9.0\t0.0\t1\n"
            "1\t0\t3\t22\t0\t0\t0\t0\t9.001\t9.001\t50.0\t1\n"
            "2\t0\t3\t21\t0\t0\t0\t0\t9.002\t9.002\t0.0\t1\n",
            source='C:/tmp/other.waypoints')
        win._loaded_mission = lm

        def _question(*a, **k):
            win._loaded_mission = other   # 模擬「對話框開著時使用者換了檔案」
            return _QtW.QMessageBox.StandardButton.Yes

        _QtW.QMessageBox.question = staticmethod(_question)
        _QtW.QMessageBox.warning = staticmethod(_question)
        win.on_sitl_write_wp()
        assert link.uploaded, '沒有送出任何東西'
        items, _ = link.uploaded[0]
        sent_lats = sorted(i['lat'] for i in items)
        orig_lats = sorted(i['lat'] for i in lm.items)
        assert sent_lats == orig_lats, '送出的是對話框打開後才載入的那份，不是確認當下的快照'

    @pytest.mark.parametrize('conn_str', ['COM7,115200', '/dev/ttyACM0', 'serial:/dev/foo'])
    def test_real_board_uses_stronger_wording_and_default_cancel(self, qmb_stub, conn_str):
        link = _FakeLink(sysid=7, conn_str=conn_str)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win.on_sitl_write_wp()
        assert qmb_stub['calls']['warning'], f'{conn_str} 沒有用 warning() 的重措辭'
        assert not qmb_stub['calls']['question'], f'{conn_str} 卻也呼叫了 question()'
        args, kwargs = qmb_stub['calls']['warning'][0]
        # QMessageBox.warning(parent, title, text, buttons, defaultButton)
        assert args[-1] == _QtW.QMessageBox.StandardButton.Cancel, \
            '真實飛控的預設鈕不是 Cancel'

    def test_sitl_uses_question_wording_and_default_yes(self, qmb_stub):
        link = _FakeLink(sysid=7, conn_str='udpin:0.0.0.0:14550')
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win.on_sitl_write_wp()
        assert qmb_stub['calls']['question']
        assert not qmb_stub['calls']['warning']
        args, kwargs = qmb_stub['calls']['question'][0]
        assert args[-1] == _QtW.QMessageBox.StandardButton.Yes

    def test_armed_auto_warning_present(self, qmb_stub):
        link = _FakeLink(sysid=9, frame=_Frame(armed=True, mode='AUTO'))
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win.on_sitl_write_wp()
        seen_text = [a[2] for a, k in (qmb_stub['calls']['warning']
                                       + qmb_stub['calls']['question']) if len(a) >= 3]
        assert seen_text and 'AUTO' in seen_text[0] and 'ARMED' in seen_text[0]

    @pytest.mark.parametrize('armed,mode', [(False, 'AUTO'), (True, 'MANUAL'), (False, 'MANUAL')])
    def test_no_armed_auto_warning_when_not_flying_auto(self, qmb_stub, armed, mode):
        link = _FakeLink(sysid=9, frame=_Frame(armed=armed, mode=mode))
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win.on_sitl_write_wp()
        seen_text = [a[2] for a, k in (qmb_stub['calls']['warning']
                                       + qmb_stub['calls']['question']) if len(a) >= 3]
        assert seen_text and 'ARMED 且在 AUTO' not in seen_text[0]

    def test_confirmation_shows_item_count_and_filename(self, qmb_stub):
        link = _FakeLink(sysid=1)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        lm = _good_mission()
        win._loaded_mission = lm
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win.on_sitl_write_wp()
        text = qmb_stub['calls']['question'][0][0][2]
        assert str(len(lm.items)) in text
        assert 'sample.waypoints' in text

    def test_confirmation_shows_every_warning(self, qmb_stub):
        # 建一份會觸發警告但不會觸發錯誤的任務（缺起飛指令 → no_takeoff 警告）。
        lm = parse_wpl_text(
            "QGC WPL 110\n"
            "0\t1\t0\t16\t0\t0\t0\t0\t25.0\t121.0\t0.0\t1\n"
            "1\t0\t3\t16\t0\t0\t0\t0\t25.001\t121.001\t50.0\t1\n"
            "2\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1\n",
            source='C:/tmp/warn.waypoints')
        assert not lm.has_errors and lm.warnings, 'fixture 沒有製造出警告，測不了這條'
        link = _FakeLink(sysid=1)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = lm
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win.on_sitl_write_wp()
        text = qmb_stub['calls']['question'][0][0][2]
        for w in lm.warnings:
            assert w.message in text, f'警告 {w.message!r} 沒有出現在確認文字裡'

    def test_disabled_while_in_flight_then_reenabled_on_completion(self, qmb_stub):
        link = _FakeLink(sysid=11)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win.on_sitl_write_wp()
        assert win.sitl_hud.btn_write_wp.enabled is False, '送出寫入後鈕沒有被停用'
        win._on_mission_uploaded(11, True, 'ok')
        assert win.sitl_hud.btn_write_wp.enabled is True, '完成回報後鈕沒有恢復'

    def test_synchronous_send_exception_clears_pending_and_reenables(self, qmb_stub):
        """⚠️ finding（major，test_bite）：`on_sitl_write_wp` 的
        `link.upload_mission_items(...)` 外面包了一層 try/except，失敗時
        會把 sysid 從 `_write_pending_sysids` 清掉、重新呼叫
        `_update_write_wp_enabled()`——但既有測試全部走的是「送出成功、之後
        `_on_mission_uploaded` 回報」這條非同步路徑，從沒有任何測試讓
        `upload_mission_items` 本身同步丟例外過。丟了兩個線索之後就沒有任何
        測試會抓到：一是這兩行清理被誤刪、二是失敗訊息不夠大聲——這裡兩個
        都鎖住。"""
        class _RaisingLink(_FakeLink):
            def upload_mission_items(self, items, verify=True):
                raise RuntimeError('假的佇列已經關閉')

        link = _RaisingLink(sysid=13)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes

        win.on_sitl_write_wp()

        assert 13 not in win._write_pending_sysids, (
            '同步送出失敗後 sysid 要被清掉，否則這台永遠卡在「寫入中」，'
            'HUD 按鈕跟 DroneDetailPanel「↥」的併發守衛都會永遠拒絕它')
        assert win.sitl_hud.btn_write_wp.enabled is True, '同步失敗後鈕要恢復可按'
        assert len(qmb_stub['calls']['critical']) == 1, '失敗必須大聲，不能只留 log'
        assert '假的佇列已經關閉' in qmb_stub['calls']['critical'][0][0][2]

    def test_double_click_cannot_start_two_transactions(self, qmb_stub):
        """鈕停用後真正的 UI 點不下去；這裡在方法層級驗證等價語意——
        第二次呼叫前先檢查鈕已停用，證明『連點兩次』在真實 UI 上根本點不到第二次。"""
        link = _FakeLink(sysid=11)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win.on_sitl_write_wp()
        assert win.sitl_hud.btn_write_wp.enabled is False
        assert 11 in win._write_pending_sysids

    def test_pending_write_refuses_a_second_call_for_the_same_sysid(self, qmb_stub):
        """finding（blocker）：只呼叫 on_sitl_write_wp 一次的既有測試測不出
        『第二個入口（DroneDetailPanel 的「↥」鈕 → _on_detail_panel_write_wp）
        在同一台還沒寫完時又排一筆』——這裡直接呼叫兩次，模擬 HUD 按鈕已經
        送出、但 DroneDetailPanel 的「↥」鈕沒有對應的 enabled 狀態、使用者
        還是按得下去的情境。第二次呼叫必須被擋在確認對話框之前，不能排出
        第二筆 upload_mission_items()。"""
        link = _FakeLink(sysid=11)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes

        win.on_sitl_write_wp()  # 第一筆：HUD 按鈕
        assert len(link.uploaded) == 1
        assert 11 in win._write_pending_sysids

        n_question_before = len(qmb_stub['calls']['question'])
        win.on_sitl_write_wp()  # 第二筆：例如 DroneDetailPanel「↥」鈕
        assert len(link.uploaded) == 1, '同一台還在寫入時不該排出第二筆'
        assert len(qmb_stub['calls']['question']) == n_question_before, \
            '第二次呼叫要在確認對話框之前就被擋下，不能又跳一次「是否要寫入」'
        assert qmb_stub['calls']['information'], '要用大聲的提示告知使用者已經有一筆在跑'

        # 第一筆真正完成之後，才可以再寫一次（不是永久卡死）。
        win._on_mission_uploaded(11, True, '4 項相符')
        win.on_sitl_write_wp()
        assert len(link.uploaded) == 2

    def test_pending_write_guard_covers_detail_panel_route_too(self, qmb_stub):
        """跟上面同一個缺陷，但用真正的兩個入口重演一次：HUD 按鈕排一筆，
        還沒回應時透過 DroneDetailPanel 的「↥」鈕（_on_detail_panel_write_wp）
        寫同一台，第二筆必須被擋。"""
        link = _FakeLink(sysid=11)
        win = _FakeWin(target_links=[link], sitl_links=[link])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes

        win.on_sitl_write_wp()  # HUD 路徑
        assert len(link.uploaded) == 1

        win._on_detail_panel_write_wp('UAV-11')  # DroneDetailPanel 路徑，同一台
        assert len(link.uploaded) == 1, 'DroneDetailPanel 路徑不該繞過 pending 檢查'


# ══════════════════════════════════════════════════════════════════════
#  G. _on_mission_uploaded
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestMissionUploadedHandler:
    def test_failure_shows_critical_and_states_not_reliably_updated(self, qmb_stub):
        win = _FakeWin(sitl_links=[_FakeLink(sysid=1)])
        win._write_pending_sysids = {1}
        win._on_mission_uploaded(1, False, 'ACK 逾時')
        assert qmb_stub['calls']['critical']
        text = qmb_stub['calls']['critical'][0][0][2]
        assert '沒有被可靠地更新' in text
        assert 'ACK 逾時' in text

    def test_success_does_not_show_critical(self, qmb_stub):
        win = _FakeWin(sitl_links=[_FakeLink(sysid=1)])
        win._write_pending_sysids = {1}
        win._on_mission_uploaded(1, True, '4 項相符')
        assert not qmb_stub['calls']['critical']
        assert any('4 項相符' in m for m in win._status.messages)

    def test_discards_pending_and_updates_button_regardless_of_outcome(self, qmb_stub):
        for ok in (True, False):
            win = _FakeWin(sitl_links=[_FakeLink(sysid=1)])
            win._write_pending_sysids = {1}
            win._on_mission_uploaded(1, ok, 'x')
            assert win._write_pending_sysids == set()
            assert win.sitl_hud.btn_write_wp.enabled is True


# ══════════════════════════════════════════════════════════════════════
#  H. on_sitl_save_wp_file
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestSaveWpFile:
    def test_no_readback_refused_without_touching_dialog(self, qmb_stub, filedialog_stub):
        win = _FakeWin()
        win._fc_mission = None
        win.on_sitl_save_wp_file()
        assert qmb_stub['calls']['information']
        assert filedialog_stub['save_calls'] == [], '沒有讀回卻仍跳出存檔對話框'

    def test_zero_item_readback_is_refused_like_no_readback(self, qmb_stub, filedialog_stub):
        """⚠️ finding（minor，test_bite）：`_fc_mission = {'sysid': N, 'items': []}`
        是真的會發生的合法狀態——飛控上剛好沒有任務時，`_recv_mission` 走的是
        **成功**路徑、送出 `mission_downloaded(sysid, [])`（見
        mission/sitl_link.py 的『n == 0 是合法答案』），`_on_mission_downloaded`
        會原封不動把它存進 `self._fc_mission`。這裡要跟 `_fc_mission is None`
        一樣被拒絕、不能真的存出一份只有表頭的 .waypoints 檔還說『已存成』。"""
        win = _FakeWin()
        win._fc_mission = {'sysid': 4, 'items': []}
        win.on_sitl_save_wp_file()
        assert qmb_stub['calls']['information'], '空清單的讀回也要走跟沒讀回一樣的拒絕訊息'
        assert filedialog_stub['save_calls'] == [], '空清單卻仍跳出存檔對話框'

    def test_never_silently_saves_the_loaded_mission_instead(self, tmp_path, qmb_stub,
                                                              filedialog_stub):
        """E 條款：這顆鈕永遠存 `_fc_mission`，不是 `_loaded_mission`——
        即使兩者都存在，也不可以錯用來源。"""
        win = _FakeWin()
        win._loaded_mission = _good_mission()
        loaded_lats = sorted(i['lat'] for i in win._loaded_mission.items)
        win._fc_mission = {'sysid': 4, 'items': parse_wpl_text(
            "QGC WPL 110\n"
            "0\t1\t0\t16\t0\t0\t0\t0\t3.0\t3.0\t0.0\t1\n"
            "1\t0\t3\t22\t0\t0\t0\t0\t3.001\t3.001\t50.0\t1\n"
            "2\t0\t3\t21\t0\t0\t0\t0\t3.002\t3.002\t0.0\t1\n",
            source='<fc>').items}
        fc_lats = sorted(i['lat'] for i in win._fc_mission['items'])
        assert fc_lats != loaded_lats, 'fixture 兩份任務長一樣，測不出用錯來源'
        out = tmp_path / 'out.waypoints'
        filedialog_stub['save'] = (str(out), '')
        win.on_sitl_save_wp_file()
        back = load_mission_file(str(out))
        assert sorted(i['lat'] for i in back.items) == fc_lats

    def test_round_trip_via_real_io(self, tmp_path, qmb_stub, filedialog_stub):
        """存檔用真正的 `save_wpl`，讀回用真正的 `load_mission_file`——
        不是任何一邊的替身。"""
        lm = _good_mission()
        win = _FakeWin()
        win._fc_mission = {'sysid': 4, 'items': lm.items}
        out = tmp_path / 'roundtrip.waypoints'
        filedialog_stub['save'] = (str(out), '')
        win.on_sitl_save_wp_file()
        assert out.exists()
        back = load_mission_file(str(out))
        assert [i['seq'] for i in back.items] == [i['seq'] for i in lm.items]
        assert [i['command'] for i in back.items] == [i['command'] for i in lm.items]
        for a, b in zip(back.items, lm.items):
            assert a['lat'] == pytest.approx(b['lat'], abs=1e-7)
            assert a['lon'] == pytest.approx(b['lon'], abs=1e-7)

    def test_appends_extension_when_missing(self, tmp_path, qmb_stub, filedialog_stub):
        lm = _good_mission()
        win = _FakeWin()
        win._fc_mission = {'sysid': 4, 'items': lm.items}
        out = tmp_path / 'no_ext'
        filedialog_stub['save'] = (str(out), '')
        win.on_sitl_save_wp_file()
        assert (tmp_path / 'no_ext.waypoints').exists()

    def test_save_failure_shows_critical(self, tmp_path, qmb_stub, filedialog_stub,
                                         monkeypatch):
        lm = _good_mission()
        win = _FakeWin()
        win._fc_mission = {'sysid': 4, 'items': lm.items}
        out = tmp_path / 'fails.waypoints'
        filedialog_stub['save'] = (str(out), '')

        def _boom(items, path):
            raise MissionFileError('模擬寫檔失敗')

        import ui.main_window as mw
        monkeypatch.setattr(mw, 'save_wpl', _boom, raising=False)
        # save_wpl 是函式內 `from mission.waypoint_file import save_wpl` 區域匯入，
        # 直接補模組層級不會生效；改成補目標模組本身的函式。
        import mission.waypoint_file as wf
        monkeypatch.setattr(wf, 'save_wpl', _boom)
        win.on_sitl_save_wp_file()
        assert qmb_stub['calls']['critical']

    def test_cancel_dialog_is_a_noop(self, qmb_stub, filedialog_stub):
        win = _FakeWin()
        win._fc_mission = {'sysid': 1, 'items': _good_mission().items}
        filedialog_stub['save'] = ('', '')
        win.on_sitl_save_wp_file()
        assert not qmb_stub['calls']['critical']


# ══════════════════════════════════════════════════════════════════════
#  I. DroneDetailPanel「↥」→ _on_detail_panel_write_wp
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestDetailPanelWriteWpRouting:
    def test_routes_to_correct_single_link(self, qmb_stub):
        l1, l3 = _FakeLink(sysid=1), _FakeLink(sysid=3)
        win = _FakeWin(sitl_links=[l1, l3])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win._on_detail_panel_write_wp('UAV-3')
        assert l1.uploaded == []
        assert len(l3.uploaded) == 1

    def test_never_bypasses_confirmation(self, qmb_stub):
        """即使走面板這條捷徑，Cancel 一樣什麼都不送。"""
        l3 = _FakeLink(sysid=3)
        win = _FakeWin(sitl_links=[l3])
        win._loaded_mission = _good_mission()
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Cancel
        win._on_detail_panel_write_wp('UAV-3')
        assert l3.uploaded == []

    def test_malformed_callsign_does_not_crash(self, qmb_stub):
        win = _FakeWin(sitl_links=[_FakeLink(sysid=1)])
        win._loaded_mission = _good_mission()
        win._on_detail_panel_write_wp('not-a-valid-callsign-format')
        # 'format' 取代 sysid 解析會丟 ValueError，必須被擋下並提示，不是崩潰
        assert qmb_stub['calls']['warning'] or qmb_stub['calls']['information']


# ══════════════════════════════════════════════════════════════════════
#  J. Write 鈕 enable/disable 邏輯本身
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestUpdateWriteWpEnabled:
    def test_disabled_with_no_links(self):
        win = _FakeWin(sitl_links=[])
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_wp.enabled is False

    def test_enabled_with_links_and_no_pending(self):
        win = _FakeWin(sitl_links=[_FakeLink(sysid=1)])
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_wp.enabled is True

    def test_disabled_while_pending_even_with_links(self):
        win = _FakeWin(sitl_links=[_FakeLink(sysid=1)])
        win._write_pending_sysids = {1}
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_wp.enabled is False

    def test_disconnect_paths_call_update_write_wp_enabled(self):
        """結構鎖：斷線相關的三條路徑（全域斷線 / 單機斷線 / HIL 全部斷線）都要
        呼叫 `_update_write_wp_enabled()`，否則鈕會卡在斷線前的狀態
        （例如卡在『寫入中』的停用樣式，即使該連線早就死了）。"""
        src = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8')
        tree = ast.parse(src)
        for fn_name in ('on_sitl_disconnect', '_on_sitl_link_disconnected',
                         '_on_hil_disconnect_all'):
            fn = next((n for n in ast.walk(tree)
                       if isinstance(n, ast.FunctionDef) and n.name == fn_name), None)
            assert fn is not None, f'找不到 {fn_name}'
            calls = {c.func.attr for c in ast.walk(fn)
                     if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}
            assert '_update_write_wp_enabled' in calls, (
                f'{fn_name} 沒有呼叫 _update_write_wp_enabled()')

    def test_hil_disconnect_all_clears_pending_sysid_for_the_disconnected_link(self):
        """⚠️ finding（major，ui_safety）：`_on_hil_disconnect_all` 在清理前先
        切斷 `link.disconnected`（避免逐機清理 handler 重入），這代表
        `_on_sitl_link_disconnected` 不會為這些 HIL link 執行——它原本負責的
        『把該機從待回報集合裡拿掉』這件事，這裡要自己補做，否則一筆寫入還沒
        回報就被使用者從 HIL 面板整批斷線，那個 sysid 會永遠卡在
        `_write_pending_sysids` 裡，連帶讓所有機的『寫入飛控』鈕永遠停用。"""
        link1 = _HilLink(sysid=3)
        link2 = _HilLink(sysid=4)
        win = _FakeWin(sitl_links=[link1, link2], hil_links=[link1, link2])
        # 模擬「對 sysid=3 的寫入已經送出、還沒等到 mission_uploaded 回報」。
        win._write_pending_sysids = {3}
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_wp.enabled is False, '前置條件：寫入中應該是停用的'

        win._on_hil_disconnect_all()

        assert 3 not in win._write_pending_sysids, (
            '斷線的那台（sysid=3）本來在等的回報永遠不會來了，'
            '待回報集合要把它拿掉，不然鈕會永遠卡住')
        assert win._hil_links == []
        assert win._sitl_links == []
        assert link1.stopped and link2.stopped
        assert win.hil_panel.disconnected_calls == 1
        # 清乾淨之後鈕應該恢復可按（沒有連線也沒有待回報）……不對，這裡是
        # 0 條連線 ⇒ enabled 的第一個條件（bool(self._sitl_links)）本身就是
        # False，鈕仍然是停用，但**理由**要變成「沒有連線」而不是「還在等
        # sysid=3 的回報」——用 `_write_pending_sysids` 是否清空來分辨這兩種
        # 停用原因，而不是只看 setEnabled 的最終布林值。
        assert win.sitl_hud.btn_write_wp.enabled is False

    def test_hil_disconnect_all_does_not_touch_unrelated_pending_sysid(self):
        """只斷開 HIL 鏈路那幾台——留在 `_sitl_links`（例如內建 SITL）裡、
        沒被斷線的其他 sysid 的待回報狀態不該被動到。"""
        hil_link = _HilLink(sysid=5)
        sitl_link = _FakeLink(sysid=9)
        win = _FakeWin(sitl_links=[hil_link, sitl_link], hil_links=[hil_link])
        win._write_pending_sysids = {5, 9}

        win._on_hil_disconnect_all()

        assert win._write_pending_sysids == {9}
        assert win._sitl_links == [sitl_link]


@qt_only
class TestInterruptedWriteOnLinkDisconnect:
    """⚠️ finding（ui_safety）：一筆寫入還沒等到 `mission_uploaded` 回報，
    連線就斷了（`_on_sitl_link_disconnected`）——飛控端可能只寫進一半的任務，
    這件事必須跟 `_on_mission_uploaded` 的失敗分支一樣大聲，不能只留一行
    通用的斷線狀態列文字讓使用者以為只是單純斷線。"""

    def test_interrupted_write_shows_a_loud_warning(self, qmb_stub):
        link = _FakeLink(sysid=6)
        win = _FakeWin(sitl_links=[link])
        win._write_pending_sysids = {6}

        win._on_sitl_link_disconnected(link, '訊號逾時')

        assert 6 not in win._write_pending_sysids
        assert len(qmb_stub['calls']['critical']) == 1, (
            '寫入被中斷應該跳出 critical 對話框，不能只是狀態列文字')
        title, msg = qmb_stub['calls']['critical'][0][0][1:3]
        assert 'UAV-6' in msg and '沒有被可靠地完成' in msg

    def test_plain_disconnect_without_pending_write_stays_quiet(self, qmb_stub):
        """對照組：沒有寫入在跑的正常斷線，不該多跳出這個對話框——
        否則使用者每次斷線都會被無意義的警告轟炸，久了就不會看。"""
        link = _FakeLink(sysid=7)
        win = _FakeWin(sitl_links=[link])
        win._write_pending_sysids = set()

        win._on_sitl_link_disconnected(link, '使用者手動斷線')

        assert qmb_stub['calls']['critical'] == []

    def test_interrupted_write_warning_does_not_fire_for_a_different_sysid(self, qmb_stub):
        """待回報集合裡的是別台飛控，這台自己斷線不該誤觸發。"""
        link6 = _FakeLink(sysid=6)
        link7 = _FakeLink(sysid=7)
        win = _FakeWin(sitl_links=[link6, link7])
        win._write_pending_sysids = {6}

        win._on_sitl_link_disconnected(link7, '訊號逾時')

        assert qmb_stub['calls']['critical'] == []
        assert win._write_pending_sysids == {6}, '沒斷線的那台的待回報狀態不該被動到'


# ══════════════════════════════════════════════════════════════════════
#  K. logger f-string 規則（本次新增/碰到的方法）
# ══════════════════════════════════════════════════════════════════════
class TestLoggerCallsArePreFormatted:
    """同 tests/test_mission_download.py::TestLoggerCallsArePreFormatted 的規則：
    Logger 的簽章是 `(message: str, exc_info: bool)`，不吃 %-args。"""

    TARGETS = {
        'ui/main_window.py': [
            'on_sitl_load_wp_file', 'on_sitl_write_wp', '_on_mission_uploaded',
            'on_sitl_save_wp_file', '_on_detail_panel_write_wp',
        ],
        'ui/widgets/drone_detail_panel.py': ['_on_upload_wp'],
    }

    @staticmethod
    def _bad_calls_in(path, func_name):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        fn = next((n for n in ast.walk(tree)
                   if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                   and n.name == func_name), None)
        if fn is None:
            return None
        bad = []
        for node in ast.walk(fn):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == 'logger'
                    and len(node.args) > 1):
                bad.append((node.lineno, node.func.attr, len(node.args)))
        return bad

    @pytest.mark.parametrize('relpath,func', [
        (rp, fn) for rp, fns in TARGETS.items() for fn in fns
    ])
    def test_no_percent_args(self, relpath, func):
        path = ROOT / relpath
        bad = self._bad_calls_in(path, func)
        assert bad is not None, f'{relpath} 找不到函式 {func}'
        assert not bad, f'{relpath}::{func} 的 logger 呼叫傳了多個位置參數：{bad}'

    def test_the_check_can_actually_fire(self):
        """⚠️ 證明這條檢查抓得到東西 —— 一個永遠回空清單的檢查和一個
        『全部通過』的檢查在報表上一模一樣。"""
        import tempfile
        with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False,
                                         encoding='utf-8') as fh:
            fh.write('def f(self):\n    logger.info("x %s", y)\n')
            tmp = pathlib.Path(fh.name)
        try:
            bad = self._bad_calls_in(tmp, 'f')
            assert bad == [(2, 'info', 2)]
        finally:
            tmp.unlink(missing_ok=True)


# ══════════════════════════════════════════════════════════════════════
#  L. __init__ 初始化
# ══════════════════════════════════════════════════════════════════════
class TestInitState:
    def test_loaded_mission_initialized_next_to_fc_mission(self):
        """finding（minor，test_bite）：舊版用整檔 substring 找
        'self._loaded_mission = None' / "self._fc_mission: dict | None = None"
        這兩段文字，違反本 repo 自己的規矩（見 5(e)：結構鎖要用 AST，不能用
        substring——docstring/註解會誤判）。任何地方（甚至一段解釋用的註解）
        只要照順序出現同樣的文字就會讓這條測試在真正的宣告被刪掉或搬遠之後
        仍然通過。

        改成 AST：找到真正賦值的 Assign/AnnAssign 陳述式，而且限定在它們
        實際所在的函式（`create_control_panel`——AST 走過整個檔案確認過，
        這兩行並不在 `__init__` 本體，是 `__init__` 呼叫的
        `create_control_panel` 裡；如果之後真的搬進 `__init__`，這裡一樣
        找得到，因為找的是「哪個函式含有這兩個陳述式」而不是寫死函式名）。
        """
        tree = ast.parse((ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8'))
        cls = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.ClassDef) and n.name == 'MainWindow')

        def _self_attr_assign_targets(stmt):
            """回傳這個陳述式裡所有 `self.<attr> = ...` / `self.<attr>: T = ...`
            的 attr 名稱（AST 層級，不管右手邊的值、不管周圍有沒有註解）。"""
            if isinstance(stmt, ast.Assign):
                targets = stmt.targets
            elif isinstance(stmt, ast.AnnAssign):
                targets = [stmt.target]
            else:
                return []
            names = []
            for t in targets:
                if (isinstance(t, ast.Attribute)
                        and isinstance(t.value, ast.Name) and t.value.id == 'self'):
                    names.append(t.attr)
            return names

        # 在 MainWindow 的每個方法裡找「同一個函式本體內，兩個陳述式都真的
        # 賦值到 self._fc_mission 跟 self._loaded_mission」——不寫死函式名，
        # 這樣「兩者搬去哪個函式」不是這條測試在鎖的東西，「兩者宣告在一起」
        # 才是。
        found = None
        for fn in ast.walk(cls):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            idx_fc = idx_loaded = None
            for i, stmt in enumerate(fn.body):
                names = _self_attr_assign_targets(stmt)
                if '_fc_mission' in names:
                    idx_fc = i
                if '_loaded_mission' in names:
                    idx_loaded = i
            if idx_fc is not None and idx_loaded is not None:
                found = (fn.name, idx_fc, idx_loaded)
                break

        assert found is not None, (
            'MainWindow 裡找不到任何一個函式，在其本體內同時對 '
            'self._fc_mission 和 self._loaded_mission 做真正的（AST 層級）'
            '賦值——這兩個狀態必須初始化在一起（見 sitl_link.py 對「兩者用途'
            '相反」的說明），不能只是原始碼裡某處剛好有這兩段文字。')
        fn_name, idx_fc, idx_loaded = found
        assert 0 < (idx_loaded - idx_fc) <= 3, (
            f'_loaded_mission（第 {idx_loaded} 個陳述式）沒有宣告在 '
            f'_fc_mission（第 {idx_fc} 個陳述式）附近（{fn_name} 內）')

    def test_ast_check_is_not_fooled_by_a_stray_comment(self):
        """證明上面那條測試真的在看 AST、不是又繞回 substring：塞一段跟原本
        宣告文字一模一樣、但只是註解（不是真正賦值）的假檔案，AST 版本要
        判定「找不到」，而不是被文字騙過去。"""
        import tempfile
        fake_src = (
            "class MainWindow:\n"
            "    def __init__(self):\n"
            "        pass\n"
            "\n"
            "    def unrelated(self):\n"
            "        # self._fc_mission: dict | None = None\n"
            "        # self._loaded_mission = None\n"
            "        return 1\n"
        )
        tree = ast.parse(fake_src)
        cls = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.ClassDef) and n.name == 'MainWindow')

        def _self_attr_assign_targets(stmt):
            if isinstance(stmt, ast.Assign):
                targets = stmt.targets
            elif isinstance(stmt, ast.AnnAssign):
                targets = [stmt.target]
            else:
                return []
            names = []
            for t in targets:
                if (isinstance(t, ast.Attribute)
                        and isinstance(t.value, ast.Name) and t.value.id == 'self'):
                    names.append(t.attr)
            return names

        found = None
        for fn in ast.walk(cls):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            idx_fc = idx_loaded = None
            for i, stmt in enumerate(fn.body):
                names = _self_attr_assign_targets(stmt)
                if '_fc_mission' in names:
                    idx_fc = i
                if '_loaded_mission' in names:
                    idx_loaded = i
            if idx_fc is not None and idx_loaded is not None:
                found = (fn.name, idx_fc, idx_loaded)
                break

        assert found is None, (
            '假檔案裡兩行都只是註解，AST 版本卻宣稱找到了真正的賦值——'
            '代表這條檢查其實還是在被文字騙，等於白改')
