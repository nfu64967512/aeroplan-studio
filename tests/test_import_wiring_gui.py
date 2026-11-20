# -*- coding: utf-8 -*-
"""tests/test_import_wiring_gui.py — 「多檔匯入 → 每架一份 → 逐機寫入」的 UI 接線。

被測的是 ui/main_window.py 的 `_apply_imported_missions` / `_preview_imported_missions` /
`on_sitl_write_imported` / `on_sitl_write_wp(mission=…)` 與 SITLHud 的兩顆鈕、兩個訊號。
規則本體（載入≠指派、問題清單）在 mission/import_missions.py，自己有測試，這裡只用真貨。

═══ 這個檔在防什麼 ═══
① **UAV-1 拿到 UAV-2 的檔**：逐機寫入必須依 `sysid_label` 配對，不看 HUD「指令對象」。
② **指派了但沒連線的 sysid 被寫到別台**：必須列出、而且 0 筆寫入。
③ **對話框擋過的事主視窗就不驗**：`_apply_imported_missions` 要再跑一次 `assignment_problems`。
④ **預覽重建圍籬**：只准走 `display_fc_mission(sysid=…)`，不准 display_path*。
⑤ **換一批匯入把「讀回飛控」那層也清掉**：只清舊匯入的 sysid 層。
⑥ **MainWindow 在本環境不能 New**（QtWebEngine 匯入順序）⇒ unbound-method + 假 self。
"""
from __future__ import annotations

import os
import pathlib
import types

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

ROOT = pathlib.Path(__file__).resolve().parents[1]

try:
    from ui.main_window import MainWindow
    import PyQt6.QtWidgets as _QtW
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    from ui.widgets.sitl_hud import SITLHud
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'ui.main_window / PyQt6 匯入失敗: {globals().get("_QT_ERR")}')

from mission.import_missions import ImportedMission, assignment_problems  # noqa: E402
from mission.waypoint_file import parse_wpl_text  # noqa: E402


def _wpl(lat0: float) -> str:
    return (
        "QGC WPL 110\n"
        f"0\t1\t0\t16\t0\t0\t0\t0\t{lat0:.7f}\t121.0000000\t0.0\t1\n"
        f"1\t0\t3\t22\t0\t0\t0\t0\t{lat0 + 0.001:.7f}\t121.0010000\t50.0\t1\n"
        f"2\t0\t3\t16\t0\t0\t0\t0\t{lat0 + 0.002:.7f}\t121.0020000\t50.0\t1\n"
        f"3\t0\t3\t21\t0\t0\t0\t0\t{lat0 + 0.003:.7f}\t121.0030000\t0.0\t1\n"
    )


def _imported(sysid, lat0=25.0, name=None):
    name = name or f'UAV-{sysid}.waypoints'
    path = f'C:/tmp/{name}'
    lm = parse_wpl_text(_wpl(lat0), source=path)
    assert not lm.has_errors
    return ImportedMission(path=path, sysid=sysid, loaded=lm, label=name,
                           suggested_sysid=sysid)


# ── 假 self / 假替身（house style，同 tests/test_mission_write.py）──────────
class _Button:
    def __init__(self):
        self.enabled = True
        self.history = []

    def setEnabled(self, v):
        self.enabled = bool(v)
        self.history.append(bool(v))


class _StatusBar:
    def __init__(self):
        self.messages = []

    def showMessage(self, text, timeout=0):
        self.messages.append(text)


class _Hud:
    def __init__(self):
        self.btn_write_wp = _Button()
        self.btn_write_imported = _Button()


class _MapWidget:
    def __init__(self):
        self.calls = []

    def display_fc_mission(self, *a, **k):
        self.calls.append(('display_fc_mission', a, k))
        return {'layer': f"uav{k.get('sysid')}"}

    def clear_fc_mission(self, sysid=None, kind=None):
        self.calls.append(('clear_fc_mission', (), {'sysid': sysid, 'kind': kind}))
        return {'2d': True, '3d': None}

    def display_path(self, *a, **k):
        self.calls.append(('display_path', a, k))

    def display_paths(self, *a, **k):
        self.calls.append(('display_paths', a, k))

    def display_fw_paths(self, *a, **k):
        self.calls.append(('display_fw_paths', a, k))


class _FakeLink:
    def __init__(self, sysid=1, conn_str='udpin:0.0.0.0:14550'):
        self.sysid_label = sysid
        self.conn_str = conn_str
        self.uploaded = []

    def get_latest_telemetry(self):
        return None

    def upload_mission_items(self, items, verify=True):
        self.uploaded.append((items, verify))


_BOUND = (
    '_run_import_dialog', 'on_import_waypoints', '_apply_imported_missions',
    '_preview_imported_missions', 'on_sitl_write_imported', 'on_sitl_write_wp',
    '_update_write_wp_enabled', '_on_mission_uploaded',
)


class _FakeWin:
    def __init__(self, sitl_links=None, target_links=None):
        self.map_widget = _MapWidget()
        self.sitl_hud = _Hud()
        self._status = _StatusBar()
        self._loaded_mission = None
        self._fc_mission = None
        self._write_pending_sysids = set()
        self._last_wp_dir = ''
        self._imported_missions = {}
        self._sitl_links = list(sitl_links or [])
        self._target = list(target_links or [])
        for name in _BOUND:
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))

    def statusBar(self):
        return self._status

    def _target_links(self):
        return list(self._target)


@pytest.fixture
def qmb_stub(monkeypatch):
    QMB = _QtW.QMessageBox
    calls = {'information': [], 'warning': [], 'critical': [], 'question': []}
    ret = {'value': QMB.StandardButton.Yes}

    def mk(name):
        def _f(*a, **k):
            calls[name].append((a, k))
            return ret['value']
        return staticmethod(_f)

    for name in calls:
        monkeypatch.setattr(QMB, name, mk(name))
    return {'calls': calls, 'ret': ret, 'QMB': QMB}


# ══════════════════════════════════════════════════════════════════════
#  A. HUD：兩顆鈕、兩個訊號；主視窗接線（源碼鎖）
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestHudAndWiring:
    def test_buttons_and_signals(self):
        h = SITLHud()
        assert hasattr(h, 'btn_import_wps') and hasattr(h, 'btn_write_imported')
        assert h.btn_write_imported.isEnabled() is False, '沒連線、沒匯入就不該可按'
        assert h.btn_import_wps.isEnabled() is True, '匯入是純讀檔，不需要連線'
        got = []
        h.cmd_import_wps.connect(lambda: got.append('import'))
        h.cmd_write_imported.connect(lambda: got.append('write'))
        h.btn_import_wps.click()
        h.btn_write_imported.setEnabled(True)
        h.btn_write_imported.click()
        assert got == ['import', 'write']

    def test_tooltips_state_the_contract(self):
        h = SITLHud()
        t_imp = h.btn_import_wps.toolTip()
        t_wr = h.btn_write_imported.toolTip()
        assert '不會自動寫入' in t_imp and '指派' in t_imp
        assert '對應 sysid' in t_wr and '確認對話框' in t_wr and '沒連線' in t_wr

    @staticmethod
    def _hud_connects():
        """AST：MainWindow 原始碼裡所有 `self.sitl_hud.<sig>.connect(<target>)` → {sig: [target 原始碼]}。
        用 AST 而不是子字串：被註解掉的那一行子字串照樣找得到（審查 2026-10-17 抓到）。"""
        import ast
        src = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8', errors='replace')
        out = {}
        for node in ast.walk(ast.parse(src)):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'connect' and isinstance(node.func.value, ast.Attribute)
                    and ast.unparse(node.func.value.value) == 'self.sitl_hud'):
                sig = node.func.value.attr
                out.setdefault(sig, []).append(ast.unparse(node.args[0]) if node.args else '')
        return out

    def test_main_window_connects_the_signals_exactly_once(self):
        con = self._hud_connects()
        assert con, 'AST 一條 HUD 接線都沒抓到 ⇒ 這條測試自己壞了'
        expect = {
            'cmd_import_wps': 'self.on_import_waypoints',
            'cmd_write_imported': 'self.on_sitl_write_imported',
            'cmd_log_download': 'self._start_log_download',
            'cmd_log_download_all': 'self._start_log_download_all',
            'cmd_log_cancel': 'self._cancel_log_download',
        }
        for sig, target in expect.items():
            assert con.get(sig) == [target], f'{sig} 接到 {con.get(sig)}（要恰好一次、接 {target}）'

    def test_menu_action_is_wired(self):
        import ast
        src = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8', errors='replace')
        assert '匯入航點檔（多檔 → 每架一份）' in src
        n = sum(1 for node in ast.walk(ast.parse(src))
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'connect'
                and ast.unparse(node.func.value) == 'action.triggered'
                and node.args and ast.unparse(node.args[0]) == 'self.on_import_waypoints')
        assert n == 1, f'選單動作接了 {n} 次（要恰好 1 次）'


# ══════════════════════════════════════════════════════════════════════
#  B. 收下指派結果 → 預覽
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestApplyAndPreview:
    def test_two_missions_two_layers_fit_only_first(self, qmb_stub):
        l1 = _FakeLink(1)
        win = _FakeWin(sitl_links=[l1])
        ms = [_imported(2, 25.1), _imported(1, 25.0)]
        out = win._apply_imported_missions(ms)
        assert sorted(out) == [1, 2] and sorted(win._imported_missions) == [1, 2]
        disp = [c for c in win.map_widget.calls if c[0] == 'display_fc_mission']
        assert [c[2]['sysid'] for c in disp] == [1, 2], '要依 sysid 排序、每架一層'
        assert [c[2]['fit'] for c in disp] == [True, False]
        assert all(c[2].get('kind') == 'import' for c in disp), \
            '匯入預覽要用 kind="import"（鍵 uav<n>-import），才不會跟同一架的「讀回」層互相蓋掉'
        for c in disp:
            path2d = c[1][0]
            assert path2d and all(len(p) == 2 for p in path2d), '2D 只收二元組'
            assert c[2]['home'] is not None
            assert 'UAV-' in c[2]['label']
        assert not [c for c in win.map_widget.calls if c[0].startswith('display_path')
                    or c[0] == 'display_fw_paths'], '預覽不准走會重建圍籬的 display_path*'
        assert win._last_wp_dir == 'C:/tmp'
        assert win.sitl_hud.btn_write_imported.enabled is True
        assert any('尚未連線：UAV-2' in m for m in win._status.messages)
        assert not qmb_stub['calls']['critical']

    def test_problems_block_everything_and_keep_previous_batch(self, qmb_stub):
        win = _FakeWin()
        prev = {3: _imported(3)}
        win._imported_missions = dict(prev)
        bad = [_imported(1), _imported(1, 25.2, name='dup_uav1.waypoints')]   # 重複 sysid
        assert assignment_problems(bad)
        out = win._apply_imported_missions(bad)
        assert out == {}
        assert win._imported_missions == prev, '被擋下就不可以動上一批'
        assert win.map_widget.calls == [], '被擋下就不可以畫、也不可以清'
        assert qmb_stub['calls']['critical']
        unassigned = [_imported(1)]
        unassigned[0].sysid = None
        assert win._apply_imported_missions(unassigned) == {}
        assert win._apply_imported_missions([]) == {}
        assert len(qmb_stub['calls']['critical']) == 3

    def test_new_batch_clears_only_old_imported_layers(self, qmb_stub):
        win = _FakeWin()
        win._imported_missions = {3: _imported(3), 4: _imported(4)}
        win._apply_imported_missions([_imported(1)])
        clears = [c for c in win.map_widget.calls if c[0] == 'clear_fc_mission']
        assert sorted(c[2]['sysid'] for c in clears) == [3, 4]
        assert all(c[2]['sysid'] is not None and c[2]['kind'] == 'import' for c in clears), \
            'sysid=None 會把全部層清掉；不帶 kind 會連同一架的「📥 讀回」層一起清掉'
        assert sorted(win._imported_missions) == [1]

    def test_one_broken_preview_does_not_stop_the_others(self, qmb_stub, monkeypatch):
        win = _FakeWin()
        orig = win.map_widget.display_fc_mission

        def boom(*a, **k):
            if k.get('sysid') == 1:
                raise RuntimeError('2D 炸了')
            return orig(*a, **k)
        win.map_widget.display_fc_mission = boom
        out = win._apply_imported_missions([_imported(1), _imported(2)])
        assert sorted(out) == [1, 2]
        assert any('只畫出 1/2' in m for m in win._status.messages)

    def test_run_import_dialog_signature_matches_the_real_dialog(self, monkeypatch, tmp_path):
        """主視窗呼叫對話框的參數名要跟 ui/dialogs/import_waypoints_dialog.py 真的一致，
        而且先選檔：QFileDialog 回空 ⇒ None、不開對話框；回檔案 ⇒ 用那些檔開、回 result()。"""
        import ui.dialogs.import_waypoints_dialog as dlg_mod
        from ui.dialogs.import_waypoints_dialog import ImportWaypointsDialog
        f1 = tmp_path / 'UAV-1.waypoints'
        f1.write_text(_wpl(25.0), encoding='utf-8')
        # 真對話框：用主視窗那組關鍵字參數建得起來、result() 在沒按確定時是空清單
        real = ImportWaypointsDialog(None, paths=[str(f1)], links_sysids=[1, 2], last_dir=str(tmp_path))
        assert real.result() == [] and len(real.missions()) == 1
        real.deleteLater()

        win = _FakeWin(sitl_links=[_FakeLink(2)])
        picks = {'ret': ([], ''), 'calls': []}

        def _open_names(*a, **k):
            picks['calls'].append((a, k))
            return picks['ret']
        monkeypatch.setattr(_QtW.QFileDialog, 'getOpenFileNames', staticmethod(_open_names))
        made = []

        class _Dlg:
            def __init__(self, parent, paths=(), links_sysids=(), last_dir=''):
                made.append({'paths': list(paths), 'links': list(links_sysids), 'last_dir': last_dir})
                self._res = [_imported(1)]

            def exec(self):
                return 1

            def result(self):
                return self._res
        monkeypatch.setattr(dlg_mod, 'ImportWaypointsDialog', _Dlg)

        assert win._run_import_dialog() is None, '一個檔都沒選就是取消'
        assert made == [], '沒選檔不可以開對話框'
        picks['ret'] = ([str(f1)], '')
        win._last_wp_dir = 'C:/somewhere'
        out = win._run_import_dialog()
        assert [m.sysid for m in out] == [1]
        assert made == [{'paths': [str(f1)], 'links': [2], 'last_dir': 'C:/somewhere'}]
        assert picks['calls'][-1][0][2] == 'C:/somewhere', '檔案對話框要從上次目錄開'

    def test_readback_draws_its_own_sysid_layer_so_imported_previews_survive(self, qmb_stub):
        """📥 從飛控讀回改成每架一層：讀回 UAV-2 不可以把 UAV-1 的匯入預覽清掉
        （2D/3D 的 sysid=None 語意是「先清全部層」）。"""
        win = _FakeWin()
        win._on_mission_downloaded = types.MethodType(MainWindow._on_mission_downloaded, win)
        items = _imported(2, 25.3).items
        win._on_mission_downloaded(2, list(items))
        disp = [c for c in win.map_widget.calls if c[0] == 'display_fc_mission']
        assert len(disp) == 1 and disp[0][2]['sysid'] == 2
        assert '讀回' in disp[0][2]['label'] and 'UAV-2' in disp[0][2]['label']
        assert disp[0][2]['home'] is not None
        assert win._fc_mission == {'sysid': 2, 'items': list(items)}
        assert not [c for c in win.map_widget.calls if c[0] == 'clear_fc_mission']

    def test_clear_all_drops_the_imported_batch_with_the_previews(self, qmb_stub):
        """「清除全部」把地圖上的匯入預覽清掉了 ⇒ _imported_missions 也要清、鈕要跟著暗，
        不然寫的是畫面上已經看不到的東西（審查 2026-10-17）。"""
        l1 = _FakeLink(1)
        win = _FakeWin(sitl_links=[l1])
        win.on_clear_all_silent = types.MethodType(MainWindow.on_clear_all_silent, win)
        win.on_clear_corners = lambda: None
        win.on_clear_paths = lambda: None
        win.obstacles = []
        win.nfz_zones = []
        win.map_widget.clear_all = lambda: win.map_widget.calls.append(('clear_all', (), {}))
        win._imported_missions = {1: _imported(1)}
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_imported.enabled is True
        win.on_clear_all_silent()
        assert win._imported_missions == {}
        assert win.sitl_hud.btn_write_imported.enabled is False
        assert ('clear_all', (), {}) in win.map_widget.calls

    def test_single_file_load_uses_its_own_layer_kind(self, qmb_stub, monkeypatch, tmp_path):
        """📂 載入航點檔（單檔）要畫在自己的 'loaded' 層：不帶 kind 的 sysid=None 會先清掉全部層
        （含其他架的匯入預覽），而 _imported_missions 還留著（審查 2026-10-17）。"""
        f1 = tmp_path / 'single.waypoints'
        f1.write_text(_wpl(25.0), encoding='utf-8')
        monkeypatch.setattr(_QtW.QFileDialog, 'getOpenFileName', staticmethod(lambda *a, **k: (str(f1), '')))
        win = _FakeWin()
        win.on_sitl_load_wp_file = types.MethodType(MainWindow.on_sitl_load_wp_file, win)
        win.on_sitl_load_wp_file()
        disp = [c for c in win.map_widget.calls if c[0] == 'display_fc_mission']
        assert len(disp) == 1
        assert disp[0][2].get('kind') == 'loaded' and disp[0][2].get('sysid') is None
        assert win._loaded_mission is not None

    def test_on_import_waypoints_uses_dialog_result(self, qmb_stub, monkeypatch):
        win = _FakeWin()
        win._run_import_dialog = lambda: None
        win.on_import_waypoints()
        assert win._imported_missions == {} and win.map_widget.calls == []
        win._run_import_dialog = lambda: [_imported(7)]
        win.on_import_waypoints()
        assert sorted(win._imported_missions) == [7]


# ══════════════════════════════════════════════════════════════════════
#  C. 逐機寫入
# ══════════════════════════════════════════════════════════════════════
@qt_only
class TestWriteImported:
    def test_each_sysid_gets_its_own_file_hud_target_ignored(self, qmb_stub):
        l1, l2 = _FakeLink(1), _FakeLink(2)
        # HUD 指令對象故意只選 UAV-2：逐機寫入不可以看它
        win = _FakeWin(sitl_links=[l1, l2], target_links=[l2])
        m1, m2, m5 = _imported(1, 25.0), _imported(2, 25.5), _imported(5, 26.0)
        win._imported_missions = {1: m1, 2: m2, 5: m5}
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win.on_sitl_write_imported()
        assert len(l1.uploaded) == 1 and len(l2.uploaded) == 1
        assert l1.uploaded[0][0] == list(m1.items) and l1.uploaded[0][1] is True
        assert l2.uploaded[0][0] == list(m2.items)
        assert l1.uploaded[0][0] != l2.uploaded[0][0], '兩架拿到同一份任務'
        assert win._loaded_mission is None, 'mission= 路徑不可以碰 _loaded_mission'
        infos = [a[0][2] for a in qmb_stub['calls']['information']]
        assert any('UAV-5' in t and '沒有寫入' in t for t in infos), '沒連線的 sysid 要列出'
        assert len(qmb_stub['calls']['question']) == 2, '每架各跳一次確認'
        assert win._write_pending_sysids == {1, 2}

    def test_two_links_with_the_same_sysid_are_named_not_blamed_on_the_hud(self, qmb_stub):
        """兩條連線都掛 sysid 2（HIL 兩列填一樣）：不可以丟給 on_sitl_write_wp 的「多於一台、去點卡片」
        （這條路不看卡片，點了也沒用），要把真正原因（哪兩條連線）講出來、0 筆寫入。"""
        a, b = _FakeLink(2, 'COM7,115200'), _FakeLink(2, 'COM9,115200')
        l1 = _FakeLink(1)
        win = _FakeWin(sitl_links=[l1, a, b])
        win._imported_missions = {1: _imported(1), 2: _imported(2)}
        qmb_stub['ret']['value'] = _QtW.QMessageBox.StandardButton.Yes
        win.on_sitl_write_imported()
        assert len(l1.uploaded) == 1 and a.uploaded == [] and b.uploaded == []
        assert not qmb_stub['calls']['warning'], '不可以走到 on_sitl_write_wp 的「多於一台」警告'
        infos = [x[0][2] for x in qmb_stub['calls']['information']]
        assert any('UAV-2' in t and 'COM7' in t and 'COM9' in t and '沒有寫入' in t for t in infos), infos

    def test_cancel_skips_only_that_aircraft(self, qmb_stub):
        l1, l2 = _FakeLink(1), _FakeLink(2)
        win = _FakeWin(sitl_links=[l1, l2])
        win._imported_missions = {1: _imported(1), 2: _imported(2)}
        answers = iter([_QtW.QMessageBox.StandardButton.Cancel,
                        _QtW.QMessageBox.StandardButton.Yes])

        def q(*a, **k):
            qmb_stub['calls']['question'].append((a, k))
            return next(answers)
        _QtW.QMessageBox.question = staticmethod(q)
        win.on_sitl_write_imported()
        assert l1.uploaded == [] and len(l2.uploaded) == 1

    def test_nothing_imported_is_a_message_not_a_write(self, qmb_stub):
        l1 = _FakeLink(1)
        win = _FakeWin(sitl_links=[l1])
        win.on_sitl_write_imported()
        assert l1.uploaded == [] and qmb_stub['calls']['information']

    def test_pending_write_on_that_sysid_is_refused(self, qmb_stub):
        l1 = _FakeLink(1)
        win = _FakeWin(sitl_links=[l1])
        win._imported_missions = {1: _imported(1)}
        win._write_pending_sysids.add(1)
        win.on_sitl_write_imported()
        assert l1.uploaded == []

    def test_write_wp_mission_param_beats_loaded_mission(self, qmb_stub):
        l1 = _FakeLink(1)
        win = _FakeWin(sitl_links=[l1], target_links=[l1])
        win._loaded_mission = _imported(1, 25.0).loaded
        other = _imported(1, 26.0).loaded
        win.on_sitl_write_wp(target_links=[l1], mission=other)
        assert l1.uploaded[0][0] == list(other.items)
        assert l1.uploaded[0][0] != list(win._loaded_mission.items)
        # 沒帶 mission= 就還是舊行為
        win._write_pending_sysids.clear()
        win.on_sitl_write_wp()
        assert l1.uploaded[1][0] == list(win._loaded_mission.items)

    def test_write_imported_button_enablement(self):
        l1 = _FakeLink(1)
        win = _FakeWin(sitl_links=[l1])
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_imported.enabled is False, '沒匯入就不可按'
        win._imported_missions = {1: _imported(1)}
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_imported.enabled is True
        win._write_pending_sysids.add(1)
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_imported.enabled is False, '有寫入在等回報就不可按'
        win._write_pending_sysids.clear()
        win._sitl_links = []
        win._update_write_wp_enabled()
        assert win.sitl_hud.btn_write_imported.enabled is False, '沒連線就不可按'
