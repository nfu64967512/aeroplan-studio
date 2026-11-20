# -*- coding: utf-8 -*-
"""tests/test_strike_ui_gate.py — 蜂群攻擊那一系（投放／STOT／打擊 BDA／嵌入式蜂群）先從 UI 拿掉、管線先斷開。

使用者 2026-10-17 定調。做法是**一個開關**（config/features.STRIKE_ENABLED，預設 False，
`AEROPLAN_STRIKE_UI=1` 暫時開）：widget 與 mixin 方法都還在（既有測試照跑），只有「接線」與「進版面」看開關。

鎖的是：
① 主視窗裡每一條攻擊信號的 connect 都在 `if _features.strike_enabled():` 的 **body**（AST；守衛式要**恰好等於**
   `_features.strike_enabled()`，不是子字串——反向、else 分支、`or True`、elif 都不算；審查 2026-10-17 抓到）。
② STOT 儀表板的建構、工具列「蜂群打擊」鈕同樣在那個守衛的 body 裡（同一個 AST 判定）。
③ 參數面板的「蜂群打擊」分頁預設不加（內容仍建、信號仍在）；開關打開才加。
④ 開關本身：預設 False、只認環境變數。
"""
from __future__ import annotations

import ast
import os
import pathlib

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

ROOT = pathlib.Path(__file__).resolve().parents[1]
MW = ROOT / 'ui' / 'main_window.py'

GATE_TEST = '_features.strike_enabled()'

GATED_CONNECTS = [
    'self.sitl_hud.connect_embedded_requested',
    'self.sitl_hud.cmd_release_store',
    'self.parameter_panel.strike_mark_targets_requested',
    'self.parameter_panel.strike_mark_base_requested',
    'self.parameter_panel.strike_mode_changed',
    'self.parameter_panel.strike_execute_requested',
    'self.parameter_panel.strike_clear_requested',
    'self.parameter_panel.strike_export_requested',
    'self.parameter_panel.strike_dtot_export_requested',
    'self.parameter_panel.strike_owa_parm_requested',
    'self.parameter_panel.strike_sitl_upload_requested',
    'self.parameter_panel.strike_recon_trigger_requested',
    'self.parameter_panel.strike_vtol_toggle_changed',
    'self.parameter_panel.strike_open_vtol_mission_planner_requested',
    'self.parameter_panel.strike_target_coord_changed',
    'self.parameter_panel.stot_ros2_run_requested',
    'self.parameter_panel.stot_ros2_abort_requested',
    'self.strike_ttt_dashboard.launch_requested',
    'self.strike_ttt_dashboard.terminal_sync_requested',
]


def _parents(tree):
    parents = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parents[child] = node
    return parents


def _in_gate_body(node, parents) -> bool:
    """node 是否位於某個 `if <GATE_TEST>:` 的 **body**（不是 orelse）之下。

    守衛式用 ast.unparse 完全相等比對；巢狀 def/lambda 不算（函式定義在守衛裡、呼叫在外面＝沒守住）。
    """
    child, p = node, parents.get(node)
    while p is not None:
        if isinstance(p, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)) and child is not node:
            # 往上穿過了一層函式定義 ⇒ 那層函式的守衛不算在這個呼叫頭上
            return False
        if isinstance(p, ast.If) and ast.unparse(p.test) == GATE_TEST:
            if any(child is s or _contains(s, child) for s in p.body):
                return True
            return False          # 在 orelse 裡 ⇒ 沒守住
        child, p = p, parents.get(p)
    return False


def _contains(stmt, node) -> bool:
    return any(n is node for n in ast.walk(stmt))


def _connect_calls(tree):
    """{signal 原始碼: [Call node, ...]} — 每個 `<sig>.connect(...)`。"""
    out = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'connect' and isinstance(node.func.value, ast.Attribute)):
            out.setdefault(ast.unparse(node.func.value), []).append(node)
    return out


class TestMainWindowWiringIsGated:
    def test_every_strike_connect_sits_in_the_gate_body(self):
        tree = ast.parse(MW.read_text(encoding='utf-8'))
        parents = _parents(tree)
        calls = _connect_calls(tree)
        missing = [s for s in GATED_CONNECTS if s not in calls]
        assert not missing, f'這些信號在 main_window 裡找不到 connect（改名了？）：{missing}'
        bad = [s for s in GATED_CONNECTS if not all(_in_gate_body(c, parents) for c in calls[s])]
        assert not bad, f'這些 connect 不在 `if {GATE_TEST}:` 的 body 裡：{bad}'

    @pytest.mark.parametrize('src, expect', [
        ('if _features.strike_enabled():\n    self.a.b.connect(self.c)\n', True),
        ('self.a.b.connect(self.c)\n', False),
        ('if not _features.strike_enabled():\n    self.a.b.connect(self.c)\n', False),            # 反向
        ('if _features.strike_enabled():\n    pass\nelse:\n    self.a.b.connect(self.c)\n', False),  # else 分支
        ('if _features.strike_enabled() or True:\n    self.a.b.connect(self.c)\n', False),        # or True
        ('if x:\n    pass\nelif _features.strike_enabled():\n    self.a.b.connect(self.c)\n', True),  # elif＝巢狀 If 的 body
        ('if _features.strike_enabled():\n    def f():\n        self.a.b.connect(self.c)\nf()\n', False),  # 定義在內、呼叫在外
        ('if _features.strike_enabled():\n    if y:\n        self.a.b.connect(self.c)\n', True),   # 巢狀仍算
    ])
    def test_the_check_can_fire(self, src, expect):
        """守衛偵測要能分辨反向／else／or True／巢狀 def（instrument-that-cannot-fire）。"""
        tree = ast.parse(src)
        parents = _parents(tree)
        calls = _connect_calls(tree)
        assert list(calls) == ['self.a.b']
        assert _in_gate_body(calls['self.a.b'][0], parents) is expect

    def test_dashboard_construction_and_toolbar_action_are_in_the_gate_body(self):
        tree = ast.parse(MW.read_text(encoding='utf-8'))
        parents = _parents(tree)
        ctor = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and ast.unparse(n.func).endswith('StrikeTTTDashboard')]
        assert ctor, '找不到 StrikeTTTDashboard(...) 的建構'
        assert all(_in_gate_body(c, parents) for c in ctor), 'STOT 儀表板建構不在開關 body 裡'
        acts = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == '_add_action'
                and any(isinstance(a, ast.Constant) and '蜂群打擊' in str(a.value) for a in n.args)]
        assert acts, '找不到工具列「蜂群打擊」鈕（改名了？）'
        assert all(_in_gate_body(a, parents) for a in acts), '工具列「蜂群打擊」鈕不在開關 body 裡'

    def test_feature_flag_is_imported_at_module_level(self):
        tree = ast.parse(MW.read_text(encoding='utf-8'))
        ok = any(isinstance(n, ast.ImportFrom) and n.module == 'config'
                 and any(a.name == 'features' and a.asname == '_features' for a in n.names)
                 for n in tree.body)
        assert ok, 'main_window 要在模組層 `from config import features as _features`'


class TestFeatureFlag:
    def test_default_is_off_and_env_turns_it_on(self, monkeypatch):
        import importlib
        from config import features
        monkeypatch.delenv('AEROPLAN_STRIKE_UI', raising=False)
        importlib.reload(features)
        assert features.STRIKE_ENABLED is False and features.strike_enabled() is False
        monkeypatch.setenv('AEROPLAN_STRIKE_UI', '1')
        importlib.reload(features)
        assert features.strike_enabled() is True
        monkeypatch.delenv('AEROPLAN_STRIKE_UI', raising=False)
        importlib.reload(features)
        assert features.strike_enabled() is False

    def test_settings_are_never_consulted(self, monkeypatch):
        """就算 config.get_settings 會炸，features 也照樣 import、照樣只看環境變數。"""
        import importlib
        import config
        from config import features
        monkeypatch.setattr(config, 'get_settings', lambda *a, **k: (_ for _ in ()).throw(RuntimeError('no')))
        monkeypatch.delenv('AEROPLAN_STRIKE_UI', raising=False)
        importlib.reload(features)
        assert features.strike_enabled() is False


try:
    import ui.main_window  # noqa: F401
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    from ui.widgets.parameter_panel import ParameterPanel
    _QT_OK = True
except Exception as _e:                       # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e


@pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')
class TestParameterPanelTab:
    @staticmethod
    def _tabs(panel):
        return [panel._tabs.tabText(i) for i in range(panel._tabs.count())]

    def test_strike_tab_is_not_added_by_default_but_its_content_exists(self, monkeypatch):
        from config import features
        monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
        p = ParameterPanel()
        assert '蜂群打擊' not in self._tabs(p), self._tabs(p)
        assert p._strike_tab_index == -1 and p._strike_scroll.isHidden()
        # 信號與元件還在（mixin／既有測試用得到）
        assert hasattr(p, 'stot_ros2_run_requested') and hasattr(p, '_stot_run_btn')

    def test_flag_on_adds_the_tab(self, monkeypatch):
        from config import features
        monkeypatch.setattr(features, 'STRIKE_ENABLED', True)
        p = ParameterPanel()
        assert '蜂群打擊' in self._tabs(p) and p._strike_tab_index >= 0
