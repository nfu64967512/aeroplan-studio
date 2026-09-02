"""右鍵不可以再畫紅心；群體指令不可以靜靜縮成一台。

────────────────────────────────────────────────────────────────────────
案發（2026-09-02，使用者回報）
────────────────────────────────────────────────────────────────────────
1. STOT 右鍵入口已經用 `MainWindow.ENABLE_STOT_RCLICK_STRIKE = False` 關掉，
   選單裡確實沒有「以此點執行 STOT 打擊」了 —— 但**右鍵一按紅心照樣出現**。

   根因：疊圖不在 main_window，在 `CesiumMapWidget.on_map_rclick`，而那裡的
   開關寫成 ``getattr(self, '_tot_gate_auto', True)``，`_tot_gate_auto`
   **全 repo 沒有任何一處指派過** ⇒ 恆為 True，是個關不掉的開關。
   「兩個入口要一起關」的註解只寫了選單與高度框，漏掉第三個。

2. 右鍵 GUIDED 起飛「不會帶著機群」。編隊那邊其實沒有問題 —— 是點過 UAV 卡片
   之後「指令對象」被換成單機，`_resolve_flight_target()` 就回 wingmen=[]，
   群體指令**靜靜**縮成一台，畫面上只有右上角一行小字。

兩條都用值層級：真的去跑那個函式，看它**做了什麼**（發出去的 JS、有沒有
呼叫 set_tot_gate、回傳值），不鎖原始碼長什麼樣。
"""
from __future__ import annotations

import ast
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


# ═══════════════════════════════════════════════════════════════════
#  1) 右鍵不畫紅心
# ═══════════════════════════════════════════════════════════════════
def _fake_widget(on_rclick: bool):
    """duck-typed self：on_map_rclick 只碰這幾個成員，不需要真的 Qt 物件。"""
    calls = {'gate': [], 'js': [], 'emit': []}
    w = types.SimpleNamespace(
        _tot_gate_on_rclick=on_rclick,
        _tot_approach=180.0,
        set_tot_gate=lambda lat, lon, approach_from=None: calls['gate'].append(
            (lat, lon, approach_from)),
        _js=lambda s: calls['js'].append(s),
        guided_goto_requested=types.SimpleNamespace(
            emit=lambda lat, lon: calls['emit'].append((lat, lon))),
    )
    return w, calls


def _run_rclick(on_rclick: bool):
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    w, calls = _fake_widget(on_rclick)
    CesiumMapWidget.on_map_rclick(w, 23.7016, 120.4327)
    return calls


def test_rclick_draws_no_totgate():
    """核心不變式：右鍵不可以呼叫 set_tot_gate。"""
    calls = _run_rclick(False)
    assert calls['gate'] == [], f'右鍵仍在畫紅心: {calls["gate"]}'


def test_rclick_clears_leftover_totgate():
    """關掉入口還不夠 —— 上一次留在畫面上的紅心也要清掉，否則看起來像沒關。"""
    calls = _run_rclick(False)
    assert any('clearTotGate' in js for js in calls['js']), calls['js']


def test_rclick_still_requests_guided_goto():
    """儀器能不能動：右鍵的**本業**（送 GUIDED 飛到）不可以被順手砍掉。

    沒有這一條的話，一個「什麼都不做」的 on_map_rclick 也會讓上面兩條全綠。
    """
    calls = _run_rclick(False)
    assert calls['emit'] == [(23.7016, 120.4327)]


def test_flag_on_would_still_draw():
    """證明這個開關是真的開關 —— 打開就會畫。

    否則「不畫」可能只是因為那段程式碼死掉了，而不是因為開關關著。
    """
    calls = _run_rclick(True)
    assert calls['gate'] == [(23.7016, 120.4327, 180.0)]
    assert calls['emit'] == [(23.7016, 120.4327)]


def _init_bool_default(name: str) -> bool:
    """從 CesiumMapWidget.__init__ 的原始碼取出某個欄位的預設字面值。

    ⚠️ 這一條是**唯一**看原始碼的檢查，理由：要驗的正好是一個常數的預設值，
       而建構真的 CesiumMapWidget 需要 QWebEngineView（視窗 / GPU）。
       用 ast 取字面值而不是字串比對，改排版、改註解都不會誤判。
    """
    src = (REPO / 'ui' / 'widgets' / 'cesium_map_widget.py').read_text(
        encoding='utf-8')
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef) or node.name != '__init__':
            continue
        for stmt in ast.walk(node):
            targets = []
            if isinstance(stmt, ast.AnnAssign):
                targets = [stmt.target]
                value = stmt.value
            elif isinstance(stmt, ast.Assign):
                targets = stmt.targets
                value = stmt.value
            else:
                continue
            for t in targets:
                if (isinstance(t, ast.Attribute) and t.attr == name
                        and isinstance(value, ast.Constant)):
                    return value.value
    raise AssertionError(f'找不到 self.{name} 的預設值指派')


def test_rclick_gate_defaults_off():
    assert _init_bool_default('_tot_gate_on_rclick') is False


def test_marking_mode_gate_defaults_on():
    """蜂群打擊分頁的「標記目標」是使用者明確進入的流程，行為維持不變。"""
    assert _init_bool_default('_tot_gate_auto') is True


def test_the_phantom_getattr_switch_is_gone():
    """那個關不掉的開關不可以再出現。

    ``getattr(self, '_tot_gate_auto', True)`` 的問題不是寫法醜，是它讓
    「沒有這個屬性」與「開關是開的」變成同一件事 —— 於是永遠關不掉。
    """
    import io
    import tokenize
    src = (REPO / 'ui' / 'widgets' / 'cesium_map_widget.py').read_text(
        encoding='utf-8')
    # 只看**真的會執行的**程式碼 —— 註解裡寫「原本是 getattr(...)」不算違規，
    # 否則這條鎖會禁止我們解釋自己修了什麼。
    code = ''.join(
        tok.string for tok in tokenize.generate_tokens(io.StringIO(src).readline)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING))
    assert "getattr(self,'_tot_gate_auto'" not in code.replace(' ', '')
    assert "getattr(self,'_tot_approach'" not in code.replace(' ', '')


# ═══════════════════════════════════════════════════════════════════
#  2) 指令對象是單機時要問清楚
# ═══════════════════════════════════════════════════════════════════
def _fake_window(selected, n_links):
    return types.SimpleNamespace(
        sitl_hud=types.SimpleNamespace(
            get_target_sysid=lambda: selected,
            _set_target=lambda sid: None),
        _sitl_links=[object()] * n_links,
    )


def test_all_target_needs_no_confirmation():
    from ui.main_window import MainWindow
    win = _fake_window(None, 3)
    assert MainWindow._confirm_single_target(win, '起飛') is True


def test_single_aircraft_connected_needs_no_confirmation():
    """只有一台在線時「單機」沒有歧義，不要拿沒有意義的對話框煩人。"""
    from ui.main_window import MainWindow
    win = _fake_window(1, 1)
    assert MainWindow._confirm_single_target(win, '起飛') is True


@pytest.mark.parametrize('answer,expect_proceed,expect_reset', [
    ('all', True, True),      # 改成全部 → 繼續送，且指令對象被復位
    ('one', True, False),     # 只操作這台 → 繼續送，不復位
    ('cancel', False, False),  # 取消 → 不送
])
def test_single_target_with_many_links_asks_first(monkeypatch, answer,
                                                  expect_proceed, expect_reset):
    """核心不變式：多台在線 + 指令對象是單機 ⇒ **一定要問**，不可以靜靜送出去。"""
    import ui.main_window as mw

    reset = []
    win = _fake_window(2, 3)
    win.sitl_hud._set_target = lambda sid: reset.append(sid)

    class FakeBox:
        Icon = mw.QMessageBox.Icon
        ButtonRole = mw.QMessageBox.ButtonRole

        def __init__(self, parent=None):
            self._buttons = {}
            self._clicked = None
            FakeBox.instances.append(self)

        def setIcon(self, *_):
            pass

        def setWindowTitle(self, t):
            self.title = t

        def setText(self, t):
            self.text = t

        def addButton(self, label, role):
            b = object()
            key = ('all' if '全部' in label
                   else 'cancel' if '取消' in label else 'one')
            self._buttons[key] = b
            return b

        def setDefaultButton(self, b):
            self.default = b

        def exec(self):
            self._clicked = self._buttons[answer]

        def clickedButton(self):
            return self._clicked

    FakeBox.instances = []
    monkeypatch.setattr(mw, 'QMessageBox', FakeBox)

    got = mw.MainWindow._confirm_single_target(win, '起飛')
    assert FakeBox.instances, '沒有跳對話框 —— 群體指令又靜靜縮成一台了'
    assert got is expect_proceed
    assert (reset == [None]) is expect_reset
    # 訊息裡要講清楚是哪一台、現場有幾台，不然使用者還是不知道發生什麼事
    box = FakeBox.instances[0]
    assert 'UAV2' in box.text and '3' in box.text


def test_group_takeoff_actually_calls_the_confirmation():
    """守衛要接在**會改變答案**的地方 —— 光有函式沒有呼叫等於沒有。"""
    src = (REPO / 'ui' / 'main_window.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if (isinstance(node, ast.FunctionDef)
                and node.name == '_send_guided_takeoff_swarm'):
            called = {
                n.func.attr for n in ast.walk(node)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            }
            assert '_confirm_single_target' in called
            return
    raise AssertionError('找不到 _send_guided_takeoff_swarm —— 錨點漂了')
