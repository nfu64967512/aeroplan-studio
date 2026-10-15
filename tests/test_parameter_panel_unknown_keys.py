"""``ParameterPanel.set_parameters`` 遇到不認得的 key 必須出聲。

背景 bug：``set_parameters`` 的迴圈是 ``if key in self.parameters:``，**沒有**
else 分支，最後卻無條件印「參數已設置」。於是：

    pp.set_parameters({"overlap": 30.0})     # 靜默 no-op + 一行成功訊息
    pp.get_parameters().get("overlap")       # -> None

呼叫端（`scripts/_full_flow_capture.py`）把 None 印出來當作正常結果，整趟
擷取流程照樣報成功 —— 一支不可能響的儀器。

⚠️ 「重疊率」在本專案有三個**不同**的量，而裸 ``overlap`` **一個都不是**：
   * ``reduce_overlap``  — bool，互補掃描開關
   * ``overlap_rate``    — 比例 0.0–0.5，**橫向**條帶間距（`get_dccpp_params()`）
   * ``*_overlap_percent`` — 百分比，相機**航向**重疊（`camera_model`）
   所以修法是讓未知 key 出聲，**不是**幫它猜一個對應。
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PyQt6.QtWidgets import QApplication
    from ui.widgets.parameter_panel import ParameterPanel
    _APP = QApplication.instance() or QApplication([])
    _OK = True
except Exception:                                    # pragma: no cover
    _OK = False

pytestmark = pytest.mark.skipif(not _OK, reason="PyQt6 / ParameterPanel 無法載入")


@pytest.fixture
def panel():
    return ParameterPanel()


def test_bare_overlap_is_not_a_key_of_this_panel(panel):
    """可觸發性前置斷言：先證明 ``overlap`` 真的不是這個面板的參數。

    若哪天有人真的加了這個 key，下面兩條測試就失去意義，這條會先變紅。
    """
    assert "overlap" not in panel.get_parameters()
    assert "overlap_rate" not in panel.get_parameters(), (
        "overlap_rate 是 get_dccpp_params() 那一組，不該出現在 get_parameters()"
    )
    assert "overlap_rate" in panel.get_dccpp_params()


def _capture_warnings(monkeypatch):
    """攔 ``parameter_panel.logger.warning``。

    ⚠️ 不能用 pytest 的 caplog：本專案的 ``utils.logger.get_logger()`` 是自訂
    Logger，不走 stdlib 的 root propagation，caplog 看不到它。用 caplog 寫的話
    這條測試會**恆綠**（記錄為空 ⇒ 斷言 'overlap' in '' 失敗…或反過來恆過），
    正好是本專案記過的「儀器不可能響」那一類。
    """
    import ui.widgets.parameter_panel as pp_mod
    seen = []
    monkeypatch.setattr(pp_mod.logger, "warning", lambda msg, *a, **k: seen.append(msg))
    return seen


def test_the_warning_capture_actually_works(panel, monkeypatch):
    """可觸發性：先證明這個攔截手法真的抓得到 warning。"""
    import ui.widgets.parameter_panel as pp_mod
    seen = _capture_warnings(monkeypatch)
    pp_mod.logger.warning("canary")
    assert seen == ["canary"]


def test_unknown_key_is_warned_not_silently_dropped(panel, monkeypatch):
    seen = _capture_warnings(monkeypatch)
    panel.set_parameters({"altitude": 80.0, "overlap": 30.0})
    assert any("overlap" in m for m in seen), (
        f"未知的 key 被靜默丟掉了 —— 這正是原始 bug。收到的警告：{seen}"
    )
    # 認得的 key 仍必須生效
    assert panel.get_parameters()["altitude"] == pytest.approx(80.0)


def test_all_known_keys_logs_no_warning(panel, monkeypatch):
    """對照組：全部 key 都認得時**不可以**發警告，否則警告會被當背景噪音。"""
    seen = _capture_warnings(monkeypatch)
    panel.set_parameters({"altitude": 75.0, "speed": 9.0})
    assert seen == [], f"全部認得卻還是發了警告：{seen}"


def test_capture_script_no_longer_sends_the_undefined_key():
    """擷取腳本不可以再送那個沒有定義的 key。

    用 AST 找「傳給 set_parameters 的 dict 字面值」，**不是**字串比對 ——
    第一版寫成 ``'"overlap": 30.0' not in text``，結果被腳本裡解釋這個 bug 的
    註解自己打敗（註解引用了那段程式碼）。字串比對分不出程式碼與註解。

    那支腳本是一次性的 GUI 擷取流程，沒辦法在測試裡真的跑起來，所以這只是
    輔助鎖；真正的保護是上面那條「未知 key 會出聲」。
    """
    import ast
    from pathlib import Path
    src = Path(__file__).resolve().parent.parent / "scripts" / "_full_flow_capture.py"
    tree = ast.parse(src.read_text(encoding="utf-8"))

    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute)
             and n.func.attr == "set_parameters"]
    assert calls, "腳本裡找不到 set_parameters 呼叫 —— 這條鎖已經失去對象"

    sent = set()
    for c in calls:
        for a in c.args:
            if isinstance(a, ast.Dict):
                sent |= {k.value for k in a.keys
                         if isinstance(k, ast.Constant) and isinstance(k.value, str)}
    assert "overlap" not in sent, f"腳本又送了未定義的 overlap（送出的 key：{sorted(sent)}）"
    assert sent, "沒有解析到任何 key，這條鎖沒有在檢查東西"

    # 讀回必須用中括號：key 名再改要當場 KeyError，不可以又回一個 None。
    def _is_dccpp_call(n):
        return (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == "get_dccpp_params")

    subscripted = [n for n in ast.walk(tree)
                   if isinstance(n, ast.Subscript) and _is_dccpp_call(n.value)]
    dotted_get = [n for n in ast.walk(tree)
                  if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                  and n.func.attr == "get" and _is_dccpp_call(n.func.value)]
    assert subscripted, "讀回沒有用中括號取 get_dccpp_params() 的值"
    assert not dotted_get, (
        "讀回改用了 .get() —— key 名打錯會靜靜回 None，正是原始 bug 的形狀"
    )
