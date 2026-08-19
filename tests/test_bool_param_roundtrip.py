# -*- coding: utf-8 -*-
"""布林參數在契約層的往返(RPZ1,2026-08-15)。

═══ 這是同一類坑的第三次 ═══

「照著建議貼,然後整批報廢」在本專案已經發生三次,每次型別不同:

  1. `rerun_hint` 用 `:g` 把 1.0 印成 `1` ⇒ rclpy 把 `1` 當 INTEGER 餵給
     宣告為 DOUBLE 的參數 ⇒ InvalidParameterTypeException ⇒ 節點起不來。
  2. `swarm_ros2_up.sh` 的 `_f()` 給整數參數(`param_verify`)補上小數點
     ⇒ 0 變 0.0 ⇒ DOUBLE 餵給 INTEGER ⇒ 同樣整批報廢(RPR)。
  3. **本檔**:`ros2 param get` 對布林參數回「Boolean value is: True」,
     結尾**沒有數字**,純數字正規式不匹配 ⇒ 判成「讀不到」⇒ 契約中止整批,
     而訊息長得像「節點沒起來」(RPZ1)。而且 `rerun_hint` 會建議
     `TGO_FROM_PREDICTOR=1.0`,照貼下去就變成第 1 類錯誤。

⇒ 三次的共同結構:**契約層自己的型別假設沒有被任何東西檢查。**
   這一檔把「讀得回來」與「建議貼得下去」兩端一起鎖住。
"""
import ast
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCHES = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
sys.path.insert(0, str(PATCHES))
sys.path.insert(0, str(ROOT / "scripts" / "wsl"))

from contract_table import (  # noqa: E402
    BOOL_PARAMS, BY_NAME, ENV_FOR, INT_PARAMS, JET_REFERENCE, evaluate,
    rerun_hint,
)
from contract_verify import parse_param_value  # noqa: E402

TOT = PATCHES / "tot_node.py"
TREE = ast.parse(TOT.read_text(encoding="utf-8"), filename=str(TOT))


def _declared_kind(name):
    """回傳 tot_node 對這個參數宣告的 Python 型別('bool'/'int'/'float')。"""
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        fn = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
        if fn not in ("g", "declare_parameter"):
            continue
        k, v = node.args[0], node.args[1]
        if not (isinstance(k, ast.Constant) and k.value == name):
            continue
        if isinstance(v, ast.Constant):
            if isinstance(v.value, bool):
                return "bool"
            if isinstance(v.value, int):
                return "int"
            if isinstance(v.value, float):
                return "float"
    return None


class TestReadingBack:
    """⭐ 讀取端:三種型別都要認得,認不得要回 None 而不是猜。"""

    @pytest.mark.parametrize("raw,want", [
        ("Boolean value is: True", "1.0"),
        ("Boolean value is: False", "0.0"),
        ("Double value is: 3100.0", "3100.0"),
        ("Double value is: -1.0", "-1.0"),
        ("Integer value is: 3", "3"),
        ("Double value is: 1e-05", "1e-05"),
        ("Double value is: .5", ".5"),
    ])
    def test_known_shapes_parse(self, raw, want):
        assert parse_param_value(raw) == want

    @pytest.mark.parametrize("raw", [
        None, "", "String value is: hello",
        "Parameter not set",
    ])
    def test_unknown_shapes_return_none(self, raw):
        """⚠️ 認不得必須回 None(= 讀不到 = 違規),**不可以**猜一個值。
        「沒讀到所以算過」是本專案整類 bug 的核心。"""
        assert parse_param_value(raw) is None

    def test_bool_is_checked_before_number(self):
        """⚠️ 順序有意義:若先跑數字正規式,`True` 不匹配就直接掉到 None。
        這條鎖住「有人把 bool 分支移到數字分支後面」。"""
        assert parse_param_value("Boolean value is: True") == "1.0"


class TestRerunHintIsPastable:
    """⭐⭐ 建議必須貼得下去。貼上去會炸的建議比不給建議更糟。"""

    @pytest.mark.parametrize("name", sorted(BOOL_PARAMS & set(BY_NAME)))
    def test_bool_hint_emits_true_or_false(self, name):
        if name not in ENV_FOR or name not in JET_REFERENCE:
            pytest.skip(f"{name} 沒有 env 對應或建議值")
        hint = rerun_hint(evaluate({}, require=[name]).violations)
        env = ENV_FOR[name]
        assert f"{env}=True" in hint or f"{env}=False" in hint, (
            f"⚠️⚠️ {name} 是 bool,建議卻印成數字:\n{hint}\n"
            f"    貼下去會變成 `-p {name}:=1.0` 餵給 BOOL ⇒ "
            f"InvalidParameterTypeException ⇒ 節點起不來。")
        assert f"{env}=1.0" not in hint and f"{env}=1" not in hint


class TestTheTypeTablesMatchTheNode:
    """⭐⭐ 三張型別表必須與節點的實際宣告一致。

    ⚠️ 這是三次事故的**共同根因**:契約層對型別的認知與節點的宣告各自演化,
       沒有任何東西保證它們一致。
    """

    @pytest.mark.parametrize("name", sorted(BOOL_PARAMS))
    def test_bool_params_really_are_bool(self, name):
        kind = _declared_kind(name)
        if kind is None:
            pytest.skip(f"{name} 不是 tot_node 的 declare_parameter 參數")
        assert kind == "bool", (
            f"⚠️ contract_table 把 {name} 列在 BOOL_PARAMS,"
            f"但 tot_node 宣告的是 {kind}")

    @pytest.mark.parametrize("name", sorted(INT_PARAMS))
    def test_int_params_really_are_int(self, name):
        kind = _declared_kind(name)
        if kind is None:
            pytest.skip(f"{name} 不是 tot_node 的 declare_parameter 參數")
        assert kind == "int", (
            f"⚠️ contract_table 把 {name} 列在 INT_PARAMS,"
            f"但 tot_node 宣告的是 {kind}")

    def test_no_table_param_is_silently_bool(self):
        """反向鎖:契約表裡任何**實際宣告為 bool** 的參數都必須在 BOOL_PARAMS。

        ⚠️ 漏一個 ⇒ 它的 rerun_hint 會印成 1.0 ⇒ 下一次照貼、下一批報廢。
        """
        missed = [c.name for c in BY_NAME.values()
                  if _declared_kind(c.name) == "bool" and c.name not in BOOL_PARAMS]
        assert not missed, (
            f"⚠️⚠️ 這些契約參數在節點是 bool,卻不在 BOOL_PARAMS:{missed}")


class TestTransientReadRetry:
    """⚠️⚠️ 暫態讀取失敗必須重試 —— 但**只放寬讀取,不放寬判定**。

    代價證據:RPW5 一整批 + RQD4 + RQG3/RQG4 共四趟被一次性讀取抖動殺掉,
    而四次的組態全是對的(同批的其他趟用同組態讀得到)。

    ⚠️ 界線:重試完仍讀不到 ⇒ **照樣違規**。放寬判定會退化成
    「沒讀到所以算過」,那正是這一層要治的病本身。
    """

    def test_retry_count_is_configurable_and_at_least_two(self):
        import contract_verify as cv
        assert cv.RETRY_READS >= 2, (
            f"重試次數 {cv.RETRY_READS} 太少 —— 一次抖動就會再殺一整批")

    def test_the_read_loop_actually_retries(self):
        """結構鎖:讀取必須在迴圈裡,而且失敗時要 sleep。"""
        src = (ROOT / "scripts" / "wsl" / "contract_verify.py").read_text(
            encoding="utf-8")
        i = src.find("for attempt in range(RETRY_READS)")
        assert i != -1, "⚠️ 讀取不再重試 —— RQG3 那類整批中止會回來"
        block = src[i:i + 700]
        assert 'ros2", "param", "get"' in block, "重試迴圈裡沒有實際的讀取"
        assert "time.sleep(RETRY_SLEEP_S)" in block, "重試之間沒有退避"

    def test_still_a_violation_after_all_retries_fail(self):
        """⭐ 反向鎖:重試用盡仍讀不到,**必須**還是違規。"""
        from contract_table import evaluate
        rep = evaluate({}, require=["vmax"])
        kinds = [v.kind for v in rep.violations if v.name == "vmax"]
        assert kinds == ["missing"], (
            "⚠️⚠️ 讀不到不再算違規了 —— 這一層就退化成『沒讀到所以算過』")
