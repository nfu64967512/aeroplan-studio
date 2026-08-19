# -*- coding: utf-8 -*-
"""ROS2 參數的**型別**必須在宣告端與傳遞端一致。

═══ 這一檔是 RPR 整批三趟報廢換來的 ═══

    rclpy.exceptions.InvalidParameterTypeException:
        Trying to set parameter 'param_verify' to '0.0' of type 'DOUBLE',
        expecting type 'INTEGER'

⚠️⚠️ 症狀是「**tot_node 沒有進入 ready_for_target**」——
   看起來像節點壞了或起太慢,完全不像型別問題。而且:
   * `bash -n` 過
   * `python -m py_compile` 過
   * 1587 條單元測試全綠
   * 部署 md5 一致
   四道關卡全部通過,然後三趟航次原地報廢。

成因:`swarm_ros2_up.sh` 的 `_f()` 會**自動補小數點**(`0` → `0.0`),
那是為了另一個坑(浮點參數少了小數點會被當成 INTEGER)。
但反過來用在**整數**參數上就變成 DOUBLE,rclpy 的型別檢查是嚴格的。

⇒ 兩個方向都會壞,而且方向相反:
      浮點參數走 raw   ⇒ `0` 被當 INTEGER  ⇒ 爆
      整數參數走 _f()  ⇒ `0.0` 是 DOUBLE   ⇒ 爆

所以這件事**必須自動檢查**,不能靠記得哪個是哪個。
"""
import ast
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SCRIPTS = [ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh",
           ROOT / "scripts" / "wsl" / "stot_trial.sh"]
TREE = ast.parse(TOT.read_text(encoding="utf-8"), filename=str(TOT))


def _declared_types():
    """參數名 -> "int" | "float" | "bool" | "other"(依 declare 的字面值)。"""
    out = {}
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        fn = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
        if fn not in ("g", "declare_parameter"):
            continue
        k, v = node.args[0], node.args[1]
        if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
            continue
        if isinstance(v, ast.Constant):
            val = v.value
            if isinstance(val, bool):
                out[k.value] = "bool"
            elif isinstance(val, int):
                out[k.value] = "int"
            elif isinstance(val, float):
                out[k.value] = "float"
            else:
                out[k.value] = "other"
    return out


TYPES = _declared_types()


def _passed_params(path):
    """從腳本抽 `-p <name>:="$VAR"` 的對應,以及該 VAR 是否走 _f()。"""
    src = path.read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r'-p\s+([a-z_0-9]+):="\$([A-Za-z_0-9]+)"', src):
        prm, var = m.group(1), m.group(2)
        d = re.search(rf'^{re.escape(var)}=(.*)$', src, re.M)
        out[prm] = (var, d.group(1) if d else "")
    return out


class TestIntegerParametersAreNotFloatified:
    """⭐⭐ RPR 報廢的直接原因。

    ⚠️ `_f()` 補小數點是為了浮點參數;套在整數參數上會讓節點**整個起不來**。
    """

    @pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
    def test_no_integer_param_goes_through_f(self, script):
        bad = []
        for prm, (var, rhs) in _passed_params(script).items():
            if TYPES.get(prm) == "int" and "_f " in rhs:
                bad.append(f"{prm}(宣告 int)由 {var} 走 _f() ⇒ 會變 DOUBLE")
        assert not bad, (
            f"{script.name} 有整數參數走 _f():\n  " + "\n  ".join(bad) +
            "\n  ⇒ InvalidParameterTypeException,節點起不來,"
            "而症狀是「沒有進入 ready_for_target」")


class TestFloatParametersDoGoThroughF:
    """反方向同樣會壞:浮點參數若不補小數點,`0` 會被當成 INTEGER。

    ⚠️ 這是 `_f()` 存在的原因,`stot_trial.sh` 的註解記著。
       兩個方向都要鎖,只鎖一邊等於把坑換個方向留著。
    """

    @pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
    def test_float_params_are_floatified_or_literal(self, script):
        bad = []
        for prm, (var, rhs) in _passed_params(script).items():
            if TYPES.get(prm) != "float":
                continue
            if "_f " in rhs:
                continue
            # 沒走 _f() 時,預設值本身必須帶小數點
            m = re.search(r':-([0-9]+(?:\.[0-9]+)?)', rhs)
            if m and "." not in m.group(1):
                bad.append(f"{prm}(宣告 float)由 {var} 傳 `{m.group(1)}`,沒有小數點")
        assert not bad, (
            f"{script.name} 有浮點參數可能被當成 INTEGER:\n  " + "\n  ".join(bad))


class TestBooleanParametersAreNotFloatified:
    """布林同理:`_f()` 會把 `False` 變成什麼都不對。"""

    @pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
    def test_bool_params_pass_raw(self, script):
        bad = [f"{prm} 由 {var} 走 _f()"
               for prm, (var, rhs) in _passed_params(script).items()
               if TYPES.get(prm) == "bool" and "_f " in rhs]
        assert not bad, f"{script.name} 有布林參數走 _f():{bad}"


class TestTheCheckActuallySeesSomething:
    """⚠️ 反向鎖:正規式抓不到東西時上面三條會**全部空跑而變綠** ——
    一條靜默通過的測試比沒有測試更糟(本專案今天已經踩過一次)。"""

    def test_declarations_were_parsed(self):
        assert len(TYPES) > 60, f"只解析到 {len(TYPES)} 個參數宣告"
        assert TYPES.get("param_verify") == "int"
        assert TYPES.get("hold_alt") == "float"
        assert TYPES.get("att_require_fc") == "bool"

    @pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
    def test_params_were_parsed(self, script):
        assert len(_passed_params(script)) > 30, \
            f"{script.name} 只抓到 {len(_passed_params(script))} 個 -p"
