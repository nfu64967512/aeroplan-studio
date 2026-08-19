# -*- coding: utf-8 -*-
"""⚠️⚠️ 沒有任何 `-p name:="$VAR"` 的 VAR 可以預設成空字串。

═══ 代價 ═══

RRA(2026-08-16)整批 **8 趟、約 5 小時**報廢。原因:

    TOT_RELEASE_SIGMA_SPEC="${RELEASE_SIGMA_SPEC:-}"    ← 預設空字串
    ... -p release_sigma_spec:="$TOT_RELEASE_SIGMA_SPEC"

空值展開成 `-p release_sigma_spec:=`,rcl 直接拒絕:

    RCLError: failed to initialize rcl: Couldn't parse parameter override rule:
    '-p release_sigma_spec:='

⇒ **節點整個起不來**,而批次看到的症狀是
   「tot_node 沒有進入 ready_for_target」—— 那長得跟
   [[fanout-port-exclusivity]](埠號被別的 client 搶走)一模一樣,
   歸因會直接歪掉。

⚠️ 最惡毒的地方:RQE~RQH 四個批次**都有**顯式傳這個變數,所以完全沒事。
   **只有走預設值的生產路徑會炸** —— 也就是說,實驗路徑全綠、正式路徑全紅。

⇒ 停用要用**哨兵字串**("off"/"none"),不能用空字串。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
UP = ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh"
SRC = UP.read_text(encoding="utf-8")

#: 抓 `-p some_param:="$SOME_VAR"`
_OVERRIDE = re.compile(r'-p\s+([A-Za-z_][A-Za-z0-9_]*):="\$([A-Za-z_][A-Za-z0-9_]*)"')
#: 抓 `VAR="${ENV:-default}"`(可能包在 $(_f ...) 裡)
_ASSIGN = re.compile(r'^(\w+)=(?:"?\$\(_f\s+)?"\$\{(\w+):-([^}]*)\}"')


def _defaults():
    out = {}
    for line in SRC.splitlines():
        m = _ASSIGN.match(line.strip())
        if m:
            out[m.group(1)] = m.group(3)
    return out


def _overrides():
    return {shell_var: param for param, shell_var in _OVERRIDE.findall(SRC)}


class TestNoParameterDefaultsToEmpty:
    """⭐⭐ 這一條擋的是整個**類別**,不是那一個變數。"""

    def test_we_actually_found_the_overrides(self):
        """前提檢查:正規式要真的抓到東西,否則這條鎖是空的。"""
        ov = _overrides()
        assert len(ov) >= 20, (
            f"只抓到 {len(ov)} 個 `-p x:=\"$VAR\"` —— 正規式可能過期了,"
            f"那這整條鎖就是假的")

    @pytest.mark.parametrize("shell_var", sorted(_overrides()))
    def test_no_override_var_defaults_to_empty(self, shell_var):
        d = _defaults()
        if shell_var not in d:
            pytest.skip(f"{shell_var} 不是 ${{ENV:-default}} 形式")
        assert d[shell_var].strip() != "", (
            f"⚠️⚠️ {shell_var} 預設成**空字串**,而它被用在 "
            f"`-p {_overrides()[shell_var]}:=\"${shell_var}\"`。\n"
            f"    空值 ⇒ `-p {_overrides()[shell_var]}:=` ⇒ rcl 解析失敗 ⇒ "
            f"節點起不來,\n"
            f"    而症狀是「沒有進入 ready_for_target」,長得像埠號被搶走。\n"
            f"    RRA 整批 8 趟就是這樣報廢的。停用請用哨兵字串(off/none)。")


#: ⚠️⚠️ rcl 會把這些字串解析成**布林**,不是字串。
#: 用它們當 STRING 參數的哨兵 ⇒ InvalidParameterTypeException ⇒ 節點起不來。
#: 實測(2026-08-16):"off" 型別錯誤 2 次、"-" 1 次;"none"/"disabled" 乾淨。
BOOLISH = {"off", "on", "true", "false", "yes", "no", "-", "0", "1"}


class TestSentinelIsNotParsedAsBool:
    """⚠️⚠️ 這是第二顆地雷,在修第一顆的時候踩到的。

    第一顆:預設空字串 ⇒ `-p name:=` ⇒ rcl 解析失敗。
    我改成哨兵 "off" —— 結果 rcl 把 "off" 當**布林 False**,
    而參數宣告是 STRING ⇒ InvalidParameterTypeException ⇒ **一樣起不來**。

    ⇒ 修一顆地雷時踩到第二顆,而兩顆的症狀完全相同
      (「tot_node 沒有進入 ready_for_target」)。
      幸好起飛前先做了節點啟動的煙霧測試,沒有再燒掉 5 小時。
    """

    def test_string_param_defaults_are_not_boolish(self):
        d = _defaults()
        ov = _overrides()
        bad = [(v, d[v]) for v in ov
               if v in d and d[v].strip().lower() in BOOLISH and not d[v].strip().replace(".", "").isdigit()]
        # 只檢查那些**宣告為字串**的參數
        tot = (ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py").read_text(encoding="utf-8")
        real = [(v, dv) for v, dv in bad
                if f'g("{ov[v]}", "")' in tot or f"g(\"{ov[v]}\", \"\")" in tot]
        assert not real, (
            "⚠️⚠️ 這些**字串**參數的預設值會被 rcl 解析成布林:"
            + repr(real)
            + "\n    ⇒ InvalidParameterTypeException ⇒ 節點起不來,"
            + "而症狀是「沒進入 ready_for_target」。"
            + "\n    改用 'none' 或 'disabled'。")


class TestTheSentinelIsUnderstoodByTheNode:
    """⭐ 哨兵字串必須真的被解析端當成「停用」,不能只是碰巧解析失敗。"""

    def test_parser_treats_sentinels_as_disabled(self):
        import ast
        import sys
        tot = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
        src = tot.read_text(encoding="utf-8")
        tree = ast.parse(src)
        fn = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "_parse_sigma_spec")
        ns = {}
        exec("class _M:\n" + "\n".join(
            "    " + l for l in ast.get_source_segment(src, fn).splitlines()), ns)
        parse = ns["_M"]._parse_sigma_spec
        for s in ("off", "OFF", "none", "-", "", "  "):
            assert parse(s) == {}, f"哨兵 {s!r} 沒有被當成停用"
        # 反向:正常規格仍要解析得出來
        assert parse("1:0:15,2:180:15") == {1: (0.0, 15.0), 2: (180.0, 15.0)}
