# -*- coding: utf-8 -*-
"""S2:`wait_goto_radius`(wait 段也飛過去,不繞圈)的迴歸鎖(2026-08-18)。

═══ 這一條改的是什麼 ═══

RRD 批次(8 趟 × 2 架 = 16 架次)裡 6 架失手。根因**不是飛行問題,是指令從未送達**:

`tot_node._goto()` 的變更偵測器存的是 `self.wp_pt[i] = (lat, lon, alt)` ——
**radius 不在裡面**。而 runin→wait 切換的那一發 `_goto`,lat/lon/alt **逐位元相同**
(`ring_point_of(i)` 不變;hold_alt=0 ⇒ `hold_alt_of()` 就是 `perch_of()`),
只有 radius 由 goto_radius=30 變成 hold_radius=1300
⇒ `moved=False` ⇒ 那發 DO_REPOSITION **根本沒有送出去**,飛機掛著舊的 param3=30 直飛。
hold_radius 只有在下一個 **30 s keepalive** 補發時才生效
⇒ **一架會不會起彎,取決於它進 wait 時距 30 s 格點還有多久 —— 逐趟隨機。**

實測驗證(16 架次):用 keepalive 相位預測「會不會起彎」命中 15/16;
5 架起彎者的「預測補發 → 實測起彎」殘差 +2.0/+0.4/+0.9/+2.2/+0.8 s(= 滾轉建立時間),
無一早於預測。⇒ 這一批其實是**兩個母體**:

| 母體 | 架次 | 放行前置角 σ_pre | 命中(觸地誤差 ≤10 m) |
|---|---|---|---|
| 直進(沒吃到 keepalive 補發) | 11 | 0.0 – 5.9° | **10/11** |
| 硬彎(吃到補發) | 5 | 29.0 – 78.3° | **0/5** |

⭐ 5.9° 與 29.0° 之間**零樣本** —— 這是離散失效模式的指紋,不是連續退化。

═══ 為什麼這一檔要寫這麼多條 ═══

⚠️⚠️ **成功的指紋是「指令的缺席」。** 開關打開後 wait 與 runin 要求同一個半徑
⇒ 切換當拍**不會**送出任何 DO_REPOSITION。在 log 裡找確認指令是找不到的,
那是對的,不是失敗。⇒ 這是本專案唯一一個「沒有正向執行證據」的改動,
所以它的正確性**只能**靠原始碼結構本身鎖住,不能靠事後看 log。

⚠️ 判「機制有沒有生效」只能看 CSV 的 `req_radius_m`(本拍**要求**的半徑,每拍都寫),
   **不能**看 `cmd_radius_m`(真正送出去的,hold-last-value)—— 後者在開關開與關時
   同樣是 30,會給**假綠燈**。⇒ 本檔 §5 §7 §8 全都是在保護這個判讀基礎。

⚠️ 本專案有「開關開著但那段從來沒真的發生」的招牌失效類別,已經**四次**:
   predict_hz 沒接線、ITCG 從未武裝、ring_pace 守衛恆偽、半徑指令被去重器吃掉。
   §2(env 接線)與 §4(`or` 不是 `and`)就是防第五次。

⚠️ 儀表歸零(`req_radius`/`sent_radius` 進 RESET_DICTS)已由
   `tests/test_tot_standby.py:345` 鎖住,本檔不重複。
"""
import ast
import pathlib
import re
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCHES = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
sys.path.insert(0, str(PATCHES))

TOT = PATCHES / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))
UP_PATH = ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh"
UP = UP_PATH.read_text(encoding="utf-8")

from contract_table import (  # noqa: E402
    BOOL_PARAMS, BY_NAME, ENV_FOR, JET_REFERENCE, SOURCE_NODE,
)

SWITCH = "wait_goto_radius"
ENV = "WAIT_GOTO_RADIUS"


# ══════════════════════════════════════════════════════════════════════════
#  取材工具
# ══════════════════════════════════════════════════════════════════════════
def _code_only(block):
    """剝掉整行註解與行尾註解,只留程式碼。

    ⚠️⚠️ 這個 repo 已經栽過**四次**:用字串比對驗程式碼,結果命中的是**註解**。
       (`swarm_takeoff.sh` 的裸檔名、`self.pn_dt / 0.7` 出現在註解裡、
        `weave_time_sync` 出現在 docstring 裡、`hold_radius` 出現在正在解釋
        舊行為的那句註解裡。)
    ⇒ 凡是「某字串**不可以**出現」的斷言,都必須先剝註解。
      本檔的取材以 AST 為主(Python 側結構上不可能命中註解);走文字的只有兩處:
      shell(沒有 AST)用 `_sh_lines()`,`contract_verify.py` 用本函式。
    ⚠️ 2026-08-18 更正:這段原本寫「只有 shell 走文字,所以這裡只給 shell 版」,
       但實際上 shell 從來沒有呼叫過本函式(它走 `_sh_lines()`),而 `_sh_lines()`
       當時只剝**整行**註解 —— 這就是 M10 逃脫的破口。已在 `_sh_lines()` 修好。

    ⚠️ S2 的註解區塊裡**逐字出現**「radius 由 30 變 1300」「hold_radius」
       「goto_radius」等字樣(它在解釋 bug 本身)⇒ 對 S2 而言這個坑的觸發
       機率比別條更高。
    """
    out = []
    for line in block.splitlines():
        s = line.strip()
        if s.startswith("#"):
            continue
        out.append(line.split("  #")[0] if "  #" in line else line)
    return "\n".join(out)


def _func(name):
    """取出頂層/類別內的 FunctionDef。"""
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    pytest.fail(f"⚠️ 找不到函式 {name}()")


def _writerow_list(fn):
    """取出某函式裡 `self._cw.writerow([...])` 的那個 List 節點。"""
    for n in ast.walk(fn):
        if (isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute)
                and n.func.attr == "writerow"
                and n.args and isinstance(n.args[0], ast.List)):
            return n.args[0]
    pytest.fail(f"⚠️ {fn.name}() 裡找不到 writerow([...])")


def _header():
    """CSV 表頭(字串常數清單)。"""
    return [e.value for e in _writerow_list(_func("_open_csv")).elts]


def _wait_radius_ifexp():
    """致動點:wait/runin 那一發 `_goto` 的 `radius=` 運算式。

    ⚠️ 用 AST 找「radius 這個 keyword 的值裡提到 wait_goto_radius」,
       而不是找行號 —— 行號會隨註解長度漂移,而 S2 的註解有 30 行。
    """
    hits = []
    for n in ast.walk(TREE):
        if not (isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute)
                and n.func.attr == "_goto"):
            continue
        for kw in n.keywords:
            if kw.arg == "radius" and SWITCH in ast.unparse(kw.value):
                hits.append(kw.value)
    assert len(hits) == 1, (
        f"⚠️⚠️ 提到 {SWITCH} 的 `_goto(radius=...)` 有 {len(hits)} 個,應該恰好 1 個。\n"
        f"    0 個 = 致動點被刪掉,開關變成裝飾品(本專案第 5 次「開關開著但那段"
        f"從來沒發生」);\n"
        f"    ≥2 個 = 有人把它複製到別的段落,而 hold_radius 仍是 ALIGN 守環與 "
        f"missed 守點**真正需要**的東西。")
    return hits[0]


def _strip_sh_trailing_comment(ln):
    """剝掉一行 shell 的**行尾**註解(追引號狀態,不誤剝字串裡的 `#`)。

    ⚠️⚠️ 這條防的是 **M10**:把 `WAIT_GOTO_RADIUS:-False` 翻成 `True`,再把
       舊行原文**原封不動貼在行尾註解**裡。舊版 `_sh_lines()` 的 docstring 寫
       「剝掉註解」,但它只剝「整行都是註解」的行(`strip().startswith("#")`),
       行尾註解一個字都沒剝 ⇒ `"${WAIT_GOTO_RADIUS:-False}"` 仍然在文字裡找得到
       ⇒ §1 §2 全綠,而實際預設已經是 True。
       **逃脫的後果**:此後任何一趟「沒有顯式關掉」的批次都靜靜混進 S2 行為,
       而症狀是**命中率變好** —— 本檔 docstring 自己寫的「最不會有人去查的方向」。
       RRD 對照組被污染 ⇒「直進 10/11 vs 硬彎 0/5」這個唯一的證據再也重現不了。
    ⚠️ 這不是理論風險:swarm_ros2_up.sh 本身大量使用行尾註解
       (159/161/317/439-443 …),那就是這個檔案的書寫風格。
    ⚠️ 只在「`#` 前面是空白 **且** 不在引號內」時截斷
       ⇒ `${VAR#pat}`、`$#`、`"a#b"` 都不受影響(誤剝程式碼會讓反向鎖假綠)。
    """
    out = []
    q = None                      # None / "'" / '"'
    i, n = 0, len(ln)
    while i < n:
        ch = ln[i]
        if q is None:
            if ch == "\\":
                out.append(ch)
                if i + 1 < n:
                    out.append(ln[i + 1]); i += 1
            elif ch in ("'", '"'):
                q = ch; out.append(ch)
            elif ch == "#" and (not out or out[-1] in " \t"):
                break             # ← 這裡開始是行尾註解
            else:
                out.append(ch)
        else:
            if ch == "\\" and q == '"':
                out.append(ch)
                if i + 1 < n:
                    out.append(ln[i + 1]); i += 1
            elif ch == q:
                q = None; out.append(ch)
            else:
                out.append(ch)
        i += 1
    return "".join(out).rstrip()


def _sh_lines():
    """shell 原始碼(剝掉整行註解**與行尾註解**)。shell 沒有 AST,只能走文字。

    ⚠️⚠️ 行尾註解一定要剝,理由見 `_strip_sh_trailing_comment()`(M10)。
    """
    out = []
    for ln in UP.splitlines():
        if ln.strip().startswith("#"):
            continue
        out.append(_strip_sh_trailing_comment(ln))
    return out


# ══════════════════════════════════════════════════════════════════════════
#  §1 開關預設必須是 False
# ══════════════════════════════════════════════════════════════════════════
class TestTheSwitchDefaultsOff:
    """⭐ 預設 False = **逐位元同 RRD 組態**。

    ⚠️⚠️ 這是對照組不被污染的唯一保證。RRD 的 16 架次是「兩個母體」宣稱的
       全部證據(直進 11 架命中 10/11、硬彎 5 架命中 0/5);預設一旦變成 True,
       之後任何一趟「沒有顯式關掉」的批次都會混進 S2 行為,而症狀是
       **命中率變好** —— 那正是最不會有人去查的方向。
    """

    def test_declared_default_is_false(self):
        """`g("wait_goto_radius", False)`(tot_node.py:594 附近)。"""
        for n in ast.walk(TREE):
            if (isinstance(n, ast.Call)
                    and (getattr(n.func, "id", None)
                         or getattr(n.func, "attr", None)) in ("g", "declare_parameter")
                    and len(n.args) >= 2
                    and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == SWITCH):
                assert n.args[1].value is False, (
                    f"⚠️⚠️ {SWITCH} 的宣告預設不是 False({n.args[1].value!r})。\n"
                    f"    餵值路徑一斷,整批就靜靜跑 S2 行為而不是 RRD 對照組。")
                return
        pytest.fail(f"⚠️ 找不到 {SWITCH} 的 declare_parameter")

    def test_class_attribute_default_is_false_too(self):
        """類別屬性 `wait_goto_radius = False`(tot_node.py:233)。

        ⚠️ 這不是重複:類別屬性是**跳過 `__init__` 的樁節點**(以及參數讀取
           那一行以前的每一拍)實際會看到的值。它若是 True,一個沒跑完
           初始化的節點就會靜靜地致動 —— 而 `self.wait_goto_radius` 在
           致動點是直接讀的,沒有 getattr 保護。
        """
        for cls in ast.walk(TREE):
            if not isinstance(cls, ast.ClassDef):
                continue
            for s in cls.body:
                if (isinstance(s, ast.Assign)
                        and any(getattr(t, "id", None) == SWITCH for t in s.targets)):
                    assert s.value.value is False, (
                        f"⚠️⚠️ {cls.name} 的類別屬性 {SWITCH} 不是 False")
                    return
        pytest.fail(f"⚠️ 找不到類別屬性 {SWITCH}")

    def test_shell_default_is_false(self):
        """`WAIT_GOTO_RADIUS:-False`(swarm_ros2_up.sh:517)。"""
        code = "\n".join(_sh_lines())
        assert f'"${{{ENV}:-False}}"' in code, (
            f"⚠️⚠️ 啟動腳本的 {ENV} 預設不是 False —— S2 會混進別的批次,"
            f"而混進去的方向是『命中率變好』,不會有人去查。")

    def test_the_shell_default_is_declared_exactly_once_and_is_false(self):
        """⚠️⚠️ 防 M10 的第二層:正向鎖 + **唯一性**(不是「某字串存在」)。

        M10 的形狀:把預設翻成 True,舊行原文留在行尾註解 ⇒ 只要斷言是
        「`"${WAIT_GOTO_RADIUS:-False}"` 這個字串存在」,註解裡那一份就餵飽它。
        `_sh_lines()` 已經改成會剝行尾註解,這一條再從另一個方向補死:
        **整份腳本裡 `${WAIT_GOTO_RADIUS:-…}` 只能出現一次,而且預設值是 False**。

        另一個逃脫形狀:第一行照舊寫 False,後面再補一行
        `TOT_WAIT_GOTO_RADIUS="${WAIT_GOTO_RADIUS:-True}"` 覆蓋掉它 ——
        「字串存在」式的斷言對「後面又被覆蓋一次」零鑑別力(前一行仍在)。
        ⇒ 逃脫後果與 M10 相同:對照組被污染,而方向是命中率變好。
        """
        code = "\n".join(_sh_lines())
        defaults = re.findall(r"\$\{" + ENV + r":-([^}]*)\}", code)
        assert defaults == ["False"], (
            f"⚠️⚠️ `${{{ENV}:-…}}` 在啟動腳本裡出現 {len(defaults)} 次、預設值為 "
            f"{defaults} —— 應該**恰好一次**且為 False。\n"
            f"    多於一次 = 後面那個覆蓋前面那個(污染對照組);\n"
            f"    值不是 False = S2 混進沒有顯式關掉的批次,而症狀是命中率變好。")
        asg = re.findall(r"TOT_" + ENV + r"=(\S+)", code)
        assert asg == [f'"${{{ENV}:-False}}"'], (
            f"⚠️⚠️ `TOT_{ENV}=` 的賦值有 {len(asg)} 處、內容 {asg} —— "
            f"應該恰好一處且為 `\"${{{ENV}:-False}}\"`")

    def test_the_shell_default_is_not_empty(self):
        """⚠️ 預設值**不可以**是空字串。

        `-p name:=` 會讓 rcl 解析失敗、節點整個起不來,而症狀長得像埠號被搶走
        —— RRA 整批 8 趟 / 5 小時就是這樣報廢的。
        """
        code = "\n".join(_sh_lines())
        assert f'"${{{ENV}:-}}"' not in code and f'"${{{ENV}}}"' not in code, (
            f"⚠️⚠️ {ENV} 可能展開成空字串 ⇒ `-p {SWITCH}:=` ⇒ 節點起不來,"
            f"而錯誤訊息指向埠號(RRA 報廢 8 趟的形狀)")


# ══════════════════════════════════════════════════════════════════════════
#  §2 env 接線(防「開關開著但那段從來沒真的發生」第五次)
# ══════════════════════════════════════════════════════════════════════════
class TestEnvIsActuallyWired:
    """⚠️⚠️ 本專案的招牌失效類別,已經四次:

    1. `predict_hz` —— 節點 declare 了,啟動腳本從來沒餵 ⇒ 一路吃預設 0.0,
       `_predict_tick()` 第一行就 return(S9 做好之後整整沒接線)。
    2. ITCG —— 開關看起來開著,但沒有任何一行證明它武裝過。
    3. `ring_pace` —— 守衛恆偽。
    4. RRD 本身 —— 半徑指令被 `_goto` 的去重器吃掉。

    四次的共同結構都是「意圖存在、接線斷掉、而且完全靜默」。
    ⇒ 這一條鎖驗的是 env → shell 變數 → `-p` → **節點屬性** 四段**首尾相連**,
      不是各自存在。四段各自存在但中間名字對不上,症狀與完全沒接線一模一樣。

    ⚠️⚠️ 2026-08-18 補第四段(防 M11):原本這個類別只驗到 `-p` 為止,而真正
       走到致動點的鏈是四段 —— `-p` 之後還有
       `p("wait_goto_radius").value` → `self.wait_goto_radius`(tot_node.py:897)。
       第四段當時**完全沒有斷言**。
    """

    def test_env_reaches_a_shell_variable(self):
        code = "\n".join(_sh_lines())
        assert f'TOT_{ENV}="${{{ENV}:-False}}"' in code, (
            f"⚠️ 找不到 `TOT_{ENV}=\"${{{ENV}:-False}}\"` —— env 沒有落到 shell 變數")

    def test_that_shell_variable_reaches_the_node(self):
        code = "\n".join(_sh_lines())
        assert f'-p {SWITCH}:="$TOT_{ENV}"' in code, (
            f"⚠️⚠️ `-p {SWITCH}:=` 沒有餵 `$TOT_{ENV}`。\n"
            f"    ⇒ 開關在命令列上看得見、節點端卻吃預設值,而**完全靜默**。\n"
            f"    這正是 predict_hz 從 S9 做好之後整整沒接線的那個形狀。")

    def test_the_parameter_is_passed_by_variable_not_by_literal(self):
        """⚠️ 反向鎖:`-p wait_goto_radius:=True` 這種寫死值不算接線。

        寫死 True 會讓對照組整批污染;寫死 False 會讓開關永遠打不開,
        而兩者在 log 上都看得到「參數有送」⇒ 假綠燈。
        """
        code = "\n".join(_sh_lines())
        for bad in (f'-p {SWITCH}:=True', f'-p {SWITCH}:=False',
                    f'-p {SWITCH}:=1', f'-p {SWITCH}:=0'):
            assert bad not in code, (
                f"⚠️⚠️ 啟動腳本把 {SWITCH} 寫死成常數({bad}) —— env 完全失效")
        # ⚠️⚠️ 防的是 **M10c**:上面那組黑名單只涵蓋 4 種拼法且**大小寫敏感**,
        #    而 ros2 的 `-p` 對 bool 也吃 `true`/`false`;更要命的是 `-p`
        #    **重複出現時後者覆蓋前者**,所以「正向鎖要求正確那一發存在」
        #    擋不住在它後面再掛一發 `-p wait_goto_radius:=true`(前一發還在,
        #    §2 的三條全綠)。逃脫的後果與 M10 同族:env 完全失效、開關被寫死
        #    打開,而命令列與 log 上看起來完全正常 ⇒ 對照組被污染。
        #    ⇒ 改用正規式數出現次數,恰好 1 次且值是那個 shell 變數。
        got = re.findall(r"-p\s+" + SWITCH + r":=(\S+)", code)
        assert got == [f'"$TOT_{ENV}"'], (
            f"⚠️⚠️ `-p {SWITCH}:=` 在啟動腳本裡出現 {len(got)} 次、值為 {got} —— "
            f"應該恰好一次且為 `\"$TOT_{ENV}\"`。\n"
            f"    重複掛一發 ⇒ 後者覆蓋前者 ⇒ env 失效而命令列上看起來完全正常。")

    def test_the_node_actually_reads_that_parameter_into_the_attribute(self):
        """⭐⭐ **第四段接線**:`p("wait_goto_radius").value` → `self.wait_goto_radius`。

        ⚠️⚠️ 這條防的是 **M11**:把 tot_node.py:897 改成
           `self.wait_goto_radius = bool(p("release_at_ring").value)`。
           在覆核前這個突變**全套 1925 條零反應** —— 前三段的鎖都還綠著,
           因為 `-p` 確實有餵、shell 變數確實有接、契約表也確實有登記。
        **逃脫的後果**(兩件,都完全靜默):
          ① `self.wait_goto_radius` 有類別屬性預設 False(tot_node.py:233),
             所以第四段一斷,開關就**永遠關著**,而 log、契約表、cmd_radius_m
             全部正常 —— 這正是本專案「開關開著但那段從來沒真的發生」的第五次。
          ② 讀錯參數名會把**兩顆開關綁在一起**:打開 RELEASE_AT_RING 會順手
             打開 S2,使用者不會知道自己同時動了兩個變數 ⇒ 對照組被污染。
        ⇒ 所以參數名必須**逐字**相符,不能只驗「有從某個參數讀進來」。
        """
        assigns = [n for n in ast.walk(TREE)
                   if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Attribute) and t.attr == SWITCH
                           for t in n.targets)]
        assert len(assigns) == 1, (
            f"⚠️⚠️ `self.{SWITCH} = …` 有 {len(assigns)} 處,應該恰好 1 處。\n"
            f"    0 處 = 第四段接線斷掉 ⇒ 永遠吃類別屬性預設 False ⇒ 開關是"
            f"裝飾品而且完全靜默;\n"
            f"    ≥2 處 = 有人在別的地方覆寫它,後寫的贏而且沒有任何告警。")
        got = ast.unparse(assigns[0].value)
        want = f"bool(p('{SWITCH}').value)"
        assert got == want, (
            f"⚠️⚠️ `self.{SWITCH}` 不是從 `{want}` 讀進來的,實得 `{got}`"
            f"(tot_node.py:{assigns[0].lineno})。\n"
            f"    讀錯參數名 ⇒ 開關永遠關著(類別屬性預設 False)且完全靜默,\n"
            f"    或與另一顆開關綁在一起(打開 A 順手打開 S2)⇒ 對照組被污染。")


# ══════════════════════════════════════════════════════════════════════════
#  §3 契約表(生效值必須可見)
# ══════════════════════════════════════════════════════════════════════════
class TestContractTableCoversTheSwitch:
    """⭐ 契約層的存在理由:**生效值與意圖值不同,而沒有任何東西在檢查**。

    ⚠️ 這一列的目的是**可見性**,不是擋值(區間 [0,1],0/1 都放行)——
       前三批白跑有兩批(RRA / RRB)的症狀都是
       「開關看起來開著,但沒有任何一行證明它生效」。
    """

    def test_it_is_in_the_contract_table(self):
        assert SWITCH in BY_NAME, (
            f"⚠️⚠️ {SWITCH} 不在契約表 ⇒ 生效值 log 印不出它 ⇒ 事後無法證明"
            f"這一趟到底跑的是 S2 還是 RRD 對照組")
        c = BY_NAME[SWITCH]
        assert c.source == SOURCE_NODE
        assert (c.lo, c.hi) == (0.0, 1.0), (
            f"⚠️ 區間應該是 [0,1](0/1 都放行:這是**對照用**的開關),實得 "
            f"[{c.lo}, {c.hi}]")

    def test_env_mapping_exists_and_matches_the_shell(self):
        """⚠️ 契約表的 env 名字與啟動腳本必須是同一個字串。

        對不上時 `rerun_hint()` 會印出一個**不存在的環境變數**,
        照著貼上去跑等於什麼都沒設 —— 而輸出看起來完全正常。
        """
        assert ENV_FOR.get(SWITCH) == ENV, (
            f"⚠️⚠️ 契約表的 env 名({ENV_FOR.get(SWITCH)!r})與啟動腳本({ENV!r})不一致")

    def test_reference_value_is_off(self):
        """JET_REFERENCE(重跑建議值)必須是 0.0 = 關閉。

        ⚠️ 建議值是 `rerun_hint()` 直接印給人貼的。若這裡是 1.0,
           任何一次契約違規的重跑提示都會**順手把 S2 打開**,
           而使用者不會知道自己同時動了兩個變數。
        """
        assert JET_REFERENCE.get(SWITCH) == 0.0, (
            f"⚠️⚠️ JET_REFERENCE[{SWITCH}] = {JET_REFERENCE.get(SWITCH)!r},應為 0.0")

    def test_it_is_declared_bool(self):
        """⚠️⚠️ BOOL_PARAMS:契約表內部用 0.0/1.0,重跑提示必須印回 True/False。

        印成 `1.0` 貼上去會變成 `-p wait_goto_radius:=1.0` 餵給宣告為 BOOL 的
        參數 ⇒ InvalidParameterTypeException ⇒ 節點整個起不來。
        這與 INT_PARAMS 是**同一類坑的第三次**(:g 印整數、整數參數被 _f() 加
        小數點、bool 被印成浮點),三次都是「照著建議貼,然後整批報廢」。
        """
        assert SWITCH in BOOL_PARAMS, (
            f"⚠️⚠️ {SWITCH} 不在 BOOL_PARAMS ⇒ 重跑提示會印 1.0 ⇒ 節點起不來")

    def test_contract_verify_reads_it_back(self):
        """⚠️ 契約表加了列,**讀回映射必須同步加**。

        否則一律判 missing,而 missing 的訊息長得像「節點沒起來」,
        整批會在第一趟被自己的閘門擋掉(RPZ1 就是這個形狀)。
        """
        vsrc = (ROOT / "scripts" / "wsl" / "contract_verify.py").read_text(
            encoding="utf-8")
        assert f'"{SWITCH}"' in _code_only(vsrc), (
            f"⚠️⚠️ contract_verify.py 的讀回清單沒有 {SWITCH} ⇒ 判 missing ⇒ "
            f"整批在第一趟被自己的閘門擋掉(RPZ1 形狀)")


# ══════════════════════════════════════════════════════════════════════════
#  §4 致動點
# ══════════════════════════════════════════════════════════════════════════
class TestActuation:
    """⭐⭐ 開關開時 wait 段要求 `goto_radius`;開關關時仍退回 `hold_radius`。"""

    def test_the_on_path_asks_for_goto_radius(self):
        e = _wait_radius_ifexp()
        assert isinstance(e, ast.IfExp), (
            "⚠️ radius= 不再是三元運算式 —— 結構變了,底下所有斷言都失去意義")
        assert (isinstance(e.body, ast.Attribute)
                and e.body.attr == "goto_radius"), (
            f"⚠️⚠️ 開關打開時要求的不是 goto_radius(實得 {ast.unparse(e.body)})。\n"
            f"    S2 的**全部內容**就是「wait 段與 runin 段要求同一個半徑」;\n"
            f"    半徑不同 ⇒ `_goto` 那一發仍然是純半徑變更 ⇒ 仍被去重器吃掉。")

    def test_the_off_path_still_falls_back_to_hold_radius(self):
        """⭐ 反向鎖:兩顆開關都關時必須逐位元同 RRD。"""
        e = _wait_radius_ifexp()
        assert (isinstance(e.orelse, ast.Attribute)
                and e.orelse.attr == "hold_radius"), (
            f"⚠️⚠️ 關閉路徑不再退回 hold_radius(實得 {ast.unparse(e.orelse)})\n"
            f"    ⇒ 開關關著時就不是逐位元舊行為 ⇒ 對照組被污染 ⇒ "
            f"RRD 的「兩個母體」宣稱失去基準。")

    def test_the_switch_is_ORed_with_far_not_ANDed(self):
        """⭐⭐ 必須是 `far or self.wait_goto_radius`,**不是** `and`。

        ⚠️⚠️ 這是本檔最容易被寫錯、而且錯了完全靜默的一條:
           `far` 是 runin 段(還在 3 km 外)的旗標,wait 段的定義就是 far=False。
           寫成 `and` ⇒ wait 段恆偽 ⇒ 開關**永遠不會生效**,
           而 log、契約表、CSV 的 cmd_radius_m **全都看起來正常**
           (cmd_radius_m 在開與關時同樣是 30)。
           那就是本專案第 5 次「開關開著但那段從來沒真的發生」。
        """
        e = _wait_radius_ifexp()
        assert isinstance(e.test, ast.BoolOp) and isinstance(e.test.op, ast.Or), (
            f"⚠️⚠️ 條件不是 or:{ast.unparse(e.test)}\n"
            f"    寫成 and ⇒ wait 段(far=False)恆偽 ⇒ 開關是裝飾品,且完全靜默。")
        names = {ast.unparse(v) for v in e.test.values}
        assert names == {"far", f"self.{SWITCH}"}, (
            f"⚠️ 條件的兩項應該恰好是 far 與 self.{SWITCH},實得 {names}")

    def test_hold_radius_survives_where_orbiting_is_genuinely_wanted(self):
        """⚠️ 反向鎖:S2 不可以把 hold_radius 從整個節點掃掉。

        ALIGN 的守環(hold_ring)與 missed 的守點盤旋是**真的要繞圈**的。
        S2 只動 wait 段這一處;把 hold_radius 全域拔掉會讓那兩處退回
        `_reposition(param3=0)`,而 param3=0 **不通過** ArduPlane 的
        `param3 > 0` 閘門 ⇒ 退回 |WP_LOITER_RAD| ⇒ 半徑由飛控自己決定。

        ⚠️⚠️ 這條防的是 **M14**,而 M14 逃脫的原因是**這條鎖以前沒有鑑別力**:
           它數的是**全檔** `attr == "hold_radius"` 的 Attribute 節點(共 11 個),
           不是 `_goto(radius=self.hold_radius)` 的**呼叫點**(只有 2 個)。
           扣掉致動點還剩 10 個,其中 8 個是參數讀取(953)、log 字串
           (1110/1117/1143)、`_check_hold_radius()` 的告警文字
           (1227/1229/1234/1240)與閘門算式(2102)⇒ `len(others) >= 2` **恆真**。
           實測:把 3819(ALIGN 守環)與 4364(missed 守點)兩處真呼叫點都改成
           `radius=self.goto_radius`,本檔完全不紅;全套 1925 條也只有
           tests/test_miss_exit.py 抓到 missed 那一處,ALIGN 守環那一處零反應。
        **逃脫的後果**:守環與守點半徑退化成 30 m —— 遠小於物理迴轉半徑
        ⇒ 飛機飛不出那個圈、`_check_hold_radius()` 的告警文字仍然照印
        (它讀的是 self.hold_radius,不是實際送出去的值)⇒ 完全靜默。
        ⇒ 改成數**呼叫點**:既不寫死行號,也不與 hold_ring 重構耦合。
        """
        sites = []
        for n in ast.walk(TREE):
            if not (isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute)
                    and n.func.attr == "_goto"):
                continue
            for kw in n.keywords:
                if kw.arg == "radius" and ast.unparse(kw.value) == "self.hold_radius":
                    sites.append(n.lineno)
        assert len(sites) >= 2, (
            f"⚠️⚠️ `_goto(radius=self.hold_radius)` 的**呼叫點**只剩 {len(sites)} 個"
            f"(實得行號 {sites}),應該至少 2 個。\n"
            f"    ALIGN 的守環(hold_ring)與 missed 的守點盤旋是**真的要繞圈**的;\n"
            f"    S2 只動 wait 段那一處,不可以順手把它們一起換成 goto_radius。\n"
            f"    ⚠️ 注意這條數的是呼叫點,不是 hold_radius 的引用次數 —— 後者靠"
            f"log 字串與告警文字就恆真(M14 就是這樣逃掉的)。")


# ══════════════════════════════════════════════════════════════════════════
#  §5 儀表必須記在偵測器**之前**
# ══════════════════════════════════════════════════════════════════════════
class TestInstrumentationOrder:
    """⭐⭐ 整個 S2 的**判讀基礎**:req 每拍都寫、cmd 只在送出時更新。

    ⚠️⚠️ 因為「成功的指紋是指令的缺席」,判讀不可能靠「有沒有送出指令」。
       唯一能區分「開關生效了」與「開關沒生效」的量是
       **本拍要求的半徑**(`req_radius_m`)。它必須在
       `moved` 判斷**之前**無條件寫下 —— 寫在 if 裡面就變成
       「只有送出時才記錄」,那它就退化成 `cmd_radius_m` 的複本,
       兩欄同時失去鑑別力,而 CSV 看起來仍然完整。
    """

    def _goto_body(self):
        return _func("_goto").body

    def _index_of(self, pred):
        for i, s in enumerate(self._goto_body()):
            if pred(s):
                return i
        return None

    def test_the_requested_radius_comes_from_the_radius_argument(self):
        """⭐⭐ `_req = self.goto_radius if radius is None else radius` —— **來源**要鎖。

        這一行是把呼叫端的 `radius=` 語意接到「儀表」與「致動」的**唯一**一行。

        ⚠️⚠️ 這條防的是 **M13**:改成 `_req = self.goto_radius`。
           本類原有的三條(req 記在 moved 之前、sent 在送出分支裡、兩者記同一個
           `_req`)在這個突變下**全部照樣成立** ⇒ 本檔 33 條全綠、全套零反應。
        **逃脫的後果**(這是本檔存在的理由被直接抽掉):
          ① 整個節點的 `radius=` 參數變裝飾品 —— ALIGN 守環(3819)、
             missed 守點(4364)、S2 致動點(4494-4497)全部退化成 goto_radius=30,
             而上面那條 `_goto(radius=self.hold_radius)` 呼叫點的鎖仍然是綠的
             (呼叫點還在,只是沒人用它的值)。
          ② `req_radius_m` / `cmd_radius_m` 在開關**開與關時同樣是 30**
             ⇒ 判讀腳本 s2_readout.py 會判「✅ 全段 = goto_radius」
             ⇒ **假綠燈**,而 CSV 看起來完全正常。
        ⇒ 連「反過來寫」(`radius if radius is None else self.goto_radius`)
          也要擋掉,所以三個位置逐一比對,不是只看有沒有提到 radius。
        """
        cand = [s for s in self._goto_body()
                if isinstance(s, ast.Assign)
                and any(getattr(t, "id", None) == "_req" for t in s.targets)]
        assert len(cand) == 1, (
            f"⚠️⚠️ `_goto` 頂層的 `_req = …` 有 {len(cand)} 處,應該恰好 1 處 —— "
            f"S2 的判讀基礎(本拍要求的半徑)就是這一個量")
        e = cand[0].value
        assert isinstance(e, ast.IfExp), (
            f"⚠️⚠️ `_req` 不再是三元運算式,實得 `{ast.unparse(e)}`。\n"
            f"    寫死成常數 ⇒ 整個節點的 `radius=` 參數失效,而 req/cmd 兩欄在"
            f"開關開與關時同樣是 30 ⇒ s2_readout 會給**假綠燈**。")
        assert ast.unparse(e.test) == "radius is None", (
            f"⚠️⚠️ `_req` 的條件不是 `radius is None`,實得 `{ast.unparse(e.test)}`。\n"
            f"    ⚠️ 用 `not radius` 會把**合法的半徑 0** 一起吃掉換成 goto_radius"
            f"(0 是 ArduPlane 明確處理的一個 case)。")
        assert ast.unparse(e.body) == "self.goto_radius", (
            f"⚠️⚠️ 沒給 radius 時的預設不是 self.goto_radius,實得 "
            f"`{ast.unparse(e.body)}`")
        assert ast.unparse(e.orelse) == "radius", (
            f"⚠️⚠️ 有給 radius 時沒有照給,實得 `{ast.unparse(e.orelse)}` —— "
            f"呼叫端的 `radius=` 變裝飾品,而所有呼叫點在原始碼上看起來完全正常")

    def test_req_radius_is_recorded_before_the_change_detector(self):
        req_i = self._index_of(
            lambda s: isinstance(s, ast.Assign)
            and any(ast.unparse(t).startswith("self.req_radius[") for t in s.targets))
        moved_i = self._index_of(
            lambda s: isinstance(s, ast.Assign)
            and any(getattr(t, "id", None) == "moved" for t in s.targets))
        assert req_i is not None, "⚠️⚠️ `self.req_radius[i] = ...` 不見了 —— S2 無法判讀"
        assert moved_i is not None, "⚠️ 變更偵測器的 `moved = ...` 不見了"
        assert req_i < moved_i, (
            f"⚠️⚠️ req_radius 記在偵測器**之後**(req@{req_i} vs moved@{moved_i})。\n"
            f"    ⇒ 「本拍要求的半徑」變成有條件記錄 ⇒ 與 cmd_radius_m 同義 ⇒\n"
            f"      永遠分不出「有下令」與「有生效」,而 CSV 看起來仍然完整。")

    def test_req_radius_is_written_unconditionally(self):
        """⚠️ 它必須是 `_goto` 的**直接**子句,不可以躲在任何 if 裡。"""
        assert self._index_of(
            lambda s: isinstance(s, ast.Assign)
            and any(ast.unparse(t).startswith("self.req_radius[") for t in s.targets)
        ) is not None, (
            "⚠️⚠️ `self.req_radius[i] = ...` 不在 _goto 的頂層 ⇒ 有條件記錄 ⇒ "
            "S2 的判讀基礎沒了(見本類 docstring)")

    def test_sent_radius_is_written_inside_the_send_branch(self):
        """⭐ `sent_radius` 必須與 `_reposition()` 在**同一個** if 分支裡。

        ⚠️ 寫在分支外 ⇒ 它每拍都更新 ⇒ 變成 req 的複本;
           寫在別的分支 ⇒ 它記的是沒送出去的東西。兩種都是靜默的。
        """
        for s in self._goto_body():
            if not isinstance(s, ast.If):
                continue
            body = [ast.unparse(x) for x in s.body]
            if not any(x.startswith("self._reposition(") for x in body):
                continue
            assert any(x.startswith("self.sent_radius[") for x in body), (
                "⚠️⚠️ `self.sent_radius[i] = ...` 不在送出 DO_REPOSITION 的那個"
                "分支裡 ⇒ cmd_radius_m 記的不是真正送出去的半徑")
            return
        pytest.fail("⚠️ 找不到呼叫 `self._reposition()` 的那個 if 分支")

    def test_the_radius_actually_sent_is_the_one_recorded_as_cmd(self):
        """⭐⭐ `_reposition()` **實際送出**的半徑,必須與 `sent_radius` 記的是同一個量。

        ⚠️⚠️ 這條防的是 **M12**:把送出改成
           `self._reposition(i, lat, lon, alt, self.goto_radius)`,而
           `self.sent_radius[i] = _req` **不動** ⇒ 本檔 33 條全綠、全套零反應。
           上一條(`sent_radius` 在送出分支裡)只檢查兩個**句子在同一個 if body**,
           **引數完全沒有比對** —— 這就是缺口。
        **逃脫的後果**:`cmd_radius_m` 的**全部意義**就是「最後真正送出去的半徑」,
        兩者脫鉤之後這一欄可以記一個**從未上線的值**,而 CSV 看起來完全正常。
        而且同一個突變會順手毀掉 RRD 對照組:開關關閉時 wait 段**要求**
        hold_radius=1300、實際**送出** goto_radius=30 ⇒ 16/16 全部變直進、
        硬彎母體消失,但 CSV 讀出來 req=1300 / cmd=1300 ⇒ 判讀會認為
        關閉路徑完全正確。這是本檔 docstring 在防的那個假綠燈的最強形式。
        """
        for s in self._goto_body():
            if not isinstance(s, ast.If):
                continue
            calls = [x for x in ast.walk(s)
                     if isinstance(x, ast.Call)
                     and isinstance(x.func, ast.Attribute)
                     and x.func.attr == "_reposition"]
            if not calls:
                continue
            assert len(calls) == 1, (
                f"⚠️ 送出分支裡有 {len(calls)} 發 `_reposition()`,應該恰好 1 發")
            call = calls[0]
            # `_reposition(self, i, lat, lon, alt, radius=0.0)` ⇒ 半徑是第 5 個位置引數
            kw = {k.arg: k.value for k in call.keywords}
            if "radius" in kw:
                sent_expr = ast.unparse(kw["radius"])
            else:
                assert len(call.args) >= 5, (
                    f"⚠️⚠️ `_reposition()` 只給了 {len(call.args)} 個位置引數,"
                    f"**沒有帶半徑** ⇒ 吃預設 param3=0.0 ⇒ 不通過 ArduPlane 的 "
                    f"`param3 > 0` 閘門 ⇒ 退回 |WP_LOITER_RAD|,半徑由飛控自己決定;\n"
                    f"    而 cmd_radius_m 仍然照記 `_req` ⇒ CSV 看起來完全正常。")
                sent_expr = ast.unparse(call.args[4])
            recs = [ast.unparse(x.value) for x in s.body
                    if isinstance(x, ast.Assign)
                    and any(ast.unparse(t).startswith("self.sent_radius[")
                            for t in x.targets)]
            assert len(recs) == 1, (
                f"⚠️ 送出分支裡 `self.sent_radius[…] = …` 有 {len(recs)} 處,應恰好 1 處")
            assert sent_expr == "_req", (
                f"⚠️⚠️ 真正送給 `_reposition()` 的半徑是 `{sent_expr}`,不是 `_req`。\n"
                f"    `_req` 才是本拍**要求**的半徑(也是 req_radius_m 記的量);\n"
                f"    送出去的與記錄下來的一旦脫鉤,cmd_radius_m 就會記一個"
                f"**從未上線的值**,而 CSV 看起來完全正常。")
            assert recs[0] == sent_expr, (
                f"⚠️⚠️ 送出的是 `{sent_expr}`、記進 cmd_radius_m 的是 `{recs[0]}` "
                f"—— 兩者必須逐字相同,否則 cmd_radius_m 記的不是真正送出去的半徑。")
            return
        pytest.fail("⚠️ 找不到呼叫 `self._reposition()` 的那個 if 分支")

    def test_both_instruments_record_the_same_quantity(self):
        """⚠️ 兩欄必須記同一個 `_req`。

        記不同的東西(例如一個記 `radius` 參數、一個記 `self.goto_radius`)
        會讓「req == cmd」這個比較失去意義 —— 而那個比較正是判斷
        「這一拍到底有沒有送出去」的方法。
        """
        srcs = set()
        for n in ast.walk(_func("_goto")):
            if isinstance(n, ast.Assign):
                for t in n.targets:
                    u = ast.unparse(t)
                    if u.startswith(("self.req_radius[", "self.sent_radius[")):
                        srcs.add(ast.unparse(n.value))
        assert srcs == {"_req"}, (
            f"⚠️⚠️ req_radius / sent_radius 記的不是同一個量:{srcs}")


# ══════════════════════════════════════════════════════════════════════════
#  §6 反向鎖:偵測器裡**刻意**不含 radius
# ══════════════════════════════════════════════════════════════════════════
class TestTheDetectorDeliberatelyIgnoresRadius:
    """⭐⭐ **這條鎖的用途是防止有人好心「順手修掉」。**

    看到「純半徑變更被去重器吃掉」的第一直覺是把 radius 併進 `wp_pt`。
    ⚠️⚠️ **不要。** 那會讓開關**關閉**時 runin→wait 的那一發也被送出去,
       ⇒ RRD 的 16 架次會全部變成「起彎組」⇒ **對照組被毀掉**:
       「直進 11 架命中 10/11 vs 硬彎 5 架命中 0/5」這個唯一的證據
       就再也重現不了,而 5.9°–29.0° 之間的零樣本斷層也沒了。

    ⇒ 這是**有意的取捨,不是遺漏**:純風險、零收益。
      S2 的修法是讓「要求的半徑從一開始就不變」,不是讓偵測器更敏感。
      (真的要修偵測器,必須是另一次獨立實驗,而且要先把 RRD 重跑一遍。)
    """

    WHY = (
        "⚠️⚠️ 把 radius 併進 `_goto` 的變更偵測器 = **毀掉 RRD 對照組**。\n"
        "    開關關閉時 16/16 會全部變成起彎組,而「直進 10/11 vs 硬彎 0/5」\n"
        "    是 S2 唯一的證據基礎。這是**有意的取捨,不是遺漏** —— 不要順手修。\n"
        "    S2 的修法在致動點(讓要求的半徑從一開始就不變),不在偵測器。")

    def test_wp_pt_is_still_a_three_tuple_of_lat_lon_alt(self):
        for n in ast.walk(_func("_goto")):
            if (isinstance(n, ast.Assign)
                    and any(ast.unparse(t).startswith("self.wp_pt[") for t in n.targets)):
                assert isinstance(n.value, ast.Tuple), (
                    f"⚠️ wp_pt 存的不是 tuple:{ast.unparse(n.value)}\n{self.WHY}")
                got = [ast.unparse(e) for e in n.value.elts]
                assert got == ["lat", "lon", "alt"], (
                    f"⚠️⚠️ wp_pt 從 (lat, lon, alt) 變成 {got}\n{self.WHY}")
                return
        pytest.fail("⚠️ 找不到 `self.wp_pt[i] = ...`")

    #: `moved` 這個運算式**刻意不准變**。白名單:三段,逐字。
    #: ⚠️ 用 `ast.unparse` 的輸出比對(不是原始碼字串),所以換行/括號/空白
    #:    怎麼排都不影響;真正變的只有「多一項 / 少一項 / 改一項」。
    MOVED_VALUES = [
        "prev is None",
        "abs(prev[2] - alt) > 5.0",
        "dist_m(prev[:2], (lat, lon)) > move_tol",
    ]

    def test_the_moved_expression_is_exactly_these_three_terms(self):
        """⭐⭐ **正向白名單**:`moved` 只能是這三段,多一項就紅。

        ⚠️⚠️ 這條防的是 **M6b** —— 與 M6 語意完全相同、但走**旁路**的寫法:

            prev = self.wp_pt.get(i)
            r  = _req                    # 中介變數,識別字不含 radius
            pr = self.wp_r.get(i)        # 另開一個字典,wp_pt 形狀不動
            moved = (prev is None or abs(prev[2] - alt) > 5.0
                     or dist_m(prev[:2], (lat, lon)) > move_tol
                     or pr != r)

        底下那條黑名單式的反向鎖(識別字不得含 "radius"/`_req` + wp_pt 必須是
        三元組)對它**完全沒有反應**:容器換了、命名換了。實測本檔 33 條全綠、
        全套 1925 條零反應。
        ⚠️ 而「好心人順手修偵測器」最可能選的正是這條路 —— 開旁路比改 wp_pt
           的形狀更小、更「乾淨」。
        **逃脫的後果**與 M6 一模一樣:開關**關閉**時 runin→wait 那一發也會被送出去
        ⇒ RRD 的 16 架次全部變成硬彎組 ⇒ 對照組被毀掉,
        「直進 10/11 vs 硬彎 0/5」這個唯一的證據再也重現不了,
        5.9°–29.0° 之間的零樣本斷層也沒了。
        """
        for s in _func("_goto").body:
            if not (isinstance(s, ast.Assign)
                    and any(getattr(t, "id", None) == "moved" for t in s.targets)):
                continue
            e = s.value
            assert isinstance(e, ast.BoolOp) and isinstance(e.op, ast.Or), (
                f"⚠️⚠️ `moved` 不再是 or 串起來的判斷式:{ast.unparse(e)}\n{self.WHY}")
            got = [ast.unparse(v) for v in e.values]
            assert got == self.MOVED_VALUES, (
                f"⚠️⚠️ `moved` 的判斷式被改過。\n"
                f"    期望(逐字,共 {len(self.MOVED_VALUES)} 段):{self.MOVED_VALUES}\n"
                f"    實得(共 {len(got)} 段):{got}\n{self.WHY}\n"
                f"    ⚠️ 多出來的那一段若是**半徑的旁路**(例如另開 self.wp_r "
                f"字典、或先把 _req 指給中介變數),語意與「把 radius 併進 wp_pt」"
                f"完全相同 —— 換容器換命名逃不掉這條白名單。")
            return
        pytest.fail("⚠️ 找不到 `moved = ...`")

    def test_the_moved_expression_never_mentions_radius(self):
        """⚠️ 黑名單(保留):上面那條白名單的補充,失敗訊息會直接指出是哪個識別字。"""
        for s in _func("_goto").body:
            if (isinstance(s, ast.Assign)
                    and any(getattr(t, "id", None) == "moved" for t in s.targets)):
                idents = set()
                for n in ast.walk(s.value):
                    if isinstance(n, ast.Name):
                        idents.add(n.id)
                    elif isinstance(n, ast.Attribute):
                        idents.add(n.attr)
                bad = {x for x in idents if "radius" in x or x == "_req"}
                assert not bad, (
                    f"⚠️⚠️ `moved` 的判斷式引用了半徑:{bad}\n{self.WHY}")
                return
        pytest.fail("⚠️ 找不到 `moved = ...`")


# ══════════════════════════════════════════════════════════════════════════
#  §7 CSV 欄位契約
# ══════════════════════════════════════════════════════════════════════════
#: RRD 批次(S2 部署**之前**)的實測表頭,逐字取自
#: runs/artifacts/RRD1/tot_RRD1.csv 的第一行。
#: ⚠️ 這 25 欄的**順序一個都不能動**:_tot_plot.py / _dive_plot.py 用
#:    r[0]…r[8] 位置解析(r[0]=t、r[1]=sysid、r[2]=lat、r[3]=lon、r[5]=gs、
#:    r[6]=d_slot、r[7]=mode、r[8]=v_cmd),插在中間會把每一欄靜靜平移一格。
RRD_HEADER = [
    "t", "sysid", "lat", "lon", "alt", "gs", "d_slot", "mode", "v_cmd",
    "slot_lat", "slot_lon", "phase", "agl", "gam_deg",
    "pred_residual_alt_m", "pred_touch_gs", "pred_secs_to_target",
    "pred_over_limit_frac", "secs_to_target", "early_by_s", "weave_yaw_deg",
    "early_by_m", "itcg_accel", "itcg_surplus_m", "lead_angle_deg",
]
S2_COLUMNS = ["req_radius_m", "cmd_radius_m", "wp_lat", "wp_lon"]
#: ⭐⭐ B1(2026-08-19,Jiang 2021 MPNG)的六欄儀表,**追加在 S2 之後**。
#: ⚠️ 這一組存在的理由與 S2 完全相同:req 是判定量(每拍都寫、夾制前),
#:    cmd 是佐證量(夾制後)。拿 cmd 判「機制生效」會給假綠燈。
#: ⚠️⚠️ 而它比 S2 更急:itcg_accel 這條通道 80,760 個樣本**全零** ——
#:    「開關開著但那一段從來沒真的發生」已經是第五次。
B1_COLUMNS = ["itcg_req_a", "itcg_cmd_a", "itcg_eps_t",
              "itcg_tgo_ref", "itcg_tgo_jiang", "itcg_sat"]


class TestCsvColumnContract:
    """⭐ 新欄位**只能尾端追加**,而且各組的順序固定。

    ⚠️ 追加了 B1 六欄之後,S2 那四欄不再位於尾端 —— 但它們的**位置**
       (緊接在 RRD 的 25 欄之後)與**組內順序**仍然不可以動,因為
       走 DictReader 的判讀腳本靠欄名、走位置解析的靠前 25 欄。
    """

    def test_the_four_new_columns_are_at_the_tail_in_order(self):
        h = _header()
        n = len(RRD_HEADER)
        assert h[n:n + 4] == S2_COLUMNS, (
            f"⚠️⚠️ S2 四欄不在 RRD 表頭之後,實得 {h[n:n + 4]}。\n"
            f"    req 與 cmd 一旦互換,判讀會拿 hold-last-value 當「本拍要求」\n"
            f"    ⇒ 開關開與關同樣讀到 30 ⇒ **假綠燈**。")
        assert h[-len(B1_COLUMNS):] == B1_COLUMNS, (
            f"⚠️⚠️ 表頭尾端不是 B1 的六欄,實得 {h[-len(B1_COLUMNS):]}。\n"
            f"    itcg_req_a 是判定量(夾制前、每拍都寫),itcg_cmd_a 是佐證量。\n"
            f"    兩者互換 ⇒ 又一次拿 hold-last-value 當證據。")

    def test_nothing_before_them_moved(self):
        """⭐⭐ 前 25 欄必須逐字等於 RRD 的實測表頭。"""
        h = _header()
        assert h[:len(RRD_HEADER)] == RRD_HEADER, (
            "⚠️⚠️ S2 之前的欄位順序被動過 —— _tot_plot.py / _dive_plot.py 是"
            "位置解析(r[0..8]),插在中間會把每一欄靜靜平移一格,"
            "而畫出來的圖**看起來完全正常**。\n"
            f"    期望前 {len(RRD_HEADER)} 欄:{RRD_HEADER}\n"
            f"    實得:{h[:len(RRD_HEADER)]}")

    def test_the_header_is_exactly_the_old_one_plus_four(self):
        # ⚠️ 2026-08-19:B1 又追加了六欄。這條鎖的意思一直是
        #    「**只准尾端追加**、既有欄位一格都不准動」,所以是
        #    RRD + S2 + B1,不是「永遠只有四欄」。
        assert _header() == RRD_HEADER + S2_COLUMNS + B1_COLUMNS

    def test_the_recorded_rrd_csv_still_matches(self):
        """現實錨點:磁碟上的 RRD 資料必須與 RRD_HEADER 逐字相同。

        ⚠️ runs/ 沒進版控 ⇒ 乾淨 checkout 會 skip。有檔案時才對。
        """
        p = ROOT / "runs" / "artifacts" / "RRD1" / "tot_RRD1.csv"
        if not p.exists():
            pytest.skip("runs/artifacts/RRD1 不在(未進版控)")
        with p.open(encoding="utf-8") as f:
            got = f.readline().strip().split(",")
        assert got == RRD_HEADER, (
            f"⚠️ RRD1 的實測表頭與本檔寫死的 RRD_HEADER 不一致 —— "
            f"其中一邊過期了\n    實得:{got}")

    def test_row_writes_exactly_as_many_values_as_the_header(self):
        """⚠️ 表頭與 `_row()` 的欄數不一致 = 整份 CSV 從錯開的那一欄起全錯,

        而 csv 模組**不會吭聲**,DictReader 只會把多的塞進 None、少的補 None。
        """
        n_head = len(_header())
        n_row = len(_writerow_list(_func("_row")).elts)
        assert n_head == n_row, (
            f"⚠️⚠️ 表頭 {n_head} 欄、_row() 寫 {n_row} 個值 —— 靜默錯位")

    def test_req_and_cmd_are_not_swapped(self):
        """⭐⭐ `req_radius_m` 必須來自 `req_radius`,`cmd_radius_m` 來自 `sent_radius`。

        ⚠️⚠️ 這兩欄互換是**最危險**的單一錯誤:兩欄的型別、量綱、數量級全都一樣,
           CSV 看起來完全正常,而判讀會拿 hold-last-value(開與關同樣是 30)
           當作「本拍要求的半徑」⇒ 對 S2 給出**假綠燈**,對 RRD 給出假紅燈。
        """
        row = _func("_row")
        binding = {}
        for n in ast.walk(row):
            if isinstance(n, ast.Assign) and len(n.targets) == 1:
                t = ast.unparse(n.targets[0])
                if t in ("_rr", "_sr", "_wp"):
                    binding[t] = ast.unparse(n.value)
        assert "req_radius" in binding.get("_rr", ""), (
            f"⚠️⚠️ `_rr` 綁的不是 self.req_radius:{binding.get('_rr')}")
        assert "sent_radius" in binding.get("_sr", ""), (
            f"⚠️⚠️ `_sr` 綁的不是 self.sent_radius:{binding.get('_sr')}")
        assert "wp_pt" in binding.get("_wp", ""), (
            f"⚠️ `_wp` 綁的不是 self.wp_pt:{binding.get('_wp')}")

        elts = _writerow_list(row).elts
        h = _header()
        col = {name: ast.unparse(elts[i]) for i, name in enumerate(h)}
        assert "_rr" in col["req_radius_m"] and "_sr" not in col["req_radius_m"], (
            f"⚠️⚠️ req_radius_m 欄寫的不是 _rr:{col['req_radius_m']}")
        assert "_sr" in col["cmd_radius_m"] and "_rr" not in col["cmd_radius_m"], (
            f"⚠️⚠️ cmd_radius_m 欄寫的不是 _sr:{col['cmd_radius_m']}")

    def test_wp_lat_comes_before_wp_lon(self):
        """⚠️ lat/lon 互換是靜默的:兩個都是合法浮點,只是把航點丟到地球另一邊。

        wp_lat/wp_lon 的用途是把「本拍瞄的航點」與各機環點綁定。綁錯的後果
        已經量過:RRD1/U2 綁錯之後航跡比從 2.02 變 0.90,一架**硬彎機被判成
        直進** —— 而它正是受測對象。
        """
        row = _func("_row")
        elts = _writerow_list(row).elts
        col = {name: ast.unparse(elts[i]) for i, name in enumerate(_header())}
        assert col["wp_lat"].endswith("_wp[0]"), (
            f"⚠️⚠️ wp_lat 欄寫的不是 _wp[0]:{col['wp_lat']}")
        assert col["wp_lon"].endswith("_wp[1]"), (
            f"⚠️⚠️ wp_lon 欄寫的不是 _wp[1]:{col['wp_lon']}")


# ══════════════════════════════════════════════════════════════════════════
#  §8 未設值寫空字串,不寫 0
# ══════════════════════════════════════════════════════════════════════════
class TestUnsetWritesEmptyStringNotZero:
    """⚠️⚠️ **0 是合法的半徑值**,不能拿它當「沒設過」的哨兵。

    `_reposition(param3=0)` 是 ArduPlane 明確處理的一個 case
    (`GCS_MAVLink_Plane.cpp:599` 的 `param3 > 0` 閘門不通過 ⇒ 退回
    |WP_LOITER_RAD|)⇒ 0 在這一欄有真正的意義。
    用 0 當「還沒設」會讓事後判讀把「節點剛起來、還沒下過任何指令」
    讀成「要求了半徑 0」,而那是一個**災難級的指令**,看起來卻像正常資料。

    這與 pred_* / early_by_* 那幾欄是同一條規則的第 N 次(見 `_row()` 的註解:
    「用 0 當『沒跑』會讓事後判讀把停用誤讀成『預測殘餘為零』」)。
    """

    @pytest.mark.parametrize("name", S2_COLUMNS)
    def test_the_sentinel_is_an_empty_string(self, name):
        elts = _writerow_list(_func("_row")).elts
        e = elts[_header().index(name)]
        assert isinstance(e, ast.IfExp), (
            f"⚠️ {name} 欄不是 `\"\" if ... is None else ...` 形式:{ast.unparse(e)}")
        assert isinstance(e.body, ast.Constant) and e.body.value == "", (
            f"⚠️⚠️ {name} 的「未設值」哨兵是 {ast.unparse(e.body)},不是空字串。\n"
            f"    0 是**合法半徑** ⇒ 拿 0 當哨兵會把「還沒下過指令」讀成"
            f"「要求了半徑 0」。")
        assert isinstance(e.test, ast.Compare) and isinstance(
            e.test.ops[0], ast.Is), (
            f"⚠️ {name} 的哨兵判斷不是 `is None`:{ast.unparse(e.test)}\n"
            f"    用 `if not _rr` 會把**合法的 0** 一起吃掉,寫成空字串。")
        assert isinstance(e.test.comparators[0], ast.Constant) and \
            e.test.comparators[0].value is None, (
            f"⚠️ {name} 的哨兵不是與 None 比較:{ast.unparse(e.test)}")
