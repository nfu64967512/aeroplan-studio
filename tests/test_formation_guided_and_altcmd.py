# -*- coding: utf-8 -*-
"""編隊層自行建立 GUIDED(A)+ tot CSV 的指令高度欄(B)。

使用者 2026-08-25 授權兩件事:「編隊層可以自己切 guided 順便可以加上指令高度」。

═══════════════════════════════════════════════════════════════════════════
 ⚠️⚠️⚠️ A 這件事**做過一次而且報廢過整批航次**
═══════════════════════════════════════════════════════════════════════════
`mavros_interface.py` 的註解逐字記著:

    2026-08-10 我把這裡改成帶 CHANGE_MODE,整批 RPG/RPH/RPI 報廢。
    master_node 啟動 2 秒就自稱 airborne 並以 1 Hz 送巡邏航點。
    帶了 CHANGE_MODE 之後,這些航點**每秒把長機從 TAKEOFF 拽回 GUIDED**
    ⇒ 起飛被取消 ⇒ UAV1 gs_max=0.0 整趟停在跑道上。

⇒ 本檔的第一等公民是 `test_takeoff_mode_is_never_overridden` ——
   它鎖的正是那次事故的解藥。其餘閘門可以討論,那一條不行。

═══════════════════════════════════════════════════════════════════════════
 B 為什麼需要指令高度
═══════════════════════════════════════════════════════════════════════════
WEAVEAIM1 量到高度振盪 ±40 m/s、週期 15.3–23.8 s,而 phugoid 理論值
π√2·V/g = 55.4–63.6 s ⇒ 快 2.7–3.6 倍 ⇒ **控制迴路振盪,不是機體模態**。
但要定案是哪一條迴路,必須能比對「指令高度 vs 實得高度」——
而 CSV 先前**只有實得**,分不出「我們一直在改指令」與「指令穩定但追不上」,
那兩者的修法完全相反。
"""
import ast
import importlib
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PATCHES = REPO / "scripts" / "wsl" / "sar_swarm_patches"
WSL = REPO / "scripts" / "wsl"
if str(PATCHES) not in sys.path:
    sys.path.insert(0, str(PATCHES))

MI_SRC = (PATCHES / "mavros_interface.py").read_text(encoding="utf-8")
TOT_SRC = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
TOT_TREE = ast.parse(TOT_SRC)


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")

    class _Node:
        def __init__(self, *a, **k):
            pass

    node_mod.Node = _Node
    qos_mod = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, *a, **k):
            self.kw = k

    class _Policy:
        RELIABLE = BEST_EFFORT = 1
        KEEP_LAST = KEEP_ALL = 1
        TRANSIENT_LOCAL = 2
        VOLATILE = 3

    qos_mod.QoSProfile = _QoSProfile
    qos_mod.ReliabilityPolicy = _Policy
    qos_mod.HistoryPolicy = _Policy
    qos_mod.DurabilityPolicy = _Policy

    class _Preset:
        value = _QoSProfile()

    class _Presets:
        SENSOR_DATA = SYSTEM_DEFAULT = SERVICES_DEFAULT = _Preset

    qos_mod.QoSPresetProfiles = _Presets
    rclpy.node, rclpy.qos = node_mod, qos_mod
    std = types.ModuleType("std_msgs")
    msg = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self, data=""):
            self.data = data

    msg.String = _String
    std.msg = msg

    for name, mod in (("rclpy", rclpy), ("rclpy.node", node_mod),
                      ("rclpy.qos", qos_mod), ("std_msgs", std),
                      ("std_msgs.msg", msg)):
        sys.modules.setdefault(name, mod)


_install_ros_stubs()
tot_node = importlib.import_module("tot_node")
TotNode = tot_node.TotNode


# ══════════════════════════════════════════════════════════════════════
#  A:_may_change_mode 的閘門
# ══════════════════════════════════════════════════════════════════════
def _extract(name):
    """把某個方法的**原始碼**抽出來 exec —— 測真的碼,但不吃模組的相依。

    ⚠️ `mavros_interface` import 了 mavros_msgs / nav_msgs / sar_swarm,
       全部樁起來要一長串,而且我補了三輪還在漏(每漏一個就是一輪紅燈)。
       本檔只測 `_may_change_mode`,它是 armed()/in_takeoff_mode()/position()
       與兩個屬性的純函式 —— 抽出來就好,和 test_nz_closed_loop 同一個手法。
    """
    tree = ast.parse(MI_SRC)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {}
            exec(compile(mod, "<extracted>", "exec"), ns)   # noqa: S102
            return ns[name]
    raise AssertionError(f"抽不到 {name} —— 錨點漂了")


_MAY = _extract("_may_change_mode")


class _Shim:
    """只有 `_may_change_mode` 真的會碰到的東西。"""

    def __init__(self, on, armed, takeoff, alt, min_alt, auto=False):
        self._change_mode = on
        self._change_mode_min_alt = min_alt
        self._armed, self._takeoff, self._alt = armed, takeoff, alt
        self._auto = auto

    def armed(self):
        return self._armed

    def in_takeoff_mode(self):
        return self._takeoff

    def in_auto_mode(self):
        return self._auto

    def position(self):
        return None if self._alt is None else (25.0, 121.0, self._alt)

    def _may_change_mode(self):
        return _MAY(self)


def _mav(on=True, armed=True, takeoff=False, alt=100.0, min_alt=15.0,
         auto=False):
    return _Shim(on, armed, takeoff, alt, min_alt, auto)


def test_the_shim_covers_every_method_the_gate_calls():
    """⚠️⚠️⚠️ **這一條在測「測試自己」,而它有前科。**

    `_may_change_mode` 的最後一支是 `except Exception: return False`,
    而那一支會把「樁少了一個方法」吞成一個**看起來合理**的 False。
    2026-08-25 加第四道閘門(`in_auto_mode`)時實際發生:樁還沒補,
    於是四條既有的鎖同時轉紅,但紅的訊息是「應為 True 卻得 False」——
    它指向閘門邏輯,不指向樁。要不是同時有正向斷言(必須為 True 的案例),
    這個缺口會表現成「所有閘門都更嚴格了」而**靜靜通過**。

    ⇒ 直接比對:原始碼裡 `self.<name>()` 呼叫到的每一個方法,樁都要有。
      這是 [[instrument-that-cannot-fire]] 的同一條教訓 ——
      每條測試都要先斷言「這個設定真的能讓 X 發生」。
    """
    fn = None
    for node in ast.walk(ast.parse(MI_SRC)):
        if isinstance(node, ast.FunctionDef) and node.name == "_may_change_mode":
            fn = node
    assert fn is not None, "抽不到 _may_change_mode —— 錨點漂了"
    called = {
        n.func.attr for n in ast.walk(fn)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and isinstance(n.func.value, ast.Name) and n.func.value.id == "self"
    }
    assert called, "一個 self.x() 都沒抽到 —— 這條鎖自己壞了"
    missing = sorted(m for m in called if not hasattr(_mav(), m))
    assert not missing, (
        f"⚠️⚠️ `_may_change_mode` 會呼叫 {missing},但 _Shim 沒有 ⇒ "
        f"AttributeError 會被它自己的 `except Exception: return False` 吞掉,"
        f"於是本檔每一條鎖都在測一個**恆為 False** 的函式。補進 _Shim。")


def test_switch_off_is_bit_identical_old_behaviour():
    """⚠️ 開關關著時**任何**狀態都不可以帶旗標 —— 那是逐位元舊行為。"""
    for armed in (True, False):
        for takeoff in (True, False):
            for alt in (0.0, 100.0, 5000.0):
                s = _mav(on=False, armed=armed, takeoff=takeoff, alt=alt)
                assert s._may_change_mode() is False


def test_takeoff_mode_is_never_overridden():
    """⭐⭐⭐ 本檔存在的理由,鎖的是 2026-08-10 報廢事故的解藥。

    飛控在 TAKEOFF 模式爬升時,模式**歸它所有**。當年無條件帶旗標,
    1 Hz 的巡邏航點每秒把長機拽回 GUIDED ⇒ 起飛被取消 ⇒
    UAV1 gs_max=0.0 整趟停在跑道上。
    """
    s = _mav(on=True, armed=True, takeoff=True, alt=500.0)
    assert s._may_change_mode() is False, (
        "在 TAKEOFF 模式下帶了 CHANGE_MODE —— 這正是 2026-08-10 整批報廢的成因")


def test_all_gates_must_pass():
    """⚠️ 名字從 `three` 改成不帶數字(2026-08-25 加了第四道 in_auto_mode)——
    把數量寫進測試名的下場是每加一道就要改名,而改名會讓 git 看起來像
    「刪了一條鎖」。條件清單在下面,數量由它自己說。"""
    assert _mav(on=True, armed=True, takeoff=False, alt=100.0)._may_change_mode() is True
    # 逐一破壞,每一項都必須擋下來
    assert _mav(on=True, armed=False, takeoff=False, alt=100.0)._may_change_mode() is False
    assert _mav(on=True, armed=True, takeoff=True, alt=100.0)._may_change_mode() is False
    assert _mav(on=True, armed=True, takeoff=False, alt=10.0)._may_change_mode() is False
    assert _mav(on=True, armed=True, takeoff=False, alt=100.0,
                auto=True)._may_change_mode() is False


def test_auto_mode_is_never_overridden():
    """⭐⭐⭐ 第四道閘門(2026-08-25):AUTO 期間模式歸飛控所有。

    這是 `test_takeoff_mode_is_never_overridden` 的**同型**。當年是編隊層
    把長機從 TAKEOFF 拽回 GUIDED ⇒ 起飛被取消 ⇒ 整批 RPG/RPH/RPI 報廢。
    新架構下飛機會在 AUTO 飛事前規劃的任務航線,而編隊層的一發
    DO_REPOSITION 帶著 CHANGE_MODE 就會把它拽出來 —— 任務中斷、
    攻擊發起點不去了,**而且完全靜默**(沒有人在讀模式)。

    ⚠️ 高度給 5000:確保擋下來的是 AUTO 這一條,不是高度那一條。
    """
    s = _mav(on=True, armed=True, takeoff=False, alt=5000.0, auto=True)
    assert s._may_change_mode() is False, (
        "在 AUTO(任務航線)下帶了 CHANGE_MODE —— 這與 2026-08-10 "
        "把長機從 TAKEOFF 拽出來是同一個機制")
    # 反向:同一組狀態、只把 auto 關掉,就必須放行 ⇒ 證明上面擋下來的
    # 真的是 AUTO 這一條,而不是別的條件順手擋掉的(假綠燈防護)。
    assert _mav(on=True, armed=True, takeoff=False, alt=5000.0,
                auto=False)._may_change_mode() is True


def test_missing_pose_falls_to_the_safe_side():
    """⚠️ 讀不到位姿要回 False,不是 True —— 不知道在哪就不該搶模式。"""
    s = _mav(on=True, armed=True, takeoff=False, alt=None)
    assert s._may_change_mode() is False


def test_the_altitude_gate_matches_the_fsm_threshold():
    """⚠️ 15 m 要和 FlightFSM / _keep_guided 同一個門檻,不可以各自為政。"""
    assert 'GUIDED_MIN_ALT", "15"' in MI_SRC
    s = _mav(on=True, alt=15.0)
    assert s._may_change_mode() is False, "門檻應為**嚴格大於**"
    assert _mav(on=True, alt=15.1)._may_change_mode() is True


def test_the_send_site_actually_consults_the_gate():
    """⚠️ 值層級測不到「送出處有沒有問閘門」—— 這條只好看結構。

    ⚠️ 用 AST 找**真正的** command_int_send 呼叫,不要用字串切割:
       `MAV_CMD_DO_REPOSITION` 第一次出現是在**註解**裡,
       切到那裡就會誤判成「送出處沒問閘門」(我第一版就是這樣紅的)。
    判準:param2 那個位置不可以是寫死的常數。
    """
    tree = ast.parse(MI_SRC)
    found = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "command_int_send"):
            found.append(node)
    assert found, "找不到 command_int_send —— 錨點漂了"
    for call in found:
        src = ast.dump(call)
        assert "_may_change_mode" in src, (
            "DO_REPOSITION 的送出處沒有問 _may_change_mode ⇒ 開關是死的")
        assert "MAV_DO_REPOSITION_FLAGS_CHANGE_MODE" in src


def test_the_switch_is_logged_either_way():
    """⚠️ 它只走環境變數、不在契約表 ⇒ 不印的話事後翻 log **無法判斷
       那一趟開著沒有**。開或關都要印([[pn-gains-outside-contract]])。"""
    # ⚠️ 兩個分支都要在:只印「開著」的話,關著那趟事後查不出來,
    #    而「沒有訊息」同時也相容於「這版沒有這個功能」。
    assert "**會**帶 CHANGE_MODE" in MI_SRC, "開著時沒有告警"
    assert "不帶 CHANGE_MODE" in MI_SRC, "關著時沒有留下任何痕跡"
    # 開著要用 warning(關著 info 就夠)
    seg = MI_SRC.split("if self._change_mode:")[1][:400]
    assert "warning" in seg, "開著時只用 info ⇒ 在一整片 log 裡看不見"


@pytest.mark.parametrize("script,needle", [
    ("rep_interactive.sh", "REPOSITION_CHANGE_MODE"),
    ("rep_interactive.sh", "export REPOSITION_CHANGE_MODE"),
    ("formation_config.sh", "REPOSITION_CHANGE_MODE"),
])
def test_the_interactive_path_wires_it(script, needle):
    assert needle in (WSL / script).read_text(encoding="utf-8")


# ══════════════════════════════════════════════════════════════════════
#  B:指令高度欄
# ══════════════════════════════════════════════════════════════════════
def _hdr():
    best = None
    for n in ast.walk(TOT_TREE):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == "writerow" and n.args
                and isinstance(n.args[0], (ast.List, ast.Tuple))
                and len(n.args[0].elts) > 20):
            elts = n.args[0].elts
            strs = sum(1 for e in elts
                       if isinstance(e, ast.Constant) and isinstance(e.value, str))
            if strs > len(elts) * 0.8 and (best is None or n.lineno < best.lineno):
                best = n
    assert best is not None
    return [e.value for e in best.args[0].elts
            if isinstance(e, ast.Constant) and isinstance(e.value, str)]


def test_alt_cmd_is_in_the_header_next_to_alt():
    h = _hdr()
    assert "alt_cmd" in h, h[-6:]
    assert "alt" in h, "實得側不見了 —— 指令側單獨存在沒有意義"


def test_alt_cmd_is_recorded_at_the_single_choke_point():
    """⚠️ 必須記在 `_reposition`(唯一送出口)。

    `_goto` / `_goto_latched` / DIVE 鎖存路徑最後都收斂到它;記在呼叫端
    會漏掉其中一條,而**漏掉的那條不會報錯** —— CSV 只會少幾拍,
    看起來像遙測抖動。
    """
    for fn in ast.walk(TOT_TREE):
        if isinstance(fn, ast.FunctionDef) and fn.name == "_reposition":
            body = ast.dump(fn)
            assert "alt_cmd" in body, "_reposition 沒有記錄指令高度"
            return
    pytest.fail("找不到 _reposition")


def test_alt_cmd_writes_empty_not_zero_when_never_sent():
    """⚠️ 0 是**合法**的指令高度(地面)。沒送過要寫空字串。"""
    assert 'self.alt_cmd.get(i, "")' in TOT_SRC, (
        "沒送過時寫的不是空字串 ⇒ 「還沒送過」會被讀成「指令高度 0」")


def test_alt_cmd_is_cleared_between_strikes():
    assert "alt_cmd" in getattr(tot_node, "RESET_DICTS", ()), (
        "指令高度沒有逐發清 ⇒ 第二發第一拍會寫出上一發的值")


def test_alt_cmd_round_trips_through_reposition():
    """⭐ 值層級:真的呼叫 `_reposition`,看那一欄拿到什麼。"""
    s = TotNode.__new__(TotNode)
    s.alt_cmd = {}
    sent = {}

    class _L:
        target_system = 1

        class mav:
            @staticmethod
            def command_int_send(*a, **k):
                sent["n"] = sent.get("n", 0) + 1

    s.link = {1: _L()}
    s._reposition(1, 25.0, 121.0, 3100.0, radius=30.0)
    assert s.alt_cmd[1] == pytest.approx(3100.0)
    assert sent.get("n") == 1, "指令沒有真的送出去 —— 這條就不是端到端了"
    # 0 必須被記成 0,不是被當成「沒送過」
    s._reposition(1, 25.0, 121.0, 0.0, radius=30.0)
    assert s.alt_cmd[1] == 0.0
    assert s.alt_cmd.get(1, "") == 0.0
