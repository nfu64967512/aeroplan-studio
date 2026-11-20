# -*- coding: utf-8 -*-
"""ALIGN 蛇形瞄準點:寫死的 120 m 落在物理最小轉彎半徑之內。

═══════════════════════════════════════════════════════════════════════════
 為什麼(2026-08-25)
═══════════════════════════════════════════════════════════════════════════
舊行為:

    aim = dest(pos, bearing_to_slot + sign*theta, 120.0)   # 機頭前 120 m
    self._goto(i, aim[0], aim[1], ...)                     # radius=None ⇒ 30

`_goto(radius=None)` ⇒ `goto_radius=30`。ArduPlane 的 DO_REPOSITION 只要
param3>0 就一律走 `update_loiter()`,L1 再 ×eas2tas²(3100 m ≈1.365)
⇒ **實際繞圈半徑 ~41 m**,而 110 m/s 飛 41 m 的圈需要

    atan(V² / (g·R)) = atan(110² / (9.807·41)) = 88.1°

⇒ 被 `ROLL_LIMIT_DEG=80` 夾住 ⇒ 整段釘在坡度上限。

實測(FORMD1 / FORME1,**先對 (t, sysid) 去重** —— 原始檔 50% 是重複列):

    weave 段     p50 60.8-61.1°、p90 78.9-79.2°、38-39% 超過 71.6°
    straight 段  p50 0.1°

⚠️ 這是 **weave vs straight** 的差,不是長機 vs 僚機:FORME1 兩機同時
   蛇形的 1779 個配對拍,配對差中位數 **+0.02°**。

⚠️⚠️ 本檔第一等公民是 `test_the_aim_point_never_lands_behind_the_slot` ——
   `max(min_m, frac*d)` 在 d 小的時候會把瞄準點推到 slot **後方**,
   而 `bearing(pos→slot) + sign*θ` 這個構造會把飛機往**離開** slot 的方向
   送,距離越遠偏得越多 ⇒ 無界發散,且完全靜默(mode 欄照樣寫 weave)。
"""
import ast
import importlib
import math
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PATCHES = REPO / "scripts" / "wsl" / "sar_swarm_patches"
WSL = REPO / "scripts" / "wsl"
if str(PATCHES) not in sys.path:
    sys.path.insert(0, str(PATCHES))


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
SRC = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
TREE = ast.parse(SRC)

G = 9.80665


def _node(min_m, frac):
    s = TotNode.__new__(TotNode)
    s.weave_aim_min_m = float(min_m)
    s.weave_aim_frac = float(frac)
    return s


def _min_turn_radius(v_mps, bank_deg):
    """協調轉彎的最小半徑 V²/(g·tanφ)。"""
    return v_mps ** 2 / (G * math.tan(math.radians(bank_deg)))


# ══════════════════════════════════════════════════════════════════════
#  ① 這個問題真的存在嗎(儀表能不能開火的等價物)
# ══════════════════════════════════════════════════════════════════════
def test_the_old_120m_really_is_inside_the_minimum_turn_radius():
    """⭐ 先證明「這個修法有東西可修」。

    如果 120 m 其實大於最小轉彎半徑,那下面每一條測試都在測一個
    不存在的問題 —— 而那種測試會全綠、看起來像守衛,其實什麼都沒守。
    """
    for v, bank in ((110.0, 80.0), (157.0, 80.0)):
        r = _min_turn_radius(v, bank)
        assert r > 120.0, (
            "V=%.0f m/s、%.0f° 坡度的最小轉彎半徑只有 %.0f m —— "
            "本檔的前提不成立" % (v, bank, r))
    # 具體數字釘住,免得日後有人改了速度假設卻沒發現結論已經變了
    assert _min_turn_radius(110.0, 80.0) == pytest.approx(218.0, abs=2.0)
    assert _min_turn_radius(157.0, 80.0) == pytest.approx(443.0, abs=3.0)


def test_a_41m_loiter_needs_more_bank_than_the_roll_limit():
    """goto_radius=30 ×eas2tas²(3100 m ≈1.365)⇒ ~41 m。"""
    r = 30.0 * 1.365
    need = math.degrees(math.atan(110.0 ** 2 / (G * r)))
    assert need > 80.0, "41 m 的圈只需要 %.1f° —— 那就不會釘在上限了" % need
    assert need == pytest.approx(88.1, abs=0.5)


# ══════════════════════════════════════════════════════════════════════
#  ② 開關語意
# ══════════════════════════════════════════════════════════════════════
def test_zero_means_bit_identical_old_behaviour():
    """⚠️ 新開關預設關閉 = 逐位元舊行為,和 release_at_ring / nz_closed_loop
       同一條規矩。關著的時候**必須**回 120.0,一個位元都不能差。"""
    s = _node(0.0, 0.5)
    for d in (0.0, 100.0, 2400.0, 32000.0):
        assert s._weave_aim_m(d) == 120.0


def test_armed_scales_with_distance():
    s = _node(300.0, 0.5)
    assert s._weave_aim_m(10000.0) == pytest.approx(5000.0)
    assert s._weave_aim_m(4000.0) == pytest.approx(2000.0)


def test_the_floor_applies_when_distance_is_small():
    s = _node(300.0, 0.5)
    # d=1000 ⇒ 0.5*d = 500 > 300 ⇒ 用 500
    assert s._weave_aim_m(1000.0) == pytest.approx(500.0)
    # d=400 ⇒ 0.5*d = 200 < 300 ⇒ 下限 300,但上界 0.9*400=360 ⇒ 300
    assert s._weave_aim_m(400.0) == pytest.approx(300.0)


# ══════════════════════════════════════════════════════════════════════
#  ③ ⭐⭐⭐ 無界發散的守衛
# ══════════════════════════════════════════════════════════════════════
def test_the_aim_point_never_lands_behind_the_slot():
    """⚠️⚠️ 瞄準點必須永遠落在「自己 → slot」這一段**之內**。

    落到 slot 後方時,`bearing(pos→slot) + sign*θ` 會把飛機往**離開** slot
    的方向送,而且距離越遠偏得越多 —— 無界發散,且完全靜默
    (mode 欄照樣寫 weave,坡度看起來也正常)。

    ⚠️ 這正是 `max(min_m, frac*d)` 單獨使用時的洞:d < min_m/frac 時
       下限會贏,而下限是**絕對**距離、不隨 d 縮小。
    """
    s = _node(300.0, 0.5)
    for d in (10.0, 50.0, 120.0, 200.0, 299.0, 333.0, 600.0, 5000.0, 40000.0):
        aim = s._weave_aim_m(d)
        assert aim <= 0.9 * d + 1e-9, (
            "d=%.0f m 時瞄準點 %.1f m 落在 slot 後方(或太靠近)" % (d, aim))


def test_frac_is_clamped_at_use_not_at_read():
    """⚠️ 夾制必須在**使用時**,不在讀回時。

    讀回時夾制會把宣告預設 0.0 變成區間內的 0.05,而那正好讓
    「餵值路徑斷掉」在契約層完全靜默 —— [[two-launcher-paths-differ]]
    那次白跑 2.5 小時,就是因為退回值合法。
    """
    # ── (a) 讀回路徑**不可以**有夾制 ───────────────────────────────
    # ⚠️⚠️ 這一半必須看 __init__ 的原始碼,不能用手工造的物件測 ——
    #    `_node()` 是 __new__ 造殼、直接塞欄位,**完全繞過** __init__,
    #    所以拿它去測「__init__ 有沒有夾制」的斷言永遠是綠的。
    #    2026-08-25 的突變實驗就抓到這一條:把夾制搬回讀回時,測試沒反應
    #    ([[instrument-that-cannot-fire]] 的同型 —— 測不到它宣稱在測的東西)。
    found = []
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Assign):
            continue
        for t in node.targets:
            if (isinstance(t, ast.Attribute)
                    and t.attr in ("weave_aim_frac", "weave_aim_min_m")
                    and isinstance(t.value, ast.Name) and t.value.id == "self"):
                found.append((t.attr, node.value))
    assert len(found) == 2, "找不到那兩個賦值(錨點漂了):%r" % [f[0] for f in found]
    for attr, rhs in found:
        calls = [c.func.id for c in ast.walk(rhs)
                 if isinstance(c, ast.Call) and isinstance(c.func, ast.Name)]
        assert "min" not in calls and "max" not in calls, (
            "self.%s 在**讀回時**就被夾制了(%r)⇒ 宣告預設 0.0 會被夾成"
            "區間內的值 ⇒ 餵值路徑斷掉時契約層完全靜默。"
            "夾制要留在 _weave_aim_m() 使用的那一刻" % (attr, calls))

    # ── (b) 而使用時**必須**有夾制 ────────────────────────────────
    s = _node(300.0, 5.0)          # 荒謬的 frac
    assert s._weave_aim_m(10000.0) == pytest.approx(9000.0)   # 夾到 0.9
    s2 = _node(300.0, 0.0)
    assert s2._weave_aim_m(10000.0) == pytest.approx(500.0)   # 夾到 0.05


def test_the_declared_defaults_are_deliberately_illegal():
    """⭐⭐ 宣告預設必須**落在契約區間外** —— 餵值路徑一斷就要有人吼。

    這是 [[two-launcher-paths-differ]] 的直接修法:那次 31 個開關靜靜
    失效,契約閘門擋不住,因為退回值合法。
    """
    sys.path.insert(0, str(PATCHES))
    ct = importlib.import_module("contract_table")
    for name, declared in (("weave_aim_min_m", 0.0), ("weave_aim_frac", 0.0)):
        c = ct.BY_NAME[name]
        in_range = (c.lo is None or declared >= c.lo) and \
                   (c.hi is None or declared <= c.hi)
        is_foam = c.foam is not None and abs(c.foam - declared) < 1e-12
        assert (not in_range) or is_foam, (
            "%s 的宣告預設 %r **通過**契約 [%r, %r] 且不是 foam ⇒ "
            "餵值路徑斷掉時契約層不會吭聲" % (name, declared, c.lo, c.hi))


# ══════════════════════════════════════════════════════════════════════
#  ④ 生效路徑:兩條 launcher 都要接
# ══════════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("script,needle", [
    ("swarm_ros2_up.sh", "-p weave_aim_min_m:="),
    ("swarm_ros2_up.sh", "-p weave_aim_frac:="),
    ("stot_trial.sh", "-p weave_aim_min_m:="),
    ("stot_trial.sh", "-p weave_aim_frac:="),
])
def test_both_launchers_pass_the_parameter(script, needle):
    """⚠️⚠️ [[two-launcher-paths-differ]]:換 launcher 讓 31 個開關靜靜失效
       (WV1-3 白跑 2.5 小時),而契約閘門擋不住 —— 因為退回值合法。"""
    txt = (WSL / script).read_text(encoding="utf-8")
    assert needle in txt, "%s 沒有把 %s 傳下去" % (script, needle)


def test_the_interactive_path_turns_it_on():
    """互動路徑是產品路徑 —— 這一行就是開關本體。"""
    txt = (WSL / "rep_interactive.sh").read_text(encoding="utf-8")
    assert "WEAVE_AIM_MIN_M" in txt
    assert "export WEAVE_AIM_MIN_M" in txt


def test_the_armed_value_actually_clears_the_turn_radius():
    """⭐⭐⭐ 端到端的**數值**判準,不是「有沒有設」而是「設的值夠不夠」。

    互動路徑餵的 300 m 必須大於 110 m/s + 80° 的最小轉彎半徑 218 m,
    否則開關開了也還是同一個坑。
    """
    txt = (WSL / "rep_interactive.sh").read_text(encoding="utf-8")
    m = [ln for ln in txt.splitlines()
         if ln.startswith("WEAVE_AIM_MIN_M=")]
    assert m, "rep_interactive.sh 沒有指定 WEAVE_AIM_MIN_M"
    val = float(m[0].split(":-")[1].rstrip('}"'))
    r = _min_turn_radius(110.0, 80.0)
    assert val > r, (
        "餵的 %.0f m 沒有大過最小轉彎半徑 %.0f m ⇒ 開關開了也是同一個坑"
        % (val, r))


# ══════════════════════════════════════════════════════════════════════
#  ⑤ 呼叫點真的用了它
# ══════════════════════════════════════════════════════════════════════
def test_the_hardcoded_120_is_gone_from_the_weave_call_site():
    """⚠️ 值層級測不到「呼叫點有沒有改用 helper」—— 這條只好看結構。

    判準:ALIGN 蛇形的 dest(...) 不可以再出現字面量 120.0。
    """
    bad = []
    for n in ast.walk(TREE):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id == "dest" and len(n.args) >= 4):
            a = n.args[3]
            if isinstance(a, ast.Constant) and a.value == 120.0:
                bad.append(n.lineno)
    assert not bad, (
        "dest(...) 還在用字面量 120.0(line %s)—— 那個距離落在物理最小"
        "轉彎半徑 218 m 之內,飛機到不了就會被 L1 拿去繞 41 m 的圈" % bad)


def test_the_call_site_records_what_it_used():
    """⚠️ 中介變數要記錄:沒有它就分不出「開關沒生效」與「生效了但沒用」,
       而那兩者的下一步完全相反。"""
    assert "self._wv_aim[i]" in SRC
    assert '"weave_aim_m"' in SRC
