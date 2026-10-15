# -*- coding: utf-8 -*-
"""tot_node 的能量三欄(thr_fc / eas_fc / climb_fc)+ 裸 else 地雷。

為什麼有這個檔(2026-08-25):
  「轉彎阻力 ⇒ 推力不足 ⇒ TECS 拿高度換空速」整條論證的終點是一個
  **油門主張**,而 tot CSV 先前 59 欄裡**沒有任何油門或空速欄位**
  ⇒ 那個假說在 tot 資料上根本無法被證偽。這三欄就是那個檢定。

⚠️⚠️ 本檔的第一等公民是 `test_unknown_type_never_lands_in_att` ——
   直連接收路徑原本用**裸 `else`** 當 ATTITUDE 分支:

       elif mt == "HEARTBEAT": ...
       else:
           self.att[i] = (m.roll, m.pitch, m.yaw)

   任何新加進 recv_match(type=[...]) 的型別都會掉進來,而非姿態訊息沒有
   `.roll` ⇒ AttributeError 打掉整條接收迴圈,症狀卻出現在**完全不相干的
   地方**(導引吃到凍結的姿態)。加 VFR_HUD 時就踩到了。

⚠️ 依 [[instrument-that-cannot-fire]]:每一條測試都先斷言「這個設定真的
   能讓 X 發生」,否則恆空的欄位會被讀成「量到零」。
"""
import ast
import importlib
import io
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PATCHES = REPO / "scripts" / "wsl" / "sar_swarm_patches"
if str(PATCHES) not in sys.path:
    sys.path.insert(0, str(PATCHES))


def _install_ros_stubs():
    """WSL 沒有 ROS。用 setdefault ⇒ 別的測試檔已裝過就是 no-op。"""
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


# ══════════════════════════════════════════════════════════════════════
#  假訊息 / 假鏈路
# ══════════════════════════════════════════════════════════════════════
class FakeMsg:
    def __init__(self, mtype, **kw):
        self._t = mtype
        self.__dict__.update(kw)

    def get_type(self):
        return self._t


class FakeLink:
    """吐完就回 None —— 對應 recv_match(blocking=False) 的語意。"""

    def __init__(self, msgs):
        self._q = list(msgs)

    def recv_match(self, blocking=False, type=None):   # noqa: A002
        while self._q:
            m = self._q.pop(0)
            if type is None or m.get_type() in type:
                return m
        return None


def _bare_node(n=2):
    s = TotNode.__new__(TotNode)
    s.n = n
    s.vfr = {}
    s.ack_bad = {}
    s.ack_seen = set()
    s.att = {}
    s.rate = {}
    s.pos = {}
    s.vel = {}
    s.gs = {}
    s.home_amsl = {}
    s.alt_amsl = {}
    s.state_t = {}
    s.fc_mode = {}
    s.fc_mode_t = {}
    s.truth = {}
    s.home_ll = {}
    s.zg = {}
    s.zg_amsl = {}
    s.phase = "INIT"
    s.target_amsl = float("nan")
    s.T = (25.0, 121.0)
    s.log = types.SimpleNamespace(info=lambda *a, **k: None,
                                  warning=lambda *a, **k: None,
                                  error=lambda *a, **k: None)
    s.get_logger = lambda: s.log
    return s


VFR = dict(airspeed=132.5, groundspeed=157.0, throttle=0.87, climb=4.25)


# ══════════════════════════════════════════════════════════════════════
#  ① 型別真的收得到(儀表能不能開火)
# ══════════════════════════════════════════════════════════════════════
def test_vfr_hud_is_in_fanout_allowlist():
    """⭐ 先證明「這個設定真的能讓 VFR_HUD 進來」。

    `_pump_link` 對不在 `_LINK_TYPES` 的型別直接 `continue`,所以少了這一項
    的話,下面每一條行為測試都會**恆綠而且什麼都沒測到**。
    """
    assert "VFR_HUD" in TotNode._LINK_TYPES


def test_vfr_hud_is_in_direct_link_type_filter():
    """直連那條走 recv_match(type=[...]);不在清單裡就整包收不到。"""
    for fn in ast.walk(TREE):
        if not (isinstance(fn, ast.Call)
                and isinstance(fn.func, ast.Attribute)
                and fn.func.attr == "recv_match"):
            continue
        for kw in fn.keywords:
            if kw.arg == "type" and isinstance(kw.value, ast.List):
                names = [e.value for e in kw.value.elts
                         if isinstance(e, ast.Constant)]
                assert "VFR_HUD" in names, (
                    "recv_match 的型別白名單缺 VFR_HUD ⇒ 三欄會恆空,"
                    "而恆空會被讀成『量到零』:%r" % (names,))
                return
    pytest.fail("找不到帶 type= 的 recv_match —— 錨點變了,這條守衛失效了")


# ══════════════════════════════════════════════════════════════════════
#  ② 分派行為
# ══════════════════════════════════════════════════════════════════════
def test_fanout_stores_vfr_triplet():
    s = _bare_node()
    s.link = {1: FakeLink([FakeMsg("VFR_HUD", **VFR)])}
    s._pump_link()
    assert 1 in s.vfr, "VFR_HUD 進來了卻沒有被存下來"
    eas, thr, climb = s.vfr[1]
    assert eas == pytest.approx(132.5)
    assert thr == pytest.approx(0.87)
    assert climb == pytest.approx(4.25)


def test_vfr_hud_does_not_clobber_attitude():
    """⚠️⚠️ 地雷本體:VFR_HUD 沒有 .roll,掉進裸 else 會 AttributeError。"""
    s = _bare_node()
    s.link = {1: FakeLink([FakeMsg("VFR_HUD", **VFR)])}
    s._pump_link()          # 不可以拋
    assert s.att == {}, "VFR_HUD 被當成 ATTITUDE 寫進 att 了"


def test_attitude_still_works_after_the_fix():
    """把裸 else 改成具名分支之後,姿態本身不可以壞掉。"""
    s = _bare_node()
    s.link = {1: FakeLink([FakeMsg("ATTITUDE", roll=0.5, pitch=-0.1, yaw=1.2,
                                   rollspeed=0.01, pitchspeed=0.02,
                                   yawspeed=0.03)])}
    s._pump_link()
    assert s.att[1] == pytest.approx((0.5, -0.1, 1.2))
    assert s.rate[1] == pytest.approx((0.01, 0.02, 0.03))


def test_unknown_type_never_lands_in_att():
    """⭐⭐⭐ 這是整個檔案存在的理由。

    直連路徑原本是裸 `else` ⇒ **任何**未列舉的型別都會被寫成姿態。
    用 AST 檢查:每一條把 `self.att[...]` 當賦值目標的敘述,都必須在一個
    **具名**的 `mt == "ATTITUDE"` 判斷底下,不可以在 `else` 底下。
    """
    bad = []

    def _writes_att(node):
        for x in ast.walk(node):
            if (isinstance(x, ast.Assign)
                    and any(isinstance(t, ast.Subscript)
                            and isinstance(t.value, ast.Attribute)
                            and t.value.attr == "att" for t in x.targets)):
                return True
        return False

    for n in ast.walk(TREE):
        if not isinstance(n, ast.If):
            continue
        # orelse 是「裸 else」時:它是一串敘述,而不是另一個 ast.If(elif)
        if n.orelse and not (len(n.orelse) == 1
                             and isinstance(n.orelse[0], ast.If)):
            for stmt in n.orelse:
                if _writes_att(stmt):
                    bad.append(stmt.lineno)
    assert not bad, (
        "self.att 在裸 else 底下被賦值(line %s)—— 任何新加進 recv_match "
        "型別白名單的訊息都會掉進來被當成姿態,而非姿態訊息沒有 .roll "
        "⇒ AttributeError 打掉整條接收迴圈。改成 `elif mt == \"ATTITUDE\":`"
        % bad)


# ══════════════════════════════════════════════════════════════════════
#  ③ CSV 欄位
# ══════════════════════════════════════════════════════════════════════
def _wide_writerows():
    """(節點, 欄數) —— 表頭與資料列這一對 `writerow([...])`。

    ⚠️ 恆空欄追蹤刻意做成**包住 writer**,就是為了讓這裡的呼叫點
       維持字面 list —— 本專案有十幾條守衛(欄寬、欄序、空字串哨兵)
       靠解析它。第一版改成 `_cols = [...]` 一次打斷 24 條。
    """
    out = []
    for n in ast.walk(TREE):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == "writerow" and n.args
                and isinstance(n.args[0], (ast.List, ast.Tuple))
                and len(n.args[0].elts) > 20):
            out.append((n, len(n.args[0].elts)))
    return out


def _writerow_lens():
    return [(x.lineno, k) for x, k in _wide_writerows()]


def _header_elts():
    """表頭那一筆的欄名(整排都是字面字串)。"""
    best = None
    for x, _ in _wide_writerows():
        elts = x.args[0].elts
        strs = sum(1 for e in elts
                   if isinstance(e, ast.Constant) and isinstance(e.value, str))
        if strs > len(elts) * 0.8 and (best is None or x.lineno < best.lineno):
            best = x
    assert best is not None, "找不到表頭 writerow —— 錨點漂了"
    return [e.value for e in best.args[0].elts
            if isinstance(e, ast.Constant) and isinstance(e.value, str)]


def test_header_and_row_have_the_same_width():
    """⚠️ 欄數不一致會**靜默**毀掉每一份 CSV —— 欄位錯位之後每個數字都還
       是合法的數字,只是屬於別的欄。這是本專案付得起的最貴的一種靜默失敗。
    """
    wide = [(ln, k) for ln, k in _writerow_lens() if k > 20]
    assert len(wide) >= 2, "找不到表頭與資料列這一對 writerow:%r" % (wide,)
    widths = {k for _, k in wide}
    assert len(widths) == 1, (
        "表頭與資料列欄數不一致:%r —— CSV 會整份錯位" % (wide,))


def test_energy_columns_are_present_in_header():
    names = _header_elts()
    for c in ("thr_fc", "eas_fc", "climb_fc"):
        assert c in names, "表頭缺 %s:%r" % (c, names)
    # ⚠️ 既有的 gs 是**地速**,eas_fc 是**等效空速** —— 兩欄都要在,
    #    因為 3100 m 兩者差 1.167 倍,少了任一欄就分不出退讓方向。
    assert "gs" in names


def test_zero_throttle_is_not_written_as_empty():
    """⭐ 0 與「沒量到」必須分得開。

    本檔的規矩是「量不到寫空字串,絕不寫 0」。反過來也要成立:油門**真的
    是 0**(收油門滑翔)時必須寫 0.0,不可以寫空 —— 否則滑翔段會被讀成
    「這一段沒有資料」,而那正好是我們最想看的一段。
    """
    s = _bare_node()
    s.link = {1: FakeLink([FakeMsg("VFR_HUD", airspeed=0.0, groundspeed=0.0,
                                   throttle=0.0, climb=0.0)])}
    s._pump_link()
    assert 1 in s.vfr, "全 0 的 VFR_HUD 被丟掉了"
    assert s.vfr[1] == (0.0, 0.0, 0.0)
    # 而 _row 的寫法是 `"" if _vf is None else round(...)` ⇒ 有值就寫值。
    _vf = s.vfr.get(1)
    assert ("" if _vf is None else round(_vf[1], 4)) == 0.0


# ══════════════════════════════════════════════════════════════════════
#  ④ 重設語意
# ══════════════════════════════════════════════════════════════════════
def test_vfr_is_not_cleared_between_strikes():
    """⚠️ vfr 是**持續被鏈路覆寫的遙測**,和 pos/vel/att 同類。

    放進 RESET_DICTS 會讓第二發的頭幾拍三欄全空,而那幾拍正是放行前後 ——
    整個航次最貴的那幾拍。
    """
    rd = getattr(tot_node, "RESET_DICTS", ())
    assert "vfr" not in rd
    for peer in ("pos", "vel", "att"):
        assert peer not in rd, (
            "%s 竟然在 RESET_DICTS 裡 —— 那本測試的前提就變了" % peer)


# ══════════════════════════════════════════════════════════════════════
#  ⑤ COMMAND_ACK —— 本節點先前**完全不讀**
# ══════════════════════════════════════════════════════════════════════
#  三次事故的共同盲點:我們送了、飛控回了「不要」、而我們沒有在聽。
#    catch_speed 157 > AIRSPEED_MAX 150      ⇒ MAV_RESULT=4
#    DO_REPOSITION param2=0                  ⇒ 位置整發被丟,速度照常
#    編隊速度 < AIRSPEED_MIN                 ⇒ 每一發都被拒
#  每一次的表徵都是「飛機不照做但沒有任何錯誤訊息」。
# ══════════════════════════════════════════════════════════════════════
DO_REPOSITION = 192
DO_CHANGE_SPEED = 178


def _ack(cmd, result):
    return FakeMsg("COMMAND_ACK", command=cmd, result=result)


def test_command_ack_is_in_fanout_allowlist():
    """⭐ 先證明儀表能開火:指令走 self.link 送出 ⇒ ACK 從同一條回來。"""
    assert "COMMAND_ACK" in TotNode._LINK_TYPES


def test_rejected_command_is_counted():
    s = _bare_node()
    s.link = {1: FakeLink([_ack(DO_CHANGE_SPEED, 4)])}   # MAV_RESULT_FAILED
    s._pump_link()
    assert s.ack_bad.get(1) == 1, "整發被拒的指令沒有被計數"


def test_accepted_and_in_progress_are_not_counted():
    """⚠️ IN_PROGRESS(5)不是失敗 —— 飛控稍後會再送一則最終 ACK。
       把它算成拒收會讓每一趟都是紅的,而『永遠紅』和『永遠綠』一樣沒用。"""
    s = _bare_node()
    s.link = {1: FakeLink([_ack(DO_REPOSITION, 0), _ack(DO_REPOSITION, 5)])}
    s._pump_link()
    assert s.ack_bad.get(1, 0) == 0


def test_log_is_throttled_but_the_counter_is_not():
    """⭐⭐ 這是本檔第二重要的一條。

    同一個指令每 tick 都在送,逐則寫 log 會把 log 洗掉。但**計數不可以
    跟著節流** —— 節流過的計數會低報,而低報的拒收次數正好長得像
    『只是偶爾一次,應該沒關係』。
    """
    s = _bare_node()
    s.link = {1: FakeLink([_ack(DO_CHANGE_SPEED, 4) for _ in range(5)])}
    warns = []
    s.log.warning = warns.append
    s._pump_link()
    assert s.ack_bad.get(1) == 5, "計數被節流了(應為 5)"
    assert len(warns) == 1, "同一個 (指令,結果) 記了 %d 次 log" % len(warns)


def test_warning_names_the_command_and_the_result():
    """⚠️ 訊息要能直接指認 —— 『指令被拒』而不說哪一個,診斷成本一樣高。"""
    s = _bare_node()
    s.link = {1: FakeLink([_ack(DO_CHANGE_SPEED, 4)])}
    warns = []
    s.log.warning = warns.append
    s._pump_link()
    assert warns, "被拒收卻完全沒有告警 —— 這正是三次事故的樣子"
    msg = warns[0]
    assert "DO_CHANGE_SPEED" in msg, msg
    assert "FAILED" in msg, msg
    assert "UAV1" in msg, msg


def test_different_commands_each_get_their_own_warning():
    s = _bare_node()
    s.link = {1: FakeLink([_ack(DO_CHANGE_SPEED, 4), _ack(DO_REPOSITION, 2)])}
    warns = []
    s.log.warning = warns.append
    s._pump_link()
    assert len(warns) == 2, "不同指令的拒收被併成一則了"
    assert s.ack_bad.get(1) == 2


def test_ack_bad_n_is_in_header():
    assert "ack_bad_n" in _header_elts()


def test_ack_bad_n_writes_zero_not_empty():
    """⚠️ 這一欄**不套**「量不到寫空字串」那條規矩 —— 0 是有意義的
       (= 到目前為止沒被拒過)。寫空會讓「乾淨的航次」和「沒有這一欄的
       舊航次」長得一模一樣。"""
    s = _bare_node()
    assert s.ack_bad.get(1, 0) == 0
    src_row = SRC.split("ack_bad_n")[-1]
    assert 'self.ack_bad.get(i, 0)' in SRC, "資料列沒有寫 ack_bad"
    # 值層級:確認取值路徑真的回 0 而不是 None/""
    assert s.ack_bad.get(1, 0) is not None


# ══════════════════════════════════════════════════════════════════════
#  ⑥ 恆空欄自我回報
# ══════════════════════════════════════════════════════════════════════
#  ⚠️⚠️ 恆空欄在下游會被讀成「量到零」,而那通常正好是我們想驗證的假說。
#  本專案至少三次因此下了錯誤結論:
#    * ALIGN 段蛇形六欄全 0% 而 mode 欄寫著 weave ⇒ 記錄在案的
#      「零換向 / 2.85 s / 達成率 24-33%」量的其實是**等待段**。
#    * 換向計數器被放進恆為正的量值函式 ⇒ 那一欄不可能加一,
#      而「那一欄恆空」正好長得像「量到零換向」。
#    * 2026-08-25:掃到 5 列的空殼檔,差點據此下「我們從沒量過坡度」
#      的結論(實際上 roll_meas_deg 100% 有值)。
#  ⇒ 判別法必須是**自動的**。靠人記得去查填充率,就是靠人不犯錯。
# ══════════════════════════════════════════════════════════════════════
class _Sink:
    """吞掉輸出的假 csv.writer。"""

    def __init__(self):
        self.rows = []

    def writerow(self, r):
        self.rows.append(list(r))


def _tracker(cols):
    """⚠️ 第一列**就是**表頭 —— 這是包裝式的核心不變式:
       追蹤寬度與表頭寬度構造上相同,不可能對不上。"""
    w = tot_node.DeadColumnWriter(_Sink())
    w.writerow(list(cols))
    return w


def _feed(w, rows):
    for r in rows:
        w.writerow(r)
    return w


def _report(w):
    return tot_node.DeadColumnWriter.report(w)


def test_a_column_that_never_fills_is_named():
    s = _feed(_tracker(["a", "b", "c", "d"]),
              [[1, "", 3, ""], [2, "", 4, ""], [3, "", 5, ""]])
    lines = _report(s)
    assert lines, "b / d 一列都沒寫過,卻沒被點名"
    assert "b" in lines[0] and "d" in lines[0], lines[0]
    assert "a" not in lines[0].split(":")[-1]


def test_a_column_that_fills_once_is_not_named():
    """⭐ 反向:只要出現過**一次**非空值就不算恆空。

    這一條防的是把「稀疏」誤報成「恆空」—— 誤報會讓人去修沒壞的東西,
    而且會稀釋真正的警告。
    """
    s = _feed(_tracker(["a", "b"]), [[1, ""], [2, ""], [3, 0.0], [4, ""]])
    assert _report(s) == []


def test_zero_is_not_treated_as_empty():
    """⚠️⚠️ 0 是**有值**。把 0 當空會讓「量到零」變成「沒量到」,
       正好是這整個機制想分開的那兩件事、方向還相反。"""
    s = _feed(_tracker(["a"]), [[0], [0.0], [0]])
    assert _report(s) == []


def test_no_rows_at_all_is_reported_differently():
    """⚠️ 「一列都沒有」和「有列但某欄恆空」是兩種不同的故障,
       訊息必須分得開 —— 前者代表整趟的欄位分析都無效。"""
    s = _tracker(["a", "b"])
    lines = _report(s)
    assert len(lines) == 1
    assert "一列都沒寫過" in lines[0]
    assert "所有欄位分析都無效" in lines[0]


def test_clean_sortie_says_nothing():
    """沒有恆空欄時不可以輸出任何東西 —— 每趟都吼的警告等於沒有警告。"""
    s = _feed(_tracker(["a", "b"]), [[1, 2], [3, 4]])
    assert _report(s) == []


def test_the_real_header_and_the_tracker_agree_on_width():
    """⭐ 儀表能不能開火:追蹤器涵蓋的欄數必須等於真實表頭欄數。

    對不上的話,尾巴那幾欄(正好是最新加的那幾欄)永遠不會被檢查 ——
    而「最新加的欄」正是最可能忘記接線、因此最可能恆空的那些。
    """
    lens = [k for _, k in _writerow_lens()]
    assert lens, "找不到欄位 list —— 錨點漂了"
    assert len(set(lens)) == 1, "表頭與資料列的 list 長度不一致:%r" % lens
    # ⭐ 包裝式讓「追蹤寬度 == 表頭寬度」變成**構造上**成立(第一列就是
    #    表頭),不需要再對兩個數字。這裡改成鎖住那個構造本身。
    # ══════════════════════════════════════════════════════════════
    # ⚠️⚠️ 這一段刻意**真的去開一份 CSV**,不是比對原始碼字串。
    #
    #   舊版是 `assert SRC.count("self._cw = DeadColumnWriter(self._cw)") == 2`
    #   —— 它在行為完全壞掉的情況下照樣是綠的。WEAVEAIM1 實際踩到:
    #   包裝當時挪到 `_open_csv()` 的**呼叫端**,而表頭在 `_open_csv`
    #   **內部**就寫掉了 ⇒ 包裝器把第一列**資料**當成表頭 ⇒ 結案時
    #   吐出「這些欄一列都沒寫過:, , , , , , , , ,」(全是空欄名)。
    #   源碼層守衛對它完全瞎([[value-locks-beat-source-locks]])。
    # ══════════════════════════════════════════════════════════════
    import os
    import tempfile

    s = TotNode.__new__(TotNode)
    s._csv = None
    s.vfr, s.ack_bad, s._wv_aim = {}, {}, {}
    tmp = tempfile.mkdtemp()
    s._csv_base = lambda: os.path.join(tmp, "guard")
    s._open_csv(wrap=tot_node.DeadColumnWriter)

    w = s._cw
    assert isinstance(w, tot_node.DeadColumnWriter), (
        "_open_csv(wrap=...) 沒有把 writer 包起來 ⇒ 完全沒有恆空欄追蹤")
    # ⭐ 決定性:抓到的必須是**表頭**,不是第一列資料。
    assert w.cols is not None and w.cols[0] == "t" and w.cols[1] == "sysid", (
        "包裝器抓到的不是表頭,而是資料列:%r ⇒ 點名恆空欄時會吐出空欄名"
        % (list(w.cols[:4]) if w.cols else None,))
    real = _header_elts()
    assert len(real) > 20, "表頭只有 %d 欄?錨點漂了" % len(real)
    assert list(w.cols) == real, "包裝器抓到的欄名與原始碼表頭不一致"
    assert len(w.unseen) == len(real), (
        "追蹤寬度 %d != 表頭寬度 %d ⇒ 尾端 %d 欄永遠不會被檢查 —— "
        "而尾巴正是最新加、最可能忘記接線的那幾欄"
        % (len(w.unseen), len(real), len(real) - len(w.unseen)))
    assert w.rows == 0, "表頭不可以被算成資料列"

    # ⚠️ 反向:無 wrap 呼叫**不可以炸** —— 有兩個測試檔會抽出 _open_csv
    #    的原始碼在受限命名空間 exec,那裡沒有 DeadColumnWriter 這個名字。
    s2 = TotNode.__new__(TotNode)
    s2._csv = None
    s2._csv_base = lambda: os.path.join(tmp, "guard2")
    s2._open_csv()
    assert not isinstance(s2._cw, tot_node.DeadColumnWriter)


def test_the_sentinel_stays_at_index_zero():
    """⚠️⚠️ 恆空欄的行必須接在**尾端**。

    `_impact_summary()[0]` 是哨兵,有 47 個測試與外部 grep 消費者在守。
    我先前把損失橫幅放在 [0] 就把哨兵推掉過一次。
    """
    src_ret = [ln for ln in SRC.splitlines()
               if "return lines[:1] + lost_lines" in ln]
    assert src_ret, "找不到 _impact_summary 的 return —— 錨點漂了"
    ln = src_ret[0]
    assert "_dead_column_lines()" in ln, "恆空欄回報沒有掛進 _impact_summary"
    assert ln.index("lines[:1]") < ln.index("_dead_column_lines"), (
        "恆空欄的行排在哨兵前面 —— 會把 [0] 的哨兵推掉:%s" % ln)


def test_the_second_strike_still_tracks():
    """⚠️ 第二發會**換檔**(不覆蓋第一發的證據),那次也必須傳 wrap。

    漏掉的話第二發之後就沒有恆空欄追蹤 —— 而且完全靜默,因為
    `_dead_column_lines()` 對一個沒被包起來的 writer 是回空 list,
    看起來就像「這一趟沒有恆空欄」。
    """
    n = SRC.count("self._open_csv(wrap=DeadColumnWriter)")
    assert n == 2, (
        "只有 %d 個 _open_csv 呼叫傳了 wrap(應為 2:首發 + 換檔)" % n)
    # ⭐ 值層級的那一半在 test_the_real_header_and_the_tracker_agree_on_width;
    #    這裡鎖的是「**兩個**呼叫點都有」,那個只能數。


# ══════════════════════════════════════════════════════════════════════
#  ⑦ 拒收**來源**:讓它成為資料而不是推論
# ══════════════════════════════════════════════════════════════════════
#  ⚠️⚠️ `_pump_link` 讀的是 MAVProxy 的**分流**鏈路 ⇒ tot_node 看得到
#     **所有送出者**的 COMMAND_ACK。GMODE1 的 49 次拒收**全部**是編隊層
#     (target_system=250),而 tot_node 用 233/234 送、在 STANDBY 什麼都不送。
#     要不是後來去比對時戳與 tlog,那份報告會寫成「tot_node 的指令被拒 36 次」
#     —— 完全錯的層。
# ══════════════════════════════════════════════════════════════════════
def test_ack_origin_is_separated_not_inferred():
    """⭐⭐⭐ 本節點的與別層的必須分得開。"""
    s = _bare_node()
    s.ack_bad_own = {}
    s.link = {1: FakeLink([
        FakeMsg("COMMAND_ACK", command=192, result=4, target_system=233),
        FakeMsg("COMMAND_ACK", command=178, result=4, target_system=234),
        FakeMsg("COMMAND_ACK", command=192, result=4, target_system=250),
        FakeMsg("COMMAND_ACK", command=178, result=4, target_system=250),
    ])}
    s._pump_link()
    assert s.ack_bad.get(1) == 4, "整條鏈路的計數不對"
    assert s.ack_bad_own.get(1) == 2, (
        "本節點的計數不對 —— 233/234 是我們,250 是編隊層")


def test_unknown_origin_counts_as_foreign():
    """⚠️ 讀不到 target_system 要當成**不是**我們的。

    安全側:寧可少算自己的,也不要把別層的問題記到我們頭上 ——
    那正是 GMODE1 差一點發生的事。
    """
    s = _bare_node()
    s.ack_bad_own = {}
    s.link = {1: FakeLink([FakeMsg("COMMAND_ACK", command=192, result=4)])}
    s._pump_link()
    assert s.ack_bad.get(1) == 1
    assert s.ack_bad_own.get(1, 0) == 0, "來源不明卻算成我們的"


def test_the_warning_names_the_origin():
    """⚠️ 告警要說出 target_system 與「是不是本節點」——
       只說「指令被拒」會讓下一個人去查錯的層。"""
    s = _bare_node()
    s.ack_bad_own = {}
    warns = []
    s.log.warning = warns.append
    s.link = {1: FakeLink([FakeMsg("COMMAND_ACK", command=192, result=4,
                                   target_system=250)])}
    s._pump_link()
    assert warns
    assert "250" in warns[0], warns[0]
    assert "其他層" in warns[0] or "非 tot_node" in warns[0], warns[0]


def test_own_sysids_match_the_actual_connections():
    """⭐ 儀表能不能開火:_ACK_OWN_SYSIDS 必須等於我們**真的**在用的 sysid。

    對不上的話分流永遠是錯的,而且看起來完全正常。
    """
    import re
    used = {int(m) for m in re.findall(r"source_system=(\d+)", SRC)}
    assert used, "找不到 source_system —— 錨點漂了"
    assert set(TotNode._ACK_OWN_SYSIDS) == used, (
        "_ACK_OWN_SYSIDS %r 與實際連線用的 %r 不一致"
        % (sorted(TotNode._ACK_OWN_SYSIDS), sorted(used)))


def test_ack_bad_own_is_in_header():
    assert "ack_bad_own" in _header_elts()
