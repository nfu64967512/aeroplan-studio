# -*- coding: utf-8 -*-
"""AUTO 預規劃航線 → 攻擊發起點盤旋 → 全員到齊 → GUIDED(2026-08-25)。

═══════════════════════════════════════════════════════════════════════════
 這個架構要買的東西
═══════════════════════════════════════════════════════════════════════════
放行距離 R0 目前是**湧現量**:守環從來沒守過(指令 95 m/s / 1300 m 圈,
實飛 124–186 m/s / 5867 m ⇒ 直穿 3.2 km),於是放行 R 由飛機「衝過環多遠」
決定。98 架次的分箱是單調的:

    R0 0–4000   殘餘高度中位 167.4 m、71.4% 超標
    R0 7000+    殘餘高度中位  17.2 m、 0.0% 超標

⇒ 把攻擊發起點做成任務航線上的一個盤旋點,R0 就變成設計量。

═══════════════════════════════════════════════════════════════════════════
 這個檔的紅線(每一條都對應一個**已經發生過**的失效形狀)
═══════════════════════════════════════════════════════════════════════════
① 開關關著時逐位元同舊行為 —— 新架構預設 False,ALIGN 照舊逐點下令。
② 全員到齊的閘門走 `_gate_all`,不是 `len(arr) >= n`
   ⇒ ALIGNW1 等 1551 s 的死結不可以換一個地方重演。
③ 抵達判定半徑 > 盤旋半徑 —— 否則 `d < reach` **數學上永遠不成立**。
   這是 ② 的**另一種**死結:閘門的前提條件無法滿足。
④ 上傳失敗要**大聲中止**,不可以靜默退回舊架構
   ⇒ [[two-launcher-paths-differ]] 白跑 2.5 小時的形狀。
⑤ `_pump_link` 的白名單與 if/elif 鏈是一對 —— 裸 else 會把新型別當成
   地形回報去讀 `m.spacing` ⇒ **整條接收迴圈掛掉**,而症狀出現在
   完全不相干的地方([[bare-else-in-dispatch]])。
⑥ 交回 GUIDED 時要預設好 `runup_t0`/`perch_T`,否則 PERCH 的第一拍會送出
   一發「回頭飛 35 km」的 DO_REPOSITION(r_fan=35000)。

⚠️ 本檔一條 `sleep` 都沒有,時間全部是注入的 —— `mission_plan` 刻意不
   import time,而 conftest.py 有一整段在講「測試凍結全域時鐘沒還原,
   八十分鐘 CPU 被吃掉」的事故。
"""
import ast
import importlib
import math
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCHES = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
if str(PATCHES) not in sys.path:
    sys.path.insert(0, str(PATCHES))

# ⚠️ 從 test_tot_standby 借 ROS 樁與節點殼 —— 它是本 repo 的實質共用 harness
#    (test_kill_deadlock.py 也是這樣借的)。pytest.ini 用 importlib 模式,
#    所以要先把 tests 目錄本身放上 sys.path。
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_tot_standby import TotNode, FakeLogger  # noqa: E402

import mission_plan as mp  # noqa: E402

tot_node = importlib.import_module("tot_node")
SRC = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
MI_SRC = (PATCHES / "mavros_interface.py").read_text(encoding="utf-8")

TGT = (25.0777, 119.94315)


def _code_only(block):
    """把整行註解與行尾註解剝掉,再做「這個字串**不可以**出現」的斷言。

    ⚠️⚠️ 這是本 repo 記錄在案的第**五**次同型誤判(前四次在
       test_ring_pace / test_release_at_ring / test_wait_goto_radius 的
       docstring 裡各有記載)。這一次踩到的是:ALIGN 的 auto 分支上面有一行
       註解寫著「用 `_gate_all` 而不是 `len(self.arr) >= self.n`」——
       那行註解**本身**就含有被禁止的字串,於是守衛對一段完全正確的程式碼
       開火。而一道會對正確輸入開火的守衛,下一個人只會把它刪掉。
    ⚠️ 註解本身有價值(它解釋了為什麼不能那樣寫),要剝的是比對用的副本,
       不是原始碼。
    """
    out = []
    for ln in block.splitlines():
        s = ln.split("#", 1)[0]
        if s.strip():
            out.append(s)
    return "\n".join(out)


# ══════════════════════════════════════════════════════════════════════
#  樁
# ══════════════════════════════════════════════════════════════════════

class _Mav:
    """記下送出去的每一則 MAVLink。⚠️ 不叫 Test* —— pytest 會去收集它。"""

    def __init__(self, sink):
        self._sink = sink

    def command_long_send(self, *a):
        self._sink.append(("cmd_long", a))

    def mission_count_send(self, *a):
        self._sink.append(("count", a))

    def mission_item_int_send(self, *a):
        self._sink.append(("item", a))

    def command_int_send(self, *a):
        self._sink.append(("cmd_int", a))


class _Link:
    def __init__(self, sysid, sink):
        self.target_system = sysid
        self.mav = _Mav(sink)


def mk(n=2, auto=True, ip_reach=5000.0, loiter_r=3300.0, ip_r=0.0,
       transit_r=0.0, fallback=False, now=1.79e9, alt=3100.0):
    """只塞受測路徑會碰到的欄位。⚠️ `__new__` 不是 `__init__`(後者要 ROS)。

    ⚠️ 刻意**不**多給欄位:本 repo 的既有教訓是樁補太滿會讓測試走一條
       生產路徑不會走的分支(test_kill_deadlock.py 的 mk 也這樣註明)。
    """
    o = TotNode.__new__(TotNode)
    o.n = n
    o.phase = "ALIGN"
    o.T = TGT
    o.t0 = now - 10.0
    o.sent = []
    o.link = {i: _Link(i, o.sent) for i in range(1, n + 1)}
    # 各機從紅心西方 30 km 進場
    o.pos = {i: (TGT[0], TGT[1] - 0.30, alt) for i in range(1, n + 1)}
    o.gs = {i: 150.0 for i in range(1, n + 1)}
    o.fc_mode = {}
    o.fc_mode_t = {}
    o.killed = {}
    o.lost_at = {}
    o.impact = {}
    o.missed = {}
    o.arr = {}
    o.ready = {}
    o.align_ok = {}
    o.perch_arr = {}
    o._mission = {}
    o._ip_of = {}
    o._auto_mode_t = {}
    o._auto_failed = ""
    o._auto_started = False
    o._auto_gate_logged = False
    o.auto_mission = auto
    o.auto_ip_r = ip_r
    o.auto_ip_alt = 0.0
    o.auto_ip_reach = ip_reach
    o.auto_loiter_r = loiter_r
    o.auto_transit_r = transit_r
    o.auto_upload_timeout = 30.0
    o.auto_mode_timeout = 20.0
    o.auto_fallback_guided = fallback
    # ⚠️ 樁預設打開:AUTO 架構要求它(契約有不變式),
    #    關著時 `_auto_handoff` 會吼,而那會污染別條鎖的 log 斷言。
    o.release_at_ring = True
    o._auto_r0 = {}
    # 幾何(生產值)
    o.approach = 90.0
    o.half = 25.0
    o.perch_alt = 3100.0
    o.perch_step = 0.0
    o.pushover_r = 6084.0
    o.dive_alt = 0.0
    o.dive_angle = 27.0
    o.perch_reach = 260.0
    o.r_fan = 35000.0
    o.vmax = 150.0
    o.plan_frac = 0.8
    o.weave_aim_min_m = 0.0
    o.slot_id = {i: i - 1 for i in range(1, n + 1)}
    o.slot_of = {i: (TGT[0], TGT[1] - 0.30) for i in range(1, n + 1)}
    # ⚠️ `ring_point_of` 在 perch_step=0 時讀 `self.pp` —— 用**節點自己的**
    #    `_ring_points()` 算式產生,不要在測試裡另抄一份(抄一份就變成
    #    「拿 A 證明 A」,而且兩份會分歧)。
    o.overshoot = 2000.0
    o._ring_points()
    o.standby_mode = False
    o._log = FakeLogger()
    o.get_logger = lambda: o._log
    return o


def _ip_pts(o, now=1.79e9):
    """跑完建構 + 上傳,回傳 {i: ip}。"""
    o._auto_start(now)
    return dict(o._ip_of)


def _finish_upload(o, now=1.79e9):
    """把每一架的協商跑完(飛控依序要每一項 + ACCEPTED)。"""
    for i, up in o._mission.items():
        for seq in range(len(up.items)):
            o._mission_event(i, "request", seq)
        o._mission_event(i, "ack", 0)


# ══════════════════════════════════════════════════════════════════════
#  紅線 ①:開關關著 = 逐位元舊行為
# ══════════════════════════════════════════════════════════════════════

class TestTheSwitchDefaultsOff:
    def test_the_declared_default_is_false(self):
        """⚠️ 從**節點原始碼**讀宣告值,不是從樁 —— 對著自己的樁斷言
        「預設是 False」是在測試自己,零價值。"""
        tree = ast.parse(SRC)
        found = {}
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name) and node.func.id == "g"
                    and node.args
                    and isinstance(node.args[0], ast.Constant)
                    and str(node.args[0].value).startswith("auto_")):
                if len(node.args) > 1 and isinstance(node.args[1], ast.Constant):
                    found[node.args[0].value] = node.args[1].value
        assert found.get("auto_mission") is False, (
            f"auto_mission 的宣告預設不是 False:{found.get('auto_mission')}")
        assert found.get("auto_fallback_guided") is False
        # ⚠️⚠️ 這兩顆的宣告預設**刻意不合法**(契約區間 [200,20000] /
        #    [1500,12000])—— 餵值路徑一斷就必須讓契約層吼。
        assert found.get("auto_ip_reach") == 0.0
        assert found.get("auto_loiter_r") == 0.0

    def test_the_class_attribute_default_is_off(self):
        assert TotNode.auto_mission is False

    def test_align_still_drives_guided_when_the_switch_is_off(self):
        """⭐ 關著時 ALIGN 分派必須走舊分支 —— 那個分支的入口是 `_goto`。

        ⚠️ 這條是**行為級**的:直接看 `_auto_tick` 有沒有被呼叫到。
           短路求值讓 `auto_mission=False` 時它連評估都不該評估。
        """
        o = mk(auto=False)
        called = []
        o._auto_tick = lambda now: called.append(now) or True
        # 只驅動 ALIGN 那一段的判別式,不跑整個 _tick(那要一大堆別的樁)
        assert not (o.phase == "ALIGN" and o.auto_mission and o._auto_tick(0.0))
        assert called == [], "開關關著卻仍然評估了 _auto_tick"

    def test_the_align_dispatch_short_circuits_on_the_switch(self):
        """⚠️ 結構鎖:`auto_mission` 必須排在 `_auto_tick` **之前**。

        反過來寫(`self._auto_tick(now) and self.auto_mission`)在關閉時
        仍然會跑整段上傳邏輯 —— 開關就只剩「不下令」而不是「不存在」,
        而那正是「逐位元舊行為」這句話會變成謊的地方。
        """
        line = [ln for ln in SRC.splitlines()
                if 'self.phase == "ALIGN"' in ln and "self.auto_mission" in ln]
        assert len(line) == 1, f"ALIGN 的 auto 分派入口不唯一:{line}"
        s = line[0]
        assert s.index("self.auto_mission") < s.index("self._auto_tick"), (
            "開關必須排在 _auto_tick 之前(短路求值)—— 否則關著時仍會執行上傳")


# ══════════════════════════════════════════════════════════════════════
#  任務航線的幾何與欄位
# ══════════════════════════════════════════════════════════════════════

class TestTheMissionItself:
    def test_the_shape_is_home_then_loiter(self):
        items = mp.build_strike_mission(
            here=(25.0, 120.0), ip=(25.1, 120.1), ip_alt=3100.0, loiter_r=3300.0)
        assert [it.command for it in items] == [
            mp.MAV_CMD_NAV_WAYPOINT, mp.MAV_CMD_NAV_LOITER_UNLIM]
        assert [it.seq for it in items] == [0, 1]

    def test_the_transit_leg_is_inserted_in_the_middle(self):
        items = mp.build_strike_mission(
            here=(25.0, 120.0), ip=(25.1, 120.1), ip_alt=3100.0,
            loiter_r=3300.0, transit=(25.5, 120.5))
        assert [it.command for it in items] == [
            mp.MAV_CMD_NAV_WAYPOINT, mp.MAV_CMD_NAV_WAYPOINT,
            mp.MAV_CMD_NAV_LOITER_UNLIM]
        assert [it.seq for it in items] == [0, 1, 2], "seq 必須連續且從 0 起"
        # 盤旋一定是**最後**一項 —— 它是「停在這裡等」的那一項
        assert items[-1].command == mp.MAV_CMD_NAV_LOITER_UNLIM

    def test_seq_zero_is_the_home_placeholder(self):
        """⚠️⚠️ 少了 seq 0 就是經典的離一錯誤:飛控要 seq N-1 而我們只有
        N-1 項 ⇒ 送不出去 ⇒ 逾時,而逾時的訊息長得像「鏈路壞了」。"""
        here = (24.5, 121.5)
        items = mp.build_strike_mission(
            here=here, ip=(25.1, 120.1), ip_alt=3100.0, loiter_r=3300.0)
        assert items[0].seq == 0
        assert items[0].current == 1, "MAVLink 慣例:只有 seq 0 的 current=1"
        assert all(it.current == 0 for it in items[1:])
        assert items[0].x == int(round(here[0] * 1e7))
        assert items[0].y == int(round(here[1] * 1e7))

    def test_the_loiter_radius_lands_in_param3(self):
        """⚠️ param3 放錯格 = 飛控拿到 0 ⇒ 退回 WP_LOITER_RAD 再乘 eas2tas²
        ⇒ 實際 2030 m,而我們**以為**半徑是指定的那個。"""
        items = mp.build_strike_mission(
            here=(25.0, 120.0), ip=(25.1, 120.1), ip_alt=3100.0, loiter_r=3300.0)
        loiter = items[-1]
        assert loiter.param3 == 3300.0
        assert (loiter.param1, loiter.param2, loiter.param4) == (0.0, 0.0, 0.0)

    def test_a_zero_radius_is_refused_not_defaulted(self):
        """⚠️⚠️ 傳 0 必須**拒絕**,不可以「善意地」給個預設值 ——
        善意的預設會讓一個設錯的組態靜靜地變成能跑的。"""
        for bad in (0.0, -1.0):
            with pytest.raises(ValueError):
                mp.build_strike_mission(here=(25.0, 120.0), ip=(25.1, 120.1),
                                        ip_alt=3100.0, loiter_r=bad)

    def test_altitudes_are_relative_frame(self):
        """⚠️ frame 必須是 GLOBAL_RELATIVE_ALT_INT —— perch_of() 給的是
        相對 HOME 的高度,用 AMSL frame 會整個差一個地形落差
        (本專案吃過 1821 m 的虧)。"""
        items = mp.build_strike_mission(
            here=(25.0, 120.0), ip=(25.1, 120.1), ip_alt=3100.0, loiter_r=3300.0)
        assert all(it.frame == mp.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                   for it in items)
        assert items[-1].z == 3100.0

    def test_the_field_order_is_the_mavlink_order(self):
        """⚠️⚠️ tot_node 用**位置引數**把它餵進 mission_item_int_send ⇒
        欄位順序是介面的一部分,不是內部細節。改順序 = 每一格都錯位,
        而每一格都還是個合法的數字。"""
        assert mp.MissionItem._fields == (
            "seq", "frame", "command", "current", "autocontinue",
            "param1", "param2", "param3", "param4", "x", "y", "z",
            "mission_type")

    @pytest.mark.parametrize("const,want", [
        ("MAV_FRAME_GLOBAL_RELATIVE_ALT_INT", 6),
        ("MAV_CMD_NAV_WAYPOINT", 16),
        ("MAV_CMD_NAV_LOITER_UNLIM", 17),
        ("MAV_MISSION_TYPE_MISSION", 0),
        ("MAV_MISSION_ACCEPTED", 0),
    ])
    def test_the_hardcoded_constants_match_pymavlink(self, const, want):
        """⚠️ mission_plan 刻意不 import pymavlink(開發機沒有),所以常數是
        寫死的。在**有** pymavlink 的機器上比對真值;沒有就 skip ——
        但上面的數值鎖照跑,不會整組消失。"""
        assert getattr(mp, const) == want
        mavutil = pytest.importorskip("pymavlink.mavutil")
        assert getattr(mavutil.mavlink, const) == want


# ══════════════════════════════════════════════════════════════════════
#  上傳協商的狀態機
# ══════════════════════════════════════════════════════════════════════

def _items(n=3):
    return mp.build_strike_mission(
        here=(25.0, 120.0), ip=(25.1, 120.1), ip_alt=3100.0,
        loiter_r=3300.0, transit=(25.5, 120.5))[:n]


class TestTheUploadHandshake:
    def test_the_happy_path_reaches_done(self):
        up = mp.MissionUploader(_items())
        assert up.begin(0.0) == {"send": "count", "count": 3}
        for seq in range(3):
            act = up.on_request(seq, 0.1 * seq)
            assert act["send"] == "item" and act["seq"] == seq
        assert up.on_ack(0, 1.0) == {"done": True}
        assert up.ok and up.finished

    def test_out_of_order_and_repeated_requests_are_served_verbatim(self):
        """⚠️ MAVLink 允許飛控重要同一項(丟包)或跳回去。假設遞增 = 少送。"""
        up = mp.MissionUploader(_items())
        up.begin(0.0)
        for seq in (2, 0, 2, 1):
            act = up.on_request(seq, 1.0)
            assert act["item"].seq == seq
        assert up.on_ack(0, 2.0) == {"done": True}

    def test_a_premature_accept_is_not_believed(self):
        """⚠️⚠️ 早到的 ACCEPTED 有兩個來源(上一次上傳的遲到 ACK、我們的
        count 少算一項),兩者都會讓「上傳成功」變成假綠燈,而假綠燈的
        下游是一架飛在錯誤航線上的飛機。寧可等到逾時再大聲失敗。"""
        up = mp.MissionUploader(_items())
        up.begin(0.0)
        up.on_request(0, 0.1)
        act = up.on_ack(0, 0.2)
        assert act["ignored"] == "premature_ack"
        assert not up.ok and not up.finished, "早到的 ACK 不可以讓它變成 done"

    def test_a_rejecting_ack_fails_loudly(self):
        up = mp.MissionUploader(_items())
        up.begin(0.0)
        act = up.on_ack(3, 1.0)          # 3 = MAV_MISSION_UNSUPPORTED
        assert "failed" in act and up.state == "failed"
        assert "type=3" in up.reason

    def test_an_out_of_range_request_fails_instead_of_hanging(self):
        """⚠️ 越界表示 count 與飛控理解不一致(離一錯誤的表徵)。
        忽略它只會逾時,而逾時的訊息會把人帶去查鏈路。"""
        up = mp.MissionUploader(_items())
        up.begin(0.0)
        act = up.on_request(99, 1.0)
        assert "failed" in act and up.state == "failed"

    def test_it_resends_on_silence_then_gives_up(self):
        up = mp.MissionUploader(_items(), item_timeout=1.0,
                                total_timeout=1000.0, max_retry=2)
        up.begin(0.0)
        assert up.poll(0.5) is None, "還沒到逾時就重送 = 洗鏈路"
        assert up.poll(2.0) == {"send": "count", "count": 3}
        assert up.poll(4.0) == {"send": "count", "count": 3}
        act = up.poll(6.0)
        assert "failed" in act, "重送用完了還不失敗 = 無限重送"

    def test_the_total_timeout_wins_even_while_retrying(self):
        up = mp.MissionUploader(_items(), item_timeout=1.0, total_timeout=5.0,
                                max_retry=1000)
        up.begin(0.0)
        for t in (2.0, 3.0, 4.0):
            up.poll(t)
        assert "failed" in up.poll(6.0)
        assert "逾時" in up.reason

    def test_terminal_states_are_sticky(self):
        """⚠️ 一則遲到的 MISSION_REQUEST 不可以讓已經失敗的上傳「復活」——
        復活之後的狀態沒有任何人在追。"""
        up = mp.MissionUploader(_items())
        up.begin(0.0)
        up.on_ack(3, 1.0)
        assert up.state == "failed"
        assert up.on_request(0, 2.0) is None
        assert up.poll(99.0) is None
        assert up.state == "failed"

    def test_an_empty_mission_is_refused(self):
        """⚠️ 空清單上傳會讓飛控**清掉**現有任務,而且看起來像成功。"""
        with pytest.raises(ValueError):
            mp.MissionUploader([])

    def test_progress_counts_distinct_items_not_sends(self):
        """⚠️ 用集合不是計數器:飛控重要同一項時,計數器會以為進度比實際快,
        而「以為送完了」的下一步是接受一則早到的 ACK ⇒ 假綠燈。"""
        up = mp.MissionUploader(_items())
        up.begin(0.0)
        for _ in range(5):
            up.on_request(0, 1.0)
        assert up.progress() == (1, 3)


# ══════════════════════════════════════════════════════════════════════
#  紅線 ⑤:白名單與 if/elif 鏈是一對(裸 else 會吃掉新型別)
# ══════════════════════════════════════════════════════════════════════

class TestTheLinkDispatchCoversEveryAllowedType:
    def test_the_mission_types_are_in_the_allowlist(self):
        assert {"MISSION_REQUEST", "MISSION_REQUEST_INT",
                "MISSION_ACK"} <= set(TotNode._LINK_TYPES)

    def test_every_allowed_type_has_its_own_branch(self):
        """⚠️⚠️⚠️ `_pump_link` 的最後一支是**裸 else**(當成 TERRAIN_REPORT
        在讀 `m.spacing`)。只把型別加進白名單、忘記加 elif 的後果不是
        「被忽略」,而是那則訊息被當成地形回報 ⇒ AttributeError ⇒
        **整條接收迴圈掛掉**,而症狀出現在完全不相干的地方
        (節點停在 INIT、定位不更新)。這正是 [[bare-else-in-dispatch]]。

        ⚠️ TERRAIN_REPORT 自己就是那個裸 else,所以它不需要 elif。
        """
        body = SRC[SRC.index("    def _pump_link"):]
        body = body[:body.index("\n    def ", 10)]
        missing = [t for t in TotNode._LINK_TYPES
                   if t != "TERRAIN_REPORT" and f'"{t}"' not in body]
        assert not missing, (
            f"⚠️⚠️ 這些型別在 _LINK_TYPES 白名單裡但 _pump_link 沒有對應分支:"
            f"{missing} ⇒ 會落進裸 else 被當成地形回報 ⇒ 讀 m.spacing 爆掉 ⇒ "
            f"整條接收迴圈死掉,而症狀出現在別的地方")

    def test_a_mission_message_never_reaches_the_terrain_branch(self):
        """⭐ 行為級:餵一則**沒有 spacing 屬性**的 MISSION_ACK 進去,
        如果它落到地形分支就會 AttributeError。這條證明分支真的在。"""
        o = mk()
        seen = []
        o._mission_event = lambda i, kind, val: seen.append((i, kind, val))

        class _Msg:                       # 刻意沒有 .spacing / .lat / .lon
            def __init__(self, mt, **kw):
                self._mt = mt
                self.__dict__.update(kw)

            def get_type(self):
                return self._mt

        msgs = [_Msg("MISSION_REQUEST_INT", seq=2),
                _Msg("MISSION_ACK", type=0),
                _Msg("MISSION_REQUEST", seq=1)]

        class _C:
            target_system = 1

            def __init__(self, q):
                self._q = list(q)

            def recv_match(self, blocking=False):
                return self._q.pop(0) if self._q else None

        o.link = {1: _C(msgs)}
        o.phase = "ALIGN"
        o._pump_link()                    # 不可以拋例外
        assert seen == [(1, "request", 2), (1, "ack", 0), (1, "request", 1)], (
            f"協商訊息沒有被分派到 _mission_event:{seen}")


# ══════════════════════════════════════════════════════════════════════
#  攻擊發起點的幾何
# ══════════════════════════════════════════════════════════════════════

class TestTheInitialPoint:
    def test_zero_means_the_pushover_ring(self):
        o = mk(ip_r=0.0)
        for i in (1, 2):
            r = tot_node.dist_m(o.T, o._auto_ip_of(i))
            assert r == pytest.approx(o.ring_of(i), rel=1e-6), (
                "auto_ip_r=0 的語意是『用本機自己的環半徑』")

    def test_an_explicit_range_becomes_a_design_quantity(self):
        """⭐⭐⭐ 這個架構存在的**唯一理由**:R0 從湧現量變成設計量。

        R0 分箱(98 架次)是單調的,7000+ 那一格 20 架次沒有任何一個
        殘餘高度超過 100 m。設 7500 就必須真的量到 7500。
        """
        o = mk(ip_r=7500.0)
        for i in (1, 2):
            r = tot_node.dist_m(o.T, o._auto_ip_of(i))
            assert r == pytest.approx(7500.0, rel=2e-3), (
                f"設了 auto_ip_r=7500 卻量到 {r:.0f} m —— "
                f"R0 還是湧現量,這個架構就白做了")

    def test_each_aircraft_gets_its_own_fan_bearing(self):
        """⚠️ 各機的攻擊發起點必須落在**自己**那條徑向線上 —— 否則兩架
        會盤旋在同一個點上(而防撞只靠高度分層)。"""
        o = mk(n=2, ip_r=7000.0)
        pts = [o._auto_ip_of(i) for i in (1, 2)]
        assert tot_node.dist_m(pts[0], pts[1]) > 1000.0, (
            "兩架的攻擊發起點幾乎重合 —— 扇形偏置沒有被套用")

    def test_the_bearing_matches_the_ring_point_and_xtrack(self):
        """⚠️⚠️ 方位算式在三個地方各有一份(`ring_point_of` / `_xtrack` /
        這裡)。不一致的話「對正」與「到達」量的是**不同的線**,
        而兩者都會回報成功。"""
        o = mk(ip_r=0.0)
        for i in (1, 2):
            want = (o.approach + o._fan_off(o.slot_id[i])) % 360.0
            assert o._auto_bearing_of(i) == pytest.approx(want)
            # 環點也必須落在同一條線上
            assert tot_node.dist_m(o.ring_point_of(i), o._auto_ip_of(i)) < 1.0

    def test_zero_altitude_means_perch_of(self):
        o = mk()
        o.auto_ip_alt = 0.0
        assert o._auto_ip_alt_of(1) == o.perch_of(1)
        o.auto_ip_alt = 2500.0
        assert o._auto_ip_alt_of(1) == 2500.0


# ══════════════════════════════════════════════════════════════════════
#  紅線 ②③:全員到齊的閘門
# ══════════════════════════════════════════════════════════════════════

class TestTheAllArrivedGate:
    def test_it_uses_gate_all_not_a_bare_length_check(self):
        """⚠️⚠️ ALIGNW1 的死結:`len(self.arr) == self.n` 在有機體被終結之後
        **數學上不可能成立** ⇒ 等了 1551 s 到逾時。新閘門不可以重演。

        ⚠️ 沿用 `self.arr` 而不是新開一格,是為了讓 test_kill_deadlock.py
           那條掃描 `arr|ready|perch_arr` 的正則**也涵蓋它** ——
           新開一格 = 那條守衛掃不到,而掃不到的守衛不會吭聲。
        """
        blk = SRC[SRC.index("if self.phase == \"ALIGN\" and self.auto_mission"):]
        blk = blk[:blk.index("elif self.phase == \"ALIGN\":")]
        code = _code_only(blk)
        assert "self._gate_all(self.arr)" in code
        # ⚠️ 先剝註解 —— 上面那段的註解本身就寫著 `len(self.arr) >= self.n`
        #    (在解釋「為什麼不能那樣寫」)。不剝就會對正確的程式碼開火。
        assert "len(self.arr)" not in code, "又出現裸長度比較 —— ALIGNW1 重演"

    def test_a_killed_aircraft_does_not_block_the_gate(self):
        """⭐ 行為級:兩架裡有一架被終結,剩下那架到了就要開閘。"""
        o = mk(n=2)
        o.killed = {2: 1.0}
        o.lost_at = {2: 1.0}
        o.arr = {1: 1.0}
        assert o._gate_all(o.arr) is True

    def test_an_all_dead_fleet_does_not_advance(self):
        """⚠️ `_gate_all` 刻意**不是** `len(got) >= _n_live()` —— 全滅時
        分母是 0,那樣寫會讓空集合通過,而每個下游計數都是 0/0。"""
        o = mk(n=2)
        o.killed = {1: 1.0, 2: 1.0}
        o.lost_at = {1: 1.0, 2: 1.0}
        assert o._gate_all({}) is False

    def test_the_gate_can_actually_open_with_the_shipped_defaults(self):
        """⚠️⚠️⚠️ **這一條在證明「這個組態真的能讓事情發生」。**

        [[instrument-that-cannot-fire]]:換向計數器被放進一個恆為正的
        量值函式 ⇒ 那一欄永遠不可能加一,而恆空正好被讀成「量到零換向」。
        ⇒ 每一條測試都要先斷言「這個設定真的能讓 X 發生」。

        這裡要證的是:飛機在**盤旋**時(離圓心 = 盤旋半徑)抵達判定
        仍然成立。出貨預設 reach=5000 > loiter=3300 ⇒ 成立。
        """
        o = mk(ip_reach=5000.0, loiter_r=3300.0, ip_r=7000.0)
        _ip_pts(o)
        _finish_upload(o)
        # 把飛機放在盤旋圈上(距圓心恰好 = 盤旋半徑),這是它真正待的地方
        for i in (1, 2):
            ip = o._ip_of[i]
            o.pos[i] = (ip[0] + o.auto_loiter_r / tot_node.EARTH, ip[1], 3100.0)
        o._auto_tick(1.79e9)
        assert set(o.arr) == {1, 2}, (
            "飛機就在盤旋圈上卻沒被判定抵達 —— 這個閘門永遠不會開")

    def test_a_reach_inside_the_loiter_radius_can_never_latch(self):
        """⭐ 反向鎖:證明上面那條**不是**自動成立的。

        reach < loiter 時飛機盤旋一整圈都不會被判定抵達 ⇒ 死結。
        契約層有 `_inv_auto_reach_above_loiter` 擋這個組合,而這一條
        證明「擋它是有理由的」,不是憑空多一道檢查。
        """
        o = mk(ip_reach=1000.0, loiter_r=3300.0, ip_r=7000.0)
        _ip_pts(o)
        _finish_upload(o)
        for i in (1, 2):
            ip = o._ip_of[i]
            for deg in range(0, 360, 15):     # 繞完整一圈
                brg = math.radians(deg)
                o.pos[i] = (ip[0] + o.auto_loiter_r * math.cos(brg) / tot_node.EARTH,
                            ip[1] + o.auto_loiter_r * math.sin(brg)
                            / (tot_node.EARTH * math.cos(math.radians(ip[0]))),
                            3100.0)
                o._auto_tick(1.79e9)
        assert o.arr == {}, (
            "reach < loiter 卻латch 了 —— 那表示判定用的不是離圓心的距離")


# ══════════════════════════════════════════════════════════════════════
#  紅線 ④:上傳失敗大聲中止
# ══════════════════════════════════════════════════════════════════════

class TestFailureIsLoud:
    def test_an_upload_failure_aborts_the_sortie_by_default(self):
        """⚠️⚠️ 不可以只是「不做事」:那會讓 all-n 閘門永遠等下去,
        而 1500 s 的逾時看起來像節點掛了。"""
        o = mk(fallback=False)
        _ip_pts(o)
        o._mission_event(1, "ack", 3)        # 飛控拒收
        assert o._auto_failed
        handled = o._auto_tick(1.79e9)
        assert handled is True
        assert o.phase == "DONE", "上傳失敗卻沒有結案 ⇒ 等到逾時"
        assert "AUTO" in o._log.text()

    def test_the_failure_is_actually_logged_as_an_error(self):
        o = mk()
        _ip_pts(o)
        o._mission_event(1, "ack", 3)
        assert "⚠️⚠️" in o._log.text() and "失敗" in o._log.text()

    def test_fallback_is_opt_in_and_returns_control_to_guided(self):
        """⭐ 顯式打開才退回,而且退回時 `_auto_tick` 回 False ⇒ ALIGN
        分派落到 GUIDED 那一支。"""
        o = mk(fallback=True)
        _ip_pts(o)
        o._mission_event(1, "ack", 3)
        assert o._auto_tick(1.79e9) is False
        assert o.phase == "ALIGN", "退回時不該結案"

    def test_the_default_really_is_abort_not_fallback(self):
        """⚠️ 反向鎖:證明上面兩條測的是**同一份**程式碼的兩個分支,
        而不是兩條互不相干的路徑。"""
        assert TotNode.auto_fallback_guided is False


# ══════════════════════════════════════════════════════════════════════
#  紅線 ⑥:交回 GUIDED 的接續
# ══════════════════════════════════════════════════════════════════════

class TestTheHandoffToGuided:
    def _at_ip(self, o):
        _ip_pts(o)
        _finish_upload(o)
        for i in list(o.pos):
            ip = o._ip_of[i]
            o.pos[i] = (ip[0], ip[1], 3100.0)
        return o

    def test_it_commands_guided_for_every_live_aircraft(self):
        o = self._at_ip(mk(ip_r=7000.0))
        o.sent.clear()
        o._auto_handoff(1.79e9)
        modes = [a for kind, a in o.sent if kind == "cmd_long"]
        # command_long_send(tgt_sys, tgt_comp, cmd, confirm, p1..p7)
        # ⇒ custom_mode 是 **param2 = a[5]**(param1 = a[4] 是 base_mode 旗標)。
        got = {a[0]: a[5] for a in modes}          # {target_system: custom_mode}
        assert got == {1: float(tot_node.PLANE_MODE_GUIDED),
                       2: float(tot_node.PLANE_MODE_GUIDED)}, (
            f"交接時沒有對每一架送 GUIDED:{got}")

    def test_it_skips_lost_airframes(self):
        o = self._at_ip(mk(ip_r=7000.0))
        o.killed = {2: 1.0}
        o.lost_at = {2: 1.0}
        o.sent.clear()
        o._auto_handoff(1.79e9)
        tgts = {a[0] for kind, a in o.sent if kind == "cmd_long"}
        assert tgts == {1}, "對已終結的殘骸下令 = 又一次對空氣說話"

    def test_runup_is_preset_so_perch_does_not_fly_back_to_the_fan(self):
        """⚠️⚠️ **這一條擋的是一個具體的錯誤指令。**

        不預設 `runup_t0` 的話,PERCH 的第一個 tick 會走
        `runup_t0 is None` 那一支 = mode "climb" =
        `_goto(slot_of[i], perch_alt, v_cl)` —— 而 slot 在 r_fan
        (生產值 **35 km**)上 ⇒ 剛飛到攻擊發起點的飛機會收到一發
        「回頭飛 35 公里」的 DO_REPOSITION,順帶把高度重錨一次。
        下一拍才會被修正,但那一發已經送出去了。
        """
        o = self._at_ip(mk(ip_r=7000.0))
        o._auto_handoff(1.79e9)
        assert o.runup_t0 == 1.79e9, "runup_t0 沒預設 ⇒ PERCH 會先送 35 km 的航點"
        assert o.perch_T is not None and o.perch_T > 1.79e9

    def test_ready_is_latched_so_the_two_gates_agree(self):
        """⚠️ `_gate_all(self.ready)` 是 `runup_t0` 的**唯一**設定點。
        兩者不一致會讓 log 說的「全員就緒」時刻與實際內切時刻差一整段。"""
        o = self._at_ip(mk(ip_r=7000.0))
        o._auto_handoff(1.79e9)
        assert set(o.ready) == {1, 2}
        assert o._gate_all(o.ready) is True

    def test_it_records_the_handoff_range_per_aircraft(self):
        """⭐ 交接當拍的斜距是**中介變數**:沒有它就分不出「放行幾何沒鎖住」
        與「鎖住了但下游沒用」,而那兩者的下一步完全相反。"""
        o = self._at_ip(mk(ip_r=7000.0))
        o._auto_handoff(1.79e9)
        assert set(o._auto_r0) == {1, 2}
        for i in (1, 2):
            assert o._auto_r0[i] == pytest.approx(
                tot_node.dist_m(o.pos[i][:2], o.T), abs=0.05)

    def test_it_shouts_when_release_at_ring_is_off(self):
        """⚠️⚠️ AUTO 的全部價值在「R0 從湧現量變設計量」,而 release_at_ring
        關著時放行改看時刻表 ⇒ R0 又變回湧現量。契約層擋一次,現場再吼一次
        —— 契約是靜態的,log 是現場的。"""
        o = self._at_ip(mk(ip_r=7000.0))
        o.release_at_ring = False
        o._auto_handoff(1.79e9)
        txt = o._log.text()
        # ⚠️⚠️ 斷言要鎖在**只有那一行 ERROR** 才有的字串上。
        #    第一版斷言 `"release_at_ring" in txt and "湧現量" in txt`,
        #    而交接摘要那一行**同時**含有這兩個字串 ⇒ 把 ERROR 整段拿掉
        #    (突變 `if False and not ...`)測試照樣綠。
        #    這是 [[verifying-the-verifier]] 的「斷言字串互相包含」。
        assert "auto_mission 開著但 release_at_ring=False" in txt

    def test_it_stays_quiet_when_release_at_ring_is_on(self):
        """⭐ 反向鎖:證明上面那條抓的是**開關**,不是每次都吼。"""
        o = self._at_ip(mk(ip_r=7000.0))
        o.release_at_ring = True
        o._auto_handoff(1.79e9)
        assert "**OFF**" not in o._log.text()

    def test_the_mode_command_is_do_set_mode_with_custom_flag(self):
        """⚠️ 走 COMMAND_LONG 才有 COMMAND_ACK 可讀 —— SET_MODE 訊息沒有
        回覆,送出去就沒下文,而本節點已經在數 ack_bad_own。"""
        mavutil = pytest.importorskip("pymavlink.mavutil")
        o = self._at_ip(mk(ip_r=7000.0))
        o.sent.clear()
        o._auto_send_mode(1, tot_node.PLANE_MODE_AUTO, 1.79e9)
        kind, a = o.sent[0]
        assert kind == "cmd_long"
        assert a[2] == mavutil.mavlink.MAV_CMD_DO_SET_MODE
        assert a[4] == float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        assert a[5] == float(tot_node.PLANE_MODE_AUTO)

    def test_auto_is_commanded_only_after_every_upload_finished(self):
        """⚠️ 上傳還沒完就切 AUTO = 飛控拿著半條航線起飛。"""
        o = mk(ip_r=7000.0)
        _ip_pts(o)
        o.sent.clear()
        o._auto_tick(1.79e9)              # 還沒 ACK
        assert not [a for kind, a in o.sent if kind == "cmd_long"], (
            "上傳未完成就送了模式切換")
        _finish_upload(o)
        o._auto_tick(1.79e9)
        got = {a[0]: a[5] for kind, a in o.sent if kind == "cmd_long"}
        assert got == {1: float(tot_node.PLANE_MODE_AUTO),
                       2: float(tot_node.PLANE_MODE_AUTO)}


# ══════════════════════════════════════════════════════════════════════
#  模式讀回(送出去 ≠ 生效)
# ══════════════════════════════════════════════════════════════════════

class TestModeReadback:
    def test_a_mode_that_never_takes_is_reported_and_resent(self):
        """⚠️⚠️ 「送出去 ≠ 生效」在本專案有前科:DO_CHANGE_SPEED 被整發
        拒收 49 次而沒有任何一行 log。HEARTBEAT.custom_mode 是唯一讀回管道。"""
        o = mk(ip_r=7000.0)
        _ip_pts(o)
        _finish_upload(o)
        o._auto_tick(1000.0)                     # 送出 AUTO
        o.fc_mode = {1: tot_node.PLANE_MODE_AUTO, 2: tot_node.PLANE_MODE_GUIDED}
        o.sent.clear()
        o._auto_tick(1000.0 + o.auto_mode_timeout + 1.0)
        resent = {a[0] for kind, a in o.sent if kind == "cmd_long"}
        assert resent == {2}, f"沒切成功的那一架沒有被重送:{resent}"
        assert "HEARTBEAT" in o._log.text()

    def test_an_aircraft_already_in_auto_is_not_nagged(self):
        o = mk(ip_r=7000.0)
        _ip_pts(o)
        _finish_upload(o)
        o._auto_tick(1000.0)
        o.fc_mode = {1: tot_node.PLANE_MODE_AUTO, 2: tot_node.PLANE_MODE_AUTO}
        o.sent.clear()
        o._auto_tick(1000.0 + o.auto_mode_timeout + 1.0)
        assert not [a for kind, a in o.sent if kind == "cmd_long"]

    def test_the_two_plane_mode_numbers_are_right(self):
        mavutil = pytest.importorskip("pymavlink.mavutil")
        assert tot_node.PLANE_MODE_GUIDED == 15
        assert tot_node.PLANE_MODE_AUTO == 10
        assert mavutil.mavlink.PLANE_MODE_AUTO == tot_node.PLANE_MODE_AUTO
        assert mavutil.mavlink.PLANE_MODE_GUIDED == tot_node.PLANE_MODE_GUIDED


# ══════════════════════════════════════════════════════════════════════
#  逐發重置(第二發要重新上傳)
# ══════════════════════════════════════════════════════════════════════

class TestPerSortieReset:
    def test_the_three_dicts_are_in_the_reset_manifest(self):
        """⚠️⚠️ 漏掉 `_mission` 的後果是**第二發完全不上傳**:上傳器還在且
        state=="done" ⇒ 上傳迴圈直接跳過 ⇒ 飛控裡是**上一個紅心**的航線,
        而節點以為一切就緒。同 wp_key/wp_t/wp_pt 的病,尺度大一整條航線。"""
        assert {"_mission", "_ip_of", "_auto_mode_t",
                "_auto_r0"} <= set(tot_node.RESET_DICTS)

    def test_a_stale_uploader_would_have_skipped_the_second_upload(self):
        """⭐ 反向鎖:證明上面那條**擋得住真的東西**。

        不清 `_mission` 的話,`_auto_start` 之後的上傳迴圈看到 finished
        的上傳器就不會再送任何東西。
        """
        o = mk(ip_r=7000.0)
        _ip_pts(o)
        _finish_upload(o)
        assert all(up.ok for up in o._mission.values())
        o.sent.clear()
        # 模擬「沒有清乾淨」:_auto_started 重置但 _mission 留著
        o._auto_started = False
        o._auto_tick(1.79e9)
        # _auto_start 會覆寫 _mission ⇒ 一定會重送 count。這正是我們要的。
        assert [a for kind, a in o.sent if kind == "count"], (
            "第二發沒有重新上傳 —— 飛控裡還是上一個紅心的航線")


# ══════════════════════════════════════════════════════════════════════
#  契約層
# ══════════════════════════════════════════════════════════════════════

class TestTheContractCoversIt:
    def _table(self):
        from contract_table import BY_NAME
        return BY_NAME

    NAMES = ("auto_mission", "auto_ip_r", "auto_ip_alt", "auto_ip_reach",
             "auto_loiter_r", "auto_transit_r", "auto_upload_timeout",
             "auto_mode_timeout", "auto_fallback_guided")

    @pytest.mark.parametrize("name", NAMES)
    def test_every_parameter_is_in_the_table(self, name):
        assert name in self._table()

    @pytest.mark.parametrize("name", NAMES)
    def test_every_parameter_has_an_env_mapping(self, name):
        """⚠️ 違規訊息印出的重跑指令少一個變數,使用者照貼就會跑到別的組態。
        對 `auto_mission` 更關鍵:它是 requested_vs_effective 唯一的把手。"""
        from contract_table import ENV_FOR
        assert name in ENV_FOR

    @pytest.mark.parametrize("name", NAMES)
    def test_every_parameter_is_read_back(self, name):
        """⚠️⚠️ 契約表加了列而 contract_verify 沒同步 ⇒ 一律判 missing,
        而 missing 的訊息長得像「節點沒起來」,整批會在第一趟被自己擋掉。"""
        src = (ROOT / "scripts" / "wsl" / "contract_verify.py").read_text(
            encoding="utf-8")
        assert f'"{name}"' in src

    @pytest.mark.parametrize("name", NAMES)
    def test_every_parameter_is_printed_in_the_contract_line(self, name):
        assert f'("{name}"' in SRC, (
            f"{name} 沒印進 _contract_values ⇒ contract_verify 會判 missing")

    @pytest.mark.parametrize("name", ("auto_mission", "auto_fallback_guided"))
    def test_the_bools_are_registered_as_bools(self, name):
        """⚠️ 印成 1.0 貼上去 = InvalidParameterTypeException = 節點起不來。
        同一類坑本專案踩過四次。"""
        from contract_table import BOOL_PARAMS
        assert name in BOOL_PARAMS

    def test_the_deadlock_invariant_fires(self):
        from contract_table import evaluate
        v = dict(auto_mission=1.0, auto_ip_reach=1000.0, auto_loiter_r=3300.0,
                 vmax=150.0, perch_alt=3100.0)
        names = {x.name for x in evaluate(v).violations}
        assert "auto_ip_reach > auto_loiter_r" in names

    def test_the_deadlock_invariant_is_silent_when_the_switch_is_off(self):
        """⚠️ 關著時這兩顆不參與任何判定 ⇒ 不可以擋住舊架構的航次。"""
        from contract_table import evaluate
        v = dict(auto_mission=0.0, auto_ip_reach=1000.0, auto_loiter_r=3300.0,
                 vmax=150.0, perch_alt=3100.0)
        names = {x.name for x in evaluate(v).violations}
        assert "auto_ip_reach > auto_loiter_r" not in names

    def test_the_flyability_invariant_fires(self):
        """⚠️ hold_radius=1300 的指令實測飛成 5867 m 的圈 —— 指定一個飛不
        出來的半徑,攻擊發起點的實際位置就變回湧現量,而這個架構存在的
        唯一理由就是讓它變成設計量。"""
        from contract_table import evaluate
        v = dict(auto_mission=1.0, auto_ip_reach=5000.0, auto_loiter_r=1300.0,
                 vmax=150.0, perch_alt=3100.0)
        names = {x.name for x in evaluate(v).violations}
        assert "auto_loiter_r ≥ 物理迴轉半徑" in names

    def test_the_shipped_defaults_satisfy_both_invariants(self):
        """⭐⭐ **這一條是最重要的一條**:出貨的預設組態必須真的能跑。

        [[promised-guards-that-do-not-exist]] 的反面 —— 一道會對正確輸入
        開火的守衛,下一個人只會把它刪掉。
        """
        from contract_table import JET_REFERENCE, evaluate
        v = dict(auto_mission=1.0, vmax=150.0, perch_alt=3100.0,
                 auto_ip_reach=JET_REFERENCE["auto_ip_reach"],
                 auto_loiter_r=JET_REFERENCE["auto_loiter_r"])
        names = {x.name for x in evaluate(v).violations}
        assert "auto_ip_reach > auto_loiter_r" not in names
        assert "auto_loiter_r ≥ 物理迴轉半徑" not in names

    def test_auto_requires_release_at_ring(self):
        """⚠️⚠️ 模擬實測:閘門即放行 R0 全距 1987 m(≈盤旋直徑)、
        抵環才放行 5 m。1987 m 會讓一部分機體落進「R0<4000」那一格,
        而那一格 71.4% 殘餘超標 ⇒ 不是合法工作點,是已知會壞的組合。"""
        from contract_table import evaluate
        v = dict(auto_mission=1.0, release_at_ring=0.0, vmax=150.0,
                 perch_alt=3100.0, auto_ip_reach=5000.0, auto_loiter_r=3300.0)
        names = {x.name for x in evaluate(v).violations}
        assert "auto_mission 需要 release_at_ring" in names

    def test_that_guard_is_silent_when_release_at_ring_is_on(self):
        from contract_table import evaluate
        v = dict(auto_mission=1.0, release_at_ring=1.0, vmax=150.0,
                 perch_alt=3100.0, auto_ip_reach=5000.0, auto_loiter_r=3300.0)
        names = {x.name for x in evaluate(v).violations}
        assert "auto_mission 需要 release_at_ring" not in names

    def test_that_guard_is_silent_when_auto_is_off(self):
        """⚠️ 關著時這條不可以擋住舊架構的航次 —— 舊架構本來就允許
        release_at_ring=0(生產預設就是)。"""
        from contract_table import evaluate
        v = dict(auto_mission=0.0, release_at_ring=0.0, vmax=150.0,
                 perch_alt=3100.0, auto_ip_reach=5000.0, auto_loiter_r=3300.0)
        names = {x.name for x in evaluate(v).violations}
        assert "auto_mission 需要 release_at_ring" not in names

    def test_both_invariants_are_wired(self):
        """⚠️ 定義了沒掛進 INVARIANTS 的不變式**跑零次**,而契約行照樣印
        「全數通過」。"""
        from contract_table import INVARIANTS
        wired = {f.__name__ for f in INVARIANTS}
        assert {"_inv_auto_reach_above_loiter",
                "_inv_auto_loiter_flyable",
                "_inv_auto_needs_release_at_ring"} <= wired


# ══════════════════════════════════════════════════════════════════════
#  兩條 launcher 的餵值通道
# ══════════════════════════════════════════════════════════════════════

class TestBothLaunchersFeedIt:
    NAMES = TestTheContractCoversIt.NAMES

    @pytest.mark.parametrize("name", NAMES)
    def test_the_interactive_launcher_passes_it(self, name):
        """⚠️⚠️ 沒有 `-p` 通路 = 純 no-op 而且零告警。
        [[two-launcher-paths-differ]]:WV1-3 白跑 2.5 小時,契約閘門擋不住
        因為退回值合法。"""
        sh = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(
            encoding="utf-8")
        assert f"-p {name}:=" in sh

    @pytest.mark.parametrize("name", NAMES)
    def test_the_batch_launcher_passes_it(self, name):
        sh = (ROOT / "scripts" / "wsl" / "stot_trial.sh").read_text(
            encoding="utf-8")
        assert f"-p {name}:=" in sh

    @pytest.mark.parametrize("name", NAMES)
    def test_no_launcher_default_is_an_empty_string(self, name):
        """⚠️⚠️ `-p name:=`(空值)會讓 rcl「Couldn't parse」⇒ 節點整個
        起不來,而症狀長得像埠號被搶走(RRA 整批 8 趟 / 5 小時)。"""
        for f in ("swarm_ros2_up.sh", "stot_trial.sh"):
            sh = (ROOT / "scripts" / "wsl" / f).read_text(encoding="utf-8")
            for line in sh.splitlines():
                if f"-p {name}:=" in line:
                    seg = line.split(f"-p {name}:=", 1)[1]
                    assert not seg.startswith('""') and not seg.startswith("''"), (
                        f"{f} 的 {name} 傳了空字串")

    @pytest.mark.parametrize("name", ("auto_mission", "auto_fallback_guided"))
    def test_the_bool_defaults_are_true_false_words(self, name):
        """⚠️ bool 參數要 `True`/`False`,給 `1.0` 會 InvalidParameterTypeException。"""
        sh = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(
            encoding="utf-8")
        env = name.upper()
        assert f'${{{env}:-False}}' in sh, f"{name} 的 shell 預設不是 False"

    def test_the_new_module_is_in_the_deploy_manifest(self):
        """⚠️⚠️ 清單漏檔已經發作三次,每一次的症狀都是「repo 改了、部署說
        一致、現場跑的是舊的或直接 ImportError,而且沒有任何訊息」。"""
        sh = (ROOT / "scripts" / "wsl" / "deploy_build.sh").read_text(
            encoding="utf-8")
        assert "mission_plan.py" in sh
        assert "sar_swarm.mission_plan" in sh, "冒煙測試沒有涵蓋新模組"


# ══════════════════════════════════════════════════════════════════════
#  編隊層:AUTO 期間不可以搶模式、不可以下令
# ══════════════════════════════════════════════════════════════════════

class TestTheFormationLayerStandsDownInAuto:
    def test_mavros_has_in_auto_mode(self):
        assert "def in_auto_mode" in MI_SRC
        assert '"AUTO"' in MI_SRC and '"CMODE(10)"' in MI_SRC, (
            "要接受 raw id 字串 —— MAVROS 對沒有名稱別名的模式回 CMODE(n)")

    def test_the_change_mode_gate_consults_it(self):
        """⚠️⚠️ 少了這一條,編隊層的一發 DO_REPOSITION 就會把飛機從 AUTO
        拽進 GUIDED —— 與 2026-08-10 把長機從 TAKEOFF 拽出來(整批
        RPG/RPH/RPI 報廢)是**同一個機制**。"""
        fn = None
        for node in ast.walk(ast.parse(MI_SRC)):
            if (isinstance(node, ast.FunctionDef)
                    and node.name == "_may_change_mode"):
                fn = node
        assert fn is not None
        called = {n.func.attr for n in ast.walk(fn)
                  if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        assert "in_auto_mode" in called
        assert "in_takeoff_mode" in called, "舊的第三道不可以被換掉"

    def test_master_whitelists_auto_in_both_mode_reasserts(self):
        """⚠️ master 先前**完全沒有 AUTO 概念**:註解寫著「操作員手動
        TAKEOFF/AUTO」,而條件式沒有排除 AUTO ⇒ 一架在飛任務航線的飛機
        會在 2 秒內被拽回 GUIDED,而且完全靜默。"""
        src = (PATCHES / "master_node.py").read_text(encoding="utf-8")
        hits = [ln for ln in src.splitlines() if "not in (" in ln and "GUIDED" in ln]
        assert hits, "找不到模式白名單 —— 錨點漂了"
        for ln in hits:
            assert '"AUTO"' in ln, f"這一行的白名單少了 AUTO:{ln.strip()}"

    def test_master_stops_commanding_entirely_while_in_auto(self):
        src = (PATCHES / "master_node.py").read_text(encoding="utf-8")
        assert 'if self.mav.mode() == "AUTO":' in src
        i = src.index('if self.mav.mode() == "AUTO":')
        # 必須排在速度控制**之前**
        assert i < src.index("self._leader_speed_control("), (
            "AUTO 檢查排在速度控制之後 ⇒ 任務期間仍會送速度指令")

    def test_backup_checks_the_mode_before_commanding_speed(self):
        """⚠️⚠️ ArduPlane 在 AUTO 之下**會接受** DO_CHANGE_SPEED
        (韌體閘門是 `is_guided_mode() || mode_auto`)⇒ 判別放在
        `_keep_guided()` 裡等於每 tick 都先送一發再說,而任務的速度剖面
        會被靜靜扭曲。"""
        src = (PATCHES / "backup_node.py").read_text(encoding="utf-8")
        blk = src[src.index("    def _formation_control"):]
        blk = blk[:blk.index("\n    def ", 10)]
        i_mode = blk.index('("RTL", "AUTO")')
        i_speed = blk.index("change_speed_async")
        assert i_mode < i_speed, (
            "AUTO/RTL 判別排在 change_speed_async 之後 ⇒ 速度指令仍會送出去")
