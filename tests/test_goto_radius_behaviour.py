"""S2(`wait_goto_radius`)的**值層級**行為鎖 —— 驗「param3 最後是多少」。

═══════════════════════════════════════════════════════════════════════════
為什麼要有這一個檔案(它和 tests/test_wait_goto_radius.py 的分工)
═══════════════════════════════════════════════════════════════════════════
前兩輪已經有 38 條**源碼層**的鎖(tests/test_wait_goto_radius.py):它們用
ast/正規式去驗「那一行長什麼樣」。對抗式再變異合計 **≥25 個逃脫**,病灶只有
一個:

    源碼層的鎖驗的是「那一行長什麼樣」,不是「那個量最後是多少」。

⇒ 所有「不改被鎖那一行、改它上游或下游」的寫法一律免疫。實測逃脫的有:
    N2  改 `self.hold_radius` 的**值來源**(讀 goto_radius 參數)
    N3  `setattr(self, "wait_goto_radius", True)` 旁路(不是 ast.Assign)
    N4  `_rr = (...).get(i)` → `.get(i + 1)`(判定量本身錯機)
    N7  在致動點那一發 `_goto` 加 `keepalive=0.0`(每拍重送)
    N8  `_goto` 簽章預設 `radius=None` → `0.0`(param3=0 = 災難級指令)
    D1  在**送出分支裡**把 `_req` 重新綁成 `self.goto_radius`
    E1  把半徑條件加在**送出閘門**的 `if` 而不是 `moved` 裡
    M12 送出去的半徑與記錄下來的半徑脫鉤

本檔**不再加任何源碼層白名單**(那是軍備競賽,已達邊際效益遞減)。
本檔一律:**真的呼叫 `_goto` / 真的跑 `_tick` 的 DIVE 迴圈,把假鏈路上收到的
每一發 MAVLink 指令錄下來,然後對錄到的 `param3` 做斷言。**
上面 8 個變異每一個都會讓本檔至少一條轉紅(逐一實測過)。

═══════════════════════════════════════════════════════════════════════════
受測的機制(RRD 的真根因,一句話)
═══════════════════════════════════════════════════════════════════════════
runin→wait 切換時那一發 `_goto` 的 lat/lon/alt **逐位元相同**,只有 radius
由 goto_radius 變 hold_radius;而 `_goto` 的變更偵測器存的是
`wp_pt=(lat, lon, alt)` —— **radius 不在裡面** ⇒ moved=False ⇒ 那發
DO_REPOSITION **根本沒有送出去**,飛機掛著舊的 param3 繼續直飛。
hold_radius 要等下一個 30 s keepalive 補發才生效 ⇒ 一架會不會起彎,取決於
它進 wait 時距 30 s 格點還有多久 —— **逐趟隨機**(RRD 16 架次分成兩個母體:
直進 11 架命中 10/11、硬彎 5 架命中 0/5)。

⚠️⚠️ **成功的指紋是「指令的缺席」。** 判定量是 `req_radius`(每拍都寫),
   不是 `sent_radius`(hold-last-value,開關開關都是 30 ⇒ 假綠燈)。

⚠️ 測的是**真的那份** tot_node.py(用 sys.modules 補 rclpy/std_msgs 的樁,
   物件用 `__new__` 造、只塞受測方法會碰到的欄位),照 tests/test_miss_exit.py
   與 tests/test_tot_standby.py 的既有模式。複製一份邏輯來測只會證明複製品
   是對的。
"""
import csv as _csv_mod
import importlib
import math
import sys
import time as _time_mod
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PATCHES = REPO / "scripts" / "wsl" / "sar_swarm_patches"
sys.path.insert(0, str(PATCHES))


def _install_ros_stubs():
    """WSL 沒有 ROS(rclpy/std_msgs 都 import 不到),補最小樁。

    ⚠️ 只在真的缺的時候補 —— 有 ROS 環境時要用真的那份,免得樁把行為蓋掉。
    """
    try:                                    # pragma: no cover - 有 ROS 時走這裡
        import rclpy  # noqa: F401
        import std_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
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
mavutil = tot_node.mavutil

DO_REPOSITION = mavutil.mavlink.MAV_CMD_DO_REPOSITION
DO_CHANGE_SPEED = mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED

#: ⚠️ 在**匯入時**抓一份真的 time.time。其他測試檔(test_tot_standby /
#:   test_miss_exit)是直接把 `tot_node.time.time` 指派掉而且不還原 ——
#:   那是**整個 time 模組**的屬性,會外洩到後續測試。本檔一律用 fixture
#:   存檔/還原,收工時把它放回原樣,不依賴別人有沒有清乾淨。
_REAL_TIME_TIME = _time_mod.time


# ═══════════════════════════════════════════════════════════════════════
#  假鏈路:把每一發 MAVLink 指令錄下來(本檔所有斷言的唯一資料來源)
# ═══════════════════════════════════════════════════════════════════════

class Cmd:
    """錄到的一發指令。param3 就是我們要盯的那個量(DO_REPOSITION 的繞圈半徑)。"""

    def __init__(self, kind, sysid, cmd, params):
        self.kind = kind
        self.sysid = sysid
        self.cmd = cmd
        self.params = params

    @property
    def param3(self):
        return self.params[2]

    def __repr__(self):                       # pragma: no cover - 只在失敗訊息用
        name = {DO_REPOSITION: "DO_REPOSITION",
                DO_CHANGE_SPEED: "DO_CHANGE_SPEED"}.get(self.cmd, str(self.cmd))
        return f"<{name} uav{self.sysid} param3={self.param3!r}>"


class FakeMav:
    """pymavlink `conn.mav` 的替身。只收 tot_node 真的會呼叫的那兩發。"""

    def __init__(self, sysid, bus):
        self._sysid = sysid
        self._bus = bus

    def command_int_send(self, target_system, target_component, frame, command,
                         current, autocontinue, p1, p2, p3, p4, x, y, z):
        # ⚠️ 逐位置解包 —— 呼叫端若改成關鍵字或改變參數順序,這裡會 TypeError
        #    而不是靜默錄到錯的欄位。param3 的位置是本檔的命脈。
        self._bus.append(Cmd("int", self._sysid, command,
                             (p1, p2, p3, p4, x, y, z)))

    def command_long_send(self, target_system, target_component, command,
                          confirmation, p1, p2, p3, p4, p5, p6, p7):
        self._bus.append(Cmd("long", self._sysid, command,
                             (p1, p2, p3, p4, p5, p6, p7)))

    def request_data_stream_send(self, *a, **k):      # 只有 __init__ 會用
        pass


class FakeLink:
    def __init__(self, sysid, bus):
        self.target_system = sysid
        self.mav = FakeMav(sysid, bus)

    def wait_heartbeat(self, timeout=None):           # 只有 __init__ 會用
        return None


class Bus(list):
    """錄音匯流排 + 「自上次標記以來的新指令」查詢。

    ⚠️ 幾乎所有斷言都要問「**這一拍**送出了什麼」,而不是「整場送出了什麼」——
       兩者混用會讓「補發」與「每拍重送」看起來一樣。
    """

    def mark(self):
        return len(self)

    def since(self, mark):
        return list(self[mark:])

    def repos(self, mark=0, sysid=None):
        return [c for c in self[mark:]
                if c.cmd == DO_REPOSITION and (sysid is None or c.sysid == sysid)]

    def radii(self, mark=0, sysid=None):
        return [c.param3 for c in self.repos(mark, sysid)]


class FakeLogger:
    def __init__(self):
        self.lines = []

    def _add(self, m):
        self.lines.append(str(m))

    info = warn = warning = error = debug = _add

    def text(self):
        return "\n".join(self.lines)


class _Sink:
    def __init__(self):
        self.calls = []

    def __call__(self, *a, **k):
        self.calls.append((a, k))

    flush = publish = write = writerow = close = lambda self, *a, **k: None


class Clock:
    """可控時鐘。`_goto` 的 keepalive/min_dt 全部走 time.time(),必須能控。"""

    def __init__(self, t0=1.79e9):
        self.t = float(t0)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


@pytest.fixture
def clock():
    """把 tot_node 用到的 time.time 換成可控時鐘,收工時還原成**真的**那個。

    ⚠️ 刻意不用 monkeypatch:monkeypatch 會還原成「進本 fixture 之前的值」,
       而那個值可能是別的測試檔留下來的 lambda。這裡直接放回匯入時抓到的
       真 `time.time`,不依賴別人有沒有清乾淨。
    """
    clk = Clock()
    tot_node.time.time = clk
    try:
        yield clk
    finally:
        tot_node.time.time = _REAL_TIME_TIME


# ═══════════════════════════════════════════════════════════════════════
#  受測節點:__new__ 造殼,只塞 DIVE 等待段真的會碰到的欄位
# ═══════════════════════════════════════════════════════════════════════

TGT = (25.0777, 119.94315)          # 海上紅心(台灣海峽)

GOTO_R = 30.0                       # 「飛過去」半徑(RRD 生效值)
HOLD_R = 1300.0                     # 「守在點上」半徑(RRD 生效值)
RING_R = 6083.0                     # pushover 環半徑
PERCH = 3100.0
KEEPALIVE = 30.0                    # `_goto` 的補發週期(簽章預設)


def wait_node(bus, switch=False, n=2, goto_radius=GOTO_R, hold_radius=HOLD_R):
    """一個停在 DIVE 段、**全員都還沒放行**的 TotNode 殼。

    ⚠️ 幾何刻意選成「runin 與 wait 的航點逐位元相同」——那正是 RRD 的形狀:
       perch_step=0 ⇒ ring_of/ring_point_of 全隊共用;
       hold_alt=0   ⇒ hold_alt_of(i) 就是 perch_of(i)。
       於是兩段之間**只有 radius 不同**,變更偵測器看不到它。
    """
    s = TotNode.__new__(TotNode)
    s.log = FakeLogger()
    s.get_logger = lambda: s.log
    s.n = n
    s.T = TGT
    s.phase = "DIVE"
    s.standby_mode = False
    s.strike_seq = 1
    # ── 扇形/環幾何(_ring_points 需要)──
    s.r_fan = 4000.0
    s.half = 25.0
    s.approach = 180.0
    s.pushover_r = RING_R
    s.overshoot = 300.0
    s.perch_reach = 260.0
    s.perch_alt = PERCH
    s.perch_step = 0.0
    s.dive_angle = 27.0
    s.dive_alt = 5.0
    s.dive_alt_agl = 5.0
    s.dive_layer = 4.0
    s.layer = 30.0
    s.ground_alt = 0.0
    s.talt = PERCH
    s.target_amsl = float("nan")
    s._ring_points()
    # ── 等待段的組態 ──
    s.hold_alt_cfg = 0.0            # ⇒ hold_alt_of(i) == perch_of(i)
    s.hold_v_cfg = 0.0              # ⇒ hold_v_of() == vmin
    s.vmin, s.vmax = 95.0, 150.0
    s.goto_radius = goto_radius
    s.hold_radius = hold_radius
    s.dive_radius = 5.0
    s.wait_goto_radius = switch     # ← 受測開關
    s.ring_pace = False
    s.release_at_ring = False
    s.weave_time_sync = False
    s.weave_theta_max = 0.0
    s.weave_deadband_s = 0.0
    s.weave_period = 30.0
    s.dive_ring_gate = True
    s.ring_gate_frac = 1.10
    s.ring_replan = True            # 已重排過 ⇒ 不走首架抵環那條
    s.dive_angle_max = 0.0          # 幾何守門員關閉
    s.pn_on = False
    s.fc = {}
    s.gate_profile = False
    s.gate_r = 1200.0
    s.gate_h = 900.0
    s.switch_r = 300.0
    s.impact_reach = 25.0
    s.impact_settle = 4.0
    s.expend = True
    s.dive_speed = 210.0
    s.line_tol, s.resend_dt = 25.0, 2.0
    s.stagger_depart = False
    s.stagger_lead = 0.0
    # ── ALIGN 段(B5 要用它:那裡的 _goto 呼叫點**不帶 radius**)──
    s.reach = 300.0
    s.plan_frac = 0.9
    s.align_tol = 200.0
    s.perch_gate = 0.9
    s.gate_hold_n = 3
    s._gate_hits = {}
    # ── 狀態 ──
    now = tot_node.time.time()
    s.t0 = now - 600.0
    s.T_arr = now - 300.0
    s.dive_go = now - 60.0
    s.dive_T = now + 1.0e6          # 時刻表遠在未來 ⇒ 不會被時刻表放行
    s.done_t = None
    s.released, s.strike, s.frozen, s.alpha = {}, {}, {}, {}
    s.commit, s.thru_dyn = {}, {}
    s.impact, s.missed, s.miss_why, s.miss_pt, s.slow_t0 = {}, {}, {}, {}, {}
    s.dmin, s.hit_alt, s.hit_d = {}, {}, {}
    s.truth, s.truth_dmin, s.truth_hit = {}, {}, {}
    s.zg, s.zg0, s.zg_warn, s.zg_amsl = {}, {}, {}, {}
    s.terr_bull, s.terr_home = {}, {}
    s.e_t, s.tgo_obs, s.weave_theta, s.tgo_cache, s.d_hist = {}, {}, {}, {}, {}
    s.itcg_e, s.itcg_a, s.itcg_s, s.itcg_side = {}, {}, {}, {}
    s.sigma_gate_hit, s.sigma_gate_starved, s.sigma_wait_t0 = {}, {}, {}
    s.slot_id = {i: i - 1 for i in range(1, n + 1)}
    s.slot_of = {i: s.slots[i - 1] for i in range(1, n + 1)}
    s.arr, s.align_ok, s.ready, s.perch_arr = {}, {}, {}, {}
    s.gs = {i: 140.0 for i in range(1, n + 1)}
    s.vel = {i: (140.0, 0.0, 0.0) for i in range(1, n + 1)}
    s.att = {i: (0.0, 0.0, 0.0) for i in range(1, n + 1)}
    s.home_amsl = {i: 30.0 for i in range(1, n + 1)}
    s.home_ll = {i: (25.0777, 121.2325) for i in range(1, n + 1)}
    s.pos = {}
    # ── 航點鎖存(受測對象)──
    s.wp_pt, s.wp_t, s.wp_key = {}, {}, {}
    s.req_radius, s.sent_radius = {}, {}
    s.last_v = {i: 0.0 for i in range(1, n + 1)}
    s.last_v_t = {i: 0.0 for i in range(1, n + 1)}
    # ── 假鏈路(唯一的觀測窗)──
    s.link = {i: FakeLink(i, bus) for i in range(1, n + 1)}
    # ── 與半徑無關、但 _tick 會經過的東西一律換成樁 ──
    s._pump = lambda: None
    s._row = lambda *a, **k: None
    s._track_dmin = lambda i, d, now: None
    s._miss_check = lambda i, d, now: False
    s._sigma_gate_ok = lambda i: True
    s._tgo_of = lambda i, d: 1.0
    s._line_alt = lambda i, d_aim: 0.0
    s._pn_step = lambda i, now: None
    s._predict_tick = lambda i, now: None
    s._proximity_check = lambda now: None
    s._observe_timing = lambda i, now: None
    s._csv = _Sink()
    s._cw = _Sink()
    s.status_pub = _Sink()
    s.gate_pub = _Sink()
    place(s, 12000.0)
    return s


def place(s, d):
    """把每一架擺在**自己那條環徑向**上、距紅心 d 公尺、高度 perch。

    ⚠️ 擺在徑向上是刻意的:`ring_point_of(i)` 不隨飛機位置改變,所以
       runin 與 wait 兩發 `_goto` 的 lat/lon 逐位元相同 —— 這是 RRD 的形狀。
    """
    for i in sorted(s.link):
        k = s.slot_id[i]
        brg = (s.approach + s._fan_off(k)) % 360.0
        lat, lon = tot_node.dest(s.T[0], s.T[1], brg, d)
        s.pos[i] = (lat, lon, PERCH)


#: runin/wait 的分界:`far = d > ring_of(i) * (ring_gate_frac + 0.4)`
FAR_EDGE = RING_R * (1.10 + 0.4)                     # = 9124.5 m
D_RUNIN = FAR_EDGE + 400.0                           # 明確在 runin 側
D_WAIT = FAR_EDGE - 1100.0                           # 明確在 wait 側,且尚未抵環


def modes(s):
    """回傳這一批位置下每一架的段別(runin / wait),用來確認前提真的成立。"""
    out = {}
    for i in sorted(s.pos):
        d = tot_node.dist_m(s.pos[i][:2], s.T)
        out[i] = "runin" if d > s.ring_of(i) * (s.ring_gate_frac + 0.4) else "wait"
    return out


def check_wire_matches_instrument(s, bus, mark):
    """B4 的不變量,抽成可以在**每一拍之後**呼叫的形式。

    送到鏈路上的 param3 必須同時等於 `req_radius[i]` 與 `sent_radius[i]`。
    ⇒ 殺 D1(送出分支裡重綁 `_req`)與 M12(送出的與記錄的脫鉤)。
    """
    for c in bus.repos(mark):
        i = c.sysid
        assert c.param3 == pytest.approx(s.req_radius[i]), (
            f"UAV{i} 送到鏈路上的 param3={c.param3} 與**本拍要求**的 "
            f"req_radius={s.req_radius[i]} 不符 —— CSV 會蓋章認證一個"
            f"從來不存在的對照組")
        assert c.param3 == pytest.approx(s.sent_radius[i]), (
            f"UAV{i} 送到鏈路上的 param3={c.param3} 與記錄的 "
            f"sent_radius={s.sent_radius[i]} 不符")


# ═══════════════════════════════════════════════════════════════════════
#  B1 / B2 —— 開關**關閉**:刻意鎖住 RRD 的舊行為(對照組必須逐位元不變)
# ═══════════════════════════════════════════════════════════════════════

class TestSwitchOffKeepsTheOldBehaviour:
    """⚠️⚠️ 本節鎖的是 **bug 本身**,不是它的修復。

    RRD 的對照組(開關關閉)必須逐位元維持舊行為 —— 否則「開關打開有沒有用」
    這個問題就沒有對照可比。刻意**不**去修 `_goto` 的偵測器(把 radius 併進
    wp_pt):那會讓開關關閉時 16/16 全部變成起彎組,純風險零收益。
    有人「順手把它修好」時,本節會紅,而且訊息會說明為什麼那不是好事。
    """

    @pytest.mark.parametrize("dt", [1.0, 3.0])
    def test_b1_pure_radius_change_is_swallowed(self, clock, dt):
        """B1:runin→wait 的純半徑變更,DO_REPOSITION **根本沒送出去**。

        防的逃脫:N1(偵測器上游 pop wp_pt)、E1(半徑條件加在送出閘門)、
        M6/M6b(把 radius 併進 wp_pt / 旁路字典)。這些改法讓 `moved` 的字面
        一個字都沒動,源碼層的白名單全綠,但**行為**已經翻面。
        逃掉的後果:對照組被毀 —— 開關關閉時 16/16 全變硬彎組,
        「兩個母體」的證據基礎整個消失。

        ⚠️ dt 取兩個值:1.0 s 是交辦指定的形狀;3.0 s **大於** min_dt=2.0 ——
           少了它,`min_dt` 的節流會替真正的受測對象(變更偵測器)擋掉一切,
           上面那三種變異就全部逃掉。
        """
        bus = Bus()
        s = wait_node(bus, switch=False)
        place(s, D_RUNIN)
        assert set(modes(s).values()) == {"runin"}, "前提:第一發必須是 runin"
        s._tick()
        first = bus.repos()
        assert len(first) == s.n, "runin 那一發本來就該送出"
        assert bus.radii() == [GOTO_R] * s.n, "runin 要的是『飛過去』半徑"

        place(s, D_WAIT)
        assert set(modes(s).values()) == {"wait"}, "前提:第二發必須是 wait"
        m = bus.mark()
        clock.advance(dt)
        s._tick()

        assert bus.repos(m) == [], (
            "runin→wait 的純半徑變更竟然送出了 DO_REPOSITION。"
            "這**不是**修好了 —— 這是把 RRD 的對照組毀掉:開關關閉時全部"
            "都會變成硬彎組,兩個母體的比較基礎就沒了。"
            "要修的是致動點(wait_goto_radius),不是 _goto 的偵測器。")
        for i in sorted(s.pos):
            assert s.req_radius[i] == pytest.approx(HOLD_R), (
                f"UAV{i} 本拍**要求**的半徑應該是守環半徑 —— "
                f"req_radius 是每拍都寫的判定量,它必須忠實反映『要什麼』")
            assert s.sent_radius[i] == pytest.approx(GOTO_R), (
                f"UAV{i} 最後真正**送出去**的仍然是 runin 的 {GOTO_R} —— "
                f"這就是飛機掛著舊 param3 直飛的那個狀態")

    def test_b2_the_divergence_ends_at_the_next_keepalive(self, clock):
        """B2:兩欄背離的長度 = 指令被吃掉的時間;keepalive 一到就補發。

        這就是「補發造成起彎」的機制。鎖住它,RRD 的兩個母體才可重現
        (實測:用 keepalive 相位預測「會不會起彎」命中 15/16)。
        防的逃脫:D1(送出分支重綁 `_req`)—— 補發時會送 30 而不是 1300;
        也順帶鎖住任何把 keepalive 拿掉的改動。
        """
        bus = Bus()
        s = wait_node(bus, switch=False)
        place(s, D_RUNIN)
        s._tick()
        t_sent = clock()
        place(s, D_WAIT)
        m0 = bus.mark()
        clock.advance(3.0)
        s._tick()
        assert bus.repos(m0) == [], "前提:wait 那一發被吃掉了"

        m = bus.mark()
        clock.advance(t_sent + KEEPALIVE + 1.5 - clock())   # dt = 31.5 s
        s._tick()
        late = bus.repos(m)
        assert len(late) == s.n, (
            f"過了 keepalive={KEEPALIVE}s 還沒補發 —— 那 hold_radius 就"
            f"永遠不會生效,RRD 的『硬彎組』根本不會存在")
        for c in late:
            assert c.param3 == pytest.approx(HOLD_R), (
                f"補發送出的 param3={c.param3},應該是 {HOLD_R}。"
                f"送 {GOTO_R} 表示有人在送出分支裡把 _req 重新綁掉了(D1),"
                f"那會讓 CSV 蓋章認證一個從來不存在的對照組")
        for i in sorted(s.pos):
            assert s.sent_radius[i] == pytest.approx(HOLD_R)
            assert s.req_radius[i] == pytest.approx(HOLD_R), "補發後兩欄不再背離"
        check_wire_matches_instrument(s, bus, m)


# ═══════════════════════════════════════════════════════════════════════
#  B3 —— 開關**打開**:S2 生效的**行為**定義
# ═══════════════════════════════════════════════════════════════════════

class TestSwitchOnAsksOneRadiusThroughout:
    def test_b3_wait_asks_the_same_radius_as_runin(self, clock):
        """⭐ B3:開關打開 ⇒ wait 與 runin 要求**同一個**半徑,兩欄不背離。

        ⚠️⚠️ **成功的指紋是「指令的缺席」**:wait 那一拍不會送出任何
        DO_REPOSITION —— 在 log 裡找確認指令是找不到的,那是對的,不是失敗。
        所以這一條的判定量是 `req_radius`(每拍都寫),不是 `sent_radius`
        (hold-last-value,開關開與關**同樣**是 30 ⇒ 假綠燈)。

        防的逃脫:M3/M3b(致動點的三元式被改掉)、N2(hold_radius 的值來源
        被改成 goto_radius ⇒ 等同開關永遠開著,連對照組都沒了)。
        逃掉的後果:整批飛行的自變數失效,而報表看起來完全正常。
        """
        bus = Bus()
        s = wait_node(bus, switch=True)
        place(s, D_RUNIN)
        s._tick()
        assert bus.radii() == [GOTO_R] * s.n

        place(s, D_WAIT)
        assert set(modes(s).values()) == {"wait"}
        m = bus.mark()
        clock.advance(3.0)
        s._tick()

        assert bus.repos(m) == [], (
            "開關打開時 wait 段竟然送出了新的 DO_REPOSITION —— "
            "S2 的成功指紋就是**指令的缺席**(要求的半徑沒變,偵測器也沒動),"
            "有指令送出表示有人動了節流或偵測器")
        for i in sorted(s.pos):
            assert s.req_radius[i] == pytest.approx(GOTO_R), (
                f"UAV{i} 開關打開時 wait 段仍應要求『飛過去』半徑 {GOTO_R}")
            assert s.sent_radius[i] == pytest.approx(GOTO_R)
            assert s.req_radius[i] == s.sent_radius[i], "兩欄不得背離"

    def test_b3_hold_radius_is_never_put_on_the_wire(self, clock):
        """⭐ B3(續):整段(含 keepalive 補發)**沒有任何** param3 == hold_radius。

        ⚠️ 只看 wait 那一拍不夠 —— RRD 的硬彎正是**補發**造成的。
           要證明 S2 生效,必須連 30 s 之後的補發也還是 goto_radius。
        """
        bus = Bus()
        s = wait_node(bus, switch=True)
        place(s, D_RUNIN)
        s._tick()
        place(s, D_WAIT)
        for _ in range(8):
            clock.advance(5.0)
            s._tick()

        radii = bus.radii()
        assert radii, "整段一發 DO_REPOSITION 都沒有,測試前提壞了"
        assert all(r == pytest.approx(GOTO_R) for r in radii), (
            f"開關打開時鏈路上出現了非 {GOTO_R} 的半徑:{sorted(set(radii))}。"
            f"param3=={HOLD_R} 就是『飛機起彎』的那一發指令")
        assert len(bus.repos()) > s.n, "前提:這一段確實經過了至少一次補發"


# ═══════════════════════════════════════════════════════════════════════
#  B4 —— 鏈路上的 param3 永遠等於當下的 req_radius
# ═══════════════════════════════════════════════════════════════════════

class TestWireEqualsInstrument:
    @pytest.mark.parametrize("switch", [False, True])
    def test_b4_every_reposition_matches_the_instrument(self, clock, switch):
        """⭐ B4:每一發 DO_REPOSITION 的 param3 == 當下的 req_radius == sent_radius。

        防的逃脫:D1(在送出分支裡把 `_req` 重新綁成 `self.goto_radius` ——
        兩個字面都還是 `_req`,源碼層比的是名字不是值)、M12(`_reposition`
        送 `self.goto_radius` 而 `sent_radius` 仍記 `_req`)。
        逃掉的後果最惡劣:**CSV 寫 req=1300、飛機實收 30**,而
        `s2_readout.py` 的整個結論就建在 req_radius_m 這一欄上
        ⇒ 判讀腳本會主動蓋章認證一個從來不存在的對照組。
        """
        bus = Bus()
        s = wait_node(bus, switch=switch)
        script = [(D_RUNIN, 0.0), (D_RUNIN, 2.5), (D_WAIT, 3.0),
                  (D_WAIT, 29.0), (D_WAIT, 0.5), (D_WAIT, 31.0)]
        seen = 0
        for d, dt in script:
            place(s, d)
            clock.advance(dt)
            m = bus.mark()
            s._tick()
            check_wire_matches_instrument(s, bus, m)
            seen += len(bus.repos(m))
        assert seen >= 2 * s.n, "前提:這段腳本至少要真的送出兩輪指令"


# ═══════════════════════════════════════════════════════════════════════
#  B5 —— param3 永遠 > 0
# ═══════════════════════════════════════════════════════════════════════

class TestParam3IsAlwaysPositive:
    """⚠️⚠️ ArduPlane 的閘門是 GCS_MAVLink_Plane.cpp `param3 > 0`;
    0 **不通過** ⇒ mode_guided → update_loiter(0) ⇒ 退回 |WP_LOITER_RAD|
    再 ×eas2tas²(WP_LOITER_RAD=1488、3100 m 的 eas2tas²=1.365 ⇒ **實際 2030 m**)
    ⇒ 半徑改由飛控自己決定,而節點完全不知道。
    這是本專案「ALIGN 整段繞在 2030 m 的圈上、抵達判定永遠不成立」的舊坑。
    """

    @pytest.mark.parametrize("switch", [False, True])
    def test_b5_no_reposition_ever_carries_a_non_positive_radius(self, clock,
                                                                 switch):
        bus = Bus()
        s = wait_node(bus, switch=switch)
        for d, dt in [(D_RUNIN, 0.0), (D_WAIT, 3.0), (D_WAIT, 31.0),
                      (D_WAIT, 31.0)]:
            place(s, d)
            clock.advance(dt)
            s._tick()
        radii = bus.radii()
        assert radii, "前提:整段確實送過 DO_REPOSITION"
        for r in radii:
            assert r > 0.0, (
                f"送出了 param3={r} ≤ 0 —— ArduPlane 會整發退回 "
                f"|WP_LOITER_RAD|×eas2tas²(3100 m 實際 2030 m),"
                f"半徑變成飛控自己決定,而且零告警")

    def test_b5_align_hold_path_also_carries_a_positive_radius(self, clock):
        """B5(續):**不帶 radius 的真實呼叫點**也要送正的 param3。

        ⚠️ 上一條 parametrize 只走 DIVE 等待段,而那裡每一發都明寫 radius=,
           所以它對 N8(簽章預設 None → 0.0)完全免疫。ALIGN 的
           `self._goto(i, *self.slot_of[i], self._alt(i), self.vmin)` 才是
           N8 真正的受害者 —— 而它正是「整段繞在 2030 m 圈上、抵達判定永遠
           不成立」那個舊坑的發生地。
        """
        bus = Bus()
        s = wait_node(bus, switch=False)
        s.phase = "ALIGN"
        for i in sorted(s.link):                     # 擺到自己的 slot 上 ⇒ hold 分支
            s.pos[i] = (*s.slot_of[i], PERCH)
        s._tick()
        radii = bus.radii()
        assert len(radii) == s.n, "ALIGN 的 hold 分支應該對每一架各送一發"
        for r in radii:
            assert r > 0.0, (
                f"ALIGN 送出 param3={r} ≤ 0 —— 飛控會退回 "
                f"|WP_LOITER_RAD|×eas2tas²(3100 m 實際 2030 m),"
                f"`d < reach` 的抵達判定就永遠不成立(ALIGN 卡死)")
            assert r == pytest.approx(GOTO_R), (
                "ALIGN 是『飛過去』語意,必須用 goto_radius")

    def test_b5_goto_without_an_explicit_radius_asks_goto_radius(self, clock):
        """⭐ B5(續):不帶 `radius=` 的 `_goto` 必須要求 `goto_radius`,不是 0。

        防的逃脫:N8(`_goto` 簽章預設 `radius=None` → `0.0` ⇒
        `radius is None` 永遠為假 ⇒ **所有沒帶 radius 的呼叫點**一律送
        param3=0)。ALIGN/PERCH/climb/runup 全部是不帶 radius 的呼叫點,
        所以 N8 一發就把整個對齊段打回 2030 m 繞圈的舊坑。
        ⚠️ 源碼層的鎖只驗 `_req` 那個三元運算式,**不驗簽章的預設值**。
        """
        bus = Bus()
        s = wait_node(bus, switch=False)
        lat, lon = s.slot_of[1]
        m = bus.mark()
        s._goto(1, lat, lon, 1234.0, s.vmin)          # ← 刻意不給 radius
        sent = bus.repos(m, sysid=1)
        assert len(sent) == 1, "第一發一定要送得出去(wp_pt 還是空的)"
        assert sent[0].param3 == pytest.approx(GOTO_R), (
            f"不帶 radius 的 _goto 送出 param3={sent[0].param3};"
            f"預設必須是 goto_radius={GOTO_R}。0 會被飛控整發退回")
        assert s.req_radius[1] == pytest.approx(GOTO_R)
        assert s.sent_radius[1] == pytest.approx(GOTO_R)


class TestHoldRadiusSurvivesWhereOrbitingIsWanted:
    """⚠️ S2 是「wait 段不要繞圈」,**不是**「把 hold_radius 廢掉」。

    真的要繞的地方還有兩處:ALIGN 的 hold_ring、以及已判 MISS 的機原地盤旋。
    那兩處若一併塌成 goto_radius=30,飛機根本飛不出那個圈(物理迴轉半徑遠大於
    30 m),而後果是**靜默的**:DO_REPOSITION 照樣 ACK,只是圈的大小由飛控
    自己決定。這一條把「該繞的地方仍然繞」鎖在**鏈路上的 param3**。
    """

    def test_missed_aircraft_still_gets_the_orbiting_radius(self, clock):
        bus = Bus()
        s = wait_node(bus, switch=True)          # ⭐ S2 開著也不准影響這裡
        place(s, D_WAIT)
        now = clock()
        s.missed = {1: now}
        s.miss_why = {1: "測試"}
        s.miss_pt = {1: (*s.slot_of[1], PERCH)}
        s._tick()
        assert bus.radii(sysid=1) == [pytest.approx(HOLD_R)], (
            f"已判 MISS 的機要**原地盤旋**,param3 必須是守環半徑 {HOLD_R};"
            f"實得 {bus.radii(sysid=1)}。塌成 {GOTO_R} 的話飛機飛不出那個圈,"
            f"而圈的大小改由飛控自己決定(零告警)")
        assert s.req_radius[1] == pytest.approx(HOLD_R)
        assert bus.radii(sysid=2) == [pytest.approx(GOTO_R)], \
            "沒被判 MISS 的機仍走 S2 的等待路徑"


# ═══════════════════════════════════════════════════════════════════════
#  B6 —— 儀表必須綁對機號
# ═══════════════════════════════════════════════════════════════════════

class TestInstrumentIsBoundToTheRightAircraft:
    """⚠️⚠️ N4(`.get(i)` → `.get(i + 1)`)的可怕之處:表頭 29 欄不動、
    `_rr` 綁的仍是 `self.req_radius`、欄位對應全部正確,而 req_radius_m
    記的是**隔壁那架**的要求半徑 —— 而 s2_readout 的整個結論就建在這一欄上。
    源碼層的 CSV 契約鎖(表頭、繫結、順序)一條都碰不到它。
    """

    def test_b6_goto_writes_each_aircraft_into_its_own_slot(self, clock):
        bus = Bus()
        s = wait_node(bus, switch=False)
        lat1, lon1 = s.slot_of[1]
        lat2, lon2 = s.slot_of[2]
        s._goto(1, lat1, lon1, 1000.0, s.vmin, radius=GOTO_R)
        s._goto(2, lat2, lon2, 1000.0, s.vmin, radius=HOLD_R)

        assert s.req_radius[1] == pytest.approx(GOTO_R)
        assert s.req_radius[2] == pytest.approx(HOLD_R)
        assert s.sent_radius[1] == pytest.approx(GOTO_R)
        assert s.sent_radius[2] == pytest.approx(HOLD_R)
        assert bus.radii(sysid=1) == [pytest.approx(GOTO_R)]
        assert bus.radii(sysid=2) == [pytest.approx(HOLD_R)]

    def test_b6_csv_row_carries_the_aircrafts_own_radius(self, clock, tmp_path):
        """⭐ B6(續):CSV 的 `req_radius_m` / `cmd_radius_m` 必須是**這一列
        那架自己**的值。用**真的** `_open_csv()` 開檔、真的 `_row()` 寫列、
        再用 DictReader 讀回來 —— 表頭與欄位對應都不是本檔自己編的。
        """
        bus = Bus()
        s = wait_node(bus, switch=False)
        s.req_radius = {1: GOTO_R, 2: HOLD_R}
        s.sent_radius = {1: GOTO_R, 2: HOLD_R}
        s.wp_pt = {1: (*s.slot_of[1], 1000.0), 2: (*s.slot_of[2], 1000.0)}
        s.pred = {}
        s._agl = lambda i: 100.0
        s._kill_arm_update = lambda: None
        s._kill_airframe = lambda i, agl, now: None
        s._lead_angle_deg = lambda i: 0.0
        s._agl_datum_logged = True
        s.last_v = {1: 100.0, 2: 100.0}
        del s._row                                   # ← 用真的那份
        s.csv_path = str(tmp_path / "tot_b6")
        s._csv = None
        s._open_csv()
        now = clock()
        for i in (1, 2):
            s._row(now, i, 4321.0, "wait", s.T)
        s._csv.close()

        with open(s._csv_base() + ".csv", newline="", encoding="utf-8") as fh:
            rows = {int(r["sysid"]): r for r in _csv_mod.DictReader(fh)}
        assert set(rows) == {1, 2}
        assert float(rows[1]["req_radius_m"]) == pytest.approx(GOTO_R), (
            "UAV1 那一列的 req_radius_m 不是 UAV1 自己的要求半徑 —— "
            "判定量錯機,s2_readout 的整個結論就會建在隔壁那架的資料上")
        assert float(rows[2]["req_radius_m"]) == pytest.approx(HOLD_R)
        assert float(rows[1]["cmd_radius_m"]) == pytest.approx(GOTO_R)
        assert float(rows[2]["cmd_radius_m"]) == pytest.approx(HOLD_R)


# ═══════════════════════════════════════════════════════════════════════
#  B7 —— 沒有半徑變更時不會每拍重送
# ═══════════════════════════════════════════════════════════════════════

class TestNoResendWithoutChange:
    @pytest.mark.parametrize("switch", [False, True])
    def test_b7_steady_state_sends_exactly_once(self, clock, switch):
        """⭐ B7:同一個航點連續跑 21 拍(共 2 s)只送出一發 DO_REPOSITION。

        防的逃脫:N7(在**致動點那一發** `_goto` 加 `keepalive=0.0` ⇒
        `dt > keepalive` 恆真 ⇒ 每拍重送 ⇒ 半徑變更立刻生效,對照組被毀)。
        源碼層的鎖驗的是 `moved` 的字面與致動點的三元式,**不驗呼叫端塞了
        什麼節流參數**。

        ⚠️⚠️ 這一條同時保護 memory 記錄過的「20 Hz 重錨高度」災難:
           每一發 DO_REPOSITION 都會走 `Plane::set_guided_WP()` →
           `set_target_altitude_current()`,把高度目標**重錨在飛機當下高度**;
           而修正走 10 Hz 的導航迴圈 ⇒ 重錨比修正還快,高度需求永遠累積不
           起來(實測:整場 ALIGN/PERCH 六機卡在 30-80 m,指令 400 m,而
           alt_error ≈ 0 —— 飛控認為「已經到高度了」)。
        ⚠️ 只數 DO_REPOSITION:DO_CHANGE_SPEED 本來就該每 2 s 送一次
           (它不碰高度斜坡),把兩者混在一起數會讓這條鎖無法解讀。
        """
        bus = Bus()
        s = wait_node(bus, switch=switch)
        place(s, D_WAIT)
        assert set(modes(s).values()) == {"wait"}, "前提:整段都在 wait"
        s._tick()
        assert len(bus.repos()) == s.n, "第一拍該送(wp_pt 還是空的)"

        m = bus.mark()
        for _ in range(20):
            clock.advance(0.1)
            s._tick()
        assert bus.repos(m) == [], (
            f"航點與半徑都沒變,卻在 2 s 內重送了 "
            f"{len(bus.repos(m))} 發 DO_REPOSITION。"
            f"每一發都會 set_target_altitude_current() 把高度目標重錨在當下"
            f"高度 ⇒ 重錨比修正還快,高度需求永遠累積不起來(20 Hz 重錨災難)")

    def test_b7_speed_is_allowed_to_keep_updating(self, clock):
        """⚠️ 反向鎖:B7 不可以被「乾脆什麼都不送」滿足。

        DO_CHANGE_SPEED **應該**繼續按既有節流送(它不碰高度斜坡)。
        少了這一條,「把 _goto 整個停掉」也會讓 B7 變綠。
        """
        bus = Bus()
        s = wait_node(bus, switch=False)
        place(s, D_WAIT)
        s._tick()
        m = bus.mark()
        for _ in range(20):
            clock.advance(0.5)      # 共 10 s ⇒ 速度節流(2 s)必然開火數次
            s._tick()
        speeds = [c for c in bus.since(m) if c.cmd == DO_CHANGE_SPEED]
        assert len(speeds) >= 2 * s.n, (
            "DO_CHANGE_SPEED 停掉了 —— 等待段的速度指令是 hold_v_of() 的"
            "唯一出口,停掉它等於整個等待段沒有速度權限")


# ═══════════════════════════════════════════════════════════════════════
#  B8 / B9 —— 值真的來自參數(需要**真的建構**一個節點)
# ═══════════════════════════════════════════════════════════════════════

class ProbeNode(TotNode):
    """真的跑一次 `TotNode.__init__`,只把「不屬於受測範圍」的外界換成樁。

    ⚠️ 為什麼非得真的建構:N2(`self.hold_radius` 改讀 `goto_radius` 參數)
       與 N3(`setattr` 旁路)都**不改任何被鎖的那一行**,只改「那個量最後
       是多少」。用 `__new__` 造殼、自己塞 hold_radius 的測試對它們一律免疫。

    ⚠️ 換掉的只有 ROS 參數/發布/計時器、MAVLink 連線、CSV 路徑 —— 參數的
       **讀取與指派**那一段完全是真的那份程式碼在跑。
    """

    def __init__(self, overrides, bus):
        self._probe_vals = {}
        self._probe_over = dict(overrides)
        self._probe_bus = bus
        self.probe_log = FakeLogger()
        self.probe_timers = []
        TotNode.__init__(self)

    # ── ROS 參數:宣告即記錄,覆寫值優先 ──
    def declare_parameter(self, name, default):
        val = self._probe_over.get(name, default)
        self._probe_vals[name] = val
        return types.SimpleNamespace(value=val)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._probe_vals[name])

    def get_logger(self):
        return self.probe_log

    def create_publisher(self, *a, **k):
        return _Sink()

    def create_subscription(self, *a, **k):
        return _Sink()

    def create_timer(self, period, cb):
        self.probe_timers.append((period, cb))
        return _Sink()


@pytest.fixture
def build_node(monkeypatch, tmp_path):
    """建一個真的 TotNode(待命模式:tlat/tlon = NaN ⇒ 不下令、不問地形)。"""
    def _build(**overrides):
        bus = Bus()
        counter = {"i": 0}

        def _fake_conn(url, source_system=None, dialect=None):
            counter["i"] += 1
            return FakeLink(counter["i"], bus)

        monkeypatch.setattr(tot_node.mavutil, "mavlink_connection", _fake_conn)
        base = {"n": 2, "csv": str(tmp_path / "probe")}
        base.update(overrides)
        return ProbeNode(base, bus)
    return _build


class TestValuesReallyComeFromParameters:
    def test_b8_hold_and_goto_radius_are_two_different_values(self, build_node):
        """B8:節點讀到的 hold_radius 與 goto_radius 真的不相等。

        防的逃脫:N2(`self.hold_radius = float(p("hold_radius").value)` →
        `float(p("goto_radius").value)`)。源碼層的三元式鎖、IfExp 鎖、
        呼叫點計數全部綠,而後果是 **wait 段關閉路徑也要求 30 ⇒ 等同開關
        永遠開著**,對照組直接消失;順帶 ALIGN 的守環與 missed 的守點半徑
        一併塌成 30 m(飛不出那個圈)。
        """
        s = build_node(goto_radius=GOTO_R, hold_radius=HOLD_R)
        assert s.goto_radius == pytest.approx(GOTO_R)
        assert s.hold_radius == pytest.approx(HOLD_R), (
            "hold_radius 沒有讀到 hold_radius 參數 —— 若它讀的是 goto_radius,"
            "wait 段的關閉路徑也會要求 30,等同開關永遠開著,對照組消失")
        assert s.hold_radius != s.goto_radius

    def test_b8_the_two_stay_distinct_on_declared_defaults(self, build_node):
        """B8(續):連**宣告預設值**都必須是兩個不同的數。

        ⚠️ 只測覆寫值不夠:N2 型的改法在覆寫相同時仍可能被抓不到。
           預設值 goto_radius=30 / hold_radius=400 本來就不同,
           它們變成同一個數就是「兩個語意被接到同一個來源」的指紋。
        """
        s = build_node()
        assert s.hold_radius != s.goto_radius, (
            "宣告預設下 hold_radius 與 goto_radius 竟然相同 —— "
            "『飛過去』與『守在點上』是語意完全相反的兩個量")

    @pytest.mark.parametrize("want", [False, True])
    def test_b9_the_switch_follows_its_parameter(self, build_node, want):
        """⭐ B9:`wait_goto_radius` 的值真的來自它自己的參數,且不被別處覆寫。

        防的逃脫:
          N3 —— 字面 `self.wait_goto_radius = bool(p("wait_goto_radius").value)`
               原封不動保留,下一行加 `setattr(self, "wait_goto_radius", True)`。
               `setattr` 不是 `ast.Assign` ⇒ 源碼層的「恰好一處指派」鎖全綠。
          M11 —— 讀錯參數名(例如讀 `release_at_ring`)。
        逃掉的後果:開關被寫死打開,對照組污染的方向是「命中率變好」——
        最難察覺的那一種。
        """
        s = build_node(wait_goto_radius=want)
        assert s.wait_goto_radius is want, (
            f"參數給 {want},節點卻是 {s.wait_goto_radius} —— "
            f"開關的值沒有跟著參數走(被別處覆寫,或讀錯參數名)")

    def test_b9_the_switch_does_not_shadow_a_neighbouring_flag(self, build_node):
        """B9(續):把**鄰居**開關翻成 True,不可以連帶把 S2 翻開。

        ⚠️ 這一條抓的是「讀錯參數名」的另一半:只測自己那顆參數時,
           `p("release_at_ring")` 這種改法在兩顆同時為 False/True 時看不出來。
        """
        s = build_node(wait_goto_radius=False, release_at_ring=True,
                       ring_pace=True, dive_from_runup=True)
        assert s.wait_goto_radius is False, (
            "鄰居開關打開就把 wait_goto_radius 帶開了 —— 讀錯參數名")
        assert s.release_at_ring is True
        assert s.ring_pace is True

    def test_b9_a_real_node_flies_the_switch_it_was_given(self, build_node,
                                                          clock):
        """⭐⭐ B9(續):**從參數一路走到鏈路**的端到端鎖。

        用真的建構出來的節點(參數 → 屬性)去跑真的 DIVE 等待段,
        斷言鏈路上的 param3。這一條是 B3 與 B8/B9 的接縫:
        中間任何一環(參數名、屬性、致動點三元式、`_req`、`_reposition`)
        被改掉,這裡都會紅,而且它**完全不看任何一行原始碼長什麼樣**。
        """
        results = {}
        for want in (False, True):
            s = build_node(wait_goto_radius=want, goto_radius=GOTO_R,
                           hold_radius=HOLD_R, perch_step=0.0, hold_alt=0.0,
                           hold_v=0.0, dive_ring_gate=True, ring_pace=False,
                           release_at_ring=False, dive_angle_max=0.0,
                           weave_time_sync=False)
            bus = s._probe_bus
            # 把節點搬進 DIVE 段的等待狀態(幾何與旗標都用它自己讀到的值)
            s.T = TGT
            s.pushover_r = RING_R
            s.perch_alt = PERCH
            s._ring_points()
            s.phase = "DIVE"
            s.ring_replan = True
            s.pn_on = False
            s.fc = {}
            s.dive_T = clock() + 1.0e6
            s.dive_go = clock() - 60.0
            s.t0 = clock() - 600.0
            s.T_arr = clock() - 300.0
            s.slot_id = {1: 0, 2: 1}
            s.slot_of = {1: s.slots[0], 2: s.slots[1]}
            s.gs = {1: 140.0, 2: 140.0}
            s.vel = {1: (140.0, 0.0, 0.0), 2: (140.0, 0.0, 0.0)}
            s.link = {1: FakeLink(1, bus), 2: FakeLink(2, bus)}
            s._pump = lambda: None
            s._row = lambda *a, **k: None
            s._track_dmin = lambda i, d, now: None
            s._miss_check = lambda i, d, now: False
            s._tgo_of = lambda i, d: 1.0
            s._line_alt = lambda i, d_aim: 0.0
            s._csv = _Sink()
            s.status_pub = _Sink()
            place(s, D_RUNIN)
            s._tick()
            place(s, D_WAIT)
            clock.advance(3.0)
            s._tick()
            results[want] = ({i: s.req_radius[i] for i in (1, 2)},
                             list(bus.radii()))

        req_off, wire_off = results[False]
        req_on, wire_on = results[True]
        assert all(v == pytest.approx(HOLD_R) for v in req_off.values()), (
            f"開關關閉時 wait 段要求的半徑應為 {HOLD_R},實得 {req_off}")
        assert all(v == pytest.approx(GOTO_R) for v in req_on.values()), (
            f"開關打開時 wait 段要求的半徑應為 {GOTO_R},實得 {req_on}")
        assert req_off != req_on, (
            "開與關要求的半徑一模一樣 ⇒ 這顆開關對**行為**沒有任何影響,"
            "整批飛行的自變數不存在")
        # 兩邊在 wait 那一拍都不該送出新指令(成功指紋 = 指令的缺席)
        assert wire_off == [pytest.approx(GOTO_R)] * 2
        assert wire_on == [pytest.approx(GOTO_R)] * 2


# ═══════════════════════════════════════════════════════════════════════
#  測試自身的健全性 —— 前提壞掉時要紅,不要靜靜地什麼都沒測到
# ═══════════════════════════════════════════════════════════════════════

class TestHarnessSanity:
    """⚠️ 本檔所有結論都建在「假鏈路真的錄到了東西」與「runin/wait 的航點
    逐位元相同」這兩個前提上。前提壞掉時要**大聲**,否則整個檔案會變成一堆
    空跑的綠燈 —— 那正是這一輪要擺脫的東西。
    """

    def test_runin_and_wait_command_the_very_same_waypoint(self, clock):
        """RRD 形狀的前提:兩段的 lat/lon/alt **逐位元相同**,只有 radius 不同。"""
        bus = Bus()
        s = wait_node(bus, switch=False)
        for i in sorted(s.pos):
            assert s.ring_point_of(i) == s.pp[s.slot_id[i]]
            assert s.hold_alt_of(i) == s.perch_of(i), (
                "hold_alt=0 時等待高度必須就是 perch 高度 —— "
                "兩段的 alt 不同的話,moved 會因為高度差而成立,"
                "本檔測的就不是『純半徑變更』了")

    def test_the_fake_link_actually_records(self, clock):
        bus = Bus()
        s = wait_node(bus, switch=False)
        s._tick()
        assert bus, "假鏈路一發都沒錄到 —— 所有斷言都會空跑"
        assert any(c.cmd == DO_REPOSITION for c in bus)

    def test_far_and_wait_distances_are_on_the_right_sides(self):
        assert D_RUNIN > FAR_EDGE > D_WAIT
        assert D_WAIT > RING_R * 1.10, (
            "wait 的取樣距離不可以小於放行閘門,否則飛機會被放行而離開等待段,"
            "測到的就不是 wait 分支")
