"""tot_node 的 miss 出口(M1/M2)、結算量凍結(M3)與摘要行向後相容。

背景 —— SW1(2026-08-04)的逃逸鏈,已由前一輪對抗式徹查驗證:
  `impact` 這個集合**沒有 miss/timeout 出口**。U2/U6 首航次從紅心旁 638/469 m
  掠過(> switch_r=300)⇒ commit 不成立 ⇒ passed 恆 False ⇒ 永遠不鎖存 ⇒
  俯衝指令繼續發,而 v_req = d/max(t_rem, 0.05) 在 t_rem 深度為負時飽和滿速
  ⇒ U2 飛到 207 km、U6 412 km 之外,數小時後隨機重穿紅心才鎖存。
  摘要行的「命中高度 {2: -16, 6: 2}」與散布 11950 s 都是這樣來的污染值。

這裡測的是**真的那份 tot_node.py**(用 sys.modules 補 rclpy/std_msgs 的樁,
物件用 __new__ 造、只塞受測方法會碰到的欄位),不是邏輯的複製品 ——
複製一份來測只會證明複製品是對的。
"""
import importlib
import math
import os
import subprocess
import sys
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
            pass

    class _Policy:
        RELIABLE = BEST_EFFORT = 1
        KEEP_LAST = KEEP_ALL = 1
        TRANSIENT_LOCAL = VOLATILE = 1

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
EARTH = tot_node.EARTH


# ═══════════════════════════════════════════════════════════════════════
#  受測物件:__new__ 造殼,只塞受測方法真的會讀到的欄位
# ═══════════════════════════════════════════════════════════════════════

class FakeLogger:
    def __init__(self):
        self.lines = []

    def _add(self, m):
        self.lines.append(str(m))

    info = warn = warning = error = debug = _add

    def text(self):
        return "\n".join(self.lines)


TGT = (25.0777, 119.94315)          # SW1 的紅心(台灣海峽海面)


def at(bearing_deg, dist_m_):
    """由紅心沿方位角推出一個座標,距離 dist_m_ 公尺。"""
    b = math.radians(bearing_deg)
    return (TGT[0] + dist_m_ * math.cos(b) / EARTH,
            TGT[1] + dist_m_ * math.sin(b) / (EARTH * math.cos(math.radians(TGT[0]))))


def node(n=6, now=1.79e9):
    """一個處於 DIVE 段的最小 TotNode。now 預設用真實量級的 epoch。"""
    s = TotNode.__new__(TotNode)
    s.n = n
    s.T = TGT
    s.log = FakeLogger()
    s.get_logger = lambda: s.log
    # 相位/時刻
    s.t0 = now - 900.0
    s.T_arr = now - 600.0
    s.phase = "DIVE"
    s.dive_go = now
    s.dive_T = now + 300.0          # ⚠️ 絕對牆鐘時戳,不是時長
    s.done_t = None
    # 幾何/門檻
    s.impact_reach = 40.0
    s.impact_alt_ok = 30.0
    s.impact_settle = 4.0
    s.switch_r = 300.0
    s.overshoot = 800.0
    s.dive_speed = 140.0
    s.dive_angle = 25.0
    s.ground_alt = -32.0
    s.zg = {}
    s.zg_amsl = {}
    # ⚠️ 每機**絕對**高度(AMSL),與 relative_alt 取自同一則 GLOBAL_POSITION_INT。
    #    釘死 AMSL 的目標一律用它算 agl(tot_node._agl),免疫飛控重設 HOME ——
    #    2026-08-07 因為兩者基準不同步,誤殺了一架 150 m/s 巡航中的飛機。
    #    樁節點預設空 dict ⇒ 沒設的測試自動退回舊式,行為不變。
    s.alt_amsl = {}
    # ⚠️ `_agl` 的第一個判斷式就讀 target_amsl;2026-08-10 起 `hit_alt` 也走 `_agl`,
    #    所以連 DIVE 段的樁都需要它。nan = 沒釘死 AMSL ⇒ 退回 `alt − z_g` 的舊式,
    #    這一檔既有測試的語義(agl = −30 − (−32) = 2 m)因此一個位元都沒變。
    s.target_amsl = float("nan")
    # miss 出口的生效值
    s.miss_far = 20000.0
    s.miss_far_k = 2.0
    s.miss_slow_gs = 5.0
    s.miss_slow_sec = 60.0
    s.miss_window = 60.0
    # ⚠️ 最近水平距(dmin)**沒有**收斂窗欄位 —— 鎖存當下就凍。
    #    2026-08-04 移除 dmin_hyst / dmin_hold / dmin_closed(見 TestMetricFreeze)。
    # 狀態
    s.pos = {i: (*at(180.0, 1000.0), 400.0) for i in range(1, n + 1)}
    s.gs = {i: 120.0 for i in range(1, n + 1)}
    # ⚠️ 預設**全部已放行** —— 逃逸規則只對已放行的機生效(見 _miss_check 的
    #    (a) 規則)。SW1 的 U2/U6 也是放行之後才飛走的,所以這是對的預設。
    #    「還在環上等放行」的情境由 TestRingCoupling 專門測。
    s.released = {i: True for i in range(1, n + 1)}
    s.impact, s.missed, s.miss_why, s.miss_pt, s.slow_t0 = {}, {}, {}, {}, {}
    s.dmin, s.hit_alt, s.hit_d, s.commit, s.thru_dyn = {}, {}, {}, {}, {}
    s.truth, s.truth_dmin, s.truth_hit = {}, {}, {}
    return s


# ═══════════════════════════════════════════════════════════════════════
#  M1 —— 首航次窗:epoch 條件的方向
# ═══════════════════════════════════════════════════════════════════════

class TestWindowEpoch:
    def test_wrong_form_is_dead_code(self):
        """反證:`now - dive_go > dive_T + window` 恆假 —— 這是要避開的死碼。

        dive_T 是 epoch(~1.79e9),now - dive_go 是時長(SW1 整趟 12007 s),
        拿時長去比 epoch 永遠不成立,而且完全靜默:窗看起來裝好了,實際沒裝。
        """
        dive_go = 1.79e9
        dive_T = dive_go + 300.0
        window = 60.0
        for late in (0.0, 1.0, 3600.0, 12007.0, 86400.0):
            now = dive_T + window + late
            wrong = (now - dive_go) > (dive_T + window)
            assert wrong is False, f"死碼寫法在 late={late} 竟然成立了"

    def test_right_form_fires_exactly_at_window_close(self):
        right_before = 1.79e9 + 300.0 + 60.0 - 0.1
        right_after = 1.79e9 + 300.0 + 60.0 + 0.1
        end = 1.79e9 + 300.0 + 60.0
        assert (right_before > end) is False
        assert (right_after > end) is True

    def test_miss_check_uses_epoch_form(self):
        """行為證據:窗關前不判、窗關後判。用的是真的 _miss_check。"""
        s = node()
        end = s.dive_T + s.miss_window
        assert s._miss_check(1, 900.0, end - 1.0) is False
        assert 1 not in s.missed
        assert s._miss_check(1, 900.0, end + 1.0) is True
        assert 1 in s.missed
        assert "逾時" in s.miss_why[1]

    def test_window_is_measured_from_dive_T_not_from_dive_go(self):
        """dive_T 在首架抵環時會被重排 —— 窗終點必須跟著移,不是釘在 dive_go。"""
        s = node()
        s.dive_T = s.dive_go + 5000.0          # 重排成很晚
        assert s._miss_check(1, 900.0, s.dive_go + 400.0) is False
        assert s._miss_check(1, 900.0, s.dive_T + s.miss_window + 1.0) is True

    def test_dive_T_none_does_not_explode(self):
        """dive_T 初始為 None。⚠️ 這裡若 TypeError,整段結算會被炸掉。"""
        s = node()
        s.dive_T = None
        assert s._miss_check(1, 900.0, s.dive_go + 99999.0) is False
        assert s.missed == {}

    def test_unknown_aircraft_is_ignored(self):
        s = node()
        assert s._miss_check(99, 412000.0, s.dive_go + 5000.0) is False

    def test_no_check_before_dive_go(self):
        """dive_go 之前蜂群還在 35 km 外的扇形上 —— 提前檢查會全軍誤判。"""
        s = node()
        s.dive_go = None
        for i in range(1, 7):
            assert s._miss_check(i, 35000.0, 1.79e9) is False
        assert s.missed == {}


# ═══════════════════════════════════════════════════════════════════════
#  預設值 vs SW1 實測:窗要涵蓋正常機,又要在逃逸機飛到天邊之前收線
# ═══════════════════════════════════════════════════════════════════════

# 全部來自 SW1(2026-08-04),以 dive_go 為 0:
#   trial_SW1.log 「首架抵環 → 重排共同命中時刻:T+83.9s」
#   trial_SW1.log 「UAV{1,4,5,3} 命中紅心 @+{52.20,62.65,66.20,71.15}s」
#   tot_SW1.csv 重算(148 萬列):正常四架 dive_go 後離目標最遠 4.1/4.2/4.7/5.3 km;
#                U6 首次 >20 km 在 +152.2 s、U2 在 +247.7 s
SW1_DIVE_T_REL = 83.9
SW1_NORMAL_IMPACTS = [52.20, 62.65, 66.20, 71.15]
SW1_NORMAL_MAX_KM = [4.1, 4.2, 4.7, 5.3]
SW1_ESCAPE_FIRST_FAR = {6: 152.2, 2: 247.7}
DEFAULT_WINDOW = 60.0
DEFAULT_FAR_KM = 20.0


class TestDefaultsAgainstSW1:
    def test_window_clears_the_slowest_normal_impact(self):
        """窗終點 +143.9 s vs 最慢的正常命中 +71.2 s ⇒ 餘裕 72.7 s(> 1 倍)。"""
        end = SW1_DIVE_T_REL + DEFAULT_WINDOW
        assert end - max(SW1_NORMAL_IMPACTS) > 60.0

    def test_far_threshold_sits_between_normal_and_escaped(self):
        """正常機最遠 5.3 km、逃逸機 207/412 km —— 門檻放 20 km 有 3.8 倍餘裕。"""
        assert max(SW1_NORMAL_MAX_KM) * 3.5 < DEFAULT_FAR_KM
        assert DEFAULT_FAR_KM < 207.0

    def test_far_threshold_clears_the_ring_at_dive_go(self):
        """SW1 的 dive_go 當下最遠一架距 T 4697 m(trial_SW1.log 的 DIVE 配速行)
        ⇒ 20 km / 4697 m = 4.3 倍餘裕。比值掉到 2 倍以下,還在環上合法等待的機
        就會被自己判成逃逸,而且是全隊同時 —— tot_node 會在 dive_go 印 error。"""
        assert DEFAULT_FAR_KM * 1000.0 / 4697.0 > 2.0

    def test_window_is_the_binding_exit_on_sw1(self):
        """實測歸因:SW1 的兩架是被**窗**收掉的(+143.9 s),不是被 20 km 規則
        (+152.2 / +247.7 s)。20 km 是備援,不是主出口 —— 別把因果講反。"""
        end = SW1_DIVE_T_REL + DEFAULT_WINDOW
        assert all(end < t for t in SW1_ESCAPE_FIRST_FAR.values())

    def test_exit_cuts_the_run_by_two_orders_of_magnitude(self):
        """SW1 實際跑到 +12002.9 s 才鎖存最後一架;出口把它壓到 +143.9 s。"""
        assert (SW1_DIVE_T_REL + DEFAULT_WINDOW) < 12002.85 / 50.0


# ═══════════════════════════════════════════════════════════════════════
#  M2 —— 逃逸 / 停滯
# ═══════════════════════════════════════════════════════════════════════

class TestMissTriggers:
    @pytest.mark.parametrize("d_km,expect", [
        (4.1, False), (5.3, False),        # SW1 正常四架 dive_go 後的最遠距離
        (19.9, False),
        (20.1, True), (207.0, True), (412.0, True),   # SW1 逃逸的 U2/U6
    ])
    def test_far_threshold(self, d_km, expect):
        s = node()
        assert s._miss_check(1, d_km * 1000.0, s.dive_go + 10.0) is expect

    def test_slow_needs_to_be_sustained(self):
        s = node()
        s.gs[1] = 1.0
        t = s.dive_go
        for _ in range(60):                 # 60 s 還不夠(門檻是 > 60)
            assert s._miss_check(1, 900.0, t) is False
            t += 1.0
        assert s._miss_check(1, 900.0, t + 1.0) is True
        assert "停滯" in s.miss_why[1]

    def test_slow_timer_resets_on_recovery(self):
        """地速回升就把計時清掉,不可以把不連續的秒數累加起來。"""
        s = node()
        t = s.dive_go
        for _ in range(10):
            s.gs[1] = 1.0
            for _ in range(30):
                s._miss_check(1, 900.0, t)
                t += 1.0
            s.gs[1] = 120.0                 # 恢復
            s._miss_check(1, 900.0, t)
            t += 1.0
        assert 1 not in s.missed
        assert 1 not in s.slow_t0

    def test_missing_gs_sample_does_not_count_as_slow(self):
        """沒有 gs 樣本 ≠ 停住。⚠️ 時間要留在窗內,否則測到的是逾時不是停滯。"""
        s = node()
        s.gs.pop(1)
        for k in range(0, 200):
            assert s._miss_check(1, 900.0, s.dive_go + k) is False
        assert s.slow_t0 == {}

    def test_latch_records_loiter_point_and_stops_dive(self):
        s = node()
        s.pos[1] = (*at(0.0, 412000.0), 3000.0)
        assert s._miss_check(1, 412000.0, s.dive_go + 10.0) is True
        assert s.miss_pt[1][:2] == s.pos[1][:2]
        assert abs(s.miss_pt[1][2]) > 0.5    # 0.0 會被 Location::sanitize() 判為未設定
        assert "MISS" in s.log.text()


# ═══════════════════════════════════════════════════════════════════════
#  M3 —— 凍結時刻由 SW1 實測撐著。⚠️⚠️ 這張表 2026-08-04 **翻案**了。
#
#  重放 tot_node 自己的鎖存規則(impact_reach=25 —— trial_SW1.log 的生效值、
#  switch_r=300、passed = commit 且 d > dmin+8)於 tot_SW1.csv 的 DIVE 段,
#  保持檔案順序(重放腳本見交接;逐架數字如下):
#      UAV  鎖存t   鎖存d   凍在鎖存   遲滯窗    首次觸海t   stot_score 的 d1
#       1   992.0   43.79    35.22     35.2       993.70       35.14
#       3  1011.0   41.98    32.62     22.6      1012.60       32.61
#       4  1002.5   40.27    29.87     29.9      1004.48       29.87
#       5  1006.0  226.16   217.90     67.1      1008.19      217.90
#
#  上一輪把「遲滯窗」那一欄當成正解,理由是它「還原了工具的 d1」。方向寫反了:
#  當時的工具**自己**也在報第二趟。真相是 ——
#    * 遲滯收窗發生在鎖存後 31.7–45.5 s,而機體在鎖存後 1.9–2.6 s 就觸海了。
#    * SITL 不終結觸海的機體:U3 掉到海面下 95.68 m、爬回 +331 m,24.1 s 後
#      再穿一次紅心(22.58 m);U5 同型(17.9 s 後 67.11 m)。
#    * 於是遲滯窗收進來的是**第二趟**,而且剛好把 U3 從 MISS_MAX=30 的失敗側
#      搬到通過側(32.6 → 22.6)。那是把指標改好看,不是改準。
#  現場證據站在「凍在鎖存」這一邊:trial_SW1.log 的逐架鎖存行印的是
#  「最近 35m / 30m / 218m / 33m」,與這一欄逐架吻合;而同一份 log 幾小時後
#  印出來的摘要行(22.6 / 67.1)才是被污染的那個。
# ═══════════════════════════════════════════════════════════════════════

SW1_DMIN_REPLAY = {
    #    鎖存d, 凍在鎖存(真值), 遲滯窗(第二趟), 鎖存t,   首次觸海t, 工具 d1
    1: (43.79,  35.22,  35.2,   992.0,  993.70,  35.14),
    3: (41.98,  32.62,  22.6,  1011.0, 1012.60,  32.61),
    4: (40.27,  29.87,  29.9,  1002.5, 1004.48,  29.87),
    5: (226.16, 217.90, 67.1,  1006.0, 1008.19, 217.90),
}


class TestFreezePolicyAgainstSW1:
    @pytest.mark.parametrize("i", sorted(SW1_DMIN_REPLAY))
    def test_freezing_at_latch_reproduces_the_tool(self, i):
        """⭐ 凍在鎖存 ⇒ 與 stot_score 的首航次 d1 一致(誤差 ≤ 一個取樣步)。

        兩支工具**必須**算同一個量。差值來自取樣量化:節點只能取到取樣點,
        工具做點到線段內插,所以節點的值是上界。
        """
        _d, frozen, _hyst, _t, _tt, d1 = SW1_DMIN_REPLAY[i]
        assert frozen >= d1                      # 節點端是上界
        assert frozen - d1 < 9.0, "差超過一個取樣步(實測步長 3–9 m)"

    @pytest.mark.parametrize("i", [3, 5])
    def test_the_hysteresis_window_reported_the_second_pass(self, i):
        """反證:遲滯窗給的那兩個數字是機體**入水後爬起來**才做出來的。"""
        _d, frozen, hyst, t_latch, t_touch, _d1 = SW1_DMIN_REPLAY[i]
        assert hyst < frozen, "遲滯窗給了一個比首航次更小的值"
        # 遲滯收窗要 31.7–45.5 s;而觸海只要 1.6–2.6 s ⇒ 窗收的一定在水下之後
        assert t_touch - t_latch < 3.0

    def test_the_polluted_value_flips_the_kpi_verdict(self):
        """U3:真值 32.62 m > _kpi_parse 的 MISS_MAX 預設 30 ⇒ 判失敗;
        第二趟的 22.6 m ⇒ 判通過。門檻兩側,不是小數點問題 ——
        所以「用哪一個」不是風格選擇,它直接決定驗收結論。"""
        _d, frozen, hyst, *_r = SW1_DMIN_REPLAY[3]
        assert frozen > 30.0 >= hyst

    def test_latch_distance_is_always_an_upper_bound_of_dmin(self):
        """鎖存是 d < impact_reach 觸發的,dmin 是鎖存前跑出來的最小值。"""
        for i, (d_latch, frozen, *_r) in SW1_DMIN_REPLAY.items():
            assert d_latch >= frozen, i

    @pytest.mark.parametrize("i", sorted(SW1_DMIN_REPLAY))
    def test_touchdown_lands_inside_the_old_hysteresis_window(self, i):
        """⚠️ 這條是「為什麼不能只把遲滯調小」的量化證據:觸海發生在鎖存後
        2 秒內,而任何以「d 回升多少」為條件的窗都關不到那麼準 ——
        U3 放行時的外衝就有 923 m,遲滯調到 555 m 以下會在出發點收窗。
        """
        _d, _f, _h, t_latch, t_touch, _d1 = SW1_DMIN_REPLAY[i]
        assert 0.0 < t_touch - t_latch <= 2.6


# ═══════════════════════════════════════════════════════════════════════
#  R1 blocker —— 逃逸門檻與 pushover 環半徑的耦合
#
#  ring = (perch_alt − dive_alt) / tan(dive_angle),而進 DIVE 的前提只是
#  「全員 d < ring_of(i)」⇒ 環半徑可以任意大。環半徑 ≥ miss_far_m 時,
#  DIVE 的第一個 tick 會把**全隊六架同時**判成逃逸、整趟 0/6 結案,
#  在總表上跟「飛行失敗」一模一樣,而真因是門檻組態。
#  SW1 沒事只是因為 dive_angle=45° ⇒ 環 6437 m ⇒ 比值 3.1,是運氣不是防護。
# ═══════════════════════════════════════════════════════════════════════

def ring_r(perch_alt, dive_alt, dive_angle_deg):
    return (perch_alt - dive_alt) / math.tan(math.radians(dive_angle_deg))


class TestRingCoupling:
    @pytest.mark.parametrize("perch,angle,km", [
        (3000.0, 45.0, 3.0),        # SW1 那類組態:遠小於 20 km
        (3000.0, 25.0, 6.4),        # tot_node 註解自己算的
        (3000.0, 5.0, 34.3),        # ⇒ 環半徑 > 預設 miss_far_m
        (6000.0, 15.0, 22.4),       # perch 拉高一樣會超過
    ])
    def test_ring_radius_can_exceed_the_default_threshold(self, perch, angle,
                                                          km):
        assert ring_r(perch, 0.0, angle) / 1000.0 == pytest.approx(km, abs=0.1)

    def test_waiting_on_the_ring_is_never_an_escape(self):
        """⭐ 還沒放行的機停在 22 km 的環上 —— 不可以被判逃逸,一架都不行。"""
        s = node()
        s.released = {i: False for i in s.pos}
        for i in sorted(s.pos):
            assert s._miss_check(i, 22400.0, s.dive_go + 0.05) is False
        assert s.missed == {}, "全隊在 DIVE 第一個 tick 就被自己判成逃逸"

    def test_released_aircraft_beyond_threshold_is_still_caught(self):
        """放行之後 d 只會變小 —— 還跑到門檻外就是真的飛走了,照抓。"""
        s = node()
        assert s._miss_check(2, 207000.0, s.dive_go + 10.0) is True
        assert "逃逸" in s.miss_why[2]

    def test_unreleased_still_reachable_by_the_other_two_exits(self):
        """⚠️ 放行閘門只擋 (a) 逃逸,不可以把 (b) 停滯 / (c) 逾時也一起關掉,
        否則卡在環上不動的機永遠不計入 done,航次不會結案。"""
        s = node()
        s.released = {i: False for i in s.pos}
        assert s._miss_check(1, 900.0, s.dive_T + s.miss_window + 1.0) is True
        assert "逾時" in s.miss_why[1]
        s2 = node()
        s2.released = {i: False for i in s2.pos}
        s2.gs[3] = 0.5
        t = s2.dive_go
        for _ in range(62):
            s2._miss_check(3, 900.0, t)
            t += 1.0
        assert 3 in s2.missed and "停滯" in s2.miss_why[3]

    def test_clamp_widens_a_too_tight_threshold(self):
        """只印 error 擋不住(下一個 tick 就動手)—— 這裡要真的夾。"""
        s = node()
        r_ring = ring_r(6000.0, 0.0, 15.0)          # 22.4 km > 20 km
        old, new, clamped = s._clamp_miss_far(r_ring, r_ring)
        assert clamped is True
        assert old == 20000.0
        assert new == pytest.approx(2.0 * r_ring)
        assert s.miss_far == pytest.approx(2.0 * r_ring)
        # 夾完之後,環上的機(即使被誤標成已放行)也不會被判逃逸
        s.released = {i: True for i in s.pos}
        assert s._miss_check(1, r_ring, s.dive_go + 0.05) is False

    def test_clamp_is_a_floor_not_an_override(self):
        """SW1 的 20000/4697 = 4.3 倍,不需要夾 —— 夾子不准把門檻改小。"""
        s = node()
        old, new, clamped = s._clamp_miss_far(4697.0, 4697.0)
        assert clamped is False
        assert (old, new) == (20000.0, 20000.0)
        assert s.miss_far == 20000.0

    def test_clamp_uses_the_larger_of_ring_and_current_distance(self):
        s = node()
        s._clamp_miss_far(1000.0, 30000.0)          # 當下最遠比環大
        assert s.miss_far == pytest.approx(60000.0)

    def test_clamp_can_be_disabled_with_k_zero(self):
        """k=0 ⇒ 只留「已放行才檢查」那一道,門檻完全照組態值。"""
        s = node()
        s.miss_far_k = 0.0
        _old, _new, clamped = s._clamp_miss_far(50000.0, 50000.0)
        assert clamped is False
        assert s.miss_far == 20000.0

    def test_clamp_runs_before_the_first_dive_tick(self):
        """⚠️⚠️ 夾子必須在**任何一次 _miss_check 之前**執行,否則第一個 tick
        就已經誤判完了,夾晚一步等於沒夾。

        結構保證:`self.dive_go = now` 與夾子都在 PERCH 分支裡,而 DIVE 是
        `elif` ⇒ 同一個 tick 不可能兩個都跑 ⇒ 夾子必定早於 DIVE 的第一個 tick。
        這裡把那個順序釘住,免得有人把夾子搬到 DIVE 分支裡而完全沒有測試會紅。
        """
        src = (PATCHES / "tot_node.py").read_text(encoding="utf-8").splitlines()

        def line_of(needle):
            for k, ln in enumerate(src):
                if needle in ln:
                    return k
            raise AssertionError(f"找不到 {needle}")

        go = line_of('self.dive_go = now')
        clamp = line_of('self._clamp_miss_far(')
        dive = line_of('elif self.phase == "DIVE":')
        assert go < clamp < dive, (go, clamp, dive)

    def test_clamp_line_carries_the_effective_value_marker(self):
        """⚠️ make_artifacts.sh 的「## 生效參數」是 `grep -E '生效值|抵達判定'`。
        夾住這件事若不含「生效值」三個字就進不了 summary.md —— 上一版的
        error log 正是這樣,吵了等於沒吵。"""
        src = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
        assert "miss_far_m 生效值已自動夾寬" in src
        mk = (REPO / "scripts" / "wsl" / "make_artifacts.sh").read_text(
            encoding="utf-8")
        assert "生效值" in mk


# ═══════════════════════════════════════════════════════════════════════
#  M2 —— impact / missed 互斥,以及 done 計數
# ═══════════════════════════════════════════════════════════════════════

class TestMutualExclusion:
    def test_impacted_aircraft_is_never_missed(self):
        s = node()
        s.impact[1] = s.dive_go + 50.0
        s.dmin[1] = 3.0
        assert s._miss_check(1, 412000.0, s.dive_go + 5000.0) is False
        assert 1 not in s.missed

    def test_landed_aircraft_already_impacted_is_not_re_flagged(self):
        """⚠️⚠️ SW1 實測:正常四架觸海後 gs 掉到 0,停滯規則會在
        +175.2/+197.6/+230.3/+242.9 s(tot_SW1.csv 重算)全部觸發 ——
        而它們早在 +52.2~71.2 s 就進了 impact。互斥鎖是唯一擋住
        「已命中卻又被記成 MISS」的東西,拿掉就會有四架假 MISS。"""
        s = node()
        s.impact[1] = s.dive_go + 52.2
        s.dmin[1] = 35.2
        s.hit_alt[1] = 448.0
        s.gs[1] = 0.0
        for k in range(0, 300):
            assert s._miss_check(1, 50.0, s.dive_go + 52.2 + k) is False
        assert 1 not in s.missed

    def test_missed_aircraft_is_frozen_for_impact_path(self):
        """已 missed ⇒ _metrics_frozen 為真 ⇒ _tick 的命中鎖存被擋掉。"""
        s = node()
        s.missed[1] = s.dive_go + 10.0
        assert s._metrics_frozen(1) is True

    def test_second_miss_check_is_idempotent(self):
        s = node()
        s._miss_check(1, 412000.0, s.dive_go + 10.0)
        first = s.missed[1]
        assert s._miss_check(1, 412000.0, s.dive_go + 20.0) is False
        assert s.missed[1] == first

    def test_done_counts_missed(self):
        """⚠️⚠️ 只把逃逸機排除出 impact 而不計入 done ⇒ 永遠不印摘要、
        永遠不進 DONE、航次不會自己結束 —— 比不修更糟。"""
        s = node()
        for i in (1, 3, 4, 5):
            s.impact[i] = s.dive_go + 40.0 + i
        assert s._done_count() == 4
        s.missed[2] = s.dive_go + 60.0
        s.missed[6] = s.dive_go + 60.0
        assert s._done_count() == s.n == 6

    def test_done_ignores_unknown_aircraft(self):
        s = node()
        s.impact[9] = 0.0                   # 不在 self.pos 裡
        assert s._done_count() == 0


# ═══════════════════════════════════════════════════════════════════════
#  M3 —— dmin / hit_alt 的時刻一致性
# ═══════════════════════════════════════════════════════════════════════

class TestMetricFreeze:
    """量測(dmin / truth_dmin)在 impact / missed **鎖存當下**凍結。

    ⚠️⚠️ 2026-08-04 翻案:上一版加了一個「鎖存後 d 從最小值回升 1000 m 才停止
       收斂」的遲滯窗,理由是「凍在鎖存 ⇒ dmin 恆 ≥ impact_reach 減一個取樣步」。
       前提沒錯,結論錯了 —— 遲滯窗實測要鎖存後 31.7–45.5 s 才收,而 SW1 的
       機體在鎖存後 1.9–2.6 s 就觸海了。SITL 不終結觸海的機體:U3 掉到海面下
       95.68 m、爬回 +331 m,24.1 s 後再穿一次紅心。窗收進來的是**第二趟**,
       不是「還沒走完的最近點」。
    ⚠️ 代價照實承認:凍在鎖存的 dmin 是首航次真最近點的**上界**,誤差最多一個
       取樣步(實測 3–9 m)。要更細的量用 stot_score.py(點到線段內插 +
       AGL≤0 觸地收窗)—— 那支有整條軌跡可以回頭看,節點在現場沒有。
    """

    def test_updates_normally_before_latch(self):
        s = node()
        for d in (900.0, 400.0, 217.7, 90.0):
            s._track_dmin(1, d, s.dive_go + 10.0)
        assert s.dmin[1] == pytest.approx(90.0)

    def test_sw1_uav5_second_pass_is_not_the_first_pass(self):
        """⭐ SW1 的 U5:鎖存當下的 dmin 是 217.9 m,而 67.1 m 是它**觸海之後**
        爬起來再穿一次做出來的(相隔 17.9 s)。

        現場證據:trial_SW1.log 的鎖存行印「UAV5 命中紅心 …(最近 218m…」。
        67.1 只出現在幾小時後才印的摘要行 —— 那時 dmin 已經被洗過。
        """
        s = node()
        t = s.dive_go + 50.0
        s._track_dmin(5, 217.9, t)
        s.impact[5] = t                                   # 鎖存 ⇒ 就地凍結
        s.hit_alt[5] = 678.0
        s.hit_d[5] = 226.2
        for k, d in enumerate((400.0, 900.0, 400.0, 67.1)):   # 觸海、爬起、重穿
            s._track_dmin(5, d, t + 5.0 * (k + 1))
        assert s.dmin[5] == pytest.approx(217.9), "收進了第二趟"
        assert s.hit_d[5] == pytest.approx(226.2), "鎖存當下的距離必須另存"

    def test_latch_freezes_immediately_with_no_grace(self):
        """⚠️ 一個 tick 都不寬限 —— 「再收斂 N 秒」是同一個病的小一號版本。"""
        s = node()
        t = s.dive_go + 50.0
        s._track_dmin(1, 30.0, t)
        s.impact[1] = t
        s._track_dmin(1, 29.99, t + 0.05)
        assert s.dmin[1] == pytest.approx(30.0)

    def test_latch_tick_itself_is_counted(self):
        """⚠️ 順序合約:_track_dmin 在**同一個 tick 的鎖存判定之前**呼叫,
        鎖存那一筆取樣要算進 dmin。挪到鎖存之後 = 少一筆 = dmin 偏大。"""
        s = node()
        t = s.dive_go + 50.0
        s._track_dmin(1, 23.0, t)       # 這一筆觸發鎖存,但先進 dmin
        s.impact[1] = t
        assert s.dmin[1] == pytest.approx(23.0)

    def test_no_convergence_window_knobs_remain(self):
        """⚠️ 旋鈕留著比拿掉更糟:留著就會有人調,而調它會把入水後的第二趟
        收回來 —— 而且是**靜默**的(數字只會變好看)。"""
        s = node()
        for attr in ("dmin_hyst", "dmin_hold", "dmin_closed"):
            assert not hasattr(s, attr), attr
        assert not hasattr(s, "_dmin_frozen"), \
            "死碼述詞回來了(真正在跑的是 _track_dmin,註解卻指向它)"

    def test_reapproach_hours_later_cannot_wash_dmin(self):
        """逃逸機在幾百公里外繞回來重穿紅心 —— 凍結是唯一的防線。"""
        s = node()
        t = s.dive_go + 50.0
        s._track_dmin(6, 469.4, t)
        s.impact[6] = t
        s._track_dmin(6, 2.24, t + 8000.0)
        assert s.dmin[6] == pytest.approx(469.4)

    def test_frozen_after_miss(self):
        """missed **立刻**凍 —— 逃逸機在 412 km 外繞回來重穿紅心是幾小時後的
        事,不准拿它去洗 dmin(SW1 的 U6 會被洗成 2.24 m 的幽靈命中)。"""
        s = node()
        s._track_dmin(2, 638.1, s.dive_go + 50.0)
        s.missed[2] = s.dive_go + 60.0
        s._track_dmin(2, 5.0, s.dive_go + 60.01)
        assert s.dmin[2] == pytest.approx(638.1)

    def test_missed_is_frozen_with_no_grace_at_all(self):
        """⚠️ miss 不吃任何寬限 —— 兩條路徑要分得開。SW1 的 U6 若吃到寬限,
        8876 s 那次 2.24 m 的重穿就會變成幽靈命中。"""
        s = node()
        t = s.dive_go + 60.0
        s._track_dmin(6, 469.4, t - 1.0)
        s.missed[6] = t
        s._track_dmin(6, 2.24, t + 0.05)
        assert s.dmin[6] == pytest.approx(469.4)

    def test_truth_basis_freezes_at_the_same_instant(self):
        """真值基準與遙測基準同一個凍結時刻,兩行才講得出同一個事件。"""
        s = node()
        t = s.dive_go + 10.0
        s.truth[1] = (*at(180.0, 300.0), 500.0)
        s._track_dmin(1, 300.0, t)
        assert s.truth_dmin[1] == pytest.approx(300.0, abs=1.0)
        s.truth[1] = (*at(180.0, 2.0), 5.0)
        s._track_dmin(1, 2.0, t + 1.0)                  # 鎖存前:跟著收
        assert s.truth_dmin[1] == pytest.approx(2.0, abs=1.0)
        s.impact[1] = t + 1.0
        s.truth[1] = (*at(180.0, 0.5), 1.0)
        s._track_dmin(1, 0.5, t + 2.0)                  # 鎖存後:凍
        assert s.truth_dmin[1] == pytest.approx(2.0, abs=1.0)
        assert s.dmin[1] == pytest.approx(2.0)

    def test_passed_behaviour_unchanged_before_latch(self):
        """⚠️⚠️ `passed = commit and d > dmin + 8` 靠 dmin 在鎖存前持續更新。
        凍結若提早生效,通過最近點就永遠偵測不到 = 把逃逸鏈再造一次。"""
        s = node()
        s.commit[1] = True
        t = s.dive_go
        for d in (400.0, 100.0, 30.0):
            s._track_dmin(1, d, t)
            assert s._passed(1, d) is False
            t += 0.05
        s._track_dmin(1, 35.0, t)
        assert s._passed(1, 35.0) is False          # 只回升 5 m,還不到 8 m
        s._track_dmin(1, 39.0, t + 0.05)
        assert s._passed(1, 39.0) is True
        assert s.dmin[1] == pytest.approx(30.0)

    def test_passed_needs_commit(self):
        s = node()
        s._track_dmin(1, 30.0, s.dive_go)
        assert s._passed(1, 500.0) is False
        s.commit[1] = True
        assert s._passed(1, 500.0) is True

    def test_one_predicate_governs_both_judgement_and_measurement(self):
        """⭐ 單一真實來源:判定與量測共用 `_metrics_frozen`。

        ⚠️ 上一版是兩個述詞 + 一份藏在 _track_dmin 裡的複製品,而註解(節點 4
           處、stot_trial.sh 1 處)全部指向那個**沒有人呼叫**的述詞 ——
           一把假的迴歸鎖:改壞真正在跑的那份,測試照樣全綠。
        """
        s = node()
        t = s.dive_go + 50.0
        assert s._metrics_frozen(1) is False
        s.impact[1] = t
        assert s._metrics_frozen(1) is True
        s._track_dmin(1, 1.0, t + 1.0)
        assert s.dmin == {}, "述詞說凍了,量測還在收"

    def test_the_predicate_has_no_side_effects(self):
        """⚠️ 述詞不可以有副作用 —— 有的話「查一下狀態」就會改掉狀態,
        而那種 bug 在這個檔案裡是查不出來的(每個測試都會查狀態)。"""
        s = node()
        s.impact[1] = s.dive_go + 50.0
        for _ in range(3):
            s._metrics_frozen(1)
        assert s.dmin == {}
        assert s.impact == {1: s.dive_go + 50.0}

    def test_no_stale_pointers_to_the_dead_predicate(self):
        """⚠️ 註解把讀者指向死碼 = 下一個人會去改一份不會執行的程式碼。"""
        for rel in ("scripts/wsl/sar_swarm_patches/tot_node.py",
                    "scripts/wsl/stot_trial.sh"):
            src = (REPO / rel).read_text(encoding="utf-8")
            for ln in src.splitlines():
                if "_dmin_frozen" in ln:
                    assert "移除" in ln or "曾經" in ln, \
                        f"{rel} 仍把讀者指向已移除的 _dmin_frozen:{ln.strip()}"

    def test_passed_without_any_dmin_sample_is_false(self):
        s = node()
        s.commit[1] = True
        assert s._passed(1, 1e6) is False           # 不可以 KeyError


# ═══════════════════════════════════════════════════════════════════════
#  摘要行:零 miss 走舊哨兵、有 miss 走新結案行
# ═══════════════════════════════════════════════════════════════════════

SENTINEL = "俯衝命中紅心:全員命中"     # 舊哨兵 = 現在的「達標描述(a)」,只有形狀 (a) 有
CLOSEOUT = "★★ 俯衝結案:"            # 固定結案哨兵,四種形狀**都**有
# ⚠️ 2026-08-05 起哨兵與達標描述分離,CLOSEOUT 不再是「有 miss 才出現」的標記。
#    四種結案形狀的完整矩陣鎖在 tests/test_summary_sentinel.py,這裡只保留
#    既有的向後相容與 miss 出口斷言。


def _all_hit(s, alt=2.0):
    for i in range(1, s.n + 1):
        s.impact[i] = s.dive_go + 40.0 + 0.1 * i
        s.dmin[i] = 1.0 + 0.1 * i
        s.hit_alt[i] = alt


class TestSummary:
    def test_zero_miss_keeps_old_sentinel_verbatim(self):
        s = node()
        _all_hit(s)
        line = s._impact_summary()[0]
        # 固定哨兵在行首、計數緊接其後,達標描述在 ';' 之後
        assert line.startswith("★★ 俯衝結案:6/6 命中、0 MISS;")
        assert CLOSEOUT in line
        # ⚠️ 向後相容:舊字串必須**逐字**還在(既有基準與歷史 log 比對靠它)
        assert SENTINEL in line
        assert "、0 MISS;" in line, "零 miss 卻報了 MISS 數 ⇒ 計數壞了"
        assert "MISS'" not in line and ": MISS" not in line, "零 miss 不該有脫靶明細"
        assert "命中時間散布 = 0.50s" in line

    def test_high_pass_verdict_still_replaces_sentinel(self):
        """既有行為不可退化:高空掠過時本來就不准印「全員命中」。"""
        s = node()
        _all_hit(s, alt=598.0)
        line = s._impact_summary()[0]
        assert SENTINEL not in line
        assert "名義命中" in line

    def test_miss_uses_new_closeout_line_and_never_says_all_hit(self):
        s = node()
        for i in (1, 3, 4, 5):
            s.impact[i] = s.dive_go + 40.0 + 0.1 * i
            s.dmin[i] = 5.0
            s.hit_alt[i] = 2.0
        for i in (2, 6):
            s.missed[i] = s.dive_go + 60.0
            s.miss_why[i] = "逃逸:離目標 412.0 km > miss_far_m 20.0 km"
            s.dmin[i] = 638.1
        line = s._impact_summary()[0]
        assert line.startswith("★★ 俯衝結案:4/6 命中、2 MISS")
        assert SENTINEL not in line, "有脫靶卻印了舊哨兵 ⇒ 舊 grep 會誤判成功"
        assert "2: MISS" in line and "6: MISS" in line
        assert "1: 5.0" in line

    def test_spread_uses_impacts_only(self):
        """⚠️⚠️ missed 的時戳是出口判定時刻,不是命中時刻。混進來就是 SW1
        那個 11950 s。也不准只加一句「太大就警告」—— 擋不住同一行的錯數字。"""
        s = node()
        for i in (1, 3, 4, 5):
            s.impact[i] = s.dive_go + 40.0 + 0.5 * i
            s.dmin[i] = 5.0
            s.hit_alt[i] = 2.0
        for i in (2, 6):
            s.missed[i] = s.dive_go + 11950.0      # 逃逸機幾小時後才結案
            s.miss_why[i] = "逾時"
            s.dmin[i] = 638.1
        line = s._impact_summary()[0]
        assert "命中時間散布 = 2.00s" in line
        assert "11950" not in line

    def test_all_missed_does_not_crash(self):
        s = node()
        for i in range(1, 7):
            s.missed[i] = s.dive_go + 60.0
            s.miss_why[i] = "逾時"
            s.dmin[i] = 900.0
        line = s._impact_summary()[0]
        assert line.startswith("★★ 俯衝結案:0/6 命中、6 MISS")

    def test_high_pass_denominator_is_the_hit_count(self):
        """⚠️ 「N/M 架在高空掠過」的分母是**命中數**,不是全隊 n。

        hit_alt 只有命中的機才有值。拿全隊當分母,一趟「2 架命中而且兩架都
        在高空掠過、4 架 MISS」的航次會印「2/6 架在高空掠過」—— 讀起來像
        「另外 4 架好好落地了」,剛好相反。
        """
        s = node()
        for i in (1, 2):
            s.impact[i] = s.dive_go + 40.0
            s.dmin[i] = 5.0
            s.hit_alt[i] = 900.0                  # 兩架都在高空
        for i in (3, 4, 5, 6):
            s.missed[i] = s.dive_go + 60.0
            s.miss_why[i] = "逾時"
            s.dmin[i] = 900.0
        line = s._impact_summary()[0]
        assert "2/2 架在高空掠過" in line
        assert "2/6 架在高空掠過" not in line

    def test_zero_miss_denominator_is_unchanged(self):
        """零 miss 時 len(hit) == n ⇒ 這個修法不動既有基準的字面輸出。"""
        s = node()
        _all_hit(s, alt=598.0)
        assert "6/6 架在高空掠過" in s._impact_summary()[0]


class TestSW1CounterfactualSummary:
    """⭐ 反事實重放:零 miss 時摘要行印出來的 dmin 必須是**首航次**的值。

    數字來自把 tot_SW1.csv 的 DIVE 段(137 萬列)逐列餵給真的那份
    `_track_dmin` / `_passed` / 鎖存分支(impact_reach=25、switch_r=300):
        UAV1 鎖存 t=992.00 d=43.79 ⇒ dmin 35.22
        UAV3 鎖存 t=1011.00 d=41.98 ⇒ dmin 32.62
        UAV4 鎖存 t=1002.50 d=40.27 ⇒ dmin 29.87
        UAV5 鎖存 t=1006.00 d=226.16 ⇒ dmin 217.90
    這裡把同樣的序列縮成幾筆餵進去,鎖的是**接線**(誰先誰後、凍不凍),
    數值本身由上面的 SW1_DMIN_REPLAY 表撐著。
    """

    def _replay_one(self, s, i, frozen, d_latch, second_pass):
        t = s.dive_go + 50.0
        s.commit[i] = True
        for k, d in enumerate((frozen + 200.0, frozen, d_latch)):
            s._track_dmin(i, d, t + 0.05 * k)
        # 鎖存那一 tick:_track_dmin 已經吃過這一筆 d(順序合約)
        s.impact[i] = t + 0.1
        s.hit_d[i] = d_latch
        s.hit_alt[i] = 2.0
        # 觸海 → 爬起來 → 第二趟(SW1 的 U3/U5 真的做過這件事)
        for k, d in enumerate((3000.0, 1500.0, second_pass)):
            s._track_dmin(i, d, t + 10.0 * (k + 1))

    def test_summary_prints_the_first_pass_not_the_second(self):
        s = node()
        want = {}
        for i, (d_latch, frozen, second, *_r) in SW1_DMIN_REPLAY.items():
            self._replay_one(s, i, frozen, d_latch, second)
            want[i] = round(frozen, 1)
        s.n = 4
        line = s._impact_summary()[0]
        assert "最近水平距 = {1: 35.2, 3: 32.6, 4: 29.9, 5: 217.9} m" in line
        assert {i: round(s.dmin[i], 1) for i in sorted(s.dmin)} == want
        for poison in ("22.6", "67.1"):
            assert f": {poison}" not in line, "第二趟的污染值進了摘要行"

    def test_latch_distances_are_reported_separately(self):
        """鎖存當下的距離另開一欄,兩個數字都要看得到。"""
        s = node()
        for i, (d_latch, frozen, second, *_r) in SW1_DMIN_REPLAY.items():
            self._replay_one(s, i, frozen, d_latch, second)
        s.n = 4
        line = s._impact_summary()[0]
        assert "鎖存當下水平距 = {1: 43.8, 3: 42.0, 4: 40.3, 5: 226.2} m" in line
        assert "命中時間散布 = 0.00s" in line

    def test_truth_line_marks_missed(self):
        s = node()
        _all_hit(s)
        for i in (2, 6):
            s.impact.pop(i)
            s.missed[i] = s.dive_go + 60.0
            s.miss_why[i] = "逃逸"
        for i in range(1, 7):
            s.truth_dmin[i] = 3.0
            s.truth_hit[i] = 1.0
        lines = s._impact_summary()
        assert len(lines) == 2
        assert "真值基準" in lines[1]
        assert "2: MISS" in lines[1] and "6: MISS" in lines[1]


# ═══════════════════════════════════════════════════════════════════════
#  _tick 的 DIVE 段接線(done 計數 / miss 分支 / 摘要觸發)
# ═══════════════════════════════════════════════════════════════════════

class _Sink:
    def __init__(self):
        self.calls = []

    def __call__(self, *a, **k):
        self.calls.append((a, k))

    flush = publish = write = lambda self, *a, **k: None


def dive_node(**kw):
    """把幾何/鏈路全部換成樁,只留 DIVE 段的控制流。"""
    s = node(**kw)
    s.released = {i: True for i in s.pos}
    s.dive_ring_gate = False
    s.ring_gate_frac = 1.10
    s.ring_replan = True
    s.dive_angle_max = 0.0
    s.pn_on = False
    s.fc = {}
    s.strike = {}
    s.expend = True
    s.gate_profile = False
    s.gate_r = 1200.0
    s.vmin, s.vmax = 45.0, 160.0
    s.goto_radius, s.hold_radius, s.dive_radius = 30.0, 400.0, 30.0
    s.line_tol, s.resend_dt = 80.0, 3.0
    s.dive_alt_agl, s.dive_layer, s.track_frac = 5.0, 4.0, 1.0
    s.wp_t, s.wp_key, s.wp_pt = {}, {}, {}
    s.slot_id = {i: i - 1 for i in s.pos}
    s.thru = {k: at(0.0, 800.0) for k in range(s.n)}
    s._csv = _Sink()
    s.status_pub = _Sink()
    s._goto = _Sink()
    s._goto_latched = _Sink()
    s._row = _Sink()
    s._pn_step = _Sink()
    s._pace_of = lambda i, d: 100.0
    s.ring_of = lambda i: 500.0
    s.ring_point_of = lambda i: at(180.0, 500.0)
    s.perch_of = lambda i: 3000.0
    s._line_alt = lambda i, d_aim: -50.0
    s._pump = lambda: None
    return s


def _tick_dive(s, now):
    """只跑 _tick 的 DIVE 段(繞開 _pump / INIT / ALIGN / PERCH)。"""
    tot_node.time.time = lambda: now        # _goto 已被樁掉,這裡只影響 DIVE
    s._tick()


class TestDiveWiring:
    def test_escaped_aircraft_is_counted_and_run_ends(self):
        """SW1 的病:U2/U6 逃逸 ⇒ done 永遠不到 6 ⇒ 不印摘要、不進 DONE。"""
        s = dive_node()
        now = s.dive_go + 50.0
        for i in (1, 3, 4, 5):              # 四架已命中
            s.impact[i] = now - 1.0
            s.dmin[i] = 3.0
            s.hit_alt[i] = 2.0
        for i in (2, 6):                    # 兩架在 200/400 km 外
            s.pos[i] = (*at(0.0, 207000.0 if i == 2 else 412000.0), 1700.0)
            s.dmin[i] = 638.1
        _tick_dive(s, now)
        assert set(s.missed) == {2, 6}
        assert s._done_count() == 6
        assert s.done_t == pytest.approx(now)
        _tick_dive(s, now + s.impact_settle + 0.1)
        assert s.phase == "DONE"
        assert CLOSEOUT in s.log.text()
        assert SENTINEL not in s.log.text()

    def test_missed_aircraft_gets_hold_not_dive(self):
        s = dive_node()
        now = s.dive_go + 50.0
        s.pos[2] = (*at(0.0, 412000.0), 1700.0)
        _tick_dive(s, now)
        assert 2 in s.missed
        s._goto.calls.clear()
        s._goto_latched.calls.clear()
        _tick_dive(s, now + 1.0)
        assert not any(a[0] == 2 for a, _ in s._goto_latched.calls), \
            "已判 MISS 還在發俯衝航點"
        held = [(a, k) for a, k in s._goto.calls if a[0] == 2]
        assert held, "已判 MISS 卻完全不下令(飛機會維持最後一個俯衝目標)"
        assert held[0][1]["radius"] == s.hold_radius     # 要繞,不是飛過去

    def test_escaped_aircraft_cannot_be_recounted_as_hit(self):
        """逃逸機幾百公里外繞回來重穿紅心 —— 不可以被記成命中(幽靈命中)。"""
        s = dive_node()
        now = s.dive_go + 50.0
        s.pos[2] = (*at(0.0, 412000.0), 1700.0)
        _tick_dive(s, now)
        assert 2 in s.missed
        s.pos[2] = (*at(0.0, 5.0), 1498.0)              # 幾小時後重穿
        _tick_dive(s, now + 11950.0)
        assert 2 not in s.impact
        assert s.dmin[2] > 1000.0                        # 也不准把 dmin 洗掉

    def test_zero_miss_path_is_untouched(self):
        """回歸:沒有任何 miss 時,飛行行為與舊哨兵字串都不變。

        ⚠️ 2026-08-05:固定結案哨兵 CLOSEOUT 現在**四種形狀都印**(見
           tot_node.CLOSEOUT_SENTINEL),所以這裡不再斷言「零 miss 不含
           CLOSEOUT」——改成斷言 log 裡沒有任何脫靶痕跡。
        """
        s = dive_node()
        now = s.dive_go + 50.0
        for i in range(1, 7):
            # alt 相對 HOME −30 ⇒ agl = −30 − (−32) = 2 m,低於 impact_alt_ok
            s.pos[i] = (*at(180.0, 5.0 + i), -30.0)
            s.dmin[i] = 5.0 + i
        _tick_dive(s, now)
        assert set(s.impact) == set(range(1, 7))
        assert s.missed == {}
        _tick_dive(s, now + s.impact_settle + 0.1)
        assert s.phase == "DONE"
        assert SENTINEL in s.log.text()
        assert CLOSEOUT in s.log.text()
        assert "、0 MISS;" in s.log.text()
        assert "判定 MISS" not in s.log.text()

    def test_dmin_freezes_at_the_latch_tick(self):
        """⭐ 接線層(不是述詞層)的凍結證據,連同它的代價。

        鎖存條件是 `d < impact_reach`(SW1 生效值 25 m),終端相鄰取樣的水平距
        步長實測 3–9 m ⇒ 鎖存那一筆必落在 16–25 m。凍在那裡 ⇒ 摘要行報 23 m,
        而這一架後來真的走到 3 m。
        ⚠️ 這個代價是**知情接受**的:節點在現場沒有整條軌跡可以回頭看,任何
           「再等一下」的窗都會等到機體入水之後(SW1 是 1.9–2.6 s 觸海、
           31.7–45.5 s 才收窗)。真最近點交給 stot_score.py 的 d1。
        """
        s = dive_node()
        s.impact_reach = 25.0
        now = s.dive_go + 50.0
        for i in range(2, 7):                       # 其餘五架先結案
            s.impact[i] = now - 1.0
            s.dmin[i] = 3.0
            s.hit_d[i] = 3.0
            s.hit_alt[i] = 2.0
        for k, d in enumerate((41.0, 32.0, 23.0, 14.0, 5.0, 3.0, 7.0)):
            s.pos[1] = (*at(180.0, d), -30.0)
            _tick_dive(s, now + 0.05 * k)
        assert 1 in s.impact
        assert s.hit_d[1] == pytest.approx(23.0, abs=0.5), "鎖存當下的距離"
        assert s.dmin[1] == pytest.approx(23.0, abs=0.5), \
            "鎖存後還在收 ⇒ 窗開著 ⇒ 遲早收到入水後的第二趟"
        line = s._impact_summary()[0]
        assert "最近水平距 = {1: 23.0" in line
        assert "鎖存當下水平距 = {1: 23.0" in line
        assert "無收斂窗" in line and "上界" in line, \
            "摘要行必須自己講清楚它是上界,不能讓讀者以為那是真最近點"

    def test_reapproach_after_the_latch_cannot_wash_dmin(self):
        """飛遠再繞回來不可以把 dmin 洗掉 —— 不管隔了 10 秒還是 3 小時。"""
        s = dive_node()
        s.impact_reach = 25.0
        now = s.dive_go + 50.0
        for i in range(2, 7):
            s.impact[i] = now - 1.0
            s.dmin[i] = 3.0
            s.hit_d[i] = 3.0
            s.hit_alt[i] = 2.0
        s.pos[1] = (*at(180.0, 23.0), -30.0)
        _tick_dive(s, now)
        assert 1 in s.impact and s.dmin[1] == pytest.approx(23.0, abs=0.5)
        s.pos[1] = (*at(180.0, 1200.0), -30.0)          # 衝出去
        _tick_dive(s, now + 10.0)
        s.pos[1] = (*at(180.0, 1.0), -30.0)             # 繞回來重穿
        _tick_dive(s, now + 20.0)
        assert s.dmin[1] == pytest.approx(23.0, abs=0.5)

    def test_stalled_unreleased_aircraft_still_reaches_the_exit(self):
        """從未放行、卡在環上不動的機也要走得到出口 ——
        否則 `if not released: continue` 會讓它永遠不計入 done。"""
        s = dive_node()
        s.released = {i: True for i in s.pos}
        s.released[4] = False
        now = s.dive_go + 50.0
        for i in (1, 2, 3, 5, 6):
            s.impact[i] = now - 1.0
            s.dmin[i] = 3.0
            s.hit_alt[i] = 2.0
        s.gs[4] = 0.5
        for k in range(0, 70):
            _tick_dive(s, now + k)
        assert 4 in s.missed
        assert s._done_count() == 6


# ═══════════════════════════════════════════════════════════════════════
#  M5 —— CSV 的 agl 欄
# ═══════════════════════════════════════════════════════════════════════

class TestCsvAgl:
    def test_agl_is_appended_after_the_positional_columns(self):
        """⚠️ 新欄只能追加在**位置解析的 12 欄之後**:
        _tot_plot.py / _dive_plot.py 用 r[0]…r[11](r[7]=mode、r[11]=phase),
        插在中間會靜默錯位。

        ⚠️ 2026-08-14:本測試原本斷言 `len(r) == 13`(agl 是最後一欄)。
           那把「追加在後面」誤鎖成「agl 必須是最末欄」—— 兩者不同。
           S1/S9 又追加了 gam_deg / pred_* 五欄,agl 仍在 index 12、
           位置解析的 12 欄仍未被推位,不變式其實沒被破壞。
           改成鎖真正的不變式:前 12 欄的位置 + agl 的位置。
        """
        s = node()
        rows = []
        s._cw = types.SimpleNamespace(writerow=rows.append)
        s.last_v = {i: 100.0 for i in s.pos}
        s.target_amsl = 0.0
        s.zg = {1: -32.0}
        s.pos[1] = (*at(180.0, 500.0), 400.0)
        s._row(s.t0 + 10.0, 1, 500.0, "dive", TGT)
        r = rows[0]
        assert len(r) >= 13
        assert r[7] == "dive" and r[11] == "DIVE"        # 位置解析的 12 欄沒被推位
        assert r[4] == pytest.approx(400.0)              # alt 相對 HOME
        assert r[12] == pytest.approx(432.0)             # agl = 400 − (−32)
        # ⚠️ 追加欄未啟用時必須寫**空字串**不是 0 —— 0 是合法的預測值,
        #    用 0 當「沒跑」會讓事後判讀把停用誤讀成「預測殘餘為零」。
        assert all(x == "" or isinstance(x, (int, float)) for x in r[13:])
        assert "CSV 高度基準生效值" in s.log.text()

    def test_datum_line_is_logged_once(self):
        s = node()
        rows = []
        s._cw = types.SimpleNamespace(writerow=rows.append)
        s.last_v = {i: 100.0 for i in s.pos}
        s.target_amsl = 0.0
        for _ in range(5):
            s._row(s.t0 + 10.0, 1, 500.0, "dive", TGT)
        assert s.log.text().count("CSV 高度基準生效值") == 1

    def test_header_has_agl_last(self):
        src = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
        assert '"slot_lon", "phase",\n                           "agl"' in src


# ═══════════════════════════════════════════════════════════════════════
#  harness:兩種結案行都要被 grep 得到
# ═══════════════════════════════════════════════════════════════════════

WSL = REPO / "scripts" / "wsl"


MISS_PARAMS = ["miss_far_m", "miss_far_k", "miss_window_s",
               "miss_slow_gs", "miss_slow_sec", "impact_settle"]

# ⚠️ 2026-08-04 移除。留在這裡當**反向**鎖:旋鈕若被誰加回來,調它會靜默地
#    把入水後的第二趟收進 dmin(SW1:U3 32.6→22.6 剛好跨過 MISS_MAX=30)。
REMOVED_PARAMS = ["dmin_hyst_m", "dmin_hold_s"]


@pytest.mark.parametrize("name", MISS_PARAMS)
def test_harness_can_actually_tune_the_exit(name):
    """出口的門檻必須從 harness 調得動,而且節點端真的宣告了同名參數。

    ⚠️ 打錯一個字的後果是**靜默**的:ROS 2 對未宣告的 -p 只會忽略,航次照跑,
       用的是節點預設值 —— 「機制沒開」和「機制開了但沒用」又分不出來。
    """
    sh = (WSL / "stot_trial.sh").read_text(encoding="utf-8")
    node_src = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
    assert f"-p {name}:=" in sh, f"stot_trial.sh 沒有把 {name} 傳下去"
    assert f'g("{name}"' in node_src, f"tot_node 沒有宣告 {name}"


@pytest.mark.parametrize("name", REMOVED_PARAMS)
def test_removed_convergence_knobs_stay_removed(name):
    """⚠️ 反向鎖:dmin 的收斂窗參數不可以回來。

    留著旋鈕就會有人調,而調它的後果是**數字變好看**(收進機體入水後爬起來
    的第二趟),沒有任何一行 log 會說出這件事 —— 本專案的頭號死因。
    """
    sh = (WSL / "stot_trial.sh").read_text(encoding="utf-8")
    node_src = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
    assert f"-p {name}:=" not in sh
    assert f'g("{name}"' not in node_src


@pytest.mark.parametrize("var", ["MISS_FAR_M", "MISS_FAR_K", "MISS_WINDOW_S",
                                 "MISS_SLOW_GS", "MISS_SLOW_SEC",
                                 "IMPACT_SETTLE"])
def test_float_params_go_through_the_decimal_point_guard(var):
    """⚠️ ROS 2 的 -p 會把 "20000" 判成 INTEGER,而宣告是 DOUBLE ⇒ 節點啟動即崩。
    所有 float 參數一律過 _f(),不要靠呼叫端記得寫小數點。"""
    sh = (WSL / "stot_trial.sh").read_text(encoding="utf-8")
    assert f'{var}="$(_f "${{{var}:-' in sh


@pytest.mark.parametrize("rel", ["stot_trial.sh", "stot_kpi_loop.sh"])
def test_break_condition_matches_both_lines(rel):
    """⚠️ 2026-08-05 起 break 條件改認**固定哨兵**「★★ 俯衝結案:」。

    舊的兩段式(舊哨兵|俯衝結案:)在「零 miss 但有機高空掠過」時兩段都不中
    (達標描述把舊哨兵蓋掉了)⇒ 空等到 timeout。四種形狀 × 三個消費者的
    完整匹配矩陣鎖在 tests/test_summary_sentinel.py。
    """
    src = (WSL / rel).read_text(encoding="utf-8")
    assert "★★ 俯衝結案:|俯衝命中紅心:全員命中" in src, \
        f"{rel} 的 break 條件沒有同步改成固定哨兵 ⇒ 某些結案形狀會空等到 timeout"


def _run_kpi(tmp_path, summary_line):
    log = tmp_path / "trial.log"
    log.write_text(summary_line + "\n", encoding="utf-8")
    out = tmp_path / "kpi.csv"
    env = dict(os.environ, TOT_LOG=str(log), CSV=str(out), TRIAL="TST",
               MISS_MAX="30", ALT_MAX="30", SPREAD_MAX="3.0",
               PYTHONIOENCODING="utf-8")
    subprocess.run([sys.executable, str(WSL / "_kpi_parse.py")],
                   env=env, check=True, capture_output=True)
    return out.read_text(encoding="utf-8").strip()


def test_kpi_parse_reads_the_closeout_line(tmp_path):
    """有 miss 時 KPI 不可以再寫 no_summary —— 那會把「跑完但脫靶」記成
    「沒跑完」,歸因整個錯掉(SW1 就是這樣被記成 timeout 的)。"""
    s = node()
    for i in (1, 3, 4, 5):
        s.impact[i] = s.dive_go + 40.0 + 0.1 * i
        s.dmin[i] = 5.0
        s.hit_alt[i] = 2.0
    for i in (2, 6):
        s.missed[i] = s.dive_go + 60.0
        s.miss_why[i] = "逃逸:離目標 412.0 km > miss_far_m 20.0 km"
        s.dmin[i] = 638.1
    row = _run_kpi(tmp_path, "[WARN] [tot_node]: " + s._impact_summary()[0])
    assert "no_summary" not in row
    assert row.split(",")[4] == "4"        # n_hit 只算真的命中的
    assert row.split(",")[5] == "0"        # 不通過
    assert "n=4" in row


def test_kpi_parse_still_reads_the_old_sentinel(tmp_path):
    s = node()
    _all_hit(s)
    row = _run_kpi(tmp_path, "[WARN] [tot_node]: " + s._impact_summary()[0])
    assert row.split(",")[4] == "6"
    assert row.split(",")[5] == "1"        # 6/6、誤差與散布都在門檻內 ⇒ 通過


def test_kpi_parse_reads_dmin_not_the_latch_distance(tmp_path):
    """⚠️⚠️ 摘要行現在有兩組水平距。_kpi_parse 的 `最近水平距 = \\{…\\}` 必須
    咬到**最近通過距離**,不是新加的「鎖存當下水平距」——咬錯就等於把
    blocker 描述的 16–25 m 下限原封不動搬進 KPI 表。"""
    s = node()
    for i in range(1, 7):
        s.impact[i] = s.dive_go + 40.0 + 0.1 * i
        s.dmin[i] = 3.0                     # 真正的最近通過距離
        s.hit_d[i] = 23.0                   # 鎖存當下(必然比較大)
        s.hit_alt[i] = 2.0
    line = s._impact_summary()[0]
    assert "鎖存當下水平距" in line
    row = _run_kpi(tmp_path, "[WARN] [tot_node]: " + line)
    assert row.split(",")[2] == "3.0", f"咬到鎖存距離了:{row}"
    assert row.split(",")[4] == "6"
    assert row.split(",")[5] == "1"
