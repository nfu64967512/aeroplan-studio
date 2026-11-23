# -*- coding: utf-8 -*-
"""協同歸向(Cooperative Homing / consensus-based ITCG)兩個話題的值層級鎖。

═══════════════════════════════════════════════════════════════════════════
 這一組鎖在守什麼
═══════════════════════════════════════════════════════════════════════════
    /swarm/coordination_state   20 Hz,全機的共識狀態(BEST_EFFORT/VOLATILE/1)
    /swarm/handover_gate        2 Hz,放行閘門的判定(RELIABLE/VOLATILE/5)

三個致命點,每一個都有實測或事故撐著:

⚠️⚠️⚠️ **放行話題絕對不可以 TRANSIENT_LOCAL。** 它會載 `RELEASE_DIVE`;
   latched 之下**任何晚連的節點一連上就立刻收到放行指令** —— 包含下一發任務
   才起來的節點。這是本檔最重要的一條鎖。

⚠️⚠️ **共識狀態不含 `estimated_tgo`。** [[predictor-underestimates-dive]]:
   預測器低估俯衝耗時**中位 +18.7 s、27/27 單邊為正**。而 Zarchan 附錄 A.1
   逐字:optimal guidance 在 tgo **尺度誤差 0.9** 之下,飛行時間 >8 s 就輸給
   普通 PN —— 我們是 1.4–1.5,超過四到五倍。
   ⇒ 廣播壞掉的 t_go 去取共識,得到的是「一致」不是「準」,
     而一致的錯誤比不一致更難發現。改為廣播原始可觀測量,消費者自己算。

⚠️⚠️ **`stamp_age_s` 是機體自己的狀態有多舊,不是訊息有多舊。**
   [[leader-state-freeze]]:長機位姿凍結但仍以 10 Hz 廣播 ——
   「停發有告警,凍結沒有」。只看訊息到達時間的訂閱端會被凍結的長機騙。

⭐ 現階段 `gate_arm=False` ⇒ **只觀測不致動**。理由:規格要的「時間差壓進
   1 秒」在實測數字下(放行差 2.4 / 7.6 / 15.2 s,WX1-3)**一次都不會觸發**,
   而一個從不觸發的閘門與一個運作中的閘門長得一模一樣(RRB 八趟白跑的形狀)。

═══════════════════════════════════════════════════════════════════════════
 ⭐ 值層級,不比源碼字串([[value-locks-beat-source-locks]])
═══════════════════════════════════════════════════════════════════════════
把方法從檔案原始碼綁成 mixin、餵數字、對送出來的 dict 下斷言。
唯一走 AST 的是綁定本身與讀 QoS 宣告 —— 讀的是**值**,不是那一行長什麼樣。
"""
import ast
import json
import math
import pathlib
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))


def _func(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError(f"tot_node.py 找不到 {name}()")


def _mixin(names):
    # ⚠️ `time` 要放進命名空間:被測方法用 time.monotonic() 取時戳。
    #    少了它是 NameError,而那會被誤讀成「方法壞了」。
    # ⚠️ `String` 是 ROS2 的訊息型別,而 rclpy 在 Windows 開發機上裝不起來。
    #    用一個只有 .data 的樁 —— 被測方法只用到那一個欄位,
    #    而測試必須**每次都跑得到**(跑不到的鎖等於沒有鎖)。
    class String:                                               # noqa: N801
        def __init__(self, data=""):
            self.data = data

    ns = {"math": math, "json": json, "time": time, "String": String}
    for n in TREE.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(
                n.targets[0], ast.Name):
            try:
                ns[n.targets[0].id] = ast.literal_eval(n.value)
            except Exception:                                   # noqa: BLE001
                pass
    for n in TREE.body:
        if isinstance(n, ast.FunctionDef):
            try:
                exec(ast.get_source_segment(SRC, n), ns)        # noqa: S102
            except Exception:                                   # noqa: BLE001
                pass
    body = "\n\n".join(
        "\n".join("    " + ln
                  for ln in ast.get_source_segment(SRC, _func(f)).splitlines())
        for f in names)
    exec("class _M:\n" + body, ns)                              # noqa: S102
    return ns["_M"]


_C = _mixin(("_coord_row", "_publish_coord", "_handover_criteria",
             "_publish_handover"))


class Pub:
    """把發出去的訊息留下來,而不是丟掉。"""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))

    def last(self):
        assert self.sent, "什麼都沒發"
        return self.sent[-1]


class Node(_C):
    """⚠️ 樁只提供依賴,不重寫任何被測邏輯。"""

    T = (25.0, 121.0)

    def __init__(self, *, n=2, gs=None, dists=None, agl=None, ages=None,
                 arm=False, spread=1.0, band=100.0, rerr=0.0,
                 max_age=1.0, target=True):
        self.n = n
        self.phase = "RUN"
        self.gate_arm = arm
        self.gate_spread_s = spread
        self.gate_alt_band_m = band
        self.gate_range_err_m = rerr
        self.state_max_age = max_age
        self.ring_gate_frac = 1.1
        self._ring = 6000.0
        self.released = {}
        self.gs = dict(gs or {1: 150.0, 2: 150.0})
        self._d = dict(dists or {1: 12000.0, 2: 12300.0})
        self._agl = dict(agl or {1: 3100.0, 2: 3120.0})
        self._age = dict(ages or {1: 0.05, 2: 0.05})
        if not target:
            self.T = (float("nan"), float("nan"))
        # 目標在正北;把「距目標多遠」換算成緯度差。
        self.pos = {i: (self.T[0] + d / 111320.0, self.T[1], self._agl[i])
                    for i, d in self._d.items()}
        self.state_t = {i: 1000.0 - a for i, a in self._age.items()}
        self.coord_pub = Pub()
        self.handover_pub = Pub()

    # ── 依賴 ──
    def ring_of(self, i):
        return self._ring

    def _zg(self, i):
        return 0.0

    @staticmethod
    def is_standby_target(lat, lon):
        return not (lat == lat and lon == lon)          # NaN 判準

    def now(self):
        return 1000.0


def coord(n, monkeypatch=None):
    """跑一次 20 Hz 廣播,回傳送出去的 dict。

    ⚠️ 把 time.monotonic 釘死在 1000.0:`stamp_age_s` 是 now − state_t,
       不釘死的話那一欄每次都不同,測不了。
    """
    real = time.monotonic
    time.monotonic = lambda: 1000.0
    try:
        n._publish_coord()
    finally:
        time.monotonic = real
    return n.coord_pub.last()


def gate(n):
    return (n._publish_handover(now=1000.0), n.handover_pub.last())[1]


# ══════════════════════════════════════════════════════════════════════════
class TestTheQosIsTheOneThatCannotFireAStaleRelease:
    """⚠️⚠️⚠️ 本檔最重要的一組。讀的是 QoS 宣告的**值**。"""

    @staticmethod
    def _qos(name):
        for n in ast.walk(TREE):
            if (isinstance(n, ast.Assign) and len(n.targets) == 1
                    and isinstance(n.targets[0], ast.Name)
                    and n.targets[0].id == name):
                kw = {k.arg: ast.unparse(k.value) for k in n.value.keywords}
                return kw
        raise AssertionError(f"tot_node.py 找不到 {name}")

    def test_the_release_topic_is_never_latched(self):
        """⚠️⚠️⚠️ TRANSIENT_LOCAL 的放行話題 = 任何晚連的節點一連上就收到
        `RELEASE_DIVE`,包含下一發任務才起來的節點。"""
        d = self._qos("HANDOVER_QOS")["durability"]
        assert "VOLATILE" in d, (
            f"⚠️⚠️⚠️ /swarm/handover_gate 的 durability 是 {d} —— "
            "latched 的放行指令會餵給每一個晚連的訂閱者。必須 VOLATILE。")

    def test_the_release_topic_is_reliable(self):
        """放行指令丟一次就是一次沒放行,而且完全靜默。"""
        assert "RELIABLE" in self._qos("HANDOVER_QOS")["reliability"]

    def test_the_release_topic_keeps_more_than_one(self):
        """arm / release / abort 會連發;depth=1 的訂閱者只看得到最後一筆,
        而看不到**理由**。"""
        assert int(self._qos("HANDOVER_QOS")["depth"]) >= 3

    def test_the_state_topic_is_not_latched_either(self):
        """⚠️ latched 的 20 Hz 狀態流 = 晚連的節點拿到一筆**過期的隊友狀態**
        並拿去算共識。新鮮度交給 state_max_age,不是交給 QoS。"""
        d = self._qos("COORD_QOS")["durability"]
        assert "VOLATILE" in d, f"COORD_QOS durability = {d}"

    def test_the_state_topic_does_not_queue(self):
        """20 Hz 的狀態流丟一兩筆無所謂,積壓才是問題。"""
        assert int(self._qos("COORD_QOS")["depth"]) == 1
        assert "BEST_EFFORT" in self._qos("COORD_QOS")["reliability"]


class TestTheStateNeverCarriesThePredictorsGuess:
    """⚠️⚠️ 對壞掉的 t_go 取共識 = 得到一致的錯誤。"""

    def test_no_tgo_field_anywhere(self):
        row = coord(Node())["uavs"][0]
        bad = [k for k in row if "tgo" in k.lower() or "t_go" in k.lower()]
        assert not bad, (
            f"⚠️⚠️ 共識狀態帶了 {bad} —— 預測器低估中位 +18.7 s、27/27 單邊,"
            "而 Zarchan 附錄 A.1:tgo 尺度誤差 0.9 就輸給普通 PN(我們 1.4–1.5)。"
            "廣播原始可觀測量,讓消費者自己算。")

    def test_it_carries_the_raw_observables_instead(self):
        row = coord(Node())["uavs"][0]
        for k in ("r_to_gate_m", "gs_mps", "agl_m", "stamp_age_s", "uav_id"):
            assert k in row, f"少了 {k} —— 消費者算不出 eta"

    def test_eta_is_recoverable_by_the_consumer(self):
        """⭐ 端到端的意義檢查:消費者拿到的兩個量除一除,要等於實測 eta。"""
        n = Node()
        row = coord(n)["uavs"][0]
        want = (n._d[1] - n._ring * n.ring_gate_frac) / n.gs[1]
        assert row["r_to_gate_m"] / row["gs_mps"] == pytest.approx(want, rel=1e-3)


class TestTheStateIsHonestAboutBeingStale:
    """⚠️⚠️ [[leader-state-freeze]]:凍結的長機仍以 10 Hz 廣播。
    「停發有告警,凍結沒有。」"""

    def test_stamp_age_is_the_vehicles_own_state_age(self):
        n = Node(ages={1: 0.05, 2: 4.2})
        rows = {r["uav_id"]: r for r in coord(n)["uavs"]}
        assert rows[2]["stamp_age_s"] == pytest.approx(4.2, abs=1e-3), (
            "⚠️ 這一欄必須是**機體自己的狀態**有多舊。訊息剛到但狀態凍結了 4.2 s,"
            "只看訊息到達時間的訂閱端會完全看不見。")

    def test_the_sequence_number_is_monotonic(self):
        """訂閱端靠它判斷發布端是不是卡住了。"""
        n = Node()
        a, b = coord(n)["seq"], coord(n)["seq"]
        assert b == a + 1

    def test_a_stale_vehicle_is_excluded_from_the_gate_and_named(self):
        g = gate(Node(ages={1: 0.05, 2: 9.0}, max_age=1.0))
        assert 2 in g["stale"], "過期的機體要被點名"
        assert "2" not in g["eta_s"], "過期的機體不可以進共識"
        assert not g["would_release"]

    def test_state_freshness_off_means_nothing_is_stale(self):
        """state_max_age=0 是「不做新鮮度檢查」的既有語意,不可以變成
        「全部都算過期」。"""
        assert gate(Node(ages={1: 9.0, 2: 9.0}, max_age=0.0))["stale"] == []


class TestTheGateObservesButDoesNotAct:
    """⭐ gate_arm=False ⇒ 只觀測。這是現階段的**設計**,不是未完成。"""

    #: 全部判準都通過的一組。
    GOOD = dict(dists={1: 12000.0, 2: 12030.0}, agl={1: 3100.0, 2: 3110.0})

    def test_it_says_would_release_but_commands_hold(self):
        g = gate(Node(**self.GOOD, arm=False))
        assert g["would_release"] is True, "判準應該通過(否則這個測試沒在測東西)"
        assert g["armed"] is False
        assert g["command"] == "HOLD", (
            "⚠️⚠️ gate_arm=False 之下**絕不可以**送出 RELEASE_DIVE —— "
            "訂閱端會照著放行,而放行決定權仍在既有邏輯手上。")

    def test_arming_it_lets_the_command_through(self):
        """反向鎖:旗標要真的有作用,否則這顆開關是裝飾品。"""
        assert gate(Node(**self.GOOD, arm=True))["command"] == "RELEASE_DIVE"

    def test_armed_still_holds_when_the_criteria_fail(self):
        g = gate(Node(dists={1: 12000.0, 2: 20000.0}, arm=True))
        assert not g["would_release"] and g["command"] == "HOLD"


class TestTheGateMeasuresBeforeItJudges:
    """⭐ 它每一拍都要印出**當下的實測散布**,不只是通過與否 ——
    那才是決定門檻該訂多少的資料。"""

    def test_it_publishes_the_measured_spread(self):
        n = Node(dists={1: 12000.0, 2: 13500.0})
        g = gate(n)
        want = (13500.0 - 12000.0) / 150.0          # 兩機同速 ⇒ eta 差 = 距離差/gs
        assert g["spread_s"] == pytest.approx(want, rel=1e-3)

    def test_it_publishes_the_measured_altitude_band(self):
        assert gate(Node(agl={1: 3100.0, 2: 3450.0}))["alt_band_m"] == \
            pytest.approx(350.0, abs=0.5)

    def test_it_publishes_the_thresholds_it_used(self):
        """⚠️ 事後判讀必須看得出「當時的門檻是多少」——
        門檻改過而 log 沒記,兩批資料就不能比。"""
        g = gate(Node(spread=2.5, band=250.0, rerr=400.0))
        assert g["thresholds"] == {"spread_s": 2.5, "alt_band_m": 250.0,
                                   "range_err_m": 400.0}

    def test_it_says_why_when_it_holds(self):
        why = " ".join(gate(Node(dists={1: 12000.0, 2: 20000.0}))["why"])
        assert "時間差" in why and "8" in why or "時間差" in why

    def test_the_measured_spread_at_our_real_numbers_fails_the_1s_gate(self):
        """⭐⭐ 這一條把「先只觀測」的理由釘成一個數字。

        實測放行時刻差 2.4 / 7.6 / 15.2 s(WX1-3)。1 秒的門檻對三趟全部不通過
        ⇒ 硬閘門會一次都不觸發,而那與「閘門壞了」完全無法區分。
        """
        for gap_s in (2.4, 7.6, 15.2):
            n = Node(dists={1: 12000.0, 2: 12000.0 + gap_s * 150.0}, spread=1.0)
            g = gate(n)
            assert g["spread_s"] == pytest.approx(gap_s, rel=1e-2)
            assert not g["would_release"], (
                f"實測 {gap_s}s 的差距竟然通過 1 s 門檻 —— 判準寫反了")


class TestTheRangeCriterionFollowsTheLiteratureForm:
    """⭐⭐⭐ 已發表的 consensus-ITCG 切進純 PN,判的是**距離誤差**不是時間誤差。

    Zhu / Wang / Lin,《Consensus-Based Impact-Time-Control Guidance Law for
    Cooperative Attack of Multiple Missiles》Eq. (11) 逐字:

        a_i = consensus terms,  if |R_i − r_i| >= 30
              N*V_i*q̇_i,        if |R_i − r_i| <  30

    ⭐ 對我們特別合適:距離**量得準**,而時間誤差要過預測器(低估中位
       +18.7 s、27/27 單邊)。

    ⚠️⚠️ 但**只採用形式,不照抄數字**:論文的 30 在我們手上的抽取文字裡
       單位不明(模擬表用 km,而 30 km 對那個場景太大)。⇒ 預設 0 = 停用,
       門檻要先量再訂。照抄一個看不懂單位的數字去擋放行,就是拿沒有出處的
       門檻當判準。
    """

    def test_the_range_spread_is_always_published_even_when_disabled(self):
        """⭐ 停用時仍要發布 —— 這一欄就是用來決定門檻該訂多少的資料。"""
        g = gate(Node(dists={1: 12000.0, 2: 13500.0}, rerr=0.0))
        assert g["range_err_m"] == pytest.approx(1500.0, abs=1.0)

    def test_disabled_means_it_cannot_block(self):
        """0 = 停用。⚠️ 停用的判準若還會擋,那是一個沒有出處的門檻在擋放行。"""
        n = Node(dists={1: 12000.0, 2: 13500.0},        # 距離差 1500 m
                 agl={1: 3100.0, 2: 3105.0}, spread=30.0, rerr=0.0)
        assert gate(n)["would_release"] is True

    def test_enabling_it_blocks_a_wide_formation(self):
        n = Node(dists={1: 12000.0, 2: 13500.0},
                 agl={1: 3100.0, 2: 3105.0}, spread=30.0, rerr=500.0)
        g = gate(n)
        assert not g["would_release"]
        assert any("距離離散" in w for w in g["why"])

    def test_enabling_it_passes_a_tight_formation(self):
        n = Node(dists={1: 12000.0, 2: 12100.0},
                 agl={1: 3100.0, 2: 3105.0}, spread=30.0, rerr=500.0)
        assert gate(n)["would_release"] is True

    def test_a_stale_vehicle_does_not_widen_the_range_spread(self):
        """⚠️ 過期的機體已經被排除在時間共識外,距離判準也必須一致 ——
        兩個判準用不同的成員集合,會讓「為什麼沒放行」講不清楚。"""
        n = Node(dists={1: 12000.0, 2: 40000.0}, ages={1: 0.05, 2: 9.0},
                 max_age=1.0, rerr=500.0)
        assert gate(n)["range_err_m"] is None, (
            "只剩一架可用時算不出離散,要回 None 而不是 0")


class TestItDoesNotInventNumbers:
    def test_no_target_means_no_range(self):
        """⚠️ 待命時 T 是 (nan, nan)。算距離會得到 nan 而 json 送不出去,
        更糟的是**送得出去但是垃圾**。"""
        row = coord(Node(target=False))["uavs"][0]
        assert row["r_to_gate_m"] is None and row["r_to_target_m"] is None

    def test_range_past_the_gate_is_clamped_not_negative(self):
        """⚠️ 越過閘門之後 r 為負,而負 r 除以 gs 給出負 eta ——
        共識會往反方向跑,且完全合理地跑。"""
        n = Node(dists={1: 3000.0, 2: 12000.0})
        rows = {r["uav_id"]: r for r in coord(n)["uavs"]}
        assert rows[1]["r_to_gate_m"] == 0.0

    def test_a_vehicle_without_speed_is_left_out_of_eta(self):
        g = gate(Node(gs={1: 150.0, 2: 0.0}))
        assert "2" not in g["eta_s"] and not g["would_release"]

    def test_a_vehicle_with_no_fix_is_simply_absent(self):
        n = Node()
        del n.pos[2]
        c = coord(n)
        assert c["n_with_state"] == 1 and c["n"] == 2, (
            "⚠️ 缺機體要看得出來:n_with_state < n 就是有機體沒狀態")
