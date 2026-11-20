"""tot_node 的待命模式(STANDBY)+ 執行期目標指定 + 可重複打擊。

背景 —— 為什麼要有這個模式:
  tot_node 的目標一直是**啟動參數**(tot_node.py 的 `self.T = (p("tlat"), p("tlon"))`),
  全檔沒有 create_subscription、沒有 add_on_set_parameters_callback ⇒ 執行期換不了
  目標。於是「AeroPlan 右鍵指定紅心」只能走批次路徑(run_west.sh → stot_repeat.sh),
  而那條路徑每一趟開頭都會 swarm_sitl_down + up + sleep 150 —— 按下去等於把六架
  SITL 拆掉重建,互動性為零。

這裡測的是**真的那份 tot_node.py**(用 sys.modules 補 rclpy/std_msgs 的樁,物件用
__new__ 造、只塞受測方法會碰到的欄位),照 tests/test_miss_exit.py 的既有模式。
複製一份邏輯來測只會證明複製品是對的。

四條紅線,每一條都有測試撐著:
  1. **既有啟動路徑零改變** —— 帶 tlat/tlon 起來就跟 2026-08-05 之前一模一樣:
     phase 直接 INIT、CSV 檔名不變、打完停在 DONE、執行期不接受換目標。
  2. **接新目標時狀態要清乾淨** —— 漏一項全部是靜默的(見 RESET_DICTS 的說明)。
  3. **座標要驗** —— (0,0) 是合法經緯度,任何範圍檢查都攔不住,必須單獨擋。
  4. **可重複** —— 打完回 STANDBY,能接第二個目標,而且不覆蓋第一發的證據。
"""
import importlib
import math
import re
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PATCHES = REPO / "scripts" / "wsl" / "sar_swarm_patches"
WSL = REPO / "scripts" / "wsl"
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
EARTH = tot_node.EARTH

SRC = (PATCHES / "tot_node.py").read_text(encoding="utf-8")
UP_SH = (WSL / "swarm_ros2_up.sh").read_text(encoding="utf-8")
TRIAL_SH = (WSL / "stot_trial.sh").read_text(encoding="utf-8")
#: ⚠️ 批次航次的**生效值**在這裡,不在 stot_trial.sh —— run_west.sh 覆寫了
#:    其中 5 項(見 TestSwarmRos2Up.WEST_OVERRIDES)。
WEST_SH = (WSL / "run_west.sh").read_text(encoding="utf-8")


# ═══════════════════════════════════════════════════════════════════════
#  受測物件
# ═══════════════════════════════════════════════════════════════════════

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


TGT = (25.0777, 119.94315)          # SW1 的紅心(台灣海峽海面)
TGT2 = (24.5000, 120.1000)          # 第二發用的另一個點


def at(bearing_deg, dist_m_, origin=TGT):
    b = math.radians(bearing_deg)
    return (origin[0] + dist_m_ * math.cos(b) / EARTH,
            origin[1] + dist_m_ * math.sin(b)
            / (EARTH * math.cos(math.radians(origin[0]))))


def node(standby=True, n=6, now=1.79e9):
    """一個「已經打完一發、狀態全是髒的」TotNode 殼,用來測重設是否乾淨。

    ⚠️ 刻意把 RESET_DICTS / RESET_NONE 裡的每一項都塞成非空 —— 重設漏掉哪一項
       就會在測試裡留下痕跡。空的初始狀態測不出漏網之魚。
    """
    s = TotNode.__new__(TotNode)
    s.log = FakeLogger()
    s.get_logger = lambda: s.log
    s.n = n
    s.standby_mode = standby
    s.phase = "STANDBY" if standby else "INIT"
    s.strike_seq = 0 if standby else 1
    s.target_src = "測試"
    s.auto_approach = False
    s.T = ((float("nan"), float("nan")) if standby else TGT)
    # 幾何(_ring_points 需要)
    s.r_fan = 4000.0
    s.half = 25.0
    s.approach = 180.0
    s.pushover_r = 2268.0
    s.overshoot = 300.0
    s.perch_reach = 260.0
    s.perch_alt = 400.0
    s.dive_angle = 10.0
    s.dive_alt = 5.0
    s.dive_alt_agl = 5.0
    # ⚠️ _zg() 是 `self.zg.get(i, self.ground_alt)` —— 地形回報進來之前所有高度
    #    都走這個後備值。基礎樁少了它,任何碰到高度的路徑(_miss_check → _zg)
    #    都會 AttributeError,而症狀看起來像「待命功能壞了」,其實是樁不完整。
    s.ground_alt = 0.0
    s.talt = 400.0
    s.target_amsl = float("nan")
    s.expend = True
    s.pace_cfg = 50.0
    s.dive_pace = 999.0
    # 還原基準(見 tot_node 的 _dive_alt0 說明)
    s._miss_far0 = 20000.0
    s._ring_r0 = 2268.0
    s._dive_alt0 = 5.0
    s.miss_far = 987654.0               # 被 _clamp_miss_far 夾寬過的髒值
    s.miss_far_k = 2.0
    s.miss_slow_gs = 5.0
    s.miss_slow_sec = 60.0
    s.miss_window = 60.0
    s.impact_reach = 25.0
    s.impact_alt_ok = 30.0
    s.impact_settle = 4.0
    s.switch_r = 300.0
    s.dive_speed = 140.0
    s.ring_replan = True
    s.csv_path = "/tmp/tot_test"
    s._csv = _Sink()
    s._cw = _Sink()
    s.status_pub = _Sink()
    s.gate_pub = _Sink()
    s._standby_stat_t = 0.0
    s._wait_pos_t = 0.0
    s._miss_cfg_logged = True
    s._agl_datum_logged = True
    s._prox_min = 123.0
    s._prox_t = now
    s._write_geom_csv = _Sink()
    s._open_csv = _Sink()
    s._terrain_probe = _Sink()
    s._publish_gate = _Sink()
    # 位置/速度(真實狀態,不在重設清單裡)
    s.pos = {i: (*at(180.0, 30000.0 + 100 * i), 3100.0) for i in range(1, n + 1)}
    s.gs = {i: 150.0 for i in range(1, n + 1)}
    s.vel = {i: (150.0, 0.0, 0.0) for i in range(1, n + 1)}
    s.att = {i: (0.0, 0.0, 0.0) for i in range(1, n + 1)}
    s.truth = {}
    s.home_amsl = {i: 30.0 for i in range(1, n + 1)}
    s.home_ll = {i: (25.0777, 121.2325) for i in range(1, n + 1)}
    s.link = {i: object() for i in range(1, n + 1)}
    # ── 全部塞髒 ──
    for name in tot_node.RESET_DICTS:
        setattr(s, name, {i: f"髒-{name}-{i}" for i in range(1, n + 1)})
    for name in tot_node.RESET_NONE:
        setattr(s, name, now - 100.0)
    s.last_v = {i: 137.0 for i in range(1, n + 1)}
    s.last_v_t = {i: now for i in range(1, n + 1)}
    s._ring_points()
    return s


# ═══════════════════════════════════════════════════════════════════════
#  1. STANDBY 判定
# ═══════════════════════════════════════════════════════════════════════

class TestStandbyDetection:
    @pytest.mark.parametrize("lat,lon", [
        (float("nan"), float("nan")),      # 兩個都 .nan(swarm_ros2_up.sh 用的)
        (float("nan"), 121.2325),          # 只有一個 NaN 也算沒給
        (25.1406, float("nan")),
        (None, None),                      # 參數根本沒宣告值
        ("", ""),                          # 空字串(shell 變數沒展開)
    ])
    def test_missing_target_is_standby(self, lat, lon):
        assert TotNode.is_standby_target(lat, lon) is True

    @pytest.mark.parametrize("lat,lon", [
        (25.1406, 121.2325),               # stot_trial.sh 的預設紅心
        (25.0777, 119.94315),              # SW1 的海上紅心
        (0.0, 0.0),                        # ⚠️ (0,0) **不是**待命,是壞座標 ——
                                           #    由 validate_target 擋,不是這裡
        (-33.9, 151.2),
    ])
    def test_given_target_is_not_standby(self, lat, lon):
        assert TotNode.is_standby_target(lat, lon) is False

    def test_infinity_is_not_treated_as_missing(self):
        """⚠️ inf 不是「沒給」,是壞值 —— 要走 validate_target 那條(會被擋下
        並印原因),不要靜悄悄變成待命,否則使用者按了右鍵完全沒有回饋。"""
        assert TotNode.is_standby_target(float("inf"), 121.0) is False
        ok, why = TotNode.validate_target(float("inf"), 121.0)
        assert ok is False and "有限值" in why

    def test_init_uses_the_single_predicate(self):
        """⚠️ 結構鎖:__init__ 必須呼叫 is_standby_target,不可以另寫一份
        `math.isnan(...) or math.isnan(...)`。兩份判準遲早會分岔,而分岔的
        症狀是「有時候待命有時候不待命」—— 最難查的那一種。"""
        assert "self.standby_mode = self.is_standby_target(*self.T)" in SRC

    def test_declared_default_is_nan_so_no_target_means_no_action(self):
        """⭐ 安全預設:沒給目標 ⇒ 待命,不是拿寫死的桃園座標開打。"""
        assert 'g("tlat", float("nan")); g("tlon", float("nan"))' in SRC
        # ⚠️ 只看**會被執行的碼**。__init__ 的註解裡刻意保留了舊預設值
        #    「(25.1406, 121.2325)」當作反面教材(那個預設的失敗形狀是六架飛機
        #    朝一個沒人指定的點衝下去,而且零錯誤訊息)。掃全文會把那行註解判成
        #    違規,逼人刪掉最該留的說明 —— 本檔已有兩處同型的脆弱鎖被修過。
        _init_src = SRC.split("def __init__")[1].split("p = self.get_parameter")[0]
        _init_code = "\n".join(ln.split("#", 1)[0] for ln in _init_src.splitlines())
        assert "25.1406" not in _init_code, \
            "tlat 的預設值又被寫回一個具體座標了"


# ═══════════════════════════════════════════════════════════════════════
#  2. 既有啟動路徑零改變(迴歸鎖)
# ═══════════════════════════════════════════════════════════════════════

LEGACY_CALLERS = ("stot_trial.sh", "stot_kpi_loop.sh",
                  "swarm_tot_up.sh", "swarm_tot_jet_up.sh")


class TestLegacyPathUnchanged:
    @pytest.mark.parametrize("name", LEGACY_CALLERS)
    def test_every_batch_caller_still_passes_the_target_explicitly(self, name):
        """⚠️⚠️ tlat/tlon 的宣告預設值已經改成 NaN。批次呼叫端**必須**自己傳值,
        否則整趟會安靜地停在 STANDBY 等一個永遠不會來的 topic,而外層只會看到
        「逾時」。這條鎖住那四個呼叫端;有人把 -p tlat 拿掉就直接紅。"""
        src = (WSL / name).read_text(encoding="utf-8")
        assert re.search(r"-p\s+tlat:=", src), f"{name} 沒有傳 tlat"
        assert re.search(r"-p\s+tlon:=", src), f"{name} 沒有傳 tlon"

    def test_legacy_node_starts_in_init_not_standby(self):
        s = node(standby=False)
        assert s.standby_mode is False
        assert s.phase == "INIT"

    def test_legacy_csv_name_is_untouched(self):
        """⚠️ 整條離線工具鏈(stot_score / _kpi_parse / make_artifacts.sh /
        stot_mission_anim)都是拿 `tot_<TRIAL>.csv` 去找檔案的。第 1 發**一定**
        要是原檔名,否則所有素材產不出來,而且是靜默的(每一項都自帶 [ -f ])。"""
        s = node(standby=False)
        assert s._csv_base() == "/tmp/tot_test"
        s2 = node(standby=True)
        s2.strike_seq = 1
        assert s2._csv_base() == "/tmp/tot_test"

    def test_legacy_node_refuses_runtime_target(self):
        """⚠️⚠️ 批次航次跑到一半被一則 topic 換掉目標 = 資料全毀。要擋,而且
        要說出為什麼(靜默丟棄會讓人以為訊息沒送到,回頭去查傳輸層)。"""
        s = node(standby=False)
        s._arm_target = _Sink()
        s._on_target(types.SimpleNamespace(data='{"lat": 24.5, "lon": 120.1}'))
        assert s._arm_target.calls == []
        assert "拒收新目標" in s.log.text()
        assert "啟動參數" in s.log.text()

    def test_legacy_node_ends_at_done_not_standby(self):
        """批次航次打完停在 DONE(既有語意),不可以自己回 STANDBY 再接目標。"""
        src_tail = SRC.split('self.phase = "DONE"')[1]
        assert "if self.standby_mode:" in src_tail.split("\n\n")[0], \
            "回 STANDBY 沒有被 standby_mode 守住"


# ═══════════════════════════════════════════════════════════════════════
#  3. 接新目標時的狀態重設 —— 逐項比對
# ═══════════════════════════════════════════════════════════════════════

#: ⚠️⚠️ **手寫一份**,不是從 tot_node import 過來的。從模組讀就變成「拿 A 證明 A」,
#:    有人刪掉一項時兩邊一起改、測試照樣綠。這份清單是人審過的規格。
EXPECTED_RESET_DICTS = {
    # ALIGN
    "slot_of", "slot_id", "arr",
    # PERCH
    "align_ok", "ready", "perch_arr",
    # DIVE / PN
    "released", "strike", "frozen", "alpha", "commit", "thru_dyn",
    # 結案與量測
    "impact", "missed", "miss_why", "miss_pt", "slow_t0",
    "dmin", "hit_alt", "hit_d", "truth_dmin", "truth_hit",
    # 航點鎖存
    "wp_key", "wp_t", "wp_pt", "req_radius", "sent_radius",
    # 紅心地表(換目標 = 換地表)
    "zg", "zg0", "zg_warn", "zg_amsl", "terr_bull", "terr_home",
    # ⭐ 2026-08-14(S4/S5)姿態通道兜底的計數與時戳。
    # ⚠️ 這三個是**這一發的累積**不是當下狀態,所以要清:
    #    att_block/att_block_n 留著會讓第二發一開始就帶著上一發的擋下紀錄;
    #    handback_t 留著會讓第二發的第一次交棒重錨被 1 秒節流吃掉。
    "att_block", "att_block_n", "handback_t",
    # ⭐ T1/T2/T3 的時間協同觀測量。⚠️ 這些是**這一發的累積**,第二發要清:
    #    e_t/tgo_obs 帶著上一發的值會讓第一拍的觀測是假的;
    #    tgo_cache 的時戳更會讓節流邏輯以為剛算過。
    # ⚠️ d_hist 是量測閉合速率的短窗歷史。留著上一發的距離 ⇒ 第一個 tick
    #    會算出跨越兩發的「閉合速率」,而兩發的幾何完全無關。
    "e_t", "tgo_obs", "weave_theta", "tgo_cache", "d_hist",
    # ⚠️ itcg_side 是**鎖存**的繞路方向。留著上一發的值 ⇒ 新目標的第一拍就
    #    往上一發的方向繞,而兩發的幾何完全無關。
    "itcg_e", "itcg_a", "itcg_s", "itcg_side",
    # ⚠️ B1(MPNG):mpng_k0 是用**放行當拍**的 R、t_go 推出來的增益鎖存,
    #    itcg_dbg 是本拍的六欄儀表。兩個都是「這一發的」,留著等於第二發
    #    用第一發的尺度出力、並寫出第一發的 ε_T ——「假資料比缺資料更貴」。
    "mpng_k0", "itcg_dbg",
    # ⚗️ 航向閘門「已放行過」的旗標。留著上一發的值 ⇒ 第二發第一拍就當成
    #    已經放行過,實驗的自變數直接失效。
    "sigma_gate_hit", "sigma_gate_starved", "sigma_wait_t0",
}
EXPECTED_RESET_NONE = {
    "t0", "T_arr", "dive_go", "dive_T", "done_t",
    "ring_t0", "runup_t0", "perch_t0", "perch_T",
}


class TestResetManifest:
    def test_dict_manifest_matches_the_reviewed_list(self):
        assert set(tot_node.RESET_DICTS) == EXPECTED_RESET_DICTS

    def test_none_manifest_matches_the_reviewed_list(self):
        assert set(tot_node.RESET_NONE) == EXPECTED_RESET_NONE

    def test_manifests_have_no_duplicates(self):
        assert len(tot_node.RESET_DICTS) == len(set(tot_node.RESET_DICTS))
        assert len(tot_node.RESET_NONE) == len(set(tot_node.RESET_NONE))

    def test_manifests_do_not_overlap(self):
        assert not (set(tot_node.RESET_DICTS) & set(tot_node.RESET_NONE))

    def test_position_state_is_deliberately_not_reset(self):
        """⚠️ pos/gs/vel/att/home_amsl/home_ll/link/last_v 是**當下狀態**不是
        「這一發的累積」,清掉只會讓第二發從零開始等定位(而且 last_v 是直接
        索引的,清成 {} 會 KeyError)。這條鎖住「別順手多清」。"""
        for name in ("pos", "gs", "vel", "att", "home_amsl", "home_ll",
                     "link", "last_v", "last_v_t", "truth"):
            assert name not in tot_node.RESET_DICTS, name


class TestResetCompleteness:
    def test_every_listed_dict_is_emptied(self):
        s = node()
        s._reset_for_new_target(*TGT2)
        for name in EXPECTED_RESET_DICTS:
            assert getattr(s, name) == {}, f"{name} 沒清乾淨"

    def test_every_listed_timestamp_is_none(self):
        s = node()
        s._reset_for_new_target(*TGT2)
        for name in EXPECTED_RESET_NONE:
            assert getattr(s, name) is None, f"{name} 還留著上一發的時戳"

    def test_stale_dive_T_would_instantly_time_out_the_next_strike(self):
        """⚠️⚠️ 這是「為什麼 dive_T 一定要歸 None」的反證。dive_T 是**絕對牆鐘
        時戳**,留著上一發的值 ⇒ _miss_check 的 (c) 條 `now > dive_T + window`
        在第二發的第一個 tick 就成立 ⇒ 全隊瞬間判逾時 MISS、0/6 結案,
        而現場看起來跟「飛行失敗」一模一樣。"""
        s = node()
        s.dive_go = 1.79e9 - 5000.0
        s.dive_T = 1.79e9 - 4000.0          # 上一發的時戳
        s.released = {i: True for i in s.pos}
        s.impact, s.missed, s.miss_why, s.miss_pt, s.slow_t0 = {}, {}, {}, {}, {}
        s.dmin, s.truth_dmin = {}, {}
        # ⚠️ node() 刻意把每個 RESET_DICTS 塞成**字串**髒值(偵測重設漏網用)。
        #    zg 也在裡面,而 _miss_check 判逾時後會印「高度 …m」,那行會走
        #    _zg() = self.zg.get(i, self.ground_alt) ⇒ 拿到字串就 TypeError。
        #    這條測的是 dive_T 過期,zg 的髒值與它無關,清掉才測得到要測的東西。
        s.zg = {}
        assert s._miss_check(1, 900.0, 1.79e9) is True     # 舊時戳 ⇒ 立刻逾時
        s2 = node()
        s2._reset_for_new_target(*TGT2)
        s2.dive_go = 1.79e9                                # 新的一發剛放行
        assert s2.dive_T is None
        assert s2._miss_check(1, 900.0, 1.79e9 + 1.0) is False

    def test_stale_waypoint_latch_would_keep_flying_the_old_target(self):
        """⚠️⚠️ wp_pt/wp_key 是「航點鎖存」。不清的話 _goto 的
        `moved = prev is None or ... dist_m(prev, (lat,lon)) > move_tol`
        會因為 prev 還在而**不重發 DO_REPOSITION** —— 六架繼續飛上一個紅心的
        航點,零錯誤訊息。這是本輪最容易漏、後果最直接的一項。"""
        s = node()
        assert s.wp_pt and s.wp_key and s.wp_t          # 前提:確實是髒的
        s._reset_for_new_target(*TGT2)
        assert s.wp_pt == {} and s.wp_key == {} and s.wp_t == {}

    def test_speed_latch_is_zeroed_not_emptied(self):
        """⚠️ last_v 是**直接索引**(self.last_v[i]),清成 {} 會在下一次 _goto
        直接 KeyError。要就地歸零 —— 歸零後 `abs(v - last_v) > 0.5` 必成立,
        新的 DO_CHANGE_SPEED 才真的送得出去。"""
        s = node()
        s._reset_for_new_target(*TGT2)
        assert set(s.last_v) == set(range(1, s.n + 1))
        assert all(v == 0.0 for v in s.last_v.values())
        assert all(v == 0.0 for v in s.last_v_t.values())

    def test_geometry_scratch_values_are_restored(self):
        """⚠️⚠️ dive_alt / pushover_r 會被 _reground() 依**上一個紅心的地形**
        就地改寫,miss_far 會被 _clamp_miss_far() 夾寬。留著去打下一個紅心 =
        用錯的地表落差算環半徑 —— 就是那個差 1821 m 的坑的換目標版本。"""
        s = node()
        s.dive_alt = -1869.0                 # 上一發重算出來的
        s.pushover_r = 41000.0
        s._reset_for_new_target(*TGT2)
        assert s.dive_alt == s._dive_alt0 == 5.0
        assert s.pushover_r == s._ring_r0 == 2268.0
        assert s.miss_far == s._miss_far0 == 20000.0
        assert s.dive_pace == s.pace_cfg == 50.0

    def test_one_shot_log_flags_are_rearmed(self):
        """⚠️ 「生效值」那幾行是一次性的。不重置的話第二發**完全沒有生效值**
        進 summary.md —— 事後分不出「機制沒開」和「機制開了但沒用」。"""
        s = node()
        s._reset_for_new_target(*TGT2)
        assert s._miss_cfg_logged is False
        assert s._agl_datum_logged is False
        assert not hasattr(s, "_prox_min")
        assert not hasattr(s, "_prox_t")
        assert s.ring_replan is False

    def test_ring_points_follow_the_new_target(self):
        """slot / 環點 / 穿越點必須跟著新紅心重算,而且真的移動了。"""
        s = node()
        old_slots = list(s.slots)
        s._reset_for_new_target(*TGT2)
        assert s.T == TGT2
        assert s.slots != old_slots
        # 每個 slot 都應該落在距新紅心 r_fan 處
        for sl in s.slots:
            assert tot_node.dist_m(s.T, sl) == pytest.approx(s.r_fan, rel=1e-3)
        for pp in s.pp:
            assert tot_node.dist_m(s.T, pp) == pytest.approx(s.pushover_r,
                                                             rel=1e-3)


# ═══════════════════════════════════════════════════════════════════════
#  4. 座標驗證
# ═══════════════════════════════════════════════════════════════════════

class TestValidateTarget:
    @pytest.mark.parametrize("lat,lon", [
        (25.1406, 121.2325), (25.0777, 119.94315),
        (-90.0, -180.0), (90.0, 180.0),           # 邊界含在內
        (0.0, 121.0), (25.0, 0.0),                # 只有一個是 0 ⇒ 合法
    ])
    def test_accepts_sane_coordinates(self, lat, lon):
        ok, why = TotNode.validate_target(lat, lon)
        assert ok is True, why

    def test_rejects_null_island(self):
        """⚠️⚠️ (0, 0) 是**合法**的經緯度,所以任何範圍檢查都攔不住它 ——
        而它幾乎一定是「欄位沒填 / 解析失敗回 0」的產物。讓六架飛機朝一個
        預設值衝下去是最壞的失敗形狀,必須單獨擋。"""
        ok, why = TotNode.validate_target(0.0, 0.0)
        assert ok is False
        assert "0, 0" in why or "Null Island" in why

    @pytest.mark.parametrize("lat,lon", [
        (90.1, 121.0), (-90.1, 121.0), (1000.0, 0.5),
    ])
    def test_rejects_out_of_range_latitude(self, lat, lon):
        ok, why = TotNode.validate_target(lat, lon)
        assert ok is False and "緯度" in why

    @pytest.mark.parametrize("lat,lon", [
        (25.0, 180.1), (25.0, -180.1), (25.0, 1e9),
    ])
    def test_rejects_out_of_range_longitude(self, lat, lon):
        ok, why = TotNode.validate_target(lat, lon)
        assert ok is False and "經度" in why

    @pytest.mark.parametrize("lat,lon", [
        (float("nan"), 121.0), (25.0, float("nan")),
        (float("inf"), 121.0), (25.0, float("-inf")),
    ])
    def test_rejects_non_finite(self, lat, lon):
        assert TotNode.validate_target(lat, lon)[0] is False

    @pytest.mark.parametrize("lat,lon", [
        ("abc", 121.0), (None, 121.0), ({}, []),
    ])
    def test_rejects_non_numeric(self, lat, lon):
        assert TotNode.validate_target(lat, lon)[0] is False

    def test_predicate_has_no_side_effects(self):
        """⚠️ 驗證是純函式 —— 「查一下」不可以改到節點狀態。"""
        assert TotNode.validate_target.__func__.__qualname__.endswith(
            "validate_target") if hasattr(TotNode.validate_target, "__func__") \
            else True
        s = node()
        before = dict(s.__dict__)
        TotNode.validate_target(0.0, 0.0)
        assert s.__dict__.keys() == before.keys()


# ═══════════════════════════════════════════════════════════════════════
#  5. 訊息解析
# ═══════════════════════════════════════════════════════════════════════

class TestParseTarget:
    def test_json_object(self):
        lat, lon, extra = TotNode.parse_target(
            '{"lat": 25.1406000, "lon": 121.2325000, "src": "aeroplan-rclick"}')
        assert (lat, lon) == (25.1406, 121.2325)
        assert extra["src"] == "aeroplan-rclick"

    def test_json_array(self):
        assert TotNode.parse_target("[25.1406, 121.2325]")[:2] == (25.1406,
                                                                   121.2325)

    def test_bare_csv(self):
        """⚠️ 這個格式是刻意留的:從 Windows 呼過來時 JSON 要穿過
        cmd/PowerShell → wsl.exe → bash → ros2 topic pub 四層引號,逐層跳脫
        是這條路徑最容易靜默壞掉的地方。CSV 形式沒有引號問題。"""
        assert TotNode.parse_target(" 25.1406 , 121.2325 ")[:2] == (25.1406,
                                                                    121.2325)

    def test_integer_looking_values_still_parse_as_float(self):
        """⚠️ 「像整數的浮點數」在本專案害過一次(stot_trial.sh:376 的 tlat/tlon
        沒套 _f(),送 121 會被 ROS2 判成 INTEGER 而整個節點崩掉)。
        這條路徑走 topic,不經過 ROS 參數型別檢查,但仍要保證轉成 float。"""
        lat, lon, _ = TotNode.parse_target('{"lat": 25, "lon": 121}')
        assert isinstance(lat, float) and isinstance(lon, float)
        assert (lat, lon) == (25.0, 121.0)

    @pytest.mark.parametrize("bad", [
        "", "   ", None,
        "{", '{"lat": 25}', '{"lon": 121}',
        "25.1406", "abc,def", '{"lat": "x", "lon": "y"}',
        '"just a string"', "12345",
    ])
    def test_bad_payloads_raise_value_error(self, bad):
        with pytest.raises(ValueError):
            TotNode.parse_target(bad)


# ═══════════════════════════════════════════════════════════════════════
#  6. 接目標的完整路徑(_on_target → _arm_target)
# ═══════════════════════════════════════════════════════════════════════

class TestOnTarget:
    def test_accepts_and_enters_init(self):
        s = node()
        s._on_target(types.SimpleNamespace(
            data='{"lat": 24.5, "lon": 120.1, "src": "aeroplan-rclick"}'))
        assert s.phase == "INIT"
        assert s.T == (24.5, 120.1)
        assert s.strike_seq == 1
        assert s.target_src == "aeroplan-rclick"

    def test_logs_the_actual_coordinates_to_seven_decimals(self):
        """⚠️⚠️ 本專案吃過太多次「參數送出去了但沒生效」的虧(JET_CDP 空字串跑了
        幾十趟、catch_speed 157 被整發拒收)。使用者要能拿這一行去跟自己在地圖上
        點的那一點逐位對帳 —— 對不上就是傳輸層壞了,不是導引壞了。"""
        s = node()
        s._on_target(types.SimpleNamespace(
            data='{"lat": 24.1234567, "lon": 120.7654321}'))
        txt = s.log.text()
        assert "lat=24.1234567" in txt
        assert "lon=120.7654321" in txt
        assert "收到目標" in txt and "生效值" in txt

    def test_rejected_message_says_why_and_shows_the_raw_string(self):
        """⚠️ 靜默丟棄最糟:使用者會以為訊息沒送到,回頭去查傳輸層。"""
        s = node()
        s._on_target(types.SimpleNamespace(data='{"lat": 0.0, "lon": 0.0}'))
        assert s.phase == "STANDBY"
        assert s.strike_seq == 0
        assert "丟棄" in s.log.text()
        assert '{"lat": 0.0, "lon": 0.0}' in s.log.text()

    def test_malformed_message_does_not_change_state(self):
        s = node()
        s._on_target(types.SimpleNamespace(data="這不是座標"))
        assert s.phase == "STANDBY" and s.strike_seq == 0
        assert "解析失敗" in s.log.text()

    def test_mid_strike_target_change_is_refused(self):
        """⚠️⚠️ 打到一半換目標會把 released/commit/dmin 全部清掉,而飛機還在
        朝舊紅心俯衝 —— 指令與狀態瞬間脫節。要改目標請先讓這一發結案。"""
        s = node()
        s.phase = "DIVE"
        s._on_target(types.SimpleNamespace(data="24.5,120.1"))
        assert s.T != (24.5, 120.1)
        assert "拒收新目標" in s.log.text()

    def test_amsl_override_from_message(self):
        """⚠️⚠️ 海上目標一定要釘 0.0:SRTM 在開闊水域沒有資料,回的外插值讓
        六架各報一個高度(實測散布 17 m,而命中門檻只有 10 m)。逐發覆寫,
        不必為了換一個海上目標去重啟節點。"""
        s = node()
        s._on_target(types.SimpleNamespace(
            data='{"lat": 25.0777, "lon": 119.94315, "amsl": 0.0}'))
        assert s.target_amsl == 0.0

    def test_approach_from_override_from_message(self):
        s = node()
        s._on_target(types.SimpleNamespace(
            data='{"lat": 24.5, "lon": 120.1, "approach_from": 42.0}'))
        assert s.approach == pytest.approx(42.0)

    def test_auto_approach_points_the_fan_at_the_swarm(self):
        """⭐ 右鍵任意點時 approach_from=180(紅心正南)幾乎一定是錯的:紅心若在
        蜂群南方,扇形會被擺到紅心的更南邊 ⇒ 六架要先**穿過紅心**才飛得到 slot。
        自動反推 = 由紅心看蜂群質心的方位。"""
        s = node()
        s.auto_approach = True
        # 蜂群擺在新紅心的正東方 12 km
        s.pos = {i: (*at(90.0, 12000.0, origin=TGT2), 3100.0)
                 for i in range(1, s.n + 1)}
        s._on_target(types.SimpleNamespace(data="24.5,120.1"))
        assert s.approach == pytest.approx(90.0, abs=0.5)
        assert "自動反推" in s.log.text()

    def test_auto_approach_without_any_fix_is_loud(self):
        """⚠️ 拿不到定位就沿用預設,但**要大聲** —— 扇形可能擺在錯的一側,
        而那個失敗要飛 20 分鐘才看得出來。"""
        s = node()
        s.auto_approach = True
        s.pos = {}
        s._on_target(types.SimpleNamespace(data="24.5,120.1"))
        assert s.approach == pytest.approx(180.0)
        assert "一台飛機的定位都沒有" in s.log.text()

    def test_missing_fixes_are_called_out_by_id(self):
        """⚠️ len(pos) < n 時 _tick 直接 return —— 沒有逾時、沒有訊息。
        互動路徑上沒有任何人在等,使用者只會看到「按了沒反應」。要點名。"""
        s = node()
        s.pos.pop(3); s.pos.pop(5)
        s._on_target(types.SimpleNamespace(data="24.5,120.1"))
        txt = s.log.text()
        assert "4/6" in txt
        assert "[3, 5]" in txt


# ═══════════════════════════════════════════════════════════════════════
#  7. 可重複性:打完回 STANDBY、能接第二個目標
# ═══════════════════════════════════════════════════════════════════════

def dive_node(standby=True):
    """一個處於 DIVE 段、六架全部命中的節點;幾何/鏈路全部換成樁。"""
    s = node(standby=standby)
    now = 1.79e9
    s.phase = "DIVE"
    # ⚠️ 這個 fixture 是「**第一發進行中**」,不是「還沒打過」。
    #    它直接組出 DIVE 段的狀態、沒有走 _arm_target(那條要 STANDBY + 真訊息),
    #    所以 strike_seq 必須手動補上,否則:
    #      (a) 打完再接一個目標會得到 seq=1(而不是 2);
    #      (b) _csv_base() 在 seq<=1 時回原檔名 ⇒ 第二發看起來會蓋掉第一發,
    #          而實際的實作是好的 —— 錯的是這個樁沒有忠實表示狀態。
    #    這是「假失敗」與「真失敗」最容易混淆的地方:兩者都紅在同一條斷言上。
    s.strike_seq = 1
    s.t0 = now - 900.0
    s.T_arr = now - 600.0
    s.T = TGT
    s.dive_go = now - 100.0
    s.dive_T = now + 300.0
    s.done_t = None
    s.ring_replan = True
    s.released = {i: True for i in s.pos}
    s.impact, s.missed, s.miss_why, s.miss_pt, s.slow_t0 = {}, {}, {}, {}, {}
    s.dmin, s.hit_alt, s.hit_d, s.commit, s.thru_dyn = {}, {}, {}, {}, {}
    s.truth, s.truth_dmin, s.truth_hit = {}, {}, {}
    s.strike, s.frozen, s.alpha = {}, {}, {}
    s.wp_t, s.wp_key, s.wp_pt = {}, {}, {}
    s.zg, s.zg0, s.zg_warn, s.zg_amsl = {}, {}, {}, {}
    s.terr_bull, s.terr_home = {}, {}
    s.slot_id = {i: i - 1 for i in s.pos}
    s.slot_of, s.arr, s.align_ok, s.ready, s.perch_arr = {}, {}, {}, {}, {}
    s.thru = {k: at(0.0, 300.0) for k in range(s.n)}
    s.dive_ring_gate = False
    s.ring_gate_frac = 1.10
    s.dive_angle_max = 0.0
    s.pn_on = False
    s.fc = {}
    s.gate_profile = False
    s.gate_r = 1200.0
    s.vmin, s.vmax = 45.0, 100.0
    s.goto_radius, s.hold_radius, s.dive_radius = 30.0, 400.0, 5.0
    s.line_tol, s.resend_dt = 25.0, 2.0
    s.dive_layer, s.track_frac = 4.0, 0.8
    s.ground_alt = 0.0
    s.dive_pace = 50.0
    s._goto = _Sink()
    s._goto_latched = _Sink()
    s._row = _Sink()
    s._pn_step = _Sink()
    s._pace_of = lambda i, d: 100.0
    s.ring_of = lambda i: 500.0
    s.ring_point_of = lambda i: at(180.0, 500.0)
    s.perch_of = lambda i: 400.0
    s._line_alt = lambda i, d_aim: -50.0
    s._pump = lambda: None
    s._proximity_check = lambda now: None
    # 六架全部就地命中
    for i in s.pos:
        s.pos[i] = (*at(180.0, 5.0), 1.0)
    return s


def _run_to_closeout(s, now=1.79e9):
    tot_node.time.time = lambda: now
    s._tick()
    tot_node.time.time = lambda: now + s.impact_settle + 0.1
    s._tick()


class TestRepeatability:
    def test_standby_node_returns_to_standby_after_closeout(self):
        s = dive_node(standby=True)
        _run_to_closeout(s)
        assert len(s.impact) == s.n
        assert tot_node.CLOSEOUT_SENTINEL in s.log.text()
        assert s.phase == "STANDBY"
        assert "回到待命" in s.log.text()

    def test_batch_node_stops_at_done(self):
        """⚠️ 迴歸鎖:批次航次打完停在 DONE,絕不自己回 STANDBY。"""
        s = dive_node(standby=False)
        _run_to_closeout(s)
        assert s.phase == "DONE"
        assert "回到待命" not in s.log.text()

    def test_second_target_is_accepted_after_closeout(self):
        """⭐ 本任務最容易漏的一項:打完之後真的能再接一個目標。"""
        s = dive_node(standby=True)
        _run_to_closeout(s)
        s._on_target(types.SimpleNamespace(data="24.5,120.1"))
        assert s.phase == "INIT"
        assert s.T == TGT2
        assert s.strike_seq == 2

    def test_second_strike_starts_from_a_clean_slate(self):
        """⚠️⚠️ 第一發的 impact/dmin/dive_T 留到第二發 ⇒ _done_count() 一開始
        就等於 n,第二發在 DIVE 的第一個 tick 就「結案」,而摘要行印的是**第一發
        的數字**。看起來完全正常,實際上第二發從來沒飛。"""
        s = dive_node(standby=True)
        _run_to_closeout(s)
        assert s._done_count() == s.n            # 前提:第一發確實滿了
        s._on_target(types.SimpleNamespace(data="24.5,120.1"))
        assert s._done_count() == 0
        for name in EXPECTED_RESET_DICTS:
            assert getattr(s, name) == {}, name
        for name in EXPECTED_RESET_NONE:
            assert getattr(s, name) is None, name

    def test_second_strike_csv_does_not_overwrite_the_first(self):
        """⚠️ 「覆蓋上一趟的證據」在本專案是明文禁止的(見 CMD_LOG_DIR 的教訓:
        cmdlog 用 "w" 開檔,每一趟都把上一趟蓋掉,P91 的資料是手動備份才留下的)。"""
        s = dive_node(standby=True)
        first = s._csv_base()
        _run_to_closeout(s)
        s._on_target(types.SimpleNamespace(data="24.5,120.1"))
        assert s._csv_base() != first
        assert s._csv_base().endswith("_s2")

    def test_three_strikes_keep_rolling(self):
        s = node()
        for k, base in ((1, "/tmp/tot_test"), (2, "/tmp/tot_test_s2"),
                        (3, "/tmp/tot_test_s3")):
            s.strike_seq = k
            assert s._csv_base() == base

    def test_disarm_is_loud_about_what_it_stops_doing(self):
        """⚠️ 「回待命」= 停止對任何機體下令。已判 MISS 的機體會維持盤旋指令,
        這件事要講出來,否則現場會以為節點掛了。"""
        s = dive_node(standby=True)
        _run_to_closeout(s)
        txt = s.log.text()
        assert "不再對任何機體下令" in txt
        assert tot_node.TARGET_TOPIC in txt


# ═══════════════════════════════════════════════════════════════════════
#  8. STANDBY 期間的行為:不下令、但照常抽鏈路
# ═══════════════════════════════════════════════════════════════════════

class TestStandbyBehaviour:
    def test_tick_pumps_but_issues_no_commands(self):
        """⚠️⚠️ pymavlink 的連線不抽就會在 MAVProxy 端累積背壓(本專案量過
        ~1 MB),等真的收到目標時讀到的是幾分鐘前的位置,而且完全沒有錯誤訊息。
        所以 STANDBY 不是「什麼都不做」,是「不下令但照常收」。"""
        s = node()
        pumped = []
        s._pump = lambda: pumped.append(1)
        s._goto = _Sink()
        s._goto_latched = _Sink()
        tot_node.time.time = lambda: 1.79e9
        s._tick()
        assert pumped == [1]
        assert s._goto.calls == [] and s._goto_latched.calls == []
        assert s.phase == "STANDBY"

    def test_pump_is_called_before_the_standby_guard(self):
        """結構鎖:_pump() 必須在 STANDBY 檢查之前。順序反了就不抽鏈路。"""
        body = SRC.split("def _tick(self):")[1].split("def ")[0]
        assert body.index("self._pump()") < body.index('self.phase == "STANDBY"')

    def test_status_heartbeat_is_throttled_to_1hz(self):
        s = node()
        pubs = []
        s.status_pub = types.SimpleNamespace(publish=lambda m: pubs.append(m))
        t = [1.79e9]
        tot_node.time.time = lambda: t[0]
        s._standby_tick()
        s._standby_tick()
        assert len(pubs) == 1, "20 Hz 的待命心跳會把 log/topic 洗掉"
        t[0] += 1.5
        s._standby_tick()
        assert len(pubs) == 2

    def test_status_says_standby_and_how_to_send_a_target(self):
        import json
        s = node()
        pubs = []
        s.status_pub = types.SimpleNamespace(publish=lambda m: pubs.append(m))
        tot_node.time.time = lambda: 1.79e9
        s._standby_tick()
        d = json.loads(pubs[0].data)
        assert d["phase"] == "STANDBY"
        assert d["target_topic"] == tot_node.TARGET_TOPIC
        assert d["n_pos"] == s.n and d["ready_for_target"] is True

    def test_gate_publishes_valid_json_with_null_target(self):
        """⚠️ 待命時 self.T 是 (nan, nan),而 json.dumps(nan) 產生 `NaN` ——
        **不是合法 JSON**,JS 的 JSON.parse 會整包丟掉(AeroPlan 端就瞎了)。"""
        import json
        s = node()
        pubs = []
        s.gate_pub = types.SimpleNamespace(publish=lambda m: pubs.append(m))
        del s._publish_gate                     # 用真的那份
        TotNode._publish_gate(s)
        assert "NaN" not in pubs[0].data
        d = json.loads(pubs[0].data)
        assert d["standby"] is True and d["target"] is None

    def test_terrain_probe_is_skipped_while_standing_by(self):
        """⚠️ 待命時 T 是 (nan, nan),int(nan) 會拋 ValueError —— 而那個
        try/except 會連同一圈的 HOME_POSITION 請求一起吞掉。直接不問。"""
        s = node()
        del s._terrain_probe
        s.link = {1: types.SimpleNamespace(mav=None)}     # 碰到就會 AttributeError
        TotNode._terrain_probe(s)                          # 不該炸

    def test_waiting_for_fixes_complains_every_30s(self):
        """⚠️⚠️ `if len(self.pos) < self.n: return` 是**沒有逾時、沒有訊息**的
        等待。互動路徑上沒有人在等,使用者只會看到「按了沒反應」。"""
        s = node()
        s.phase = "INIT"
        s._pump = lambda: None
        s.pos.pop(2)
        t = [1.79e9]
        tot_node.time.time = lambda: t[0]
        s._tick()
        assert "等機體定位中" in s.log.text()
        n1 = len(s.log.lines)
        for _ in range(20):
            t[0] += 1.0
            s._tick()
        assert len(s.log.lines) == n1, "抱怨沒有節流,會把 log 洗掉"
        t[0] += 31.0
        s._tick()
        assert len(s.log.lines) == n1 + 1

    def test_target_topic_is_not_latched(self):
        """⚠️⚠️ 目標是**命令**不是狀態。latch(TRANSIENT_LOCAL)之下,下一個
        開機的 tot_node 一連上就會收到上一場的目標,在沒有人按任何按鈕的情況下
        自己開打 —— 這是本專案最不能容忍的一類失敗。"""
        assert "TARGET_QOS" in SRC
        blk = SRC.split("TARGET_QOS = QoSProfile(")[1].split(")")[0]
        assert "VOLATILE" in blk
        assert "TRANSIENT_LOCAL" not in blk
        assert "RELIABLE" in blk


# ═══════════════════════════════════════════════════════════════════════
#  9. swarm_ros2_up.sh —— 拉起 tot_node 的接線
# ═══════════════════════════════════════════════════════════════════════

def _sh_default(src, var):
    """從 `VAR="$(_f "${X:-預設}")"` 或 `VAR="${X:-預設}"` 撈出預設值。"""
    m = re.search(r'^%s="(?:\$\(_f )?"?\$\{[A-Z_]+:-([^}"]*)\}' % re.escape(var),
                  src, re.M)
    assert m, f"{var} 找不到"
    return m.group(1)


class TestSwarmRos2Up:
    def test_starts_tot_node_in_standby(self):
        assert "ros2 run sar_swarm tot_node" in UP_SH
        assert "-p tlat:=.nan -p tlon:=.nan" in UP_SH

    def test_nan_is_written_the_yaml_way(self):
        """⚠️ ROS2 的 YAML:NaN 要寫 `.nan`。寫成 `nan` 會被解析成**字串**,
        參數型別檢查當場拋 InvalidParameterTypeException(本專案踩過)。"""
        assert "tlat:=nan" not in UP_SH and "tlon:=nan" not in UP_SH
        assert "target_amsl:=\"$TOT_TARGET_AMSL\"" in UP_SH
        assert _sh_default(UP_SH, "TOT_TARGET_AMSL") == ".nan"

    def test_float_params_go_through_the_decimal_point_helper(self):
        """⚠️ ROS 2 把 "200" 判成 INTEGER 而參數宣告是 DOUBLE ⇒ 節點啟動即崩。
        stot_trial.sh:92 的 _f() 要照抄過來,而且真的用在每一個 float 上。"""
        assert '_f() { case "$1" in *.*|*e*|*E*)' in UP_SH
        for var in ("TOT_R_FAN", "TOT_PERCH_ALT", "TOT_DIVE_ANGLE",
                    "TOT_VMIN", "TOT_VMAX", "TOT_MISS_FAR_M", "TOT_GATE_R"):
            assert re.search(r'^%s="\$\(_f ' % var, UP_SH, re.M), var

    def test_uses_its_own_fanout_port(self):
        """⚠️ MAVProxy 的分流只餵「先連上的那個 client」,所以 tot_node 要獨佔一路。

        ⚠️⚠️ 2026-08-07 更正:這條原本要求 **6960**,理由只寫了「不要和 AeroPlan
           的 6760 共用」—— 那個推理是對的,但**盤點漏了一個消費者**:
           6960+10i 同時就是編隊層的 WP 鏈路(master_node 連 6960、
           backup_node 連 6970),而且 launch 檔讓它們比 tot_node 早起。
           於是待命 tot_node 永遠是第二 client,收到 0 bytes。
           實測位元組計數與完整推導見 tests/test_link_port_exclusivity.py。
        """
        base = _sh_default(UP_SH, "TOT_LINK_BASE")
        assert base == "6860"
        assert base != "6760", "那是 AeroPlan 的"
        assert base != "6960", "那是編隊層的(master_node / backup_node)"

    #: ⚠️⚠️ run_west.sh **覆寫**掉的參數 —— 這些的真相源不是 stot_trial.sh。
    #:
    #: 2026-08-07 事故:本測試原本一律比對 stot_trial.sh 的預設,而
    #: **批次航次實際跑的是 run_west.sh 覆蓋後的值**。於是這條鎖一路綠燈,
    #: 卻鎖住了一組噴射機改裝後沒有任何一趟航次用過的泡棉機基準 ——
    #: 互動路徑的巡航因此只有 82 m/s(應為 ~157)。
    #: ⇒ run_west.sh 有 export 的,比 run_west.sh;其餘才比 stot_trial.sh。
    #: 逐項的理由與來源行號見 tests/test_jet_config_parity.py。
    WEST_OVERRIDES = ("VMIN", "PERCH_CLIMB_V", "WEAVE_PERIOD",
                      "PN_VMAX", "DIVE_SPEED",
                      # 3100 m 噴射機巡航剖面(一整組耦合的幾何,見
                      # tests/test_jet_config_parity.py::TestRingGeometryIsTheJetProfile)
                      "R_FAN", "REACH", "PERCH_ALT", "TALT",
                      "DIVE_ANGLE", "GATE_R")

    @pytest.mark.parametrize("up_var,trial_var", [
        ("TOT_R_FAN", "R_FAN"), ("TOT_HALF_ANGLE", "HALF_ANGLE"),
        # ⚠️ TOT_VMAX **刻意不在這裡** —— 它是兩邊唯一都不照抄的參數,
        #    由 test_vmax_is_deliberately_not_mirrored 單獨鎖住並說明理由。
        ("TOT_VMIN", "VMIN"), ("TOT_MARGIN", "MARGIN"),
        ("TOT_REACH", "REACH"), ("TOT_LAYER_M", "LAYER_M"),
        ("TOT_PERCH_REACH", "PERCH_REACH"),
        ("TOT_IMPACT_REACH", "IMPACT_REACH"),
        ("TOT_OVERSHOOT", "OVERSHOOT"), ("TOT_SWITCH_R", "SWITCH_R"),
        ("TOT_DIVE_SPEED", "DIVE_SPEED"), ("TOT_DIVE_RADIUS", "DIVE_RADIUS"),
        ("TOT_PERCH_ALT", "PERCH_ALT"), ("TOT_TALT", "TALT"),
        ("TOT_DIVE_ANGLE", "DIVE_ANGLE"), ("TOT_ALIGN_TOL", "ALIGN_TOL"),
        ("TOT_TRACK_FRAC", "TRACK_FRAC"), ("TOT_DIVE_PACE", "DIVE_PACE"),
        ("TOT_PERCH_BAND", "PERCH_BAND"), ("TOT_PERCH_SETTLE", "PERCH_SETTLE"),
        ("TOT_LINE_TOL", "LINE_TOL"), ("TOT_RESEND_DT", "RESEND_DT"),
        ("TOT_HDEM_TC", "HDEM_TC"), ("TOT_GATE_R", "GATE_R"),
        ("TOT_N_LAT", "N_LAT"), ("TOT_N_VERT", "N_VERT"),
        ("TOT_PN_LEAD", "PN_LEAD"), ("TOT_PN_VMAX", "PN_VMAX"),
        ("TOT_FREEZE_R", "FREEZE_R"), ("TOT_FREEZE_TGO", "FREEZE_TGO"),
        ("TOT_MISS_FAR_M", "MISS_FAR_M"), ("TOT_MISS_FAR_K", "MISS_FAR_K"),
        ("TOT_MISS_WINDOW_S", "MISS_WINDOW_S"),
        ("TOT_MISS_SLOW_GS", "MISS_SLOW_GS"),
        ("TOT_MISS_SLOW_SEC", "MISS_SLOW_SEC"),
        ("TOT_DIVE_PITCH_MIN", "DIVE_PITCH_MIN"),
        ("TOT_DIVE_PITCH_MAX", "DIVE_PITCH_MAX"),
        ("TOT_DIVE_THR_MAX", "DIVE_THR_MAX"),
        ("TOT_PN_THR_TRIM", "PN_THR_TRIM"),
    ])
    def test_geometry_defaults_match_stot_trial(self, up_var, trial_var):
        """⚠️⚠️ 這些值是抄過來的,不是編的。抄一份就會漂移,所以用測試釘住。

        真相源分兩種:
          * run_west.sh 有 export 的 → 以它為準(那是批次航次的生效值)
          * 其餘 → stot_trial.sh(每一行都有實測血淚註解)
        """
        a = float(_sh_default(UP_SH, up_var))
        if trial_var in self.WEST_OVERRIDES:
            m = re.search(r'^export %s="\$\{%s:-([-\d.]+)\}"' % (trial_var, trial_var),
                          WEST_SH, re.M)
            assert m, "%s 列在 WEST_OVERRIDES 但 run_west.sh 沒有 export 它" % trial_var
            b = float(m.group(1))
            assert a == b, ("%s=%s 與 run_west.sh 的 %s=%s 不一致 —— "
                            "互動路徑又漂回泡棉機基準了" % (up_var, a, trial_var, b))
            return
        b = float(_sh_default(TRIAL_SH, trial_var))
        assert a == b, f"{up_var}={a} 與 stot_trial.sh 的 {trial_var}={b} 不一致"

    def test_vmax_is_deliberately_not_mirrored(self):
        """⚠️ VMAX 是**唯一**兩邊都不照抄的:run_west.sh 沒有 export 它,
        而互動路徑刻意設成 175(⇒ 規劃巡航 0.9×175 = 157.5,使用者定調)。
        批次路徑仍是 100 —— 那是因為批次有獨立的巡航段(jet_show_ros2.py),
        tot_node 只負責末端;互動路徑沒有巡航段,ALIGN 要自己飛完 50 km。"""
        assert not re.search(r'^export VMAX=', WEST_SH, re.M)
        # ⚠️ 150 ⇒ v_plan = 0.9×150 = **135 EAS** = 機體設計巡航
        #    (run_west.sh:68「157 m/s 地速 @3100 m = EAS 135」)。
        #    數值本身的理由與 EAS/地速的坑見 tests/test_jet_config_parity.py。
        assert float(_sh_default(UP_SH, "TOT_VMAX")) == 150.0
        assert float(_sh_default(TRIAL_SH, "VMAX")) == 100.0

    def test_dive_pitch_min_is_negative(self):
        """⚠️⚠️ tot_node 的放開條件是 `if dive_pitch_min < 0.0:`。設 0.0 等於讓
        整段「放開低頭/抬頭權限」的程式碼永遠不執行,PTCH_LIM_MIN_DEG 停在巡航段
        的 −20 ⇒ 實際航跡角上限只有 ~16°(實測 DV1 從紅心上空 2.3–2.7 km 掠過)。"""
        assert float(_sh_default(UP_SH, "TOT_DIVE_PITCH_MIN")) < 0.0

    def test_cleanup_covers_tot_node(self):
        # ⚠️ 2026-08-07 收緊:pattern 從 '[t]ot_node' 改成 '[t]ot_node --ros-args'。
        #    舊的會打中任何「命令列提到 tot_node」的行程(實測被 log 監看指令
        #    打中而讓整層拒絕啟動)。括號寫法仍要保留 —— 那是防自我匹配的。
        assert "pgrep -f '[t]ot_node --ros-args'" in UP_SH
        assert "pgrep -f '[t]ot_node'" not in UP_SH, "又放寬回去了"

    def test_cleanup_pattern_cannot_match_itself(self):
        """⚠️ 命令列裡出現裸的進程名 ⇒ pkill/pgrep 會匹配到自己。括號寫法
        `[t]ot_node` 在 cmdline 裡是字面的 `[t]ot_node`,當正規式不匹配它自己。

        ⚠️⚠️ 只檢查**會被執行的行**。這個檔案的註解裡刻意保留了歷史事故的原文
        (:266 那句「`pkill -x tot_node` 把正在俯衝的 R451 殺在距共同命中時刻
        2.5 秒」),那是要留給下一個人看的反面教材 —— 拿正規式掃全文會把它判成
        違規,逼人刪掉最有價值的那行註解。本 repo 已經踩過同型的脆弱鎖一次
        (stot_repeat.sh 的註解引用舊寫法,害 test_repeat_shell_block 恆紅)。
        """
        code_only = "\n".join(
            ln.split("#", 1)[0] for ln in UP_SH.splitlines()
        )
        for m in re.finditer(r"p(?:kill|grep)[^;\n]*?tot_node", code_only):
            assert "[t]ot_node" in m.group(0), m.group(0)

    def test_trap_kills_the_process_group_not_just_the_wrapper(self):
        """⚠️⚠️ `ros2 run` 是 CLI wrapper,節點是它的**子行程**。只 kill wrapper
        的話節點還活著,會跟下一輪搶控制,而且沒有任何錯誤訊息
        (stot_trial.sh:421-431 記著這個坑)。"""
        trap = [ln for ln in UP_SH.splitlines() if ln.startswith("trap ")]
        assert trap, "trap 不見了"
        assert any('kill -TERM -"$TOT_PGID"' in ln for ln in trap)
        assert "setsid nohup ros2 run sar_swarm tot_node" in UP_SH
        assert 'ps -o pgid= -p "$TOT_PID"' in UP_SH, \
            "PGID 要讀回來,不要假設等於 $!"

    def test_refuses_to_start_over_a_running_batch_trial(self):
        """⚠️⚠️ 無差別 pkill 會把正在俯衝的批次航次殺掉(stot_trial.sh:414 實測:
        R451 被殺在距共同命中 2.5 秒,整趟被記成 timeout)。要分辨,不要替
        使用者決定。"""
        assert '*"tlat:=.nan"*' in UP_SH
        assert "TOT_FORCE" in UP_SH

    def test_verifies_standby_actually_happened(self):
        """⚠️⚠️ 「起了但沒生效」是本專案代價最高的一類錯。背景啟動看起來都一樣
        成功 —— 要讀回來驗。"""
        assert tot_node.STANDBY_SENTINEL in UP_SH, \
            "swarm_ros2_up.sh 的驗證式與 tot_node 的 STANDBY_SENTINEL 不一致"
        assert tot_node.STANDBY_SENTINEL in SRC

    def test_sentinel_is_ascii_bracket_free(self):
        """⚠️ 全形「（」與半形「(」在終端機裡幾乎看不出差別,而 grep 失配的後果
        是「明明待命成功卻報失敗」。哨兵不准帶括號。"""
        for ch in "()（）":
            assert ch not in tot_node.STANDBY_SENTINEL

    def test_sitl_layer_is_never_touched(self):
        """⚠️⚠️ 鐵則:常駐服務路徑**不可以**拆 SITL(那是批次路徑的行為:
        stot_repeat.sh 每趟 swarm_sitl_down + up + sleep 150)。"""
        for forbidden in ("swarm_sitl_down.sh", "swarm_sitl_up.sh",
                          "pkill -x arduplane", "pkill -f mavproxy"):
            assert forbidden not in UP_SH.split("# 用法：")[-1] or \
                forbidden == "swarm_sitl_up.sh", forbidden
        # swarm_sitl_up.sh 只准出現在說明文字裡,不准被執行
        for ln in UP_SH.splitlines():
            s = ln.strip()
            if s.startswith("#") or s.startswith("echo"):
                continue
            assert "swarm_sitl_down.sh" not in s
            assert "bash swarm_sitl_up.sh" not in s

    def test_tot_node_can_be_disabled(self):
        """⚠️ 常駐服務要能只起編隊層(既有用法不能被綁死)。"""
        assert _sh_default(UP_SH, "TOT_ENABLE") == "1"
        assert 'if [ "$TOT_ENABLE" = "1" ]; then' in UP_SH

    def test_no_crlf(self):
        assert "\r" not in UP_SH


class TestStaleTotNodeGuardDoesNotSelfMatch:
    """⚠️ 2026-08-07:殘留節點的偵測把**我自己的 log 監看指令**當成批次航次節點。

    `pgrep -f 'tot_node'` 比對整條命令列,所以
        tail -F … | grep -E '…|✗ tot_node|…'
    會中；它沒有 `tlat:=.nan` ⇒ 被歸成「帶目標的批次節點」⇒ **整層拒絕啟動**,
    而錯誤訊息叫人去 tail trial_*.log(那裡什麼都沒有)。
    """

    @staticmethod
    def _matches(cmdline):
        """複刻腳本的判定:pattern 要中,且命令列要真的有 --ros-args。"""
        return "tot_node --ros-args" in cmdline and "--ros-args" in cmdline

    def test_pattern_is_tightened_in_the_script(self):
        assert "pgrep -f '[t]ot_node --ros-args'" in UP_SH

    def test_loop_double_checks_ros_args(self):
        i = UP_SH.index("_tot_idle=\"\"; _tot_batch=\"\"")
        body = UP_SH[i:i + 900]
        assert '*"--ros-args"*' in body
        assert "continue" in body

    @pytest.mark.parametrize("cmdline", [
        # 實際打中的那一條
        "bash -c until grep -qE 'ready_for_target|\u2717 tot_node' /tmp/x.log; do sleep 3; done",
        "tail -F /tmp/a.log | grep --line-buffered '\u2717 tot_node'",
        "vim scripts/wsl/sar_swarm_patches/tot_node.py",
        "python3 -c print('tot_node')",
    ])
    def test_log_watchers_are_not_mistaken_for_nodes(self, cmdline):
        assert not self._matches(cmdline), cmdline

    @pytest.mark.parametrize("cmdline", [
        "/usr/bin/python3 /opt/ros/humble/bin/ros2 run sar_swarm tot_node --ros-args -p n:=2 -p tlat:=.nan",
        "/usr/bin/python3 /home/u/fixwing_ws/install/sar_swarm/lib/sar_swarm/tot_node --ros-args -p n:=6 -p tlat:=25.0",
    ])
    def test_real_nodes_still_match(self, cmdline):
        assert self._matches(cmdline), cmdline

    def test_classification_still_separates_idle_from_batch(self):
        idle = ("ros2 run sar_swarm tot_node --ros-args -p tlat:=.nan -p tlon:=.nan")
        batch = ("ros2 run sar_swarm tot_node --ros-args -p tlat:=25.0777 -p tlon:=120.7")
        assert self._matches(idle) and "tlat:=.nan" in idle
        assert self._matches(batch) and "tlat:=.nan" not in batch

    def test_the_dont_kill_indiscriminately_lesson_is_kept(self):
        """⚠️ 無差別 pkill 曾把正在俯衝的 R451 殺在共同命中前 2.5 秒。
        收緊 pattern 不可以順手改成無差別殺。"""
        assert "R451" in UP_SH
        assert "拒絕啟動" in UP_SH
