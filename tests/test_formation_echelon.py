# -*- coding: utf-8 -*-
"""n=2 的雁型編隊與編隊生效值(2026-08-24)。

═══════════════════════════════════════════════════════════════════════════
 這個檔在守什麼
═══════════════════════════════════════════════════════════════════════════
2026-08-24 實測發現三件互相獨立、但合起來讓「雁型」在 n=2 **結構上不可能
出現**的事:

1. **n=2 不生 wingman_node。** `_roles(2)` 回 (master=1, wingmen=[], backup=2)。
   而實作雁型的 `_v_target()` 整支在 `wingman_node.py` 裡 ——
   `v_spacing_m` / `v_angle_deg` 也都是它的參數。
   ⇒ n=2 之下那三顆從來不會被讀。

2. **備長的 slot 方位寫死 180.0**(`backup_node._fly_standby`)——
   正後方、零側偏。實測 13 趟 n=2:沿航向 -404~-957 m(在後方),
   垂航向分量 29-887 m 只是漂移不是指令。

3. **互動路徑一個編隊參數都沒設** ⇒ 吃 `swarm_ros2_up.sh` 的泡棉裸預設
   NOMINAL=14 / WAIT=10 / CATCH=20,而飛控 `AIRSPEED_MIN` 是 **25**。
   ArduPlane 的 DO_CHANGE_SPEED 只在 AIRSPEED_MIN <= 目標 <= AIRSPEED_MAX
   時才接受,區間外回 MAV_RESULT=4、**整發丟掉**。而直連送出只知道
   「送出去了」,收不收要靠 COMMAND_ACK 側錄 ⇒ **完全靜默**。
   ⇒ 互動路徑上編隊層送出的每一發速度指令都被拒收。
     這就是「編隊成形從未被證明」的完整解答。

═══════════════════════════════════════════════════════════════════════════
 紅線
═══════════════════════════════════════════════════════════════════════════
A. **批次路徑一個位元都不能變** —— stot_trial.sh 現在 96% 進俯衝,
   共用組態抽出來之後 n=6 的值必須與它原本算的完全相同。
B. **v_tail_angle_deg=0 必須是逐位元舊行為**(正後方)。
C. 編隊速度必須在飛控接受的區間內,否則整條通道靜默失效。
D. 側偏必須被爬升相位閘門擋著 —— 2.5 km 間距的側偏是 1057 m,
   爬升途中橫移 1 km 正是 wingman 實測掉 30% 爬升率的原因。
"""
import math
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WSL = REPO / "scripts" / "wsl"
FC = WSL / "formation_config.sh"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_tot_standby import TotNode, FakeLogger  # noqa: E402,F401  (裝 ROS 樁的副作用)


def _install_backup_stubs():
    """backup_node 比 tot_node 多要三樣東西的樁。

    ⚠️ 只在真的缺的時候補 —— 有 ROS 環境時要用真的那份,免得樁把行為蓋掉。
    ⚠️ `sar_swarm` 是**套件別名**:節點裡寫 `from sar_swarm.geodesy import ...`,
       而測試把 patches 目錄放在 sys.path 上 ⇒ `geodesy` 是頂層模組、
       `sar_swarm.geodesy` 不存在。這裡把 patches 目錄註冊成 sar_swarm 的
       搜尋路徑,讓兩種寫法指向**同一份檔案**(不是複製一份樁)。
    """
    import types
    patches = str(REPO / "scripts" / "wsl" / "sar_swarm_patches")
    if patches not in sys.path:
        sys.path.insert(0, patches)
    try:
        import geographic_msgs.msg  # noqa: F401
    except ImportError:
        gm = types.ModuleType("geographic_msgs")
        gmm = types.ModuleType("geographic_msgs.msg")

        class _GeoPoseStamped:
            def __init__(self, *a, **k):
                pass

        gmm.GeoPoseStamped = _GeoPoseStamped
        gm.msg = gmm
        sys.modules.setdefault("geographic_msgs", gm)
        sys.modules.setdefault("geographic_msgs.msg", gmm)
    if "sar_swarm" not in sys.modules:
        pkg = types.ModuleType("sar_swarm")
        pkg.__path__ = [patches]          # ⇒ sar_swarm.X 解析到同一份檔案
        sys.modules["sar_swarm"] = pkg
    # master_role 還要 rclpy.qos 的三個名字,而 test_tot_standby 的樁只補了兩個。
    # ⚠️ 補在既有的樁上而不是另做一份 —— 兩份樁會各自漂移。
    import rclpy.qos as _q
    if not hasattr(_q, "QoSPresetProfiles"):
        class _Preset:
            class _P:
                def __init__(self, **k):
                    self.kw = k
            SENSOR_DATA = SYSTEM_DEFAULT = SERVICES_DEFAULT = _P()

            @staticmethod
            def get_from_short_key(_k):
                return _Preset._P()
        _q.QoSPresetProfiles = _Preset
    for _n in ("LivelinessPolicy", "QoSLivelinessPolicy", "QoSDurabilityPolicy",
               "QoSHistoryPolicy", "QoSReliabilityPolicy"):
        if not hasattr(_q, _n):
            setattr(_q, _n, _q.ReliabilityPolicy)


_install_backup_stubs()

#: 飛控接受 DO_CHANGE_SPEED 的區間。⚠️ 與 swarm_sitl_up.sh 的 `AIRSPEED_MIN 25`
#: 逐字對齊 —— 這個數字變了,下面那條速度測試就必須跟著變,不可以放寬。
#:
#: ⚠️⚠️ 2026-08-28:AIRSPEED_MAX 230 → **110**。230 是**低阻力改裝**剖面的上限;
#:    主線換成原廠 plane-jet(AIRFRAME=stockjet)之後,swarm_sitl_up.sh 的
#:    stockjet 分支寫的是 110。把 230 留在這裡會讓這條「速度在飛控區間內」
#:    的紅線對 106–230 那一整段**完全失去鑑別力** —— 而編隊層的舊值
#:    (FORM_V 135 / FORM_CATCH 157)正好落在那一段裡。
AIRSPEED_MIN = 25.0
AIRSPEED_MAX = 110.0

#: ⭐⭐ 機體**飛得到**的平飛頂速(EAS)。與上面那個是**兩件事**:
#:    AIRSPEED_MAX 是「送得出去嗎」(超過即 MAV_RESULT=4 整發拒收),
#:    這個是「飛得到嗎」(超過**不會**被拒收,只是永遠達不到 —— 油門飽和、
#:    SEP≈0、零告警,比拒收更難診斷)。
#:    2026-08-27 實測:節流閥釘住、60 s 穩態取樣、sd 0.013。
AERO_VMAX_EAS = 105.5


def sh(script):
    """在 WSL bash 裡跑一段 shell,回傳 stdout。"""
    r = subprocess.run(
        ["wsl.exe", "-e", "bash", "-lc", script],
        capture_output=True, text=True, encoding="utf-8", errors="replace")
    return r.stdout or ""


def cfg(**env):
    """source formation_config.sh,回傳最終生效值 dict。

    ⚠️ 先賦值再 source,**不可以**用 `VAR=x . file` 的前綴式 —— bash 對
       special builtin 的賦值前綴是暫時的,`.` 回來就還原,而由它推出的
       其他值會留下來 ⇒ 一組互相矛盾的值。這正是本檔測的其中一條。
    """
    pre = "; ".join('%s=%s' % (k, v) for k, v in env.items())
    out = sh("cd /mnt/c/Users/joy46/Documents/Projects/aeroplan-studio/scripts/wsl"
             " && { %s; } ; . ./formation_config.sh >/dev/null 2>&1"
             " && for v in $FORMATION_VARS; do eval \"echo $v=\\$$v\"; done" % pre)
    d = {}
    for ln in out.splitlines():
        if "=" in ln:
            k, _, v = ln.partition("=")
            d[k.strip()] = v.strip()
    return d


@pytest.fixture(scope="module")
def wsl_ok():
    if not sh("echo ok").strip().startswith("ok"):
        pytest.skip("沒有 WSL —— 這道守衛這次沒被執行(skip 不是通過)")
    return True


# ══════════════════════════════════════════════════════════════════════
#  紅線 A:批次路徑逐位元不變
# ══════════════════════════════════════════════════════════════════════

def test_batch_values_are_unchanged(wsl_ok):
    """⭐⭐ n=6 的生效值必須與**當前剖面的設計點**完全相同。

    ⚠️ 批次路徑目前 96% 進俯衝。抽共用組態是為了修互動路徑,
       絕對不可以順手改動這條會動的路徑 —— 而那種改動不會報錯,
       只會讓下一批的數字悄悄變成另一個操作點。

    ⭐⭐⭐ 2026-08-28:主線剖面換成原廠 plane-jet(AIRFRAME=stockjet),
    三個速度檔位**不再由換算式推導**,而是釘死的宣告值:

        FORM_V     = 80   使用者定調的設計巡航(EAS)
        FORM_CATCH = 95   ≤ AIRSPEED_MAX 110 且 ≤ 實測天花板 105.5,> 巡航
        FORM_WAIT  = 55   0.69x 巡航,遠高於實測下限 25.6 與 AIRSPEED_MIN 25
        FORM_LMAX  = round(107 * 0.16 * 30 * 0.7) = 360
        V_TAIL     = 4000 * 1.5 = 6000(幾何**不隨速度縮**)

    ⚠️ 為什麼不再用 1.16 / 0.67 去推:那兩個比例是從 lowdrag「加速側 +21 /
       減速側 −45」的不對稱權限算出來的,而那個不對稱來自 T/W 0.274。
       stockjet 的 T/W 是 1.4286,權限分佈完全不同 ⇒ 照抄比例會得到一組
       沒有依據的數字。
    """
    d = cfg(N=6)
    assert d.get("FORM_V") == "80", d
    assert d.get("FORM_CATCH") == "95", d
    assert d.get("FORM_WAIT") == "55", d
    assert d.get("FORM_CATCH_R") == "360", d
    assert d.get("FORM_FORMED_R") == "200", d
    assert d.get("V_SPACING") == "4000", d
    assert d.get("V_ANGLE") == "35", d
    assert d.get("V_TAIL") == "6000", d


def test_the_lowdrag_profile_still_reproduces_the_old_values(wsl_ok):
    """⭐⭐⭐ **新鎖(2026-08-28)**:舊剖面必須逐格重現 2026-08-27 之前的值。

    保留 `AIRFRAME=lowdrag` 的唯一理由就是**可重現**。它一旦漂掉,
    2026-08-27 之前那幾十趟航次就再也跑不出來,而它們是目前唯一有
    完整 KPI 的資料集。

    這裡的期望值是從 stot_trial.sh:376-412 的算式**手算**出來的:
        FORM_V     = round(157 * 0.857) = 135
        FORM_CATCH = round(135 * 1.16)  = 157
        FORM_WAIT  = round(135 * 0.67)  = 90
        FORM_LMAX  = round(157 * 0.16 * 30 * 0.7) = 528

    ⭐ 這條會紅的生產改動:動 formation_config.sh 的 lowdrag 分支
       (包括「順手統一一下換算式」)。
    """
    d = cfg(N=6, AIRFRAME="lowdrag")
    assert d.get("FORM_V") == "135", d
    assert d.get("FORM_CATCH") == "157", d
    assert d.get("FORM_WAIT") == "90", d
    assert d.get("FORM_CATCH_R") == "528", d


def test_the_two_profiles_do_not_give_the_same_answer(wsl_ok):
    """⭐ 先證明剖面這個自由度**真的被用到**了。

    兩個剖面若給出同一組數字,上面兩條測試其中一條就是在測空氣 ——
    而「守衛要測在會改變答案的地方」是 [[weave-window-is-too-short]]
    當天燒出來的教訓(生產組態上四個突變三個沒咬)。
    """
    assert cfg(N=6).get("FORM_V") != cfg(N=6, AIRFRAME="lowdrag").get("FORM_V")


def test_n2_tail_distance_is_the_spacing_not_1_5x(wsl_ok):
    """⭐ n=2 沒有僚機 ⇒ 備長**就是**編隊夥伴,它的距離就是間距。

    取 1.5 倍(n>=3 的規則)會讓實際間距比要求的多 50% ——
    要求 2500 會飛成 3750,而且沒有任何訊息說它變了。
    """
    d = cfg(N=2, V_SPACING=2500)
    assert d.get("V_TAIL") == "2500", d
    assert d.get("V_SPACING") == "2500", d
    d6 = cfg(N=6, V_SPACING=2500)
    assert d6.get("V_TAIL") == "3750", "n>=3 仍應取 1.5 倍:%r" % d6


# ══════════════════════════════════════════════════════════════════════
#  紅線 C:速度必須在飛控接受的區間內
# ══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("n", [2, 6])
def test_every_commanded_speed_is_inside_the_fc_window(wsl_ok, n):
    """⚠️⚠️ 這一條就是那個失效本身。

    NOMINAL=14 / WAIT=10 / CATCH=20 全部低於 AIRSPEED_MIN=25
    ⇒ 每一發 DO_CHANGE_SPEED 都被整發拒收,而且**零告警**。
    """
    d = cfg(N=n)
    for k in ("FORM_V", "FORM_WAIT", "FORM_CATCH"):
        v = float(d[k])
        assert AIRSPEED_MIN <= v <= AIRSPEED_MAX, (
            "⚠️⚠️ %s = %s 落在飛控接受區間 [%.0f, %.0f] 之外\n"
            "    ⇒ DO_CHANGE_SPEED 會回 MAV_RESULT=4、整發丟掉,而且完全靜默。\n"
            "    直連送出只知道「送出去了」,收不收要靠 COMMAND_ACK 側錄。"
            % (k, d[k], AIRSPEED_MIN, AIRSPEED_MAX))


def test_wait_is_below_nominal_or_max_consensus_degenerates(wsl_ok):
    """⚠️ max-consensus 是 `max(wait, min(nominal, spd))`。

    wait > nominal 時它恆等於 wait —— 整個協同讓速退化成一個常數,
    而且完全靜默。`swarm_ros2_up.sh` 的 fast/jet preset 就是這樣壞的
    (WAIT=45 > NOMINAL=35)。
    """
    d = cfg(N=2)
    assert float(d["FORM_WAIT"]) < float(d["FORM_V"]), d


def test_the_groundspeed_to_eas_conversion_is_applied(wsl_ok):
    """⚠️⚠️ 指令端一律 EAS,幾何一律地速。混用會系統性差一個 E2T 修正。

    lowdrag:使用者定調 157 是**地速** ⇒ 指令值必須是 135 EAS
    (x0.857 = sqrt(sigma(3100)));把 157 直接送下去 = 實際地速 183(+17%)。

    ⚠️⚠️ 2026-08-28:stockjet **不適用 0.857**。實測比值是
       105.5 / 143.1 = **0.737**(差的 1.16 倍是皮托管少掉的 E2T 修正)。
       硬套 0.857 會從地速 107 得到 92 而不是使用者定調的 80。
       ⇒ stockjet 的巡航空速是**宣告值**,地速是它的實測對應值,
         兩者不再由一條換算式綁在一起 —— 但**方向**與**量級**仍然要鎖住,
         否則「哪個是 EAS 哪個是地速」這個坑就完全沒有守衛了。
    """
    d = cfg(N=2)
    gs, eas = float(d["FORM_V_GS"]), float(d["FORM_V"])
    assert eas < gs, "⚠️ EAS 必須小於地速,否則換算方向反了"
    ratio = eas / gs
    assert 0.70 < ratio < 0.79, (
        "EAS/地速 = %.3f 落在實測比值 0.737 的鄰域之外(地速 %.0f、EAS %.0f)"
        % (ratio, gs, eas))


def test_the_lowdrag_profile_keeps_the_standard_atmosphere_ratio(wsl_ok):
    """⭐ 舊剖面用的是標準大氣的 sqrt(sigma(3100)) = 0.857,不是實測比值。
    兩個剖面用**不同**的換算依據,這件事要鎖住,否則「統一一下」會
    無聲地把 lowdrag 的重現值改掉。"""
    d = cfg(N=6, AIRFRAME="lowdrag")
    gs, eas = float(d["FORM_V_GS"]), float(d["FORM_V"])
    assert abs(eas - gs * 0.857) < 1.0, (
        "地速 %.0f 應換算成 EAS %.0f,實得 %.0f" % (gs, gs * 0.857, eas))


def test_no_commanded_speed_is_above_what_the_airframe_can_reach(wsl_ok):
    """⭐⭐⭐ **新鎖(2026-08-28)**:速度指令在飛控區間內**還不夠**,
    還要是這具機體**飛得到**的。

    這兩件事的失效方式完全不同:
      > AIRSPEED_MAX  ⇒ MAV_RESULT=4 **整發拒收**(至少還有 ACK 可查)
      > 氣動天花板    ⇒ 送得出去、看起來一切正常、**永遠達不到**
                        ⇒ 油門飽和、SEP≈0、僚機追不上長機而且零告警

    後者正是 [[jet-conversion-never-applied]] 那幾十趟的形狀:指令 135 EAS、
    機體只推得到 ~98.5,而每一層看起來都正常。

    ⭐ 這條會紅的生產改動:把 FORM_CATCH 調到 105.5 以上
       (例如「追趕再快一點」直接照抄 lowdrag 的 157)。
    """
    d = cfg(N=6)
    for k in ("FORM_V", "FORM_WAIT", "FORM_CATCH"):
        v = float(d[k])
        assert v <= AERO_VMAX_EAS, (
            "⚠️⚠️ %s = %s 高於機體實測平飛頂速 %.1f EAS。"
            "  這**不會**被拒收 —— 指令送得出去、飛機永遠達不到,"
            "而症狀只會表現成「編隊收不攏」。"
            % (k, d[k], AERO_VMAX_EAS))


# ══════════════════════════════════════════════════════════════════════
#  紅線:formed_radius 要有鑑別力
# ══════════════════════════════════════════════════════════════════════

def test_formed_radius_is_smaller_than_the_spacing(wsl_ok):
    """⚠️⚠️ 到位判準若 >= 間距,容差球會把長機本身包進去。

    `swarm_ros2_up.sh` 的裸預設就是這樣:FORMED_RADIUS=250 > V_TAIL=220
    ⇒ 「到位」分不出「間距 220」與「疊在長機身上」。
    """
    for n, sp in ((2, 2500), (6, 4000)):
        d = cfg(N=n, V_SPACING=sp)
        assert float(d["FORM_FORMED_R"]) < float(d["V_TAIL"]), (n, d)


# ══════════════════════════════════════════════════════════════════════
#  紅線 B/D:備長的雁型方位
# ══════════════════════════════════════════════════════════════════════

def mk_backup(angle=0.0, side=1, thr=0.0, alt=3100.0, expand=120.0):
    import backup_node
    o = backup_node.BackupNode.__new__(backup_node.BackupNode)
    o.uav_id = 2
    o.v_tail_dist = 2500.0
    o.v_tail_angle = angle
    o.v_tail_side = side
    o._last_eff_angle = None
    from sar_swarm.phase_gate import LatchedAltGate, MonotonicRamp
    o._cruise_gate = LatchedAltGate(thr, hysteresis_m=150.0, dwell_s=0.0)
    o._expand = MonotonicRamp(expand)

    class _Mav:
        def position(self):
            return (25.0, 121.0, alt)

    o.mav = _Mav()
    o._t = [0.0]
    o._now = lambda: o._t[0]

    class _Log:
        def info(self, *a, **k):
            pass

    o.get_logger = lambda: _Log()
    return o


def test_angle_zero_is_bit_identical_to_the_old_hardcoded_180():
    """⭐⭐ 預設必須逐位元等同舊行為。

    ⚠️ 這是這次改動的安全帶。備長的 slot 決定 n=2 的整個編隊幾何,
       改錯了不會報錯 —— 只會讓下一趟的間距悄悄變成別的值。
    """
    o = mk_backup(angle=0.0)
    assert o._tail_bearing() == 180.0


@pytest.mark.parametrize("side,expect", [(1, 205.0), (-1, 155.0)])
def test_angle_set_gives_the_echelon_bearing(side, expect):
    """閘門停用(thr=0)時側偏全量施加 ⇒ 180 +/- theta。"""
    o = mk_backup(angle=25.0, side=side, thr=0.0)
    # ⚠️ 每拍最多只能累積 `MonotonicRamp.max_step_s` = 3 s（夾住排程抖動）
    #    ⇒ 不可以用兩拍跳 1000 s 來「讓 ramp 走完」，那只會累積 3 s。
    #    用真實 tick 的步長（1 s）跑滿 expand_time_s。
    for t in range(0, 200):
        o._t[0] = float(t)
        b = o._tail_bearing()
    assert abs(b - expect) < 0.01, b


def test_lateral_offset_is_deferred_until_cruise_altitude():
    """⭐⭐⭐ 紅線 D:爬升途中**不可以**帶側偏。

    2.5 km 間距的側偏是 2500*sin(25°) = 1057 m。wingman_node 實測:
    側偏最大的機爬升率 8.4 vs 12.0 m/s(掉 30%),而且地速也同時偏低
    ⇒ 不是「飛快犧牲爬升」,是側偏帶坡度、過載 n=1/cos(phi)、
      誘導阻力隨 n^2 上升的額外能量消耗。

    ⚠️ 今天 ALIGNW1 的 UAV4/UAV5 就是爬不上去而墜海的。
    """
    o = mk_backup(angle=25.0, thr=3000.0, alt=1000.0)   # 還在爬升
    for t in range(0, 400):
        o._t[0] = float(t)
        b = o._tail_bearing()
    assert abs(b - 180.0) < 0.01, (
        "⚠️⚠️ 爬升段(alt 1000 < 門檻 3000)竟然帶了 %.1f° 側偏 "
        "= %.0f m 橫移" % (b - 180.0, 2500 * math.sin(math.radians(b - 180.0))))


def test_expansion_is_gradual_not_a_step():
    """⚠️ 不可以硬切換:2.5 km 間距下硬切讓 slot 橫向瞬移 1057 m,

    飛機當場變成「落後 1 km」,速度控制器會全力追一個憑空出現的誤差。
    """
    o = mk_backup(angle=25.0, thr=3000.0, alt=3100.0, expand=120.0)
    seen = []
    for t in range(0, 200):                # 步長 1 s：見 max_step_s 的說明
        o._t[0] = float(t)
        seen.append(o._tail_bearing() - 180.0)
    assert seen[0] < 1.0, "第一拍就全量展開了:%r" % seen[:3]
    assert seen[-1] > 24.0, "展開沒有走完:%r" % seen[-3:]
    # 單調不減
    assert all(b >= a - 1e-9 for a, b in zip(seen, seen[1:])), seen


def test_the_ramp_freezes_instead_of_collapsing_on_a_sag():
    """⚠️⚠️ 掉回爬升段時展開角要**凍結**,不可以塌回 0。

    塌回等於命令 slot 橫向瞬移回去 —— 而 wingman_node 本來就有
    「不可以在門檻上硬切換」的註解,但只防了往上展開那一邊。
    """
    o = mk_backup(angle=25.0, thr=3000.0, alt=3100.0, expand=120.0)
    for t in range(0, 60):
        o._t[0] = float(t)
        o._tail_bearing()
    mid = o._tail_bearing() - 180.0
    assert mid > 5.0, "展開還沒開始,測不到凍結:%.2f" % mid
    o.mav.position = lambda: (25.0, 121.0, 1000.0)      # 掉回爬升段
    for t in range(60, 300):
        o._t[0] = float(t)
        b = o._tail_bearing() - 180.0
    assert b >= mid - 1e-6, "展開角塌回了:%.2f -> %.2f" % (mid, b)


# ══════════════════════════════════════════════════════════════════════
#  源碼層:寫死的 180 不可以回來
# ══════════════════════════════════════════════════════════════════════

def test_no_hardcoded_180_bearing_survives():
    """⚠️ 兩個呼叫點原本各自寫死 180.0。

    側偏一旦隨爬升相位變化,寫死的那兩處就會各自演化,而它們算出來的
    slot 只差幾百公尺 —— 不會報錯,只會讓 `_formation_control` 的誤差
    對著 A 點量、WP 送到 B 點。
    """
    src = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    assert "self.v_tail_dist, 180.0)" not in src, (
        "⚠️ 還有呼叫點寫死 180.0 —— 兩處必須共用 _tail_bearing()")
    assert src.count("self._tail_bearing()") >= 1


def test_launch_key_names_match_the_node(wsl_ok):
    """⚠️⚠️ launch 的鍵名與節點 declare 的鍵名必須**逐字**相同。

    對不上時 ROS2 把 launch 傳的值當成無害 override **靜默丟掉**,
    節點吃自己的預設。兩邊數字剛好一樣時完全沒有症狀 —— 直到有人
    調 launch 然後發現「改了沒作用」。這正是本專案的招牌陷阱。
    """
    node = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    launch = (WSL / "sar_swarm_patches"
              / "sar_failover_nodes.launch.py").read_text(encoding="utf-8")
    for key in ("v_tail_angle_deg", "v_tail_side",
                "cruise_alt_threshold", "cruise_gate_hyst_m",
                "cruise_gate_dwell_s", "expand_time_s"):
        assert 'declare_parameter("%s"' % key in node, "節點沒宣告 %s" % key
        assert '"%s"' % key in launch, "launch 沒有 %s" % key


def test_the_declared_default_angle_is_zero():
    """⭐⭐ 鎖**宣告的預設值**,不是鎖測試裡塞進去的屬性。

    ⚠️ 刪除實驗抓到的缺口:把 `declare_parameter("v_tail_angle_deg", 25.0)`
       改掉,上面那條「逐位元舊行為」的測試**照樣是綠的** —— 因為它用
       `mk_backup(angle=0.0)` 直接塞屬性,繞過了 declare。
       ⇒ 有人改預設就會讓**每一趟 n=2 悄悄帶上 1057 m 側偏**,
         而側偏正是爬升率掉 30% 的原因(今天 UAV4/5 就是爬不上去墜海的)。
    """
    src = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    assert 'declare_parameter("v_tail_angle_deg", 0.0)' in src, (
        "⚠️⚠️ v_tail_angle_deg 的宣告預設不是 0.0 ⇒ 雁型變成預設開啟。\n"
        "    側偏會帶坡度、過載 1/cos(phi)、誘導阻力隨 n^2 上升,\n"
        "    而爬升段帶側偏正是機體爬不上去的直接原因。")


def test_every_launch_argument_has_a_matching_defaults_key():
    """⚠️⚠️ `DeclareLaunchArgument(k, default_value=_FORMATION_DEFAULTS[k])`
    的兩個 k 必須是**同一個字**。

    ⚠️ 刪除實驗抓到的缺口:把 `_FORMATION_DEFAULTS` 的鍵打錯一個字母,
       上面那條「鍵名對齊」的測試照樣綠 —— 因為 DeclareLaunchArgument
       那一行仍然含著正確的名字。真正會爆的是 launch 期的 KeyError,
       而那要跑起來才知道。
    """
    import re as _re
    src = (WSL / "sar_swarm_patches"
           / "sar_failover_nodes.launch.py").read_text(encoding="utf-8")
    # ⚠️ 先切出字典區塊再抓鍵 —— 不可以整檔逐行 `^\s+"k":`:
    #    同一行可以有兩個鍵(`"v_spacing_m": "4000.0", "v_angle_deg": "35.0",`),
    #    行首錨會漏掉第二個 ⇒ 對**未突變**的源碼就報一堆假紅燈。
    #    我第一版就是這樣,而假紅燈會讓下一個人直接刪掉這道守衛。
    _i = src.index("_FORMATION_DEFAULTS = {")
    _j = src.index("\ndef _formation_params", _i)
    keys = set(_re.findall(r'"([a-z0-9_]+)"\s*:', src[_i:_j]))
    used = _re.findall(r'_FORMATION_DEFAULTS\["([a-z0-9_]+)"\]', src)
    missing = sorted(set(used) - keys)
    assert not missing, (
        "⚠️⚠️ 這些鍵被 _FORMATION_DEFAULTS[...] 取用,但字典裡沒有:%s\n"
        "    ⇒ launch 期 KeyError。而字典鍵打錯一個字母時,\n"
        "      DeclareLaunchArgument 那一行仍含正確名字 ⇒ 純字串檢查抓不到。"
        % missing)
    for k in ("v_tail_angle_deg", "v_tail_side"):
        assert k in keys, "⚠️ _FORMATION_DEFAULTS 沒有 %s" % k


def test_formation_params_are_declared_as_float_in_every_node():
    """⭐⭐⭐ `_formation_params()` 對每一顆都強制 `value_type=float`。

    任何節點把其中一顆宣告成 int,建構期就會拋:
        InvalidParameterTypeException: 'x' to '1.0' of type 'DOUBLE',
        expecting type 'INTEGER'
    ⇒ **節點整個起不來**,而症狀是「沒有進入 ready」—— 看起來像節點壞了
      或起太慢,完全不像型別問題。RPR 整批三趟就是這樣報廢的。

    ⚠️ 既有的 `tests/test_ros_param_types.py` **只涵蓋 tot_node.py**,
       所以編隊層的同型錯誤它抓不到。2026-08-24 實測:我把
       `v_tail_side` 宣告成 int,四道關卡(bash -n / py_compile /
       3100 條單元測試 / 部署 md5)全部通過,而節點在真的起動時當場崩。
    ⚠️ 這一條是**源碼層**的;真正的證據是 deploy_build.sh 的節點啟動冒煙。
    """
    import ast as _ast
    launch_src = (WSL / "sar_swarm_patches"
                  / "sar_failover_nodes.launch.py").read_text(encoding="utf-8")
    i = launch_src.index("_FORMATION_DEFAULTS = {")
    j = launch_src.index("\ndef _formation_params", i)
    import re as _re
    keys = set(_re.findall(r'"([a-z0-9_]+)"\s*:', launch_src[i:j]))
    assert keys, "抓不到 _FORMATION_DEFAULTS 的鍵"

    bad = []
    for name in ("master_node.py", "backup_node.py", "wingman_node.py"):
        path = WSL / "sar_swarm_patches" / name
        if not path.is_file():
            continue
        tree = _ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in _ast.walk(tree):
            if not (isinstance(node, _ast.Call)
                    and getattr(node.func, "attr", "") == "declare_parameter"
                    and len(node.args) >= 2):
                continue
            k = node.args[0]
            v = node.args[1]
            if not (isinstance(k, _ast.Constant) and k.value in keys):
                continue
            if isinstance(v, _ast.Constant) and isinstance(v.value, bool):
                continue                     # bool 走另一條通道
            if isinstance(v, _ast.Constant) and isinstance(v.value, int):
                bad.append("%s:%d %s = %r (int)" % (name, node.lineno, k.value,
                                                    v.value))
    assert not bad, (
        "⚠️⚠️⚠️ 這些 _FORMATION_DEFAULTS 參數在節點端被宣告成 **int**,\n"
        "    而 launch 一律送 float ⇒ 建構期 InvalidParameterTypeException,\n"
        "    節點整個起不來,症狀卻是「沒有進入 ready」:\n      %s"
        % "\n      ".join(bad))


def test_the_bare_script_is_a_flyable_config():
    """⭐⭐⭐ 裸跑 `rep_interactive.sh` 必須產出一個**契約會通過**的組態。

    2026-08-24 實測:裸跑時契約閘門當場擋下 ——
        ✗ [泡棉禁區] state_max_age = 0.0
        ✗ [泡棉禁區] att_eas_floor = 0.0
    兩個都是 `swarm_ros2_up.sh` 的裸預設(0 = 停用)。

    而 ALIGNW1 那一趟契約是**全數通過**的 ⇒ 當時是有人在**命令列上**帶了
    這兩個值。又一次 [[two1-config-not-in-repo]]:能跑的組態只活在一行指令裡,
    而腳本裸跑不是一個可飛的組態 —— 那件事只有真的去跑才會發現。

    ⚠️ 值取自 ALIGNW1 的契約快照(那一趟真的飛了),不是憑空訂的 ——
       契約日誌自己警告「建議值沒有實測背書,照著設是開一趟新實驗」。
    """
    src = (WSL / "rep_interactive.sh").read_text(encoding="utf-8")
    import re as _re
    for key, want in (("STATE_MAX_AGE", "1.0"), ("ATT_EAS_FLOOR", "32.0")):
        m = _re.search(r'^%s="\$\{%s:-([0-9.]+)\}"' % (key, key), src, _re.M)
        assert m, (
            "⚠️⚠️ rep_interactive.sh 沒有設 %s ⇒ 吃 swarm_ros2_up.sh 的裸預設 0.0,\n"
            "    而契約表把 0.0 列為**泡棉禁區**,整批會在起跑前被擋下。" % key)
        assert m.group(1) == want, (
            "%s = %s,而 ALIGNW1(唯一飛過的參照)是 %s" % (key, m.group(1), want))
    assert "export STATE_MAX_AGE ATT_EAS_FLOOR" in src, (
        "⚠️ 設了但沒 export ⇒ `setsid env ...` 起的子行程看不到,等於沒設。")


# ══════════════════════════════════════════════════════════════════════
#  紅線 E:備長的縱向速度律必須是**有號**的(FORMB1,2026-08-24)
# ══════════════════════════════════════════════════════════════════════

def mk_backup_speed(along, nominal=135.0, catch=157.0, catch_r=528.0):
    """只塞 `_formation_control` 速度那一段會碰到的欄位。"""
    import backup_node
    o = backup_node.BackupNode.__new__(backup_node.BackupNode)
    o.uav_id = 2
    o.nominal_speed = nominal
    o.catch_speed = catch
    o.catch_radius = catch_r
    o.formed_radius = 200.0
    o.my_fix = (25.0, 121.0)
    o.leader = {"lat": 25.0, "lon": 121.0, "heading": 0.0, "speed": nominal}
    o._along_to_slot = lambda _slot: along
    return o


def test_the_backup_slows_down_when_it_is_ahead_of_its_slot():
    """⭐⭐⭐ n=2 起飛時的幾何:僚機在長機後方 30 m,slot 在後方 2500 m
    ⇒ 僚機**超前 slot 2470 m**,應該**減速**讓自己落到 slot 上。

    ⚠️⚠️ 舊的無號 `speed_for_error()` 看到 |err|=2470 就命令 catch_speed 157
       ⇒ **加速、超前更多**。FORMB1 實測後果:
           長機 3173 m / 爬升率 4.91 m/s     僚機 1257 m / **1.50 m/s**
       僚機整段被釘在 157 EAS 追一個它已經超前的點,而追趕與爬升搶同一份
       剩餘功率 ⇒ 600 s 只爬到 917 m,而側向展開閘門要 3000 m
       ⇒ **雁型永遠等不到**。

    ⚠️ `speed_for_along_error` 的 docstring 自己預言過:「這個 bug 先前不會發作…
       **修好速度通道等於同時解鎖了這個 bug**」—— 而 2026-08-24 正好把速度通道
       從 14/10/20 修成 135/90/157。
    """
    from geodesy import speed_for_along_error
    v_lo = max(25.0, 135.0 * 0.67)
    ahead = speed_for_along_error(-2470.0, 528.0, 135.0, 157.0, v_lo)
    assert ahead < 135.0, (
        "⚠️⚠️ 超前 slot 2470 m 竟然沒有減速(%.1f EAS)⇒ 會加速逃離 slot" % ahead)
    assert abs(ahead - v_lo) < 0.01, "應該飽和到下限 %.1f,實得 %.1f" % (v_lo, ahead)

    behind = speed_for_along_error(+2470.0, 528.0, 135.0, 157.0, v_lo)
    assert abs(behind - 157.0) < 0.01, "落後時應該飽和到 catch,實得 %.1f" % behind


def test_backup_uses_the_signed_law_not_the_unsigned_one():
    """⚠️ 源碼層:`_formation_control` 必須走有號律。

    ⚠️⚠️ 無號律**留在檔案裡當退路**(拿不到長機航向時)是對的 —— 但主路徑
       必須是有號的。這一條分辨得出「兩支都在」與「還在用舊的」。
    """
    src = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    i = src.index("def _formation_control")
    body = src[i:i + 2600]
    assert "speed_for_along_error(" in body, (
        "⚠️⚠️ backup 的速度律還是無號的 ⇒ 超前 slot 時會加速逃離")
    assert "_along_to_slot(" in body


def test_the_along_projection_uses_the_leader_heading():
    """⚠️ 用**長機航向**而不是本機航向 —— 本機航向會讓這個量隨自己轉動而變,
    那是 trail 投影正回饋的同一種錯誤。
    """
    src = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    i = src.index("def _along_to_slot")
    body = src[i:i + 900]
    assert 'L["heading"]' in body, "沒有用長機航向:%s" % body[:300]


@pytest.mark.parametrize("along,expect", [
    (0.0, 135.0),          # 在 slot 上 → nominal
    (528.0, 157.0),        # 落後一個 catch_radius → 飽和到 catch
    (-528.0, 90.45),       # 超前一個 catch_radius → 飽和到下限
    (264.0, 146.0),        # 半個 → 線性內插
])
def test_the_signed_law_is_linear_and_saturates_both_ways(along, expect):
    """⚠️ 兩側都要飽和。只做落後那一側的話,超前就會退回無號行為。"""
    from geodesy import speed_for_along_error
    got = speed_for_along_error(along, 528.0, 135.0, 157.0, max(25.0, 135.0 * 0.67))
    assert abs(got - expect) < 0.6, (along, got, expect)


# ══════════════════════════════════════════════════════════════════════
#  紅線 F:盤旋半徑(FORMB1/FORMC1 的坡度實測,2026-08-25)
# ══════════════════════════════════════════════════════════════════════

def test_the_reposition_radius_default_is_bit_identical_old_behaviour():
    """⚠️ 預設 30 = 逐位元舊行為。要新值必須明設。

    ⚠️⚠️ 我**不知道**最佳半徑:小 ⇒ 坡度大 ⇒ 留不住高度;大 ⇒ 站位鬆。
       所以這是一個要用實驗定的值,預設不可以替使用者選。
    """
    src = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    assert 'declare_parameter("reposition_radius_m", 30.0)' in src, (
        "⚠️ 盤旋半徑的宣告預設不是 30.0 ⇒ 舊行為被悄悄改掉")


def test_reposition_radius_reaches_the_mavros_interface():
    """⚠️ 宣告了但沒傳進去 = 沒改。

    `MavrosInterface` 原本只從**行程級 env** 讀它 ⇒ 長機與僚機
    共用同一個值，改不了單邊。必須走建構參數。
    """
    bk = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    # ⚠️ 把空白拿掉再比：呼叫是多行的，直接比字串會因為
    #    換行與縮排而失效（而那種失效看起來像「沒接上」）。
    flat = "".join(bk.split())
    assert 'reposition_radius=float(g("reposition_radius_m").value)' in flat, (
        "⚠️ backup_node 沒有把 reposition_radius_m 傳給 MavrosInterface")
    mi = (WSL / "sar_swarm_patches"
          / "mavros_interface.py").read_text(encoding="utf-8")
    assert "reposition_radius=None" in mi, "MavrosInterface 沒有接受建構參數"
    assert "reposition_radius if reposition_radius is not None" in mi, (
        "⚠️ 建構參數沒有優先於 env ⇒ 傳了也沒用")


def test_a_30m_loiter_radius_is_not_flyable_at_cruise_speed():
    """⭐⭐⭐ 這一條把「為什麼 30 是錯的」釘成數字,而不是一句註解。

    盤旋半徑 R 在速度 V 下需要的坡度是 atan(V^2/(g*R))。
    30 m(L1 乘 eas2tas^2 後約 41 m)在 157 m/s 下需要 **89.3°** ——
    而 ROLL_LIMIT_DEG 是 80。飛機只能頂在上限追一個捕獲不到的圈。

    FORMB1/FORMC1 實測坡度 p90 = 74-75°、42% 的拍 >60°,而長機是 0%。
    """
    G = 9.80665
    for r_cmd, v, want_min in ((30.0 * 1.365, 157.0, 85.0),):
        need = math.degrees(math.atan(v * v / (G * r_cmd)))
        assert need > want_min, (r_cmd, need)
    # 由可持續坡度反推的半徑
    for bank, lo, hi in ((30.0, 4200, 4500), (45.0, 2400, 2600), (60.0, 1400, 1500)):
        r = 157.0 ** 2 / (G * math.tan(math.radians(bank)))
        assert lo < r < hi, (bank, r)


def test_reposition_radius_is_declared_as_float_and_keys_match():
    """⚠️ 今天已經因為這兩件事各壞過一次:
    int/float 不符 ⇒ 節點起不來;鍵名不符 ⇒ launch 的值被靜默丟掉。
    """
    node = (WSL / "sar_swarm_patches" / "backup_node.py").read_text(encoding="utf-8")
    launch = (WSL / "sar_swarm_patches"
              / "sar_failover_nodes.launch.py").read_text(encoding="utf-8")
    assert 'declare_parameter("reposition_radius_m", 30.0)' in node, "不是 float"
    assert '"reposition_radius_m"' in launch, "launch 沒有這個鍵"
    up = (WSL / "swarm_ros2_up.sh").read_text(encoding="utf-8")
    assert "reposition_radius_m:=" in up, "swarm_ros2_up.sh 沒有傳下去"


# ══════════════════════════════════════════════════════════════════════
#  紅線 G:相對長機的高度框(使用者 2026-08-25 定調)
# ══════════════════════════════════════════════════════════════════════

def mk_backup_alt(mode="relative", my_rel=500.0, my_amsl=532.0,
                  leader_alt=1000.0, dz=50.0, form_alt=3100.0, gmin=15.0):
    import backup_node
    o = backup_node.BackupNode.__new__(backup_node.BackupNode)
    o.uav_id = 2
    o.alt_mode = mode
    o.v_tail_dz = dz
    o.formation_alt = form_alt
    o._guided_min_alt = gmin
    o.my_rel = my_rel
    o.my_amsl = my_amsl
    o.leader = None if leader_alt is None else {"alt": leader_alt}
    o._log = FakeLogger()
    o.get_logger = lambda: o._log
    return o


def test_absolute_mode_is_bit_identical_to_the_old_constant():
    """⭐⭐ 預設 absolute 必須回**原本那個常數**,一個位元都不差。"""
    o = mk_backup_alt(mode="absolute")
    assert o._formation_alt_cmd() == 3100.0


def test_relative_mode_cancels_the_altitude_datum():
    """⭐⭐⭐ 這一條就是使用者提的做法為什麼比較對。

        target_rel = my_rel + (leader_amsl - my_amsl) + dz

    三項都是實測、基準自洽:`(leader_amsl - my_amsl)` 是兩機**真實**的
    高度差,AMSL 相減時兩邊的 datum 一起消掉 —— 不需要知道 HOME 海拔。

    ⚠️⚠️ 這一點是關鍵:本專案反覆踩「兩個基準不同步就平移 32 m 而且
       完全靜默」的坑([[agl-datum-false-kill]])。只量**差量**就不會有
       那個問題。
    """
    # 我在 rel 500(AMSL 532 ⇒ HOME 海拔 32);長機在 AMSL 1000
    # ⇒ 長機比我高 468 m ⇒ 我要飛到 rel 500+468+50 = 1018
    o = mk_backup_alt(my_rel=500.0, my_amsl=532.0, leader_alt=1000.0, dz=50.0)
    assert abs(o._formation_alt_cmd() - 1018.0) < 1e-6, o._formation_alt_cmd()

    # ⭐ 換一個 HOME 海拔(AMSL 全部 +200),答案**必須不變** ——
    #    那正是「datum 消掉了」的定義。
    o2 = mk_backup_alt(my_rel=500.0, my_amsl=732.0, leader_alt=1200.0, dz=50.0)
    assert abs(o2._formation_alt_cmd() - 1018.0) < 1e-6, o2._formation_alt_cmd()


def test_relative_falls_back_loudly_when_an_input_is_missing():
    """⚠️⚠️ 缺任何一項就**退回 absolute 並說出來**。

    絕對不可以拿「猜一個 HOME 海拔」去補 —— 那會產出一個看起來合理
    但平移了幾十公尺的高度指令,而且完全靜默。
    """
    for kw in (dict(leader_alt=None), dict(my_amsl=None), dict(my_rel=None)):
        o = mk_backup_alt(**kw)
        assert o._formation_alt_cmd() == 3100.0, kw
        txt = "\n".join(o._log.lines)
        assert "退回 absolute" in txt, (kw, txt)


def test_the_fallback_warning_is_printed_once_not_every_tick():
    o = mk_backup_alt(leader_alt=None)
    for _ in range(50):
        o._formation_alt_cmd()
    assert sum(1 for ln in o._log.lines if "退回 absolute" in ln) == 1


# ⚠️ 參數要**真的觸發夾制**。第一版用 leader_alt=0:
#      500 + (0 - 532) + 50 = 18  > 地板 15  ⇒ 根本沒夾到,
#    而測試卻宣稱自己在測夾制。一條到不了它宣稱要測的分支的測試
#    等於沒測([[instrument-that-cannot-fire]] 同型)。
@pytest.mark.parametrize("leader_alt,expect", [
    (-500.0, 15.0),       # 500 + (-500-532) + 50 = -482 ⇒ 夾到 guided_min_alt
    (100000.0, 3100.0),   # 夾到 formation_alt
])
def test_relative_is_clamped_both_ways(leader_alt, expect):
    """⚠️ 相對框不可以把飛機帶到比編隊巡航高度更高、或離地太近的地方。

    長機的高度是**外部輸入**(GeoPose 廣播)—— 它壞掉時不可以直接變成
    我的高度指令。
    """
    o = mk_backup_alt(leader_alt=leader_alt)
    assert abs(o._formation_alt_cmd() - expect) < 1e-6


def test_alt_mode_is_a_string_parameter_not_in_the_float_dict():
    """⚠️⚠️ `_formation_params()` 對 `_FORMATION_DEFAULTS` 的每一顆都強制
    `value_type=float`。把字串放進去會在建構期拋
    InvalidParameterTypeException ⇒ **節點整個起不來**,而症狀是
    「沒有進入 ready」。今天已經因為 `v_tail_side` 踩過一次。
    """
    import re as _re
    src = (WSL / "sar_swarm_patches"
           / "sar_failover_nodes.launch.py").read_text(encoding="utf-8")
    i = src.index("_FORMATION_DEFAULTS = {")
    j = src.index("\ndef _formation_params", i)
    keys = set(_re.findall(r'"([a-z0-9_]+)"\s*:', src[i:j]))
    assert "alt_mode" not in keys, (
        "⚠️⚠️ alt_mode 是字串,放進 _FORMATION_DEFAULTS 會讓節點起不來")
    assert "v_tail_dz_m" in keys, "v_tail_dz_m 是數值,應該走 formation"
    assert '"alt_mode": LaunchConfiguration("alt_mode")' in src, (
        "⚠️ alt_mode 沒有從 launch 傳給 backup_node")


def test_sep_up_completes_the_leader_frame_triple():
    """⭐ (along, cross, up) 三個都是**差量** ⇒ 兩邊的 datum 一起消掉。

    這正是使用者提的「長機 = (0,0,0)、僚機 = (0, 2500, 50)」。
    """
    from formation_log import COLUMNS
    for c in ("sep_along", "sep_cross", "sep_up"):
        assert c in COLUMNS, "少了 %s ⇒ 長機座標系不完整" % c
