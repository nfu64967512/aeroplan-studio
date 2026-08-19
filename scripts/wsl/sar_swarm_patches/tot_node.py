#!/usr/bin/env python3
"""tot_node — 蜂群同時抵達(Time-on-Target)協同節點【ROS2 協同管道 ②】。

架構分工(兩管道):
  ① MAVLink/飛控(Mission Planner 做得到的):單機機體控制 —— 由 ArduPilot
     自駕儀執行。本節點只透過 DO_REPOSITION + DO_CHANGE_SPEED 下達「去哪、多快」。
  ② ROS2 協同(MP 做不到的):**本節點** —— 算出扇形 slot 幾何、slot 指派(防撞)、
     共同抵達時刻 T_arr、每台的模式(直線/全速/S型機動)、齊射時序。算完經①下達。

I/O:機體狀態與指令走 MAVLink 分流(6760+10i;pymavlink)。協同狀態發佈到 ROS2:
  pub /swarm/tot_gate    std_msgs/String  {target, r_fan, half_angle, slots[], approach}
  pub /swarm/tot_status  std_msgs/String  {phase, T_arr, uavs:{id:{mode,d,arr}}}
  sub /swarm/tot_target  std_msgs/String  {lat, lon, [approach_from], [amsl], [src]}
(state-in 亦可改走 MAVROS/uavN/global_position 以求全一致;此處為自足直讀分流。)

狀態機(視覺震撼終端俯衝版):
  STANDBY— 待命。**沒有目標**(tlat/tlon 未給或為 .nan)時停在這裡:照常抽
           MAVLink 鏈路、照常發 /swarm/tot_status,但**一個指令都不下**。
           收到 /swarm/tot_target 才重設狀態並進 INIT。打完回 STANDBY,
           可以接第二個目標。見 `_arm_target` / `_reset_for_new_target`。
  ALIGN  — TOT 對齊扇形 slot(快機 S 型燒時間、慢機直線加速),同步抵扇形。
  PERCH  — 各機沿切線內切到 pushover 環,同步爬升到 perch_alt 高點佔位。
  DIVE   — 全員同步 pushover,全速俯衝(終端加速)收斂目標 T;微高度分層防撞、
           時間校正 → ~同時衝擊(1m 級)。

兩條啟動路徑,**語意完全分開**:
  (1) 批次航次(stot_trial.sh / stot_kpi_loop.sh / swarm_tot_up.sh):啟動時就
      用 -p tlat/tlon 帶目標 ⇒ 直接進 INIT,行為與 2026-08-05 之前**一字不差**,
      打完進 DONE 就不動了。
  (2) 常駐服務(swarm_ros2_up.sh):啟動時帶 `-p tlat:=.nan -p tlon:=.nan`
      ⇒ 進 STANDBY 等 AeroPlan 右鍵送目標,打完自己回 STANDBY。
"""
import csv, json, math, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from pymavlink import mavutil

# ⚠️ 兩種佈局都要 import 得到,而且**失敗要大聲**:預測器 import 壞掉時
#    節點若靜默退回「沒有預測」,就會產出一趟看起來正常、其實沒有影子資料的航次。
try:
    from sar_swarm import predictor as _pred          # 部署版(md5 驗證過)
except ImportError:                                    # pragma: no cover
    try:
        import predictor as _pred                      # 開發機同目錄
    except ImportError:
        _pred = None

EARTH = 111320.0
# ⚠️ 另有一個同名的**區域**變數 GRAV 寫在 PN 姿態律裡(那段自成一體,不動它)。
#    這個模組層常數是給 gate_of() 用的,兩者數值相同。
G0 = 9.80665

# ═══════════════════════════════════════════════════════════════════════════
#  結案哨兵 —— 給機器 grep 的固定字串
# ═══════════════════════════════════════════════════════════════════════════
# ⚠️⚠️ CLOSEOUT_SENTINEL 必須出現在**每一種**結案形狀的摘要行裡:
#      (a) 零 miss + 全部低空觸地   (b) 零 miss + 有機高空掠過(RD1 形狀)
#      (c) 有 miss + 有命中         (d) 全部 miss
#    它的語意是「這趟飛完了」,**不是**「這趟成功了」;成功與否由 _kpi_parse.py
#    從同一行的數字判。上一版把哨兵和達標描述綁在同一個字串上:零 miss 但有機
#    高空掠過時,達標描述(「⚠️ 名義命中…」)把哨兵整個蓋掉 ⇒ 三個消費者
#    (stot_trial.sh、stot_kpi_loop.sh、_kpi_parse.py)全部落空 ⇒ 航次被記成
#    timeout(預設要空等 120×12 s = 1440 s)、還會跳過 make_artifacts.sh。
#    RD1 正是這個形狀(6/6 命中、命中高度 598–1460 m、零 miss)。
#    ⇒ 哨兵(機器讀)與達標描述(人讀)從此**分離**,不准互相覆寫。
#    改這個字串 = 必須同步改三個消費者與 tests/test_summary_sentinel.py。
CLOSEOUT_SENTINEL = "★★ 俯衝結案:"
# 舊哨兵。形狀 (a) 仍逐字保留它,既有基準與歷史 log 的比對靠它;
# 其餘三種形狀**不准**出現(免得舊 grep 把脫靶/高空掠過讀成成功)。
LEGACY_ALL_HIT = "俯衝命中紅心:全員命中"

GATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)  # latched:晚連的前端也拿得到

# ═══════════════════════════════════════════════════════════════════════════
#  待命模式:執行期目標指定
# ═══════════════════════════════════════════════════════════════════════════
#: 目標指定 topic。**型別選 std_msgs/String + JSON,理由逐條寫在這裡** ——
#:  (1) 本節點既有的 /swarm/tot_gate、/swarm/tot_status 就是 String+JSON,
#:      沿用同一個慣例,前端的解析器只要一份。
#:  (2) 不需要新增 .msg ⇒ **不必 colcon build**。自訂訊息要改 sar_swarm 的
#:      CMakeLists/package.xml 再重建整包(= 一定要跑 deploy_build.sh),
#:      而發佈端(AeroPlan 從 Windows 呼 `wsl.exe … ros2 topic pub`)還得先
#:      source 到同一份 install/ 才認得那個型別。String 是 std_msgs,
#:      任何 ROS2 環境都認得,`ros2 topic pub` 一行就送得出去。
#:  (3) 可擴充:approach_from / amsl / src 這些欄位是**選填**,前端不填就走
#:      節點的預設,將來加欄位不會破壞舊發佈端(自訂 msg 加欄位會)。
#:  (4) 反面選項 sensor_msgs/NavSatFix 的語意是「某載具的 GPS 定位」,
#:      geometry_msgs/PointStamped 的語意是「某座標系裡的一個點」——
#:      兩者都得另外約定「哪個欄位是紅心」,並不比 JSON 清楚。
TARGET_TOPIC = "/swarm/tot_target"
#: 待命就緒哨兵 —— **給機器 grep 的固定字串**(同 CLOSEOUT_SENTINEL 的用意)。
#: swarm_ros2_up.sh 起完 tot_node 之後會 grep 這一行來確認它**真的**進了
#: STANDBY,而不是「起了但卡在 wait_heartbeat / 參數型別崩了」。
#: ⚠️⚠️ 改這個字串 = 必須同步改 swarm_ros2_up.sh 的驗證式;
#:    tests/test_tot_standby.py 有一條迴歸鎖比對兩邊。刻意全 ASCII 括號 +
#:    固定詞序,避免全形/半形括號不一致這種**看不出來**的失配。
STANDBY_SENTINEL = "STANDBY 待命就緒"
#: 生效值契約哨兵 —— **給機器解析的固定字串**。
#: 既有的「tot_node 生效值:」那一行是給人看的:它涵蓋不全(缺 dive_speed /
#: dive_pace / gate_r / dive_angle),而且格式為了可讀性混了單位與中文。
#: ⚠️⚠️ 涵蓋不全 = **想比對也讀不到** ⇒ 契約層對那幾項是瞎的,
#:    而那幾項恰好是 dive-pace-saturates-vmin / u1-descent-authority 的主角。
#: 這一行的契約:`<名稱>=<數字>` 以單一空白分隔,全 ASCII,**只增不改**。
#: 消費端 `scripts/wsl/contract_verify.py --from-log`;迴歸鎖
#: `tests/test_contract_table.py::TestSentinelsAgree`。
#: ⚠️ **刻意不帶括號**:STANDBY_SENTINEL 上面那段註解已經記取過教訓 ——
#:    全形/半形括號不一致是**看不出來**的失配,而失配的表現是「全部讀不到」,
#:    又剛好會被誤讀成「節點沒起來」。不用括號就沒有這個失效模式。
CONTRACT_SENTINEL = "生效值契約"
#: ⭐ 主迴圈週期(秒)。**單一事實來源** —— timer 與 α 濾波增益都必須引用它。
#: ⚠️ 先前 `create_timer(0.05, …)` 寫死,而參數 `pn_hz` 另外宣告 50 Hz,
#:    兩者各自獨立 ⇒ 名義 50 Hz / 實際 20 Hz,α 濾波時間常數因此差 2.5 倍
#:    而且沒有任何東西會吭聲。生效速率一律由這個常數導出並印進契約行。
TICK_PERIOD_S = 0.05
#: ArduPlane 的 GUIDED 自訂模式編號。⚠️ 姿態指令在非 GUIDED 下會被**靜默丟棄**。
PLANE_MODE_GUIDED = 15
#: 韌體對 SET_ATTITUDE_TARGET 的斷流逾時(s)。斷流會**無聲**掉回 L1 導航。
GUIDED_TIMEOUT_S = 3.0
#: ⚠️⚠️ 目標 topic **絕對不可以 latched(TRANSIENT_LOCAL)**。
#:  gate/status 是「狀態」,晚連的前端補一份是對的;目標是**命令**。
#:  latch 之下,下一個開機的 tot_node 一連上就會收到上一場的目標,
#:  在沒有人按任何按鈕的情況下自己開打 —— 這是本專案最不能容忍的一類失敗。
#: ⚠️ 用 RELIABLE 而非 BEST_EFFORT:`ros2 topic pub --once` 在 Humble 之下
#:  預設 `--wait-matching-subscriptions 1`,會等到配對成功才送;BEST_EFFORT
#:  遇到瞬時丟包就靜默掉一次打擊指令,而使用者只會看到「按了沒反應」。
TARGET_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=1,
                        durability=DurabilityPolicy.VOLATILE)

# ⚠️⚠️ 接新目標時要清空的**每機字典** —— 這份清單就是規格,漏一項全部是靜默的:
#   * 漏 wp_key/wp_t/wp_pt ⇒ _goto/_goto_latched 的鎖存認為「航點沒動過」,
#     整隊繼續飛**上一個目標**的航點,而且不會有任何錯誤訊息。
#   * 漏 impact/missed  ⇒ _done_count() 一開始就等於 n,第二發在 DIVE 的第一個
#     tick 就「結案」,摘要行印的是上一發的數字。
#   * 漏 dmin/hit_alt/hit_d ⇒ _metrics_frozen 為真 ⇒ 第二發全程不量測。
#   * 漏 zg/zg0/terr_bull/terr_home ⇒ 沿用**上一個紅心**的地表高度;
#     這正是 memo 記載差 1821 m 的那個坑的換目標版本。
#   * 漏 released/ready/align_ok/arr ⇒ 第二發直接從「大家都就緒」起跑。
# 測試 tests/test_tot_standby.py 逐項比對這份清單(手寫一份對照,少一項就紅)。
#: ⚠️⚠️ Jiang 2021 (Aerospace 8(8):231) 的 N′ 符號:原文**自相矛盾**。
#:   Eq.(28) 印 `N′ = (2−N)/(6(N−1)(2N−1))`;Eq.(39) 印 `(N−2)/…`。
#:   True  = 用 (2−N)(Eq.28,本輪指派值)
#:   False = 用 (N−2)(Eq.39)
#: 反面證據(支持 Eq.39 才對、Eq.28 是 typo):
#:   ① 把 Eq.(29) 展到二階必須退回 Eq.(30) 的係數 1/(2(2N−1)),
#:      而 1/(6(N−1)) + N′ = 1/(2(2N−1)) 只有 (N−2) 成立。
#:      N=3 實算:(N−2) ⇒ 0.1000·σ²(= Eq.30 的 1/10);(2−N) ⇒ 0.0667·σ²。
#:   ② Eq.(52-53) 的 Lyapunov 宣稱 K₃ 與 σ 同號,需要 N′ > 0;N′ < 0 時
#:      K₃ 反號,而原文對此一個字都沒交代。
#: ⚠️ 這個常數**只影響 `itcg_tgo_jiang` 這一欄記錄,不影響任何指令**。
#:   在我們的工作點(σ 中位 15.3°、r/V ≈ 40 s)兩式只差 0.24% 的 tgo
#:   ≈ 0.09 s ⇒ 選哪一邊都不會動到本輪結論。σ 到 60° 才拉開到 4%。
MPNG_NPRIME_2_MINUS_N = True

RESET_DICTS = (
    "slot_of", "slot_id", "arr",                       # ALIGN
    "align_ok", "ready", "perch_arr",                  # PERCH
    "released", "strike", "frozen", "alpha",           # DIVE / PN
    "commit", "thru_dyn", "att_block", "att_block_n", "handback_t",
    "e_t", "tgo_obs", "weave_theta", "tgo_cache", "d_hist",
    # ⚠️ itcg_side 是**鎖存**的方向,留著上一發的值 ⇒ 新目標的第一拍
    #    就往上一發的方向繞路,而兩發的幾何完全無關。
    "itcg_e", "itcg_a", "itcg_s", "itcg_side", "sigma_gate_hit",
    # ⚠️ B1:MPNG 的增益鎖存(mpng_k0)與本拍儀表(itcg_dbg)同樣要逐發清 ——
    #    增益是用**上一發放行當拍**的 R、t_go 推的,留著就等於用錯的尺度;
    #    儀表留著則會讓第二發的第一拍寫出上一發的 ε_T(假資料比缺資料更貴)。
    "mpng_k0", "itcg_dbg",
    "sigma_gate_starved", "sigma_wait_t0",
    "impact", "missed", "miss_why", "miss_pt", "slow_t0",
    "dmin", "hit_alt", "hit_d", "truth_dmin", "truth_hit",
    "wp_key", "wp_t", "wp_pt",                         # 航點/速度鎖存
    # ⚠️ S2 儀表:不清會讓第二發的第一拍帶著上一發的半徑寫進 CSV,
    #    而那一欄正是判「機制有沒有生效」的決定性量。
    "req_radius", "sent_radius",
    "zg", "zg0", "zg_warn", "zg_amsl",                 # 紅心地表(換目標=換地表)
    "terr_bull", "terr_home",
)
#: 要歸 None 的時刻與旗標(全部是「這一發打到哪」的時戳)。
#: ⚠️ dive_T / T_arr / dive_go 都是**絕對牆鐘時戳**,留著上一發的值比沒有更糟:
#:    _miss_check 的 (c) 條 `now > dive_T + window` 會在第二發的第一個 tick
#:    就成立 ⇒ 全隊瞬間判逾時 MISS。
RESET_NONE = ("t0", "T_arr", "dive_go", "dive_T", "done_t",
              "ring_t0", "runup_t0", "perch_t0", "perch_T")


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def dest(lat, lon, brg, d):
    b = math.radians(brg)
    return (lat + d * math.cos(b) / EARTH, lon + d * math.sin(b) / mlon(lat))


def dist_m(a, b):
    return math.hypot((b[0] - a[0]) * EARTH, (b[1] - a[1]) * mlon(a[0]))


def bearing(a, b):
    return math.degrees(math.atan2((b[1] - a[1]) * mlon(a[0]),
                                   (b[0] - a[0]) * EARTH)) % 360.0


class TotNode(Node):
    #: 常駐服務旗標的 **class 層預設**。__init__ 會依 tlat/tlon 覆寫成真值。
    #
    #  ⚠️ 為什麼要有 class 層預設而不是只在 __init__ 設:_tick() 會讀它,
    #     而既有的單元測試(tests/test_miss_exit.py 等)是**不跑 __init__** 的 ——
    #     它們手工組一個精簡 stub 只填自己要驗的欄位,這是本 repo 既有的測試模式
    #     (沒有 ROS2 runtime 就跑不了真的 __init__)。少了這行,2026-08-05 加入
    #     待命模式的那一刻,四條與待命完全無關的 DIVE 迴歸測試會一起 AttributeError。
    #     用 class 預設而不是 _tick 裡寫 getattr,是因為「批次航次 = 非待命」這件事
    #     應該只寫一次、寫在型別上,而不是散在每個讀取點。
    standby_mode = False
    # ⚠️ 同樣的理由(見上):觸水終結的三個純量也要有 class 預設,否則
    #    2026-08-06 加入它的那一刻,一整批與落海完全無關的迴歸測試會
    #    AttributeError: 'TotNode' object has no attribute 'kill_on_touch'。
    # ⚠️⚠️ 預設刻意是 **False**(不是 __init__ 用的 True):樁節點不該意外
    #    對任何機體送出 disarm。真正的預設值在參數宣告處,由 __init__ 覆寫。
    #    killed / _kill_lo 兩個 dict **不放 class 層** —— 可變物件放類別會
    #    在實例之間共用;有 kill_on_touch=False 擋著就到不了它們。
    # ── ⭐ 2026-08-14 大整改新增的類別層安全預設 ──────────────────────
    # ⚠️⚠️ 理由與 kill_on_touch 完全相同:樁節點(跳過 __init__)不該意外
    #    啟用任何新功能。**每一個都取「停用」值**,真正的預設在參數宣告處。
    # ⚠️ 可變物件(dict)必須放 __init__ 不能放這裡 —— 放類別會在實例間共用。
    #    下面用 `= None` 佔位,消費端一律 `or {}` / `is None` 保護。
    predict_hz = 0.0            # 0 = 不跑線上預測
    predict_thr_max = 100.0
    predict_gamma_cap = 0.0
    tgo_from_predictor = False   # T1:預設沿用靜態幾何配速
    tgo_predict_dt = 0.5
    tgo_ring_transit = False   # 類別層安全預設(樁節點跳過 __init__)
    itcg_on = False
    itcg_gain = 0.0
    itcg_a_max_g = 0.0
    # ── ⭐⭐ B1:Jiang 2021 MPNG(Aerospace 8(8):231,CC BY)的 Eq.(41) ──
    # ⚠️ 預設 False = **逐位元**走現行的 surplus/σ_need 路徑。
    #    這是本專案的既有紀律(S1/S2 兩顆開關同樣寫法):新律要能一顆開關
    #    退回舊行為,否則「開了之後變差」與「開之前就壞了」分不開。
    mpng_on = False
    # K_ε / K_s,量綱 1/(m·s)。0 = 走論文的建議式 40/(r₀·tgo₀) 自動推。
    mpng_k_eps = 0.0
    mpng_k_s = 0.0
    itcg_e = {}
    itcg_a = {}
    itcg_s = {}
    itcg_side = {}
    release_sigma_spec = {}
    sigma_gate_hit = {}
    sigma_gate_starved = {}
    sigma_wait_t0 = {}
    release_sigma_timeout = 0.0
    ring_pace = False
    release_at_ring = False   # S1:抵環即放行(純幾何,不看時刻表)
    wait_goto_radius = False  # S2:wait 段也飛過去,不繞圈
    req_radius = {}           # 儀表:本拍**要求**的半徑(不管有沒有送出去)
    sent_radius = {}          # 儀表:最後真正送出去的半徑
    dive_from_runup = False   # S1:內切一開始就翻 DIVE,不在環上等
    _ring_transit_warned = False
    d_hist = {}
    weave_time_sync = False      # T3:預設不做蛇形
    weave_theta_max = 45.0
    weave_deadband_s = 0.5
    state_max_age = 0.0         # 0 = 不做狀態新鮮度檢查
    att_require_fc = False
    att_require_guided = False
    att_eas_floor = 0.0
    att_agl_floor = 0.0
    att_handback = False
    param_verify = 0
    stall_prev_glide = 0
    hold_alt_cfg = 0.0          # 0 = 等待段用 perch_of()
    hold_v_cfg = 0.0            # 0 = 等待段用 vmin
    tecs_pitch_min = 0.0        # 0 = 跟隨 dive_pitch_min
    pn_alpha_tau = 1.75
    tick_dt = TICK_PERIOD_S
    tick_hz = 1.0 / TICK_PERIOD_S
    _pred_warned = False
    _tgo_warned = False

    kill_on_touch = False
    kill_agl = 0.0
    kill_confirm_n = 2
    # ⚠️ 預設 1e9 = **永遠解除不了保險**，樁節點不該意外終結機體。
    #    真正的預設在參數宣告處，由 __init__ 覆寫。
    kill_arm_agl = 1e9
    # ⚠️ 同上:待命告警的三個純量也要有 class 預設,樁節點才不會 AttributeError。
    #    `_standby_pos_ok = True` 在這裡是**安全**的方向 —— 樁節點不會因為
    #    class 預設而多印一行不存在的告警。真正的初值由 __init__ 設成 False。
    _standby_pos_warn_t = 0.0
    _standby_pos_ok = True
    _link_desc = "(未知)"

    def __init__(self):
        super().__init__("tot_node")
        g = self.declare_parameter
        g("n", 6)
        # ⚠️⚠️ tlat/tlon 的預設值 2026-08-05 從 (25.1406, 121.2325) 改成 **NaN**。
        #    理由是「安全預設」:沒給目標就該待命,不該拿一個寫死的桃園座標
        #    自己開打。舊預設的失敗形狀是最壞的一種 —— 沒有錯誤、沒有警告,
        #    六架飛機朝一個沒有人指定的點衝下去。
        #    ⚠️ 現有四個呼叫端(stot_trial.sh:376、stot_kpi_loop.sh:55、
        #       swarm_tot_up.sh:32、swarm_tot_jet_up.sh:24)**全部都明確傳值**,
        #       所以這個改動對批次航次是零影響;tests/test_tot_standby.py 有一條
        #       迴歸鎖去 grep 它們,將來有人把 -p tlat 拿掉會直接紅。
        # ⚠️ ROS2 的 YAML 解析:NaN 要寫 `.nan`。寫成 `nan` 會被解析成**字串**,
        #    參數型別檢查當場拋 InvalidParameterTypeException(本專案踩過)。
        g("tlat", float("nan")); g("tlon", float("nan")); g("talt", 90.0)
        g("link_base", 6760); g("link_stride", 10)  # MAVLink 分流(可改 6960 避開 AeroPlan 的 6760)
        g("approach_from", 180.0); g("r_fan", 2000.0); g("half_angle", 30.0)
        g("vmin", 14.0); g("vmax", 30.0); g("plan_frac", 0.9); g("margin", 40.0)
        g("reach", 120.0); g("layer_m", 12.0)
        # ── 終端俯衝(視覺震撼):爬到 perch_alt 高點 → 同步 pushover → 全速俯衝 ──
        g("perch_alt", 280.0)     # 高點佔位高度(配 pushover_r 讓俯衝約在 T 觸底)
        g("pushover_r", 480.0)    # pushover 環半徑(距 T;越小俯衝越陡)
        g("dive_alt", 5.0)        # 俯衝瞄準高度(壓低 → 持續最大俯角、穿越 T 時貼地)
        g("dive_layer", 4.0)      # 終端微高度分層(防撞):各機差 4m 垂直
        g("overshoot", 800.0)     # 穿越瞄準:瞄 T 後方 overshoot m,直線俯衝「穿過」T
                                  # (非到 T 繞圈 → 固定翼 loiter 半徑 ~66m 打不到點)
        g("switch_r", 130.0)      # 兩段俯衝:d>switch_r 瞄準 T(俯衝「砸向」T、貼地),
                                  # d<switch_r 才改瞄穿越點穿過去(未進 loiter 半徑前切換)
        g("dive_speed", 42.0)     # 俯衝指令空速(> 頂速 → TECS 壓機頭追速 = 俯衝+終端加速);
                                  # 機體端須配 AIRSPEED_MAX 放寬 + TECS_SINK_MAX + PTCH_LIM_MIN 陡
        g("dive_angle", 25.0)     # 設計俯衝角(度)。⚠️ 不是想要幾度就幾度:實測本機
                                  # 持續可達 25.7-30.4°(瞬時可到 52-63°,但撐不住)。
                                  # 硬綁 45° 的結果是掉不到底、從紅心上空 244-720m 飛過。
                                  # 這個角度用來「反推 pushover 環半徑」,見 auto_ring。
        g("auto_ring", True)      # True = 由 (perch_alt, dive_alt, dive_angle) 反推
                                  # pushover_r = (perch_alt-dive_alt)/tan(dive_angle),
                                  # 保證所需下滑角落在可達範圍內 → 真的砸得到紅心。
        g("perch_alt_gate", 0.85) # PERCH→DIVE 也要看高度:需達 perch_alt 的這個比例。
                                  # 舊版只看距離 → 還沒爬到就開始俯衝(實測只到 778-1187m
                                  # 而非指令的 1500m),下滑角自然不夠。
        g("track_frac", 1.0)      # ⚠️ 2026-07-27 改回 1.0(原 0.65/0.50)。下面那套
                                  # 「飛機只追得到指令的 0.5-0.65 倍」是在**航點每 tick
                                  # 重發**的年代量的 —— 高度斜坡一直被 set_target_
                                  # altitude_current() 重錨,飛機當然追不到,於是補了
                                  # 一個 2 倍的過度補償。_goto 鎖存修好之後補償就過頭:
                                  # 實測 B3 六機在**距目標 1000-1500 m 就已經觸地**
                                  # (alt 一路等於各自的 z_g),然後貼地滑行進紅心 ——
                                  # 「命中高度 0」是這樣來的假象,地面摩擦還把地速從
                                  # 45-55 拖到 19-38 m/s,命中散布因此爆到 13 s。
                                  # 下面保留原始記錄供對照:
                                  # 下滑線「實際追蹤率」。三次實測一致:飛機只做到指令
                                  # 角度的 ~0.65 倍(指令45°→實得25.7-30.4°;25°→15.6°;
                                  # 15°→10°)。所以航點高度要照 角度/track_frac 去要,
                                  # 才會真的沿著幾何下滑線落到紅心(等效:瞄準點壓到地面
                                  # 以下)。不補這個係數 → 永遠在紅心上空 220-250m 掠過。
        g("ground_alt", 0.0)      # 紅心所在地表的「相對起飛點高度」(m)。SITL 的
                                  # relative_alt 是相對 HOME,目標區地勢不見得同高:
                                  # 實測撞地時讀到 -39~-55m,代表紅心地表約在 -45m。
                                  # 設好它 → 下滑線終點落在真正的地表,且命中高度會以
                                  # 「離紅心地表多高」回報,真正命中 = 0。
                                  # ⚠️⚠️ 上面那個 -45m 是 **SITL 地形模型**的值,不是海面。
                                  #    本任務的紅心在台灣海峽開闊海面 —— 驗收基準是
                                  #    **0 m AMSL(海面)**,而 SRTM 在水域沒有資料、
                                  #    回的外插值讓 SITL 的「地面」落在 -13~-15m AMSL。
                                  #    也就是說照地形庫瞄,會瞄到海平面以下十幾公尺。
                                  #    海上目標一律用 target_amsl=0.0 釘死,別用這個。
        g("align_tol", 30.0)      # 進俯衝前的「橫向對正」門檻(m)。飛機必須先貼上自己
                                  # 那條「紅心徑向線」才准俯衝 —— 對正在 STOT 之前做完,
                                  # 終端就不必再修正,才能又快又準。
                                  # (只靠終端捕獲去修 → 要嘛慢速盤旋才準,要嘛快但偏。)
        g("dive_pace", 0.0)       # 俯衝配速 (m/s)。0 = 自動取 0.8·dive_speed。
                                  # ⚠️ 要填「實際可達的俯衝地速」而不是指令空速:
                                  # 配速太樂觀(100 vs 實際 28-76)→ t_rem 立刻歸零 →
                                  # 全員爆全速、失去等待餘裕 → 命中散布 14.5s。
                                  # 配保守一點(≈vmin)才有餘裕讓近機慢下來等遠機。
        g("dive_radius", 30.0)    # 俯衝航點的 param3 繞圈半徑(見 _reposition)
        g("dive_spdweight", 0.0)  # 進俯衝時推給飛控的 TECS_SPDWEIGHT。
                                  # 0 = pitch 只追高度(俯得凶,但空速沒人管會衰減:
                                  #     實測終端 gs 掉到 31-39 m/s、俯衝拖成 250s、
                                  #     同時性散布 19.8s —— 變成慢速盤旋而不是俯衝)。
                                  # 1 = 高度/空速平衡(保速、夠快、同時性好)。
        g("line_tol", 80.0)       # 高於下滑線這麼多就重發一次航點(重新建立較陡的斜坡)
        g("resend_min_dt", 3.0)   # 重發最小間隔(秒)。無限制重發會一直重設高度斜坡。
        g("expend_on_impact", True)  # 命中即視為消耗(不再下令、不脫離)
        # ── 觸水終結(SITL 不會因為落海而終結機體)────────────────────
        # ⚠️⚠️ SITL 沒有「水」的概念:機體砸進海面之後會**沉到水面下**再
        #    **爬回空中**,然後飛第二趟。TWO1(2026-08-06)實測:
        #      UAV1 agl 最低 −87.9 m → 觸地後 79 筆爬回 +15.7 m
        #      UAV2 agl 最低 −123.2 m → 觸地後 233 筆(11.7 s)爬回 **+1103.2 m**
        #    要堵的是本節點**現場**的 impact 鎖存:它只看水平距(見 DIVE 段的
        #    `d < self.impact_reach or passed`),**完全不看高度**,而 miss 窗
        #    有 60 s ⇒ 一架在紅心旁 > impact_reach 觸水、因此還沒鎖存的機,
        #    爬回來重穿就會被記成 impact,污染摘要行與 `grep -c "命中紅心 @"`。
        #    (計分器那一側 stot_score.py 的「AGL≤0 無條件收窗」已於 2026-08-04
        #     堵死,所以這裡修的不是計分器。)
        # ⚠️ ArduPlane 的 CRASH_DETECT 幫不上忙:is_flying.cpp 要求
        #    control_mode == AUTO,而 STOT 全程 GUIDED ⇒ 永不觸發。
        g("kill_on_touchdown", True)  # 觸水即強制 disarm(False = 回到舊行為)
        g("kill_agl_m", 0.0)          # AGL ≤ 此值視為觸水(m,離紅心地表)
        g("kill_confirm_n", 2)        # 連續幾筆才動手(20 Hz ⇒ 0.1 s),防單筆跳點
        # ⚠️⚠️ 解除保險門檻：要先量到 agl ≥ 此值，這架才可能被判觸水。
        #    一架從來沒飛起來過的飛機不可能觸水（見 _kill_arm_update）。
        g("kill_arm_agl", 50.0)
        g("perch_band", 40.0)    # 抵環後要求的高度收斂帶(m,離 perch_alt)
        g("perch_settle", 25.0)  # 收斂等待上限(s),超時就認賠放行避免卡死
        g("impact_settle", 4.0)   # 全員命中後仍續錄這麼多秒才結算(見 DIVE 收尾)
        g("dive_hdem_tconst", 0.0)  # 進俯衝時推給飛控的 TECS_HDEM_TCONST(0 = 不動)。
                                  # ⚠️ 高度需求的濾波時間常數,預設 3.0 s —— 這是
                                  # 「俯衝角追不到」的機體側瓶頸:實測指令 10°,
                                  # 六機只做到 5.4-7.2°(比例 0.54-0.72),而且指令
                                  # 越陡比例越高(B3 指令 20° 做到 18.6°)⇒ 是**固定
                                  # 滯後**不是固定比例,正是一階濾波的特徵。
                                  # 只在俯衝段調小,巡航段不受影響。
        g("perch_reach", 70.0)    # 認定「已就位 pushover 環」的半徑
        g("impact_reach", 40.0)   # 認定「已穿越 T(衝擊)」的水平半徑(另有通過最近點判定)
        # 落地與否的高度門檻。上面的水平判定完全不看高度,所以要靠這個把「高空掠過」
        # 從「真的砸下去」裡分出來(30 m 與 KPI 的 ALT_MAX 同源)。
        g("impact_alt_ok", 30.0)
        # ══ miss 出口(M1/M2)══════════════════════════════════════════════
        # ⚠️⚠️ 為什麼一定要有:`impact` 這個集合**沒有 miss/timeout 出口**。
        #    SW1 實測的逃逸鏈:U2/U6 首航次從紅心旁 638/469 m 掠過(> switch_r)
        #    ⇒ commit 不成立 ⇒ passed 恆 False ⇒ 永遠不鎖存 ⇒ 俯衝指令繼續發,
        #    而 v_req = d/max(t_rem, 0.05) 在 t_rem 深度為負時飽和成滿速
        #    ⇒ U2 飛到 207 km、U6 412 km 之外,數小時後隨機重穿紅心才鎖存。
        #    後果是摘要行的每一個數字都被污染:命中散布 11950 s、命中高度
        #    {2: -16, 6: 2} 看起來還「很好」,其實是幾百公里外繞回來的。
        # ⚠️ 這裡**不是**放寬命中判定。重放已反證:把 passed 放寬到 dmin<800
        #    會讓 U2 在 1498 m、U6 在 1069 m 高空被判命中 = 幽靈命中。
        #    miss 就是 miss —— 出口的用途是讓航次能結案,不是讓脫靶變命中。
        g("miss_far_m", 20000.0)    # dive_go 之後離目標水平距超過這個 ⇒ 判逃逸。
                                    # ⚠️ 只在 dive_go 之後檢查:之前蜂群還在 35 km
                                    #    的扇形上,提前檢查會全軍誤判。
                                    # 20 km 的依據:SW1 正常四架 dive_go 後離目標
                                    # 最遠 4.1–5.3 km,逃逸兩架 207/412 km ——
                                    # 分離度 40 倍以上,門檻放哪裡都一樣。
        # ── 最近水平距(dmin)沒有收斂窗參數,這是刻意的 ──────────────────────
        # ⚠️⚠️ 2026-08-04 移除 dmin_hyst_m / dmin_hold_s。曾經有一個「鎖存後
        #    d 從最小值回升 1000 m 才收窗」的遲滯窗,實測收窗要 31.7–45.5 s,
        #    而那段時間**機體早就在海面下了** —— SITL 不終結觸海的機體,SW1 的
        #    U3 掉到 −95.7 m 又爬回 +331 m、24.1 s 後再穿一次紅心。遲滯窗把那
        #    第二趟收了進來:U3 32.6→22.6(剛好跨過 MISS_MAX=30 變成「通過」)、
        #    U5 217.9→67.1(差 3.2 倍)。那是把指標改好看,不是改準。
        #    現在 dmin 與 hit_alt 一律在 impact / missed **鎖存當下**凍結。
        # ⚠️ 代價要講清楚:鎖存條件是 d < impact_reach,終端取樣步長 3–9 m
        #    ⇒ 鎖存當下的 dmin 是首航次真最近點的**上界**,誤差最多一個取樣步。
        #    要更細的量,用 stot_score.py(點到線段內插 + 觸地收窗),那支才是
        #    量測的權威;摘要行只是航次現場的粗值。
        g("miss_far_k", 2.0)        # miss_far_m 的**自動下限倍數**:
                                    # dive_go 當下若 miss_far_m <
                                    # k × max(環半徑, 各機當下距 T),就把
                                    # miss_far_m 夾寬到那個值並印生效值。
                                    # ⚠️⚠️ 逃逸門檻與 pushover 環半徑是耦合的:
                                    #    ring = (perch_alt−dive_alt)/tan(dive_angle)
                                    #    ⇒ dive_angle 25° 是 6.4 km、5° 是 34 km;
                                    #    perch 6000 m + 15° 也有 22.4 km。
                                    #    環半徑 ≥ miss_far_m 時,DIVE 的第一個 tick
                                    #    會把**全隊同時**判成逃逸、整趟 0/6 結案,
                                    #    看起來跟飛行失敗一模一樣。只印警告擋不住
                                    #    (下一個 tick 就動手),所以這裡真的夾。
                                    # 0 = 關掉自動夾(只留「已放行才檢查」那道)。
        g("miss_slow_gs", 5.0)      # 地速低於此(m/s)…
        g("miss_slow_sec", 60.0)    # …且持續這麼久 ⇒ 判停滯(墜毀/卡住)
        g("miss_window_s", 60.0)    # 首航次窗 = dive_go … (dive_T + 這麼多秒)。
                                    # 窗關閉時既未 impact 也未 missed 的機強制判 miss。
                                    # ⚠️⚠️ dive_T 是**絕對牆鐘時戳**(now + … + 2.0),
                                    #    不是時長。條件必須寫 now > dive_T + window;
                                    #    寫成 now - dive_go > dive_T + window 是死碼
                                    #    (拿 ≤ 1.2e4 的時長比 ~1.79e9 的 epoch,恆假)。
        # 目標地表 AMSL。NaN = 向地形資料庫查(陸地目標用)。
        # **海上目標一定要釘 0.0** —— 見 TERRAIN_REPORT 處理裡的說明。
        g("target_amsl", float("nan"))
        # 接近告警門檻。防撞靠高度分層,所以「水平近」不算失敗,
        # 「水平近**且**垂直也近」才算。預設 200/50 是 P91 實測的安全側
        # (最近 348m/82m,零次同時觸發)。
        g("prox_h", 200.0); g("prox_v", 50.0)
        # ══ 終端比例導引(PN):最後一段改下姿態,繞開 L1/TECS ═══════════════
        g("pn_enable", False)   # 實測:PN 版本三軸全面變差(誤差 21-24m、高度 37-53m、散布 14-24s,且 9/12 逾時),暫時停用,改由航點導引基準線出發
        g("fc_base", 5762); g("fc_stride", 10)
                                  # ⚠️ 直連飛控 SERIAL1,不走 MAVProxy 分流:
                                  # 實測同一道 SET_MESSAGE_INTERVAL,分流上 uav3
                                  # 只拿到 5 Hz 而 uav1 有 50 Hz(ACK 都是 0),
                                  # MAVProxy 會把速率搶回去。直連三次都穩 50.2 Hz。
        g("tel_us", 25000)        # 遙測間隔(µs)。⚠️ 不能填 20000:
                                  # cap = loop_period_us/800 = 25 ms,要求更快會回
                                  # MAV_RESULT_DENIED(實測 result=2),然後你就默默
                                  # 拿 5 Hz 的狀態去跑 50 Hz 的導引律。
        # ⚠️⚠️ 2026-08-14:這個宣告值**與生效值不符**,而且不符了很久。
        #    `_pn_cmd` 只從 `_tick` 被呼叫,而 `_tick` 是 `create_timer(0.05)` = **20 Hz**
        #    (見下方 TICK_HZ)。所以姿態指令實際是 20 Hz 出去的,不是 50。
        #    ⇒ 它唯一的消費者 `self.pn_dt`(用於 α 濾波增益)因此把時間常數
        #      算成 0.7 s,實際卻是 0.7 × 50/20 = **1.75 s**,慢 2.5 倍。
        #    保留這個參數只為相容;真正生效的速率由 TICK_HZ 導出並印進契約行。
        g("pn_hz", 50.0)          # ⚠️ 名義值,非生效值。生效速率見 TICK_HZ
        # α 濾波的時間常數(s)。⚠️ 預設 1.75 是**逐位元還原舊行為**的值:
        # 舊式 k = (1/50)/0.7 = 0.0285714…,新式 k = (1/20)/1.75 = 0.0285714…,完全相同。
        # 要調 α 濾波請改這個,不要改 pn_hz —— 改 pn_hz 以前只會動增益不會動速率。
        g("pn_alpha_tau", 1.75)
        # ⭐ S3:TECS 自己能要求的俯仰下限,與姿態迴路的 PTCH_LIM_MIN_DEG 分離。
        # ⚠️ 預設 0.0 = 「跟隨 dive_pitch_min」= **逐位元維持舊行為**。
        #    設成負值(建議 −15)才會拆開:PN 走姿態通道不經過 TECS,所以 TECS
        #    不需要 −80;而綁在一起時,任何一次掉回 TECS 它就有 −80 可以拿高度
        #    換速度(實測「alt 227 → −5.9,11 秒撞地」)。
        g("tecs_pitch_min", 0.0)
        # ══ S4/S5/S6 安全兜底(2026-08-14)══════════════════════════════════
        # ⚠️⚠️ 全部預設 = 現行行為。這一組是為了「把姿態通道往前延伸到滑翔段」
        #    而準備的:那等於把能量管理從 TECS 手上接過來,而 TECS 原本在兜底。
        #    ⇒ 兜底必須先存在,才准啟用接管。順序不可以反。
        #
        # S4a 狀態新鮮度看門狗。0 = 停用(現行)。
        # ⚠️ 全檔先前**沒有任何 stale 檢查**:狀態凍結時指令照樣 20 Hz 送出去,
        #    於是 GUIDED_TIMEOUT 永遠不會觸發,飛機用一個凍結的姿態飛進地面。
        #    leader-state-freeze 記過同一個形狀(停發有告警、凍結沒有)。
        #    建議 0.5 s = GUIDED_TIMEOUT(3.0)的 1/6。
        g("state_max_age", 0.0)
        # S4b 禁止姿態鏈路靜默降級。False = 現行(fc 缺席就退到 6860 分流線)。
        # ⚠️ 那條分流線只有 5-20 Hz 而且有背壓失效史(wp_link_drain)。
        #    姿態指令掉到那條線上不會有任何訊息,只會變得又慢又抖。
        g("att_require_fc", False)
        # S4c 參數讀回驗證。0=關 / 1=讀回並印生效值 / 2=讀回失敗就不准進 DIVE。
        # ⚠️ 俯仰權限的解鎖是 fire-and-forget,而且只在一個一次性視窗裡送,
        #    最壞情況只有一個 tick。「送出去了但沒吃進去」是已知會發生的。
        g("param_verify", 0)
        # S5a 主動交棒:釋放姿態通道前先把 guided 航點高度重錨到當下高度。
        # ⚠️ 不做的話,TECS 帶著整段下降累積的**過期 hgt_dem** 接手,
        #    在 200 m/s 下是滿油門 + PTCH_LIM_MAX 的暴力拉起,而且全程靜默。
        g("att_handback", False)
        # S5b/S5c 節點端地板。0 = 停用。
        # ⚠️ EAS 不是地速:157 是 3100 m 的**地速**(= EAS 135),
        #    拿 157 當 EAS 門檻會超天花板 17%(eas-vs-groundspeed-157)。
        g("att_eas_floor", 0.0)
        g("att_agl_floor", 0.0)
        # S5d 失速保護相位化。0 = 不動 .parm(現行 STALL_PREVENTION=0)。
        # ⚠️ 姿態通道下 update_load_factor 是**唯一還活著**的空速保護,
        #    而 .parm 把它關掉了。滑翔段要開、進 DIVE 要關(陡俯衝不能被它限制)。
        g("stall_prevention_glide", 0)
        # S6 姿態通道啟用前確認飛控真的在 GUIDED。False = 現行(不檢查)。
        # ⚠️ 節點完全不讀 HEARTBEAT、不追飛控模式。目前 GUIDED 是靠 _reposition
        #    的 param2=CHANGE_MODE 副作用進去的;若滑翔段在那之前就送姿態,
        #    SET_ATTITUDE_TARGET 會被**靜默丟棄**(同 reposition-needs-change-mode)。
        g("att_require_guided", False)
        # ══ S8 下降式等待(2026-08-14)══════════════════════════════════════
        # ⚠️⚠️ 這是第一個真的改變飛行行為的開關,兩項預設都 = 現行行為。
        #
        # 【要解的問題】RPQ 三趟證明:守環修好之後每一架都**完美守住 perch 3100**
        #   (交接高度 3054–3299,等於卸掉 −53 m),於是確定性地落在失敗帶。
        #   而幾何上:環半徑 = perch/tan(dive_angle) = 6084;
        #   gate 上界 = 環 − hold_radius = 4784;
        #   交接高度釘在 perch 時最小 γ = atan(3100/4784) = 32.9° > 全中帶 29.1°。
        #   ⇒ **只要交接高度停在 perch,任何 gate_r 都無解。必須先卸掉 ≥437 m。**
        #
        # 【為什麼不能直接調低 perch_alt】
        #   auto_ring 會讓環半徑跟著等比例縮小(ring = perch/tan γ),
        #   γ = atan(perch/gate) 因此**完全不變** —— 那是這個幾何的陷阱。
        #   ⇒ 必須把「等待高度」與「環半徑」**解耦**:環仍由 perch_alt 定,
        #     但等待時飛在更低的 hold_alt 上。
        #
        # hold_alt:等待/守環段的目標高度(m,相對 HOME)。0 = 用 perch_of()(現行)。
        # ⚠️ 取 min(perch_of(i), hold_alt) —— 只准比 perch 低,不准更高,
        #    否則 perch_step 的高度階梯會被這一個純量抹平。
        g("hold_alt", 0.0)
        # hold_v:等待/守環段的速度指令。0 = 用 vmin(現行)。
        # ⚠️⚠️ 這一項是 hold_alt 能不能真的執行的**前提**:
        #    tot_node 在守環時同時下令「降到目標高度 + v_cmd=vmin(95)」,
        #    而實測 12/12 架次地速比指令高 +1.6~+61 m/s ⇒ 速度誤差恆為負
        #    ⇒ 油門需求打到 THR_MIN=0 ⇒ 觸發 AP_TECS.cpp:1245-1252 的抗失控閂鎖
        #    (油門觸底就把降高需求扣在上一拍,而且**不設旗標**)⇒ 每一次都自己
        #    把下降權限關掉。把守環速度提到計畫巡航值,油門才會離開地板。
        g("hold_v", 0.0)
        # ══ S9 線上預測器:影子模式(2026-08-14)════════════════════════════
        # ⚠️ predict_hz = 0 ⇒ **完全不跑**,零開銷、零行為改變(現行)。
        #    > 0 ⇒ 每 1/predict_hz 秒對每架跑一次正向積分,**只記錄不致動**。
        #
        # 為什麼要先影子跑一輪:Leblebici 的可達性觸發要用預測值來決定何時俯衝,
        # 而預測器在**飛行中**的表現我們一次都沒量過。離線用實測 γ(t) 重放的
        # RMS 是 5.4 m/s,但那是「已知答案再對答案」;線上是「用當下狀態外推」。
        # ⚠️⚠️ 而且已知它會系統性樂觀:用 RPQ 的交接狀態外推,預測「會命中」,
        #    實際殘餘 416–3084 m —— 因為互動路徑的 PN 實得俯角只有 1.0–30.1°
        #    (中位 11.5°),遠低於預設 cap 33°。影子模式就是要把這個偏差量出來。
        g("predict_hz", 0.0)
        # 預測器假設的 THR_MAX。⚠️ 設計 25 與實跑 100 差 4 倍,預測會差 6–9 倍靈敏度。
        g("predict_thr_max", 100.0)
        # 預測器夾的可達俯角。0 = 用模組預設(實測 33°)。
        g("predict_gamma_cap", 0.0)
        # ══ T1 用預測器的積分 t_go 取代靜態幾何配速(2026-08-14)══════════════
        # ⚠️ False = 沿用 `d / _pace_of(i, d)`(現行行為,逐位元不變)。
        #
        # 【為什麼要換】散布 = 0.76 × 交接時間差 + 1.62(R²=0.779, n=5)——
        #   俯衝段幾乎原樣傳遞上游時間差,**沒有修正能力**(速度兩端都飽和)。
        #   而放行判定用的 `d/_pace_of` 是**靜態幾何式**不是預測:
        #       pace = max(10, dive_pace·cos(γ_geom)/cos(γ_design))
        #   它假設 150 m/s,實測平均閉合只有 121 ⇒ Zarchan 的尺度因子 SF = 1.24,
        #   外加 +2.9~+41.8 s(中位 +11.6)的偏置。
        #
        # 【要求的精度】散布 ≤3 s ⇒ 交接時間差 ≤1.8 s(我方擬合)/ 1.7 s(Zarchan
        #   傳播式)—— 兩條獨立路徑吻合。對應 SF ∈ [0.96, 1.04]、BIAS ≤ 0.5 s。
        #   **靜態幾何式在數學上就達不到**,不是調參問題。
        #
        # ⚠️ 換成積分 t_go 只消掉**系統性偏置**,它仍是開迴路 ——
        #    真正要把 e_t 逼到零需要閉迴路(見 T3 的 weave 振幅調變)。
        #    這一項是那個閉迴路的**前提**:估計不準,閉迴路就在追一個錯的目標。
        g("tgo_from_predictor", False)
        # 積分 t_go 的更新節流(秒)。0 = 每次呼叫都積分(貴)。
        g("tgo_predict_dt", 0.5)
        # ⭐ 把「還要多久才進得了 pushover 環」併入 t_go(RPZ1 的 +37.2 s)。
        # ⚠️ 宣告預設刻意留 False:餵值路徑斷掉時契約層才抓得到。
        g("tgo_ring_transit", False)
        # ── ⭐⭐ ITCG:命中時間控制(2026-08-15,使用者定調傳統 PN 不適合多機)──
        # ⚠️ 宣告預設一律留「關閉/無效」值,餵值路徑斷掉時契約層才抓得到。
        g("itcg_on", False)        # 時間項總開關
        # ⚠️ itcg_gain 是**純修整係數**(無因次):1.0 = 「正好抵銷 PN 對前置角
        #    的回復加速度」。舊註解寫的 `a = K·V²/R·(e_t/R)` 早在 2026-08-15
        #    就被換掉了,契約表 181 行的說明字串一併訂正。
        g("itcg_gain", 0.0)
        g("itcg_a_max_g", 0.0)     # 橫向偏置上限(g)
        # ── ⭐⭐ B1:Jiang 2021 MPNG(2026-08-19 使用者裁決導入)────────
        # ⚠️ 宣告預設 False = **逐位元**同現行的 surplus/σ_need 路徑。
        #    這一輪的首要目的不是「讓它變好」,是把**權限上界量準**:
        #    實測上界只有 +2.88 s,而 n=8 的需求是 7.3–9.0 s ⇒ 差一個數量級。
        #    在量準之前,任何「導入新律就解決散布」的說法都沒有依據。
        g("mpng_on", False)
        # K_ε / K_s(1/(m·s))。0 = 走論文的建議式 40/(r₀·tgo₀) 自動推。
        # ⚠️ 論文那個數字是他們 330 m/s **等速**、r₀=10 km、tgo₀≈30 s 的算例值;
        #    我們是 161.7 m/s 中位、r₀≈6854 m、tgo₀≈41.3 s 的**加速**俯衝 ⇒
        #    代進去 K≈1.41e−4,a_s = 14.78·ε_T m/s²,在 ε_T≈0.27 s 就撞上
        #    0.4 g 的夾制。也就是說在我們的工作點這條律**整段貼在夾制上**、
        #    退化成 bang-bang ⇒ 增益幾乎不影響結果,真正的自變數是
        #    `itcg_a_max_g`。要調就掃它,不要掃這兩個。
        g("mpng_k_eps", 0.0)
        g("mpng_k_s", 0.0)
        # ── ⚗️ 因果實驗專用:只在指定前置角放行 ────────────────────────
        # 空字串 = 停用(生產組態)。格式 "sysid:目標σ度:容差度,..."
        # ⚠️ 這條路徑**不是生產邏輯**,是為了把「放行航向」與「放行距離」分開。
        g("release_sigma_spec", "")
        # 航向閘門的餓死保護(秒)。0 = 不逾時(⚠️ 那會讓飛機被閘門餓死)。
        g("release_sigma_timeout", 0.0)
        # ── ⭐⭐ V1:用速度取代環上盤旋等待(2026-08-16)────────────────
        # ⚠️ 宣告預設 False = 逐位元舊行為(盤旋)。實測:盤旋讓兩架的時刻差
        #    在最後 3.24 km 從 4.4 s 放大到 40.2 s,而直線穿過只要 21–25 s。
        g("ring_pace", False)
        g("release_at_ring", False)
        g("wait_goto_radius", False)
        g("dive_from_runup", False)
        # ══ T3 時間誤差閉迴路:用蛇形航跡吸收 e_t(2026-08-14)══════════════
        # ⚠️ False = 等待段不做蛇形(現行行為)。
        #
        # 【為什麼是蛇形而不是速度】速度迴路兩端都飽和,已無權限;
        #   而最短飛行時間是**高度受限**(實測 d=4000 與 d=8000 的 t_min 相同)
        #   ⇒ 壓平航跡不會變快,蛇形**只能延遲不能加速**。
        #   所以這個迴路只對「太早到」(e_t > 0)作用,對「太晚到」無能為力 ——
        #   後者必須靠協商時把 t_c 訂在所有 t_min 之上。
        #
        # 【為什麼不需要調增益】路徑拉長倍率 = 1/cosθ,所以
        #       t_go_新 = t_go / cosθ  ⇒  cosθ = t_go / (t_go + e_t)
        #   θ 是**解析解**不是比例增益。閉迴路來自 t_go 每拍重新積分。
        g("weave_time_sync", False)
        # 蛇形的最大偏航角(度)。⚠️ 上限另受**換向率**約束:
        # ω = g·tanφ/V,把航向從 +θ 甩到 −θ 要 2θ/ω,而它必須 ≤ 半週期的一半。
        # 157 m/s、45° 傾角 ⇒ ω ≈ 3.58 °/s ⇒ 週期 22 s 之下 θ 只能到 ~9.8°。
        # 這個夾限在程式裡現算,不是寫死。
        g("weave_theta_max", 45.0)
        # 死區(秒):|e_t| 小於它就不動,避免在雜訊上抖動。
        g("weave_deadband_s", 0.5)
        g("gate_r", 1200.0)       # 交接水平距(m)。⚠️ gate_theta_max > 0 時**不生效**,
                                  # 交棒距改由 gate_of() 每拍算(見那裡的完整論證)。
        # ── 動態交棒(2026-08-06)────────────────────────────────────────
        # 綁環半徑的固定比例是**錯的參數化**:828 架次全掃,比值 f 對殘餘高度的
        # Spearman 只有 +0.103,而交棒視線俯角 γ 是 +0.892。要綁的是「要求 PN
        # 交出多陡」,不是交棒距離本身。gate_theta_max <= 0 ⇒ 完全退回絕對 gate_r。
        g("gate_theta_max", 0.0)  # PN 可持續達到的航跡角(度)。0 = 關閉動態交棒。
                                  # ⚠️ 實測 PN 只交得出 23.3–33.9°(三趟六架次),
                                  #    所以 35 是有依據的上緣,40 沒有。
        g("gate_phi", 0.80)       # 可行性餘裕:gate = d_min/Φ。Φ 越小交棒越早、越安全。
        g("gate_r_min", 2500.0)   # 交棒距下限(m)
        g("gate_r_max", 6000.0)   # 交棒距上限(m)。另外還會被本機環半徑夾住。
        g("gate_hold_n", 3)       # 連續幾拍成立才交棒(防單拍雜訊提早觸發)
        g("gate_h", 600.0)        # 交接高度(離本機自己的地表 m)。只有 gate_profile
                                  # 打開才會用到 —— 見下。
        g("gate_profile", False)  # 「交接盒」剖面:航點段不再直接瞄紅心地表,改成
                                  # 把飛機送到「距 T = gate_r、離紅心地表 gate_h」
                                  # 那一點,讓 PN 從較大的視線俯角接手。
                                  # ⚠️ 為什麼要這個:觸地誤差 ≈ 殘餘AGL / tan(終端俯角)。
                                  # 目前終端俯角約 19°,放大 2.9 倍,誤差 p90 已經
                                  # 4.14 m 貼著 5 m 門檻(實測成功率 93.9%,n=33)。
                                  # 放大倍率 30° 時降到 1.7、40° 時 1.2。
                                  # 壓殘餘 AGL 那條路已經到頂(pn_n_vert 5→7 反而更差:
                                  # 誤差 p90 4.14→6.52),所以要改動放大倍率本身。
                                  # ⚠️ 不能改用 dive_angle 讓整段變陡:auto_ring 會把
                                  # pushover 環拉進噴射的迴轉半徑內,蜂群站不住
                                  # (B9 實測 30° → 高度散 253-479 m、散布 18 s)。
                                  # 這裡只改**交接之前**的高度剖面,環的幾何不動。
                                  # ❌ **實測失敗,預設維持關閉。** gate_r=800 /
                                  # gate_h=350(視線俯角 23.6°)實跑:橫向完美
                                  # (最近通過 0.07-0.39 m)但**五機下不去**,
                                  # 從紅心上空 64-113 m 掠過,只有最慢那架落地。
                                  # 原因是 PN 的縱向角速率權限
                                  #   γ̇ = (nz − cosγ)·g / V   ——**與速度成反比**。
                                  # 保持高度再轉換成速度衝下來,gs 從 60-70 升到
                                  # 76-91 m/s,等於在最需要俯衝權限的時候把權限降低,
                                  # 實得只有 16.5-19.7° 而需要 23.6°。
                                  # ⇒ 「加大交接俯角」在這個架構下走不通;同一條式子
                                  #    指出的可行方向是**降低終端速度**(pn_vmax)。
        g("pn_n_lat", 4.0)        # 橫向導航常數(掃描 3~4 平坦,取 4 留延遲餘裕)
        g("pn_n_vert", 5.0)       # 縱向導航常數。3→5 是 2026-07-27 實測改的:
                                  # 觸地誤差 ≈ 凍結時的殘餘 AGL / tan(終端俯角)
                                  # (實測放大約 2.9 倍),所以要在凍結前把縱向
                                  # 視線角速率壓乾淨。實測 n=5 各跑 5 趟:
                                  #   n_vert=3:殘餘 AGL 0.96-1.75 m、誤差 2.64-5.35 m、2/3 過
                                  #   n_vert=5:殘餘 AGL 0.35-0.95 m、誤差 0.84-2.64 m、5/5 過
                                  # ⚠️ 不要改成「把 pn_freeze_r 調小」去解同一件事,
                                  #    那是擋 λ̇ 的 1/d² 奇異點用的,拆掉會整批崩掉
                                  #    (實測誤差 2083 m、散布 32 s)。
        g("pn_lead", 1.0)         # 縱向 γ 指令前導(s)
        g("pn_freeze_tgo", 1.5)   # t_go 小於此就凍結指令(但仍持續發送)
        g("pn_freeze_r", 100.0)   # 或距離小於此
        g("pn_vmax", 80.0)        # PN 油門律的速度上限(見 _pn_cmd 的說明)。
                                  # 原本硬寫 135 —— 終端衝太快,殘餘高度會被
                                  # 1/tan(俯角) 放大成縱向落點誤差。
        # ── PN 油門律的三顆旋鈕(見 _pn_cmd)────────────────────────────
        g("stagger_depart", False)  # ⭐ 分時獨立解脫:用時變 t_go 而非統一的空間
                                  # 座標 gate_r 決定何時交接 PN。編隊縱深 8 km 時,
                                  # 同一個圓的抵達時刻天生差 51 s,空間門檻表達不了
                                  # 同時性。False = 回到舊的 gate_r-only 行為。
        g("stagger_lead", 0.0)    # t_go 觸發的提前量(秒)。>0 = 更早解脫,
                                  # 給 PN 更多收斂距離;0 = 剛好卡在時刻表上。
        g("perch_step", 0.0)      # ⭐ 高度通道協同(Perch Altitude Timer)。
                                  # 各機 perch = perch_alt + perch_step×slot_id,
                                  # 於是環半徑 = (perch_i − dive_alt)/tanγ 也各不相同,
                                  # **俯衝時間**因此天然錯開 —— 時間協同從油門通道
                                  # 搬到高度通道,油門就能全部留給終端速度。
                                  # 27° 之下 dt/d(perch) ≈ 0.0093 s/m。0 = 關(舊行為)。
        g("dive_full_thr", False)  # 終端俯衝鎖全油門(放棄用油門調時間)。
                                   # 只有 perch_step > 0 時才該開 —— 否則同時性
                                   # 會失去唯一的執行器。
        g("dive_ring_gate", True)  # 放行前必須先抵達 pushover 環(見放行段的說明)。
                                   # 關掉的話俯衝會在抵環前 30 km 就開始,下滑線
                                   # 被攤平成 ~10°,環等於不存在。
        g("ring_gate_frac", 1.10)  # 「算抵環」的寬容倍率(環半徑的幾倍以內)。
        g("weave_period", 22.0)   # ALIGN 的 S 型換向週期(秒)。⚠️ 22 是泡棉機
                                  # 時代的值;157 m/s 的轉彎率只有 3.58°/s,
                                  # 半週期 11 s 之下 θ>19.7° 根本轉不完。
                                  # 高速機體要 120 s 才吸收得到設計的時間量。
        g("pn_los_floor", 0.0)    # 幾何前饋:指令俯角不准比「直指紅心的視線」淺。
                                  # 0 = 關(純 PN,舊行為);1.0 = 完全對著紅心。
                                  # ⚠️ 這是陡俯角唯一有效的開關 —— 純 PN 的縱向通道
                                  # 根本不引用 dive_angle,單改角度參數不會有效果。
        g("pn_nz_min", 0.05)      # PN 法向過載下限。0.05 = 不准負 g 推頭 ⇒ 俯角上限
                                  # 被它鎖死(推到 −80° 需要 9259 m 高度,我們只有 3148)。
                                  # 陡俯角要放到負值,例如 −0.2。
        # ⭐⭐ 俯衝段的 THR_MAX。**這是「全油門」真正的上限。**
        # ⚠️⚠️ `_pn_cmd` 送的 thr 是 0..1,韌體換算成 forced_throttle = thr×100
        #    之後再被 servos.cpp 夾到 [THR_MIN, THR_MAX] ⇒ **送 1.0 只等於 THR_MAX**。
        #    巡航設計值把 THR_MAX 壓到 25(推重比改裝),而 tot_node 先前
        #    **完全沒有斷言 THR_MAX** ⇒ 俯衝也只有 25%。
        #    實測 E11(uav3 dataflash,ThO 0–100):2000–2500 m 帶 ThO_max 22.6、
        #    3000–3500 m 帶貼著 25.0,1000–1500 m 帶 p50 直接是 0.0 ——
        #    「終端全油門」從來沒有真的全開過。
        #    先前沒踩到是因為改裝前 THR_MAX 一直是 100(JET_THRMAX 為空)。
        # ⇒ 與 TECS_SPDWEIGHT / PTCH_LIM 同理:相位相依的權限就該在進相位時斷言。
        # ⭐⭐ DO_REPOSITION 的 param3。**兩個用途語意相反,不可共用一個值。**
        #   goto_radius：「飛到該點」——要小,飛控才不會把它變成繞圈捕獲。
        #                 L1 仍會乘 eas2tas²(3100 m 約 1.365),30 → ~41 m。
        #   hold_radius：「守在該點」——要 ≥ 物理迴轉半徑,否則飛機飛不出那個圈。
        #                 60 m/s TAS、45° 傾角 ⇒ V²/(g·tanφ) = 367 m,取 400 留餘裕。
        # ⚠️⚠️ 上面那行是**泡棉機時代**(60 m/s TAS)的算式。噴射組態下
        #    vmin 95 EAS ⇒ TAS 111 @3100 m ⇒ 迴轉半徑 **1252 m**,而預設仍是 400
        #    ⇒ 守環退化成繞自己的最小圈。`_check_hold_radius()` 每次開機都會用
        #    ERROR 吼這件事(TWO1 的 trial log 就吼了),但沒有人能改它 ——
        #    因為它**先前沒有環境變數通路**。2026-08-06 補上 HOLD_RADIUS /
        #    GOTO_RADIUS 的 -p 通路,預設值不動,好讓這件事可以被受控實驗。
        # ⚠️ 先前兩者共用 radius=0.0 ⇒ 一律退回 WP_LOITER_RAD×eas2tas² = 2030 m,
        #    「飛過去」的那幾個 mode 因此永遠到不了(ALIGN 死結、抵扇形散布 94 s)。
        g("goto_radius", 30.0); g("hold_radius", 400.0)
        g("dive_thr_max", 100.0)
        g("dive_pitch_max", 45.0) # PN 姿態指令的抬頭上限(度)。⚠️ 要和飛控的
                                  # PTCH_LIM_MAX_DEG 一致,否則節點以為有 45° 權限、
                                  # 韌體只給 15°,兩邊不一致且無訊息。
        g("dive_angle_max", 0.0)  # 放行的「幾何守門員」:所需俯角 atan(h/d) 一旦到達
                                  # 這個上限就立刻放行,不管時刻表(0 = 關閉,舊行為)。
                                  # ⚠️ 等待中的飛機在環上會持續往內漂而高度不變,
                                  # 所需俯角因此一路變陡。實測 AB1 的 UAV3 被等到
                                  # 距紅心 2 km、高 3402 m 才放行 —— 需要 59.7°,
                                  # 而機體實得 35.3°,結果從 1518 m 上空飛過。
                                  # 這條讓「時間對齊」永遠讓位給「幾何可達性」。
                                  # 設成實測可達俯角(本機體約 35–40°)略低一點。
        g("perch_climb_v", 0.0)   # PERCH 階段的爬升空速(0 = 沿用 max(vmin·1.45, 65))。
                                  # ⚠️ 爬升速度必須用**最佳爬升速度**,不能沿用巡航值。
                                  # 爬升率 = V·(T−D)/W,而 D 隨 V² 上升 ⇒ 太快就沒有
                                  # 剩餘功率。低推重比機體差距非常大:實測 Q1 用
                                  # 101.5 m/s 指令(實際飛到 137)只爬 1.5–3 m/s,
                                  # 爬到 2790 m 花了 19 分鐘、整趟因此逾時;
                                  # 同一台機在 80 m/s 的剩餘推力是 39 N(vs 10 N),
                                  # 可爬 14.5 m/s。
        g("dive_pitch_min", 0.0)  # 進俯衝時才放開的低頭上限(度,負值)。
                                  # 0 = 不動(舊行為)。⚠️ 低頭權限是**相位相依**的:
                                  # 終端要 −80°,但巡航段開著 −80°,一旦油門飽和
                                  # TECS 就會拿高度去換速度、把飛機開進地面。
                                  # 低推重比機體(THR_MAX 壓過的)油門很容易飽和,
                                  # 實測 alt 227 → −5.9 只花 11 秒。
        g("pn_thr_trim", 0.36)    # 前饋油門。0.36 是抄 plane-jet.parm 的
                                  # TRIM_THROTTLE 36 來的,**綁在舊機體上**。
                                  # 換機體(尤其用 THR_MAX 改推重比)一定要重設:
                                  # SET_ATTITUDE_TARGET 的 thrust 是 0..1 對應
                                  # THR_MIN..THR_MAX,THR_MAX=19 時 0.36 只有 6.8%
                                  # 的伺服行程。
        g("pn_thr_gain", 0.030)   # 速度誤差 → 油門的比例增益(每 m/s)
        g("pn_late_full", True)   # 逾期共同命中時刻時全油門趕(見 _pn_cmd 的說明)。
                                  # False = 回到舊行為(退化成維持現速),只為了 A/B。
        # ══ 待命模式(執行期目標指定)═════════════════════════════════════
        # ⭐ 扇形要擺在**蜂群來的那一側**。approach_from 是「由紅心看出去、
        #    扇形所在的方位角」,預設 180 = 紅心正南 —— 那是批次航次寫死
        #    LOC=TAOYUAN、紅心固定在起飛點北方時才對的值。
        # ⚠️⚠️ 右鍵任意點時它幾乎一定是錯的:紅心若在蜂群南方,180° 會把扇形
        #    擺到紅心的更南邊 ⇒ 六架要先**穿過紅心**才飛得到 slot,ALIGN 段
        #    直接從目標頭頂飛過去。而這件事沒有任何檢查會擋 —— 航次照跑、
        #    照結案,只是誤差莫名其妙。
        # ⇒ 執行期目標一律改成「由當下蜂群質心反推 approach_from」,並把
        #   算出來的值印出來。訊息裡明確帶 approach_from 則以訊息為準。
        # ⚠️ 只作用於**執行期收到的目標**;啟動參數帶目標那條路徑一個位元都不動。
        g("auto_approach", True)
        g("csv", "/tmp/tot_node")
        p = self.get_parameter
        self.n = int(p("n").value)
        self.T = (float(p("tlat").value), float(p("tlon").value))
        # ⭐ 待命判定 —— **只看目標在不在**,不看別的旗標。
        #    `standby_mode` 是「這個節點是不是常駐服務」(啟動後不可變);
        #    `self.phase == "STANDBY"` 是「現在有沒有目標」(每一發之間會變)。
        #    兩者分開的理由:批次航次打完要停在 DONE(既有行為),
        #    常駐服務打完要回 STANDBY 等下一個目標。
        self.standby_mode = self.is_standby_target(*self.T)
        self.auto_approach = bool(p("auto_approach").value)
        self.strike_seq = 0        # 已接受的目標數(第 1 發沿用原本的 CSV 檔名)
        self.target_src = "啟動參數 -p tlat/tlon"
        self.talt = float(p("talt").value)
        self.approach = float(p("approach_from").value)
        self.r_fan = float(p("r_fan").value)
        self.half = float(p("half_angle").value)
        self.vmin = float(p("vmin").value); self.vmax = float(p("vmax").value)
        self.plan_frac = float(p("plan_frac").value)
        self.margin = float(p("margin").value)
        self.reach = float(p("reach").value)
        self.layer = float(p("layer_m").value)
        self.perch_alt = float(p("perch_alt").value)
        self.pushover_r = float(p("pushover_r").value)
        self.dive_alt = float(p("dive_alt").value)
        self.dive_layer = float(p("dive_layer").value)
        self.overshoot = float(p("overshoot").value)
        self.switch_r = float(p("switch_r").value)
        self.dive_speed = float(p("dive_speed").value)
        self.dive_angle = float(p("dive_angle").value)
        self.perch_gate = float(p("perch_alt_gate").value)
        self.track_frac = max(0.2, min(1.0, float(p("track_frac").value)))
        self.dive_radius = float(p("dive_radius").value)
        self.pace_cfg = float(p("dive_pace").value)
        self.dive_spdw = float(p("dive_spdweight").value)
        self.hdem_tconst = float(p("dive_hdem_tconst").value)
        self.align_tol = float(p("align_tol").value)
        self.ground_alt = float(p("ground_alt").value)
        # 下滑線的終點高度 = 紅心地表(+dive_alt 微量餘裕),而不是相對 HOME 的 0
        self.dive_alt_agl = self.dive_alt          # 離地餘裕(m)
        self.dive_alt = self.ground_alt + self.dive_alt
        # ⚠️⚠️ 接下一個目標時要**還原**的三個量。它們在飛行中會被就地改寫:
        #    dive_alt / pushover_r 由 _reground() 依實測地形重算、
        #    miss_far 由 _clamp_miss_far() 夾寬。留著上一個紅心的值去打
        #    下一個紅心 = 用錯的地表落差算環半徑,正是那個差 1821 m 的坑。
        self._dive_alt0 = self.dive_alt
        self.line_tol = float(p("line_tol").value)
        self.resend_dt = float(p("resend_min_dt").value)
        self.expend = bool(p("expend_on_impact").value)
        self.kill_on_touch = bool(p("kill_on_touchdown").value)
        self.kill_agl = float(p("kill_agl_m").value)
        self.kill_confirm_n = max(1, int(p("kill_confirm_n").value))
        self.kill_arm_agl = float(p("kill_arm_agl").value)
        self.perch_band = float(p("perch_band").value)
        self.perch_settle = float(p("perch_settle").value)
        self.ring_t0 = None    # 全員抵環的時刻(高度收斂等待的起點)
        self.dive_T = None   # 共識鎖定的全域期望命中時刻(進 DIVE 時廣播)
        self.ring_replan = False  # 首架抵環時是否已重排共同命中時刻(見放行段)
        self.impact_settle = float(p("impact_settle").value)
        self.done_t = None     # 全員命中的時刻(收尾要再多錄 impact_settle 秒)
        self.perch_reach = float(p("perch_reach").value)
        self.auto_ring = bool(p("auto_ring").value)
        if self.auto_ring:
            # 由「可達的下滑角」反推環半徑,而不是硬給半徑再期待飛機做得到。
            # ⚠️ 這裡只能用 ground_alt 這個**參數**,因為地形回報還沒進來。
            #    真值到手之後由 _reground() 重算 —— 差 1821 m 的那個坑就在這。
            r_need = (self.perch_alt - self.dive_alt) / math.tan(
                math.radians(self.dive_angle))
            self.get_logger().info(
                f"auto_ring：perch {self.perch_alt:.0f}m / 角度 {self.dive_angle:.0f}° "
                f"→ pushover_r {self.pushover_r:.0f} → {r_need:.0f}m"
                f"(暫用 ground_alt={self.ground_alt:.0f};待地形回報後重算)")
            self.pushover_r = max(self.perch_reach, r_need)
        self._ring_r0 = self.pushover_r     # 見 _dive_alt0 的說明
        self.impact_reach = float(p("impact_reach").value)
        self.impact_alt_ok = float(p("impact_alt_ok").value)
        self.miss_far = float(p("miss_far_m").value)
        self._miss_far0 = self.miss_far     # 見 _dive_alt0 的說明
        self.miss_far_k = float(p("miss_far_k").value)
        self.miss_slow_gs = float(p("miss_slow_gs").value)
        self.miss_slow_sec = float(p("miss_slow_sec").value)
        self.miss_window = float(p("miss_window_s").value)
        self.target_amsl = float(p("target_amsl").value)
        self.prox_h = float(p("prox_h").value)
        self.prox_v = float(p("prox_v").value)

        # ── 扇形 slot + pushover 環 + 穿越點(T 後方 overshoot,直線穿過 T)──
        self._ring_points()
        self.spacing = (self.r_fan * math.radians(2 * self.half)
                        / max(1, self.n - 1))   # n=1 無間距語意,避免除零
        self.pn_on = bool(p("pn_enable").value)
        self.fc_base = int(p("fc_base").value)
        self.fc_stride = int(p("fc_stride").value)
        self.tel_us = float(p("tel_us").value)
        self.pn_dt = 1.0 / max(5.0, float(p("pn_hz").value))   # ⚠️ 名義值,見 pn_hz 註解
        # ⭐ 生效速率:由**實際的 timer 週期**導出,不是由參數宣告。
        self.tick_dt = TICK_PERIOD_S
        self.tick_hz = 1.0 / TICK_PERIOD_S
        self.pn_alpha_tau = max(1e-3, float(p("pn_alpha_tau").value))
        self.tecs_pitch_min = float(p("tecs_pitch_min").value)
        self.state_max_age = float(p("state_max_age").value)
        self.att_require_fc = bool(p("att_require_fc").value)
        self.param_verify = int(p("param_verify").value)
        self.att_handback = bool(p("att_handback").value)
        self.att_eas_floor = float(p("att_eas_floor").value)
        self.att_agl_floor = float(p("att_agl_floor").value)
        self.stall_prev_glide = int(p("stall_prevention_glide").value)
        self.att_require_guided = bool(p("att_require_guided").value)
        self.hold_alt_cfg = float(p("hold_alt").value)
        self.hold_v_cfg = float(p("hold_v").value)
        self.predict_hz = float(p("predict_hz").value)
        self.predict_thr_max = float(p("predict_thr_max").value)
        self.predict_gamma_cap = float(p("predict_gamma_cap").value)
        self.tgo_from_predictor = bool(p("tgo_from_predictor").value)
        self.tgo_predict_dt = float(p("tgo_predict_dt").value)
        self.tgo_ring_transit = bool(p("tgo_ring_transit").value)
        self.itcg_on = bool(p("itcg_on").value)
        self.itcg_gain = float(p("itcg_gain").value)
        self.itcg_a_max_g = float(p("itcg_a_max_g").value)
        self.mpng_on = bool(p("mpng_on").value)
        self.mpng_k_eps = float(p("mpng_k_eps").value)
        self.mpng_k_s = float(p("mpng_k_s").value)
        self.release_sigma_spec = self._parse_sigma_spec(
            str(p("release_sigma_spec").value or ""))
        self.release_sigma_timeout = float(p("release_sigma_timeout").value)
        self.ring_pace = bool(p("ring_pace").value)
        self.release_at_ring = bool(p("release_at_ring").value)
        self.wait_goto_radius = bool(p("wait_goto_radius").value)
        self.dive_from_runup = bool(p("dive_from_runup").value)
        self.weave_time_sync = bool(p("weave_time_sync").value)
        self.weave_theta_max = float(p("weave_theta_max").value)
        self.weave_deadband_s = float(p("weave_deadband_s").value)
        self.e_t = {}          # i -> 最近一次的抵達時間誤差(s);> 0 = 太早到
        self.tgo_obs = {}      # i -> 最近一次觀測到的 t_go(s)
        self.d_hist = {}       # i -> [(mono, d)] 量測閉合速率用的短窗歷史
        self.itcg_e = {}       # i -> 最近一次的 e_t(公尺;> 0 = 太早到)
        self.itcg_a = {}       # i -> 最近一次的 ITCG 橫向偏置(m/s²)
        self.itcg_s = {}       # i -> 等效路徑餘裕(公尺;> 0 = 要繞路)
        self.itcg_side = {}    # i -> ±1,鎖存的繞路方向(避免 σ≈0 翻面抖動)
        self.mpng_k0 = {}      # i -> 鎖存的 K(1/(m·s));自動推導時用放行當拍的 R、t_go
        self.itcg_dbg = {}     # i -> 本拍儀表 dict(見 _row 尾端六欄)
        self.sigma_gate_hit = {}   # i -> 航向閘門是否已放行過(只印一次)
        self.sigma_gate_starved = {}   # i -> 是否為**逾時放行**(該架次要剔除)
        self.sigma_wait_t0 = {}        # i -> 開始等待航向視窗的時刻(monotonic)
        self.weave_theta = {}  # i -> 最近一次的蛇形偏航角(度)
        self.tgo_cache = {}     # i -> (t_stamp, t_go)  積分結果的節流快取
        self.pred = {}          # i -> 最近一次預測結果(dict)
        self.pred_t = {}        # i -> 最近一次預測的時戳
        self._pred_warned = False
        self._ring_transit_warned = False
        self._tgo_warned = False
        # S1 觀測層:狀態新鮮度、飛控模式、角速率、姿態通道被擋的理由
        self.state_t = {}      # 最後一次收到 GLOBAL_POSITION_INT 的單調時戳
        self.fc_mode = {}      # HEARTBEAT.custom_mode
        self.fc_mode_t = {}
        self.rate = {}         # ATTITUDE 的 (rollspeed, pitchspeed, yawspeed)
        self.att_block = {}    # i -> 最近一次姿態被兜底擋下的理由(只記最後一個)
        self.att_block_n = {}  # i -> 累計被擋次數(結案時印出來)
        self.handback_t = {}   # i -> 最近一次姿態交棒重錨的時戳
        self.gate_r = float(p("gate_r").value)
        self.gate_theta_max = float(p("gate_theta_max").value)
        self.gate_phi = float(p("gate_phi").value)
        self.gate_r_min = float(p("gate_r_min").value)
        self.gate_r_max = float(p("gate_r_max").value)
        self.gate_hold_n = max(1, int(p("gate_hold_n").value))
        self._gate_hits = {}      # {i: 連續成立的拍數}(見 gate_of / 交棒判定)
        self.gate_h = float(p("gate_h").value)
        self.n_lat = float(p("pn_n_lat").value)
        self.n_vert = float(p("pn_n_vert").value)
        self.pn_lead = float(p("pn_lead").value)
        self.fz_tgo = float(p("pn_freeze_tgo").value)
        self.fz_r = float(p("pn_freeze_r").value)
        self.pn_vmax = float(p("pn_vmax").value)
        self.stagger_depart = bool(p("stagger_depart").value)
        self.stagger_lead = float(p("stagger_lead").value)
        self.perch_step = float(p("perch_step").value)
        self.dive_full_thr = bool(p("dive_full_thr").value)
        self.dive_ring_gate = bool(p("dive_ring_gate").value)
        self.ring_gate_frac = float(p("ring_gate_frac").value)
        self.weave_period = max(4.0, float(p("weave_period").value))
        self.pn_los_floor = float(p("pn_los_floor").value)
        self.pn_nz_min = float(p("pn_nz_min").value)
        self.dive_pitch_max = float(p("dive_pitch_max").value)
        self.dive_thr_max = float(p("dive_thr_max").value)
        self.goto_radius = float(p("goto_radius").value)
        self.hold_radius = float(p("hold_radius").value)
        self.dive_angle_max = float(p("dive_angle_max").value)
        self.perch_climb_v = float(p("perch_climb_v").value)
        self.dive_pitch_min = float(p("dive_pitch_min").value)
        self.pn_thr_trim = float(p("pn_thr_trim").value)
        self.pn_thr_gain = float(p("pn_thr_gain").value)
        self.pn_late_full = bool(p("pn_late_full").value)
        self.gate_profile = bool(p("gate_profile").value)
        self.vel = {}         # 每機 NED 速度 (vn, ve, vd)
        self.att = {}         # 每機姿態 (roll, pitch, yaw) rad
        self.alpha = {}       # 攻角估計(低通)
        self.strike = {}      # 是否已進入終端 PN
        self.frozen = {}      # 凍結後保持的指令
        self.fc = {}          # 直連飛控的連線
        self.home_amsl = {}   # 每機 home 海拔
        # ⚠️ 每機**絕對**高度(AMSL),與 relative_alt 取自同一則訊息。
        #    釘死 AMSL 的目標用它算 agl,免疫飛控重設 HOME(見 _agl 的事故說明)。
        self.alt_amsl = {}
        self.zg = {}    # 每機自量的紅心地表高度(相對 HOME)
        self.zg0 = {}   # 第一次量到的 z_g = log 上印過的那個值(離線工具只讀得到它)
        self.zg_warn = {}  # 上次為漂移出過聲的 z_g(節制警告頻率)
        self.zg_amsl = {}  # 紅心地表的 AMSL(GPS 基準),換算真值高度用
        self.home_ll = {}    # 每機 HOME 經緯(地表差的減數)
        self.terr_bull = {}  # 紅心地表 AMSL(地形資料庫)
        self.terr_home = {}  # 本機 HOME 地表 AMSL(同一資料庫,offset 才會抵消)
        self.truth = {}       # SITL 真值位置(SIM_STATE);只量測,不進導引
        self.truth_dmin = {}  # 真值基準的最近水平距
        self.truth_hit = {}   # 真值基準的命中高度(離紅心地表)
        self.dmin = {}   # 各機對 T 的最近水平距(記錄真實穿越)
        #                  ⚠️ 在 impact/missed **鎖存當下**凍結(見 _track_dmin);
        #                     沒有收斂窗 —— 窗會收進機體入水後爬起來的第二趟。
        self.commit = {}  # 各機是否已鎖定穿越(進 switch_r 後不再翻回瞄 T)
        self.thru_dyn = {}  # 鎖定當下算出的穿越點(在「本機→T」的延長線上)
        self.hit_alt = {}  # 各機命中當下的高度(驗證是俯衝命中而非高空掠過)
        self.hit_d = {}    # 各機**鎖存瞬間**的水平距(與 hit_alt 同一時刻)
        #                    ⚠️ 這不是最近水平距:鎖存是 d < impact_reach 觸發的,
        #                       dmin 記的是鎖存前那一段跑動出來的最小值,
        #                       必然 hit_d ≥ dmin。兩個都印,不要互相取代。
        self.wp_key = {}   # 俯衝航點鎖存鍵(同 key 不重發 → 不重設高度斜坡)
        self.wp_t = {}     # 上次重發時刻(節制重發頻率)
        self.wp_pt = {}    # 上次實際送出的航點 (lat, lon, alt),判斷「有沒有動」
        self.align_ok = {}  # 是否已在遠端對正過徑向線(鎖存,避免極限環)
        self.ready = {}     # PERCH 就緒(高度+對正,鎖存;全員就緒才一起內切)
        self.released = {}  # DIVE 已放行(閉迴路判定後鎖存,避免在門檻上抖動)
        self.runup_t0 = None  # 同步內切的起始時刻(None = 還在 slot 上爬升對正)

        # ── ① MAVLink 分流連線(每機一條)──
        self.link, self.pos, self.gs = {}, {}, {}
        self.slot_of, self.slot_id, self.arr = {}, {}, {}
        self.perch_arr, self.impact = {}, {}
        self.killed = {}     # {i: 觸水終結時刻}(見 _kill_airframe)
        self._kill_lo = {}   # {i: 連續 AGL ≤ kill_agl 的筆數}
        self._kill_armed = {}       # {i: True} 曾量到 agl ≥ kill_arm_agl
        self._kill_arm_warn_t = {}  # 「不終結」告警的節流（30 s）
        # ── miss 出口的狀態(見 miss_far_m 的說明)────────────────────────
        # ⚠️ impact 與 missed **互斥**:先到先鎖。已 impact 的不得再被判 miss,
        #    已 missed 的不得再被判 impact(否則逃逸機繞回來就變成幽靈命中)。
        self.missed = {}     # {i: 判定時刻}
        self.miss_why = {}   # {i: 判定理由字串}(摘要行要印出來)
        self.miss_pt = {}    # {i: (lat, lon, alt)} 判定當下鎖存的原地盤旋點
        self.slow_t0 = {}    # {i: 低速起算時刻}(gs 回升就清掉,不累加)
        self.last_v, self.last_v_t = {}, {}
        lbase = int(p("link_base").value); lstride = int(p("link_stride").value)
        # 診斷用:告警訊息要講得出「是哪些埠」,不然使用者無從查起
        self._link_desc = (f"{lbase}+{lstride}i "
                           f"({lbase}..{lbase + lstride * (self.n - 1)})")
        for i in range(1, self.n + 1):
            c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{lbase + lstride * (i - 1)}",
                                           source_system=233, dialect="ardupilotmega")
            c.wait_heartbeat(timeout=8)
            # 明確要求資料流(同 AeroPlan 作法),避免晚連 client 收不到 GLOBAL_POSITION_INT
            try:
                c.mav.request_data_stream_send(c.target_system or i, 1,
                                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
                # 主線也要 ATTITUDE(直連還沒建立前的後備來源)
                c.mav.command_long_send(
                    c.target_system or i, 1,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    30, 100000, 0, 0, 0, 0, 0)
                # SIM_STATE(#108)= SITL 真值,用來對照遙測的命中誤差。
                # 10 Hz 就夠(只是量測),遠離 25 ms 的 SET_MESSAGE_INTERVAL 上限。
                c.mav.command_long_send(
                    c.target_system or i, 1,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    108, 100000, 0, 0, 0, 0, 0)
            except Exception:
                pass
            self.link[i] = c
            self.last_v[i] = 0.0; self.last_v_t[i] = 0.0

        self.gate_pub = self.create_publisher(String, "/swarm/tot_gate", GATE_QOS)
        self.status_pub = self.create_publisher(String, "/swarm/tot_status", 10)
        # ⭐ 目標訂閱。**兩條路徑都訂**,但啟動時就帶目標的那條會在 _on_target
        #    裡被擋掉(phase 不是 STANDBY 就拒收)—— 批次航次跑到一半被一則
        #    topic 換掉目標是絕對不能發生的事。
        self.target_sub = self.create_subscription(
            String, TARGET_TOPIC, self._on_target, TARGET_QOS)
        self.phase = "STANDBY" if self.standby_mode else "INIT"
        self.t0 = None
        self.T_arr = None
        self.dive_go = None
        self._standby_stat_t = 0.0     # STANDBY 的 status 節流(1 Hz)
        self._wait_pos_t = 0.0         # 「還在等定位」抱怨的節流(30 s)
        self._standby_pos_warn_t = 0.0  # STANDBY 版的同一個抱怨(30 s)
        # ⚠️ 初值 False:第一次湊滿時要印那行正向回報,不能一開始就當成「已就緒」
        self._standby_pos_ok = False
        self.csv_path = str(p("csv").value)
        self._csv = None
        self._cw = None
        self._open_csv()
        if not self.standby_mode:
            # ⚠️ 既有路徑一字不差:strike_seq=1 ⇒ _csv_base() 回原本的檔名,
            #    geom CSV 也照舊在啟動當下寫出來。
            self.strike_seq = 1
            self._write_geom_csv()
        self._publish_gate()
        self._terrain_probe()
        # ⚠️ 週期改成引用 TICK_PERIOD_S:先前這裡寫死 0.05,而參數 pn_hz 宣告 50 Hz,
        #    兩個數字各自獨立 ⇒ 沒有任何東西保證它們一致(實際差 2.5 倍)。
        self.create_timer(TICK_PERIOD_S, self._tick)   # 終端 PN 需要高頻
        self.create_timer(1.0, self._publish_gate)  # latched 定期補發
        # 地形快取是逐步載入的:第一次問可能還沒 loaded(spacing=0),要持續問。
        self.create_timer(3.0, self._terrain_probe)
        self.get_logger().info(
            f"tot_node up：扇形 R={self.r_fan:.0f} ±{self.half:.0f}° {self.n} slot "
            f"間隔 {self.spacing:.0f}m；pushover R={self.pushover_r:.0f} perch={self.perch_alt:.0f}m "
            f"→ 俯衝 → 目標 "
            f"{'(待命中,尚未指定)' if self.standby_mode else self.T}")
        if self.standby_mode:
            # ⚠️⚠️ 「待命」必須大聲。安靜地不動,和「節點根本沒起來」在現場長得
            #    一模一樣 —— 這個專案最大的敵人正是這種靜默。含「生效值」三個字
            #    是刻意的:make_artifacts.sh 的「## 生效參數」靠它撈進 summary.md。
            self.get_logger().warning(
                f"★★ tot_node {STANDBY_SENTINEL} 生效值:tlat/tlon 未給或為 .nan "
                f"⇒ **一個指令都不下**。指定目標請發 {TARGET_TOPIC}"
                f"(std_msgs/String;內容可以是 JSON {{\"lat\":…,\"lon\":…}} "
                f"或最省事的 \"lat,lon\");"
                f"auto_approach={self.auto_approach}"
                f"(由蜂群質心反推扇形方位;訊息帶 approach_from 則以訊息為準);"
                f"target_amsl={self.target_amsl}"
                f"(海上目標請在訊息裡帶 amsl=0.0);"
                f"expend_on_impact={self.expend}"
                f"{'(命中即消耗 ⇒ 第二個目標需要新的機體)' if self.expend else ''}")
        # ⚠️⚠️ 把**生效值**印出來,不要只印幾何。
        #    實測踩過:HS1 吃到 jet 那套幾何(R_FAN/PERCH/DIVE_ANGLE),但
        #    reach/vmin/vmax/margin/layer_m 全部退回 stot_trial.sh 預設 ——
        #    混血組態,零警告,事後只能靠算術反推(43933/(0.9×100)+40 ≡ T_arr 528)。
        #    更關鍵的是 **reach**:它是 ALIGN 唯一的推進條件
        #    (`if i not in self.arr and d < self.reach`),而 ALIGN 的 _reposition
        #    送 param3=0 ⇒ ArduPlane 退回 WP_LOITER_RAD。兩者不匹配時飛機會在
        #    固定半徑上繞圈,arr 永遠湊不滿 6,**沒有 timeout、沒有錯誤訊息**。
        #    實測 G1/HS1/NEW1 三趟卡死:繞圈平台 2031/2035/2028 m vs reach 260。
        self.get_logger().warning(
            f"tot_node 生效值:reach={self.reach:.0f} perch_reach={self.perch_reach:.0f} "
            f"impact_reach={self.impact_reach:.0f} vmin={self.vmin:.0f} vmax={self.vmax:.0f} "
            f"margin={self.margin:.0f} plan_frac={self.plan_frac:.2f} "
            f"layer_m={self.layer:.0f} dive_radius={self.dive_radius:.0f} "
            f"goto_radius={self.goto_radius:.0f} hold_radius={self.hold_radius:.0f} "
            f"n={self.n}")
        # ⚠️ 這行警告先前是對的(param3 恆為 0),現在半徑已分離成
        #    goto_radius / hold_radius,改印實際生效的等效繞圈半徑供對照。
        self.get_logger().warning(
            f"⚠️ 抵達判定 d < reach={self.reach:.0f};飛過去用 param3="
            f"{self.goto_radius:.0f}(L1 再 ×eas2tas²),守環用 "
            f"{self.hold_radius:.0f}。若前者 ×eas2tas² > reach,ALIGN 會卡死。")
        self._emit_contract_line()
        self._check_hold_radius()

    # ── 生效值契約行 ──────────────────────────────────────────────────
    def _contract_values(self):
        """本節點**實際生效**的契約量。鍵名必須與 contract_table 一致。

        ⚠️ 一律取 `self.<屬性>`(建構子從參數伺服器讀回並轉型後的值),
           **不可以**回頭讀 `p("x").value` —— 那樣只是把宣告值再抄一遍,
           而「宣告了但沒生效」正是這一整類 bug 的形狀。
        ⚠️ dive_pace 給的是**組態值** `pace_cfg`(0 = 交給節點自動取
           0.8×dive_speed)。契約表知道 0 的語意並據此算實效值;這裡不先
           展開是為了讓快照能分辨「設成 0」與「算出來剛好是這個數」。
        """
        return [
            # 契約表直接管的
            ("vmin", self.vmin), ("vmax", self.vmax),
            ("dive_speed", self.dive_speed), ("dive_pace", self.pace_cfg),
            ("gate_r", self.gate_r), ("perch_alt", self.perch_alt),
            ("r_fan", self.r_fan), ("dive_angle", self.dive_angle),
            ("reach", self.reach), ("perch_reach", self.perch_reach),
            ("impact_reach", self.impact_reach), ("half_angle", self.half),
            # ⭐ 陡俯角的兩個開關 —— 兩條路徑不一致,必須讀得到才擋得住
            ("pn_los_floor", self.pn_los_floor), ("pn_nz_min", self.pn_nz_min),
            # 不變式需要的輔助量(本身不在契約表裡)
            ("hold_radius", self.hold_radius), ("goto_radius", self.goto_radius),
            ("gate_r_min", self.gate_r_min), ("gate_r_max", self.gate_r_max),
            ("n", float(self.n)),
            # ── ⭐ 2026-08-14 追加(S1 觀測層)────────────────────────────
            # ⚠️⚠️ gate_theta_max 先前**不在這裡** ⇒ 契約層看不見「動態交棒到底
            #    有沒有開」。它 <= 0 時 gate_of() 的整條動態分支是死碼、直接回
            #    gate_r,而 log 照樣印「(絕對 gate_r)」—— 兩種情況長得一樣。
            ("gate_theta_max", self.gate_theta_max), ("gate_phi", self.gate_phi),
            ("gate_hold_n", float(self.gate_hold_n)),
            # ⚠️ 生效速率:由 TICK_PERIOD_S 導出,不是由 pn_hz 宣告。
            #    兩者先前差 2.5 倍而沒有任何東西在比對(見 pn_hz 的註解)。
            ("tick_hz", self.tick_hz), ("pn_alpha_tau", self.pn_alpha_tau),
            ("pn_hz_nominal", 1.0 / max(1e-9, self.pn_dt)),
            # 俯衝段實際推送給飛控的權限值 —— 這些決定 PN 能不能交出陡俯角
            ("dive_pitch_min", self.dive_pitch_min),
            ("dive_pitch_max", self.dive_pitch_max),
            ("pn_lead", self.pn_lead), ("pn_vmax", self.pn_vmax),
            ("line_tol", self.line_tol), ("perch_step", self.perch_step),
            ("tecs_pitch_min", self.tecs_pitch_min),
            # ⭐ S4/S5/S6 安全兜底的開關。**全部要印** —— 兜底有沒有開,
            #    是事後判讀「那一趟到底受不受保護」的唯一依據。
            ("state_max_age", self.state_max_age),
            ("att_require_fc", float(self.att_require_fc)),
            ("att_require_guided", float(self.att_require_guided)),
            ("att_eas_floor", self.att_eas_floor),
            ("att_agl_floor", self.att_agl_floor),
            ("att_handback", float(self.att_handback)),
            ("param_verify", float(self.param_verify)),
            ("stall_prevention_glide", float(self.stall_prev_glide)),
            # ⭐ S8:等待段的高度與速度。兩者是「交接 γ」的直接上游。
            ("hold_alt", self.hold_alt_cfg), ("hold_v", self.hold_v_cfg),
            ("predict_hz", self.predict_hz),
            # ⭐ 放行的幾何守門員。⚠️ 0.0 = 關閉,而關閉時放行只剩
            #    perch_settle 逾時、**完全不看幾何** —— RPY3 的 78.5 s 就是
            #    這樣來的(UAV1 在所需俯角 41.5° 時被放行,實測天花板 37–40°)。
            ("dive_angle_max", self.dive_angle_max),
            ("predict_thr_max", self.predict_thr_max),
            ("predict_gamma_cap", self.predict_gamma_cap),
            # ⭐ T1:t_go 估計器的來源。這是散布的直接上游 ——
            #    靜態幾何式 SF=1.24、偏置 +11.6 s,數學上達不到 ≤1.8 s 的要求。
            ("tgo_from_predictor", float(self.tgo_from_predictor)),
            ("tgo_predict_dt", self.tgo_predict_dt),
            ("tgo_ring_transit", float(self.tgo_ring_transit)),
            # ⭐ ITCG 三件套。⚠️ itcg_on=1 而 gain=0 等於開了但沒效果 ——
            #    契約層有不變式擋這種「看起來啟用了」的組合。
            ("itcg_on", float(self.itcg_on)), ("itcg_gain", self.itcg_gain),
            ("itcg_a_max_g", self.itcg_a_max_g),
            # ⭐ B1:MPNG 開關與兩個增益。⚠️ 必須進契約快照 ——
            #    「改導引行為卻不在契約表內」正是 RPY3 那一批報廢的形狀。
            ("mpng_on", float(self.mpng_on)),
            ("mpng_k_eps", self.mpng_k_eps), ("mpng_k_s", self.mpng_k_s),
            # ⭐ V1:1 = 用速度調節取代盤旋等待
            ("ring_pace", float(self.ring_pace)),
            ("release_at_ring", float(self.release_at_ring)),
            ("wait_goto_radius", float(self.wait_goto_radius)),
            ("dive_from_runup", float(self.dive_from_runup)),
            ("perch_band", float(self.perch_band)),
            ("perch_settle", float(self.perch_settle)),
            # ⭐ T3:閉迴路開關與它的兩個可行性參數。
            #    ⚠️ weave_period 必須夠大,否則換向率夾限會把 θ 壓到吸不到時間
            #       —— 「開了但沒效果」比不開更糟,契約層有不變式擋。
            ("weave_time_sync", float(self.weave_time_sync)),
            ("weave_theta_max", self.weave_theta_max),
            ("weave_period", self.weave_period),
        ]

    def _emit_contract_line(self):
        body = " ".join(f"{k}={v:g}" for k, v in self._contract_values())
        self.get_logger().warning(f"tot_node {CONTRACT_SENTINEL}:{body}")

    # ⚠️⚠️ 2026-08-05 新增。這個檢查以前不存在,而它要擋的約束**就寫在同一份
    #    檔案裡**(hold_radius 的參數說明:「守在該點——要 ≥ 物理迴轉半徑,
    #    否則飛機飛不出那個圈」)。沒有任何一行程式碼去驗它,於是預設值
    #    hold_radius=400 一路違反了它自己的契約,而且完全靜默。
    #
    #    後果(SW1 實測,環半徑 3120 m):守環退化成「繞著環點打自己的最小圈」,
    #    距目標距離在 **1724–4925 m** 之間振盪(±1400 m)。六機**進環時** d 很
    #    一致(3108–3131 m),**放行時**卻散到 1806–3449 m —— 放行 h/d 因此從
    #    0.78 散到 1.68、需求俯角 38.1° → 59.3°。
    #    P91 打得中不是因為它守得比較好(振幅更大:3170–8035),而是它的環半徑
    #    6140 m 是 SW1 的兩倍,同樣的振幅換算成 h/d 只有 0.42–0.63。
    #    ⇒ 「環半徑要大於迴轉半徑好幾倍」是這個架構的隱含前提,先前沒人寫下來。
    def _check_hold_radius(self):
        """守環半徑 vs 物理迴轉半徑。不過就大聲吼(不中止,留給操作者決定)。"""
        # ISA 密度比 → EAS 指令換算成實際 TAS;迴轉半徑 ∝ TAS²
        sigma = (1.0 - 2.25577e-5 * max(self.perch_alt, 0.0)) ** 4.2559
        tas = self.vmin / math.sqrt(max(sigma, 1e-6))
        r_turn = tas * tas / (9.81 * math.tan(math.radians(45.0)))
        if self.hold_radius >= r_turn:
            self.get_logger().warning(
                f"✔ 守環半徑檢查:hold_radius={self.hold_radius:.0f} m ≥ "
                f"物理迴轉半徑 {r_turn:.0f} m(vmin={self.vmin:.0f} EAS ⇒ "
                f"TAS {tas:.0f} @ {self.perch_alt:.0f} m、45° 傾角)")
            return
        self.get_logger().error(
            f"⚠️⚠️ hold_radius={self.hold_radius:.0f} m **小於**物理迴轉半徑 "
            f"{r_turn:.0f} m(vmin={self.vmin:.0f} EAS ⇒ TAS {tas:.0f} @ "
            f"{self.perch_alt:.0f} m、45° 傾角)—— 飛機飛不出這個圈,守環會退化成"
            f"繞自己的最小圈,**放行時距目標的距離會在環半徑 ±{r_turn:.0f} m 之間"
            f"隨機**,放行 h/d 與需求俯角因此逐機不同。SW1 就是這樣:環 3120 m、"
            f"放行 d 散到 1806–3449 m、需求俯角 38.1°→59.3°。"
            f"處置:①把 hold_radius 提到 ≥{r_turn:.0f};或②把環半徑放大到"
            f"迴轉半徑的數倍(P91 用 6140 m 就打中了);或③在放行端加相位閘門"
            f"(等飛機回到自己徑向線且 d≈環半徑才放行)。")
        # 環半徑相對迴轉半徑的餘裕 —— 這是「振幅佔環半徑多少」的直接指標。
        # ⚠️ 對照組用的是**實測振幅**,不是這裡算出來的 r_turn:指令速度對機體
        #    未必有權限(P91 不管指令多少都飛 164–171 m/s),拿 vmin 去推迴轉
        #    半徑會低估。實測(hold_ring/wait 段的 d 全距):
        #      SW1 環 3120 m、振幅 ±1400 m ⇒ 2.2×(放行 d 散到 1806–3449,打不中)
        #      P91 環 6140 m、振幅 ±1900 m ⇒ 3.2×(6/6 觸海、誤差 3.0 m)
        #    n=2,還不足以定出門檻值,但方向明確:比值越小,放行幾何越隨機。
        ring = self.pushover_r if not self.perch_step else self.ring_of(
            min(self.slot_id, key=lambda k: self.slot_id[k]) if self.slot_id else 1)
        if ring > 0:
            self.get_logger().error(
                f"   環半徑 {ring:.0f} m / 迴轉半徑 {r_turn:.0f} m = "
                f"{ring / r_turn:.2f}×(以 vmin 推估,實測會更小)。"
                f"實測對照:SW1 2.2× 打不中、P91 3.2× 六發全中(n=2,非門檻)。")

    # ══════════════════════════════════════════════════════════════════════
    #  待命模式:CSV 換檔 / 目標解析 / 狀態重設 / 進出 STANDBY
    # ══════════════════════════════════════════════════════════════════════

    def _csv_base(self):
        """本發的 CSV 檔名前綴。

        ⚠️ 第 1 發**一定要**沿用原本的 `csv` 參數值:整條離線工具鏈
           (stot_score / _kpi_parse / stot_mission_anim / _tot_plot /
           make_artifacts.sh)都是拿 `tot_<TRIAL>.csv` 這個名字去找檔案的。
           第 2 發起才加 `_s2`、`_s3`…,免得第二發把第一發的證據覆蓋掉
           —— 「覆蓋上一趟的證據」在本專案是明文禁止的(見 CMD_LOG_DIR)。
        """
        return (self.csv_path if self.strike_seq <= 1
                else f"{self.csv_path}_s{self.strike_seq}")

    def _open_csv(self):
        """開(或換)本發的軌跡 CSV。表頭與既有版本逐欄相同。"""
        if self._csv is not None:
            try:
                self._csv.close()
            except Exception:
                pass
        self._csv = open(self._csv_base() + ".csv", "w", newline="")
        self._cw = csv.writer(self._csv)
        # ⚠️⚠️ `agl` **只能追加在最後一欄**。_tot_plot.py / _dive_plot.py 是用
        #    位置解析(r[0]…r[11]),插在中間會把它們整個弄錯而且完全靜默。
        #    其餘消費者(stot_score / stot_mission_anim / impact_feasibility /
        #    spread_growth)都走 csv.DictReader,按欄名解析,追加欄不影響。
        # ⭐ S1/S9 追加欄(2026-08-14)。⚠️ 一律**追加在最後**,理由同上。
        self._cw.writerow(["t", "sysid", "lat", "lon", "alt", "gs", "d_slot",
                           "mode", "v_cmd", "slot_lat", "slot_lon", "phase",
                           "agl",
                           # ── ⭐ 2026-08-14 追加(位置解析的消費者不受影響)──
                           # gam_deg              當下航跡角(度)。
                           #   ⚠️⚠️ **下降為負**(atan2(-vz, |vh|),vz 是 NED
                           #   的下正)。這行註解 2026-08-15 之前寫成「下降為正」
                           #   —— 完全寫反。方向寫錯的註解比沒有註解更糟:
                           #   事後判讀會把「最陡」讀成 max() 而實際上要 min(),
                           #   結論剛好顛倒。(我自己在 RQA 的分析就中了一次。)
                           # pred_residual_alt_m  預測的殘餘高度(水平掠過紅心那一刻)
                           #                      —— 與 stot_score 的「最低AGL」同一個量,
                           #                      而誤差 = 1.167 × 它
                           # pred_touch_gs        預測觸地**地速**(m/s)
                           # pred_secs_to_target  預測剩餘飛行時間(s)
                           # pred_over_limit_frac 預測剖面**要求超過飛機能力**的時間比例。
                           #                      > 0 ⇒ 這條剖面要飛機做做不到的事,
                           #                      上面幾個 pred_* 都不可信。
                           "gam_deg", "pred_residual_alt_m", "pred_touch_gs",
                           "pred_secs_to_target", "pred_over_limit_frac",
                           # ── ⭐ T2 時間協同的觀測量(2026-08-14)──────────
                           # ⚠️⚠️ 欄名一律**帶方向與單位**。本專案反覆吃過
                           #    「描述對但沒說方向」的虧(memory/confirm-direction-
                           #    not-just-topic):`e_t` 這種符號名看不出正負代表什麼,
                           #    而 early_by_s 的正負搞反會讓閉迴路往**錯的方向**蛇形,
                           #    飛出來只長得像「效果不好」。
                           #
                           # secs_to_target  到紅心還要多久(s)。用靜態幾何式還是
                           #                 積分式,由 tgo_from_predictor 決定
                           #                 —— 契約行有記,事後查得到。
                           # early_by_s      **早到幾秒**(正 = 太早、負 = 太晚)
                           #                 = (共同命中時刻 − now) − secs_to_target
                           #                 這是散布的直接上游:
                           #                 散布 = 0.76 × 交接時間差 + 1.62(R²=0.779)
                           # weave_yaw_deg   蛇形的**偏航角**指令(度),不是擺幅。
                           #                 0 = 沒在蛇形。
                           "secs_to_target", "early_by_s", "weave_yaw_deg",
                           # ⭐⭐ ITCG(2026-08-15)。⚠️ 只能**追加**在最後:
                           #    _tot_plot.py / _dive_plot.py 是照位置解析 r[0]…,
                           #    插在中間會靜靜地把每一欄都平移一格。
                           # early_by_m   **早到幾公尺**(正 = 太早)
                           #              = V·(共同命中時刻 − now) − 斜距
                           #              ⚠️ 與 early_by_s 是**不同的量**:
                           #              這個不需要估 t_go,那個需要。
                           #              兩欄並存是刻意的 —— 可以互相對照。
                           # itcg_accel   ITCG 疊加在橫向通道的偏置(m/s²)。
                           #              正負 = 繞路方向(鎖存,不會逐拍翻面)。
                           #              0 = 沒出力(關閉/已遲到/末端凍結)
                           # ⚠️ early_by_m 是**等速假設**的舊形式,RQC 證明它在
                           #    我們的加速俯衝上恆為負 ⇒ 保留只為對照,**不是**
                           #    導引律在用的量。律用的是下面這個。
                           # itcg_surplus_m 等效路徑餘裕(公尺;正 = 要繞遠路)
                           #    = 斜距 × (早到秒數 / t_go)
                           # ⚗️ lead_angle_deg 前置角(度):0 = 機頭正對紅心、
                           #    180 = 正背對。**放行瞬間的這個值就是本次實驗的
                           #    自變數** —— 先前完全沒有這一欄,那 4 架乾淨機體
                           #    的航向是我從軌跡反推出來的。
                           "early_by_m", "itcg_accel", "itcg_surplus_m",
                           "lead_angle_deg",
                           # ⭐⭐ S2 儀表(2026-08-18)。**只能尾端追加**:
                           #    _tot_plot.py / _dive_plot.py 用 r[0..8] 位置解析,
                           #    其餘判讀腳本走 DictReader ⇒ 尾端追加安全。
                           # ⚠️⚠️ req_radius_m = 本拍**要求**的半徑;
                           #    cmd_radius_m = 最後真正**送出去**的半徑。
                           #    兩者必須分開記,因為 `_goto` 的變更偵測器看不到
                           #    radius ⇒ 純半徑變更會被吃掉、指令從未送出。
                           #    「送出去的」在開關開與關時同樣是 30 ⇒ 拿它當
                           #    生效證明會給**假綠燈**。決定性的量是 req_radius_m。
                           # ⚠️ wp_lat/wp_lon = 本拍瞄的航點。沒有它就得靠
                           #    「最近的 pp」反推各機環點,而那個綁定會綁錯:
                           #    RRD1/U2 綁錯之後航跡比從 2.02 變 0.90,一架硬彎機
                           #    會被判成直進 —— 而它正是受測對象。
                           "req_radius_m", "cmd_radius_m", "wp_lat", "wp_lon",
                           # ⭐⭐ B1 MPNG 儀表(2026-08-19)。**只能尾端追加**。
                           # ⚠️⚠️ 這條通道 80,760 個樣本**全零** —— 從來沒有
                           #    承載過訊號。「開關開著但那一段從來沒真的發生」
                           #    是本專案的頭號失效類別(已第五次)⇒ 上線第一件
                           #    事是讓 itcg_req_a 非零、itcg_sat 有分佈,
                           #    **再談效果**。觀測性先於功能。
                           # itcg_req_a    本拍**要求**的橫向偏置(m/s²),
                           #               夾制**前**、凍結**前**、遲到早退**前**。
                           #               ⇐ 判定量。每拍都寫。
                           # itcg_cmd_a    實際疊進 a_h 的值(夾制**後**)。
                           #               ⚠️ 佐證量,不是判定量:它在律沒出力
                           #               時是 0,而 0 也是合法輸出 ⇒ 拿它判
                           #               「機制生效」會給假綠燈(S2 的
                           #               cmd_radius_m 就是這個坑)。
                           # itcg_eps_t    ε_T = t̄go − tgo(秒)。> 0 = 會太早到。
                           # itcg_tgo_ref  本拍實際使用的 t_go(我們現行的估計器)
                           # itcg_tgo_jiang Jiang Eq.(29) 的 t_go。**只記錄不使用** ——
                           #               它的優勢只在 σ > 40°,而我們 σ 中位
                           #               15.4°;實測餵 32 架次 |誤差| 中位從
                           #               3.73 s **變差到** 4.28 s。並列讓資料說話。
                           # itcg_sat      夾制旗標(0/1)。整段都是 1 ⇒ 律退化成
                           #               bang-bang ⇒ 調增益無用,只能動夾制。
                           "itcg_req_a", "itcg_cmd_a", "itcg_eps_t",
                           "itcg_tgo_ref", "itcg_tgo_jiang", "itcg_sat"])

    def _write_geom_csv(self):
        """幾何 CSV(紅心 + slot + 環點)。內容與既有版本逐列相同。"""
        gf = open(self._csv_base() + "_geom.csv", "w", newline="")
        gw = csv.writer(gf); gw.writerow(["kind", "lat", "lon"])
        gw.writerow(["target", self.T[0], self.T[1]])
        for k, s in enumerate(self.slots):
            gw.writerow([f"slot{k}", s[0], s[1]])
        for k, s in enumerate(self.pp):
            gw.writerow([f"pp{k}", s[0], s[1]])
        gf.close()

    @staticmethod
    def is_standby_target(lat, lon):
        """「目標到底有沒有給」的**唯一判準**:任一座標是 NaN 就算沒給。

        ⚠️ 不要在這裡塞「等於某個預設值就當沒給」之類的啟發式 —— 那會讓
           「使用者真的想打桃園那一點」被當成待命,而且完全無聲。
        ⚠️ 轉不成浮點數也算沒給:待命是**安全側**,寧可不動也不要拿一個
           看不懂的值去打。
        """
        try:
            return math.isnan(float(lat)) or math.isnan(float(lon))
        except (TypeError, ValueError):
            return True

    @staticmethod
    def validate_target(lat, lon):
        """座標合理性。回傳 (ok, reason)。

        ⚠️ 刻意寫成**純函式**(不碰節點狀態):沒有 ROS runtime 的環境也能測,
           而且「檢查」不會有副作用(見 _metrics_frozen 的同一條規矩)。
        """
        try:
            lat = float(lat); lon = float(lon)
        except (TypeError, ValueError):
            return False, "lat/lon 不是數字"
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return False, f"lat/lon 不是有限值(lat={lat!r}, lon={lon!r})"
        if not -90.0 <= lat <= 90.0:
            return False, f"緯度 {lat!r} 超出 ±90"
        if not -180.0 <= lon <= 180.0:
            return False, f"經度 {lon!r} 超出 ±180"
        # ⚠️ (0, 0) 是幾內亞灣外海的 Null Island。它是**合法**的經緯度,所以
        #    任何範圍檢查都攔不住它 —— 而它幾乎一定是「欄位沒填/解析失敗回 0」
        #    的產物。讓六架飛機朝一個預設值衝下去是最壞的失敗形狀,單獨擋掉。
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return False, "座標是 (0, 0)(Null Island)—— 幾乎一定是未填值"
        return True, ""

    @staticmethod
    def parse_target(data):
        """把 /swarm/tot_target 的 payload 解析成 (lat, lon, extra)。

        接受兩種寫法,**兩種都是刻意的**:
          1. JSON 物件 `{"lat": 25.14, "lon": 121.23, "approach_from": 0.0,
             "amsl": 0.0, "src": "aeroplan-rclick"}` —— 正式格式,可帶選填欄位。
          2. 純 `"lat,lon"` —— 從 Windows 端呼叫時 JSON 要穿過
             cmd/PowerShell → wsl.exe → bash → ros2 topic pub 四層引號,
             逐層跳脫是這條路徑最容易靜默壞掉的地方。CSV 形式沒有引號問題。
        解析失敗一律 raise ValueError(呼叫端負責印出**原始字串**)。
        """
        if data is None:
            raise ValueError("空訊息")
        s = str(data).strip()
        if not s:
            raise ValueError("空訊息")
        extra = {}
        if s[0] in "{[":
            try:
                obj = json.loads(s)
            except Exception as e:
                raise ValueError(f"JSON 解析失敗:{e}")
            if isinstance(obj, dict):
                if "lat" not in obj or "lon" not in obj:
                    raise ValueError("JSON 缺 lat 或 lon 欄位")
                lat, lon = obj["lat"], obj["lon"]
                extra = {k: v for k, v in obj.items() if k not in ("lat", "lon")}
            elif isinstance(obj, (list, tuple)) and len(obj) >= 2:
                lat, lon = obj[0], obj[1]
            else:
                raise ValueError("JSON 既不是 {lat,lon} 物件也不是 [lat,lon] 陣列")
        else:
            parts = [x for x in s.replace(";", ",").split(",") if x.strip()]
            if len(parts) < 2:
                raise ValueError("純文字格式要寫 \"lat,lon\"")
            lat, lon = parts[0], parts[1]
        try:
            lat = float(lat); lon = float(lon)
        except (TypeError, ValueError):
            raise ValueError(f"lat/lon 轉不成浮點數:{lat!r}, {lon!r}")
        return lat, lon, extra

    def _swarm_centroid(self):
        """蜂群質心(lat, lon);沒有任何定位就回 None。"""
        if not self.pos:
            return None
        lats = [v[0] for v in self.pos.values()]
        lons = [v[1] for v in self.pos.values()]
        return (sum(lats) / len(lats), sum(lons) / len(lons))

    def _reset_for_new_target(self, lat, lon):
        """把節點恢復成「剛啟動、還沒飛」的狀態,再把目標換成新的。

        ⚠️⚠️ 清單本身是規格,寫在模組頂端的 RESET_DICTS / RESET_NONE。
           這裡只負責照著清,不要在這裡加零散的 `self.xxx = {}` ——
           那樣測試就鎖不住(測試比對的是那兩個常數)。
        """
        for name in RESET_DICTS:
            getattr(self, name).clear()
        for name in RESET_NONE:
            setattr(self, name, None)
        self.ring_replan = False
        # ⚠️ last_v / last_v_t 是**直接索引**的(`self.last_v[i]`,不是 .get),
        #    清成 {} 會在下一次 _goto 直接 KeyError。要就地歸零,不是清空 ——
        #    歸零之後 `abs(v - last_v) > 0.5` 必定成立 ⇒ 新的 DO_CHANGE_SPEED
        #    會真的送出去,不會被上一發的速度鎖存吃掉。
        for i in list(self.last_v):
            self.last_v[i] = 0.0
            self.last_v_t[i] = 0.0
        # 飛行中被就地改寫的三個量,全部還原成啟動時算出來的值(見 _dive_alt0)
        self.miss_far = self._miss_far0
        self.pushover_r = self._ring_r0
        self.dive_alt = self._dive_alt0
        self.dive_pace = self.pace_cfg
        # 一次性 log 的旗標:第二發要重新印一次,否則「生效值」只有第一發有
        self._miss_cfg_logged = False
        self._agl_datum_logged = False
        self._wait_pos_t = 0.0
        for a in ("_prox_min", "_prox_t"):
            self.__dict__.pop(a, None)
        self.T = (float(lat), float(lon))
        self._ring_points()          # slot / 環點 / 穿越點全部依新紅心重算

    def _arm_target(self, lat, lon, extra=None, src="topic"):
        """接受一個新目標:重設 → 換 CSV → 進 INIT。回傳 True 表示已接受。

        ⚠️ 呼叫端必須先過 validate_target 與 phase 檢查。這裡只做「接受」。
        """
        extra = extra or {}
        self.strike_seq += 1
        self._reset_for_new_target(lat, lon)
        # ── 扇形方位:訊息 > 自動反推 > 啟動參數 ────────────────────────
        appr_src = f"啟動參數 approach_from={self.approach:.1f}°"
        if "approach_from" in extra:
            try:
                self.approach = float(extra["approach_from"]) % 360.0
                appr_src = f"訊息指定 {self.approach:.1f}°"
            except (TypeError, ValueError):
                self.get_logger().error(
                    f"⚠️ 訊息的 approach_from={extra['approach_from']!r} 不是數字,"
                    f"改用 {self.approach:.1f}°")
        elif self.auto_approach:
            cen = self._swarm_centroid()
            if cen is None:
                self.get_logger().error(
                    f"⚠️⚠️ auto_approach 開著,但**一台飛機的定位都沒有** ⇒ "
                    f"沿用 approach_from={self.approach:.1f}°。"
                    f"扇形可能擺在紅心的錯誤側(蜂群要穿過紅心才到得了 slot)。"
                    f"處置:確認 SITL 分流埠 {self.link and 'OK' or '未連上'}、"
                    f"等 /swarm/tot_status 的 n_pos 到 {self.n} 再送目標。")
            else:
                self.approach = bearing(self.T, cen)
                appr_src = (f"自動反推:由紅心看蜂群質心 "
                            f"({cen[0]:.7f}, {cen[1]:.7f}) = {self.approach:.1f}°")
        # ── 目標地表 AMSL:訊息可覆寫(海上目標一定要 0.0)──────────────
        if "amsl" in extra:
            try:
                self.target_amsl = float(extra["amsl"])
            except (TypeError, ValueError):
                self.get_logger().error(
                    f"⚠️ 訊息的 amsl={extra['amsl']!r} 不是數字,沿用 "
                    f"target_amsl={self.target_amsl}")
        self._ring_points()          # approach 可能剛被改掉,再算一次
        self._write_geom_csv()
        if self.strike_seq > 1:
            self._open_csv()         # 第 2 發起換檔,不覆蓋第 1 發的證據
        self.target_src = src
        self.phase = "INIT"
        # ⚠️⚠️ **把實際收到的座標逐字印出來**,而且用 %.7f。
        #    本專案吃過太多次「參數送出去了但沒生效」的虧(JET_CDP 空字串跑了
        #    幾十趟、catch_speed 157 被整發拒收)。使用者要能拿這一行去跟自己
        #    在地圖上點的那一點對帳 —— 對不上就是傳輸層壞了,不是導引壞了。
        self.get_logger().warning(
            f"★★ 收到目標(生效值):lat={self.T[0]:.7f} lon={self.T[1]:.7f}"
            f"(來源 {src};第 {self.strike_seq} 發);"
            f"扇形方位 approach_from={self.approach:.1f}°({appr_src});"
            f"target_amsl={self.target_amsl};"
            f"r_fan={self.r_fan:.0f} half_angle={self.half:.0f}° "
            f"perch_alt={self.perch_alt:.0f} dive_angle={self.dive_angle:.0f}° "
            f"環半徑={self.pushover_r:.0f}(尚未依地形重算);"
            f"CSV={self._csv_base()}.csv;"
            f"目前有定位的機體 {sorted(self.pos)} / 需要 {self.n} 台")
        cen = self._swarm_centroid()
        if cen is not None:
            self.get_logger().warning(
                f"   對帳用:蜂群質心距紅心 {dist_m(self.T, cen) / 1000.0:.2f} km;"
                f"各機距紅心 "
                f"{ {i: round(dist_m(self.T, self.pos[i][:2])) for i in sorted(self.pos)} } m")
        if len(self.pos) < self.n:
            self.get_logger().error(
                f"⚠️⚠️ 只有 {len(self.pos)}/{self.n} 台有定位 —— 節點會停在 INIT "
                f"**等**,不會有任何動作也不會逾時。缺 "
                f"{[i for i in range(1, self.n + 1) if i not in self.pos]}。"
                f"處置:先確認 SITL 六架都在(pgrep -x arduplane)、分流埠通,"
                f"再看 /swarm/tot_status 的 n_pos。")
        self._publish_gate()
        self._terrain_probe()
        return True

    def _on_target(self, msg):
        """/swarm/tot_target 回呼。**不接受就要大聲說為什麼**。"""
        raw = getattr(msg, "data", msg)
        try:
            lat, lon, extra = self.parse_target(raw)
        except ValueError as e:
            self.get_logger().error(
                f"✗ 目標訊息解析失敗:{e}。原始字串 = {str(raw)!r}。"
                f"格式:JSON {{\"lat\": 25.1406, \"lon\": 121.2325}} 或 "
                f"\"25.1406,121.2325\"")
            return
        ok, why = self.validate_target(lat, lon)
        if not ok:
            self.get_logger().error(
                f"✗ 目標座標不合理,已丟棄:{why}。原始字串 = {str(raw)!r}")
            return
        # ⚠️⚠️ **只有 STANDBY 能接目標。** 兩件事靠這一條擋住:
        #    (1) 批次航次(啟動參數帶目標)跑到一半被 topic 換掉目標;
        #    (2) 常駐服務打到一半(ALIGN/PERCH/DIVE)被換目標 —— 那會把
        #        released/commit/dmin 全部清掉,而飛機還在朝舊紅心俯衝,
        #        指令與狀態瞬間脫節。要改目標請先讓這一發結案。
        if self.phase != "STANDBY":
            self.get_logger().error(
                f"✗ 目前 phase={self.phase},**不是 STANDBY,拒收新目標** "
                f"(收到 lat={float(lat):.7f} lon={float(lon):.7f})。"
                + ("本節點是用啟動參數 -p tlat/tlon 帶目標起來的(批次航次),"
                   "整個生命週期只打這一個目標。"
                   if not self.standby_mode else
                   f"這一發還沒結案(打完會自己回 STANDBY)。"
                   f"要中止請重啟本節點。"))
            return
        self._arm_target(lat, lon, extra, src=str(extra.get("src", "topic")))

    def _to_standby(self, why):
        """打完一發 → 回待命。⚠️ 只有常駐服務走這條;批次航次停在 DONE。"""
        self.phase = "STANDBY"
        self.T = (float("nan"), float("nan"))
        try:
            self._csv.flush()
        except Exception:
            pass
        self._publish_gate()
        self.get_logger().warning(
            f"★★ 回到待命(STANDBY):{why}。已完成 {self.strike_seq} 發。"
            f"**此後不再對任何機體下令** —— 已判 MISS 的機體會維持判定當下鎖存的"
            f"盤旋指令,已命中的機體 expend_on_impact={self.expend}。"
            f"下一個目標請發 {TARGET_TOPIC}"
            f"(下一發的 CSV 會寫到 {self.csv_path}_s{self.strike_seq + 1}.csv,"
            f"不會覆蓋這一發)。")

    def _standby_tick(self):
        """待命期間的 tick:只抽鏈路(已在 _tick 做過)+ 1 Hz 心跳狀態。

        ⚠️⚠️ **鏈路一定要繼續抽。** pymavlink 的連線不抽就會在 MAVProxy 端
           累積背壓(本專案量過 ~1 MB),等真的收到目標時,讀到的是幾分鐘前的
           位置 —— 而且完全沒有錯誤訊息。所以 STANDBY 不是「什麼都不做」,
           是「不下令但照常收」。

        ⚠️⚠️ 2026-08-07 補的告警:「抽了但一筆都沒抽到」以前是**完全靜默**的。
           INIT 有那條 30 s 一次的「等機體定位中」,但 STANDBY 在 `_tick()` 裡
           走的是它前面那條 early return ⇒ 永遠印不到。
           實際後果:待命節點的分流埠被編隊層搶走(6960 先連者得),
           `n_pos` 恆 0、`ready_for_target` 恆 false,而 log 從啟動那三行之後
           **再也沒有任何一行**。使用者在 AeroPlan 按右鍵打擊,按了沒反應,
           現場看起來和「節點掛了」一模一樣 —— 但 `ros2 node list` 看得到它。
           待命是**長時間**狀態,不出聲的代價比 INIT 更高,不是更低。
        """
        # ⚠️ 在 1 Hz 節流**之前** —— 飛機在 STANDBY 就已起飛爬升，
        #    解除保險要跟得上那段爬升（見 _kill_arm_update）。
        self._kill_arm_update()
        now = time.time()
        if now - self._standby_stat_t < 1.0:
            return
        self._standby_stat_t = now
        self._standby_warn_pos(now)
        self.status_pub.publish(String(data=json.dumps({
            "phase": "STANDBY", "T_arr": None, "uavs": {},
            "standby": True,
            "ready_for_target": len(self.pos) >= self.n,
            "n_pos": len(self.pos), "n": self.n,
            "target_topic": TARGET_TOPIC,
            "strikes_done": self.strike_seq})))

    def _standby_warn_pos(self, now):
        """待命中定位湊不滿就出聲(30 s 一次),湊滿之後回報一次「就緒」。

        ⚠️ 只在**狀態改變**或每 30 s 印,不要每秒洗版 —— 待命是長時間狀態。
        """
        ok = len(self.pos) >= self.n
        if ok:
            if not self._standby_pos_ok:            # 由缺 → 齊,補一行正向回報
                self._standby_pos_ok = True
                self.get_logger().info(
                    f"✔ 待命定位齊備:{len(self.pos)}/{self.n} 台有 "
                    f"GLOBAL_POSITION_INT ⇒ ready_for_target=true,可以接目標了。")
            return
        self._standby_pos_ok = False
        if now - self._standby_pos_warn_t < 30.0:
            return
        self._standby_pos_warn_t = now
        miss = [i for i in range(1, self.n + 1) if i not in self.pos]
        self.get_logger().error(
            f"⚠️ 待命中收不到機體定位:{len(self.pos)}/{self.n} 台有 "
            f"GLOBAL_POSITION_INT,缺 {miss}。**ready_for_target=false,"
            f"這時候發 {TARGET_TOPIC} 不會有任何動作。** "
            f"最常見的原因是分流埠 {self._link_desc} 被別的 client 先佔走 "
            f"—— MAVProxy 的 --out=tcpin 只餵第一個連上的 client,"
            f"後連的收到 0 bytes 且沒有任何錯誤。"
            f"查法:`ss -tnpi | grep -A1 tot_node` 看有沒有 bytes_received。")

    def _publish_gate(self):
        if self.phase == "STANDBY":
            # ⚠️ 待命時 self.T 是 (nan, nan),而 json.dumps(nan) 產生的是
            #    `NaN` —— **不是合法 JSON**,嚴格的解析器(含 JS 的 JSON.parse)
            #    會整包丟掉。所以待命的 gate 明確送 target: null。
            self.gate_pub.publish(String(data=json.dumps({
                "standby": True, "target": None,
                "r_fan": self.r_fan, "half_angle": self.half,
                "approach_from": self.approach, "slots": [],
                "target_topic": TARGET_TOPIC})))
            return
        self.gate_pub.publish(String(data=json.dumps({
            "target": {"lat": self.T[0], "lon": self.T[1], "alt": self.talt},
            "r_fan": self.r_fan, "half_angle": self.half,
            "approach_from": self.approach,
            "slots": [{"id": k, "lat": s[0], "lon": s[1]}
                      for k, s in enumerate(self.slots)]})))

    def _pump(self):
        # ⚠️ 直連飛控那條線也必須抽:50 Hz 的 GLOBAL_POSITION_INT/ATTITUDE 是下到
        #    self.fc(5762+10i)的,只讀 self.link(6960 分流)的話 self.att 會是
        #    舊值或空的,攻角估計 α = θ - γ 就整個錯,俯仰指令跟著錯 ——
        #    這正是 PN 版命中高度掉到 37-53m、誤差 21-24m 的原因。
        # ⚠️ 順序有意義:兩條線寫的是**同一份** self.pos/vel/att,而分流線實測只有
        #    5-20 Hz、直連才是 50 Hz。分流線一定要抽(不抽會在 MAVProxy 端累積
        #    背壓),但要**先**抽,讓直連的新值最後落地 —— 否則每次分流吐一包,
        #    50 Hz 的姿態就被慢半拍的值蓋掉一次。
        self._pump_link()
        for i, c in self.fc.items():
            k = 0
            while k < 400:
                m = c.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE",
                                       "HEARTBEAT"],
                                 blocking=False)
                if m is None:
                    break
                k += 1
                mt = m.get_type()
                if mt == "GLOBAL_POSITION_INT":
                    self.pos[i] = (m.lat / 1e7, m.lon / 1e7,
                                   m.relative_alt / 1000.0)
                    self.vel[i] = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
                    self.gs[i] = math.hypot(m.vx, m.vy) / 100.0
                    self.home_amsl[i] = m.alt / 1000.0 - m.relative_alt / 1000.0
                    # ⚠️⚠️ 同一則訊息的**絕對**高度，單獨存一份。
                    #    釘死 AMSL 的目標要用它算 agl（見 _agl）——
                    #    relative_alt 與 home_amsl 都會隨飛控重設 HOME 而跳，
                    #    但**同一筆**裡兩者反向等量，alt 這一欄不會跳。
                    self.alt_amsl[i] = m.alt / 1000.0
                    # 海面目標(target_amsl 已釘死)不需要等 TERRAIN_REPORT —— 該問的
                    # 只有本機 HOME 的 GPS 海拔,這裡就有了。不在這裡算的話,地形回報
                    # 若沒進來(海上本來就常常沒有),z_g 會一路退回 ground_alt 預設值。
                    # ⚠️ 待命時不算 z_g:那會在**還沒有紅心**的時候印出一行
                    #    「UAVi 紅心地表 z_g = …」,而 stot_score.load_zg 是用
                    #    正規式撈這一行的 —— 待命的值會被離線工具當成本發的基準。
                    if (self.phase != "STANDBY"
                            and not math.isnan(self.target_amsl)
                            and i not in self.zg):
                        self.zg[i] = self.target_amsl - self.home_amsl[i]
                        self.zg_amsl[i] = self.target_amsl
                        self.get_logger().info(
                            f"UAV{i} 紅心地表 z_g = {self.zg[i]:+.2f} m(相對 HOME;"
                            f"目標釘死 {self.target_amsl:.2f} AMSL(海面,不查地形庫)"
                            f"− HOME GPS 海拔 {self.home_amsl[i]:.2f})")
                        self._reground()
                    # ⭐ S4a:狀態新鮮度。只在**位置**更新時打時戳 —— 導引吃的是位置,
                    #    只有姿態在更新而位置凍結,對導引而言仍然是失聯。
                    self.state_t[i] = time.monotonic()
                elif mt == "HEARTBEAT":
                    # ⭐ S1/S6:節點先前完全不追飛控模式。只記錄,不動作。
                    self.fc_mode[i] = int(getattr(m, "custom_mode", -1))
                    self.fc_mode_t[i] = time.monotonic()
                else:
                    self.att[i] = (m.roll, m.pitch, m.yaw)
                    # ⭐ S1:角速率先前整組被丟棄,而 Heun/RK2 積分 γ̇ 用得到,
                    #    訊息本來就已經在解了。
                    self.rate[i] = (getattr(m, "rollspeed", 0.0),
                                    getattr(m, "pitchspeed", 0.0),
                                    getattr(m, "yawspeed", 0.0))

    def _terrain_probe(self):
        """量紅心地表:對**紅心**和**本機 HOME** 各送一次 TERRAIN_CHECK。

        ⚠️ 兩個獨立的坑,先前都踩著:

        (1) **從來沒送過 TERRAIN_CHECK**。ArduPilot 只有收到 TERRAIN_CHECK 才會送
            TERRAIN_REPORT(`AP_Terrain::handle_terrain_check` → `send_terrain_report`,
            且把被查詢的 lat/lon 原樣帶回)。我們只被動收,拿到的其實是 MAVProxy
            替「飛機正下方」查的點 —— 參考面整場隨飛行漂移,正是 memo 裡再三警告
            不要用的那個量。實測啟動當下六機報 -28.79~-49.07 m,那是各自航路下方的
            地表,不是紅心的。

        (2) **相減的基準錯了**。舊式 `terrain_amsl(紅心) - home_amsl(GPS)` 混用了
            兩個不同的高度基準:地形資料庫(SRTM)和 GPS/EKF 的 HOME 海拔在桃園
            差了 10.9 m,不會抵消。而 SITL 物理引擎
            (`SIM_Aircraft::ground_height_difference`)用的是
            `terrain(current) - terrain(home)` —— **兩次資料庫讀值相減**,offset 抵消。
            實測:舊式六機一律得 -32.08(假的一致),正確式得
            -42.90/-48.62/-55.30/-52.12/-44.94/-38.72(散布 16.6 m,且與交接文件
            §4 記載的數值完全吻合)。以 1.8 m 誤差/1 m 地表誤差放大 → 光這一項就
            值 ~30 m 的水平誤差。

        每機都要各問一次:各 SITL instance 有自己的地形快取。"""
        # ⚠️ 待命時 self.T 是 (nan, nan),int(nan) 會拋 ValueError。下面雖然有
        #    try/except 兜著,但那會讓同一圈的 HOME_POSITION 請求也一起被跳過
        #    —— 等於待命越久、home_ll 越缺。直接不問。
        if self.phase == "STANDBY":
            return
        for i, c in self.link.items():
            try:
                c.mav.terrain_check_send(int(self.T[0] * 1e7),
                                         int(self.T[1] * 1e7))
                if i in self.home_ll:
                    hl = self.home_ll[i]
                    c.mav.terrain_check_send(int(hl[0] * 1e7), int(hl[1] * 1e7))
                else:   # MAV_CMD_REQUEST_MESSAGE(512) → HOME_POSITION(242)
                    c.mav.command_long_send(c.target_system or i, 1, 512, 0,
                                            242, 0, 0, 0, 0, 0, 0)
            except Exception:
                pass

    @staticmethod
    def _same_pt(a_lat, a_lon, b_lat, b_lon):
        return abs(a_lat - b_lat) < 1e-5 and abs(a_lon - b_lon) < 1e-5

    #: _pump_link 要處理的訊息型別。**不要**把它交給 recv_match 的 type= 參數,
    #  理由見 _pump_link 的說明。
    _LINK_TYPES = frozenset((
        "GLOBAL_POSITION_INT", "ATTITUDE", "TERRAIN_REPORT", "HEARTBEAT",
        "SIM_STATE", "HOME_POSITION"))

    def _pump_link(self):
        """抽分流鏈路(6960+10i)。

        ⚠️⚠️ 2026-08-06 修:**不可以**用 recv_match(type=[...]) 來過濾。
           pymavlink 的 recv_match 讀到「不在 type 白名單」的訊息時**一樣回 None**,
           和「沒有資料」無法區分 ⇒ 下面這個 `while` 迴圈會在**第一則不匹配的
           訊息**就 break,每個 tick 只吃掉一則。

           實測(2026-08-06,兩機航次 TWO1,直接連 6960 探 12 秒):
             SIM_STATE 106、AHRS 42、AHRS2 42、ATTITUDE 42、GLOBAL_POSITION_INT 42、
             VFR_HUD 42、SYS_STATUS 42、POWER_STATUS 42、MEMINFO 42、
             NAV_CONTROLLER_OUTPUT 42、MISSION_CURRENT 42、SERVO_OUTPUT_RAW 42
           12 種型別、白名單只佔 5 種 ⇒ 隨機取一則命中率不到一半,迴圈幾乎立刻斷。
           串流約 500 則/秒,節點 20 Hz × 1 則 = 20 則/秒 —— 永遠追不上。
           症狀:`0/N 台有 GLOBAL_POSITION_INT`,節點停在 INIT 永遠等下去,
           而 socket 的 RecvQ 一路漲(實測 22 KB,MAVProxy 端 Send-Q 到 587 KB)。
           這也正是記憶裡 master_node 那條「never-drained pymavlink WP link,
           ~1 MB MAVProxy 背壓」的同一個病。

           正確做法:**不帶過濾地收乾淨**,型別分派在 Python 端做。
        ⚠️ 加 k 上限是防「串流比 tick 還快時單機把整個 tick 吃光」——
           和 _pump() 的 400 同一個理由。
        """
        for i, c in self.link.items():
            k = 0
            while k < 600:
                m = c.recv_match(blocking=False)
                if m is None:
                    break
                k += 1
                mt = m.get_type()
                if mt not in self._LINK_TYPES:
                    continue
                if mt == "GLOBAL_POSITION_INT":
                    self.pos[i] = (m.lat / 1e7, m.lon / 1e7,
                                   m.relative_alt / 1000.0)
                    # vx/vy/vz = 北/東/下 (cm/s)
                    self.vel[i] = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
                    self.gs[i] = math.hypot(m.vx, m.vy) / 100.0
                    # home 的海拔 = 絕對高 - 相對高(用來把地形 AMSL 轉成相對高)
                    self.home_amsl[i] = m.alt / 1000.0 - m.relative_alt / 1000.0
                    # ⚠️⚠️ 同一則訊息的**絕對**高度，單獨存一份。
                    #    釘死 AMSL 的目標要用它算 agl（見 _agl）——
                    #    relative_alt 與 home_amsl 都會隨飛控重設 HOME 而跳，
                    #    但**同一筆**裡兩者反向等量，alt 這一欄不會跳。
                    self.alt_amsl[i] = m.alt / 1000.0
                    self.state_t[i] = time.monotonic()          # ⭐ S4a
                elif mt == "ATTITUDE":
                    self.att[i] = (m.roll, m.pitch, m.yaw)
                    self.rate[i] = (getattr(m, "rollspeed", 0.0),   # ⭐ S1
                                    getattr(m, "pitchspeed", 0.0),
                                    getattr(m, "yawspeed", 0.0))
                elif mt == "HEARTBEAT":                          # ⭐ S1/S6
                    self.fc_mode[i] = int(getattr(m, "custom_mode", -1))
                    self.fc_mode_t[i] = time.monotonic()
                elif mt == "SIM_STATE":
                    # SITL 真值。**只做量測,絕不進導引** —— 驗收時要能同時報
                    # 「遙測量到的 miss」與「真值 miss」:兩者差約 2 m,因為用遙測量
                    # 會把 EKF 定位誤差抵消掉,用真值量則是疊加。先都記下來,
                    # 讓使用者選驗收基準,而不是我們替他選一個好看的。
                    tlat = m.lat_int / 1e7 if getattr(m, "lat_int", 0) else m.lat
                    tlon = m.lon_int / 1e7 if getattr(m, "lon_int", 0) else m.lon
                    self.truth[i] = (tlat, tlon, m.alt)
                elif mt == "HOME_POSITION":
                    self.home_ll[i] = (m.latitude / 1e7, m.longitude / 1e7)
                elif self.phase == "STANDBY":
                    # 待命:TERRAIN_REPORT 照抽不照算(理由同 _pump 的 z_g 守衛)。
                    # ⚠️ 「抽」不能省 —— 不抽會在 MAVProxy 端累積背壓。
                    continue
                else:
                    # ⚠️ 要用 terrain_height(我們問的那一點的地表 AMSL),不是
                    #    current_height(飛機正下方的離地高)——後者的參考面會隨
                    #    飛機飛過的地形一直變。
                    # ⚠️ 必須比對回報座標:TERRAIN_REPORT 原樣帶回被查詢的 lat/lon,
                    #    不比對就會混進 MAVProxy 替「飛機正下方」查的點。
                    # ⚠️ 而且要拿**紅心地表減 HOME 地表**(兩次資料庫讀值相減),
                    #    不能減 GPS 的 home_amsl —— 見 _terrain_probe 的說明。
                    if m.spacing != 0:
                        rlat, rlon = m.lat / 1e7, m.lon / 1e7
                        if self._same_pt(rlat, rlon, self.T[0], self.T[1]):
                            self.terr_bull[i] = m.terrain_height
                        elif (i in self.home_ll
                              and self._same_pt(rlat, rlon, *self.home_ll[i])):
                            self.terr_home[i] = m.terrain_height
                    # ⚠️⚠️ 目標在**海上**時絕不能問地形資料庫。SRTM 在開闊水域沒有
                    #    資料,回的是外插/快取的垃圾:同一個紅心座標,六架各報一個
                    #    高度,而且是單調爬升的。實測 P91 紅心 AMSL 報成
                    #    -10.70/-7.40/-4.13/-1.59/+0.86/+3.53,EL1 報成
                    #    -11.02/-16.57/-23.28/-20.11/-12.95/-6.64 —— 散布 17 m,
                    #    直接吃掉 10 m 精度預算。海面就是 0 m AMSL,是已知常數,
                    #    設 target_amsl 把它釘死,不要向資料庫要一個我們早就知道的數。
                    #    仍然用「目標 AMSL − HOME 地表(同一資料庫)」相減,維持
                    #    上面那條「不能減 GPS home_amsl」的規則。
                    pinned = not math.isnan(self.target_amsl)
                    zg = src = None
                    if pinned and i in self.home_amsl:
                        # ⚠️ 釘死模式下**減數要換成 GPS 的 home_amsl**,不能再用
                        #    terr_home。上面那條「用資料庫相減」的規則,前提是紅心與
                        #    HOME 兩個讀值都來自同一個資料庫、共用同一個基準面偏移,
                        #    相減才會抵消。現在紅心改成「真 AMSL 0(海面)」,而飛機
                        #    回報的高度是 GPS 相對 HOME —— 要跟 GPS 的 HOME 海拔相減
                        #    才同基準。混用會把資料庫與 GPS 的基準差直接算進殘餘高度。
                        zg = self.target_amsl - self.home_amsl[i]
                        src = (f"目標釘死 {self.target_amsl:.2f} AMSL(海面,不查地形庫)"
                               f"− HOME GPS 海拔 {self.home_amsl[i]:.2f}")
                    elif not pinned and i in self.terr_bull and i in self.terr_home:
                        zg = self.terr_bull[i] - self.terr_home[i]
                        src = (f"紅心 {self.terr_bull[i]:.2f} − HOME 地表 "
                               f"{self.terr_home[i]:.2f} AMSL")
                    if zg is not None:
                        # 紅心地表的 AMSL(GPS 基準)= 換算 SIM_STATE 真值高度用
                        if pinned:
                            self.zg_amsl[i] = self.target_amsl
                        elif i in self.home_amsl:
                            self.zg_amsl[i] = self.home_amsl[i] + zg
                        if i not in self.zg:
                            self.zg0[i] = zg        # log 印過的那個「初始值」
                            self.get_logger().info(
                                f"UAV{i} 紅心地表 z_g = {zg:+.2f} m(相對 HOME;{src})")
                        # ⚠️⚠️ 陸地目標(target_amsl=NaN)時這一行是**無條件賦值**:
                        #    每收到一次 TERRAIN_REPORT 就更新,但只有第一次會印。
                        #    stot_score.load_zg 只讀得到那第一個值,而 CSV 的 agl
                        #    欄用的是**時變**的 z_g ⇒ 兩套基準悄悄分岔。基準混用
                        #    是本專案列為最高風險的一類錯,所以漂移必須出聲。
                        #    ⚠️ 措辭刻意避開 `紅心地表 z_g = `,才不會被 load_zg
                        #       的正規式撈走、把「初始值」的語意改掉。
                        elif abs(zg - self.zg_warn.get(i, self.zg0[i])) > 1.0:
                            self.zg_warn[i] = zg
                            self.get_logger().warn(
                                f"⚠️ UAV{i} 紅心地表高度漂移:初始 "
                                f"{self.zg0[i]:+.2f} → 現在 {zg:+.2f} m"
                                f"(Δ{zg - self.zg0[i]:+.1f} m;{src})。"
                                f"CSV 的 agl 欄跟著新值走,log 只印過初始值 ——"
                                f"離線評分請以 CSV 的 agl 欄為準。")
                        self.zg[i] = zg
                        self._reground()

    def perch_of(self, i):
        """本機的 perch 高度(m,相對 HOME)。

        ⚠️ 高度通道協同的核心:各機站在**不同高度**的環上,環半徑
           r_i = (perch_i − dive_alt)/tan(γ) 因此不同,俯衝時間也就不同。
           把「誰先到」從油門(速度)搬到幾何(高度),油門就自由了。
        """
        k = self.slot_id.get(i, 0) if self.perch_step else 0
        return self.perch_alt + self.perch_step * k

    def _fan_off(self, k):
        """扇形第 k 個 slot 相對進場方位的偏置(度)。

        ⚠️⚠️ 2026-08-07:原本五處都直接寫 `2*half*k/(self.n - 1)`,而
           `_ring_points()` 由 `__init__` 呼叫 ⇒ **n=1 會在節點建構期就
           ZeroDivisionError,tot_node 根本起不來**(而 stot_trial.sh 已經會
           傳 `-p n:=` ⇒ SWARM_COUNT=1 真的走得到)。
        ⚠️ n=1 取**扇形中心**(0.0)而不是 `max(1, n-1)` 硬算 —— 後者會給
           `-half`,也就是扇形**邊緣**,單機沒有理由飛到邊上。
        ⚠️ n>=2 逐位元不變:分母與原式相同,運算順序也相同。
        ⚠️ 另有四處 `(self.n - 1) / 2.0` 是**乘法項**(高度分層),n=1 給 0
           是正確的,**不要**一起改。
        """
        if self.n <= 1:
            return 0.0
        return -self.half + 2.0 * self.half * k / (self.n - 1)

    def hold_alt_of(self, i):
        """等待/守環段的目標高度(m,相對 HOME)。

        ⭐ S8:與 `ring_of()` **刻意解耦** —— 環半徑仍由 `perch_alt` 決定,
        等待高度另外給。這樣才能在不動環幾何的前提下降低交接高度,
        而交接 γ = atan(h/gate) 正是命中與否的唯一自變數。

        ⚠️ 只准比 `perch_of(i)` 低:高於它會把 `perch_step` 的高度階梯抹平,
           而那個階梯是「把同時性從油門搬到幾何」的既有機制。
        ⚠️ hold_alt = 0 ⇒ 回傳 perch_of(i) ⇒ **逐位元同現行行為**。
        """
        if self.hold_alt_cfg <= 0.0:
            return self.perch_of(i)
        return min(self.perch_of(i), self.hold_alt_cfg)

    def hold_v_of(self):
        """等待/守環段的速度指令。0 = vmin(現行行為)。

        ⚠️ 提高它的**唯一目的**是讓油門離開地板 —— 油門觸底會觸發 TECS 的
        抗失控閂鎖,把降高需求扣住而且不設任何旗標(見 hold_v 的參數說明)。
        ⚠️ 夾在 [vmin, vmax]:低於 vmin 會被飛控整發拒收(MAV_RESULT=4),
           高於 vmax 同理。
        """
        if self.hold_v_cfg <= 0.0:
            return self.vmin
        return max(self.vmin, min(self.vmax, self.hold_v_cfg))

    def ring_of(self, i):
        """本機的 pushover 環半徑(m)。perch_step=0 時退化成全隊共用。"""
        if not self.perch_step:
            return self.pushover_r
        return max(self.perch_reach,
                   (self.perch_of(i) - self.dive_alt)
                   / math.tan(math.radians(self.dive_angle)))

    def ring_point_of(self, i):
        """本機的環點(依自己的環半徑,不是全隊共用的)。"""
        if not self.perch_step:
            return self.pp[self.slot_id[i]]
        off = self._fan_off(self.slot_id[i])
        brg = (self.approach + off) % 360.0
        return dest(self.T[0], self.T[1], brg, self.ring_of(i))

    def gate_of(self, i):
        """本機**當下**的 PN 交棒距離(m)。每拍現算,不快取。

        ═══ 為什麼不是常數,也不是環半徑的固定比例 ═══
        交棒之後 PN 段要在剩下的水平航程內把高度卸完 —— 而航點/TECS 段
        幾乎沒有下降權限(實測指令降高只被吃掉 1–24%),所以**待卸高度
        ≈ 全部**。決定成敗的是「PN 段被要求飛多陡」,不是交棒距離本身。

        全檔案庫掃描(169 份 log / 828 架次,scripts 見 scratchpad/scan_gate.py):
            Spearman(殘餘高度, 交棒視線俯角 γ=atan(h/d)) = **+0.892**(8 個箱全單調)
            Spearman(殘餘高度, 比值 gate/環半徑 f)       = **+0.103**(幾乎無關)
        且 f≈0.39 的 66 架次殘餘從 3 m 到 1879 m(626 倍),γ 解釋得了、f 解釋不了。
        ⇒ **綁環半徑(固定 f)是錯的參數化**;要綁的是「要求 PN 交出的航跡角」。

        ═══ 公式(幾何,不是文獻結論)═══
            R_turn = V² / (g·(cos γ_now − n_z))          頂端推頭圓弧半徑
            h_arc  = R_turn·(1 − cos θ_max)              圓弧本身能卸掉的高度
            d_min  = R_turn·sin θ_max + (h − h_arc)/tan θ_max     (h > h_arc)
                   = √(h·(2·R_turn − h))                          (圓弧就夠)
            gate   = clamp(d_min / Φ_gate, ...)
        ⭐ 式子裡**沒有設計俯角 γ_design** —— 環的俯角只決定環半徑,不決定交棒距。
           這正是「先訂角度、再由它決定 PN 段距離」的正確形式。

        ═══ 證據等級(逐項)═══
        [幾何] d_min 的兩支式子與單調性(越陡越省航程 ⇒ 取 θ_max 是最小值)。
        [文獻] 「交棒門檻 = 動態幾何量的倍率」與「先訂角度、釘一個座標、讓距離浮動」
               的結構:NASA TM-104744 §3.9(TAEM `RCIR < 1.1·RTURN`)、
               USA005512 p.4-7(「all glide slopes have the same pullup altitude」)。
               ⚠️ 那些倍率原文自承 empirical,**形式可搬、數值不可搬**。
        [文獻缺口] 「由待卸高度反推交棒距離」在文獻上**不存在**:對 5 篇協同導引
               論文全文掃 altitude/glide slope/energy/gravity/drag ⇒ 命中 0 次
               (它們是等速質點、無高度維度)。架構最像我們的 Yu 2020(CJA
               33(3):990–1005)在 Remark 4 與 Conclusions 明文把「the design
               criterion of the demarcation points for the two stages」列為 future work。
        [相關] θ_max 與 Φ_gate 的數值是用我們自己三趟六架次校準的,無受控實驗。

        ⚠️ V 用**總速** hypot(gs, vz) 而不是 gs —— CSV 的 gs 只有水平分量,
           用它會低估 R_turn ⇒ 交棒點偏近 ⇒ 偏不安全。vz 早就存在 self.vel 裡。
        """
        if self.gate_theta_max <= 0.0:
            return self.gate_r                     # 退回絕對值(逐位元舊行為)
        p = self.pos.get(i)
        if p is None:
            return self.gate_r
        h = p[2] - self._zg(i)                     # 離紅心地表的高度
        if h <= 0.0:
            return self.gate_r
        v = self.vel.get(i)
        gs = self.gs.get(i, 0.0)
        vz = v[2] if v else 0.0                    # MAVLink vz:向下為正
        V = math.hypot(gs, vz)
        # ⚠️⚠️ 本檔有**兩種相反的 γ 符號約定**,而且三處都叫 `gam`:
        #   ① 這裡(gate_r_of):vz 是 MAVLink 的**下正** ⇒ **下降為正**
        #   ② _pn_cmd:hdot = −vd 是**上正** ⇒ **下降為負**
        #   ③ CSV 的 gam_deg:atan2(−vz, …) ⇒ **下降為負**(與 ② 同)
        # 各自內部自洽,不要「順手統一」—— 改符號會直接翻轉這裡的比較方向。
        # 但引用任何一個 γ 時**必須先確認是哪一種**。
        # tests/test_gamma_sign_conventions.py 鎖住這三處。
        gam = math.atan2(vz, max(gs, 1e-6))        # ① 下降為正
        den = math.cos(gam) - self.pn_nz_min
        if V <= 1.0 or den <= 1e-6:
            return self.gate_r
        th = math.radians(self.gate_theta_max)
        r_turn = V * V / (G0 * den)
        h_arc = r_turn * (1.0 - math.cos(th))
        if h > h_arc:
            d_min = r_turn * math.sin(th) + (h - h_arc) / math.tan(th)
        else:
            d_min = math.sqrt(max(h * (2.0 * r_turn - h), 0.0))
        g = d_min / max(self.gate_phi, 1e-3)
        # 上限有三道:
        #   ① 絕對上限 gate_r_max
        #   ② 不得超過本機環半徑(交棒不能發生在抵環之前,否則放行判定與 PN 交接的
        #      先後會反轉)。AM21 在 8.3–10.5 km 交棒 ⇒ 誤差 22.6 km。
        #   ③ ⭐⭐ 2026-08-14 新增:**不得落進守環擺盪帶**。
        #      守環是 loiter,飛機距紅心的水平距在 [環 ± hold_radius] 之間擺盪。
        #      gate 若落在帶內,放行當下飛機可能已經在 gate 內側 ⇒ 當場交棒
        #      ⇒ **交棒距離變成隨機量**,而交棒幾何是命中與否的唯一自變數。
        #      RPO 實測代價(gate 5600 / hold 1300 / 環 6147):
        #          交棒距 ±11 m → ±1075 m、真落地 5/12 → 1/6。
        #      ⚠️ 這一道對**靜態** gate_r 由 contract_table 的同名不變式在起飛前擋;
        #         動態 gate 每拍現算,契約層看不到,所以必須在這裡夾。
        #         少了它,打開 gate_theta_max 就是直接重演 RPO。
        swing_inner = self.ring_of(i) - self.hold_radius
        cap = min(self.gate_r_max, self.ring_of(i))
        if swing_inner > self.gate_r_min:
            cap = min(cap, swing_inner)
        return max(self.gate_r_min, min(g, cap))

    def _ring_points(self):
        """扇形 slot / pushover 環點 / 穿越點。環半徑改變時要重算(見 _reground)。"""
        self.slots, self.pp, self.thru = [], [], []
        for k in range(self.n):
            off = self._fan_off(k)
            brg = (self.approach + off) % 360.0
            self.slots.append(dest(self.T[0], self.T[1], brg, self.r_fan))
            self.pp.append(dest(self.T[0], self.T[1], brg, self.pushover_r))
            self.thru.append(dest(self.T[0], self.T[1],
                                  (brg + 180.0) % 360.0, self.overshoot))

    def _home_amsl_mean(self):
        """全隊起飛點海拔的**均值濾波**估算(m AMSL)。

            Home_AMSL = (1/N)·Σ (alt_i_AMSL − alt_i_relative)

        每機的 GPS 絕對海拔減掉它自己的相對高度,就是它的起飛點海拔;
        六機取平均把單機的 GPS 垂直雜訊(數公尺級)壓下去。

        ⚠️ **這是 Home-invariance 的基礎**:系統不該依賴任何寫死的起飛點高度。
           先前 stot_trial.sh 把 ground_alt 寫死 −48,而實際 z_g 是 −1886 ~ −2703,
           換一個起飛點就換一組錯誤,而且完全靜默(見 _reground 的說明)。
        ⚠️ 回傳 None 表示還沒收到足夠的回報 —— 呼叫端必須處理,不要當 0 用。
        """
        vals = [v for v in self.home_amsl.values() if v is not None]
        if len(vals) < max(2, self.n // 2):
            return None
        return sum(vals) / len(vals)

    def _reground(self):
        """⭐⭐ 量到真實地形後重算 dive_alt 與環半徑。

        ⚠️⚠️ 這是本專案「掉不到底、從紅心上空 1500–2300 m 飛過」的**根因**,
        而且它偽裝成導引律問題,害我們在 PN / 俯角 / 推頭權限上繞了很久。

        auto_ring 在 __init__ 就用 `ground_alt` 這個**寫死的常數**反推環半徑
        (stot_trial.sh 給 −48),但 LOC 南移 100 km 之後 HOME 落在中央山脈
        1839 m AMSL、紅心在 −30 m ⇒ 真實 z_g 是 **−1869 m**,差了 1821 m。
        於是環半徑照「要掉 3100 m」算,飛機實際要掉 4969 m。

        殘餘高度 = 真實落差 − pushover_r·tanγ = 4969 − 3148 = **1821 m**,
        和實測完全對得上(AC1 UAV1 1879、AB1 UAV3 1518、AD1 1982–2336)。
        命中的那幾架是 PN 硬拖下來的,超出了指令下滑線 —— 所以「有時候會中」
        更誤導,讓人以為是隨機的權限不足。

        地形是每機各自回報的(zg 有 40 m 級的散布),取全隊平均當幾何基準。
        """
        # ⚠️ 待命時沒有紅心可言 —— zg 若被 target_amsl 釘死的那條路徑填滿,
        #    這裡會拿「上一發的環半徑」去重算,而下一發一接目標就會被
        #    _reset_for_new_target 還原成 _ring_r0,兩邊打架。直接不算。
        if self.phase == "STANDBY":
            return
        if not self.auto_ring or len(self.zg) < self.n:
            return
        zg_mean = sum(self.zg.values()) / len(self.zg)
        # 用 GPS 的均值濾波起飛點海拔做**獨立交叉檢核**。兩者來源不同:
        #   zg_mean   來自地形資料庫(TERRAIN_REPORT):紅心地表 − HOME 地表
        #   home_amsl 來自 GPS:alt_AMSL − alt_relative,再跨機平均
        # 若地形資料庫沒載到(海面、資料缺格),zg 會是 0 而看起來「正常」——
        # 這個檢核會把它抓出來,而不是讓整趟飛完才發現殘餘高度不對。
        # ⚠️⚠️ 這段交叉檢核的 log **必須放在冪等守衛之後**。
        #    放在前面時每一筆 TERRAIN_REPORT 都印一行 —— 實測 HS1/G1/NEW1
        #    各刷出 4 萬行,佔整份 trial log 的 **99.90%**,把唯一一行有效的
        #    「UAVn→slotN」埋掉。階段 3 卡死了四趟沒被發現,這是主因之一。
        #    ⇒ 觀測性本身就是驗收能力的一部分:看不見的失敗等於沒有失敗。
        dive_alt_new = zg_mean + self.dive_alt_agl
        if abs(dive_alt_new - self.dive_alt) < 5.0:
            return                      # 已經一致,不要每個 tick 重算
        # ⚠️ 六架問的是**同一個紅心座標**,z_g 本來就該一樣。散布大 = 地形資料庫在
        #    亂給(水域無資料時回外插值)。先前只是「取平均」把它吞掉,平均掉的是
        #    誤差的一半、不是誤差本身:實測 P91 散布 14.3 m、EL1 16.6 m,而命中門檻
        #    只有 10 m。這裡把它講出來,並指向正解 target_amsl。
        zg_spread = max(self.zg.values()) - min(self.zg.values())
        if zg_spread > 2.0 and math.isnan(self.target_amsl):
            self.get_logger().warn(
                f"⚠️ 六機對同一紅心回報的地表高度散布 {zg_spread:.1f} m —— 同一點不該有"
                f"多個高度,地形資料庫在該處無有效資料(海面?)。取平均只會把誤差減半。"
                f"海上目標請設 target_amsl=0.0 釘死海平面。各機 z_g = "
                f"{ {i: round(v, 1) for i, v in sorted(self.zg.items())} }")
        ham = self._home_amsl_mean()
        if ham is not None:
            basis = ("地形資料庫" if math.isnan(self.target_amsl)
                     else f"釘死海平面 {self.target_amsl:.1f} AMSL")
            self.get_logger().info(
                f"起飛點海拔(GPS 均值濾波,n={len(self.home_amsl)})= {ham:+.1f} m AMSL;"
                f"{basis}給的紅心相對落差 z_g = {zg_mean:+.1f} m "
                f"(跨機散布 {zg_spread:.1f} m)"
                f"⇒ 紅心地表約 {ham + zg_mean:+.1f} m AMSL")
            if abs(zg_mean) < 1.0 and abs(ham) > 200.0:
                self.get_logger().warn(
                    "⚠️ z_g ≈ 0 但起飛點在 %.0f m AMSL —— 地形資料庫可能沒載到,"
                    "俯衝幾何會少算落差。海面目標屬正常,山區請檢查地形快取。" % ham)
        drop = self.perch_alt - dive_alt_new
        r_new = max(self.perch_reach,
                    drop / math.tan(math.radians(self.dive_angle)))
        self.get_logger().warn(
            f"★ 依實測地形重算幾何:紅心地表 z_g={zg_mean:+.0f}m ⇒ 真實落差 "
            f"{drop:.0f}m(原以為 {self.perch_alt - self.dive_alt:.0f}m);"
            f"環半徑 {self.pushover_r:.0f} → {r_new:.0f}m。"
            f"⚠️ 不重算的話殘餘高度會是 "
            f"{drop - self.pushover_r * math.tan(math.radians(self.dive_angle)):.0f}m")
        self.dive_alt = dive_alt_new
        self.pushover_r = r_new
        self._ring_points()

    # ══════════════════════════════════════════════════════════════════
    #  終端比例導引(PN)—— 直接下姿態,繞開 L1 與 TECS
    # ══════════════════════════════════════════════════════════════════
    def _fc_connect(self):
        """另開一組「直連飛控」的連線,並把狀態拉到 50 Hz。"""
        for i in range(1, self.n + 1):
            try:
                port = self.fc_base + self.fc_stride * (i - 1)
                c = mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{port}", source_system=234,
                    dialect="ardupilotmega")
                c.wait_heartbeat(timeout=6)
                for mid in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
                    c.mav.command_long_send(
                        c.target_system or i, 1,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        float(mid), self.tel_us, 0, 0, 0, 0, 0)
                self.fc[i] = c
                self.get_logger().info(f"UAV{i} 直連飛控 {port} 就緒(50 Hz)")
            except Exception as e:
                self.get_logger().warn(f"UAV{i} 直連飛控失敗:{e}")

    def _att_target(self, i, roll, pitch, thr):
        """SET_ATTITUDE_TARGET:roll/pitch 直接寫進 nav_roll/nav_pitch。

        type_mask = 132 (0x84)。⚠️ 不是直覺的 7:韌體會先把 mask 反相(^0xFF)
        再解讀。132 反相成 0x7B ⇒ roll ✓ pitch ✓ yaw ✗,且油門一併接管。
        ⚠️ 必須持續發:GUIDED_TIMEOUT 3.0 s,斷流就默默掉回 L1。"""
        # ⭐ S4/S5/S6 安全兜底。全部預設關閉 ⇒ 回傳 None ⇒ 行為逐位元同舊版。
        why = self._att_gate(i)
        if why is not None:
            self.att_block[i] = why
            self.att_block_n[i] = self.att_block_n.get(i, 0) + 1
            if self.att_block_n[i] == 1 or self.att_block_n[i] % 100 == 0:
                self.get_logger().error(
                    f"⚠️ UAV{i} 姿態指令被兜底擋下(第 {self.att_block_n[i]} 次):{why}"
                    f" —— **停發**,讓 GUIDED_TIMEOUT({GUIDED_TIMEOUT_S:.0f}s)刻意觸發")
            self._att_handback(i)
            return
        fc = self.fc.get(i)
        # ⚠️⚠️ S4b:先前是 `self.fc.get(i) or self.link[i]` —— fc 缺席就**靜默**退到
        #    6860 分流線。那條線只有 5-20 Hz 而且有背壓失效史(wp_link_drain),
        #    姿態指令掉上去不會有任何訊息,只會變得又慢又抖。
        c = fc if (fc is not None or self.att_require_fc) else self.link[i]
        if c is None:
            self.att_block[i] = "直連飛控鏈路缺席且 att_require_fc=1"
            self.att_block_n[i] = self.att_block_n.get(i, 0) + 1
            if self.att_block_n[i] == 1:
                self.get_logger().error(
                    f"✗ UAV{i} 沒有直連飛控鏈路,而 att_require_fc=1 ⇒ **不降級、停發姿態**")
            return
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        c.mav.set_attitude_target_send(
            0, c.target_system or i, 1, 132,
            [cr * cp, sr * cp, cr * sp, -sr * sp], 0.0, 0.0, 0.0, float(thr))

    # ── S4/S5/S6:姿態通道的兜底閘門 ──────────────────────────────────
    def _att_gate(self, i):
        """姿態指令送出前的兜底檢查。回傳 None = 放行,回傳字串 = 擋下的理由。

        ⚠️⚠️ 這些檢查存在的理由:把姿態通道從終端段往前延伸到滑翔段,等於把能量
        管理從 TECS 手上接過來。終端段只有十幾秒、掉回 L1 的後果有限;滑翔段
        長得多,而且 `swarm_sitl_up.sh:253-258` 記著沒有 TECS 兜底的下場:
        「alt 227 → −5.9,11 秒撞地」。

        ⚠️ **每一條預設都是關閉的**,所以本函式預設回傳 None,行為與舊版逐位元相同。
           要啟用哪一條就設哪一個參數 —— 這樣才能單獨歸因。
        """
        # S4a 狀態新鮮度:位置凍結時**主動停發**,讓 GUIDED_TIMEOUT 刻意觸發。
        # ⚠️ 不停發的話,凍結的姿態會以 20 Hz 持續送出 ⇒ 逾時永遠不會到 ⇒
        #    飛機用一個固定姿態飛進地面,而且全程沒有任何告警。
        if self.state_max_age > 0.0:
            t = self.state_t.get(i)
            if t is None:
                return "從未收到位置"
            age = time.monotonic() - t
            if age > self.state_max_age:
                return f"狀態過期 {age:.2f}s > {self.state_max_age:.2f}s"
        # S6 飛控模式:不在 GUIDED 時 SET_ATTITUDE_TARGET 會被**靜默丟棄**。
        if self.att_require_guided:
            m = self.fc_mode.get(i)
            if m is None:
                return "讀不到飛控模式(HEARTBEAT 未進來)"
            if m != PLANE_MODE_GUIDED:
                return f"飛控模式 {m} 不是 GUIDED({PLANE_MODE_GUIDED})"
        # S5b EAS 地板。⚠️ 用 EAS 不是地速:157 是 3100 m 的地速(= EAS 135),
        #    拿 157 當 EAS 門檻會超天花板 17%(eas-vs-groundspeed-157)。
        if self.att_eas_floor > 0.0 and i in self.pos:
            eas = self._eas_of(i)
            if eas is not None and eas < self.att_eas_floor:
                return f"EAS {eas:.1f} < 地板 {self.att_eas_floor:.1f}"
        # S5c AGL 地板。契約層另外強制它 > kill_agl_m(否則地板比終結線還低)。
        if self.att_agl_floor > 0.0:
            agl = self._agl(i)
            if agl is not None and agl < self.att_agl_floor:
                return f"AGL {agl:.1f} < 地板 {self.att_agl_floor:.1f}"
        return None

    def _observe_timing(self, i, now):
        """T2:計算並記錄抵達時間誤差 e_t。**只觀測,不影響任何指令。**

        e_t = (t_c − now) − t_go        > 0 表示「太早到」

        ⚠️⚠️ 這裡刻意**不看** `weave_time_sync`:觀測必須先於致動,
        否則「閉迴路有沒有效」永遠只能靠事後反推。
        第一版把 e_t 寫在 `_weave_theta_of()` 裡,結果 T3 關著時整欄是空的 ——
        那讓 T1 的驗證航次白跑了一趟(RPX,2026-08-14)。
        """
        if self.dive_T is None or i not in self.pos:
            return
        try:
            d = dist_m(self.pos[i][:2], self.T) if self.T else None
            if d is None:
                return
            tgo = self._tgo_of(i, d)
            self.tgo_obs[i] = tgo
            self.e_t[i] = (self.dive_T - now) - tgo
        except Exception:                                        # noqa: BLE001
            pass

    def _predict_tick(self, i, now):
        """S9 影子模式:跑一次終端預測,只存起來,**不影響任何指令**。

        ⚠️⚠️ 這裡刻意**不**回傳、不被任何導引邏輯讀取 —— 影子的意義就是
        「先證明它在飛行中準不準」,再談拿它做決策。任何一行用 self.pred
        去改指令的程式碼都必須是另一個步驟、另一個開關、另一趟驗證。
        """
        if _pred is None or self.predict_hz <= 0.0 or self.T is None:
            return
        if self.pred_t.get(i, 0.0) + 1.0 / self.predict_hz > now:
            return
        self.pred_t[i] = now
        try:
            _dn, _de, d, h, _R, _vn, _ve, vh, hdot, V = self._geom(i)
            cap = (self.predict_gamma_cap if self.predict_gamma_cap > 0.0
                   else _pred.PN_GAMMA_CAP_DEG)
            # ⚠️ home_amsl 給紅心地表的 AMSL:海上目標釘死 0。密度要用 AMSL 算,
            #    用相對 HOME 的高度會在地形落差大的地方系統性偏(1821 m 那個坑)。
            zg_amsl = self.zg_amsl.get(i)
            self.pred[i] = _pred.predict_terminal(
                h, d, V, thr_max_pct=self.predict_thr_max,
                home_amsl=(0.0 if zg_amsl is None else float(zg_amsl)),
                gamma_cap_deg=cap)
        except Exception as e:                                   # noqa: BLE001
            # ⚠️ 影子模式失敗**不可以**影響飛行,但要留下痕跡 —— 靜默失敗是本專案的病。
            if not self._pred_warned:
                self._pred_warned = True
                self.get_logger().error(f"⚠️ 線上預測器失敗(影子模式,不影響指令):{e!r}")

    def _eas_of(self, i):
        """由 3D 地速與高度估 EAS。⚠️ 無風假設(SITL 目前無 SIM_WIND 設定)。"""
        v = self.vel.get(i)
        if v is None or i not in self.pos:
            return None
        tas = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        h = max(0.0, self.alt_amsl.get(i, self.pos[i][2]))
        sig = max(1e-3, (1.0 - 2.25577e-5 * h) ** 4.25588)
        return tas * math.sqrt(sig)

    def _att_handback(self, i):
        """S5a:釋放姿態通道前,把 guided 航點高度**重錨到當下高度**。

        ⚠️⚠️ 不做的話,TECS 會帶著整段下降累積出來的**過期 hgt_dem** 接手 ——
        在 200 m/s 下那是滿油門 + PTCH_LIM_MAX 的暴力拉起,而且全程靜默。
        預設關閉;要在啟用姿態接管時一起開。
        """
        if not self.att_handback or i not in self.pos:
            return
        if self.handback_t.get(i, 0.0) + 1.0 > time.monotonic():
            return                       # 每秒最多一次,避免和導引指令打架
        self.handback_t[i] = time.monotonic()
        try:
            lat, lon, z = self.pos[i]
            # ⚠️ _reposition 本來就一律帶 CHANGE_MODE(見它的 docstring),
            #    所以這裡不需要額外旗標;半徑用 goto_radius(飛過去語意)。
            self._reposition(i, lat, lon, z, radius=self.goto_radius)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().warn(f"UAV{i} 姿態交棒重錨失敗:{e!r}")

    def _geom(self, i):
        """相對紅心的 3D 狀態(當地切平面,1.5 km 內誤差 < 1 cm)。"""
        lat, lon, z = self.pos[i]
        dn = (self.T[0] - lat) * EARTH
        de = (self.T[1] - lon) * mlon(lat)
        d = max(1e-3, math.hypot(dn, de))
        h = z - self._zg(i)
        R = max(1e-3, math.hypot(d, h))
        vn, ve, vd = self.vel.get(i, (0.0, 0.0, 0.0))
        vh = math.hypot(vn, ve)
        hdot = -vd
        V = max(1e-3, math.hypot(vh, hdot))
        return dn, de, d, h, R, vn, ve, vh, hdot, V

    @staticmethod
    def _parse_sigma_spec(text):
        """解析 `"1:0:20,2:180:20"` → `{1: (0.0, 20.0), 2: (180.0, 20.0)}`。

        ⚠️ 解析失敗一律**大聲丟掉整條規格**而不是部分接受:半套規格會讓
           一半機體走實驗路徑、一半走生產路徑,而那是分不出來的髒對照。
        """
        out = {}
        t = text.strip().lower()
        # ⚠️ "off"/"none" 是**顯式停用**的哨兵。空字串在 shell 端不能用 ——
        #    `-p name:=` 會讓 rcl 解析失敗、節點起不來(RRA 整批報廢的原因)。
        if not t or t in ("off", "none", "-"):
            return out
        for part in text.split(","):
            bits = part.strip().split(":")
            if len(bits) != 3:
                return {}
            try:
                out[int(bits[0])] = (float(bits[1]), float(bits[2]))
            except ValueError:
                return {}
        return out

    def _lead_angle_deg(self, i):
        """前置角 σ:速度向量與「指向紅心的視線」之間的夾角(度,取絕對值)。

        σ = 0°   機頭正對紅心
        σ = 180° 機頭正背對紅心

        ⭐ 單一入口。`_itcg_bias` 的側向選擇與放行的航向閘門用的**必須是同一個
        定義** —— 兩處各寫一次正弦餘弦是 T1 那個教訓的翻版(六個地方各寫
        `d/_pace_of`,要換就得改六處,漏一個就靜默不一致)。
        """
        p, v = self.pos.get(i), self.vel.get(i)
        if p is None or v is None or self.T is None:
            return None
        dn = (self.T[0] - p[0]) * 111320.0
        de = (self.T[1] - p[1]) * 111320.0 * math.cos(math.radians(p[0]))
        if math.hypot(dn, de) < 1.0 or math.hypot(v[0], v[1]) < 1.0:
            return None
        cross = dn * v[1] - de * v[0]
        dot = dn * v[0] + de * v[1]
        return abs(math.degrees(math.atan2(cross, dot)))

    def _sigma_gate_ok(self, i):
        """⚗️ **實驗專用**的放行閘門:只在指定的前置角視窗放行。

        ⚠️⚠️ 這條路徑**只為了一個因果實驗**存在,不是生產邏輯。

        要回答的問題:俯衝耗時 σ=6.46 s 的變異,是**放行航向**造成的,
        還是**放行距離**造成的?現有 27 架次裡,乾淨的那 4 架同時滿足
        「航向 27°」與「距離 6760 m」,**完全共線,分不開**。

        ⭐ 幾何上有一個免費的對照:從紅心看盤旋圓的**兩條切線**,
           切點到紅心的距離都是 `sqrt(D² − a²)`(D=環半徑、a=hold_radius)——
           **完全相同**,但一個切點機頭朝向紅心、另一個背對。
           實算 D=6147、a=1300 ⇒ 兩點都在 6058 m,σ 分別是 2° 與 178°。
           ⇒ 放兩架在兩個切點,**距離自動被控制住,只有航向不同**。
           不需要 LM 建議的 2×2 全因子(而且在圓形盤旋上 R 與 σ 本來就耦合,
           2×2 的四格根本取不到)。

        規格字串:`"1:0:20,2:180:20"` = sysid 1 等 σ=0°±20°、sysid 2 等 180°±20°。
        """
        spec = self.release_sigma_spec.get(i)
        if spec is None:
            return True                      # 沒指定的機體不受此閘門管
        sig = self._lead_angle_deg(i)
        if sig is None:
            return False
        want, tol = spec
        ok = abs(sig - want) <= tol
        # ⚠️⚠️ 閘門不可以**無聲餓死**。RQE(2026-08-15)的教訓:
        #    對準組等 σ≤8° 等了 84 s,而 miss 窗(dive_T+60 s)先關了 ⇒
        #    對照組**一筆資料都沒有**,而 log 只說「逾時 MISS」,
        #    看起來像導引失敗,實際上是**我的實驗閘門把飛機餓死了**。
        #    ⇒ 逾時就放行,但把這一筆**標記出來**,分析時要能剔除。
        if not ok and self.release_sigma_timeout > 0.0:
            t0 = self.sigma_wait_t0.setdefault(i, time.monotonic())
            if time.monotonic() - t0 >= self.release_sigma_timeout:
                if not self.sigma_gate_starved.get(i):
                    self.sigma_gate_starved[i] = True
                    self.get_logger().error(
                        f"⚠️⚠️ UAV{i} 航向閘門**逾時放行**:等了 "
                        f"{self.release_sigma_timeout:.0f}s 仍未進入 "
                        f"{want:.0f}±{tol:.0f}° 視窗(當下 σ={sig:.1f}°)⇒ "
                        f"照常放行。**這一架次的航向不是實驗指定值,分析時必須剔除。**")
                return True
        if ok and not self.sigma_gate_hit.get(i):
            self.sigma_gate_hit[i] = True
            self.get_logger().warn(
                f"⚗️ UAV{i} 航向閘門放行:σ={sig:.1f}°(目標 {want:.0f}±{tol:.0f}°)")
        return ok

    def _itcg_bias(self, i, now, dn, de, d, R, vn, ve, vh, V):
        """ITCG:命中時間控制的**橫向**偏置(m/s²)。回傳 (a_t, e_t_m)。

        ⭐⭐ 為什麼要有這一項(使用者 2026-08-15 定調:傳統 PN 不適合多機):
           純 PN 的式子裡**沒有時間這個量**。它保證撞到,不保證什麼時候撞到。
           實測反證:RQA2 兩架的放行幾何幾乎相同(5120 m/31° 與 5590 m/29°),
           卻飛出 γ 中位 −27.1° vs −32.4°、俯衝耗時 **55 s vs 33 s**。
           上游已經對齊到放行差 −2.5 s,PN 仍然自己散開 21 s ——
           因為它沒有任何一項在看時間。

        ## 誤差用**公尺**不用秒(Kim & Shin 2023, Aerospace 10(3):308, eq 5,逐字)

            e_t = V·(t_d − t) − R          **e_t > 0 ⇒ 太早到**(剩下的時間
                                            足以飛比 R 更遠 ⇒ 要繞路)

        ⚠️⚠️ 刻意**不用**秒的形式 `(t_d − t) − t_go`:那需要估 t_go,而 t_go
           是本專案已經吃過兩次虧的量(時基污染吐出 17.8 億秒;靜態幾何式
           系統性低估 2 倍)。公尺形式的三個量 V、R、t_d **全部直接可測**,
           結構上繞開整個 t_go 問題。

        ## 這條律**只能延後、不能提前**(同篇 eq 6,逐字)

            de_t/dt = −V(1 − cos σ) ≤ 0

        σ 是前置角(速度向量與視線的夾角)。cos σ ≤ 1 ⇒ 導數恆 ≤ 0 ⇒
        e_t 單調不增。⇒ **e_t ≤ 0(已經遲到)時任何橫向機動都救不回來**,
        所以下面直接回 0,不做無效的掙扎。
        ⇒ 也因此共同命中時刻 t_d 必須取 `max_j`(我們已經是),取平均會讓
           最慢的那架永遠追不上。

        ## ⚠️ 側向的選擇要**鎖存**

        文獻的 `sign(σ)` 在 σ = 0 有奇異點。σ 在零附近抖動會讓偏置左右翻面,
        而翻面的頻率會直接寫進航跡 —— 本專案已經有過一次同構的失效
        ([[stot-altitude-gate-chatter]]:閘門沒遲滯 ⇒ 翻轉次數↔末端誤差,
        UAV5 翻 20 次 → 35.6 km)。所以第一次越過死區時把方向鎖住。
        """
        if not self.itcg_on or self.dive_T is None:
            return 0.0, None
        t_rem = self.dive_T - now
        if t_rem <= 0.0:
            return 0.0, None
        # ⚠️⚠️ 這裡**不用**文獻的等速形式 `e_t = V·(t_d − t) − R`。
        #    RQC(2026-08-15)實測:那個形式在我們的剖面上恆為負 ⇒ 整條律
        #    一次都沒出力(8/8 架、itcg_accel 非零樣本 = 0),而契約顯示
        #    itcg_on=1、gain=1.0 全部 OK。**最糟的一種失效。**
        #    根因:文獻假設**等速**,我們的俯衝是重力加速。放行瞬間實測
        #    瞬時地速 142.7 m/s,而預測器算出的隱含平均速度是 **208 m/s** ——
        #    用瞬時速度乘剩餘時間必然構不到斜距。
        #    (同一個陷阱我在前一輪就標記過:「文獻是純阻力減速,我們是重力
        #     加速」,方向相反。標記了還是踩,所以現在把兩種形式都記下來。)
        e_naive = V * t_rem - R                  # 舊的等速形式,**只記錄不使用**
        self.itcg_e[i] = e_naive
        # ⭐ 改用秒形式,速度剖面由校準過的預測器承擔(它已經是 t_go 的唯一入口)。
        t_go = self._tgo_of(i, d)
        e_s = t_rem - t_go                       # 秒;> 0 = 太早到
        # 等效**路徑餘裕**(公尺):以剩餘航程的比例換算,避開瞬時速度。
        surplus = R * (e_s / max(t_go, 1e-3))
        self.itcg_s[i] = surplus
        # ══ ⭐⭐ 觀測性先於功能(B1,2026-08-19)════════════════════════════
        #
        # ⚠️⚠️ 這條通道**從來沒有承載過訊號**:RRD+RRE 的 dive 段 18,567 筆
        #    有值、**非零 0 筆**;更早的統計是 80,760 個樣本全零。
        #    「開關開著但那一段從來沒真的發生」是本專案的頭號失效類別
        #    (已經第五次)。所以下面把 σ、ε_T、要求值全部**在任何早退之前**
        #    算好並記錄 —— 早退本身就是要被觀測的事件,不是可以沉默的事。
        #
        # ⚠️ 這一段是**純計算**:不改任何指令、不寫任何鎖存。
        #    唯一的副作用是寫進 `itcg_dbg`(只給 CSV 用)。
        #
        # 前置角 σ:速度向量相對視線的夾角。cross > 0 = 速度偏向視線左側。
        # ⚠️ 原本這三行在兩個早退之後才算。往上移**不改任何輸出**
        #    (它們不依賴任何被早退跳過的量),換到的是「遲到/凍結時
        #    σ 與 ε_T 也看得到」—— 沒有這個就無法區分「律沒出力」是
        #    因為遲到、因為凍結、還是因為根本沒跑到這裡。
        cross = dn * ve - de * vn
        dot = dn * vn + de * ve
        sigma = math.atan2(cross, dot)
        sg = abs(sigma)
        N = self.n_lat
        # ── Jiang 2021 Eq.(29) 的 t_go(**只記錄不使用**)──────────────
        #
        #     tgo = (r/V)·(1 + N′·σ²)·(σ/sin σ)^{1/(N−1)}
        #
        # ⚠️⚠️ **不拿它取代現行的 t_go。** 它的優勢只在大前置角:原文
        #    行 1079 逐字說 σ 低於 40° 時三種估計法 "similar",而我們的
        #    工作點是 σ 中位 15.4°、32 架次裡 28 架 ≤ 23.8°。
        #    實測拿它餵我們 32 架次:|誤差| 中位從現行的 3.73 s **變差到
        #    4.28 s**。所以並列記錄一欄,讓資料說話,不換掉任何東西。
        #
        # ⚠️ N′ 的符號在原文**自相矛盾**:Eq.(28) 印 `(2−N)/(6(N−1)(2N−1))`,
        #    Eq.(39) 印 `(N−2)/…`。這裡照本輪指派用 **(2−N)**(見常數
        #    `MPNG_NPRIME_2_MINUS_N`),但必須記下反面證據:
        #      * 把 Eq.(29) 展到二階要能退回 Eq.(30) 的 1/(2(2N−1)),
        #        1/(6(N−1)) + N′ 必須等於 1/(2(2N−1)) —— 只有 (N−2) 成立;
        #      * Lyapunov 那一段(Eq.52-53)宣稱 K₃ 與 σ 同號,也要 N′ > 0。
        #    ⇒ 很可能 Eq.(28) 才是 typo。**但這一欄只記錄不致動**,而且在
        #    我們的工作點 σ=15.3° 兩式只差 0.24% 的 tgo(r/V≈40 s ⇒ 0.09 s),
        #    改哪一邊都不會動到本輪結論。要翻案就改那個常數,一行。
        try:
            n_prime = ((2.0 - N) if MPNG_NPRIME_2_MINUS_N else (N - 2.0)) / (
                6.0 * (N - 1.0) * (2.0 * N - 1.0))
            ratio = 1.0 if sg < 1e-6 else sg / math.sin(sg)
            tgo_j = ((R / max(V, 1e-3)) * (1.0 + n_prime * sg * sg)
                     * ratio ** (1.0 / (N - 1.0)))
        except (ValueError, ZeroDivisionError, OverflowError):
            # ⚠️ 記錄層絕不可以因為算不出來就讓導引律整個爆掉(σ→180° 時
            #    sin σ→0 會炸)。算不出來就是**沒有值**,寫空字串不寫 0。
            tgo_j = None
        # ── 本拍**要求**的橫向偏置(夾制前、凍結前、遲到早退前)──────
        # ⚠️⚠️ req/cmd 雙欄紀律:判定量是 req(每拍都寫),不是 cmd。
        #    cmd 是 hold-last-value 的,拿它當「機制生效」的證據會給假綠燈 ——
        #    S2 那一輪的 `cmd_radius_m` 在開/關時同樣是 30,就是這個坑。
        side_ins = self.itcg_side.get(i)
        if side_ins is None:
            side_ins = 1.0 if sigma > 0.0 else -1.0
        if self.mpng_on:
            # ══ Jiang 2021 Eq.(41) 的時間項(a_ε + a_s)════════════════
            #
            #   a_M = N·V·λ̇  +  K_ε·N·V²·sin(σ)·ε_T  +  K_s·N·V²·sign(σ)·ε_T
            #         └ a_PNG ┘   └────── a_ε ──────┘   └────── a_s ──────┘
            #
            # ⭐ 我們的掛點**結構上已經就是 Eq.(41)**:`_pn_cmd` 的
            #    `a_h = n_lat·vh·λ̇` 就是 a_PNG,`a_h += a_t` 就是那個加號。
            #    所以這個函式只需要回傳 a_ε + a_s,呼叫端一行都不用改。
            #
            # ⭐ ε_T ≡ t̄go − tgo(Eq.49)。**ε_T > 0 = 還有多的時間 = 會太早到**,
            #    與我們的 `e_s = t_rem − t_go` 是**同一個量**(逐字同義)⇒ 直接用。
            #    方向由 Eq.(53) 的 Lyapunov 反推確認:V̇₁ = ε_T·ε̇_T ≤ 0 只有在
            #    ε_T = t̄go − tgo 時才成立,反過來會多出一個 +2 的殘項。
            #
            # ⭐ 為什麼是**橫向**不是縱向:縱向通道管俯角(能量與命中精度),
            #    時間只能靠加長水平航程去吸收。Sheng Yi 2021 與 Jiang 2021
            #    兩篇獨立同結論。混進縱向會直接動到已達標的命中誤差,而
            #    「精度不在授權範圍內」是使用者的硬性要求。
            #
            # ⭐ a_s(sign 項)存在的**唯一**理由是解 σ₀ = 0 的啟動問題:
            #    σ=0 時 λ̇=0 ⇒ a_PNG=0,sin σ=0 ⇒ a_ε=0,整條律會卡死。
            #    Eq.(42) 把 sign(0) 定義成 **+1**(不是 0)就是為了讓
            #    a_s = K_s·N·V²·ε_T ≠ 0 把 σ 推離零(原文行 843-844:
            #    "effective even if the initial leading angle is zero ...
            #     has no singularity")。
            # ⚠️⚠️ 代價:a_s 的大小**與 σ 無關**且在 σ 過零時整項跳 2 倍幅值。
            #    我們俯衝段 σ 中位只有 0.2°(32 架次實測)⇒ 幾乎整段貼在
            #    零附近,正是它抖振最兇的區。所以下面**不用**瞬時 sign(σ),
            #    改用既有的鎖存 `side` —— 把不連續換成一次性選擇。
            #    ([[stot-altitude-gate-chatter]]:閘門沒遲滯 ⇒ 翻轉次數直接
            #     寫進末端誤差,UAV5 翻 20 次 → 35.6 km。同構的失效。)
            k_eps, k_s = self.mpng_k_eps, self.mpng_k_s
            if k_eps <= 0.0 or k_s <= 0.0:
                # 論文建議值(行 1149-1153,N=3):K_ε = K_s = 40/(r₀·tgo₀)。
                # 量綱 1/(m·s) ✓。用**放行當拍**的 R、t_go 鎖存 —— 不鎖的話
                # 增益會隨距離一路飄,而 R 在俯衝段從 6854 m 掉到 0。
                k0 = 40.0 / max(R * max(t_go, 1e-3), 1.0)
                _lat = getattr(self, "mpng_k0", None)
                if _lat is not None:
                    k0 = _lat.setdefault(i, k0)
                if k_eps <= 0.0:
                    k_eps = k0
                if k_s <= 0.0:
                    k_s = k0
            # ⚠️ sin(|σ|) 而不是 sin(σ):正負號整個由鎖存的 side 承擔,
            #    否則 σ 翻號時 a_ε 與 a_s 會互相抵銷成零。
            a_mag = (k_eps * math.sin(sg) + k_s) * N * V * V * e_s
        else:
            # ⭐⭐ 現行路徑(2026-08-15 起):偏置大小不是拍出來的,
            #    **直接反解「要維持多大前置角」**。
            #
            #   路徑加長比例  1/cos σ = 1 + surplus/R      ⇒ σ_need
            #   PN 對前置角的回復加速度  a_PN = N·vh·λ̇ = N·vh·V·sin σ / R
            #   ⇒ 要把 σ 穩住在 σ_need,偏置必須正好抵銷這一項。
            #
            # ⚠️ 這樣 itcg_gain 就變成**純修整係數**(1.0 = 正好抵銷),而不是
            #    一個要靠試飛去湊的量綱不明的數字。先前寫成 K·V²/R·(e/R) 是我拍的,
            #    量級對不上:實算在 0.4g 上限下只買得到 0.6 s,遠低於需要的量。
            # ⚠️ 這五行與 2026-08-15 版**逐字相同**(只是縮排 + 換了變數名
            #    a_t → a_mag),`mpng_on=False` 必須逐位元同舊行為。
            frac = surplus / max(R, 1.0)
            cos_need = 1.0 / (1.0 + max(frac, 0.0))
            sig_need = math.acos(max(-1.0, min(1.0, cos_need)))
            a_mag = self.itcg_gain * self.n_lat * vh * V * math.sin(sig_need) / max(R, 1.0)
        cap = self.itcg_a_max_g * 9.80665
        a_req = a_mag * side_ins
        dbg = {"req_a": a_req, "eps_t": e_s, "tgo_ref": t_go,
               "tgo_jiang": tgo_j}

        def _emit(a_cmd, sat):
            """把本拍儀表寫進 `itcg_dbg`。⚠️ 只寫紀錄,**不改任何指令**。

            ⚠️ 用 getattr:樁節點跳過 __init__,沒有這個 dict。記錄層絕不可以
               因為缺屬性而讓導引律爆掉(與 `_row` 同一套護欄)。
            """
            dbg["cmd_a"] = a_cmd
            dbg["sat"] = sat
            _d = getattr(self, "itcg_dbg", None)
            if _d is not None:
                _d[i] = dbg
            return a_cmd
        # ══ 以下是致動路徑。三道早退全部保留,而且每一道都有理由 ══════════
        # ⚠️ 遲到(e_s ≤ 0)時不出力:這條律數學上只能加長航程。
        #    ⭐ MPNG 也適用:Eq.(41) 形式上對稱(ε_T<0 給反向加速度),但物理上
        #      切不進直線以內 —— 已經是最短路徑了。留著這道早退,否則會出現
        #      「為了趕時間而在直線航跡上加橫向指令」的純浪費。
        if e_s <= 0.0:
            return _emit(0.0, 0), e_naive
        # ⚠️ 末端凍結:沿用 PN 既有的凍結條件,不另立一套門檻 ——
        #    兩套門檻會在中間那段互相打架,而且沒有任何一層會報告。
        #    ⭐ MPNG 更需要它:原文行 735 逐字說 ε_T = 0 時「the law proposed in
        #      this paper is equivalent to the PNG law」—— 但那是**漸近**的,
        #      不保證在 fz_r 之前就歸零。這道是命中精度的保險絲。
        if d < self.fz_r:
            return _emit(0.0, 0), e_naive
        side = self.itcg_side.get(i)
        if side is None:
            # 死區內不鎖 —— 等它明確偏向一邊再定,避免把雜訊鎖成方向。
            if abs(sigma) < math.radians(2.0):
                return _emit(0.0, 0), e_naive
            side = 1.0 if sigma > 0.0 else -1.0
            self.itcg_side[i] = side
            self.get_logger().info(
                f"ITCG UAV{i}:繞路方向鎖定 {'左' if side > 0 else '右'}"
                f"(σ={math.degrees(sigma):+.1f}°、路徑餘裕 {surplus:+.0f} m、"
                f"律={'MPNG' if self.mpng_on else 'surplus'})")
        # ⚠️⚠️ 夾制是**唯一**的硬上界,也是這一輪真正的自變數。
        #    實測幾何重放(n=32,RRD1-8 + RRE1-8)給出的權限上界:
        #      itcg_a_max_g = 0.4(RRE 契約快照值)⇒ **+1.45 s**
        #      itcg_a_max_g = 1.0(契約上限)      ⇒ **+10.59 s**
        #    而 n=8 同時到達的需求是 **7.3–9.0 s**。⇒ 0.4 差了一個數量級,
        #    契約上限 1.0 才勉強夠 —— 換句話說**這條律能不能用,不取決於
        #    K_ε/K_s,取決於敢不敢把夾制開到接近契約上限**。
        # ⚠️ 契約區間是 0.1–1.0(contract_table.py)。要跑 2.0 得先由使用者
        #    裁決改契約,不可以在這一輪偷渡 —— contract_verify 會整批中止。
        a_cmd = max(-cap, min(cap, a_mag * side))
        # 飽和旗標:1 = 這一拍被夾制吃掉了要求值的一部分。
        # ⚠️ 這一欄是判斷「權限夠不夠」的直接證據。整段都是 1 ⇒ 律退化成
        #    bang-bang ⇒ 增益怎麼調都沒用,只能動夾制。
        sat = 1 if abs(a_mag) > cap else 0
        return _emit(a_cmd, sat), e_naive

    def _pn_cmd(self, i, now):
        """終端律 → (roll rad, pitch rad, thrust)。三通道解耦。"""
        GRAV = 9.80665
        dn, de, d, h, R, vn, ve, vh, hdot, V = self._geom(i)
        gam = math.atan2(hdot, max(vh, 1e-3))

        # ── 橫向:真 PN。λ̇ 由速度向量解析求得(零微分雜訊)──
        lam_dot = (de * vn - dn * ve) / (d * d)
        a_h = self.n_lat * vh * lam_dot
        # ⭐⭐ ITCG:時間項疊在**橫向**通道上。
        # ⚠️ 為什麼是橫向不是縱向:縱向通道管的是俯角(能量與命中精度),
        #    時間要靠加長水平航程去吸收。兩篇獨立文獻同一結論
        #    (Sheng Yi 2021 J.Supercomputing 77:212-243;Jiang 2021 Aerospace 8(8):231)
        #    —— 縱向管角度、側向管時間。混進縱向會直接動到已達標的命中誤差。
        a_t, _e_t_m = self._itcg_bias(i, now, dn, de, d, R, vn, ve, vh, V)
        self.itcg_a[i] = a_t
        a_h += a_t
        # ⚠️ 分母是 g·cosγ 不是 g:維持俯角 γ 需要 n·cosφ = cosγ,
        #    用 atan(a/g) 在 31° 俯衝會少要約 14% 的坡度。
        roll = math.atan2(a_h, GRAV * max(math.cos(gam), 0.25))

        # ── 縱向:同一條律在垂直面,用「角速率」形式(角度形式會要出 -90°)──
        sig_dot = (hdot * d - h * (-(dn * vn + de * ve) / d)) / (R * R)
        gdot = -self.n_vert * sig_dot
        # 法向過載夾制 n_z = cosγ + V·γ̇/g ∈ [nz_min, 3.0]
        # ⚠️ nz 下限 0.05 等於「不准做負 g 推頭」,它直接把最小推頭曲率半徑鎖成
        #    R_min = V²/((cosγ − nz_min)·g)。實算從平飛 70 m/s 推到 −45° 要 8.4 s、
        #    吃掉 659 m 水平;推到 −80° 要 49.5 s、需要 9259 m 高度 —— 而我們只有
        #    3148 m。也就是說**俯角上限是被這個下限決定的,不是被機體決定的**。
        #    放到負值(小負 g)才有陡俯角的權限。
        nz = math.cos(gam) + V * gdot / GRAV
        nz = max(self.pn_nz_min, min(3.0, nz))
        gdot = (nz - math.cos(gam)) * GRAV / V
        gam_cmd = gam + gdot * self.pn_lead

        # ⭐⭐ 幾何前饋:指令俯角永遠不准比「直指紅心的視線」更淺。
        #
        # ⚠️⚠️ 這條是「把 DIVE_ANGLE 改成 45 卻得不到 45° 俯衝」的真因。純 PN 的
        #    縱向通道(上面那幾行)**完全沒有引用 self.dive_angle** —— 它只對視線角
        #    速率積分,終端航跡角是積出來的副產品,不是被要求的量。檔頭 73-76 行
        #    記錄的「硬綁 45° 結果掉不到底、從紅心上空 244–720 m 飛過」就是這個。
        #
        # 取 min(更負 = 更陡):PN 想更陡就聽 PN 的(它有前置量),PN 想比視線淺就
        # 否決 —— 比視線淺在幾何上就是「會從目標上方飛過去」,沒有任何情況該這樣。
        # 附帶效果正是「俯角隨距離與速度改變」:視線俯角 atan(h/d) 隨接近自然變陡,
        # 遠處淺(追得上、拉得住)、終端陡(殘餘高度不再被 1/tanγ 放大成誤差)。
        if self.pn_los_floor > 0.0:
            gam_los = -math.atan2(h, max(d, 1e-3))
            gam_cmd = min(gam_cmd, gam_los * self.pn_los_floor)

        # 攻角現場估計(低通),把 γ 指令換成姿態指令
        th_meas = self.att.get(i, (0.0, 0.0, 0.0))[1]
        a_raw = th_meas - gam
        a_prev = self.alpha.get(i, a_raw)
        # ⚠️ 舊式是 `min(1.0, self.pn_dt / 0.7)`,而 pn_dt 來自宣告的 50 Hz、
        #    實際呼叫率卻是 20 Hz ⇒ 宣稱 τ=0.7 s、實際 τ=1.75 s。
        #    新式用**實際**週期與明確的 τ,預設 1.75 讓數值逐位元不變。
        k = min(1.0, self.tick_dt / self.pn_alpha_tau)
        self.alpha[i] = a_prev + k * (a_raw - a_prev)
        pitch = gam_cmd + self.alpha[i] / max(math.cos(roll), 0.5)
        # ⚠️ 這裡的下限以前寫死 -80,**不吃 dive_pitch_min** —— 兩者剛好同值,所以
        #    改參數看不出任何差別,是本專案的招牌靜默失敗。改成引用參數。
        pitch = max(math.radians(self.dive_pitch_min if self.dive_pitch_min < 0.0
                                 else -80.0),
                    min(math.radians(self.dive_pitch_max), pitch))

        # 韌體的耦合夾制(roll 上限隨 pitch 收斂),自己先夾好才能記錄飽和
        r_lim = math.radians(80.0) * abs(math.cos(th_meas))
        roll = max(-r_lim, min(r_lim, roll))

        # ── 油門:唯一的命中時刻通道,與兩個轉向通道解耦 ──
        # ⚠️ 時間窗過了就不要硬夾 0.4 s:那會讓 v_req = R/0.4 直接飽和成全速,
        #    終端一路衝過頭。
        #
        # ⚠️⚠️ 但「過期後退化成維持現速」是錯的,而且是俯衝速度上不去的**主因**。
        #    逾期 ⇒ t_rem = R/V ⇒ v_req = R/(R/V) = V ⇒ thr = trim + gain·0 = 固定
        #    trim(0.36)。也就是說飛機一落後於時程,油門就被鎖在 36% 滑行 ——
        #    而拉高 dive_pace 必然讓全隊逾期,所以「催速度」反而變成「熄火」。
        #    實測 N2(10°/pace 90):六機 brake 全 0.0 s(從沒被要求慢過),觸地
        #    地速 69.9–78.4;而 0.36 油門在 10° 下的平衡速度算出來是 73.4 m/s ——
        #    完全吻合。那不是機體極限,是一個常數。
        #    (同一台機全油門俯衝的平衡速度是 112 m/s。)
        #    ⇒ 逾期代表「已經來不及了」,唯一合理的動作是全速趕,不是維持現速。
        # ⚠️ dive_T 是**絕對牆鐘時戳**,而且在共識鎖定前是 None。減法一律放在
        #    守衛之後(同 _miss_check 的 (c) 條)。PN 只在 DIVE 段跑、理論上
        #    踩不到 None,但「理論上踩不到」的 TypeError 會打斷整個 tick,
        #    而 tick 被打斷是靜默的 —— 這種便宜的守衛不要省。
        #    真的踩到時當成**已逾期**:不知道還剩多少時間就全速趕,與下面
        #    「逾期唯一合理的動作是全速趕」同一個立場。
        t_left = 0.0 if self.dive_T is None else self.dive_T - now
        late = t_left <= 1.0
        t_rem = t_left if not late else max(1.0, R / max(V, 1e-3))
        # ⚠️ 上限不要放到 135:終端速度越快,觸地誤差越大。機制是「殘餘高度被
        #    1/tan(俯角) 放大成縱向落點誤差」—— 實測 R2 的 UAV5 以 95.5 m/s 進場,
        #    橫向最近通過距離只有 0.49 m(導引其實很準),卻在紅心正上方約 2 m
        #    掠過、再往前 5.35 m 才觸地;同一趟其餘五機 51.8-77.3 m/s,觸地誤差
        #    只有 1.41-2.12 m。散布現在有很大餘裕(0.33-0.80 s vs 門檻 3.0),
        #    拿一點去換誤差是划算的。
        v_req = max(45.0, min(self.pn_vmax, R / t_rem))
        if late and self.pn_late_full:
            v_req = self.pn_vmax          # 逾期:全速趕,別退化成維持現速
        thr = max(0.0, min(1.0,
                           self.pn_thr_trim + self.pn_thr_gain * (v_req - V)))
        # ⭐ 高度通道協同開啟後,同時性由**各機不同的 perch 高度**決定,
        #    油門不再需要拿來調時間 ⇒ 鎖全油門把終端地速拉到最大。
        #    ⚠️ 只有 perch_step > 0 時才該開:否則同時性會失去唯一的執行器。
        #    thrust 是**直接的百分比**(forced_throttle = thrust×100),再被
        #    servos.cpp 夾到 [THR_MIN, THR_MAX] ⇒ 送 1.0 就是 THR_MAX(21%)。
        if self.dive_full_thr:
            thr = 1.0
        return roll, pitch, thr, R, V

    def _pn_step(self, i, now):
        """對單機跑一次終端律並發送。回傳 True 表示已在終端段。"""
        if i not in self.pos or i not in self.vel:
            return False
        roll, pitch, thr, R, V = self._pn_cmd(i, now)
        t_go = R / max(V, 1e-3)
        # 終端凍結:t_go 或距離夠小就鎖住指令,但「仍要持續發送」——
        # 停發 3 s 會觸發 GUIDED_TIMEOUT,控制權默默回到 L1。
        # ⚠️ **只凍結 roll/pitch,不要凍結油門。** 凍結的理由是 PN 的視線角速率
        #    λ̇ = (de·vn − dn·ve)/d² 在 d→0 時分母爆掉,那是**轉向**通道的奇異點;
        #    油門通道沒有這個問題。連油門一起凍住的結果是:最後 fz_r 公尺用「凍結
        #    當下剛好是多少」的油門飛完 —— 如果凍結瞬間正在收油門,就等於滑行進
        #    紅心。而驗收第一優先量的正是**觸地**速度。
        if t_go <= self.fz_tgo or R <= self.fz_r:
            if i not in self.frozen:
                self.frozen[i] = (roll, pitch)
            roll, pitch = self.frozen[i]
        self._att_target(i, roll, pitch, thr)
        return True

    #: 共同命中時刻距現在的合理上界(s)。⚠️ 這不是調校參數,是**荒謬值攔截線**:
    #: 全程航程本身只有十幾分鐘,任何超過這個數的時刻表都不可能是對的。
    #: 訂在 3600 而不是貼著 600,是為了讓它只攔真正的時基錯誤,不去干涉排程。
    DIVE_T_HORIZON_MAX_S = 3600.0

    def _set_dive_T(self, now, horizon_s, why):
        """設定共同命中時刻,附荒謬值攔截。

        ⚠️⚠️ RPY3 的 `dive_T` 被寫成 56 年後,而 log 照樣印出
           「★ 首架抵環 → 重排共同命中時刻:T+1786674677.0s(舊表已過期 0s)」——
           數字就在畫面上,格式看起來完全正常,沒有任何一層攔它。
           時刻表是**放行、速度、miss 出口**三條路的共同上游,它壞掉的時候
           三條路一起靜默失效。所以攔截放在寫入點,不放在讀取點。
        """
        if not (0.0 < horizon_s < self.DIVE_T_HORIZON_MAX_S):
            self.get_logger().error(
                f"⚠️⚠️ 拒收共同命中時刻({why}):T+{horizon_s:.1f}s 超出合理上界 "
                f"{self.DIVE_T_HORIZON_MAX_S:.0f}s ⇒ **時基污染**。"
                f"沿用舊表 {'(尚未鎖定)' if self.dive_T is None else f'T+{self.dive_T - now:.1f}s'}。"
                f"檢查 _tgo_of 的 tgo_cache 是否被兩種時鐘混寫。")
            return False
        self.dive_T = now + horizon_s
        return True

    #: 量測閉合速率的窗長(s)。⚠️ 太短會抓到蛇形的單一相位,太長會跟不上
    #: PERCH 的內衝加速。20 s 約是 157 m/s 走 3 km,足以跨過幾個取樣的抖動。
    RING_CLOSE_WINDOW_S = 20.0
    #: 閉合速率下限(m/s)。量不到或幾乎沒在推進時用它,**刻意取得慢** ——
    #: 進環時間估晚只是大家多等一會兒(代價是終端速度),估早則直接毀掉同步。
    RING_CLOSE_MIN_MS = 10.0
    #: 進環時間的硬上限(s)。⚠️ 沒有它,「幾乎沒在推進」會讓 fallback
    #: (= transit + 幾何段)吐出上萬秒 —— 而 fallback 本身沒有上界,
    #: 等於把 600 s 的合理性界整條繞過去。
    RING_TRANSIT_MAX_S = 300.0

    def _ring_transit_s(self, i, d):
        """本機**還要多久才進得了 pushover 環**(s);已在環內回 0。

        ⭐⭐ 存在的理由(RPZ1,2026-08-15):
           放行有兩個條件串在一起 —— 先是**空間**閘門(必須飛進環),
           才輪到**時間**時刻表。而 `dive_T = max_j(t_go_j) + 2` 裡的 t_go
           算的是「直飛紅心要多久」,**完全沒有把進環時間算進去**。
           後果:RPZ1 的 UAV2 在環外 851 m,時刻表早就輪到它卻不能放行,
           足足等了 37 s;UAV1 已經先走 ⇒ 放行時刻差 +37.2 s ⇒ 散布 36.2 s。
           而同一趟的俯衝耗時差只有 −1.0 s —— 下游已經修好了,問題整個
           搬到上游的放行段。

        ⚠️⚠️ 閉合速率一定要用**量測窗口**,不能用瞬時徑向速率:
           RPZ1-UAV2 放行前的瞬時徑向速率是 −91 m/s(看起來 9 秒就到),
           但它在蛇形,實際 37 s 只推進 851 m(**23 m/s**)。
           用瞬時值會把進環時間低估 4 倍,等於這條修法白做。

        ⚠️ 估晚 vs 估早**不對稱**:估晚只是全隊多等一會兒(代價是終端速度),
           估早會讓 dive_T 太早、受限的那一架照樣遲到,散布原樣保留。
           所以下限 `RING_CLOSE_MIN_MS` 取得保守。

        文獻依據(Bin et al.,*Impact time control guidance against maneuvering
        targets based on a nonlinear virtual relative model*,式 49):
            T_d > r_go|_{C_M=0, r=r_0} / V_R
        可行命中時刻的下界由「**當下狀態可飛的最短幾何航程**」決定,而不是
        直線距離。有空間閘門時,那個最短航程 = 到環的航程 + 環內俯衝航程。
        """
        if not self.tgo_ring_transit or not self.dive_ring_gate:
            return 0.0
        try:
            gate = self.ring_of(i) * self.ring_gate_frac
        except Exception:                                        # noqa: BLE001
            return 0.0
        if d <= gate:
            return 0.0
        mono = time.monotonic()
        hist = self.d_hist.setdefault(i, [])
        hist.append((mono, d))
        cut = mono - self.RING_CLOSE_WINDOW_S
        # ⚠️⚠️ 條件看的是 `hist[1]` 不是 `hist[0]` —— 要**留住一筆跨過窗緣的樣本**,
        #    否則有效窗長會塌成「最後兩筆之間的間隔」。
        #    第一版寫成 `hist[0][0] < cut` ⇒ 窗緣那一筆被刪掉 ⇒ 只剩最近 1 s ⇒
        #    閉合速率量成 57 m/s(瞬時值)而不是 23 m/s(窗口值),
        #    進環時間低估 2.5 倍 —— 正是這條修法要防的那個錯誤本身。
        #    測試 test_uses_the_whole_window_not_the_last_step 抓到的。
        while len(hist) > 2 and hist[1][0] < cut:
            hist.pop(0)
        closing = self.RING_CLOSE_MIN_MS
        if len(hist) >= 2:
            span = mono - hist[0][0]
            if span >= 1.0:
                # 正 = 正在逼近。⚠️ 方向:距離**變小**才算有在推進。
                closing = (hist[0][1] - d) / span
        closing = max(self.RING_CLOSE_MIN_MS, closing)
        t = (d - gate) / closing
        # ⚠️⚠️ 進環時間本身要有硬上限。沒有的話,「幾乎沒在推進」會直接吐出
        #    上萬秒 —— 而那個值會同時流進 `dive_T` 與放行比較式,把 600 s 的
        #    合理性界從 fallback 那條路繞過去(fallback = transit + …,
        #    transit 大到多少它就大到多少,而 fallback 是**沒有上界的**)。
        #    300 s 在 157 m/s 下是 47 km,比整場交戰還長 ⇒ 只攔真正的病態值。
        #    夾住之後由 `_set_dive_T` 的荒謬值攔截去處理排不出時刻表的情況。
        if t > self.RING_TRANSIT_MAX_S:
            if not self._ring_transit_warned:
                self._ring_transit_warned = True
                self.get_logger().error(
                    f"⚠️⚠️ UAV{i} 進環時間 {t:.0f}s 超過上限 "
                    f"{self.RING_TRANSIT_MAX_S:.0f}s(距紅心 {d:.0f}m、"
                    f"閘門 {gate:.0f}m、量到的閉合速率 {closing:.1f} m/s)"
                    f"⇒ 夾住。這通常代表這架**幾乎沒有在朝紅心推進**,"
                    f"時刻表對它是排不出來的。")
            t = self.RING_TRANSIT_MAX_S
        return t

    def _tgo_of(self, i, d):
        """本機到紅心的**剩餘時間**(s)。⭐ T1:所有 t_go 的唯一入口。

        ⭐ 包含**進環時間**(見 `_ring_transit_s`)。因為 T1 已經把 t_go 收成
        單一入口,這一項自動同時作用在 `dive_T = max_j(t_go_j)+2` 與逐機的
        放行比較式 `t_go_i >= dive_T − now` 上 —— 兩邊不可能不一致。
        （這正是當初把六個 `d/_pace_of` 收成一個函式換來的。）

        ⚠️⚠️⚠️ 這裡**故意不收 `now` 參數**。RPY3(2026-08-14)的教訓:
           舊簽章是 `_tgo_of(self, i, d, now=None)`,九個呼叫點裡兩個傳
           `time.time()`(牆鐘)、七個讓它預設成 `time.monotonic()`(開機時鐘),
           而**兩者共用同一個 `tgo_cache`**。於是:
             ① `_observe_timing` 用牆鐘寫入 cache = (1.7867e9, t_go)
             ② 抵環重排用開機時鐘讀 cache:monotonic − epoch = **−17.8 億**
             ③ 新鮮度判斷 `−17.8億 < 0.5` ⇒ 誤判「快取還新鮮」
             ④ 外推 `cached[1] − (−17.8億)` = **17.8 億秒**
             ⑤ `dive_T = now + 17.8億 + 2` ⇒ 共同命中時刻排到 56 年後
           後果全部靜默:時刻表放行永不成立(放行退化成只剩幾何守門員 ⇒
           順序任意)、`v_req = R/(dive_T−now) ≈ 0` 被夾到 vmin ⇒ 俯衝段
           零時間權限、`_miss_check` 的逾時條也永不成立。
           實測代價:散布 78.5 s,且同組態的另一趟只有 12.1 s(重現性全失)。
           節流時鐘現在固定走 monotonic 且**不可由外部注入** —— 把時基做成
           結構上不可能弄錯,而不是靠呼叫端自律。

        ⚠️⚠️ 先前六個呼叫點各自寫 `d / self._pace_of(i, d)` —— 同一個概念散在
        六處,要換估計器就得改六個地方,漏一個就會出現「兩種 t_go 同時存在」
        而且完全靜默。集中成一個方法之後,估計器是可替換的。

        `tgo_from_predictor = False`(預設)⇒ 逐位元回到 `d / _pace_of(i, d)`。
        True ⇒ 用校準過的點質量積分器正向積分到觸地,回傳真正的 t_go。

        ⚠️ 積分很貴(一次約 500 步),用 `tgo_predict_dt` 節流;
           節流期間回傳上次的值**減去經過的時間**,而不是重算 —— 那樣在兩次
           積分之間仍然是單調遞減的,不會出現階梯。
        """
        # ⚠️⚠️ 進環時間與「環到紅心」必須**分段相加**,不能直接把整段 d 交給
        #    估計器再另外加 transit —— 那會把 gate 到現在位置那一段算兩次。
        transit = self._ring_transit_s(i, d)
        d_eff = d if transit <= 0.0 else min(d, self.ring_of(i) * self.ring_gate_frac)
        fallback = transit + d_eff / max(self._pace_of(i, d), 1e-3)
        if not self.tgo_from_predictor or _pred is None or self.T is None:
            return fallback
        mono = time.monotonic()          # ⚠️ 只此一處取時鐘,不接受外部注入
        cached = self.tgo_cache.get(i)
        if cached is not None:
            age = mono - cached[0]
            # ⚠️⚠️ `0.0 <= age` 這半邊是新加的護欄。**負的經過時間本身就是
            #    「時基被污染」的證據** —— 沒有這半邊,負 age 會同時
            #    (a) 讓新鮮度判斷通過、(b) 讓外推變成加法,兩個錯誤剛好互相
            #    掩護,於是吐出 17.8 億秒而且一路無聲。
            if 0.0 <= age < self.tgo_predict_dt:
                t = cached[1] - age      # 線性外推,保持單調遞減
                # ⚠️⚠️ 合理性上下界必須**兩條路徑都有**。舊版只有新鮮路徑
                #    (下面的 `0.0 < t < 600.0`)有,外推路徑沒有 ——
                #    護欄只裝在一條路上,結果就從沒裝的那條漏出去。
                if 0.0 < t < 600.0:
                    return t
            self.tgo_cache.pop(i, None)  # 快取不可信 ⇒ 丟掉,下面重算
        try:
            _dn, _de, dd, h, _R, _vn, _ve, _vh, _hd, V = self._geom(i)
            cap = (self.predict_gamma_cap if self.predict_gamma_cap > 0.0
                   else _pred.PN_GAMMA_CAP_DEG)
            zg_amsl = self.zg_amsl.get(i)
            # ⚠️ 環外時只讓預測器算「從環俯衝到紅心」那一段;它假設的是
            #    直接俯衝,而環外的機體並不會從那裡俯衝。
            dd = dd if transit <= 0.0 else min(dd, self.ring_of(i) * self.ring_gate_frac)
            r = _pred.predict_terminal(
                h, dd, V, thr_max_pct=self.predict_thr_max,
                home_amsl=(0.0 if zg_amsl is None else float(zg_amsl)),
                gamma_cap_deg=cap)
            # ⚠️ 用**水平掠過紅心**的時刻,不是觸地時刻:驗收量的是前者,
            #    而且觸地時刻在打過頭的情況下會拖很長。
            t = r.get("t_at_crossing")
            if t is None or not (0.0 < t < 600.0):
                return fallback           # 預測不可信就退回,但要留痕
            t += transit
            if not (0.0 < t < 600.0):     # 加了 transit 之後也要重驗上界
                return fallback
            self.tgo_cache[i] = (mono, t)
            return t
        except Exception as e:                                   # noqa: BLE001
            if not self._tgo_warned:
                self._tgo_warned = True
                self.get_logger().error(
                    f"⚠️ 積分 t_go 失敗,**退回靜態幾何配速**:{e!r}")
            return fallback

    def _weave_theta_of(self, i, d, now):
        """T3:由抵達時間誤差解析求出蛇形偏航角(度)。0 = 不蛇形。

        ⭐ 閉迴路結構:
            e_t = (t_c − now) − t_go_i          ← t_go 每拍由預測器重新積分
            cosθ = t_go / (t_go + e_t)          ← 路徑拉長 1/cosθ 倍
            θ    = acos(...)  夾限後下令
        下一拍 t_go 會反映新航跡 ⇒ e_t 自然收斂。**沒有比例增益要調。**

        ⚠️⚠️ 只對「太早到」作用。太晚到(e_t < 0)時蛇形幫不上忙 ——
           最短飛行時間是高度受限的,壓平航跡不會變快(實測 d=4000 與 d=8000
           的 t_min 完全相同)。那種情況必須靠協商把 t_c 訂在所有 t_min 之上,
           而不是讓這個迴路去追一個達不到的目標。

        ⚠️ 換向率是硬約束:ω = g·tanφ/V。把航向從 +θ 甩到 −θ 要 2θ/ω,
           而它必須塞得進半個週期,否則飛機還沒轉到就被要求反向,
           **實得 cosθ 遠大於指令值、吸收到的時間遠小於設計值**
           —— 那是「看起來有做,其實沒效果」,比不做更糟。
        """
        # ⚠️ 致動的開關在這裡,**觀測不受它影響** —— e_t 由 _observe_timing()
        #    獨立計算並記錄。第一版把兩者綁在一起,結果關著 T3 就完全量不到 e_t,
        #    而「先量再致動」正是這一步存在的理由。自己違反了自己的順序。
        if not self.weave_time_sync or self.dive_T is None:
            return 0.0
        tgo = self._tgo_of(i, d)
        e_t = (self.dive_T - now) - tgo
        self.e_t[i] = e_t
        if e_t <= self.weave_deadband_s or tgo <= 1.0:
            self.weave_theta[i] = 0.0
            return 0.0
        # 解析解:要把 t_go 拉長到 t_go + e_t
        c = tgo / (tgo + e_t)
        th = math.degrees(math.acos(max(0.05, min(1.0, c))))
        # 夾限①:使用者上限
        th = min(th, self.weave_theta_max)
        # 夾限②:換向率可行性 —— 2θ/ω 必須 ≤ 半週期的一半 ⇒ θ ≤ ω·P/8
        v = max(20.0, self._geom(i)[9])
        omega = math.degrees(GRAV * math.tan(math.radians(45.0)) / v)   # °/s
        th = min(th, omega * self.weave_period / 8.0)
        self.weave_theta[i] = th
        return th

    def _pace_of(self, i, d):
        """本機專屬的預估俯衝**地速**(放行判定用)。

        ⚠️ 全隊不能共用一個 pace。地速 = 總速 · cos(航路角),而各機在環上的高度
        本來就有差(實測 F5 進入高度 817–1039 m),角度不同 ⇒ 同樣的總速換算出
        來的地速就不同:F5 進入 1039 m 的機地速峰值只有 78.6,進入 887 m 的有
        93.8,但兩者總速都在 88–96 之間。用單一 pace 去反推放行時刻,這個差異
        會原封不動變成命中散布 —— 高速組態散布卡在 5.6–6.4 s 就是這麼來的。

        以設計俯角為基準做正規化:航路角剛好等於 dive_angle 時 pace 不變,
        比設計角陡的機 pace 較低(需要更久)⇒ 晚一點放行。這樣既保留既有的
        dive_pace 調校,又把各機的幾何差異補回去。"""
        h = max(0.0, self.pos[i][2] - self._zg(i))
        gam = math.atan2(h, max(d, 1.0))
        ref = math.radians(max(1.0, self.dive_angle))
        return max(10.0, self.dive_pace * math.cos(gam) / math.cos(ref))

    def _proximity_check(self, now):
        """S 型機動的接近監測。

        ⚠️ 為什麼需要:weave 的橫向擺幅 A = V·sinθ·P/4。週期從 22 s 拉到 120 s
           之後,擺幅從約 757 m 放大到約 2.1 km(2.8 倍)。P91 在舊設定下實測
           最近接近 348 m 水平 / 82 m 垂直 —— 那個裕度在新設定下夠不夠,**推論不出來**,
           只能量。防撞靠的是高度分層(layer_m),水平接近本身不是失敗,
           但「水平近 **且** 垂直也近」就是。這裡只觀測與示警,不介入導引。
        """
        if now - getattr(self, "_prox_t", 0.0) < 1.0:
            return                              # 1 Hz 就夠,別把 log 洗掉
        self._prox_t = now
        ids = sorted(self.pos)
        worst = None
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                i, j = ids[a], ids[b]
                h = dist_m(self.pos[i][:2], self.pos[j][:2])
                v = abs(self.pos[i][2] - self.pos[j][2])
                if worst is None or h < worst[0]:
                    worst = (h, v, i, j)
                if h < self.prox_h and v < self.prox_v:
                    self.get_logger().warn(
                        f"⚠️⚠️ 接近告警:UAV{i}–UAV{j} 水平 {h:.0f}m 垂直 {v:.0f}m "
                        f"(門檻 {self.prox_h:.0f}/{self.prox_v:.0f});"
                        f"weave 擺幅過大或高度分層 layer_m={self.layer:.0f}m 不足")
        if worst is not None:
            self._prox_min = min(getattr(self, "_prox_min", 1e9), worst[0])

    def _zg(self, i):
        """本機自量的地表高度(相對 HOME);沒有地形回報就退回全域 ground_alt。"""
        return self.zg.get(i, self.ground_alt)

    def _agl(self, i):
        """離紅心地表的高度(m)。**觸水終結與 CSV 的 agl 欄都走這裡。**

        ═══ 2026-08-07 事故:誤殺一架 150 m/s 巡航中的飛機 ═══
        原本一律用 `relative_alt − z_g`。這兩個量的**基準不同步**:

          * `relative_alt` 每一筆都跟著飛控當下的 HOME 走
          * `z_g` 在 `_pump_link` 的 `i not in self.zg` 保護下**只算一次就凍結**

        UAV2 在收到目標後 9 秒,HOME 的 GPS 海拔從 +32.0 m 跳到 −0.31 m
        (飛控重設 HOME / EKF 原點)。`relative_alt` 立刻低了 32 m,而 `z_g`
        還停在 −31.99 ⇒ `agl` 瞬間變 **−0.2 m** ⇒ 觸水判定成立 ⇒ 強制 disarm。
        飛機當時在桃園上空 200+ m、地速 150 m/s。
        `kill_confirm_n=2` 在 20 Hz 只有 0.1 秒,而基準漂移是**持續偏移**
        不是單筆跳點 —— 連續筆數對它一點防護力都沒有。
        節點自己 0.7 秒後才印出「紅心地表高度漂移」告警,已經太遲。

        修法:釘死 AMSL 的目標**在 AMSL 空間算**,

            agl = alt_amsl − target_amsl

        而 `alt_amsl` 與 `relative_alt` 取自**同一則** GLOBAL_POSITION_INT
        (`alt_amsl = relative_alt + home_amsl`,見 `_pump_link`)⇒
        HOME 一跳,兩者反向等量,agl **恆定不變**。這不是啟發式的否決條件,
        是把基準對齊,所以不存在「否決期過了又誤殺」的殘留風險。

        ⚠️ 基準穩定時兩式**逐位元相同**(z_g ≡ target_amsl − home_amsl),
           所以既有的海上航次資料不受影響。
        ⚠️ 沒釘 AMSL(陸上目標、走地形庫)的路徑**一個位元都沒動** ——
           那條路的 z_g 本來就是地形回報,和 HOME 基準無關。
        """
        if not math.isnan(self.target_amsl) and i in self.alt_amsl:
            return self.alt_amsl[i] - self.target_amsl
        return self.pos[i][2] - self._zg(i)

    def _goto(self, i, lat, lon, alt, v, move_tol=40.0, min_dt=2.0,
              keepalive=30.0, radius=None):
        """去某點 + 速度。⚠️ **航點必須鎖存,不能每 tick 重發。**

        這是 ALIGN/PERCH「爬不上去」的真因,和 _goto_latched 註解裡講的是同一個病,
        只是先前只在 DIVE 修掉:每一次 DO_REPOSITION 都會走 `Plane::set_guided_WP()`,
        裡面呼叫 `set_target_altitude_current()` —— 把高度目標**重新錨在飛機當下的
        高度**。之後 `Mode::update_target_altitude()` 才會把目標推回航點高度,但那是
        跟著 10 Hz 的導航迴圈跑的,而我們用 20 Hz 重發 ⇒ 重錨比修正還快,高度需求
        永遠累積不起來,而且每次下沉都被當成新目標吃進去(單向棘輪)。

        實測(2026-07-27):整場 ALIGN/PERCH 六機卡在 30-80 m(指令 400 m),
        NAV_CONTROLLER_OUTPUT 的 alt_error 六機有五機 ≈ 0 —— 飛控認為「已經到高度了」。
        `perch_alt_gate` 要求 0.9×400=360 m,永遠過不了 ⇒ 這才是 PERCH 死結的真因,
        commit 9bcd985 把等待速度 45→65 只是治到症狀。
        反證:把 tot_node 殺掉、不再重發之後,六機靠著「最後那一發」航點在 3 分鐘內
        自己從 37 m 爬到 335-375 m。

        所以只在「目標真的動了」時重發:位置移動 > move_tol 或高度差 > 5 m,
        且距上次至少 min_dt 秒;另加 keepalive 秒的補發以防丟包。
        DO_CHANGE_SPEED 不碰高度斜坡,可以照原本的節流繼續每 2 秒送。"""
        c = self.link[i]; now = time.time()
        if abs(v - self.last_v[i]) > 0.5 or (now - self.last_v_t[i]) > 2.0:
            c.mav.command_long_send(c.target_system or i, 1,
                                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                                    0, float(v), -1, 0, 0, 0, 0)
            self.last_v[i] = v; self.last_v_t[i] = now
        # ⚠️⚠️ 儀表要記在偵測器**之前**。RRD 的教訓:radius 不在 wp_pt 裡,
        #    所以純半徑變更 moved=False、指令根本沒送出去,而「送出去的半徑」
        #    在開關開與關時**同樣**是 30 ⇒ 拿它當生效證明會給假綠燈。
        #    決定性的量是「本拍**要求**的半徑」。
        _req = self.goto_radius if radius is None else radius
        self.req_radius[i] = _req
        prev = self.wp_pt.get(i)
        moved = (prev is None or abs(prev[2] - alt) > 5.0
                 or dist_m(prev[:2], (lat, lon)) > move_tol)
        dt = now - self.wp_t.get(i, 0.0)
        if (moved and dt >= min_dt) or dt > keepalive:
            self._reposition(i, lat, lon, alt, _req)
            self.sent_radius[i] = _req
            self.wp_pt[i] = (lat, lon, alt)
            self.wp_t[i] = now
            self.wp_key[i] = None      # 讓 DIVE 的鎖存鍵重新生效

    def _reposition(self, i, lat, lon, alt, radius=0.0):
        """param3 = 繞圈半徑。**傳 0 = 災難**,而且那個「180m」的舊註解已經過期。

        ⚠️⚠️ ArduPlane 的閘門是 GCS_MAVLink_Plane.cpp:599 `param3 > 0`;
           0 不通過 ⇒ mode_guided → update_loiter(0) ⇒ navigation.cpp:362-366
           退回 |WP_LOITER_RAD| ⇒ AP_L1_Control.cpp:166 再 ×eas2tas²。
           下面那句「實測 180m」是**舊機體、低空**時寫的;現在
           `WP_LOITER_RAD=1488`、3100 m 的 eas2tas²=1.365 ⇒ **實際 2030 m**。
           ⇒ ALIGN 整段繞在 2030 m 的圈上,抵達判定 `d < reach` 永遠不成立。
           **註解沒跟上參數變更,是這個 bug 藏了四趟的原因。**
        ⚠️ 同一個 bug 在 mavros_interface.py(編隊層)也發作過一次,
           修好後階段 2 的 worst 誤差 3664 → 570 m。

        ⇒ 呼叫端**必須**依用途給半徑,兩者語意完全相反:
             飛過去(ALIGN/climb/runup/穿越)⇒ goto_radius(小,~30)
             守在點上(hold_ring/wait)      ⇒ hold_radius(要 ≥ 物理迴轉半徑)
        
        飛機一接近就被捕獲進 180m 的繞圈 → 水平距永遠壓不下去(實測卡在 124-140m)。
        俯衝段要傳一個小半徑,才可能真的打到紅心中心。

        ⚠️⚠️ **param2 必須帶 MAV_DO_REPOSITION_FLAGS_CHANGE_MODE(=1)。**
           GCS_MAVLink_Plane.cpp:584-605:
               if ((param2 & CHANGE_MODE) || control_mode == mode_guided) { …照做… }
               return MAV_RESULT_FAILED;
           ⇒ param2=0 而飛機不在 GUIDED ⇒ **整發丟掉**,而 ACK 走這條沒人排空的
           連線 ⇒ 零告警。本節點又完全不追飛控模式(無 HEARTBEAT 處理),
           所以它永遠不會發現自己在對空氣下令。
        ⚠️ 這不是理論風險:互動路徑用 `MANUAL=1` + `swarm_takeoff.sh` 起飛,
           飛控**停在 TAKEOFF 模式**;編隊層本來會事後改宣告 GUIDED,但
           backup_node.py:263 的讓權 `return` 在 `_formation_control()` 之前
           ⇒ 一讓權 `_keep_guided()` 就不再被呼叫 ⇒ 沒人切模式。
           RPF1 實測:UAV2 整趟 1250 s 在桃園上空繞圈,距 slot 32 km 不縮短,
           但地速 148-164 m/s 正常(DO_CHANGE_SPEED 不吃這個閘門)。
        ⚠️ 副作用要知情:帶旗標後本節點的位置指令會**強制蓋過操作員的 RTL**。
           編隊層有 RTL/AUTO 白名單會停發,本節點沒有(它不讀模式)。
           打擊期間要保留操作員權限,必須另外加停手機制。"""
        c = self.link[i]
        c.mav.command_int_send(
            c.target_system or i, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0,
            float(mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE), float(radius),
            float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))

    def _goto_latched(self, i, lat, lon, alt, v, key):
        """俯衝專用:DO_REPOSITION 只在 key 改變時發一次,速度照常每 tick 更新。

        ⚠️ 這是「俯衝不下來」的真因:ArduPlane 的 GUIDED 高度是沿 prev_WP→next_WP
        線性斜坡(ALT_SLOPE_MIN 生效)。每 tick 重發 DO_REPOSITION 會把 prev_WP 重設
        成「現在位置＋現在高度」→ 斜坡每 0.33 s 重新從當前高度起算,下降需求永遠
        累積不起來,結果只掉 ~100 m 就飛過目標(實測命中高度 862-1060 m)。
        改成鎖存:整段俯衝只發一次航點,讓飛控自己把那條 45° 斜坡飛完。"""
        c = self.link[i]; now = time.time()
        if abs(v - self.last_v[i]) > 0.5 or (now - self.last_v_t[i]) > 2.0:
            c.mav.command_long_send(c.target_system or i, 1,
                                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                                    0, float(v), -1, 0, 0, 0, 0)
            self.last_v[i] = v; self.last_v_t[i] = now
        if self.wp_key.get(i) != key:
            self._reposition(i, lat, lon, alt, radius=self.dive_radius)
            self.wp_key[i] = key
            self.wp_t[i] = now

    def _alt(self, i):
        """每 slot 專屬高度層(對齊階段防撞):slot_id 對稱分佈於 talt ± (n-1)/2·layer。
        S 型機動時就算兩機水平路徑逼近,也差 ≥layer_m 垂直 → 不碰撞。"""
        return self.talt + self.layer * (self.slot_id[i] - (self.n - 1) / 2.0)

    def _dive_alt(self, i):
        """終端俯衝微高度分層(防撞):收斂同一水平點 T,但各機差 dive_layer 垂直。"""
        return self.dive_alt + self.dive_layer * (self.slot_id[i] - (self.n - 1) / 2.0)

    def _xtrack(self, i):
        """飛機偏離「自己那條紅心徑向線」的橫向距離(m)。
        徑向線 = 由 T 沿本機 slot 方位射出的直線;偏差 = 距T · |sin(方位差)|。
        這是「有沒有正對紅心」的量化指標:0 = 完全對正,俯衝下去就是直線命中。"""
        if i not in self.pos or i not in self.slot_id:
            return 0.0
        off = self._fan_off(self.slot_id[i])
        brg_radial = (self.approach + off) % 360.0
        d = dist_m(self.T, self.pos[i][:2])
        b = bearing(self.T, self.pos[i][:2])
        return abs(d * math.sin(math.radians(b - brg_radial)))

    def _line_alt(self, i, d_aim):
        """瞄準點在 dive_angle 下滑線上的高度。
        d_aim = 瞄準點與 T 的水平距(T 本身 = 0;穿越點在 T 後方 → 傳負值)。

        ⚠️ 關鍵:高度要算「瞄準點自己位置」的線上高度,不是「飛機當下位置」的。
        用飛機當下的 d 去算(alt = d·tan45 ≈ 現在高度)等於命令「飛到瞄準點時
        保持現在高度」= 幾乎平飛,飛機浮在線上方、到近端才被迫暴拉 → 正是
        「高空掠過紅心」的真因。瞄準點在線上時,ArduPlane GUIDED 的高度斜坡
        (prev_WP=當前位置 → next_WP)本身就是那條 45° 直線,落後時斜率自動變陡。"""
        lay = self.dive_layer * (self.slot_id[i] - (self.n - 1) / 2.0)
        # 下滑線終點錨在「本機自己量到的地表」,不是全隊共用的常數
        base = self._zg(i) + self.dive_alt_agl
        z_wp = base + lay + d_aim * math.tan(
            math.radians(self.dive_angle))            # 幾何下滑線上的目標高度
        z_ac = self.pos[i][2] if i in self.pos else None
        if z_ac is not None and self.track_frac < 0.999:
            # 飛機只做到指令斜率的 track_frac 倍 ⇒ 指令要「加倍陡」才會真的落到 z_wp。
            # 由當下高度反推:demanded_drop = (z_ac - z_wp)/track_frac,
            # 瞄準點高度 = z_ac - demanded_drop(通常會是負值 = 瞄到地面以下,正常)。
            z_wp = z_ac - (z_ac - z_wp) / self.track_frac
        # 相對高度剛好 0.0 會被 Location::sanitize() 判為「未設定」→ 指令被拒
        return z_wp if abs(z_wp) > 0.5 else -0.5

    def _assign_and_plan(self):
        """slot 指派(依橫向、航跡不交叉)+ 共同抵達時刻。"""
        ids = sorted(self.pos)
        def cross(i):
            b = bearing(self.T, self.pos[i][:2])
            return math.sin(math.radians(b - self.approach)) * dist_m(self.T, self.pos[i][:2])
        order = sorted(ids, key=cross)
        slot_order = sorted(range(self.n), key=lambda k:
            math.sin(math.radians(self._fan_off(k))))
        for i, k in zip(order, slot_order):
            self.slot_of[i] = self.slots[k]; self.slot_id[i] = k
        self.t0 = time.time()
        v_plan = self.plan_frac * self.vmax
        tmax = max(dist_m(self.pos[i][:2], self.slot_of[i]) / v_plan for i in ids)
        self.T_arr = self.t0 + tmax + self.margin
        self.get_logger().info(
            f"ALIGN：T_arr=+{self.T_arr - self.t0:.0f}s；各機距 slot "
            f"{ {i: round(dist_m(self.pos[i][:2], self.slot_of[i])) for i in ids} }")

    def _row(self, now, i, d, mode, tgt):
        # ⭐ S9:影子預測掛在記錄點,保證「有記錄就有預測」,兩者時間對齊。
        #    ⚠️ 掛在這裡而不是導引迴圈裡,是為了讓它**物理上不可能**影響指令。
        self._predict_tick(i, now)   # 影子模式;predict_hz=0 時整段跳過
        self._observe_timing(i, now)  # T2 觀測;與 T3 致動開關無關
        # ⚠️⚠️ 這兩欄的**基準不同**,混用會系統性偏掉一整個地形落差:
        #    alt = 相對 HOME(飛控 GLOBAL_POSITION_INT.relative_alt 原值)
        #    agl = 「離紅心地表多高」,由 `_agl()` 給 —— 釘死 AMSL 的目標走
        #          `alt_amsl − target_amsl`(對 HOME 重設免疫);沒釘死才退回
        #          `alt − z_g` 的地形庫路徑。
        #    SW1 的紅心地表 z_g ≈ −32 m,兩欄差 32 m —— 先前只有 alt 一欄,
        #    離線分析每次都要自己去 log 裡撈 z_g,撈錯就靜默偏 32 m。
        #
        # ⚠️ 2026-08-10 更正:這段原本寫「agl … 和 log 的『命中高度』同基準」。
        #    那句在 2026-08-07 把 `_agl` 改到 AMSL 空間之後就**不成立**了 ——
        #    當時 `hit_alt` 還停在 `pos − _zg` 的 HOME 相對基準,兩者只有在
        #    HOME 穩定時才逐位元相同。現在 `hit_alt` 也改成 `_agl(i)`,
        #    這句才重新成立;不要在只改其中一邊的情況下把這句留著。
        agl = self._agl(i)
        self._kill_arm_update()      # 觸水終結的解除保險門檻
        if not getattr(self, "_agl_datum_logged", False):
            self._agl_datum_logged = True
            self.get_logger().info(
                f"CSV 高度基準生效值:alt = 相對 HOME(m);"
                f"agl = alt − 紅心地表 z_g;各機 z_g = "
                f"{ {k: round(self._zg(k), 1) for k in sorted(self.pos)} } m"
                f"(未收到 TERRAIN_REPORT 者退回 ground_alt={self.ground_alt:+.1f};"
                f"target_amsl={self.target_amsl})")
        # ⭐ 追加欄。未啟用時寫空字串 —— 不是 0,因為 0 是**合法的預測值**,
        #    用 0 當「沒跑」會讓事後判讀把停用誤讀成「預測殘餘為零」。
        # ⚠️ 用 getattr:樁節點跳過 __init__,沒有 vel/pred。
        #    記錄層絕不可以因為缺屬性而讓整個 _row 爆掉 —— 那會連軌跡都寫不出來。
        _v = (getattr(self, "vel", None) or {}).get(i)
        _gam = ("" if _v is None
                else round(math.degrees(math.atan2(
                    -_v[2], max(math.hypot(_v[0], _v[1]), 1e-6))), 2))
        _p = (self.pred or {}).get(i) if hasattr(self, "pred") else None
        _et = (getattr(self, "e_t", None) or {}).get(i)
        _wv = (getattr(self, "weave_theta", None) or {}).get(i)
        _tc = (getattr(self, "tgo_cache", None) or {}).get(i)
        _em = (getattr(self, "itcg_e", None) or {}).get(i)
        _ia = (getattr(self, "itcg_a", None) or {}).get(i)
        _is = (getattr(self, "itcg_s", None) or {}).get(i)
        try:
            _la = self._lead_angle_deg(i)
        except Exception:                                    # noqa: BLE001
            _la = None                 # ⚠️ 記錄層絕不可以因為算不出來就整個爆掉
        # ⚠️ 用 getattr + `or {}`:樁節點跳過 __init__(與上面同一套護欄)。
        _dbg = (getattr(self, "itcg_dbg", None) or {}).get(i)
        _rr = (getattr(self, "req_radius", None) or {}).get(i)
        _sr = (getattr(self, "sent_radius", None) or {}).get(i)
        _wp = (getattr(self, "wp_pt", None) or {}).get(i)
        _tg = (getattr(self, "tgo_obs", None) or {}).get(i)
        if _tg is None and _tc is not None:
            _tg = _tc[1]
        self._cw.writerow([round(now - self.t0, 1), i, self.pos[i][0],
                           self.pos[i][1], self.pos[i][2], round(self.gs.get(i, 0), 1),
                           round(d), mode, round(self.last_v[i], 1),
                           tgt[0], tgt[1], self.phase, round(agl, 1),
                           _gam,
                           "" if not _p or _p["h_at_crossing"] is None
                           else round(_p["h_at_crossing"], 1),
                           "" if not _p else round(_p["gs_end"], 1),
                           "" if not _p else round(_p["t_go"], 1),
                           "" if not _p else round(_p["gamma_sat_frac"], 3),
                           # ⚠️ 未啟用時寫空字串不是 0 —— 0 是合法的誤差值。
                           "" if _tg is None else round(_tg, 2),
                           "" if _et is None else round(_et, 2),
                           "" if _wv is None else round(_wv, 1),
                           # ⚠️ 未啟用寫空字串不是 0 —— 0 是合法的誤差值,
                           #    用 0 當「沒跑」會讓事後判讀把停用讀成「剛好準時」。
                           "" if _em is None else round(_em, 1),
                           "" if _ia is None else round(_ia, 3),
                           "" if _is is None else round(_is, 1),
                           "" if _la is None else round(_la, 1),
                           # ⭐⭐ S2 儀表。未設值寫空字串不寫 0(0 是合法半徑)。
                           "" if _rr is None else round(_rr, 1),
                           "" if _sr is None else round(_sr, 1),
                           "" if _wp is None else _wp[0],
                           "" if _wp is None else _wp[1],
                           # ⭐⭐ B1 MPNG 儀表。⚠️ 未設值一律寫**空字串不寫 0**:
                           #    0 是合法的偏置/誤差值,用它當「沒跑」會讓事後
                           #    判讀把「整條律沒啟動」誤讀成「剛好不需要出力」。
                           #    這正是 RQC 那一批看了半天才發現的東西。
                           "" if _dbg is None else round(_dbg["req_a"], 4),
                           "" if _dbg is None else round(_dbg["cmd_a"], 4),
                           "" if _dbg is None else round(_dbg["eps_t"], 3),
                           "" if _dbg is None else round(_dbg["tgo_ref"], 2),
                           "" if _dbg is None or _dbg["tgo_jiang"] is None
                           else round(_dbg["tgo_jiang"], 2),
                           "" if _dbg is None else _dbg["sat"]])
        # ── 觸水終結。**故意**放在 _row 尾端,不是放在呼叫端 ────────────
        # ⚠️⚠️ KIL2(2026-08-06)的教訓:第一版把這一行放在 DIVE 迴圈的呼叫端,
        #    縮排看起來對、bash/py 都不會報錯,但那一層(和 `done =
        #    self._done_count()` 同層)其實在**逐機迴圈之外** ⇒ 每個 tick 只用
        #    迴圈殘留的 `i` 跑一次 ⇒ **只有最後一架被評估過**。
        #    實測後果:UAV1 有 72 筆連續 AGL≤0 卻從未被終結,照樣爬回 +109.5 m;
        #    只有 UAV2 出現終結訊息。log 沒有任何異常。
        # ⇒ 放在這裡有三個好處,都是不變量而不是慣例:
        #    ① 一定在 writerow 之後(計分器的觸地內插要 (AGL>0, AGL≤0) 這對相鄰
        #       取樣,少了後者整架會被判 n_touch=0);
        #    ② DIVE 迴圈裡有三個 _row 呼叫點(missed / 未放行 / 主路徑),前兩個
        #       後面就 `continue`,放呼叫端一定會漏;
        #    ③ ALIGN/PERCH 段撞海也一樣會被終結 —— 那本來就該終結。
        #    agl 已經在上面算好,不必再算一次,也不會和 CSV 那一欄不同步。
        self._kill_airframe(i, agl, now)

    # ══════════════════════════════════════════════════════════════════════
    #  miss 出口 + 結算量的時刻一致性(M1/M2/M3)
    # ══════════════════════════════════════════════════════════════════════

    def _metrics_frozen(self, i):
        """結算量是否已鎖存 —— **判定與量測共用這一個述詞**(唯一真實來源)。

        涵蓋 impact / missed 的互斥鎖,以及 dmin / hit_alt / truth_dmin 的凍結。

        ⚠️⚠️ **只能在鎖存之後凍結,鎖存之前一個位元都不能變** —— `_passed()`
           的判定 `d > dmin + 8` 完全靠 dmin 在鎖存前持續更新來偵測「已通過
           最近點」。提早凍結 = 永遠偵測不到通過 = 把逃逸鏈再造一次。

        ⚠️⚠️ 2026-08-04:量測曾經另有一個 `_dmin_frozen` 述詞 + 遲滯收斂窗
           (鎖存後 d 回升 1000 m 才停止收斂)。那是錯的,而且錯的方向剛好
           讓數字變好看:
             * 遲滯收窗實測發生在鎖存後 31.7–45.5 s,而 SW1 的機體在鎖存後
               1.9–2.6 s 就觸海了。SITL 不終結觸海的機體 —— U3 掉到海面下
               95.7 m、爬回 +331 m,24.1 s 後再穿一次紅心。窗收進去的是
               **第二趟**,不是「還沒走完的最近點」。
             * 後果:U3 32.6→22.6(剛好從 MISS_MAX=30 的失敗側跨到通過側)、
               U5 217.9→67.1(差 3.2 倍)。舊的鎖存值才是真值 —— 與現場
               trial_SW1.log 印的「最近 35 / 33 / 30 / 218 m」逐架吻合。
           而且那個述詞從頭到尾是死碼:全 repo 只有測試呼叫它,`_track_dmin`
           另寫了一份同語意的程式碼在跑,註解卻把讀者指向述詞 —— 一把假的
           迴歸鎖。現在只剩這一個述詞,兩邊都用它。
        """
        return i in self.impact or i in self.missed

    def _track_dmin(self, i, d, now):
        """更新最近水平距(遙測基準 + SITL 真值基準)。鎖存當下即凍結。

        ⚠️ `now` 只留作日後排錯用的呼叫端契約(每個 tick 的時戳);凍結條件
           不看時間 —— 「鎖存之後」本身就是時間條件,再疊一個秒數窗只會多一
           個可以調鬆的旋鈕。
        ⚠️ 這支**每個 tick、每一架**都會被呼叫(在所有 continue 之前)。
        ⚠️ 這裡回報的是首航次真最近點的**上界**:鎖存條件是 d < impact_reach,
           終端取樣步長實測 3–9 m ⇒ 誤差最多一個取樣步。要更細的量用
           stot_score.py(點到線段內插 + 觸地收窗),別在這裡加窗把量測拉長
           到機體已經入水之後。
        """
        if self._metrics_frozen(i):
            return
        if d < self.dmin.get(i, 1e9):
            self.dmin[i] = d
        if i in self.truth:                 # 同一件事的 SITL 真值基準
            td = dist_m(self.T, self.truth[i][:2])
            if td < self.truth_dmin.get(i, 1e9):
                self.truth_dmin[i] = td

    def _passed(self, i, d):
        """已 committed 且通過最近點(d 由最小值回升 8 m)⇒ 視為穿越。"""
        return bool(self.commit.get(i)) and d > self.dmin.get(i, 1e9) + 8.0

    def _clamp_miss_far(self, r_ring, d_max):
        """把逃逸門檻夾到「環半徑 / 當下最遠距」的 miss_far_k 倍以上。

        ⚠️⚠️ R1 blocker:這裡以前只有一行 error log —— 只會吵,不會擋。
           門檻設太緊的後果是 DIVE 第一個 tick 全隊同時判逃逸、整趟 0/6,
           而那一行 error(i) 不含「生效值」字樣 ⇒ 進不了 summary.md,
           (ii) 在 phase 翻 DIVE 的那個 tick 印,下一個 tick 就動手,
           人根本來不及介入。本專案的死敵就是這種「看得到卻擋不住」。

        ⚠️ 這**不是**放寬命中判定:miss_far 只決定「多遠算飛走了」,
           impact_reach / switch_r / passed 一個字都沒動。夾寬它只會讓
           脫靶機晚一點被判 MISS(仍有首航次窗兜底),不會把脫靶變命中。

        回傳 (old, new, clamped)。
        """
        old = self.miss_far
        need = self.miss_far_k * max(r_ring, d_max)
        if self.miss_far >= need:
            return old, old, False
        self.miss_far = need
        return old, need, True

    def _miss_check(self, i, d, now):
        """miss 出口(M1 首航次窗 + M2 逃逸/停滯)。回傳 True 表示這次剛鎖存。

        ⚠️ 只在 dive_go **之後**檢查 —— 之前蜂群還在 35 km 外的扇形上,
           d > miss_far_m 對每一架都成立,提前檢查會全軍誤判。
        ⚠️ impact 與 missed 互斥:已 impact 的直接跳出。
        """
        if self.dive_go is None or i not in self.pos or self._metrics_frozen(i):
            return False
        why = None
        # (a) 逃逸:離目標水平距曾經超過門檻。
        # ⚠️⚠️ R1 blocker:**只對已放行的機檢查**。逃逸門檻與 pushover 環半徑
        #    是耦合的 —— ring = (perch_alt − dive_alt)/tan(dive_angle),
        #    dive_angle 調淺(25° 是 6.4 km、5° 是 34 km)或 perch_alt 拉高
        #    (6000 m + 15° ⇒ 22.4 km)都會讓環半徑超過 miss_far_m。
        #    而進 DIVE 的前提只是「全員 d < ring_of(i)」,還沒放行的機**合法地**
        #    停在環上 ⇒ DIVE 的第一個 tick 就會把全隊六架同時判成逃逸、整趟
        #    瞬間結案(0/6),從總表上看跟「飛行失敗」一模一樣,而真因是組態。
        #    已放行的機是朝紅心飛的,d 只會變小,不存在同樣的誤判。
        #    (另一半保險是 dive_go 當下把 miss_far 夾住,見 _clamp_miss_far。)
        if self.released.get(i) and d > self.miss_far:
            why = (f"逃逸:離目標 {d/1000.0:.1f} km "
                   f"> miss_far_m {self.miss_far/1000.0:.1f} km")
        else:
            # (b) 停滯:地速持續低於門檻(墜毀/卡住)。gs 回升就把計時清掉,
            #     不可以把不連續的秒數累加起來。
            gs = self.gs.get(i)
            if gs is not None and gs < self.miss_slow_gs:
                t0 = self.slow_t0.setdefault(i, now)
                if now - t0 > self.miss_slow_sec:
                    why = (f"停滯:gs {gs:.1f} < {self.miss_slow_gs:.1f} m/s "
                           f"持續 {now - t0:.0f}s > {self.miss_slow_sec:.0f}s")
            else:
                self.slow_t0.pop(i, None)
        # (c) 首航次窗逾時。⚠️⚠️ dive_T 是**絕對牆鐘時戳**,不是時長,而且
        #     初始為 None、首架抵環時還會重排一次 ⇒ 用之前一定要 None 檢查,
        #     否則這一行的 TypeError 會把整段結算炸掉。
        if why is None and self.dive_T is not None \
                and now > self.dive_T + self.miss_window:
            why = (f"逾時:首航次窗已關閉 "
                   f"{now - (self.dive_T + self.miss_window):.1f}s"
                   f"(窗 = dive_go … dive_T+{self.miss_window:.0f}s)")
        if why is None:
            return False
        self.missed[i] = now
        self.miss_why[i] = why
        # 判定當下**鎖存**一個原地盤旋點:此後不再發俯衝指令,改用既有的
        # 守點路徑(hold_radius = 物理迴轉半徑)讓它在原地繞,不飛回紅心。
        # ⚠️ 要鎖存而不是每 tick 用當下位置 —— _goto 的 move_tol=40 m 會在
        #    飛機繞出 40 m 時重發,變成追著自己跑。
        # ⚠️ 相對高度剛好 0.0 會被 Location::sanitize() 判為「未設定」→ 指令被拒
        #    (和 _line_alt 結尾同一個坑)。
        z = self.pos[i][2]
        self.miss_pt[i] = (self.pos[i][0], self.pos[i][1],
                           z if abs(z) > 0.5 else -0.5)
        self.get_logger().warn(
            f"✗ UAV{i} 判定 MISS @+{now - self.dive_go:.1f}s(自 dive_go){why}"
            f";最近水平距凍結於 {self.dmin.get(i, float('nan')):.1f} m,"
            f"高度 {self.pos[i][2] - self._zg(i):+.0f}m(離紅心地表)"
            f"。停發俯衝指令,原地盤旋。")
        return True

    def _should_kill(self, agl, n_lo):
        """觸水終結的純述詞(無 ROS 依賴,單元測試就打這個)。

        n_lo = 到此為止**連續**落在 AGL ≤ kill_agl 的取樣筆數。
        要連續 kill_confirm_n 筆才算數:單筆的高度跳點(地形快取、
        GLOBAL_POSITION_INT 的 relative_alt 抖動)不足以終結一架飛機。
        """
        return (self.kill_on_touch
                and agl <= self.kill_agl
                and n_lo >= self.kill_confirm_n)

    def _kill_arm_update(self):
        """把「這架**曾經真的飛起來過**」記成閂鎖 —— 觸水終結的物理前提。

        ═══ 2026-08-10 REP1 事故 ═══
        起飛後 25 s 就下目標,兩機在 **6 秒內**全被觸水終結,而當下地速 95–106 m/s:

            t=0.0 uav=1 alt=-30.252 gs=100.4 agl= 1.8
            t=0.0 uav=2 alt=-36.089 gs=105.8 agl=-4.0    ⇒ 判定觸水

        真因不是基準不同步(那條 2026-08-07 已修),而是**飛控的高度估計還沒收斂**:
        剛 arm/起飛的那幾十秒,EKF 原點還在沉澱,`relative_alt` 回報 −30 m。
        `agl` 忠實地反映了一個**錯的輸入**,再多的基準對齊也救不了。

        ⇒ 補上一條**物理前提**:一架從來沒被觀測到離地過的飛機,不可能觸水。
           只有在某一刻量到 `agl ≥ kill_arm_agl` 之後,才為該機解除保險。
        ⚠️ 這是**閂鎖不是計時器**:不靠「起飛後 N 秒」這種會隨機體/風況漂的常數,
           而是要求觀測到那件事真的發生過。
        ⚠️ 它**不會擋掉正確的終結**:真的要觸水,必先飛起來 ——
           AP4 那架在 80 m 巡邏了 10 分鐘才掉下去,閂鎖早就解除,照樣終結。
        ⚠️ 待命期間也要更新(飛機在 STANDBY 就已經起飛爬升),
           所以 `_standby_tick` 與 `_row` 兩邊都要叫。
        """
        if not self.kill_on_touch:
            return
        for i in list(self.pos):
            if self._kill_armed.get(i):
                continue
            try:
                a = self._agl(i)
            except (KeyError, IndexError):
                continue
            if a >= self.kill_arm_agl:
                self._kill_armed[i] = True
                self.get_logger().info(
                    f"✔ UAV{i} 觸水終結解除保險:已觀測到 agl={a:+.0f} m "
                    f"≥ {self.kill_arm_agl:.0f} m(此前不論 agl 多低都不會終結 —— "
                    f"剛起飛時飛控的高度估計還沒收斂)")

    def _kill_airframe(self, i, agl, now):
        """砸進海面就強制 disarm,不讓 SITL 把它浮起來再飛第二趟。

        手段:MAV_CMD_COMPONENT_ARM_DISARM,param1=0(解除)、
        param2=21196(force magic,略過飛控端的「飛行中不准解除」檢查)。

        ⚠️ 為什麼不用別的:
          * MAV_CMD_DO_FLIGHTTERMINATION 要先推 AFS_ENABLE(預設 0)與
            AFS_TERM_ACTION,而且**不解除 arming** —— 前置條件多、保證少。
          * `pkill arduplane` 最徹底但跨層,而 stot_repeat.sh 每趟本來就重啟
            SITL;留作日後的保險絲即可,不放在這裡。
          * SITL 端沒有對應參數:它沒有「水」的概念(見 kill_on_touchdown
            的宣告處)。

        ⚠️⚠️ 呼叫點必須在 `_row()` **之後**。提前終結會讓「第一筆 AGL ≤ 0」
           那一列可能沒進 CSV,而 stot_score.py 的觸地內插要用
           (AGL>0, AGL≤0) 這一對相鄰取樣 —— 少了後者,一趟完美航次會被
           判成 n_touch=0。
        """
        if not self.kill_on_touch or i in self.killed:
            return
        if agl > self.kill_agl:
            self._kill_lo[i] = 0
            return
        # ⚠️⚠️ 物理前提:沒飛起來過就不可能觸水(見 _kill_arm_update 的事故說明)。
        #    這一條要在計數之前 —— 不然保險解除的當下計數已經滿了,等於沒擋。
        if not self._kill_armed.get(i):
            now_t = time.time()
            if now_t - self._kill_arm_warn_t.get(i, -1e9) > 30.0:
                self._kill_arm_warn_t[i] = now_t
                self.get_logger().warn(
                    f"⚠️ UAV{i} agl={agl:+.1f} m ≤ {self.kill_agl:.1f} m 但**不終結** —— "
                    f"這架還沒被觀測到爬過 {self.kill_arm_agl:.0f} m。"
                    f"剛起飛時飛控的高度估計未收斂會回報負高度(REP1 實測 −36 m "
                    f"而地速 106 m/s),那是壞輸入不是觸水。")
            return
        self._kill_lo[i] = self._kill_lo.get(i, 0) + 1
        if not self._should_kill(agl, self._kill_lo[i]):
            return
        c = self.link.get(i)
        if c is None:
            return
        try:
            c.mav.command_long_send(
                c.target_system or i, 1,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 21196, 0, 0, 0, 0, 0)
        except Exception as e:                       # 連線已斷就算了,不要拖垮迴圈
            self.get_logger().warn(f"UAV{i} 觸水終結送出失敗:{e}")
            return
        self.killed[i] = now
        t_rel = "" if self.dive_go is None else f" @+{now - self.dive_go:.1f}s"
        # ⚠️ 地速要印出來。2026-08-07 的誤殺(基準漂移,見 `_agl`)當下
        #    這架飛機在 150 m/s 巡航 —— 有這個數字,誤殺一眼就看得出來;
        #    沒有的話,log 上「agl=-0.2m ⇒ disarm」看起來完全合理。
        #    ⚠️ 刻意**只印不擋**:真正的觸水發生在 ~140 m/s 的俯衝末端,
        #    拿速度當閘門會把真的終結延後到「浮起來之後」,那正是
        #    second-pass contamination 那條教訓要防的事。
        self.get_logger().warn(
            f"☠ UAV{i} 觸水終結{t_rel}:agl={agl:+.1f}m(連續 "
            f"{self._kill_lo[i]} 筆 ≤ {self.kill_agl:.1f}m,"
            f"地速 {self.gs.get(i, float('nan')):.0f} m/s)⇒ 強制 disarm。"
            f"SITL 不會自己終結落海機體,不解除就會浮起來再飛一趟。")

    def _done_count(self):
        """結案計數 = impact ∪ missed。

        ⚠️⚠️ 摘要行與 phase=DONE 都被 `done == self.n` 擋著。若只把逃逸機
           排除出 impact 而**不**計入 done,結果是永遠不印摘要、永遠不進
           DONE、航次不會自己結束 —— 比不修更糟。
        ⚠️ 用集合在迴圈**外**算,而不是在迴圈裡 `done += 1`:迴圈裡有好幾條
           `continue`,計數點很容易被某條路徑跳過(而且完全靜默)。
        """
        return sum(1 for i in self.pos
                   if i in self.impact or i in self.missed)

    def _impact_summary(self):
        """產生結算摘要行(list[str])。抽成函式是為了能在沒有 ROS 的環境下測。

        行的結構(**哨兵與達標描述分離**,見 CLOSEOUT_SENTINEL 的說明):

            ★★ 俯衝結案:{命中}/{n} 命中、{miss} MISS[{miss 原因}];{達標描述}。{量測}
            └──── 固定哨兵(機器 grep)────┘└─ 固定計數 ─┘        └─ 人讀 ─┘

        四種結案形狀的達標描述:
          (a) 零 miss + 全部低空觸地 ⇒ 「俯衝命中紅心:全員命中」(= 舊哨兵,
              **逐字**保留,既有基準與歷史 log 的比對靠它)
          (b) 零 miss + 有機高空掠過 ⇒ 「⚠️ 名義命中,但 N/M 架在高空掠過…」
              (RD1 形狀。上一版在這裡把哨兵蓋掉 ⇒ 三個消費者全部落空)
          (c) 有 miss + 有命中       ⇒ 「命中的 N 架全部低空觸地」或 (b) 的句子
          (d) 全部 miss              ⇒ 「無人命中」
        (b)(c)(d) 一律**不准**出現「全員命中」四個字,免得舊 grep 把脫靶或
        高空掠過讀成成功。
        """
        hit = sorted(self.impact)
        miss = sorted(self.missed)
        ids = sorted(set(hit) | set(miss))
        # ⚠️⚠️ spread **只由 impact 計**。missed 的時戳是「出口判定時刻」,
        #    不是命中時刻;SW1 把逃逸機混進來的結果就是 11950 s 那個數字。
        imp = [self.impact[i] - self.dive_go for i in hit]
        spread = (max(imp) - min(imp)) if len(imp) >= 2 else 0.0

        def _fmt(fn):
            return "{" + ", ".join(
                (f"{i}: {fn(i)}" if i in self.impact else f"{i}: MISS")
                for i in ids) + "}"

        dmins = _fmt(lambda i: round(self.dmin[i], 1))
        halts = _fmt(lambda i: round(self.hit_alt.get(i, -1)))
        # ⚠️ 鎖存瞬間的水平距**另外印**,不拿它取代 dmin。
        #    兩者的關係是 hit_d ≥ dmin,差值 = 鎖存前那一段跑動出來的收斂量。
        #    以前的做法是讓 dmin 退化成 hit_d,結果是任何完美航次都被記成
        #    16–25 m(= impact_reach 減一個取樣步)。
        hitds = _fmt(lambda i: (round(self.hit_d[i], 1)
                                if i in self.hit_d else "n/a"))
        # ⚠️⚠️ 命中判定(見 impact_reach 那段)**只看水平距離**,不看高度。
        #    後果:從紅心正上方 1400 m 掠過、d 開始回升,一樣被記成「命中」。
        #    RD1 就是這樣報出「全員命中」的 —— 六架的命中高度其實是
        #    598/775/927/746/1460/1291 m,一架都沒落地,而摘要行照樣寫全員命中。
        #    這裡不改判定門檻(那會動到既有基準),但**必須把它講出來**:
        #    高於 impact_alt_ok 的一律點名,不准再靜默地混進「全員命中」。
        too_high = {i: round(h) for i, h in self.hit_alt.items()
                    if h > self.impact_alt_ok}
        # ⚠️ 分母是**命中數**,不是全隊 self.n。hit_alt 只有命中的機才有值,
        #    拿全隊當分母會在有 miss 的航次印出「2/6 架在高空掠過」而實際上
        #    只有 2 架命中 —— 那句話讀起來像「4 架好好落地了」,剛好相反。
        #    零 miss 時 len(hit) == self.n,值不變。
        # ⚠️ verdict 是**達標描述**(人讀),不參與結案偵測。哨兵另外由
        #    CLOSEOUT_SENTINEL 固定放在行首,verdict 再怎麼變都蓋不掉它。
        if too_high:
            verdict = (f"⚠️ 名義命中,但 {len(too_high)}/{len(hit)} 架在高空掠過"
                       f"(未落地){too_high}")
        elif not hit:
            verdict = "無人命中"
        elif miss:
            verdict = f"命中的 {len(hit)} 架全部低空觸地"
        else:
            # 形狀 (a):唯一准出現舊哨兵的路徑,而且必須**逐字**。
            verdict = LEGACY_ALL_HIT
        tail = (f"命中時間散布 = {spread:.2f}s "
                f"(≈{spread*self.dive_speed:.1f}m 縱向間隔);最近水平距 = {dmins} m;"
                f"命中高度 = {halts} m(門檻 {self.impact_alt_ok:.0f}m;"
                f"俯衝角 {self.dive_angle:.0f}°)"
                f";鎖存當下水平距 = {hitds} m"
                f"(與命中高度同一時刻。最近水平距在鎖存當下凍結、**無收斂窗**,"
                f"是首航次真最近點的上界(誤差 ≤ 一個取樣步);"
                f"更細的量以 stot_score.py 的 d1 為準)")
        # ⚠️⚠️ 哨兵 + 計數是**無條件**的:四種形狀走的是同一條 f-string,
        #    沒有 if/else 分支能把它繞過去。這正是上一版壞掉的地方 ——
        #    分支一多,「零 miss 但高空掠過」那條路徑就沒人印哨兵。
        #    miss 原因字典只在有 miss 時追加,不影響前面的固定部分。
        head = f"{CLOSEOUT_SENTINEL}{len(hit)}/{self.n} 命中、{len(miss)} MISS"
        if miss:
            head += f"{ {i: self.miss_why[i] for i in miss} }"
        lines = [f"{head};{verdict}。{tail}"]
        # 真值基準另外報一行。⚠️ 兩個基準差約 2 m(EKF 定位誤差),
        # 遙測基準會把它抵消、真值基準會疊加 —— 宣稱 ≤5 m 之前必須講清楚
        # 用的是哪一個,不能挑好看的那個報。
        if self.truth_dmin:
            tids = sorted(self.truth_dmin)

            def _tfmt(fn):
                return "{" + ", ".join(
                    (f"{i}: MISS" if i in self.missed else f"{i}: {fn(i)}")
                    for i in tids) + "}"

            lines.append(
                f"★★ 真值基準(SIM_STATE):"
                f"最近水平距 = {_tfmt(lambda i: round(self.truth_dmin[i], 1))} m;"
                f"命中高度 = {_tfmt(lambda i: round(self.truth_hit.get(i, -999)))} m")
        return lines

    def _tick(self):
        self._pump()
        # ⚠️ 順序有意義:_pump() 在 STANDBY 檢查**之前** —— 待命時鏈路照抽
        #    (見 _standby_tick 的說明),只是不下令。
        if self.phase == "STANDBY":
            self._standby_tick()
            return
        if len(self.pos) < self.n:
            # ⚠️⚠️ 這條 return 是**沒有逾時、沒有訊息**的等待:少一台的定位,
            #    整個節點就永遠停在這裡,現場看起來和「節點掛了」一模一樣。
            #    批次航次因為有外層 timeout 還撐得住,互動路徑(右鍵指定目標)
            #    完全沒有人在等 —— 使用者只會看到「按了沒反應」。30 s 抱怨一次。
            if self.phase == "INIT":
                now = time.time()
                if now - self._wait_pos_t > 30.0:
                    self._wait_pos_t = now
                    self.get_logger().error(
                        f"⚠️ 等機體定位中:{len(self.pos)}/{self.n} 台有 "
                        f"GLOBAL_POSITION_INT,缺 "
                        f"{[i for i in range(1, self.n + 1) if i not in self.pos]}。"
                        f"在湊滿之前**不會有任何動作、也不會逾時**。")
            return
        if self.phase == "INIT":
            self._assign_and_plan()
            self.phase = "ALIGN"
        now = time.time()
        stat = {}

        if self.phase == "ALIGN":
            for i in sorted(self.pos):
                d = dist_m(self.pos[i][:2], self.slot_of[i])
                t_rem = self.T_arr - now
                if i not in self.arr and d < self.reach:
                    self.arr[i] = now
                    self.get_logger().info(
                        f"UAV{i}→slot{self.slot_id[i]} @+{now-self.t0:.1f}s "
                        f"(偏差 {now-self.T_arr:+.1f}s)")
                if i in self.arr:
                    mode = "hold"; self._goto(i, *self.slot_of[i], self._alt(i), self.vmin)
                else:
                    v_req = d / max(t_rem, 0.1)
                    if v_req > self.vmax:
                        mode = "max"; self._goto(i, *self.slot_of[i], self._alt(i), self.vmax)
                    elif v_req >= self.vmin:
                        mode = "straight"; self._goto(i, *self.slot_of[i], self._alt(i), v_req)
                    else:
                        # S 型機動:以航向偏角 θ 精確控制「前向閉合率」= v_req。
                        # 令 cosθ = v_req/vmin ⇒ θ = acos(v_req/vmin),平均前向剛好 v_req。
                        mode = "weave"
                        ratio = max(0.05, min(0.985, v_req / self.vmin))
                        theta = math.degrees(math.acos(ratio))
                        # ⚠️⚠️ 換向週期以前寫死 22 s —— 那是**泡棉機時代**的值,沒有
                        #    隨機體改速度重新標定,而它決定 weave 到底吸不吸得到時間。
                        #    轉彎率 ω = g·tanφ/V:泡棉 14 m/s 是 40.1 °/s(θ=53° 只要
                        #    2.6 s ≪ 半週期 11 s,完全夠);本機體 157 m/s 只有
                        #    **3.58 °/s**,把航向從 +θ 甩到 −θ 要 2θ/ω:
                        #      θ=20° → 11.2 s、θ=30° → 16.8 s、θ=45° → 25.1 s
                        #    ⇒ 半週期 11 s 之下**任何 θ > 19.7° 都轉不完就被要求反向**,
                        #    實際達成的 cosθ 遠大於指令值,吸收到的時間遠小於設計值。
                        #    週期要 ≥ 4× 換向時間。120 s 之下 θ=45° 在 40 km 航段可
                        #    吸收 105 s(θ=30° 是 39 s、θ=20° 只有 16 s)。
                        #    ⚠️ 代價是橫向擺幅 A = V·sinθ·P/4,θ=45°/P=120 ⇒ 3331 m,
                        #      防撞分層(layer_m)要跟著放大。
                        sign = 1.0 if math.sin(
                            2 * math.pi * (now - self.t0) / self.weave_period) >= 0 else -1.0
                        bs = bearing(self.pos[i][:2], self.slot_of[i])
                        aim = dest(self.pos[i][0], self.pos[i][1], bs + sign * theta, 120.0)
                        self._goto(i, aim[0], aim[1], self._alt(i), self.vmin)
                stat[i] = {"mode": mode, "d": round(d),
                           "arr": round(self.arr[i] - self.t0, 1) if i in self.arr else None}
                self._row(now, i, d, mode, self.slot_of[i])
            self._csv.flush()
            self._proximity_check(now)
            if len(self.arr) == self.n:
                got = [self.arr[i] - self.t0 for i in self.arr]
                self.get_logger().warn(
                    f"★ 全員抵扇形。抵達散布 = {max(got)-min(got):.1f}s → PERCH(對準內切)"
                    f";ALIGN 全程最近水平接近 {getattr(self, '_prox_min', float('nan')):.0f}m"
                    f"(weave 週期 {self.weave_period:.0f}s;P91 舊設定實測 348m)")
                self.phase = "PERCH"; self.perch_t0 = now
                v_plan = self.plan_frac * self.vmax
                # 內切到 pushover 環的剩餘航程 ≈ (距T - pushover_r)
                tmax = max((dist_m(self.pos[i][:2], self.T) - self.ring_of(i)) / v_plan
                           for i in self.pos)
                self.perch_T = now + tmax + 6.0

        elif self.phase == "PERCH":
            # 對準內切:朝「穿越點」(T 後方)飛 → 航向已對準穿過 T 的直線(轟炸航線),
            # 爬到 perch_alt、速度同步在 perch_T 抵 pushover 環。就位不繞圈(續朝內、
            # 保持對準)→ DIVE 是「已對準的直線俯衝穿越」,不再轉彎偏開 T。
            for i in sorted(self.pos):
                thru = self.thru[self.slot_id[i]]
                pp = self.ring_point_of(i)
                d = dist_m(self.pos[i][:2], self.T)      # 距 T
                z = self.pos[i][2]
                # ⚠️ 距離 + 高度雙閘門。舊版只看距離 → 還沒爬到高點就被判定就位,
                # 俯衝從 778-1187m(而非指令 1500m)起跳,下滑角自然不足 → 掉不到底。
                # ⚠️⚠️ 就緒判定用**共同基準高度**,不是本機的階梯高度。
                #    階梯的用途是「在環上站不同高度」,而環在 25 km 內切之後才到 ——
                #    那段有 161 s,爬 1000 m 只要 39 s,完全來得及。
                #    把階梯放進就緒判定,等於要求高 slot 的機在扇形就先爬完 1000 m,
                #    於是它們更晚就緒。實測 AM81:就緒散布 57.5 s > 抵扇形散布 44.7 s
                #    —— PERCH 不但沒吸收散布,還把它放大了,而放大量正好是爬升時間差。
                z_ok = z >= self.perch_alt * self.perch_gate
                xt = self._xtrack(i)                     # 橫向偏離徑向線
                # ⚠️ 對正要「鎖存」,不能每 tick 重判:否則會形成極限環 ——
                #    飛出去對正 → 內切 → 稍微飄掉 → 又被趕出去,永遠進不了俯衝
                #    (實測 15 分鐘只有 2/6 通過閘門)。一旦在遠端對正過就放行。
                if xt <= self.align_tol:
                    self.align_ok[i] = True
                x_ok = self.align_ok.get(i, False)
                # ⚠️ 距離閘門要用「離環點 pp 多近」而不是「已越過環」:只要越過環就
                #    放行的話,各機開始俯衝時距 T 可差好幾百公尺 → 起跑點不等距 →
                #    再怎麼配速也壓不掉命中散布(實測卡在 3.3s)。收斂到 pp 才等距。
                # ⚠️ 距離閘門不能收得比「固定翼站位精度」還緊:曾用「離環點 <130m」
                #    當閘門,結果 3/6 永遠進不來(噴射迴轉半徑 ~275m,對定點只能繞)。
                #    所以這裡仍用「已越過環」,起跑點不等距的問題改用「錯開放行」解
                #    (見 DIVE 的 release_t):近的晚放,大家用同速飛 → 同時命中。
                # ⚠️ 「就緒」和對正一樣必須**鎖存**,而且要等**全員**就緒才一起內切。
                #    舊版每 tick 重判 z_ok:高度一沉就把飛機從內切改回「退到 4000m
                #    的 slot」,目標點一換就觸發一次 DO_REPOSITION,高度又被重錨
                #    (見 _goto)—— 於是「沉→被趕出去→換航點→再沉」自我維持。
                #    實測 B2:距離在 2834-4460 m、高度在 180-576 m、橫偏在 3-664 m
                #    之間振盪,六機沒有一架進得了環。
                if i not in self.ready and z_ok and x_ok:
                    self.ready[i] = now
                    self.get_logger().info(
                        f"UAV{i} 就緒 @+{now-self.t0:.1f}s "
                        f"(alt {z:.0f}m 橫偏 {xt:.0f}m)")
                if self.runup_t0 is not None and i not in self.perch_arr \
                        and d < self.ring_of(i):
                    self.perch_arr[i] = now
                    self.get_logger().info(
                        f"UAV{i} 就位對準環 @+{now-self.t0:.1f}s "
                        f"(alt {z:.0f}m 橫偏 {xt:.0f}m)")
                if i in self.perch_arr:
                    # 就位後「守在自己徑向線上的環點 pp」等其他機,而不是繼續內切。
                    # ⚠️ 舊版就位後仍朝 thru 慢飛 → 先就位的一路內漂,等最後一架好時
                    #    各機距 T 已經差很多 → 俯衝起跑點不等距 → 命中散布 26.7s。
                    #    瞄 pp 同時保住「半徑」和「對正」(pp 就在自己的徑向線上)。
                    # ⚠️ 守環是**刻意要繞**的,和其他 mode 語意相反 ⇒ 給大半徑。
                    mode = "hold_ring"
                    # ⭐ S8:高度與速度都改走 helper。兩者預設回傳
                    #    perch_of(i) / vmin ⇒ 逐位元同舊行為。
                    self._goto(i, *pp, self.hold_alt_of(i), self.hold_v_of(),
                               radius=self.hold_radius)
                elif self.runup_t0 is None:
                    # 高度不夠或還沒對正 → 先退回扇形 slot(4000m 外、就在徑向線上)
                    # 邊飛邊爬邊對正;飛向 slot 本身就會把飛機拉回徑向線。
                    # ⚠️ 曾用「守在 pushover 環上爬」,但環就在附近,飛機憑慣性/迴轉半徑
                    #    仍持續內漂:實測 6 機有 5 機爬到高度時已經只剩 320-707m,
                    #    需要 54-71° 才打得到 → 物理上不可能 → 全部從紅心上空飛過。
                    #    (唯一在 2070m 進入俯衝的 UAV2 只需 22.3°,就真的砸到 93m。)
                    #    退到扇形 slot 可保證「爬完才開始內切」,進入距離 = 環半徑。
                    # ⚠️ 這裡不能用 vmin。實測死結:退回扇形 slot 等對正/爬升時
                    #    用 vmin(45 m/s)飛,22 kg 噴射撐不住高度 —— 五機停在
                    #    4052-4172m 外、高度一路掉到 216-306m,永遠達不到
                    #    0.9*perch_alt=360m 的閘門,PERCH 卡死 20 分鐘不動。
                    # ⚠️ 用最佳爬升速度,不是巡航速度 —— 見 perch_climb_v 的說明。
                    v_cl = (self.perch_climb_v if self.perch_climb_v > 1.0
                            else max(self.vmin * 1.45, 65.0))
                    # 同理:扇形上的爬升只爬到共同基準,階梯留給內切段。
                    mode = "climb"; self._goto(i, *self.slot_of[i], self.perch_alt, v_cl)
                else:
                    t_rem = self.perch_T - now
                    v = max(self.vmin, min(self.vmax, (d - self.ring_of(i)) / max(t_rem, 0.1)))
                    # ⭐⭐ 2026-08-14(S8 修補):runup 也要吃 hold_alt。
                    # ⚠️ 第一版只改了 hold_ring 與 wait,**漏了這裡**,而 runup
                    #    是三段裡最長的一段(RPU 實測 191–213 s)。
                    #    後果(RPU 六架次實測):
                    #        climb 爬到 2661–2832
                    #        runup **再爬 +439~+894** → 3155–3719   ← 反向做功
                    #        hold_ring 只剩 25–41 s 把它降回 2732–2772
                    #    ⇒ 降多少取決於分到幾秒,而那逐機不同
                    #      ⇒ 交接高度散布 916 m 完全沒有收斂,散布 7.2–12.6 s。
                    # ⇒ hold_alt 有效(每架確實降了 424–944 m),問題是上游一直在拉高。
                    # ⚠️ climb 段**不改**:那一段的任務就是爬到高度,
                    #    在那裡壓低會讓「爬完了沒」的判定失去意義。
                    mode = "runup"; self._goto(i, *thru, self.hold_alt_of(i), v)
                stat[i] = {"mode": mode, "d": round(d), "z": round(self.pos[i][2])}
                self._row(now, i, d, mode, thru)
            self._csv.flush()
            # 全員就緒才一起放行內切 —— 起跑點齊、航線已對正,之後不再重判。
            if self.runup_t0 is None and len(self.ready) == self.n:
                self.runup_t0 = now
                v_plan = self.plan_frac * self.vmax
                tmax = max((dist_m(self.pos[i][:2], self.T) - self.ring_of(i))
                           / v_plan for i in self.pos)
                self.perch_T = now + tmax + 6.0
                zs = {i: round(self.pos[i][2]) for i in sorted(self.pos)}
                self.get_logger().warn(
                    f"★ 全員就緒(alt {zs})→ 同步內切到環,預計 {tmax+6.0:.0f}s")
            # ⭐⭐ S1(2026-08-18):`dive_from_runup` —— 內切一開始就翻 DIVE。
            #
            # ⚠️⚠️ 為什麼:RRB 實測「抵環 → 放行」中間隔了中位 **32.1 s**,
            #    而那 32 秒裡飛機一路往內漂(6079 → 3417 m)、掉高度中位 **970 m**,
            #    高度散布從抵環的 257 m 惡化到放行的 805 m
            #    ——**高度收斂閘門的成果在等待期間被完全吃回去。**
            #    更致命的是前置角:抵環當拍 σ 中位 **0.1°**(14/14 都在 0.0–0.3°),
            #    放行當拍 σ 中位 **130.2°**、最小值 73.7°。
            #    ⇒ 全部語料裡**從來沒有取樣過 σ≈0 的放行**(見
            #      [[sigma-tolerance-collapses]]:σ=0 時 2 s 預算容許 ±125°,
            #      σ=125° 只剩 ±1.1°)。
            #
            # ⇒ 把 ring_t0 的計時起點從「全員抵環」提前到「內切起點」,
            #   phase 在內切開始後 perch_settle 秒就翻 DIVE,飛機直接走 DIVE 的
            #   runin/wait 分支飛向環,抵環當拍由 `release_at_ring` 放行。
            #   hold_ring 整段不再發生。
            #
            # ⚠️⚠️ 這裡有一顆**靜默地雷**,必須連帶處理:下面
            #    `ring_t0 is not None and phase == "PERCH"` 這個視窗裡每 tick 重送
            #    陡俯角參數(PTCH_LIM_MIN_DEG=-80 / TECS_PITCH_MIN / TECS_SPDWEIGHT /
            #    THR_MAX / TECS_HDEM_TCONST),而原始碼自己的註解就寫著
            #    「送出去了沒吃進去是已知會發生的」。
            #    若 band 一成立就翻相,視窗會塌成 **1 個 tick** ⇒ 那組開關等於沒開,
            #    而 [[steep-dive-switches-are-the-fix]] 記載它們是 0/6 變 4/6 的關鍵。
            #    ⇒ dive_from_runup 開著時**停用 band 捷徑**,強制等滿 perch_settle,
            #      讓重送視窗保證有 perch_settle × 20 Hz 個 tick。
            if self.dive_from_runup:
                if self.runup_t0 is not None and self.ring_t0 is None:
                    self.ring_t0 = self.runup_t0
                    self.get_logger().warn(
                        "★ dive_from_runup:內切起點即開始 DIVE 倒數"
                        f"({self.perch_settle:.0f}s),不在環上等")
            elif len(self.perch_arr) == self.n and self.ring_t0 is None:
                self.ring_t0 = now      # 全員抵環,開始「高度收斂」等待
            # ⚠️ 抵環後先讓高度收斂再俯衝。各機抵環時的高度散布實測 200+ m
            #    (F3 496–720、F5 817–1039),而 gs = V·cos(航路角),進入高度不同
            #    ⇒ 航路角不同 ⇒ 同樣的總速換算出的**地速差 25–30 m/s**
            #    (連跑實測最慢 71–81 對最快 100–108,每一趟都這樣)。
            #    這既讓「六機全部 ≥100」做不到,也是命中散布的來源。
            #    全部進到 perch_band 內就放行;拖太久(perch_settle)就認賠放行,
            #    免得像早期版本那樣卡死在閘門上。
            if self.ring_t0 is not None and self.phase == "PERCH":
                band = max(abs(self.pos[i][2] - self.perch_of(i))
                           for i in self.pos)
                waited = now - self.ring_t0
                # ⚠️ 見上面 dive_from_runup 的說明:band 捷徑會讓陡俯角參數的
                #    重送視窗塌成 1 個 tick,而那個失敗是完全靜默的。
                band_ok = (band <= self.perch_band) and not self.dive_from_runup
                if band_ok or waited >= self.perch_settle:
                    zs = {i: round(self.pos[i][2]) for i in self.pos}
                    self.get_logger().warn(
                        f"★ 全員對準內切(alt {zs};高度收斂 {band:.0f}m/"
                        f"門檻 {self.perch_band:.0f}m,等了 {waited:.0f}s)"
                        f"→ DIVE 直線俯衝穿越!")
                    self.phase = "DIVE"; self.dive_go = now
                if self.pn_on and not self.fc:
                    self._fc_connect()
                # TECS_SPDWEIGHT 是相位相依的:俯衝要 0(pitch 只追高度 → 肯壓機頭),
                # 但巡航要 1(=0 時 pitch 不管空速,速度只剩油門在撐,編隊追不上長機,
                # 實測 slot 誤差從 289m 一路惡化到 722m)。所以只在進俯衝的當下切 0。
                for j in self.link:
                    lk = self.link[j]
                    lk.mav.param_set_send(lk.target_system or j, 1,
                                          b"TECS_SPDWEIGHT", self.dive_spdw,
                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    # ⭐ 俯衝的油門權限:PN 送的 thr=1.0 會被夾到 THR_MAX,
                    #    巡航設計值 25 之下「全油門」只有 25%。這裡放開。
                    if self.dive_thr_max > 0.0:
                        lk.mav.param_set_send(
                            lk.target_system or j, 1, b"THR_MAX",
                            self.dive_thr_max,
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    if self.hdem_tconst > 0.0:
                        lk.mav.param_set_send(
                            lk.target_system or j, 1, b"TECS_HDEM_TCONST",
                            self.hdem_tconst,
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    # 低頭權限也是相位相依的,理由和 TECS_SPDWEIGHT 一樣:
                    # 終端俯衝要 −80°,但巡航段開著 −80° 會讓 TECS 在油門飽和時
                    # 拿高度去換速度,把飛機開進地面(高速改裝的 T/W 只有 0.274,
                    # 油門很容易飽和;實測 alt 227 → −5.9,11 秒撞地)。
                    # 所以巡航段由 .parm 壓在 −20°,這裡才放開。
                    if self.dive_pitch_min < 0.0:
                        # ⭐⭐ 2026-08-14(S3)把兩個限制**拆開**送。
                        # 先前兩者綁死成同一個 −80,而它們管的是不同的東西:
                        #   PTCH_LIM_MIN_DEG —— 姿態迴路的俯仰下限。PN 要陡俯角就
                        #     一定要放開(mode_guided 的姿態指令被它夾)。
                        #   TECS_PITCH_MIN   —— **TECS 自己**能要求的俯仰下限。
                        #     PN 走姿態通道時根本不經過 TECS,所以它不需要 −80。
                        # ⚠️ 綁在一起的代價:任何一次 GUIDED_TIMEOUT 掉回 TECS,
                        #    TECS 手上就有 −80° 可以拿高度換速度 —— 正是上面註解
                        #    記載的「alt 227 → −5.9,11 秒撞地」那個機制。
                        # ⚠️ 預設 tecs_pitch_min = dive_pitch_min ⇒ **逐位元維持舊行為**。
                        #    要拆開請顯式設 tecs_pitch_min(建議 −15),那是行為改動。
                        _tecs_min = (self.tecs_pitch_min
                                     if self.tecs_pitch_min < 0.0
                                     else self.dive_pitch_min)
                        for pname, _v in ((b"TECS_PITCH_MIN", _tecs_min),
                                          (b"PTCH_LIM_MIN_DEG", self.dive_pitch_min)):
                            lk.mav.param_set_send(
                                lk.target_system or j, 1, pname, _v,
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                        # ⚠️ 抬頭上限以前**從來沒被放開**:.parm 把它壓在 +15(巡航段
                        #    T/W 0.274 只爬得動 ~12°,那是對的),但 PN 的 python 端
                        #    夾在 +45 —— 節點以為有 45° 抬頭權限、韌體只給 15°,
                        #    兩邊不一致而且無訊息。陡俯角時 PN 要拉平/修正下衝就要
                        #    正俯仰,那部分會被靜默削掉。MIN 放開時 MAX 也要一起放。
                        for pname in (b"TECS_PITCH_MAX", b"PTCH_LIM_MAX_DEG"):
                            lk.mav.param_set_send(
                                lk.target_system or j, 1, pname,
                                self.dive_pitch_max,
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                        # ⚠️ 這是把飛控從巡航夾制放開的**唯一**通道,而且只在
                        #    「ring_t0 已定且 phase 還是 PERCH」這個視窗內執行 ——
                        #    phase 一翻成 DIVE 就再也不會執行。perch_settle=5s 的
                        #    最壞情況只送出一個 tick,而分流鏈路「送出去了但沒吃進去」
                        #    是已知會發生的(swarm_sitl_up.sh:100-102 自己記著)。
                        #    再走一次直連飛控鏈路當備援,兩條都送。
                        fc = self.fc.get(j) if self.fc else None
                        if fc is not None:
                            for pname, val in (
                                    # ⭐ S3:直連備援也要拆,否則兩條鏈路送不同的值,
                                    #    而「後到的贏」會讓生效值取決於時序 —— 最糟的
                                    #    一種不確定性(看起來隨機、事後查不出來)。
                                    (b"TECS_PITCH_MIN", _tecs_min),
                                    (b"PTCH_LIM_MIN_DEG", self.dive_pitch_min),
                                    (b"TECS_PITCH_MAX", self.dive_pitch_max),
                                    (b"PTCH_LIM_MAX_DEG", self.dive_pitch_max)):
                                try:
                                    fc.mav.param_set_send(
                                        fc.target_system or j, 1, pname, val,
                                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                                except Exception:
                                    pass
                # 共同命中時刻:用「最遠的那架 × 可達俯衝速」定 T_impact,近機照這個
                # 時刻放慢配速 → 六機同時到。配速用實際可達速(非 dive_speed 指令值),
                # 否則 t_rem 立刻歸零、全員全速、失去等待餘裕。
                dmax = max(dist_m(self.pos[i][:2], self.T) for i in self.pos)
                self.dive_pace = self.pace_cfg if self.pace_cfg > 1.0 else min(
                    self.vmax, 0.8 * self.dive_speed)
                # ⭐⭐ **不變式:pace ≥ vmin。** 違反就等於沒有同步迴路。
                #    俯衝速度指令是 `v = max(vmin, min(dive_speed, d/t_rem))`;
                #    pace < vmin ⇒ 時刻表要求的速度**恆低於**下限 ⇒ 整段被 max() 夾住
                #    ⇒ ① 近機無法慢下來等遠機(同步權限歸零)
                #       ② dive_speed 永遠不生效(終端速度被壓在 vmin)
                #    實測 RPJ(2026-08-11):pace 50(泡棉機常數)vs vmin 95
                #    ⇒ 早到 25–52 s、散布 25.00 s、終端只有 115–120 m/s。
                #    ⚠️ 這是**靜默**失效:飛機照飛、命中照樣算,只有時刻表對不上。
                if self.dive_pace < self.vmin:
                    self.get_logger().error(
                        f"⚠️⚠️ 俯衝配速 {self.dive_pace:.0f} < vmin {self.vmin:.0f} "
                        f"⇒ v_req 恆低於下限、整段被夾住:**同步迴路等於不存在**,"
                        f"且 dive_speed={self.dive_speed:.0f} 不會生效"
                        f"(終端速度會被壓在 vmin)。"
                        f"DIVE_PACE 設 0 走自動 = min(vmax, 0.8·dive_speed)"
                        f" = {min(self.vmax, 0.8 * self.dive_speed):.0f}")
                # 共同命中時刻用「最慢的那架自己的預估航程時間」定,不是 dmax/單一 pace。
                self._set_dive_T(now, max(
                    self._tgo_of(j, dist_m(self.pos[j][:2], self.T))
                    for j in self.pos) + 2.0, "進 DIVE 初排")
                # 錯開放行:近的先在環上多等一會兒,之後大家用同一個配速飛。
                # ⚠️ 這是「不靠減速」達成同時命中的關鍵 —— 靠減速行不通,因為近機
                #    需要的速度會低於噴射可持續下限 vmin(45),被夾住就一定早到
                #    (實測早到 14.5s;把 vmin 壓到 38 又會失速亂飛)。
                lag = {j: round((dmax - dist_m(self.pos[j][:2], self.T))
                                / self.dive_pace, 1) for j in sorted(self.pos)}
                self.get_logger().info(
                    f"DIVE 配速:最遠 {dmax:.0f}m / {self.dive_pace:.0f} m/s "
                    f"→ 共同命中 T+{self.dive_T-now:.0f}s;預估錯開 {lag}"
                    f"(實際放行是閉迴路,每 tick 用當下距離重判)")
                # ── miss 出口的生效值(專案慣例:新機制一定要印出**實際生效**的
                #    數字。P91 的對照之所以失效,就是因為沒有這一行,事後分不出
                #    「機制沒開」和「機制開了但沒用」)。
                #    只印一次:上面這整段在 ring_t0 之後每 tick 都會跑。
                if self.dive_go is not None \
                        and not getattr(self, "_miss_cfg_logged", False):
                    self._miss_cfg_logged = True
                    r_ring = max(self.ring_of(j) for j in self.pos)
                    d_now = {j: round(dist_m(self.pos[j][:2], self.T))
                             for j in sorted(self.pos)}
                    # ⚠️⚠️ 先夾再印 —— 印出來的必須是**實際生效**的那個數字。
                    #    夾在這裡是安全的:_miss_check 的前提是 dive_go 已設定,
                    #    而這段就在 dive_go 之後、DIVE 分支的第一個 tick 之前。
                    _old_far, _new_far, _clamped = self._clamp_miss_far(
                        r_ring, float(max(d_now.values())))
                    self.get_logger().warn(
                        f"★ miss 出口生效值:首航次窗 = dive_go … "
                        f"dive_T+{self.miss_window:.0f}s"
                        f"(= dive_go+{self.dive_T + self.miss_window - self.dive_go:.1f}s;"
                        f"絕對時戳 {self.dive_T + self.miss_window:.1f},"
                        f"比較式寫成 now > dive_T + window ——"
                        f"dive_T 是 epoch 不是時長);"
                        f"miss_far_m={self.miss_far:.0f}m"
                        f"(原值 {_old_far:.0f}m、miss_far_k={self.miss_far_k:.1f}、"
                        f"{'已夾寬' if _clamped else '未夾'};"
                        f"只對**已放行**的機檢查);"
                        f"miss_slow={self.miss_slow_gs:.1f}m/s 持續 "
                        f"{self.miss_slow_sec:.0f}s;"
                        f"dive_go 當下各機距 T = {d_now} m、最大環半徑 "
                        f"{r_ring:.0f}m;"
                        f"impact_reach={self.impact_reach:.0f}m、"
                        f"switch_r={self.switch_r:.0f}m、"
                        f"impact_alt_ok={self.impact_alt_ok:.0f}m(未改動);"
                        f"最近水平距(dmin)生效值:**鎖存當下凍結、無收斂窗**"
                        f"(2026-08-04 移除遲滯/硬上限;真最近點以 stot_score "
                        f"的觸地收窗為準);"
                        f"impact_settle={self.impact_settle:.1f}s")
                    # ⚠️⚠️ 逃逸門檻必須明顯高於「還在環上合法等待」的距離,
                    #    否則等放行的機會被自己判成逃逸,而且是**全隊同時**。
                    #    環半徑 = (perch_alt − dive_alt)/tan(dive_angle):
                    #    dive_angle 調淺時它會長很快(25° 是 6.4 km,5° 是 34 km)。
                    #    R1 起這裡**真的動手夾**,不再只是印一行 error。
                    if _clamped:
                        self.get_logger().error(
                            f"⚠️⚠️ miss_far_m 生效值已自動夾寬:"
                            f"{_old_far:.0f}m → {_new_far:.0f}m"
                            f"(= miss_far_k {self.miss_far_k:.1f} × "
                            f"max(環半徑 {r_ring:.0f}m, 當下最遠 "
                            f"{max(d_now.values())}m))。"
                            f"原值相對環半徑太緊,還在環上等放行的機會被判成"
                            f"逃逸,而且是全隊同時 —— 已夾住,不是只警告。"
                            f"(SW1 的比值是 20000/4697 = 4.3,不需要夾)")

        elif self.phase == "DIVE":
            for i in sorted(self.pos):
                d = dist_m(self.pos[i][:2], self.T)
                # ── 終端交接:進了 gate_r 就改用 PN 直接下姿態 ──
                # 航點導引本質到不了個位數公尺:GUIDED 高度需求被
                # TECS_HDEM_TCONST 節流、L1 的時間常數 ζ·T/π ≈ 4 s 在終端
                # 根本來不及修。PN 用 50 Hz 姿態指令繞開兩者。
                # ── 閉迴路放行 ──────────────────────────────────────────
                # 需要的時間 d/配速 >= 剩下的時間 ⇒ 該走了;還有餘裕就繼續等。
                # ⚠️ 舊版是在進 DIVE 當下把 release_t 一次算死。但等待中的飛機
                #    還在動(守環點、固定翼站不住只能繞),最久要等 ~12 s,
                #    45 m/s 下就是 540 m 的位移 —— 錯開時間完全失真。
                #    實測 Q1:進俯衝後半徑 2000 m 處散布已經 10.70 s,而俯衝段
                #    本身一路在收斂(10.70 → 5.50)⇒ 散布是放行條件造成的,
                #    不是終端導引沒有權限。改成每 tick 用當下距離重判即可自我修正
                #    (往內漂 → d 變小 → 更晚放;往外飄 → 更早放),放行後鎖存。
                if not self.released.get(i):
                    # ⭐⭐ 環閘門:還沒到 pushover 環就不准放行,不管時刻表怎麼說。
                    #
                    # ⚠️⚠️ 沒有這道閘門,DIVE 階段一開始(蜂群還在 35 km 扇形上)
                    #    時刻表就把全隊放掉了 —— 因為 dive_T 是用「當下距離」算的,
                    #    d/pace 一開始就幾乎等於剩餘時間。放行後瞄準點是紅心本身,
                    #    從 34 km 瞄紅心的下滑線只有 **10°**,不是 dive_angle。
                    #    實測 AE1:UAV2 在「dive」模式下平飛了 33 km(33 km 只掉 70 m,
                    #    實得俯角 1.8°),到 d=2000 m 還在 4871 m,然後才暴拉到 70°
                    #    —— 那時已經來不及,殘餘 2113 m。
                    #    環半徑修對了也沒用,因為俯衝在抵環前 30 km 就開始了,
                    #    環等於不存在。
                    #    ⚠️ 這不會重新引入「目標上空盤旋」:時間對齊仍在中途段
                    #    (ALIGN 的 weave)完成,環上的等待上限還是 perch_settle。
                    at_ring = (not self.dive_ring_gate) or \
                        d <= self.ring_of(i) * self.ring_gate_frac
                    # ⭐ 抵環時重排時刻表。dive_T 原本是在進 DIVE 的當下(蜂群還在
                    #    35 km 扇形)用當時的距離算死的,等飛機真的飛完 30 km 抵環,
                    #    dive_T − now 已經深度為負 ⇒ v_req = R/t_rem 對每一架都飽和
                    #    成 pn_vmax ⇒ **俯衝段完全沒有時間調節權限**,抵達順序就是
                    #    抵環順序。實測 AG1 散布 44.8 s(而俯衝段本身有 3.6 s 權限)。
                    #    第一架抵環時以「當下六機各自的預估航程」重排一次。
                    if at_ring and not self.ring_replan:
                        self.ring_replan = True
                        t_need = max(
                            self._tgo_of(j, dist_m(self.pos[j][:2], self.T))
                            for j in self.pos)
                        # ⚠️ 舊表過期量要在覆寫**之前**算 —— 覆寫後再減就永遠是 0,
                        #    而 RPY3 印出來的正是「舊表已過期 0s」:那行本來就該
                        #    暴露問題,卻因為算在覆寫之後而長得一切正常。
                        stale = (None if self.dive_T is None
                                 else (now + t_need + 2.0) - self.dive_T)
                        if self._set_dive_T(now, t_need + 2.0, "首架抵環重排"):
                            self.get_logger().warn(
                                f"★ 首架抵環 → 重排共同命中時刻:T+{t_need + 2.0:.1f}s"
                                f"(舊表{'尚未鎖定' if stale is None else f'已過期 {stale:+.0f}s'})")
                    if not at_ring:
                        pass
                    elif (self.release_at_ring and d <= self.ring_of(i)) \
                            or (not self.release_at_ring
                                and self._tgo_of(i, d) >= self.dive_T - now):
                        # ⭐⭐ S1(2026-08-18):`release_at_ring` = **抵環即放行**。
                        #
                        # ⚠️⚠️ 左邊的短路是刻意的:release_at_ring 開著時
                        #    **完全不計算 `_tgo_of`、完全不讀 `dive_T`**。
                        #    理由是時刻表本身不可達 —— [[predictor-underestimates-dive]]
                        #    記載預測器低估俯衝耗時中位 +18.7 s、27/27 單邊為正,
                        #    所以 `_tgo_of >= dive_T - now` 幾乎一放行就成立,
                        #    真正決定放行時刻的是「飛機漂到哪裡」而不是時刻表。
                        #    RRB 實測:放行 R 橫跨 2333–5560 m ⇒ 光是放行距離散布
                        #    就值 ~21 s 的俯衝耗時散布([[sigma-tolerance-collapses]]:
                        #    σ=0 時耗時 ≈ R/V,ΔR 500 m ⇒ Δ耗時 3.3 s)。
                        #
                        # ⚠️ 判準用 `d <= ring_of(i)`(嚴格抵環),**不是** at_ring 的
                        #    `ring_of(i) * ring_gate_frac`(1.10 ⇒ 6692 m)。
                        #    ring_gate_frac 同時是 `_ring_transit_s`、`_tgo_of`、
                        #    `ring_pace` 的 eta 閘門、以及 runin/wait 的 far 判定
                        #    這四個機制對「環在哪裡」的共同認知 —— 為了放行去改它
                        #    會讓四者分裂。所以這裡另外用嚴格判準,frac 不動。
                        #
                        # ⚠️ n=8:純幾何、逐機獨立、不搶共用資源 ⇒ 可 scale。
                        #    但 `_reground()` 會在飛行中改寫 pushover_r ⇒ ring_of(i)
                        #    會跳,而純幾何放行沒有第二道防線(環一縮就全隊同拍放行)。
                        #    ⇒ 海上目標務必用 TARGET_AMSL 釘死([[sea-level-target-datum]])。
                        #
                        # ⚗️ 實驗閘門:只在指定前置角視窗放行(生產組態下恆為 True)
                        if self._sigma_gate_ok(i):
                            self.released[i] = True
                            if self.release_at_ring:
                                self.get_logger().warn(
                                    f"UAV{i} 抵環放行 @ d={d:.0f}m "
                                    f"(環 {self.ring_of(i):.0f}m、σ="
                                    f"{self._lead_angle_deg(i):.1f}°、"
                                    f"alt {self.pos[i][2]:.0f}m)")
                    elif self.dive_angle_max > 0.0:
                        # ⭐ 幾何守門員:再等下去所需俯角就超過機體能力 ⇒ 立刻放行,
                        #    不管時刻表怎麼排。
                        # ⚠️ 這是「打不到」的真因,不是導引律沒權限:等待中的飛機
                        #    在環上會持續往內漂,而高度不變 ⇒ 所需俯角 atan(h/d) 一路
                        #    變陡。實測 AB1 的 UAV3 放行時已在距紅心 2 km、高 3402 m
                        #    的位置,需要 59.7° 而實得 35.3° —— 那時任何導引律都救不了。
                        #    同一趟正常的機需要 22–37°,可見放行本來就偏晚。
                        #    守門員讓「時間對齊」永遠讓位給「幾何可達性」:寧可早到,
                        #    也不要為了對時間而把自己等到打不到。
                        h_now = max(0.0, self.pos[i][2] - self._zg(i))
                        gam_req = math.degrees(math.atan2(h_now, max(d, 1.0)))
                        if gam_req >= self.dive_angle_max and self._sigma_gate_ok(i):
                            self.released[i] = True
                            self.get_logger().warn(
                                f"UAV{i} 幾何放行 @ d={d:.0f}m h={h_now:.0f}m "
                                f"(所需俯角 {gam_req:.1f}° ≥ 上限 "
                                f"{self.dive_angle_max:.0f}°;時刻表還要等 "
                                f"{self.dive_T - now - self._tgo_of(i, d):.1f}s)")
                released = self.released.get(i, False)
                # ⚠️ 還在環上等放行的機不可以進 PN:那會一邊用姿態律撲向目標、
                #    一邊被 wait 分支用 DO_REPOSITION 拉回環點,兩套指令打架。
                # ⚠️ 已判 MISS 的同理:PN 會繼續把它撲向紅心,而下面的 missed
                #    分支又要它原地盤旋 —— 所以守門條件要一起加上 missed。
                if self.pn_on and self.fc and released \
                        and not self._metrics_frozen(i):
                    # ⭐⭐ 分時獨立解脫(Staggered Departure)
                    #
                    # ⚠️⚠️ 舊版用**統一的空間座標** gate_r 解脫:所有機都在
                    #    「距紅心 gate_r」這個同心圓上交接 PN。編隊縱深小的時候
                    #    這等價於同時解脫;但 B-500 的縱深是 **8 km**,六機抵達
                    #    同一個圓的時刻天然就差 8000/157 = **51 s**,
                    #    而俯衝段的油門權限只有 3.6 s —— 補不回來。
                    #    實測 AO1 散布 33.4 s,正是這個幾何差的殘留。
                    #
                    # 改成**時變 t_go 觸發**:每機自己算「照現在的可達速度飛到
                    # 紅心還要多久」(t_go),當 t_go ≤ 距共識命中時刻的剩餘時間,
                    # 就輪到它單獨退出編隊、啟動 PN。
                    #   後方的機 t_go 大 ⇒ **先**解脫(它要飛比較久)
                    #   前方的機 t_go 小 ⇒ **後**解脫
                    # 於是 8 km 的空間縱深被轉換成解脫時刻的錯開,六機**同時**到達。
                    # 這正是「用時間座標取代空間座標」——同時性本來就是時間問題,
                    # 用一個空間門檻去表達它,在編隊縱深不可忽略時必然失真。
                    #
                    # ⚠️ gate_r 仍保留為**上限**:再近就一定要交接,否則 PN 沒有
                    #    收斂距離。t_go 只讓它「提早」,不會讓它更晚。
                    #
                    # ⚠️⚠️ **實測 AM21 反證:預設關閉(stagger_depart=False)。**
                    #    把 t_go 套在 PN 交接上是錯的 —— gate_r 是**導引律**的
                    #    性質(PN 需要有界的接戰距離),不是協同的性質。
                    #    實測:六機在 d=8.3–10.5 km、h=4.5–5.0 km 就交接 PN
                    #    (設計 gate_r 只有 4800 m),結果誤差 22652 m、
                    #    最低 AGL 5045 m、散布 138.3 s —— 全隊飛散。
                    #    分時解脫該做在**離開編隊**那一層,而 `released` 的判定
                    #    `d/_pace_of >= dive_T − now` 本來就已經是 t_go 觸發。
                    #    要重新啟用請先給 PN 一個獨立的最大接戰距離上限。
                    if not self.strike.get(i):
                        t_go = self._tgo_of(i, d)
                        # ⚠️ 減法必須在 None 守衛**之後**。上一版是先算
                        #    `t_left = self.dive_T - now` 再在下一行的條件裡寫
                        #    `self.dive_T is not None` —— 守衛是死碼,dive_T 還
                        #    沒共識鎖定時先炸 TypeError,整個 tick 被打斷。
                        #    正確寫法見 _miss_check 的 (c) 條。
                        t_left = None if self.dive_T is None else self.dive_T - now
                        by_tgo = (self.stagger_depart
                                  and t_left is not None
                                  and t_go >= t_left - self.stagger_lead)
                        # ⚠️ gate 每拍現算(gate_theta_max>0 時)。連續 gate_hold_n 拍
                        #    成立才交棒 —— 動態 gate 會隨 h/V 抖動,單拍觸發等於
                        #    把雜訊接成指令(參照 stot-altitude-gate-chatter 的先例:
                        #    閘門沒遲滯 ⇒ 翻轉次數 ↔ 末端誤差)。交棒本身仍是不可逆鎖存。
                        g_r = self.gate_of(i)
                        if d <= g_r:
                            self._gate_hits[i] = self._gate_hits.get(i, 0) + 1
                        else:
                            self._gate_hits[i] = 0
                        by_gate = self._gate_hits.get(i, 0) >= self.gate_hold_n
                        if by_tgo or by_gate:
                            self.strike[i] = True
                            h_g = self.pos[i][2] - self._zg(i)
                            los = math.degrees(math.atan2(h_g, max(d, 1.0)))
                            r_ring = self.ring_of(i)
                            self.get_logger().info(
                                f"UAV{i} 交接終端 PN @ d={d:.0f}m "
                                f"h={h_g:.0f}m "
                                f"[{'t_go 觸發' if by_tgo else 'gate_r 觸發'}:"
                                f"t_go {t_go:.1f}s vs 剩餘 "
                                f"{'n/a(dive_T 未鎖定)' if t_left is None else f'{t_left:.1f}s'}]"
                                # ⭐ 判別式的自變數以前完全沒進 log,只能事後從 d 反推。
                                #    γ 是 828 架次裡唯一單調的量(ρ=+0.892),一定要記。
                                f" gate={g_r:.0f}m 環={r_ring:.0f}m "
                                f"gate/環={g_r / max(r_ring, 1.0):.3f} "
                                f"視線俯角 γ={los:.1f}° "
                                f"({'動態 θmax=%.0f° Φ=%.2f' % (self.gate_theta_max, self.gate_phi) if self.gate_theta_max > 0 else '絕對 gate_r'})")
                    if self.strike.get(i):
                        self._pn_step(i, now)
                thru = self.thru[self.slot_id[i]]   # 穿越瞄準點(T 後方)
                # 追蹤最近水平距(遙測 + 真值基準)。impact / missed 鎖存當下
                # 凍結,沒有收斂窗(見 _track_dmin / _metrics_frozen)。
                # ⚠️ 位置很重要:必須在**同一個 tick 的鎖存判定之前**呼叫,
                #    鎖存那一筆取樣才會被算進 dmin。挪到鎖存之後 = 少一筆。
                self._track_dmin(i, d, now)
                # 一旦進入 switch_r 就「鎖定穿越」(committed):此後永遠瞄穿越點,
                # 不因穿過後 d 回升 > switch_r 又翻回瞄 T → 修掉繞圈 re-attack。
                if not self.commit.get(i) and d < self.switch_r:
                    self.commit[i] = True
                    # ⚠️ 穿越點要放在「本機當下位置 → T」的延長線上,不能用 slot
                    #    徑向上那個固定點。飛機若還偏離徑向 δ,瞄固定 thru 的那條
                    #    直線會從 T 旁邊 δ·overshoot/(overshoot+switch_r) 處通過
                    #    (本組參數 = 0.5δ)—— 等於在最後 300 m 把辛苦收斂掉的
                    #    橫偏又還回去一半。沿「本機→T」延長,這條線**定義上通過 T**,
                    #    幾何誤差 0,而且一樣是「穿過去」不會被捕獲進 loiter。
                    b = bearing(self.pos[i][:2], self.T)
                    self.thru_dyn[i] = dest(self.T[0], self.T[1], b,
                                            self.overshoot)
                # 瞄準點與其「線上高度」必須成對決定:只換瞄準點卻沿用舊高度,
                # 會把 45° 下滑線壓平成 ~9°(切換瞬間目標變遠、高度沒跟著降)。
                if self.commit.get(i):
                    aimpt = self.thru_dyn.get(i, thru)
                    alt = self._line_alt(i, -self.overshoot)
                elif self.gate_profile and self.pn_on and d > self.gate_r:
                    # 交接盒剖面:航點段只負責把飛機送到「距 T = gate_r、
                    # 離紅心地表 gate_h」那一點,不要沿 10° 下滑線一路降到地面。
                    # 這樣 PN 是從 atan(gate_h/gate_r) 的視線俯角接手,
                    # 放大倍率 1/tan(俯角) 因此變小。瞄準點放在**本機自己的徑向上**
                    # (由 T 沿「T→本機」方位射出 gate_r),航線不會被拉歪。
                    b = bearing(self.T, self.pos[i][:2])
                    aimpt = dest(self.T[0], self.T[1], b, self.gate_r)
                    alt = self._zg(i) + self.gate_h
                else:
                    aimpt = self.T; alt = self._line_alt(i, 0.0)
                # 衝擊判定:進 impact_reach,或已 committed 且通過最近點(d 回升)→ 記為穿越
                # ⚠️ `i not in self.missed` 是互斥鎖的另一半:逃逸機在幾百公里外
                #    繞回來重穿紅心時,不可以被記成命中(SW1 的 U2/U6 就是這樣把
                #    散布污染成 11950 s 的)。miss 就是 miss。
                passed = self._passed(i, d)
                if not self._metrics_frozen(i) \
                        and (d < self.impact_reach or passed):
                    self.impact[i] = now
                    # 以「離紅心地表多高」回報:真正砸中 = 0
                    # ⚠️ hit_alt / hit_d / dmin 三個都在**這一刻**定案:_track_dmin
                    #    已經在同一個 tick 的前面吃過這一筆 d,之後 _metrics_frozen
                    #    就把它凍住。三個數字同屬一個時刻,對得起來。
                    #
                    # ⚠️⚠️ 2026-08-10 修:這裡原本是 `self.pos[i][2] - self._zg(i)`,
                    #    也就是 **HOME 相對**基準;而 CSV 的 agl 欄自 2026-08-07 起
                    #    走 `_agl()` 的 **AMSL** 基準。兩者只在 HOME 穩定時逐位元相同 ——
                    #    正是本檔自稱最高風險的那一類「兩套基準悄悄分岔」,而且
                    #    2079 附近的註解還宣稱兩者同基準(已一併更正)。
                    #
                    #    後果不是小數點:飛控重設 HOME 實測平移 **32.3 m**
                    #    (見 `_agl` 的事故紀錄),而 too_high 的門檻 impact_alt_ok 只有
                    #    **30 m** ⇒ 一次真落地會被寫成「高空掠過」,
                    #    反向的 HOME 跳變則會把一次 40 m 的掠過寫成「全員命中」——
                    #    那正是 hit-criterion-ignores-altitude 那條教訓要防的事,
                    #    只是從另一條路走進來。
                    #
                    #    ⚠️ 這一發是**不可逆鎖存**:`_metrics_frozen` 之後永不更新,
                    #      整趟的驗收結論就由那一瞬間的高度讀數定案。
                    #    ⚠️ AP3/AP5 沒被污染是**運氣**:兩趟的 HOME 重設都發生在
                    #      命中之後約 1 秒(撞海觸發的),不是因為設計上擋得住。
                    #    ⇒ 改用 `_agl(i)`,和 CSV 的 agl 欄同一個基準、對 HOME 免疫。
                    #      HOME 穩定時兩式逐位元相同,所以既有資料不受影響。
                    self.hit_alt[i] = self._agl(i)
                    self.hit_d[i] = d
                    if i in self.truth and i in self.zg_amsl:
                        self.truth_hit[i] = self.truth[i][2] - self.zg_amsl[i]
                    self.get_logger().info(
                        f"UAV{i} 命中紅心 @+{now-self.dive_go:.2f}s "
                        f"(鎖存當下 {d:.0f}m、最近水平距 "
                        f"{self.dmin.get(i, float('nan')):.1f}m(此刻凍結) "
                        f"命中高度 {self.hit_alt[i]:+.0f}m(離紅心地表) "
                        f"gs {self.gs.get(i,0):.0f}m/s)")
                # ── M1/M2:miss 出口 ────────────────────────────────────
                # ⚠️ 放在衝擊判定**之後** ⇒ 同一個 tick 內 impact 優先於 miss,
                #    互斥鎖不會把「這一 tick 剛好命中」的機判成逾時。
                # ⚠️ 也放在 `if not released` **之前** ⇒ 從未放行、卡在環上不動的
                #    機一樣走得到出口(否則 continue 會讓它永遠不計入 done)。
                self._miss_check(i, d, now)
                if i in self.missed:
                    # 已判 MISS:停發俯衝指令,改用既有的守點路徑原地盤旋。
                    mode = "missed"
                    mp = self.miss_pt.get(i)
                    if mp is not None:
                        self._goto(i, mp[0], mp[1], mp[2], self.vmin,
                                   radius=self.hold_radius)
                    stat[i] = {"mode": mode, "d": round(d),
                               "why": self.miss_why.get(i, "")}
                    self._row(now, i, d, mode, self.T)
                    continue
                if not released:
                    # 還沒輪到:在自己的環點上等,保持高度與對正
                    # ⚠️ 等待段有兩種速度,不能共用一個。環外還有 30 km 要轉場,
                    #    用守環的 vmin(70)飛要 430 s,整趟會逾時;但真的到了環上
                    #    又必須慢下來,快速繞環會把站位精度做壞(迴轉半徑 ∝ V²)。
                    far = d > self.ring_of(i) * (self.ring_gate_frac + 0.4)
                    mode = "runin" if far else "wait"
                    # ⚠️ 這裡**兩種語意都有**,不能共用一個半徑:
                    #    far(runin)= 還在 30 km 外轉場 ⇒ 要**飛過去** ⇒ goto_radius
                    #    近(wait) = 已在環上等待     ⇒ 要**繞**     ⇒ hold_radius
                    # ⭐ S8:wait(已在環上等)吃 hold_alt / hold_v;
                    #    runin(還在 30 km 外轉場)維持 perch 與 vmax 不動 ——
                    #    轉場段降高沒有意義,而且會和爬升交互作用。
                    # ⭐ T3:wait 段用蛇形吸收「太早到」的時間誤差。
                    #    weave_time_sync=False(預設)⇒ th=0 ⇒ 瞄環點,逐位元同舊行為。
                    th = 0.0 if far else self._weave_theta_of(i, d, now)
                    if th > 0.0:
                        # 以環點方位為基準左右擺;換向由 weave_period 決定。
                        sign = 1.0 if math.sin(2 * math.pi * (now - self.t0)
                                               / self.weave_period) >= 0 else -1.0
                        rp = self.ring_point_of(i)
                        bs = bearing(self.pos[i][:2], rp)
                        aim = dest(self.pos[i][0], self.pos[i][1],
                                   bs + sign * th, max(300.0, 0.5 * d))
                        self._goto(i, aim[0], aim[1], self.hold_alt_of(i),
                                   self.hold_v_of(), radius=self.goto_radius)
                    elif self.ring_pace and not far:
                        # ⭐⭐ V1(2026-08-16):**用速度取代盤旋等待**。
                        #
                        # ⚠️⚠️ 舊行為在環附近用 `hold_radius` ⇒ 飛機**繞圈**等待。
                        #    實測代價(同一段 3.24 km 的直線距離):
                        #      直線穿過  21–25 s、航跡 3176–3224 m、迂迴 **0.99×**
                        #      盤旋等待 114–200 s、航跡 9621–15332 m、迂迴 **3.0–4.7×**
                        #    而兩架的時刻差在這 3 公里裡從 **4.4 s 放大到 40.2 s**
                        #    (30 km→10 km 完全沒有惡化,離散全在這一段)。
                        #    ⇒ 盤旋是用成本 100+ 秒的機制去吸收 4.4 秒的誤差,
                        #      而吸收的過程本身製造了 40 秒的新誤差。
                        #    更糟的是:從盤旋切出的相位不受控 ⇒ 放行航向 σ 隨機
                        #    (實測中位 **151°**,機頭幾乎背對紅心)⇒ 俯衝迂迴 1.2–2.7×。
                        #
                        # 改成:**直線飛向環點**(goto_radius 不是 hold_radius),
                        # 用**速度**把抵達時刻調到放行時刻。
                        #   吸收能力(3.24 km,95–157 m/s)= 13.5 s,需求 4.4 s ⇒ 餘裕 3 倍。
                        #
                        # ⚠️ n=8 檢查:速度調節**每架獨立**、不搶共用資源 ⇒ 天然 scale。
                        #    對照:盤旋在 n=8 是七架擠同一個環,空間上放不下
                        #    (環周長 8.2 km、迴轉半徑 2294 m)。
                        #
                        # ⚠️ 已知風險(LM 審查):低速進入 ⇒ 動壓低 ⇒ 突風佔比上升,
                        #    **可能**放大俯衝耗時變異。實測 `耗時 vs 進入速度
                        #    r = +0.437`(符號與擔憂相反,疑似與 σ 共線)。
                        #    ⇒ 既不能宣稱無害也不能宣稱有害,**V1 必須監看這一項**。
                        #    且實測進入速度只到 97 m/s,沒測過 95 附近,不可外推。
                        # ⚠️⚠️⚠️ 參考量**不可以**用 `dive_T` 的時間裕度。
                        #    RRB(2026-08-16)整批 8 趟白跑就是這樣:第一版寫
                        #    `slack = (dive_T − now) − t_go`,而**飛機永遠是遲到的**
                        #    (見 memory/predictor-underestimates-dive:e_s 從放行
                        #    第一拍就 ≤0,27/27 單邊)⇒ `slack > 0.5` 幾乎不成立
                        #    ⇒ 每次都掉到「全速趕」⇒ v_cmd 恆等於 vmax=150,
                        #    十四架次無一例外。**分支開著,但邏輯從未執行。**
                        #    ⚠️ 這與 ITCG 沒武裝是**同一個根因**,而我明明量過、
                        #    還寫進記憶了,卻把 V1 建在同一個壞掉的參考上。
                        #
                        # ⇒ 改成**相對參考**:各機互相比,不看絕對時刻表。
                        #    目標 = 全隊裡預估抵環最晚的那個時刻;每架把速度調到
                        #    「在那個時刻抵環」。最慢的那架維持 vmax,其餘放慢等它。
                        #    ⚠️ 這條**完全不依賴 dive_T**,所以時刻表壞掉也不影響。
                        #    ⚠️ n=8:max 是對全隊取,而全隊位姿本來就在廣播 ⇒ 可 scale。
                        #    ⚠️⚠️ 分母**必須**是 vmax(能力值),絕不可以用實測地速
                        #    self.gs —— 那會形成正回饋而棘輪到底:
                        #      最遠那架算出 v = d_max / t_ring,而 t_ring ≥ d_max/gs_自己
                        #      ⇒ v ≤ 自己現在的速度 ⇒ 下一拍更慢 ⇒ t_ring 更大 ⇒ 再更慢。
                        #      全隊一路滑到 vmin,而「釘在 vmin」正是
                        #      [[dive-pace-saturates-vmin]] 記錄過、**同時**毀掉散布與
                        #      終端速度的那個坑。
                        #    用 vmax ⇒ t_ring = max_j(d_j)/vmax,只跟幾何有關 ⇒ 無回饋,
                        #    且最遠那架恰好得到 v=vmax(不被自己拖慢)。
                        #    ⚠️ 閘門要用**各自**的 ring_of(q):perch_step 開著時每架環半徑
                        #    不同,拿本機的閘門去算別人的剩餘距離會系統性偏掉。
                        gate = self.ring_of(i) * self.ring_gate_frac
                        d_go = max(0.0, d - gate)      # 到放行閘門還有多遠
                        eta = [max(0.0, dist_m(self.pos[q][:2], self.T)
                                   - self.ring_of(q) * self.ring_gate_frac) / self.vmax
                               for q in self.pos]
                        t_ring = max(eta) if eta else 0.0
                        if t_ring > 0.5 and d_go > 1.0:
                            v = max(self.vmin, min(self.vmax, d_go / t_ring))
                        else:
                            v = self.vmax
                        self._goto(i, *self.ring_point_of(i), self.hold_alt_of(i),
                                   v, radius=self.goto_radius)
                    else:
                        # ⭐⭐ S2(2026-08-18):`wait_goto_radius` —— wait 段也飛過去,不繞圈。
                        #
                        # ⚠️⚠️ RRD(8 趟 16 架次)的真根因,而且它是一個**指令從未送達**
                        #    的 bug,不是飛行問題:
                        #    runin→wait 切換時,這一發 `_goto` 的 lat/lon/alt **逐位元
                        #    相同**(ring_point_of 不變;hold_alt=0 ⇒ hold_alt_of 就是
                        #    perch_of),只有 radius 由 30 變 1300。而 `_goto` 的變更
                        #    偵測器(見 3055-3057)存的是 `wp_pt=(lat,lon,alt)` ——
                        #    **radius 不在裡面** ⇒ moved=False ⇒ 那發 DO_REPOSITION
                        #    **根本沒有送出去**,飛機掛著舊的 param3=30 繼續直飛。
                        #    hold_radius 只有在下一個 30 s keepalive 補發時才生效,
                        #    ⇒ 一架會不會起彎,取決於它進 wait 時距 30 s 格點還有多久,
                        #      **逐趟隨機**。
                        #
                        #    實測驗證(16 架次):用 keepalive 相位預測「會不會起彎」
                        #    命中 15/16;5 架起彎者的「預測補發 → 實測起彎」殘差是
                        #    +2.0/+0.4/+0.9/+2.2/+0.8 s(= 滾轉建立時間),無一早於預測。
                        #    ⇒ 這一批其實是**兩個母體**:
                        #        沒吃到補發 11 架 → 直線(弧擬合 2.2萬–58萬 m)→ 命中 10/11
                        #        吃到補發    5 架 → 硬彎(1833–2965 m)      → 命中 0/5
                        #
                        # ⚠️⚠️ **成功的指紋是「指令的缺席」。** 開關打開後 wait 與 runin
                        #    要求同一個半徑 ⇒ 9216 m 當拍**不會**送出任何 DO_REPOSITION。
                        #    在 log 裡找確認指令是找不到的,那是對的,不是失敗。
                        #    ⇒ 判機制有沒有生效要看 CSV 的 `req_radius_m`(本拍**要求**的
                        #      半徑),不能看「送出去的」——後者在開關關閉時同樣是 30。
                        #
                        # ⚠️ 刻意**不**去修 `_goto` 的偵測器(把 radius 併進 wp_pt):
                        #    那會讓開關關閉時 16/16 全部變成起彎組,純風險零收益。
                        # ⚠️ n=8:逐機獨立、不搶共用資源;hold_radius 仍保留給
                        #    ALIGN 的 hold_ring 與 missed 的守點盤旋(那兩處是真的要繞)。
                        self._goto(i, *self.ring_point_of(i),
                                   self.perch_of(i) if far else self.hold_alt_of(i),
                                   self.vmax if far else self.hold_v_of(),
                                   radius=(self.goto_radius
                                           if (far or self.wait_goto_radius)
                                           else self.hold_radius))
                    stat[i] = {"mode": mode, "d": round(d),
                               "slack": round((self.dive_T - now)
                                              - self._tgo_of(i, d), 1)}
                    self._row(now, i, d, mode, self.T)
                    continue
                if i in self.impact:
                    # done 的計數已移到迴圈外的 _done_count()(見那裡的說明)
                    mode = "expended" if self.expend else "impact"
                    if not self.expend:
                        # 保留機體 → 續飛穿越點 egress(俯衝速度),不繞 T
                        self._goto(i, *thru, alt, self.dive_speed)
                    # expend_on_impact:命中即視為消耗,不脫離、不拉起。
                    # ⚠️⚠️ 註解再訂正(2026-08-06):2026-08-05 那次「訂正」訂反了。
                    #    上面的 PN 呼叫 `if self.strike.get(i): self._pn_step(i, now)`
                    #    縮排在 `if self.pn_on and self.fc and released
                    #    and not self._metrics_frozen(i):` **裡面**,而
                    #    `_metrics_frozen` 就是 `i in self.impact or i in self.missed`
                    #    ⇒ **鎖存之後 PN 就停了**,不會再每 tick 發姿態指令。
                    #    CSV 上 impact 之後那段是自由飛行(加上本輪新增的觸水
                    #    強制 disarm ⇒ 無動力)。要改這裡的行為前請先重讀守門條件,
                    #    不要照舊註解推理。
                else:
                    # 俯衝砸向 T:指令上限 dive_speed(> 頂速 → TECS 壓機頭俯衝加速);
                    # 近機(會早到)v_req 偏低 → 放慢到最低 0.62·dive_speed 等後機(仍在俯衝)
                    # 俯衝段也要做時間對齊(這是 STOT 的 S):近機必須能「慢下來等」。
                    # ⚠️ 舊版速度下限是 0.62·dive_speed(=87 m/s,比可達速還高)→ 全員
                    #    永遠全速 → 完全沒有等待餘裕 → 命中散布 26.7s。下限放到 vmin。
                    t_rem = self.dive_T - now
                    v_req = d / max(t_rem, 0.05)
                    v = max(self.vmin, min(self.dive_speed, v_req))
                    # ⚠️⚠️ **不要**在這裡用 dive_full_thr 覆寫成 dive_speed。
                    #    實測 AME1 反證:改成 v = dive_speed(290 EAS)之後
                    #    散布 62.7 → 147.3 s、地速 184.3 → 161.8、6/6 → 5/6。
                    #    290 EAS 遠超機體頂速 165.7,TECS 追一個追不到的目標會
                    #    持續壓機頭換速度,把俯衝幾何弄亂。
                    #    這一行是**速度指令**不是油門;全油門的執行點在
                    #    _pn_cmd 的 thr=1.0。兩者不是同一件事,我曾誤判為
                    #    「只關了一半」。min(dive_speed, v_req) 多數時候給的是
                    #    v_req —— 那是依時刻表算的合理值,不是硬煞車。
                    mode = "dive"
                    # 鎖存:整段俯衝只發一次航點,讓 ArduPlane 把 prev_WP→next_WP 的
                    # 高度斜坡整條飛完 = 真正的直線俯衝。
                    # 但若飛機浮在下滑線上方 > line_tol,就「有節制地」重發一次:
                    # 重發會以當下位置為新起點 → 建立更陡的斜坡把它拉回線上。
                    # (無限制重發 = 每 tick 重設斜坡 = 原本掉不下來的病根,故加最小間隔。)
                    key = "thru" if self.commit.get(i) else "T"
                    # ⚠️ 這條「我離下滑線多高」的判斷式必須用**本機自己量到的**紅心
                    #    地表(_zg),不能用全隊共用的 ground_alt 常數 —— _line_alt
                    #    算瞄準高度時用的是 _zg,兩邊不一致的話重發閘門就對不準:
                    #    六機 z_g 散布 16.6 m(−38.72 ~ −55.31),ground_alt 卻是 −48,
                    #    等於各機的門檻被平移了 −7 ~ +7 m。
                    z_line = self._zg(i) + self.dive_alt_agl + self.dive_layer * (
                        self.slot_id[i] - (self.n - 1) / 2.0) + \
                        d * math.tan(math.radians(self.dive_angle))
                    # ⚠️ 交接盒剖面下不可以做這個重發:那條 z_line 是 10° 下滑線,
                    #    而交接盒剖面**本來就是要飛在它上面**,判斷式會一路成立 →
                    #    每 resend_dt 秒重發一次 → 高度斜坡不斷被重錨(就是
                    #    d7a377a 修掉的那個病)。
                    gate_leg = (self.gate_profile and self.pn_on
                                and d > self.gate_r)
                    if (not gate_leg
                            and self.pos[i][2] - z_line > self.line_tol
                            and now - self.wp_t.get(i, 0.0) > self.resend_dt):
                        key = f"{key}#{int(now)}"        # 換 key → 觸發一次重發
                    if not self.strike.get(i):
                        self._goto_latched(i, *aimpt, alt, v, key=key)
                stat[i] = {"mode": mode, "d": round(d), "alt": round(alt),
                           "z": round(self.pos[i][2]), "gs": round(self.gs.get(i, 0))}
                self._row(now, i, d, mode, self.T)
            self._csv.flush()
            done = self._done_count()      # = impact ∪ missed(見 _done_count)
            if done == self.n and self.done_t is None:
                # ⚠️ impact_settle 是**續錄**的秒數,不是量測窗。dmin / hit_alt
                #    早在各自鎖存當下就凍住了(見 _track_dmin),多跑這幾秒不會
                #    改變摘要行的任何數字 —— 它買的是 CSV 與 cmdlog 上那段
                #    「命中之後」的軌跡,事後畫圖與歸因要用。
                # ⚠️⚠️ 上一版這裡宣稱多跑幾秒能讓 dmin「走完最近點」。那是錯的:
                #    真正會被多收進來的是機體觸海之後爬起來的第二趟(SW1 的 U3
                #    在鎖存後 24.1 s 再穿一次)。要更細的最近點請用 stot_score,
                #    不要靠延長現場量測窗。
                self.done_t = now
            if done == self.n and now - self.done_t >= self.impact_settle:
                for ln in self._impact_summary():
                    self.get_logger().warn(ln)
                # ⚠️ 順序:先 DONE(既有語意「這一發飛完了」,摘要行已印),
                #    常駐服務再往下轉回 STANDBY。批次航次到這裡就停,
                #    行為與 2026-08-05 之前一字不差。
                self.phase = "DONE"
                if self.standby_mode:
                    self._to_standby("這一發全員結案(impact ∪ missed)")

        self.status_pub.publish(String(data=json.dumps(
            {"phase": self.phase,
             "T_arr": round(self.T_arr - self.t0, 1) if self.T_arr else None,
             "uavs": stat})))


def main():
    rclpy.init()
    node = TotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
