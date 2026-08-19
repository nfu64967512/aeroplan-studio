"""Task 3 — wingman node (UAV 2-5) — formation flying.

預設 = **極座標 V**：訂閱 /swarm/whoLeader（當前長機 gps + 航向 + id），算出「相對長機
航向固定方位、固定距離」的 slot → V 隨長機轉向旋轉、僚機永遠排在長機後方（後左/後右）。
長機由 whoLeader.id 決定，備援接管時 id 自動換成 UAV6、僚機跟著繞新長機重組。
use_biomimetic:=true → 仿生 V（訂閱全機位置、相鄰追蹤 + 推拉 + 障礙分裂繞行）。

固定翼 Gotcha 5：移動指令一律走 send_guided_waypoint（~25s 一筆），不可用 setpoint。

Topics
  sub  /swarm/whoLeader       std_msgs/String   {id,lat,lon,alt,heading}  極座標 V 輸入
  sub  /uav{j}/mavros/global_position/global  sensor_msgs/NavSatFix  (biomimetic 全機)
  sub  /uav{master}/geopose   geographic_msgs/GeoPoseStamped   (固定 slot 後備 / 長機定位)
  sub  /swarm/master_switch   std_msgs/String                  new master id
  sub  /swarm/rtl             std_msgs/Bool                    formation RTL toggle
  sub  /swarm/obstacles       std_msgs/String   JSON [[lat,lon,radius],...]（可選 → 分裂）
  out  send_guided_waypoint(lat,lon,alt)  (real mode)          formation target
"""
import json
import os
import math

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from sar_swarm.strike_authority import StrikeAuthority
from sar_swarm.geodesy import (dist_m, polar_slot, speed_for_error,
                               speed_for_along_error,
                               advance_along_heading, EARTH_M_PER_DEG,
                               calculate_projected_ground_speed,
                               gamma_for_climb_rate)
from sar_swarm.phase_gate import LatchedAltGate, MonotonicRamp
from sar_swarm.master_role import SWITCH_QOS, HEARTBEAT_QOS
from sar_swarm.biomimetic_formation import (
    BiomimeticFormation, FormationParams, Agent)

# 後備固定 slot（無 whoLeader 時用，沿用原星型偏置）
SLOTS = {2: (-300.0, -200.0), 3: (-600.0, -400.0),
         4: (-300.0, 200.0), 5: (-600.0, 400.0)}

# 極座標 V（隨長機航向旋轉）：每架僚機 (rank 1=近/2=遠, side +1=左翼/-1=右翼)。
# UAV1 長機在 V 頂點、UAV6 在 V 尾（見 backup_node），合為 6 機一個 V。
V_RANK_SIDE = {2: (1, +1), 3: (2, +1), 4: (1, -1), 5: (2, -1)}

# 錐形（formation_shape=CONE）：長機在錐尖,四架僚機均勻分佈在後方同一個圓周上,
# UAV6 留在錐軸上更後方(見 backup_node)。角度以「錐體橫截面」量:0°=正上方,
# 順時針。⚠️ **垂直分量是錐形之所以是錐的關鍵** —— 只有水平圓的話,從側面看仍是
# 一條線,視覺上和平面 V 沒有差別。
CONE_PHASE_DEG = {2: 0.0, 3: 90.0, 4: 180.0, 5: 270.0}


def v_rank_side(uav_id, n_uavs):
    """(rank, side) —— 字典命中就用字典,未命中才由**序位**推。

    ⚠️⚠️ 2026-08-07:原本是 `V_RANK_SIDE.get(uav_id, (1, 0))`,那個 (1, 0) 的
       靜默預設是**同點疊飛**的來源:
         n=7 ⇒ UAV6 未命中 ⇒ (1,0) ⇒ bearing_off = 180 + 0×角 = 180、
               distance = 1×v_spacing ⇒ 正後方 4 km、零側偏,**無任何 log**
         n=8 ⇒ UAV6 與 UAV7 **都**退回 (1,0) ⇒ 兩架被指到**同一個經緯度**,
               唯一分隔是 (uav_id-2)×layer_sep 的 15 m 垂直差,
               而 formed_radius 是 200 m(水平)⇒ **系統判定「編隊成形」**。
    ⚠️ 只在字典未命中時才走公式,是為了讓 n<=6 **逐位元不變** ——
       序位公式的排序與這份手訂字典不同(字典是 2→(1,+1) 3→(2,+1) 4→(1,-1)
       5→(2,-1),先左翼由近到遠再右翼;序位公式會給左右交錯),
       改用公式會靜默改變既有六機的隊形。
    """
    hit = V_RANK_SIDE.get(uav_id)
    if hit is not None:
        return hit
    # 未命中:用「這架在僚機序列裡的第幾個」推,左右交錯、由近到遠
    idx = max(0, uav_id - 2)                 # UAV2 → 0
    return idx // 2 + 1, (+1 if idx % 2 == 0 else -1)


def cone_phase_deg(uav_id, n_wingmen):
    """錐形相位角。未命中時**均分圓周**,而不是全部退回 0°(那會三架重合)。"""
    hit = CONE_PHASE_DEG.get(uav_id)
    if hit is not None:
        return hit
    k = max(1, n_wingmen)
    return 360.0 * (max(0, uav_id - 2) % k) / k


class WingmanNode(Node):
    def __init__(self):
        super().__init__("wingman_node")
        self.declare_parameter("uav_id", 2)
        self.declare_parameter("master_id", 1)
        self.declare_parameter("n_uavs", 6)
        self.declare_parameter("use_mavros", False)
        # False = 簡單星型：僚機以固定 slot 跟「當前長機」的 GeoPose（長機切換沿用
        # /swarm/master_switch）；True = 仿生 V（相鄰追蹤）。
        self.declare_parameter("use_biomimetic", True)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("auto_takeoff", True)
        self.declare_parameter("takeoff_alt", 50.0)
        self.declare_parameter("launch_type", "runway")   # "runway" | "catapult"
        self.declare_parameter("formation_alt", 50.0)
        self.declare_parameter("layer_sep", 15.0)
        self.declare_parameter("wp_interval", 1.0)     # 157 m/s 下每秒走 157 m;3 s 的量化就是 471 m
        self.declare_parameter("wp_move_thresh", 40.0)  # 25 m 在 157 m/s 下永遠觸發,形同無效
        self.declare_parameter("origin_lat", 22.60)
        self.declare_parameter("origin_lon", 120.00)
        # ── biomimetic 參數 ──
        self.declare_parameter("spacing_m", 250.0)
        self.declare_parameter("sep_radius_m", 60.0)
        self.declare_parameter("obstacle_margin_m", 80.0)
        self.declare_parameter("formation_shape", "V")
        # ── 極座標 V 參數（隨長機航向旋轉）──
        # ── B-500 寬間距觀測編隊 ────────────────────────────────────────
        # rank 間距(僚機離長機 = rank × 此值)。舊值 120 是泡棉機的緊密編隊;
        # 高動態分佈式海洋觀測要 2–3 英里(3218–4828 m)的松耦合間距。
        # 預設取區間中位 4000 m ⇒ rank1 4 km、rank2 8 km。
        # ⚠️ 這個值不再由「追趕權限」反推,它是**任務需求**(觀測涵蓋寬度)。
        #    代價要正視:追趕權限只有 26 m/s 地速,收斂 4 km 的初始誤差需要
        #    ~154 s(理想)/ ~250 s(考慮加速度受限)。航程必須夠長。
        self.declare_parameter("v_spacing_m", 4000.0)
        # V 半角。⚠️ 寬間距下這個角度決定觀測**涵蓋寬度**而不只是隊形外觀:
        #    rank2 的側向偏置 = 2·v_spacing·sin(θ)
        #      θ=25° → 3381 m    θ=35° → 4589 m    θ=45° → 5657 m
        #    六機的總涵蓋寬度 = 2 × 該值(左右對稱)。35° 給 9.2 km 掃幅。
        #    數值精度:4–8 km 的偏移換算成緯度約 0.036–0.072°,float64 有 ~1e-11°
        #    的解析度(≈1 µm),完全無精度問題;MAVLink 的 int32 1e7 格式解析度
        #    是 1.1 cm,也遠優於需求。真正的限制是 flat-earth 近似的曲率誤差
        #    (見 _slot_precision_check),8 km 處約 1.6 m —— 對 200 m 的到位門檻
        #    可忽略,但已加開機自檢把它印出來,不要靠假設。
        self.declare_parameter("v_angle_deg", 35.0)
        # ── 松耦合鏈路容錯:whoLeader 丟包時的 Dead Reckoning ──
        # dr_hold: 超過這麼久沒收到就開始外推(使用者指定 5 s)
        # dr_max : 外推的上限,超過就承認失聯、退回固定 slot。
        #   取 45 s:以 157 m/s 地速,45 s 的直線外推誤差主要來自「長機是否轉彎」。
        #   本任務全程直線,若長機在斷訊期間以 60° 傾角轉 10°,45 s 後橫向偏差
        #   約 V·(1−cos10°)·t ≈ 107 m —— 仍小於 200 m 的到位門檻。
        #   再長就不該假裝知道長機在哪。
        # ── 動態爬升率限幅(見 _climb_authority)──
        # climb_gate_hi: slot 誤差超過此值就壓爬升率(使用者指定 1 km)
        # climb_cap_lo : 大落後時的爬升率上限(使用者指定 1.5 m/s)
        # climb_cap_hi : 進 catch_radius 後恢復的名目爬升率
        # ⭐⭐ 高度閘門(與 master_node 的 cruise_alt_threshold 對齊)。
        #    爬升段僚機只用**最佳爬升空速**飛向目標區,不做 slot 誤差的速度補償 ——
        #    否則會在爬升中被要求減速到 wait_speed 90 EAS,爬升率跟著掉。
        #    越過門檻進入平飛段才開始精確的 V 隊形追趕。
        self.declare_parameter("cruise_alt_threshold", 3000.0)
        # ⭐⭐ 高度閘門的**遲滯**與**最短駐留**(見 _update_climb_phase)。
        #    裸比較 alt < thr 沒有遲滯 —— P91 實測 UAV5 在 200 s 內來回穿越
        #    3000 m 門檻 10 次,每次只低於門檻 1–23 m。巡航高度 3100、閘門 3000,
        #    餘裕只有 100 m,而追趕中的飛機本來就會拿高度換速度 ⇒ 必然抖動。
        #    hyst 要大於「正常追趕造成的下沉量」、小於「真的掉下去」。
        self.declare_parameter("cruise_gate_hyst_m", 150.0)
        self.declare_parameter("cruise_gate_dwell_s", 5.0)
        # 側向展開的混合高度帶:從閘門起算,經過這麼多公尺才展到滿角。
        # 300 m 在 12 m/s 爬升率下是 25 s —— 足夠讓速度控制器跟上 slot 的移動。
        self.declare_parameter("expand_time_s", 120.0)
        # ⭐⭐ 前置角投影(Lookahead / carrot-chasing)的前置距離 L1(m)。
        #    導引 WP = slot 沿**長機航向**再往前 L1,誤差仍對真 slot 量。
        #    ⚠️ 實測 P91/T1/U1 三趟的 `bearing_to_wp` 中位數都是 −73°~−86°:
        #       飛機整段巡航都用接近正橫的角度對著自己的目標,而 behind_deg 是
        #       90° —— 一直卡在切換邊緣。把目標往前推 L1 之後,近場的夾角
        #       ≈ atan(側偏 / L1),側偏 400 m 時是 15°,收斂成漸進切入。
        #    ⚠️ L1 太大 = 目標永遠在遠方 ⇒ 橫向收斂變慢(純追點的老問題);
        #       太小 = 回到正橫。1500 m 在 157 m/s 下約 9.6 s 前置。
        self.declare_parameter("l1_lookahead_m", 1500.0)
        # 巡航段的最大坡度(度)。誘導阻力 ∝ n² = (1/cos φ)²:
        #   15° → n²=1.07(+7%);45° → n²=2.0(+100%)。
        # 98 km 的航程足夠用 15° 慢慢展開側偏(轉彎半徑 V²/(g·tanφ) ≈ 9.4 km)。
        # ⚠️⚠️ **預設關掉(0 = 不動飛控的 ROLL_LIMIT_DEG)。**
        #    壓坡度可以省誘導阻力,但同時**拿掉了重新指向的權限**:
        #    15° 在 157 m/s 下轉彎半徑 9.4 km,一旦指錯方向要數分鐘才轉得回來。
        #    實測 X1 就是這樣把「目標落在身後」放大成 112 km 誤差。
        #    等橫向導引本身乾淨了再開,而且要配著「指向誤差」一起看。
        self.declare_parameter("cruise_roll_limit_deg", 0.0)
        # ⚠️⚠️ 側偏凍結**預設關掉**。兩次實測都是它自己製造出更大的問題:
        #      X1:凍結點沿長機航向放,落在本機身後 ⇒ behind% 99.8–100%
        #      Z1:補了 _ahead_of_nose 之後改成落在**機頭前方 500 m**,
        #          而 nose-frame 的側向分量 ≈ 0 ⇒ 橫向導引被完全移除
        #          ⇒ 四架整趟固定飛 292°(長機 259°),誤差以 63 m/s 線性發散。
        #          這正是註解裡警告過的 P81 開迴路模式。
        #    根本原因是僚機有 50–85% 的時間待在 CLIMB 相位(真的掉高度),
        #    凍結因此幾乎全程生效 —— 一個「例外路徑」變成了主路徑。
        #    ⇒ 先把能量問題解決、讓 CLIMB 相位回到少數,再談要不要凍結。
        self.declare_parameter("cross_freeze", False)
        # ⚠️⚠️ 收工時要把坡度上限**還回去**。`tot_node` 只斷言俯仰限制
        #    (TECS_PITCH_*/PTCH_LIM_*),**沒有**斷言 ROLL_LIMIT_DEG ——
        #    15° 會一路留到終端俯衝,把 PN 的橫向機動權限砍掉。
        #    預設 80 = swarm_sitl_up.sh 的 DIVE_ROLL_MAX 預設值。
        #    (swarm_ros2_down.sh 用 SIGTERM ⇒ destroy_node 會跑到;
        #     還原走**直連鏈路**,所以先被砍掉的 MAVROS 不影響。)
        self.declare_parameter("roll_limit_restore_deg", 80.0)
        # 爬升段「禁止身後航點」時,投影到機頭前方多遠(m)。
        self.declare_parameter("trail_ahead_m", 500.0)
        # 目標點與機頭夾角超過此值就算「在身後」,不下發、改投影前方。
        self.declare_parameter("behind_deg", 90.0)
        self.declare_parameter("best_climb_eas", 95.7)   # √(T/3c),THR_MAX=21
        self.declare_parameter("climb_gate_hi", 1000.0)
        self.declare_parameter("climb_cap_lo", 1.5)
        self.declare_parameter("climb_cap_hi", 5.0)   # 解除限制後的名目爬升率
        self.declare_parameter("climb_gamma_max", 5.0)  # 大落後時的航跡角上限(度)
        self.declare_parameter("climb_eas_floor", 90.0) # 大落後時的等效空速下限
        # HOME 的絕對海拔(m AMSL)。密度比只看絕對高度,而 MAVROS 的
        # position() 回的是相對 HOME 的高度 —— 兩者要相加才是 AMSL。
        # ⚠️ 本專案的 HOME 在山區(1839–2673 m AMSL),用相對高度算 σ 會差很多。
        self.declare_parameter("home_amsl", 0.0)
        self.declare_parameter("dr_hold_s", 5.0)
        self.declare_parameter("dr_max_s", 45.0)
        self.declare_parameter("cone_half_deg", 30.0)  # 錐形半頂角（formation_shape=CONE）
        # ── 編隊速度控制：落後加速追、到位回 nominal（讓快的等、慢的趕上）──
        # ⚠️⚠️ 預設值一律是 **jet plane**(B-500 機體),不是泡棉機。
        #    泡棉機的 14/10/20 m/s 曾讓僚機被命令用 1/4 速度飛、物理上不可能跟上,
        #    間距單調拉開到 13.7 km。任何沒經過 stot_trial.sh 的路徑都會吃到預設,
        #    所以預設本身必須是可飛的。
        #    機體:W=215.8 N、c=0.003168、T=4.142×THR_MAX(21)=87.0 N
        #    巡航 135 EAS(地速 157)/ 追趕 157 EAS / 等待 90 EAS(用滿減速權限)
        self.declare_parameter("nominal_speed", 135.0)  # 隊形維持巡航空速 (m/s)
        self.declare_parameter("catch_speed", 157.0)    # 落後時的追趕空速
        # ⭐⭐ 飛控接受 DO_CHANGE_SPEED 的**硬性區間**。ArduPlane commands_logic.cpp:
        #       else if ((speed_target_ms >= aparm.airspeed_min.get()) &&
        #                (speed_target_ms <= aparm.airspeed_max.get()))
        #     區間外直接 return false ⇒ MAV_RESULT_FAILED(4),**指令整個丟掉**。
        # ⚠️⚠️ 實測 V1:AIRSPEED_MAX=150、catch_speed=157 ⇒ **每一發追趕指令都被拒收**,
        #     而且兩條鏈路都一樣(拒收發生在飛控裡,不是傳輸層)。
        #     也就是說「落後就加速追」這條唯一有權限的縱向通道**從來沒有生效過** ——
        #     飛機只吃得到 nominal 135(≤150 才被接受)。這解釋了為什麼歷次航次
        #     都看到「速度指令正常但誤差收不掉」。
        #     加了回應檢驗才看得見(MAV_RESULT=4),先前完全靜默。
        self.declare_parameter("airspeed_min", 25.0)
        self.declare_parameter("airspeed_max", 150.0)
        self.declare_parameter("formed_radius", 200.0) # 距 slot < 此 = 到位（3 英里間距下的動態雜訊量級）
        self.declare_parameter("catch_radius", 528.0)  # 速度斜坡的比例區寬度;τ_loop = catch_r/Δ ≈ 24 s
        # slot 前饋秒數:目標 = slot 沿長機航向外推 lead×leader_speed。導引 WP 有
        # ~3-5s 有效滯後 → 高速巡航誤差地板 ≈ v×lag(27m/s 時 ~100-150m)怎麼調參
        # 都消不掉;前饋直接抵消。0 = 關閉(慢速編隊不需要)。
        self.declare_parameter("wp_lead_time", 4.0)
        self.declare_parameter("guided_min_alt", 15.0) # 高於此(m)才強制 GUIDED（修手動起飛後空盤旋）
        # ⚠️⚠️ **爬升脫離高度:低於此不下任何編隊指令,只維持 GUIDED。**
        #    RPM1(n=3 首航,2026-08-11)實測:僚機起飛後被本層以 12–14 m/s 的
        #    Dead Reckoning 指令帶著飛,而噴射機的 AIRSPEED_MIN = **25**
        #    ⇒ 每一發 DO_CHANGE_SPEED 都被飛控 MAV_RESULT=4 整發拒收
        #    ⇒ UAV3 沉回地面(rel_alt −3.96),tot_node 等不到全員、1500 s 逾時。
        #    n=2 躲過六趟,只因為長機/備長不走本節點,僚機路徑從未被執行。
        # ⇒ 起飛爬升交給飛控自己的 TAKEOFF 模式(jet_fast.parm 的 TKOFF_ALT=300),
        #   本層在脫離高度之前放手。使用者 2026-08-11 定調:
        #   「n≥3 的僚機在打擊期間也是純打擊成員」—— 這一條把同一個原則
        #   往前延伸到爬升段。
        # ⚠️ 這個閘門**失敗是安全的**:沒到高度就繼續放手,飛控照樣爬。
        #    不要拿它跟 rep_interactive.sh 的就緒閘門類比 —— 那個沒過會讓整趟逾時,
        #    而「不可達門檻」已經害過 RPB1/RPC1 兩趟。
        # ⚠️ 預設 120:高於 formation_alt(50)確保接手時不是從地面爬,
        #    又遠低於 TKOFF_ALT(300)確保到得了。
        self.declare_parameter("climbout_hold_alt", 120.0)

        g = self.get_parameter
        self.uav_id = int(g("uav_id").value)
        self.master_id = int(g("master_id").value)
        self.n_uavs = int(g("n_uavs").value)
        self.slot = SLOTS.get(self.uav_id, (-300.0, 0.0))
        self.use_mavros = bool(g("use_mavros").value)
        self.use_biomimetic = bool(g("use_biomimetic").value)
        self.v_spacing = float(g("v_spacing_m").value)
        self.v_angle = float(g("v_angle_deg").value)
        self._cruise_alt_thr = float(g("cruise_alt_threshold").value)
        self.cruise_gate_hyst = float(g("cruise_gate_hyst_m").value)
        self.cruise_gate_dwell = float(g("cruise_gate_dwell_s").value)
        self.expand_time_s = float(g("expand_time_s").value)
        self.l1_lookahead_m = float(g("l1_lookahead_m").value)
        self.cruise_roll_limit = float(g("cruise_roll_limit_deg").value)
        self.cross_freeze = bool(g("cross_freeze").value)
        self.roll_limit_restore = float(g("roll_limit_restore_deg").value)
        self._reached_cruise = False   # 曾經進過巡航段?(區分初次爬升 vs 掉高度)
        self._roll_limit_set = False
        self._freeze_logged = False
        self.trail_ahead_m = float(g("trail_ahead_m").value)
        self._climb_trail_wp = None
        self.behind_deg = float(g("behind_deg").value)
        self._cruise_trail_logged = False
        # 爬升/巡航相位:單一真相來源,每個 _fly tick 更新一次。
        # 遲滯與最短駐留的實測理由見 sar_swarm/phase_gate.py 的模組說明。
        self._gate = LatchedAltGate(self._cruise_alt_thr,
                                    self.cruise_gate_hyst,
                                    self.cruise_gate_dwell)
        self._expand = MonotonicRamp(self.expand_time_s)
        self._trail_logged = False
        self._last_eff_angle = -99.0
        self.best_climb_eas = float(g("best_climb_eas").value)
        self._climb_phase_logged = False
        self.climb_gate_hi = float(g("climb_gate_hi").value)
        self.climb_cap_lo = float(g("climb_cap_lo").value)
        self.climb_cap_hi = float(g("climb_cap_hi").value)
        self.climb_gamma_max = float(g("climb_gamma_max").value)
        self.climb_eas_floor = float(g("climb_eas_floor").value)
        self._home_amsl = float(g("home_amsl").value)
        self._climb_cap = None      # 目前生效的爬升率上限(None = 還沒設過)
        self._climb_t = 0.0
        self._dr_hold = max(0.5, float(g("dr_hold_s").value))
        self._dr_max = max(self._dr_hold, float(g("dr_max_s").value))
        # 隊形:V(平面雁型)或 CONE(立體錐形)。錐形的半頂角越大越扁、越小越細長。
        self.formation_shape = str(g("formation_shape").value).upper()
        self.cone_half = float(g("cone_half_deg").value)
        self._cone_dz = 0.0        # 錐形的垂直分量(m,上為正);V 恆為 0
        self.leader = None     # /swarm/whoLeader: {id, lat, lon, alt, heading}
        self._leader_t = -1e9  # 最後一次收到 whoLeader 的時刻(外推的起算點)
        self._dr_active = False   # 是否正在 Dead Reckoning
        # ── 內容看門狗(見 on_wholeader / _leader_estimate 的說明)──────────
        # ⚠️ _leader_move_t 的初值用**節點啟動時刻**而不是 -1e9:用 -1e9 的話
        #    開機第一筆 whoLeader 進來之前 c_age 會是 1e9,當場誤判「內容凍結」。
        self._leader_prev = None
        self._leader_move_t = self.get_clock().now().nanoseconds * 1e-9
        self._content_warn_t = -1e9
        self._content_watchdog = os.environ.get("LEADER_CONTENT_WATCHDOG", "1") != "0"
        self._dr_span = 0.0        # 本次已外推多久(秒)
        self.nominal_speed = float(g("nominal_speed").value)
        self.catch_speed = float(g("catch_speed").value)
        self.airspeed_min = float(g("airspeed_min").value)
        self.airspeed_max = float(g("airspeed_max").value)
        # ⚠️ 開機就把「追趕權限是不是假的」講清楚,不要等航次跑完才發現。
        if self.catch_speed > self.airspeed_max:
            self.get_logger().error(
                f"UAV{self.uav_id} ⚠️ catch_speed {self.catch_speed:.0f} > "
                f"AIRSPEED_MAX {self.airspeed_max:.0f} —— 飛控會**整發拒收**"
                f"(MAV_RESULT=4),追趕通道形同關閉。指令將被夾到 "
                f"{self.airspeed_max:.0f};要真的拿到追趕權限請提高 "
                f"AIRSPEED_MAX_CAP。")
        self._speed_clamped = False
        self._decel_logged = False
        self.formed_radius = float(g("formed_radius").value)
        self.catch_radius = float(g("catch_radius").value)
        self.wp_lead_time = float(g("wp_lead_time").value)
        self._guided_min_alt = float(g("guided_min_alt").value)
        self._climbout_hold_alt = float(g("climbout_hold_alt").value)
        self._climbout_done = False        # 一次性鎖存:脫離後不再回到放手狀態
        self._climbout_log_t = 0.0
        self._last_speed = 0.0
        self._last_speed_t = 0.0
        self._last_guided_t = 0.0
        self._operator_override = False  # RTL/AUTO 操作者接管中(不發 WP、不搶模式)
        # 高度分層：每架不同高，V 垂直分離、不碰撞（收斂 RTL 亦然）
        self.formation_alt = (float(g("formation_alt").value)
                              + (self.uav_id - 2) * float(g("layer_sep").value))
        self.home = (float(g("origin_lat").value), float(g("origin_lon").value))
        self._rtl = False
        self.master_fix = None
        self.fleet = {}        # {uav_id: (lat, lon)}  全隊位置（biomimetic 輸入）
        self.obstacles = []    # [(lat, lon, radius), ...]

        self.bf = BiomimeticFormation(FormationParams(
            shape=str(g("formation_shape").value),
            spacing_m=float(g("spacing_m").value),
            sep_radius_m=float(g("sep_radius_m").value),
            obstacle_margin_m=float(g("obstacle_margin_m").value)))

        # SIM fallback / 長機定位：訂閱長機 GeoPose
        self.geopose_sub = None
        self._bind_master(self.master_id)
        self.create_subscription(String, "/swarm/master_switch",
                                 self.on_switch, SWITCH_QOS)
        # whoLeader: 當前長機狀態 + 航向（極座標 V 的輸入；長機切換時 id 會變）
        self.create_subscription(String, "/swarm/whoLeader",
                                 self.on_wholeader, HEARTBEAT_QOS)
        self.create_subscription(Bool, "/swarm/rtl", self.on_rtl, 1)
        self.create_subscription(String, "/swarm/obstacles", self.on_obstacles, 1)
        # 回報自己到 slot 的誤差，讓長機決定要不要減速等隊形成形
        self.status_pub = self.create_publisher(String, "/swarm/formation_status",
                                                HEARTBEAT_QOS)

        # MAVROS 模式：訂閱全機 global fix → fleet（biomimetic 相鄰追蹤輸入）
        if self.use_mavros:
            from sensor_msgs.msg import NavSatFix
            from rclpy.qos import QoSPresetProfiles
            for j in range(1, self.n_uavs + 1):
                self.create_subscription(
                    NavSatFix, f"/uav{j}/mavros/global_position/global",
                    (lambda m, jj=j: self._on_fleet_fix(jj, m)),
                    QoSPresetProfiles.SENSOR_DATA.value)

        _mode = ("BIOMIMETIC" if self.use_biomimetic
                 else "V(polar, follows leader heading)")
        self.get_logger().info(
            f"wingman UAV{self.uav_id} {_mode} up, n={self.n_uavs}, "
            f"v_spacing={self.v_spacing:.0f}m v_angle={self.v_angle:.0f}deg")
        self._speed_units_banner()
        self._cmd_log_open()
        self._slot_precision_check()

        # real mode：飛 slot
        self.fsm = None
        if self.use_mavros:
            from sar_swarm.flight import FlightFSM
            from sar_swarm.mavros_interface import MavrosInterface
            self.mav = MavrosInterface(self, self.uav_id)
            self.fsm = FlightFSM(self, self.mav, float(g("takeoff_alt").value),
                                 auto_arm=bool(g("auto_arm").value),
                                 auto_takeoff=bool(g("auto_takeoff").value),
                                 catapult=(str(g("launch_type").value) == "catapult"))
            self._last_wp = None
            self._last_wp_t = 0.0
            self._wp_interval = float(g("wp_interval").value)
            self._wp_move_thresh = float(g("wp_move_thresh").value)
            # 打擊期間讓權給 tot_node(見 strike_authority 的事故說明)
            self._strike_auth = StrikeAuthority(self)
            self.create_timer(1.0, self._fly)
            self.get_logger().info(f"UAV{self.uav_id} real mode: fly formation slot")

    # ── 訂閱回呼 ──────────────────────────────────────────────────────────
    def _speed_units_banner(self):
        """開機自報:把每個速度指令在**兩種口徑**下都印出來。

        ⚠️ 本專案最頑固的一類錯誤是空速/地速混用,而它從來不會報錯 ——
           只會讓閉合率被系統性低估 17–30%(視高度)。指令端一律是 EAS,
           幾何端一律是地速;把兩者並列印出來,誤用當場看得見。
        """
        alt = self.formation_alt + self._home_amsl
        rows = (("nominal(隊形維持)", self.nominal_speed),
                ("catch  (落後追趕)", self.catch_speed),
                ("climb_eas_floor  ", self.climb_eas_floor))
        self.get_logger().info(
            f"UAV{self.uav_id} 速度口徑 @ {alt:.0f} m AMSL"
            f"(σ={__import__('sar_swarm.geodesy', fromlist=['x']).density_ratio(alt):.4f}):")
        for name, eas in rows:
            vg, tas, _ = calculate_projected_ground_speed(eas, alt, 0.0, 0.0)
            self.get_logger().info(
                f"    {name}  EAS {eas:6.1f}  →  TAS {tas:6.1f}  →  平飛地速 {vg:6.1f} m/s")

    def _cmd_log_open(self):
        """逐秒指令記錄器。⚠️ 絕不阻塞主迴圈:開檔失敗就靜默停用。"""
        self._cmdlog = None
        self._cmdlog_t = 0.0
        try:
            import os
            d = os.environ.get("CMD_LOG_DIR",
                               os.path.expanduser("~/fixwing_ws/runs/cmdlog"))
            os.makedirs(d, exist_ok=True)
            f = open(os.path.join(d, f"uav{self.uav_id}_cmd_telemetry.csv"),
                     "w", buffering=1)
            f.write("timestamp_ns,flight_phase,cmd_speed_eas,cmd_wp_lat,cmd_wp_lon,"
                    "cmd_wp_alt,distance_to_wp,bearing_to_wp_deg,"
                    "current_alt_amsl,current_gs" + chr(10))
            self._cmdlog = f
        except Exception as e:
            self.get_logger().warn(f"指令記錄器停用:{e!r}")

    def _cmd_log(self, phase, spd, wp, now):
        """1 Hz 寫一行。bearing_to_wp_deg 的絕對值 > 90° = 目標點在飛機身後。"""
        if self._cmdlog is None or (now - self._cmdlog_t) < 1.0:
            return
        self._cmdlog_t = now
        try:
            p = self.mav.position() if self.mav is not None else None
            me = self.fleet.get(self.uav_id)
            hdg = self.mav.heading() if self.mav is not None else 0.0
            d = brg = float("nan")
            if me is not None and wp is not None:
                d = dist_m(me, wp[:2])
                b = math.degrees(math.atan2(
                    (wp[1] - me[1]) * EARTH_M_PER_DEG
                    * math.cos(math.radians(me[0])),
                    (wp[0] - me[0]) * EARTH_M_PER_DEG)) % 360.0
                brg = (b - hdg + 540.0) % 360.0 - 180.0
            gs = 0.0
            v = self.mav.velocity() if self.mav is not None else None
            if v:
                gs = math.hypot(v[0], v[1])
            self._cmdlog.write(
                ("%d,%s,%.2f,%.7f,%.7f,%.1f,%.1f,%.1f,%.1f,%.1f" + chr(10))
                % (now * 1e9, phase, spd,
                   wp[0] if wp else 0.0, wp[1] if wp else 0.0,
                   wp[2] if wp and len(wp) > 2 else self.formation_alt,
                   d, brg,
                   (p[2] if p else 0.0) + self._home_amsl, gs))
        except Exception:
            pass          # I/O 問題絕不影響控制

    def _slot_precision_check(self):
        """開機自檢:在**本機實際的 slot 距離**上量 flat-earth 近似的誤差。

        ⚠️ 寬間距(4–8 km)之下不可以再假設「1.5 km 內誤差 < 1 cm」那句話還成立。
        用 haversine 反算 polar_slot 產出的座標,比對指令距離與方位,把殘差印出來。
        數值溢出/精度漂移若真的發生,這裡會直接看到,而不是等飛出去才發現。
        """
        rank, side = v_rank_side(self.uav_id, self.n_uavs)
        d_cmd = rank * self.v_spacing
        if d_cmd <= 0.0:
            self.get_logger().error(
                f"v_spacing_m={self.v_spacing} 無效(<=0);slot 會塌到長機身上")
            return
        lat0, lon0 = 24.5, 121.2                      # 任務區代表緯度
        brg_off = 180.0 + side * self.v_angle
        slat, slon, _ = polar_slot(lat0, lon0, 0.0, d_cmd, brg_off)
        d_act = dist_m((lat0, lon0), (slat, slon))    # 專案用的同一套量測
        err = abs(d_act - d_cmd)
        lvl = self.get_logger().warn if err > 0.01 * d_cmd else self.get_logger().info
        lvl(f"slot 自檢 UAV{self.uav_id}: 指令 {d_cmd:.0f} m @ {brg_off:.0f}° → "
            f"實得 {d_act:.1f} m(殘差 {err:.2f} m = {100 * err / d_cmd:.3f}%);"
            f"側向偏置 {d_cmd * math.sin(math.radians(brg_off)):+.0f} m")
        if err > 0.01 * d_cmd:
            self.get_logger().warn(
                "⚠️ 平面近似殘差超過 1% —— 間距已大到該換球面公式,"
                "或 v_spacing_m 被設成了不合理的值")

    def _bind_master(self, master_id):
        if self.geopose_sub is not None:
            self.destroy_subscription(self.geopose_sub)
        self.master_id = master_id
        self.geopose_sub = self.create_subscription(
            GeoPoseStamped, f"/uav{master_id}/geopose", self.on_master_geopose, 10)

    def _on_fleet_fix(self, j, msg):
        if abs(msg.latitude) > 1e-6 or abs(msg.longitude) > 1e-6:
            self.fleet[j] = (msg.latitude, msg.longitude)

    def on_obstacles(self, msg: String):
        try:
            self.obstacles = [(float(o[0]), float(o[1]), float(o[2]))
                              for o in json.loads(msg.data)]
        except Exception:
            self.obstacles = []

    def on_switch(self, msg: String):
        new_master = int(json.loads(msg.data)["new_master_id"])
        if new_master == self.master_id:
            return
        self.get_logger().warn(
            f"wingman UAV{self.uav_id}: master UAV{self.master_id} -> UAV{new_master}")
        self._bind_master(new_master)
        self._last_wp = None
        self._last_wp_t = 0.0

    def on_wholeader(self, msg: String):
        try:
            d = json.loads(msg.data)
            lat, lon = float(d["lat"]), float(d["lon"])
            if abs(lat) < 1e-3 and abs(lon) < 1e-3:
                return                      # 拒收 (0,0) 長機座標，否則 slot 會算到赤道
            self.leader = {"id": int(d["id"]), "lat": lat, "lon": lon,
                           "alt": float(d["alt"]), "heading": float(d["heading"]),
                           "speed": float(d.get("speed", self.nominal_speed))}
            # 收到就重置外推:記下這一筆的到達時刻,並清掉推算狀態
            self._leader_t = self.get_clock().now().nanoseconds * 1e-9
            # ── ⭐⭐ 內容新鮮度(2026-08-07 ANM1 事故)────────────────────
            # 上面那行是**無條件**刷新的 —— 它量的是「訊息有沒有到」,
            # 不是「內容有沒有更新」。ANM1 的長機 MAVROS 死掉之後,
            # master_role 對定位快取沒有年齡檢查,繼續 10 Hz 廣播**同一組座標**
            # 448 秒;而 whoLeader 的 speed/omega 兩欄不經 MAVROS、還在變,
            # 所以訊息看起來完全正常。結果:age 恆 ≈ 0.1 s ⇒
            # _leader_estimate 的 5 s / 45 s 兩道告警**結構上永遠進不去**,
            # 僚機忠實地飛向一個永遠不動的 slot,飛到 68 km 外,全程零告警。
            # ⇒ 既有的容錯鏈防的是**斷線**,而這次是**說謊**。
            #    這裡補的就是那個盲點:位置不動就視同失聯,重用同一條鏈。
            # ⚠️ 1.0 m 的門檻要大於 GPS 抖動、小於任何真實移動(巡航 ~100 m/s
            #    ⇒ 0.1 s 就走 10 m)。LEADER_CONTENT_WATCHDOG=0 可關閉。
            if (self._leader_prev is None
                    or dist_m(self._leader_prev, (lat, lon)) > 1.0):
                self._leader_move_t = self._leader_t
                self._leader_prev = (lat, lon)
            if self._dr_active:
                self._dr_active = False
                self.get_logger().info(
                    f"長機廣播恢復(外推了 {self._dr_span:.1f} s)—— 退出 Dead Reckoning")
            self._dr_span = 0.0
        except Exception:
            pass

    def _leader_estimate(self):
        """長機狀態,失聯時以 Dead Reckoning 外推。

        ⚠️⚠️ 2–3 英里間距之下 LoRa Mesh 的丟包率與延遲呈指數上升,
        `/swarm/whoLeader` 斷幾秒是**常態**而不是異常。舊行為是直接讓
        `_v_target()` 回傳 None → 呼叫端退回固定星型 slot → 整個編隊在丟包的
        瞬間跳到另一組座標,鏈路一恢復又跳回來。那個來回本身就是散隊的來源。

        改成:斷訊 ≤ dr_hold 秒 → 用最後一筆的 heading/speed 在本地推算長機航跡,
        編隊照常維持;超過 dr_max 秒才放棄(那時多半是真的失聯,交給 backup 接管)。

        ⚠️ 只外推**位置**,不外推 heading —— 沒有轉彎率資訊時假設直線是最安全的,
        而本任務全程直線接近,這個假設剛好成立。若長機在斷訊期間轉彎,
        外推位置會偏,但偏差沿航跡方向累積得比橫向慢,恢復後幾秒內就收得回來。
        """
        L = self.leader
        if L is None:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self._leader_t
        # ⭐⭐ 見 on_wholeader 的說明:訊息一直到、內容卻凍住,是這條容錯鏈的盲點。
        #    取「到達年齡」與「內容年齡」的**大者**,讓下面既有的 5 s / 45 s
        #    兩道告警對「說謊」也生效 —— 不新增分支,重用已經寫好且驗證過的路徑。
        if self._content_watchdog:
            c_age = now - self._leader_move_t
            if c_age > age and now - self._content_warn_t > 10.0:
                self._content_warn_t = now
                self.get_logger().warn(
                    f"whoLeader 仍在到達(訊息年齡 {age:.1f}s)但**內容已凍結** "
                    f"{c_age:.1f}s —— 視同失聯。長機位置不動而它宣稱在飛,"
                    f"多半是上游 MAVROS 死了而發布端沒有年齡檢查。"
                    f"LEADER_CONTENT_WATCHDOG=0 可關閉。")
            age = max(age, c_age)
        if age <= self._dr_hold:
            return L                                  # 鏈路正常,直接用
        if age > self._dr_max:
            if self._dr_active:
                self._dr_active = False
                self.get_logger().error(
                    f"長機已 {age:.0f} s 無廣播(上限 {self._dr_max:.0f} s)"
                    f"—— 放棄外推,退回固定 slot")
            return None
        if not self._dr_active:
            self._dr_active = True
            self.get_logger().warn(
                f"長機廣播中斷 {age:.1f} s → 啟動 Dead Reckoning "
                f"(heading {L['heading']:.0f}° speed {L.get('speed', 0.0):.0f} m/s)")
        self._dr_span = age
        # 以最後一筆的航向與速度直線外推。span 用「整段斷訊時間」而不是
        # 上一 tick 到現在,這樣不論本迴圈抖動多少,推算位置都只由 age 決定
        # (無累積積分誤差)。
        v = float(L.get("speed", 0.0) or 0.0)
        lat, lon = advance_along_heading(L["lat"], L["lon"],
                                         L["heading"], v * age)
        est = dict(L)
        est["lat"], est["lon"] = lat, lon
        est["dr"] = age                               # 給下游判斷可信度
        return est

    def on_rtl(self, msg: Bool):
        active = bool(msg.data)
        if active != self._rtl:
            self._rtl = active
            self._last_wp = None
            self._last_wp_t = 0.0
            self.get_logger().warn(
                f"wingman UAV{self.uav_id}: formation RTL {'ON' if active else 'OFF'}")

    def on_master_geopose(self, msg: GeoPoseStamped):
        p = msg.pose.position
        self.master_fix = (p.latitude, p.longitude)
        # SIM 模式無 MAVROS fix → 用長機 GeoPose 餵 fleet 的長機位置
        if not self.use_mavros:
            self.fleet[self.master_id] = (p.latitude, p.longitude)

    # ── ENU 互轉（以 origin 為原點的局部平面公尺）────────────────────────
    def _enu(self, ll, origin):
        de = (ll[1] - origin[1]) * EARTH_M_PER_DEG * math.cos(math.radians(origin[0]))
        dn = (ll[0] - origin[0]) * EARTH_M_PER_DEG
        return de, dn

    def _latlon(self, enu, origin):
        lat = origin[0] + enu[1] / EARTH_M_PER_DEG
        lon = origin[1] + enu[0] / (EARTH_M_PER_DEG * math.cos(math.radians(origin[0])))
        return lat, lon

    def _slot_at(self, ref):
        """後備：ref (lat,lon) + 固定 slot 偏置 (north,east) → 目標 lat/lon。"""
        lat, lon = ref
        n, e = self.slot
        return (lat + n / EARTH_M_PER_DEG,
                lon + e / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))

    def _update_climb_phase(self):
        """推進爬升/巡航相位的 latch。**每個 _fly tick 只呼叫一次。**

        其餘消費者(_v_target 的展開角與 bearing_off、_formation_control 的
        速度、_cmd_log 的相位標籤)一律讀 `self._climbing`,不可以各自
        重新比較高度 —— 那正是 P91 的 UAV5 抖 10 次的來源,詳細實測證據
        見 sar_swarm/phase_gate.py 的模組說明。
        """
        p = self.mav.position() if self.mav is not None else None
        now = self.get_clock().now().nanoseconds * 1e-9
        climbing, changed = self._gate.update(p[2] if p else None, now)
        if changed and climbing:
            self.get_logger().warning(
                f"UAV{self.uav_id} 退回爬升段:alt {(p[2] if p else 0.0):.0f} m "
                f"已連續 {self._gate.dwell_s:.0f} s 低於 "
                f"{self._gate.exit_alt_m:.0f} m"
                f"(累計翻轉 {self._gate.flips} 次)")
        if not climbing and not self._reached_cruise:
            self._reached_cruise = True     # 之後的爬升段一律走側偏凍結
        # ⭐⭐ 進巡航段就把最大坡度壓到 cruise_roll_limit。
        #    誘導阻力 ∝ n² = (1/cos φ)²:15° 只多 7%,45° 多 100%。
        #    U1 實測:四架帶側偏的僚機掉了 1000 m 高度(最低 1523 m),
        #    而 UAV6(slot 無側偏、不需要持續帶坡度)完全不掉、誤差 560 m。
        #    98 km 航程足夠用 15° 慢慢展開(轉彎半徑 ≈ 9.4 km @157 m/s)。
        # ⚠️ 只在跨越時送一次:直連鏈路無 ACK,每 tick 送會排擠導引航點。
        if (not climbing and not self._roll_limit_set
                and self.cruise_roll_limit > 0.0 and self.mav is not None):
            if self.mav.param_set_async("ROLL_LIMIT_DEG",
                                        self.cruise_roll_limit):
                self._roll_limit_set = True
                self.get_logger().info(
                    f"UAV{self.uav_id} 巡航段坡度上限 → "
                    f"{self.cruise_roll_limit:.0f}°"
                    f"(誘導阻力 n²={1.0 / math.cos(math.radians(self.cruise_roll_limit)) ** 2:.2f}×)")
        return climbing

    @property
    def _climbing(self):
        return self._gate.climbing

    def _along_to_slot(self, slot):
        """slot 相對本機的**沿航向**分量(長機座標系)。>0 = slot 在前方。

        ⚠️ 用**長機航向**而不是本機航向 —— 本機航向會讓這個量隨自己轉動而變,
           那正是 trail 投影正回饋的同一種錯誤(見 _carrot_on_path)。
        """
        me = self.fleet.get(self.uav_id)
        L = self.leader
        if me is None or not L or "heading" not in L:
            return None
        hl = math.radians(float(L["heading"]))
        dn = (slot[0] - me[0]) * EARTH_M_PER_DEG
        de = ((slot[1] - me[1]) * EARTH_M_PER_DEG
              * math.cos(math.radians(me[0])))
        return dn * math.cos(hl) + de * math.sin(hl)

    def _ahead_of_nose(self, me, target, hdg):
        """保證 target 落在本機機頭前方:**保留側向分量,只把縱向推到前方**。

        ⚠️⚠️ 這是本專案反覆踩到的那條鐵律的共用實作:
          - 不可以整個改投影到「飛機目前指向」的前方 —— 那是開迴路直飛,
            橫向導引被完全移除(實測 P81:誤差 10573 → 28018 m,比不修還糟)。
          - 也不可以直接下發身後的點 —— ArduPlane 會轉頭回去追,
            大坡度轉彎吃掉推力(實測 P23 的三段爬升率崩落)。
          ⇒ 正解只有一個:**橫向照留、縱向夾到 ≥ trail_ahead_m**。
        """
        hb = math.radians(hdg)
        dn = (target[0] - me[0]) * EARTH_M_PER_DEG
        de = ((target[1] - me[1]) * EARTH_M_PER_DEG
              * math.cos(math.radians(me[0])))
        al = dn * math.cos(hb) + de * math.sin(hb)
        cr = -dn * math.sin(hb) + de * math.cos(hb)
        if al >= self.trail_ahead_m:
            return target                      # 已經在前方,原樣放行
        al = self.trail_ahead_m
        return (me[0] + (al * math.cos(hb) - cr * math.sin(hb)) / EARTH_M_PER_DEG,
                me[1] + (al * math.sin(hb) + cr * math.cos(hb))
                / (EARTH_M_PER_DEG * math.cos(math.radians(me[0]))))

    def restore_roll_limit(self):
        """把巡航段壓下去的 ROLL_LIMIT_DEG 還回俯衝用的值。

        ⚠️ 沒有這一步的話 15° 會留到終端俯衝 —— `tot_node` 只斷言俯仰限制,
           不碰滾轉,PN 的橫向機動就被悄悄砍掉了(而且不會有任何錯誤訊息)。
        """
        if not self._roll_limit_set or self.mav is None:
            return
        if self.mav.param_set_async("ROLL_LIMIT_DEG", self.roll_limit_restore):
            self._roll_limit_set = False
            self.get_logger().info(
                f"UAV{self.uav_id} 坡度上限還原 → {self.roll_limit_restore:.0f}°")

    def _formation_control(self, tlat, tlon):
        """速度控制 + 回報誤差 + 起飛後保持 GUIDED。
        落後 slot 越多 → 飛越快追上；到位 → 回 nominal（與長機同速維持隊形）。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        my = self.fleet.get(self.uav_id)               # use_mavros 下 = 自己經緯
        err = dist_m(my, (tlat, tlon)) if my is not None else None
        # 到位後跟長機目前速度（長機在等隊形 → 一起慢；長機巡航 → 一起巡航），落後才衝 catch
        nominal = (self.leader.get("speed", self.nominal_speed)
                   if self.leader is not None else self.nominal_speed)
        # ⭐ 高度閘門:還沒到巡航高度就只用最佳爬升空速,不做 slot 速度補償。
        #   ⚠️ 相位一律讀 _update_climb_phase() latch 過的 _climbing,
        #      不可以在這裡重新裸比較 —— 那正是 P91 抖 10 次的來源。
        p_now = self.mav.position() if self.mav is not None else None
        climbing = self._climbing
        if climbing:
            if not self._climb_phase_logged:
                self._climb_phase_logged = True
                self.get_logger().info(
                    f"UAV{self.uav_id} CLIMBING(alt "
                    f"{(p_now[2] if p_now else 0.0):.0f} < "
                    f"{self._cruise_alt_thr:.0f} m)—— 用最佳爬升空速 "
                    f"{self.best_climb_eas:.0f} EAS,暫停 slot 速度補償")
            spd = self.best_climb_eas
        else:
            if self._climb_phase_logged:
                self._climb_phase_logged = False
                self.get_logger().info(
                    f"UAV{self.uav_id} CRUISE_FORMATION —— 開始 V 隊形追趕")
            # ⭐⭐ 縱向改用**有號**的沿航向分量。
            # ⚠️⚠️ 舊的 speed_for_error 吃無號距離,僚機**超前** slot 時
            #    仍命令 catch_speed ⇒ 加速、超前更多。實測 S31:四架僚機
            #    along 全部為正 +10383 ~ +28014 m(佔誤差 98%)、單調發散。
            #    這個 bug 先前踩不到:機體推力不足時僚機一律落後,而且
            #    實驗 A 之前速度指令零權限,兩側都沒有執行器。
            # ⚠️ 橫向不進速度律 —— 它由胡蘿蔔導引處理。速度只管縱向。
            along = self._along_to_slot((tlat, tlon))
            if along is None:
                spd = speed_for_error(err, self.formed_radius,
                                      self.catch_radius, nominal,
                                      self.catch_speed)
            else:
                # ⚠️ 減速下限必須**低於長機當下速度**,否則誤差凍結而非收斂:
                #    長機被 max-consensus 壓在 wait_speed 等人,僚機超前時
                #    若也降到同一個值 ⇒ 同速 ⇒ 閉合率 0 ⇒ 卡在 10 km 不動。
                #    0.67 是本專案已確立的 wait/巡航 比例。
                v_lo = max(self.airspeed_min, nominal * 0.67)
                spd = speed_for_along_error(along, self.catch_radius,
                                            nominal, self.catch_speed, v_lo)
                if along < 0.0 and not self._decel_logged:
                    self._decel_logged = True
                    self.get_logger().info(
                        f"UAV{self.uav_id} 超前 slot {-along:.0f} m ⇒ 減速到 "
                        f"{spd:.0f} EAS(長機 {nominal:.0f},下限 {v_lo:.0f})")
        # ⚠️⚠️ 夾到飛控接受的區間再送。區間外 ArduPlane 直接 return false
        #    ⇒ MAV_RESULT_FAILED,**整發丟掉**(不是夾住,是不執行)——
        #    寧可拿到 AIRSPEED_MAX 也不要拿到「上一次的舊值」。
        spd_cmd = max(self.airspeed_min, min(spd, self.airspeed_max))
        if spd_cmd != spd and not self._speed_clamped:
            self._speed_clamped = True
            self.get_logger().warning(
                f"UAV{self.uav_id} 速度指令 {spd:.0f} 超出飛控區間 "
                f"[{self.airspeed_min:.0f}, {self.airspeed_max:.0f}] "
                f"⇒ 夾到 {spd_cmd:.0f}(否則整發被拒收)")
        spd = spd_cmd
        if abs(spd - self._last_speed) > 0.5 or (now - self._last_speed_t) > 3.0:
            self.mav.change_speed_async(spd)
            self._last_speed, self._last_speed_t = spd, now
        self._climb_authority(err, now)
        self.status_pub.publish(String(data=json.dumps(
            {"id": self.uav_id, "err": (err if err is not None else -1.0)})))
        self._keep_guided(now)

    def _climb_authority(self, err, now):
        """動態爬升率限幅:大落後時把剩餘推力全部讓給水平追趕。

        ⚠️⚠️ 這是 AO1 實測 UAV6 **高空失速**的直接對策。證據:
              t=237–268 s、高度 1918→1768 m(掉了 150 m)、最低空速 **11.4 m/s**
              (AIRSPEED_MIN 是 25)。它同時被要求「爬到 3100 m」與「追 6 km 外的
              slot」,而 T/W 只有 0.403 —— 剩餘推力不夠兼顧,TECS 於是拿空速去換
              爬升率,一路換到失速。
              (t=131 時它的 slot 誤差只有 370 m、幾乎到位,之後才被甩開 ——
               失速就發生在被甩開的過程中。)

        機制:能量守恆。可用剩餘功率 P = V(T − D) 只有一份,
              爬升率吃掉的部分就不能拿來加速。落後很多時,**水平速度優先**:
              把 TECS_CLIMB_MAX 壓到 climb_cap_lo,飛機就不會為了爬升而放掉空速。
              進了 catch_radius 的比例控制區再放開,讓它把高度補回來。

        ⚠️ 只在跨越門檻時送參數(帶遲滯),不要每個 tick 送 —— 直連鏈路無 ACK,
           洗頻寬會排擠導引航點。
        """
        if err is None or self.mav is None or self.climb_gate_hi <= 0.0:
            return          # climb_gate_hi <= 0 = 關閉(基準線對照用)
        far = err > self.climb_gate_hi
        near = err < self.catch_radius
        want = self._climb_cap
        if far:
            want = self.climb_cap_lo
        elif near:
            want = self.climb_cap_hi
        if want == self._climb_cap or (now - self._climb_t) < 3.0:
            return
        p = self.mav.position()
        alt_amsl = (p[2] if p else 0.0) + self._home_amsl
        # 把爬升率上限換算成**航跡角**上限一併下發。兩者是同一件事的兩種表達,
        # 但 TECS 吃爬升率、飛控的姿態限制吃角度 —— 只設一邊會被另一邊繞過。
        #   γ_max = asin(RoC_max / TAS)
        gam_cap = gamma_for_climb_rate(want, self.climb_eas_floor, alt_amsl)
        if far:
            gam_cap = min(gam_cap, self.climb_gamma_max)
        ok = self.mav.param_set_async("TECS_CLMB_MAX", want)
        self.mav.param_set_async("PTCH_LIM_MAX_DEG", max(3.0, gam_cap + 3.0))
        if ok:
            self._climb_cap, self._climb_t = want, now
            vg, tas, _ = calculate_projected_ground_speed(
                self.climb_eas_floor, alt_amsl, gam_cap, 0.0)
            self.get_logger().info(
                f"UAV{self.uav_id} 能量分配 → 爬升率 ≤{want:.1f} m/s、"
                f"γ ≤{gam_cap:.1f}°(slot 誤差 {err:.0f} m,{alt_amsl:.0f} m AMSL)"
                f";{'推力優先給水平' if far else '恢復爬升'}:"
                f"EAS {self.climb_eas_floor:.0f} → TAS {tas:.0f} → 地速 {vg:.0f} m/s")

    def _keep_guided(self, now):
        """手動/自動起飛後若不在 GUIDED 就切回，否則 guided 航點不生效（會空盤旋）。
        **RTL/AUTO 白名單**:操作者/GCS 蓄意下的返航或任務(如降落)視為
        「操作者接管」— 不搶回 GUIDED(舊版每 2 s 搶回,降落/RTL 前必須整層
        殺節點;白名單後單機可被 GCS 隨時接管,回 GUIDED 即自動歸隊)。"""
        p = self.mav.position()
        mode = self.mav.mode()
        if mode in ("RTL", "AUTO"):
            if not self._operator_override:
                self._operator_override = True
                self.get_logger().warn(
                    f"UAV{self.uav_id}: operator mode {mode} — standing down "
                    f"(formation resumes when GUIDED returns)")
            return
        if self._operator_override:
            self._operator_override = False
            self.get_logger().info(f"UAV{self.uav_id}: back from operator mode — rejoining")
        if (p is not None and p[2] > self._guided_min_alt
                and mode != "GUIDED" and (now - self._last_guided_t) > 2.0):
            self.mav.set_mode_async("GUIDED")
            self._last_guided_t = now
            self.get_logger().info(f"UAV{self.uav_id}: re-asserting GUIDED for formation")

    def _v_target(self):
        """極座標 V slot：以長機（whoLeader）航向為基準，永遠排在長機後方的固定方位。
        無 whoLeader → None（讓 caller 退回固定 slot）。"""
        L = self._leader_estimate()      # 失聯時走 Dead Reckoning,不直接放棄編隊
        if L is None:
            return None
        if self.formation_shape == "CONE":
            # 錐形:四架僚機在長機後方 back 公尺處、半徑 R 的圓周上均分。
            # 側向分量走 polar_slot 的方位偏置,垂直分量走 self._cone_dz(見下方送 WP 處)。
            th = math.radians(cone_phase_deg(self.uav_id,
                                             max(1, self.n_uavs - 2)))
            back = self.v_spacing * math.cos(math.radians(self.cone_half))
            radius = self.v_spacing * math.sin(math.radians(self.cone_half))
            lat_off = radius * math.sin(th)            # 側向(右為正)
            self._cone_dz = radius * math.cos(th)      # 垂直(上為正)
            distance = math.hypot(back, lat_off)
            bearing_off = 180.0 - math.degrees(math.atan2(lat_off, back))
        else:
            self._cone_dz = 0.0
            rank, side = v_rank_side(self.uav_id, self.n_uavs)
            distance = rank * self.v_spacing
            # ⭐⭐ 爬升段直線跟隨、平飛段才向兩側展開。
            #
            # ⚠️⚠️ 實測 P13:UAV5/6 的爬升率只有 8.4–8.6 m/s(UAV1–3 是 12.0–12.2),
            #    而它們的**地速也較低**(93.7/96.2 vs 102)。速度與爬升率同時偏低
            #    ⇒ 不是「被要求飛快而犧牲爬升」,是有額外的能量消耗。
            #    唯一的差別是它們的 slot 側向偏置最大(rank2 = ±4589 m):
            #    爬升途中還要橫向移動 4.6 km ⇒ 持續帶坡度 ⇒ 過載 n = 1/cos φ,
            #    誘導阻力隨 n² 上升。側向偏置越大的機爬得越差,順序完全吻合。
            #
            # ⇒ 爬升段把 bearing_off 收成正後方(純 trail),不帶任何坡度;
            #   平飛段推力餘裕充足,再優雅地展開到 V 的側翼。
            #
            # ⚠️ **不可以在門檻上硬切換** —— rank2 的側向偏置是 4589 m,
            #    瞬間切換會讓 slot 橫向跳 4.6 km,飛機當場變成「落後 4.6 km」,
            #    速度控制器會全力追一個憑空出現的誤差。用一段高度做線性混合。
            # ⚠️ 混合用**時間**而非高度:高度帶最多只有 100 m 可用(閘門 3000、
            #    巡航 3100),飛機一改平就沒有高度可以繼續混合,等於還是硬切換。
            #    實算:300 m 高度帶 = 25 s ⇒ slot 橫向移動率 184 m/s,
            #    比飛機的追趕速度還快 —— slot 會橫向跑掉,永遠追不到。
            #    改成越過閘門後以 expand_time_s 秒線性展開,可延續到平飛段。
            #    120 s ⇒ rank2 的橫向移動率 4589/120 = 38 m/s,追得上。
            eff_angle = self.v_angle
            if self._cruise_alt_thr > 0.0:
                p_now = self.mav.position() if self.mav is not None else None
                now_s = self.get_clock().now().nanoseconds * 1e-9
                # 單調累積(爬升段凍結、巡航段前進),永遠不會塌回 0°
                eff_angle = self.v_angle * self._expand.update(
                    now_s, advance=not self._climbing)
                if abs(eff_angle - self._last_eff_angle) > 2.0:
                    self._last_eff_angle = eff_angle
                    self.get_logger().info(
                        f"UAV{self.uav_id} 側向展開 {eff_angle:.0f}°/"
                        f"{self.v_angle:.0f}°(alt "
                        f"{(p_now[2] if p_now else 0.0):.0f} m,"
                        f"側偏 {distance * math.sin(math.radians(eff_angle)):+.0f} m)")
            bearing_off = 180.0 + side * eff_angle  # 後左=180+θ、後右=180-θ、正後=180
            # ⚠️⚠️ **航點永遠不可以落在飛機後方。**
            #    3 s 彈射之下 UAV6 只落後長機 2355 m,而它的 slot 在 6000 m 後
            #    ⇒ slot 在**飛機後方 3645 m**。ArduPlane 收到身後的航點會轉頭飛
            #    過去,轉彎中爬升率崩掉 —— 實測 P23 的三次掉落(300–800、
            #    1400–2000、2000–2800 m)正對應各機轉頭的時刻,而且地速同步下降。
            #    我先前判斷「往後退是免費的(減速權限是加速的 2 倍)」漏了這點:
            #    導引律不是用減速讓它後退,是給它一個身後的航點。
            #    ⇒ 若 slot 在自己後方,爬升段改成沿長機航跡前進(縱向間距靠
            #      速度差自然拉開),平飛段才真的去站 slot。
            # ⚠️⚠️ **航點永遠不可以落在飛機後方。**
            #    3 s 彈射之下 UAV6 只落後長機 2355 m,而它的 slot 在 6000 m 後
            #    ⇒ slot 在**飛機後方 3645 m**。ArduPlane 收到身後的航點會轉頭
            #    飛過去,大坡度轉彎讓過載 n = 1/cos φ 上升、誘導阻力 ∝ n²,
            #    吃掉推力 ⇒ 地速與爬升率同時掉。
            #    ⇒ 爬升段:(a) bearing_off 收成 180°(純 trail,無側向);
            #             (b) 若 slot 落在機頭後方(|bearing| > 90°),
            #                 **不下發該點**,改投影到機頭前方 ≥ trail_ahead_m。
            #    縱向缺口純粹靠速度差在平飛段拉開,爬升段不准回頭。
            # ⭐⭐ 側偏凍結(Cross-track Freeze)—— 只在「曾經進過巡航段之後
            #    又掉高度」時生效,**初次爬升維持原本的純 trail**(階段 1 已通過,
            #    不要動它:側偏偏置越大爬得越差,順序完全吻合)。
            #
            # ⚠️⚠️ U1 實測:掉高度 ⇒ 閘門退回爬升段 ⇒ bearing_off 被收成 180°
            #    ⇒ slot 橫向瞬移回長機正後方(rank2 是 4589 m)。飛機為了追這個
            #    瞬移的目標要大坡度轉彎,而它正好是在**沒有能量**的時候被要求轉彎。
            #    ⇒ 掉高度時保持**當下的側偏**,沿平行於長機航跡的直線爬升,
            #      橫向一步都不動。高度回來之後再繼續 Lookahead 漸進展開。
            if (self.cross_freeze and self._cruise_alt_thr > 0.0
                    and self.mav is not None
                    and self._climbing and self._reached_cruise):
                me_f = self.fleet.get(self.uav_id)
                if me_f is not None:
                    hl = math.radians(L["heading"])
                    dn_f = (me_f[0] - L["lat"]) * EARTH_M_PER_DEG
                    de_f = ((me_f[1] - L["lon"]) * EARTH_M_PER_DEG
                            * math.cos(math.radians(L["lat"])))
                    # 本機相對長機的沿航向 / 垂直航向分量
                    al_f = dn_f * math.cos(hl) + de_f * math.sin(hl)
                    cr_f = -dn_f * math.sin(hl) + de_f * math.cos(hl)
                    # 目標:同一條平行線上,往前 L1(側偏 cr_f 原封不動)
                    al_t = al_f + max(self.l1_lookahead_m, self.trail_ahead_m)
                    tn = al_t * math.cos(hl) - cr_f * math.sin(hl)
                    te = al_t * math.sin(hl) + cr_f * math.cos(hl)
                    # ⚠️⚠️ **這一點仍然必須在飛機的機頭前方。**
                    #    「沿長機航向往前 1500 m」只保證它在**長機**的前方 ——
                    #    本機當下若指向別處(掉高度時常常正在轉),它就落在身後。
                    #    實測 X1(沒有這道保護):爬升段 behind% = **99.8–100%**、
                    #    |bearing| 中位數 112°、d2wp 剛好 1499(= L1)——
                    #    距離對、方向全錯。再配上 15° 坡度上限轉不回來,
                    #    UAV4 末端誤差 112 km。這正是既有程式碼一再強調的
                    #    「航點永遠不可以落在飛機後方」,而我先前 return 得太早、
                    #    把那道保護整個跳過了。
                    tlat_f = L["lat"] + tn / EARTH_M_PER_DEG
                    tlon_f = L["lon"] + te / (EARTH_M_PER_DEG
                                              * math.cos(math.radians(L["lat"])))
                    tlat_f, tlon_f = self._ahead_of_nose(
                        me_f, (tlat_f, tlon_f), self.mav.heading())
                    self._ff_target = None
                    if not self._freeze_logged:
                        self._freeze_logged = True
                        self.get_logger().info(
                            f"UAV{self.uav_id} 側偏凍結:掉高度期間保持側偏 "
                            f"{cr_f:+.0f} m 直線爬升(不收回 180°、不轉彎)")
                    return (tlat_f, tlon_f)
            elif self._freeze_logged and not self._climbing:
                self._freeze_logged = False
                self.get_logger().info(
                    f"UAV{self.uav_id} 高度回到巡航段 —— 恢復 Lookahead 漸進展開")
            if self._cruise_alt_thr > 0.0 and self.mav is not None:
                q = self.mav.position()
                me = self.fleet.get(self.uav_id)
                if q is not None and self._climbing:
                    bearing_off = 180.0          # (a) 橫向隔離(僅初次爬升)
                    if me is not None:
                        slat, slon, _ = polar_slot(L["lat"], L["lon"],
                                                   L["heading"], distance,
                                                   bearing_off)
                        hdg = self.mav.heading()
                        b = math.degrees(math.atan2(
                            (slon - me[1]) * EARTH_M_PER_DEG
                            * math.cos(math.radians(me[0])),
                            (slat - me[0]) * EARTH_M_PER_DEG)) % 360.0
                        rel = (b - hdg + 540.0) % 360.0 - 180.0
                        if abs(rel) > 90.0:      # (b) 目標在身後 → 禁止下發
                            # 同上:保留橫向、只推縱向(爬升段 bearing_off=180
                            # 已無側偏,所以橫向分量本來就接近 0,但公式一致)
                            hb = math.radians(hdg)
                            dn0 = (slat - me[0]) * EARTH_M_PER_DEG
                            de0 = ((slon - me[1]) * EARTH_M_PER_DEG
                                   * math.cos(math.radians(me[0])))
                            al0 = max(dn0 * math.cos(hb) + de0 * math.sin(hb),
                                      self.trail_ahead_m)
                            cr0 = -dn0 * math.sin(hb) + de0 * math.cos(hb)
                            self._climb_trail_wp = (
                                me[0] + (al0 * math.cos(hb)
                                         - cr0 * math.sin(hb)) / EARTH_M_PER_DEG,
                                me[1] + (al0 * math.sin(hb) + cr0 * math.cos(hb))
                                / (EARTH_M_PER_DEG
                                   * math.cos(math.radians(me[0]))))
                            if not self._trail_logged:
                                self._trail_logged = True
                                self.get_logger().info(
                                    f"UAV{self.uav_id} 爬升段:slot 在身後"
                                    f"({rel:+.0f}°),改投影機頭前方 "
                                    f"{self.trail_ahead_m:.0f} m,禁止回頭轉彎")
                        else:
                            self._climb_trail_wp = None
                elif self._trail_logged:
                    self._trail_logged = False
                    self._climb_trail_wp = None
                    self.get_logger().info(
                        f"UAV{self.uav_id} 越過高度閘門 —— 恢復 V 隊形 slot 站位")
            if self._climb_trail_wp is not None:
                self._ff_target = None
                return self._climb_trail_wp
            # ⭐⭐ 巡航段的「身後航點」保護已**整段移除** —— 它本身就是那個 bug。
            #
            # ⚠️⚠️ 實測 BL1 的鐵證:`|bearing_to_wp|` 的 p90 是 81.3 / 81.6 /
            #    82.3 / 85.7° —— 四架、整趟,**從來不超過 ~86°**,而 behind_deg
            #    正好是 90°。這不是物理限制,是幾何硬夾:目標被機制固定壓在
            #    門檻正下方。
            #
            #    舊實作把 slot 投影到**本機機頭座標系**再夾縱向:
            #        al2 = dn2·cos(hb2) + de2·sin(hb2)   # hb2 = **本機航向**
            #        cr2 = −dn2·sin(hb2) + de2·cos(hb2)
            #        al2 = max(al2, trail_ahead_m)
            #    當 slot 落在機頭後方時,al2 被夾成 500,於是目標的方位
            #    ≈ ψ ± atan2(cr2, 500) ≈ ψ ± 89° —— **目標的方位是相對本機航向定義的**。
            #    飛機朝它轉 → ψ 改變 → 目標跟著轉 89° → 永遠追不到。
            #    **航點在追飛機的機頭,不是飛機在追航點。** 這就是 P71 記錄的
            #    「飛機把時間都花在轉圈上」,以及 dpsi 穩定在 29–33°、
            #    橫向以 v·sinΔ ≈ 60 m/s 線性發散的成因。
            #    對照組:UAV6(backup_node)沒有這段投影,整趟穩定跟在長機航向。
            #
            # ⇒ 改用**胡蘿蔔點**(見 _carrot_on_path):落在「slot 所在、平行於
            #   長機航跡」的那條線上。它只由 (本機**位置**, 長機位置/航向, slot)
            #   決定,**與本機航向無關** ⇒ 正回饋被打斷。
            #   不變量由 tests/test_lookahead_geometry.py 釘住。
        lat, lon, _ = polar_slot(L["lat"], L["lon"], L["heading"], distance, bearing_off)
        # slot 前饋:導引 WP 瞄外推點抵消 ~3-5s 滯後;但回傳（＝算誤差用）的是真 slot,
        # 否則 err 含前置量 → formed 判定被污染（實測 err 平台正好卡在解除門檻上）。
        # §4.5 旋轉前饋:框在弧轉中(omega≠0)時,slot 的未來位置 = 長機沿弧前進
        # + slot 方位隨框旋轉 —— 線性外推會永遠指向弧外(實測弧轉 RMS 反而惡化)。
        # ⭐⭐ 前置角投影(Lookahead / carrot-chasing):導引 WP = slot 沿**長機航向**
        #    再往前 (ff + L1)。誤差仍然對真 slot 量(回傳的是 (lat, lon))。
        #
        #    ff  = wp_lead_time × 長機速度 —— 補導引鏈路 ~3–5 s 的滯後(瞄未來的 slot)
        #    L1  = l1_lookahead_m         —— 把目標推離正橫,讓夾角變小
        #
        # ⚠️⚠️ 為什麼需要 L1(P91/T1/U1 三趟一致):`bearing_to_wp` 中位數
        #    −73°~−86°,飛機整段巡航都用接近正橫的角度對著目標,而 behind_deg
        #    是 90° —— 一直卡在切換邊緣,而且正橫追蹤要求持續大坡度轉彎,
        #    誘導阻力 ∝ n² 把能量吃光(U1:掉 1000 m 高度)。
        #    推遠之後近場夾角 ≈ atan(側偏 / L1):側偏 400 m ⇒ 15°,漸進切入。
        # ⚠️ L1 只影響**導引點**,不影響誤差量測與 formed 判定 —— 否則 err
        #    會含前置量,formed 平台會卡在解除門檻上(實測過)。
        lead = self.wp_lead_time * float(L.get("speed", 0.0) or 0.0)
        om = float(L.get("omega", 0.0) or 0.0)             # 框轉率 deg/s
        if abs(om) > 0.2:
            # §4.5 弧轉:slot 的未來位置 = 長機沿弧前進 + slot 方位隨框旋轉。
            # 弧轉中「平行於長機航跡」不成立,維持原本的沿弧外推。
            dpsi = om * self.wp_lead_time
            adv = advance_along_heading(
                L["lat"], L["lon"], L["heading"] + dpsi / 2.0, lead)
            flat, flon, _ = polar_slot(adv[0], adv[1], L["heading"] + dpsi,
                                       distance, bearing_off)
            self._ff_target = (flat, flon)
        else:
            me_c = self.fleet.get(self.uav_id)
            self._ff_target = (
                self._carrot_on_path(me_c, (lat, lon), L["heading"], lead)
                if me_c is not None else
                advance_along_heading(lat, lon, L["heading"], lead))
        return (lat, lon)

    def _carrot_on_path(self, me, slot, leader_hdg, lead_m):
        """導引點 = slot 所在、**平行於長機航跡**那條線上,本機投影點前方 L1。

        ⚠️⚠️ **不變量(這一項的全部重點):本機航向不得影響本點的位置。**
        舊實作把 slot 投影到本機機頭座標系,於是目標方位變成
        「相對本機航向 ±89°」—— 飛機一轉,目標跟著轉,形成正回饋,
        `|bearing_to_wp|` 因此被硬夾在 behind_deg(90°)正下方,
        實測 p90 = 81–86°、dpsi 穩定 29–33°、橫向以 v·sinΔ 發散。
        這裡只用 (本機**位置**, 長機位置/航向, slot) —— 全部與 ψ 無關。

        縱向:`al` = slot 在本機前方多遠(長機軸)。
          - slot 還在前方(al > L1)⇒ 直接瞄 slot,保留縱向位置回授(行為不變)
          - slot 落到身後或太近(al ≤ L1)⇒ 瞄同一條線上前方 L1,
            **不要求掉頭**;縱向缺口交給速度差(那是有權限的通道)。
        橫向:`cr` 原封不動保留 —— 橫向導引全程有效(不是 P81 的開迴路直飛)。
        """
        hl = math.radians(leader_hdg)
        dn = (slot[0] - me[0]) * EARTH_M_PER_DEG
        de = ((slot[1] - me[1]) * EARTH_M_PER_DEG
              * math.cos(math.radians(me[0])))
        al = dn * math.cos(hl) + de * math.sin(hl)      # 沿長機航跡
        cr = -dn * math.sin(hl) + de * math.cos(hl)     # 垂直長機航跡
        al_c = max(al, max(0.0, self.l1_lookahead_m)) + lead_m
        return (me[0] + (al_c * math.cos(hl) - cr * math.sin(hl))
                / EARTH_M_PER_DEG,
                me[1] + (al_c * math.sin(hl) + cr * math.cos(hl))
                / (EARTH_M_PER_DEG * math.cos(math.radians(me[0]))))

    def _biomimetic_target(self):
        """相鄰追蹤 slot：goal=長機，挑最近前位鄰機為參考、V 翼偏置，再對障礙推離分裂。"""
        goal_ll = self.fleet.get(self.master_id)
        me_ll = self.fleet.get(self.uav_id)
        if goal_ll is None or me_ll is None or len(self.fleet) < 2:
            return None
        origin = goal_ll
        agents = []
        for uid, ll in self.fleet.items():
            e, n = self._enu(ll, origin)
            agents.append(Agent(sysid=uid, x=e, y=n, is_leader=(uid == self.master_id)))
        me = next((a for a in agents if a.sysid == self.uav_id), None)
        if me is None:
            return None
        goal = (0.0, 0.0)                       # 長機在 origin → goal 即原點
        ref = self.bf.forward_reference(me, agents, goal)
        if ref is None:
            return None
        sx, sy = self.bf.desired_slot(me, ref, goal)
        # 障礙推離（分裂繞行）：slot 落在障礙感知圈內 → 側推出去
        for (olat, olon, orad) in self.obstacles:
            oe, on = self._enu((olat, olon), origin)
            dx, dy = sx - oe, sy - on
            d = math.hypot(dx, dy)
            reach = orad + self.bf.p.obstacle_margin_m
            if 0.0 < d < reach:
                k = (reach - d)
                sx += k * dx / d
                sy += k * dy / d
        return self._latlon((sx, sy), origin)

    # ── 飛行 ──────────────────────────────────────────────────────────────
    def _fly(self):
        if not self.fsm.update():               # 仍在 TAKEOFF / arm / GUIDED
            return
        # ⚠️⚠️ tot_node 打擊中 ⇒ 停止下令。僚機在打擊期間是**扇形 slot 的成員**,
        #    該聽 tot_node 的 ALIGN/PERCH/DIVE,不是編隊層的 V 字 slot。
        #    兩層同時發航點的後果見 strike_authority(AP2:飛機被壓在 50–80 m)。
        # ⚠️ 只停下令 —— formation_status 與遙測照發,長機那邊的成形判定
        #    與 AeroPlan HUD 都還看得到這一架。
        if self._strike_auth.yielded():
            # 讓的是「位置權」,不是「模式維持」—— 與 backup_node 同一個修正。
            # `_keep_guided()` 只在 `_formation_control()` 最後一行被呼叫,而那行
            # 在這個 return 下面 ⇒ 一讓權就沒人維持 GUIDED,tot_node 的
            # DO_REPOSITION 會被飛控整發丟掉且零告警(param2=0 是刻意的,
            # 見 mavros_interface.send_guided_waypoint 的說明)。
            # n>2 才會用到本節點,但 n-agnostic 是硬要求,不能只修 n=2 的路徑。
            self._keep_guided(self.get_clock().now().nanoseconds * 1e-9)
            return
        # ── 爬升脫離前放手(見 climbout_hold_alt 的宣告說明)────────────────
        # ⚠️ **鎖存**:一旦脫離就不再回到放手狀態。否則編隊層把飛機帶回
        #    formation_alt(50 m,低於門檻 120)時會再度放手 ⇒ 兩個狀態來回跳,
        #    正是 stot-altitude-gate-chatter 記載的那種「閘門沒遲滯」的病。
        if not self._climbout_done:
            _now = self.get_clock().now().nanoseconds * 1e-9
            _p = self.mav.position()
            if _p is None or _p[2] < self._climbout_hold_alt:
                if _now - self._climbout_log_t > 10.0:
                    self._climbout_log_t = _now
                    self.get_logger().info(
                        f"UAV{self.uav_id}: 爬升脫離中(alt "
                        f"{(_p[2] if _p else float('nan')):.0f} < "
                        f"{self._climbout_hold_alt:.0f} m)—— 本層不下令,"
                        f"交給飛控 TAKEOFF 模式")
                self._keep_guided(_now)
                return
            self._climbout_done = True
            self.get_logger().info(
                f"UAV{self.uav_id}: ✔ 已脫離爬升段(alt {_p[2]:.0f} m)—— 編隊層接手")
        # ⚠️ 相位每 tick 只更新一次,之後 _v_target / _formation_control /
        #    _cmd_log 全部讀同一個 latch —— 三處各自裸比較會在同一 tick 內
        #    看到不同的相位(高度就在門檻上跳動時)。
        self._update_climb_phase()
        self._ff_target = None                  # 只有 _v_target 會設定（slot 前饋）
        if self._rtl:
            target, tag = self._slot_at(self.home), "RTL home"   # RTL：繞 HOME 固定 slot
        elif self.use_biomimetic:
            target, tag = self._biomimetic_target(), "biomimetic slot"
        else:
            target, tag = self._v_target(), "V slot"             # 預設：隨長機航向的極座標 V
        if target is None:                      # 後備：固定 slot 跟長機 GeoPose
            if self.master_fix is None:
                return
            target, tag = self._slot_at(self.master_fix), "star slot (fallback)"
        tlat, tlon = target
        self._formation_control(tlat, tlon)     # 速度匹配 + 回報 + 保持 GUIDED（誤差對真 slot）
        # 導引 WP 用前饋外推點（有的話）——飛向未來的 slot、誤差照舊對真 slot 量
        wlat, wlon = self._ff_target if self._ff_target is not None else (tlat, tlon)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._operator_override:
            return          # 操作者接管中:DO_REPOSITION 會強切 GUIDED,必須停發
        moved = (self._last_wp is None
                 or dist_m(self._last_wp, (wlat, wlon)) > self._wp_move_thresh)
        if moved and (now - self._last_wp_t) >= self._wp_interval:
            # 錐形的垂直分量加在這裡。V 的 _cone_dz 恆為 0,行為不變。
            wp_alt = self.formation_alt + self._cone_dz
            phase = "CLIMB" if self._climbing else "CRUISE"
            self._cmd_log(phase, self._last_speed, (wlat, wlon, wp_alt), now)
            if self.mav.send_guided_waypoint(wlat, wlon, wp_alt):
                self._last_wp = (wlat, wlon)
                self._last_wp_t = now
                self.get_logger().info(
                    f"UAV{self.uav_id} -> {tag} -> "
                    f"({tlat:.5f},{tlon:.5f}) @ {wp_alt:.0f} m")


def main():
    rclpy.init()
    node = WingmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_roll_limit()   # 交棒給 tot_node 之前把坡度權限還回去
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
