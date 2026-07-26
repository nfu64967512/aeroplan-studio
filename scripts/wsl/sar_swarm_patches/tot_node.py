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
(state-in 亦可改走 MAVROS/uavN/global_position 以求全一致;此處為自足直讀分流。)

狀態機(視覺震撼終端俯衝版):
  ALIGN  — TOT 對齊扇形 slot(快機 S 型燒時間、慢機直線加速),同步抵扇形。
  PERCH  — 各機沿切線內切到 pushover 環,同步爬升到 perch_alt 高點佔位。
  DIVE   — 全員同步 pushover,全速俯衝(終端加速)收斂目標 T;微高度分層防撞、
           時間校正 → ~同時衝擊(1m 級)。
"""
import csv, json, math, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from pymavlink import mavutil

EARTH = 111320.0
GATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)  # latched:晚連的前端也拿得到


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
    def __init__(self):
        super().__init__("tot_node")
        g = self.declare_parameter
        g("n", 6); g("tlat", 25.1406); g("tlon", 121.2325); g("talt", 90.0)
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
        g("pn_hz", 50.0)          # 姿態指令率 = ArduPlane 主迴圈率
        g("gate_r", 1200.0)       # 交接水平距(m)
        g("gate_h", 600.0)        # 交接高度(離本機自己的地表 m)→ 視線俯角 26.6°
        g("pn_n_lat", 4.0)        # 橫向導航常數(掃描 3~4 平坦,取 4 留延遲餘裕)
        g("pn_n_vert", 3.0)       # 縱向導航常數
        g("pn_lead", 1.0)         # 縱向 γ 指令前導(s)
        g("pn_freeze_tgo", 1.5)   # t_go 小於此就凍結指令(但仍持續發送)
        g("pn_freeze_r", 100.0)   # 或距離小於此
        g("csv", "/tmp/tot_node")
        p = self.get_parameter
        self.n = int(p("n").value)
        self.T = (float(p("tlat").value), float(p("tlon").value))
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
        self.line_tol = float(p("line_tol").value)
        self.resend_dt = float(p("resend_min_dt").value)
        self.expend = bool(p("expend_on_impact").value)
        self.impact_settle = float(p("impact_settle").value)
        self.done_t = None     # 全員命中的時刻(收尾要再多錄 impact_settle 秒)
        if bool(p("auto_ring").value):
            # 由「可達的下滑角」反推環半徑,而不是硬給半徑再期待飛機做得到。
            r_need = (self.perch_alt - self.dive_alt) / math.tan(
                math.radians(self.dive_angle))
            self.get_logger().info(
                f"auto_ring：perch {self.perch_alt:.0f}m / 角度 {self.dive_angle:.0f}° "
                f"→ pushover_r {self.pushover_r:.0f} → {r_need:.0f}m")
            self.pushover_r = r_need
        self.perch_reach = float(p("perch_reach").value)
        self.impact_reach = float(p("impact_reach").value)

        # ── 扇形 slot + pushover 環 + 穿越點(T 後方 overshoot,直線穿過 T)──
        self.slots, self.pp, self.thru = [], [], []
        for k in range(self.n):
            off = -self.half + 2 * self.half * k / (self.n - 1)
            brg = (self.approach + off) % 360.0
            self.slots.append(dest(self.T[0], self.T[1], brg, self.r_fan))
            self.pp.append(dest(self.T[0], self.T[1], brg, self.pushover_r))
            self.thru.append(dest(self.T[0], self.T[1],
                                  (brg + 180.0) % 360.0, self.overshoot))
        self.spacing = self.r_fan * math.radians(2 * self.half) / (self.n - 1)
        self.pn_on = bool(p("pn_enable").value)
        self.fc_base = int(p("fc_base").value)
        self.fc_stride = int(p("fc_stride").value)
        self.tel_us = float(p("tel_us").value)
        self.pn_dt = 1.0 / max(5.0, float(p("pn_hz").value))
        self.gate_r = float(p("gate_r").value)
        self.gate_h = float(p("gate_h").value)
        self.n_lat = float(p("pn_n_lat").value)
        self.n_vert = float(p("pn_n_vert").value)
        self.pn_lead = float(p("pn_lead").value)
        self.fz_tgo = float(p("pn_freeze_tgo").value)
        self.fz_r = float(p("pn_freeze_r").value)
        self.vel = {}         # 每機 NED 速度 (vn, ve, vd)
        self.att = {}         # 每機姿態 (roll, pitch, yaw) rad
        self.alpha = {}       # 攻角估計(低通)
        self.strike = {}      # 是否已進入終端 PN
        self.frozen = {}      # 凍結後保持的指令
        self.fc = {}          # 直連飛控的連線
        self.home_amsl = {}   # 每機 home 海拔
        self.zg = {}    # 每機自量的紅心地表高度(相對 HOME)
        self.zg_amsl = {}  # 紅心地表的 AMSL(GPS 基準),換算真值高度用
        self.home_ll = {}    # 每機 HOME 經緯(地表差的減數)
        self.terr_bull = {}  # 紅心地表 AMSL(地形資料庫)
        self.terr_home = {}  # 本機 HOME 地表 AMSL(同一資料庫,offset 才會抵消)
        self.truth = {}       # SITL 真值位置(SIM_STATE);只量測,不進導引
        self.truth_dmin = {}  # 真值基準的最近水平距
        self.truth_hit = {}   # 真值基準的命中高度(離紅心地表)
        self.dmin = {}   # 各機對 T 的最近水平距(記錄真實穿越)
        self.commit = {}  # 各機是否已鎖定穿越(進 switch_r 後不再翻回瞄 T)
        self.thru_dyn = {}  # 鎖定當下算出的穿越點(在「本機→T」的延長線上)
        self.hit_alt = {}  # 各機命中當下的高度(驗證是俯衝命中而非高空掠過)
        self.wp_key = {}   # 俯衝航點鎖存鍵(同 key 不重發 → 不重設高度斜坡)
        self.wp_t = {}     # 上次重發時刻(節制重發頻率)
        self.wp_pt = {}    # 上次實際送出的航點 (lat, lon, alt),判斷「有沒有動」
        self.align_ok = {}  # 是否已在遠端對正過徑向線(鎖存,避免極限環)
        self.ready = {}     # PERCH 就緒(高度+對正,鎖存;全員就緒才一起內切)
        self.runup_t0 = None  # 同步內切的起始時刻(None = 還在 slot 上爬升對正)

        # ── ① MAVLink 分流連線(每機一條)──
        self.link, self.pos, self.gs = {}, {}, {}
        self.slot_of, self.slot_id, self.arr = {}, {}, {}
        self.perch_arr, self.impact = {}, {}
        self.last_v, self.last_v_t = {}, {}
        lbase = int(p("link_base").value); lstride = int(p("link_stride").value)
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
        self.phase = "INIT"
        self.t0 = None
        self.T_arr = None
        self.dive_go = None
        self.csv_path = str(p("csv").value)
        self._csv = open(self.csv_path + ".csv", "w", newline="")
        self._cw = csv.writer(self._csv)
        self._cw.writerow(["t", "sysid", "lat", "lon", "alt", "gs", "d_slot",
                           "mode", "v_cmd", "slot_lat", "slot_lon", "phase"])
        gf = open(self.csv_path + "_geom.csv", "w", newline="")
        gw = csv.writer(gf); gw.writerow(["kind", "lat", "lon"])
        gw.writerow(["target", self.T[0], self.T[1]])
        for k, s in enumerate(self.slots):
            gw.writerow([f"slot{k}", s[0], s[1]])
        for k, s in enumerate(self.pp):
            gw.writerow([f"pp{k}", s[0], s[1]])
        gf.close()
        self._publish_gate()
        self._terrain_probe()
        self.create_timer(0.05, self._tick)   # 20 Hz:終端 PN 需要高頻   # 3 Hz 協同迴圈
        self.create_timer(1.0, self._publish_gate)  # latched 定期補發
        # 地形快取是逐步載入的:第一次問可能還沒 loaded(spacing=0),要持續問。
        self.create_timer(3.0, self._terrain_probe)
        self.get_logger().info(
            f"tot_node up：扇形 R={self.r_fan:.0f} ±{self.half:.0f}° {self.n} slot "
            f"間隔 {self.spacing:.0f}m；pushover R={self.pushover_r:.0f} perch={self.perch_alt:.0f}m "
            f"→ 俯衝 → 目標 {self.T}")

    def _publish_gate(self):
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
                m = c.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE"],
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
                else:
                    self.att[i] = (m.roll, m.pitch, m.yaw)

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

    def _pump_link(self):
        for i, c in self.link.items():
            while True:
                m = c.recv_match(
                    type=["GLOBAL_POSITION_INT", "ATTITUDE", "TERRAIN_REPORT",
                          "SIM_STATE", "HOME_POSITION"],
                    blocking=False)
                if m is None:
                    break
                mt = m.get_type()
                if mt == "GLOBAL_POSITION_INT":
                    self.pos[i] = (m.lat / 1e7, m.lon / 1e7,
                                   m.relative_alt / 1000.0)
                    # vx/vy/vz = 北/東/下 (cm/s)
                    self.vel[i] = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
                    self.gs[i] = math.hypot(m.vx, m.vy) / 100.0
                    # home 的海拔 = 絕對高 - 相對高(用來把地形 AMSL 轉成相對高)
                    self.home_amsl[i] = m.alt / 1000.0 - m.relative_alt / 1000.0
                elif mt == "ATTITUDE":
                    self.att[i] = (m.roll, m.pitch, m.yaw)
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
                    if i in self.terr_bull and i in self.terr_home:
                        zg = self.terr_bull[i] - self.terr_home[i]
                        # 紅心地表的 AMSL(GPS 基準)= 換算 SIM_STATE 真值高度用
                        if i in self.home_amsl:
                            self.zg_amsl[i] = self.home_amsl[i] + zg
                        if i not in self.zg:
                            self.get_logger().info(
                                f"UAV{i} 紅心地表 z_g = {zg:+.2f} m(相對 HOME;"
                                f"紅心 {self.terr_bull[i]:.2f} − HOME 地表 "
                                f"{self.terr_home[i]:.2f} AMSL)")
                        self.zg[i] = zg

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
        c = self.fc.get(i) or self.link[i]
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        c.mav.set_attitude_target_send(
            0, c.target_system or i, 1, 132,
            [cr * cp, sr * cp, cr * sp, -sr * sp], 0.0, 0.0, 0.0, float(thr))

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

    def _pn_cmd(self, i, now):
        """終端律 → (roll rad, pitch rad, thrust)。三通道解耦。"""
        GRAV = 9.80665
        dn, de, d, h, R, vn, ve, vh, hdot, V = self._geom(i)
        gam = math.atan2(hdot, max(vh, 1e-3))

        # ── 橫向:真 PN。λ̇ 由速度向量解析求得(零微分雜訊)──
        lam_dot = (de * vn - dn * ve) / (d * d)
        a_h = self.n_lat * vh * lam_dot
        # ⚠️ 分母是 g·cosγ 不是 g:維持俯角 γ 需要 n·cosφ = cosγ,
        #    用 atan(a/g) 在 31° 俯衝會少要約 14% 的坡度。
        roll = math.atan2(a_h, GRAV * max(math.cos(gam), 0.25))

        # ── 縱向:同一條律在垂直面,用「角速率」形式(角度形式會要出 -90°)──
        sig_dot = (hdot * d - h * (-(dn * vn + de * ve) / d)) / (R * R)
        gdot = -self.n_vert * sig_dot
        # 法向過載夾制 n_z = cosγ + V·γ̇/g ∈ [0.05, 3.0]
        nz = math.cos(gam) + V * gdot / GRAV
        nz = max(0.05, min(3.0, nz))
        gdot = (nz - math.cos(gam)) * GRAV / V
        gam_cmd = gam + gdot * self.pn_lead

        # 攻角現場估計(低通),把 γ 指令換成姿態指令
        th_meas = self.att.get(i, (0.0, 0.0, 0.0))[1]
        a_raw = th_meas - gam
        a_prev = self.alpha.get(i, a_raw)
        k = min(1.0, self.pn_dt / 0.7)
        self.alpha[i] = a_prev + k * (a_raw - a_prev)
        pitch = gam_cmd + self.alpha[i] / max(math.cos(roll), 0.5)
        pitch = max(math.radians(-80.0), min(math.radians(45.0), pitch))

        # 韌體的耦合夾制(roll 上限隨 pitch 收斂),自己先夾好才能記錄飽和
        r_lim = math.radians(80.0) * abs(math.cos(th_meas))
        roll = max(-r_lim, min(r_lim, roll))

        # ── 油門:唯一的命中時刻通道,與兩個轉向通道解耦 ──
        # ⚠️ 時間窗過了就不要硬夾 0.4 s:那會讓 v_req = R/0.4 直接飽和成全速,
        #    終端一路衝過頭。過期後改用「以現速估的 t_go」,退化成維持現速。
        t_left = self.dive_T - now
        t_rem = t_left if t_left > 1.0 else max(1.0, R / max(V, 1e-3))
        v_req = max(45.0, min(135.0, R / t_rem))
        thr = max(0.0, min(1.0, 0.36 + 0.030 * (v_req - V)))
        return roll, pitch, thr, R, V

    def _pn_step(self, i, now):
        """對單機跑一次終端律並發送。回傳 True 表示已在終端段。"""
        if i not in self.pos or i not in self.vel:
            return False
        roll, pitch, thr, R, V = self._pn_cmd(i, now)
        t_go = R / max(V, 1e-3)
        # 終端凍結:t_go 或距離夠小就鎖住指令,但「仍要持續發送」——
        # 停發 3 s 會觸發 GUIDED_TIMEOUT,控制權默默回到 L1。
        if t_go <= self.fz_tgo or R <= self.fz_r:
            if i not in self.frozen:
                self.frozen[i] = (roll, pitch, thr)
            roll, pitch, thr = self.frozen[i]
        self._att_target(i, roll, pitch, thr)
        return True

    def _zg(self, i):
        """本機自量的地表高度(相對 HOME);沒有地形回報就退回全域 ground_alt。"""
        return self.zg.get(i, self.ground_alt)

    def _goto(self, i, lat, lon, alt, v, move_tol=40.0, min_dt=2.0,
              keepalive=30.0):
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
        prev = self.wp_pt.get(i)
        moved = (prev is None or abs(prev[2] - alt) > 5.0
                 or dist_m(prev[:2], (lat, lon)) > move_tol)
        dt = now - self.wp_t.get(i, 0.0)
        if (moved and dt >= min_dt) or dt > keepalive:
            self._reposition(i, lat, lon, alt)
            self.wp_pt[i] = (lat, lon, alt)
            self.wp_t[i] = now
            self.wp_key[i] = None      # 讓 DIVE 的鎖存鍵重新生效

    def _reposition(self, i, lat, lon, alt, radius=0.0):
        """param3 = 繞圈半徑。⚠️ 傳 0 時 ArduPlane 用 WP_LOITER_RAD(實測 180m),
        飛機一接近就被捕獲進 180m 的繞圈 → 水平距永遠壓不下去(實測卡在 124-140m)。
        俯衝段要傳一個小半徑,才可能真的打到紅心中心。"""
        c = self.link[i]
        c.mav.command_int_send(
            c.target_system or i, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, float(radius),
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
        off = -self.half + 2 * self.half * self.slot_id[i] / (self.n - 1)
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
            math.sin(math.radians((self.approach - self.half
                     + 2 * self.half * k / (self.n - 1)) - self.approach)))
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
        self._cw.writerow([round(now - self.t0, 1), i, self.pos[i][0],
                           self.pos[i][1], self.pos[i][2], round(self.gs.get(i, 0), 1),
                           round(d), mode, round(self.last_v[i], 1),
                           tgt[0], tgt[1], self.phase])

    def _tick(self):
        self._pump()
        if len(self.pos) < self.n:
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
                        sign = 1.0 if math.sin(2 * math.pi * (now - self.t0) / 22.0) >= 0 else -1.0
                        bs = bearing(self.pos[i][:2], self.slot_of[i])
                        aim = dest(self.pos[i][0], self.pos[i][1], bs + sign * theta, 120.0)
                        self._goto(i, aim[0], aim[1], self._alt(i), self.vmin)
                stat[i] = {"mode": mode, "d": round(d),
                           "arr": round(self.arr[i] - self.t0, 1) if i in self.arr else None}
                self._row(now, i, d, mode, self.slot_of[i])
            self._csv.flush()
            if len(self.arr) == self.n:
                got = [self.arr[i] - self.t0 for i in self.arr]
                self.get_logger().warn(
                    f"★ 全員抵扇形。抵達散布 = {max(got)-min(got):.1f}s → PERCH(對準內切)")
                self.phase = "PERCH"; self.perch_t0 = now
                v_plan = self.plan_frac * self.vmax
                # 內切到 pushover 環的剩餘航程 ≈ (距T - pushover_r)
                tmax = max((dist_m(self.pos[i][:2], self.T) - self.pushover_r) / v_plan
                           for i in self.pos)
                self.perch_T = now + tmax + 6.0

        elif self.phase == "PERCH":
            # 對準內切:朝「穿越點」(T 後方)飛 → 航向已對準穿過 T 的直線(轟炸航線),
            # 爬到 perch_alt、速度同步在 perch_T 抵 pushover 環。就位不繞圈(續朝內、
            # 保持對準)→ DIVE 是「已對準的直線俯衝穿越」,不再轉彎偏開 T。
            for i in sorted(self.pos):
                thru = self.thru[self.slot_id[i]]
                pp = self.pp[self.slot_id[i]]
                d = dist_m(self.pos[i][:2], self.T)      # 距 T
                z = self.pos[i][2]
                # ⚠️ 距離 + 高度雙閘門。舊版只看距離 → 還沒爬到高點就被判定就位,
                # 俯衝從 778-1187m(而非指令 1500m)起跳,下滑角自然不足 → 掉不到底。
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
                        and d < self.pushover_r:
                    self.perch_arr[i] = now
                    self.get_logger().info(
                        f"UAV{i} 就位對準環 @+{now-self.t0:.1f}s "
                        f"(alt {z:.0f}m 橫偏 {xt:.0f}m)")
                if i in self.perch_arr:
                    # 就位後「守在自己徑向線上的環點 pp」等其他機,而不是繼續內切。
                    # ⚠️ 舊版就位後仍朝 thru 慢飛 → 先就位的一路內漂,等最後一架好時
                    #    各機距 T 已經差很多 → 俯衝起跑點不等距 → 命中散布 26.7s。
                    #    瞄 pp 同時保住「半徑」和「對正」(pp 就在自己的徑向線上)。
                    mode = "hold_ring"; self._goto(i, *pp, self.perch_alt, self.vmin)
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
                    mode = "climb"; self._goto(i, *self.slot_of[i], self.perch_alt, max(self.vmin * 1.45, 65.0))
                else:
                    t_rem = self.perch_T - now
                    v = max(self.vmin, min(self.vmax, (d - self.pushover_r) / max(t_rem, 0.1)))
                    mode = "runup"; self._goto(i, *thru, self.perch_alt, v)
                stat[i] = {"mode": mode, "d": round(d), "z": round(self.pos[i][2])}
                self._row(now, i, d, mode, thru)
            self._csv.flush()
            # 全員就緒才一起放行內切 —— 起跑點齊、航線已對正,之後不再重判。
            if self.runup_t0 is None and len(self.ready) == self.n:
                self.runup_t0 = now
                v_plan = self.plan_frac * self.vmax
                tmax = max((dist_m(self.pos[i][:2], self.T) - self.pushover_r)
                           / v_plan for i in self.pos)
                self.perch_T = now + tmax + 6.0
                zs = {i: round(self.pos[i][2]) for i in sorted(self.pos)}
                self.get_logger().warn(
                    f"★ 全員就緒(alt {zs})→ 同步內切到環,預計 {tmax+6.0:.0f}s")
            if len(self.perch_arr) == self.n:
                zs = {i: round(self.pos[i][2]) for i in self.pos}
                self.get_logger().warn(
                    f"★ 全員對準內切(alt {zs})→ DIVE 直線俯衝穿越!")
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
                    if self.hdem_tconst > 0.0:
                        lk.mav.param_set_send(
                            lk.target_system or j, 1, b"TECS_HDEM_TCONST",
                            self.hdem_tconst,
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                # 共同命中時刻:用「最遠的那架 × 可達俯衝速」定 T_impact,近機照這個
                # 時刻放慢配速 → 六機同時到。配速用實際可達速(非 dive_speed 指令值),
                # 否則 t_rem 立刻歸零、全員全速、失去等待餘裕。
                dmax = max(dist_m(self.pos[i][:2], self.T) for i in self.pos)
                self.dive_pace = self.pace_cfg if self.pace_cfg > 1.0 else min(
                    self.vmax, 0.8 * self.dive_speed)
                self.dive_T = now + dmax / self.dive_pace + 2.0
                # 錯開放行:近的先在環上多等一會兒,之後大家用同一個配速飛。
                # ⚠️ 這是「不靠減速」達成同時命中的關鍵 —— 靠減速行不通,因為近機
                #    需要的速度會低於噴射可持續下限 vmin(45),被夾住就一定早到
                #    (實測早到 14.5s;把 vmin 壓到 38 又會失速亂飛)。
                self.release_t = {}
                for j in self.pos:
                    dj = dist_m(self.pos[j][:2], self.T)
                    self.release_t[j] = now + (dmax - dj) / self.dive_pace
                lag = {j: round(self.release_t[j] - now, 1) for j in sorted(self.release_t)}
                self.get_logger().info(
                    f"DIVE 配速:最遠 {dmax:.0f}m / {self.dive_pace:.0f} m/s "
                    f"→ 共同命中 T+{self.dive_T-now:.0f}s;錯開放行 {lag}")

        elif self.phase == "DIVE":
            done = 0
            for i in sorted(self.pos):
                d = dist_m(self.pos[i][:2], self.T)
                # ── 終端交接:進了 gate_r 就改用 PN 直接下姿態 ──
                # 航點導引本質到不了個位數公尺:GUIDED 高度需求被
                # TECS_HDEM_TCONST 節流、L1 的時間常數 ζ·T/π ≈ 4 s 在終端
                # 根本來不及修。PN 用 50 Hz 姿態指令繞開兩者。
                # ⚠️ 還在環上等放行的機不可以進 PN:那會一邊用姿態律撲向目標、
                #    一邊被 wait 分支用 DO_REPOSITION 拉回環點,兩套指令打架。
                #    目前 pushover 環(2541 m)在 gate_r(1200 m)之外所以碰不到,
                #    但 gate_r 一調大就會踩到 —— 先擋起來。
                released = now >= getattr(self, "release_t", {}).get(i, 0.0)
                if self.pn_on and self.fc and released and i not in self.impact:
                    if not self.strike.get(i) and d <= self.gate_r:
                        self.strike[i] = True
                        self.get_logger().info(
                            f"UAV{i} 交接終端 PN @ d={d:.0f}m "
                            f"h={self.pos[i][2]-self._zg(i):.0f}m")
                    if self.strike.get(i):
                        self._pn_step(i, now)
                thru = self.thru[self.slot_id[i]]   # 穿越瞄準點(T 後方)
                if d < self.dmin.get(i, 1e9):
                    self.dmin[i] = d                # 追蹤最近水平距(遙測基準)
                if i in self.truth:                 # 同一件事的 SITL 真值基準
                    td = dist_m(self.T, self.truth[i][:2])
                    if td < self.truth_dmin.get(i, 1e9):
                        self.truth_dmin[i] = td
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
                else:
                    aimpt = self.T; alt = self._line_alt(i, 0.0)
                # 衝擊判定:進 impact_reach,或已 committed 且通過最近點(d 回升)→ 記為穿越
                passed = self.commit.get(i) and d > self.dmin[i] + 8.0
                if i not in self.impact and (d < self.impact_reach or passed):
                    self.impact[i] = now
                    # 以「離紅心地表多高」回報:真正砸中 = 0
                    self.hit_alt[i] = self.pos[i][2] - self._zg(i)
                    if i in self.truth and i in self.zg_amsl:
                        self.truth_hit[i] = self.truth[i][2] - self.zg_amsl[i]
                    self.get_logger().info(
                        f"UAV{i} 命中紅心 @+{now-self.dive_go:.2f}s (最近 {self.dmin[i]:.0f}m "
                        f"命中高度 {self.hit_alt[i]:+.0f}m(離紅心地表) "
                        f"gs {self.gs.get(i,0):.0f}m/s)")
                if now < getattr(self, "release_t", {}).get(i, 0.0):
                    # 還沒輪到:在自己的環點上等,保持高度與對正
                    mode = "wait"
                    self._goto(i, *self.pp[self.slot_id[i]], self.perch_alt, self.vmin)
                    stat[i] = {"mode": mode, "d": round(d),
                               "wait": round(self.release_t[i] - now, 1)}
                    self._row(now, i, d, mode, self.T)
                    continue
                if i in self.impact:
                    done += 1; mode = "expended" if self.expend else "impact"
                    if not self.expend:
                        # 保留機體 → 續飛穿越點 egress(俯衝速度),不繞 T
                        self._goto(i, *thru, alt, self.dive_speed)
                    # expend_on_impact:命中即視為消耗,停止下令(不脫離、不拉起)
                else:
                    # 俯衝砸向 T:指令上限 dive_speed(> 頂速 → TECS 壓機頭俯衝加速);
                    # 近機(會早到)v_req 偏低 → 放慢到最低 0.62·dive_speed 等後機(仍在俯衝)
                    # 俯衝段也要做時間對齊(這是 STOT 的 S):近機必須能「慢下來等」。
                    # ⚠️ 舊版速度下限是 0.62·dive_speed(=87 m/s,比可達速還高)→ 全員
                    #    永遠全速 → 完全沒有等待餘裕 → 命中散布 26.7s。下限放到 vmin。
                    t_rem = self.dive_T - now
                    v_req = d / max(t_rem, 0.05)
                    v = max(self.vmin, min(self.dive_speed, v_req))
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
                    if (self.pos[i][2] - z_line > self.line_tol and
                            now - self.wp_t.get(i, 0.0) > self.resend_dt):
                        key = f"{key}#{int(now)}"        # 換 key → 觸發一次重發
                    if not self.strike.get(i):
                        self._goto_latched(i, *aimpt, alt, v, key=key)
                stat[i] = {"mode": mode, "d": round(d), "alt": round(alt),
                           "z": round(self.pos[i][2]), "gs": round(self.gs.get(i, 0))}
                self._row(now, i, d, mode, self.T)
            self._csv.flush()
            if done == self.n and self.done_t is None:
                # ⚠️ 不能在最後一架「剛進 impact_reach」的瞬間就收工。那一架還沒飛到
                #    真正的最近點,dmin 就凍在 19-23 m,看起來像導引誤差,其實是
                #    取樣被截斷。實測 B4:最後三架(3/5/6)報 11.1-23.3 m,但它們
                #    偏離自己徑向線只有 1.5 m —— 誤差全是「還沒走到」的縱向殘量;
                #    早 8 秒命中、因此多取樣了 8 秒的 UAV4 報 0.05 m。
                #    多跑 impact_settle 秒讓所有人走完最近點再結算。
                self.done_t = now
            if done == self.n and now - self.done_t >= self.impact_settle:
                imp = [self.impact[i] - self.dive_go for i in self.impact]
                spread = max(imp) - min(imp)
                dmins = {i: round(self.dmin[i], 1) for i in sorted(self.impact)}
                halts = {i: round(self.hit_alt.get(i, -1)) for i in sorted(self.impact)}
                self.get_logger().warn(
                    f"★★ 俯衝命中紅心:全員命中。命中時間散布 = {spread:.2f}s "
                    f"(≈{spread*self.dive_speed:.1f}m 縱向間隔);最近水平距 = {dmins} m;"
                    f"命中高度 = {halts} m(俯衝角 {self.dive_angle:.0f}°)")
                # 真值基準另外報一行。⚠️ 兩個基準差約 2 m(EKF 定位誤差),
                # 遙測基準會把它抵消、真值基準會疊加 —— 宣稱 ≤5 m 之前必須講清楚
                # 用的是哪一個,不能挑好看的那個報。
                if self.truth_dmin:
                    tmins = {i: round(self.truth_dmin[i], 1)
                             for i in sorted(self.truth_dmin)}
                    thalts = {i: round(self.truth_hit.get(i, -999))
                              for i in sorted(self.truth_dmin)}
                    self.get_logger().warn(
                        f"★★ 真值基準(SIM_STATE):最近水平距 = {tmins} m;"
                        f"命中高度 = {thalts} m")
                self.phase = "DONE"

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
