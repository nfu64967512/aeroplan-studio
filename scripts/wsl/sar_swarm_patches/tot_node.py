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
        g("dive_angle", 45.0)     # 俯衝角(度)。指令高度沿下滑線 alt = d·tan(角度) → 直線
                                  # 俯衝「正好在紅心觸地」,而非平飛到目標上空才開始降。
        g("expend_on_impact", True)  # 命中即視為消耗(不再下令、不脫離)
        g("perch_reach", 70.0)    # 認定「已就位 pushover 環」的半徑
        g("impact_reach", 40.0)   # 認定「已穿越 T(衝擊)」的水平半徑(另有通過最近點判定)
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
        self.expend = bool(p("expend_on_impact").value)
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
        self.dmin = {}   # 各機對 T 的最近水平距(記錄真實穿越)
        self.commit = {}  # 各機是否已鎖定穿越(進 switch_r 後不再翻回瞄 T)
        self.hit_alt = {}  # 各機命中當下的高度(驗證是俯衝命中而非高空掠過)
        self.wp_key = {}   # 俯衝航點鎖存鍵(同 key 不重發 → 不重設高度斜坡)

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
        self.create_timer(0.33, self._tick)   # 3 Hz 協同迴圈
        self.create_timer(1.0, self._publish_gate)  # latched 定期補發
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
        for i, c in self.link.items():
            while True:
                m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
                if m is None:
                    break
                self.pos[i] = (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)
                self.gs[i] = math.hypot(m.vx, m.vy) / 100.0

    def _goto(self, i, lat, lon, alt, v):
        c = self.link[i]; now = time.time()
        if abs(v - self.last_v[i]) > 0.5 or (now - self.last_v_t[i]) > 2.0:
            c.mav.command_long_send(c.target_system or i, 1,
                                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                                    0, float(v), -1, 0, 0, 0, 0)
            self.last_v[i] = v; self.last_v_t[i] = now
        self._reposition(i, lat, lon, alt)

    def _reposition(self, i, lat, lon, alt):
        c = self.link[i]
        c.mav.command_int_send(
            c.target_system or i, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
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
            self._reposition(i, lat, lon, alt)
            self.wp_key[i] = key

    def _alt(self, i):
        """每 slot 專屬高度層(對齊階段防撞):slot_id 對稱分佈於 talt ± (n-1)/2·layer。
        S 型機動時就算兩機水平路徑逼近,也差 ≥layer_m 垂直 → 不碰撞。"""
        return self.talt + self.layer * (self.slot_id[i] - (self.n - 1) / 2.0)

    def _dive_alt(self, i):
        """終端俯衝微高度分層(防撞):收斂同一水平點 T,但各機差 dive_layer 垂直。"""
        return self.dive_alt + self.dive_layer * (self.slot_id[i] - (self.n - 1) / 2.0)

    def _line_alt(self, i, d_aim):
        """瞄準點在 dive_angle 下滑線上的高度。
        d_aim = 瞄準點與 T 的水平距(T 本身 = 0;穿越點在 T 後方 → 傳負值)。

        ⚠️ 關鍵:高度要算「瞄準點自己位置」的線上高度,不是「飛機當下位置」的。
        用飛機當下的 d 去算(alt = d·tan45 ≈ 現在高度)等於命令「飛到瞄準點時
        保持現在高度」= 幾乎平飛,飛機浮在線上方、到近端才被迫暴拉 → 正是
        「高空掠過紅心」的真因。瞄準點在線上時,ArduPlane GUIDED 的高度斜坡
        (prev_WP=當前位置 → next_WP)本身就是那條 45° 直線,落後時斜率自動變陡。"""
        lay = self.dive_layer * (self.slot_id[i] - (self.n - 1) / 2.0)
        a = self.dive_alt + lay + d_aim * math.tan(math.radians(self.dive_angle))
        # 相對高度剛好 0.0 會被 Location::sanitize() 判為「未設定」→ 指令被拒
        return a if abs(a) > 0.5 else -0.5

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
                if i not in self.perch_arr and d < self.pushover_r:
                    self.perch_arr[i] = now              # 已越 pushover 環
                    self.get_logger().info(
                        f"UAV{i} 就位對準環 @+{now-self.t0:.1f}s (alt {self.pos[i][2]:.0f}m)")
                if i in self.perch_arr:
                    # 就位後續朝穿越點慢飛(保持對準內切、不繞圈)→ DIVE 為已對準直線穿越。
                    # (曾試「守在環上盤旋等齊」→ 噴射盤旋掉高、穿越偏 320m,反而更糟。)
                    mode = "aligned"; self._goto(i, *thru, self.perch_alt, self.vmin)
                else:
                    t_rem = self.perch_T - now
                    v = max(self.vmin, min(self.vmax, (d - self.pushover_r) / max(t_rem, 0.1)))
                    mode = "runup"; self._goto(i, *thru, self.perch_alt, v)
                stat[i] = {"mode": mode, "d": round(d), "z": round(self.pos[i][2])}
                self._row(now, i, d, mode, thru)
            self._csv.flush()
            if len(self.perch_arr) == self.n:
                zs = {i: round(self.pos[i][2]) for i in self.pos}
                self.get_logger().warn(
                    f"★ 全員對準內切(alt {zs})→ DIVE 直線俯衝穿越!")
                self.phase = "DIVE"; self.dive_go = now
                dmax = max(dist_m(self.pos[i][:2], self.T) for i in self.pos)
                # ⚠️ 配速要用「實際可達俯衝速度」(泡棉俯衝 ~0.8·dive_speed),不能用
                # dive_speed 本身:否則 t_rem 立刻歸零、全員爆 v_req=dive_speed → 無法
                # 讓前機等後機 = 落單。用可達速配 → 近機 v_req 偏低被放慢 → 等齊。
                self.dive_pace = 0.8 * self.dive_speed
                self.dive_T = now + dmax / self.dive_pace + 1.0

        elif self.phase == "DIVE":
            done = 0
            for i in sorted(self.pos):
                d = dist_m(self.pos[i][:2], self.T)
                thru = self.thru[self.slot_id[i]]   # 穿越瞄準點(T 後方)
                if d < self.dmin.get(i, 1e9):
                    self.dmin[i] = d                # 追蹤真實最近水平距
                # 一旦進入 switch_r 就「鎖定穿越」(committed):此後永遠瞄穿越點,
                # 不因穿過後 d 回升 > switch_r 又翻回瞄 T → 修掉繞圈 re-attack。
                if not self.commit.get(i) and d < self.switch_r:
                    self.commit[i] = True
                # 瞄準點與其「線上高度」必須成對決定:只換瞄準點卻沿用舊高度,
                # 會把 45° 下滑線壓平成 ~9°(切換瞬間目標變遠、高度沒跟著降)。
                if self.commit.get(i):
                    aimpt = thru; alt = self._line_alt(i, -self.overshoot)
                else:
                    aimpt = self.T; alt = self._line_alt(i, 0.0)
                # 衝擊判定:進 impact_reach,或已 committed 且通過最近點(d 回升)→ 記為穿越
                passed = self.commit.get(i) and d > self.dmin[i] + 8.0
                if i not in self.impact and (d < self.impact_reach or passed):
                    self.impact[i] = now
                    self.hit_alt[i] = self.pos[i][2]   # 命中高度(判定是否真俯衝到紅心)
                    self.get_logger().info(
                        f"UAV{i} 命中紅心 @+{now-self.dive_go:.2f}s (最近 {self.dmin[i]:.0f}m "
                        f"命中高度 {self.pos[i][2]:.0f}m gs {self.gs.get(i,0):.0f}m/s)")
                if i in self.impact:
                    done += 1; mode = "expended" if self.expend else "impact"
                    if not self.expend:
                        # 保留機體 → 續飛穿越點 egress(俯衝速度),不繞 T
                        self._goto(i, *thru, alt, self.dive_speed)
                    # expend_on_impact:命中即視為消耗,停止下令(不脫離、不拉起)
                else:
                    # 俯衝砸向 T:指令上限 dive_speed(> 頂速 → TECS 壓機頭俯衝加速);
                    # 近機(會早到)v_req 偏低 → 放慢到最低 0.62·dive_speed 等後機(仍在俯衝)
                    t_rem = self.dive_T - now
                    v_req = d / max(t_rem, 0.05)
                    v = max(0.62 * self.dive_speed, min(self.dive_speed, v_req))
                    mode = "dive"
                    # 鎖存:整段俯衝只發一次航點(key = 是否已 commit),讓 ArduPlane
                    # 把 prev_WP→next_WP 的高度斜坡整條飛完 = 真正的 45° 直線俯衝
                    self._goto_latched(i, *aimpt, alt, v,
                                       key=("thru" if self.commit.get(i) else "T"))
                stat[i] = {"mode": mode, "d": round(d), "alt": round(alt),
                           "z": round(self.pos[i][2]), "gs": round(self.gs.get(i, 0))}
                self._row(now, i, d, mode, self.T)
            self._csv.flush()
            if done == self.n:
                imp = [self.impact[i] - self.dive_go for i in self.impact]
                spread = max(imp) - min(imp)
                dmins = {i: round(self.dmin[i], 1) for i in sorted(self.impact)}
                halts = {i: round(self.hit_alt.get(i, -1)) for i in sorted(self.impact)}
                self.get_logger().warn(
                    f"★★ 俯衝命中紅心:全員命中。命中時間散布 = {spread:.2f}s "
                    f"(≈{spread*self.dive_speed:.1f}m 縱向間隔);最近水平距 = {dmins} m;"
                    f"命中高度 = {halts} m(俯衝角 {self.dive_angle:.0f}°)")
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
