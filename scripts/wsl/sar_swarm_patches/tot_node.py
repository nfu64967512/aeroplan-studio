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

狀態機:ALIGN(TOT 對齊扇形 slot)→ SALVO(自 slot 沿切線同時衝 T,高度分層防撞)。
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
        g("approach_from", 180.0); g("r_fan", 2000.0); g("half_angle", 30.0)
        g("vmin", 14.0); g("vmax", 30.0); g("plan_frac", 0.9); g("margin", 40.0)
        g("reach", 120.0); g("salvo_reach", 150.0); g("layer_m", 12.0)
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
        self.salvo_reach = float(p("salvo_reach").value)
        self.layer = float(p("layer_m").value)

        # ── 扇形 slot(以 T 為圓心、r_fan、approach±half)──
        self.slots = []
        for k in range(self.n):
            off = -self.half + 2 * self.half * k / (self.n - 1)
            self.slots.append(dest(self.T[0], self.T[1],
                                   (self.approach + off) % 360.0, self.r_fan))
        self.spacing = self.r_fan * math.radians(2 * self.half) / (self.n - 1)

        # ── ① MAVLink 分流連線(每機一條)──
        self.link, self.pos, self.gs = {}, {}, {}
        self.slot_of, self.slot_id, self.arr = {}, {}, {}
        self.last_v, self.last_v_t = {}, {}
        for i in range(1, self.n + 1):
            c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{6760 + 10 * (i - 1)}",
                                           source_system=233, dialect="ardupilotmega")
            c.wait_heartbeat(timeout=8)
            self.link[i] = c
            self.last_v[i] = 0.0; self.last_v_t[i] = 0.0

        self.gate_pub = self.create_publisher(String, "/swarm/tot_gate", GATE_QOS)
        self.status_pub = self.create_publisher(String, "/swarm/tot_status", 10)
        self.phase = "INIT"
        self.t0 = None
        self.T_arr = None
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
        gf.close()
        self._publish_gate()
        self.create_timer(0.33, self._tick)   # 3 Hz 協同迴圈
        self.create_timer(1.0, self._publish_gate)  # latched 定期補發
        self.get_logger().info(
            f"tot_node up：扇形 R={self.r_fan:.0f} ±{self.half:.0f}° {self.n} slot "
            f"間隔 {self.spacing:.0f}m → 目標 {self.T}")

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
        c.mav.command_int_send(
            c.target_system or i, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
            float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))

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
                    mode = "hold"; self._goto(i, *self.slot_of[i], self.talt, self.vmin)
                else:
                    v_req = d / max(t_rem, 0.1)
                    if v_req > self.vmax:
                        mode = "max"; self._goto(i, *self.slot_of[i], self.talt, self.vmax)
                    elif v_req >= self.vmin:
                        mode = "straight"; self._goto(i, *self.slot_of[i], self.talt, v_req)
                    else:
                        # S 型機動:以航向偏角 θ 精確控制「前向閉合率」= v_req。
                        # 飛機以 vmin 空速朝偏 slot 方位 ±θ 的點飛 → 前向 = vmin·cosθ。
                        # 令 cosθ = v_req/vmin ⇒ θ = acos(v_req/vmin),則平均前向剛好
                        # v_req,恰在 T_arr 抵達(舊版固定側擺→前向失控、太早到 86s)。
                        mode = "weave"
                        ratio = max(0.05, min(0.985, v_req / self.vmin))
                        theta = math.degrees(math.acos(ratio))
                        sign = 1.0 if math.sin(2 * math.pi * (now - self.t0) / 22.0) >= 0 else -1.0
                        bs = bearing(self.pos[i][:2], self.slot_of[i])
                        aim = dest(self.pos[i][0], self.pos[i][1], bs + sign * theta, 120.0)
                        self._goto(i, aim[0], aim[1], self.talt, self.vmin)
                stat[i] = {"mode": mode, "d": round(d),
                           "arr": round(self.arr[i] - self.t0, 1) if i in self.arr else None}
                sl = self.slot_of[i]
                self._cw.writerow([round(now - self.t0, 1), i, self.pos[i][0],
                                   self.pos[i][1], self.pos[i][2], round(self.gs.get(i, 0), 1),
                                   round(d), mode, round(self.last_v[i], 1),
                                   sl[0], sl[1], self.phase])
            if len(self.arr) == self.n and self.phase == "ALIGN":
                got = [self.arr[i] - self.t0 for i in self.arr]
                self.get_logger().warn(
                    f"★ 全員抵扇形。抵達散布 = {max(got)-min(got):.1f}s → SALVO")
                self.phase = "SALVO"; self.salvo_t0 = now
                # 齊射:自 slot 沿切線同時衝 T(等距→等速→同時);高度分層防撞
                self.salvo_T_arr = now + self.r_fan / (self.plan_frac * self.vmax) + 8.0
        elif self.phase == "SALVO":
            done = 0
            for i in sorted(self.pos):
                d = dist_m(self.pos[i][:2], self.T)
                alt = self.talt + self.layer * (self.slot_id[i] - (self.n - 1) / 2.0)
                if d < self.salvo_reach:
                    done += 1; mode = "strike"
                    self._goto(i, *self.T, alt, self.vmin)
                else:
                    t_rem = self.salvo_T_arr - now
                    v = max(self.vmin, min(self.vmax, d / max(t_rem, 0.1)))
                    mode = "salvo"; self._goto(i, *self.T, alt, v)
                stat[i] = {"mode": mode, "d": round(d), "alt": round(alt)}
                self._cw.writerow([round(now - self.t0, 1), i, self.pos[i][0],
                                   self.pos[i][1], self.pos[i][2], round(self.gs.get(i, 0), 1),
                                   round(d), mode, round(self.last_v[i], 1),
                                   self.T[0], self.T[1], self.phase])
            self._csv.flush()
            if done == self.n:
                self.get_logger().warn("★★ 齊射:全員抵達目標 T。")
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
