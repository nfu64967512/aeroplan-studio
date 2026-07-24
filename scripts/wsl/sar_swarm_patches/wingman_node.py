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
import math

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from sar_swarm.geodesy import (dist_m, polar_slot, speed_for_error,
                               advance_along_heading, EARTH_M_PER_DEG)
from sar_swarm.master_role import SWITCH_QOS, HEARTBEAT_QOS
from sar_swarm.biomimetic_formation import (
    BiomimeticFormation, FormationParams, Agent)

# 後備固定 slot（無 whoLeader 時用，沿用原星型偏置）
SLOTS = {2: (-300.0, -200.0), 3: (-600.0, -400.0),
         4: (-300.0, 200.0), 5: (-600.0, 400.0)}

# 極座標 V（隨長機航向旋轉）：每架僚機 (rank 1=近/2=遠, side +1=左翼/-1=右翼)。
# UAV1 長機在 V 頂點、UAV6 在 V 尾（見 backup_node），合為 6 機一個 V。
V_RANK_SIDE = {2: (1, +1), 3: (2, +1), 4: (1, -1), 5: (2, -1)}


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
        self.declare_parameter("wp_interval", 3.0)     # 重發導引航點間隔；短=slot 新、僚機不進盤旋、跟得緊
        self.declare_parameter("wp_move_thresh", 25.0)  # slot 移動超過此(m)才重發
        self.declare_parameter("origin_lat", 22.60)
        self.declare_parameter("origin_lon", 120.00)
        # ── biomimetic 參數 ──
        self.declare_parameter("spacing_m", 250.0)
        self.declare_parameter("sep_radius_m", 60.0)
        self.declare_parameter("obstacle_margin_m", 80.0)
        self.declare_parameter("formation_shape", "V")
        # ── 極座標 V 參數（隨長機航向旋轉）──
        self.declare_parameter("v_spacing_m", 120.0)   # rank 間距（近 rank1、遠 rank2）
        self.declare_parameter("v_angle_deg", 25.0)    # V 半角（後左/後右 = 180±此角）
        # ── 編隊速度控制：落後加速追、到位回 nominal（讓快的等、慢的趕上）──
        self.declare_parameter("nominal_speed", 14.0)  # 隊形維持巡航空速 (m/s)
        self.declare_parameter("catch_speed", 20.0)    # 落後時的追趕空速
        self.declare_parameter("formed_radius", 250.0) # 距 slot < 此 = 到位（導引航點投遞的實際精度 ~100m）
        self.declare_parameter("catch_radius", 400.0)  # 距 slot > 此 = 全速追
        # slot 前饋秒數:目標 = slot 沿長機航向外推 lead×leader_speed。導引 WP 有
        # ~3-5s 有效滯後 → 高速巡航誤差地板 ≈ v×lag(27m/s 時 ~100-150m)怎麼調參
        # 都消不掉;前饋直接抵消。0 = 關閉(慢速編隊不需要)。
        self.declare_parameter("wp_lead_time", 0.0)
        self.declare_parameter("guided_min_alt", 15.0) # 高於此(m)才強制 GUIDED（修手動起飛後空盤旋）

        g = self.get_parameter
        self.uav_id = int(g("uav_id").value)
        self.master_id = int(g("master_id").value)
        self.n_uavs = int(g("n_uavs").value)
        self.slot = SLOTS.get(self.uav_id, (-300.0, 0.0))
        self.use_mavros = bool(g("use_mavros").value)
        self.use_biomimetic = bool(g("use_biomimetic").value)
        self.v_spacing = float(g("v_spacing_m").value)
        self.v_angle = float(g("v_angle_deg").value)
        self.leader = None     # /swarm/whoLeader: {id, lat, lon, alt, heading}
        self.nominal_speed = float(g("nominal_speed").value)
        self.catch_speed = float(g("catch_speed").value)
        self.formed_radius = float(g("formed_radius").value)
        self.catch_radius = float(g("catch_radius").value)
        self.wp_lead_time = float(g("wp_lead_time").value)
        self._guided_min_alt = float(g("guided_min_alt").value)
        self._last_speed = 0.0
        self._last_speed_t = 0.0
        self._last_guided_t = 0.0
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
            self.create_timer(1.0, self._fly)
            self.get_logger().info(f"UAV{self.uav_id} real mode: fly formation slot")

    # ── 訂閱回呼 ──────────────────────────────────────────────────────────
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
        except Exception:
            pass

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

    def _formation_control(self, tlat, tlon):
        """速度控制 + 回報誤差 + 起飛後保持 GUIDED。
        落後 slot 越多 → 飛越快追上；到位 → 回 nominal（與長機同速維持隊形）。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        my = self.fleet.get(self.uav_id)               # use_mavros 下 = 自己經緯
        err = dist_m(my, (tlat, tlon)) if my is not None else None
        # 到位後跟長機目前速度（長機在等隊形 → 一起慢；長機巡航 → 一起巡航），落後才衝 catch
        nominal = (self.leader.get("speed", self.nominal_speed)
                   if self.leader is not None else self.nominal_speed)
        spd = speed_for_error(err, self.formed_radius, self.catch_radius,
                              nominal, self.catch_speed)
        if abs(spd - self._last_speed) > 0.5 or (now - self._last_speed_t) > 3.0:
            self.mav.change_speed_async(spd)
            self._last_speed, self._last_speed_t = spd, now
        self.status_pub.publish(String(data=json.dumps(
            {"id": self.uav_id, "err": (err if err is not None else -1.0)})))
        self._keep_guided(now)

    def _keep_guided(self, now):
        """手動/自動起飛後若不在 GUIDED 就切回，否則 guided 航點不生效（會空盤旋）。"""
        p = self.mav.position()
        if (p is not None and p[2] > self._guided_min_alt
                and self.mav.mode() != "GUIDED" and (now - self._last_guided_t) > 2.0):
            self.mav.set_mode_async("GUIDED")
            self._last_guided_t = now
            self.get_logger().info(f"UAV{self.uav_id}: re-asserting GUIDED for formation")

    def _v_target(self):
        """極座標 V slot：以長機（whoLeader）航向為基準，永遠排在長機後方的固定方位。
        無 whoLeader → None（讓 caller 退回固定 slot）。"""
        L = self.leader
        if L is None:
            return None
        rank, side = V_RANK_SIDE.get(self.uav_id, (1, 0))
        distance = rank * self.v_spacing
        bearing_off = 180.0 + side * self.v_angle      # 後左=180+θ、後右=180-θ、正後=180
        lat, lon, _ = polar_slot(L["lat"], L["lon"], L["heading"], distance, bearing_off)
        # slot 前饋:導引 WP 瞄外推點抵消 ~3-5s 滯後;但回傳（＝算誤差用）的是真 slot,
        # 否則 err 含前置量 → formed 判定被污染（實測 err 平台正好卡在解除門檻上）。
        # §4.5 旋轉前饋:框在弧轉中(omega≠0)時,slot 的未來位置 = 長機沿弧前進
        # + slot 方位隨框旋轉 —— 線性外推會永遠指向弧外(實測弧轉 RMS 反而惡化)。
        ff = self.wp_lead_time * float(L.get("speed", 0.0) or 0.0)
        if ff > 0.0:
            om = float(L.get("omega", 0.0) or 0.0)         # 框轉率 deg/s
            if abs(om) > 0.2:                              # 弧轉:沿弧外推
                dpsi = om * self.wp_lead_time              # 前饋窗內框轉角
                adv = advance_along_heading(               # 長機沿弧的弦前進(中點航向)
                    L["lat"], L["lon"], L["heading"] + dpsi / 2.0, ff)
                flat, flon, _ = polar_slot(adv[0], adv[1],
                                           L["heading"] + dpsi,   # slot 用未來航向
                                           distance, bearing_off)
                self._ff_target = (flat, flon)
            else:                                          # 直線:原線性外推
                self._ff_target = advance_along_heading(lat, lon, L["heading"], ff)
        else:
            self._ff_target = None
        return (lat, lon)

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
        moved = (self._last_wp is None
                 or dist_m(self._last_wp, (wlat, wlon)) > self._wp_move_thresh)
        if moved and (now - self._last_wp_t) >= self._wp_interval:
            if self.mav.send_guided_waypoint(wlat, wlon, self.formation_alt):
                self._last_wp = (wlat, wlon)
                self._last_wp_t = now
                self.get_logger().info(
                    f"UAV{self.uav_id} -> {tag} -> "
                    f"({tlat:.5f},{tlon:.5f}) @ {self.formation_alt:.0f} m")


def main():
    rclpy.init()
    node = WingmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
