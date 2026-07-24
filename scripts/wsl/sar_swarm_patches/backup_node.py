"""Task 4 — backup node (UAV 6).

Watches the master's heartbeat and takes over on either path:
  - silence: 3 s pass with no beat (master process dead), or
  - low health (P0-C fast path): heartbeats still arrive but carry
    health.gps_ok == False or health.batt < low_batt_threshold.
Takeover:
  1. broadcast Master_Switch announcing itself as the new master,
  2. activate its own master role — GeoPose @10 Hz + heartbeat @2 Hz.

While on standby (use_mavros) it does NOT just loiter — it flies the V tail: a polar
slot v_tail_dist directly behind the leader (from /swarm/whoLeader), rotating with
the leader heading, so UAV6 is part of the formation. On takeover it stops trailing,
holds as the new leader, and publishes whoLeader with its own id → the wingmen
re-target around UAV6 automatically.

It seeds its GeoPose from the last fix it heard from the old master, so the
formation reference does not jump on handover.

Topics
  sub  /uav{master}/geopose   geographic_msgs/GeoPoseStamped
  sub  /swarm/heartbeat       std_msgs/String
  pub  /swarm/master_switch   std_msgs/String                  on takeover
  (then, as master) /uav{id}/geopose + /swarm/heartbeat
"""
import json
import math

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from sar_swarm.geodesy import (dist_m, polar_slot, speed_for_error,
                               advance_along_heading, EARTH_M_PER_DEG)
from sar_swarm.master_role import (HEARTBEAT_QOS, SWITCH_QOS, DEEP_SUB_QOS,
                                   MasterPublisher)


class BackupNode(Node):
    def __init__(self):
        super().__init__("backup_node")
        self.declare_parameter("uav_id", 6)
        self.declare_parameter("master_id", 1)
        self.declare_parameter("timeout", 3.0)
        self.declare_parameter("low_batt_threshold", 0.20)  # 心跳活著但電量低於此 → 提前接管 (P0-C)
        self.declare_parameter("origin_lat", 22.60)
        self.declare_parameter("origin_lon", 120.00)
        self.declare_parameter("altitude", 1000.0)
        self.declare_parameter("speed", 30.0)
        self.declare_parameter("heading_deg", 90.0)
        self.declare_parameter("use_mavros", False)
        self.declare_parameter("auto_arm", True)          # False → launch 不 arm（操作員自己起飛）
        self.declare_parameter("auto_takeoff", True)      # False → launch 不起飛，只設 GUIDED
        self.declare_parameter("launch_type", "runway")   # "runway" | "catapult"
        self.declare_parameter("test_batt", -1.0)        # 測試覆寫自身電量（接管後 UAV6 回報用）
        # 待命時也排進編隊（V 形最後面），不是只在原地盤旋
        self.declare_parameter("formation_alt", 50.0)
        self.declare_parameter("layer_sep", 15.0)
        self.declare_parameter("slot_north", -900.0)     # 固定 slot 後備（無 whoLeader 時）
        self.declare_parameter("slot_east", 0.0)
        self.declare_parameter("v_tail_dist_m", 220.0)   # 極座標 V 尾：長機正後方此距離（隨航向轉）
        self.declare_parameter("wp_interval", 3.0)        # 重發導引航點間隔（同 wingman）
        self.declare_parameter("wp_move_thresh", 25.0)
        # ── 編隊速度控制（同 wingman）──
        self.declare_parameter("nominal_speed", 14.0)
        self.declare_parameter("catch_speed", 20.0)
        self.declare_parameter("formed_radius", 250.0)
        self.declare_parameter("catch_radius", 400.0)
        self.declare_parameter("wp_lead_time", 0.0)  # slot 前饋秒數(同 wingman,抵消 WP 滯後)
        # ── 修法 1/2/3 參數 ──
        self.declare_parameter("takeover_grace", 30.0)   # 啟動寬限秒數(冷開機容忍)
        self.declare_parameter("grace_timeout", 15.0)    # 寬限期內的心跳逾時門檻
        self.declare_parameter("expected_followers", 4)  # 佐證閘/手風琴的僚機數(2-5)
        self.declare_parameter("wait_speed", 10.0)       # 代理長機手風琴等待速度
        self.declare_parameter("formed_hysteresis", 1.4) # 成形解除倍數(同 master)
        self.declare_parameter("guided_min_alt", 15.0)

        g = self.get_parameter
        self.uav_id = int(g("uav_id").value)
        self.master_id = int(g("master_id").value)
        self.timeout = float(g("timeout").value)
        self.low_batt_threshold = float(g("low_batt_threshold").value)
        self.use_mavros = bool(g("use_mavros").value)
        # 編隊跟隨（待命時排 V 尾）：高度分層 + 固定 slot + 航點節流
        self.formation_alt = (float(g("formation_alt").value)
                              + (self.uav_id - 2) * float(g("layer_sep").value))
        self.slot = (float(g("slot_north").value), float(g("slot_east").value))
        self.v_tail_dist = float(g("v_tail_dist_m").value)
        self.wp_lead_time = float(g("wp_lead_time").value)
        self.leader = None     # /swarm/whoLeader: {id, lat, lon, alt, heading}
        self._wp_interval = float(g("wp_interval").value)
        self._wp_move_thresh = float(g("wp_move_thresh").value)
        self._last_wp = None
        self._last_wp_t = 0.0
        self.nominal_speed = float(g("nominal_speed").value)
        self.catch_speed = float(g("catch_speed").value)
        self.formed_radius = float(g("formed_radius").value)
        self.catch_radius = float(g("catch_radius").value)
        self._guided_min_alt = float(g("guided_min_alt").value)
        self._last_speed = 0.0
        self._last_speed_t = 0.0
        self._last_guided_t = 0.0
        self._operator_override = False  # RTL/AUTO 操作者接管中
        self.my_fix = None     # 自己經緯（算 V 尾編隊誤差用）
        self.status_pub = self.create_publisher(String, "/swarm/formation_status",
                                                HEARTBEAT_QOS)

        self.master = MasterPublisher(
            self, self.uav_id, float(g("origin_lat").value),
            float(g("origin_lon").value), float(g("altitude").value),
            float(g("speed").value), float(g("heading_deg").value),
            use_mavros=self.use_mavros)               # idle until takeover

        self.seen_hb = False
        self.last_hb = self._now()
        self.taken_over = False
        self.last_master_fix = None
        self.last_health = None        # 最近一次心跳帶的 master 健康度 {batt, gps_ok}

        # ── 修法 1:接管資格閘 ──────────────────────────────────────
        # (a) 啟動寬限:開機後 takeover_grace 秒內,心跳逾時門檻放大到
        #     grace_timeout(容忍長機冷開機 GPS/EKF 慢) —— 冷啟動誤接管的
        #     直接根因(實測三連拍重現)。
        # (b) 佐證閘:接管前近 5 s 內須聽到 ≥min(2, expected_followers) 架
        #     僚機的 formation_status(聾機/地面態不得宣告;僚機在地面
        #     不回報 → 地面永不接管,起飛後自然解鎖)。
        self._takeover_grace = float(g("takeover_grace").value)
        self._grace_timeout = float(g("grace_timeout").value)
        self._expected_followers = int(g("expected_followers").value)
        self._boot_t = self._now()
        self._heard = {}               # {wingman_id: last_t} 佐證閘用
        self._gate_log_t = 0.0
        self.create_subscription(String, "/swarm/formation_status",
                                 self._on_status, HEARTBEAT_QOS)
        # ── 修法 2:讓權回收(yield-back)──────────────────────────
        self._yield_since = None       # 接管後聽到健康原長機心跳的起算時刻
        # ── 修法 3:代理長機手風琴 ──────────────────────────────────
        self.wait_speed = float(g("wait_speed").value)
        self.formed_hyst = float(g("formed_hysteresis").value)
        self._acting_was_formed = False
        self._acting_speed = 0.0
        self._acting_speed_t = 0.0
        self._acting_hdg = None        # 接管後凍結的隊形航向(直線續航用)
        self._acting_wp_t = 0.0
        self.create_timer(1.0, self._acting_tick)

        self.create_subscription(GeoPoseStamped, f"/uav{self.master_id}/geopose",
                                 self.on_master_geopose, 10)
        self.create_subscription(String, "/swarm/whoLeader",
                                 self.on_wholeader, HEARTBEAT_QOS)
        self.create_subscription(String, "/swarm/heartbeat",
                                 self.on_heartbeat, DEEP_SUB_QOS)
        self.switch_pub = self.create_publisher(String, "/swarm/master_switch", SWITCH_QOS)
        self.create_timer(0.5, self.watchdog)
        self.get_logger().info(
            f"backup UAV{self.uav_id} up, watching UAV{self.master_id} "
            f"(timeout {self.timeout:.1f} s)")

        # real mode: get airborne now as a hot standby (GUIDED, holding). When the
        # watchdog fires it activates its master role and the wingmen reform around
        # UAV6's GeoPose — an airborne handover instead of a reference on the ground.
        self.fsm = None
        if self.use_mavros:
            from sar_swarm.flight import FlightFSM
            from sar_swarm.mavros_interface import MavrosInterface
            from sensor_msgs.msg import NavSatFix
            from rclpy.qos import QoSPresetProfiles
            self.mav = MavrosInterface(self, self.uav_id)
            self.create_subscription(
                NavSatFix, f"/uav{self.uav_id}/mavros/global_position/global",
                self._on_my_fix, QoSPresetProfiles.SENSOR_DATA.value)
            self.fsm = FlightFSM(self, self.mav, 50.0,
                                 auto_arm=bool(g("auto_arm").value),
                                 auto_takeoff=bool(g("auto_takeoff").value),
                                 catapult=(str(g("launch_type").value) == "catapult"))
            self.create_timer(0.2, self._fly_standby)
            self.get_logger().info(
                f"backup UAV{self.uav_id}: airborne hot standby (GUIDED), "
                f"trailing the V tail until takeover")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _slot_at(self, ref):
        """ref (lat,lon[,alt]) + 固定 slot 偏置 (north,east) m → 目標 lat/lon。"""
        lat, lon = ref[0], ref[1]
        n, e = self.slot
        return (lat + n / EARTH_M_PER_DEG,
                lon + e / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))

    def _on_my_fix(self, msg):
        if abs(msg.latitude) < 1e-3 and abs(msg.longitude) < 1e-3:
            return                          # 忽略 (0,0) 假定位
        self.my_fix = (msg.latitude, msg.longitude)

    def _formation_control(self, tlat, tlon):
        """同 wingman：離 V 尾 slot 越遠就加速追，到位回 nominal；回報誤差；保持 GUIDED。"""
        now = self._now()
        err = dist_m(self.my_fix, (tlat, tlon)) if self.my_fix is not None else None
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
        """起飛後若被切出 GUIDED（操作員手動 TAKEOFF/AUTO），重新宣告 GUIDED 讓導引航點生效。
        RTL/AUTO 白名單 = 操作者接管(同 wingman):不搶模式、停發 WP。"""
        p = self.mav.position()
        mode = self.mav.mode()
        if mode in ("RTL", "AUTO"):
            if not self._operator_override:
                self._operator_override = True
                self.get_logger().warn(
                    f"UAV{self.uav_id}: operator mode {mode} — standing down")
            return
        if self._operator_override:
            self._operator_override = False
            self.get_logger().info(f"UAV{self.uav_id}: back from operator mode — rejoining")
        if (p is not None and p[2] > self._guided_min_alt
                and mode != "GUIDED" and (now - self._last_guided_t) > 2.0):
            self.mav.set_mode_async("GUIDED")
            self._last_guided_t = now
            self.get_logger().info(f"UAV{self.uav_id}: re-asserting GUIDED for formation")

    def _fly_standby(self):
        # climb into GUIDED, then trail at the V tail — a polar slot directly behind
        # the leader (rotates with the leader heading). Still ready to lead instantly.
        if not self.fsm.update():
            return
        if self.taken_over:
            return                          # now master/leader — hold; wingmen trail us
        if self.leader is not None:         # 極座標 V 尾：長機正後方、隨長機航向旋轉
            L = self.leader
            tlat, tlon, _ = polar_slot(L["lat"], L["lon"], L["heading"],
                                       self.v_tail_dist, 180.0)
            # slot 前饋:導引 WP 瞄外推點抵消滯後;誤差(_formation_control)照舊對真 slot 量,
            # 否則 err 含前置量 → master 的 formed 判定被污染。
            # §4.5 旋轉前饋(同 wingman):框弧轉中沿弧外推,免系統性指向弧外。
            ff = self.wp_lead_time * float(L.get("speed", 0.0) or 0.0)
            if ff > 0.0:
                om = float(L.get("omega", 0.0) or 0.0)
                if abs(om) > 0.2:
                    dpsi = om * self.wp_lead_time
                    adv = advance_along_heading(
                        L["lat"], L["lon"], L["heading"] + dpsi / 2.0, ff)
                    wlat, wlon, _ = polar_slot(adv[0], adv[1],
                                               L["heading"] + dpsi,
                                               self.v_tail_dist, 180.0)
                else:
                    wlat, wlon = advance_along_heading(tlat, tlon, L["heading"], ff)
            else:
                wlat, wlon = tlat, tlon
        elif self.last_master_fix is not None:
            tlat, tlon = self._slot_at(self.last_master_fix)   # 後備：固定 slot
            wlat, wlon = tlat, tlon
        else:
            return                          # no leader reference yet
        self._formation_control(tlat, tlon)   # 速度匹配 + 回報 + 保持 GUIDED（每 tick）
        now = self._now()
        if self._operator_override:
            return          # 操作者接管中:DO_REPOSITION 會強切 GUIDED,停發
        moved = (self._last_wp is None
                 or dist_m(self._last_wp, (wlat, wlon)) > self._wp_move_thresh)
        if moved and (now - self._last_wp_t) >= self._wp_interval:
            if self.mav.send_guided_waypoint(wlat, wlon, self.formation_alt):
                self._last_wp = (wlat, wlon)
                self._last_wp_t = now
                self.get_logger().info(
                    f"UAV{self.uav_id} -> V-tail slot -> "
                    f"({tlat:.5f},{tlon:.5f}) @ {self.formation_alt:.0f} m")

    def on_master_geopose(self, msg: GeoPoseStamped):
        p = msg.pose.position
        self.last_master_fix = (p.latitude, p.longitude, p.altitude)

    def on_wholeader(self, msg: String):
        if self.taken_over:
            return                  # we are the leader now; don't trail anyone
        try:
            d = json.loads(msg.data)
            lat, lon = float(d["lat"]), float(d["lon"])
            if abs(lat) < 1e-3 and abs(lon) < 1e-3:
                return                      # 拒收 (0,0) 長機座標
            self.leader = {"id": int(d["id"]), "lat": lat, "lon": lon,
                           "alt": float(d["alt"]), "heading": float(d["heading"]),
                           "speed": float(d.get("speed", self.nominal_speed))}
        except Exception:
            pass

    def _on_status(self, msg: String):
        """佐證閘 + 代理手風琴共用:記錄各僚機 formation_status 新鮮度與誤差。"""
        try:
            d = json.loads(msg.data)
            i = int(d["id"])
            if i != self.uav_id:
                self._heard[i] = (self._now(), float(d.get("err", -1.0)))
        except (ValueError, KeyError):
            pass

    def _eligible(self, now):
        """修法 1(b):近 5 s 聽到 ≥min(2, expected) 架僚機才可接管(防聾機/地面誤搶)。"""
        fresh = sum(1 for (t, _) in self._heard.values() if now - t <= 5.0)
        return fresh >= min(2, self._expected_followers)

    def on_heartbeat(self, msg: String):
        m = json.loads(msg.data)
        mid = int(m["master_id"])
        if mid == self.uav_id:
            return                      # 自己的心跳(接管後也會收到)不作數
        if self.taken_over:
            # ── 修法 2:讓權回收 ── 原長機心跳回來且健康 → 穩定 3 s 就讓位
            if mid == self.master_id:
                h = m.get("health") or {}
                healthy = (h.get("gps_ok") is not False
                           and not (h.get("batt") is not None
                                    and h["batt"] < self.low_batt_threshold))
                now = self._now()
                if healthy:
                    if self._yield_since is None:
                        self._yield_since = now
                    elif (now - self._yield_since) >= 3.0:
                        self._yield_back()
                else:
                    self._yield_since = None
            return
        self.master_id = mid
        self.last_hb = self._now()
        self.seen_hb = True
        self.last_health = m.get("health")    # {"batt":..,"gps_ok":..} or None (P0-C)

    def _yield_back(self):
        """把領導權還給復活的原長機:停自己的 master 角色、回 V 尾僚機。"""
        self.get_logger().warn(
            f"master UAV{self.master_id} heartbeat healthy again — "
            f"UAV{self.uav_id} yielding leadership back")
        self.master.deactivate()
        self.master.heading_override = None
        self.taken_over = False
        self._yield_since = None
        self._acting_was_formed = False
        self._acting_hdg = None
        self.last_hb = self._now()           # 重新武裝 watchdog(乾淨起算)
        self.seen_hb = True
        self.switch_pub.publish(String(data=json.dumps(
            {"new_master_id": self.master_id, "stamp": self._now()})))

    def watchdog(self):
        if self.taken_over or not self.seen_hb:
            return
        now = self._now()
        # 修法 1(a):啟動寬限 — 開機 grace 秒內用放大的逾時門檻(容忍冷開機)
        in_grace = (now - self._boot_t) < self._takeover_grace
        eff_timeout = self._grace_timeout if in_grace else self.timeout
        gap = now - self.last_hb
        # 路徑 1:心跳消失
        if gap > eff_timeout:
            if self._eligible(now):
                self.take_over(f"no heartbeat for >{eff_timeout:.1f} s")
            elif (now - self._gate_log_t) > 5.0:
                self._gate_log_t = now
                self.get_logger().info(
                    f"heartbeat gone {gap:.1f} s but eligibility gate not met "
                    f"(fresh wingman reports < min(2,{self._expected_followers})) "
                    f"— standing by, not claiming")
            return
        # 路徑 2(P0-C 快路徑):健康過低 → 提前接管(同樣過資格閘)
        h = self.last_health
        if h is not None:
            batt = h.get("batt")
            gps_ok = h.get("gps_ok")
            if (gps_ok is False or (batt is not None and batt < self.low_batt_threshold)) \
                    and self._eligible(now):
                self.take_over(
                    f"master UAV{self.master_id} health too low "
                    f"(gps_ok={gps_ok} batt={batt})")

    def _acting_tick(self):
        """代理長機:(a) 手風琴速度控制(同 master 遲滯邏輯);(b) **持續帶隊**
        — 凍結接管當下的隊形航向、沿其直線續航(每 ~8 s 推前方 1500 m 航點)。
        沒有 (b) 時代理長機的 FCU 抱著舊航點原地盤旋 → 廣播航向持續旋轉 →
        slot 繞著它轉 → 僚機永遠在 1×slot 距圓環上追(驗收 P4 實測);
        對應 spec §3.3「接任後先直線」。"""
        if not self.taken_over:
            return
        mav0 = getattr(self, "mav", None)
        if mav0 is not None and mav0.mode() in ("RTL", "AUTO"):
            return          # 代理長機本身被操作者接管(返航/降落)→ 停帶隊
        now = self._now()
        # ── (b) 直線續航 ──
        if self._acting_hdg is None:       # 接管後第一拍:凍結最後隊形航向
            if self.leader is not None:
                self._acting_hdg = float(self.leader.get("heading", 0.0))
            else:
                self._acting_hdg = float(getattr(self.master, "_heading_deg", 0.0))
            self.master.heading_override = self._acting_hdg
            self.get_logger().info(
                f"acting-leader: continuing straight on frozen heading "
                f"{self._acting_hdg:.0f}° (§3.3 直線續航)")
        if (now - self._acting_wp_t) > 8.0 and self.my_fix is not None:
            tgt = advance_along_heading(self.my_fix[0], self.my_fix[1],
                                        self._acting_hdg, 1500.0)
            mav = getattr(self, "mav", None)
            if mav is not None and mav.send_guided_waypoint(
                    tgt[0], tgt[1], self.formation_alt):
                self._acting_wp_t = now
        # ── (a) 手風琴 ──
        fresh = [e for (t, e) in self._heard.values()
                 if e >= 0 and (now - t) <= 5.0]
        worst = max(fresh) if fresh else None
        if self._acting_was_formed:
            formed = (worst is not None
                      and worst <= self.formed_radius * self.formed_hyst
                      and len(fresh) >= self._expected_followers - 1)
        else:
            formed = (len(fresh) >= self._expected_followers
                      and worst is not None and worst <= self.formed_radius)
        spd = self.nominal_speed if formed else self.wait_speed
        self.master.cmd_speed = spd          # 廣播給僚機(到位跟這個速度)
        if formed != self._acting_was_formed:
            self.get_logger().info(
                f"acting-leader accordion: {'formed — nominal' if formed else 'waiting'} "
                f"{spd:.0f} m/s (worst {worst if worst is not None else -1:.0f} m, "
                f"fresh {len(fresh)}/{self._expected_followers})")
        self._acting_was_formed = formed
        if abs(spd - self._acting_speed) > 0.5 or (now - self._acting_speed_t) > 3.0:
            mav = getattr(self, "mav", None)     # sim 模式無 mav
            if mav is not None:
                mav.change_speed_async(spd)
            self._acting_speed, self._acting_speed_t = spd, now

    def take_over(self, reason="master presumed down"):
        self.taken_over = True
        self.get_logger().warn(
            f"{reason} -- UAV{self.master_id} -> UAV{self.uav_id} taking over as master")

        # announce the switch (latched, reliable)
        self.switch_pub.publish(String(data=json.dumps(
            {"new_master_id": self.uav_id, "stamp": self._now()})))

        # continue the GeoPose from the last known master fix, then go active
        if self.last_master_fix is not None:
            self.master.lat, self.master.lon, self.master.alt = self.last_master_fix
        self.master.activate()
        self.get_logger().info(
            f"UAV{self.uav_id} now master: GeoPose @10 Hz, heartbeat @2 Hz")


def main():
    rclpy.init()
    node = BackupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
