"""Shared master-role publisher: GeoPose at 10 Hz + heartbeat at 2 Hz.

master_node owns this and runs it active from start. backup_node owns one too but
keeps it idle until the watchdog fires, then activates it to take over. Same code
path either way, so the backup becomes a real master, not a stand-in.

GeoPose source:
  - sim mode (default): a simulated moving fix (origin + course + speed), runs
    without SITL.
  - MAVROS mode (use_mavros): the real fix from /uav{id}/mavros/global_position/
    global (sensor_msgs/NavSatFix), so GeoPose follows the actual SITL aircraft.

Topics
  pub  /uav{id}/geopose   geographic_msgs/GeoPoseStamped   own lat/lon/alt @ 10 Hz
  pub  /swarm/whoLeader   std_msgs/String   {id, lat, lon, alt, heading} @ 10 Hz
  pub  /swarm/heartbeat   std_msgs/String        {master_id, seq, health} @ 2 Hz
  sub  /uav{id}/mavros/global_position/global    sensor_msgs/NavSatFix    (MAVROS mode)
  sub  /uav{id}/mavros/global_position/compass_hdg  std_msgs/Float64      (MAVROS mode)
  sub  /uav{id}/mavros/battery                 sensor_msgs/BatteryState (MAVROS mode)

/swarm/whoLeader carries the CURRENT leader's state + id (heading included) so wingmen
fly a leader-heading-relative polar V (rotates with the leader, always behind it). When
the backup takes over it publishes whoLeader with its own id, so followers re-target
the new leader automatically — the leader role is whatever whoLeader.id says.

heartbeat health = {"batt": <0~1 or null>, "gps_ok": <bool or null>} so the backup
can take over early when the master's own health drops, not only when it goes silent.
"""
from __future__ import annotations

import json
import math

from geographic_msgs.msg import GeoPoseStamped
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSPresetProfiles,
                       QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String

EARTH_M_PER_DEG = 111320.0

# heartbeat: lossy-link friendly, only the freshest beat matters
HEARTBEAT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=1,
                           durability=DurabilityPolicy.VOLATILE)
# master switch: latched + reliable so a node that missed it still gets the last one
SWITCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=1,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
# 訂閱端加深快取:同一 topic 有多個發布者(master+backup 都發 heartbeat/
# whoLeader)時,depth-1 KEEP_LAST 會被「自己的樣本」蓋掉對方的 → backup
# 永遠聽不到 master 心跳(讓權失效)、master 誤判 foreign 消失(週期振盪)。
# 訂閱端深度與發布端獨立且相容;BEST_EFFORT 保留。
DEEP_SUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=8,
                          durability=DurabilityPolicy.VOLATILE)


class MasterPublisher:
    def __init__(self, node, uav_id, origin_lat, origin_lon, altitude,
                 speed, heading_deg, use_mavros=False):
        self.node = node
        self.uav_id = uav_id
        self.lat = origin_lat
        self.lon = origin_lon
        self.alt = altitude
        self.speed = speed
        self.heading = math.radians(heading_deg)   # from North, CW
        self._heading_deg = float(heading_deg)      # leader heading broadcast on whoLeader
        self.cmd_speed = 14.0                       # 長機目前指令空速，廣播給僚機「到位時跟著這個速度」
        self.turn_rate = 0.0                        # 框轉率 deg/s(§4.5 弧轉中 = ±v/R;直線 0)
                                                    # 廣播給僚機做「旋轉前饋」— 線性前饋在持續
                                                    # 旋轉的框上永遠指向弧外(系統性偏差)
        self.heading_override = None                # 不為 None 時用此航向廣播（轉角盤旋等隊形時凍結方位，
                                                    # slot 才不會跟著盤旋亂轉、僚機能穩穩排成下一段的 V）
        self.use_mavros = use_mavros
        self.active = False
        # 自我降權(防腦裂):偵測到別機的 whoLeader 時設 True —— 停發
        # whoLeader/GeoPose(不與代理長機爭權),但**心跳照發**(帶健康),
        # 讓代理長機(backup)看得到「原長機活著且健康」而把領導權讓回來。
        self.suspend_who = False
        self.seq = 0
        self._fix = None                            # latest MAVROS fix
        self._batt = None                           # 0~1, None=unknown (health)
        self._gps_ok = None                         # bool, None=unknown (health)

        self.geopose_pub = node.create_publisher(
            GeoPoseStamped, f"/uav{uav_id}/geopose", 10)
        self.who_pub = node.create_publisher(String, "/swarm/whoLeader", HEARTBEAT_QOS)
        self.hb_pub = node.create_publisher(String, "/swarm/heartbeat", HEARTBEAT_QOS)

        if use_mavros:
            from sensor_msgs.msg import NavSatFix, BatteryState
            from std_msgs.msg import Float64
            node.create_subscription(
                NavSatFix, f"/uav{uav_id}/mavros/global_position/global",
                self._on_fix, QoSPresetProfiles.SENSOR_DATA.value)
            node.create_subscription(
                Float64, f"/uav{uav_id}/mavros/global_position/compass_hdg",
                self._on_hdg, QoSPresetProfiles.SENSOR_DATA.value)
            node.create_subscription(
                BatteryState, f"/uav{uav_id}/mavros/battery",
                self._on_batt, QoSPresetProfiles.SENSOR_DATA.value)

        node.create_timer(0.1, self._geopose_tick)     # 10 Hz
        node.create_timer(0.5, self._heartbeat_tick)   # 2 Hz

    def activate(self):
        self.active = True

    def deactivate(self):
        """Step down: stop publishing GeoPose/whoLeader/heartbeat (avoids dual authority)."""
        self.active = False

    def _on_fix(self, msg):
        # Ignore the (0,0) fixes MAVROS emits before the EKF global origin locks (or
        # on a transient GPS glitch). Broadcasting one on whoLeader would compute every
        # follower's slot near the equator and fling the whole formation off toward (0,0).
        if abs(msg.latitude) < 1e-3 and abs(msg.longitude) < 1e-3:
            return
        self._fix = (msg.latitude, msg.longitude, msg.altitude)
        # 收到有效（非 0,0）定位 = GPS 正在運作 → gps_ok=True。
        # 不再信任 MAVROS 的 NavSatFix.status 欄位：ArduPilot SITL / 部分實機的
        # global_position/global 常回 status=-1（STATUS_NO_FIX）即使定位良好，
        # 舊版 `status>=0` 會誤判 gps_ok=False，讓備長（UAV6）在長機明明有定位時
        # 就走「低健康快路徑」接管 → 一開機就變 6 號當長機（與假 0% 電量同類 bug）。
        # 真正 GPS 失效時 MAVROS 會停送有效定位，由心跳逾時路徑接管，不靠此欄位。
        self._gps_ok = True

    def _on_hdg(self, msg):
        self._heading_deg = float(msg.data)           # compass heading, deg from North CW

    def _on_batt(self, msg):
        # sensor_msgs/BatteryState.percentage 0~1；nan/負值/0 = 未知。
        # percentage<=0 視為「沒算出百分比」而非真的沒電：ArduPlane SITL（未設電池容量）
        # 常回 voltage=12.5V 但 percentage=0.0，舊版會讓備長因為假的 0% 一起飛就誤接管。
        # 真的低電量是逐漸下降的正小數（0.05/0.02…）會在 <threshold 時觸發，不會卡在剛好 0。
        p = msg.percentage
        if p is None or math.isnan(p) or p <= 0.0:
            self._batt = None
        else:
            self._batt = max(0.0, min(1.0, float(p)))

    def _eff_batt(self):
        """test_batt >= 0（owning node 參數）取代真實電量，供低健康接管驗證。"""
        try:
            tb = float(self.node.get_parameter("test_batt").value)
        except Exception:
            tb = -1.0
        return tb if tb >= 0.0 else self._batt

    def _geopose_tick(self):
        if not self.active or self.suspend_who:
            return          # suspend:讓權模式 — whoLeader/GeoPose 停發(心跳照發)

        if self.use_mavros:
            if self._fix is None:
                return                              # wait for the first MAVROS fix
            self.lat, self.lon, self.alt = self._fix
        else:
            dt = 0.1
            vn = self.speed * math.cos(self.heading)
            ve = self.speed * math.sin(self.heading)
            self.lat += (vn * dt) / EARTH_M_PER_DEG
            self.lon += (ve * dt) / (EARTH_M_PER_DEG * math.cos(math.radians(self.lat)))

        msg = GeoPoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "wgs84"
        msg.pose.position.latitude = self.lat
        msg.pose.position.longitude = self.lon
        msg.pose.position.altitude = self.alt
        msg.pose.orientation.w = 1.0
        self.geopose_pub.publish(msg)

        # whoLeader: 當前長機完整狀態 + id + 航向，供僚機算「隨長機轉向的極座標 V」
        hdg = self.heading_override if self.heading_override is not None else self._heading_deg
        self.who_pub.publish(String(data=json.dumps(
            {"id": self.uav_id, "lat": self.lat, "lon": self.lon,
             "alt": self.alt, "heading": hdg,
             "speed": self.cmd_speed, "omega": self.turn_rate})))

    def _heartbeat_tick(self):
        if not self.active:
            return
        self.seq += 1
        # health 欄位讓 backup 走低健康快路徑接管 (P0-C)
        self.hb_pub.publish(String(data=json.dumps(
            {"master_id": self.uav_id, "seq": self.seq,
             "stamp": self.node.get_clock().now().nanoseconds * 1e-9,
             "health": {"batt": self._eff_batt(), "gps_ok": self._gps_ok}})))
