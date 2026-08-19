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

from sar_swarm.strike_authority import StrikeAuthority
from sar_swarm.geodesy import (dist_m, polar_slot, speed_for_error,
                               advance_along_heading, EARTH_M_PER_DEG)
from sar_swarm.master_role import (HEARTBEAT_QOS, SWITCH_QOS, DEEP_SUB_QOS,
                                   MasterPublisher)


class BackupNode(Node):
    def __init__(self):
        super().__init__("backup_node")
        self.declare_parameter("uav_id", 6)
        self.declare_parameter("master_id", 1)
        # ⚠️⚠️ 心跳逾時 = 「長機被判定失聯」的門檻,也就是接管的觸發條件。
        #    舊值 3.0 是**緊密編隊 + 短鏈路**時代的值。拉到 2–3 英里之後
        #    LoRa Mesh 的丟包率與延遲呈指數上升,3 s 的窗口會被單純的通訊抖動
        #    打穿 ⇒ backup 誤判長機死亡 → 接管 → 長機心跳恢復 → 讓權 → 再誤判…
        #    這就是 leader thrashing,而它比「慢一點接管」危險得多:
        #    每次領導權易手,四架僚機的 whoLeader.id 都跟著換,整個 V 重組一次。
        #    放到 9.0 s(使用者指定 8–10 s 區間)。
        #    ⚠️ 代價要正視:長機**真的**失效時,接管會晚 6 s。以 157 m/s 計
        #    等於多飛 942 m 無人領隊 —— 僚機在這段期間會走 Dead Reckoning
        #    (dr_max 45 s),所以編隊不會散,這個代價是可接受的。
        #    兩者刻意配對:dr_max(45)≫ timeout(9),外推撐得過選舉窗口。
        self.declare_parameter("timeout", 9.0)
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
        self.declare_parameter("v_tail_dist_m", 6000.0)   # 極座標 V 尾：長機正後方此距離（隨航向轉）
        self.declare_parameter("wp_interval", 1.0)        # 重發導引航點間隔（同 wingman）
        self.declare_parameter("wp_move_thresh", 40.0)
        # ── 編隊速度控制（同 wingman）──
        # ⚠️⚠️ 預設值一律是 **jet plane**(B-500 機體),不是泡棉機。
        #    泡棉機的 14/10/20 m/s 曾讓僚機被命令用 1/4 速度飛、物理上不可能跟上,
        #    間距單調拉開到 13.7 km。任何沒經過 stot_trial.sh 的路徑都會吃到預設,
        #    所以預設本身必須是可飛的。
        #    機體:W=215.8 N、c=0.003168、T=4.142×THR_MAX(21)=87.0 N
        #    巡航 135 EAS(地速 157)/ 追趕 157 EAS / 等待 90 EAS(用滿減速權限)
        self.declare_parameter("nominal_speed", 135.0)
        self.declare_parameter("catch_speed", 157.0)
        self.declare_parameter("formed_radius", 200.0)
        self.declare_parameter("catch_radius", 528.0)
        self.declare_parameter("wp_lead_time", 0.0)  # slot 前饋秒數(同 wingman,抵消 WP 滯後)
        # ── 修法 1/2/3 參數 ──
        self.declare_parameter("takeover_grace", 30.0)   # 啟動寬限秒數(冷開機容忍)
        self.declare_parameter("grace_timeout", 15.0)    # 寬限期內的心跳逾時門檻
        self.declare_parameter("expected_followers", 4)  # 佐證閘/手風琴的僚機數(2-5)
        self.declare_parameter("wait_speed", 90.0)       # 代理長機手風琴等待速度
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
        self._elig_warn_t = -1e9   # 資格閘告警節流(見 _eligible)
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
        # 打擊期間讓權給 tot_node(見 strike_authority 的事故說明)
        self._strike_auth = StrikeAuthority(self)
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
        # ⚠️⚠️ tot_node 打擊中 ⇒ 停止下令。AP2 實測沒有這一段時,本節點以 1 Hz
        #    重發 `UAV2 -> V-tail slot -> (…) @ 50 m`,把一架正要飛 48 km 去打擊
        #    的飛機釘在起飛點旁邊 50 m 高度繞圈。
        # ⭐ n=2 時備長**就是第 2 發彈**(使用者 2026-08-07 定調:
        #    「打擊期間備長是純打擊成員」)⇒ 這時候它該聽 tot_node,不是編隊層。
        if self._strike_auth.yielded():
            # ⚠️⚠️ **讓的是「位置權」,不是「模式維持」。**
            #    `_keep_guided()` 原本只在 `_formation_control()` 的最後一行被呼叫,
            #    而那一行在這個 return 的**下面** ⇒ 一讓權,模式維持就整個停掉。
            #    後果(RPF1 實測):互動路徑用 MANUAL=1 + swarm_takeoff.sh 起飛,
            #    飛控停在 TAKEOFF;讓權當下沒人把它切成 GUIDED,而 tot_node 的
            #    DO_REPOSITION 在非 GUIDED 下被飛控整發丟掉且**零告警**
            #    ⇒ UAV2 地速 148-164 m/s 看似正常,卻在桃園上空繞圈 1250 s。
            # ⚠️ 不要改成「讓航點自己帶 CHANGE_MODE」—— 那樣**編隊層的巡邏航點**
            #    會在起飛滾行時把飛機從 TAKEOFF 拽出來,起飛直接被取消
            #    (RPG/RPH/RPI 三批就是這樣報廢的,見 mavros_interface 的說明)。
            #    模式維持要留在這裡,因為 `_keep_guided()` 有 `p[2] > guided_min_alt`
            #    的離地守衛 —— 飛機還在地上時它什麼都不做,正是我們要的。
            self._keep_guided(self._now())
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
        """接管資格閘:防「聾機」與「地面機」誤搶領導權。

        原設計:近 5 s 聽到 ≥ min(2, expected_followers) 架僚機的
        formation_status 才可接管。`_heard` 在 `_on_status` 排除自己 ⇒ 只數僚機。

        ⚠️⚠️ 2026-08-07:`_roles` 改成 n>=2 就配備長之後,**n=2 沒有任何僚機**
           (角色是 長機1 + 備長2),`expected_followers = n-2 = 0`
           ⇒ `min(2, 0) = 0` ⇒ `fresh >= 0` 恆真
           ⇒ **佐證閘退化成無條件通過**,等於把防護整個關掉。
           而 n=2 配備長的**唯一理由**就是失效接管,所以也不能直接關掉接管。

        ⇒ 僚機數 < 2 時改用另一組佐證,保住同一個不變量
          (「聾機/地面機不得宣告」):
            ① 自己的定位有效(不是聾機、不是還沒收到 MAVROS 的機體)
            ② **曾經**收過長機心跳(不是冷開機就搶;真正的失效是「收過然後斷了」)
          兩者皆為真才有資格。冷開機時 ② 不成立 ⇒ 不會誤接管。

        ⚠️ n>=4 的路徑逐位元不變:expected = n-2 >= 2 ⇒ `min(2, expected) = 2`,
           與 2026-08-07 之前的寫死 4 給出的 `min(2,4) = 2` 相同。
        """
        fresh = sum(1 for (t, _) in self._heard.values() if now - t <= 5.0)
        need = min(2, self._expected_followers)
        if need >= 1:
            return fresh >= need
        # 僚機數 < 2(n <= 3)—— 沒有僚機可以作證,改用自身狀態 + 曾聽過長機
        own_fix_ok = self.my_fix is not None       # 不是聾機、MAVROS 有定位
        heard_leader = self.seen_hb                # 曾收過長機心跳(不是冷開機就搶)
        if not (own_fix_ok and heard_leader):
            if now - self._elig_warn_t > 10.0:
                self._elig_warn_t = now
                self.get_logger().warn(
                    f"UAV{self.uav_id} 接管資格未達(僚機數 0,改用自身佐證):"
                    f"定位有效={own_fix_ok} / 曾收過長機心跳={heard_leader}")
            return False
        return True

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
        # ⚠️⚠️ 打擊期間**暫停接管職責**(使用者 2026-08-07 定調)。
        #    理由不是「怕麻煩」:tot_node 活躍時長機的 whoLeader / 心跳本來就
        #    可能因為讓權而變稀疏,而長機「不動」是因為它在俯衝,**不是失效**。
        #    這時候接管只會製造第二個控制器 —— 正是這一整段要消滅的東西。
        #    ⚠️ 讓權會自己過期(5 s):tot_node 真的掛了 ⇒ 讓權解除 ⇒
        #    接管判定立刻恢復,所以這不是把防護關掉,是把它讓開。
        if self._strike_auth.yielded():
            self._hb_reset_on_resume = True
            return
        # ⚠️ 剛收回控制時,`last_hb` 可能已經舊了(讓權期間沒在更新的話),
        #    直接比較會在收回的當下誤判「心跳消失」而立刻接管。
        #    收回後把計時基準重置一次,給長機一個完整的 timeout 視窗。
        if getattr(self, "_hb_reset_on_resume", False):
            self._hb_reset_on_resume = False
            self.last_hb = self._now()
            self.get_logger().warn(
                "▶ 收回接管職責 —— 心跳計時基準已重置,長機有完整的 "
                "%.1f s 視窗證明自己還活著。" % self.timeout)
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
        對應 spec §3.3「接任後先直線」。

        ⚠️ 已經接管之後,tot_node 打擊中一樣要讓權 —— 代理長機也是控制器。"""
        if not self.taken_over:
            return
        if self._strike_auth.yielded():
            self._keep_guided(self._now())   # 同 _fly_standby:讓位置權,不讓模式維持
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
