"""Task 2 — master node (UAV 1).

Publishes its own GeoPose at 10 Hz and a heartbeat at 2 Hz. The GeoPose is the
formation reference the wingmen track; the heartbeat is what the backup watches.

Here the GeoPose is a simulated moving fix (origin + course + speed) so the node
runs without SITL. For a real flight, replace _sim with a subscription to
/uav{id}/mavros/global_position/global (sensor_msgs/NavSatFix) and copy it through.

Params: uav_id, origin_lat, origin_lon, altitude, speed, heading_deg
"""
import json
import math
from sar_swarm.geodesy import dist_m
from sar_swarm.strike_authority import StrikeAuthority

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from sar_swarm.master_role import (HEARTBEAT_QOS, SWITCH_QOS, DEEP_SUB_QOS,
                                   MasterPublisher)




class MasterNode(Node):
    def __init__(self):
        super().__init__("master_node")
        self.declare_parameter("uav_id", 1)
        self.declare_parameter("origin_lat", 22.60)
        self.declare_parameter("origin_lon", 120.00)
        self.declare_parameter("altitude", 1000.0)
        self.declare_parameter("speed", 30.0)
        self.declare_parameter("heading_deg", 90.0)     # due East
        self.declare_parameter("use_mavros", False)     # source GeoPose from MAVROS
        self.declare_parameter("test_batt", -1.0)        # 測試覆寫自身電量（>=0 取代真實；可 param set）
        self.declare_parameter("auto_guided", True)     # switch FCU to GUIDED on start
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("auto_takeoff", True)
        self.declare_parameter("takeoff_alt", 50.0)
        self.declare_parameter("launch_type", "runway")   # "runway" | "catapult"
        self.declare_parameter("patrol_alt", 80.0)        # leader cruise alt for the patrol
        self.declare_parameter("patrol_box_m", 1500.0)    # patrol 方框邊長 m（大＝直線長、少轉角）
        # ── §4.5 轉彎程序(轉前減速 + 定大半徑 fillet 弧)──
        # 尖角轉彎 = slot 以機體 bank 極限的角速率(~20°/s)繞長機掃掠,外側僚機
        # 需求速度遠超 v_catch → 轉角散隊(點雲實測)。改為半徑 R 的圓弧:
        # slot 掃掠率 ω=v_corner/R(R=800、v=16 → ~1.1°/s),外側 slot 需求速度
        # = v·d_max/R(雙側式,S=300 幾何 → ~21 ≤ v_catch 31 ✓)。
        self.declare_parameter("turn_radius", 800.0)      # 弧半徑 m [推測,雙側式求]
        self.declare_parameter("preslow_margin", 250.0)   # 入彎點前多少 m 開始減速 [推測]
        self.declare_parameter("patrol_capture", 200.0)   # advance to next leg within this (m)
        self.declare_parameter("auto_patrol", True)       # False=operator-driven (no auto patrol)
        # 到了轉角先別轉：等隊形重新成形（或逾時）再轉，免得長機急轉把僚機甩在外圈
        self.declare_parameter("patrol_hold_timeout", 60.0)
        # ── 編隊速度控制（長機端）──：僚機沒到位就放慢等，全部到位再回正常巡航
        # ⚠️⚠️ 預設值一律是 **jet plane**(B-500 機體),不是泡棉機。
        #    泡棉機的 14/10/20 m/s 曾讓僚機被命令用 1/4 速度飛、物理上不可能跟上,
        #    間距單調拉開到 13.7 km。任何沒經過 stot_trial.sh 的路徑都會吃到預設,
        #    所以預設本身必須是可飛的。
        #    機體:W=215.8 N、c=0.003168、T=4.142×THR_MAX(21)=87.0 N
        #    巡航 135 EAS(地速 157)/ 追趕 157 EAS / 等待 90 EAS(用滿減速權限)
        self.declare_parameter("nominal_speed", 135.0)     # 隊形成立後的巡航速度
        self.declare_parameter("wait_speed", 90.0)        # 有人落後時長機放慢等
        # 僚機誤差 <= 此值算到位。⚠️ 這個值是**相對於 v_spacing** 才有意義:
        #   緊密編隊 v_spacing=120 之下 250 比間距還大 ⇒ 永遠算到位,隊形收不緊;
        #   寬間距 v_spacing=4000 之下 200 是間距的 5%,是合理的到位判準。
        #   200 m 也和 3 英里間距的動態雜訊量級相符(GPS + WP 滯後 + 陣風)。
        #   ⚠️ 不要為了「看起來精準」而設小:設 20 m 會讓 formed 永遠不成立,
        #   長機一路停在 wait_speed,整個編隊被自己的門檻拖住。
        self.declare_parameter("formed_radius", 200.0)
        # 成形後的解除門檻 = formed_radius × 此倍數。巡航中 slot 追蹤誤差會比等待時
        # 膨脹（WP 滯後 + TECS 落後，實測 +~70m）；倍數太小（1.4）會一釋放就被打回
        # wait，長機在 wait/cruise 間震盪、群速卡在 wait_speed。放寬到 ~1.8 讓巡航撐得住。
        self.declare_parameter("formed_hysteresis", 1.4)
        # ── Max-Consensus 速度對齊 ────────────────────────────────────────
        # ⚠️⚠️ 舊行為是**二值**切換(formed ? nominal : wait),等於只用了「加速側」
        #    的權限:落後機自己追,長機要嘛全速要嘛掉到 wait 一格。
        #    但低推重比固定翼的速度權限是**不對稱**的 —— 減速靠阻力,加速靠剩餘推力:
        #      巡航 135 EAS:加速側可用 +21(到 156);減速側可用 −45(到 90)
        #      減速加速度 −cV²/m = −2.62 m/s² vs 加速 +1.33 m/s² ⇒ **快 2 倍**
        #    換算成修正 167 s 抵達散布所需的航程:
        #      只讓落後機加速      185.3 km
        #      只讓領先機減速       53.2 km
        #      兩邊同時用           41.3 km   ← 現有的 40 km 交棒段就夠
        #    所以改成「全隊向**最慢者**對齊」(max-consensus):長機主動讓出速度,
        #    讓出的量由最差僚機的落後量連續決定,而不是掉進一個固定的 wait 檔位。
        #    參考:arXiv 2607.14781 的 max-consensus 同時到達(Tᵢᵈ = max_j Tⱼ)。
        self.declare_parameter("catch_speed", 157.0)       # 僚機的追趕上限(算閉合率用)
        self.declare_parameter("close_time", 60.0)        # 期望把最差落後量收掉的秒數
        self.declare_parameter("max_consensus", True)
        # ⭐⭐ 高度閘門:爬升段**強制關閉** max-consensus。
        #    實測 AMB(3 s 彈射間隔):六機幾乎同時離地 ⇒ 長機立刻偵測到僚機落後
        #    ⇒ 主動減速等待,而它是在**爬升途中**減速的 ⇒ 爬升率跟著掉。
        #    UAV1 的爬升航程因此從 27.4 km 惡化到 **49.6 km**(+78%)。
        #    協同讓速與爬升搶同一份剩餘推力(T/W 僅 0.403),必須時序上隔開:
        #      alt < 門檻 → CLIMBING:全力爬,不下任何減速指令
        #      alt ≥ 門檻 → CRUISE_FORMATION:解鎖速度對齊
        # ⚠️⚠️ **master 端的高度閘門預設關閉(0.0)**,與 wingman 端分開。
        #    實測 AMD1:master+wingman 一起開閘門,散布 62.7 → 188.2 s。
        #    原因是 master 端閘門會在爬升段(28 km / 280 s)關掉手風琴,
        #    而**缺口正是在那一段形成的** —— 長機全速跑掉、僚機拉開。
        #    手風琴必須在缺口形成的當下就壓制它。
        #    wingman 端閘門是另一回事(把追不到的 catch_speed 換成可達的
        #    最佳爬升速度),那一項有效,見 wingman_node。
        self.declare_parameter("master_alt_gate", 0.0)     # False = 回到舊的二值切換
        self.declare_parameter("status_stale", 5.0)       # 超過此秒數沒回報就忽略該僚機
        self.declare_parameter("expected_followers", 5)    # 收齊這麼多僚機回報才可能判定成形（含備長）
        self.declare_parameter("guided_min_alt", 15.0)

        g = self.get_parameter
        vid = int(g("uav_id").value)
        use_mavros = bool(g("use_mavros").value)
        self.master = MasterPublisher(
            self, vid, float(g("origin_lat").value),
            float(g("origin_lon").value), float(g("altitude").value),
            float(g("speed").value), float(g("heading_deg").value),
            use_mavros=use_mavros)
        self.master.activate()
        self._own_id = vid
        # ── 自我降權(防腦裂)──:聽 whoLeader;有「別機」的新鮮 whoLeader
        # (代理長機,如冷啟動時 backup 先接管)→ 停發 whoLeader + 停巡邏,
        # 心跳照發(帶健康)讓代理長機把權讓回來;對方消失 ≥2 s 才恢復。
        self._foreign_leader_t = -1e9    # 最後一次收到別機 whoLeader 的時刻
        self._suspended_log = False
        self.create_subscription(String, "/swarm/whoLeader",
                                 self._on_who, DEEP_SUB_QOS)
        # Step down if another node (the backup) takes over — prevents the
        # low-health takeover path from leaving two nodes publishing whoLeader.
        self.create_subscription(String, "/swarm/master_switch",
                                 self._on_master_switch, SWITCH_QOS)
        src = "MAVROS global_position" if use_mavros else "sim"
        self.get_logger().info(
            f"master_node up as UAV{vid}: GeoPose @10 Hz ({src}), heartbeat @2 Hz")

        # real mode: GUIDED takeoff, then fly a patrol box on guided WPs so the
        # whole V actually goes somewhere; wingmen track this leader's GeoPose.
        self.fsm = None
        if use_mavros and bool(g("auto_guided").value):
            from sar_swarm.flight import FlightFSM
            from sar_swarm.mavros_interface import MavrosInterface
            self.mav = MavrosInterface(self, vid)
            self.fsm = FlightFSM(self, self.mav, float(g("takeoff_alt").value),
                                 auto_arm=bool(g("auto_arm").value),
                                 auto_takeoff=bool(g("auto_takeoff").value),
                                 catapult=(str(g("launch_type").value) == "catapult"))
            self._airborne = False
            self._patrol_alt = float(g("patrol_alt").value)
            self._patrol_cap = float(g("patrol_capture").value)
            self._auto_patrol = bool(g("auto_patrol").value)
            olat, olon = float(g("origin_lat").value), float(g("origin_lon").value)
            EARTH = 111320.0
            # patrol_box_m = 方框邊長 m。越大＝直線段越長、轉角越少 → 高速下僚機
            # 有足夠直線把 slot 誤差收斂（小方框的頻繁 90° 轉角會一直把僚機甩開）。
            B = float(g("patrol_box_m").value)
            box = [(B, 0.0), (B, B), (0.0, B), (0.0, 0.0)]
            self._patrol = [(olat + n / EARTH,
                             olon + e / (EARTH * math.cos(math.radians(olat))))
                            for n, e in box]
            self._leg = 0
            self._leg_sent = False
            self._rtl_announced = False
            self._patrol_hold_timeout = float(g("patrol_hold_timeout").value)
            self._corner_since = 0.0     # 到轉角的時刻；0=還沒到角（轉彎閘門用）
            # §4.5 轉彎程序狀態
            self._turn_radius = float(g("turn_radius").value)
            self._preslow_margin = float(g("preslow_margin").value)
            self._arc = None             # None=直線段;dict=弧轉進行中(§4.5)
            self._formed_now = False     # 最近一次速度控制算出的成形狀態（給轉彎閘門看）
            self.rtl_pub = self.create_publisher(Bool, "/swarm/rtl", 1)
            # 編隊速度控制狀態
            self._nominal_speed = float(g("nominal_speed").value)
            self._wait_speed = float(g("wait_speed").value)
            self._catch_speed = float(g("catch_speed").value)
            self._close_time = max(1.0, float(g("close_time").value))
            self._max_consensus = bool(g("max_consensus").value)
            self._cruise_alt_thr = float(g("master_alt_gate").value)
            self._climb_gate_logged = False
            self._formed_radius = float(g("formed_radius").value)
            self._formed_hyst = float(g("formed_hysteresis").value)
            self._status_stale = float(g("status_stale").value)
            self._expected_followers = int(g("expected_followers").value)
            self._guided_min_alt = float(g("guided_min_alt").value)
            self._foll_err = {}          # id -> (err_m, t)；僚機回報的編隊誤差
            self._was_formed = False     # 成形邊緣偵測：未成形→成形時記錄一次
            self._last_lead_speed = 0.0
            self._last_lead_speed_t = 0.0
            self._last_guided_t = 0.0
            self.create_subscription(String, "/swarm/formation_status",
                                     self._on_status, HEARTBEAT_QOS)
            # 打擊期間讓權給 tot_node(見 strike_authority 的事故說明)
            self._strike_auth = StrikeAuthority(self)
            self.create_timer(0.5, self._tick)
            self.get_logger().info(f"UAV{vid} master: GUIDED takeoff, then patrol the box")

    @staticmethod
    def _bearing_deg(a, b):
        """a,b=(lat,lon) → 羅盤方位（從正北順時針，度）。"""
        dN = (b[0] - a[0])
        dE = (b[1] - a[1]) * math.cos(math.radians(a[0]))
        return math.degrees(math.atan2(dE, dN)) % 360.0

    def _on_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._foll_err[int(d["id"])] = (float(d["err"]),
                                            self.get_clock().now().nanoseconds * 1e-9)
        except Exception:
            pass

    def _formed(self, now):
        """收齊所有僚機的新鮮回報、且每架都在 formed_radius 內 → 隊形成立。
        還沒收齊（剛起飛、沒人回報）→ 視為未成形，長機維持 wait_speed 等大家跟上，
        不會一起飛還沒成形就用巡航速度把僚機甩在後面。

        遲滯：一旦成形，放寬解除條件（容忍 best-effort 偶發漏報一架、誤差放寬
        formed_hysteresis 倍），避免單筆掉包就在 wait/cruise 間抖動，長機速度才不會忽快忽慢。"""
        fresh = [err for (err, t) in self._foll_err.values()
                 if err >= 0 and (now - t) <= self._status_stale]
        worst = max(fresh) if fresh else None
        if self._was_formed:
            ok = (worst is not None and worst <= self._formed_radius * self._formed_hyst
                  and len(fresh) >= self._expected_followers - 1)
            return ok, worst
        if len(fresh) < self._expected_followers:
            return False, worst          # 回報不齊 → 未成形（長機等）
        return worst <= self._formed_radius, worst

    def _leader_speed_control(self, now):
        # ⭐ 高度閘門:還沒到巡航高度就不做協同讓速(見 cruise_alt_threshold)。
        # ⚠️⚠️ 這道閘門**預設關閉(master_alt_gate=0.0)且應該維持關閉** ——
        #    受控實測:爬升段關掉 max-consensus 讓散布從 62.7 s 惡化到 188.2 s。
        #    因果是相反的:缺口正是在爬升段形成的,手風琴必須在那時就壓制。
        # ⚠️ 若哪天要重新啟用,**不要用下面這個裸比較** —— 它沒有遲滯,
        #    僚機端同樣的寫法在 P91 讓 UAV5 在 200 s 內翻了 10 次相位、
        #    末端誤差發散到 33 km。改用 sar_swarm.phase_gate.LatchedAltGate。
        p_now = self.mav.position() if hasattr(self, "mav") else None
        if (self._cruise_alt_thr > 0.0 and p_now is not None
                and p_now[2] < self._cruise_alt_thr):
            if not self._climb_gate_logged:
                self._climb_gate_logged = True
                self.get_logger().info(
                    f"CLIMBING(alt {p_now[2]:.0f} < 門檻 {self._cruise_alt_thr:.0f} m)"
                    f" —— max-consensus 暫停,全力爬升")
            self._formed_now = False
            self._keep_guided_only(now)
            return
        if self._climb_gate_logged:
            self._climb_gate_logged = False
            self.get_logger().info(
                f"CRUISE_FORMATION(alt {p_now[2]:.0f} m)—— 解鎖 max-consensus 速度對齊")
        formed, worst = self._formed(now)
        self._formed_now = formed            # 給轉彎閘門看
        if formed:
            spd = self._nominal_speed
        elif not self._max_consensus or worst is None:
            spd = self._wait_speed           # 舊行為:二值切換 / 還沒收到任何回報
        else:
            # ⭐ Max-Consensus:全隊向最慢者對齊 —— 長機讓出速度,讓出量由最差
            #    落後量連續決定。閉合率 = catch_speed − v_lead,要在 close_time
            #    秒內收掉 worst 公尺 ⇒ v_lead = catch_speed − worst/close_time。
            #    夾在 [wait_speed, nominal_speed]:落後小 → 幾乎不減速(不犧牲群速);
            #    落後大 → 一路掉到 wait_speed(用滿減速權限)。
            #    ⚠️ 這是連續的,不是檔位 —— 舊的二值切換會在 formed 邊界上抖動,
            #      而抖動本身又會製造新的落後(長機忽快忽慢,僚機追不上相位)。
            spd = self._catch_speed - worst / self._close_time
            spd = max(self._wait_speed, min(self._nominal_speed, spd))
        self.master.cmd_speed = spd          # 廣播給僚機：到位的就跟著長機這個速度（不會超車）
        if formed and not self._was_formed:
            self.get_logger().info(
                f"formation formed (worst {worst:.0f} m) — leader resuming nominal "
                f"{self._nominal_speed:.0f} m/s cruise")
        self._was_formed = formed
        if abs(spd - self._last_lead_speed) > 0.5 or (now - self._last_lead_speed_t) > 3.0:
            self.mav.change_speed_async(spd)
            self._last_lead_speed, self._last_lead_speed_t = spd, now
            if not formed:
                w = f"{worst:.0f} m" if worst is not None else "awaiting reports"
                mode = "max-consensus" if (self._max_consensus and worst is not None) \
                    else "binary wait"
                rate = (self._catch_speed - spd) if worst is not None else 0.0
                self.get_logger().info(
                    f"leader slowing to {spd:.0f} m/s [{mode}] — worst follower err {w}"
                    f"(閉合率 {rate:.0f} m/s ⇒ 預估 "
                    f"{(worst / rate) if (worst and rate > 0.5) else float('inf'):.0f} s)")
        # 起飛後若被切出 GUIDED（操作員手動 TAKEOFF/AUTO），重新宣告 GUIDED
        p = self.mav.position()
        if (p is not None and p[2] > self._guided_min_alt
                and self.mav.mode() not in ("GUIDED", "RTL")
                and (now - self._last_guided_t) > 2.0):
            self.mav.set_mode_async("GUIDED")
            self._last_guided_t = now
            self.get_logger().info("leader: re-asserting GUIDED for patrol")

    def _keep_guided_only(self, now):
        """爬升段:只維持 GUIDED,不下任何速度指令。

        ⚠️ 不能整段 return —— 起飛後若被切出 GUIDED,導引航點會失效、飛機空盤旋。
           保留這一段是為了「不干擾爬升」而不是「不控制」。
        """
        p = self.mav.position()
        if (p is not None and p[2] > self._guided_min_alt
                and self.mav.mode() not in ("GUIDED", "RTL")
                and (now - self._last_guided_t) > 2.0):
            self.mav.set_mode_async("GUIDED")
            self._last_guided_t = now
            self.get_logger().info("leader: re-asserting GUIDED (climb phase)")

    def _on_who(self, msg):
        """記錄「別機」whoLeader 的新鮮度(自己的忽略)。"""
        try:
            if int(json.loads(msg.data).get("id", -1)) != self._own_id:
                self._foreign_leader_t = self.get_clock().now().nanoseconds * 1e-9
        except (ValueError, TypeError):
            pass

    def _tick(self):
        if not self.fsm.update():        # still climbing out into GUIDED
            return
        # ── 讓權檢查 ①(最優先):tot_node 正在打擊 ⇒ 本節點停止下令 ──
        # ⚠️⚠️ 沒有這一段的後果(AP2 實測):tot_node 在 ALIGN 指揮飛機
        #    飛 48 km 向西、爬到 400 m,而本節點同時以 1 Hz 重發
        #    `master -> leg 2 (…) @ 80 m` + `leader slowing to 10 m/s`
        #    ⇒ 兩個控制器對同一架飛機下相反的令,編隊層靠重發頻率贏,
        #    飛機被壓在 50–80 m 在起飛點繞圈,距目標幾乎不減少。
        #    批次路徑是靠 swarm_ros2_down.sh 把本層整個收掉來交棒的
        #    (stot_trial.sh:425),互動路徑沒有那一步 —— 這就是那一步。
        # ⚠️ 只停下令,遙測 / 心跳 / whoLeader 照發(見 strike_authority)。
        if self._strike_auth.yielded():
            return
        # ── 讓權檢查 ②(每 tick)──:有別機在當長機就退讓,不打架
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._foreign_leader_t) < 2.0:
            self.master.turn_rate = 0.0
            if not self.master.suspend_who:
                self.master.suspend_who = True
                self.get_logger().warn(
                    "foreign leader on whoLeader — suspending own leadership "
                    "(heartbeat continues; waiting for yield-back)")
            return                       # 停巡邏/停調速,僚機跟代理長機
        if self.master.suspend_who:      # 對方消失 ≥2 s → 收回領導權
            self.master.suspend_who = False
            self.get_logger().info(
                "foreign leader gone — resuming own leadership + patrol")
        if not self._airborne:
            self._airborne = True
            self.get_logger().info("master airborne in GUIDED — leading the patrol")
        # Clear the corner heading-freeze every tick up front; the corner-hold block
        # below re-asserts it only while actually holding. This guarantees no stale
        # next-leg bearing leaks onto whoLeader via any early return (RTL / operator
        # mode), which would otherwise mis-orient the wingmen's whole V.
        self.master.heading_override = None
        # Operator RTL on the leader (from the GCS) triggers a coordinated formation
        # RTL: tell the wingmen to bring the V home (each at its layered altitude, so
        # the converging formation never collides). The leader rides its own FCU RTL.
        if self.mav.mode() == "RTL":
            self.master.turn_rate = 0.0
            self.rtl_pub.publish(Bool(data=True))     # re-announce each tick (idempotent)
            if not self._rtl_announced:
                self._rtl_announced = True
                self.get_logger().warn(
                    "leader in RTL — formation RTL: wingmen returning home in a layered V")
            return                                    # stop patrol; FCU RTL flies it home
        if self._rtl_announced:                       # leader left RTL → resume formation
            self._rtl_announced = False
            self.rtl_pub.publish(Bool(data=False))
            self.get_logger().info("leader out of RTL — resuming patrol")
        self._leader_speed_control(                    # 僚機沒到位就放慢等；保持 GUIDED
            self.get_clock().now().nanoseconds * 1e-9)
        if not self._auto_patrol:
            return   # operator mode: publish GeoPose for wingmen, leader flown by GCS guided
        # ── §4.5 轉彎程序:轉前減速 + 定大半徑 fillet 弧 ──────────────────
        # 取代舊「飛到尖角→定點等隊→急轉」:急轉時 slot 以機體 bank 角速率
        # (~20°/s)繞長機掃掠,外側僚機需求速度遠超 v_catch → 轉角散隊(點雲
        # 實測)。fillet 弧讓 slot 掃掠率 = v_corner/R(~1.1°/s),僚機跟得上;
        # heading 不凍結 —— 就是要僚機的 V 隨弧緩慢旋轉。
        fix = (self.master.lat, self.master.lon)   # MasterPublisher tracks the real fix
        now = self.get_clock().now().nanoseconds * 1e-9
        P = self._patrol[self._leg]                              # 這段的角點
        Nx = self._patrol[(self._leg + 1) % len(self._patrol)]   # 下一角點

        if self._arc is not None:
            a = self._arc                     # ── 弧轉進行中 ──
            # 進度 = 長機相對圓心的方位角(unwrap 累計),與目標掃角比較
            th = self._bearing_deg(a["c"], fix)
            d = (th - a["last_th"] + 540.0) % 360.0 - 180.0      # wrap to ±180
            if d * a["s"] > 0:                # 只累計轉向方向的前進量
                a["swept"] += abs(d)
            a["last_th"] = th
            remain = a["sweep"] - a["swept"]
            # carrot:弧上領先 look 度(≈ v·10s 的弧長,8°..20° 夾住)
            look = max(8.0, min(20.0, math.degrees(
                self._wait_speed * 10.0 / self._turn_radius)))
            th_c = math.radians(th + a["s"] * min(look, max(remain, 0.0) + 5.0))
            clat = a["c"][0] + (self._turn_radius * math.cos(th_c)) / 111320.0
            clon = a["c"][1] + (self._turn_radius * math.sin(th_c)) / (
                111320.0 * math.cos(math.radians(a["c"][0])))
            self.mav.send_guided_waypoint(clat, clon, self._patrol_alt)
            self.master.cmd_speed = self._wait_speed             # 轉前已減速,弧上維持
            if abs(self._wait_speed - self._last_lead_speed) > 0.5 \
                    or (now - self._last_lead_speed_t) > 3.0:
                self.mav.change_speed_async(self._wait_speed)
                self._last_lead_speed, self._last_lead_speed_t = self._wait_speed, now
            if remain <= look * 0.6:          # 弧完成 → 切下一段直線
                self._leg = (self._leg + 1) % len(self._patrol)
                self._leg_sent = False
                self._arc = None
                self.master.turn_rate = 0.0   # 直線:旋轉前饋歸零
                self.get_logger().info(
                    f"§4.5 arc done (swept {a['swept']:.0f}°) — leg {self._leg}")
            return

        # ── 直線段:追這段的入彎點(P 前 L=R·tan(Δψ/2))────────────────
        psi_in = self._bearing_deg(fix, P)
        psi_out = self._bearing_deg(P, Nx)
        dpsi = (psi_out - psi_in + 540.0) % 360.0 - 180.0        # 轉角量(±)
        L = self._turn_radius * math.tan(math.radians(min(abs(dpsi), 120.0) / 2.0))
        dist_P = dist_m(fix, P)
        self.mav.send_guided_waypoint(P[0], P[1], self._patrol_alt)
        if not self._leg_sent:
            self._leg_sent = True
            self.get_logger().info(
                f"master -> leg {self._leg} ({P[0]:.5f},{P[1]:.5f}) @ "
                f"{self._patrol_alt:.0f} m (turn-entry L={L:.0f} m)")
        # 轉前減速:入彎點前 preslow_margin 內壓到 wait_speed(§4.5)
        if dist_P < L + self._preslow_margin:
            self.master.cmd_speed = self._wait_speed
            if abs(self._wait_speed - self._last_lead_speed) > 0.5 \
                    or (now - self._last_lead_speed_t) > 3.0:
                self.mav.change_speed_async(self._wait_speed)
                self._last_lead_speed, self._last_lead_speed_t = self._wait_speed, now
        # 抵達入彎點 → 建弧:圓心在入彎點側向(轉向側)R 處
        if dist_P <= max(L, self._patrol_cap) and abs(dpsi) > 20.0:
            s = 1.0 if dpsi > 0 else -1.0                        # +1=右轉(順時針)
            perp = psi_in + s * 90.0                             # 指向圓心的方位
            c_lat = fix[0] + (self._turn_radius *
                              math.cos(math.radians(perp))) / 111320.0
            c_lon = fix[1] + (self._turn_radius *
                              math.sin(math.radians(perp))) / (
                111320.0 * math.cos(math.radians(fix[0])))
            self._arc = {"c": (c_lat, c_lon), "s": s, "sweep": abs(dpsi),
                         "swept": 0.0,
                         "last_th": self._bearing_deg((c_lat, c_lon), fix)}
            # 廣播框轉率 → 僚機做旋轉前饋(線性前饋在持續旋轉下永遠指向弧外)
            self.master.turn_rate = s * math.degrees(
                self._wait_speed / self._turn_radius)
            self.get_logger().info(
                f"§4.5 arc turn start: Δψ={dpsi:.0f}° R={self._turn_radius:.0f} m "
                f"(slot 掃掠率 ~{math.degrees(self._wait_speed/self._turn_radius):.1f}°/s)")
        elif dist_P <= max(L, self._patrol_cap):
            # 轉角量太小(<20°)不值得弧轉:直接切段
            self._leg = (self._leg + 1) % len(self._patrol)
            self._leg_sent = False


    def _on_master_switch(self, msg):
        """被接管 → 降權但**心跳照發**(suspend_who);收到指回自己的 switch
        (backup 讓權)→ 收回領導。舊版在此整個 deactivate() 連心跳一起停,
        backup 從此看不到原長機復活 → 讓權死鎖 + 腦裂(實測根因)。"""
        try:
            nid = int(json.loads(msg.data)["new_master_id"])
        except Exception:
            return
        if nid != self._own_id:
            if not self.master.suspend_who:
                self.master.suspend_who = True
                self.get_logger().warn(
                    f"master UAV{self._own_id}: superseded by UAV{nid} — suspending "
                    f"whoLeader/GeoPose (heartbeat continues for yield-back)")
        else:                            # backup 把領導權讓回來了
            self.master.suspend_who = False
            if not self.master.active:
                self.master.activate()
            self.get_logger().info(
                f"master UAV{self._own_id}: leadership returned — resuming")


def main():
    rclpy.init()
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
