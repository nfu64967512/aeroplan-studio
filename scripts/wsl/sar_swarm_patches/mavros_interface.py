"""MAVROS interface for one aircraft: state, setpoints, and the launch services.

Composition helper (not a Node) — it attaches its subs, pub, and service clients to
a host node under /uav{id}/mavros/*. It imports mavros_msgs, so import it only on
the real-mode path; sim mode never touches it.

Provides:
  - state:    connected(), armed(), mode(), position(), heading()
  - launch:   set_mode_async(), arm_async(), takeoff_async()
  - speed:    change_speed_async(airspeed)   (MAV_CMD_DO_CHANGE_SPEED)
  - setpoint: send_position_setpoint(e, n, u, yaw)   -> setpoint_raw/local
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from rclpy.qos import QoSPresetProfiles

from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from nav_msgs.msg import Odometry

from sar_swarm.formation_control import yaw_from_quaternion

MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_SET_SERVO = 183


class MavrosInterface:
    def __init__(self, node, vehicle_id: int):
        self.node = node
        ns = f"/uav{vehicle_id}/mavros"

        self._state: Optional[State] = None
        self._odom: Optional[Odometry] = None

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        node.create_subscription(State, f"{ns}/state", self._on_state, 10)
        node.create_subscription(Odometry, f"{ns}/global_position/local",
                                 self._on_odom, sensor_qos)

        self.sp_pub = node.create_publisher(
            PositionTarget, f"{ns}/setpoint_raw/local", 10)
        self.gp_pub = node.create_publisher(
            GlobalPositionTarget, f"{ns}/setpoint_raw/global", 10)

        self.cli_mode = node.create_client(SetMode, f"{ns}/set_mode")
        self.cli_arm = node.create_client(CommandBool, f"{ns}/cmd/arming")
        self.cli_takeoff = node.create_client(CommandTOL, f"{ns}/cmd/takeoff")
        self.cli_cmd = node.create_client(CommandLong, f"{ns}/cmd/command")

        # ── Direct pymavlink link for GUIDED waypoints ─────────────────────
        # ArduPlane fixed-wing GUIDED ignores SET_POSITION_TARGET (the message
        # behind MAVROS setpoint_raw/global), so position commands go out as
        # legacy "guided waypoints" (MISSION_ITEM_INT, current=2) over a direct
        # MAVLink link to this aircraft's mavproxy fan-out (NODE_BASE+10*(id-1)).
        # MAVROS still provides state / arming / mode / takeoff (above).
        self._wp_sysid = int(vehicle_id)
        self._wp = None
        # 速度指令權威鏈的計數器與開關(見 change_speed_async)
        self._speed_ok_n = 0
        self._speed_fail_n = 0
        import os as _os
        self._speed_direct = _os.environ.get("SPEED_DIRECT", "1") != "0"
        # DO_REPOSITION 的 param3(盤旋半徑,m)。**必須 > 0**,見 send_guided_waypoint。
        # 取小值:L1 仍會乘 eas2tas²(3100 m 約 1.365),30 → ~41 m,
        # 對 4000 m 級的 slot 可忽略,等效於「飛到該點」。
        self._reposition_radius = float(_os.environ.get("REPOSITION_RADIUS", "30"))
        try:
            import os
            from pymavlink import mavutil
            host = os.environ.get("SAR_NODE_CMD_HOST", "127.0.0.1")
            base = int(os.environ.get("SAR_NODE_CMD_BASE", "6960"))
            port = base + 10 * (self._wp_sysid - 1)
            self._wp = mavutil.mavlink_connection(
                f"tcp:{host}:{port}", source_system=250, dialect="ardupilotmega")
            self._wp.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            node.get_logger().info(
                f"uav{self._wp_sysid} guided-WP link up on {host}:{port}")
            # ⚠️⚠️ **這條鏈路一定要排空,不能只寫不讀。**
            #    MAVProxy 是單執行緒:`--out=tcpin` 的某個客戶端不讀,
            #    核心收方緩衝滿 → TCP 視窗關閉 → MAVProxy 卡在那個 write 上
            #    ⇒ **該機所有輸出一起凍結**(MAVROS / tot_node / AeroPlan 全部)。
            #    實測 RPG1(2026-08-10)UAV1:SITL→MAVProxy 進來 2.4 MB,
            #    但每個下游只拿到 9–16 kB(UAV2 同期是 2.2 MB,差 200 倍)
            #    ⇒ tot_node 眼中 UAV1 的位姿**永遠停在 t=0**,`d` 逐位元固定
            #      在 34148,整趟不動,最後逾時。
            #    症狀極具欺騙性:飛機真的在飛、MAVROS 也回得出 mode=GUIDED
            #    (那是**鎖存的舊值**),只有位元組計數器看得出來。
            #    這與 master-node-stall-diagnosis 記載的是同一個根因 ——
            #    當時定位了,但從沒在本檔修掉。
            # ⚠️ 排空器掛在**自己的 timer** 上,不是掛在節點的 tick:
            #    tick 在讓權(strike_authority)時會整段 return,
            #    排空**絕對不可以**跟著停 —— 那正好是打擊期間,最不能凍結的時候。
            self._wp_drain_timer = node.create_timer(0.2, self._drain_wp)
        except Exception as e:
            node.get_logger().warn(
                f"uav{vehicle_id} guided-WP link failed ({e}); formation flight disabled")
            self._wp = None

    def _drain_wp(self):
        """把直連鏈路的收方向讀掉丟棄。內容不需要(位姿走 MAVROS),
        重點是**不要讓 MAVProxy 塞住**。單次上限避免佔住 executor。"""
        if self._wp is None:
            return
        try:
            for _ in range(400):
                if self._wp.recv_match(blocking=False) is None:
                    break
        except Exception:
            pass

    # ---- subscriptions -----------------------------------------------------
    def _on_state(self, msg: State):
        self._state = msg

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    # ---- state accessors ---------------------------------------------------
    def connected(self) -> bool:
        return self._state is not None and self._state.connected

    def armed(self) -> bool:
        return self._state is not None and self._state.armed

    def mode(self) -> str:
        return self._state.mode if self._state else ""

    def in_takeoff_mode(self) -> bool:
        """ArduPlane TAKEOFF flight mode. MAVROS has no name alias for it, so
        State.mode reads back as the raw id string 'CMODE(13)' (or 'TAKEOFF' on
        builds that do map it)."""
        return self.mode() in ("TAKEOFF", "CMODE(13)")

    def have_pose(self) -> bool:
        return self._odom is not None

    def position(self) -> Optional[Tuple[float, float, float]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        return (p.x, p.y, p.z)

    def velocity(self) -> Optional[Tuple[float, float, float]]:
        if self._odom is None:
            return None
        v = self._odom.twist.twist.linear
        return (v.x, v.y, v.z)

    def heading(self) -> float:
        """ENU heading from East. Prefer the velocity vector; fall back to yaw."""
        v = self.velocity()
        if v is not None and math.hypot(v[0], v[1]) > 2.0:
            return math.atan2(v[1], v[0])
        if self._odom is not None:
            q = self._odom.pose.pose.orientation
            return yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return 0.0

    def orientation(self):
        return self._odom.pose.pose.orientation if self._odom else None

    # ---- launch services (async, non-blocking) -----------------------------
    def set_mode_async(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode            # ArduPlane: "GUIDED", "AUTO", "TAKEOFF"
        return self.cli_mode.call_async(req)

    def arm_async(self, value: bool):
        req = CommandBool.Request()
        req.value = value
        return self.cli_arm.call_async(req)

    def takeoff_async(self, altitude: float):
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        return self.cli_takeoff.call_async(req)

    def fire_catapult_async(self, channel: int = 7, pwm: int = 2000):
        """Release the catapult/bungee: drive a servo output high (MAV_CMD_DO_SET_SERVO).

        The SITL plane-catapult model applies the launch impulse while servo output
        `channel` reads > 1700 (see SIM_Plane.cpp: input.servos[6] for ch7). On real
        hardware wire this channel to the catapult release relay. Re-issuing is safe:
        the sim latches the impulse once and only re-applies after the servo drops.
        """
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_SET_SERVO
        req.param1 = float(channel)       # servo output channel (1-based)
        req.param2 = float(pwm)           # PWM; > 1700 triggers the sim launcher
        return self.cli_cmd.call_async(req)

    def change_speed_async(self, airspeed: float):
        """MAV_CMD_DO_CHANGE_SPEED,**帶回應檢驗 + 直連鏈路權威路徑**。

        參數規範(嚴格,不可改):
          param1 = 0.0        → 0 = Airspeed(1 = Groundspeed、3 = Throttle)
          param2 = target_eas → 目標等效空速 m/s
          param3 = -1.0       → throttle 不變
          param4 = 0.0

        ⚠️⚠️ 為什麼要走**兩條路**(T1 實測):
          指令記錄器記到 `cmd_speed_eas=157`,飛機卻整段停在 ~72 EAS ——
          也就是爬升腳本用**直連鏈路**送的那一發還在生效,而 wingman 每 3 s
          經 MAVROS 送的 157 沒有蓋過去。導引航點本來就走直連
          (ArduPlane 的 GUIDED 不吃 SET_POSITION_TARGET),`param_set_async`
          也早就因為「MAVROS param plugin 起飛階段常常還沒 ready」改走直連。
          速度是縱向**唯一**有權限的通道,不能只靠一條沒有驗證過的路。
        ⇒ 直連鏈路當權威(和爬升腳本同一條,後發者勝)。
          `SPEED_DIRECT=0` 可關掉直連,改走 MAVROS 做 A/B 對照。

        ⚠️⚠️⚠️ **一次只准走一條路。** 2026-08-07 之前這裡是「兩條都送」,
           那會直接**殺死 MAVROS**:

             [uav1.mavros.cmd]: CMD: Unexpected command 178, result 4
             terminate called after throwing an instance of 'std::future_error'
               what():  std::future_error: Promise already satisfied
             process has died [exit code -6]

           機制:兩條路把**同一個** cmd 178 送給**同一台飛控**,飛控對每一發
           各回一個 COMMAND_ACK,而 MAVProxy 把 ACK 扇出到所有輸出 ⇒ MAVROS
           的 command plugin 收到兩個 ACK,對同一個 promise `set_value` 兩次
           ⇒ C++ 端丟 `std::future_error`,而那是**沒有人接的例外** ⇒
           `terminate()` ⇒ SIGABRT。

           後果**不只是少一個節點**:master_role 會抱著最後一筆定位以 10 Hz
           繼續廣播(speed/omega 還在變,看起來完全正常),僚機忠實地飛向一個
           永遠不動的 slot —— ANM1 那趟飛到 68 km 外,全程 WARN/ERROR 計數 = 0。
           見 [[leader-state-freeze]] 與 tests/test_leader_state_freeze.py。

           ⚠️ 這條不是「順手清理」:直連那條**本來就是權威**(上面 T1 的實測),
              MAVROS 那一發從頭到尾只是觀測探針。用一個探針換掉整個 MAVROS,
              是這個專案裡代價最差的一筆交易。觀測性改由 `_speed_report`
              直接記錄直連的送出結果,不需要再送一發真的指令。
        """
        eas = float(airspeed)
        if self._speed_direct:
            # 權威路徑:只送這一發。**不可以**再補一發 MAVROS(見上)。
            self._speed_report(eas, self._change_speed_direct(eas))
            return None

        # SPEED_DIRECT=0:A/B 對照組,只走 MAVROS ⇒ 同樣只有一發,不會雙 ACK。
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_CHANGE_SPEED
        req.param1 = 0.0                  # 0 = airspeed(不是地速、不是油門)
        req.param2 = eas                  # 目標等效空速
        req.param3 = -1.0                 # throttle: no change
        req.param4 = 0.0
        fut = self.cli_cmd.call_async(req)
        fut.add_done_callback(
            lambda f: self._on_speed_response(f, eas, None))
        return fut

    def _speed_report(self, eas: float, direct_ok):
        """直連路徑的可觀測量。

        ⚠️ 直連是 `command_long_send`,**沒有回應可以等** —— 這裡能講的只有
           「送出去了沒有」。飛控到底收不收(例如 catch_speed > AIRSPEED_MAX
           ⇒ MAV_RESULT=4,見 `_speed_warn` 的三種失敗)要靠 tot_node 的
           COMMAND_ACK 側錄,不要在這裡假裝知道。
        """
        if direct_ok:
            self._speed_ok_n += 1
            return
        self._speed_warn("直連鏈路送不出 DO_CHANGE_SPEED(WP link 未建立?)",
                         eas, direct_ok)

    def _change_speed_direct(self, eas: float):
        """走導引航點那條直連鏈路送 DO_CHANGE_SPEED(與爬升腳本同源)。

        ⚠️ 1 發/次、最快 3 s 一次,相對 1 Hz 的導引航點可忽略,不會洗頻寬。
        """
        if self._wp is None:
            return False
        try:
            from pymavlink import mavutil
            self._wp.mav.command_long_send(
                self._wp.target_system or self._wp_sysid, 1,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                0, eas, -1, 0, 0, 0, 0)
            return True
        except Exception:
            return False

    def _on_speed_response(self, fut, eas: float, direct_ok):
        """檢驗 MAVROS 的 CommandLong 回應;失敗就 WARN(節流,不洗日誌)。"""
        try:
            res = fut.result()
        except Exception as e:
            self._speed_warn(f"MAVROS 速度服務例外 {e!r}", eas, direct_ok)
            return
        # MAVROS: success(bool) + result(uint8 = MAV_RESULT;0 = ACCEPTED)
        ok = bool(getattr(res, "success", False))
        mav_result = int(getattr(res, "result", 255))
        if ok and mav_result in (0, 255):
            self._speed_ok_n += 1
            return
        self._speed_warn(
            f"MAVROS 拒收 DO_CHANGE_SPEED(success={ok} MAV_RESULT={mav_result})",
            eas, direct_ok)

    def _speed_warn(self, why: str, eas: float, direct_ok):
        """⚠️⚠️ `DO_CHANGE_SPEED` 有**三種**互相獨立的失敗原因,不要混為一談:

          (1) 目標速度不在 [AIRSPEED_MIN, AIRSPEED_MAX] ⇒ MAV_RESULT=4
              (ArduPlane commands_logic.cpp::do_change_speed)
              實測 V1:catch_speed 157 > AIRSPEED_MAX 150 ⇒ 追趕指令全滅。
          (2) **飛機不在 GUIDED / AUTO** ⇒ MAV_RESULT=4,連 do_change_speed
              都不會被呼叫(GCS_MAVLink_Plane.cpp::handle_command_DO_CHANGE_SPEED
              開頭就擋掉)。實測 W1:目標 96 EAS 明明在區間內仍被拒。
          (3) MAVROS 沒收到 COMMAND_ACK ⇒ success=False 但 MAV_RESULT=**0**
              (欄位沒被填)。那是**傳輸層**的問題,不是飛控拒絕。

        ⇒ 一起印出模式與速度,讓日誌自己說明是哪一種,不要再靠猜。
        """
        self._speed_fail_n += 1
        n = self._speed_fail_n
        # 第 1、2、5、20、40… 次才印,避免 3 s 一次洗爆日誌
        if n in (1, 2, 5) or n % 20 == 0:
            path = ("直連鏈路已代送 ✓" if direct_ok
                    else ("直連鏈路也失敗 ✗" if direct_ok is False
                          else "直連鏈路已關閉(SPEED_DIRECT=0)"))
            mode = getattr(self._state, "mode", "?") if self._state else "?"
            self.node.get_logger().warning(
                f"速度指令未生效({n} 次):{why};目標 {eas:.0f} EAS;"
                f"目前模式 {mode}。{path}")

    def param_set_async(self, name: str, value: float):
        """走直連 pymavlink 鏈路設飛控參數(不經 MAVROS 的 param 服務)。

        ⚠️ 走 self._wp 而不是 MAVROS:MAVROS 的 param plugin 在 Humble 上要
        ParamSetV2 且需先完成整份 param list 的同步,起飛階段常常還沒 ready。
        直連鏈路是本專案已經在用的路徑(tot_node 也是這樣放開俯仰限制)。
        ⚠️ fire-and-forget,沒有 ACK。呼叫端要自己節流,不要每個 tick 都送。
        """
        if self._wp is None:
            return False
        try:
            from pymavlink import mavutil
            self._wp.mav.param_set_send(
                self._wp.target_system or self._wp_sysid, 1,
                name.encode() if isinstance(name, str) else name,
                float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            return True
        except Exception:
            return False

    def services_ready(self) -> bool:
        return all(c.service_is_ready() for c in
                   (self.cli_mode, self.cli_arm, self.cli_takeoff, self.cli_cmd))

    # ---- setpoint ----------------------------------------------------------
    def send_position_setpoint(self, e: float, n: float, u: float,
                               yaw: float, vel_ff: Optional[Tuple[float, float, float]] = None):
        """Position setpoint to setpoint_raw/local (ENU; MAVROS converts to NED).

        Position + yaw are commanded; acceleration and yaw-rate are ignored. The
        run-in speed is set separately by change_speed_async (ArduPlane fixed-wing
        tracks airspeed, not a velocity setpoint). Pass vel_ff only if you want a
        velocity feedforward as well.
        """
        pt = PositionTarget()
        pt.header.stamp = self.node.get_clock().now().to_msg()
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        mask = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
        if vel_ff is None:
            mask |= (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY
                     | PositionTarget.IGNORE_VZ)
        else:
            pt.velocity.x, pt.velocity.y, pt.velocity.z = vel_ff
        pt.type_mask = mask

        pt.position.x = e
        pt.position.y = n
        pt.position.z = u
        pt.yaw = yaw
        self.sp_pub.publish(pt)

    def send_global_setpoint(self, lat: float, lon: float, alt_rel: float,
                             yaw: float = float("nan")):
        """Global position setpoint (lat/lon, alt relative to home) to
        setpoint_raw/global. Used for formation: every aircraft is commanded in the
        same WGS84 frame, so per-vehicle local origins don't matter."""
        gp = GlobalPositionTarget()
        gp.header.stamp = self.node.get_clock().now().to_msg()
        gp.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        gp.type_mask = (GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY
                        | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX
                        | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ
                        | GlobalPositionTarget.IGNORE_YAW_RATE)
        gp.latitude = lat
        gp.longitude = lon
        gp.altitude = alt_rel
        if not math.isnan(yaw):
            gp.yaw = yaw
        self.gp_pub.publish(gp)

    # ---- guided waypoint (the path that actually flies on ArduPlane FW) -----
    def send_guided_waypoint(self, lat: float, lon: float, alt_rel: float) -> bool:
        """Fly to lat/lon at alt_rel (m, relative to home) as a GUIDED waypoint.

        Sent as MISSION_ITEM_INT with current=2 (legacy "fly to here") over the
        direct pymavlink link, because ArduPlane fixed-wing GUIDED does not act on
        SET_POSITION_TARGET offboard setpoints (send_global_setpoint above is kept
        for VTOL / other firmware). The FCU registers a new guided WP with a few
        seconds of latency and re-issuing resets it, so callers must NOT spam this
        — re-target only when the destination has actually moved. Returns False if
        the link is unavailable.
        """
        if self._wp is None:
            return False
        try:
            from pymavlink import mavutil
            # MAV_CMD_DO_REPOSITION (192) via COMMAND_INT — the modern guided "fly to
            # here" that ArduPlane fixed-wing GUIDED reliably honours INCLUDING altitude.
            # The old MISSION_ITEM_INT current=2 path did NOT hold the commanded altitude
            # for the leader (it drifted up to a firmware default ~1000m); DO_REPOSITION
            # fixes that and is safe to re-issue continuously (no reset-latency quirk).
            # param1=-1 keep current speed, param2=**模式旗標**, param3=**盤旋半徑**,
            # param4=NaN yaw。
            #
            # ⭐⭐ **param2 = 0 是刻意的:編隊層不可以強制改模式。**
            #    ArduPlane 的閘門(GCS_MAVLink_Plane.cpp:584-605):
            #        if ((param2 & CHANGE_MODE) || control_mode == mode_guided) { …照做… }
            #        return MAV_RESULT_FAILED;
            #    ⇒ param2=0 而飛機不在 GUIDED ⇒ 整發丟掉。
            #    **這正是我們要的**:飛機在 TAKEOFF 模式滾行爬升時,本層的巡邏航點
            #    就該被丟掉,模式歸飛控所有,等爬過 guided_min_alt 再由
            #    `_keep_guided()` 用 set_mode 正式接手。
            #
            # ⚠️⚠️ 2026-08-10 我把這裡改成帶 CHANGE_MODE,**整批 RPG/RPH/RPI 報廢**。
            #    nodes.log 的時序是決定性證據:
            #        463.776  UAV1 master: GUIDED takeoff, then patrol the box
            #        465.777  master airborne in GUIDED — leading the patrol  ← 還在地上、還沒 arm
            #        465.777  master -> leg 0 (…) @ 80 m
            #    master_node 啟動 2 秒就自稱 airborne 並以 1 Hz 送巡邏航點。
            #    帶了 CHANGE_MODE 之後,這些航點**每秒把長機從 TAKEOFF 拽回 GUIDED**
            #    ⇒ 起飛被取消 ⇒ UAV1 gs_max=0.0 整趟停在跑道上。
            #    (UAV2 逃過,只因備長的航點有位移門檻、重送得少。)
            #
            # ⇒ 教訓:**CHANGE_MODE 是「打擊權威」層的東西,不是通用原語。**
            #    tot_node._reposition() 該帶(它要負責建立 GUIDED);本層不該帶。
            #    RPF 那個「讓權後沒人切模式」的問題,正解在 backup_node:
            #    讓 `_keep_guided()` 在讓權期間**繼續跑**,而不是把改模式塞進航點。
            #
            # ⚠️⚠️ **param3 絕對不可以是 0。** 這是本專案最貴的一個 bug,
            #    而且它在兩個節點各發作過一次(tot_node 的 ALIGN、這裡的編隊層)。
            #    ArduPlane 的閘門是 GCS_MAVLink_Plane.cpp:599
            #        if (!isnan(packet.param3) && packet.param3 > 0)
            #    ⇒ **0 不通過**,active_radius_m 維持 0
            #    ⇒ mode_guided.cpp `navigate()` → `update_loiter(0)`
            #    ⇒ navigation.cpp:362-366  radius<=1 ⇒ 退回 |WP_LOITER_RAD|(=1488)
            #    ⇒ AP_L1_Control.cpp:166   ×eas2tas²(3100 m 是 1.365)= **2030 m**
            #    ⇒ 飛機忠實地「**繞著目標點盤旋 2030 m**」,而不是飛過去。
            #    而 commands.cpp:97 `set_guided_WP()` 每次都把 auto_state.crosstrack
            #    設成 false,所以 navigation.cpp:343 那個「距離夠遠就切回航點導引」的
            #    逃生門在 GUIDED 下**永遠到不了** —— 沒有任何錯誤訊息。
            #
            # ⚠️ 實測 SG1 的決定性證據(dataflash,末端 60 s,n=1500/機):
            #      `NTUN.Dist − NTUN.XT` 是**常數** 2034.1/2039.5/2051.0/2030.2 m,
            #      與 `WP_LOITER_RAD × CTUN.E2T²` 逐機吻合到 **<1 m**;
            #      `|NavBrg − TBrg|` 在 ~9000 筆樣本裡 max = 0.01°
            #      (loiter 分支下這兩個是同一個量,那是它的指紋);
            #      `NTUN.XTi` 恆為 0(loiter 分支不跑 XTRACK 積分)。
            #    幾何後果:縱向被 _carrot_on_path 釘在 L1+lead ≈ 1823 m、
            #    距離被盤旋半徑釘在 ~2300 m ⇒ 橫向只能是
            #      √(2302² − 1823²) = 1402 m(實測 1389)。
            #    **橫向誤差因此有硬地板,不可能收斂** —— 就算完美貼到圈上仍有 ~900 m。
            #
            # ⚠️ 不要改 WP_LOITER_RAD 全域值:階段 3 的 ALIGN 也讀它,全域改會外溢。
            #    param3 是**逐指令**的,只作用在這一發 GUIDED reposition。
            # ⚠️ 也不要動 l1_lookahead_m:它不是成因,而且調**小**會讓 cross 變**大**
            #    (along→0 時 cross→Dist≈2300)。綁住橫向的是盤旋半徑,不是 L1。
            self._wp.mav.command_int_send(
                self._wp_sysid, 1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                -1.0, 0.0,
                float(self._reposition_radius), float("nan"),
                int(lat * 1e7), int(lon * 1e7), float(alt_rel))
            return True
        except Exception as e:
            self.node.get_logger().warn(f"guided reposition send failed: {e}")
            return False
