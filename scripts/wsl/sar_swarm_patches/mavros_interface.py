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
        except Exception as e:
            node.get_logger().warn(
                f"uav{vehicle_id} guided-WP link failed ({e}); formation flight disabled")
            self._wp = None

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
        """MAV_CMD_DO_CHANGE_SPEED: param1=0 airspeed, param2=speed, param3=throttle(-1)."""
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_CHANGE_SPEED
        req.param1 = 0.0                  # 0 = airspeed
        req.param2 = float(airspeed)
        req.param3 = -1.0                 # throttle: no change
        req.param4 = 0.0
        return self.cli_cmd.call_async(req)

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
            # param1=-1 keep current speed, param2=0, param3=0 radius, param4=NaN yaw.
            self._wp.mav.command_int_send(
                self._wp_sysid, 1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                -1.0, 0.0, 0.0, float("nan"),
                int(lat * 1e7), int(lon * 1e7), float(alt_rel))
            return True
        except Exception as e:
            self.node.get_logger().warn(f"guided reposition send failed: {e}")
            return False
