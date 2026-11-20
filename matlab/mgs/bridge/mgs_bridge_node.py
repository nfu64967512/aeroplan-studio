#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mgs_bridge_node.py — WSL 端 ROS 2 ↔ MATLAB 橋接。

把 WSL 裡的多機導引堆疊接到 Windows 上的 MATLAB(matlab/mgs),兩個方向都通:

    ROS 2  ──► TCP ──►  MATLAB    現場狀態、tot_gate/tot_status、生效值契約
    MATLAB ──► TCP ──►  ROS 2     影子軌跡、SIL 指令、指定紅心

═══ 為什麼是 TCP 而不是直接讓 MATLAB 講 DDS ══════════════════════════════

本機 WSL2 是**預設的 NAT 網路模式**(`wsl -l -v` 是 2,`.wslconfig` 不存在,
WSL IP 172.17.155.121 與 Windows 不同網段)。DDS 的 multicast discovery 在
NAT 下**不會**跨到 Windows,所以 ROS Toolbox 直接開 `ros2node` 看得到自己、
看不到 WSL 裡的節點 —— 而且它**不會報錯**,只是一個 topic 都收不到。那正是
本專案最貴的那種失效形狀。

TCP 反過來是可靠的:WSL2 會把 Windows 的 `127.0.0.1:PORT` 轉發到 WSL 裡
listen 的同一個埠。所以 MATLAB 只要 `tcpclient("127.0.0.1", 5799)` 就通,
**不用改任何 WSL 網路設定**。

如果你真的要走原生 DDS(`c.io.transport = 'ros2'`),需要把 WSL 切到
mirrored 模式:在 `%USERPROFILE%\.wslconfig` 寫

    [wsl2]
    networkingMode=mirrored

再 `wsl --shutdown`。本機 WSL 2.7.12 + Windows 10.0.22621 支援。⚠️ 切換會
影響**所有** WSL 網路行為(含 SITL 的埠),所以預設不動它;`mgs.preflight`
會告訴你目前是哪一種。

═══ 線上格式 ═══════════════════════════════════════════════════════════

NDJSON:一行一個 JSON 物件,以 `\n` 分隔。沒有長度前綴、沒有二進位框架 ——
故意的,因為 `/swarm/tot_gate` 與 `/swarm/tot_status` 本來就是
`std_msgs/String` 裝 JSON,轉一手不需要換表示法。

每個訊息都有 `type` 與 `t`(WSL 的 `time.time()`)。schema 版本在 `hello`
握手裡對過,不一致就**斷線**,不是將就著跑。

  MATLAB → WSL
    {"type":"hello",    "schema":1, "peer":"matlab", "mode":"shadow|sil|offline"}
    {"type":"sub",      "topics":["status","gate","state"]}
    {"type":"shadow",   "t_sim":.., "uavs":[{...}]}      → /mgs/shadow
    {"type":"cmd",      "uav":1, "roll":..,"pitch":..,"thr":..} → /mgs/cmd
    {"type":"target",   "lat":..,"lon":..,"amsl":..}     → /swarm/tot_target
    {"type":"contract", "snapshot":true}                 → 觸發生效值讀回
    {"type":"ping"}

  WSL → MATLAB
    {"type":"hello_ack","schema":1,"ros_distro":"humble","n_uav":6,"topics":[..]}
    {"type":"status",   "data":{...}}   /swarm/tot_status 原樣轉發
    {"type":"gate",     "data":{...}}   /swarm/tot_gate   原樣轉發
    {"type":"state",    "uavs":[{...}]} MAVROS 逐機狀態,10 Hz
    {"type":"contract", "ok":bool,"values":{...},"sources":{...},"stderr":".."}
    {"type":"error",    "what":"..."}
    {"type":"pong"}

⚠️⚠️ **`target` 是會真的讓飛機動的。** 它直接送到 `/swarm/tot_target`,
tot_node 收到就開打。所以它預設**關閉**,要 `--allow-target` 才會轉發;
沒開就回一則 error 說明,不是安靜地丟掉。

═══ 用法 ═══════════════════════════════════════════════════════════════

    # WSL,在 ROS2 環境已 source 之後
    python3 matlab/mgs/bridge/mgs_bridge_node.py --port 5799 --n 6

    # 允許 MATLAB 指定紅心(會讓飛機動,想清楚再開)
    python3 matlab/mgs/bridge/mgs_bridge_node.py --allow-target

⚠️ 本檔必須是 LF 換行。Windows 端改過之後請確認:CRLF 會讓 WSL 的
shebang 解析失敗,而錯誤訊息會指向完全不相干的行。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional

SCHEMA = 1
DEFAULT_PORT = 5799

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
except Exception as exc:  # pragma: no cover - only hit outside a ROS env
    sys.stderr.write(
        "mgs_bridge_node: rclpy import failed (%s).\n"
        "Source your ROS 2 environment first:\n"
        "    source /opt/ros/humble/setup.bash\n" % exc
    )
    raise

# MAVROS 訊息是可選的:沒有它照樣能轉發 tot_gate/tot_status,只是沒有逐機
# 精細狀態。**不靜默降級** —— hello_ack 會把實際訂到的 topic 列出來。
try:
    from sensor_msgs.msg import NavSatFix, Imu
    from nav_msgs.msg import Odometry
    HAVE_MAVROS_MSGS = True
except Exception:
    HAVE_MAVROS_MSGS = False

# ⚠️ 指令側是分開的一個 try:沒有 mavros_msgs 時,狀態側照樣要能跑。
#    合在一起 import 會讓「拿不到指令」升級成「連狀態都沒有」。
try:
    from mavros_msgs.msg import AttitudeTarget, NavControllerOutput
    from mavros_msgs.srv import MessageInterval
    HAVE_CMD_MSGS = True
except Exception:
    HAVE_CMD_MSGS = False

try:
    from mavros_msgs.msg import VfrHud
    HAVE_VFR = True
except Exception:
    HAVE_VFR = False

#: MAVLink messages mgs.shadowDiff needs, and the rate to ask for.
#:
#: ⚠️⚠️ TWO SEPARATE PROBLEMS, AND ONLY ONE OF THEM IS OBVIOUS.
#:
#: 1. ATTITUDE_TARGET (83) is not streamed by ArduPlane at all unless asked,
#:    and tot_node sends SET_ATTITUDE_TARGET on its OWN direct link, so
#:    without this request MAVROS never sees the guidance command.
#:
#: 2. Everything else IS streamed -- at about **4 Hz**.  Measured on this
#:    stack: global_position 3.97, imu/data 3.97, vfr_hud 4.03.  Running the
#:    bridge at --state-hz 20 does NOT fix that: it re-emits the latest
#:    cached value 20 times a second, so the file looks like 20 Hz while
#:    carrying 4 Hz of information.  The guidance replay in mgs.shadowDiff
#:    then runs its alpha low-pass at the right rate on stale inputs, and the
#:    plant layer differentiates a 4 Hz signal.  Ask for the real rate.
#:
#: ⚠️ 20 Hz = 50000 us, comfortably above the 25 ms SET_MESSAGE_INTERVAL floor.
MSG_ATTITUDE_TARGET = 83
WANTED_MSGS = {
    30: "ATTITUDE",               # -> imu/data
    33: "GLOBAL_POSITION_INT",    # -> global_position/global + /local
    62: "NAV_CONTROLLER_OUTPUT",  # -> nav_controller_output/output
    74: "VFR_HUD",                # -> vfr_hud (the APPLIED throttle)
    83: "ATTITUDE_TARGET",        # -> setpoint_raw/target_attitude
}

TARGET_TOPIC = "/swarm/tot_target"   # 與 tot_node.py:126 同一個字串
STATUS_TOPIC = "/swarm/tot_status"
GATE_TOPIC = "/swarm/tot_gate"
SHADOW_TOPIC = "/mgs/shadow"
CMD_TOPIC = "/mgs/cmd"

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CONTRACT_VERIFY = os.path.join(REPO_ROOT, "scripts", "wsl", "contract_verify.py")


def sensor_qos(depth: int = 10) -> "QoSProfile":
    """MAVROS 的每機 topic 要用 BEST_EFFORT 訂。

    ⚠️⚠️ 這條是靜默失效的教科書例子。MAVROS 用 **BEST_EFFORT** 發感測資料,
    而 rclpy 的預設訂閱是 **RELIABLE** ⇒ QoS 不相容 ⇒ **一則訊息都收不到**,
    而 ROS 只在 discovery 當下印一行 WARN 就沒有下文了:

        New publisher discovered on topic '/uav1/mavros/global_position/global',
        offering incompatible QoS. No messages will be received from it.
        Last incompatible policy: RELIABILITY

    橋接照常連上、握手照常成功、MATLAB 照常收到 hello_ack —— 只是 state 永遠
    是空的。2026-08-22 實測踩到。

    方向是不對稱的:BEST_EFFORT 的**訂閱者**收得到 RELIABLE 的發布者(可靠性
    更強,滿足較弱的要求),反過來不行。所以這裡一律用 BEST_EFFORT。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def latched_qos(depth: int = 1) -> "QoSProfile":
    """tot_node 的 gate 是 latched(TRANSIENT_LOCAL),要對上才收得到。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class MgsBridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("mgs_bridge")
        self.args = args
        self._conns: List["ClientConn"] = []
        self._conns_lock = threading.Lock()
        self.subscribed_topics: List[str] = []

        # ── ROS → MATLAB ────────────────────────────────────────────────
        self.create_subscription(
            String, STATUS_TOPIC, self._on_status, 10)
        self.subscribed_topics.append(STATUS_TOPIC)
        self.create_subscription(
            String, GATE_TOPIC, self._on_gate, latched_qos())
        self.subscribed_topics.append(GATE_TOPIC)

        self.uav: Dict[int, Dict[str, Any]] = {}
        if HAVE_MAVROS_MSGS and args.n > 0:
            for i in range(1, args.n + 1):
                ns = f"/uav{i}/mavros"
                self.create_subscription(
                    NavSatFix, f"{ns}/global_position/global",
                    self._mk_fix_cb(i), sensor_qos())
                # ⚠️ NOT local_position/*.  Two reasons, both measured:
                #   1. the `local_position` plugin is not in this deployment's
                #      slim_pluginlists.yaml allowlist, and
                #   2. the flight controller does not send LOCAL_POSITION_NED
                #      at all (checked against the raw MAVLink stream on a
                #      spare MAVProxy fan-out), so enabling the plugin would
                #      not help either.
                # Attitude comes from imu/data (ATTITUDE) and velocity from
                # the global_position Odometry, both of which do publish.
                self.create_subscription(
                    Imu, f"{ns}/imu/data",
                    self._mk_att_cb(i), sensor_qos())
                self.create_subscription(
                    Odometry, f"{ns}/global_position/local",
                    self._mk_vel_cb(i), sensor_qos())
                self.subscribed_topics.append(f"{ns}/*")

                # ── 指令側 ──────────────────────────────────────────────
                # 影子比對要分得出分歧在哪一層,而位置只是最後一層的結果。
                # 這兩個 topic 給的是中間兩層:
                #   target_attitude       節點送出去的姿態指令(夾制前的意圖)
                #   nav_controller_output 飛控**夾制後**的 nav_roll/nav_pitch
                # 對上模型的 cmd.pitch 與 u(2),就能把落差歸給
                # 「導引律」/「韌體夾制」/「機體響應」其中一層。
                # aspd_error 另外直接對上時間通道(v_req - V)。
                if HAVE_VFR:
                    # ⭐ The APPLIED throttle (%), post-clamp and post-slew.
                    # That is what the plant actually got, and it is what
                    # mgs.shadowDiff's L3 needs.  ATTITUDE_TARGET.thrust is
                    # the COMMAND side and is a different quantity.
                    self.create_subscription(
                        VfrHud, f"{ns}/vfr_hud",
                        self._mk_vfr_cb(i), sensor_qos())
                if HAVE_CMD_MSGS:
                    self.create_subscription(
                        AttitudeTarget, f"{ns}/setpoint_raw/target_attitude",
                        self._mk_att_tgt_cb(i), sensor_qos())
                    # ⚠️ 名字是 `nav_controller_output/output`,**多一層**。
                    #    訂錯名字的症狀極具誤導性:訂閱**本身**會把那個 topic
                    #    變出來,所以 `ros2 topic list` 看得到它、
                    #    `ros2 topic info` 顯示 Subscription count: 1 /
                    #    Publisher count: 0 —— 看起來像「topic 存在但沒人發」,
                    #    而不是「我名字打錯了」。2026-08-22 實測踩到。
                    self.create_subscription(
                        NavControllerOutput, f"{ns}/nav_controller_output/output",
                        self._mk_navout_cb(i), sensor_qos())

        # ── MATLAB → ROS ────────────────────────────────────────────────
        self.shadow_pub = self.create_publisher(String, SHADOW_TOPIC, 10)
        self.cmd_pub = self.create_publisher(String, CMD_TOPIC, 10)
        # ⚠️ target 的 publisher 一定要建,即使 --allow-target 沒開:延後建立
        #    會讓「第一次送 target」多一個 discovery 延遲,而那個延遲看起來
        #    就像「指令沒送出去」。開關擋在轉發那一層,不擋在建立這一層。
        self.target_pub = self.create_publisher(String, TARGET_TOPIC, latched_qos())

        self.create_timer(1.0 / max(args.state_hz, 1), self._pump_state)
        # 訂閱名字打錯是靜默的(見上面的說明),所以開機 5 秒後點名一次:
        # 每個訂閱的 topic 有沒有**真的**發布者。有問題就大聲講,不要讓
        # 「收不到」跟「沒有分歧」長得一樣。
        self._audit_timer = self.create_timer(20.0, self._audit_publishers)
        self._audit_last = None
        # ⚠️ ArduPlane does not stream ATTITUDE_TARGET (msg 83) unless asked.
        #    Without it there is no guidance layer to compare against at all.
        #
        # ⚠️⚠️ AND THE REQUEST DOES NOT SURVIVE A STACK RESTART.  The batch
        #    runners rebuild SITL + MAVROS for every sortie, which throws away
        #    a request made before the rebuild -- so a one-shot request only
        #    works if you happen to sequence it right, and fails silently if
        #    you do not.  Re-request periodically instead; the command is
        #    idempotent, and only the first success and later transitions are
        #    logged so the log does not fill up.
        self._msgreq_timer = self.create_timer(20.0, self._request_streams)
        self._msgreq_ok = set()

        self.get_logger().info(
            "mgs_bridge up: TCP %s:%d, schema %d, %d MAVROS namespaces, "
            "target forwarding %s"
            % (args.host, args.port, SCHEMA, args.n if HAVE_MAVROS_MSGS else 0,
               "ENABLED" if args.allow_target else "disabled (--allow-target)")
        )
        if not HAVE_MAVROS_MSGS:
            self.get_logger().warning(
                "mavros/sensor/geometry messages not importable -- per-UAV "
                "state will be EMPTY.  tot_gate/tot_status still forward.")
        if not HAVE_CMD_MSGS:
            self.get_logger().warning(
                "mavros_msgs not importable -- the COMMAND side "
                "(target_attitude, nav_controller_output) is absent, so "
                "mgs.shadowDiff can only compare the response layer and "
                "CANNOT attribute a divergence to guidance vs clamp vs plant.")

    def _request_streams(self) -> None:
        if not HAVE_CMD_MSGS:
            return
        for i in range(1, self.args.n + 1):
            svc = f"/uav{i}/mavros/set_message_interval"
            cli = self.create_client(MessageInterval, svc)
            if not cli.wait_for_service(timeout_sec=1.0):
                if i in self._msgreq_ok:
                    self._msgreq_ok.discard(i)
                    self.get_logger().warning(
                        "uav%d lost %s -- the stack probably restarted; will "
                        "keep re-requesting msg %d"
                        % (i, svc, MSG_ATTITUDE_TARGET))
                continue
            for mid in sorted(WANTED_MSGS):
                req = MessageInterval.Request()
                req.message_id = mid
                req.message_rate = float(self.args.state_hz)
                cli.call_async(req)
            if i not in self._msgreq_ok:
                self._msgreq_ok.add(i)
                self.get_logger().info(
                    "uav%d: requested %s at %.0f Hz -- re-sent every 20 s so a "
                    "stack rebuild cannot silently drop it"
                    % (i, ", ".join("%d/%s" % (m, WANTED_MSGS[m])
                                    for m in sorted(WANTED_MSGS)),
                       self.args.state_hz))

    def _audit_publishers(self) -> None:
        # Repeats, and only speaks when the answer CHANGES -- a stack rebuild
        # mid-session silently removes publishers, and a one-shot audit taken
        # before the rebuild would keep saying everything is fine.
        want = []
        for i in range(1, self.args.n + 1):
            ns = f"/uav{i}/mavros"
            want += [f"{ns}/global_position/global",
                     f"{ns}/imu/data",
                     f"{ns}/global_position/local"]
            if HAVE_CMD_MSGS:
                want += [f"{ns}/nav_controller_output/output"]
            if HAVE_VFR:
                want += [f"{ns}/vfr_hud"]
        dead = [t for t in want if self.count_publishers(t) == 0]
        key = tuple(sorted(dead))
        if key == self._audit_last:
            return
        self._audit_last = key
        if dead:
            self.get_logger().warning(
                "%d subscribed topics have NO PUBLISHER: %s. "
                "A subscription CREATES the topic name, so a typo looks "
                "exactly like 'the topic exists but is quiet'.  Check the "
                "name against `ros2 topic list` before assuming the data is "
                "simply not being produced." % (len(dead), ", ".join(dead)))
        else:
            self.get_logger().info(
                "publisher audit: all %d subscribed topics have a publisher"
                % len(want))

    # ── ROS callbacks ──────────────────────────────────────────────────
    def _on_status(self, msg: String) -> None:
        self._broadcast({"type": "status", "t": time.time(),
                         "data": _loads(msg.data)})

    def _on_gate(self, msg: String) -> None:
        self._broadcast({"type": "gate", "t": time.time(),
                         "data": _loads(msg.data)})

    def _mk_fix_cb(self, i: int):
        def cb(m):
            d = self.uav.setdefault(i, {})
            d["lat"] = m.latitude
            d["lon"] = m.longitude
            d["amsl"] = m.altitude
            d["t_fix"] = time.time()
        return cb

    def _mk_att_cb(self, i: int):
        def cb(m):
            d = self.uav.setdefault(i, {})
            q = m.orientation
            r, pi, yw = _quat_to_rpy(q.x, q.y, q.z, q.w)
            # MAVROS publishes attitude in ENU/ROS convention (x East, y North,
            # z Up).  The model works in NED.  Roll is the same, pitch is the
            # same, and yaw is offset: yaw_NED = 90 - yaw_ENU.
            d["roll_deg"] = math.degrees(r)
            d["pitch_deg"] = math.degrees(pi)
            d["yaw_deg"] = (90.0 - math.degrees(yw)) % 360.0
            d["t_pose"] = time.time()
        return cb

    def _mk_vel_cb(self, i: int):
        def cb(m):
            d = self.uav.setdefault(i, {})
            v = m.twist.twist.linear
            # MAVROS local_position 是 ENU。MATLAB 端 (mgs) 用 NED,轉換在
            # **這裡**做一次,不要兩邊各做一半 —— 那是符號錯誤的溫床。
            d["vn"], d["ve"], d["vd"] = v.y, v.x, -v.z
            d["V"] = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            d["t_vel"] = time.time()
        return cb

    def _mk_att_tgt_cb(self, i: int):
        def cb(m):
            d = self.uav.setdefault(i, {})
            q = m.orientation
            r, pi, yw = _quat_to_rpy(q.x, q.y, q.z, q.w)
            d["cmd_roll_deg"] = math.degrees(r)
            d["cmd_pitch_deg"] = math.degrees(pi)
            d["cmd_thrust"] = m.thrust
            d["t_cmd"] = time.time()
        return cb

    def _mk_vfr_cb(self, i: int):
        def cb(m):
            d = self.uav.setdefault(i, {})
            d["throttle_pc"] = float(m.throttle)
            d["airspeed"] = float(m.airspeed)
            d["groundspeed"] = float(m.groundspeed)
            d["t_vfr"] = time.time()
        return cb

    def _mk_navout_cb(self, i: int):
        def cb(m):
            d = self.uav.setdefault(i, {})
            # NavControllerOutput 的 nav_roll/nav_pitch 已經是**度**。
            d["nav_roll_deg"] = m.nav_roll
            d["nav_pitch_deg"] = m.nav_pitch
            d["aspd_error"] = m.aspd_error
            d["alt_error"] = m.alt_error
            d["wp_dist"] = m.wp_dist
            d["t_nav"] = time.time()
        return cb

    def _pump_state(self) -> None:
        if not self.uav:
            return
        uavs = []
        now = time.time()
        for i, d in sorted(self.uav.items()):
            row = {"id": i}
            row.update(d)
            # 資料新鮮度隨值一起送。收到一筆看起來正常但其實 40 秒前的狀態
            # 而不知道,是本專案付過代價的失效之一(長機位姿凍結仍在廣播)。
            ages = [now - d[k]
                    for k in ("t_fix", "t_pose", "t_vel", "t_cmd", "t_nav",
                              "t_vfr")
                    if k in d]
            row["age_s"] = round(max(ages), 3) if ages else None
            uavs.append(row)
        self._broadcast({"type": "state", "t": now, "uavs": uavs})

    # ── MATLAB → ROS ───────────────────────────────────────────────────
    def handle_message(self, conn: "ClientConn", msg: Dict[str, Any]) -> None:
        kind = msg.get("type")

        if kind == "hello":
            got = msg.get("schema")
            if got != SCHEMA:
                conn.send({"type": "error", "t": time.time(),
                           "what": "schema mismatch: bridge %d, peer %s. "
                                   "Refusing the connection rather than "
                                   "guessing at field names." % (SCHEMA, got)})
                conn.close()
                return
            conn.hello = True
            conn.send({"type": "hello_ack", "t": time.time(), "schema": SCHEMA,
                       "ros_distro": os.environ.get("ROS_DISTRO", "?"),
                       "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                       "n_uav": self.args.n,
                       "have_mavros_msgs": HAVE_MAVROS_MSGS,
                       "have_cmd_msgs": HAVE_CMD_MSGS,
                       "have_vfr": HAVE_VFR,
                       "allow_target": self.args.allow_target,
                       "topics": self.subscribed_topics})
            return

        if not conn.hello:
            conn.send({"type": "error", "t": time.time(),
                       "what": "send hello first (schema handshake)"})
            return

        if kind == "ping":
            conn.send({"type": "pong", "t": time.time()})

        elif kind == "shadow":
            self.shadow_pub.publish(String(data=json.dumps(msg)))

        elif kind == "cmd":
            self.cmd_pub.publish(String(data=json.dumps(msg)))

        elif kind == "target":
            if not self.args.allow_target:
                conn.send({"type": "error", "t": time.time(),
                           "what": "target forwarding is DISABLED. This "
                                   "message would have made real aircraft "
                                   "fly. Restart the bridge with "
                                   "--allow-target if that is what you want."})
                return
            payload = {k: msg[k] for k in ("lat", "lon", "amsl",
                                           "approach_from") if k in msg}
            if "lat" not in payload or "lon" not in payload:
                conn.send({"type": "error", "t": time.time(),
                           "what": "target needs lat and lon"})
                return
            self.target_pub.publish(String(data=json.dumps(payload)))
            self.get_logger().warning(
                "★ forwarded a TARGET from MATLAB to %s: %s"
                % (TARGET_TOPIC, payload))
            conn.send({"type": "target_ack", "t": time.time(),
                       "published": payload})

        elif kind == "contract":
            threading.Thread(target=self._contract_snapshot, args=(conn,),
                             daemon=True).start()

        else:
            conn.send({"type": "error", "t": time.time(),
                       "what": "unknown message type %r" % (kind,)})

    def _contract_snapshot(self, conn: "ClientConn") -> None:
        """讀回**現場生效值**,走 contract_verify.py --snapshot。

        ⚠️ 刻意不自己寫一份 `ros2 param get` 迴圈。讀回邏輯只能有一份:
        它要處理布林參數的字串形狀、逾時重試、以及「哪個節點擁有哪個參數」,
        每一項都已經在 contract_verify.py 裡踩過坑了。
        """
        if not os.path.isfile(CONTRACT_VERIFY):
            conn.send({"type": "contract", "t": time.time(), "ok": False,
                       "what": "contract_verify.py not found at %s"
                               % CONTRACT_VERIFY})
            return
        out = os.path.join("/tmp", "mgs_contract_%d.json" % os.getpid())
        try:
            p = subprocess.run(
                [sys.executable, CONTRACT_VERIFY, "--snapshot", out],
                capture_output=True, text=True, timeout=120)
        except subprocess.TimeoutExpired:
            conn.send({"type": "contract", "t": time.time(), "ok": False,
                       "what": "contract_verify.py timed out after 120 s"})
            return
        doc: Dict[str, Any] = {}
        if os.path.isfile(out):
            try:
                with open(out, "r", encoding="utf-8") as fh:
                    doc = json.load(fh)
            except Exception as exc:
                conn.send({"type": "contract", "t": time.time(), "ok": False,
                           "what": "snapshot unreadable: %s" % exc})
                return
        # ⚠️ rc != 0 代表**契約被違反**,而不是工具壞掉。原樣送出去,由 MATLAB
        #    端決定要不要中止 —— 這一層不做判斷,也不做美化。
        conn.send({"type": "contract", "t": time.time(),
                   "ok": p.returncode == 0,
                   "returncode": p.returncode,
                   "values": doc.get("values", {}),
                   "sources": doc.get("sources", {}),
                   "stdout": p.stdout[-8000:],
                   "stderr": p.stderr[-8000:]})

    # ── fan-out ────────────────────────────────────────────────────────
    def _broadcast(self, msg: Dict[str, Any]) -> None:
        line = json.dumps(msg, default=_default) + "\n"
        data = line.encode("utf-8")
        with self._conns_lock:
            dead = []
            for c in self._conns:
                if not c.hello:
                    continue
                if not c.send_raw(data):
                    dead.append(c)
            for c in dead:
                self._conns.remove(c)

    def add_client(self, c: "ClientConn") -> None:
        with self._conns_lock:
            self._conns.append(c)

    def drop_client(self, c: "ClientConn") -> None:
        with self._conns_lock:
            if c in self._conns:
                self._conns.remove(c)


class ClientConn:
    """一條 TCP 連線。NDJSON 進、NDJSON 出。"""

    def __init__(self, sock: socket.socket, addr, node: MgsBridge):
        self.sock = sock
        self.addr = addr
        self.node = node
        self.hello = False
        self.alive = True
        self.wlock = threading.Lock()
        self.sock.settimeout(None)

    def send(self, msg: Dict[str, Any]) -> bool:
        return self.send_raw((json.dumps(msg, default=_default) + "\n").encode("utf-8"))

    def send_raw(self, data: bytes) -> bool:
        if not self.alive:
            return False
        try:
            with self.wlock:
                self.sock.sendall(data)
            return True
        except OSError:
            self.alive = False
            return False

    def close(self) -> None:
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass

    def serve(self) -> None:
        buf = b""
        try:
            while self.alive:
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8"))
                    except Exception as exc:
                        self.send({"type": "error", "t": time.time(),
                                   "what": "bad JSON line: %s" % exc})
                        continue
                    try:
                        self.node.handle_message(self, msg)
                    except Exception as exc:
                        self.node.get_logger().error(
                            "handler raised on %r: %s" % (msg.get("type"), exc))
                        self.send({"type": "error", "t": time.time(),
                                   "what": "handler raised: %s" % exc})
        except OSError:
            pass
        finally:
            self.close()
            self.node.drop_client(self)
            self.node.get_logger().info("client %s disconnected" % (self.addr,))


def serve_forever(node: MgsBridge, args: argparse.Namespace) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(8)
    node.get_logger().info("listening on %s:%d" % (args.host, args.port))
    while True:
        sock, addr = srv.accept()
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn = ClientConn(sock, addr, node)
        node.add_client(conn)
        node.get_logger().info("client %s connected" % (addr,))
        threading.Thread(target=conn.serve, daemon=True).start()


def _loads(s: str) -> Any:
    try:
        return json.loads(s)
    except Exception:
        # tot_node 的 STANDBY gate 曾經送過含 NaN 的非法 JSON。原文轉出去,
        # 讓 MATLAB 端看得到到底收到什麼,不要在這裡吃掉。
        return {"_raw": s, "_parse_error": True}


def _default(o: Any) -> Any:
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None          # NaN/Inf 不是合法 JSON;送 null,不要送 NaN
    return str(o)


def _quat_to_rpy(x: float, y: float, z: float, w: float):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main() -> int:
    ap = argparse.ArgumentParser(description="ROS 2 <-> MATLAB bridge for mgs")
    ap.add_argument("--host", default="0.0.0.0",
                    help="bind address; 0.0.0.0 so Windows can reach it "
                         "through WSL2 loopback forwarding")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--n", type=int, default=6,
                    help="number of /uavI/mavros namespaces to subscribe")
    ap.add_argument("--state-hz", type=float, default=20.0,
                    help="per-UAV state rate. DEFAULT 20 = the node's own "
                         "TICK_PERIOD_S. mgs.shadowDiff replays the guidance "
                         "law at the recording rate, and the alpha low-pass "
                         "gain is tick_dt/alpha_tau, so any other rate is a "
                         "DIFFERENT FILTER: measured L1 residue on the "
                         "synthetic null case is 0.013 deg at 20 Hz, 0.037 "
                         "at 10 Hz, 0.023 at 50 Hz. "
                         "Change it only if you know why.")
    ap.add_argument("--allow-target", action="store_true",
                    help="allow MATLAB to publish %s. THIS MAKES REAL "
                         "AIRCRAFT FLY." % TARGET_TOPIC)
    args = ap.parse_args()

    rclpy.init()
    node = MgsBridge(args)
    threading.Thread(target=serve_forever, args=(node, args), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM(pkill)走的是這一條:context 已經被外部關掉了。
        # ⚠️ 這時候再呼叫 rclpy.shutdown() 會噴
        #    "rcl_shutdown already called",而那串 traceback 看起來像節點壞了。
        #    停止服務本來就該安靜。
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
