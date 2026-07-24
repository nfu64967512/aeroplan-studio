"""Run the heartbeat-watchdog role nodes (tasks 2-4) for a failover test.

Starts the master (UAV1), four wingmen (UAV2-5), and the backup (UAV6). No SITL
needed — master_node simulates its GeoPose. Kill the master and watch the backup
take over and the wingmen rebind:

  ros2 launch sar_swarm sar_failover_nodes.launch.py                 # sim GeoPose
  ros2 launch sar_swarm sar_failover_nodes.launch.py use_mavros:=true # real SITL via MAVROS
                                          # wingmen = simple fixed-slot STAR trailing UAV1
                                          # does NOT take off by default — operator/GCS launches;
                                          # nodes set GUIDED + stream formation once airborne.
                                          # add auto_takeoff:=true to auto-arm + climb out.
  ros2 launch sar_swarm sar_failover_nodes.launch.py use_mavros:=true use_biomimetic:=true
                                          # wingmen = biomimetic V (neighbour tracking)
  ros2 launch sar_swarm sar_failover_nodes.launch.py use_mavros:=true with_raft:=true
                                          # + health-priority Raft on real MAVROS health
  ros2 topic echo /swarm/heartbeat
  # then: pkill -f master_node    (backup UAV6 takes over within ~2.5 s, wingmen rebind)

Two failover layers, kept independent (no dual authority):
  /raft/heartbeat       Raft leader  -> LeaderWatch gates coordinator/gcs_bridge  (P0-A/B)
  /swarm/heartbeat+switch  master role -> wingman formation reference             (P0-C)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from sar_swarm.dds_profile import ensure_udp_only

# 原點預設（向下相容既有行為）。⚠️ 這 origin 必須與 SITL 生成點一致：
# master_node 用它建巡邏方框、僚機的後備固定 slot 也以它為參考。若 SITL 生在
# 桃園機場(25.0777,121.2325)卻沿用預設 22.60,120.00，長機 auto_patrol 會朝
# ~120km 外飛 → 整個編隊順移。故改為可由 launch 參數覆寫（見下方 DeclareLaunchArgument），
# 由 swarm_formation_up.sh 依 LOC 傳入 origin_lat/lon/heading_deg 讓兩邊同址。
# ⚠️ altitude 舊預設 1000（sim 巡航高度）在 MAVROS 實機模式下會讓長機爬到 1000m、
# 遠高於編隊(patrol_alt≈80m) → 看起來永遠不收斂。改為與編隊同高度帶（100m）。
ORIGIN_DEFAULTS = {"origin_lat": "22.60", "origin_lon": "120.00",
                   "altitude": "100.0", "speed": "30.0", "heading_deg": "90.0"}

N_UAVS = 6   # raft cluster size when with_raft:=true (majority = 4)


def _origin_params():
    """由 launch 參數建 origin 參數 dict；數值型別強制 float（declare_parameter 為 double）。"""
    return {k: ParameterValue(LaunchConfiguration(k), value_type=float)
            for k in ("origin_lat", "origin_lon", "altitude", "speed", "heading_deg")}


# 編隊位置約束（速度控制）可調參數 —— 由 launch 參數覆寫節點內建預設，
# 讓「長機等僚機、僚機追上長機」的行為可從 swarm_formation_up.sh 直接調。
#   nominal_speed  隊形成立後的巡航空速
#   wait_speed     有僚機落後時長機放慢等的空速（越小＝越肯等）
#   formed_radius  僚機距 slot < 此(m) 算「到位」（越小＝要求越嚴、長機更常等）
#   catch_speed    僚機落後時的追趕空速（越大＝追越兇）
#   catch_radius   僚機距 slot > 此(m) 用全速追（formed_radius~catch_radius 線性內插）
# V 隊形幾何（間隔）：
#   v_spacing_m    V 相鄰 rank 間距（僚機離長機的距離＝rank×此值）＝「編隊間隔」
#   v_angle_deg    V 半角（後左=180+θ、後右=180-θ）
#   v_tail_dist_m  備長(UAV6) 在長機正後方的跟隨距離
# ⚠️ formed_radius 必須 < v_spacing_m 才收得緊；否則「在 slot 250m 內就算到位」
#    但間距才 50m → 永遠鬆散脫隊、速度約束不觸發。
_FORMATION_DEFAULTS = {"nominal_speed": "14.0", "wait_speed": "10.0",
                       "formed_radius": "250.0", "catch_speed": "20.0",
                       "catch_radius": "400.0",
                       "v_spacing_m": "120.0", "v_angle_deg": "25.0",
                       "v_tail_dist_m": "220.0",
                       # WP 追蹤：導引航點刷新間隔 / slot 移動門檻。
                       # 越小＝slot 航點越連續刷新 → 少「飛到舊點盤旋再跳」的 S 形蛇行。
                       # ⚠️ 預設 3.0/25.0 對移動編隊太粗，會 S 形；~1.0/10.0 順很多。
                       "wp_interval": "1.0", "wp_move_thresh": "10.0",
                       "patrol_box_m": "1500.0", "formed_hysteresis": "1.4",
                       "wp_lead_time": "0.0",
                       # §4.5 轉彎程序:定大半徑弧 + 轉前減速(轉角耦合)
                       "turn_radius": "800.0", "preslow_margin": "250.0"}


def _formation_params():
    """速度控制參數 dict（float 型別）；spread 進 master/wingman/backup 節點。
    節點未宣告的鍵會被 ROS2 當作無害 override 忽略，故三種節點可共用同一份。"""
    return {k: ParameterValue(LaunchConfiguration(k), value_type=float)
            for k in _FORMATION_DEFAULTS}


def generate_launch_description():
    ensure_udp_only()    # UDP-only FastDDS before nodes spawn (SHM stalls subs on the Jetson)
    use_mavros = LaunchConfiguration("use_mavros")
    with_raft = LaunchConfiguration("with_raft")
    use_biomimetic = LaunchConfiguration("use_biomimetic")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_takeoff = LaunchConfiguration("auto_takeoff")
    auto_patrol = LaunchConfiguration("auto_patrol")
    launch_type = LaunchConfiguration("launch_type")
    origin = _origin_params()      # origin_lat/lon/altitude/speed/heading_deg（可被參數覆寫）
    formation = _formation_params()  # nominal/wait/catch 速度 + formed/catch 半徑（編隊位置約束）
    nodes = [
        DeclareLaunchArgument("origin_lat", default_value=ORIGIN_DEFAULTS["origin_lat"],
                              description="編隊/SITL 生成原點緯度（須與 SITL 生成點一致，否則 auto_patrol 順移）"),
        DeclareLaunchArgument("origin_lon", default_value=ORIGIN_DEFAULTS["origin_lon"],
                              description="編隊/SITL 生成原點經度"),
        DeclareLaunchArgument("altitude", default_value=ORIGIN_DEFAULTS["altitude"],
                              description="sim 模式巡航海拔；MAVROS 模式僅作初始占位"),
        DeclareLaunchArgument("speed", default_value=ORIGIN_DEFAULTS["speed"],
                              description="sim 模式長機速度"),
        DeclareLaunchArgument("heading_deg", default_value=ORIGIN_DEFAULTS["heading_deg"],
                              description="生成航向（度，正北順時針）；巡邏方框方位"),
        DeclareLaunchArgument("nominal_speed", default_value=_FORMATION_DEFAULTS["nominal_speed"],
                              description="隊形成立後的巡航空速 m/s"),
        DeclareLaunchArgument("wait_speed", default_value=_FORMATION_DEFAULTS["wait_speed"],
                              description="有僚機落後時長機放慢等的空速 m/s（越小＝越肯等）"),
        DeclareLaunchArgument("formed_radius", default_value=_FORMATION_DEFAULTS["formed_radius"],
                              description="僚機距 slot < 此(m) 算到位（越小＝要求越嚴、長機更常等）"),
        DeclareLaunchArgument("catch_speed", default_value=_FORMATION_DEFAULTS["catch_speed"],
                              description="僚機落後時的追趕空速 m/s（越大＝追越兇）"),
        DeclareLaunchArgument("catch_radius", default_value=_FORMATION_DEFAULTS["catch_radius"],
                              description="僚機距 slot > 此(m) 用全速追"),
        DeclareLaunchArgument("v_spacing_m", default_value=_FORMATION_DEFAULTS["v_spacing_m"],
                              description="V 隊形相鄰 rank 間距 m（＝編隊間隔；越小＝越密）"),
        DeclareLaunchArgument("v_angle_deg", default_value=_FORMATION_DEFAULTS["v_angle_deg"],
                              description="V 半角（度）；後左=180+θ、後右=180-θ"),
        DeclareLaunchArgument("v_tail_dist_m", default_value=_FORMATION_DEFAULTS["v_tail_dist_m"],
                              description="備長(UAV6) 在長機正後方的跟隨距離 m"),
        DeclareLaunchArgument("wp_interval", default_value=_FORMATION_DEFAULTS["wp_interval"],
                              description="導引航點刷新間隔 s（越小越連續、少 S 形；預設 1.0）"),
        DeclareLaunchArgument("wp_move_thresh", default_value=_FORMATION_DEFAULTS["wp_move_thresh"],
                              description="slot 移動超過此 m 才重發航點（越小越即時；預設 10）"),
        DeclareLaunchArgument("patrol_box_m", default_value=_FORMATION_DEFAULTS["patrol_box_m"],
                              description="巡邏方框邊長 m（大＝直線長少轉角，高速下僚機好收斂）"),
        DeclareLaunchArgument("formed_hysteresis", default_value=_FORMATION_DEFAULTS["formed_hysteresis"],
                              description="成形解除門檻倍數（巡航誤差膨脹 ~+70m，太小會 wait/cruise 震盪）"),
        DeclareLaunchArgument("wp_lead_time", default_value=_FORMATION_DEFAULTS["wp_lead_time"],
                              description="slot 前饋秒數（外推 lead×leader_speed 抵消導引 WP 滯後；0=關）"),
        DeclareLaunchArgument("turn_radius", default_value=_FORMATION_DEFAULTS["turn_radius"],
                              description="§4.5 弧轉半徑 m（slot 掃掠率=v_corner/R；大=轉角耦合好）"),
        DeclareLaunchArgument("preslow_margin", default_value=_FORMATION_DEFAULTS["preslow_margin"],
                              description="入彎點前多少 m 開始減速至 wait_speed"),
        DeclareLaunchArgument("use_mavros", default_value="false",
                              description="source GeoPose from MAVROS instead of sim"),
        DeclareLaunchArgument("with_raft", default_value="false",
                              description="also start one health-biased raft_node per "
                                          "aircraft, fed real /uav{i}/mavros health (P0-A/B)"),
        DeclareLaunchArgument("use_biomimetic", default_value="false",
                              description="wingmen formation: false=simple fixed-slot star "
                                          "trailing the master (UAV1, → UAV6 on failover); "
                                          "true=biomimetic V (neighbour tracking)"),
        # default: launch does NOT arm/take off — operator launches each aircraft
        # (GCS / manual); nodes only set GUIDED and stream formation once airborne.
        DeclareLaunchArgument("auto_takeoff", default_value="false",
                              description="false=launch does NOT take the aircraft off "
                                          "(operator/GCS launches); true=auto TAKEOFF+climb"),
        DeclareLaunchArgument("auto_arm", default_value="false",
                              description="false=launch does NOT arm; true=auto-arm"),
        # default off: leader holds (operator/GCS flies it). true=leader flies the
        # patrol box itself on guided WPs — steady heading legs, so the heading-relative
        # V slots stay put and the wingmen actually converge (formation speed-control test).
        DeclareLaunchArgument("auto_patrol", default_value="false",
                              description="true=leader auto-flies the patrol box "
                                          "(steady legs); false=leader holds / GCS-driven"),
        # runway = wheeled climb-out in TAKEOFF mode; catapult = fire servo 7 after arming
        # to release the launcher (SITL needs frame plane-catapult + TKOFF_THR_MINACC —
        # see start_sitl_swarm.sh LAUNCH=catapult).
        DeclareLaunchArgument("launch_type", default_value="runway",
                              description="runway | catapult (catapult fires servo 7 to "
                                          "release the launcher after arming)"),
        Node(package="sar_swarm", executable="master_node", name="master_node",
             output="screen",
             parameters=[{"uav_id": 1, "use_mavros": use_mavros, "auto_patrol": auto_patrol,
                          "auto_arm": auto_arm, "auto_takeoff": auto_takeoff,
                          "launch_type": launch_type, **origin, **formation}]),
        Node(package="sar_swarm", executable="backup_node", name="backup_node",
             output="screen",
             parameters=[{"uav_id": 6, "master_id": 1, "timeout": 6.0,
                          "use_mavros": use_mavros,
                          "auto_arm": auto_arm, "auto_takeoff": auto_takeoff,
                          "launch_type": launch_type, **origin, **formation}]),
    ]
    for vid in (2, 3, 4, 5):
        nodes.append(Node(package="sar_swarm", executable="wingman_node",
                          name=f"wingman_node_{vid}", output="screen",
                          parameters=[{"uav_id": vid, "master_id": 1,
                                       "use_mavros": use_mavros,
                                       "use_biomimetic": use_biomimetic,
                                       "auto_arm": auto_arm,
                                       "auto_takeoff": auto_takeoff,
                                       "launch_type": launch_type, **origin, **formation}]))

    # with_raft:=true -> one health-priority raft_node per aircraft (P0-A/B), fed
    # the REAL /uav{i}/mavros/battery + global_position. Each runs under namespace
    # /uav{i} only to get a unique node name (raft_node hardcodes the name); all of
    # raft_node's topics (/raft/*, /uav{i}/mavros/*, /swarm/*) are ABSOLUTE, so the
    # namespace does not remap them — the cluster still shares /raft/* correctly.
    # Single uniform election window so health (not role/id) decides the leader.
    for vid in range(1, N_UAVS + 1):
        nodes.append(Node(package="fleet_consensus", executable="raft_node",
                          namespace=f"uav{vid}", output="screen",
                          condition=IfCondition(with_raft),
                          parameters=[{"node_id": f"uav{vid}", "cluster_size": N_UAVS,
                                       "heartbeat_period": 0.5, "health_bias": True,
                                       "election_min": 3.0, "election_max": 6.0}]))
    return LaunchDescription(nodes)
