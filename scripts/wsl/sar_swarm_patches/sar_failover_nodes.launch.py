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

N_UAVS = 6   # 預設機數(向下相容);實際值見 _swarm_count()


def _swarm_count():
    """本次要起幾架。預設 6,由環境變數 SWARM_COUNT 覆寫。

    ⚠️⚠️ 2026-08-05 之前這個檔案把角色**寫死**成 6 架:
             backup_node  uav_id: 6
             wingman      for vid in (2, 3, 4, 5)
       用 N=2 起 SITL 時,launch 照樣會生出 UAV3/4/5 的僚機節點與 UAV6 的備用
       長機 —— 那四架**根本不存在**。節點不會報錯,只會安靜地等一個永遠不會
       出現的機體;而 `ros2 node list` 看起來一切正常。這正是本專案的招牌
       失敗形狀:「看起來有跑,實際上半套」。

    ⚠️ 為什麼用環境變數而不是 LaunchConfiguration:角色分配要跑 Python 的
       for 迴圈,必須在**建 description 的當下**就知道 N,而 LaunchConfiguration
       要到執行期才解析。要用它就得包 OpaqueFunction,對這個檔案是過度設計。
       swarm_ros2_up.sh 會 export SWARM_COUNT。
    """
    import os
    try:
        n = int(os.environ.get("SWARM_COUNT", N_UAVS))
    except (TypeError, ValueError):
        n = N_UAVS
    return max(1, min(n, 12))


def _roles(n):
    """(長機, 僚機清單, 備用長機或 None)。

    規則:長機恆為 UAV1;n >= 2 時**最後一架**當備用長機,其餘是僚機。

        n=1 → 1 /  —      /  —      單機
        n=2 → 1 /  —      /  2      一長一備長
        n=3 → 1 /  2      /  3      多一台僚機
        n=4 → 1 /  2,3    /  4
        n=6 → 1 /  2,3,4,5/  6      與寫死版逐項相同

    ⚠️⚠️ 2026-08-07 訂正。舊註解寫著「n == 2 刻意不配備用長機 —— 把第二架
       拿去當備援等於**完全沒有編隊**」,那句話**在事實上是錯的**:
       `backup_node.py:11-13` 明文「slot v_tail_dist directly behind the leader,
       rotating with the leader heading, **so UAV6 is part of the formation**」,
       而 `:214-226 _formation_control()` 逐拍算 slot 誤差、跑追趕、發
       formation_status。備長**是**編隊成員 ⇒ n=2 的「一長一備長」就是一個
       二機編隊,而且比「一長一僚」多了失效接管。
    ⚠️ n == 6 逐位元不變:述詞 `n>=3` 與 `n>=2` 只在 n=2 這一個整數上取值不同;
       n=6 兩者皆 True ⇒ backup=6、wingmen 生成式一字未動 ⇒ 回傳
       (1, [2,3,4,5], 6) 完全相同。
    ⚠️ 「發布 formation_status 的節點數 = n − 1」這個不變量在改動兩側都成立
       (n=2:改前 wingmen=[2]+backup=None,改後 wingmen=[]+backup=2,都是 1)
       ⇒ master 的 expected_followers = n−1 不必動。
    """
    backup = n if n >= 2 else None
    wingmen = [v for v in range(2, n + 1) if v != backup]
    return 1, wingmen, backup


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
_FORMATION_DEFAULTS = {"nominal_speed": "135.0", "wait_speed": "90.0",
                       "formed_radius": "200.0", "catch_speed": "157.0",
                       "catch_radius": "528.0",
                       "v_spacing_m": "4000.0", "v_angle_deg": "35.0",
                       # max-consensus 將長機速度連續化（見 master_node）
                       "close_time": "60.0",
                       # 動態爬升率限幅(見 wingman_node._climb_authority)
                       "climb_gate_hi": "1000.0", "climb_cap_lo": "1.5",
                       # 高度閘門:爬升段關閉協同讓速(見 master/wingman)
                       "cruise_alt_threshold": "3000.0",
                       # 閘門遲滯 + 最短駐留(見 wingman_node._update_climb_phase)。
                       # ⚠️ 沒有遲滯時 P91 的 UAV5 在 200 s 內翻了 10 次,
                       #    每次只低於門檻 1–23 m,卻每次都把追趕速度與側向
                       #    展開角一起打掉 —— 階段 2 的極限環就是這樣來的。
                       "cruise_gate_hyst_m": "150.0",
                       "cruise_gate_dwell_s": "5.0",
                       # master 端的閘門獨立且預設關(0=關);見 master_node 說明
                       "master_alt_gate": "0.0",
                       # 側向展開的混合高度帶(見 wingman_node._v_target)
                       "expand_time_s": "120.0",
                       # 前置角投影 L1(m)+ 巡航段坡度上限(度)。
                       # ⚠️ 三趟實測 bearing_to_wp 中位數都是 −73°~−86°(正橫追蹤),
                       #    正橫追蹤要求持續大坡度 ⇒ 誘導阻力 ∝ n² ⇒ U1 掉 1000 m。
                       # 飛控接受 DO_CHANGE_SPEED 的區間;超出會**整發拒收**
                       # (MAV_RESULT=4)。要和 SITL 的 AIRSPEED_MIN/MAX 對齊。
                       "airspeed_min": "25.0", "airspeed_max": "165.0",
                       "l1_lookahead_m": "1500.0",
                       "cruise_roll_limit_deg": "0.0",   # 0 = 不動飛控(見 wingman_node)
                       # 收工還原值(tot_node 不斷言 ROLL_LIMIT_DEG,不還會留到俯衝)
                       "roll_limit_restore_deg": "80.0",
                       "v_tail_dist_m": "6000.0",
                       # 錐形隊形的半頂角(formation_shape=CONE 才生效)
                       "cone_half_deg": "30.0",
                       # ⚠️ 編隊高度先前完全沒有從 launch 傳出來,一律吃節點預設 50 m。
                       #    長程表演要在 3000 m 高度巡航時,僚機會全部飛去 50 m、
                       #    長機自己爬到巡航高度 —— 實測六機散在 10 km、一架落地。
                       #    formation_alt = 僚機基準高度、layer_sep = 每架的分層間隔、
                       #    patrol_alt = 長機巡航高度(三者要在同一個高度帶)。
                       "formation_alt": "50.0", "layer_sep": "15.0",
                       "patrol_alt": "80.0",
                       # WP 追蹤：導引航點刷新間隔 / slot 移動門檻。
                       # 越小＝slot 航點越連續刷新 → 少「飛到舊點盤旋再跳」的 S 形蛇行。
                       # ⚠️ 預設 3.0/25.0 對移動編隊太粗，會 S 形；~1.0/10.0 順很多。
                       "wp_interval": "1.0", "wp_move_thresh": "40.0",
                       "patrol_box_m": "1500.0", "formed_hysteresis": "1.4",
                       "wp_lead_time": "4.0",
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
    # ── 機數與角色:**必須在建 master_node 之前算出來** ─────────────────
    # ⚠️⚠️ 2026-08-06 修掉的靜默失敗:master_node.py:107 的
    #    `expected_followers` 寫死 5,而 :222 `if len(fresh) < self._expected_followers:
    #    return False` ⇒ **n=2 時只有 1 個發布者(UAV2),1 < 5 恆真,
    #    「成形」永遠不會被宣告**。實測 /tmp/nodes.log 的 `formation formed`
    #    出現 **0 次**,只有連續的「leader slowing … worst follower err 3844 m」。
    #    而這個參數先前**沒有任何環境變數通路**,launch 檔也沒給 ⇒ 無法修正。
    #    發布者是誰:wingman_node.py:303 與 backup_node.py:123 會發
    #    /swarm/formation_status,master_node.py:188 只訂閱 ⇒ **發布者數 = n − 1**。
    #    (n=6 ⇒ 僚機 4 + 備長 1 = 5,與先前寫死的值逐項相同 ⇒ 六機航次零影響。)
    _n = _swarm_count()
    _master_id, _wingmen, _backup_id = _roles(_n)
    _expected_followers = max(1, _n - 1)
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
        DeclareLaunchArgument("expand_time_s",
                              default_value=_FORMATION_DEFAULTS["expand_time_s"],
                              description="側向展開的時間常數(s);避免 slot 橫向跑贏飛機"),
        DeclareLaunchArgument("airspeed_min",
                              default_value=_FORMATION_DEFAULTS["airspeed_min"],
                              description="飛控 AIRSPEED_MIN;低於此的速度指令會被拒收"),
        DeclareLaunchArgument("airspeed_max",
                              default_value=_FORMATION_DEFAULTS["airspeed_max"],
                              description="飛控 AIRSPEED_MAX;高於此的速度指令會被拒收"),
        DeclareLaunchArgument("l1_lookahead_m",
                              default_value=_FORMATION_DEFAULTS["l1_lookahead_m"],
                              description="前置角投影距離 L1(m);把導引點推離正橫"),
        DeclareLaunchArgument("cruise_roll_limit_deg",
                              default_value=_FORMATION_DEFAULTS["cruise_roll_limit_deg"],
                              description="巡航段最大坡度(度);壓誘導阻力 n²"),
        DeclareLaunchArgument("roll_limit_restore_deg",
                              default_value=_FORMATION_DEFAULTS["roll_limit_restore_deg"],
                              description="收工還原的坡度上限(度);俯衝要用"),
        DeclareLaunchArgument("master_alt_gate",
                              default_value=_FORMATION_DEFAULTS["master_alt_gate"],
                              description="長機端高度閘門(0=關);與僚機端分開"),
        DeclareLaunchArgument("cruise_alt_threshold",
                              default_value=_FORMATION_DEFAULTS["cruise_alt_threshold"],
                              description="爬升/巡航的高度閘門(m);其下關閉協同讓速"),
        DeclareLaunchArgument("cruise_gate_hyst_m",
                              default_value=_FORMATION_DEFAULTS["cruise_gate_hyst_m"],
                              description="高度閘門遲滯(m);退回爬升段要低於 thr-此值"),
        DeclareLaunchArgument("cruise_gate_dwell_s",
                              default_value=_FORMATION_DEFAULTS["cruise_gate_dwell_s"],
                              description="退回爬升段前要連續低於遲滯線的秒數"),
        DeclareLaunchArgument("climb_gate_hi", default_value=_FORMATION_DEFAULTS["climb_gate_hi"],
                              description="slot 誤差超過此(m)就壓爬升率、讓推力給水平追趕"),
        DeclareLaunchArgument("climb_cap_lo", default_value=_FORMATION_DEFAULTS["climb_cap_lo"],
                              description="大落後時的爬升率上限 m/s"),
        DeclareLaunchArgument("close_time", default_value=_FORMATION_DEFAULTS["close_time"],
                              description="max-consensus：期望收掉最差落後量的秒數"),
        DeclareLaunchArgument("v_spacing_m", default_value=_FORMATION_DEFAULTS["v_spacing_m"],
                              description="V 隊形相鄰 rank 間距 m（＝編隊間隔；越小＝越密）"),
        DeclareLaunchArgument("v_angle_deg", default_value=_FORMATION_DEFAULTS["v_angle_deg"],
                              description="V 半角（度）；後左=180+θ、後右=180-θ"),
        DeclareLaunchArgument("v_tail_dist_m", default_value=_FORMATION_DEFAULTS["v_tail_dist_m"],
                              description="備長(UAV6) 在長機正後方的跟隨距離 m"),
        # ⚠️ 隊形是字串,不能塞進 _formation_params()(那個 dict 一律轉 float)。
        DeclareLaunchArgument("formation_shape", default_value="V",
                              description="固定 slot 的隊形:V(平面雁型)|CONE(立體錐形)"),
        DeclareLaunchArgument("cone_half_deg", default_value=_FORMATION_DEFAULTS["cone_half_deg"],
                              description="錐形半頂角(度);越大越扁、越小越細長"),
        DeclareLaunchArgument("formation_alt", default_value=_FORMATION_DEFAULTS["formation_alt"],
                              description="僚機編隊基準高度 m(要和 patrol_alt 同高度帶)"),
        DeclareLaunchArgument("layer_sep", default_value=_FORMATION_DEFAULTS["layer_sep"],
                              description="僚機之間的高度分層間隔 m(防撞)"),
        DeclareLaunchArgument("patrol_alt", default_value=_FORMATION_DEFAULTS["patrol_alt"],
                              description="長機巡航高度 m"),
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
             parameters=[{"uav_id": _master_id, "use_mavros": use_mavros,
                          "auto_patrol": auto_patrol,
                          "auto_arm": auto_arm, "auto_takeoff": auto_takeoff,
                          "launch_type": launch_type,
                          # ⭐ 見上面 _expected_followers 的說明:寫死 5 會讓
                          #    n<6 的航次永遠判不出「成形」,而且完全靜默。
                          "expected_followers": _expected_followers,
                          **origin, **formation}]),
    ]
    # ── 角色分配(N 感知)────────────────────────────────────────────────
    #    n=6 ⇒ master=1 / wingmen=(2,3,4,5) / backup=6,與 2026-08-05 之前的
    #    寫死版逐項相同 ⇒ 既有六機航次零影響。
    #    ⚠️ _n / _roles / _expected_followers 已在 nodes 清單**之前**算好(見那裡),
    #       因為 master_node 就在清單裡,需要 _master_id 與 _expected_followers。
    if _backup_id is not None:
        nodes.append(
            Node(package="sar_swarm", executable="backup_node", name="backup_node",
                 output="screen",
                 # ⚠️⚠️ 這裡的 9.0 會**覆寫** backup_node 自己宣告的 timeout。
                 #    實測 AN1/AO1 的 log:「backup UAV6 up, watching UAV1 (timeout 6.0 s)」
                 #    —— 節點端已經改成 9.0,但 launch 這一行曾把它蓋回 6.0,
                 #    節點的預設值形同虛設。這正是本專案的招牌陷阱:
                 #    「改了參數卻沒作用,而且沒有任何錯誤訊息」。
                 #    寬間距下要和 Raft 選舉窗口(8–12 s)與 DR(45 s)對齊 ⇒ 9.0。
                 parameters=[{"uav_id": _backup_id, "master_id": _master_id,
                              "timeout": 9.0,
                              # ⚠️⚠️ 2026-08-07 補:backup_node.py 的
                              #    `expected_followers` 預設寫死 4,而 launch
                              #    從來沒傳過 ⇒ 永遠是 4。兩個消費點都會壞:
                              #    (a) `_eligible()` 要 `fresh >= min(2, expected)`,
                              #        而 `_heard` 排除自己 ⇒ 只數僚機。
                              #        n=3 只有 1 架僚機 ⇒ 永遠不能接管;
                              #        n=2 僚機數 0 ⇒ 更不可能。
                              #        **而 n=2 配備長的唯一理由就是失效接管。**
                              #    (b) 代理長機手風琴 `len(fresh) >= expected`
                              #        在 n≤4 恆假 ⇒ 接管後永久停在 wait_speed 90。
                              # ⚠️ 語意與 master 的 n−1 **不同**:備長聽得到的
                              #    只有僚機,所以是 n−2。
                              # n=6 ⇒ max(0, 4) = 4 = 現行寫死值 ⇒ 逐位元相同。
                              "expected_followers": max(0, _n - 2),
                              "use_mavros": use_mavros,
                              "auto_arm": auto_arm, "auto_takeoff": auto_takeoff,
                              "launch_type": launch_type, **origin, **formation}]))
    for vid in _wingmen:
        nodes.append(Node(package="sar_swarm", executable="wingman_node",
                          name=f"wingman_node_{vid}", output="screen",
                          parameters=[{"uav_id": vid, "master_id": _master_id,
                                       # ⚠️⚠️ 2026-08-07 補:wingman_node.py 的
                                       #    `n_uavs` 預設寫死 6,launch 從沒傳過
                                       #    ⇒ 永遠是 6(實測 log:`up, n=6`)。
                                       #    n≤5 無害(多餘的訂閱收不到訊息);
                                       #    **n≥7 致命**:UAV7 訂閱不到自己
                                       #    ⇒ fleet.get(uav_id) 恆 None
                                       #    ⇒ 追趕通道整個關閉(err=None ⇒ nominal)
                                       #    ⇒ 回報 err=-1 被 master 丟棄
                                       #    ⇒ formation formed 永遠不出現。
                                       # n=6 ⇒ 傳 6 = 現行預設 ⇒ 逐位元相同。
                                       "n_uavs": _n,
                                       "use_mavros": use_mavros,
                                       "use_biomimetic": use_biomimetic,
                                       "auto_arm": auto_arm,
                                       "auto_takeoff": auto_takeoff,
                                       # 隊形(字串)單獨傳,見 DeclareLaunchArgument 的說明
                                       "formation_shape": LaunchConfiguration(
                                           "formation_shape"),
                                       "launch_type": launch_type, **origin, **formation}]))

    # with_raft:=true -> one health-priority raft_node per aircraft (P0-A/B), fed
    # the REAL /uav{i}/mavros/battery + global_position. Each runs under namespace
    # /uav{i} only to get a unique node name (raft_node hardcodes the name); all of
    # raft_node's topics (/raft/*, /uav{i}/mavros/*, /swarm/*) are ABSOLUTE, so the
    # namespace does not remap them — the cluster still shares /raft/* correctly.
    # Single uniform election window so health (not role/id) decides the leader.
    # ⚠️ B-500 寬間距:選舉窗口從 3–6 s 放寬到 8–12 s,心跳週期 0.5→1.0 s。
    #    2–3 英里的 LoRa Mesh 鏈路會有秒級的丟包叢集,窗口太窄會讓「通訊抖動」
    #    被當成「節點死亡」而反覆改選(leader thrashing)。
    #    隨機化區間 (8,12) 的寬度 4 s 仍遠大於一次心跳來回,分票機率夠低。
    #    與 backup_node 的 timeout=9.0 對齊,兩條路徑不會互相搶接管。
    # ⚠️ raft 的 cluster_size 必須等於**實際**機數:多數決是 floor(n/2)+1,
    #    寫死 6 而只起 2 架 ⇒ 永遠湊不到 4 票,選舉一輪一輪空轉且不會報錯。
    for vid in range(1, _n + 1):
        nodes.append(Node(package="fleet_consensus", executable="raft_node",
                          namespace=f"uav{vid}", output="screen",
                          condition=IfCondition(with_raft),
                          parameters=[{"node_id": f"uav{vid}", "cluster_size": _n,
                                       "heartbeat_period": 1.0, "health_bias": True,
                                       "election_min": 8.0, "election_max": 12.0}]))
    return LaunchDescription(nodes)
