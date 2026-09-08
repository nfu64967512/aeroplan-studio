# ArduPilot QuadPlane 整合筆記（供 HV-DCCPP 實作代理使用）

來源：主控代理的 ArduPilot/MAVLink 知識（ArduPlane 4.4–4.6 世代），已用本機 pymavlink 2.4.49 驗證常數值。
**實作時以 pymavlink 的 `mavutil.mavlink.MAV_CMD_*` 常數為準，不要手寫數字。** 標示「待驗證」者需在文件中註明以實機韌體版本確認。

兩台平台都是 lift-and-cruise 4+1 → ArduPlane 韌體 + QuadPlane 功能（Q_ENABLE=1，Q_FRAME_CLASS=1 quad，Q_FRAME_TYPE 依馬達配置）。

## 1. 任務項目（MAV_MISSION_TYPE_MISSION）

| 用途 | MAV_CMD | 值 | param1 | param2 | param3 | param4 | x/y/z |
|---|---|---|---|---|---|---|---|
| 垂直起飛 | NAV_VTOL_TAKEOFF | 84 | 0 | 0 | 0 | yaw(deg, NaN=不指定) | lat, lon（可 0=當前）, alt=起飛完成高度（相對 home） |
| 強制轉換為固定翼 | DO_VTOL_TRANSITION | 3000 | 3 = MAV_VTOL_STATE_MC(3)/ 4 = MAV_VTOL_STATE_FW | | | | ArduPilot 在 NAV_VTOL_TAKEOFF 後進入下一個 NAV_WAYPOINT 時自動轉換；DO_VTOL_TRANSITION 可選 |
| 一般航點 | NAV_WAYPOINT | 16 | hold s（固定翼忽略） | 接受半徑 m（0=WP_RADIUS） | 通過半徑（0；負值=逆時針繞） | yaw | lat, lon, alt |
| 爬升／下降盤旋到指定高度 | NAV_LOITER_TO_ALT | 31 | heading_required(0/1) | 半徑 m（正=順時針，負=逆時針） | 0 | xtrack(0/1) | lat, lon, alt=目標高度 |
| 定圈數盤旋 | NAV_LOITER_TURNS | 18 | 圈數 | 0 | 半徑 m（Plane 用 param3；正/負=方向） | xtrack | lat, lon, alt |
| 改變速度 | DO_CHANGE_SPEED | 178 | 0=airspeed, 1=groundspeed | 速度 m/s（-1 不變） | 油門%（-1 不變） | 0 | |
| 相機等距觸發 | DO_SET_CAM_TRIGG_DIST | 206 | 距離 m（0=停止） | 快門積分 ms（0） | 立即觸發 1/0 | 0 | |
| 條帶起點 / 終點 | 用 DO_SET_CAM_TRIGG_DIST(dist) / DO_SET_CAM_TRIGG_DIST(0) 包住作業路徑；LiDAR 可用 DO_SET_SERVO / DO_DIGICAM_CONTROL 或 AUX 功能（待驗證，依酬載） |
| 降落序列起點 | DO_LAND_START | 189 | 0 | 0 | 0 | 0 | lat, lon（可 0） |
| 垂直降落 | NAV_VTOL_LAND | 85 | 0（ArduPilot 4.5+ param1 可設 approach 型式，待驗證） | 0 | approach alt（待驗證） | yaw | lat, lon, alt(0) |
| 回家 | NAV_RETURN_TO_LAUNCH | 20 | | | | | |
| 跳轉 | DO_JUMP | 177 | 目標 seq | 重複次數 | | | |

Frame：`MAV_FRAME_GLOBAL_RELATIVE_ALT`=3（相對 home 高度，MSL_home + alt），`MAV_FRAME_GLOBAL_TERRAIN_ALT`=10（相對地形，需 TERRAIN_ENABLE=1 且有地形資料），`MAV_FRAME_GLOBAL`=0（MSL）。
HV-DCCPP 自己算了坡度受限高度包絡（MSL），建議輸出 **MAV_FRAME_GLOBAL_RELATIVE_ALT（alt = z_msl − home_msl）**，並提供 TERRAIN_ALT 選項。
每個 NAV_WAYPOINT 對 QuadPlane 的固定翼段是「L1 導航追蹤」，轉彎會提前切入；Dubins 弧段離散點（論文 Eq.28/29）作為航點時，航點間距應 ≥ WP_RADIUS 且避免過密（建議弧段每 20–30° 一點，直線段只留端點）。

QGC WPL 110 文字格式（Mission Planner 可直接開啟 `.waypoints` / `.txt`）：
```
QGC WPL 110
<seq>\t<current>\t<frame>\t<command>\t<p1>\t<p2>\t<p3>\t<p4>\t<lat>\t<lon>\t<alt>\t<autocontinue>
```
第 0 行為 home（frame 0, command 16, current 1）。pymavlink `mavwp.MAVWPLoader().load(path)` 可讀回驗證。

## 2. Rally（備降點）與 Fence（空域）

- Rally：MAV_MISSION_TYPE_RALLY=2，項目 MAV_CMD_NAV_RALLY_POINT=5100，x/y=lat/lon，z=alt（相對 home），frame=3。QuadPlane 到 rally 點後可用 Q_RTL_MODE / rally 點降落行為（ArduPlane 4.x：RTL 到最近 rally 點，若 Q_RTL_MODE=1 則在 rally 點 VTOL 降落；待驗證）。
- Fence：MAV_MISSION_TYPE_FENCE=1；多邊形 inclusion 頂點 MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001（param1=頂點數），exclusion 5002；圓形 inclusion 5003 / exclusion 5004（param1=半徑）；MAV_CMD_NAV_FENCE_RETURN_POINT=5000。FENCE_ENABLE=1，FENCE_TYPE bitmask（1=max alt, 2=circle, 4=polygon, 8=min alt），FENCE_ALT_MAX（m）、FENCE_ACTION（1=RTL, 6=guided, QuadPlane 可 QLAND...）。
- 任務區域 geofence：inclusion polygon = 任務區域外擴 buffer；禁區 = exclusion polygons；高度上限 = FENCE_ALT_MAX。

## 3. 平台參數 → ArduPilot 參數對照（每個鍵都要在文件表格中出現）

| Table 1 鍵 | ArduPilot 參數（ArduPlane 4.5+ 名稱；括號為舊名） | 換算 |
|---|---|---|
| cruise_airspeed_mps | AIRSPEED_CRUISE（舊 TRIM_ARSPD_CM cm/s） | m/s |
| min_mission_airspeed_mps | AIRSPEED_MIN（舊 ARSPD_FBW_MIN） | m/s |
| max_airspeed_mps | AIRSPEED_MAX（舊 ARSPD_FBW_MAX） | m/s |
| stall_airspeed_mps | AIRSPEED_STALL（4.5+）；Q_ASSIST_SPEED 應 > stall | m/s |
| transition_airspeed_mps | Q_TRANSITION 完成條件 ≈ AIRSPEED_MIN 或 Q_ASSIST_SPEED；Q_TRANSITION_MS 為逾時 | |
| planning_climb_angle_deg / max_fw_climb_rate_mps | TECS_CLMB_MAX（m/s）=V·sin(γ)；TECS_PITCH_MAX | 1.13 / 1.00 m/s |
| planning_descent_angle_deg / max_fw_descent_rate_mps | TECS_SINK_MAX（m/s）；TECS_SINK_MIN | 1.50 / 1.33 m/s |
| max_bank_angle_deg | ROLL_LIMIT_DEG（舊 LIM_ROLL_CD centideg） | 20 / 25 |
| turn_radius_planning_m | WP_LOITER_RAD（盤旋半徑，m）≥ 該值；NAVL1_PERIOD 影響切入；RTL_RADIUS | 200 / 120 |
| transition_start_agl_m | NAV_VTOL_TAKEOFF 的 alt（起飛完成高度） | 60 / 50 |
| transition_complete_agl_m | 起飛後第一個 NAV_WAYPOINT 高度 ≥ 此值 | 80–100 / 70–90 |
| min_fw_operation_agl_m | 規劃器最低作業 AGL；FENCE_ALT_MIN 可設 | 120 / 100 |
| vtol_site_clear_radius_m / obstacle_horizontal_buffer_m | 規劃器起降點檢查（ArduPilot 無對應） | |
| reserve_fraction / 可用電量 Wh | BATT_CAPACITY（mAh）、BATT_LOW_MAH / BATT_CRT_MAH（=保留量）、BATT_FS_LOW_ACT / BATT_FS_CRT_ACT（QuadPlane 建議 RTL / QLAND） | 30% |
| battery_monitor_validated=false | 規劃器加保守係數；文件註明未校正 BATT_AMP_PERVLT 前不得依賴 | |
| airspeed_sensor_validated=false | ARSPD_USE=0 時 Plane 用合成空速；規劃器不得依賴自動轉換裕度 → 轉換高度加裕度 | |
| c2_link_range_km | FS_LONG_ACTN / FS_SHORT_ACTN（失鏈行為）、FS_LONG_TIMEOUT；規劃器硬限制 | |
| terrain_clearance_m | TERRAIN_FOLLOW=1 + 航點 frame=10 時 alt=淨空；或規劃器包絡 | |
| alternate_landing_sites_required | Rally 點 + Q_RTL_MODE | |
| wind | 規劃器輸入；ArduPilot 有 EKF 風估計（WIND 訊息）可回饋 | |

## 4. MAVLink 連線與上傳流程（pymavlink）

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')   # 或 'tcp:127.0.0.1:5760', 'com5:57600'
m.wait_heartbeat()
# 上傳任務：MISSION_COUNT(type) → 對方回 MISSION_REQUEST_INT(seq) → 回 MISSION_ITEM_INT(seq) … → MISSION_ACK
# rally / fence 同樣流程但 mission_type=2 / 1
# 遙測：GLOBAL_POSITION_INT(lat/lon 1e7, alt mm MSL, relative_alt mm), SYS_STATUS/BATTERY_STATUS(電量%), WIND(方向/速度), EXTENDED_SYS_STATE(vtol_state, landed_state), MISSION_CURRENT(seq), HEARTBEAT(custom_mode)
# 動態重規劃：以 MISSION_SET_CURRENT / 重新上傳任務（Plane 支援飛行中重新上傳；建議在作業路徑端點切換）或 GUIDED 模式 SET_POSITION_TARGET_GLOBAL_INT
```
建議實作：`hv_dccpp/ardupilot.py`（純輸出：Plan → 任務項目列表、.waypoints 檔、rally、fence、參數建議檔 .param）與 `hv_dccpp/mavlink_link.py`（可選依賴 pymavlink：上傳、遙測→PlanState、事件偵測）。另寫 `tools/mock_autopilot.py`：用 pymavlink 模擬一台回應任務協定的載具，供無 SITL 時的整合測試；有 SITL 時同一測試直接對 SITL 跑。

## 5. SITL

- 本機 Mission Planner 的 sitl 目錄只有 ArduCopter.exe；ArduPlane.exe 需另行下載（firmware.ardupilot.org/Tools/MissionPlanner/sitl/）。或在 WSL 安裝 ardupilot 原始碼建置。兩者皆需使用者同意下載。
- QuadPlane SITL：`sim_vehicle.py -v ArduPlane -f quadplane --console --map`，或 `arduplane -M quadplane --defaults quadplane.parm`；預設 home 為 CMAC（-35.363, 149.165）；可用 `-L` 或 `--home lat,lon,alt,yaw` 指定；可注入風：`SIM_WIND_SPD`, `SIM_WIND_DIR`, `SIM_WIND_TURB`；地形：`TERRAIN_ENABLE`（需地形 tile，離線可關）。
- 驗證項目：任務上傳 ACK=0；自動起飛 → 轉換 → 沿條帶 → 盤旋爬升 → VTOL 降落全程無 fence breach；比較 SITL 實際軌跡 vs 規劃軌跡（橫向誤差、AGL、時間、能耗估計 vs SIM 電池）。
