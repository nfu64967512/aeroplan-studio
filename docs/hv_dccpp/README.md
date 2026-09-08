# HV-DCCPP — 異構 VTOL 動態完全覆蓋路徑規劃

**H**eterogeneous-**V**TOL **D**ynamic **C**omplete **C**overage **P**ath **P**lanning：
以 Wang et al. 2025（*Defence Technology* 47:197–215）的 DCCPP 框架為基礎，擴充為
**兩種構型混編**（A = Pioneer PNP、B = HERO PNP，皆為 lift-and-cruise 4+1 VTOL）的
多機區域覆蓋任務規劃器，輸出可直接上傳 ArduPilot QuadPlane 的任務檔。

## 1. 這是什麼

給定「任務區域多邊形 + 機隊 + DEM + 風 + 空域限制」，本套件產生每架 UAV 的**完整 4-D 航線**
（x, y, z, t 與模式／階段／酬載開關／空速／剩餘能量），並自行處理：

| 能力 | 內容 |
|---|---|
| 覆蓋分解 | 多邊形簡化 → 凸分割 → 合併 → 掃描方向評分（風、地形坡度、迴轉帶、禁飛）→ 牛耕條帶 |
| 異構配置 | 每個胞選構型（LiDAR/RGB 感測需求、續航、鏈路半徑）、機數 λ、殘帶再分割（A 掃前 k 條、B 補剩餘） |
| 序列決策 | 事件驅動 Beam-IDP（k-nearest + 波束）決定條帶順序與掃描方向；`legacy_mode` 可退化為論文的鎖步 IDP |
| 高度剖面 | 坡度受限包絡（climb/descent 角、風中地速）、地形淨空 h_clr、最低固定翼高度、感測器 GSD 上限、空域上限、投影平滑 |
| 路徑幾何 | Dubins（6 word，含風中時間積分）、keep-out 切線圖／可視圖繞行（AVOID）、起降走廊與爬升／下降盤旋 |
| 能量 | 表 1 參數 → 巡航／懸停／轉換功率、爬降修正、`EnergyBudgetTable`（任一點的返航能量下界）、返航與能量修復 |
| 安全 | PRECHECK（輸入／氣象／站點／機體／區域／時間窗七段檢查）、高度分層、4D 分離（σ_t 成長）、起降走廊排程、失鏈幽靈航跡、氣象中止 |
| 動態 | UAV_EXIT／NEW_AREA／ENERGY_UPDATE／WIND_UPDATE／TARGET_CUE／LINK_LOSS／PRIORITY_CHANGE／WEATHER_ABORT 增量重規劃（未受影響機的計畫物件保持不變） |
| 輸出 | QGC WPL 110 `.waypoints`、`.plan`、rally、fence、`.param`；MAVLink 上傳／遙測／重上傳；MQTT 橋接；既有 Tkinter 系統的替換層 |

平台參數表（表 1）與任務輸入（表 2）以使用者提供的數值為準，未驗證的參數在 `PlatformSpec.provisional`
中列出，PRECHECK 會以 `provisional_params` 警告提醒。

## 2. 安裝

```bash
# Python >= 3.11（開發環境為 3.13）
pip install -e .                       # numpy / scipy / shapely>=2 / matplotlib
pip install -e ".[mavlink,mqtt,test]"  # 可選：pymavlink（MAVLink）、paho-mqtt（橋接）、pytest
```

不需要網路；DEM 可用外部資料或內建的合成地形（`hv_dccpp.scenario.synthetic_dem`）。
`hv_dccpp.viz` 會把 matplotlib 後端設為 `Agg`，故套件不會在 `import hv_dccpp` 時載入它
（既有以 TkAgg 繪圖的系統可安心 import）。

## 3. 快速上手（JSON → 規劃 → 驗證 → ArduPilot 檔案）

`examples/mission_min.json` 是一份最小任務輸入（平地 DEM、1 架 Pioneer + 1 架 HERO、3 km × 1.2 km 區域）。

```python
import dataclasses
import hv_dccpp as hv

inputs = hv.load_mission("examples/mission_min.json")           # 表 2 任務輸入（JSON / dict / 路徑）
cfg = dataclasses.replace(hv.PlannerConfig(), dem_step_m=50.0)  # 地形網格 50 m（平地夠用；預設 10 m）
ctx = hv.build_context(inputs, cfg)                             # ENU 框架／地形／風場／能量模型／感測器

rep = hv.preflight(inputs, cfg, ctx)                            # 起飛前檢查
print("preflight:", rep.status, "errors:", rep.errors[:3], "warnings:", len(rep.warnings))
if not rep.ok:
    raise SystemExit(1)

plan = hv.plan_mission(inputs, cfg, ctx=ctx)                    # 分解 → 配置 → Beam-IDP → 高度剖面 → 去衝突
print(f"makespan = {plan.makespan_s / 60:.1f} min, cells={len(plan.cells)}, strips={len(plan.strips)}")
for uid, p in plan.plans.items():
    print(f"  {uid}({p.cls}) 條帶 {len(p.strips)} 距離 {p.distance_m / 1000:.1f} km "
          f"落地 {p.t_land_s:.0f} s 能量餘裕 {p.energy_margin_wh:.0f} Wh")

cache = hv.build_validation_cache(plan, ctx, budget_only=True)  # 驗證用能量預算表
vrep = hv.validate(plan, ctx, cache, cfg)                       # 覆蓋／能量／坡度／AGL／空域／4D 分離
print("validate:", vrep.passed, "coverage:", {c.area_id: round(c.coverage, 3) for c in vrep.coverage},
      "conflicts:", len(vrep.conflicts))

summary = hv.export_all(plan, ctx, cfg, "out/ardupilot")        # QGC WPL + rally + fence + .param + .plan
for uid, s in summary["uavs"].items():
    print(f"  {uid}: {s['n_items']} 個任務項目 → {s['files']['waypoints']}")
```

實測輸出（`python examples/quickstart.py`，約 3 s）：

```
preflight: GO_WITH_WARNINGS errors: [] warnings: 15
makespan = 28.9 min, cells=3, strips=8
  A1(A) 條帶 5 距離 32.1 km 落地 1736 s 能量餘裕 518 Wh
  B1(B) 條帶 3 距離 23.0 km 落地 1389 s 能量餘裕 138 Wh
validate: True coverage: {'Z1': 1.0} conflicts: 0
  A1: 98 個任務項目 → out/ardupilot/A1.waypoints
  B1: 54 個任務項目 → out/ardupilot/B1.waypoints
```

任務 JSON 必填鍵：`home`、`areas`、`dem`、`fleet`、`weather_obs`；另需 `c2`（鏈路半徑，D11）與
`airspace.terrain_clearance_m`（地形淨空），否則 PRECHECK 回 NO_GO 並列出缺少的鍵名。
`fleet[i].spec` 可寫 `"A"`／`"B"`（或 `"PIONEER"`／`"HERO"`）；`areas` 也接受 GeoJSON。
完整欄位見 `hv_dccpp/mission.py` 的 `load_mission` 說明與 `examples/mission_min.json`。

動態事件重規劃：

```python
cache = hv.cache_from_plan(plan, ctx)                  # 由既有計畫重建成本快取
tel = {"A1": hv.Telemetry(uav_id="A1", t_s=600.0, x_m=6845.0, y_m=-200.0, z_m=160.0,
                          theta_rad=1.57, v_air_mps=21.5, e_tel_wh=400.0,
                          current_strip_id="Z1_1_A_5", strip_progress=0.4, link_ok=True)}
res = hv.replan(plan, hv.EnergyUpdate(uav_id="A1", e_tel_wh=400.0, t_s=600.0), tel, ctx, cache, cfg, [])
print(res.changed_uavs, res.notes, res.plan.makespan_s)
```

## 4. 模組地圖

第 0–1 層可單獨使用（純 numpy/shapely）；上層依序疊加。括號為契約章節。

| 層 | 模組 | 職責 |
|---|---|---|
| 0 | `geo.py` (§1) | WGS84 ↔ ECEF ↔ ENU、DEM → `TerrainGrid`（ENU z、含曲率）、走廊 max/min、障礙燒入、`raw_view()` 自然地形 |
| 0 | `platform.py` (§2) | 表 1 `PlatformSpec`（A/B）、感測器（硬體／需求分離）、`EnergyModel`、轉換與起降時間 |
| 0 | `dubins.py` (§5) | Dubins 6 word、向量化批次求解、離散化、風中時間／能量積分 |
| 0 | `geometry.py` (§6) | 多邊形簡化／凸分割／合併、掃描座標、條帶鋪設、迴轉帶 |
| 1 | `mission.py` (§3) | 表 2 `MissionInputs`、`PlannerConfig`、`Waypoint`/`UAVPlan`/`MissionPlan`、`load_mission` |
| 1 | `wind.py` (§4) | 分層風場、風三角、`R_eff`、地面坡度、圓周時間因子 |
| 2 | `context.py` (§6A) | `build_context`：ENU 投影、地形重取樣與燒入、風場、能量模型、感測器、鏈路多邊形、空域上限 |
| 2 | `routing.py` (§6B) | keep-out 集合（2.5D）、切線圖／可視圖、`avoid()`（AVOID 路徑） |
| 2 | `scenario.py` (§16) | 合成 DEM、mini／hills／論文 Table 2 三個內建情境 |
| 3 | `altitude.py` (§11) | 坡度受限包絡、局部包絡（給條帶的保守 AGL）、`build_3d`、起降走廊、盤旋、站點檢查、分層 |
| 3 | `decompose.py` (§7) | 高度帶、方向候選與代理時間、條帶鋪設、精確時間、適配度、殘帶候選 |
| 3 | `costs.py` (§8) | op／entry／transfer／rtb 成本矩陣、`EnergyBudgetTable`、`CostCache`（含風／狀態刷新） |
| 3 | `preflight.py` (§3A) | PRECHECK：輸入完整性、站點、能量、鏈路、空域、時間窗、每區可覆蓋性 |
| 3 | `allocate.py` (§9) | 構型選擇與機數、拍賣式配置、殘帶對、起飛排程、多架次 |
| 3 | `deconflict.py` (§13) | 4D 分離、空間互斥、起降走廊排程、修復（起飛延遲／換序／延遲盤旋） |
| 4 | `planner.py` (§10) | Beam-IDP、返航附加、能量修復、2D 組裝 → `build_3d`、`plan_mission` 總流程 |
| 4 | `dynamic.py` (§12) | 8 類事件的增量重規劃、幽靈航跡、佔位計畫 |
| 4 | `sim.py` (§14) | 覆蓋柵格、運動學／能量／鏈路空域驗證、追蹤模擬、基線比較、論文對照 |
| 4 | `viz.py` (§15) | 2D 平面圖、剖面、甘特、覆蓋、能量圖（Agg 後端） |
| 5 | `ardupilot.py` | `UAVPlan` → MAVLink 任務項目、QGC WPL 110、rally／fence／建議參數／`.plan` |
| 5 | `mavlink_link.py` | `VehicleLink`／`FleetLink`：上傳、遙測 → `dynamic.Telemetry`、事件偵測、差異重上傳 |
| 5 | `integration/legacy_adapter.py` | 既有 Tkinter 系統的 `waypoint_generator` 替換件（相同簽章） |
| 5 | `integration/mqtt_bridge.py` | MQTT 請求／回應橋接（含舊格式相容） |

契約無法照做之處一律記錄於 `hv_dccpp/CONTRACT_DEVIATIONS.md`（含本次整合的 AGL 基準與 4D 走廊例外）。

## 5. 接到既有系統（`integration/legacy_adapter.py`）

既有 `main_app.py` 呼叫的是 `self.waypoint_generator.generate_complete_mission(...)`，
把該物件換成 `HVDCCPPGeneratorAdapter` 即可（簽章完全相同，內部改走 HV-DCCPP）：

```python
from hv_dccpp import HVDCCPPGeneratorAdapter          # 延遲載入，不需 pymavlink 也能 import

self.waypoint_generator = HVDCCPPGeneratorAdapter(
    platform="hero_pnp",              # 或 "pioneer_pnp"
    home_latlon=(23.7027, 120.4193),  # None → 取第一次呼叫的第一個角點
    wind=(5.0, 270.0),                # (風速 m/s, 風來向 deg)
    c2_range_km=25.0, terrain_clearance_m=36.0, waiver_max_agl_m=200.0,
    sensor_kind="rgb",                # 攝影測量；"lidar" 僅 Pioneer
    angle_mode="force",               # params.angle 當唯一掃描方向（不可行時自動退回 hint 並記警告）
    spacing_mode="overlap_hint",      # params.spacing → 側向重疊率提示
)

# 既有呼叫完全不用改：
lines, latlon_path = self.waypoint_generator.generate_complete_mission(
    region_corners, params, idx, sub_count, start_from_left, loiter_time)
```

* `params` 以 duck typing 讀 `altitude / angle / spacing / speed / safety_distance`（缺者忽略）。
* 回傳 `(qgc_lines, latlon_path)`：QGC WPL 110 文字列 + NAV 項目的 `(lat, lon)` 序列（起飛點起、落點止）。
* 其他相容方法：`plan_multi(corners, drones)`（異構多機）、`waypoints_to_qgc_format(...)`、
  `subdivide_polygon(...)`、`subdivide_rectangle(...)`、`collision_avoidance.calculate_loiter_delay(...)`。
* 呼叫後可取 `adapter.last_plan`（`MissionPlan`）、`adapter.last_inputs`、`adapter.last_items[uav_id]`、
  `adapter.warnings`，直接餵給 `sim.validate` / `viz.save_all` / `mavlink_link`。
* 與既有輸出的差異：VTOL 起降改用 `NAV_VTOL_TAKEOFF(84)` / `NAV_VTOL_LAND(85)`，不再輸出
  `CONDITION_YAW(115)`，也不會在任務中段插入 `NAV_TAKEOFF(22)`；高度一律 `MAV_FRAME_GLOBAL_RELATIVE_ALT`。
* 實測：五角形區域（HERO、120 m、間距提示 50 m → 28 條條帶）單機約 0.6 s，990 個航點縮減為 364 個 NAV 列。

## 6. MQTT 橋接（`integration/mqtt_bridge.py`）

```python
from hv_dccpp import MqttBridge                       # 需要 paho-mqtt（v1 / v2 皆可）
bridge = MqttBridge(host="127.0.0.1", port=1883, async_handling=True)  # 每則請求獨立執行緒規劃
bridge.connect()
bridge.loop_forever()                                  # 或 bridge.loop_start() / loop_stop() 走背景 loop
```

| 主題 | 方向 | 內容 |
|---|---|---|
| `hvdccpp/mission/request` | 收 | 任務請求（下方 schema）；舊系統相容主題 `drone/waypoints` |
| `hvdccpp/mission/<mission_id>/<drone_id>` | 送 | 每架一則：`qgc_wpl`、`waypoints`、`summary`、`rally`、`geofence` |
| `hvdccpp/status` | 送 | `online` / `planning` / `planned` / `error` 事件（含 `assumptions`、`timing_ms`） |

請求（`?` 可省略，省略者由 `BridgeDefaults` 補值並列於 `summary.assumptions`）：

```json
{
  "mission_id": "m001",
  "home": {"lat": 23.7027, "lon": 120.4193, "alt_msl": 100},
  "alternates": [{"id": "ALT1", "lat": 23.70, "lon": 120.41}],
  "areas": [{"area_id": "Z1", "polygon": [{"lat": 23.70, "lon": 120.42}, "…≥3 點"],
             "sensor_req": "rgb", "priority": 1.0, "time_window_s": [0, 7200]}],
  "drones": [{"id": "A1", "platform": "pioneer_pnp", "auw_kg": 24.6, "available_wh": 1350,
              "nominal_wh": 1500, "sensors": ["lidar"], "lost_link_policy": "CONTINUE_THEN_RTB"},
             {"id": "B1", "platform": "hero_pnp", "soc": 0.9, "sensors": ["rgb"]}],
  "wind": {"speed_mps": 5, "dir_from_deg": 270, "gust_mps": 7},
  "c2_range_km": 25, "terrain_clearance_m": 36,
  "airspace": {"max_agl_m": 120, "waiver_max_agl_m": 200,
               "no_fly": [{"zone_id": "NF1", "polygon": [{"lat": 23.7, "lon": 120.5}]}]},
  "obstacles": [{"obs_id": "T1", "geom_type": "point", "coords": [{"lat": 23.71, "lon": 120.43}],
                 "height_agl_m": 80, "category": "tower"}],
  "dem": {"flat_msl_m": 100, "step_m": 50},
  "planner": {"dem_step_m": 50}
}
```

回應的 `qgc_wpl` 可由 Mission Planner／QGC 直接開啟；`summary.status` 為
`GO` / `GO_WITH_WARNINGS` / `NO_GO` / `NOT_USED` / `ERROR`（未獲派任務的機也會收到一則）。
離線測試不需 broker：

```python
from hv_dccpp import handle_request_full
res = handle_request_full(payload, raise_errors=True)   # res.payloads / res.plan / res.ctx / res.assumptions
```

舊格式（`{"waypoints": [{"lat","lon"} × 4]}`、`{"polygon": [...]}`、分段訊息、單點累積）會自動轉成
單一 `sensor_req="any"` 區域，機隊採建構子的 `default_drones`。

## 7. Mock autopilot 與 SITL

**Mock（無需 ArduPilot，本專案用它做閉迴路驗證）**

```bash
# 終端 1：模擬一架 QuadPlane（UDP 監聽、時間加速 60 倍、風 5 m/s from 270）
python tools/mock_autopilot.py --listen udpin:127.0.0.1:14601 --home 23.95,120.90,100,0 \
       --speed 21.5 --time-scale 60 --wind 5,270 --sysid 1

# 終端 2：規劃 → 上傳 → 起飛 → 注入 ENERGY_UPDATE → 重規劃 → 重上傳 → 等待落地
python examples/run_mock_autopilot_loop.py --time-scale 60 --ports 14601,14602
```

`run_mock_autopilot_loop.py` 會自行啟動兩個 mock 子程序，實測約 50 s 牆鐘時間跑完 mini 情境，
輸出 `figures/mock_loop_trace.png` 與 `figures/ardupilot/mock/{before,after}/*`。

程式介面（`mavlink_link`）：

```python
from hv_dccpp import VehicleLink, FleetLink
link = VehicleLink("udpout:127.0.0.1:14601", uav_id="A1", time_source="boot")
link.wait_heartbeat(timeout_s=10)
fleet = FleetLink({"A1": link})
reports = fleet.upload_plan(plan, ctx, cfg, out_dir="out/ardupilot", set_params=True)  # 任務+rally+fence+參數
link.set_mode("AUTO"); link.arm(); link.start_mission(1)
tel, events = fleet.poll(plan, ctx, cfg, timeout_s=1.0)      # → {uid: dynamic.Telemetry}, [事件]
# ...dynamic.replan 之後：
fleet.replan_and_reupload("A1", res.plan.plans["A1"], ctx, cfg, out_dir="out/ardupilot")
```

**ArduPilot SITL（未在本機實測，指令供參）**

```bash
sim_vehicle.py -v ArduPlane -f quadplane --console --map --out=udp:127.0.0.1:14601
```

然後把上面的連線字串改成 `VehicleLink("udpin:127.0.0.1:14601")`（SITL 主動送、我方監聽）。
建議先套用 `export_all` 產生的 `<uav>.param`（`Q_ENABLE`、`TERRAIN_ENABLE`、`WP_RADIUS`、
`TECS_*`、`Q_TRANS_*` 等），fence／rally 亦可由 `<uav>_fence.txt`／`<uav>_rally.txt` 載入。
上傳的高度框架預設為 `MAV_FRAME_GLOBAL_RELATIVE_ALT`（`alt_frame="terrain"` 則用 AGL + `TERRAIN_ENABLE=1`）。

## 8. 測試

```bash
python -m pytest tests -q                      # 695 個測試，約 75 s（全部通過）
python -m pytest tests/test_planner.py -q      # 單一模組
python -m pytest tests -q --timeout=300        # 需要 pytest-timeout
```

範例腳本（`figures/` 為輸出目錄）：

| 腳本 | 內容 | 實測時間 |
|---|---|---|
| `examples/quickstart.py` | 本 README 的快速上手（JSON → 規劃 → 驗證 → 匯出） | 3 s |
| `examples/run_mini.py` | §W mini 情境全流程 + 追蹤模擬 + 圖 + ArduPilot 匯出 | 3.5 s |
| `examples/run_hills.py` | 8 機／3 區／障礙／禁飛／分層風 + 基線比較（hetero / all_A / all_B / legacy） | 43 s |
| `examples/run_paper_table2.py` | 論文 Table 2 同構退化對照（`legacy_mode`；含覆蓋柵格繪圖 ~40 s） | 65 s |
| `examples/run_dynamic_events.py` | 5 類事件的孤立與連鎖重規劃 | 9 s |
| `examples/run_mock_autopilot_loop.py` | mock autopilot 閉迴路（上傳／飛行／事件／重上傳；`--time-scale 60`） | 50 s |

## 9. 已知限制

* **殘餘 4D 衝突**：山丘情境（8 機共用一個起降站、makespan 5552 s）修復後仍有 4 筆 4D 衝突（其餘驗證旗標全數通過、三區覆蓋率皆 1.000）。σ_t = k_t·(t − t_sync)
  在兩小時任務末端成長到 ±250 s，任何「同點但相隔數分鐘」的交會都會被判為衝突；解法是規格 §13.6 步 (iii)
  的 IDP 重排（`deconflict.repair(resequence_hook=…)` 介面已備妥，本套件未實作）。單機時間平移已被
  修復器驗證為「補此失彼」（不會採用）。實務上可用即時遙測推進 `t_sync` 讓 σ_t 歸零。
* **論文 Table 2 對照**：本套件單一 home（三機同站起飛）且路徑含起降走廊，per-UAV 路徑長 680/689/840 km，
  高於論文的 590–650 km；作業段（oper）為 393–518 km，與論文同量級。
* **`all_B` 基線**：HERO 無 LiDAR，含 LiDAR 需求的區域無法覆蓋（回報 1 個未覆蓋區）。
* **異構不一定比同構快**：山丘情境 hetero 5552 s vs all_A 3924 s（慢 29 %）——makespan 由最慢的 B 機決定，
  而 3 架 Pioneer 已能獨力覆蓋全部三區。異構的價值在於「B 機便宜、可同時提供 RGB 與較低作業高度」而非單純縮短工期；
  若目標是 makespan，應調整 λ 上限或讓 B 機多架次。
* **臨時參數**：表 1 中 `provisional` / `provisional_supplementary` 的欄位（C2 半徑、轉換高度、
  電池標稱容量、站點風限等）為工程估計；正式任務前應以實測值覆寫（`platform_overrides`）。
* **酬載觸發線路**：LiDAR 以 `DO_SET_SERVO`／`DO_SET_RELAY`、相機以 `DO_SET_CAM_TRIGG_DIST`，
  實際接線與參數需在機上驗證。

---

規格與契約文件：`docs/reference/HV-DCCPP_規格書_v1.1.md`、`docs/reference/模組契約_v1.1.md`、
`docs/HV-DCCPP_設計文件.md`、`docs/平台參數表.md`、`docs/進度紀錄.md`。
