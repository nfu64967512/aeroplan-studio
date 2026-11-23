# HARVEST 設計文件（實作／資料流／即時性優先版）

Heterogeneous-VTOL Dynamic Complete Coverage Path Planning：Pioneer PNP（構型 A）+ HERO PNP（構型 B）兩構型 lift-and-cruise 4+1 VTOL 群飛偵查路徑演算法。基礎：Wang et al. 2025, *Defence Technology* 47:197–215（以下簡稱「論文」）。本文件所有數值以 `platform_params.md` 表 1／表 2 為權威來源；「暫定／待填入」項目一律做成可設定參數並標記 `provisional`。

---

## 1 問題定義與假設

### 1.1 問題敘述

給定：
- 一支異構機隊 $U=\{U_i\}_{i=1}^{n_U}$，每架屬於構型 $c(i)\in\{A,B\}$，各有本架次 AUW、可用電量、感測器。
- 任務區域集合 $Z=\{Z_k\}_{k=1}^{n_Z}$，每區為（可能凹、可能含洞）多邊形，附感測需求 $\sigma_k\in\{\text{lidar},\text{rgb},\text{both},\text{any}\}$ 與優先度 $\pi_k$。
- 起飛點 $H$、備降點集合 $\mathcal{A}$、DEM/DSM、障礙物、分層風場、C2 鏈路範圍、空域限制、氣象門檻（表 2 全部）。

求：每架 UAV 的三維航點序列（含模式旗標、時間戳、剩餘能量），使得
$$\min\ T_{\max}=\max_i T_i^{\text{land}},$$
$T_i^{\text{land}}$ 為 $U_i$ 落地（含返航 RTB）時刻，並滿足：(C1) 每個需覆蓋的條帶恰被一架符合感測需求的 UAV 掃過一次；(C2) 每航點剩餘能量足以抵達最近可用備降點且保留 `reserve_fraction`；(C3) 所有航點位於 C2 鏈路範圍與空域限制內；(C4) 航跡坡度 $\in[-\tan\gamma_d,\tan\gamma_c]$、轉彎半徑 $\ge R_g$（含風放大）、AGL $\ge$ 淨空；(C5) 任兩機 4D 分離；(C6) 起降走廊互斥。

### 1.2 假設（沿用論文並修訂）

| # | 論文假設 | 本設計 |
|---|---|---|
| A1 | 同構 | 兩構型；參數相同時退化為論文（§13） |
| A2 | 從區外初始位置出發、定速 | 從起飛點垂直起飛，進入路徑起點 = 轉換完成點；空速定值（`cruise_airspeed_mps`），地速由風三角決定 |
| A3 | FOV 梯形固定 | 感測器模型（LiDAR 扇形／相機矩形）；論文梯形 FOV 保留為選項 |
| A4 | 升降時地速不變 | 升降時空速不變；能量另計爬升功率 |
| A5 | 一機只覆蓋一區 | 一機可依序覆蓋多個凸胞（cell）；IDP 在「團隊條帶集合」上執行 |
| A6 | 無風、無能量 | 分層風場；能量硬限制；RTB 必含 |
| A7 | 懸停不可用 | 懸停僅用於垂直起降、轉換、緊急；任務轉彎一律 Dubins（`turn_radius_planning_m`） |

### 1.3 座標與單位

幾何一律在本地 ENU（East-North-Up，公尺）平面計算，原點為起飛點 $H$；輸入 WGS84 → ENU、輸出 ENU → WGS84（§4.3）。時間單位 s、能量 Wh、角度內部用 rad（介面用 deg）。

---

## 2 符號表

| 符號 | 意義 | 單位 |
|---|---|---|
| $c(i)$ | $U_i$ 構型 ∈{A,B} | — |
| $V_a^{c}$ | 巡航空速 `cruise_airspeed_mps` | m/s |
| $V_g(\hat t,z)$ | 沿航向 $\hat t$、高度 $z$ 的地速 | m/s |
| $\mathbf{w}(z)=(W,\theta_w)$ | 高度 $z$ 的風速／風向（吹來方向） | m/s, deg |
| $R_p^{c}$ | `turn_radius_planning_m` | m |
| $R_g^{c}$ | 風中規劃地面轉彎半徑 $=R_p(1+W/V_a)^2$ | m |
| $\gamma_c,\gamma_d$ | `planning_climb_angle_deg`, `planning_descent_angle_deg` (3°, 4°) | deg |
| $h_{\min}^{c}$ | `min_fw_operation_agl_m` | m AGL |
| $h_{\text{op}}$ | 胞／構型的目標作業 AGL | m |
| $h_{ts}, h_{tc}$ | `transition_start_agl_m`, `transition_complete_agl_m` | m AGL |
| $w^{c}(h)$ | 感測器地面覆蓋寬（swath） | m |
| $s^{c}(h)$ | 條帶間距 $=w(1-o_{\text{side}})$ | m |
| $V_{\max}^{\text{sens}}(h)$ | 感測器允許的地速上限 | m/s |
| $Z_k, C_{k,q}$ | 區域、其凸胞 | — |
| $L_{q,l}$ | 胞 $q$ 第 $l$ 條作業路徑（條帶），端點 $p^L,p^R$ | — |
| $\alpha_q$ | 胞 $q$ 掃描方向角 | rad |
| $S_i=(x,y,\psi,R,V_a,t,E)$ | UAV 狀態（位置、航向、地面轉彎半徑、空速、時刻、剩餘能量） | — |
| $\tau^{\text{op}}_{i,l}=[\tau^{LR},\tau^{RL}]$ | 作業路徑二元時間 | s |
| $\tau^{\text{in}}_{i,l}=[\tau^{L},\tau^{R}]$ | 進入路徑二元時間 | s |
| $\tau^{\text{tr}}_{i,l,m}=[\tau^{L},\tau^{R}]$ | 轉移路徑二元時間 | s |
| $\mathbf{T}_{i}$ | 時間成本矩陣 $(n+1)\times n$（論文 $D_{i,k}$ 的時間版） | s |
| $\mathbf{E}_{i}$ | 決策矩陣 ∈{−1,0,1}（論文 Eq.14） | — |
| $P_{cr},P_{cl},P_{ds},P_{hv},P_{tr}$ | 巡航／爬升／下降／懸停／轉換功率 | W |
| $E_{\text{usable},i}$ | 可用能量（扣保留） | Wh |
| $\rho_{\text{cov}}$ | 覆蓋驗證柵格解析度 | m |
| $K,B$ | k-nearest 剪枝數、beam 寬 | — |

---

## 3 平台／感測器／能量模型

### 3.1 PlatformSpec（完整鏡射表 1）

```python
Range = tuple[float, float]           # (lo, hi)，規劃時依用途取保守端

@dataclass(frozen=True, slots=True)
class PlatformSpec:
    name: str                                   # "Pioneer PNP" | "HERO PNP"
    platform_class: Literal["A", "B"]
    platform_type: str                          # "lift_and_cruise_vtol_4plus1"
    wingspan_m: float
    fuselage_length_m: float
    wing_area_m2: float
    mtow_kg: float
    reference_auw_kg: float
    reference_payload_kg: float
    payload_max_kg: float                       # HERO "1.0 以下" → 1.0
    cruise_airspeed_mps: float
    cruise_airspeed_kmh: float                  # 僅顯示；建構時檢查 == 3.6*mps
    min_mission_airspeed_mps: float | Range     # HERO (15.0,16.0)→規劃取 16.0
    transition_airspeed_mps: float
    stall_airspeed_mps: float
    max_airspeed_mps: float | None              # HERO None(待實機確認)→fallback 1.25*cruise
    max_climb_angle_deg: float
    planning_climb_angle_deg: float
    max_descent_angle_deg: float
    planning_descent_angle_deg: float
    max_fw_climb_rate_mps: float
    max_fw_descent_rate_mps: float
    max_bank_angle_deg: float
    turn_radius_theoretical_m: float
    turn_radius_planning_m: float
    vtol_site_clear_radius_m: float
    obstacle_horizontal_buffer_m: float
    transition_start_agl_m: float
    transition_complete_agl_m: float | Range    # (80,100)/(70,90)
    min_fw_operation_agl_m: float
    wind_resistance_public: int                 # Beaufort，非放行門檻
    reference_endurance_min: float
    reference_total_range_km: float
    reserve_fraction: float
    usable_range_fraction: float | Range        # (0.55,0.60)→取 0.55
    usable_distance_km: float | Range           # (133,145)/(79,86)→取 lo
    symmetric_mission_radius_km: float | Range  # (66,72)/(39,43)→取 lo
    airspeed_sensor_validated: bool
    battery_monitor_validated: bool
    c2_link_range_km: float | None              # 待填入 → 由 MissionInputs.c2_link 提供
    terrain_clearance_m: float | None           # 待任務定義 → 由 MissionInputs.airspace 提供
    alternate_landing_sites_required: bool
    # --- 表 1 之外、能量與走廊模型必要的補充參數（皆 provisional） ---
    battery_nominal_wh: float | None = None     # 估算 P_cruise_ref 用；None→以「基準分鐘」為能量單位
    hover_power_ratio: float = 3.0              # P_hover/P_cruise_ref，lift-and-cruise 典型 2.5–4
    transition_duration_s: float = 15.0
    transition_ground_run_m: float = 150.0      # 轉換段水平距離
    vtol_climb_rate_mps: float = 2.0            # 垂直起降速率
    vtol_descent_rate_mps: float = 1.5
    prop_efficiency: float = 0.6                # 爬升額外功率用
    provisional: frozenset[str] = frozenset()   # 標記為暫定的鍵名集合
```

`PLATFORM_PIONEER`、`PLATFORM_HERO` 以表 1 數值建構；`provisional` 內含 `{"vtol_site_clear_radius_m","obstacle_horizontal_buffer_m","transition_start_agl_m","transition_complete_agl_m","c2_link_range_km","terrain_clearance_m","max_airspeed_mps"(HERO),"battery_nominal_wh","hover_power_ratio",...}`。

派生量（`PlatformSpec` 屬性方法）：
- `R_ground(W)` $=R_p(1+W/V_a)^2$（趨線最大曲率半徑，保守；選項 `"linear"`: $R_p(1+W/V_a)$）。
- `climb_dist(dh)` $=dh/\tan\gamma_c$；`descent_dist(dh)` $=dh/\tan\gamma_d$。
- `orbit_gain_m()` $=2\pi R_p\tan\gamma_c$（A：65.8 m/圈，B：39.5 m/圈）。
- `transition_complete_lo/hi`：爬升距離與能量用 lo（最壞）、走廊淨空用 hi。
- 一致性檢查（建構時 assert，允許 5% 誤差）：$V_a\tan 3°\approx$ `max_fw_climb_rate_mps`（21.5·0.0524=1.127 ✓；19·0.0524=0.996 ✓）；`reference_total_range_km`/`reference_endurance_min` 對應平均地速（242 km/176 min=22.9 m/s ≈ 21.5 m/s 微順風，接受）。

### 3.2 感測器模型（SensorSpec）

```python
class Sensor(Protocol):
    kind: Literal["lidar", "rgb"]
    def swath_m(self, h_agl: float) -> float
    def strip_spacing_m(self, h_agl: float) -> float
    def max_ground_speed_mps(self, h_agl: float) -> float     # inf 表示無限制
    def agl_window(self) -> tuple[float, float]                # 感測品質允許的 (h_min, h_max)
    def end_extension_m(self, v_g: float) -> float             # 條帶端點外延

@dataclass(frozen=True, slots=True)
class LidarSpec:                      # Pioneer 5 kg LiDAR
    kind = "lidar"
    fov_full_deg: float               # 例 120
    fov_usable_deg: float             # 例 70（邊緣入射角過大不計）
    point_rate_hz: float              # 例 600_000
    max_range_m: float                # 例 450
    required_density_pt_m2: float     # 例 20
    side_overlap: float = 0.20
    # swath = 2 h tan(fov_usable/2); h_max = max_range*cos(fov_usable/2)
    # V_max = point_rate / (swath * density)

@dataclass(frozen=True, slots=True)
class CameraSpec:                     # HERO 0.6 kg 測繪相機
    kind = "rgb"
    focal_length_mm: float            # 例 16
    sensor_width_mm: float            # 例 23.5
    sensor_height_mm: float           # 例 15.6
    pixel_pitch_um: float             # 例 3.9
    max_shutter_rate_hz: float        # 例 1.0
    forward_overlap: float = 0.75
    side_overlap: float = 0.65
    gsd_required_m: float = 0.05
    # swath = h*sensor_width/f ; footprint_len = h*sensor_height/f
    # GSD = h*pixel_pitch/f  → h_max = gsd_required*f/pixel_pitch
    # V_max = footprint_len*(1-forward_overlap)*max_shutter_rate

@dataclass(frozen=True, slots=True)
class TrapezoidFovSpec:               # 論文 Eq.(2)(3)，legacy 選項
    kind: Literal["lidar","rgb"]; mount_angle_deg: float; v_fov_deg: float; h_fov_deg: float
```

作業 AGL 選擇（每胞、每構型）：
$$h_{\text{op}}=\min\{h_{\max}^{\text{sens}},\ h_{\max}^{\text{airspace}}\},\qquad \text{可行}\iff h_{\text{op}}\ge\max\{h_{\min}^{c},\ h_{\min}^{\text{sens}},\ \text{clearance}+h_{\text{obs,cell}}\}$$
LiDAR 偏好高（swath 大、條帶少）受 `max_range_m`／密度限制；相機受 GSD 限制。地速上限 $V_{\text{g,max}}=\min(V_{\max}^{\text{sens}}, V_a+W)$；若順風地速 $>V_{\max}^{\text{sens}}$，作業段空速降至 $\max(V_{\text{min,mission}}, V_{\max}^{\text{sens}}-W\cos\Delta)$（僅作業段允許降速，不得低於 `min_mission_airspeed_mps`）。

### 3.3 風場模型

`WindField(layers: list[WindLayer])`，`WindLayer(z_bottom_msl, z_top_msl, speed_mps, dir_from_deg, gust_mps)`。`wind_vec(z)` 回傳吹向向量 $\mathbf w=-W(\sin\theta_w,\cos\theta_w)$（ENU，$\theta_w$ 為氣象「吹來」方位）。風三角：
$$V_g(\hat t)=\sqrt{V_a^2-(\mathbf w\cdot\hat n)^2}+\mathbf w\cdot\hat t,\quad \hat n\perp\hat t;\qquad |\mathbf w\cdot\hat n|\ge V_a\Rightarrow\text{不可行}$$
向量化：`ground_speed(track_unit: (N,2), z: (N,), v_air: float) -> (N,)`。

### 3.4 能量模型

```python
@dataclass
class EnergyModel:
    spec: PlatformSpec
    auw_kg: float                      # 本架次 AUW（表 2）
    available_wh: float                # 表 2 可用電量（mAh 輸入時 = mAh/1000*V_nom）
    nominal_wh: float | None
    correction: float = 1.0            # 線上修正係數 κ（§10）
```
- 基準巡航功率 $P_{cr}^{\text{ref}}=E_{\text{nom}}/(60\cdot\text{reference\_endurance\_min})$；若 `nominal_wh is None`，能量單位改用「基準分鐘」：可用能量 = `available_fraction × reference_endurance_min`，其餘公式不變。
- AUW 修正：$P_{cr}=\kappa\,P_{cr}^{\text{ref}}\,(m/m_{\text{ref}})^{1.5}$。
- 爬升：$P_{cl}=P_{cr}+m g V_a\sin\gamma/\eta_p$；下降：$P_{ds}=\max(0.5P_{cr},\,P_{cr}-m g V_a\sin\gamma/\eta_p)$。
- 懸停：$P_{hv}=\text{hover\_power\_ratio}\cdot P_{cr}^{\text{ref}}(m/m_{\text{ref}})^{1.5}$；轉換 $P_{tr}=P_{hv}$（保守），持續 `transition_duration_s`。
- 保留：$E_{\text{res}}=\text{reserve\_fraction}\cdot E_{\text{nom}}$（以標稱為基，非以可用為基）；`battery_monitor_validated=False` 時再扣 $k_{\text{unval}}E_{\text{nom}}$（預設 0.10）；`airspeed_sensor_validated=False` 時地速計算的 $V_a$ 乘 0.95。
- $E_{\text{usable}}=E_{\text{avail}}-E_{\text{res}}-k_{\text{unval}}E_{\text{nom}}$。
- 自洽檢查：預設參數下 A 的可用巡航距離 $=(1-0.30-0.10-\text{起降損耗})\times242\approx133$ km，落於表 1 `usable_distance_km` 133–145 ✓（pytest `test_energy_usable_distance_matches_table`）。
- 段能量：`segment_energy(mode, duration_s, gamma) = P_mode * duration_s / 3600`。

---

## 4 任務輸入與前置檢查

### 4.1 MissionInputs（完整鏡射表 2）

```python
@dataclass(frozen=True, slots=True)
class Site:                    # 起飛點／備降點
    site_id: str; lat: float; lon: float; alt_msl_m: float | None   # None→由 DEM 取
    kind: Literal["home", "alternate"]; clear_radius_m: float | None = None
    approach_heading_deg: float | None = None   # None→逆風

@dataclass(frozen=True, slots=True)
class AreaSpec:
    area_id: str; polygon_wgs84: list[tuple[float, float]]   # (lat, lon)，可含 holes
    holes_wgs84: list[list[tuple[float, float]]] = ()
    sensor_req: Literal["lidar", "rgb", "both", "any"] = "any"
    priority: float = 1.0          # 對應論文 val_k
    time_window_s: tuple[float, float] | None = None

@dataclass(frozen=True, slots=True)
class DemGrid:                 # DEM／DSM
    lat0: float; lon0: float; dlat: float; dlon: float
    z_msl: np.ndarray            # (rows, cols) float32, m MSL
    is_dsm: bool = False; vertical_datum: Literal["msl","ellipsoid"] = "msl"
    geoid_undulation_m: float = 0.0

@dataclass(frozen=True, slots=True)
class Obstacle:
    obs_id: str; geom_type: Literal["point","line","polygon"]
    coords_wgs84: list[tuple[float, float]]; top_msl_m: float | None; height_agl_m: float | None
    category: str = "tower"      # tower/building/tree/powerline
    horizontal_buffer_m: float | None = None    # None→spec.obstacle_horizontal_buffer_m

@dataclass(frozen=True, slots=True)
class WindLayer: z_bottom_msl_m: float; z_top_msl_m: float; speed_mps: float; dir_from_deg: float; gust_mps: float = 0.0

@dataclass(frozen=True, slots=True)
class SortieEnergy: uav_id: str; auw_kg: float; available_wh: float | None; available_mah: float | None; pack_voltage_nom_v: float | None

@dataclass(frozen=True, slots=True)
class PayloadRequirement: sensor: LidarSpec | CameraSpec | TrapezoidFovSpec; gsd_required_m: float | None; forward_overlap: float | None; side_overlap: float | None; shutter_rate_hz: float | None

@dataclass(frozen=True, slots=True)
class C2Link: range_km: float | None; coverage_polygon_wgs84: list[tuple[float,float]] | None; ground_station: Site
    lost_link_policy: Literal["rtb","continue_then_rtb","nearest_alternate"] = "rtb"; continue_waypoints: int = 3

@dataclass(frozen=True, slots=True)
class Airspace: max_agl_m: float | None = 120.0; max_msl_m: float | None = None
    no_fly_wgs84: list[list[tuple[float,float]]] = (); time_window_s: tuple[float,float] | None = None
    terrain_clearance_m: float = 30.0     # 表 1 terrain_clearance_m「待任務定義」的任務層預設

@dataclass(frozen=True, slots=True)
class WeatherThresholds: max_mean_wind_mps: float = 10.0; max_gust_mps: float = 14.0; min_visibility_m: float = 3000.0
    max_precip_mmh: float = 0.5; temp_range_c: tuple[float,float] = (0.0, 40.0); abort_gust_mps: float = 16.0

@dataclass(frozen=True, slots=True)
class WeatherObs: mean_wind_mps: float; gust_mps: float; visibility_m: float; precip_mmh: float; temp_c: float

@dataclass(frozen=True, slots=True)
class UAVInstance: uav_id: str; spec: PlatformSpec; sensor: LidarSpec | CameraSpec | TrapezoidFovSpec; sortie: SortieEnergy

@dataclass(frozen=True, slots=True)
class MissionInputs:
    home: Site; alternates: list[Site]; areas: list[AreaSpec]; dem: DemGrid
    obstacles: list[Obstacle]; wind: list[WindLayer]; fleet: list[UAVInstance]
    payload_reqs: dict[str, PayloadRequirement]       # per area_id 或 "default"
    c2: C2Link; airspace: Airspace; weather_thresholds: WeatherThresholds; weather_obs: WeatherObs
    t0_epoch_s: float = 0.0
```

### 4.2 前置檢查（preflight.py）

```
PREFLIGHT(inputs) -> FeasibilityReport
  1 氣象 go/no-go：weather_obs 逐項對 weather_thresholds；任一超限 → status=NO_GO, 終止。
  2 風 vs 平台：每層 W+gust < min(V_a) − 3 m/s（側風可行裕度），Beaufort 5 (≤10.7 m/s) 為能力宣稱僅作警告。
  3 表 1 缺項補齊：c2_link_range_km←c2.range_km；terrain_clearance_m←airspace.terrain_clearance_m；缺者 → NO_GO。
  4 幾何：所有 WGS84 → ENU（§4.3）；多邊形有效化 (shapely make_valid)、面積>0；區域須在 DEM 範圍內。
  5 起降點：DEM 坡度 < 5°；半徑 vtol_site_clear_radius_m 內無障礙；obstacle_horizontal_buffer_m 內無高障礙；
    alternate_landing_sites_required 且 alternates 為空 → NO_GO。
  6 鏈路：所有區域頂點與備降點到 ground_station 距離 ≤ range_km（或在 coverage_polygon 內）；
    否則該區標記 link_infeasible。
  7 對稱任務半徑粗檢：dist(H, 區域最遠點) ≤ symmetric_mission_radius_km(lo) 之對應構型才可指派。
  8 感測需求可達性：每區至少一架構型能滿足 sensor_req 且 AGL 窗可行（§3.2）。
  9 輸出 FeasibilityReport(status, warnings[], errors[], per_area{...}, per_uav{...})。
```

### 4.3 WGS84 ↔ ENU（geo.py）

WGS84：$a=6378137$, $f=1/298.257223563$, $e^2=2f-f^2$, $N(\varphi)=a/\sqrt{1-e^2\sin^2\varphi}$。
- 大地→ECEF：$X=(N+h)\cos\varphi\cos\lambda,\ Y=(N+h)\cos\varphi\sin\lambda,\ Z=(N(1-e^2)+h)\sin\varphi$。
- ECEF→ENU（原點 $(\varphi_0,\lambda_0,h_0)$）：$\mathbf e=\mathbf R(\mathbf P-\mathbf P_0)$，
  $\mathbf R=\begin{bmatrix}-\sin\lambda_0&\cos\lambda_0&0\\-\sin\varphi_0\cos\lambda_0&-\sin\varphi_0\sin\lambda_0&\cos\varphi_0\\\cos\varphi_0\cos\lambda_0&\cos\varphi_0\sin\lambda_0&\sin\varphi_0\end{bmatrix}$。
- 反向：ENU→ECEF $\mathbf P=\mathbf R^\top\mathbf e+\mathbf P_0$；ECEF→大地用 Bowring 閉式（論文引 [39] Zhu 1994）+ 一次牛頓修正，精度 <1 mm。
- 高程基準：$h_{\text{ell}}=H_{\text{msl}}+N_{\text{geoid}}$（`geoid_undulation_m`，任務區單一常數，預設 0 並警告）。
- 地球曲率：DEM 節點與航點皆經 ECEF 轉 ENU，故 ENU 的 $z$ 已含曲率下垂（50 km 處約 196 m）；AGL $=z-z_{\text{terrain}}$ 在同一框架下正確，輸出時反轉回 MSL。此即論文 Eq.(35) 的精確等價，不需另作修正。
- API：`GeoFrame(lat0, lon0, h0_ell)`, `.to_enu(lat, lon, h_ell) -> (N,3)`, `.to_wgs84(e, n, u) -> (N,3)`；全部 numpy 向量化。

### 4.4 地形（terrain.py）

`TerrainGrid`：DEM 重取樣到 ENU 規則網格（步長 `dem_step_m`，預設 10 m；用 `scipy.ndimage.map_coordinates(order=1)` 做雙線性，即論文 Eq.(4)）；障礙物柵格化為「抬升地形」`z_obs = max(z_dem, top_msl)`（線／點以其 buffer 膨脹後燒入）；`slope_deg` 由 `np.gradient`；`z_at(xy)`, `slope_at(xy)`, `max_in_polygon(poly)`。

---

## 5 區域分解

### 5.1 流程

```
DECOMPOSE(area, keepouts, fleet_classes, sensors, wind, terrain) -> list[Cell]
  1 P ← area.polygon − ∪keepouts（禁飛區、無法飛越的高障礙 buffer）
  2 P ← douglas_peucker(P, eps = 0.1 · min_c s^c)                # 論文 DP 簡化，但不整體凸化
  3 cells ← CONVEX_SPLIT(P)                                        # §5.2
  4 for cell in cells:
      cell.h_op[c], cell.feasible[c] ← ALT_WINDOW(cell, c)          # §3.2，c ∈ {A,B}
      cell.dir_candidates ← DIRECTION_CANDIDATES(cell)              # §5.3
      for c in feasible classes: cell.strips[c], cell.alpha[c] ← STRIPS(cell, c, best_dir)  # §5.4
      cell.fit[c] ← FIT(cell, c)                                    # §5.5
  5 return cells
```

### 5.2 凸分割（CONVEX_SPLIT）

論文對整區凸化會造成凹角處大量無效覆蓋；本設計採「反射頂點切割、最小寬度和最小」遞迴法（等價於論文引 [42] 的思路）：
```
CONVEX_SPLIT(P):
  if is_convex(P, tol=2°) and P has no holes: return [P]
  reflex ← reflex_vertices(P) ∪ hole_vertices(P)
  best ← None
  for v in reflex:
    for line in {extend(edge_prev(v)), extend(edge_next(v)), bisector(v)} ∪ {v→u : u ∈ vertices visible from v}:
      parts ← shapely.ops.split(P, line);  if any(area(p) < A_min) continue
      score ← Σ_p (min_width(p) / s_ref)         # 估計條帶總數；s_ref = 較窄構型的條帶間距
              + λ_cut · len(cut) + λ_n · (len(parts) − 1)
      best ← argmin score
  return ∪ CONVEX_SPLIT(p) for p in best.parts
MERGE_PASS: 相鄰兩胞若 union 為凸（tol）且方向候選相同 → 合併（避免碎片）。
```
複雜度 $O(r\cdot n_v^2)$，$n_v\le 60$、$r\le 15$ 時 <20 ms。

### 5.3 掃描方向候選（DIRECTION_CANDIDATES）

候選集合 = 每條邊方向（論文最小寬度方向必在其中）∪ 風向／逆風向 ∪ 地形等高線主方向（DEM 梯度主軸 ⊥）。每候選 $\alpha$ 評分（時間）：
$$T(\alpha)=\sum_l \tfrac{L_l}{2}\Big(\tfrac1{V_g(\alpha)}+\tfrac1{V_g(\alpha+\pi)}\Big)+(n_l-1)\,t_{\text{turn}}(\alpha)+\lambda_{\text{terr}}\cdot\mathbb 1[\max\text{ along-strip slope}>\tan\gamma_c]\cdot\infty$$
$n_l=\lceil W(\alpha)/s\rceil$，$W(\alpha)$ 為垂直於 $\alpha$ 的寬度（shapely `minimum_rotated_rectangle` 改為逐候選投影寬）；$t_{\text{turn}}=(\pi R_g+\max(0,s-2R_g))/V_a$。條帶方向若跨越坡度 $>3°$ 的山脊，3° 爬升無法貼地，AGL 上限（法規 120 m AGL）可能被迫違反 → 該方向標記 `agl_cap_violation`，優先選沿等高線方向。取最小 $T$ 者，保留前 3 名供配置階段換向。

### 5.4 條帶產生（STRIPS）

```
STRIPS(cell, c, alpha) -> list[Strip]
  u ← (cos α, sin α); n ← (−sin α, cos α)          # n 指向 cell 內部（自 E_min 起）
  proj ← [(p − v0)·n for p in cell.vertices]; W ← max(proj)
  h ← cell.h_op[c]; w ← sensor_c.swath_m(h); s ← sensor_c.strip_spacing_m(h)
  n_l ← ceil((W − (w − s)) / s) ; if n_l < 1: n_l ← 1
  offsets ← w/2 + s·(l − 1), l = 1..n_l ; 若最後一條 offset+w/2 > W+w/2 → 整體置中收縮 (offsets −= overshoot/2)
  for l: band ← cell ∩ slab(offset_l ± w/2)          # 論文 Eq.(6)(7) 的上下界，Eq.(8)(9) 端點=band 沿 u 的投影極值
        t_min, t_max ← min/max((p − v0)·u for p in band.exterior)
        ext ← sensor_c.end_extension_m(V_g)           # 相機: footprint_len/2 + V_g·t_settle；LiDAR: V_g·t_settle
        pL ← v0 + (t_min − ext)·u + offset_l·n ; pR ← v0 + (t_max + ext)·u + offset_l·n
        strips.append(Strip(id, cell.id, l, pL, pR, psi_LR=α, width=w, cls=c, h_op=h))
  return strips
```
兩趟法（陡地形）：高度規劃後若某條帶 AGL$_{\min}<h$（因空域上限壓迫，§9.2），以 $h'=\text{AGL}_{\min}$ 重新產生該胞條帶（單調收縮，最多 2 次）。

### 5.5 構型適配度與殘帶

$$\text{fit}_c(q)=\omega_1\,\frac{T^{\min}_{\text{cell}}}{T^{c}_{\text{cell}}}+\omega_2\,\mathbb 1[\sigma_q\ \text{允許}\ c]+\omega_3\,\frac{\min(W,\,4R_g^c)}{4R_g^c}+\omega_4\,\frac{d(H,q)}{r^{c}_{\text{sym}}}\cdot(-1)$$
（時間效率、感測需求、狹窄胞對轉彎半徑的懲罰、距離對續航的懲罰）。`sensor_req="any"` 且寬胞：A 以 $k_A$ 條帶覆蓋前 $k_A s_A$ 寬，殘帶寬 $W_r=W-k_A s_A$ 由 B 以 $\lceil W_r/s_B\rceil$ 條覆蓋；$k_A\in\{0..n_A\}$ 由配置階段以 makespan 決定（產生子胞 `cell_q^A`, `cell_q^B`）。`sensor_req="both"`：兩構型各自產生完整條帶集（高度分層，§9.3）。

---

## 6 任務配置

成本一律為時間；目標 min-max 落地時刻。輸入：所有胞（含子胞）的條帶集合 $\mathcal L$、各 UAV 初始狀態（轉換完成點，§9.1）與能量。

### 6.1 單機工作量估計

對 UAV $i$ 與胞 $q$：$T^{\text{work}}_{i,q}=\sum_l\bar\tau^{\text{op}}_{i,l}+(n_l-1)\bar\tau^{\text{tr}}+\tau^{\text{in}}_{i,q}$（$\bar\tau^{\text{op}}$ 為兩方向平均、$\bar\tau^{\text{tr}}$ 相鄰條帶 U-turn 時間、$\tau^{\text{in}}$ 最短進入時間）；$T^{\text{rtb}}_{i,q}$ 由胞質心至最近可用降落點（含下降走廊）；能量 $E_{i,q}=E(\text{takeoff})+E(T^{\text{work}})+E(T^{\text{rtb}})$；可行 $\iff E_{i,q}\le E_{\text{usable},i}$ ∧ 鏈路 ∧ 感測 ∧ 半徑。

### 6.2 擴充配置指標與貪婪配置（Algorithm 1'）

$$J_q=\omega_1\frac{\sum_i\min_l\tau^{\text{in}}_{i,q,l}}{\sum_{q'}\sum_i\min_l\tau^{\text{in}}_{i,q',l}}+\omega_2\frac{n_q}{\sum n_{q'}}+\omega_3\frac{T^{\text{op}}_q}{\sum T^{\text{op}}_{q'}}+\omega_4\frac{\pi_q}{\sum\pi_{q'}}+\omega_5\frac{\max_c \text{fit}_c(q)}{\sum_{q'}\max_c\text{fit}_c(q')}$$
預設 $\omega=(0.2,0.2,0.2,0.2,0.2)$；$\omega_5=0$ 且其餘 0.25 時退化為論文 Eq.(30)。$\lambda_q=\text{round}(n_U J_q/\sum J)$，修正使 $\sum\lambda_q=n_U$ 且每個有需求的胞 $\lambda_q\ge1$（`both` 胞的 A、B 子任務各 ≥1）。

```
ALLOCATE(cells, fleet, costs) -> dict[team_id, Team]
  1 for q: 計算 J_q, λ_q；feasible[i,q] (§6.1)
  2 Z_R ← cells sorted by J desc ; U_R ← fleet
  3 while Z_R: q ← pop ; cand ← [i ∈ U_R : feasible[i,q]] ；
      依 key = τ_in[i,q] + M·(1 − fit_{c(i)}(q)) 取前 λ_q 架 → team_q ; U_R −= team_q
      若 cand 不足 λ_q：λ_q ← |cand|；若 |cand| = 0 → 進入多架次佇列 (§6.4)
  4 BALANCE：估計 T̂_i = t_takeoff_i + τ_in + T_work_q/λ_q + T_rtb；
      repeat ≤ n_U 次：從 argmax T̂ 團隊挑一架移至可行且使 max T̂ 下降最多的團隊；無改善則停
  5 若 U_R 非空：以 min-max 準則把剩餘 UAV 加入使 max T̂ 降幅最大的團隊
  6 團隊合併：同構型且胞相鄰、單一 UAV 團隊 → 合併成多胞團隊（IDP 在聯集條帶上跑，§8）
  7 return teams
```

### 6.3 拍賣式配置（選項）

`allocate(method="auction")`：每輪每架 UAV 對每個未指派胞出價 $b_{i,q}=\Delta\hat T_{\max}$（加入後 makespan 增量），取全域最小出價成交；同構型排除競價（同一胞由同構型多機分擔時以 λ 決定）。適合動態重配置（只對受影響胞重新拍賣）。

### 6.4 多架次（選項 `allow_multi_sortie`）

無法一次涵蓋的胞：以條帶索引區間切成子胞使每段能量 $\le E_{\text{usable}}$；第二架次起飛時刻 = 落地 + `turnaround_s`（換電，預設 600 s）；makespan 含之。

### 6.5 起飛排程

起降走廊互斥：起飛間隔 `takeoff_separation_s`（預設 60 s）；起飛順序 = 進入時間最長者先起（減小 makespan）；每架初始時刻 $t_i^0=k_i\cdot$ 間隔，饋入 IDP 初始狀態。

---

## 7 三類路徑成本

### 7.1 作業路徑（時間二元）

$$\tau^{\text{op}}_{i,l}=\Big[\tfrac{|p^R-p^L|}{V_g(\hat u_l)},\ \tfrac{|p^R-p^L|}{V_g(-\hat u_l)}\Big]$$
$V_g$ 用 $V_{a,\text{op}}=\min(V_a,\ \text{由 } V^{\text{sens}}_{\max}\text{ 反推})$；高度取 $h_{\text{op}}+\bar z_{\text{terr}}$ 對應風層。條帶內若有可飛越障礙（頂高+淨空 ≤ 空域上限）不影響 2D 長度，交由高度規劃。

### 7.2 進入路徑（時間二元）

起點 $S_i=(x_{tc},y_{tc},\psi_{\text{into-wind}},R_g^{c(i)},V_a,t_i^0+t_{\text{vtol}}+t_{\text{tr}},E_i^0-E_{\text{takeoff}})$。
```
ENTRY_TIME(S_i, strip, side) :
  target state S^L=(pL, ψ_LR) 或 S^R=(pR, ψ_RL)
  path ← OBSTACLE_AWARE_DUBINS(S_i, S^side, keepouts, R_g)            # §7.4
  t ← Σ_seg |seg| / V_g(seg.dir, seg.z) ；另加爬升不足時的盤旋圈數 n_orb·(2πR_g/V_a) (§9.1)
  return t, path
```

### 7.3 轉移路徑（時間二元）

自條帶 $l$ 終點狀態到條帶 $m$ 端點：方向交替（論文 Eq.24–26）由決策矩陣約束：LR 掃 $l$ → 到 $p^R_m$ 以 RL 掃 $m$。
$$\tau^{\text{tr}}_{i,l,m}=\Big[\text{time}(\mathcal D(S^L_l\!\to\!S^L_m)),\ \text{time}(\mathcal D(S^R_l\!\to\!S^R_m))\Big]$$
同胞平行條帶（間距 $d$）閉式：$d\ge2R_g$：$\pi R_g+(d-2R_g)$；$d<2R_g$：一般 Dubins 求解器（LRL/RLR）。跨胞轉移：一般 Dubins + keep-out 檢查。時間 = 弧段以 $V_a$（保守，趨線平均地速 ≈ $V_a$）、直線段以 $V_g$。

### 7.4 障礙感知 Dubins（routing.py）

```
OBSTACLE_AWARE_DUBINS(S_a, S_b, keepouts, R):
  d ← dubins(S_a, S_b, R)                                  # 6 種候選向量化，取最短
  if not intersects(d.polyline, keepouts): return d
  G ← visibility graph over vertices of keepouts.buffer(R + margin) ∪ {S_a.xy, S_b.xy}   # 任務初始化時預建，僅新增兩點
  poly ← dijkstra(G, S_a.xy, S_b.xy)
  # 轉角以 Dubins 圓弧接合（論文 Eq.27 的多障礙一般化）：每轉角插入切線點，半徑 R；相鄰轉角過近時以 R 圓繞行
  return join_with_arcs(poly, S_a.psi, S_b.psi, R)
```
弧段離散化：論文 Eq.(28)(29)，每段最大轉角 `chi_max_deg`（預設 15°）。

### 7.5 成本矩陣與快取（costs.py）

```python
class CostCache:
    op:   dict[cls, np.ndarray]          # (n_strips, 2)   只依構型（V_a, 風）而非個機
    tr:   dict[cls, np.ndarray]          # (n_strips, n_strips, 2)  只依構型 R_g、V_a
    entry: dict[uav_id, np.ndarray]      # (n_strips, 2)   依個機初始狀態
    rtb:  dict[(cls, site_id), np.ndarray]   # (n_strips, 2) 自條帶兩端點到各降落點下降起點
    wind_key: int                        # 風場版本；變更僅使 op/tr 的時間層失效，Dubins 幾何層保留
```
矩陣 $\mathbf T_i$（論文 Eq.13 的時間版）由上述切片即時組合：第 0 列 `entry[i] + op[c]`、其餘 `tr[c][l,:] + op[c]`；不落地實體化 $n_U$ 份。

---

## 8 異構 IDP

### 8.1 狀態與初始化（論文 Eq.32 擴充）

團隊 $\mathcal U_g$、條帶集合 $\mathcal L_g$（可跨多胞）。狀態 $\Sigma(s)=\{S_i(s)\}_{i\in\mathcal U_g}$，$\mathcal L^{\text{NAD}}(s)$ 未覆蓋條帶；$S_i(1)$ = 轉換完成狀態（含 $t_i^0$ 起飛延遲）；$R_i(1)=\varnothing$。

### 8.2 階段轉移成本

單機增量：$\Delta_i=\tau^{\text{tr}}_{i,l_i(s),l}[\text{dir}]+\tau^{\text{op}}_{i,l}[\text{dir}']$（進入階段用 $\tau^{\text{in}}$）；方向由交替約束決定。
階段成本（min-max 導向）：
$$g(\Sigma,\text{sel})=\max_{i}\big(t_i+\Delta_i\big)+\beta\,\tfrac1{|\mathcal U_g|}\sum_i\Delta_i+\mu\sum_i \mathbb 1[\text{cross-cell}]$$
$\beta=0.1$ 破平局，$\mu$ 抑制無謂跨胞。`metric="paper"` 時改為論文 Eq.(33) $\sum_i\Delta_i/\text{dis}_i(s)$（退化測試用）。
硬剪枝：$E_i-E(\Delta_i)-E^{\text{rtb}}_{\min}(l)<0$ 的選擇不列入（$E^{\text{rtb}}_{\min}(l)$ 自條帶 $l$ 終點至最近降落點的最小能量，預算表查詢）。

### 8.3 Beam-IDP（Algorithm 2'）

```
BEAM_IDP(team, strips, cache, K=6, B=3):
  beam ← [Node(Σ(1), NAD=strips, R=∅, g=0)]
  while any(node.NAD for node in beam):
    next ← []
    for node in beam:
      if |node.NAD| ≥ |team|:
        cand_i ← for each i: K nearest strips in node.NAD by min(tr[i,·]) (含兩端)   # k-nearest 剪枝
        SEL ← 所有「各機取不同條帶」的組合 ∏ cand_i 去重（|SEL| ≤ K^λ，λ=|team|，λ≤4 時 ≤ 1296）
      else:
        SEL ← 由 team 中選 |NAD| 架的排列 A(λ,|NAD|) × 指派到 NAD              # 論文 else 分支
      for sel in SEL:
        Δ ← 向量化查表 (tr/op/entry) ；若能量剪枝失敗 continue
        next.append(child(node, sel, g = node.g + stage_cost))
    beam ← top-B of next by (max_i t_i, g)     # 主鍵 makespan 現值，次鍵 g
    去重：以 (frozenset(NAD), tuple(current strip & dir per UAV)) 為鍵保留最佳
  best ← argmin over beam of max_i t_i
  return backtrack(best) → per-UAV 序列 [(strip, dir)], E_i (Eq.14), t_i
```
$B=1,K=\infty$ 且 `metric="paper"`、單胞、同構 → 與論文 Algorithm 2 等價（除成本單位）。

### 8.4 返航與能量修復

```
APPEND_RTB(plan_i):
  for site in sorted(landing sites by rtb[c,site][last_strip, dir]):
    path ← OBSTACLE_AWARE_DUBINS(S_last, descent_entry(site)) ; if energy ok → 加入 ; break
  if none: 彈出 plan_i 最後一條帶 → 放回 leftover ; 重試
leftover 非空 → 回到 ALLOCATE（僅對 leftover 條帶，其他團隊計畫凍結）→ 多架次或報告 uncovered。
```
逐航點備降可達檢查：$E(wp)\ge E_{\text{res}}+\min_{a\in\mathcal A\cup\{H\}}E_{\text{fly}}(wp\to a)$；$E_{\text{fly}}$ 以直線距離／逆風最壞地速估計並查預算表（每航點 $O(|\mathcal A|)$）。

---

## 9 三維高度與起降走廊

### 9.1 起降走廊（corridor.py）

起飛（模式旗標）：
```
HOVER:      H (z_site) → z_site + transition_start_agl_m           垂直，vtol_climb_rate_mps；能量 P_hv
TRANSITION: 逆風航向 ψ_w，水平 transition_ground_run_m，爬至 transition_complete_agl(lo)；P_tr, transition_duration_s
CRUISE:     3° 爬升至 h_op：需水平距離 d_c = (h_op − h_tc)/tan γ_c（A：Δh=40 m→763 m；Δh=100 m→1.9 km）
            若 |進入路徑| < d_c：插入 n_orb = ceil((d_c − |entry|)·tan γ_c / orbit_gain) 圈盤旋（半徑 R_g，中心在走廊內、避開 keep-out）
```
降落：4° 下降至 $h_{tc}$（下降起點 = 降落點上風 $d_d=(h_{\text{cruise}}-h_{tc})/\tan\gamma_d$ 處，航向逆風）→ TRANSITION（減速到 `transition_airspeed_mps`）→ HOVER 垂直下降 `vtol_descent_rate_mps`。走廊 = 以進場線為軸、寬 $2R_g$、長 $d_d$ 的矩形 + 起降點 `vtol_site_clear_radius_m` 圓，需無障礙（`obstacle_horizontal_buffer_m`）。走廊佔用時間窗登記於 `CorridorSchedule`（§11）。

### 9.2 高度剖面（altitude.py，取代論文 Algorithm 3）

```
ALTITUDE_PROFILE(path2d, phases, terrain, spec, airspace):
  1 P ← resample(path2d, dd)                                  # dd 預設 20 m（弧段依 Eq.28/29 已離散）
  2 z_terr ← terrain.z_at(P) （含障礙抬升）
  3 z_req ← z_terr + h_req(phase)   # OPERATE/TRANSFER: h_op；ENTRY/RTB: max(h_min_fw, clearance)；沿途加 clearance
  4 slope-limited envelope（永不低於 z_req）：
      forward:  z_f[j] = max(z_req[j], z_f[j−1] − dd·tan γ_d)      # 限制下降率
      backward: z_b[j] = max(z_req[j], z_b[j+1] − dd·tan γ_c)      # 限制爬升率
      z = max(z_f, z_b)
  5 空域上限：cap = z_terr + max_agl（或 max_msl）；若 z > cap 於區間 J：
      標記 agl_cap_violation(J)；嘗試 z_req' 以 h_op' = max(h_min_fw, h_op − δ) 重跑（δ=10 m，≤3 次）；
      仍違反 → 回報 §5.4 兩趟法（改方向或縮條帶）
  6 平滑（論文 Eq.37/38 GDA，c1=1, c2=0.5，iters=20）後再執行第 4 步夾持（保證約束）
  7 共線點刪減（論文 Eq.39，ε=1e-3）但保留 phase 邊界點與 sensor_on 切換點
  8 AGL_min/AGL_max per strip 回傳（供 §5.4 兩趟法與相機 GSD 檢查：AGL_max ≤ h_gsd_max）
```
說明：作業路徑上 AGL $\ge h_{\text{op}}$ 由包絡構造保證，因此以 $h_{\text{op}}$ 算 swath 對 LiDAR 保守；相機需檢查 AGL$_{\max}$。

### 9.3 高度分層

- 構型 B 以 $h^B_{\text{op}}$ 作業；構型 A 於同一或相鄰胞時 $h^A_{\text{op}}\leftarrow\max(h^A_{\text{op}},\ h^B_{\text{op}}+\Delta z_{\text{sep}})$，$\Delta z_{\text{sep}}$ 預設 30 m（受 A 的 LiDAR 測距與空域上限約束；不可行則改以時間分離）。
- 進入／返航段使用「過境高度」$h_{\text{transit}}^{c}=h^c_{\min}+\Delta z_{\text{transit}}\cdot\mathbb 1[c=A]$。
- 起降走廊高度 $\le h_{tc}(\text{hi})$ 與作業層分離。

---

## 10 動態重規劃

事件 → 統一處理：
```
REPLAN(event, plan, telemetry, t_e):
  1 快照：每機 S_i(t_e)（位置、航向、能量、現行條帶與已掃比例 ρ）；部分完成條帶 → 剩餘段成為新條帶（端點 = 目前位置投影 + 外延）
  2 受影響集合 A：
      UAV_EXIT: 該機團隊；NEW_AREA: 分解新區 → 對全部團隊做「增量拍賣」但僅接受使 makespan 增量最小的 ≤2 個團隊變更；
      LOW_ENERGY/ENERGY_DEVIATION(κ 更新): 該機（先重驗 RTB；不足則彈出條帶→leftover→同構型鄰隊）；
      LINK_LOSS: 依 lost_link_policy 執行預算好的動作（航點附帶 lost_link_action）；規劃端把該機視為 EXIT；
      WIND_UPDATE: cache.wind_key++ → 重算 op/tr 時間層（Dubins 幾何層若 R_g 變化 >5% 才重算）→ 所有團隊剩餘條帶重跑 IDP（暖啟動：以現行序列為 beam 初始解）；
      CUE(target xy): 產生 confirm 子任務（圓 r=150 m、sensor_req=rgb、priority ↑）→ 對構型 B 拍賣，得標機以最小 makespan 增量插入繞行；
      WEATHER_ABORT: 全機 RTB（走廊排程 §11）。
  3 未受影響團隊計畫凍結（增量重規劃）；受影響團隊：ALLOCATE(leftover ∪ 未掃條帶, A) → BEAM_IDP（初始狀態 = 快照，遵守「先完成現行條帶」規則）→ ALTITUDE → DECONFLICT（僅對變更機重驗）。
  4 輸出 diff：僅變更機的航點清單重新下傳。
```
能量偏差監控：每航點比對 `E_actual` 與計畫 `energy_remaining_wh`；偏差 $>5\%E_{\text{nom}}$ → $\kappa\leftarrow\kappa\cdot E_{\text{used,act}}/E_{\text{used,plan}}$（限幅 0.8–1.5）並觸發 ENERGY_DEVIATION。

---

## 11 防撞／去衝突

1. 高度分層（§9.3）為第一道；同層才做時間分離。
2. 4D 分離檢查（deconflict.py）：每機航點依時間戳線性內插，取樣 Δt=2 s，兩兩以時間窗滑動（僅比較 |t_a−t_b|<Δt 的點）；違規 $\iff$ 水平 $<d_h$（預設 100 m）且垂直 $<d_v$（30 m）。用 KD-tree（`scipy.spatial.cKDTree`）對 (x,y,t·V_a) 三維點查詢，$O(N\log N)$。
3. 修復：優先級低者（makespan 貢獻小者）延遲：在最近前一條帶末插入一圈盤旋（$2\pi R_g/V_a$ s）或起飛延遲 +Δ；重驗能量；最多 3 次，否則垂直偏置 ±15 m（限 AGL 窗內）。
4. 起降走廊互斥：`CorridorSchedule` 區間表；起飛間隔固定；落地預約窗 [t_arr−30, t_arr+90]；衝突則在保持點（走廊入口外 2 km、過境高度）盤旋等待，能量檢查後才允許，否則改備降點。
5. 條帶共享：同胞同構型多機時，IDP 已保證條帶互斥；相鄰條帶反向同時掃描的側距 $=s\ge$ swath·(1−overlap) 通常 $<d_h$ → 以高度微分層 ±10 m（同胞內奇偶條帶）或時間偏移處理；預設同胞多機由 IDP 的階段同步即已錯開端點時序，檢查器把關。

---

## 12 複雜度與即時性

### 12.1 各階段複雜度（$n_U=8$, $n_Z=3$, 條帶 $n\approx80$, 胞 $n_C\approx8$, keep-out 頂點 $V\approx100$, DEM $10^6$ 節點）

| 階段 | 複雜度 | 估計耗時 |
|---|---|---|
| 前置檢查＋WGS84→ENU | $O(\text{點數})$ | 5 ms |
| DEM 重取樣＋障礙燒入 | $O(\text{grid})$，一次性 | 150 ms（初始化，不計入重規劃） |
| 凸分割 | $O(r n_v^2)$ | 20 ms |
| 方向候選＋條帶 | $O(n_C\cdot n_{\text{dir}}\cdot n)$ | 15 ms |
| 可視圖 | $O(V^2\log V)$ 一次性 | 30 ms |
| 成本快取：op $2n$、tr $2\times2n^2$ Dubins（向量化）、entry $2n_Un$ | $O(n^2)$ | 60 ms |
| 配置＋平衡 | $O(n_U n_C\log)$ | 5 ms |
| Beam-IDP：階段 $\approx n/\lambda$，每階段 $B\cdot K^\lambda$ | $O(\tfrac n\lambda B K^\lambda)$；$K=6,\lambda\le3,B=3$ → $\sim 2\times10^4$ 向量化查表 | 120 ms |
| RTB＋能量修復 | $O(n_U|\mathcal A|)$ | 10 ms |
| 高度剖面：每機約 3000 點 | $O(N)$ | 60 ms |
| 去衝突 KD-tree | $O(N\log N)$ | 40 ms |
| 輸出 ENU→WGS84 | $O(N)$ | 5 ms |
| **合計（重規劃路徑）** | | **≈0.4–0.6 s** |

論文完整列舉 $A^\lambda_{n}$ 在 $n=27,\lambda=4$ 時 $\approx 4\times10^5$／階段，不可即時；k-nearest 使其降至 $6^4=1296$。

### 12.2 即時策略

1. 成本矩陣以構型為鍵（2 份而非 8 份）；Dubins 六案向量化 numpy（一次計算所有 $(l,m)$ 對）。
2. 幾何層／時間層分離快取：風更新只重算時間層。
3. k-nearest（$K$）＋beam（$B$）；預設 $K=6,B=3$；`realtime_budget_s` 超時則自動降 $B\to1$。
4. 增量重規劃：未受影響團隊計畫凍結；受影響團隊 IDP 以現行序列暖啟動。
5. 能量預算表（每條帶端點→各降落點）預計算，IDP 剪枝 $O(1)$ 查表。
6. 覆蓋率柵格驗證僅在 validate（非規劃迴圈）執行。
7. 全流程無 Python 逐點迴圈於熱路徑；`@dataclass(slots=True)`；DEM float32。

---

## 13 與論文對照

| 論文元件 | 本設計 | 退化條件 |
|---|---|---|
| Eq.1 運動學（定速、$R_{\min}$、$\gamma$） | 同；$R_g$ 含風；$\gamma_c\neq\gamma_d$ | $W=0,\gamma_c=\gamma_d$ |
| Eq.2–3 梯形 FOV | LiDAR／相機模型；`TrapezoidFovSpec` 選項 | 選 legacy FOV |
| Eq.4 DEM 雙線性 | `map_coordinates(order=1)` | 同 |
| DP 簡化＋整體凸化 | DP 簡化＋凸分割＋合併 | `convexify_whole=True` |
| 最小寬度方向 | 候選集含之；時間評分 | $W=0$、平地 → 選最小寬度 |
| Eq.6–9 條帶與端點 | 同幾何（置中收縮、端點外延） | `end_extension=0` |
| Eq.10–12 二元距離 | 二元時間 | $V_g\equiv V$ 時等比 |
| Eq.13–14 $D,E$ | $\mathbf T$ 切片組合、$\mathbf E$ 相同 | — |
| Eq.17–26 MDTSP | 加能量／鏈路／感測硬限制、RTB | 限制關閉 |
| Eq.27–29 Dubins 繞行／離散 | 可視圖＋Dubins 接合；Eq.28/29 離散 | 單圓威脅時等價 |
| Eq.30–31 $J_k,\lambda_k$ | 加 $\omega_5$ fit、時間單位 | $\omega_5=0$ |
| Algorithm 1 貪婪 | Alg.1' ＋ BALANCE ＋拍賣選項 | 關閉 BALANCE |
| Eq.32–34 IDP | Beam-IDP、min-max 階段成本、跨胞 | $B=1,K=\infty$,`metric="paper"` |
| Eq.35 曲率修正 | ECEF→ENU 精確 | — |
| Eq.36 高度下降 | forward/backward 坡度包絡 | 同 $\gamma$ 時等價 |
| Eq.37–39 平滑／刪減 | 同＋夾持 | 同 |
| 動態事件（退出／新區） | ＋能量／失鏈／風／cue／氣象；增量 | — |
| 無 | 起降走廊、盤旋爬升、能量、備降、去衝突、前置檢查 | 新增 |

`legacy_mode=True` 開關一次設定上述所有退化條件，用於 §17 對照測試。

---

## 14 參數表

### 14.1 表 1 全部鍵（值不變；規劃取用端標示）

| 鍵 | Pioneer PNP (A) | HERO PNP (B) | 規劃取用 |
|---|---|---|---|
| platform_type | lift_and_cruise_vtol_4plus1 | 同 | 模式機 |
| wingspan_m | 3.20 | 2.18 | 起降淨空校核 |
| fuselage_length_m | 1.68 | 1.14 | 不用於規劃 |
| wing_area_m2 | 1.00 | 0.53 | 翼載（僅報告） |
| mtow_kg | 24.9 | 7.0 | AUW ≤ mtow 檢查 |
| reference_auw_kg | 24.6 | 6.65 | 功率 AUW 修正基準 |
| reference_payload_kg | 5.0 | 0.6 | 能耗模型基準 |
| payload_max_kg | 5.0 | 1.0 | 酬載檢查 |
| cruise_airspeed_mps | 21.5 | 19.0 | $V_a$ |
| cruise_airspeed_kmh | 77.4 | 68.4 | 顯示 |
| min_mission_airspeed_mps | 18.0 | 15.0–16.0 →16.0 | 作業降速下限 |
| transition_airspeed_mps | 16.0 | 14.0 | 走廊 |
| stall_airspeed_mps | 14.0 | 12.0 | 安全檢查 |
| max_airspeed_mps | 35.0 | 待實機確認 → 1.25·cruise (provisional) | 上限檢查 |
| max_climb_angle_deg | 3.5 | 3.5 | 驗證上限 |
| planning_climb_angle_deg | 3.0 | 3.0 | $\gamma_c$ |
| max_descent_angle_deg | 5.0 | 5.0 | 驗證上限 |
| planning_descent_angle_deg | 4.0 | 4.0 | $\gamma_d$ |
| max_fw_climb_rate_mps | 1.13 | 1.00 | 自洽檢查 |
| max_fw_descent_rate_mps | 1.50 | 1.33 | 自洽檢查 |
| max_bank_angle_deg | 20 | 25 | $R$ 自洽（$V^2/(g\tan\phi)$：A 129.5 m ✓、B 79 m ✓） |
| turn_radius_theoretical_m | 130 | 75 | 報告 |
| turn_radius_planning_m | 200 | 120 | $R_p$ |
| vtol_site_clear_radius_m | 8 | 6 | 起降點檢查 (provisional) |
| obstacle_horizontal_buffer_m | 30 | 25 | 起降點／keep-out (provisional) |
| transition_start_agl_m | 60 | 50 | 走廊 (provisional) |
| transition_complete_agl_m | 80–100 | 70–90 | 走廊 lo/hi (provisional) |
| min_fw_operation_agl_m | 120 | 100 | $h_{\min}$ |
| wind_resistance_public | 5 | 5 | 警告門檻 |
| reference_endurance_min | 176 | 136 | $P_{cr}^{\text{ref}}$ |
| reference_total_range_km | 242 | 144 | 自洽 |
| reserve_fraction | 0.30 | 0.30 | $E_{\text{res}}$ |
| usable_range_fraction | 0.55–0.60 | 0.55–0.60 | 自洽（取 0.55） |
| usable_distance_km | 133–145 | 79–86 | 自洽（取 lo） |
| symmetric_mission_radius_km | 66–72 | 39–43 | 粗檢（取 lo） |
| airspeed_sensor_validated | false | false | $V_a$×0.95 |
| battery_monitor_validated | false | false | 加扣 10% |
| c2_link_range_km | 待填入 | 待填入 | 由 MissionInputs.c2 |
| terrain_clearance_m | 待任務定義 | 待任務定義 | 由 Airspace.terrain_clearance_m |
| alternate_landing_sites_required | true | true | 前置檢查 |

### 14.2 演算法參數（`PlannerConfig`）

| 參數 | 預設 | 說明 |
|---|---|---|
| dem_step_m | 10 | ENU 重取樣 |
| dp_eps_factor | 0.1 | DP 容差 = 0.1·min 條帶間距 |
| convex_tol_deg | 2 | 凸判定 |
| A_min_m2 | $s^2$ | 最小胞面積 |
| n_dir_candidates | ≤8 | 方向候選 |
| t_settle_s | 2.0 | 端點外延 |
| lidar side_overlap | 0.20 | |
| wind_turn_radius_model | trochoid_max | $R_g$ |
| chi_max_deg | 15 | 弧離散 |
| K, B | 6, 3 | 剪枝、beam |
| beta, mu | 0.1, 30 s | 階段成本 |
| omega | (0.2,0.2,0.2,0.2,0.2) | $J_q$ |
| takeoff_separation_s | 60 | |
| turnaround_s | 600 | 多架次 |
| dd_m | 20 | 剖面步長 |
| gda_c1, gda_c2, gda_iters | 1, 0.5, 20 | 平滑 |
| eps_collinear | 1e-3 | 刪減 |
| dz_sep_m, d_h_m, d_v_m | 30, 100, 30 | 分層／分離 |
| k_unvalidated_energy | 0.10 | |
| energy_dev_trigger | 0.05 | κ 更新 |
| realtime_budget_s | 1.0 | 超時降 B |
| cov_raster_m | 5 | 驗證柵格 |

---

## 15 套件架構與 API（ANGLE 主體）

### 15.1 目錄

```
hv_dccpp/
  __init__.py       plan_mission, replan, validate, PlannerConfig 匯出
  geo.py            GeoFrame：WGS84↔ECEF↔ENU、geoid
  specs.py          PlatformSpec, LidarSpec, CameraSpec, TrapezoidFovSpec, UAVInstance, PLATFORM_PIONEER/HERO
  mission.py        MissionInputs 與所有表 2 子 dataclass；load_mission(json|dict)
  terrain.py        TerrainGrid
  wind.py           WindField
  energy.py         EnergyModel, EnergyBudgetTable
  sensor.py         Sensor 協定實作、altitude_window()
  preflight.py      preflight() -> FeasibilityReport
  decompose.py      simplify, convex_split, direction_candidates, make_strips, residual_bands
  dubins.py         dubins_all(starts, ends, R) 向量化、discretize_arcs
  routing.py        KeepoutSet, VisibilityGraph, obstacle_aware_dubins
  costs.py          CostCache.build / .refresh_wind
  allocate.py       allocate(method="greedy"|"auction")
  idp.py            beam_idp
  corridor.py       takeoff_profile, landing_profile, CorridorSchedule
  altitude.py       altitude_profile
  deconflict.py     check_separation, repair
  replan.py         replan(event, ...)
  plan.py           Waypoint, UAVPlan, MissionPlan, to_geojson, to_qgc_wpl
  validate.py       coverage_raster, checks, baselines
  viz.py            plot_2d, plot_profile, plot_gantt
  config.py         PlannerConfig
```

### 15.2 核心資料型別（輸出）

```python
class Mode(Enum): HOVER=0; TRANSITION=1; CRUISE=2
class Phase(Enum): TAKEOFF, CLIMB, ORBIT, ENTRY, OPERATE, TRANSFER, RTB, DESCENT, LAND, HOLD

@dataclass(slots=True)
class Waypoint:
    e: float; n: float; u: float            # ENU m
    lat: float; lon: float; alt_msl_m: float; agl_m: float
    t_s: float                              # 自 t0 起秒
    mode: Mode; phase: Phase; sensor_on: bool
    strip_id: str | None; cell_id: str | None
    airspeed_mps: float; ground_speed_mps: float; heading_deg: float
    energy_remaining_wh: float
    lost_link_action: str                   # "rtb"|"continue"|"alt:<site_id>"
    nearest_alternate: str; alt_reachable: bool

@dataclass(slots=True)
class UAVPlan:
    uav_id: str; platform_class: str; waypoints: list[Waypoint]
    strips: list[tuple[str, int]]           # (strip_id, dir ∈{−1,+1})
    t_takeoff_s: float; t_land_s: float; landing_site: str
    energy_used_wh: float; energy_margin_wh: float; distance_m: float
    orbits_inserted: int; warnings: list[str]

@dataclass(slots=True)
class MissionPlan:
    plans: dict[str, UAVPlan]; makespan_s: float; uncovered_strips: list[str]
    feasibility: FeasibilityReport; cells: list[Cell]; strips: dict[str, Strip]
    cost_cache_key: int; timing_ms: dict[str, float]
```

### 15.3 函式簽章（型別＋單位）

```python
# geo.py
class GeoFrame:
    def __init__(self, lat0_deg: float, lon0_deg: float, h0_ell_m: float, geoid_undulation_m: float = 0.0)
    def to_enu(self, lat_deg: ArrayLike, lon_deg: ArrayLike, h_msl_m: ArrayLike) -> np.ndarray   # (N,3) m
    def to_wgs84(self, enu: np.ndarray) -> np.ndarray                                             # (N,3) deg,deg,m MSL
# terrain.py
class TerrainGrid:
    @classmethod
    def from_dem(cls, dem: DemGrid, frame: GeoFrame, step_m: float, obstacles: list[Obstacle] | None) -> "TerrainGrid"
    def z_at(self, xy: np.ndarray) -> np.ndarray                 # m (ENU u)
    def slope_at(self, xy: np.ndarray) -> np.ndarray             # deg
    def max_z_in(self, poly: Polygon) -> float
# wind.py
class WindField:
    def wind_vec(self, z_msl: np.ndarray) -> np.ndarray          # (N,2) m/s 吹向
    def ground_speed(self, track_unit: np.ndarray, z_msl: np.ndarray, v_air: float) -> np.ndarray
    def turn_radius_ground(self, spec: PlatformSpec, z_msl: float) -> float
# sensor.py
def altitude_window(sensor, spec: PlatformSpec, airspace: Airspace, terrain: TerrainGrid, cell: Polygon) -> tuple[float, float] | None
# energy.py
class EnergyModel:
    def power_w(self, mode: Mode, gamma_rad: float = 0.0) -> float
    def segment_wh(self, mode: Mode, duration_s: float, gamma_rad: float = 0.0) -> float
    def usable_wh(self) -> float
class EnergyBudgetTable:                                          # (strip_end, site) → Wh
    def __init__(self, strips, sites, models: dict[str, EnergyModel], wind: WindField, terrain: TerrainGrid)
    def min_rtb_wh(self, cls: str, strip_id: str, side: int) -> float
# decompose.py
def simplify(poly: Polygon, eps_m: float) -> Polygon
def convex_split(poly: Polygon, s_ref_m: float, cfg: PlannerConfig) -> list[Polygon]
def direction_candidates(cell: Polygon, wind: WindField, terrain: TerrainGrid, spec, sensor, cfg) -> list[tuple[float, float]]  # (alpha_rad, T_est_s)
def make_strips(cell: Cell, cls: str, alpha_rad: float, sensor, spec, v_g_fn) -> list[Strip]
def residual_bands(cell: Cell, k_a: int, sensors: dict, specs: dict) -> tuple[Cell, Cell]
# dubins.py
def dubins_all(starts: np.ndarray, ends: np.ndarray, R: float) -> tuple[np.ndarray, np.ndarray]   # starts/ends (N,3)=(x,y,psi) → lengths (N,), words (N,)
def dubins_path(start, end, R, chi_max_rad) -> np.ndarray       # (M,3) 離散 (x,y,psi)
# routing.py
class KeepoutSet: def __init__(self, polys: list[Polygon], buffer_m: float)
class VisibilityGraph: def __init__(self, keepouts: KeepoutSet); def shortest(self, a: np.ndarray, b: np.ndarray) -> np.ndarray
def obstacle_aware_dubins(sa, sb, R: float, keepouts: KeepoutSet, vg: VisibilityGraph, chi_max) -> tuple[np.ndarray, float]   # (polyline, length_m)
# costs.py
class CostCache:
    @classmethod
    def build(cls, strips, fleet, init_states, sites, wind, terrain, keepouts, vg, cfg) -> "CostCache"
    def refresh_wind(self, wind: WindField) -> None
    def T_matrix(self, uav_id: str) -> np.ndarray               # (n+1, n, 2) s
# allocate.py
def allocate(cells, strips, fleet, states, cache, budget: EnergyBudgetTable, cfg, method: str = "greedy") -> list[Team]
# idp.py
def beam_idp(team: Team, strips: dict, cache: CostCache, budget: EnergyBudgetTable, cfg, warm_start=None) -> dict[str, Sequence]
# corridor.py
def takeoff_profile(site: Site, spec, energy: EnergyModel, wind, terrain, t0: float) -> tuple[list[Waypoint], State]
def landing_profile(site: Site, spec, energy, wind, terrain, arrive_state: State) -> list[Waypoint]
def orbit_climb(state: State, dh_m: float, spec, keepouts) -> tuple[list[Waypoint], State]
# altitude.py
def altitude_profile(path2d: np.ndarray, phases: np.ndarray, h_req: np.ndarray, terrain, spec, airspace, cfg) -> tuple[np.ndarray, dict]   # z (N,), stats
# deconflict.py
def check_separation(plans: dict[str, UAVPlan], cfg) -> list[Conflict]
def repair(plans, conflicts, energy_models, cfg) -> dict[str, UAVPlan]
# plan.py / __init__.py
def plan_mission(inputs: MissionInputs, cfg: PlannerConfig = PlannerConfig()) -> MissionPlan
def replan(plan: MissionPlan, event: Event, telemetry: dict[str, Telemetry], inputs: MissionInputs, cfg) -> MissionPlan
def validate(plan: MissionPlan, inputs: MissionInputs, cfg) -> ValidationReport
```

---

## 16 資料流與流程圖（對應論文 Fig.6）

### 16.1 文字流程圖

```
┌─ 輸入 (表 2) ───────────────────────────────────────────────────────────────┐
│ 起降點 WGS84 │ 區域多邊形+感測需求+優先度 │ DEM/DSM │ 障礙物 │ 分層風 │ 各架 AUW/Wh │
│ 酬載需求 │ C2 鏈路 │ 空域 │ 氣象門檻+觀測 │                    平台表 1 (A/B)   │
└──────┬──────────────────────────────────────────────────────────────────────┘
       ▼
[0] PREFLIGHT ── NO_GO ──► FeasibilityReport (終止)
       │ GO
       ▼
[1] 初始化：GeoFrame(H) → ENU；TerrainGrid(DEM+障礙)；WindField；KeepoutSet(禁飛+高障礙 buffer)；
    VisibilityGraph；EnergyModel[i]；Sensor[i]；起降走廊可行性；takeoff schedule t_i^0
       ▼
[2] 區域分解：DP 簡化 → 凸分割 → 每胞 (h_op, swath, 方向候選(含風/地形)) → 條帶(A/B) → fit → 殘帶子胞
       ▼
[3] 成本快取：op[cls] (n,2) │ tr[cls] (n,n,2) │ entry[i] (n,2) │ rtb[cls,site] (n,2) │ EnergyBudgetTable
       ▼
[4] 配置 (Alg.1')：J_q → λ_q → 貪婪挑機 (可行性: 能量/鏈路/感測/半徑) → BALANCE → 團隊(可多胞)
       ▼
[5] Beam-IDP (Alg.2')：每團隊；k-nearest；min-max 階段成本；能量剪枝 → 序列 [(strip,dir)]、E_i、t_i
       ▼
[6] RTB 附加＋能量修復 → leftover → (回 [4] 僅對 leftover) / 多架次 / uncovered 報告
       ▼
[7] 2D 路徑組裝：起飛走廊 → (盤旋爬升) → 進入 Dubins → 條帶/轉移 Dubins (Eq.28/29 離散) → RTB → 降落走廊
       ▼
[8] 高度剖面：resample → 地形 → 坡度包絡 → 空域上限 → GDA 平滑 → 夾持 → 刪減；AGL 統計 → (兩趟法回 [2] 局部)
       ▼
[9] 時間戳＋能量積分＋逐航點備降可達＋lost-link 動作 → 4D 去衝突 (分層/時間) → 走廊排程
       ▼
[10] 輸出 MissionPlan：每機 Waypoint[] (ENU+WGS84, mode{HOVER,TRANSITION,CRUISE}, phase, t, Wh) ；makespan；
     FeasibilityReport；GeoJSON / QGC WPL；timing_ms
       ▲
       │ 事件 (UAV_EXIT / NEW_AREA / LOW_ENERGY / LINK_LOSS / WIND_UPDATE / CUE / WEATHER_ABORT)
[R] REPLAN：快照 → 受影響集合 → 凍結其餘 → [4]-[9] 增量 → diff 下傳
```

### 16.2 資料流不變量

- 所有幾何在 `GeoFrame` 建立後只以 ENU 存在；`Waypoint` 同時保存 ENU 與 WGS84（一次反轉）。
- `CostCache` 只依 `(strips 版本, 風版本, 初始狀態版本)`；三者任一變更才重建對應層。
- `MissionPlan` 不可變（frozen）；`replan` 產生新物件並附 `diff`。

---

## 17 驗證計畫與 pytest

### 17.1 驗證項目（validate.py）

1. **覆蓋完整性**：對每區以 `cov_raster_m`=5 m 柵格化；每架每個 `sensor_on` 航段依實際 AGL 算 swath（相機另算 footprint 沿航向重疊）→ 掃過多邊形（shapely `buffer(swath/2, cap_style=flat)`）union → 柵格覆蓋率 = 區內被覆蓋格數/總格數；要求 ≥ 99.5%（`both` 區兩感測器各自 ≥ 99.5%）；輸出未覆蓋格 GeoJSON。
2. **最小分離**：§11 檢查器重跑，回報最小水平／垂直距離與時刻。
3. **坡度／AGL**：相鄰航點坡度 $\in[-\tan\gamma_d^{\max},\tan\gamma_c^{\max}]$（用 max 值驗、planning 值規劃）；AGL $\ge$ clearance 於 dd 取樣；作業段 AGL $\ge h_{\min}^c$；空域上限。
4. **能量與備降**：逐航點 `energy_remaining_wh ≥ reserve + E(→nearest alternate)`；落地餘量 ≥ reserve。
5. **鏈路**：所有航點在 range/polygon 內。
6. **Makespan 比較**：(a) 異構 vs 同構基線（全部視為 B 或全部視為 A，同一演算法）；(b) `legacy_mode` 於同構機隊 vs 本演算法（時間成本、beam）；報告 makespan、總距離、計算時間。
7. **論文情境重現**：以論文 Table 2（3 機、Z1 六頂點、V=230、R=4000、h=1200、w=2200）於平地 `legacy_mode` 執行，路徑長應在 590–650 km 區間（論文 620.9／648.1/606.1/591.3 km）。

### 17.2 pytest 清單

```
tests/test_geo.py
  test_enu_roundtrip_mm_accuracy            # 隨機點 WGS84→ENU→WGS84 誤差 <1e-3 m
  test_enu_curvature_drop_50km              # 50 km 處 u ≈ −196 m
tests/test_specs.py
  test_table1_keys_exact                    # PlatformSpec 欄位集合 == 表 1 鍵集合 ∪ 補充鍵
  test_table1_values_pioneer_hero           # 數值逐一等於表 1
  test_climb_rate_consistency               # V·tan3° ≈ max_fw_climb_rate_mps (5%)
  test_bank_angle_radius_consistency
  test_provisional_flags_present
tests/test_sensor.py
  test_lidar_swath_and_speed_limit
  test_camera_gsd_altitude_and_speed_limit  # h_max = gsd·f/pitch；V_max 公式
  test_trapezoid_fov_matches_paper          # h=1200, α_m=72.7, α_v=45, α_h=76.98 → w≈2200, l≈1000
tests/test_energy.py
  test_energy_usable_distance_matches_table # A 133–145 km、B 79–86 km
  test_auw_scaling_1p5
  test_reserve_and_unvalidated_penalty
tests/test_wind.py
  test_ground_speed_triangle                # 頂/逆/側風
  test_crosswind_infeasible
  test_turn_radius_ground_trochoid          # A, W=6 → 327 m
tests/test_decompose.py
  test_concave_L_split_into_two_convex
  test_polygon_with_hole_split
  test_strip_count_and_coverage_band        # 條帶 union ⊇ cell
  test_direction_prefers_min_width_no_wind  # 退化為論文
  test_direction_changes_with_wind
  test_residual_band_assignment
tests/test_dubins.py
  test_parallel_uturn_closed_form           # d≥2R: πR+(d−2R)
  test_six_cases_vs_bruteforce
  test_obstacle_aware_avoids_keepout
tests/test_costs.py
  test_cost_matrix_shape_and_binary
  test_wind_refresh_keeps_geometry_layer
tests/test_allocate.py
  test_sensor_requirement_hard_constraint   # rgb 區不派 A
  test_energy_infeasible_uav_not_assigned
  test_lambda_sum_equals_fleet
  test_balance_reduces_makespan
tests/test_idp.py
  test_single_uav_sequence_alternates_direction   # Eq.24–26
  test_each_strip_once_all_covered                # Eq.20–21
  test_beam1_paper_metric_equals_reference_dp     # 小例 (n=6, λ=2) 與完整列舉相同
  test_knearest_within_2pct_of_full_enum          # n=12, λ=2
  test_energy_pruning_leaves_leftover
tests/test_corridor_altitude.py
  test_takeoff_profile_modes_and_agl        # HOVER→TRANSITION→CRUISE
  test_orbit_count_for_climb_deficit        # Δh=100 m, entry 500 m → n_orb
  test_envelope_never_below_terrain_plus_h
  test_envelope_slopes_within_limits
  test_ridge_steeper_than_3deg_agl_cap_flag
  test_smoothing_then_clamp_keeps_constraints
tests/test_deconflict.py
  test_layering_A_above_B
  test_detect_and_repair_crossing
  test_corridor_exclusive_slots
tests/test_replan.py
  test_uav_exit_reassigns_partial_strip
  test_new_area_incremental_keeps_unaffected
  test_wind_update_refreshes_time_layer_only
  test_cue_dispatches_class_B
  test_energy_deviation_updates_kappa
tests/test_validate_scenario.py
  test_example_scenario_coverage_ge_99p5
  test_example_scenario_all_checks_pass
  test_example_scenario_runtime_under_1s     # 8 機/3 區/~80 條帶（初始化除外）
  test_legacy_mode_reproduces_paper_table2_range
  test_hetero_makespan_le_homogeneous_baseline
```

---

## 18 示例情境（examples/scenario_hills.py）

- 原點 $H$：(23.9500°N, 120.9000°E)，海拔由 DEM 取；備降 $A_1$：(23.9800, 120.9350)、$A_2$：(23.9250, 120.9450)。
- 合成 DEM：20 km × 16 km、10 m 步長；$z=350+120\sin(x/1800)\cos(y/2500)+\text{ridge}$，ridge 沿 NE–SW 高 220 m、半寬 900 m（最大坡度 ≈ 7° > 3°），穿過 $Z_1$ 東緣。
- 區域：
  - $Z_1$：六邊形約 4.5 km × 3 km（13 km²），`sensor_req="lidar"`，priority 1.0，距 $H$ 9 km（A 團隊）。
  - $Z_2$：凹 L 形（2.2 km × 2.0 km，缺角 1.0 × 0.9 km），`sensor_req="rgb"`，priority 0.8，距 $H$ 4 km → 凸分割為 2 胞（B）。
  - $Z_3$：矩形 1.6 km × 1.4 km，`sensor_req="both"`，priority 1.2，距 $H$ 6 km → A、B 各一套條帶，分層 30 m。
- 機隊：A1、A2（Pioneer，AUW 24.6，可用 1350 Wh，`nominal 1500`），B1–B3（HERO，AUW 6.65，可用 380 Wh，`nominal 420`）。感測：A LiDAR (fov_usable 70°, 600 kHz, range 450 m, 20 pt/m²)；B 相機 (f 16 mm, 23.5×15.6 mm, 3.9 µm, 1 Hz, GSD 0.05 m → h_max 205 m，故 $h_{\text{op}}^B=\min(205,120)=120$… 受空域 120 m AGL 限制取 120；A 受同限 → $h^A_{\text{op}}=120$，於 $Z_3$ 分層則 A 需 150 m 超過空域上限 → 自動改用時間分離並記錄 warning)。
- 障礙：電塔 point (高 60 m) 於 $Z_2$ 內；輸電線 line (35 m) 橫越 $Z_1$ 西側；建築群 polygon (25 m) 近 $H$ 東 1 km；禁飛區小矩形於 $Z_1$ 北 2 km。
- 風：層 1（地面–200 m AGL）4 m/s from 240°；層 2（200–800 m MSL 以上）6 m/s from 250°；作業層位於層 2 之下緣 → 條帶方向評估對比 6 m/s。氣象觀測：平均 5 m/s、陣風 8、能見度 8 km、無雨、28 °C → GO。
- C2：range 25 km，地面站 = $H$；空域 max_agl 120 m、terrain_clearance 30 m；lost_link_policy `continue_then_rtb`。
- 預期輸出（figures/）：`scenario_2d.png`（胞、條帶、每機路徑、走廊、盤旋）、`profiles.png`（每機高度 vs 地形，山脊處包絡）、`gantt.png`（相位時序、走廊佔用）、`coverage.png`（柵格覆蓋率）、`validation.json`。預期：makespan ≈ 55–70 min，A1 承擔 $Z_1$ 東半（含山脊，方向沿等高線），A2 承擔 $Z_1$ 西半＋$Z_3$ LiDAR，B 三機分 $Z_2$ 兩胞與 $Z_3$ 相機；異構 makespan 低於全 B 基線（B 續航不足 $Z_1$）與全 A 基線（A 轉彎半徑大於 $Z_2$ 凹角胞寬度導致轉移時間增加）。
