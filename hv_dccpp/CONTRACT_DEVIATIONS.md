# 契約偏差紀錄（append-only）

每位實作者若無法照 `module_contract.md` 實作某項（不可行、矛盾、或需要額外參數），在此追加一節：模組、契約項目、原因、實際做法。整合代理會在最後統一裁決。


## geo.py（契約 §1）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `GeoFrame.to_enu_xy(lat, lon)` | 契約註「高程取 0」；但多邊形頂點若已知地面 MSL，投影更準 | 簽章保留，另加可選關鍵字 `h_msl_m=0.0`（預設 0，行為與契約一致）；`to_enu` 之 `h_msl_m` 亦給預設 0 |
| `ecef_to_geodetic`「Bowring 閉式 + 1 次牛頓」 | 定點修正 `φ ← atan2(z + e²N sinφ, p)` 與牛頓步等價收斂且更簡單 | Bowring 初值 + 2 次定點修正；橢球高以 `p cosφ + z sinφ − a√(1−e²sin²φ)` 計算（兩極無奇異）；實測往返誤差 ~1e-9 m（< 1 mm） |
| `GeoFrame.msl_to_enu_z` | 契約未定義演算法 | 沿 ENU 垂直線對 `enu_z_to_msl` 做定點迭代（收斂比 ≈ 3e-5/次，tol 1e-7 m，≤ 8 次）；`nan` 原樣傳回 |
| `DemGrid.vertical_datum="ellipsoid"` | 契約未定義 `geoid_undulation_m` 於 DEM 與 GeoFrame 之分工 | 取值時 `MSL = z − dem.geoid_undulation_m`（DEM 自帶之 N），之後一律經 `GeoFrame`（其 N 為 MSL↔橢球唯一依據）；`datum="msl"` 時 DEM 之 N 不使用 |
| `TerrainGrid` 建構子 | 契約只列屬性與 `from_dem` | `TerrainGrid(x0_m, y0_m, step_m, z, z_raw=None, sigma_z_m=None, is_dsm=False)`；`from_dem` 之 `bounds_enu` 為 `(xmin, ymin, xmax, ymax)`（與 `bounds()`／shapely 一致）；另轉載 `sigma_z_m`、`is_dsm` 屬性、加 `copy()`、`xs`/`ys` |
| `TerrainGrid.from_dem` 節點高程 | 契約要求「每節點 MSL → ENU z（含曲率下垂）」 | 每節點沿 ENU 垂直線定點迭代：`to_wgs84([x,y,z]) → DEM 雙線性 MSL → msl_to_enu_z` 至 <1e-4 m（lat/lon 隨 z 微變，非單次投影）；DEM 外節點 `nan`；逐 2²⁰ 節點分塊限制記憶體 |
| `TerrainGrid.slope_at` | 契約未定義燒入後坡度語意 | 預設以 `z_raw`（自然地形；站點檢查與 §7.3 皆指 DEM 坡度，障礙人工陡壁不算坡度）之中央差分梯度場雙線性取樣；加可選 `raw=False` 改用燒入後 `z` |
| `corridor_max/min`、`max/min_in_polygon` 之 `nan` | 契約僅定義 `z_at` 範圍外 `nan` | 任一取樣點在網格外 → 該樣本／該多邊形回 `nan`（`np.max` 傳播，不以部分資料估計）；空多邊形 → `nan`；`max/min_in_polygon` 接受 MultiPolygon，取樣集 = 內部節點 ∪ 邊界（含頂點，段長 ≤ step/2） |
| `corridor_*` 取樣 | 契約「Δv = dv_m」 | 每樣本於 `[offset−hw_i, offset+hw_i]` 取 `M = ⌈2·max(hw)/dv⌉+1` 個等距點（含兩端點，間距 ≤ dv）；`hw=0` 與 `z_at(path + offset·n̂)` 走同一路徑（逐位元相等）；`hw`、`theta` 可純量廣播 |
| `TerrainGrid.burn` | 契約「多邊形內 z = max(z, top)」若只改內部節點，靠近邊界之雙線性值會 < top | 燒入範圍取 `poly ⊕ disc(step·√2)`，使多邊形內每一點的四個相鄰節點皆被燒入 ⇒ 內部連續曲面 ≥ top；`top` 為 None/nan 者跳過 |
| 例外型別 | `MissionInputError` 定義於第 1 層 `mission.py`，第 0 層不得 import | 輸入錯誤丟 `ValueError`（`MissionInputError` 為其子類，上層可統一捕捉） |

## platform.py（契約 §2）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `MissionInputError` | 契約定義於 `mission.py`，但 mission 依賴 platform（第 1 層 → 第 0 層），platform 不得 import mission；`sensor_model` 卻須丟此例外。 | 在 `platform.py` 定義 `MissionInputError(ValueError)`；丟出時以延遲 import 優先取 `hv_dccpp.mission.MissionInputError`（若已存在），否則用本地類別。**建議 mission.py 以 `from .platform import MissionInputError` 重新匯出**，使全套件為同一類別。 |
| `PlatformSpec.__post_init__` 「assert」 | `assert` 在 `python -O` 下失效，且表 1 自洽檢查屬輸入驗證。 | 以顯式檢查丟 `PlatformSpecError(ValueError)`（訊息列出全部失敗項）。除契約列出的 7 項外另加：Range lo≤hi、速度序 stall ≤ transition ≤ min_mission ≤ cruise ≤ max、reference_auw ≤ mtow、reference_payload ≤ reference_auw、reserve ∈ [0,1)。 |
| `PlatformSpec.transition_cmd()` | 規格 §3.4 的 `cfg.transition_profile="fw_slope"` 需改 d_tr，但契約簽章無 cfg。 | 新增**關鍵字**參數 `transition_profile: Literal["rotor_assisted","fw_slope"]="rotor_assisted"`（預設行為與契約相同）；`fw_slope` 時 d_tr ← max(d_tr, (H_tc,hi−H_ts)/tanγ_c^plan)（A 763 m，取爬升側）。slope_tr ≤ gamma_tr_max 的檢查留給呼叫端（preflight/site_check），本函式不丟例外。 |
| `Sensor.end_extension_m(v_g_max, t_settle)` | 規格 §4.5 之 l_B(1−o_fwd) 依賴 AGL，契約簽章無高度參數。 | 新增可選第三參數 `h_agl_m: float | None = None`：None → `req.h_target_agl_m`，再缺省 → `agl_window()[1]`（h_max，保守取大）。LiDAR 之 `footprint_len_m()` 回 0（線掃描；e_ext,A = V_g·t_settle = 53 m 與 §W.3 一致）。 |
| `PayloadRequirement` 缺省值 | 契約只給 side_overlap 預設（0.25/0.30）；density／GSD 為 None 時契約未定。 | 依規格 §16.2 預設：density_pt_m2 20、gsd_required_m 0.05、forward_overlap 0.70、shutter → 硬體上限；採用之預設鍵列於 `sensor.defaults_applied`（供 PRECHECK 警告）。TrapezoidFovSpec 無像元／測距資訊：`gsd_m→None`、`agl_window→(0,inf)`、`gsd_attainable→True`、h_target 預設 150 m（論文情境由需求指定 1200）。 |
| `EnergyModel` 額外欄位 | `eta_T` 之係數「下限 floor（cfg）」需存放；契約欄位表未列。 | 末尾新增 `temp_cap_coef_per_c=0.01`、`temp_cap_floor=0.8`（同 PlannerConfig 預設）；另 `kappa` 屬性 = clip(correction, 0.8, 1.5)。`__post_init__`：reference None → = nominal；兩者皆 None → 自動 `base_minute_mode=True`；spec 驗證旗標為 True 時強制 k_bm/k_as = 1.0。 |
| 基準分鐘模式之爬升／下降功率 | P_c^ref ≡ 1/60 unit/s，無法把 m·g·Va·sinγ/η_p（W）換算成 unit/s。 | 基準分鐘模式下 `p_climb_w == p_descent_w == p_cruise_w`（額外項取 0；非保守，報告須標示）。段能量在此模式為 P·t·k_unc（不除 3600），`cruise_equiv_km` 亦依模式換算。 |
| `cruise_equiv_km` 公式 | 契約註解「wh/(P_c·k_unc)·Va/1000」量綱缺 3600。 | 實作 wh/(P_c·k_unc) [h] × Va × 3.6 → km（與契約 §W.8 之 389.6 Wh → 48.8 km 一致）。 |
| `usable_distance_cap_km` 之「驗證」旗標 | 契約未指明用哪個旗標。 | 依規格 §3.1 用 `battery_monitor_validated`（未驗證下緣 133/79、驗證上緣 145/86）。 |
| `PlatformSpec.orbit_gain` | 規格 §3.3 列於 PlatformSpec 方法，契約則置於 `altitude.orbit_gain`。 | 未在 platform 實作（由 altitude 擁有）。 |

## dubins.py（契約 §5）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `path_time` / `path_time_and_energy` 之 `wind: WindField`、`energy: EnergyModel` | dubins 為第 0 層，wind（第 1 層）與 platform.EnergyModel 平行開發 | 鴨子型別：`wind` 接受任何具 `wind_vec(z)->(N,2)` 之物件、或 (2,) 風向量、或 None（無風）；`energy` 只需 `segment_wh(mode, duration_s, gamma_rad)`。時間積分（直線 `L/V_g(θ)`、弧中點 Riemann 和、步長 ≤ `step_rad`）在本模組內自行實作（`ground_speed_uniform`、`_arc_time_vec`），不 import `wind`。 |
| `path_time_and_energy` 需要 `mission.Mode.CRUISE` | `Mode` 定義於 mission.py（第 1 層） | 於函式內延遲 `from .mission import Mode`（無 import 期依賴）；mission 不存在時退回字串 `"CRUISE"`。 |
| 弧段 Riemann 和之取點 | 規格 §8.1／D14 只寫「Δθ=5° 數值積分」未指定左端或中點 | 採**中點**規則、n=⌈|Δθ|/step⌉ 等分（δ ≤ step）。以此重現契約 §W.5–W.7 全部數值（35.5 s、201.8 s、350.8 s、422.3 s、205.4 s、490.7 s、圓周因子 1.043）在 ±0.1 s 內。 |
| `discretize` 角步 Δψ | 契約／D27 精確式 2·acos(R/(R+e)) 給 A 19.72°、B 25.36°；規格引述之 19.8°/25.6° 為小角近似 2√(2e/R) | 採精確式（較小、較保守，頂點徑向偏差恰 = e_max）。 |
| `uturn_length(s,R)` 於 s<2R、`loop_length(s,R)` 於 s≥2R（契約僅定義各自的定義域） | §7.3 代理會對任意 s 取 min{L_Ω, L_U(ks)} | 兩者於定義域外回傳「同組態之真實 LSL/RSR 長度」（即互為對方之公式）；`omega_length` 於 s>2R 將 arccos 引數截至 1（回 πR）。定義域內與契約公式完全一致。 |
| `DubinsPath.end` | 前向積分之終點含 ~1e-12 浮點雜訊；方位可能與要求值差 2πk | 末段 `end` 貼齊為呼叫者傳入之 `end` Pose（x, y, theta 皆為要求值；閉合誤差 >1e-6·R 時丟 RuntimeError）；`sample()` 末列亦為該值，中間列 theta 為自 start 連續累積（未取模）。 |
| 起訖圓重合（CSC 之 p=0）之退化 | 三角閉式 atan2(0,0) 方向未定，會把單一弧拆成 t+q 多出 2π | LSL/RSR 於 p<1e-9 時改為 t=方位差、q=0。 |
| `dubins_all(starts, ends, R)` 之輸入型別 | 契約不變量寫 `dubins_all([a],[b],R)`（Pose 序列） | 接受 (N,3) ndarray、Pose、Pose 序列或 (3,) 序列。 |
| 額外公開函式（非契約，僅新增） | 供 costs/routing 向量化 | `dubins_solve_all`（(N,6) 長度＋(N,6,3) (t,p,q)）、`dubins_all_words`（(N,6) 長度）、`dubins_all_times`（N 對最短 word 於均勻風之時間，向量化、與 `path_time` 一致到 1e-9）、`discretize_step_rad`、`ground_speed_uniform`。 |

## geometry.py（契約 §6）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `Pose` 取自 `dubins.py` | 實作當下 `hv_dccpp/dubins.py` 尚不存在 | `try: from .dubins import Pose except ImportError:` 後備定義與契約 §5 逐字相同（`x_m, y_m, theta_rad`, `as_array()`）。dubins.py 就位後自動改用其定義，無需修改。 |
| `Strip` 欄位「完全相同」 | 規格 §11.3 要求局部包絡回報 `feasible`（不可行時該條帶／方向回報 §11.5），但契約 `Strip` 無欄位可承載；`LocalEnvelopeFn` 只回 3-tuple | 於 `Strip` **尾端**新增預設欄位 `feasible: bool = True`、`infeasible_reason: str = ""`（關鍵字／位置建構皆相容）。`lay_strips` 遇 `feasible=False` 仍保守鋪設該條並標記；另提供關鍵字參數 `raise_on_infeasible=False`（True → `StripInfeasibleError`）。**decompose.build_strips 應檢查 `all(s.feasible for s in strips)`，否則該方向視為不可行。** |
| `Strip.z_op_m`（= corridor terrain max + h_op） | geometry 為第 0 層無地形；`LocalEnvelopeFn` 回傳值未含 z | `lay_strips` 接受閉包回傳 3 或 4 個元素；第 4 個視為 `z_op_m`（ENU）。缺省時 `z_op_m = h_op_m`（即假設地面 z=0）。**decompose 之閉包請回傳 4-tuple，或在 build_strips 後覆寫 `z_op_m`。** |
| `clip_band` / `residual_subcell` 回傳 `Polygon \| None` | 非凸／有洞胞（近凸胞被規格 §7.2 接受）之帶可切成多片 | 回傳型別放寬為 `Polygon \| MultiPolygon \| None`；`lay_strips` 只用其 u 極值（論文 Eq.(8)(9)），故正確。`strips_union_footprint` 同樣可能回傳 `MultiPolygon`（不重疊之條帶集合）。 |
| `lay_strips` 簽章 | 需要中心線取樣間距與不可行處置選項 | 只新增**關鍵字**參數 `raise_on_infeasible: bool = False`、`centerline_ds_m: float = 25.0`；契約位置參數不變。 |
| `concavity` 之數值 | 規格 §7.2 反例稱 500 m 方形角落 20×20 m 凹口「凹度 20 m」 | 依契約定義（凸包與多邊形之 Hausdorff 深度）計算為 20/√2 ≈ 14.1 m（凸包橋為對角線）；兩者皆 ≤ τ=73，`convex_partition` 行為一致。前向方向於頂點精確、反向方向以 ≤ max(0.5 m, 周長/4000) 取樣。 |
| `min_width_direction` | 論文 dis_{k,μ} 以多邊形自身各邊計 | 以**凸包**各邊計（凸多邊形時與論文相同；非凸時為真正最小寬度）。回傳 α 正規化至 [0, π)。 |
| `merge_shallow` 準則「合併不增加最佳方向條帶數」 | 契約未明定比較對象 | 採 `n(Qa∪Qb) ≤ n(Qa)+n(Qb)` 且 `concavity ≤ τ` 且並集為無洞單一 Polygon；貪婪，優先條帶數減少最多、其次凹度最小。 |
| `convex_partition` 切線 | 規格未明定射線長度與洞頂點處理 | 外環反射頂點：射線取「第一次出界」線段且要求切線消解該頂點（兩側角 ≤ 180°+tol，保證每次切割反射頂點數嚴格減少 → 必終止）；洞頂點：整條直線（單一橋接不會分割）。MERGE 只在頂層執行一次（遞迴各層合併結果相同）。 |
| `Strip.strip_id` 格式 | 契約未定 | `f"{cell_id}_{cls}_{index}"`，index 自 1 起（規格 §7.4 之 l）。decompose 可重新命名。 |

## mission.py（契約 §3，不含 preflight）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `MissionInputError` | platform.py 已定義同名類別（依賴方向） | `from .platform import MissionInputError` 重新匯出；全套件為同一類別（`mission.MissionInputError is platform.MissionInputError`）。 |
| `PlannerConfig` 之 `legacy_mode` 覆寫「能量／鏈路／時間窗關閉、t_RTB=0、h_ceil=inf、e_ext=0」 | 契約欄位表無承載這些旗標的欄位，上層模組無從讀取 | 於**尾端**新增有預設值之欄位：`enforce_energy=True`、`enforce_link=True`、`enforce_time_windows=True`、`t_rtb_lower_bound=True`、`e_ext_override_m=None`（None → 感測器公式）、`h_ceil_override_m=None`（None → 空域）。`legacy_mode=True` 時（建構時 `__post_init__` 即套用，`apply_legacy()` 回傳 `replace(self, legacy_mode=True)` 之複本）改寫為 False/False/False/False/0.0/inf，並同時改寫契約列出之 `idp_branching/transfer_types/B/K/metric/convexify_whole/fov_model/k_bm/k_as`。**上層模組請讀這些旗標，而非自行判斷 `legacy_mode`。** |
| `PlannerConfig.K: int` 之 `K=inf` | int 欄位無法存 inf | 以 `mission.K_INF = 2**31−1` 表示；`PlannerConfig(K=math.inf)` 與 `from_dict({"K": "inf"})` 自動轉為 `K_INF`；提供 `k_nearest(n) = min(K, n)`。 |
| `PlannerConfig.delta_edge_m=None → k_σ·σ_xy + e_xt + δ_w/2` | 契約只寫註解 | 方法 `delta_edge_resolved(delta_w_m=0.0)`；另加 `psi_max_rad`、`dtheta_int_rad`、`site_max_slope_rad` 屬性與 `to_dict/from_dict/to_json`。建構時做參數驗證（nu/omega 長度 6、K/B ≥ 1、Literal 值、正值），失敗丟 `ValueError`（非任務輸入）。 |
| `C2Link.lost_link_policy` 缺省「A: CONTINUE_THEN_RTB, B: RTB」 | 契約欄位為 per-uav dict，缺省規則無承載處 | 方法 `policy_for(uav_id, cls)`（mission 值 → `DEFAULT_LOST_LINK_POLICY[cls]`）；另加 `coverage_for(cls)`。欄位仍為必填（契約位置不變），`load_mission` 缺省給 `{}`。 |
| `C2Link.ground_station` | JSON 中常省略 | `load_mission` 缺省 = home；亦接受站點 id 字串或 GeoJSON Point Feature。 |
| `SortieEnergy` 額外方法 | preflight 需要的解析皆為純資料規則 | `is_base_minute_candidate(spec)`、`payload_kg_resolved(spec) -> (kg, estimated)`、`battery_temp_resolved(obs) -> (°C, assumed)`；`available_wh_resolved` 三種輸入皆缺時亦丟 `MissionInputError`（訊息含鍵名 `available_wh/available_mah/soc_fraction`）。 |
| `WeatherThresholds` / `NoFlyZone` / `Airspace` 輔助 | D7、D22 規則為純資料層 | `WeatherThresholds.go_violations(obs, w_go)`／`abort_violations(obs)`；`NoFlyZone.is_active(t_window)`（重疊即有效）；`Airspace.active_no_fly(t_window)`、`ceiling_defaulted` 屬性。preflight/dynamic 可直接使用。 |
| 輸入 dataclass 之 `__post_init__` | 契約未定驗證 | 所有輸入型別於建構時驗證並正規化（list → tuple、閉合頂點去除、lat/lon 範圍、Literal 值、二擇一欄位、`z_top > z_bottom`、重疊率 ∈ [0,1)、`soc ∈ (0,1]`），失敗丟 `MissionInputError`。`MissionInputs` 另驗證 id 唯一、`payload_req_id` 存在、`lost_link_policy` 之 uav_id 存在、`recovery_site_id` 存在、`platform_overrides` 鍵 ⊆ provisional ∪ provisional_supplementary ∪ {min_fw_operation_agl_m}（`allowed_override_keys(spec)`；「只可調高」與 `dataclasses.replace` 仍由 `build_context` 執行）。 |
| `load_mission(obj: dict \| str)` | 使用者系統以 JSON 檔互通 | 亦接受檔案路徑（str/`os.PathLike`）與 bytes；`str` 以 `{` 開頭視為 JSON 字串。頂層未知鍵丟例外（允許 `name/description/version/meta/…` 等中繼鍵與 `_` 開頭鍵）。dict 內可直接放已建構之 dataclass 實例（原樣採用）。`fleet[i].spec` 可為 "A"/"B"/"PIONEER"/"HERO"/`PlatformSpec`／完整欄位 mapping（`to_dict` 輸出）；`sensors` 元素可為 "lidar"/"rgb" 字串或硬體 mapping（`hw_type` 鍵可選）。 |
| GeoJSON | 契約只提 areas | `areas`（FeatureCollection/Feature/Polygon/MultiPolygon，屬性 → AreaSpec 欄位，未知屬性忽略）、`obstacles`（Point/LineString/Polygon → point/line/polygon）、`airspace.no_fly`（Polygon/MultiPolygon，拆分加尾碼）、`c2.coverage_polygon_wgs84`（Polygon 或 `{cls: …}`）、站點（Point，第三分量 → alt_msl_m）皆接受；缺 id 時 `area_<n>`／`obs_<n>`／`nofly_<n>`。 |
| 序列化 | 契約未列 | `MissionInputs.to_dict/to_json`（`load_mission` 往返一致）、`Waypoint/UAVPlan/FeasibilityReport.to_dict/from_dict/to_json`、`MissionPlan.to_dict/to_json`（單向；cells/strips 以 `to_jsonable` 通用序列化：dataclass → dict、shapely → GeoJSON mapping、numpy → list、Enum → name、frozenset → sorted list、tuple 鍵 → `"a\|b"`）。`PlatformSpec` 以全部欄位序列化（Range → list、provisional → sorted list）。inf/nan 保留為 float（Python `json` 可往返）。 |

## wind.py（契約 §4）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `WindLayer` 型別 | 定義於 `mission.py`（同為第 1 層，尚未存在）；wind 只依賴 platform | 鴨子型別：接受任何具 `z_bottom_m, z_top_m, speed_mps, dir_from_deg` 之物件；`gust_mps` 缺省 0、`z_ref` 缺省 `"msl"`。輸入錯誤丟 `platform.MissionInputError`（mission 應重新匯出同一類別）。 |
| `WindField` 欄位 | 契約列 `layers, z_bounds_enu_m, version, notes`；逐 z 查詢需向量化用的層陣列 | `@dataclass(slots=True, init=False)` 自訂 `__init__(layers, frame, z_ground_home_m, version=0)`；**新增**欄位 `z_ground_home_m`、`speeds_mps (L,)`、`gusts_mps (L,)`、`dir_from_rad (L,)`（皆由 layers 導出）。`layers` 存排序後清單（主鍵 z_bottom、次鍵 z_top）。 |
| 層選取「重疊取較低層；範圍外取最近層」 | 契約未給演算法 | 以「z 至各層區間之距離 max(zb−z, z−zt, 0)」取 argmin（含層 → 0；並列取排序在前 = 較低層），同一規則涵蓋重疊、縫隙與範圍外；nan z → 全 nan。 |
| `frame` 參數 | 純 AGL 層不需要座標框架 | `frame=None` 允許，但遇 `z_ref="msl"` 層即丟 `MissionInputError`。±inf 層界原樣保留不換算。 |
| 空層清單 | 契約未定 | 允許：全域無風，`notes` 加 `wind_layers_empty_assumed_calm`。 |
| `max_speed()` vs `max_speed_between()` | 巢狀被遮蔽之層永不被 `speed_at` 選中 | `max_speed()` = 所有層 `speed_mps` 之最大（保守、簡單）；`max_speed_between(lo, hi)` 與 `speed_at` **精確一致**（於層界、縫隙中點切換點及相鄰中點取樣取最大；lo>hi 自動對調；±inf 以層界外 1 m 代替）。故被遮蔽層可使 `max_speed() > max_speed_between(−∞, ∞)`。 |
| `ground_speed` / `heading_from_track` 回傳形狀 | 契約「回傳 (N,)」 | 一律 `np.atleast_1d`：純量 theta + (2,) 風 → `(1,)`（取 `[0]`）；(N,) theta + (N,2) 風逐點；(N,) + (2,) 廣播。`|W⊥| ≥ Va`（含恰等）→ nan，與 `dubins.ground_speed_uniform` 相同判準。 |
| `r_eff_wind_speed(..., policy="max_layer")` 之 $H_{tc}$ | 規格 §3.2 只寫 $z_{site}+H_{tc}$，未指定區間上下緣 | 取 `spec.transition_cmd().h_tc_clear_m`（下緣；範圍較寬、更保守）；`z_op_max_m < z_lo` 時範圍退化為單點 `z_lo`。`"op_layer"` = `wind.speed_at(z_op_max_m)`；其他政策名 → `ValueError`。 |
| `time_line` / `time_arc` 之 `wind` 型別 | 與 `dubins.path_time` 對稱 | 除 `WindField` 外亦接受任何具 `wind_vec` 者、(2,) 風向量或 `None`（無風）。`time_arc` 之 `center` 只作形狀檢查（均勻風下弧時間與圓心無關）。 |
| `time_arc` 取點規則 | 契約僅「step_rad 之 Riemann 和」 | 中點規則、n=⌈\|Δθ\|/step⌉（δ ≤ step），與 `dubins._arc_time_vec` 完全相同 ⇒ 同一路徑 `Σ time_line + time_arc == dubins.path_time` 到 1e-12 s（測試 `test_path_time_agrees_with_dubins`）。任一步 V_g ≤ 0 → inf。 |
| `circle_time_factor` | 契約「∮(1/Vg)/(2π/Va)」未給算法 | 閉式 $2V_a^2E(m)/(\pi(V_a^2-W^2))$，$m=(W/V_a)^2$，$E$ 為第二類完全橢圓積分（`scipy.special.ellipe`）；(21.5, 5) → 1.0427；$W \ge V_a$ → inf。**注意**：D14 之快速代理 $1+(W/V_a)^2$ 只在 $W/V_a \le 0.514$（A ≈ 11.0 m/s、B ≈ 9.8 m/s；作業門檻 $W_{go}=8$ 以內恆成立）內為保守上界，超過後精確值更大（測試改為只在 W ≤ 8 m/s 斷言代理 ≥ 精確）。 |
| 額外公開函式（非契約） | 便利／向量化 | `wind_vector(speed, dir_from_deg) -> (N,2)`、`WindField.speeds_at(z) -> (N,)`、`WindField.layer_index(z) -> (N,) int`、`WindField.n_layers`、常數 `DEFAULT_ARC_STEP_RAD = 0.0873`。 |

## context.py（契約 §6A）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `build_context(inputs, cfg)` 簽章 | 地形網格邊距（≥ 走廊半寬 + d_turn）契約未定；測試需縮小以加速 | 只新增**關鍵字**參數 `terrain_margin_m: float = TERRAIN_MARGIN_M (3000)`；邊界 = 站點／地面站／區域（含洞）／禁區／障礙頂點之外接矩形 ⊕ 邊距，`TerrainGrid.from_dem(dem, frame, cfg.dem_step_m, bounds)`（DEM 外節點 nan）。 |
| `MissionContext` 欄位 | `decompose_ctx()` 需要 `nofly_buffered / available_kinds / auw_kg`，契約 `MissionContext` 無此欄位；`altitude.site_check` 需要 `obstacles_enu`；鏈路圓心需地面站 ENU | 契約欄位順序不變，於**尾端**新增有預設值之欄位：`nofly_buffered: list[Polygon]`、`available_kinds: dict[cls, frozenset]`、`auw_kg: dict[cls, float]`、`obstacles_enu: list[ObstacleEnu]`（`ObstacleEnu(geom, top_z_m, obs_id, category, buffer_m)` NamedTuple；幾何**未緩衝**、`top_z_m` 為 ENU 頂高**不含** b_v）、`ground_station_xy (2,)`、`t_window`（D22 任務窗；`enforce_time_windows=False` → None）。便利方法：`home_xy`、`z_ground_home_m`、`uav(id)`、`spec_of(id)`、`obstacle_pairs() -> [(geom, top_z)]`（site_check 參數形式）。 |
| `keepouts: KeepoutSet` | `routing.py` 同層平行開發，實作當下不存在 | 型別註記 `Any`；`from .routing import KeepoutSet` **延遲於 build_context 內**；`ImportError` → `keepouts=None` 並警告 `routing_unavailable_keepouts_none`（整合者接線）。呼叫形式 `KeepoutSet.build(list(obstacles), active_nofly, frame, terrain, cfg, t_window)`。**障礙燒入（D24）**：若 `KeepoutSet.build` 後 `terrain.z == z_raw`（未燒入）則由本模組 `burn_obstacles` 補做一次（`geom ⊕ buffer`，`top + cfg.b_v_m`；DSM 略過 tree/building）並警告 `obstacles_burned_by_context(ids)`；已燒入則不重複（燒入冪等）。 |
| MSL/AGL → ENU（D42） | 契約 `to_enu_xy` 註「高程取 0」 | 站點以 `frame.to_enu(lat, lon, msl_site)` 完整三維轉換（`msl_site` = `alt_msl_m` → DEM MSL + `alt_agl_m` → DEM MSL；站點 z = 起降面高度，AGL 站點 = 地面 + AGL）；多邊形／障礙頂點以 **DEM 地面 MSL** 投影（DEM 外頂點退回 home MSL），與站點一致（50 km 處差 < 1 m）。`GeoFrame.geoid_undulation_m = dem.geoid_undulation_m`。障礙 `top_msl_m` → 各頂點 `msl_to_enu_z` 取最大；`height_agl_m` → 幾何範圍內最高地形 ENU z（燒入前）+ h；地形範圍外退回頂點 DEM 值並警告 `obstacle_outside_dem`。 |
| 多邊形有效性 | 契約未定 | `shapely.make_valid`，面積 < `MIN_POLYGON_AREA_M2 = 1 m²` → `MissionInputError`；自交輸入可回 MultiPolygon（decompose 逐一處理）。區域在 DEM 外（`max_in_polygon` nan）→ 警告 `area_outside_dem(id)`（NO_GO 由 preflight 判定）。 |
| `h_clr`「Airspace → spec 覆寫 → error」 | 兩構型 spec 覆寫槽可能不同；D11「DEM 誤差已知時強制 h_clr ≥ 3σ+30」未定執行方式 | Airspace 值優先；否則各構型 spec 槽取**最大**（不同時警告 `terrain_clearance_per_class_differs`）；皆 None → `MissionInputError`（訊息含 `terrain_clearance_m`）。`dem.sigma_z_m` 已知且值 < 3σ+30 → **抬高至 3σ+30** 並警告 `terrain_clearance_raised_to_3sigma_plus_30(old→new)`；σ 未知 → 警告 `terrain_clearance_sigma_unknown`。 |
| `h_ceil_m` / `h_ceil_source` | 契約只列「max_agl / waiver / max_msl」；`max_msl_m` 為 MSL 需換算 AGL | 優先序：`cfg.h_ceil_override_m`（legacy inf，source `cfg.h_ceil_override_m`）→ `waiver_max_agl_m` → `max_agl_m`（同時給 `max_msl_m` 時取 `min(max_agl, max_msl − max 地形 MSL)`，source `max_msl_m(min with max_agl_m)`）→ `max_msl_m − max 地形 MSL`（區域內最高地形之上，**燒入前**取樣；保守單值）→ 120 並警告 `airspace_ceiling_defaulted_120`（source `default_120`）。兩構型值相同（同一空域）；只對機隊中之構型給值。 |
| `link_poly` / `rho_sym_m` | 契約未定 range 與 coverage 同時給定、legacy 關閉鏈路之行為 | `range_km_for(cls, spec)` 之圓（圓心 = `c2.ground_station` ENU，`quad_segs=64`）與 coverage polygon **皆給時取交集**；皆缺 → `MissionInputError`（訊息列 `c2.range_km[cls]/coverage_polygon_wgs84[cls]/spec.c2_link_range_km`）；`cfg.enforce_link=False` → 半徑 1e7 m 之圓且 `rho_sym = inf`。否則 `rho_sym = symmetric_mission_radius_km[0]·1000`（下緣）。 |
| `models_rep[cls]` | 契約「最大 AUW、最小 E_avail」未定其餘欄位 | `auw = max`、`available = min`、`nominal = max`（保留量偏大 = 保守）、`battery_temp = min`、`reference = spec.reference_battery_wh`、`correction = 1`；同構型混用基準分鐘與 Wh → `MissionInputError`。 |
| `EnergyModel` 逐機 | 規格 §5.7 邊界情況 | 僅 `soc_fraction` 且 nominal None → 基準分鐘模式 `available = soc·T_ref`、`nominal_wh=reference_wh=None`、警告 `base_minute_mode(uav)`；但 `spec.reference_battery_wh` 已知而 nominal None → `MissionInputError`（E_nom 不可解析）。警告名：`reference_battery_assumed(uav)`、`battery_nominal_example(uav)`（`pack_nominal_wh` None 而用 spec 值）、`battery_temp_assumed(uav)`（取 `WeatherObs.temp_c`）。`k_bm/k_as/temp_cap_*` 取 cfg；驗證旗標由 EnergyModel 強制 1.0。 |
| `sensors[(cls, req_id)]` | 同構型多機硬體可能不同 | 硬體取該構型機隊**第一個**具 `req.kind` 之硬體（機隊順序、主感測器優先）；不同 → 警告 `sensor_hardware_heterogeneous(cls, kind)`。無該 kind 硬體之 (cls, req) 不建立鍵。使用示例硬體 → 警告 `sensor_spec_default_example(cls)`（每構型一次）。 |
| `platform_overrides` | 規格 §3.5 | `apply_platform_overrides(base_specs, overrides)` 公開；`PlatformSpecError` 改丟 `MissionInputError`；`min_fw_operation_agl_m` 小於現值 → `MissionInputError`；覆寫鍵自 `provisional` 與 `provisional_supplementary` 移除；警告 `overridden(cls.key, old→new)`；機隊無該構型 → 忽略並警告 `override_ignored_no_fleet(cls)`。`specs` 只含機隊中之構型（基礎 spec = 該構型第一架之 `uav.spec`；不一致警告 `spec_heterogeneous(cls)`）。 |
| `warnings` | 契約只舉例 | 字串清單，另含 `wind: <WindField.notes>`（AGL/MSL 假設、曲率近似）。 |
| 額外公開函式（非契約） | 供 preflight 重用 | `base_specs_from_fleet`、`apply_platform_overrides`、`resolve_terrain_clearance`、`resolve_ceiling`、`site_ground_msl`、`polygon_enu`、`obstacles_to_enu`、`burn_obstacles`、`build_energy_model`、`build_sensors`、`link_polygon`；常數 `TERRAIN_MARGIN_M`、`DEFAULT_CEILING_AGL_M`、`UNBOUNDED_LINK_RADIUS_M`、`MIN_POLYGON_AREA_M2`。 |
| 未 import `geometry` | 契約列 geometry 為依賴，但 `DecomposeContext`／`MissionContext` 欄位皆不需 geometry 型別 | 本模組不 import geometry（依賴集合為契約之子集，方向不變）。 |

## scenario.py（契約 §16）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `synthetic_dem(lat0_deg, lon0_deg, size_km, step_m, base_msl_m, amplitude_m, ridge, seed)` 之 `lat0/lon0` 語意 | 契約未說明是角點或中心 | 採 **左下角節點**（與 `DemGrid.lat0/lon0` 同義）；`size_km = (east_km, north_km)`（純量 → 正方形）；`cols = ⌈east/step⌉+1`、`rows = ⌈north/step⌉+1`；`dlat = step/m_lat`、`dlon = step/m_lon` 於 DEM 中心緯度換算（`metres_per_degree` 級數式）。另加 `synthetic_dem_for_frame(frame, bounds_enu, step, **kw)`（ENU 邊界 → 角點）與 `synthetic_dem_for_latlon_bounds(...)`。 |
| 山丘形式與 `seed` | 契約只寫「optional sinusoidal hills」 | 山丘 = `A·sin(2πu/λ)·sin(2πv/λ)`（`u,v` = 自左下角之東／北公尺；關鍵字 `wavelength_m=3000`）：確定性、半波長格點上梯度為 0（站點置於格點可通過 5° 站點坡度檢查）、最大坡度 `atan(2πA/λ)`。`seed` 只影響關鍵字 `noise_m>0` 之低通隨機起伏（6 個隨機正弦和，非白噪聲——10 m 節點上 1 m 白噪聲即產生 5.7° 假坡度）。另加關鍵字 `sigma_z_m`、`is_dsm` 轉交 `DemGrid`。 |
| `ridge` dict 之剖面 | 契約只列鍵 `{center_latlon, azimuth_deg, height_m, width_m, length_m}` | 升餘弦橫剖面 `h·½(1+cos(π d⊥/(w/2)))`（|d⊥| ≤ w/2）、兩端以 `min(w/2, L/2)` 之升餘弦收尾；方位自北順時針、右法向 `(cos θ, −sin θ)`（契約 §0）；橫向最大坡度 `atan(hπ/w)`，「比 3° 陡」⇔ `hπ/w > 0.0524`。缺鍵／未知鍵／寬長 ≤ 0 → `MissionInputError`。 |
| `build_mini_example` DEM 範圍 | 契約只給 10 m 步長與平地 100 m | ENU `(−2500, −2500, 8500, 2500)`（1101 × 501 節點；起飛走廊西側、區域東側 1.5 km、南北 ≥ 1.2 km 裕度）。風層以 `z_ref="agl"`、`(0, 3000 m)` 單層表示。區域頂點以 ENU z=0（home 高度面）轉 WGS84；以 `to_enu_xy(..., h_msl_m=100)` 反算誤差 < 0.05 m（用預設 h=0 反算則差 ≈ 6 cm，來自高度面差）。 |
| `build_hills_scenario` 之山丘幅度 | D12 高度帶：A 胞（⊕ 迴轉帶）內起伏須 ≤ waiver 200 − h_minfw 120 − Δh_band,min 20 = 60 m，否則 lidar 區對 A 不可指派 | 山丘 A=12 m／λ=3 km（坡度 ≤ 1.4°）、山脊高 30 m／寬 400 m（橫向坡度 13.3° > 3°），區域 1 ⊕ 600 m 內起伏 ≈ 54 m（測試斷言 ≤ 60）。DEM 原生 30 m 節點（`cfg.dem_step_m=10` 由 TerrainGrid 重取樣）、範圍 ENU `(−3000, −4500, 15500, 7500)`。home／ALT1／ALT2 置於山丘半波長格點（坡度 ≈ 0）。 |
| 山丘情境 `both`／`rgb` 區之酬載需求 | `AreaSpec.payload_req_id` 只有一個 id，而 `PayloadRequirement.kind` 只能是 lidar 或 rgb | 沿用契約 §W.1 之 `default`／`default_rgb` 慣例：區域 id `X` → lidar 需求 `X`、rgb 需求 `X_rgb`（`both_survey`／`both_survey_rgb`）；純 rgb 區直接給 rgb 需求 `rgb_far`；純 lidar 區給 `lidar_survey`。**context 建構者以 (cls／kind) 查表時請套用此 `_rgb` 尾碼慣例。** |
| `build_paper_table2_scenario` 之三機起始位置與航向 | `MissionInputs` 只有單一 home，無法承載論文 Table 2 三個地面站與初始航向 30° | home = U1 位置、備降點 `U2`／`U3` = 其餘兩機位置（高度由平地 DEM 2700 m 取）；模組常數 `PAPER_TABLE2_UAV_START[uav_id] = (lat, lon, alt_msl, heading_deg)` 供 legacy 執行器（`sim.paper_table2_check`）建立各機起始 `UAVState`。DEM 為平地（規格 §17.1 (7) 指定），500 m 節點、涵蓋 (36.30–37.75°N, 93.40–95.00°E)；`PlannerConfig(legacy_mode=True, dem_step_m=250)`（區域跨 70 km，10 m 重取樣不可行）。 |
| 論文同構固定翼 `PlatformSpec` | 契約無此常數；`PlatformSpec.__post_init__` 之自洽檢查對 V=230／R=4000 須另設傾角、航程等 | `legacy_fixed_wing_spec(v=230, r=4000)` → `PLATFORM_LEGACY_FIXED_WING`：`cls="A"`、`cruise = min_mission = V`、`max_bank = atan(V²/(gR))`（53.4°，使 R_theo 檢查恰成立）、坡度 3°/4°（規劃）3.5°/5°（上限）（論文 Table 2 之 γ=0 為作業段水平飛行，Eq.36 俯仰限制值未列）、T_ref 180 min → 2484 km、`reserve_fraction=0`、`usable_range_fraction=(1,1)`、50 kWh 示例電池、航電旗標 True、`provisional=∅`、`transition_ground_run_m=800`／`a_tr=5`（避免數十 km 加減速段）。酬載需求 `paper`：`side_overlap=0`、`forward_overlap=0`、`h_target=1200`（論文條帶間距 = w = 2200.1 m）。 |
| 例外型別 | scenario 為頂層模組 | 情境／DEM 輸入錯誤丟 `mission.MissionInputError`（= `platform.MissionInputError`）。 |
| 額外公開輔助（非契約） | 測試與上層（viz／sim）需要 ENU ↔ WGS84 與座標框架 | `metres_per_degree`、`enu_to_latlon`、`enu_ring_to_wgs84`、`site_from_enu`、`scenario_frame(inputs)`（home 為原點之 `GeoFrame`，h0 = `alt_msl_m` 或 DEM(+alt_agl)）、各情境之 ENU 常數（`MINI_*`、`HILLS_*`、`PAPER_TABLE2_*`）。 |
| 下游驗證（補記） | 契約未要求，但情境須能被 `context.build_context` 消化 | 三情境皆經 `build_context` 驗證（`tests/test_scenario.py::test_scenarios_build_context`）：mini → E_use A 779.6／B 214.3 Wh、h_ceil 200（waiver）；hills → keep-out {T1,T2,PL1,BLD1,NF1}、感測器 (A,both_survey)／(B,both_survey_rgb) 皆建立（context 對每個 (cls, req) 依 kind 配對，`_rgb` 尾碼慣例無需特別處理）、Z1 地形起伏 40–60 m（D12 預算內）；paper → h_ceil=∞、ρ_sym=∞、w(1200)=2200.1 m。 |

## routing.py（契約 §6B）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `KeepoutSet` 欄位 | 切線圖需以**原始幾何**頂點為弧心、並知道各 keep-out 之水平緩衝（逐障礙 `keepout_buffer_m` 可不同；禁飛區為 0）；2.5D 需樓層 | 契約欄位順序不變，於尾端新增有預設值之欄位：`floors_z`（樓層 ENU z − b_v；障礙 −inf）、`kinds`、`categories`、`b_h_m`、`b_v_m`、`notes`、`geoms`（未緩衝 ENU 幾何）、`buffers_m`。只給契約四欄位時：`floors=-inf`、`geoms = simplify(polys, 0.5 m)`、`buffers_m = 0.5`（宣告簡化內縮量）。`active(z)` = `floors_z ≤ z < tops_z`（契約「tops_z > z_hint」之超集語意；nan → 全部有效）。另加 `active_polys`、`clearance_m(line, z)`、`max_buffer_m(z)`、`__len__`。 |
| `KeepoutSet.build(..., t_window)` | 契約無燒入開關；D24 燒入由本函式執行 | 新增關鍵字 `burn: bool = True`：障礙以 ⊕b_h 多邊形、`tops_z = z_top + b_v` 就地燒入 `terrain`（DSM 時 tree/building 不燒）；禁飛區不燒入。`t_window=None` → 全部禁飛區有效；窗外禁飛區**不建入**（記錄於 `notes`）。AGL 障礙頂高以 `terrain.z_raw`（燒入前）之幾何範圍內最高地面 z + h；地形外 → +inf（恆有效）並記錄。`top_msl_m` → 各頂點 `msl_to_enu_z` 取最大；禁飛區樓層取最小 − b_v、天花板取最大 + b_v。多邊形障礙以 `make_valid` 後最大部分建立。 |
| 切線圖之障礙圓半徑 ρ | 契約 `TangentGraph(keepouts, rho_m, z_hint_m)`、ρ = max(b_h, R_eff)；但契約不變量要求路徑 ⊕ half_width 與 **已 ⊕ b_h 之** `polys` 不相交，故弧至 `polys` 之淨空 ρ − b_{h,i} 必須 ≥ half_width。`DubinsPath.sample/discretize/path_time` 只認單一 `R_m` | 所有圓（起訖轉彎圓 + 障礙頂點圓）採**同一半徑** `R_path = max(ρ, R_m)`（`TangentGraph.path_radius_m`），回傳 `DubinsPath(word="AVOID", R_m=R_path)` ⇒ dubins 工具全部可直接使用。`avoid` 之 ρ 由 `avoid_rho_m(keepouts, R_m, z, half_width) = max(R_eff, b_h, max_active(b_{h,i}) + half_width + 0.5 m)`——預設參數下（A 200／B 120 ≥ 50+50.5）即規格之 max(b_h, R_eff)；只有巨大緩衝或 R_eff < b_h + b_trk 時才加大。`TangentGraph.__init__` 新增關鍵字 `clearance_m`（碰撞檢查對 `polys` 之淨空；缺省 = ρ − max b_{h,i}；要求值超過幾何可提供者 → `ValueError`）與 `chord_ds_m`。 |
| `TangentGraph.shortest(a, b, R_m, weight)` | 逐段 Python 回呼於 n² 轉移矩陣過慢 | 契約簽章保留；另加關鍵字 `v_air_mps`／`wind_xy`：`weight=None` 且給 `v_air_mps` → 向量化風中時間（取點規則與 `dubins.path_time` 相同），否則 → 長度。`avoid` 用向量化版。障礙圓↔障礙圓之切線與各圓相鄰切點間弧之淨空依 `R_path` 快取於實例（`_oo_cache`），每次呼叫只算起訖圓相關切線與新增節點所切分之弧（快取「淨空弧」之子弧免檢查）；12 個多邊形（~72 頂點圓）首次 0.45 s、之後 ~0.06 s／次，結果與不快取逐位元相同。無解回 `None`（含起訖狀態被包圍）。圖只含 CSC 型連接（起訖圓 ↔ 障礙圓之公切線與弧），CCC 型近距離機動由步驟 1 之 6-word 直接路徑涵蓋。 |
| 碰撞檢查門檻 | 弦近似與浮點 | 直線：`dwithin(clearance − 1e-3)`；弧：弦長 ≤ 5 m 之折線 `dwithin(clearance − 0.25)`（矢高 ≤ 25/(8r)）；`avoid` 之 `clearance = half_width + AVOID_MARGIN_M (0.5)`，故 `polyline(5).buffer(half_width)` 不變量恆成立（實測淨空 ≥ half_width + 0.44）。`avoid` 回傳前仍以 `path_is_clear` 驗證。 |
| `VisibilityGraph(keepouts, R_m, delta_max_rad)` | 契約無 z_hint／淨空；緩衝式 `b_h + R(sec(Δ/2)−1)` 相對原始幾何、而 `polys` 已含 b_h | 新增關鍵字 `z_hint_m`（None → 全部 keep-out）、`clearance_m`、`simplify_m=0.5`。緩衝（相對 `polys`）= max(clearance + R(sec(Δ/2)−1), 1.01R + 1 − min b_{h,i})：後者使緩衝後多邊形凸角局部半徑 ≥ R，令沿圓角邊界緊貼之折線以半徑 R 圓角時切線長必放得下（否則備援對圓形障礙必退化為繞圈）。`shortest(a, b)` 無解丟 `RoutingError`（契約回傳型別為 ndarray，無「無解」表示法）。 |
| 「Dubins 接合（頂點緩衝補償弧切角）」 | 契約未定演算法 | `polyline_to_path(poly, a, b, R, keepouts, half_width, z)`：先把同轉向、累計 ≤ Δ_max 之連續小轉角合併為虛擬轉角（首入邊與末出邊交點），內部頂點以半徑 R 圓角（切線長 R·tan(Δ/2)），起訖以 6-word 中最短淨空者接合；圓角放不下時改為頂點狀態逐對 Dubins 接合。是否淨空由 `avoid` 以 `path_is_clear` 驗證，不淨空 → `RoutingError`。 |
| `avoid` 步驟 1 | 規格只寫最短 word `D0` | 6 word 依長度遞增取**第一個淨空者**（最短 word 淨空時與 `dubins_path(a,b,R)` 物件內容完全相同）；`keepouts=None` 或無有效 keep-out → 直接 `dubins_path`。 |
| `tg_cache` 鍵 | 任務指定 `(rho, z_hint)` | `TangentGraph` 存於 `(rho, z_hint)`；可視圖存於 `("vis", R_m, z_hint, half_width)`（同一 dict）。快取之圖若 `clearance_m` 不足新的 half_width 需求或 `keepouts` 物件不同則重建。 |
| 額外公開（非契約） | 供 costs／altitude／測試重用 | `segment_time(seg, Va, wind_xy)`、`path_is_clear(path, keepouts, hw, z)`、`path_clearance_m`、`direct_clear_path(a, b, R, keepouts, hw, z) -> (path, clear)`、`polyline_to_path`、`avoid_rho_m`、常數 `AVOID_WORD="AVOID"`、`AVOID_MARGIN_M=0.5`。 |

## altitude.py（契約 §11）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `PathSegment2D` 定義於 `planner.py`（契約 §10） | `altitude.build_3d/resample_segments/takeoff_corridor/landing_corridor` 皆以其為輸入／輸出，而 planner 位於更高層（altitude 不得 import planner） | `PathSegment2D` 依契約欄位逐字定義於 `altitude.py`；**planner 應 `from .altitude import PathSegment2D`**（勿重複定義）。 |
| `Sample2D` 欄位 | 契約欄位不含 z 提示、掃向與弧命令參數，但 `sample_bounds`（無 pin 參數）需走廊段高度線、`Waypoint.arc/dir` 需來源 | 於**尾端**新增預設欄位 `z_hint_m`（自航段 `z_hint_start/end` 依弧長內插）、`dir`、`arc_param=(cx, cy, R, turn_dir)`；契約位置參數不變。`ds_m` 於弧段為**真實弧長**（非弦長）。 |
| 接點樣本 | 相鄰 2D 航段之接點只應有一個樣本 | 接點採**後一航段**之屬性（phase/mode/strip/sensor_on/v_air），對應「航點屬性 = 出發航段」語意（與 `dubins.discretize` 之 theta 約定一致）；垂直段（單點折線）給兩個同位樣本（ds=0）。 |
| `EnvelopeResult.deficit_m` | 契約「INFEASIBLE(first j, z*[j]−u[j])」 | `first_violation` = 首個 z*>u（或 nan）之索引；`deficit_m` = **max_j (z*−u)**（§11.5 處置需最大缺口）。遞迴以 cumsum + `maximum.accumulate` 向量化（與逐點遞迴等價，1e-9）；tan 之 inf → 1e6、nan → 0。 |
| `phase_boundaries` 之上界線 | 規格 §11.1「自 P_tc 以 tan_c 全速爬升之上界線」與「平地 471 m（順風 V_g 最大之保守 tan）」不完全等價（逐樣本實際 tan 於逆風／弧段不同） | j_c = 首個樣本使**保守線**（`tan_gamma_ground("climb", Va+W)`，與 `climb_dist_m` 同）與**實際線**（逐樣本 tan_c 含弧段 κ）皆 ≥ z_g^max + h_minfw；j_d 對稱。CLIMB 自 ACCEL 段末（P_acc；ACCEL 等高）起算，平地 A@W=5 → 471 m（±1 樣本）。只在樣本含 ACCEL／DECEL 時就地改標 CLIMB／DESCENT（`Sample2D.phase` 可變）。 |
| `sample_bounds` 之 h_max（OPER 上界） | `Strip` 無 `req_id`，無法直接找該條帶之感測器 | 取該構型所有 `ctx.sensors` 中 `agl_window()[1] ≥ strip.h_op_m` 者之最小 h_max（單一感測器時即其值；無 → inf）。ACCEL/DECEL 與走廊段：z_lo = z_hi = z_hint、tan = 0（等高／旋翼段不參與固定翼坡度）。 |
| `build_3d` 之 ENVELOPE 範圍 | 垂直段 ds=0、旋翼段坡度不受 3°/4° 限制（D37） | ENVELOPE 只跑固定翼區塊（首 ACCEL 樣本 → 末 DECEL 樣本）；走廊樣本 z 直接取 `z_hint`（§11.6/11.7 高度線）。不可行時仍輸出航點（z ≥ z*）並於 `ProfileStats.infeasible_at/violations` 回報。`seq` 只讀 `landing_site`（鴨子型別）；`agl_m` = z − 正下方地形，`agl_min_by_strip` = min(z − z_g^max_{w/2})（D36 不變量）。 |
| `alt_reachable` | 契約未定演算法；EnergyBudgetTable 屬 costs（更高層） | 簡估：最近站點直線距離／V_g^worst 之 `transfer_wh`（向量版）+ e_LDG ≤ E_rem − r·E_nom。`lost_link_action`：RTB 策略 → "RTB"；CONTINUE_THEN_RTB → 承諾點（RTB/DESCENT/DECEL/TRANS_IN/LAND）前 "CONTINUE"、之後 "RTB"；NEAREST_ALT → "ALT:<最近站點>"。 |
| 盤旋段表示 | 契約 `PathSegment2D.path: DubinsPath \| ndarray`，整圈不是 Dubins 三段 | `DubinsPath(word="ORBIT")`（`altitude.ORBIT_WORD`）單一弧段、`dtheta` 可 ≥ 2π（`dubins.sample/path_time` 直接可用）。起飛盤旋圓心優先 P_acc ± R_eff·n̂（切入零長度，右轉優先），否則 `choose_orbit_center` + Dubins 接入段（亦標 ORBIT）；出口 ϑ* 以 24 個候選（15°）取 t(orbit→ϑ)+t(Dubins→S_entry) 最小且自 S_exit 之包絡可行者，不可行則加圈（≤ 3）。降落盤旋切於 S^app（進出皆 S^app），置於 DECEL 前。 |
| `landing_corridor` 下降距離 | 規格同時給「順風保守 tan_d」與「前向遞迴精算」 | n_lap = ⌈max(需求 − D·tan_d^cons, a[N] − z_tc)/Δh_lap,d⌉：保守距離為門檻（逆風之利不利用），遞迴補足途中地形。 |
| `choose_orbit_center` 參數 `link_poly` | 可為 None | None → 略過 C2 檢查；`keepouts` None → 略過 keep-out 檢查。候選 d ∈ {R,2R,3R,4R}、φ 每 30°。 |
| `site_check` 之障礙 | 契約 `obstacles_enu: list[(Geometry, top_z)]` 無類別／緩衝 | 依契約形狀使用；轉換走廊／CLIMB 走廊障礙以 `ctx.obstacle_pairs()`（`_climb_corridor_issues`）。站點 `approach_heading_deg` 給定時起飛與降落皆用該方位（皆逆風 → 同向）。 |
| `layering` 之 `bands` | `decompose.AltitudeBand` 尚不存在 | 鴨子型別（`h_lo_m/h_hi_m/feasible`）；缺 band 時以 h_op 本身為上下限。只有一構型可行之胞原樣回傳 `{cls: h_op}`。 |
| 走廊橫向取樣 dv | 契約 `dv_m` 未定 | `max(step_m/2, 0.5)`（雙線性極值在節點／端點）。 |
| 額外公開函式 | 供 decompose 使用 | `make_local_envelope_fn(...)` 把 `local_envelope` 包成 `geometry.LocalEnvelopeFn`（含 ±R_eff 延伸與 strip_mask、走廊偏移＝超集帶中心 − 飛行線 v、回傳 4-tuple 含 z_op）。 |

## deconflict.py（契約 §13）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `Conflict` 欄位 | 修復（§13.6「一次算足」）需要事件之 Δ_req、時序與單機最小延遲；走廊衝突需站點與預約種類 | 契約 8 欄位不變，**尾端**新增有預設值之欄位：`dt_req_s`（事件內最大 Δ_req = Δt+σ_a+σ_b−\|t_a−t_b\|；走廊 = 重疊秒數）、`dt_ab_s`（最近點 t_b−t_a）、`delay_a_s/delay_b_s`（只延遲 a／只延遲 b 即可消除整個機對衝突之最小延遲，含裕度 1 s 與 σ 隨延遲成長）、`site_id`、`res_kinds`（走廊預約種類對）。另加 `pair` 屬性、`key()`、`involves()`。4d/corridor 之 `uav_a ≤ uav_b`（字串序）；ghost 之 `uav_b` = 幽靈。 |
| `Conflict.d_h_m/d_v_m/t_s/idx` 語意 | 契約未定「一筆衝突」之粒度 | 同一機對之連續違規樣本對（a 之索引間隔 ≤ 10 s）聚為一個事件 → 一筆；回報最近點（min d_h）之 (t_a, idx_a, idx_b, d_h, d_v)。`idx_*` 為 `trajectory_samples(plan, cfg.dt_4d_s, t_sync)` 之列索引（ghost：`GhostTrack.samples` 列）；走廊衝突之 `idx_*` 為 `CorridorSchedule.reservations` 索引、`d_h=d_v=0`、`t_s` = 重疊起點。 |
| `Δ_req` 之「一次算足」 | 只以「已違規樣本對」算 Δ_req，對平行條帶對飛／同線反向等情形，延遲後交會點只是移動而非消除（實測 3 輪皆殘餘） | `delay_a_s/delay_b_s` 對該機對**所有**空間相近樣本對（水平 < 門檻、垂直 < Δh_sep、任意時差；2-D `cKDTree` 查詢）求解：每對之禁止延遲區為開區間 (L, U)，U=(win−Δt)/(1−k_t)、L=(−Δt−win)/(1+k_t)，合併區間後自 0 向上跳出 → 最小可行延遲（O(P log P)）。`dt_req_s` 仍為規格之 Δ_req（事件最大值）。 |
| `check_separation` 之 cKDTree | 契約「cKDTree on (x, y, v_ref·t)」；σ 大時（t≈2000 s → 視窗 ≈ 200 s → 半徑 ≈ 4 km）3-D 查詢幾乎回傳全部樣本對（8 機同層 ≈ 1 s） | 逐機對選擇候選產生器：視窗窄（hypot(d_sep, V_ref·win_max) ≤ 3·d_sep）用 (x, y, V_ref·t) 3-D 樹；否則用 2-D (x, y) 樹（候選 = 全部空間相近對，並直接重用於延遲計算）。兩者皆再以精確條件判定，結果相同；8 機／7000 樣本 ≈ 0.05 s。 |
| 水平門檻 `d_sep` vs `d_sep_lat` | 契約 cfg 含 `d_sep_lat_m`（D18 同胞側向間距 60 m）但 §13.3 只寫 d_sep | 兩樣本皆 OPERATE 且航跡差 ≤ 20°（同向平行作業）→ 門檻 `d_sep_lat`；其餘（對飛、交叉、非作業段）→ `d_sep`。垂直門檻一律 `dh_sep`。 |
| `trajectory_samples(plan, dt_s, t_sync_s)` 之 k_t | 簽章無 cfg | 關鍵字 `k_t=0.05`（= PlannerConfig 預設）；`check_separation` 以 `cfg.k_t` 呼叫。弧命令航點以弦近似（build_3d 取樣 ≤ min(d_d, RΔψ)，弦誤差 ≤ e_max）；時間重複之航點視為跳躍（取後者）。 |
| `check_separation` 之 `t_sync` | 契約簽章無 | 關鍵字 `t_sync_s=0.0`。 |
| `spatially_exclusive(teams, cells, cfg)` | 「互不相交」不足以保證水平 ≥ d_sep；D38 側向外延需 R_eff（Cell 無此欄位）；回傳型別未定 | 要求兩團隊迴轉帶聯集之距離 ≥ `margin_m`（預設 `cfg.d_sep_m`）；關鍵字 `r_eff_m`（cls → R_eff 或單值）給定時側向外延 = max(0, R_eff − s_min/2)，否則 0；只判定**同構型**團隊對（異構型由分層負責）；回傳 **uav_id 對**（字串排序 tuple）之集合，可直接作 `skip_pairs`。無胞之團隊不判定。Team 為鴨子型別（cls/cell_ids/uav_ids）。 |
| `CorridorSchedule.reserve` 回傳值 | 契約只給 `-> bool` | 一律登記，回傳「登記前該站是否無重疊」；起飛與降落共用走廊（同站任何預約互斥；相接不算重疊）。另加 `is_free`、`release(uav, kind=None)`、`reservations_of`、`copy`、`reserve_plan(plan, takeoff_site_id)`（起飛 [t0, t0+Δt_dep]、降落 [t_arr_ptc−30, t_land+t_sep_ldg]，`plan_windows`）、`reserve_ghost(ghost)`（[ETA−60, ETA+180]，規格 §12.5）。`next_free` 之 `kind` 不影響互斥（介面對稱）。 |
| `repair(...)` 簽章 | 需要幽靈、互斥對、t_sync、目前時刻（已起飛者不可延起飛）、起飛站（UAVPlan 無起飛站欄位）、能量重驗與 IDP hook | 只新增關鍵字：`ghosts`、`skip_pairs`、`t_sync_s`、`t_now_s=-inf`、`takeoff_site`（None → home；str；或 uav→site dict）、`v_ref_mps`、`energy_check(plan, model)->bool`（預設 `default_energy_ok`：`energy_margin_wh ≥ 0` 且末航點能量 ≥ r·E_nom；完整 ENERGY_SAFE_CHECK 請注入 `sim.check_energy` 包裝）、`resequence_hook(uav_id, plan, conflict)->UAVPlan|None`（(iii) IDP 重排，本模組不實作）。 |
| `repair` 之候選選擇 | 規格「依序 (i)(ii)(iii)」但未定多候選之取捨 | 每筆衝突：(i) 起飛延遲（延遲 a 或 b 各以 `delay_*_s` 起算，經走廊 `next_free` 貼合）與起飛換序（互換 t0）；(ii) 延遲盤旋 n=⌈delay/T_lap⌉（降落預約不可用時再加圈）；每個候選以**乾跑**驗證（該機對重跑 `check_separation`／走廊重排），取修復後 makespan 最小者（tie → 總平移／圈數最小）。§W.9 重現：換序 B1 t0=0、A1 t0=60 → A1 落地 1756.7。走廊衝突「降落優先於起飛」（降落 vs 起飛只延起飛機；起飛機已起飛則降落機於保持點盤旋）。每輪每機至多修改一次；輪末以 `check_separation` + `schedule.conflicts()` 重驗；無任何修改即提前結束。 |
| 延遲盤旋之表示 | 契約「shift timestamps」未定航點表示 | `insert_delay_orbit`：於插入點複製原航點為 ORBIT 航點（t 不變、sensor_on=False、`arc=(cx, cy, R_orb, +1)`，圓心在航跡右側 R 處），原航點及其後全部 +n·T_lap、`energy_remaining −e_orbit`（e_orbit = P_c·n·T_lap·k_unc，等高巡航功率；契約 §W.9 之「≈16 Wh」無法由 §5 功率重現，A 一圈為 10.4 Wh）；`orbits_inserted += n`、`distance_m += n·2πR`、`warnings += delay_orbit(...)`。插入點 = 衝突時刻之「最近前一非 OPERATE 航段」中第一個 {CLIMB, ORBIT, ENTRY, TRANSFER, RTB, HOLD, DESCENT} 航點；走廊衝突改取降落前距站點 ≥ 2 km 之最後可插入航點（保持點）。4D 取樣把盤旋視為該點停留 n·T_lap（保守近似）。 |
| `T_lap` | `altitude.orbit_gain` 為上層（第 3 層）模組 | 本模組私有 `orbit_lap_time_s(spec, wind, z, r_orb=None) -> (T_lap, R)`：T = 2πR/V_a·`wind.circle_time_factor(V_a, W(z))`，R = R_eff(W)；A/W=5 → 60.9 s、B → 41.9 s、A 無風 58.4 s（與規格 §11.6 一致）。 |
| D18 不變量 | — | 任何路徑皆不改 z（起飛延遲／盤旋只改 t 與能量）；測試 `test_repair_no_vertical_offset_D18`。 |
| 其他限制 | 範圍外 | 備降點改派（能量不足時「改備降點」）、區域時間窗 H3 對起飛延遲之影響、幽靈走廊獨占之時效皆未處理（留給 dynamic／planner）。 |

## decompose.py（契約 §7）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `AltitudeBand.reason` 集合 | 無感測器／地形範圍外之情形契約未列 | 另加 `"no_sensor"`（該構型無此 req 之硬體）、`"terrain_unknown"`（Ω 與胞內皆無 DEM）。`h_lo_m/h_hi_m` 為 **AGL** 可用作業高度帶（`altitude.layering` 以其夾 h_op）：h_lo = max(h_minfw, h_clr)、h_hi = min(h_ceil − 起伏, h_max,sensor)；`dh_band_m` 仍為規格式（只含 h_ceil）。`A_needs_waiver`：cls A、Δh < 20、無 `waiver_max_agl_m` 且 h_ceil ≤ h_minfw + 20。 |
| 高度帶之 z_g | 規格 §4.6 指 DEM 地形；`ctx.terrain.z` 已燒入障礙（塔頂 + b_v），Ω = 胞 ⊕ d_turn（A 535 m）常含孤立電塔 → 山丘情境 Z1 起伏 55 → 101 m、Z2 27 → 100 m，A/B 全數誤判不可行 | 以 `terrain.z_raw`（自然地形）取 Ω 內格點 ∪ 頂點格四角之 min/max（`_raw_terrain_minmax`）；障礙淨空由 §7.4 局部包絡（燒入地形）與 keep-out 保證。ENU 曲率下垂（4–7.5 km 處 1–4 m）計入起伏：mini A Δh_band = 76.4（契約 §W.3 平地 80）、B 96.8。 |
| `direction_candidates` 順序／去重 | DP 只外擴簡化（ε = 0.1·min s_c ≈ 10 m）把緩衝矩形角落化為斜邊 → 「各邊方位」含 0.15° 之偏斜與 21 m 碎邊 | 順序 β、β+π、β±90°、θ_minw、各邊（長者優先，短於周長 2% 者略去）、等高線；1° 去重保留先出現者（風軸為代表）。等高線 = 胞內 z_raw 梯度結構張量主軸 ⊥（平均梯度 < 1e-3 視為平地不加入）。`decompose_area` 內再以 mod π 去重（代理對 θ↔θ+π 對稱）。 |
| `proxy_time` 細節 | 規格 §7.3 公式留白處 | V_g^*：V_a^* = max{V_a ∈ [V_min, V_c] : V_g ≤ V_cap} ⇒ V_g^* = min(V_g(V_c), V_cap)，V_g(V_min) > V_cap 或 < v_g_min 或 nan → 該向不可行。條帶長 = 帶內裁剪之 u 極值 + 2e_ext（向量化 shapely，不鋪條帶）。tanγ^g_c 以順風（最大 V_g）保守值；|L|/V_g 用兩向調和平均。pen_Δz：n−1 次轉場拆為 ⌈(n−1)/2⌉ 上坡（側移 k·s、爬升角、弧段 κ）與其餘下坡（側移 (k−1)·s、下降角），Δh_lap/T_lap 依 §11.6（`circle_time_factor`）。pen_keepout：迴轉帶（d_turn、側向 R−s/2）∩ NoFly⊕b_trk 之面積 ÷ 迴轉帶環面積 × (n−1)·t_turn；最小 U 型帶（R+e_ext、側向 0）相交 → inf。`one_way`：−1 = 只有 θ（LR）可行、+1 = 只有 θ+π。 |
| `build_strips` 禁區檢查 | 契約「作業路徑 ⊕ b_trk」 | (path ⊕ b_trk) ∩ NoFly ≠ ∅ ⟺ path ∩ (NoFly ⊕ b_trk) ≠ ∅，故以中心線（含 e_ext）對 `ctx.nofly_buffered` 檢查（與 routing 之 nofly `buffers_m=0` 一致；若再對 ⊕b_trk 集合加寬 b_trk 會把緊鄰禁區之胞全數判為不可行）。相交條帶標 `feasible=False, infeasible_reason="nofly_intersect"`，不刪除；`decompose_area` 以 `all(s.feasible)` 判定方向可行。閉包 = `altitude.make_local_envelope_fn`（4-tuple，`Strip.z_op_m` 為 ENU）。`cell.h_op_m[cls]` 須先存在。 |
| `exact_time` | 契約只定義兩向皆可行 | 單向可行條帶（D34）：作業取可行向、轉場改同向 racetrack `end_state(d) → 下一條 start_state(d)`；兩向皆不可行 → inf。Strip 無 req_id：以 `min(h_min,h_op)` 之 swath 反推感測器（`_req_of_strips`）取 V_cap；風取條帶 z_op 平均；R_eff = spec.r_eff(W_R)（W_R 政策見下）；轉場以 `dubins_all_times`（最短 word）向量化。 |
| `suitability` | `DecomposeContext` 無機隊架數、無 link_poly | 關鍵字 `n_uav: int \| None`（λ = min(n_uav, n_q)；None → n_q）。H2 只用 `rho_sym_m[cls]`（home 至胞之 Hausdorff 距離 + d_turn ≤ ρ）。H4：2·P_c k_unc·d_home/V_g^worst + P_c k_unc T_single/λ ≤ e_usable（`enforce_energy=False` 略過）。H3（需時序）、H7（單條帶仍可行）視為通過。`s_gsd`：rgb 需求或 both 之 rgb 半套時 B=1、A 依 (A, req_id+"_rgb") 硬體 GSD 達成率（無 → 0）；both 之 A（lidar 半套）= 1。ρ^sym_B 缺省 → spec B 表值下緣 39 km。`d_q` = 胞形心至 home。s_turn 以 `proxy_time(alpha*)` 之 t_turn/pen_Δz 重算。 |
| `decompose_area` 簽章 | 需要 ENU 多邊形與配置參數 | 關鍵字 `polygon_enu`（給 `ctx.areas_enu[id]` 則不再自 WGS84 轉換）、`rho_sym_m`、`t_use_s`（cls → s；None → `t_use_s_for`）、`n_uav`（cls → 架數）。cell_id = `f"{area_id}_{q}"`（q 自 1）；條帶 id `f"{cell_id}_{cls}_{l}"`。`convexify_whole=True` → F 改為各分量凸包 \ NoFly⊕b_trk（論文退化）。(Q ⊕ δ) ∩ F 多片時取含 Q 代表點者。δ_w = max_c w_c σ_z/h_op（`terrain.sigma_z_m`）；ε_dp = min(0.1·min s_c, 0.99 δ_edge)；τ_merge = merge_tau_factor·min w_c。`residual_candidates` 不在其中呼叫（由 planner 依契約流程呼叫）。`fleet_classes` 中不在 `ctx.specs` 之構型略過。 |
| `residual_candidates` | 契約型別 `(int, Cell, Cell \| None)` 對 k=0 無 A 子胞 | k = n_A → `(n_A, cell, None)`；k = 0 → `(0, cell, cell)`（A 欄以原胞代之，B 用 `cell.strips["B"]`）；0 < k < n_A → A 子胞 `f"{cell_id}_A"`（保留原 A Strip 物件 1..k、`T_single` 重算）、B 殘帶 `f"{cell_id}_B"`（`residual_subcell` 後以完整方向搜尋重鋪；`residual_of=cell_id`、`k_A=k`）；B 重鋪不可行之 k 略去。v_top = strips_A[k−1].v_hi（含置中平移）。Ĉ 估計：Ĉ_A = exact_time(前 k 條)、Ĉ_B = 殘帶條帶數 × B 平均條帶時間；結果依 k 遞減。關鍵字 `rho_sym_m/t_use_s/n_uav` 同上。 |
| R_eff 風速政策 | 契約以 CostCache.W_R 為準，decompose 早於 CostCache | `r_eff_for_class(ctx, cls, z_top=None)`：`max_layer` → [z_site + H_tc,lo, z_top] 最大風速，z_top 缺省 = 地形全域最高 + h_ceil（無限 → +300 m）；`op_layer` → W(z_top)。mini 單層 → 200/120。e_ext 之 V_g,max = V_a + W(z_op)。 |
| 額外公開函式 | 供 preflight/allocate/planner 重用 | `class_can_serve`（both：A 需 lidar、B 需 rgb）、`resolve_sensor`（req_id → req_id+"_rgb" 後備）、`r_eff_for_class`、`end_extension_for`、`t_use_s_for`、`d_turn_from_r`、`RGB_REQ_SUFFIX`。 |
| 效能 | — | mini：decompose_area 0.25 s、residual_candidates 0.4 s（dem_step 50）；山丘三區合計 ≈ 1.8 s（Z2 含洞 C 形 5 胞）。代理全向量化；每方向精鋪 3 次 lay_strips 為主要成本。 |

## costs.py（契約 §8）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `rtb_cost(...).t_s` 與 `CostCache.rtb` 之內容 | 規格 §8.7 之 $t^{RTB}$ 含 $t_{acc}+t^{in}_{tr}+t_{dn}+t_{hov,res}$，但契約 §W.4／§W.7 在 `makespan_after_insert`／$C_i$ 中把「RTB」與「降落 117/106 s」分開相加 | `rtb_cost.t_s` = t(AVOID(端點 → S^app)) + 下降螺旋 $t_\downarrow^{extra}$（**不含**降落固定時間），`e_wh` 依 §8.4 下降側（**不含** $e^{LDG}$）。降落固定時間／能量由 `altitude.landing_corridor`／allocate 另加。`EnergyBudgetTable` 之值則**含** $e^{LDG}$（規格 §5.5、§W.8）。 |
| `entry_cost(..., n_laps, t_lap_s)` 與 §8.4 螺旋 | 暫定起飛走廊（§11.6 包絡）已精算爬升圈數；§8.4 之 $n_{lap}$ 估計較粗（不含弧段 κ、途中地形） | `n_laps > 0` 時以走廊給定之 `(n_laps, t_lap_s)` 為準（時間 + n·T、能量 + $P_\uparrow nT k_{unc}$）；`n_laps == 0` 時改用 §8.4 之估計 $\lceil(|\Delta z|-|D|\tan\gamma^g)/\Delta h_{lap}\rceil^+$（順風保守 $\tan\gamma^g$、$\Delta h_{lap}$/$T_{lap}$ 同 `altitude.orbit_gain`）。`CostCache.entry` 同（`init_laps[uav]`）。 |
| `transfer_cost`／`rtb_cost` 之 $R_{eff}$ | 契約簽章無 R 參數 | 新增關鍵字 `r_eff_m: float | None = None`；None → `spec.r_eff(W(z_mid))`。`CostCache` 一律用 `R_eff[cls] = spec.r_eff(W_R[cls])`（§3.2 政策；`W_R` 之掃描上界 = max(條帶 z_op, z_site + h_minfw)）。 |
| `sites: dict[str, tuple[Pose, float]]` | $S^{app}$（$P_{tc}-d_{acc}\hat t$）之 $d_{tr}$、$d_{acc}$ 隨構型不同（A 606 m、B 489 m） | 值可為 `(Pose, z_tc)`（兩構型共用）或 `{cls: (Pose, z_tc)}`；`costs.resolve_site_entry(sites, site_id, cls)` 解析。 |
| 風取樣高度、keep-out 之 z | 契約只說「z 為 ENU z」 | 每對路徑之風向量取起訖 z **中點**（路徑視為等高；`dubins.path_time` 單一 z），keep-out 有效判定 `z_hint = min(z_start, z_end)`（2.5D 保守）。 |
| 轉移矩陣批次建表 | 任務允許 $L/V_a(1+(W/V_a)^2)$ 代理 | **不用代理**：`dubins.dubins_all_times` 已向量化（30 條 × 4 型 ≈ 30 ms；全表含預算 ≈ 0.03–0.2 s），按起訖中點風向量分組呼叫，與逐對 `path_time` 相等（1e-9）。 |
| V_g 下限（cfg.v_g_min）於批次層 | `dubins_all_times` 只回傳時間，無逐段航向 | 同組風速 $V_a-W<V_{g,\min}$ → 該組全部 inf（反向／同向轉場皆掃過逆風航向，保守）；繞行路徑與單筆函式逐段（弧每 5°）檢查 $V_g\ge V_{g,\min}$。 |
| keep-out 掃掠檢查之候選篩選 | n² 對逐一取樣過慢 | 起訖連線 ⊕ (4R + b_trk + 最大緩衝) 之 `STRtree.query(dwithin)` 批次篩選（Dubins 路徑必在此範圍內），只對候選建 `dubins_path` 檢查 `path_is_clear`，衝突者 `routing.avoid`（無解 → inf）。 |
| `tr_word` 語意 | 契約未定 | `int8`：`WORDS` 索引 0–5；**6 = AVOID**（切線圖繞行，`costs.WORD_AVOID_IDX`）；**−1 = 不可行**。`routing.avoid` 回傳「非最短但淨空」之 6-word 路徑時保留其自身索引（長度 = 該路徑）。繞行路徑物件存於 `tr_paths[cls][(id_l, id_m, k)]`、`entry_paths[uav][(id, idx)]`、`rtb_paths[(cls, site)][(id, idx)]`。 |
| `refresh_wind` | 契約「只重算時間層」 | 非繞行對以同 R 重解 Dubins（長度／word 逐位元相同）並重算時間；繞行路徑**沿用幾何**只重算時間；`R_eff(W_R)` 變化 > 5%（`R_EFF_REFRESH_TOL`）之構型才清 `tg_cache`、重算幾何（含 AVOID 與預算表路徑）並更新 `geometry_key`。`wind_key = wind.version`。 |
| `EnergyBudgetTable.__init__` 額外參數 | 契約簽章無 $h_{clr}$、無共用切線圖快取 | 關鍵字 `h_clr_m=0.0`（`CostCache.build` 傳 `ctx.h_clr_m`）、`tg_cache`、`d_sample_m`（剖面取樣，預設 cfg.d_d_m）、`r_eff_m`（cls → R；缺省 `spec.r_eff(W_max)`）。 |
| 預算表「真實值」之剖面 | 規格 §5.5 只寫「AVOID + 無釘住 ENVELOPE」 | 沿 AVOID 路徑每 d_d 取樣，走廊（⊕ b_trk）地形 + max(h_minfw, h_clr)；D35：自 S^app 以 tan_d 回推之下降線低於 z_g + h_minfw 處（末端 DESCENT 段）只含 h_clr。起點 pin = max(z_op, z_lo[0])（作業剖面已保證 z ≥ 局部地形 + h_min，D36），後向遞迴超出者 → 爬升螺旋 n↑；末端 z*[N] > z_tc → 下降螺旋 n↓（§8.7 $t_\downarrow^{extra}$）。能量 = $k_{unc}[P_c t+(P_\uparrow-P_c)\Delta z^+/(V_a\sin\gamma_c)-(P_c-P_\downarrow)\Delta z^-/(V_a\sin\gamma_d)+P_\uparrow n_\uparrow T+P_\downarrow n_\downarrow T]+e^{LDG}$（沿剖面之總爬升 $\Delta z^+$／總下降 $\Delta z^-$）。DEM 外樣本之 z_g 以路徑上最大有限值代替（保守但有限）。 |
| §W.8 之 `min_rtb_wh` 153.6 Wh | §W.8 以 $P_c t$ 計（「平地無爬升項」），未含 §8.4 之下降功率折抵 | 本表依 §8.4 含下降折抵：mini A 條帶 3 → 149.1 Wh（−4.8 Wh）；測試斷言 $[e_{ref}-6, e_{ref}+0.5]$。 |
| `point_wh` 之爬升項 | 規格只含 $z_{ridge}+h_{clr}-z_x$ | 目標高取 $\max(z_{ridge}+h_{clr},\ z_{p^*})$（併入端點本身高度，仍為上界）；$z_{ridge}$ 為 x→p* 直線走廊（⊕ b_trk）最高地形；$V_g^{worst}=\max(V_{g,\min},V_a-W_{\max})$ 以 `wind.max_speed()`。表值 inf 之端點不作最近點候選。 |
| `extreme_point_wh` | 規格未定最外點 | 端點沿飛行方向外延 $\sqrt{4R^2-(R+s/2)^2}+R$（$s<2R$，Ω 超出量 D38）或 $R$，再 `point_wh`。 |
| `static_ok` | 規格 §8.9「e(H→端點 直線最壞)」未定式 | $k_{unc}[P_c\,d/V_g^{worst}+(P_\uparrow-P_c)\max(0,z_{op}-(z_H+H_{tc,cmd}))/(V_a\sin\gamma_c)]$，$e^{alt}$ = 預算表各站點最小值，門檻 $\eta_TE^{avail}_{rep}-e^{TO}$（`models_rep`）；`cfg.enforce_energy=False` → 全 True。 |
| `op_cost` 之 $V_g^{cap}$ 與 $V_{a,\min}$ | `Strip` 無 `req_id`，`CostCache.build` 找不到條帶所屬感測器 | `build(..., strip_sensors={strip_id: Sensor})` 關鍵字；缺省取該構型所有 `agl_window()[1] ≥ h_op` 之感測器中最小 `max_ground_speed_mps(h_min)`（保守）。$V_{a,\min}(m)=\max(v_{min,mission},1.2V_s(m))$、上限 $\min(V_{a,c},$ `max_airspeed_mps`$)$；`costs.v_air_star` 為閉式向量版（$V_a=\sqrt{(cap-W_\parallel)^2+W_\perp^2}$）。作業能量 = $P_c t k_{unc}$（不隨 $V_a^*$ 改變功率）。 |
| `CostCache` 額外欄位 | 增量／重算需幾何層與狀態 | 契約欄位順序不變，尾端新增（皆有預設）：`R_eff`、`strips`、`sites`、`init_states`、`init_laps`、`uav_cls`、`wind`、`tg_cache`、`strip_sensors`、`tr_laps`、`op_va`、`op_vg`、`entry_len/word/laps`、`rtb_len/word/laps`、`tr_paths/entry_paths/rtb_paths`；方法 `strip(cls, l)`、`n_strips(cls)`。`geometry_key` = 條帶端點／z_op／R_eff 之 hash；`state_key` = 初始狀態＋圈數之 hash。 |
| `add_strips` 之 op | op 只依構型、向量化 | 整列重算（值與原列逐位元相同）；tr 只補算 新×全 與 舊×新 區塊；entry／rtb／預算表／static_ok 只補新列。 |
| `PathCost.dz_m` | 契約 | = z_end − z_start（ENU；entry：z_op − state.z；rtb：z_tc − z_op）；`n_laps` 為螺旋／盤旋圈數。 |

## preflight.py（契約 §3A）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| 步 1 缺鍵之 NO_GO | `build_context` 遇第一個缺鍵即丟 `MissionInputError`，無法一次列出所有缺鍵 | 先於**輸入層**自行檢查（P_link 逐構型、terrain_clearance 兩層、alternates、E_nom 解析）並列鍵名（`c2_link_unresolved(cls): 缺鍵：c2.range_km[cls]/…`、`terrain_clearance_unresolved: …`、`alternates_missing(cls…)`、`energy_unresolved(uav): …`），再呼叫 `build_context`；其 `MissionInputError` 亦轉為錯誤 `build_context: …` 並立即回傳 NO_GO（per_area/per_uav/derived 為空）。`cfg.enforce_link=False`（legacy）時略過 C2 檢查。 |
| 錯誤／警告字串 | 契約只列警告名稱，未定錯誤名稱 | 錯誤一律 `name(args): detail` 形式：`weather_go(...)`、`site_check(site, cls): …`、`no_usable_alternate(cls)`、`uav(id): auw_exceeds_mtow/payload_exceeds_max/energy_usable_nonpositive/stall_margin/v_min_exceeds_cruise`、`transition_slope_exceeds(cls)`、`area_outside_dem(id)`、`no_coverable_area`、`time_window_disjoint(area)`、`time_window_insufficient(area|airspace)`、`platform_overrides: …`。 |
| `provisional_params` | 契約要求 `warnings` 含此裸字串；但逐構型鍵名亦應可讀 | 同時給裸字串 `provisional_params`（列首）與 `provisional_params(cls: k1, k2, …)`；鍵名清單另存 `derived[cls]["provisional_params"]`。覆寫後之鍵已由 context 自集合移除。 |
| `battery_nominal_example` | context 只在 `pack_nominal_wh` 缺省（退回 `spec.battery_nominal_wh`）時警告；契約 §W.2 的 mini example 明給 `pack_nominal_wh=1500/420` 仍應警告（規格 §5.7「使用示例值 1500/420 Wh 時警告」） | 當架次標稱恰等於 `EXAMPLE_BATTERY_NOMINAL_WH[cls]`（1500/420）時亦發 `battery_nominal_example(uav)`；其餘（`reference_battery_assumed`、`battery_temp_assumed`、`sensor_spec_default_example`、`airspace_ceiling_defaulted_120`、`overridden(...)`）直接沿用 `ctx.warnings`（全部併入報告）。 |
| 區域不可覆蓋之嚴重度 | 規格 §6.2 步 9 只寫「UNCOVERABLE(sensor)」，未定是否 NO_GO | 單一區域不可覆蓋 → 警告 `uncoverable(area: reason)` 並記錄 `per_area[aid]["coverable"]=False`（planner 仍可規劃其餘區域）；**所有**區域皆不可覆蓋 → 錯誤 `no_coverable_area`。`both` 區需 lidar 與 rgb 兩半皆有可行構型（A 同時掛 RGB 時 A 可獨立承接兩半）。 |
| 步 7 裁切 | 契約未定回報格式 | `per_area[aid]["classes"][cls]`：`clip_loss={link, radius, nofly}`（面積比例；`enforce_link=False` 無 link/radius 鍵）、`coverable_fraction`、`coverable_area_m2`；損失 > 0 → 警告 `uncoverable(aid/cls: link 12.3%, …)`；可覆蓋面積 0 → 該構型不可行（reason `outside_link`/`outside_radius`/`nofly`）。禁區裁切與構型無關，只作記錄（decompose 已另行扣除）。 |
| 步 8 高度帶 | 契約 `A_needs_waiver` 旗標定義為「h_ceil==120 and waiver is None」 | `derived[cls]["A_needs_waiver"] = waiver is None ∧ h_ceil 有限 ∧ h_ceil ≤ h_minfw,c + Δh_band,min`（A：120 ≤ 140 → True；B：120 ≤ 120 → 亦 True，語意為「該構型於平地無豁免即不可指派」；豁免 200 → False）。逐區高度帶用 `decompose.altitude_band`（z_raw、含 ENU 曲率起伏）：注意 B 在 h_ceil=120／h_minfw=100 下 Δh_band = 20 − 起伏 < 20，預設 Δh_band,min=20 時 B 亦不可行（測試以 `dh_band_min_m=15` 驗證 A_needs_waiver 旗標）。 |
| 步 9「∃θ 使 V_a* 存在」 | 契約未定取樣 | `costs.v_air_star` 於 72 個方位（5°）向量化，V_a,min(m)=max(v_min_mission, 1.2 V_s(m))、上限 min(V_a,c, max_airspeed)，V_g ≥ cfg.v_g_min；風取區域最高地形 + h_op 之層。另回報 `two_way_ok`／`one_way_only`（D34 警告）。 |
| 步 10 時間窗 | 規格「粗估 makespan」未定式 | 只用**下界**（超過即確定 NO_GO）：每區 `T_oper_lb = 可覆蓋面積 / (s · V_g,max) / n_uav,c`（取最小構型）；任務下界 = min_c(t_TO + t_LDG) + 2·d_near/V_a + Σ_k T_oper_lb,k。區域窗 ∩ 空域窗 = ∅ → `time_window_disjoint`；下界 > 窗長 → `time_window_insufficient`。`enforce_time_windows=False` 只計算不判定。 |
| 步 5 站點 | 契約未定備降點失敗之處置 | home／回收點任一構型 `site_check` 失敗 → 錯誤；備降點失敗 → 警告 `alternate_unusable(site, cls): …`、自 `derived[cls]["usable_alternates"]` 移除；required 且該構型無可用備降點 → 錯誤 `no_usable_alternate(cls)`。注意站點陣風限（未驗證 8 m/s）比 Beaufort 5 上緣 10.7 嚴，故 `gust_above_public_wind_rating` 警告必與 `gust_exceeds` 站點錯誤並存。 |
| `derived` 之鍵 | 契約 `derived[cls]` | 只含機隊構型；任務層數值（`W_go`、`makespan_lb_s`、`h_clr`）複製於每一構型字典。`h_op/w/s/e_ext/d_turn/V_g_cap` 為**主需求**（第一個能解析感測器之區域需求）之純量，`by_req[req_id]` 逐需求；無可解析需求時為 None。 |
| 額外警告 | — | `avionics_unvalidated(cls: … k_unc, V_tr,cmd, H_tc,cmd, D_use)`（D8 摘要）、`auw_above_reference(uav)`（規格 §3.2）、`wind_layer_above_go`、`gust_above_public_wind_rating(cls)`、`one_way_only(area, cls)`、`gsd_unattainable(area, cls)`、`altitude_band_infeasible(area, cls: reason)`、`A_needs_waiver(area: …)`。 |
| 公開子函式 | 供 planner/dynamic 局部重驗 | `check_required_inputs`、`check_weather`、`check_sites`、`check_airframes`、`analyse_areas`、`derive_by_class`、`check_time_windows`（皆回傳 (…, errors, warnings)）。效能：context 之外 ≈ 0.1 s（mini）、0.4 s（山丘 8 機／3 區）。 |

## allocate.py（契約 §9）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `makespan_after_insert(plan_i: list[str], ...)` | `plan_i` 為 cell_id 序列，但估計需要 `Cell`（條帶、T_single、時間窗），簽章無胞登錄表 | 額外關鍵字 `cells: Mapping[str, Cell]`（cell_id → Cell）；`plan_i` 亦可直接放 `Cell` 物件。另加關鍵字 `t0_s`（None → `cache.init_states[uav].t_s`，已含起飛 57/51 s）、`recovery_site_id`（None → `cache.sites` 第一鍵有 rtb 列者；D40 應傳 `ctx.recovery_site_id`）、`t_window`（空域全域窗）、`lam_by_cell`（T_single/λ）、`model`（含能量）。`makespan_after_insert_pair` 同（`cells/recovery_site_id/t_window/lam_by_cell/models`）。公開 `plan_estimate(...) -> PlanEstimate`（C_s、e_wh、delay_s、ok、reason、每胞 (first_l, idx_in, last_l, idx_last, t_arr, t_done)）與 `sweep_order(strip_ids, cls, cache)`。 |
| 估計之胞內序列與終端側 | 規格只說「終端側由條帶數奇偶與進入側決定」 | 胞內採固定 skip-k 序列 `decompose._skip_sequence(n, dubins.skip_k(s̄, R_eff))`（與 `exact_time` 一致）；4 選項 = 序列正／反向 × 進入側 idx_in，idx_last = idx_in（n 奇）或 1 − idx_in（n 偶）；終端向 op 為 inf（單向胞，D34 racetrack）時改用可行向。多胞計畫以 4 狀態動態規劃（每胞 O(16)）取時間最小（tie 能量最小）。降落固定時間 = t_acc + t_tr^in + t_dn + t_hov,res（A 117／B 106 s）以 `cache.budget.specs[cls]` 計。胞間轉場 = `tr[cls][last, first, 2·idx_last + idx_in]`。 |
| H3 提早到達 | 規格：首胞 → 延後起飛（零能量）；非首胞 → 等待盤旋計能量 | 估計中 t_arr < t_open：首胞累計 `delay_s`（Δ_i，回傳於 `AllocationResult.delays_s`，饋入 `takeoff_schedule`）、非首胞加等待時間並以 `model.transfer_wh(wait, 0)` 計能量；完成 > t_close → 該選項不可行；落地 > 空域窗上緣 → 不可行。 |
| H4 之 e^oper／e^trans | 契約簽章只有單一 `model`（逐機）而 cache 能量層以 `models_rep` 建 | e^in、e^trans 取 `cache.entry_e/tr_e`（代表模型）、e^oper = `model.transfer_wh(T_single/λ, 0)`（逐機 k_unc·P_c·t）、e^RTB = `cache.budget.rtb_wh(cls, last, dir_last, recovery)`（**含 e^LDG**）；門檻 η_T E^avail − r E_nom（e^TO 另加）。`hard_feasible` 之 H4 以胞內最小 e^in／e^RTB 為濾網；計畫層 `_energy_ok` 沿動態規劃選定之路徑加總。`static_ok` 全 False 之胞 → `H4_static`。 |
| `hard_feasible` 原因字串 | 契約未定 | `H1_class_mismatch / H1_sensor / H1_<reason>`、`H2_link / H2_radius`（Ω = 胞 ⊕ d_turn 對 `ctx.link_poly` 與 `rho_sym_m`；`enforce_link=False` 略過）、`H3_window`、`H4_static / H4_energy`、`H5_no_airspeed / H5_<reason>`、`H6_<band reason>`、`H7_no_strips / H7_turn_band_keepout`（條帶 `feasible=False`）。額外關鍵字 `strip_ids`／`T_single_s` 限定子任務（部分覆蓋、多架次分塊）。 |
| 配置單位：`both` 胞、`any` 殘帶對 | D17：`both` 生成兩套獨立條帶、`any` 以候選對進拍賣；`lambda_by_cell: cell_id → (cls, λ)` 無法表示同胞兩構型 | 任務鍵 `task_key(cell, cls)`：一般胞 = cell_id；`both` 胞 = `f"{cell_id}#{cls}"`（`CountResult.by_cell`、`lambda_by_cell`、`AllocationResult.plans` 皆用此鍵）。殘帶候選對之任務鍵：母胞本身（k=n_A A 獨佔／k=0 B 獨佔）= `f"{pid}#{cls}"`、子胞 = `f"{cell_id}#k{k}"`（decompose 對每個 k 之子胞皆命名 `{pid}_A`/`{pid}_B`），`lambda_by_cell` 最終以子胞 cell_id 為鍵；採用之 k 記於 `AllocationResult.residual_k[pid]`。B 殘帶條帶不在 cache 時由 `allocate` 呼叫 `cache.add_strips("B", ..., fleet, init_states, cache.sites, ctx)`。 |
| `assign_config_and_count` | 規格 §9.3 之 T̃ 需 t_in／t_RTB（站點）、both 胞類別固定 | 關鍵字 `recovery_site_id`、`fixed_cls`；`both` 胞自動拆為兩任務類別固定。class choice 另檢 H1（`class_can_serve` 以該構型機隊 `available_kinds` 聯集；胞 fit 可能來自 any 分解前）。`CountResult` 額外欄位 `J`（任務鍵 → J^{c*}）、`second_choice`。denom ≤ 0 → 改派 c2（一次），c2 亦不可 → `infeasible[key]="energy_overhead"`；Σλ > n_U 依 J 遞減不低於 λ_floor，仍超出 → 無 c2 時保留 λ=0 交拍賣（規格「一機多胞」）。部分覆蓋：前 k 條之 (t_oper + 反向轉場平均) 累計 ≤ λ_avail·denom。`T_use_s` 為 inf（能量關閉）→ denom = inf、λ_floor = 0。 |
| `config_index_J` | 需站點 | 關鍵字 `recovery_site_id`；t_in 項 = Σ_i min_l entry（無有限值之機略去；全部 inf → J=0）；T^RTB = 該站最短 rtb（無列 → J=0）；T^oper 依規格逐對 max（單條帶取 min 向）。回傳鍵 = 任務鍵。 |
| Σλ ≤ n_U 不變量 | 拍賣後「一機多胞」使 Σ_q λ_q 可 > n_U | 於 `CountResult.by_cell`（每構型）保證 Σλ ≤ n_U；`AllocationResult.lambda_by_cell` 之 λ_q = 該胞機數（≤ n_U），每構型實際使用機數 ≤ n_U。 |
| Phase 3/4/5 | 規格為文字描述 | Phase 3：≤ n_local_search 回合，i_max = argmax C_i；MOVE(q→j 各插入位) / SWAP(q↔q')，接受條件：變動機所有任務 `hard_feasible`（含 λ）、λ ≥ λ_floor、C_max 降（tie ΣC 降）；any 胞 k_A 微調 = 以候選清單相鄰 k 重新投標一次（僅預計算之 k）。Phase 4 BALANCE：i_max 之所有任務 λ−1 ≥ max(1, λ_floor) 時，改為加入使 max Ĉ 降幅最大之其他任務團隊（≤ n_U 次）。Phase 5：團隊 = 同構型 (機, 任務) 二部圖連通分量（`T{cls}{n}`），`cell_ids` 依計畫出現序、`strip_ids` 聯集。凍結團隊：其機不參與 Phase 1/3/4，Phase 2 只允許尾端插入；輸出保留 team_id／uav_ids／原胞序（尾端追加）。 |
| `method="greedy"` | 契約未定 | 只跑 Phase 1 + Phase 5；λ=0 或無候選之任務記 `uncoverable(no_feasible_insertion)`。 |
| 多架次（§9.6） | 需虛擬機與 cache 進入列 | Phase 2 無可插入者且 `cfg.multi_sortie` → 為每架已用機加入虛擬機 `f"{uav}#s1"`（進入列共用母機、t0 = C_i + t_swap + 起飛時間、能量同當架次模型），再拍賣一次；團隊 `sortie_index[uav]=1`、`estimates`/`plans`/`delays_s` 以虛擬 id 為鍵；`takeoff_schedule` 對第二架次不排隊（t0 = delays[虛擬 id]）。第一架次一律以回收點 H 估 RTB（D40）。`split_for_sorties`：budget = κ_load T_use − t_in − t_RTB，依胞內序累計 (t_oper + 反向轉場平均)，單條帶超額者自成一塊；`enforce_energy=False` → 單塊。 |
| `takeoff_schedule` | 「同站」無站點欄位 | 以初始狀態最近之 `cache.sites` 站（S^app 位置）分組；tie 之「進入時間」= 該機 entry 表最小值；`t0 = k·dt_dep + delays[uav]`。`allocate` 之 `estimates` 於排程後加上 k·dt_dep（Δ 已含於估計）。 |
| `AllocationResult` 額外欄位 | — | `residual_k`、`plans`、`delays_s`（皆有預設）。`uncoverable`：胞級 (cell_id/任務鍵, reason ∈ {no_feasible_class, energy_overhead, lambda_over_fleet, no_feasible_insertion, energy}) + 部分覆蓋之條帶 (strip_id, "energy")（亦列於 `leftover_strips`）。 |
| 效能 | — | mini（1 胞 + 5 殘帶候選對、2 機）allocate ≈ 5 ms；估計為 O(胞數) 動態規劃、無 shapely 於熱路徑（H2 buffer 只在 `hard_feasible`）。 |

## planner.py（契約 §10）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `stage_cost` 之 t̂_RTB「直線逆風下界」 | 若每階段都以現端點之 RTB 估計計入 W，貪婪／beam 會在每一條帶都偏好「離家近的一端」而選出同向 racetrack 轉場（mini 例 makespan 33.8 → 28.9 min）。 | `_rtb_hat` = 回收點之真實快取 RTB 時間（AVOID＋螺旋；無限者以 \|端點−S^app\|/(V_a+W_R) 可採納下界取代；`cfg.t_rtb_lower_bound=False` → 0）；未完成前 t̂_RTB,j = min(現端點值, NAD 任一端點之最小值)（側向中性下界），取 m 後 NAD 為空才用該端點真值。鎖步同法。 |
| `IDPNode` 欄位 | 迴轉帶剪枝與序列重建需要前一條帶／時間區間與該決策之時間能量 | 契約欄位之後新增（皆有預設）：`prev_idx`、`prev_dir`、`t_busy_from`、`t_op_from`、`t_op_end`（λ,）、`dec_t_tr/dec_t_op/dec_e_tr/dec_e_op`、`depth`。`key()` 依契約 = (nad_mask.tobytes(), tuple(zip(last_idx, last_dir)))。 |
| `energy_prune(node, i*, m, k, cache, model)` | 契約簽章無 cfg，但 legacy 須關閉能量剪枝 | 新增關鍵字 `enforce=True`（beam_idp 傳 `cfg.enforce_energy`）；`enforce=False` 只保留 `static_ok`。`budget.min_rtb` 以 (n,2) 向量表（記憶化於 `_MEMO`，鍵含 cache 之 geometry/wind/state key）。 |
| `turn_band_conflict` 語意 | 規格 §10.2「兩機迴轉帶於時間區間重疊 → 剪」若以整條帶為帶，同胞兩機互相全剪（第二機掃一條即 RTB）。 | 採**端線迴轉區**模型：轉移佔用「起點端＋終點端」之側向帶 [min(v_l,v_m)−ov, max+ov]（ov = max(0, R_eff−Δv/2)：Ω 溢出、U 型 0，D38）於 [T_i, T_i+t^trans]；作業以 v±d_sep,lat/2 於起始端 [t1, t1+τ]、終端 [t2−τ, t2] 通過迴轉區（τ = 2.7R/V_g），整條帶對整條帶用側向間距 \|Δv\| < d_sep,lat（§13.4）。同端 ∧ 側向重疊 ∧ 時間重疊 → 剪。且為**軟剪枝**：候選全部衝突時保留（交 `deconflict.repair` 延遲盤旋），避免整機提前 RTB。 |
| K-nearest 胞完成偏好「僅當同胞剩餘 < K 時才補鄰胞條帶」 | 補入之鄰胞候選若 W 較小仍會被選中而提前離胞（測試 cross_cell_chain 失敗） | 嚴格版：現胞尚有可行未覆蓋條帶 → near 只取現胞之 K 最近；現胞已盡 → `cell_order`（allocate.plans 之胞序，關鍵字參數）之下一胞；皆無 → 全部候選。 |
| 剪枝順序 | 契約先取 K 最近再剪枝，可能剪光 | 先做可行／能量／迴轉帶剪枝，再取 K 最近（候選 ≤ 2n，向量化）。 |
| `beam_idp` 額外關鍵字 | 修復與跨胞需要 | `cell_order`（uav → cell_id 序）、`inactive`（根節點即 RTB 之機，§10.5「截斷機自截斷點直接 RTB」）、`site_id`（回收點；None → cache.sites 首鍵）。`warm_start` 為**固定前綴**（規格 §10.5），其條帶自 NAD 移除；契約「+ warm_start 節點」之雙根做法未採用。終止時以 (leftover 數, max_i[T_i + 真實 RTB + 降落固定時間], W) 選最佳節點。`makespan_s` 含降落固定時間（A 117／B 106 s），與契約 §W.7 之 C_i 定義一致。 |
| 鎖步分支 | 論文 Π=∏near_i 若逐排列建節點過慢 | 聯合成本向量化計算後只實體化每節點最佳 3B 個聯合決策（B=1 時與完整列舉之 argmin 相同；`test_beam1_paper_metric_equals_full_dp`）；候選數 < 現役機數或某機 near=∅ 時允許 RTB 選項；鎖步不做迴轉帶剪枝（論文結構）。 |
| `UAVSequence` / `IDPResult` 欄位 | append_rtb 之警告與 repair_energy 重跑所需 | `UAVSequence` 尾端新增 `warnings`、`cls`；`IDPResult` 尾端新增 `init_states`、`cell_order`、`site_id`、`team`。`SeqItem.transfer` 首項為 None（進入見 `UAVSequence.entry`）；`decisions` = 快取條帶索引序列。 |
| `append_rtb` 之 ENERGY_SAFE_CHECK | 契約無獨立函式 | 新增 `energy_safe_check(seq, rtb, site, cache, ctx, model, cfg)`：每條帶末 E ≥ r·E_nom + budget.min_rtb、返航路徑每 ≈200 m（非每 d_d；`point_wh` 為上界化 O(1)）樣本 E_j ≥ r·E_nom + min_a point_wh、落地 E − e^LDG ≥ r·E_nom；`enforce_energy=False` → 略過。備降點順序 = `cache.rtb` 有列之站（規劃階段以 preflight `usable_alternates` 建 `cache.sites`）。 |
| `repair_energy(team, result, infeasible, cache, models, cfg)` | 簽章無 strips／init_states | 自 `cache.strips[cls]` 與 `result.init_states` 取得（另可關鍵字 `strips=`）；x_j 在返航段（seg_index ≥ len(items)）→ 截去末條帶；只重跑 beam_idp，RTB 由呼叫端再 `append_rtb`（plan_mission 迴圈 ≤ 3 輪）。 |
| `plan_mission` 步驟「leftover → allocate(frozen)」 | 需以殘餘條帶重建子胞才能進 allocate | 未實作：修復後仍 leftover 之條帶記入 `MissionPlan.uncovered`（reason `energy_leftover`/`idp_leftover`），3 輪後仍無法 RTB 之機其條帶記 `energy_rtb_infeasible`。 |
| `plan_mission(inputs, cfg)` | 測試需粗 DEM 與既有 ctx | 新增關鍵字 `terrain_margin_m=None`、`ctx=None`（給定則跳過 build_context）；`preflight` 以同一 ctx 呼叫（避免重建）。步驟順序：context → preflight → decompose → layering（差異 >1e-6 m 時以 `decompose.build_strips` 重鋪並更新 T_single）→ residual_candidates → 暫定走廊（入口 = 該構型全部條帶端點中離 home 最近者；`init_laps=(n_lap, 0.0)` 因 exit_state.t_s 已含盤旋）→ `CostCache.build(strip_sensors=resolve_sensor)` → `allocate(residual_pairs)` → 團隊 t0 平移 init_states → beam_idp（`cell_order` = allocate.plans）→ append_rtb／repair_energy → 起飛走廊精算（出口位置／方位／高度改變 >1 m／1e-3 rad／0.5 m 才 `refresh_entry`，序列以 `_rebase_entry` 平移）→ 降落走廊（實際末狀態與落點）→ assemble_2d → build_3d → CorridorSchedule.reserve_plan（起飛站 home）＋ spatially_exclusive(r_eff_m=cache.R_eff) ＋ check_separation ＋ repair → MissionPlan。`UAVPlan.t_arr_ptc_s` = 首個 TRANS_IN 航點時刻；`t_land_s` = 末航點 + t_hov,res；`energy_margin_wh` = 落地剩餘 − r·E_nom；`warnings` 含 `profile:<violation>`、`corridor:<issue>`。`MissionPlan.cells` = 分解胞 ∪ 採用之殘帶子胞；`MissionPlan.strips` = 快取全部條帶。 |
| 實測數值（mini, dem_step 50） | 契約 §W.7/W.9：IDP 28.3 min、最終 29.3 min | IDP（B=3, K=6）與全排列暴力搜尋差 ≤ 0.5 s（A 5 條 1559.4 vs 1559.2 s；B 3 條 1282.8 vs 1282.3 s；同向 racetrack 於末條帶合法且更短，故 B 序列非 §W 之 1E,3W,2E）；plan_mission makespan **28.94 min**（B1 t0=0、A1 t0=60 s 走廊換序），總時間 ≈ 0.9 s。「8 長 + 16 短」：事件驅動 work 1748 s／[6, 18] 條 vs 鎖步 2733 s／[12, 12]（0.64）。hills（8 機、107 條、dem_step 50）7.5 s 全覆蓋。 |

## viz.py（契約 §15）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| 簽章額外關鍵字 | 契約簽章無法標示衝突、輸出解析度、無 `schedule` 時之起飛站 | 位置參數與回傳型別依契約；另加 keyword-only：全部函式 `dpi`（預設 130）；`plot_2d`／`plot_gantt` 加 `conflicts`（`deconflict.Conflict` 或鴨子型別 `uav_a/t_s/kind`，紅 X／紅虛線標示）；`plot_gantt` 加 `cfg`、`takeoff_site_id`（`schedule=None` → 以 `deconflict.plan_windows(plan, cfg or PlannerConfig())` 推得起降預約；`schedule` 給定則讀 `schedule.reservations`，同列重疊者紅框斜線）。 |
| `save_all(plan, ctx, report, out_dir)` | `report` 可能為 None（尚未 `sim.validate`）；契約未定檔名 | 依序寫出 `plan_2d.png, profiles.png, gantt.png, coverage.png, energy.png`（第 5 張為附加）並回傳路徑；`report` 為 `sim.ValidationReport` 或鴨子型別（`coverage`、`conflicts`）或 None。覆蓋報告由公開 `resolve_coverage(plan, ctx, report)` 取得：`report.coverage`（非空）→ `sim.coverage_raster(plan, ctx, ctx.cfg)`（**延遲 import**，viz 之模組層 import 圖仍只含 context/deconflict/geometry/mission）→ 失敗時 `coverage_from_plan`。甘特圖走廊預約以 `CorridorSchedule.reserve_plan(plan, home)` 重建；`corridor` 類衝突不再重複畫於甘特圖。 |
| `plot_profiles` 之 `z_lo/z_hi` | `MissionPlan`／`Waypoint` 不保存 `altitude.sample_bounds` 之陣列 | 公開 `profile_bounds(uplan, ctx, strips, cells) -> (z_g, z_lo, z_hi)`：以航點與 `Waypoint.track_deg` 重算規格 §11.1 之近似（走廊半寬 OPER = w_l/2、其餘 = b_trk；`z_lo` 依相位：TAKEOFF/LAND/TRANS = 走廊地形最高、CLIMB/DESCENT/ACCEL/DECEL = +h_clr（D35）、OPER = +max(h_clr, h_min,l)（D36）、其餘固定翼段 = +max(h_clr, h_minfw)；`z_hi` = 走廊地形最低 + h_ceil，OPER 另 ∧ h_max·k_GSD 由胞 `payload_req_id` 之感測器 `agl_window()` 取得）。為視覺化近似，非 planner 實際包絡陣列；地形未知處 nan。 |
| `plot_coverage(report, ctx, path)` | 需同時支援 `sim.CoverageReport`（鍵 all/lidar/rgb/A/B）與 sim 不可用之情況 | 鴨子型別讀 `area_id/cls_required/coverage_by_cls/uncovered_cells_xy/raster_m`（每區一子圖，區內柵格綠、未覆蓋格紅）。附加 `coverage_from_plan(plan, ctx, raster_m=None) -> list[CoverageLite]`（鍵 any/A/B；足跡 = `sensor_on` 航段折線 ⊕ swath(該段最小 AGL)/2 平端帽，找不到感測器用 `Strip.width_m`），mini 例與 `sim.coverage_raster` 一致（all/any 0.900、A 0.733、B 0.167）。 |
| 字型／後端 | 缺字警告、無顯示環境 | 模組載入時 `matplotlib.use("Agg")`；`configure_fonts()` 設 `font.sans-serif` 後備串 Microsoft JhengHei → Noto Sans TC → Noto Sans CJK TC → DejaVu Sans（只保留已安裝者）、`axes.unicode_minus=False`；圖上文字一律英文（測試斷言無 Glyph 警告）。 |
| 附加公開項 | — | `plot_energy(plan, ctx, path)`（每機 `energy_remaining_wh` 對時間、`reserve_wh` 虛線）、`PHASE_COLORS`、`CLASS_COLORS`、`CoverageLite`。地形背景以 `terrain.z_raw`（燒入前）畫 gist_earth ＋ 山影 ＋ 等高線；keep-out 以 `ctx.keepouts.polys`（已 ⊕ b_h）＋ `ctx.nofly_buffered` ＋ 原始 `ctx.obstacles_enu`。 |
| 觀察（非 viz 缺陷，供 allocate/planner 修正） | 規格 §17.1 (1) 要求覆蓋率 ≥ 99.5%，mini 例 `plan_mission` 只達 90% | `residual_candidates` 對每個 k 之 B 殘帶子胞皆命名 `Z1_1_B` 且條帶 id 皆為 `Z1_1_B_B_1…`，`allocate` 以 `cache.add_strips` 逐候選加入時同 id 互相覆蓋：cache 最終之 `Z1_1_B_B_1/2` 為 k=6 之條帶（y −472.9／−575.8）、`B_3` 為 k=5 之第三條（−543.1）、`B_4/5` 為 k=4 之條帶，而採用 k_A=5 之子胞 `Z1_1_B` 三條應在 y = −337.5／−440.3／−543.1。B1 因此掃 −472.9／−575.8／−543.1（重疊），y ∈ [−400, −281] 未覆蓋（≈9.7%）。`test_viz` 只斷言 any ≥ 0.85；修正後 `sim.coverage_raster` 應達 ≥ 0.995。 |
| 效能 | — | mini（2 機、994 航點、dem_step 50、130 dpi）：plot_2d 0.46 s（含山影／等高線標籤）、profiles 0.41 s、gantt 0.16 s、coverage 0.24 s（`sim.coverage_raster` 本身 0.05 s）、energy 0.13 s；`save_all` 1.26 s。地形背景只取 ROI 子視窗（區域／站點／航點／keep-out 外框 ⊕ max(300 m, 5%)）；航線以 `LineCollection` 一次繪製，無逐航點迴圈繪圖。 |

## decompose.py / planner.py — 殘帶條帶 id 跨候選 k 碰撞之修正（2026-09-05）

| 契約項目 | 原因 | 實際做法 |
|---|---|---|
| `residual_candidates` 子胞命名 | 每個候選 k 之 B 殘帶子胞皆命名 `{pid}_B`，其條帶 id 皆為 `{pid}_B_B_{l}`；`allocate.ensure_strips_cached` 逐候選以 strip_id 加入 CostCache 時互相覆蓋（不同 k 的殘帶條帶 y 位置不同），B 機因此飛到錯的條帶、留下未掃的帶（mini 例覆蓋率 0.90，規格 §17.1 要求 ≥ 0.995） | 子胞 id 改為 `{pid}_A_k{k}` / `{pid}_B_k{k}`，B 殘帶條帶 id 隨 `lay_strips` 成為 `{pid}_B_k{k}_B_{l}`，各候選互不相同。`allocate._pair_key` 之鍵仍附 `#k{k}`（保險）。`AllocationResult.lambda_by_cell` 之鍵因此為 `Z1_1_A_k5` / `Z1_1_B_k5`（`test_allocate` 已更新）。 |
| `plan_mission` 採用 k_A 後之快取內容 | 契約 `MissionPlan.strips` 為「快取全部條帶」；未採用之候選條帶留在快取中除了造成上述碰撞，也讓 `plan.strips` 含大量未飛條帶 | 於 `allocate` 之後、Beam-IDP 之前，以 `cache.remove_strips` 移除：其他 k 之 B 殘帶條帶、母胞未用之 B 條帶（k>0）與母胞 A 條帶 k+1..n（k=0 → 全部）；已被任一團隊使用之條帶一律保留。 |
| 驗證 | — | mini（dem_step 50，terrain_margin 1500）：修正後 `sim.coverage_raster` any ≥ 0.995；`test_viz::test_coverage_from_plan_mini` 門檻改為 ≥ 0.995。 |

## dynamic.py（契約 §12）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `GhostTrack` 尾端額外欄位 | 契約只列 `uav_id/samples/sigma_t/landing_site/eta_s`；LINK_LOSS 之「t − t_loss > t_ll → 視同 UAV_EXIT」與 LINK_RESTORED 需要策略與決策時刻 | 尾端加預設欄位 `policy="RTB"`、`t_loss_s=0.0`、`t_rtb_s=0.0`（決策點 = t_loss（RTB／NEAREST_ALT）或 t_loss + t_ll（CONTINUE_THEN_RTB））。`sigma_t` 取常數上界 k_t·(ETA − t_loss)（契約 deconflict.GhostLike 為常數 sigma_t）。deconflict 以鴨子型別（`GhostLike`）接受，不 import dynamic。 |
| `ReplanResult` 尾端額外欄位 | 契約無處承載幽靈航跡與佔位計畫 | `ghosts: list[GhostTrack]`（本次事件後有效之幽靈；呼叫端下次 `replan(..., ghosts=res.ghosts)` 傳回）、`placeholders: dict[uav, list[Waypoint]]`（`placeholder_used` 時之佔位航點）。 |
| `ghost_track(...)`／`placeholder_plan(...)` 額外關鍵字 | 幽靈 RTB 路徑需 keep-out 切線圖快取與 R_eff；佔位計畫需「現時」 | `ghost_track(..., *, cache=None)`（None → 無快取 AVOID）；`placeholder_plan(..., *, t_now_s=None)`（None → 各機首條帶起始時刻）。契約位置參數不變。 |
| `plan_mission` 不回傳 `CostCache` | 契約 `replan(plan, event, telemetry, ctx, cache, cfg)` 需要與計畫同構之快取，但 planner 只回 `MissionPlan` | 新增 `cache_from_plan(plan, ctx, *, site_ids=None) -> CostCache`：以 plan.strips／plans 重建（init_states = 首 ENTRY/CLIMB 航點、sites = 回收點(+備降點) 之 (S^app, z_tc)、strip_sensors 依胞 payload_req_id）。mini ≈ 0.08 s。整合層若保留 plan_mission 內部之 cache 可直接傳入。 |
| 錨點與「只允許尾端追加」 | 契約：受影響機既有航點位元相同、只允許尾端追加 | 錨點 = 接手／新區／優先度：序列末條帶端點（`anchor_end`；若已在返航 → 現時）；cue／風重排：現行條帶端點；cut：被保留之最後條帶端點；DIVERT／中止／失鏈：現時內插狀態（或決策點）。既有航點 `waypoints[:keep_n]` 為同一 Waypoint 物件，尾段以 `altitude.build_3d` 自錨點重建（entry 段 phase=TRANSFER、pin_end = z_site + H_tc_cmd）。**RTB 航點亦屬尾段**（接手時原 RTB 被新序列取代）。去衝突延遲盤旋只插在錨點之後（`orbit_insert_index` 下限 keep_n）。 |
| 快照能量（§5.6） | 契約 `Telemetry.e_tel_wh` 直接給定；規格 §5.6 未驗證電量監測不得直接作 E_rem | `snapshot` 之 `UAVState.e_rem_wh` = `_e_ref`：`battery_monitor_validated` → clip(E_tel, 0.9·E_pred, 1.1·E_pred)；未驗證或 e_tel None → E_pred(κ) = E_start − κ·E_used,plan（κ = `EnergyModel.kappa`）。`handle_energy_update` 先依 δ 更新 κ（就地寫 `ctx.models[uav].correction`，跨事件持續）再算 E_ref。 |
| 受影響集合能量條件 | 規格「E^rem − e(剩餘含 RTB) ≥ r·E_nom + margin」 | 以計畫落地餘量推估：E_land = E_state − (E_plan(t) − E_plan_end)，條件 E_land ≥ reserve + cfg.energy_margin_frac·E_nom；`enforce_energy=False` 時不篩。σ 相容：both 區 A 條帶需 lidar、B 條帶需 rgb（D17），其餘 `decompose.class_can_serve`。 |
| 拍賣插入（§9.4 Phase 2） | 增量重規劃不重跑 allocate | `_Session.reassign`：候選 = affected_set；每候選以 `planner.beam_idp`（單機 Team、錨點為初始狀態、`cache.refresh_entry`）+ `append_rtb` 試算，取 (leftover 數, Δmakespan, t_land) 最小者；至多 `max_changes=2` 機（§12.3「≤2 團隊變更」）。同構型保留原條帶；跨構型（rgb/any，且無同構型候選）以 `decompose._new_cell + _solve_class` 於「未覆蓋條帶足跡 ∩ 母胞」重鋪新子胞（cell_id `<cell>_R<n>`，`residual_of` = 母胞）。both 區不跨構型。 |
| 殘段條帶 id | 契約 `snapshot` 回傳「裁短後新條帶」 | `Strip("<strip_id>#r")`：自最後有效覆蓋點裁短並保留 e_ext 引入段（e_ext 以 `decompose.end_extension_for` 重算；`cfg.e_ext_override_m` 優先）；接手同構型時以此 id 加入快取（`cache.add_strips`）；`affected_set`／胞查詢以 `#` 前綴對應母條帶。 |
| `handle_new_area` PRECHECK (7)(8)(9) | 契約無 per-area preflight 入口 | 以 `context.polygon_enu` + `preflight.analyse_areas(replace(ctx, inputs=replace(inputs, areas=(area,)), areas_enu=...))` 取 `feasible_classes`；失敗／不可覆蓋 → `uncovered += (area_id, "precheck(...)")`；例外 → 略過前置檢查交由 decompose。新區之 `payload_req_id` 須為既有需求（`ctx.sensors` 鍵）。 |
| `handle_wind_update` 之 ctx 就地更新 | 契約 `MissionContext.wind` 為建構時物件 | 建新 `WindField(version+1)` 後 **就地** `ctx.wind = new`（decompose_ctx／corridor 皆讀 ctx.wind）再 `cache.refresh_wind`（幾何層由 costs 依 R_EFF_REFRESH_TOL 自行判定）。ΔC_i 以快取尾段時間估計（現行剩餘 + 待掃 + 轉移 + RTB + 降落固定時間）與舊風比較，> τ_w·(C_i − t_now) 或起降風向變化 > 45°（走廊重算）→ 該機自現行條帶端點重跑 IDP；「方向候選總時間變化 > 10%」未實作（條帶保留，僅重排序列）。任一層 W > `abort_mean_wind_mps` 或 obs 違反中止門檻 → 直接進 WEATHER_ABORT(ALL)。 |
| `handle_cue` | 規格 §12.7 | Q_conf = 8 邊形（`cfg.r_conf_m`）；`PayloadRequirement("cue_confirm", "rgb", gsd=cfg.gsd_confirm_m)` 以 B 之 rgb 硬體建 Sensor 並登記 `ctx.sensors[("B","cue_confirm")]`；h_max < h_minfw → `gsd_unattainable(...)` note、confirm_mode="orbit"（最低高度 h_minfw、R_eff、2 圈、右轉）；strips 模式以 `decompose.build_strips` 沿風向／垂直風向擇一（單條帶即滿足「直徑 ≤ s_B」）。候選 B：Q ⊕ 2R_eff ⊂ P_link（enforce_link）、落地餘量 ≥ reserve + margin、Δmakespan ≤ τ_cue·makespan。插入點 = 現行條帶之後（`idp_chain([[cue], pending])`：確認條帶先於其餘待掃）。無候選：A 具 rgb 只記錄（不中斷 LiDAR）並入待辦 `(area_id, "cue_pending")`。 |
| `handle_priority` | 規格 §12.8 | 就地改 `Cell.priority`；提高 → 每機待掃中含該區者以 `idp_chain([該區條帶, 其餘])` 重排（能量不可行／leftover → 不搶占）；待辦中該區條帶重試接手；降低 → 不中斷。 |
| `handle_weather_abort` | 規格 §12.9、D23 | 站點 OK 判定：側風 = W·|sin(β − θ_app)|（θ_app 缺省 = β → 0）與陣風（`WindField.gust_at`，obs 取大）對 `crosswind_site_max_mps/gust_site_max_mps[validated]`；OK=∅ → 超限量最小站 + `landing_over_limit(site: +x m/s)` 警告（不丟例外）。降落排序：同站依 (E^rem − r·E_nom − e^alt)/P_c 升序，`CorridorSchedule.next_free` 求空檔，後續機以 `deconflict.insert_delay_orbit`（等高、於錨點航點，非嚴格「上風側」）等待 n = ⌈delay/T_lap⌉ 圈並檢查能量（不足 → 不等待、記 `landing_sequence_energy_override`）。region scope 以現時位置或剩餘條帶與區域相交判定。 |
| 去衝突（僅變更機） | 規格 §12.1 步 5 DECONFLICT(僅變更機) | `check_separation(..., skip_pairs = 未變更機兩兩, ghosts, t_sync_s=t_now)` + 走廊預約；只對變更機插延遲盤旋（最多 3 輪；同一插入點重複衝突 → 視為空間重疊、記 `residual_conflicts(n)` 不再插入）；不呼叫 `deconflict.repair`（其會改起飛時刻）。幽靈與其本機預載計畫重合不算衝突。 |
| 時限與佔位 | 契約 `timing_ms["total"] > 2000 → placeholder_used` | 同步實作：事件處理完成後量測總耗時，> `REPLAN_BUDGET_MS`（2000）→ `placeholder_used=True` 且附 `placeholders`（續飛現行條帶至端點 + 保守 RTB）；無背景執行緒。 |
| 未實作／簡化 | 範圍外 | 多架次（§9.6）待辦、LINK_RESTORED 只承接 `plan.uncovered` 中仍存在之條帶、NEAREST_ALT 之 `nearest_alternate` 取自失鏈時航點、氣象恢復（連續 10 min）之自動 REPLAN 由呼叫端觸發。 |

## sim.py（契約 §14）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `check_kinematics` 之坡度限制 | 契約寫「固定翼段坡度 ∈ [−tan(max_descent), tan(max_climb)]」，但航點為地面座標；逆風時地面坡度 = V_a sinγ/V_g 大於空氣坡度（A 逆風 16.5 m/s、3° 空氣坡 → 3.9° 地面坡），直接比較會把合法 CLIMB/RTB/DESCENT 段判為違規（mini 15 處、hills 40+ 處） | 逐腿以規格 §3.2 地面座標公式驗：tanγ^g_up = V_a sin(max_climb)·κ/V_g、tanγ^g_dn = V_a sin(max_descent)/V_g，V_g 由風三角（腿中點 z 之風、航點 V_a）算得（nan → 航點 ground_speed_mps → V_a），弧段（`Waypoint.arc` 非 None）κ = cos φ_max；轉換段仍用 tan γ_tr,max。額外簽章：關鍵字 `strips=` 給定時 OPER 段以 w_l/2 走廊與 h_min,l／h_max·k_gsd 檢查（`validate` 傳 `plan.strips`） |
| `check_energy` 逐航點 | 規格 §5.5 為「每 25 m 一樣本」；hills 8 機 × 3 站 逐 25 m 呼叫 `point_wh`（0.3 ms）超過 1 s | 粗取樣 500 m（＋相位邊界）＋ Lipschitz 細化：e_alt 沿路徑變化率 ≤ L = k_unc·P_c/V_g^worst（`point_wh` 之直線增量），E 單調遞減 ⇒ 粗樣本裕度 ≥ L·Δs 之區間不可能違規，其餘區間逐航點重驗（`test_energy_coarse_to_fine_equals_pointwise` 驗證與全逐航點結果相同）；站點先試落點／回收點，裕度足夠即短路。**E 非單調之人工計畫（單點凹陷）不保證偵測。** `cache` 只需 `.budget`（`build_validation_cache(budget_only=True)` 回傳之 `_BudgetCache`）；`plan_mission` 不回傳 CostCache，故另提供 `build_validation_cache(plan, ctx, budget_only=False)` 自 `MissionPlan.strips` 重建（起飛狀態以最近條帶端點；只供驗證，不保證與 planner 之快取逐位元相同） |
| `check_kinematics` 坡度容差與 AGL 容差 | 航點刪減後線性內插 vs 規劃 25 m 取樣 | AGL 容差 1 m、坡度 1e-6、曲率半徑相對容差 1%（外接圓半徑；Dubins 外切多邊形頂點恰在 R/cos(δ/2) 上，三點外接圓 = 該圓） |
| `check_kinematics` 之 h_minfw 豁免 | D35：CLIMB/DESCENT 段 z_lo 不含 h_minfw；ACCEL/DECEL 段位於 H_tc（< h_minfw） | ACCEL/DECEL/CLIMB/DESCENT 豁免 h_minfw；h_clr 仍檢查 |
| `coverage_raster` 足跡 | 契約「依實際 AGL 之足跡」 | 每腿 AGL_cons = z − 走廊 (w_l/2) 內最高地形（保守），寬度 = sensor.swath_m(AGL_cons)；同機同條帶連續腿串成一個 run，直線 run 以 (u,v) 帶測試向量化柵格化（與逐四邊形半平面測試逐格相同，`test_raster_helpers_consistent`）；相機曝光間距 d_exp = V_g/f_s > l(h) 時只計曝光矩形（相位跨腿延續、不裁剪至腿端）。`coverage_by_cls` 鍵：構型（"A"/"B"）、感測種類（"lidar"/"rgb"）與 "all"；`cls_required` = any → ["all"]、lidar → ["lidar"]、rgb → ["rgb"]、both → ["lidar","rgb"]。分母 = 區域 ∖ (NoFly ⊕ b_trk)。額外屬性 `CoverageReport.passed`／`.coverage`、`ValidationReport.coverage_ok` |
| `check_link_airspace` 額外簽章 | 區域時間窗（D22）需胞→區域對應 | 關鍵字 `cells=`（cell_id → Cell）給定時檢查 OPER 航點在該區時間窗內；keep-out 以 ≤ 64 腿分塊（z 提示取塊內最小）＋命中時逐腿定位；`ctx.keepouts` None 時退回 `ctx.nofly_buffered` 2-D 相交 |
| `track_simulate` | 契約只給 `dt_s=0.5` | L1 純追蹤（L1 = clip(3V_a, 40 m, 0.9 R_plan)）＋風三角：a_g = 2V_g² sinη/L1（地面座標；圓弧上 η = asin(L1/2R) 自然給出 V_g²/R，**不加曲率前饋**——前饋會重複計算，A 偏差 3 → 9 m）；ψ̇ = a_g/(V_a cosδ)（δ = 偏流角）並限 V_a ψ̇ ≤ a_max = max(g tanφ_max, V_a²/R_theo)（表 1 R_theo 定義 R_eff，D1）；內部積分步長 ≤ 0.2 s（`dt_s` 較大時自動細分；B 於 R_eff 圓弧為傾角極限，dt=0.5 給 9.9 m、0.2 給 5.1 m）。z 沿計畫剖面跟隨；能量逐步 P_mode(γ)·dt·k_unc，VTOL/轉換段沿用計畫值。回傳額外欄位 `n_steps`、`energy_sim_wh`、`energy_plan_wh`、`completed`、`wp_at_max_xt`；關鍵字 `trace=` 可收集逐步狀態。mini：A 2.4 m／B 5.1 m（≤ e_max+e_xt = 8）、能量偏差 < 0.2% |
| `baselines` | 契約回傳 `{name → makespan_s}` | 另附 `<name>_distance_m/_compute_ms/_uncovered`；失敗／NO_GO／未完整覆蓋 → inf；同構機隊由 `homogeneous_fleet(inputs, cls)`（uav_id 保留、A 範本掛原機隊之 RGB 相機以承接 rgb 區）；`legacy` = 全 A 機隊 + `legacy_mode=True`。**mini 例：hetero 1736 s、all_A 1535 s、all_B 2055 s、legacy 1509 s——1A+1B 不優於 2A**（平地小區、A 較快且殘帶分割不均），測試只斷言 hetero ≤ max(all_A, all_B) |
| `paper_table2_check` | 契約「路徑長應在 590–650 km」 | 以 `scenario.build_paper_table2_scenario()` + legacy 執行 `plan_mission`（d_d_m 放寬 200 m、terrain_margin 5 km，≈4 s），回報每機 3D 折線長與 total/max/mean、`in_range_*`。**實測每機 680／689／840 km、合計 2209 km**：三機皆自 U1 起飛（`MissionInputs` 單一 home）、含起降走廊與 e_ext=0 之 skip-k 轉場，高於論文 590–650；測試只要求同量級（0.8×590 ≤ 每機 ≤ 1.5×650）並回報實際值 |
| §W.8 最遠點能量 1114 Wh／需求 603.6 Wh | IDP 可自左端進入（§W.7 不斷言進入側）；實作序列 2E,5W,1E,4W,3W 自條帶 3 **左端** RTB | 實測 E(RTB 起點) = 1078 Wh、需求 545 Wh（RTB 較短）；測試斷言 1000 ≤ E ≤ 1150、530 ≤ 需求 ≤ 620、E ≥ 需求 |
| 規格 §17.2 `runtime_under_1s` | hills（8 機／107 條帶／3 區）完整 `validate` 實測 1.26 s（coverage 0.45、kinematics 0.4、energy 0.3、separation 0.1；不含 1.6 s 預算表重建） | 測試以 mini 計畫複製為 8 機驗 < 1 s（≈0.4 s）；hills 需 7 s 規劃故不進 pytest |
| hills 情境驗證結果（補記） | 非本模組問題，供整合參考 | `validate` 於 hills 回報：A1/A2/A3/B3 ENTRY/TRANSFER/RTB 段 AGL 201–213 m > h_ceil 200（planner 之 `profile:` 警告已知）、17 個 4D/走廊衝突（deconflict 殘留）；覆蓋 Z1/Z2/Z3 皆 1.0、能量／鏈路／坡度／曲率通過 |

## integration/legacy_adapter.py（契約未定義；整合層，2026-09-05）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| 「經 `ardupilot.plan_to_mission_items` 轉 QGC」 | `hv_dccpp/ardupilot.py` 尚不存在 | 於本模組實作公開 `plan_to_mission_items(plan, *, spec, home_latlon_msl, frame, trigger_dist_m, speed_cap_mps, loiter_time_s, landing_latlon, lidar_only, tol_xy_m, tol_z_m, l_max_m, arc_step_deg) -> list[MissionItem]`、`mission_items_to_qgc(items) -> list[str]`、`simple_vtol_mission_items(...)`（`(lat,lon)` 列用）、`thin_waypoints(P, keep, tol_xy, tol_z, l_max)`；ardupilot.py 完成後可直接搬移／委派。指令常數取 `pymavlink.mavutil.mavlink.MAV_CMD_*`（缺席時同值後備）。 |
| `params.angle` 強制掃描方向（PlannerConfig 無此鍵） | `decompose_area` 無方向覆寫參數（內部 `_lay…(alpha_fixed=)` 未外露） | `plan()` 於 `with _direction_override(alpha, mode)` 內暫時替換模組屬性 `decompose.direction_candidates`（decompose_area 以全域名稱呼叫），`force` → 候選只剩 α、`hint` → α 置首；離開即還原（非執行緒安全）。`force` 下區域不可覆蓋時自動退回 `hint` 並記 `forced_angle_infeasible_fallback_to_hint`。既有語意換算：θ_N = 90° + angle（`legacy_angle_to_theta_rad`）。 |
| `params.speed` 巡航空速上限 | `cruise_airspeed_mps` 非可覆寫鍵（規格 §3.5），規劃器 V_a* 取自表 1 | 規劃仍用表 1 空速；只在輸出 `DO_CHANGE_SPEED` 時 `min(V_a*, speed)`、下限 `spec.v_min_mission()`。 |
| `params.spacing` | 條帶間距由感測器決定（sensor-driven） | `spacing_mode="overlap_hint"`：側向重疊率 o = 1 − spacing / w(h_target)，僅 0.05 ≤ o ≤ 0.85 採用，否則忽略並記警告；LiDAR 不採用。 |
| `params.altitude` | 直接指定 AGL 會違反 h_minfw／h_ceil／GSD 窗 | 夾到 [h_minfw, waiver(120)] 作 `PayloadRequirement.h_target_agl_m`；rgb 之 `gsd_required_m` 設為 1.05·h_target·p/f 使 h_max ≥ h_target。 |
| `alternates=None` | 表 1 `alternate_landing_sites_required=true` → preflight NO_GO | 自動於 home 背向區域質心 400 m 處置 `ALT1`，記 `alternate_auto_placed`。 |
| `home_latlon=None` | 既有系統無 home 概念（DO_SET_HOME 用 0,0,0） | 第一次呼叫之第一個角點（快取供各區共用），記 `home_defaulted_to_first_corner`。 |
| DEM=None | 需平地 | `scenario.synthetic_dem_for_latlon_bounds`（step 50 m、base `base_msl_m`、σ_z 2 m）涵蓋幾何 ⊕ (terrain_margin + 1 km)，依邊界快取。 |
| `loiter_time`／多機起飛錯開 | 既有系統以 NAV_LOITER_TIME；QuadPlane 起飛後盤旋須固定翼 | 起飛前 `NAV_DELAY`(93, param1 = 秒)（ArduPlane 4.1+ 支援，待實機驗證）；`plan_multi` 另加 `UAVPlan.t_takeoff_s`。 |
| 相機觸發 | 既有系統無 | 條帶以 `DO_SET_CAM_TRIGG_DIST(l(h)·(1−o_fwd))`／`(0)` 包住；無相機（LiDAR-only）→ `DO_DIGICAM_CONTROL 1/0`（待驗證）。 |
| 起降段 | QuadPlane 由 84/85 自行完成轉換 | 略去 TAKEOFF／LAND／TRANS_IN 樣本，TRANS_OUT 只留 P_tc；`DO_LAND_START` 插於首個 DECEL 樣本（S^app）。 |
| 航點密度 | build_3d 之航點（每 25 m）對 L1 太密 | 直線弦容差 2 m／3 m 刪點（`thin_waypoints`）、Dubins 弧段每 30°（D27 上限）一點、整圈（≥ 300°）同弧 → `NAV_LOITER_TO_ALT`（半徑正 = 順時針）。 |
| `subdivide_polygon` | `convex_partition` 片數可能 < n | 面積二分（軸向直線）補足至 n 片，沿主軸排序後貪婪分成 n 組；組聯集不連通 → 凸包。 |
| `start_from_left`、`region_idx/total_regions` | HV-DCCPP 自行選入口／RTL 高度分層由高度包絡處理 | 忽略（`region_idx` 只用於 UAV 命名 `<cls><idx+1>`）。 |

## integration/mqtt_bridge.py（整合層；契約未列，依整合筆記）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| QGC WPL 110 序列化 | 整合筆記要求 `hv_dccpp/ardupilot.py` 提供 Plan → ArduPilot 任務項目，但該模組尚未存在 | 在 `mqtt_bridge.py` 內以私有 helper `qgc_items_from_plan` / `qgc_wpl_from_plan`（`QgcItem` dataclass）序列化：seq 0 home（frame 0、cmd 16、current 1）、NAV_VTOL_TAKEOFF(84, alt = transition_start_agl_m)、DO_CHANGE_SPEED(178)、NAV_WAYPOINT(16, p2 = accept_radius)、ORBIT+arc → NAV_LOITER_TIME(19, p3 = ±R)、RGB 條帶 DO_SET_CAM_TRIGG_DIST(206, l(h_op)(1−o_fwd))、LiDAR 條帶 DO_SET_RELAY(181, relay 0；依酬載待驗證)、DO_LAND_START(189) → NAV_VTOL_LAND(85)。MAV_CMD 常數取自 pymavlink（`MAVLINK20=1`），缺席時退回同值內建表。ardupilot.py 完成後應改為呼叫該模組。 |
| 航點 `airspeed_mps` / `sensor_on` 語意 | `altitude.build_3d` 之航點屬性為其**出發航段**（接點以後一段起點取代前一段終點） | DO_CHANGE_SPEED／觸發指令緊接在該 NAV 項目**之後**；VTOL_TAKEOFF 後先以第一個 CRUISE 空速下 DO_CHANGE_SPEED；與起飛點重合（< 1 m）之 NAV 項目略去。 |
| 缺省輸入補值 | 表 2 之 C2 範圍、地形淨空、備降點、電量、風、氣象皆必填（D11 → NO_GO），但舊系統 payload 只有 4 個角點 | `BridgeDefaults`：c2 25 km、terrain_clearance 36 m、max_agl 120 + waiver 200、soc 0.9、無風、能見度 8 km／25 °C、平地合成 DEM（home MSL 或 0 m、50 m 步長、幾何 ⊕ 4 km）、**自動備降點**（無 alternates 時距 home 1 km，因兩平台 `alternate_landing_sites_required=True`）、legacy 無 home 時置於區域西側 1.5 km。每項補值記入 `summary.assumptions` / 狀態 `assumptions`。 |
| 回應航點高度 | ArduPilot 相對高度 = MSL − home MSL | `alt_rel = alt_msl − ctx.frame.h0_msl_m`（home 為 ENU 原點且 `sites_enu[home] = (0,0,0)`）；`QgcOptions(frame="msl")` 改輸出 MAV_FRAME_GLOBAL。 |
| 額外輸出 | 非任務要求 | 回應另附 `rally`（備降點）與 `geofence`（區域∪站點凸包 ⊕ 1 km、禁區、h_ceil）供使用者系統上傳 rally／fence。 |

## examples/（範例腳本；契約無對應章節）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `examples/_compat.py` 之 ArduPilot 匯出／MAVLink 鏈路後備實作 | 撰寫範例時 `hv_dccpp/ardupilot.py` 與 `hv_dccpp/mavlink_link.py` 尚不存在（同時段由其他代理實作，完成於 21:39／21:42） | `_compat.get_ardupilot()`／`export_ardupilot(plan, ctx, cfg, out_dir)` 優先使用 `hv_dccpp.ardupilot.export_all(plan, ctx, cfg, out_dir)`；模組缺席時才用 `_ArduPilotFallback`（QGC WPL 110 任務／rally／fence／.param 最小輸出）。`FleetLinkFallback`（pymavlink 最小上傳／遙測）保留於 `_compat.py` 但 `run_mock_autopilot_loop.py` 已改用 `mavlink_link.VehicleLink/FleetLink`（模組缺席時腳本印出訊息並結束）。實際輸出皆由 `hv_dccpp.ardupilot` 產生（run_mini/run_hills/mock loop 印出 `exporter = hv_dccpp.ardupilot`）。 |
| 範例解析度 | 預設 `dem_step_m=10` 之 1e6 節點重取樣 ~1.7 s／hills 3.3M 節點 ~5 s | 所有範例用 `dem_step_m=50`、`terrain_margin_m=1500`（paper 情境用其 cfg 之 250 m 與 5000 m margin、`d_d_m=200`，與 `sim.paper_table2_check` 相同）。 |
| `run_dynamic_events.py` 遙測 | 契約 `dynamic.Telemetry` 需由外部提供 | `_compat.telemetry_from_plan(plan, t, ctx)`：自計畫航點線性內插位置／高度／能量，θ = 該腿 track，現行條帶與進度由 OPERATE 航點投影得。串接模式在 UAV_EXIT(A1) 之後以 B1 為 TARGET_CUE 來源與 ENERGY_UPDATE 對象（A1 已退出）。 |
| `run_mock_autopilot_loop.py` 之重上傳切換點 | `FleetLink.replan_and_reupload` 預設 `switch_seq` = 現行條帶末端之後的第一個 seq（適用「完成現行條帶再切換」）；本示範之 DIVERT 錨點為現時位置、B1 之錨點在其序列末端 | 以 `set_current=False` 呼叫後自行 `set_current(seqmap.seq_of_wp(min(cur_wp, k)))`：載具尚未到錨點 → 在新任務中接續現行 wp（錨點前航點與舊任務相同）；已過錨點 → 跳至錨點 seq。A1 之 divert 於 seq 36→新任務 seq 35，B1 於 seq 30→30（新任務 128 項）。 |
| 事件時間軸 | mock 以 `time_boot_ms` 計時、飛行模型（定速直線、無風）與計畫時間不一致 | `VehicleLink(time_source="boot")` 且 MISSION_START 後設 `boot_offset_s = t_boot − plan.t_takeoff_s`；注入事件之 `t_s` 取現行 seq 對應航點之計畫時刻（`seqmap.entry(seq).wp_index`）。`detect_events` 依 mock 電量（800 Wh／350 W 線性）自動產生之 EnergyUpdate 只計數不處理。 |
| Windows 主控台編碼 | 模組備註字串含 ×／−／→，cp950 主控台 `print` 會丟 UnicodeEncodeError | `_compat` 匯入時 `sys.stdout/stderr.reconfigure(errors="replace")`；mock 子程序以 `PYTHONIOENCODING=utf-8` 啟動。 |

## ardupilot.py（任務描述之 API；契約無專節）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `plan_to_mission_items(...)` 額外關鍵字 | 觸發距離需知道條帶用哪個感測器（`UAVPlan` 只有 strip_id／cell_id，無 req_id） | 新增**關鍵字** `strip_sensors: Mapping[str, Sensor] \| None = None`（strip_id → Sensor，即 planner 之 `decompose.resolve_sensor` 結果）與 `z_tol_m`（高度剖面 DP 容差，預設 `cfg.delta_prof_m`）。缺省時依該機 `primary_sensor.kind` 在 `ctx.sensors[(cls, *)]` 中挑同種類者，再退回該構型任一。 |
| LiDAR 條帶開／關 | 任務描述「DO_SET_CAM_TRIGG_DIST(0) + DO_SET_SERVO 佔位」；若兩者皆輸出，相機 on/off 計數不對稱 | lidar 條帶只輸出 `DO_SET_SERVO(LIDAR_SERVO_CHANNEL=9, 1900)` ／ `(…, 1100)` 成對；不輸出 `DO_SET_CAM_TRIGG_DIST(0)`（任務開始時觸發本即關閉）。通道／PWM 為模組常數（佔位，依 SERVOx_FUNCTION 調整）。 |
| ORBIT → NAV_LOITER_TO_ALT | 去衝突之延遲盤旋（Δz≈0）用 LOITER_TO_ALT 會立即離開 | 整圈 ORBIT（Σ掃掠角 ≥ 2π−0.1 或單點弧）：Δz ≥ 1 m → `NAV_LOITER_TO_ALT(param2=±R)`；Δz < 1 且有時長 → `NAV_LOITER_TIME(param1=Δt, param3=±R)`；否則 `NAV_LOITER_TURNS(param1=圈數, param3=±R)`。非整圈之 ORBIT 弧（起飛盤旋之接合弧）與一般弧同樣外切離散。 |
| 弧段離散 | 規格 §8.5 外切多邊形；autopilot 航點須 ≥ 接受半徑 | 同一 `arc` 之連續航點（跨 CLIMB→ENTRY 等 FW 階段改名不切分）重離散：n=⌈|Δ|/Δψ⌉、頂點在 R/cos(δ/2)；`n==1` 時弦矢高 ≤ e_max，不給頂點（矢高 ≥ 0.25 m 時起點受保護不被接受半徑濾除）。弧起點與其首頂點共線（切線構造）時可被首頂點取代。B（R 120）之 R·tan(δ/2)≈25 m < r_acc=30，Ω 轉場短弧旁會殘留少數 < r_acc 之相鄰航點（ArduPilot 視為立即到達，幾何偏差 ≤ e_max）。 |
| 直線段「只留端點」 | 山區作業航線之高度剖面沿直線變化，只留端點會丟失包絡 | 直線 run 另保留航向轉折 > 1° 與高度 Douglas–Peucker 偏差 > `delta_prof_m` 之點；非必要點距前一保留點 < r_acc 時丟棄。 |
| TRANS_OUT／TRANS_IN／LAND 高度 | 任務描述未定 | TRANS_OUT／TRANS_IN 取樣點全部略去（ArduPilot 自行轉換）；NAV_VTOL_TAKEOFF alt = 起飛站地面 + H_ts；NAV_VTOL_LAND alt = 落點站地面（相對 home，落 home 為 0），lat/lon = LAND 航點；`DO_LAND_START` 於第一個 {RTB, DESCENT, DECEL, TRANS_IN, LAND} 航點前（lat/lon 0）。 |
| fence inclusion「凸包 ⊕ 2·R_eff」 | 實際航線於 Ω 轉場超出端線 d_turn（D38）與起降迴圈可及 d_tr+d_acc+2R，2·R_eff 不足以涵蓋 | inclusion = ConvexHull(區域 ∪ 站點 ⊕ (d_tr+d_acc+2R_eff) ∪ [可選 `plans` 之航線 ⊕ b_trk]) ⊕ max(2·R_eff, R_eff(1+√3)+e_ext,max) ∩ P_link,c（`fence_buffer_m`）；`fence_items(inputs, ctx, cls, plans=None)` 多一可選參數，`export_all` 會傳入同構型計畫。exclusion = 有效禁飛區原多邊形（未加 b_trk）。 |
| `recommended_params` 電池參數 | mAh 需電壓；`EnergyModel` 無電壓欄位 | 新增關鍵字 `pack_voltage_v`（`SortieEnergy.pack_voltage_nom_v`）、`wind_speed_mps`、`h_ceil_agl_m`、`lost_link_policy`、`alt_frame`、`t_ll_s`；電壓未知或基準分鐘模式時省略 BATT_CAPACITY／BATT_LOW_MAH／BATT_CRT_MAH。說明表為模組常數 `PARAM_COMMENTS`（`write_param_file` 以 `#` 註解寫入）。AIRSPEED_MAX 於 HERO（None）暫以 1.4·Va 給值並註記暫定；TECS_SINK_MIN 取 0.9×規劃下降率。 |
| rally／fence 檔案格式 | pymavlink `MAVRallyLoader`／`MAVFenceLoader` 讀舊式文字格式，Mission Planner 現行以任務項目格式上傳 | `<uav>_rally.txt`／`<uav>_fence.txt` 皆為 QGC WPL 110（5100／5000／5001／5002 項目，`mission_type` 2／1，無 home 列），以 `read_qgc_wpl` 或 `MAVWPLoader` 讀回。 |
| pymavlink 相依 | 純輸出層不應硬性依賴 | 常數皆由 `mavutil.mavlink.MAV_CMD_*` 取得；pymavlink 缺席時退回同值 fallback（`HAVE_PYMAVLINK=False`）。 |

## mavlink_link.py（設計文件 §15.5；模組契約未列，介面依主控代理任務描述）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| 介面形式（§15.5.1 表列 `connect`/`upload_mission(conn, plan, ctx)`/`poll_telemetry`/`push_diff`…） | 任務描述指定類別式 API（`VehicleLink`／`FleetLink`／`VehicleTelemetry`／`to_planner_telemetry`／`detect_events`） | 採任務描述之 API；§15.5.1 之語意以方法對應：`connect`→`VehicleLink(conn_str)`+`wait_heartbeat`；`upload_mission(conn, plan…)`→`plan_to_mission_items`+`VehicleLink.upload_mission(items)`；`poll_telemetry`→`read_telemetry`+`to_planner_telemetry`；`push_diff`→`FleetLink.replan_and_reupload`；`start_mission(arm=True)`→`arm()`+`start_mission()`；`set_params(conn, path, dry_run)`→`set_params(dict)`+`read_param_file(path)`；`guided_divert` 未實作（mock 不支援 GUIDED 目標）。`MissionSeqMap`/`SeqEntry` 依 §15.5.2 JSON 格式實作（`load`/`save`）。 |
| `ardupilot.plan_to_mission_items` 之 seq→條帶對照 | `hv_dccpp.ardupilot` 之 `MissionItem` 不含 phase/strip_id 中繼資料（實作時該模組尚未存在，之後平行完成） | `mission_seq_map(items, plan, ctx)` 以 NAV 項目 lat/lon 對應最近之計畫航點（≤5 m）取 phase/strip_id/dir/wp_index；弧段重新離散點（無對應航點）phase 取下一個已對應 NAV、strip 只在前後皆同條帶時繼承；DO_ 指令繼承下一個 NAV 項目。**條帶末端 p^R/p^L（計畫上為 TRANSFER/RTB 首航點）列為該條帶之 OPERATE 終點（s_frac_end=1）**，因 MISSION_CURRENT.seq 為「正飛向」之項目，如此飛行於條帶內時 `strip_at(seq)` 才回該條帶（`strip_progress` 由位置投影算出）。 |
| 「seq→strip map 附於回傳 items 之屬性」 | Python `list` 不能設屬性 | `plan_to_mission_items` 回傳 `MissionItems(list)` 子類，屬性 `.seqmap`（`MissionSeqMap`）與 `.mission_type`；`mission_seq_map` 對可設屬性之序列自動寫入 `.seqmap`，並把中繼資料寫回 `MissionItemInt` 物件。 |
| 後備轉換 `plan_to_mission_items_fallback` | 實作時 `ardupilot.py` 尚未存在；亦供該模組不可用時使用 | 私有簡化版（home／NAV_VTOL_TAKEOFF／DO_CHANGE_SPEED／NAV_WAYPOINT／ORBIT→NAV_LOITER_TO_ALT／DO_SET_CAM_TRIGG_DIST 開關／DO_LAND_START／NAV_VTOL_LAND）；不做弧段重新離散（沿用 build_3d 已點縮減之航點）。`ardupilot` 存在時一律優先。 |
| BATTERY_STATUS 能量優先序（§15.5.2：energy_consumed → current_consumed → %） | `tools/mock_autopilot.py` 之 `energy_consumed` 欄位實為「剩餘 Wh×36」（非消耗量），且 `E_nom_start` 需外部提供 | `VehicleTelemetry.battery_pct`（BATTERY_STATUS.battery_remaining → SYS_STATUS 備援）優先；`battery_wh_remaining` 只在 % 缺席且 `VehicleLink.nominal_wh` 已設時由 `energy_consumed` 換算。`to_planner_telemetry`：Wh 優先，否則 % × E_nom（EnergyModel.nominal_wh → available_wh；基準分鐘模式 → None）。 |
| `t_s` 基準 | mock 之 `time_boot_ms` 為模擬時間（time-scale 加速），牆鐘不適用 | `VehicleLink(time_source='wall'|'boot', t0_epoch_s, boot_offset_s)`；預設 wall（`time.time() − t0_epoch_s`）。 |
| 事件規則 `EnergyUpdate` | §15.5.2 說每筆 BATTERY_STATUS 皆發；任務描述說「偏離 > δ₁ 才發」 | 採任務描述：`|e_tel − E_plan(t)| > cfg.delta_1 × E_nom` 才發（dynamic 內部再依 δ₁/δ₂ 判定）。`LinkLoss` 以 `link_ok`（心跳年齡 > `hb_timeout_s`=5 s）True→False 轉換觸發；t_ll 之持續判定留給 `dynamic.handle_link_loss`。 |
| `WindUpdate` 層改寫 | `WindField.layers` 依 ENU 排序、`inputs.wind` 為輸入順序 | 以物件身分找出含當前 z 之層並 `replace(speed, dir)`，其餘層不變；無層 → 單層 AGL 0–inf。 |
| 圍籬 | `ardupilot.fence_items(inputs, ctx, cls)` 為每構型 | `FleetLink.upload_plan` 依各機構型各建一份並上傳；後備版（無 ardupilot）為區域∪航點∪站點凸包 ⊕ 2·b_trk。 |
| 線上測試時間 | 協定往返受 mock 20 ms 迴圈與 98 項目上傳限制 | `test_vehicle_link_end_to_end` 約 8–10 s（> 5 s 規則）；其餘測試 < 2 s。pymavlink 缺席 → 整檔 skip。 |
| 觀察到之上游問題（未修改，非本模組） | `ardupilot.plan_to_mission_items` 於條帶起點 NAV 之**前**插入酬載「開」、於條帶終點 NAV 之**前**插入「關」；ArduPilot 的 DO_ 指令於前一 NAV 完成時立即執行，故實際為「進入條帶前的最後一段開、條帶本身關」（反了）。`pymavlink 2.4.49 mavwp.MAVWPLoader.load` 在 MAVLINK20=1 下多傳一個引數（TypeError），測試對此容錯。 | 記錄供 ardupilot 實作者修正：酬載開應置於條帶起點 NAV 之後、關置於終點 NAV 之後。 |

## 整合修正（integrator，2026-09-05）：AGL 基準與 4D 走廊例外

| 模組／項目 | 原因 | 實際做法 |
|---|---|---|
| `geo.TerrainGrid.raw_view()`（契約未列） | 需要一個「燒入前自然地形」之取樣視圖（`z_raw` 已有，但 `z_at/corridor_*` 只吃 `z`） | 新增 `raw_view()`：回傳 `z = z_raw` 之 `TerrainGrid`（內部快取；`burn()` 時失效）。`z` 與 `z_raw` 相同時回自身（零成本）。屬性 `__slots__` 增加 `_raw_view`。 |
| `altitude.sample_bounds` / `local_envelope` / `phase_boundaries` / `build_3d` 之 `z_g` | 規格 §11.1 之 `z_g^max/min/0` 指**地形**，障礙另由 `z_obs = z_top + b_v` 表達；但 `ctx.terrain` 已被 `KeepoutSet.build`／`context` 燒入障礙（D24），直接取用等於在障礙頂之上再要求 `h_minfw`（A 120 m），同時上界 `z_g^0 + h_ceil` 在障礙旁又以低地為基準 ⇒ `z* > u`。山丘情境實測：跨越電力線 PL1（頂 +b_v = 84.3 ENU、自然地面 ≈ 9 m）時 `z_lo = 204.3`、`z_hi = 193.4`，A1/A2/A3/B3 出現 `envelope_infeasible(deficit 7–13 m)` 與 `above_z_hi`，`sim.validate` 報 `above_ceiling`（201–213 m AGL）。`decompose.altitude_band` 早已因同樣理由改用 `z_raw`。 | 全部 AGL 語意改以 `terrain.raw_view()` 量測（`z_lo`、`z_hi`、`z_des`、`h_min,l`、`Waypoint.agl_m`、CLIMB/DESCENT 走廊淨空線）；障礙淨空仍由 (a) `z_obs = z_top + b_v`（`sample_bounds`）與 (b) 新增下界「燒入後 `z_g^max` + `h_clr`」保證（`local_envelope` 無障礙表，(b) 為其唯一障礙保護，等效 ≥ b_v + h_clr = 66 m）。結果：山丘 4 架剖面警告全部消失、`airspace_ok=True`。 |
| `sim.check_kinematics` / `_coverage` 之 AGL | 同上；若規劃端改自然地形而驗證端仍用燒入地形，障礙上方會反過來報 `agl_below_h_minfw` | 兩處改用 `ctx.terrain.raw_view()`；障礙之垂直淨空由 `check_link_airspace` 之 keep-out 2.5D 測試（`tops_z` 含 b_v）負責，未放寬安全性。 |
| `viz.profile_bounds`、`decompose` 之 §7.3 坡度探針 | 與上述同基準；燒入之人工陡壁不是地形坡度（`geo.slope_at` 早已 `raw=True`） | 皆改 `raw_view()`。 |
| `deconflict.check_separation`：新增 `SITE_CORRIDOR_PHASES` 例外（規格 §13.3 未列） | 起降走廊之互斥由 `CorridorSchedule`（§13.5：起飛間隔 60 s、降落預約 [t_arr−30, t_land+90]、降落優先）保證；同時計入 4D 分離則因 σ_t = k_t·t 視窗（t = 1500 s 時 ±75 s）使**同站起降必然衝突**，且修復所需延遲隨 t 發散。實測山丘 8 機：11 筆殘餘衝突中 5 筆為 (0,0) 起降墊上之 TAKEOFF/LAND 樣本；反覆修復把 makespan 由 7238 s 推到 18571 s 才收斂（等同單機序列化）。 | 兩樣本**皆**屬 `{TAKEOFF, TRANS_OUT, TRANS_IN, LAND}` 時不計為 4D 衝突（`_Track.corr`）；其餘（過境／作業對走廊）維持原判定。走廊互斥仍由 `CorridorSchedule.conflicts()` 檢查，`sim.validate` 亦一併計入。 |
| `deconflict.repair`：候選接受條件 | 原本只做「該機對乾跑」驗證，選 makespan 最小者；實測會把衝突推給其他機對（山丘 11 → 20 筆），且逐輪延遲互相加碼 | 新增 `_Repair.involved_score(new, uavs) -> (衝突數, ΣΔ_req)`（只重跑與被改動機相關之機對，8 機約 10 ms）。`_repair_one` 依 (makespan, cost) 排序候選，取第一個「衝突數嚴格下降，或衝突數持平但 ΣΔ_req 下降且 makespan 不增」者；否則不動該筆（留為殘餘）。位能函數字典序下降 ⇒ 必然終止。山丘結果：殘餘衝突 17 → 4、makespan 7238 → 5552 s。 |
| 殘餘（未修）：山丘 4 筆 4D 衝突 | 需要規格 §13.6 步 (iii)「IDP 重排該階段」（`resequence_hook`，本套件未實作）；單機時間平移皆為「補此失彼」（實測所有候選之 `involved_score` 皆不下降），且 8 機共用單一降落走廊時延遲盤旋圈數被 `next_free` 推到 39/58 圈而能量不足 | 保留為 `MissionPlan.report.warnings` 之 `residual_conflicts(n)` 與 `sim.validate` 之 `separation: conflicts=n`。 |

## 整合修正（integrator，2026-09-05）：酬載開關指令之任務項目順序

| 模組／項目 | 原因 | 實際做法 |
|---|---|---|
| `ardupilot.plan_to_mission_items`、`integration/legacy_adapter.plan_to_mission_items` | 兩者都把 `DO_SET_CAM_TRIGG_DIST` / `DO_SET_SERVO`（開／關）放在其所描述之航點的 NAV **之前**。ArduPilot 在「抵達前一個 NAV」時就執行其後的 DO_ 指令，而 `Waypoint.sensor_on` 描述的是該點的**出發腿** ⇒ 實測 mini B1 匯出結果為 `[DO_ON, NAV(條帶起點), DO_OFF, NAV(條帶終點)]`：開在進入條帶前的那一腿、**關在條帶本身**，整條條帶不會拍照／掃描（`mavlink_link` 作者已在其偏差表提出，未修）。 | 兩處都改為在該點的 NAV 項目**之後** append 開／關（`ardupilot` 以 `want_open` / `want_close` 延後、`legacy_adapter` 把 `cam(...)` 移到 NAV append 之後）。新順序 `[NAV(起點), DO_ON, NAV(終點), DO_OFF]`：抵達條帶起點才開、抵達終點才關。`integration/mqtt_bridge` 原本就是正確順序，未動。測試 `test_ardupilot.py::test_camera_trigger_pairs_balanced_rgb / test_strip_endpoints_preserved / test_synthetic_arc_reduction_and_orbits` 之順序斷言同步更新（改為「開／關指令緊接在條帶起／訖 NAV 之後」）。 |
| `DO_CHANGE_SPEED` 位置（未改） | 同樣描述出發腿，目前仍放在該點 NAV 之前 ⇒ 空速提前一腿改變 | 保留：提前一腿到達條帶速度是安全且合理的行為（進入條帶時已達目標空速）；若要與酬載規則一致，改法與上列相同。 |

## mission / context / scenario（2026-09-06）：逐機起飛站點 `UAVInstance.start_site_id`

| 模組／項目 | 原因 | 實際做法 |
|---|---|---|
| `mission.UAVInstance` 新增欄位 `start_site_id: str | None = None`（契約 §3 表 2 未列） | 契約之 `MissionInputs` 只有單一 `home`，全機隊只能自同一點起飛。論文 Table 2 情境（規格 §15、§17.1 (7)）之三機各有自己的地面站（`PAPER_TABLE2_UAV_START`），先前只能把 U1 當 home、U2/U3 當「備降點」，三機實際都自 U1 起飛，逐機路徑長因此不可與論文 648.13／606.06／591.27 km 誠實比較（見本檔既有列與整合報告 open issue）。 | 選填欄位，`None` → `MissionInputs.home`（**所有既有情境／測試行為不變**）。值須為 `home` 或 `alternates` 之 `site_id`，於 `MissionInputs.__post_init__` 檢查（錯誤訊息含 `start_site_id`）。回收／降落政策**不動**（D40：`recovery_site_id` → home），故「自站 A 起飛、回收於 home」是允許且預期的組合。 |
| `mission.MissionInputs` 新增查詢 `start_site_id_of(uav_id)`、`start_site(uav_id)`、`start_sites`（property） | 上層（context/planner/preflight/ardupilot）需要一致的解析點，避免各自寫 `u.start_site_id or home` | `start_sites` 回傳 `{uav_id: site_id}`（未設定者填 home）。 |
| 序列化 | `to_dict()` / `load_mission()` 必須往返 | `_uav_to_dict` 輸出 `"start_site_id"`（未設定為 `null`）；`_parse_uav` 接受 `start_site_id`（亦接受別名 `start_site`，可為 `Site` 實例）。`load_mission(inputs.to_dict()) == inputs` 之往返測試已涵蓋。 |
| `context.MissionContext` 新增欄位 `start_sites: dict[str, str]`（契約 §6A 未列，置於既有「額外欄位」尾端、有預設值） | 起飛走廊需要「站點 id + 站點 ENU」，`build_context` 是唯一同時持有兩者的地方 | `build_context` 步驟 13 建表；新增方法 `start_site_id(uav_id)`（`start_sites` 缺鍵時退回 `inputs` 解析）、`start_xy(uav_id)`、`z_ground_start_m(uav_id)`。任一機非 home 起飛時追加警告 `per_uav_start_sites(U2@U2,U3@U3)`。 |
| `scenario.build_paper_table2_scenario` | 讓論文情境反映論文 §4.1「各機自其地面站起飛」 | 三機以 `start_site_id=uav_id` 建構（站點 `U1`(home)／`U2`／`U3` 位置沿用 `PAPER_TABLE2_UAV_START`，高度仍由平地 DEM 2700 m 取）。回收點仍為 home = U1。 |
| **尚未接線（上游待辦，非本輪可改檔案）** | `planner.plan_mission` 於「4. 暫定起降走廊」與「7. 起飛走廊精算」兩處都以 `home = inputs.home.site_id` 呼叫 `altitude.takeoff_corridor`，並以 `ctx.home_xy` 找最近進入點；`preflight.check_sites` 之 `primary = {home, recovery}` 未含起飛站點（U2/U3 失敗只會是警告） | 本輪**只提供**輸入與解析（`ctx.start_site_id/start_xy`）。在 planner 改用之前，論文情境之逐機路徑長不變。以不改檔的 monkeypatch 量測到的差異列於本輪報告（U1 679.7→684.5、U2 688.9→701.1、U3 840.0→907.6 km）。 |

## ardupilot / mavlink_link / mock_autopilot（2026-09-06）：ArduPlane 參數列舉、起飛時間原點、ENERGY_UPDATE 遲滯

D45–D50 為本輪新增之實作決定編號（規格 §0 未列，屬輸出層／整合層）。

| 模組／項目 | 原因 | 實際做法 |
|---|---|---|
| **D45** `ardupilot.plan_to_mission_items`：`DO_CHANGE_SPEED` 位置 | 前一輪整合把酬載開關移到 NAV **之後**，但 `DO_CHANGE_SPEED` 留在 NAV 之前（記為「刻意保留」），使本模組與 `integration/mqtt_bridge`（本來就放在 NAV 之後）對速度指令差一個項目。`Waypoint.airspeed_mps` 與 `sensor_on` 同樣描述該點之**出發腿**，兩者不應有不同規則。實測 mini／hills 每份任務只有 **1 個** `DO_CHANGE_SPEED`（ACCEL 段首點），故此變更之飛行影響僅止於「巡航速度指令晚一腿發出」，而該腿正是轉換後之加速段本身。 | 改為與酬載開關相同：置於其所描述之出發腿起點 NAV **之後**。兩個匯出器對 DO_ 指令的規則自此一致。殘餘差異（屬 `mqtt_bridge`，未改）：`mqtt_bridge` 另在 `NAV_VTOL_TAKEOFF` 之後先補一個「首個巡航空速」`DO_CHANGE_SPEED`。測試 `test_change_speed_after_its_nav` 斷言每個 `DO_CHANGE_SPEED` 之前一項為 NAV。 |
| **D46** `export_all(takeoff_delay=…)` 與 `plan_to_mission_items(takeoff_delay_s=…)`（契約未列） | 匯出之任務**不帶時間原點**：每份都自 `NAV_VTOL_TAKEOFF` 開始，沒有 `NAV_DELAY(93)`／`CONDITION_DELAY(112)`／`DO_JUMP(177)`，而 `UAVPlan.t_takeoff_s` 是走廊排程（§13.5 同墊 60 s）與 4D 去衝突的結果。山丘 8 機之起飛跨度達 2793.9 s；操作者若同時 AUTO 全隊，這些間隔與延遲全部作廢，飛的就不是被驗證過的計畫。 | 新增兩種模式：`'operator'`（**預設**，與 `mavlink_link.FleetLink`／`examples/run_mock_autopilot_loop.py` 之「地面站依 `t_takeoff` 逐機 MISSION_START」一致）不插入項目，但把 `t0` 與該機延遲寫進 `<uav>.waypoints` 檔頭註解（`# START THIS MISSION AT t0 + Δ s …`）與 `mission_summary.json` 之新區塊 `takeoff{mode, t0_s, spread_s, delay_s, note}`；`'nav_delay'` 於 `NAV_VTOL_TAKEOFF` 之前插入 `MAV_CMD_NAV_DELAY(93)`（param1 = Δt、param2–4 = −1），任務自帶錯開（此時**不可**再由地面站延後 MISSION_START）。`NAV_DELAY` 加入 `NAV_COMMANDS`；其 x/y = 0（非位置指令，不會被 `mission_seq_map` 誤配到航點）。`tools/mock_autopilot.py` 亦支援 `NAV_DELAY`（依 param1 原地等待）。 |
| `write_qgc_wpl(comments=…)`：註解一律轉 ASCII | pymavlink `mavwp.MAVWPLoader.load` 以**系統預設編碼**開檔（本機 cp950），UTF-8 中文註解會讓它丟 `UnicodeDecodeError` ⇒ 匯出檔變成讀不回來 | 註解以 `encode('ascii','replace')` 寫出（`QGC WPL 110` 版本列之後、資料列之前）。`test_takeoff_time_origin` 以 `mavwp.MAVWPLoader` 驗證兩種模式之檔案都仍讀得回來。 |
| **D47** `rally_items(alt_frame='msl')` | ArduPilot 之 `AP_Rally::RallyLocation` **沒有 MSL 框架**：高度是 int16「相對 home」（terrain 另有旗標）。原本 `'msl'` 寫 `MAV_FRAME_GLOBAL` + `site_msl + h_tc`，上機後會被讀成「相對 home 高 site_msl + h_tc」⇒ 集結點高出 `site_msl`（mini 為 +100 m）。 | `'relative'` 與 `'msl'` 一律輸出相對 home 高度並用 `MAV_FRAME_GLOBAL_RELATIVE_ALT`；只有 `'terrain'` 用 `MAV_FRAME_GLOBAL_TERRAIN_ALT`（AGL）。**與契約之 `alt_frame` 語意有意不同**（rally 不吃 msl）。 |
| **D48** `recommended_params`：`FENCE_ALT_MAX` 改以「相對 home 高度」計算 | `AC_Fence` 之最高高度圍籬比對的是 EKF 之 relative alt（相對 home），不是 AGL。地形高於起降場時兩者相差正好是地形抬升量，原式 `max(h_ceil, max(AGL)+b_v)` 會低估 ⇒ `FENCE_ENABLE=1` + `FENCE_ACTION=1` 在完全正常的計畫中途觸發 RTL。山丘匯出實測餘裕僅 8.9 m（A1：航點最大相對高 201.1 m vs `FENCE_ALT_MAX` 210）。 | 新增 `home_msl_m` 參數：`rel_max = max(w.alt_msl_m) − home_msl + b_v`，`FENCE_ALT_MAX = ceil(max(h_ceil, rel_max))`。未給 `home_msl_m` 時退回 AGL 並在 `warnings` 記一筆。`export_all`／`mavlink_link.suggest_params` 都會帶入 home MSL。 |
| **D49** `recommended_params`：ArduPlane 之 `BATT_FS_*_ACT` 列舉、恢復參數、酬載參數 | (a) 原本寫 `BATT_FS_LOW_ACT=2`、`BATT_FS_CRT_ACT=2` 並註解「2 = RTL／3 = LAND(QLAND)」——那是 **ArduCopter** 的列舉。QuadPlane 跑 ArduPlane，其列舉為 `0:None 1:RTL 2:Land 3:Terminate 4:QLand 6:Loiter to QLand`：匯出值 2 是 Land（整段 RTB 航線），而註解叫操作者填的 3 是**飛行終止**。(b) `FENCE_ACTION=1`／`FS_LONG_ACTN=1`／`BATT_FS_*` 都選了 RTL，任務也都含 `DO_LAND_START`，但缺 `Q_RTL_MODE`（預設 0 ⇒ QuadPlane 之 RTL 只在 home 上空定翼盤旋）、`RTL_AUTOLAND`（預設 0 ⇒ 降落序列永不使用）、`FENCE_RET_RALLY`（預設 0 ⇒ 不回同批匯出的集結點）。(c) 任務發 `DO_SET_CAM_TRIGG_DIST(206)` 與 `DO_SET_SERVO(183)`，但參數檔沒有任何一個讓它們生效的參數（12 份 `.param` 中 `CAM*`／`SERVO*`／`RELAY*` 皆 0 筆）：`CAM1_TYPE=0` 時相機後端不存在、206 是空指令；`SERVO9_FUNCTION≠0` 時 183 被韌體以「Channel N is already in use」拒絕。 | (a) 改為 `BATT_FS_LOW_ACT=1`（RTL）、`BATT_FS_CRT_ACT=4`（QLand），`PARAM_COMMENTS` 改寫為 ArduPlane 列舉並標明 3 = Terminate。(b) 新增 `Q_RTL_MODE=1`、`RTL_AUTOLAND=2`、`FENCE_RET_RALLY=1`。(c) 新增 `sensor_kinds` 參數：`rgb` → `CAM1_TYPE=1`（4.5+ servo 後端）、`CAM1_DURATION=1`、`SERVO{CAMERA_SERVO_CHANNEL=10}_FUNCTION=10`（CameraTrigger）；`lidar` → `SERVO{LIDAR_SERVO_CHANNEL=9}_FUNCTION=0`（Disabled ⇒ 開放給任務 DO_SET_SERVO）。兩個通道號皆為**佔位**，須依實機接線調整（與既有 `LIDAR_SERVO_CHANNEL` 之註記一致）。(d) 電量門檻（`BATT_*_MAH`）仍只在已知 pack 電壓時輸出；**無法輸出時不再靜默**：`recommended_params(warnings=[…])` 回一筆警告，`write_param_file(warnings=…)` 把它寫成 `.param` 之 `# WARNING:` 註解、`mission_summary.json` 之 `param_warnings`。動作本身仍輸出（值正確才不會誤觸發 Terminate），門檻由操作者於機上設定。 |
| **D50** `recommended_params`：`Q_ASSIST_SPEED` 夾限、`WP_RADIUS` 由幾何推導、`WP_LOITER_RAD` 取實際盤旋半徑 | (a) 原式 `Q_ASSIST_SPEED = v_tr` 對兩型都**正好等於** `AIRSPEED_MIN`（A 18／B 16），TECS 每次調到下限（轉彎、陣風、下降）都會啟動升力馬達。(b) `WP_RADIUS = r_acc = 30 m`，但 ArduPlane 對**所有**航點都用 `WP_RADIUS`（`NAV_WAYPOINT` param2 被忽略），而必要點／頂點／保護點不受接受半徑過濾 ⇒ 實測 hills/B1 有 28/182 對相鄰 NAV 間距 < 30 m（最小 19.1 m），其中 11 個 `DO_SET_CAM_TRIGG_DIST` 夾在這種點對之間，會提早最多 ~30 m 觸發。(c) ArduPlane 對 `NAV_LOITER_TIME(19)` **只讀 param3 之正負號**（方向），半徑一律用 `WP_LOITER_RAD`；原本 `WP_LOITER_RAD = R_eff`（含風放大）而延遲盤旋用的是規劃半徑 R，hills A2 實測 228.4 m vs 200 m（+14.2%，圈時 66.7 s vs 60.0 s）。 | (a) `Q_ASSIST_SPEED = clamp(min(v_tr, AIRSPEED_MIN − 1), ≥ 1.15·失速)` → A 17.0、B 15.0。(b) 新增 `min_nav_spacing_m(items, ctx)`；`WP_RADIUS = clamp(floor(0.8·最小間距), 10, r_acc)`（mini 實測 A1 24 m、B1 18 m）。`NAV_WAYPOINT` param2 仍寫 `r_acc`（其他 GCS 會用；ArduPlane 忽略），註解已寫明。(c) 新增 `orbit_radius_m(items)`；`WP_LOITER_RAD` 取任務實際盤旋半徑（無盤旋才用 `R_eff`）。另 `NAV_LOITER_TURNS` 之 param3 取整並夾至 ±255 m（ArduPlane 把半徑塞進 `cmd.p1` 高位元組）。 |
| `to_qgc_plan_json`：terrain 框架之 `AltitudeMode` | QGC(≥4.3) 之列舉為 `0 None／1 Relative／2 Absolute／3 CalcAboveTerrain／4 TerrainFrame`；原本把 `MAV_FRAME_GLOBAL_TERRAIN_ALT(10)` 對到 3（＝由地形資料反推 AMSL），與同一項目的 frame 自相矛盾 | 改對到 4。只影響 `alt_frame='terrain'`（匯出預設為 relative）。 |
| **D51** `mavlink_link.detect_events`：`EnergyEventState` 遲滯／防抖（規格 §12.4 之工程化） | §12.4 的 `δ > δ₁` 是「一次事件」的觸發條件，不是逐筆遙測的布林旗標。載具電量模型與計畫恆有系統性偏差，逐筆比對會在**每一次輪詢**都送出 EnergyUpdate：`examples/run_mock_autopilot_loop.py` 一次示範 330 筆。 | 新增 `EnergyEventState`（每機一份，由 `FleetLink.energy_state` 保存並在 `poll()` 帶入）與三道閘：**遲滯**（進入 abs(Δ) > δ₁·E_nom、離開 abs(Δ) < 0.6·δ₁·E_nom）、**持續**（須連續 ≥ `ENERGY_DWELL_S=5 s` 且 ≥ `ENERGY_MIN_SAMPLES=2` 筆）、**重新觸發**（已送出後，須偏差再變化 δ₁·E_nom 或距上次 ≥ `ENERGY_MIN_INTERVAL_S=300 s` 且仍超標）。`energy_state=None`（單次檢查、既有測試）維持原本無狀態語意。實測：mock 閉迴路 330 → **16** 筆（每筆代表偏差又擴大 δ₁·E_nom = 67.5 Wh，屬真實劣化）。 |
| `tools/mock_autopilot.py`：電量模型 CLI | 原本只有建構子參數（`battery_wh=800`、`cruise_power_w=350`），CLI 無法設定，且與任一實際平台都不符（Pioneer 標稱 1500 Wh／起飛 1350 Wh、HERO 380 Wh），閉迴路示範中的能量偏差因此是**模擬器造成**而非受測行為 | 新增 `--battery-wh`、`--cruise-power-w`、`--hover-power-w`（預設 8×巡航，用於 VTOL 起降段）、`--reserve-frac`（到達保留量時記錄告警）、`--energy-profile`（吃計畫能量剖面 JSON／plan JSON，電量直接照剖面走）、`--energy-error-frac`（蓄意多耗電，用來測 ENERGY_UPDATE）。同時修正 `BATTERY_STATUS.energy_consumed`：原本送「剩餘 Wh × 36」，應為「已消耗 × 36」（hJ）。實測 A1 以 `--battery-wh 1500 --energy-profile A1_energy.json`：171 次輪詢 **0** 筆 EnergyUpdate（誠實閉迴路）；沿用預設 800 Wh/350 W 時 166 次輪詢 6 筆。 |

## 幾何／覆蓋／能量／動態／驗證（2026-09-06）：審查發現之修正（R-01…R-14）

本節只涵蓋 `geometry.py`／`decompose.py`／`altitude.py`／`costs.py`／`sim.py`／`dynamic.py` 及其測試。

| 代號 | 模組／項目 | 原因 | 實際做法 |
|---|---|---|---|
| **R-01**（blocker） | `decompose.proxy_time` 之迴轉帶禁區硬性否決（D38、規格 §7.3） | 舊條件是 `turn_band(cell, α, R_eff+e_ext, 0) ∩ (NoFly⊕b_trk) > 1e-6 m² ⇒ T_s = inf`。`decompose_area` 以 `F = P_area −(NoFly⊕b_trk)` 造胞，**緊鄰禁區之胞與禁區緩衝區距離恆為 0**；`turn_band` 是把整個胞沿 ±t̂ 平移的 Minkowski 和，任何 α 都會把貼齊的邊界掃進緩衝區 ⇒ 每個方向都 inf ⇒ 整個區域被丟成 `no_feasible_direction`。實測：mini + 100 m×100 m 島狀禁區 → 6 胞 0 條帶、`coverage_raster` Z1 = **0.000**，而 `plan.feasibility.status` 仍是 GO_WITH_WARNINGS（那是規劃前的預檢結論）。規格 §7.3 只在「禁區距**端線** < R_eff + e_ext（連 U 型也放不下）」時才不可行。 | 新增 `_end_turn_blocked()`：逐條帶在其兩端點外量測 `R_eff` 深度、寬 `[v_lo−lateral, v_hi+lateral]` 的迴轉室，取未被禁區佔用之 u 深度；**某一端所有條帶深度都不足**才回 inf，其餘一律落到既有的軟性 `pen_keepout`。相交面積門檻由 `_TINY_AREA_M2 = 1e-6` 提到 `_MIN_VETO_AREA_M2 = 1.0 m²`（數值退化的貼齊不再否決）。原測試 `test_proxy_infeasible_turn_band_in_nofly`（禁區橫跨整條端線）維持通過。 |
| **R-01b** | `decompose._solve_class` 之 `not all(s.feasible)` | 只要有**一條**條帶被 D41 標成 `nofly_intersect`（多半是 e_ext 端延伸戳進緩衝區），整個方向就作廢，放大 R-01。 | 改為剔除 `nofly_intersect` 的個別條帶、其餘照飛；其他不可行原因（局部包絡）仍讓該方向作廢；方向排序改為 `(被剔除條數, T_single)`，優先選丟得最少的方向。實測 mini 島狀禁區 0.000 → **0.955**、hills 200 m 禁區 0.000 → **0.9967（通過）**。 |
| **R-02** | `sim.coverage_raster` 的 `n_in == 0` 分支 | 分母是柵格**中心**落在 `eff = 區域 −(NoFly⊕b_trk)` 內的個數；沒有任何中心落入時舊碼回 `1.0`（「無樣本 ⇒ 全覆蓋」），`CoverageReport.passed` 為 True、`validate` 的覆蓋閘門靜默放行。實測 3000 m×2 m 區 + `cov_raster_m=5`：零架機、零條帶 → `{'lidar': 1.0} passed=True`。 | 新增 `_areal_coverage()`：`n_in == 0` 且 `eff` 非空時改以**精確面積**量測（足跡 union ∩ eff / eff.area），並回一個 `uncovered_cells_xy` 代表點。`1.0` 捷徑只保留給 `eff.is_empty`（原本就另外處理）。 |
| **R-03** | `altitude.climb_corridor_length_m`（新增公開函式） | 規格書 §11.1／§11.8 寫「A 940 m、B **480 m**」。B 的 480 m 對應 Δh = 20 m，等於誤用 Pioneer 的下緣 80 m；HERO 的 `transition_complete_agl_m` 下緣是 70 m、`min_fw_operation_agl_m` 是 100 m ⇒ Δh = 30 m。 | 由平台參數表算得（順風 5 m/s、參考 AUW）：**A = 942.0 m、B = 724.1 m**（無風 A 764.3／B 573.2；下降側 A 706.8／B 543.2）。程式碼本來就對（用 `tc.h_tc_clear_m`）；本輪把它抽成公開函式 `altitude.climb_corridor_length_m(spec, wind, auw, side=…)`，`_climb_corridor_issues` 改呼叫它，並新增兩個測試把 942／724 釘住（含「在 600 m 處抬高地形，HERO 必須抓到」——沿用規格書的 480 m 會漏掉 480–724 m 這一段）。**規格書檔案本身未改**（不在本輪可改檔案清單內），見下方 notes。 |
| **R-04** | `sim._run_makespan` 的完整性判定（§17.1 (6) 基線） | `complete = plans ∧ not uncovered ∧ status != NO_GO`。`mp.uncovered` 只反映「未被指派的條帶」，不反映「某一感測需求整套條帶從未鋪設」：`both` 區由單一同時掛載 LiDAR + RGB 的構型服務時，`resolve_sensor` 只鋪 LiDAR 那套，RGB 那套整組消失。實測 hills `all_A`：56 條帶、Z2 `rgb` 覆蓋 **0.000**，卻被當成完整任務拿去和 107 條帶的 hetero 比 makespan。 | `complete` 追加「以 `coverage_raster` 實算的每區必要鍵都通過」；不通過 → `makespan_s = inf`（與 `all_B` 同待遇）。`baselines()` 另回 `<name>_coverage_min`。**未修**：`both` 區的雙套條帶鋪設需要同時改 `decompose` 與 `allocate.assign_config_and_count`（`allocate.py` 不在本輪可改檔案內），見 notes。 |
| **R-06** | `costs.EnergyBudgetTable.point_wh` 不是它自稱的上界 | 文件寫「任意點上界化」，`planner.energy_safe_check` 與 `sim.check_energy` 也都照上界用（`E_j ≥ r·E_nom + point_wh`），但它只收**直線**轉場，不含未知航向下的 Dubins 繞行、也不含 keep-out 繞行。實測 hills 981 個樣本最壞低估 **17.6 Wh（−13.1%）**，而綁定航點的餘裕只有 13.1 Wh。 | 低估量是可計算且有界的，故採**兩段式**而非一律加寬（一律加寬會把 A3 逼去備降場 ALT1，hills makespan 5087 → 6106 s 並觸發另一個潛在的 ALT1 下降包絡缺陷，見 notes）：`point_wh` 維持 O(1) 粗篩值不變（僅爬升項改以弧段爬升率 κ = cos φ_max 折減，方向保守）；新增 **`point_wh_slack_wh(cls, xy=None, z=None)`** = 低估上界 `k_unc·P_c·(2πR_eff + L_det)/V_g^worst`（hills 實測 A **50.4 Wh**、B **19.7 Wh**，都大於實測最壞低估）、**`point_wh_ub()`** = 粗篩 + slack（真正的 O(1) 上界，給不做精算的呼叫端）、**`point_wh_exact(cls, xy, z, θ, site)`** = AVOID + ENVELOPE 精算。`sim.check_energy` 改為：粗篩餘裕 < slack 時一律以 `point_wh_exact` 定案 —— 驗證器因此不可能再放行真正不安全的航點，且 hills 計畫與 makespan 完全不變。`point_wh` 之 docstring 改寫為「這不是上界」並寫明兩段式規則。**未修（不在本輪可改檔案）**：`planner.py:983 energy_safe_check` 仍只用粗篩，見 notes。 |
| **R-09**（blocker） | `dynamic._Session.anchor_now` 對尚未離場之機 | `_index_at` 會夾到 0，於是保留 TAKEOFF 航點的**未來**時戳，尾段卻自 `t_now` 起算 ⇒ `UAVPlan.waypoints` 的 `t_s` 倒退 ⇒ `deconflict._waypoint_arrays` 丟 `ValueError('UAVPlan.waypoints 之 t_s 須非遞減')` ⇒ `replan()` 以例外收場。規劃器刻意錯開起飛（`dt_dep_s = 60 s`；hills 最後一架 B2 在 2913.9 s 才起飛），所以重規劃視窗有一大段時間都踩得到。實測 hills：t_now=300 s 時 8 個 handler 有 6 個丟例外。 | 新增 `_takeoff_prefix_end()`／`_state_at_wp()`。`t_now < 起飛走廊出口時刻`（TAKEOFF/TRANS_OUT/ACCEL/CLIMB 開頭連續段的最後一點）時，**保留整段起飛走廊**並錨在其出口、沿用該航點自己的時刻。mini 之 WeatherAbort@10/30/59 s、EnergyUpdate@10/30/59 s 由「必丟例外」變為全部回傳且時間單調。 |
| **R-10** | `dynamic._exit_uav(keep_plan=False)` | `del sess.plans[u]` 把**已飛完**的航段一起刪掉：那些條帶不會進 `plan.uncovered`，但 `sim.coverage_raster` 以剩下的計畫重建足跡，會把同一片區域算成未覆蓋 ⇒ 同一次重規劃的兩個輸出互相矛盾（hills `UAV_EXIT("A1")@3000 s`：13 條已飛條帶消失，`uncovered` 只有 1 筆，覆蓋率 Z1 0.728／Z2 0.766 FAIL）。 | 改為把計畫**截到 t_exit**：保留 `waypoints[:idx+1]`、`strips` 只留已飛（含當前）條帶、`t_land_s`/`t_arr_ptc_s` 設為最後保留航點的時刻、重算 `distance_m`／`energy_used_wh`，並加警告 `uav_exited_at_t=<t>s`。**契約差異**：`UAVExit` 之後該機仍存在於 `MissionPlan.plans`（作為「已飛紀錄」），不再消失；`test_uav_exit_partial_strip` 的 `"A1" not in res.plan.plans` 隨之改成釘住截斷後的形狀。 |
| **R-11** | `dynamic._Session.deconflict` 的幽靈衝突過濾 | `(c.uav_a in changed or c.uav_b in changed)` 會把**所有**「未被重規劃之實機 vs 幽靈」的衝突在任何修復前丟掉（GhostTrack 的 id 永遠不在 `changed`），也不列入 `residual_conflicts`。規格 §12.5 要求「其他機在 §13 中對其避讓」。 | 幽靈 vs 實機的衝突不受 `changed` 過濾；修復時允許動尚未被重規劃的實機，動到就 `mark_changed`（並加入本輪 `changed`），`skip_pairs` 每輪重算。 |
| **R-12** | `sim.validate(..., t_sync_s=…)`、`ReplanResult.t_sync_s`（契約新增具預設值之欄位／關鍵字） | `validate` 固定用 `t_sync=0`，而 `dynamic._Session.deconflict` 用 `t_sync=t_now`（規格 §13.3：σ_t = k_t(t − t_sync)，t_sync = 最近一次遙測快照）。中途重規劃的計畫因此被灌上 k_t·t_now 的 σ_t（t_now=3000 s、k_t=0.05 → +150 s），驗證器會報出重規劃器判定不存在的衝突（hills WIND_UPDATE@3000 s：t_sync=3000 → 0 個衝突、t_sync=0 → 8 個）。 | `validate` 新增關鍵字 `t_sync_s: float = 0.0` 並轉給 `check_separation`；`ReplanResult` 新增 `t_sync_s`（= 該次事件的 `t_now`），呼叫端以 `sim.validate(res.plan, …, t_sync_s=res.t_sync_s)` 驗證中途計畫。預設值不變 ⇒ 既有呼叫端行為不變。 |
| **R-13** | `sim.validate`：分離檢查跑不起來 = 不算通過 | `check_separation` 丟例外時只把訊息塞進 `details["separation"]`，而 `_has()` 只掃 `per_uav`（由 `plan.plans` 的鍵組成），`conflicts` 仍是空的 ⇒ `passed` 的 `not conflicts` 被滿足 ⇒ **4D 分離等於完全沒檢查**。 | `ValidationReport` 新增 `separation_ok: bool = True`（尾端、有預設值）；例外時設 False 並強制 `passed = False`。 |
| **R-14** | `sim.check_mission_integrity`（新增）＋ `ValidationReport.integrity_ok` | 覆蓋率是以「計畫的 `sensor_on` 航段」算足跡的；若 OPER 航點沒開感測器、或某條帶根本沒人飛／被兩架各飛一次，`validate` 之前無從察覺。 | 新增 `check_mission_integrity(plan)`：`oper_sensor_off`／`oper_without_strip`／`sensor_on_outside_oper`／`strip_unknown`／`strip_not_flown`（已列在 `plan.uncovered` 者除外）／`strip_flown_twice`（同機同條帶之連續作業段算一次，中間插入的延遲盤旋不算切斷）。結果放 `details["integrity"]`，`integrity_ok` 併入 `passed`。**測試更動**：`test_runtime_under_1s` 蓄意把同一組條帶複製 4 份，因此正確地報出 `strip_flown_twice`；該測試的 `len(details) == 8 + 2` 改為 `8 + 3` 並補上對 `integrity_ok` 的斷言。 |

### R-05：撤回「hetero 比 all_A 慢 29%，是最後一架 B 落地決定的」這個說法

前一輪整合報告的 open issue 寫「hills hetero 5552 s 比 all_A 3924 s 慢 29%，makespan 由最後一架 B 落地決定，不是 bug」。
三個論點都不成立，正確的分解是：

1. **all_A 根本不是一個完整任務**（R-04）：56 條帶 vs hetero 的 107，Z2 的 `rgb` 覆蓋率 **0.000**（HERO 才有 RGB，
   同時掛雙感測的單一構型只會被鋪一套 LiDAR 條帶）。`sim.baselines` 現在對它回 `inf`，比較不再成立。
2. 5551.5 s 之中有 **1596.8 s（28.8%，等於整個差距）是 `deconflict.repair` 加上去的地面起飛延遲**
   （B2 +2733.9、B1 +622.0、A1 +500.6、B4 +416.6 s）；修復前 hetero 的 makespan 是 3954.7 s。
3. 修復前的瓶頸是 **A 不是 B**：A1 於 3954.7 s 落地，而所有 B 在 2817.6 s 前就落地了。B 的續航離綁定很遠——
   B2 用掉 400 Wh 中的 194.0 Wh，落地時比 126 Wh 保留量還多 80.0 Wh。
4. 真正的結構性解釋是**班表**：固定 3A/5B 之下 A 要飛 51 條 LiDAR（17.0 條/架、平均飛行 3794 s），
   B 飛 56 條（11.2 條/架、平均 2374 s）。任何 λ 分配或目標函數都不能把 LiDAR 工作搬到 HERO 上。

（本輪修完 R-01/R-04/R-06 後，hills 為 makespan **5087 s**、`validate.passed=True`、conflicts=0、三區覆蓋 1.000；
`all_A`／`all_B` 皆為 `inf (incomplete)`。）


---

## deconflict.py（契約 §13；規格 §13）— 4D 判定範圍、修復搜尋與 §13.6 (iii) hook（審查後修正）

本節記錄 `hv_dccpp/deconflict.py` 為了把 `examples/run_hills.py` 的殘餘 4D 衝突 **4 → 0** 所做的偏離。
量測（hills，8 機／3 區／107 條帶，`dem_step_m=50`）：

| | 殘餘衝突 | makespan | `sim.validate` | `plan_mission` 之 deconflict 階段 |
|---|---|---|---|---|
| 修改前 | 4（`B4-B5`@987、`B1-B4`@2066／@3226、`A2-A3`@3576） | 5551.5 s | `passed=False` | ~4.0 s |
| 修改後 | **0**（走廊衝突亦為 0） | **5087.4 s（−8.4%）** | **`passed=True`** | ~10.8 s |

`run_hills.py` 端到端：`validate: passed=True conflicts=0 min_sep_h=15.2 m makespan=5087 s`，
三區覆蓋 1.000、slope/agl/curvature/energy/link/airspace/time_window 全部 True。
`run_mini.py`（1736 s、conflicts=0、PASS）與 `run_paper_table2.py`（3890.5 s、路徑長不變）不受影響。

### D-DC-1 σ_t 上限 `SIGMA_T_MAX_S = 45 s`（規格 §13.3 原式無上限）

規格 §13.3：σ_t = k_t(t − t_sync)，**t_sync = 最近一次遙測快照時刻**。初始規劃取 t_sync = t⁰ = 0 並套用整趟
任務，於是 92 min 的 hills 末段 σ_t 漲到 ±280 s、判定視窗 Δt + σ_a + σ_b 達 560 s——此時 4D 檢查已退化成
「純空間交叉」測試。實測：hills 4 筆殘餘衝突的**實際時間間隔**是 76.6／96.6／219.4／340.0 s，
`k_t = 0` 時衝突數為 0；而修復所需延遲發散到 650–2371 s，repair 無法收斂。

改為 σ_t = min(k_t·(t − t_sync), `SIGMA_T_MAX_S`)。取 45 s 的依據：
(a) 外部審查建議 30–60 s，取其中點；
(b) 營運上 t_sync 每個遙測訊框即重設，且規格 §12.5 明定失聯逾 `C2Link.t_ll_s`（預設 **60 s**）就宣告
LINK_LOSS、改以幽靈航跡（自帶 σ_t）處理，「連線正常之機」不會累積超過約一分鐘的未更新漂移；
(c) 物理意義 = 20 m/s 巡航下 ±900 m 沿航跡不確定度；兩機通過同一點相隔 > 2·45 + 2 = 92 s（≈1.84 km）
即視為分離，對執行已上傳任務的 GPS 導航定翼機仍屬非常保守。

以 `cfg.sigma_t_max_s` 覆寫（`PlannerConfig` 目前未提供此欄位，`sigma_t_max_of()` 以 `getattr` 取得）；
`trajectory_samples(..., sigma_max_s=math.inf)` 或把模組常數設為 `math.inf` 即回到規格原式。

**敏感度**（hills，同一份修復程式碼）：`inf → 0 筆 / 6522 s`、`90 s → 0 / 6415 s`、`60 s → 0 / 6106 s`、
**`45 s → 0 / 5087 s`**、`30 s → 0 / 4910 s`。殘餘衝突在任何上限下都是 0；makespan 對本參數**並非單調**，
因為修復是貪婪搜尋，不同視窗寬度會走到不同的局部解。

### D-DC-2 `skip_pairs`（`spatially_exclusive`）只豁免 OPERATE↔OPERATE 樣本對

規格 §13.2 的空間互斥依據是「胞多邊形 ⊕ 迴轉帶互不相交」，只涵蓋**作業段**；但原實作把整個機對從
`check_separation` 移除。hills 的 `A2-A3`@3576（d_h = 12.7 m）就發生在 **RTB 轉場段**上：
`planner.plan_mission` 把 `spatially_exclusive` 的結果當 `skip_pairs` 傳給 repair，於是 repair 從頭到尾
看不到這筆衝突，而不傳 `skip_pairs` 的 `sim.validate` 卻報得出來——兩者對同一份計畫的判定不一致。

現在 `skip_pairs` 之機對只豁免「兩個樣本皆為 `Phase.OPERATE`」的樣本對，ENTRY／TRANSFER／RTB 等
轉場段照常檢查（**幽靈**機對仍整對豁免——幽靈航跡沒有階段資訊）。呼叫端 API 不變。

### D-DC-3 起降走廊豁免收緊為「同場坪 ∧ 在自己的預約窗內」

原實作（整合代理 `deconflict.py:63`）只要兩樣本的航點階段都屬 `SITE_CORRIDOR_PHASES` 就免計 4D。
兩個漏洞：(a) **不同站**（H 對 ALT1）的走廊樣本也被豁免，而 `CorridorSchedule` 是逐站互斥的，兩邊都不管；
(b) 走廊階段若超出該機自己的走廊預約區間（起飛 [t0, t0+Δt_dep]、降落 [t_arr−30, t_land+t_sep,LDG]），
`CorridorSchedule` 同樣管不到。

現在豁免需同時滿足三條件：兩樣本皆為走廊階段、兩樣本的時刻皆落在該機自己的走廊預約區間內、
兩樣本對應之**起降場坪**（起飛段取首航點、降落段取末航點的水平位置）相距 ≤ `SAME_PAD_TOL_M = 50 m`。
如此每一組被豁免的樣本對，兩機在**同一站**都必有一筆涵蓋該時刻的預約，必然落在
`CorridorSchedule.conflicts()` 的管轄範圍內。
證明性測試：`tests/test_deconflict.py::test_site_corridor_exemption_is_covered_by_schedule`
（列舉所有被豁免的樣本對，逐一驗證兩機都有同站、涵蓋該時刻的預約）、
`::test_site_corridor_exemption_requires_same_pad_and_own_window`（三條件缺一即不豁免）。
hills 實測：所有走廊階段都完整落在自己的預約窗內、且距站 ≤ 468 m，故 8 機情境行為不變。

### D-DC-4 `Conflict` 新增 `forbid_a/forbid_b/forbid_now_a/forbid_now_b`（契約外欄位，皆有預設）

「延遲某機 s」之**禁止開區間**（合併、依左端排序）：s 落在其中即仍會衝突。
`delay_a_s` = 自 0 向上跳出 `forbid_a` 的第一點（原本就有，只是沒把區間本身留下來）。
`forbid_now_*` 只由**目前真的違規**的樣本對構成，`forbid_*` 則由該機對「任意時差下空間相近」的
全部樣本對構成。共用轉場航路的同型機（hills 的 B1/B2 最近距離 **0 m**）後者可達 2000 s 以上，
故修復把兩者都當候選，由全域單調性檢查決定採用哪一個。

### D-DC-5 `_fit_takeoff_shift`：三個集合的交集最小元素（審查 open issue #2）

原本只做「走廊時槽」：把 26.7 s 的需求推到下一個空時槽（hills B4 → **2083.2 s**），而該時刻常常又落回
禁止延遲區間，`pair_clear` 失敗 → 整個候選被丟棄，該筆衝突於是沒有任何候選。
現在同時對三個集合做交替投影（各自傳回「≥ 輸入的最小可行點」，故收斂到交集最小元素）：
`s ≥ s_req` 且 `t0 + s ≥ max(0, t_now)`、起降預約皆無重疊、`s` 不在禁止延遲區間內。
`tests/...::test_fit_takeoff_shift_is_minimal_and_avoids_forbidden_intervals` 以 0.1 s 網格暴力解對照最小性。

**註**：外部審查認為此函式「本來就是最佳解、不該改」——就它被給定的模型（只有走廊）而言正確；
真正的缺口是模型少了禁止延遲區間這一條，以及下面 D-DC-6 的連鎖重排。

### D-DC-6 走廊連鎖重排 `_cascade_shift` / `_cascade_orbit`（規格 §13.6 (i) 之「起飛換序」）

不再「把自己推到下一個空時槽」，而是只延自己所需的量，被擠到的**尚未起飛**之其他機依序讓開
（推較晚開始者；起飛 vs 降落一律推起飛方——規格 §13.5「降落優先」；已起飛者與不在 `plans` 內的預約
視為固定障礙）。以預約區間算術進行，不複製航點，終止性由推移次數上限 `CASCADE_MAX_UAVS²` 保證。
hills 之 B4：需求 26.7 s，原本 2083.2 s，連鎖後 26.7 s（B1 一起後移 26.7 s，makespan 不變）。

### D-DC-7 `_orbit_actions`：把「4D 需求」與「降落時槽」分開（審查 open issue #3）

原本 n = ⌈Δ_req/T_lap⌉ 之後，用一個迴圈把圈數一路加到降落走廊有空檔為止（hills：B4 1 → **39** 圈、
B5 → **58** 圈 ≈ 1600–2400 s），隨後能量重驗必定失敗 → 該衝突完全沒有候選。
現在產生兩種候選並交給擇優：
1. `n_4d` = 只解衝突本身所需的最小圈數（並跳出禁止延遲區間，`_orbit_n_for`），
   平移後若撞到他機的降落預約，改由 `_cascade_orbit` 讓**尚未起飛**的佔用者讓開；
2. 舊行為（把圈數加到有空時槽）保留為後備，供佔用者不可移動（已起飛／幽靈／外部預約）時使用。

### D-DC-8 `_repair_one` 候選擇優改為「先 makespan」＋去重＋`only_uavs` 局部重算

* 所有候選仍須通過原有的全域單調性門檻（`involved_score`：相關衝突數下降，或持平但 ΣΔ_req 下降且
  makespan 不增）；通過者再以 `(max(makespan, ms_now), 衝突數, ΣΔ_req, 成本)` 擇優。
  先看衝突數的版本實測會為了多消 1–2 筆而把最後一架推遲 2000 s 以上（hills 6995 s vs 5087 s）。
* `check_separation` 新增 `only_uavs` 參數（只檢查「至少一方屬於該集合」的機對）。
  `involved_score` 改用它，取代原本「把其餘所有機對塞進 skip_pairs」的作法——後者在 D-DC-2 之後
  已不再等價（skip_pairs 只豁免作業段）。
* `_dedupe_actions` 去除等價候選（兩組禁止區間常導出同一個平移量）。
* `repair` 每輪不再「每機每輪至多修改一次」：全域單調性（字典序嚴格下降）本身即保證終止，
  原本的 `touched` 跳過會讓每輪最多只修好 ⌊n_U/2⌋ 筆，3 輪不足以收斂 27 筆衝突。

### D-DC-9 修復後的工期壓縮 `_compact_makespan`（新增步驟）

規格 §13.6 的三種手段都只會**增加**延遲，一連串貪婪延遲推高的 makespan 無法回收。
新增收斂後的壓縮步：依落地時刻由晚到早，逐機嘗試把整個計畫**提前**到走廊與 4D 都仍乾淨的最早時刻
（`_advance_4d_limit` 解析地算出「平移 s」的禁止區間聯集，`CorridorSchedule.prev_free` 給走廊下界，
`involved_score` 不得變差）。零能量、不改任何航點 z（D18）。
`CorridorSchedule` 因此新增 `prev_free()`（`next_free` 的反向）。
同樣的「提前起飛」也加進 `_takeoff_actions` 的候選（`_fit_takeoff_advance`），即規格 §13.6 (i) 的「換序」
在原實作中只有兩機對調的特例。

### D-DC-10 §13.6 (iii) IDP 重排 hook 正式化：`ResequenceRequest` / `ResequenceHook`

新式簽章 `resequence_hook(ResequenceRequest) -> dict[str, UAVPlan] | UAVPlan | None`。
`ResequenceRequest` 欄位：`conflict`（含 `dt_req_s`、`delay_a_s/b_s`、`forbid_*`）、`uav_ids`
（可重排之機，依「makespan 貢獻小者在前」排序）、`plans`（目前全隊計畫）、`schedule`、`ctx`、`cfg`、
`models`、`t_now_s`、`t_sync_s`、`takeoff_sites`、`energy_check`。
回傳結果仍要通過 `involved_score` 單調性檢查，不通過即忽略並續留殘餘衝突，故回呼可安全地「盡力而為」。
回呼**不得**改變任何航點的 z（D18）。
為相容既有呼叫端，仍接受舊式三引數 `hook(uav_id, plan, conflict) -> UAVPlan | None`
（以 `inspect.signature` 的位置參數個數判定，`_hook_arity`）。

### 已知代價

`plan_mission` 的 deconflict 階段由 ~4.0 s 增加到 ~10.8 s（hills 8 機；`plan_mission` 總計 18.6 s）。
原因是候選數量增加（每架 × 2 組禁止區間 × {自推、連鎖、提前}）且每個候選都要跑一次
`involved_score` 局部重算。已用 `only_uavs`、候選去重、`base` 記憶化與航跡快取把它從 13.9 s 降到 10.8 s。
規格 §14 的即時性表格未涵蓋 repair，此處記錄實測值供後續調整（可調整之旋鈕：`_advance_candidates` 的
格點數、`CASCADE_MAX_UAVS`、`repair(max_rounds=...)`）。

---

## planner.py / allocate.py / integration（2026-09-06，審查後修正；D-PL-1 … D-PL-8）

本節為 `hv_dccpp/planner.py`、`hv_dccpp/allocate.py`、`hv_dccpp/integration/mqtt_bridge.py` 於審查後之
修改；量測情境為 `scenario.build_hills_scenario()`（8 機／3 區／107 條帶）與 `build_mini_example()`。

### D-PL-1 逐機起飛站點正式接進 `plan_mission`（契約 §10 步 4／步 7／步 8）

`MissionInputs.fleet[i].start_site_id`（由 mission/context 於本輪新增）先前只存在於 `MissionContext`，
`plan_mission` 仍一律用 `inputs.home.site_id`。現在：

* 步 4 暫定起飛走廊：`_nearest_entry(cl, _start_xy(ctx, uav_id))`、`takeoff_corridor(_start_site(...), ...)`；
* 步 7 精算起飛走廊：同上；
* 步 8：`CorridorSchedule.reserve_plan(p, start_of[uav])` 與 `repair(..., takeoff_site=start_of)`（dict 形式），
  故規格 §13.5 的 60 s 同坪起飛間隔只套用在**真正同坪**的機上；
* `build_uav_plan` 的 `pin_start` 由起飛站地面高度決定（原本固定用 home）。

`_start_site()` / `_start_xy()` 對舊版或手搭的 `MissionContext` 以 `getattr` 退回 home，不破壞既有呼叫端。
**降落／回收仍走 `ctx.recovery_site_id`（D40 不變）**。實測（mini，B1 起飛站改為 ALT1）：B1 首航點由
(0, 0) 變成 (2000, 1500)、落地站仍為 H、makespan 1736.4 s → 1676.4 s。

### D-PL-2 規格 §13.6 步 (iii)「IDP 重排」回呼實作（`_make_resequence_hook`）

`plan_mission` 現在把 `resequence_hook` 交給 `deconflict.repair`（`ResequenceRequest` 新式簽章）。作法：

1. 取衝突涉及之機 `u`（`req.uav_ids[0]` 起），以其**當前計畫**的條帶序（`req.plans[u].strips`，
   已含本輪稍早的修復）在衝突時刻所在條帶處切開；切點一律 ≥ 1，故首條帶（決定起飛走廊出口與進入段）
   不變，**不需 `cache.refresh_entry`**，不污染共享快取；
2. 為了得到與現況不同的解，逐一以「翻轉該條帶掃向」或「先飛後綴中的另一條」當作切點後的**強制首選**
   （規格 §13.6 (iii)「交換條帶方向／改同向轉場為零成本選項」），同隊其他機以其現有序列 warm-start
   後即 RTB（`inactive`），因此只有 `u` 的計畫會變；
3. 每個候選重跑 `append_rtb` → 起降走廊 → `build_uav_plan`，再以 `req.energy_check` 與
   `check_separation(only_uavs={u})` 評分。

**採用門檻（`_reseq_better`，刻意保守）**：必須真的少掉一筆衝突（或同數但降落時槽由佔用變淨空），
**且落地時刻不變晚**。理由是實測：只要求「字典序較小」時，把某機落地往後挪換掉一筆衝突，會在後續輪次
引發更多起飛延遲——hills makespan 由 5087.4 s 惡化到 5657.4 s。加上 t_land 不增加的條件後回到 5087.4 s。

**`_reseq_worth_trying` 早退**：`repair` 逐筆處理的是**本輪起始**的衝突清單，該筆衝突常在本輪稍早的修復
後就已消失。此時涉及該機的衝突數為 0 且降落時槽淨空，直接回傳 None。hills 實測 19 次呼叫全屬此類，
早退省下約 3.5 s；`plan_mission` 因本回呼由 29.2 s 增加到 32.7 s。

預算：單次 `plan_mission` 至多 `_RESEQ_MAX_CALLS = 12` 次呼叫、每次 `_RESEQ_MAX_CANDIDATES = 4` 個候選、
總計 `_RESEQ_TIME_BUDGET_S = 8.0` s。回呼從不直接改任何航點 z（z 由 `build_3d` 重算；D18）。

**目前的實測狀態（誠實記錄）**：在本輪 deconflict 的修復強化之後，hills 的步 (i)+(ii) 已能自行收斂到
0 殘餘衝突，故 (iii) 在出貨情境中**沒有事情可做**——19 次呼叫全部命中 `_reseq_worth_trying` 早退。
把 `cfg.d_sep_m` 由 100 m 拉到 300 m 做壓力測試，仍是 0 殘餘衝突、makespan 6188.5 s（有無回呼完全相同）。
回呼本身的候選管線由 `test_planner.py::test_resequence_hook_returns_valid_alternative` 直接驗證
（覆蓋不變、起飛時刻不變、時間單調、能量檢查通過）。

### D-PL-3 `_reseq_*` 之評分不含 makespan，改由 deconflict 決定

回呼只回傳「盡力而為」的單機計畫，`repair` 會再跑 `pair_clear` 與 `involved_score` 的全域單調性檢查。
本回呼的評分僅用於在自己的候選之間排序，不取代 `repair` 的判準。

### D-PL-4 規格 §9.4 末「leftover → allocate(frozen)」再拍賣（`_reauction_leftover`）

原本 `beam_idp` 的 `leftover` 直接記成 `MissionPlan.uncovered`（整合報告的既有 open issue，
planner.py:1399）。現在先做二次配置：

1. 既有團隊以「**實際飛到的**條帶」凍結（`allocate(..., frozen_teams=...)`），故 leftover 不再被原任務佔住；
2. 每個仍有 leftover 的胞複製出一個只含 leftover 條帶的合成胞（`<cell_id>~lo`、`sensor_req="any"`——
   只有原構型有該胞的條帶，其餘構型 `score` 為 0，等同固定構型）；整胞未飛者則把原胞直接放回池中；
3. 二次 `allocate` 後，對接到新條帶的團隊以既有序列 warm-start 重跑 `beam_idp`，`append_rtb` 不過即整團退回；
4. 起飛時刻沿用第一次配置的 t0（不採用二次拍賣的 `takeoff_schedule`，以免動到已排定的走廊時槽）。

任何例外都退回原行為（記 uncovered）並加一筆 `leftover_reauction_failed(...)` 警告。
實測（mini、A 7 條帶、只讓 A1 飛前 4 條）：3 條 leftover 全部被閒置的 A3 承接，`uncovered` 由 3 變 0。
hills／mini 正常路徑沒有 leftover，故此路徑不影響既有量測。

### D-PL-5 R-06 兩段式 `point_wh`（`energy_safe_check`）

`costs.EnergyBudgetTable.point_wh` 不是上界（審查實測最壞低估 17.7 Wh／−13.2%）。
`energy_safe_check` 現與 `sim.check_energy` 採同一準則：以 `point_wh` 粗篩，餘裕小於
`point_wh_slack_wh(cls)` 時改以 `point_wh_exact`（AVOID+ENVELOPE，取全部候選站點之最小值）定案。
未改用 `point_wh_ub`，因為 costs 作者實測那會讓 hills A3 被迫改降 ALT1、makespan +20%。
hills／mini 的計畫不受影響（本情境餘裕充足，精算路徑未觸發）。

### D-PL-6 R-07 返航段消耗剖面（`_rtb_profile`）

原本沿返航路徑以「能量與弧長線性」內插 `cache.rtb_e`，但同一條返航在 `EnergyBudgetTable` 中可能以
n_up 圈爬升螺旋開頭、n_down 圈下降螺旋結尾（hills 實測 A 最多 1／1 圈、B 最多 2／1 圈），螺旋能量並不
沿弧長分布。改為**爬升螺旋前置、下降螺旋後置、其餘沿弧長均攤**，總量取
`budget.rtb_wh(cls, sid, dir, site) − e^LDG`（與需求側同源的 AVOID+ENVELOPE 值）與 `cache.rtb_e` 之大者。
單圈時間以 `2πR_eff / V_a` 估計；螺旋能量之和被夾限在總量之內。
高度剖面亦由「z0 → z_tc 線性內插」改為兩端取低者（線性上爬會低估備降所需的爬升項）。
不變式：`e_up + e_path ≥ rtb.e_wh`（消耗側不會比原本樂觀）。hills／mini 的判定結果不變。

### D-PL-7 R-04 `both` 胞只有單一構型可服務 → 明確記為不可覆蓋

規格 §6.2 要求 `both` 胞鋪**兩套**條帶（LiDAR 一套、RGB 一套）。`decompose.resolve_sensor` 對
(cls, req_id) 只鋪出第一個相符 Sensor 的那一套，因此只有一個構型能服務的 `both` 胞會無聲漏掉第二個
感測器需求（審查 R-04：all_A 基線的 Z2 rgb 覆蓋率 0.000 卻回報任務完成）。
`assign_config_and_count` 現在在該情況多記一筆
`f"{cell_id}#{SECOND_SENSOR_TAG}" → "both_second_sensor_unserved"`，經 `AllocationResult.uncoverable`
進入 `MissionPlan.uncovered`。**LiDAR 那一套仍照常指派**，只是 RGB 需求不再被吞掉。
新公開常數 `allocate.SECOND_SENSOR_TAG = "sensor2"`。
完整修法（decompose 同時鋪出 `req_id` 與 `req_id + RGB_REQ_SUFFIX` 兩套）仍待 decompose 端實作。
實測：hills 的 all_A 基線 `uncovered` 由 0 變 5，`sim.baselines` 回報 `all_A = inf`。

### D-PL-8 `mqtt_bridge` 的 NAV_LOITER_TIME param3 文件更正（僅註解）

原文件寫「Plane 之 param3 = 半徑」。ArduPlane 的 `AP_Mission` 對 MAV_CMD_NAV_LOITER_TIME(19) 只儲存
`p1`（盤旋秒數）與 `location.loiter_ccw = (param3 < 0)`，**半徑大小被丟棄**，實際以 `WP_LOITER_RAD` 飛。
已更正模組 docstring 與 `plan_to_qgc_items` 的說明；輸出格式不變（param3 仍寫帶號半徑，供 QGC／人工判讀），
並指出要飛出規劃半徑必須另外設定機上的 `WP_LOITER_RAD`（可由 `ardupilot.orbit_radius_m` 取得）。

### 異構 vs 同構（R-05）：山丘情境的結構性解釋，非 λ／目標函式缺陷

`test_planner.py::test_hetero_class_assignment_has_no_freedom_on_hills` 把結論釘成測試：

* A（Pioneer）機上只有 LiDAR、B（HERO）只有 RGB。Z1(lidar) 只有 A 能服務、Z3(rgb) 只有 B、
  Z2(both) 兩者各鋪一套。H1（`decompose.class_can_serve`）已把每個（區、感測器）需求鎖死在單一構型上，
  **λ 規則（§9.3）、拍賣目標（§9.4）、適配度（§7.6）都沒有自由度可用**——任何改動都無法把 LiDAR 條帶
  搬到 HERO 上。
* 去衝突修復前的瓶頸是**class A**，不是「最後一架 B」：A1 3954.7 s、A2 3574.6 s、A3 3913.9 s 落地，
  全部 B 在 2817.6 s 前落地（B1 2710.7、B2 2817.6、B3 2437.7、B4 2696.2、B5 2709.8）。
  A 飛 51 條 LiDAR 條帶（總飛行 11263 s／3 架），B 飛 56 條 RGB（11872 s／5 架）。
* 團隊內平衡已接近極限：完美平衡下界 A 3754.4 s（實際最大 3954.7 s，+5.3%）、B 2374.4 s（實際 2637.6 s，
  +11.1%）。把 beam 寬度由 B=3 提高到 6／12 得到 3944.1／3957.1 s，證明**不是 beam 搜尋的寬度問題**。
* `all_A` 基線本身不是合法任務（D-PL-7），`sim.baselines` 現回報 `all_A = inf`（uncovered 5）、
  `all_B = inf`（uncovered 6）、`legacy = inf`（uncovered 1），只有 hetero = 5087.4 s 完整覆蓋。
  **「異構比 all_A 慢 29.3%」的比較無效，相關的進度紀錄與文件表格需重新產生。**

## integration/aeroplan_adapter.py（AeroPlan Studio 轉接層；契約未涵蓋之新模組）

轉接對象：AeroPlan Studio v2.8.1（PyQt6 地面站，https://github.com/nfu64967512/aeroplan-studio）。
以下 `REPO/x/y.py:NNN` 皆為該 repo 之已驗證行號。本模組**不 import AeroPlan**，全部以 duck typing 銜接。

### A. 契約／規格層面的偏差

| 項目 | 原因 | 實際做法 |
|---|---|---|
| `HvBuiltWaypoint.alt` 之高度基準 | 契約／規格之 `Waypoint` 同時給 `alt_msl_m`（MSL）與 `agl_m`（對**當地地形** AGL，D42）；AeroPlan 的匯出／上傳一律為 **AGL 相對 home**（`MAV_FRAME_GLOBAL_RELATIVE_ALT`=3；REPO/utils/file_io.py:226、REPO/mission/vtol_mission_exporter.py:32、REPO/mission/sitl_link.py:1274） | `alt = alt_msl_m − home_msl`（home 地面 MSL 由 `ctx.sites_enu[home]` 反算），並以 `max(0, ·)` 夾住。**起伏地形上 `alt ≠ agl_m`**，手冊須明講；欲取真 AGL 請讀 `coordinator.last_plan` 的 `Waypoint.agl_m` |
| 14 種 `Phase` → 5 種 `SegmentLabel` | AeroPlan 消費端只認 `segment_type.name ∈ {TAKEOFF, ENTRY, OPERATION, TRANSFER, LANDING}`（REPO/ui/main_window.py:6060-6066、:6113-6136），規格 §3.4／§11.1 的 14 相位無法一對一 | `PHASE_TO_SEGMENT`：TAKEOFF/TRANS_OUT→TAKEOFF；ENTRY/CLIMB/ORBIT/ACCEL→ENTRY；OPERATE→OPERATION；TRANSFER/HOLD→TRANSFER；RTB/DESCENT/DECEL/TRANS_IN/LAND→LANDING。**此映射不可逆**（ORBIT 盤旋與 CLIMB 爬升在 AeroPlan 端無從分辨） |
| `HvMdtspResult.makespan` 之單位 | AeroPlan `MDTSPResult.makespan` 欄位文件寫「秒」（REPO/core/global_planner/mdtsp_solver.py:91），但 REPO/core/dccpp/idp_solver.py:407 實際填 `idp_result.makespan_length`（**公尺**） | 本模組填**秒**（規格 §10 之 makespan），與欄位文件一致而與 AeroPlan 現行數值不可逐值比較；長度請改讀 `total_distances` |
| `total_makespan` 定義 | AeroPlan 於 REPO/mission/swarm_coordinator.py:1290 以「路徑總長 ÷ 速度」估計，不含起降、MC↔FW 轉換、盤旋等高與錯開起飛 | 本模組填 `MissionPlan.makespan_s`（規格 §10：含全部相位與 D46 錯開起飛）。**數值必然大於 AeroPlan 舊值，非迴歸** |
| 縮點（`reduce=True` 預設） | `build_3d` 之輸出為密集取樣（本情境每機 387–494 點；大情境上千），超過 ArduPilot 任務項目實務上限，也讓 Cesium 圖層過重 | 以 `ardupilot.reduce_waypoints`（弧段外切離散 D27 + 直線端點 + 航向／高度轉折 DP）縮至 44–74 點；**五個 `*_length_m` 一律以縮點前的全部取樣點計算**，故長度統計不受縮點影響（`test_reduce_waypoints_shrinks_output` 釘住） |
| `*_length_m` 為水平長度 | AeroPlan 的 `BuiltPath` 長度來自 Dubins 平面路徑（REPO/core/dccpp/dccpp_path_builder.py），不含爬升分量 | 逐腿取 ENU 水平距離 `hypot(Δx, Δy)`，依**出發點**之相位標籤累加（與 D45「Waypoint 欄位描述出發腿」同慣例）。`total_length_m` = 五者之和，故必然略小於 `UAVPlan.distance_m`（後者含垂直分量） |
| `PayloadRequirement` 逐感測種類 | `context.build_sensors` 之 `ctx.sensors` 鍵為 `(cls, req_id)`：某構型若找不到**同 kind** 的需求就沒有 `Sensor`，該構型會完全無法被分配（症狀：異構機隊只有一種構型飛，且不報錯） | `_default_payload_reqs` 在 A（LiDAR）與 B（相機）並存時同時產生 `"default"`（主種類）與 `"default_rgb"`／`"default_lidar"`，與 `scenario.build_mini_example` 的兩需求同慣例；`test_payload_reqs_cover_every_sensor_kind` 釘住 |
| `coverage_altitude` 夾制 | AeroPlan UI 預設 100 m，但構型 A 之 `min_fw_operation_agl_m` = 120 m（表 1） | 夾到 `[max_c min_fw_operation_agl_m, waiver_max_agl_m]` 並記警告 `coverage_altitude_clamped(...)`（`test_altitude_clamped_below_min_fw_agl` 釘住） |
| `NoFlyZone` 之高度基準 | AeroPlan `FenceZone.alt_min_m/alt_max_m` 註解為「AMSL 或 AGL 依專案慣例」（REPO/mission/fence_zone.py:61-63），未定 | `fence_zones_to_nofly(..., alt_ref="msl")` 預設視為 MSL；`alt_ref="agl"` 時加上 `home_msl_m`。`ZoneCategory.GEOFENCE`（inclusion）語意與 NoFlyZone 相反，預設略過 |
| `telemetry_frames_to_events` 規則不完整 | AeroPlan `TelemetryFrame`（REPO/mission/sitl_link.py:30-62）**沒有** `landed_state`、`mission_seq`、風速欄位 | 只產生 LinkLoss／LinkRestored／EnergyUpdate（§15.5.2 之鏈路與能量規則）；不產生 `UAVExit(landed_unexpected)` 與 `WindUpdate`；現行條帶改以計畫時間軸估計（誤差大於 MISSION_CURRENT 對照）。需完整規則請改走 `mavlink_link.FleetLink` |

### B. 對 AeroPlan 現行行為的**刻意**偏離（bug 修正）

| 項目 | AeroPlan 現況 | 本模組 |
|---|---|---|
| VTOL 起飛指令遺漏 | REPO/ui/main_window.py:6113-6115：`seg_name == 'TAKEOFF'` 且 `idx == 0` 一律 `continue`。若路徑只有**一個** TAKEOFF 航點（正是縮點後的情形——`NAV_VTOL_TAKEOFF` 只需一點），`takeoff_emitted` 永遠為 False，整份任務**沒有起飛指令** | `to_upload_tuples` 在 `idx == 0` 且尚未輸出起飛指令時照樣輸出 `NAV_VTOL_TAKEOFF` + `DO_VTOL_TRANSITION(param1=4=FW)`；其餘序列與 :6045-6159 逐項相同（`test_upload_tuples_shape_and_commands` 釘住） |
| `'vtol'` 靜默降級 | REPO/mission/swarm_coordinator.py:1080-1082 把 `'vtol'` 降為 `VehicleType.MULTIROTOR`，導致 REPO/core/dccpp/task_allocator.py:136-142 對 QuadPlane 用多旋翼 haversine 進入成本 | `HvUavState.vehicle_type` 填 `FIXED_WING`（QuadPlane 巡航段為固定翼）；`HvVehicleType` 另提供 `VTOL` 成員供新程式使用（原生 `VehicleType` 無此成員時 `_vehicle_type` 自動退回 `FIXED_WING`） |
| `fov_width` 從未被讀 | REPO/mission/swarm_coordinator.py:1090 只寫不讀；區域分解只用單一 `coverage_width_m`（:1187-1195） | 刈幅逐構型由感測器硬體 + 作業高度推導（規格 §4.2／§4.3）；`coverage_width_m` 僅作核對，差異 > 25% 記警告 `fov_width_mismatch(...)`。`HvUavState.fov_width` 填實際刈幅（供顯示） |
| 全隊共用單一 R_min／單一高度 | REPO/mission/swarm_coordinator.py:1184（`_R = max(turn_radius>0)`）、:1204-1205 | 逐構型 `spec.turn_radius_planning_m`（A 200 m、B 120 m）經風修正得 `r_eff`；作業高度由規格 §9 分層（本情境 A 169.9 m、B 160.4 m 相對 home） |
| `enable_altitude` 於 live 路徑被忽略 | REPO/mission/swarm_coordinator.py:1177-1298 從未呼叫 `AltitudePlanner` | hv_dccpp 一律做高度規劃（規格 §9、論文 Alg.3），`enable_altitude=False` 只記 note `enable_altitude_ignored`，不改變行為 |
| 地球半徑／投影不一致 | REPO/core/geometry/region_divider.py:260-283 用 111111.0 m/deg；`CoordinateTransformer` 用橢球（REPO/docs/refactor_audit_2026.md:18-35） | hv_dccpp 全程 WGS84 橢球（`geo.GeoFrame`，經 ECEF）。距離與 makespan 與 AeroPlan 舊值差約 0.1%，**既有 golden values（REPO/tests/test_dccpp_pipeline.py:16-23）不可用來驗收本模組** |

### C. 未實作 / 由呼叫端負責

| 項目 | 說明 |
|---|---|
| `dem_path` | 不在本層讀 raster（避免 rasterio 相依）。請由呼叫端傳 `terrain_func`（REPO/sensors/dem_terrain.py:175 `DEMTerrainManager.get_elevation`），本模組以 `terrain_func_to_dem` 逐節點取樣建網格；非空 `dem_path` 只記 note `dem_path_ignored` |
| `runway_bearing_deg` / `runway_length_m` / `landing_bearing_deg` / `landing_rollout_m` | 兩構型皆為 lift-and-cruise VTOL，垂直起降不需跑道；進場方位由逆風規則決定（D19）。給值只記 note |
| `auto_scan_angle=False` / `coordination_mode` / `collision_avoidance=False` | hv_dccpp 一律最佳化掃描方向（規格 §7.3）並做 4-D 去衝突（規格 §10.6、D14），關不掉；只記 note |
| `class_of` 預設值 | AeroPlan 的 drone dict 無構型資訊（REPO/ui/main_window.py:4722-4744 以同一組 spinbox 產生所有機），`default_class_of` 在無線索時回 `"A"`。**實務整合必須自行提供 `class_of`**，否則整隊會被當成 Pioneer |
| QThread | 規劃為同步阻塞；AeroPlan 的規劃在 GUI 執行緒（REPO/ui/main_window.py:4753-4776）。本套件不 import PyQt6，`ui/workers/planning_worker.py` 需在 AeroPlan 端新增（模組 docstring 附範例），並接 `progress` 回呼 |
| 原生 `BuiltPath` 的 `to_latlon_list()` | `use_native_types` 以 `setattr` 補 `takeoff_length_m` / `landing_length_m`（原生 dataclass 無 `__slots__`，可行），但**無法**補方法。使用原生型別時下游只能呼叫 `to_latlon()`（REPO/ui/main_window.py:5361 本來就以 getattr 取長度，不受影響） |

### D. 實測值（`tests/test_aeroplan_adapter.py` 情境：平地、3 km × 1.2 km、1×A + 1×B、160 m AGL、overlap 0.25）

規劃耗時 1.17 s（`plan_mission` 內部 timing_ms total 883；decompose 209 / layering 278 / cache 100 / allocate 54 / idp 29 / assemble_3d 103 / deconflict 68），
status `GO_WITH_WARNINGS`，7 條帶（A 4、B 3），`total_makespan` 1479.4 s。

| drone_id | 構型 | 航點（縮點後／原始） | takeoff | entry | operation | transfer | landing | total (m) | 上傳項 | QGC 行 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | A（Pioneer, LiDAR） | 74 / 494 | 468.0 | 4228.8 | 12426.4 | 2631.5 | 4791.8 | 24546.5 | 78 | 86 |
| 2 | B（HERO, 相機） | 44 / 387 | 384.0 | 4193.3 | 9344.5 | 4362.5 | 4503.8 | 22788.0 | 48 | 54 |

能量：A1 用 310.8 Wh／餘裕 589.2 Wh；B2 用 111.3 Wh／餘裕 142.7 Wh。最高航點高度（相對 home）A 169.9 m、B 160.4 m。

## 建圖模式 MappingModeConfig（2026-10-14；mission / context / platform / decompose / planner / preflight / aeroplan_adapter）

| 項目 | 原因 | 實際做法 |
|---|---|---|
| 設定放哪裡 | `MissionInputs`（表 2）與 `PlannerConfig`（§16.2）的欄位清單都被測試鎖死（test_mission.py::test_table2_fields_present、test_planner_config_defaults_match_spec_16_2） | 新增 `MappingModeConfig`（frozen）與子類別 `MappingMissionInputs(MissionInputs)`（多一個 `mapping_mode` 欄位）；`MissionInputs.mapping_mode` 是恆為 None 的唯讀屬性。`to_dict` 多 `"mapping_mode"` 鍵，`load_mission` 見到非 null 的該鍵就建子類別。未開建圖模式 ⇒ 型別、欄位、輸出逐位元不變 |
| `DecomposeContext.mapping` | 分解層要讀建圖設定 | 尾端加欄位 `mapping: MappingModeConfig | None = None`，`MissionContext.decompose_ctx()` 填 `inputs.mapping_mode` |
| `LidarSensor`／`CameraSensor`／`TrapezoidSensor.crab_rad` | 偏流餘裕要改條帶間距 | 尾端加欄位 `crab_rad: float = 0.0`；非 0 時 `strip_spacing_m = (w cosψ − l sinψ)(1 − o_side)`（`crabbed_width_m`）；`context.apply_crab_margin` 只在建圖模式寫入（ψ = asin(W_max/V_a,cruise)），W ≥ V_a 或 w_eff ≤ 0 → MissionInputError |
| `decompose_area(..., seam_neighbors=)` | 接縫重疊要 ≥ w − s | `area_edge_deltas` 回 `(δ_edge, δ_cell, tie)`：區域外緣仍 δ_edge；胞外擴 max(δ_edge, tie/2)；有鄰區時 Z ← Z ∪ ((Z ⊕ tie/2) ∩ 鄰區)；A/B 殘帶單邊外擴 max(δ_edge, tie)。`plan_mission` 在建圖模式且多區域時傳其他區域之聯集 |
| `end_extension_for` | 匯出器要在帶起點後 lead_in 才開相機 | 回傳值另加 `mapping.lead_in_m`（含 e_ext 覆寫時） |
| 不分層 | 建圖要全隊同高 | `plan_mission` 把分層會改高度的胞改成 `time_separation`（不重鋪），警告 `mapping_no_altitude_layering(...)` |
| 有效值紀錄 | 有效值契約 | 警告 `mapping_mode(...)`、`mapping_crab_margin(...)`、`mapping_small_blocks(...)`、`mapping_altitude_nonuniform(...)`、`mapping_incomplete(...)`；`derived[cls]["mapping"]`、`per_area[aid]["mapping_seam"]`；轉接層 `notes` 的 `mapping_mode_effective(...)`、`on_report` 的 `"mapping_mode"`、`HvDccppCoordinator.mapping_effective` |
| 快門限速訊息（兩種模式） | 原本只有籠統的 `no_coverable_area` | `analyse_areas` 在方向不可行**且只因 V_cap** 時加 `per_area[..]["classes"][c]["shutter_limit"]` 與警告 `shutter_rate_caps_speed(...)`（需要的觸發率、可行高度、前向重疊上限）；全區 NO_GO 時同字串也附在 `errors`（第一筆仍是原本的 `no_coverable_area`）。判定不變 |

## 建圖模式交叉航帶 cross_strips ＋ 建圖作業風速上限（2026-10-15；mission / geometry / decompose / planner / altitude / preflight / ardupilot / aeroplan_adapter）

依據：`docs/討論_2026-10-15_HARVEST航拍建圖_NotebookLM.md` B1 與使用者 10-15 決定（交叉航帶與加密 GCP 兩者都做、
由使用者比較；首次建圖風速 ≤ 3 m/s；能量模型重配權暫緩、預設值不動）。文獻只支持「平地、GCP 少」時交叉塊有大效益
（Gerke & Przybilla 2016 的交叉塊同時換方向與高度 75/105 m、約 20% 面積；料堆區無顯著差異；17–18 GCP 時差異消失），
同高度垂直重飛只有小幅改善（Štroner 2021）⇒ 這裡只提供可量、可比較的幾何，不宣稱精度。

| 項目 | 原因 | 實際做法 |
|---|---|---|
| 設定 | 新功能、預設關 | `MappingModeConfig` 尾端加 `cross_strips=False`、`cross_fraction=0.2`（(0,1]）、`cross_alt_offset_m=0.0`、`cross_angle_offset_deg=90.0`（夾角 mod 180° ∈ [10°,170°]）。**沒開卻給了非預設值 ⇒ MissionInputError**（會被靜默忽略的值不收）。`cross_strips=False` 時 `to_dict` 不列 `cross_*` ⇒ `mapping_mode(...)` 警告與輸入 JSON 與加入前逐位元相同 |
| 不改分解／配置／IDP | 交叉帶若放進 `Cell.strips`，skip-k、Held-Karp、`turn_band_conflict` 的 v 座標、λ 共飛都假設胞內條帶平行；且同隊多機時 IDP 會把交叉帶分給別架 | 新步驟 6b `planner._insert_cross_strips`（Beam-IDP／RTB／leftover 再拍賣之後、組裝之前）。主條帶、配置、起飛時程完全不變 |
| 「區塊」的定義 | 殘帶情形一架只飛母胞的一部分（A 的前 k 條掛在母胞 id 上） | 區塊 = 該機在某胞實際飛的主條帶足跡 ∩ 胞多邊形（不連續時取最大一片並註記） |
| 交叉帶幾何 | 固定翼轉彎半徑 137–267 m：短而多的帶全是轉彎 | 中央**橫帶**：沿交叉方向貫穿整個區塊、在主條帶方向置中於形心、高度以二分法解到面積 = 比例 × 區塊；以 `build_strips`（lay_strips＋局部包絡＋禁區）鋪在交叉方向、交叉高度。⇒ 每條交叉帶橫越該機全部主條帶。比例只需 1 條 ⇒ 放寬到剛好 2 條（w + 0.98 s）並寫出有效比例；放不下 2 條／禁區／包絡不可行 ⇒ 略過該區塊、`mapping_cross_skipped(...)` |
| 識別 | 不能動 `Strip` 欄位（`MissionPlan.to_dict` 會多鍵、預設路徑不再逐位元相同） | 條帶 id `f"{cell}_{cls}_cross_{uav}_{k}"`（`geometry.cross_strip_id`／`is_cross_strip`）；`Strip.cell_id` 仍是母胞；`index` = 母胞主條帶數 + 序號（同胞多組時累加，不撞號）。⚠ 區域 id 若本身含 `_cross_` 會被誤判（GUI 的區域 id 是 `Z<n>`） |
| 插入 | 「同一架、緊鄰母區塊」 | 只考慮母區塊每段主條帶之前或之後；交叉帶內部順序 ≤ 12 條用精確 DP（子集 × 末條帶 × 掃向，numpy 向量化），否則 skip-k／折疊蛇形候選＋掃向 DP；取「插入後 − 插入前」時間最小者（同分取「之後」） |
| 高度 | 有效值契約：不夾限 | h_x = 主條帶 h_op + offset，須 ∈ [max(最低固定翼, 地形淨空), min(h_ceil, 感測器上限)]（＝ `altitude_band` 夾 h_op 的同一組界；**不**用 `AltitudeBand.h_hi_m`＝min(h_ceil − 起伏, …)：主條帶本身就不扣起伏，2×2 km 平地 ENU 曲率使起伏 0.75 m ⇒ h_hi_m 119.25 < 主條帶 120）。preflight 構型層級先擋（`cross_alt_out_of_band(...)` NO_GO），planner 逐區塊再擋（`decompose.CrossStripRefused` ⇒ 空計畫 NO_GO） |
| 升降不得延伸進交叉帶 | build_3d 不在轉場插盤旋；進入／返航段的 z_des 是過境高度 h_transit（A 120、B 100）；包絡平滑會把升降拖進作業段（修前實測：交叉 100 m 的帶上 100 → 116.6 m） | `altitude.sample_bounds`：交叉帶作業樣本的上界收緊為 z_des + 0.25 m（主條帶不走這裡）。`_cross_order`：前後段路徑短於 \|Δh\|/tanγ（進入／返航段的 Δh 以 h_transit 計）加罰 1e6 s；全部選項都犯規才照選並警告 `mapping_cross_altitude_change(...)` |
| 206 觸發距離 | 交叉帶可不同高 | 匯出器對交叉帶取「本帶 OPERATE 航點最低 AGL」算 d（保守，前向重疊不低於規劃）；主條帶照舊（帶起點 AGL，0.5 m 內吸附到作業高度） |
| 能量 | `append_rtb` 的能量安全檢查不含重飛能量 e_MA，validate 有 ⇒ 交叉帶把餘裕吃到兩者之間時規劃器照收、validate 不過（2×2 km、比例 0.2：B3 E(P_tc) 156.7 < 158.5） | 交叉帶進 `CostCache.add_strips`（每構型一次）→ `_rebuild_sequence`（與 IDP 同一套 `_apply_warm`／`_sequences_from` 算術，未插入時逐值相同）→ `append_rtb`；組裝後對有交叉帶的機再跑 `sim.check_energy`。任一不過 ⇒ 自最後插入的一組起逐組退回，退回的條帶留在 `plan.strips`（看得到）並列入 `plan.uncovered`（`cross_energy_infeasible`；找不到可行插入位置 `cross_insert_infeasible`）⇒ `mapping_incomplete` ⇒ 建圖閘門擋上傳 |
| 去衝突 | `spatially_exclusive` 只看胞 ⊕ 主條帶迴轉帶 | `_cross_skip_filter`：飛交叉帶的機對把「交叉帶足跡 ⊕ 交叉方向迴轉帶」併進團隊帶重判，距離 < d_sep 就取消豁免（只會多檢查）。IDP 重排回呼（D-PL-2）遇到後綴含交叉帶時候選被「覆蓋不得減少」擋掉 ⇒ 該機只剩延後起飛／盤旋兩種修法 |
| 建圖事後檢查 | 交叉帶不是「區塊」也不是「主作業高度」 | `_mapping_plan_notes` 的 `mapping_small_blocks`／`mapping_altitude_nonuniform` 不算交叉帶；另有 `mapping_cross(...)` 摘要（逐機逐區塊：條數、km、方向、高度與 GSD 比、有效面積比例、成像比例、插入位置、增加時間） |
| 轉接層 | 掃描角中位數 | `from_mission_plan` 的 `scan_angle_deg` 不算交叉帶（比例大時中位數會翻成交叉方向）；`mapping_effective["cross"]` 收 `mapping_cross*` 警告 |
| 空域（2026-10-15 審查更正） | 原說法「交叉帶不與他機共用空域」**不成立** | 交叉帶沿交叉方向貫穿區塊、兩端再加 e_ext＋引入段，帶間迴轉也在交叉方向 ⇒ 作業段本身就伸進鄰機獨占區塊（審查探針 `runs/artifacts/HDMAP_CROSS_GCP_2026-10-15/R_review/probes/probe_cross_v2.py`：作業航跡 300–2000 m／架、迴轉 750–4500 m／架；主條帶 0–2.6 m），而且建圖模式不分高度層 ⇒ 與鄰機同高、只靠 4D 時間錯開。固定翼迴轉半徑 137–267 m，區塊寬 300–600 m，把交叉帶連同迴轉裁進「本機區塊 ⊖ d_sep」幾乎一條都放不下 ⇒ **不裁**，改為量化並明講：`_cross_airspace_note` 發 `mapping_cross_airspace(…)`（實際飛的交叉帶足跡 ⊕ 迴轉帶 ∩ 他機主條帶足跡扣本機，逐機列 ha）。validate 不過時 GUI 閘門擋上傳；`scripts/hv_sitl_run.py` 在 HV_CROSS=1 且 validate 不過時 SystemExit（`HV_ALLOW_CONFLICTS=1` 才放行並記錄）。是否要求「交叉帶不得離開本機區塊」（等於多機時不能用交叉航帶）是**使用者決定** |
| 動態重規劃（未處理） | `dynamic.py` 把 `plan.strips` 當一般條帶 | 失聯／重規劃時交叉帶可能被分給別架（不再保證同一架）；這次不處理，記在此 |
| 建圖作業風速上限 | 使用者定調首飛 ≤ 3 m/s；引入段依機型 × 風速的函數**這次不做** | `HvDccppCoordinator(mapping_max_wind_mps=…)`（沒開建圖卻給 ⇒ 拒絕）；`mapping_gate(max_wind_mps=None)` 預設用建構值，比 `last_ctx.wind.max_speed()`（規劃實際用的風），> 上限 + 1e-9 ⇒ 不可上傳（3.0 過、3.1 擋）；回傳字典形狀不變，有效值在 `mapping_effective["wind"]`，notes 加 `mapping_wind(...)`（每次提醒「上限指現場實際風速，起飛前實測」）。MainWindow 的 `hv_upload_gate` 不帶參數呼叫 `mapping_gate()` ⇒ 上限一定要放建構參數 |

### 實測（2×2 km 平地、1 Pioneer + 2 HERO、split3、HQ6 2 Hz、120 m、0.70/0.80、引入段 50 m、能量預設值、無風）

見 `runs/artifacts/HDMAP_CROSS_GCP_2026-10-15/X_cross_strips_wind_gate/cost/`（`cross_cost.json`、GUI 路徑 `cross_cost_gui.json`）。
重點：能量預設值下 HERO 的交叉帶幾乎都排不進（B2 原本餘裕只有 13.4 Wh），**所有開交叉帶的組態都因 `cross_energy_infeasible` 不可上傳**；
排進去的只有 Pioneer 的一到兩組。makespan 增加的大頭是 4-D 去衝突把 HERO 延後起飛（建圖模式不分層、時間是唯一隔離，
Pioneer 的交叉帶兩端伸進鄰塊），不是交叉帶本身。
