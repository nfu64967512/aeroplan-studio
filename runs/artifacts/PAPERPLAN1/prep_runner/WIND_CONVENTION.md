# SIM_WIND_DIR 的方向約定 — 從本 repo 自己的 ArduPilot 原始碼驗證

寫於 2026-09-21。**沒有起任何 SITL / ArduPilot / MAVProxy 行程**；下面每一條都是
讀本 repo 內既有檔案得到的。

## 結論（一句話）

`SIM_WIND_DIR` 是氣象「**來向**」（true deg）。`SIM_WIND_DIR = 270` ⇒ 風從西邊來、
往東吹 ⇒ **向東的航段地速比向西的航段高**，差值約 2W。

## 證據鏈（四段，每一段都有檔案與行號）

### 1. 參數本身的定義：來向

`runs/artifacts/AIRFRAME_PARAMS/sim_remap_design/approachA/orig/libraries/SITL/SITL.cpp:89-94`
（本 repo 早先為 SIM_SRV_REMAP 工作留下的 ArduPilot 原始碼副本；
`runs/artifacts/AIRFRAME_SIM_MODEL/s1b/review2/tree_3deee9c/SITL.cpp:94` 是另一份，內容相同）

```
    // @Param: WIND_DIR
    // @DisplayName: Direction simulated wind is coming from
    // @Description: Allows you to set wind direction (true deg) in sim
    // @Units: deg
    AP_GROUPINFO("WIND_DIR",      10, SIM,  wind_direction,  180),
```

預設 180，與 `sitl/models/plane.parm:1296` 的 `SIM_WIND_DIR,180` 相符。

### 2. 風向量怎麼組出來：NED + 一個負號

`runs/artifacts/AIRFRAME_SIM_MODEL/s1b/review2/tree_3deee9c/SIM_Aircraft.cpp:888-916`

```
void Aircraft::update_wind(const struct sitl_input &input)
{
    // wind vector in earth frame
    wind_ef = Vector3f(cosf(radians(input.wind.direction))*cosf(radians(input.wind.dir_z)),
                       sinf(radians(input.wind.direction))*cosf(radians(input.wind.dir_z)),
                       sinf(radians(input.wind.dir_z))) * input.wind.speed;
    ...
    // the AHRS wants wind with opposite sense
    wind_ef = -wind_ef;
}
```

地球座標是 NED（x = 北、y = 東）。代 `direction = 270`：

| 步驟 | x (北) | y (東) |
|---|---|---|
| `Vector3f(cos 270, sin 270, ·) * W` | 0 | −W |
| `wind_ef = -wind_ef` | 0 | **+W** |

⇒ `wind_ef` 是**空氣團自己的速度向量**（風往哪裡去），270° 來向 ⇒ 指向正東。

### 3. 地速怎麼被風改掉

`SIM_Aircraft.cpp:763`（同檔，`update_position()` 內）

```
    // velocity relative to air mass, in earth frame
    velocity_air_ef = velocity_ef - wind_ef;
```

移項：`velocity_ef = velocity_air_ef + wind_ef`。
自我檢核：一架相對地面靜止（`velocity_ef = 0`）的飛機，在往東吹的風裡應該相對空氣
以 W 往西移動 —— 代入得 `velocity_air_ef = 0 − (0, +W, 0) = (0, −W, 0)` ✓ 正確。

所以 `SIM_WIND_DIR = 270`（來向西）時：

* 機頭向**東**飛：`groundspeed = airspeed + W`（順風，**比較快**）
* 機頭向**西**飛：`groundspeed = airspeed − W`（逆風，**比較慢**）

### 4. 飛控回報的 WIND 訊息是同一個約定

`C:/Users/joy46/AppData/Roaming/Python/Python314/site-packages/pymavlink/message_definitions/v1.0/ardupilotmega.xml:1517-1522`
（本專案 `hv_dccpp/mavlink_link.py` 用的就是這份 dialect）

```
    <message id="168" name="WIND">
      <description>Wind estimation.</description>
      <field type="float" name="direction" units="deg">Wind direction (that wind is coming from).</field>
```

`hv_dccpp/mavlink_link.py:1773-1774` 把它讀成 `wind_speed_mps` / `wind_dir_from_deg`，
C2 補丁再把這兩欄寫進 `flight.csv` 的 `wind_spd_est` / `wind_dir_from_est`。
⇒ **來向**，與 `SIM_WIND_DIR` 同一個約定，也與規劃層 `Scenario.wind = (speed, dir_from_deg)`
（`hv_dccpp/integration/aeroplan_adapter.py:963` `WindLayer(..., dir_from, ...)`）同一個約定。
三邊一致，不需要轉換。

### 5. 參數名在這版韌體裡確實存在（不會被靜默略過）

`sitl/default_params/firmware_param_names.json` 的 `_provenance` 說明這份清單是
「以 MAVLink PARAM_REQUEST_LIST 從實際開機的 SITL 讀出完整參數名清單；每份都已驗證
收到數量 == param_count 且無缺號」，韌體是 `ArduPlane V4.8.0-dev (sitl/ArduPlane.exe)`。
其中 `plane`/`vtol` 兩組都列有：
`SIM_WIND_DIR`(1314) `SIM_WIND_DIR_Z`(1315) `SIM_WIND_SPD`(1316) `SIM_WIND_T`(1317)
`SIM_WIND_TC`(1318) `SIM_WIND_TURB`(1319) `SIM_WIND_T_ALT`(1320) `SIM_WIND_T_COEF`(1321)
以及 `MIN_GROUNDSPEED`(1996)。

⇒ C2 補丁的風場讀回閘門 `lk.get_param("SIM_WIND_SPD")` 等五個名字**都讀得到**，
不會踩「ArduPilot 對不認識的參數名靜默略過、讀回 None」那條陷阱。
`MIN_GROUNDSPEED` 在 `sitl/models/plane.parm:613` 是 `0`，且 `--defaults` 鏈上沒有任何
檔案設它（見 `verify_patch.json.sim_wind_on_chain`）⇒ 閘門的 `MIN_GROUNDSPEED == 0`
斷言預期會通過。

---

## 驗收判準（**方向性**的，不是只看大小）

C2 的 RUNBOOK 原本寫「east-minus-west of (gs − as) = +10.0 m/s ± 1.0」。
那是個**絕對值**式的寫法：把 SIM_WIND_DIR 設成 90 也會得到 10 m/s 的東西向差，
只是**符號相反**，而符號正是「風有沒有照我們以為的方向吹」的唯一證據。
所以判準改寫成有號的：

> **A2（有號）** 在 5 m/s / 270° 的架次裡，對每一架飛機取巡航段（固定翼、AGL > 100 m）
> 的 `groundspeed − airspeed` 中位數，依航跡方位角分成向東（bearing 45°–135°）與
> 向西（225°–315°）兩組，則必須
>
> &nbsp;&nbsp;&nbsp;&nbsp;`median_east(gs − as)  >  median_west(gs − as)`
>
> 且 `median_east − median_west = +2W = +10.0 ± 1.0 m/s`。
> 逐項預期（含既有的 +0.44/+0.38 m/s 密度偏置，它與航向無關、兩邊都加）：
> 向東約 **+5.4 m/s**、向西約 **−4.6 m/s**。
>
> **若差值是 −10 m/s（東比西慢）**：風真的有作用，但方向與我們以為的相反 ⇒
> **不要**改 `SIM_WIND_DIR` 了事，先回頭看上面第 2 節的推導哪一步錯了，
> 並把結果寫進本檔；規劃層的 `dir_from_deg` 也用同一個約定，兩邊會一起錯。
>
> **若差值約 0**：風根本沒生效（或 `MIN_GROUNDSPEED > 0` 讓 TECS 自己補了空速）⇒
> 該格作廢，查 `wind_readback.json`。

> **A3（獨立第二讀數）** `flight.csv` 的 `wind_dir_from_est` 在固定翼巡航穩定後
> 應收斂到 **270 ± 20°**、`wind_spd_est` 收斂到 **5 ± 1 m/s**。
> 這一條與 A2 互為獨立證據：A2 來自 GPS 地速與空速管，A3 來自 EKF 的風估計。
> **兩者都指向同一個方向才算數**；只有一個對的話，是儀器問題不是風的問題。

> **A4（180° 那一格）** `W55S` 是 `SIM_WIND_DIR = 180`（南風，往北吹）。
> 掃描條帶是東西向 ⇒ 這一格的東西向 `gs − as` 差值應**回到 0 ± 1 m/s**，
> 改為在**南北向**的轉場腿上看到差值，並且飛機必須出現蟹行角（crab）。
> 這一格存在的理由就是：270° 只測到風三角形的沿航跡分量，180° 才測到橫風分量。

## 一個相關但**沒有**在這裡解決的問題

`SIM_WIND_T`（風隨高度的剖面）本 repo 的兩份 SITL.cpp 副本都沒有含到那一段的
metadata（只到 `WIND_TURB`）。C2 補丁把它設成 `1` 並註記為「none ⇒ 風不隨高度衰減」。
這個值**沒有在本檔驗證**；它只影響低空（`SIM_WIND_T_ALT` 預設 60 m）的風強度，
而本任務的巡航在 119.5 m AGL，兩種剖面在該高度都是全風速。標成推論，不是量到的。
