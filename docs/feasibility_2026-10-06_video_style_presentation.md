# AeroPlan 影片式任務呈現：可行性評估（最終裁定）

> **範圍**：參考影片是一部期刊補充影片（103 s），本文把其中 15 個視覺元素 E1–E15 拿來對照 `quad_swarm` 分支**目前的工作樹**。
> **判定依據**：讀碼加上多位驗證者交叉驗證。MainWindow 在自動化環境裡建不起來，所以凡是 GUI 行為，都只追過程式碼，沒有自動化實機確認。少數爭議點由我另外查過原始碼與打包的 `Cesium.js` 後再裁定。

---

## 1. 一句話結論

**可以做到，可行性高。** 最小必要範圍約 **14–18 人日**，做完後 AeroPlan 內能完整呈現 8 項、近似呈現 5 項。另外三個選做階段約 +8–11 天，做了以後 E11 也能近似。E13 只能做到單一酬載小窗（近似），每機一個獨立視窗不建議做。

**最關鍵的理由**：影片裡最難的是「多架機依時間同步播放」，而這件事需要的資料已經在記憶體裡了。

- HARVEST 每個航點都帶 `t_s / heading_deg / agl_m / sensor_on / phase`（`hv_dccpp/mission.py:1307-1334`）。
- 規劃完成後，整份計畫一直留在 `self._hv_coordinator.last_plan`（`hv_dccpp/integration/aeroplan_adapter.py:1925,1930`；`ui/main_window.py:5539,6152`）。
- Cesium 頁面也已經有 GLB 模型、拖尾、角錐光錐、半球、flyTo 和時間軸元件。

真正缺的有三樣：

1. 把 `t_s` 帶出來的匯出器。目前時間戳在 `aeroplan_adapter.py:1393-1408` 被丟掉了。
2. 一個由 Cesium 時鐘驅動的播放圖層。
3. GUI 接線。

另外要先修掉三個一上鏡頭就會穿幫的既有錯誤：

- 2D→3D 切換時，3D 航線會被清空。
- 同一架 UAV 在不同地方用了至少五套色盤。
- 光錐的半透明填色被做了兩次座標變換。

**限制**：播放出來的是「規劃時間軸」，不是飛行模擬。要當論文證據，請用 8 趟 SITL 飛行日誌回放，這些資料本機都有。

---

## 1.5 實測補充（主 session，2026-10-06，非讀碼推論）

上面的對照是讀碼加交叉驗證得來的。以下幾點是我另外**實際執行**量到的結果。
證據放在 `runs/artifacts/PLAYBACK_FEASIBILITY/`（該目錄被 `.gitignore:109` 忽略，只存在本機）。

| # | 實測內容 | 結果 | 證據 |
|---|---|---|---|
| T1 | AeroPlan 產生的 3D 頁（`data/tmp_maps/cesium_0001.html`）能否**脫離 MainWindow** 渲染 | ✅ 可以。做法：本機 http 伺服器把 `file:///…/aeroplan-studio/` 改寫成 `/`，再用 Playwright `channel="chrome"` 無頭開頁。GPU 走 ANGLE D3D11（RTX 4070），等 `globe.tilesLoaded` 之後截圖 | `cesium_preview_server.py`、`shoot.py` |
| T2 | 不寫任何新繪圖碼，只呼叫既有 JS 函式（`setCorners`、`addPath`、`addNFZCircle`、`addRadarDome`、`updateUAVPosition`、`_appendTrail`、`updateFOVCone`），能畫出多少影片元素 | 能畫出作業區、每機彩色航線、禁航柱、半球罩、3 架 GLB 加標籤、拖尾、光錐**線框**、圖例。也就是 E2、E3、E7、E8 的 3D 外觀，以及 E9 的一半 | `smoke.js`、`smoke_existing_functions.png` |
| T3 | E9 光錐半透明填色是否真的被雙重變換（原列為風險 #1） | ✅ **確認是缺陷**。3 架 UAV 的填色 primitive 的 `_boundingSphereWC` 中心離 UAV 約 **764,921 km**（離地心約 767,637 km），從第二次更新起就飛到太空。畫面上只看得到線框 | 頁面上量測 `fovFillPrimitives[1..3]` |
| T4 | 一筆壞座標會怎樣 | ⚠ **整個 3D 視圖靜默凍結**。我一開始把 `addPath` 的點傳成陣列而不是 `{lat,lon,alt}`，產生 NaN，幾何 worker 在 `projectTo2D` 丟出例外。Cesium 因此把 `useDefaultRenderLoop` 設成 false，而 `main.html:172-174` 已把 `_showRenderLoopErrors` 關掉，所以**沒有任何錯誤訊息**，相機與實體都不再更新。只要 planner 輸出含 NaN，就會在 App 內重現這個情況 | 已用 `scene.renderError` 監聽器抓到堆疊 |
| T5 | 瀏覽器窗格或視窗被隱藏時能否截圖 | ❌ 拿到的是舊影格或全黑畫布，和 `grabCanvas` 註解記載的一致。錄影一定要走無頭 harness（P5） | — |
| T6 | ion 地形守衛 | ✅ 確認失效。產生的頁面第 189 行是 `defaultAccessToken !== ''`，而 token 也是 `''`。另外 Google 圖磚與 ion 都需要網路 | `data/tmp_maps/cesium_0001.html:115,189` |
| T7 | HARVEST 時間戳 | ✅ 確認。`hv_dccpp/mission.py:1308-1334` 的 `Waypoint` 有 `t_s`、`heading_deg`、`sensor_on`、`phase`；`aeroplan_adapter.py:1393-1404` 輸出時只帶走 `sensor_on`，`t_s` 被丟掉；`last_plan` 在 `:1930` 一直保存在記憶體裡 | — |
| T8 | 2D→3D 切換會清掉群飛航線 | 程式碼層面確認：`cesium_map_widget.py:588-617` 的 `sync_state` 會存 `_swarm_data`，但 `_refresh_paths` 先執行 `clearPaths()`，之後只重畫舊式 `self.paths`。**還沒在桌面實際操作驗證** | — |

**對路線圖的影響**

- 第 7 節風險 #9「GUI 無法自動化驗收」大幅降低：同一份 `main.html` 已經證實可以在 harness 裡跑。P1 的 harness 可以直接用 `channel="chrome"` 吃 GPU，不一定要用 swiftshader。
- P2 要多一項：NaN／壞輸入防護。做法是在 JS 端驗證座標，並在 `scene.renderError` 時自動恢復且回報 Python，避免靜默凍結。
- 第 7 節風險 #1（光錐雙重變換）已實測確認，P2 修正時要把 T3 當突變對照：舊版距離約 7.6e5 km，修好後必須 ≤ agl/2+5 m。

---

## 2. 影片元素對照表

狀態欄以驗證者的判定為準；讀者與驗證者意見不同的地方都有標註。

| ID | 影片元素 | AeroPlan 現況 | 證據（file:line） | 缺口 | 工作量 |
|---|---|---|---|---|---|
| E1 | 2D 衛星底圖加 hillshade、「search area-n」標籤、游標經緯高、北箭頭、量測工具、任務外框 | **partial** | `ui/widgets/map_widget.py:212-235`：Google 衛星、MousePosition（只有經緯度）、MeasureControl、比例尺<br>`map_widget.py:1353-1387`：已提交區域只用 JS 注入<br>`map_widget.py:2113-2118`：規劃一開始就清掉這些區域<br>`ui/main_window.py:5815-5817`：`areas` 只放 `corners_snapshot`<br>`ui/widgets/dual_map_widget.py:615-618`：自動 geofence 矩形，約等於任務外框 | 游標沒有高程、沒有北箭頭、2D 沒有 DEM hillshade；多區規劃後區域外框和標籤會消失；量測與游標外掛走 CDN，離線時直接失效 | M（區域標籤與北箭頭 S；高程與 hillshade M） |
| E2 | 2D 紅色禁航圓，中心有圖示 | **partial** | `map_widget.py:358-379`：舊 `nfz_zones` 畫紅圓，沒有中心圖示<br>`main_window.py:844-847,3117-3133`、`dual_map_widget.py:1121-1130`：FenceZoneRegistry 只送到 3D<br>`main_window.py:5912-5920,4800-4808`：HARVEST 與 DCCPP 讀的是 registry | **2D 顯示的禁航區，和規劃器實際避開的不是同一份**，這會影響論文圖的真實性；缺中心圖示 | S |
| E3 | 每架機的往返條帶、一機一色、右上角圖例、轉移段 | **live（2D）／3D 脆弱**。cesium-3d 讀者原判 partial，被驗證者推翻 | `main_window.py:5852`（唯一呼叫點）→ `dual_map_widget.py:687-689` → `map_widget.py:1778-1952`、`410-502`（重載後會重畫）、`1954-2020`（圖例）<br>`ui/widgets/cesium_map_widget.py:851-928`<br>`ui/widgets/cesium_templates/main.html:383-396`：3D 圖例一段一列；`runs/artifacts/aeroplan_flow/shots/04_dccpp.png` 可見重複列 | 3D 版在 2D→3D 切換時被清除（見 E15）<br>至少五套色盤：`map_widget.py:1435-1444`、`tactical_theme.py:81-90`、`cesium_map_widget.py:37-42`、`vtol_mission_exporter.py:361-369`、`main.html:1395`<br>HARVEST 有拍攝的航段多半標成 TRANSFER，被畫成虛線（`main_window.py:8289-8295`）<br>圖例在右下，名稱是「Drone n」 | S |
| E4 | 每個航點的黑底黃框編號 | **partial**。map-2d 讀者判 missing；驗證者指出 3D 已有 WP 標籤，改判 partial | `main.html:95`：航點標記圖層預設關閉<br>`main.html:354-380`：每次 addPath 都從 WP0 重新編號<br>`main.html:543-586`：飛控回讀時編 1..N，只在 3D、只有單機<br>`map_widget.py:1804-1872`：2D 群飛沒有航點標記<br>`aeroplan_adapter.py:1955`：`upload_tuples` 才是真正上傳的序號 | 2D 完全沒有；3D 編號每段重來，對不上飛控 seq，樣式也不同 | S（3D）／M（含 2D） |
| E5 | 「場景同步完成」與「規劃完成，是否開始模擬」兩個對話框 | **partial** | `main_window.py:2128-2142`：「任務預覽」只是統計框<br>規劃→起 SITL→上傳→一鍵起飛整條手動鏈：`ui/widgets/vtol_recon_panel.py:1037-1044`、`main_window.py:8407-8654,7831-8125`、`ui/widgets/sitl_hud.py:1279` | 規劃後沒有確認對話框，沒有自動串接，也沒有同步檢查 | S（接到播放）／M（自動串 SITL） |
| E6 | 從星空飛入作業區 | **partial** | `main.html:131`：`skyBox:false`<br>`main.html:1350-1382`：flyToScene／flyToPos<br>`main.html:2572-2579`：初始視角 2000 m，duration 0<br>`assets/cesium/Build/Cesium/Assets/Textures/SkyBox/tycho2t3_80_*.jpg`：已打包但沒用 | 要打開 skybox，加一段兩階段 flyTo；在 2D 模式按「飛向」只會切換、不會飛（`dual_map_widget.py:346-351`） | S |
| E7 | 3D 地形上的貼地區域、紫色半球禁航區、地面車輛模型 | **partial** | `main.html:3600-3686`：addRadarDome 是真的半球，但顏色寫死紅色，而且只能從戰術模組的雷達模擬觸發（`main_window.py:8911-8925`）<br>`main.html:1332-1347`：禁航圓畫成 0–300 m 圓柱<br>`dual_map_widget.py:778-787`：3D 沒有 `display_survey_areas`<br>ion 地形走不通：`main.html:188-195`、`cesium_map_widget.py:158,287`<br>`main_window.py:6159-6196`：DEM 是唯一的地形來源，每邊約 80 點<br>`aeroplan_adapter.py:1062`：`obstacles=()` | 半球沒接到 NFZ；已提交區域在 3D 不畫；預設地形是平的；資料模型裡沒有「地面物件」 | M（其中半球接 NFZ 只要 S） |
| E8 | 每架機的 3D 模型、紅旗標籤、同色拖尾 | **live（僅 SITL/HIL 遙測）／規劃或日誌回放 partial**。讀者 2:3 分歧，差別只在範圍，所以拆成兩部分判定 | `main_window.py:6501,6770-6795` → `dual_map_widget.py:903-930` → `main.html:1825-1986`（GLB 與標籤）、`1659-1688`（拖尾）<br>`runs/artifacts/aeroplan_flow/shots/06_telemetry.png` | 只有 SITL 在跑時才會動<br>拖尾上限 **1500 點**（`main.html:1664`；同處註解寫 500，是錯的）<br>拖尾顏色（`main.html:1395`）與圖例不同<br>標籤是 `✈ UAV1 [MODE] ●ARM`，不是旗標<br>2D 模式期間 3D 收不到點，拖尾會缺一段 | 外觀 S；回放引擎 M（工作量記在 E10） |
| E9 | 青色半透明光錐掃過地面 | **partial**。多數驗證者判 live；**本裁定採 cesium-3d 驗證者的意見**：線框是 live，影片要的半透明填色失效。**已實測確認（§1.5 T3）** | `main.html:2821` 把 ENU 矩陣烘進 GeometryInstance，`main.html:2855` 之後每次更新又設一次 `primitive.modelMatrix`<br>打包的 `Cesium.js` 中 `OXe(e,t,n){let i=!n…}`：在非 scene3DOnly 時會先把幾何轉到世界座標，而 `main.html` 沒有設 `scene3DOnly`，所以從第二次更新起被轉兩次<br>`main.html:3028-3230`：updateFOVCone<br>`main_window.py:8743-8790,8946-8990`：勾選框→每幀推送 | 要修填色<br>光錐沒有受 `sensor_on` 控制<br>地面交點固定在橢球高 0（`main.html:2995`），也不用 pitch/roll<br>顏色依 sysid<br>沒有回放驅動 | 修填色 S；接回放 S；與地形求交 M |
| E10 | 左上倍速標記，播放中可變速 | **partial**。gui-wiring 判 missing；因為已有可用的時間軸元件和孤兒時鐘引擎，裁定 partial | `main.html:125-126`：timeline／animation 開著但沒人驅動<br>`main.html:858-863`：時鐘停住<br>`main.html:4002-4288` 與 `cesium_map_widget.py:1392-1394`：孤兒 strikeExecuteAnimation<br>`mission/sitl_launcher.py:700-705`：`start_multi` 沒有 speedup 參數<br>`cli/sitl_cmd.py:35,85,92-101`：CLI 的 `--speedup` 只印出來、沒傳下去 | 目前沒有任何東西可以加速；需要先有回放引擎 | 在回放引擎上只要 S；引擎本身 M |
| E11 | 任務中出現新區域→重新規劃→接著播 | **orphan（核心演算法）／GUI 與播放 missing** | `hv_dccpp/dynamic.py:86-89`、`1441-1527`、`2180-2187`、`2193-2235`<br>`tests/hv_dccpp/test_dynamic.py:191-208`<br>ui／mission／cli 都沒有呼叫者 | 語意是「新條帶接在各機隊尾，每次 reassign 最多改 2 架」，不是影片那種立即改派<br>replan 會就地改 ctx 與 cache<br>新區必須命名 `Z{aid}`（`aeroplan_adapter.py:929-930,1288-1294`） | L（規劃時間軸版）／XL（含空中重新上傳） |
| E12 | 3D 主畫面左下角的 2D 子母畫面 | **missing**。sensor-render 判 partial，理由是 2D 仍收遙測；但完全沒有 inset 容器 | `dual_map_widget.py:225-231,277-307`：QStackedWidget，一次只看得到一個 | 需要 inset；左下角已被 animation 元件與 `#sitlStatus` 佔用（`main.html:24-29,126`） | M |
| E13 | 每架機一個獨立機載相機視窗，含 Distance 讀數 | **partial**。plan-data 判 missing，被驗證者推翻 | `main.html:1541-1608`：setFpvCamera，雲台 −90° 俯視，接管主相機<br>`dual_map_widget.py:214-220,417-503`：FPV 選單 | 一次只能看一架；接管主畫面；只吃遙測；FOV 與光錐參數無關；沒有距離讀數；沒有可被看到的地面目標 | 多視窗 L；單一 PiP M |
| E14 | 跟機／跟機群運鏡、拉近 | **partial** | `main.html:1446-1514`：chase 只能跟單機、只讀 `_UAV_STATE`，俯角預設值從未生效<br>`main.html:824-838`：grabCanvas 在 `:831` 強制設成 `JulianDate.now()`<br>`scripts/_full_flow_capture.py`、`scripts/_encode_frames.py`：未納管，曾產出 `runs/artifacts/aeroplan_flow/aeroplan_flow.mp4`（262 影格，2 Hz 牆鐘） | 沒有機群跟隨、沒有關鍵影格、沒有決定性錄影 | M |
| E15 | 2D 規劃與 3D 模擬共用同一份場景狀態 | **partial** | `dual_map_widget.py:516-760`：雙邊代理<br>`dual_map_widget.py:315-329` 單向同步 → `cesium_map_widget.py:588-617`（`_refresh_all` 不重畫 `_swarm_data`）→ `main.html:261-270`（clearPaths）<br>`dual_map_widget.py:893-898`：`__getattr__` 靜默落到 2D<br>`main_window.py:1560`：clear_corners 只清 2D | **最自然的操作（在 2D 規劃、再切 3D）會讓 3D 航線被清掉。** 參數面板的 DCCPP 全部消失；VTOL 偵察面板另有 `_vtolEntities` 疊層，會保留下來（`main_window.py:8202-8220`）<br>兩個視圖無法同時顯示 | 修清除 S；單一場景狀態 M |

**裁定時順手更正的事實**

- 拖尾上限是 1500 點，不是 500。
- ion 地形這條路有三個斷點：
  1. token 是空字串。
  2. Python 的 `.replace` 會連守衛條件裡的哨兵字串一起換掉，條件變成永遠為 false（`cesium_map_widget.py:287`）。
  3. 1.115 版已經沒有 `createWorldTerrain`。
- `docs/adr/0009-uniform-single-lens-camera.md:83` 只記載 2161→3198 s，撐不起「1400–3200 s」的說法。以 30x 播放約 72–107 s，剛好和影片長度相當。
- `runs/artifacts/HV4SITL16` 缺 `compare.png`、`flight_summary.json` 和 `separation_from_tracks.json`。
- Playwright 1.59 和 PyAV 18.1 實際裝在 `C:\Users\joy46\AppData\Roaming\Python\Python314\site-packages`。
- 「GPS ToW 就是跨機共同時鐘」**沒有被證實**：推算出來各 SITL instance 之間約有 0.2–0.7 s 的偏差。

---

## 3. 已經有、但沒接上的東西

這一節是最便宜的收穫。

| # | 項目 | 位置 | 現況 | 可以拿來做 | 注意 |
|---|---|---|---|---|---|
| 1 | HARVEST 帶時間戳的計畫（`last_plan` / `last_ctx`） | `aeroplan_adapter.py:1829-1832,1925,1930`；`main_window.py:5539,6152` | 只有 `upload_tuples` 在用（`main_window.py:8051-8061`） | E8、E9、E10 回放的資料來源 | 延遲盤旋只存成單一航點，要自己展開成圓（`hv_dccpp/deconflict.py:981-1012`）；fence 繞行會改寫 `assembled_paths`，但不會改 `last_plan`（`main_window.py:5320,5646`） |
| 2 | `strikeExecuteAnimation` | `main.html:4002-4288`；`cesium_map_widget.py:1392-1394` | 沒有呼叫者 | SampledPosition、PathGraphics、clock 的寫法範本 | 只抄寫法，不要直接重用：speedMul 被烘進樣本時間（`:4060-4066`）、時間錨在 `now()`（`:4030`）、沒有 targets 時會丟例外（`:4179`）、模型 scale 0.015 與即時路徑的 1.0 不一致 |
| 3 | `renderTerminalDive` | `main.html:5346-5723`；`cesium_map_widget.py:1588-1639` | 沒有呼叫者 | 用 `trackedEntity` 跟拍的先例（`main.html:5707`） | 會重設全域時鐘 |
| 4 | `hv_dccpp.dynamic`（NewArea / replan / cache_from_plan） | `dynamic.py:86-89,1441-1527,2180-2235` | 只有測試與範例在用 | E11 | 會就地修改 ctx 與 cache |
| 5 | `telemetry_frames_to_events` / `events_from_telemetry` | `aeroplan_adapter.py:1610,1984-1989` | 沒有呼叫者 | 未來把即時遙測轉成事件 | — |
| 6 | `deconflict.trajectory_samples` | `deconflict.py:577-591` | GUI 沒用 | 等時間間隔取樣 | 線性內插；延遲盤旋一樣要先展開 |
| 7 | `sim._footprint_quads_runs`、`track_simulate` 的 trace | `hv_dccpp/sim.py:357-444`、`1275-1392` | 只有測試與範例 | 足跡四邊形、覆蓋區漸進著色 | 回傳值沒有 uid／t_s（`:445`），要先改 |
| 8 | `addRadarDome` 真半球 | `main.html:3600-3686` | live，但只接到雷達模擬 | E2／E7 的半球禁航區 | 顏色寫死紅色 |
| 9 | SkyBox、NaturalEarthII 貼圖 | `assets/cesium/Build/Cesium/Assets/Textures/` | 已打包但沒用 | E6 星空；離線底圖 | — |
| 10 | Cesium Animation／Timeline 元件 | `main.html:125-126` | 畫面上有，但時鐘沒人驅動 | E10 倍速轉盤、拖曳時間軸 | — |
| 11 | `capture_png` / `grabCanvas`、`set_view_position` | `cesium_map_widget.py:428-466`；`main.html:824-838` | 只有未納管的腳本在呼叫，或根本沒人呼叫 | 截圖、運鏡 | grabCanvas 會強制設成 `now()`，也抓不到 HTML 疊層 |
| 12 | 錄影與編碼腳本 | `scripts/_full_flow_capture.py`、`scripts/_encode_frames.py` | 未納管；2026-09-03 曾在使用者桌面建出真的 MainWindow 並產出 mp4（`runs/artifacts/aeroplan_flow/flow.txt`） | 編碼器（PyAV）可以直接重用 | `_full_flow_capture.py` 會啟動 SITL，**agent 不可以執行** |
| 13 | 8 趟論文 SITL 的原始日誌與萃取快取 | `sitl/vtol/i100–i102/logs/00000013–20.BIN`（約 718 MB，未納管也未忽略）；`runs/artifacts/PAPERPLAN1/update_paper/cache/*.npz`（24 檔，已被 gitignore） | 只有論文分析腳本在讀 | 實飛回放，這是真正的論文證據 | 先備份並記錄雜湊 |
| 14 | ExportDialog 與 KML/GPX 匯出 | `ui/dialogs/export_dialog.py:23,625`；`mission/mavlink_exporter.py:147-228` | 只在自我測試裡建構 | 匯出規劃航點 | — |
| 15 | 打擊頁的「動畫速度」 | `ui/widgets/parameter_panel.py:3835-3841,4447` | 值有收集，但沒人讀 | 移除，或接到新的播放功能 | — |

**順帶發現的既有缺陷**

這些不算 orphan，但做呈現時會出問題：

- `check_bullseye_hit` 用到 `math` 卻沒 import，NameError 被吞掉，命中爆炸永遠不會觸發（`cesium_map_widget.py:14-25,483-486,1186-1190`）。
- 2D 視角回報用 `Image()` 載入 `pyqt://`，但 `acceptNavigationRequest` 攔不到這種請求，所以 `_view_state` 可能永遠是空的，重載後回到預設中心、zoom 20（`map_widget.py:83-90,942-954`）。這點**看起來合理，但未實測**。
- chase 相機的俯角預設值從未生效（`main.html:1458-1507`）；戰術頁光錐的 tooltip 寫「與 Pitch/Roll 同步」，在角錐模式下並不成立（`parameter_panel.py:2632-2636`）。
- 載入 DEM 時會開啟 `enableLighting`（`main.html:2507`），而時鐘停在開頁當下的牆鐘時間，所以晚上開啟時地形會偏暗。

---

## 4. 三條路線比較與建議

| | A：AeroPlan 內建規劃回放 | B：獨立場景匯出加播放器 | C：實飛日誌回放優先 |
|---|---|---|---|
| 核心做法 | `last_plan` → CZML → `main.html` 的 sim 圖層加 GUI 播放列 | 純 Python 輸出 bundle（scene.json、CZML、player.html、plan2d.html），用 Playwright 決定性逐幀算圖 | 日誌（npz／BIN／CSV）→ CZML 加 manifest；計算規劃與實飛的偏差；第二階段才做規劃預覽 |
| 影片覆蓋（完整/近似/不做） | 8 / 7 / 0 | 11 / 3 / 1，但「完整」是在獨立網頁裡，**不在 AeroPlan GUI 內** | 7 / 7 / 1（不做 E11） |
| 是否在 AeroPlan 內 | 是 | 主要在瀏覽器；GUI 只有一個開啟用的對話框 | 是，選單上一個入口 |
| 工作量 | 20–32 天 | 11–18 天 | 12–19 天 |
| 會不會修既有 GUI 錯誤 | 會（先修 3D 清除、色盤、grabCanvas） | 不會 | 只保證回放圖層不被清掉 |
| 論文價值 | 中高：規劃時間軸必須加浮水印 | 高：輸出決定性、有內容雜湊，畫出的光錐就是規劃器的足跡 | **最高**：用實飛資料，分隔距離鎖定已發表的 `NUMBERS.json`，並以覆蓋餘裕為門檻為偏差著色 |
| 實飛價值 | 中：起飛前預演，可延伸成「規劃幽靈機」 | 中低：只能事後檢視 | **高**：任何 SITL、HIL、真實 DataFlash 都能事後回放 |
| 最大風險 | 工作量大，GUI 無法自動化驗收 | 兩套播放器會分歧；沒滿足「在 AeroPlan 內」 | 跨機時鐘還沒定；CAM 紀錄未證實；沒涵蓋 plan→simulate 主流程 |

三者其實共用同一個骨架：**Qt-free 匯出器 → CZML → Cesium `CzmlDataSource` 圖層 → Playwright harness 驗收**。差別只在資料來源和宿主程式。

### 建議（組合式）

整體上取 **A 的產品形態、C 的工程手法、B 的錄影手法**：

1. **一份 CZML 契約**，以 `timing ∈ {planned, estimated, flown}` 區分來源，規劃、舊 DCCPP 估算、實飛三種來源都用同一個播放圖層。
2. **播放圖層放在 `main.html` 裡一個獨立的 `CzmlDataSource`**，不會被 `clearPaths` 清掉。同時把 `CesiumMapWidget._load_cesium`（`cesium_map_widget.py:237-310`）的模板組裝抽成一個 Qt-free 函式，讓**同一份 `main.html`** 既能在 AeroPlan 裡跑，也能在 Playwright harness 裡跑。不另寫 B 的 `player.html`。
3. **錄影走 harness**：以固定 dt 推進時鐘、等 `globe.tilesLoaded`，再用 `page.screenshot`，這樣 HTML 疊層也會被拍進去。不依賴 `QWidget.grab` 或 `grabCanvas`。
4. **先做規劃回放，再做實飛回放。** 前者對應使用者要的「規劃→確認→模擬」流程，後者是論文證據。若要先交論文補充影片，可以把 P6 提前到 P3 之後。
5. **E11 延後到選做階段；E13 不做多視窗。**

---

## 5. 建議路線圖

標「（新）」的路徑是提議新增的檔案；其餘都是現有檔案。
**所有測試都要在來源端擋住 I/O**：用 monkeypatch 讓 `serial.Serial`、`mavutil.mavlink_connection`、`subprocess.Popen`、`socket.socket.connect` 一被呼叫就丟例外。

| 階段 | 天數 | 交付物 | 可執行的驗收方式 | 看得到什麼 |
|---|---|---|---|---|
| **P1 第一個畫面** | 1.5–2 | 新增 `mission/playback/tracks.py`：`harvest_tracks` 最小版，延遲盤旋先偵測並標警告<br>新增 `mission/playback/czml.py`：每架 UAV 的位置（LAGRANGE 1 次）、path、label、properties<br>新增 `tests/web/sim_harness.html`：用相對 http 路徑載入本機 Cesium<br>新增 `tools/playback/probe_harness.py`：在 127.0.0.1 起 http.server，用 Playwright 開頁 | 1. `python -m pytest tests/playback/test_tracks.py -q -p no:cacheprovider`，夾具為 `hv_dccpp.scenario.build_mini_example()` → `build_context` → `plan_mission`，全程 headless。檢查：每架時間非遞減；首尾樣本時間等於 `waypoints[0].t_s` 與 `t_land_s`（誤差 ≤1e-6）；航點時刻的位置等於 `ctx.frame.to_wgs84`（誤差 ≤0.01 m）；兩次匯出 sha256 相同；I/O 守衛有效。<br>2. `python tools/playback/probe_harness.py`（chromium 加 `--use-angle=swiftshader --enable-unsafe-swiftshader`）：隨機 10 個時刻，`entity.position` 與 Python 的值相差 ≤0.5 m；console 沒有 error；截圖像素標準差大於門檻，存到 `runs/artifacts/PLAYBACK_P1/` | 在桌面 Chrome 開同一頁，可以看到 UAV 沿條帶移動、有拖尾，Animation 轉盤可以調倍速 |
| **P2 先修會穿幫的錯誤** | 2–2.5 | (a) `CesiumMapWidget.display_swarm_raw` 存下 `_swarm_data`，`_refresh_all` 重畫它<br>(b) 3D 圖例改為一架一列<br>(c) 統一色盤 `uav_color(drone_id, cls)`，2D、3D、即時遙測都用它<br>(d) 補 `clear_corners` 的 3D 代理<br>(e) 2D 改畫 FenceZoneRegistry 的圓，並加中心圖示<br>(f) 修光錐填色的雙重變換<br>(g) 新增 `ui/widgets/cesium_page.py`，提供 Qt-free 的 `build_cesium_html` | 1. 用 `object.__new__(CesiumMapWidget)` 建物件、把 `_js` 換成記錄器，呼叫 `sync_state` 後，最後一個 `clearPaths()` 之後必須有 `addPath(`。這個測試**在目前工作樹上必須失敗**（突變對照）。若 import 本身就會中止，就把重畫邏輯抽成純函式再測。<br>2. 色盤測試：2D 與 3D 對同一架回傳的顏色相同。<br>3. 用 harness 載入真正的 `main.html` 做光錐測試：填色 primitive 的 boundingSphere 中心到 UAV 的距離 ≤ agl/2+5 m，舊版必須失敗。<br>4. 使用者在桌面手動確認一次：在 2D 規劃後切到 3D，航線還在 | 在 AeroPlan 裡，2D 規劃後切 3D 終於看得到航線；光錐有半透明填色 |
| **P3 播放核心** | 3–4 | 展開延遲盤旋；`sensor_on` 與 phase 區間；heading unwrap 後轉 HPR 四元數，並套用 GLB 機頭偏移（`main.html:1396-1430`）；紅旗標籤加引線；拖尾顏色等於圖例；NFZ 改畫紫色半球加中心圖示；區域多邊形加「search area-n」；航點編號取自 `upload_tuples` 的 seq；倍速標記搭配 `clock.multiplier` 1/6/10/30x；開 SkyBox 加飛入；光錐由回放驅動且受 `sensor_on` 控制；sim 圖層改用 `CzmlDataSource` | 1. pytest：用 `insert_delay_orbit(n=2)` 造一份計畫，盤旋窗內每個樣本到圓心距離為 R±1 m，而且不會停住超過 1 s，關掉展開後必須失敗；足跡橫向寬度等於 `swath_m(agl)`（`hv_dccpp/platform.py:702-719`），誤差 ≤1%。<br>2. Playwright：隨機 20 個時刻位置誤差 ≤0.5 m；Python 算出的四元數與 `Cesium.Transforms.headingPitchRollQuaternion` 相差 <1e-9；`simSetSpeed(30)` 後 `multiplier===30` 且標記文字為 `30x`；**呼叫 `clearPaths()` 與 sync_state 的 JS 序列後，sim 實體仍在**；τ=0 時相機高度 >1e7 m，飛入結束後 <5e4 m；`sensorOn` 為 false 時光錐隱藏 | 在 harness 裡看到影片中的 E2、E3、E4（3D）、E6、E8、E9、E10 |
| **P4 GUI 接線** | 2.5–3 | 新增 `ui/controllers/playback_controller.py`（只依賴有 `sim_*` 方法的 map 物件，可用假物件測）<br>DualMapWidget 加 `sim_*` 代理與播放列<br>VTOL 偵察頁加「模擬回放」按鈕<br>E5 兩個對話框；「同步完成」以 JS 回傳的區域、禁航區、UAV、航點數量與 Python 逐項比對為準<br>`divergence_guard`：比對 `last_plan` 與實際上傳的路徑<br>等 4.5 s 安定窗口結束（`dual_map_widget.py:282-293`）才載入<br>CZML 分段送<br>畫面浮水印「PLANNED TIMELINE」 | 1. pytest 用 FakeMap 記錄呼叫順序：`ensure_3d` → 等待 → 多段 `sim_load`，串回來等於 `to_czml` 的輸出；數量不符時顯示差異，而不是「同步完成」；把某個 assembled 航點平移 200 m 後，`divergence_guard` 必須標記該機。<br>2. 使用者在桌面跑 10 項手動清單，截圖存到 `runs/artifacts/PLAYBACK_GUI/`：對話框出現、飛入、全部 UAV 在動、拖尾色等於圖例、光錐只在作業段出現、30x 標記、拖曳時間軸、2D↔3D 來回後仍在、載入 DEM 後 UAV 沒有沉入地形、SITL 遙測與模擬可並存 | 在 AeroPlan 裡完成「規劃→確認→3D 模擬」 |
| **P5 錄影、小地圖、運鏡** | 2.5–3 | `simFollow`：跟單機用 `trackedEntity`，跟機群用隱藏的質心實體<br>`keys.json` 運鏡腳本<br>左下 canvas 向量小地圖（`#sitlStatus` 與 animation 元件讓位）<br>新增 `tools/playback/render_video.py`：固定 dt、等 tilesLoaded、`page.screenshot`，最後交給 `scripts/_encode_frames.py`（**先把這支納入版控**） | 用本機影像跑兩次，每張影格的 sha256 都相同；mp4 影格數正確；標記文字正確；跟拍目標的投影位置離畫面中心 ≤5%；小地圖上的 UAV 點與 Python 投影相差 ≤3 px | 可重現的補充影片，對應 E12（近似）和 E14 |
| **P6 實飛 SITL 回放（論文證據）** | 2–3 | **先**為 24 個 BIN 與 npz 建 sha256 清單並備份<br>新增 `mission/playback/from_flown.py`：npz 的 pos/att 轉成同一份 CZML，標 `timing=flown`；規劃航線讀 `plan/*.waypoints`；兩種時鐘對齊方法並列，殘差寫進 meta；畫面燒入來源說明「ArduPlane SITL ×5」 | pytest，只讀檔案：用 CZML 樣本重算 B2–B3 最近距離，等於 `runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json` 的 `flown_exact_m`（誤差 ≤0.5 m）；把 B3 時鐘人為平移 +2 s，該值必須變動超過 1 m（突變對照）；npz 每個 POS 樣本與 CZML 相差 ≤0.1 m | 8 趟論文實飛的規劃與實飛疊合回放 |
| P7（選做）E11 新增區域 | 4–5 | 暫停取得 `t_now` → 畫新區 → deepcopy ctx → `cache_from_plan` → 在 worker 執行 `dynamic.replan(NewArea(AreaSpec('Z{aid}',…), t_now), {}, …)` → `from_mission_plan` → 更新 coordinator → 用 CZML 的 delete 加新增送出增量 → 跳對話框 → 接著播 | pytest：原 ctx 的指紋不變；變更的 UAV 在 `t<t_now` 的樣本與舊版完全相同；未變更的 UAV 位元組相同；Playwright：替換前後 `t_now` 處位置跳動 <1 m | E11 近似 |
| P8（選做）2D 打磨 | 2.5–4 | 已提交區域加標籤，改在 `_build_map` 繪製以跨重載保留；北箭頭；2D 航點編號 LayerGroup；（選）游標高程與 hillshade | 對 folium 輸出的 HTML 做 pytest：區域、標籤、禁航圓與 DivIcon、北箭頭都在，航點標籤數等於 `upload_tuples` 總數；拿掉區域繪製後必須失敗 | E1、E4 的 2D 部分 |
| P9（選做）單一酬載畫中畫 | 2–3 | 第二個 Viewer 共用 `clockViewModel`，相機朝正下方，FOV 等於 HFOV，加 Distance 讀數 | 酬載 viewer 的時間等於主 viewer；相機位置與 UAV 相差 ≤1 m；相機朝向與當地向下方向的內積 >0.99 | E13 近似（只有一窗） |

**合計**：核心 P1–P6 約 **14–18 人日**；選做 P7–P9 約 **+8.5–12 人日**。

---

## 6. 不建議做或做不到的部分

| 項目 | 原因 | 替代做法 |
|---|---|---|
| E13 每架機一個原生酬載視窗 | 每個 Viewer 各占一個 WebGL context、各有一份圖磚快取，GLB 也要各載一次（`ui/resources/copter.glb` 113 MB、`vtol.glb` 100 MB）。視窗被遮住時 Chromium 會節流 rAF，畫面就停了。**資料模型裡也沒有「被看到的地面目標」**（`aeroplan_adapter.py:1062` 傳的是 `obstacles=()`），影片中「目標出現在畫面裡」只能擺拍，真實性不足 | P9 只做一個 PiP；若要畫面，標成「示意標記」 |
| E11 照影片的語意（立即改派、機群重新分工） | hv 的語意是新條帶接在隊尾、每次 reassign 最多改 2 架；要改這個語意屬於規劃器研究，不是呈現層的事。空中重新上傳到 SITL／飛控是 XL，而且有安全疑慮 | P7 只做「規劃時間軸上的重規劃」，UI 文字誠實說明語意 |
| E7 真實山地加車輛模型 | ion 地形三處斷點；Google 圖磚有授權問題；8 趟 SITL 是平地（home 100 m MSL、`TERRAIN_ENABLE 0`），套上真實地形反而會讓航跡埋進山裡 | 只用 DEM（`max_size` 調高），地面物件做成選用的 markers 圖層；地形示範用 `build_hills_scenario` |
| 把規劃回放當成飛行模擬或效能證據 | 它用的是規劃時的風、固定的轉換時間和空速；SITL 從沒設過風，匯出器設的又是地速而不是空速 | 畫面固定浮水印；效能主張只用 P6 的實飛資料 |
| 靠 SITL `--speedup` 做 30x | `start_multi` 沒有 speedup 參數；遙測沒有內插，30x 時位置、拖尾和光錐都會跳格；每筆遙測還要多次 runJavaScript，負擔會放大 | 用 Cesium 時鐘回放，倍速只是 `clock.multiplier` |
| 在 GUI 裡錄論文影片 | 視窗不在前景時 `QWidget.grab` 會拿到舊影格；`grabCanvas` 抓不到 HTML 疊層，還會把時鐘設成 `now()` | 用 harness 決定性逐幀錄製（P5） |
| B 的獨立 `player.html` | 會和 `main.html` 形成兩套播放器，日後必然分歧 | 用 Qt-free 的 `build_cesium_html`，讓同一頁兩處都能跑 |

---

## 7. 風險與要先確認的未知數

| # | 未知數或風險 | 為何重要 | 怎麼確認（不碰任何埠） | 何時 |
|---|---|---|---|---|
| 1 | ~~光錐填色的雙重變換：只確認到程式碼層面~~ **已實測確認（§1.5 T3）**：填色離 UAV 約 76.5 萬 km | E9 是影片招牌畫面 | P2 修正時，用 T3 的 boundingSphere 量測當突變對照 | P2 |
| 2 | 2D→3D 切換會清掉 3D 航線，目前只追過程式碼（`04_dccpp.png` 是先切到 3D 才規劃，所以沒踩到） | 決定 demo 能不能從 2D 開始 | 使用者在桌面操作一次 | P2 之前 |
| 3 | 實飛回放的跨機時鐘：GPS ToW 和 release 錨點哪個對，各 instance 推算偏差 0.2–0.7 s | 以約 18 m/s 計，誤差可達數公尺到十幾公尺 | 兩法並列、殘差寫進 meta，並鎖定 `NUMBERS.json`。注意：這只證明「和已發表的數字一致」，不證明時鐘絕對正確 | P6 |
| 4 | BIN 裡是否真有 CAM 紀錄：位元組掃描看到 CAM（0x51），log 15 中 A1/B2/B3 為 171/114/228 筆；BEAMIDP1 三機都是 754，很可疑 | 決定能不能畫實拍足跡 | 用 pymavlink 只讀檔案驗證，並確認路徑是 `.bin` 檔、不是埠名 | P6 之前 |
| 5 | 規劃與實際上傳不一致：fence 繞行改寫了 `assembled_paths`，但沒改 `last_plan`（`main_window.py:5320,5646`） | 播放的路徑會和飛的不一樣 | P4 的 `divergence_guard` 搭配突變測試 | P4 |
| 6 | 歷史 8 趟的規劃 4D 時間軸沒有存檔，在已大幅修改的工作樹上重規劃不一定能重現 | 「規劃幽靈機」可能是錯的 | 重新匯出 `.waypoints` 並逐位元組比對；不一致就不畫幽靈機。之後在 `scripts/hv_sitl_run.py:318-345` 加上時間軸傾印 | P6 |
| 7 | 高度基準：`alt_rel` 被當成橢球高使用；DEM 是絕對高程；`geoid_undulation_m` 預設 0.0（`hv_dccpp/geo.py:108`） | 一開地形，UAV 可能浮在空中或埋進地形 | 在 meta 明確寫出 `height_mode`；手動清單第 9 項專門檢查 | P3、P4 |
| 8 | 影像來源是 Google 圖磚：需要網路、公開發表有授權疑慮、每次載入不一定相同 | 論文影片的可重現性與合法性 | 測試只用本機影像（NaturalEarthII）；正式版改用授權明確的圖源（例如國土測繪中心正射影像）或預先下載的本機圖磚，下載要使用者同意 | P5 |
| 9 | GUI 無法自動化驗收；Playwright 的 Chromium 和 QtWebEngine 6.11 內建版本不同 | harness 通過不代表 App 內一定正常 | 把邏輯集中在可測的 JS 和 controller，GUI 只留薄接線；在使用者桌面跑手動清單。MainWindow 已證實能在使用者桌面建起來（`runs/artifacts/aeroplan_flow/flow.txt`） | P4 |
| 10 | 效能：8 個 GLB、PathGraphics、光錐，加上大段 runJavaScript 與 4.5 s 安定窗口 | QtWebEngine 可能掉幀或凍結 | P1 起在 harness 記錄 FPS；CZML 分段送；安定窗口結束後才載入 | P1–P4 |
| 11 | 證據檔案很脆弱：約 718 MB 的 BIN 未納管，npz 被 gitignore，錄影腳本也未納管 | 跑一次 `git clean` 或清磁碟就沒了 | 建 sha256 清單、備份、把腳本納入版控 | **立即，P1 之前** |
| 12 | 安全：曾有子代理誤開真實的 COM38 | 會碰到實體飛控板 | 所有新測試都加 I/O 守衛；agent 不得執行 `scripts/_full_flow_capture.py`、`scripts/hv_sitl_run.py` 等會起 SITL 或開埠的腳本 | 全程 |
| 13 | 只有 HARVEST（VTOL 加 `hv_dccpp`）帶時間戳；舊 DCCPP 只能等速估算（`core/dccpp/dccpp_path_builder.py:98-140`） | 兩種時間觀念混在同一個 UI，容易誤解 | CZML 標記 `timing=estimated`，畫面顯示「估算」 | P3 |