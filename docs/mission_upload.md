# 航點檔讀寫（Mission Planner 風格）

對應 Mission Planner / QGroundControl 的四個動作：**Load WP File**（載入航點檔）、
**Write WPs**（寫入飛控）、**Read WPs**（從飛控讀回，即既有的「下載」）、
**Save WP File**（存成航點檔）。本文件只講這個 UI 層；底層解析器/序列化器
（`mission/waypoint_file.py`）與底層上傳協定（`mission/sitl_link.py` 的
`upload_mission_items` / `mission_uploaded`）已經是既有、已測試的核心，這裡不重複。

## 按鈕在哪裡

SITL HUD（`ui/widgets/sitl_hud.py`，`btn_download` 旁邊）：

| 按鈕 | 作用 | 需要連線？ |
|---|---|---|
| 📥 從飛控讀回任務航線（既有） | 從飛控讀回目前任務 | 需要 |
| 📂 載入航點檔 (Load WP File) | 從 `.waypoints` / `.plan` 檔讀進來，預覽到地圖 | 不需要 |
| ✍️ 寫入飛控 (Write WPs) | 把「已載入的航點檔」寫入**恰好一台**飛控 | 需要，且預設停用 |
| 💾 存成航點檔 (Save WP File) | 把「剛從飛控讀回」的任務存成 `.waypoints` | 不需要（但要先讀回過） |

另外，`DroneDetailPanel`（單機詳細面板）原本就死掉的「↥」按鈕現在也接上了：
點它等同於對**那一台**飛機執行 Write WPs（`write_wp_requested` 訊號 →
`MainWindow._on_detail_panel_write_wp` → `on_sitl_write_wp(target_links=[該 link])`），
守衛邏輯完全共用，不會因為走這條路就少檢查。HUD 上另一顆「↧」下載快速鈕
仍是死的，不在本次範圍內。

## 安全規則

1. **只能對恰好一台 UAV 寫入。** HUD 的「指令對象」若選到 0 台或多台，
   按下「寫入飛控」會先跳出提示、什麼都不送。這個專案曾經發生過「兩架飛機
   拿到同一份任務」的事故，這裡刻意用「零台/多台一律拒絕」擋掉，不用猜的。
2. **一定會跳確認對話框，而且內容就是真的要送出去的那份**（目標機、
   連線方式、檔名、項目數、第一/最後一個航點、以及檔案裡所有警告的完整
   清單）。對話框開著的時候另外去載入別的檔案，不會偷換掉已經要送出的
   內容——快照是在使用者按下「是」的那一刻才做的。
3. **對真實飛控（序列埠連線）用更重的措辭。** `looks_like_real_board_connection()`
   只用連線字串判斷是不是 `COMx` / `/dev/tty*` / 含 `serial`——這**只是
   提示用的啟發式**，判斷錯了不影響安全（不管哪種情況都一律要求明確按
   「是」），但判斷對的話：標題會變成「⚠️ 寫入真實飛控」、圖示換成警告、
   預設鈕從「是」改成「取消」，逼使用者多一步主動選擇。若飛機目前
   `ARMED` 且在 `AUTO` 模式，不論真板還是 SITL，對話框都會在最前面加一行
   「這將取代一份飛行中的任務」的警告。
4. **寫入期間會停用，避免連點造成競爭。** `MainWindow._write_pending_sysids`
   記錄「哪些 sysid 正有一筆寫入在等回報」；HUD 的「寫入飛控」鈕只在
   `_write_pending_sysids` 為空且有連線時可按，但這只擋得住 HUD 這一顆
   按鈕——`DroneDetailPanel` 的「↥」鈕沒有對應的 enabled 狀態，所以
   `on_sitl_write_wp()` 的入口自己會查一次 `_write_pending_sysids`，兩條
   路徑寫同一台飛控時彼此都擋得住，不管哪一個先按。斷線（含 HIL 全部
   斷線）、寫入送出時丟例外、寫入被連線中斷打斷，都會清掉 pending 狀態、
   重新啟用按鈕，並且中斷的情況會額外跳出提示（見下一條），不會讓按鈕
   永遠卡在「寫入中」。
5. **失敗一定要夠大聲。** `SITLLink.mission_uploaded` 這個訊號只有
   `MainWindow._on_mission_uploaded` 一個接收端（`on_sitl_connect` 裡每條
   link 接一次），成功會在狀態列顯示「✅ …任務寫入完成（已驗證）」；
   失敗（含逾時、被拒絕、只寫一半、驗證不符）一律跳 `QMessageBox.critical`，
   措辭明確寫著「**沒有被可靠地更新**」，並提醒「請先按📥從飛控讀回任務
   航線確認飛控上實際存著的內容，不要假設剛才的寫入已經生效」——絕不會
   讓失敗看起來像成功。寫入途中連線掉了（例如拔線、HIL 板斷電）也會用
   同一套「不要假設剛才的寫入已經生效」措辭另外跳一次提示，不是只有一行
   不起眼的斷線狀態列訊息。

### 「已寫入並讀回驗證 N 項相符」保證什麼、不保證什麼

`upload_mission_items(items, verify=True)` 內部走的是：上傳 → 立刻讀回 →
逐項比對，比對規則在 `mission/sitl_link.py::_mission_readback_diffs`：

- **保證**：項數相符；`command` 完全相同；位置類指令的 lat/lon 誤差
  ≤ 1 個 int32×1e7 的格（約 1.1 cm）、alt 誤差在飛控以 cm 存放的量化範圍內；
  `param1..4` 在已知的「飛控本來就會量化/夾值」白名單內視為相符（見
  `_mission_param_matches` 與 `_MISSION_CMDS_LOSSY_PARAMS`）。
- **不保證／刻意不比對**：
  - **`current` 欄位完全不比。** MAVLink 任務協定裡 `current==2` 是
    「GUIDED 單點」、`current==3` 是「只改高度」——這兩種值即使出現在上傳
    途中的某一項，飛控也會**立刻 ACK、不進任務**，所以本專案的寫入路徑
    一律把 `current` 送成 `0`。讀回時 `current==1` 只表示「這是目前正在
    執行的 nav 指令」，跟檔案內容完全無關（`MissionItemProtocol_Waypoints::get_item`
    的行為），驗證邏輯因此完全跳過這個欄位的比對。
  - **`seq 0`（Home 槽）整項跳過不比。** 飛控自己會用目前的 GPS 位置覆寫
    Home，跟檔案裡寫的內容本來就會不一樣，這是正常現象，不是驗證失敗。
    Write WPs 的送出仍然會合法地送出 seq 0（否則整體序列不合法），只是
    不拿它跟讀回結果比對是否相符。
  - **這是協定層面的相符，不是「飛控會照著飛」的保證。** 驗證只確認飛控
    儲存體裡的任務內容跟送出的一致，不代表飛控的任務執行邏輯、地形跟隨、
    圍籬互動等在實際飛行中會如預期——那些要另外用真的飛行驗證。
  - **本文件與本輪測試都只驗證到 `tools/mock_autopilot.py`**（見下方
    「測試方式」），**不是**真的 ArduPilot 韌體，也不是真的飛控板；mock
    對協定的模擬有其極限，真板子仍可能有 mock 沒有重現的行為差異。

## 支援的檔案格式與會被拒絕的情況

- 支援讀取：`.waypoints` / `.txt`（Mission Planner / QGroundControl 的
  `QGC WPL` 純文字格式）、`.plan`（QGroundControl JSON）。
- 支援寫出：`.waypoints`（`QGC WPL 110`）。
- 載入時的**錯誤**（會整份拒絕，不部分載入，也不會覆蓋先前已成功載入的
  那份）：檔案格式錯誤（缺欄位、不是數字）、任務是空的（0 或只有 Home 槽
  1 項）、項目數超過 `MAX_ITEMS`（飛控任務儲存上限換算出來的筆數）、
  欄位超出 MAVLink 合法範圍（lat/lon/command/frame）、NAV 指令的座標是
  `(0,0)` 且不在「原地起飛」白名單內（多半是漏填座標）、`DO_JUMP` 目標
  不存在或會形成「永遠繞不出、繞不到任何 NAV 指令」的死圈（含兩個
  `DO_JUMP` 前後相接形成的死圈）。
- 載入時的**警告**（仍允許寫入，但會在確認對話框裡完整列出，需要使用者
  自己判斷是否要繼續）：`seq` 欄位跟它在清單中的實際位置不符、指令不在
  已知列舉內（飛控可能仍支援）、Home（seq 0）位於 `(0,0)`、相對高度
  ≤ 0 或 < 1 m、`DO_*` 指令帶了飛控通常不會使用的非零 lat/lon、任務裡
  沒有任何 NAV 指令、第一個真正的動作不是起飛、降落之後還有執行不到的
  項目、最後一項不是 RTL/降落/盤旋、相鄰兩個 NAV 點相距超過 50 km。
- 寫入前的額外拒絕（跟檔案內容無關）：尚未載入任何檔案；指令對象不是
  恰好一台；該台已有一筆寫入在等回報。

## 失敗訊息與處置

| 情境 | 顯示 | 該怎麼做 |
|---|---|---|
| 載入檔案格式錯誤 | `QMessageBox.critical`「載入航點檔失敗」+ 原因 | 修正檔案格式後重試 |
| 載入檔案有驗證錯誤 | `QMessageBox.critical`「航點檔有錯誤，無法寫入」+ 完整錯誤清單 | 依清單修正檔案；先前已載入的任務不會被蓋掉 |
| 寫入對象不是恰好一台 | `QMessageBox.warning`／`information` | 在 HUD 選單一台 UAV 再重試 |
| 該台已有寫入在跑 | `QMessageBox.information` | 等前一筆完成（成功或失敗）再試 |
| 使用者在確認對話框按取消 | 不送出，狀態列/log 記一筆「使用者取消」 | 需要的話重新開一次 Write WPs |
| 送出當下丟例外（如 link 已失效） | `QMessageBox.critical`「Write WPs 失敗」 | 檢查連線狀態後重試 |
| `mission_uploaded(ok=False)`（逾時/拒絕/驗證不符） | `QMessageBox.critical`，措辭含「沒有被可靠地更新」 | 按「📥 從飛控讀回任務航線」確認實際內容，不要假設已生效 |
| 寫入中途連線中斷 | `QMessageBox.critical`，同一套「不要假設已生效」措辭 | 重新連線後讀回確認 |
| 存檔時飛控尚未讀回過任務 | `QMessageBox.information`「Save WP File」 | 先按「📥 從飛控讀回任務航線」 |
| 存檔寫入失敗（如路徑不可寫） | `QMessageBox.critical`「存檔失敗」 | 換一個可寫的路徑重試 |

## 測試方式與涵蓋範圍

### 有測到的

- **單元測試**：`tests/test_waypoint_file.py`（解析器/驗證器/序列化器）、
  `tests/test_mission_upload.py`（`upload_mission_items` 底層協定，含
  `TestLoopbackAgainstMockAutopilot`）、`tests/test_mission_write.py`（本次
  UI 層：HUD 按鈕/訊號、`DroneDetailPanel` 的「↥」、`MainWindow` 的
  Load/Write/Save/`_on_mission_uploaded`/`_update_write_wp_enabled` 等
  handler）、`tests/test_mission_download.py`（既有的讀回半邊，作為
  Write 要鏡射的樣板）、`tests/test_sitl_link_demux.py`、
  `tests/test_hil_connect_panel.py`、`tests/test_mission_tools.py`。
- **UI handler 測試手法**：因為 `MainWindow` 在本環境無法真的建構
  （`QtWebEngineWidgets` 會直接崩潰、沒有 traceback），所有 `MainWindow`
  層級的測試都是用 `types.MethodType` 把 unbound method 綁到一個假的
  `self` 物件上執行，外加 AST 層級的結構檢查（例如「`mission_uploaded`
  只在 `on_sitl_connect` 裡接一次」「HIL 全連線一定會走 `on_sitl_connect`」
  「`on_sitl_write_wp` 不會呼叫到 `on_sitl_upload_mission`」），不是真的
  Qt 訊號鏈路。`SITLHud` 與 `DroneDetailPanel` 本身不碰
  `QtWebEngineWidgets`，這兩個是真的建構物件測試。
- **突變測試（mutation testing）**：針對這批新增的 handler 手動構造了一批
  行為突變（例如「取消也照樣送出」「允許多台目標」「快照在對話框開著時
  被重新讀取」「拿掉 armed+AUTO 警告」「忘記標記/解除 pending」等），
  證明每一個都會被對應的測試抓到（紅），修正後轉綠。突變測試的產物只
  存在於本機 scratchpad，沒有進到 repo 樹。
- **端到端 loopback（本輪 final agent 獨立重做，未沿用前幾輪的結果）**：
  對 `tools/mock_autopilot.py`（純 Python、透過 127.0.0.1 UDP 臨時埠模擬
  ArduPilot QuadPlane 的任務協定）走了一次完整流程：
  1. 用 `load_mission_file()` 讀 `runs/artifacts/HV4SITL1/plan/A1.waypoints`
     ——一份真的 AeroPlan 匯出檔（98 個項目，含 `NAV_DELAY` 錯時起飛），
     驗證結果 0 錯誤、0 警告。
  2. 開一條真的 `SITLLink`，對 mock 呼叫
     `upload_mission_items(items, verify=True)`；`mission_uploaded`
     只發了一次，`ok=True`，訊息「已寫入並讀回驗證 98 項相符」。
  3. `download_mission()` 讀回，逐欄（`seq/command/frame/current/
     autocontinue/param1..4/lat/lon/alt`）跟送出的內容比對：**除了
     `seq=0` 的 `current` 欄位之外全部相符**——這正是上一節說明過的、
     `_mission_readback_diffs` 本來就刻意不比對的那個欄位/那一項，不是
     缺陷。
  4. 把讀回的 98 項用 `save_wpl()` 存成暫存 `.waypoints`，再用
     `load_mission_file()` 重新讀回：0 錯誤，98 項與存檔前逐欄完全相符
     （0 個差異）。
  - 這個流程只連 `127.0.0.1` 的臨時埠、沒有開任何序列埠、沒有啟動
    SITL/ArduPilot，過程有硬性 timeout 與保證清理（`finally` 關 socket、
    `join` thread）。

### 沒有測到的（已知落差）

- **沒有對真的 ArduPilot 韌體測過**，不管是軟體 SITL（`sim_vehicle.py`）
  還是真板子（Cube Black/Orange+）——本輪任務規則明確禁止開啟任何序列埠
  或啟動 SITL 行程，所以「mock 之外」的協定邊角案例（例如真韌體特有的
  `MISSION_TYPE` 互動、`PreArm` 檢查、參數與任務的交互作用）未經驗證。
- **沒有測過真的 GUI 訊號鏈路**——`MainWindow` 無法在本環境實例化，
  所有 UI 層測試都是 unbound method + 假 `self`，真的按鈕點擊、真的
  Qt event loop、真的 `QMessageBox` 彈窗互動都沒有走過。
- **`DroneDetailPanel` 的「↧」下載快速鈕仍是死的**——同一類缺陷，這次
  只修了「↥」上傳，下載那顆不在本次範圍內。

## 手動測試清單（建議操作順序）

1. **先在 SITL 上跑一輪完整流程**（`sim_vehicle.py` 或既有的 SITL 啟動
   方式，非本次自動化測試允許的範圍，需要使用者自己手動啟動）：
   - 連線 → 📥 讀回一次現有任務，確認地圖顯示正常。
   - 📂 載入一份 `.waypoints`／`.plan`（可用 `runs/artifacts/` 底下任何
     一份既有匯出），確認預覽正確、警告清單合理。
   - HUD 選定單一 UAV → ✍️ 寫入飛控，確認對話框內容（目標、連線方式、
     項目數、首尾航點、警告）與實際要送出的一致，按「是」。
   - 確認狀態列出現「✅ 任務寫入完成（已驗證）」，再按一次 📥 讀回，
     比對跟剛才寫入的內容一致。
   - 💾 存成航點檔，確認存出來的 `.waypoints` 內容跟讀回的一致，且能
     被 Mission Planner / QGroundControl 正常開啟。
   - 故意選 0 台或多台指令對象、故意在對話框按「取消」、故意在寫入還沒
     回報時再按一次「寫入飛控」，確認三種情況都被正確擋下且訊息清楚。
2. **再拿一顆接了真飛控板的機體（螺旋槳先拆下）測一次同樣的流程**：
   - 確認連線字串是 `COMx` 之類時，對話框標題/圖示/預設鈕確實變成
     「真實飛控」的加重版本，且預設鈕是「取消」。
   - 全程只做到 ARMED 前為止（螺旋槳已拆，即使誤觸發也不會轉動），
     驗證寫入/讀回一致後即結束，不進行實際起飛。
   - 若板子當下已經 ARMED 在 AUTO（例如接續前一次飛行測試），刻意觸發
     一次 Write WPs，確認「這將取代一份飛行中的任務」的警告有出現。

---

*本文件覆蓋 UI 層（`ui/widgets/sitl_hud.py`、`ui/main_window.py`、
`ui/widgets/drone_detail_panel.py`）；底層解析/上傳協定
（`mission/waypoint_file.py`、`mission/sitl_link.py`）另有自己的模組
docstring 與測試，不在此重複。*
