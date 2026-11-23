# 評估：用 Emlid Studio 做 PPK、接進 AeroPlan 航測後處理 — 2026-10-21

> 範圍：只評估，**沒有改任何程式**。實測用的是 Emlid 論壇上公開的三個附件（使用者 2026-10-21 同意下載），只放在 session scratchpad，沒有進 repo。
> 證據等級標記：**【實測】**＝用我們的 `survey/ppk` 程式跑過真檔；**【文件】**＝Emlid 官方文件；**【論壇】**＝Emlid 員工或使用者在官方論壇的發言／截圖；**【推論】**＝依 RTKLIB 行為推得、沒有對 Emlid Studio 實檔驗證。

## 結論

1. **Emlid Studio（以下簡稱 ES）的軌跡 .pos 就是 RTKLIB 格式**，現在的 `pos.py` 不用改就讀得進來【實測：ES 1.5 輸出 2383 筆，`read_pos` 通過】。前提是在 ES 設定選 `Solution format = Lat/Lon/Height`、`Time format = hh:mm:ss GPST` 或 `ww ssss GPST`、經緯度用十進位度。
2. **建議只用 ES 解整條軌跡**。照片配對、曝光中心、槓桿臂與姿態、只取 fix 的規則都沿用我們自己的程式。**不要直接讀 `_events.pos`**。實測同一個 UBX 發現：ES 的事件時間是**下降邊緣、四捨五入到 1 ms**（33/33 相符；上升邊緣只有 24/33）。邊緣計數器跳號（遺失的觸發）在 events.pos 裡完全看不到。而且事件位置是天線位置，沒有加曝光中心時間偏移，也沒有加槓桿臂。
3. **現在的「既有 .pos」路線有一個違反有效值契約的缺口**：只要求填框架名稱，**不要求基站座標，也不核對 .pos 實際用了哪個基站位置**。ES 的基站位置預設取 RINEX 表頭，表頭永遠寫 `WGS84/ellipsoidal`（Emlid 員工確認）。所以操作員填「e-GNSS[2025]」時，座標可能其實是單點平均出來的基站位置，下游不會發現。
4. **`pos.py` 也擋不住「標 GPST、實際是 UTC」的檔案**【實測：把 ES 檔時間整體減 18 s，`pos.py` 照收】。但 .pos 表頭的 `% obs start (weekWWWW TOW s)` 可以當數值鎖：在 ES 1.5 實檔上，它與 rover.ubx 第一個有效 RAWX epoch 只差 7 ms；時間偏 18 s 時，第一筆解會早於 obs start 18.000 s。
5. **最小安全改動約 3.5–4 人日**，分四塊：`pos.py` 讀出表頭中繼資料；「既有 .pos」路線加三道數值鎖（rover 時間跨度、`% ref pos` 對基站座標、PPK 對飛控位置的粗差）；GUI 在 .pos 模式也要求基站座標；以真的 ES 輸出錄製測試檔。完全不改程式也能用，但不符合契約，不建議拿來做交付物。

---

## 1. ES 的 .pos 格式與要怎麼設定（問題 1）

### 1.1 實檔核對【實測】

檔案：Emlid 員工 kirill.pavlyuchuk 2023-04-18 在 <https://community.emlid.com/t/34127> 附上的 ES 輸出（他用 Forward 濾波、沒用基站觀測，所以全部是單點解 Q=5）。表頭逐字如下（員工本機路徑以 `<略>` 代替，座標遮掉後幾位）：

```
% program   : ES 1.5
% inp file  : <略>/localhost_raw_20230407222732.23O
% inp file  : <略>/localhost_raw_20230407222732.23P
% obs start : 2023/04/07 22:27:59.0 GPST (week2256 512879.0s)
% obs end   : 2023/04/07 23:27:10.0 GPST (week2256 516430.0s)
%
% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=# of satellites)
%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
2023/04/07 22:27:59.000   39.72xxxxxx -105.11xxxxxx  1689.1619   5  19   3.7208   2.6554   7.4951  -0.9589   1.1670   1.7045   0.00    0.0
2023/04/07 22:27:59.999   39.72xxxxxx -105.11xxxxxx  1689.1612   5  20   3.4903   2.5267   7.0477  -1.0957   0.9476   1.3293   0.00    0.0
```

觀察：

| 項目 | ES 1.5 實檔 | 對 `pos.py` 的意義 |
|---|---|---|
| 程式名 | `% program   : ES 1.5` | 可記入 qa.json 當來源（目前沒讀） |
| 時間 | `GPST` 曆法時間 `yyyy/mm/dd hh:mm:ss.sss`（time_format=hms） | 接受 |
| 高度 | `lat/lon/height=WGS84/ellipsoidal` | 接受；**但 WGS84 只是標籤**（見 1.3） |
| 欄位 | latitude…ratio 共 14 欄，**沒有速度欄** | 接受；`interp.py` 改用中央差分切線（已支援） |
| 處理選項 | **沒有** `% pos mode`、`% elev mask` 等行（out-outopt 關） | 無法從檔案核對 ES 的解算設定 |
| `% ref pos` | 本檔沒有（單點模式，RTKLIB 只在相對定位模式印） | 相對定位時應該有：2025 年使用者說 ES rover .pos 有 `% ref pos   :` 行【論壇 t/40656】，本評估沒有實檔 |
| 行尾 | LF、檔尾有換行 | 接受 |
| epoch | 1 Hz，時間有接收機時鐘抖動（`.999`／`.000`），間隔最小 0.993 s | 接受；嚴格遞增 |

`read_pos` 結果：`n=2383 fmt=llh tf=hms datum=WGS84 vel=no Q={5: 2383} skipped=[]`。同一個討論串的 `_events.pos` 也通過（33 筆）。

### 1.2 ES 設定 → `pos.py` 要求對照

ES「Kinematic processing settings」的輸出區塊有這些選項【論壇：Emlid 員工 2024-11-07 截圖 t/40187；beta 截圖 t/25831】：Satellites、Integer ambiguity resolution（GPS／GLONASS／BDS）、**Solution format**（截圖值 Lat/Lon/Height；另有 E/N/U-Baseline、NMEA0183【論壇 t/33302、t/35422】）、**Time format**（下拉四項：`ww ssss GPST`、`hh:mm:ss GPST`、`hh:mm:ss UTC`、`hh:mm:ss JST`）、Latitude/Longitude format、Debug trace。截圖裡**沒有**高度型別（ellipsoidal／geodetic）、輸出表頭、輸出速度、欄位分隔符號這幾個 RTKPOST 有的選項。

| ES 設定 | 要選 | `pos.py` 行為 |
|---|---|---|
| Positioning mode | Kinematic | （無法從檔案核對） |
| Filter type | **Combined**（與 `rtklib.py` 的 fix-and-hold 前提一致） | （無法核對） |
| Solution format | **Lat/Lon/Height** | E/N/U-Baseline 會被拒（ENU）；NMEA 副檔名是 .nmea |
| Time format | **hh:mm:ss GPST** 或 **ww ssss GPST** | 表頭寫 `UTC`／`JST` 會被拒；**若 ES 選 UTC 卻仍標 GPST，會被靜默接受**（未驗證，見 §6） |
| Latitude/Longitude format | 十進位度 | 度分秒會被拒 |
| GLONASS AR | 基站不是 Reach 時選 **Off**（ES 官方說法【文件 tuning-solution】，與 `rtklib.py` 的 `pos2-gloarmode=off` 一致） | （無法核對） |
| 基站座標 | **手動輸入**（Lat/Lon/Height、Ellip. height、天線高） | 目前完全沒核對 ⇒ 見 §4 M2b |
| Rover 天線 | 天線高 0；天線型號留空（理由見 §3.3） | （無法核對） |

### 1.3 這些地方要小心

- **`WGS84` 標籤沒有意義**：Emlid 員工 2023-05-10 回覆「As for the base datum, it's always shown as WGS84」【論壇 t/34343】。解算結果落在**基站座標所屬的框架**，高度是那個框架橢球的橢球高【論壇 t/37358 員工回覆】。這與 `pos.py`、`outputs.py` 現在的設計一致：只接受 `WGS84` 標籤，真正的框架由 `PpkSettings.frame` 宣告。問題在於宣告與實際用的基站座標之間沒有任何連結，見結論 3。
- **沒有速度欄**：ES 沒有輸出速度的選項，Hermite 切線會改用中央差分。`interp.py` 的文件已量化這個誤差：同一條軌跡最大 0.10 mm，直線接轉彎處約 1.8 mm，可以接受。
- **epoch 率與 age**：本檔 rover 是 1 Hz。我們的相機模組 F9P 應該是 10 Hz；1 Hz 的話 `max_gap_s=0.25` 會把全部照片標 `ppk_gap` 退回飛控位置（安全的失敗，但等於不能用）。ES 會不會內插 1 s／30 s 的基站觀測（`rtklib.py` 預設 `misc-timeinterp=on`）未知，`age(s)` 欄會反映出來，目前沒有讀它。

---

## 2. `_events.pos` 與我們「逐邊緣配對」的比較（問題 2）

### 2.1 同一個 UBX 的逐筆對照【實測】

同一討論串的使用者也上傳了原始 UBX（Reach M2，`localhost_raw_20230407222732.UBX` 9,046,608 B）。我們用 `ubx.py`／`events.py` 解析：

- 封包：RXM-RAWX 3559、RXM-SFRBX 62178、TIM-TM2 36，**沒有 NAV-PVT**，檢查碼錯誤 0。所以 `events.py` 的 NAV-PVT 時間基準鎖不會啟動（會出警告）；RAWX 閏秒 18 與設定相符。
- TIM-TM2：timeBase 全是 1（GNSS 時間，Reach 設成 GPS）、ch 0。36 則中 33 則有效，3 則 `time_not_valid`。
- **count 跳號**：有效段內 count 從 1 到 61，只收到 33 則 ⇒ `events.py` 回報遺失 28 個邊緣（整檔 33 個）。6 個跳號處的上升與下降時刻完全相同（推測是同一 epoch 內有多個邊緣），符合整合手冊「兩個 epoch 間只報最後一個邊緣」的描述；另外 2 個跳號處（3→14、14→18）沒有這個現象。
- 脈衝：先下降、0.395 ms 後上升（低電位有效）。

把 ES 的 `_events.pos`（33 筆）逐筆對照：

| 對照 | 結果 |
|---|---|
| 筆數 | ES 33 ＝ 我們的有效邊緣 33（3 個無效的 ES 也排除了） |
| 時間 = 下降邊緣四捨五入到 ms | **33/33** |
| 時間 = 上升邊緣四捨五入到 ms | 24/33 |
| ES − 下降邊緣 | −432～+491 µs（就是 1 ms 量化） |
| 事件位置 vs 我們在 ES 自己的 .pos 上線性內插 | 中位數 2.5 cm、最大 16.9 cm（單點解、1 Hz、低速）；事件的 ns 與 sd 欄直接抄前後某一個 epoch |

注意：這組資料的下降邊緣同時也是**較早**的邊緣，所以分不出 ES 是「固定取下降邊緣」還是「取較早的邊緣」。

### 2.2 兩種做法的差別

| 項目 | ES `_events.pos` | 我們（`events.py` → `interp.py` → `pipeline._apply_ppk`） |
|---|---|---|
| 與照片的對應 | 只有時間順序，沒有檔名也沒有 count；**張數必須等於照片數**才能 geotag【文件】；多出來的要使用者手動刪【論壇 t/29869】 | count−seq 最長一致鏈＋時鐘擬合，逐邊緣對 Pi 幀；有歧義就拒絕 |
| 遺失的邊緣 | 看不到（本例 28 個遺失，events.pos 沒有任何痕跡） | `lost` 逐段列出；`tm2_missing`／`fit_extrapolated` 旗標 |
| 無效標記 | 本例排除；但 2023-02 員工案例中 ES 把 3153 個無效標記也算成事件（共 3270）【論壇 t/32988】，行為不一致 | `invalid` 逐筆附原因，不進配對 |
| 邊緣極性 | 固定（本例＝下降邊緣） | `edge="rising"\|"falling"` 由呼叫端宣告，必須與 Pi 記的同一個 |
| 時間解析度 | 1 ms（20 m/s 下最多約 1 cm） | ns |
| 曝光中心 | 沒有（就是邊緣時刻） | 加上 `(sensor_ts − edge) + exposure/2 + readout/2` |
| 相機中心 | 天線位置；槓桿臂「由攝影測量軟體處理」【論壇 t/43428 員工】 | 槓桿臂＋飛控姿態；沒有姿態就丟錯 |
| 內插 | 方法未公開；與線性內插差到 17 cm（本例） | Hermite／線性，誤差已量化；跨缺口改線性並標 `ppk_gap` |
| fix 規則 | 每個事件只有一個 Q（抄自某一個 epoch） | 前後兩個 epoch 都要 fix、沒有缺口、非外插 |

### 2.3 建議

**只用 ES 解整條軌跡（Kinematic processing 的 rover .pos），配對與內插沿用我們的。** 理由：

1. 我們的邊緣極性預設是 rising。若相機模組要的是上升邊緣，ES 的下降邊緣會讓**所有**照片差一個脈衝寬：`events.py` 文件寫明 CAM1_DURATION 0.1 s 等於 2 m，而且擬合看不出來。
2. events.pos 沒有曝光中心偏移，所以就算要用，也得回頭讀 Pi 會期把偏移加回去，等於還是要做配對。
3. HARVEST 三機同時飛、每架數千張，張數只要差一張，「按順序對」整批錯位。這正是 `events.py` 開頭「never pair blindly by order」要防的事，論壇上也有人寫過同樣的風險【論壇 t/18708】。

若仍要直接讀 events.pos，會失去的檢查有：count−seq 一致鏈與歧義拒絕、遺失／重複／無效邊緣的明細、逐張檔名綁定、`tm2_missing`／`fit_extrapolated`／`counter_shift` 旗標、NAV-PVT 時間基準鎖與 RAWX 閏秒鎖、邊緣極性選擇、曝光中心偏移、槓桿臂＋姿態（含「有槓桿臂沒姿態就丟錯」）、前後 epoch 都 fix 的規則、`ppk_gap`、σ 線性傳播（完全相關上界），以及 ns 級時間。

---

## 3. 非 Emlid 的 F9P UBX、基站輸入、框架與高程（問題 3）

### 3.1 我們相機模組的 F9P UBX 能不能進 ES

- ES rover 欄標示接受「U-BLOX・RINEX」，基站欄接受「U-BLOX・RTCM3・RINEX」【論壇：beta 10 截圖 t/25831】。UBX 會在背景以預設設定轉 RINEX【論壇 t/25831 #1、文件 kinematic-processing】。Emlid 員工 2024-11 說 ES「compatible with any receivers' logs, as long as they are in RINEX or UBX format」【論壇 t/40262】。
- Reach M2 的 UBX 實際內容是 RAWX、SFRBX、TIM-TM2【實測】。我們的 rover.ubx 還多了 NAV-PVT。RTKLIB 系的轉檔器會忽略不認得的訊息；ES 是否也如此是【推論】。
- **建議的做法不依賴 ES 轉 TIM-TM2**：ES 只需要 RAWX／SFRBX 轉出觀測檔與星曆，時間標記由我們自己從同一個 rover.ubx 讀。ES 順手產生的 `_events.pos` 直接忽略。
- 若將來想用 ES 的事件，有兩點要先做：F9P 出廠的 TM2 時間基準是 **UTC**（CFG-TP-TIMEGRID_TP1=0，`events.py` 文件已註明），Reach 用的是 GNSS／GPS。ES 怎麼處理 UTC 基準的 TM2 沒有驗證，要嘛把 F9P 改成 GPS 基準，要嘛先做 §5 的極性與時間對照。

### 3.2 基站資料怎麼給 ES

| 基站 | ES 輸入 | 注意 |
|---|---|---|
| e-GNSS 連續站 RINEX | Base 欄放 RINEX 觀測檔；Navigation 欄放 rover 或基站的星曆（ES 必須有導航檔【文件】） | 取樣 1 s 或 30 s；ES 會不會內插基站觀測未知（看 `age(s)`）。e-GNSS 的 Hatanaka 壓縮檔（.crx）ES 吃不吃未驗證，必要時先解壓 |
| Reach 基站（RS2／RS3） | Base 欄放 UBX 或 RINEX | 基站本身要先定位：架在已知點，或先用 ES Static 對 e-GNSS 解出「One best」.pos 再拖進基站欄【文件 kinematic-processing】，這樣座標就繼承 e-GNSS 的框架 |
| 基站座標 | **預設取 RINEX 表頭**【文件】；可手動輸入 Lat/Lon/Height（dd）＋**Ellip. height**＋天線高，選了天線型號會自動加天線偏移（截圖「RS2 offset will be applied」）【論壇 t/25831 #213】 | 一定要手動輸入。表頭位置通常是接收機單點平均，整批照片會偏公尺級，而且看不出來 |

### 3.3 框架名稱與高程基準怎麼接到有效值契約

| 契約要求 | ES 實際行為 | 做法 |
|---|---|---|
| 基站框架名稱必填（成為 `crs_h`） | ES 沒有「框架」概念；表頭永遠寫 WGS84【論壇 t/34343】 | 維持 `PpkSettings.frame` 必填。另外**要求 .pos 路線也填基站座標**，並與 .pos 的 `% ref pos` 逐位核對（M2b）。框架名稱才算真正綁到「ES 實際用的那組座標」 |
| 高度一律橢球高 | ES 沒有正高輸出選項（截圖無此項）；高度＝基站框架橢球的橢球高【論壇 t/37358、t/33302】 | `pos.py` 照舊要求 `ellipsoidal`。正高仍由下游以明確的大地起伏模型換算 |
| 正高不可當 N=0 | ES 的欄位叫「Ellip. height」，但使用者可能把水準點的正高填進去。測區大地起伏約 18.5 m（見 `評估_2026-10-14`），**所有照片會整體低約 18.5 m，ES 不會警告** | (a) M2b 讓操作員在 AeroPlan 再填一次橢球高，與 `% ref pos` 比對，兩邊不一致就擋；(b) M3 拿 PPK 天線高對飛控 h_ell 做粗差檢查（約 18.5 m 會被抓到）。兩邊都填錯同一個正高時，只有 (b) 抓得到 |
| 天線參考點一致 | ES 認得 rover 天線型號時，.pos 高度是 ARP；若再填量測天線高，就變成地面點【論壇 t/38079 員工】 | `rtklib.py` 沒設 rover 天線（相位中心近似）。`session.json` 的槓桿臂是對「相機模組天線」量的，所以 ES 的 rover 天線高**一定要填 0**（否則整條軌跡往下移一個天線高），型號留空，兩條路線才一致（後者未實測） |

---

## 4. 最小實作方案與工時（問題 4）

前提：沿用「既有 .pos」欄位與 `_ppk_trajectory()` 的 `pos_file` 分支，**不新增 Emlid 專屬流程**。ES 只是「使用者自備 .pos」的一種來源。

| 編號 | 內容 | 檔案 | 工時 |
|---|---|---|---|
| M1 | `pos.py` 讀出表頭中繼資料，不改既有行為：`program`、`inp file` 清單、`obs start／end` 括號內的 GPST 週／週內秒、`ref pos`（llh 或 xyz）、`antenna1／2`（有才讀）。加進 `PosTrajectory`，`pipeline` 寫入 qa.json 的 `trajectory` | `survey/ppk/pos.py`、`survey/pipeline.py` | 0.5 日 |
| M2a | **rover 時間跨度鎖**（兩條路線都套）：`obs start ≥ rover.ubx 第一個 week>0 的 RAWX epoch − 0.1 s`、`obs end ≤ 最後一個 + 0.1 s`、**第一筆解 ≥ obs start − 0.05 s**；缺 obs start 就丟錯。時間偏 18 s 或用錯架次的 .pos 會被擋下。另記 `age(s)` 統計，中位數 > 1 s 時出警告 | `survey/pipeline.py`（`_apply_ppk` 已經讀過 rover.ubx，不用重讀） | 0.5 日 |
| M2b | **基站座標鎖**：.pos 路線也必須給 `BaseStation`（緯、經、橢球高、天線高、框架），與 `% ref pos` 比對（水平 ≤ 5 mm、高 ≤ 5 mm；RTKLIB 印 9 位小數度與 4 位小數 m）；沒有 `ref pos` 行就丟錯（代表不是相對定位）。`antenna2` 只記錄、不比對（ES 會把天線型號偏移加進去，見 §3.2） | `survey/pipeline.py`（`PpkSettings.__post_init__`、`_ppk_trajectory`） | 0.5 日 |
| M3 | **PPK 對飛控粗差檢查**（兩條路線都套）：已採用 PPK 的照片，`天線位置 − fc_interp 位置` 的中位數水平 > 15 m 或垂直 > 15 m ⇒ 該架 QA 不通過。抓三種錯：正高當橢球高（約 18.5 m）、時間偏 18 s（20 m/s 下 360 m）、多機同時飛時拿到別架的 .pos。門檻是提案值，要用首飛資料校準 | `survey/pipeline.py` | 0.5 日 |
| M4 | GUI：.pos 模式也啟用基站座標欄（提示「要與 Emlid Studio 輸入的一致，會核對 .pos 的 % ref pos」）；檔名結尾是 `_events.pos` 時直接擋下並說明要給整條軌跡；檔案篩選改成「RTKLIB／Emlid Studio 軌跡 (*.pos)」 | `ui/dialogs/survey_process_dialog.py` | 0.5 日 |
| M5 | 操作說明：ES 設定檢查表（§1.2）、基站輸入（§3.2）、為什麼不用 events.pos（§2.3） | `docs/` | 0.25 日 |
| M6 | 測試（見 §5），含真 ES 輸出的錄製檔與每道鎖的突變驗證 | `tests/test_survey_ppk_pos.py`、`tests/test_survey_pipeline.py` | 1 日 |
| — | 使用者在 ES 1.10 產生錄製檔（§5.1） | （使用者操作） | 約 1–2 小時 |

合計約 **3.75 人日**（不含使用者操作）。只做 M1＋M2＋M4 是「符合契約的最小集」，約 2 人日＋其測試。M3 對 RTKLIB 路線同樣有用，建議一起做。

**不改程式能不能先用？** 能：PPK 分頁選「用架次表填的既有 .pos」，每架填 ES 的 rover .pos，`pos.py` 讀得進來（已實測）。但基站座標與時間系統都沒被核對，不符合有效值契約，**不建議用在交付物**。

---

## 5. 需要的測試

### 5.1 錄製檔（用真的 ES 輸出）

論壇檔案是第三方的位置資料，**不放進 repo**。建議由使用者用 ES 1.10、我們自己相機模組的一段 rover.ubx（地面靜置或台架即可）加一天的 e-GNSS RINEX，產生以下檔案作為錄製檔：

1. 正常組：Kinematic、Combined、Lat/Lon/Height、`hh:mm:ss GPST`、手動基站座標 ⇒ 必須通過，並驗證 `% ref pos` 等於手動輸入的座標、`% obs start` 對得上 rover.ubx。
2. `ww ssss GPST` ⇒ 必須通過，而且與第 1 組逐筆相同。
3. **`hh:mm:ss UTC`** ⇒ **必須被拒**。這一組是用來回答「ES 選 UTC 時表頭標籤到底寫不寫 UTC」：寫 UTC 就是 `pos.py` 擋；仍寫 GPST 就要靠 M2a 擋。
4. 度分秒經緯度 ⇒ 必須被拒。
5. 同一次處理產生的 `_events.pos` ⇒ 放進軌跡欄必須被擋（M4），並與我們的 `extract_edges` 做極性對照（§2.1 的做法）。這能順便回答 ES 對 F9P 的 **UTC 基準 TM2** 怎麼處理。
6. 基站座標故意用 RINEX 表頭預設值 ⇒ M2b 必須擋下。

### 5.2 單元／整合測試

- `pos.py`：ES 1.5 表頭結構（`% program   : ES 1.5`、無處理選項行、無速度欄、LF、`.999` 時間抖動）解析出 M1 的中繼資料；用合成數值，不放論壇的座標。
- M2a：通過；時間欄整體 −18 s ⇒ 擋（本評估已示範 `pos.py` 單獨會照收，第一筆解比 obs start 早 18.000 s）；別架的 rover.ubx ⇒ 擋；前段有 week=0 的 RAWX（Reach 檔前 7 個 epoch 就是）⇒ 不誤擋。
- M2b：一致 ⇒ 通過；差 1 cm ⇒ 擋；沒有 `ref pos` ⇒ 擋；ECEF 格式的 `ref pos` ⇒ 正確換算後比對。
- M3：正高當橢球高（全部 −18.5 m）⇒ 擋；時間偏 18 s ⇒ 擋；正常飛行雜訊 ⇒ 不擋。
- **每道鎖都要做突變驗證**：把鎖拿掉，對應測試必須變紅（依過去「驗證驗證器」的教訓）。測試照慣例設 `AEROPLAN_NO_FILE_LOG=1`、一檔一行程、`-p no:cacheprovider`。

---

## 6. 未驗證項目

1. **ES 1.10（2026-08-19，最新版）沒有實檔**。所有實測都是 ES 1.5（2023-04）的輸出。1.6～1.10 的更新紀錄沒有提到 .pos 格式變動【文件 changelogs】，但沒有驗證。
2. **ES 選 `hh:mm:ss UTC` 時，欄名行寫 `UTC` 還是 `GPST`**。有使用者指出地圖下方仍顯示 GPST，員工說已記下回饋【論壇 t/40187】，但檔案本身怎麼寫沒有看到。
3. `% obs start` 括號內的週／週內秒**永遠是 GPST**：這是依 RTKLIB `postpos.c outheader` 的記憶【推論】，ES 1.5 的 GPST 檔上一致，UTC 檔沒有驗證。M2a 依賴這一點，所以 §5.1 第 3 組要一起確認。
4. ES 相對定位時一定會寫 `% ref pos`，而且是手動輸入的標石座標（不是 ARP）：論壇有使用者提到這行【論壇 t/40656】，沒有實檔。
5. ES 是否內插 1 s／30 s 的基站觀測、輸出 epoch 率是否等於 rover 觀測率。
6. ES 對 F9P **UTC 基準** TIM-TM2 的處理、極性是「固定下降邊緣」還是「較早的邊緣」（本例兩者相同）。
7. ES 事件位置的內插方法；本例與線性內插差到 17 cm，fix 解時的差異不明。
8. e-GNSS Hatanaka 壓縮檔能不能直接進 ES。
9. Rover 天線高填 0、型號留空時，ES 輸出的參考點是否與 `rtklib.py` 一致（槓桿臂的基準點）。
10. ES 的完整設定清單（Positioning mode、Filter type 選項、Elevation／SNR mask 預設值）只看到截圖的下半部與文件片段。
11. Reach M2 的接收機晶片型號沒有在本評估內核對（只確認 UBX 內容是 RAWX／SFRBX／TIM-TM2）。
12. M3 的 15 m 門檻沒有用真飛行資料校準。

---

## 7. 來源

官方文件（2026-10-21 讀取）：
- Kinematic Processing：<https://docs.emlid.com/emlid-studio/kinematic-processing-workflow/>（基站預設取 RINEX 表頭、UBX 背景轉檔、輸出 .pos 與 `_events.pos`）
- Drone Data Processing：<https://docs.emlid.com/emlid-studio/drone-data-processing/drone-data-processing-workflow/>（「can only geotag photos when the number of timestamps is equal to the number of photos」）
- RTK Drone Data Processing：<https://docs.emlid.com/emlid-studio/drone-data-processing/rtk-drone-data-processing-workflow/>（MRK 的相機偏移會自動套用）
- Convert to RINEX：<https://docs.emlid.com/emlid-studio/convert-to-rinex-workflow/>
- Tuning Post-Processing Solution：<https://docs.emlid.com/emlid-studio/analyzing-logs/tuning-solution/>（Combined、SNR／仰角遮罩、非 Reach 設備時 GLONASS AR 關）
- Changelogs：<https://docs.emlid.com/emlid-studio/reference/changelogs/>（最新 1.10，2026-08-19）

官方論壇（原文以 Discourse `/raw/<id>` 讀取）：
- <https://community.emlid.com/t/34127>：本評估實測的三個檔案（員工附的 `.pos`／`_events.pos`、使用者附的 UBX）
- <https://community.emlid.com/t/40187>：Time format 改 UTC；員工設定截圖（四個時間格式選項）
- <https://community.emlid.com/t/34343>：表頭基站框架「always shown as WGS84」（員工）
- <https://community.emlid.com/t/37358>：ES 以 RTKLIB 為基礎；高度是基站框架橢球的橢球高（員工）
- <https://community.emlid.com/t/43428>：天線偏移取自 RINEX 表頭；events.pos 不加相機偏移，由攝影測量軟體處理（員工）
- <https://community.emlid.com/t/32988>：無效時間標記被算進事件（117＋3153＝3270）（員工）
- <https://community.emlid.com/t/40262>：ES 接受任何接收機的 RINEX 或 UBX（員工）
- <https://community.emlid.com/t/25831>：beta 討論串；rover「U-BLOX・RINEX」與基站輸入截圖（#195、#213）；2021 年輸出不含處理選項
- <https://community.emlid.com/t/40656>：ES rover .pos 有 `% ref pos` 行
- <https://community.emlid.com/t/18708>：重複時間標記，以及「按順序對」的風險
- <https://community.emlid.com/t/35799>、<https://community.emlid.com/t/29869>：geotag 要用 events.pos；多餘事件要手動刪
- <https://community.emlid.com/t/33302>、<https://community.emlid.com/t/35422>：Solution format 選項；ES 只輸出地理座標與橢球高
- <https://community.emlid.com/t/38079>：認得天線型號時 .pos 高度是 ARP；填量測天線高就變成地面點（員工）
- RTKLIB 2.4.2 手冊 Appendix B（論壇引用，本評估未逐字核對）：<https://www.rtklib.com/prog/manual_2.4.2.pdf>

## 附錄 A：實測檔案與腳本

檔案只放在 session scratchpad 的獨立目錄 `emlid_dl/`，沒有進 repo，以 `python -E -P` 解析、不執行：

| 檔案 | 大小 | SHA-256 |
|---|---|---|
| `localhost_raw_20230407222732 (3).pos`（員工附） | 336,702 B | `a037efa14c8504f0cab0415743ddf9524a60ec6181c6ffc645e3ec3f0affad33` |
| `localhost_raw_20230407222732_events (3).pos`（員工附） | 5,352 B | `8c8c4f129f73ddb0bc93a3f6751b54053d7e7202dbd83c44a3dd43bc1f1409e3` |
| `localhost_raw_20230407222732_UBX.zip`（使用者附） | 5,178,759 B | `7ac41e476c45e37bc5bf048a3b551e866c78354fd290c05064e8bbe666d69659` |

用到的腳本（scratchpad `scripts/`）：`check_emlid.py`（pos.py＋events.py）、`tm2_dump.py`（逐則 TIM-TM2）、`cmp_events.py`（極性與 ms 量化對照、事件位置對照）、`ns_check.py`（事件與前後 epoch）、`rawx_span.py`（obs start／end 對 RAWX）、`utc_shift_demo.py`（−18 s 示範）。程式只 import repo 的 `survey/ppk`，沒有修改任何檔案。
