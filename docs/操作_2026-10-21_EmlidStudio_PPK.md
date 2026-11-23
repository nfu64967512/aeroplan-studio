# 操作說明：用 Emlid Studio 解 PPK 軌跡，接進航測後處理 — 2026-10-21

> 依據：`docs/評估_2026-10-21_EmlidStudio_PPK接入.md`（§1.2 設定、§2.3 為什麼不用 events.pos、§3.2 基站輸入、§4 M1–M4）。
> 程式：`survey/ppk/pos.py`（讀表頭）、`survey/pipeline.py`（三道數值鎖）、`ui/dialogs/survey_process_dialog.py`（PPK 分頁）。
> 以下設定只在 Emlid Studio（ES）1.5 的實檔上核對過；1.10 沒有實檔（評估 §6）。

## 1. 流程

1. ES 的 **Kinematic** 處理：Rover 放相機模組的 `rover.ubx`（或轉好的 RINEX），Base 放基站觀測，Navigation 放星曆。
2. 基站座標**手動輸入**（不要用 RINEX 表頭的預設值），記下輸入的緯度、經度、**橢球高**、天線高。
3. 匯出 rover 的 `.pos`（**整條軌跡**，不是 `_events.pos`）。
4. AeroPlan「航測後處理 → PPK」：選「用架次表填的既有 .pos」，架次表每架填自己的 `.pos`，
   **基站座標欄照第 2 步填同一組數值**，框架名稱填基站座標所屬的框架（例如 `e-GNSS[2025]`）。
5. 開始處理。照片配對、曝光中心、槓桿臂、只取 fix 的規則都由 AeroPlan 自己做（與 RTKLIB 路線相同）。

## 2. ES 設定檢查表

| ES 設定 | 要選 | 選錯會怎樣 |
|---|---|---|
| Positioning mode | Kinematic | 單點／靜態沒有 `% ref pos` ⇒ 整架擋下 |
| Filter type | Combined | （無法從檔案核對） |
| Solution format | Lat/Lon/Height | E/N/U-Baseline 讀檔就拒絕 |
| Time format | `hh:mm:ss GPST` 或 `ww ssss GPST` | 表頭寫 UTC／JST 讀檔就拒絕；**標 GPST 但其實是 UTC** ⇒ 時間跨度鎖擋下（第一筆解早 18 s） |
| Latitude/Longitude format | 十進位度 | 度分秒讀檔就拒絕 |
| GLONASS AR | 基站不是 Reach 時選 Off | （無法核對） |
| 基站座標 | 手動輸入：緯度、經度、Ellip. height（**橢球高**）、天線高 | 見第 3 節 |
| Rover 天線 | 天線高 **0**、型號留空 | 填了天線高整條軌跡會往下移一個天線高（槓桿臂是對相機模組天線量的） |

## 3. 基站座標要怎麼填

- **兩邊填同一組**：ES 裡手動輸入的座標，與 AeroPlan PPK 分頁的「基站緯度／經度／橢球高」必須一致。
  AeroPlan 會拿 `.pos` 表頭的 `% ref pos`（ES 實際用的基站位置）逐項比對，水平、垂直各容許 5 mm；
  不一致就**整架不用 PPK**（照片保留飛控位置、標 `ppk_unavailable`、QA 未通過），訊息會寫出兩組座標與差值。
- **標石的橢球高**：AeroPlan 的「基站橢球高」是標石（天線高另填）。若訊息說「垂直差恰好等於天線高」，
  代表 ES 的 `ref pos` 可能是天線參考點而不是標石，請確認兩邊的高度定義（ES 這一點未用實檔驗證，評估 §6 第 4 項）。
- **不要填正高**：欄位是 Ellip. height。若 ES 與 AeroPlan 兩邊都填了同一個正高，`ref pos` 對得上，
  但整條軌跡會低一個大地起伏 N（本場址約 18.5 m）；這種情況由第 4 節的粗差檢查擋下（QA 未通過）。
- **框架名稱**：ES 表頭永遠寫 WGS84，那只是標籤。照片的 `crs_h` 用 AeroPlan 填的框架名稱，
  所以基站座標一定要是那個框架的座標（例如 e-GNSS 站公告值，或先用 ES Static 對 e-GNSS 解出的位置）。

## 4. AeroPlan 會擋什麼（數值鎖）

兩條路線（RTKLIB 自己解、既有 .pos）都套。結果寫進 `qa.json` 的 `qa.per_sortie.<呼號>.ppk`，QA 表也有對應列。

| 鎖 | 條件 | 抓的是 | 不通過 |
|---|---|---|---|
| 基站座標（M2b） | `% ref pos` 與填的基站座標，水平、垂直各 ≤ 5 mm；沒有 `% ref pos` 就擋 | ES 用了 RINEX 表頭的單點平均位置；不是相對定位 | 整架不用 PPK |
| 觀測跨度（M2a） | `% obs start` ≥ rover.ubx 第一個有效（week>0）RAWX − 0.1 s；`% obs end` ≤ 最後一個 + 0.1 s；第一筆解 ≥ obs start − 0.05 s；缺 obs start／end 就擋 | 時間標 GPST 實為 UTC（−18 s）；別架、別次的 .pos；rover.ubx 被截斷 | 整架不用 PPK |
| age(s) | 中位數 > 1 s 出警告 | 基站觀測取樣太稀（例如 30 s 被內插） | 只警告 |
| 對飛控粗差（M3） | 已採用 PPK 的照片，PPK 相機中心 − 飛控相機中心的中位數，水平或垂直 > 15 m | 正高當橢球高、時間偏移但表頭也一起偏、拿到別架的 .pos | 該架 QA 未通過 |
| events.pos | 檔名結尾 `_events.pos` | 把 ES 的事件檔當軌跡 | 開始處理前就擋 |

15 m 是**提案值**，沒有用首飛資料校準；首飛後依實際的 PPK−飛控差距調整（`PpkSettings.fc_gross_h_m／fc_gross_v_m`，
生效值寫進 `qa.json` 的 `effective.ppk`）。

## 5. 為什麼不用 `_events.pos`

- 它只按時間順序對照片（張數必須剛好相等），遺失或多出的觸發看不到；多機、每架數千張時差一張就整批錯位。
- 時間是下降邊緣、四捨五入到 1 ms；我們的相機模組預設上升邊緣，差一個脈衝寬（CAM1_DURATION 0.1 s ≈ 2 m）而且看不出來。
- 位置是天線位置，沒有曝光中心偏移、沒有槓桿臂與姿態。

AeroPlan 從同一個 `rover.ubx` 自己讀 TIM-TM2，逐邊緣對 Pi 的幀，再在 ES 的整條軌跡上內插，以上三點都處理掉。
