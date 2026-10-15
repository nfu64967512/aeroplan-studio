# 圍籬對齊掃描角 —— 已完成

2026-09-04 · 分支 `quad_swarm` · 接續 `handoff_2026-09-03_fence_scan_angle.md`

---

## 結論

圍籬會跟著掃描角轉了。實測**方向誤差 0.0000°**（驗收門檻 1°），
量法是把圍籬頂點投回同一個 AEQD 平面，取**四條邊**與掃描角的夾角（模 90°）。

面積的部分要分兩種情況講，這是交接文件原本沒有分開的地方：

| 情境 | 對齊後 / 軸對齊 |
|---|---|
| 斜作業區 + 掃描線沿長軸（操作員實際會做的事） | **0.38–0.52×** |
| 軸對齊的方形作業區 + 硬設一個斜掃描角 | **1.68–1.91×** |

⚠️⚠️ **對齊掃描角不保證面積變小。** 交接文件量到的 2.06× 是拿「一疊沒有被
作業區裁切的掃描線」量的 —— 那等於預設點雲**沿掃描方向拉長**。真實情況裡點雲的
外形跟著**作業區**，條帶方向才跟著**掃描角**。兩者一致時省很多（上表第一列），
打架時反而變大（第二列）。

使用者要的是「方向跟航線一致」，不是「面積最小」（面積最小是 MRR，而 MRR
幾乎垂直於航線），所以這是**已知且刻意**的取捨，已鎖在
`tests/test_fence_scan_angle.py::test_manual_angle_fighting_the_area_can_enlarge_the_fence`。

---

## ⚠️⚠️⚠️ 途中挖到的三個坑（比原任務重要）

### 1. 45° 是**驗不出錯**的角度，而它正好是最像好題目的那一格

交接文件建議用 45° 做驗收。實際上正確轉換是 `φ = 90 − θ`，而最容易寫成
`φ = θ`（羅盤角直接當數學角）。兩者差 `90 − 2θ`，而「在旋轉框架裡取軸對齊
包絡」是 **90° 週期**的 ⇒ **θ 為 45 的倍數時兩種寫法給出同一個矩形**。

θ=30° 時錯誤寫法算出來的矩形甚至**跟完全沒轉一模一樣**，但仍會回報
`method='scan'`。⇒ 驗收角一律用 30°/62°/123.4° 這種非 45 倍數的角。
已鎖成正反兩條測試（`test_multiples_of_45_cannot_detect_the_missing_conversion`
與 `..._does_detect_...`）。

### 2. 本專案有 **三套** 掃描角慣例，同一顆滑桿在不同規劃器裡代表不同方向

已整理成 `core/geometry/polygon.py` 尾端的轉換函式與註解：

| 慣例 | 誰在用 | → 羅盤 |
|---|---|---|
| 羅盤（0=北、順時針） | `optimal_scan_angle` / `alpha_k_deg` / VTOL 偵察面板 | 不用轉 |
| 自東順時針 | `CoverageParameters.angle`（`CoveragePlanner`） | `angle + 90` |
| 自北**逆**時針 | `CoveragePathPlanner.plan_coverage(scan_angle_deg=)` | `−angle` |

⚠️⚠️ 後兩者吃的是**同一個** `flight_params['angle']`。固定翼在單區域與多區域下
**同一個滑桿值會飛出不同方向的掃描線**（angle=30 → 單區域羅盤 330、多區域 120）。
這是既有的規劃器歧異，本次沒有動它（超出任務範圍），只讓圍籬跟著**各自那條路徑
實際飛的方向**走。要收斂的話兩處轉換要一起改，
`test_the_two_planner_conventions_really_do_disagree` 會在有人統一時變紅。

### 3. 圍籬建構失敗時會留著**上一趟**的圍籬

`_auto_build_fence` 的 `except` 只記一行 warning 就 `return`，**沒有**清掉
`_last_fence_bundle`（航點不足那一支有清）。於是建構失敗時兩張圖繼續畫著上一趟的
矩形，而「上傳電子圍籬」會把那個舊矩形送上飛控，使用者完全看不出來。已修＋上鎖。

---

## 交接文件裡**不成立**的一件事

反例組不要用那個三角形。`(-300,-250) (350,-250) (0,300)` 的
`minimum_rotated_rectangle` 有**兩個面積差 0.02% 的候選**（651.9×548.5 對
650×550），經緯度往返造成的次公尺誤差就足以讓 shapely 換一個選，
長邊方向從羅盤 147.5° 跳到 **28.7°** —— 後者離掃描角 30° 只有 1.3°，
反例組整個失效。

改用「長軸在羅盤 75° 的 4:1 細長作業區 + 手動掃描角 30°」：
MRR 給 75°、對齊掃描角給 30°，差 45°（模 90 之下的最大值），而且 4:1 的長寬比
讓 MRR 方向沒有懸念。

---

## 改了哪些檔

| 檔案 | 改動 |
|---|---|
| `mission/geofence_manager.py` | `build(scan_angle_deg=)`＋第三種 method `'scan'`；`scan_angle_to_math_deg` / `_resolve_mode` / `_scan_aligned_rect`；矛盾組合（mrr+角度、scan 無角度、非有限角度）一律拋 `GeofenceError` 不做仲裁 |
| `core/geometry/polygon.py` | `coverage_angle_to_compass()` / `fw_scan_angle_to_compass()` ＋三套慣例的說明 |
| `ui/widgets/dual_map_widget.py` | 五個 `display_*` 與 `_auto_build_fence` 加 `scan_angle_deg`（預設 `None`＝維持軸對齊）；建構失敗改成清掉舊圍籬 |
| `ui/main_window.py` | 八個呼叫點接線；只有**真的照掃描角排線**的分支才給角度；`_generate_grid_with_coverage_planner` 回報自己實際用的角度（它內部那條退路會換慣例） |
| `ui/widgets/parameter_panel.py` | `set_parameters` 對不認得的 key 發警告（原本靜默丟棄還印成功） |
| `scripts/_full_flow_capture.py` | 拿掉沒有定義的 `"overlap"` key；讀回改用 `get_dccpp_params()["overlap_rate"]` |
| `README.md` | 圍籬段落補上 SCAN 模式 |

新增測試：`tests/test_fence_scan_angle.py`（65 條）、
`tests/test_parameter_panel_unknown_keys.py`（5 條）。
全庫 **4055 綠 / 227 skip**，基準線 3985，零回歸。

**突變驗證 22/22 全咬**（忘了 90− 轉換、符號鏡射、轉錯方向、忘了轉回來、
buffer 做在錯框架、scan 悄悄退回 aabb、scan 換成 mrr、沒角度也偷偷轉、
矛盾組合不再拋錯、各層不再往下傳角度、失敗時留著舊圍籬、兩套慣例被統一…）。

---

## 順帶：`overlap` 那個小問題的真正答案

不是 setter/getter 的 key 對不起來 —— **整個 repo 根本沒有 `overlap` 這個 key**。
`set_parameters` 的迴圈是 `if key in self.parameters:`，沒有 else，最後卻無條件
印「參數已設置」⇒ 拼錯的 key 靜默 no-op 加一行成功訊息。

⚠️ 而且**不要**幫它猜一個對應。「重疊率」在本專案是三個不同的量：
`reduce_overlap`（bool，互補掃描）、`overlap_rate`（比例 0.0–0.5，**橫向**條帶
間距）、`*_overlap_percent`（百分比，相機**航向**重疊）。裸 `overlap` 一個都不是；
硬塞進 `dccpp_overlap_spin` 等於自己決定它是哪一個軸向。

---

## 還沒解的（下一個 session）

1. **`vehicle_type` 設不進去**（同一個呼叫裡的第二個靜默 bug）：
   `set_parameters` 認得這個 key、也寫進了 `self.parameters`，但**沒有對應的
   UI 分支**去更新 combo box，而且 `set_parameters` 從不 emit `parameters_changed`
   ⇒ `main_window.current_vehicle_type` 完全不會變。擷取腳本讀回的是它自己剛寫進去
   的那個 dict，所以印出 `載具=multirotor` 卻沒有任何東西真的切換。
2. **`reduce_overlap` 沒有出口**：不在 `main_window` 的 `flight_keys` 白名單裡，
   也不在 `flight_params` 裡 ⇒ 勾了那個 checkbox 對下游零影響。
3. **固定翼單區域 / 多區域掃描方向不一致**（上面第 2 點）—— 要不要統一是產品決定。
4. `display_swarm_coverage` 全 repo **零呼叫者**，是死路徑；本次仍照樣加了參數以保持
   五個入口一致，但它沒有任何測試能覆蓋（因為叫不到）。
