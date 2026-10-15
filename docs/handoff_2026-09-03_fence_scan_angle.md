# 交接：圍籬方向要跟掃描角一致

2026-09-03 · 分支 `quad_swarm`

---

## 下一個 session 要解的事

**電子圍籬目前是軸對齊矩形（正南北 / 正東西），不跟著航線的掃描角轉。**
掃描線斜著走的時候，圍籬會比實際需要的大很多 —— 看起來很多餘。

已量過的浪費（`scripts/_fence_angle_probe.py`，矩形作業區 8 條掃描線）：

| 掃描角 | 目前 aabb 面積 | 對齊掃描角 | 倍數 |
|---|---|---|---|
| 0° | 252 000 | 252 000 | 1.00× |
| 30° | 484 268 | 252 000 | 1.92× |
| **45°** | **520 200** | **252 000** | **2.06×** |
| 60° | 484 268 | 252 000 | 1.92× |
| 90° | 252 000 | 252 000 | 1.00× |

---

## 根因（已定位到一行）

`ui/widgets/dual_map_widget.py:592`

```python
mgr = GeofenceConstraintManager(buffer_radius_m=buffer_m)
bundle = mgr.build(waypoints=flat)
```

沒傳 `method`，所以吃預設值 —— `mission/geofence_manager.py:183` 的
`method: str = "aabb"`。`"aabb"` 走 `mp.envelope`（軸對齊包絡），
見 `geofence_manager.py:362-365` 與 `286-296`（buffer 之後又收斂一次）。

---

## ⚠️⚠️ `method="mrr"` **不是**修法

程式裡已經有 `"mrr"`（`minimum_rotated_rectangle`），改一個字就能跑，
但**那是錯的**，因為它解的是另一個問題：MRR 求的是「面積最小」，
不是「對齊掃描角」。這兩件事在非矩形作業區會分岔。

實測（三角形作業區 `(-300,-250) (350,-250) (0,300)`，掃描角 30°）：

```
aabb 332 717   mrr 323 282   對齊掃描角 330 000
mrr 長邊方向 = 122.4°        ← 掃描角是 30°，差了約 92°
```

MRR 的面積確實比較小（少 2%），但它幾乎**垂直於**航線。
使用者要的是跟航線一致，不是面積最小。

> 注意這個三角形案例的 `aabb/scan` 只有 1.01× —— 浪費倍數同時取決於
> **作業區形狀**與**掃描角**，不是只看角度。矩形作業區 + 斜掃描角才是最糟的組合。

---

## ⚠️⚠️⚠️ 這個 bug 在 0° / 90° 完全看不出來

掃描角 0° 或 90° 時，軸對齊矩形**恰好等於**對齊掃描角的矩形（上表兩端都是 1.00×）。

**今天那趟 AeroPlan 完整流程的掃描線就是東西向**，所以截圖上看不出問題
（`runs/artifacts/aeroplan_flow/shots/04_dccpp.png`）。

⇒ 任何驗收測試如果用軸對齊的掃描角，會**恆綠**，而且看起來像測到了。
這正是本專案記錄過的「儀器不可能觸發」那一類。測試一定要用
30° / 45° 這種非軸對齊的角度，並且先斷言「這個設定真的能讓兩者不同」。

---

## 掃描角在哪裡拿得到

兩條規劃路徑都算得出掃描角，但**都沒有傳到圍籬那一層**：

| 來源 | 位置 |
|---|---|
| DCCPP | `mission/swarm_coordinator.py:1342` 產生 `effective_scan_angles`（dict，key 是 uav id）；`ui/main_window.py:5213` 讀 `.get(1)` |
| 基本演算法 | `core/geometry/polygon.py:454` `PolygonUtils.optimal_scan_angle(polygon)` |
| DCCPP 內部 | `core/dccpp/area_processor.py:443` `scan_angle_deg` 參數 |

`GeofenceConstraintManager.build()` 的簽名在 `mission/geofence_manager.py:233`，
目前只收 `waypoints / takeoff / targets / raw_result` —— **沒有角度參數**。

---

## 建議的修法

1. `GeofenceConstraintManager` 新增第三種 method，或給 `build()` 一個
   `scan_angle_deg: Optional[float]` 參數。
2. 有角度時：把點集旋 `-θ` → 取 `envelope` → buffer → 旋回 `+θ`。
   （`scripts/_fence_angle_probe.py` 的 `rect_aligned()` 就是這個做法，14 行。）
   ⚠️ buffer 要在**旋轉後的框架裡**做，否則 `286-296` 那段又會把它收斂回軸對齊。
3. `dual_map_widget.py` 的 `_rebuild_fence`（約 575-598）目前只拿到 `paths_list`
   與 `altitude`，要把掃描角一路傳進來。
4. 沒有角度時（例如手繪圍籬、匯入的航線）**維持現行 aabb**，不要猜。

---

## 驗收判準（不要用「看起來比較貼」）

- 值層級：`圍籬長邊方向` 與 `掃描角` 的夾角 < 1°（模 180°）。
- 面積：45° 掃描角的矩形作業區，面積要從 2.06× 降到 1.0×（±2%）。
- **對照組**：0° 掃描角時面積與方向都**不可以變**（證明沒有亂轉）。
- **反例組**：三角形作業區 + 30° 掃描角，要能分辨出
  「對齊掃描角（30°）」與「MRR（122.4°）」是不同的答案 ——
  只測矩形作業區的話這兩者永遠一樣，測了等於沒測。
- 突變驗證：把新方法改回 `envelope` / 改成 `mrr`，上述測試都要變紅。

---

## 順帶：一個沒解的小問題

`ParameterPanel.set_parameters({"overlap": 30.0})` 設不進去 —— 設定後
`get_parameters()` 讀回 `overlap` 是 `None`。setter 與 getter 的 key 可能對不起來。
今天那趟流程不受影響（重疊率有自己的預設），但值得順手查。

---

## 今天這個 session 做完的事

**WSL / ROS 側（`swarm_ws` @ `quad_swarm`，三個 commit）**

- `df6d0fd` 路徑帶逐點高度，爬升段納入防撞判斷
- `2ecb92a` CLIMB 階段：先散開再爬到進場層才開始進場
- `9e10dfb` + `c68197b` 僚機真的會為長機讓行（優先權撞號 + 長機不在 `gcs/sub` 上，兩個獨立的洞）
- 離線 222 綠、突變逐條驗過、純 SITL 與 Gazebo 各跑完一次完整流程

**AeroPlan 側（未 commit）**

- `scripts/_capture_probe.py` 擷取可行性探針
- `scripts/_full_flow_capture.py` 完整流程腳本化 + 錄影截圖（10/10 階段通過）
- `scripts/_encode_frames.py` PyAV 編碼（⚠️ 這台機器沒有 cv2，有 PyAV 18.1.0）
- `scripts/_fence_angle_probe.py` 本文件的量測來源
- 產物在 `runs/artifacts/aeroplan_flow/`

⚠️ 腳本化跑 GUI 會被**模態對話框**卡死，已知三個來源都要擋：
`QMessageBox`（129 個呼叫點）、`SITLLaunchDialog.get_config()`、
`QFileDialog.getSaveFileName`。擋的時候要**記錄**，靜默的自動回答會讓
「跳過了一個本來要人決定的決定」看起來像成功。

⚠️ 而且「沒有丟例外」**不等於**成功 —— 這個 App 大量把錯誤轉成對話框然後
`return`。今天踩到兩次：DCCPP 查錯 key 恆報 0、蜂群打擊跳「目標未設定」卻記成完成。
`_full_flow_capture.py` 的 `stage()` 現在會把期間跳出的 warning/critical 算成失敗。

**架構圖**：https://claude.ai/code/artifact/0229d9b5-d95c-4fea-b559-967ee0d850c6
