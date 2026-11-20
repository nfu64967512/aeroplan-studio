# HARVEST 交接說明（2026-09-10）

> **給下一個對話。**
> 位置：`C:\Users\joy46\Documents\Projects\aeroplan-studio`，分支 `quad_swarm`，
> HEAD `5e0295f`（working tree **dirty**，本 session 的改動都還沒 commit）。
> **紀律**：本文只寫查得到出處的東西。每個數字後面都有腳本或產物路徑。
> 被自己的量測推翻的結論**留著並標明推翻**，不刪 —— 那些是最貴的資訊。

---

## 0. 三十秒摘要

| | |
|---|---|
| 這一輪主線 | 走廊豁免的安全缺陷（vendored #8）→ 實飛驗證 → 先行技術 → 三篇全文 → 封閉基準 |
| 交付 | 1 個安全修正（已實飛驗證）、1 個被自己否決的優化、**4 份新文件**（先行技術／貢獻主張 v3／三篇全文／封閉基準） |
| 回歸 | `tests/hv_dccpp/` **846 passed**（2026-09-10 於**關掉 §2 那個改動之後**重跑確認，12m52s） |
| ⚠ 樹上有紅的 | `tests/test_dccpp_strip_pitch.py` **11 failed**（`core/dccpp/`，**不是**本 session 造成，已開 task chip） |

**最重要的一句話**：這一輪有**三次**「我先下結論、後被自己的量測推翻」。
接手的人請先讀 **§4（被推翻的結論）**，那比 §1 的成果更有用。

---

## 1. ✅ 做完且已驗證的：走廊豁免改比站點身分（vendored #8）

### 缺陷

`deconflict._pair_metrics` 的起降走廊豁免比的是場坪**座標**（≤ `SAME_PAD_TOL_M` = 50 m），
而 `CorridorSchedule` 的互斥是以 **`site_id` 字串**為鍵。兩者不一致 ⇒
**兩個相距 < 50 m 的不同站**被豁免，卻不會被排進同一條隊列。

逐樣本清點（4 台、每機一個起飛站）：

| 起飛站間距 | 被豁免的真違規 | 真同站 | **不同站** |
|---|---|---|---|
| 200 / 100 m | 0 | 0 | 0 |
| 60 m | 496 | **0** | **496** |
| 30 m | 1037 | **0** | **1037** |
| 15 m | 1062 | **0** | **1062** |

受害樣本 **100% 是不同站** ⇒ 座標 proxy 沒帶來任何好處。

### 修法與實飛驗證

`UAVPlan.takeoff_site`（`landing_site` 的對稱項）+ 豁免改比身分。
五趟四台 SITL（全部 4/4 落地、覆蓋 100%）：

| 航次 | 建置 | 起降點 | 自動指派 | 實飛最小水平／垂直 | 違規 |
|---|---|---|---|---|---|
| PAD8_CTRL | 修好 | 共點 | 開 | 151.6 / 28.2 m | 0 / 316 |
| PAD8_SPREAD | 修好 | 10 m × 4 | 開 | 148.8 / 61.3 m | 0 / 1077 |
| PAD8_BROKEN | **壞的** | 10 m × 4 | 開 | 149.3 / 60.7 m | 0 / 1080 |
| PAD8_FIXED_RAW | 修好 | 10 m × 4 | **關** | 52.7 / 112.6 m | **0** / 346 |
| PAD8_BROKEN_RAW | **壞的** | 10 m × 4 | **關** | **5.0 / 19.9 m** | **5** / 378 |

⚠⚠⚠ **前三趟分辨不出修法** —— 回收站自動指派把兩台移到 ALT1，兩臂收斂到同一份計畫。
受控實驗必須 `HV_RECOVERY_AUTO=0`。**A/B 兩臂結果一樣時，先問是不是被下游抹平。**

守衛：`tests/hv_dccpp/test_corridor_exemption_identity.py`（6 條）、突變 **8/8**。
文件：`docs/hv_dccpp/VENDOR.md` 的 vendored #8。

### 順帶做完的

* `per_uav_takeoff_sites` 預設開回 True；SITL 的 `--home` 逐機讀回規劃的起飛站
  （`hv_sitl_run._home_of`），起飛前比對誤差 ≤ 2 m，不符就停。
* `spawn_positions` 從 `ui/main_window.py` 搬到 `utils/math_utils.py`
  （SITL 路徑不碰 Qt），`ui.main_window` 再匯出，搬移前後逐位元相同。
* 新情境 `gui_legal_120_split_spread10`（HV4SITL14）：四台四個起降點、間距 10 m。
* `hv_sitl_report._min_separation` 補上**垂直**與違規計數 —— 原本只算水平，
  而判準是水平 < 100 **且**垂直 < 50，只看水平判斷不了。

---

## 2. ❌ 做了又關掉的：`allocate.sweep_order` 接搜尋序

`decompose.exact_time` 早就搜尋條帶序（vendored #2），但 `allocate.sweep_order`
無條件用固定 skip-k 序 ⇒ `T_single` 是搜尋序的成本、`_chain` 的端點卻是固定序的。

接上之後 **hills 一個情境** makespan 5087.4 → 5037.6（−49.8 s），我就寫進文件說站得住。
**封閉基準 24 格否決了它**：淨 **+731 s**，其中 `HARVEST_mix S1 n=8` **+10.70%**、
`HARVEST_mix S4 n=8` **+13.17%**，其餘 18 格 0.00%。

⇒ `_chain` 已改回**不傳 cfg**。介面、成本矩陣的值層級鎖與 8 條測試都保留，
`test_chain_deliberately_uses_the_fixed_order` 把「刻意不用」這個**決定**鎖住，
斷言訊息裡帶著那些數字。**要重開就先重跑那 24 格。**

---

## 3. 📄 論文線（**五份文件互相有訂正關係，讀的順序有差**）

⚠ 由上而下讀。下面的會訂正上面的，**不要只讀其中一份**。

| 文件 | 內容 | 狀態 |
|---|---|---|
| `docs/hv_dccpp/論文_貢獻主張_v2.md` | 2026-09-08 的四項主張 | ⚠ **C1/C2 已失效**，檔頭有指向 v3 的告示 |
| `docs/hv_dccpp/論文_先行技術_2026-09-09.md` | 七角度網搜 + 對抗查證（**36 筆全部確認存在、零捏造**）+ NotebookLM 逐字追問 | 現行 |
| `docs/hv_dccpp/論文_貢獻主張_v3_訂正.md` | 三項主張的訂正寫法 | ⚠ **C1 已被二次訂正**，見下 |
| `docs/hv_dccpp/論文_三篇全文閱讀_2026-09-10.md` | Xiao / Coombes / Gong 全文 + 回原始碼查證 | 現行 |
| `docs/hv_dccpp/封閉基準_2026-09-10.md` | 效率矩陣重跑 | 現行 |

### 現在還站得住的主張（四項）

1. **異質轉彎半徑當分配驅動量** —— 唯一完全沒被碰過的。每篇迴轉感知規劃器都假設全隊同一半徑。
2. **VTOL 相位／轉換高度當規劃變數** —— arXiv 上 `"coverage path planning"` 121 篇、
   加 `AND VTOL` **0 篇**。
3. **起降走廊當逐場坪互斥資源** —— 34 篇裡「沒有找到」；而 §1 的實飛證明它是**安全相關**的。
4. **覆蓋規劃器內的度量式 4D 隔離 + 成長時間不確定度** —— 最接近的 Kusnur 是固定 `d_min`、
   同高度、以停止動作修復。

### ⚠⚠⚠ 不可以再說的話

* 「`⌈2R/s⌉` 是我們的」—— 逐字等於 Huang 2022 的 span coefficient `λ = ceil(2R_min/w)`。
  **要當出處引用，不是相關工作。**
* 「`λ_floor` 是下界」—— **它不是**。`T_single` 含 (n−1) 次迴轉，拆給 λ 架後只剩 (n−λ) 次
  ⇒ 可能大於真正最小值。是**保守配額規則**。Gong 的 `K_lower = L_in/L_max` 才是合法下界。
* 「閉式機數下界只在點服務設定」—— 面覆蓋裡就有（Gong 2026）。
* 「分離的起飛站／降落站在覆蓋文獻零篇」—— Yakunin 2025 有（沿道路的移動地面平台）。
  我們剩下的是「固定多場坪 + 逐機指派 + **走廊互斥**」。
* 「垂直於風是 Coombes 證明的最佳方向」—— 只對**掃描直線段**成立。加進 trochoid 轉彎時間後
  最佳角在 255–265°，不是 270°。

---

## 4. ⚠⚠⚠ 被自己的量測推翻的結論（**這一節最有用**）

| 我先寫的 | 實情 | 怎麼發現的 |
|---|---|---|
| 「hills −49.8 s ⇒ sweep_order 改動站得住」 | 封閉基準 24 格淨 **+731 s** | 一個情境不足以判定配置層改動 |
| 「λ_floor 是把 Gong 的下界收緊」 | λ_floor **不是下界** | 讀 Gong 全文 |
| 「我們的估計器用固定 skip 序」 | **早就改成搜尋**（vendored #2） | 回原始碼查證 |
| 「Xiao 低估轉彎」 | 在**他們自己**的參數下是**高估** 20–47% | 用他們的 d 與 2R 實算 |
| 「估計器轉彎時間對方向盲是缺陷」 | `proxy_time` 只是預篩，`exact_time` 已傳風向量 | 查證代理實測 0.1% 訊號 |
| 「50/50 對稱化刪掉不對稱」 | 那是兩條**可達**鏈的精確算術平均 | 查證代理讀 `_pair_poses` |
| 「覆蓋文獻零篇做分離起降點」 | Yakunin 2025 有 | NotebookLM 逐字 + 索引全文核對 |

⚠ **共同的錯誤模式**：查證還沒跑完（15 個只回來 3 個）就先寫結論，5 條裡錯 2 條。
**等 `agents_done` 到齊再寫。**

---

## 5. 🔴 open：接手的人可以直接做的

### 5.1 `core/dccpp/` 的重疊率值層級鎖是**紅的**（已開 task chip，使用者已啟動）

```bash
python -m pytest tests/test_dccpp_strip_pitch.py -q
```
**11 failed / 7 passed**，最關鍵那條**方向是反的**：「重疊率 20% 沒有讓條帶變密（20 → 17）」。
記憶裡記著這個缺陷「已修 + 值層級鎖」，現在不只失效是**反向**。
⚠ 不是本 session 造成（打的是 `core/dccpp/` 不是 `hv_dccpp/`，那三個檔 mtime 09-06），
也不是 API 漂移（`w_ovl` 仍是公尺、仍檢查 `[0, w_i)`）。

### 5.2 封閉基準的兩格覆蓋不足（**不可進論文**）

`HARVEST S1 n=2` 覆蓋 **0.5277**、`S5 n=8` **0.617**（mix 對應 0.5238 / 0.5096）。
與 2026-09-07 封存表逐位相同 ⇒ 既存問題。
README 明文：覆蓋 < 0.99 時其餘指標無意義。

### 5.3 `HARVEST_mix S5 n=16` 規劃逾時

`TIMEOUT>1800s`；同組 `HARVEST S5 n=16` 用 **1757 s**，就在門檻邊緣。

### 5.4 付費牆擋住、投稿前要補的文獻

**AIAA SciTech 2018-0894**《Heterogeneous, Multiple Depot Multi-UAV Path Planning for
Remote Sensing Tasks》標題直接命中我們的功能 1+6，**最該先弄到手**。
其餘：Coombes 2020 JFR、TR-C 三篇 4D 衝突解消、Sundar & Rathinam JIRS。

### 5.5 量完決定**不做**的（不要重新提案）

* **消費 `cell.alt_dirs`**：第 2 名／第 1 名的胞內時間比中位 **1.107**，只有 **3/12** 在 2% 以內。
  已把「目前沒有任何消費者」寫進 `hv_dccpp/geometry.py` 的欄位說明。
* **`phase3` 的 2-hop 接力**：在下降已停住的狀態枚舉，hills 8 個候選**通過 0 個**。
* **加轉彎成本的值層級鎖**：已經有三層。

---

## 6. 工具與證據在哪

| 東西 | 路徑 |
|---|---|
| 封閉基準驅動 | `research/cpp_benchmark/rerun_project_rows.py`（`JOBS=6` 約 1 小時） |
| 基準結果（現行樹） | `research/cpp_benchmark/results/project_rows_5e0295f5_dirty.csv` |
| 基準結果（改動前對照） | `research/cpp_benchmark/results/harvest_before_sweeporder.json` |
| 三組數字比較 | `scratchpad/bench_compare.py` |
| SITL 航次 | `python scripts/hv_sitl_run.py fly <情境>`；`HV_TRIAL` 指定產物目錄 |
| 航次報告 | `python scripts/hv_sitl_report.py <情境>` |
| 突變實驗 | `scratchpad/pad_identity_mutations.py`、`scratchpad/sweep_order_mutations.py` |
| NotebookLM 筆記本 | `HARVEST 先行技術：異質 VTOL 覆蓋規劃`（`0acd4e8e`），34 篇外部論文 |

### ⚠ 讀基準結果的兩個坑

1. `project_rows_*.csv` 第一行是 `# provenance: {...}`，直接 `DictReader` 會把它當欄位名
   ⇒ 每列 method/scenario 全 None，**不會報錯**。
2. 依**字母**排序會讓 09-08 的 `project_rows_c56...` 蓋掉今天的 `5e0...`（`'5' < 'c'`）
   ⇒ 取 mtime 最新。

### ⚠ SITL 的兩個坑（已修，但要知道）

1. **串流速率參數改名**：這版韌體只有 `MAV1_`～`MAV3_`，**沒有任何 `SR*`**。
   名字錯了唯一的徵狀是讀回 `None`。
2. **取樣率的瓶頸在讀取端不在載具**：`read_telemetry` 等 `EXTENDED_SYS_STATE`，
   拿不到就跑滿逾時 ⇒ 四條鏈路固定 1 Hz 牆鐘 = 5 s 模擬。
   三趟不同串流設定都拿到**剛好 164 筆**才看出來。現在 `HV_POLL_S=0.05` ⇒ 1.00 s。

---

## 7. 未 commit

本 session 的改動**全部還在 working tree**（HEAD `5e0295f` dirty）。
主要生產檔：`hv_dccpp/{allocate,deconflict,mission,planner,dynamic,geometry,ardupilot}.py`、
`hv_dccpp/integration/aeroplan_adapter.py`、`scripts/hv_sitl_{run,report,scenarios}.py`、
`utils/math_utils.py`、`ui/main_window.py`。
新測試：`tests/hv_dccpp/test_corridor_exemption_identity.py`、`tests/hv_dccpp/test_sweep_order_search.py`。

⚠ `core/dccpp/` 的三個檔在 09-06 就是 modified 狀態，**不是本 session 動的**，
task chip 那個 session 正在處理，commit 前要跟它協調。
