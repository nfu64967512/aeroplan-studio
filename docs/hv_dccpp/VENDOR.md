# hv_dccpp 是 vendored 進來的（不是外部相依）

**來源** `C:/Users/joy46/Documents/Projects/vtol演篹法`　**版本** 0.1.0　**日期** 2026-09-06

原本 AeroPlan 是用 `pip install -e "<那個資料夾>"` 把 `hv_dccpp` 當外部套件引用，
程式碼住在 AeroPlan 之外。依使用者決定改為 **vendor**：整份複製進本專案，
`pip uninstall hv_dccpp` 之後 `import hv_dccpp` 解析到 `aeroplan-studio/hv_dccpp/`。

## ⚠ 名稱對照：HARVEST vs `hv_dccpp`

2026-09-09 起演算法一律叫 **HARVEST**（Heterogeneous Allocation and Routing for
VTOL Endurance-constrained Survey Tasks，見 `docs/adr/0010-harvest-naming.md`）。

| 名字 | 指的是 |
|---|---|
| **HARVEST** | 本專案對這套方法的名稱。論文、面板文案、log、我們自己的程式與文件一律用它 |
| `hv_dccpp` | **上游套件名**。目錄、import 路徑、`HvDccppCoordinator` 一律**維持原名** |

⚠ 改套件目錄名等於從上游分叉，下面那套重新同步程序就作廢了 ⇒ **不要改**。

⚠⚠⚠ 同理，跨進 `integration/aeroplan_adapter.py` 的四個參數鍵
（`hv_cfg`、`hv_class`、`hv_dccpp`、`hv_uav_id`）也**不可以**改名。
面板送進來的其餘 `hv_*` 鍵由我們自己的 `swarm_coordinator._hv_get` 讀，
只改一邊會讓全部參數**靜靜退回預設值** —— 那是「有效值契約」明文禁止的靜默失敗。

## 搬進來的東西

| 位置 | 內容 |
|---|---|
| `hv_dccpp/` | 函式庫本體（26 個模組，含 `integration/aeroplan_adapter.py`）。**可 import 的套件**，與 `core/` `mission/` `ui/` 同層 |
| `tests/hv_dccpp/` | 函式庫自己的 809 項測試（跑一次約 72 s） |
| `tools/mock_autopilot.py` | 以 pymavlink 寫的最小 QuadPlane 模擬器（`tests/hv_dccpp/test_mavlink_link.py` 會啟動它） |
| `tools/hv_dccpp/examples/` | 5 支示範腳本 + quickstart |
| `docs/hv_dccpp/` | 設計文件、AEROPLAN 接入手冊、平台參數表、規格書、模組契約、進度紀錄、上游 pyproject |

**沒有搬**：`figures/`（7.7 MB 產生出來的圖）、`document_pdf.pdf`（7.9 MB 論文原文）、`*.rar`。
都是可重新產生或外部取得的資產，放進 git 只會讓 clone 變重。原始資料夾**沒有刪**，
它現在的角色是「上游開發用的副本」。

## vendored 修改（原本只有 1 條，2026-09-09 起 8 條）

⚠ **#5／#6／#7 目前只有原始碼內的註解，這份文件還沒補**：
#5 = `HvBuiltWaypoint.sensor_on`（覆蓋率驗算要知道相機開沒開）、
#6 = `ardupilot.py` 航點精簡不可弄丟條帶終點、
#7 = 逐機起飛站（n 台 ⇒ n 個起降點）。同步上游時**它們一樣要重新套上**。

### #1（測試路徑）

`tests/hv_dccpp/test_mavlink_link.py`：`ROOT` 原本是 `Path(__file__).parents[1]`
（上游測試就放在 repo 根的 `tests/`）。搬到 `tests/hv_dccpp/` 之後那樣會指到 `tests/`，
找不到 `tools/mock_autopilot.py` 就**整條 skip** —— 而 skip 在報表上跟通過長得一樣。
改成往上搜尋含 `tools/mock_autopilot.py` 的那一層，兩種佈局都對。

~~套件本體（`hv_dccpp/*.py`）**一行都沒有改**~~ —— 2026-09-08 起不再成立，見 #2。
（#1 本身仍然只動測試檔，沒有 `__file__` / 相對路徑假設。）

### #2（估計層的掃描序，2026-09-08）⚠⚠⚠ 這條動到套件本體

**問題**：估計層（`decompose.exact_time`、`allocate.sweep_order`）用**固定**的
skip-k 序列，而規劃層（`planner` 的 IDP beam，K=6/B=3）是**搜尋**的。固定序列每換一個
skip 類別就要「繞回去」一次，超出理想值的量是 `(k−1)·max(0, n−2k)·s` —— **對條帶數
二次**。`exact_time` 決定 `cell.T_single_s`，往下餵三件事：掃描方向的最終排序、
`λ_floor`、配置階段的 makespan ⇒ 估計器系統性悲觀，而且**系統性懲罰條帶數多的方向**。

**改了什麼**

| 檔案 | 改動 |
|---|---|
| `hv_dccpp/mission.py` | `PlannerConfig` 新增 `sweep_sequence: "fixed"｜"search"`（預設 `search`）與 `sweep_exact_max_n: int = 9`；`LEGACY_OVERRIDES` 加 `"sweep_sequence": "fixed"` |
| `hv_dccpp/decompose.py` | 新增 `_seq_candidates` / `_held_karp_path` / `_best_sequence`；`exact_time` 改為建 n×n 轉場成本矩陣（單次 `dubins_all_times`）後對候選集取 argmin；新增 `import itertools` |
| `tests/hv_dccpp/test_mission.py` | `EXPECTED_DEFAULTS` 補上那兩個欄位（它們**不在規格 §16.2**，是本專案的偏離） |

**不可能退步**：候選集第一個就是原本的 `_skip_sequence(n, k)`，且所有候選用**同一個**
成本矩陣評分 ⇒ argmin ≤ 固定序列。已逐格斷言（14 組 n×構型，零退步）。

**可完整退回**：`PlannerConfig(sweep_sequence="fixed")` 逐位元回到上游行為；
`legacy_mode=True` 也會強制 `fixed`（否則 D32 的「一鍵退化回基準線」不成立）。

**實測改善**（純幾何 L_min 成本，A: R=200/s=168.05、B: R=120/s=100，k=3）：

| n | A 改善 | B 改善 | 對 Held–Karp 的 gap |
|---|---|---|---|
| 6 | 2.43% | 3.04% | 0.00% |
| 10 | 9.54% | 9.18% | 0.00% |
| 14 | 18.45% | 18.20% | 0.00% |
| 16 | 19.45% | 19.22% | — |

14 組裡 12 組與最佳解差 **0.00%**，最差 1.56%。

⚠ **沒有改** `allocate.sweep_order`：它的輸出被 `_task_options` 用來依**奇偶**決定
進入／終端側（`n 奇 → idx_last = idx_in`），而折疊蛇形會改變那個奇偶結構。
要改必須連同 `_task_options` 一起重新推導，尚未做。

⚠ **這條會讓舊的規劃數字失效**：`T_single` 變小 ⇒ λ_floor、方向排序、makespan 都會動。
`research/cpp_benchmark/results/` 裡的專案列必須重跑。

**同步上游時**：#2 要重新套上，或先確認上游已自行處理估計／規劃兩層的不一致。


### vendored 修改 #3：全隊起飛時刻全域重排（2026-09-08）

**檔案**：`hv_dccpp/deconflict.py`（新增 `scan_done_s`、`_objective_of`、
`_fresh_schedule`、`_global_reschedule`，並在 `repair()` 收尾處呼叫）、
`hv_dccpp/mission.py`（`PlannerConfig` 新增 4 個欄位）。

**為什麼**：`repair()` 是逐衝突貪婪（`_repair_one` 一有候選通過就 return），
收尾的 `_compact_makespan` **一次只平移一機**（`involved_score(new, {u})`）
⇒「A 晚一點起飛好讓 B 早很多」這種協同平移**結構上做不到**。

實測 HV4SITL9（1 Pioneer + 3 HERO、一大區切 4）：起飛被排成
0 / 289.1 / 623.9 / **1265.6** s，佔 makespan 的 **70.3%**。而驅動它的 6 筆衝突
**全部**是同一個場坪 H 的**降落**走廊排隊（`kind='corridor'`、
`res_kinds=('landing','landing')`、d_h = d_v = **0.0**）—— 沒有一筆是空間衝突。
起飛被押後，是為了解 500 s 後才發生的降落坪衝突。

**做法**：對可動機的每一種起飛順序（n ≤ 6 全枚舉，否則走 4 個啟發序），逐機插入
「最早可行」時刻，可行＝ 4D 分離 ＋ 走廊時刻表兩者皆與已擺放者相容。取目標函數最小者，
**套用前再跑一次全機對的完整驗證**；沒有候選全綠就原樣退回 ⇒ 不可能讓結果變差。

**效果**（生產 GUI 路徑，情境 `gui_legal_120_split`）：

| | makespan | 掃描完成 | 未覆蓋 | 起飛時刻 |
|---|---|---|---|---|
| 關掉（＝上游） | 1799.8 | 1594.6 | 0 | 289.1 / 623.9 / 0.0 / 1265.6 |
| 開啟 | **1450.8** | **1216.4** | 0 | 916.0 / 0.0 / 612.0 / 280.0 |
| | **−19.4%** | **−23.7%** | | |

航程、條帶數、航點數逐機**逐位元不變**（`shift_plan` 在 `energy_wh=0` 下只動時間戳）。
規劃耗時 1.77 s → 17.12 s。

⚠⚠ **平移不具不變性**：4D 判準是 `|t_a − t_b| < Δt + σ_a + σ_b`，而
`σ_t = min(k_t·(t − t_sync), SIGMA_T_MAX_S)` 是**絕對時間**的函數。全隊同步平移
（相對時距一格沒動）在 Δ=+300 s 就會多出 1 筆衝突。⇒ 不可以建「逐對可行集」那種
鬆弛，每個候選都要真檢查；也因此**提早起飛的改善有一部分來自 σ 變小**（模型餘裕），
不是純排程壓縮。要單獨衡量後者，把 `cfg.k_t` 設 0 再比一次
（實測單改 k_t=0 就有 −304.6 s，量級相當）。

⚠ **這條與「放寬降落緩衝」相消不相加**：先做重排再把 `t_sep_ldg_s` 90→30，
實測 1498.8 s，**比只做重排差 37.5 s**。不要兩個一起套。

⚠ 更大的單一槓桿其實是**第二個起降場坪**（同組態邊際 −451.2 s），但那需要真正接上
`UAVInstance.start_site_id`（轉接層目前 0 命中），且 `planner.append_rtb` 選降落站
只看能量、**完全不看走廊忙碌**（給 8 個備降點，四機照樣全落 H）。尚未做。

**關掉方式**：`PlannerConfig.global_reschedule = False` ⇒ 逐位元退回上游行為
（已由 `tests/hv_dccpp/test_global_reschedule.py` 的刪除實驗鎖住，含四個起飛時刻）。

**同步上游時**：#3 要重新套上。


### vendored 修改 #4：逐機回收站 + validate 起飛站修正（2026-09-09）

**檔案**：`hv_dccpp/mission.py`（`UAVInstance.recovery_site_id` 欄位 + 兩處驗證 +
`MissionInputs.recovery_site_id_of`）、`hv_dccpp/context.py`（`recovery_sites` 表 +
`recovery_site_id_of`）、`hv_dccpp/planner.py`（`_site_order(..., preferred=)` +
`append_rtb` 讀逐機偏好）、`hv_dccpp/preflight.py`（逐機回收站計入 primary）、
`hv_dccpp/sim.py`（**修既有 bug**，見下）。

**為什麼**：HV4SITL9 的 makespan 有 **70.3%** 是起飛錯開，而驅動它的 6 筆衝突
**全部**是同一個場坪 H 的**降落**走廊排隊（`kind='corridor'`、
`res_kinds=('landing','landing')`、d_h = d_v = 0.0），沒有一筆是空間衝突。
而上游只有**全域** `recovery_site_id`，加上 `_site_order` 把它無條件排第一、
`append_rtb` 取第一個能量可行者 ⇒ 實測**給 8 個備降點，四機仍 100% 落 H、
makespan 一秒沒動**。

**量到的槓桿差距**（生產路徑端到端，基準 1450.8 s）：

| 變體 | makespan | validate |
|---|---|---|
| 只分**起飛**站 | 1518.3（**更差**） | 有警告 |
| 只分**降落**站 | **1125.3（−22.4%）** | 通過 |
| 兩個都分 | 1224.9 | 有警告 |

起飛站分流的**精確天花板只有 −52.6 s（−3.6%）**，且 60 m 間距那組比基準差 41.8 s
（B3 被插 23 圈盤旋、航程 3.2 倍）⇒ **起飛站本輪不做**。

**做了什麼**：`UAVInstance.recovery_site_id`（選填，預設 None）。逐字鏡射既有的
`start_site_id`：欄位 → `__post_init__` 正規化 → `MissionInputs.__post_init__` 站點
存在性檢查 → `build_context` 建表並發 `per_uav_recovery_sites` 警告 → `append_rtb`
把它插到 `_site_order` 最前面。

⚠ **插到最前面而不是只回它**：`append_rtb` 取第一個**能量可行**者，所以偏好站吃不下時
仍會退回全域 recovery 與其他備降點 —— 指派不會把任務打死。反過來說，
**「宣告了偏好」不保證真的落在那裡**，要驗證得讀回 `UAVPlan.landing_site`。

**⚠⚠⚠ 順帶修掉一個既有 bug**：`sim.py` 的 `validate()` 原本寫
`home = ctx.inputs.home.site_id` 然後每一架都 `sched.reserve_plan(p, home)`，
**完全不看逐機起飛站**，而 planner 早就逐機登記 ⇒ 兩條管線用不同站點，
只要有人從別的場坪起飛，validate 就報**不存在的**走廊衝突（實測四組全部
`validate_failed(conflicts=2)`）。已改成用 planner 的同一支 `_start_site`。
⚠ `dynamic.py` 三處、`viz.py` 兩處有**同樣的寫死**，本輪未動（起飛站不在範圍內）。

**⚠⚠ 尚未做（這一條只給表達能力，不是完整功能）**：**自動指派規則**。
誰該去哪一站目前要由呼叫端決定，而實測顯示那是一張彩票：同一份幾何的 16 種指派
分布在 **921–1394 s**，其中 2 種 validate 不過；場坪位置同樣敏感
（400 m **驗證失敗**、500 m 間距低於基準、600 m 最好 926.8、800 m 掉掉六成效益）。
換一個相鄰幾何（HV4SITL8）建議指派只剩 −3.8%。

⚠⚠⚠ 而且自動指派**必須**帶兩道回歸守衛，因為量到**最快的指派同時是最不安全的**：
基準最近接 148.4 m、100 m 內 0 筆；最快那組 **51.9 m**、100 m 內 6 筆、
≥3 台同時在空中 265.9 s（基準 0）。而 `_action_key` 第一鍵是 makespan，會
**主動**選中它。能量同理：`energy_safe_check` 只保證 ≥ 保留量，換站被授權從
190.7 Wh 吃到剩 5 Wh 仍全綠 ⇒ 要**逐機對基準的回歸界**，不是地板。
另有二階陷阱：`planner.py` 的 `e_alt = min(point_wh … for s in sites)` 對**所有已宣告**
站取 min ⇒ 多宣告一個場坪就讓**全隊**（含從不換站的機）能量檢查變寬，
所以 A/B 對照組不能拿原廠單站當基準。

**關掉方式**：不設 `recovery_site_id` ⇒ 逐位元退回上游行為
（`tests/hv_dccpp/test_per_uav_recovery_site.py` 鎖住）。

**同步上游時**：#4 要重新套上。


### vendored 修改 #8：走廊豁免改比站點身分（2026-09-09）⚠⚠⚠ 安全性

**檔案**：`hv_dccpp/deconflict.py`（`_corridor_masks` / `_corridor_sites` / `_Track.site()`
/ `_pair_metrics` 的豁免判準）、`hv_dccpp/mission.py`（`UAVPlan.takeoff_site` 欄位 +
`to_dict`）、`hv_dccpp/planner.py`（`build_uav_plan` 蓋上起飛站）、
`hv_dccpp/dynamic.py`（重規劃沿用原起飛站）。

**缺陷**：兩層寫法不一致。

* `_pair_metrics` 的起降走廊豁免：兩機**場坪座標**相距 ≤ `SAME_PAD_TOL_M`（50 m）
  就當成同一站，不計 4D 衝突，理由寫在 `SITE_CORRIDOR_PHASES` 的說明裡 ——
  「該處互斥由 `CorridorSchedule` 保證」。
* `CorridorSchedule` 的互斥：以 **`site_id` 字串**為鍵（`is_free` 比 `r.site_id != site_id`）。

⇒ 兩個**不同**的站相距 < 50 m 時豁免成立，但排程根本不會把它們排進同一條隊列，
真違規就被靜靜丟掉。舊 docstring 明寫「不同站（相距數 km）」——在 #7（每機一個起飛站）
之前那是真的，所以 0–50 m 這一帶從來不存在，缺陷潛伏至今。

**怎麼發現的**：查「起飛站間距越小 makespan 越好，是不是保護失效」。
先確認那個排序是**非單調的雜訊**（1401 / 1177 / 1425 / 1259 / 1188 / 1190 s），
再逐樣本清點「被豁免掉的真違規」，發現失效線正好落在 `SAME_PAD_TOL_M` 上：

| 起飛站間距 | 被豁免的真違規 | 其中真同站 | 其中**不同站** | 最近的一筆（水平／垂直） |
|---|---|---|---|---|
| 全部同點 | 0 | 0 | 0 | — |
| 200 m | 0 | 0 | 0 | — |
| 100 m | 0 | 0 | 0 | — |
| 60 m | 496 | **0** | **496** | 30.0 m ／ 6.0 m |
| 30 m | 1037 | **0** | **1037** | 15.0 m ／ 3.6 m |
| 15 m | 1062 | **0** | **1062** | 7.5 m ／ 22.3 m |

門檻是水平 100 m／垂直 50 m，而 `validate` **每一組都回報通過**。
受害樣本 **100% 是不同站**，真同站一筆都沒有 ⇒ 座標 proxy 沒有帶來任何好處，
只帶來這個洞。最嚴重一筆是一機**起飛**、一機**降落**（B2 於 t=756 s 爬過 20 m，
B3 於 t=766 s 降到 14 m，水平 30 m）。

**做了什麼**：`UAVPlan.takeoff_site`（`landing_site` 的對稱項，預設 `""`）。
`_corridor_sites` 逐樣本給出站點身分（起飛段取 `takeoff_site`、降落段取 `landing_site`，
與場坪座標**共用同一組遮罩**），`_pair_metrics` 改成

```python
same_pad = np.where(known, sa_id == sb_id, near_pad)   # 身分未知才退回座標
```

⚠ 保留座標退回路徑是為了手搭 `UAVPlan` 的既有測試；**正式規劃路徑一律帶身分**，
由 `test_planner_always_stamps_the_takeoff_site` 鎖住。

**修前／修後**（同一組情境，4 台、1560×800 m）：

| 站點間距 | 最小水平間距 | makespan | 被豁免的真違規 |
|---|---|---|---|
| 全部同點 | 148.2 → 148.2 m | 1401.3 → 1401.3 s | 0 → 0 |
| 200 m | 17.9 → 17.9 m | 1177.3 → 1177.3 s | 0 → 0 |
| 60 m | 30.0 → **52.5 m** | 1259.0 → 1589.2 s | 496 → **0** |
| 30 m | 15.0 → **101.2 m** | 1188.3 → 1277.9 s | 1037 → **0** |
| 15 m | 7.5 → **101.6 m** | 1190.1 → 1279.0 s | 1062 → **0** |
| 10 m | 15.0 → **107.2 m** | 1192.0 → 1281.3 s | — → **0** |

⇒ 修好後 10 m 間距也合法（4D 修復自己把起降錯開），代價是 makespan **+7～26%**。
豁免本身**沒有被關掉**：同一批情境仍有 11 868–27 926 筆樣本對被豁免。

**⚠⚠⚠ 順帶：既有那條「守著豁免」的測試咬不到這個缺陷。**
`test_site_corridor_exemption_is_covered_by_schedule` **自己**把兩機都
`reserve_plan(p, "H")`，再斷言兩者站點相同 —— 循環論證，結構上不可能紅。
另一條 `..._requires_same_pad_and_own_window` 測「不同場坪」用的是 **3 km 外**。
而 `SITE_CORRIDOR_PHASES` 的 docstring 就引用前者當作豁免正當性的證據。

**守衛**：`tests/hv_dccpp/test_corridor_exemption_identity.py`（6 條）＋
`scratchpad/pad_identity_mutations.py`（**8/8 突變全咬**）。
⚠ 第 8 臂（規劃器蓋成 `landing_site`）第一版**沒咬到** —— mini example 全隊都是
H 起 H 降，突變是空操作；已改成先把一架的起飛站設成 ALT1 再驗。

**實飛驗證**（2026-09-09，四台 SITL，情境 `gui_legal_120_split_spread10`，
`spawn_spread_m=10` ⇒ 四台四個起降點）：

| 航次 | 建置 | 起降點 | 回收站自動指派 | 實飛最小水平／垂直 | 違規 |
|---|---|---|---|---|---|
| PAD8_CTRL | 修好 | 共點 | 開 | 151.6 / 28.2 m | 0 / 316 |
| PAD8_SPREAD | 修好 | 10 m × 4 | 開 | 148.8 / 61.3 m | 0 / 1077 |
| PAD8_BROKEN | **壞的** | 10 m × 4 | 開 | 149.3 / 60.7 m | 0 / 1080 |
| PAD8_FIXED_RAW | 修好 | 10 m × 4 | **關** | 52.7 / 112.6 m | **0** / 346 |
| PAD8_BROKEN_RAW | **壞的** | 10 m × 4 | **關** | **5.0 / 19.9 m** | **5** / 378 |

五趟都 4/4 落地、覆蓋 100%、生成點誤差 0.02 m、兩兩間距 10.01 m。
最嚴重那一筆是 B3 在 AGL 22.1 m、B4 在 AGL 2.2 m、水平 **5.0 m**，
而 `validate` 回報 **passed=True / conflicts=0**。

⚠⚠⚠ **前三趟分辨不出修法**：回收站自動指派把 B3/B4 移到 ALT1，兩臂都收斂到
同一份計畫（889.5 s、148.2 m）。原始計畫的差別是 51.5 m vs 5.0 m，被下游抹平。
受控實驗必須 `HV_RECOVERY_AUTO=0`。

**同步上游時**：#8 要重新套上。


## 之後要同步上游時

1. 用同一份腳本重新複製（排除 `__pycache__`、`figures`、`*.rar`、`.pytest_cache`、`*.egg-info`）。
2. 把上面那一條 `ROOT` 修改重新套上（或先確認上游已經自己改成往上搜尋）。
3. 跑 `python -m pytest tests/hv_dccpp -q`，預期 **809 passed**。
4. 再跑 `python -m pytest tests/test_hv_dccpp_adapter.py -q`（AeroPlan 這一側的接線）。

## 打包

`aeroplan_studio.spec` 已有 `hiddenimports += collect_submodules('hv_dccpp')`
（`hv_dccpp/__init__.py` 用 PEP 562 `__getattr__` 延遲載入整合層，靜態分析看不到）。
套件內沒有任何非 `.py` 資料檔，所以不需要 `collect_data_files`。

## 相依

`numpy>=1.26`、`scipy>=1.11`、`shapely>=2.0`、`matplotlib>=3.8`；
`pymavlink` 只有 `hv_dccpp.mavlink_link` 與 mock autopilot 會用到。
這些 AeroPlan 本來就有，vendor 之後也不需要額外安裝任何東西。
