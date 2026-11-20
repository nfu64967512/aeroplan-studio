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

## vendored 修改（原本只有 1 條，2026-09-09 起 8 條，2026-10-20 起 10 條，2026-10-21 起 14 條）

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

**檔案**：`hv_dccpp/deconflict.py`（`_corridor_masks` / `_corridor_sites`（#13 起併入 `_build_track`，判準不變）/ `_Track.site()`
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


### vendored 修改 #9：無風基準——風速 < 2 m/s 時方位不跟風向（2026-10-20）

**檔案**：

- `hv_dccpp/wind.py`：新增 `CALM_WIND_MPS = 2.0`、`reference_dir_from_rad`。
- `hv_dccpp/altitude.py`：`_CALM_WIND_MPS` 改引用 `CALM_WIND_MPS`；改 `_corridor_heading`。
- `hv_dccpp/decompose.py`：改 `direction_candidates`。
- `hv_dccpp/context.py`：加 `calm_wind_direction_ignored(…)` 有效值警告。

**缺陷**：「無風」不是唯一基準。兩處讓 0 m/s 的計畫隨輸入風向改變：

- 沒有 keep-out 時，`_corridor_heading` 在 W < 2 m/s 會退回 θ = β（輸入風向）。
- `direction_candidates` 一定把 β、β+π、β±90° 排在最前面，去重時保留先出現的，同一條軸的代表方向因此跟著風向變。

`gui_legal_120_1a2b` 在 0 m/s 時：來自 270°／0°／90° 的總時間是 945.2／1209.3／1101.8 s。建圖模式的條帶分配也不同，其中 270° 那份 validate 還不過。

**做了什麼**：`reference_dir_from_rad(wind, z)` 的規則：

- 沒有風層，或 z 處風速 < 2 m/s ⇒ 0（正北，等於舊碼「沒有風層」時的 β）。
- 否則 ⇒ `dir_from_at(z)`。

轉換方位（沒有 keep-out 時，以及 keep-out 迴圈的起始值）與掃描軸候選都改用它。側風／陣風檢查、地速、時間、偏流仍照真實風向量算。

**逐位元不變**：

- W ≥ 2 m/s（含門檻 2.0 本身）。
- 沒有風層的呼叫端。
- 0 m/s 來自 0°（GUI 預設）。

**會變**：0 ≤ W < 2 m/s 而且來向 ≠ 0° 的計畫。計畫會等於來自 0° 的那份；前面那個例子現在三種來向都是 1209.3 s，匯出檔逐位元相同。

**沒改**：`dynamic.py` 的 `_site_limits` 與 cue 條帶仍用 β。那條是函式庫的線上重規劃，AeroPlan 沒接。

**守衛**：

- `tests/hv_dccpp/test_wind_calm_and_crab_basis.py` 前 8 條，含有風與門檻的整份計畫值鎖，對照組是修改前的 HEAD b15070b。
- 突變：無風時走廊跟風向、候選跟風向、門檻改 ≤、有風時走廊朝北、不寫有效值，**5/5 抓到**。

**同步上游時**：#9 要重新套上。評估在 `docs/評估_2026-10-20_HARVEST偏流餘裕與無風基準.md` 第 2 節。

### vendored 修改 #10：建圖偏流餘裕可改用側風分量（`crab_margin_basis`，**預設不變**）（2026-10-20）

**檔案**：

| 檔案 | 改了什麼 |
|---|---|
| `hv_dccpp/wind.py` | `crosswind_max_mps` |
| `hv_dccpp/mission.py` | `MappingModeConfig.crab_margin_basis` |
| `hv_dccpp/context.py` | `apply_crab_margin(basis=…)` |
| `hv_dccpp/decompose.py` | `track_crab_rad`、`track_sensor`，用在 `proxy_time`、`build_strips`、`suitability`、`build_cross_set` |
| `hv_dccpp/planner.py` | `_mapping_crab_effective` |
| `hv_dccpp/integration/aeroplan_adapter.py` | `_crab_flown_effective` |

**為什麼**：舊的偏流餘裕 ψ = asin(W_max/V_a) 不看風向。風沿著條帶吹（實測偏流 0.1°）也照樣縮 20%（5 m/s）間距、多 18–32% 條帶。

**做了什麼**：`crab_margin_basis` 有兩個值。

- `"max_speed"`：**預設**，舊行為。
- `"crosswind"`：鋪航跡 θ 的條帶時用 ψ(θ) = asin(W⊥(θ)/V_a)，W⊥ = 所有風層中最大的 |W sin(θ − β)|。

crosswind 基準的實作：

- `ctx.sensors` 仍存全方向最壞的 ψ_max，給不知方向的檢查用（高度帶 d_turn、接縫重疊目標、圍籬、preflight），所以那些量兩種基準相同。
- 知道方向的消費者才用 ψ(θ)，間距只會變寬或不變。
- 有效值：`mapping_crab_effective(…)` 逐（構型、航跡軸）列出側風、ψ、間距。
- 轉接層的 `mapping_effective.per_class` 改成實際飛的偏流與間距，最壞值留在 `crab_envelope_deg`／`s_envelope_m`。

**逐位元不變（預設）**：`to_dict` 在 max_speed 時不列這個鍵，作法比照 `cross_*`，所以 `mapping_mode(…)` 警告字串、`mapping_effective`、計畫都與修改前相同（值鎖）。

**拒絕**：

- 未知的基準值。
- `crab_margin=False` 卻設了 crosswind（不會生效的值）。

**為什麼沒設成預設**（評估第 5.2 節）：

1. 首飛情境（`gui_legal_120_1a2b_pw3`、3 m/s 來自 270°）在 crosswind 下 validate 不過：兩架 HERO 降落在相距 10 m 的坪、只隔 90 s。舊基準在 5 m/s 來自 300° 也有同類失敗，根因在 4D 排程。
2. 規劃器會偏好沿風條帶，文獻（Coombes 等、ASPRS 2024）建議垂直於風；我們實飛順風切入的轉彎誤差最大（足跡 32 m，側風 16–18 m）。
3. 實飛顯示這個餘裕本來就不是保護覆蓋的東西：內部從沒不足；損失在條帶頭尾與邊界帶。換成 crosswind 會失去沿風時附帶的重疊。

切換預設的條件見評估第 5.4 節。

**守衛**：

- 新測試檔其餘 14 條（逐條帶間距、方向評分、有效值、預設＝修改前的值鎖）。
- `tests/test_hv_mapping_mode_adapter.py::test_crab_margin_follows_crosswind_component`（GUI 轉接層）。
- 突變：track_sensor 不作用、側風改 cos、鋪條帶或方向評分忽略航跡、不寫有效值、預設改成 crosswind、`to_dict` 列出預設、不拒絕無效設定，**全抓到**。

**同步上游時**：#10 要重新套上。

### vendored 修改 #11：殘留衝突時收尾的全域重排照樣出手、改求全綠（2026-10-21）

**檔案**：`hv_dccpp/deconflict.py`（`_global_reschedule` 新增 `resolve` 參數與第二段搜尋；`repair()` 收尾拿掉 `and not current`）。建在 #3 之上。報告（不改計畫）：`hv_dccpp/planner.py`（`_reschedule_resolved_note`）、`hv_dccpp/integration/aeroplan_adapter.py`（notes 原樣一份）、`ui/main_window.py`（`_HV_LOUD_NOTES`）。

**為什麼**：首飛情境 `gui_legal_120_1a2b_pw3`、crosswind 基準、規劃風 3 m/s 來自 270°，validate 留 1 筆 4D 衝突。B2／B3 降在各自的坪 TO_B2／TO_B3（相距 10 m），水平 10 m、垂直 13 m、dt_ab 90 s，門檻 Δt + σ_a + σ_b = 92 s。逐筆記錄 repair 的候選：

1. 第 1 輪貪婪為了 A1/B3 把 A1 延後 1391 s。A1 的起飛（TO_A1，離 TO_B3 20 m）因此落在 1451 s，緊接在 B3 降落（1345 s）之後。
2. B2/B3 的每個候選都只在這一對內成立：
   - B3 +178.5 s（`Conflict.delay_b_s`）會壓到 A1 的起飛爬升：水平 20 m、垂直 48.7 m、dt −0.5 s，Δ_req 91.5。
   - B2 +271 s 也撞上 A1。
   - B3 繞 2／5 圈、B2 繞 7 圈，也都留下一筆新衝突。

   (衝突數, ΣΔ_req) 沒有嚴格下降，全被 `_repair_one` 的單調性門檻擋掉。門檻本身是對的：放行會把衝突推給別的機對。
3. 解得掉的是三架一起的協同平移。能做這件事的只有 #3 的全域重排，而它被 `and not current` 擋著，只在已經沒有衝突時才跑。

同一個重排在殘留狀態下照原參數跑一次，就找到全綠解。

**做法**：

- `repair()` 收尾：有殘留時也呼叫全域重排，帶 `resolve=True`。`global_reschedule=False` 仍是總開關。
- `resolve=True` 時 `incumbent` 從 ∞ 起算。現況是紅的，任何全綠擺放都比它好，只在全綠候選之間比目標函數。makespan 由不能動的機決定時，全綠解的 makespan 與現況相同，沒有這一條就會被退掉。
- 兩段搜尋：
  1. 落地上界 = 現況 makespan。
  2. 第一段無解才把上界放寬一趟最長架次（`land_cap + max(dur)`）。紅的計畫不能上傳，拉長工期換全綠仍比較好。評估次數兩段共用 `max_evals`。
- 套用前的全機對完整驗證不變。resolve 時它**不再多餘**：兩架都已起飛者之間的衝突，擺放時從不檢查，重排也解不掉，只能靠這一道整個退回。
- 套用時在被動到的每一架 `UAVPlan.warnings` 加 `residual_resolved_by_reschedule(makespan a->b s, cap=kept|relaxed)`。`UAVPlan.warnings` 不進 `feasibility.warnings`（與既有的 `delay_orbit(...)`、`resequenced` 相同），所以 `plan_mission` 在 repair 之後把它彙整成**一條規劃器警告** `reschedule_resolved_residual(...)`：makespan 前後、差值與百分比、cap、參與重排的機。`a` 是貪婪修復後仍有殘留、不能上傳的那份；cap=relaxed 且變長時寫「工期拉長」。逐機註記的格式對不上時原樣帶出，不吞掉。
  - 轉接層 report：`warnings` 裡有它；`notes` 除了 `preflight:` 前綴那份，再原樣放一份（`_HV_LOUD_NOTES` 比的是 notes 的開頭，帶前綴比對不到）。
  - GUI：`reschedule_resolved_residual` 在 `_HV_LOUD_NOTES` 裡 ⇒ 規劃完狀態列與偵察頁狀態字（`harvest_report_lines`）都看得到；尾端的 validation_notes 那則不會把它蓋掉（GUI 逐則累加）。
  - 首飛格的值：`makespan 2697.9->2671.0 s（-26.9 s，-1.0%；…），cap=kept（落地上界維持原 makespan）；參與重排：A1、B2、B3`。
  - 沒出手（沒有殘留、重排找不到全綠解、`global_reschedule=False`）就一個字都不寫，`feasibility.warnings` 與計畫都和以前一樣。
- 沒有殘留時走 `resolve=False`，與修改前逐位元相同。

**改前改後**：32 格 = 風速 1／2／3／5 m/s × 風向 270／300／315／0° × 偏流基準（max_speed／crosswind）。GUI 路徑、建圖 HQ6、側向 0.7、前向 0.8。完整表與逐格摘要在 `runs/artifacts/RESIDUAL_4D_2026-10-21/`。

- validate：改前 3 格不過，改後 **32 格全過**。
- 其餘 27 格：匯出的 `.plan`、`.param`、`.waypoints`、圍籬、集結點逐位元相同。
- 有變的 5 格：

| 規劃風 | 基準 | validate | makespan s | 最小水平間距 m | 改後降落站（A1/B2/B3） | 怎麼變的 |
|---|---|---|---|---|---|---|
| 3 m/s 來自 270°（首飛） | crosswind | FAIL(1) → pass | 2697.9 → 2671.0 | 95.9 → 318.1 | 自己的坪 | 第一段：B2 起飛 120 → 752 s、A1 1451 → 1424 s |
| 2 m/s 來自 270° | crosswind | FAIL(1) → pass | 2672.9 → 2645.6 | 95.1 → 318.8 | 自己的坪 | 第一段：B2 起飛 120 → 748 s、A1 1435 → 1408 s |
| 5 m/s 來自 300° | max_speed | FAIL(2) → pass | 3255.5 → 3199.5 | 38.5 → 87.8 | H／ALT1／H | 兩份計畫都要第二段；回收站自動指派選了較快的那份（降回自己坪的是 3357.8 s，+3.1%） |
| 3 m/s 來自 270° | max_speed | pass → pass | 3180.1 → 2658.5 | 108.3 → 108.3 | H／H／ALT1 | 自動指派的候選以前是紅的、現在變綠又比較快 ⇒ 被選中 |
| 5 m/s 來自 270° | max_speed | pass → pass | 3362.3 → 3118.8 | 120.6 → 120.6 | H／ALT1／H | 同上 |

條帶、條帶間距、`uncovered` 在 5 格都不變。後 3 格改了降落站，所以航程與能量會跟著變。

**哪些會變**：

- `plan_mission` 層：只有 repair 收尾時仍有殘留的計畫會變。`tests/hv_dccpp/test_wind_calm_and_crab_basis.py` 的 7 個整份計畫值鎖（有風、預設建圖）逐位元不變。
- GUI 路徑的回收站自動指派（`hv_recovery_auto`）會另外規劃一份「改降 H／ALT1」的候選，紅的候選會被 `recovery_regression_guard` 擋掉。候選以前是紅、現在變綠又比較快，就會被選中，所以原本全綠的格子也可能改降落站（上表後兩格）。關掉自動指派（`HV_RECOVERY_AUTO=0`）重跑，這兩格改前改後逐位元相同。
- 規劃耗時：GUI 路徑任一次 repair 收尾是紅的，就多跑一次重排。首飛格 40 → 72 s，5 m/s 300° 52 → 127 s（矩陣是平行跑的，耗時只當量級看）。

**守衛**：`tests/hv_dccpp/test_repair_resolves_residual.py`，18 條：

- 合成情境：相鄰兩坪、間隔 90 s，`max_rounds=0` 只剩收尾這一步；每條「改後全綠」都配一條舊閘門下的前置斷言。
- 掃描完成目標：全綠解掃得比現況晚，仍要選它。
- 第二段放寬上界。
- 兩架都已起飛互撞：原樣退回、殘留照報。
- 報告（快）：上界維持／放寬兩種的整條警告字串值鎖；舊閘門下、貪婪自己解掉、mini 範例整條規劃器路徑都沒有這條警告；格式對不上時原樣帶出。
- 真實首飛情境（慢，規劃兩次共約 2–3 分鐘）：validate 通過；舊閘門下仍是那一筆 B2/B3、而且報告不說重排出手過；改後警告字串值鎖、轉接層 report 的 warnings 與 notes 都有、依到達順序餵 `MainWindow._on_hv_report` 後在 loud note 與偵察頁狀態字裡。

突變 8 臂全抓到：閘門改回 `and not current`、`resolve` 永遠 False、`incumbent` 不從 ∞ 起算、拿掉第二段、第二段一律跑、resolve 時拿掉套用前驗證、不留註記、忽略 `global_reschedule` 開關。

**同步上游時**：#11 要重新套上（在 #3 之上）。

### vendored 修改 #12：206 觸發距離扣掉 50 Hz 量化超出量（`trigger_overshoot_guard`，建圖預設開）（2026-10-21）

**檔案**：`hv_dccpp/ardupilot.py`

- 新增 `CAMERA_UPDATE_HZ = 50`、`trigger_overshoot_fraction(sensor)`、`commanded_trigger_distance_m(sensor, h)`。
- `CameraTriggerConfig` 尾端加 `trigger_overshoot_guard: bool = True`，並做型別檢查。
- `plan_to_mission_items` 的條帶開與地形重發 206 都改走同一個 `cmd_dist`。

**缺陷**：ArduPlane 以 50 Hz 跑 `AP_Camera::update`，每個週期才比一次「離上一張 ≥ d」，實際間距是 d 再加 0 到 v_g/50。

- 前向重疊 0.8 時一條帶剛好 l/d = 5 視，與驗收門檻 5 相同。
- 邊界只有最外側一條帶拍到的那一帶（10–20 m 寬），(5d̄ − l)/d̄ 比例的位置只剩 4 視。
- 2026-10-20 四趟實飛：724–5,136 m²（重播），全在邊緣帶，內部 0。

**做了什麼**：給了 `camera_cfg` 而且 `trigger_overshoot_guard=True`（預設）時，206 的 d = l(1 − o_fwd)·(1 − f_s/50)。

- f_s 是規劃拍照率（`r.shutter_rate_hz`），也就是規劃器算條帶地速上限 V_g,cap = l(1 − o_fwd)·f_s 的那個數。
- 扣掉 V_g,cap/50 之後，任何地速 ≤ V_g,cap 的實際間距都 ≤ l(1 − o_fwd)。
- 首飛相機 2 Hz：18.848 → 18.094 m，照片 +3.7–4.5%。
- 沒有拍照率（V_g,cap = ∞）⇒ 不扣，寫一次警告。f_s ≥ 50 Hz ⇒ ValueError。

**逐位元不變**：

- 沒給 `camera_cfg`（非建圖的舊匯出路徑）。
- `trigger_overshoot_guard=False`。
- 建圖計畫本身（分解、條帶、排程、.param、圍籬、集結點）：只有每條帶那一列 206 的 p1 改變。

**為什麼用 V_g,cap、不用規劃地速**：規劃風與實際風不符時（規劃無風、實飛 3 m/s），依規劃地速扣，M_BLIND 還剩 588 m²；依 V_g,cap 扣只剩 152 m²（角落的條帶頭轉彎效應），只多 1–2% 照片。

**限制**：

- 地速落在 V_g,cap 的最後 f_s/50 內時，CAM1_INTRVAL_MIN 先綁住，保護救不到。
- 不處理相機模組挑幀延遲、漏拍、CAM_MAX_ROLL 延後的首張。

**守衛**：

- `tests/test_hv_trigger_overshoot_guard.py`（5 條）：值、開關、只改 p1、警告；以飛前 50 Hz 模擬掃空速，開保護時帶內每一段間距 ≤ l(1 − o_fwd)。
- 改過的值鎖：`tests/hv_dccpp/test_camera_trigger_config.py`、`test_cross_strips.py`、`tests/test_hdmap_fix_exporter.py`、`test_hdmap_gui_coordinator.py`、`test_hdmap_cross_strips.py`、`test_survey_sitl_s1.py`。
- `tests/test_hdmap_exposure_gate.py`：原本靠「真規劃的邊緣帶一定有不足」來測「只有邊緣不擋」；改成兩份規劃：保護開 ⇒ 邊緣帶 0、保護關 ⇒ 邊緣有不足且不擋。
- 突變 9 種**全抓到**：保護不作用、沒給 camera_cfg 也扣、地形重發不扣、比例減半、開關失效、預設關、不警告、不檢查 bool、不檢查 f_s ≥ 50。

**同步上游時**：#12 要重新套上。評估、改前改後對照與拒絕「最外側條帶外推」的理由在 `docs/評估_2026-10-21_HARVEST邊界帶視數不足.md`。

### vendored 修改 #13：去衝突加速，規劃結果逐位元不變（2026-10-21）

**檔案**：`hv_dccpp/deconflict.py`（主體）、`hv_dccpp/geo.py`（`TerrainGrid.raw_view` 改寫，新增 `same_as_raw`、`corridor_minmax`）、`hv_dccpp/altitude.py`（`local_envelope`、`sample_bounds`、`build_3d` 驗證段共三處改用 `corridor_minmax`／`same_as_raw`）。建在 #3、#8、#11 之上。評估與剖析在 `docs/評估_2026-10-19_HARVEST運算速度.md`。

**為什麼**：作業點（嘉義真實地形）規劃 8.8 s 裡去衝突佔 2.7 s，其中 #3 的全域重排就佔 34–41%（4 架以上 58–86%）。9 個測試情境裡有 4 個重排完全沒改結果，卻花了 0.2–31 s。它把便宜的走廊檢查排在昂貴的 4D 檢查之後，每個候選都建一份完整的 UAVPlan（每個航點一次 `dataclasses.replace`）、重建所有已擺放者的軌跡與 KD 樹、而且要等全部機對算完才知道「有衝突」。

**做了什麼**（每一項都不改結果）：

- `_global_reschedule`：每個候選先查走廊時刻表（實測擋掉 63–95% 的候選；兩者都是純判斷，次序不影響可行與否）；4D 不再建 UAVPlan，候選軌跡由航點陣列直接平移（`_ShiftedTrack`：`t + d` 與 `shift_plan` 的 `w.t_s + d` 是同一個 IEEE 加法；`t_s` 不是 float／np.float64 時整機退回真的 `shift_plan`）；已擺放者的軌跡與 KD 樹只建一次；遇到第一筆衝突就停。順序、候選格點、`evals` 計數、#11 的兩段搜尋與套用前全機驗證都不變。
- 一次 `repair()` 範圍內的快取 `_Scope`（ContextVar，離開 repair 即丟，每個執行緒各一份，各類有 LRU 上限）：
  - 軌跡：鍵是建軌跡的**全部輸入**（航點 t/x/y/z/航跡角的 float64 位元組、階段碼、起降區間、站點、Δt_4d／t_sync／k_t／σ 上限的位元組）⇒ 就地改 `Waypoint` 欄位、換同長度航點清單、改起降站或時刻都會 miss（只是重建，騙不過）。
  - KD 樹、機對結果：以軌跡物件身分為鍵、條目持有強參照（id 不會被回收）。機對只存「無衝突／有衝突／(t_s, uav_a, uav_b, Δ_req) 序列」；`Conflict` 是可變物件，一律現算、不共用。
  - 禁止區間的左端點表（`_escape` 二分搜尋用）。
- 分級的分離檢查 `_sep_core`：`_Repair.involved_score` 只要 (衝突數, ΣΔ_req) ⇒ 不算 `_pair_delays` 與禁止區間（`_SCORE`）；`pair_clear` 只要有無（`_ANY`）。加總的值與順序與完整清單相同。
- `check_separation` 走同一個核心；範圍外每次現算，只在本次呼叫內共用 KD 樹（以前每個機對各重建兩棵）。
- 軌跡一次建好（`_build_track`）：走廊遮罩只算一次（以前 `_corridor_pads`／`_corridor_sites` 各算一次）。#8 的站點身分判準原封不動搬進 `_build_track`（`_corridor_masks`／`_corridor_sites` 已併入）。
- 航點欄位用 `np.fromiter` 抽：它只在 `None` 時與 `float()` 不同（變 nan，不丟 TypeError）⇒ 看到 nan 就整份改走原式，值與例外都同原版。
- `shift_plan`：位置參數建 `Waypoint`（實測 1.9k 航點 6.8 → 0.7 ms）；`Waypoint` 欄位表變了或是子類別 ⇒ 自動退回 `replace`。
- `_escape`／`_escape_down`：結構成立（全 float 二元組、左端非遞減、`lo[i+1] ≥ hi[i]`）且 ≥ 8 段時二分搜尋，否則照舊線性掃。
- `geo.TerrainGrid.raw_view`：**永遠回傳另一個物件**，`z`／`z_raw` 是 `self.z_raw` 的唯讀檢視（不複製）。
  - 原文件「`burn()` 不影響其內容」從此對任何時候拿到的視圖都成立。舊版在 z 與 z_raw 相等時回傳 `self`，之後燒入會改到已拿走的「自然地形」——無 nan 的網格其實早就違反（只是所有呼叫點都現取現用，沒出事）。對視圖本身 `burn()` 會因唯讀報錯。
  - 網格邊緣有 nan 時舊版 `np.array_equal` 永遠不等 ⇒ 白白複製一份、`altitude` 又重算一次同樣的燒入後走廊最大值。
  - 「兩者是否相同」改問 `same_as_raw()`：逐值相等（＝舊版回傳 self 的條件）或位元組完全相同（含 nan 位置）。與視圖一起快取，`burn()` 改動後重設。呼叫端不要再比物件身分（`altitude` 兩處已改）。
- `geo.TerrainGrid.corridor_minmax`：同參數的 max／min 只取樣一次（取代覆蓋層的 `_corridor_z` 行程層級記憶）。

**和覆蓋層驗證版的差別**（處理評估第 4 節列的風險）：軌跡快取鍵從「計畫身分＋幾個欄位」改成內容簽章（覆蓋層會被就地改 `Waypoint` 欄位騙過；代價是每次查快取要讀一次航點欄位）；機對快取不再共用 `Conflict` 物件；沒有任何行程層級狀態（覆蓋層有 `_CZ_LAST`、`_ESC_MEMO`）；`raw_view` 不違反文件保證；合併了 #11。`terrain_func_to_dem` 向量化**沒做**（同一時間另一個工作階段在改 `aeroplan_adapter.py`）。

**驗證**：

- 16 個情境（MINI、T2、S2／S3／S4_1A2B、S4_1A2B_w90（preflight NO_GO，比例外）、R12、ROT12、SQ6、S3n4／S4n4_mix、HILLS、S1n8_homo、S1n8_mix、S2n8_mix、DEM12＝嘉義真實地形 12 km²、1 Pioneer＋2 HERO），各自新行程、`PYTHONHASHSEED=0`，改前（只把這三個檔換回改前快照，其餘同一份 repo）與改後的完整輸出（每個航點每個欄位、所有時間、完整驗證報告、協調器屬性；15 個有計畫的情境共 3.0 M 葉）逐葉比對：**0 差異**（DEM12、8 架三種與 S4_1A2B 另外重複兩次，也是 0）；NO_GO 那一個例外訊息相同。允許的差異只有兩類，都不是規劃輸出：原版兩個行程之間本來就會變的 `keepouts.tree._tree`（STRtree 的記憶體位址），以及 `terrain._raw_view` 快取的內部結構（內容逐位元比過：同一份 z_raw）。比對期間 `hv_dccpp/`、`mission/`、`core/` 其餘檔案的 md5 前後相同。
- 突變：`deconflict` 19 種抓到 17 種，剩下 2 種在數學上等價（`bisect_left` 之後 `<` 換 `<=`；KD 樹條目已持有強參照時的身分檢查）；`geo` 7／7；`altitude` 捷徑 3／3（⚠ 這 3 種原本**沒有任何既有測試**咬得到——燒入障礙時 `zb_max` 那一支從沒被測過，新測試補上）。

**計時**（i7-12700、Python 3.14；依序執行、每次新行程、只量 `plan_coverage_dccpp`／`plan_mission` 本身；同機有其他工作階段在跑，括號是該次的系統 CPU 中位數）：

| 情境 | 改前 | 改後 | 倍數 |
|---|---|---|---|
| DEM12 作業點（嘉義真實地形、1P＋2H） | 8.76／8.12／9.97 s | 5.46／5.79／6.21 s | 中位數 8.8 → 5.8 s（1.5×） |
| S1n8_homo（8 架同構型） | 61.8／61.8／79.0 s | 6.8／6.8／11.9 s（52%） | 61.8 → 6.8 s（9.1×） |
| S1n8_mix（4A＋4B） | 54.7／58.2／60.8 s（13–19%） | 12.9／13.9／14.3 s（13–19%） | 58.2 → 13.9 s（4.2×） |
| S2n8_mix（4A＋4B） | 45.7／50.6／89.9 s（65%） | 11.6／12.4／14.4 s | 50.6 → 12.4 s（4.1×） |
| S4n4_mix／S3n4_mix | 52.4／97.4 s | 7.5／13.5 s | 7.0×／7.2× |
| HILLS（8 架、障礙燒入） | 74.3 s | 22.9 s | 3.2× |
| 其餘兩三架的基準情境 | 1.7–9.4 s | 1.4–4.6 s | 1.1–2.2× |

S1n8_mix 另有兩次在外部負載 36%／64% 下跑出 19.6／31.8 s，不列入。⚠ S1n8_mix 比評估文件的覆蓋層數字（約 12 s）慢約 1–2 s：軌跡快取改成內容簽章後，每次查快取都要讀一次所有航點欄位（約 4,900 次查詢 × 600 航點）。這是「就地改 `Waypoint` 欄位也騙不過」的代價；覆蓋層只比計畫身分，所以免費、但騙得過。
- 差分測試 `tests/hv_dccpp/test_deconflict_speedup_identity.py`：改前的 `deconflict.py` 凍結在 `tests/hv_dccpp/reference/deconflict_pre_speedup.py`（說明見同目錄 README，以 LF 正規化 md5 鎖住），在同一程序裡逐筆比 `check_separation`、`repair`（mini 範例複製成 2／6／8 架並擾動：起飛平移、已起飛、幽靈、空間互斥對、兩種目標、小評估預算）、`_global_reschedule`（resolve 兩種 × 評估預算 0／1／7／60、幽靈佔住最早時槽）、驗證器換成原版 `check_separation` 後的報告。驗證器也用新版 `check_separation`，所以不能自己驗自己。
- 快取騙不過：同一個快取範圍內就地改階段、航跡角、x、t，換同長度航點清單，改 `landing_site`／`t_arr_ptc_s`，每一步都要等於原版重算、而且每一步都真的改變了結果；IDP 回呼在 repair 中途就地改計畫，結果仍與原版相同；repair 不就地改輸入計畫；範圍離開（含例外）即清掉、別的執行緒看不到。
- `tests/hv_dccpp/test_terrain_raw_view_guarantee.py`：視圖永不是 self、唯讀、共用記憶體；先拿視圖後燒入，視圖不變；`same_as_raw` 的語意（含 nan 邊緣）；與舊版複製視圖逐位元同取樣；`corridor_minmax` 等於分開呼叫；燒入障礙（不在 keep-out）時 `local_envelope` 與 `sample_bounds` 的下界仍 ≥ 障礙頂 + h_clr；mini 範例強制關掉捷徑整份計畫相同。

**同步上游時**：#13 要重新套上，並跑上面兩個測試檔。上游若改了 `deconflict.py` 的**行為**，`reference/deconflict_pre_speedup.py` 要換成「上游新版＋#3／#8／#11，但不含 #13」，並更新測試裡的 md5；純加速的修改不要動它。

### vendored 修改 #14：網格邊緣有 nan 時照樣補燒障礙（2026-10-21）⚠ 會改結果

**檔案**：`hv_dccpp/context.py`（`build_context` 第 9 步，一行判斷）。建在 #13 之上（用到 #13 新增的 `TerrainGrid.same_as_raw`）。

**缺陷**：上游的設計是「`KeepoutSet.build` 沒燒入時，由 `build_context` 補燒一次」（D24，只做一次）。它用 `np.array_equal(terrain.z, terrain.z_raw)` 判斷「沒燒入」。

- 地形網格是「所有幾何 ⊕ 3000 m」（`TERRAIN_MARGIN_M`），超出 DEM 的節點是 nan。
- `np.array_equal` 不把 nan 當相等 ⇒ 只要外圈有 nan，這個條件**永遠是 False**，補燒那一支從不執行。
- 真 DEM 幾乎都是這樣。內建情境 MINI（50.5 萬個 nan 節點）、HILLS（110.8 萬個）也是，只有 PAPER_TABLE2 沒有。
- 上游 `test_context` 的 DEM 比網格大，所以沒測到。

**做了什麼**：條件改成 `terrain.same_as_raw()`，也就是逐值相等，或位元組完全相同（含 nan 位置）。

- 燒入時 nan 節點仍是 nan（`max(nan, top) = nan`）。
- 所以「位元組相同」正好等於「KeepoutSet 這一輪沒改到任何有限節點」。這就是無 nan 網格上舊判斷的意思。

**改前改後**：重現用 `test_context` 的幾何（平地 100 m、塔 T1 40 m AGL、輸電線 PL1 頂 130 m MSL），DEM 只蓋到幾何外 400 m。腳本與輸出在 `runs/artifacts/CONTEXT_BURN_NAN_2026-10-21/`（不進 git）。

| 情境 | 改前 | 改後 |
|---|---|---|
| routing 不可用（`keepouts=None`）＋ nan 邊緣 | 0 個節點燒入；塔位置 z = −0.5 m（應 ≥ 69.5）；沒有警告 | 燒入 242 個節點。與無 nan 對照組**逐節點、逐值相同**，警告全文也相同（`obstacles_burned_by_context(T1,PL1)`） |
| routing 可用＋ nan 邊緣，唯一的障礙貼著 DEM 邊緣 | KeepoutSet 量不到地面 ⇒ top = +inf，不燒入；context 也跳過 ⇒ 這座塔沒進地形 | context 用頂點 DEM 值算出頂高 65.69 m，燒入 9 個節點，加警告 `(TE)` |
| routing 可用＋ nan 邊緣，唯一的障礙頂低於地面 | 沒有警告 | 地形逐位元不變，但**多一條** `obstacles_burned_by_context(PLlow)`（見下面的限制） |
| routing 可用＋ nan 邊緣，一般障礙（KeepoutSet 照常燒入） | context 不補燒 | 不變（仍只燒一次） |
| DSM 上只有 tree／building | 兩邊都依 D24 略過 | 不變 |
| 內建 MINI／HILLS／PAPER_TABLE2 | — | `build_context` 的 `terrain.z` 位元組、警告全文、keep-out 頂高都**逐位元相同** ⇒ 整份計畫不變。HILLS 的 4 個障礙由 KeepoutSet 燒入，另外兩個情境沒有障礙 |
| AeroPlan GUI／轉接層 | — | 不受影響：`aeroplan_adapter` 一律給 `obstacles=()`，這一支被 `obstacles_enu` 短路 |

會變的只有**帶障礙的輸入**：`MissionInputs.from_dict`（hv_dccpp CLI／JSON）與 `mqtt_bridge`。而且只在 KeepoutSet 這一輪**沒改到任何有限節點**時才會變。

補燒和 KeepoutSet 燒入相比：燒到的是同一批 242 個節點，值差 ≤ 2.7e-5 m。差異來自兩邊的 ENU 換算路徑不同（KeepoutSet 用 `to_enu_xy`，context 用帶 DEM 高的 `_latlon_to_xy`），是上游原本就有的。

**限制**：這次沒修，因為問題出在代理本身。判準仍是「地形有沒有被改」，而不是「KeepoutSet 有沒有燒」。

- 障礙頂全部低於地面時，KeepoutSet 的燒入什麼都沒改 ⇒ context 再跑一次（同樣什麼都沒改），卻報 `obstacles_burned_by_context(...)`，警告不實。無 nan 網格在上游本來就這樣，這次只是讓 nan 網格與它一致。
- 貼著 DEM 邊緣的障礙（KeepoutSet top = +inf），只有在**其他障礙都沒改到地形**時才會被補燒。只要另有一個障礙被 KeepoutSet 燒入，它照樣不燒。keep-out 仍以 +inf 恆有效，水平避讓不受影響。
- 改用 `keepouts is None` 當判準比較直接，但會改到無 nan 網格的既有行為（上面「頂低於地面」與「貼邊緣」兩種），所以不在這次範圍。

**守衛**：`tests/hv_dccpp/test_context_burn_nan_edge.py`（3 條）。

- routing 不可用＋ nan 邊緣：先確認網格外圈真的是 nan，而且舊判斷在燒入前就已經是 False（確保測試有鑑別力）。燒入的節點、值、警告全文要與全覆蓋 DEM 的對照組逐位元相同；nan 位置不變；障礙頂 + b_v 有燒進地形。
- routing 可用＋ nan 邊緣：KeepoutSet 已燒入 ⇒ 不再補燒（仍只燒一次）。
- 貼邊緣的塔：KeepoutSet 的 top = +inf、網格 `z_at` = nan，context 補燒到 top + b_v。
- 突變 3 種**全抓到**。做法是在行程內換掉 `same_as_raw`，不動任何檔案：改回 `array_equal`、永不補燒（各紅 2 條）、一律補燒（紅「只燒一次」那條）。
- 改後一檔一行程全綠：`test_context` 37、`test_context_burn_nan_edge` 3、`test_altitude` 22、`test_terrain_raw_view_guarantee` 17、`test_deconflict_speedup_identity` 42。其他用到障礙的 10 個檔（`test_allocate`、`test_ardupilot`、`test_decompose`、`test_dem_datum`、`test_mission`、`test_preflight`、`test_routing`、`test_scenario`、`test_viz`、`test_planner`）共 294 條也全綠。

**同步上游時**：#14 要重新套上（依賴 #13 的 `same_as_raw`），並跑 `tests/hv_dccpp/test_context_burn_nan_edge.py`。

## 之後要同步上游時

1. 用同一份腳本重新複製（排除 `__pycache__`、`figures`、`*.rar`、`.pytest_cache`、`*.egg-info`）。
2. 把上面那一條 `ROOT` 修改重新套上（或先確認上游已經自己改成往上搜尋）。
3. 跑 `python -m pytest tests/hv_dccpp -q`，預期 **809 passed**。
4. 再跑 `python -m pytest tests/test_hv_dccpp_adapter.py -q`（AeroPlan 這一側的接線）。
5. #9、#10（2026-10-20）也要重新套上，並跑 `tests/hv_dccpp/test_wind_calm_and_crab_basis.py`：
   它的整份計畫值鎖一變，就代表有風（或預設建圖）的規劃改了。
6. #11（2026-10-21）也要重新套上，並跑 `tests/hv_dccpp/test_repair_resolves_residual.py`。
7. #12（2026-10-21）也要重新套上，並跑 `tests/test_hv_trigger_overshoot_guard.py` 與
   `tests/hv_dccpp/test_camera_trigger_config.py`（206 的 p1 ＝ l(1 − o_fwd)·(1 − f_s/50)）。
8. #13（2026-10-21，去衝突加速）也要重新套上，並跑 `tests/hv_dccpp/test_deconflict_speedup_identity.py` 與
   `tests/hv_dccpp/test_terrain_raw_view_guarantee.py`。上游改了 `deconflict.py` 的行為時，先把
   `tests/hv_dccpp/reference/deconflict_pre_speedup.py` 換成「上游新版＋#3／#8／#11、不含 #13」（見該目錄 README）。
9. #14（2026-10-21，nan 邊緣網格補燒障礙）也要重新套上，並跑 `tests/hv_dccpp/test_context_burn_nan_edge.py`。
   上游若自己把 `build_context` 的判斷改掉了，先看它在 nan 邊緣網格上還補不補燒。

## 打包

`aeroplan_studio.spec` 已有 `hiddenimports += collect_submodules('hv_dccpp')`
（`hv_dccpp/__init__.py` 用 PEP 562 `__getattr__` 延遲載入整合層，靜態分析看不到）。
套件內沒有任何非 `.py` 資料檔，所以不需要 `collect_data_files`。

## 相依

`numpy>=1.26`、`scipy>=1.11`、`shapely>=2.0`、`matplotlib>=3.8`；
`pymavlink` 只有 `hv_dccpp.mavlink_link` 與 mock autopilot 會用到。
這些 AeroPlan 本來就有，vendor 之後也不需要額外安裝任何東西。
