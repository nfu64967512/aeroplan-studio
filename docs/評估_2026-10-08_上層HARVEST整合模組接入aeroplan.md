# 上層＋HARVEST 整合模組接入 aeroplan：評估報告

> 日期：2026-10-08。依據的交接文件是 `vtol-hil-studio/docs/upper/AEROPLAN_HANDOFF_zh.md`，內容是 v0.5，但第 1 行仍寫 v0.4。
> 範圍：把 vtol-hil-studio 的 `harvest/`、`resilient_4d_vtol_coverage_system/`、`vtol_coverage_stack/` 和 4 個 `aeroplan_shim` 檔接進 aeroplan-studio（分支 `quad_swarm`，HEAD `0cc0a0d`，工作樹有大量未提交修改）。
> 標記：【實測】是跑過的；【讀碼】是讀程式得出的；【推論】是沒驗證的。〔建議〕表示只是建議，你還沒決定。
> 對抗驗證：原始定為 blocker 或 high 的發現，都交給另一個代理做過反駁式複核，§3 全部屬於這一類。§4 除了標「已對抗驗證」的之外，全部沒經過對抗驗證；有些條目另標「未經對抗驗證」，只是重申。
> aeroplan 的行號取自 2026-10-08 約 17:05–17:25 的工作樹快照，之後可能還會漂，動手時請用函式名重新定位。沒有標 repo 的路徑都在 aeroplan；`vtol_coverage_stack/…`、`harvest/…` 指 vtol-hil-studio 那一份。

> **勘誤（2026-10-08 18:10，產出報告之後補）：H8／P12／N26 的 SITL 強制參數已經由另一個 session 在 17:41 修掉，報告正文是修之前的快照。**
> - 現況【讀碼】：`mission/sitl_launcher.py:243-252` 已有 `VTOL_ENFORCED_PARAMS`（含 `ARMING_REQUIRE 1`）與 `build_defaults_list`，強制檔排在 `--defaults` 最後；`_SCHED_LOOP_RATE`（`:199`）VTOL 改為 100（韌體要求 ≥100，`SCHED_LOOP_RATE=50` 會被 `Arm: quadplane needs SCHED_LOOP_RATE >= 100` 擋下）。`tests/test_sitl_sysid_cmdline.py` 同時被改（17:42）。
> - 因此正文裡這幾處**作廢**：§0 結論第 4 類「aeroplan 的 VTOL SITL 起不來」、H8、§6.2 N26、§7 步驟 4 的 P12「補回 ARMING_REQUIRE」與驗收中的 `test_vtol_sitl_enforced.py`、H8 末條【推論】「SCHED_LOOP_RATE 50 不會觸發 gyro rate 檢查」（現在是 100）、§2.2 最後一段與 §4.6 FLY-1 的相關描述。
> - **仍然成立**：H8 的另一半——放行之後要有值層級的確認（切 AUTO 後 N 秒內相對高度或馬達輸出要變化，否則大聲中止、不記 `record_release`），因為 `release_check` 本身看不出飛機沒離地。這一半沒有被那次修復涵蓋。
> - 這次修復沒有經過本報告的對抗驗證，也沒有重跑 `sitl_link.py` 的 PreArm 路徑；要不要信它，請以另一個 session 留在 `runs/artifacts/SITL_ARM_TAKEOFF_2026-10-08/` 的實測為準。
> - 這個例子本身說明：aeroplan 今天仍有別的 session 在改，報告裡所有行號與「現況」都該用函式名重新定位（見 §8）。

---

## 0 結論

- **可以接，但不能照交接 v0.5 的 P1–P10 原樣接。**
  - 套件放進 aeroplan 本身沒有問題【實測】：沒有命名衝突，import 不會污染 aeroplan，在 aeroplan 的直譯器和 sys.path 下規劃出來的結果和金樣本逐位元相同（result `952950ad…`、payload `b04b36a8…`、upper `c49a3e4e…`、request `dad12148…`）。
  - 任務經 aeroplan 真的 `SITLLink` 寫入路徑送出時逐欄不變，讀回差異是 0。
- **對抗驗證之後沒有 blocker。** 原本的 3 個 blocker 都降為 high，總共留下 9 個 high（§3），分成四類：
  1. **前置條件**：兩邊的關鍵程式都沒提交。aeroplan 這邊的接點（`use_harvest`、`HarvestPlanningWorker`、`vtol_recon_panel.py`、`upload_mission_items(verify)`、`_on_mission_uploaded`）只存在工作樹裡，而且今天還有別的 session 在改。
  2. **交接的閘門設計有漏洞**：
     - P7 只守 `_sitl_broadcast`。SITL 分頁「Command」裡的逐機面板可以直接對單機切 AUTO 再解鎖，完全不經過它。
     - aeroplan 的 `arm()` 在 PreArm 擋下時一律改送 force（21196），沒有不 force、會回報結果的 API。
     - P6 的判斷條件在 MainWindow 裡讀不到 coordinator。
     - Write WPs、圍籬上傳、打擊流程都不在 P 清單裡。
  3. **規劃入口**：
     - 組請求時會靜默丟掉禁航區、DEM 和多區。
     - 新路徑的 13 則大聲註記在 GUI 上沒有地方顯示，反而會顯示上一趟留下的舊註記。
  4. **aeroplan 的 VTOL SITL 起不來**：它吃到 `ARMING_REQUIRE 0`，切到 AUTO 也不會離地。今天 17:07 在 aeroplan GUI 起的 i0 就是這樣：745 s 內四軸馬達一直是 PWM 1000，最高只到 0.27 m。這台不是評估代理起的；如果是你在試，飛不起來就是這個原因。
- **建議路線**〔建議〕（步驟編號和 §5、§6、§7 一致）：
  - 步驟 0：先做決策；
  - 步驟 1：兩邊各自凍結；
  - 步驟 2：純新增複製，只做核對、不接線；
  - 步驟 3：離線接入，只看不飛：新路徑的結果一律不能上傳、不能進 AUTO；
  - 步驟 4：安全閘放在同一個變更集：P6／P7／P10，加上鏈路層閘門、不 force 的解鎖、帳本強化、SITL 強制參數（強制參數〔建議〕可以先獨立修，見 N26）；
  - 步驟 5：在 aeroplan 的 SITL 上驗證流程。
  - HIL 和真機不在這次範圍。時間點〔建議〕放在研討會（10/16）之後。
- **理由**：上傳放行這條鏈依賴四個前提——逐機進 AUTO 只有一條路、解鎖不 force、帳本一定有紀錄、飛機真的會起飛。這四個在 aeroplan 現況都不成立。先做「只看不飛」，就能先拿到「看得到新計畫、能匯出可重現的 JSON」這部分價值，不必承擔放行風險。

---

## 1 評估做了什麼

**範圍**：共 9 個面向，每個面向都有交回結果。

| 面向 | 主要做法 |
|---|---|
| 交接錨點與工作樹漂移 | 逐條核對交接引用的行號，比對 HEAD blob 和工作樹，用 `git diff`／`ls-files`／mtime 追溯變動 |
| 環境、相依、匯入副作用 | 解析直譯器版本；AST 掃描全部 import；在共置的 sys.path 下匯入 69 個模組加 3 個 shim；共置跑規劃並量 GIL 延遲；建模擬佈局測 pytest 收集行為 |
| 共置乾跑 A | 跑 shim 測試、做突變實驗、重跑 wiring-check、用故意接錯的假根目錄測它；用假 `_mav` 驅動真的 `SITLLink` 上傳與讀回；`plan_upload` 情境測試 |
| 共置乾跑 B | 用真的 `SwarmCoordinator` 跑新路徑；6 鍵字典交給 aeroplan 顯示、上傳、匯出、驗算等消費端（用未綁定方法加假 self）；Qt 跨執行緒傳遞 |
| 安全閘 | 普查所有上傳、解鎖、切 AUTO、任務開始的入口；實測 E1–E7（閘門判斷、帳本併發、實例位移、逐機面板、KAMIKAZE、force 解鎖、心跳 sysid） |
| 規劃入口與 UI | 實測矩形判定、起降場位置與頂點順序、aeroplan 式請求、盤旋插入頻率、hull 圍籬包覆、Qt 信號 |
| 可飛性 | 檢查指令序列與容量；以韌體讀碼核對空速、PreArm；離線解析今天 i0 的 BIN 複本；用時間扭曲重算間距 |
| 架構與產品 | 核對 ADR-0007 到 0014 的關係；查論文版 HARVEST 有沒有被版本控管；E9 適用範圍；航線交錯 |
| 版本與流程 | 核對 manifest；用 dr 與 master 兩版 harvest 跑範例；模擬行尾轉換；檢查兩邊的提交狀態 |

**方法與守衛**
- 兩個 repo 都只讀。凡是會 import 兩邊程式的實驗，都在守衛下跑：PYTHONPATH 第一項是守衛模組，先斷言 `EVAL_GUARD_ACTIVE`，並攔截 `socket.connect`、`subprocess`、序列埠；另設 `QT_QPA_PLATFORM=offscreen`、`PYTHONDONTWRITEBYTECODE=1`；cwd 放在暫存目錄。跑完都做一次污染掃描：sys.modules 污染是 0；vtol-hil-studio 沒有新檔；aeroplan 的 mtime 有變動，但各代理的日誌與命令紀錄都歸因不到評估行程（見下方「評估期間 aeroplan 有人在動」）。
- 不建構 MainWindow，改用「未綁定方法加假 self」呼叫。`SITLLink` 用假的 `_mav`。SITL 日誌先複製出來，再用 DFReader 離線解析。
- git 只用唯讀子命令（`GIT_OPTIONAL_LOCKS=0`／`--no-optional-locks`）。
- 原始定為 blocker 或 high 的發現，都交給另一個代理嘗試推翻（反駁式驗證）。原始定為 medium 以下的沒有做這一步。

**沒做的事**
- 沒起 SITL 或 GUI，沒開 COM 埠，沒把套件複製進 aeroplan，沒跑 aeroplan 全套測試。
- 沒跑 PyInstaller，沒在 Python 3.10–3.12 下實際執行。
- 沒在 SITL 上量讀回耗時，沒查 QGC／Mission Planner 是否接受 `#` 註解行，沒逐點比對 6 鍵顯示路徑和 `mission.items` 的幾何。

**評估期間 aeroplan 有人在動**
- 17:07–17:21 GUI 和 SITL i0–i2 在跑。
- `ui/widgets/vtol_recon_panel.py` 在 17:04:53 被改過；`core/dccpp/collision_avoidance.py` 在 17:13、17:22、17:27 各被改一次。
- `.git/lost-found/` 在 17:07:50 新建了 71 個檔。
- 各評估代理的日誌裡都找不到自己寫入的紀錄，所以推定是你另一個 session，但來源沒有查證。

主要命令見附錄 B。

---

## 2 交接文件核對

### 2.1 核對結果數量

各面向會重複核對同一條說法，所以合計含重複。

| 面向 | 核對條數 | confirmed | drifted | wrong | 無法驗證 |
|---|---|---|---|---|---|
| 交接錨點與漂移 | 41 | 24 | 15 | 2 | 0 |
| 環境相依 | 24 | 15 | 5 | 3 | 1 |
| 共置乾跑 A | 24 | 12 | 10 | 2 | 0 |
| 共置乾跑 B | 23 | 13 | 8 | 2 | 0 |
| 安全閘 | 20 | 7 | 8 | 4 | 1 |
| 規劃入口與 UI | 18 | 9 | 7 | 2 | 0 |
| 可飛性 | 13 | 8 | 3 | 2 | 0 |
| 架構與產品 | 19 | 12 | 6 | 1 | 0 |
| 版本與流程 | 26 | 18 | 6 | 2 | 0 |
| **合計（含重複）** | **208** | **118** | **68** | **20** | **2** |

confirmed 的主要是以下幾類：
- P1、P2 的行號；P4 的 `_configure_hv_dccpp :5922`；`sitl_link` 的 API 行號；requirements、README、launch.json。
- 金樣本四個 sha，以及範例數字：任務 430／272／277 項，放行 0／148.3／76.8 s，落地 2532／2742／2701 s。
- wiring-check 對現在的 aeroplan 是 9 失敗、4 警告；shim 測試 60 條通過；HIL 會被拒；`_hil_links` 是 `_sitl_links` 的子集（R6）。
- 無法驗證的兩條：兩個套件在 3.10／3.11／3.12 下的語法（本機只有兩個 3.14），以及 P10「到放行時刻才讀回」的時序（沒在 SITL 上量）。

### 2.2 漂移的錨點（舊行號 → 現在）

| 交接位置 | 交接寫的 | 10-08 工作樹 |
|---|---|---|
| P3 `algo_combo` | `ui/widgets/vtol_recon_panel.py:526-528` | `:559-561`；預設改成 HARVEST 在 `:582-585` |
| P3 `get_params`／`fov_width_m`・`overlap_rate`／`planner` | `:1209`／`:1222-1223`／`:1251` | `:1247`／`:1260-1261`／`:1289` |
| P4 背景執行緒條件／FOV 檢查／`run_async` | `ui/main_window.py:5532`／`:8176`／`:8182` | `:5563`／`:8181`／`:8196`（use_hv）與 `:8204`（run_async）；另外新增起降場閘門 `:8186-8194` |
| P5 範圍／呼叫／定義／避撞 | `:5640-5658`／`:5646`／`:4781`／`:5651-5658` | `:5671-5689`／`:5677`／`:4812`／`:5681-5697` |
| P6 `on_sitl_upload_mission`／`_on_mission_uploaded` | `:7831`／`:7729` | `:7836`／`:7734`（HEAD blob 中前者在 `:7417`，後者不存在） |
| P6 sysid fallback／`upload_tuples`／`_build_wps_for_path` | `:8104-8113`／`:8058`／`:7924` | `:8108-8119`（迴圈從 `:8104` 起）／`:8063`／`:7929` |
| P7 `_sitl_broadcast`／`_target_links`／`mission_start` | `:6813`／`:6796-6808`／`:6832` | `:6818`（HEAD 在 `:6694`）／`:6801-6816`／`:6837`；全 repo 沒有任何地方呼叫 `_sitl_broadcast('mission_start')` |
| P8 `_export_dccpp_waypoints` | `:6311` | `:6316` |
| §4.1 `start`／預設 sysid／quadplane | `mission/sitl_launcher.py:474-484`／`:562`／`:583` | `:509-519`／`:601`（改成 requested_instance+1）／`:587-588`（`--model` 在 `:614`）；另外 `:623` 一律帶 `--sysid` |
| P6 port 公式／`_procs` | `:420`／`:685-686` | `tcp_port_for :455-467`；`_procs` 宣告在 `:442`、append 在 `:718` |
| P6 心跳 sysid 記錄 | `mission/sitl_link.py:760-772` | `:768`（`_frame_for` 在 `:758-770`） |

**沒有漂的**：
- P1 `mission/swarm_coordinator.py:930-931`；P2 `:1539`、`:1653`、`:1660-1663`；P4 `:5922`。
- P7 `:910`／`:912`／`:922`、`ui/widgets/sitl_hud.py:798`、`:1279-1281`；P8 `:6213`、`:6243`。
- `mission/sitl_link.py:240`、`:354`、`:2116-2121`、`:2137`、`:2155`；`ui/workers/planning_worker.py:96-110`。

**交接之後 aeroplan 和接線有關的變化**【實測】：
- 起降場「沒選就不規劃」的閘門（`ui/main_window.py:8186-8194`）。它讓新路徑的 home 和 pad 有了自然的來源。
- `sitl_launcher` 改成寫 `MAV_SYSID`、一律帶 `--sysid`、`SCHED_LOOP_RATE` 固定 50。舊的 `SYSID_THISMAV` 在 4.6 以後會被靜默略過，10-07 GUI 曾出現三台都回報 system 1。這次修正後，心跳 sysid 檢查才第一次有機會在 aeroplan SITL 上通過。
- 圍籬預設改成 hull，`FENCE_RADIUS` 改以 Home 計算（`mission/geofence_manager.py:143-146`、`:565-581`）。
- `utils/logger.py` 新增 `AEROPLAN_NO_FILE_LOG`；根目錄的 `conftest.py` 被重寫成只設這個環境變數，另外新增了未追蹤的 `tests/conftest.py`。

### 2.3 錯誤或不完整的說法（去掉重複後）

**錯誤**

| 交接原文 | 實際情況 | 出處 |
|---|---|---|
| §4 P7（:284）「依 HUD 指令對象選單機……這也是現成唯一的逐機 AUTO 路徑」「要在 `_sitl_broadcast` 統一把關」；`vtol_coverage_stack/wiring_check.py:12` 的說明同義 | 逐機面板、KAMIKAZE LAUNCH、動態偵打切換都不經 `_sitl_broadcast`（H1） | 漂移、乾跑 A、安全閘 |
| P4 步驟 1、P6 步驟 1（:291-292、:304）用 `configure_vcs` 清 coordinator，條件寫 `coordinator.vcs_output is not None` | MainWindow 每次規劃都新建一個區域變數的 coordinator（`ui/main_window.py:5518`），沒有常駐的 coordinator 可讀（H7） | 漂移、規劃 UI |
| P10 步驟 1（:333）「正常解鎖（不 force）」 | aeroplan 沒有不 force 的解鎖 API（H2） | 安全閘 |
| §8（:567）連線限制「i＝0–15」 | 埠被佔用時會位移到 instance+1 起再找 16 個，實測回傳 16（port 5920）。被拒屬於 fail-closed | 安全閘（其他面向判 drifted） |
| §7 第 3 點：在 aeroplan SITL 上驗「放行順序和時刻」 | 閘門、放行順序、讀回 sha、帳本持久化都驗得了；「飛機真的照時刻起飛」驗不了，因為 SITL 不會離地（H8） | 可飛性（驗證後修正為部分錯） |
| §8 E9（:558）多區、DEM、豁免高度會被拒 | 經 `build_request` 時這三項根本表達不出來，結果是靜默；禁航區只有呼叫端真的傳了才會被拒（H3） | 架構、乾跑 B |
| §4.2（:386）建議停用的欄位含「禁航區、地形」 | 停用禁航區欄位，會讓同一分頁「8. Fence 管理」畫的 NFZ 被靜默忽略。應從清單刪掉這兩項：禁航區改成把 `_hv_fence_zones()` 原樣傳進 `no_fly`、讓 check_request 大聲拒絕；地形改成已載入 DEM 時在規劃前擋下。這份清單同時漏了高度、轉換高度、速度、轉彎半徑、掃描角（H3） | 規劃 UI（PU-06 驗證） |
| §1 表：aeroplan 不對結果做 NFZ 繞行或避撞改寫 | 這是期望，不是現況。現在會改寫，要靠 P5 才成立 | 乾跑 B |
| §3（:243-244）`aeroplan_adapter` 會延遲 import `core.dccpp` | 那是 docstring 裡的範例（`harvest/integration/aeroplan_adapter.py:426-428`）；實測規劃期間沒有 import 任何 aeroplan 模組 | 環境、乾跑 B |
| §2.4:164 BUNDLE_MANIFEST 的 LF sha `124373b3…` | 那是 v0.4 的值，現在是 `f40e65f4…` | 版本與流程 |
| §2.1:127「選 dr 版的話，換了 harvest 金樣本一定會變」 | 實測換成 dr 版 harvest，三個內容 sha 完全不變 | 版本與流程 |
| §8（:610）「Cube 系列的任務容量約 700 項上下」 | SITL 與 CubeBlack 是 655 項，Cube Orange／Orange+ 是 1312 項 | 可飛性 |
| `tests/test_vcs_shim.py:3` 說「工作樹上 conftest.py 已被刪」 | 被刪的是 `pytest.ini`；conftest.py 還在，10-08 重寫過。「不攔截硬體」這半句是對的 | 環境、乾跑 A |

**成立但不完整**

| 交接原文 | 實際情況 | 出處 |
|---|---|---|
| §2.4（:215-217）副作用是會先 import aeroplan 根目錄的 `__init__.py` | 說法成立（以乾跑 A 為準，見 §2.5 C2），但漏寫 `resilient_4d_vtol_coverage_system/main_integration_test.py` 也會被 import（ENV-2） | 環境、乾跑 A |

### 2.4 文件本身的不一致

- 第 1 行和 §6.3 的「v0.4 重跑」（:492）沒改，但變更紀錄已經到 v0.5（:645）。
- §2.4:160 的樹雜湊 `29175cad…`／`3fc68890…` 和 :164 的 `124373b3…` 是 v0.4（manifest_20261007T191337）的值。v0.5 是 `fe0189cf…`／`f25c6297…`／`f40e65f4…`。:161 的金樣本和 :163 的外部 manifest sha `ee6d533b…` 是對的。
- §0 第 4 點寫「238 筆、83 個修改、刪 8」。現在約 253–257 筆（隨時間變），其中 91 個修改、7 個刪除；`conftest.py` 已從刪除變成修改，被刪的 7 個裡包含 `pytest.ini`。
- §4.2（:381）說 `out["six_key"]` 是 `VcsSixKey`。實測 `coordinator.vcs_output['six_key']` 是一般 dict、沒有旗標，只有 `plan_with_vcs` 回傳的那份是 `VcsSixKey`（淺複製，兩者共用 `assembled_paths`）。
- `docs/adr/README.md` 的索引把 0010 標成 Proposed（ADR 本文已是 Accepted），也沒列 0013、0014；這兩份都還沒進 git。

### 2.5 面向之間的矛盾，以及以哪個為準

- **C1 規劃耗時**：交接寫約 50 s。各面向實測：環境 90.7 s（3.14.3）／79.7 s（3.14.6）、乾跑 B 95.1 s、規劃 UI 93.4／96.5 s、版本 92.8／96.3 s，範例結果的 `compute_s` 是 101.59。【推論】差異可能與當時 3 台 SITL 加 GUI 的負載有關，但沒有在空機下量過；交接 50 s 是在哪個版本、什麼條件下量的也不明。規劃一律以約 1.5–2 分鐘估。
- **C2 pytest 會不會 import 根目錄的 `__init__.py`**：環境面向說不會（pytest 遇到不合法識別字的 `aeroplan-studio` 就停）；乾跑 A 說會，而且是在 Package setup 時以頂層模組 `__init__` 載入，相對 import 落進 except，無害。【推論】差別在於前者只量收集階段，後者是實際執行。以乾跑 A 為準，兩者都判定無害。真正多出來的副作用是 `main_integration_test.py` 被 import（環境面向實測）。
- **C3 依計畫起 SITL 時，pad 和 sysid 對不對得上**：漂移面向【讀碼】認為 home 取顯示路徑首點、alt 0、sysid 由對話框給，會被 `plan_upload` 整批拒絕。規劃 UI 面向【實測】發現顯示路徑首點和 pad 相距 0.00 m，墊相隔 60 m 時不會被排開，sysid 依 drone_id 排序一致。現行預設流程以實測為準；漂移面向和可飛性 FLY-9 指出的風險，只在 fan-out 自訂 sysid、或 drone_id 順序和 sysid 不一致時成立。
- **C4 避撞會不會預設就改寫**：乾跑 B 原本說「預設會抬到 212.5 m」；驗證後確認，用面板預設參數跑範例是 0 衝突、0 改動。規劃 UI 說「預設會跑」也對。結論：避撞預設會執行，範例在預設值下不會被改寫，P5 仍然必須跳過它。
- **C5 §7.3 的流程驗證做不做得到**：可飛性的核對判 wrong，驗證把它修正為部分錯：飛機不起飛也驗得了閘門與帳本。以驗證結果為準。
- **C6 F-A1「論文版程式無法還原」**：驗證發現 `hv_dccpp` 本身和 7 支 `tests/hv_dccpp` 守衛測試，vtol-hil-studio 的 commit `765b04e` 已有同內容（CRLF 正規化後 68/68 和 aeroplan 現檔相同）。在兩邊 git 都找不到的是：`swarm_coordinator.py` 的現行內容、`planning_worker.py`、ADR-0013／0014，以及 3 支頂層守衛測試（含 `test_vendored_naming_boundary.py`）。以驗證結果為準。
- **C7 交接是不是把工作樹行號說成 HEAD 行號**：漂移（DRIFT-2）和安全閘（S5）面向這樣認為；這兩條的驗證都指出交接 :258 明寫的是「`quad_swarm` 工作樹」。以驗證為準：只是措辭含糊，不算寫錯；真正的問題是 10-08 之後又漂了一次。
- **C8 DO_CHANGE_SPEED 19 m/s 會被「拒收」還是「夾到下限」**：可飛性認為是拒收（之後飛 AIRSPEED_CRUISE 24）；`scripts/hv_sitl_flight.py:15-16` 記的是夾掉（飛 20）。拒收有旁證（COMMAND_LONG 路徑實測回 MAV_RESULT=4），但任務項目路徑沒有實測，所以標【推論】。
- **C9 真板 Hero 的 AIRSPEED_MIN**：FLY-2 原本擔心大於等於 19；驗證查到板子參數檔是 12、MAX 25。以驗證為準。
- 小差異：`.claude/worktrees` 數量 12 或 13，wiring-check 掃描時間 28–34 s。都不影響結論。

---

## 3 阻擋項與高風險（經驗證）

驗證之後**沒有 blocker**。下面 9 項都是 high，必須在對應的接入步驟之前處理。原本定為 blocker 的 S1、FLY-1、F-A1 都降為 high；被降到 medium 的發現放在 §4。

**新增 P 項對照**（交接的 P1–P10 之外，本報告新增，§5、§7 沿用這些代號）：
- **P11** 圍籬（PU-08／DRIFT-5／FLY-5）
- **P12** SITL 啟動與強制參數（H8／DRIFT-6）
- **P13** 逐機旁路與鏈路層閘門（H1）
- **P14** Write WPs 與匯出檔辨識（H5）
- **P15** 飛前參數與容量核對（FLY-2／FLY-6／PU-09）
- **P16** 動態偵打切換守衛（B8）

### H1 逐機面板與打擊流程繞過 P7，可以對單機直接切 AUTO、直接解鎖
（DRIFT-1、A-01：確認為 high；S1：blocker 降為 high；S2：high 降為 medium）

- **問題**：P7 只在 `_sitl_broadcast` 把關。SITL 分頁「Command」子分頁裡的 DroneDetailPanel 是活的 UI（`ui/main_window.py:958-959`），它的模式下拉選單選 AUTO 會直接呼叫 `link.set_mode('AUTO')`；ARM 按鈕不看當下模式就直接呼叫 `link.arm()`，PreArm 擋下時還會改送 force。兩者都不經 `broadcast_check`／`release_check`，也沒有確認框。
- **證據**：
  - 【讀碼】`ui/widgets/drone_detail_panel.py:47-51`（`_MODE_LIST` 含 AUTO）、`:291`、`:751-760`、`:762-768`；鏈路來自 FleetRegistry（`:746-749`），所有鏈路都在 `ui/main_window.py:6533-6535` 註冊，HIL 也經 `:6645` 進來。
  - 【實測】在守衛下建一個真的 DroneDetailPanel，接假鏈路：選 AUTO 再按 ARM，鏈路收到 `[('set_mode','AUTO'),('arm',)]`。同樣的指令交給 `broadcast_check`，會被擋下或導去 release_check。
  - 【實測】用一個滿足 P2／P6／P7 的最小假根目錄（放入真的 `drone_detail_panel.py` 和 `strike_controller.py`）跑 wiring-check，結果印「接線自查通過」，結束碼 0。
  - 打擊流程：`ui/controllers/strike_controller.py:700-737` 只要 `_diamond_plans` 不是空的，就對 `_sitl_links[i]` 逐條呼叫 `auto_start()`（`:727`）。`_diamond_plans` 只在 `:1312` 清；打擊儀表板預設隱藏（`ui/main_window.py:638`），做過打擊規劃才會出現。
  - 另外，`guided_takeoff` 會切 TAKEOFF 再解鎖（PreArm 擋下時同樣改送 force），而 `broadcast_check` 在新路徑任務在場時仍放行它（`vcs_upload_gate.py:82` 的 RELEASE_GATED_CMDS 只有 auto_start、mission_start）。
- **影響**：
  - 依 `t_release_s` 逐機放行是 4D 隔離唯一的執行手段，這裡被繞過；§7 第 3 點承諾的「AUTO 裡解鎖都被擋」也不成立。
  - 被旁路放出去的機不會有 `record_release`，`release_check` 會因此擋住排在後面的機，反而把操作員推回面板逐機切。
  - 第一版只會上傳到 aeroplan 自己起的 SITL，所以後果限於 SITL（這是降級的理由），但這是交接自己定為「必須同一次提交」的閘門被無聲繞過。
- **〔建議〕**
  - 把閘門下沉到 SITLLink：`arm`／`set_mode`／`auto_start`／`mission_start`／`guided_takeoff`／`takeoff` 預設拒絕；AUTO 只接受 P10 在 `release_check` 通過後發出的一次性 release token；RTL／QRTL／QLAND／LOITER／disarm 這類中止動作永遠放行。要確認 P10 自己合法送出的 `set_mode('AUTO')` 不會被擋。
  - DroneDetailPanel 改成發信號給 MainWindow，走同一道閘門；`configure_vcs` 選新路徑時，清掉 `_diamond_plans` 並隱藏打擊儀表板。
  - wiring-check 加掃 `ui/` 底下所有直接呼叫 `set_mode`／`arm`／`auto_start`／`mission_start` 的地方。
  - 交接刪掉「現成唯一的逐機 AUTO 路徑」這句。
- **對應**：P7、P10，以及新增的 P13（逐機旁路與鏈路層閘門）。在「只看不飛」階段就要先把這些入口擋住。

### H2 aeroplan 的解鎖在 PreArm 擋下時一律改送 force，沒有「不 force、會回報結果」的 API
（S3：確認為 high）

- **證據**：
  - 【讀碼】`mission/sitl_link.py:310` 的 `arm()` 沒有參數；`:1018-1024` 走 `_arm_with_prearm(force_fallback=True)`，`:1002-1014` 先送一般解鎖，失敗就送 21196；`guided_takeoff`（`:1096`、`:1115`）和 `auto_start`（`:1198`、`:1210`）也一樣在失敗後 force。`arm()` 丟進佇列就返回，SITLLink 沒有任何「解鎖結果」信號。
  - 【實測】假 `_mav` 不回報解鎖時，送出的 command_long `(p1,p2)` 依序是 `(1,0)`、`(1,21196)`，耗時 3.55 s。只有私有的 `_arm_with_prearm(force_fallback=False)` 會只送 `(1,0)` 並回 False，但它要在接收執行緒裡持 `_mav_lock` 呼叫，不能從 GUI 執行緒直接用。
- **影響**：P10 步驟 1 與步驟 4 要的「不 force、PreArm 卡住就停、確認已解鎖」做不出來。force 之後遙測顯示已解鎖，確認那一步反而會通過；PreArm 擋下的問題（例如 VTOL land too short）也會被蓋掉。
- **〔建議〕**新增 `arm(force_fallback=False)`，加上解鎖結果或 ACK 信號。新路徑在場時，閘門與 P10 一律不 force，失敗就大聲停下。和 P10 同一次提交。
- **對應**：P10、P7。交接的 P1–P10 沒有列出這個 aeroplan 端必須新增的 API。

### H3 組請求會靜默丟掉禁航區、DEM、多區、豁免
（B2、PU-06、F-A2：確認為 high）

- **證據**：
  - 【讀碼】`vtol_coverage_stack/aeroplan_shim/integrations/vcs_request.py:20-23` 的簽章沒有 terrain、areas、waiver 參數；`:22` 的 `no_fly` 預設是 `()`；`:31-32` 固定只組 1 區；`:38` terrain 寫死 flat；`:41` waiver 寫死 None。但同檔 docstring `:3-5` 寫這些會「在按下規劃前就大聲拒絕」。
  - `request.py` 的拒絕邏輯（`:220-222`、`:276-278`）只有呼叫端真的傳了才走得到；交接 §4.2 的範例（:369-373）沒有傳 `no_fly`。
  - aeroplan 已經有 `_hv_fence_zones()`（`ui/main_window.py:5912-5920`，含 NFZ 和 THREAT）；`:5450-5455` 在面板沒帶 dem_path 時會自動帶入已載入的 DEM；P2 的 `plan_on_coordinator(self, drones=drones)` 會丟掉 `dem_path`／`terrain_func` 等所有 kwargs（`mission/swarm_coordinator.py:1539-1557`）。
  - 【實測】在 registry 加一個 NFZ 之後，顯示路徑被繞行改寫（航點 971→1513），但 `result.uavs` 完全沒變，也沒有跳出任何對話框。結果的 14 則 notes 裡沒有一則提到地形或 flat。
- **影響**：違反有效值契約。畫了禁航區、載了 DEM，結果卻照平地、無禁航區規劃；畫面顯示「已繞開」，實際上傳的任務會穿過去。§8 E9 在 GUI 這條路上不成立。
- **〔建議〕**
  - P4 改成必做，不是〔建議〕：把 `_hv_fence_zones()` 原樣傳進 `no_fly`；已載入 DEM、`survey_area_polygons()` 多於 1 區、或勾了豁免時，在規劃前擋下並寫明原因。
  - 上游讓 `build_request` 增加 terrain／areas 參數、拿掉 `no_fly` 預設值，讓拒絕真的走得到。
  - P3 停用不適用的欄位並標示「作業設定 aeroplan_d80pro_v1 固定值」。交接的停用清單要補上高度、轉換高度、速度、轉彎半徑、掃描角。
  - 交接 §4.2:386 的停用清單要刪掉「禁航區」，改成傳 `no_fly`、讓 check_request 大聲拒絕；「地形」改成已載入 DEM 時在規劃前擋下。照交接原文停用禁航區欄位，同一分頁「8. Fence 管理」畫的 NFZ 會被靜默忽略（PU-06 驗證）。
- **對應**：P3、P4（E9）。

### H4 只接 P2–P4 時，既有上傳會從 6 鍵字典重組出錯的任務
（B3：確認為 high）

- **證據**：
  - 【實測】用未綁定的 `on_sitl_upload_mission` 接假鏈路：sysid 1／2／3 分別收到 428／269／274 項。任務裡沒有 84 NAV_VTOL_TAKEOFF；DO_CHANGE_SPEED 取面板值 22（HARVEST 是空速 21.5／19.0）；每個 NAV_WAYPOINT 的 param2 都是 60（HARVEST 以 30 為主）。
  - 鏈路 sysid 是 [2,3,7] 時，sysid 7 經 fallback 收到 Pioneer 的 428 項任務。
  - 【讀碼】`ui/main_window.py:8057-8060` 的 `_hv_owns_result` 在新路徑恆為 False，於是走 `:7929` 的 `_build_wps_for_path`；fallback 在 `:8108-8119`。資料來源的優先序是 `:7842` `_diamond_plans` → `:7847` `map_widget.paths` → `:7875` `_dccpp_result`。
- **影響**：這是交接「P2 不可以單獨上線」那條警告的實證。
- **〔建議〕**維持同一變更集。新路徑的判斷必須排在 `:7842`、`:7847` 兩個分支之前；交接的「第一個敘述」要求已經涵蓋這點。在「只看不飛」階段，P6 的第一個敘述改成「新路徑結果在場就拒絕上傳」。把這組假鏈路測試做成回歸測試，斷言新路徑時 upload 一次都沒被呼叫。
- **對應**：P2、P6。

### H5 匯出（P8）從 6 鍵字典重組，匯出檔還能經 Write WPs 寫上飛機
（B4：high，部分修正；DRIFT-3、PU-10：medium，未經對抗驗證；S4：修正為 medium）

- **證據**：
  - 【實測】`VTOLMissionExporter.export_all` 輸出 430／271／276 行：84 的高度一律 40（面板值）；DO_CHANGE_SPEED 的 param1=1.0，是地速；NAV_WAYPOINT 的 param2=60。對照 result：430／272／277 項；84 是 60／50／50 m；178 的 param1=0（空速）21.5／19.0；接受半徑以 30 為主，部分 15–27。
  - 原發現說匯出檔「多出 DO_SET_HOME」，驗證修正為：seq 0 寫 179 只是 home 佔位，不算多一個指令。
  - 匯出入口在 `ui/main_window.py:6213`、`:6243`、`:6316`。偵察頁的匯出送的是偵察頁自己的轉換高度（`ui/widgets/vtol_recon_panel.py:1344-1347`），DCCPP 分頁才讀 parameter_panel（`ui/main_window.py:6227-6234`），兩者預設都是 40。
  - Write WPs：先 `on_sitl_load_wp_file`（`:7531`），再 `on_sitl_write_wp`（`:7583`，寫入在 `:7723`）。這條路不經 `plan_upload`、不記帳本，遇到真板只多一個預設停在「取消」的警告框；`mission/waypoint_file.py:437`、`:458` 會跳過 `#` 開頭的行。
- **影響**：
  - 匯出檔是一份沒經過 HARVEST 驗證、起飛高度不同、沒有放行時刻的任務。
  - 如果照 P8 改成從 items 寫 `.waypoints`，這份檔又能經 Write WPs 寫進任何鏈路（含 HIL）。帳本看不到這次寫入，aeroplan 重開後一鍵起飛閘門就失效。
  - 同一份檔用 Mission Planner 或 QGC 寫入，aeroplan 本來就看不到，所以只靠 Write WPs 辨識檔案只是部分解。
- **〔建議〕**
  - P8 從「建議」升為同一變更集必做：新路徑只匯出三份 JSON（請求、result、payload），NO_GO 也要存。
  - 如果要出 `.waypoints`，sha 放在旁檔，不寫進 `.waypoints`；Write WPs 遇到旁檔或 items_sha256 吻合的檔，就拒絕，或改走 `plan_upload`＋`record_upload`。
  - 「只看不飛」階段〔建議〕先不出 `.waypoints`。
  - wiring-check 的 P8 從警告改為失敗。
- **對應**：P8，以及新增的 P14（Write WPs 與匯出檔辨識）。

### H6 新路徑的大聲註記沒有顯示出口，還會顯示上一趟的舊註記
（PU-04、F-A3：確認為 high）

- **證據**：
  - 【讀碼】shim 的 `mission/vcs_branch.py:84-97` 不會呼叫 `hv_on_report`；`ui/workers/planning_worker.py:99-100` 只把它綁到 `report.emit`。
  - `ui/main_window.py:577` 初始化 `_hv_last_report`，`:6088` 是唯一改寫它的地方，規劃開始時不會清；`:6096` 的橫幅和 `:6134` 的失敗對話框都讀它。
  - 【實測】先模擬上一趟 hv_dccpp 留下的 note，再呼叫 `_on_hv_failed(str(VcsNoGo([...])))`：對話框裡同時出現這次的 NO_GO 原因和上一趟的舊註記。
  - 範例結果有 14 則 notes，其中 13 則是 loud。
  - 既有的顯示出口 `_on_hv_report` 期待字串前綴（`:6023-6055`），而且只在狀態列顯示前 3 則、15 秒（`:6106`）；新路徑的 notes 是 `{code, loud, text}`，只有 2 則對得上前綴。
  - 10-08 起 VTOL 分頁的預設演算法是 hv_dccpp（`ui/widgets/vtol_recon_panel.py:582-585`），所以「同一個 session 先跑過 hv_dccpp」很常發生。
- **影響**：Pioneer 120 m 零裕度、轉移段越界、暫定能量參數、Hero 能量餘裕只剩 61–63 Wh，這些操作員都看不到，還可能被舊警示誤導。
- **〔建議〕**P4 在每次規劃開始時清掉 `_hv_last_report`；新路徑另寫一個出口，GO 和 NO_GO 都依 `notes[*].loud` 用對話框或常駐面板顯示，並存檔（P8）。配一條測試，wiring-check 也檢查這一項。
- **對應**：P4。

### H7 舊結果殘留；P6 的條件在 MainWindow 裡沒有 coordinator 可讀
（PU-05：high，部分修正；DRIFT-4、B5：medium，未經對抗驗證；A-02：medium，未經對抗驗證）

- **證據**：
  - 【讀碼】`ui/main_window.py:5518` 每次規劃都新建一個 `SwarmCoordinator`（區域變數），`configure_vcs` 清的是這個新物件。`_dccpp_result` 只在 `:1959`、`:4112` 清、在 `:5846` 換掉，規劃開始時不清；`_on_hv_failed`（`:6123-6137`）也不清。同步的 DCCPP 規劃不會換掉 `_hv_worker`（只有 `:5993-6009` 會換）。
  - 【實測】`VcsSixKey` 經真的 HarvestPlanningWorker 跨執行緒傳回時是同一個物件，`is_vcs_result` 為 True。
  - 【實測】把 `VcsSixKey` 當 `active_result` 傳給 `broadcast_check`／`one_click_allowed`，會被當成「沒有新路徑結果」而放行，因為 `_vcs_present` 只認 `kind=='harvest_execution_result'`（`vcs_upload_gate.py:380-391`）。
- **影響**：
  - 先跑一次新路徑成功、再改跑同步 DCCPP，然後上傳：如果 P6 照字面去讀 `self._hv_worker.coordinator.vcs_output`，OR 條件成立，會上傳舊的新路徑任務，畫面上卻是 DCCPP。
  - 如果 P7 把 `_dccpp_result` 傳成 `active_result`，閘門就失效。
- **〔建議〕**
  - MainWindow 自己保存唯一一份 `self._vcs_output`（和 `_vcs_no_go_output`），只在 `_on_hv_finished` 收到 `is_vcs_result` 的結果時設定。
  - 每次按「規劃」、通過前置閘門時（和清地圖同一處），一起清 `_dccpp_result`、`_vcs_output`、`_hv_last_report`；`:1959`、`:4112` 也一併清。
  - P6 的條件改成 `is_vcs_result(self._dccpp_result) or self._vcs_output is not None`；P7 的 `active_result` 明寫為 `self._vcs_output['result']`。
  - 上游〔建議〕讓 `out['six_key']` 就是回傳的那個 `VcsSixKey`，並把完整輸出掛在上面（`six.vcs_output`）；`_vcs_present` 收到帶 `vcs_guard` 的物件、或任何不認得的非 None 物件，一律當作「有」。
- **對應**：P4 步驟 1、P6 步驟 1、P7。

### H8 aeroplan 的 VTOL SITL 起不來（ARMING_REQUIRE=0），release_check 抓不到
（FLY-1：blocker 降為 high）

- **證據**：
  - 【讀碼】`sitl/default_params/plane.parm:84` 是 `ARMING_REQUIRE 0`。`mission/sitl_launcher.py:624-640` 的 `--defaults` 只有 plane.parm、vtol_default.parm、identity.parm，沒有覆寫。
  - 未追蹤的 `tests/test_vtol_sitl_enforced.py:29-34` import 了 `VTOL_ENFORCED_PARAMS`，但它在工作樹和 HEAD 都不存在。
  - `mission/sitl_link.py:668-674` 的註解宣稱啟動器已經注入 vtol_enforced.parm，實際沒有；`:955-1000` 一看到心跳 armed 就印「PreArm 檢查通過（真機同樣可解鎖）」。
  - 【實測】把今天 i0 的 `sitl/plane/i0/logs/00000115.BIN` 複製出來離線解析：韌體 V4.8.0-dev (85d417ee)，PARM ARMING_REQUIRE=0；27.0 s 進 AUTO 並執行 VTOLTakeoff；之後 RCOU 的 C3、C5–C8 一直是 1000，POS RelHomeAlt 最高 0.27 m；ARM 紀錄 0 筆；記錄總長 823.6 s。
- **影響**：
  - 第一版唯一准許的上傳對象，就是這種 SITL。P10 會拿到「已解鎖」、切 AUTO、記下 `record_release`，飛機卻停在地上，沒有任何錯誤。
  - 這是 aeroplan 原本就有的缺陷，舊路徑和 hv_dccpp 一樣受影響。沒有實體風險（三份板子參數檔都是 ARMING_REQUIRE,1），但直接違反有效值契約。
- **〔建議〕**
  - 在安全閘提交之前或同時，把 VTOL 強制參數注入補回 launcher：ARMING_REQUIRE=1，放在 `--defaults` 的最後一份。規格就是那份未追蹤的 22 條測試。因為它也影響舊路徑和 hv_dccpp，〔建議〕獨立先修，見 §6.2 N26。
  - P10 放行後加一道值層級的確認：切 AUTO 之後 N 秒內相對高度要超過 X m，或馬達輸出要大於 1000；做不到就大聲中止、不記 release。N 和 X 由你定。
  - 【推論】補上注入後兩項 PreArm 不會卡：NAV_VTOL_LAND 前一段 490.9／490.9／608.6 m，大於門檻 108 m；SCHED_LOOP_RATE 50 不會觸發 gyro rate 檢查。這兩項要在 GUI SITL 上實際重驗。
- **對應**：新增的 P12（SITL 啟動與強制參數）、P10。

### H9 aeroplan 端論文版 HARVEST 的接點沒有被版本控管
（F-A1：blocker 降為 high）

- **證據**：
  - 【實測】`mission/swarm_coordinator.py`（+358／−30，mtime 09-09）是論文實飛 `_plan_gui` 路徑呼叫的檔（95a337b 的 `runs/artifacts/BEAMIDP1/hv_sitl_run.py:207`，use_harvest=True）。它現在的內容在兩個 repo 的任何 git 物件裡都找不到：rev-list 找到 6 個 blob，LF 正規化後沒有一個和工作樹相同。論文 runs 的 `git_state.txt` 只記了 HEAD 和檔名清單，沒有內容雜湊。
  - `ui/workers/planning_worker.py` 在 aeroplan 的任何 ref 裡都不存在。aeroplan 沒追蹤的還有 ADR-0013／0014、3 支頂層守衛測試（`test_vendored_naming_boundary.py`、`test_fleet_split_guards.py`、`test_nfz_defeats_auto_split.py`）和 7 支 `tests/hv_dccpp` 守衛測試。
  - 其中 `hv_dccpp` 本身（VENDOR #2–#8）和 7 支 `tests/hv_dccpp` 守衛測試，vtol-hil-studio 的 `765b04e` 有同內容（CRLF 正規化後 68/68 和 aeroplan 現檔相同）；`docs/harvest_vendor_manifest.json` 比對 69/69 相符。
  - 在兩邊 git 都找不到的是：`swarm_coordinator.py` 現行內容、`planning_worker.py`、ADR-0013／0014，以及 3 支頂層守衛測試（含 `test_vendored_naming_boundary.py`）。
  - HEAD 的 `ui/main_window.py:917` import 了從未提交的 vtol_recon_panel，`:5918` import 了已經不存在的 `HvPlanningWorker`。
- **影響**：P1／P2／P4 要改的正是這些檔。在未提交的工作樹上直接改，論文版 `swarm_coordinator.py` 的狀態就找不回來；改在 HEAD 開的乾淨分支上，又缺 `HarvestPlanningWorker` 等檔。不會造成事故（所以不是 blocker），但會切斷「論文可重現」這條線。
- **〔建議〕**接入前由你，或你指定的 session，先單獨提交並打 tag（例如 `aasrc2026-harvest`）。範圍至少包含：
  - `swarm_coordinator.py`、`ui/workers/`、`vtol_recon_panel.py`、`mission/path_segments.py`；
  - ADR-0013／0014，以及 3 支頂層守衛測試和 7 支 `tests/hv_dccpp` 守衛測試（後者 vtol-hil-studio 有同內容，但 aeroplan 這邊仍要納管）；
  - `sitl_link.py`、`main_window.py` 的寫入與讀回改動，連同 `tests/test_mission_upload.py`、`tests/test_mission_download.py`。
  - 和正在改 `main_window.py`／`vtol_recon_panel.py` 的 session 錯開時間。
- **對應**：前置條件（P1、P2、P4、P6、P10 都依賴）。

---

## 4 各面向發現（精簡）

### 4.1 環境相依
本面向沒有 high，以下全部未經對抗驗證。

**正面結果【實測】**
- 不帶路徑的 `python` 會解析到 `C:\Program Files\Python314\python.exe` 3.14.3。numpy 2.3.5、scipy 1.17.0、shapely 2.1.2、PyYAML 6.0.3、pymavlink 2.4.49 都和 `BUNDLE_MANIFEST.json` 記錄的相同。
- 命名衝突 0 個：AST 掃了 843 條 import，沒有一條解析到 aeroplan 的模組。匯入 69 個模組加 3 個 shim，沒有污染、沒有寫檔。共置規劃 GO，sha 和金樣本相同。
- 匯入副作用只有三項：第一次規劃時，在 worker 執行緒設 `MAVLINK20=1`／`MAVLINK_DIALECT`；scipy 加 2 條 warnings 篩選；`harvest.viz` 會把 matplotlib 設成 Agg，但管線不會載入 viz。
- 背景規劃時主執行緒的 p95 延遲約 20 ms、最大 43–80 ms，GUI 不會凍結。複製量是 84 個檔、3,430,547 B。

**問題**
- **ENV-1**（medium）import 邊界掃描是 shim 測試和 wiring-check 共用的，會掃整棵 aeroplan 樹：3685 個檔（.claude 1479、runs 1080、scratchpad 518），約 28–34 s。`vtol_coverage_stack/wiring_check.py:33-36` 只略過頂層目錄。套件提交之後新建的 `.claude/worktrees/<x>/harvest/__init__.py` 會因字串引用被誤判失敗（實測 4 筆）。同 A-05、DRIFT-9。〔建議〕只掃 git 追蹤的檔，或略過以 `.` 開頭的目錄和 runs、scratchpad。
- **ENV-2**（medium）`pytest.ini` 被刪了：在根目錄不帶路徑跑 pytest 會收到 120 條（shim 測試兩份），還會 import `resilient_4d_vtol_coverage_system/main_integration_test.py`。〔建議〕還原 HEAD 版的 pytest.ini。
- **ENV-3**（medium）帳本檔不存在時被當成空帳本，一鍵起飛會放行（實測從 False 變成 `(True,'')`）。`data/vcs_flight_ledger.json` 不在 `.gitignore` 裡；SITL 每次啟動會刪 eeprom（`mission/sitl_launcher.py:659-661`），但帳本紀錄還留著。同 S6。
- **ENV-4**（low）兩個 Python 3.14 共用 site-packages：用 py 啟動器會拿到 3.14.6，結果逐位元相同，但每次規劃都出 environment_warnings。〔建議〕launch.json 改用絕對路徑。
- **ENV-5**（low）pymavlink 用 v1 還是 v2，取決於「開鏈路之前有沒有先規劃過」。〔建議〕在 `main.py` 最前面先 setdefault `MAVLINK20`。
- **ENV-6**（low）邊界檢查漏掉 `from harvest import *` 和 `harvest.VehicleLink` 這種屬性存取。
- **ENV-7**（low）缺第三方相依時丟的是原始的 ModuleNotFoundError，不是 `VcsUnavailable`（`mission/vcs_branch.py:70-74`）。
- **ENV-8**（low）打包：HEAD 版 spec 沒處理這三個套件，打包版選新路徑一定得到 `VcsUnavailable`（fail-closed）【推論：沒跑過 PyInstaller】。

### 4.2 共置乾跑 A：shim 測試、wiring-check、上傳轉換

**正面結果【實測】**
- shim 測試在 prepend 與 importlib 兩種模式都是 60 passed，但不會 import 任何 aeroplan 模組。突變實驗中只有 2 條「證明攔截生效」的測試會紅，所以「60 passed」只證明 shim 本身的邏輯，不證明 aeroplan 接線正確。
- **A-06**：用假 `_mav` 驅動真的 `_send_mission_items`，三架 979 項除了 seq 0 的 current（1 改成 0，這是既定設計）全部原樣送出；讀回 0 差異；`result_sha256` 前綴是 952950adbd74。所以 P6 步驟 4 的形狀轉換在 aeroplan 現有寫入程式上成立。
- **A-09**：用 aeroplan 真實的 conn_str 格式跑 `plan_upload`：自起 i0–2、位移到 i4–6 都通過；COM、不在自起清單、10962、心跳 sysid 不符、兩架 Hero 標示對調（離墊 868.4 m）、構型錯配、多出 HIL 鏈路都被拒。

**問題**（全部未經對抗驗證）
- **A-02**（medium）帳本只記 ok=True 的上傳。飛控 ACK 接受但讀回失敗時，任務其實已寫入，卻沒記帳。另見 H7。
- **A-03**（medium）aeroplan 的有損讀回比對抓不到這幾種改動：178 的 speed_type 0→1（空速變地速）、速度 21.5→20.0、接受半徑 30→27、30→29.5，實測都判 ok=True。原因：`mission/sitl_link.py:206-209` 把 16、178 列為有損，`:2146` 讀回值是 0 一律放行，`:2148` 的容差是 max(1.0, 0.1×|sent|)。〔建議〕新路徑用嚴格比對器，只涵蓋 16／84／85／178。
- **A-04**（medium）wiring-check 對故意接錯的程式也回 0，包括：use_vcs 分支沒有 return、P6 條件寫反、`broadcast_check` 的結果被丟掉、必要呼叫放在死碼裡、`verify=False`。〔建議〕收緊這幾條規則，每條配一個負向測試。
- **A-05**（medium）同 ENV-1；另外也抓不到 `importlib.import_module("harvest." + "mavlink_link")` 這種動態載入。
- **A-07**（low）`tests/test_vcs_shim.py:3` 的說明文字錯了（見 §2.3）。
- **A-08**（low）證明攔截的金絲雀測試探測的是 127.0.0.1:5762（`tests/test_vcs_shim.py:112`、`:126`），也就是 SITL i0 的 SERIAL1 預設埠【推論：沒確認當時有在監聽】。攔截一旦失效，這條測試就會去連你正在跑的 SITL。〔建議〕改用綁定但不監聽的埠。
- **A-09**（low）自起的 SITL 位移到 i16，或手打 tcp:localhost，會被拒（fail-closed），但拒絕訊息會誤導成「可能是實體板」。
- **A-10**（low）`mission_uploaded` 沒有關聯 ID，和 Write WPs 共用同一個 slot；鏈路執行緒結束時，佇列裡還沒送的上傳不會回報；SITL 重起之後帳本會留下過期紀錄。
- **A-11**（low）`aeroplan_upload_items` 會靜默丟掉 mission_type（`vcs_upload_gate.py:286-288`）。

### 4.3 共置乾跑 B：真的 SwarmCoordinator 與 6 鍵字典

**正面結果【實測】**
- 用真的 `SwarmCoordinator` 跑範例：GO，耗時 95.1 s，四份 sha 和金樣本相同，規劃期間沒有 import 任何 aeroplan 模組。
- 6 鍵字典交給 aeroplan 的顯示、2D／3D、圍籬、SITL 生成點都能跑完，不丟例外。
- **B9**：`VcsSixKey` 經 PyQt6 6.11 的 `pyqtSignal(dict)` 傳遞時保留同一個物件。`dict()`、`.copy()`、`{**}` 會丟掉旗標，aeroplan 現行程式沒有這種複製。〔建議〕把 finished_ok 改成 `pyqtSignal(object)`【推論，屬防禦性修改】。

**問題**
- **B1**（high 降為 medium，已對抗驗證）避撞後處理會原地改寫新路徑的顯示路徑（它假設三機同時起飛），本來就該由 P5 跳過。但「預設就會抬到 212.5 m」不成立：用面板預設值（speed 22、min_sep 50、alt_offset 50、上限 100）跑範例是 0 衝突。真正的新缺口是 wiring-check 的 P5 規則（`wiring_check.py:129-133`）只看 `_apply_fence_avoidance_to_dccpp_result` 的函式本體，看不到 `_display_dccpp_result` 內嵌的避撞段（`ui/main_window.py:5681-5697`），而且只出警告。`collision_avoidance.py` 評估期間被改了三次，數字可能還會變。
- **B5**（medium，未經對抗驗證）見 H7。
- **B6**（medium，未經對抗驗證）時間顯示的語意會誤導：
  - makespan 2742.3 s 其實是 UAV-10 的落地時刻，含 148.3 s 的放行延遲。
  - 摘要表的逐機時間用面板 18 m/s 重算，Pioneer 47.67 min，比 makespan 45.7 min 還長；HARVEST 實際的 t_land−t_takeoff 是 2531.7／2594.0／2624.3 s。
  - 3D 轉換高度全是 40。
  - `ui/main_window.py:5831-5845` 把面板值寫進結果，當成「權威值」。
  - 放行時刻在 GUI 沒有任何地方用到。
- **B7**（medium，未經對抗驗證）覆蓋率驗算會出假紅燈：刈幅 46.4 m 時算出 51.20%，100 m 時 64.80%。原因是縮點後每條航帶只有起點標 sensor_on；HARVEST validate 的 coverage_ok 是 True。
- **B8**（medium，未經對抗驗證）`strike_controller._on_strike_recon_trigger`（`:1992`）會讀 `_dccpp_result['assembled_paths']`，再依 index 呼叫 `_sitl_links[a.uav_id-1].upload_mission`（`:2097-2102`），不排除 HIL，也不在 P 清單和 wiring-check 的範圍內。

### 4.4 安全閘（指令路徑普查）

結論：照交接的設計，P6／P7／P10 在 aeroplan 現有程式上不會是唯一的閘門。S1 與 S3 見 H1、H2。

- **S2**（high 降為 medium，已對抗驗證）KAMIKAZE LAUNCH 依 index 呼叫 `auto_start`；TERMINAL SYNC 依 sysid 對 FleetRegistry 的鏈路呼叫 `guided_goto`／`change_speed`（`strike_controller.py:885-903`）。觸發前提很窄：同一個 session 留著沒清的打擊計畫，而且操作員在覆蓋任務中按下打擊專用的按鈕。同一個殘留狀態也會讓 `on_sitl_upload_mission` 優先上傳打擊任務（`:7842`）。
- **S4**（high 降為 medium，已對抗驗證）新路徑任務可能不留帳本紀錄就進到載具，有三種情境：
  - (a) 匯出檔隔天經 Write WPs 寫回，這是最實際的情境；
  - (b) ACK 接受後讀回失敗，但要整批每一架都失敗才會導致放行；
  - (c) 寫帳本丟 PermissionError，只在人為佔住檔案 handle 時重現過。
  - 帳本裡只要有一筆紀錄就會擋。aeroplan 沒設 `sys.excepthook`，P6 若沒把 `record_upload` 包起來，slot 裡的例外會讓程式中止【推論】。
- **S5**（high 降為 medium，已對抗驗證）見 4.8。另外：工作樹版的 `tests/test_mission_upload.py`（`-k 'not Loopback'`）225 條在守衛下全部通過。
- **S6**（medium，未經對抗驗證）帳本併發問題：
  - 兩個實例交錯讀寫，前一筆會被蓋掉（遺失更新）；另一個 handle 開著時寫入丟 `PermissionError [WinError 5]`，不是 LedgerError。
  - `config/settings.py:79-80` 的 data_dir 依 project_root 決定，所以每個 worktree 各有一份帳本。
  - `_vcs_present` 是全域判斷，一筆過期紀錄就會擋住所有鏈路（是否接受見 §6.2 N27）。
- **S7**（medium）「自己起的 SITL」判定不夠嚴：
  - 死掉的行程留在 `_procs` 裡（只有 stop() 才清）。
  - 別的程式 LISTEN 同一個埠時，會連到佔用者（`mission/sitl_launcher.py:546-548`）。
  - 〔建議〕`own_sitl_conns` 只收 `proc.poll() is None` 的，這要從〔建議〕改成必要；核對該埠所有 LISTEN 的 PID 都等於 `proc.pid`。
- **S8**（medium）P6／P10 需要的 API 大多要新增：
  - 心跳 sysid：目前只有私有的 `_vehicle_srcs`／`_frames`，標示不符時 `get_latest_telemetry()` 回 None。
  - 原始 custom_mode 和心跳時戳。
  - 位置是否有效的旗標。
  - 能關聯請求的讀回。
  - 機型綁定：`vehicle_hint` 只有 VTOL／PLANE／COPTER。
- **S9**（medium，推論）P10「到 t_release 才讀回任務」做不到：逐項讀回每項最多等 3 s×3 次，而且整段持 `_mav_lock`；50 Hz 下 277 項推估要 5.5 s 以上，而晚放行容差 `RELEASE_LATE_TOL_S` 是 2 s。〔建議〕讀回移到預備階段，讀回後鎖住該機的上傳入口。
- **S10**（medium）`broadcast_check` 不管的指令：
  - `guided_takeoff`、`takeoff`、`set_mode GUIDED／TAKEOFF`、`set_params`、`vtol_transition` 實測全部放行。
  - 點圖起飛（`ui/main_window.py:7113-7187`）和右鍵起飛（`:7273-7387`）不經閘門。
  - 圍籬上傳對全部鏈路含 HIL（`:3196-3216`），預設 `FENCE_ACTION=1`。
  - 快速參數寫入不經閘門。
  - 〔建議〕新路徑在場時改成白名單。
- **S11**（low）P10 需要結果本體，但 aeroplan 沒有常駐保存；重開之後帳本有紀錄卻沒辦法放行。這不是 fail-open。
- **S12**（low）靜態的 wiring-check 驗不出這些繞過路徑。

### 4.5 規劃入口與 UI

**正面結果【實測】**
- **PU-14**：新路徑應該從「VTOL 偵察」分頁進。顯示路徑第一點和三個 pad 的距離是 0.00 m；墊相隔 60 m 時不會被排開；SITL 的 sysid 依 drone_id 排序。〔建議〕請求的 uav_id 用 `'UAV-{drone_id}'`、sysid 用 drone_id，不要沿用範例的 UAV-9／10／11。
- 照現行流程用範例結果建 hull 圍籬：11 個頂點、5.218 km²，包得住三個墊（離邊 85.6 m）和 973 個任務點。

**問題**
- **PU-01**（high 降為 medium，已對抗驗證）手點的四邊形幾乎都過不了矩形判定（0.5°／1 m，`vtol_coverage_stack/request.py:27-31`）：點擊誤差 σ 為 0.5／2／5 m 時，200 次分別被拒 93／193／199 次。修正：「工具 → 多邊形編輯器（Ctrl+M）→ 匯入角點 (JSON)」可以帶入精確角點，軸對齊的方框能通過。被拒時是在規劃前大聲報 RequestError。
- **PU-02**（high 降為 medium，已對抗驗證）起降墊必須在掃描座標 x<x_T 的位置（範例約 −487 m）；起點落在哪一側，由頂點 0→1 那條邊和頂點 3 在哪一邊決定（`upper.py:101-109`）。home 在北 500 m、在西 300 m、或改從東北角逆時針點角點，都會在 1–3 s 內 NO_GO，而且訊息用的是掃描座標。這個限制只屬於 `*_contain` 臂；起降場的位置會反過來決定航帶方向。
- **PU-03**（high 降為 medium，已對抗驗證）aeroplan 式的輸入常會插入 19 NAV_LOITER_TIME：5 組幾何裡 2 組有插。aeroplan 式輸入 A（單一起降場三墊排一列、home 在作業區西 900 m）那組插了 233.793 s、半徑 200 m 的 19；金樣本範例沒有插。aeroplan 不推參數，所以實際盤旋半徑是 `WP_LOITER_RAD` 75（`mission/sitl_launcher.py:366`）。盤旋在 ORBIT 相位，不影響放行時刻。新發現：harvest 自己的上傳器會設 `WP_LOITER_RAD`（`harvest/mavlink_link.py:981`），但 aeroplan 被禁止用它，`vcs_upload_gate` 也不設、不核這個參數。
- **PU-07**（high 降為 medium，已對抗驗證）見 4.8。
- **PU-08**（medium，未經對抗驗證）圍籬：
  - `FENCE_ALT_MAX` 是顯示點最高值加 20，等於 140 m（`mission/geofence_manager.py:433-436`），擋不住 120 m 的法規上限。
  - `on_sitl_upload_fence`（`ui/main_window.py:7811-7834`）不排除 HIL、沒有新路徑閘門、寫參數前不做全表備份，違反交接 §0.1 第 1、4 條。
  - 圍籬是從顯示路徑建的，不是從實際上傳的 items 建的。
  - 同 DRIFT-5、FLY-5。
- **PU-09**（medium）aeroplan 不推送也不核對任何參數。同 FLY-2。
- **PU-11**（medium）`_on_vtol_recon_planned`（`:8220-8259`）和驗算（`:8357-8362`）拿面板值重算。同 B6、B7。
- **PU-12**（low）wiring-check 的盲點：避撞段、圍籬、規劃開始時清狀態、`_hv_last_report`。
- **PU-13**（low）新路徑時風的輸入被藏起來（`ui/widgets/vtol_recon_panel.py:833-852`）；機數可以設 2–12；規劃按鈕被不相干的相機足跡判斷擋住；`hv_wind` 是 (speed, dir_from)（`:1304`），而 `build_request` 的參數順序是 wind_from_deg、wind_speed_mps，容易接反。
- **DRIFT-7**（low）面板有三處 `== 'hv_dccpp'` 的二元判斷（`:878`、`:899-900`、`:1254`），新增的 `'vcs'` 選項會被當成 DCCPP。

### 4.6 可飛性

**正面結果**
- **FLY-8**【實測】三份任務都能通過 aeroplan 的上傳前檢查，指令只有 16／84／85／178，frame 0 只出現在 seq 0。
- **FLY-7**【讀碼】「VTOL land too short」的門檻是 108 m，實際距離是 490.9／490.9／608.6 m。但在 GUI 起的 SITL 上，這道檢查目前根本沒在跑（見 H8）。

**問題**
- **FLY-2**（high 降為 medium，已對抗驗證）Hero 的 DO_CHANGE_SPEED 是 19 m/s，低於 aeroplan SITL 的 AIRSPEED_MIN 20（`mission/sitl_launcher.py:277-279`）。【推論】韌體會拒收這筆指令，Hero 改飛 CRUISE 24 m/s（快 26%），落地約提早 500 s；讀回比對看不出來，因為存進去的內容沒變。只影響 aeroplan 的通用 SITL，板子上 Hero 的 AIRSPEED_MIN 是 12。〔建議〕上傳到 aeroplan SITL 時大聲提示；開放任何非 aeroplan SITL 的上傳之前，把「178 的 param2 必須落在讀回的 [MIN, MAX]」做成硬性檢查。
- **FLY-3**（medium，推論，未經對抗驗證）用時間扭曲重算 aeroplan SITL 下的間距：UAV-11／UAV-9 的最小水平間距從 182.9 m 掉到 87.3 m，低於 d_sep 100 m 約 31 s。爬升、TECS、轉換等參數也都和 ARC 參數組不同，SIM_WIND_SPD 是 0。所以在 aeroplan SITL 上看到的時刻與間距都不能當證據。
- **FLY-4**（medium）Pioneer 最高 120.00 m；aeroplan 在飛行中沒有任何機制強制 120 m（GUI SITL 的 FENCE_ENABLE=0）。HV4SITL8 實飛到 120.2 m。
- **FLY-5**（medium）同 PU-08。
- **FLY-6**（low）任務容量是 655／1312 項，模組沒有事先檢查（超過時飛控會回 NO_SPACE，是大聲失敗）。〔建議〕加一道 n_items ≤ 655 的檢查。
- **FLY-9**（low）見 §2.5 C3。〔建議〕新路徑起 SITL 時直接用 `result.uavs[*].pad` 和 sysid。
- **FLY-10**（info）舊坑仍然成立：P5 沒接好時避撞會逐輪抬高；新路徑不輸出 .param，所以 Q_RTL_ALT 仍是 25；SITL 從來沒設過風。

### 4.7 架構與產品

F-A1、F-A2、F-A3 見 H9、H3、H6。以下未經對抗驗證。

- **F-A4**（medium）120 m 的回歸：Pioneer 高度 119.99999665 m；請求會拒絕任何非 null 的 `waiver_max_agl_m`，表達不了論文用的 119.5 m。95a337b 的實飛：天花板 120.0 那次超標 354.93 s、最高 120.1742 m；119.5 那次 0 超標。〔建議〕在 h_ceil 可以設定之前，新路徑只限 SITL。
- **F-A5**（medium）ADR-0013：上層沒有等面積的分區臂；capability 分區用的「幅寬×空速×續航」加權正是 ADR-0013 否決的做法。預覽 makespan 4482 s 對 HARVEST 的 2742 s。兩邊的證據落在不同參數區間。〔建議〕採「範圍限定」：ADR-0013 繼續管 hv_dccpp 路徑；要推翻它，先做同一份請求的配對對照。
- **F-A6**（medium）掃描方向由點角點的順序決定，偵察區沒有矩形繪製工具，ADR-0014 的方向保證因此失效。
- **F-A7**（medium）E9 的適用範圍很窄：只有論文那個 1.243 km² 的矩形配 1A+2B 被接受；你 10-07 實際畫的區（內角 54.5°～115.8°）被拒。兩個 HARVEST 選項的差異主要來自相機假設（條帶間距 21.09 m 對約 97 m），所以在 GUI 上互相對照沒有意義。
- **F-A8**（medium）同一個 GUI 會出現兩個「HARVEST」。主線在 baseline 之後有 33 個 commit、+13,424 行，vtol-hil-studio 已把 HARVEST 重新定位為「執行層」。〔建議〕ADR-0015 把名稱分層，論文寫明用的是哪一版 HARVEST。
- **F-A9**（medium）交付方式從整合計畫書 §5 的「子行程、不 vendor」改成 in-process 複製，交接沒有交代為什麼。
- **F-A10**（info）VENDOR 的 8 條修改在主線都還在，manifest 比對 69/69 相符。
- **F-A11**（info）新的分區消除了「Pioneer 條帶穿插三個子區」：放行錯開只占 makespan 的 5.4%。但轉移段仍會進入別機的分區，而且只有一個情境的證據。
- **F-A12**（medium）接入時機，見 §6 N2、N3。
- **F-A13**（medium）現在接入對飛行主線沒有直接貢獻。〔建議〕把「只認自己起的 SITL、HIL 整批拒絕」單獨接到既有的上傳路徑，可以保護 hv_dccpp 路徑（R6：`ui/main_window.py:6654`、`:8132`）。
- **F-A14**（medium）ADR-0007「6 鍵同形，下游一行都不用改」的前提，對新路徑反過來變成危險來源。
- **F-A15**（low）新路徑沒有進度回報，也不能取消。
- **F-A16**（low）ADR 索引和 `docs/PROJECT_STATUS.md` 都過期。

### 4.8 版本與流程（含錨點漂移）

- **DRIFT-2、PV-2、PU-07、S5**（都由 high 降為 medium，已對抗驗證）P1／P2／P3／P4／P6／P10 的接點只存在於未提交的工作樹：
  - HEAD 的旗標叫 `use_hv_dccpp`（HEAD:705），沒有 `use_harvest`；HEAD 的 `sitl_link` 沒有 `upload_mission_items`、`mission_uploaded`、`_mission_readback_diffs`。
  - `git diff --numstat`：main_window +584／−117、swarm_coordinator +358／−30、sitl_link +595／−26、sitl_launcher +73／−21、sitl_hud +47／0。
  - main 落後 quad_swarm 76 個 commit，所以不能在從 main 分出的 worktree 上接。
  - 【實測】wiring-check 的 P2 順序檢查只認字面上的 `use_harvest`（`wiring_check.py:88-97`），在 HEAD 上會靜默跳過。
  - 修正說明：這是提交衛生和可追溯性的問題，不會讓接入在行為上出事；在乾淨 checkout 上接，會在執行時丟 AttributeError（大聲失敗）。〔建議〕把「同一次提交」改寫成「同一變更集，不得出現 P2 已生效但 P6／P7／P10 還沒接的中間狀態」。
- **PV-1**（high 降為 medium，已對抗驗證）vtol-hil-studio 這邊：兩個套件、`scripts/make_aeroplan_bundle.py`、相關測試、`docs/upper/` 都沒進 git；外部 manifest 放在被 gitignore 的 `var/`。修正：10-07 的三份 manifest 是交接迭代時一口氣產生的，之後約 21.5 小時原始樹沒有變；結果的 provenance 帶三個樹雜湊，複製後仍能識別版本。加重：交接手抄的識別值已經混了兩份凍結（§2.4）。〔建議〕複製當下要有一份持久的凍結紀錄，數值從 manifest 檔本身取。
- **PV-3**（medium，未經對抗驗證）aeroplan 正被修改，見 §1、§2.2。
- **PV-4**（medium）v0.4 和 v0.5 的雜湊混用，見 §2.4。
- **PV-5**（medium）金樣本、selftest、鏡像測試都分辨不出 harvest 的版本：dr 版（`dd67b9f5`）和 master 版跑出的三個內容 sha 相同，鏡像測試兩版都是 11 passed。〔建議〕加第二個「一定會插盤旋」的金樣本請求。
- **PV-6**（medium，讀碼）dr 和 hv014 兩個分支的 harvest 完全相同。選 dr 版時要另外補兩件事：`LoiterExportError` 在 `vtol_coverage_stack/pipeline.py:321` 沒被攔住（CLI 結束碼 1、不寫結果、P8 存不到 NO_GO）；`vtol_coverage_stack/settings.py:78`、`:86` 的白名單仍允許 19。〔建議〕第一版用 master `3a001ca`，並把「匯出含 19」改成 NO_GO（代價見 §6.1 §9-1）。
- **PV-7**（low）行尾：模擬 commit 再 checkout 之後，55/84 個檔的原始 sha 會變，但外部核對和 verify-bundle 仍然通過（它們接受 LF sha）。
- **PV-8**（low）`require_bundle` 檢查的是 code_root 底下的檔，不核對實際載入模組的 `__file__`。
- **PV-9**（low）耗時被低估，見 §2.5 C1。
- **PV-10**（info）vtol-hil-studio 工作樹的其他修改不碰這三個套件；但整合相關的決定紀錄只寫在未提交的 CLAUDE.md 裡。
- **DRIFT-6**（medium）§4.1 的 SITL 對應沒有對應的 P 項，見 §2.5 C3。
- **DRIFT-8**、**DRIFT-12**（low）實例範圍描述不精確；文件與 shim 的描述文字漂移。

---

## 5 工作量估計

以下數字取自各面向的估計，彼此有重疊，不宜直接相加。P11–P16 的定義見 §3 開頭。

| 工作包 | 主要檔案 | 正式程式 | 測試 | 出處 |
|---|---|---|---|---|
| 前置：aeroplan 基線提交 | main_window、swarm_coordinator、sitl_link、sitl_launcher、sitl_hud 的未提交 diff（見 4.8）；vtol_recon_panel.py 1347 行；ui/workers | 不寫程式，要另外審查 | — | 漂移、版本 |
| 前置：vtol-hil-studio 提交 | 約 70 個未追蹤檔，加上 CLAUDE.md | 不改程式；重產 manifest 約 5 分鐘 | — | 版本 |
| 交接文件修正 | 約 25 個錨點、約 8 行數字、標頭版號、新增 P11–P16 | 約 1–2 小時 | — | 漂移、版本 |
| P1 | `mission/swarm_coordinator.py` | +4 行 | 1 條 | 規劃 UI |
| P2 | 同上（放在 use_harvest 與 FOV／DEM 區塊之前；dem／terrain 非空就 raise） | +10–15 行 | 3 條（約 60 行） | 規劃 UI |
| P3 | `ui/widgets/vtol_recon_panel.py`（含三處二元判斷、風輸入、固定 1A+2B） | +60–90 行（架構面向估 +40–80） | 6 條（約 100 行） | 規劃 UI、架構 |
| P4 | `ui/main_window.py`，另加一個請求組裝的純函式模組 | +200–300 行，加 150–200 行 | 約 20 條（約 350 行） | 規劃 UI |
| P5 | `_display_dccpp_result`（兩段都跳過、不寫回面板值） | +10–20 行 | 3 條 | 規劃 UI、乾跑 B |
| P6 | `ui/main_window.py` | 150–250 行（乾跑 A）或 300–400 行（安全閘） | 見下 | 乾跑 A、安全閘 |
| P7 | `_sitl_broadcast` | 約 50 行 | 見下 | 安全閘 |
| P8 | 三個匯出函式 | 60–150 行 | 5 條（約 120 行） | 乾跑 B、規劃 UI |
| P9 | ADR-0015、索引、PROJECT_STATUS、ADR-0010 頂端說明 | 150–250 行 | — | 架構 |
| P10 | 逐機放行（預備 → 讀回 → 倒數 → release_check → AUTO → 讀回 → 記帳、中止、放行表） | 500–800 行 | 見下 | 安全閘 |
| P11 圍籬（新） | `on_sitl_upload_fence`、`upload_fence_zones_to_fc` | 20–50 行 | 3 條 | 規劃 UI、可飛性 |
| P12 SITL 啟動與強制參數（新） | `mission/sitl_launcher.py`、`on_sitl_launch` | 注入 30–40 行；alive_conns 與監聽 PID 約 40 行；若要依構型疊參數再加約 60 行 | 規格測試已有 22 條；放行後爬升確認 3–4 條 | 可飛性、安全閘 |
| P13 逐機旁路與鏈路層閘門（新） | SITLLink、drone_detail_panel、strike_controller、點圖／右鍵起飛、參數寫入 | 閘門加 token 約 150 行；各入口合計約 200 行（漂移面向估 3–4 檔 +80–150 行） | 10–20 條假鏈路測試 | 安全閘、漂移 |
| P14 Write WPs（新） | `on_sitl_write_wp`、`on_sitl_load_wp_file` | 含在 P13 的入口估計裡（各 10–40 行） | — | 安全閘 |
| P15 飛前參數與容量核對（新） | P6／P10 預備 | 約 50 行；容量檢查約 10 行 | 6–8 條；2 條 | 可飛性 |
| P16 動態偵打切換守衛（新） | `strike_controller._on_strike_recon_trigger` | 約 5 行 | — | 乾跑 B |
| SITLLink 新 API | 心跳 sysid、custom_mode 與時戳、位置時戳、不 force 解鎖與結果、讀回請求關聯 | 100–150 行 | — | 安全閘 |
| 機型綁定 UI | `on_sitl_connect` 擴充 | 約 100 行 | — | 安全閘 |
| 顯示語意 | 摘要表、makespan 標示、驗算停用、信號型別 | 約 25–115 行 | — | 乾跑 B |
| 上游：wiring-check | 收緊規則、旁路掃描、掃描範圍、星號 import 與屬性存取 | 約 80＋30＋40 行 | 約 10＋6 條 | 乾跑 A、B，環境 |
| 上游：帳本強化 | pending／verified、fail closed、鎖、唯一 tmp、重試、缺檔策略 | 50–150 行 | 6–8 條 | 乾跑 A、安全閘 |
| 上游：嚴格讀回比對器 | 白名單 16／84／85／178 | 約 60 行 | 10 條 | 乾跑 A |
| 上游：其他 | ImportError 改成 VcsUnavailable、python 版本只比 major.minor、19 改 NO_GO | 45–70 行 | 4–6 條 | 環境、版本 |
| aeroplan 小修 | pytest.ini、.gitignore、launch.json、main.py 設 MAVLINK20 | 少於 30 行 | — | 環境 |
| 打包（選做） | spec | 約 15 行，另需約 0.5–1 天 | 在 dist 上跑 verify 與 selftest | 環境 |

**合計的讀法**
- **只看不飛（步驟 3）**：架構面向估得較小（swarm_coordinator +10、panel +40–80、main_window +150–250、2–3 支測試）。規劃 UI 面向逐項估 P1–P5、P8 加圍籬，約 550–850 行正式程式加約 700 行測試。〔建議〕以後者規劃，因為它把請求組裝、清狀態、註記出口都算進去了。
- **安全閘（步驟 4）**：架構面向只估了 P6／P7／P10 本體，約 +400–700 行。安全閘面向逐項列了旁路、SITLLink API、帳本和測試，估 2,000–3,000 行（含 800–1,200 行測試）、4–7 人日。以後者為準。
- 可飛性的修正約 200–250 行，動 4–6 個檔，20–30 條測試，另要一趟 GUI SITL 實飛。
- 把兩大塊直接相加約 3,250–4,550 行（含測試）【推論：沒扣掉重疊】。
- **每次同步**：人工約 10–15 分鐘，CPU 約 4 分鐘（兩邊各跑一次 selftest），還要和另一個 session 協調時段。
- **衝突風險高**：要改的函式正好集中在未提交 diff 最多的 `main_window.py`，而且今天還有人在改。

---

## 6 需要你決定的事項

「時點」欄的意思（步驟編號同 §0、§7）：
- **接入前必須**：步驟 0 就要決定；
- **離線接入前**：步驟 3 之前；
- **安全閘前**：步驟 4 之前；
- **SITL 驗證前**：步驟 5 之前；
- **真機前**：本次範圍外，但上真機前要定；
- **不擋接入**：不影響這次的任何步驟。

### 6.1 交接 §9 的 15 項

| # | 事項 | 選項 | 〔建議〕與本次新增的依據 | 時點 |
|---|---|---|---|---|
| §9-1 | 複製哪個 harvest 版本 | master `3a001ca`／`dr-dccpp-gui` | master `3a001ca`，並把「匯出含 19」改成 NO_GO，或至少在 GUI 跳確認。dr 與 hv014 的 harvest 完全相同；換成 dr 版金樣本不會變，但要另補 LoiterExportError 轉 NO_GO、白名單拿掉 19、第二個金樣本（PV-5、PV-6、PU-03）。**代價**：以 PU-03 的 5 組 aeroplan 式幾何計，改成 NO_GO 後有 2 組會變 NO_GO（只跳確認則不會）；要可用，得選 dr／HV-014（連同上述配套），或改用逐分區起降墊（範例那種佈局；它不插盤旋只有範例單一情境的證據，見 N9） | **接入前必須** |
| §9-2 | 先 commit 再複製 | 要／不要 | 至少要有持久的凍結紀錄：在 vtol-hil-studio commit 或打 tag，或在 aeroplan 的匯入提交裡一起提交外部 manifest 檔本體。數值從 manifest 檔取，不從交接文字抄（PV-1） | **接入前必須** |
| §9-3 | 作業區大小、架次、機數 | — | 不擋接入。注意 SITL 與 CubeBlack 的任務容量是 655 項（FLY-6） | 真機前 |
| §9-4 | `boundary_tracking_margin_m` | 0／10 m | 本次沒有新資訊 | 不擋接入 |
| §9-5 | 連拍率 | — | 本次沒有新資訊 | 不擋接入 |
| §9-6 | `containment_violation`、`accept_radius_exceeds_overrun` 要不要改 NO_GO | 維持註記／改 NO_GO | 素材沒有建議改或不改，由你決定。後果要先知道：交接 §8 記載範例請求本身就有 containment_violation，改成 NO_GO 會讓範例請求本身 NO_GO，也就是目前的上層＋HARVEST 對這份範例產生不出可飛計畫；要改，得先解決交接 §8「HARVEST 不改就做不到」的第 3 項（走廊注入，F-A11）。〔評估者建議〕離線階段至少先讓 H6 的註記出口把它們顯示出來 | 真機前 |
| §9-7 | 放行要做多完整、2 s 晚放行容差、QHOVER 時放行 arm | — | 讀回移到預備階段，讀回後鎖住該機的上傳入口，放行時只核狀態（S9）；容差和 UAV-10 地面等待約 148 s 會不會自動上鎖，等 SITL 實測再定。QHOVER 時放行 arm：broadcast_check 放行的 arm 實際會走 force fallback（S3 驗證），所以〔建議〕不 force 的解鎖 API（H2）做好之前，新路徑在場時廣播 arm 一律擋下；做好之後才考慮在 QHOVER／QLOITER 放行 | 安全閘前 |
| §9-8 | ADR-0015 | (a) 兩份 HARVEST 並存 (b) ADR-0013 (c) 和 0011／0012／0014 的關係 (d) 交付方式 | (a) GUI 標籤改成「HARVEST（hv_dccpp 論文版，AASRC2026）」與「上層＋HARVEST 主線（D-80Pro，實驗）」，識別字不改（F-A8）；(b) 採範圍限定（F-A5）；(c) 寫明 ADR-0007 的前提對新路徑反轉、Pioneer 120 m 零裕度、0012 的欄位對新路徑失效、0014 的方向保證失效（F-A4、F-A6、F-A14）；(d) 記下改用 in-process 複製的取捨，並約定只同步有 tag 的版本（F-A9、PV-11） | (a)(d) **接入前必須**；(b)(c) 離線接入前 |
| §9-9 | 機載電腦的交付管道 | — | 本次沒有新資訊 | 不擋接入 |
| §9-10 | (a) Pioneer 轉場高度的裕度；(b) 作業段到 1.4 cm GSD 的兩條航帶接不接受 | — | (a) 對齊論文的 119.5 m（95a337b 的實飛證據）；在可以設定之前，新路徑只限 SITL（F-A4、FLY-4）。(b) 本次沒有新資訊：operate_altitude_off_survey 註記照舊（範例 Pioneer 第一條、最後一條航帶的作業段到 115.5／117.9 m），仍待你決定 | 真機前 |
| §9-11 | 開 §8 列的四個 HV 號 | — | 本次沒有新資訊 | 不擋接入 |
| §9-12 | 「規劃使用 0.01」 | — | 已確認，結案 | — |
| §9-13 | 任務讀回：照 §5.4 規則，還是定義讀回雜湊 | — | 新路徑改用嚴格比對器，不動契約；aeroplan 現有的有損比對器抓不到 speed_type、10% 以內的速度改動、接受半徑改動（A-03） | 安全閘前 |
| §9-14 | 帳本檔位置、要不要進 git | `data/`／`%LOCALAPPDATA%` | 交接建議的 `data/vcs_flight_ledger.json`（含 `.tmp`）可用，要加進 `.gitignore`、以 aeroplan 根目錄算絕對路徑。缺點：data_dir 依 project_root 決定，每個 worktree 各一份帳本，跨 worktree 互相看不到；替代方案是整台機器共用 `%LOCALAPPDATA%`（ENV-3、S6），兩者由你選。另外要決定：檔案不存在時是否 fail-closed（現在視同空帳本而放行，ENV-3） | 安全閘前 |
| §9-15 | 支援 PyInstaller 打包 | — | 第一版不支援；打包版必定 `VcsUnavailable`，屬 fail-closed（ENV-8） | 不擋接入 |

### 6.2 本次新增

| # | 事項 | 選項 | 〔建議〕 | 時點 |
|---|---|---|---|---|
| N1 | aeroplan 基線提交的範圍與執行者 | 和 P 改動一起提交／先做基線 | 依序做三個提交：基線（打 tag）→ 純新增 → P 接線。代理不自行提交別人進行中的修改（H9、4.8） | **接入前必須** |
| N2 | 在研討會（10/16）前還是後接 | vtol-hil-studio `CLAUDE.md:327` 寫「研討會後才整合」，交接 §0 第 5 點寫「提前」 | 會後（F-A12） | **接入前必須** |
| N3 | 和另一個 session、正在跑的 SITL 怎麼協調 | — | 確認那邊收尾、SITL 停了才動手；動手當下重跑 wiring-check，並用函式名重新定位錨點 | **接入前必須** |
| N4 | 第一版的範圍 | 只看不飛／完整上傳放行 | 只看不飛 | **接入前必須** |
| N5 | 閘門放在哪一層 | SITLLink 層預設拒絕＋release token／在 UI 層逐一修 | 放鏈路層；UI 層的 `broadcast_check` 只負責使用者體驗（H1） | 安全閘前 |
| N6 | 新路徑結果在場時，逐機面板的 AUTO／ARM、KAMIKAZE、TERMINAL SYNC、動態偵打切換、點圖起飛怎麼處理 | 停用／導到 P10 | 只看不飛階段全部停用；安全閘階段 AUTO 導到 P10，其他擋下 | 離線接入前 |
| N7 | 合格的矩形從哪來 | 專用矩形工具／取外接矩形（大聲標出多覆蓋的面積）／用多邊形編輯器匯入 JSON | 素材無偏好，由你選。外接矩形要大聲標出多覆蓋的面積；短期可先用多邊形編輯器匯入角點；不論選哪個，多區一律擋下（PU-01、F-A6） | 離線接入前 |
| N8 | 航帶方向與起點側由誰決定 | GUI 正規化頂點順序／上游提供公開的 orient、suggest 入口 | 由上游提供入口，aeroplan 不要寫死 x_T（PU-02） | 離線接入前 |
| N9 | 起降墊佈局 | 單一起降場排一列（實測 GO，但常插盤旋）／每個分區一個墊（範例那種） | 素材沒有偏好，取決於 §9-1。原因：單一起降場排一列常插 19（PU-03：5 組 aeroplan 式幾何裡 2 組），所以要看 §9-1 選的版本怎麼處理 19；照 §9-1 的〔建議〕（master＋19 改 NO_GO），這 2 組會變 NO_GO | 離線接入前 |
| N10 | 有 NFZ、THREAT 或 DEM 時 | 拒絕／允許但大聲提示 | 拒絕，依有效值契約（H3） | 離線接入前 |
| N11 | 圍籬 | 新路徑停用上傳圍籬／改由 `mission.items` 加墊建 hull 並排除 HIL；`FENCE_ALT_MAX` 用 140 還是 120 | 第一版停用；120 m 要和 §9-10 一起定（PU-08） | 安全閘前 |
| N12 | Write WPs 遇到新路徑的匯出檔 | 拒絕／改走 plan_upload／只記帳 | 拒絕；sha 放旁檔（H5） | P8 之前 |
| N13 | 讀回失敗但任務已寫入（ok=False） | 先記 pending 照樣擋／不記 | 上傳前先記一筆「上傳意圖」（A-02、S4） | 安全閘前 |
| N14 | SITL 重起時自動 `forget_link` | 是／否（交接目前規定不要手刪帳本） | 是（A-10、S6） | 安全閘前 |
| N15 | aeroplan SITL 的參數 | 只驗流程／依構型疊 ARC 參數並設風 | 只驗流程並在畫面標示；但 Hero 19 m/s 低於 AIRSPEED_MIN 這件事要大聲提示（FLY-2、FLY-3） | SITL 驗證前 |
| N16 | 時間與覆蓋率怎麼顯示 | 用 HARVEST 的數字／兩套並列；驗算停用／改用 strips | 用結果裡的值；驗算先停用（B6、B7） | 離線接入前 |
| N17 | 自起 SITL 位移到 i≥16 | 放寬到和 launcher 一致／維持拒絕只改訊息 | 維持拒絕、改訊息 | 安全閘前 |
| N18 | wiring-check 與邊界掃描的範圍 | 只掃 git 追蹤檔／略過 .claude、runs、scratchpad | 兩者都做，同時收緊規則（A-04、A-05） | 複製前（在上游改） |
| N19 | `pytest.ini` | 還原 HEAD 版／加 `--ignore` | 還原 | 不擋接入 |
| N20 | 動 `vtol_recon_panel.py`、`main_window.py` 的 session 是誰 | — | 請你確認；17:04 的修改和 `.git/lost-found` 的來源沒有查證 | **接入前必須** |
| N21 | request_id 的生成規則 | 內容雜湊／時間戳／讓使用者命名 | 會影響 sha 的可重現性 | 離線接入前 |
| N22 | 上游讓 `VcsSixKey` 帶著 `vcs_output` | 做／不做 | 做（B5） | 離線接入前（上游） |
| N23 | 長期是否收斂成一份 HARVEST | aeroplan 只留主線複本、hv_dccpp 凍結當論文版／維持兩份 | 前者（F-A8、F-A9） | 長期 |
| N24 | 「只認自起 SITL、HIL 整批拒絕」要不要先單獨接到既有上傳 | — | 可以考慮，它能保護 hv_dccpp 路徑（F-A13） | 獨立於本次 |
| N25 | 既有 hv_dccpp 也提供 D-80Pro 相機（修訂 ADR-0009）；D-80Pro 質量 | — | 這樣兩個選項才可比（F-A7）；能量模型目前沒算吊艙質量【推論】 | 長期／真機前 |
| N26 | launcher 的 VTOL 強制參數注入（ARMING_REQUIRE=1；規格是未追蹤的 `tests/test_vtol_sitl_enforced.py` 22 條）要不要現在就獨立修 | 現在獨立修／併入步驟 4 安全閘 | 獨立先修：這是 aeroplan 原本就有的缺陷，舊路徑和 hv_dccpp 路徑一樣受影響，你今天 17:07 在 GUI 起的 SITL 就停在地上（H8、FLY-1 驗證）；修完要在 GUI SITL 重驗能正常解鎖 | 不擋接入／越早越好 |
| N27 | 帳本只要有一筆紀錄就全域擋住所有鏈路（含無關機）的 AUTO 與一鍵起飛，這個可用性代價是否接受 | 接受全域擋／改成依 conn 分別擋 | 素材無建議，由你決定。注意：過期紀錄會擋住全部鏈路，會逼人去手刪帳本，反而又變 fail-open（S6） | 安全閘前 |

---

## 7 建議的接入順序與驗收

### 步驟 0：決策
- 內容：決定 §6 標「接入前必須」的各項：§9-1、§9-2、§9-8(a)(d)、N1–N4、N20。
- 驗收：決定寫進 ADR-0015 草稿或交接 v0.6；交接先修正 §2.3 列的錯誤與不完整說法，以及 §2.2 的錨點。

### 步驟 1：兩邊凍結
- **aeroplan**：照 N1、N3 先做基線提交並打 tag。驗收：
  - `git -C <aeroplan> ls-files --error-unmatch ui/workers/planning_worker.py ui/widgets/vtol_recon_panel.py docs/adr/0013-fleet-split-stays-equal-area.md tests/test_vendored_naming_boundary.py` 結束碼 0；
  - `git -C <aeroplan> status --porcelain -- mission/swarm_coordinator.py ui/main_window.py mission/sitl_link.py` 輸出為空；
  - HEAD 版 `ui/main_window.py` 裡不再出現 `HvPlanningWorker`；
  - `hv_dccpp/` 對 vtol-hil-studio 的 `docs/harvest_vendor_manifest.json` 逐檔比 sha256，69/69 相符。
- **vtol-hil-studio**：〔建議〕先做上游修正（wiring-check 收緊與掃描範圍、帳本 pending 與缺檔策略、19 改 NO_GO、ImportError 改成 VcsUnavailable），再 commit 或打 tag，然後照交接 §2.4 (1) 執行。完整命令以交接 §2.4 (1)–(3b) 為準，下面只是摘要：
  ```powershell
  & "C:/Program Files/Python314/python.exe" -m vtol_coverage_stack selftest          # 印「selftest 通過」
  & "C:/Program Files/Python314/python.exe" -m vtol_coverage_stack manifest --write
  & "C:/Program Files/Python314/python.exe" scripts/make_aeroplan_bundle.py
  $man = "var/aeroplan_bundle/manifest_<ts>.json"   # 換成上一行剛印出的路徑
  Get-FileHash $man -Algorithm SHA256
  ```
  - 驗收：
    - 如果選擇 commit，外部 manifest 的 `uncommitted` 要是 false；manifest 的 sha 和三個樹雜湊要記下來（數值從 manifest 檔取）。
    - 在 vtol-hil-studio 端對 aeroplan 的唯讀快照跑一次 wiring-check，記下失敗與警告的項目清單；同時記下 shim 測試的條數。這兩個是步驟 2 的對照基準。上游沒改 wiring-check、帳本時，預期就是 9 失敗、4 警告、60 條。

### 步驟 2：純新增複製（由你執行），不接線
- 命令：交接 §2.4 的 (2)、(3a)、(3b)。`$man` 沿用步驟 1 的值；`$src`、`$dst` 照交接 §2.4 (2) 設定。
- 驗收：
  - (3a) `scripts/make_aeroplan_bundle.py verify --manifest $man --target-root $dst` 結束碼 0；
  - `-m vtol_coverage_stack verify-bundle` 結束碼 0；`selftest` 通過，四個金樣本 sha 相同；
  - `-m pytest tests/test_vcs_shim.py -q -p no:cacheprovider` 全部通過，條數等於步驟 1 記下的上游條數（上游沒改時是 60；一定要指定檔案）；
  - `-m vtol_coverage_stack wiring-check --root .` 的失敗與警告項目，和步驟 1 記下的預期清單一致（上游沒改 wiring-check 時就是 9 失敗、4 警告），import 邊界 0 筆——這證明複製沒有改變任何行為；
  - `git status` 只多出三個目錄和 4 個 shim；提交訊息寫上外部 manifest 的 sha256、三個樹雜湊、上游的 commit 或 tag。

### 步驟 3：離線接入，只看不飛
- **內容**：
  - P1、P2：P2 在 dem／terrain 非空時直接 raise。
  - P3：選單三態、停用欄位、顯示風輸入。
  - P4：請求組裝寫成純函式，涵蓋矩形、方向、`no_fly=_hv_fence_zones()`、DEM／多區／豁免一律拒絕、uav_id 與 sysid、風用關鍵字對應；每次規劃清狀態；註記出口；不定量進度提示。
  - P5：兩段後處理都跳過。
  - P8：三份 JSON，NO_GO 也存；這一階段不出 `.waypoints`。
  - P9。
  - 顯示語意的修正。
- **新路徑結果在場時一律硬擋**：`on_sitl_upload_mission` 第一個敘述就拒絕；`_sitl_broadcast` 拒絕 auto_start、mission_start、AUTO、arm；逐機面板的 AUTO／ARM、KAMIKAZE、TERMINAL SYNC、`_on_strike_recon_trigger`、`on_sitl_upload_fence`、Write WPs 全擋；`configure_vcs` 選新路徑時清掉 `_diamond_plans`。
- **驗收**（在守衛下，用未綁定方法加假 self、假鏈路）：
  1. `VcsSixKey` 經 finished_ok 往返後，`is_vcs_result` 仍為 True。
  2. 有 NFZ、勾了避撞時，`_display_dccpp_result` 不改動新路徑的 `assembled_paths`（逐點比對）。
  3. 新路徑時呼叫 `on_sitl_upload_mission`，假鏈路的 upload 被呼叫 0 次。
  4. 逐機面板選 AUTO、按 ARM 之後，假鏈路的 `calls == []`；`_diamond_plans` 非空時，KAMIKAZE 的 `auto_start` 被呼叫 0 次。
  5. 新路徑 GO 再跑同步 DCCPP，上傳走舊程式；新路徑 GO 再跑新路徑失敗，`_dccpp_result` 已被清掉。
  6. 先注入一份舊的 hv report，再跑新路徑：顯示的 loud code 就是 result 裡那 13 個，沒有舊註記。
  7. 有 NFZ 時 RequestError；載入 DEM、兩區、勾豁免時，規劃前就被擋。
  8. 風的參數有值層級的測試，wind_from_deg 和 wind_speed_mps 不會接反。
  9. 由你在 GUI 手動規劃範例請求：約 1.5–2 分鐘後 GO；把匯出的 JSON 拿去跑 `-m vtol_coverage_stack validate --result result.json --payload payload.json`，結束碼 0。
  10. wiring-check：P2、P5 通過；這一階段 REQUIRED_CALLS 的 6 項仍失敗是預期內的。〔建議〕上游加一個「只看」模式，沒有的話就以上面這些行為測試為準。

### 步驟 4：安全閘（同一變更集）
- **內容**：P6、P7（`active_result` 是結果本體）、P10（讀回放在預備階段、不 force 解鎖、AUTO 後確認爬升）、P11–P16（定義見 §3 開頭；P12 的強制參數若已照 N26 先修，這裡只補 alive_conns 與監聽 PID）、SITLLink 新 API、帳本強化、嚴格讀回比對器、wiring-check 收緊。
- **驗收**：
  - `wiring-check --root .` 結束碼 0；同時，A-04 那組「故意接錯」的假根目錄要回 1。
  - 旁路掃描 `grep -rn --include=*.py -E "\.set_mode\(|\.arm\(\)|\.auto_start\(\)|\.mission_start\(\)" ui mission integrations cli` 只剩白名單裡的呼叫點。
  - 上傳整合測試（假 `_mav` 驅動真的 SITLLink）：三架 0 差異；A-03 的 10 個負對照全部 ok=False。
  - `plan_upload` 的情境結果和 A-09 一致，i16 和 localhost 的拒絕訊息要能和「可能是實體板」區分開。
  - 帳本：檔案不存在、pending、PermissionError、兩個實例交錯寫入，各有測試，行為符合你在 N13、N27、§9-14 的決定。
  - `pytest tests/test_vtol_sitl_enforced.py`（22 條）通過；`pytest tests/test_mission_upload.py -k "not Loopback"` 仍是 225 passed。
  - 不 force 解鎖：假 `_mav` 不回報解鎖時只送 `(1,0)` 並回報失敗，不會送 21196。

### 步驟 5：在 aeroplan SITL 上驗證流程
需要你同意起 SITL，並避開其他 session。這一步只驗流程接對了，不能當計畫的證據。
- 驗收：
  - BIN 的 PARM 裡 ARMING_REQUIRE=1，而且看得到 PreArm 訊息。
  - 放行後 RCOU C5–C8 大於 1000、RelHomeAlt 超過你定的門檻；做不到的話要大聲中止，帳本裡也不能有 `record_release`。
  - 每架的 178 都被接受（看得到「Set airspeed」訊息），或 GUI 已經大聲警告 Hero 19 m/s 低於 AIRSPEED_MIN。
  - 一鍵起飛、逐機面板的 AUTO、KAMIKAZE 都被擋下；放行順序照 `release_schedule`；只要有一架沒讀回通過，第一架也放不出去。
  - aeroplan 重開之後，帳本仍擋住一鍵起飛；SITL 重起之後，該 conn 的紀錄被核銷。
  - 量出 272–430 項讀回實際要多久，以及解鎖後在地上 QHOVER 等約 148 s 會不會自動上鎖。這兩個數字是 §9-7 的依據。
- **不在範圍**：HIL、真機。機體同構的時序驗證在 vtol-hil-studio 自己的 SITL（520–527，上層試飛用 524–526）另外做（交接 §7 第 4 點）。

---

## 8 限制與未驗證項目

- 九個面向都有交回結果，沒有缺漏的面向。環境面向沒有 high，所以整個面向都沒經過對抗驗證；全報告原始定為 medium 以下的發現也都沒有對抗驗證（由 high 降為 medium 的那幾條有驗證過）。
- 行號和行為是 17:05–17:25 的快照。那段時間另有 session 在改 aeroplan，而且 GUI 和 3 台 SITL 在跑，耗時數字是在這種負載下量的。17:04:53 對 `vtol_recon_panel.py` 的修改、17:07:50 `.git/lost-found` 的來源，都沒有查證。
- 沒有實際驗證的推論：
  - 韌體對 19 m/s 是「拒收」而不是「夾到下限」（FLY-2）；
  - 讀回 277 項要 5.5 s 以上（S9）；
  - 時間扭曲下間距會跌破 100 m（FLY-3）；
  - PyInstaller 的行為（ENV-8）；
  - QGC／Mission Planner 是否接受 `#` 註解行；
  - 掃描方向造成約 25% 條帶增量（F-A6）；
  - 盤旋半徑落差帶來的 10–20 s 時間差（PU-03）；
  - 耗時和交接 50 s 的差異來自機器負載（C1）。
- 沒跑或沒查的：沒起 SITL 或 GUI；沒在 3.10–3.12 下執行；沒在 SCHED_LOOP_RATE 50 下完整飛過一趟 VTOL（過去成功的 HV4SITL* 都是自組 launcher、200 Hz）；沒逐點比對 6 鍵顯示路徑和 `mission.items` 的幾何；沒查真板 Hero 在 TECS_TUNE 讀回的參數值。
- 證據多半來自單一情境（範例請求）。航線交錯有改善、盤旋插入的頻率，都不能從單一情境推廣。
- 有人提到 conftest 裡有「Qt ABI 自檢、time 還原 autouse」，但在根目錄、`tests/` 和 13 個 worktree 的 conftest 裡都找不到，這句話指的是什麼不清楚。
- 本報告沒有修改兩個 repo 的任何檔案。

---

## 附錄 A 被推翻的說法

沒有任何一條發現被整條推翻：所有對抗驗證的結論不是 confirmed 就是 partially。以下是驗證時被修正掉的說法，正文已經不採用：

- 三個 blocker（S1 逐機面板、FLY-1 SITL 起不來、F-A1 未納管）都不到 blocker：前兩者的後果限於 SITL、事後查得到；後者不會造成事故。
- B1「避撞預設就會把高度抬到 212.5 m」：面板預設值下是 0 衝突；212.5 m 要用非預設參數才會出現。
- PU-03「aeroplan 式輸入一定立刻插入盤旋」：5 組幾何裡只有 2 組插；放行時刻不受影響；秒數截斷只差 0.79 s。
- PU-01「aeroplan 沒有產生合格矩形的步驟」：多邊形編輯器的「匯入角點 (JSON)」可以做到。
- PU-02「x_T 只有模組內部知道」：它可以由公式推導出來，但仍〔建議〕由上游提供入口。
- F-A1「論文版程式無法還原」：`hv_dccpp` 和 7 支 `tests/hv_dccpp` 守衛測試已收在 vtol-hil-studio 的 `765b04e`。
- B4「匯出檔多出 DO_SET_HOME」：seq 0 是 home 佔位。另外，「VTOL 匯出讀的是 DCCPP 分頁的轉換高度」只對 DCCPP 分頁的匯出成立。
- DRIFT-1「KAMIKAZE 連 HIL 也包括在內」：這是 aeroplan 原本就有的行為，不是新路徑帶進來的風險。
- DRIFT-2、S5「交接把工作樹行號寫成 HEAD 行號」：交接明寫是工作樹。
- PV-1「版本變動很頻繁」「複製後無法得知是哪一版」：三份 manifest 是一次迭代裡產生的，之後沒再變；複製後靠樹雜湊仍能識別版本。
- PV-2「用 git add -p 只挑 P 的改動不可行」：沒有人提議這樣做，而且選擇性暫存可以用 `git apply --cached` 非互動完成，只是解決不了相依問題。
- FLY-2「真板上 Hero 的 AIRSPEED_MIN 可能 ≥ 19」：板子參數是 12。`hv_sitl_flight.py` 記的是「夾到下限」，不是拒收。
- S4 的三種情境：只有 (a) 匯出檔經 Write WPs 寫回是實際會發生的；其他兩種的前提很窄。
- A-01 的位置寫成「HUD 卡片的詳細面板」，實際是 SITL 分頁的「Command」子分頁。
- F-A3 說範例有 13 則 notes、12 則 loud，實際是 14 則、13 則 loud。

## 附錄 B 主要命令與探針

**git（全部唯讀）**
- `git -C <aeroplan> diff --numstat HEAD`
- `git show HEAD:<檔> | grep -c <符號>`，和工作樹比對次數
- `git ls-files --error-unmatch`、`git ls-files -d`、`git ls-files --others`
- `git check-ignore -v --no-index`
- `git rev-list --all --objects`
- `git rev-list --count master..dr-dccpp-gui`、`git log -1 master -- harvest`
- `GIT_OPTIONAL_LOCKS=0 git status --porcelain`

**模組命令**（在守衛下執行，或在暫存的模擬佈局裡執行）
- `python -m vtol_coverage_stack wiring-check --root .`，或直接呼叫 `wiring_check.main(['--root', <root>])`
- `selftest`、`verify-bundle`
- `scripts/make_aeroplan_bundle.py verify`
- `pytest tests/test_vcs_shim.py -p no:cacheprovider`，prepend 和 importlib 兩種模式各跑一次
- `pytest tests/test_mission_upload.py -k "not Loopback"`

**旁路普查**
- `grep -rn --include=*.py -E "\.set_mode\(|\.arm\(\)|\.auto_start\(\)|\.mission_start\(\)" ui mission integrations cli`

**探針腳本**：都在評估用的暫存目錄 `work/…`，不在任何 repo 裡。
- 環境：`forbidden_scan.py`、`import_all.py`、`run_colocated.py`、`collect_probe.py`
- 乾跑 A：`upload_sim.py`、`gate_sim.py`、`fakewire/`、`refute_A01/panel_bypass.py`
- 乾跑 B：`run_plan.py`、`run_consumers.py`、`run_display_ca.py`、`run_qt_flags.py`、`refute_colocB/run_refute.py`
- 安全閘：`exp_safety.py`（E1–E7）、`refute_cmdpath/repro.py`
- 規劃 UI：`rect_probe.py`、`aeroplan_style_request_probe.py`、`fence_probe.py`、`qt_signal_probe.py`、`loiter_freq_probe.py`
- 可飛性：`warp.py`、`probe2.py`、`probe4.py`、`preflight_check.py`、`fly_rebut/parse_bin.py`（讀 i0 BIN 的複本）
- 架構：`e9_scope.py`、`manifest_drift.py`、`R-arch/r_checks.py`
- 版本：`run_plan.py`（master_root／dr_root 兩份）、`sim_verify.py`、`PV_rebut/drift.py`

---

## 修訂紀錄

21 條審查意見全部採納，沒有不改的項目。逐條處理如下：

1. §2.1：重數素材，版本與流程確實是 26 條（confirmed 18）。該列改成 26｜18｜6｜2｜0，合計改成 208｜118｜68｜20｜2。
2. §4.5 PU-03：233.793 s、200 m 改標為 aeroplan 式輸入 A（單一起降場三墊排一列、home 在西 900 m）；並寫明金樣本範例沒有插。
3. H9 證據、§2.5 C6、附錄 A 的 F-A1 一行：改成「aeroplan 沒追蹤，但 765b04e 有同內容」，兩邊都找不到的只列 swarm_coordinator.py 現行內容、planning_worker.py、ADR-0013／0014、3 支頂層守衛測試；拿掉 test_vendored_naming_boundary.py 的重複計數。另外對兩個 repo 跑了唯讀 `git log --all --name-only`，確認 ADR-0013／0014 和 3 支頂層測試在兩邊任何 ref 都查無，765b04e 確實含 tests/hv_dccpp。
4. §2.3：拆成「錯誤」與「成立但不完整」兩表，:215-217 那條移到後者，內容照審查意見改寫。
5. §2.5 C7、附錄 A：改成 DRIFT-2、S5。
6. §0：建議路線改成步驟 0–5，和 §5、§6、§7 一致。
7. §7：步驟 2 的 wiring-check 與 shim 測試驗收，改成對照步驟 1 記下的上游預期清單與條數；步驟 1 加上「記下清單與條數」。保留「上游沒改時是 9／4、60」當作預設值，因為上游修正只是〔建議〕。
8. §7 步驟 1 的 PowerShell：補上 `$man` 的定義，註明完整命令照交接 §2.4 (1)–(3b)；步驟 2 註明 `$man` 沿用步驟 1 的值、`$src`、`$dst` 照交接 §2.4 (2) 設定。
9. §0、H1、H2 標題與證據：「一律 force」改成「PreArm 擋下時一律改送 force」。
10. §1：污染掃描的描述照審查意見改寫。
11. §2.5 C1：負載歸因改標【推論】，並寫明交接 50 s 的條件不明；§8 的推論清單同步加上這一項。
12. H3〔建議〕加上停用清單要刪掉「禁航區」、改處理「地形」的那句；§2.3「錯誤」表也加了一列。
13. §6.1 §9-7：加上 arm 的建議，並接上 H2／S3。
14. §6.1 §9-6：改成「素材沒有建議，由你決定」，寫明後果和前置條件；H6 那半句標為評估者建議。審查意見寫「論文範例」，報告改用「範例請求」：素材顯示範例請求（1.2 × 1.5 km）和論文的 1.243 km² 矩形不是同一個（F-A7、upper.json）。
15. §6.1 §9-1：補上代價（5 組裡 2 組會變 NO_GO）；註明「逐分區起降墊不插盤旋」只有範例單一情境的證據。§4.8 PV-6 加上交叉引用。
16. §6.1 §9-10：補上第二個子題，標「本次沒有新資訊」。
17. §6.2：新增 N26；H8、§0、§7 步驟 4 加上交叉引用。
18. §6.1 §9-14：補上 worktree 各一份帳本的缺點與 `%LOCALAPPDATA%` 替代方案；§6.2 新增 N27（素材無建議）；§4.4 S6、§7 步驟 4 加上交叉引用。
19. §6.2 N7、N9：照審查意見改寫，N9 寫出原因。
20. 開頭標記說明：改成「§4 除了標『已對抗驗證』的之外，全部沒經過對抗驗證」，並補上 PU-02、PU-03、PU-07 漏掉的「已對抗驗證」。同時修正 §1、§8 原本「medium 以下都沒驗證」的寫法，改成「原始定為 medium 以下」，因為由 high 降為 medium 的幾條其實驗證過。
21. §3 開頭：加入 P11–P16 對照；§5 開頭也加上引用。
