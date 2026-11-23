# ADR-0010: 演算法定名 HARVEST（暫訂）

**狀態**: **Accepted（2026-09-09 使用者定調並執行）**
**日期**: 2026-09-07
**決策者**: @james zheng

## Context / 背景

現行名稱 `HV-DCCPP`（Heterogeneous-VTOL Dynamic Complete Coverage Path Planning）
是為了標示與 Wang et al. 2025《Defence Technology》47:197-215 的 DCCPP 的關係而取的。
要寫成學術論文時它有兩個問題：

1. **把貢獻歸類錯了。** 規格書 §1.1 的目標函數是 `min C_max`（最小化最大落地時刻），
   受限於覆蓋、運動學、高度、能量、鏈路半徑、空域、去衝突、氣象八組約束。
   這是**異構機隊、最小化最大完成時間的受限指派與路徑問題**，覆蓋只是 C1 那一條。
   叫它 coverage path planning 等於自己把貢獻縮小成 DCCPP 的變形。
2. **字面上太貼近基礎方法。** 名稱幾乎逐字包含 DCCPP。

## Decision / 決策

**HARVEST** — Heterogeneous Allocation and Routing for VTOL Endurance-constrained Survey Tasks

| 字母 | 對應 | 為什麼是它 |
|---|---|---|
| H | **H**eterogeneous | 兩個構型（Pioneer PNP / HERO PNP）性能不同 |
| A | **A**llocation | 規格書 §9「任務配置」，含 §9.3 構型指派與架數 |
| R | **R**outing | §8 三類路徑與時間成本、§10 排程 |
| V | **V**TOL | 真實垂直起降剖面，非跑道起降 |
| E | **E**ndurance-constrained | ⭐ 核心新機制：`λ_floor = ⌈T_single/(κ·T_use − t_in − t_RTB)⌉` |
| S | **S**urvey | 作業型態是面積覆蓋 |
| T | **T**asks | 任務層級的產出（含起降時刻、備降、能量餘裕、GO/NO-GO） |

論文完整寫法：

> HARVEST: heterogeneous allocation and routing for VTOL endurance-constrained survey tasks

### 為什麼不是別的

* `FECA`（Flight-Envelope-Constrained Allocation）：英文口頭報告時與 *fecal* 太近。
* `RAVEN`：RQ-11 Raven 本身就是知名偵查無人機，在偵查題目上撞名。
* `VECTOR`：展開最完整，但這個字在文獻檢索裡雜訊太大，日後不好被引用。
* `VESTA` / `FEATHER`：讀起來像專案代號而不是方法名。
* 純描述式（`EC-HVMP`）：精確但口頭講不出來。

⚠ `envelope` 在任務規劃文獻裡已有另一個意思（分配給使用者的資源額度剖面），
所以本方法一律寫 **endurance-constrained**，不用 envelope 當關鍵字。

⚠ `Survey` 是刻意選的：本方法做的是**面積覆蓋**，用 `Surveillance`（持續監視）
會描述錯。需要軍事語境時在副標寫 reconnaissance survey，不要動主名。

## Consequences / 後果

### 論文端

Related Work 必須有這一句（改名不能取代交代血緣，審稿人一定找得到基礎方法）：

> HARVEST 沿用 DCCPP 的區域分解與完全覆蓋判準，並以續航與航程約束下的
> 構型與架數指派、真實垂直起降剖面、4-D 去衝突，取代其同構型假設。

**不要**把方法描述成全新的覆蓋演算法。覆蓋分解與條帶生成不是本方法的新東西，
那樣寫在審查時很危險。

### 程式碼端（**2026-09-09 已執行**，見文末「執行紀錄」）

`hv_dccpp` / `HvDccpp` / `HV-DCCPP` 出現在 **106 個檔案、1383 處**。
改名時分兩層，不要一次全掃：

| 層 | 動作 |
|---|---|
| AeroPlan 這一側 | `use_hv_dccpp` → `use_harvest`、`hv_params` → `harvest_params`、`HvPlanningWorker`、面板文案、ADR、`runs/artifacts/*/README.md` 全部改 |
| vendored 套件 `hv_dccpp/` | **維持原名**。它是上游套件，`docs/hv_dccpp/VENDOR.md` 記著重新同步程序，改目錄名等於分叉 |

對照關係要寫進 `docs/hv_dccpp/VENDOR.md`：HARVEST 是本專案對這套方法的名稱，
`hv_dccpp` 是上游套件名。

### 還沒決定的

* 投稿場域。HARVEST 帶農業語感；精準農業本來就是覆蓋路徑規劃的主流應用，
  所以遙測／機器人／一般航太期刊沒問題，但投 *Defence Technology*（基礎論文的同一本）
  時這個字會不太搭，屆時可改用 HAVEN 或 HARP。
* 是否要同時定一個**問題名**（例如 endurance-constrained heterogeneous VTOL
  survey planning）與**方法名**分開。目前先只用一個名字。


---

## 執行紀錄（2026-09-09）

使用者：「我們的 HV-DCCPP 應該都要改成 HARVEST 演算法」。依上表分兩層執行。

### 已改（AeroPlan 這一側）

散文字串 `HV-DCCPP` → `HARVEST`：**33 個檔、143 處**。涵蓋 `ui/`（面板文案、
log 訊息）、`mission/`、`scripts/`、`tests/`（我們自己的）、`research/`、
`requirements.txt`、`aeroplan_studio.spec`、`runs/artifacts/*/README.md`、
以及 `docs/` 裡我們自己寫的文件。

### 刻意沒改（附理由）

| 對象 | 為什麼 |
|---|---|
| `hv_dccpp/`（套件本體，9 檔） | 上游套件。`VENDOR.md` 記著重新同步程序與 4 條 vendored 修改，改目錄名等於分叉 |
| `tests/hv_dccpp/` | 同上，是上游自己的測試 |
| `tools/mock_autopilot.py` | 同上，隨套件複製進來的 |
| `docs/hv_dccpp/` 的上游隨附文件（設計文件／規格書／模組契約／接入手冊／進度紀錄／VENDOR） | 同上；改它們會讓與上游的比對失效 |
| `runs/artifacts/*/hv_sitl_*.py`（30 檔） | 那是每趟航次**當下**的腳本快照，是證據不是程式碼 |
| `docs/adr/000{7,8,9}`、`README.md` | ADR 是**有日期的紀錄**。改寫舊 ADR 的內文等於竄改當時的決策脈絡；改為在文件頂端加一行對照說明 |
| `data/logs/*.log` | 歷史紀錄 |

### ⚠⚠⚠ 刻意**沒有**動的一類：跨進 vendored 轉接層的參數鍵

`hv_dccpp/integration/aeroplan_adapter.py` 只認得四個 `hv_*` 名字：
`hv_cfg`、`hv_class`、`hv_dccpp`、`hv_uav_id`。面板送進來的其餘 `hv_*` 鍵
（`hv_waiver_max_agl_m`、`hv_min_fw_agl_m`、`hv_dh_band_min_m` …）由**我們自己的**
`swarm_coordinator._hv_get` 讀。

**只改一邊會讓全部參數靜靜退回預設值**，而那正是本專案「有效值契約」明文禁止的
靜默失敗（見 `docs/hv_dccpp/` 的契約說明與相關 ADR）。這些內部鍵改名是**純風險、
零可見效益**，ADR-0010 原文也沒有要求 ⇒ 維持原名。
