# ADR-0010: 演算法定名 HARVEST（暫訂）

**狀態**: Proposed（暫訂，未執行改名）
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

### 程式碼端（尚未執行）

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
