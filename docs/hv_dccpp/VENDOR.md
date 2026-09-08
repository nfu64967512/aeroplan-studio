# hv_dccpp 是 vendored 進來的（不是外部相依）

**來源** `C:/Users/joy46/Documents/Projects/vtol演篹法`　**版本** 0.1.0　**日期** 2026-09-06

原本 AeroPlan 是用 `pip install -e "<那個資料夾>"` 把 `hv_dccpp` 當外部套件引用，
程式碼住在 AeroPlan 之外。依使用者決定改為 **vendor**：整份複製進本專案，
`pip uninstall hv_dccpp` 之後 `import hv_dccpp` 解析到 `aeroplan-studio/hv_dccpp/`。

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

## vendored 修改（原本只有 1 條，2026-09-08 起 2 條）

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
