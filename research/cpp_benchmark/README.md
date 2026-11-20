# 覆蓋路徑規劃效率基準（論文用）

2026-09-07 建立、2026-09-08 從 session 暫存目錄搶救進 repo。

## ⚠⚠⚠ 為什麼有這個目錄

這整套原本只存在 **session 專屬的暫存目錄**（`AppData/Local/Temp/claude/.../scratchpad/bench`），
25 個檔案還互相用那條絕對路徑 import。session 一結束就全沒了 ——
而這是論文效率比較那一章的全部證據。搬進 repo 並把路徑改成相對之後，
`matrix.py` 重跑 `spiral_v2 S1 4` 與 `darp_v2_r1 S3 8`，
`total_km` / `coverage` / `scan_rate` 與封存的 CSV **逐位相同**，搬移無損。

## 這是什麼

16 個方法 × 5 種作業區形狀 × 機隊規模 = **228 格**的效率矩陣。
所有長度與覆蓋率一律由 `bench.metrics` 這把**唯一的尺**量，
沒有任何方法自己報成績（自報分數是這類比較最常見的作弊來源）。

共同設定：高度 140 m、HFOV 60° ⇒ 刈幅 161.7 m、條帶間距 113.2 m（側向重疊 30%）、
參考速度 20 m/s，每條航跡都從 home (0,0) 出發並回到 home
（已稽核：所有方法的 `|first−home|` 與 `|last−home|` 最大值都是 0.0 m）。

情境：

| id | 作業區 | 機隊 |
|:--|:--|:--|
| S1 | 單一 5.0×4.0 km（20 km²） | 2, 4, 8, 16 |
| S2 | 4 塊 1.25×4.0 km，分離（20 km²） | 4, 8 |
| S3 | 細長 8.0×1.0 km（8 km²） | 4, 8 |
| S4 | L 形凹多邊形，4×4 減 2×2（12 km²） | 4, 8 |
| S5 | 大面積 10×8 km（80 km²） | 8, 16 |

## ⚠⚠⚠ 哪些數字還能用，哪些已經過期

**15 個基準線（spiral / grid+NN / DARP / boustrophedon 各家變體）：仍然有效。**
它們是自足的，只用 shapely + numpy + 標準庫，**不 import 本專案任何規劃器**，
所以不受 repo 改動影響。上面那兩格重跑逐位相同就是證據。

**HARVEST 與 legacy DCCPP 那兩列：已經過期，不可以直接進論文。**
封存表格自己的 provenance 警告就寫著：那兩個是對**工作樹 + 未提交修改**量的
（HEAD `c565319` 加未提交，`mission/swarm_coordinator.py` md5 `0c9cb713…`），
而且當時另一個 session 正在改同一批檔案。2026-09-08 之後 `swarm_coordinator.py`
又有大改（最低固定翼作業高度可調、高度帶下限、切區三個缺陷修正、
禁航區子區改整數 id），所以那兩列**必須對一個釘死的 commit 重跑**。

## 重現方式

```bash
cd research/cpp_benchmark
JOBS=10 PYTHONUTF8=1 python -u run_matrix.py     # 228 格，一格一個子行程
python -u make_table.py                          # 產生 results/matrix_table.md
```

單格：`python matrix.py <method> <scenario> <n>`，輸出一行 `@@ROW@@{json}`。

確定性：當時兩次獨立跑完整矩陣，**228 格的 `total_km` 與 `scan_rate` 全部逐位相同**，
所以表裡的差異都不是 run-to-run 噪音。

## 目錄

| 路徑 | 內容 |
|---|---|
| `bench.py` | 唯一的尺：`metrics()`、`Track`、共同幾何工具 |
| `matrix.py` / `run_matrix.py` / `make_table.py` | 矩陣的 worker / driver / 表格產生器 |
| `algo_*.py` | 15 個基準線實作。`_v2` / `_r1` / `_r2` 是後續審查版；表裡各版都獨立列名 |
| `gt_harness.py` | 拿 legacy DCCPP 當 ground truth 的對照 harness |
| `audits/` | 公平性稽核與消融：起降點一致、迴圈退化、螺旋內縮、轉彎計費、DARP 壓力測試 |
| `results/` | 封存的 CSV 與表格（`matrix_table.md` 是論文表 0 的來源） |
| `notes/` | 當時的執行紀錄與逐項檢查輸出，保留供追溯 |

## 已知的方法學限制（審稿人會問）

* `scan_rate` 與 `overhead` 是**純幾何**（零轉彎半徑折線）。`*_r200` 那幾欄是
  補上 200 m 轉彎半徑之後的值，兩者要分開講，不可以混用。
* `overhead` 的理論下限在 pitch = 0.7×swath 時是 **1.429**，不是 1.0。
* 覆蓋率 < 0.99 的格子其效率數字沒有意義，已排除在排名與勝場之外；
  回傳空計畫的格子計 0 分（沒覆蓋不該被當成缺值獎勵）。
* 所有基準線都是我們自己的實作，不是原作者釋出的程式碼。
  拿它們當「文獻方法」講的時候必須說清楚這一點。

* ⚠⚠⚠ **`DCCPP` / `DCCPP_mix` 兩列不檢查能量，所以 `coverage` 一欄不對等。**
  `core/dccpp/` 的**十個 .py 檔**對 `energy|battery|endurance|fuel` 是 **0 次命中**
  （對照：`hv_dccpp` 有五個檔各 31–74 次）。後果在 S1 n=2 這格看得最清楚：

  | 方法 | coverage | max_km |
  |---|---|---|
  | DCCPP / DCCPP_mix | 1.0 | **104.25** |
  | HARVEST / HARVEST_mix | 0.5277 / 0.5238 | 59.06 / 58.83 |

  DCCPP 要求每台飛 **104.25 km** 才拿到 1.0，比 HARVEST **拒絕**的 59.06 km 還長 77%；
  HARVEST 那 0.5277 是 `hard_feasible` 回 `H4_energy` 之後**明講**蓋不完
  （`uncovered=[('Z3_1','no_feasible_insertion')]`、`validate.passed=False`、
  子區印 `all=0.0000 FAIL`，三個管道都響）。
  ⇒ **「HARVEST 覆蓋率輸給 DCCPP」這句話不可以寫進論文**，那是「會檢查續航」對上
  「不檢查續航」。要比就得先給基準線補一個能量檢查，或在表上明標這個差異。
  重跑：`grep -ric "energy" core/dccpp/*.py`（全 0）、
  `cd research/cpp_benchmark && python -u matrix.py HARVEST S1 2`。
  查證日 2026-09-10；細節見記憶 `coverage-ceiling-root-cause`。

---

## 2026-09-08 更新：公平性修正、重跑、以及兩個新欄位

### 一、修掉一個讓基準線被綁手的符號錯

`bench.min_width_angle` 與 6 個基準線各自的複本都把「旋轉 −th 之後的投影」
寫成 `-s*x + c*y`（其中 `s = sin(-th) = −sin(th)`），等於量「對 x 軸鏡射後」的
寬度。旋轉矩形實測寬度多算 **95%**。`algo_darp.py` 用的是 `s, c = sin(th), cos(th)`
再 `-s*x + c*y`，那是**對的**，沒有動它。

⚠ 比較論文裡不可以讓自己的方法贏在對手被綁手，所以基準線一律修好重跑。

影響很小但真實 —— 五個情境全部軸對齊（sin 項在 0° 消失），寬度**選擇**不受影響，
但等寬邊上的 argmin 取捨會變。228 格重跑後有 8 格動了，全部在 S4（L 形）：

| 方法 | 格 | 舊 | 新 | Δ | 覆蓋率 |
|---|---|---|---|---|---|
| boustrophedon | S4 n=4 | 126.46 | 127.21 | +0.59% | 0.9948 → 0.9975 |
| boustrophedon | S4 n=8 | 136.86 | 137.53 | +0.49% | 0.9930 → 0.9975 |
| grid_nn | S4 n=4 | 117.80 | 118.54 | +0.63% | 1.0 |
| grid_nn | S4 n=8 | 126.59 | 125.80 | −0.62% | 1.0 |
| grid_nn_v2 | S4 n=4 | 114.13 | 114.36 | +0.20% | 1.0 |
| grid_nn_v2 | S4 n=8 | 120.92 | 125.84 | +4.07% | 1.0 |
| grid_nn_v2b | S4 n=4 | 112.59 | 113.49 | +0.80% | 1.0 |
| grid_nn_v2b | S4 n=8 | 125.42 | 127.06 | +1.31% | 1.0 |

兩格 boustrophedon 的覆蓋率因此**跨過 0.99 的門檻**，回到排名內。
其餘 190 格逐位不變。舊表封存在 `results/matrix_results_ARCHIVE_2026-09-07.csv`。

### 二、專案那 4 列已用新的工作樹重跑

24 格 DCCPP **逐位不變**（沒動到那支規劃器）。24 格 HARVEST 有 22 格變了，
其中 **6 格從「空計畫」變成有計畫** —— 那正是 `split_area_for_fleet` 的效果
（混編機隊 + 單一胞會產出空計畫，見 ADR 與進度紀錄）。

### 三、新增 `self_uncovered` / `self_validate` 兩欄

⚠⚠⚠ **方法學修正。** 舊版 `run_planner` 只取 `assembled_paths`，把規劃器**自己**
的「這幾個胞我蓋不到」訊號（`plan.uncovered`）整個丟掉。後果是
HARVEST S1 n=2 這種「機隊吃不下這塊地」的格子，在表上看起來像是
「交出了一份只蓋 52% 的計畫」，而實際上規劃器有明講 `no_feasible_insertion`
且 `validate` 回 `passed=False`。

**會宣告「我蓋不完」的方法，不該和默默少蓋的方法記成同一件事。**
基準線沒有這個概念（它們只會給你一條路徑），所以這兩欄對基準線是空的 ——
這件事本身就是論文可以講的一個差異。

重跑後 48 格專案方法裡有 **7 格 `self_validate = FAIL`**，而覆蓋率 < 0.99 的只有 4 格。
另外 3 格是**覆蓋率看不見的失效**，而且全部發生在異質（`_mix`）組態：

| 格 | coverage_ok | energy_ok | conflicts | 真正的原因 |
|---|---|---|---|---|
| HARVEST_mix S3 n=4 | True | True | **1** | B3–B4 於 t=1761 s 水平只差 **13.96 m**（垂直 40 m），4-D 去衝突判為衝突 |
| HARVEST_mix S1 n=8 | True | **False** | 0 | 能量預算超支 |
| HARVEST_mix S5 n=16 | True | — | — | 同類（未逐項展開） |

⇒ 覆蓋率是**必要不充分**的驗收條件。這三格拿任何以覆蓋率為準的尺去量都會是綠燈。
