# 納管準備（label `sorties2`）—— **我沒有 commit，也沒有 `git add`**

HEAD 仍是 `5e0295f5eb63e6597a787120d66526d3e4b17e7f`（分支 `quad_swarm`），
staged 0 筆，stash 未動。以下是給 lead 的清單。

---

## 1. 我在工作樹改的原始碼：只有一支

**`scripts/hv_sitl_run.py`** —— 本 label 的改動是三段（`sorties2/src/spawn_tol.diff`，
139 行的 unified diff，對照 `src/hv_sitl_run.py.before` 與 `.after`）：

1. 模組常數區新增 `SPAWN_TOL_DEFAULT_M = 2.0`、`SPAWN_TOL_M`（讀 `HV_SPAWN_TOL_M`）、
   `SPAWN_RIGID_TOL_M = 0.5`，含 12 行的理由註解與 `<= 0` 的防呆。
2. 生成點迴圈：新增 `settle_trace`（只記錄、不判定）與逐機 `d_north_m` /
   `d_east_m` / `err_m_full` / `bearing_deg`；把兩處寫死的 `2.0` 換成 `SPAWN_TOL_M`。
3. 迴圈之後：新增 `pairwise`（規劃 vs 實得間距）與 `rigid_translation`；
   `spawn_readback.json` 多寫 `tol_m` / `tol_default_m` / `tol_overridden` / `tol_source`；
   覆寫生效時印 `⚠⚠⚠` 警告，且此時剛體平移檢查升格為硬閘門。

**預設行為未變**（已驗證）：
* 不設 `HV_SPAWN_TOL_M` ⇒ `SPAWN_TOL_M == SPAWN_TOL_DEFAULT_M == 2.0`。
* `HV_SPAWN_TOL_M=6` ⇒ 6.0，`tol_source == "HV_SPAWN_TOL_M"`。
* `HV_SPAWN_TOL_M=0` ⇒ 匯入即 `SystemExit`（exit 1）。
* 預設路徑**沒有新增任何失敗模式**：剛體平移檢查在未覆寫時只記錄不擋飛。
* `py_compile` 通過；`HV4W55R` 是在**不給** `HV_SPAWN_TOL_M` 的情況下飛完的
  （生成點 0.134/0.183/0.215 m、容差 2.0），即實機驗證了預設路徑仍然正常。

⚠ **`git diff scripts/hv_sitl_run.py` 會顯示 431 insertions / 27 deletions，
那不全是我的。** 這支檔案在我接手前已經有 `c2_wind` 的 SITL 風場補丁
（`HV_SIM_WIND_*`、`_write_wind`、風場讀回閘門、`flight.csv` 兩個新欄位）與
`prep_runner` 的 `HV_PLAN_CEIL_M` / `HV_PLAN_ALT_M`。我這一輪只加了上面三段
（`spawn_tol.diff` 是**只含我這一輪**的差異）。

⚠ 另外兩支也是髒的、但**不是我改的**：`scripts/hv_sitl_scenarios.py`（+136，
`beamidp_1a2b_1195`）與 `scripts/hv_sitl_report.py`（+93）。驗證者已指出交接說明
漏掉 `hv_sitl_report.py`；要納管四趟新證據就必須一起納管它，因為那四趟的
`separation_from_tracks.json` / `coverage_from_tracks.json` 是它產生的。

---

## 2. 新增的未納管證據（全部 trackable，`git check-ignore` 實測不被忽略）

| 路徑 | 檔數 | 內容 |
|---|---|---|
| `runs/artifacts/HV4W55S/` | — | 5@180 架次全套產物（plan、params_readback.csv、wind_readback.json、spawn_readback.json、mission_upload.json、release_schedule.json、三機 flight.csv、separation/coverage、compare.png、隨產物封存的 `hv_sitl_*.py` 與 `git_state.txt`） |
| `runs/artifacts/HV4W55R/` | — | 天花板 120.0 對照架次全套產物 |
| `runs/artifacts/PAPERPLAN1/sorties2/` | 39 | 本 label 的分析、腳本、文件與 log |

三者合計 **74 + 39 = 113 個檔、1 854 482 B（1.77 MiB）**。
`.BIN`（`sitl/vtol/i10x/logs/000000{19,20}.BIN`，每趟約 105 MB）**不要納管**，
既有 `.gitignore` 也沒有把它們放進來。

`sorties2/` 內的檔案：

* `ACCEPTANCE.md` —— 兩趟逐項驗收 + 天花板對照表（**主要交付物**）
* `SPAWN_ANNOTATION.md` —— U1 要的生成點位移註記（**主要交付物**）
* `spawn_record.json` / `ceiling_control.json` —— 彙整記錄
* `HV4W55S_accept.json` / `HV4W55R_accept.json` —— 逐閘門
* `HV4W55S_binalt.json` / `HV4W55R_binalt.json` / `HV4W55_recheck_binalt.json`
* `HV4W55S_binsep.json` / `HV4W55R_binsep.json` / `HV4W55_binsep_from_prior_label.json`
* `HV4W55S_windaxis.json` / `HV4W55R_windaxis.json` / `HV4W55_windaxis.json` / `HV4W05_windaxis.json` / `HV4W00_windaxis.json`
* `HV4W55S_spawn_probe.json` / `HV4W55S_spawn_arc.json` / `exported_alt.json`
* `scripts/`（7 支：3 支沿用 `sorties` 標籤只改輸出目錄，4 支本 label 新寫）
* `src/hv_sitl_run.py.before` / `.after` / `spawn_tol.diff`
* `logs/`（兩趟 `fly` 與 `report` 的完整 stdout/stderr，以及 `.t0`/`.t1` 牆鐘）

---

## 3. 建議的納管方式（兩個 commit，指令**未執行**）

```bash
# (1) 原始碼：生成點容差的明示覆寫
git add scripts/hv_sitl_run.py
git commit -F - <<'MSG'
feat(sitl): 生成點容差可明示覆寫（HV_SPAWN_TOL_M），並新增不可覆寫的剛體平移檢查

預設仍是 2.0 m，不給環境變數時行為與改動前相同。HV4W55S（5 m/s @ 180°）的三架機
在解鎖前被 SITL 地面模型移離 --home 約 5.0–5.1 m，使該閘門在解鎖前就 SystemExit；
使用者決定接受該位移並飛完這一格，但不接受「靜靜放寬預設」。

- HV_SPAWN_TOL_M：不給 ⇒ 2.0 m；給了 ⇒ 終端印 ⚠⚠⚠ 警告，且 spawn_readback.json
  記下 tol_m / tol_default_m / tol_overridden / tol_source 四欄，事後看產物即知。
- 逐機記錄位移向量（d_north_m / d_east_m / bearing_deg / err_m_full）與解鎖前的
  settle_trace（時間、兩軸位移、機首、地速、空速、飛控風估計）。只記錄、不判定。
- 新增 pairwise（規劃 vs 實得間距）與 rigid_translation（門檻 0.5 m，不吃
  HV_SPAWN_TOL_M）。容差被明示放寬時，它升格為硬閘門 —— 因為「規劃分了起降站、
  SITL 沒分」正是會把間距打爆的那種不一致，放寬逐機容差擋不住、間距檢查擋得住。
  預設路徑不新增任何失敗模式。

證據與機制調查：runs/artifacts/PAPERPLAN1/sorties2/SPAWN_ANNOTATION.md
MSG

# (2) 證據
git add runs/artifacts/HV4W55S runs/artifacts/HV4W55R runs/artifacts/PAPERPLAN1/sorties2
git commit -F - <<'MSG'
runs: HV4W55S（5@180 橫風格）與 HV4W55R（天花板 120.0 對照）兩趟航次與分析

HV4W55S：全批第一次量到風三角形的橫向半邊。掃描條帶是東西向，180° 來風對它是純
橫風，所以有號風判準改讀南北向航段：北減南 +9.90 / +9.68 / +9.61 m/s（判準 +2W =
+10.0 ± 1.0 且為正），東減西 −0.02 / −0.02 / −0.01 ≈ 0；270° 的架次恰好相反。
東西向條帶的 gs − as 由無風的 +0.44 / +0.38 降到 −0.130 / −0.250，與純蟹行預測
√(Va²−W²) − Va 加密度偏置的 −0.149 / −0.290 差 0.019 / 0.040 m/s。
102/102 參數、15/15 風場（MIN_GROUNDSPEED 0）、83/53/77 項、3/3 落地、
實飛最高 119.6786 m（0 s 超過 120）、全速率最小間距 267.075 m、0 違規。
生成點以明示的 HV_SPAWN_TOL_M=6.0 通過，產物標記 tol_overridden=true。

HV4W55R：與 HV4W55 只差天花板（120.0 vs 119.5），風場相同、匯出項數相同、
三個 .param 逐位元相同 ⇒ 單一變數的高度對照。
匯出航點超過 120.000 m：88 / 216（HV4W55 為 0 / 216）；
實飛最高 120.1742 m（HV4W55 為 119.6752）；
實飛超過 120 m 的時間 354.93 模擬秒（HV4W55 為 0.00 s）；
代價是規劃完工時間 +0.0279 s（不是 +0.02 s，該值為四捨五入假影）。

分析與腳本在 runs/artifacts/PAPERPLAN1/sorties2/（ACCEPTANCE.md、
SPAWN_ANNOTATION.md、ceiling_control.json、spawn_record.json、scripts/）。
.BIN 未納管（每趟約 105 MB）。
MSG
```

兩段訊息都要依交接規則在結尾加上
`Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`。

⚠ lead 要自己決定 `scripts/hv_sitl_scenarios.py` 與 `scripts/hv_sitl_report.py`
（別人改的、與這批證據綁在一起）怎麼一起納管；我沒有碰它們，也沒有把它們放進
上面的 `git add`。

⚠ `.gitignore` 檔尾有兩行殘留垃圾（`追趕` 與 `; else echo REAL`），
不是我造成的，但納管前值得順手清掉。
