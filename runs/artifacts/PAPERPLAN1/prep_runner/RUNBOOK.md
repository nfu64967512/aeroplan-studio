# HV4W* 出動手冊（AASRC2026 審稿意見 C1 高度 + C2 風）

寫於 2026-09-21，label `prep_runner`。
**本手冊撰寫過程中沒有起過任何 SITL / ArduPilot / MAVProxy 行程。**
底下所有「規劃層」的數字都是本次真的跑出來的（`plan_matrix.json`），
所有「實飛」的期望值都標成期望值。

指令一律從 repo 根目錄執行：

```
C:\Users\joy46\Documents\Projects\aeroplan-studio
```

---

## 0. 這一批要回答什麼

| 審稿意見 | 決策 | 這批怎麼回答 |
|---|---|---|
| C1 高度 30/30 架次超過 120.000 m | D2：規劃天花板壓到 **119.5 m** 再飛，**不修** `reduce_waypoints()` 的圓弧內插缺陷 | 四趟全部在 119.5 天花板下飛，量到的最高點要留在 120.000 m 以下 |
| C2 「含風規劃被無風模擬驗證」 | D1：只跑 **0 與 5 m/s**；8 m/s 用既有 NO_GO 證據，不飛 | 四格：規劃風 × 模擬風 的 2×2 子集，外加一個 180° 橫風方向格 |

**已完成的前置工作（本 label）**

1. `runs/artifacts/HV4SITL15`（39 檔 2 420 789 B）與 `HV4SITL16`（27 檔 389 578 B）
   已逐檔 SHA-256 備份到 `evidence_backup/`，清單在 `evidence_backup_manifest.json`，
   兩邊**逐檔雜湊相同**。
2. `runs/artifacts/PAPERPLAN1/c2_wind/patch/c2_wind_sitl.patch` 已套上
   `scripts/hv_sitl_run.py`（`git apply` 乾淨、`py_compile` 通過）。
3. 新增高度旋鈕 `HV_PLAN_CEIL_M` / `HV_PLAN_ALT_M`（同一個檔，同樣風格）。
4. 「不給任何 `HV_*` 就與今天完全一樣」已驗證 —— 見 §9。

---

## 1. 出動矩陣

⚠ **trial 名稱一律用新的 `HV4W*`。絕對不可以用 `HV4SITL15` / `HV4SITL16`**：
航次產物是覆寫式的，而那兩個是論文證據（`scripts/hv_sitl_scenarios.py:281` 也把
`gui_legal_120_1a2b` 的預設 trial 寫死成 `HV4SITL15`，所以 **`HV_TRIAL` 一定要給**，
`fly` 與 `report` 兩個指令都要給）。

情境固定是 `gui_legal_120_1a2b`（1 Pioneer + 2 HERO，1560×800 m，起降點間距 10 m）。

| # | trial | 規劃風 | 模擬風 | 天花板 | 這一格回答什麼 |
|---|---|---|---|---|---|
| 1 | `HV4W00` | 0 | 0 @270 | 119.5 | 控制組：兩邊都沒風 |
| 2 | `HV4W05` | 0 | **5** @270 | 119.5 | **忽略風的代價**（審稿人的正題）。計畫與 #1 **逐位元相同**，唯一的差別是飛機真的遇到風 |
| 3 | `HV4W55` | 5 @270 | 5 @270 | 119.5 | 論文的主張，誠實驗證 |
| 4 | `HV4W55S` | 5 @**180** | 5 @**180** | 119.5 | 唯一測到風三角形**橫風分量**的一格（掃描條帶是東西向，270° 只測到沿航跡分量） |
| 5（選配） | `HV4W55R` | 5 @270 | 5 @270 | **120.0** | 高度的 A/B 對照：與 #3 只差天花板 |

**#1 與 #2 的計畫逐位元相同**，已驗證（`plan/*.waypoints`、`*.param`、`*.plan`
共 9 個檔的 SHA-256 全數相同）。這是這組設計最乾淨的一刀：兩趟飛同一份任務，
差別只有空氣。

**為什麼沒有 6 / 7 / 8 m/s**：D1。8 m/s 規劃器直接拒收 —— 陣風寫死是風速的 1.3 倍
（`hv_dccpp/integration/aeroplan_adapter.py:961`），站點陣風上限 8.0 m/s
⇒ W ≥ 6.16 m/s 就是 preflight NO_GO。證據已存在，不需要起飛：
`runs/artifacts/PAPERPLAN1/c2_wind/evidence/planner_wind_sweep.json`（W=8）與
`..._2_4_6_7.json`（W=7）。

**注意 #5 不是 HV4SITL15/16**：那兩趟的模擬風是 **0**（`SIM_WIND_*` 從來沒設過），
所以它們是「規劃 5 / 模擬 0 / 天花板 120」，不是 #3 的 120 m 版本。
想要乾淨的高度 A/B 就得飛 #5。

---

## 2. 規劃層已經量到的數字（本次實跑，`plan_matrix.json`）

全部 8 格都 `GO_WITH_WARNINGS`、`validate.passed=True`、`conflicts=0`、`uncovered=0`。

| cell | 天花板 | makespan s | 總航程 km | 最小水平間距 m | 距 100 m 門檻 | 匯出最高相對高度 m | 條帶 A1/B2/B3 |
|---|---|---|---|---|---|---|---|
| `HV4W00` / `HV4W05` | 119.5 | **1205.51** | 30.437 | **104.64** | **+4.64 ⚠** | 119.5521 | 3/2/4 |
| `HV4W55` | 119.5 | **1260.45** | 30.661 | 105.70 | +5.70 | 119.5526 | 3/2/4 |
| `HV4W55S` | 119.5 | **966.22** | 33.511 | 322.38 | +222.38 | 119.5526 | 3/2/4 |
| （參考）W00/W05 | 120.0 | 1205.48 | 30.434 | 105.06 | +5.06 | 120.0521 | 3/2/4 |
| `HV4W55R` | 120.0 | 1260.43 | 30.658 | 105.46 | +5.46 | 120.0526 | 3/2/4 |
| （參考）W55S | 120.0 | 966.18 | 33.509 | 322.22 | +222.22 | 120.0526 | 3/2/4 |

* `min_sep_v_m` 八格**全部是 Infinity** —— 代表規劃層沒有任何一對在水平上逼近到需要
  看垂直的程度。不要把它讀成「垂直間距無限大」。
* `RTL_RADIUS`（＝ `R_eff`，`hv_dccpp/ardupilot.py:933` `float(round(r_eff, 1))`）
  在**八格全部**是 A 200.0 m / B 120.0 m ⇒ 在 W = 5 m/s 下風修正**完全沒有作用**
  （W\* = 5.17 / 5.03 m/s），這正是 C6 的結論，本批次不會改變它。
* 每架匯出 **32** 個參數 ⇒ 參數閘門仍是 **34 × 3 = 102**（32 + `ARMING_REQUIRE`
  + `SCHED_LOOP_RATE`）。本批次沒有動這個數字。

### ⚠ 最可能翻車的一格：`HV4W00` / `HV4W05` 的間距

規劃層 **104.64 m**，離 100.0 m 的門檻只有 **4.64 m**（任務要求 < 5 m 就要標記）。
有意思的是方向相反：把天花板從 120 壓到 119.5，**W55 變好**（105.46 → 105.70）
而 **W00/W05 變差**（105.06 → 104.64）。刈幅隨天花板變 ⇒ 條帶間距變 ⇒ 排程重排，
這個量對天花板**不是單調的**。

實飛會更差多少？既有唯一的一組成對資料（`HV4SITL15`）：規劃 105.46 → 實飛 **104.2**
（`separation_from_tracks.json`，B2–B3 對，該時刻垂直只差 29.0 m，遠低於 50 m 的
`dh_sep`，所以水平一旦掉到 100 以下就是真的違規）。差 **−1.26 m**。
**[估計]** 照這個差套到 W00/W05 ⇒ 實飛約 **103.4 m**，仍在門檻之上，但只剩 3.4 m。
這是**單一架次的單一數字**，不是分布，不要當成保證。

> **訂正**：任務簡報說「實飛值是 101.0 m、只剩 1.0 m 裕度」。我掃了
> `runs/artifacts/HV4SITL*/separation_from_tracks.json` 全部 10 份，最小的實飛值是
> **104.2 m**（HV4SITL15），沒有任何一份是 101.0。101.42 m 出現在
> `c2_wind/evidence/planner_wind_sweep.json` 的 **W=2 m/s 規劃值**，
> 那是規劃層、不是實飛，而且 W=2 這一格 D1 已經不跑了。

---

## 3. 起飛前檢查

### 3.1 一次性（整批開始前做一次）

```powershell
cd C:\Users\joy46\Documents\Projects\aeroplan-studio

# (a) 機器上不可以有別人的 ArduPlane 在跑；有的話**不要**殺，等它跑完
Get-CimInstance Win32_Process -Filter "Name='ArduPlane.exe'" |
  ForEach-Object { "$($_.ProcessId)`t$($_.CommandLine)" }

# (b) 我們的埠段（6760 / 6770 / 6780）必須是空的
Get-NetTCPConnection -State Listen -LocalPort 6760,6770,6780 -ErrorAction SilentlyContinue

# (c) 論文證據的備份還在，而且雜湊相同
python runs\artifacts\PAPERPLAN1\prep_runner\scripts\check_backup.py

# (d) 磁碟：每趟約 105 MB 的 .BIN（三架 × 35 MB）
Get-PSDrive C | Select-Object Used,Free
```

檢查時的狀態（2026-09-21 01:3x）：沒有任何 `ArduPlane.exe`；6760/6770/6780 無監聽；
C 槽可用 **109 GB**；`sitl/vtol/i100..i103` 的 log 已累積 **1.45 GB**，
`LASTLOG.TXT` 都是 `14` ⇒ 下一趟寫 `00000015.BIN`。

### 3.2 每一趟開始前

1. `HV_TRIAL` 已設，而且是**沒用過的** `HV4W*`。
   `if (Test-Path runs\artifacts\$env:HV_TRIAL) { throw }` —— 存在就換名字，不要覆寫。
2. `HV_SPEEDUP` 留在 **5**（預設）。
3. `HV_SIM_WIND_TURB` 留在 **0**（預設）。湍流走 `rand()`
   （`SIM_Aircraft.cpp:901-912`），會直接打掉 SITL 的決定性，整組 A/B 作廢。
4. 上一趟的 `.BIN` 已經抽出來了（見 §5 的 (d)）。
5. ⚠ `git diff scripts/hv_sitl_scenarios.py` 看一眼 —— 這個檔在 2026-09-21 01:22
   被另一個並行的 agent 改過（見 §10.5）。情境表被動到就要重新跑 §2 的規劃矩陣：
   `python runs\artifacts\PAPERPLAN1\prep_runner\scripts\plan_matrix.py`（8 格約 100 s）。

---

## 4. 每一趟的指令

一次只跑一趟（每趟三個 `ArduPlane.exe`，約 3 核）。

### PowerShell

```powershell
cd C:\Users\joy46\Documents\Projects\aeroplan-studio
$env:PYTHONUTF8 = '1'

# ── 1. HV4W00  規劃 0 / 模擬 0 / 天花板 119.5 ─────────────────────────
$env:HV_TRIAL='HV4W00'
$env:HV_PLAN_WIND_SPD='0'; $env:HV_PLAN_WIND_DIR='270'
$env:HV_SIM_WIND_SPD='0';  $env:HV_SIM_WIND_DIR='270'
$env:HV_PLAN_CEIL_M='119.5'
python -u scripts\hv_sitl_run.py fly gui_legal_120_1a2b
python -u scripts\hv_sitl_report.py gui_legal_120_1a2b

# ── 2. HV4W05  規劃 0 / 模擬 5@270 / 天花板 119.5 ─────────────────────
$env:HV_TRIAL='HV4W05'
$env:HV_PLAN_WIND_SPD='0'; $env:HV_PLAN_WIND_DIR='270'
$env:HV_SIM_WIND_SPD='5';  $env:HV_SIM_WIND_DIR='270'
$env:HV_PLAN_CEIL_M='119.5'
python -u scripts\hv_sitl_run.py fly gui_legal_120_1a2b
python -u scripts\hv_sitl_report.py gui_legal_120_1a2b

# ── 3. HV4W55  規劃 5@270 / 模擬 5@270 / 天花板 119.5 ─────────────────
$env:HV_TRIAL='HV4W55'
$env:HV_PLAN_WIND_SPD='5'; $env:HV_PLAN_WIND_DIR='270'
$env:HV_SIM_WIND_SPD='5';  $env:HV_SIM_WIND_DIR='270'
$env:HV_PLAN_CEIL_M='119.5'
python -u scripts\hv_sitl_run.py fly gui_legal_120_1a2b
python -u scripts\hv_sitl_report.py gui_legal_120_1a2b

# ── 4. HV4W55S 規劃 5@180 / 模擬 5@180 / 天花板 119.5 ─────────────────
$env:HV_TRIAL='HV4W55S'
$env:HV_PLAN_WIND_SPD='5'; $env:HV_PLAN_WIND_DIR='180'
$env:HV_SIM_WIND_SPD='5';  $env:HV_SIM_WIND_DIR='180'
$env:HV_PLAN_CEIL_M='119.5'
python -u scripts\hv_sitl_run.py fly gui_legal_120_1a2b
python -u scripts\hv_sitl_report.py gui_legal_120_1a2b

# ── 5.（選配）HV4W55R 與 #3 只差天花板 120.0 ──────────────────────────
$env:HV_TRIAL='HV4W55R'
$env:HV_PLAN_WIND_SPD='5'; $env:HV_PLAN_WIND_DIR='270'
$env:HV_SIM_WIND_SPD='5';  $env:HV_SIM_WIND_DIR='270'
Remove-Item Env:\HV_PLAN_CEIL_M -ErrorAction SilentlyContinue   # ★ 不給 ⇒ 情境原值 120
python -u scripts\hv_sitl_run.py fly gui_legal_120_1a2b
python -u scripts\hv_sitl_report.py gui_legal_120_1a2b

# ── 收尾：清掉遺留的環境變數，免得污染下一批 ──────────────────────
Get-ChildItem Env: | Where-Object Name -like 'HV_*' | ForEach-Object { Remove-Item "Env:\$($_.Name)" }
```

⚠ PowerShell 的環境變數會**留在 session 裡**。`HV4W55R` 那一格一定要用
`Remove-Item Env:\HV_PLAN_CEIL_M`；只是不寫 `$env:HV_PLAN_CEIL_M='...'` 沒有用，
上一格的值還在，會靜靜地用 119.5 飛完整趟。

### bash（Git Bash / WSL 的 Windows python 皆可）

```bash
cd /c/Users/joy46/Documents/Projects/aeroplan-studio

run () {   # run <TRIAL> <planW> <planDir> <simW> <simDir> [ceil]
  env PYTHONUTF8=1 HV_TRIAL="$1" \
      HV_PLAN_WIND_SPD="$2" HV_PLAN_WIND_DIR="$3" \
      HV_SIM_WIND_SPD="$4"  HV_SIM_WIND_DIR="$5" \
      ${6:+HV_PLAN_CEIL_M=$6} \
      python -u scripts/hv_sitl_run.py fly gui_legal_120_1a2b
  env PYTHONUTF8=1 HV_TRIAL="$1" python -u scripts/hv_sitl_report.py gui_legal_120_1a2b
}

run HV4W00   0 270 0 270 119.5
run HV4W05   0 270 5 270 119.5
run HV4W55   5 270 5 270 119.5
run HV4W55S  5 180 5 180 119.5
run HV4W55R  5 270 5 270            # 選配：不給第 6 個參數 ⇒ 天花板 120.0
```

`env VAR=v cmd` 的寫法讓變數只在那一行生效，不會像 PowerShell 那樣殘留。

### 旋鈕一覽

| 環境變數 | 預設 | 作用 |
|---|---|---|
| `HV_TRIAL` | 情境的 `trial`（本情境 = `HV4SITL15` ⚠） | 產物目錄 `runs/artifacts/<HV_TRIAL>`。`fly` 與 `report` 都要給 |
| `HV_PLAN_WIND_SPD` / `_DIR` | 不給 ⇒ 情境值 (5, 270) | 規劃器**以為**會遇到的風 |
| `HV_SIM_WIND_SPD` / `_DIR` | 0 / 270 | 飛機**真的**遇到的風（寫進 `iN/wind.parm`，排在 `--defaults` 最後） |
| `HV_SIM_WIND_TURB` / `_T` | 0 / 1 | 湍流（保持 0）／風剖面型態 |
| `HV_PLAN_CEIL_M` | 不給 ⇒ 情境原值 | 空域 AGL 上限 `h_ceil`（走 `waiver_max_agl_m` 那一支） |
| `HV_PLAN_ALT_M` | 不給 ⇒ `min(情境航高, 新天花板)` | 目標航高 `coverage_altitude_m` |
| `HV_SPEEDUP` | 5 | 保持 5 |
| `HV_POLL_S` / `HV_TELEM_HZ` | 0.05 / 10 | 取樣率，保持預設 |

---

## 5. 每一趟結束後立刻做的事

⚠ 航次資料是**覆寫式**的；`.BIN` 更糟 —— 三個 instance 目錄已經 1.45 GB。

```powershell
# (a) 報表（上面每一格的第二行已經做了）
$env:HV_TRIAL='HV4W55'; python -u scripts\hv_sitl_report.py gui_legal_120_1a2b

# (b) 風真的有作用嗎（有號的東西向差；同一支腳本 C2 已經量過無風基線）
python runs\artifacts\PAPERPLAN1\c2_wind\wind_probe_from_flightcsv.py HV4W00 HV4W05 HV4W55 HV4W55S

# (c) 規劃 vs 實飛（不會動到各趟自己的產物）
python runs\artifacts\PAPERPLAN1\c2_wind\replay_planned_vs_flown.py HV4W00 HV4W05 HV4W55 HV4W55S
python scripts\hv_sitl_compare.py HV4W00 HV4W05 HV4W55 HV4W55S

# (d) 全速率高度真值（1 Hz 的 flight.csv 會漏掉峰值）——**這一趟跑完馬上抽**
#     LASTLOG.TXT 現在是 14，所以第一趟是 00000015.BIN，之後遞增
foreach ($i in 100,101,102) {
  $n = (Get-Content "sitl\vtol\i$i\logs\LASTLOG.TXT").Trim().PadLeft(8,'0')
  python -m pymavlink.tools.mavlogdump --types GPS,WIND,ARSP --format csv `
    "sitl\vtol\i$i\logs\$n.BIN" > "runs\artifacts\$env:HV_TRIAL\i$i`_$n.csv"
}
```

(d) 用的是 `runs/artifacts/PAPERPLAN1/c1_altitude/binlog_alt2.py` 同一條路子；
那支腳本本身也可以直接跑，它會把 `HV4SITL*` 的 dataflash 高度抽出來。

---

## 6. 逐趟驗收判準

### 每一趟都必須成立（不成立 ⇒ 該格作廢，不要硬收）

| 代號 | 判準 | 看哪裡 |
|---|---|---|
| G1 | 參數讀回 **102/102** 相符（34 參數 × 3 架） | `params_readback.csv`；腳本沒過會自己 `SystemExit` |
| G2 | 風場讀回 **15/15** 相符（`SIM_WIND_SPD/DIR/TURB/T` + `MIN_GROUNDSPEED` × 3 架） | `wind_readback.json`（新增） |
| G3 | 生成點誤差 ≤ 2.0 m × 3 架 | `spawn_readback.json` |
| G4 | 任務上傳 = 讀回項數。本次規劃量到：`HV4W00`/`HV4W05`/`HV4W55`/`HV4W55R` 都是 **A1 82 / B2 53 / B3 84**；`HV4W55S` 是 **A1 83 / B2 53 / B3 77**（180° 風讓配置改變） | `mission_upload.json` |
| G5 | 三架都落地，`mode_change_rejected` 與 `link_lost` 皆空 | `release_schedule.json`、腳本回傳 0 |
| G6 | `flight.csv` 的**實際中位取樣間隔** ≤ 1.0 s 模擬時間 | 自己量；參數讀回 10 Hz **不等於** 真的 10 Hz |

### C1（高度）——四趟都要看

| 代號 | 判準 | 期望值 |
|---|---|---|
| H1 | 匯出的 `.waypoints` 最高相對高度 | **119.5521 m**（A1）／119.5329（B2）／119.5328（B3），已在規劃層量到 |
| H2 | **實飛最高 AGL < 120.000 m** —— 這一批的正題 | **[估計]** 119.60–119.67 m。推導：C1 量到的追蹤超出量是 +0.050 / +0.096 / +0.129 m（逐架，dataflash 全速率），加在 H1 上 ⇒ 最壞 119.5328 + 0.129 = **119.662 m**，距 120.000 還有 **0.338 m** |
| H3 | 以 dataflash（5.6–6.3 Hz）為準，不是 1 Hz 的 `flight.csv` | §5 (d) |

⚠ H2 的裕度只有 **0.34 m**，而那個 +0.129 m 是**既有樣本的最大值**，不是上界。
若任何一架量到 > 120.000 m，見 §8 的 F-H2。

### C2（風）——逐格

**基線（C2 已量，無風的三趟）**：`median(groundspeed − airspeed)` = **+0.44 m/s**
（Pioneer，Va 21.5）／**+0.38 m/s**（HERO，Va 19.0）。與航向**無關**、與空速成比例
（2.05 % / 2.00 %）⇒ 是空速/密度的常數偏置，**不是風**。東減西的不對稱是
0.00–0.02 m/s（10 架次）⇒ 沿東西向的風 ≤ 0.01 m/s。

| trial | 判準 |
|---|---|
| `HV4W00` | 東減西 `(gs − as)` = **0.00 ± 0.5 m/s**（複製無風基線）；`wind_spd_est` ≈ 0 |
| `HV4W05` | **東減西 = +10.0 ± 1.0 m/s**，且**東 > 西**（東約 +5.4、西約 −4.6，密度偏置 +0.4 疊在兩邊）。`wind_dir_from_est` → 270 ± 20°。**飛的是與 `HV4W00` 逐位元相同的任務** ⇒ makespan / 航程 / 間距的差額**全部**歸因於風 |
| `HV4W55` | 同 `HV4W05` 的風判準；另外 makespan 與規劃的 1260.45 s 的誤差應**小於** `HV4W05` 對 1205.51 s 的誤差 —— 這就是「含風規劃有用」的量 |
| `HV4W55S` | 東西向差值**回到 0 ± 1 m/s**（180° 對東西向條帶是純橫風），差值改出現在南北向的轉場腿上；巡航時應看得到蟹行角。`wind_dir_from_est` → 180 ± 20° |

**方向性是判準的一部分，不是只看大小。**
完整推導（含 ArduPilot 原始碼的行號）在 `WIND_CONVENTION.md`：
`SIM_WIND_DIR` 是氣象**來向**（`libraries/SITL/SITL.cpp:89-94`
「Direction simulated wind is coming from」），`SIM_Aircraft.cpp:891-916` 把它轉成
NED 風向量再取負，`:763` 的 `velocity_air_ef = velocity_ef − wind_ef` ⇒
**270° ⇒ 向東的航段地速比向西高**。
若量到 **−10 m/s**（東比西慢）：風有作用但方向相反 ⇒ **不要**直接改 `SIM_WIND_DIR`，
先回去看 `WIND_CONVENTION.md` 第 2 節哪一步錯了（規劃層的 `dir_from_deg` 用同一個
約定，會一起錯）。若量到 **≈0**：風沒生效，查 G2。

### 間距（全部四趟）

| 代號 | 判準 |
|---|---|
| S1 | `separation_from_tracks.json` 的 `n_violations` = **0**（違規＝水平 < 100 **且** 垂直 < 50） |
| S2 | `min_h_m` ≥ 100.0 m。**`HV4W00` / `HV4W05` 規劃只有 104.64 m**，是最可能翻車的一格 |
| S3 | 實飛 `min_h_m` 與 `validation.json` 的 `min_sep_h_m` 的差 ≤ 3 m（HV4SITL15 量到 −1.26 m） |

---

## 7. 預期牆鐘與磁碟

`fly` 一趟 = 規劃+匯出（**實測 12–13 s**）＋ SITL 開機與四道閘門（約 20–40 s，
生成點收斂要等連續兩筆相隔 ≥1.5 s 且相距 ≤0.5 m）＋ 飛行 `makespan / 5` ＋ 收拾。
腳本的硬上限是 `makespan/5 + 420 s`。

| trial | 規劃 makespan | 飛行牆鐘 | 一趟總計 |
|---|---|---|---|
| `HV4W00` | 1205.5 s | 241 s | 約 **5 分** |
| `HV4W05` | 1205.5 s | 241 s | 約 **5 分** |
| `HV4W55` | 1260.5 s | 252 s | 約 **5 分** |
| `HV4W55S` | 966.2 s | 193 s | 約 **4 分** |
| `HV4W55R`（選配） | 1260.4 s | 252 s | 約 **5 分** |

四趟 ≈ **20 分**飛行 + 報表約 1 分/趟 ⇒ **抓 35 分**（含檢查與 `.BIN` 抽取）。
五趟（含選配的 `HV4W55R`）⇒ 抓 45 分。
再加 D3 的 `BEAMIDP1`（§10，makespan 2348.0 s ⇒ 飛行 7.8 分、一趟約 9 分）⇒ 抓 60 分。

磁碟：每趟 `.BIN` 約 **105 MB**（3 × 35 MB）＋ 產物約 1 MB。四趟 ≈ 420 MB。
C 槽目前可用 109 GB，不是問題；但 `sitl/vtol/i100..i103` 已經 **1.45 GB**，
抽完 §5 (d) 之後可以考慮清掉舊的 `.BIN`（**不要**清 `runs/artifacts/` 下的東西）。

---

## 8. 閘門沒過怎麼辦

| 代號 | 症狀 | 處置 |
|---|---|---|
| **F-G1** | `參數沒有生效，停止起飛` | 看 `params_readback.csv` 的 `actual`。**讀回 `None` ＝ 韌體沒有這個參數名**（改名或打錯），不是通訊問題。比對 `sitl/default_params/firmware_param_names.json`。**不要**把該參數從閘門裡拿掉 —— 那正是 2026-09-14 修掉的缺陷 |
| **F-G2** | `風場參數沒有生效，停止起飛` | 看 `wind_readback.json`。(a) `SIM_*` 讀回 `None` ⇒ 這不是 SITL build，換 `sitl/ArduPlane.exe`。(b) 值不對 ⇒ 確認 `iN/wind.parm` 真的在 `--defaults` 的**最後**（`sitl_cmdlines.txt` 最後一段應該是 `...\iN\wind.parm`）。(c) **`MIN_GROUNDSPEED` ≠ 0 ⇒ 整批作廢，不要繞過**：TECS 會為了維持地速而在逆風腿自己加空速，風的影響被飛控吃掉，而且**沒有任何徵狀** |
| **F-G3** | 生成點誤差 > 2 m | 規劃分了站、SITL 沒分。**不要**改成用 `HOME`。檢查 `--home` 是否逐機不同（`sitl_cmdlines.txt`） |
| **F-G5** | 切 AUTO 三次都被拒 / 中途斷線 | 先看是不是埠撞到別人（§3.1 (a)(b)）。`HV4SITL6` 就是 speedup 10 造成的；**不要**動 `HV_SPEEDUP` 去「修」它 |
| **F-H2** | 實飛最高 > 120.000 m | 把天花板再壓一階：`HV_PLAN_CEIL_M=119.0`，重飛**該格**並換 trial 名（`HV4W55_C1190`）。代價已量過：匯出最高 119.0526 m、makespan 1260.48 s（+0.03 s）、航程 +1 m、間距 105.95 m（反而更好），見 `c1_altitude/ablation_cap.json`。**不要**在送審前去修 `reduce_waypoints()` 的內插 —— 那會動到每一份既有產物 |
| **F-S2** | 實飛 `min_h_m` < 100 m | 這是**真的違規**，要報告不要藏。先確認不是取樣造成的（`step_s=2.0` 的線性內插使水平距離是**上界**）。可行的對策只有兩條，都要記錄：(i) 提高 `spawn_spread_m`（現在 10 m）；(ii) 天花板改 119.0（W00 在 120 是 105.06、在 119.5 是 104.64，**不單調**，必須重新規劃量過才知道 119.0 是多少，不要外推） |
| **F-C2 方向反了** | 東減西 = −10 m/s | 見 §6 與 `WIND_CONVENTION.md`。**不要**直接改 `SIM_WIND_DIR` 把符號湊對 |
| **F-C2 沒有風** | 東減西 ≈ 0 但 G2 通過 | 查 `SIM_WIND_T`（本批用 1）與 `SIM_WIND_TC`（風的時間常數，預設 5 s；speedup 5 ⇒ 模擬時間 25 s 才到穩態，起飛後前 30 s 本來就不該拿來算） |
| **趟數不足時的取捨** | 只有時間飛兩趟 | 飛 `HV4W00` 與 `HV4W05`。那一對是計畫**逐位元相同**、只有空氣不同的乾淨對照，單獨就能回答 C2 的正題 |

---

## 9. 「不給任何 `HV_*` 就與今天一樣」的驗證

`python runs/artifacts/PAPERPLAN1/prep_runner/scripts/verify_patch.py`
（已跑，結果在 `verify_patch.json`，**沒有起任何行程**：`_instance_dir` 被導到
`prep_runner/verify/`、`subprocess.Popen` 被換成假的，飛機/計畫用的是
**HV4SITL15 存檔**裡的真值。）

`VERDICT_no_env_behaviour_unchanged: true`，逐項：

* **`--defaults` 鏈**：舊版產出的鏈與 **HV4SITL15 存檔的 `sitl_cmdlines.txt` 逐項相同**；
  新版只在**最後**多一個 `wind.parm`，前 6 項的內容與順序完全不變，`--home` 也不變。
* **`SIM_WIND_*` 的實效值**：今天整條鏈上**一個檔都沒有設** `SIM_WIND_*`
  ⇒ 吃韌體預設（`SPD 0 / DIR 180 / T 0 / TURB 0`）。新版不給環境變數時寫的是
  `SPD 0 / DIR 270 / TURB 0 / T 1 / DIR_Z 0`。`DIR` 與 `T` 兩個值與今天不同，
  但 **`SPD = 0` ⇒ 風向量恆等於零向量**（`SIM_Aircraft.cpp:891`，整個向量乘上 speed），
  模擬上**完全等價**。這是唯一一處「檔案值變了但行為沒變」，寫在這裡免得日後有人誤讀。
* **參數閘門**：新舊 `expected_params()` 的輸出**逐鍵相同**，每架 **34** 個、
  合計 **102** 列，匯出檔 32 個全部涵蓋。C4 的數字不動。
* **`flight.csv`**：多兩欄 `wind_spd_est` / `wind_dir_from_est`（飛控自己的 EKF 風估計），
  前 9 欄不變。
* **環境旋鈕**：舊 7 個 → 新 15 個，新增的 8 個全部是 `HV_SIM_WIND_*`（4）、
  `HV_PLAN_WIND_*`（2）、`HV_PLAN_CEIL_M`、`HV_PLAN_ALT_M`。
* **整條規劃管線的逐位元回歸**（最強的一道）：不給任何 `HV_*`（只給 `HV_TRIAL`
  指到暫存目錄）重跑 `plan gui_legal_120_1a2b`，產出的 9 個計畫檔
  （`A1/B2/B3` × `.waypoints`/`.param`/`.plan`）與**存檔的 HV4SITL15 逐位元相同**
  （SHA-256 全數相符）。`scenario.json` 讀回 `altitude_m 120.0`、`h_ceil_m 120.0`、
  `waiver_max_agl_m null`、`wind [5.0, 270.0]`、`sim_wind.spd_mps 0.0`。
  產物在 `plan_matrix/NOENV/`，紀錄在 `verify_patch.json` 的
  `no_env_replan_vs_archived_HV4SITL15`。

---

## 10. D3（beam-IDP 交錯）的那一趟

### 10.1 ⚠ 這四趟**不能**當 D3 的證據

`gui_legal_120_1a2b` 的計畫裡**沒有交錯**。這是量到的，不是猜的：
`split_area_for_fleet` 把 1560×800 m 切成 3 個子區（三台 ⇒ 三塊），每塊再按
1A:2B 的比例分給兩個構型 ⇒ 每個胞只有 **n = 1（A）或 n = 2（B）** 條帶，而
`skip_k(96.99, 200) = 5`、`skip_k(96.99, 120) = 3`。
`_skip_sequence(n, k)` 在 `k ≥ n` 時**等於 `range(n)`，也就是順序掃描**
（`hv_dccpp/decompose.py:689-693`）⇒ 六個胞**沒有一個**觸發。
逐胞普查在 `runs/artifacts/PAPERPLAN1/c3_beamidp/plan_strips_gui_legal_120_1a2b.json`。

### 10.2 要飛的是 `beamidp_1a2b_1195`（trial `BEAMIDP1`）

⚠ 這個情境**不是我加的**：2026-09-21 01:22 另一個並行的 agent 把它加進
`scripts/hv_sitl_scenarios.py`（三塊 1600×1200 m = 5.76 km²，天花板 119.5 已經寫死在
情境裡）。我在同一天稍後用**真正的 runner** 跑過它的 `plan`，數字如下（本 repo 實跑）：

| 量 | 值 |
|---|---|
| status / validate | `GO_WITH_WARNINGS` / `passed=True`、conflicts 0、uncovered 0 |
| makespan | **2348.0 s** ⇒ speedup 5 下飛行牆鐘 **7.8 分**，一趟總計約 **9 分** |
| 最小水平間距 | **180.6 m**（裕度 +80.6 m，安全） |
| 條帶 | A1 13 / B2 13 / B3 13 |
| 航程 | A1 41.74 / B2 37.17 / B3 33.81 km |
| 能量餘裕 | A1 441 / B2 **89** / B3 **100** Wh |
| 匯出項數（G4 用） | A1 **263** / B2 **165** / B3 **175** |
| `RTL_RADIUS`（= R_eff） | A 200.0 / B 120.0（與其他格相同） |
| 匯出最高相對高度 | A1 **119.6703** / B2 119.5136 / B3 119.5540 m |

指令（**不要**設 `HV_PLAN_CEIL_M`，天花板已經在情境裡）：

```powershell
cd C:\Users\joy46\Documents\Projects\aeroplan-studio
$env:PYTHONUTF8='1'
Remove-Item Env:\HV_PLAN_CEIL_M -ErrorAction SilentlyContinue   # ★ 先清掉，否則沿用上一格的值
Remove-Item Env:\HV_PLAN_ALT_M  -ErrorAction SilentlyContinue
$env:HV_TRIAL='BEAMIDP1'
$env:HV_PLAN_WIND_SPD='5'; $env:HV_PLAN_WIND_DIR='270'
$env:HV_SIM_WIND_SPD='5';  $env:HV_SIM_WIND_DIR='270'
python -u scripts\hv_sitl_run.py plan beamidp_1a2b_1195     # 先 plan，12 s
python -u scripts\hv_sitl_run.py fly  beamidp_1a2b_1195     # 約 9 分
python -u scripts\hv_sitl_report.py   beamidp_1a2b_1195
```

```bash
env PYTHONUTF8=1 HV_TRIAL=BEAMIDP1 \
    HV_PLAN_WIND_SPD=5 HV_PLAN_WIND_DIR=270 \
    HV_SIM_WIND_SPD=5  HV_SIM_WIND_DIR=270 \
    python -u scripts/hv_sitl_run.py fly beamidp_1a2b_1195
env PYTHONUTF8=1 HV_TRIAL=BEAMIDP1 python -u scripts/hv_sitl_report.py beamidp_1a2b_1195
```

### 10.3 ⚠⚠ 這一趟特有的兩個風險

1. **高度裕度只剩 0.20 m。** A1 匯出的最高點是 **119.6703 m**，比天花板高
   **+0.17 m** —— 是論文那個胞（+0.052 m）的**三倍多**。圓弧內插的誤差隨
   「頂點離本地 ENU 原點的距離」增長（等 AGL 面是拋物面），而這個情境的作業區
   跨度 ±2900 m，比 1560×800 遠得多。
   **[估計]** 加上 C1 量到的追蹤超出量上限 +0.129 m ⇒ 實飛約 **119.80 m**，
   距 120.000 只剩 **0.20 m**，而那個 +0.129 是既有樣本的最大值、不是上界。
   ⇒ 這一趟的 H2 判準**比其他四趟脆弱**。若量到 > 120.000：情境把
   `waiver_max_agl_m` 寫死成 119.5，所以用 `HV_PLAN_ALT_M=119.0` 壓航高（不是壓
   天花板）重飛，並換 trial 名。
2. **兩架 HERO 的能量餘裕只剩 89 / 100 Wh**，是目前所有情境裡最接近能量邊界的一組。
   起飛前確認 `report.json` 的 `energy_ok`，落地後比對實際用量與 `energy_used_wh`。

### 10.4 我自己那份 stanza（已被取代，保留當備案）

`patch/add_beamidp_scenario.SUPERSEDED.patch` 是我在看到上面那個情境之前寫的
「單一大區 3000×3000 m = 9.00 km²」版本（`gui_legal_120_1a2b_9km2`，trial `HV4WIDP`）。
`git apply --check` 通過、語法與註冊都驗過，但 **5.76 km² 的三塊版更省**
（makespan 2348.0 s vs 3013.9 s），所以**不要套**，除非需要「單一大區自動切」這條
路徑本身的證據。它量到的數字仍然有用，因為裡面有一個反例：

> 同一份 3000×3000 幾何，天花板 **120.0 → 119.5** 讓 makespan 掉了 **−21.6 %**
> （3842.6 → 3013.9 s），最小水平間距從 190.6 掉到 136.5 m。
> 也就是說 C1 那句「壓天花板幾乎不花錢（makespan +0.02 s）」**只對論文那個
> 1560×800 的胞成立**，不是通則 —— 刈幅隨天花板變、條帶佈局跟著變、配置與排程
> 就整個重排。任何別的幾何都要**重新量**，不可以外推。
> 資料：`beamidp_sortie_c1195.json` / `beamidp_sortie_c120.json`。

### 10.5 ⚠ 並行編輯的紀錄

`scripts/hv_sitl_scenarios.py` 在 2026-09-21 01:22（我跑規劃矩陣的期間）被另一個
agent 改過。我確認過那是**純新增**：只多了 `beamidp_1a2b_1195` 一個鍵，
`gui_legal_120_1a2b` 的每一個欄位都沒變（`coverage_altitude_m=120.0`、
`waiver_max_agl_m=None`、`wind=(5.0, 270.0)`、`spawn_spread_m=10.0`…）
⇒ §2 那張表的八格數字**不受影響**。
但這件事本身要記著：**同一批人同時在改同一個檔**，下一次跑之前先
`git diff scripts/hv_sitl_scenarios.py` 看一眼。

## 11. 產物清單（每一趟應該長這樣）

```
runs/artifacts/HV4W55/
  scenario.json            ← 同時有 "wind"（規劃）與 "sim_wind"（模擬）、
                              "h_ceil_m": 119.5、"waiver_max_agl_m": 119.5、"altitude_m": 119.5
  wind_readback.json       ← 新增：SIM_WIND_* + MIN_GROUNDSPEED × 3
  params_readback.csv      ← 不變：34 × 3 = 102 列
  spawn_readback.json  mission_upload.json  release_schedule.json
  validation.json  report.json  plan/mission_summary.json
  sitl_cmdlines.txt        ← 最後一段是 ...\iN\wind.parm
  A1/flight.csv  B2/flight.csv  B3/flight.csv   ← 11 欄（多 wind_spd_est / wind_dir_from_est）
  plan/{A1,B2,B3}.{param,plan,waypoints}  plan/*_fence.txt  plan/*_rally.txt
  compare.png  flight_summary.json  separation_from_tracks.json
  coverage_from_tracks.json  git_state.txt  + 四份驅動腳本的副本
外加（不在 trial 目錄裡）：
  sitl/vtol/i100..i102/wind.parm
  sitl/vtol/i100..i102/logs/00000015.BIN 起，每架約 35 MB
```

---

## 附：本 label 產出的檔案

| 檔 | 內容 |
|---|---|
| `RUNBOOK.md` | 本檔 |
| `WIND_CONVENTION.md` | `SIM_WIND_DIR` 方向約定的四段證據鏈 + 有號驗收判準 |
| `plan_matrix.json` | 八格的規劃層量測（逐機條帶／航程／R_eff／匯出最高高度／參數數） |
| `verify_patch.json` | 「不給環境變數就與今天一樣」的逐項驗證 |
| `beamidp_sortie_c1195.json` / `_c120.json` | D3 幾何的規劃層量測 |
| `evidence_backup/` + `evidence_backup_manifest.json` | HV4SITL15/16 的逐檔 SHA-256 備份 |
| `src/hv_sitl_run.py.before` / `.after` / `before_vs_after.diff` | 本 label 對 `scripts/hv_sitl_run.py` 的全部改動 |
| `patch/add_beamidp_scenario.SUPERSEDED.patch` | 我寫的 D3 情境 stanza（單一大區 9 km²）。**不要套** —— 已被別的 agent 加進去的 `beamidp_1a2b_1195`（5.76 km²）取代，見 §10.4 |
| `scripts/` | 上面每一個數字的產生器（全部純 Python、不起 SITL） |
| `plan_matrix/` | 八格各自的完整規劃產物，外加 `BEAMIDP1_probe/`（D3 情境的規劃產物） |
| `logs/`、`logs_*.log` | 上面每一次執行的完整 stdout/stderr |
