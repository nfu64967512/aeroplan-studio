# 納管準備（label `update_paper`）—— **我沒有 commit，也沒有 `git add`**

> 這份是 `sorties2/COMMIT_PLAN.md` 的**增補**，不是取代。
> `sorties2` 已經把「原始碼一支（`scripts/hv_sitl_run.py` 的 `HV_SPAWN_TOL_M` 覆寫）」與
> 「兩趟新架次的產物」整理好了；本檔只補上**本 label 新增的文字包與更正副本**，
> 並把整批的納管順序寫成一張可以照做的清單。

**機器狀態（本 label 結束時，我自己查的）**
- `HEAD` 仍是 `5e0295f5eb63e6597a787120d66526d3e4b17e7f`（分支 `quad_swarm`），**staged 0 筆**，stash 未動。
- 沒有 `ArduPlane.exe`／`MAVProxy` 程序在跑。**本 label 一趟 SITL 都沒飛。**
- `sitl/vtol/i10{0,1,2}/logs` 最高仍是 `00000020.BIN`（＝ `sorties2` 飛完 HV4W55R 之後的狀態）⇒ **沒有多飛任何架次**。
- **論文原始飛行證據完好**：我自己以 size + sha256 逐檔比對 `prep_runner/evidence_backup_manifest.json`，
  HV4SITL15 **39／39**、HV4SITL16 **27／27**，**0 不符、0 缺檔、0 多檔**（沒有執行他們的 `check_backup.py`，我自己寫了比對）。
- 沒有 `pip install`、沒有碰 COM 埠、沒有碰 WSL 側、沒有用過 `HV4SITL15`／`HV4SITL16` 作為 trial 名。

---

## 1. 本 label 新增的檔案

### 1a. `runs/artifacts/PAPERPLAN1/update_paper/`（主交付物）

**可納管檔：31 個、約 476 KB**（不含 `cache/*.npz`）。逐檔 size + sha256 在 `update_paper/INDEX.json`。

| 子項 | 內容 |
|---|---|
| `PAPER_TEXT_ZH.md`、`TABLES_ZH.md` | **定稿文字與表格**（取代 `paper_text/` 的同名兩檔） |
| `CORRECTIONS_K1_K9.md`、`README.md`、`COMMIT_PLAN_ADDENDUM.md` | 更正說明、可／不可主張清單、本檔 |
| `out/*.json`（6 檔） | 機器可讀數字：`NUMBERS`、`metrics`、`wind`、`corrections`、`spawn_verification`、`extract_index` |
| `corrected/`（11 檔） | 前面各 label 產物的更正副本與 `INDEX.md`／`INDEX.json` |
| `scripts/up_*.py`（8 支） | 重現以上全部的程式 |
| `INDEX.json` | 本 label 產物的 size + sha256 |

⚠ **`update_paper/cache/*.npz`（24 檔、約 9.3 MiB）不要納管。**
它們是 24 個 `.BIN` 的純衍生物，`scripts/up_extract.py` 可以重生；
既有 `.gitignore:127` 的 `*.npz` 規則**已經**排除它們（`git check-ignore` 實測命中）。

### 1b. 六張指路小卡（**新增檔案，沒有覆寫任何既有檔**）

`READ_ME_FIRST__corrections_in_update_paper.md`，放在：
`PAPERPLAN1/`、`PAPERPLAN1/paper_text/`、`PAPERPLAN1/paper_text/out/`、
`PAPERPLAN1/prep_text/`、`PAPERPLAN1/sorties/`、`PAPERPLAN1/sorties2/`。

它們的存在理由：有人直接打開 `paper_text/TABLES_ZH.md` 時，必須立刻知道那份已被更正。
若 lead 認為六張太多，**至少保留 `paper_text/` 與 `PAPERPLAN1/` 兩張**。

---

## 2. 整批的納管順序（建議三筆 commit）

### commit 1 —— 原始碼（`sorties2` 已備好訊息）

```
scripts/hv_sitl_run.py
```
只有 `HV_SPAWN_TOL_M` 覆寫那三段是 `sorties2` 這一輪加的（`sorties2/src/spawn_tol.diff`）。
⚠ `git diff scripts/hv_sitl_run.py` 顯示 **458 行變更**，其中大部分是先前 `c2_wind` 的 SITL 風場補丁與
`prep_runner` 的 `HV_PLAN_CEIL_M`；**訊息裡要講清楚哪些是哪一輪的**。

⚠ **另外兩支必須一起進**，否則新證據無法重現：
```
scripts/hv_sitl_scenarios.py   (+136；beamidp_1a2b_1195 等情境)
scripts/hv_sitl_report.py      (+93；產生四趟新證據的 separation_from_tracks.json / coverage_from_tracks.json)
```
⚠ `scripts/hv_sitl_scenarios.py:281` 把 `HV4SITL15` 寫死成 `gui_legal_120_1a2b` 的預設 trial 名；
納管前**請順手加一行註解**說明「此預設名已是論文證據，任何人重跑都必須設 `HV_TRIAL`」，
否則下一個人重跑會直接覆寫論文證據。（本 label 未改這支。）

### commit 2 —— 飛行證據（八趟）

```
runs/artifacts/{HV4SITL15,HV4SITL16,HV4W00,HV4W05,HV4W55,HV4W55R,HV4W55S,BEAMIDP1}/
```
⚠ `sitl/vtol/i10x/logs/*.BIN`（每趟約 27–59 MB，共八趟 24 個檔）**不要納管**。
⚠ 但這表示 `update_paper` 與 `analysis` 的重算**無法被第三方重現**（他們沒有 .BIN）。
建議在 commit 訊息或 `README` 中寫明：**可重現的是「由 .BIN 導出的 out/*.json」到「表格文字」這一段**；
.BIN → out/*.json 那一段只能由持有 log 的人重跑。這件事現在沒有寫在任何地方。

### commit 3 —— 分析與文字包

```
runs/artifacts/PAPERPLAN1/
docs/hv_dccpp/AASRC2026_abstract/
```
含本 label 全部產物與六張指路小卡。`analysis/cache/*.npz` 與 `update_paper/cache/*.npz` 依既有規則自動排除。

---

## 3. 納管前順手該清的兩件事（都不是本 label 造成的）

1. **`.gitignore` 最後兩行是垃圾**：`追趕` 與 `; else echo REAL`
   （明顯是 shell 意外寫進去的；`sorties2/COMMIT_PLAN.md` 也提過）。刪掉它們不影響任何規則。
2. **`PAD8_BROKEN_RAW`／`PAD8_FIXED_RAW`**（表 3 兩臂）仍被 `.gitignore:109` 排除。
   要嘛放行納管，要嘛表 3 必須標明「第三方無從核對」。**這是決策，不是清理。**

---

## 4. Commit 訊息草稿（**兩筆皆需在結尾附上 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`**）

### commit 3 的訊息

```
docs(aasrc2026): 定稿表格與 C1-C7 替換文字，含天花板單變因對照與側風架次

新增 label update_paper：自 24 個 dataflash .BIN 獨立重算全部論文數字，
產生定稿表格（TABLES_ZH.md）與逐條批註替換文字（PAPER_TEXT_ZH.md）。

主要變動
- C1 改以單變因對照為主證據：HV4W55（天花板 119.5）對 HV4W55R（120.0），
  同情境、同機隊、同 5 m/s 270 度規劃風與模擬風、三機參數檔逐位元相同。
  匯出超標航點 88 -> 0、實飛超過 120.000 m 的時間 354.93 s -> 0.00 s，
  代價為規劃完工時間 +0.0279 s。HV4SITL15/16 降為歷史無風對照。
- C2 新增側向（cross-track）證據：HV4W55S（5 m/s 180 度）量到北減南
  +9.51~+9.85 m/s、東減西歸零，側風地速虧損與理論風三角相符至
  0.006~0.041 m/s。同時寫明它與 270 度架次不是單變因對照。
- 套用九項數字更正（K1-K9）：間距量尺差改為 0.31-3.51 m / 0.34-0.90 m；
  參數嚴格比對補「到 float32 精度」（91/96，逐位元 85/96）；
  刪除 HV4W55S 的純順風解釋；「逐位元重現」限縮為生成位置；
  BEAMIDP1 腳註移除混淆的 makespan；天花板代價改為 +0.0279 s；
  附錄 B :276 改引 HV4SITL16；電池遙測理由改寫；表 Z 不再主張同一把尺。
- 前面各 label 的產物一律未覆寫：更正以 corrected/ 副本加索引處理，
  並在六個資料夾留下指路小卡。

證據：runs/artifacts/PAPERPLAN1/update_paper/{out/*.json, INDEX.json}
```

---

## 5. 給 lead 的四件「不要讀成乾淨」

1. **`update_paper` 的重算無法被沒有 .BIN 的人重現**（見 §2 commit 2 的警語）。這件事目前沒寫在任何 README 裡，
   本 label 已在自己的 `README.md` 寫了，但**整批的 commit 訊息也該寫**。
2. **HV4W55S 生成位移的量值仍未解釋**（方位與偏航已解釋並量到）。論文照實聲明，但這是一個開放問題，
   要解只能跑受控 SITL 探測（`SIM_GND_BEHAV` 掃描或 `LOG_DISARMED=1`），本輪規則不允許。
3. **HV4W55R 的實飛最小間距只有 100.579 m**（門檻 100，餘裕 0.579 m），最近接時刻垂直差 28.9 m（低於 50 m 規則）。
   這是 `gui_legal_120_1a2b` 這個情境本來就有的性質，天花板改動既沒造成也沒修好（+0.153 m）。
   全批最緊的是 HV4W05 的 **0.291 m**。
4. **`sorties2` 提到的 S3 驗收（實飛間距須在規劃值 3 m 內）在八趟中沒有一趟過**：
   落差 −4.20 ~ −5.27 m（270° 各趟）與 **−56.20 m**（HV4W55S）。
   綁安全的判準（≥ 100 m、0 違規）八趟全過。論文已把這件事寫成限制 (2) 與表 2b 表註 (f)，
   但**驗收清單本身需要修**：拿一個 2 s 網格上界去要求 3 m 以內，判準本身有問題。
