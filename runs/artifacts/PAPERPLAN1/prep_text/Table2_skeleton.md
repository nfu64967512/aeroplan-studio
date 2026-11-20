# 修訂後表 2 骨架 — 每一格都標明數字從哪裡來
目的：飛完之後**不需要任何猜測或臨場判斷**，照著「來源」欄抄即可。
`⟪TRIAL⟫` = 主結果航次標籤（建議 `HV4C1W55`：119.5 m 天花板＋規劃風 5/270＋模擬風 5/270）。

## 主結果航次的啟動指令（旋鈕已在工作樹，我沒有執行）

```
# PowerShell；⚠ HV_TRIAL 在 fly 與 report 都要設，否則會蓋掉 HV4SITL15
$env:HV_TRIAL="HV4C1W55"; $env:HV_PLAN_CEIL_M="119.5"
$env:HV_PLAN_WIND_SPD="5"; $env:HV_PLAN_WIND_DIR="270"
$env:HV_SIM_WIND_SPD="5";  $env:HV_SIM_WIND_DIR="270"; $env:HV_SIM_WIND_TURB="0"
python scripts\hv_sitl_run.py plan gui_legal_120_1a2b     # 先看規劃數字，不起 SITL
python scripts\hv_sitl_run.py fly  gui_legal_120_1a2b
python scripts\hv_sitl_report.py gui_legal_120_1a2b
```
旋鈕出處：`scripts/hv_sitl_run.py:62-65`（模擬風）、`:785-789`（規劃風）、`:805-829`（天花板）、`:512-523`（風讀回閘）。

---

## 表 2 規劃與 SITL 飛行結果對照（1 Pioneer + 2 HERO）

| # | 項目 | 規劃 | SITL 飛行 | 兩欄數字的來源（飛完照抄這一欄） |
|---|---|---|---|---|
| 1 | 起降完成 | 3 架 | ⟪3／3⟫ | **待飛**。`runs/artifacts/⟪TRIAL⟫/report.json`；規劃欄固定為機隊數 3 |
| 2 | 母區覆蓋率（1.248 km²） | 未覆蓋 0 | ⟪100.0 %（上界）⟫ | 規劃欄：**已算出**＝0（`c1_altitude/ablation_cap.json` 的 `119.5.uncovered`），飛後以 `⟪TRIAL⟫/plan/mission_summary.json` 覆核。飛行欄：**待飛**，`⟪TRIAL⟫/coverage_from_tracks.json`（`hv_sitl_report.py`）。⚠ 刈幅緩衝由 138.56 m 改為 **137.99 m**，分母仍 1,248,034.8 m² |
| 3 | 全程最高高度 AGL（作業上限 120 m；規劃天花板 119.5 m） | **119.55 m**（＝匯出後之最大值） | ⟪119.5x／119.5x／119.5x m⟫ | 規劃欄：**已算出**＝A1 119.5526／B2 119.5332／B3 119.5331 m（`ablation_cap.json` 的 `119.5.uavs.*.max_export_rel_m`），取最大寫 119.55。飛行欄：**待飛**，`⟪TRIAL⟫/{A1,B2,B3}/flight.csv`（1 Hz）；若要 5–6 Hz 真值用 `sitl/vtol/i10x/logs/*.BIN`（做法見 `c1_altitude/binlog_alt2.py`） |
| 4 | 航程 A1／B2／B3（km） | **10.29／7.93／12.44** | ⟪x.xx／x.xx／x.xx（+x.x %）⟫ | 規劃欄：**已算出**＝10.2936／7.9265／12.4406 km（`ablation_cap.json` 的 `119.5.uavs.*.dist_km`；與 120 m 版差 ≤ 1 m），飛後以 `⟪TRIAL⟫/plan/mission_summary.json` 覆核。飛行欄：**待飛**，`⟪TRIAL⟫/{ac}/flight.csv` 積分（公式同 `scripts/hv_sitl_report.py`） |
| 5 | 完工時間 makespan（s） | **1260.5** | ⟪xxxx.x⟫ | 規劃欄：**已算出**＝1260.45 s（`ablation_cap.json` 的 `119.5.makespan_s`）。飛行欄：**待飛**，`⟪TRIAL⟫/report.json`／`release_schedule.json`；⚠ 沿用「觸地且上鎖」判準，並在表註保留 <2 m 判準之值 |
| 6 | 最小水平間距（m；門檻 100） | **105.7** | ⟪xxx.x（垂直 xx.x；違規 0/xxx）⟫ | 規劃欄：**已算出**＝105.699 m（`ablation_cap.json` 的 `119.5.min_sep_h_m`，**略優於 120 m 版的 105.464**）。飛行欄：**待飛**，`⟪TRIAL⟫/separation_from_tracks.json`（2 s 網格內插上界） |
| 7 | 飛控參數讀回（每機 34 項＝匯出 32＋平台 2） | 匯出 32 項 × 3 機 ＝ 96 項 | ⟪102／102⟫ 與正規化後期望值相符 | **項數：可預期不變**（32 由 `ardupilot.py:922-987` 決定，酬載與電池模型未變）。**值：必須重產**——`FENCE_ALT_MAX` 取計畫最高相對高度（`ardupilot.py:961-976`，HV4SITL16 為 150）會隨天花板改變。來源：`⟪TRIAL⟫/params_readback.csv`。⚠ 別沿用 HV4SITL16 的 102/102，那是「靜默吃舊參數」 |
| 8 | 任務上傳（項數） | ⟪xx／xx／xx⟫ | ⟪讀回一致⟫ | **待飛**，`⟪TRIAL⟫/mission_upload.json`。舊值 82／53／84 對應 120 m 天花板，**不可沿用**。參考：`reduce_waypoints` 後之航點數在 119.5 m 為 73／46／73（`ablation_cap.json`），與任務項數不同（後者另含速度／相機指令） |
| 9 | *(新增，若版面允許)* 模擬風讀回 | 5.0 m/s ／ 270° | ⟪SIM_WIND_SPD 5.0、DIR 270、TURB 0，ok⟫ | **待飛**，`⟪TRIAL⟫/wind_readback.json`（`hv_sitl_run.py:512-523`，不生效即中止起飛）。這是「風真的開了」的唯一硬證據 |

**表 2 表註（三句，缺一不可）**
1. ⟪本表之架次規劃與模擬均採 5 m/s／270° 風場；風場生效以參數讀回確認，東西向平均地速差實測 ⟪x.x⟫ m/s（理論 10.0 m/s）。⟫
2. makespan 以觸地且上鎖判定（以高度 <2 m 判定為 ⟪xxxx.x⟫ s）；覆蓋率與間距為航跡回算上界。
3. 參數「相符」係指讀回值與韌體正規化後之期望值差異 ≤ max(0.01, 0.1 % × 期望值)；正規化含 1 類刻意覆寫（`Q_RTL_MODE`，3 機）與 1 類整數化（`AIRSPEED_MAX`，2 架 HERO）。原始規劃值嚴格比對為 91／96。
4. 規劃天花板為 119.5 m（作業上限 120 m、法規四百呎 = 121.92 m）；0.5 m 裕度之理由見 §3.2。產物中之 `waiver_max_agl_m` 欄位為天花板旋鈕，非實際申請之豁免。

---

## 表 2b（新增）風場對照矩陣骨架

| 標籤 | 規劃天花板 | 規劃風 | 模擬風 | makespan 規劃／實飛（誤差） | 航程 規劃／實飛（誤差） | 最小水平間距 | 東西向地速差 | 來源 |
|---|---|---|---|---|---|---|---|---|
| HV4SITL16（既有） | 120.0 m | 5／270 | **0** | 1260.4／1268.0（**+7.6 s, +0.60 %**） | 30.658／31.358 km（**+2.28 %**） | 105.5／**101.0** m | **≤ 0.01 m/s** | **既有產物**：`HV4SITL16/`；`c2_wind/evidence/planned_vs_flown_baseline.json`、`wind_absence_probe.json` |
| ⟪HV4C1W00⟫ | 119.5 m | 0 | 0 | ⟪1205.x⟫／⟪⟫ | ⟪30.4x⟫／⟪⟫ | ⟪⟫ | ⟪≈0⟫ | **待飛**。規劃欄可先跑 `plan` 取得；W=0／120 m 的參考值為 makespan 1205.48 s、30.435 km（`c2_wind/evidence/planner_wind_sweep.json`，我今天重跑同值） |
| ⟪HV4C1W55⟫（**主結果**） | 119.5 m | 5／270 | 5／270 | **1260.5**／⟪⟫ | **30.66**／⟪⟫ | **105.7**／⟪⟫ | ⟪10.0 ± 1.0⟫ | 規劃欄**已算出**（`ablation_cap.json` 119.5 列）；飛行欄**待飛** |
| ⟪HV4C1W05⟫ | 119.5 m | **0** | 5／270 | ⟪1205.x⟫／⟪⟫ | ⟪30.4x⟫／⟪⟫ | ⟪⟫ | ⟪10.0 ± 1.0⟫ | **待飛**。這一格就是「忽略風場的代價」 |
| ⟪HV4C1W55S⟫（方向對照，選配） | 119.5 m | 5／**180** | 5／**180** | ⟪966.x⟫／⟪⟫ | ⟪33.5x⟫／⟪⟫ | ⟪⟫ | ⟪南北向差 10.0⟫ | **待飛**。⚠ 規劃層實測（120 m、5/180）為 makespan **966.2 s**、總航程 **33.509 km**、A1 由 10.293 → **13.603 km**：差異主要來自**配置與排程改變**，不是風成本，**不可與其他列直接比 makespan**。條帶方位仍為東西向（`prep_text/strip_heading_vs_wind_dir.json`），故此列確實測到側向分量 |

**表 2b 表註**
- 「規劃風」為規劃器假設之風，「模擬風」為 SITL 機體實際遭遇之風，兩者為獨立旋鈕（`scripts/hv_sitl_run.py:62-65, 785-789`）。
- 風場生效判準：東西向平均地速差 10.0 ± 1.0 m/s（Pioneer 26.5 對 16.5、HERO 24.0 對 14.0 m/s）；未達 5 m/s 即判該架次作廢。
- 紊流係數固定為 0，以維持架次間之可重現性。
- 平均風 ≥ 6.2 m/s 時本流程判定為 NO_GO（陣風 = 1.3 × 風速，場址陣風上限 8 m/s），故未進行 8 m/s 之架次。

---

## 飛完後的 5 分鐘檢查清單

1. `⟪TRIAL⟫/wind_readback.json` 三機 `ok` 皆為真 → 否則該架次作廢，不要進表。
2. `⟪TRIAL⟫/params_readback.csv` 列數 = 102、`ok` 全真 → 表 2 第 7 列。順手確認 `FENCE_ALT_MAX` 的新值並更新附錄 B。
3. `⟪TRIAL⟫/plan/mission_summary.json` 的 makespan 與逐機 distance **應等於** 1260.45 s、10.2936／7.9265／12.4406 km（`ablation_cap.json` 119.5 列）。**不相等就先查清楚再飛下一趟**——代表規劃輸入與消融實驗不同。
4. `⟪TRIAL⟫/{ac}/flight.csv` 的最高相對高度 **必須 < 120.000** → C1 的驗收條件；否則 119.5 m 的裕度不夠，要再往下壓。
5. `⟪TRIAL⟫/plan/A1.param` 的 `RTL_RADIUS` 應仍為 200（B 為 120）→ 證實 $R_{eff}$ 在 W = 5 仍是 $R_{plan}$（C6 的誠實註記靠這一行）。
6. 確認 `runs/artifacts/HV4SITL15`、`HV4SITL16` 仍在且未被覆寫。
