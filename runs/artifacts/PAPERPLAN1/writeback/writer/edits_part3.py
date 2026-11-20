# -*- coding: utf-8 -*-
"""Edit list, part 3 (appendices A-E; none of these are printed in the Word file)."""
from edits_part1 import L

E = []

# ---------------------------------------------------------------- Appendix A
E.append(dict(
    id="E27", serves="決定 2（先行文獻含 [15]）",
    evidence="本檔參考文獻 [15]",
    old="| 各元件均有先行文獻 [5]–[13] |",
    new="| 各元件均有先行文獻 [5]–[13]、[15] |",
))
E.append(dict(
    id="E28", serves="C2-g（原列「必須附 SITL 未設風表註」已過時：主結果現為設風架次）",
    evidence="PAPER_TEXT_ZH.md C2-g；NUMBERS.json trials.HV4W05／HV4W55（+2.43% → +1.70%；1281.906 − 1234.780 = 47.126 s）；metrics.json 放行 612 → 656 s",
    old=L(253),
    new="| 「規劃與實飛時間只差數秒」／「含風規劃較快」 | 「含風規劃之預測誤差較小（+1.70% 對 +2.43%），但完工時間反而長 47.13 s，其中 44.0 s 是刻意延後放行」 | 含風規劃買到的是預測準確度；百分比對 t0 判準敏感，須連同表2 表註引用 |",
))
E.append(dict(
    id="E29", serves="C4-e＋K2（102/102 必須給定義；「嚴格 91/96」會被重算成 85/96）",
    evidence="NUMBERS.json corrections.K2_param_strict_comparison",
    old=L(254),
    new=L(254) + "\n"
    "| 「參數讀回 102／102 相符」（不給定義）／「原始值嚴格比對 91／96」 | 「102／102 與韌體正規化後期望值相符；到 float32 精度之原始值比對為 91／96，逐位元為 85／96」 | 判準 max(0.01, 0.1%)；6 筆為 float32 往返表示誤差 |",
))
E.append(dict(
    id="E30", serves="C3-b（門檻為 n > k）＋PAPER_TEXT 附錄 A 彙總之其餘新增列（C1、K1、K3–K6、K8、K9、C5、C6、C7、決定 1、決定 3）",
    evidence="PAPER_TEXT_ZH.md「附錄 A 新增／修改彙總」；CORRECTIONS_K1_K9.md；c7_costfn（Pioneer 順風 26.5／逆風 16.5／側風 √(21.5²−5²) = 20.91 m/s）",
    old=L(259),
    new=(
        "| 「以 skip-5／skip-3 飛行」 | 「公式值 k = 5／3；本情境每胞 n ≤ k，固定跳帶序列退化為順序掃描，未觸發」 | 真正門檻是 n > k，不是「條帶數少於 3」 |\n"
        "| 「所飛之條帶序與順序來回掃描完全一致」 | 「本情境無可改變成本之排序自由度，四種序之成本逐項相同」 | 序不同（若干胞為 [1, 0]），相同的是成本 |\n"
        "| 「Beam-IDP 與 Held–Karp 只差 0.01%」 | 「高出 5.24%，單胞最多 9.11%」 | 0.01% 是單機 n = 9 的探測 |\n"
        "| 「BEAMIDP1 完工時間 −3.22%」 | 不列；若列須原句揭露「含風規劃、無風模擬」 | 與 HV4SITL15／16 同一缺陷（K5） |\n"
        "| 「高度貼齊 120 m 上限」 | 「低於 120 m 作業上限（規劃天花板 119.5 m）」 | 120 m 天花板時實際超出；單變因對照量得 354.93 s |\n"
        "| 「降天花板的代價只有 +0.02 s」 | 「+0.0279 s」或「1260.4261 → 1260.4540 s」 | +0.02 s 是四捨五入假影（K6） |\n"
        "| 「最小水平間距 101.0 m」「重飛 104.2 m」 | 「100.43 m（HV4W55）」；HV4SITL15／16 為 100.69 m | 舊值用平面近似緯度換算＋2 s 網格上界 |\n"
        "| 「與既有報表程式之值差 3.4–5.3 m」 | 「差 0.31–3.51 m（對 2 s 網格報表）或 0.34–0.90 m（對原生取樣平面框）」 | 3.4–5.3 m 不可重現（K1） |\n"
        "| 「作業區 1.248 km²」 | 「1.2434 km²（WGS84 橢球）」 | 1.248 為平面近似，高估 0.37% |\n"
        "| 「迎風時間成本」「逆風速度」「含風時間成本」 | 「考慮風場之飛行時間成本」 | 程式算完整風三角：Pioneer 順風 26.5／逆風 16.5／側風 20.9 m/s，三項皆有飛行佐證 |\n"
        "| 「180° 比 270° 快」「側風預測較差」 | 不寫 | 兩者為規劃器各自重解之不同計畫 |\n"
        "| 「R_eff 為本文提出」「R_eff 出自 [5]」「R_eff 保證任何相位傾角 ≤ φ_max」 | 「R_eff 為定常風中 trochoid 之既有結果[15]；本文之選擇是取最壞相位曲率」 | [5] 無 (1+W/V_a)²；HERO 在 W = 5 m/s 需 26.07° > 25° |\n"
        "| 「擺線（cycloid）」 | 「trochoid（次擺線）」 | [5] 原文用 trochoid |\n"
        "| 「Wang 等人以轉彎半徑 4000 m…」 | 不寫該數字 | 未經原文核對（決定 3） |\n"
        "| 「表5 是同一把尺」 | 「HARVEST 的 0 來自其原生四維排程，基準沒有這個排程；代價為起飛佇列」 | 兩臂受檢物件不同（K9） |\n"
        "| 「取自已納管的基準 CSV」 | 「該 CSV 目前未納管，`bench.py` 亦為已修改狀態」 | `git status` 為 `??` 與 ` M` |\n"
        "| 「其餘 10 格只用到 17.0%–68.7%」／「量測工具校準相差 1.62%」 | 「23.2%–68.7%」／「誤差帶 −6.86% 至 +2.31%」 | 17.0%、1.62% 在產物中不存在 |\n"
        "| 「180° 為純順風，把停放機體往北推 5 m」 | 「機體在地面自由偏航轉向來風，位移鎖在機體縱軸；5 m 是圓弧的弦，方位 43.6°–44.4°」 | 位移方位為東北（K3） |\n"
        "| 「兩次獨立執行逐位元重現」 | 「生成位置逐位元重現」 | `spawn_readback.json`／`report.json` 兩檔不同（K4） |\n"
        "| 「這個機體沒有電池遙測」 | 「SITL 記錄的是通用 12.6 V 模擬電池，不是規劃模型之電池，故能量餘裕為規劃模型值」 | `BAT` 訊息每趟每機都有（K8） |"
    ),
))

E.append(dict(
    id="E30b", serves="表號順延（附錄 A 之「表3 即反例」→ 表4）",
    evidence="本檔 3.4 節（原 3.3 節）",
    old="| 「覆蓋率 100% 證明計畫可行」 | 「覆蓋率為必要而非充分之驗收條件」 | 表3 即反例 |",
    new="| 「覆蓋率 100% 證明計畫可行」 | 「覆蓋率為必要而非充分之驗收條件」 | 表4 即反例 |",
))

# ---------------------------------------------------------------- Appendix B
E.append(dict(
    id="E31", serves="附錄 B 出處全面改指向新架次（C1–C7、K1、K2、K7、決定 1、決定 5）",
    evidence="各列自身所列之出處",
    old=L(267, 285),
    new=(
        "| 全部飛行數字（表2、表3、3.3 與 3.5 節、限制） | `runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json`（自 24 個 .BIN 重算；明細在同目錄 `metrics.json`、`wind.json`、`corrections.json`、`spawn_verification.json`） |\n"
        "| 主結果情境：1 Pioneer + 2 HERO、規劃天花板 119.5 m、HFOV 60°、重疊 0.30、規劃風與模擬風 5 m/s／270°、GSD 0.05、h_minfw 100、起飛坪相距 10 m | `runs/artifacts/HV4W55/scenario.json`、`wind_readback.json`（15／15） |\n"
        "| 作業區 WGS84 1559.6 × 797.3 m、1.2434 km²（平面近似 1 248 000 m²，高估 0.37%）；自動切 3 子區 | `NUMBERS.json` `survey_geometry`；`HV4W55/plan/mission_summary.json` |\n"
        "| 刈幅／間距：119.5 m 時 137.99／96.59 m；120.0 m 時 138.56／96.99 m | `NUMBERS.json` `survey_geometry.swath_and_spacing` |\n"
        "| 規劃航程 10.2936／7.9265／12.4406 km；飛行 10.6860／8.2970／12.9054 km；makespan 1260.454 → 1281.906 s（下穿 2 m） | `update_paper/out/metrics.json` `HV4W55.timing`；`NUMBERS.json` `trials.HV4W55` |\n"
        "| 最高高度 119.6221／119.6356／119.6752 m、高於 120 m 0.00 s；匯出最高 119.5526／119.5332／119.5331 m | `metrics.json` `HV4W55.altitude` |\n"
        "| 覆蓋率 99.9666%（未覆蓋 415.2 m²，刈幅 137.99 m） | `NUMBERS.json` `trials.HV4W55.coverage` |\n"
        "| 最小水平間距 100.4256 m（B2–B3，垂直 28.394 m），違規 0／7309；規劃 105.699 m | `NUMBERS.json` `trials.HV4W55.separation` |\n"
        "| 參數 102/102（匯出 93、韌體覆寫 3、常數備援 6 即 `ARMING_REQUIRE`／`SCHED_LOOP_RATE`；`AIRSPEED_MAX` 整數化 2）；原始值比對到 float32 精度 91/96、逐位元 85/96；判準 max(0.01, 0.1%)；關閉整數救援則 100/102 | `HV4W55/params_readback.csv`；`NUMBERS.json` `corrections.K2_param_strict_comparison` |\n"
        "| 任務 82/53/84（送出＝讀回） | `HV4W55/mission_upload.json` |\n"
        "| 表3 與 3.3 節之風判準（東−西／北−南、EKF 風）與 30 m／100 m 門檻 | `NUMBERS.json` `trials.*.wind_test`；`update_paper/out/wind.json` `floors` |\n"
        "| HV4SITL16 未設風：`sitl_cmdlines.txt` 無 `SIM_WIND`；巡航段東西向差 +0.005／−0.002／−0.019 m/s（HV4SITL15 為 −0.001／−0.018／+0.003） | `HV4SITL16/sitl_cmdlines.txt`；`NUMBERS.json` `corrections.K7_appendixB_276` |\n"
        "| 天花板單變因對照：88 → 0 項、120.1742 → 119.6752 m、354.93 → 0.00 s、+0.0279 s | `NUMBERS.json` `ceiling_control`；`runs/artifacts/{HV4W55,HV4W55R}/plan/mission_summary.json` |\n"
        "| 離散化成因與驗證器盲點 | `hv_dccpp/ardupilot.py:388-395`；`hv_dccpp/sim.py:58`（`_AGL_TOL_M = 1.0`） |\n"
        "| 側風三角殘差 +0.041／+0.006／+0.025 m/s | `NUMBERS.json` `crosstrack_wind_triangle` |\n"
        "| HV4W55S 生成位置 5.006／5.061／5.108 m、剛體平移 0.022 m | `HV4W55S/spawn_readback.json`；`update_paper/out/spawn_verification.json`；`NUMBERS.json` `corrections.K3_K4_spawn` |\n"
        "| NO_GO 6.16 m/s（陣風 1.3 × W、上限 8.0 m/s；W = 7 回 9.1 > 8.0） | `hv_dccpp/integration/aeroplan_adapter.py:961`；`runs/artifacts/PAPERPLAN1/c2_wind/evidence/{planner_wind_sweep_2_4_6_7.json, r_eff_and_gust_limits.json}` |\n"
        "| R_eff、W* 5.1675／5.0333、幾何自洽 129.46／78.92 m、HERO 傾角 26.07°；R_eff = R_plan（W = 5 m/s） | `hv_dccpp/platform.py:296-304`；`runs/artifacts/PAPERPLAN1/c6_reff/{reff_numbers.json, ground_circle_bank.json}`；各趟 `plan/*.param` 之 `RTL_RADIUS`／`WP_LOITER_RAD`（八趟皆 200／120） |\n"
        "| 地速式（完整風三角） | `hv_dccpp/wind.py:278-306`；`hv_dccpp/ardupilot.py:653` |\n"
        "| 表4：5.0/19.9 m、5/378；52.7/112.6 m、0/346 | `runs/artifacts/PAD8_BROKEN_RAW/`、`PAD8_FIXED_RAW/` 的 `separation_from_tracks.json`、`validation.json`、`coverage_from_tracks.json`（⚠ 被 `.gitignore:109` 排除，未納管） |\n"
        "| 3.5 節：迴轉時間 1696.88 → 1115.73 s（−34.25%）、距離 −33.65%、能量 −32.53%、Held–Karp 1060.15 s；限制 (3)「成本逐項相同」（每個 HERO 胞 42.35 s／758.0 m） | `runs/artifacts/PAPERPLAN1/prep_beamidp/{tables.json, beamidp_table.json, beamidp_table.CORRECTION.json}`；BEAMIDP1 飛行數字 `NUMBERS.json` `trials.BEAMIDP1`、`BEAMIDP1/mission_upload.json` |\n"
        "| 表5／限制 (4)：航程 0.91%–22.87%（平均 13.19%）、覆蓋 0.528／0.617、能量 112.5%／114.4%、133 km 時最緊 95.4%、誤差帶 −6.86% 至 +2.31%、24 組態、衝突 114／572／959／688／753、起飛佇列 330–6460 s、最後落地 2576.7–9851.3 s | `runs/artifacts/PAPERPLAN1/prep_baseline_control/out/{stepP_final_table.json, stepI_dplan_rule.json, stepC_kpi_scan_control.json}`；`NUMBERS.json` `corrections.K9_table_z_rulers`；`docs/hv_dccpp/封閉基準_2026-09-10.md:36-57` |\n"
        "| 限制 (7)：匯出超出 +0.0136 至 +0.1703 m、追蹤超出 +0.0265 至 +0.1475 m、最壞餘裕 0.3031 m | `metrics.json` `*.altitude.per_uav`；`NUMBERS.json` `trials.BEAMIDP1.altitude` |\n"
        "| 平台參數 | `docs/hv_dccpp/平台參數表.md`；`hv_dccpp/platform.py:435-478` |\n"
        "| κ_load 0.85、k_unc 1.2075 = k_bm 1.15 × k_as 1.05、d_sep 100/60、Δh_sep 50、k_t 0.05、Δt 2 | `hv_dccpp/mission.py:1035-1043, 1087-1092`；`hv_dccpp/platform.py:951, 976-988` |\n"
        "| σ_max 45 s；走廊 60/30/90 s | `hv_dccpp/deconflict.py:75, 93`；`mission.py:1047-1048` |\n"
        "| λ_floor 實作 | `hv_dccpp/allocate.py:697-723` |\n"
        "| k = ⌈2R/s⌉ 實作 | `hv_dccpp/dubins.py:619-623` |\n"
        "| 法規四百呎 | 全國法規資料庫 民用航空法 第99條之14（pcode=K0090001） |\n"
        "| 參考文獻書目 | 12 個 DOI 以 Crossref 直查；[8][9][10][11][12] 內容以出版社 PDF 第一手核對；⚠ [15] 未以 Crossref 複核（本輪未連網），[4] 之內文數字未核對故已刪 |"
    ),
))
E.append(dict(
    id="E32", serves="決定 2（[15] 之 DOI）",
    evidence="使用者提供 DOI 10.2514/1.44580",
    old=L(288),
    new=L(288) + "　[15] 10.2514/1.44580（未複核）",
))
