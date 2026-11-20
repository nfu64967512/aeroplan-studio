# -*- coding: utf-8 -*-
"""Edit list, part 2 (section 3 results, conclusion, limitations, references)."""
from edits_part1 import L

E = []

# ---------------------------------------------------------------- 3.1
E.append(dict(
    id="E18", serves="決定 1（1.2434 km²）、C1-b（法規 121.92 m 與規劃天花板 119.5 m）、C2-a（規劃風與模擬風為獨立變因、解鎖前讀回）、K1（間距量尺）、N2（完工時間判準）",
    evidence="NUMBERS.json survey_geometry（area_m2 1 243 447.5、flat 1 248 000）、conventions（separation_frame、makespan、coverage）；metrics.json *.altitude.per_uav.rate_hz（5.1–6.4 Hz）；HV4W55/wind_readback.json（15 列）；HV4W55/scenario.json",
    old=L(177),
    new=(
        "作業區為標稱 1560 m × 800 m 之平地矩形（WGS84 橢球面積 1.2434 km²），由系統自動切為 3 個子區；"
        "機隊為 1 架 Pioneer（A1）與 2 架 HERO（B2、B3），起飛坪相距 10 m，共用降落場坪 H；"
        "空域上限取法規四百呎（121.92 m）以下之 120 m AGL，不申請豁免，規劃天花板設為 119.5 m 以保留匯出離散化與高度追蹤之裕度（3.3 節）；"
        "最低固定翼作業高度 100 m，目標 GSD 0.05 m/px。規劃風與模擬風設為獨立變因（表3），主結果兩者均為 5 m/s、270°；"
        "模擬風於解鎖前讀回確認（三機 15 項），紊流為 0、不隨高度變化。"
        "飛行採 ArduPlane 4.8.0-dev QuadPlane SITL（內建機體模型、5 倍速），依規劃起飛時刻逐機放行。"
        "覆蓋率以航跡加刈幅緩衝回算，為上界；間距以飛控原生 5.1–6.4 Hz 紀錄於 WGS84 橢球座標求連續時間最小值；完工時間以最後一架下穿 2 m 判定。"
    ),
))

# ---------------------------------------------------------------- 3.2
E.append(dict(
    id="E19", serves="C1-c（高度句取代「貼齊上限」）、K1（100.43 m、垂直差 28.4 m）、C4-b＋決定 4（34 項之定義；91/96 到 float32 精度；85/96 逐位元）、K2",
    evidence="NUMBERS.json trials.HV4W55.altitude（119.6221／119.6356／119.6752、t_above_120_s 0.0）、.separation（100.4256、vert_at_min_m 28.3942、margin 0.4256）、.params（strict_float32 91/96、strict_exact 85/96）；metrics.json HV4W55.timing.per_uav（dist_err 3.813／4.675／3.736%）；HV4W55/params_readback.csv（EXPECT_FALLBACK = ARMING_REQUIRE／SCHED_LOOP_RATE；FIRMWARE_OVERRIDE = Q_RTL_MODE；AIRSPEED_MAX 26.6 → 26.0）；NUMBERS.json corrections.K2_param_strict_comparison（6 筆 float32 往返）",
    old=L(181),
    new=(
        "結果如表2（架次 HV4W55）。三機完成起降，全程最高 119.675 m AGL、高於 120 m 之時間 0 s；"
        "最小水平間距 100.43 m（B2–B3）僅餘 0.43 m（見限制 (2)）。"
        "參數每機比對 34 項：任務產生器匯出 32 項，另 2 項（`ARMING_REQUIRE`、`SCHED_LOOP_RATE`）為 SITL 平台設定而非韌體需求；"
        "「相符」指與韌體正規化後期望值之差 ≤ max(0.01, 0.1%)，正規化含 `Q_RTL_MODE` 刻意覆寫（3 機，韌體不接受其與 `RTL_AUTOLAND` 同時啟用）與 `AIRSPEED_MAX` 整數化（2 機）；"
        "對原始規劃值（96 項）則到 float32 精度為 91／96、逐位元為 85／96。"
    ),
))

# ---------------------------------------------------------------- Table 2
E.append(dict(
    id="E20", serves="C1／C2（主結果改為 HV4W55）、決定 1、K1、K2、C4-c、C2-c（表註第一句）、N2",
    evidence="TABLES_ZH.md 表 2；NUMBERS.json trials.HV4W55（coverage、altitude.exported_max_rel_m 119.5526、makespan 1260.454 → 1281.906（+1.702%）、separation（planned 105.6992、n_paired_samples 7309、n_violations 0）、distance（+4.004%）、wind_test.signal_100m 9.9526／9.9314／9.9575）；metrics.json HV4W55.timing.per_uav（10.2936／7.9265／12.4406 → 10.686／8.297／12.9054 km）；HV4W55/{mission_upload.json, wind_readback.json, scenario.json}",
    old=L(183, 196),
    new=(
        "**表2 規劃與 SITL 飛行結果對照（1 Pioneer + 2 HERO；架次 HV4W55）**\n\n"
        "| 項目 | 規劃 | SITL 飛行 |\n"
        "|---|---|---|\n"
        "| 起降完成 | 3 架 | 3／3 |\n"
        "| 母區覆蓋率（1.2434 km²） | 未覆蓋 0 | 99.97%（上界） |\n"
        "| 全程最高高度 AGL（上限 120 m） | 匯出 ≤119.553 m | 119.622／119.636／119.675 m |\n"
        "| 航程 A1／B2／B3（km） | 10.29／7.93／12.44 | 10.69／8.30／12.91（+4.0%） |\n"
        "| 完工時間 makespan（s） | 1260.5 | 1281.9（+1.70%） |\n"
        "| 最小水平間距（m；門檻 100） | 105.70 | 100.43（垂直 28.4；違規 0/7309） |\n"
        "| 飛控參數讀回（34 項 × 3 機） | 匯出 32 項 × 3 機 | 102／102 相符（原始值：float32 精度 91／96，逐位元 85／96） |\n"
        "| 任務上傳（項數） | 82／53／84 | 讀回一致 |\n\n"
        "表註：規劃風與模擬風均為 5 m/s、270°（解鎖前讀回 15／15，風判準見表3）；百分比對完工時間判準（3.1 節）敏感，引用須連同本註；"
        "每機 32 項為本情境之匯出項數；`waiver_max_agl_m: 119.5` 為天花板設定而非豁免；平面近似之 1.248 km² 高估 0.37%。"
    ),
))

# ---------------------------------------------------------------- new 3.3 (+ Table 3, Fig 3), renumbered 3.4
E.append(dict(
    id="E21", serves="C1-d（離散化機制＋單變因對照，以 HV4W55R 為準）、K6（+0.0279 s）、C2-b（生效、忽略風之代價、買到的是準確度）、C2-h（側向分量；不跨風向比較）、C2-i＋K3（生成位置揭露）、K1（表3 間距）；圖3 改用 fig5_compliance",
    evidence="NUMBERS.json ceiling_control（exported n_above_120 [0, 88]、flown max_agl [119.6752, 120.1742]、t_above_120_s [0.0, 354.925]、planned_makespan_cost_of_1195_s 0.027894、identical_files 6）；trials.*（makespan、separation.flown_exact_m／plan_minus_flown_m、altitude、wind_test.signal_100m／signal_30m／null_30m／ekf_wind）；crosstrack_wind_triangle.residual_mps（0.041／0.0061／0.0246）；corrections.K3_K4_spawn（err 5.0056／5.0607／5.1081 m、max_abs_spacing_delta 0.0218 m、benign cells ≤ 0.2146 m）；metrics.json *.windaxis.ew_half_sum（0.3864–0.4453）、*.timing.per_uav（A1 放行 612 → 656 s）；hv_dccpp/ardupilot.py:388-395、hv_dccpp/sim.py:58；c2_wind/evidence/r_eff_and_gust_limits.json（6.154 m/s）",
    old=L(198, 203),
    new=(
        "#### 3.3 規劃天花板與風場之對照\n\n"
        "**規劃天花板。** 任務匯出將圓弧重新離散為多邊形、新頂點高度以線性內插，而等 AGL 曲面於局部東-北-天座標為拋物面，故迴轉頂點可高出天花板，"
        "且規劃驗證器（只檢查規劃航點、容差 1 m）無法察覺。HV4W55R 與 HV4W55 僅天花板不同（同風場、同匯出項數、參數檔逐位元相同）："
        "120.0 m 時匯出 88 個航點高於 120 m、實飛最高 120.174 m、累計 354.93 s 高於 120 m，119.5 m 時皆為 0（表3、圖3(a)），代價為規劃完工時間 +0.0279 s，此係本作業區之性質，不宜外推。\n\n"
        "**風場。** 表3 之運動學判準（東向減西向之地速 − 空速，理論 2W）與飛控 EKF3 風速（4.93–4.97 m/s）證明模擬風確實生效。"
        "HV4W05 與 HV4W00 飛同一任務檔、僅模擬風不同，完工時間增加 26.90 s；含風規劃之預測誤差較小（+1.70% 對 +2.43%），完工時間卻晚 47.13 s（44.0 s 為刻意延後放行），"
        "即買到的是預測準確度而非速度。風向 180° 之 HV4W55S 以北−南判準與側風地速虧損（與 $\\sqrt{V_a^2-W^2}-V_a$ 相差 ≤ 0.041 m/s）驗證風三角之側向分量；"
        "兩風向為規劃器各自重解之計畫，不比較其完工時間。\n\n"
        "**表3 規劃天花板與風場之對照架次（1 Pioneer + 2 HERO，同一情境）**\n\n"
        "| 架次 | 天花板 (m) | 規劃風／模擬風 | 高於 120 m (s) | makespan 規劃→實飛 (s) | 實飛間距 (m) | 風判準 (m/s) |\n"
        "|---|---|---|---|---|---|---|\n"
        "| HV4SITL16 | 120.0 | 5/270°／無 | 330.14 | 1260.4→1255.9 | 100.685 | 東−西 −0.019 至 +0.005 |\n"
        "| HV4W00 | 119.5 | 0／0 | 0 | 1205.5→1207.9 | 100.435 | 東−西 −0.009 至 +0.009 |\n"
        "| HV4W05 | 119.5 | 0／5/270° | 0 | 1205.5→1234.8 | 100.291 | 東−西 +9.915 至 +9.969 |\n"
        "| HV4W55 | 119.5 | 5/270°／5/270° | 0 | 1260.5→1281.9 | 100.426 | 東−西 +9.931 至 +9.958 |\n"
        "| HV4W55R | 120.0 | 5/270°／5/270° | 354.93 | 1260.4→1282.2 | 100.579 | 東−西 +9.934 至 +9.960 |\n"
        "| HV4W55S | 119.5 | 5/180°／5/180° | 0 | 966.2→996.9 | 266.177 | 北−南 +9.512 至 +9.853 |\n"
        "| W ≥ 6.16 m/s | — | — | — | 規劃器判定 NO_GO，未出動 | — | — |\n\n"
        "表註：(a) 風判準＝巡航段逐航向 90° 扇區之（地速 − 空速）中位數差，理論 2W = +10.0 m/s，並消去與航向無關之尺度偏移；"
        "180° 架次改取相對高度 30 m 以上樣本（其餘 100 m），其東−西為 −0.046 至 +0.014 m/s。"
        "(b) HV4W55S 三機於解鎖前沿地面偏離起飛站 5.0–5.1 m（剛體平移，間距變化 ≤ 0.022 m），係在明示且記錄之容差覆寫（2.0 → 6.0 m）下放行；"
        "位移方向鎖在機體縱軸，量值未能自模擬器原始碼導出；其餘七趟 ≤ 0.215 m。"
        "(c) 陣風取 1.3 倍風速、場址上限 8.0 m/s，W ≥ 6.16 m/s 即 NO_GO。"
        "(d) HV4SITL16 與 HV4SITL15（未列）為同計畫重飛（完工時間差 0.025 s），模擬未設風，與 119.5 m 各趟差兩個變因。"
        "(e) 規劃間距為 2 s 網格上界，較實飛高 4.20–5.27 m（HV4W55S 56.20 m）；違規皆 0；每格一趟。\n\n"
        "> 【插入圖3：`fig5_compliance.png`，寬 17.0 cm，置中】\n"
        "> 圖說：見附錄 C\n\n"
        "#### 3.4 起降走廊判準對照\n\n"
        + L(203).replace("（表3）", "（表4）")
    ),
))
E.append(dict(
    id="E22", serves="表號順延（原表3 → 表4）",
    evidence="本檔第三節",
    old="**表3 起降走廊豁免判準對照（1 Pioneer + 3 HERO）**",
    new="**表4 起降走廊豁免判準對照（1 Pioneer + 3 HERO）**",
))

# ---------------------------------------------------------------- Table 4 note + new 3.5
E.append(dict(
    id="E23", serves="決定 5（PAD8 未納管、第三方無從核對）、K1（表4 仍為舊量尺，須明講）；新增 3.5 節（C3-c、C3-f／K5：BEAMIDP1 只證可執行、完工時間不引用）",
    evidence="`git ls-files runs/artifacts/PAD8_BROKEN_RAW runs/artifacts/PAD8_FIXED_RAW` = 0 筆（.gitignore:109）；PAPERPLAN1/prep_beamidp/tables.json（fleet seq_t 1696.88、bid_t 1115.73、hk_t 1060.15、fleet_delta_turn_time_pct −34.25）；各胞 n：Pioneer 6／7、HERO 13／7／6（同檔 table_md_en）；NUMBERS.json trials.BEAMIDP1（separation.flown_exact_m 180.7735、altitude.t_above_120_s 0.0）、corrections.K5_beamidp_makespan_confound（sim wind 0）；BEAMIDP1/mission_upload.json（263／165／175 送出＝讀回）",
    old=L(212),
    new=(
        "表註：回收站自動指派關閉；間距為 2 s 網格內插上界（既有報表程式之量尺，非 3.1 節之方法）；單一情境、單次執行；"
        "兩臂原始資料（`PAD8_BROKEN_RAW`／`PAD8_FIXED_RAW`）未納入版本控制，第三方目前無從核對。\n\n"
        "#### 3.5 跳帶排序之觸發情境\n\n"
        "表2 情境之各胞條帶數（Pioneer 1、HERO 2）未超過 k，排序無作用（限制 (3)）。"
        "另設觸發情境（作業區改為三塊標稱 1600 × 1200 m，其餘同表2；各胞 n = 6–13 > k），以規劃器自身之考慮風場 Dubins 胞內成本評分："
        "全機隊迴轉時間由順序掃描之 1696.88 s 降為 1115.73 s（−34.25%），距 Held–Karp 最佳解仍高 5.24%，均為規劃層模型值。"
        "交錯航線之可執行性另經 SITL 飛行確認（BEAMIDP1：三機完成起降、最小間距 180.77 m、高於 120 m 之時間 0 s；模擬未設風，其完工時間不引用）。"
    ),
))

# ---------------------------------------------------------------- 4 conclusion
E.append(dict(
    id="E24", serves="結論（C1：刪「高度貼齊上限」；C2；C4-d；C7-a；K6 用 +0.0279 s）",
    evidence="PAPER_TEXT_ZH.md「第四節結論」；NUMBERS.json ceiling_control、trials.HV4W55",
    old=L(216),
    new=(
        "HARVEST 將考慮風場之飛行時間成本、跳帶迴轉、決策級機數配額、VTOL 起降能量與法規高度帶、起降走廊互斥與四維隔離整合於同一規劃流程，並匯出至 ArduPilot QuadPlane。"
        "5 m/s 模擬風場之 SITL 閉迴路飛行中三機完成起降、最高高度低於 120 m、參數讀回 102／102 相符；"
        "單變因對照顯示規劃天花板降 0.5 m 即可消除 354.93 s 之超高飛行，代價 +0.0279 s；納入風場換得的是預測準確度而非較短之完工時間；"
        "走廊判準對照則顯示覆蓋率不足以作為唯一驗收條件。"
    ),
))

# ---------------------------------------------------------------- 4 limitations
E.append(dict(
    id="E25", serves="限制：C2-d（1）、K1（2）、C3-a（3，原句「少於 3」是錯的）、C5-b（4）、C6-c（5）、C2-e（6）、C1-e（7）、（8）",
    evidence="NUMBERS.json trials.HV4W55.separation（margin 0.4256）、trials.HV4W05.separation（margin 0.2909）、trials.BEAMIDP1.altitude（margin 0.3031）；metrics.json *.altitude.per_uav（匯出超出 0.0136–0.1703、追蹤超出 0.0265–0.1475 m）；prep_beamidp/beamidp_table.CORRECTION.json（HERO 胞 42.35 s）；prep_baseline_control/out/stepP_final_table.json（efficiency 0.910–22.873%、coverage 0.5277／0.617、d_ceq 109.923／111.825 對 D_model 97.726 km）；c6_reff/reff_numbers.json（W* 5.1675／5.0333）；c2_wind/evidence/r_eff_and_gust_limits.json（6.154）；各趟 plan/*.param（RTL_RADIUS／WP_LOITER_RAD 八趟皆 200／120）；hv_dccpp/allocate.py:694-724",
    old=L(218),
    new=(
        "本研究之限制：(1) 僅以 SITL 內建機體模型驗證；風場為單層、定常、無紊流；W ≥ 6.16 m/s 即 NO_GO，風場驗證上限為 5 m/s（情境預設值，非場址實測）。"
        "(2) 覆蓋率為上界；間距餘裕僅 0.426 m（八趟最緊 0.291 m），且最近接時刻垂直差 28.4 m 低於 50 m 門檻，水平間距為唯一防線，同型 HERO 間無垂直隔離通道；規劃間距為上界，不蘊含實飛 ≥ 100 m。"
        "(3) 跳帶排序之觸發條件為單胞條帶數 n > k = ⌈2R_eff/s⌉，而非「條帶數少於 3」；表2 情境 n = 1／2 未超過 k = 5／3，四種條帶序之迴轉成本逐項相同，此缺口源自切區結果；"
        "3.5 節之效果為模型值，順序掃描對照組無法實飛。"
        "(4) 最長單機航程於封閉基準 10 組情境均較團隊既有之 DCCPP 實作長 0.91%–22.87%，本文不主張航程效率；另 2 組 HARVEST 自判不可行而回報未覆蓋（覆蓋率 0.528、0.617），"
        "恰為基準航線超出配置器能量預算 12.5% 與 14.4% 者，其成因（切區後才計算每胞機數而不再細切）列為後續工作。"
        "(5) W = 5 m/s 低於 W* = 5.1675／5.0333 m/s，R_eff 退化為 R_plan（匯出之迴轉半徑逐趟為 200／120 m），其風修正僅在 W* 至 6.15 m/s 生效，本文未驗證。"
        "(6) 條帶方位不隨風向調整；270° 與 180° 架次為各自重解之不同計畫。"
        "(7) 圓弧匯出離散化缺陷（3.3 節）以降低天花板迴避而未修正：匯出偏差 +0.014 至 +0.170 m、飛控追蹤另 +0.027 至 +0.148 m，此裕度不可外推（最壞餘裕 0.303 m）。"
        "(8) 各條件僅 1 趟、單一作業區，SITL 為決定性模擬。後續將進行硬體上模擬、陣風與分層風對照及轉彎半徑消融實驗。"
    ),
))

# ---------------------------------------------------------------- references
E.append(dict(
    id="E26", serves="決定 2（新增參考文獻 [15]）",
    evidence="使用者提供之書目：Techy & Woolsey, JGCD 32(6), 2009, pp. 1736-1746, DOI 10.2514/1.44580（⚠ 本輪未連網，未以 Crossref 複核）",
    old=L(240),
    new=L(240) + "\n"
    "[15] Techy, L., and Woolsey, C. A., \"Minimum-Time Path Planning for Unmanned Aerial Vehicles in Steady Uniform Winds\", "
    "Journal of Guidance, Control, and Dynamics, Vol. 32, No. 6, pp 1736-1746, 2009.",
))
