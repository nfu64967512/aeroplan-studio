# -*- coding: utf-8 -*-
"""Edit list, part 1 (front matter, abstract, intro, methods, Table 1).

Each edit: dict(id, serves, evidence, old, new). `old` must occur exactly once in the
text at the moment the edit is applied (apply_writeback.py asserts this).
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
BEFORE = ROOT / "runs/artifacts/PAPERPLAN1/writeback/cowork_brief.before.md"
_L = BEFORE.read_text(encoding="utf-8").split("\n")


def L(a, b=None):
    """1-based line(s) of the BACKUP file, joined with a newline."""
    b = a if b is None else b
    return "\n".join(_L[a - 1:b])


E = []

# ---------------------------------------------------------------- header
E.append(dict(
    id="E01", serves="決定 6（版本戳）",
    evidence="使用者決定；`git log` 3442fcb／95a337b／ff38a77",
    old=L(1) + "\n\n> 本檔是",
    new=L(1) + "\n\n**v2026-09-21，對應 commit 3442fcb（證據 95a337b、腳本 ff38a77）**\n\n> 本檔是",
))
E.append(dict(
    id="E02", serves="通則（數字來源改為 NUMBERS.json）",
    evidence="runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json",
    old="（出處見附錄 B）。**請勿改動數字",
    new="（出處見附錄 B；本版依指導老師審閱意見 C1–C7 改寫，飛行數字一律以 `runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json` 為準）。**請勿改動數字",
))
E.append(dict(
    id="E03", serves="C1／K1（舊圖3 內嵌已更正之舊值）＋本輪新圖 fig5／fig4",
    evidence="docs/hv_dccpp/AASRC2026_abstract/fig3_results.png（圖內印有 最小 101.0 m、違規 0/310、飛行最高 120.10/120.12/120.16 m、「法規上限 120 m，不豁免」）；figures_src/fig{3,4,5}_data.json；fig5_compliance.png 2007×732 px（17.0 × 6.2 cm）、fig4_wind.png 2007×708 px（17.0 × 6.0 cm）",
    old=L(13, 14),
    new=(
        "| `fig5_compliance.png`（.svg） | **圖3** 高度合規與實飛間距，300 dpi，寬 17.0 cm（檔名為 fig5，於正文為圖3） |\n"
        "| `fig4_wind.png`（.svg） | **備用圖4** 風場對照，300 dpi，寬 17.0 cm；**本版未排入**，Word 實測有餘時才接在圖3 之後（見 0.4） |\n"
        "| `fig3_results.png`（.svg） | ⚠ **本版不放進 Word**：圖內嵌 HV4SITL16 之舊值（最小間距 101.0 m、違規 0/310、最高 120.10／120.12／120.16 m），已被更正（見附錄 D） |\n"
        "| `figures_src/` | 產圖腳本與數據出處（`fig3_data.json`、`fig4_data.json`、`fig5_data.json`），不放進 Word |"
    ),
))
E.append(dict(
    id="E04", serves="C6（新增式 (4)，式號順延）",
    evidence="本檔 2.3 節",
    old="式號 (1)…(5) 靠右",
    new="式號 (1)…(6) 靠右",
))
E.append(dict(
    id="E05", serves="版面配置（新增 3.3、3.5 節與表3）",
    evidence="本檔第三節",
    old=L(47, 48),
    new=(
        "| 第 3 頁 | 2.3 核心公式 (1)–(6)＋表1；三、SITL 驗證：3.1 設置、3.2 主結果＋表2 |\n"
        "| 第 4 頁 | 3.3 天花板與風場對照＋表3＋圖3；3.4 起降走廊判準對照＋表4；3.5 跳帶排序觸發情境；四、結論與限制；誌謝；參考文獻 |"
    ),
))
E.append(dict(
    id="E06", serves="4 頁限制（刪減順序改寫，附估計頁數）",
    evidence="runs/artifacts/PAPERPLAN1/writeback/writer/CHANGELOG.md 第一節與 length_estimate.json",
    old=L(54, 60),
    new=(
        "> ⚠ 實測：v2026-09-14 稿原樣以 Word 套官方格式檔排版、**未取消文件格線**時約 **6 頁**；取消格線（0.2 節）後的頁數尚未量測。"
        "本版（v2026-09-21）為回應審閱意見 C1–C7 新增 3.3、3.5 節與表3，以字數估計取消格線後約 __PAGES_AFTER__ 頁"
        "（同法估 v2026-09-14 稿約 __PAGES_BEFORE__ 頁，該稿依其刪減順序做完約 __PAGES_BEFORE_CUT__ 頁；估計法見 "
        "`runs/artifacts/PAPERPLAN1/writeback/writer/CHANGELOG.md`；此估計法以舊稿實測校準約偏低 15%）。請先取消格線，再依下列順序刪減，直到 4 頁為止。\n\n"
        "1. 2.1、2.2 節文字各只保留第一句（「系統架構如圖1。」「規劃流程如圖2。」後接圖）。\n"
        "2. 表1 改成內文一句話：「Pioneer／HERO 巡航空速 21.5／19.0 m/s，規劃轉彎半徑 200／120 m（規劃裕度可吸收之風速 W* = 5.1675／5.0333 m/s），"
        "可用航程 133–145／79–86 km（估計值），兩型共用 HFOV 60° 相機。」表1 之表註保留，接在該句之後。\n"
        "3. 3.4 節刪文字，只留表4 與表註。\n"
        "4. 3.5 節只留「另設觸發情境…均為規劃層模型值。」一句。\n"
        "5. 表3 刪 HV4SITL16 與「W ≥ 6.16 m/s」兩列（表註 (c)(d) 保留）。\n\n"
        "⚠ 1–5 項全做後估計仍約 __PAGES_CUT__ 頁。**以下 6–8 項會刪掉實質內容，須由使用者決定，Cowork 不可自行執行**（全做後約 __PAGES_CUT8__ 頁）：\n\n"
        "6. 圖1 改用同資料夾之 `fig1_architecture_compact.png`（同內容，高 5.3 cm；原圖 6.8 cm）。\n"
        "7. 刪圖2 與其圖說，並把 2.2 節首句「規劃流程如圖2。」改為「規劃流程如下。」（2.2 節其餘文字保留）。\n"
        "8. 3.4 節與表4 改為一句：「另一受控對照中，走廊豁免以場坪座標判定時最小水平間距降至 5.0 m（垂直 19.9 m；2 s 網格上界），以場坪身分判定則為 52.7 m 且無違規，"
        "而規劃驗證與覆蓋率兩者皆無法察覺（單一情境、單次執行；原始資料未納管）。」\n\n"
        "**不可刪**：摘要、表2、表3、圖3、第四節的「限制」、所有表註（表註是 caveat，刪了就是誤導）。"
        "備用圖4（`fig4_wind.png`）與附錄 E 之備用 3.6 節（表5）只有在 Word 實測仍不足 4 頁時才排入。"
    ),
))

# ---------------------------------------------------------------- abstract
E.append(dict(
    id="E07", serves="C1-a（刪「貼齊 120 m」）、C4-a＋決定 4、C7-a（用詞）、決定 1（1.2434 km²）、K1（100.43 m）、C2（主結果改為設風架次 HV4W55）、C1-f（間距 96.6 m）",
    evidence="NUMBERS.json trials.HV4W55（coverage 0.999666、flown_max_agl 119.6752、min_sep 100.4256、params n_ok 102/102）；metrics.json HV4W55.timing.per_uav（dist_err 3.813／4.675／3.736%）；NUMBERS.json ceiling_control（t_above_120 354.925 s）；survey_geometry（area_km2 1.243447、spacing 96.5907 m）",
    old=L(81),
    new=(
        "本文針對法規高度上限下之異質垂直起降（VTOL）無人機隊測繪任務，提出 HARVEST 規劃流程。在 120 m 作業上限下，"
        "小型複合翼 VTOL 之轉彎半徑（120–200 m）與條帶間距（約 96.6 m）同量級，且每架次皆含起降與場坪排隊之固定成本。"
        "HARVEST 結合考慮風場之飛行時間成本、依轉彎半徑與可用能量之決策級機數配額、Beam-IDP 條帶排序、起降走廊逐坪互斥與具時間不確定度之四維隔離產生任務，"
        "並匯出至 ArduPilot QuadPlane。以 1 架 Pioneer 與 2 架 HERO 覆蓋 1.2434 km² 作業區，於 5 m/s 模擬風場中進行 SITL 閉迴路飛行，"
        "三機皆完成起降，航跡回算覆蓋率 99.97%（上界），全程最高高度 119.68 m AGL，低於 120 m 上限（規劃天花板 119.5 m），"
        "航程較規劃多 3.7%–4.7%，最小水平間距 100.43 m（門檻 100 m），飛控參數讀回與韌體正規化後之期望值 102/102（34 項 × 3 機）相符。"
        "單變因對照顯示，規劃天花板若取 120.0 m，實飛累計 354.93 s 高於 120 m。"
        "另一受控對照顯示，走廊豁免若以場坪座標而非場坪身分判定，最小水平間距降至 5.0 m，而覆蓋率與規劃驗證均無法察覺。"
    ),
))

# ---------------------------------------------------------------- intro
E.append(dict(
    id="E08", serves="C1-f（刈幅／間距隨 119.5 m 天花板連動）",
    evidence="NUMBERS.json survey_geometry.swath_and_spacing['119.5'] = 137.9867／96.5907 m",
    old="本文取 120 m 為上限；此時相機刈幅約 138.6 m、條帶間距約 97.0 m，",
    new="本文取 120 m 為作業上限、119.5 m 為規劃天花板；於後者，相機刈幅約 138.0 m、條帶間距約 96.6 m，",
))
E.append(dict(
    id="E09", serves="決定 3（[4] 之 4000 m 未核對，刪數字）＋C7-a（用詞統一）",
    evidence="使用者決定；[4] 題名 “…Coverage 3D Path Planning Method for Complex Areas”（DOI 10.1016/j.dt.2025.01.008）；PAPER_TEXT_ZH.md C7-a",
    old="Wang 等人[4]則以轉彎半徑 4000 m 之同質固定翼機隊驗證協同覆蓋規劃。含風時間成本[5]、",
    new="Wang 等人[4]則以同質固定翼機隊驗證複雜區域之協同覆蓋規劃。考慮風場之飛行時間成本[5]、",
))
E.append(dict(
    id="E10", serves="C6-e、C7、決定 2（R_eff 為既有結果，引 [15]）",
    evidence="PAPER_TEXT_ZH.md C6-e；使用者決定 2",
    old="(i) 以各構型含風有效轉彎半徑計算作業時間，",
    new="(i) 以各構型之風修正有效轉彎半徑[15]與考慮風場之地速計算作業時間，",
))

# ---------------------------------------------------------------- 2.1
E.append(dict(
    id="E11", serves="C2-a（模擬風於解鎖前讀回）",
    evidence="runs/artifacts/HV4W55/wind_readback.json（15 列全數相符）",
    old="逐項讀回參數、比對任務項數，不符即停飛",
    new="逐項讀回參數與模擬風設定、比對任務項數，不符即停飛",
))

# ---------------------------------------------------------------- 2.3
E.append(dict(
    id="E12", serves="C1-f（h = 119.5 m）、C7-b（考慮風場之地速；直線與圓弧同式）",
    evidence="NUMBERS.json survey_geometry；hv_dccpp/wind.py:278-291（ground_speed）、:370-421（直線段與弧段呼叫同一式）",
    old=L(131),
    new=(
        "取 $h$ = 119.5 m（規劃天花板）、$\\alpha_H$ = 60°、$o_s$ = 0.30，得 $w$ = 138.0 m、$s$ = 96.6 m。"
        "時間成本以考慮風場之地速[5]與風修正有效轉彎半徑 $R_{eff}$ 之 Dubins 路徑[6]計算，直線段與圓弧段以同一地速式逐段積分："
    ),
))
E.append(dict(
    id="E13", serves="C6（式號順延：原式 (5) → (6)）",
    evidence="本檔 2.3 節",
    old=L(151),
    new=L(151).replace("\\tag{5}", "\\tag{6}"),
))
E.append(dict(
    id="E14", serves="C6（式號順延：原式 (4) → (5)）",
    evidence="本檔 2.3 節",
    old=L(142),
    new=L(142).replace("\\tag{4}", "\\tag{5}"),
))
E.append(dict(
    id="E15", serves="C7-c（完整風三角；空速型別）、C6-a（新增式 (4)）、C6-b＋決定 2（R_eff 為既有結果 [15]，本文之選擇為取最壞相位曲率）、C3-e（k 值與 W*）",
    evidence="hv_dccpp/wind.py:278-306（ground_speed 即完整風三角）；hv_dccpp/platform.py:296-304（r_eff、w_star）；hv_dccpp/mission.py r_eff_wind_policy = \"max_layer\"；PAPERPLAN1/c6_reff/reff_numbers.json（binding_branch_at_W['5'] = R_plan）；k = ⌈2·200/96.59⌉ = 5、⌈2·120/96.59⌉ = 3",
    old=L(140),
    new=(
        "$W_\\parallel$、$W_\\perp$ 為風速沿航跡之切向與法向分量（完整風三角），$\\delta$ 為弧段積分步長（≤5°）。"
        "有效轉彎半徑取定常風中轉彎地面軌跡（trochoid，次擺線[5,15]）之順風相位曲率半徑，並以規劃半徑為下限：\n\n"
        "$$R_{eff,c}(W)=\\max\\Big\\{R_{plan,c},\\ R_{theo,c}\\big(1+\\tfrac{W}{V_{a,c}}\\big)^2\\Big\\},\\qquad R_{theo,c}=\\frac{V_{a,c}^2}{g\\tan\\varphi_{max,c}},\\qquad W^*_c=V_{a,c}\\Big(\\sqrt{\\tfrac{R_{plan,c}}{R_{theo,c}}}-1\\Big) \\tag{4}$$\n\n"
        "Word 線性格式：\n"
        "```\n"
        "R_(eff,c)(W)=max{R_(plan,c), R_(theo,c)(1+W/V_(a,c))^2},  R_(theo,c)=V_(a,c)^2/(g tan φ_(max,c)),  W_c^*=V_(a,c)(√(R_(plan,c)/R_(theo,c))−1)\n"
        "```\n\n"
        "此為既有結果[15]，非本文之貢獻；本文僅取此最壞相位曲率半徑作為地面座標之單一 Dubins 半徑，使條帶排序與成本表全程留在地面座標。"
        "$W$ 為作業高度帶內之最大層風速，$W^*$ 為規劃半徑之內建裕度可吸收之風速。"
        "當 $s<2R_{eff}$ 時，跳帶數依 span coefficient $k=\\lceil 2R_{eff}/s\\rceil$ 決定[7,8]（本文情境 W = 5 m/s 低於 $W^*$，$R_{eff}=R_{plan}$，k 為 Pioneer 5、HERO 3）。"
        "各胞機數以決策級配額決定："
    ),
))
E.append(dict(
    id="E17", serves="C6-d（掛符號、新增 W* 列）、C1-f（刈幅／間距）、N3（R_theo 幾何自洽值）、C6-f（HERO 26.07° 只加表註）",
    evidence="TABLES_ZH.md 表 1；PAPERPLAN1/c6_reff/reff_numbers.json（W* 5.1675／5.0333、R_theo_formula 129.46／78.92 m）；PAPERPLAN1/c6_reff/ground_circle_bank.json（B_W5 bank_deg_on_R_eff 26.072°）；NUMBERS.json survey_geometry（137.9867／96.5907 m）",
    old=L(163, 171),
    new=(
        "| 參數 | Pioneer（A 構型） | HERO（B 構型） |\n"
        "|---|---|---|\n"
        "| 巡航空速 $V_a$ | 21.5 m/s | 19.0 m/s |\n"
        "| 轉彎半徑 $R_{theo}$／$R_{plan}$（最大傾角 $\\varphi_{max}$） | 130／200 m（20°） | 75／120 m（25°） |\n"
        "| 規劃裕度可吸收之風速 $W^*$（式 (4)） | 5.1675 m/s | 5.0333 m/s |\n"
        "| 續航／可用航程（含 30% 保留） | 176 min／133–145 km | 136 min／79–86 km |\n"
        "| 最低固定翼作業高度／空域上限 | 100 m／120 m AGL | 100 m／120 m AGL |\n"
        "| 相機（兩型共用） | HFOV 60°、側疊 30% ⇒ 於 119.5 m 刈幅 137.99 m、間距 96.59 m | 同左 |\n\n"
        "表註：規格為公開測試值與規劃值；電池能量、相機焦距（16 mm）與像素間距（3.9 μm）為假設值，續航與可用航程為估計值。"
        "$R_{theo}$ 為廠商值（幾何自洽值 129.46／78.92 m）；以表列值計，HERO 於 W = 5 m/s 之順風相位需傾角 26.07°，略大於 25°。"
    ),
))
