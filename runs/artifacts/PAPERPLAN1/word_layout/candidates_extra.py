"""內容層級的候選（會真的少掉東西，須使用者決定），以及候選套用順序。"""
from __future__ import annotations

import re

import paper_model as pm
from candidates import CANDIDATES, renum, set_para
from candidate_texts import LIM3_FOLD35, LIM3_ORIG, LIM_NEW

# ── K10 與 K8 的順序無關化：K10 若遇到已併入 3.5 的限制 (3)，保留併入版 ──
_orig_lim = CANDIDATES["K10"]["fn"]


def c_lim_safe(bl):
    k = [i for i, b in enumerate(bl) if b["kind"] == "para"
         and pm.text_of(b["runs"]).startswith("本研究之限制")]
    folded = "另設觸發情境" in pm.text_of(bl[k[0]]["runs"])
    text = LIM_NEW.replace(LIM3_ORIG, LIM3_FOLD35) if folded else LIM_NEW
    bl[k[0]]["runs"] = pm.inline(renum(text, bl))
    return bl


CANDIDATES["K10"]["fn"] = c_lim_safe

# ── K12：3.4 節（刪減 8 之句）併入結論，caveat 全帶過去 ──
CONC_WITH_34 = (
    "HARVEST 將考慮風場之飛行時間成本、跳帶迴轉、決策級機數配額、VTOL 起降能量與法規高度帶、起降走廊互斥與"
    "四維隔離整合於同一規劃流程，並匯出至 ArduPilot QuadPlane。5 m/s 模擬風場之 SITL 閉迴路飛行中三機完成起降、"
    "最高高度低於 120 m、參數讀回與韌體正規化後之期望值 102／102 相符；單變因對照顯示規劃天花板降 0.5 m 即可"
    "消除 354.93 s 之超高飛行，代價 +0.0279 s；納入風場換得的是預測準確度而非較短之完工時間。另一受控對照中，"
    "走廊豁免以場坪座標判定時最小水平間距降至 5.0 m（垂直 19.9 m；2 s 網格上界），以場坪身分判定則為 52.7 m "
    "且無違規，而規劃驗證與覆蓋率兩者皆無法察覺（單一情境、單次執行；原始資料未納管），顯示覆蓋率不足以作為"
    "唯一驗收條件。")


def c_34_into_conc(bl):
    ks = [k for k, b in enumerate(bl) if b.get("sec") == "3.4"]
    assert ks, "3.4 not found"
    for k in reversed(ks):
        del bl[k]
    kc = [i for i, b in enumerate(bl) if b["kind"] == "para"
          and pm.text_of(b["runs"]).startswith("HARVEST 將考慮風場之飛行時間成本、跳帶迴轉")]
    assert len(kc) == 1
    bl[kc[0]]["runs"] = pm.inline(renum(CONC_WITH_34, bl))
    # 3.5 若還在，改號為 3.4
    for b in bl:
        if b["kind"] == "h2" and pm.text_of(b["runs"]).startswith("3.5 "):
            b["runs"] = pm.inline("3.4 " + pm.text_of(b["runs"])[4:])
    return bl


# ── D1：刪圖1（2.1 節恢復全文，首句改「系統架構如下。」，圖號遞補）──
S21_FULL = None


def c_del_fig1(bl):
    orig = pm.parse()
    ko = pm.find(orig, kind="para", sec="2.1")
    full = pm.text_of(orig[ko[0]]["runs"])
    assert full.startswith("系統架構如圖1。")
    rest = pm.merge_runs(pm.slice_runs(orig[ko[0]]["runs"], len("系統架構如圖1。"), len(full)))
    kf = pm.one(bl, kind="fig", fignum="1")
    kc = pm.one(bl, kind="fcap", fignum="1")
    for k in sorted((kf, kc), reverse=True):
        del bl[k]
    kp = pm.find(bl, kind="para", sec="2.1")
    bl[kp[0]]["runs"] = pm.merge_runs(pm.inline("系統架構如下。") + rest)
    # 圖號：剩下的圖依序改為 1、2…
    figs = [b for b in bl if b["kind"] == "fig"]
    mapping = {}
    for i, b in enumerate(figs, start=1):
        mapping[b["fignum"]] = str(i)
    for b in bl:
        if b.get("fignum") in mapping:
            b["fignum"] = mapping[b["fignum"]]
    pm.map_text(bl, lambda t: re.sub(r"圖([1-9])", lambda m: "圖" + mapping.get(m.group(1), m.group(1)), t))
    return bl


# ── D2：式 (1)(2) 改寫為文字（內容保留、公式形式拿掉；其後式號遞補為 (1)–(4)）──
EQ12_PROSE = (
    "規劃目標為最小化機隊完工時間（makespan）$C_{\\max}=\\max_i C_i$，$C_i$ 為第 $i$ 機起飛、進場、作業、轉移、"
    "返場與降落時間之和。條帶刈幅 $w=2h\\tan(\\alpha_H/2)$、間距 $s=w(1-o_s)$，作業高度 "
    "$h_{op}=\\max(h_{\\min fw},\\min(h_{tgt},h_{max},h_{ceil}))$。")


def c_eq12_prose(bl):
    k1 = [i for i, b in enumerate(bl) if b["kind"] == "para"
          and pm.text_of(b["runs"]).startswith("規劃目標為最小化機隊完工時間")][0]
    ke1 = [i for i, b in enumerate(bl) if b["kind"] == "eq" and b["num"] == "(1)"][0]
    k2 = [i for i, b in enumerate(bl) if b["kind"] == "para"
          and pm.text_of(b["runs"]).startswith("條帶幾何由")][0]
    ke2 = [i for i, b in enumerate(bl) if b["kind"] == "eq" and b["num"] == "(2)"][0]
    assert k1 < ke1 < k2 < ke2
    for k in sorted((ke1, k2, ke2), reverse=True):
        del bl[k]
    bl[k1]["runs"] = pm.inline(EQ12_PROSE)
    ren = {"(3)": "(1)", "(4)": "(2)", "(5)": "(3)", "(6)": "(4)"}
    for b in bl:
        if b["kind"] == "eq":
            b["num"] = ren[b["num"]]
            b["eq_renum_from"] = [k for k, v in ren.items() if v == b["num"]][0]
    pm.map_text(bl, lambda t: re.sub(r"式 \(([3-6])\)", lambda m: f"式 ({int(m.group(1)) - 2})", t))
    return bl


# ── R2：期刊名依 ISO 4 縮寫 ──
ISO4 = {
    "Journal of Intelligent & Robotic Systems": "J. Intell. Robot. Syst.",
    "American Journal of Mathematics": "Am. J. Math.",
    "Biosystems Engineering": "Biosyst. Eng.",
    "Journal of Guidance, Control, and Dynamics": "J. Guid. Control Dyn.",
    "Defence Technology": "Def. Technol.",
    "Autonomous Robots": "Auton. Robots",
}


def c_iso4(bl):
    for b in bl:
        if b["kind"] == "ref":
            t = pm.text_of(b["runs"])
            for a, z in ISO4.items():
                t = t.replace(a, z)
            b["runs"] = [(t, {})]
    return bl


# ── L2：表註 10 點（brief 未規定表註字級；表格本身仍 11 點）──
def c_note10(bl):
    for b in bl:
        if b["kind"] == "note":
            b["size"] = 10
    return bl


CANDIDATES.update({
    "K12": {"fn": c_34_into_conc, "kind": "改寫", "title": "3.4 節併入結論（caveat 全帶過去）",
            "lost": "刪 3.4 小節標題；其數字與「單一情境、單次執行；原始資料未納管」移入結論段",
            "text": CONC_WITH_34},
    "L2": {"fn": c_note10, "kind": "版面", "title": "表註改 10 點（表格仍 11 點）",
           "lost": "無內容；brief 未規定表註字級，但與內文 11 點不同", "text": "（字級變更）"},
    "R2": {"fn": c_iso4, "kind": "內容", "title": "參考文獻期刊名改 ISO 4 縮寫",
           "lost": "期刊全名改縮寫（J. Intell. Robot. Syst. 等 6 種）", "text": "; ".join(f"{a} → {z}" for a, z in ISO4.items())},
    "D1": {"fn": c_del_fig1, "kind": "內容", "title": "刪圖1（2.1 節恢復全文，首句改「系統架構如下。」）",
           "lost": "系統架構圖（其內容 2.1 節文字已完整描述）；圖號遞補", "text": "（刪圖1與圖說；2.1 節改為「系統架構如下。」＋原 2.1 其餘文字）"},
    "D2": {"fn": c_eq12_prose, "kind": "內容", "title": "式(1)(2) 改為行內文字，其後式號遞補",
           "lost": "式 (1)(2) 之顯示方程式形式（定義改寫於行內，內容不變）", "text": EQ12_PROSE},
})

ORDER = ["T1", "K2", "K3", "D1", "K1", "D2", "K4", "L1", "F1", "K5", "K6", "K7", "L2", "F2",
         "K11", "K12", "K10", "K8", "K9", "X1", "R1", "R2"]


# ════════════════════════════════════════════════════════════════════
#  P 系列：刪非 caveat 之敘述（會少內容，但不動任何 caveat／表註／數字表）
# ════════════════════════════════════════════════════════════════════
P1_ABS = (
    "本文提出 HARVEST，針對 120 m 法規高度上限下之異質垂直起降（VTOL）無人機隊測繪（轉彎半徑 120–200 m 與條帶"
    "間距約 96.6 m 同量級，且每架次皆含起降與場坪排隊之固定成本），整合考慮風場之飛行時間成本、決策級機數配額、"
    "Beam-IDP 條帶排序、起降走廊逐坪互斥與具時間不確定度之四維隔離，並匯出至 ArduPilot QuadPlane。1 架 Pioneer "
    "與 2 架 HERO 於 5 m/s 模擬風場之 SITL 閉迴路飛行中皆完成起降，覆蓋率 99.97%（上界），最高 119.68 m AGL，"
    "低於 120 m 上限（規劃天花板 119.5 m），最小水平間距 100.43 m（門檻 100 m），參數讀回與韌體正規化後之期望值 "
    "102/102 相符。單變因對照中天花板取 120.0 m 則 SITL 飛行累計 354.93 s 高於 120 m；受控對照中走廊豁免以場坪"
    "座標而非身分判定時最小水平間距降至 5.0 m，而覆蓋率與規劃驗證均無法察覺。")

P2_INTRO = (
    "小型複合翼 VTOL 無人機兼具定點起降與長航程巡航，適合區域測繪；異質機隊之規劃須兼顧轉彎半徑、續航、起降場坪"
    "與飛航高度。依民用航空法第99條之14，遙控無人機未經核准時飛航高度不得逾距地面或水面四百呎[1]，本文取 120 m "
    "為作業上限、119.5 m 為規劃天花板。胞分解與來回掃描[2]、DARP 區域分派[3]、固定翼機隊協同覆蓋[4]、考慮風場之"
    "飛行時間成本[5]、曲率受限路徑[6]、迴轉與跳帶排序[7,8]、機數閉式界[9]、逐機轉彎半徑[10]、異質機隊多區覆蓋[11]、"
    "逐機起降點[12]與 QuadPlane 能量感知覆蓋[13]均已有文獻；本文不主張單一元件為新，而是提出 HARVEST"
    "（Heterogeneous Allocation and Routing for VTOL Endurance-constrained Survey Tasks）將其整合並輸出至 "
    "ArduPilot 飛控[14]，貢獻為：(i) 以考慮風場之地速計算各構型作業時間，並以決策級機數配額取代僅作為搜尋種子之"
    "機數界；(ii) 將 VTOL 起降能量、轉換高度與法規高度帶納入規劃；(iii) 以場坪身分為鍵之起降走廊互斥，搭配具時間"
    "不確定度之四維隔離。")

P4_CONC = (
    "HARVEST 將考慮風場之飛行時間成本、跳帶迴轉、決策級機數配額、VTOL 起降能量與法規高度帶、起降走廊互斥與四維"
    "隔離整合並匯出至 ArduPilot QuadPlane，並以 5 m/s 模擬風場之 SITL 閉迴路飛行與讀回檢核驗證其可執行性；規劃"
    "天花板降 0.5 m 即消除超高飛行（代價 +0.0279 s），納入風場換得的是預測準確度而非較短之完工時間，覆蓋率不足以"
    "作為唯一驗收條件。")
P4_CORRIDOR = (
    "另一受控對照中，走廊豁免以場坪座標判定時最小水平間距降至 5.0 m（垂直 19.9 m；2 s 網格上界），以場坪身分判定"
    "則為 52.7 m 且無違規，而規劃驗證與覆蓋率兩者皆無法察覺（單一情境、單次執行；原始資料未納管）。")

P5_33 = (
    "**規劃天花板。**匯出將圓弧重新離散、新頂點高度以線性內插，而等 AGL 曲面於局部東-北-天座標為拋物面，故迴轉"
    "頂點可高出天花板，且規劃驗證器（只檢查規劃航點、容差 1 m）無法察覺。HV4W55R 與 HV4W55 僅天花板不同（同風場、"
    "同匯出項數、參數檔逐位元相同）：120.0 m 時 88 個匯出航點高於 120 m，SITL 飛行最高 120.174 m、累計 354.93 s "
    "高於 120 m；119.5 m 時兩者皆為 0（表3），代價為規劃完工時間 +0.0279 s（本作業區之性質，不宜外推）。**風場。**"
    "運動學判準（東減西之地速 − 空速，理論 2W）與 EKF3 風速（4.89–4.97 m/s）證明模擬風生效（表3）。僅模擬風不同之 "
    "HV4W05 與 HV4W00 完工時間差 26.90 s；含風規劃之預測誤差較小（+1.70% 對 +2.43%），完工時間卻晚 47.13 s"
    "（44.0 s 為刻意延後放行），即買到的是預測準確度而非速度。180° 之 HV4W55S 以北−南判準與側風地速虧損（與 "
    "$\\sqrt{V_a^2-W^2}-V_a$ 相差 ≤ 0.041 m/s）驗證側向分量；兩風向為各自重解之計畫，不比較完工時間。")


def c_p1(bl):
    return set_para(bl, "本文", P1_ABS, kinds=("abs",))


def c_p2(bl):
    k1 = [i for i, b in enumerate(bl) if b["kind"] == "para" and b.get("sec") == "一"]
    assert len(k1) == 2
    del bl[k1[1]]
    bl[k1[0]]["runs"] = pm.inline(P2_INTRO)
    return bl


def c_p4(bl):
    kc = [i for i, b in enumerate(bl) if b["kind"] == "para"
          and pm.text_of(b["runs"]).startswith("HARVEST 將考慮風場之飛行時間成本")]
    assert len(kc) == 1
    merged34 = "另一受控對照中" in pm.text_of(bl[kc[0]]["runs"])
    bl[kc[0]]["runs"] = pm.inline(renum(P4_CONC + (P4_CORRIDOR if merged34 else ""), bl))
    return bl


def c_p5(bl):
    ks = pm.find(bl, kind="para", sec="3.3")
    if len(ks) == 2:
        del bl[ks[1]]
    bl[ks[0]]["runs"] = pm.inline(renum(P5_33, bl))
    return bl


def c_split_tables(bl):
    bl[0].setdefault("_layout", {})["table_split"] = True
    return bl


CANDIDATES.update({
    "P1": {"fn": c_p1, "kind": "內容", "title": "摘要縮為約 8 行",
           "lost": "摘要不再列 1.2434 km²、航程 +3.7%–4.7%、「（34 項 × 3 機）」、「依轉彎半徑與可用能量之」（正文皆有）",
           "text": P1_ABS},
    "P2": {"fn": c_p2, "kind": "內容", "title": "前言兩段併一段並縮短",
           "lost": "刈幅 138.0 m／間距 96.6 m 同量級一句（摘要與 2.2 節已有）、「本文並以 SITL…驗證」句、各文獻之描述詞縮短",
           "text": P2_INTRO},
    "P4": {"fn": c_p4, "kind": "內容", "title": "結論段縮為一句（數字留在摘要與正文）",
           "lost": "結論段不再重述 102／102、354.93 s（摘要與 3.2、3.3 節已有）", "text": P4_CONC},
    "P5": {"fn": c_p5, "kind": "內容", "title": "3.3 節再縮",
           "lost": "「飛行最高 119.675 m」（表1/表2 已有）、「270° 各趟 4.93–4.97」細分、若干連接詞", "text": P5_33},
    "L3": {"fn": c_split_tables, "kind": "版面", "title": "表格可跨頁（表頭重複）",
           "lost": "無內容；表格可能被分頁切開", "text": "（版面設定）"},
})
ORDER[:0] = ["P1", "P2"]
ORDER += ["P5", "P4", "L3"]


# ── F2 與 D1 相容：圖說沿用目前圖號 ──
def c_cap_keepnum(bl):
    ks = [k for k, b in enumerate(bl) if b["kind"] == "fcap"
          and "高度合規與 SITL 飛行間距" in pm.text_of(b["runs"])]
    assert len(ks) == 1
    n = re.match(r"圖(\d)", pm.text_of(bl[ks[0]]["runs"])).group(1)
    from candidate_texts import FIG3_CAP_FIX
    bl[ks[0]]["runs"] = pm.inline(FIG3_CAP_FIX.replace("圖3", f"圖{n}", 1))
    return bl


CANDIDATES["F2"]["fn"] = c_cap_keepnum


# ── L4：圖3 改排在 3.3 節文字之後、表3 之前（版面；內容與順序說明不變）──
def c_move_fig3(bl):
    kf = [k for k, b in enumerate(bl) if b["kind"] == "fig" and "fig5_compliance" in b["file"]]
    assert len(kf) == 1
    kf = kf[0]
    fig, cap = bl[kf], bl[kf + 1]
    assert cap["kind"] == "fcap"
    del bl[kf:kf + 2]
    kt = [k for k, b in enumerate(bl) if b["kind"] == "tcap" and "規劃天花板與風場" in pm.text_of(b["runs"])][0]
    bl[kt:kt] = [fig, cap]
    return bl


# ── E1：刪圖3（brief 列為不可刪；只用來量「要到 4 頁得付多少代價」）──
def c_del_fig3(bl):
    kf = [k for k, b in enumerate(bl) if b["kind"] == "fig" and "fig5_compliance" in b["file"]][0]
    assert bl[kf + 1]["kind"] == "fcap"
    del bl[kf:kf + 2]
    return bl


CANDIDATES.update({
    "L4": {"fn": c_move_fig3, "kind": "版面", "title": "圖3 改排在表3 之前（3.3 節文字之後）",
           "lost": "無內容；圖、表順序對調以減少頁底空白", "text": "（位置調整）"},
    "E1": {"fn": c_del_fig3, "kind": "不可刪", "title": "（僅供估價）刪圖3",
           "lost": "brief 0.4 列為不可刪；只用來量到 4 頁的代價", "text": "（刪圖3與圖說）"},
})
ORDER += ["L4", "E1"]


# ── P6：刪 2.2 節規劃流程文字（刪減 7 原本要保留；另一個內容決定）──
def c_del_flow(bl):
    ks = [k for k, b in enumerate(bl) if b["kind"] == "para"
          and pm.text_of(b["runs"]).startswith("規劃流程如下。")]
    assert len(ks) == 1
    del bl[ks[0]]
    kh = [k for k, b in enumerate(bl) if b["kind"] == "h2" and pm.text_of(b["runs"]).startswith("2.2 規劃流程")]
    for k in reversed(kh):
        del bl[k]
    for b in bl:
        if b["kind"] == "h2" and pm.text_of(b["runs"]).startswith("2.3 核心公式"):
            b["runs"] = pm.inline("2.2 核心公式")
    return bl


# ── P8：刪關鍵詞列（官方格式檔無此欄）──
def c_del_kw(bl):
    k = pm.one(bl, kind="keywords")
    del bl[k]
    return bl


CANDIDATES.update({
    "P6": {"fn": c_del_flow, "kind": "內容", "title": "刪 2.2 節規劃流程文字（圖2 已刪時）",
           "lost": "規劃流程之文字摘要（刪減 7 原要求保留）", "text": "（刪「規劃流程如下。通過前置檢查後…輸出任務檔。」整段）"},
    "P8": {"fn": c_del_kw, "kind": "內容", "title": "刪關鍵詞列",
           "lost": "關鍵詞（官方格式檔本無此欄）", "text": "（刪「關鍵詞：…」一行）"},
})
ORDER[:0] = ["P6", "P8"]


# ════════════════════════════════════════════════════════════════════
#  報告用組合（全部疊在 V1+6+7+8 之上）
# ════════════════════════════════════════════════════════════════════
_A = ["L1", "K1", "K2", "K3", "K4", "K5", "K6", "K7", "K8", "K9", "K10", "K11", "K12", "F1", "F2"]
_B = ["L1", "K1", "K4", "K5", "K7", "K8", "K10", "K11", "K12", "F1", "F2", "P1", "P2", "P4", "P5"]
REPORT_COMBOS = {
    "C1 全部零損改寫(A)": _A,
    "C2 A＋L2＋R1R2T1X1": _A + ["L2", "R1", "R2", "T1", "X1"],
    "C3 C2＋L3L4＋D1D2": _A + ["L2", "L3", "L4", "R1", "R2", "T1", "X1", "D1", "D2"],
    "C4 最大刪敘述(保留圖3)": _B + ["L2", "L3", "L4", "L5", "R1", "R2", "T1", "X1", "D1", "D2", "P6", "P8", "F3"],
    "C5 四頁(須刪圖1與圖3)": _B + ["L2", "L3", "R1", "R2", "D1", "D2", "E1"],
}


# ── K9 與 K12 相容：若 3.4 已併入結論，K9 保留走廊對照（含 caveat）──
from candidate_texts import CONC_NEW as _CONC_NEW  # noqa: E402

CONC_NEW_WITH_34 = (_CONC_NEW.split("；走廊判準對照顯示")[0] + "。" + P4_CORRIDOR[:-1] +
                    "，顯示覆蓋率不足以作為唯一驗收條件。")


def c_conc_safe(bl):
    kc = [i for i, b in enumerate(bl) if b["kind"] == "para"
          and pm.text_of(b["runs"]).startswith("HARVEST 將考慮風場之飛行時間成本")]
    assert len(kc) == 1
    merged34 = "另一受控對照中" in pm.text_of(bl[kc[0]]["runs"])
    bl[kc[0]]["runs"] = pm.inline(renum(CONC_NEW_WITH_34 if merged34 else _CONC_NEW, bl))
    return bl


CANDIDATES["K9"]["fn"] = c_conc_safe


# ── L5：取消孤行控制（允許段落首／末行單獨留在頁底／頁頂）──
def c_no_widow(bl):
    bl[0].setdefault("_layout", {})["no_widow"] = True
    return bl


CANDIDATES["L5"] = {"fn": c_no_widow, "kind": "版面", "title": "取消孤行控制",
                    "lost": "無內容；段落可能在頁底或頁頂只剩一行", "text": "（版面設定）"}
ORDER.append("L5")


import candidates_f3 as _f3  # noqa: E402
_f3.register(ORDER)
_f3.register_r3(ORDER)
