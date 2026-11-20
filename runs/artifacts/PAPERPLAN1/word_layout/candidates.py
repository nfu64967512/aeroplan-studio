"""在 V1+6+7+8 之上逐一套用的候選刪減（不刪任何 caveat 或表註）。

每個候選：
  text  —— 可直接貼回 cowork_brief.md 的替換文字（md 行內語法，表號／圖號用 md 原編號；
           套到已做刪減 2／7 的版本時，程式自動依表1 併入正文、刪圖2 後之遞補規則改號）
  lost  —— 會少掉什麼（誠實列出；「無」表示只是改寫或版面）
  kind  —— layout（純版面）／rewrite（同內容改寫）／dedup（重複處刪一處）／content（真的少內容）
"""
from __future__ import annotations

import re

import paper_model as pm


# ────────────────────────────────────────────────────────────────────
#  表號／圖號：候選文字以 md 原編號書寫，套用時依目前版本改號
# ────────────────────────────────────────────────────────────────────
def renum(md: str, bl: list) -> str:
    t1_is_table = any(b["kind"] == "tcap" and pm.text_of(b["runs"]).startswith("表1 兩型")
                      for b in bl)
    fig2_present = any(b["kind"] == "fig" and "fig2_flowchart" in b["file"] for b in bl)
    if not t1_is_table:
        md = re.sub(r"表([2-4])", lambda m: "表" + str(int(m.group(1)) - 1), md)
    if not fig2_present:
        md = md.replace("圖3", "圖2")
    return md


def set_para(bl: list, startswith: str, new_md: str, kinds=("para", "abs", "note")) -> list:
    ks = [k for k, b in enumerate(bl) if b["kind"] in kinds
          and pm.text_of(b["runs"]).startswith(renum(startswith, bl))]
    assert len(ks) == 1, (startswith, ks)
    bl[ks[0]]["runs"] = pm.inline(renum(new_md, bl))
    return bl


def replace_in_para(bl: list, startswith: str, old_md_plain: str, new_md: str) -> list:
    ks = [k for k, b in enumerate(bl) if b["kind"] in ("para",)
          and pm.text_of(b["runs"]).startswith(renum(startswith, bl))]
    assert len(ks) == 1, (startswith, ks)
    runs = bl[ks[0]]["runs"]
    old = pm.text_of(pm.inline(renum(old_md_plain, bl)))
    bl[ks[0]]["runs"] = pm.replace_text(runs, old, renum(new_md, bl))
    return bl


from candidate_texts import *  # noqa: F401,F403,E402
# ────────────────────────────────────────────────────────────────────
#  候選函式
# ────────────────────────────────────────────────────────────────────
def c_abs(bl):
    return set_para(bl, "本文針對法規高度上限下", ABS_NEW)


def c_intro(bl):
    set_para(bl, "小型複合翼垂直起降", INTRO1_NEW)
    return set_para(bl, "覆蓋路徑規劃以胞分解", INTRO2_NEW)


def c_merge22(bl):
    """2.2 併入 2.1：刪 2.2 標題、2.1 標題改「系統架構與規劃流程」、2.3 改為 2.2。"""
    if not pm.find(bl, kind="h2", startswith="2.2 規劃流程"):
        return bl          # P6 已刪 2.2 節
    k = pm.one(bl, kind="h2", startswith="2.2 規劃流程")
    del bl[k]
    k1 = pm.one(bl, kind="h2", startswith="2.1 系統架構")
    bl[k1]["runs"] = pm.inline("2.1 系統架構與規劃流程")
    k3 = pm.one(bl, kind="h2", startswith="2.3 核心公式")
    bl[k3]["runs"] = pm.inline("2.2 核心公式")
    for b in bl:
        if b.get("sec") == "2.2":
            b["sec"] = "2.1"
    return bl


def c_32(bl):
    return set_para(bl, "結果如表2", P32_NEW)


def c_33(bl):
    ks = pm.find(bl, kind="para", sec="3.3")
    assert len(ks) == 2
    del bl[ks[1]]
    bl[ks[0]]["runs"] = pm.inline(renum(P33_NEW, bl))
    return bl


def c_t3n(bl):
    return set_para(bl, "表註：(a) 風判準", T3N_NEW, kinds=("note",))


def c_35(bl):
    """3.5 節併入限制 (3)：刪 3.5 標題與段落。"""
    ks = [k for k, b in enumerate(bl) if b.get("sec") == "3.5"]
    for k in reversed(ks):
        del bl[k]
    k = [i for i, b in enumerate(bl) if b["kind"] == "para"
         and pm.text_of(b["runs"]).startswith("本研究之限制")]
    assert len(k) == 1
    old = pm.text_of(pm.inline(renum(LIM3_ORIG, bl)))
    bl[k[0]]["runs"] = pm.replace_text(bl[k[0]]["runs"], old, renum(LIM3_FOLD35, bl))
    return bl


def c_conc(bl):
    return set_para(bl, "HARVEST 將考慮風場之飛行時間成本、跳帶迴轉", CONC_NEW)


def c_lim(bl):
    return set_para(bl, "本研究之限制", LIM_NEW)


MONTHS = ("January|February|March|April|May|June|July|August|September|October|"
          "November|December")


def c_ref(bl):
    """參考文獻依官方格式檔範例只列年份（刪期刊月份；[1] 法規公布日與 [14] 存取日保留）。"""
    for b in bl:
        if b["kind"] == "ref" and not b["runs"][0][0].startswith(("[1]", "[14]")):
            b["runs"] = [(re.sub(rf", (?:{MONTHS}) (\d{{4}})\.$", r", \1.", t), f)
                         for t, f in b["runs"]]
    return bl


def c_ack(bl):
    """（僅在無計畫補助時）刪誌謝整節——md 自身寫明「若無計畫補助，整節刪除」。"""
    ks = [k for k, b in enumerate(bl) if b.get("sec") == "誌謝"]
    for k in reversed(ks):
        del bl[k]
    return bl


def c_eq6(bl):
    """式 (6) 由三行排成兩行（σ_t 定義與場坪互斥同一行）；式子內容不變。"""
    k = [i for i, b in enumerate(bl) if b["kind"] == "eq" and b["linear"] == pm.EQ_LINEAR["6"]]
    bl[k[0]]["linear"] = pm.EQ_LINEAR["6b"]
    return bl


# 純版面（由 render_xml.LAYOUT 旗標實作）
def c_punct(bl):
    bl[0].setdefault("_layout", {})["compress_punct"] = True
    return bl


def c_space(bl):
    bl[0].setdefault("_layout", {}).update({"indent": False, "spacing": False})
    return bl


def c_nospacing(bl):
    bl[0].setdefault("_layout", {}).update({"spacing": False})
    return bl


def c_noindent(bl):
    bl[0].setdefault("_layout", {}).update({"indent": False})
    return bl


def c_addstyle(bl):
    bl[0].setdefault("_layout", {}).update({"indent": True, "spacing": True})
    return bl


def c_23all(bl):
    # 若 D2 已把式 (1)(2) 改寫為文字，「條帶幾何由…」段已不存在
    if any(b["kind"] == "para" and pm.text_of(b["runs"]).startswith("條帶幾何由作業高度") for b in bl):
        set_para(bl, "條帶幾何由作業高度", P23A_NEW)
    set_para(bl, "取 ", P23B_NEW)
    return set_para(bl, "此式由 trochoid", P23C_NEW)


def c_34(bl):
    return set_para(bl, "另一受控對照中", S34_NEW)


def c_t1fix(bl):
    ks = [k for k, b in enumerate(bl) if b.get("from_table1")]
    assert len(ks) == 1
    bl[ks[0]]["runs"] = pm.inline(renum(T1_FIX, bl))
    return bl


def c_cap(bl):
    ks = [k for k, b in enumerate(bl) if b["kind"] == "fcap"
          and "高度合規與 SITL 飛行間距" in pm.text_of(b["runs"])]
    assert len(ks) == 1
    bl[ks[0]]["runs"] = pm.inline(renum(FIG3_CAP_FIX, bl))
    return bl


def c_title(bl):
    k = pm.one(bl, kind="title")
    bl[k]["runs"] = pm.inline(TITLE_ALT1)
    return bl


CANDIDATES = {
    "L1": {"fn": c_eq6, "kind": "版面", "title": "式(6) 由三行排成兩行",
           "lost": "無（同一式子；σ_t 定義與場坪互斥改排同一列）", "text": "（式子不變，只改排列）"},
    "K1": {"fn": c_merge22, "kind": "改寫", "title": "2.2 併入 2.1，省一個小節標題",
           "lost": "無內容；小節標題改「2.1 系統架構與規劃流程」，2.3 改號為 2.2",
           "text": "刪「2.2 規劃流程」標題；「2.1 系統架構」→「2.1 系統架構與規劃流程」；「2.3 核心公式」→「2.2 核心公式」"},
    "K2": {"fn": c_abs, "kind": "改寫", "title": "摘要精簡",
           "lost": "無數字、無 caveat；刪「任務」「產生任務」「場坪（身分）」等重複字，併句", "text": ABS_NEW},
    "K3": {"fn": c_intro, "kind": "改寫", "title": "前言兩段精簡（相關研究段改為列舉句）",
           "lost": "不再寫出「Wang 等人」與「覆蓋路徑規劃以…為基礎」之句首，「相機刈幅」→「刈幅」；引用 [1]–[14] 全保留",
           "text": INTRO1_NEW + "\n\n" + INTRO2_NEW},
    "K4": {"fn": c_23all, "kind": "改寫", "title": "2.3 三段說明文字精簡",
           "lost": "無（「作業高度」第二次出現改用 h；「直線段與圓弧段」→「直線與圓弧」；刪「當」「之」）",
           "text": P23A_NEW + "\n\n" + P23B_NEW + "\n\n" + P23C_NEW},
    "K5": {"fn": c_32, "kind": "去重複", "title": "3.2 刪與表2／表3 重複之句",
           "lost": "刪「三機完成起降，全程最高 119.675 m AGL、高於 120 m 之時間 0 s」——表2 起降列與高度列、表3 HV4W55 列、3.3 節皆已寫",
           "text": P32_NEW},
    "K6": {"fn": c_33, "kind": "改寫", "title": "3.3 兩段併為一段並精簡",
           "lost": "無數字、無 caveat；「規劃器各自重解」→「各自重解」；「高於 120 m 之匯出航點與飛行時間皆為 0」→「兩者皆為 0」",
           "text": P33_NEW},
    "K7": {"fn": c_t3n, "kind": "改寫", "title": "表3 表註同內容精簡",
           "lost": "無；(d) 改寫為「HV4SITL15／16（未列）」以配合刪減 5 已刪 HV4SITL16 列", "text": T3N_NEW},
    "K8": {"fn": c_35, "kind": "改寫", "title": "3.5 節併入限制 (3)",
           "lost": "刪 3.5 小節標題；3.5 節之句全部移入限制 (3)，「均為規劃層模型值」保留", "text": LIM3_FOLD35},
    "K9": {"fn": c_conc, "kind": "改寫", "title": "結論段精簡",
           "lost": "刪連接語「單變因對照顯示」「則」「即可…之」；數字全保留", "text": CONC_NEW},
    "K10": {"fn": c_lim, "kind": "改寫", "title": "限制 8 項併為 5 項",
            "lost": "無 caveat：(5)(6)(8) 併入 (1)，(7) 改號 (5)；(2)(3) 編號不變（3.2 節「見限制 (2)」仍對）",
            "text": LIM_NEW},
    "K11": {"fn": c_34, "kind": "改寫", "title": "3.4 節（刪減 8 之句）精簡",
            "lost": "刪「兩者」「而」「且」", "text": S34_NEW},
    "F1": {"fn": c_t1fix, "kind": "修正", "title": "刪減 2 之句補回 R_theo 與最大傾角（修懸空指涉）",
           "lost": "不是刪減：刪減 2 讓表註「以表列值計…略大於 25°」失去所指，本條補回 130／75 m 與 20°／25°",
           "text": T1_FIX},
    "F2": {"fn": c_cap, "kind": "修正", "title": "圖3 圖說配合重產之圖（HV4W55R 已在圖內）",
           "lost": "不是刪減：原圖說寫「單變因對照 HV4W55R 見表3」，重產後 (a) 已並列 HV4W55R／HV4W55",
           "text": FIG3_CAP_FIX},
    "R1": {"fn": c_ref, "kind": "內容", "title": "參考文獻依格式檔範例只列年份",
           "lost": "期刊出版月份（[2]–[8]、[11]–[13]）；[1] 公布日與 [14] 存取日保留",
           "text": "例：[2] …, Vol. 9, No. 3, pp 247-253, 2000.（原 December 2000）"},
    "T1": {"fn": c_title, "kind": "內容", "title": "改用 md 之備選題目一（題目排成一行）",
           "lost": "題目不含「HARVEST」與「SITL驗證」——須使用者決定", "text": TITLE_ALT1},
    "X1": {"fn": c_ack, "kind": "內容", "title": "刪誌謝（僅限無計畫補助時）",
           "lost": "誌謝整節——須使用者決定（md 自註「若無計畫補助，整節刪除」）", "text": "（整節刪除）"},
}

# 資訊用（不是刪減）：加回首行縮排兩字與標題／圖表前後距，會多幾行
INFO = {"S+": {"fn": c_addstyle, "title": "加回首行縮排兩字與段前後距（2–6 pt）"}}

COMBOS: dict = {}
EXTRA: dict = {}
