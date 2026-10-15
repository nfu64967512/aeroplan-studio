"""區塊模型 → WordprocessingML（Flat OPC）→ 插入官方格式檔副本 → .docx／.pdf，以 Word 量頁。

為何改走 XML：以 COM 逐段插字時，Word 會對每個 run 加 w:hint="eastAsia"，
使希臘字母、°、×、≤、−、² 等「共用字元」一律改用標楷體的全形字形（α、κ 變寬、
「km²」在 km 與 ² 之間斷行）。自行產生 run 的 rFonts（ascii／hAnsi＝Times New Roman、
eastAsia＝標楷體、不設 hint）才能讓英數與符號確實用 Times New Roman（0.1 節）。
表格也直接以 XML 產生：固定欄寬（依字寬量出，資料列盡量一行）、三線框、儲存格上下邊距 0。

方程式：先以私有 Word 把線性格式 BuildUp 成 OMML（含「#(n)」式號），快取後嵌入段落。
圖：先放佔位段落，插入後以 InlineShapes.AddPicture 原寸（100%）置入。
只用 DispatchEx 私有執行個體；只開自己的副本。
"""
from __future__ import annotations

import json
import os
import re
import time
from xml.sax.saxutils import escape

from fontTools.ttLib import TTFont

import paper_model as pm
from render_word import CM, WD_FORMAT_DOCX, WD_INFO_LINE, WD_INFO_PAGE, WD_INFO_VPOS, WD_PDF, \
    WD_STAT_PAGES, is_cjk

TNR_FONT = "Times New Roman"
KAI_FONT = "標楷體"
SYM_FONT = "Cambria"

_F = {k: TTFont(p) for k, p in {
    "r": "C:/Windows/Fonts/times.ttf", "b": "C:/Windows/Fonts/timesbd.ttf",
    "i": "C:/Windows/Fonts/timesi.ttf", "bi": "C:/Windows/Fonts/timesbi.ttf",
    "kai": "C:/Windows/Fonts/kaiu.ttf"}.items()}
_CMAP = {k: f.getBestCmap() for k, f in _F.items()}
_HMTX = {k: f["hmtx"].metrics for k, f in _F.items()}
_UPM = {k: f["head"].unitsPerEm for k, f in _F.items()}
TNR_SET = set(_CMAP["r"])
KAI_SET = set(_CMAP["kai"])

# 段落樣式：size pt、align（left/center/both）、before/after pt、首行縮排字數、凸排 pt、keepNext
STYLE = {
    "title":    dict(size=14, bold=True, align="center"),
    "blank":    dict(size=12, align="center"),
    "center12": dict(size=12, align="center"),
    "abs_head": dict(size=12, bold=True, align="center"),
    "abs":      dict(size=11, align="both", indent_chars=2),
    "keywords": dict(size=11, align="both"),
    "h1":       dict(size=12, bold=True, align="left", before=6, keep=True),
    "h2":       dict(size=11, bold=True, align="left", before=2, keep=True),
    "para":     dict(size=11, align="both", indent_chars=2),
    "eq":       dict(size=11, align="center"),
    "tcap":     dict(size=11, align="center", before=3, keep=True),
    "note":     dict(size=11, align="both", after=3),
    "fig":      dict(size=11, align="center", before=3, keep=True),
    "fcap":     dict(size=11, align="center", after=3),
    "ack":      dict(size=11, align="both", indent_chars=2),
    "ref":      dict(size=11, align="left", hanging=24),
}
# 版面預設依官方格式檔範例：段落無首行縮排、段前後距 0（格式檔的說明段落即如此）；
# 「indent」「spacing」為可選之加寬版面（首行縮排兩字、標題與圖表前後 2–6 pt），會多佔行數。
LAYOUT_DEFAULT = {"indent": False, "spacing": False, "compress_punct": False, "table_split": False,
                  "no_widow": False}
LAYOUT = dict(LAYOUT_DEFAULT)


# ────────────────────────────────────────────────────────────────────
#  字寬量測（決定表格欄寬）
# ────────────────────────────────────────────────────────────────────
def char_w(ch: str, size: float, bold=False, italic=False) -> float:
    o = ord(ch)
    if is_cjk(ch):
        k = "kai"
    else:
        k = ("bi" if bold and italic else "b" if bold else "i" if italic else "r")
        if o not in _CMAP[k]:
            k = "kai" if o in _CMAP["kai"] else None
    if k is None:
        return 0.6 * size
    g = _CMAP[k].get(o)
    if g is None:
        return 0.5 * size
    return _HMTX[k][g][0] / _UPM[k] * size


def runs_w(runs: list, size: float = 11) -> float:
    w = 0.0
    prev = None
    for t, f in runs:
        s = size * (0.65 if (f.get("sub") or f.get("sup")) else 1.0)
        for ch in t:
            w += char_w(ch, s, f.get("b", False), f.get("i", False))
            # Word 之中英自動間距（autoSpaceDE／DN）：CJK 與相鄰英數之間約加 0.25 em
            if prev is not None and prev != " " and ch != " " and (is_cjk(prev) != is_cjk(ch)):
                w += 0.25 * size
            prev = ch
    return w


def tokens_w(runs: list, size: float = 11) -> float:
    """最長不可斷字段（拉丁字串）之寬；CJK 字元之間可斷。"""
    best, cur = 0.0, 0.0
    for t, f in runs:
        s = size * (0.65 if (f.get("sub") or f.get("sup")) else 1.0)
        for ch in t:
            if ch == " " or is_cjk(ch):
                best = max(best, cur)
                cur = char_w(ch, s) if is_cjk(ch) else 0.0
            else:
                cur += char_w(ch, s, f.get("b", False), f.get("i", False))
    return max(best, cur)


# ────────────────────────────────────────────────────────────────────
#  XML 片段
# ────────────────────────────────────────────────────────────────────
def _slot(ch: str) -> str:
    if is_cjk(ch) or ch in " \u2003":
        return "n"
    o = ord(ch)
    if o in TNR_SET:
        return "n"
    if o in KAI_SET:
        return "ea"
    return "sym"


def run_xml(text: str, f: dict, size: float, bold_all=False) -> str:
    out = []
    # 依字元字型槽切段
    seg, cur = "", None
    for ch in text:
        s = _slot(ch)
        if cur is None or s == cur:
            seg += ch
            cur = s
        else:
            out.append((seg, cur))
            seg, cur = ch, s
    if seg:
        out.append((seg, cur))
    xs = []
    for seg, slot in out:
        if slot == "sym":
            fonts = f'<w:rFonts w:ascii="{SYM_FONT}" w:hAnsi="{SYM_FONT}" w:eastAsia="{KAI_FONT}" w:cs="{SYM_FONT}"/>'
        elif slot == "ea":
            fonts = f'<w:rFonts w:ascii="{TNR_FONT}" w:hAnsi="{TNR_FONT}" w:eastAsia="{KAI_FONT}" w:cs="{TNR_FONT}" w:hint="eastAsia"/>'
        else:
            fonts = f'<w:rFonts w:ascii="{TNR_FONT}" w:hAnsi="{TNR_FONT}" w:eastAsia="{KAI_FONT}" w:cs="{TNR_FONT}"/>'
        rp = fonts
        rp += "<w:b/><w:bCs/>" if (f.get("b") or bold_all) else "<w:b w:val=\"0\"/><w:bCs w:val=\"0\"/>"
        rp += "<w:i/><w:iCs/>" if f.get("i") else "<w:i w:val=\"0\"/><w:iCs w:val=\"0\"/>"
        hs = int(round(size * 2))
        rp += f'<w:sz w:val="{hs}"/><w:szCs w:val="{hs}"/>'
        if f.get("sub"):
            rp += '<w:vertAlign w:val="subscript"/>'
        elif f.get("sup"):
            rp += '<w:vertAlign w:val="superscript"/>'
        xs.append(f'<w:r><w:rPr>{rp}</w:rPr><w:t xml:space="preserve">{escape(seg)}</w:t></w:r>')
    return "".join(xs)


def ppr_xml(st: dict, keep_override=None) -> str:
    keep = st.get("keep", False) if keep_override is None else keep_override
    x = "<w:pPr>"
    if keep:
        x += "<w:keepNext/>"
    x += '<w:widowControl w:val="0"/>' if LAYOUT.get("no_widow") else "<w:widowControl/>"
    x += '<w:snapToGrid w:val="0"/>'
    before = st.get("before", 0) if LAYOUT["spacing"] else 0
    after = st.get("after", 0) if LAYOUT["spacing"] else 0
    x += f'<w:spacing w:before="{int(before * 20)}" w:after="{int(after * 20)}" w:line="240" w:lineRule="auto"/>'
    if st.get("hanging"):
        h = int(st["hanging"] * 20)
        x += f'<w:ind w:left="{h}" w:hanging="{h}"/>'
    elif st.get("indent_chars") and LAYOUT["indent"]:
        c = int(st["indent_chars"] * 100)
        x += f'<w:ind w:left="0" w:firstLineChars="{c}" w:firstLine="{int(st["indent_chars"] * st["size"] * 20)}"/>'
    else:
        x += '<w:ind w:left="0" w:firstLine="0"/>'
    x += f'<w:jc w:val="{st["align"]}"/>'
    hs = int(round(st["size"] * 2))
    x += f'<w:rPr><w:rFonts w:ascii="{TNR_FONT}" w:hAnsi="{TNR_FONT}" w:eastAsia="{KAI_FONT}"/><w:sz w:val="{hs}"/><w:szCs w:val="{hs}"/></w:rPr>'
    x += "</w:pPr>"
    return x


def p_xml(kind: str, runs: list, keep_override=None, st_over=None) -> str:
    st = dict(STYLE[kind])
    st.update(st_over or {})
    body = "".join(run_xml(t, f, st["size"], st.get("bold", False)) for t, f in runs if t)
    return f"<w:p>{ppr_xml(st, keep_override)}{body}</w:p>"


# ── 表格 ──
AVAIL_PT = 17.0 * CM
PAD_PT = 2.85                     # 儲存格左右邊距（0.1 cm）
SLACK_PT = 4.0


def _row_lines(nat_rows, w, pad):
    import math
    tot = 0
    for row in nat_rows:
        tot += max(max(1, math.ceil(x / max(wj - pad, 1.0) - 1e-9)) for x, wj in zip(row, w)) - 1
    return tot


def col_widths(rows: list) -> list:
    """欄寬：資料列能一行排下就一行；排不下時窮舉各欄候選寬度，使多出的行數最少。"""
    import itertools
    nc = len(rows[0])
    head, data = rows[0], rows[1:]
    pad = 2 * PAD_PT + SLACK_PT
    natm = [[runs_w(c) for c in r] for r in rows]
    nat = [max(natm[i][j] for i in range(1, len(rows))) for j in range(nc)]
    hnat = [natm[0][j] for j in range(nc)]
    hmin = [tokens_w(head[j]) for j in range(nc)]
    tmin = [max([tokens_w(r[j]) for r in data] + [hmin[j]]) for j in range(nc)]
    need = [max(nat[j], hmin[j]) + pad for j in range(nc)]
    need2 = [max(nat[j], hmin[j], hnat[j] / 2.0 + 3.0) + pad for j in range(nc)]   # 表頭至多兩行
    if sum(need2) <= AVAIL_PT:
        w = need2[:]
    elif sum(need) <= AVAIL_PT:
        w = need[:]
    else:
        cands = []
        for j in range(nc):
            c = sorted({natm[i][j] + pad for i in range(1, len(rows))} | {tmin[j] + pad}, reverse=True)
            c = [x for x in c if x >= tmin[j] + pad - 1e-9]
            cands.append(c)
        best = None
        for combo in itertools.product(*cands):
            if sum(combo) > AVAIL_PT:
                continue
            cost = _row_lines(natm[1:], combo, pad) * 10 + _row_lines([natm[0]], combo, pad)
            key = (cost, -min(a - b for a, b in zip(combo, [t + pad for t in tmin])))
            if best is None or key < best[0]:
                best = (key, list(combo))
        w = best[1]
    # 剩餘寬度：先讓表頭少斷行，再按比例分給各欄
    extra = AVAIL_PT - sum(w)
    want = [max(0.0, hnat[j] + pad - w[j]) for j in range(nc)]
    sw = sum(want)
    if sw > 0:
        give = min(extra, sw)
        w = [w[j] + give * want[j] / sw for j in range(nc)]
        extra -= give
    s_ = sum(w)
    w = [x + extra * x / s_ for x in w]
    return w


def table_xml(rows: list) -> str:
    nc, nr = len(rows[0]), len(rows)
    widths = col_widths(rows)
    tw = [int(round(x * 20)) for x in widths]
    tw[-1] += int(round(AVAIL_PT * 20)) - sum(tw)
    x = "<w:tbl><w:tblPr>"
    x += f'<w:tblW w:w="{sum(tw)}" w:type="dxa"/><w:jc w:val="center"/>'
    x += ('<w:tblBorders><w:top w:val="single" w:sz="12" w:space="0" w:color="000000"/>'
          '<w:left w:val="none" w:sz="0" w:space="0" w:color="auto"/><w:bottom w:val="single" w:sz="12" w:space="0" w:color="000000"/>'
          '<w:right w:val="none" w:sz="0" w:space="0" w:color="auto"/><w:insideH w:val="none" w:sz="0" w:space="0" w:color="auto"/>'
          '<w:insideV w:val="none" w:sz="0" w:space="0" w:color="auto"/></w:tblBorders>')
    x += '<w:tblLayout w:type="fixed"/>'
    m = int(round(PAD_PT * 20))
    x += (f'<w:tblCellMar><w:top w:w="0" w:type="dxa"/><w:left w:w="{m}" w:type="dxa"/>'
          f'<w:bottom w:w="0" w:type="dxa"/><w:right w:w="{m}" w:type="dxa"/></w:tblCellMar>')
    x += '<w:tblLook w:val="0000" w:firstRow="0" w:lastRow="0" w:firstColumn="0" w:lastColumn="0" w:noHBand="1" w:noVBand="1"/>'
    x += "</w:tblPr><w:tblGrid>" + "".join(f'<w:gridCol w:w="{w}"/>' for w in tw) + "</w:tblGrid>"
    st_c = dict(size=11, align="center")
    st_l = dict(size=11, align="left")
    for i, row in enumerate(rows):
        x += "<w:tr><w:trPr><w:cantSplit/>" + ("<w:tblHeader/>" if i == 0 else "") + "</w:trPr>"
        for j, cell in enumerate(row):
            x += f'<w:tc><w:tcPr><w:tcW w:w="{tw[j]}" w:type="dxa"/>'
            if i == 0:
                x += '<w:tcBorders><w:bottom w:val="single" w:sz="6" w:space="0" w:color="000000"/></w:tcBorders>'
            x += '<w:vAlign w:val="center"/></w:tcPr>'
            st = st_l if j == 0 else st_c
            # 整張表不拆頁：每列都與下一段同頁（最後一列連同表註首行）；
            # 版面選項 table_split 時只讓表頭與第一列同頁
            keep = True if not LAYOUT.get("table_split") else (i == 0)
            body = "".join(run_xml(t, f, 11) for t, f in cell if t)
            x += f"<w:p>{ppr_xml(st, keep_override=keep)}{body}</w:p></w:tc>"
        x += "</w:tr>"
    x += "</w:tbl>"
    return x, [round(v / CM, 2) for v in widths]


PKG_HEAD = ('<?xml version="1.0" standalone="yes"?><?mso-application progid="Word.Document"?>'
            '<pkg:package xmlns:pkg="http://schemas.microsoft.com/office/2006/xmlPackage">'
            '<pkg:part pkg:name="/_rels/.rels" pkg:contentType="application/vnd.openxmlformats-package.relationships+xml">'
            '<pkg:xmlData><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>'
            '</Relationships></pkg:xmlData></pkg:part>'
            '<pkg:part pkg:name="/word/document.xml" pkg:contentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml">'
            '<pkg:xmlData><w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" '
            'xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math"><w:body>')
PKG_TAIL = "</w:body></w:document></pkg:xmlData></pkg:part></pkg:package>"


# ────────────────────────────────────────────────────────────────────
#  方程式 OMML 快取
# ────────────────────────────────────────────────────────────────────
def eq_key(linear: str, num: str) -> str:
    k = next(k for k, v in pm.EQ_LINEAR.items() if v == linear)
    return f"{k}|{num}"


def build_eq_cache(word, base_docx: str, path: str, pairs=None) -> dict:
    """pairs: [(EQ_LINEAR 之鍵, "(n)")]；預設為原式號。"""
    if pairs is None:
        pairs = [(k, f"({k.rstrip('b')})") for k in pm.EQ_LINEAR]
    doc = word.Documents.Open(base_docx, ReadOnly=True, AddToRecentFiles=False, Visible=False)
    cache = {}
    try:
        for key, numtxt in pairs:
            lin = pm.EQ_LINEAR[key]
            num = f"{key}|{numtxt}"
            doc.Content.Delete()
            body = lin + "#" + numtxt
            doc.Range(0, 0).InsertAfter(body)
            r = doc.Range(0, len(body))
            r.Font.Size = 11
            doc.OMaths.Add(r)
            r.OMaths(1).BuildUp()
            x = doc.Paragraphs(1).Range.WordOpenXML
            a = x.index("<m:oMathPara")
            b = x.index("</m:oMathPara>") + len("</m:oMathPara>")
            assert x.count("<m:oMathPara") == 1
            omml = x[a:b]
            # 字級 11 pt（Word 內文樣式預設 14 pt）
            omml = re.sub(r"<w:sz w:val=\"\d+\"/>", "", omml)
            omml = re.sub(r"<w:szCs w:val=\"\d+\"/>", "", omml)
            omml = omml.replace("</w:rPr>", '<w:sz w:val="22"/><w:szCs w:val="22"/></w:rPr>')
            cache[num] = omml
    finally:
        doc.Close(False)
    if path:
        json.dump(cache, open(path, "w", encoding="utf-8"), ensure_ascii=False)
    return cache


# ────────────────────────────────────────────────────────────────────
#  組裝與量頁
# ────────────────────────────────────────────────────────────────────
def body_xml(blocks: list, eqs: dict) -> tuple:
    parts, figs, tinfo = [], [], []
    for b in blocks:
        k = b["kind"]
        if k == "blank":
            parts.append(p_xml("blank", [], st_over={"size": b.get("size", 12)}))
        elif k == "eq":
            st = STYLE["eq"]
            parts.append(f"<w:p>{ppr_xml(st)}{eqs[eq_key(b['linear'], b['num'])]}</w:p>")
        elif k == "fig":
            tag = f"@@FIG{len(figs)}@@"
            figs.append((tag, b["file"]))
            parts.append(p_xml("fig", [(tag, {})]))
        elif k == "table":
            x, w = table_xml(b["rows"])
            tinfo.append({"tnum": b.get("tnum"), "col_cm": w})
            parts.append(x)
        else:
            parts.append(p_xml(k, b["runs"], st_over=({"size": b["size"]} if b.get("size") else None)))
    return PKG_HEAD + "".join(parts) + PKG_TAIL, figs, tinfo


class XmlRenderer:
    def __init__(self, word, base_docx: str, figdir: str, eqs: dict):
        self.word, self.base, self.figdir, self.eqs = word, base_docx, figdir, eqs

    def build(self, blocks: list, out_docx: str, out_pdf: str | None, keep_open=False) -> dict:
        t0 = time.time()
        LAYOUT.clear()
        LAYOUT.update(LAYOUT_DEFAULT)
        LAYOUT.update(blocks[0].get("_layout", {}))
        need = [(eq_key(b["linear"], b["num"]).split("|")[0], b["num"]) for b in blocks
                if b["kind"] == "eq" and eq_key(b["linear"], b["num"]) not in self.eqs]
        if need:
            self.eqs.update(build_eq_cache(self.word, self.base, None, need))
        xml, figs, tinfo = body_xml(blocks, self.eqs)
        doc = self.word.Documents.Open(self.base, ReadOnly=True, AddToRecentFiles=False,
                                       Visible=False)
        self.doc = doc
        try:
            ps = doc.PageSetup
            geom0 = (ps.PageWidth, ps.PageHeight, ps.TopMargin, ps.BottomMargin,
                     ps.LeftMargin, ps.RightMargin)
            hdr0 = doc.Sections(1).Headers(1).Range.Text
            if LAYOUT["compress_punct"]:
                doc.JustificationMode = 1            # wdJustificationModeCompress（標點壓縮）
            doc.Content.Delete()
            doc.Range(0, 0).InsertXML(xml)
            # 格式檔留下的最後一個空段落：與前一段合併（先讓它與前一段同格式）
            n = doc.Paragraphs.Count
            last = doc.Paragraphs(n).Range
            if last.Text == "\r" and n > 1:
                prev = doc.Paragraphs(n - 1)
                last.ParagraphFormat = prev.Format
                last.Font.Size = prev.Range.Font.Size
                doc.Range(doc.Content.End - 2, doc.Content.End - 1).Delete()
            fig_info = []
            for tag, fname in figs:
                rng = doc.Content
                f = rng.Find
                f.ClearFormatting()
                ok = f.Execute(FindText=tag, MatchCase=True, Forward=True, Wrap=0)
                assert ok, tag
                rng.Text = ""
                shp = doc.InlineShapes.AddPicture(FileName=os.path.join(self.figdir, fname),
                                                  LinkToFile=False, SaveWithDocument=True,
                                                  Range=rng)
                fig_info.append({"file": fname, "width_cm": round(shp.Width / CM, 3),
                                 "height_cm": round(shp.Height / CM, 3),
                                 "scale_pct": [round(shp.ScaleWidth, 2), round(shp.ScaleHeight, 2)]})
            # 表格列尾標記（end-of-row）的段落屬性無法由 XML 指定，改以 COM 取消貼齊格線，
            # 並把列尾標記字級設為 11 pt（表內文字本來就全是 11 pt）
            for t in doc.Tables:
                for r in t.Rows:
                    r.Range.ParagraphFormat.DisableLineHeightGrid = True
                t.Range.Font.Size = 11
            # 空白行（題目／作者／摘要之間）：段落符號字級依格式檔為 12 pt
            for pi in range(1, min(doc.Paragraphs.Count, 12) + 1):
                pr = doc.Paragraphs(pi).Range
                if pr.Text == "\r" and not pr.Information(12):
                    pr.Font.Size = 12
            geom1 = (ps.PageWidth, ps.PageHeight, ps.TopMargin, ps.BottomMargin,
                     ps.LeftMargin, ps.RightMargin)
            assert geom0 == geom1, (geom0, geom1)
            assert doc.Sections(1).Headers(1).Range.Text == hdr0
            doc.Repaginate()
            m = measure(doc)
            m["figures"] = fig_info
            m["tables"] = tinfo
            m["n_omath"] = doc.OMaths.Count
            m["build_s"] = round(time.time() - t0, 1)
            m["layout"] = dict(LAYOUT)
            doc.SaveAs2(FileName=out_docx, FileFormat=WD_FORMAT_DOCX, AddToRecentFiles=False)
            if out_pdf:
                doc.ExportAsFixedFormat(OutputFileName=out_pdf, ExportFormat=WD_PDF,
                                        OpenAfterExport=False, OptimizeFor=0)
        finally:
            if not keep_open:
                doc.Close(False)
                self.doc = None
        return m


HEAD_RE = re.compile(r"^(一、|二、|三、|四、|2\.\d |3\.\d |誌謝$|參考文獻$|表\d |圖\d )")


def measure(doc) -> dict:
    pages = doc.ComputeStatistics(WD_STAT_PAGES)
    ps = doc.PageSetup
    top, bottom = ps.TopMargin, ps.PageHeight - ps.BottomMargin
    end = doc.Range(doc.Content.End - 1, doc.Content.End - 1)
    last_line_no = end.Information(WD_INFO_LINE)
    vpos = end.Information(WD_INFO_VPOS)
    # 最後一行底緣：用最後一段的單行行高（最後一段為參考文獻 11 pt）
    lastp = doc.Paragraphs(doc.Paragraphs.Count).Range
    nlines = max(1, lastp.ComputeStatistics(1))
    y_first = lastp.Information(WD_INFO_VPOS)
    lh = (vpos - y_first) / (nlines - 1) if nlines > 1 else 14.3
    used = vpos + lh - top
    where = []
    n_grid_on = 0
    n_rowend = 0
    for p in doc.Paragraphs:
        if not p.Format.DisableLineHeightGrid:
            if p.Range.Text == "\r\x07":      # 表格列尾標記：不是段落，Word 不接受設定
                n_rowend += 1
            else:
                n_grid_on += 1
        r = p.Range
        t = r.Text.strip("\r\x07 ")
        if r.InlineShapes.Count:
            where.append(("[圖檔]", r.Information(WD_INFO_PAGE), round(r.Information(WD_INFO_VPOS) / CM, 1)))
        elif HEAD_RE.match(t) and not r.Information(12):
            where.append((t[:12], r.Information(WD_INFO_PAGE), round(r.Information(WD_INFO_VPOS) / CM, 1)))
    # 每頁實際用到的高度（最後一行底緣 − 版心頂）與頁底空白；總內容高 = Σ 各頁用量
    per_page = []
    for pg in range(1, pages + 1):
        if pg < pages:
            nxt = doc.GoTo(What=1, Which=1, Count=pg + 1).Start
            lastc = doc.Range(max(nxt - 1, 0), max(nxt - 1, 0))
        else:
            lastc = end
        y = lastc.Information(WD_INFO_VPOS)
        h = 14.33
        if lastc.Paragraphs(1).Range.InlineShapes.Count:
            h = lastc.Paragraphs(1).Range.InlineShapes(1).Height
        per_page.append(round((y + h - top) / CM, 2))
    body_top_cm = None
    return {
        "pages": pages,
        "used_cm_per_page": per_page,
        "content_cm": round(sum(per_page), 2),
        "pages_equiv": round(sum(per_page) / ((bottom - top) / CM), 3),
        "last_page_lines": last_line_no,
        "last_line_top_cm": round(vpos / CM, 2),
        "last_page_used_cm": round(used / CM, 2),
        "body_height_cm": round((bottom - top) / CM, 2),
        "last_page_fill_pct": round(100 * used / (bottom - top), 1),
        "n_paragraphs": doc.Paragraphs.Count,
        "n_paragraphs_snap_to_grid_on": n_grid_on,
        "n_table_row_end_marks_reported_on": n_rowend,
        "where": where,
    }
