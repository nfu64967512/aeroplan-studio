"""區塊模型 → Word（官方格式檔副本）→ .docx／.pdf，並以 Word 自身量頁。

只用 DispatchEx 開的**私有** Word（Visible=False、DisplayAlerts=0）；只開自己複製的檔、
只關自己的檔、只 Quit 自己的執行個體。不碰使用者已開啟的 Word。

排版規則（cowork_brief.md 0.1–0.3）：
  * A4、邊界 2 cm、頁首頁尾沿用格式檔（清空內文、不動節屬性）。
  * 中文標楷體、英數 Times New Roman；內文 11 pt、單行間距。
  * 每一段（含表格儲存格、圖、圖說、參考文獻）DisableLineHeightGrid = True（取消貼齊格線）。
  * 題目 14 pt 粗體置中；作者／單位 12 pt 置中；「摘要」「誌謝」「參考文獻」12 pt 粗體。
  * 章標題 12 pt 粗體靠左、小節 11 pt 粗體。
  * 數學式：Word 內建方程式（OMath，由線性格式 BuildUp），式號以「#(n)」靠右。
  * 表：標題在上置中；11 pt、單行、儲存格上下邊距 0；三線表（上、表頭下、下）。
  * 圖：inline、置中、原寸 17.0 cm（不縮放）；圖說在下置中。
"""
from __future__ import annotations

import os
import time
import unicodedata

import pythoncom
import win32com.client
from fontTools.ttLib import TTFont

import paper_model as pm

TNR_CMAP = set(TTFont("C:/Windows/Fonts/times.ttf").getBestCmap())
KAI_CMAP = set(TTFont("C:/Windows/Fonts/kaiu.ttf").getBestCmap())
SYMBOL_FONT = "Cambria"          # 行內 ⌈⌉⇒∧∥⊥ 等 TNR 沒有的符號（Cambria Math 行高 5.58 em，不能用）

CM = 28.3464567
WD_FORMAT_DOCX = 16
WD_PDF = 17
WD_STAT_PAGES = 2
WD_INFO_VPOS = 6
WD_INFO_LINE = 10
WD_INFO_PAGE = 3


def is_cjk(ch: str) -> bool:
    o = ord(ch)
    return (0x2E80 <= o <= 0x9FFF or 0xF900 <= o <= 0xFAFF or 0xFE30 <= o <= 0xFE4F
            or 0xFF00 <= o <= 0xFFEF or 0x3000 <= o <= 0x303F)


def needs_symbol_font(ch: str) -> bool:
    return (ord(ch) not in TNR_CMAP) and not is_cjk(ch) and ch not in "\r\n\t\u2003"


# ────────────────────────────────────────────────────────────────────
#  段落樣式表（單位 pt）
# ────────────────────────────────────────────────────────────────────
STYLE = {
    "title":    dict(size=14, bold=True, align=1),
    "blank":    dict(size=12, align=1),
    "center12": dict(size=12, align=1),
    "abs_head": dict(size=12, bold=True, align=1),
    "abs":      dict(size=11, align=3, indent_chars=2),
    "keywords": dict(size=11, align=3),
    "h1":       dict(size=12, bold=True, align=0, before=6, keep=True),
    "h2":       dict(size=11, bold=True, align=0, before=2, keep=True),
    "para":     dict(size=11, align=3, indent_chars=2),
    "eq":       dict(size=11, align=1),
    "tcap":     dict(size=11, align=1, before=3, keep=True),
    "note":     dict(size=11, align=3, after=3),
    "fig":      dict(size=11, align=1, keep=True, before=3),
    "fcap":     dict(size=11, align=1, after=3),
    "ack":      dict(size=11, align=3, indent_chars=2),
    "ref":      dict(size=11, align=3, hanging=24),
}


class WordSession:
    """私有 Word 執行個體（with 區塊結束一定 Quit 自己）。"""

    def __enter__(self):
        pythoncom.CoInitialize()
        self.word = win32com.client.DispatchEx("Word.Application")
        self.word.Visible = False
        self.word.DisplayAlerts = 0
        self.word.Options.CheckGrammarAsYouType = False
        self.word.Options.CheckSpellingAsYouType = False
        return self

    def __exit__(self, *exc):
        try:
            for d in list(self.word.Documents):
                d.Close(False)          # 本執行個體只開過我們自己的檔
        finally:
            self.word.Quit()
            pythoncom.CoUninitialize()


class Renderer:
    def __init__(self, word, template_copy: str, figdir: str):
        self.word = word
        self.template = template_copy
        self.figdir = figdir

    # ── 基本插入 ──
    def _end(self):
        return self.doc.Content.End - 1

    def _para_format(self, rng, st: dict):
        pf = rng.ParagraphFormat
        pf.Alignment = st.get("align", 3)
        pf.LineSpacingRule = 0                      # wdLineSpaceSingle
        pf.SpaceBefore = st.get("before", 0)
        pf.SpaceAfter = st.get("after", 0)
        pf.DisableLineHeightGrid = True             # 取消「文件格線被設定時，貼齊格線」
        pf.KeepWithNext = bool(st.get("keep", False))
        pf.WidowControl = True
        pf.CharacterUnitLeftIndent = 0
        pf.CharacterUnitFirstLineIndent = 0
        pf.LeftIndent = 0
        pf.RightIndent = 0
        pf.FirstLineIndent = 0
        if st.get("hanging"):
            pf.LeftIndent = st["hanging"]
            pf.FirstLineIndent = -st["hanging"]
        if st.get("indent_chars"):
            pf.CharacterUnitFirstLineIndent = st["indent_chars"]

    def _base_font(self, rng, size, bold=False):
        f = rng.Font
        f.Name = "Times New Roman"
        f.NameAscii = "Times New Roman"
        f.NameOther = "Times New Roman"
        f.NameFarEast = "標楷體"
        f.Size = size
        f.Bold = bool(bold)
        f.Italic = False
        f.Subscript = False
        f.Superscript = False

    def _apply_runs(self, base: int, runs: list):
        off = 0
        for text, f in runs:
            n = len(text)
            if n and (f.get("b") or f.get("i") or f.get("sub") or f.get("sup")):
                r = self.doc.Range(base + off, base + off + n)
                if f.get("b"):
                    r.Font.Bold = True
                if f.get("i"):
                    r.Font.Italic = True
                if f.get("sub"):
                    r.Font.Subscript = True
                if f.get("sup"):
                    r.Font.Superscript = True
            for k, ch in enumerate(text):
                if needs_symbol_font(ch):
                    r = self.doc.Range(base + off + k, base + off + k + 1)
                    r.Font.Name = SYMBOL_FONT
                    self.symbol_chars.add(ch)
            off += n

    def add_para(self, kind: str, runs: list, st_over: dict | None = None):
        st = dict(STYLE[kind])
        st.update(st_over or {})
        text = pm.text_of(runs)
        start = self._end()
        self.doc.Range(start, start).InsertAfter(text + "\r")
        rng = self.doc.Range(start, start + len(text) + 1)
        self._para_format(rng, st)
        self._base_font(rng, st["size"], st.get("bold"))
        self._apply_runs(start, runs)
        self.last_style = st
        return rng

    def add_eq(self, linear: str, num: str):
        st = STYLE["eq"]
        start = self._end()
        body = linear + "#" + num
        self.doc.Range(start, start).InsertAfter(body + "\r")
        rng = self.doc.Range(start, start + len(body) + 1)
        self._para_format(rng, st)
        self._base_font(rng, st["size"])
        r = self.doc.Range(start, start + len(body))
        self.doc.OMaths.Add(r)
        r.OMaths(1).BuildUp()
        # BuildUp 會把其後的空段落併進方程式段落；補回一個方程式外的空段落供後續插入
        if self.doc.Range(self._end(), self._end()).OMaths.Count:
            self.doc.Content.InsertParagraphAfter()
            tail = self.doc.Paragraphs(self.doc.Paragraphs.Count).Range
            assert tail.OMaths.Count == 0
        self.last_style = st

    def add_fig(self, fname: str):
        st = STYLE["fig"]
        start = self._end()
        self.doc.Range(start, start).InsertAfter("\r")
        rng = self.doc.Range(start, start + 1)
        self._para_format(rng, st)
        self._base_font(rng, st["size"])
        path = os.path.join(self.figdir, fname)
        shp = self.doc.InlineShapes.AddPicture(FileName=path, LinkToFile=False,
                                               SaveWithDocument=True,
                                               Range=self.doc.Range(start, start))
        self.figs.append({"file": fname, "width_cm": shp.Width / CM,
                          "height_cm": shp.Height / CM,
                          "scale_w": shp.ScaleWidth, "scale_h": shp.ScaleHeight})
        self.last_style = st

    def add_table(self, rows: list):
        start = self._end()
        # 表格建在目前最後一個（空）段落上
        rng = self.doc.Range(start, start)
        nr, nc = len(rows), len(rows[0])
        tbl = self.doc.Tables.Add(rng, nr, nc)
        tbl.Borders.Enable = False
        tbl.TopPadding = 0
        tbl.BottomPadding = 0
        tbl.LeftPadding = 2.85
        tbl.RightPadding = 2.85
        tbl.Spacing = 0
        tr = tbl.Range
        self._base_font(tr, 11)
        pf = tr.ParagraphFormat
        pf.LineSpacingRule = 0
        pf.SpaceBefore = 0
        pf.SpaceAfter = 0
        pf.DisableLineHeightGrid = True
        pf.CharacterUnitFirstLineIndent = 0
        pf.CharacterUnitLeftIndent = 0
        pf.FirstLineIndent = 0
        pf.LeftIndent = 0
        pf.Alignment = 1
        pf.KeepWithNext = False
        for i, row in enumerate(rows):
            for j, cell_runs in enumerate(row):
                c = tbl.Cell(i + 1, j + 1)
                c.Range.Text = pm.text_of(cell_runs)
                self._apply_runs(c.Range.Start, cell_runs)
                if j == 0:
                    c.Range.ParagraphFormat.Alignment = 0
        tbl.Range.Cells.VerticalAlignment = 1
        # 三線表：上框、表頭下框、下框（1.5／0.75／1.5 pt），無直線
        tbl.Rows(1).Borders(-1).LineStyle = 1
        tbl.Rows(1).Borders(-1).LineWidth = 12
        tbl.Rows(1).Borders(-3).LineStyle = 1
        tbl.Rows(1).Borders(-3).LineWidth = 6
        tbl.Rows(nr).Borders(-3).LineStyle = 1
        tbl.Rows(nr).Borders(-3).LineWidth = 12
        tbl.Rows(1).HeadingFormat = True
        # 表頭與第一列資料不拆頁
        for i in range(1, min(nr, 2) + 1):
            for p in tbl.Rows(i).Range.Paragraphs:
                p.KeepWithNext = True
        tbl.AllowAutoFit = True
        tbl.AutoFitBehavior(1)          # 依內容
        tbl.AutoFitBehavior(2)          # 再撐滿版心
        tbl.Rows.Alignment = 1
        self.tables.append({"rows": nr, "cols": nc})

    # ── 主流程 ──
    def build(self, blocks: list, out_docx: str, out_pdf: str | None) -> dict:
        t0 = time.time()
        self.figs, self.tables, self.symbol_chars = [], [], set()
        self.doc = self.word.Documents.Open(self.template, ReadOnly=True,
                                            AddToRecentFiles=False, Visible=False)
        doc = self.doc
        try:
            doc.Content.Delete()
            assert doc.Tables.Count == 0 and doc.InlineShapes.Count == 0
            for b in blocks:
                k = b["kind"]
                if k == "blank":
                    self.add_para("blank", [("", {})], {"size": b.get("size", 12)})
                elif k == "eq":
                    self.add_eq(b["linear"], b["num"])
                elif k == "fig":
                    self.add_fig(b["file"])
                elif k == "table":
                    self.add_table(b["rows"])
                else:
                    self.add_para(k, b["runs"])
            # 刪掉最後的空段落：先把它格式化成與最後一段相同，再併掉前一個段落符號
            last = doc.Paragraphs(doc.Paragraphs.Count).Range
            if last.Text == "\r":
                self._para_format(last, self.last_style)
                self._base_font(last, self.last_style["size"])
                doc.Range(doc.Content.End - 2, doc.Content.End - 1).Delete()
            doc.Repaginate()
            m = self.measure()
            m["build_s"] = round(time.time() - t0, 1)
            m["figures"] = self.figs
            m["symbol_font_chars"] = "".join(sorted(self.symbol_chars))
            doc.SaveAs2(FileName=out_docx, FileFormat=WD_FORMAT_DOCX, AddToRecentFiles=False)
            if out_pdf:
                doc.ExportAsFixedFormat(OutputFileName=out_pdf, ExportFormat=WD_PDF,
                                        OpenAfterExport=False, OptimizeFor=0)
        finally:
            doc.Close(False)
            self.doc = None
        return m

    def measure(self) -> dict:
        doc = self.doc
        pages = doc.ComputeStatistics(WD_STAT_PAGES)
        end = doc.Range(doc.Content.End - 1, doc.Content.End - 1)
        ps = doc.PageSetup
        top, bottom = ps.TopMargin, ps.PageHeight - ps.BottomMargin
        # 最後一頁：逐行（Word 行號）與最後一行底緣位置
        last_line_no = end.Information(WD_INFO_LINE)
        vpos = end.Information(WD_INFO_VPOS)
        # 最後一行的行高：以最後一段的行距近似（參考文獻 11 pt 單行）
        lh = self._line_height_at_end()
        used = vpos + lh - top
        # 各章節／圖表起始頁
        where = []
        for p in doc.Paragraphs:
            r = p.Range
            t = r.Text.strip()
            if not t and r.InlineShapes.Count:
                where.append(("圖", r.Information(WD_INFO_PAGE), round(r.Information(WD_INFO_VPOS) / CM, 2)))
                continue
            if t[:2] in ("一、", "二、", "三、", "四、") or t[:3] in ("2.1", "2.2", "2.3", "3.1", "3.2", "3.3", "3.4", "3.5") or t in ("誌謝", "參考文獻") or (t.startswith("表") and t[1:2].isdigit() and " " in t[:4]) or (t.startswith("圖") and t[1:2].isdigit()):
                where.append((t[:14], r.Information(WD_INFO_PAGE), round(r.Information(WD_INFO_VPOS) / CM, 2)))
        # 檢核：所有段落都已取消貼齊格線
        n_par = doc.Paragraphs.Count
        n_grid = sum(1 for p in doc.Paragraphs if not p.Format.DisableLineHeightGrid)
        return {
            "pages": pages,
            "last_page_lines": last_line_no,
            "last_line_top_cm": round(vpos / CM, 2),
            "last_page_used_cm": round(used / CM, 2),
            "body_height_cm": round((bottom - top) / CM, 2),
            "last_page_fill_pct": round(100 * used / (bottom - top), 1),
            "n_paragraphs": n_par,
            "n_paragraphs_snap_to_grid_on": n_grid,
            "where": where,
        }

    def _line_height_at_end(self) -> float:
        # 11 pt Times New Roman／標楷體混排之單行行高（由 Word 行距量得，見 measure_line_height）
        return getattr(self, "line_h", 14.0)


def measure_line_height(word, template_copy: str) -> float:
    """在格式檔副本上量 11 pt 中英混排單行（取消貼齊格線）之行高。"""
    doc = word.Documents.Open(template_copy, ReadOnly=True, AddToRecentFiles=False,
                              Visible=False)
    try:
        doc.Content.Delete()
        txt = ("測試Test行高 123 " * 30).strip()
        r = doc.Range(0, 0)
        r.InsertAfter(txt + "\r" + txt + "\r")
        for p in doc.Paragraphs:
            p.Format.DisableLineHeightGrid = True
            p.Format.LineSpacingRule = 0
            p.Format.SpaceBefore = 0
            p.Format.SpaceAfter = 0
            p.Range.Font.Name = "Times New Roman"
            p.Range.Font.NameFarEast = "標楷體"
            p.Range.Font.Size = 11
        doc.Repaginate()
        a = doc.Paragraphs(1).Range
        y0 = a.Information(WD_INFO_VPOS)
        b = doc.Paragraphs(2).Range
        y1 = b.Information(WD_INFO_VPOS)
        lines = a.ComputeStatistics(1)       # wdStatisticLines
        return (y1 - y0) / max(lines, 1)
    finally:
        doc.Close(False)


def make_docx_base(word, template_copy: str, out_docx: str) -> dict:
    """格式檔副本（.doc）→ .docx（Word 2013+ 相容模式，方程式才可編輯）。內文原樣保留。"""
    doc = word.Documents.Open(template_copy, ReadOnly=True, AddToRecentFiles=False,
                              Visible=False)
    try:
        info = {"compat_before": doc.CompatibilityMode}
        doc.SaveAs2(FileName=out_docx, FileFormat=WD_FORMAT_DOCX, AddToRecentFiles=False)
        doc.SetCompatibilityMode(15)
        # 內文樣式本身也取消貼齊格線：表格列尾標記（無法逐段設定）因而也不貼齊
        doc.Styles(-1).ParagraphFormat.DisableLineHeightGrid = True
        doc.Save()
        info["compat_after"] = doc.CompatibilityMode
        return info
    finally:
        doc.Close(False)
