"""以私有 Word 開交付檔，檢核排版規則（頁數、字型、字級、貼齊格線、圖寬、三線表、方程式）。"""
import sys, os, collections
sys.stdout.reconfigure(encoding="utf-8")
from render_word import WordSession, CM

path = os.path.abspath(sys.argv[1])
with WordSession() as ws:
    doc = ws.word.Documents.Open(path, ReadOnly=True, AddToRecentFiles=False, Visible=False)
    try:
        doc.Repaginate()
        ps = doc.PageSetup
        print("pages", doc.ComputeStatistics(2), "| A4", round(ps.PageWidth / CM, 2), "x", round(ps.PageHeight / CM, 2),
              "| margins cm", [round(x / CM, 2) for x in (ps.TopMargin, ps.BottomMargin, ps.LeftMargin, ps.RightMargin)])
        print("header:", repr(doc.Sections(1).Headers(1).Range.Text[:40]), "| footer fields", doc.Sections(1).Footers(1).Range.Fields.Count,
              "align", doc.Sections(1).Footers(1).Range.ParagraphFormat.Alignment)
        grid_on, rowend = 0, 0
        spacing = collections.Counter()
        for p in doc.Paragraphs:
            f = p.Format
            if not f.DisableLineHeightGrid:
                if p.Range.Text == "\r\x07":
                    rowend += 1
                else:
                    grid_on += 1
            spacing[(f.LineSpacingRule, round(f.SpaceBefore, 1), round(f.SpaceAfter, 1))] += 1
        print("paragraphs", doc.Paragraphs.Count, "| snap-to-grid ON (real paragraphs)", grid_on, "| table end-of-row marks", rowend)
        print("line-spacing rules/space (rule, before, after):", dict(spacing))
        # fonts per character class (sampled over runs via Range.Characters is slow; use words)
        fonts = collections.Counter()
        sizes = collections.Counter()
        for p in doc.Paragraphs:
            r = p.Range
            if r.OMaths.Count or r.InlineShapes.Count:
                continue
            fonts[(r.Font.Name, r.Font.NameFarEast)] += 1
            sizes[r.Font.Size] += 1
        print("paragraph fonts (ascii, eastAsia):", dict(fonts))
        print("paragraph sizes:", dict(sizes))
        for i, s in enumerate(doc.InlineShapes, 1):
            print(f"figure {i}: width {s.Width / CM:.3f} cm height {s.Height / CM:.3f} cm scale {s.ScaleWidth:.1f}%/{s.ScaleHeight:.1f}%")
        print("equations (OMath):", doc.OMaths.Count)
        for i, t in enumerate(doc.Tables, 1):
            b = t.Rows(1).Borders
            top = t.Rows(1).Borders(-1).LineStyle, t.Rows(1).Borders(-1).LineWidth
            hdr = t.Rows(1).Borders(-3).LineStyle, t.Rows(1).Borders(-3).LineWidth
            last = t.Rows(t.Rows.Count).Borders(-3).LineStyle, t.Rows(t.Rows.Count).Borders(-3).LineWidth
            vert = t.Borders(-6).LineStyle, t.Borders(-2).LineStyle, t.Borders(-4).LineStyle
            mid = t.Rows(2).Borders(-3).LineStyle if t.Rows.Count > 2 else None
            print(f"table {i}: rows {t.Rows.Count} cols {t.Columns.Count} | top {top} header-bottom {hdr} bottom {last} | inner-vert/left/right {vert} | row2-bottom {mid} | "
                  f"cell pad T/B {t.TopPadding}/{t.BottomPadding} | font {t.Range.Font.Name}/{t.Range.Font.NameFarEast} {t.Range.Font.Size}pt")
    finally:
        doc.Close(False)
