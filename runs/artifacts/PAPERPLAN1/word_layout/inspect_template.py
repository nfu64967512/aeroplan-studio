import win32com.client, pythoncom, os, sys, json
sys.stdout.reconfigure(encoding='utf-8')
W = os.path.abspath('work')
word = win32com.client.DispatchEx("Word.Application")
word.Visible = False
word.DisplayAlerts = 0
try:
    doc = word.Documents.Open(os.path.join(W,'template_copy.doc'), ReadOnly=True, AddToRecentFiles=False)
    ps = doc.PageSetup
    print('page', ps.PageWidth, ps.PageHeight, 'margins T/B/L/R', ps.TopMargin, ps.BottomMargin, ps.LeftMargin, ps.RightMargin,
          'header/footer dist', ps.HeaderDistance, ps.FooterDistance, 'layoutmode', ps.LayoutMode, 'charsLine', ps.CharsLine, 'linesPage', ps.LinesPage)
    print('pages', doc.ComputeStatistics(2))
    for i,p in enumerate(doc.Paragraphs):
        r = p.Range
        f = p.Format
        t = r.Text.replace('\r','\r').replace('\x07','<c>')
        print(i, repr(t[:90]), '| style', p.Style.NameLocal, '| font', r.Font.Name, r.Font.NameFarEast, r.Font.Size, 'b', r.Font.Bold,
              '| align', f.Alignment, 'ls', f.LineSpacingRule, f.LineSpacing, 'grid', f.DisableLineHeightGrid, 'sb/sa', f.SpaceBefore, f.SpaceAfter, 'ind', f.FirstLineIndent, f.LeftIndent, 'charunit', f.CharacterUnitFirstLineIndent)
    for s in doc.Sections:
        for k in (1,2,3):
            h = s.Headers(k)
            if h.Exists:
                print('HEADER', k, repr(h.Range.Text), h.Range.Font.Name, h.Range.Font.Size)
            fo = s.Footers(k)
            if fo.Exists:
                print('FOOTER', k, repr(fo.Range.Text), fo.Range.Fields.Count, fo.PageNumbers.Count)
    print('tables', doc.Tables.Count, 'shapes', doc.Shapes.Count, 'inl', doc.InlineShapes.Count)
    st = doc.Styles('Normal') if False else doc.Styles(-1)
    print('Normal', st.NameLocal, st.Font.Name, st.Font.NameFarEast, st.Font.Size, st.ParagraphFormat.DisableLineHeightGrid, st.ParagraphFormat.LineSpacingRule)
    doc.Close(False)
finally:
    word.Quit()
