"""逐段量行數：python linecount.py [base|<候選代號>...]

在格式檔副本上以與正式排版相同的 XML 段落格式逐段插入，回報每段行數與最後一行填滿比例，
用來把候選改寫調到「真的少一行」。只用私有 Word。
"""
from __future__ import annotations

import copy
import os
import sys

sys.stdout.reconfigure(encoding="utf-8")
sys.dont_write_bytecode = True

import candidates as cand                                  # noqa: E402
import paper_model as pm                                   # noqa: E402
import render_xml as rx                                    # noqa: E402
rx.LAYOUT.update(rx.LAYOUT_DEFAULT)
from render_word import CM, WordSession                    # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "work", "template_base.docx")


def para_lines(word, items):
    """items: [(label, kind, runs)] → [(label, lines, last_line_fill)]"""
    doc = word.Documents.Open(BASE, ReadOnly=True, AddToRecentFiles=False, Visible=False)
    out = []
    try:
        left = doc.PageSetup.LeftMargin
        width = doc.PageSetup.PageWidth - left - doc.PageSetup.RightMargin
        for label, kind, runs in items:
            doc.Content.Delete()
            xml = rx.PKG_HEAD + rx.p_xml(kind, runs) + rx.PKG_TAIL
            doc.Range(0, 0).InsertXML(xml)
            p = doc.Paragraphs(1).Range
            n = p.ComputeStatistics(1)
            end = doc.Range(p.End - 1, p.End - 1)
            x = end.Information(5) - left          # wdHorizontalPositionRelativeToPage
            out.append((label, n, round(x / width, 2)))
    finally:
        doc.Close(False)
    return out


def main(which):
    base = pm.apply_cuts(pm.parse(), [1, 2, 3, 4, 5, 6, 7, 8])
    sets = {"base": base}
    for cid in which:
        if cid != "base":
            sets[cid] = cand.CANDIDATES[cid]["fn"](copy.deepcopy(base))
    with WordSession() as ws:
        for name, bl in sets.items():
            items = [(f"{k:02d} {b['kind']} {pm.text_of(b['runs'])[:18]}", b["kind"], b["runs"])
                     for k, b in enumerate(bl)
                     if b["kind"] in ("abs", "para", "note", "fcap", "ack", "keywords")]
            res = para_lines(ws.word, items)
            tot = sum(n for _, n, _ in res)
            print(f"== {name}: total text lines {tot}")
            for lab, n, fill in res:
                print(f"   {n:2d}  last={fill:.2f}  {lab}")


if __name__ == "__main__":
    main(sys.argv[1:] or ["base"])
