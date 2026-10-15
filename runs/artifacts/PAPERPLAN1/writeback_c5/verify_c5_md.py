"""驗收：新的 cowork_brief.md（不套任何刪減，只套 L1/L2/L3 版面規則）排出的 Word 全文須與 C5_reference.txt 逐字相同且為 4 頁；
另以長版 cowork_brief_long.md 套完整 C5 組合重排，確認排版工具修改（式的內容改依 LaTeX 特徵）沒有改變原本的結果。"""
import difflib
import json
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
sys.dont_write_bytecode = True
WL = Path(r"C:\Users\joy46\Documents\Projects\aeroplan-studio\runs\artifacts\PAPERPLAN1\word_layout")
OUT = Path(r"C:\Users\joy46\Documents\Projects\aeroplan-studio\runs\artifacts\PAPERPLAN1\writeback_c5")
sys.path.insert(0, str(WL))
import os
os.chdir(WL)

import paper_model as pm            # noqa: E402
import run_variants as rv           # noqa: E402
import pythoncom                    # noqa: E402
import win32com.client              # noqa: E402

REF = (OUT / "C5_reference.txt").read_text(encoding="utf-8")


def text_of(docx: str) -> tuple[str, int]:
    pythoncom.CoInitialize()
    w = win32com.client.DispatchEx("Word.Application")
    w.Visible = False
    w.DisplayAlerts = 0
    try:
        d = w.Documents.Open(docx, ReadOnly=True, AddToRecentFiles=False)
        t = d.Content.Text.replace("\r", "\n")
        pages = d.ComputeStatistics(2)
        d.Close(False)
    finally:
        w.Quit()
    return t, pages


def check(label: str, docx: str) -> bool:
    t, pages = text_of(docx)
    diff = list(difflib.unified_diff(REF.splitlines(), t.splitlines(), "C5_reference", label, lineterm="", n=0))
    (OUT / f"diff_{label}.txt").write_text("\n".join(diff), encoding="utf-8")
    ok = not diff and pages == 4
    print(f"{label}: pages={pages} diff_lines={len(diff)} => {'OK' if ok else 'MISMATCH'}")
    for line in diff[:20]:
        print("   ", line)
    return ok


# 1) 新 md，不套刪減
rv.main(["V0+L1+L2+L3"])
ok1 = check("new_md_V0pL1pL2pL3", str(WL / "variants" / "V0pL1pL2pL3.docx"))

# 2) 長版＋完整 C5 組合（parse 預設路徑於定義時綁定，改 __defaults__）
pm.parse.__defaults__ = (pm.MD.parent / "cowork_brief_long.md",)
rv.main(["C5 四頁(須刪圖1與圖3)"])
ok2 = check("long_md_C5", str(WL / "variants" / "C5.docx"))

json.dump({"new_md_ok": ok1, "long_md_ok": ok2}, open(OUT / "verify_result.json", "w"), indent=1)
sys.exit(0 if ok1 and ok2 else 1)
