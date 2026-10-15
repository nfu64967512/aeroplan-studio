import re, sys
sys.stdout.reconfigure(encoding="utf-8")
import pythoncom, win32com.client
def text(p):
    pythoncom.CoInitialize()
    w = win32com.client.DispatchEx("Word.Application"); w.Visible = False; w.DisplayAlerts = 0
    try:
        d = w.Documents.Open(p, ReadOnly=True, AddToRecentFiles=False); t = d.Content.Text; d.Close(False)
    finally:
        w.Quit()
    return t
WL = r"C:\Users\joy46\Documents\Projects\aeroplan-studio\runs\artifacts\PAPERPLAN1\word_layout\variants"
full = text(WL + r"\V0.docx")
four = text(WL + r"\V0pL1pL2pL3.docx")
num = re.compile(r"\d+(?:\.\d+)?")
fs, ks = set(num.findall(full)), num.findall(four)
new = sorted({n for n in ks if n not in fs}, key=lambda x: float(x))
print("numbers in 4-page:", len(set(ks)), "| not found anywhere in full version:", new)
