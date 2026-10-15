"""把 4 頁版 cowork_brief.md 保留之 4 條顯示方程式由 \\tag{3..6} 改為 \\tag{1..4}，並修正 0.4 註記中誤入的 TAB。"""
import re
import sys

P = r"C:\Users\joy46\Documents\Projects\aeroplan-studio\docs\hv_dccpp\AASRC2026_abstract\cowork_brief.md"
raw = open(P, "rb").read().decode("utf-8")
MAP = {"3": "1", "4": "2", "5": "3", "6": "4"}

lines = raw.split("\r\n")
changed = 0
for i, line in enumerate(lines):
    if line.startswith("$$") and line.endswith("$$"):
        m = re.search(r"\\tag\{(\d)\}\$\$$", line)
        if m and m.group(1) in MAP:
            lines[i] = line[:m.start(1)] + MAP[m.group(1)] + line[m.end(1):]
            changed += 1
out = "\r\n".join(lines)

bad = "`\tag{1}`–`\tag{4}`"          # 先前 heredoc 把 \\t 變成 TAB
good = "`\\tag{1}`–`\\tag{4}`"
n_note = out.count(bad)
out = out.replace(bad, good)
tabs_left = [k for k, l in enumerate(out.split("\r\n"), 1) if "\t" in l and "ag{" in l]

open(P, "wb").write(out.encode("utf-8"))
print("tags changed:", changed, "| note fixed:", n_note, "| lines still containing TAB+ag{:", tabs_left)
sys.exit(0 if changed == 4 and n_note == 1 and not tabs_left else 1)
