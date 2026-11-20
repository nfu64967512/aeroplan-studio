P = r"C:\Users\joy46\Documents\Projects\aeroplan-studio\docs\hv_dccpp\AASRC2026_abstract\cowork_brief.md"
s = open(P, "rb").read().decode("utf-8")
bad = "`" + chr(9) + "ag{}`"
good = "`" + chr(92) + "tag{}`"
n = s.count(bad)
s = s.replace(bad, good)
open(P, "wb").write(s.encode("utf-8"))
print("fixed", n, "| TAB count now:", s.count(chr(9)))
