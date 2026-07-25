#!/usr/bin/env python3
"""_kpi_parse.py — 從 tot_node 的收尾行解析 KPI 並判定成功,append 到 CSV。
由 stot_kpi_loop.sh 呼叫;參數走環境變數(TRIAL/CSV/MISS_MAX/ALT_MAX/SPREAD_MAX)。"""
import os
import re

LOG = os.environ.get("TOT_LOG", "/tmp/kpi_tot.log")
CSV = os.environ.get("CSV", "/tmp/stot_kpi.csv")
TRIAL = os.environ.get("TRIAL", "?")
MISS_MAX = float(os.environ.get("MISS_MAX", 30))
ALT_MAX = float(os.environ.get("ALT_MAX", 30))
SPREAD_MAX = float(os.environ.get("SPREAD_MAX", 3.0))

txt = open(LOG, encoding="utf-8", errors="replace").read()
line = ""
for ln in txt.splitlines():
    if "俯衝命中紅心:全員命中" in ln:
        line = ln
if not line:
    with open(CSV, "a", encoding="utf-8") as f:
        f.write(f"{TRIAL},,,,0,0,no_summary\n")
    raise SystemExit

m_sp = re.search(r"命中時間散布 = ([\d.]+)s", line)
spread = float(m_sp.group(1)) if m_sp else 999.0
# 最近水平距 = {1: 11.7, 2: 9.7, ...} m
m_miss = re.search(r"最近水平距 = \{([^}]*)\}", line)
miss = [float(x) for x in re.findall(r":\s*(-?[\d.]+)", m_miss.group(1))] if m_miss else []
# 命中高度 = {1: 5, 2: 0, ...} m
m_alt = re.search(r"命中高度 = \{([^}]*)\}", line)
alts = [abs(float(x)) for x in re.findall(r":\s*(-?[\d.]+)", m_alt.group(1))] if m_alt else []

n_hit = len(miss)
mx_miss = max(miss) if miss else 999.0
mx_alt = max(alts) if alts else 999.0
ok = (n_hit == 6 and mx_miss <= MISS_MAX and mx_alt <= ALT_MAX
      and spread <= SPREAD_MAX)
why = []
if n_hit != 6:
    why.append(f"n={n_hit}")
if mx_miss > MISS_MAX:
    why.append(f"miss{mx_miss:.0f}")
if mx_alt > ALT_MAX:
    why.append(f"alt{mx_alt:.0f}")
if spread > SPREAD_MAX:
    why.append(f"spread{spread:.1f}")
with open(CSV, "a", encoding="utf-8") as f:
    f.write(f"{TRIAL},{spread:.2f},{mx_miss:.1f},{mx_alt:.1f},{n_hit},"
            f"{1 if ok else 0},{'|'.join(why) or 'ok'}\n")
