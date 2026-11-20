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

# ⚠️ CSV 是**KPI 彙總表**,本檔一律 append。若不小心把軌跡 CSV(tot_*.csv)
#    餵進來,就會在 12 MB 的航次資料尾巴悄悄多一列髒資料 —— 已經發生過一次,
#    六趟存檔全中,而且沒有任何錯誤訊息。這裡直接擋掉,寧可吵也不要靜默污染。
if os.path.exists(CSV) and os.path.getsize(CSV) > 0:
    with open(CSV, encoding="utf-8", errors="replace") as _f:
        _head = _f.readline().strip()
    if _head.startswith("t,sysid") or _head.startswith("time,sysid"):
        raise SystemExit(
            f"拒絕寫入:{CSV} 看起來是軌跡 CSV(表頭 '{_head[:40]}'),"
            "不是 KPI 彙總表。CSV 環境變數應指向 /tmp/stot_kpi.csv 之類的彙總檔。")

txt = open(LOG, encoding="utf-8", errors="replace").read()
# ⚠️ 訂正(2026-08-05):舊註解寫「零 miss → 舊哨兵」,那是錯的 ——
#    零 miss 但**有機高空掠過**時(RD1 形狀:6/6 命中、命中高度 598–1460 m),
#    上一版的達標描述會把「俯衝命中紅心:全員命中」整串蓋掉,這裡就抓不到行,
#    寫出 `TRIAL,,,,0,0,no_summary`,而 stot_trial.sh / stot_kpi_loop.sh 則空等
#    到 timeout。現在 tot_node 改成**哨兵與達標描述分離**:
#      ★★ 俯衝結案:{命中}/{n} 命中、{miss} MISS[…];{達標描述}。{量測}
#    CLOSEOUT 這個固定哨兵在四種結案形狀(全部低空觸地 / 高空掠過 / 部分 MISS /
#    全部 MISS)下都存在,而達標描述只給人看。
#    來源合約:sar_swarm_patches/tot_node.py 的 CLOSEOUT_SENTINEL / LEGACY_ALL_HIT。
CLOSEOUT = "★★ 俯衝結案:"          # 固定哨兵:四種形狀都有,唯一該用來偵測結案的字串
LEGACY = "俯衝命中紅心:全員命中"     # 只為了讀 2026-08-05 以前的舊 log(當時沒有哨兵)
# ⚠️ 脫靶那幾架印的是 `2: MISS` 而不是數字,下面的
#    `re.findall(r":\s*(-?[\d.]+)")` 因此**只會抓到命中的那幾架** ⇒
#    n_hit 自動變成真正的命中數 ⇒ n_hit != 6 判失敗。這是刻意的:
#    出口的用途是讓航次能結案,不是讓脫靶被算成命中。
line = ""
for ln in txt.splitlines():
    if CLOSEOUT in ln or LEGACY in ln:
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
