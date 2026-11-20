# Independent page-length estimate for the Word rendering of cowork_brief.md body.
# Model: A4, 2 cm margins -> 17.0 x 25.7 cm; 11 pt; CJK/fullwidth = 1 unit, ASCII = 0.5 unit;
# 43.8 units per line; single spacing with document grid OFF.
# Line pitch is the main uncertainty -> report a band (0.47 / 0.494 / 0.53 cm per line).
import math, re, sys, json, unicodedata
src = sys.argv[1]
L = open(src, encoding='utf-8').read().splitlines()
start = next(i for i,l in enumerate(L) if l.startswith('### 題目'))
end = next(i for i,l in enumerate(L) if l.startswith('## 附錄 A'))
body = L[start:end]
UPL = 43.8
def units(s):
    s = re.sub(r'\$\$.*?\$\$', '', s)
    s = re.sub(r'\$([^$]*)\$', lambda m: re.sub(r'[\{}_^]', '', m.group(1)), s)
    s = s.replace('**', '').replace('`', '')
    u = 0.0
    for ch in s:
        u += 1.0 if unicodedata.east_asian_width(ch) in ('W', 'F') else 0.5
    return u
lines = 0.0; eq_cm = 0.0; fig_cm = 0.0; tables = 0; in_code = False
figs = {'fig1_architecture.png': 6.8, 'fig2_flowchart.png': 7.59, 'fig5_compliance.png': 6.2}
skip_prefixes = ('Word 線性格式', '備選', '**主推', '### 題目', '### 作者與', '### 摘要（')
i = 0
while i < len(body):
    l = body[i]
    if l.startswith('```'):
        in_code = not in_code; i += 1; continue
    if in_code or not l.strip() or l.startswith('---'):
        i += 1; continue
    if l.startswith('$$'):
        eq_cm += 2.0 if ('aligned' in l or 'W^*' in l) else 1.2; i += 1; continue
    if l.startswith('> 【插入圖'):
        for f, h in figs.items():
            if f in l: fig_cm += h
        lines += 1  # caption line
        i += 1; continue
    if l.startswith('> '):
        i += 1; continue
    if l.startswith('（**待使用者') or l.startswith(skip_prefixes):
        i += 1; continue
    if l.startswith('|'):
        rows = []
        while i < len(body) and body[i].startswith('|'):
            if not re.match(r'^\|[-|\s]+\|$', body[i]):
                rows.append([c.strip() for c in body[i].strip('|').split('|')])
            i += 1
        ncol = len(rows[0])
        # column widths proportional to max content per column (capped), sum = UPL
        mx = [max(units(r[c]) for r in rows if c < len(r)) for c in range(ncol)]
        tot = sum(min(m, UPL) for m in mx)
        w = [max(4.0, UPL * min(m, UPL) / tot) for m in mx]
        for r in rows:
            lines += max(math.ceil(units(r[c]) / (w[c] - 1.0)) if units(r[c]) else 1 for c in range(min(ncol, len(r))))
        tables += 1
        lines += 0.5
        continue
    if l.startswith('#'):
        lines += 1.3; i += 1; continue
    lines += math.ceil(units(l) / UPL); i += 1
out = {}
for pitch in (0.47, 0.494, 0.53):
    cm = lines * pitch + eq_cm + fig_cm
    out[str(pitch)] = round(cm / 25.7, 2)
print(json.dumps({'text_lines': round(lines, 1), 'eq_cm': eq_cm, 'fig_cm': round(fig_cm, 2), 'tables': tables, 'pages_by_line_pitch_cm': out}, ensure_ascii=False))
