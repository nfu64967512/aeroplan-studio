"""caveat 清單檢核：每個 caveat 關鍵片語必須仍出現在排版後的文字裡（去空白比對）。"""
from __future__ import annotations

import paper_model as pm

CAVEATS = [
    # 摘要／前言
    "覆蓋率99.97%（上界）", "模擬風場", "韌體正規化後之期望值", "本文不主張單一元件為新",
    # 2.3
    "非本文之貢獻", "並非機數下界", "略大於25°", "為假設值", "為估計值",
    # 3.1
    "不申請豁免", "為上界", "連續時間最小值", "紊流為0" , "讀回確認",
    # 3.2 與表2 表註
    "SITL平台設定而非韌體需求", "max(0.01,0.1%)", "91／96", "85／96", "引用須連同本註",
    "天花板設定而非豁免", "高估0.37%",
    # 3.3
    "無法察覺", "不宜外推", "44.0s為刻意延後放行", "不比較",
    # 表3 表註 (a)–(e)
    "尺度偏移", "−0.046至+0.014m/s", "（2.0→6.0m）", "量值未能自模擬器原始碼導出", "≤0.215m",
    "陣風取1.3倍風速", "差兩個變因", "2s網格上界", "每格一趟",
    # 3.4（刪減 8 之句或表4 表註）
    "單一情境、單次執行",
    # 3.5
    "模型值",
    # 限制
    "僅以SITL內建機體模型", "無紊流", "非場址實測", "NO_GO", "0.426m", "0.291m", "28.4m低於50m門檻",
    "唯一防線", "無垂直隔離", "不蘊含SITL飛行≥100m", "而非「條帶數少於3」", "此缺口源自切區結果",
    "順序掃描對照組無法以SITL飛行", "不主張航程效率", "0.528、0.617", "12.5%與14.4%", "列為後續工作",
    "6.15m/s生效", "條帶方位不隨風向調整", "各自重解", "此裕度不可外推", "0.303m", "決定性模擬",
    "1趟",
]
# 3.4 表註（有表4 時）或刪減 8 之句（無表4 時）二擇一
EITHER = [("未納入版本控制", "原始資料未納管")]
# 「紊流為 0」K 改寫可能寫成「紊流 0」
ALIASES = {"紊流為0": ["紊流為0", "紊流0"]}


def flat(blocks: list) -> str:
    out = []
    for b in blocks:
        if "runs" in b:
            out.append(pm.text_of(b["runs"]))
        if b["kind"] == "table":
            for row in b["rows"]:
                for c in row:
                    out.append(pm.text_of(c))
    return "".join(out).replace(" ", "").replace("\u2003", "")


def missing(blocks: list) -> list:
    t = flat(blocks)
    miss = []
    for c in CAVEATS:
        alts = ALIASES.get(c, [c])
        if not any(a in t for a in alts):
            miss.append(c)
    for a, b in EITHER:
        if a not in t and b not in t:
            miss.append(f"{a}|{b}")
    return miss
