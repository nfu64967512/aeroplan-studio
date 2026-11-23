"""cowork_brief.md 第 1–2 節 → 可排版的區塊模型（純 Python，不碰 Word）。

區塊（dict）的 kind：
  title / blank / center12 / abs_head / abs / keywords / h1 / h2 / para / eq /
  tcap（表標題）/ table / note（表註）/ fig / fcap（圖說）/ ref / ack

每個區塊的文字以 runs 表示：[(text, fmt)]，fmt 為 dict（b 粗體、i 斜體、sub、sup、font）。
第 0 節與附錄不印；附錄 C 只取圖說定稿。所有刪減選項（0.4 節 1–8）與候選刪減
都寫成對區塊串列的純函式，方便逐一套用、逐一量頁。
"""
from __future__ import annotations

import copy
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
D = ROOT / "docs" / "hv_dccpp" / "AASRC2026_abstract"
MD = D / "cowork_brief.md"

FA = "\u2061"   # function application（Word 方程式）
EM = "\u2003"   # em space
Q2 = EM + EM

# ── 顯示方程式：依 2.3 節之 LaTeX 寫成 Word 線性格式（UnicodeMath），以 # 產生右側式號 ──
# 與 md 所附「Word 線性格式」同義；差異只在排版：(1)(3) n-ary 用 ▒ 綁定被加數、
# (2) tan 後之分式不加括號（同 LaTeX \tan\frac）、(5) 分子 3600 E_use 同 LaTeX、
# (4) 過寬拆兩行、(6) 依 LaTeX 含 conflict(a,b)⟺ 且拆三行。
EQ_LINEAR = {
    "1": ("min" + FA + "C_max=max_i" + FA + "C_i," + Q2 +
          "C_i=t_i^TO+T_i^in+∑▒T_i^oper+∑▒T_i^trans+T_i^RTB+T_i^LDG"),
    "2": ("w=2h tan" + FA + "〖α_H/2〗," + Q2 + "s=w(1−o_s)," + Q2 +
          "h_op=max" + FA + "(h_(min fw),min" + FA + "(h_tgt,h_max,h_ceil))"),
    "3": ("V_g (θ)=√(V_a^2−W_⊥^2)+W_∥," + Q2 +
          "t=∑_line▒L/(V_g (θ))+∑_arc▒∑_(j=1)^n▒(R_eff δ)/(V_g (θ_j))"),
    "4": ("█(R_(eff,c) (W)=max" + FA + "{R_(plan,c),R_(theo,c) (1+W/V_(a,c))^2},@" +
          "R_(theo,c)=(V_(a,c)^2)/(g tan" + FA + "φ_(max,c))," + Q2 +
          "W_c^*=V_(a,c) (√(R_(plan,c)/R_(theo,c))−1))"),
    "5": ("λ_floor=⌈T_single/(κ_load T_use−t_in−t_RTB)⌉," + Q2 +
          "T_use=(3600 E_use)/(P_c k_unc)"),
    "6": ("█(\"conflict\"(a,b)⟺d_h<d_thr∧d_v<Δh_sep∧|t_a−t_b|<Δt+σ_t (t_a)+σ_t (t_b),@" +
          "σ_t (t)=min" + FA + "{k_t max" + FA + "(0,t−t_sync),σ_max},@" +
          "\"site\"(r_a)=\"site\"(r_b)∧u_a≠u_b⇒I(r_a)∩I(r_b)=∅)"),
    # 候選 L1：同一式 (6) 排成兩行（σ_t 定義與場坪互斥同列）
    "6b": ("█(\"conflict\"(a,b)⟺d_h<d_thr∧d_v<Δh_sep∧|t_a−t_b|<Δt+σ_t (t_a)+σ_t (t_b),@" +
           "σ_t (t)=min" + FA + "{k_t max" + FA + "(0,t−t_sync),σ_max}," + Q2 +
           "\"site\"(r_a)=\"site\"(r_b)∧u_a≠u_b⇒I(r_a)∩I(r_b)=∅)"),
}

# 每條顯示方程式 LaTeX 中獨有的片段 → EQ_LINEAR 的鍵（與顯示式號無關）
EQ_SIGNATURES = [
    (r"\min C_{\max}", "1"),
    (r"w=2h\tan", "2"),
    (r"V_g(\theta)=", "3"),
    (r"R_{eff,c}(W)", "4"),
    (r"\lambda_{floor}", "5"),
    (r"\mathrm{conflict}", "6"),
]

GREEK = {"alpha": "α", "beta": "β", "gamma": "γ", "delta": "δ", "theta": "θ",
         "kappa": "κ", "lambda": "λ", "mu": "μ", "sigma": "σ", "varphi": "φ",
         "phi": "ϕ", "tau": "τ", "omega": "ω", "psi": "ψ"}
UPGREEK = {"Delta": "Δ", "Sigma": "Σ", "Omega": "Ω", "Lambda": "Λ", "Phi": "Φ"}
SYMS = {"parallel": "∥", "perp": "⊥", "lceil": "⌈", "rceil": "⌉", "cdot": "·",
        "times": "×", "ge": "≥", "le": "≤", "geq": "≥", "leq": "≤", "infty": "∞",
        "to": "→", "Rightarrow": "⇒", "iff": "⟺", "land": "∧", "wedge": "∧"}
OPNAMES = {"min", "max", "tan", "sin", "cos", "log", "exp"}


# ════════════════════════════════════════════════════════════════════
#  行內數學（$…$）→ runs：拉丁字母斜體、數字與運算子正體、上下標、Times New Roman
# ════════════════════════════════════════════════════════════════════
def tex_runs(s: str, fmt: dict | None = None) -> list:
    fmt = dict(fmt or {})
    out: list = []
    i = 0
    n = len(s)

    def emit(text, **kw):
        f = dict(fmt)
        f.update(kw)
        f["math"] = True
        out.append((text, f))

    def atom(j):
        """解析 j 起的一個原子，回傳 (runs, 新位置)。"""
        if s[j] == "{":
            depth, k = 1, j + 1
            while depth:
                if s[k] == "{":
                    depth += 1
                elif s[k] == "}":
                    depth -= 1
                k += 1
            return s[j + 1:k - 1], k
        if s[j] == "\\":
            m = re.match(r"\\([A-Za-z]+|.)", s[j:])
            return m.group(0), j + len(m.group(0))
        return s[j], j + 1

    while i < n:
        c = s[i]
        if c in "_^":
            sub, i = atom(i + 1)
            key = "sub" if c == "_" else "sup"
            out.extend(tex_runs(sub, {**fmt, key: True}))
            continue
        if c == "{":
            grp, i = atom(i)
            out.extend(tex_runs(grp, fmt))
            continue
        if c == "\\":
            m = re.match(r"\\([A-Za-z]+)", s[i:])
            if not m:                       # \, \; \  等間距
                i += 2
                continue
            cmd = m.group(1)
            i += len(m.group(0))
            if cmd in GREEK:
                emit(GREEK[cmd], i=True)
            elif cmd in UPGREEK:
                emit(UPGREEK[cmd])
            elif cmd in SYMS:
                emit(SYMS[cmd])
            elif cmd in OPNAMES:
                emit(cmd)
                if i < n and s[i] == " " and i + 1 < n and s[i + 1].isalpha():
                    emit(" ")
            elif cmd == "sqrt":
                arg, i = atom(i)
                emit("√(")
                out.extend(tex_runs(arg, fmt))
                emit(")")
            elif cmd in ("left", "right", "textstyle", "displaystyle", "mathrm"):
                pass
            else:
                raise ValueError(f"unknown TeX command \\{cmd} in {s!r}")
            continue
        if c == " ":
            i += 1
            continue
        if c.isalpha():
            emit(c, i=True)
        elif c == "-":
            emit("−")
        elif c in "=<>":
            emit(f" {c} ")
        else:
            emit(c)
        i += 1
    return out


def merge_runs(runs: list) -> list:
    out: list = []
    for t, f in runs:
        if out and out[-1][1] == f:
            out[-1] = (out[-1][0] + t, f)
        else:
            out.append((t, dict(f)))
    return out


PLAIN_SUB = re.compile(r"\b([RW])_(eff|plan|theo)\b")


def plain_runs(text: str, fmt: dict) -> list:
    """一般文字：R_eff 類下標、W* 上標、ᵃ ᵇ 上標。"""
    out: list = []
    pos = 0
    pat = re.compile(r"(?<![A-Za-z])[RW]_(?:eff|plan|theo)\b|W\*(?=[ ）)=])|[ᵃᵇ]")
    for m in pat.finditer(text):
        if m.start() > pos:
            out.append((text[pos:m.start()], dict(fmt)))
        g = m.group(0)
        if g in "ᵃᵇ":
            out.append(({"ᵃ": "a", "ᵇ": "b"}[g], {**fmt, "sup": True}))
        elif g.startswith("W*"):
            out.append(("W", dict(fmt)))
            out.append(("*", {**fmt, "sup": True}))
        else:
            a, b = g.split("_")
            out.append((a, dict(fmt)))
            out.append((b, {**fmt, "sub": True}))
        pos = m.end()
    if pos < len(text):
        out.append((text[pos:], dict(fmt)))
    return out


def inline(md: str, base: dict | None = None) -> list:
    """md 行內標記 → runs：**粗體**、$數學$、`程式碼`（印成一般文字）。"""
    base = dict(base or {})
    runs: list = []
    tok = re.compile(r"\*\*(.+?)\*\*|\$(.+?)\$|`([^`]+)`")
    pos = 0
    for m in tok.finditer(md):
        if m.start() > pos:
            runs += plain_runs(md[pos:m.start()], base)
        if m.group(1) is not None:
            runs += inline(m.group(1), {**base, "b": True})
        elif m.group(2) is not None:
            runs += tex_runs(m.group(2), base)
        else:
            runs += plain_runs(m.group(3), base)
        pos = m.end()
    if pos < len(md):
        runs += plain_runs(md[pos:], base)
    return merge_runs(runs)


def text_of(runs: list) -> str:
    return "".join(t for t, _ in runs)


# ════════════════════════════════════════════════════════════════════
#  md → 區塊
# ════════════════════════════════════════════════════════════════════
def _captions(md: str) -> dict:
    """附錄 C 之圖說定稿（去掉給 Cowork 的檔名註記）。"""
    sec = md[md.index("## 附錄 C"):md.index("## 附錄 D")]
    caps = {}
    for line in sec.splitlines():
        m = re.match(r"- (圖(\d))(.*)$", line.strip())
        if m:
            rest = re.sub(r"^（檔案 `[^`]+`）", "", m.group(3)).strip()
            caps[m.group(2)] = f"圖{m.group(2)} {rest}"
    return caps


def parse(md_path: Path = MD) -> list:
    md = md_path.read_text(encoding="utf-8")
    caps = _captions(md)
    body = md[md.index("## 1. 封面資訊"):md.index("## 附錄 A")]
    lines = body.splitlines()
    blocks: list = []
    sec = "cover"
    i = 0
    tnum = None
    in_code = False
    while i < len(lines):
        raw = lines[i]
        s = raw.strip()
        i += 1
        if s.startswith("```"):
            in_code = not in_code
            continue
        if in_code or not s or s == "---":
            continue
        if s.startswith("## "):
            continue
        if s.startswith("### ") or s.startswith("#### "):
            h = s.lstrip("#").strip()
            if s.startswith("#### "):
                sec = h.split()[0]
                blocks.append({"kind": "h2", "runs": inline(h), "sec": sec})
            elif h.startswith("題目"):
                sec = "title"
            elif h.startswith("作者與服務單位"):
                sec = "authors"
            elif h.startswith("摘要"):
                sec = "abstract"
                blocks.append({"kind": "blank", "size": 12, "sec": sec})
                blocks.append({"kind": "abs_head", "runs": inline("摘要"), "sec": sec})
            elif h in ("誌謝", "參考文獻"):
                sec = h
                blocks.append({"kind": "h1", "runs": inline(h), "sec": sec})
            else:
                sec = h.split("、")[0]
                blocks.append({"kind": "h1", "runs": inline(h), "sec": sec})
            continue
        if sec == "title":
            if s.startswith("**主推：**"):
                t = s.replace("**主推：**", "").strip()
                blocks.append({"kind": "title", "runs": inline(t), "sec": sec})
                blocks.append({"kind": "blank", "size": 12, "sec": sec})
            continue
        if sec == "authors":
            blocks.append({"kind": "center12", "runs": inline(s), "sec": sec})
            continue
        if s.startswith("$$"):
            buf = s
            while not buf.rstrip().endswith("$$") or buf.strip() == "$$":
                buf += "\n" + lines[i]
                i += 1
            tag = re.search(r"\\tag\{(\d)\}", buf).group(1)
            # 2026-09-21：式的「內容」依 LaTeX 特徵找 EQ_LINEAR 的鍵，「顯示式號」取 md 的 \tag ——
            # 兩者原本綁在同一個數字上，4 頁版刪掉式(1)(2)後剩下的式子就無法在 md 裡重編為 (1)–(4)。
            keys = [k for sig, k in EQ_SIGNATURES if sig in buf]
            assert len(keys) == 1, f"方程式特徵比對 {keys}：{buf[:60]}"
            blocks.append({"kind": "eq", "linear": EQ_LINEAR[keys[0]], "num": f"({tag})",
                           "latex": buf, "sec": sec})
            continue
        if s.startswith("Word 線性格式"):
            continue
        if s.startswith(">"):
            m = re.search(r"插入圖(\d)：`([^`]+)`", s)
            if m:
                blocks.append({"kind": "fig", "file": m.group(2), "fignum": m.group(1),
                               "sec": sec})
                blocks.append({"kind": "fcap", "runs": inline(caps[m.group(1)]),
                               "fignum": m.group(1), "sec": sec})
            continue
        m = re.match(r"\*\*(表(\d) .+?)\*\*", s)
        if m:
            tnum = m.group(2)
            blocks.append({"kind": "tcap", "runs": inline(m.group(1)), "tnum": tnum,
                           "sec": sec})
            continue
        if s.startswith("|"):
            rows = []
            j = i - 1
            while j < len(lines) and lines[j].strip().startswith("|"):
                cells = [c.strip() for c in lines[j].strip().strip("|").split("|")]
                if not all(re.fullmatch(r":?-{3,}:?", c) for c in cells):
                    rows.append([inline(c) for c in cells])
                j += 1
            i = j
            blocks.append({"kind": "table", "rows": rows, "tnum": tnum, "sec": sec})
            continue
        if s.startswith("表註："):
            blocks.append({"kind": "note", "runs": inline(s), "tnum": tnum, "sec": sec})
            continue
        if s.startswith("（**待使用者填寫**"):
            continue                        # 給使用者的指示，不印
        if s.startswith("**關鍵詞：**"):
            s = s.replace("**關鍵詞：** ", "**關鍵詞：**")
            blocks.append({"kind": "keywords", "runs": inline(s), "sec": sec})
            continue
        # 段首粗體引句（「**規劃天花板。** 任務…」）後之半形空白在中文排版中多餘
        s = re.sub(r"^(\*\*[^*]+[。：]\*\*) ", r"\1", s)
        if sec == "參考文獻":
            blocks.append({"kind": "ref", "runs": inline(s), "sec": sec})
            continue
        if sec == "abstract":
            blocks.append({"kind": "abs", "runs": inline(s), "sec": sec})
            continue
        if sec == "誌謝":
            blocks.append({"kind": "ack", "runs": inline(s), "sec": sec})
            continue
        blocks.append({"kind": "para", "runs": inline(s), "sec": sec})
    return blocks


# ════════════════════════════════════════════════════════════════════
#  區塊操作小工具
# ════════════════════════════════════════════════════════════════════
def find(blocks, kind=None, sec=None, startswith=None, contains=None, **kw):
    out = []
    for k, b in enumerate(blocks):
        if kind and b["kind"] != kind:
            continue
        if sec and b.get("sec") != sec:
            continue
        if startswith and not text_of(b.get("runs", [])).startswith(startswith):
            continue
        if contains and contains not in text_of(b.get("runs", [])):
            continue
        if any(b.get(a) != v for a, v in kw.items()):
            continue
        out.append(k)
    return out


def one(blocks, **kw) -> int:
    ks = find(blocks, **kw)
    assert len(ks) == 1, (kw, ks)
    return ks[0]


def replace_text(runs: list, old: str, new_md: str | None = None, new_runs=None) -> list:
    """把 runs 中（可跨 run 的）一段文字 old 換成 new（md 行內語法）。"""
    full = text_of(runs)
    k = full.index(old)
    assert full.count(old) == 1, old
    before = slice_runs(runs, 0, k)
    after = slice_runs(runs, k + len(old), len(full))
    mid = new_runs if new_runs is not None else (inline(new_md) if new_md else [])
    return merge_runs(before + mid + after)


def slice_runs(runs: list, a: int, b: int) -> list:
    out, pos = [], 0
    for t, f in runs:
        s, e = pos, pos + len(t)
        lo, hi = max(a, s), min(b, e)
        if lo < hi:
            out.append((t[lo - s:hi - s], dict(f)))
        pos = e
    return out


def map_text(blocks: list, fn) -> None:
    """對所有 runs（含表格儲存格）逐 run 套用 fn(text)->text。"""
    for b in blocks:
        if "runs" in b:
            b["runs"] = [(fn(t), f) for t, f in b["runs"]]
        if b["kind"] == "table":
            b["rows"] = [[[(fn(t), f) for t, f in c] for c in row] for row in b["rows"]]


def renumber_tables(blocks: list, mapping: dict) -> None:
    def fn(t):
        return re.sub(r"表([1-9])", lambda m: "表" + mapping.get(m.group(1), m.group(1)), t)
    map_text(blocks, fn)
    for b in blocks:
        if b.get("tnum") in mapping:
            b["tnum"] = mapping[b["tnum"]]


def renumber_figs(blocks: list, mapping: dict) -> None:
    def fn(t):
        return re.sub(r"圖([1-9])", lambda m: "圖" + mapping.get(m.group(1), m.group(1)), t)
    map_text(blocks, fn)


# ════════════════════════════════════════════════════════════════════
#  0.4 節刪減選項 1–8（逐字依 brief）
# ════════════════════════════════════════════════════════════════════
def cut1(bl):
    """2.1、2.2 節文字各只保留第一句。"""
    for sec, first in (("2.1", "系統架構如圖1。"), ("2.2", "規劃流程如圖2。")):
        ks = find(bl, kind="para", sec=sec)
        assert ks and text_of(bl[ks[0]]["runs"]).startswith(first), sec
        bl[ks[0]]["runs"] = inline(first)
        bl[ks[0]]["cut1_rest"] = True
        for k in reversed(ks[1:]):
            del bl[k]
    return bl


T1_SENTENCE = ("Pioneer／HERO 巡航空速 21.5／19.0 m/s，規劃轉彎半徑 200／120 m"
               "（規劃裕度可吸收之風速 W* = 5.1675／5.0333 m/s），可用航程 133–145／79–86 km"
               "（估計值），兩型共用 HFOV 60° 相機。")


def cut2(bl):
    """表1 改成內文一句話，表註接在該句之後（表號隨之遞補）。"""
    kc = one(bl, kind="tcap", tnum="1")
    kt = one(bl, kind="table", tnum="1")
    kn = one(bl, kind="note", tnum="1")
    note = bl[kn]["runs"]
    note = replace_text(note, "表註：", new_runs=[])
    runs = merge_runs(inline(T1_SENTENCE) + note)
    new = {"kind": "para", "runs": runs, "sec": bl[kc]["sec"], "from_table1": True}
    for k in sorted((kc, kt, kn), reverse=True):
        del bl[k]
    bl.insert(kc, new)
    renumber_tables(bl, {"2": "1", "3": "2", "4": "3"})
    return bl


def _tnum_of(bl, orig):
    """表號可能已被 cut2 遞補；以表標題文字辨識原表。"""
    key = {"3": "規劃天花板與風場之對照架次", "4": "起降走廊豁免判準對照"}[orig]
    k = [i for i in find(bl, kind="tcap") if key in text_of(bl[i]["runs"])]
    assert len(k) == 1
    return bl[k[0]]["tnum"]


def cut3(bl):
    """3.4 節刪文字，只留表4 與表註。"""
    for k in reversed(find(bl, kind="para", sec="3.4")):
        del bl[k]
    return bl


S35 = ("另設觸發情境（作業區改為三塊標稱 1600 × 1200 m，其餘同表2；各胞 n = 6–13 > k），"
       "以規劃器自身之考慮風場 Dubins 胞內成本評分：全機隊迴轉時間由順序掃描之 1696.88 s "
       "降為 1115.73 s（−34.25%），距 Held–Karp 最佳解仍高 5.24%，均為規劃層模型值。")


def cut4(bl):
    """3.5 節只留「另設觸發情境…均為規劃層模型值。」一句。"""
    ks = find(bl, kind="para", sec="3.5")
    assert len(ks) == 1
    full = text_of(bl[ks[0]]["runs"])
    a = full.index("另設觸發情境")
    b = full.index("均為規劃層模型值。") + len("均為規劃層模型值。")
    bl[ks[0]]["runs"] = merge_runs(slice_runs(bl[ks[0]]["runs"], a, b))
    return bl


def cut5(bl):
    """表3 刪 HV4SITL16 與「W ≥ 6.16 m/s」兩列（表註 (c)(d) 保留）。"""
    t = _tnum_of(bl, "3")
    kt = one(bl, kind="table", tnum=t)
    rows = bl[kt]["rows"]
    keep = [r for r in rows if text_of(r[0]) not in ("HV4SITL16", "W ≥ 6.16 m/s")]
    assert len(keep) == len(rows) - 2
    bl[kt]["rows"] = keep
    return bl


def cut6(bl):
    """圖1 改用 fig1_architecture_compact.png。"""
    k = one(bl, kind="fig", fignum="1")
    bl[k]["file"] = "fig1_architecture_compact.png"
    return bl


def cut7(bl, orig_md_blocks=None):
    """刪圖2 與其圖說；2.2 節首句改「規劃流程如下。」（2.2 節其餘文字保留，圖號遞補）。"""
    kf = one(bl, kind="fig", fignum="2")
    kc = one(bl, kind="fcap", fignum="2")
    for k in sorted((kf, kc), reverse=True):
        del bl[k]
    orig = orig_md_blocks or parse()
    ko = find(orig, kind="para", sec="2.2")
    full = text_of(orig[ko[0]]["runs"])
    assert full.startswith("規劃流程如圖2。")
    rest = merge_runs(slice_runs(orig[ko[0]]["runs"], len("規劃流程如圖2。"), len(full)))
    kp = find(bl, kind="para", sec="2.2")
    bl[kp[0]]["runs"] = merge_runs(inline("規劃流程如下。") + rest)
    for k in reversed(kp[1:]):
        del bl[k]
    for b in bl:
        if b.get("fignum") == "3":
            b["fignum"] = "2"
    renumber_figs(bl, {"3": "2"})
    return bl


S34 = ("另一受控對照中，走廊豁免以場坪座標判定時最小水平間距降至 5.0 m（垂直 19.9 m；"
       "2 s 網格上界），以場坪身分判定則為 52.7 m 且無違規，而規劃驗證與覆蓋率兩者皆無法察覺"
       "（單一情境、單次執行；原始資料未納管）。")


def cut8(bl):
    """3.4 節與表4 改為一句。"""
    ks = [k for k, b in enumerate(bl) if b.get("sec") == "3.4" and b["kind"] != "h2"]
    ins = ks[0]
    for k in reversed(ks):
        del bl[k]
    bl.insert(ins, {"kind": "para", "runs": inline(S34), "sec": "3.4"})
    return bl


CUTS = {1: cut1, 2: cut2, 3: cut3, 4: cut4, 5: cut5, 6: cut6, 7: cut7, 8: cut8}


def apply_cuts(blocks, items):
    bl = copy.deepcopy(blocks)
    for it in items:
        bl = CUTS[it](bl)
    return bl
