"""量頁驅動：python run_variants.py V0 V1 ...（無參數＝全部）。結果寫 results/<名>.json。"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import time

sys.stdout.reconfigure(encoding="utf-8")
sys.dont_write_bytecode = True

import paper_model as pm            # noqa: E402
import candidates as cand           # noqa: E402
import candidates_extra as cx       # noqa: E402
import caveat_check as cc           # noqa: E402
from render_word import WordSession, make_docx_base   # noqa: E402
import render_xml as rx   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.join(HERE, "work")
VAR = os.path.join(HERE, "variants")
RES = os.path.join(HERE, "results")
TEMPLATE_ORIG = str(pm.D / "AASRC2026-投稿格式_加長型摘要_.doc")
TEMPLATE_COPY = os.path.join(WORK, "template_copy.doc")
FIGDIR = str(pm.D)
OUT = str(pm.D / "out")


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def variants() -> dict:
    """名稱 → (刪減項, 候選清單)。"""
    v = {
        "V0": ([], []),
        "V1": ([1, 2, 3, 4, 5], []),
        "V1+6": ([1, 2, 3, 4, 5, 6], []),
        "V1+7": ([1, 2, 3, 4, 5, 7], []),
        "V1+8": ([1, 2, 3, 4, 5, 8], []),
        "V1+6+7+8": ([1, 2, 3, 4, 5, 6, 7, 8], []),
    }
    base = [1, 2, 3, 4, 5, 6, 7, 8]
    for cid in cand.CANDIDATES:
        v[f"V1+6+7+8+{cid}"] = (base, [cid])
    for name, ids in cx.REPORT_COMBOS.items():
        v[name] = (base, ids)
    for name, ids in cand.COMBOS.items():
        v[name] = (base, ids)
    for name, (cuts, ids) in cand.EXTRA.items():
        v[name] = (cuts, ids)
    return v


def blocks_for(cuts, cids):
    orig = pm.parse()
    bl = pm.apply_cuts(orig, cuts)
    for cid in sorted(cids, key=lambda c: cx.ORDER.index(c) if c in cx.ORDER else 99):
        spec = cand.CANDIDATES.get(cid) or cand.INFO.get(cid)
        bl = spec["fn"](bl)
    return bl


def main(names):
    os.makedirs(VAR, exist_ok=True)
    os.makedirs(RES, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(TEMPLATE_COPY) or sha(TEMPLATE_COPY) != sha(TEMPLATE_ORIG):
        shutil.copyfile(TEMPLATE_ORIG, TEMPLATE_COPY)
    allv = variants()
    names = names or list(allv)
    for nm in names:
        if nm not in allv:
            parts = nm.split("+")
            assert parts[0] in ("V0", "V1"), nm
            cuts = [] if parts[0] == "V0" else [1, 2, 3, 4, 5]
            cids = []
            for x in parts[1:]:
                if x.isdigit():
                    cuts.append(int(x))
                else:
                    cids.append(x)
            allv[nm] = (cuts, cids)
    with WordSession() as ws:
        base = os.path.join(WORK, "template_base.docx")
        print("docx base:", make_docx_base(ws.word, TEMPLATE_COPY, base))
        eqs = rx.build_eq_cache(ws.word, base, os.path.join(WORK, "eq_cache.json"))
        rd = rx.XmlRenderer(ws.word, base, FIGDIR, eqs)
        for name in names:
            cuts, cids = allv[name]
            bl = blocks_for(cuts, cids)
            safe = name.split(" ")[0] if " " in name else name.replace("+", "p")
            docx = os.path.join(VAR, f"{safe}.docx")
            pdf = os.path.join(VAR, f"{safe}.pdf")
            t0 = time.time()
            m = rd.build(bl, docx, pdf)
            m["caveats_missing"] = cc.missing(bl)
            m["caveats_checked"] = len(cc.CAVEATS) + len(cc.EITHER)
            m.update({"name": name, "cuts": cuts, "candidates": cids,
                      "docx": docx, "pdf": pdf})
            json.dump(m, open(os.path.join(RES, f"{safe}.json"), "w", encoding="utf-8"),
                      ensure_ascii=False, indent=1)
            print(f"{name:28s} pages={m['pages']} last_lines={m['last_page_lines']} "
                  f"fill={m['last_page_fill_pct']}% content={m['content_cm']}cm grid_on={m['n_paragraphs_snap_to_grid_on']} "
                  f"cav_miss={m['caveats_missing']} "
                  f"per_page={m['used_cm_per_page']} ({time.time() - t0:.0f}s)")
    assert sha(TEMPLATE_ORIG) == sha(TEMPLATE_COPY)


if __name__ == "__main__":
    main(sys.argv[1:])
