#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 contract_table.py 的生效值契約表匯出成 JSON,給 MATLAB 端讀。

⚠️⚠️ 存在的理由:**表只能有一份**。

如果 MATLAB 端自己手抄一份區間/泡棉禁區,兩份就會漂移,而漂移的症狀正是
本專案付過最貴代價的那一種 —— 照跑、照出數字、結論卻是錯的。所以
`matlab/mgs/contract_table.json` 是**產生物**,不是原始檔:改契約請改
`scripts/wsl/sar_swarm_patches/contract_table.py`,再重跑本檔。

用法(Windows 或 WSL 皆可,本檔不 import ROS2/pymavlink):

    python matlab/mgs/bridge/export_contract_table.py

輸出 `matlab/mgs/contract_table.json`,含:

  meta.source_sha1  contract_table.py 的 sha1 —— MATLAB 端每次載入都比對,
                    不一致就中止。**這是「表過期了但沒人發現」的唯一防線。**
  table[]           每個關鍵參數一列 {name, source, lo, hi, foam, why, evidence}
  env_for{}         參數名 → 批次腳本環境變數名
  jet_reference{}   噴射組態的參考值
  bool_params[]     以 0/1 表達的參數(區間檢查要另眼看待)
  int_params[]      整數參數
  unproven[]        ⚠️ 尚未被實測釘死的契約:命中這些的違規只是提示,不是證據
  invariants[]      關係式的**名稱與說明**。⚠️ 判定邏輯是 Python callable,
                    沒有匯出;MATLAB 端只會顯示「本表有 N 條不變式未在此檢查」
                    並要求走 scripts/wsl/contract_verify.py。刻意不做半套。
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
PATCHES = os.path.join(ROOT, "scripts", "wsl", "sar_swarm_patches")
OUT = os.path.join(HERE, "..", "contract_table.json")

sys.path.insert(0, PATCHES)
import contract_table as ct  # noqa: E402


def _jsonable(v):
    """None 保留成 null;MATLAB 端用 isempty() 判「這一側不設限」。"""
    if isinstance(v, (bool, int, float, str)) or v is None:
        return v
    return str(v)


def main() -> int:
    src_path = os.path.join(PATCHES, "contract_table.py")
    with open(src_path, "rb") as fh:
        sha1 = hashlib.sha1(fh.read()).hexdigest()

    rows = []
    for con in ct.TABLE:
        d = dataclasses.asdict(con)
        rows.append({k: _jsonable(v) for k, v in d.items()})

    inv = []
    for i, iv in enumerate(ct.INVARIANTS):
        # INVARIANTS 的形狀在 contract_table.py 裡可能是 dataclass 也可能是
        # tuple/callable。不猜:能 asdict 就 asdict,不能就只留可讀的字串。
        try:
            d = dataclasses.asdict(iv)
            inv.append({k: _jsonable(v) for k, v in d.items()})
        except TypeError:
            name = getattr(iv, "name", None) or getattr(iv, "__name__", None)
            # ⚠️⚠️ **不可以**用 str(iv):callable 的 repr 含記憶體位址
            #    ("<function _inv_… at 0x000001905FFA37F0>")⇒ 同一份來源
            #    每次匯出的位元組都不同。實測連跑三次三個 md5。
            #    後果:這是**產生檔**,每次重跑都製造假 diff,而多 session
            #    同時工作時就分不出「契約真的改了」與「有人重跑了匯出」。
            # ⭐ 改存 docstring 第一行 —— 穩定,而且比位址有用得多
            #    (MATLAB 端本來就只顯示名稱與說明)。
            doc = (getattr(iv, "__doc__", "") or "").strip().splitlines()
            inv.append({"name": name or f"invariant_{i}",
                        "doc": doc[0].strip() if doc else ""})

    doc = {
        "meta": {
            "source": os.path.relpath(src_path, ROOT).replace("\\", "/"),
            "source_sha1": sha1,
            "n_contracts": len(rows),
            "n_invariants": len(inv),
            "generator": "matlab/mgs/bridge/export_contract_table.py",
            "note": (
                "GENERATED FILE. Edit contract_table.py and re-run the "
                "generator; do not hand-edit this JSON."
            ),
        },
        "table": rows,
        "env_for": {k: _jsonable(v) for k, v in ct.ENV_FOR.items()},
        "jet_reference": {k: _jsonable(v) for k, v in ct.JET_REFERENCE.items()},
        "bool_params": sorted(ct.BOOL_PARAMS),
        "int_params": sorted(ct.INT_PARAMS),
        "unproven": sorted(ct.UNPROVEN),
        "invariants": inv,
    }

    out = os.path.abspath(OUT)
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=1, sort_keys=False)
        fh.write("\n")
    sys.stdout.write(
        "wrote %s\n  %d contracts, %d invariants, sha1 %s\n"
        % (out, len(rows), len(inv), sha1[:12])
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
