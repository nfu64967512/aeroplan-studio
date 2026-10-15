# -*- coding: utf-8 -*-
"""⚠️⚠️⚠️ contract_verify.py 的**兩條 import 路徑必須帶同一組名字**。

═══════════════════════════════════════════════════════════════════════════
 為什麼有這個檔(2026-08-24 燒掉一趟 SITL 換來的)
═══════════════════════════════════════════════════════════════════════════
`contract_verify.py` 這樣載入契約表:

    try:
        from sar_swarm.contract_table import (...)      # 已安裝的 package
    except ImportError:
        from contract_table import (...)                # 相對路徑退路

上面那條是**生產路徑**(deploy_build.sh 用 md5 驗過的那一份),下面那條是
開發機直跑的退路 —— 檔案自己的註解就是這樣寫的。

⇒ 也就是說:**測試走下面那條、真機走上面那條。**

實測後果:`requested_vs_effective` 只被加進**下面**那條。
`contract_table.py` 裡那個函式有 30 條測試全部通過,而它的**呼叫端**在真機上
第一次執行就 `NameError`,整批航次中止(LATOBS,2026-08-24 12:48)。

⭐ 諷刺的地方值得留著:那一層檢查加進來的理由,就寫在它自己的呼叫點旁邊 ——
  「沒有這一層時,五顆 export True 的開關在節點上跑 0,而每一道閘門都說 OK
   ⇒ WV1-3 三趟約 2.5 小時 SITL 白跑」。**防白跑的東西自己讓一趟白跑了。**

═══════════════════════════════════════════════════════════════════════════
 這個檔守的不是「有沒有 import 某個名字」
═══════════════════════════════════════════════════════════════════════════
是**兩份清單相等**。逐個列名的寫法會在下次有人加第七個名字時再犯一次;
比對集合則對「加什麼」免疫 —— 加什麼都好,兩邊一起加就對。

⚠️ 這是本專案反覆出現的同一個形狀:兩條路徑,一條被測、一條被出貨。
   [[two-launcher-paths-differ]] 是同一件事的另一個實例
   (tot_node 的兩條啟動路徑參數覆蓋 55/55 vs 24/55,換 launcher 讓 31 個
    開關靜靜失效,而契約閘門擋不住因為退回值合法)。
"""
import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
CV = ROOT / "scripts" / "wsl" / "contract_verify.py"
CT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "contract_table.py"


def _import_pairs(tree):
    """回傳 [(模組名, {名字, ...}), ...],只看 contract_table 那些。"""
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom) or not node.module:
            continue
        if not node.module.endswith("contract_table"):
            continue
        out.append((node.module, {a.name for a in node.names}))
    return out


@pytest.fixture(scope="module")
def cv_tree():
    if not CV.exists():
        pytest.skip("找不到 contract_verify.py")
    return ast.parse(CV.read_text(encoding="utf-8"), filename=str(CV))


def test_there_really_are_two_import_paths(cv_tree):
    """⭐ 先證明這個檔守的東西存在,否則它可能在防一個已經消失的形狀。

    ⚠️ 這條哪天紅了,不代表要刪掉它 —— 先確認 try/except 那段是**刻意**
       改成單一路徑的。若是,整個檔可以退休;若不是,那就是有人把生產路徑
       刪掉了,而那比原本的 bug 更糟。
    """
    pairs = _import_pairs(cv_tree)
    mods = {m for m, _ in pairs}
    assert len(pairs) >= 2, (
        "⚠️ contract_verify.py 只剩 %d 條 contract_table 的 import(%s)。" %
        (len(pairs), sorted(mods)))
    assert any(m.startswith("sar_swarm.") for m, _ in pairs), (
        "⚠️⚠️ 找不到 `sar_swarm.contract_table` 那一條 —— 那是**生產路徑**"
        "(deploy_build.sh 用 md5 驗過的那一份)。少了它,真機會落到相對路徑,"
        "而相對路徑載到的可能是沒被驗過的檔。")


def test_both_paths_import_exactly_the_same_names(cv_tree):
    """⭐⭐⭐ 本檔的主鎖。

    ⚠️ 比對**集合相等**,不是「有沒有某個名字」。逐個列名會在下次有人加
       第七個名字時再犯一次同樣的錯;集合相等對「加什麼」免疫。
    """
    pairs = _import_pairs(cv_tree)
    # 同一個「頂層 try/except」下的兩條;檔裡另有函式內的區域 import
    # (required_gamma_deg),那一對同樣必須相等,所以全部一起比。
    by_kind = {}
    for mod, names in pairs:
        by_kind.setdefault("pkg" if mod.startswith("sar_swarm.") else "rel",
                           []).append(names)
    assert "pkg" in by_kind and "rel" in by_kind

    # 依出現順序配對:第 k 個 package import 對第 k 個相對 import。
    for k, (a, b) in enumerate(zip(by_kind["pkg"], by_kind["rel"])):
        assert a == b, (
            "⚠️⚠️⚠️ 第 %d 對 import 的名字清單不一致。\n"
            "    只有 package 那條有:%s\n"
            "    只有相對那條有  :%s\n"
            "    ⇒ **測試走相對那條、真機走 package 那條** —— 少的那些名字\n"
            "      在真機上第一次用到就 NameError,而單元測試全綠。\n"
            "      2026-08-24 就是這樣中止了一整批航次\n"
            "      (`requested_vs_effective` 只加在相對那條)。"
            % (k + 1, sorted(a - b) or "無", sorted(b - a) or "無"))


def test_every_imported_name_actually_exists_in_the_table(cv_tree):
    """⚠️ 兩邊一致還不夠 —— 兩邊可能**一起**寫錯一個不存在的名字。

    那樣 package 路徑會在 import 當下就 ImportError,然後**靜靜落到**相對
    路徑,而相對路徑也會 ImportError…… 但如果剛好只有一邊拼錯,錯誤會被
    `except ImportError` 吃掉,變成「用了退路」而沒有任何訊息。
    """
    if not CT.exists():
        pytest.skip("找不到 contract_table.py")
    ct = ast.parse(CT.read_text(encoding="utf-8"), filename=str(CT))
    top = set()
    for node in ct.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            top.add(node.name)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    top.add(t.id)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            top.add(node.target.id)
    wanted = set()
    for _, names in _import_pairs(cv_tree):
        wanted |= names
    missing = sorted(wanted - top)
    assert not missing, (
        "⚠️⚠️ contract_verify.py 要 import 的名字在 contract_table.py 裡不存在:"
        "%s\n"
        "    ⚠️ package 那條會在 import 當下 ImportError,然後被 `except`\n"
        "      **靜靜**吃掉、落到相對路徑 —— 沒有任何訊息說「生產路徑掛了」。"
        % missing)
