"""`allocate.sweep_order` 的搜尋分支（2026-09-10）。

⚠⚠⚠ 為什麼有這個檔：`decompose.exact_time` 早在 vendored #2 就改成**搜尋**條帶序，
但 `allocate.sweep_order` 一直無條件用**固定** skip-k 序 ⇒ 兩個估計層對同一件事
給不同答案：`task.T_single_s` 是**搜尋序**的成本，而 `_chain` 的進入／RTB 端點
卻取自**固定序**的頭尾 —— 兩者對「到底在飛哪條路線」意見不同。

實測這不是惰性的（`scratchpad/sweep_order_decides.py`）：

===========  ==================  ==================
情境         makespan 舊 → 新    指派 / 航程
===========  ==================  ==================
mini         逐位元不變          相同
hills        5087.4 → 5037.6 s   **改變**（A1 航程 90 492 → 75 181 m）
===========  ==================  ==================

⚠⚠ **本檔最重要的一條是 `test_matrix_matches_exact_time`。**
`sweep_order` 的搜尋需要一張條帶兩兩轉場成本矩陣，而那張矩陣的加權規則
（雙向胞取 k=1/k=2 各半、單向胞取 k=3·idx）是 `decompose.exact_time._pair_poses`
的**第二份實作**。兩處算同一件事就是漂移風險，所以要鎖在**值**上：
兩邊對同一組條帶、同一個固定序列算出來的轉場總時間必須相符到機器精度。
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

import hv_dccpp.allocate as AL                                  # noqa: E402
from hv_dccpp.decompose import _skip_sequence, exact_time       # noqa: E402
from hv_dccpp.dubins import skip_k                              # noqa: E402
from tests.hv_dccpp.test_allocate import Bundle                 # noqa: E402


@pytest.fixture(scope="module")
def b():
    return Bundle(1, 1)


def _ids(b):
    return [s.strip_id for s in b.cell.strips["A"]]


def _fixed_seq(b, n):
    ids = _ids(b)[:n]
    strips = [b.cache.strips["A"][s] for s in ids]
    s_mean = float(np.mean([s.spacing_m for s in strips]))
    return _skip_sequence(n, skip_k(s_mean, b.cache.R_eff.get("A", s_mean)))


def test_matrix_matches_exact_time(b):
    """⭐⭐⭐ 值層級：`_task_cost_matrix` 與 `exact_time` 必須是**同一個量**。

    判準是「同一組條帶、同一個固定序列，兩邊的轉場總時間相符」。
    ⚠ 不是比原始碼長相 —— `cache.tr` 的第三軸是 k = 2·idx_l + idx_m，
    索引挑錯（例如把 k=1/k=2 寫成 k=0/k=3）不會炸，只會**靜靜挑到壞順序**。
    """
    ids = _ids(b)
    cfg_fixed = dataclasses.replace(b.cfg, sweep_sequence="fixed")
    dctx = dataclasses.replace(b.ctx.decompose_ctx(), cfg=cfg_fixed)
    strips_all = b.cell.strips["A"]

    checked = 0
    for n in range(3, len(ids) + 1):
        idxs = [b.cache.idx["A"][s] for s in ids[:n]]
        C = AL._task_cost_matrix(idxs, "A", b.cache)
        seq = _fixed_seq(b, n)
        mine = float(C[seq[:-1], seq[1:]].sum())
        ref = exact_time(strips_all[:n], "A", dctx) - sum(
            exact_time([s], "A", dctx) for s in strips_all[:n])
        assert mine == pytest.approx(ref, abs=1e-6), (
            f"n={n}：我的矩陣算出 {mine:.6f}，exact_time 算出 {ref:.6f} "
            f"⇒ cache.tr 的 k 索引或權重與 _pair_poses 漂移了")
        checked += 1
    assert checked >= 3, "有效的 n 太少，這條鎖不到東西"


def test_search_actually_changes_the_order(b):
    """⭐ 前置斷言：搜尋分支**真的**會給出不同的序列。

    ⚠ 少了這條，下面幾條「搜尋不比固定差」全部可以靠「兩者相同」通過 ——
    正是本專案記錄在案的「點不著的儀表」。
    """
    ids = _ids(b)
    diff = [n for n in range(3, len(ids) + 1)
            if AL.sweep_order(ids[:n], "A", b.cache, b.cfg)
            != AL.sweep_order(ids[:n], "A", b.cache, None)]
    assert diff, "搜尋序與固定序在每個 n 都相同 ⇒ 這個開關點不著，要換情境"


def test_search_never_worse_than_fixed(b):
    """⭐⭐ 值層級：同一張成本矩陣下，搜尋序的轉場總成本不可以比固定序高。

    `_best_sequence` 的候選集第一個就是固定序 ⇒ 這在結構上應成立；
    鎖它是為了擋「候選集被改掉」或「比較用了不同矩陣」。
    """
    ids = _ids(b)
    for n in range(3, len(ids) + 1):
        idxs = [b.cache.idx["A"][s] for s in ids[:n]]
        C = AL._task_cost_matrix(idxs, "A", b.cache)
        pos = {l: i for i, l in enumerate(idxs)}
        def cost(order):
            p = [pos[l] for l in order]
            return float(C[p[:-1], p[1:]].sum())
        c_sea = cost(AL.sweep_order(ids[:n], "A", b.cache, b.cfg))
        c_fix = cost(AL.sweep_order(ids[:n], "A", b.cache, None))
        assert c_sea <= c_fix + 1e-9, f"n={n}：搜尋序 {c_sea:.4f} 比固定序 {c_fix:.4f} 還差"


def test_no_cfg_is_bit_identical_to_old_behaviour(b):
    """⚠ 相容性：不給 `cfg` 時必須逐位元等於固定 skip-k 序（既有呼叫端不受影響）。"""
    ids = _ids(b)
    for n in range(1, len(ids) + 1):
        got = AL.sweep_order(ids[:n], "A", b.cache)
        want = [b.cache.idx["A"][ids[:n][p]] for p in _fixed_seq(b, n)]
        assert got == want, f"n={n}：{got} != {want}"


def test_legacy_mode_still_degrades_to_fixed(b):
    """⚠⚠ D32：`legacy_mode` 要能一鍵退回基準線演算法。

    `LEGACY_OVERRIDES` 把 `sweep_sequence` 設成 `"fixed"`；若 `sweep_order` 改成
    無條件搜尋，這個不變式就破了（而且**不會有任何徵狀**）。

    ⚠⚠⚠ 第一版拿 `sweep_order(..., cfg_legacy)` 去比 `sweep_order(..., None)` ——
    那是**兩個會一起動的東西**：把 cfg 檢查整個拿掉時兩邊同時變成搜尋序，測試照樣綠
    （突變實驗實測沒咬到）。所以這裡比的是**獨立算出來的**固定 skip-k 序。
    """
    ids = _ids(b)
    cfg_legacy = dataclasses.replace(b.cfg, sweep_sequence="fixed")
    for n in range(3, len(ids) + 1):
        want = [b.cache.idx["A"][ids[:n][q]] for q in _fixed_seq(b, n)]
        assert AL.sweep_order(ids[:n], "A", b.cache, cfg_legacy) == want, (
            f"n={n}：legacy 沒有退回固定序")


def test_task_options_endpoints_follow_the_searched_sequence(b):
    """⭐⭐ `_task_options` 的端點必須取自**搜尋**序 —— 這才是這個修正的實際出口。

    ⚠ 奇偶關係（`idx_last = idx_in if n 奇 else 1−idx_in`）只跟 n 有關、與順序無關，
    所以換序列不會動到它；會動的只有 first/last。
    """
    ids = _ids(b)
    n = len(ids)
    cell = b.subcell("PALL", ids)
    # ⚠ 用 `_task_of` 工廠而不是手搭 `_Task`：欄位加減時這條不會靜靜過期
    if not hasattr(AL, "_task_of"):
        pytest.fail("allocate._task_of 不在了，這條測試要重寫")
    task = AL._task_of(cell, "A", list(ids), cell.T_single_s["A"])
    order = AL.sweep_order(ids, "A", b.cache, b.cfg)
    opts = AL._task_options(task, b.cache, b.cfg)
    assert opts, "沒有任何可行選項 ⇒ 這條測不到東西"
    ends = {(order[0], order[-1]), (order[-1], order[0])}
    for first, idx_in, last, idx_last in opts:
        assert (first, last) in ends, f"端點 {(first, last)} 不是搜尋序的頭尾 {ends}"
        assert idx_last in (idx_in, 1 - idx_in)


def test_one_way_cell_uses_the_same_direction_transfer(b):
    """⭐⭐ 單向胞（D34）分支：兩條帶不能都雙向時，只能同向 racetrack ⇒ k = 3·idx。

    ⚠⚠⚠ 為什麼要手工造單向：mini fixture 的條帶**全部雙向**，所以
    `_task_cost_matrix` 的 else 分支在其他測試裡**一次都沒被走到** ——
    突變實驗把 `idx` 寫死成 0 時六條全綠（實測沒咬到）。
    這裡把 `op` 的一欄設成 inf 造出單向胞，讓那個分支真的被執行。
    """
    import copy
    cache = copy.copy(b.cache)
    cache.op = {k: v.copy() for k, v in b.cache.op.items()}
    ids = _ids(b)[:4]
    idxs = [cache.idx["A"][sid] for sid in ids]

    for forced_idx in (0, 1):
        for l in idxs:                       # 全部條帶只剩 forced_idx 這一向可行
            cache.op["A"][l, 1 - forced_idx] = math.inf
        C = AL._task_cost_matrix(idxs, "A", cache)
        tr = cache.tr["A"]
        a, bb = idxs[0], idxs[1]
        want = float(tr[a, bb, 3 * forced_idx])
        assert C[0, 1] == pytest.approx(want, abs=1e-9), (
            f"單向（只剩 idx={forced_idx}）時應取 k={3*forced_idx}，"
            f"實得 {C[0,1]:.6f} 而非 {want:.6f}")
        # 前置斷言：這一組**真的**走到了 else 分支（否則本條測不到東西）
        assert not bool(np.isfinite(cache.op["A"][a]).all()), "沒有造出單向胞"
        cache.op = {k: v.copy() for k, v in b.cache.op.items()}


def test_chain_deliberately_uses_the_fixed_order(b):
    """⚠⚠⚠ 鎖住「配置層**刻意**不用搜尋序」這個決定（2026-09-10 由封閉基準否決）。

    曾經把 `cfg` 傳進 `_task_options`，讓端點跟著 `exact_time` 的搜尋序走。
    單一情境（hills）好：makespan 5087.4 → 5037.6 s。
    但 `research/cpp_benchmark` 的 24 格否決了它：

    ===================  ==========
    格                   makespan Δ
    ===================  ==========
    HARVEST S5 n=16      −3.17%
    HARVEST S1 n=16      −0.51%
    HARVEST_mix S4 n=4   −2.19%
    HARVEST_mix S2 n=4   +1.09%
    HARVEST_mix S1 n=8   **+10.70%**
    HARVEST_mix S4 n=8   **+13.17%**
    ===================  ==========

    淨 **+731 s（更差）**，其餘 18 格 0.00%。

    ⚠ 這條是**決定的鎖**，不是能力的鎖：有人把 `cfg` 傳回去時要先看見這些數字，
    並重跑那 24 格。`sweep_order` 的 `cfg` 介面與成本矩陣的值層級鎖都保留著。
    """
    ids = _ids(b)
    cell = b.subcell("PCHAIN", ids)
    o_sea = AL.sweep_order(ids, "A", b.cache, b.cfg)
    o_fix = AL.sweep_order(ids, "A", b.cache, None)
    assert (o_sea[0], o_sea[-1]) != (o_fix[0], o_fix[-1]), (
        "搜尋序與固定序的端點相同 ⇒ 這條分不出走的是哪一個，要換 n")

    est = AL.plan_estimate([cell], "A1", b.cache, b.cfg)
    assert est.ok and est.per_task, "估計失敗 ⇒ 這條測不到東西"
    first, _idx_in, last, *_ = est.per_task[0]
    assert (first, last) in ((o_fix[0], o_fix[-1]), (o_fix[-1], o_fix[0])), (
        f"`_chain` 用了 {(first, last)}；目前的決定是端點取自**固定序** "
        f"{(o_fix[0], o_fix[-1])}，不是搜尋序 {(o_sea[0], o_sea[-1])}")
