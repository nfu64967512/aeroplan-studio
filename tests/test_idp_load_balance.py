"""IDP strip 分配的負載平衡測試。

背景
----
論文（Wang et al., Defence Technology 47 (2025) 197-215）的目標是
minmax dis_{i,k} —— 最小化最長那架的路徑（時間）。我們原本的
``_partition`` 只是把掃描線**依索引等量切塊**，再讓每架各自最小化自己的
子問題，因此 makespan 可能被某一架單獨拉高。

``balance_load=True``（預設）改成對**真實成本**搜尋：從等分切點出發，
窮舉「區塊 ↔ UAV」指派，再對邊界做爬山，只接受嚴格下降。

三種模式（`assignment=`）：
  index      = 最舊的等索引切分（僅供對照）
  contiguous = 連續切塊 + 邊界爬山，改善有限（makespan 0 ~ -0.8%）
  joint      = **論文 Sec 3.3 的聯合 MDTSP**（預設）：放棄連續限制，
               任一架可拿任一組條帶，用局部搜尋逼近 minmax。
               實測 makespan -0.4 ~ -3.9%，且自然浮現「跳躍式掃描」
               （例：UAV3 拿 [6,8,10]、UAV4 拿 [7,9,11] 交錯）。

三者都以前一級的解為起點且只接受嚴格下降 ⇒ **保證不會更差**，
這是這組測試真正要鎖住的性質。
"""

from __future__ import annotations

import math

import pytest

from core.dccpp.area_processor import AreaProcessor
from core.dccpp.idp_solver import IDP_Solver
from core.geometry.coordinate import CoordinateTransformer

_LAT, _LON = 25.0, 121.0


def _square(half_m: float):
    dlat = half_m / 111320.0
    dlon = half_m / (111320.0 * math.cos(math.radians(_LAT)))
    return [(_LAT - dlat, _LON - dlon), (_LAT - dlat, _LON + dlon),
            (_LAT + dlat, _LON + dlon), (_LAT + dlat, _LON - dlon)], dlat, dlon


def _setup(half_m: float, w_i: float, m: int, overlap: float = 0.2):
    poly, dlat, dlon = _square(half_m)
    ar = AreaProcessor().process_latlon(poly, w_i=w_i, w_ovl=w_i * overlap)
    tf = CoordinateTransformer(*ar.centroid_latlon)
    uavs = [{
        'uav_id': i + 1,
        'lat': _LAT - dlat + 2 * dlat * (i / max(1, m - 1)),
        'lon': _LON - dlon,
        'heading': (i * 47) % 360,
        'speed': 18.0,
    } for i in range(m)]
    return ar, tf, uavs


def _makespan(ar, tf, uavs, balance: bool = True, assignment: str = 'contiguous'):
    r = IDP_Solver(min_turn_radius=60.0, altitude=120.0, balance_load=balance,
                   assignment=assignment).solve(uavs, ar, tf)
    tot = [p.total_length_m for p in r.per_uav.values()]
    return max(tot), max(tot) - min(tot), r


CASES = [
    (1000.0, 138.6, 3),
    (1000.0, 138.6, 6),
    (2000.0, 200.0, 4),
    (2000.0, 150.0, 6),
]


@pytest.mark.parametrize("half,w_i,m", CASES)
def test_balancing_is_never_worse_than_index_split(half, w_i, m):
    """核心不變式：平衡後的 makespan 不得比等索引切分差。

    搜尋以舊解為起點、只接受嚴格下降，所以這在設計上就該成立；
    這條測試把它釘住，避免日後改搜尋策略時偷偷退步。
    """
    ar, tf, uavs = _setup(half, w_i, m)
    base, _, _ = _makespan(ar, tf, uavs, assignment='index')
    bal, _, _ = _makespan(ar, tf, uavs, assignment='contiguous')
    assert bal <= base + 1e-6, (
        f"平衡後 makespan {bal:.1f} 比等索引切分 {base:.1f} 還差"
    )


def test_balancing_strictly_helps_somewhere():
    """儀表能不能響：至少要有一個組態是**嚴格**改善的。

    若哪天平衡邏輯被改成 no-op，上面那條「不得更差」仍會全綠，
    這條才會抓到。
    """
    gains = []
    for half, w_i, m in CASES:
        ar, tf, uavs = _setup(half, w_i, m)
        base, _, _ = _makespan(ar, tf, uavs, assignment='index')
        bal, _, _ = _makespan(ar, tf, uavs, assignment='contiguous')
        gains.append(base - bal)
    assert max(gains) > 1.0, (
        f"沒有任何組態被改善（各組態改善量 {[round(g,2) for g in gains]}），"
        "平衡搜尋可能已失效"
    )


def test_every_scan_line_assigned_exactly_once():
    """平衡不能弄丟或重複條帶 —— 覆蓋完整性的前提。"""
    for half, w_i, m in CASES:
        ar, tf, uavs = _setup(half, w_i, m)
        _, _, r = _makespan(ar, tf, uavs, assignment='contiguous')
        seen = [sl.index for p in r.per_uav.values() for sl in p.ordered_scan_lines]
        assert sorted(seen) == list(range(len(ar.scan_lines))), (
            f"條帶分配有遺漏或重複：{sorted(seen)} vs {len(ar.scan_lines)} 條"
        )


def test_blocks_stay_contiguous():
    """保留連續切塊：每架拿到的條帶索引必須是連續區間。

    不連續會讓轉場飛越整個區域，違背這個設計選擇。
    """
    for half, w_i, m in CASES:
        ar, tf, uavs = _setup(half, w_i, m)
        _, _, r = _makespan(ar, tf, uavs, assignment='contiguous')
        for uid, p in r.per_uav.items():
            idx = sorted(sl.index for sl in p.ordered_scan_lines)
            if not idx:
                continue
            assert idx == list(range(idx[0], idx[0] + len(idx))), (
                f"UAV{uid} 拿到不連續的條帶 {idx}"
            )


def test_directions_length_matches_strips():
    for half, w_i, m in CASES:
        ar, tf, uavs = _setup(half, w_i, m)
        _, _, r = _makespan(ar, tf, uavs, assignment='contiguous')
        for p in r.per_uav.values():
            assert len(p.directions) == len(p.ordered_scan_lines)
            assert all(d in (-1, 1) for d in p.directions)


def test_speed_is_honoured_in_the_objective():
    """帶 speed 時目標應是**時間** —— 較慢的機體該少分一點。

    這裡把其中一架的速度砍半，它分到的作業長度不應該增加。
    """
    ar, tf, uavs = _setup(1000.0, 138.6, 3)
    fast = IDP_Solver(min_turn_radius=60.0, altitude=120.0,
                      balance_load=True).solve(uavs, ar, tf)
    slowed = [dict(u) for u in uavs]
    slowed[0]['speed'] = 6.0          # 1/3 速
    slow = IDP_Solver(min_turn_radius=60.0, altitude=120.0,
                      balance_load=True).solve(slowed, ar, tf)
    n_fast = len(fast.per_uav[1].ordered_scan_lines)
    n_slow = len(slow.per_uav[1].ordered_scan_lines)
    assert n_slow <= n_fast, (
        f"UAV1 變慢後反而分到更多條帶（{n_fast} -> {n_slow}）"
    )


# ══════════════════════════════════════════════════════════════════
#  聯合 MDTSP 指派（assignment='joint'，預設）
# ══════════════════════════════════════════════════════════════════
# 論文 Sec 3.3 把區域內所有作業路徑當「城市」、l_k 架 UAV 當 travellers 聯合
# 最佳化 minmax。我們的 'contiguous' 只在連續區塊內最佳化，'joint' 放棄連續
# 限制，用局部搜尋（以 contiguous 解為起點，只接受嚴格下降）。

@pytest.mark.parametrize("half,w_i,m", CASES)
def test_joint_is_never_worse_than_contiguous(half, w_i, m):
    """joint 以 contiguous 解為起點且只接受嚴格下降 ⇒ 不得更差。"""
    ar, tf, uavs = _setup(half, w_i, m)
    c, _, _ = _makespan(ar, tf, uavs, assignment='contiguous')
    j, _, _ = _makespan(ar, tf, uavs, assignment='joint')
    assert j <= c + 1e-6, f"joint makespan {j:.1f} 比 contiguous {c:.1f} 差"


def test_joint_strictly_beats_contiguous_somewhere():
    """儀表能不能響：至少一個組態要嚴格改善，否則 joint 等於沒接上。"""
    gains = []
    for half, w_i, m in CASES:
        ar, tf, uavs = _setup(half, w_i, m)
        c, _, _ = _makespan(ar, tf, uavs, assignment='contiguous')
        j, _, _ = _makespan(ar, tf, uavs, assignment='joint')
        gains.append(c - j)
    assert max(gains) > 1.0, f"joint 沒有改善任何組態：{[round(g,2) for g in gains]}"


def test_joint_actually_produces_non_contiguous_groups():
    """joint 的價值就在放棄連續 —— 至少要有一架拿到不連續的條帶。

    若哪天局部搜尋壞掉變成 no-op，上面兩條仍會綠（不更差、且有改善量可能
    來自別的組態），這條才會抓到「自由度根本沒被用到」。

    ⚠️⚠️ **2026-09-04 換組態**：原本用 ``(1000.0, 138.6, 6)``。Dubins 修好之後
    那個組態的 joint 改善變成 **0.00 m、不連續分組 0 組** —— 不是局部搜尋壞了，
    是**成本正確之後連續分組本來就是局部最佳**。四個 CASES 裡現在只剩
    ``(2000.0, 200.0, 4)`` 還會用到「放棄連續」這個自由度（改善 151.15 m）。

    ⇒ 這件事本身是個發現：joint 的價值有一部分是壞成本模型的產物。
      舊 Dubins 在小於 2R 的轉場灌水 1.6–1.9 倍、在別處回傳短於最佳解的長度，
      讓「跳著配」看起來划算。修好之後大多數情況相鄰條帶就是最便宜的。
    """
    ar, tf, uavs = _setup(2000.0, 200.0, 4)
    _, _, r = _makespan(ar, tf, uavs, assignment='joint')
    noncontig = 0
    for p in r.per_uav.values():
        idx = sorted(sl.index for sl in p.ordered_scan_lines)
        if idx and idx != list(range(idx[0], idx[0] + len(idx))):
            noncontig += 1
    assert noncontig > 0, "joint 產生的每一組都是連續的 —— 局部搜尋沒有生效"


@pytest.mark.parametrize("half,w_i,m", CASES)
def test_joint_assigns_every_strip_exactly_once(half, w_i, m):
    """放棄連續之後更要確認沒有條帶被漏掉或重複。"""
    ar, tf, uavs = _setup(half, w_i, m)
    _, _, r = _makespan(ar, tf, uavs, assignment='joint')
    seen = [sl.index for p in r.per_uav.values() for sl in p.ordered_scan_lines]
    assert sorted(seen) == list(range(len(ar.scan_lines))), (
        f"joint 條帶分配有遺漏或重複：{sorted(seen)}"
    )


def test_joint_respects_its_evaluation_budget():
    """預算設成 0 時必須立刻回起始解，不能無限搜尋或炸掉。"""
    ar, tf, uavs = _setup(1000.0, 138.6, 6)
    r = IDP_Solver(min_turn_radius=60.0, altitude=120.0,
                   assignment='joint', max_joint_evals=0).solve(uavs, ar, tf)
    seen = [sl.index for p in r.per_uav.values() for sl in p.ordered_scan_lines]
    assert sorted(seen) == list(range(len(ar.scan_lines)))


def test_unknown_assignment_mode_is_rejected():
    with pytest.raises(ValueError, match="joint/contiguous/index"):
        IDP_Solver(min_turn_radius=60.0, assignment='nonsense')


# ── 速度不一致：只有「搬移」動作能改變條帶**數量** ──────────────
# 2026-08-24 刪除實驗發現：在等速組態下把 relocate 關掉，所有測試仍全綠
#（swap 就夠了，因為等長條帶 + 相近進場成本讓數量分配已是最佳）。
# 速度不同時最佳解的數量分配才會偏離等分，這時非 relocate 不可。

_UNEVEN_SPEEDS = [18.0, 4.5, 18.0]


def _setup_uneven_speed():
    ar, tf, uavs = _setup(1000.0, 138.6, 3)
    for u, sp in zip(uavs, _UNEVEN_SPEEDS):
        u['speed'] = sp
    return ar, tf, uavs


def _makespan_time(ar, tf, uavs, assignment):
    r = IDP_Solver(min_turn_radius=60.0, altitude=120.0,
                   assignment=assignment).solve(uavs, ar, tf)
    times = [p.total_length_m / _UNEVEN_SPEEDS[u - 1]
             for u, p in sorted(r.per_uav.items())]
    counts = [len(r.per_uav[u].ordered_scan_lines) for u in sorted(r.per_uav)]
    return max(times), counts, r


def test_joint_beats_contiguous_when_speeds_differ():
    """速度不同時 joint 要明顯更好 —— 這條需要 relocate 才過得了。"""
    ar, tf, uavs = _setup_uneven_speed()
    c, _, _ = _makespan_time(ar, tf, uavs, 'contiguous')
    j, _, _ = _makespan_time(ar, tf, uavs, 'joint')
    assert j < c - 1.0, f"joint {j:.1f}s 沒有勝過 contiguous {c:.1f}s"


def test_joint_changes_strip_counts_when_speeds_differ():
    """joint 必須能改變**條帶數量**分配，不能只會等量交換。

    swap 保持數量不變，只有 relocate 會動數量。若 relocate 被關掉或壞掉，
    joint 的數量分配會與 contiguous 完全相同，這條就會紅。
    """
    ar, tf, uavs = _setup_uneven_speed()
    _, c_counts, _ = _makespan_time(ar, tf, uavs, 'contiguous')
    _, j_counts, _ = _makespan_time(ar, tf, uavs, 'joint')
    assert j_counts != c_counts, (
        f"joint 的條帶數量分配與 contiguous 相同（都是 {c_counts}）—— "
        "搬移動作可能沒有生效"
    )


def test_slow_uav_gets_fewer_strips_than_the_fast_ones():
    """1/4 速的那架分到的條帶數必須少於兩架全速的。"""
    ar, tf, uavs = _setup_uneven_speed()
    _, counts, _ = _makespan_time(ar, tf, uavs, 'joint')
    slow, fast1, fast2 = counts[1], counts[0], counts[2]
    assert slow < fast1 and slow < fast2, (
        f"慢速機分到 {slow} 條，快速機 {fast1}/{fast2} 條 —— 速度沒被納入目標"
    )
