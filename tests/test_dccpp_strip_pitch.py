"""條帶間距（pitch）與重疊率的值層級鎖。

背景
----
2026-08-24 對照論文（Wang et al., Defence Technology 47 (2025) 197-215）發現：
``generate_paper_scan_lines`` 照抄了 Eq.(7) 印出的 ``ΔwB_l = l·w_i + (l-1)·w_ovl``，
而那個 ``+`` 與「條帶寬度恆為 w_i」自相矛盾。實際後果是**重疊率完全失效**：

    w_i=100, w_ovl=20  ⇒ 應有中線間距 80，實測 100
    w_i=100, w_ovl=40  ⇒ 應有中線間距 60，實測 100

因為中線 = (ΔwT + ΔwB)/2 時，w_ovl 項剛好對消。使用者在 UI 上調重疊率，
規劃結果**一個字都不會變** —— 正是本專案反覆踩到的那類靜默失效。

這組測試鎖在**值**上（實際產生的掃描線間距），不是鎖在公式長相上，
所以不論日後是誰、從哪條路徑改壞了條帶排布，都會紅。
"""

from __future__ import annotations

import math

import pytest

from core.dccpp.area_processor import AreaProcessor


def _square_polygon(half_m: float = 1000.0, lat: float = 25.0, lon: float = 121.0):
    dlat = half_m / 111320.0
    dlon = half_m / (111320.0 * math.cos(math.radians(lat)))
    return [(lat - dlat, lon - dlon), (lat - dlat, lon + dlon),
            (lat + dlat, lon + dlon), (lat + dlat, lon - dlon)]


def _mid_offsets(w_i: float, w_ovl: float):
    """實際產生的掃描線中線位移序列。"""
    res = AreaProcessor().process_latlon(_square_polygon(), w_i=w_i, w_ovl=w_ovl)
    return [(s.delta_w_top + s.delta_w_bot) / 2.0 for s in res.scan_lines], res


@pytest.mark.parametrize("w_i,w_ovl", [
    (100.0, 0.0),
    (100.0, 20.0),
    (100.0, 40.0),
    (138.6, 27.72),      # 面板預設：20% 重疊
    (200.0, 100.0),      # 50% 重疊
])
def test_strip_pitch_equals_w_i_minus_w_ovl(w_i, w_ovl):
    """中線間距必須等於 w_i - w_ovl。這是重疊率有沒有生效的唯一判準。"""
    mids, _ = _mid_offsets(w_i, w_ovl)
    assert len(mids) >= 3, f"掃描線太少（{len(mids)}），測不出間距"
    gaps = [b - a for a, b in zip(mids, mids[1:])]
    expected = w_i - w_ovl
    for g in gaps:
        assert g == pytest.approx(expected, rel=1e-6), (
            f"w_i={w_i}, w_ovl={w_ovl}：中線間距應為 {expected}，實際 {g}"
        )


@pytest.mark.parametrize("w_i,w_ovl", [(100.0, 20.0), (138.6, 27.72), (200.0, 100.0)])
def test_strip_width_is_constant(w_i, w_ovl):
    """每條條帶的寬度（ΔwB - ΔwT）必須恆等於 w_i，不能隨 l 變寬。

    照抄 Eq.(7) 的 '+' 會讓寬度變成 w_i + 2(l-1)·w_ovl，一路變寬。
    """
    _, res = _mid_offsets(w_i, w_ovl)
    widths = [s.delta_w_bot - s.delta_w_top for s in res.scan_lines]
    for w in widths:
        assert w == pytest.approx(w_i, rel=1e-6), (
            f"條帶寬度應恆為 {w_i}，實際出現 {w}（w_ovl={w_ovl}）"
        )


def test_overlap_actually_changes_the_plan():
    """刪除實驗：把重疊率調大，掃描線數量必須增加。

    這條是整組測試的「儀表能不能響」自檢 —— 舊實作在這裡會完全相同，
    因為 w_ovl 被對消掉了。
    """
    _, r0 = _mid_offsets(100.0, 0.0)
    _, r20 = _mid_offsets(100.0, 20.0)
    _, r50 = _mid_offsets(100.0, 50.0)
    n0, n20, n50 = len(r0.scan_lines), len(r20.scan_lines), len(r50.scan_lines)
    assert n20 > n0, f"重疊率 20% 沒有讓條帶變密（{n0} -> {n20}）"
    assert n50 > n20, f"重疊率 50% 沒有讓條帶更密（{n20} -> {n50}）"


def test_zero_overlap_pitch_equals_fov_width():
    """重疊率 0 時 pitch 就是 w_i —— 邊界情況不能被上面的修正弄壞。"""
    mids, _ = _mid_offsets(120.0, 0.0)
    gaps = [b - a for a, b in zip(mids, mids[1:])]
    assert all(g == pytest.approx(120.0, rel=1e-6) for g in gaps)


def test_strips_span_the_area_width():
    """條帶總覆蓋必須蓋滿區域最小寬度，否則邊緣會整條漏掉。"""
    for w_i, w_ovl in ((100.0, 20.0), (138.6, 27.72)):
        _, res = _mid_offsets(w_i, w_ovl)
        W = res.min_width_result.min_width
        pitch = w_i - w_ovl
        n = len(res.scan_lines)
        span = (n - 1) * pitch + w_i
        assert span >= W - 1e-6, (
            f"w_i={w_i}, w_ovl={w_ovl}：{n} 條條帶只覆蓋 {span:.1f} m，"
            f"區域寬度 {W:.1f} m"
        )


def test_panel_convention_is_unaffected():
    """VTOL 偵察面板把重疊折進 fov_width_m 並送 overlap_rate=0，
    修正前後結果都應該一致（pitch = 送進來的值）。"""
    pitch = 110.9
    mids, _ = _mid_offsets(pitch, 0.0)
    gaps = [b - a for a, b in zip(mids, mids[1:])]
    assert all(g == pytest.approx(pitch, rel=1e-6) for g in gaps)


# ══════════════════════════════════════════════════════════════════
#  最後一條條帶不可被靜默丟掉
# ══════════════════════════════════════════════════════════════════
# 2026-08-24：最後一條的帶通常超出區域，若中線取幾何中心會落到多邊形外，
# _seg_polygon_intersections 找不到交點 ⇒ `continue` 把整條靜默丟掉，
# 遠緣留下最寬達 w_i/2 的漏掃帶。實測 2.0x1.5 km 區域覆蓋率卡在 97.0%。

@pytest.mark.parametrize("half_m,w_i", [
    (750.0, 110.9),     # 實際踩到的組態（W=1492.4 不是 pitch 的整數倍）
    (750.0, 137.0),
    (500.0, 91.0),
    (1000.0, 173.0),
])
def test_last_strip_is_not_dropped(half_m, w_i):
    """條帶的橫向覆蓋必須到達區域遠緣，不能少一條。"""
    from core.dccpp.area_processor import AreaProcessor
    poly = _square_polygon(half_m=half_m)
    res = AreaProcessor().process_latlon(poly, w_i=w_i, w_ovl=0.0)
    W = res.min_width_result.min_width
    n = len(res.scan_lines)
    # 需要幾條才蓋得住
    need = max(1, math.ceil((W - w_i) / w_i) + 1)
    assert n >= need, (
        f"W={W:.1f}, w_i={w_i} 需要 {need} 條，實際只產生 {n} 條"
        "（最後一條可能因中線落在區域外被靜默丟掉）"
    )


@pytest.mark.parametrize("half_m,w_i", [(750.0, 110.9), (500.0, 91.0)])
def test_generated_strips_actually_cover_the_area(half_m, w_i):
    """值層級：把掃描線當航跡丟進覆蓋率驗算，必須接近 100%。

    這條直接量「規劃出來的東西到底蓋不蓋得住」，不依賴任何內部公式，
    所以不論條帶數、中線位置怎麼改壞，都會紅。
    """
    from core.dccpp.area_processor import AreaProcessor
    from core.recon import verify_coverage
    poly = _square_polygon(half_m=half_m)
    res = AreaProcessor().process_latlon(poly, w_i=w_i, w_ovl=0.0)
    paths = {}
    for i, sl in enumerate(res.scan_lines):
        assert sl.pL_latlon and sl.pR_latlon, "掃描線缺 lat/lon 端點"
        paths[i] = [sl.pL_latlon, sl.pR_latlon]
    rep = verify_coverage(paths, poly, sweep_width_m=w_i, cell_size_m=15.0)
    assert rep.coverage_pct >= 99.5, (
        f"w_i={w_i} 的條帶只覆蓋 {rep.coverage_pct:.2f}%（漏 {len(rep.missed_points)} 格）"
    )


# ══════════════════════════════════════════════════════════════════
#  pitch 修正帶進來的兩個新守衛，各自要有能響的測試
# ══════════════════════════════════════════════════════════════════
# 2026-09-10：把 pitch 修成 w_i - w_ovl 之後冒出兩個新的靜默失效面，
# 兩者都**不在**上面任何一條的覆蓋範圍內（突變實驗 M7 / M9 全綠溜過）：
#   1. W - w_i 剛好是 pitch 整數倍時，多算一條 ⇒ 末條 clamp 把它拉回去，
#      產生一條與前一條**完全重合**的掃描線（多飛一趟、gap=0）。
#   2. 重疊率拉高後條帶數真的會撞到 _MAX_SCAN_LINES；靜默截斷等於交出
#      一份蓋不滿區域的規劃。
# lat/lon 推出來的 W 永遠是零碎值，撞不到整數倍，所以第 1 條直接餵公尺凸包。


def test_no_duplicate_scan_line_at_exact_multiple_width():
    """W - w_i 是 pitch 整數倍時不可產生重合的掃描線。

    這條鎖住 n_layers 的浮點容差方向：容差取反（+eps）會多出第 7 條，
    被末條 clamp 拉回 W - w_i 而與第 6 條重合（中線間距 0）。
    """
    from core.dccpp.area_processor import find_min_width, generate_paper_scan_lines
    hull = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 600.0), (0.0, 600.0)]
    mwr = find_min_width(hull)
    assert mwr.min_width == pytest.approx(600.0), "前置條件：W 必須正好是 600"
    w_i, w_ovl = 100.0, 0.0
    # 前置斷言：這個組態真的落在「整數倍」這個會出事的點上
    assert (mwr.min_width - w_i) % (w_i - w_ovl) == pytest.approx(0.0)

    sl = generate_paper_scan_lines(hull, mwr, w_i=w_i, w_ovl=w_ovl)
    mids = [(s.delta_w_top + s.delta_w_bot) / 2.0 for s in sl]
    gaps = [b - a for a, b in zip(mids, mids[1:])]
    assert gaps, f"掃描線太少（{len(sl)}）"
    assert min(gaps) > 1e-6, (
        f"出現重合/反序的掃描線：中線 {mids}（最小間距 {min(gaps)}）"
    )
    assert len(sl) == 6, f"600 m 寬 / 100 m 條帶應為 6 條，實際 {len(sl)}"


def test_excessive_overlap_is_rejected_not_silently_truncated():
    """條帶數撞到上限時必須拋錯，不可靜默截斷成蓋不滿的規劃。"""
    from core.dccpp.area_processor import _MAX_SCAN_LINES

    # 前置斷言：這個組態真的需要超過上限的條帶數（否則這條測試量不到東西）
    poly = _square_polygon(half_m=1000.0)
    w_i, w_ovl = 37.0, 36.63
    W = AreaProcessor().process_latlon(poly, w_i=w_i, w_ovl=0.0).min_width_result.min_width
    need = math.ceil((W - w_i) / (w_i - w_ovl)) + 1
    assert need > _MAX_SCAN_LINES, (
        f"前置條件不成立：只需 {need} 條，沒有超過上限 {_MAX_SCAN_LINES}"
    )

    with pytest.raises(ValueError, match=r"掃描線"):
        AreaProcessor().process_latlon(poly, w_i=w_i, w_ovl=w_ovl)

    # 反面：合法組態不能被這個守衛誤殺
    ok = AreaProcessor().process_latlon(poly, w_i=100.0, w_ovl=50.0)
    assert 0 < len(ok.scan_lines) <= _MAX_SCAN_LINES


@pytest.mark.parametrize("w_i,w_ovl", [
    (100.0, 0.0), (100.0, 20.0), (138.6, 27.72), (200.0, 100.0),
    # 下面兩組會觸發「末條往內拉」的 clamp 分支 —— 那條分支同樣要誠實回報
    (100.0, 10.0), (138.6, 0.0),
])
def test_reported_offsets_match_the_actual_line(w_i, w_ovl):
    """回報的 ΔwT/ΔwB 必須真的描述掃描線的位置。

    ⚠ 上面所有 pitch/寬度的判準都是讀 ``delta_w_top``/``delta_w_bot``。
    如果這兩個欄位與實際產生的端點脫鉤，那整組鎖就是在量一個與飛行無關的
    數字 —— 量儀本身壞掉而且一定是綠的。這條把回報值釘回實際幾何：
    把端點投影到 nb_min 上，位移必須等於 (ΔwT + ΔwB)/2。
    """
    _, res = _mid_offsets(w_i, w_ovl)
    mwr = res.min_width_result
    import numpy as np
    v2 = np.asarray(mwr.v2, dtype=float)
    nb = np.asarray(mwr.nb_min, dtype=float)
    assert res.scan_lines, "沒有掃描線"
    for s in res.scan_lines:
        reported = (s.delta_w_top + s.delta_w_bot) / 2.0
        for tag, p in (("pL", s.pL_xy), ("pR", s.pR_xy)):
            actual = float(np.dot(np.asarray(p, dtype=float) - v2, nb))
            assert actual == pytest.approx(reported, abs=1e-6), (
                f"條帶 {s.index} 的 {tag} 實際位移 {actual:.6f} m "
                f"與回報的 {reported:.6f} m 不符（w_i={w_i}, w_ovl={w_ovl}）"
            )
