# -*- coding: utf-8 -*-
"""SAR 熱力圖解析度的值層級守衛（2026-08-25）。

使用者回報「熱力圖方塊都是一大格的」。量下去:2.8 km 區域 × 解析度 20
⇒ 格子 **139×141 m**,而掃描寬度只有 **50 m** —— 格子是感測器足跡的 2.8 倍。

⚠️⚠️ 這不是渲染不夠細,是**取樣不足**:JS 的覆蓋判定是
`_pointInConvexQuad(cell.latCenter, cell.lonCenter, ...)` —— 只看格子**中心點**。
格子比足跡大時,一格會整格翻或整格不翻,所以覆蓋率會**同時**高估
（掃到一角就整格算掃過）與低估（擦過中心以外完全不算）。調顏色修不好。

⚠️ 而控制項寫的是「網格解析度: 20」—— 一個沒有單位的數字。格子邊長由
**區域跨距 ÷ 行列數**決定,而區域是使用者在地圖上畫的,面板看不到它
⇒ 使用者沒有任何線索知道 20 代表多大。這是本則要修的東西。
"""

import pytest

from ui.main_window import (HEATMAP_M_PER_DEG, heatmap_cell_size_m,
                            heatmap_resolution_note)


# 使用者畫的偵察區量級
AREA = (25.070, 25.095, 121.215, 121.243)      # 約 2.8 x 2.8 km @ lat 25


def test_metres_per_degree_matches_the_javascript():
    """⚠⚠ 必須和 main.html 的 mPerDegLat 是**同一個常數**。

    JS 用它算 cellWidthM 再推 normalizedW = 掃描寬度 / 格子寬。面板若用別的
    常數,顯示的「格子邊長」就不是 JS 實際採用的那個 —— 兩個數字各自都對,
    合起來是錯的。本專案已有前例（兩套每度公尺常數讓生成點間距差
    0.000229 m 而卡在避撞門檻上）。
    """
    import pathlib
    import re
    js = pathlib.Path('ui/widgets/cesium_templates/main.html').read_text(
        encoding='utf-8')
    m = re.search(r'mPerDegLat\s*=\s*([0-9.]+)', js)
    assert m, "main.html 找不到 mPerDegLat ⇒ 這條守衛失效（可能被改名了）"
    assert float(m.group(1)) == pytest.approx(HEATMAP_M_PER_DEG), (
        f"JS 用 {m.group(1)}、Python 用 {HEATMAP_M_PER_DEG} ⇒ 顯示值與實際不符")


def test_cell_size_scales_inversely_with_grid():
    """解析度加倍 ⇒ 格子邊長減半。這是使用者唯一在意的因果。"""
    ns20, ew20 = heatmap_cell_size_m(*AREA, 20)
    ns40, ew40 = heatmap_cell_size_m(*AREA, 40)
    assert ns40 == pytest.approx(ns20 / 2, rel=1e-9)
    assert ew40 == pytest.approx(ew20 / 2, rel=1e-9)


def test_reproduces_the_reported_symptom():
    """⚠ 前提自檢:使用者回報的那個組態,算出來真的是大格子。

    沒有這條的話,下面的警告測試可能在測一個**不會發生**的情境。
    """
    ns, ew = heatmap_cell_size_m(*AREA, 20)
    assert ns > 100 and ew > 100, f"重現不出大格子({ns:.0f}x{ew:.0f}m)"
    _note, ok = heatmap_resolution_note(ns, ew, 50.0, 20)
    assert not ok, "使用者實際踩到的組態卻沒有被警告"


def test_undersampling_is_warned_with_a_concrete_target():
    """格子比掃描寬度大就要警告,而且要給**一個可以照做的數字**。

    只說「太粗」沒有用 —— 使用者不知道該調到多少。
    """
    ns, ew = heatmap_cell_size_m(*AREA, 20)
    note, ok = heatmap_resolution_note(ns, ew, 50.0, 20)
    assert not ok
    assert '⚠' in note
    # 建議值必須真的能解決問題
    import re
    m = re.search(r'解析度需 ≈ (\d+)', note)
    assert m, f"沒有給出建議解析度：{note}"
    suggested = int(m.group(1))
    ns2, ew2 = heatmap_cell_size_m(*AREA, suggested)
    _n2, ok2 = heatmap_resolution_note(ns2, ew2, 50.0, suggested)
    assert ok2, (
        f"照建議調到 {suggested} 之後**仍然**被警告 ⇒ 建議值是錯的"
        f"（{ns2:.0f}x{ew2:.0f}m vs 掃描寬度 50m）")


def test_adequate_resolution_is_not_warned():
    """已知合法組態不可以誤報 —— 否則警告會被當雜訊忽略。

    ⚠ 斷言要挑**那一種**警告,不能抓泛用的「⚠」:這串現在可能同時帶
    「取樣不足」與「重繪成本」兩種警告,用 ⚠ 一把抓會把成本提醒誤判成
    取樣警告(我第一版就這樣紅了)。斷言用的字串本身要能被斷言分開。
    """
    ns, ew = heatmap_cell_size_m(*AREA, 100)
    note, ok = heatmap_resolution_note(ns, ew, 50.0, 100)
    assert ok, f"28m 格子配 50m 掃描寬度不該被警告：{note}"
    assert '格子比掃描寬度' not in note, f"取樣充足卻報取樣不足：{note}"


def test_note_always_states_the_cell_size_in_metres():
    """不論警不警告,都要講出格子是幾公尺 —— 那是使用者唯一看得懂的量。"""
    for grid in (5, 20, 100):
        ns, ew = heatmap_cell_size_m(*AREA, grid)
        note, _ok = heatmap_resolution_note(ns, ew, 50.0, grid)
        assert 'm／格' in note
        assert f'{grid}×{grid}' in note


def test_cell_count_is_reported_too():
    """格數要講出來 —— 那是效能成本的直接指標（每格一個 Cesium entity）。"""
    ns, ew = heatmap_cell_size_m(*AREA, 100)
    note, _ = heatmap_resolution_note(ns, ew, 50.0, 100)
    assert '10,000' in note


def test_zero_sweep_width_does_not_crash_or_warn():
    """掃描寬度為 0（尚未設定）時不可以除以零,也不該亂警告。"""
    ns, ew = heatmap_cell_size_m(*AREA, 20)
    note, ok = heatmap_resolution_note(ns, ew, 0.0, 20)
    assert ok and note


def test_degenerate_area_does_not_crash():
    """區域退化成一點（角點重合）時不可以炸掉。"""
    ns, ew = heatmap_cell_size_m(25.0, 25.0, 121.0, 121.0, 20)
    assert ns == 0.0 and ew == 0.0


def test_grid_zero_is_clamped_not_divide_by_zero():
    heatmap_cell_size_m(*AREA, 0)


# ══════════════════════════════════════════════════════════════
# 非正方區域:南北/東西必須**都**看
# ══════════════════════════════════════════════════════════════

# 東西向遠長於南北向：EW 格子 ~5x NS 格子
WIDE_AREA = (25.070, 25.0745, 121.215, 121.275)


def test_wide_area_really_is_lopsided():
    """前提自檢:這塊區域的東西向格子必須明顯大於南北向。

    ⚠ 沒有這條的話,下面那條會在一個「近乎正方」的區域上測 —— 那時漏掉
    東西向也不會被抓到(實測:原本的 2.8km 區域 139 vs 141,突變臂 M6
    『只比南北邊長』完全存活)。
    """
    ns, ew = heatmap_cell_size_m(*WIDE_AREA, 20)
    assert ew > ns * 3, f"區域不夠扁({ns:.0f} vs {ew:.0f})⇒ 測不到漏看東西向"


def test_undersampling_in_the_east_west_direction_is_caught():
    """⚠ 只有東西向格子過大時,一樣要警告。

    覆蓋判定看的是格子中心點,南北或東西**任一**方向過粗都會造成整格翻。
    只比其中一邊 = 漏掉一半的情況。
    """
    ns, ew = heatmap_cell_size_m(*WIDE_AREA, 20)
    assert ns < 50.0 < ew, f"這個 fixture 沒構成「只有東西向過大」({ns:.0f}/{ew:.0f})"
    _note, ok = heatmap_resolution_note(ns, ew, 50.0, 20)
    assert not ok, "東西向格子超過掃描寬度卻沒被警告"


# ══════════════════════════════════════════════════════════════
# 面板:回填管道要真的把字寫上去
# ══════════════════════════════════════════════════════════════

def test_panel_displays_the_cell_info():
    """⚠ 面板的回填方法必須真的改到標籤。

    這條擋的是「方法在、但什麼都沒做」—— 那樣 main_window 算得再對,
    使用者還是看不到任何東西,而且不會有錯誤。
    """
    from PyQt6.QtWidgets import QApplication

    from ui.widgets.parameter_panel import ParameterPanel
    _app = QApplication.instance() or QApplication([])
    panel = ParameterPanel()
    panel.set_heatmap_cell_info('139 × 141 m／格（20×20 = 400 格）', ok=False)
    assert '139' in panel._heatmap_cell_label.text()
    assert '400' in panel._heatmap_cell_label.text()
    panel.set_heatmap_cell_info('28 × 28 m／格', ok=True)
    assert '28' in panel._heatmap_cell_label.text()


def test_panel_grid_spinbox_can_reach_a_useful_resolution():
    """前提自檢:面板真的調得到「格子 ≤ 掃描寬度」的解析度。

    ⚠ 若上限太低,就算警告講得再清楚,使用者也**照做不了** ——
    那時該修的是上限,不是警告。
    """
    from PyQt6.QtWidgets import QApplication

    from ui.widgets.parameter_panel import ParameterPanel
    _app = QApplication.instance() or QApplication([])
    panel = ParameterPanel()
    hi = panel._heatmap_grid_spin.maximum()
    ns, ew = heatmap_cell_size_m(*AREA, hi)
    _n, ok = heatmap_resolution_note(ns, ew, 50.0, hi)
    assert ok, (
        f"網格上限 {hi} 在 2.8km 區域只能到 {max(ns, ew):.0f} m／格，"
        f"仍粗於掃描寬度 50 m ⇒ 使用者照著警告調也解決不了")


def test_large_grids_warn_about_redraw_cost():
    """⚠ 格數多的成本是**超線性**的:每格一個 Cesium Entity,而改色會讓
    整批 primitive 重建(不走 per-instance 顏色屬性路徑)。

    使用者有權在按下去之前就知道,而不是飛到一半才發現卡頓。
    """
    from ui.main_window import HEATMAP_HEAVY_CELLS
    g = int(HEATMAP_HEAVY_CELLS ** 0.5) + 5
    ns, ew = heatmap_cell_size_m(*AREA, g)
    note, _ok = heatmap_resolution_note(ns, ew, 50.0, g)
    assert '重繪' in note, f"{g}×{g} = {g*g} 格卻沒有提醒成本：{note}"


def test_small_grids_do_not_warn_about_cost():
    """已知輕量的組態不可以誤報。"""
    from ui.main_window import HEATMAP_HEAVY_CELLS
    g = int(HEATMAP_HEAVY_CELLS ** 0.5) - 5
    ns, ew = heatmap_cell_size_m(*AREA, g)
    note, _ok = heatmap_resolution_note(ns, ew, 50.0, g)
    assert '重繪' not in note
