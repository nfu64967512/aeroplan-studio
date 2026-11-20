"""禁航區把自動切出來的子區吃穿 —— 「畫一大區、系統自動切」唯一已量到的靜默失效。

使用者 2026-09-08 定調走「一大區、系統分小區」這條路。這個檔守住那條路上唯一
會**安靜出錯**的地方：禁航區在掃描步進軸方向太寬時，子區被吃穿 ⇒ 整胞
``no_feasible_class``，而 ``status`` 仍是 ``GO_WITH_WARNINGS``、``errors`` 仍是空的。

閉式門檻：``禁航區跨度 S ≥ 子區寬 W − 刈幅``。
1560×800 m 切 4 塊 ⇒ W = 200 m、刈幅 138.56 m ⇒ **61.4 m**。

實測（生產路徑，先前航次）：

===== ================== =========
跨度  makespan           上工台數
===== ================== =========
40 m  1505 s             4/4
50 m  1538 s             4/4
60 m  1540 s             4/4
70 m  2463 s（**+60%**） 4/4
100 m —                  **1/4**
150 m —                  **1/4**
===== ================== =========

⇒ 翻轉點在 60–70 m，與閉式吻合。守衛必須在 60 安靜、在 70 叫。
"""

from __future__ import annotations

import math

import pytest

from mission.swarm_coordinator import nfz_defeats_fleet_split, split_area_for_fleet

pytest.importorskip("shapely")

LAT0, LON0 = 23.95, 120.90
M_LAT = 111132.0
M_LON = 111412.0 * math.cos(math.radians(LAT0))
SWATH = 2 * 120.0 * math.tan(math.radians(60.0) / 2)      # 120 m AGL、HFOV 60° ⇒ 138.56 m


def _rect(cx, cy, half_e, half_n):
    return [(LAT0 + (cy - half_n) / M_LAT, LON0 + (cx - half_e) / M_LON),
            (LAT0 + (cy - half_n) / M_LAT, LON0 + (cx + half_e) / M_LON),
            (LAT0 + (cy + half_n) / M_LAT, LON0 + (cx + half_e) / M_LON),
            (LAT0 + (cy + half_n) / M_LAT, LON0 + (cx - half_e) / M_LON)]


class _Zone:
    """最小的 FenceZone 替身（`_zone_polygon_latlon` 只讀 `vertices`）。"""

    def __init__(self, verts):
        self.vertices = verts


def _subareas(n=4):
    mother = [{"area_id": 1, "priority": 1.0, "polygon": _rect(0, 0, 780, 400)}]
    return split_area_for_fleet(mother, n, min_piece_width_m=SWATH)


def _zone(span_m, *, cx=-400.0, cy=100.0, half_e=75.0):
    return _Zone(_rect(cx, cy, half_e, span_m / 2.0))


# ── 前置斷言：門檻算在真的幾何上 ────────────────────────────────────────

def test_the_geometry_really_produces_200m_subareas():
    """⚠ 少了這一條，下面每個門檻數字都可能是算在想像的幾何上。"""
    subs = _subareas(4)
    assert len(subs) == 4
    widths = sorted((max(p[0] for p in a["polygon"]) - min(p[0] for p in a["polygon"])) * M_LAT
                    for a in subs)
    assert widths[0] == pytest.approx(200.0, abs=1.0), widths
    assert SWATH == pytest.approx(138.56, abs=0.01)
    assert 200.0 - SWATH == pytest.approx(61.44, abs=0.05)     # 閉式門檻


# ── 守衛必須咬在實測翻轉點上 ────────────────────────────────────────────

@pytest.mark.parametrize("span", [40.0, 50.0, 60.0])
def test_silent_below_the_measured_flip(span):
    """60 m 以下實測是 4/4 台、makespan 沒有異常 ⇒ 守衛不可以叫（假警報也是缺陷）。"""
    assert nfz_defeats_fleet_split(_subareas(4), [_zone(span)], swath_m=SWATH) == []


@pytest.mark.parametrize("span,measured", [(70.0, "makespan +60%"),
                                           (100.0, "只剩 1/4 台"),
                                           (150.0, "只剩 1/4 台")])
def test_fires_above_the_measured_flip(span, measured):
    hits = nfz_defeats_fleet_split(_subareas(4), [_zone(span)], swath_m=SWATH)
    assert hits, f"跨度 {span} m 實測 {measured}，守衛卻沒叫"
    h = hits[0]
    assert h["span_m"] >= h["budget_m"]
    assert h["width_m"] == pytest.approx(200.0, abs=1.0)
    assert h["budget_m"] == pytest.approx(61.44, abs=1.0)


# ── 反向臂：沒有禁航區、不重疊、參數缺席時都不可以叫 ──────────────────────

def test_no_zones_no_warning():
    assert nfz_defeats_fleet_split(_subareas(4), [], swath_m=SWATH) == []


def test_zone_outside_the_area_is_ignored():
    """禁航區在作業區外 5 km ⇒ 與規劃無關，不可以叫。"""
    far = _Zone(_rect(6000.0, 0.0, 500.0, 500.0))
    assert nfz_defeats_fleet_split(_subareas(4), [far], swath_m=SWATH) == []


def test_zone_straddling_the_edge_counts_only_the_overlapping_part():
    """⭐ 禁航區大半在作業區**外**時，只有伸進來的那一截算數。

    ⚠⚠ 這條是突變實驗補出來的缺口：先前每個測試的禁航區都**完全**落在區內，
    於是「量整個禁航區」與「量交集」給一樣的答案 ⇒ 把 `zs.intersection(shape)`
    改成 `zs` 的突變**咬不到**。這裡放一個南北 500 m 的大禁航區，但只有 30 m
    伸進母區 ⇒ 量交集要安靜（30 < 61.4），量整個區會誤報（500 ≥ 61.4）。
    """
    subs = _subareas(4)
    straddler = _Zone(_rect(-400.0, 620.0, 75.0, 250.0))    # y 從 370 到 870，母區到 400
    assert nfz_defeats_fleet_split(subs, [straddler], swath_m=SWATH) == [], \
        "只伸進 30 m 的禁航區不該觸發（量到整個 500 m 就會誤報）"

    # 前置斷言：它**真的**有碰到母區，否則這條測的是「不重疊」而不是「量交集」
    deep = _Zone(_rect(-400.0, 500.0, 75.0, 250.0))         # y 從 250 到 750，伸進 150 m
    assert nfz_defeats_fleet_split(subs, [deep], swath_m=SWATH), \
        "同樣位置但伸進 150 m 就必須叫，否則上面那條是因為不重疊才安靜"


def test_zero_swath_is_inert():
    """刈幅算不出來時（hfov 沒給）不要亂叫 —— 那時門檻無意義。"""
    assert nfz_defeats_fleet_split(_subareas(4), [_zone(150.0)], swath_m=0.0) == []


# ── 門檻要跟著機隊台數走（子區越窄，容得下的禁航區越小）──────────────────

def test_threshold_tightens_as_the_fleet_grows():
    """2 台 ⇒ 子區 400 m ⇒ 門檻 261 m；4 台 ⇒ 200 m ⇒ 門檻 61 m。

    ⚠ 這條同時是反不可能觸發：同一個 120 m 跨度的禁航區，在 2 台時**必須安靜**、
    在 4 台時**必須叫**。若守衛把門檻寫死，兩邊會同時綠或同時紅。
    """
    z = [_zone(120.0)]
    assert nfz_defeats_fleet_split(_subareas(2), z, swath_m=SWATH) == [], "2 台時不該叫"
    assert nfz_defeats_fleet_split(_subareas(4), z, swath_m=SWATH), "4 台時必須叫"


def test_message_fields_are_actionable():
    """診斷要帶得出「哪一塊、多寬、容得下多少」，否則使用者不知道要調什麼。"""
    h = nfz_defeats_fleet_split(_subareas(4), [_zone(150.0)], swath_m=SWATH)[0]
    assert set(h) == {"area_id", "span_m", "width_m", "budget_m"}
    assert isinstance(h["area_id"], int)


def test_note_prefix_is_registered_as_loud_in_the_gui():
    """⚠⚠ 「兩條管線其中一條是死的」：算得出來但 GUI 不顯示 = 等於沒算。"""
    pytest.importorskip("PyQt6")
    from ui.main_window import MainWindow
    assert "nfz_too_wide_for_auto_split" in MainWindow._HV_LOUD_NOTES
