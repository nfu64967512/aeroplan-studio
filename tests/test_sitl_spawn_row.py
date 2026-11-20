"""共用起降場時，SITL 的每架生成點要排成一列（`ui.main_window.spread_colocated_homes`）。

⚠⚠⚠ 2026-09-09 實際現象：GUI 起 4 台 SITL，log 印出

    [SITL] UAV-1 起飛點: (25.074968, 121.235507) hdg=0.0°
    [SITL] UAV-2 起飛點: (25.074968, 121.235507) hdg=0.0°
    [SITL] UAV-3 起飛點: (25.074968, 121.235507) hdg=0.0°
    [SITL] UAV-4 起飛點: (25.074968, 121.235507) hdg=0.0°
    [SITL] 生成座標: … (來源: DCCPP 每架獨立起降點 (4 個))

四個「獨立」起飛點是**同一個座標** —— 因為全隊共用一個起降場，每台計畫的第一個
航點當然相同。那份清單沒有錯，但直接餵給 ``start_multi(homes=…)`` 會讓四台實體
**生在同一點上互相重疊**。

⚠⚠ 最重要的反向臂在 `test_genuinely_separate_homes_are_untouched`：
逐機回收站／多場坪功能會產生**真的不同**的起降點，那些**絕對不可以**被拉成一列。
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("PyQt6")

from ui.main_window import (SITL_COLOCATED_SPREAD_M,       # noqa: E402
                            spread_colocated_homes)

LAT, LON = 25.074968, 121.235507


def _sep_m(a, b):
    dN = (b[0] - a[0]) * 111320.0
    dE = (b[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0]))
    return math.hypot(dN, dE)


def _colocated(n=4, hdg=0.0):
    return [(LAT, LON, hdg)] * n


# ── 前置斷言：輸入真的是共點的 ──────────────────────────────────────────

def test_the_input_really_is_colocated():
    """少了這條，下面的「排開了」可能只是因為輸入本來就分開。"""
    homes = _colocated()
    assert all(_sep_m(homes[0], h) < 1e-6 for h in homes)


# ── 主行為 ──────────────────────────────────────────────────────────────

def test_colocated_homes_are_spread_into_a_row():
    out = spread_colocated_homes(_colocated(4))
    assert len(out) == 4
    for i in range(3):
        assert _sep_m(out[i], out[i + 1]) == pytest.approx(SITL_COLOCATED_SPREAD_M, abs=0.05)


def test_default_spacing_is_ten_metres():
    assert SITL_COLOCATED_SPREAD_M == 10.0


def test_spacing_is_configurable():
    out = spread_colocated_homes(_colocated(3), spacing_m=25.0)
    assert _sep_m(out[0], out[1]) == pytest.approx(25.0, abs=0.05)


def test_row_is_centred_on_the_original_point():
    """對稱展開 ⇒ 中心仍是原起飛點，不會整排偏到一邊。"""
    out = spread_colocated_homes(_colocated(4))
    mid_lat = sum(p[0] for p in out) / len(out)
    mid_lon = sum(p[1] for p in out) / len(out)
    assert _sep_m((mid_lat, mid_lon), (LAT, LON)) < 0.05


def test_row_runs_perpendicular_to_the_takeoff_heading():
    """⭐ 並排而不是一台擋在另一台的起飛路徑上。

    航向 0°（正北）⇒ 這一列必須沿東西展開（南北位移 ≈ 0）。
    """
    out = spread_colocated_homes(_colocated(4, hdg=0.0))
    for p in out:
        assert abs((p[0] - LAT) * 111320.0) < 0.05, p        # 南北位移 ≈ 0
    assert max(abs((p[1] - LON) * 111320.0 * math.cos(math.radians(LAT))) for p in out) > 10.0

    # 反向臂：航向 90°（正東）⇒ 改成沿南北展開
    out90 = spread_colocated_homes(_colocated(4, hdg=90.0))
    for p in out90:
        assert abs((p[1] - LON) * 111320.0 * math.cos(math.radians(LAT))) < 0.05, p


def test_each_aircraft_keeps_its_own_heading():
    homes = [(LAT, LON, 10.0), (LAT, LON, 20.0), (LAT, LON, 30.0)]
    assert [h[2] for h in spread_colocated_homes(homes)] == [10.0, 20.0, 30.0]


# ── ⚠⚠ 反向臂：真的分開的起降場不可以被動到 ────────────────────────────

def test_genuinely_separate_homes_are_untouched():
    """⭐⭐⭐ 多場坪／逐機回收站會產生真的不同的起降點 —— 動它們就毀了那個功能。"""
    homes = [(LAT, LON, 0.0),
             (LAT + 600.0 / 111132.0, LON, 0.0),            # 正北 600 m
             (LAT, LON + 400.0 / (111320.0 * math.cos(math.radians(LAT))), 0.0)]
    assert spread_colocated_homes(homes) is homes


def test_a_single_outlier_keeps_the_whole_set_untouched():
    """只要有**一個**點跟別人不同就整組不動 —— 寧可不排也不要拆散真正的多場坪。"""
    homes = [(LAT, LON, 0.0), (LAT, LON, 0.0),
             (LAT + 100.0 / 111132.0, LON, 0.0)]
    assert spread_colocated_homes(homes) is homes


def test_tolerance_boundary():
    """容差內視為共點（GPS 抖動、浮點誤差），容差外就是真的分開。"""
    near = [(LAT, LON, 0.0), (LAT + 0.5 / 111320.0, LON, 0.0)]     # 相距 0.5 m
    assert spread_colocated_homes(near, tol_m=1.0) is not near
    far = [(LAT, LON, 0.0), (LAT + 2.0 / 111320.0, LON, 0.0)]      # 相距 2 m
    assert spread_colocated_homes(far, tol_m=1.0) is far


# ── 退化輸入 ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("homes", [None, [], [(LAT, LON, 0.0)]])
def test_degenerate_inputs_pass_through(homes):
    assert spread_colocated_homes(homes) is homes


def test_two_element_tuples_are_accepted():
    """有些來源不帶航向；不可以因此爆掉。"""
    out = spread_colocated_homes([(LAT, LON), (LAT, LON)])
    assert len(out) == 2
    assert _sep_m(out[0], out[1]) == pytest.approx(SITL_COLOCATED_SPREAD_M, abs=0.05)
