"""轉彎可行性檢查的值層級鎖。

`FixedWingConstraints.is_path_executable()` 判斷一串航點的每個轉角，
飛機有沒有足夠空間用它的最小轉彎半徑轉過去。

⚠️⚠️⚠️ **2026-09-05 修正**:舊版的切線長公式寫成 ``T = R / tan(α/2)``，
正確是 ``T = R · tan(α/2)``。除號讓結論**完全反過來**:

    10° 緩彎   正確 T=11.3 m   舊版 1479.8 m  → 誤判不可行
    179° 折返  正確 T=14835 m  舊版     1.1 m  → 誤判可行

實測舊版把「緩彎 + 3 km 長腿」判成不可行、把「174° 急折返 + 300 m 短腿」
判成可行。這個函式當時**產品端零呼叫者**，所以錯了很久沒人發現。

⚠️ **90° 是不動點**:tan(45°)=1，乘除同值 ⇒ 直角轉彎在錯誤與正確版本下
   得到**一模一樣**的答案。只用直角驗收會恆綠。本檔因此一定要同時放
   一個緩彎與一個折返，而且先斷言「這兩個方向在錯誤公式下會反過來」。
"""

import math

import pytest

from core.base.fixed_wing_constraints import FixedWingConstraints


def _c(speed=21.5, bank=20.0):
    return FixedWingConstraints(cruise_airspeed_mps=speed,
                                max_bank_angle_deg=bank, safety_factor=1.0)


R_MIN = _c().get_min_turn_radius()


# ═══════════════════════════════════════════════════════════════════
#  0. 可觸發性 —— 先證明錯誤公式與正確公式在這些角度上會給相反答案
# ═══════════════════════════════════════════════════════════════════
def test_the_wrong_formula_really_would_flip_these_cases():
    """沒有這一條，下面的測試可能只是在量一組兩種公式都通過的角度。

    ⚠️ 特別注意 90°:它是 tan(45°)=1 的不動點，兩種公式**相同**。
    所以驗收角度必須避開 90°。
    """
    def t_right(a):
        return R_MIN * math.tan(math.radians(a / 2.0))

    def t_wrong(a):
        return R_MIN / math.tan(math.radians(a / 2.0))

    # 緩彎:正確很小、錯誤很大
    assert t_right(10.0) < 20.0 < 1000.0 < t_wrong(10.0)
    # 急折返:正確很大、錯誤很小
    assert t_wrong(179.0) < 10.0 < 1000.0 < t_right(179.0)
    # 90° 是不動點 —— 用它驗收會恆綠
    assert t_right(90.0) == pytest.approx(t_wrong(90.0), rel=1e-12)


# ═══════════════════════════════════════════════════════════════════
#  1. 方向性:緩彎可行、急彎不可行
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("label,pts,expect", [
    ("緩彎 + 長腿",        [(0, 0), (3000, 0), (6000, 300)], True),
    ("直角 + 長腿",        [(0, 0), (3000, 0), (3000, 3000)], True),
    ("直角 + 短腿",        [(0, 0), (100, 0), (100, 100)], False),
    ("急折返 174° + 短腿", [(0, 0), (300, 0), (0, 30)], False),
    ("純直線",             [(0, 0), (1000, 0), (2000, 0)], True),
])
def test_feasibility_direction(label, pts, expect):
    r = _c().is_path_executable(pts, coord_type="metric")
    assert r.is_feasible is expect, (
        f"{label}: 判定 {r.is_feasible}，應為 {expect}。{r.summary()}"
    )


def test_each_corner_may_claim_only_half_the_shared_leg():
    """相鄰轉角**共用**一條腿，所以每個轉角只能用一半。

    ⚠️ 這條是為了鎖住那個 ÷2。少了它，兩個相鄰的急轉角會各自宣稱整條腿，
    合計需要的切線長超過腿長，路徑其實飛不出來卻判成可行。

    案例挑在剛好分得出來的區間:90° 轉角的切線長 T = R·tan45° = R = 129.5 m。
    腿長 200 m ⇒ 半腿 100 < 129.5(不可行)，整腿 200 > 129.5(會誤判可行)。
    腿長若差到兩倍以上，兩種寫法給一樣的答案，就驗不出來。
    """
    c = _c()
    R = c.get_min_turn_radius()
    leg = 200.0
    assert leg / 2 < R < leg, (
        f"這組腿長 {leg} 對 R={R:.1f} 分不出半腿與整腿，換一組")
    pts = [(0, 0), (leg, 0), (leg, leg)]
    r = c.is_path_executable(pts, coord_type="metric")
    assert not r.is_feasible, "用了整條腿當可用空間 —— ÷2 被拿掉了"


def test_a_full_reversal_is_never_feasible():
    """180° 原地折返:任何半徑都做不到，容得下的 R 必須是 0。"""
    r = _c().is_path_executable([(0, 0), (1000, 0), (0, 0.0)], coord_type="metric")
    assert not r.is_feasible
    i, r_need, r_fit, ang = r.violations[0]
    assert ang == pytest.approx(180.0, abs=0.5)
    assert r_fit == 0.0


# ═══════════════════════════════════════════════════════════════════
#  2. 單調性 —— 腿越長越可行、彎越急越不可行
# ═══════════════════════════════════════════════════════════════════
def test_longer_legs_make_the_same_corner_feasible():
    """同一個轉角，把腿拉長就應該從不可行變可行。"""
    c = _c()
    short = c.is_path_executable([(0, 0), (100, 0), (100, 100)], coord_type="metric")
    long_ = c.is_path_executable([(0, 0), (3000, 0), (3000, 3000)], coord_type="metric")
    assert not short.is_feasible and long_.is_feasible
    assert long_.min_required_radius > short.min_required_radius


def test_sharper_turn_shrinks_the_fitting_radius():
    """腿長固定，偏轉角越大 ⇒ 容得下的半徑越小(嚴格遞減)。"""
    c = _c()
    L = 2000.0
    prev = None
    for ang in (10.0, 45.0, 80.0, 120.0, 160.0):
        # 第一段朝 +x;偏轉 ang 之後第二段的方向角就是 ang。
        # ⚠ 第一版寫成 radians(180-ang)，等於把偏轉角反過來掃，
        #   於是 R_fit 變成遞增，測試自己失敗。
        th = math.radians(ang)
        pts = [(0, 0), (L, 0), (L + L * math.cos(th), L * math.sin(th))]
        r = c.is_path_executable(pts, coord_type="metric")
        if prev is not None:
            assert r.min_required_radius < prev, (
                f"偏轉 {ang}° 的 R_fit 沒有比前一個更小")
        prev = r.min_required_radius


def test_smaller_aircraft_radius_is_never_harder():
    """半徑小的飛機不可能比半徑大的更難通過同一條路徑。

    對應 Pioneer(160 m) 與 HERO(100 m) 的真實差異。
    """
    pts = [(0, 0), (400, 0), (400, 400)]
    big = FixedWingConstraints(cruise_airspeed_mps=21.5, max_bank_angle_deg=20.0,
                               safety_factor=1.0)      # R 較大
    small = FixedWingConstraints(cruise_airspeed_mps=19.0, max_bank_angle_deg=25.0,
                                 safety_factor=1.0)    # R 較小
    assert small.get_min_turn_radius() < big.get_min_turn_radius()
    rb = big.is_path_executable(pts, coord_type="metric")
    rs = small.is_path_executable(pts, coord_type="metric")
    assert not (rb.is_feasible and not rs.is_feasible), (
        "大半徑通過而小半徑不通過 —— 方向反了")


# ═══════════════════════════════════════════════════════════════════
#  3. 退化輸入
# ═══════════════════════════════════════════════════════════════════
def test_duplicate_waypoints_do_not_break_it():
    """⚠️ 實際匯出的路徑裡真的有相鄰間距 0.0 m 的航點(起降段同座標)。

    零長度腿讓偏轉角無定義，舊版會拿它算出垃圾角度。
    """
    c = _c()
    clean = c.is_path_executable([(0, 0), (3000, 0), (6000, 300)], coord_type="metric")
    dup = c.is_path_executable(
        [(0, 0), (0, 0), (3000, 0), (3000, 0), (6000, 300)], coord_type="metric")
    assert dup.is_feasible == clean.is_feasible
    assert dup.min_required_radius == pytest.approx(clean.min_required_radius)


@pytest.mark.parametrize("pts", [[], [(0, 0)], [(0, 0), (100, 0)],
                                 [(0, 0), (0, 0), (0, 0)]])
def test_too_few_distinct_points_is_vacuously_feasible(pts):
    r = _c().is_path_executable(pts, coord_type="metric")
    assert r.is_feasible


def test_latlon_and_metric_agree():
    """同一條路徑用經緯度或公尺座標，判定必須一致。"""
    LAT, LON = 25.05, 121.20
    M_LAT = 111320.0
    M_LON = 111320.0 * math.cos(math.radians(LAT))
    metric = [(0, 0), (100, 0), (100, 100)]
    latlon = [(LAT + y / M_LAT, LON + x / M_LON) for x, y in metric]
    c = _c()
    a = c.is_path_executable(metric, coord_type="metric")
    b = c.is_path_executable(latlon, coord_type="latlon")
    assert a.is_feasible == b.is_feasible
    assert b.min_required_radius == pytest.approx(a.min_required_radius, rel=0.02)


# ═══════════════════════════════════════════════════════════════════
#  4. 違規回報要指得出是哪一個轉角
# ═══════════════════════════════════════════════════════════════════
def test_violation_points_at_the_right_waypoint():
    """一條路徑上只有一個壞轉角時，違規必須指到那一個索引。

    航點:0(0,0) 1(4000,0) 2(8000,0) 3(8000,120) 4(8000,4000)
      轉角1 直行、轉角3 直行(兩段都朝 +y)，
      **只有轉角 2** 是 90° 且被 120 m 的短腿夾住 ⇒ 唯一違規。
    """
    pts = [(0, 0), (4000, 0), (8000, 0), (8000, 120), (8000, 4000)]
    r = _c().is_path_executable(pts, coord_type="metric")
    assert not r.is_feasible
    idxs = [v[0] for v in r.violations]
    assert idxs == [2], f"應該只有航點 2 違規，實際 {idxs}"


def test_summary_names_the_numbers():
    """摘要要講得出是哪一段、幾度、需要多少、容得下多少。"""
    r = _c().is_path_executable([(0, 0), (100, 0), (100, 100)], coord_type="metric")
    s = r.summary()
    assert "不可行" in s
    assert "轉折角" in s
    assert "R=" in s
