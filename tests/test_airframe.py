"""機體能力紀錄的值層級鎖。

鎖三類東西:

  1. **推導值**:轉彎半徑、爬升率、可用航程、對稱半徑。這些是規劃直接吃的量，
     漂了會靜靜改變航線而不報錯。
  2. **矛盾組合會拋錯**:速度階梯順序、能懸停卻要跑道、坡度越界。
  3. **已知的活陷阱**:兩台的最低規劃空速都**低於**現行 SITL 的
     ``AIRSPEED_MIN=20``。把它寫成斷言，SITL 改成逐機之後這條會變紅，
     提醒把鎖一起更新。

⚠️ 每組推導值的斷言前都先斷言「這個設定真的能讓量值不同」，
   否則一條量不到東西的測試跟一條通過的測試在報表上長得一樣。
"""

import math

import pytest

from core.base.airframe import (
    AIRFRAMES, HERO_PNP, PIONEER_PNP, Airframe, AirframeError,
)

#: 現行 ``sitl/default_params/vtol_default.parm`` 的值。
#: ⚠ 這是**單一全域值**，不是逐機的 —— 見 test_known_airspeed_floor_trap。
SITL_AIRSPEED_MIN = 20.0


# ═══════════════════════════════════════════════════════════════════
#  1. 推導值
# ═══════════════════════════════════════════════════════════════════
def test_the_two_airframes_are_actually_different():
    """可觸發性:兩台在每個關鍵量上都要不同，否則後面的比較沒有鑑別力。"""
    for attr in ("cruise_airspeed_mps", "min_mission_airspeed_mps",
                 "stall_airspeed_mps", "transition_airspeed_mps",
                 "max_bank_angle_deg", "mass_kg", "catalogue_range_km"):
        assert getattr(PIONEER_PNP, attr) != getattr(HERO_PNP, attr), (
            f"兩台的 {attr} 相同 —— 這份紀錄就分不出它們"
        )


@pytest.mark.parametrize("af,expect", [(PIONEER_PNP, 129.51), (HERO_PNP, 78.94)])
def test_theoretical_turn_radius(af, expect):
    """R = V²/(g·tan φ)。"""
    assert af.theoretical_turn_radius_m == pytest.approx(expect, abs=0.05)


@pytest.mark.parametrize("af,expect", [(PIONEER_PNP, 160.0), (HERO_PNP, 100.0)])
def test_design_turn_radius_is_what_planning_uses(af, expect):
    assert af.turn_radius_m == pytest.approx(expect)


@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP])
def test_design_radius_leaves_margin_over_theoretical(af):
    """設計半徑必須大於理論下限 —— 風、姿態建立、L1 追蹤誤差都要空間。"""
    assert af.turn_radius_m > af.theoretical_turn_radius_m, (
        f"{af.name} 的設計半徑沒有留裕度"
    )


@pytest.mark.parametrize("af,expect", [(PIONEER_PNP, 1.1252), (HERO_PNP, 0.9944)])
def test_climb_rate_from_path_angle(af, expect):
    """爬升率由**航跡角**推出，不是飛控的 TECS_CLMB_MAX。

    ⚠️ 容差收到 1e-3:3° 之下 sin=0.052336、tan=0.052408,只差 0.14%。
    容差留 5e-3 的話 ``sin`` 寫成 ``tan`` **驗不出來**(實測突變存活)。
    """
    assert af.climb_rate_mps == pytest.approx(expect, abs=1e-3)


def test_climb_rate_really_uses_sin_not_tan():
    """小角度下 sin≈tan，所以用**陡角**去分辨這兩個寫法。

    ⚠️ 沒有這一條，上面那組在 3° 的數值鎖抓不到 sin/tan 互換 ——
    真實航跡角就是小角度，這是這類公式測試的通病。
    """
    steep = Airframe(name="陡角測試", can_hover=True, needs_runway=False,
                     has_turn_radius=True, cruise_airspeed_mps=20.0,
                     min_mission_airspeed_mps=16.0, stall_airspeed_mps=12.0,
                     max_climb_path_angle_deg=60.0)
    # 可觸發性:先證明 60° 之下兩者差很多
    assert abs(math.sin(math.radians(60)) - math.tan(math.radians(60))) > 0.8
    assert steep.climb_rate_mps == pytest.approx(20.0 * math.sin(math.radians(60)),
                                                 abs=1e-6)
    assert steep.climb_rate_mps < 20.0, "爬升率不可能超過空速(用 tan 就會)"


def test_climb_rate_is_far_below_the_sitl_tecs_setting():
    """⚠️⚠️ 實機爬升率約 1 m/s，而 SITL 的 TECS_CLMB_MAX 設 5.0。

    模型若拿飛控參數當爬升能力，爬升距離會少算約 4 倍
    (100 m 爬升:模型 480 m vs 實機 1908 m)。
    """
    from core.dccpp.vtol_profile import TECS_CLMB_MAX
    for af in (PIONEER_PNP, HERO_PNP):
        assert af.climb_rate_mps < TECS_CLMB_MAX / 3.0, (
            f"{af.name} 的爬升率不再遠低於 TECS_CLMB_MAX —— "
            "這條警告的前提變了，要重新確認模型有沒有跟著改"
        )


@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP])
def test_climb_distance_for_100m(af):
    """兩台爬升角相同(3.0°)，所以爬 100 m 的水平距離也相同。"""
    assert af.climb_distance_m(100.0) == pytest.approx(1908.1, abs=1.0)
    assert af.climb_distance_m(0.0) == 0.0
    assert af.climb_distance_m(-5.0) == 0.0


@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP])
def test_descent_is_steeper_than_climb(af):
    assert af.max_descent_path_angle_deg > af.max_climb_path_angle_deg
    assert af.descent_distance_m(100.0) < af.climb_distance_m(100.0)


@pytest.mark.parametrize("af,km,radius", [(PIONEER_PNP, 133.1, 66.55),
                                          (HERO_PNP, 79.2, 39.6)])
def test_range_budget(af, km, radius):
    assert af.usable_range_km == pytest.approx(km, abs=0.1)
    assert af.max_symmetric_radius_km == pytest.approx(radius, abs=0.05)


# ═══════════════════════════════════════════════════════════════════
#  2. 型錄速度一致性
# ═══════════════════════════════════════════════════════════════════
def test_only_hero_cruises_faster_than_its_catalogue_point():
    """⚠️ HERO 規劃巡航 19.0 高於型錄隱含的 17.65 ⇒ 飛得比最省航程的速度快，
    可用比率要同時吸收保留電量**和**速度懲罰。Pioneer 沒有這個問題。

    兩台用同一個 0.55 比率，保守程度其實不一樣，這條把它釘住。
    """
    assert PIONEER_PNP.catalogue_implied_airspeed_mps == pytest.approx(22.92, abs=0.02)
    assert HERO_PNP.catalogue_implied_airspeed_mps == pytest.approx(17.65, abs=0.02)
    assert not PIONEER_PNP.cruises_faster_than_catalogue
    assert HERO_PNP.cruises_faster_than_catalogue


# ═══════════════════════════════════════════════════════════════════
#  3. 三個性質確實是分開的
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP])
def test_hover_runway_and_turn_radius_are_three_separate_facts(af):
    """⚠️⚠️ 這正是 ``turn_radius > 0`` 混在一起的三件事。

    VTOL 同時是「能懸停」「不需要跑道」「有轉彎半徑」——
    只看轉彎半徑推不出前兩件事的任何一件。
    """
    assert af.can_hover is True
    assert af.needs_runway is False
    assert af.has_turn_radius is True
    assert af.turn_radius_m > 0
    # 可觸發性:證明「有轉彎半徑」推不出「要跑道」
    assert af.has_turn_radius and not af.needs_runway


def test_no_turn_radius_means_zero_radius():
    a = Airframe(name="純多旋翼", can_hover=True, needs_runway=False,
                 has_turn_radius=False, cruise_airspeed_mps=12.0,
                 min_mission_airspeed_mps=3.0, stall_airspeed_mps=0.5)
    assert a.turn_radius_m == 0.0
    assert a.theoretical_turn_radius_m == 0.0


# ═══════════════════════════════════════════════════════════════════
#  4. 矛盾組合要拋錯
# ═══════════════════════════════════════════════════════════════════
def _kw(**over):
    base = dict(name="測試機", can_hover=True, needs_runway=False,
                has_turn_radius=True, cruise_airspeed_mps=20.0,
                min_mission_airspeed_mps=16.0, stall_airspeed_mps=12.0,
                transition_airspeed_mps=14.0)
    base.update(over)
    return base


def test_the_valid_baseline_really_constructs():
    """可觸發性:先證明基準組態是合法的，否則下面每條 raises 都可能是別的原因。"""
    assert Airframe(**_kw()).name == "測試機"


@pytest.mark.parametrize("over,frag", [
    (dict(min_mission_airspeed_mps=25.0), "順序反了"),
    # ⚠️ 這一格要同時把 transition 抬到 16，否則「轉換空速 14 低於失速 16」
    #   那條也會觸發，而它的訊息**也含「失速」** —— 拿掉速度階梯檢查時
    #   斷言仍然通過(實測突變存活)。比對字串要挑各自獨有的片段。
    (dict(stall_airspeed_mps=16.0, transition_airspeed_mps=16.0), "沒有高於"),
    (dict(transition_airspeed_mps=5.0), "轉換空速"),
    (dict(needs_runway=True), "跑道"),
    (dict(max_bank_angle_deg=95.0), "坡度上限"),
    (dict(max_bank_angle_deg=0.0), "坡度上限"),
    (dict(max_climb_path_angle_deg=0.0), "max_climb_path_angle_deg"),
    (dict(reserve_fraction=1.5), "保留電量"),
    (dict(usable_range_fraction=0.0), "可用比率"),
    (dict(can_hover=False), "懸停"),
])
def test_contradictions_raise(over, frag):
    with pytest.raises(AirframeError, match=frag):
        Airframe(**_kw(**over))


@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP])
def test_speed_ladder_holds(af):
    """巡航 ≥ 規劃下限 > 轉換 ≥ 失速。"""
    assert af.cruise_airspeed_mps >= af.min_mission_airspeed_mps
    assert af.min_mission_airspeed_mps > af.stall_airspeed_mps
    assert af.transition_airspeed_mps >= af.stall_airspeed_mps


def test_hero_stall_uses_the_conservative_value_not_the_catalogue_one():
    """⚠️ 型錄失速 10 m/s 是特定測試條件;規劃一律用 12。"""
    assert HERO_PNP.stall_airspeed_mps == 12.0, (
        "HERO 的失速值被改回 10 —— 那是特定測試條件，不可當操作下限"
    )


# ═══════════════════════════════════════════════════════════════════
#  5. 已知的活陷阱
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP])
def test_known_airspeed_floor_trap(af):
    """⚠️⚠️ 兩台的最低規劃空速都**低於**現行 SITL 的 AIRSPEED_MIN=20。

    現況:``AIRSPEED_MIN`` 是 ``vtol_default.parm`` 裡的**單一全域值**，
    而 ``ui/widgets/vtol_recon_panel.py`` 的速度下限因此寫死 20。
    ⇒ HERO 的巡航 19.0 連輸入都輸不進去，兩台的最低空速也都進不去;
      真的送下去也會被飛控靜默夾到 20。

    這條斷言是把陷阱**記錄**下來。等 AIRSPEED_MIN 改成逐機之後它會變紅，
    那時要做的是把這條測試換成「逐機值有正確寫進該機的 .parm」。
    """
    assert af.min_mission_airspeed_mps < SITL_AIRSPEED_MIN, (
        f"{af.name} 的最低規劃空速已經不低於全域 AIRSPEED_MIN 了 —— "
        "若是因為 SITL 改成逐機，請改寫這條鎖"
    )


def test_hero_cruise_is_below_the_panel_floor():
    """HERO 巡航 19.0 低於面板寫死的下限 20 ⇒ 今天不可能規劃 HERO 的巡航。"""
    assert HERO_PNP.cruise_airspeed_mps < SITL_AIRSPEED_MIN
    assert PIONEER_PNP.cruise_airspeed_mps > SITL_AIRSPEED_MIN, (
        "對照組:Pioneer 的巡航在下限之上，所以這不是「所有機體都進不去」"
    )


@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP])
def test_transition_altitudes_are_still_flagged_as_placeholders(af):
    """⚠️ 轉換高度目前是暫定值。旗標關掉之前，任何報表都要標註。

    讀到原廠 .param 覆寫之後，把旗標設 False，這條會變紅提醒更新。
    """
    assert af.transition_agl_is_placeholder is True
    assert af.transition_start_agl_m < af.transition_complete_agl_m
    assert af.transition_complete_agl_m <= af.min_fw_segment_agl_m


def test_registry_covers_both():
    assert set(AIRFRAMES) == {"Pioneer PNP", "HERO PNP"}
    for name, af in AIRFRAMES.items():
        assert af.name == name
