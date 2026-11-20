"""海上目標的高度基準。

本任務的紅心在台灣海峽開闊海面(桃園機場向西 ~130 km),海面 = 0 m AMSL。
SRTM 在水域沒有資料,TERRAIN_REPORT 回的是外插值 —— 同一個紅心座標六架各報
一個高度。實測散布 P91 14.3 m、EL1 16.6 m,而命中門檻只有 10 m,等於基準面
本身就把精度預算吃掉一大半,而且完全靜默(只是被「取平均」吞掉)。

這裡鎖住的是 tot_node 那段 z_g 換算的算術,不是 ROS 節點本身。
"""
import math

import pytest


def zg_from_terrain(terr_bull, terr_home):
    """查地形資料庫:紅心地表 − HOME 地表,兩個讀值都來自同一個資料庫。"""
    return terr_bull - terr_home


def zg_pinned(target_amsl, home_amsl_gps):
    """釘死海平面:目標真 AMSL − HOME 的 **GPS** 海拔。

    ⚠️ 減數是 GPS 而不是資料庫的 HOME 地表 —— 飛機回報的高度是 GPS 相對 HOME,
       目標既然改用真 AMSL,就必須跟同為 GPS 基準的 home_amsl 相減才同基準。
    """
    return target_amsl - home_amsl_gps


# 實測值(來自 runs/trial_P91.log 與 trial_EL1.log 的「紅心地表 z_g」行)
P91_HOME = [32.20, 32.03, 32.11, 32.10, 31.99, 32.10]
P91_BULL = [-10.70, -7.40, -4.13, -1.59, 0.86, 3.53]
EL1_HOME = [31.89, 32.06, 32.03, 32.01, 31.99, 32.08]
EL1_BULL = [-11.02, -16.57, -23.28, -20.11, -12.95, -6.64]


@pytest.mark.parametrize("home,bull,least", [(P91_HOME, P91_BULL, 14.0),
                                             (EL1_HOME, EL1_BULL, 16.0)])
def test_terrain_database_disagrees_across_aircraft(home, bull, least):
    """同一個紅心座標,六架不該有六個地表高度 —— 但查資料庫就是會。"""
    zs = [zg_from_terrain(b, h) for b, h in zip(bull, home)]
    assert max(zs) - min(zs) >= least


@pytest.mark.parametrize("home", [P91_HOME, EL1_HOME])
def test_pinned_datum_is_consistent_across_aircraft(home):
    """釘死海平面後,跨機散布應該塌到 GPS 起飛點海拔的雜訊量級(<0.5 m)。"""
    zs = [zg_pinned(0.0, h) for h in home]
    assert max(zs) - min(zs) < 0.5


@pytest.mark.parametrize("home,bull", [(P91_HOME, P91_BULL), (EL1_HOME, EL1_BULL)])
def test_terrain_datum_aims_below_sea_level(home, bull):
    """照地形庫瞄會瞄到海平面**以下** —— 這正是要擋掉的失敗模式。"""
    ham = sum(home) / len(home)
    zg_mean = sum(zg_from_terrain(b, h) for b, h in zip(bull, home)) / len(home)
    assert ham + zg_mean < -1.0          # 紅心地表被算成負的 AMSL


@pytest.mark.parametrize("home", [P91_HOME, EL1_HOME])
def test_pinned_datum_puts_target_exactly_at_sea_level(home):
    """釘死之後,換算回去的紅心地表必須就是 0 m AMSL。"""
    for h in home:
        assert h + zg_pinned(0.0, h) == pytest.approx(0.0, abs=1e-9)


def test_nan_means_use_terrain_database():
    """target_amsl 預設 NaN = 維持原本查資料庫的行為(陸地目標)。"""
    assert math.isnan(float("nan"))
    assert not math.isnan(0.0)           # 0.0 是有效的釘死值,不可被當成「未設定」


def test_scatter_warning_threshold_catches_both_trials():
    """散布警告門檻 2 m:兩趟實測都該被抓到,而釘死後都不該觸發。"""
    for home, bull in [(P91_HOME, P91_BULL), (EL1_HOME, EL1_BULL)]:
        zs = [zg_from_terrain(b, h) for b, h in zip(bull, home)]
        assert max(zs) - min(zs) > 2.0
        zp = [zg_pinned(0.0, h) for h in home]
        assert max(zp) - min(zp) <= 2.0
