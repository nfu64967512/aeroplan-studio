# -*- coding: utf-8 -*-
"""避撞「不合理組合要大聲、解不掉要大聲」的值層級守衛(2026-10-08)。

起因:`build_avoidance_kwargs` 送 `max_alt_offset_m`,而 `CollisionAvoidance.__init__`
沒有這個參數 ⇒ 每次規劃 TypeError,被 `_display_dccpp_result` 的 `except Exception`
吞成一行 WARNING,避撞後處理從沒在 GUI 跑過。

這裡鎖的是 test_collision_alt_cap.py 沒覆蓋到的那一圈:
  * 接縫送出的**每一個** key 建構子都收得下(以後再加欄位漏一邊,當場紅)
  * 不合理組合只警告、不偷偷夾值
  * 解不掉(跑滿輪數/停滯)是 ERROR,不是安靜結束
"""

import inspect
import logging

import pytest

from core.dccpp.collision_avoidance import CollisionAvoidance
from tests.test_collision_alt_cap import _overlapping_pair


def test_seam_keys_are_all_accepted_by_the_constructor():
    """接縫產出的 kwargs 必須全是建構子認得的名字 —— 這次事故的直接形狀。"""
    from ui.main_window import build_avoidance_kwargs
    accepted = set(inspect.signature(CollisionAvoidance.__init__).parameters)
    for params in ({}, {'max_alt_offset_m': None}, {'max_alt_offset_m': 80.0},
                   {'avoid_strategy': 'temporal'}):
        kw = build_avoidance_kwargs(params, cruise_speed_mps=18.0)
        assert set(kw) <= accepted, f"建構子不認得 {set(kw) - accepted}"


def test_cap_below_single_offset_is_an_error(caplog):
    """上限 < 單次抬升 ⇒ 連抬一次都不行 ⇒ 避撞形同關閉,必須 ERROR。"""
    with caplog.at_level(logging.ERROR):
        ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=60.0,
                                max_alt_offset_m=40.0)
    assert any('抬升上限' in r.message for r in caplog.records)
    assert ca.max_alt_offset == pytest.approx(40.0)      # 警告,但不夾值
    assert ca.alt_offset == pytest.approx(60.0)


def test_offset_below_separation_warns_but_keeps_the_value(caplog):
    with caplog.at_level(logging.WARNING):
        ca = CollisionAvoidance(min_separation_m=80.0, alt_offset_m=20.0)
    assert any('高度錯開量' in r.message for r in caplog.records)
    assert ca.alt_offset == pytest.approx(20.0)


def test_sane_combination_is_silent(caplog):
    """對照組:合理組合不可以響(否則警告失去意義)。"""
    with caplog.at_level(logging.WARNING):
        CollisionAvoidance(min_separation_m=50.0, alt_offset_m=60.0,
                           max_alt_offset_m=200.0)
    assert not caplog.records


def test_nan_cap_is_rejected():
    """NaN 讓 `x > cap` 恆為 False ⇒ 上限靜靜失效。"""
    with pytest.raises(ValueError):
        CollisionAvoidance(max_alt_offset_m=float('nan'))


@pytest.mark.parametrize("rounds", [0, -1])
def test_non_positive_rounds_is_rejected(rounds):
    with pytest.raises(ValueError):
        CollisionAvoidance(max_rounds=rounds)


def test_running_out_of_rounds_is_an_error(caplog):
    """跑滿輪數仍有衝突 ⇒ ERROR(以前是一行 WARNING「可能需要多輪迭代」)。"""
    paths = _overlapping_pair()
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            cruise_speed_mps=22.0, max_alt_offset_m=1e9,
                            max_rounds=2)
    with caplog.at_level(logging.ERROR):
        ca.run(paths)
    assert any('跑滿' in r.message and r.levelno >= logging.ERROR
               for r in caplog.records)
    assert ca.detect(paths), "這個夾具在 2 輪內就解開了 ⇒ 測不到「跑滿」"


def test_stall_is_an_error_and_stops_early(caplog):
    """所有衝突都被上限擋下 ⇒ 本輪零動作 ⇒ 停滯保險絲 ERROR,而且提早結束。"""
    paths = _overlapping_pair()
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            cruise_speed_mps=22.0, max_alt_offset_m=30.0,
                            max_rounds=8)
    with caplog.at_level(logging.ERROR):
        _cf, actions = ca.run(paths)
    assert any('停滯' in r.message for r in caplog.records)
    assert len(actions) == 1, f"上限 == 單次偏移,只該動一次(實際 {len(actions)})"


def test_remaining_conflicts_reports_and_does_not_leak_between_runs():
    """GUI 靠 `remaining_conflicts` 決定要不要彈警告 ⇒ 它必須反映**這一趟**。

    ⚠ 上一趟沒解掉、這一趟沒有任何衝突(提早 return)時,若不清空,GUI 會對一份
    乾淨的規劃彈出上一趟的警告。
    """
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            cruise_speed_mps=22.0, max_alt_offset_m=30.0)
    ca.run(_overlapping_pair())
    assert ca.remaining_conflicts, "上限 == 單次偏移 ⇒ 完全重疊的航跡應該解不開"
    far = {1: _overlapping_pair()[1], 2: _overlapping_pair()[2]}
    for w in far[2].waypoints:
        w.lon += 0.5                           # 拉到幾十公里外 ⇒ 沒有衝突
    ca.run(far)
    assert ca.remaining_conflicts == []


def test_run_state_does_not_leak_between_runs():
    """同一個實例連跑兩次:第二次的基準高度必須重新量,不可沿用上一趟。

    ⚠ 兩趟必須**航高不同**才咬得到:同高度時沿用的舊基準恰好等於新基準,
    漏掉重設完全看不出來。第二趟若沿用 120 m 的舊基準,200 m 的路徑一開始就
    「比基準高 80 m」> 上限 ⇒ 一次都抬不動,衝突沒解掉。
    """
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            cruise_speed_mps=22.0, max_alt_offset_m=60.0)
    first = _overlapping_pair(alt=120.0)
    ca.run(first)
    second = _overlapping_pair(alt=200.0)
    ca.run(second)
    rise = lambda ps, base: max(w.alt for w in ps[2].waypoints) - base
    assert rise(first, 120.0) > 0, "第一趟就沒抬動 ⇒ 夾具失效"
    assert rise(second, 200.0) == pytest.approx(rise(first, 120.0))


# ──────────────────────────────────────────────────────────────
#  上限必須涵蓋「斜坡點」,不只是衝突窗
# ──────────────────────────────────────────────────────────────
# 隨機路徑模糊檢查(300 組)抓到 1 組:UAV-4 飛到 +105 m,上限 100 m。
# 成因:漸升/漸降的斜坡點在衝突窗**外**,先前輪次已抬過的高度會再疊一次,
# 而上限只看窗內峰值 ⇒ 完全重疊的夾具(窗口不移動)測不到。
# 下面是那組路徑的凍結版(不依賴 random 的版本行為)。
_RAMP_OVERSHOOT_FIXTURE = (
    (1, 25.00026448232989, 121.00014844203811, 0.00040814256296143713, -0.00038902271831999435,
     [120, 110, 110, 130, 130, 110, 130, 120, 120, 120, 110, 120, 130, 120, 120, 120, 120, 120, 120, 110, 110, 110, 120, 120, 120]),
    (2, 25.000040077179726, 120.99978629564397, 0.0003206104347386569, -0.0004885014151107323,
     [110, 110, 130, 120, 120, 110, 110, 110, 110, 120, 130, 110, 120, 120, 120, 110, 110, 120, 120, 120, 130, 130, 130, 120, 110, 110, 110, 110, 110, 110, 130, 120, 120, 120, 110, 120, 130, 120, 130, 110]),
    (3, 25.000104576330386, 121.00017007671947, -0.0004303657891196945, -0.0002214186967795603,
     [120, 130, 120, 130, 120, 110, 110, 130, 130, 120, 130, 110, 130, 120, 120, 120, 110, 130, 120, 110, 110, 130, 110, 120, 110]),
    (4, 25.000292765146888, 120.99998197993196, 0.0003850518707078854, -0.00038159593003850425,
     [130, 130, 120, 120, 120, 110, 120, 120, 130, 130, 110, 120, 120, 120, 130, 110, 120, 130, 130, 120, 120, 120, 130, 110, 120]),
)


def test_cap_also_bounds_the_ramp_points():
    """任何非起降航點的累加抬升(相對該機一開始的最高點)都不得超過上限。"""
    from tests.test_collision_alt_cap import _Path, _WP
    paths = {uid: _Path([_WP(lat0 + dlat * i, lon0 + dlon * i, float(a))
                         for i, a in enumerate(alts)])
             for uid, lat0, lon0, dlat, dlon, alts in _RAMP_OVERSHOOT_FIXTURE}
    base = {uid: max(w.alt for w in bp.waypoints) for uid, bp in paths.items()}
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            cruise_speed_mps=20.0, max_alt_offset_m=100.0)
    ca.run(paths)
    lifted = {uid: max(w.alt for w in bp.waypoints) - base[uid]
              for uid, bp in paths.items()}
    assert max(lifted.values()) > 0, "夾具沒抬動任何一架 ⇒ 測不到上限"
    for uid, rise in lifted.items():
        assert rise <= 100.0 + 1e-6, (
            f"UAV-{uid} 某個航點比起始最高點高了 {rise:.1f} m,超過上限 100 m"
            f"(斜坡點被疊了第二次)")
