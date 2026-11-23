"""避撞解衝突的收斂性與策略有效性測試。

背景（2026-08-24）
------------------
`IDP_Solver(assignment='joint')` 會產生**交錯**的條帶指派（例如 UAV3 拿
[6,8,10]、UAV4 拿 [7,9,11]），兩架因此同時飛在相鄰條帶上。實測 6 架、
2.0x1.5 km 偵察區：

  * 避撞前兩機最小 3D 間距只有 **1.6 m**（門檻 50 m）
  * 原本的 `run()` 只做**一輪**解衝突 → 只拉到 32.4 m，仍有 2 個殘留衝突
  * 迭代到收斂後 → **60.2 m，0 殘留**

另外發現 `temporal` 策略是**完全無效**的：它插入的 loiter 航點與參考航點
同經緯度同高度，而時間是由「累積距離 / 速度」推出來的，零長度線段貢獻
0 秒延遲；`AvoidanceAction.delay_s` 沒有任何下游會消費。實測跑 5 輪、
回報 5 個動作，最小間距**始終停在 1.6 m**。現已改為大聲報錯並退回 altitude。
"""

from __future__ import annotations

import itertools
import math

import pytest

from core.dccpp.collision_avoidance import CollisionAvoidance
from core.dccpp.dccpp_path_builder import BuiltPath, BuiltWaypoint, SegmentLabel

_R = 6378137.0


def _hav(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _R * math.asin(math.sqrt(a))


def _path(uav_id, lat0, lon0, lat1, lon1, alt, n=40):
    """兩點之間等距切 n 段的直線航跡。"""
    wps = []
    for i in range(n + 1):
        f = i / n
        wps.append(BuiltWaypoint(
            lat=lat0 + (lat1 - lat0) * f,
            lon=lon0 + (lon1 - lon0) * f,
            alt=alt,
            heading_compass_deg=90.0,
            segment_type=SegmentLabel.OPERATION,
        ))
    return BuiltPath(uav_id=uav_id, waypoints=wps)


def _min_separation(paths, speed, dt=1.0):
    """獨立量測：兩兩最小 3D 間距（不使用被測物的偵測器）。"""
    def sample(bp):
        w = bp.waypoints
        cum = [0.0]
        for i in range(1, len(w)):
            cum.append(cum[-1] + math.hypot(
                _hav(w[i-1].lat, w[i-1].lon, w[i].lat, w[i].lon),
                abs(w[i].alt - w[i-1].alt)))
        out, t, j = [], 0.0, 0
        while t * speed <= cum[-1]:
            s = t * speed
            while j + 1 < len(cum) and cum[j+1] < s:
                j += 1
            if j + 1 >= len(cum):
                break
            seg = cum[j+1] - cum[j]
            f = 0.0 if seg <= 0 else (s - cum[j]) / seg
            a, b = w[j], w[j+1]
            out.append((a.lat + (b.lat-a.lat)*f, a.lon + (b.lon-a.lon)*f,
                        a.alt + (b.alt-a.alt)*f))
            t += dt
        return out
    S = {u: sample(bp) for u, bp in paths.items()}
    best = math.inf
    for x, y in itertools.combinations(sorted(S), 2):
        A, B = S[x], S[y]
        for i in range(min(len(A), len(B))):
            d = math.hypot(_hav(A[i][0], A[i][1], B[i][0], B[i][1]), A[i][2] - B[i][2])
            best = min(best, d)
    return best


def _stacked_paths(n_uav=3, alt=120.0):
    """n 架幾乎重疊的同向航跡 —— 需要多輪高度錯開才解得完。

    ⚠ 參數要選得「解得開」：解衝突每輪只把 ID 較大的那架抬 alt_offset_m，
    所以 alt_offset 必須 > min_sep，否則堆疊的機體永遠拉不到門檻
    （alt_offset=30 / min_sep=50 時三架永遠停在間隔 30 m）。
    這裡用 alt_offset=60 > min_sep=50：第一輪解不完、第二輪才收斂，
    正好用來驗證 run() 有沒有迭代。
    """
    lat0, lon0 = 25.0, 121.0
    return {
        i + 1: _path(i + 1, lat0 + i * 1e-5, lon0,
                     lat0 + i * 1e-5, lon0 + 0.02, alt)
        for i in range(n_uav)
    }


MIN_SEP, SPEED, ALT_OFF = 50.0, 18.0, 60.0


def test_the_scenario_really_is_conflicted():
    """前提自檢：這組航跡必須真的違反間距，否則下面全是假綠燈。"""
    paths = _stacked_paths()
    assert _min_separation(paths, SPEED) < MIN_SEP
    ca = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                            cruise_speed_mps=SPEED)
    assert ca.detect(paths), "偵測器沒抓到本來就存在的衝突"


def test_single_round_is_not_enough():
    """單輪解不完 —— 這是 run() 必須迭代的理由。

    若哪天有人把 run() 改回單輪，這條會提醒他單輪本來就不夠。
    """
    paths = _stacked_paths()
    ca = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                            cruise_speed_mps=SPEED)
    conflicts = ca.detect(paths)
    ca.resolve(paths, conflicts)          # 只做一輪
    assert ca.detect(paths), "單輪就解完了，這個場景測不出迭代的必要"


def test_run_iterates_until_no_conflict_remains():
    """run() 必須迭代到收斂，而不是解一輪就放棄。"""
    paths = _stacked_paths()
    ca = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                            cruise_speed_mps=SPEED)
    conflicts, actions = ca.run(paths)
    assert conflicts, "場景本來就該有衝突"
    assert not ca.detect(paths), "run() 之後仍有殘留衝突"


def test_run_actually_achieves_the_required_separation():
    """值層級：用**獨立量測**確認間距真的拉開了，不是只讓偵測器閉嘴。"""
    paths = _stacked_paths()
    before = _min_separation(paths, SPEED)
    ca = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                            cruise_speed_mps=SPEED)
    ca.run(paths)
    after = _min_separation(paths, SPEED)
    assert after > before, f"間距沒有改善（{before:.1f} -> {after:.1f}）"
    assert after >= MIN_SEP - 1e-6, (
        f"run() 後最小間距 {after:.1f} m 仍低於門檻 {MIN_SEP} m"
    )


def test_max_rounds_is_respected_and_reported():
    """輪數上限要被遵守，且不能無聲失敗。"""
    paths = _stacked_paths(n_uav=3)
    ca = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=0.0,
                            cruise_speed_mps=SPEED, max_rounds=2)
    conflicts, actions = ca.run(paths)
    # alt_offset=0 ⇒ 不可能解開；重點是它有回來、沒有無限迴圈
    assert conflicts
    assert ca.detect(paths), "alt_offset=0 竟然解開了衝突？"


def test_temporal_strategy_is_not_a_silent_noop():
    """`temporal` 曾經完全無效卻回報動作數。現在必須至少和 altitude 一樣有效。"""
    paths_t = _stacked_paths()
    paths_a = _stacked_paths()
    ca_t = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                              cruise_speed_mps=SPEED, strategy='temporal')
    ca_a = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                              cruise_speed_mps=SPEED, strategy='altitude')
    ca_t.run(paths_t)
    ca_a.run(paths_a)
    sep_t = _min_separation(paths_t, SPEED)
    assert sep_t >= MIN_SEP - 1e-6, (
        f"temporal 策略後最小間距只有 {sep_t:.1f} m —— 它可能又變回 no-op"
    )
    assert sep_t == pytest.approx(_min_separation(paths_a, SPEED), rel=1e-6), (
        "temporal 應已退回 altitude，兩者結果卻不同"
    )


def test_no_conflict_case_returns_cleanly():
    """本來就沒衝突時不該做任何動作。"""
    lat0, lon0 = 25.0, 121.0
    paths = {
        1: _path(1, lat0, lon0, lat0, lon0 + 0.02, 120.0),
        2: _path(2, lat0 + 0.01, lon0, lat0 + 0.01, lon0 + 0.02, 120.0),
    }
    assert _min_separation(paths, SPEED) > MIN_SEP
    ca = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                            cruise_speed_mps=SPEED)
    conflicts, actions = ca.run(paths)
    assert conflicts == [] and actions == []
