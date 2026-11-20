"""stot_score.py 的「首航次窗」—— 逃逸的機不准用後來的隨機重穿補救。

背景(SW1,2026-08-04):
  tot_node 的 impact 集合沒有 miss/timeout 出口。首航次脫靶的機因為 commit
  不成立 ⇒ passed 恆 False ⇒ 不鎖存 ⇒ 持續發俯衝令、v_req 飽和滿速,於是
  UAV2 飛到 207 km、UAV6 飛到 412 km 外,幾小時後隨機重新穿過紅心才鎖存。

  stot_score 舊路徑取的是**全程全域**最近通過點,正好取到那個隨機重穿:
  UAV6 於是印出「最近 2.24 m、AGL 0.89 m」—— 看起來是六機裡打得最準的一架,
  實際上它首航次在 469 m 外、殘餘高度 1216 m 掠過。時間散布因此變成 11950 s。

  ⚠️ 這裡鎖住的是「怎麼算分」,不是「怎麼飛」。首航次沒打中就是 MISS,
     不准用放寬 miss_r 的方式把它算成命中(重放已證明放寬到 800 m 會讓那兩架
     在 1498 m / 1069 m 高空被判命中,製造幽靈命中)。
"""
import csv
import io
import json
import math
import os
import re
import sys
import tokenize

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl"))
import stot_score  # noqa: E402
from stot_score import EARTH, first_pass_window, mlon  # noqa: E402

# ═══════════════════════════════════════════════════════════════════════
#  函式層:窗的邊界行為(用 (0,0) 當目標,經緯度換算才是位元精確的,
#  「回升恰好 1000 m」這種邊界斷言才不會被浮點噪音翻盤)
# ═══════════════════════════════════════════════════════════════════════
ORIGIN = (0.0, 0.0)
ORIGIN_ML = mlon(0.0)


def _pt(t, d_north, alt=1000.0, mode="dive", gs=100.0, agl=None):
    """在正北 d_north 公尺處造一個軌跡點(tuple 順序同 stot_score 的 load)。"""
    return (t, ORIGIN[0] + d_north / EARTH, ORIGIN[1], alt, gs, mode, agl)


def _win(pts, **kw):
    return first_pass_window(pts, ORIGIN, ORIGIN_ML, **kw)


def test_distance_round_trip_is_bit_exact_at_origin():
    """邊界測試的前提:目標在 (0,0) 時公尺→經緯度→公尺是位元精確的。"""
    for d in (500.0, 1000.0, 1500.0, 2000.0):
        assert stot_score.horiz_d(ORIGIN[0] + d / EARTH, 0.0,
                                  ORIGIN, ORIGIN_ML) == d


class TestHysteresisBoundary:
    """回升「恰好 1000 m」不收窗,超過才收 —— 條件是嚴格大於。"""

    def test_exactly_one_thousand_does_not_close(self):
        pts = [_pt(0, 2000.0), _pt(1, 500.0), _pt(2, 1500.0)]
        _i0, i1, info = _win(pts, hyst=1000.0)
        assert info["closed_by"] == "end-of-track"
        assert i1 == 2                       # 窗一路開到軌跡結束
        assert info["d1"] == 500.0

    def test_one_centimetre_past_the_threshold_closes(self):
        pts = [_pt(0, 2000.0), _pt(1, 500.0), _pt(2, 1500.01)]
        _i0, i1, info = _win(pts, hyst=1000.0)
        assert info["closed_by"] == "hysteresis"
        assert i1 == 2
        assert info["d1"] == 500.0

    def test_threshold_follows_the_running_minimum_not_the_start(self):
        """遲滯是相對「目前最近點」量的,不是相對出發點。"""
        pts = [_pt(0, 5000.0), _pt(1, 500.0), _pt(2, 200.0), _pt(3, 1150.0)]
        _i0, _i1, info = _win(pts, hyst=1000.0)
        # 相對 500 早就超過 1000,但相對 200 只回升 950 ⇒ 還不收
        assert info["closed_by"] == "end-of-track"
        assert info["d1"] == 200.0


class TestWindowStart:
    """窗從**放行**那一刻起算,不是從 phase 翻成 DIVE 那一刻。"""

    def test_ring_hold_is_not_the_first_pass(self):
        """SW1 UAV3 的回歸:環上盤旋的局部最小值被當成首航次 ⇒ d1 = 2098 m。

        wait 段在環上來回擺盪(3000 → 2100 → 4200),那個 2100 是繞圈繞出來的,
        不是俯衝。放行後才真的衝進 22 m。
        """
        pts = ([_pt(t, d, mode="wait") for t, d in
                ((0, 3000.0), (1, 2100.0), (2, 4200.0))]
               + [_pt(t, d) for t, d in
                  ((3, 3200.0), (4, 22.0), (5, 3000.0))])
        i0, _i1, info = _win(pts, hyst=1000.0)
        assert i0 == 3                       # 從第一筆非 wait 起算
        assert info["d1"] == 22.0
        assert info["t1"] == 4

    def test_runin_is_also_before_release(self):
        """⭐ 2026-08-04 blocker:只跳 wait 會漏掉 runin。

        tot_node:2039 是 `mode = "runin" if far else "wait"` —— 兩個都是放行前。
        SW1 的 UAV5 在 DIVE 頭 284 筆是 runin(t=939.9–955.2),窗因此從
        t=939.9 / d=4706 m 起算而不是放行的 t=964.5。
        這裡把 runin 段做成「先近再遠」的形狀:少了這個修法,窗會在 runin 的
        局部最小值(1200 m)上被遲滯收掉,真正的 32.6 m 首航次根本掃不到。
        """
        pts = ([_pt(0, 3000.0, mode="runin"), _pt(1, 1200.0, mode="runin"),
                _pt(2, 3000.0, mode="runin"), _pt(3, 2500.0, mode="wait")]
               + [_pt(t, d) for t, d in ((4, 2000.0), (5, 32.6), (6, 2000.0))])
        i0, _i1, info = _win(pts, hyst=1000.0)
        assert i0 == 4, "runin 被當成放行後的俯衝"
        assert info["d1"] == 32.6, "咬到 runin 段的局部最小值"

    def test_all_wait_falls_back_but_says_so(self):
        """整段都是 wait 也不能整架丟掉 —— 但**不可以靜默**退回。

        ⚠️⚠️ 2026-08-04 blocker:這兩條測試上一版鎖的是「靜默退回」,把錯的
           行為當成正確行為釘住了。整段 DIVE 都在放行前 = 這架從來沒被放行
           (逾時航次最常見的形狀),窗退回整段之後 d(t) 的最小值來自環上盤旋,
           不是攻擊航次。退回本身沒錯,錯的是沒有任何欄位或警告說出來 ——
           呼叫端於是把繞圈的最近點印成一個看起來很正常的 MISS(甚至 HIT)。
        """
        pts = [_pt(t, d, mode="wait") for t, d in ((0, 900.0), (1, 40.0))]
        i0, _i1, info = _win(pts, hyst=1000.0)
        assert i0 == 0
        assert info["d1"] == 40.0
        assert info["start"] == "no-release-found"

    def test_all_runin_falls_back_too_and_says_so(self):
        pts = [_pt(t, d, mode="runin") for t, d in ((0, 900.0), (1, 40.0))]
        i0, _i1, info = _win(pts, hyst=1000.0)
        assert i0 == 0
        assert info["d1"] == 40.0
        assert info["start"] == "no-release-found"

    def test_a_released_track_is_marked_release(self):
        """對照組:真的有放行點時標記必須是 release,不能一律標成可疑。"""
        pts = ([_pt(0, 3000.0, mode="wait")]
               + [_pt(t, d) for t, d in ((1, 2000.0), (2, 30.0), (3, 2000.0))])
        i0, _i1, info = _win(pts, hyst=1000.0)
        assert i0 == 1
        assert info["start"] == "release"

    def test_window_start_returns_the_flag_alongside_the_index(self):
        """window_start 的回傳形狀被下游拆解 —— 釘住它。"""
        assert stot_score.window_start(
            [_pt(0, 100.0, mode="wait")]) == (0, "no-release-found")
        assert stot_score.window_start(
            [_pt(0, 100.0, mode="wait"), _pt(1, 90.0)]) == (1, "release")

    def test_mode_vocabulary_matches_the_node(self):
        """⚠️ 這條規則是「放行時刻」的代理,靠 mode 字串反推。節點端改名時
        這裡會**靜默**失準,所以把來源釘住。

        ⚠️⚠️ 2026-08-05 blocker:上一版斷言寫死 `== {"wait","runin"}`,而節點
           那時已經新增了 mode "missed" —— 那條斷言等於把「漏掉一個 mode」釘成
           正確答案,綠燈跑過去了。改成**從 tot_node.py 的 DIVE 相推導**詞彙:
           節點再新增/改名任何一個 mode,這條就會紅,而不是靜默失準。
        """
        src = (os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl",
                            "sar_swarm_patches", "tot_node.py"))
        with open(src, encoding="utf-8") as f:
            node_src = f.read()
        assert 'mode = "runin" if far else "wait"' in node_src
        assert 'mode = "missed"' in node_src
        dive = node_src[node_src.index('elif self.phase == "DIVE":'):]
        seen = set()
        for ln in re.findall(r"^\s*mode = (.+)$", dive, re.M):
            seen.update(re.findall(r'"(\w+)"', ln))
        assert seen, "抓不到 DIVE 相的 mode 賦值 —— 節點結構變了,這條測試失效"
        assert seen == set(stot_score.KNOWN_DIVE_MODES), (
            "節點 DIVE 相的 mode 詞彙 %s 與 stot_score 的 PRE/POST 兩堆 %s 脫節"
            % (sorted(seen), sorted(stot_score.KNOWN_DIVE_MODES)))

    def test_missed_is_neither_proof_of_release_nor_of_pre_release(self):
        """⭐ blocker 本體:missed 兩種列序都會出現,所以只能「不算證據」。

        tot_node:2004 把 _miss_check 放在 `if not released` **之前**(註解:
        「從未放行、卡在環上不動的機一樣走得到出口」)⇒
          從未放行  wait/runin → missed …
          放行後脫靶 wait/runin → dive → … → missed
        """
        assert "missed" in stot_score.PRE_RELEASE_MODES, "missed 被當成放行證據"
        assert "missed" not in stot_score.POST_RELEASE_MODES
        assert set(stot_score.PRE_RELEASE_MODES).isdisjoint(
            stot_score.POST_RELEASE_MODES)

    def test_missed_before_release_is_not_a_release_point(self):
        """⭐ blocker 本體:從未放行的機逾時後 mode 從 wait/runin 變成 missed。

        排除式(mode not in {wait,runin})會把第一筆 missed 當成放行點 ⇒
        start 標 "release" ⇒ UNKNOWN 機制在它唯一要救的形狀上完全不觸發。
        """
        pts = ([_pt(t, d, mode="wait") for t, d in ((0, 3000.0), (1, 2400.0))]
               + [_pt(t, d, mode="missed")
                  for t, d in ((2, 3200.0), (3, 2400.0))])
        assert stot_score.window_start(pts) == (0, "no-release-found")

    def test_missed_after_release_keeps_the_real_release_point(self):
        """對照組:真的放行、俯衝、脫靶,才被 _miss_check 標成 missed。

        ⚠️ 所以**不可以**反過來把 missed 當成「還沒放行」而整段跳過 ——
           這種機的放行點必須照樣在 dive 那一筆被找到。
        """
        pts = ([_pt(0, 3000.0, mode="wait"), _pt(1, 2500.0, mode="runin")]
               + [_pt(2, 1000.0), _pt(3, 620.0)]
               + [_pt(t, d, mode="missed")
                  for t, d in ((4, 900.0), (5, 1100.0))])
        assert stot_score.window_start(pts) == (2, "release")

    def test_impact_and_expended_also_count_as_released(self):
        """納入式的另一半:窗的第一筆就可能已經是 impact/expended
        (同一 tick 命中鎖存),那也是放行後。"""
        for m in ("impact", "expended"):
            pts = [_pt(0, 3000.0, mode="wait"), _pt(1, 40.0, mode=m)]
            assert stot_score.window_start(pts) == (1, "release"), m

    def test_an_unrecognised_mode_is_not_treated_as_release(self):
        """⚠️ 失效方向是刻意選的:節點改名 ⇒ 退到 UNKNOWN + 警告(出聲),
        不是被當成放行(靜默)。靜默失敗是這專案的頭號死因。"""
        pts = [_pt(0, 3000.0, mode="wait"), _pt(1, 40.0, mode="teleport")]
        assert stot_score.window_start(pts) == (0, "no-release-found")
        assert stot_score.unknown_modes(pts) == ["teleport"]

    def test_known_modes_produce_no_vocabulary_complaint(self):
        pts = [_pt(0, 3000.0, mode=m) for m in stot_score.KNOWN_DIVE_MODES]
        assert stot_score.unknown_modes(pts) == []

    def test_outbound_overshoot_cannot_close_the_window(self):
        """放行瞬間機頭沒轉正、先往外衝 —— 那不是「首航次結束」。

        SW1 實測外衝量 UAV3 923 m、UAV2 696 m、UAV5 640 m,距 1000 m 遲滯只剩
        77 m 餘裕。這裡故意讓外衝 1500 m > 遲滯:少了「必須真的接近過」這個
        條件,窗會在出發點就收掉,而且會印出一個看起來很合理的 3000 m d1。
        """
        pts = [_pt(0, 3000.0), _pt(1, 4500.0), _pt(2, 1000.0),
               _pt(3, 20.0), _pt(4, 2000.0)]
        _i0, _i1, info = _win(pts, hyst=1000.0)
        assert info["closed_by"] == "hysteresis"
        assert info["d1"] == 20.0            # 不是 3000.0


class TestTouchdownClosesTheWindow:
    """⓪ AGL ≤ 0 無條件收窗 —— 落水之後不可能再有合法的攻擊航次。

    ⚠️⚠️ 這條規則擋的不是「幾小時後的重穿」(遲滯本來就擋得住),而是
       **同一分鐘內的第二趟**:SITL 不會因為觸海就終結機體。SW1 的 UAV3 在
       t=1012.6 掉到海面下 95.7 m、爬回 +331 m,24.1 s 後再穿一次(22.58 m);
       UAV5 同型(17.9 s 後 67.11 m)。兩趟之間的回升峰值只有 555.5 / 637.0 m,
       遲滯 1000 m 收不掉,於是工具報的是第二趟。
    """

    def _agl(self, p):
        return p[3]                        # 這些合成點的 alt 就當成 AGL

    def test_touchdown_beats_hysteresis(self):
        pts = [_pt(0, 3000.0, alt=500.0), _pt(1, 32.6, alt=100.0),
               _pt(2, 500.0, alt=-50.0),          # 觸海
               _pt(3, 1000.0, alt=200.0), _pt(4, 22.6, alt=100.0)]
        _i0, i1, info = _win(pts, hyst=1000.0, agl_of=self._agl)
        assert info["closed_by"] == "touchdown"
        assert i1 == 2
        assert info["d1"] == 32.6, "報了第二趟"

    def test_window_keeps_the_touchdown_sample(self):
        """⚠️ 收在**第一筆 AGL ≤ 0**,不是它前一筆:觸地點要靠
        (AGL>0, AGL≤0) 這一對相鄰取樣內插,少了後者就掃不到穿越點。"""
        pts = [_pt(0, 3000.0, alt=500.0), _pt(1, 40.0, alt=50.0),
               _pt(2, 60.0, alt=-10.0)]
        _i0, i1, info = _win(pts, agl_of=self._agl)
        assert i1 == 2
        assert self._agl(pts[i1]) <= 0.0

    def test_touchdown_sample_can_itself_be_the_closest(self):
        """陡俯衝時最近點就在觸地那一筆 —— 先更新最小值再收窗,不能先 break。"""
        pts = [_pt(0, 3000.0, alt=500.0), _pt(1, 100.0, alt=50.0),
               _pt(2, 3.0, alt=-1.0)]
        _i0, _i1, info = _win(pts, agl_of=self._agl)
        assert info["closed_by"] == "touchdown"
        assert info["d1"] == 3.0

    def test_no_datum_leaves_the_rule_inert(self):
        """沒有高度基準時這條規則整條失效,窗退回遲滯 —— 不可以把 AGL 當 0。"""
        pts = [_pt(0, 3000.0, alt=-500.0), _pt(1, 40.0, alt=-500.0),
               _pt(2, 2000.0, alt=-500.0)]
        _i0, _i1, info = _win(pts, hyst=1000.0, agl_of=None)
        assert info["closed_by"] == "hysteresis"
        assert info["d1"] == 40.0


class TestEscape:
    def test_beyond_escape_radius_closes_the_window(self):
        pts = [_pt(0, 3000.0), _pt(1, 500.0), _pt(2, 25000.0)]
        _i0, i1, info = _win(pts, hyst=1000.0, escape_r=20000.0)
        assert info["closed_by"] == "escape"
        assert i1 == 2
        assert info["escaped"] is True

    def test_escape_after_the_window_still_counts(self):
        """SW1 UAV2/UAV6 的形狀:首航次在 600 m 外收窗,幾小時後才飛到 200 km。"""
        pts = [_pt(0, 3000.0), _pt(1, 638.0), _pt(2, 2000.0),
               _pt(3, 50000.0), _pt(4, 207000.0)]
        _i0, i1, info = _win(pts, hyst=1000.0, escape_r=20000.0)
        assert info["closed_by"] == "hysteresis"
        assert i1 == 2                       # 窗在回升處就收了
        assert info["d1"] == 638.0
        assert info["escaped"] is True
        # ⚠️ max_d 要掃到底,不能一碰到 20 km 就 break —— 否則警告行永遠只會印
        #    「20.0 km」,把 207 km 的離譜程度壓成剛好過門檻。
        assert info["max_d"] == 207000.0

    def test_clean_pass_is_not_flagged_as_escape(self):
        pts = [_pt(0, 3000.0), _pt(1, 30.0), _pt(2, 4000.0)]
        _i0, _i1, info = _win(pts, hyst=1000.0, escape_r=20000.0)
        assert info["escaped"] is False


# ═══════════════════════════════════════════════════════════════════════
#  端對端:合成一份小 CSV 跑 main()
# ═══════════════════════════════════════════════════════════════════════
T_LAT, T_LON = 25.0777, 119.94315
_ML = mlon(T_LAT)

HDR = ["t", "sysid", "lat", "lon", "alt", "gs", "d_slot", "mode", "v_cmd",
       "slot_lat", "slot_lon", "phase"]


def _row(t, sysid, north, east, alt, mode, phase="DIVE", agl=None):
    r = [round(t, 3), sysid,
         "%.9f" % (T_LAT + north / EARTH), "%.9f" % (T_LON + east / _ML),
         "%.3f" % alt, "%.1f" % (120.0 if mode == "wait" else 100.0),
         0, mode, 95.0, T_LAT, T_LON, phase]
    if agl is not None:
        r.append("%.3f" % agl)
    return r


def _straight_pass(sysid, east, north0, alt0, alt_rate, t0=0.0, n=41,
                   wait_until=5.0, **kw):
    """以 100 m/s 沿正北→正南直線通過紅心旁 `east` 公尺處,1 Hz 取樣。"""
    out = []
    for k in range(n):
        t = t0 + k
        mode = "wait" if (t - t0) < wait_until else "dive"
        out.append(_row(t, sysid, north0 - 100.0 * k,
                        east, alt0 + alt_rate * k, mode, **kw))
    return out


def _write_trial(tmp_path, rows, name="tot_T1.csv"):
    p = tmp_path / name
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HDR if len(rows[0]) == len(HDR) else HDR + ["agl"])
        w.writerows(rows)
    g = tmp_path / name.replace(".csv", "_geom.csv")
    with open(g, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kind", "lat", "lon"])
        w.writerow(["target", T_LAT, T_LON])
    return str(p)


def _build_sw1_shaped_rows():
    """四架:兩架乾淨命中、一架單純脫靶、一架脫靶後飛遠再隨機重穿。"""
    rows = []
    # UAV1 乾淨命中:t=20 通過 5 m 外,當下殘餘高度 500 m
    rows += _straight_pass(1, east=5.0, north0=2000.0, alt0=1000.0,
                           alt_rate=-25.0)
    # UAV2 首航次 800 m 脫靶 → 飛到 50 km 外 → t≈5020 隨機重穿 3 m、高度 0
    rows += _straight_pass(2, east=800.0, north0=2000.0, alt0=2000.0,
                           alt_rate=-20.0)
    for k in range(1, 101):                       # 逃逸段:每秒往東 500 m
        rows.append(_row(40 + k, 2, -2000.0, 800.0 + 500.0 * k, 1200.0, "dive"))
    rows += _straight_pass(2, east=3.0, north0=2000.0, alt0=500.0,
                           alt_rate=-25.0, t0=5000.0, wait_until=-1.0)
    # UAV3 單純脫靶 800 m,沒有逃逸。t1 故意排在 35 s(離兩架 HIT 很遠),
    # 這樣「散布有沒有把 MISS 混進來」在數字上分得出來:只算 HIT 是 5 s,
    # 混進 UAV3 會變成 15 s。
    rows += _straight_pass(3, east=800.0, north0=3500.0, alt0=2000.0,
                           alt_rate=-20.0)
    # UAV4 乾淨命中:t=25 通過 10 m 外,當下殘餘高度 250 m
    rows += _straight_pass(4, east=10.0, north0=2500.0, alt0=1250.0,
                           alt_rate=-40.0)
    # 濾掉的雜訊:非 DIVE 階段的列不該被算進去
    rows.append(_row(0.0, 1, 0.0, 0.0, 3000.0, "weave", phase="ALIGN"))
    return rows


def _run(csv_path, *extra):
    argv = [csv_path, "--ground", "0", "--json"] + list(extra)
    rc = stot_score.main(argv)
    assert rc == 0
    return rc


@pytest.fixture
def trial(tmp_path):
    return _write_trial(tmp_path, _build_sw1_shaped_rows())


def _score(capsys, csv_path, *extra):
    _run(csv_path, *extra)
    cap = capsys.readouterr()
    return json.loads(cap.out), cap.err


class TestWindowedScoring:
    def test_clean_hits_get_exact_t1_and_d1(self, trial, capsys):
        s, _ = _score(capsys, trial)
        u1, u4 = s["per_uav"]["1"], s["per_uav"]["4"]
        assert u1["status"] == "HIT"
        assert u1["d1_m"] == pytest.approx(5.0, abs=0.05)
        assert u1["t1_s"] == pytest.approx(20.0, abs=0.05)
        assert u4["status"] == "HIT"
        assert u4["d1_m"] == pytest.approx(10.0, abs=0.05)
        assert u4["t1_s"] == pytest.approx(25.0, abs=0.05)

    def test_first_pass_miss_is_a_miss(self, trial, capsys):
        """800 m 外掠過就是 MISS —— 不管它後來繞回來穿得多準。"""
        s, _ = _score(capsys, trial)
        assert s["per_uav"]["2"]["status"] == "ESCAPE"     # MISS 的一種
        assert s["per_uav"]["3"]["status"] == "MISS"       # 沒飛遠,只是脫靶
        for i in ("2", "3"):
            assert s["per_uav"][i]["d1_m"] == pytest.approx(800.0, abs=0.5)

    def test_escape_flag_separates_the_two_failure_modes(self, trial, capsys):
        s, _ = _score(capsys, trial)
        assert s["per_uav"]["2"]["escaped"] is True
        assert s["per_uav"]["3"]["escaped"] is False
        assert s["window"]["per_uav"]["2"]["max_d_m"] > 20000.0
        assert s["window"]["per_uav"]["3"]["max_d_m"] < 20000.0

    def test_spread_uses_hits_only(self, trial, capsys):
        """散布只由 HIT 的 t1 算 —— 脫靶機不進統計。"""
        s, _ = _score(capsys, trial)
        v = s["verdict"]
        assert v["hit_ids"] == [1, 4]
        assert v["escape_ids"] == [2]
        assert v["miss_ids"] == [3]
        assert v["n_hit"] == 2
        assert v["all_hit"] is False
        # HIT 的 t1 = 20 與 25 ⇒ 5 s。
        #   把 UAV3(t1=35)混進來 ⇒ 15 s;把 UAV2 的重穿(t≈5020)混進來 ⇒ 5000 s。
        assert s["per_uav"]["3"]["t1_s"] == pytest.approx(35.0, abs=0.05)
        assert v["hit_spread_s"] == pytest.approx(5.0, abs=0.1)

    def test_altitude_is_taken_at_t1_not_at_the_global_minimum(self, trial,
                                                              capsys):
        """UAV2 首航次殘餘 1600 m;全域最小值那一點是 0 m,那是幽靈命中。"""
        s, _ = _score(capsys, trial)
        assert s["per_uav"]["2"]["alt_agl_m"] == pytest.approx(1600.0, abs=1.0)
        assert s["per_uav"]["1"]["alt_agl_m"] == pytest.approx(500.0, abs=1.0)
        assert s["per_uav"]["4"]["alt_agl_m"] == pytest.approx(250.0, abs=1.0)

    def test_miss_is_loud(self, trial, capsys):
        """有 MISS 一定要吼出來 —— 靜默通過是這專案的頭號死因。"""
        s, err = _score(capsys, trial)
        assert s["warnings"], "有 2 架沒命中卻沒有任何警告"
        # ⚠️ UAV3 是「單純脫靶、沒飛遠」—— 逃逸那條警告救不了它。少了未命中
        #    彙總那一行,它就完全無聲無息地不見了,而那正是最該吼的一種。
        assert any("UAV3" in w for w in s["warnings"]), "單純脫靶的機被靜默吞掉"
        assert any("UAV2" in w for w in s["warnings"])
        assert "⚠️" in err                    # --json 時警告走 stderr,不吞掉
        assert "UAV3" in err

    def test_window_metadata_is_reported(self, trial, capsys):
        s, _ = _score(capsys, trial)
        w = s["window"]
        assert w["enabled"] is True
        assert w["hysteresis_m"] == 1000.0
        assert w["escape_r_m"] == 20000.0
        assert w["alt_datum"] == "arg:--ground"
        # 窗從放行(t=5)起算,不是從第一筆 DIVE(t=0,還在 wait)
        assert w["per_uav"]["1"]["t_start"] == pytest.approx(5.0, abs=0.01)
        assert w["per_uav"]["1"]["n_samples"] < w["per_uav"]["1"]["n_samples_full"]

    def test_miss_r_is_overridable_but_does_not_invent_hits(self, trial, capsys):
        """放寬 miss_r 會讓 800 m 外、1600 m 高的那架「算命中」—— 幽靈命中。

        旗標留著是為了做敏感度對照,但殘餘高度會照實印出來當場拆穿它。
        """
        s, _ = _score(capsys, trial, "--miss-r", "1000")
        assert s["per_uav"]["2"]["status"] == "HIT"
        assert s["per_uav"]["2"]["alt_agl_m"] == pytest.approx(1600.0, abs=1.0)
        # 判成 HIT 卻仍舊飛到 50 km 外 ⇒ 一定要有一行講「命中沒有鎖存」
        assert any("鎖存" in w for w in s["warnings"])


class TestBackwardCompatibility:
    def test_no_window_reproduces_the_polluted_old_numbers(self, trial, capsys):
        """--no-window 必須原封不動重現舊路徑(對照用),包含它的病。"""
        s, _ = _score(capsys, trial, "--no-window")
        # 舊路徑取全域最近點:UAV2 的隨機重穿 3 m / AGL 0 m
        assert s["per_uav"]["2"]["miss_m"] == pytest.approx(3.0, abs=0.05)
        assert s["per_uav"]["2"]["alt_agl_m"] == pytest.approx(0.0, abs=1.0)
        assert s["per_uav"]["2"]["t_s"] == pytest.approx(5020.0, abs=0.05)
        # 散布被重穿拖成幾千秒 —— 這正是 SW1 那個 11950 s 的形狀
        assert s["spread_s"] == pytest.approx(5000.0, abs=0.1)

    def test_no_window_emits_no_verdict_and_no_status(self, trial, capsys):
        s, _ = _score(capsys, trial, "--no-window")
        assert s["verdict"] is None
        assert s["window"]["enabled"] is False
        assert s["per_uav"]["1"]["status"] is None
        assert s["warnings"] == []

    def test_legacy_fields_survive_the_window(self, trial, capsys):
        """舊欄位一個都不能少,下游 _kpi_parse / stot_repeat 還在讀。"""
        s, _ = _score(capsys, trial)
        for k in ("n", "n_touch", "max_miss_m", "spread_s", "max_abs_alt_m",
                  "touch_max_miss_m", "touch_spread_s", "touch_gs_mean_ms",
                  "touch_gs_min_ms", "touch_gs_max_ms", "per_uav"):
            assert k in s, k
        for k in ("gs_peak_ms", "miss_m", "alt_agl_m", "alt_rel_m", "t_s",
                  "gs_ms", "impact_miss_m", "impact_t_s", "impact_gs_ms",
                  "n_samples", "z_g"):
            assert k in s["per_uav"]["1"], k

    def test_window_narrows_the_search_but_keeps_the_same_geometry(self, trial,
                                                                   capsys):
        """沒逃逸的機:開窗與不開窗應該給出同一個最近點。"""
        won, _ = _score(capsys, trial)
        woff, _ = _score(capsys, trial, "--no-window")
        for i in ("1", "3", "4"):
            assert won["per_uav"][i]["miss_m"] == woff["per_uav"][i]["miss_m"]
            assert won["per_uav"][i]["t_s"] == woff["per_uav"][i]["t_s"]


class TestAglColumn:
    """另一支節點正在給 CSV 加 agl 欄:新舊兩種都要吃得下,而且要標明基準。"""

    def test_agl_column_is_used_when_present(self, tmp_path, capsys):
        rows = [_row(t, 1, 2000.0 - 100.0 * t, 5.0, 1000.0 - 25.0 * t,
                     "wait" if t < 5 else "dive",
                     agl=1000.0 - 25.0 * t - 100.0) for t in range(41)]
        p = _write_trial(tmp_path, rows, name="tot_AGL.csv")
        s, _ = _score(capsys, p)
        assert s["window"]["alt_datum"] == "csv:agl"
        assert s["per_uav"]["1"]["alt_datum"] == "csv:agl"
        # alt(t1=20) = 500,agl 欄比它低 100 ⇒ 用欄位就是 400,用 --ground 是 500
        assert s["per_uav"]["1"]["alt_agl_m"] == pytest.approx(400.0, abs=1.0)

    def test_old_csv_without_agl_still_scores(self, trial, capsys):
        s, _ = _score(capsys, trial)
        assert s["window"]["alt_datum"] == "arg:--ground"
        assert s["per_uav"]["1"]["alt_agl_m"] == pytest.approx(500.0, abs=1.0)


# ═══════════════════════════════════════════════════════════════════════
#  R1 blocker —— 觸地判定必須跟 alt_agl_m 用**同一個**高度基準
#
#  上一版:agl 欄只餵給 alt_agl_m,觸地掃描仍寫死 `alt − log 的 z_g`,
#  而 per_uav[i]["alt_datum"] / window["alt_datum"] 卻被標成 "csv:agl"。
#  觸地是判定 ①②③ 與 all_touch 的**唯一**來源 ⇒ 同一份報表裡兩套基準並存
#  而且標錯,這是本專案列為最高風險的一類錯。
# ═══════════════════════════════════════════════════════════════════════

def _score_argv(capsys, *argv):
    """直接指定完整引數(要測「沒有 --log / 沒有 --ground」的情境)。"""
    assert stot_score.main(list(argv) + ["--json"]) == 0
    cap = capsys.readouterr()
    return json.loads(cap.out), cap.err


class TestTouchDatum:
    def _agl_pass(self, tmp_path, name="tot_TD.csv", zg=100.0):
        """一架乾淨命中:t=20 正對紅心時 agl 剛好歸零(觸地誤差 0 m)。

        ⚠️ 觸地點必須落在首航次窗內(遲滯 1000 m ⇒ 窗大約收在 t=31),
           否則測到的是「窗外沒掃到」而不是「基準用錯了」。
        """
        rows = [_row(t, 1, 2000.0 - 100.0 * t, 0.0, 600.0 - 25.0 * t,
                     "wait" if t < 5 else "dive",
                     agl=600.0 - 25.0 * t - zg) for t in range(41)]
        return _write_trial(tmp_path, rows, name=name)

    def test_touchdown_is_scored_from_the_agl_column_without_any_log(
            self, tmp_path, capsys):
        """⭐ 失效情境 A:log 輪替掉了(stot_rescore_all.py 是
        `if os.path.exists(logp)` 才加 --log)、也沒給 --ground。

        上一版:g=None ⇒ touch 全 None ⇒ 印「六機都沒有真的落地」、判未達標。
        資料明明就在 agl 欄裡,一趟完美航次被判成全員沒落地。
        """
        p = self._agl_pass(tmp_path)
        s, _ = _score_argv(capsys, p)                  # 沒有 --log、沒有 --ground
        u1 = s["per_uav"]["1"]
        assert u1["z_g"] is None, "前提:確實沒有任何 z_g 來源"
        assert s["window"]["touch_datum"] == "csv:agl"
        assert u1["impact_miss_m"] is not None, "agl 欄就在那裡,卻說沒落地"
        assert u1["impact_miss_m"] == pytest.approx(0.0, abs=1.0)
        assert u1["impact_t_s"] == pytest.approx(20.0, abs=0.05)
        assert s["n_touch"] == 1

    def test_touch_and_closest_point_never_use_different_data(self, tmp_path,
                                                              capsys):
        p = self._agl_pass(tmp_path)
        s, _ = _score_argv(capsys, p, "--ground", "0")
        u1 = s["per_uav"]["1"]
        assert u1["alt_datum"] == u1["touch_datum"] == "csv:agl"
        assert s["window"]["alt_datum"] == s["window"]["touch_datum"]

    def test_agl_column_wins_over_a_stale_ground_argument(self, tmp_path,
                                                          capsys):
        """⭐ 失效情境 B:陸地目標時 tot_node 的 z_g 每收到一次 TERRAIN_REPORT
        就更新,但只有第一次進 log ⇒ agl 欄用時變 z_g、touch 用初始 z_g,
        差多少完全靜默,而報表宣稱 alt_datum = csv:agl。

        造一份兩者會給出**不同觸地點**的資料:agl 在正對紅心那一刻歸零
        (誤差 0 m),alt−0 則要再飛 4 s 才歸零(誤差 400 m)。
        標籤說 csv:agl,量出來就必須是 csv:agl 的那一個。
        """
        rows = [_row(t, 1, 2000.0 - 100.0 * t, 0.0, 600.0 - 25.0 * t,
                     "wait" if t < 5 else "dive",
                     agl=600.0 - 25.0 * t - 100.0) for t in range(41)]
        p = _write_trial(tmp_path, rows, name="tot_TB.csv")
        s, _ = _score_argv(capsys, p, "--ground", "0")
        u1 = s["per_uav"]["1"]
        assert u1["touch_datum"] == "csv:agl"
        assert u1["impact_t_s"] == pytest.approx(20.0, abs=0.05)
        assert u1["impact_miss_m"] == pytest.approx(0.0, abs=1.0), \
            "掃描仍在用 alt−ground(那個會給 400 m)"

    def test_no_datum_at_all_is_unmeasurable_not_a_flyover(self, tmp_path,
                                                           capsys):
        """「量不出來」和「沒落地」是兩件事,不可以印成同一句。"""
        rows = [_row(t, 1, 2000.0 - 100.0 * t, 0.0, 1000.0 - 25.0 * t,
                     "wait" if t < 5 else "dive") for t in range(41)]
        p = _write_trial(tmp_path, rows, name="tot_ND.csv")
        s, err = _score_argv(capsys, p)                # 三個來源一個都沒有
        assert s["per_uav"]["1"]["touch_datum"] == "none"
        assert s["window"]["touch_datum"] == "none"
        assert s["n_touch"] == 0
        assert "算不出來" in err
        # 文字模式也要講清楚,不能只印「六機都沒有真的落地」
        stot_score.main([p])
        assert "算不出來" in capsys.readouterr().out

    def test_partial_agl_column_falls_back_and_says_so(self, tmp_path, capsys):
        """半有半無時退回 alt−z_g —— 在同一條軌跡上混兩套基準比退回更危險。"""
        rows = []
        for t in range(41):
            r = _row(t, 1, 2000.0 - 100.0 * t, 0.0, 1000.0 - 25.0 * t,
                     "wait" if t < 5 else "dive",
                     agl=1000.0 - 25.0 * t)
            if t % 3 == 0:                       # 挖掉三分之一
                r[-1] = ""
            rows.append(r)
        p = _write_trial(tmp_path, rows, name="tot_PA.csv")
        s, err = _score_argv(capsys, p, "--ground", "0")
        u1 = s["per_uav"]["1"]
        assert u1["alt_datum"] == u1["touch_datum"] == "arg:--ground"
        assert "agl 欄只有" in err

    def test_mixed_datums_across_aircraft_are_labelled_mixed(self, tmp_path,
                                                             capsys):
        """不同機用不同基準時,頂層標籤不准只印好看的那一個。"""
        rows = [_row(t, 1, 2000.0 - 100.0 * t, 0.0, 1000.0 - 25.0 * t,
                     "wait" if t < 5 else "dive",
                     agl=1000.0 - 25.0 * t) for t in range(41)]
        rows += [_row(t, 2, 2000.0 - 100.0 * t, 50.0, 1000.0 - 25.0 * t,
                      "wait" if t < 5 else "dive") for t in range(41)]
        for r in rows:                            # UAV2 的 agl 欄留空
            if len(r) == len(HDR):
                r.append("")
        p = _write_trial(tmp_path, rows, name="tot_MX.csv")
        s, _ = _score_argv(capsys, p, "--ground", "0")
        assert s["per_uav"]["1"]["touch_datum"] == "csv:agl"
        assert s["per_uav"]["2"]["touch_datum"] == "arg:--ground"
        assert s["window"]["touch_datum"] == "mixed:arg:--ground+csv:agl"


# ═══════════════════════════════════════════════════════════════════════
#  2026-08-04 blocker —— 落海後爬起來的**第二趟**
#
#  SITL 不會因為機體觸海就終結它。SW1 的 UAV3 於 t=1012.6 掉到海面下 95.7 m、
#  爬回 +331 m,24.1 s 後再穿一次紅心(22.58 m);UAV5 同型(17.9 s 後
#  67.11 m)。這兩個數字一度被當成首航次寫進交接文檔,而真值是 32.62 / 217.90。
#  回升峰值只有 555.5 / 637.0 m ⇒ 1000 m 遲滯永遠收不掉;而遲滯調到 < 555 會
#  撞上 UAV3 放行時 923 m 的外衝 —— 兩邊夾死,只能靠觸地收窗。
# ═══════════════════════════════════════════════════════════════════════

def _sea_bounce_rows(sysid=1, d1_east=32.6, d2_east=22.6):
    """SW1 UAV3 的形狀:runin 擺盪 → 首航次 d1 → 觸海 → 爬起來 → 第二趟 d2。"""
    seq = [
        # 放行前:runin 段自己就有一個「先近再遠」的局部最小(1200 m)
        (0.0, 3000.0, 0.0, 1000.0, "runin"),
        (1.0, 1200.0, 0.0, 1000.0, "runin"),
        (2.0, 3000.0, 0.0, 1000.0, "runin"),
        (3.0, 2500.0, 0.0, 900.0, "wait"),
    ]
    # 首航次:100 m/s 沿正北→正南穿過,alt 每秒掉 50 m
    for k, north in enumerate(range(300, -500, -100)):
        seq.append((4.0 + k, float(north), d1_east, 300.0 - 50.0 * k, "dive"))
    # 觸海之後爬回來,再穿一次(更準,但那不是首航次)
    for k, north in enumerate(range(-500, 300, 100)):
        alt = (100.0, 200.0, 250.0, 200.0, 150.0, 100.0, 50.0, 0.0)[k]
        seq.append((12.0 + k, float(north), d2_east, alt, "dive"))
    return [_row(t, sysid, n, e, alt, m) for t, n, e, alt, m in seq]


class TestSeaBounceSecondPass:
    def test_second_pass_after_touchdown_is_not_the_first_pass(self, tmp_path,
                                                                capsys):
        p = _write_trial(tmp_path, _sea_bounce_rows(), name="tot_SB.csv")
        s, _ = _score_argv(capsys, p, "--ground", "0")
        u1 = s["per_uav"]["1"]
        assert u1["d1_m"] == pytest.approx(32.6, abs=0.1)
        assert u1["d1_m"] != pytest.approx(22.6, abs=0.1), "報了第二趟"
        assert s["window"]["per_uav"]["1"]["closed_by"] == "touchdown"

    def test_hysteresis_alone_would_have_reported_the_second_pass(self,
                                                                  tmp_path,
                                                                  capsys):
        """反證:把觸地收窗關掉(沒有任何高度基準)⇒ 遲滯收不掉 ⇒ 報 22.6。

        這是「為什麼不能只調遲滯」的證據 —— 兩趟之間的回升峰值(這裡 500 m,
        SW1 是 555/637 m)遠小於放行時的外衝,遲滯放哪裡都會壞掉一邊。
        """
        p = _write_trial(tmp_path, _sea_bounce_rows(), name="tot_SB2.csv")
        s, _ = _score_argv(capsys, p)          # 沒有 --log / --ground / agl 欄
        u1 = s["per_uav"]["1"]
        assert u1["touch_datum"] == "none"     # 前提:收窗規則 ⓪ 失效
        assert u1["d1_m"] == pytest.approx(22.6, abs=0.1)

    def test_agl_column_closes_the_window_too(self, tmp_path, capsys):
        """新 CSV 有 agl 欄時走同一條規則(基準跟著 tot_node 走)。"""
        rows = _sea_bounce_rows()
        for r in rows:
            r.append("%.3f" % (float(r[4]) - 10.0))    # agl = alt − 10
        p = _write_trial(tmp_path, rows, name="tot_SB3.csv")
        s, _ = _score_argv(capsys, p)                  # 只靠 agl 欄
        assert s["per_uav"]["1"]["alt_datum"] == "csv:agl"
        assert s["per_uav"]["1"]["d1_m"] == pytest.approx(32.6, abs=0.1)
        assert s["window"]["per_uav"]["1"]["closed_by"] == "touchdown"


# ═══════════════════════════════════════════════════════════════════════
#  2026-08-04 blocker —— gs 峰值的取樣集合
#
#  舊版只排除 wait,窗內還留著 runin(轉場,指令速度 vmax)與 expended
#  (命中鎖存後的彈道自由墜落)。SW1 UAV5 窗內逐模式最大地速:
#  runin 156.4 / wait 140.7 / dive 139.2 / expended 204.1 —— 報表印的
#  「gs峰值 204.1」是重力做出來的,不是導引也不是飛控。
#  (飛控 AIRSPEED_MAX = 150;超過它的取樣一律先懷疑取樣集合。)
# ═══════════════════════════════════════════════════════════════════════

def _gs_rows():
    seq = [
        (0.0, 3000.0, 0.0, 1000.0, "runin", 156.4),
        (1.0, 2900.0, 0.0, 1000.0, "runin", 156.4),
        (2.0, 400.0, 5.0, 500.0, "dive", 100.0),
        (3.0, 300.0, 5.0, 400.0, "dive", 139.2),
        (4.0, 200.0, 5.0, 300.0, "dive", 139.2),
        (5.0, 100.0, 5.0, 200.0, "dive", 130.0),
        (6.0, 0.0, 5.0, 150.0, "dive", 120.0),
        (7.0, -100.0, 5.0, 100.0, "expended", 204.1),
        (8.0, -200.0, 5.0, 50.0, "expended", 204.1),
        (9.0, -300.0, 5.0, 0.0, "expended", 204.1),
    ]
    out = []
    for t, north, east, alt, mode, gs in seq:
        r = _row(t, 1, north, east, alt, mode)
        r[5] = "%.1f" % gs
        out.append(r)
    return out


class TestGsPeakSampling:
    def test_peak_comes_from_dive_samples_only(self, tmp_path, capsys):
        p = _write_trial(tmp_path, _gs_rows(), name="tot_GS.csv")
        s, _ = _score_argv(capsys, p, "--ground", "0")
        u1 = s["per_uav"]["1"]
        assert u1["gs_peak_ms"] == pytest.approx(139.2, abs=0.05)
        assert u1["gs_peak_ms"] != pytest.approx(204.1, abs=0.05), \
            "expended(命中後的彈道墜落)混進峰值"
        assert u1["gs_peak_ms"] != pytest.approx(156.4, abs=0.05), \
            "runin(轉場 vmax)混進峰值"

    def test_sampling_set_is_named_in_the_report(self, tmp_path, capsys):
        """⚠️「峰值」兩個字講不出它從哪一堆取樣挑出來 —— 那正是 204.1 的縫。"""
        p = _write_trial(tmp_path, _gs_rows(), name="tot_GS2.csv")
        s, _ = _score_argv(capsys, p, "--ground", "0")
        u1 = s["per_uav"]["1"]
        assert u1["gs_peak_from"] == 'mode=="dive"'
        assert u1["gs_peak_n"] == 5
        stot_score.main([p, "--ground", "0"])
        out = capsys.readouterr().out
        assert "mode==dive" in out
        assert "expended" in out

    def test_no_dive_sample_is_none_not_zero(self, tmp_path, capsys):
        """一筆 dive 都沒有時回 None —— 不可以印 0.0 讓人以為量到了。"""
        rows = [r for r in _gs_rows() if r[7] != "dive"]
        p = _write_trial(tmp_path, rows, name="tot_GS3.csv")
        s, _ = _score_argv(capsys, p, "--ground", "0")
        assert s["per_uav"]["1"]["gs_peak_ms"] is None
        assert s["per_uav"]["1"]["gs_peak_n"] == 0


def _code_only(src):
    """把 Python 原始碼裡的 `#` 註解拿掉,只留**程式碼**(字串裡的 # 不動)。

    ⚠️⚠️ 為什麼需要這一步(2026-08-05):下面對內嵌 PY 區塊的迴歸鎖是**子字串**
       比對,而那段區塊的註解正好逐字引用了舊寫法來說明它為什麼錯
       (「舊寫法 `True if not _v else …` 的理由是…」)。直接對整段原始碼比對
       的話,那句**說明文字**會讓 `assert "True if not _v" not in block` 紅掉
       —— 測試等於在逼人刪掉最該留下的那段說明,而下一個把舊 bug 寫進註解的人
       會再踩一次。鎖要鎖住**程式碼會不會退回 fail-open**,不是鎖住文件措辭。

    ⚠️ 反過來也成立,而且更重要:剝掉註解之後,「有吼出來」那幾條
       (`"⚠️⚠️" in branch`、`"版本不符" in branch`)只有寫在 print() 裡才算數
       —— 用一句註解是滿足不了的。所以這一步同時讓鎖變**嚴**,不是放寬。

    用 tokenize 而不是 `line.split("#")`:字串字面值裡的 `#`(例如訊息本身)
    不可以被當成註解切掉,否則剝完的程式碼會殘缺,連帶讓下面的結構比對失準。
    """
    lines = src.splitlines(keepends=True)
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(src).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError) as e:
        raise AssertionError("內嵌 PY 區塊不是合法 Python,剝不掉註解:%r" % (e,))
    for tok in reversed([t for t in toks if t.type == tokenize.COMMENT]):
        (srow, scol), (erow, ecol) = tok.start, tok.end
        assert srow == erow, "註解跨行?%r" % (tok,)
        ln = lines[srow - 1]
        lines[srow - 1] = ln[:scol] + ln[ecol:]
    return "".join(lines)


# ═══════════════════════════════════════════════════════════════════════
#  判定必須三處同步:stot_score.py / stot_rescore_all.py / stot_repeat.sh
#
#  ⚠️ stot_score:648 加了「首航次全員命中」之後,另外兩處沒同步 ⇒ 同一份 JSON
#     兩支工具給出不同結論。而總表(rescore/repeat 寫的那份)才是拿來排序、
#     挑「最好的一趟」的依據 —— 漂移的方向剛好是**寬鬆**的那一邊。
# ═══════════════════════════════════════════════════════════════════════

# ═══════════════════════════════════════════════════════════════════════
#  2026-08-04 blocker ① —— ①③ 的觸地量含 MISS/ESCAPE,方向是**樂觀膨脹**
#
#  脫靶機也有「觸地」:那是它衝過紅心之後在幾百公尺外砸海,速度由重力給、
#  誤差就是脫靶量。SW1 的 UAV6 被本工具自己判成 ESCAPE(觸地誤差 1060.85 m、
#  飛出 412.4 km),impact_gs 卻是 279.9 m/s,把六機平均從 119.0 墊到 151.2
#  (+27%)⇒ 越過門檻 100 ⇒ ① 那一行看起來達標,而 fails 清單裡沒有「俯衝地速」。
#  ②(散布)早就補了 hit_touch_spread_s,①③ 是同一個病。
# ═══════════════════════════════════════════════════════════════════════

def _gs_row(t, sysid, north, east, alt, mode, gs):
    r = _row(t, sysid, north, east, alt, mode)
    r[5] = "%.1f" % gs
    return r


def _touching_pass(sysid, east, gs, escape=False):
    """一架:放行 → 俯衝 → 在 AGL=0 穿越(觸地)。gs 全程固定成 `gs`。"""
    seq = [(0.0, 3000.0, 500.0, "wait"), (1.0, 2500.0, 500.0, "wait"),
           (2.0, 400.0, 300.0, "dive"), (3.0, 300.0, 200.0, "dive"),
           (4.0, 200.0, 100.0, "dive"), (5.0, 100.0, 50.0, "dive"),
           (6.0, 0.0, 20.0, "dive"), (7.0, -100.0, -30.0, "dive")]
    out = [_gs_row(t, sysid, n, east, alt, m, gs) for t, n, alt, m in seq]
    if escape:
        # 觸地之後仍往外衝到 412 km —— SW1 UAV6 的形狀(窗外,只影響 max_d)
        out.append(_gs_row(8.0, sysid, -100.0, 30000.0, -30.0, "dive", gs))
        out.append(_gs_row(9.0, sysid, -100.0, 412000.0, -30.0, "dive", gs))
    return out


@pytest.fixture
def inflated(tmp_path):
    """兩架 HIT 觸地 80 m/s + 一架 ESCAPE 觸地 280 m/s。

    全員平均 = (80+80+280)/3 = 146.7 ≥ 門檻 100 ⇒ 舊版判 speed_ok = True;
    只算 2 架 HIT 是 80.0 < 100 ⇒ 第一優先其實**沒過**。
    """
    rows = (_touching_pass(1, east=5.0, gs=80.0)
            + _touching_pass(2, east=10.0, gs=80.0)
            + _touching_pass(3, east=800.0, gs=280.0, escape=True))
    return _write_trial(tmp_path, rows, name="tot_INF.csv")


class TestHitOnlyTouchMetrics:
    def test_escape_aircraft_inflates_the_all_hands_mean(self, inflated,
                                                         capsys):
        """前提先釘住:全員平均確實被墊到門檻之上。"""
        s, _ = _score(capsys, inflated, "--ground", "0")
        assert s["verdict"]["escape_ids"] == [3]
        assert s["touch_gs_mean_ms"] == pytest.approx(146.7, abs=0.1)
        assert s["touch_gs_mean_ms"] >= 100.0

    def test_hit_only_gs_mean_is_in_the_json(self, inflated, capsys):
        s, _ = _score(capsys, inflated, "--ground", "0")
        v = s["verdict"]
        assert v["hit_touch_gs_mean_ms"] == pytest.approx(80.0, abs=0.1)
        assert v["hit_touch_gs_min_ms"] == pytest.approx(80.0, abs=0.1)
        assert v["hit_touch_gs_max_ms"] == pytest.approx(80.0, abs=0.1)
        assert v["n_hit_touch"] == 2
        # 全員值不准消失 —— 對照歷史航次還要用它
        assert s["touch_gs_max_ms"] == pytest.approx(280.0, abs=0.1)

    def test_hit_only_max_miss_is_in_the_json(self, inflated, capsys):
        """③ 的方向相反(悲觀):全員 max 整個是脫靶機自己的脫靶量。"""
        s, _ = _score(capsys, inflated, "--ground", "0")
        v = s["verdict"]
        assert v["hit_touch_max_miss_m"] < 100.0
        assert s["touch_max_miss_m"] > 700.0      # 脫靶機主導
        assert v["hit_touch_max_miss_m"] < s["touch_max_miss_m"]

    def test_speed_verdict_follows_the_hit_only_value(self, inflated, capsys):
        """⭐ blocker 本體:fails 清單以前永遠不含速度那一項。

        ⚠️ 2026-08-14:標籤由「俯衝地速」改成「終端總速」——
           驗收量由地速改為沿航跡總速(使用者定調)。
           判定的**行為**沒有變:仍然跟隨 HIT-only 值,而這條測試鎖的就是那件事。
           標籤與行為要分開看,改標籤不等於改判定。
        """
        stot_score.main([inflated, "--ground", "0"])
        out = capsys.readouterr().out
        assert "終端總速" in out.split("判定:")[-1], \
            "HIT-only 只有 80 m/s,判定卻說第一優先過了"

    def test_both_values_are_printed_and_the_all_hands_one_is_labelled(
            self, inflated, capsys):
        stot_score.main([inflated, "--ground", "0"])
        out = capsys.readouterr().out
        one = [ln for ln in out.splitlines() if ln.startswith("①")][0]
        assert "只算 2 架 HIT" in one and "80.0" in one
        # 全員值仍在,但同一段裡要標明它含了誰、而且不是達標依據
        assert "146.7" in out
        assert "含 1 架 ESCAPE" in out
        assert "不是達標依據" in out


class TestCleanTrialFormatting:
    def test_no_hit_only_annotation_when_everyone_hits(self, tmp_path, capsys):
        """全員 HIT ⇒ HIT-only 與全員值恆等 ⇒ 不印「只算 N 架」那一套。"""
        rows = (_touching_pass(1, east=5.0, gs=120.0)
                + _touching_pass(2, east=10.0, gs=120.0))
        p = _write_trial(tmp_path, rows, name="tot_CLEAN.csv")
        stot_score.main([p, "--ground", "0"])
        out = capsys.readouterr().out
        one = [ln for ln in out.splitlines() if ln.startswith("①")][0]
        assert "只算" not in one
        assert "不是達標依據" not in out
        s, _ = _score(capsys, p, "--ground", "0")
        v = s["verdict"]
        assert v["all_hit"] is True
        # 恆等:全員 == HIT-only
        assert v["hit_touch_gs_mean_ms"] == s["touch_gs_mean_ms"]
        assert v["hit_touch_max_miss_m"] == s["touch_max_miss_m"]


# ═══════════════════════════════════════════════════════════════════════
#  2026-08-04 blocker ② —— 窗起點找不到放行點時**靜默**退回整段
#
#  整段 DIVE 都是 wait/runin = 這架從來沒被放行(逾時航次最常見的形狀)。
#  舊版靜默 return 0,於是環上盤旋的局部最小被印成
#  「UAV1 MISS 首航次d1=2100.00 t1=1.00 收窗 hysteresis」——
#  零警告、win[i] 也沒有欄位標示。而 stot_rescore_all.py:88 會把逾時航次掃進
#  總表重算,總表正是拿來排序、挑「最好的一趟」的依據。
# ═══════════════════════════════════════════════════════════════════════

def _ring_hold_only_rows(sysid=1, mode="wait"):
    """整段都在環上盤旋:d 擺盪 3000 → 2100 → 4200 → …,從未放行。"""
    seq = [(0.0, 3000.0), (1.0, 2100.0), (2.0, 4200.0),
           (3.0, 3300.0), (4.0, 2400.0), (5.0, 4500.0)]
    return [_row(t, sysid, d, 0.0, 1500.0, mode) for t, d in seq]


class TestNoReleaseFound:
    def _run(self, tmp_path, capsys, mode="wait", name="tot_NR.csv"):
        p = _write_trial(tmp_path, _ring_hold_only_rows(mode=mode), name=name)
        return _score(capsys, p, "--ground", "0"), p

    def test_start_flag_reaches_the_json(self, tmp_path, capsys):
        (s, _err), _p = self._run(tmp_path, capsys)
        assert s["window"]["per_uav"]["1"]["start"] == "no-release-found"
        assert s["per_uav"]["1"]["window_start"] == "no-release-found"

    def test_status_is_unknown_not_miss(self, tmp_path, capsys):
        """⭐ blocker 本體:2100 m 是繞圈繞出來的,不是「脫靶 2100 m」。"""
        (s, _err), _p = self._run(tmp_path, capsys)
        u1 = s["per_uav"]["1"]
        assert u1["status"] == "UNKNOWN"
        assert u1["status"] != "MISS"
        assert u1["d1_m"] == pytest.approx(2100.0, abs=1.0)  # 值還在,只是不可信
        assert s["verdict"]["unknown_ids"] == [1]
        assert s["verdict"]["n_unknown"] == 1

    def test_it_is_loud(self, tmp_path, capsys):
        (s, err), _p = self._run(tmp_path, capsys)
        assert any("no-release-found" in w or "從來沒有被放行" in w
                   for w in s["warnings"]), "靜默退回"
        assert "⚠️⚠️" in err

    def test_runin_only_is_the_same_case(self, tmp_path, capsys):
        (s, _err), _p = self._run(tmp_path, capsys, mode="runin",
                                  name="tot_NR2.csv")
        assert s["per_uav"]["1"]["status"] == "UNKNOWN"

    def test_unknown_cannot_be_a_phantom_hit(self, tmp_path, capsys):
        """環上的局部最小若剛好落進 miss_r,舊版會把繞圈印成 HIT。"""
        seq = [(0.0, 3000.0), (1.0, 120.0), (2.0, 4200.0)]
        rows = [_row(t, 1, d, 0.0, 1500.0, "wait") for t, d in seq]
        p = _write_trial(tmp_path, rows, name="tot_NR3.csv")
        s, _ = _score(capsys, p, "--ground", "0")
        assert s["per_uav"]["1"]["d1_m"] == pytest.approx(120.0, abs=1.0)
        assert s["per_uav"]["1"]["status"] == "UNKNOWN", "幽靈命中"
        assert s["verdict"]["all_hit"] is False

    def test_downstream_verdict_rejects_it(self, tmp_path, capsys):
        """總表(排序、挑最好的一趟)必須拒收 —— all_hit 為 False 就夠了。"""
        (s, _err), _p = self._run(tmp_path, capsys, name="tot_NR4.csv")
        import stot_rescore_all
        assert s["verdict"]["all_hit"] is False
        assert not stot_rescore_all.verdict(
            {"n_touch": 6, "touch_max_miss_m": 2.0, "touch_spread_s": 1.0,
             "max_abs_alt_m": 3.0, "touch_gs_mean_ms": 120.0,
             "verdict": s["verdict"]}, 100.0, 30.0)

    def test_released_tracks_are_untouched(self, trial, capsys):
        """對照組:正常航次的 start 全是 release,判定一個字都沒改。"""
        s, _ = _score(capsys, trial)
        for i in ("1", "2", "3", "4"):
            assert s["window"]["per_uav"][i]["start"] == "release"
        assert s["verdict"]["n_unknown"] == 0
        assert s["verdict"]["unknown_ids"] == []


class TestVerdictStaysInSync:
    def _summary(self, all_hit):
        return {"n_touch": 6, "touch_max_miss_m": 2.0, "touch_spread_s": 1.0,
                "max_abs_alt_m": 3.0, "touch_gs_mean_ms": 120.0,
                "verdict": {"all_hit": all_hit}}

    def test_rescore_gates_on_the_first_pass(self):
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                        "scripts", "wsl"))
        import stot_rescore_all
        assert stot_rescore_all.verdict(self._summary(True), 100.0, 30.0)
        assert not stot_rescore_all.verdict(self._summary(False), 100.0, 30.0), \
            "首航次兩架脫靶,觸地基準卻讓它過了"

    def test_rescore_rejects_a_summary_without_a_verdict(self, capsys):
        """⭐ 2026-08-05 blocker C:缺 verdict 一律 **fail-closed**。

        ⚠️ 這條測試上一版是反過來寫的(`assert verdict(...)`,標題還寫著
           「舊 JSON 沒有 verdict ⇒ 這一項不成立條件」)—— 它把 bug 本身鎖住了。
           理由現在不成立:stot_rescore_all.score() **從不傳 --no-window**,
           所以「沒有 verdict」在這裡的唯一成因是**計分器版本不符**(部署版
           stot_score.py 沒跟 repo 同步;實測 15,140 B、grep 不到 all_hit,
           卻仍接受 --json 並回 0)。工具太舊 ⇒ 首航次那一項**量不出來**,
           正確結論是不予通過並吼一聲,不是預設通過 —— 否則首航次閘門在版本
           不符時整個消失,而總表正是拿來排序、挑「最好的一趟」的依據。
        """
        import stot_rescore_all
        stot_rescore_all._MISMATCH.clear()
        s = self._summary(True)          # 其餘六項全過,唯一的變因就是 verdict
        assert stot_rescore_all.verdict(s, 100.0, 30.0) is True, "前提不成立"
        s["verdict"] = None
        assert stot_rescore_all.verdict(s, 100.0, 30.0, trial="ZZ") is False, \
            "缺 verdict 卻放行 = 首航次閘門在版本不符時整個消失"
        err = capsys.readouterr().err
        assert "版本不符" in err, "fail-closed 但靜默 = 換一種靜默失敗"
        assert "stot_score.py" in err, "沒指出要重新部署哪一支"
        assert "ZZ" in err, "沒說是哪一趟"

    def test_rescore_shouts_once_but_never_goes_silent(self, capsys):
        """掃 100 趟不可以洗版,但每一趟都必須留下痕跡。"""
        import stot_rescore_all
        stot_rescore_all._MISMATCH.clear()
        s = self._summary(True)
        s["verdict"] = None
        for t in ("A1", "A2", "A3"):
            stot_rescore_all.verdict(s, 100.0, 30.0, trial=t)
        err = capsys.readouterr().err
        assert err.count("版本不符") == 1, "整段說明重複印了"
        for t in ("A1", "A2", "A3"):
            assert t in err, "第 %s 趟被靜默吞掉" % t

    def test_score_tool_refuses_to_pass_in_no_window_mode(self, tmp_path,
                                                          capsys):
        """③ 同一個 blocker 在 stot_score.py 自己身上:--no-window 不予通過。

        舊寫法 `first_pass_ok = (not a.window) or verdict["all_hit"]` 在
        --no-window 之下恆 True ⇒ 一個「刻意不做首航次判定」、而且數字已知被
        逃逸鏈污染的診斷模式,有資格印「✅ 全項通過」。
        """
        rows = (_touching_pass(1, east=5.0, gs=120.0)
                + _touching_pass(2, east=6.0, gs=120.0))
        p = _write_trial(tmp_path, rows, name="tot_NW.csv")

        assert stot_score.main([p, "--ground", "0", "--miss-max", "50",
                                "--alt-max", "50"]) == 0
        won = capsys.readouterr().out.strip().splitlines()[-1]
        assert "首航次全員命中" not in won, "前提不成立:窗開著時這一項本來就沒過"

        assert stot_score.main([p, "--ground", "0", "--miss-max", "50",
                                "--alt-max", "50", "--no-window"]) == 0
        out = capsys.readouterr().out
        last = out.strip().splitlines()[-1]
        assert last.startswith("判定:❌"), "--no-window 印出了通過"
        # 標籤要和「真的沒有全員命中」分得開:一個是飛行結果,一個是量測模式
        assert "--no-window ⇒ 未判定,不予通過" in last, last
        # 說明必須落在哨兵**之後**,否則會被 stot_trial.sh 的收尾擷取砍掉
        assert "⚠️⚠️ --no-window" in out
        assert out.index(stot_score.SUMMARY_SENTINEL) < out.index("⚠️⚠️ --no-window")

    def test_no_window_json_is_unchanged(self, trial, capsys):
        """fail-closed 只改**判定**,不准動 --no-window 的對照數字。"""
        s, err = _score(capsys, trial, "--no-window")
        assert s["verdict"] is None
        assert s["window"]["enabled"] is False
        assert s["warnings"] == [], "模式說明跑進 warnings 了(那是資料品質欄)"
        assert s["spread_s"] == pytest.approx(5000.0, abs=0.1)

    def test_repeat_shell_block_gates_on_the_first_pass(self):
        sh = os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl",
                          "stot_repeat.sh")
        with open(sh, encoding="utf-8") as f:
            src = f.read()
        assert "first_pass_ok" in src, "stot_repeat.sh 內嵌的判定沒同步"
        assert "and first_pass_ok" in src

    def test_repeat_shell_block_fails_closed_without_a_verdict(self):
        """① 三處的第三處。內嵌區塊呼叫的是**部署版** $SR/stot_score.py,
        「部署版 != repo 版」是本專案的已知常態 ⇒ 這條路徑現在就踩得到。

        ⚠️ 從 .sh 原始碼讀,不抄一份常數 —— 抄的話 .sh 改了測試不會紅。

        ⚠️ 比對前先用 _code_only() 剝掉 `#` 註解:鎖的是**程式碼**有沒有退回
           fail-open,不是註解裡能不能引用舊寫法。那段註解正是在解釋舊寫法為
           什麼錯,它有價值,不該被這條鎖逼著刪掉。剝完之後「有吼出來」那幾條
           反而更嚴 —— 只有寫進 print() 才算數。
        """
        sh = os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl",
                          "stot_repeat.sh")
        with open(sh, encoding="utf-8") as f:
            src = f.read()
        blocks = re.findall(r"<<'PY'\n(.*?)\nPY\n", src, re.S)
        assert len(blocks) == 2, "內嵌 PY 區塊萃取失敗:%d 個" % len(blocks)
        block = _code_only(blocks[0])
        assert "True if not _v" not in block, "又退回 fail-open 了"
        # 缺 verdict 的分支必須把 first_pass_ok 設成 False,而且要吼
        m = re.search(r"if isinstance\(_v, dict\):(.*?)\nok = ", block, re.S)
        assert m, "缺 verdict 的分支結構跑掉了:\n%s" % block[-800:]
        branch = m.group(1)
        assert "first_pass_ok = False" in branch, branch
        assert "⚠️⚠️" in branch, "fail-closed 但靜默"
        assert "版本不符" in branch, branch
        # 航次自己宣告失敗(stot_trial.sh rc != 0)也要把 pass 壓成 0
        assert "TRIAL_RC" in block and 'trial_rc == "0"' in block


# ═══════════════════════════════════════════════════════════════════════
#  2026-08-05 blocker ① —— mode "missed" 讓「從未放行」偽裝成「放行後脫靶」
#
#  節點本輪新增 mode "missed",而且刻意放在 `if not released` **之前**
#  (tot_node:2004,註解:「從未放行、卡在環上不動的機一樣走得到出口」)。
#  於是從未放行的機在逾時後,DIVE 列的 mode 從 wait/runin 變成 missed,而舊的
#  排除式判準(mode not in {wait,runin})把第一筆 missed 當成放行點 ⇒
#    wait 盤旋 → missed 盤旋            ⇒ 判 MISS、start=release、零警告,
#                                         d1=2400 m(守點盤旋的局部最小)被
#                                         印成「攻擊航次脫靶 2.4 km」
#    同形狀但 miss_pt 落在紅心附近        ⇒ 判 **HIT**、all_hit **True** ⇒
#                                         first_pass_ok 在 stot_score /
#                                         stot_rescore_all.verdict /
#                                         stot_repeat.sh 三處同時變 True
#                                         = 幽靈達標(最嚴重的後果)
#  而 missed 也會出現在**放行之後**(真的俯衝、脫靶,才被 _miss_check 標記),
#  那種機的放行點必須照樣在 dive 那一筆被找到 —— 所以不能只把 missed 塞進
#  PRE_RELEASE_MODES 了事,判準得改成納入式(見 stot_score.window_start)。
# ═══════════════════════════════════════════════════════════════════════

def _orbit_rows(sysid, t0, n, centre_d, radius, mode, alt=1500.0):
    """繞著「離紅心 centre_d 公尺」的守點畫圓(半徑 radius)。

    d(t) 於是在 centre_d ± radius 之間擺盪,局部最小 = centre_d − radius。
    這正是「環上等待」與「missed 之後在 miss_pt 守點盤旋」的形狀 ——
    那個局部最小是繞圈繞出來的,不是攻擊航次的距離。
    """
    out = []
    for k in range(n):
        th = 2.0 * math.pi * k / 8.0
        out.append(_row(t0 + k, sysid, centre_d + radius * math.cos(th),
                        radius * math.sin(th), alt, mode))
    return out


def _shape_a():
    """(A) 整段 wait —— 從未放行,而且從未逾時。局部最小 2400 m。"""
    return _orbit_rows(1, 0.0, 24, 3200.0, 800.0, "wait")


def _shape_b():
    """(B) wait 盤旋 → missed 盤旋(守點遠離紅心)。局部最小 2400 m。"""
    return (_orbit_rows(1, 0.0, 12, 3200.0, 800.0, "wait")
            + _orbit_rows(1, 12.0, 12, 3200.0, 800.0, "missed"))


def _shape_c():
    """(C) 同 (B),但 miss_pt 守點落在紅心附近 ⇒ 局部最小 40 m。

    ⚠️⚠️ 這是最危險的形狀:40 m 遠小於 miss_r(300),舊判準會判 HIT。
    """
    return (_orbit_rows(1, 0.0, 12, 3200.0, 800.0, "wait")
            + _orbit_rows(1, 12.0, 12, 140.0, 100.0, "missed"))


def _shape_d():
    """(D) 正常放行 → dive 俯衝(620 m 脫靶)→ missed 守點盤旋。

    對照組:放行點必須落在**第一筆 dive**(t=8),判定不受 missed 影響。
    """
    rows = _orbit_rows(1, 0.0, 8, 3200.0, 800.0, "wait")
    for k in range(9):                      # 直線衝過紅心旁 620 m
        rows.append(_row(8.0 + k, 1, 3000.0 - 400.0 * k, 620.0,
                         1200.0 - 100.0 * k, "dive"))
    rows += _orbit_rows(1, 17.0, 16, 900.0, 200.0, "missed", alt=300.0)
    return rows


class TestMissedModeCannotForgeARelease:
    def _score_shape(self, tmp_path, capsys, rows, name):
        p = _write_trial(tmp_path, rows, name=name)
        return _score(capsys, p, "--ground", "0")

    def test_a_all_wait_stays_unknown(self, tmp_path, capsys):
        """(A) 現況正確,先釘住 —— 修 missed 不可以把這個弄壞。"""
        s, err = self._score_shape(tmp_path, capsys, _shape_a(), "tot_MA.csv")
        u1 = s["per_uav"]["1"]
        assert u1["status"] == "UNKNOWN"
        assert u1["window_start"] == "no-release-found"
        assert s["verdict"]["all_hit"] is False
        assert "⚠️⚠️" in err

    def test_b_wait_then_missed_is_unknown_not_a_2400m_miss(self, tmp_path,
                                                             capsys):
        """(B) ⭐ 舊版:status=MISS、start=release、n_unknown=0、**警告消失**。

        2400 m 是守點盤旋的局部最小,被印成「攻擊航次脫靶 2.4 km」。
        """
        s, err = self._score_shape(tmp_path, capsys, _shape_b(), "tot_MB.csv")
        u1 = s["per_uav"]["1"]
        assert u1["status"] == "UNKNOWN", "missed 被當成放行點"
        assert u1["status"] != "MISS"
        assert u1["window_start"] == "no-release-found"
        assert s["verdict"]["n_unknown"] == 1
        assert s["verdict"]["unknown_ids"] == [1]
        assert s["verdict"]["all_hit"] is False
        # 值還在(不要整架丟掉),但必須被標成不可信
        assert u1["d1_m"] == pytest.approx(2400.0, abs=1.0)
        assert any("從來沒有被放行" in w for w in s["warnings"]), "警告消失"
        assert "⚠️⚠️" in err

    def test_c_a_guard_orbit_over_the_bullseye_is_not_a_hit(self, tmp_path,
                                                             capsys):
        """(C) ⭐⭐ 最嚴重的後果:幽靈達標。

        守點盤旋的局部最小 40 m 遠小於 miss_r(300)⇒ 舊版判 HIT、all_hit=True
        ⇒ first_pass_ok 在三處同時變 True,一趟「從未放行」的航次被排進
        「最好的一趟」。
        """
        s, _err = self._score_shape(tmp_path, capsys, _shape_c(), "tot_MC.csv")
        u1 = s["per_uav"]["1"]
        assert u1["status"] == "UNKNOWN", "幽靈命中"
        assert u1["status"] != "HIT"
        assert s["verdict"]["all_hit"] is False, "幽靈達標:all_hit 被撐成 True"
        assert s["verdict"]["hit_ids"] == []
        assert any("從來沒有被放行" in w for w in s["warnings"])

    def test_c_downstream_first_pass_ok_stays_false(self, tmp_path, capsys):
        """三處判定同步:總表(排序、挑最好的一趟)也必須拒收這一趟。"""
        s, _err = self._score_shape(tmp_path, capsys, _shape_c(),
                                    "tot_MC2.csv")
        import stot_rescore_all
        assert not stot_rescore_all.verdict(
            {"n_touch": 6, "touch_max_miss_m": 2.0, "touch_spread_s": 1.0,
             "max_abs_alt_m": 3.0, "touch_gs_mean_ms": 120.0,
             "verdict": s["verdict"]}, 100.0, 30.0)

    def test_c_the_old_exclusion_rule_would_have_forged_the_hit(
            self, tmp_path, capsys, monkeypatch):
        """反證:把判準換回排除式,同一份 CSV 立刻變成 HIT + all_hit=True。

        ⚠️ 這條是「為什麼不能只把 missed 加進 PRE_RELEASE_MODES」的另一半 ——
           排除式**本身**就是病灶,加不加 missed 都會在下一個新 mode 上重演。
        """
        def exclusion_rule(pts, _pre=("wait", "runin")):
            for k, p in enumerate(pts):
                if p[stot_score.P_MODE] not in _pre:
                    return k, stot_score.START_RELEASE
            return 0, stot_score.START_NO_RELEASE

        monkeypatch.setattr(stot_score, "window_start", exclusion_rule)
        s, _err = self._score_shape(tmp_path, capsys, _shape_c(),
                                    "tot_MC3.csv")
        assert s["per_uav"]["1"]["status"] == "HIT"
        assert s["verdict"]["all_hit"] is True

    def test_d_released_then_missed_finds_the_real_release_point(self, tmp_path,
                                                                  capsys):
        """(D) 對照組:放行點在第一筆 dive(t=8),判定一個字都沒改。"""
        s, _err = self._score_shape(tmp_path, capsys, _shape_d(), "tot_MD.csv")
        u1, w1 = s["per_uav"]["1"], s["window"]["per_uav"]["1"]
        assert w1["start"] == "release"
        assert w1["t_start"] == pytest.approx(8.0, abs=0.01)
        assert u1["status"] == "MISS", "放行後脫靶被降級成 UNKNOWN"
        assert u1["d1_m"] == pytest.approx(620.0, abs=1.0), "咬到守點盤旋"
        assert s["verdict"]["n_unknown"] == 0

    def test_unknown_mode_gets_its_own_loud_warning(self, tmp_path, capsys):
        """節點改名時,UNKNOWN 讀起來像「這趟沒放行」,而真因是工具脫節 ——
        兩者的處置完全不同,要分得開。"""
        rows = [_row(t, 1, d, 0.0, 1500.0, "teleport")
                for t, d in ((0, 3000.0), (1, 40.0), (2, 4200.0))]
        p = _write_trial(tmp_path, rows, name="tot_MU.csv")
        s, err = _score(capsys, p, "--ground", "0")
        assert s["per_uav"]["1"]["status"] == "UNKNOWN"
        assert any("不認得的 mode" in w and "teleport" in w
                   for w in s["warnings"]), "mode 詞彙脫節被靜默吞掉"
        assert "teleport" in err

    def test_a_normal_trial_raises_no_vocabulary_complaint(self, trial, capsys):
        """對照組:正常航次不可以被這條新警告吵。"""
        s, _err = _score(capsys, trial)
        assert not [w for w in s["warnings"] if "不認得的 mode" in w]


# ═══════════════════════════════════════════════════════════════════════
#  2026-08-05 blocker ② —— degraded 路徑的行數撐爆 `tail -12` 的固定預算
#
#  stot_trial.sh:443 以前是 `stot_score.py … | tail -12`。degraded 區塊固定
#  8 行 + 每架 MISS/ESCAPE/UNKNOWN 一則警告 + 判定 1 行:
#    2 架 ESCAPE(SW1 形狀)= 剛好 12 行、**零餘裕**
#    3 架 ESCAPE          = 13 行,被砍掉的第一行正好是 ①「只算 N 架 HIT」
#                           ⇒ 收尾畫面從「⚠️ 全員值 …」開頭,正是本輪特地要
#                             降級的灌水數字,而降級後的真值被砍掉
#  把 12 改成 20 只是把線往後挪。改成哨兵擷取,行數自動伸縮。
# ═══════════════════════════════════════════════════════════════════════

def _escape_fleet(n_escape):
    """2 架乾淨 HIT + n 架 ESCAPE —— 每架 ESCAPE 各帶一則專屬警告。"""
    rows = (_touching_pass(1, east=5.0, gs=80.0)
            + _touching_pass(2, east=10.0, gs=80.0))
    for k in range(n_escape):
        rows += _touching_pass(3 + k, east=800.0 + 50.0 * k, gs=280.0,
                               escape=True)
    return rows


def _harness_tail(out):
    """複製 stot_trial.sh 的收尾擷取:從哨兵的**下一行**取到結尾。"""
    lines = out.splitlines()
    assert stot_score.SUMMARY_SENTINEL in lines, "哨兵不見了"
    return lines[lines.index(stot_score.SUMMARY_SENTINEL) + 1:]


class TestSummarySentinel:
    def _text(self, tmp_path, capsys, n_escape):
        p = _write_trial(tmp_path, _escape_fleet(n_escape),
                         name="tot_ESC%d.csv" % n_escape)
        assert stot_score.main([p, "--ground", "0"]) == 0
        return capsys.readouterr().out

    def test_sentinel_precedes_the_summary_in_text_mode(self, tmp_path,
                                                        capsys):
        out = self._text(tmp_path, capsys, 2)
        tail = _harness_tail(out)
        assert any(ln.startswith("①") for ln in tail)
        assert any(ln.startswith("判定:") for ln in tail)
        # 哨兵之前的東西(逐機表)不該混進收尾
        assert not any(ln.startswith("目標 ") for ln in tail)

    def test_sentinel_never_pollutes_json_stdout(self, tmp_path, capsys):
        """--json 的 stdout 必須是純 JSON,下游 jq / stot_repeat.sh 會噎到。"""
        p = _write_trial(tmp_path, _escape_fleet(2), name="tot_ESCJ.csv")
        s, _err = _score(capsys, p, "--ground", "0")   # _score 會 json.loads
        assert s["verdict"]["n_escape"] == 2

    @pytest.mark.parametrize("n_escape", [2, 3, 4])
    def test_the_hit_only_line_survives_any_escape_count(self, tmp_path,
                                                          capsys, n_escape):
        """⭐ blocker 本體:① 那一行在收尾擷取中必須存活,不管幾架 ESCAPE。"""
        out = self._text(tmp_path, capsys, n_escape)
        tail = _harness_tail(out)
        one = [ln for ln in tail if ln.startswith("①")]
        assert one, "① 被收尾擷取砍掉了"
        assert "只算 2 架 HIT" in one[0]
        assert "80.0" in one[0], "HIT-only 的真值不見了"
        # 每架 ESCAPE 的專屬警告也要在
        for i in range(3, 3 + n_escape):
            assert any("UAV%d" % i in ln for ln in tail), i
        assert tail[-1].startswith("判定:")

    @pytest.mark.parametrize("n_escape,first_kept", [(3, "全員值"),
                                                      (4, "mode==dive")])
    def test_the_old_line_budget_would_have_cut_it(self, tmp_path, capsys,
                                                    n_escape, first_kept):
        """反證:舊的 `tail -12` 從 3 架 ESCAPE 起就砍掉 ①,而且越多越糟。

        3 架:收尾畫面的第一行變成「⚠️ 全員值 …」—— 被脫靶機自由墜落墊高的
              灌水數字,正是本輪特地要降級的那個,而降級後的真值被砍掉。
        4 架:再往下滑一行,連那句灌水警告都被砍掉,開頭變成一句沒有主詞的
              「參考:mode==dive 取樣的峰值地速 …」。
        兩種都是「開頭是半句話」,所以真正的病是**固定行數**,不是 12 這個值。
        """
        out = self._text(tmp_path, capsys, n_escape)
        old = out.splitlines()[-12:]
        assert not any(ln.startswith("①") for ln in old), \
            "前提不成立:這個形狀沒有撐爆 12 行預算"
        assert first_kept in old[0]
        # 開頭是縮排的續行 = 半句話,讀者看不到它在講哪一項
        assert old[0].startswith(" "), old[0]
        # 哨兵擷取則完整保住
        assert any(ln.startswith("①") for ln in _harness_tail(out))

    def test_the_summary_block_grows_past_twelve_lines(self, tmp_path, capsys):
        """把「預算已經不夠用」這件事本身釘住,免得下次又用行數思考。"""
        n2 = len(_harness_tail(self._text(tmp_path, capsys, 2)))
        n4 = len(_harness_tail(self._text(tmp_path, capsys, 4)))
        assert n4 > n2, "收尾區塊不隨警告數成長?那這條測試沒在測東西"
        assert n4 > 12

    def test_the_harness_extracts_by_sentinel_not_by_line_count(self):
        """契約:stot_score 印哨兵、stot_trial.sh 從哨兵取 —— 兩邊要對得上。"""
        sh = os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl",
                          "stot_trial.sh")
        with open(sh, encoding="utf-8") as f:
            src = f.read()
        assert stot_score.SUMMARY_SENTINEL in src, \
            "harness 沒用哨兵(或哨兵字串兩邊不一致)"
        calls = [ln for ln in src.splitlines()
                 if "stot_score.py" in ln and not ln.lstrip().startswith("#")]
        assert calls, "找不到 harness 呼叫 stot_score 的那一行"
        for ln in calls:
            assert "tail -12" not in ln, ln
            assert not re.search(r"\|\s*tail\s+-\d+\s*$", ln), \
                "又回到固定行數預算:%s" % ln

    def test_the_harness_is_loud_when_the_sentinel_is_missing(self):
        """⚠️ 哨兵不見時**絕不可以靜默印空白** —— 那看起來像「跑完了」。"""
        sh = os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl",
                          "stot_trial.sh")
        with open(sh, encoding="utf-8") as f:
            src = f.read()
        assert "⚠️⚠️ stot_score 收尾哨兵" in src, "哨兵不見時沒有退路"
        assert "cat \"$SCORE_TXT\"" in src, "退路沒有印全文"

    def test_kpi_loop_has_no_score_truncation_budget(self):
        """一併檢查另一支 harness:它根本沒呼叫 stot_score(只 tail CSV 一行),
        所以沒有同型問題 —— 把這個結論釘住,免得以後加上去時漏改。"""
        sh = os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl",
                          "stot_kpi_loop.sh")
        with open(sh, encoding="utf-8") as f:
            src = f.read()
        if "stot_score.py" in src:
            assert stot_score.SUMMARY_SENTINEL in src, \
                "stot_kpi_loop.sh 開始用 stot_score 了,收尾也要改成哨兵擷取"


# ═══════════════════════════════════════════════════════════════════════
#  2026-08-05 blocker —— 高度基準有**三個**決定點,不是註解宣稱的「一次決定」
#
#  收窗用的 win_agl 由**整段** full 判;觸地掃描、殘餘高度、alt_datum/
#  touch_datum 標籤由**窗內** pts 判;partial-agl 警告也只看 pts。
#  ⇒ agl 欄只要在**窗外**缺值(kill -KILL 截掉末列、新舊 CSV 併檔、float()
#    解析失敗走 except ValueError),收窗就退回 alt−z_g,而報表照樣標 csv:agl,
#    而且**不會**印 partial-agl 警告。
#
#  實測(下面 _datum_rows 這份合成 CSV,--ground 0):
#    收窗用 alt−0 在 t=10 判「觸地」收窗,agl 欄要到 t=22 才歸零
#    ⇒ 窗提早 10 個取樣收掉 ⇒ 一架**正對紅心通過**的機被判
#      d1=1000.00 m / MISS / impact_miss_m=None / n_touch=0,零警告。
#  這是「同一份報表兩套基準而且標錯」搬到收窗層重演,方向不保證保守。
# ═══════════════════════════════════════════════════════════════════════

def _datum_rows(hole, sysid=1, n=41):
    """兩套基準相差 1200 m 的直線通過,所以「用哪一套收窗」在數字上分得開。

      north = 2000 − 100 t   ⇒ t=20 正對紅心(east 恆 0)
      alt   = 1000 − 100 t   ⇒ alt−0(--ground 0)在 t=10 歸零
      agl   = alt + 1200     ⇒ agl 欄在 t=22 才歸零
      mode  = wait(t<5)→ dive ⇒ 放行點在 t=5
    `hole(t)` 為真的那幾列把 agl 欄挖空。
    """
    rows = []
    for t in range(n):
        alt = 1000.0 - 100.0 * t
        r = _row(t, sysid, 2000.0 - 100.0 * t, 0.0, alt,
                 "wait" if t < 5 else "dive", agl=alt + 1200.0)
        if hole(t):
            r[-1] = ""                       # 最後一欄就是 agl
        rows.append(r)
    return rows


class TestDatumIsDecidedOnce:
    """基準只能有**一個**決定點,而且要涵蓋所有使用點:收窗、觸地、殘餘高度、標籤。"""

    def _run(self, tmp_path, capsys, hole, name, *extra):
        p = _write_trial(tmp_path, _datum_rows(hole), name=name)
        return _score_argv(capsys, p, "--ground", "0", *extra)

    # ── 形狀 ①:缺值全在**窗外**(放行前 5 列)—— blocker 的原始形狀 ──────
    def test_missing_before_release_no_longer_truncates_the_window(
            self, tmp_path, capsys):
        """⭐ 修正前:MISS / d1=1000.00 / n_touch=0 / 標 csv:agl / 零警告。

        agl 欄從放行點(t=5)之後是完整的,評分範圍內每一筆都有值 ⇒ 基準就是
        csv:agl,收窗也必須用 csv:agl ⇒ 窗開到 t=22 才收,那架正對紅心通過的
        機拿回它的 d1=0。
        """
        s, _err = self._run(tmp_path, capsys, lambda t: t < 5, "tot_DA.csv")
        u1, w1 = s["per_uav"]["1"], s["window"]["per_uav"]["1"]
        assert u1["status"] == "HIT", "窗被另一套基準提早收掉了"
        assert u1["d1_m"] == pytest.approx(0.0, abs=0.05)
        assert w1["closed_by"] == "touchdown"
        assert w1["t_end"] == pytest.approx(22.0, abs=0.05), \
            "收窗時刻 %s 不是 agl 欄歸零的 t=22 ⇒ 收窗用的不是報表宣稱的基準" \
            % w1["t_end"]
        assert u1["alt_datum"] == u1["touch_datum"] == "csv:agl"
        assert s["n_touch"] == 1 and u1["impact_miss_m"] is not None

    def test_missing_before_release_is_still_loud(self, tmp_path, capsys):
        """基準沒退,但缺值本身仍要出聲 —— 那是 CSV 被截斷/併檔的唯一線索。"""
        _s, err = self._run(tmp_path, capsys, lambda t: t < 5, "tot_DB.csv")
        assert "agl" in err and "5/41" in err, err

    # ── 形狀 ②:缺值在**窗尾之後**(kill -KILL 截斷末列)────────────────
    def test_truncated_tail_falls_back_but_the_label_says_so(self, tmp_path,
                                                             capsys):
        """退回是可以的(範圍內缺值就是退回),**靜默**不行,標錯更不行。

        修正前:收窗用 alt−0(t=10 收)、報表標 csv:agl、觸地掃描用 agl 掃不到
        ⇒ n_touch=0、impact=None、零警告 —— 一份報表兩套基準而且標錯。
        """
        s, err = self._run(tmp_path, capsys, lambda t: t >= 30, "tot_DC.csv")
        u1, w1 = s["per_uav"]["1"], s["window"]["per_uav"]["1"]
        assert u1["alt_datum"] == u1["touch_datum"] == "arg:--ground"
        assert w1["closed_by"] == "touchdown"
        assert w1["t_end"] == pytest.approx(10.0, abs=0.05)
        # 標籤說 arg:--ground,收窗與觸地就都必須是 arg:--ground 那一套:
        # 窗被「觸地」收掉,同一套基準的觸地掃描就一定掃得到那個穿越點。
        assert u1["impact_miss_m"] is not None, \
            "窗說觸地、觸地掃描說沒觸地 ⇒ 兩套基準又分岔了"
        assert u1["impact_miss_m"] == pytest.approx(1000.0, abs=1.0)
        assert "退回 arg:--ground" in err, err

    # ── 形狀 ③(反向):缺值在**窗內** ───────────────────────────────────
    def test_missing_inside_the_window_falls_back_and_says_so(self, tmp_path,
                                                              capsys):
        s, err = self._run(tmp_path, capsys, lambda t: t in (8, 9),
                           "tot_DD.csv")
        u1 = s["per_uav"]["1"]
        assert u1["alt_datum"] == u1["touch_datum"] == "arg:--ground"
        # 筆數要是**決定範圍**(放行點之後 36 筆)的,不是窗內那 6 筆 ——
        # 統計範圍與決定範圍不一致正是上一版的病灶。
        assert "只有 34/36 筆有值" in err and "評分範圍" in err, err

    # ── 不管哪一種形狀,標籤與**實際用的那一套**都要對得上 ────────────────
    @pytest.mark.parametrize("hole,name,want", [
        (lambda t: t < 5, "tot_DE.csv", "csv:agl"),
        (lambda t: t >= 30, "tot_DF.csv", "arg:--ground"),
        (lambda t: t in (8, 9), "tot_DG.csv", "arg:--ground"),
        (lambda t: False, "tot_DH.csv", "csv:agl"),
    ])
    def test_the_label_is_the_datum_that_closed_the_window(self, tmp_path,
                                                           capsys, hole,
                                                           name, want):
        """收窗時刻直接指認基準:csv:agl 收在 t=22、alt−0 收在 t=10。"""
        s, _err = self._run(tmp_path, capsys, hole, name)
        u1, w1 = s["per_uav"]["1"], s["window"]["per_uav"]["1"]
        assert u1["alt_datum"] == u1["touch_datum"] == want
        assert s["window"]["alt_datum"] == s["window"]["touch_datum"] == want
        assert w1["closed_by"] == "touchdown"
        assert w1["t_end"] == pytest.approx(22.0 if want == "csv:agl" else 10.0,
                                            abs=0.05)

    @pytest.mark.parametrize("hole,name", [
        (lambda t: t < 5, "tot_DI.csv"),
        (lambda t: t >= 30, "tot_DJ.csv"),
        (lambda t: t in (8, 9), "tot_DK.csv"),
    ])
    def test_every_hole_produces_a_warning(self, tmp_path, capsys, hole, name):
        """缺值**必有**警告 —— 不管缺在窗內、窗外還是窗尾之後。"""
        _s, err = self._run(tmp_path, capsys, hole, name)
        assert [w for w in err.splitlines() if "agl" in w], \
            "agl 欄有缺值卻一句警告都沒有:%r" % err

    def test_a_complete_column_stays_quiet_and_uses_it(self, tmp_path, capsys):
        """完整的 agl 欄不要被這條規則吵到(退路也不是備案而是主要路徑)。"""
        s, err = self._run(tmp_path, capsys, lambda t: False, "tot_DL.csv")
        assert s["per_uav"]["1"]["alt_datum"] == "csv:agl"
        assert not [w for w in err.splitlines() if "agl 欄" in w], err

    def test_no_window_widens_the_scope_to_the_whole_track(self, tmp_path,
                                                           capsys):
        """--no-window 時被評分的是整段 ⇒ 決定範圍也是整段(放行前也算)。

        範圍要跟著**使用點**走,這正是修正的重點:兩者不一致就會標錯。
        """
        s, err = self._run(tmp_path, capsys, lambda t: t < 5, "tot_DM.csv",
                           "--no-window")
        assert s["per_uav"]["1"]["alt_datum"] == "arg:--ground"
        assert "退回 arg:--ground" in err, err

    def test_the_scope_rule_is_printed(self, tmp_path, capsys):
        """規則本身要進報表:讀者要看得出這一欄是用什麼範圍決定的。"""
        s, _err = self._run(tmp_path, capsys, lambda t: False, "tot_DN.csv")
        assert "datum_rule" in s["window"]
        assert "放行點之後" in s["window"]["datum_rule"]

    def test_no_datum_at_all_still_disables_the_touchdown_rule(self, tmp_path,
                                                               capsys):
        """三個來源一個都沒有時,收窗規則 ⓪ 整條失效而不是把 AGL 當 0。"""
        rows = [_row(t, 1, 2000.0 - 100.0 * t, 0.0, 1000.0 - 100.0 * t,
                     "wait" if t < 5 else "dive") for t in range(41)]
        p = _write_trial(tmp_path, rows, name="tot_DO.csv")
        s, err = _score_argv(capsys, p)          # 沒有 --log、沒有 --ground
        assert s["per_uav"]["1"]["touch_datum"] == "none"
        assert s["window"]["per_uav"]["1"]["closed_by"] != "touchdown"
        assert "算不出來" in err


# ═══════════════════════════════════════════════════════════════════════
#  2026-08-05 blocker —— stot_rescore_all 吞例外 ⇒ 成功率系統性灌高
#
#  `except Exception: return None` + 呼叫端 `if s is None: continue` 把整趟從
#  總表移除,而被移除的一律是「CSV 壞掉 / 空檔 / 逾時報廢 / 目標座標找不到」
#  這一類**失敗**航次;批次統計的分母只算得分成功的那些 ⇒ 成功率只會被墊高,
#  方向永遠樂觀。結尾那句「重算 N 趟」印的也是存活數,讀者看不出少了誰。
#  而這支正是「引用歷史成功率之前先跑」的工具。
# ═══════════════════════════════════════════════════════════════════════

def _clean_dive(sysid, east, t0, n=21):
    """100 m/s 垂直穿過:k=10 時 north=0 且 agl=0(east=0 就是正中紅心)。"""
    out = []
    for k in range(n):
        alt = 1000.0 - 100.0 * k
        out.append(_row(t0 + k, sysid, 1000.0 - 100.0 * k, east, alt,
                        "wait" if k < 2 else "dive", agl=alt))
    return out


@pytest.fixture(scope="module")
def broken_runs(tmp_path_factory):
    """2 趟算得出來(1 過 1 不過)+ 3 趟計分失敗,每種失敗一個分類。"""
    d = tmp_path_factory.mktemp("runs")
    # G1:六機全中 ⇒ 全項過
    rows = []
    for i in range(1, 7):
        rows += _clean_dive(i, 0.0, 0.5 * (i - 1))
    _write_trial(d, rows, name="tot_G1.csv")
    # G2:UAV6 在 800 m 外掠過 ⇒ 首航次沒有全員命中 ⇒ 不過(但算得出來)
    rows = []
    for i in range(1, 6):
        rows += _clean_dive(i, 0.0, 0.5 * (i - 1))
    rows += _clean_dive(6, 800.0, 2.5)
    _write_trial(d, rows, name="tot_G2.csv")
    # B1:空檔,連 *_geom.csv 都沒有 ⇒ 找不到目標座標
    (d / "tot_B1.csv").write_text("", encoding="utf-8")
    # B2:有 geom、有表頭,但一列 DIVE 都沒有(航次在 DIVE 之前就報廢)
    _write_trial(d, [_row(0.0, 1, 100.0, 0.0, 500.0, "weave", phase="ALIGN",
                          agl=500.0)], name="tot_B2.csv")
    # B3:CSV 被寫壞(alt 欄是垃圾)⇒ stot_score 拋 ValueError、非零離開
    bad = _clean_dive(1, 0.0, 0.0)
    bad[3][4] = "NaN-ish"
    _write_trial(d, bad, name="tot_B3.csv")
    return str(d)


def _rescore(capsys, runs, *extra):
    import stot_rescore_all
    assert stot_rescore_all.main(["--runs", runs, "--speed-min", "90"]
                                 + list(extra)) == 0
    return capsys.readouterr().out


class TestRescoreKeepsFailedTrials:
    def test_failed_trials_stay_in_the_table(self, broken_runs, capsys):
        """壞掉的航次照樣佔一行,而且看得出「不是沒跑,是算不出來」。"""
        out = _rescore(capsys, broken_runs)
        for t in ("B1", "B2", "B3", "G1", "G2"):
            assert re.search(r"^%s\s" % t, out, re.M), "%s 從總表消失了" % t
        assert out.count("計分失敗:") == 3

    def test_the_denominator_includes_them(self, broken_runs, capsys):
        """⭐ 分母含失敗列:1/5 而不是 1/2 —— 差別就是 20% 對 50%。"""
        out = _rescore(capsys, broken_runs)
        assert "1/5 趟通過" in out, out
        assert re.search(r"^B\s+3\s+0/3\s+3", out, re.M), \
            "批次 B 的分母不是 3(失敗列被踢出分母了)"

    def test_the_closing_line_counts_both(self, broken_runs, capsys):
        out = _rescore(capsys, broken_runs)
        assert "重算 5 趟" in out
        assert "2 趟成功計分" in out and "3 趟**計分失敗**" in out

    def test_reasons_are_classified_and_listed(self, broken_runs, capsys):
        out = _rescore(capsys, broken_runs)
        assert "計分失敗分類" in out
        for kind in ("no-target", "no-phase-data", "score-failed"):
            assert kind in out, "%s 這一類沒被印出來:%s" % (kind, out)

    def test_a_failed_trial_can_never_be_counted_as_a_pass(self, broken_runs,
                                                           capsys):
        out = _rescore(capsys, broken_runs)
        for ln in out.splitlines():
            if "計分失敗:" in ln:
                assert "✅" not in ln, ln

    def test_the_saved_csv_keeps_them_too(self, broken_runs, capsys, tmp_path):
        """--csv 產出的總表正是拿去排序、挑「最好的一趟」的東西。"""
        outp = tmp_path / "rescored.csv"
        _rescore(capsys, broken_runs, "--csv", str(outp))
        with open(outp, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        assert [r["trial"] for r in rows] == ["B1", "B2", "B3", "G1", "G2"]
        for r in rows:
            if r["trial"].startswith("B"):
                assert r["pass"] == "0" and r["reason"], r
            else:
                assert r["reason"] == ""

    def test_the_speed_record_ignores_failures_without_dropping_them(
            self, broken_runs, capsys):
        """速度紀錄只能從算得出來的列挑,但那不准把失敗列踢出分母。"""
        out = _rescore(capsys, broken_runs)
        assert "速度紀錄" in out and "G1" in out
        assert "1/5 趟通過" in out


def _full_summary(n_touch=6):
    """一份「欄位齊全」的 summary —— 鍵從 REQUIRED_SUMMARY_KEYS 推導,不要手抄。

    手抄清單就是這條測試在防的那個病:守衛涵蓋 6 個鍵、下游下標 11 個,
    測試若自己寫死一份就會跟著漏,而且漏得跟被測物一模一樣(同源錯誤)。
    """
    import stot_rescore_all
    s = {k: 0.0 for k in stot_rescore_all.REQUIRED_SUMMARY_KEYS}
    s["n_touch"] = n_touch
    s["n"] = 6
    return s


class TestRescoreScoreReportsWhy:
    """score() 不吞例外:每一種失敗都帶著自己的分類回去。"""

    def _fake_run(self, monkeypatch, **kw):
        import stot_rescore_all

        class P:
            returncode = kw.get("rc", 0)
            stdout = kw.get("stdout", "")
            stderr = kw.get("stderr", "")

        def run(*_a, **_k):
            if kw.get("raise_"):
                raise kw["raise_"]
            return P()
        monkeypatch.setattr(stot_rescore_all.subprocess, "run", run)

    def test_missing_csv_is_its_own_reason(self, tmp_path):
        import stot_rescore_all
        s, why = stot_rescore_all.score("NOPE", 100.0, str(tmp_path))
        assert s is None and why[0] == stot_rescore_all.R_NO_CSV

    def test_truncated_stdout_is_bad_json(self, tmp_path, monkeypatch):
        import stot_rescore_all
        (tmp_path / "tot_X.csv").write_text("x", encoding="utf-8")
        self._fake_run(monkeypatch, rc=0, stdout='{"n_touch": 6')
        s, why = stot_rescore_all.score("X", 100.0, str(tmp_path))
        assert s is None and why[0] == stot_rescore_all.R_BAD_JSON

    def test_a_summary_without_n_touch_is_rejected(self, tmp_path, monkeypatch):
        """半個 JSON 混進總表,後面每個取欄位的地方都會炸(或更糟:靜默 None)。

        ⚠️ 2026-08-05:這一條以前斷言 R_BAD_JSON。缺欄位的 dict 現在歸類成
           **版本不符**(R_VERSION_BAD)—— 兩者的修法不同:壞 JSON 是這趟資料
           爛了,缺欄位是**工具太舊**,要重新部署再重算。分類本來就是為了讓
           讀者分得出「該回頭修資料還是修工具」。
        """
        import stot_rescore_all
        (tmp_path / "tot_X.csv").write_text("x", encoding="utf-8")
        self._fake_run(monkeypatch, rc=0, stdout='{"hello": 1}')
        s, why = stot_rescore_all.score("X", 100.0, str(tmp_path))
        assert s is None and why[0] == stot_rescore_all.R_VERSION_BAD
        assert "n_touch" in why[1], why

    def test_a_json_that_is_not_an_object_is_bad_json(self, tmp_path,
                                                      monkeypatch):
        """合法 JSON 但不是 dict(list / 數字)⇒ 那不是版本問題,是壞資料。"""
        import stot_rescore_all
        (tmp_path / "tot_X.csv").write_text("x", encoding="utf-8")
        self._fake_run(monkeypatch, rc=0, stdout='[1, 2, 3]')
        s, why = stot_rescore_all.score("X", 100.0, str(tmp_path))
        assert s is None and why[0] == stot_rescore_all.R_BAD_JSON

    def test_timeout_is_not_silently_dropped(self, tmp_path, monkeypatch):
        import stot_rescore_all
        import subprocess as sp
        (tmp_path / "tot_X.csv").write_text("x", encoding="utf-8")
        self._fake_run(monkeypatch, raise_=sp.TimeoutExpired("cmd", 180))
        s, why = stot_rescore_all.score("X", 100.0, str(tmp_path))
        assert s is None and why[0] == stot_rescore_all.R_TIMEOUT

    def test_a_good_run_returns_no_reason(self, tmp_path, monkeypatch):
        import stot_rescore_all
        (tmp_path / "tot_X.csv").write_text("x", encoding="utf-8")
        self._fake_run(monkeypatch, rc=0, stdout=json.dumps(_full_summary()))
        s, why = stot_rescore_all.score("X", 100.0, str(tmp_path))
        assert why is None and s["n_touch"] == 6

    def test_a_numeric_trial_name_does_not_kill_the_whole_table(self):
        """批次前綴抓不到就標 "?" —— 不要讓一個名字把整份總表 AttributeError 掉。"""
        import stot_rescore_all
        assert stot_rescore_all._batch_of("1A") == "?"
        assert stot_rescore_all._batch_of("SW1") == "SW"
