# -*- coding: utf-8 -*-
"""避撞高度棘輪的值層級守衛(2026-08-25)。

現象:3D 畫面上四條航線裡**兩條特別高**。量下去:規劃航高 120 m,
而 UAV-2 飛在 **360 m**、UAV-4 在 180 m。

機制:`_resolve_altitude` 每輪把衝突配對中 ID 較大的那架 `alt += alt_offset`,
**而且是累加的**;衝突沒解掉就再抬一輪,最多 8 輪 ⇒ 120 + 10×30 = 360。

⚠️ 這不只是難看:掃描寬度(138.6 m)與中心 GSD(3.46 cm/px)都是照 120 m 算的,
飛在 3 倍高度上拍回來的東西不符合使用者設定的目標 GSD。

兩個修法都鎖在這裡:
  ① 面板真的把 `alt_offset_m` / `max_alt_offset_m` 送出去(以前**完全沒送** ⇒
     吃寫死預設 30,而面板最小間距預設 50 ⇒ 30 < 50 無條件成立)
  ② 累加上限:撞到上限就**停止抬升**並大聲失敗,不再無界棘輪

⚠️⚠️ 這兩個修法**不會解掉衝突**,只是讓失控停下來並誠實回報。實測剩餘衝突
發生的當下兩架都在 120 m(Δalt=0)⇒ 偏移根本沒落在衝突點上。那是另一個 bug,
不在本檔範圍;本檔只鎖「高度不會再無界上漲」與「解不掉時要大聲」。
"""

import logging

import pytest

from core.dccpp.collision_avoidance import CollisionAvoidance


class _WP:
    """最小航點替身:避撞只碰 lat/lon/alt/segment_type。"""

    def __init__(self, lat, lon, alt, seg='OPERATION'):
        self.lat, self.lon, self.alt = lat, lon, alt
        self.segment_type = type('S', (), {'name': seg})()


class _Path:
    def __init__(self, wps):
        self.waypoints = wps


def _overlapping_pair(alt=120.0, n=40):
    """兩條**完全重疊**的航跡 —— 高度錯開的最壞情況。

    ⚠ 這個 fixture 必須真的能讓棘輪發生,否則守衛就是
    [[instrument-that-cannot-fire]]。下面 test_fixture_really_ratchets 斷言這件事。
    """
    wps_a = [_WP(25.07 + i * 0.0005, 121.22, alt) for i in range(n)]
    wps_b = [_WP(25.07 + i * 0.0005, 121.22, alt) for i in range(n)]
    return {1: _Path(wps_a), 2: _Path(wps_b)}


def _peak_excess(paths, base=120.0):
    """各機巡航段最高高度超出規劃航高多少。"""
    out = {}
    for uid, bp in paths.items():
        alts = [w.alt for w in bp.waypoints
                if w.segment_type.name not in ('TAKEOFF', 'LANDING')]
        out[uid] = max(alts) - base
    return out


# ══════════════════════════════════════════════════════════════
# 對照組:先證明沒有上限時**真的**會棘輪
# ══════════════════════════════════════════════════════════════

def test_fixture_really_ratchets_without_a_cap():
    """⚠ 先斷言這個設定真的會讓高度失控 —— 否則下面的上限測試等於沒測。"""
    paths = _overlapping_pair()
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            time_step_s=1.0, cruise_speed_mps=22.0,
                            strategy='altitude', max_alt_offset_m=1e9)
    ca.run(paths)
    excess = max(_peak_excess(paths).values())
    assert excess >= 90.0, (
        f"對照組沒有棘輪(只漲了 {excess:.0f} m)⇒ 這個 fixture 測不到上限的效果")


# ══════════════════════════════════════════════════════════════
# ② 累加上限
# ══════════════════════════════════════════════════════════════

@pytest.mark.parametrize("cap", [30.0, 60.0, 100.0, 150.0])
def test_cumulative_rise_never_exceeds_cap(cap):
    """任何一架的累加抬升都不得超過上限 —— 這是使用者看得到的那條線。"""
    paths = _overlapping_pair()
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            time_step_s=1.0, cruise_speed_mps=22.0,
                            strategy='altitude', max_alt_offset_m=cap)
    ca.run(paths)
    for uid, exc in _peak_excess(paths).items():
        assert exc <= cap + 1e-6, f"UAV-{uid} 抬了 {exc:.1f} m,超過上限 {cap:.1f} m"


def test_cap_beats_the_measured_runaway():
    """實測那趟是 +240 m;預設上限 100 必須把它壓下來。"""
    paths = _overlapping_pair()
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            time_step_s=1.0, cruise_speed_mps=22.0,
                            strategy='altitude', max_alt_offset_m=100.0)
    ca.run(paths)
    assert max(_peak_excess(paths).values()) <= 100.0


def test_hitting_the_cap_is_loud(caplog):
    """撞到上限必須**大聲**:靜靜停下來和靜靜飛到 360 m 一樣糟。"""
    paths = _overlapping_pair()
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            time_step_s=1.0, cruise_speed_mps=22.0,
                            strategy='altitude', max_alt_offset_m=60.0)
    with caplog.at_level(logging.ERROR):
        ca.run(paths)
    assert any('累加上限' in r.message for r in caplog.records), \
        "撞到上限卻沒有 ERROR ⇒ 使用者不會知道衝突沒解掉"


def test_capped_uav_records_no_action(caplog):
    """撞到上限時**不可以**照抬但不記錄 —— 那會讓停滯保險絲燒不斷。"""
    paths = _overlapping_pair()
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=30.0,
                            time_step_s=1.0, cruise_speed_mps=22.0,
                            strategy='altitude', max_alt_offset_m=30.0)
    with caplog.at_level(logging.ERROR):
        _cf, actions = ca.run(paths)
    # 上限 == 單次偏移 ⇒ 只能抬一輪,第二輪起全部被夾住
    per_uav = {}
    for a in actions:
        per_uav[a.uav_id] = per_uav.get(a.uav_id, 0) + 1
    assert all(n * 30.0 <= 30.0 + 1e-6 for n in per_uav.values()), \
        f"被夾住之後還在記動作:{per_uav}"
    assert any('停滯' in r.message or '累加上限' in r.message
               for r in caplog.records)


def test_cap_does_not_touch_the_no_conflict_case():
    """沒有衝突時上限不該有任何副作用(已知合法樣本不可以響)。"""
    paths = {1: _Path([_WP(25.07 + i * 0.0005, 121.22, 120.0) for i in range(40)]),
             2: _Path([_WP(25.07 + i * 0.0005, 121.90, 120.0) for i in range(40)])}
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=50.0,
                            time_step_s=1.0, cruise_speed_mps=22.0,
                            strategy='altitude', max_alt_offset_m=100.0)
    conflicts, actions = ca.run(paths)
    assert conflicts == [] and actions == []
    assert all(w.alt == 120.0 for bp in paths.values() for w in bp.waypoints)


# ══════════════════════════════════════════════════════════════
# ① 面板 → 規劃器的接線(漏一個參數不會有症狀,所以要值層級鎖)
# ══════════════════════════════════════════════════════════════

def test_every_panel_avoidance_param_reaches_the_avoider():
    """⚠⚠⚠ 面板送出的避撞參數必須**每一個**都到得了 CollisionAvoidance。

    漏掉一個不會有任何症狀:消費端吃自己的預設,而預設常常剛好等於面板預設
    ⇒ 只有使用者**改動**那一格時才會發現「改了沒用」。掃描角就是這樣壞掉的。
    所以這裡刻意用**每個都不等於預設**的值。
    """
    from ui.main_window import build_avoidance_kwargs
    panel_params = {
        'min_separation_m': 77.0,      # 預設 100
        'alt_offset_m': 88.0,          # 預設 30
        'max_alt_offset_m': 222.0,     # 預設 100
        'avoid_strategy': 'altitude',
    }
    kw = build_avoidance_kwargs(panel_params, cruise_speed_mps=19.0)
    assert kw['min_separation_m'] == pytest.approx(77.0)
    assert kw['alt_offset_m'] == pytest.approx(88.0)
    assert kw['max_alt_offset_m'] == pytest.approx(222.0)
    assert kw['cruise_speed_mps'] == pytest.approx(19.0)

    # 而且真的建得起來、真的用了那個上限
    ca = CollisionAvoidance(**kw)
    assert ca.max_alt_offset == pytest.approx(222.0)
    assert ca.alt_offset == pytest.approx(88.0)


def test_seam_does_not_coerce_a_dangerous_combination():
    """接縫不可以偷偷把 alt_offset 夾到 min_sep —— 顯示的值必須就是用的值。"""
    from ui.main_window import build_avoidance_kwargs
    kw = build_avoidance_kwargs(
        {'min_separation_m': 80.0, 'alt_offset_m': 20.0}, cruise_speed_mps=22.0)
    assert kw['alt_offset_m'] == pytest.approx(20.0)


def test_no_call_site_bypasses_the_seam():
    """⚠ 結構性守衛:`CollisionAvoidance(...)` 只能經由接縫建構。

    為什麼這條**必須**是 AST 而不是值層級:值層級鎖得住「接縫翻對了嗎」,
    但鎖不住「呼叫點有沒有用接縫」—— 繞過去自己拼 kwargs,接縫的測試照樣全綠
    (實測突變臂 M14 就是這樣活下來的)。

    ⚠ 這是**結構事實**,不是行為,所以用 AST 解析而不是 `SRC.count(...)`
    字串比對 —— 後者對換行/改名/加註解全都會誤判,見 [[verifying-the-verifier]]。
    """
    import ast
    import pathlib

    src = pathlib.Path('ui/main_window.py').read_text(encoding='utf-8')
    tree = ast.parse(src)

    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name)
             and n.func.id == 'CollisionAvoidance']
    assert calls, "找不到任何 CollisionAvoidance(...) ⇒ 這條守衛失去意義(可能被改名了)"

    for c in calls:
        starred = [k for k in c.keywords if k.arg is None]      # **kwargs
        ok = any(isinstance(k.value, ast.Call)
                 and isinstance(k.value.func, ast.Name)
                 and k.value.func.id == 'build_avoidance_kwargs'
                 for k in starred)
        assert ok, (
            f"main_window.py:{c.lineno} 的 CollisionAvoidance(...) 沒有走 "
            f"build_avoidance_kwargs ⇒ 面板參數可能被漏掉而完全沒有症狀")


def test_vtol_tab_label_has_no_emoji():
    """分頁標籤不得含 emoji。

    ⚠ 這條放不進面板測試:標籤是 `addTab()` 的參數,存在 main_window 裡,
    面板自己的 widget 樹**看不到它**(突變臂 M15 就是這樣活下來的)。
    標籤是字面值 ⇒ 結構事實 ⇒ 用 AST 讀那個字面值。
    """
    import ast
    import pathlib

    tree = ast.parse(pathlib.Path('ui/main_window.py').read_text(encoding='utf-8'))
    labels = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'addTab'):
            for arg in node.args:
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                    labels.append(arg.value)
    vtol = [t for t in labels if 'VTOL' in t]
    assert vtol, "找不到 VTOL 分頁標籤(可能被改名了 ⇒ 這條守衛失效)"
    for t in vtol:
        bad = [ch for ch in t
               if 0x1F000 <= ord(ch) <= 0x1FAFF
               or 0x2600 <= ord(ch) <= 0x27BF
               or 0x2460 <= ord(ch) <= 0x24FF]
        assert not bad, f"分頁標籤 {t!r} 仍含裝飾符號 {bad}"


# ══════════════════════════════════════════════════════════════
# 自動上限:不可以擋住**解得開**的情境
# ══════════════════════════════════════════════════════════════

@pytest.mark.parametrize("n_uav", [2, 3, 4])
def test_auto_cap_never_blocks_a_solvable_case(n_uav):
    """⚠⚠⚠ 上限的頭號風險不是「太鬆」而是「太緊」。

    太鬆只是高度難看;**太緊會把本來解得開的情境變成解不開**,而且錯誤訊息
    長得像「這組路徑本來就無解」。我第一版寫死 100 m 就是這樣,一次弄紅了
    tests/test_collision_avoidance_rounds.py 三條(n=3 合法需要 240 m)。

    實測:合法收斂需要 2 x (n-1) x alt_offset —— 直覺的 (n-1) 是**不夠**的,
    因為每輪抬的是「配對中 ID 較大的那架」,中間那幾架會跟著一起爬。
    """
    import sys
    sys.path.insert(0, 'tests')
    from test_collision_avoidance_rounds import (_stacked_paths, MIN_SEP,
                                                 SPEED, ALT_OFF)
    paths = _stacked_paths(n_uav=n_uav)
    ca = CollisionAvoidance(min_separation_m=MIN_SEP, alt_offset_m=ALT_OFF,
                            cruise_speed_mps=SPEED)          # 自動上限
    ca.run(paths)
    assert not ca.detect(paths), (
        f"n={n_uav} 本來解得開,卻被自動上限 {ca._effective_cap(paths):.0f} m 擋住")


def test_auto_cap_matches_the_measured_requirement():
    """自動上限的公式必須就是量出來的 2(n-1)xoffset,不是隨手一個數字。"""
    paths = {i: _Path([_WP(25.0, 121.0, 120.0)]) for i in range(1, 5)}
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=60.0,
                            cruise_speed_mps=22.0)
    assert ca._effective_cap(paths) == pytest.approx(2 * (4 - 1) * 60.0)


def test_explicit_cap_overrides_auto():
    """使用者填了數字就以他的為準(絕對天花板),自動值不得蓋過去。"""
    paths = {i: _Path([_WP(25.0, 121.0, 120.0)]) for i in range(1, 5)}
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=60.0,
                            cruise_speed_mps=22.0, max_alt_offset_m=90.0)
    assert ca._effective_cap(paths) == pytest.approx(90.0)


def test_panel_zero_means_auto():
    """面板填 0 ⇒ 送 None ⇒ 自動;不可以送 0 把上限變成「完全不准抬」。"""
    from ui.main_window import build_avoidance_kwargs
    kw = build_avoidance_kwargs({'max_alt_offset_m': None}, cruise_speed_mps=22.0)
    assert kw['max_alt_offset_m'] is None
    ca = CollisionAvoidance(**kw)
    paths = {i: _Path([_WP(25.0, 121.0, 120.0)]) for i in range(1, 4)}
    assert ca._effective_cap(paths) > 0
