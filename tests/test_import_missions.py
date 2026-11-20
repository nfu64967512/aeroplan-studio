# -*- coding: utf-8 -*-
"""mission/import_missions.py —— 多檔匯入、每檔一架的純邏輯鎖（不需要 Qt）。

防的是：
① 載入就等於指派（`load_many` 回來的 sysid 必須是 None）。
② 檔名猜 sysid 猜過頭（沒有分隔符、超出 1–255、兩個互相矛盾的數字都要回 None）。
③ 壞檔被靜靜丟掉（讀不進來的檔要**留在清單裡**，而且是擋住「確定」的理由）。
④ 重複 sysid 沒被抓到（這個專案發生過「兩架拿到同一份任務」）。
"""
from __future__ import annotations

import pathlib

import pytest

from mission import import_missions as im
from mission.waypoint_file import LoadedMission, MissionIssue

SAMPLE_WPL = (
    "QGC WPL 110\n"
    "0\t1\t0\t16\t0\t0\t0\t0\t25.0000000\t121.0000000\t0.0\t1\n"
    "1\t0\t3\t22\t0\t0\t0\t0\t25.0010000\t121.0010000\t50.0\t1\n"
    "2\t0\t3\t16\t0\t0\t0\t0\t25.0020000\t121.0020000\t50.0\t1\n"
    "3\t0\t3\t17\t0\t0\t80\t0\t25.0025000\t121.0025000\t60.0\t1\n"
    "4\t0\t3\t21\t0\t0\t0\t0\t25.0030000\t121.0030000\t0.0\t1\n"
)
# 沒有起飛、沒有降落、最後一項不是終點 ⇒ 只有警告、沒有錯誤
NO_TOL_WPL = (
    "QGC WPL 110\n"
    "0\t1\t0\t16\t0\t0\t0\t0\t25.0000000\t121.0000000\t0.0\t1\n"
    "1\t0\t3\t16\t0\t0\t0\t0\t25.0010000\t121.0010000\t50.0\t1\n"
    "2\t0\t3\t16\t0\t0\t0\t0\t25.0020000\t121.0020000\t50.0\t1\n"
)
# 只有 Home 槽 ⇒ `empty` 錯誤
EMPTY_WPL = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t25.0\t121.0\t0.0\t1\n"


def _write(tmp_path: pathlib.Path, name: str, text: str) -> str:
    p = tmp_path / name
    p.write_text(text, encoding='utf-8')
    return str(p)


# ═══════════════════════════ suggest_sysid ═══════════════════════════════
@pytest.mark.parametrize('name, want', [
    ('UAV-3.waypoints', 3),
    ('uav3.waypoints', 3),
    ('UAV_7.plan', 7),
    ('uav 12.waypoints', 12),
    ('area_B_uav2.waypoints', 2),
    ('mission_sys3.waypoints', 3),
    ('sysid3.plan', 3),
    ('sysid-4.waypoints', 4),
    ('mission_3.waypoints', 3),
    ('area-B-12.plan', 12),
    ('survey 5.txt', 5),
    ('uav3_area_7.waypoints', 3),      # 明確模式優先於結尾數字
    ('mission_uav3_3.waypoints', 3),   # 兩者一致
    ('UAV-255.waypoints', 255),
    ('UAV-1.waypoints', 1),
])
def test_suggest_sysid_recognises_explicit_patterns(name, want):
    assert im.suggest_sysid(name) == want
    assert im.suggest_sysid('C:/some/dir/' + name) == want, '只看檔名，不看目錄'


@pytest.mark.parametrize('name', [
    'mission.waypoints',               # 什麼都沒有
    'flight01.waypoints',              # 沒有分隔符的結尾數字不算
    'mission_v2.waypoints',            # 分隔符後不是純數字
    'mission_2024.waypoints',          # 超出 1–255
    'UAV-0.waypoints',                 # 0 是廣播，不是飛機
    'UAV-256.waypoints',               # 超出 uint8
    'uav1_uav2.waypoints',             # 兩個明確模式互相矛盾 ⇒ 不猜
    'sys3_uav4.waypoints',             # 同上（跨兩種模式）
    'uav3x.waypoints',                 # 數字後面黏著字母
    'nouav3.waypoints',                # uav 前面黏著字母
    'route_12_final.waypoints',        # 數字不在結尾、也沒有明確模式
])
def test_suggest_sysid_refuses_to_guess(name):
    assert im.suggest_sysid(name) is None


# ═══════════════════════════ load_many ═══════════════════════════════════
def test_load_many_loads_each_file_and_never_assigns(tmp_path):
    a = _write(tmp_path, 'UAV-1.waypoints', SAMPLE_WPL)
    b = _write(tmp_path, 'mission.waypoints', NO_TOL_WPL)
    ms = im.load_many([a, b])
    assert [m.path for m in ms] == [a, b]
    assert [m.label for m in ms] == ['UAV-1.waypoints', 'mission.waypoints']
    assert [m.sysid for m in ms] == [None, None], '載入 ≠ 指派：sysid 一律 None'
    assert [m.suggested_sysid for m in ms] == [1, None]
    assert len(ms[0].loaded.items) == 5 and not ms[0].has_errors
    assert len(ms[1].loaded.items) == 3 and not ms[1].has_errors
    assert {i.code for i in ms[1].loaded.warnings} >= {'no_takeoff', 'last_not_terminal'}
    assert ms[0].items is ms[0].loaded.items


def test_load_many_keeps_unreadable_files_in_the_list_with_an_error(tmp_path):
    good = _write(tmp_path, 'UAV-1.waypoints', SAMPLE_WPL)
    garbage = _write(tmp_path, 'UAV-2.waypoints', 'this is not a waypoint file\n')
    missing = str(tmp_path / 'UAV-3.waypoints')
    bad_json = _write(tmp_path, 'UAV-4.plan', '{"mission": [1, 2')
    ms = im.load_many([good, garbage, missing, bad_json])
    assert len(ms) == 4, '一個檔壞掉不可以讓整批消失'
    for m in ms[1:]:
        assert m.loaded.fmt == 'unreadable'
        assert m.loaded.items == []
        assert m.has_errors
        assert [i.code for i in m.loaded.errors] == ['load_failed']
    assert '認不出格式' in ms[1].loaded.errors[0].message
    assert '讀不到檔案' in ms[2].loaded.errors[0].message
    assert 'JSON' in ms[3].loaded.errors[0].message
    assert not ms[0].has_errors


def test_load_many_keeps_semantic_errors_from_the_validator(tmp_path):
    p = _write(tmp_path, 'UAV-1.waypoints', EMPTY_WPL)
    (m,) = im.load_many([p])
    assert m.loaded.fmt == 'wpl' and m.has_errors
    assert 'empty' in {i.code for i in m.loaded.errors}


def test_load_many_accepts_pathlike_and_empty(tmp_path):
    p = tmp_path / 'UAV-1.waypoints'
    p.write_text(SAMPLE_WPL, encoding='utf-8')
    (m,) = im.load_many([p])
    assert m.path == str(p) and m.suggested_sysid == 1
    assert im.load_many([]) == []


def test_load_many_disambiguates_same_basename_from_different_folders(tmp_path):
    """⚠️ 兩個資料夾各一個 UAV-1.waypoints ⇒ 表格／狀態列／確認框只看 label 會以為是同一個檔。
    撞名的 label 變 `<上層資料夾>/<檔名>`；沒撞的維持純檔名；上層也同名就再往上一層。"""
    for d in ('siteA', 'siteB', 'siteA/x', 'siteB/x'):
        (tmp_path / d).mkdir(parents=True, exist_ok=True)
    a = _write(tmp_path / 'siteA', 'UAV-1.waypoints', SAMPLE_WPL)
    b = _write(tmp_path / 'siteB', 'UAV-1.waypoints', SAMPLE_WPL)
    c = _write(tmp_path / 'siteB', 'mission.waypoints', NO_TOL_WPL)
    ms = im.load_many([a, b, c])
    assert [m.label for m in ms] == ['siteA/UAV-1.waypoints', 'siteB/UAV-1.waypoints', 'mission.waypoints']
    assert [m.path for m in ms] == [a, b, c], 'path 不動，只改顯示用的 label'
    assert [m.suggested_sysid for m in ms] == [1, 1, None]
    probs = im.assignment_problems([im.ImportedMission(m.path, 1, m.loaded, m.label, m.suggested_sysid) for m in ms[:2]])
    assert probs == ['sysid 1 被 2 個檔同時指派：siteA/UAV-1.waypoints、siteB/UAV-1.waypoints']
    # 上層資料夾也同名（siteA/x、siteB/x）⇒ 再往上一層
    ax = _write(tmp_path / 'siteA' / 'x', 'm.waypoints', SAMPLE_WPL)
    bx = _write(tmp_path / 'siteB' / 'x', 'm.waypoints', SAMPLE_WPL)
    assert [m.label for m in im.load_many([ax, bx])] == ['siteA/x/m.waypoints', 'siteB/x/m.waypoints']
    # 讀不進來的檔一樣參與（它也會出現在表格裡）
    missing_a = str(tmp_path / 'siteA' / 'nope.waypoints')
    missing_b = str(tmp_path / 'siteB' / 'nope.waypoints')
    assert [m.label for m in im.load_many([missing_a, missing_b])] == ['siteA/nope.waypoints', 'siteB/nope.waypoints']


def test_disambiguate_labels_works_incrementally_and_reverts(tmp_path):
    """對話框是一批一批加的：撞名的對象可能早就在清單裡 ⇒ 要能對整份清單重算；移除後要回到純檔名。"""
    (tmp_path / 'siteA').mkdir()
    (tmp_path / 'siteB').mkdir()
    a = _write(tmp_path / 'siteA', 'UAV-1.waypoints', SAMPLE_WPL)
    b = _write(tmp_path / 'siteB', 'UAV-1.waypoints', SAMPLE_WPL)
    ms = im.load_many([a])
    assert ms[0].label == 'UAV-1.waypoints'
    ms += im.load_many([b])
    assert [m.label for m in ms] == ['UAV-1.waypoints', 'UAV-1.waypoints'], '單看新的一批看不出撞名'
    assert im.disambiguate_labels(ms) is ms
    assert [m.label for m in ms] == ['siteA/UAV-1.waypoints', 'siteB/UAV-1.waypoints']
    ms.pop(1)
    im.disambiguate_labels(ms)
    assert ms[0].label == 'UAV-1.waypoints'
    # 同一個路徑出現兩次本來就分不開：不會無窮迴圈，label 退到完整路徑的各段
    dup = im.load_many([a, a])
    assert dup[0].label == dup[1].label and dup[0].label.endswith('siteA/UAV-1.waypoints')
    assert im.disambiguate_labels([]) == []


# ═══════════════════════════ assignment_problems ═════════════════════════
def _mk(label, sysid=None, errors=0, items=5):
    issues = [MissionIssue('error', None, 'x', f'錯誤 {k}') for k in range(errors)]
    lm = LoadedMission(f'C:/x/{label}', 'wpl', [{'seq': i} for i in range(items)], issues)
    return im.ImportedMission(path=lm.path, sysid=sysid, loaded=lm, label=label)


def test_no_files_is_a_problem():
    assert im.assignment_problems([]) == ['沒有任何檔案']


def test_fully_assigned_distinct_and_clean_has_no_problems():
    assert im.assignment_problems([_mk('a', 1), _mk('b', 2), _mk('c', 255)]) == []


def test_unassigned_is_reported_per_file():
    probs = im.assignment_problems([_mk('a', 1), _mk('b', None), _mk('c', None)])
    assert probs == ['b：尚未指派 sysid', 'c：尚未指派 sysid']


def test_duplicate_sysid_is_reported_with_both_names():
    """⚠️⚠️ 「兩架拿到同一份任務」的反面：同一份給兩架也一樣要擋。"""
    probs = im.assignment_problems([_mk('a', 3), _mk('b', 3), _mk('c', 4)])
    assert len(probs) == 1
    assert 'sysid 3' in probs[0] and 'a' in probs[0] and 'b' in probs[0] and '2 個檔' in probs[0]


@pytest.mark.parametrize('bad', [0, 256, -1, 3.5, 'x'])
def test_out_of_range_or_non_integer_sysid_is_reported(bad):
    probs = im.assignment_problems([_mk('a', bad)])
    assert len(probs) == 1 and '1–255' in probs[0]


def test_float_that_is_integral_is_accepted():
    assert im.assignment_problems([_mk('a', 3.0)]) == []


def test_files_with_errors_block_even_when_assigned():
    probs = im.assignment_problems([_mk('a', 1, errors=2), _mk('b', 2)])
    assert len(probs) == 1
    assert probs[0].startswith('a：有 2 個錯誤（錯誤 0…另 1 個）')


def test_every_reason_is_listed_not_just_the_first():
    probs = im.assignment_problems([_mk('a', None, errors=1), _mk('b', 2), _mk('c', 2)])
    assert any('尚未指派' in p for p in probs)
    assert any('1 個錯誤' in p for p in probs)
    assert any('sysid 2 被 2 個檔' in p for p in probs)
    assert len(probs) == 3


# ═══════════════════════════ summaries ═══════════════════════════════════
def test_summaries_have_the_documented_shape(tmp_path):
    a = _write(tmp_path, 'UAV-1.waypoints', SAMPLE_WPL)
    b = _write(tmp_path, 'mission.waypoints', NO_TOL_WPL)
    ms = im.load_many([a, b])
    ms[0].sysid = 1
    s0, s1 = im.summaries(ms)
    keys = {'path', 'sysid', 'n_items', 'home', 'n_nav', 'has_takeoff', 'has_land',
            'issues_error', 'issues_warning', 'summary'}
    assert set(s0) == keys and set(s1) == keys
    assert s0['path'] == a and s0['sysid'] == 1 and s0['n_items'] == 5
    assert s0['home'] == {'lat': 25.0, 'lon': 121.0, 'alt': 0.0}
    assert s0['n_nav'] == 4, 'seq 0 的 Home 不算 NAV'
    assert s0['has_takeoff'] and s0['has_land']
    assert s0['issues_error'] == 0 and s0['issues_warning'] == 0
    assert '5 項' in s0['summary'] and '可繪製 4 點' in s0['summary'] and '盤旋圈 1' in s0['summary']
    assert '起飛有' in s0['summary'] and '降落有' in s0['summary']
    assert s1['sysid'] is None and s1['n_nav'] == 2
    assert not s1['has_takeoff'] and not s1['has_land']
    assert s1['issues_warning'] >= 2 and s1['issues_error'] == 0
    assert '起飛無' in s1['summary'] and '降落無' in s1['summary']
    assert '盤旋圈' not in s1['summary']


def test_summary_of_unreadable_file_says_so(tmp_path):
    p = _write(tmp_path, 'UAV-2.waypoints', 'garbage\n')
    (s,) = im.summaries(im.load_many([p]))
    assert s['n_items'] == 0 and s['home'] is None and s['n_nav'] == 0
    assert s['issues_error'] == 1
    assert s['summary'].startswith('讀不進來：')


def test_summary_counts_errors_and_warnings_separately(tmp_path):
    p = _write(tmp_path, 'UAV-1.waypoints', EMPTY_WPL)
    (s,) = im.summaries(im.load_many([p]))
    assert s['issues_error'] == 1 and s['n_items'] == 1 and s['n_nav'] == 0
    assert '錯誤 1' in s['summary']


# ═══════════════════════════ 配色（2D/3D/對話框共用）══════════════════════
def test_sysid_color_is_stable_and_distinct_for_small_fleets():
    from mission import mission_view as mv
    assert mv.sysid_color(None) == mv.FC_MISSION_DEFAULT_COLOR
    cols = [mv.sysid_color(s) for s in range(1, len(mv.SYSID_PALETTE) + 1)]
    assert len(set(cols)) == len(cols), '小機隊裡每架都要不同色'
    assert mv.sysid_color(1) == mv.sysid_color(1 + len(mv.SYSID_PALETTE)), '循環取色'
    assert mv.fc_mission_label(3) == 'UAV-3 任務'
    assert mv.fc_mission_label(None) == 'FC Mission'
    assert mv.fc_mission_label(3, '自訂') == '自訂'
