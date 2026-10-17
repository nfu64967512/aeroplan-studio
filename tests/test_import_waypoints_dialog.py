# -*- coding: utf-8 -*-
"""ui/dialogs/import_waypoints_dialog.py —— offscreen 建對話框，鎖「顯示的 sysid 就是送出的 sysid」。

不開 SITL／COM／MainWindow；QFileDialog 那條不測（用 add_paths 直接餵路徑）。
"""
from __future__ import annotations

import os
import pathlib

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

try:
    from PyQt6.QtWidgets import QApplication, QComboBox
    from ui.dialogs.import_waypoints_dialog import (COL_ISSUES, COL_ITEMS, COL_NAME, COL_SYSID, COL_TOL,
                                                    PLACEHOLDER, ImportWaypointsDialog)
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f'Qt 匯入失敗：{globals().get("_QT_ERR", "")}')

from mission import import_missions as im  # noqa: E402

SAMPLE_WPL = (
    "QGC WPL 110\n"
    "0\t1\t0\t16\t0\t0\t0\t0\t25.0000000\t121.0000000\t0.0\t1\n"
    "1\t0\t3\t22\t0\t0\t0\t0\t25.0010000\t121.0010000\t50.0\t1\n"
    "2\t0\t3\t16\t0\t0\t0\t0\t25.0020000\t121.0020000\t50.0\t1\n"
    "3\t0\t3\t21\t0\t0\t0\t0\t25.0030000\t121.0030000\t0.0\t1\n"
)
EMPTY_WPL = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t25.0\t121.0\t0.0\t1\n"


@pytest.fixture(scope='module')
def qt_app():
    return QApplication.instance() or QApplication([])


def _write(tmp_path: pathlib.Path, name: str, text: str = SAMPLE_WPL) -> str:
    p = tmp_path / name
    p.write_text(text, encoding='utf-8')
    return str(p)


def _combo(dlg, row) -> QComboBox:
    w = dlg.table.cellWidget(row, COL_SYSID)
    assert isinstance(w, QComboBox)
    return w


def _combo_texts(cmb) -> list:
    return [cmb.itemText(i) for i in range(cmb.count())]


# ═══════════════════════════ 表格與預設值 ════════════════════════════════
def test_rows_match_files_and_filename_guess_is_only_the_default(qt_app, tmp_path):
    a = _write(tmp_path, 'UAV-1.waypoints')
    b = _write(tmp_path, 'UAV-2.waypoints')
    c = _write(tmp_path, 'mission.waypoints')
    dlg = ImportWaypointsDialog(paths=[a, b, c], links_sysids=[1, 2, 3], last_dir=str(tmp_path))
    assert dlg.table.rowCount() == 3
    assert [dlg.table.item(r, COL_NAME).text() for r in range(3)] == ['UAV-1.waypoints', 'UAV-2.waypoints',
                                                                       'mission.waypoints']
    assert [dlg.table.item(r, COL_ITEMS).text() for r in range(3)] == ['4', '4', '4']
    assert dlg.table.item(0, COL_TOL).text() == '起飛✓／降落✓'
    assert dlg.table.item(0, COL_ISSUES).text() == '0 錯誤／0 警告'
    # 檔名猜到的進了下拉的預設；猜不到的停在「請選擇」
    assert _combo(dlg, 0).currentData() == 1 and _combo(dlg, 1).currentData() == 2
    assert _combo(dlg, 2).currentData() is None and _combo(dlg, 2).currentText() == PLACEHOLDER
    assert [m.sysid for m in dlg.missions()] == [1, 2, None]
    # 連線中的 sysid 才在下拉裡，而且沒有「未連線」字樣
    assert _combo_texts(_combo(dlg, 2)) == [PLACEHOLDER, 'UAV-1', 'UAV-2', 'UAV-3']
    assert dlg.lbl_links.text().startswith('連線中的 UAV：UAV-1、UAV-2、UAV-3')
    # 一個沒指派 ⇒ 確定不能按，理由列出
    assert not dlg.btn_ok.isEnabled()
    assert 'mission.waypoints：尚未指派 sysid' in dlg.lbl_problems.text()
    assert dlg.result() == [], '還沒按確定就不該有結果'
    assert '檔名猜到 sysid 1' in dlg.issues_view.toPlainText()


def test_suggested_but_unconnected_sysid_is_offered_and_marked(qt_app, tmp_path):
    a = _write(tmp_path, 'UAV-7.waypoints')
    dlg = ImportWaypointsDialog(paths=[a], links_sysids=[1, 2])
    cmb = _combo(dlg, 0)
    assert _combo_texts(cmb) == [PLACEHOLDER, 'UAV-1', 'UAV-2', 'UAV-7（未連線）']
    assert cmb.currentData() == 7 and '未連線' in cmb.currentText()
    assert dlg.btn_ok.isEnabled(), '未連線只是標示，不是擋住確定的理由（寫入時另有連線檢查）'


def test_without_any_link_every_sysid_is_selectable_but_marked_offline(qt_app, tmp_path):
    a = _write(tmp_path, 'mission_3.waypoints')
    dlg = ImportWaypointsDialog(paths=[a], links_sysids=[])
    cmb = _combo(dlg, 0)
    texts = _combo_texts(cmb)
    assert texts[0] == PLACEHOLDER and len(texts) == 1 + 255
    assert texts[1] == 'UAV-1（未連線）' and texts[-1] == 'UAV-255（未連線）'
    assert all('未連線' in t for t in texts[1:])
    assert cmb.currentData() == 3
    assert '沒有任何連線' in dlg.lbl_links.text()
    assert dlg.btn_ok.isEnabled()


# ═══════════════════════════ 擋住確定 ════════════════════════════════════
def test_duplicate_sysid_blocks_ok_until_the_user_fixes_it(qt_app, tmp_path):
    """⚠️⚠️ 兩個檔指到同一架 ⇒ 不能按確定；改掉其中一個 ⇒ 立刻可以。"""
    a = _write(tmp_path, 'UAV-1.waypoints')
    b = _write(tmp_path, 'uav1_copy.waypoints')
    dlg = ImportWaypointsDialog(paths=[a, b], links_sysids=[1, 2])
    assert [m.sysid for m in dlg.missions()] == [1, 1]
    assert not dlg.btn_ok.isEnabled()
    assert 'sysid 1 被 2 個檔同時指派' in dlg.lbl_problems.text()
    assert dlg.set_sysid(1, 2) is True
    assert [m.sysid for m in dlg.missions()] == [1, 2]
    assert dlg.btn_ok.isEnabled() and dlg.lbl_problems.text() == ''
    # 使用者再改回去 ⇒ 又被擋
    _combo(dlg, 1).setCurrentIndex(_combo(dlg, 1).findData(1))
    assert not dlg.btn_ok.isEnabled()


def test_set_sysid_refuses_values_not_in_the_dropdown(qt_app, tmp_path):
    a = _write(tmp_path, 'UAV-1.waypoints')
    dlg = ImportWaypointsDialog(paths=[a], links_sysids=[1, 2])
    assert dlg.set_sysid(0, 9) is False and dlg.missions()[0].sysid == 1
    assert dlg.set_sysid(5, 1) is False


def test_file_with_errors_blocks_ok_and_is_shown_not_dropped(qt_app, tmp_path):
    good = _write(tmp_path, 'UAV-1.waypoints')
    empty = _write(tmp_path, 'UAV-2.waypoints', EMPTY_WPL)
    garbage = _write(tmp_path, 'UAV-3.waypoints', 'nonsense\n')
    dlg = ImportWaypointsDialog(paths=[good, empty, garbage], links_sysids=[1, 2, 3])
    assert dlg.table.rowCount() == 3, '壞檔要留在表格裡讓人看到'
    assert dlg.table.item(1, COL_ISSUES).text().startswith('1 錯誤')
    assert dlg.table.item(2, COL_ITEMS).text() == '0'
    assert not dlg.btn_ok.isEnabled()
    txt = dlg.lbl_problems.text()
    assert 'UAV-2.waypoints：有 1 個錯誤' in txt and 'UAV-3.waypoints：有 1 個錯誤' in txt
    assert '任務是空的' in dlg.issues_view.toPlainText()
    assert '認不出格式' in dlg.issues_view.toPlainText()
    # 移除兩個壞檔 ⇒ 可以按
    dlg.table.setCurrentCell(2, COL_NAME)
    assert dlg.remove_selected() is True
    dlg.table.setCurrentCell(1, COL_NAME)
    assert dlg.remove_selected() is True
    assert dlg.table.rowCount() == 1 and dlg.btn_ok.isEnabled()


def test_ok_is_refused_with_no_files(qt_app):
    dlg = ImportWaypointsDialog(paths=[], links_sysids=[1])
    assert dlg.table.rowCount() == 0 and not dlg.btn_ok.isEnabled()
    assert dlg.lbl_problems.text() == '✘ 沒有任何檔案'
    assert not dlg.btn_remove.isEnabled()


def test_on_ok_recomputes_the_same_rule_and_refuses(qt_app, tmp_path):
    """按鈕灰掉只是 UI；_on_ok 自己也要再算一次同一份規則（兩處用同一個函式）。"""
    a = _write(tmp_path, 'mission.waypoints')
    dlg = ImportWaypointsDialog(paths=[a], links_sysids=[1])
    dlg.btn_ok.setEnabled(True)                      # 模擬按鈕狀態跟真實狀態脫節
    dlg._on_ok()
    assert dlg.result() == [] and not dlg.btn_ok.isEnabled()
    assert '尚未指派' in dlg.lbl_problems.text()


# ═══════════════════════════ result() 形狀 ═══════════════════════════════
def test_result_returns_assigned_imported_missions_after_ok(qt_app, tmp_path):
    a = _write(tmp_path, 'UAV-1.waypoints')
    b = _write(tmp_path, 'mission.waypoints')
    dlg = ImportWaypointsDialog(paths=[a, b], links_sysids=[1, 2])
    assert dlg.set_sysid(1, 2)
    accepted = []
    dlg.accepted.connect(lambda: accepted.append(True))
    dlg._on_ok()
    assert accepted == [True]
    res = dlg.result()
    assert isinstance(res, list) and len(res) == 2
    for m in res:
        assert isinstance(m, im.ImportedMission)
        assert isinstance(m.sysid, int)
        assert m.loaded.items and not m.has_errors
    assert [(m.label, m.sysid) for m in res] == [('UAV-1.waypoints', 1), ('mission.waypoints', 2)]
    assert [m.path for m in res] == [a, b]
    assert res[0].suggested_sysid == 1 and res[1].suggested_sysid is None
    assert im.assignment_problems(res) == [], '送出去的清單本身就要過同一道檢查'
    # 顯示的就是送出的：表格下拉與 result 逐列一致
    assert [_combo(dlg, r).currentData() for r in range(2)] == [m.sysid for m in res]


def test_add_paths_skips_duplicates_and_keeps_user_choices(qt_app, tmp_path):
    a = _write(tmp_path, 'UAV-1.waypoints')
    b = _write(tmp_path, 'mission.waypoints')
    dlg = ImportWaypointsDialog(paths=[a], links_sysids=[1, 2, 3])
    assert dlg.set_sysid(0, 3)                       # 使用者改掉檔名猜的 1
    assert dlg.add_paths([a, b]) == 1, '同一個路徑不重複加入'
    assert dlg.table.rowCount() == 2
    assert dlg.missions()[0].sysid == 3, '重建表格不可以把使用者選過的洗掉'
    assert _combo(dlg, 0).currentData() == 3
    assert dlg.missions()[1].sysid is None


def test_user_clearing_a_row_to_placeholder_survives_add_and_remove(qt_app, tmp_path):
    """⚠️ 使用者明確把一列選回「請選擇」(None) ⇒ 之後加入／移除檔案重建表格時**不可以**再猜回檔名的值。
    以前 _refresh 只看 `m.sysid is not None`，清掉的列就被靜靜猜回 1。"""
    a = _write(tmp_path, 'UAV-1.waypoints')
    b = _write(tmp_path, 'UAV-2.waypoints')
    c = _write(tmp_path, 'mission.waypoints')
    dlg = ImportWaypointsDialog(paths=[a], links_sysids=[1, 2, 3])
    assert dlg.missions()[0].sysid == 1 and dlg.btn_ok.isEnabled()
    assert dlg.set_sysid(0, None) is True
    assert dlg.missions()[0].sysid is None and _combo(dlg, 0).currentText() == PLACEHOLDER
    assert not dlg.btn_ok.isEnabled()
    # 加入檔案 ⇒ 重建表格：第 0 列仍是 None；沒碰過的新列照常拿檔名猜的預設
    assert dlg.add_paths([b]) == 1
    assert dlg.missions()[0].sysid is None and _combo(dlg, 0).currentData() is None
    assert dlg.missions()[1].sysid == 2, '沒碰過的列仍拿檔名猜的當預設'
    assert not dlg.btn_ok.isEnabled() and 'UAV-1.waypoints：尚未指派 sysid' in dlg.lbl_problems.text()
    # 移除別列 ⇒ 再重建：第 0 列還是 None
    dlg.add_paths([c])
    dlg.table.setCurrentCell(2, COL_NAME)
    assert dlg.remove_selected() is True
    assert dlg.table.rowCount() == 2
    assert dlg.missions()[0].sysid is None and _combo(dlg, 0).currentData() is None
    assert not dlg.btn_ok.isEnabled()
    # 真的用下拉（不是 set_sysid）選回「請選擇」也一樣算使用者碰過
    _combo(dlg, 1).setCurrentIndex(0)
    assert dlg.missions()[1].sysid is None
    dlg.add_paths([c])
    assert [m.sysid for m in dlg.missions()] == [None, None, None]
    # 移除掉再加回來 ⇒ 是一個新列，重新拿檔名猜的預設
    dlg.table.setCurrentCell(0, COL_NAME)
    assert dlg.remove_selected() is True
    dlg.add_paths([a])
    assert dlg.missions()[-1].path == a and dlg.missions()[-1].sysid == 1


def test_same_basename_in_different_folders_is_distinguishable_everywhere(qt_app, tmp_path):
    """⚠️ 兩個資料夾各一個 UAV-1.waypoints：表格、紅字理由（＝確認框用的那份）都要分得出來；
    撞名的對象可能早就在清單裡，所以加入時要對整份清單重算；移除後恢復純檔名。"""
    (tmp_path / 'siteA').mkdir()
    (tmp_path / 'siteB').mkdir()
    a = _write(tmp_path / 'siteA', 'UAV-1.waypoints')
    b = _write(tmp_path / 'siteB', 'UAV-1.waypoints')
    c = _write(tmp_path / 'siteB', 'UAV-2.waypoints')
    dlg = ImportWaypointsDialog(paths=[a], links_sysids=[1, 2])
    assert dlg.table.item(0, COL_NAME).text() == 'UAV-1.waypoints', '沒撞名就是純檔名'
    assert dlg.add_paths([b, c]) == 2
    names = [dlg.table.item(r, COL_NAME).text() for r in range(3)]
    assert names == ['siteA/UAV-1.waypoints', 'siteB/UAV-1.waypoints', 'UAV-2.waypoints']
    assert [m.label for m in dlg.missions()] == names
    txt = dlg.lbl_problems.text()
    assert 'sysid 1 被 2 個檔同時指派：siteA/UAV-1.waypoints、siteB/UAV-1.waypoints' in txt
    assert dlg.table.item(0, COL_NAME).toolTip().startswith(a)
    # 移除 siteB 那份 ⇒ 不再撞名，回到純檔名
    dlg.table.setCurrentCell(1, COL_NAME)
    assert dlg.remove_selected() is True
    assert [dlg.table.item(r, COL_NAME).text() for r in range(2)] == ['UAV-1.waypoints', 'UAV-2.waypoints']
    # 送出的 label 也是分得開的那個：拿掉 UAV-2.waypoints、把 siteB 那份加回來並指到 2
    dlg.table.setCurrentCell(1, COL_NAME)
    assert dlg.remove_selected() is True
    assert dlg.add_paths([b]) == 1
    assert [m.label for m in dlg.missions()] == ['siteA/UAV-1.waypoints', 'siteB/UAV-1.waypoints']
    assert dlg.set_sysid(1, 2) is True
    dlg._on_ok()
    assert [(m.label, m.sysid) for m in dlg.result()] == [('siteA/UAV-1.waypoints', 1), ('siteB/UAV-1.waypoints', 2)]
