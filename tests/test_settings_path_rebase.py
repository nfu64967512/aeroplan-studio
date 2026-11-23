"""專案搬家之後，存檔的絕對路徑要被校正（`config.settings._rebase_saved_paths`）。

⚠⚠⚠ 2026-09-09 實際發生：專案從 ``Documents/aeroplan-studio`` 搬到
``Documents/Projects/aeroplan-studio``，而 ``config/settings.json`` 裡是**搬家前**的
八條絕對路徑。`GlobalSettings.load` 是 ``PathSettings(**config_data['paths'])``
—— **存檔會蓋掉計算值** ⇒ 八條全部指向舊位置。

而症狀只有一則「飛行器配置文件不存在」的警告，其餘七條**完全靜默**：

* `vehicle_profiles` 永遠是 None（載不到任何機型設定）
* `icons_dir` / `styles_dir` 指向不存在的目錄
* `PathSettings._ensure_dirs` 還會在**舊位置**建出一整組空目錄（實測確實建了）

⚠ 所以正確做法**不是**把那則警告消掉 —— 它是唯一露出水面的那一角。

本檔直接測純函式 `_rebase_saved_paths`，不碰 `PathSettings`（它的
`__post_init__` 會真的去建目錄，測試不該產生垃圾）。
"""

from __future__ import annotations

import os
from pathlib import Path

from config.settings import _rebase_saved_paths

REAL_ROOT = str(Path(__file__).resolve().parent.parent)
OLD = r"C:\Users\joy46\Documents\aeroplan-studio"


def _saved(root=OLD, **extra):
    d = {
        "project_root": root,
        "data_dir": os.path.join(root, "data"),
        "config_dir": os.path.join(root, "config"),
        "log_dir": os.path.join(root, "data", "logs"),
    }
    d.update(extra)
    return d


# ── 前置斷言：沒搬家時什麼都不該動 ──────────────────────────────────────

def test_same_root_is_untouched():
    """⚠ 少了這條，下面每一條都可能只是「這個函式一律改寫」而已。"""
    same = _saved(root=REAL_ROOT)
    assert _rebase_saved_paths(same) == same


def test_missing_or_empty_root_is_a_noop():
    assert _rebase_saved_paths({}) == {}
    assert _rebase_saved_paths({"data_dir": "/x"}) == {"data_dir": "/x"}
    assert _rebase_saved_paths({"project_root": "", "data_dir": "/x"}) == \
        {"project_root": "", "data_dir": "/x"}


# ── 搬家：舊根目錄底下的路徑要跟著搬 ────────────────────────────────────

def test_paths_under_the_old_root_are_rebased():
    out = _rebase_saved_paths(_saved())
    assert out["project_root"] == REAL_ROOT
    assert out["data_dir"] == os.path.join(REAL_ROOT, "data")
    assert out["config_dir"] == os.path.join(REAL_ROOT, "config")
    assert out["log_dir"] == os.path.join(REAL_ROOT, "data", "logs")


def test_rebased_config_dir_actually_finds_the_vehicle_profiles():
    """⭐ 這就是那則警告的來源：校正之後檔案必須真的找得到。"""
    out = _rebase_saved_paths(_saved())
    assert os.path.exists(os.path.join(out["config_dir"], "vehicle_profiles.yaml"))


# ── 不可以好心辦壞事：使用者自訂到別處的路徑要留著 ──────────────────────

def test_paths_outside_the_old_root_are_left_alone():
    """使用者刻意把匯出目錄指到 D 槽 ⇒ 搬家不該把它拉回專案裡。"""
    custom = r"D:\my-exports"
    out = _rebase_saved_paths(_saved(export_dir=custom))
    assert out["export_dir"] == custom
    assert out["data_dir"] == os.path.join(REAL_ROOT, "data")      # 其餘仍照常校正


def test_a_sibling_directory_is_not_mistaken_for_a_child():
    """``…/aeroplan-studio-backup`` 只是**前綴**相同，不是舊根目錄的子目錄。

    ⚠ 少了分隔符檢查就會把它一起搬走。
    """
    sibling = OLD + "-backup"
    out = _rebase_saved_paths(_saved(cache_dir=sibling))
    assert out["cache_dir"] == sibling


def test_non_string_values_survive():
    out = _rebase_saved_paths(_saved(some_flag=True, some_num=3))
    assert out["some_flag"] is True and out["some_num"] == 3


def test_it_says_so_out_loud(capsys):
    """⚠ 靜靜校正跟靜靜不校正一樣糟 —— 使用者要知道路徑被改過。"""
    _rebase_saved_paths(_saved())
    out = capsys.readouterr().out
    assert "專案已搬家" in out and REAL_ROOT in out, out
    # 沒搬家時不可以吵
    _rebase_saved_paths(_saved(root=REAL_ROOT))
    assert "專案已搬家" not in capsys.readouterr().out
