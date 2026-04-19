"""
暫存 HTML 檔案集中管理
========================

所有由 Folium / Cesium / polygon_editor 產生的暫存 HTML 全部寫入
``<project_root>/data/tmp_maps/``，程式啟動時自動清除整個資料夾，
確保不會累積垃圾檔案。

用法::

    from utils.temp_cache import (
        get_temp_html_dir, clear_temp_html_dir, new_temp_html_path,
    )

    # 程式啟動時清空（建議在 MainWindow 初始化或 main() 入口呼叫）
    clear_temp_html_dir()

    # 需要寫 HTML 時取得可用的檔名
    path = new_temp_html_path(prefix='cesium')   # → .../data/tmp_maps/cesium_0001.html
    with open(path, 'w', encoding='utf-8') as f:
        f.write(html)

檔案保留策略：
    - 預設覆蓋策略：每個 prefix 最多保留 `keep_last` 個舊檔
      （預設 keep_last=1 → 只留最新一份，前一份自動刪除）
    - 程式啟動時 clear_temp_html_dir() 一次性全清
"""
from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Optional

from utils.logger import get_logger

logger = get_logger()


# 專案根目錄 (utils/ 的上層)
_PROJECT_ROOT = Path(__file__).resolve().parent.parent

# 暫存 HTML 資料夾
_TEMP_HTML_DIR = _PROJECT_ROOT / 'data' / 'tmp_maps'


def get_temp_html_dir() -> Path:
    """回傳暫存 HTML 資料夾路徑（自動建立）"""
    _TEMP_HTML_DIR.mkdir(parents=True, exist_ok=True)
    return _TEMP_HTML_DIR


def clear_temp_html_dir() -> int:
    """清空 data/tmp_maps/ 內所有 .html 檔案。

    Returns
    -------
    int
        被刪除的檔案數量
    """
    d = get_temp_html_dir()
    removed = 0
    for p in d.iterdir():
        if p.is_file() and p.suffix.lower() == '.html':
            try:
                p.unlink()
                removed += 1
            except OSError as e:
                logger.warning(f'[TempCache] 無法刪除 {p}: {e}')
    if removed:
        logger.info(f'[TempCache] 啟動清理：移除 {removed} 個舊暫存 HTML')
    return removed


def new_temp_html_path(prefix: str = 'map', keep_last: int = 1) -> Path:
    """產生一個新的暫存 HTML 檔案路徑（覆蓋策略 — 不累積）。

    Parameters
    ----------
    prefix :
        檔名前綴 (例如 'cesium', 'folium', 'polygon_editor')
    keep_last :
        該前綴最多保留幾個舊檔案；超過者自動刪除 (預設 1 = 只留最新一份)

    Returns
    -------
    Path
        新檔案的完整路徑（尚未寫入）；呼叫者自行用 open() 寫入
    """
    d = get_temp_html_dir()

    # 找出已存在的同 prefix 檔案 (依 mtime 排序)
    pattern = re.compile(rf'^{re.escape(prefix)}_(\d+)\.html$')
    existing: list[tuple[int, Path]] = []
    max_seq = 0
    for p in d.iterdir():
        if not p.is_file():
            continue
        m = pattern.match(p.name)
        if m:
            seq = int(m.group(1))
            existing.append((seq, p))
            max_seq = max(max_seq, seq)

    # 依 mtime 刪除超過 keep_last 的舊檔
    existing.sort(key=lambda x: x[1].stat().st_mtime)
    if len(existing) >= keep_last:
        # 要新增 1 個 → 只能留 keep_last - 1 個舊的
        remove_count = len(existing) - max(keep_last - 1, 0)
        for _, p in existing[:remove_count]:
            try:
                p.unlink()
            except OSError:
                pass

    # 產生新序號
    new_seq = max_seq + 1
    new_path = d / f'{prefix}_{new_seq:04d}.html'
    return new_path


def purge_legacy_temp_html_in_root() -> int:
    """一次性清理舊版堆在專案根目錄的 ``tmp*.html``（遷移期工具）。

    Returns
    -------
    int
        被刪除的數量
    """
    removed = 0
    for p in _PROJECT_ROOT.iterdir():
        if p.is_file() and p.name.startswith('tmp') and p.suffix == '.html':
            try:
                p.unlink()
                removed += 1
            except OSError:
                pass
    if removed:
        logger.info(f'[TempCache] 清理專案根目錄舊版暫存 HTML: {removed} 個')
    return removed
