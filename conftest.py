"""Pytest 全域 fixture 設定 — 確保測試可從任何目錄執行

pytest 會自動偵測此檔並在 import 測試模組前執行。
此檔負責：
    1. 將專案根目錄加入 sys.path
    2. 避開專案根 __init__.py 的相對匯入問題
    3. 設定測試時的統一日誌等級
"""
import sys
from pathlib import Path

# 將專案根目錄置於 sys.path 首位
_project_root = Path(__file__).resolve().parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# 測試時抑制 INFO 日誌，只看 WARNING+
import logging
logging.getLogger('UAVPathPlanner').setLevel(logging.WARNING)
