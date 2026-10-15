"""pytest 根設定 —— 測試絕不寫進使用者的 data/logs

utils.logger 第一次 get_logger() 就會掛 FileHandler 到
<repo>/data/logs/UAVPathPlanner_<日期>.log（使用者 GUI 的真日誌）；
2026-10-07 一次診斷因此在 App 關閉後追加了 124 行假紀錄。

pytest 在 import 任何測試模組前就會執行本檔，所以在這裡打開開關即可；
測試啟動的子行程也會繼承這個環境變數。開關的語意見 utils/logger.py::resolve_log_dir。
⚠ 在 tests/ 裡面執行 pytest 時本檔不會被載入 ⇒ tests/conftest.py 設同一個開關。
鎖在 tests/test_logger_no_user_log.py。
"""
import os

os.environ["AEROPLAN_NO_FILE_LOG"] = "1"
