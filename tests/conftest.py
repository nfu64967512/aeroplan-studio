"""tests/ 的 pytest 設定 —— 與根 conftest.py 相同的日誌開關

在 tests/ 裡面執行 pytest（例如 ``cd tests && pytest test_x.py``）時，沒有 pytest.ini
可以把 rootdir 釘在 repo 根，rootdir 會變成 tests/，根 conftest.py **不會被載入**，
get_logger() 就又寫進使用者的 data/logs（2026-10-08 實測漏了一行）。這裡補上同一個開關。
"""
import os

os.environ["AEROPLAN_NO_FILE_LOG"] = "1"
