"""tests/ 的 pytest 設定 —— 與根 conftest.py 相同的日誌開關

在 tests/ 裡面執行 pytest（例如 ``cd tests && pytest test_x.py``）時，沒有 pytest.ini
可以把 rootdir 釘在 repo 根，rootdir 會變成 tests/，根 conftest.py **不會被載入**，
get_logger() 就又寫進使用者的 data/logs（2026-10-08 實測漏了一行）。這裡補上同一個開關。
"""
import os

os.environ["AEROPLAN_NO_FILE_LOG"] = "1"

# 3D 模型合併快取（ui/widgets/glb_merge.py）：建 CesiumMapWidget 的測試不可以把 ~180 MB 的合併檔寫進
# repo 的 data/cache/models/。要測合併的測試自己 setenv("AEROPLAN_GLB_MERGE", "1") 並指定 tmp 快取目錄。
os.environ.setdefault("AEROPLAN_GLB_MERGE", "0")

import pytest  # noqa: E402


@pytest.fixture(autouse=True, scope="session")
def _plan_snapshot_dir(tmp_path_factory):
    """飛行計畫快照：與根 conftest.py 同一個設定（在 tests/ 裡面跑 pytest 時根 conftest 不會被載入）。"""
    key = "AEROPLAN_PLAN_SNAPSHOT_DIR"
    if os.environ.get(key):
        yield
        return
    os.environ[key] = str(tmp_path_factory.mktemp("plan_snapshots"))
    try:
        yield
    finally:
        os.environ.pop(key, None)
