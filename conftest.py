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

# 3D 模型合併快取（ui/widgets/glb_merge.py）：建 CesiumMapWidget 的測試不可以把 ~180 MB 的合併檔寫進
# repo 的 data/cache/models/。要測合併的測試自己 setenv("AEROPLAN_GLB_MERGE", "1") 並指定 tmp 快取目錄。
os.environ.setdefault("AEROPLAN_GLB_MERGE", "0")

import pytest  # noqa: E402


@pytest.fixture(autouse=True, scope="session")
def _plan_snapshot_dir(tmp_path_factory):
    """飛行計畫快照（survey/plan_snapshot.py）：建圖上傳的測試（例如 test_hdmap_fix_gui 跑 _on_mission_uploaded）
    會寫快照；沒設 AEROPLAN_PLAN_SNAPSHOT_DIR 時 default_root() 在 pytest 中丟例外（不可以寫進 repo 的 runs/survey）
    ⇒ 整個測試階段指到 pytest 的暫存目錄。要測預設路徑的測試自己 monkeypatch。"""
    key = "AEROPLAN_PLAN_SNAPSHOT_DIR"
    if os.environ.get(key):
        yield
        return
    os.environ[key] = str(tmp_path_factory.mktemp("plan_snapshots"))
    try:
        yield
    finally:
        os.environ.pop(key, None)
