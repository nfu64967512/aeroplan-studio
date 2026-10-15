"""測試絕不寫進使用者的 data/logs —— 鎖住根 conftest.py 與 utils.logger 的環境變數開關

背景
----
utils.logger 第一次 get_logger() 就掛 FileHandler 到
<repo>/data/logs/UAVPathPlanner_<日期>.log，那是使用者 GUI 的真日誌。
任何 import 專案模組的 pytest 都會寫進去；2026-10-07 一次診斷在 App 關閉後
追加了 124 行假紀錄（假 PID 1001、「FORCE ARM」警告）。

舊的根 conftest 只做 ``logging.getLogger('UAVPathPlanner').setLevel(WARNING)``，
而 Logger.__init__ 一被 import 就把等級設回 INFO ⇒ 它從來沒擋住過任何一行。

這裡分兩半
----------
* 本行程的實況：import 一個用 get_logger() 的真模組、寫一行，
  data/logs 不可出現這一行，本行程也不可有任何指向 data/logs 的 FileHandler。
* 對照組（證明上面那半不是空轉）：拿掉開關時預設目錄確實是 data/logs，
  而且開著時 FileHandler 確實會寫檔。對照組一律導到 tmp_path，
  即使 logger 改壞了也不會寫進使用者的日誌。
"""
import logging
import os
import uuid
from datetime import datetime
from pathlib import Path

import pytest

from utils import logger as ulog

REPO = Path(__file__).resolve().parent.parent
USER_LOG_DIR = REPO / "data" / "logs"


def _snapshot(d: Path) -> dict:
    if not d.is_dir():
        return {}
    return {p.name: (p.stat().st_size, p.stat().st_mtime_ns) for p in d.iterdir() if p.is_file()}


def _all_loggers() -> list:
    return [logging.getLogger()] + [
        lg for lg in logging.Logger.manager.loggerDict.values() if isinstance(lg, logging.Logger)
    ]


def _file_handlers_into(d: Path) -> list:
    root = os.path.normcase(str(d.resolve()))
    return [
        f"{lg.name} -> {h.baseFilename}"
        for lg in _all_loggers()
        for h in lg.handlers
        if isinstance(h, logging.FileHandler) and os.path.normcase(h.baseFilename).startswith(root)
    ]


def _flush_all() -> None:
    for lg in _all_loggers():
        for h in lg.handlers:
            h.flush()


def _close(lg: ulog.Logger) -> None:
    for h in list(lg.logger.handlers):
        lg.logger.removeHandler(h)
        h.close()


def _probe_name() -> str:
    return f"no_user_log_probe_{uuid.uuid4().hex[:12]}"


# ------------------------------------------------------------------
# 本行程的實況
# ------------------------------------------------------------------
def test_conftest_turns_file_log_off():
    assert os.environ.get(ulog.NO_FILE_LOG_ENV) == "1", \
        "conftest.py（根或 tests/）沒有設 AEROPLAN_NO_FILE_LOG=1（被刪了？）⇒ 測試會寫進使用者的 data/logs"
    assert ulog.resolve_log_dir() is None


def test_importing_a_real_module_and_logging_does_not_write_user_logs():
    before = _snapshot(USER_LOG_DIR)

    import mission.sitl_launcher as sl          # 模組層級就 logger = get_logger()

    tag = f"{_probe_name()} 這一行不可以出現在 data/logs"
    sl.logger.error(tag)                        # ERROR：不會被任何等級濾掉
    _flush_all()

    assert sl.logger.log_file is None, f"get_logger() 掛了 FileHandler：{sl.logger.log_file}"
    assert ulog.get_logger().log_file is None
    assert _file_handlers_into(USER_LOG_DIR) == [], "本行程有 FileHandler 指向使用者的 data/logs"

    # 只看這段期間有變動的檔：GUI 若同時開著，它自己寫日誌是正常的，不能誤判成測試寫的
    after = _snapshot(USER_LOG_DIR)
    changed = [n for n, st in after.items() if before.get(n) != st]
    leaked = [n for n in changed
              if tag in (USER_LOG_DIR / n).read_text(encoding="utf-8", errors="replace")]
    assert leaked == [], f"測試的日誌行寫進了使用者的日誌：{leaked}"


# ------------------------------------------------------------------
# 對照組：證明上面不是因為 FileHandler 本來就不會寫
# ------------------------------------------------------------------
def test_without_switch_default_is_the_user_log_dir(monkeypatch):
    monkeypatch.delenv(ulog.NO_FILE_LOG_ENV, raising=False)
    monkeypatch.delenv(ulog.LOG_DIR_ENV, raising=False)
    assert Path(ulog.resolve_log_dir()).resolve() == USER_LOG_DIR.resolve()


def test_file_handler_really_writes_when_switch_is_off(monkeypatch, tmp_path):
    env_dir = tmp_path / "env"
    arg_dir = tmp_path / "arg"
    monkeypatch.delenv(ulog.NO_FILE_LOG_ENV, raising=False)
    monkeypatch.setenv(ulog.LOG_DIR_ENV, str(env_dir))

    name = _probe_name()
    lg = ulog.setup_logger(name, log_dir=str(arg_dir), log_to_console=False)
    try:
        tag = f"{name} 開關關掉時要真的寫進檔案"
        lg.error(tag)
        _flush_all()
        assert lg.log_file is not None
        log_file = Path(lg.log_file)
        # AEROPLAN_LOG_DIR 連呼叫端明傳的 log_dir 一起取代
        assert log_file.parent == env_dir
        assert log_file.name == f"{name}_{datetime.now():%Y%m%d}.log"
        assert tag in log_file.read_text(encoding="utf-8")
    finally:
        _close(lg)
    assert not arg_dir.exists()


@pytest.mark.parametrize("value", ["1", "true", "YES", "on", "y", "anything"])
def test_switch_values_that_turn_file_log_off(monkeypatch, tmp_path, value):
    monkeypatch.setenv(ulog.NO_FILE_LOG_ENV, value)
    monkeypatch.setenv(ulog.LOG_DIR_ENV, str(tmp_path / "env"))
    assert ulog.resolve_log_dir() is None

    # main.py 走的就是 setup_logger(log_to_file=True) 這條
    lg = ulog.setup_logger(_probe_name(), log_dir=str(tmp_path / "arg"), log_to_file=True,
                           log_to_console=False)
    try:
        assert lg.log_file is None
        assert not any(isinstance(h, logging.FileHandler) for h in lg.logger.handlers)
    finally:
        _close(lg)
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("value", ["", "0", "false", "No", " off "])
def test_switch_values_that_leave_file_log_on(monkeypatch, tmp_path, value):
    monkeypatch.setenv(ulog.NO_FILE_LOG_ENV, value)
    monkeypatch.setenv(ulog.LOG_DIR_ENV, str(tmp_path))
    assert ulog.resolve_log_dir() == str(tmp_path)
