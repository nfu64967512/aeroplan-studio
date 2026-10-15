"""
日誌工具模組
提供統一的日誌管理功能，支援文件輸出和格式化
"""

import logging
import os
import sys
from datetime import datetime
from typing import Optional
from pathlib import Path


# ==========================================
# 日誌等級映射
# ==========================================
LOG_LEVELS = {
    'DEBUG': logging.DEBUG,
    'INFO': logging.INFO,
    'WARNING': logging.WARNING,
    'ERROR': logging.ERROR,
    'CRITICAL': logging.CRITICAL
}


# ==========================================
# 日誌檔目錄（環境變數開關）
# ==========================================
# 預設寫進 <repo>/data/logs —— 那是使用者 GUI 的真日誌。任何 import 專案模組的
# pytest／診斷腳本都會經由 get_logger() 寫進去（2026-10-07 一次診斷追加了 124 行假紀錄），
# 所以測試與腳本要用下列環境變數把它關掉或導走。GUI 正常啟動時兩個都不會設。
#   AEROPLAN_NO_FILE_LOG   設為 0/false/no/off/空字串以外的任何值 ⇒ 完全不掛 FileHandler
#   AEROPLAN_LOG_DIR       取代日誌目錄（連呼叫端明傳的 log_dir 一起取代）
NO_FILE_LOG_ENV = 'AEROPLAN_NO_FILE_LOG'
LOG_DIR_ENV = 'AEROPLAN_LOG_DIR'
DEFAULT_LOG_DIR = os.path.join(Path(__file__).resolve().parent.parent, 'data', 'logs')


def resolve_log_dir(log_dir: Optional[str] = None) -> Optional[str]:
    """
    決定 FileHandler 要寫的目錄

    優先序: AEROPLAN_NO_FILE_LOG > AEROPLAN_LOG_DIR > 參數 log_dir > DEFAULT_LOG_DIR

    返回:
        目錄路徑；None 表示不掛 FileHandler
    """
    # 認不得的值一律當「關」—— 寧可少寫一份日誌，也不要讓測試寫進使用者的真日誌
    if os.environ.get(NO_FILE_LOG_ENV, '').strip().lower() not in ('', '0', 'false', 'no', 'off'):
        return None
    return os.environ.get(LOG_DIR_ENV) or log_dir or DEFAULT_LOG_DIR


# ==========================================
# 日誌格式定義
# ==========================================
class LogFormatter(logging.Formatter):
    """自訂日誌格式器，支援顏色輸出（終端機）"""
    
    # ANSI 顏色碼
    COLORS = {
        'DEBUG': '\033[36m',      # 青色
        'INFO': '\033[32m',       # 綠色
        'WARNING': '\033[33m',    # 黃色
        'ERROR': '\033[31m',      # 紅色
        'CRITICAL': '\033[35m',   # 紫色
        'RESET': '\033[0m'
    }
    
    def __init__(self, use_color: bool = True):
        """
        初始化格式器
        
        參數:
            use_color: 是否使用顏色
        """
        super().__init__(
            fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.use_color = use_color
    
    def format(self, record):
        """格式化日誌記錄"""
        if self.use_color and sys.stdout.isatty():
            # 終端機環境，使用顏色
            levelname = record.levelname
            if levelname in self.COLORS:
                record.levelname = (
                    f"{self.COLORS[levelname]}{levelname}{self.COLORS['RESET']}"
                )
        
        return super().format(record)


# ==========================================
# 日誌管理器
# ==========================================
class Logger:
    """統一的日誌管理器"""
    
    _instances = {}
    
    def __init__(self, name: str = 'UAVPathPlanner',
                level: str = 'INFO',
                log_dir: Optional[str] = None,
                log_to_file: bool = True,
                log_to_console: bool = True):
        """
        初始化日誌管理器
        
        參數:
            name: 日誌名稱
            level: 日誌等級
            log_dir: 日誌目錄
            log_to_file: 是否輸出到文件
            log_to_console: 是否輸出到控制台
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(LOG_LEVELS.get(level.upper(), logging.INFO))
        
        # 清除現有處理器
        self.logger.handlers.clear()
        
        # 控制台處理器 — 強制 UTF-8 避免 Windows cp950 遇到 emoji/✓/⚠ 爆炸
        if log_to_console:
            # 在 Windows 上，sys.stdout 預設是 cp950；改用 UTF-8 包裝
            # 搭配 errors='replace' 以防極端情況無法編碼也不中斷日誌
            try:
                import io as _io
                stream = _io.TextIOWrapper(
                    sys.stdout.buffer, encoding='utf-8',
                    errors='replace', write_through=True,
                )
            except Exception:
                stream = sys.stdout   # fallback
            console_handler = logging.StreamHandler(stream)
            console_handler.setFormatter(LogFormatter(use_color=True))
            self.logger.addHandler(console_handler)
        
        # 文件處理器
        # 實際寫入的檔案路徑（沒掛 FileHandler 時為 None），供呼叫端與測試核對生效值
        self.log_file: Optional[str] = None
        log_dir = resolve_log_dir(log_dir) if log_to_file else None
        if log_dir is not None:
            # 確保日誌目錄存在
            os.makedirs(log_dir, exist_ok=True)

            # 建立日誌文件（按日期命名）
            log_filename = f"{name}_{datetime.now().strftime('%Y%m%d')}.log"
            log_filepath = os.path.join(log_dir, log_filename)

            file_handler = logging.FileHandler(log_filepath, encoding='utf-8')
            file_handler.setFormatter(LogFormatter(use_color=False))
            self.logger.addHandler(file_handler)
            self.log_file = log_filepath
        
        # 防止日誌向上傳播
        self.logger.propagate = False
    
    def debug(self, message: str, exc_info: bool = False):
        """輸出 DEBUG 等級日誌"""
        self.logger.debug(message, exc_info=exc_info)

    def info(self, message: str, exc_info: bool = False):
        """輸出 INFO 等級日誌"""
        self.logger.info(message, exc_info=exc_info)

    def warning(self, message: str, exc_info: bool = False):
        """輸出 WARNING 等級日誌"""
        self.logger.warning(message, exc_info=exc_info)

    def error(self, message: str, exc_info: bool = False):
        """輸出 ERROR 等級日誌"""
        self.logger.error(message, exc_info=exc_info)

    def critical(self, message: str, exc_info: bool = False):
        """輸出 CRITICAL 等級日誌"""
        self.logger.critical(message, exc_info=exc_info)

    def exception(self, message: str):
        """輸出異常日誌（包含堆疊追蹤）"""
        self.logger.exception(message)
    
    def set_level(self, level: str):
        """
        設定日誌等級
        
        參數:
            level: 日誌等級（DEBUG, INFO, WARNING, ERROR, CRITICAL）
        """
        self.logger.setLevel(LOG_LEVELS.get(level.upper(), logging.INFO))
    
    @classmethod
    def get_instance(cls, name: str = 'UAVPathPlanner', **kwargs):
        """
        獲取日誌實例（單例模式）
        
        參數:
            name: 日誌名稱
            **kwargs: 其他初始化參數
        
        返回:
            Logger 實例
        """
        if name not in cls._instances:
            cls._instances[name] = cls(name, **kwargs)
        return cls._instances[name]


# ==========================================
# 便捷函數
# ==========================================
def setup_logger(name: str = 'UAVPathPlanner',
                level: str = 'INFO',
                log_dir: Optional[str] = None,
                log_to_file: bool = True,
                log_to_console: bool = True) -> Logger:
    """
    設定日誌管理器
    
    參數:
        name: 日誌名稱
        level: 日誌等級
        log_dir: 日誌目錄
        log_to_file: 是否輸出到文件
        log_to_console: 是否輸出到控制台
    
    返回:
        Logger 實例
    """
    return Logger(name, level, log_dir, log_to_file, log_to_console)


def get_logger(name: str = 'UAVPathPlanner') -> Logger:
    """
    獲取日誌實例
    
    參數:
        name: 日誌名稱
    
    返回:
        Logger 實例
    """
    return Logger.get_instance(name)


# ==========================================
# 日誌裝飾器
# ==========================================
def log_function_call(logger: Optional[Logger] = None):
    """
    日誌裝飾器，記錄函數調用
    
    參數:
        logger: Logger 實例（可選）
    
    使用範例:
        @log_function_call()
        def my_function(arg1, arg2):
            pass
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            _logger = logger or get_logger()
            _logger.debug(f"調用函數: {func.__name__}")
            try:
                result = func(*args, **kwargs)
                _logger.debug(f"函數 {func.__name__} 執行成功")
                return result
            except Exception as e:
                _logger.error(f"函數 {func.__name__} 執行失敗: {e}")
                raise
        return wrapper
    return decorator


def log_execution_time(logger: Optional[Logger] = None):
    """
    日誌裝飾器，記錄函數執行時間
    
    參數:
        logger: Logger 實例（可選）
    
    使用範例:
        @log_execution_time()
        def my_function():
            pass
    """
    import time
    
    def decorator(func):
        def wrapper(*args, **kwargs):
            _logger = logger or get_logger()
            start_time = time.time()
            
            try:
                result = func(*args, **kwargs)
                execution_time = time.time() - start_time
                _logger.info(f"函數 {func.__name__} 執行時間: {execution_time:.4f} 秒")
                return result
            except Exception as e:
                execution_time = time.time() - start_time
                _logger.error(
                    f"函數 {func.__name__} 執行失敗 "
                    f"({execution_time:.4f} 秒): {e}"
                )
                raise
        return wrapper
    return decorator


# ==========================================
# 全局日誌實例
# ==========================================
logger = get_logger()
