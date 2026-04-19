"""
AeroPlan Studio — Collaborative UAV Mission Planning Suite
版本: 2.6.0

注意：本檔實際上沒有被 main.py 使用（main.py 採用 top-level import）。
為相容性保留 lazy 載入，避免在 pytest / 工具掃描時因相對匯入失敗。
"""

__version__ = "2.6.0"
__author__ = "AeroPlan Studio"
__license__ = "MIT"

# 延後匯入 — 避免非 package 執行環境 (pytest / IDE 掃描) 出錯
try:
    from .config.settings import get_settings, init_settings      # noqa: F401
    from .utils.logger import get_logger                           # noqa: F401
    __all__ = ['get_settings', 'init_settings', 'get_logger']
except ImportError:
    __all__ = []
