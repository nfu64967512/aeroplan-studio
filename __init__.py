"""
AeroPlan Studio — Collaborative UAV Mission Planning Suite
版本: 2.5.0
"""

__version__ = "2.5.0"
__author__ = "AeroPlan Studio"
__license__ = "MIT"

from .config.settings import get_settings, init_settings
from .utils.logger import get_logger

__all__ = [
    'get_settings',
    'init_settings',
    'get_logger',
]
