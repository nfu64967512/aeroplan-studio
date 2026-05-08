"""ui.controllers — MainWindow 邏輯拆分子模組 (2026 重構)

原本 main_window.py 達 5731 行為 God Class，拆分為以下 Mixin：

    strike_controller.StrikeControllerMixin
        蜂群打擊 (Swarm Strike) 所有 handlers
        包含 STOT/DTOT、VTOL、SITL 上傳、OWA 參數、ReconToStrike 動態切換

使用方式 (main_window.py)::

    from ui.controllers import StrikeControllerMixin

    class MainWindow(QMainWindow, StrikeControllerMixin):
        ...

Mixin 不持有任何狀態，所有 self.* 屬性由 MainWindow 提供。
"""
from .strike_controller import StrikeControllerMixin

__all__ = ['StrikeControllerMixin']
