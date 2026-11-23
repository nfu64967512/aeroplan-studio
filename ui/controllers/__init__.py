"""ui.controllers — MainWindow 邏輯拆分子模組 (2026 重構)

原本 main_window.py 達 5731 行為 God Class，拆分為以下 Mixin：

    strike_controller.StrikeControllerMixin
        蜂群打擊 (Swarm Strike) 所有 handlers
        包含 STOT/DTOT、VTOL、SITL 上傳、OWA 參數、ReconToStrike 動態切換

    stot_ros2_controller.StotRos2ControllerMixin
        地圖點擊 → WSL ROS2 STOT 全流程（run_west.sh）的橋接
        ⚠️ 與上面那一條是**平行的第二條實作**，不是取代。
           兩者同時跑會搶控制權，所以啟動時會用 dashboard.set_strike_running()
           互鎖。實體分成兩個檔案，review diff 時一眼就能確認舊路徑零改動。

    rclick_strike_controller.RClickStrikeControllerMixin
        地圖右鍵「以此點執行 STOT 打擊」→ **已經待命中**的 tot_node
        ⚠️ **第三條**路徑。上面兩條都是「一按就從頭跑一整趟」；這一條假設
           SITL 與 ROS2 層都已經常駐（兩個 CMD 開好的），只送一個座標過去。
           不拆 SITL、不起 ROS2 層、不殺 tot_node。

使用方式 (main_window.py)::

    from ui.controllers import (StrikeControllerMixin, StotRos2ControllerMixin,
                               RClickStrikeControllerMixin)

    class MainWindow(QMainWindow, StrikeControllerMixin, StotRos2ControllerMixin,
                     RClickStrikeControllerMixin):
        ...

Mixin 不持有任何狀態，所有 self.* 屬性由 MainWindow 提供
（StotRos2ControllerMixin 例外：它的狀態由 self._stot() 延遲建立，
  這樣 main_window.__init__ 不必為它多開欄位）。
"""
from .strike_controller import StrikeControllerMixin
from .stot_ros2_controller import StotRos2ControllerMixin
from .rclick_strike_controller import RClickStrikeControllerMixin

__all__ = ['StrikeControllerMixin', 'StotRos2ControllerMixin',
           'RClickStrikeControllerMixin']
