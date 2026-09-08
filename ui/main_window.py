"""
主視窗模組
整合地圖、參數面板、任務面板等核心 UI 組件
"""

import math
from typing import Optional, Tuple
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QSplitter, QStatusBar, QToolBar, QMessageBox,
    QFileDialog, QLabel, QScrollArea, QApplication
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence, QShortcut

# 修正導入路徑
from config import get_settings
from utils.logger import get_logger
from utils.file_io import write_waypoints, create_waypoint_line
from utils.math_utils import (  # 0-2 標準化：WGS84 單一來源
    EARTH_RADIUS_M, point_at_distance_bearing,
)


# ═════════════════════════════════════════════════════════════════════
#  可測接縫：生成點幾何 / 上傳參數來源
#  （抽成模組層級函式，才能用值層級測試鎖住，不必去戳 MainWindow 的巨大方法）
# ═════════════════════════════════════════════════════════════════════

#: 生成點間距至少要比避撞門檻高這個倍率，避免「剛好等於門檻」的刀鋒情況。
SPAWN_SPACING_MARGIN = 1.2


#: FOV 光錐的參數來源標記。兩個分頁共用同一個 3D 光錐實體,
#: 所以「現在吃誰的參數」必須是一個**明確的值**,不能靠哪個 checkbox 剛好被勾。
FOV_SOURCE_TACTICAL = 'tactical'
FOV_SOURCE_VTOL = 'vtol'

#: 兩個面板的光錐勾選框屬性名(**故意不同**,歷史因素)。集中在這裡,
#: 由 test_both_panels_expose_a_fov_checkbox 保證兩邊都找得到。
FOV_CHECKBOX_ATTRS = ('fov_cone_check', '_fov_cone_check')

#: 相對高度低於這個值就當作「還在地面」。Cesium 端對 `!(alt > 0)` 會直接
#: 刪掉該機光錐(main.html:2715),所以貼地時本來就畫不出來。
FOV_AIRBORNE_MIN_ALT_M = 0.5


def fov_pose_summary(poses: dict) -> tuple:
    """(收到姿態的機數, 其中真正離地的機數)。

    ⚠⚠ 為什麼要分開數:VTOL 停在起降場時 `alt_rel ≈ 0`,Cesium 端會**每一拍
    刪掉**那架的光錐(連 per-sysid 狀態一起 delete)。那正好是使用者第一次
    勾選這個框的時刻 ⇒ 什麼都不會出現,看起來就是功能沒做完。
    不是壞掉,但必須講出來 —— 沉默地畫不出東西是這個專案最貴的失效模式。
    """
    n = len(poses)
    airborne = sum(1 for p in poses.values()
                   if len(p) > 2 and p[2] > FOV_AIRBORNE_MIN_ALT_M)
    return n, airborne


#: Cesium 端 frustum 的**硬夾制**(ui/widgets/cesium_templates/main.html:2455/2625)
#: 掛載角 Math.max(0, Math.min(89.5, am));半角 clamp 到 [1, 85]。
FOV_CONE_MOUNT_RANGE = (0.0, 89.5)
FOV_CONE_HALF_ANGLE_RANGE = (1.0, 85.0)


def fov_cone_clamp_warning(mount_deg: float, hfov_deg: float,
                           vfov_deg: float) -> str:
    """3D 光錐會**默默夾掉**哪些值?沒有就回空字串。

    ⚠⚠ 為什麼需要這個:戰術頁的掛載角範圍是 0~85,永遠碰不到 JS 的夾制;
    但 VTOL 頁是 **-84~+84** —— 把 VTOL 頁接上光錐,等於第一次讓使用者
    碰得到「設 -30 度、畫出來卻是 0 度(nadir)」而**完全沒有提示**。
    面板的足跡數字仍用 -30 算 ⇒ 面板與 3D 畫面對不上,而兩邊都不會報錯。

    ⇒ 夾制本身是 Cesium 那側的既有限制,這裡不改它;但**不可以靜靜地**
      讓顯示值與實際渲染值分家。
    """
    lo_m, hi_m = FOV_CONE_MOUNT_RANGE
    lo_a, hi_a = FOV_CONE_HALF_ANGLE_RANGE
    msgs = []
    if mount_deg < lo_m:
        msgs.append(f"掛載角 {mount_deg:.1f}° 會被 3D 光錐夾成 {lo_m:.0f}°"
                    f"(向後傾的角錐畫不出來)")
    elif mount_deg > hi_m:
        msgs.append(f"掛載角 {mount_deg:.1f}° 會被夾成 {hi_m:.1f}°")
    for name, full in (("HFOV", hfov_deg), ("VFOV", vfov_deg)):
        half = full / 2.0
        if half > hi_a:
            msgs.append(f"{name} {full:.1f}° 的半角會被夾成 {hi_a:.0f}°")
        elif 0 < half < lo_a:
            msgs.append(f"{name} {full:.1f}° 的半角會被撐成 {lo_a:.0f}°")
    return "；".join(msgs)


#: SAR 熱力圖的公尺/度換算。
#: ⚠⚠ **必須**和 ui/widgets/cesium_templates/main.html 的 `mPerDegLat = 111320.0`
#: 一致。JS 端用它算 `cellWidthM` 再推 `normalizedW = 掃描寬度 / 格子寬`,
#: 面板若用別的常數,顯示出來的「格子邊長」就不是 JS 實際採用的那個
#: —— 兩個數字都對得起自己,合起來卻是錯的(本專案已有前例:兩套
#: 每度公尺常數讓生成點間距差 0.000229 m 而卡在避撞門檻上)。
HEATMAP_M_PER_DEG = 111320.0

#: 超過這個格數就提醒重繪成本。每格一個 Cesium Entity,且改色會讓整批
#: primitive 重建(非 per-instance 屬性路徑),所以成本隨格數**超線性**成長。
HEATMAP_HEAVY_CELLS = 4000


def heatmap_cell_size_m(lat_min, lat_max, lon_min, lon_max, grid):
    """網格解析度換算成格子邊長(南北 m, 東西 m)。

    JS 端 `initSARHeatmap` 把區域切成 grid x grid,所以格子邊長完全由
    **區域跨距 / 行列數** 決定 —— 而區域是使用者在地圖上畫的,
    面板自己看不到。「解析度 20」對使用者是個沒有單位的數字。
    """
    g = max(1, int(grid))
    import math as _m
    lat_mid = (float(lat_min) + float(lat_max)) / 2.0
    ns = abs(float(lat_max) - float(lat_min)) * HEATMAP_M_PER_DEG / g
    ew = (abs(float(lon_max) - float(lon_min)) * HEATMAP_M_PER_DEG
          * _m.cos(_m.radians(lat_mid)) / g)
    return ns, ew


def heatmap_resolution_note(cell_ns_m, cell_ew_m, sweep_width_m, grid):
    """給使用者看的格子大小說明 + 取樣不足警告。

    ⚠⚠ 判準是**格子邊長 vs 掃描寬度**,不是「格子看起來大不大」。
    JS 的覆蓋判定是「格子**中心點**落不落在足跡內」(_pointInConvexQuad),
    所以格子比足跡大時,一格會整格翻或整格不翻 —— 畫面上就是一大塊一大塊,
    而且覆蓋率會同時**高估**(掃到一角就整格算掃過)與**低估**(擦過中心以外)。
    那不是渲染不夠細,是**取樣不足**,調再多顏色也修不好。
    """
    worst = max(cell_ns_m, cell_ew_m)
    cells = grid * grid
    txt = (f"{cell_ns_m:.0f} × {cell_ew_m:.0f} m／格"
           f"（{grid}×{grid} = {cells:,} 格）")
    # ⚠⚠ 成本不是線性的。每格是一個獨立 Cesium Entity,而更新時是
    #   `entity.rectangle.material = 新 Color` —— 這條路徑**不走** Cesium 的
    #   per-instance 顏色屬性快取(那只給 updatersWithAttributes 的 updater),
    #   所以每次改色都會讓整批 primitive 重建。格數一多,飛行中每一拍都在重建。
    #   ⇒ 使用者有權在按下去之前就知道,而不是飛到一半發現卡頓。
    if cells > HEATMAP_HEAVY_CELLS:
        txt += "　⚠ 格數偏多，飛行中重繪可能變慢"
    if sweep_width_m > 0 and worst > sweep_width_m:
        # ⚠ 必須**無條件進位**。四捨五入會給出一個「照做了還是會被警告」的
        #   建議值(實測 141/50*20 = 56.4 → 56 ⇒ 格子 50.3 m 仍 > 50)。
        #   給一個不管用的數字比不給更糟 —— 使用者會照做,然後發現沒解決。
        import math as _mm
        need = int(_mm.ceil(worst / sweep_width_m * grid))
        return (txt + f"　⚠ 格子比掃描寬度 {sweep_width_m:.0f} m 還大"
                      f"（覆蓋判定只看格子中心）；要對得上感測器footprint，"
                      f"解析度需 ≈ {need}"), False
    return txt, True


def resolve_fov_param_source(source, tactical_panel, vtol_panel):
    """決定由哪個面板提供 FOV 光錐參數。

    ⚠⚠ 這個接縫存在的理由跟 `build_avoidance_kwargs` 一樣:
    「光錐畫出來了」和「光錐吃的是**這一頁**的參數」在畫面上長得一模一樣。
    兩頁的 HFOV 預設都是 60、VFOV 都是 45 ⇒ 接錯來源時**完全看不出來**,
    只有當使用者去改其中一頁的值才會發現改了沒用 ——
    正是 [[two-pipelines-one-is-dead]] 的形狀。

    ⚠ 回傳的是**面板物件**,呼叫端一律用 `get_fov_radius()` /
    `get_fov_visual_params()` 取值 —— 兩個面板刻意提供同名方法。

    Args:
        source: FOV_SOURCE_VTOL / FOV_SOURCE_TACTICAL / None
        tactical_panel: 戰術分析頁(parameter_panel),永遠存在
        vtol_panel: VTOL 偵察頁,可能還沒建立 ⇒ None 時退回戰術頁
    """
    if source == FOV_SOURCE_VTOL and vtol_panel is not None:
        return vtol_panel
    return tactical_panel


def build_avoidance_kwargs(params: dict, cruise_speed_mps: float) -> dict:
    """把面板參數翻成 CollisionAvoidance 的建構引數。

    ⚠⚠ 拉出來當**接縫**的理由:這一層以前是寫在呼叫點的一長串 kwargs,
    漏掉一個參數不會有任何症狀 —— 規劃器吃自己的預設,而預設常常剛好等於
    面板預設 ⇒ 只有使用者**改動**那一格時才會發現「改了沒用」。
    掃描角就是這樣壞掉的(面板送了、消費端沒收),避撞上限差一點重演。

    ⚠ 這裡**不做任何夾制或修正**:面板顯示什麼就送什麼,不合理的組合由
    CollisionAvoidance 自己大聲警告。偷偷改值會製造「顯示的值 ≠ 用的值」。
    """
    return dict(
        min_separation_m=float(params.get('min_separation_m', 100.0)),
        alt_offset_m=float(params.get('alt_offset_m', 30.0)),
        time_step_s=1.0,
        cruise_speed_mps=float(cruise_speed_mps),
        strategy=params.get('avoid_strategy', 'altitude'),
        # ⚠ None 是合法值(= 自動),不可以 float(None)
        max_alt_offset_m=(None if params.get('max_alt_offset_m') is None
                          else float(params['max_alt_offset_m'])),
    )


def resolve_spawn_spacing_m(params: dict, logger_=None) -> float:
    """決定多機生成點的間距（公尺）。

    ⚠ 間距**不可等於**避撞門檻。舊行為：VTOL 偵察頁的 get_params() 不送
    ``takeoff_spacing`` ⇒ 退回 50.0，而同一頁送出的 ``min_separation_m`` 也是
    50.0 ⇒ 間距恰好壓在門檻上，浮點誤差往哪邊倒都可能被判成衝突。
    """
    min_sep = float(params.get('min_separation_m', 0.0) or 0.0)
    spacing = float(params.get('takeoff_spacing', 0.0) or 0.0)
    if spacing <= 0.0:
        return max(50.0, min_sep * SPAWN_SPACING_MARGIN)
    if min_sep > 0.0 and spacing < min_sep * SPAWN_SPACING_MARGIN:
        if logger_ is not None:
            logger_.warning(
                f'[生成點] 起飛間距 {spacing:.1f} m 未高過避撞門檻 '
                f'{min_sep:.1f} m 的 {SPAWN_SPACING_MARGIN} 倍 ⇒ 相鄰機在 t=0 '
                f'就可能被判成衝突。自動放大到 {min_sep * SPAWN_SPACING_MARGIN:.1f} m。'
            )
        return min_sep * SPAWN_SPACING_MARGIN
    return spacing


def spawn_positions(base_lat: float, base_lon: float, num_drones: int,
                    takeoff_bearing_deg: float, spacing_m: float) -> list:
    """沿起飛方向的垂直方向、以起飛點為中心對稱排開 N 個生成點。

    ⚠ 一定要用 ``point_at_distance_bearing``（它用 ``EARTH_RADIUS_M``），
    不可再用手刻的 111320.0 m/deg：避撞偵測的 ``_haversine`` 用的是
    ``EARTH_RADIUS_M``（2πR/360 = 111319.4908 m/deg），兩個常數差 0.51 m/deg，
    會讓「排好 50 m」量起來只有 49.999771 m，剛好掉到 50 m 門檻底下，
    於是 t=0 每一組相鄰機都被誤判成衝突（cos φ 會約掉 ⇒ 與緯度無關）。
    """
    if num_drones <= 1:
        return [(base_lat, base_lon)]
    out = []
    for i in range(num_drones):
        offset_m = (i - (num_drones - 1) / 2.0) * spacing_m
        brg = (takeoff_bearing_deg + 90.0) % 360.0
        if offset_m < 0.0:
            brg = (brg + 180.0) % 360.0
        out.append(point_at_distance_bearing(base_lat, base_lon, abs(offset_m), brg))
    return out


def resolve_upload_speed_radius(planned: dict, panel_params: dict) -> tuple:
    """上傳飛控要用的 (巡航速度, 迴轉半徑)。

    ⚠ 優先用「這份規劃當時實際使用的」值。``panel_params`` 來自**基本演算法**
    分頁；用 VTOL 偵察頁規劃時那個 panel 可能從沒被碰過，``speed`` 還停在初始
    預設 3.0 m/s ⇒ 匯出的檔寫 18 m/s、上傳的任務卻是 3 m/s。
    """
    planned = planned or {}
    speed = float(panel_params.get('speed', 15.0))
    radius = float(panel_params.get('turn_radius', 50.0))
    if planned.get('cruise_speed_mps'):
        speed = float(planned['cruise_speed_mps'])
    if planned.get('turn_radius_m'):
        radius = float(planned['turn_radius_m'])
    return speed, radius
from mission import MissionManager
from core.global_planner.coverage_planner import CoveragePlanner, CoverageParameters, ScanPattern
from core.geometry.polygon import (
    coverage_angle_to_compass, fw_scan_angle_to_compass,
)
from core.global_planner.astar import AStarPlanner
from core.global_planner.rrt import RRTPlanner, RRTStarPlanner
from core.global_planner.fixed_wing_planner import (
    FixedWingPlanner, FixedWingParameters, TurnRadiusChecker,
)
from core.collision import CollisionChecker
from mission.swarm_coordinator import SwarmCoordinator

# 2026 重構：Swarm Strike 邏輯已抽出至 Controller Mixin (ui/controllers/)
# StotRos2ControllerMixin 是**平行的第二條**打擊路徑（WSL ROS2 管線），
# 與 StrikeControllerMixin 的 MAVLink 上傳路徑互不相干，兩者以 dashboard 互鎖。
# RClickStrikeControllerMixin 是**第三條**：右鍵一點 → 送給已待命的 tot_node，
# 不拆 SITL、不起 ROS2 層（那兩件事由兩個常駐 CMD 負責）。
from ui.controllers import (
    StrikeControllerMixin, StotRos2ControllerMixin, RClickStrikeControllerMixin,
)
# 右鍵語意（到達 / 攻擊）與高度 0 快捷的判定放在純邏輯層，UI 只負責跳窗。
from integrations import rclick_strike

# 戰術蜂群打擊面板（VTOL 全任務生命週期）— 彈出式對話框
from ui.widgets.tactical_swarm_strike_panel import TacticalSwarmStrikeDialog

# 獲取配置和日誌實例
settings = get_settings()
logger = get_logger()

# 常數定義
MIN_CORNERS = 3   # 最少邊界點數量
MAX_CORNERS = 100  # 最大邊界點數量
_WP_LIMIT = 655   # Mission Planner 單次最大匯入航點數

# HV-DCCPP 兩個構型的顯示名稱（數值以 docs/平台參數表.md 為準，這裡只給人看）
HV_PLATFORM_NAMES = {'A': 'Pioneer PNP', 'B': 'HERO PNP'}

# ── 關視窗時還在跑的 HV-DCCPP 規劃執行緒（見 _hv_park_running_worker）────────
#   放在模組層級是刻意的：這裡是唯一在視窗死後還握著它的地方。
_HV_ORPHAN_WORKERS: set = set()


def _hv_wait_for_orphan_workers(timeout_ms: int = 60000) -> None:
    """程式要結束了：等寄放中的規劃執行緒跑完。"""
    for w in list(_HV_ORPHAN_WORKERS):
        try:
            if w.isRunning():
                logger.info('[HV-DCCPP] 等待背景規劃收尾…')
                w.wait(timeout_ms)
        except RuntimeError:
            pass
        _HV_ORPHAN_WORKERS.discard(w)


def _hv_park_running_worker(window, worker) -> None:
    """把「視窗要關了但還在跑」的規劃執行緒從視窗手上接過來。

    ⚠⚠ 為什麼不能不管它：
      * 若 worker 是 MainWindow 的 QObject 子物件，視窗被銷毀時 Qt 會對一個
        **還在跑的 QThread** 呼叫解構子 ⇒ qFatal（`QThread: Destroyed while thread
        is still running`），整個程式 abort，而不是正常關閉。
      * 就算 parent=None，只要 Python 這邊沒人再參考它，GC 一收就是同一種死法。
      * 也不能 terminate()：numpy/scipy 的 C 層會留下不一致狀態。
      * 更不能「等它自己停」：hv 的求解器沒有取消點，而且轉接層會把 progress
        回呼丟出來的例外吞掉 —— stop() 只能讓結果不被採用，停不了計算。
    所以：拆掉所有連到（即將消失的）視窗的信號，把物件存到模組層級，
    程式結束前再統一等它們收尾。
    """
    try:
        window._hv_disconnect_worker(worker)
    except Exception:
        pass
    _HV_ORPHAN_WORKERS.add(worker)
    try:
        worker.finished.connect(lambda w=worker: _HV_ORPHAN_WORKERS.discard(w))
    except Exception:
        pass
    try:
        from PyQt6.QtWidgets import QApplication
        from PyQt6.QtCore import Qt as _Qt
        app = QApplication.instance()
        if app is not None:
            # UniqueConnection：多關幾次視窗也只會接一次
            app.aboutToQuit.connect(_hv_wait_for_orphan_workers,
                                    _Qt.ConnectionType.UniqueConnection)
    except Exception:
        pass
    logger.warning('[HV-DCCPP] 視窗關閉時規劃仍在執行，已寄放該執行緒等它自行結束')


class MainWindow(QMainWindow, StrikeControllerMixin, StotRos2ControllerMixin,
                 RClickStrikeControllerMixin):
    """
    主視窗類
    
    整合地圖顯示、參數控制、任務管理等核心功能
    """
    
    # 信號定義
    mission_changed = pyqtSignal(object)  # 任務變更信號
    waypoints_updated = pyqtSignal(list)  # 航點更新信號
    
    def __init__(self):
        """初始化主視窗"""
        super().__init__()

        # 視窗基本設置（使用 settings 替代 Config）
        self.setWindowTitle(settings.ui.window_title)
        self._apply_responsive_geometry()
        self.setMinimumSize(settings.ui.min_window_width, settings.ui.min_window_height)
        
        # 初始化核心組件
        self.mission_manager = MissionManager()
        
        # 初始化變數
        self.init_variables()
        
        # 建立 UI
        self.init_ui()
        
        # 載入樣式表
        self.load_stylesheet()
        
        # 顯示歡迎信息
        self.statusBar().showMessage("無人機路徑規劃工具已就緒", 5000)
        
        logger.info("主視窗初始化完成")
    
    def _apply_responsive_geometry(self,
                                    target_ratio: float = 0.85,
                                    min_w: int = 1280,
                                    min_h: int = 720):
        """依螢幕尺寸動態決定主視窗大小，跨 1080p / 2K / 4K 不跑版。

        規則：
          - 取得主螢幕 availableGeometry (扣掉工作列)
          - 預設佔 85% × 85% 空間
          - 水平置中、垂直偏上 (讓標題列不會被遮住)
          - 視 DPR 縮放：4K 上也會維持合理視覺比例
          - 夾底：不小於 min_w × min_h (1280×720)

        Parameters
        ----------
        target_ratio : float
            視窗佔螢幕可用區域的比例 (0~1)
        min_w, min_h : int
            視窗最小像素尺寸 (logical px)
        """
        try:
            from PyQt6.QtGui import QGuiApplication
            screen = (self.screen() if hasattr(self, 'screen') and self.screen()
                      else QGuiApplication.primaryScreen())
            if screen is None:
                # fallback: 使用 settings 固定值
                self.setGeometry(100, 100,
                                 settings.ui.window_width,
                                 settings.ui.window_height)
                return

            avail = screen.availableGeometry()   # QRect (logical px，已扣工作列)
            scr_w, scr_h = avail.width(), avail.height()

            # 計算目標尺寸 (依比例 + 下限)
            win_w = max(int(scr_w * target_ratio), min_w)
            win_h = max(int(scr_h * target_ratio), min_h)
            # 上限不能超過螢幕可用區 (4K 下會卡到 3400 左右)
            win_w = min(win_w, scr_w)
            win_h = min(win_h, scr_h)

            # 居中對齊 (垂直偏上 8%)
            x = avail.left() + (scr_w - win_w) // 2
            y = avail.top() + max(0, (scr_h - win_h) // 3)

            self.setGeometry(x, y, win_w, win_h)

            # DPI 資訊記錄 (供除錯用)
            try:
                dpr = screen.devicePixelRatio()
                logger.info(
                    f'[Window] 螢幕 {scr_w}x{scr_h} (DPR={dpr:.1f}), '
                    f'視窗尺寸 {win_w}x{win_h} @ ({x},{y})'
                )
            except Exception:
                pass
        except Exception as e:
            logger.warning(f'[Window] 自適應尺寸失敗，使用 settings 預設: {e}')
            self.setGeometry(100, 100,
                             settings.ui.window_width,
                             settings.ui.window_height)

    def init_variables(self):
        """初始化變數"""
        self.current_mission = None
        self.corners = []  # 邊界點（**正在畫**的那一塊區）
        # ── 模式二：多個作業區 ────────────────────────────────────────
        #   ⚠ 刻意不動 `self.corners` 的語義 —— 全檔有 83 處在用它（既有單區
        #     規劃、圍籬建構、匯出、螺旋規劃…），改語義會牽動一大片。
        #     所以另開一個「已完成的區」清單，`corners` 維持「正在畫的那一塊」。
        #     兩者都空時，行為與改動前**逐位元相同**。
        self.survey_areas: list = []      # [[(lat, lon), ...], ...] 已完成的作業區
        self.waypoints = []  # 航點（扁平化，供單台匯出使用）
        self.spiral_waypoint_altitudes: list = []  # 螺旋掃描每航點高度（alt_step > 0 時有效）
        self.sub_paths = []  # 各子區域路徑（分割模式時使用）
        self.obstacles = []  # 障礙物

        # 當前演算法
        self.current_algorithm = 'grid'

        # 當前載具設定
        self.current_vehicle_type = '多旋翼'
        self.current_vehicle_model = 'DJI Mavic 3'

        # 飛行參數
        self.flight_params = {
            'altitude': 10.0,
            'speed': 3.0,
            'angle': 0.0,
            'spacing': 20.0,
            'yaw_speed': 60.0,
            'subdivisions': 1,
            'region_spacing': 3.0,
            'gap_spacings_m': [],        # 各邊界獨立間隔
            'v_spacing_m': 3.0,          # 網格垂直間隔
            'transit_stagger_enabled': False,
            'transit_stagger_alt': 10.0,
            'turn_radius': 50.0,  # 固定翼轉彎半徑 (m)
            'lock_heading': False,
            'heading_angle': 0.0,
            # ── 固定翼專用 ─────────────────────────────────────
            'fw_takeoff_bearing': 0.0,
            'fw_runway_length': 50.0,
            'fw_landing_bearing': 180.0,
            'fw_pattern_alt': 80.0,
            'fw_downwind_offset': 200.0,
            'fw_pattern_leg': 300.0,
            'fw_final_dist': 400.0,
            'fw_scan_mode': 'spiral',
            'fw_autoland': False,
            # ── 螺旋專用 ────────────────────────────────────────
            'spiral_curvature': 1.0,
            'spiral_alt_step': 0.0,
        }

        # 固定翼任務結果（三階段路徑）
        self.fw_mission_result: dict = {}

        # 禁航區清單（FixedWingNFZPlanner 使用）
        # 每筆格式：{'type': 'polygon'/'circle', 'name': str,
        #            'vertices': [(lat,lon),...] | 'center':(lat,lon), 'radius': float}
        self.nfz_zones: list = []

        # 螺旋/同心圓掃描圓心模式
        self.circle_center: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.picking_circle_center: bool = False

        # 固定翼起飛點（跑道位置，獨立於掃描區中心）
        self.home_point: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.picking_home_point: bool = False

        self._dccpp_result = None  # 當前 DCCPP 最佳化結果

        # ── HV-DCCPP（異構雙構型 VTOL）狀態 ──
        self._hv_worker = None        # HvPlanningWorker（背景規劃執行緒）
        self._hv_coordinator = None   # hv_dccpp 的 HvDccppCoordinator（匯出／上傳／重規劃）
        self._hv_classes: dict = {}   # {drone_id: 'A'|'B'}
        self._hv_display_ctx: dict = {}
        self._hv_last_report: dict = {}   # 最近一次 on_report 的重點（未覆蓋、被夾的參數…）
        self._hv_on_done = None           # 背景規劃完成後要叫的回呼（見 _hv_notify_done）
        self._hv_alternates = None    # [(lat, lon), ...] 備降點；目前由呼叫端填，UI 尚未提供點選

        # 即時路徑生成設定（已停用：改由「預覽」按鈕 / Enter 鍵手動觸發，見 on_preview_paths）
        # 保留屬性以維持 _schedule_path_generation / _auto_generate_path 方法的向後相容
        self.auto_generate_path = False
        self.path_generation_timer = None  # 延遲生成計時器
        self.path_generation_delay = 300   # 延遲時間 (ms)

        # 戰術蜂群打擊面板（VTOL 全任務生命週期）— 單例浮動對話框
        self._swarm_strike_dialog: Optional[TacticalSwarmStrikeDialog] = None
        self.picking_swarm_launch: bool = False   # 地圖點擊設為蜂群起飛點
        self.picking_swarm_target: bool = False   # 地圖點擊設為蜂群目標
    
    def init_ui(self):
        """初始化 UI 組件"""
        # 創建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 主佈局：垂直排列 — 上方為「地圖 | 控制面板」splitter；
        # 下方為「STOT TIME-ON-TARGET 同步狀態儀表板」（蜂群打擊規劃完成時顯示）。
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 創建分割器（地圖 | 控制面板）— 可拖拉決定比例
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(6)
        splitter.setChildrenCollapsible(False)
        # 全域 QSS 已涵蓋 QSplitter 樣式（border + hover NEUTRAL cyan）

        # 左側：地圖區域
        self.map_widget = self.create_map_widget()
        splitter.addWidget(self.map_widget)

        # 右側：控制面板
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # 依「視窗寬度」自適應分割比例（地圖 60%，控制面板 40%）
        # 使用視窗寬度而非螢幕寬度，避免 4K 上 splitter 超出視窗
        win_w = self.width() or 1280
        map_w   = int(win_w * 0.60)
        panel_w = int(win_w * 0.40)
        splitter.setStretchFactor(0, 60)
        splitter.setStretchFactor(1, 40)
        splitter.setSizes([map_w, panel_w])
        # 快取 splitter 以便 resizeEvent 時重新分配
        self._main_splitter = splitter
        
        main_layout.addWidget(splitter, 1)   # stretch 1 → 占滿剩餘空間

        # ── STOT TIME-ON-TARGET Dashboard（底部固定高度）──────
        # 蜂群打擊規劃完成時顯示 4-12 機 TTT + Loiter/Delay 補償；
        # 平時隱藏不佔空間，clear() 後也會自動隱藏。
        # KAMIKAZE LAUNCH 按鈕 → 觸發 GCS 端排程同步啟動所有 SITL。
        try:
            from ui.widgets.strike_ttt_dashboard import StrikeTTTDashboard
            self.strike_ttt_dashboard = StrikeTTTDashboard(self)
            self.strike_ttt_dashboard.setVisible(False)   # 預設隱藏
            main_layout.addWidget(self.strike_ttt_dashboard, 0)
            # 連接按鈕信號到 strike controller
            self.strike_ttt_dashboard.launch_requested.connect(
                self.launch_kamikaze_synchronized
            )
            # TERMINAL SYNC → 閉環終端同步打擊（FleetRegistry 共享黑板 + ToT 修正）
            self.strike_ttt_dashboard.terminal_sync_requested.connect(
                self.launch_terminal_sync_strike
            )
            self.strike_ttt_dashboard.abort_requested.connect(
                self.strike_ttt_dashboard.stop_countdown
            )
            # ABORT 同時中止閉環終端同步打擊（若進行中）
            self.strike_ttt_dashboard.abort_requested.connect(
                self.abort_terminal_sync
            )
        except Exception as e:
            logger.warning(f'[TTT Dashboard] 初始化失敗: {e}')
            self.strike_ttt_dashboard = None

        # 創建工具列
        self.create_toolbar()
        
        # 創建狀態列
        self.create_statusbar()
        
        # 創建選單
        self.create_menus()

        # 設置快捷鍵
        self.setup_shortcuts()

        # 建立 Undo/Redo Stack（Ctrl+Z / Ctrl+Y）
        try:
            from ui.undo_commands import setup_undo_stack
            setup_undo_stack(self)
        except Exception as e:
            logger.warning(f'[Undo] 初始化失敗: {e}')
    
    def create_map_widget(self):
        """創建地圖組件（DualMapWidget：2D Folium + 3D Cesium 雙模式）"""
        from ui.widgets.dual_map_widget import DualMapWidget

        map_widget = DualMapWidget(self)

        # 連接信號
        map_widget.corner_added.connect(self.on_corner_added)
        map_widget.corner_moved.connect(self.on_corner_moved)
        # 3D 右鍵 GUIDED 飛到此點（Mission Planner 風格：右鍵 → 跳高度框 → 送長機）
        if hasattr(map_widget, 'guided_goto_requested'):
            map_widget.guided_goto_requested.connect(self.on_rclick_guided_goto)

        return map_widget

    def open_click_map_window(self):
        """打開可點擊的地圖視窗（使用 tkintermapview）"""
        try:
            import tkinter as tk
            import tkintermapview

            # 創建 Tkinter 視窗
            self.tk_map_window = tk.Toplevel()
            self.tk_map_window.title("點擊地圖添加角點 - 左鍵點擊添加")
            self.tk_map_window.geometry("900x700")

            # 創建地圖
            map_widget = tkintermapview.TkinterMapView(
                self.tk_map_window,
                width=900,
                height=650,
                corner_radius=0
            )
            map_widget.pack(fill="both", expand=True)

            # 設置位置
            map_widget.set_position(
                settings.map.default_lat,
                settings.map.default_lon
            )
            map_widget.set_zoom(settings.map.default_zoom)

            # 嘗試設置 Google 衛星圖
            try:
                map_widget.set_tile_server(
                    "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                    max_zoom=20
                )
            except:
                pass

            # 儲存標記列表
            self.tk_markers = []
            self.tk_polygon = None

            def on_click(coords):
                lat, lon = coords
                # 添加標記
                point_num = len(self.corners) + 1
                marker = map_widget.set_marker(
                    lat, lon,
                    text=f"P{point_num}",
                    marker_color_circle="green"
                )
                self.tk_markers.append(marker)

                # 添加到主視窗
                self.on_manual_corner_added(lat, lon)

                # 更新多邊形
                if len(self.corners) >= 3:
                    if hasattr(self, 'tk_polygon') and self.tk_polygon:
                        self.tk_polygon.delete()
                    self.tk_polygon = map_widget.set_polygon(
                        self.corners,
                        fill_color="green",
                        outline_color="darkgreen",
                        border_width=2
                    )

                logger.info(f"Tkinter 地圖點擊: ({lat:.6f}, {lon:.6f})")

            map_widget.add_left_click_map_command(on_click)

            # 添加說明標籤
            info_frame = tk.Frame(self.tk_map_window)
            info_frame.pack(fill="x", pady=5)
            tk.Label(
                info_frame,
                text="左鍵點擊地圖添加角點 | 滾輪縮放 | 右鍵拖曳移動",
                font=("Arial", 10)
            ).pack()

            logger.info("已打開 Tkinter 地圖視窗")

        except ImportError:
            QMessageBox.warning(
                self, "缺少套件",
                "請先安裝 tkintermapview:\n\npip install tkintermapview"
            )
        except Exception as e:
            logger.error(f"打開 Tkinter 地圖失敗: {e}")
            QMessageBox.critical(self, "錯誤", f"打開地圖視窗失敗:\n{str(e)}")
    
    def create_control_panel(self):
        """創建控制面板（右側）"""
        from ui.widgets.parameter_panel import ParameterPanel
        from ui.widgets.mission_panel import MissionPanel

        # 外層容器：垂直排列 [ScrollArea(參數面板)] + [任務面板]
        panel_widget = QWidget()
        panel_layout = QVBoxLayout(panel_widget)
        panel_layout.setContentsMargins(4, 4, 4, 4)
        panel_layout.setSpacing(6)

        # 參數面板
        self.parameter_panel = ParameterPanel(self)
        self.parameter_panel.corner_added.connect(self.on_manual_corner_added)
        self.parameter_panel.clear_corners_requested.connect(self.on_clear_corners)
        self.parameter_panel.remove_last_corner_requested.connect(self.on_delete_last_corner)
        self.parameter_panel.open_click_map_requested.connect(self.open_click_map_window)
        self.parameter_panel.pick_center_requested.connect(self.on_pick_circle_center)
        self.parameter_panel.drag_circle_requested.connect(self.on_drag_circle_mode)
        self.map_widget.circle_defined.connect(self.on_circle_drag_defined)
        self.parameter_panel.pick_home_point_requested.connect(self.on_pick_home_point)
        self.parameter_panel.clear_home_point_requested.connect(self.on_clear_home_point)
        self.parameter_panel.manage_nfz_requested.connect(self.open_nfz_dialog)
        self.parameter_panel.nfz_draw_polygon_requested.connect(self.on_nfz_draw_polygon)
        self.parameter_panel.nfz_draw_circle_requested.connect(self.on_nfz_draw_circle)
        self.parameter_panel.nfz_poly_finish_requested.connect(self.on_nfz_poly_finish)
        self.map_widget.nfz_polygon_drawn.connect(self.on_nfz_polygon_drawn)
        self.map_widget.nfz_circle_drawn.connect(self.on_nfz_circle_drawn)
        # ── 統一 Fence Zone（NFZ + 威脅 + 圍籬） ──
        self.parameter_panel.fence_zone_create_requested.connect(
            self.open_fence_zone_dialog
        )
        self.parameter_panel.fence_zone_clear_all_requested.connect(
            self.clear_all_fence_zones
        )
        self.parameter_panel.fence_zone_upload_requested.connect(
            self.upload_fence_zones_to_fc
        )
        # 直接從 3D 地圖繪製 — 完成後自動開對話框預填頂點
        self.parameter_panel.fence_zone_draw_polygon_requested.connect(
            self.on_fence_zone_draw_polygon
        )
        self.parameter_panel.fence_zone_draw_circle_requested.connect(
            self.on_fence_zone_draw_circle
        )
        # 地圖頂部工具列 — 不論側邊面板在哪個分頁，這 3 個按鈕都看得到
        if hasattr(self.map_widget, 'fence_zone_draw_polygon_requested'):
            self.map_widget.fence_zone_draw_polygon_requested.connect(
                self.on_fence_zone_draw_polygon
            )
            self.map_widget.fence_zone_draw_circle_requested.connect(
                self.on_fence_zone_draw_circle
            )
            self.map_widget.fence_zone_manage_requested.connect(
                self.open_fence_zone_dialog
            )
        # 旗標：地圖完成繪製時，由此值決定要路由到原 NFZ 流程或新 Fence 流程
        #   None     → 原 NFZ 流程（沿用既有行為）
        #   'polygon'→ 完成多邊形後開 FenceZoneDialog
        #   'circle' → 完成圓形後開 FenceZoneDialog
        self._fence_draw_pending: str | None = None
        # FenceZoneRegistry singleton — 對 add/remove 即時同步 3D 視覺 + 面板計數
        from mission.fence_zone import FenceZoneRegistry, ZoneCategory
        _fz_reg = FenceZoneRegistry.instance()
        _fz_reg.zone_added.connect(self._on_fence_zone_added)
        _fz_reg.zone_removed.connect(self._on_fence_zone_removed)

        def _on_fz_changed():
            n_all = _fz_reg.count()
            # DCCPP 繞行只考慮 NFZ + THREAT（GEOFENCE 是 inclusion 性質，不繞）
            n_avoid = len(_fz_reg.by_category(ZoneCategory.NFZ)) \
                    + len(_fz_reg.by_category(ZoneCategory.THREAT))
            self.parameter_panel.update_fence_zone_count(n_all, n_avoid)
            if hasattr(self, 'vtol_recon_panel'):
                self.vtol_recon_panel.update_fence_zone_count(n_all, n_avoid)

        _fz_reg.zones_changed.connect(_on_fz_changed)
        self.parameter_panel.dccpp_coverage_requested.connect(self._on_dccpp_coverage_requested)
        self.parameter_panel.dccpp_export_requested.connect(self._on_dccpp_export_requested)
        self.parameter_panel.vtol_export_requested.connect(self._on_vtol_export_requested)
        self.parameter_panel.delete_last_corner_requested.connect(self.on_delete_last_corner)
        self.parameter_panel.dem_loaded.connect(self._on_dem_loaded)
        self.parameter_panel.fence_export_requested.connect(self._on_fence_export)
        # 圍籬建構完成 → 同步更新所有 Tab 的狀態標籤
        if hasattr(self.map_widget, 'fence_built'):
            self.map_widget.fence_built.connect(self.parameter_panel.update_fence_status)

        # 將參數面板包進 QScrollArea，解決高解析度以外螢幕時內容被截斷的問題
        scroll = QScrollArea()
        scroll.setWidget(self.parameter_panel)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        # 自適應最小寬度：螢幕寬度的 22%，至少 280px
        screen = QApplication.primaryScreen()
        min_panel_w = max(280, int((screen.availableGeometry().width() if screen else 1280) * 0.22))
        scroll.setMinimumWidth(min_panel_w)

        # 任務面板
        self.mission_panel = MissionPanel(self)

        # 以 QSplitter 取代 VBox → 使用者可拖拉決定「參數面板 / 任務面板」比例
        from PyQt6.QtWidgets import QSplitter, QSizePolicy
        self._right_panel_splitter = QSplitter(Qt.Orientation.Vertical)
        self._right_panel_splitter.setChildrenCollapsible(False)
        self._right_panel_splitter.setHandleWidth(6)
        # 全域 QSS 已涵蓋 QSplitter 樣式
        scroll.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.mission_panel.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred
        )
        self._right_panel_splitter.addWidget(scroll)
        self._right_panel_splitter.addWidget(self.mission_panel)
        # 預設比例：參數面板 78% / 任務面板 22%
        self._right_panel_splitter.setStretchFactor(0, 78)
        self._right_panel_splitter.setStretchFactor(1, 22)
        panel_layout.addWidget(self._right_panel_splitter, 1)

        # SITL HUD 分頁（Mission Planner 風格即時遙測）
        from ui.widgets.sitl_hud import SITLHud
        self.sitl_hud = SITLHud(self)
        self.sitl_hud.connect_requested.connect(self.on_sitl_connect)
        self.sitl_hud.connect_embedded_requested.connect(self.on_embedded_swarm_connect)
        self.sitl_hud.disconnect_requested.connect(self.on_sitl_disconnect)
        self.sitl_hud.uav_disconnect_requested.connect(self.on_sitl_disconnect_one)
        self.sitl_hud.launch_sitl_requested.connect(self.on_sitl_launch)
        self.sitl_hud.stop_sitl_requested.connect(self.on_sitl_stop_local)
        self.sitl_hud.cmd_arm.connect(lambda: self._sitl_broadcast('arm'))
        self.sitl_hud.cmd_disarm.connect(lambda: self._sitl_broadcast('disarm'))
        self.sitl_hud.cmd_set_mode.connect(lambda m: self._sitl_broadcast('set_mode', m))
        self.sitl_hud.cmd_takeoff.connect(lambda a: self._sitl_broadcast('takeoff', a))
        self.sitl_hud.cmd_upload.connect(self.on_sitl_upload_mission)
        self.sitl_hud.cmd_download.connect(self.on_sitl_download_mission)
        self.sitl_hud.cmd_upload_fence.connect(self.on_sitl_upload_fence)
        self.sitl_hud.cmd_auto_start.connect(lambda: self._sitl_broadcast('auto_start'))
        self.sitl_hud.cmd_guided_takeoff.connect(
            lambda a: self._sitl_broadcast('guided_takeoff', a))
        self.sitl_hud.cmd_guided_goto_mode.connect(self.on_guided_goto_mode)
        self.sitl_hud.cmd_guided_takeoff_mode.connect(self.on_guided_takeoff_mode)
        self.sitl_hud.cmd_param_set.connect(
            lambda n, v, t: self._sitl_broadcast('set_param', (n, v, t))
        )
        self.sitl_hud.cmd_param_get.connect(
            lambda n: self._sitl_broadcast('get_param', n)
        )
        self.sitl_hud.cmd_params_batch.connect(
            lambda items: self._sitl_broadcast('set_params', items)
        )
        self.sitl_hud.cmd_release_store.connect(self.on_release_store)
        self.sitl_hud.cmd_vtol_transition.connect(
            lambda state: self._sitl_broadcast('vtol_transition', state)
        )

        # 加到參數面板的 QTabWidget 作為第 4 個分頁
        # SITL 分頁採內部 QTabWidget：
        #   - "Command"：ADOS 風 FleetRail + DroneDetailPanel（GCS 互動主入口）
        #   - "HUD / Servo"：既有 SITLHud（保留 SERVO OUTPUT、legacy 按鈕等）
        from PyQt6.QtWidgets import QTabWidget
        from ui.widgets.sitl_command_panel import SitlCommandPanel
        from ui.widgets.hil_connect_panel import HILConnectPanel

        sitl_inner_tabs = QTabWidget()

        # 「HUD / Servo」（含 SITL 連線 + 嵌入式蜂群入口）置於最前、為預設分頁
        sitl_hud_scroll = QScrollArea()
        sitl_hud_scroll.setWidget(self.sitl_hud)
        sitl_hud_scroll.setWidgetResizable(True)
        sitl_hud_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        sitl_inner_tabs.addTab(sitl_hud_scroll, "HUD / Servo")

        self.sitl_command_panel = SitlCommandPanel()
        sitl_inner_tabs.addTab(self.sitl_command_panel, "Command")

        # 「HIL」：實體飛控多機連線（最多 6 顆，序列埠 / udp / tcp）
        self.hil_panel = HILConnectPanel()
        self.hil_panel.connect_requested.connect(self._on_hil_connect_all)
        self.hil_panel.disconnect_requested.connect(self._on_hil_disconnect_all)
        sitl_inner_tabs.addTab(self.hil_panel, "HIL")

        # ── 「VTOL 偵察」：VTOL 群飛偵察一頁式工作流 ────────────────
        # 底層規劃仍走既有 DCCPP（本面板只是把 vehicle_type 釘成 vtol），
        # 額外負責 DCCPP 沒做的兩件事：感測器足跡建模、規劃後覆蓋率驗算。
        from ui.widgets.vtol_recon_panel import VTOLReconPanel
        self.vtol_recon_panel = VTOLReconPanel()
        self.vtol_recon_panel.pick_area_requested.connect(
            self.parameter_panel.open_click_map_requested.emit)
        self.vtol_recon_panel.clear_area_requested.connect(self.on_clear_corners)
        # 模式二：多個作業區
        self.vtol_recon_panel.commit_area_requested.connect(self.commit_current_area)
        self.vtol_recon_panel.clear_areas_requested.connect(self.clear_all_survey_areas)
        self.vtol_recon_panel.delete_last_corner_requested.connect(
            self.on_delete_last_corner)
        # 自動建構的電子圍籬狀態也餵給 VTOL 面板 —— 否則地圖上明明畫著圍籬，
        # ⑥ Fence 管理卻只顯示 registry 的 0 個，看起來像 bug（實際是兩套系統）。
        if hasattr(self.map_widget, 'fence_built'):
            self.map_widget.fence_built.connect(
                self.vtol_recon_panel.update_fence_status)
        # 面板建立時補一次現況（可能已經規劃過）
        self.vtol_recon_panel.update_fence_status(
            getattr(self.map_widget, 'last_fence_bundle', None))
        self.vtol_recon_panel.plan_requested.connect(self._on_vtol_recon_plan)
        self.vtol_recon_panel.verify_coverage_requested.connect(
            self._on_vtol_recon_verify)
        self.vtol_recon_panel.show_missed_requested.connect(
            self._on_vtol_recon_show_missed)
        self.vtol_recon_panel.show_heatmap_requested.connect(
            self._on_vtol_recon_heatmap)
        self.vtol_recon_panel.export_requested.connect(self._on_vtol_export_requested)
        self.vtol_recon_panel.fov_cone_toggle_requested.connect(
            self._on_vtol_fov_cone_toggle)
        self.vtol_recon_panel.fov_visual_changed.connect(
            self._on_vtol_fov_visual_changed)
        self.vtol_recon_panel.upload_requested.connect(self.on_sitl_upload_mission)
        self.vtol_recon_panel.launch_sitl_requested.connect(
            lambda n: self.on_sitl_launch('VTOL', n))
        # VTOL 分頁的 Fence 管理 —— 接同一組 handler、共用同一個 registry
        self.vtol_recon_panel.fence_zone_create_requested.connect(
            self.open_fence_zone_dialog)
        self.vtol_recon_panel.fence_zone_draw_polygon_requested.connect(
            self.on_fence_zone_draw_polygon)
        self.vtol_recon_panel.fence_zone_draw_circle_requested.connect(
            self.on_fence_zone_draw_circle)
        self.vtol_recon_panel.fence_zone_upload_requested.connect(
            self.upload_fence_zones_to_fc)
        self.vtol_recon_panel.fence_zone_clear_all_requested.connect(
            self.clear_all_fence_zones)
        # 邊界點在別的分頁 / 地圖上改動時即時同步，不必等到切分頁
        self.parameter_panel.corner_count_changed.connect(
            self.vtol_recon_panel.set_corner_count)
        vtol_recon_scroll = self.parameter_panel._make_resizable_tab(
            [self.vtol_recon_panel]
        )
        self._vtol_recon_tab_index = self.parameter_panel._tabs.addTab(
            vtol_recon_scroll, "VTOL 偵察"
        )

        self.parameter_panel._tabs.addTab(sitl_inner_tabs, "🛰 SITL")

        # 任務操作面板隨分頁切換可見元件：只有「基本演算法」分頁才顯示完整預覽/匯出
        self.parameter_panel._tabs.currentChanged.connect(self._on_main_tab_changed)
        self._on_main_tab_changed(0)  # 初始化

        self._sitl_link = None        # 主鏈路（向後相容）
        self._sitl_links: list = []   # 多機鏈路列表
        # 最近一次從飛控讀回的任務 {'sysid': int, 'items': [dict, ...]}
        self._fc_mission: dict | None = None
        self._hil_links: list = []    # HIL 實體飛控鏈路（_sitl_links 的子集）
        self._sitl_launcher = None
        
        # 連接信號
        self.parameter_panel.parameters_changed.connect(self.on_parameters_changed)
        self.mission_panel.preview_requested.connect(self.on_preview_paths)
        self.mission_panel.export_requested.connect(self.on_export_waypoints)
        self.mission_panel.clear_requested.connect(self.on_clear_all)

        # ── 戰術模組信號連接 ──────────────────────────────────────────
        self.parameter_panel.elevation_slicer_changed.connect(self._on_elev_slicer_changed)
        self.parameter_panel.elevation_slicer_cleared.connect(self._on_elev_slicer_cleared)
        self.parameter_panel.sar_heatmap_init_requested.connect(self._on_sar_heatmap_init)
        self.parameter_panel.sar_heatmap_reset_requested.connect(self._on_sar_heatmap_reset)
        self.parameter_panel.sar_heatmap_clear_requested.connect(self._on_sar_heatmap_clear)
        self.parameter_panel.fov_cone_toggle_requested.connect(self._on_fov_cone_toggle)
        self.parameter_panel.fov_visual_changed.connect(
            self._on_tactical_fov_visual_changed)
        self.parameter_panel.radar_sim_requested.connect(self._on_radar_sim)
        self.parameter_panel.radar_clear_requested.connect(self._on_radar_clear)
        self.parameter_panel.rcs_toggle_requested.connect(self._on_rcs_toggle)

        # 戰術模組內部狀態
        self._fov_cone_enabled = False   # FOV 光錐是否啟用
        # 目前由哪一頁提供光錐參數(None = 沒開)。見 resolve_fov_param_source()
        self._fov_cone_source = None
        self._rcs_enabled = False        # RCS 渲染是否啟用
        # 各 sysid 最近一筆 UAV 姿態快取（給 FOV 視覺參數變更時即時重投影用）
        #   {sysid: (lat, lon, alt, heading_deg, pitch_deg, roll_deg)}
        self._last_uav_pose_for_fov: dict[int, tuple] = {}

        # ── 蜂群打擊模組信號連接 ──────────────────────────────────────
        self.parameter_panel.strike_mark_targets_requested.connect(self._on_strike_mark_targets)
        self.parameter_panel.strike_mark_base_requested.connect(self._on_strike_mark_base)
        self.parameter_panel.strike_mode_changed.connect(self._on_strike_mode_changed)
        self.parameter_panel.strike_execute_requested.connect(self._on_strike_execute)
        self.parameter_panel.strike_clear_requested.connect(self._on_strike_clear)
        self.parameter_panel.strike_export_requested.connect(self._on_strike_export)
        self.parameter_panel.strike_dtot_export_requested.connect(self._on_strike_dtot_export)
        self.parameter_panel.strike_owa_parm_requested.connect(self._on_strike_owa_parm)
        self.parameter_panel.strike_sitl_upload_requested.connect(self._on_strike_sitl_upload)
        self.parameter_panel.strike_recon_trigger_requested.connect(self._on_strike_recon_trigger)
        self.parameter_panel.strike_vtol_toggle_changed.connect(self._on_strike_vtol_toggled)
        self.parameter_panel.strike_open_vtol_mission_planner_requested.connect(
            self.on_open_swarm_strike_panel
        )
        # 目標座標手動輸入（lat/lon spinbox） → 同步 _strike_targets + 3D 標記
        self.parameter_panel.strike_target_coord_changed.connect(
            self._on_strike_target_coord_input
        )
        # 切換到蜂群打擊分頁 → 自動啟用目標標記模式（離開時關閉）
        self.parameter_panel.tab_changed.connect(self._on_param_tab_changed)

        # ── WSL ROS2 STOT 全流程（第二條、平行的打擊路徑）────────────
        # 目標座標一律取自 self._strike_targets[0]（地圖點擊 / ⓿ 區 spinbox
        # 兩條輸入都收斂到那裡），這邊不另接 strike_target_added，
        # 否則一次點擊會新增兩個目標。
        self.parameter_panel.stot_ros2_run_requested.connect(self._on_stot_ros2_run)
        self.parameter_panel.stot_ros2_abort_requested.connect(self._on_stot_ros2_abort)

        # 蜂群打擊內部狀態
        self._strike_marking_mode = False
        self._strike_base_marking_mode = False         # STOT 基地標記模式
        self._strike_launch_base = None                # (lat, lon) 或 None (STOT 模式才用)
        self._strike_targets: list = []                # [(lat, lon), ...]
        # 菱形編隊 STOT 飽和打擊（單目標、4 機）規劃結果快取
        self._diamond_plans = None                     # list[UAVStrikePlan] 或 None
        self._diamond_target = None
        self._diamond_cfg = None
        # 快取最近一次規劃結果，供匯出使用
        self._strike_result = None   # {'trajectories': [...], 'targets': [...], 'params': {...}, 'mode': 'DTOT'/'STOT', 'is_vtol': bool}
        # VTOL 模式專屬快取
        self._vtol_strike_planner = None   # VTOLSwarmStrikePlanner 實例
        self._vtol_strike_plans = None     # VTOLPlan list
        # ReconToStrikeManager 快取
        self._recon_strike_manager = None
        self._recon_strike_report = None

        return panel_widget
    
    def _add_action(self, toolbar, text: str, tip: str, slot):
        """(1-7) 建立並加入一個工具列 QAction，收斂重複樣板。

        等同 QAction(text,self)+setStatusTip(tip)+triggered.connect(slot)+addAction。
        回傳該 QAction（呼叫端需要時可保留）。
        """
        action = QAction(text, self)
        action.setStatusTip(tip)
        action.triggered.connect(slot)
        toolbar.addAction(action)
        return action

    def create_toolbar(self):
        """創建工具列"""
        toolbar = QToolBar("主工具列")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self._add_action(toolbar, "🆕 新建", "創建新任務", self.on_new_mission)
        self._add_action(toolbar, "📂 開啟", "開啟現有任務", self.on_open_mission)
        self._add_action(toolbar, "💾 儲存", "儲存當前任務", self.on_save_mission)
        toolbar.addSeparator()
        self._add_action(toolbar, "👁 預覽", "預覽飛行路徑", self.on_preview_paths)
        self._add_action(toolbar, "📤 匯出", "匯出航點檔案", self.on_export_waypoints)
        toolbar.addSeparator()
        self._add_action(
            toolbar, "🎯 蜂群打擊",
            "開啟 VTOL 蜂群打擊任務規劃面板（起飛 → 巡航 → 2km 決斷圈 → ROE）",
            self.on_open_swarm_strike_panel,
        )
        toolbar.addSeparator()
        self._add_action(toolbar, "🗑 清除", "清除所有標記和路徑", self.on_clear_all)
    
    def create_statusbar(self):
        """創建狀態列"""
        statusbar = QStatusBar()
        self.setStatusBar(statusbar)
        
        # 添加永久顯示的資訊
        self.coord_label = QLabel("座標: --")
        statusbar.addPermanentWidget(self.coord_label)
        
        self.waypoint_label = QLabel("航點: 0")
        statusbar.addPermanentWidget(self.waypoint_label)
        
        self.distance_label = QLabel("距離: 0.0m")
        statusbar.addPermanentWidget(self.distance_label)
    
    def create_menus(self):
        """創建選單列（PyQt6 兼容版本）"""
        menubar = self.menuBar()
        
        # === 檔案選單 ===
        file_menu = menubar.addMenu("檔案(&F)")
        
        action = file_menu.addAction("新建任務")
        action.setShortcut(QKeySequence("Ctrl+N"))
        action.triggered.connect(self.on_new_mission)
        
        action = file_menu.addAction("開啟任務")
        action.setShortcut(QKeySequence("Ctrl+O"))
        action.triggered.connect(self.on_open_mission)
        
        action = file_menu.addAction("儲存任務")
        action.setShortcut(QKeySequence("Ctrl+S"))
        action.triggered.connect(self.on_save_mission)

        file_menu.addSeparator()

        # 專案檔 (.aeroplan) — 包含目標標記、邊界、STOT 基地等 UI 狀態
        action = file_menu.addAction("儲存專案 (.aeroplan)")
        action.setShortcut(QKeySequence("Ctrl+Shift+S"))
        action.setStatusTip("儲存當前所有 UI 狀態 (邊界 + 打擊目標 + 參數)")
        action.triggered.connect(self.on_save_project)

        action = file_menu.addAction("讀取專案 (.aeroplan)")
        action.setShortcut(QKeySequence("Ctrl+Shift+O"))
        action.setStatusTip("載入先前儲存的專案檔")
        action.triggered.connect(self.on_load_project)

        file_menu.addSeparator()
        
        action = file_menu.addAction("匯出航點")
        action.setShortcut(QKeySequence("Ctrl+E"))
        action.triggered.connect(self.on_export_waypoints)
        
        file_menu.addSeparator()
        
        action = file_menu.addAction("退出")
        action.setShortcut(QKeySequence("Ctrl+Q"))
        action.triggered.connect(self.close)
        
        # === 編輯選單 ===
        edit_menu = menubar.addMenu("編輯(&E)")
        
        action = edit_menu.addAction("清除路徑")
        action.triggered.connect(self.on_clear_paths)
        
        action = edit_menu.addAction("清除邊界")
        action.triggered.connect(self.on_clear_corners)
        
        action = edit_menu.addAction("清除全部")
        action.setShortcut(QKeySequence("Ctrl+R"))
        action.triggered.connect(self.on_clear_all)
        
        # === 檢視選單 ===
        view_menu = menubar.addMenu("檢視(&V)")

        action = view_menu.addAction("重置視圖")
        action.triggered.connect(self.on_reset_view)

        action = view_menu.addAction("顯示網格")
        action.triggered.connect(self.on_toggle_grid)

        view_menu.addSeparator()

        # 重置版面 (所有 Splitter 恢復預設比例)
        action = view_menu.addAction("重置版面比例")
        action.setShortcut(QKeySequence("Ctrl+Shift+R"))
        action.setStatusTip("將所有可拖拉分隔線恢復為預設比例")
        action.triggered.connect(self.on_reset_layout)

        # 切換視窗自適應尺寸
        action = view_menu.addAction("自適應視窗尺寸 (85%)")
        action.triggered.connect(self._apply_responsive_geometry)
        
        # === 工具選單 ===
        tools_menu = menubar.addMenu("工具(&T)")

        action = tools_menu.addAction("🗺️ 多邊形編輯器")
        action.setShortcut(QKeySequence("Ctrl+M"))
        action.triggered.connect(self.on_open_polygon_editor)

        action = tools_menu.addAction("🖱️ 點擊地圖視窗 (Tkinter)")
        action.triggered.connect(self.open_click_map_window)

        tools_menu.addSeparator()

        action = tools_menu.addAction("相機配置")
        action.triggered.connect(self.on_camera_config)

        action = tools_menu.addAction("飛行器配置")
        action.triggered.connect(self.on_vehicle_config)

        tools_menu.addSeparator()

        action = tools_menu.addAction("障礙物管理")
        action.triggered.connect(self.on_obstacle_manager)
        
        # === 說明選單 ===
        help_menu = menubar.addMenu("說明(&H)")
        
        action = help_menu.addAction("使用說明")
        action.triggered.connect(self.on_show_help)
        
        action = help_menu.addAction("關於")
        action.triggered.connect(self.on_about)
    
    def _on_enter_key(self):
        """Enter 鍵依當前分頁決定行為：
        - 基本演算法 (tab 0)：生成路徑
        - DCCPP (tab 2)：觸發 DCCPP 最佳化
        - 其他分頁（SITL 等）：不做任何動作（僅用於數字輸入）
        """
        tab_index = self.parameter_panel._tabs.currentIndex()
        if tab_index == 0:
            self.on_preview_paths()
        elif tab_index == 2:
            self.parameter_panel._on_dccpp_btn_clicked()
        # 其他分頁不執行任何動作

    def setup_shortcuts(self):
        """設置快捷鍵"""
        # Enter 鍵 - 依當前分頁決定行為
        enter_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Return), self)
        enter_shortcut.activated.connect(self._on_enter_key)

        # 也支援小鍵盤的 Enter
        enter_shortcut2 = QShortcut(QKeySequence(Qt.Key.Key_Enter), self)
        enter_shortcut2.activated.connect(self._on_enter_key)

        # Space 鍵 - 也可以生成路徑（備選）
        space_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        space_shortcut.activated.connect(self.on_preview_paths)

        # Escape 鍵 - 清除路徑
        esc_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        esc_shortcut.activated.connect(self.on_clear_paths)

        # Delete 鍵 - 刪除最後一個角點
        del_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Delete), self)
        del_shortcut.activated.connect(self.on_delete_last_corner)

        # Backspace 鍵 - 也可以刪除最後一個角點
        backspace_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Backspace), self)
        backspace_shortcut.activated.connect(self.on_delete_last_corner)

        logger.info("快捷鍵設置完成: Enter=依分頁動作, Delete=刪除角點, Esc=清除路徑")

    def _update_transit_paths(self, planner, sub_paths):
        """計算並顯示轉場路徑（若啟用高度錯層）"""
        if not self.flight_params.get('transit_stagger_enabled', False):
            self.map_widget.clear_transit_paths()
            return
        if not self.corners or not sub_paths:
            return
        # 以多邊形質心作為預設起飛點
        home_lat = sum(c[0] for c in self.corners) / len(self.corners)
        home_lon = sum(c[1] for c in self.corners) / len(self.corners)
        home = (home_lat, home_lon)

        stagger_alt = self.flight_params.get('transit_stagger_alt', 10.0)
        base_alt = self.flight_params.get('altitude', 10.0)
        transit_paths = planner.generate_transit_paths(
            home=home,
            sub_paths=sub_paths,
            base_alt=base_alt,
            stagger_alt=stagger_alt,
        )
        self.map_widget.display_transit_paths(transit_paths)
        logger.info(f"轉場路徑已更新: {len(transit_paths)} 條，高度錯層 {stagger_alt}m")

    def on_pick_circle_center(self):
        """進入圓心點選模式：下次地圖點擊將設定為圓心"""
        self.picking_circle_center = True
        self.statusBar().showMessage("點擊地圖上任意位置以設定螺旋/圓心掃描的圓心...", 0)

    def on_drag_circle_mode(self):
        """進入地圖拖曳定義圓形模式（Mission Planner 風格）"""
        self.picking_circle_center = True   # 確保點擊攔截
        self.map_widget.set_circle_draw_mode(True)
        self.statusBar().showMessage(
            "按住滑鼠左鍵並拖曳以定義圓形掃描區域（放開滑鼠確認）...", 0
        )
        logger.info("進入地圖拖曳定義圓形模式")

    def on_circle_drag_defined(self, lat: float, lon: float, radius_m: float):
        """接收地圖拖曳圓形結果（同時設定圓心 + 半徑）"""
        self.circle_center = (lat, lon)
        self.picking_circle_center = False
        # 若拖曳半徑有效，更新 flight_params
        if radius_m > 1.0:
            self.flight_params['circle_max_radius'] = radius_m
        # 更新參數面板（顯示圓心 + 半徑，並更新 spinbox）
        self.parameter_panel.set_circle_center_display(lat, lon, radius_m)
        # 地圖圓形已由 map_widget.on_circle_draw_complete 渲染，不重複渲染
        r_disp = radius_m if radius_m > 1.0 else self.flight_params.get('circle_max_radius', 200.0)
        self.statusBar().showMessage(
            f"圓形已定義：圓心 ({lat:.6f}, {lon:.6f})，半徑 {r_disp:.0f} m", 5000
        )
        logger.info(f"拖曳圓形定義：圓心={lat:.6f},{lon:.6f}, r={radius_m:.1f}m")

    def on_circle_center_set(self, lat: float, lon: float):
        """設定圓心座標並更新 UI（僅點選模式）"""
        self.circle_center = (lat, lon)
        self.picking_circle_center = False
        # 更新參數面板顯示
        self.parameter_panel.set_circle_center_display(lat, lon)
        # 在地圖上繪製掃描範圍圓形
        radius = self.flight_params.get('circle_max_radius', 200.0)
        if hasattr(self.map_widget, 'draw_circle_overlay'):
            self.map_widget.draw_circle_overlay(lat, lon, radius)
        self.statusBar().showMessage(f"圓心已設定: ({lat:.6f}, {lon:.6f})，半徑 {radius:.0f}m", 5000)
        logger.info(f"圓心掃描中心點設定: ({lat:.6f}, {lon:.6f})")

    def on_pick_home_point(self):
        """進入起飛點點選模式：下次地圖點擊將設定為起飛點"""
        self.picking_home_point = True
        self.statusBar().showMessage("點擊地圖上任意位置以設定起飛點（跑道位置）...", 0)
        logger.info("進入起飛點選模式")

    def on_home_point_set(self, lat: float, lon: float):
        """設定起飛點並更新 UI"""
        self.home_point = (lat, lon)
        self.picking_home_point = False
        self.parameter_panel.set_home_point_display(lat, lon)
        self.map_widget.set_home_point_overlay(lat, lon)
        self.statusBar().showMessage(f"起飛點已設定: ({lat:.6f}, {lon:.6f})", 5000)
        logger.info(f"起飛點設定: ({lat:.6f}, {lon:.6f})")

    def on_clear_home_point(self):
        """清除起飛點"""
        self.home_point = None
        self.picking_home_point = False
        self.parameter_panel.clear_home_point_display()
        if hasattr(self.map_widget, 'clear_home_point_overlay'):
            self.map_widget.clear_home_point_overlay()
        self.statusBar().showMessage("起飛點已清除", 3000)
        logger.info("起飛點已清除")

    # ═════════════════════════════════════════════════════════════
    #  蜂群打擊面板（VTOL 全任務生命週期）— 浮動對話框整合
    # ═════════════════════════════════════════════════════════════
    def on_open_swarm_strike_panel(self):
        """開啟戰術蜂群打擊面板（單例浮動對話框）。

        - 首次開啟建立對話框並接線
        - 預填座標：起飛點用已設定的 home_point，目標用第一個邊界角點（若存在）
        - 非模態 .show()：視窗保持浮在主視窗上方但不阻擋地圖互動
        """
        if self._swarm_strike_dialog is None:
            dlg = TacticalSwarmStrikeDialog(self)
            panel = dlg.panel
            # ── 信號接線 ──
            panel.pick_launch_requested.connect(self.on_swarm_pick_launch_request)
            panel.pick_target_requested.connect(self.on_swarm_pick_target_request)
            panel.mission_planned.connect(self.on_swarm_mission_planned)
            panel.mission_exported.connect(self.on_swarm_mission_exported)
            panel.mission_deployed.connect(self.on_swarm_mission_deployed)
            self._swarm_strike_dialog = dlg

            # ── 預填座標（若主視窗已有對應狀態） ──
            if self.home_point is not None:
                panel.set_launch_point(*self.home_point)
            if self.corners:
                panel.set_target_point(*self.corners[0])

            logger.info("戰術蜂群打擊面板：已建立")

        self._swarm_strike_dialog.show()
        self._swarm_strike_dialog.raise_()
        self._swarm_strike_dialog.activateWindow()

    def on_swarm_pick_launch_request(self):
        """面板 LAUNCH PICK 按鈕 → 進入地圖拾取模式，暫時隱藏面板。"""
        self.picking_swarm_launch = True
        self.picking_swarm_target = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.hide()
        self.statusBar().showMessage(
            "⚡ SWARM STRIKE：點擊地圖以設定起飛點（LAUNCH POINT）...", 0
        )
        logger.info("進入蜂群起飛點拾取模式")

    def on_swarm_pick_target_request(self):
        """面板 TARGET PICK 按鈕 → 進入地圖拾取模式。"""
        self.picking_swarm_target = True
        self.picking_swarm_launch = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.hide()
        self.statusBar().showMessage(
            "⚡ SWARM STRIKE：點擊地圖以設定打擊目標（TARGET）...", 0
        )
        logger.info("進入蜂群目標拾取模式")

    def on_swarm_launch_picked(self, lat: float, lon: float):
        """使用者在地圖上點擊後，將座標回填至面板並重新顯示對話框。"""
        self.picking_swarm_launch = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.panel.set_launch_point(lat, lon)
            self._swarm_strike_dialog.show()
            self._swarm_strike_dialog.raise_()
            self._swarm_strike_dialog.activateWindow()
        self.statusBar().showMessage(
            f"⚡ 蜂群起飛點已設定: ({lat:.6f}, {lon:.6f})", 4000
        )
        logger.info(f"蜂群起飛點設定: ({lat:.6f}, {lon:.6f})")

    def on_swarm_target_picked(self, lat: float, lon: float):
        """同上，目標版本。"""
        self.picking_swarm_target = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.panel.set_target_point(lat, lon)
            self._swarm_strike_dialog.show()
            self._swarm_strike_dialog.raise_()
            self._swarm_strike_dialog.activateWindow()
        self.statusBar().showMessage(
            f"⚡ 蜂群打擊目標已設定: ({lat:.6f}, {lon:.6f})", 4000
        )
        logger.info(f"蜂群打擊目標設定: ({lat:.6f}, {lon:.6f})")

    def on_swarm_mission_planned(self, params, waypoints):
        """蜂群任務已規劃完成（已顯示於面板預覽區）。"""
        logger.info(
            f"蜂群任務規劃完成 — ROE={params.roe_mode.value} WP={len(waypoints)}"
        )

    def on_swarm_mission_exported(self, file_path: str):
        """蜂群任務已匯出 .waypoints 檔案。"""
        self.statusBar().showMessage(f"⚡ 蜂群任務已匯出: {file_path}", 5000)
        logger.info(f"蜂群任務匯出: {file_path}")

    def on_swarm_mission_deployed(self, waypoints):
        """面板 DEPLOY 按鈕 — 此處僅 log 與狀態提示；實際 MAVLink 上傳由連線層處理。

        若後續要串接 SITL/實機上傳，請在此：
            1. 取得當前 MAVLink 連線 (self.sitl_connection 或等效)
            2. 呼叫 mission upload (mission_clear_all + mission_item_int × N)
            3. 確認 MISSION_ACK 後發 status 回饋
        """
        logger.info(f"[SwarmStrike] DEPLOY 請求 — {len(waypoints)} 個航點")
        self.statusBar().showMessage(
            f"⚡ SWARM STRIKE DEPLOYED — {len(waypoints)} WP（MAVLink 上傳待接線）",
            8000,
        )

    @staticmethod
    def _haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """兩點間大地距離（公尺）"""
        R = EARTH_RADIUS_M
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return R * 2 * math.asin(math.sqrt(a))

    # ══════════════════════════════════════════════════════════════
    #  模式二：多個作業區
    # ══════════════════════════════════════════════════════════════
    def survey_area_polygons(self) -> list:
        """要送去規劃的所有作業區。

        = 已完成的區 + （正在畫的那一塊，如果它已經有 3 個角點）。
        兩者都沒有就回空清單 —— 呼叫端本來就會先擋。
        """
        out = [list(p) for p in getattr(self, 'survey_areas', []) if len(p) >= 3]
        cur = list(getattr(self, 'corners', []) or [])
        if len(cur) >= 3:
            out.append(cur)
        return out

    def commit_current_area(self) -> bool:
        """把正在畫的那一塊收進清單，然後清空畫布準備畫下一塊。"""
        cur = list(self.corners or [])
        if len(cur) < MIN_CORNERS:
            QMessageBox.warning(
                self, '區域未完成',
                f'目前只有 {len(cur)} 個角點，至少要 {MIN_CORNERS} 個才能構成一塊區。')
            return False
        self.survey_areas.append(cur)
        self.corners.clear()
        self.map_widget.clear_corners()
        self._sync_area_counts()
        self._render_committed_areas()
        logger.info(f'作業區 #{len(self.survey_areas)} 完成（{len(cur)} 個角點）；'
                    f'畫布已清空，可以畫下一塊')
        self.statusBar().showMessage(
            f'已完成第 {len(self.survey_areas)} 塊作業區，繼續圈下一塊或直接規劃', 8000)
        return True

    def clear_all_survey_areas(self):
        """清掉所有已完成的區（不動正在畫的那一塊 —— 那個由既有的清除鈕負責）。"""
        n = len(self.survey_areas)
        self.survey_areas.clear()
        self._sync_area_counts()
        self._render_committed_areas()
        logger.info(f'已清除 {n} 塊已完成的作業區')
        self.statusBar().showMessage(f'已清除 {n} 塊作業區', 5000)

    def _sync_area_counts(self):
        """把「已完成幾塊 / 正在畫幾點」同步到面板。"""
        for name in ('vtol_recon_panel', 'parameter_panel'):
            panel = getattr(self, name, None)
            fn = getattr(panel, 'set_area_count', None)
            if callable(fn):
                try:
                    fn(len(self.survey_areas), len(self.corners or []))
                except Exception as e:      # noqa: BLE001
                    logger.warning(f'{name}.set_area_count 失敗：{e}')

    def _render_committed_areas(self):
        """在地圖上畫出已完成的區。

        ⚠ 地圖端沒有這個能力時只記一行 log，**不要**讓規劃因此失敗 ——
        區域清單是資料，畫不畫得出來是顯示問題，兩者不該綁在一起。
        """
        fn = getattr(self.map_widget, 'display_survey_areas', None)
        if callable(fn):
            try:
                fn([list(p) for p in self.survey_areas])
                return
            except Exception as e:          # noqa: BLE001
                logger.warning(f'display_survey_areas 失敗：{e}')
        elif self.survey_areas:
            logger.info(f'地圖沒有 display_survey_areas，'
                        f'{len(self.survey_areas)} 塊已完成的區不會被畫出來'
                        f'（不影響規劃）')

    def on_delete_last_corner(self):
        """刪除最後一個角點（誤點救援）—— 2D 與 3D 同時生效。

        ⚠ 舊版只 pop 了 ``self.corners`` 與 2D 的 corners/markers，再呼叫
        ``map_widget._render_map()``：
          * 3D（Cesium）的 ``corners`` 與標記**完全沒被動到** ⇒ 切到 3D 那顆
            被刪的點還在，而且兩邊索引從此漂移，再刪一次就刪錯點；
          * ``DualMapWidget`` 當時**沒有 markers 代理**，`self.map_widget.markers`
            會 AttributeError，整個刪除流程在那一行就中斷。
        改成呼叫 `DualMapWidget.remove_last_corner()`，2D+3D 一起刪。
        """
        if not self.corners:
            return
        removed = self.corners.pop()
        # 地圖端（2D + 3D）對稱刪除
        remover = getattr(self.map_widget, 'remove_last_corner', None)
        if callable(remover):
            remover()
        else:                                   # 極舊版 widget 的保底
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            self.map_widget._render_map()
        # 更新 UI（各分頁的角點計數：基本演算法 + VTOL 偵察都靠這個）
        self.parameter_panel.update_corner_count(len(self.corners))
        self._sync_area_counts()
        self.update_statusbar()
        logger.info(f"刪除角點: ({removed[0]:.6f}, {removed[1]:.6f}), "
                    f"剩餘 {len(self.corners)} 個（2D+3D 同步）")

    def _schedule_path_generation(self):
        """排程延遲路徑生成（防止頻繁更新）"""
        if self.path_generation_timer:
            self.path_generation_timer.stop()

        self.path_generation_timer = QTimer()
        self.path_generation_timer.setSingleShot(True)
        self.path_generation_timer.timeout.connect(self._auto_generate_path)
        self.path_generation_timer.start(self.path_generation_delay)

    def _auto_generate_path(self):
        """自動生成路徑（靜默模式，不顯示對話框）"""
        algorithm = getattr(self, 'current_algorithm', 'grid')
        _circle_mode = algorithm in ('spiral', 'circular') and self.circle_center is not None
        if not _circle_mode and len(self.corners) < MIN_CORNERS:
            return

        try:
            # 使用 CoveragePlanner 生成路徑
            planner = CoveragePlanner()

            # 獲取當前選擇的演算法模式
            algorithm = getattr(self, 'current_algorithm', 'grid')
            if algorithm == 'grid':
                pattern = ScanPattern.GRID
            elif algorithm == 'spiral':
                pattern = ScanPattern.SPIRAL
            elif algorithm == 'circular':
                pattern = ScanPattern.CIRCULAR
            else:
                pattern = ScanPattern.GRID

            # 判斷是否為固定翼
            is_fixed_wing = self.current_vehicle_type == '固定翼'

            # 建立覆蓋參數
            params = CoverageParameters(
                spacing=self.flight_params['spacing'],
                angle=self.flight_params['angle'],
                pattern=pattern,
                is_fixed_wing=is_fixed_wing,
                turn_radius=self.flight_params.get('turn_radius', 50.0),
                smooth_turns=is_fixed_wing,
                circle_base_altitude=self.flight_params['altitude'],
                grid_alt_step=self.flight_params.get('grid_alt_step', 0.0),
                grid_base_altitude=self.flight_params['altitude'],
            )

            # 生成覆蓋路徑（支援分區）
            n_regions = self.flight_params.get('subdivisions', 1)
            spacing_m = self.flight_params.get('region_spacing', 0.0)
            gap_spacings_m = self.flight_params.get('gap_spacings_m') or None
            v_spacing_m = self.flight_params.get('v_spacing_m', 0.0) or None

            # 圍籬要對齊掃描角。只有 GRID 會照 params.angle 排掃描線；
            # SPIRAL / CIRCULAR 完全不看它，那時要傳 None 讓圍籬維持軸對齊。
            fence_angle = (coverage_angle_to_compass(params.angle)
                           if pattern == ScanPattern.GRID else None)

            if n_regions > 1:
                sub_paths = planner.plan_subdivided_coverage(
                    self.corners, params,
                    n_regions=n_regions,
                    spacing_m=spacing_m,
                    gap_spacings_m=gap_spacings_m,
                    v_spacing_m=v_spacing_m,
                )
                self.sub_paths = sub_paths  # 保存各子區域路徑供匯出使用
                path = [pt for sp in sub_paths for pt in sp]
                if not path:
                    return
                self.map_widget.display_paths(sub_paths, self.flight_params['altitude'],
                                              scan_angle_deg=fence_angle)
                # 顯示轉場路徑（如啟用高度錯層）
                self._update_transit_paths(planner, sub_paths)
            else:
                self.sub_paths = []
                path = planner.plan_coverage(self.corners, params)
                if not path:
                    return
                self.map_widget.display_path(path, self.flight_params['altitude'],
                                             scan_angle_deg=fence_angle)
                self.map_widget.clear_transit_paths()

            # 計算總飛行距離
            total_distance = 0.0
            for i in range(len(path) - 1):
                lat1, lon1 = path[i]
                lat2, lon2 = path[i + 1]
                dlat = (lat2 - lat1) * 111111.0
                dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
                total_distance += math.sqrt(dlat**2 + dlon**2)

            # 儲存航點
            self.waypoints = path

            # 更新狀態列
            self.waypoint_label.setText(f"航點: {len(path)}")
            self.distance_label.setText(f"距離: {total_distance:.0f}m")

            # 啟用匯出按鈕並更新任務統計
            self.mission_panel.update_mission_stats({
                'waypoint_count': len(path),
                'total_distance': total_distance,
                'estimated_time': total_distance / max(self.flight_params['speed'], 0.1),
                'area': 0,
                'regions': self.flight_params.get('subdivisions', 1),
            })

            self.statusBar().showMessage(f"即時生成: {len(path)} 個航點, {total_distance:.0f}m", 2000)
            logger.info(f"即時路徑生成: {len(path)} 個航點")

            # ── 自動同步到 Mission Planner ──────────────────────────
            self._sync_to_mission_planner(path)

        except Exception as e:
            logger.error(f"即時路徑生成失敗: {e}")

    def _sync_to_mission_planner(self, path):
        """將航點寫入同步檔，供 Mission Planner 的 FileSystemWatcher 偵測"""
        import os
        SYNC_DIR  = r"C:\Users\joy46\Documents\mp_sync"
        SYNC_FILE = os.path.join(SYNC_DIR, "aeroplan_sync.waypoints")
        try:
            os.makedirs(SYNC_DIR, exist_ok=True)
            alt = self.flight_params.get('altitude', 50)
            lines = ["QGC WPL 110"]
            # Home point
            lines.append("0\t1\t0\t16\t0\t0\t0\t0\t0\t0\t0\t1")
            for i, wp in enumerate(path, start=1):
                lat = wp[0] if len(wp) > 0 else 0
                lon = wp[1] if len(wp) > 1 else 0
                a   = wp[2] if len(wp) > 2 else alt
                lines.append(
                    f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{a:.2f}\t1"
                )
            with open(SYNC_FILE, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
            logger.info(f"[MP Sync] 已同步 {len(path)} 個航點到 {SYNC_FILE}")
        except Exception as e:
            logger.warning(f"[MP Sync] 同步失敗: {e}")

    def load_stylesheet(self):
        """載入戰術 HUD 主題（Tactical HUD Theme）

        標準啟動路徑中 main.py 已於建立 QApplication 時呼叫 apply_tactical_theme。
        此方法重複套用無副作用 (app.setStyleSheet 冪等)，用途：
          * 保留 __init__ 既有呼叫鏈的相容性
          * 支援直接實例化 MainWindow 的測試 / 除錯情境
        """
        try:
            from ui.resources.tactical_theme import apply_tactical_theme
            app = QApplication.instance()
            if app is not None:
                apply_tactical_theme(app)
                logger.info("戰術主題 (Tactical HUD) 已套用")
            else:
                logger.warning("QApplication 尚未建立，跳過主題套用")
        except Exception as e:
            logger.error(f"載入戰術主題失敗: {e}")
    
    # ==========================================
    # 信號處理函數
    # ==========================================
    
    def on_corner_added(self, lat, lon):
        """處理新增邊界點（從地圖點擊）"""
        # GUIDED 點圖起飛模式：最高優先攔截 → 整群 GUIDED 起飛（2D/3D 地圖皆可）
        if getattr(self, '_guided_takeoff_mode', False):
            try:
                if self.map_widget.corners:
                    self.map_widget.corners.pop()
                if getattr(self.map_widget, 'markers', None):
                    self.map_widget.markers.pop()
                self.map_widget._render_map()
            except Exception:
                pass
            self._send_guided_takeoff_swarm(lat, lon)
            return

        # GUIDED 點圖飛到模式：次優先攔截 → 長機 GUIDED 飛往該點（僚機由蜂群跟）
        if getattr(self, '_guided_goto_mode', False):
            try:
                if self.map_widget.corners:
                    self.map_widget.corners.pop()
                if getattr(self.map_widget, 'markers', None):
                    self.map_widget.markers.pop()
                self.map_widget._render_map()
            except Exception:
                pass
            self._send_guided_goto(lat, lon)
            return

        # 起飛點點選模式：優先攔截（在圓心模式之前）
        if self.picking_home_point:
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_home_point_set(lat, lon)
            return

        # 圓心點選模式：攔截點擊作為掃描圓心
        if self.picking_circle_center:
            # 撤銷 map_widget 已自動加入的角點
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_circle_center_set(lat, lon)
            return

        # 蜂群打擊：起飛點拾取模式
        if self.picking_swarm_launch:
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_swarm_launch_picked(lat, lon)
            return

        # 蜂群打擊：目標拾取模式
        if self.picking_swarm_target:
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_swarm_target_picked(lat, lon)
            return

        # 檢查是否超過最大數量
        if len(self.corners) >= MAX_CORNERS:
            QMessageBox.warning(
                self, "已達上限",
                f"已達到最大邊界點數量 ({MAX_CORNERS} 個)！"
            )
            return

        self.corners.append((lat, lon))
        remaining = MAX_CORNERS - len(self.corners)
        logger.info(f"新增邊界點 #{len(self.corners)}: ({lat:.6f}, {lon:.6f}) [剩餘: {remaining}]")
        self.parameter_panel.update_corner_count(len(self.corners))
        self._sync_area_counts()
        self.update_statusbar()

    def on_manual_corner_added(self, lat, lon):
        """處理手動新增邊界點（從參數面板）"""
        # 檢查是否超過最大數量
        if len(self.corners) >= MAX_CORNERS:
            QMessageBox.warning(
                self, "已達上限",
                f"已達到最大邊界點數量 ({MAX_CORNERS} 個)！"
            )
            return

        self.corners.append((lat, lon))
        # 在地圖上添加標記
        self.map_widget.add_corner(lat, lon)
        remaining = MAX_CORNERS - len(self.corners)
        logger.info(f"手動新增邊界點 #{len(self.corners)}: ({lat:.6f}, {lon:.6f}) [剩餘: {remaining}]")
        self.parameter_panel.update_corner_count(len(self.corners))
        self._sync_area_counts()
        self.update_statusbar()

    def on_corner_moved(self, index, lat, lon):
        """處理移動邊界點"""
        if 0 <= index < len(self.corners):
            self.corners[index] = (lat, lon)
            logger.info(f"移動邊界點 #{index+1}: ({lat:.6f}, {lon:.6f})")
            self.update_statusbar()
    
    def on_parameters_changed(self, params):
        """處理參數變更"""
        # 處理演算法變更
        if 'algorithm' in params:
            self.current_algorithm = params['algorithm']
            logger.info(f"演算法變更: {self.current_algorithm}")

        # 處理載具變更
        if 'vehicle_type' in params:
            if params['vehicle_type'] != self.current_vehicle_type:
                # 換了載具，之前算的 DCCPP 結果（含每架起飛點、載具類型）就不再適用。
                # 不清掉的話它會一路活到 process 結束，繼續影響 SITL 啟動的
                # 載具比對、生成座標與台數。
                if getattr(self, '_dccpp_result', None) is not None:
                    logger.info(
                        f"[DCCPP] 載具由 {self.current_vehicle_type} 改為 "
                        f"{params['vehicle_type']}，清除既有 DCCPP 結果快取"
                    )
                    self._dccpp_result = None
            self.current_vehicle_type = params['vehicle_type']
        if 'vehicle_model' in params:
            self.current_vehicle_model = params['vehicle_model']

        # 更新飛行參數
        flight_keys = [
            'altitude', 'speed', 'angle', 'spacing', 'yaw_speed',
            'subdivisions', 'region_spacing', 'gap_spacings_m', 'v_spacing_m',
            'transit_stagger_enabled', 'transit_stagger_alt',
            'turn_radius', 'lock_heading', 'heading_angle',
            # 固定翼專用
            'fw_takeoff_bearing', 'fw_runway_length',
            'fw_landing_bearing', 'fw_pattern_alt',
            'fw_downwind_offset', 'fw_pattern_leg', 'fw_final_dist',
            'fw_scan_mode',
            # 螺旋/同心圓圓心模式
            'circle_center_lat', 'circle_center_lon',
            'circle_max_radius', 'circle_alt_step', 'circle_min_radius',
            # 螺旋專用
            'spiral_curvature', 'spiral_alt_step',
            # 固定翼 AutoLand
            'fw_autoland',
        ]
        for key in flight_keys:
            if key in params:
                self.flight_params[key] = params[key]

        # 圓心半徑變更時更新地圖圓形
        if self.circle_center and 'circle_max_radius' in params:
            c_lat, c_lon = self.circle_center
            if hasattr(self.map_widget, 'draw_circle_overlay'):
                self.map_widget.draw_circle_overlay(
                    c_lat, c_lon, self.flight_params['circle_max_radius']
                )

        logger.info(f"參數已更新: {params}")
    
    def on_preview_paths(self):
        """預覽飛行路徑"""
        algorithm = getattr(self, 'current_algorithm', 'grid')
        # 螺旋/同心圓圓心模式：不需要邊界點，只需設定圓心
        _is_circle_center_mode = (
            algorithm in ('spiral', 'circular') and self.circle_center is not None
        )
        if not _is_circle_center_mode and len(self.corners) < MIN_CORNERS:
            QMessageBox.warning(
                self, "邊界不足",
                f"需要至少 {MIN_CORNERS} 個邊界點才能生成路徑\n"
                f"（螺旋/同心圓模式請先點擊「地圖拖曳定義圓形」設定圓心）"
            )
            return

        # ── Schema 驗證飛行參數 (2026 重構：fail-fast 防無效輸入)──
        try:
            from config.schemas import FlightParameters
            fp = FlightParameters.from_dict({
                'altitude':     self.flight_params.get('altitude', 50.0),
                'speed':        self.flight_params.get('speed', 15.0),
                'angle':        self.flight_params.get('angle', 0.0),
                'spacing':      self.flight_params.get('spacing', 20.0),
                'turn_radius':  self.flight_params.get('turn_radius', 50.0),
                'vehicle_type': {
                    '多旋翼': 'multirotor', '固定翼': 'fixed_wing', 'VTOL': 'vtol',
                }.get(self.current_vehicle_type, 'multirotor'),
            })
            fp.validate()
        except ValueError as e:
            QMessageBox.warning(
                self, '飛行參數錯誤',
                f'預覽前參數驗證失敗，請修正設定:\n\n{e}'
            )
            logger.warning(f'[Preview] flight_params 驗證失敗: {e}')
            return
        except Exception as e:
            logger.debug(f'[Preview] Schema 驗證異常 (忽略): {e}')

        try:
            # 獲取當前選擇的演算法
            algorithm = getattr(self, 'current_algorithm', 'grid')

            # 判斷是否為固定翼
            is_fixed_wing = self.current_vehicle_type == '固定翼'

            path = None
            # 圍籬對齊用的掃描方位角（羅盤：0=北、順時針）。
            # ⚠️ 預設 None＝維持軸對齊。下面只有**真的照掃描角排線**的分支才寫它；
            #    圓心螺旋 / A* / RRT / DWA 都不看掃描角，留 None 才不會把圍籬
            #    轉到一個航線沒走的方向。
            fence_angle = None

            # ── 固定翼三階段路徑規劃（螺旋/同心圓/網格均啟用）──────────
            # _preview_fixed_wing_mission 內部已完整處理：
            #   地圖顯示、self.waypoints、self.fw_mission_result、統計、對話框
            # 成功後直接 return，避免下方的多旋翼後處理覆蓋地圖與航點
            if is_fixed_wing and algorithm in ['spiral', 'circular', 'grid']:
                path = self._preview_fixed_wing_mission(algorithm)
                if path is None:
                    return  # 已在內部顯示錯誤
                return  # ← 固定翼任務已完整處理，跳過後續多旋翼流程

            # 根據演算法類型生成路徑
            elif algorithm in ['grid', 'spiral', 'circular']:
                # ── 螺旋/同心圓：圓心模式 ──────────────────────────────────────
                if algorithm in ('spiral', 'circular'):
                    if self.circle_center is None:
                        QMessageBox.information(
                            self, "提示",
                            "螺旋/同心圓掃描需要先設定圓心。\n\n"
                            "方法：\n"
                            "  1. 在右側參數面板按「從地圖點選圓心」\n"
                            "  2. 在地圖上點擊想要的中心位置\n\n"
                            "現已自動進入點選圓心模式，請點擊地圖。"
                        )
                        self.on_pick_circle_center()
                        return

                    # 圓心模式：直接用圓心 + 最大半徑生成路徑
                    c_lat, c_lon = self.circle_center
                    max_r    = self.flight_params.get('circle_max_radius', 200.0)
                    alt_step = self.flight_params.get('circle_alt_step', 0.0)
                    min_r    = self.flight_params.get('circle_min_radius', 0.0)
                    spacing  = self.flight_params['spacing']
                    fw       = is_fixed_wing
                    tr       = self.flight_params.get('turn_radius', 50.0)
                    alt      = self.flight_params['altitude']

                    spiral_curvature = self.flight_params.get('spiral_curvature', 1.0)
                    spiral_alt_step  = self.flight_params.get('spiral_alt_step', 0.0)

                    circle_planner = CoveragePlanner()
                    if algorithm == 'spiral':
                        path = circle_planner.plan_spiral_from_center(
                            c_lat, c_lon, max_r, spacing,
                            is_fixed_wing=fw, turn_radius=tr,
                            curvature=spiral_curvature,
                            alt_step=spiral_alt_step,
                            base_alt=alt,
                        )
                        _spiral_alts = circle_planner.get_spiral_altitudes()
                    else:  # circular
                        path = circle_planner.plan_circle_survey_from_center(
                            c_lat, c_lon, max_r, spacing,
                            base_alt=alt, alt_step=alt_step,
                            min_radius_m=min_r,
                        )
                        _spiral_alts = []

                    # ── NFZ 繞行修正（多旋翼圓心模式）────────────────────
                    if self.nfz_zones and path:
                        path = self._apply_nfz_correction_multirotor_latlon(
                            path, c_lat, c_lon
                        )

                    if not path:
                        QMessageBox.warning(self, "路徑為空", "無法生成路徑，請檢查參數設定（半徑/間距）。")
                        return

                    _orig_n = len(path)
                    if _orig_n > _WP_LIMIT:
                        path, _spiral_alts = CoveragePlanner.limit_path_waypoints(
                            path, _WP_LIMIT, _spiral_alts if _spiral_alts else None
                        )
                        _trimmed = True
                    else:
                        _trimmed = False
                    # 附加 NFZ 繞行回程（避免匯出 bare RTL 直線飛回 home 穿越禁區）。
                    # 置於 trim 之後：回程航點不參與 spiral_alts 對位，匯出時自動以
                    # 巡航高度填充。
                    if self.nfz_zones:
                        path = self._append_nfz_safe_return(path, c_lat, c_lon)
                    self.waypoints = path
                    self.spiral_waypoint_altitudes = _spiral_alts
                    self.sub_paths = []

                    # 確保圓形資料是最新的（不觸發額外渲染）
                    self.map_widget._circle_center = (c_lat, c_lon)
                    self.map_widget._circle_radius_m = max_r
                    # 顯示路徑（同時包含圓形，只渲染一次）
                    self.map_widget.display_path(path, alt)

                    # 統計
                    total_distance = sum(
                        self._haversine(path[i], path[i+1]) for i in range(len(path)-1)
                    )
                    _wp_txt = f"航點: {len(path)}"
                    if _trimmed:
                        _wp_txt += f" (原{_orig_n},已簡化)"
                    self.waypoint_label.setText(_wp_txt)
                    self.distance_label.setText(f"距離: {total_distance:.0f}m")
                    self.mission_panel.update_mission_stats({
                        'waypoint_count': len(path),
                        'total_distance': total_distance,
                        'estimated_time': total_distance / max(self.flight_params['speed'], 0.1),
                        'area': math.pi * max_r ** 2,
                        'regions': 1,
                    })
                    self.statusBar().showMessage(f"路徑生成完成：{len(path)} 個航點", 5000)

                    # ── 任務預覽彈窗 ──────────────────────────────────────
                    _speed = max(self.flight_params.get('speed', 15.0), 0.1)
                    _est_s = total_distance / _speed
                    _mode_name = '螺旋掃描' if algorithm == 'spiral' else '同心圓掃描'
                    _wp_note = (f"（原 {_orig_n} 點，已簡化至 {len(path)}）" if _trimmed else "")
                    QMessageBox.information(
                        self, "任務預覽",
                        f"模式: {_mode_name}\n"
                        f"中心: {c_lat:.6f}°, {c_lon:.6f}°\n"
                        f"最大半徑: {max_r:.0f} m   間距: {spacing:.0f} m\n\n"
                        f"航點數: {len(path)} {_wp_note}\n"
                        f"總飛行距離: {total_distance:.0f} m\n"
                        f"預估飛行時間: {_est_s / 60:.1f} min\n"
                        f"覆蓋面積: {math.pi * max_r ** 2 / 10000:.2f} ha"
                    )
                    return  # skip the rest of the elif block
                # ── END 圓心模式 ──────────────────────────────────────────────

                # 覆蓋路徑規劃（Grid/Spiral/Circular，多邊形模式）
                planner = CoveragePlanner()
                if algorithm == 'grid':
                    pattern = ScanPattern.GRID
                elif algorithm == 'spiral':
                    pattern = ScanPattern.SPIRAL
                else:
                    pattern = ScanPattern.CIRCULAR

                params = CoverageParameters(
                    spacing=self.flight_params['spacing'],
                    angle=self.flight_params['angle'],
                    pattern=pattern,
                    is_fixed_wing=is_fixed_wing,
                    turn_radius=self.flight_params.get('turn_radius', 50.0),
                    smooth_turns=is_fixed_wing,
                    circle_base_altitude=self.flight_params['altitude'],
                )

                # 只有 GRID 照 params.angle 排掃描線；SPIRAL/CIRCULAR 不看它。
                if pattern == ScanPattern.GRID:
                    fence_angle = coverage_angle_to_compass(params.angle)

                n_regions = self.flight_params.get('subdivisions', 1)
                spacing_m = self.flight_params.get('region_spacing', 0.0)
                gap_spacings_m = self.flight_params.get('gap_spacings_m') or None
                v_spacing_m = self.flight_params.get('v_spacing_m', 0.0) or None

                if n_regions > 1:
                    # 分區覆蓋：各子區域獨立規劃，顯示不同顏色
                    sub_paths = planner.plan_subdivided_coverage(
                        self.corners, params,
                        n_regions=n_regions,
                        spacing_m=spacing_m,
                        gap_spacings_m=gap_spacings_m,
                        v_spacing_m=v_spacing_m,
                    )
                    # ── NFZ 繞行修正（各子區域）──────────────────────────
                    if self.nfz_zones and self.corners:
                        _ref_lat = sum(p[0] for p in self.corners) / len(self.corners)
                        _ref_lon = sum(p[1] for p in self.corners) / len(self.corners)
                        # 各子區域：掃描段繞行 + 附加 NFZ 安全回程（每機回各自 sub_path[0]，
                        # 匯出端逐 sub_path 加 RTL → 避免 bare RTL 穿越禁區）
                        sub_paths = [
                            self._append_nfz_safe_return(
                                self._apply_nfz_correction_multirotor_latlon(
                                    sp, _ref_lat, _ref_lon),
                                _ref_lat, _ref_lon,
                            )
                            for sp in sub_paths
                        ]
                    self.sub_paths = sub_paths  # 保存各子區域路徑供匯出使用
                    # 扁平化供匯出使用，各子區域之間不插入多餘航點
                    path = [pt for sp in sub_paths for pt in sp]
                    self.map_widget.display_paths(sub_paths, self.flight_params['altitude'],
                                                  scan_angle_deg=fence_angle)
                    # 顯示轉場路徑
                    self._update_transit_paths(planner, sub_paths)
                else:
                    self.sub_paths = []
                    path = planner.plan_coverage(self.corners, params)
                    # ── NFZ 繞行修正（單區域）────────────────────────────
                    if self.nfz_zones and self.corners:
                        _ref_lat = sum(p[0] for p in self.corners) / len(self.corners)
                        _ref_lon = sum(p[1] for p in self.corners) / len(self.corners)
                        path = self._apply_nfz_correction_multirotor_latlon(
                            path, _ref_lat, _ref_lon
                        )
                        # 附加 NFZ 安全回程（避免匯出 bare RTL 直穿禁區）
                        path = self._append_nfz_safe_return(path, _ref_lat, _ref_lon)
                    self.map_widget.clear_transit_paths()

            elif algorithm == 'astar':
                # A* 路徑規劃（點對點）
                if len(self.corners) >= 2:
                    astar_planner = AStarPlanner(
                        collision_checker=None,
                        step_size=self.flight_params['spacing'],
                        heuristic='euclidean'
                    )
                    path = astar_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        boundary=self.corners
                    )

            elif algorithm in ['rrt', 'rrt_star']:
                # RRT/RRT* 路徑規劃
                if len(self.corners) >= 2:
                    # 計算搜索區域
                    lats = [c[0] for c in self.corners]
                    lons = [c[1] for c in self.corners]
                    search_area = (min(lats), min(lons), max(lats), max(lons))

                    # 創建空的碰撞檢測器（無障礙物）
                    collision_checker = CollisionChecker()

                    if algorithm == 'rrt':
                        rrt_planner = RRTPlanner(
                            collision_checker=collision_checker,
                            step_size=0.0001,  # 經緯度單位
                            goal_sample_rate=0.1,
                            max_iter=1000
                        )
                    else:
                        rrt_planner = RRTStarPlanner(
                            collision_checker=collision_checker,
                            step_size=0.0001,
                            goal_sample_rate=0.1,
                            max_iter=1000,
                            search_radius=0.0005
                        )

                    path = rrt_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        search_area=search_area
                    )

            elif algorithm == 'dijkstra':
                # Dijkstra 使用與 A* 相同的邏輯
                if len(self.corners) >= 2:
                    astar_planner = AStarPlanner(
                        collision_checker=None,
                        step_size=self.flight_params['spacing'],
                        heuristic='euclidean',
                        heuristic_weight=0.0  # weight=0 等同於 Dijkstra
                    )
                    path = astar_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        boundary=self.corners
                    )

            elif algorithm == 'dwa':
                # DWA 需要即時規劃，這裡生成直線路徑作為全域參考
                QMessageBox.information(
                    self, "DWA 演算法",
                    "DWA (動態窗口) 是局域即時規劃演算法，\n"
                    "需要配合飛行控制器使用。\n\n"
                    "目前生成直線路徑作為參考。"
                )
                path = self.corners.copy()

            else:
                # 預設使用 Grid
                planner = CoveragePlanner()
                params = CoverageParameters(
                    spacing=self.flight_params['spacing'],
                    angle=self.flight_params['angle'],
                    pattern=ScanPattern.GRID
                )
                path = planner.plan_coverage(self.corners, params)
                fence_angle = coverage_angle_to_compass(params.angle)

            if not path:
                QMessageBox.warning(self, "路徑生成失敗", "無法生成覆蓋路徑，請檢查邊界點設定")
                return

            # 計算統計資訊（使用 CoveragePlanner 工具函數）
            coverage_planner = CoveragePlanner()
            area = coverage_planner.calculate_coverage_area(self.corners)
            mission_time = coverage_planner.estimate_mission_time(path, self.flight_params['speed'])

            # 計算總飛行距離
            total_distance = 0.0
            for i in range(len(path) - 1):
                lat1, lon1 = path[i]
                lat2, lon2 = path[i + 1]
                dlat = (lat2 - lat1) * 111111.0
                dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
                total_distance += math.sqrt(dlat**2 + dlon**2)

            # 若超出 MP 匯入上限，自動簡化
            path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "覆蓋路徑")

            # 儲存航點
            self.waypoints = path

            # 在地圖上顯示路徑
            # 多區域時 display_paths 已在規劃迴圈中呼叫，此處只處理單區域
            n_reg = self.flight_params.get('subdivisions', 1)
            if n_reg <= 1:
                # ⚠ 這一行是**四個分支共用的尾巴**（多邊形網格、A*、DWA、預設）。
                #   fence_angle 只有真正照掃描角排線的分支會設，其餘留 None。
                self.map_widget.display_path(path, self.flight_params['altitude'],
                                             scan_angle_deg=fence_angle)

            # 更新狀態列
            _wp_txt = f"航點: {len(path)}"
            if _trimmed:
                _wp_txt += f" (原{_orig_n},已簡化)"
            self.waypoint_label.setText(_wp_txt)
            self.distance_label.setText(f"距離: {total_distance:.0f}m")

            # 顯示結果
            region_info = (f"  分割區域: {n_reg} 個\n" if n_reg > 1 else "")
            _trim_note = (f" (原 {_orig_n}，已簡化至 {_WP_LIMIT} 上限)" if _trimmed else "")
            QMessageBox.information(
                self, "路徑生成完成",
                f"覆蓋路徑已生成！\n\n"
                f"邊界點: {len(self.corners)} 個\n"
                f"航點數: {len(path)} 個{_trim_note}\n"
                f"覆蓋面積: {area:.0f} m²\n"
                f"總飛行距離: {total_distance:.0f} m\n"
                f"預估飛行時間: {mission_time/60:.1f} 分鐘\n\n"
                f"參數:\n"
                f"  高度: {self.flight_params['altitude']} m\n"
                f"  速度: {self.flight_params['speed']} m/s\n"
                f"  間距: {self.flight_params['spacing']} m\n"
                f"  角度: {self.flight_params['angle']}°\n"
                f"{region_info}"
                f"  演算法: {algorithm}"
            )

            # 啟用匯出按鈕並更新任務統計
            self.mission_panel.update_mission_stats({
                'waypoint_count': len(path),
                'total_distance': total_distance,
                'estimated_time': mission_time,
                'area': area,
                'regions': self.flight_params.get('subdivisions', 1),
            })

            self.statusBar().showMessage(f"路徑生成完成：{len(path)} 個航點", 5000)
            logger.info(f"路徑生成完成：{len(path)} 個航點，距離 {total_distance:.0f}m")

        except Exception as e:
            logger.error(f"預覽失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "預覽錯誤", f"生成路徑時發生錯誤：\n{str(e)}")
    
    @staticmethod
    def _calc_bearing(lat1, lon1, lat2, lon2) -> float:
        """計算兩點間方位角（度，0=北，順時針）"""
        lat1r = math.radians(lat1)
        lat2r = math.radians(lat2)
        dlon  = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(lat2r)
        x = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    # ──────────────────────────────────────────────────────────────────────────
    # 固定翼專用：三階段任務規劃
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _trim_to_wp_limit(path, label: str = "") -> tuple:
        """
        若路徑超過 _WP_LIMIT，使用 RDP 自動簡化後回傳。
        回傳 (trimmed_path, was_trimmed, original_count)
        """
        orig_n = len(path)
        if orig_n <= _WP_LIMIT:
            return list(path), False, orig_n
        trimmed, _ = CoveragePlanner.limit_path_waypoints(path, _WP_LIMIT)
        if label:
            logger.warning(
                f"[WP 限制] {label}: {orig_n} → {len(trimmed)} 航點"
                f"（Mission Planner 上限 {_WP_LIMIT}）"
            )
        return trimmed, True, orig_n

    def _build_fw_params(self) -> FixedWingParameters:
        """從 flight_params 建立 FixedWingParameters"""
        import math
        fp       = self.flight_params
        speed    = fp.get('speed', 18.0)
        bank_deg = fp.get('fw_max_bank_deg', 45.0)
        tan_b    = math.tan(math.radians(max(bank_deg, 1.0)))
        r_min    = speed ** 2 / (9.81 * tan_b)          # R_min = V²/(g·tan φ)
        turn_radius_m = max(fp.get('turn_radius', 50.0), r_min)  # 不得低於 R_min
        return FixedWingParameters(
            cruise_speed_mps=speed,
            mission_altitude_m=fp.get('altitude', 100.0),
            turn_radius_m=turn_radius_m,
            takeoff_bearing_deg=fp.get('fw_takeoff_bearing', 0.0),
            takeoff_runway_m=fp.get('fw_runway_length', 50.0),
            takeoff_climb_alt_m=fp.get('altitude', 100.0) * 0.3,
            landing_bearing_deg=fp.get('fw_landing_bearing', 180.0),
            pattern_altitude_m=fp.get('fw_pattern_alt', 80.0),
            downwind_offset_m=fp.get('fw_downwind_offset', 200.0),
            pattern_leg_length_m=fp.get('fw_pattern_leg', 300.0),
            final_approach_dist_m=fp.get('fw_final_dist', 400.0),
            landing_rollout_m=fp.get('fw_landing_rollout', 0.0),
            scan_spacing_m=fp.get('spacing', 50.0),
            use_autoland=bool(fp.get('fw_autoland', False)),
        )

    def _preview_fixed_wing_mission(self, algorithm: str):
        """
        固定翼三階段任務規劃與地圖顯示

        顯示三種顏色路徑：
          黃色 - 起飛路徑
          藍色 - 主任務（螺旋/同心圓）
          橙色 - 五邊進場降落

        返回扁平化的路徑列表（lat, lon），供統計使用；
        出錯時返回 None。
        """
        try:
            fw_params = self._build_fw_params()
            scan_mode = algorithm  # 'spiral' | 'circular' | 'grid'
            scan_angle = self.flight_params.get('angle', 0.0)   # 網格掃描角度（度）
            # 圍籬對齊用的羅盤方位；預設 None＝維持軸對齊。
            # ⚠️⚠️ 本方法的兩條網格分支吃**同一個** scan_angle，但下游規劃器的
            #   慣例不同：多分區走 CoverageParameters（自東順時針，羅盤=角+90），
            #   單區域走 CoveragePathPlanner（自北逆時針，羅盤=−角）。
            #   scan_angle=30 時兩者實際飛的方向差 30°。這是既有的規劃器歧異；
            #   這裡各自用對應的轉換，讓圍籬跟著**真正飛的那個方向**。
            fw_fence_angle = None

            # 決定中心點與任務多邊形：優先使用圓心模式，其次使用邊界點
            # 網格掃描必須使用邊界多邊形，圓心模式下跳過圓形多邊形邏輯
            if scan_mode == 'grid':
                # 網格掃描需要實際多邊形邊界
                if len(self.corners) < 3:
                    QMessageBox.warning(
                        self, "缺少區域定義",
                        "固定翼網格掃描需要在地圖上點選至少 3 個邊界點。"
                    )
                    return None
                center_lat = sum(c[0] for c in self.corners) / len(self.corners)
                center_lon = sum(c[1] for c in self.corners) / len(self.corners)
                mission_polygon = self.corners
            elif self.circle_center is not None:
                center_lat, center_lon = self.circle_center
                max_r = self.flight_params.get('circle_max_radius', 200.0)
                # 以 16 邊形近似圓形多邊形，供 plan_coverage 計算半徑用
                mission_polygon = []
                for i in range(16):
                    angle = math.radians(i * 22.5)
                    dlat = max_r * math.cos(angle) / 111111.0
                    dlon = max_r * math.sin(angle) / (
                        111111.0 * math.cos(math.radians(center_lat)))
                    mission_polygon.append(
                        (center_lat + dlat, center_lon + dlon))
            elif len(self.corners) >= 3:
                center_lat = sum(c[0] for c in self.corners) / len(self.corners)
                center_lon = sum(c[1] for c in self.corners) / len(self.corners)
                mission_polygon = self.corners
            else:
                QMessageBox.warning(
                    self, "缺少區域定義",
                    "請先設定飛行區域：\n"
                    "  • 螺旋/同心圓：按「從地圖點選圓心」後點擊地圖\n"
                    "  • 網格：在地圖上點選至少 3 個邊界點"
                )
                return None

            planner = FixedWingPlanner()
            n_regions = self.flight_params.get('subdivisions', 1)
            spacing_m = self.flight_params.get('region_spacing', 0.0)

            # 起飛/降落點：優先使用獨立設定的 home_point，否則沿用掃描區中心
            if self.home_point is not None:
                home_lat, home_lon = self.home_point
            else:
                home_lat, home_lon = center_lat, center_lon

            # 進場方向 = home → survey_center（兩點距離 > 10m 才有意義）
            _dist_to_center = self._haversine((home_lat, home_lon), (center_lat, center_lon))
            if _dist_to_center > 10.0:
                _dlat_r = math.radians(center_lat - home_lat)
                _dlon_r = math.radians(center_lon - home_lon)
                _x = math.sin(_dlon_r) * math.cos(math.radians(center_lat))
                _y = (math.cos(math.radians(home_lat)) * math.sin(math.radians(center_lat))
                      - math.sin(math.radians(home_lat)) * math.cos(math.radians(center_lat))
                      * math.cos(_dlon_r))
                approach_bearing = (math.degrees(math.atan2(_x, _y)) + 360) % 360
            else:
                approach_bearing = fw_params.takeoff_bearing_deg

            # 共用起飛 / 降落路徑（不論分區與否皆相同）
            takeoff_path = planner.generate_takeoff_path(home_lat, home_lon, fw_params)
            landing_path = planner.generate_landing_path(home_lat, home_lon, fw_params)
            takeoff_ll = [(lat, lon) for lat, lon, _ in takeoff_path]
            landing_ll = [(lat, lon) for lat, lon, _ in landing_path]

            # 轉彎半徑合規性檢測（共用）
            check = TurnRadiusChecker.check(
                fw_params.turn_radius_m,
                fw_params.cruise_speed_mps,
                self.current_vehicle_model,
            )
            turn_status = {
                'ok': '符合規格',
                'warning': '接近極限（建議增大）',
                'critical': '超出極限！請立即修改',
            }.get(check['status'], '')

            def _path_dist(pts):
                return sum(self._haversine(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

            if n_regions > 1:
                # ── 多分區固定翼掃描 ────────────────────────────────────────
                if scan_mode == 'spiral':
                    _pattern = ScanPattern.SPIRAL
                elif scan_mode == 'grid':
                    _pattern = ScanPattern.GRID
                else:
                    _pattern = ScanPattern.CIRCULAR
                coverage_params = CoverageParameters(
                    spacing=fw_params.scan_spacing_m,
                    angle=scan_angle,
                    pattern=_pattern,
                    is_fixed_wing=True,
                    turn_radius=fw_params.turn_radius_m,
                    smooth_turns=True,
                    circle_base_altitude=fw_params.mission_altitude_m,
                    circle_direction=fw_params.circle_direction,
                )
                if _pattern == ScanPattern.GRID:
                    fw_fence_angle = coverage_angle_to_compass(scan_angle)
                cov_planner = CoveragePlanner()
                sub_scan_paths = cov_planner.plan_subdivided_coverage(
                    mission_polygon, coverage_params,
                    n_regions=n_regions,
                    spacing_m=spacing_m,
                )
                if not sub_scan_paths:
                    QMessageBox.warning(self, "路徑為空", "分區掃描路徑生成失敗，請檢查區域定義。")
                    return None

                self.fw_mission_result = None
                self.sub_paths = sub_scan_paths
                all_scan_pts = [pt for sp in sub_scan_paths for pt in sp]
                path = takeoff_ll + all_scan_pts + landing_ll
                path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "固定翼多分區")
                self.waypoints = path

                self.map_widget.display_fw_paths(takeoff_ll, sub_scan_paths, landing_ll,
                                                 scan_angle_deg=fw_fence_angle)

                takeoff_dist = _path_dist(takeoff_ll)
                landing_dist = _path_dist(landing_ll)
                scan_dists   = [_path_dist(sp) for sp in sub_scan_paths if len(sp) >= 2]
                total_dist   = takeoff_dist + sum(scan_dists) + landing_dist

                _wp_txt = f"航點: {len(path)}"
                if _trimmed:
                    _wp_txt += f" (原{_orig_n},已簡化)"
                self.waypoint_label.setText(_wp_txt)
                self.distance_label.setText(f"距離: {total_dist:.0f}m")
                self.mission_panel.update_mission_stats({
                    'waypoint_count': len(path),
                    'total_distance': total_dist,
                    'estimated_time': total_dist / max(self.flight_params.get('speed', 18.0), 0.1),
                    'area': CoveragePlanner().calculate_coverage_area(mission_polygon),
                    'regions': n_regions,
                })

                _fw_speed = max(self.flight_params.get('speed', 18.0), 0.1)
                scan_lines = "".join(
                    f"  區域 {i + 1}: {sd:.0f} m  ({sd / _fw_speed / 60:.1f} min)\n"
                    for i, sd in enumerate(scan_dists)
                )
                _takeoff_time = takeoff_dist / _fw_speed / 60
                _landing_time = landing_dist / _fw_speed / 60
                QMessageBox.information(
                    self, "固定翼多分區任務生成完成",
                    f"任務區域已分割為 {n_regions} 個子區域，各區域獨立掃描路徑。\n\n"
                    f"[起飛] {takeoff_dist:.0f} m  ({_takeoff_time:.1f} min)\n"
                    f"[掃描]\n{scan_lines}"
                    f"[降落] {landing_dist:.0f} m  ({_landing_time:.1f} min)\n\n"
                    f"總飛行距離: {total_dist:.0f} m\n"
                    f"預估總時間: {total_dist / _fw_speed / 60:.1f} min\n\n"
                    f"[轉彎半徑] {fw_params.turn_radius_m:.0f}m  "
                    f"傾斜: {check['required_bank']:.1f}°  {turn_status}"
                )
                logger.info(
                    f"固定翼多分區任務: {n_regions} 區, {len(path)} 航點, {total_dist:.0f}m"
                )
                return path

            else:
                # ── 單區域固定翼任務（三階段完整生成）──────────────────────
                # 螺旋/同心圓 + 圓心模式：使用 AdvancedScanGenerator（固定翼曲率約束）
                # 避免多邊形裁切把圓環切成不連續段，再直線相連形成八字路徑
                if scan_mode in ('spiral', 'circular') and self.circle_center is not None:
                    _alt = fw_params.mission_altitude_m
                    _max_r = self.flight_params.get('circle_max_radius', 200.0)
                    _mission_path, mission_latlon = self._generate_advanced_scan_mission(
                        scan_mode, center_lat, center_lon, _max_r,
                        fw_params, approach_bearing,
                    )
                    if _mission_path and fw_params.turn_radius_m > 0:
                        _mission_path = planner.smooth_path_with_arcs(
                            _mission_path, fw_params.turn_radius_m
                        )
                        mission_latlon = [(lat, lon) for lat, lon, _ in _mission_path]
                    result = {
                        'takeoff_path': takeoff_path,
                        'mission_path': _mission_path,
                        'mission_latlon': mission_latlon,
                        'landing_path': landing_path,
                        'full_path': takeoff_ll + mission_latlon + landing_ll,
                    }
                else:
                    if scan_mode == 'grid':
                        # 使用 CoveragePathPlanner（燈泡型轉彎）取代預設網格生成
                        # 圍籬角度由該函式**自己**回報（它內部有一條會換慣例的
                        # 退路），這裡不自己算。
                        result = self._generate_grid_with_coverage_planner(
                            mission_polygon, fw_params, scan_angle,
                            planner, takeoff_path, landing_path,
                            home_lat, home_lon,
                        )
                    else:
                        result = planner.generate_full_mission(
                            polygon=mission_polygon,
                            home_lat=home_lat,
                            home_lon=home_lon,
                            params=fw_params,
                            scan_pattern=scan_mode,
                            scan_angle_deg=scan_angle,
                        )

                # ── NFZ 路徑修正（繞行禁航區）───────────────────────────
                if self.nfz_zones:
                    _raw_mission_ll = result.get('mission_latlon', [])
                    if _raw_mission_ll:
                        _corrected_ll = self._apply_nfz_correction_latlon(
                            _raw_mission_ll, center_lat, center_lon
                        )
                        _alt_nfz = fw_params.mission_altitude_m
                        result['mission_latlon'] = _corrected_ll
                        result['mission_path'] = [
                            (lat, lon, _alt_nfz) for lat, lon in _corrected_ll
                        ]
                        # ── 回程/離場段也要繞 NFZ（鏡像 DCCPP _split_landing_tail）──
                        # 先前 takeoff_ll（離場）與 landing_ll（回程+降落）直接串接，
                        # 跳過 NFZ 檢查 → 離場/回程可能直穿禁區。
                        # 修法：把 takeoff_ll + 已修正掃描段 + landing_ll[0]（進場入口）
                        # 當成一條巡航包絡一起繞行；僅豁免最終下降進場型態 landing_ll[1:]
                        # （VG 折線會破壞下滑道幾何、且須保住觸地 NAV_LAND）。
                        if len(landing_ll) >= 2:
                            _approach_entry = [landing_ll[0]]
                            _descent_tail = list(landing_ll[1:])
                        else:
                            _approach_entry = list(landing_ll)
                            _descent_tail = []
                        _cruise_env = takeoff_ll + _corrected_ll + _approach_entry
                        _cruise_corr = self._apply_nfz_correction_latlon(
                            _cruise_env, center_lat, center_lon
                        )
                        result['full_path'] = _cruise_corr + _descent_tail

                # ── 插入進場弧：起飛末端 → 任務第一點（Dubins 雙約束）─────
                _mission_ll = result.get('mission_latlon', [])
                _tkoff_path = result.get('takeoff_path', [])
                if _mission_ll and _tkoff_path:
                    _p_end   = (_tkoff_path[-1][0], _tkoff_path[-1][1])
                    _p_first = _mission_ll[0]

                    # 由任務前兩點推算進入航向（羅盤方位）
                    if len(_mission_ll) >= 2:
                        _dx = _mission_ll[1][1] - _mission_ll[0][1]
                        _dy = _mission_ll[1][0] - _mission_ll[0][0]
                        _cos0 = math.cos(math.radians(_mission_ll[0][0]))
                        _first_hdg = (math.degrees(
                            math.atan2(_dx * _cos0, _dy)
                        ) + 360) % 360
                    else:
                        _first_hdg = fw_params.takeoff_bearing_deg

                    # 優先使用 Dubins（同時滿足出發 + 到達航向約束）
                    _entry_arc = planner.connect_with_dubins_latlon(
                        _p_end[0], _p_end[1], fw_params.takeoff_bearing_deg,
                        _p_first[0], _p_first[1], _first_hdg,
                        fw_params.turn_radius_m,
                        fw_params.mission_altitude_m,
                        step_size_m=8.0,
                    )
                    # Fallback：若 Dubins 失敗則用原圓弧平滑
                    if not _entry_arc:
                        _entry_arc = planner.generate_entry_arc(
                            _p_end,
                            fw_params.takeoff_bearing_deg,
                            _p_first,
                            fw_params.turn_radius_m,
                            fw_params.mission_altitude_m,
                            min_turn_deg=5.0,
                        )

                    if _entry_arc:
                        _arc_ll = [(_la, _lo) for _la, _lo, _ in _entry_arc]
                        result['mission_latlon'] = _arc_ll + _mission_ll
                        result['mission_path'] = (
                            _entry_arc + result.get('mission_path', [])
                        )
                        result['full_path'] = (
                            [(la, lo) for la, lo, _ in result['takeoff_path']]
                            + result['mission_latlon']
                            + [(la, lo) for la, lo, _ in result['landing_path']]
                        )
                        logger.info(
                            f"插入進場弧：{len(_entry_arc)} 個弧段點，"
                            f"起飛→任務轉向平滑化"
                        )

                # ── 插入離場弧：任務最後一點 → 對齊下風邊 → 五邊進場 ────
                # 幾何關係：
                #   飛機在下風邊飛行方向 = runway_hdg（downwind_start → downwind_end）
                #   ∴ lead_in 須在 downwind_start 的 opp_hdg 方向（飛行路徑的前方）
                #   弧段以 lead_in 為目標，飛機抵達後航向 ≈ runway_hdg，
                #   再直線飛入 downwind_start，完全消除急轉彎。
                _mission_ll_ex = result.get('mission_latlon', [])
                _landing_path  = result.get('landing_path', [])
                if len(_mission_ll_ex) >= 2 and _landing_path:
                    _p_last           = _mission_ll_ex[-1]
                    _p_downwind_start = (_landing_path[0][0], _landing_path[0][1])
                    _pat_alt          = fw_params.pattern_altitude_m

                    # 飛機進入下風邊的飛行方向 = runway_hdg
                    # lead_in 在 downwind_start 的 opp_hdg 方向（= 路徑上游）
                    _runway_hdg = fw_params.landing_bearing_deg
                    _opp_hdg    = (_runway_hdg + 180.0) % 360.0

                    _lead_dist = max(2.0 * fw_params.turn_radius_m, 150.0)
                    _dlat_m = 1.0 / 111_111.0
                    _dlon_m = 1.0 / (111_111.0 * math.cos(
                        math.radians(_p_downwind_start[0])) + 1e-12)
                    # lead_in = downwind_start + lead_dist * opp_hdg（上游方向）
                    _lead_lat = (_p_downwind_start[0]
                                 + _lead_dist * math.cos(math.radians(_opp_hdg)) * _dlat_m)
                    _lead_lon = (_p_downwind_start[1]
                                 + _lead_dist * math.sin(math.radians(_opp_hdg)) * _dlon_m)
                    _p_lead_in = (_lead_lat, _lead_lon)

                    # 最後一段航向（倒數第二點 → 最後一點）
                    _lx = (math.sin(math.radians(_p_last[1] - _mission_ll_ex[-2][1]))
                           * math.cos(math.radians(_p_last[0])))
                    _ly = (math.cos(math.radians(_mission_ll_ex[-2][0]))
                           * math.sin(math.radians(_p_last[0]))
                           - math.sin(math.radians(_mission_ll_ex[-2][0]))
                           * math.cos(math.radians(_p_last[0]))
                           * math.cos(math.radians(_p_last[1] - _mission_ll_ex[-2][1])))
                    _h_last = (math.degrees(math.atan2(_lx, _ly)) + 360) % 360

                    # 弧段：任務末端 → lead_in（Dubins 雙約束：出發=_h_last, 到達=runway_hdg）
                    _exit_arc = planner.connect_with_dubins_latlon(
                        _p_last[0], _p_last[1], _h_last,
                        _p_lead_in[0], _p_lead_in[1], _runway_hdg,
                        fw_params.turn_radius_m,
                        _pat_alt,
                        step_size_m=8.0,
                    )
                    # Fallback：Dubins 失敗時用原圓弧平滑
                    if not _exit_arc:
                        _exit_arc = planner.generate_entry_arc(
                            _p_last,
                            _h_last,
                            _p_lead_in,
                            fw_params.turn_radius_m,
                            _pat_alt,
                            min_turn_deg=5.0,
                        )
                    _exit_ll   = ([(_la, _lo) for _la, _lo, _ in _exit_arc]
                                  + [_p_lead_in])
                    _exit_path = (_exit_arc
                                  + [(_lead_lat, _lead_lon, _pat_alt)])
                    result['mission_latlon'] = _mission_ll_ex + _exit_ll
                    result['mission_path']   = (
                        result.get('mission_path', []) + _exit_path
                    )
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + [(la, lo) for la, lo, _ in result['landing_path']]
                    )
                    logger.info(
                        f"插入離場弧：{len(_exit_arc)} 個弧段點 + lead_in，"
                        f"runway_hdg={_runway_hdg:.0f}°，opp={_opp_hdg:.0f}°→五邊進場"
                    )

                # ── 最終任務段弧段平滑：消除進/離場弧接入殘留銳角 ─────
                if result.get('mission_path') and fw_params.turn_radius_m > 0:
                    _mp_final = planner.smooth_path_with_arcs(
                        result['mission_path'], fw_params.turn_radius_m
                    )
                    result['mission_path']   = _mp_final
                    result['mission_latlon'] = [(la, lo) for la, lo, _ in _mp_final]
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + [(la, lo) for la, lo, _ in result['landing_path']]
                    )

                # ── 任務末端 → 五邊進場橋接 + lead_in 轉角弧平滑 ────────────────────
                # 問題：離場弧使飛機到達 lead_in 時的航向 = bearing(p_last→lead_in)，
                #        但 lead_in→downwind_start 的航向 = runway_hdg，
                #        兩者不同 → lead_in 處出現銳角。
                # 修正：
                #   1. 以 [arc_exit, lead_in, downwind_start] 做 3 點弧平滑，
                #      將 lead_in 轉角替換為內切圓弧，弧出口與 runway_hdg 對齊。
                #   2. 弧尾接上 downwind_start，消除藍橙路徑視覺斷線。
                _mp_tail = result.get('mission_path', [])
                _lp_head = result.get('landing_path', [])
                if len(_mp_tail) >= 2 and len(_lp_head) >= 1:
                    _bridge_pt3d  = _lp_head[0]          # downwind_start
                    _bridge_ll    = (_bridge_pt3d[0], _bridge_pt3d[1])

                    # 只對末端 3 點做弧平滑，避免干擾已平滑的前段路徑
                    _tail3        = [_mp_tail[-2], _mp_tail[-1], _bridge_pt3d]
                    _tail3_smooth = planner.smooth_path_with_arcs(
                        _tail3, fw_params.turn_radius_m
                    )
                    # _tail3_smooth[0] = arc_exit（已在 _mp_tail[:-1] 末端，跳過避免重複）
                    # _tail3_smooth[1:] = [t_in, arc_pts..., t_out, downwind_start]
                    _extended = _mp_tail[:-1] + _tail3_smooth[1:]

                    result['mission_path']   = _extended
                    result['mission_latlon'] = [(la, lo) for la, lo, _ in _extended]
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + landing_ll
                    )
                    _arc_n = max(0, len(_tail3_smooth) - 3)
                    logger.info(
                        f"任務→五邊接縫弧：lead_in 轉角平滑，"
                        f"插入 {_arc_n} 個弧段點，"
                        f"downwind_start ({_bridge_ll[0]:.6f}, {_bridge_ll[1]:.6f})"
                    )

                self.fw_mission_result = result
                self.sub_paths = []
                path = result['full_path']
                path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "固定翼單區任務")
                self.waypoints = path

                # 確保 MAVLink 匯出的任務航點也在 _WP_LIMIT 以內
                # 預算 = 限制 - 起飛點數 - 降落點數 - 固定指令頭(1)
                _tkoff_n = len(result.get('takeoff_path', []))
                _land_n  = len(result.get('landing_path', []))
                _mission_budget = max(10, _WP_LIMIT - _tkoff_n - _land_n - 1)
                _mp = result.get('mission_path', [])
                if len(_mp) > _mission_budget:
                    _mp_ll  = [(la, lo) for la, lo, _ in _mp]
                    _mp_alt = [al for _, _, al in _mp]
                    _mp_ll_trim, _mp_alt_trim = CoveragePlanner.limit_path_waypoints(
                        _mp_ll, _mission_budget, _mp_alt
                    )
                    if not _mp_alt_trim:
                        _mp_alt_trim = [(_mp[0][2] if _mp else 100.0)] * len(_mp_ll_trim)
                    result['mission_path']   = [(la, lo, al) for (la, lo), al
                                                in zip(_mp_ll_trim, _mp_alt_trim)]
                    result['mission_latlon'] = _mp_ll_trim
                    _trimmed = True

                mission_ll = result['mission_latlon']
                # 只有真正排掃描線的那條分支會填這個 key；螺旋/同心圓沒有掃描角，
                # 拿不到就維持 None＝軸對齊圍籬。
                fw_fence_angle = result.get('fence_scan_angle_deg')
                self.map_widget.display_fw_paths(takeoff_ll, mission_ll, landing_ll,
                                                 scan_angle_deg=fw_fence_angle)
                # 提供完整 3D 高度資訊給 Cesium 地圖
                if hasattr(self.map_widget, 'set_fw_result'):
                    self.map_widget.set_fw_result(result)
                if self.nfz_zones:
                    self.map_widget.display_nfz_zones(self.nfz_zones)

                stats = FixedWingPlanner.estimate_mission_stats(
                    result, self.flight_params.get('speed', 18.0)
                )

                _wp_txt = f"航點: {stats['waypoint_count']}"
                if _trimmed:
                    _wp_txt = f"航點: {len(path)} (原{_orig_n},已簡化)"
                self.waypoint_label.setText(_wp_txt)
                total_dist = stats['total_dist_m']
                self.distance_label.setText(f"距離: {total_dist:.0f}m")
                self.mission_panel.update_mission_stats({
                    'waypoint_count': stats['waypoint_count'],
                    'total_distance': total_dist,
                    'estimated_time': stats['estimated_time_s'],
                    'area': CoveragePlanner().calculate_coverage_area(mission_polygon),
                    'regions': 1,
                })

                QMessageBox.information(
                    self, "固定翼任務生成完成",
                    f"三階段任務路徑已生成！\n\n"
                    f"[起飛階段]\n"
                    f"  方向: {fw_params.takeoff_bearing_deg:.0f}°  "
                    f"跑道: {fw_params.takeoff_runway_m:.0f}m\n"
                    f"  起飛距離: {stats['takeoff_dist_m']:.0f}m\n\n"
                    f"[主任務 ({scan_mode.upper()} 掃描)]\n"
                    f"  航點數: {len(result['mission_path'])} 個\n"
                    f"  掃描距離: {stats['mission_dist_m']:.0f}m\n\n"
                    f"[五邊進場降落]\n"
                    f"  進場方向: {fw_params.landing_bearing_deg:.0f}°  "
                    f"五邊高度: {fw_params.pattern_altitude_m:.0f}m\n"
                    f"  降落距離: {stats['landing_dist_m']:.0f}m\n\n"
                    f"總飛行距離: {total_dist:.0f}m\n"
                    f"預估時間: {stats['estimated_time_s'] / 60:.1f} min\n\n"
                    f"[轉彎半徑檢測]\n"
                    f"  半徑: {fw_params.turn_radius_m:.0f}m  "
                    f"所需傾斜: {check['required_bank']:.1f}°\n"
                    f"  狀態: {turn_status}"
                )
                logger.info(
                    f"固定翼任務生成: {stats['waypoint_count']} 個航點, "
                    f"總距離 {total_dist:.0f}m, 轉彎半徑={fw_params.turn_radius_m:.0f}m"
                )
                return path

        except Exception as e:
            logger.error(f"固定翼任務生成失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "固定翼任務錯誤", f"生成路徑時發生錯誤：\n{str(e)}")
            return None

    # ══════════════════════════════════════════════════════════════════
    # 整合 3：FixedWingNFZPlanner — 禁航區管理與路徑修正
    # ══════════════════════════════════════════════════════════════════
    # ── NFZ 地圖繪製 ──────────────────────────────────────────
    def on_nfz_draw_polygon(self):
        """啟用 NFZ 多邊形繪製模式：點擊地圖新增頂點，雙擊或按完成按鈕結束"""
        self.map_widget.set_nfz_polygon_draw_mode(True)
        if hasattr(self.parameter_panel, 'nfz_finish_poly_btn'):
            self.parameter_panel.nfz_finish_poly_btn.setVisible(True)
        self.statusBar().showMessage("🚫 NFZ 模式：點擊地圖新增頂點，雙擊或按「完成多邊形」完成")

    def on_nfz_poly_finish(self):
        """完成 NFZ 多邊形繪製"""
        self.map_widget.finish_nfz_polygon()
        if hasattr(self.parameter_panel, 'nfz_finish_poly_btn'):
            self.parameter_panel.nfz_finish_poly_btn.setVisible(False)

    def on_nfz_draw_circle(self):
        """啟用 NFZ 圓形拖曳繪製模式"""
        self.map_widget.set_nfz_circle_draw_mode(True)
        self.statusBar().showMessage("🚫 NFZ 圓形模式：按住滑鼠拖曳定義圓形禁航區")

    def on_nfz_polygon_drawn(self, vertices: list):
        """接收地圖繪製完成的多邊形 — 依 _fence_draw_pending 路由到對應流程。"""
        if hasattr(self.parameter_panel, 'nfz_finish_poly_btn'):
            self.parameter_panel.nfz_finish_poly_btn.setVisible(False)

        # ── Fence Zone 統一流程 ──
        if self._fence_draw_pending == 'polygon':
            self._fence_draw_pending = None
            self._open_fence_dialog_with_polygon(vertices)
            return

        # ── 原 NFZ 流程 ──
        name = f"NFZ_Poly_{len(self.nfz_zones) + 1}"
        zone = {'type': 'polygon', 'vertices': vertices, 'name': name}
        self.nfz_zones.append(zone)
        self.on_nfz_changed(self.nfz_zones)
        self.statusBar().showMessage(f"✅ 已新增 NFZ 多邊形「{name}」({len(vertices)} 頂點)")

    def on_nfz_circle_drawn(self, lat: float, lon: float, radius_m: float):
        """接收地圖拖曳完成的圓形 — 依 _fence_draw_pending 路由到對應流程。"""
        # ── Fence Zone 統一流程 ──
        if self._fence_draw_pending == 'circle':
            self._fence_draw_pending = None
            self._open_fence_dialog_with_circle(lat, lon, radius_m)
            return

        # ── 原 NFZ 流程 ──
        name = f"NFZ_Circle_{len(self.nfz_zones) + 1}"
        zone = {'type': 'circle', 'center': (lat, lon), 'radius': radius_m, 'name': name}
        self.nfz_zones.append(zone)
        self.on_nfz_changed(self.nfz_zones)
        self.statusBar().showMessage(f"✅ 已新增 NFZ 圓形「{name}」(r={radius_m:.0f}m)")

    # ──────────────────────────────────────────────────────────────
    #  Fence Zone — 直接從 3D 地圖繪製
    # ──────────────────────────────────────────────────────────────
    def on_fence_zone_draw_polygon(self):
        """進入地圖多邊形繪製模式（fence 流程）— 完成後自動開對話框預填。"""
        self._fence_draw_pending = 'polygon'
        # 重用既有 NFZ polygon 繪製機制
        self.map_widget.set_nfz_polygon_draw_mode(True)
        if hasattr(self.parameter_panel, 'nfz_finish_poly_btn'):
            self.parameter_panel.nfz_finish_poly_btn.setVisible(True)
        self.statusBar().showMessage(
            "⊕ Fence 多邊形繪製：點擊地圖新增頂點，雙擊或按「完成多邊形」結束 "
            "→ 完成後自動開對話框",
            0,
        )

    def on_fence_zone_draw_circle(self):
        """進入地圖圓形拖曳模式（fence 流程）— 完成後自動開對話框預填。"""
        self._fence_draw_pending = 'circle'
        self.map_widget.set_nfz_circle_draw_mode(True)
        self.statusBar().showMessage(
            "⊕ Fence 圓形拖曳：按住滑鼠從圓心往外拖曳，放開完成 "
            "→ 完成後自動開對話框",
            0,
        )

    def _open_fence_dialog_with_polygon(self, vertices: list) -> None:
        """地圖完成多邊形繪製後呼叫 — 開對話框並預填頂點。"""
        if not vertices or len(vertices) < 3:
            self.statusBar().showMessage(
                '⚠ Fence 多邊形頂點不足（< 3 點），已取消', 4000
            )
            return
        from ui.dialogs.fence_zone_dialog import FenceZoneDialog
        from mission.fence_zone import FenceZoneRegistry
        dialog = FenceZoneDialog(self)
        dialog.set_vertices(vertices)

        def _accepted(zone):
            FenceZoneRegistry.instance().add(zone)
            self.statusBar().showMessage(
                f'[Fence] 已新增 {zone.name} '
                f'({zone.category.value}, polygon, {len(vertices)} 頂點)',
                4000,
            )

        dialog.zone_accepted.connect(_accepted)
        dialog.exec()

    def _open_fence_dialog_with_circle(self, lat: float, lon: float,
                                       radius_m: float) -> None:
        """地圖完成圓形拖曳後呼叫 — 開對話框並預填圓心 / 半徑。"""
        if radius_m <= 0:
            self.statusBar().showMessage(
                '⚠ Fence 圓形半徑無效，已取消', 4000
            )
            return
        from ui.dialogs.fence_zone_dialog import FenceZoneDialog
        from mission.fence_zone import FenceZoneRegistry
        dialog = FenceZoneDialog(self)
        dialog.set_circle(lat, lon, radius_m)

        def _accepted(zone):
            FenceZoneRegistry.instance().add(zone)
            self.statusBar().showMessage(
                f'[Fence] 已新增 {zone.name} '
                f'({zone.category.value}, circle, r={radius_m:.0f}m)',
                4000,
            )

        dialog.zone_accepted.connect(_accepted)
        dialog.exec()

    def open_nfz_dialog(self):
        """開啟禁航區管理對話框"""
        from ui.dialogs.nfz_manager_dialog import NFZManagerDialog
        dialog = NFZManagerDialog(self, self.nfz_zones)
        dialog.nfz_changed.connect(self.on_nfz_changed)
        dialog.exec()

    # ──────────────────────────────────────────────────────────────
    #  統一 Fence Zone（NFZ + 威脅 + 圍籬整合）
    # ──────────────────────────────────────────────────────────────
    def open_fence_zone_dialog(self):
        """開啟統一 Fence Zone 設定對話框（fence-style 新增）。"""
        from ui.dialogs.fence_zone_dialog import FenceZoneDialog
        from mission.fence_zone import FenceZoneRegistry
        dialog = FenceZoneDialog(self)

        def _on_accepted(zone):
            FenceZoneRegistry.instance().add(zone)
            self.statusBar().showMessage(
                f'[Fence] 已新增區域 {zone.name} ({zone.category.value}, '
                f'{zone.shape.value})', 4000
            )

        dialog.zone_accepted.connect(_on_accepted)
        dialog.exec()

    def _on_fence_zone_added(self, zone):
        """FenceZoneRegistry → Cesium：把新增的區送進 3D 視覺。"""
        cesium = self._get_cesium_widget()
        if cesium is None or not hasattr(cesium, 'add_fence_zone'):
            return
        # 取頂點：POLYGON 直接、CIRCLE 展開為 N 邊形
        verts = zone.polygon_vertices(circle_segments=16)
        cesium.add_fence_zone(
            zone_id=zone.id,
            vertices=verts,
            alt_min=zone.alt_min_m,
            alt_max=zone.alt_max_m,
            name=zone.name,
            color_hex=zone.color_hex,
            category=zone.category.value,
            inclusion=bool(zone.inclusion),
        )

    def _on_fence_zone_removed(self, zone_id: str):
        cesium = self._get_cesium_widget()
        if cesium is None or not hasattr(cesium, 'remove_fence_zone'):
            return
        cesium.remove_fence_zone(zone_id)

    def clear_all_fence_zones(self):
        """清除所有 fence zones（registry + 視覺）。"""
        from mission.fence_zone import FenceZoneRegistry
        FenceZoneRegistry.instance().clear()
        cesium = self._get_cesium_widget()
        if cesium is not None and hasattr(cesium, 'clear_fence_zones'):
            cesium.clear_fence_zones()
        self.statusBar().showMessage('[Fence] 已清除所有區域', 3000)

    def upload_fence_zones_to_fc(self):
        """把所有 fence zones 以 ArduPilot Polygon Fence 協定上傳到各連線飛控。

        ArduPilot 4.2+ 支援多區 fence：
            POLYGON inclusion → MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001)
            POLYGON exclusion → MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION (5002)
            CIRCLE  inclusion → MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION         (5003)
            CIRCLE  exclusion → MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION         (5004)
        """
        from mission.fence_zone import FenceZoneRegistry
        reg = FenceZoneRegistry.instance()
        zones = reg.all()
        if not zones:
            self.statusBar().showMessage('[Fence] 沒有 fence 區域可上傳', 4000)
            return

        # 上傳到所有 SITL 連線
        links = getattr(self, '_sitl_links', None) or []
        if not links:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(
                self, 'SITL 未連線',
                '尚未連線任何 SITL，無法上傳 fence。\n請先在 SITL 分頁啟動 + 連線。'
            )
            return

        ok_count = 0
        for link in links:
            try:
                if hasattr(link, 'upload_fence_zones'):
                    link.upload_fence_zones(
                        zones,
                        fence_type_bits=reg.fence_type_bits(),
                        alt_max_m=reg.max_altitude_m(default=200.0),
                        alt_min_m=reg.min_altitude_m(default=-10.0),
                    )
                    ok_count += 1
            except Exception as e:
                logger.error(f'[Fence] 上傳到 link 失敗: {e}', exc_info=True)
        self.statusBar().showMessage(
            f'[Fence] 已送出上傳指令至 {ok_count}/{len(links)} 個飛控 '
            f'({len(zones)} 個區域)',
            6000,
        )

    def on_nfz_changed(self, nfz_zones: list):
        """更新禁航區清單，更新地圖顯示，並重新生成路徑"""
        self.nfz_zones = nfz_zones
        count = len(nfz_zones)
        logger.info(f"禁航區更新：{count} 個")
        # 更新 parameter_panel 計數顯示
        if hasattr(self.parameter_panel, 'nfz_count_label'):
            self.parameter_panel.nfz_count_label.setText(f"目前禁航區：{count} 個")
        # 更新地圖上的 NFZ 視覺化
        self.map_widget.display_nfz_zones(nfz_zones)
        # 重新生成路徑（若已有路徑）
        if self.waypoints:
            self.on_preview_paths()

    def _apply_nfz_correction_multirotor_latlon(
        self,
        path_latlon: list,
        ref_lat: float,
        ref_lon: float,
        safety_buffer_m: float = 5.0,
    ) -> list:
        """
        多旋翼路徑的 NFZ 繞行修正（Visibility Graph + Dijkstra）。

        多旋翼不需要固定翼的大轉彎半徑緩衝，
        使用 safety_buffer_m（預設 5m）作為 NFZ 邊界安全餘量。

        Args:
            path_latlon     : [(lat, lon), ...] 原始路徑
            ref_lat/ref_lon : 座標轉換參考原點
            safety_buffer_m : NFZ 邊界向外膨脹距離（公尺）

        Returns:
            修正後的 [(lat, lon), ...]
        """
        if not self.nfz_zones or not path_latlon:
            return path_latlon

        import math as _math
        from core.global_planner.nfz_planner import FixedWingNFZPlanner
        from core.base.fixed_wing_constraints import FixedWingConstraints
        from utils.math_utils import latlon_to_meters, meters_to_latlon

        # 建立等效約束：讓 r_min ≈ safety_buffer_m
        # r_min = V² / (g·tan(φ))，取 V=10m/s，解出 φ
        V = 10.0
        tan_phi = V ** 2 / (9.81 * max(safety_buffer_m, 0.5))
        bank_deg = min(80.0, _math.degrees(_math.atan(tan_phi)))
        constraints = FixedWingConstraints(
            cruise_airspeed_mps=V,
            max_bank_angle_deg=bank_deg,
            stall_speed_mps=4.0,
            safety_factor=1.0,
        )
        # buffer_factor=1.0 → buffer_distance = r_min ≈ safety_buffer_m
        nfz_planner = FixedWingNFZPlanner(constraints, buffer_factor=1.0)

        for zone in self.nfz_zones:
            if zone['type'] == 'polygon':
                nfz_planner.add_polygon_nfz(
                    zone['vertices'], zone['name'],
                    coord_type='latlon',
                    ref_latlon=(ref_lat, ref_lon),
                )
            elif zone['type'] == 'circle':
                clat, clon = zone['center']
                cx, cy = latlon_to_meters(clat, clon, ref_lat, ref_lon)
                nfz_planner.add_circle_nfz(
                    (cx, cy), zone['radius'], zone['name']
                )

        metric_path = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in path_latlon
        ]

        correction = nfz_planner.correct_path(metric_path)

        if correction.is_modified:
            logger.info(
                f"多旋翼 NFZ 路徑修正：{correction.segments_rerouted} 段繞行，"
                f"增加 {correction.added_length:.0f}m，"
                f"航點 {len(path_latlon)} → {len(correction.corrected_path)}"
            )
        else:
            logger.info("多旋翼 NFZ 檢查：路徑無需修正")

        return [
            meters_to_latlon(x, y, ref_lat, ref_lon)
            for x, y in correction.corrected_path
        ]

    def _apply_nfz_correction_latlon(
        self,
        path_latlon: list,
        ref_lat: float,
        ref_lon: float,
    ) -> list:
        """
        對 lat/lon 路徑套用禁航區繞行修正。

        內部使用公尺座標計算，完成後轉回 lat/lon 返回。

        Args:
            path_latlon: [(lat, lon), ...] 原始路徑
            ref_lat, ref_lon: 座標轉換參考原點

        Returns:
            修正後的 [(lat, lon), ...]（若無 NFZ 或無交叉則原樣返回）
        """
        if not self.nfz_zones or not path_latlon:
            return path_latlon

        from core.global_planner.nfz_planner import FixedWingNFZPlanner
        from utils.math_utils import latlon_to_meters, meters_to_latlon

        fw_params = self._build_fw_params()
        constraints = self._build_fw_constraints(fw_params)
        nfz_planner = FixedWingNFZPlanner(constraints, buffer_factor=1.5)

        # 加入所有禁航區（轉為公尺座標）
        for zone in self.nfz_zones:
            if zone['type'] == 'polygon':
                nfz_planner.add_polygon_nfz(
                    zone['vertices'], zone['name'],
                    coord_type='latlon',
                    ref_latlon=(ref_lat, ref_lon),
                )
            elif zone['type'] == 'circle':
                clat, clon = zone['center']
                cx, cy = latlon_to_meters(clat, clon, ref_lat, ref_lon)
                nfz_planner.add_circle_nfz(
                    (cx, cy), zone['radius'], zone['name']
                )

        # 將路徑轉為公尺座標
        metric_path = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in path_latlon
        ]

        correction = nfz_planner.correct_path(metric_path)

        if correction.is_modified:
            logger.info(
                f"NFZ 路徑修正：{correction.segments_rerouted} 段繞行，"
                f"增加 {correction.added_length:.0f}m，"
                f"航點 {len(path_latlon)} → {len(correction.corrected_path)}"
            )
        else:
            logger.info("NFZ 檢查：路徑無需修正")

        return [
            meters_to_latlon(x, y, ref_lat, ref_lon)
            for x, y in correction.corrected_path
        ]

    def _append_nfz_safe_return(
        self,
        path: list,
        ref_lat: float,
        ref_lon: float,
        fixed_wing: bool = False,
    ) -> list:
        """在掃描路徑尾端附加「回 home（path[0]）的 NFZ 繞行回程段」。

        多旋翼 / survey 匯出僅在尾端加 bare RTL（cmd 20），韌體會直線飛回 home
        → 直穿 NFZ。本方法以 Visibility Graph 繞行算出 path[-1] → home 的安全折線，
        把中繼繞行點 + home 顯式接到 path 尾端；匯出時的 RTL 變成 home→home 的
        無動作收尾。

        僅在「直線回程確實穿越 NFZ（產生繞行）」時才附加；無 NFZ / 不穿越則原樣
        回傳，維持既有行為（bare RTL 直線回程本就 NFZ-free）。

        Args:
            path: 掃描航點 [(lat, lon), ...]（path[0] 視為起飛/home 點）
            ref_lat, ref_lon: 座標轉換參考原點
            fixed_wing: True 用固定翼 buffer（R_min×1.5）；False 用多旋翼小 buffer

        Returns:
            附加回程後的新 path（未穿越時為原 path）
        """
        if not self.nfz_zones or len(path) < 2:
            return path
        home = path[0]
        last = path[-1]
        if abs(last[0] - home[0]) < 1e-9 and abs(last[1] - home[1]) < 1e-9:
            return path  # 已在 home，無需回程
        corrector = (self._apply_nfz_correction_latlon if fixed_wing
                     else self._apply_nfz_correction_multirotor_latlon)
        corrected = corrector([last, home], ref_lat, ref_lon)
        if len(corrected) <= 2:
            return path  # 直線回程未穿越 NFZ → 沿用 bare RTL，不附加多餘航點
        logger.info(
            f"NFZ 安全回程：附加 {len(corrected) - 1} 個繞行航點（避免 bare RTL 穿越禁區）"
        )
        # 去掉第一點（== last，重複），其餘繞行中繼點 + home 接到尾端
        return list(path) + [tuple(p) for p in corrected[1:]]

    # ══════════════════════════════════════════════════════════════════
    # 輔助：固定翼約束建立
    # ══════════════════════════════════════════════════════════════════
    def _build_fw_constraints(self, fw_params: 'FixedWingParameters'):
        """
        從 FixedWingParameters 建立 FixedWingConstraints。

        以使用者設定的 turn_radius_m 反推等效傾斜角：
            R = V²/(g·tan φ)  →  φ = atan(V²/(g·R))
        safety_factor=1.0（不額外放大），使 get_min_turn_radius() 直接返回 turn_radius_m，
        確保 AdvancedScanGenerator / CoveragePathPlanner 的轉彎幾何與 UI 設定一致。
        """
        from core.base.fixed_wing_constraints import FixedWingConstraints
        import math as _math
        speed = fw_params.cruise_speed_mps
        stall = min(12.0, speed * 0.6)
        r = max(fw_params.turn_radius_m, 10.0)
        # tan φ = V²/(g·R)
        tan_phi = speed ** 2 / (9.81 * r)
        bank_deg = _math.degrees(_math.atan(tan_phi))
        bank_deg = max(5.0, min(bank_deg, 80.0))  # 安全夾限
        return FixedWingConstraints(
            cruise_airspeed_mps=max(speed, stall + 1.0),
            max_bank_angle_deg=bank_deg,
            stall_speed_mps=stall,
            safety_factor=1.0,  # 不放大：R_min 直接等於 turn_radius_m
        )

    def _poses_to_latlon(self, poses, ref_lat: float, ref_lon: float):
        """將 List[Pose3D]（公尺座標）轉回 [(lat, lon), ...]"""
        from utils.math_utils import meters_to_latlon
        return [meters_to_latlon(p.x, p.y, ref_lat, ref_lon) for p in poses]

    # ══════════════════════════════════════════════════════════════════
    # 整合 1：AdvancedScanGenerator — 螺旋 / 同心圓掃描
    # ══════════════════════════════════════════════════════════════════
    def _generate_advanced_scan_mission(
        self,
        scan_mode: str,
        center_lat: float,
        center_lon: float,
        max_radius: float,
        fw_params: 'FixedWingParameters',
        approach_bearing: float,
    ):
        """
        使用 AdvancedScanGenerator 生成螺旋或同心圓掃描路徑。

        Args:
            scan_mode: 'spiral' 或 'circular'
            center_lat/lon: 掃描圓心（地理座標）
            max_radius: 最大半徑 [m]
            fw_params: 固定翼飛行參數
            approach_bearing: 進場航向 [度]，用於螺旋起始角

        Returns:
            (_mission_path, mission_latlon)
            _mission_path: [(lat, lon, alt), ...]
            mission_latlon: [(lat, lon), ...]
        """
        from core.global_planner.advanced_scan_patterns import (
            AdvancedScanGenerator, SpiralParams, ConcentricParams
        )
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator

        constraints = self._build_fw_constraints(fw_params)
        dubins_gen = DubinsTrajectoryGenerator(constraints)
        adv_gen = AdvancedScanGenerator(
            constraints=constraints,
            dubins_gen=dubins_gen,
            altitude_m=fw_params.mission_altitude_m,
        )
        _alt = fw_params.mission_altitude_m
        spacing = fw_params.scan_spacing_m

        if scan_mode == 'spiral':
            direction = fw_params.circle_direction  # 1=CCW, -1=CW
            params = SpiralParams(
                center=(0.0, 0.0),
                max_radius=max_radius,
                spacing=spacing,
                direction=direction,
                start_angle_deg=approach_bearing,
            )
            adv_result = adv_gen.generate_spiral(params, inward=True)
        else:  # circular (同心圓)
            params = ConcentricParams(
                center=(0.0, 0.0),
                max_radius=max_radius,
                spacing=spacing,
                arc_step_deg=5.0,
            )
            adv_result = adv_gen.generate_concentric(params)

        if adv_result.warnings:
            for w in adv_result.warnings:
                logger.warning(f"AdvancedScan: {w}")

        if not adv_result.poses:
            # 退回舊方法
            logger.warning("AdvancedScanGenerator 無法生成路徑，退回 CoveragePlanner")
            _cov = CoveragePlanner()
            if scan_mode == 'spiral':
                _sp_curv = self.flight_params.get('spiral_curvature', 1.0)
                _sp_alt_step = self.flight_params.get('spiral_alt_step', 0.0)
                mission_latlon = _cov.plan_spiral_from_center(
                    center_lat, center_lon, max_radius,
                    spacing, is_fixed_wing=True,
                    turn_radius=fw_params.turn_radius_m,
                    curvature=_sp_curv, alt_step=_sp_alt_step,
                    base_alt=_alt, entry_bearing_deg=approach_bearing,
                )
            else:
                _min_r = self.flight_params.get('circle_min_radius', 0.0)
                mission_latlon = _cov.plan_circle_survey_from_center(
                    center_lat, center_lon, max_radius, spacing,
                    base_alt=_alt, direction=fw_params.circle_direction,
                    min_radius_m=_min_r, start_angle_deg=approach_bearing,
                )
            _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]
            return _mission_path, mission_latlon

        mission_latlon = self._poses_to_latlon(adv_result.poses, center_lat, center_lon)
        _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]
        logger.info(
            f"AdvancedScan ({scan_mode}): {len(mission_latlon)} 航點, "
            f"總長={adv_result.total_length:.0f}m, "
            f"可行={'是' if adv_result.is_feasible else '否'}"
        )
        return _mission_path, mission_latlon

    # ══════════════════════════════════════════════════════════════════
    # 整合 2：CoveragePathPlanner — 固定翼網格掃描（燈泡型轉彎）
    # ══════════════════════════════════════════════════════════════════
    def _generate_grid_with_coverage_planner(
        self,
        mission_polygon,
        fw_params: 'FixedWingParameters',
        scan_angle: float,
        planner: 'FixedWingPlanner',
        takeoff_path,
        landing_path,
        home_lat: float,
        home_lon: float,
    ) -> dict:
        """
        使用 CoveragePathPlanner（Bulb Turn 燈泡型轉彎）生成固定翼網格掃描路徑。
        取代原本 generate_full_mission() 中的網格覆蓋邏輯。

        Returns:
            與 generate_full_mission() 相容的 result dict
        """
        from core.global_planner.coverage_path_planner import CoveragePathPlanner
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator
        from utils.math_utils import latlon_to_meters

        constraints = self._build_fw_constraints(fw_params)
        dubins_gen = DubinsTrajectoryGenerator(constraints)
        cov_planner = CoveragePathPlanner(
            constraints=constraints,
            dubins_gen=dubins_gen,
            altitude_m=fw_params.mission_altitude_m,
            airspeed_mps=fw_params.cruise_speed_mps,
        )

        # 以第一個角點為參考原點轉換到公尺座標
        ref_lat = sum(p[0] for p in mission_polygon) / len(mission_polygon)
        ref_lon = sum(p[1] for p in mission_polygon) / len(mission_polygon)
        metric_poly = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in mission_polygon
        ]

        cov_result = cov_planner.plan_coverage(
            polygon=metric_poly,
            scan_angle_deg=scan_angle,
            spacing_m=fw_params.scan_spacing_m,
            coord_type="metric",
        )

        if cov_result.warnings:
            for w in cov_result.warnings:
                logger.warning(f"CoveragePathPlanner: {w}")

        _alt = fw_params.mission_altitude_m
        takeoff_ll = [(lat, lon) for lat, lon, _ in takeoff_path]
        landing_ll = [(lat, lon) for lat, lon, _ in landing_path]

        if not cov_result.poses_3d:
            # 退回原方法
            # ⚠️⚠️ 這條退路換了掃描角慣例：generate_full_mission 把
            #   scan_angle_deg 直接塞進 CoverageParameters.angle（自東順時針），
            #   而上面 CoveragePathPlanner 是自北逆時針。scan_angle=30 時兩者
            #   實際飛的方向差 30°，所以圍籬角度**必須**跟著這條分支重算，
            #   不能沿用呼叫端算好的那個。
            _fb = planner.generate_full_mission(
                polygon=mission_polygon,
                home_lat=home_lat,
                home_lon=home_lon,
                params=fw_params,
                scan_pattern='grid',
                scan_angle_deg=scan_angle,
            )
            if isinstance(_fb, dict):
                _fb['fence_scan_angle_deg'] = coverage_angle_to_compass(scan_angle)
            return _fb

        mission_latlon = self._poses_to_latlon(cov_result.poses_3d, ref_lat, ref_lon)
        _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]

        logger.info(
            f"CoveragePathPlanner: {cov_result.num_scan_lines} 掃描線, "
            f"{len(mission_latlon)} 航點, 總長={cov_result.total_length:.0f}m"
        )

        return {
            'takeoff_path': takeoff_path,
            'mission_path': _mission_path,
            'mission_latlon': mission_latlon,
            'landing_path': landing_path,
            'full_path': takeoff_ll + mission_latlon + landing_ll,
            # 這條分支**實際**用的掃描方向，換算成羅盤（0=北、順時針），供圍籬對齊。
            # 由產生路徑的分支自己填，呼叫端不要另外猜（上面退路用的是別套慣例）。
            'fence_scan_angle_deg': fw_scan_angle_to_compass(scan_angle),
        }

    def on_export_waypoints(self):
        """匯出航點"""
        if not self.waypoints:
            QMessageBox.warning(self, "無資料", "請先設定邊界點並預覽路徑生成航點")
            return

        # ── 固定翼三階段任務匯出 ──────────────────────────────────────
        if self.current_vehicle_type == '固定翼' and self.fw_mission_result:
            self._export_fixed_wing_waypoints()
            return

        # ── 多台無人機分區匯出 ──────────────────────────────────────────
        if self.sub_paths and len(self.sub_paths) > 1:
            self._export_multi_uav_waypoints()
            return

        # ── 單台無人機：以高度為預設檔名 ────────────────────────────────
        altitude = self.flight_params['altitude']
        default_name = f"高度{altitude:.0f}m"

        # 開啟匯出對話框
        filepath, selected_filter = QFileDialog.getSaveFileName(
            self, "儲存航點檔案",
            default_name,
            "Mission Planner (*.waypoints);;QGC Waypoint Files (*.waypoints);;CSV Files (*.csv);;All Files (*)"
        )

        if not filepath:
            return

        try:
            speed    = self.flight_params['speed']
            yaw_spd  = self.flight_params.get('yaw_speed', 60.0)

            # ── CSV ──────────────────────────────────────────────────────────
            if selected_filter == "CSV Files (*.csv)" or filepath.endswith('.csv'):
                if not filepath.endswith('.csv'):
                    filepath += '.csv'
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write("sequence,latitude,longitude,altitude,speed\n")
                    for i, (lat, lon) in enumerate(self.waypoints):
                        _wp_alt = (self.spiral_waypoint_altitudes[i]
                                   if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                                   else altitude)
                        f.write(f"{i},{lat:.8f},{lon:.8f},{_wp_alt:.1f},{speed:.1f}\n")
                format_name = "CSV"

            # ── Mission Planner ──────────────────────────────────────────────
            elif selected_filter == "Mission Planner (*.waypoints)":
                if not filepath.endswith('.waypoints'):
                    filepath += '.waypoints'

                waypoint_lines = ['QGC WPL 110']
                seq = 0

                # seq 0: DO_SET_HOME（以目前位置為 Home）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=179,  # MAV_CMD_DO_SET_HOME
                    lat=0.0, lon=0.0, alt=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # seq 1: DO_CHANGE_SPEED（設定巡航速度）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=178,  # MAV_CMD_DO_CHANGE_SPEED
                    param1=0.0,    # 0=Airspeed（旋翼機等效地速）
                    param2=speed,  # 目標速度 (m/s)
                    param3=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # seq 2+: 航點 + CONDITION_YAW 交錯
                lock_heading  = self.flight_params.get('lock_heading', False)
                fixed_heading = self.flight_params.get('heading_angle', 0.0)

                for i, (lat, lon) in enumerate(self.waypoints):
                    _wp_alt = (self.spiral_waypoint_altitudes[i]
                               if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                               else altitude)
                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,  # MAV_CMD_NAV_WAYPOINT
                        lat=lat, lon=lon, alt=_wp_alt,
                        param1=0.0,  # hold time
                        param2=2.0,  # acceptance radius
                        current=0, autocontinue=1
                    ))
                    seq += 1

                    # 鎖定 Heading：每個航點都插入固定方向的 CONDITION_YAW
                    # 自動 Heading：只在相鄰航點之間插入，計算到下一點的方位角
                    if lock_heading:
                        heading = fixed_heading
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,  # MAV_CMD_CONDITION_YAW
                            param1=round(heading, 1),
                            param2=yaw_spd,
                            param3=0.0,  # 0=絕對角度
                            param4=0.0,  # 0=順時針
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    elif i < len(self.waypoints) - 1:
                        next_lat, next_lon = self.waypoints[i + 1]
                        heading = self._calc_bearing(lat, lon, next_lat, next_lon)
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,  # MAV_CMD_CONDITION_YAW
                            param1=round(heading, 1),
                            param2=yaw_spd,
                            param3=0.0,  # 0=絕對角度
                            param4=0.0,  # 0=順時針
                            current=0, autocontinue=1
                        ))
                        seq += 1

                # 返航（RTL）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=20,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                format_name = "Mission Planner"

            # ── QGC WPL 110 ──────────────────────────────────────────────────
            else:
                if not filepath.endswith('.waypoints'):
                    filepath += '.waypoints'

                home_lat, home_lon = self.waypoints[0]
                waypoint_lines = ['QGC WPL 110']

                # HOME 點
                waypoint_lines.append(create_waypoint_line(
                    seq=0, command=16,
                    lat=home_lat, lon=home_lon, alt=0.0,
                    current=1, autocontinue=1
                ))
                # 起飛點
                waypoint_lines.append(create_waypoint_line(
                    seq=1, command=22,  # MAV_CMD_NAV_TAKEOFF
                    lat=home_lat, lon=home_lon, alt=altitude,
                    param1=15.0,
                    current=0, autocontinue=1
                ))
                # 航點
                for i, (lat, lon) in enumerate(self.waypoints):
                    _wp_alt = (self.spiral_waypoint_altitudes[i]
                               if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                               else altitude)
                    waypoint_lines.append(create_waypoint_line(
                        seq=2 + i, command=16,
                        lat=lat, lon=lon, alt=_wp_alt,
                        param1=0.0, param2=2.0,
                        current=0, autocontinue=1
                    ))
                # RTL
                waypoint_lines.append(create_waypoint_line(
                    seq=2 + len(self.waypoints), command=20,
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                format_name = "QGC"

            QMessageBox.information(
                self, "匯出成功",
                f"航點檔案已匯出！\n\n"
                f"格式：{format_name}\n"
                f"檔案：{filepath}\n"
                f"航點數：{len(self.waypoints)}\n"
                f"高度：{altitude} m\n"
                f"速度：{speed} m/s"
            )
            self.statusBar().showMessage(f"已匯出 {len(self.waypoints)} 個航點至 {filepath}", 5000)
            logger.info(f"匯出航點 ({format_name}): {filepath}")

        except Exception as e:
            logger.error(f"匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def _export_fixed_wing_waypoints(self):
        """匯出固定翼三階段任務 .waypoints 檔（Mission Planner 格式）"""
        altitude = self.flight_params['altitude']
        default_name = f"固定翼任務_{altitude:.0f}m"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "儲存固定翼任務檔案",
            default_name,
            "Mission Planner (*.waypoints);;All Files (*)"
        )
        if not filepath:
            return
        if not filepath.endswith('.waypoints'):
            filepath += '.waypoints'

        try:
            fw_params = self._build_fw_params()
            planner = FixedWingPlanner()
            lines = planner.generate_mavlink_waypoints(
                mission_result=self.fw_mission_result,
                params=fw_params,
                speed_mps=self.flight_params.get('speed', 18.0),
            )

            from utils.file_io import write_waypoints
            write_waypoints(filepath, lines)

            stats = FixedWingPlanner.estimate_mission_stats(
                self.fw_mission_result, self.flight_params.get('speed', 18.0)
            )

            QMessageBox.information(
                self, "固定翼任務匯出成功",
                f"已匯出固定翼三階段任務！\n\n"
                f"檔案：{filepath}\n"
                f"總航點：{stats['waypoint_count']} 個\n"
                f"總距離：{stats['total_dist_m']:.0f} m\n"
                f"預估時間：{stats['estimated_time_s'] / 60:.1f} min\n\n"
                f"包含：起飛路徑 + {self.current_algorithm.upper()} 掃描 + 五邊進場降落"
            )
            self.statusBar().showMessage(f"已匯出固定翼任務至 {filepath}", 5000)
            logger.info(f"固定翼任務匯出: {filepath}")

        except Exception as e:
            logger.error(f"固定翼匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def _export_multi_uav_waypoints(self):
        """多台無人機分區匯出：每個子區域生成一個獨立 .waypoints 檔案"""
        import os

        output_dir = QFileDialog.getExistingDirectory(
            self, "選擇輸出資料夾", "",
            QFileDialog.Option.ShowDirsOnly
        )
        if not output_dir:
            return

        altitude        = self.flight_params['altitude']
        speed           = self.flight_params['speed']
        yaw_spd         = self.flight_params.get('yaw_speed', 60.0)
        stagger_enabled = self.flight_params.get('transit_stagger_enabled', False)
        stagger_alt     = self.flight_params.get('transit_stagger_alt', 10.0)
        lock_heading    = self.flight_params.get('lock_heading', False)
        fixed_heading   = self.flight_params.get('heading_angle', 0.0)

        exported_files = []
        try:
            for i, sub_path in enumerate(self.sub_paths):
                if not sub_path:
                    continue

                # 此台無人機的實際飛行高度
                uav_alt = altitude + i * stagger_alt if stagger_enabled else altitude

                # 自動命名
                if stagger_enabled:
                    filename = f"區域{i+1}_高度{uav_alt:.0f}m.waypoints"
                else:
                    filename = f"區域{i+1}_高度{altitude:.0f}m.waypoints"
                filepath = os.path.join(output_dir, filename)

                # 生成 Mission Planner 格式航點
                waypoint_lines = ['QGC WPL 110']
                seq = 0

                # HOME
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=179,
                    lat=0.0, lon=0.0, alt=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # DO_CHANGE_SPEED
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=178,
                    param1=0.0, param2=speed, param3=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # 航點 + CONDITION_YAW
                for j, (lat, lon) in enumerate(sub_path):
                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,
                        lat=lat, lon=lon, alt=uav_alt,
                        param1=0.0, param2=2.0,
                        current=0, autocontinue=1
                    ))
                    seq += 1

                    if lock_heading:
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,
                            param1=round(fixed_heading, 1),
                            param2=yaw_spd, param3=0.0, param4=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    elif j < len(sub_path) - 1:
                        next_lat, next_lon = sub_path[j + 1]
                        heading = self._calc_bearing(lat, lon, next_lat, next_lon)
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,
                            param1=round(heading, 1),
                            param2=yaw_spd, param3=0.0, param4=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1

                # RTL
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=20,
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                exported_files.append(filename)
                logger.info(f"匯出子區域 {i+1}: {filepath}")

            QMessageBox.information(
                self, "匯出成功",
                f"已匯出 {len(exported_files)} 個航點檔案至：\n{output_dir}\n\n"
                + "\n".join(exported_files)
            )
            self.statusBar().showMessage(
                f"已匯出 {len(exported_files)} 個子區域航點至 {output_dir}", 5000
            )
            logger.info(f"多台無人機匯出完成: {len(exported_files)} 個檔案")

        except Exception as e:
            logger.error(f"多台無人機匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def on_new_mission(self):
        """創建新任務"""
        # 如果有未儲存的變更，詢問是否儲存
        if self.current_mission and self.has_unsaved_changes():
            reply = QMessageBox.question(
                self, "未儲存的變更",
                "當前任務有未儲存的變更，是否儲存？",
                QMessageBox.StandardButton.Yes | 
                QMessageBox.StandardButton.No | 
                QMessageBox.StandardButton.Cancel
            )
            
            if reply == QMessageBox.StandardButton.Yes:
                self.on_save_mission()
            elif reply == QMessageBox.StandardButton.Cancel:
                return
        
        # 清除當前任務
        self.on_clear_all_silent()
        
        # 創建新任務
        self.current_mission = self.mission_manager.create_mission("新任務")
        
        self.statusBar().showMessage("已創建新任務", 3000)
        logger.info("創建新任務")
    
    def on_open_mission(self):
        """開啟任務"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "開啟任務檔案",
            "",
            "Mission Files (*.json);;All Files (*)"
        )
        
        if filepath:
            try:
                mission = self.mission_manager.load_mission(filepath)
                self.current_mission = mission
                
                # TODO: 載入任務參數到 UI
                
                self.statusBar().showMessage(f"已載入任務：{mission.name}", 3000)
                logger.info(f"載入任務: {filepath}")
                
            except Exception as e:
                logger.error(f"載入任務失敗: {e}")
                QMessageBox.critical(self, "載入錯誤", f"載入任務時發生錯誤：\n{str(e)}")
    
    # ═════════════════════════════════════════════════════════════════
    #  專案檔 (.aeroplan) 保存 / 讀取
    # ═════════════════════════════════════════════════════════════════
    def on_save_project(self):
        """儲存當前 UI 狀態為 .aeroplan 專案檔"""
        from mission.project_io import (
            build_project_from_main_window, save_project,
        )
        path, _ = QFileDialog.getSaveFileName(
            self, '儲存 AeroPlan 專案', '', 'AeroPlan Project (*.aeroplan)'
        )
        if not path:
            return
        try:
            proj = build_project_from_main_window(self)
            save_project(proj, path)
            self.statusBar().showMessage(f'專案已儲存: {path}', 4000)
            logger.info(f'[Project] 儲存至 {path}')
        except Exception as e:
            logger.error(f'[Project] 儲存失敗: {e}', exc_info=True)
            QMessageBox.critical(self, '儲存失敗', str(e))

    def on_load_project(self):
        """從 .aeroplan 檔案復原 UI 狀態"""
        from mission.project_io import load_project, apply_project_to_main_window
        path, _ = QFileDialog.getOpenFileName(
            self, '讀取 AeroPlan 專案', '', 'AeroPlan Project (*.aeroplan)'
        )
        if not path:
            return
        try:
            proj = load_project(path)
            apply_project_to_main_window(proj, self)
            self.statusBar().showMessage(
                f'專案已載入: {path} '
                f'(邊界 {len(proj.corners)} 點, 打擊目標 {len(proj.strike_targets)} 個)',
                5000,
            )
            logger.info(f'[Project] 從 {path} 載入')
        except Exception as e:
            logger.error(f'[Project] 載入失敗: {e}', exc_info=True)
            QMessageBox.critical(self, '載入失敗', str(e))

    def on_save_mission(self):
        """儲存任務"""
        if not self.current_mission:
            QMessageBox.warning(self, "無任務", "沒有任務可儲存")
            return
        
        try:
            filepath = self.mission_manager.save_mission(self.current_mission)
            
            if filepath:
                self.statusBar().showMessage("任務已儲存", 3000)
                logger.info(f"儲存任務: {filepath}")
            else:
                QMessageBox.warning(self, "儲存失敗", "無法儲存任務")
                
        except Exception as e:
            logger.error(f"儲存任務失敗: {e}")
            QMessageBox.critical(self, "儲存錯誤", f"儲存時發生錯誤：\n{str(e)}")
    
    def on_clear_paths(self):
        """清除路徑"""
        self.map_widget.clear_paths()
        self.waypoints.clear()
        self.spiral_waypoint_altitudes.clear()
        self.waypoint_label.setText("航點: 0")
        self.distance_label.setText("距離: 0.0m")
        # 路徑清掉了，DCCPP 結果也不再有效 —— 否則它仍會決定 SITL 的
        # 生成座標 / 台數 / 載具比對。
        self._dccpp_result = None
        logger.info("已清除路徑")
    
    def on_clear_corners(self):
        """清除邊界"""
        self.map_widget.clear_corners()
        self.corners.clear()
        self.parameter_panel.update_corner_count(0)
        # 同時清除圓心設定
        self.circle_center = None
        self.picking_circle_center = False
        if hasattr(self, 'parameter_panel') and hasattr(self.parameter_panel, 'circle_center_label'):
            self.parameter_panel.circle_center_label.setText("圓心：尚未設定")
            # 重置為 caption 灰色，呼應未設定狀態
            self.parameter_panel.circle_center_label.setProperty('role', 'caption')
            self.parameter_panel.circle_center_label.style().polish(
                self.parameter_panel.circle_center_label
            )
        if hasattr(self.map_widget, 'clear_circle_overlay'):
            self.map_widget.clear_circle_overlay()
        # 同時清除起飛點設定
        self.home_point = None
        self.picking_home_point = False
        if hasattr(self, 'parameter_panel'):
            self.parameter_panel.clear_home_point_display()
        if hasattr(self.map_widget, 'clear_home_point_overlay'):
            self.map_widget.clear_home_point_overlay()
        logger.info("已清除邊界")
    
    def on_clear_all(self):
        """清除全部（帶確認）"""
        reply = QMessageBox.question(
            self, "確認清除",
            "確定要清除所有標記和路徑嗎？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.on_clear_all_silent()
    
    def on_clear_all_silent(self):
        """清除全部（不帶確認）

        ⚠️ 舊版就只有「清除邊界 + 清除路徑 + 清 obstacles」三行 —— 從來沒有
        呼叫過 `map_widget.clear_all()`。也就是說「清除全部」實際上等於
        「清除邊界」加「清除路徑」，凡是不屬於這兩類的疊圖（電子圍籬、
        VTOL 三段軌跡、飛控讀回的 AUTO 航線、TOT 閘門、爆炸特效……）
        一個都沒被清掉，而 log 照樣印「已清除全部」。
        使用者看到的就是：按了清除全部，2D 還有琥珀虛線矩形、3D 還有
        黃色圍籬與黃/藍軌跡。
        """
        self.on_clear_corners()
        self.on_clear_paths()
        self.obstacles.clear()
        # 真正把兩張圖都清乾淨（含圍籬、VTOL 軌跡、飛控航線等非路徑疊圖）
        if hasattr(self.map_widget, 'clear_all'):
            self.map_widget.clear_all()
        # `clear_all` 會把地圖上的 NFZ 一起抹掉。這裡不同步清掉**模型**的話，
        # `self.nfz_zones` 會留著一份畫面上看不見、但仍會參與繞行修正、
        # 也仍會被下一次 display_nfz_zones() 原地畫回來的禁航區 —— 典型的
        # 「畫面清了、來源沒清」靜默分歧。
        if self.nfz_zones:
            self.on_nfz_changed([])
        logger.info("已清除全部")
    
    def on_reset_view(self):
        """重置視圖"""
        self.map_widget.reset_view()
    
    def on_toggle_grid(self):
        """切換網格顯示"""
        # TODO: 實現網格顯示切換
        QMessageBox.information(self, "網格顯示", "網格顯示功能開發中")
    
    def on_camera_config(self):
        """相機配置"""
        try:
            from ui.dialogs.camera_config import CameraConfigDialog
            dialog = CameraConfigDialog(self)
            dialog.exec()
        except ImportError:
            QMessageBox.information(self, "相機配置", "相機配置功能開發中")
    
    def on_vehicle_config(self):
        """飛行器配置"""
        try:
            from ui.dialogs.vehicle_config import VehicleConfigDialog
            dialog = VehicleConfigDialog(self)
            dialog.exec()
        except ImportError:
            QMessageBox.information(self, "飛行器配置", "飛行器配置功能開發中")
    
    def on_obstacle_manager(self):
        """障礙物管理"""
        try:
            from ui.dialogs.obstacle_manager import ObstacleManagerDialog

            dialog = ObstacleManagerDialog(self, self.obstacles)
            dialog.obstacles_changed.connect(self.on_obstacles_changed)
            dialog.exec()

        except ImportError as e:
            logger.error(f"無法載入障礙物管理對話框: {e}")
            QMessageBox.warning(self, "載入失敗", "無法載入障礙物管理功能")

    def on_open_polygon_editor(self):
        """開啟多邊形編輯器"""
        try:
            from ui.widgets.polygon_editor import PolygonEditorWindow

            # 創建編輯器視窗
            self.polygon_editor_window = PolygonEditorWindow(max_corners=MAX_CORNERS)

            # 如果已有角點，載入到編輯器
            if self.corners:
                self.polygon_editor_window.editor.set_corners(self.corners)

            # 連接信號 - 當編輯完成時同步角點
            self.polygon_editor_window.polygon_completed.connect(self._on_polygon_editor_completed)
            self.polygon_editor_window.editor.corners_changed.connect(self._on_polygon_editor_corners_changed)

            self.polygon_editor_window.show()
            logger.info("已開啟多邊形編輯器")

        except Exception as e:
            logger.error(f"開啟多邊形編輯器失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "錯誤", f"無法開啟多邊形編輯器：\n{str(e)}")

    def _on_polygon_editor_completed(self, corners):
        """多邊形編輯器完成編輯"""
        self._sync_corners_from_editor(corners)
        QMessageBox.information(
            self, "角點已同步",
            f"已從編輯器同步 {len(corners)} 個角點到主視窗"
        )

    def _on_polygon_editor_corners_changed(self, corners):
        """多邊形編輯器角點變更（即時同步）"""
        self._sync_corners_from_editor(corners)

    def _sync_corners_from_editor(self, corners):
        """從編輯器同步角點"""
        # 清除現有角點
        self.corners.clear()
        self.map_widget.corners.clear()
        self.map_widget.markers.clear()

        # 添加新角點
        for lat, lon in corners:
            self.corners.append((lat, lon))

        # 重新初始化地圖並添加角點
        self.map_widget.init_map()
        for lat, lon in self.corners:
            self.map_widget.add_corner(lat, lon)

        # 更新 UI
        self.parameter_panel.update_corner_count(len(self.corners))
        self._sync_area_counts()
        self.update_statusbar()
        logger.info(f"已同步 {len(corners)} 個角點")

    def on_obstacles_changed(self, obstacles):
        """處理障礙物變更"""
        self.obstacles = obstacles
        logger.info(f"障礙物已更新: {len(obstacles)} 個")
    
    def on_show_help(self):
        """顯示說明"""
        help_text = """
        <h2>AeroPlan Studio — 使用說明</h2>
        <p><i>Collaborative UAV Mission Planning Suite</i></p>

        <h3>① 定義作業區域</h3>
        <ul>
            <li><b>新增邊界點：</b>在 2D 地圖上左鍵點擊（至少 3 點）</li>
            <li><b>移動邊界點：</b>直接拖曳地圖標記</li>
            <li><b>多邊形編輯器：</b>Ctrl+M 開啟，可精確輸入經緯度</li>
            <li><b>螺旋/同心圓中心：</b>右側面板「地圖拖曳定義圓形」於地圖上拖曳</li>
        </ul>

        <h3>② 規劃路徑</h3>
        <ul>
            <li><b>選擇載具：</b>右側面板選 多旋翼 / 固定翼 / VTOL 與型號</li>
            <li><b>選擇演算法：</b>Grid / Spiral / Circular / A* / RRT / RRT* / Dijkstra / DWA</li>
            <li><b>調整參數：</b>高度、速度、間距、重疊率、掃描角度…</li>
            <li><b>預覽：</b>按 Enter 或「預覽」按鈕</li>
            <li><b>2D ↔ 3D：</b>地圖頂端 🗺 2D 衛星 / 🌐 3D Cesium 切換</li>
        </ul>

        <h3>③ DCCPP 多機協同（固定翼/多旋翼）</h3>
        <ul>
            <li>切到 <b>DCCPP</b> 分頁，設定 UAV 數量、FOV、重疊率、轉彎半徑</li>
            <li>可載入 DEM 地形（GeoTIFF / .npy）→ 啟用 GDA 高度平滑</li>
            <li>按「DCCPP 最佳化」執行 GreedyAllocator → IDP → Dubins → AltitudePlanner</li>
            <li>3D 地圖會以分色顯示每架 UAV 的完整三維航線</li>
        </ul>

        <h3>④ SITL 模擬（單機 / 多機）</h3>
        <ul>
            <li>切到 <b>🛰 SITL</b> 分頁</li>
            <li><b>單機：</b>選 PLANE / COPTER → 🚀 啟動 → 自動 tcp:5760 連線</li>
            <li><b>多機：</b>先做完 DCCPP，再按 🚀 啟動，會依每台 UAV 起飛點生成 N 個獨立 SITL（各自 console、SYSID、無漂移）</li>
            <li><b>上傳任務：</b>按 uav_id 將每台對應路徑分發到對應 SITL</li>
            <li><b>📂 參數檔：</b>批次匯入外部 .param 設定</li>
            <li>HUD 即時顯示 模式 / ARMED / 姿態 / 速度 / 電量 / GPS / FC log</li>
        </ul>

        <h3>⑤ 匯出</h3>
        <ul>
            <li>Ctrl+E 匯出 QGC WPL 110 (.waypoints)，可直接給 Mission Planner / QGC</li>
            <li>群飛模式可逐機匯出獨立任務檔</li>
        </ul>

        <h3>快捷鍵</h3>
        <ul>
            <li><b>Enter / Space</b>：生成路徑</li>
            <li><b>Esc</b>：清除路徑</li>
            <li><b>Delete / Backspace</b>：刪除最後一個角點</li>
            <li><b>Ctrl+N / O / S</b>：新建 / 開啟 / 儲存任務</li>
            <li><b>Ctrl+E</b>：匯出航點</li>
            <li><b>Ctrl+R</b>：清除全部</li>
            <li><b>Ctrl+M</b>：多邊形編輯器</li>
        </ul>
        """

        QMessageBox.information(self, "AeroPlan Studio 使用說明", help_text)

    def on_about(self):
        """關於"""
        about_text = """
        <h2>AeroPlan Studio</h2>
        <p><b>Collaborative UAV Mission Planning Suite</b></p>
        <p>版本 2.8.1 · PyQt6 · MIT License</p>
        <p>面向多機協同作業的專業級無人機任務規劃平台，整合：</p>
        <ul>
            <li>2D Folium / 3D Cesium 雙模式地圖（離線可用）</li>
            <li>Grid / Spiral / Circular 覆蓋路徑與 A* / RRT / DWA 規劃</li>
            <li>DCCPP 多機協同最佳化（GreedyAllocator + IDP + Dubins + GDA）</li>
            <li>固定翼三階段任務（起飛 → 螺旋/同心圓 → 五邊進場降落）</li>
            <li>ArduPilot SITL 多機模擬整合 + Mission Planner 風格 HUD</li>
            <li>QGC WPL 110 / MAVLink 任務匯出</li>
            <li>DEM 真實 3D 地形顯示（CustomHeightmapTerrainProvider）</li>
        </ul>
        <p>舊名：UAV Path Planner / DWA_path_planner</p>
        <p>© 2026 AeroPlan Studio</p>
        """

        QMessageBox.about(self, "關於 AeroPlan Studio", about_text)
    
    # ==========================================
    # 輔助函數
    # ==========================================
    
    def has_unsaved_changes(self):
        """檢查是否有未儲存的變更"""
        # TODO: 實現變更檢測
        return False
    
    def update_statusbar(self):
        """更新狀態列"""
        # 更新航點數量
        self.waypoint_label.setText(f"航點: {len(self.waypoints)}")
        
        # 更新邊界點數量
        if self.corners:
            self.statusBar().showMessage(f"邊界點: {len(self.corners)} 個", 2000)
    
    @staticmethod
    def _repair_path_kinks(
        path: list, planner, max_iters: int = 12,
        kink_angle_deg: float = 40.0,
    ) -> tuple:
        """閉迴圈：偵測並修復路徑中「不連貫轉角」（往回折尖刺）。

        相比舊版 _remove_path_spikes 的關鍵升級：
          • 閉迴圈 — 每輪重新偵測全路徑，修一個改一個，直到完全乾淨或卡住
          • detour 取代 — 尖刺點移除後若 A→C 穿 NFZ，改用 correct_path 在
            A、C 之間補繞行（而非放棄，這正是舊版漏掉尖刺的原因）

        kink 定義：連續三點 A-B-C，B 處入向 (B→A) 與出向 (B→C) 夾角過小
        （unit(B→A)·unit(B→C) > cos(kink_angle) → 兩邊近同向 → 路徑折返）。

        修復策略（每個 kink）：
          1. 若移除 B 後 A→C 不穿 NFZ → 直接移除 B（最乾淨）
          2. 否則 → correct_path([A, C]) 補繞行，用 detour 中間點取代 B
          3. 兩者都失敗 → 保留 B（極少數，下輪不再重試同一個）

        Args:
            path           : [(x, y), ...] metric 座標
            planner        : FixedWingNFZPlanner
            max_iters      : 閉迴圈最大輪數
            kink_angle_deg : 判定折返的夾角閾值（越大抓越多）

        Returns:
            (repaired_path, is_clean)
                repaired_path : 修復後路徑（端點保留）
                is_clean      : True = 已無 kink；False = 達上限仍有殘留
        """
        import math as _m
        from shapely.geometry import LineString as _LS
        if len(path) < 3:
            return list(path), True

        planner._rebuild_merged()
        merged = planner._merged_buffered
        polys = []
        if merged is not None and not merged.is_empty:
            polys = (list(merged.geoms) if merged.geom_type == 'MultiPolygon'
                     else [merged])

        def _crosses(a, b):
            if not polys:
                return False
            seg = _LS([a, b])
            for p in polys:
                if not seg.intersects(p):
                    continue
                inter = seg.intersection(p)
                if inter.is_empty:
                    continue
                if inter.geom_type in ('Point', 'MultiPoint'):
                    continue
                if hasattr(inter, 'length') and inter.length > 0.5:
                    return True
            return False

        cos_th = _m.cos(_m.radians(kink_angle_deg))

        def _first_kink(p):
            """回傳第一個 kink 的 index（B 點），無則 None。"""
            for k in range(1, len(p) - 1):
                A, B, C = p[k - 1], p[k], p[k + 1]
                bax = A[0] - B[0]; bay = A[1] - B[1]
                bcx = C[0] - B[0]; bcy = C[1] - B[1]
                la = _m.hypot(bax, bay)
                lc = _m.hypot(bcx, bcy)
                if la < 1e-6 or lc < 1e-6:
                    continue
                dot = (bax * bcx + bay * bcy) / (la * lc)
                if dot > cos_th:
                    return k
            return None

        cur = list(path)
        skip_indices: set = set()   # 修不掉的 kink（避免無限重試）
        for _ in range(max_iters):
            # 找第一個「非 skip」的 kink
            k = None
            kk = _first_kink(cur)
            while kk is not None:
                # 用座標當 key 判斷是否已標記為修不掉
                key = (round(cur[kk][0], 1), round(cur[kk][1], 1))
                if key in skip_indices:
                    # 跳過這個，往後找下一個 kink
                    found_next = None
                    for k2 in range(kk + 1, len(cur) - 1):
                        A, B, C = cur[k2 - 1], cur[k2], cur[k2 + 1]
                        bax = A[0]-B[0]; bay = A[1]-B[1]
                        bcx = C[0]-B[0]; bcy = C[1]-B[1]
                        la = _m.hypot(bax, bay); lc = _m.hypot(bcx, bcy)
                        if la < 1e-6 or lc < 1e-6:
                            continue
                        if (bax*bcx + bay*bcy)/(la*lc) > cos_th:
                            found_next = k2
                            break
                    kk = found_next
                else:
                    k = kk
                    break
            if k is None:
                return cur, True   # 全部乾淨（或剩下的都標記為修不掉）

            A, B, C = cur[k - 1], cur[k], cur[k + 1]
            # 策略 1：移除 B 後 A→C 不穿 NFZ → 直接移除
            if not _crosses(A, C):
                cur = cur[:k] + cur[k + 1:]
                continue
            # 策略 2：A→C 穿 NFZ → correct_path 補繞行取代 B
            try:
                rr = planner.correct_path([A, C])
                if rr.is_modified and rr.corrected_path and len(rr.corrected_path) >= 2:
                    cur = cur[:k] + list(rr.corrected_path[1:-1]) + cur[k + 1:]
                    continue
            except Exception:
                pass
            # 策略 3：都失敗 → 標記跳過，下輪不再重試
            skip_indices.add((round(B[0], 1), round(B[1], 1)))

        # 達上限 — 回報是否還有殘留
        return cur, (_first_kink(cur) is None)

    @staticmethod
    def _segment_aware_fence_avoidance(
        wps: list,
        metric_wps: list,
        planner,
    ) -> tuple:
        """Segment-type 差異化避讓 — 取代 ``planner.correct_path`` 用於 DCCPP。

        為何不直接用 correct_path？
            correct_path 是「點對點繞行」演算法，遇到 NFZ 內 waypoint 會
            「skip 到下個 NFZ 外 waypoint」。對 DCCPP boustrophedon scan 來說：
            多條 scan strip 的中段若全在 NFZ 內 → 被整批 skip → 大量偵蒐
            覆蓋被丟掉，NFZ 周圍出現大空洞，且所有繞行收斂到同一個 VG 頂點。

        Segment-type 差異化策略：
            • OPERATION 段（scan strip 本身）→ **截斷在 NFZ buffered 邊界**
              保留 NFZ 外的偵蒐覆蓋，僅 NFZ 內那段被放棄（這是 NFZ 的本意）
            • TAKEOFF / ENTRY / TRANSFER / LANDING 段 → VG 繞行 NFZ 外圍
              這些段本來就是 dead heading，繞外圈不會損失任何偵蒐

        Returns:
            (new_metric_xy, new_wp_refs)
                new_metric_xy : list[(x, y)] 修正後的 metric 航點
                new_wp_refs   : list[int] 每個新點對應到原 wps 的索引
                                （用於屬性繼承 — alt / heading / segment_type）
        """
        import math as _m
        from shapely.geometry import LineString, Point

        if len(wps) < 2:
            return list(metric_wps), list(range(len(metric_wps)))

        # 從 planner 取已合併且 buffered 的禁區 MultiPolygon
        planner._rebuild_merged()
        merged = planner._merged_buffered
        if merged is None or merged.is_empty:
            return list(metric_wps), list(range(len(metric_wps)))

        # 兼容 Polygon / MultiPolygon — 統一成 list of polygons
        polys = list(merged.geoms) if merged.geom_type == 'MultiPolygon' else [merged]

        def _is_op(wp) -> bool:
            """判斷 wp 是否為 OPERATION segment（scan strip 本身）"""
            seg = getattr(wp, 'segment_type', None)
            if seg is None:
                return False
            return getattr(seg, 'name', '') == 'OPERATION'

        def _pt_in_any_poly(xy) -> bool:
            p = Point(xy)
            return any(poly.contains(p) for poly in polys)

        def _seg_crosses_any(a, b) -> bool:
            """線段是否穿入任一 buffered 內部（沿邊滑行不算）"""
            seg = LineString([a, b])
            if seg.length < 1e-9:
                return False
            for poly in polys:
                if not seg.intersects(poly):
                    continue
                # 用中點 + 多點檢查穿入
                inter = seg.intersection(poly)
                if inter.is_empty:
                    continue
                if inter.geom_type in ('Point', 'MultiPoint'):
                    continue
                if hasattr(inter, 'length') and inter.length > 0.5:
                    return True
            return False

        # 主迴圈
        out_xy: list = [metric_wps[0]]
        out_refs: list = [0]
        n = len(metric_wps)
        i = 1

        while i < n:
            last_xy = out_xy[-1]
            last_ref = out_refs[-1]
            cur_xy = metric_wps[i]

            # 安全段 → 直接保留
            if not _seg_crosses_any(last_xy, cur_xy):
                out_xy.append(cur_xy)
                out_refs.append(i)
                i += 1
                continue

            # 穿越 NFZ — 判斷處理方式
            is_scan_strip = _is_op(wps[last_ref]) and _is_op(wps[i])

            if is_scan_strip:
                # ── OPERATION 段：difference-based 邊界截斷，保留「所有」NFZ 外掃描覆蓋 ──
                # 偵察任務以「覆蓋率」為優先：用 lane.difference(buffered_NFZ) 取得此線段在
                # NFZ 外的所有子段（依距 last_xy 排序），全部保留——即使 cur_xy 落在 NFZ 內，
                # 仍保留入口側覆蓋直到邊界；子段之間的缺口（NFZ 內）用 correct_path VG 繞行。
                #
                # 取代舊「Case A：while skip-inside → 丟整條尾段 / 整批 lane」的覆蓋損失：
                # 舊法在 lane 的遠端落在 NFZ 內時，會把該 lane（含其在 NFZ 另一側的覆蓋）整段
                # 跳過 → 偵察區大量空洞。新法逐線段裁切、保留每一段 NFZ 外覆蓋，缺口才繞行。
                # 不重排 / 不改順序（起飛接線、Dubins R_min、geofence 全部維持）。
                seg_line = LineString([last_xy, cur_xy])
                try:
                    outside = seg_line.difference(merged)
                except Exception:
                    outside = None
                pieces = []
                if outside is not None and not outside.is_empty:
                    geoms = (list(outside.geoms)
                             if outside.geom_type == 'MultiLineString' else [outside])
                    for g in geoms:
                        if getattr(g, 'geom_type', '') != 'LineString' or g.length <= 0.5:
                            continue
                        cs = list(g.coords)
                        a_pt = (cs[0][0], cs[0][1])
                        b_pt = (cs[-1][0], cs[-1][1])
                        da = _m.hypot(a_pt[0] - last_xy[0], a_pt[1] - last_xy[1])
                        db = _m.hypot(b_pt[0] - last_xy[0], b_pt[1] - last_xy[1])
                        if db < da:           # 端點依「距 last_xy」排：a=近端、b=遠端
                            a_pt, b_pt, da = b_pt, a_pt, db
                        pieces.append((da, a_pt, b_pt))
                    pieces.sort(key=lambda t: t[0])

                if not pieces:
                    # 整段都在 NFZ 內 → 無 NFZ 外覆蓋可保留；不加點、不穿越，直接前進
                    i += 1
                    continue

                for (_, a_pt, b_pt) in pieces:
                    prev = out_xy[-1]
                    # prev → 此覆蓋子段近端 a_pt：若中間跨 NFZ（缺口）→ correct_path VG 繞行
                    if _m.hypot(a_pt[0] - prev[0], a_pt[1] - prev[1]) > 0.5:
                        if _seg_crosses_any(prev, a_pt):
                            try:
                                r = planner.correct_path([prev, a_pt])
                                if (r.is_modified and r.corrected_path
                                        and len(r.corrected_path) >= 2):
                                    for dpt in r.corrected_path[1:-1]:
                                        out_xy.append(dpt)
                                        out_refs.append(last_ref)
                            except Exception:
                                pass
                        out_xy.append(a_pt)
                        out_refs.append(last_ref)
                    # 保留此 NFZ 外覆蓋子段的遠端（最大化偵察覆蓋）
                    out_xy.append(b_pt)
                    out_refs.append(i)
                i += 1
            else:
                # ── 非 OPERATION 段（TRANSFER/ENTRY/TAKEOFF/LANDING）：VG 繞行 ──
                j = i
                while j < n and _pt_in_any_poly(metric_wps[j]):
                    j += 1
                if j >= n:
                    break
                try:
                    result = planner.correct_path([last_xy, metric_wps[j]])
                    if result.is_modified and result.corrected_path:
                        for dpt in result.corrected_path[1:]:
                            out_xy.append(dpt)
                            out_refs.append(j)
                    else:
                        out_xy.append(metric_wps[j])
                        out_refs.append(j)
                except Exception:
                    out_xy.append(metric_wps[j])
                    out_refs.append(j)
                i = j + 1

        return out_xy, out_refs

    @staticmethod
    def _split_takeoff_head(
        waypoints: list,
    ) -> tuple:
        """剝離開頭「起飛爬升型態」供 NFZ 避障豁免（_split_landing_tail 的鏡像）。

        為什麼需要
        ----------
        VTOL 的起飛前兩點是**同一個經緯度**（純垂直爬升）。NFZ 避障後會用
        ``_resample_alts_by_arclength`` 依**水平**弧長比例重採樣高度剖面，
        而垂直段的水平弧長是 0 ⇒ 它在比例上佔 0，整段 0 m→轉換高度 的爬升
        會被擠掉，變成從地面直接以巡航高度出發。

        作法與尾端對稱：
          1. 找出開頭連續的 TAKEOFF 區段。
          2. 把「最後一個 TAKEOFF 航點（爬升完成點）」留在避障段 ``rest``，
             讓「爬升完成點 → 第一個作業點」這條長離場巡航段仍會繞 NFZ。
          3. 其餘（地面點 + 垂直段 + 過渡段）放進 ``exempt_head`` 原樣豁免，
             保住垂直起飛幾何與匯出時的 NAV_VTOL_TAKEOFF。

        Returns:
            (exempt_head, rest)
        """
        n = len(waypoints)
        i = 0
        while i < n and getattr(
            getattr(waypoints[i], 'segment_type', None), 'name', ''
        ) == 'TAKEOFF':
            i += 1
        if i == 0:
            return [], list(waypoints)
        # 爬升完成點留給避障段，讓離場巡航仍被涵蓋
        exempt_head = list(waypoints[:max(0, i - 1)])
        rest = list(waypoints[max(0, i - 1):])
        return exempt_head, rest

    @staticmethod
    def _split_landing_tail(
        waypoints: list,
    ) -> tuple:
        """剝離尾端「下降進場型態」供 NFZ 避障豁免，但把回程到達點併入避障前段。

        DCCPP 固定翼路徑尾端是一串 ``SegmentLabel.LANDING`` 航點（場周入口 →
        下降階梯 → 觸地 NAV_LAND），全部聚在 home 附近。真正會穿越 NFZ 的是
        「最後一個作業點 → 第一個 LANDING 航點（場周入口）」這條長回程巡航段。

        先前的實作把「整段」尾端 LANDING 一起豁免，連這條長回程巡航段都跳過
        避障，導致回程直穿 NFZ（即使用者回報的 bug）。本方法改為：

          1. 找出尾端連續的 LANDING 區段。
          2. 把「第一個 LANDING 航點（場周入口／回程到達點）」併入前段 ``head``，
             讓「最後作業點 → 場周入口」的長回程巡航段進入 NFZ 避障
             （VG 繞行 + Dubins fillet）。
          3. 其餘下降進場型態（下降階梯 + 觸地點）放進 ``exempt_tail`` 原樣豁免，
             保住匯出時的 NAV_LAND 與既定下滑道幾何（VG 折線會破壞進場下滑道）。

        Args:
            waypoints: 單架 UAV 的完整 assembled 航點（含尾端 LANDING 段）。

        Returns:
            (head, exempt_tail)
                head        : list — 需做 NFZ 避障的航點（已併入回程到達點）
                exempt_tail : list — 豁免避障、原樣接回的下降進場航點
        """
        ls = len(waypoints)
        while ls > 0 and getattr(
            getattr(waypoints[ls - 1], 'segment_type', None), 'name', ''
        ) == 'LANDING':
            ls -= 1
        landing_tail = list(waypoints[ls:])
        head = list(waypoints[:ls])
        if landing_tail:
            # 回程到達點（第一個 LANDING 航點 = 場周入口）併入避障前段，
            # 使長回程巡航段被 NFZ 避障涵蓋；其餘下降進場仍豁免。
            head.append(landing_tail[0])
            landing_tail = landing_tail[1:]
        return head, landing_tail

    @staticmethod
    def _resample_alts_by_arclength(
        query_xy: list,
        ref_xy: list,
        ref_alt: list,
    ) -> list:
        """沿累積弧長比例，把 (ref_xy, ref_alt) 高度剖面重採樣到 query_xy 各點。

        用途：路徑經 Dubins fillet 倒角 / kink 修復重新離散化後，原 waypoint 索引
        失效。若改用「幾何最近原始航點」推高度，繞 NFZ 的 detour 點會繼承到鄰近
        低高度的 takeoff/landing 航點 → 路徑貼地。改以弧長比例內插，detour 點即可
        保持其所屬巡航段的高度。

        Args:
            query_xy: 目標折線 [(x, y), ...]（最終 corrected_path，公尺）
            ref_xy:   基準折線 [(x, y), ...]（fillet 前 corrected_path，公尺）
            ref_alt:  基準折線各點高度（與 ref_xy 等長）

        Returns:
            query_xy 各點對應高度 list（長度與 query_xy 相同）
        """
        import math as _m
        nq = len(query_xy)
        nref = len(ref_xy)
        if nref == 0 or not ref_alt:
            return [0.0] * nq
        if nref == 1:
            return [ref_alt[0]] * nq

        def _cum(path: list) -> list:
            cum = [0.0]
            for k in range(1, len(path)):
                cum.append(cum[-1] + _m.hypot(
                    path[k][0] - path[k - 1][0],
                    path[k][1] - path[k - 1][1],
                ))
            return cum

        ref_cum = _cum(ref_xy)
        ref_total = ref_cum[-1] or 1.0
        q_cum = _cum(query_xy)
        q_total = q_cum[-1] or 1.0

        out: list = []
        j = 0
        m = min(nref, len(ref_alt))
        for k in range(nq):
            target = (q_cum[k] / q_total) * ref_total   # 對應到 ref 的弧長位置
            while j < m - 2 and ref_cum[j + 1] < target:
                j += 1
            seg = ref_cum[j + 1] - ref_cum[j]
            t = 0.0 if seg <= 1e-9 else (target - ref_cum[j]) / seg
            t = max(0.0, min(1.0, t))
            out.append(ref_alt[j] + (ref_alt[j + 1] - ref_alt[j]) * t)
        return out

    def _apply_fence_avoidance_to_dccpp_result(
        self,
        dccpp_result: dict,
        centroid_lat: float,
        centroid_lon: float,
        turn_radius: float,
    ) -> None:
        """DCCPP 規劃完成後，把所有 NFZ + THREAT 分類的 fence 區域當作禁區，
        對每架 UAV 的 assembled 航點做 segment-type 差異化避讓修正。

        策略（_segment_aware_fence_avoidance）：
          • OPERATION 段（scan strip）：截斷在 NFZ 邊界，保留 NFZ 外的覆蓋
          • 其它段（TAKEOFF/TRANSFER 等）：VG 繞 NFZ 外圍
          • 回程：尾端 LANDING 進場型態原樣豁免，但「最後作業點 → 場周入口」
            這條長回程巡航段會併入避障（見 _split_landing_tail），避免回程穿越 NFZ

        診斷策略：所有失敗都會明確 log + 把摘要彈到 statusBar，避免「靜默失敗」。
        當有區域卻 0 機繞行 → 彈 QMessageBox 確保使用者一定知道。
        """
        try:
            from mission.fence_zone import FenceZoneRegistry, ZoneCategory
        except Exception as e:
            logger.warning(f'[DCCPP-Fence] FenceZoneRegistry 載入失敗: {e}')
            return
        reg = FenceZoneRegistry.instance()
        total_zones = reg.count()
        avoid_zones = (reg.by_category(ZoneCategory.NFZ)
                       + reg.by_category(ZoneCategory.THREAT))

        # ── 診斷起點 — 永遠 log 一次 ──
        logger.info(
            f'[DCCPP-Fence] 後處理開始 — registry 共 {total_zones} 區域，'
            f'其中 NFZ+THREAT = {len(avoid_zones)} 個'
        )
        if not avoid_zones:
            logger.info(
                '[DCCPP-Fence] 無 NFZ / THREAT 分類區域 → 跳過繞行 '
                '(GEOFENCE 不在此處處理)'
            )
            return

        assembled = dccpp_result.get('assembled_paths') or {}
        if not assembled:
            logger.warning(
                '[DCCPP-Fence] dccpp_result 中無 assembled_paths → 無路徑可修正'
            )
            return

        import math as _m
        import copy as _copy
        try:
            from core.global_planner.nfz_planner import FixedWingNFZPlanner
            from core.base.fixed_wing_constraints import FixedWingConstraints
        except Exception as e:
            logger.warning(f'[DCCPP-Fence] 模組載入失敗，跳過繞行: {e}')
            return

        # ── 本地 equirect 投影（centroid 為原點）──
        cos_lat = max(_m.cos(_m.radians(centroid_lat)), 1e-6)

        def to_metric(lat: float, lon: float) -> tuple:
            x = (lon - centroid_lon) * 111320.0 * cos_lat
            y = (lat - centroid_lat) * 111320.0
            return (x, y)

        def to_latlon(x: float, y: float) -> tuple:
            lat = centroid_lat + y / 111320.0
            lon = centroid_lon + x / (111320.0 * cos_lat)
            return (lat, lon)

        # ── 構造 FixedWingConstraints + 動態 buffer_factor ──
        # Buffer 大小依機型差異化（讓繞行更貼近 NFZ，減少偵蒐死區）：
        #   • 固定翼：R_min × 1.5 ≥ ~90m（物理硬約束，無法再縮）
        #   • 多旋翼：可懸停急轉，buffer 僅需 WP_RADIUS + GPS 容差 ≈ 10m
        # 原本對兩者都用 30m 下限對多旋翼過度保守，造成 NFZ 周圍 20m 偵蒐空檔。
        is_fixed_wing = turn_radius and turn_radius > 0
        r_min = float(turn_radius) if is_fixed_wing else 5.0
        if is_fixed_wing:
            # 固定翼：R_min 是物理硬約束（飛機飛不過更小的圈），buffer 必須容納
            min_buffer_m = max(20.0, r_min * 1.1)  # 至少 R_min + 10% 餘裕
        else:
            # 多旋翼：可懸停、零轉彎半徑；buffer 主要為 GPS 漂移 + WP advance
            #   取 10m（≈ WP_RADIUS 一半 + GPS 5m），讓掃描貼近 NFZ
            min_buffer_m = 10.0
        buffer_factor = max(1.5, min_buffer_m / max(r_min, 1.0))
        cruise_v = max(_m.sqrt(r_min * 9.81), 1.5)
        stall_v = max(cruise_v * 0.5, 0.5)
        try:
            constraints = FixedWingConstraints(
                cruise_airspeed_mps=cruise_v,
                max_bank_angle_deg=45.0,
                stall_speed_mps=stall_v,
                max_speed_mps=cruise_v * 1.5,
                safety_factor=1.0,
            )
        except Exception as e:
            logger.warning(f'[DCCPP-Fence] 約束建立失敗: {e}')
            return

        # ★★★ per-UAV 扇形展開（已停用以最大化覆蓋率）★★★
        # 原為避免多機繞行擠在同一走廊，曾用 base buffer + i×12m 的擴大 buffer 做扇形分散；
        # 但擴大 buffer 會把「截斷掃描覆蓋」的邊界也外推 → UAV_i 在 NFZ 周圍多損失 12×i m
        # 的偵察覆蓋（純覆蓋犧牲、與飛安無關）。偵察任務以覆蓋率為優先，且多機已用高度分層
        # （altitude_step）避撞，無需橫向扇形；故設 0，讓每架都用最小安全 buffer、保留最大覆蓋。
        spread_per_uav_m = 0.0

        # 計算「可加入的區域數」與「基準 buffer」（給最終摘要 / 警告用）
        added_count = sum(
            1 for z in avoid_zones
            if len(z.polygon_vertices(circle_segments=16)) >= 3
        )
        if added_count == 0:
            logger.warning('[DCCPP-Fence] 沒有有效區域可加入 — 跳過繞行')
            return
        actual_buffer = r_min * buffer_factor   # 基準 buffer（UAV idx=0）

        # ★★★ 修正二：Dubins fillet 倒角驗證（避免 90° 直角）★★★
        # correct_path 回傳的折線在 NFZ buffered 多邊形「頂點」處轉彎，
        # 對 4 角矩形 NFZ 是直角。固定翼 L1 飛不過 90° 急彎。
        # 用 _ArduPilotDubinsPlanner._apply_fillets() 後處理：
        #   每個轉角用 R_min 圓弧倒角、自動計算離散段數使 chord ≥ 2·WP_R
        try:
            from core.global_planner.dccpp_heterogeneous_nfz_manager import (
                _ArduPilotDubinsPlanner, ArduPilotParams,
            )
            from core.global_planner.heterogeneous_nfz_planner import (
                PlannerConfig as _FilletCfg,
            )
            # WP_RADIUS 預設 20m（多數機型常用值），未來可從 FCU 讀取
            _ap_for_fillet = ArduPilotParams(
                wp_radius=20.0,
                level_roll_limit_deg=35.0,
                airspeed_cruise=max(cruise_v, 1.0),
            )
            _fillet_planner = _ArduPilotDubinsPlanner(
                _FilletCfg(r_min=r_min, arc_segments=_ap_for_fillet.arc_segments_cap),
                _ap_for_fillet,
            )
            _fillet_enabled = True
        except Exception as e:
            logger.warning(f'[DCCPP-Fence] Dubins fillet 模組載入失敗: {e}')
            _fillet_planner = None
            _fillet_enabled = False

        # ── 逐機跑 segment-aware avoidance（每架用獨立 planner 實現扇形展開）──
        sorted_uav_ids = sorted(assembled.keys())
        n_rerouted = 0
        for uav_idx, uav_id in enumerate(sorted_uav_ids):
            apath = assembled[uav_id]
            # ★ 降落段豁免（修正回程穿越 NFZ）：
            #   尾端連續的 LANDING 段是「飛回 home 降落」的進場型態；其下降階梯 +
            #   觸地點 (NAV_LAND) 不可被 VG 繞掉/丟棄（否則匯出就沒有 NAV_LAND，
            #   且 VG 折線會破壞進場下滑道幾何）。
            #   但「最後一個作業點 → 第一個 LANDING 航點（場周入口）」這條長回程
            #   巡航段先前被一起豁免，導致回程直穿 NFZ。_split_landing_tail 會把
            #   場周入口併入避障前段，使回程巡航走 VG 繞行 + Dubins fillet，
            #   其餘下降進場型態 (landing_tail) 仍原樣豁免接回。
            _full_wps = apath.waypoints
            # 起飛段也要豁免：VTOL 的垂直爬升水平弧長為 0，
            # 被 _resample_alts_by_arclength 重採樣時會整段被擠掉。
            takeoff_head, _rest_wps = self._split_takeoff_head(_full_wps)
            wps, landing_tail = self._split_landing_tail(_rest_wps)
            if not wps or len(wps) < 2:
                continue

            # 此架 UAV 的擴大 buffer — buffer_factor 動態加 spread
            this_buffer_m = (r_min * buffer_factor) + uav_idx * spread_per_uav_m
            this_factor = this_buffer_m / max(r_min, 1.0)
            per_uav_planner = FixedWingNFZPlanner(
                constraints, buffer_factor=this_factor,
            )
            for z in avoid_zones:
                verts_ll = z.polygon_vertices(circle_segments=16)
                if len(verts_ll) < 3:
                    continue
                verts_xy = [to_metric(lat, lon) for lat, lon in verts_ll]
                try:
                    per_uav_planner.add_polygon_nfz(
                        verts_xy, name=z.name, coord_type='metric',
                    )
                except Exception:
                    pass

            metric_wps = [to_metric(wp.lat, wp.lon) for wp in wps]

            # ★★★ Segment-aware avoidance（OPERATION 段截斷在 NFZ 邊緣，局部繞行）★★★
            # 讓掃描線「貼著 NFZ 牆邊」結束，保留 NFZ 周圍最大覆蓋；其餘段 VG 繞行。
            # 對 in/out 配對的端點做 5m nudge，避免 VG 在邊界上失敗。
            #
            # 註：曾嘗試「NFZ-aware cell 覆蓋分解」(重排整條覆蓋以求 MP 式乾淨)，但重排
            # 會打亂起飛接線、cell 轉場可能飛出 geofence、且以 fillet 取代 Dubins 導致
            # 轉彎半徑可能 < R_min（固定翼飛不過）。為保實飛安全（起飛/邊界/轉彎半徑），
            # 改回此 segment-aware 局部繞行：保留原 DCCPP 覆蓋順序 + Dubins R_min + 邊界內。
            ref_indices = None
            try:
                corrected_path, ref_indices = self._segment_aware_fence_avoidance(
                    wps, metric_wps, per_uav_planner,
                )
            except Exception as e:
                logger.warning(
                    f'[DCCPP-Fence] UAV {uav_id} segment-aware 失敗: {e}'
                )
                continue

            # 偵測是否有修改
            is_modified = (
                len(corrected_path) != len(metric_wps)
                or any(
                    abs(a[0]-b[0]) > 1e-6 or abs(a[1]-b[1]) > 1e-6
                    for a, b in zip(corrected_path, metric_wps)
                )
            )

            logger.info(
                f'[DCCPP-Fence] UAV {uav_id} (idx={uav_idx}, '
                f'buf={per_uav_planner.buffer_distance:.0f}m): '
                f'orig_wp={len(wps)}, '
                f'segment-aware={len(corrected_path)} pts, '
                f'modified={is_modified}'
            )

            if not is_modified:
                continue

            # ★★★ 高度剖面擷取（修正繞行段貼地）★★★
            # segment-aware 後的 corrected_path/ref_indices 高度是「正確」的：
            #   繞行(detour)點的 ref 指向巡航高度的目標 waypoint，截斷點指向 OPERATION。
            # 但下方 safety-net / kink-repair / fillet 會重新離散化路徑並丟棄 ref_indices，
            # 之後 ref-mapping 改用「最近原始航點」推高度 → 繞 NFZ 的 detour 點在幾何上
            # 反而最靠近低高度的 takeoff 爬升 / landing 下降航點，繼承到接近地面的高度，
            # 造成「路徑貼地」。故在此先記下 (xy, alt) 剖面，最後用弧長內插還原高度。
            # 巡航高度 = head 航點高度最大值（climb 爬升到它、descent 從它下降）
            _cruise_alt = max(
                (getattr(w, 'alt', 0.0) for w in wps), default=0.0
            )
            _alt_prof_xy = list(corrected_path)
            _alt_prof_z = [
                getattr(wps[r], 'alt', _cruise_alt)
                if 0 <= r < len(wps) else _cruise_alt
                for r in ref_indices
            ] if ref_indices else []

            # ★★★ Safety net：walk 最終 path、抓出任何漏網的穿越段 ★★★
            # 對每對相鄰 (a, b)，若仍穿越 NFZ 內部 → 用 correct_path 局部
            # 補一次 detour（只影響該對，不破壞前面截斷邏輯）。
            # 這保證即使 segment-aware 有遺漏（如端點在 NFZ 邊界、奇數
            # crossings 等 edge case），最終輸出絕無 NFZ 穿越。
            per_uav_planner._rebuild_merged()
            _merged = per_uav_planner._merged_buffered
            if _merged is not None and not _merged.is_empty:
                from shapely.geometry import LineString as _LS
                _polys = (list(_merged.geoms) if _merged.geom_type == 'MultiPolygon'
                          else [_merged])
                repaired_path = [corrected_path[0]]
                n_repairs = 0
                for k in range(1, len(corrected_path)):
                    a = repaired_path[-1]
                    b = corrected_path[k]
                    seg = _LS([a, b])
                    # 是否穿入 polygon 內部（> 0.5m 才算）
                    cross = False
                    for p in _polys:
                        if not seg.intersects(p): continue
                        inter = seg.intersection(p)
                        if inter.is_empty: continue
                        if inter.geom_type in ('Point', 'MultiPoint'): continue
                        if hasattr(inter, 'length') and inter.length > 0.5:
                            cross = True; break
                    if not cross:
                        repaired_path.append(b)
                        continue
                    # 漏網穿越 → 用 correct_path 局部補
                    try:
                        rr = per_uav_planner.correct_path([a, b])
                        if rr.is_modified and rr.corrected_path and len(rr.corrected_path) > 2:
                            for dpt in rr.corrected_path[1:]:
                                repaired_path.append(dpt)
                            n_repairs += 1
                            continue
                    except Exception:
                        pass
                    # 補不出來 — 退而求其次，加 b（可能仍穿越，但至少不更糟）
                    repaired_path.append(b)
                if n_repairs > 0:
                    logger.info(
                        f'[DCCPP-Fence] UAV {uav_id} safety-net: '
                        f'修補 {n_repairs} 段漏網穿越，最終 {len(repaired_path)} 點'
                    )
                    corrected_path = repaired_path
                    ref_indices = None   # safety-net 後 ref 對不上，走 fallback

            # ★ 閉迴圈 kink 修復（pass 1，fillet 前）：清折線往回折尖刺 ★
            # 起飛段穿越 NFZ 時 VG 繞行常產生閃電狀折返；fillet 對 α<1° 的
            # 折返會「直接保留 B 不倒角」，所以尖刺必須在 fillet 前先修掉。
            if len(corrected_path) >= 3:
                try:
                    fixed, clean = self._repair_path_kinks(
                        corrected_path, per_uav_planner,
                    )
                    if len(fixed) != len(corrected_path):
                        logger.info(
                            f'[DCCPP-Fence] UAV {uav_id} kink-repair(前): '
                            f'{len(corrected_path)} → {len(fixed)} 點, '
                            f'clean={clean}'
                        )
                        corrected_path = fixed
                        ref_indices = None
                except Exception as e:
                    logger.warning(
                        f'[DCCPP-Fence] UAV {uav_id} kink-repair(前) 失敗: {e}'
                    )

            # ★ Dubins fillet 倒角：把折線轉角換成 R_min 圓弧 ★
            if _fillet_enabled and len(corrected_path) >= 3:
                try:
                    smoothed = _fillet_planner._apply_fillets(corrected_path)
                    logger.info(
                        f'[DCCPP-Fence] UAV {uav_id} fillet: '
                        f'{len(corrected_path)} → {len(smoothed)} 點 '
                        f'(轉角已用 R_min={r_min:.0f}m 圓弧倒角)'
                    )
                    # fillet 後 ref_indices 對不上長度，用「最近原始 wp」邏輯
                    corrected_path = smoothed
                    ref_indices = None
                except Exception as e:
                    logger.warning(
                        f'[DCCPP-Fence] UAV {uav_id} fillet 失敗（保留原折線）: {e}'
                    )

            # ★ 閉迴圈 kink 修復（pass 2，fillet 後）：最終 gate ★
            # fillet 對極銳角會保留原 B（不倒角），或在 R 縮減處留下殘餘
            # 折返；這裡再跑一次閉迴圈把它們徹底清掉。
            if len(corrected_path) >= 3:
                try:
                    fixed2, clean2 = self._repair_path_kinks(
                        corrected_path, per_uav_planner,
                    )
                    if len(fixed2) != len(corrected_path):
                        logger.info(
                            f'[DCCPP-Fence] UAV {uav_id} kink-repair(後): '
                            f'{len(corrected_path)} → {len(fixed2)} 點, '
                            f'clean={clean2}'
                        )
                        corrected_path = fixed2
                        ref_indices = None
                    if not clean2:
                        logger.warning(
                            f'[DCCPP-Fence] UAV {uav_id} 仍有殘餘 kink '
                            f'(閉迴圈達上限) — 視覺可能仍有小折返'
                        )
                except Exception as e:
                    logger.warning(
                        f'[DCCPP-Fence] UAV {uav_id} kink-repair(後) 失敗: {e}'
                    )

            # ★★★ 最終硬性 NFZ gate（絕不穿越）★★★
            # fillet 倒角會「切入」轉角，可能把貼著 buffered 邊界的路徑切進禁區；
            # 前面的 safety-net 在 fillet *之前* 跑，抓不到 fillet/kink 造成的殘留穿越。
            # 這裡做最終 gate：對最終折線每對相鄰點，若仍穿入 NFZ → correct_path 重繞；
            # 重繞後仍穿越則保留並警告（極少數，通常為端點落在禁區內的任務設計問題）。
            try:
                per_uav_planner._rebuild_merged()
                _mg = per_uav_planner._merged_buffered
                if _mg is not None and not _mg.is_empty and len(corrected_path) >= 2:
                    from shapely.geometry import LineString as _LSg
                    _plg = (list(_mg.geoms) if _mg.geom_type == 'MultiPolygon' else [_mg])

                    def _pen_g(a, b):
                        s = _LSg([a, b])
                        for pg in _plg:
                            if s.intersects(pg):
                                it = s.intersection(pg)
                                if (not it.is_empty
                                        and it.geom_type not in ('Point', 'MultiPoint')
                                        and getattr(it, 'length', 0.0) > 0.5):
                                    return True
                        return False

                    import math as _mg2

                    def _diff_pieces(a, b):
                        """[a,b] 在 NFZ 外的子段，依距 a 排序 → [(near,far), ...]。"""
                        try:
                            outs = _LSg([a, b]).difference(_mg)
                        except Exception:
                            return None
                        if outs is None or outs.is_empty:
                            return []
                        gs = (list(outs.geoms) if outs.geom_type == 'MultiLineString'
                              else [outs])
                        out = []
                        for g in gs:
                            if getattr(g, 'geom_type', '') != 'LineString' or g.length <= 0.5:
                                continue
                            cs = list(g.coords)
                            p0 = (cs[0][0], cs[0][1]); p1 = (cs[-1][0], cs[-1][1])
                            d0 = _mg2.hypot(p0[0]-a[0], p0[1]-a[1])
                            d1 = _mg2.hypot(p1[0]-a[0], p1[1]-a[1])
                            if d1 < d0:
                                p0, p1, d0 = p1, p0, d1
                            out.append((d0, p0, p1))
                        out.sort(key=lambda t: t[0])
                        return out

                    def _walk_boundary(prev, nxt):
                        """沿 buffered NFZ 外緣走較短弧繞過（頂點向外偏移 3m）— 不依賴 VG
                        可視性，對「端點落在邊界」也保證繞開。回傳中繼點（不含端點）。"""
                        from shapely.ops import substring as _sub
                        from shapely.geometry import Point as _Pt
                        best = None
                        for pg in _plg:
                            if not _LSg([prev, nxt]).intersects(pg):
                                continue
                            ring = pg.exterior
                            L = ring.length
                            ta = ring.project(_Pt(prev)); tb = ring.project(_Pt(nxt))
                            lo, hi = (ta, tb) if ta <= tb else (tb, ta)
                            if (hi - lo) <= (L - (hi - lo)):
                                arc = _sub(ring, lo, hi)
                                coords = list(arc.coords)
                                if ta > tb:
                                    coords = coords[::-1]
                            else:
                                a1 = list(_sub(ring, hi, L).coords)
                                a2 = list(_sub(ring, 0, lo).coords)
                                coords = a1 + a2
                                if ta < tb:
                                    coords = coords[::-1]
                            cx, cy = pg.centroid.x, pg.centroid.y
                            mids = []
                            for (x, y) in coords[1:-1]:
                                dx, dy = x - cx, y - cy
                                d = _mg2.hypot(dx, dy) or 1.0
                                mids.append((x + dx / d * 3.0, y + dy / d * 3.0))
                            # 取繞行後不再穿越者
                            if best is None or len(mids) < len(best):
                                best = mids
                        return best or []

                    def _route_gap(prev, nxt):
                        """prev→nxt 若跨 NFZ → 回傳中繼繞行點（不含端點）。
                        先試 correct_path（VG）；失敗則沿外緣走邊界（保證繞開）。"""
                        if not _pen_g(prev, nxt):
                            return []
                        try:
                            rr = per_uav_planner.correct_path([prev, nxt])
                            cp = rr.corrected_path if rr.is_modified else None
                            if (cp and len(cp) >= 2
                                    and not any(_pen_g(cp[t], cp[t+1]) for t in range(len(cp)-1))):
                                return [tuple(p) for p in cp[1:-1]]
                        except Exception:
                            pass
                        # 後備：沿 buffered 外緣走邊界
                        bw = _walk_boundary(prev, nxt)
                        return bw

                    _gated = [corrected_path[0]]
                    _ng = 0
                    for _k in range(1, len(corrected_path)):
                        _a = _gated[-1]
                        _b = corrected_path[_k]
                        if not _pen_g(_a, _b):
                            _gated.append(_b)
                            continue
                        # 穿越 → difference 裁成 NFZ 外子段（端點落邊界、非 buffer 內部），
                        # 缺口在邊界點之間用 correct_path 繞行（此時 VG 可運作）。
                        _pcs = _diff_pieces(_a, _b)
                        if not _pcs:
                            # 整段在 NFZ 內（兩端皆內部，極少）→ 丟棄此段、保留連通性交給下一段
                            continue
                        _ng += 1
                        for (_, _p0, _p1) in _pcs:
                            _prev = _gated[-1]
                            if _mg2.hypot(_p0[0]-_prev[0], _p0[1]-_prev[1]) > 0.5:
                                _mid = _route_gap(_prev, _p0)
                                if _mid is not None:
                                    _gated.extend(_mid)
                                # _mid is None（繞行失敗）→ 直接接 _p0（極少數，仍貼邊不深入）
                                _gated.append(_p0)
                            _gated.append(_p1)
                    if _ng > 0:
                        logger.info(
                            f'[DCCPP-Fence] UAV {uav_id} 最終 NFZ gate：'
                            f'重繞 {_ng} 段 fillet 後殘留穿越'
                        )
                        corrected_path = _gated
                        ref_indices = None
            except Exception as e:
                logger.warning(f'[DCCPP-Fence] UAV {uav_id} 最終 NFZ gate 失敗: {e}')

            # 高度用「弧長內插」還原（修正繞行段貼地）：
            #   以 fillet 前擷取的正確高度剖面 (_alt_prof_xy/_z) 為基準，沿累積弧長
            #   比例重採樣到最終 corrected_path。這樣繞 NFZ 的 detour 點維持巡航高度，
            #   而非被「最近原始航點」誤判繼承到 takeoff/landing 的低高度。
            interp_alts = self._resample_alts_by_arclength(
                corrected_path, _alt_prof_xy, _alt_prof_z,
            ) if _alt_prof_z else None

            # 為每個新 metric 點找出「對應的原 waypoint」
            #   有 ref_indices → 直接使用（segment_type 等屬性更準確）
            #   否則（fillet 後）→ 用「最近原始 wp」fallback（僅用於 segment_type/heading 等屬性，
            #   高度一律改用上面的弧長內插結果）
            new_wps: list = []
            oi = 0
            for k, (nx, ny) in enumerate(corrected_path):
                if ref_indices is not None and k < len(ref_indices):
                    ref = wps[ref_indices[k]]
                else:
                    while oi < len(metric_wps) - 1:
                        dx0 = metric_wps[oi][0] - nx
                        dy0 = metric_wps[oi][1] - ny
                        dx1 = metric_wps[oi + 1][0] - nx
                        dy1 = metric_wps[oi + 1][1] - ny
                        if (dx1 * dx1 + dy1 * dy1) < (dx0 * dx0 + dy0 * dy0):
                            oi += 1
                        else:
                            break
                    ref = wps[oi]
                lat, lon = to_latlon(nx, ny)
                # 複製原 wp，只覆寫 lat / lon — 不論底下是 BuiltWaypoint
                # 或 AssembledWaypoint 或其他型別都通用
                new_wp = _copy.copy(ref)
                try:
                    new_wp.lat = lat
                    new_wp.lon = lon
                except AttributeError:
                    # 萬一是 frozen dataclass，退回 dataclasses.replace
                    import dataclasses as _dc
                    new_wp = _dc.replace(ref, lat=lat, lon=lon)
                # 高度改用弧長內插（避免繞行段貼地）
                if interp_alts is not None and k < len(interp_alts):
                    try:
                        new_wp.alt = interp_alts[k]
                    except AttributeError:
                        import dataclasses as _dc
                        new_wp = _dc.replace(new_wp, alt=interp_alts[k])
                new_wps.append(new_wp)
            # ★ 下降進場型態原樣接回（避障豁免）— 確保匯出時仍有 LANDING → NAV_LAND；
            #   回程巡航段（到場周入口）已在上方 new_wps 中完成 NFZ 繞行。
            apath.waypoints = takeoff_head + new_wps + landing_tail
            n_rerouted += 1

        # ── 統一摘要 ──
        logger.info(
            f'[DCCPP-Fence] 完成 — {n_rerouted}/{len(assembled)} 機已繞行 '
            f'{added_count} 區 (per-UAV 扇形展開 + Dubins fillet 平滑)'
        )

        if n_rerouted > 0:
            self.statusBar().showMessage(
                f'[DCCPP] {n_rerouted}/{len(assembled)} 機已繞行 '
                f'{added_count} 個禁區 (buffer={actual_buffer:.0f}m, '
                f'扇形 +{spread_per_uav_m:.0f}m/機, fillet)',
                6000,
            )
        else:
            # 有區域卻沒繞 → 強制彈警告
            from PyQt6.QtWidgets import QMessageBox
            msg = (
                f'⚠ DCCPP 規劃完成，但 {added_count} 個 fence 禁區都沒影響到路徑。\n\n'
                f'可能原因：\n'
                f'  • 路徑沒有真的穿越禁區（規劃結果剛好都在禁區外）\n'
                f'  • 規劃覆蓋區與禁區完全不重疊\n'
                f'  • Segment-aware avoidance 偵測不到穿越（buffer 過小？）\n'
            )
            msg += '請檢查 log 取得詳細診斷。'
            QMessageBox.warning(self, 'Fence 繞行未生效', msg)

    def _on_dccpp_coverage_requested(self, params: dict, *, run_async: bool = True,
                                     on_done=None):
        """
        處理 DCCPP 最佳化覆蓋請求

        流程：
        1. 驗證邊界點 >= 3
        2. 組裝 areas / drones 資料結構
        3. 呼叫 SwarmCoordinator.plan_coverage_dccpp()
        4. 將結果轉為 swarm_raw 格式並在地圖顯示（`_display_dccpp_result`）
        5. 更新狀態列

        Args:
            params:    面板送來的參數字典（ParameterPanel.get_dccpp_params 或
                       VTOLReconPanel.get_params）
            run_async: HV-DCCPP 分支是否丟到背景執行緒。
                       ⚠ 呼叫端若在本函式回來後立刻讀 `self._dccpp_result`（舊寫法），
                       **必須**傳 False，否則會讀到上一次的結果或 None 而且不會報錯。
                       新的呼叫端請改用 `on_done`。
            on_done:   `on_done(ok: bool)`，在**結果已經顯示完**之後呼叫一次。
                       同步路徑當場叫、背景路徑等 worker 回來才叫；失敗／取消時
                       以 `ok=False` 呼叫 —— 呼叫端才不會永遠停在「規劃中…」。
        """
        from PyQt6.QtWidgets import QMessageBox
        try:
            # ⚠ 多區模式：已完成的區也算數。只看 `self.corners` 的話，
            #   使用者按了「完成本區」之後畫布是空的，就會被誤擋。
            if not self.survey_area_polygons():
                QMessageBox.warning(
                    self, "邊界點不足",
                    "請先在地圖上設定至少 3 個邊界點，再執行 DCCPP 最佳化。\n"
                    "（多區模式：按「完成本區」收起來的區也算。）"
                )
                if on_done:
                    on_done(False)
                return

            # ⚠ 這道閘門必須在**清地圖之前**，而且對每一種規劃都要問：
            #   背景規劃還在跑時再送一次（不論是不是 HV），下面的
            #   clear_all_display_layers_js() 會先把圖層清掉，接著舊的那趟
            #   完成時又用舊結果覆蓋 self._dccpp_result —— 畫面與匯出全部對不上，
            #   而且不會有任何錯誤訊息。舊版是靠 UI 凍結擋住這件事的。
            if self._hv_planning_in_flight():
                QMessageBox.information(
                    self, "HV-DCCPP",
                    "背景規劃仍在執行中，請等它結束再送出新的規劃。\n"
                    "（規劃無法中途取消：hv 的求解器沒有取消點）")
                if on_done:
                    on_done(False)
                return

            import math

            num_drones    = int(params.get('num_drones', 3))
            fov_width_m   = float(params.get('fov_width_m', 100.0))
            overlap_rate  = float(params.get('overlap_rate', 0.1))
            altitude      = float(params.get('altitude', 100.0))
            speed         = float(params.get('speed', 15.0))
            turn_radius   = float(params.get('turn_radius', 0.0))
            vehicle_type  = params.get('vehicle_type', 'multirotor')
            auto_scan     = bool(params.get('auto_scan_angle', True))
            enable_alt    = bool(params.get('enable_altitude', True))
            mounting_angle = float(params.get('mounting_angle_deg', 0.0))
            coord_mode    = params.get('coordination_mode', 'uncoordinated')
            dem_path      = params.get('dem_path', '')
            # ⚠⚠⚠ 「VTOL 偵查」頁的 get_params() **沒有 dem_path 這個鍵**
            #   （DEM 選檔在「參數」頁，而那一頁從不設 hv_dccpp）⇒ HV 規劃一律
            #   拿到空字串 ⇒ terrain_func=None ⇒ 轉接層把地形當成海平面 0 m 的平面。
            #   後果：對地淨空與下降走廊的地形檢查**結構上永遠不會觸發**。
            #   使用者已經載入 DEM 時就把它接回來（`_on_dem_loaded` 存的那一份）。
            if not dem_path:
                _dm = getattr(self, '_dem_manager', None)
                _p = getattr(_dm, 'dem_path', None) if _dm is not None else None
                if _p:
                    dem_path = str(_p)
                    logger.info(f'覆蓋規劃：面板沒帶 dem_path，改用已載入的 DEM {dem_path}')
                elif params.get('hv_dccpp'):
                    logger.warning(
                        '覆蓋規劃：沒有任何 DEM ⇒ HV-DCCPP 會把地形當成海平面 0 m 的'
                        '平面，對地淨空與下降走廊的地形檢查不會生效')

            # ── 組裝區域 ──
            # ⚠ `scan_angle_deg` 一定要放進來：規劃器在 auto_scan_angle=False 時
            #   走 `area_dict.get('scan_angle_deg', 0.0)`。舊版沒帶這個鍵 ⇒ 就算
            #   關掉自動選角，掃描方向也永遠是 0°，使用者改角度完全沒反應。
            scan_angle_deg = float(params.get('scan_angle_deg', 0.0))
            # 模式二：使用者可以圈**好幾塊**小區。`survey_areas` 是已完成的那些，
            # `corners` 是正在畫的那一塊（沒畫完就不算）。兩者都沒有時退回舊行為。
            _polys = self.survey_area_polygons()
            areas = [{'area_id': i + 1, 'polygon': list(p), 'priority': 1.0,
                      'scan_angle_deg': scan_angle_deg}
                     for i, p in enumerate(_polys)]
            if len(areas) > 1:
                logger.info(f'[DCCPP] 多區模式：{len(areas)} 塊作業區'
                            f'（各 {[len(p) for p in _polys]} 個角點）')
            if not auto_scan:
                logger.info(
                    f'[DCCPP] 手動掃描角度 {scan_angle_deg:.1f}°（0°=北，順時針）')

            # ── 組裝無人機 ──
            # ⚠ 質心要取**所有**區的角點，不是只取正在畫的那一塊，否則多區時
            #   沒設起飛點的話會把 home 放在最後一塊區的中心。
            _all_pts = [c for p in _polys for c in p]
            centroid_lat = sum(c[0] for c in _all_pts) / len(_all_pts)
            centroid_lon = sum(c[1] for c in _all_pts) / len(_all_pts)
            # 優先使用使用者設定的起飛點，否則用多邊形質心
            if self.home_point is not None:
                base_lat, base_lon = self.home_point
            else:
                base_lat, base_lon = centroid_lat, centroid_lon
            user_takeoff_hdg = float(params.get('takeoff_bearing', 0.0))
            takeoff_spacing_m = resolve_spawn_spacing_m(params, logger)
            _spawn_pts = spawn_positions(
                base_lat, base_lon, num_drones, user_takeoff_hdg, takeoff_spacing_m,
            )
            drones = []
            for i in range(num_drones):
                # 生成點幾何見模組層級的 spawn_positions()（有值層級守衛）
                pos = _spawn_pts[i]
                drones.append({
                    'drone_id': i + 1,
                    'name': f'UAV-{i + 1}',
                    'position': pos,
                    'speed': speed,
                    'heading': user_takeoff_hdg,
                    'turn_radius': turn_radius,
                    'vehicle_type': vehicle_type,
                    'fov_width': fov_width_m,
                })

            # ── 清除舊路徑（JS 方式，不重載頁面），避免與新 DCCPP 路徑重疊 ──
            self.map_widget.clear_all_display_layers_js()

            self.statusBar().showMessage("DCCPP 最佳化計算中…", 0)
            from PyQt6.QtWidgets import QApplication
            QApplication.processEvents()

            # ── 呼叫 DCCPP 求解器 ──
            coordinator = SwarmCoordinator()
            user_runway_length = float(params.get('runway_length', 500.0))
            user_landing_hdg = float(params.get('landing_bearing', 180.0))
            # 降落滑行距離：將觸地點沿進場反方向提前此距離，
            # 確保 DCCPP 規劃的降落點不會和起飛點重疊
            user_landing_rollout = float(params.get('landing_rollout', 0.0))
            _is_vtol = str(vehicle_type).lower() == 'vtol'
            plan_kwargs = dict(
                areas=areas,
                drones=drones,
                coverage_width_m=fov_width_m,
                overlap_rate=overlap_rate,
                auto_scan_angle=auto_scan,
                coverage_altitude=altitude,
                terrain_func=None,
                enable_altitude=enable_alt,
                mounting_angle_deg=mounting_angle,
                dem_path=dem_path,
                coordination_mode=coord_mode,
                runway_bearing_deg=user_takeoff_hdg,
                runway_length_m=user_runway_length,
                landing_bearing_deg=user_landing_hdg,
                landing_rollout_m=user_landing_rollout,
                # VTOL：把「轉換高度」往下傳，起降才會是垂直剖面而不是跑道滑行。
                # 非 VTOL 時為 None，走原本的固定翼跑道起降。
                vtol_transition_alt_m=(
                    float(params.get('transition_alt', 0.0) or 0.0)
                    if _is_vtol else None
                ),
            )

            # 顯示階段要用到、但在規劃前就決定好的值。
            # ⚠ corners_snapshot 一定要在這裡拍：規劃跑在背景執行緒時使用者仍可
            #   在地圖上改邊界點，事後再讀 self.corners 會畫出與實際飛的路徑不一致
            #   的多邊形與面積，而且完全不會報錯。
            display_ctx = dict(
                params=params, corners_snapshot=list(self.corners),
                centroid_lat=centroid_lat, centroid_lon=centroid_lon,
                altitude=altitude, speed=speed, turn_radius=turn_radius,
                vehicle_type=vehicle_type, auto_scan=auto_scan,
            )

            # ── HV-DCCPP（異構雙構型 VTOL）分支 ──
            #   面板勾選且 vehicle_type == 'vtol' 才接手；其餘一律走既有 DCCPP。
            #   見 docs/AEROPLAN_接入手冊.md §5 與 docs/adr/0007-hv-dccpp-planner.md。
            if self._configure_hv_dccpp(coordinator, params, vehicle_type) and run_async:
                self._hv_on_done = on_done
                self._start_hv_planning(coordinator, plan_kwargs, display_ctx)
                return          # ★ 後處理與 on_done 改由 finished_ok 觸發

            dccpp_result = coordinator.plan_coverage_dccpp(**plan_kwargs)
            # HV 分支同步跑時也要把轉接器留下來（匯出／上傳／重規劃要用）
            self._hv_coordinator = getattr(coordinator, 'hv_coordinator', None)
            self._display_dccpp_result(dccpp_result, **display_ctx)
            if on_done:
                on_done(True)

        except Exception as e:
            logger.error(f"DCCPP 最佳化失敗: {e}", exc_info=True)
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "DCCPP 失敗", f"DCCPP 最佳化路徑規劃失敗：\n{e}")
            if on_done:
                on_done(False)

    def _display_dccpp_result(self, dccpp_result: dict, *, params: dict,
                              corners_snapshot: list,
                              centroid_lat: float, centroid_lon: float,
                              altitude: float, speed: float, turn_radius: float,
                              vehicle_type: str, auto_scan: bool):
        """把 DCCPP 規劃結果套用到 UI：避讓後處理、地圖、統計、metrics。

        由 `_on_dccpp_coverage_requested` 直接呼叫（同步路徑），或由
        `HvPlanningWorker.finished_ok` 觸發（HV-DCCPP 背景執行緒路徑）。
        內容是原本寫在 handler 裡的後處理，**逐行照搬**，只有兩處改動：
          * 兩次 `list(self.corners)` 改讀 `corners_snapshot`（規劃前拍的快照）；
          * 自己帶一個 try/except，讓「規劃失敗」與「顯示失敗」不會互相掩蓋。

        ⚠ 這裡的每一件事都碰 widget，只能在 GUI 執行緒跑。
        """
        import math          # noqa: F401 — 巢狀 _haversine 用得到
        from PyQt6.QtWidgets import QMessageBox
        try:
            # 註：fence-zone 避讓延後到「起降航段組裝完成後」才執行，
            # 否則 takeoff/landing 加上去的 home ↔ scan-area 連線會繞過避讓
            # 檢查（穿越 NFZ）。見下方 auto_landing 區塊之後的呼叫。

            # 儲存 DCCPP 參數供匯出時使用
            self.flight_params.update({
                'vehicle_type': vehicle_type,
                'altitude': altitude,
                'speed': speed,
                'turn_radius': turn_radius,
            })

            # ── 將 DCCPP 結果轉為 display_swarm_raw 格式 ──
            # 把規劃器**實際**用的掃描角回報給面板。自動模式下使用者才看得到
            # 幾何選了幾度，取消自動時也才有一個合理的起跳值。
            # ⚠ 只有 DCCPP v2 管線會給這個 key；回退到舊 solver 時拿不到，
            #   那就維持面板原顯示，不要自己重算一個數字充當（會變兩套值）。
            _eff = (dccpp_result.get('effective_scan_angles') or {}).get(1)
            if _eff is not None and hasattr(self, 'vtol_recon_panel'):
                self.vtol_recon_panel.set_effective_scan_angle(_eff, auto=auto_scan)
                logger.info(
                    f'[DCCPP] 實際生效掃描角 {_eff:.1f}°'
                    f'（{"自動" if auto_scan else "手動"}）')

            results = dccpp_result.get('results', {})
            total_makespan = dccpp_result.get('total_makespan', 0.0)
            allocation = dccpp_result.get('allocation', {})
            assembled_paths = dccpp_result.get('assembled_paths', None)

            # ── 為固定翼 UAV 添加起飛 / 降落路徑（受 auto_landing 開關控制）──
            auto_landing = bool(params.get('auto_landing', True))
            # 新版 swarm_coordinator 已在 BuiltPath 中自動補上 TAKEOFF/LANDING；
            # 偵測到第一個航點已是 TAKEOFF 時跳過舊 assembler 補丁，避免重複。
            _already_has_to_ld = bool(
                assembled_paths
                and any(
                    apath.waypoints
                    and getattr(apath.waypoints[0].segment_type, 'name', '') == 'TAKEOFF'
                    for apath in assembled_paths.values()
                )
            )
            # VTOL 不需要跑道式起降，垂直起降由匯出器處理
            if (auto_landing and assembled_paths and not _already_has_to_ld
                    and vehicle_type == 'fixed_wing' and turn_radius > 0):
                from core.trajectory.dccpp_path_assembler import DCCPPPathAssembler as _Asm
                _asm = _Asm(
                    default_altitude=altitude,
                    turn_radius=turn_radius,
                )
                # 使用起飛點或多邊形質心作為 Home
                home = self.home_point if self.home_point else (centroid_lat, centroid_lon)
                # 使用使用者設定的跑道參數
                user_takeoff_hdg = float(params.get('takeoff_bearing', 0.0))
                user_landing_hdg = float(params.get('landing_bearing', 180.0))
                user_pattern_alt = float(params.get('pattern_alt', 80.0))
                user_landing_rollout = float(params.get('landing_rollout', 0.0))
                for uav_id, apath in assembled_paths.items():
                    if not apath.waypoints:
                        continue
                    _asm.prepend_takeoff(
                        apath, home[0], home[1],
                        takeoff_bearing_deg=user_takeoff_hdg,
                        climb_alt_m=min(30.0, altitude * 0.3),
                    )
                    _asm.append_landing(
                        apath, home[0], home[1],
                        landing_bearing_deg=user_landing_hdg,
                        pattern_alt_m=user_pattern_alt,
                        landing_rollout_m=user_landing_rollout,
                    )

            # ── Fence-zone 後處理 — 繞行所有 NFZ + 威脅區 ──
            # 關鍵：必須在「起降航段組裝完成後」執行。否則 home ↔ 進場/離場
            # 連線（剛剛 prepend/append 加上去的）會跳過避讓檢查，造成
            # 「fence avoidance 跑了但飛機還是穿越 NFZ」的 bug。
            # 多旋翼 / VTOL 不走上方 auto_landing 分支，但此處仍會處理它們的
            # operation 航段繞行。
            self._apply_fence_avoidance_to_dccpp_result(
                dccpp_result, centroid_lat, centroid_lon, turn_radius,
            )

            # ── 避撞偵測與解衝突 ──
            _collision_enabled = bool(params.get('collision_avoidance', False))
            _collision_conflicts = []
            _collision_actions = []
            if _collision_enabled and assembled_paths and len(assembled_paths) >= 2:
                try:
                    from core.dccpp.collision_avoidance import CollisionAvoidance
                    _ca = CollisionAvoidance(
                        **build_avoidance_kwargs(params, speed))
                    _collision_conflicts, _collision_actions = _ca.run(assembled_paths)
                    if _collision_conflicts:
                        self.statusBar().showMessage(
                            f"避撞偵測: {len(_collision_conflicts)} 個衝突, "
                            f"已執行 {len(_collision_actions)} 個避撞動作",
                            5000,
                        )
                except Exception as _ca_err:
                    logger.warning(f"避撞模組失敗: {_ca_err}", exc_info=True)

            drones_data = []
            total_wps = 0
            total_dist = 0.0

            # ── 優先使用 assembled_paths（含 Dubins 曲線）──
            if assembled_paths:
                for uav_id, apath in assembled_paths.items():
                    # 按 SegmentLabel 分組：起飛/進入/作業歸 operation_paths，轉移/降落歸 transfer_paths
                    takeoff_pts = []
                    entry_pts = []
                    op_pts = []
                    xfer_pts = []
                    landing_pts = []
                    current_seg = []
                    current_label = None

                    for wp in apath.waypoints:
                        pt = (wp.lat, wp.lon, getattr(wp, 'alt', altitude))
                        if wp.segment_type != current_label:
                            if current_seg and current_label is not None:
                                lname = current_label.name
                                if lname == 'TAKEOFF':
                                    takeoff_pts.append(current_seg)
                                elif lname == 'ENTRY':
                                    entry_pts.append(current_seg)
                                elif lname == 'OPERATION':
                                    op_pts.append(current_seg)
                                elif lname == 'TRANSFER':
                                    xfer_pts.append(current_seg)
                                elif lname == 'LANDING':
                                    landing_pts.append(current_seg)
                            current_seg = [pt]
                            current_label = wp.segment_type
                        else:
                            current_seg.append(pt)
                    # flush last segment
                    if current_seg and current_label is not None:
                        lname = current_label.name
                        if lname == 'TAKEOFF':
                            takeoff_pts.append(current_seg)
                        elif lname == 'ENTRY':
                            entry_pts.append(current_seg)
                        elif lname == 'OPERATION':
                            op_pts.append(current_seg)
                        elif lname == 'TRANSFER':
                            xfer_pts.append(current_seg)
                        elif lname == 'LANDING':
                            landing_pts.append(current_seg)

                    # 各段分別保留，讓 map_widget 以不同線型/顏色繪製
                    total_wps += len(apath.waypoints)
                    start_pos = (apath.waypoints[0].lat, apath.waypoints[0].lon) if apath.waypoints else (centroid_lat, centroid_lon)
                    # 構型（只有 HV-DCCPP 有）：讓 2D/3D 一眼看得出哪幾架是
                    # Pioneer、哪幾架是 HERO —— 兩者的轉彎半徑與作業高度差很多，
                    # 混在一起看只會看到「一堆顏色」。同構規劃時是 None，畫法不變。
                    _cls = getattr(apath, 'cls', None)
                    drones_data.append({
                        'drone_id': uav_id,
                        'start_position': start_pos,
                        'takeoff_paths': takeoff_pts,
                        'entry_paths': entry_pts,
                        'operation_paths': op_pts,
                        'transfer_paths': xfer_pts,
                        'landing_paths': landing_pts,
                        'uav_class': _cls,
                        'platform': HV_PLATFORM_NAMES.get(_cls or '', ''),
                    })
            else:
                # ── 備用：從 MDTSPResult 提取 ──
                for area_id, mdtsp_result in results.items():
                    assigned_uavs = allocation.get(area_id, [])
                    if hasattr(mdtsp_result, 'uav_paths') and mdtsp_result.uav_paths:
                        for uav_id, path_seq in mdtsp_result.uav_paths.items():
                            op_paths = []
                            xfer_paths = []
                            for seg in path_seq:
                                if hasattr(seg, 'waypoints') and seg.waypoints:
                                    op_paths.append(seg.waypoints)
                                    total_wps += len(seg.waypoints)
                                elif isinstance(seg, (list, tuple)):
                                    if len(seg) >= 2:
                                        op_paths.append(list(seg))
                                        total_wps += len(seg)

                            start_pos = op_paths[0][0] if op_paths and op_paths[0] else (centroid_lat, centroid_lon)
                            drones_data.append({
                                'drone_id': uav_id,
                                'start_position': start_pos,
                                'operation_paths': op_paths,
                                'transfer_paths': xfer_paths,
                            })
                    elif hasattr(mdtsp_result, 'ordered_segments'):
                        for idx, uav_id in enumerate(assigned_uavs):
                            seg_per_uav = []
                            for seg in mdtsp_result.ordered_segments:
                                pts = []
                                if hasattr(seg, 'left_point') and hasattr(seg, 'right_point'):
                                    pts = [seg.left_point, seg.right_point]
                                elif isinstance(seg, (list, tuple)) and len(seg) >= 2:
                                    pts = list(seg)
                                if pts:
                                    seg_per_uav.append(pts)
                                    total_wps += len(pts)

                            if seg_per_uav:
                                start_pos = seg_per_uav[0][0]
                                drones_data.append({
                                    'drone_id': uav_id,
                                    'start_position': start_pos,
                                    'operation_paths': seg_per_uav,
                                    'transfer_paths': [],
                                })

            # 計算總距離
            def _haversine(a, b):
                """兩點 3D 距離（含高度差）。

                點的格式是 (lat, lon, alt)；只算水平的話，VTOL 的垂直起降段
                會被當成 0 公尺。
                """
                dlat = (b[0] - a[0]) * 111320
                dlon = (b[1] - a[1]) * 111320 * math.cos(math.radians(a[0]))
                dz = (b[2] - a[2]) if (len(a) > 2 and len(b) > 2) else 0.0
                return math.hypot(math.hypot(dlat, dlon), dz)

            # ⚠ 五個航段都要算。原本只加 operation_paths ⇒「飛行距離」只是掃描
            #   條帶的長度，與表格的總航程對不起來（實測 45.6 km vs 66.0 km），
            #   而且起降、進場、轉場全部被當成沒飛。
            for dd in drones_data:
                for _key in ('takeoff_paths', 'entry_paths', 'operation_paths',
                             'transfer_paths', 'landing_paths'):
                    for path in dd.get(_key, []):
                        for j in range(len(path) - 1):
                            total_dist += _haversine(path[j], path[j + 1])

            # 衝突標記
            conflict_markers = []
            if _collision_conflicts:
                for cf in _collision_conflicts:
                    conflict_markers.append({
                        'lat': cf.pos_a[0], 'lon': cf.pos_a[1],
                        'min_dist': cf.min_dist_m,
                        'duration': cf.duration_s,
                        'uav_a': cf.uav_a, 'uav_b': cf.uav_b,
                    })

            swarm_data = {
                'drones': drones_data,
                'areas': [{'area_id': 1, 'polygon': list(corners_snapshot)}],
                'stats': {
                    'total_drones': len(drones_data),
                    'total_waypoints': total_wps,
                    'total_distance': total_dist,
                    'estimated_time': total_makespan,
                    'conflicts': len(_collision_conflicts),
                    'avoidance_actions': len(_collision_actions),
                },
                'conflicts': conflict_markers,
            }
            # 把**權威**載具類型記進結果裡。SITL 啟動時要用這個值，不要再從
            # 「航點裡有沒有 TAKEOFF 段」反推 —— 那個啟發式沒有 TAKEOFF 就一律
            # 判成 COPTER，會把使用者選的 PLANE 悄悄改掉。
            dccpp_result['vehicle_type'] = vehicle_type
            # 把這次規劃**實際使用**的 VTOL 轉換高度一起記下來。
            # 上傳飛控時必須用同一個值，否則上傳的任務與畫面上的軌跡不一致
            # （舊行為是去讀 DCCPP 分頁的 spin box，VTOL 偵察頁設的值被忽略）。
            if vehicle_type == 'vtol':
                dccpp_result['vtol_transition_alt_m'] = float(
                    params.get('transition_alt', 0.0) or 0.0
                )
            # 同理，速度與迴轉半徑也要記「這次規劃實際用的值」。
            # 舊行為：上傳飛控去讀 parameter_panel（**基本演算法**分頁）。用
            # VTOL 偵察頁規劃時那個 panel 可能從沒被碰過，speed 還停在初始預設
            # 3.0 m/s ⇒ 匯出的檔寫 18 m/s、上傳的任務卻是 3 m/s，同一架飛機
            # 兩份互相矛盾的任務。
            dccpp_result['cruise_speed_mps'] = float(speed)
            dccpp_result['turn_radius_m'] = float(turn_radius)
            self._dccpp_result = dccpp_result
            # 圍籬對齊規劃器**實際**用的掃描角（`_eff`，羅盤 0=北順時針，
            # 與 GeofenceConstraintManager 同慣例）。
            # ⚠ 回退到舊 solver 時 `_eff` 是 None —— 必須原樣傳下去。寫成
            #   `_eff or 0.0` 會讓每一趟退回的航次都建出**正北**對齊的圍籬，
            #   而畫面上跟正確的圍籬長得一模一樣，看不出來。
            self.map_widget.display_swarm_raw(swarm_data, scan_angle_deg=_eff)

            vtype_label = {"fixed_wing": "固定翼", "vtol": "VTOL (4+1)", "multirotor": "多旋翼"}.get(vehicle_type, "多旋翼")

            # ── 更新任務統計面板 ──
            try:
                _area_m2 = CoveragePlanner().calculate_coverage_area(list(corners_snapshot))
            except Exception:
                _area_m2 = 0.0
            self.waypoint_label.setText(f"航點: {total_wps}")
            self.distance_label.setText(f"距離: {total_dist:.0f}m")
            self.mission_panel.update_mission_stats({
                'waypoint_count': total_wps,
                'total_distance': total_dist,
                'estimated_time': total_makespan,
                'area': _area_m2,
                'regions': len(drones_data),
            })

            self.statusBar().showMessage(
                f"DCCPP 最佳化完成 | {vtype_label} × {len(drones_data)} 台 | "
                f"共 {total_wps} 航點 | 總距離 {total_dist:.0f}m | "
                f"Makespan {total_makespan:.1f}s ({total_makespan / 60:.1f}min)",
                10000
            )
            logger.info(
                f"DCCPP 最佳化完成: {len(drones_data)} 台 {vtype_label} | "
                f"makespan={total_makespan:.1f}s"
            )

            # 儲存 DCCPP metrics 供比較
            self._dccpp_metrics = {
                'algorithm': 'DCCPP (最佳化)',
                'num_drones': len(drones_data),
                'vehicle_type': vtype_label,
                'total_waypoints': total_wps,
                'total_distance': total_dist,
                'estimated_time': total_makespan,
                'makespan': total_makespan,
            }
        except Exception as e:
            logger.error(f"DCCPP 結果顯示失敗: {e}", exc_info=True)
            QMessageBox.warning(self, "DCCPP 失敗", f"DCCPP 最佳化路徑規劃失敗：\n{e}")

    # ══════════════════════════════════════════════════════════════════
    # HV-DCCPP（異構雙構型 VTOL 覆蓋規劃）
    #   套件 hv_dccpp；接法與參數對照見 docs/AEROPLAN_接入手冊.md，
    #   決策見 docs/adr/0007-hv-dccpp-planner.md、0008-planning-in-worker-thread.md。
    # ══════════════════════════════════════════════════════════════════
    def _hv_class_of(self, drone: dict) -> str:
        """逐機構型 A/B。

        ⚠ 這個回呼**不能省**：面板組出來的 drone dict 只有 drone_id / name /
        position / speed / heading / turn_radius / vehicle_type / fov_width，
        沒有任何構型線索，轉接層的 default_class_of 最後一條規則會把整隊當成
        構型 A（Pioneer，24.6 kg、200 m 轉彎半徑、120 m 最低高度）——
        結果完全錯誤而且不會報錯。
        """
        return (getattr(self, '_hv_classes', None) or {}).get(int(drone.get('drone_id', 0)), 'A')

    def _hv_fence_zones(self):
        """禁飛區 + 威脅區（GEOFENCE 是 inclusion，語意相反，不能送給 hv）。"""
        try:
            from mission.fence_zone import FenceZoneRegistry, ZoneCategory
            reg = FenceZoneRegistry.instance()
            return reg.by_category(ZoneCategory.NFZ) + reg.by_category(ZoneCategory.THREAT)
        except Exception as e:      # noqa: BLE001
            logger.warning(f'[HV-DCCPP] 取不到 fence zone：{e}')
            return []

    def _configure_hv_dccpp(self, coordinator, params: dict, vehicle_type: str) -> bool:
        """決定這一次規劃要不要走 HV-DCCPP，要的話把旗標設到 coordinator 上。

        Returns:
            True 表示已切到 HV 分支（coordinator.use_hv_dccpp 已打開）。
        """
        enabled = (str(vehicle_type).lower() == 'vtol'
                   and bool(params.get('hv_dccpp', False)))
        coordinator.use_hv_dccpp = enabled
        if not enabled:
            self._hv_classes = {}
            return False

        # 兩種構型的台數是各自獨立填的（面板「機隊組成」）；總台數 = 兩者相加。
        # 舊版只有 hv_num_a、B 靠總數相減，這裡兩個都收，對不起來時以相加為準，
        # 免得畫面上的台數與實際規劃的機隊不一致而且沒人看得出來。
        n_a = max(0, int(params.get('hv_num_a', 1) or 0))
        n_b = params.get('hv_num_b')
        n_total = int(params.get('num_drones', 0) or 0)
        if n_b is None:
            n_a = min(n_a, n_total)
            n_b = max(0, n_total - n_a)
        else:
            n_b = max(0, int(n_b))
            if n_a + n_b != n_total:
                logger.warning(
                    f'[HV-DCCPP] 台數對不上：Pioneer {n_a} + HERO {n_b} = {n_a + n_b}，'
                    f'但 num_drones={n_total}；以相加值為準')
        # {drone_id: 'A'|'B'} — 前 n_a 台是構型 A（Pioneer + LiDAR），其餘是 B（HERO + 相機）
        self._hv_classes = {i + 1: ('A' if i < n_a else 'B')
                            for i in range(n_a + n_b)}

        coordinator.hv_class_of = self._hv_class_of
        coordinator.hv_params = dict(params)
        coordinator.hv_home = self.home_point
        coordinator.hv_alternates = getattr(self, '_hv_alternates', None)
        coordinator.hv_wind = params.get('hv_wind')
        # ★ 在 GUI 執行緒先把禁飛區取好再交給規劃器：FenceZoneRegistry 是主執行緒的
        #   QObject 單例，規劃跑在背景執行緒時不該由那邊去讀它。
        coordinator.hv_fence_zones = self._hv_fence_zones()
        logger.info(
            f'[HV-DCCPP] 啟用：構型 A {n_a} 台 / 構型 B {n_total - n_a} 台，'
            f'home={self.home_point}, 風={params.get("hv_wind")}, '
            f'C2={params.get("hv_c2_range_km")} km')
        if not self._hv_classes:
            logger.warning('[HV-DCCPP] num_drones=0，沒有任何飛機可規劃')
        return True

    def _hv_planning_in_flight(self) -> bool:
        """背景規劃是否還在跑（任何規劃請求都要先問這一句）。"""
        worker = getattr(self, '_hv_worker', None)
        return worker is not None and worker.isRunning()

    def _hv_disconnect_worker(self, worker) -> None:
        """把 worker 的四條信號從本視窗拆掉（視窗要關、或要換一個 worker 時）。"""
        for sig in ('progress', 'report', 'failed', 'finished_ok', 'cancelled'):
            try:
                getattr(worker, sig).disconnect()
            except (TypeError, RuntimeError):
                pass          # 本來就沒接 / C++ 物件已消失

    def _start_hv_planning(self, coordinator, plan_kwargs: dict, display_ctx: dict) -> None:
        """把 HV-DCCPP 規劃丟到背景執行緒（大情境數十秒，同步跑會讓視窗「沒有回應」）。"""
        from ui.workers.planning_worker import HvPlanningWorker

        if self._hv_planning_in_flight():
            # 正常情況下 `_on_dccpp_coverage_requested` 的閘門已經先擋掉了；
            # 這裡是防止有人日後從別的地方直接呼叫進來。
            logger.warning('[HV-DCCPP] 已有規劃在執行，忽略這次請求')
            return

        old = getattr(self, '_hv_worker', None)
        if old is not None:
            # 上一趟的 worker：拆信號並交給 Qt 回收，否則每規劃一次就多留一個
            # QThread（連同它的 coordinator、地形網格與 CostCache）在記憶體裡。
            self._hv_disconnect_worker(old)
            try:
                old.deleteLater()
            except RuntimeError:
                pass

        self._hv_display_ctx = dict(display_ctx)
        # ★ 必須存成成員：區域變數會被 GC 回收，QThread 直接消失。
        # ★ parent=None（不掛在 MainWindow 底下）：規劃沒跑完就關視窗時，
        #   Qt 會連同子物件一起銷毀，而在「還在跑的 QThread」上呼叫解構子是 qFatal
        #   ⇒ 整個程式 abort 而不是正常關閉。改由 closeEvent 的 _hv_park_running_worker
        #   接手（見該函式）。
        self._hv_worker = HvPlanningWorker(coordinator, plan_kwargs, parent=None)
        self._hv_worker.progress.connect(self._on_hv_progress)
        self._hv_worker.report.connect(self._on_hv_report)
        self._hv_worker.failed.connect(self._on_hv_failed)
        self._hv_worker.finished_ok.connect(self._on_hv_finished)
        self._hv_worker.cancelled.connect(self._on_hv_cancelled)
        self.statusBar().showMessage('HV-DCCPP 規劃中…', 0)
        self._hv_worker.start()

    def _on_hv_progress(self, frac: float, msg: str):
        """進度回報。⚠ 0.40 → 0.90 之間沒有中間進度（plan_mission 整段無回呼）。"""
        self.statusBar().showMessage(f'HV-DCCPP {frac * 100:.0f}%：{msg}', 0)

    #: `notes` 裡真的會改變飛行結果、必須讓操作員看到的幾種（其餘只寫進 log）
    _HV_LOUD_NOTES = (
        'coverage_altitude_clamped',    # 你輸入的作業高度被夾到構型下限
        'fov_width_mismatch',           # 面板的刈幅與感測器推導值差 > 25%
        'alternate_auto_placed',        # 系統自己放了一個備降點（不見得能降落）
        'coverage_altitude_ignored',
        'overlap_rate_out_of_range',
        # ⚠⚠ 轉接層從來不呼叫 hv_dccpp.validate()（全 repo grep 不到），
        #   所以 SwarmCoordinator 的 HV 分支自己補跑一次，失敗時走這個前綴。
        #   時空去衝突與能量預算只有它會檢查，preflight 不會。
        'validate_failed',
        # ⚠⚠⚠ 混編機隊 + 只切出一個胞的作業區 ⇒ 整份計畫是空的
        #   （makespan=0.0、assembled_paths=None）而 status 仍是 GO_WITH_WARNINGS
        'no_aircraft_assigned',
        # 部分機隊閒置：只顯示「1/2 架」的話使用者不知道要調哪個旋鈕
        'partial_fleet_assigned',
        # ⚠⚠⚠ AGL 上限 − 最低固定翼作業高度 < 高度帶下限 ⇒ 該構型分不到工作，
        #   全部落空時整趟是 NO_GO，而上游訊息把四個原因擠在同一句話裡。
        'altitude_band_too_narrow',
        # 最低固定翼作業高度被調到表 1 值以下：合法（表 1 說要依法規調整），
        # 但會同時降低寫進飛控的 FENCE_ALT_MIN 與 Q_RTL_ALT。
        'min_fw_lowered',
        # 高度帶下限被調到規格預設 20 以下：同時放寬 GSD 可達性判準。
        'dh_band_min_relaxed',
    )

    def _on_hv_report(self, payload: dict):
        """preflight / 規劃報告：status、warnings、**notes**、uncovered。

        ⚠ `notes` 不能只記個數量：手冊 §8 與 §15 裡最會影響操作員判斷的幾則
        （作業高度被夾、刈幅對不上、備降點是系統自己放的、參數被忽略）
        都是走 `notes` 而不是 `warnings`。
        """
        timing = payload.get('timing_ms') or {}
        notes = list(payload.get('notes') or [])
        # HV 分支補跑的 validate()：走 payload 的獨立鍵，避免跟 preflight 的 notes 打架
        for _vn in (payload.get('validation_notes')
                    or ([payload['validation_note']] if payload.get('validation_note') else [])):
            notes.append(str(_vn))
        warnings = list(payload.get('warnings') or [])
        logger.info(
            f"[HV-DCCPP] status={payload.get('status')} "
            f"errors={len(payload.get('errors') or [])} "
            f"warnings={len(warnings)} notes={len(notes)} "
            f"total={float(timing.get('total', 0.0)):.0f} ms")
        for w in warnings:
            logger.warning(f'[HV-DCCPP] {w}')
        loud = []
        for n in notes:
            text = str(n)
            if text.startswith(self._HV_LOUD_NOTES):
                loud.append(text)
                logger.warning(f'[HV-DCCPP] {text}')
            else:
                logger.info(f'[HV-DCCPP] note: {text}')
        # 存起來給 _on_hv_finished 在後處理**之後**再顯示一次
        # （後處理最後一定會用「DCCPP 最佳化完成」蓋掉狀態列）
        self._hv_last_report = {'loud_notes': loud,
                                'uncovered': list(payload.get('uncovered') or []),
                                'status': payload.get('status')}
        for u in self._hv_last_report['uncovered']:
            logger.error(f'[HV-DCCPP] 未覆蓋：{u}')

    def _hv_show_report_banner(self):
        """把 `_on_hv_report` 收集到的重點顯示出來（要在後處理之後叫，否則會被蓋掉）。"""
        rep = getattr(self, '_hv_last_report', None) or {}
        uncovered, loud = rep.get('uncovered') or [], rep.get('loud_notes') or []
        if uncovered:
            from PyQt6.QtWidgets import QMessageBox
            self.statusBar().showMessage(f'⚠ HV-DCCPP 有未覆蓋區域：{uncovered}', 0)
            QMessageBox.warning(
                self, 'HV-DCCPP 覆蓋不完整',
                '下列區域沒有被完全覆蓋：\n  ' + '\n  '.join(str(u) for u in uncovered)
                + '\n\n（多半要改機隊組成或放寬約束，見接入手冊 §15.2 第 13 條）')
        elif loud:
            self.statusBar().showMessage('HV-DCCPP：' + '；'.join(loud[:3]), 15000)

    def _hv_notify_done(self, ok: bool):
        """呼叫並清掉這一趟的 `on_done`。

        ⚠ 一定要在成功、失敗、取消三條路上都叫到，否則 VTOL 偵察面板那種
        「按下去就顯示『規劃中 …』」的呼叫端會永遠停在那個字樣。
        """
        cb = getattr(self, '_hv_on_done', None)
        self._hv_on_done = None
        if cb is None:
            return
        try:
            cb(bool(ok))
        except Exception as e:      # noqa: BLE001 - 回呼壞掉不該連累規劃流程
            logger.error(f'[HV-DCCPP] on_done 回呼失敗：{e}', exc_info=True)

    def _on_hv_failed(self, msg: str):
        """⚠⚠ 失敗對話框一定要帶上 `_on_hv_report` 收到的重點。

        上游的 NO_GO 只寫「所有區域皆無可行構型（感測相容／高度帶／鏈路／方向）」,
        四個原因擠在同一句話裡；真正說得出「AGL 上限 120 − 最低作業高度 120 = 0 m」
        的是我們自己補的那些 note。只記進 log 檔的話,使用者在畫面上只會看到那句
        沒有資訊量的話。
        """
        from PyQt6.QtWidgets import QMessageBox
        self.statusBar().clearMessage()
        logger.error(f'[HV-DCCPP] 規劃失敗：{msg}')
        loud = list((getattr(self, '_hv_last_report', None) or {}).get('loud_notes') or [])
        text = msg if not loud else (msg + '\n\n' + '\n'.join(f'• {n}' for n in loud))
        QMessageBox.critical(self, 'HV-DCCPP 失敗', text)
        self._hv_notify_done(False)

    def _on_hv_cancelled(self):
        """規劃跑完了但結果已被放棄（呼叫過 stop()，多半是關視窗）。"""
        logger.info('[HV-DCCPP] 規劃結果已放棄（使用者取消）')
        try:
            self.statusBar().clearMessage()
        except RuntimeError:
            pass          # 視窗已經沒了
        self._hv_notify_done(False)

    def _on_hv_finished(self, result: dict):
        """背景規劃完成 → 回到 GUI 執行緒做後處理（與同步路徑同一支）。"""
        worker = getattr(self, '_hv_worker', None)
        coordinator = getattr(worker, 'coordinator', None)
        self._hv_coordinator = getattr(coordinator, 'hv_coordinator', None)
        ctx = getattr(self, '_hv_display_ctx', None) or {}
        self._display_dccpp_result(result, **ctx)
        # 後處理會把狀態列改成「DCCPP 最佳化完成…」，所以警示要在它之後才顯示
        self._hv_show_report_banner()
        self._hv_notify_done(True)

    def _on_dem_loaded(self, dem_path: str):
        """使用者選取 DEM → 載入並在 3D 地圖上疊加 heatmap"""
        try:
            from sensors.dem_terrain import DEMTerrainManager
            dem = DEMTerrainManager()
            if not dem.load_dem(dem_path):
                QMessageBox.warning(self, 'DEM', f'載入失敗：\n{dem_path}')
                return
            self._dem_manager = dem
            # 將 DEM 注入 3D 地圖：供「地形跟隨」模式採樣每個航點的地形海拔
            if hasattr(self.map_widget, 'set_dem_manager'):
                self.map_widget.set_dem_manager(dem)
            # 嘗試在 3D 地圖顯示
            cesium = None
            if hasattr(self, 'map_widget'):
                cesium = getattr(self.map_widget, 'map_3d', None) or \
                         (self.map_widget if hasattr(self.map_widget, 'show_dem_overlay') else None)
            if cesium is None or not hasattr(cesium, 'show_dem_overlay'):
                QMessageBox.information(
                    self, 'DEM',
                    f'✅ DEM 已載入\n'
                    f'緯度範圍: {dem._lat_min:.4f} ~ {dem._lat_max:.4f}\n'
                    f'經度範圍: {dem._lon_min:.4f} ~ {dem._lon_max:.4f}\n'
                    f'解析度: {dem._cols}x{dem._rows}\n\n'
                    f'（3D 地圖不可用，無法視覺化）'
                )
                return
            ok = cesium.show_dem_overlay(dem, alpha=0.6)
            # 同步建立 3D 立體地形網格（真實起伏）
            try:
                if hasattr(cesium, 'show_dem_terrain_3d'):
                    cesium.show_dem_terrain_3d(dem, max_size=80)
            except Exception as _e3:
                logger.warning(f'[DEM3D] 3D 地形網格生成失敗: {_e3}')
            # 自動切換到 3D 頁
            try:
                if hasattr(self.map_widget, '_switch_mode'):
                    self.map_widget._switch_mode(1)  # _MODE_3D
            except Exception:
                pass
            if ok:
                QMessageBox.information(
                    self, 'DEM',
                    f'✅ DEM 已疊加到 3D 地圖\n'
                    f'緯度: {dem._lat_min:.4f} ~ {dem._lat_max:.4f}\n'
                    f'經度: {dem._lon_min:.4f} ~ {dem._lon_max:.4f}\n'
                    f'解析度: {dem._cols}x{dem._rows}'
                )
            else:
                QMessageBox.warning(self, 'DEM', '3D 疊加失敗，請查看日誌（可能缺 Pillow）')
        except Exception as e:
            logger.error(f'[DEM] 載入/顯示失敗: {e}', exc_info=True)
            QMessageBox.critical(self, 'DEM', f'錯誤: {e}')

    def _on_dccpp_export_requested(self):
        """處理 DCCPP 匯出按鈕請求

        若 DCCPP 載具類型為 VTOL，自動轉派到 VTOL 匯出流程。
        """
        from PyQt6.QtWidgets import QMessageBox
        if getattr(self, '_dccpp_result', None) is None:
            QMessageBox.warning(
                self, "無 DCCPP 結果",
                "請先執行 DCCPP 最佳化規劃，再匯出任務。"
            )
            return

        # 偵測 DCCPP 載具類型：若為 VTOL 則轉派
        vehicle_type = self.flight_params.get('vehicle_type', '')
        if vehicle_type == 'vtol':
            transition_alt = 40.0
            try:
                transition_alt = self.parameter_panel.vtol_transition_alt_spin.value()
            except Exception:
                pass
            self._on_vtol_export_requested({'transition_alt': transition_alt})
            return

        try:
            self._export_dccpp_waypoints()
        except Exception as e:
            logger.error(f"匯出 DCCPP 任務失敗: {e}", exc_info=True)
            QMessageBox.warning(self, "匯出失敗", f"DCCPP 任務匯出失敗：\n{e}")

    def _on_vtol_export_requested(self, params: dict):
        """處理 VTOL 匯出按鈕請求"""
        from PyQt6.QtWidgets import QMessageBox, QFileDialog
        if getattr(self, '_dccpp_result', None) is None:
            QMessageBox.warning(
                self, "無 DCCPP 結果",
                "請先執行 DCCPP 最佳化規劃，再匯出 VTOL 任務。"
            )
            return

        assembled_paths = self._dccpp_result.get('assembled_paths', {})
        if not assembled_paths:
            QMessageBox.warning(self, "無可匯出路徑", "DCCPP 結果中沒有組裝路徑資料。")
            return

        export_dir = QFileDialog.getExistingDirectory(
            self, "選擇 VTOL 任務匯出目錄", ""
        )
        if not export_dir:
            return

        try:
            from mission.vtol_mission_exporter import VTOLMissionExporter

            altitude = self.flight_params.get('altitude', 100.0)
            speed = self.flight_params.get('speed', 15.0)
            turn_radius = self.flight_params.get('turn_radius', 50.0)
            transition_alt = params.get('transition_alt', 40.0)

            exporter = VTOLMissionExporter(
                transition_alt_m=transition_alt,
                cruise_speed_ms=speed,
                turn_radius_m=turn_radius,
            )

            # 匯出航點檔案
            exported_files = exporter.export_all(
                assembled_paths=assembled_paths,
                export_dir=export_dir,
                altitude=altitude,
                speed=speed,
            )

            # 在 Cesium 3D 圖台繪製 VTOL 軌跡視覺化
            try:
                cesium = self._get_cesium_widget()
                if cesium:
                    vtol_data = exporter.to_cesium_data(assembled_paths, altitude)
                    cesium.draw_vtol_swarm_paths(vtol_data)
            except Exception as ve:
                logger.warning(f'[VTOL] 3D 視覺化失敗: {ve}')

            QMessageBox.information(
                self, "VTOL 匯出成功",
                f"VTOL (4+1 混飛模式) 任務匯出完成！\n"
                f"目錄：{export_dir}\n"
                f"轉換高度：{transition_alt}m\n\n"
                + "\n".join(exported_files)
            )
            self.statusBar().showMessage(
                f"已匯出 {len(exported_files)} 個 VTOL 任務檔案至 {export_dir}", 5000
            )
            logger.info(f"VTOL 匯出完成: {len(exported_files)} 個檔案")

        except Exception as e:
            logger.error(f"匯出 VTOL 任務失敗: {e}", exc_info=True)
            QMessageBox.warning(self, "匯出失敗", f"VTOL 任務匯出失敗：\n{e}")

    def _export_dccpp_waypoints(self):
        """
        匯出 DCCPP 最佳化結果為每架 UAV 獨立的 .waypoints 檔案
        格式：QGC WPL 110（Mission Planner 相容）

        航點指令對應（符合論文三類路徑 + 起飛降落）：
          DO_SET_HOME (179)     - Home 佔位
          DO_CHANGE_SPEED (178) - 空速設定
          NAV_TAKEOFF (22)      - 起飛到指定高度 (param1=爬升角, param4=yaw)
          NAV_WAYPOINT (16)     - 爬升/進入/作業/轉移/進場航點
                                  param2=turn_radius（讓飛控提前轉彎）
          NAV_LAND (21)         - 降落觸地 (param4=著陸方向)
        """
        import os, math
        from PyQt6.QtWidgets import QFileDialog, QMessageBox

        export_dir = QFileDialog.getExistingDirectory(
            self, "選擇 DCCPP 任務匯出目錄", ""
        )
        if not export_dir:
            return

        assembled_paths = self._dccpp_result.get('assembled_paths', {})
        if not assembled_paths:
            QMessageBox.warning(self, "無可匯出路徑", "DCCPP 結果中沒有組裝路徑資料。")
            return

        altitude = self.flight_params.get('altitude', 100.0)
        speed = self.flight_params.get('speed', 15.0)
        turn_radius = self.flight_params.get('turn_radius', 50.0)
        exported_files = []

        for uav_id, apath in assembled_paths.items():
            if not apath.waypoints:
                continue

            waypoint_lines = ['QGC WPL 110']
            seq = 0

            # ── seq 0: DO_SET_HOME ──
            home_wp = apath.waypoints[0]
            waypoint_lines.append(create_waypoint_line(
                seq=seq, command=179,
                lat=home_wp.lat, lon=home_wp.lon, alt=0.0,
                current=0, autocontinue=1
            ))
            seq += 1

            # ── seq 1: DO_CHANGE_SPEED (空速) ──
            waypoint_lines.append(create_waypoint_line(
                seq=seq, command=178,
                param1=1.0, param2=speed, param3=0.0,
                current=0, autocontinue=1
            ))
            seq += 1

            # ── 依 SegmentLabel 分派 MAVLink 指令 ──
            # 先找到最後一個 LANDING 的索引 → 只在那一點發 NAV_LAND
            last_landing_idx = -1
            for _i, _w in enumerate(apath.waypoints):
                if getattr(getattr(_w, 'segment_type', None), 'name', '') == 'LANDING':
                    last_landing_idx = _i

            takeoff_emitted = False
            for idx, wp in enumerate(apath.waypoints):
                wp_alt = wp.alt if wp.alt > 0 else altitude

                if getattr(wp.segment_type, 'name', '') == 'TAKEOFF':
                    if idx == 0:
                        # Home 地面點已在 DO_SET_HOME 處理，跳過
                        continue
                    if not takeoff_emitted and idx >= 1:
                        # 跑道末端 → NAV_TAKEOFF（param4=0 使用當前航向）
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=22,  # NAV_TAKEOFF
                            param1=10.0,  # 最小爬升俯仰角
                            param4=0.0,
                            lat=wp.lat, lon=wp.lon, alt=wp_alt,
                            current=0, autocontinue=1
                        ))
                        seq += 1
                        takeoff_emitted = True
                    else:
                        # 爬升過渡點
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=16,
                            lat=wp.lat, lon=wp.lon, alt=wp_alt,
                            param2=turn_radius,
                            current=0, autocontinue=1
                        ))
                        seq += 1

                elif getattr(wp.segment_type, 'name', '') == 'LANDING':
                    if idx == last_landing_idx:
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=21,  # NAV_LAND
                            param4=getattr(wp, 'heading_compass_deg', getattr(wp, 'heading_deg', 0.0)),
                            lat=wp.lat, lon=wp.lon, alt=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    else:
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=16,
                            lat=wp.lat, lon=wp.lon, alt=wp_alt,
                            param2=turn_radius,
                            current=0, autocontinue=1
                        ))
                        seq += 1

                else:
                    # ENTRY / OPERATION / TRANSFER → NAV_WAYPOINT
                    # acceptance_radius = turn_radius（讓固定翼提前轉彎）
                    accept_r = turn_radius
                    if getattr(wp.segment_type, 'name', '') == 'OPERATION':
                        # 作業段密集航點：減小 acceptance_radius 避免跳點
                        if idx < len(apath.waypoints) - 1:
                            nxt = apath.waypoints[idx + 1]
                            dist_next = math.hypot(
                                (nxt.lat - wp.lat) * 111320,
                                (nxt.lon - wp.lon) * 111320 * math.cos(math.radians(wp.lat))
                            )
                            accept_r = max(turn_radius / 10.0,
                                           min(dist_next / 3.0, turn_radius))

                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,
                        lat=wp.lat, lon=wp.lon, alt=wp_alt,
                        param2=accept_r,
                        current=0, autocontinue=1
                    ))
                    seq += 1

            filename = f"DCCPP_UAV{uav_id}_高度{altitude:.0f}m.waypoints"
            filepath = os.path.join(export_dir, filename)
            write_waypoints(filepath, waypoint_lines)
            exported_files.append(filename)
            logger.info(f"匯出 DCCPP UAV-{uav_id}: {filepath} ({seq} 航點)")

        # 匯出簡報
        briefing_path = os.path.join(export_dir, "DCCPP_briefing.txt")
        with open(briefing_path, 'w', encoding='utf-8') as f:
            f.write("DCCPP 最佳化覆蓋任務簡報\n")
            f.write("=" * 40 + "\n\n")
            total_makespan = self._dccpp_result.get('total_makespan', 0.0)
            f.write(f"無人機數量：{len(assembled_paths)}\n")
            f.write(f"預估 Makespan：{total_makespan:.1f}s ({total_makespan/60:.1f}min)\n")
            f.write(f"飛行高度：{altitude:.0f}m\n")
            f.write(f"飛行速度：{speed:.1f}m/s\n")
            f.write(f"轉彎半徑：{turn_radius:.0f}m\n\n")
            for uav_id, apath in assembled_paths.items():
                total_len = apath.total_length_m
                f.write(
                    f"UAV-{uav_id}：{len(apath.waypoints)} 航點，"
                    f"總長 {total_len:.0f}m\n"
                    f"  起飛 {getattr(apath, 'takeoff_length_m', 0.0):.0f}m + "
                    f"進場 {getattr(apath, 'entry_length_m', 0.0):.0f}m + "
                    f"作業 {getattr(apath, 'operation_length_m', 0.0):.0f}m + "
                    f"轉移 {getattr(apath, 'transfer_length_m', 0.0):.0f}m + "
                    f"降落 {getattr(apath, 'landing_length_m', 0.0):.0f}m\n"
                )
        exported_files.append("DCCPP_briefing.txt")

        QMessageBox.information(
            self, "匯出成功",
            f"DCCPP 任務匯出完成！\n目錄：{export_dir}\n\n"
            + "\n".join(exported_files)
        )
        self.statusBar().showMessage(
            f"已匯出 {len(exported_files)} 個 DCCPP 任務檔案至 {export_dir}", 5000
        )
        logger.info(f"DCCPP 匯出完成: {len(exported_files)} 個檔案")

    # ─────────────────────────────────────────────────────────────────
    # SITL / MAVLink 即時遙測
    # ─────────────────────────────────────────────────────────────────
    def on_sitl_connect(self, conn_str: str, sysid_label: int = 1,
                        vehicle_hint: str = ''):
        """啟動 SITL 背景接收執行緒（單一連線）

        vehicle_hint: 由 UI 傳入的載具類型（'VTOL'/'PLANE'/'COPTER'）
                      用於區分 QuadPlane VTOL 與純固定翼（兩者 MAV_TYPE 相同）
        """
        from mission.sitl_link import SITLLink
        link = SITLLink(conn_str, sysid_label=sysid_label,
                        vehicle_hint=vehicle_hint, parent=self)
        link.connected.connect(self.sitl_hud.on_connected)
        # 帶入 link 身分 → 逐機斷線清理（手動快速斷線 / 自然斷線皆走此路）
        link.disconnected.connect(
            lambda reason, l=link: self._on_sitl_link_disconnected(l, reason))
        link.telemetry.connect(self._on_sitl_telemetry)
        link.status_text.connect(self.sitl_hud.on_status_text)
        link.error.connect(self.sitl_hud.on_error)
        # SERVO 參數（FUNCTION/MIN/TRIM/MAX/REVERSED）→ ServoOutputPanel
        link.servo_param.connect(self.sitl_hud.on_servo_param)
        # 任務下載結果 → 畫到地圖。
        # ⚠️ 接在**建立 link 時**，不是接在按鈕的處理函式裡：這樣不論觸發來源
        #    是 HUD 的按鈕、DroneDetailPanel 的 ↧、還是程式自己呼叫
        #    link.download_mission()，結果都會畫出來，而且每條 link 只接一次。
        #    接在處理函式裡會每按一次多接一條 ⇒ 第 N 次點擊畫 N 遍。
        link.mission_downloaded.connect(self._on_mission_downloaded)
        link.mission_download_failed.connect(self._on_mission_download_failed)
        link.start()
        self._sitl_links.append(link)
        if self._sitl_link is None:
            self._sitl_link = link

        # ── 註冊到 FleetRegistry：讓 ADOS 風 SitlCommandPanel 看見此 UAV ──
        # callsign 規則：sysid 1 → "UAV-1"…等，與既有 status bar 一致。
        try:
            from mission.fleet_registry import FleetRegistry
            callsign = f'UAV-{int(sysid_label)}'
            FleetRegistry.instance().register(callsign, link, int(sysid_label))
        except Exception as e:
            logger.warning(f'[SITL] 註冊 FleetRegistry 失敗: {e}')

        logger.info(f'[SITL] 啟動連線執行緒: {conn_str} (sysid={sysid_label})')

    def on_embedded_swarm_connect(self):
        """連線到嵌入式蜂群（模式 A1：AeroPlan 當監看 GCS）。

        不啟動任何內建 SITL，改為依使用者在 EmbeddedSwarmDialog 設定的
        嵌入式 IP / 機數 / port / sysid，對原生 ArduPlane SITL 的每個 TCP server
        各建立一條 MAVLink 監看連線（沿用既有 on_sitl_connect 機制）。

        對應固定翼蜂群文件 §4.1：導引由嵌入式 ROS 蜂群負責，本端只顯示遙測。
        """
        from ui.dialogs.embedded_swarm_dialog import EmbeddedSwarmDialog

        # 已有連線時先提醒：避免新舊連線疊加造成同一 sysid 兩張卡片
        if self._sitl_links:
            ret = QMessageBox.question(
                self, '已有連線',
                f'目前已有 {len(self._sitl_links)} 條 SITL 連線。\n'
                f'是否先全部斷線，再連線到嵌入式蜂群？',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            )
            if ret != QMessageBox.StandardButton.Yes:
                return
            self.on_sitl_disconnect()

        cfg = EmbeddedSwarmDialog.get_config(parent=self)
        if cfg is None:
            logger.info('[Embedded Swarm] 使用者取消連線')
            return

        connections = cfg.get('connections') or []
        if not connections:
            QMessageBox.warning(self, '嵌入式蜂群', '沒有可連線的端點')
            return

        direction = cfg.get('direction', 'dial')
        host = cfg.get('host', '')
        logger.info(
            f'[Embedded Swarm] 連線 {len(connections)} 機 '
            f'(direction={direction}, host={host or "—"})'
        )

        # 蜂群為固定翼 → vehicle_hint 固定 'PLANE'
        for c in connections:
            self.on_sitl_connect(
                c['conn_str'], sysid_label=int(c['sysid']), vehicle_hint='PLANE',
            )

        self.statusBar().showMessage(
            f'🛰 已連線嵌入式蜂群 {len(connections)} 機'
            f'（{"主動 tcp→" + host if direction == "dial" else "被動 tcpin 監聽"}）',
            6000,
        )

    def on_sitl_disconnect(self):
        """斷開所有 SITL 連線"""
        # 先把 FleetRegistry 內對應 callsign 反註冊（不擋停止流程）
        try:
            from mission.fleet_registry import FleetRegistry
            reg = FleetRegistry.instance()
            for link in self._sitl_links:
                try:
                    cs = f'UAV-{int(getattr(link, "sysid_label", 1))}'
                    reg.unregister(cs)
                except Exception:
                    pass
        except Exception:
            pass
        for link in self._sitl_links:
            # 先切斷 disconnected → 避免逐機清理 handler 在全域斷線時重入
            try:
                link.disconnected.disconnect()
            except Exception:
                pass
            try:
                link.stop()
                link.wait(2000)
            except Exception:
                pass
        self._sitl_links.clear()
        self._sitl_link = None
        # HIL 鏈路是 _sitl_links 子集，一併視為已斷線
        if self._hil_links:
            self._hil_links.clear()
            self.hil_panel.on_disconnected()
        self.sitl_hud.on_disconnected('使用者中斷')   # 內含 clear_cards + 復位指令對象
        if hasattr(self.map_widget, 'clear_uav'):
            self.map_widget.clear_uav()

    # ─────────────────────────────────────────────────────────────────
    # HIL：實體飛控多機連線（SITL 分頁 HIL 內頁）
    # ─────────────────────────────────────────────────────────────────
    def _on_hil_connect_all(self, rows: list):
        """一次建立多顆實體飛控的 MAVLink 鏈路（上限 6，與 SITL 共用管線）。

        rows: [{'conn': 'COM7,115200', 'sysid': 1, 'vehicle_hint': ''}, ...]
        """
        ok = 0
        for row in rows:
            before = len(self._sitl_links)
            try:
                self.on_sitl_connect(
                    row['conn'],
                    int(row.get('sysid', 1)),
                    row.get('vehicle_hint', ''),
                )
            except Exception as e:
                logger.error(f'[HIL] 建立鏈路失敗 {row.get("conn")}: {e}')
                continue
            if len(self._sitl_links) > before:
                self._hil_links.append(self._sitl_links[-1])
                ok += 1
        self.hil_panel.on_connect_result(ok, len(rows))
        self.statusBar().showMessage(f'[HIL] 已建立 {ok}/{len(rows)} 條飛控鏈路', 6000)

    def _on_hil_disconnect_all(self):
        """只斷開 HIL 鏈路，保留內建 SITL / 嵌入式蜂群連線。"""
        try:
            from mission.fleet_registry import FleetRegistry
            reg = FleetRegistry.instance()
        except Exception:
            reg = None
        for link in self._hil_links:
            if reg is not None:
                try:
                    reg.unregister(f'UAV-{int(getattr(link, "sysid_label", 1))}')
                except Exception:
                    pass
            # 先切斷 disconnected → 避免逐機清理 handler 重入
            try:
                link.disconnected.disconnect()
            except Exception:
                pass
            try:
                link.stop()
                link.wait(2000)
            except Exception:
                pass
            try:
                self._sitl_links.remove(link)
            except ValueError:
                pass
        self._hil_links.clear()
        if self._sitl_link is not None and self._sitl_link not in self._sitl_links:
            self._sitl_link = self._sitl_links[0] if self._sitl_links else None
        self.hil_panel.on_disconnected()
        self.statusBar().showMessage('[HIL] 已斷開所有實體飛控鏈路', 5000)

    def on_sitl_disconnect_one(self, sysid: int):
        """單獨斷開指定 sysid 的 SITL 連線（HUD 卡片快速斷線鈕）。

        僅停止該機鏈路，其餘連線維持。實際清理（移除卡片 / 反註冊 FleetRegistry /
        指令對象復位）由該鏈路結束時發出的 disconnected 信號統一交給
        _on_sitl_link_disconnected 處理，避免重複清理。
        """
        link = next((l for l in self._sitl_links
                     if int(getattr(l, 'sysid_label', 1)) == int(sysid)), None)
        if link is None:
            return
        logger.info(f'[SITL] 單獨斷線 UAV-{sysid}')
        try:
            link.stop()
            link.wait(2000)
        except Exception as e:
            logger.error(f'[SITL] 單獨斷線失敗 sysid={sysid}: {e}')

    def _on_sitl_link_disconnected(self, link, reason: str):
        """單一 SITL 鏈路結束（手動快速斷線 / 連線自然中斷）→ 僅清理該機。

        全域斷線（on_sitl_disconnect）已預先切斷 disconnected 信號，故不會走到這裡。
        """
        sysid = int(getattr(link, 'sysid_label', 1))
        if link in self._sitl_links:
            self._sitl_links.remove(link)
        # FleetRegistry 反註冊（callsign 與註冊時一致）
        try:
            from mission.fleet_registry import FleetRegistry
            FleetRegistry.instance().unregister(f'UAV-{sysid}')
        except Exception:
            pass
        # 移除該機 HUD 卡片 + 地圖圖標（僅該 sysid）
        try:
            self.sitl_hud.remove_card(sysid)
        except Exception:
            pass
        if hasattr(self.map_widget, 'clear_uav'):
            try:
                self.map_widget.clear_uav(sysid)
            except Exception:
                pass
        # 維護主鏈路指標（向後相容）
        if self._sitl_link is link:
            self._sitl_link = self._sitl_links[0] if self._sitl_links else None
        # 全部離線 → 還原全域連線狀態；否則僅於 log 區提示
        if not self._sitl_links:
            self.sitl_hud.on_disconnected(reason)
        else:
            self.sitl_hud.lbl_status.setText(
                f'🔌 UAV-{sysid} 已斷線（剩餘 {len(self._sitl_links)} 台連線）')

    def _on_sitl_telemetry(self, frame):
        """收到一幀遙測 → 更新 HUD + 推到 3D 地圖（多機支援）"""
        # 主機 (sysid_label==1) 才更新 HUD，避免多機資訊互相覆蓋
        # 所有 sysid 都送進 HUD（多機卡片）
        self.sitl_hud.on_telemetry(frame)
        # 投放讀回確認：PWM 真的動了才播落點模擬（見 on_release_store）
        if getattr(self, '_pending_releases', None):
            self._confirm_pending_release(frame)
        if frame.is_valid_gps() and hasattr(self.map_widget, 'update_uav_position'):
            _pitch = getattr(frame, 'pitch', 0.0)
            _roll = getattr(frame, 'roll', 0.0)
            self.map_widget.update_uav_position(
                frame.lat, frame.lon, frame.alt_rel,
                frame.heading, frame.ground_speed,
                sysid=frame.sysid, mode=frame.mode, armed=frame.armed,
                vehicle_type=getattr(frame, 'vehicle_type', ''),
                pitch_deg=_pitch,
                roll_deg=_roll,
            )
            # 同步更新戰術模組（FOV 光錐、熱力圖、RCS）
            self._update_tactical_on_uav_move(
                frame.lat, frame.lon, frame.alt_rel,
                frame.heading, _pitch, _roll,
                sysid=frame.sysid,
            )

    def _target_links(self) -> list:
        """指令派送對象鏈路：依 HUD「指令對象」選取決定範圍。

        HUD 選取單機 → 只回該 sysid 的鏈路；未選（全部）→ 回全部鏈路。
        供 _sitl_broadcast / 圍籬上傳等「快捷操作」共用，達成
        「選中某台 → 下方快捷操作只單獨操作他」。
        """
        sid = None
        try:
            sid = self.sitl_hud.get_target_sysid()
        except Exception:
            sid = None
        if sid is None:
            return list(self._sitl_links)
        return [l for l in self._sitl_links
                if int(getattr(l, 'sysid_label', 1)) == int(sid)]

    def _sitl_broadcast(self, cmd: str, arg=None):
        """派送指令到 SITL 連線（依 HUD 指令對象：單機或全部）"""
        if not self._sitl_links:
            QMessageBox.information(self, 'SITL', '尚未連線任何 SITL')
            return
        links = self._target_links()
        if not links:
            QMessageBox.information(
                self, 'SITL', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return
        for link in links:
            try:
                if cmd == 'arm':       link.arm()
                elif cmd == 'disarm':  link.disarm()
                elif cmd == 'set_mode':link.set_mode(arg)
                elif cmd == 'takeoff': link.takeoff(arg)
                elif cmd == 'guided_takeoff': link.guided_takeoff(arg)
                elif cmd == 'vtol_transition': link.vtol_transition(int(arg))
                elif cmd == 'auto_start': link.auto_start()
                elif cmd == 'mission_start': link.mission_start()
                elif cmd == 'set_param':
                    name, value, ptype = arg
                    link.set_param(name, value, ptype)
                elif cmd == 'set_params':
                    link.set_params(arg)
                elif cmd == 'get_param':
                    link.get_param(arg)
            except Exception as e:
                logger.error(f'[SITL] {cmd} 失敗: {e}')

    # ── 投放：送指令 → **等 PWM 讀回真的變了** → 才播落點模擬 ──────────
    # 設計要點：動畫不是按鈕觸發的，是**飛控回報的 SERVO_OUTPUT_RAW 觸發的**。
    # 按鈕只負責送指令；只有在該通道的 PWM 真的跳到目標值之後才會有動畫。
    # 這樣「畫面上有東西掉下去」就等於「伺服真的動了」，而不是兩件不相干的事
    # 剛好同時發生 —— 否則指令被飛控吃掉（例如 SERVOn_FUNCTION 不是 Disabled）
    # 時畫面照樣播，那是最糟的一種假綠燈。
    _RELEASE_CONFIRM_TIMEOUT_S = 4.0     # 超過這麼久沒看到 PWM 變化 = 判定沒動
    _RELEASE_PWM_TOL = 25                # μs：讀回值與目標值的容差

    def on_release_store(self, index: int, kind: str = 'dumb'):
        """SITL 面板的「一號彈 / 二號彈」。

        只做一件事：把該掛架的伺服通道打到 HI（MAV_CMD_DO_SET_SERVO）。
        落點模擬由 `_confirm_pending_release()` 在**讀回確認**之後才啟動。
        """
        import time as _time
        if not self._sitl_links:
            QMessageBox.information(self, '投放', '尚未連線任何 SITL')
            return
        links = self._target_links()
        if not links:
            QMessageBox.information(
                self, '投放', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return

        ch = self.sitl_hud.release_channel(index)
        pwm = self.sitl_hud.release_pwm()
        label = f'{index} 號彈'
        tol = self._RELEASE_PWM_TOL

        if not hasattr(self, '_pending_releases'):
            self._pending_releases = []

        sent, already = 0, []
        for link in links:
            sysid = int(getattr(link, 'sysid_label', 1))
            f = link.get_latest_telemetry()
            before = int(f.servo_pwm[ch - 1]) if f else 0
            # 整條 servo_pwm 都是 0 = 還沒收到 SERVO_OUTPUT_RAW，不是「PWM 是 0」
            has_stream = bool(f and any(f.servo_pwm))
            if has_stream and abs(before - pwm) <= tol:
                # 已經停在放的位置：再送一次不會有任何變化，也就無從確認。
                # 這不是投放成功，是「上一發投完沒歸位」。
                already.append(sysid)
                continue
            try:
                link.set_servo(ch, pwm)
                sent += 1
                self._pending_releases.append(dict(
                    sysid=sysid, ch=ch, target=pwm, before=before,
                    kind=kind, label=label, t0=_time.monotonic(),
                    has_stream=has_stream))
            except Exception as e:
                logger.error(f'[投放] UAV-{sysid} set_servo ch{ch}={pwm} 失敗: {e}')

        if already:
            QMessageBox.information(
                self, '投放',
                f'UAV-{"/".join(str(x) for x in already)} 的 ch{ch} 已經停在 '
                f'{pwm} μs（上一發投完沒有歸位）。\n'
                f'再送一次不會有任何 PWM 變化，因此無法確認投放，已略過。\n\n'
                f'請先把該通道送回鎖住位置再投。')
        if sent == 0:
            return

        logger.warning(f'[投放] {label}: SERVO ch{ch} → {pwm} μs，送出 {sent} 台，'
                       f'等待 PWM 讀回確認')
        self.statusBar().showMessage(
            f'💣 {label} 指令已送出（ch{ch} → {pwm} μs，{sent} 台）'
            f'　等待 SERVO_OUTPUT_RAW 讀回確認…', 6000)
        self._ensure_release_sweeper()

    def _ensure_release_sweeper(self):
        """逾時掃描器：沒等到 PWM 變化就大聲說「沒動」，不要靜靜地什麼都沒發生。"""
        if getattr(self, '_release_timer', None) is None:
            self._release_timer = QTimer(self)
            self._release_timer.setInterval(500)
            self._release_timer.timeout.connect(self._sweep_pending_releases)
        if not self._release_timer.isActive():
            self._release_timer.start()

    def _sweep_pending_releases(self):
        import time as _time
        pend = getattr(self, '_pending_releases', None)
        if not pend:
            if getattr(self, '_release_timer', None) is not None:
                self._release_timer.stop()
            return
        now = _time.monotonic()
        for pr in list(pend):
            if now - pr['t0'] < self._RELEASE_CONFIRM_TIMEOUT_S:
                continue
            pend.remove(pr)
            if not pr['has_stream']:
                why = (f'收不到 ch{pr["ch"]} 的 SERVO_OUTPUT_RAW —— 無法確認。'
                       f'請確認 SERVO OUTPUT 面板有在跳動。')
            else:
                # 實測（2026-09-01, ArduCopter SITL）：0/1/51 打得上去，
                # 33 Motor1 與 24 Flap 完全擋掉 —— 是「被主動驅動的功能」擋的，
                # 不是「只要不是 0 就會被擋」。訊息照實寫，不要給錯的線索。
                why = (f'ch{pr["ch"]} 的 PWM 仍停在 {pr["before"]} μs，沒有變化。'
                       f'先查 SERVO{pr["ch"]}_FUNCTION：若被指派給飛控會主動'
                       f'驅動的功能（例如 Motor / Flap），DO_SET_SERVO 會被擋掉；'
                       f'投放通道請設為 0 (Disabled)。')
            logger.error(f'[投放] UAV-{pr["sysid"]} {pr["label"]} 未確認：{why}')
            self.statusBar().showMessage(
                f'⚠️ UAV-{pr["sysid"]} {pr["label"]} **未確認投放**：{why}', 15000)

    def _confirm_pending_release(self, frame):
        """每幀遙測都檢查：等到的那一刻就用**這一幀**當投放點去播動畫。

        用當下這一幀而不是按鈕按下那一刻的位置：兩者相差數百毫秒，
        高速飛行下就是幾十公尺，落點會差一截。
        """
        pend = getattr(self, '_pending_releases', None)
        if not pend:
            return
        tol = self._RELEASE_PWM_TOL
        for pr in list(pend):
            if pr['sysid'] != int(getattr(frame, 'sysid', 1)):
                continue
            try:
                now_pwm = int(frame.servo_pwm[pr['ch'] - 1])
            except Exception:
                continue
            if abs(now_pwm - pr['target']) > tol:
                continue
            if now_pwm == pr['before']:
                continue          # 本來就在那個值，不算「動了」
            pend.remove(pr)
            logger.warning(
                f'[投放] UAV-{pr["sysid"]} {pr["label"]} 已確認：'
                f'ch{pr["ch"]} {pr["before"]} → {now_pwm} μs')
            self.statusBar().showMessage(
                f'💥 UAV-{pr["sysid"]} {pr["label"]} 已確認投放'
                f'（ch{pr["ch"]} {pr["before"]} → {now_pwm} μs）', 8000)
            self._play_store_drop(frame, pr)

    def _play_store_drop(self, frame, pr):
        """在 3D 地圖上播一發落點模擬。⚠️ 純視覺化：無風、無升力、單一阻力係數。"""
        cesium = self._get_cesium_widget()
        if cesium is None or not hasattr(cesium, 'drop_store'):
            return
        # ⚠️ _get_cesium_widget() **不管現在顯示 2D 還是 3D 都會回傳物件**。
        # App 預設開在 2D，直接畫下去就是畫進看不見的圖台，狀態列還照樣報成功
        # —— 這個坑在 _get_visible_cesium_widget() 的註解裡已經記過一次。
        # 這裡不用 _get_visible_cesium_widget()：它會跳 modal 對話框，
        # 而投放已經發生了，不該被擋住。改成「照播、但明講看不到」。
        try:
            from ui.widgets.dual_map_widget import _MODE_3D
            in_3d = getattr(self.map_widget, '_mode', None) == _MODE_3D
        except Exception:
            in_3d = True
        if not in_3d:
            self.statusBar().showMessage(
                f'💥 UAV-{pr["sysid"]} {pr["label"]} 已確認投放。'
                f'落點模擬需要切到 3D 圖台才看得到。', 8000)
            return
        if not frame.is_valid_gps():
            logger.info('[投放] 無有效 GPS，略過落點模擬')
            return
        try:
            from mission import ballistics
            traj = ballistics.simulate_from_frame(frame, kind=pr['kind'])
            if traj.tof <= 0.0:
                logger.info('[投放] 機體已在地面高度，不播落點模擬')
                return
            cesium.drop_store(traj, f'UAV-{pr["sysid"]} {pr["label"]}')
            logger.info(
                f'[投放] UAV-{pr["sysid"]} 彈道：飛行 {traj.tof:.1f}s、'
                f'前拋 {traj.range_m:.0f}m、落地 {traj.impact_speed:.0f}m/s')
        except Exception as e:
            logger.error(f'[投放] 落點模擬失敗: {e}', exc_info=True)

    # ── GUIDED 點圖起飛（點地圖 → 整群一次 GUIDED 起飛）────────────────
    def on_guided_takeoff_mode(self, enabled: bool):
        """切換『點圖起飛』模式（2D/3D 地圖點一下 → 整群 GUIDED 起飛）。與點圖飛到互斥。"""
        if enabled and not self._sitl_links:
            QMessageBox.information(self, 'GUIDED', '尚未連線任何 SITL / 蜂群')
            self.sitl_hud.btn_guided_takeoff_pt.setChecked(False)
            self._guided_takeoff_mode = False
            return
        self._guided_takeoff_mode = bool(enabled)
        if enabled and getattr(self, '_guided_goto_mode', False):
            # 互斥：開「點圖起飛」時自動關「點圖飛到」（setChecked 會連動清旗標）
            self.sitl_hud.btn_guided_goto.setChecked(False)
        self.statusBar().showMessage(
            '🛫 點圖起飛已開啟：在 2D/3D 地圖點一下 → 長機先起飛，確認 GUIDED 後僚機跟隨'
            if enabled else '點圖起飛已關閉', 5000)

    # 分階段起飛參數
    # 確認高度取較高值（≈ 接近 TKOFF_ALT）：固定翼在過低高度進 GUIDED 盤旋容易
    # 失速，且僚機接手/節點接管時需要安全高度餘裕，故設 45 m 而非 20 m。
    _TKOFF_CONFIRM_ALT = 45.0     # m：長機升空確認高度（達此高度才算「已穩定起飛」）
    _TKOFF_CLIMB_TIMEOUT = 60     # 秒：長機爬升逾時保護
    _TKOFF_GUIDED_TIMEOUT = 12    # 秒：長機切 GUIDED 確認逾時（逾時仍放行僚機）

    # ── 長機/僚機選取（單一事實來源，避免到處重複 min/sorted sysid）──────
    def _sorted_links(self):
        """SITL 連線依 sysid_label 由小到大排序（[0]=長機）。"""
        return sorted(self._sitl_links, key=lambda l: getattr(l, 'sysid_label', 1))

    def _get_leader_link(self):
        """長機 = sysid 最小者（無連線回 None）。"""
        return self._sorted_links()[0] if self._sitl_links else None

    def _resolve_flight_target(self):
        """點圖 GUIDED 指令的飛行對象解析（與下方快捷操作一致地尊重 HUD 選取）。

        回傳 (target_link, wingmen)：
          - HUD 選取單機 → (該機, [])：只操作該機，不帶僚機（純單機 GUIDED）。
          - 未選（全部）  → (長機, 其餘)：維持長機帶隊、僚機 V 編隊跟隨。
        target_link 為 None 代表已連線但選取的機剛好離線（呼叫端應提示並中止）。
        """
        links = self._sorted_links()
        if not links:
            return None, []
        try:
            sid = self.sitl_hud.get_target_sysid()
        except Exception:
            sid = None
        if sid is None:
            return links[0], links[1:]            # 全部 → 長機帶隊
        target = next((l for l in links
                       if int(getattr(l, 'sysid_label', 1)) == int(sid)), None)
        return target, []                          # 單機 → 無僚機

    def _confirm_single_target(self, action: str) -> bool:
        """指令對象被點成單機、但現場不只一台時，問清楚再送。

        ⚠️ 這是實測踩到的（2026-09-02）：點一下 UAV 卡片只是想看它，副作用
           卻是把「指令對象」換成那一台。之後右鍵起飛只有那一台會動，**而且
           完全沒有提示** —— 畫面上只有右上角一行小字。使用者看到的現象是
           「僚機不跟隨」，會往編隊那邊找原因，但那裡根本沒有問題。

        回傳 True = 繼續送。
        """
        try:
            sid = self.sitl_hud.get_target_sysid()
        except Exception:
            return True
        if sid is None:                       # 已經是「全部」
            return True
        if len(self._sitl_links) <= 1:        # 本來就只有一台，沒有歧義
            return True
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning)
        box.setWindowTitle('指令對象是單機')
        box.setText(
            f'目前指令對象是 <b>UAV{sid}（單獨）</b>，'
            f'{action}只會送給這一台。<br><br>'
            f'現場共有 {len(self._sitl_links)} 台。要改成「全部 UAV」嗎？')
        b_all = box.addButton('改成全部 UAV',
                              QMessageBox.ButtonRole.AcceptRole)
        b_one = box.addButton(f'只操作 UAV{sid}',
                              QMessageBox.ButtonRole.DestructiveRole)
        box.addButton('取消', QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(b_all)
        box.exec()
        clicked = box.clickedButton()
        if clicked is b_all:
            self.sitl_hud._set_target(None)
            return True
        return clicked is b_one

    def _send_guided_takeoff_swarm(self, lat: float, lon: float, alt: float = 60.0):
        """點地圖 → 長機帶領的「分階段」GUIDED 起飛（非整群同時）：

        階段1：只有長機（sysid 最小者）單獨 GUIDED 起飛。
        階段2：輪詢長機遙測，待其升空（alt ≥ 確認高度）→ 切 GUIDED → 確認 mode=GUIDED。
        階段3：長機 GUIDED 確認後，才讓僚機跟著起飛；之後由蜂群節點維持 V 編隊跟隨。
        """
        if not self._sitl_links:
            return
        if not self._confirm_single_target('起飛'):
            return
        # 尊重 HUD 指令對象：選單機 → 只起飛該機（wingmen=[]）；全部 → 長機帶隊
        leader, wingmen = self._resolve_flight_target()
        if leader is None:
            QMessageBox.information(
                self, 'GUIDED', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return
        try:
            leader.guided_takeoff(alt)           # 階段1：長機單獨起飛
        except Exception as e:
            logger.error(f'[GUIDED] 長機起飛失敗: {e}')
            return
        self._stage_takeoff = {'leader': leader, 'wingmen': wingmen,
                               'alt': alt, 'phase': 'climb', 'ticks': 0}
        self.statusBar().showMessage(
            f'🛫 長機 UAV-{getattr(leader, "sysid_label", 1)} 單獨 GUIDED 起飛中… '
            f'（{len(wingmen)} 架僚機待命，確認長機 GUIDED 後跟隨）', 8000)
        from PyQt6.QtCore import QTimer
        if getattr(self, '_takeoff_timer', None) is None:
            self._takeoff_timer = QTimer(self)
            self._takeoff_timer.timeout.connect(self._tick_staged_takeoff)
        self._takeoff_timer.start(1000)          # 每秒檢查長機狀態

    def _tick_staged_takeoff(self):
        """分階段起飛狀態機：輪詢長機遙測，依序推進 climb → confirm_guided → 放行僚機。"""
        st = getattr(self, '_stage_takeoff', None)
        if not st:
            self._takeoff_timer.stop()
            return
        st['ticks'] += 1
        leader = st['leader']
        frame = leader.get_latest_telemetry()
        alt = frame.alt_rel if frame else 0.0
        mode = frame.mode if frame else '---'

        def _release_wingmen(note: str):
            for w in st['wingmen']:
                try:
                    w.guided_takeoff(st['alt'])
                except Exception as e:
                    logger.error(f'[GUIDED] 僚機起飛失敗: {e}')
            self.statusBar().showMessage(note, 8000)
            self._takeoff_timer.stop()
            self._stage_takeoff = None

        if st['phase'] == 'climb':
            if alt >= self._TKOFF_CONFIRM_ALT:
                try:
                    leader.set_mode('GUIDED')        # 升空 → 切 GUIDED
                except Exception as e:
                    logger.error(f'[GUIDED] 長機切 GUIDED 失敗: {e}')
                st['phase'] = 'confirm_guided'
                st['ticks'] = 0
                self.statusBar().showMessage(
                    f'🛫 長機已升空 {alt:.0f} m → 切 GUIDED 中，確認後放行僚機…', 6000)
            elif st['ticks'] >= self._TKOFF_CLIMB_TIMEOUT:
                self._takeoff_timer.stop()
                self._stage_takeoff = None
                self.statusBar().showMessage('⚠ 長機起飛逾時，已中止分階段流程', 6000)
        elif st['phase'] == 'confirm_guided':
            if mode == 'GUIDED':
                _release_wingmen(
                    f'✅ 長機 GUIDED 確認 → {len(st["wingmen"])} 架僚機跟隨起飛、組 V 編隊')
            elif st['ticks'] >= self._TKOFF_GUIDED_TIMEOUT:
                _release_wingmen('⚠ 長機 GUIDED 未及時確認，仍放行僚機起飛')

    # ── GUIDED 點圖飛到（長機 GUIDED，僚機由蜂群節點跟隨）──────────────
    def on_guided_goto_mode(self, enabled: bool):
        """切換『點圖飛到 (GUIDED)』模式。與點圖起飛互斥。"""
        if enabled and not self._sitl_links:
            QMessageBox.information(self, 'GUIDED', '尚未連線任何 SITL / 蜂群')
            self.sitl_hud.btn_guided_goto.setChecked(False)
            self._guided_goto_mode = False
            return
        self._guided_goto_mode = bool(enabled)
        if enabled and getattr(self, '_guided_takeoff_mode', False):
            # 互斥：開「點圖飛到」時自動關「點圖起飛」
            self.sitl_hud.btn_guided_takeoff_pt.setChecked(False)
        self.statusBar().showMessage(
            '🎯 點圖飛到已開啟：在地圖點一下 → 長機 GUIDED 飛往該點（僚機跟）'
            if enabled else '點圖飛到已關閉', 5000)

    def _send_guided_goto(self, lat: float, lon: float, alt: float = 60.0):
        """送 GUIDED 飛到。尊重 HUD 指令對象：選單機 → 只該機飛往；
        全部 → 送長機（sysid 最小者），僚機由蜂群節點追蹤其 geopose 跟隨。"""
        if not self._sitl_links:
            return
        leader, wingmen = self._resolve_flight_target()
        if leader is None:
            QMessageBox.information(
                self, 'GUIDED', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return
        try:
            leader.guided_goto(lat, lon, alt)
            tail = '，僚機跟隨' if wingmen else '（單機）'
            self.statusBar().showMessage(
                f'🎯 UAV-{getattr(leader, "sysid_label", 1)} GUIDED 飛往 '
                f'({lat:.5f},{lon:.5f}) @ {alt:.0f}m{tail}', 6000)
        except Exception as e:
            logger.error(f'[GUIDED] guided_goto 失敗: {e}')

    # ── 3D 右鍵：先選動作（到達 / 攻擊），再走各自的流程 ──────────────
    def on_rclick_guided_goto(self, lat: float, lon: float):
        """3D 地圖右鍵 → 動作選單。

        兩個動作的前置條件、確認方式、後續狀態顯示完全不同，所以是**選單**
        而不是塞進高度框裡的一個選項：
          * 「攻擊」根本不需要高度（幾何由 tot_node 決定）。跳一個高度框
            再把它丟掉，正是這個專案最痛恨的靜默行為。
          * 兩者的前置檢查不一樣（到達看 SITL 連線，攻擊看 ROS2 圖上有沒有
            待命中的 tot_node），擋下來的訊息也必須不一樣。

        「飛到此點」= 既有行為，一個位元都沒動（見 _rclick_guided_arrive）。
        高度框輸入 0 仍然是「攻擊」的快捷 —— 但一定會跳確認框。
        """
        mode = self._rclick_pick_action(lat, lon)
        if mode is None:                                  # 按 Esc / 點別處
            return
        if mode == rclick_strike.MODE_STRIKE:
            self.rclick_strike_at(lat, lon)
            return
        self._rclick_guided_arrive(lat, lon)

    # ── 地圖右鍵的 STOT 打擊入口（2026-09-02 暫時關閉）────────────────
    # 現在專心做投彈機群，右鍵不要再跳出紅心/STOT 那條路。
    # 要開回來把這個改成 True 就好 —— 底層的 rclick_strike_at /
    # stot_ros2_controller 都沒有動，只是拿掉入口。
    # ⚠️ 兩個入口要一起關：選單項，以及高度輸入框「輸入 0 = STOT 打擊」
    #    的快捷。只關選單的話，打 0 還是會進 STOT。
    ENABLE_STOT_RCLICK_STRIKE = False

    def _rclick_pick_action(self, lat: float, lon: float):
        """右鍵動作選單。回傳 MODE_ARRIVE / MODE_STRIKE / None（取消）。"""
        from PyQt6.QtGui import QCursor
        from PyQt6.QtWidgets import QMenu
        menu = QMenu(self)
        head = menu.addAction(f'📍 {lat:.7f}, {lon:.7f}')   # 攤開座標，方便對帳
        head.setEnabled(False)
        menu.addSeparator()
        act_goto = menu.addAction('✈️  飛到此點（GUIDED）')
        act_hit = None
        if self.ENABLE_STOT_RCLICK_STRIKE:
            act_hit = menu.addAction('🎯  以此點執行 STOT 打擊')
        picked = menu.exec(QCursor.pos())
        if picked is act_goto:
            return rclick_strike.MODE_ARRIVE
        if act_hit is not None and picked is act_hit:
            return rclick_strike.MODE_STRIKE
        return None

    def _rclick_guided_arrive(self, lat: float, lon: float):
        """3D 地圖右鍵 → 跳高度輸入框 → 送長機 GUIDED。
        長機在地面：長機帶頭起飛到該高度 → 僚機 3s 內接續 → 長機飛往該點（僚機 V 跟隨）。
        長機已在空中：直接 GUIDED 飛往該點。

        ⚠️ 這一段是**原本 on_rclick_guided_goto 的內容原樣搬過來**，行為零改動。
           唯一的差別是高度下限從 5.0 放寬到 0.0，因為 0 是「攻擊」的快捷、
           必須進得來（進來之後就被 mode_for_altitude 接走，不會流到 guided_goto）。
        """
        if not self._sitl_links:
            QMessageBox.information(self, 'GUIDED', '尚未連線任何 SITL / 蜂群')
            return
        from PyQt6.QtWidgets import QInputDialog
        _hint = ('\n（輸入 0 = 改走 STOT 打擊，會再跳一次確認）'
                 if self.ENABLE_STOT_RCLICK_STRIKE else '')
        # STOT 入口關閉時高度下限提到 ALT_ARRIVE_MIN：0 不再有特殊語意，
        # 讓它可輸入卻沒有作用只會讓人困惑。
        _lo = 0.0 if self.ENABLE_STOT_RCLICK_STRIKE \
            else float(rclick_strike.ALT_ARRIVE_MIN)
        alt, ok = QInputDialog.getDouble(
            self, 'GUIDED 目標高度',
            f'目標點 ({lat:.5f}, {lon:.5f})\n指定高度 (m，相對起飛點):' + _hint,
            float(getattr(self, '_last_guided_alt', 80.0)), _lo, 2000.0, 0)
        if not ok:
            return
        # 高度 0 = 攻擊快捷。**不可以默默改語意** ⇒ rclick_strike_at 一定會跳確認框。
        if (self.ENABLE_STOT_RCLICK_STRIKE
                and rclick_strike.mode_for_altitude(alt)
                == rclick_strike.MODE_STRIKE):
            self.rclick_strike_at(lat, lon, via_alt_zero=True)
            return
        if not rclick_strike.arrive_altitude_ok(alt):
            # 0 < alt < 5 是舊 UI（下限 5.0）產生不出來的值。不要靜默夾到 5，
            # 那會讓使用者以為自己指定的高度生效了。
            QMessageBox.warning(
                self, 'GUIDED',
                f'高度 {alt:.0f} m 太低（合法範圍 '
                f'{rclick_strike.ALT_ARRIVE_MIN:.0f}–'
                f'{rclick_strike.ALT_ARRIVE_MAX:.0f} m）。\n'
                + (f'0 是「STOT 打擊」的快捷，其餘小於 '
                   f'{rclick_strike.ALT_ARRIVE_MIN:.0f} m 的值一律不送。'
                   if self.ENABLE_STOT_RCLICK_STRIKE
                   else f'小於 {rclick_strike.ALT_ARRIVE_MIN:.0f} m '
                        f'的值一律不送。'))
            return
        self._last_guided_alt = alt
        # 尊重 HUD 指令對象：以「選取機（或長機）」的遙測判斷是否已升空
        target, _ = self._resolve_flight_target()
        if target is None:
            QMessageBox.information(
                self, 'GUIDED', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return
        frame = target.get_latest_telemetry()
        airborne = bool(frame and frame.armed and frame.alt_rel > 15.0)
        if airborne:
            self._send_guided_goto(lat, lon, alt)        # 已在空中 → 直接飛往
        else:
            self._rclick_takeoff_goto(lat, lon, alt)     # 地面 → 帶頭起飛後飛往

    def _rclick_takeoff_goto(self, lat: float, lon: float, alt: float):
        """長機帶頭起飛（僚機 3s 內接續），長機升空後 GUIDED 飛往 (lat,lon,alt)。
        尊重 HUD 指令對象：選單機 → 只該機起飛飛往（無僚機）。"""
        from PyQt6.QtCore import QTimer
        leader, wingmen = self._resolve_flight_target()
        if leader is None:
            QMessageBox.information(
                self, 'GUIDED', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return
        try:
            leader.guided_takeoff(alt)
        except Exception as e:
            logger.error(f'[GUIDED] 長機起飛失敗: {e}')
            return
        self.statusBar().showMessage(
            f'🛫 長機 UAV-{getattr(leader, "sysid_label", 1)} 起飛至 {alt:.0f}m → '
            f'3s 後 {len(wingmen)} 架僚機接續，升空後飛往目標點', 9000)

        def _release_wingmen():
            for w in wingmen:
                try:
                    w.guided_takeoff(alt)
                except Exception as e:
                    logger.error(f'[GUIDED] 僚機起飛失敗: {e}')
            self.statusBar().showMessage(f'🛫 {len(wingmen)} 架僚機接續起飛、組 V 編隊', 6000)
        QTimer.singleShot(3000, _release_wingmen)        # 僚機 3s 內接續起飛

        # 輪詢長機升空 → 切 GUIDED 飛往目標點
        self._pending_goto = {'leader': leader, 'lat': lat, 'lon': lon, 'alt': alt, 'ticks': 0}
        if getattr(self, '_goto_timer', None) is None:
            self._goto_timer = QTimer(self)
            self._goto_timer.timeout.connect(self._tick_pending_goto)
        self._goto_timer.start(1000)

    def _tick_pending_goto(self):
        pg = getattr(self, '_pending_goto', None)
        if not pg:
            self._goto_timer.stop()
            return
        pg['ticks'] += 1
        leader = pg['leader']
        frame = leader.get_latest_telemetry()
        alt = frame.alt_rel if frame else 0.0
        if alt >= min(pg['alt'] * 0.7, 40.0):            # 升空到目標 7 成或 40m → 飛往目標
            try:
                leader.guided_goto(pg['lat'], pg['lon'], pg['alt'])
                self.statusBar().showMessage(
                    f'🎯 長機升空 {alt:.0f}m → GUIDED 飛往目標點 '
                    f'({pg["lat"]:.5f},{pg["lon"]:.5f}) @ {pg["alt"]:.0f}m，僚機 V 跟隨', 7000)
            except Exception as e:
                logger.error(f'[GUIDED] 長機飛往目標失敗: {e}')
            self._goto_timer.stop()
            self._pending_goto = None
        elif pg['ticks'] >= 60:
            self._goto_timer.stop()
            self._pending_goto = None

    def _on_fence_export(self):
        """
        匯出當前自動建構的電子圍籬（QGC .plan / Mission Planner .fen）。
        每個 Tab 上的「🛡 匯出電子圍籬」按鈕都會走到這裡。
        """
        from PyQt6.QtWidgets import QFileDialog
        from datetime import datetime
        bundle = getattr(self.map_widget, 'last_fence_bundle', None)
        if bundle is None:
            try:
                from ui.widgets.mil_std_message import MilStdMessageBox, Severity
                MilStdMessageBox(
                    severity=Severity.CAUTION,
                    title='電子圍籬',
                    what_happened='系統尚未自動建構任何電子圍籬',
                    detail='規劃路徑後（任一演算法、DCCPP 或蜂群打擊），系統會立即自動建構矩形圍籬。',
                    recommended_action='請先在「基本演算法」或「DCCPP」分頁規劃一條路徑，再回此頁匯出。',
                    parent=self,
                ).exec()
            except Exception:
                QMessageBox.information(
                    self, '電子圍籬',
                    '尚無圍籬可匯出。\n請先規劃路徑，系統會自動建立圍籬。'
                )
            return

        default_name = f'fence_{datetime.now():%Y%m%d_%H%M%S}'
        path, _flt = QFileDialog.getSaveFileName(
            self, '匯出電子圍籬', default_name,
            'QGroundControl Plan (*.plan);;Mission Planner Fence (*.fen);;'
            'JSON 全部資料 (*.json);;All files (*)'
        )
        if not path:
            return
        try:
            from mission.geofence_manager import GeofenceConstraintManager
            p_lower = path.lower()
            if p_lower.endswith('.fen'):
                out = GeofenceConstraintManager.export_mission_planner_fen(bundle, path)
                fmt = 'Mission Planner .fen'
            elif p_lower.endswith('.plan'):
                out = GeofenceConstraintManager.export_qgc_plan(bundle, path)
                fmt = 'QGroundControl .plan'
            else:
                # 預設用 .plan 格式（較完整含 mission + geoFence）
                if not p_lower.endswith('.json'):
                    path = path + '.plan'
                out = GeofenceConstraintManager.export_qgc_plan(bundle, path)
                fmt = 'QGroundControl .plan (預設格式)'
            self.statusBar().showMessage(f'🛡️ {fmt} 已匯出: {out}', 6000)
            QMessageBox.information(
                self, '匯出完成',
                f'格式：{fmt}\n路徑：{out}\n\n'
                f'圍籬：{bundle.geofence.vertex_count()} 頂點 {bundle.geofence.method.upper()}\n'
                f'高度：[{bundle.geofence.alt_min_m:.0f}, {bundle.geofence.alt_max_m:.0f}] m\n'
                f'Buffer：{bundle.geofence.buffer_radius_m:.0f} m'
            )
        except Exception as e:
            logger.error(f'[Fence] 匯出失敗: {e}', exc_info=True)
            QMessageBox.warning(self, '匯出失敗', f'寫入檔案失敗：\n{e}')

    # ── 飛控任務讀回（= Mission Planner 的 Ctrl+F → Read WPs）─────────
    def on_sitl_download_mission(self):
        """向飛控索取目前存著的任務航線，畫到地圖上。

        結果是非同步的：`SITLLink.mission_downloaded` 在建立 link 時就接到
        `_on_mission_downloaded`（見 on_sitl_connect），所以這裡只負責發起。
        """
        if not self._sitl_links:
            QMessageBox.information(self, 'SITL', '尚未連線任何 SITL')
            return
        links = self._target_links()
        # ⚠️ 多機同時下載會讓後畫的蓋掉先畫的（疊加層只有一份），而且
        #    看起來像「只讀到一架」。指定指令對象才是這個功能的正常用法。
        if len(links) > 1:
            QMessageBox.information(
                self, 'SITL',
                f'目前指令對象是「全部」（{len(links)} 架）。\n'
                '任務航線疊加層只有一份，多架同時讀回會互相覆蓋。\n\n'
                '請先在 HUD 上點選單一 UAV 卡片，再讀回任務。')
            return
        for link in links:
            try:
                link.download_mission()
            except Exception as e:
                logger.error(f'[SITL] 讀回任務失敗: {e}', exc_info=True)
        self.statusBar().showMessage('正在向飛控讀回任務航線…', 3000)

    def _on_mission_downloaded(self, sysid: int, items: list):
        """飛控任務讀回成功 → 疊到地圖上。"""
        from mission import mission_view as _mv
        try:
            path2d = _mv.mission_to_path_2d(items)
            path3d = _mv.mission_to_path_3d(items)
            loiters = _mv.mission_loiters(items)
            home = _mv.mission_home(items)
            summary = _mv.mission_summary(items)

            if hasattr(self.map_widget, 'display_fc_mission'):
                self.map_widget.display_fc_mission(path2d, path3d, loiters,
                                                   home=home)
            self._fc_mission = {'sysid': int(sysid), 'items': list(items)}

            # ⚠️ 「n 項但 0 點可繪製」是一個真實且會發生的狀態（整份都是 DO_*，
            #    或全部是 lat=lon=0 的純指令項）。這時地圖上什麼都不會出現，
            #    如果狀態列只說「下載完成」，使用者會去查地圖 —— 查錯地方。
            if items and not path2d:
                QMessageBox.information(
                    self, '任務讀回',
                    f'UAV-{sysid} 的任務有 {len(items)} 項，但**沒有一項帶座標**\n'
                    '（全是 DO_*/CONDITION_* 之類的純指令項）。\n'
                    '地圖上不會有航線 —— 這不是地圖壞掉。')
            self.statusBar().showMessage(f'UAV-{sysid} {summary}', 8000)
            logger.info(f'[SITL] UAV-{sysid} {summary}')
        except Exception as e:
            logger.error(f'[SITL] 繪製讀回任務失敗: {e}', exc_info=True)
            self.statusBar().showMessage(f'讀回任務但繪製失敗：{e}', 8000)

    def _on_mission_download_failed(self, sysid: int, reason: str):
        """讀回失敗 —— **不可以**靜默，也不可以畫成空航線。

        空航線是一個合法答案（飛控裡真的沒有任務），把失敗畫成空的，
        使用者就會以為「AUTO 沒有上傳成功」而去查上傳側 —— 查錯地方。
        """
        logger.warning(f'[SITL] UAV-{sysid} 任務讀回失敗: {reason}')
        self.statusBar().showMessage(f'UAV-{sysid} 任務讀回失敗：{reason}', 10000)

    def on_sitl_upload_fence(self):
        """把目前自動建構的電子圍籬上傳到所有連線的 SITL。"""
        if not self._sitl_links:
            QMessageBox.information(self, 'SITL', '尚未連線任何 SITL')
            return
        bundle = getattr(self.map_widget, 'last_fence_bundle', None)
        if bundle is None:
            QMessageBox.warning(self, 'SITL',
                '尚無圍籬可上傳。\n建議：先規劃任何路徑，系統會自動建立圍籬，再點此上傳。')
            return
        # 依 HUD 指令對象縮限範圍（選中單機 → 只上傳該機）
        links = self._target_links()
        if not links:
            QMessageBox.information(
                self, 'SITL', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return
        for link in links:
            try:
                link.upload_fence(bundle)
            except Exception as e:
                logger.error(f'[SITL] 上傳圍籬失敗 sysid={getattr(link,"sysid_label","?")}: {e}')
        self.statusBar().showMessage(
            f'🛡️ 已送出圍籬上傳請求到 {len(links)} 台 SITL', 5000
        )

    def on_sitl_upload_mission(self):
        """把當前路徑上傳到所有連線的 SITL（支援 Diamond 蜂群打擊 / DCCPP / 一般路徑）"""
        if not self._sitl_links:
            QMessageBox.information(self, 'SITL', '尚未連線任何 SITL')
            return
        # ── 優先：菱形蜂群打擊（每架獨立任務 + 友善參數）──────
        if getattr(self, '_diamond_plans', None):
            return self._on_diamond_strike_sitl_upload()
        # 從 map_widget.paths 收集所有航點；若無則改抓 DCCPP / swarm 結果
        wps = []
        try:
            default_alt = float(self.parameter_panel.get_parameters().get('altitude', 50))
            paths = getattr(self.map_widget, 'paths', [])
            for path in paths:
                for p in path:
                    if len(p) >= 3:
                        wps.append((float(p[0]), float(p[1]), float(p[2])))
                    elif len(p) == 2:
                        wps.append((float(p[0]), float(p[1]), default_alt))

            # DCCPP / 群飛結果（display_swarm_raw 不會塞進 self.paths）
            if not wps:
                swarm = getattr(self.map_widget, '_swarm_data', None) \
                        or getattr(getattr(self.map_widget, 'map_2d', None), '_swarm_data', None)
                if swarm:
                    for d in swarm.get('drones', []):
                        for p in d.get('path', []):
                            if len(p) >= 3:
                                wps.append((float(p[0]), float(p[1]), float(p[2])))
                            elif len(p) == 2:
                                wps.append((float(p[0]), float(p[1]), default_alt))

            # DCCPP 多機：每架 SITL 綁到各自的 UAV 路徑
            # ★ 修 bug：原本用 link_idx 為 key（依 self._sitl_links 順序），
            #   但 link 順序不一定等於 sysid 順序（使用者分批啟動 / 重連時錯亂）；
            #   且 link 數 > 路徑數時會用 max(keys) 讓多 link 共用同一份 mission
            #   → 兩架固定翼上傳到同一份任務的 bug。
            #   改為以 uav_id 為 key，上傳時用 link.sysid_label 對應 UAV 路徑。
            per_uav_wps: dict = {}     # {uav_id: (link_idx_for_log, wps_list)}

            if not wps and getattr(self, '_dccpp_result', None):
                # DCCPP：輸出完整任務指令序列，讓 SITL AUTO 模式完整起飛→任務→降落。
                # 每個 tuple: (lat, lon, alt, cmd, p1, p2, p3, p4)
                #
                # ── MAVLink 任務指令對照 ──
                # CMD_WP   = 16  (MAV_CMD_NAV_WAYPOINT)       → 普通航點
                # CMD_TO   = 22  (MAV_CMD_NAV_TAKEOFF)        → 固定翼跑道起飛
                # CMD_LAND = 21  (MAV_CMD_NAV_LAND / RTL)     → 固定翼降落
                # CMD_SPD  = 178 (MAV_CMD_DO_CHANGE_SPEED)    → 設定巡航速度
                # ── VTOL QuadPlane 專用 ──
                # CMD_VTOL_TO   = 84   (MAV_CMD_NAV_VTOL_TAKEOFF)    → 垂直起飛
                # CMD_VTOL_LAND = 85   (MAV_CMD_NAV_VTOL_LAND)       → 垂直降落
                # (3000 MAV_CMD_DO_VTOL_TRANSITION 不使用)  → 飛行模式轉換
                #   param1=3 → MC（多旋翼）  param1=4 → FW（固定翼）
                CMD_WP, CMD_TO, CMD_LAND, CMD_SPD = 16, 22, 21, 178
                CMD_VTOL_TO, CMD_VTOL_LAND = 84, 85
                params = self.parameter_panel.get_parameters()
                turn_radius = float(params.get('turn_radius', 50.0))
                cruise_speed = float(params.get('speed', 15.0))
                # ⚠ 優先用「這份規劃當時實際使用的」值，別讀別的分頁。
                #   見規劃端寫入 cruise_speed_mps / turn_radius_m 的註解。
                cruise_speed, turn_radius = resolve_upload_speed_radius(
                    self._dccpp_result, params,
                )

                # 判斷是否為 VTOL 載具 → 使用垂直起降指令
                is_vtol = self.flight_params.get('vehicle_type', '') == 'vtol'

                # VTOL 轉換高度：多旋翼垂直爬升到此高度後，再轉換為固定翼巡航
                # 預設 100m；優先讀取 UI 參數面板的設定值
                vtol_transition_alt = 100.0
                if is_vtol:
                    # 優先用「這份規劃當時實際使用的」轉換高度，
                    # 才不會上傳一份與畫面軌跡不同剖面的任務。
                    planned = (self._dccpp_result or {}).get('vtol_transition_alt_m')
                    if planned:
                        vtol_transition_alt = float(planned)
                    else:
                        try:
                            vtol_transition_alt = float(
                                self.parameter_panel.vtol_transition_alt_spin.value()
                            )
                        except Exception:
                            pass
                    logger.info(
                        f'[SITL] VTOL 轉換高度: {vtol_transition_alt}m '
                        f'(多旋翼垂直爬升 → 轉換 FW → 巡航)'
                    )

                ap = self._dccpp_result.get('assembled_paths') or {}
                # 依 uav_id 排序 → 對應 SITL instance 0/1/2...
                sorted_paths = sorted(ap.items(), key=lambda kv: kv[0])

                def _build_wps_for_path(first_path):
                    """把一個 BuiltPath 轉成 (lat,lon,alt,cmd,p1,p2,p3,p4) 列表。

                    VTOL 模式任務序列（ArduPilot QuadPlane）：
                      DO_CHANGE_SPEED → NAV_VTOL_TAKEOFF → NAV_WAYPOINT × N
                      → NAV_VTOL_LAND
                    （不插 DO_VTOL_TRANSITION：飛控自己會在飛往下一航點時前向
                      過渡、在 NAV_VTOL_LAND 進場時轉回 VTOL。）

                    固定翼模式任務序列：
                      DO_CHANGE_SPEED → NAV_TAKEOFF → NAV_WAYPOINT × N → NAV_LAND
                    """
                    wps_local = []
                    wp_list = getattr(first_path, 'waypoints', None) or []
                    if not wp_list:
                        return wps_local
                    last_landing_idx = -1
                    has_takeoff_seg = False
                    for _i, _w in enumerate(wp_list):
                        _nm = getattr(getattr(_w, 'segment_type', None), 'name', '')
                        if _nm == 'LANDING':
                            last_landing_idx = _i
                        if _nm == 'TAKEOFF':
                            has_takeoff_seg = True

                    # 1) 設定巡航速度
                    wps_local.append((0.0, 0.0, 0.0, CMD_SPD,
                                      0.0, cruise_speed, -1.0, 0.0))

                    # 2) 起飛指令
                    takeoff_emitted = False
                    if not has_takeoff_seg and wp_list:
                        _w0 = wp_list[0]
                        _alt0 = float(getattr(_w0, 'alt', default_alt) or default_alt)
                        if _alt0 < 20.0:
                            _alt0 = max(default_alt, 50.0)
                        if is_vtol:
                            # ── VTOL QuadPlane 起飛序列 ──────────────────
                            # 階段 1: NAV_VTOL_TAKEOFF (cmd=84)
                            #   多旋翼模式垂直爬升到「轉換高度」
                            #   alt = vtol_transition_alt（例如 100m）
                            #   到達此高度後飛控會自動懸停等待下一指令
                            wps_local.append((float(_w0.lat), float(_w0.lon),
                                              vtol_transition_alt,
                                              CMD_VTOL_TO, 0.0, 0.0, 0.0, 0.0))
                            # 階段 2: 不插 DO_VTOL_TRANSITION(FW)。
                            #   官方 QuadPlane AUTO Missions：NAV_VTOL_TAKEOFF
                            #   到達高度後飛向下一航點時會「as needed」自動前向
                            #   過渡，明確指令是多餘的（與匯出器同一決定）。
                            # 階段 3: 第一個 NAV_WAYPOINT 會帶飛機爬升到任務高度
                            #   例如任務高度 200m → 固定翼以爬升姿態飛向第一航點
                            #   （這一步由後面的 for 迴圈自動產生）
                        else:
                            # 固定翼: NAV_TAKEOFF (cmd=22) → 跑道/手拋起飛
                            # param1 = pitch_angle (最低仰角 10°)
                            wps_local.append((float(_w0.lat), float(_w0.lon), _alt0,
                                              CMD_TO, 10.0, 0.0, 0.0, 0.0))
                        takeoff_emitted = True

                    for idx, wp in enumerate(wp_list):
                        seg_name = getattr(
                            getattr(wp, 'segment_type', None), 'name', ''
                        ).upper()
                        lat = getattr(wp, 'lat', None)
                        lon = getattr(wp, 'lon', None)
                        alt = float(getattr(wp, 'alt', default_alt) or default_alt)
                        hdg = float(getattr(wp, 'heading_compass_deg', 0.0) or 0.0)
                        if lat is None or lon is None:
                            continue
                        lat = float(lat); lon = float(lon)

                        if seg_name == 'TAKEOFF':
                            if idx == 0:
                                continue
                            if not takeoff_emitted:
                                if is_vtol:
                                    # VTOL: 垂直起飛到轉換高度 → 轉 FW
                                    # 同上：不插 DO_VTOL_TRANSITION(FW)，飛控自動過渡
                                    wps_local.append((lat, lon, vtol_transition_alt,
                                                      CMD_VTOL_TO, 0.0, 0.0, 0.0, 0.0))
                                else:
                                    wps_local.append((lat, lon, alt, CMD_TO,
                                                      10.0, 0.0, 0.0, 0.0))
                                takeoff_emitted = True
                            else:
                                wps_local.append((lat, lon, alt, CMD_WP,
                                                  0.0, turn_radius, 0.0, 0.0))
                        elif seg_name == 'LANDING':
                            if idx == last_landing_idx:
                                if is_vtol:
                                    # ── VTOL QuadPlane 降落 ──────────────────
                                    # 只發 NAV_VTOL_LAND (cmd=85)。
                                    # ⚠ 舊版在它前面多塞一個「降落點上空 @ 轉換
                                    #   高度」的 NAV_WAYPOINT ⇒ 進場段變 0 m ⇒
                                    #   PreArm「VTOL land too short, min 108m」
                                    #   拒絕解鎖（也是上傳版比匯出版多一項重複
                                    #   航點的來源）。固定翼進場 + airbrake +
                                    #   垂直下降都由 NAV_VTOL_LAND 自己處理，
                                    #   規劃器已把進場定位點放在 land_approach_m
                                    #   外，這裡不需要再補。
                                    wps_local.append((lat, lon, 0.0, CMD_VTOL_LAND,
                                                      0.0, 0.0, 0.0, hdg))
                                else:
                                    wps_local.append((lat, lon, 0.0, CMD_LAND,
                                                      0.0, 0.0, 0.0, hdg))
                            elif (is_vtol
                                  and abs(alt - vtol_transition_alt) < 1e-6
                                  and last_landing_idx >= 0
                                  and abs(lat - float(wp_list[last_landing_idx].lat)) < 1e-9
                                  and abs(lon - float(wp_list[last_landing_idx].lon)) < 1e-9):
                                # 「降落點正上方 @ 轉換高度」懸停點：跳過，
                                # 理由同上（與匯出器的鏡像跳過一致）。
                                continue
                            else:
                                wps_local.append((lat, lon, alt, CMD_WP,
                                                  0.0, turn_radius, 0.0, 0.0))
                        else:
                            wps_local.append((lat, lon, alt, CMD_WP,
                                              0.0, turn_radius, 0.0, 0.0))
                    return wps_local

                # ── HV-DCCPP：改用轉接層的 8-tuple（手冊 §12.3）──
                #   _build_wps_for_path 的 TAKEOFF 分支先做 `if idx == 0: continue`
                #   才檢查 takeoff_emitted，而 hv 縮點後的路徑起飛段**只有一個航點**
                #   且就在 idx 0 ⇒ 整份任務不會有起飛指令，飛控直接從第一個
                #   NAV_WAYPOINT 開始。to_upload_tuples 在 idx 0 照樣輸出
                #   NAV_VTOL_TAKEOFF + DO_VTOL_TRANSITION(4)。
                _hv = getattr(self, '_hv_coordinator', None)
                _hv_owns_result = (
                    _hv is not None
                    and getattr(_hv, 'last_result', None) is self._dccpp_result)
                if _hv_owns_result:
                    try:
                        per_uav_wps = {
                            int(k): v for k, v in _hv.upload_tuples(
                                is_vtol=is_vtol,
                                vtol_transition_alt_m=float(vtol_transition_alt)).items()
                            if v}
                        logger.info(
                            f'[SITL] HV-DCCPP upload_tuples: '
                            f'{ {k: len(v) for k, v in per_uav_wps.items()} }')
                    except Exception as e:      # noqa: BLE001
                        logger.error(
                            f'[SITL] HV-DCCPP upload_tuples 失敗，退回 _build_wps_for_path：{e}',
                            exc_info=True)
                        _hv_owns_result = False

                # 每個 UAV 建立獨立 wps；以 uav_id 為 key（不再用 link_idx）
                if not _hv_owns_result:
                    for uav_id, bpath in sorted_paths:
                        path_wps = _build_wps_for_path(bpath)
                        if path_wps:
                            per_uav_wps[int(uav_id)] = path_wps
                            logger.info(
                                f'[SITL] DCCPP path 已建立: UAV-{uav_id}  ({len(path_wps)} 點)'
                            )
                # 若只有 1 筆路徑，維持舊行為（讓 wps 非空觸發上傳判斷）
                if per_uav_wps:
                    first_uav_id = sorted(per_uav_wps.keys())[0]
                    wps = per_uav_wps[first_uav_id]
        except Exception as e:
            logger.error(f'[SITL] 收集航點失敗: {e}', exc_info=True)
        if not wps and not per_uav_wps:
            QMessageBox.warning(self, 'SITL', '目前沒有可上傳的路徑，請先預覽生成路徑')
            return

        # 分派：每架 link 用 link.sysid_label 對應自己的 UAV 路徑
        # 修 bug：之前用 link_idx 對應 → 順序錯亂時兩架機可能收到同一份 mission。
        # 改為「按 sysid 對應」確保 sysid=1 飛 UAV-1 路徑、sysid=2 飛 UAV-2 路徑...
        # 沒有對應路徑的 link 會被跳過（並警告），避免重複上傳。
        if per_uav_wps:
            uploaded = []   # [(sysid, uav_id, count)]
            skipped = []    # [(sysid, reason)]
            sorted_uav_ids = sorted(per_uav_wps.keys())
            used_uav_ids = set()    # 同一份 path 不重複上傳
            for link_idx, link in enumerate(self._sitl_links):
                sysid = int(getattr(link, 'sysid_label', link_idx + 1) or (link_idx + 1))
                # 1) 直接用 sysid 對應 uav_id 找路徑
                if sysid in per_uav_wps and sysid not in used_uav_ids:
                    target_uav_id = sysid
                # 2) 沒有完全對應的 sysid → 找尚未使用的最小 uav_id 作為 fallback
                #    (常見情境：UAV id 從 1..N 連續但 sysid 從 0 起算)
                else:
                    remaining = [u for u in sorted_uav_ids if u not in used_uav_ids]
                    if not remaining:
                        skipped.append((sysid, 'all paths uploaded'))
                        logger.warning(
                            f'[SITL] sysid={sysid} 無可用 DCCPP 路徑，跳過'
                        )
                        continue
                    target_uav_id = remaining[0]
                used_uav_ids.add(target_uav_id)
                path_wps = per_uav_wps[target_uav_id]
                link.upload_mission(path_wps)
                uploaded.append((sysid, target_uav_id, len(path_wps)))
                logger.info(
                    f'[SITL] sysid={sysid} ← UAV-{target_uav_id} ({len(path_wps)} 點)'
                )
            self.statusBar().showMessage(
                f'已為 {len(uploaded)}/{len(self._sitl_links)} 台 SITL 上傳獨立 '
                f'DCCPP 路徑（{len(skipped)} 跳過）', 6000
            )
        else:
            for link in self._sitl_links:
                link.upload_mission(wps)
            self.statusBar().showMessage(
                f'已對 {len(self._sitl_links)} 個 SITL 上傳 {len(wps)} 個航點', 5000
            )

    def _on_main_tab_changed(self, index: int):
        """切換右側分頁時，調整任務面板顯示內容
        index 0 = 基本演算法 → 顯示完整預覽 / 匯出
        其他   → 隱藏預覽 / 匯出（避免重複），保留清除類按鈕
        """
        try:
            mp = self.mission_panel
            is_basic = (index == 0)
            mp.preview_btn.setVisible(is_basic)
            mp.export_btn.setVisible(is_basic)
            if hasattr(mp, '_stats_card'):
                mp._stats_card.setVisible(is_basic)
        except Exception as e:
            logger.warning(f'切換分頁時調整 mission_panel 失敗: {e}')
        # VTOL 偵察分頁的「偵察區」狀態要跟主視窗的 corners 同步。
        # 邊界點是在別的分頁 / 地圖上畫的，切過來時補一次即可。
        try:
            if hasattr(self, 'vtol_recon_panel'):
                self.vtol_recon_panel.set_corner_count(len(self.corners))
        except Exception as e:
            logger.warning(f'同步 VTOL 偵察分頁邊界點失敗: {e}')

    # ══════════════════════════════════════════════════════════════════
    #  VTOL 群飛偵察分頁
    # ══════════════════════════════════════════════════════════════════

    def _on_vtol_recon_plan(self, params: dict):
        """VTOL 偵察分頁按下「執行 ⋯ 偵察規劃」。

        這一頁有兩套規劃器可選（面板的「規劃演算法」下拉）：
          * `planner='dccpp'`    —— 專案既有實作，同步跑（1 秒等級）。
          * `planner='hv_dccpp'` —— 本專案的異構雙構型版本，丟背景執行緒
            （12 台的大情境要數十秒，同步跑會讓視窗「沒有回應」）。
        兩條路徑都走 `_on_dccpp_coverage_requested`，結果一樣是那 6 鍵字典，
        所以下面整理摘要表的程式碼共用。
        """
        panel = self.vtol_recon_panel
        # ⚠ 多區模式：已完成的區也算數（見 survey_area_polygons）
        if not self.survey_area_polygons():
            QMessageBox.warning(self, "尚未劃定偵察區",
                                "請先在地圖上標出至少 3 個邊界點。")
            panel.set_plan_status("尚未規劃")
            return
        if params.get('fov_width_m', 0.0) <= 0.0:
            QMessageBox.warning(self, "感測器參數無效",
                                "有效條帶間距為 0，請檢查相機 FOV / 重疊率設定。")
            panel.set_plan_status("感測器參數無效")
            return

        use_hv = bool(params.get('hv_dccpp'))
        logger.info(f"[VTOL 偵察] 規劃演算法 = {params.get('planner', 'dccpp')}"
                    f"（{'背景執行緒' if use_hv else '同步'}）")
        panel.set_plan_status(
            ("HV-DCCPP 規劃中 …（大情境需數十秒）" if use_hv else "DCCPP 規劃中 …"))
        # ★ 結果一律由 on_done 接手：HV 走背景執行緒，這個函式回來時還沒有結果。
        #   以前這裡直接讀 self._dccpp_result，那在非同步下會讀到上一次的值而且不報錯。
        self._on_dccpp_coverage_requested(
            params, run_async=use_hv,
            on_done=lambda ok: self._on_vtol_recon_planned(params, ok))

    def _on_vtol_recon_planned(self, params: dict, ok: bool):
        """規劃（不論哪一套）顯示完成後：整理每架的航程／時間回填面板。"""
        panel = self.vtol_recon_panel
        result = getattr(self, '_dccpp_result', None) if ok else None
        if not result:
            panel.set_plan_status("規劃未產生結果（詳見日誌）")
            panel.set_plan_result([], "規劃未產生結果（詳見日誌）")
            return

        # ── VTOL 專屬 3D 視覺化 ──────────────────────────────
        # 先前 drawVTOLSwarmPaths（垂直段 + MC/FW 轉換點標記）只掛在**匯出**
        # 流程上，所以規劃完看到的是一般折線，非得按「匯出航點」才會出現轉換
        # 標記。規劃當下就畫，畫面才會跟實際規劃的剖面一致。
        try:
            cesium = self._get_cesium_widget()
            if cesium is not None:
                from mission.vtol_mission_exporter import VTOLMissionExporter
                _viz = VTOLMissionExporter(
                    transition_alt_m=float(params.get('transition_alt', 40.0)),
                    cruise_speed_ms=float(params.get('speed', 18.0)),
                    turn_radius_m=float(params.get('turn_radius', 60.0)),
                )
                cesium.draw_vtol_swarm_paths(
                    _viz.to_cesium_data(result.get('assembled_paths') or {},
                                        float(params.get('altitude', 100.0)))
                )
        except Exception as _ve:
            logger.warning(f'[VTOL 偵察] 3D 視覺化失敗: {_ve}', exc_info=True)

        speed = max(0.1, float(params.get('speed', 18.0)))
        # 表格與 makespan 必須用**同一支**時間估算，否則畫面上兩個數字又對不起來。
        # estimate_flight_time_s 每段取 max(水平/巡航, 垂直/爬升率)，
        # 所以 VTOL 的純垂直起降段不會被算成 0 秒。
        import math as _m
        from core.dccpp.dccpp_path_builder import estimate_flight_time_s
        from core.dccpp.vtol_profile import Q_PILOT_SPD_UP
        rows = []
        for uav_id, apath in sorted((result.get('assembled_paths') or {}).items(),
                                    key=lambda kv: kv[0]):
            wps = getattr(apath, 'waypoints', None) or []
            # 航程用 3D 長度：VTOL 的垂直爬升也是真的飛了那段距離
            dist_m = 0.0
            for a, b in zip(wps, wps[1:]):
                h = self._polyline_length_m(
                    [(float(a.lat), float(a.lon)), (float(b.lat), float(b.lon))]
                )
                dist_m += _m.hypot(h, float(b.alt) - float(a.alt))
            t_min = estimate_flight_time_s(apath, speed, Q_PILOT_SPD_UP) / 60.0
            # 構型：異構規劃時每一列要看得出是 Pioneer 還是 HERO
            # （兩者的轉彎半徑 200/120 m、最低作業高度 120/100 m 差很多）
            _cls = getattr(apath, 'cls', None)
            rows.append((uav_id, len(wps), dist_m / 1000.0, t_min,
                         HV_PLATFORM_NAMES.get(_cls or '', '')))

        makespan = float(result.get('total_makespan', 0.0) or 0.0)
        # ⚠ 規劃器可能只用了機隊的一部分（小作業區用不到那麼多台），
        #   而且**不會有任何訊息**說某幾台沒分到工作。要在摘要裡講出來，
        #   否則使用者是在起飛前才發現有飛機沒有任務。
        asked = int(params.get('num_drones', len(rows)) or len(rows))
        idle = max(0, asked - len(rows))
        by_cls = {}
        for r in rows:
            if len(r) > 4 and r[4]:
                by_cls[r[4]] = by_cls.get(r[4], 0) + 1
        cls_txt = ('｜' + '、'.join(f'{k}×{v}' for k, v in sorted(by_cls.items()))
                   if by_cls else '')
        idle_txt = f'｜⚠ {idle} 台沒有分到工作' if idle else ''
        summary = (
            f"規劃完成：{len(rows)}/{asked} 架{cls_txt} ｜ 總航程 "
            f"{sum(r[2] for r in rows):.2f} km ｜ makespan {makespan / 60.0:.1f} min"
            f"{idle_txt}"
        )
        panel.set_plan_result(rows, summary)
        logger.info(f'[VTOL 偵察] {summary}')

    @staticmethod
    def _polyline_length_m(pts: list) -> float:
        """折線長度（公尺），用等距圓柱近似。"""
        import math as _m
        total = 0.0
        for (la1, lo1), (la2, lo2) in zip(pts, pts[1:]):
            dN = (la2 - la1) * 111320.0
            dE = (lo2 - lo1) * 111320.0 * _m.cos(_m.radians((la1 + la2) / 2.0))
            total += _m.hypot(dN, dE)
        return total

    def _on_vtol_recon_verify(self, cfg: dict):
        """覆蓋率驗算：拿規劃出來的航跡對偵察區做幾何覆蓋檢查。"""
        from core.recon import verify_coverage
        panel = self.vtol_recon_panel
        result = getattr(self, '_dccpp_result', None)
        if not result:
            QMessageBox.warning(self, "尚無規劃結果", "請先執行 DCCPP 偵察規劃。")
            return
        # ⚠ 只取**作業段（OPERATION）**的航點。
        #   論文 §2.3 明確定義：coverage 只在 Operation Path 上有效，
        #   Entry / Transfer / 起降段感測器不計覆蓋。把全部航點都拿去驗算
        #   等於把轉場與爬升也算成掃過，覆蓋率會虛高。
        from core.trajectory.dccpp_path_assembler import SegmentLabel
        paths = {}
        for uid, ap in (result.get('assembled_paths') or {}).items():
            wps = getattr(ap, 'waypoints', None) or []
            op = [w for w in wps
                  if getattr(getattr(w, 'segment_type', None), 'name', '') ==
                  SegmentLabel.OPERATION.name]
            paths[int(uid)] = [(float(w.lat), float(w.lon)) for w in op]
        if not any(paths.values()):
            QMessageBox.warning(
                self, "無作業段航跡",
                "規劃結果中沒有 OPERATION 段航點，無法驗算覆蓋率。"
            )
            return
        # ⚠ 驗算用的是**感測器實際掃描寬度**，不是條帶間距。
        #   條帶間距（已扣重疊）是規劃器排條帶用的；感測器每一趟真正看到的
        #   是完整的 swath。拿間距去驗算會低估覆蓋率。
        swath = float(cfg.get('swath_width_m', 0.0)) or float(cfg.get('sweep_width_m', 0.0))
        try:
            report = verify_coverage(
                uav_paths=paths,
                polygon=list(self.corners),
                sweep_width_m=swath,
                cell_size_m=float(cfg.get('cell_size_m', 20.0)),
            )
        except ValueError as e:
            QMessageBox.warning(self, "驗算參數有誤", str(e))
            return
        self._vtol_coverage_report = report
        panel.set_coverage_result(report)
        logger.info(f'[VTOL 偵察] 覆蓋率驗算 — {report.summary()}')
        self.statusBar().showMessage(f'[VTOL 偵察] {report.summary()}', 8000)

    def _on_vtol_recon_heatmap(self):
        """把覆蓋率驗算結果畫成 SAR 熱力圖（紅=漏掃、藍=已充分掃過）。

        重用戰術模組既有的 SAR 熱力圖網格與色階，所以「清除熱力圖」按鈕
        一樣清得掉，視覺語言也與搜救模式一致。
        """
        report = getattr(self, '_vtol_coverage_report', None)
        if report is None:
            QMessageBox.information(self, "尚未驗算",
                                    "請先按「驗算覆蓋率」，再用熱力圖呈現。")
            return
        grid = report.to_display_grid()
        if not grid:
            self.statusBar().showMessage('[VTOL 偵察] 驗算格網為空，無法呈現', 4000)
            return
        try:
            cesium = self._get_visible_cesium_widget("呈現熱力圖")
            if cesium is None:
                return
            cesium.render_coverage_heatmap(grid)
            gaps = sum(1 for v in grid['values'] if v is not None and v <= 0.0)
            self.statusBar().showMessage(
                f"[VTOL 偵察] 覆蓋熱力圖 {grid['rows']}×{grid['cols']}，"
                f"其中 {gaps} 格全漏掃（覆蓋率 {report.coverage_pct:.2f}%）", 8000
            )
            logger.info(
                f'[VTOL 偵察] 熱力圖呈現 {grid["rows"]}x{grid["cols"]}，'
                f'全漏掃 {gaps} 格'
            )
        except Exception as e:
            logger.error(f'[VTOL 偵察] 熱力圖呈現失敗: {e}', exc_info=True)
            QMessageBox.warning(self, "呈現失敗", f"無法呈現覆蓋熱力圖：\n{e}")

    def _on_vtol_recon_show_missed(self):
        """把漏掃格心標到 3D 圖台上。"""
        report = getattr(self, '_vtol_coverage_report', None)
        if report is None or not report.missed_points:
            self.statusBar().showMessage('[VTOL 偵察] 沒有漏掃格可顯示', 4000)
            return
        try:
            cesium = self._get_visible_cesium_widget("標示漏掃格")
            if cesium is None:
                return
            cesium.draw_coverage_gaps(report.missed_points)
            note = '（已抽樣）' if report.missed_truncated else ''
            self.statusBar().showMessage(
                f'[VTOL 偵察] 已標出 {len(report.missed_points)} 個漏掃格{note}', 6000
            )
        except Exception as e:
            logger.error(f'[VTOL 偵察] 標示漏掃格失敗: {e}', exc_info=True)
            QMessageBox.warning(self, "標示失敗", f"無法標示漏掃格：\n{e}")

    def on_sitl_launch(self, vehicle: str, count: int = 1):
        """啟動 N 台專案內 SITL binary

        載具以**使用者在 SITL 分頁選的為準**。若任務（DCCPP / 蜂群打擊）規劃的
        載具與之不符，跳確認框讓使用者決定，不自動覆寫 —— 舊版會靜靜換掉，
        造成「畫面顯示 PLANE、實際跑 ArduCopter」。
        """
        from mission.sitl_launcher import SITLLauncher
        if self._sitl_launcher is None:
            self._sitl_launcher = SITLLauncher()

        try:
            # ── 載具比對 ────────────────────────────────────────────────
            # 原則：使用者在 SITL 分頁選的就是準的。任務結果只能「提醒」，
            # 不能自己把 vehicle 換掉 —— 舊版會靜靜換掉，而下拉還停在原選項，
            # 於是畫面說 PLANE、實際跑 ArduCopter.exe --model quad。
            user_choice = vehicle.upper()
            expected = None      # 任務認為應該用的載具
            source = ''
            diamond_plans = getattr(self, '_diamond_plans', None)
            if diamond_plans:
                expected, source = 'PLANE', '蜂群打擊'
            else:
                dccpp = getattr(self, '_dccpp_result', None)
                if dccpp:
                    # 讀權威欄位，而不是從航點反推。
                    # 舊版用「有沒有 TAKEOFF 段」猜，空路徑 / 無起飛段一律判 COPTER。
                    vt = dccpp.get('vehicle_type')
                    if not vt:
                        # 舊的快取結果沒有這個欄位 → 退回目前面板設定
                        vt = (getattr(self, 'flight_params', None) or {}).get('vehicle_type')
                    expected = {
                        'fixed_wing': 'PLANE',
                        'vtol':       'VTOL',
                        'multirotor': 'COPTER',
                    }.get(vt)
                    source = 'DCCPP'

            if expected and expected != user_choice:
                logger.warning(
                    f'[SITL] 載具不一致：SITL 分頁選 {user_choice}，'
                    f'但{source}任務是 {expected}'
                )
                reply = QMessageBox.question(
                    self, '載具與任務不符',
                    f'SITL 分頁選的是「{user_choice}」，'
                    f'但目前的{source}任務規劃是「{expected}」。\n\n'
                    f'要改用 {expected} 啟動嗎？\n'
                    f'（選「否」則照你選的 {user_choice} 啟動）',
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.Yes,
                )
                if reply == QMessageBox.StandardButton.Yes:
                    vehicle = expected
                    logger.info(f'[SITL] 使用者同意改用 {expected}')
                    # 把下拉同步過去，別讓畫面繼續顯示沒有生效的選項
                    try:
                        self.sitl_hud.set_vehicle(expected)
                    except Exception:
                        pass
                else:
                    logger.info(f'[SITL] 使用者維持 {user_choice}')
        except Exception as _e:
            logger.warning(f'[SITL] 載具比對失敗，沿用使用者選擇: {_e}')

        if not SITLLauncher.is_available():
            QMessageBox.warning(
                self, 'SITL 不可用',
                '找不到 sitl/ArduPlane.exe 或 ArduCopter.exe\n'
                '請從 Mission Planner sitl 資料夾複製到專案 sitl/ 目錄'
            )
            return

        try:
            from config.settings import get_settings
            _s = get_settings()
            lat = _s.map.default_lat
            lon = _s.map.default_lon

            # ── 優先使用當前任務的原點，讓 SITL 飛機生在任務附近 ──
            # 否則 AUTO 後會試圖飛幾十公里到任務點 → 極端 bank/pitch 墜機
            origin_src = None
            spawn_heading = 0.0
            per_uav_homes: list = []  # [(lat, lon, heading), ...] 依 uav_id 排序
            try:
                import math as _m
                # 0. 蜂群打擊（DiamondSwarmStrikePlanner）每架獨立起飛點 — 最高優先
                #    每架 UAV 的 formation_path[0] = slot 起飛點 (slot_lat0, slot_lon0)
                #    formation_path[0] → formation_path[1] 沿 cruise_heading 斜爬升 →
                #    用兩點方位角作為機頭航向，讓 SITL 一生成就對齊跑道方向
                diamond_plans = getattr(self, '_diamond_plans', None)
                if diamond_plans:
                    for plan in sorted(diamond_plans, key=lambda p: p.sysid):
                        fp = plan.formation_path or []
                        if len(fp) < 2:
                            continue
                        u_lat = float(fp[0][0])
                        u_lon = float(fp[0][1])
                        u_hdg = 0.0
                        # 找第一個與 fp[0] 不同位置的點來算航向（爬升段沿 cruise_heading）
                        for nxt in fp[1:]:
                            dN = (float(nxt[0]) - u_lat) * 111320.0
                            dE = (float(nxt[1]) - u_lon) * 111320.0 * _m.cos(_m.radians(u_lat))
                            if abs(dN) + abs(dE) > 1e-6:
                                u_hdg = (_m.degrees(_m.atan2(dE, dN))) % 360.0
                                break
                        per_uav_homes.append((u_lat, u_lon, u_hdg))
                        logger.info(
                            f'[SITL] Diamond UAV{plan.sysid} [{plan.role}] 起飛點: '
                            f'({u_lat:.6f}, {u_lon:.6f}) hdg={u_hdg:.1f}° '
                            f'attack@{plan.attack_bearing_deg:.0f}°'
                        )
                    if per_uav_homes:
                        lat, lon = per_uav_homes[0][0], per_uav_homes[0][1]
                        spawn_heading = per_uav_homes[0][2]
                        origin_src = f'蜂群打擊每架獨立起飛點 ({len(per_uav_homes)} 架)'

                # 1. DCCPP 結果中每架 UAV 的起飛點
                if not per_uav_homes:
                    dccpp = getattr(self, '_dccpp_result', None)
                    if dccpp:
                        ap = dccpp.get('assembled_paths') or {}
                        for uav_id, bpath in sorted(ap.items(), key=lambda kv: kv[0]):
                            wps_u = getattr(bpath, 'waypoints', None) or []
                            if not wps_u:
                                continue
                            u_wp0 = wps_u[0]
                            u_lat = float(u_wp0.lat)
                            u_lon = float(u_wp0.lon)
                            u_hdg = 0.0
                            # ⚠ 不能只看 wps_u[1]：VTOL 起飛第一段是**純垂直**
                            #   （wps_u[0] 與 wps_u[1] 同經緯度），只比第二點會
                            #   算不出方位而靜靜退回 0°，SITL 就會生在錯的朝向。
                            #   往後掃到第一個真的有水平位移的點（與上面
                            #   Diamond 分支同一套作法）。
                            for nxt in wps_u[1:]:
                                dN = (float(nxt.lat) - u_lat) * 111320.0
                                dE = (float(nxt.lon) - u_lon) * 111320.0 * _m.cos(_m.radians(u_lat))
                                if abs(dN) + abs(dE) > 1e-6:
                                    u_hdg = (_m.degrees(_m.atan2(dE, dN))) % 360.0
                                    break
                            per_uav_homes.append((u_lat, u_lon, u_hdg))
                            logger.info(
                                f'[SITL] UAV-{uav_id} 起飛點: '
                                f'({u_lat:.6f}, {u_lon:.6f}) hdg={u_hdg:.1f}°'
                            )
                        if per_uav_homes:
                            lat, lon = per_uav_homes[0][0], per_uav_homes[0][1]
                            spawn_heading = per_uav_homes[0][2]
                            origin_src = f'DCCPP 每架獨立起飛點 ({len(per_uav_homes)} 個)'
                # 2. 當前多邊形的第一個角點
                if origin_src is None and getattr(self, 'corners', None):
                    c0 = self.corners[0]
                    lat, lon = float(c0[0]), float(c0[1])
                    origin_src = '多邊形第一點'
                # 3. Home point
                if origin_src is None and getattr(self, 'home_point', None):
                    lat, lon = float(self.home_point[0]), float(self.home_point[1])
                    origin_src = 'Home 點'
            except Exception as _e:
                logger.warning(f'[SITL] 讀取任務原點失敗，改用預設座標: {_e}')

            logger.info(f'[SITL] 生成座標: ({lat:.6f}, {lon:.6f}) heading={spawn_heading:.1f}°'
                        + (f' (來源: {origin_src})' if origin_src else ' (預設)'))

            # 若有任務每架獨立起飛點（蜂群打擊 / DCCPP），count 自動對齊路徑數量
            if per_uav_homes:
                if count != len(per_uav_homes):
                    logger.info(
                        f'[SITL] 依任務結果將 SITL 數量 {count} → {len(per_uav_homes)} '
                        f'(來源: {origin_src})'
                    )
                count = len(per_uav_homes)

            # 彈出 SITL 啟動設定對話框（per-instance sysid + 嵌入式 fan-out IP/port）
            # 在這裡彈而不是在 sitl_hud._on_launch_clicked() 是因為 count 可能被
            # DCCPP/Diamond 自動展開，需先收斂到實際 instance 數量才能組對話框列數。
            from ui.dialogs.sitl_launch_dialog import SITLLaunchDialog
            dlg_result = SITLLaunchDialog.get_config(
                vehicle=vehicle, count=count, parent=self,
            )
            if dlg_result is None:
                # 使用者取消 → 不啟動 SITL，重置 HUD 按鈕狀態
                logger.info('[SITL] 使用者取消啟動')
                self.sitl_hud.on_sitl_stopped()
                return

            # ── 出生地點預設覆寫 ─────────────────────────────────
            # 使用者在對話框選了常用地點（虎尾 / 桃園機場…）→ 覆寫生成點。
            # 但任務（DCCPP / 蜂群打擊）的每架獨立起飛點優先，不被覆寫，
            # 以免飛機生在遠離任務的地方 AUTO 後暴衝墜機。
            home_override = dlg_result.get('home_override')
            if home_override and not per_uav_homes:
                lat, lon, spawn_heading = (
                    float(home_override[0]), float(home_override[1]),
                    float(home_override[2]),
                )
                origin_src = f'出生地點預設 {dlg_result.get("home_preset", "")}'
                logger.info(
                    f'[SITL] 依出生地點預設覆寫生成座標: '
                    f'({lat:.6f}, {lon:.6f}) heading={spawn_heading:.1f}°'
                )
            elif home_override and per_uav_homes:
                logger.info(
                    '[SITL] 已有任務每架起飛點，忽略出生地點預設覆寫（任務原點優先）'
                )

            # 從 dialog 取出 fan-out 與防火牆設定（停用 fan-out 時 instance_configs=None）
            enable_fanout = bool(dlg_result.get('enable_fanout'))
            instance_configs = (
                list(dlg_result.get('instances') or []) if enable_fanout else None
            )
            auto_firewall = bool(dlg_result.get('auto_firewall')) if enable_fanout else False

            results = self._sitl_launcher.start_multi(
                vehicle=vehicle, count=count,
                lat=lat, lon=lon, alt=0.0, heading=spawn_heading,
                spacing_deg=0.0008,
                homes=per_uav_homes if per_uav_homes else None,
                instance_configs=instance_configs,
                auto_firewall=auto_firewall,
            )
            first_conn = results[0][1]
            self.sitl_hud.on_sitl_launched(f'{vehicle} x{count}', first_conn)
            logger.info(f'[SITL] 內建 {vehicle} x{count} 已啟動')

            # 對每台啟動 MAVLink 連線。
            # sysid_label 用**邏輯序號** logical+1（與 start_multi 實際寫進
            # MAV_SYSID 的值同源），不用 actual_instance —— port 位移時兩者會差
            # 一個位移量，舊寫法就是這樣標成 1/2/3 卻連到 sysid 7/8/9 的機體。
            QApplication.processEvents()
            import time as _t; _t.sleep(0.5)
            for logical, conn, actual in results:
                sysid_label = logical + 1
                if instance_configs and logical < len(instance_configs):
                    cfg = instance_configs[logical] or {}
                    if cfg.get('sysid') is not None:
                        sysid_label = int(cfg['sysid'])
                if actual != logical:
                    logger.info(
                        f'[SITL] i{logical} 因 port 佔用位移至 i{actual}；'
                        f'sysid 仍釘在 {sysid_label}（與飛控 MAV_SYSID 一致）'
                    )
                self.on_sitl_connect(conn, sysid_label=sysid_label,
                                     vehicle_hint=vehicle)
        except Exception as e:
            logger.error(f'[SITL] 啟動失敗: {e}', exc_info=True)
            # 半途失敗會留下已經起來的 SITL 行程占著 port，且 _allocated_instances
            # 不會被清 ⇒ 下一次重試 instance 只會越飄越高。先把這批收乾淨。
            try:
                if self._sitl_launcher is not None:
                    self._sitl_launcher.stop()
            except Exception:
                logger.exception('[SITL] 清理部分啟動的實例失敗')
            QMessageBox.critical(self, 'SITL 啟動失敗', str(e))
            self.sitl_hud.on_sitl_stopped()

    def on_sitl_stop_local(self):
        """停止本地 SITL（含所有實例與連線）"""
        if self._sitl_links:
            self.on_sitl_disconnect()
        if self._sitl_launcher is not None:
            self._sitl_launcher.stop()
        self.sitl_hud.on_sitl_stopped()
        logger.info('[SITL] 內建 SITL 已停止')

    # ══════════════════════════════════════════════════════════════════
    #  戰術模組事件處理
    # ══════════════════════════════════════════════════════════════════

    def _on_elev_slicer_changed(self, min_alt: float, max_alt: float):
        """高程切片參數變更 → 呼叫 3D 地圖更新"""
        self.map_widget.update_elevation_slicer(min_alt, max_alt)
        self.statusBar().showMessage(
            f'[FSDM] 高程切片啟用: {min_alt:.0f}m ~ {max_alt:.0f}m', 5000
        )

    def _on_elev_slicer_cleared(self):
        """清除高程切片"""
        self.map_widget.clear_elevation_slicer()
        self.statusBar().showMessage('[FSDM] 高程切片已清除', 3000)

    def _on_sar_heatmap_init(self, params: dict):
        """初始化搜救機率熱力圖（使用目前邊界區域）"""
        corners = self.map_widget.corners
        if len(corners) < 3:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "無邊界區域",
                                "請先在地圖上定義至少 3 個邊界點作為搜索區域")
            return

        # 計算邊界矩形
        lats = [c[0] for c in corners]
        lons = [c[1] for c in corners]
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)

        grid_size = params.get('grid_size', 20)
        sweep_w = params.get('sweep_width', 50.0)
        quality = params.get('quality', 0.8)

        self.map_widget.init_sar_heatmap(
            lat_min, lat_max, lon_min, lon_max,
            rows=grid_size, cols=grid_size,
            sweep_width=sweep_w, quality=quality,
        )
        ns, ew = heatmap_cell_size_m(lat_min, lat_max, lon_min, lon_max,
                                     grid_size)
        note, ok = heatmap_resolution_note(ns, ew, sweep_w, grid_size)
        self.parameter_panel.set_heatmap_cell_info(note, ok=ok)
        self.statusBar().showMessage(
            f'[SAR] 熱力圖初始化: {grid_size}×{grid_size} 網格 — {note}', 8000
        )

    def _on_sar_heatmap_reset(self):
        """重置搜救熱力圖"""
        self.map_widget.reset_sar_heatmap()
        self.statusBar().showMessage('[SAR] 熱力圖已重置', 3000)

    def _on_sar_heatmap_clear(self):
        """清除搜救熱力圖"""
        self.map_widget.clear_sar_heatmap()
        self.map_widget.clear_fov_cone()
        self.statusBar().showMessage('[SAR] 熱力圖已清除', 3000)

    def _on_fov_cone_toggle(self, enabled: bool):
        """FOV 光錐開關（戰術分析頁）"""
        self._set_fov_cone(enabled, FOV_SOURCE_TACTICAL)

    def _on_vtol_fov_cone_toggle(self, enabled: bool):
        """FOV 光錐開關（VTOL 偵察頁）—— 參數改吃本頁的。"""
        self._set_fov_cone(enabled, FOV_SOURCE_VTOL)

    def _set_fov_cone(self, enabled: bool, source: str):
        """兩頁共用的光錐開關。

        擁有權規則(刻意寫成**單一**進入點,不讓兩邊各自維護狀態):
          * 開啟 ⇒ 這一頁取得擁有權,另一頁的勾選框**跟著取消**
            (取消勾選是唯一能讓使用者看出擁有權易主的方式)。
          * 關閉 ⇒ 只有擁有者才關得掉;非擁有者按了不動光錐,但會說明原因。

        ⚠ 為什麼不允許「兩頁都開、各畫各的」:3D 場景裡是**同一批**
          per-sysid 光錐實體,兩個來源會互相覆寫成最後一筆遙測的參數,
          畫面上只會看到閃爍,看不出是兩個來源在搶。
        """
        other = (FOV_SOURCE_TACTICAL if source == FOV_SOURCE_VTOL
                 else FOV_SOURCE_VTOL)
        src_name = {FOV_SOURCE_TACTICAL: '戰術分析頁',
                    FOV_SOURCE_VTOL: 'VTOL 偵察頁'}

        if not enabled:
            if self._fov_cone_source not in (None, source):
                # 非擁有者關不掉別人的光錐 —— 但要講清楚,不能靜靜沒反應
                self.statusBar().showMessage(
                    f'[FOV] 光錐目前由{src_name[self._fov_cone_source]}控制，'
                    f'這裡的取消不影響它', 4000)
                self._refresh_fov_cone_status()
                return
            self._fov_cone_enabled = False
            self._fov_cone_source = None
            self.map_widget.clear_fov_cone()
            self._last_uav_pose_for_fov.clear()
            self.statusBar().showMessage('[FOV] 光錐關閉', 3000)
            self._refresh_fov_cone_status()
            return

        # ── 開啟 ──
        # ⚠ 2D 視圖時 dual_map_widget.update_fov_cone 仍會把光錐畫進**隱藏的**
        #   3D 場景 ⇒ 看起來像功能壞掉。這裡直接把圖台切過去。
        if self._get_visible_cesium_widget('顯示 FOV 光錐') is None:
            self._uncheck_fov_source(source)
            self._refresh_fov_cone_status()
            return

        self._fov_cone_enabled = True
        self._fov_cone_source = source
        self._uncheck_fov_source(other)          # 擁有權易主要看得見
        self.statusBar().showMessage(
            f'[FOV] 光錐啟用 — 參數來源：{src_name[source]}', 4000)
        self._on_fov_visual_source_changed()     # 立即用新來源重投影
        self._refresh_fov_cone_status()

    def _heatmap_radius_m(self) -> float:
        """SAR 熱力圖的掃描半徑 —— **永遠**吃戰術分析頁。

        ⚠ 這是唯一允許直接讀 `self.parameter_panel.get_fov_radius()` 的地方,
        AST 守衛 test_no_consumer_hardcodes_the_tactical_panel 對本函式開白名單。
        """
        return self.parameter_panel.get_fov_radius()

    def _fov_checkbox(self, source: str):
        """取得某一頁的光錐勾選框。

        ⚠⚠ 兩個面板的屬性名**不一樣**(VTOL `fov_cone_check`、戰術
        `_fov_cone_check`)。找不到時若只是 `return`,結果是兩個勾選框都留在
        「已勾選」而 `_fov_cone_source` 只認得其中一個 ⇒ 使用者看到兩個開關
        都亮著,光錐卻吃另一頁的數字,而且沒有任何錯誤。
        ⇒ 名稱寫在這裡集中管理,並由 test_both_panels_expose_a_fov_checkbox 鎖住。
        """
        panel = (getattr(self, 'vtol_recon_panel', None)
                 if source == FOV_SOURCE_VTOL else self.parameter_panel)
        if panel is None:
            return None
        for name in FOV_CHECKBOX_ATTRS:
            box = getattr(panel, name, None)
            if box is not None:
                return box
        return None

    def _uncheck_fov_source(self, source: str) -> None:
        """取消某一頁的光錐勾選框，且**不**再觸發 toggled 訊號。"""
        box = self._fov_checkbox(source)
        if box is None or not box.isChecked():
            return
        box.blockSignals(True)
        box.setChecked(False)
        box.blockSignals(False)

    def _refresh_fov_cone_status(self) -> None:
        """把「誰在控制光錐 / 為什麼沒畫」寫回 VTOL 面板的狀態標籤。"""
        panel = getattr(self, 'vtol_recon_panel', None)
        if panel is None or not hasattr(panel, 'set_fov_cone_status'):
            return
        if not self._fov_cone_enabled:
            panel.set_fov_cone_status('光錐未啟用', ok=True)
        elif self._fov_cone_source == FOV_SOURCE_VTOL:
            mount, hfov, vfov = panel.get_fov_visual_params()
            poses = self._last_uav_pose_for_fov
            n, airborne = fov_pose_summary(poses)
            # ⚠⚠ Cesium 端 main.html:2715 對 `!(alt > 0)` 直接刪掉該機的光錐
            #   （連 per-sysid 狀態一起 delete）。VTOL 停在起降場時 alt_rel≈0,
            #   正好是使用者第一次勾這個框的時刻 ⇒ 什麼都不會出現。
            #   這不是壞掉,但**必須講出來**,否則看起來就是功能沒做。
            if not n:
                tail = '，尚未收到 UAV 姿態（需連線中的 UAV）'
            elif not airborne:
                tail = (f'，{n} 架都在地面（相對高度 ≈ 0）'
                        f'—— 光錐要離地後才畫得出來')
            else:
                tail = f'，{airborne}/{n} 架已投影'
            clamp = fov_cone_clamp_warning(mount, hfov, vfov)
            panel.set_fov_cone_status(
                f'使用本頁參數：掛載 {mount:.1f}° / HFOV {hfov:.1f}°'
                f' / VFOV {vfov:.1f}°{tail}'
                + (f'　⚠ {clamp}' if clamp else ''),
                ok=bool(airborne) and not clamp)
        else:
            panel.set_fov_cone_status(
                '光錐由戰術分析頁控制，吃的是那一頁的參數', ok=False)

    def _on_fov_visual_source_changed(self) -> None:
        """來源或參數變更 → 用快取姿態立即重投影。"""
        src = resolve_fov_param_source(
            self._fov_cone_source, self.parameter_panel,
            getattr(self, 'vtol_recon_panel', None))
        mount, hfov, vfov = src.get_fov_visual_params()
        self._on_fov_visual_changed(mount, hfov, vfov)

    def _on_tactical_fov_visual_changed(self, mount_deg: float,
                                        hfov_deg: float, vfov_deg: float):
        """戰術頁的三個角度變更 —— 只有戰術頁擁有光錐時才重投影。

        ⚠ 少了這道守衛,VTOL 頁擁有光錐時去動戰術頁的滑桿會把光錐**改成
          戰術頁的角度**,而 VTOL 面板上顯示的仍是自己的值 ⇒ 顯示與實際分家。
        """
        if self._fov_cone_source != FOV_SOURCE_TACTICAL:
            return
        self._on_fov_visual_changed(mount_deg, hfov_deg, vfov_deg)

    def _on_vtol_fov_visual_changed(self, mount_deg: float,
                                     hfov_deg: float, vfov_deg: float):
        """VTOL 頁的三個角度變更 —— 只有本頁擁有光錐時才重投影。"""
        if self._fov_cone_source != FOV_SOURCE_VTOL:
            return
        self._on_fov_visual_changed(mount_deg, hfov_deg, vfov_deg)
        self._refresh_fov_cone_status()

    def _on_fov_visual_changed(self, mount_deg: float,
                                hfov_deg: float, vfov_deg: float):
        """掛載角 / HFOV / VFOV spinbox 變更 → 用快取的最新 UAV 姿態
        立即重新投影 3D 光錐（不需等下一筆遙測進來）。
        """
        if not self._fov_cone_enabled:
            return
        if not self._last_uav_pose_for_fov:
            # 尚未收到任何 UAV 姿態 → 留待下一筆遙測時自動帶入新參數
            return
        fov_r = resolve_fov_param_source(
            self._fov_cone_source, self.parameter_panel,
            getattr(self, 'vtol_recon_panel', None)).get_fov_radius()
        for sysid, pose in self._last_uav_pose_for_fov.items():
            lat, lon, alt, heading, pitch, roll = pose
            self.map_widget.update_fov_cone(
                lat, lon, alt, fov_r,
                heading, pitch, roll,
                sysid=sysid,
                hfov_deg=hfov_deg,
                vfov_deg=vfov_deg,
                mount_angle_deg=mount_deg,
            )

    def _on_radar_sim(self, params: dict):
        """模擬雷達威脅掃描：建立穹頂 + 播放脈衝動畫"""
        lat = params.get('lat', 23.70)
        lon = params.get('lon', 120.42)
        radius = params.get('radius', 5000.0)
        name = params.get('name', 'Radar')

        self.map_widget.add_radar_dome(lat, lon, 0.0, radius, name)
        # 飛向雷達位置
        self.map_widget.fly_to_position(lat, lon, 0.0, radius * 2.5)
        # 播放掃描脈衝動畫（需要延遲以等待穹頂建立完成）
        from PyQt6.QtCore import QTimer
        QTimer.singleShot(500, lambda: self.map_widget.animate_radar_scan(0, 3000))

        self.statusBar().showMessage(
            f'[Radar] 威脅穹頂已建立: {name} R={radius/1000:.1f}km', 5000
        )

    def _on_radar_clear(self):
        """清除所有雷達穹頂"""
        self.map_widget.clear_radar_domes()
        # 同時關閉 RCS 渲染
        self._rcs_enabled = False
        self.map_widget.clear_rcs_sensitivity(1)
        self.statusBar().showMessage('[Radar] 所有雷達穹頂已清除', 3000)

    def _on_rcs_toggle(self, enabled: bool):
        """RCS 渲染開關"""
        self._rcs_enabled = enabled
        if not enabled:
            self.map_widget.clear_rcs_sensitivity(1)
        self.statusBar().showMessage(
            f'[RCS] 敏感度渲染 {"啟用" if enabled else "關閉"}', 3000
        )

    def _update_tactical_on_uav_move(self, lat: float, lon: float, alt: float,
                                      heading_deg: float, pitch_deg: float,
                                      roll_deg: float, sysid: int = 1):
        """SITL UAV 位置更新時，同步更新戰術模組視覺化

        此方法應在 UAV 位置更新後呼叫，以驅動：
        1. FOV 光錐位置同步
        2. SAR 熱力圖掃描更新
        3. RCS 敏感度動態計算
        """
        if self._fov_cone_enabled:
            # ⚠ 參數一律走接縫取得 —— 寫死 self.parameter_panel 的話,
            #   VTOL 頁改參數會**完全沒有反應**而且沒有任何錯誤。
            _src = resolve_fov_param_source(
                self._fov_cone_source, self.parameter_panel,
                getattr(self, 'vtol_recon_panel', None))
            fov_r = _src.get_fov_radius()
            mount_deg, hfov_deg, vfov_deg = _src.get_fov_visual_params()
            # 快取最新姿態 → 拖動掛載角 / HFOV / VFOV 時可即時重投影
            self._last_uav_pose_for_fov[int(sysid)] = (
                lat, lon, alt, heading_deg, pitch_deg, roll_deg
            )
            # ★ 傳入 sysid → 每架 UAV 各自獨立的 FOV entity，
            #   不再互相覆寫造成「跳到誰那就顯示誰」閃爍。
            # HFOV/VFOV > 0 時自動切換為梯形角錐 frustum（含掛載角斜投影）
            self.map_widget.update_fov_cone(
                lat, lon, alt, fov_r,
                heading_deg, pitch_deg, roll_deg,
                sysid=sysid,
                hfov_deg=hfov_deg,
                vfov_deg=vfov_deg,
                mount_angle_deg=mount_deg,
            )
            # 同時更新熱力圖（光錐掃過的區域）
            # 傳 sysid → 熱力圖採用該機 frustum 真實梯形覆蓋判定
            # （HFOV/VFOV/掛載角拖滑桿改變時，掃描範圍即時跟著變）
            # ⚠⚠ 熱力圖半徑**不可以**跟著光錐來源跑。
            #   熱力圖是戰術頁的功能;光錐改成可切換來源之後,VTOL 頁一開光錐
            #   就會順手把熱力圖掃描半徑從 50 改成掃描寬度的一半(69.3),
            #   而使用者完全不知道自己動到了另一頁的東西。
            self.map_widget.update_heatmap(
                lat, lon, self._heatmap_radius_m(), sysid=int(sysid))

        if self._rcs_enabled:
            self.map_widget.update_rcs_sensitivity(
                lat, lon, alt, heading_deg, sysid,
            )

    # ═════════════════════════════════════════════════════════════════
    # Swarm Strike 處理 — 已抽出至 ui.controllers.StrikeControllerMixin
    # (MainWindow 透過多重繼承自動獲得所有 _on_strike_* 方法)
    # ═════════════════════════════════════════════════════════════════

    def _get_cesium_widget(self):
        """取得 CesiumMapWidget 實例（通過 DualMapWidget）

        ⚠ 這個函式**不保證 3D 圖台正在顯示** —— 它只回傳物件，2D 模式下也照樣
        回傳。要畫給使用者看的東西請改用 `_get_visible_cesium_widget()`。
        """
        if hasattr(self.map_widget, 'map_3d'):
            return self.map_widget.map_3d
        return None

    def _get_visible_cesium_widget(self, what: str):
        """取得 Cesium widget，並確保 3D 圖台真的切到前面來。

        ⚠ 為什麼需要這個：`_get_cesium_widget()` 回傳的是 `map_widget.map_3d`，
        **不管現在顯示的是 2D 還是 3D**。App 開機預設是 2D
        （dual_map_widget.py `self._mode = _MODE_2D`），所以覆蓋熱力圖／漏掃標示
        會畫進看不見的圖台，狀態列還照樣回報成功；而原本寫的
        `if cesium is None: 請切換到 3D 圖台` 永遠不會被執行到 —— 那是死碼。

        回傳 None 表示無法呈現（已經提示過使用者）。
        """
        cesium = self._get_cesium_widget()
        if cesium is None:
            QMessageBox.information(
                self, "需要 3D 圖台", f"目前沒有 3D Cesium 圖台，無法{what}。")
            return None
        ensure = getattr(self.map_widget, 'ensure_3d', None)
        if callable(ensure) and not ensure():
            QMessageBox.information(
                self, "需要 3D 圖台", f"請切換到 3D Cesium 圖台再{what}。")
            return None
        return cesium

    def on_reset_layout(self):
        """重置所有 Splitter 為預設比例 (Ctrl+Shift+R)

        涵蓋三層：
          1. 主水平 splitter (地圖 ⇄ 右側控制面板) → 60:40
          2. 右側垂直 splitter (參數面板 ⇄ 任務面板) → 78:22
          3. 各 tab 內的垂直 splitter (所有 GroupBox 平均分配)
        """
        try:
            # (1) 主 splitter
            if hasattr(self, '_main_splitter') and self._main_splitter:
                w = self.width() or 1280
                self._main_splitter.setSizes([int(w * 0.60), int(w * 0.40)])
            # (2) 右側面板 splitter
            if hasattr(self, '_right_panel_splitter') and self._right_panel_splitter:
                h = self.height() or 720
                self._right_panel_splitter.setSizes([int(h * 0.78), int(h * 0.22)])
            # (3) ParameterPanel 內的所有 tab splitter
            if hasattr(self, 'parameter_panel') and hasattr(
                self.parameter_panel, 'reset_resizable_layout'
            ):
                self.parameter_panel.reset_resizable_layout()
            self.statusBar().showMessage('已重置版面比例', 3000)
            logger.info('[Layout] 版面比例已重置')
        except Exception as e:
            logger.warning(f'[Layout] 重置失敗: {e}')

    def resizeEvent(self, event):
        """視窗大小變更時，保持地圖/控制面板 60:40 比例。

        這讓使用者在 1080p / 2K / 4K 之間拖拉視窗到不同螢幕時，
        splitter 會自動重新分配，不會跑版。
        """
        super().resizeEvent(event)
        splitter = getattr(self, '_main_splitter', None)
        if splitter is None:
            return
        try:
            w = event.size().width()
            if w < 200:
                return
            # 讀當前使用者是否手動改過比例；若變動 <5% 則視為自動，重新調整
            cur = splitter.sizes()
            if len(cur) == 2 and sum(cur) > 0:
                cur_ratio = cur[0] / sum(cur)
                if abs(cur_ratio - 0.60) > 0.05:
                    # 使用者自己拖過分隔線 → 尊重其設定，不覆寫
                    return
            splitter.setSizes([int(w * 0.60), int(w * 0.40)])
        except Exception:
            pass

    def closeEvent(self, event):
        """視窗關閉事件"""
        # 先停掉閉環終端同步打擊 tick + 視覺化 telemetry 訂閱，避免 timer/handler 在
        # widget 銷毀後仍觸發（存取已刪除的 statusBar / cesium）。
        try:
            if hasattr(self, 'abort_terminal_sync'):
                self.abort_terminal_sync()
        except Exception:
            pass
        try:
            if getattr(self, '_strike_viz_state', None) is not None:
                self._finalize_strike_visualization()
        except Exception:
            pass
        # ROS2 STOT 航次還在跑時，要由使用者決定「中止 / 留著繼續跑 / 不關」。
        # 靜默殺掉會製造不可歸因的失敗；靜默留著則會變成沒人監看的孤兒航次。
        try:
            if self._stot_close_decision() is False:
                event.ignore()
                return
        except Exception:
            pass
        # HV-DCCPP 規劃執行緒：不能殺、不能丟著不管，只能寄放（見 _hv_park_running_worker）。
        try:
            _hv_worker = getattr(self, '_hv_worker', None)
            if _hv_worker is not None:
                if _hv_worker.isRunning():
                    _hv_worker.stop()               # 放棄結果（規劃本身停不下來）
                    if not _hv_worker.wait(3000):
                        _hv_park_running_worker(self, _hv_worker)
                    else:
                        self._hv_disconnect_worker(_hv_worker)
                else:
                    self._hv_disconnect_worker(_hv_worker)
                self._hv_worker = None
        except Exception:
            pass
        # 確保 SITL 執行緒安全停止
        for _link in self._sitl_links:
            try:
                _link.stop()
                _link.wait(2000)
            except Exception:
                pass
        self._sitl_links.clear()
        # 停止本地 SITL 子行程
        if self._sitl_launcher is not None:
            try:
                self._sitl_launcher.stop()
            except Exception:
                pass

        if self.current_mission and self.has_unsaved_changes():
            reply = QMessageBox.question(
                self, "未儲存的變更",
                "當前任務有未儲存的變更，確定要退出嗎？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.No:
                event.ignore()
                return
        
        logger.info("應用程式關閉")
        event.accept()