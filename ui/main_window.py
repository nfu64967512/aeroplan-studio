"""
主視窗模組
整合地圖、參數面板、任務面板等核心 UI 組件
"""

import math
import os
import re
import time
from pathlib import Path
from typing import Optional, Tuple
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QSplitter, QStatusBar, QToolBar, QMessageBox,
    QFileDialog, QLabel, QScrollArea, QApplication
)
from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence, QShortcut

# 修正導入路徑
from config import get_settings
from config import features as _features   # 蜂群攻擊 UI／管線開關（2026-10-17 預設關）
from utils.logger import get_logger
from utils.file_io import write_waypoints, create_waypoint_line
from utils.math_utils import (  # 0-2 標準化：WGS84 單一來源
    EARTH_RADIUS_M, point_at_distance_bearing,
    spawn_positions as _spawn_positions,
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


def looks_like_real_board_connection(conn_str: str) -> bool:
    """這條連線字串背後看起來是不是**真實飛控**（序列埠），而不是 SITL / 網路連線。

    給 Write WPs 的確認對話框用來加重措辭、把預設鈕從「是」換成「取消」——
    純粹是**提示**用的啟發式，判斷錯了不影響安全（寫入前一律要求明確按
    「是」），但判斷對了能讓使用者在對著真板子按下去之前多想一秒。

    判準：Windows 的 COMx、Linux/macOS 的 /dev/tty*、或字面含 'serial'
    （pymavlink 的 'serial:' scheme）。udp*/tcp* 一律視為 SITL/網路連線。
    """
    s = (conn_str or '').strip().lower()
    if not s:
        return False
    if s.startswith('com') and len(s) > 3 and s[3].isdigit():
        return True
    if '/dev/tty' in s:
        return True
    if 'serial' in s:
        return True
    return False


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


def describe_avoidance_problem(error=None, remaining=None, min_sep_m=None):
    """把「避撞沒做成」翻成要給使用者看的文字;一切正常回傳 None。

    ⚠⚠ 為什麼是獨立函式:避撞出問題的兩種形狀 —— 丟例外、或跑完仍有衝突 ——
    過去都**只進日誌**。2026-10-08 實測 `CollisionAvoidance.__init__()` 少一個
    參數,每次規劃 TypeError,被 `except Exception` 吞成一行 WARNING,使用者
    拿到沒做過避撞的路徑還以為有。這裡只負責「說什麼」,「怎麼彈出來」在
    `MainWindow._notify_avoidance_problem`。

    Args:
        error: 避撞模組丟出的例外(None = 沒丟)
        remaining: `CollisionAvoidance.remaining_conflicts`(跑完仍存在的衝突)
        min_sep_m: 最小間距門檻,只用來說明
    """
    if error is not None:
        return (
            "避撞模組執行失敗,這份規劃**沒有**做避撞處理 —— 目前顯示的路徑"
            "可能有機與機之間距離不足的位置。\n\n"
            f"錯誤:{type(error).__name__}: {error}\n\n"
            "請不要直接用這份路徑飛行;詳情見日誌。")
    if not remaining:
        return None
    thr = f"(門檻 {min_sep_m:.0f} m)" if min_sep_m else ""
    lines = [f"避撞跑完後仍有 {len(remaining)} 個衝突沒有解掉{thr}:"]
    shown = sorted(remaining, key=lambda c: c.min_dist_m)[:5]
    for c in shown:
        lines.append(f"  • UAV-{c.uav_a} / UAV-{c.uav_b}:最近只有 {c.min_dist_m:.1f} m"
                     f"(t={c.time_s:.0f} s)")
    if len(remaining) > len(shown):
        lines.append(f"  …另外還有 {len(remaining) - len(shown)} 個")
    lines.append("")
    lines.append("常見原因:已達「抬升上限」,或衝突落在起降段(高度錯開動不了)。"
                 "可加大抬升上限/高度錯開量,或拉開起飛間距後重新規劃。")
    return "\n".join(lines)


def run_collision_avoidance(params: dict, speed: float, assembled_paths, *,
                            status, notify):
    """跑避撞,回傳 (第一輪偵測到的衝突, 累計動作)。

    ⚠⚠ 失敗不可以只進日誌:丟例外、或跑完仍有衝突,都要讓使用者**看見**。
    對話框用 `QTimer.singleShot(0, ...)` 延後到這一輪顯示做完才彈 —— 否則模態
    對話框會蓋在還沒畫出來的舊地圖上。

    Args:
        status: callable(text, ms) —— 顯示一般狀態列訊息
        notify: callable(text) —— 讓使用者看見的警告(狀態列 + 對話框)
    """
    conflicts, actions = [], []
    if not (bool(params.get('collision_avoidance', False))
            and assembled_paths and len(assembled_paths) >= 2):
        return conflicts, actions
    problem = None
    try:
        from core.dccpp.collision_avoidance import CollisionAvoidance
        ca = CollisionAvoidance(**build_avoidance_kwargs(params, speed))
        conflicts, actions = ca.run(assembled_paths)
        problem = describe_avoidance_problem(
            remaining=ca.remaining_conflicts, min_sep_m=ca.min_sep)
        if conflicts and problem is None:
            status(f"避撞偵測: {len(conflicts)} 個衝突, "
                   f"已執行 {len(actions)} 個避撞動作", 5000)
    except Exception as err:
        logger.error(f"避撞模組失敗: {err}", exc_info=True)
        problem = describe_avoidance_problem(error=err)
    if problem is not None:
        QTimer.singleShot(0, lambda t=problem: notify(t))
    return conflicts, actions


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


#: 生成點排列。⚠ 2026-09-09 搬到 ``utils.math_utils``（SITL 情境也要用，而那條路徑
#: 完全不碰 Qt）。這裡**再匯出**同一個物件，既有 `from ui.main_window import
#: spawn_positions` 的呼叫端與測試不用改。搬移前後逐位元相同（6 種機數 × 5 種間距實測）。
spawn_positions = _spawn_positions


#: SITL 生成點在共用起降場時要排開的間距（公尺）。VTOL 的
#: ``vtol_site_clear_radius_m`` 是 6–8 m，10 m 剛好把兩台的淨空圈分開。
SITL_COLOCATED_SPREAD_M: float = 10.0


def spread_colocated_homes(homes: list, spacing_m: float = SITL_COLOCATED_SPREAD_M,
                           tol_m: float = 1.0, logger_=None) -> list:
    """把**共點**的每架起飛點排成一列；本來就分開的原樣不動。

    ⚠⚠⚠ 為什麼需要：`per_uav_homes` 取的是每台計畫的第一個航點，而全隊共用一個
    起降場時那四個點是**同一個座標**（實測 HV 規劃四台都是
    ``(25.074968, 121.235507)``）。那份清單忠實反映計畫沒有錯，但直接餵給
    ``start_multi(homes=…)`` 會讓四台 SITL **生在同一個點上**、彼此重疊。

    排開的方向取**垂直於起飛航向**（與 :func:`spawn_positions` 同一個慣例），
    這樣是並排而不是一台擋在另一台的起飛路徑上。以起飛點為中心對稱展開。

    ⚠ 只在「所有點彼此都在 ``tol_m`` 內」時才動作。使用者真的規劃了多個起降場時
    （逐機回收站／多場坪）那些座標本來就不同，**不可以**把它們拉到一條線上。
    """
    if not homes or len(homes) < 2:
        return homes
    lat0, lon0 = float(homes[0][0]), float(homes[0][1])
    for h in homes[1:]:
        dN = (float(h[0]) - lat0) * 111320.0
        dE = (float(h[1]) - lon0) * 111320.0 * math.cos(math.radians(lat0))
        if math.hypot(dN, dE) > float(tol_m):
            return homes                                   # 真的分開 ⇒ 不動
    hdg0 = float(homes[0][2]) if len(homes[0]) > 2 else 0.0
    pts = spawn_positions(lat0, lon0, len(homes), hdg0, float(spacing_m))
    out = [(p[0], p[1], (float(h[2]) if len(h) > 2 else hdg0))
           for p, h in zip(pts, homes)]
    if logger_ is not None:
        logger_.info(
            f'[SITL] 每架起飛點共用同一個起降場 ⇒ 沿垂直於起飛航向 '
            f'{hdg0:.0f}° 排成一列，間距 {float(spacing_m):.1f} m（{len(out)} 台）。'
            f'不排開的話四台會生在同一個座標上互相重疊。'
        )
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

# HARVEST 兩個構型的顯示名稱（數值以 docs/平台參數表.md 為準，這裡只給人看）
HV_PLATFORM_NAMES = {'A': 'Pioneer PNP', 'B': 'HERO PNP'}

# ── 關視窗時還在跑的 HARVEST 規劃執行緒（見 _hv_park_running_worker）────────
#   放在模組層級是刻意的：這裡是唯一在視窗死後還握著它的地方。
_HV_ORPHAN_WORKERS: set = set()


def _hv_wait_for_orphan_workers(timeout_ms: int = 60000) -> None:
    """程式要結束了：等寄放中的規劃執行緒跑完。"""
    for w in list(_HV_ORPHAN_WORKERS):
        try:
            if w.isRunning():
                logger.info('[HARVEST] 等待背景規劃收尾…')
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
    logger.warning('[HARVEST] 視窗關閉時規劃仍在執行，已寄放該執行緒等它自行結束')


# ═════════════════════════════════════════════════════════════════════
#  高精地圖預計成果（規劃預測，非實際成果）：背景計算、過期判斷、重設
#  （模組層級 ⇒ 用假 self／裸實例也能跑，見 tests/test_hdmap_preview_gui.py）
# ═════════════════════════════════════════════════════════════════════
#: 地圖上預計成果圖層的 id（2D／3D 同一個）
HDMAP_PREVIEW_LAYER_ID = 'hdmap_plan'


def _win_attr(win, name: str, default=None):
    """讀視窗屬性：裸 PyQt 實例（測試的 ``cls.__new__``）getattr 會丟 RuntimeError ⇒ 退回 ``__dict__``。"""
    try:
        return getattr(win, name, default)
    except RuntimeError:
        return (getattr(win, '__dict__', None) or {}).get(name, default)


def _qt_object_alive(obj) -> bool:
    """Qt 物件的 C++ 端還在不在（``sip.isdeleted``）；不是 sip 包裝的物件（測試的替身）一律當作在。"""
    try:
        from PyQt6 import sip
        return not sip.isdeleted(obj)
    except TypeError:
        return True
    except Exception:          # noqa: BLE001
        return True


def _hdmap_preview_abandon(worker, *, closing: bool = False, retired: bool = False) -> None:
    """放棄一個預覽 worker：拆信號、放棄結果；還在跑就寄放到模組層級等它自己結束。

    ``retired=True``：結果已經收到（信號是排隊送來的，那一刻 run() 可能還沒 return）⇒ 同樣不能直接丟參考，
    只是不寫「結果已放棄」的 log。

    算圖沒有取消點；QThread 在還在跑的時候被解構（parent 一起刪，或 Python 這邊沒人參考被 GC）是 qFatal
    ⇒ 跟 HARVEST 規劃執行緒共用 ``_HV_ORPHAN_WORKERS``，程式結束前由 ``_hv_wait_for_orphan_workers`` 統一等。
    """
    try:
        worker.stop()
    except Exception:                                        # noqa: BLE001
        pass
    for sig in ('finished_ok', 'failed'):
        try:
            getattr(worker, sig).disconnect()
        except (TypeError, RuntimeError, AttributeError):
            pass                                             # 本來就沒接 / C++ 物件已消失
    try:
        running = bool(worker.isRunning())
    except (RuntimeError, AttributeError):
        running = False
    if running and closing:
        try:
            if worker.wait(3000):
                running = False
        except (RuntimeError, AttributeError):
            running = False
    if not running:
        try:
            worker.deleteLater()
        except (RuntimeError, AttributeError):
            pass
        return
    _HV_ORPHAN_WORKERS.add(worker)
    try:
        worker.finished.connect(lambda w=worker: _HV_ORPHAN_WORKERS.discard(w))
        if worker.isFinished():                              # 剛好在 connect 之前結束 ⇒ finished 已經發過了
            _HV_ORPHAN_WORKERS.discard(worker)
            return
    except Exception:                                        # noqa: BLE001
        pass
    try:
        from PyQt6.QtWidgets import QApplication
        from PyQt6.QtCore import Qt as _Qt
        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(_hv_wait_for_orphan_workers, _Qt.ConnectionType.UniqueConnection)
    except Exception:                                        # noqa: BLE001
        pass
    if not retired:
        logger.info('[高精地圖預覽] 計算還在跑，結果已放棄；執行緒寄放到它自己結束')


def hdmap_preview_stale_reason(win, stamp):
    """預覽是哪一份計畫算的（``stamp = (hv, hv.last_result)``）跟目前畫面上的還是不是同一份？是 ⇒ None；否則回原因。"""
    if not stamp:
        return '預覽已被重設（計畫已變更或清除）'
    s_hv, s_res = stamp
    hv = _win_attr(win, '_hv_coordinator')
    if hv is None or hv is not s_hv:
        return '規劃器物件已換（重新規劃或清除）'
    if getattr(hv, 'last_result', None) is not s_res:
        return '計畫結果已換（重新規劃）'
    if _win_attr(win, '_dccpp_result') is not s_res:
        return '目前畫面上的結果已不是這份建圖計畫'
    if getattr(hv, 'mapping_mode', None) is None:
        return '這份計畫不是建圖模式'
    return None


def hdmap_preview_summary(preview) -> str:
    """偵察頁狀態字／狀態列用的一行摘要（都標明是規劃預測）。"""
    s = getattr(preview, 'stats', None) or {}
    area = s.get('area_km2') or {}
    k = s.get('k_product')

    def _f(v, spec):
        try:
            f = float(v)
        except (TypeError, ValueError):
            return '—'
        return format(f, spec) if math.isfinite(f) else '—'
    lo, hi = (list(s.get('data_gb_range') or []) + [None, None])[:2]
    # 跟測區面積並列時要用「測區內」的可成圖面積（survey.preview 的 product 含足跡伸出測區外的部分，會比測區大）
    prod, where = area.get('product_in_survey'), '測區內'
    if prod is None:
        prod, where = area.get('product'), ''
    # 破洞數寫明依據（上傳閘門格／預覽格）：預覽格較細時兩邊可能不同，「會不會擋上傳」只看閘門那一格
    from survey.preview import hole_count_text
    return (f"預計成果（規劃預測，非實際成果）：{where}可成圖 {_f(prod, '.3f')} km²"
            f"（視數 ≥{_f(k, 'g')}）／測區 {_f(area.get('survey'), '.3f')} km²、"
            f"{hole_count_text(s)}、"
            f"GSD 中位 {_f((s.get('gsd_cm') or {}).get('median'), '.2f')} cm、"
            f"照片 {_f(s.get('photos_total'), ',.0f')} 張（約 {_f(lo, '.1f')}–{_f(hi, '.1f')} GB）")


#: 預計成果可以蓋掉的狀態列訊息（自己上一則、規劃後處理的「完成」）；其他（HARVEST 的 loud note、未覆蓋區域…）不蓋
HDMAP_STATUS_REPLACEABLE = ('預計成果', 'DCCPP 最佳化完成')


def hdmap_preview_status_message(win, text: str, timeout_ms: int) -> bool:
    """預計成果的狀態列訊息：**只在狀態列空著、或顯示的是可蓋掉的那幾種時才顯示**（回傳有沒有顯示）。

    規劃完成後約 1 秒預覽才算完；那時狀態列上可能是 HARVEST 的重點（loud note 15 s、未覆蓋區域常駐）
    ⇒ 不能被輔助功能的摘要蓋掉。摘要本身一定在偵察頁的狀態字裡。"""
    try:
        sb = win.statusBar()
    except Exception:                                        # noqa: BLE001 - 假 self／視窗已關
        return False
    cur = ''
    try:
        cur = str(sb.currentMessage() or '')
    except (AttributeError, RuntimeError, TypeError):
        cur = ''
    if cur and not cur.startswith(HDMAP_STATUS_REPLACEABLE):
        return False
    try:
        sb.showMessage(text, int(timeout_ms))
    except RuntimeError:
        return False
    return True


def hdmap_preview_show_status(win, summary: str = None) -> None:
    """偵察頁的預計成果狀態字：摘要＋（有的話）地圖圖層畫不出來的原因與重試方法。"""
    panel = _win_attr(win, 'vtol_recon_panel')
    if panel is None or not hasattr(panel, 'set_hdmap_preview_status'):
        return
    if summary is None:
        preview = _win_attr(win, '_hdmap_preview')
        if preview is None:
            return
        summary = hdmap_preview_summary(preview)
    err = _win_attr(win, '_hdmap_preview_draw_error')
    text = summary
    if err:
        text += (f'\n⚠ 預計成果已算好，但地圖圖層畫不出來：{err}'
                 '（取消再勾「地圖顯示預計成果」會重畫；對話框照樣可以開）')
    panel.set_hdmap_preview_status(text)


def hdmap_preview_unavailable_reason(win) -> str:
    """工具選單／偵察頁按「預覽高精地圖成果…」但沒有預計成果時，說明為什麼（依目前狀態）。"""
    if _win_attr(win, '_hdmap_preview_worker') is not None:
        return '預計成果還在背景計算，算好偵察頁的「預覽高精地圖成果…」就可以按。'
    in_flight = _win_attr(win, '_hv_planning_in_flight')
    try:
        planning = bool(in_flight()) if callable(in_flight) else False
    except Exception:                                        # noqa: BLE001
        planning = False
    if planning:
        return '規劃還在背景執行：完成後（建圖模式）會自動計算預計成果。'
    err = _win_attr(win, '_hdmap_preview_error')
    if err:
        return f'上一份建圖計畫的預計成果無法產生：{err}'
    return '目前沒有預計成果：請在 VTOL 偵察頁用「航測建圖」模式（HARVEST）規劃，規劃完成後會自動計算。'


#: HARVEST 的 loud note 裡，建圖狀態字已經另外寫出來的（相機／觸發、交叉航帶、上傳閘門）
_HARVEST_NOTES_SHOWN_ELSEWHERE = ('mapping_setup', 'mapping_upload_blocked', 'mapping_cross_')


def harvest_report_lines(report) -> list:
    """``_hv_last_report``（_on_hv_report 累積的未覆蓋區域與 loud note）→ 偵察頁建圖狀態字要附上的幾行。

    成功的規劃裡這些重點原本只在狀態列出現 15 秒（未覆蓋區域常駐），之後的狀態列訊息會把它們蓋掉
    ⇒ 寫進持久的狀態字（建圖狀態字已另外寫出的相機／交叉航帶／上傳閘門類不重複）。"""
    rep = report if isinstance(report, dict) else {}
    out = []
    unc = list(rep.get('uncovered') or [])
    if unc:
        out.append('⚠ HARVEST 有未覆蓋區域：' + '、'.join(str(u) for u in unc))
    for n in rep.get('loud_notes') or []:
        t = str(n)
        if not t.startswith(_HARVEST_NOTES_SHOWN_ELSEWHERE):
            out.append(f'HARVEST：{t}')
    return out


def harvest_plan_wind(hv):
    """規劃器**生效**的風（``hv.last_ctx.inputs.wind``，不是面板現在的值）→
    ``{'speed_mps', 'dir_from_deg', 'gust_mps', 'n_layers'}``；取不到規劃（沒有 last_ctx）⇒ None。

    規劃器沒有風層 ⇒ ``n_layers == 0``、風速 0、風向 None（無風規劃）。多層取風速最大那層
    （AeroPlan 轉接層只建一層 0–3000 m AGL、陣風 1.3 倍）。"""
    ctx = _win_attr(hv, 'last_ctx') if hv is not None else None
    if ctx is None:
        return None
    layers = tuple(getattr(getattr(ctx, 'inputs', None), 'wind', None) or ())
    if not layers:
        return {'speed_mps': 0.0, 'dir_from_deg': None, 'gust_mps': 0.0, 'n_layers': 0}
    lay = max(layers, key=lambda l: float(l.speed_mps))
    return {'speed_mps': float(lay.speed_mps), 'dir_from_deg': float(lay.dir_from_deg) % 360.0,
            'gust_mps': float(getattr(lay, 'gust_mps', 0.0) or 0.0), 'n_layers': len(layers)}


#: 規劃風低於這個值就當「無風規劃」（面板一位小數；CWA／網格寫入可能帶更多位）
_PLAN_WIND_CALM_MPS = 0.05


def harvest_wind_text(wind) -> str:
    """:func:`harvest_plan_wind` 的結果 → 「規劃風 X m/s 來自 Y°」（偵察頁結果摘要、log、SITL 對話框共用）。

    0 m/s 明寫「無風規劃」，並提醒風向仍影響進場方向（評估_2026-10-20 §5.2：0 m/s 來自 270° 與 0° 的計畫不同）。"""
    if wind is None:
        return '規劃風：取不到規劃器生效值'
    if not wind.get('n_layers'):
        return '規劃風：無（規劃器沒有風層 ⇒ 無風規劃）'
    s, d = float(wind['speed_mps']), round(float(wind['dir_from_deg'])) % 360
    layers = f'（{wind["n_layers"]} 層取最大）' if int(wind['n_layers']) > 1 else ''
    if s < _PLAN_WIND_CALM_MPS:
        return f'規劃風 0 m/s＝無風規劃{layers}（風向 {d}° 仍影響進場方向）'
    return f'規劃風 {round(s, 1):g} m/s 來自 {d}°{layers}'


def sitl_plan_wind_offer(win):
    """GUI 啟動 SITL 時「帶入規劃風」要帶的值 → ``{'speed_mps', 'dir_from_deg', 'source', 'text'}`` 或 None。

    有效值優先：目前顯示的計畫是 HARVEST 規劃的 ⇒ 用**規劃器生效的風**（面板之後改過也不算，飛的是那份計畫）；
    否則（沒有計畫／DCCPP 計畫，DCCPP 不考慮風）退回偵察頁風格子現在的值，來源寫明。都沒有 ⇒ None。"""
    hv, res = _win_attr(win, '_hv_coordinator'), _win_attr(win, '_dccpp_result')
    panel = _win_attr(win, 'vtol_recon_panel')
    p_spd = getattr(panel, 'hv_wind_speed_spin', None)
    p_dir = getattr(panel, 'hv_wind_dir_spin', None)
    panel_wind = None
    if p_spd is not None and p_dir is not None:
        try:
            panel_wind = (float(p_spd.value()), float(p_dir.value()) % 360.0)
        except (RuntimeError, TypeError, ValueError):
            panel_wind = None
    if hv is not None and res is not None and getattr(hv, 'last_result', None) is res:
        w = harvest_plan_wind(hv)
        if w is not None:
            spd = float(w['speed_mps'])
            d = 0.0 if w['dir_from_deg'] is None else float(w['dir_from_deg'])
            src = '目前的 HARVEST 計畫（規劃器生效值）'
            if panel_wind is not None and (abs(panel_wind[0] - spd) > 0.05
                                           or abs((panel_wind[1] - d + 180.0) % 360.0 - 180.0) > 0.5):
                src += (f'；⚠ 偵察頁風格子現在是 {round(panel_wind[0], 1):g} m/s 來自 {panel_wind[1]:.0f}°，'
                        '與計畫不同——帶的是計畫用的那個')
            return {'speed_mps': spd, 'dir_from_deg': d, 'source': src, 'text': harvest_wind_text(w)}
    if panel_wind is not None:
        why = 'DCCPP 計畫（不考慮風）' if res else '還沒有計畫'
        return {'speed_mps': panel_wind[0], 'dir_from_deg': panel_wind[1],
                'source': f'偵察頁風格子（{why}）',
                'text': harvest_wind_text({'speed_mps': panel_wind[0], 'dir_from_deg': panel_wind[1],
                                           'gust_mps': 0.0, 'n_layers': 1})}
    return None


# ─────────────────────────────────────────────────────────────────────
#  網格風重估（core.grid_wind_estimate）→ GUI（2026-10-21）
#    畫面上有 HARVEST 計畫、又有 Open-Meteo 網格／分層風時，用網格風重算那份計畫固定翼航段的航時與能量，
#    跟規劃器自己的估計並列。計畫、上傳的任務、防撞時間軸都不動（只顯示）。
#    docs/實作_2026-10-21_網格風重估航時能量.md §8
# ─────────────────────────────────────────────────────────────────────
#: 重估的問題併進 ``_hv_last_report`` loud note 時的前綴（每次重估先把舊的整批換掉）
GRID_WIND_NOTE_PREFIX = 'grid_wind_estimate('
#: 抓網格／分層風的請求點數（每軸）＝ WindGridWorker／WindProfileWorker 的預設 nx、ny（測試鎖住兩者相等）。
#:   範圍變大時點數**不變**（6×6＝36 點，在 core.wind_field.OPEN_METEO_MAX_POINTS 之內）⇒ 點距變大，狀態文字寫出點距
WIND_FETCH_GRID_N = 6


def _same_bbox(a, b, tol: float = 1e-9) -> bool:
    try:
        return len(a) == 4 and len(b) == 4 and all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))
    except (TypeError, ValueError):
        return False


def _bbox_size_km(bb) -> tuple:
    """bbox (lat_min, lon_min, lat_max, lon_max) → (東西寬 km, 南北高 km)。"""
    la0, lo0, la1, lo1 = (float(x) for x in bb)
    return ((lo1 - lo0) * 111.32 * math.cos(math.radians(0.5 * (la0 + la1))), (la1 - la0) * 111.32)


def harvest_display_plan(win):
    """畫面上那份 HARVEST 計畫 → ``(hv, plan, ctx)``（同一次規劃的 MissionPlan 與 MissionContext）。
    沒有計畫、是 DCCPP 計畫、或轉接器最後一次的結果不是畫面上那份 ⇒ None（同 harvest_wind_text 的判斷）。"""
    hv = _win_attr(win, '_hv_coordinator')
    res = _win_attr(win, '_dccpp_result')
    if hv is None or res is None or getattr(hv, 'last_result', None) is not res:
        return None
    plan, ctx = getattr(hv, 'last_plan', None), getattr(hv, 'last_ctx', None)
    if plan is None or ctx is None or not getattr(plan, 'plans', None):
        return None
    return hv, plan, ctx


def harvest_plan_extent_points(plan, ctx) -> tuple:
    """計畫全部航點＋各機實際用的起降場 → ``([(lat, lon), …], 航點數, [起降場 id])``。

    起降場取 UAVPlan.takeoff_site／landing_site 與 ctx.start_sites，位置用 ctx.sites_enu 經同一個 ctx.frame 轉 WGS84。
    航點本來就從坪起降；明列起降場是為了範圍說明照實寫，也不靠航點剛好落在坪上。"""
    import numpy as np
    pts, n_wp, ids = [], 0, set()
    starts = getattr(ctx, 'start_sites', None) or {}
    for uid, p in plan.plans.items():
        for w in p.waypoints:
            pts.append((float(w.lat_deg), float(w.lon_deg)))
            n_wp += 1
        for s in (getattr(p, 'takeoff_site', ''), getattr(p, 'landing_site', ''), starts.get(uid)):
            if s:
                ids.add(str(s))
    sites = getattr(ctx, 'sites_enu', None) or {}
    used = sorted(s for s in ids if s in sites)
    if used:
        enu = np.array([[float(sites[s][0][0]), float(sites[s][0][1]), float(sites[s][1])] for s in used])
        pts.extend((float(la), float(lo)) for la, lo, _h in ctx.frame.to_wgs84(enu))
    return [(la, lo) for la, lo in pts if math.isfinite(la) and math.isfinite(lo)], n_wp, used


def wind_fetch_scope(win, margin_m: float):
    """按「抓氣象網格」時要抓的範圍 → ``{'bbox', 'area_bbox', 'kind', 'text'}``；沒作業區也沒計畫 ⇒ None。

    * 畫面上有 HARVEST 計畫（kind 'plan'）：作業區＋計畫全部航點＋起降場，外擴 ``margin_m``。2026-10-21 實測只抓作業區外擴
      500 m 時，A1 有 2.5–3.3 km 的起降轉移不在網格內，重估只能沿用規劃風；
    * 沒有計畫（kind 'area'）：作業區外擴 ``margin_m``（原狀，``_wind_area_bbox``）。
    請求點數不變（WIND_FETCH_GRID_N²）⇒ 範圍變大點距跟著變大，``text`` 寫出點距。``area_bbox``（作業區外擴）另外留著：
    分層風的剖面柱與寫入規劃風仍以作業區中心為準（_wind_profile_centre）。"""
    from core.wind_field import area_bbox
    n = WIND_FETCH_GRID_N
    area_bb = win._wind_area_bbox()
    tgt = harvest_display_plan(win)
    why = '沒有 HARVEST 計畫；規劃完成後再抓會改成涵蓋整條航線與起降場'
    if tgt is not None:
        try:
            pts, n_wp, sites = harvest_plan_extent_points(tgt[1], tgt[2])
        except Exception as e:                               # noqa: BLE001 - 退回作業區範圍，並寫明
            logger.error(f'[氣象網格] 取不到 HARVEST 計畫的航點：{type(e).__name__}: {e}', exc_info=True)
            pts, n_wp, sites = [], 0, []
            why = f'⚠ 取不到 HARVEST 計畫的航點（{type(e).__name__}: {e}），只涵蓋作業區'
        if pts:
            bb = area_bbox([pts] + list(win.survey_area_polygons() or []), margin_m=margin_m)
            w_km, h_km = _bbox_size_km(bb)
            text = (f"網格範圍：作業區＋目前 HARVEST 計畫的全部 {n_wp} 個航點與起降場（{'、'.join(sites) or '—'}），"
                    f"外擴 {margin_m:.0f} m；約 {w_km:.1f} × {h_km:.1f} km，{n}×{n} 點"
                    f"（點距約 {w_km / (n - 1):.2f} × {h_km / (n - 1):.2f} km）")
            return {'bbox': tuple(float(x) for x in bb), 'area_bbox': area_bb, 'kind': 'plan', 'text': text}
        if why.startswith('沒有'):
            why = '⚠ HARVEST 計畫裡沒有航點，只涵蓋作業區'
    if area_bb is None:
        return None
    w_km, h_km = _bbox_size_km(area_bb)
    return {'bbox': tuple(float(x) for x in area_bb), 'area_bbox': area_bb, 'kind': 'area',
            'text': f"網格範圍：作業區外擴 {margin_m:.0f} m（{why}）；約 {w_km:.1f} × {h_km:.1f} km，{n}×{n} 點"}


def _take_fetch_scope(st: dict, extent):
    """抓取結果到了：取走那次請求記下的範圍（``fetch_scope``）；範圍對不上（不是那次抓的）⇒ None。"""
    sc = st.pop('fetch_scope', None)
    return sc if isinstance(sc, dict) and _same_bbox(sc.get('bbox'), extent) else None


def _grid_wind_source(st: dict):
    """目前的網格風來源（分層風與單層網格互相取代，只會有一個；分層風優先）。"""
    return st.get('profile') if st.get('profile') is not None else st.get('grid')


def grid_wind_estimate_lines(est, *, src_notes=(), scope_kind=None) -> list:
    """PlanWindEstimate → 風場狀態列那幾行：summary_lines()（來源、規劃器用的風、總任務時間、每機航時／餘裕／
    不在網格內的長度、⚠ 問題）＋ 網格外合計與怎麼補 ＋ 來源本身沒寫的說明（例如單層網格套用到所有高度）。"""
    lines = list(est.summary_lines())
    unc = sum(float(e.uncovered_m) for e in est.uavs.values())
    if unc > 0:
        lines.append(f"⚠ 共 {unc / 1000:.1f} km 航段不在網格範圍內，那幾段沿用規劃風（重估差 0）："
                     + ("網格在那幾段缺值，或計畫在抓取後換過" if scope_kind == 'plan'
                        else "重抓氣象網格會改用涵蓋整條航線與起降場的範圍"))
    lines.extend(f"說明：{n}" for n in est.notes if n not in tuple(src_notes))
    return lines


def _grid_wind_estimate_fresh(win, st: dict):
    """記下的重估還對應畫面上的計畫與目前的網格風 ⇒ True；計畫換過（estimate_matches）、改成 DCCPP、清除、網格換了 ⇒ False。"""
    from core.grid_wind_estimate import estimate_matches
    for_ = st.get('estimate_for')
    tgt = harvest_display_plan(win)
    if for_ is None or tgt is None or tgt[0] is not for_[0] or tgt[1] is not for_[1]:
        return False
    if _grid_wind_source(st) is not for_[2]:
        return False
    est = st.get('estimate')
    return est is None or estimate_matches(est, tgt[1])


def grid_wind_estimate_status_text(win) -> str:
    """風場狀態列的重估那幾行；算的那份計畫或網格風已經不是現在的 ⇒ 改寫成「已作廢」（不留舊計畫的數字）。"""
    get_state = _win_attr(win, '_wind_state')
    if get_state is None:
        return ''
    try:                                                     # 每次刷新風場狀態列都會走到（含即時風的 slot）：不能丟例外
        st = get_state()
        text = st.get('estimate_text') or ''
        if not text or _grid_wind_estimate_fresh(win, st):
            return text
    except Exception as e:                                   # noqa: BLE001
        logger.error(f'[網格風重估] 判斷是否過時失敗：{type(e).__name__}: {e}', exc_info=True)
        return f'⚠ 網格風重估：無法判斷是否還對應目前的計畫（{type(e).__name__}: {e}），請重抓氣象網格'
    return '⚠ 網格風重估已作廢：畫面上的計畫或網格風換過了（規劃完成、重抓或清除氣象網格時會重算）'


def grid_wind_estimate_digest(win) -> str:
    """規劃摘要那一行用的短句；沒有（或已作廢的）重估 ⇒ ''。"""
    get_state = _win_attr(win, '_wind_state')
    if get_state is None:
        return ''
    try:
        st = get_state()
        if not (st.get('estimate_text') and _grid_wind_estimate_fresh(win, st)):
            return ''
    except Exception as e:                                   # noqa: BLE001 - 摘要壞掉不能連累規劃結果的顯示
        logger.error(f'[網格風重估] 摘要讀不到：{type(e).__name__}: {e}', exc_info=True)
        return f'⚠ 網格風重估讀不到（{type(e).__name__}）'
    est = st.get('estimate')
    if est is None:
        return '⚠ 網格風重估失敗（見風場狀態）'
    d = est.makespan_grid_s - est.makespan_plan_s
    pct = 100.0 * d / est.makespan_plan_s if est.makespan_plan_s > 0 else float('nan')
    s = f"{'分層風' if est.source_kind == 'profile' else '氣象網格'}重估 makespan {d:+.1f} s（{pct:+.2f}%）"
    if est.problems:
        s += f"，⚠ {len(est.problems)} 項問題（見風場狀態）"
    unc = sum(float(e.uncovered_m) for e in est.uavs.values())
    if unc > 0:
        s += f"，{unc / 1000:.1f} km 不在網格內"
    return s


def _grid_wind_loud(win, problems: list, *, announce: bool) -> None:
    """重估的問題 → ``_hv_last_report`` 的 loud note（整批換掉；背景規劃進行中不碰，那份報告屬於新的一趟）
    ＋ 主視窗狀態列 15 s（``announce``：這次是新算的）。"""
    in_flight = _win_attr(win, '_hv_planning_in_flight')
    try:
        planning = bool(in_flight()) if callable(in_flight) else False
    except Exception:                                        # noqa: BLE001
        planning = False
    rep = _win_attr(win, '_hv_last_report')
    if not planning and (problems or isinstance(rep, dict)):
        rep = dict(rep) if isinstance(rep, dict) else {}
        old = list(rep.get('loud_notes') or [])
        new = [n for n in old if not str(n).startswith(GRID_WIND_NOTE_PREFIX)]
        new.extend(f'{GRID_WIND_NOTE_PREFIX}{p})' for p in problems)
        if new != old:
            rep['loud_notes'] = new
            win._hv_last_report = rep
    if problems and announce:
        more = f'（共 {len(problems)} 項，見偵察頁風場狀態）' if len(problems) > 2 else ''
        win.statusBar().showMessage('⚠ 網格風重估：' + '；'.join(problems[:2]) + more, 15000)


def refresh_grid_wind_estimate(win, reason: str = ''):
    """（重）算網格風重估並更新風場狀態列；回傳 PlanWindEstimate 或 None。從不丟例外（會在 Qt slot 裡被呼叫）。

    觸發：HARVEST 規劃完成（同步／背景兩條路）、網格或分層風抵達、清除氣象網格、清除路徑、換載具。
    * 沒有網格／分層風，或畫面上不是 HARVEST 計畫 ⇒ 清掉（不留舊計畫的數字）；
    * 同一份計畫（estimate_matches）＋同一份網格 ⇒ 沿用；計畫或網格換過 ⇒ 重算；
    * 重估失敗 ⇒ 狀態列寫明「失敗」（大聲），不當成「沒有差」；
    * problems（地速不可行、餘裕變負、超 σ_t、重建不可信）⇒ log warning ＋ _grid_wind_loud（橫幅）。"""
    try:
        from core.grid_wind_estimate import estimate_plan
        get_state = _win_attr(win, '_wind_state')
        if get_state is None:
            return None
        st = get_state()
        old_text = st.get('estimate_text') or ''
        src = _grid_wind_source(st)
        tgt = harvest_display_plan(win)
        est, text, loud, fresh = None, '', [], False
        if src is not None and tgt is not None:
            hv, plan, ctx = tgt
            if st.get('estimate') is not None and _grid_wind_estimate_fresh(win, st):
                est, text = st['estimate'], old_text
                loud = list(est.problems)                    # 沿用：橫幅的 loud note 照舊（不重新跳狀態列）
            else:
                fresh = True
                try:
                    est = estimate_plan(plan, ctx, src)
                except Exception as e:                       # noqa: BLE001 - 重估壞掉不能吃掉規劃結果，也不能當成沒差
                    logger.error(f'[網格風重估] 失敗（{reason}）：{type(e).__name__}: {e}', exc_info=True)
                    text = (f'⚠ 網格風重估失敗（{type(e).__name__}: {e}）：這份計畫的航時與能量只有規劃器'
                            '（規劃風）的估計')
                    loud = [f'重估失敗（{type(e).__name__}: {e}）']
                else:
                    scope = st.get('src_scope') if isinstance(st.get('src_scope'), dict) else {}
                    text = '\n'.join(grid_wind_estimate_lines(est, src_notes=tuple(getattr(src, 'notes', ()) or ()),
                                                              scope_kind=scope.get('kind')))
                    loud = list(est.problems)
                    logger.info(f'[網格風重估] {reason}：' + ' ｜ '.join(est.summary_lines()))
                    for p in loud:
                        logger.warning(f'[網格風重估] {p}')
            st['estimate_for'] = (hv, plan, src)
        else:
            st['estimate_for'] = None
        st['estimate'] = est
        st['estimate_text'] = text
        if fresh or old_text:
            _grid_wind_loud(win, loud, announce=fresh)
        if text != old_text:
            refresh = _win_attr(win, '_refresh_wind_status')
            if refresh is not None:
                refresh()
        return est
    except Exception as e:                                   # noqa: BLE001 - slot 裡不能帶例外回 Qt（會 qFatal）
        logger.error(f'[網格風重估] 更新顯示失敗（{reason}）：{type(e).__name__}: {e}', exc_info=True)
        return None


def hdmap_preview_reset(win, why: str = '') -> None:
    """規劃開始／失敗／取消、清除全部、清除路徑、換載具：丟掉預計成果（地圖層、結果、面板狀態），
    放棄還在算的那一趟；開著的預覽對話框標成「不再對應目前的計畫」（不關掉，使用者可能正在看）。"""
    w = _win_attr(win, '_hdmap_preview_worker')
    if w is not None:
        _hdmap_preview_abandon(w)
    had = _win_attr(win, '_hdmap_preview') is not None or w is not None
    drawn = bool(_win_attr(win, '_hdmap_preview_drawn', False))
    try:
        win._hdmap_preview_worker = None
        win._hdmap_preview = None
        win._hdmap_preview_payload = None
        win._hdmap_preview_drawn = False
        win._hdmap_preview_draw_error = None
        win._hdmap_preview_error = None
    except Exception:                                        # noqa: BLE001 - 假 self 不收屬性也不該擋清除
        pass
    if drawn:
        mwid = _win_attr(win, 'map_widget')
        if mwid is not None and hasattr(mwid, 'clear_hdmap_preview'):
            try:
                mwid.clear_hdmap_preview(HDMAP_PREVIEW_LAYER_ID)
            except Exception as e:                           # noqa: BLE001
                logger.warning(f'[高精地圖預覽] 清除地圖圖層失敗：{e}')
    panel = _win_attr(win, 'vtol_recon_panel')
    if panel is not None and hasattr(panel, 'set_hdmap_preview_state'):
        try:
            panel.set_hdmap_preview_state('none')
        except Exception as e:                               # noqa: BLE001
            logger.warning(f'[高精地圖預覽] 面板狀態重設失敗：{e}')
    dlg = _win_attr(win, '_hdmap_preview_dialog')
    if dlg is not None and hasattr(dlg, 'mark_stale'):
        try:
            dlg.mark_stale(f'計畫已變更（{why or "重設"}）：這份預覽不再對應目前的計畫')
        except RuntimeError:
            pass                                             # 對話框已經被 Qt 刪掉
    if had or drawn:
        logger.info(f'[高精地圖預覽] 已清除（{why or "重設"}）')


# ═════════════════════════════════════════════════════════════════════
#  航測建圖（HARVEST 建圖模式）：上傳前參數讀回閘門、相機回授、tlog
#  （模組層級純函式 ⇒ 值層級測試不必建 MainWindow）
# ═════════════════════════════════════════════════════════════════════
#: 讀回閘門逾時（每輪秒數）與補送輪數：最壞 3 × 3 = 9 s／架（read_params 只在工作執行緒呼叫）
MAPPING_READBACK_TIMEOUT_S = 3.0
MAPPING_READBACK_RETRIES = 2
#: 這些參數在開機時建立相機後端／回授中斷／relay／輸出功能 ⇒ 改了要**重開飛控**才生效：
#:   CAM1_TYPE（後端在 AP_Camera::init 建立）；CAM1_FEEDBAK_PIN（AP_Camera_Params.cpp 4.4.4–master 標
#:   @RebootRequired）；CAM1_FEEDBAK_POL（AP_Camera_Backend::setup_feedback_callback 只在開機時依極性掛一次
#:   RISING／FALLING 中斷，沒標 @RebootRequired 但改了不重開中斷邊緣不變）；CAM1_COMPID（master @RebootRequired，
#:   匯出器目前不輸出，先列著）；RELAY*（AP_Relay 開機設定腳位）；SERVOn_FUNCTION（輸出功能開機配置）。
_MAPPING_REBOOT_RE = re.compile(r'^(CAM1_TYPE|CAM1_FEEDBAK_PIN|CAM1_FEEDBAK_POL|CAM1_COMPID|RELAY.*|SERVO\d+_FUNCTION)$')
#: SERVOn_FUNCTION 讀回閘門：只有目前是這些值才允許自動寫（0 = 停用；10 = 相機觸發；-1 = GPIO）。
#:   其他值 ＝ 這個輸出正在用（馬達、舵面…），寫了下次開機就少一個輸出 ⇒ 拒絕並說出會失去什麼（R1 #5）。
_SERVO_FN_RE = re.compile(r'^SERVO\d+_FUNCTION$')
_SERVO_FN_REPLACEABLE = (0.0, 10.0, -1.0)
#: SERVOn_FUNCTION 常見值（SRV_Channel.h）；表外的就顯示數字
_SERVO_FUNCTION_LABELS = {
    -1: 'GPIO', 0: '停用（Disabled）', 1: 'RC 直通（RCPassThru）', 4: '副翼（Aileron）',
    10: '相機觸發（CameraTrigger）', 19: '升降舵（Elevator）', 21: '方向舵（Rudder）', 24: '襟翼（Flap）',
    25: '自動襟翼（FlapAuto）', 26: '空氣煞車（Airbrake）', 70: '油門（Throttle）', 73: '左油門（ThrottleLeft）',
    74: '右油門（ThrottleRight）', 75: '左傾轉馬達（TiltMotorLeft）', 76: '右傾轉馬達（TiltMotorRight）',
    77: '左升降副翼（ElevonLeft）', 78: '右升降副翼（ElevonRight）', 79: 'V 尾左（VTailLeft）',
    80: 'V 尾右（VTailRight）', 81: '加力油門（BoostThrottle）',
    **{33 + i: f'馬達 {i + 1}（Motor{i + 1}）' for i in range(8)},
    **{82 + i: f'馬達 {i + 9}（Motor{i + 9}）' for i in range(4)},
    **{51 + i: f'RC 輸入 {i + 1}（RCIN{i + 1}）' for i in range(16)},
}
#: LOG_BITMASK 的相機位元（ArduPlane Parameters.cpp 4.5.7:627：bit 12 = Camera）；沒設 ⇒ 飛完沒有 CAM 紀錄
LOG_BITMASK_CAMERA_BIT = 12
#: 讀回閘門每架一定會讀的額外參數（空速包絡＋日誌）
_MAPPING_EXTRA_READ = ('AIRSPEED_MIN', 'AIRSPEED_MAX', 'LOG_BITMASK', 'LOG_DISARMED')
#: MAV_PARAM_TYPE → SITLLink.set_param 的型別名（ArduPilot 其實不看 param_type，用的是自己存的型別）
_PTYPE_NAMES = {1: 'INT8', 2: 'INT8', 3: 'INT16', 4: 'INT16', 5: 'INT32', 6: 'INT32', 9: 'REAL32'}
#: 專案根目錄（tlog 存 runs/survey/<日期>/）
_PROJECT_ROOT = Path(__file__).resolve().parent.parent


def mapping_airspeed_commands(items) -> list:
    """任務裡每個「空速型」DO_CHANGE_SPEED（178，param1=0）的 param2（m/s）。

    -2（回預設空速）與 ≤ 0 不算；地速型（param1=1）不受 AIRSPEED_MIN/MAX 限制，也不算。
    ArduPlane do_change_speed：空速不在 [AIRSPEED_MIN, AIRSPEED_MAX] 內 ⇒ **靜默忽略**（4.4.4–master 相同）。
    """
    out = []
    for it in items:
        if int(it.get('command', 0)) != 178 or float(it.get('param1', 0.0)) != 0.0:
            continue
        v = float(it.get('param2', 0.0))
        if v > 0.0:
            out.append(v)
    return out


def servo_function_label(v) -> str:
    """SERVOn_FUNCTION 值 → 中文名稱（表外的顯示數字）。"""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    if math.isfinite(f) and f == int(f) and int(f) in _SERVO_FUNCTION_LABELS:
        return f'{int(f)} = {_SERVO_FUNCTION_LABELS[int(f)]}'
    return f'功能 {f:g}'


def _finite(v):
    """讀回值 → float；None／NaN／非數字 ⇒ None。"""
    if v is None or isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def mapping_readback_rows(expected: dict, got: dict, types: dict, speeds, *,
                          airspeed_expected: dict | None = None) -> list:
    """讀回值逐項比對 → ``[{'name','kind','expected','actual','ok','writable','write_value','ptype', ...}, ...]``。

    * 相機參數（kind ``value``）：``mission.sitl_link.values_match``（float32 格點比對；讀不到 = 不符），
      可寫（寫成預期值）。**例外** SERVOn_FUNCTION（R1 #5）：只有目前是 0（停用）或已經是相機／GPIO 功能
      （10／-1）才可寫；其他值（馬達、舵面…）⇒ ``writable=False`` 並在 ``refuse`` 說出會失去哪個輸出。
    * 空速（kind ``airspeed``，**永遠不自動寫**：飛行包絡）：讀回的 AIRSPEED_MIN／MAX 要等於匯出 .param 的值
      （``airspeed_expected``；R2：ArduPlane 存成 AP_Int16，26.6 會靜默變 26），而且任務每個空速型
      DO_CHANGE_SPEED 都要落在 [MIN, MAX]（含邊界，超出 ArduPlane 會靜默忽略該指令）。``detail`` 說哪一條不符。
    * LOG_BITMASK（kind ``bit``，R2）：要含 bit 12（Camera），否則飛完沒有 CAM 紀錄。這是位元測試、不是值比對；
      可寫但**只用 OR 補上這一個位元**（``write_value`` = 讀回值 | 4096，其他位元不動）；讀不到 ⇒ 不可寫。
    * LOG_DISARMED（kind ``info``，不擋）：只顯示；0 ⇒ 未解鎖時不記錄，地面測試拍照不會進日誌。
    """
    from mission.sitl_link import values_match
    rows = []
    for name, exp in expected.items():
        act = got.get(name)
        ok = bool(values_match(exp, act, types.get(name)))
        row = {'name': name, 'kind': 'value', 'expected': float(exp), 'actual': act, 'ok': ok,
               'writable': True, 'write_value': float(exp), 'ptype': types.get(name)}
        if not ok and _SERVO_FN_RE.match(name):
            a = _finite(act)
            if a is None:
                row['writable'] = False
                row['refuse'] = '讀不到目前的輸出功能，無法確認這個輸出沒有在用 ⇒ 不自動寫，請自己檢查'
            elif a not in _SERVO_FN_REPLACEABLE:
                row['writable'] = False
                row['refuse'] = (f'這個輸出目前是 {servo_function_label(a)}；寫成 {servo_function_label(exp)} '
                                 f'會在下次開機拿掉這個輸出 ⇒ 不自動寫。請改用沒在用的輸出通道（重新設定 relay／回授通道）'
                                 f'或自己確認後手動改')
        rows.append(row)
    speeds = [float(s) for s in (speeds or [])]
    aexp = dict(airspeed_expected or {})
    for name in ('AIRSPEED_MIN', 'AIRSPEED_MAX'):
        need, fails = [], []
        act = got.get(name)
        a = _finite(act)
        ev = aexp.get(name)
        if ev is not None:
            need.append(f'= {float(ev):g}（匯出 .param）')
            if not values_match(ev, act, types.get(name)):
                fails.append(f'與匯出的 .param {float(ev):g} 不同（飛控實際用 {_fmt_param(act)}）')
        if speeds:
            lo, hi = min(speeds), max(speeds)
            if name == 'AIRSPEED_MIN':
                need.append(f'≤ {lo:g}（任務最低空速）')
                if a is None or not a <= lo:
                    fails.append(f'任務空速 {lo:g} m/s 低於 AIRSPEED_MIN ⇒ 該 DO_CHANGE_SPEED 會被飛控靜默忽略')
            else:
                need.append(f'≥ {hi:g}（任務最高空速）')
                if a is None or not a >= hi:
                    fails.append(f'任務空速 {hi:g} m/s 高於 AIRSPEED_MAX ⇒ 該 DO_CHANGE_SPEED 會被飛控靜默忽略')
        if not need:
            continue
        ok = a is not None and not fails
        rows.append({'name': name, 'kind': 'airspeed', 'expected': ' 且 '.join(need), 'actual': act, 'ok': bool(ok),
                     'writable': False, 'ptype': types.get(name),
                     'detail': '；'.join(fails) if fails else ('讀不到' if a is None else '')})
    # LOG_BITMASK：位元測試（不是 values_match）
    bit = 1 << LOG_BITMASK_CAMERA_BIT
    act = got.get('LOG_BITMASK')
    a = _finite(act)
    row = {'name': 'LOG_BITMASK', 'kind': 'bit', 'expected': f'含 bit {LOG_BITMASK_CAMERA_BIT}（Camera）',
           'actual': act, 'ptype': types.get('LOG_BITMASK')}
    if a is None:
        row.update(ok=False, writable=False,
                   refuse='讀不到 LOG_BITMASK，無法只補 bit 12（不能把未知的其他位元蓋掉）⇒ 不自動寫')
    else:
        iv = int(round(a))
        has = bool(iv & bit)
        row.update(ok=has, writable=not has, write_value=float(iv | bit))
        if not has:
            row['write_note'] = (f'寫入時只補這一個位元：{iv} OR {bit} = {iv | bit}（其他日誌位元不變）；'
                                 f'沒有它飛完沒有 CAM 紀錄，後處理會失敗')
    rows.append(row)
    # LOG_DISARMED：資訊（不擋）
    act = got.get('LOG_DISARMED')
    a = _finite(act)
    if a is None:
        note = '讀不到 LOG_DISARMED'
    elif int(round(a)) == 0:
        note = 'LOG_DISARMED=0：未解鎖時不記錄日誌 ⇒ 地面測試拍照不會進日誌（飛行中正常）'
    else:
        note = f'LOG_DISARMED={a:g}：未解鎖時也記錄（地面測試拍照會進日誌）'
    rows.append({'name': 'LOG_DISARMED', 'kind': 'info', 'expected': '（資訊，不擋）', 'actual': act, 'ok': True,
                 'writable': False, 'ptype': types.get('LOG_DISARMED'), 'info': note})
    return rows


def run_mapping_readback(jobs, *, timeout_s: float = MAPPING_READBACK_TIMEOUT_S,
                         retries: int = MAPPING_READBACK_RETRIES) -> dict:
    """**工作執行緒**裡跑：每架 ``link.read_params``（PARAM_REQUEST_READ，不用 MAVFTP）→ 比對。

    jobs：``[{'sysid', 'link', 'expected': {name: 值}, 'speeds': [m/s, ...],
    'airspeed_expected': {'AIRSPEED_MIN': 值, 'AIRSPEED_MAX': 值}（可省）}, ...]``
    每架讀：預期的相機參數 + AIRSPEED_MIN／MAX + LOG_BITMASK + LOG_DISARMED。
    回傳 ``{sysid: {'rows': [...], 'error': None | str}}``；讀取本身丟例外 ⇒ ``error``（視為不符）。
    """
    out = {}
    for job in jobs:
        sysid = int(job['sysid'])
        names = list(dict.fromkeys(list(job['expected']) + list(_MAPPING_EXTRA_READ)))
        types = {}
        try:
            got = job['link'].read_params(names, timeout_s=timeout_s, retries=retries, types_out=types)
        except Exception as e:      # noqa: BLE001 — 讀不到就是不符，要說出原因
            out[sysid] = {'rows': [], 'error': f'{type(e).__name__}: {e}'}
            continue
        out[sysid] = {'rows': mapping_readback_rows(job['expected'], got, types, job.get('speeds'),
                                                    airspeed_expected=job.get('airspeed_expected')),
                      'error': None}
    return out


class MappingReadbackWorker(QThread):
    """上傳前讀回閘門的工作執行緒（read_params 是同步等待，不可在 GUI 執行緒跑）。"""
    done = pyqtSignal(dict)

    def __init__(self, jobs, parent=None):
        super().__init__(parent)
        self._jobs = list(jobs)

    def run(self):
        try:
            res = run_mapping_readback(self._jobs)
        except Exception as e:      # noqa: BLE001 — 執行緒裡的例外不能吞，交回 GUI 顯示
            res = {int(j['sysid']): {'rows': [], 'error': f'{type(e).__name__}: {e}'} for j in self._jobs}
        self.done.emit(res)


def _fmt_param(v) -> str:
    if v is None:
        return '讀不到'
    if isinstance(v, str):
        return v
    return f'{float(v):g}'


def mapping_reboot_names(bad: dict) -> dict:
    """{sysid: [不符、會寫、而且寫了要重開飛控的參數名]}（CAM1_TYPE／CAM1_FEEDBAK_PIN／CAM1_FEEDBAK_POL／
    CAM1_COMPID／RELAY*／SERVOn_FUNCTION；見 ``_MAPPING_REBOOT_RE``）。"""
    out = {}
    for sysid, r in bad.items():
        names = [row['name'] for row in r.get('rows', [])
                 if not row['ok'] and row['writable'] and _MAPPING_REBOOT_RE.match(row['name'])]
        if names:
            out[sysid] = names
    return out


def mapping_mismatch_text(bad: dict) -> str:
    """讀回不符清單（每架：名稱／預期／實際）。相符的不列。"""
    lines = []
    for sysid in sorted(bad):
        r = bad[sysid]
        lines.append(f'UAV-{sysid}：')
        if r.get('error'):
            lines.append(f'  讀回失敗：{r["error"]}')
        for row in r.get('rows', []):
            if row['ok']:
                continue
            if row['writable']:
                tail = f'（{row["write_note"]}）' if row.get('write_note') else ''
            elif row.get('refuse'):
                tail = f'（{row["refuse"]}）'
            else:
                tail = '（飛行包絡參數，不會自動寫入，請自行調整）'
            detail = f'：{row["detail"]}' if row.get('detail') else ''
            lines.append(f'  {row["name"]}：預期 {_fmt_param(row["expected"])}，'
                         f'實際 {_fmt_param(row["actual"])}{detail}{tail}')
    reboot = mapping_reboot_names(bad)
    if reboot:
        names = sorted({n for v in reboot.values() for n in v})
        lines.append('')
        lines.append(f'注意：{"、".join(names)} 寫入後要重開飛控才生效（相機後端、回授中斷、relay 與輸出功能在開機時建立）；'
                     '寫完不會自動上傳任務。')
    return '\n'.join(lines)


def mapping_tlog_path(callsign: str, when=None):
    """tlog 路徑：runs/survey/<YYYYMMDD>/<callsign>_<HHMMSS>.tlog（專案根目錄下）。"""
    import datetime as _dt
    t = when or _dt.datetime.now()
    return _PROJECT_ROOT / 'runs' / 'survey' / t.strftime('%Y%m%d') / f'{callsign}_{t.strftime("%H%M%S")}.tlog'


def flight_log_path(callsign: str, log_id: int, time_utc: int = 0, when=None):
    """飛控 DataFlash 日誌（.bin）落地路徑：runs/flight_logs/<YYYYMMDD>/<callsign>_log<id>_<HHMMSS>.bin。

    日期時間優先用飛控給的 ``LOG_ENTRY.time_utc``（該份日誌的開始時間，UTC 秒；轉成本地時間），
    飛控沒有 GPS 時間（0）就用現在。⚠ ``log_id`` 是「從最舊起算的序號」，開機刪舊檔會前移，
    所以檔名同時帶時間；SITLLink 那邊存檔遇到同名會自動加 _2/_3，永不覆蓋。
    """
    import datetime as _dt
    if int(time_utc or 0) > 0:
        t = _dt.datetime.fromtimestamp(int(time_utc))
    else:
        t = when or _dt.datetime.now()
    return (_PROJECT_ROOT / 'runs' / 'flight_logs' / t.strftime('%Y%m%d')
            / f'{callsign}_log{int(log_id)}_{t.strftime("%H%M%S")}.bin')


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, '') or default)
    except ValueError:
        return default


def fmt_alt_m(a: float) -> str:
    """規劃高度的顯示字串：到 0.01 m、去掉多餘的 0（100 → '100'、100.4 → '100.4'、173.2051 → '173.21'）。
    不能用 :.0f：100.4 m 會印成「100 m」，而 100 m 與 100.4 m 選到的分層不同（80／120 等距取較低層）。"""
    s = f"{float(a):.2f}".rstrip('0').rstrip('.')
    return '0' if s in ('', '-0') else s


def profile_layer_choice(heights, levels, alt_m: float, used_h: float) -> tuple:
    """分層風依規劃高度選了 used_h 這一層的**實際理由**（不一律說「最接近」）→ (理由, [警告…])。

    heights：分層風有的層；levels：作業區中心**有值**的層（profile_column 的 [{'h_m','speed_mps','dir_from_deg'}]）；
    used_h：core.profile_wind_at 選的層（最接近、等距取較低層、只在中心有值的層裡挑）。情況：
      * 規劃高度高於最高層／低於最低層 ⇒ ⚠ 這是端點層的值、不代表規劃高度的風（不外推）；
      * 最接近的層在作業區中心沒有值 ⇒ 寫明改用哪一層；
      * 兩層一樣近 ⇒ 依規則取較低層；較高那層的風較強時 ⚠ 提醒（風速上限要看較強的）；
      * 其餘 ⇒ 最接近規劃高度的那一層。"""
    hs = sorted(float(h) for h in heights)
    by_h = {float(lv['h_m']): lv for lv in (levels or [])}
    a, u = float(alt_m), float(used_h)
    at = fmt_alt_m(a)
    reasons, warns = [], []
    near = min(hs, key=lambda h: (abs(h - a), h)) if hs else u
    above = bool(hs) and a > hs[-1] + 1e-6
    below = bool(hs) and a < hs[0] - 1e-6
    if above:
        reasons.append(f"規劃高度 {at} m 高於最高層 {hs[-1]:g} m")
    elif below:
        reasons.append(f"規劃高度 {at} m 低於最低層 {hs[0]:g} m")
    if abs(near - u) > 1e-6 and near not in by_h:
        reasons.append(f"最接近規劃高度 {at} m 的 {near:g} m 在作業區中心沒有值，改用 {u:g} m")
    if above or below:
        warns.append(f"⚠ 這是 {u:g} m 的預報值，不代表 {at} m 的風（只有 {'／'.join(f'{h:g}' for h in hs)} m 這幾層，"
                     "不外推" + ("；高處的風可能更強）" if above else "）"))
    elif not reasons:
        tied = [h for h in by_h if abs(h - u) > 1e-6 and abs(abs(h - a) - abs(u - a)) < 1e-6]
        if tied and u in by_h:
            o = tied[0]
            reasons.append(f"規劃高度 {at} m 與 {min(u, o):g}／{max(u, o):g} m 一樣近，依規則取較低的 {u:g} m")
            so, su = float(by_h[o]['speed_mps']), float(by_h[u]['speed_mps'])
            if so > su + 0.05:
                warns.append(f"⚠ {o:g} m 的風較強（{so:.1f} m/s 來自 {float(by_h[o]['dir_from_deg']):.0f}°，"
                             f"{u:g} m 是 {su:.1f} m/s）：檢查風速上限請以較強者為準")
        else:
            reasons.append(f"最接近規劃高度 {at} m 的那一層")
    return '；'.join(reasons), warns


def profile_plan_note(heights, levels, alt_m: float, used_h: float) -> str:
    """分層風寫入規劃器時接在「規劃風已寫入…」後面的說明：'已寫入 80 m 的預報風（理由）[；⚠ 警告…]'。"""
    why, warns = profile_layer_choice(heights, levels, alt_m, used_h)
    return f"已寫入 {float(used_h):g} m 的預報風（{why}）" + ''.join('；' + w for w in warns)


#: 遙測合幀頻率（Hz）：每架最新一幀以這個節奏推到地圖；0 ⇒ 不合幀（每幀都畫，舊行為）。
#: 使用者 2026-10-17 定調畫面要流暢 ⇒ 30 Hz（地圖一批一次 JS，6 架也只有 30 次/秒）；3D 端另有取樣插值把 30 Hz 補成顯示幀率。
#: 環境變數 AEROPLAN_TELEMETRY_HZ 可調。
TELEMETRY_FLUSH_HZ: float = _env_float('AEROPLAN_TELEMETRY_HZ', 30.0)
#: HUD 機卡重繪上限（Hz）：自繪姿態儀一次約 0.6–2 ms，6 張卡 30 Hz 會吃掉一成以上主執行緒；15 Hz 肉眼已平順。
#: 模式／解鎖這類離散狀態改變時不受上限、立刻畫。環境變數 AEROPLAN_HUD_HZ；0 ⇒ 跟合幀同頻。
HUD_MAX_HZ: float = _env_float('AEROPLAN_HUD_HZ', 15.0)


def _forget_tele_state(win, sysid=None) -> None:
    """清掉遙測合幀快取（最新幀／髒旗標）：sysid=None 全清，否則只清那一架。

    ⚠ 全域斷線／HIL 斷開全部／關窗都要呼叫：那幾條路先把 disconnected 信號切掉，逐機清理不會跑，而已排的 flush
      計時器一到就會把斷線的飛機畫回來（幽靈卡片＋地圖標記，審查 2026-10-17 抓到）。
    """
    st = win.__dict__.get('_tele')
    if not st:
        return
    if sysid is None:
        st['latest'].clear()
        st['dirty'].clear()
        st.get('hud_t', {}).clear()
        st.get('hud_state', {}).clear()
        st.get('atts', {}).clear()
        st.get('att_seen', {}).clear()
    else:
        st['latest'].pop(int(sysid), None)
        st['dirty'].discard(int(sysid))
        st.get('hud_t', {}).pop(int(sysid), None)
        st.get('hud_state', {}).pop(int(sysid), None)
        st.get('atts', {}).pop(int(sysid), None)
        st.get('att_seen', {}).pop(int(sysid), None)


#: 合幀之間每架最多留幾筆姿態樣本（50 Hz × 30 Hz 合幀 ≈ 每批 1～2 筆；主執行緒卡住時只留最新的這麼多筆）
ATT_SAMPLES_MAX = 64


def _att_sample_ms(frame):
    """這一幀姿態樣本的時間（epoch ms，浮點）：飛控時間軸換到牆鐘的 att_ts；沒有就退回收到時刻 att_t；都沒有回 None。"""
    for attr in ('att_ts', 'att_t'):
        v = float(getattr(frame, attr, 0.0) or 0.0)
        if v > 0.0 and math.isfinite(v):
            return v * 1000.0
    return None


def _note_att_sample(st: dict, sysid: int, frame) -> None:
    """每一則新的 ATTITUDE（att_t 和上一幀不同）記一筆 [ms, roll, pitch, 機頭] 到 st['atts'][sysid]。

    2026-10-22：合幀每 33 ms 只送每架最新一幀，網路鏈路的 ATTITUDE 是 50 Hz ⇒ 不記下來的話約四成樣本到不了 3D
    （20 Hz 時就已經丟 4%）。flush 時這些比最新一幀舊的樣本放在批次項目的 'atts' 一起送（_take_att_samples）。
    機頭與 _uav_batch_item 的 hdg 同一個規則（收過 ATTITUDE ⇒ yaw，含正北 0.0）。"""
    att_t = float(getattr(frame, 'att_t', 0.0) or 0.0)
    if not (att_t > 0.0 and math.isfinite(att_t)):
        return
    seen = st.setdefault('att_seen', {})
    if seen.get(sysid) == att_t:
        return
    seen[sysid] = att_t
    ms = _att_sample_ms(frame)
    vals = [float(getattr(frame, k, 0.0) or 0.0) for k in ('roll', 'pitch', 'yaw')]
    if ms is None or not all(math.isfinite(v) for v in vals):
        return
    q = st.setdefault('atts', {}).setdefault(sysid, [])
    q.append([ms, vals[0], vals[1], vals[2] % 360.0])
    if len(q) > ATT_SAMPLES_MAX:
        del q[:len(q) - ATT_SAMPLES_MAX]


def _take_att_samples(st: dict, sysid: int, item: dict) -> None:
    """取走這架累積的姿態樣本：比批次項目自己那一筆（'at'）舊的放進 item['atts']（時間由舊到新）；那一筆本身不重複放。"""
    q = st.get('atts', {}).pop(sysid, None)
    if not q:
        return
    at = item.get('at')
    older = [s for s in q if at is None or s[0] < at - 0.5]
    if older:
        item['atts'] = older


def _forget_log_state(win, sysid=None) -> None:
    """清掉主視窗的日誌下載等待狀態與解鎖紀錄：sysid=None 全清，否則只清那一架。

    模組層函式而不是方法：三個呼叫點（逐機斷線、全域斷線、HIL 斷開全部）在測試裡都是用
    假 self 跑的，假 self 沒有這個方法；直接讀 `__dict__` 則讓沒有這兩欄的假 self 也能過。
    """
    for key in ('_log_dl', '_armed_state'):
        d = win.__dict__.get(key)
        if not d:
            continue
        if sysid is None:
            d.clear()
        else:
            d.pop(int(sysid), None)


def pick_latest_log(entries) -> dict | None:
    """``log_list`` 的項目裡挑「最新一份」：id 最大（飛控的序號依時間遞增）；空清單回 None。"""
    rows = [e for e in (entries or []) if isinstance(e, dict) and 'id' in e]
    if not rows:
        return None
    return max(rows, key=lambda e: (int(e.get('id', 0)), int(e.get('time_utc', 0) or 0)))


def assign_links_to_uavs(links, uav_ids):
    """鏈路 → UAV 對應（與既有上傳同一規則）：sysid 等於某 UAV id 就用它，否則給尚未用過的最小 id。

    回傳 ``([(link, uav_id), ...], [沒分到的 sysid, ...])``；同一份任務不會給兩架。
    """
    ids = sorted(int(u) for u in uav_ids)
    used, pairs, skipped = set(), [], []
    direct = {int(getattr(l, 'sysid_label', 0)) for l in links} & set(ids)
    for link in links:
        sysid = int(getattr(link, 'sysid_label', 0))
        if sysid in ids and sysid not in used:
            uid = sysid
        else:
            remaining = [u for u in ids if u not in used and u not in direct]
            if not remaining:
                skipped.append(sysid)
                continue
            uid = remaining[0]
        used.add(uid)
        pairs.append((link, uid))
    return pairs, skipped


def mapping_expected_airspeed(hv, drone_id, camera_cfg) -> dict:
    """匯出 .param 會寫的 AIRSPEED_MIN／MAX（同一支 hv_dccpp.ardupilot.recommended_params；值只看機型 spec，
    與 export_all 傳的其他引數無關）。讀回閘門拿它跟飛控實際值比（R2：AP_Int16 會把 26.6 靜默存成 26）。"""
    from hv_dccpp.ardupilot import recommended_params
    uid = hv.id_map.get(int(drone_id))
    up = hv.last_plan.plans.get(uid) if uid is not None else None
    if up is None:
        raise ValueError(f'drone_id={drone_id} 在這份計畫裡沒有任務')
    spec = hv.last_ctx.specs[up.cls]
    p = recommended_params(spec, None, hv.cfg, sensor_kinds=('rgb',), camera_cfg=camera_cfg, warnings=[])
    return {k: float(p[k]) for k in ('AIRSPEED_MIN', 'AIRSPEED_MAX') if k in p}


# ═════════════════════════════════════════════════════════════════════
#  HARVEST 起飛時程（R1 #1）：4D 去衝突有一部分是靠「延後起飛」錯開的（UAVPlan.t_takeoff_s），
#  建圖模式又不分層 ⇒ 時間是唯一的隔離。任務檔本身不帶時間原點，所以一鍵起飛（全部同時 AUTO）會把
#  規劃的 293 m 間距吃到 12 m（R1 probe stagger2）。這裡：上傳報告列每架的起飛時程、時程不同就擋一鍵起飛、
#  「依時程放行」由 GCS 依時程逐機送同一支 auto_start。
# ═════════════════════════════════════════════════════════════════════
#: 起飛時程差 ≤ 這個秒數視為同時（一鍵起飛不擋）
TAKEOFF_STAGGER_TOL_S = 1.0


def _normalize_takeoff_schedule(sched, id_map) -> dict:
    """``HvDccppCoordinator.takeoff_schedule()`` 的回傳 → ``{drone_id: 秒}``；看不懂 ⇒ ValueError（不猜）。

    接受：``{drone_id: 秒}``、``{drone_id: {'offset_s'|'delay_s'|'t_offset_s': 秒, ...}}``、
    ``{uav_id 字串: ...}``（以 id_map 反查）、外層包一層 ``{'delay_s': {...}}``／``{'offsets': {...}}``、
    或 ``[{'drone_id'|'uav_id', 'offset_s'|'delay_s'}, ...]``。
    """
    keys = ('offset_s', 'delay_s', 't_offset_s', 'offset')
    rev = {str(u): int(d) for d, u in (id_map or {}).items()}

    def _did(k):
        if isinstance(k, bool):
            raise ValueError(f'看不懂的機號 {k!r}')
        if isinstance(k, int):
            return int(k)
        s = str(k)
        if s in rev:
            return rev[s]
        return int(s)

    def _sec(v):
        if isinstance(v, dict):
            for k in keys:
                if v.get(k) is not None:
                    return float(v[k])
            raise ValueError(f'起飛時程項目沒有秒數欄位：{v!r}')
        return float(v)

    if isinstance(sched, dict):
        for wrap in ('offsets', 'delay_s', 'offsets_s'):
            if isinstance(sched.get(wrap), dict):
                sched = sched[wrap]
                break
        out = {_did(k): _sec(v) for k, v in sched.items()}
    elif isinstance(sched, (list, tuple)):
        out = {}
        for it in sched:
            if not isinstance(it, dict):
                raise ValueError(f'起飛時程項目不是 dict：{it!r}')
            k = it.get('drone_id', it.get('uav_id'))
            out[_did(k)] = _sec(it)
    else:
        raise ValueError(f'起飛時程型別看不懂：{type(sched).__name__}')
    for d, s in out.items():
        if not math.isfinite(s) or s < -1e-6:
            raise ValueError(f'機 {d} 的起飛時程 {s!r} 不合法')
    return out


def harvest_takeoff_offsets(hv) -> tuple:
    """HARVEST 計畫每架的起飛時程 → ``({drone_id: 相對全隊最早起飛的秒數}, 來源說明)``。

    優先用轉接層的 ``HvDccppCoordinator.takeoff_schedule()``（另一個套件正在加）；沒有這個方法、或回傳看不懂
    ⇒ 改從 ``last_plan.plans[uid].t_takeoff_s − min`` 自己算（與 export_all 的 ``delay_s`` 同一條：t0 = 全隊最小
    t_takeoff_s），來源字串會說用的是哪一個（有效值契約：不靜默換來源）。只回有任務（在 last_plan.plans 裡）的機。
    """
    fn = getattr(hv, 'takeoff_schedule', None)
    if callable(fn):
        try:
            raw = _normalize_takeoff_schedule(fn(), getattr(hv, 'id_map', None))
            if raw:
                t0 = min(raw.values())
                return {d: float(s) - t0 for d, s in raw.items()}, 'HvDccppCoordinator.takeoff_schedule()'
        except Exception as e:      # noqa: BLE001 — 看不懂就改用規劃本身的 t_takeoff_s，但要說出來
            logger.error(f'[起飛時程] takeoff_schedule() 的結果用不了（{type(e).__name__}: {e}）'
                         f'⇒ 改用 UAVPlan.t_takeoff_s 自己算')
    plan = getattr(hv, 'last_plan', None)
    plans = dict(getattr(plan, 'plans', None) or {})
    if not plans:
        return {}, 'UAVPlan.t_takeoff_s（沒有計畫）'
    t0 = min(float(p.t_takeoff_s) for p in plans.values())
    out = {}
    for did, uid in (getattr(hv, 'id_map', None) or {}).items():
        p = plans.get(uid)
        if p is not None:
            out[int(did)] = float(p.t_takeoff_s) - t0
    return out, 'UAVPlan.t_takeoff_s − 全隊最早起飛'


def link_flight_state(link) -> tuple:
    """這條連線的飛控**能不能**被換任務／放行 → ``(代碼, 原因)``；可以 ⇒ ``(None, None)``。

    代碼：``'armed'``（已解鎖，可能在飛）、``'auto'``（AUTO 模式：換任務會改到正在執行的任務）、
    ``'unknown'``（沒有遙測或還沒收到心跳、模式 '---'：狀態不明，不猜）。
    """
    sysid = getattr(link, 'sysid_label', '?')
    get = getattr(link, 'get_latest_telemetry', None)
    f = None
    if callable(get):
        try:
            f = get()
        except Exception:          # noqa: BLE001
            f = None
    if f is None:
        return 'unknown', f'UAV-{sysid} 還沒收到遙測，無法確認未解鎖'
    mode = str(getattr(f, 'mode', '---') or '---').upper()
    if bool(getattr(f, 'armed', False)):
        return 'armed', f'UAV-{sysid} 已解鎖（模式 {mode}）'
    if mode in ('---', ''):
        return 'unknown', f'UAV-{sysid} 還沒收到心跳，無法確認飛行模式'
    if mode == 'AUTO':
        return 'auto', f'UAV-{sysid} 在 AUTO 模式'
    return None, None


def link_flight_state_problem(link):
    """:func:`link_flight_state` 的原因字串（None ＝ 可以）。"""
    return link_flight_state(link)[1]


def build_strip_monitor(hv, drone_id, items):
    """建圖上傳的任務項目 → 條帶監測器（``survey.strip_monitor.StripPhotoMonitor``）：段表＋條帶 id＋逐段預估張數。

    回傳 (monitor, None) 或 (None, 原因)。原因要給操作員看（沒有監測器 ⇒ 只剩總張數、沒有逐條帶警報）：
    段數與計畫條帶數不符時**不**硬湊（不靜默錯位）。逐段預估與「預估 M 張」用同一個模型
    （``expected_photos_by_segment``；合計 ＋ 帶外 ＝ ``expected_photo_count``）。
    """
    try:
        from survey.strip_monitor import StripPhotoMonitor, camera_segments
        from hv_dccpp.integration.aeroplan_adapter import expected_photos_by_segment
        uid = hv.id_map[int(drone_id)]
        up = hv.last_plan.plans[uid]
        strip_ids = [s for s, _d in up.strips]
        by_seg, _outside = expected_photos_by_segment(items)
        segs = camera_segments(items, strip_ids=strip_ids, expected=by_seg)
        return StripPhotoMonitor(segs), None
    except Exception as e:          # noqa: BLE001 - 照實說，不讓上傳失敗
        return None, f'{type(e).__name__}: {e}'


def send_link_command(link, cmd: str, arg=None):
    """逐機送一個 HUD 指令。一鍵起飛（_sitl_broadcast）與「依時程放行」走**同一支**：
    auto_start ＝ SITLLink.auto_start()（AUTO + ARM + MISSION_START）。"""
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


def schedule_spread_s(entries) -> float:
    """起飛時程的最大差（秒）。"""
    offs = [float(e['offset_s']) for e in entries]
    return (max(offs) - min(offs)) if offs else 0.0


def release_schedule_lines(entries) -> list:
    return [f'UAV-{e["sysid"]}（機 {e["uav"]} {e.get("callsign", "?")}）：T+{e["offset_s"]:.0f} s'
            for e in sorted(entries, key=lambda x: (x['offset_s'], x['sysid']))]


def _refresh_countdown_of(win) -> None:
    fn = getattr(win, '_refresh_release_countdown', None)
    if callable(fn):
        fn()


def _cancel_active_release(win, reason: str) -> list:
    """飛控上的任務／時程變了（重新上傳、Write WPs、別的上傳路徑）⇒ 還在倒數的放行用的是舊時程，全部取消。"""
    rel = getattr(win, '_release', None)
    if rel is None or not getattr(rel, 'active', False):
        return []
    cancelled = rel.cancel()
    msg = (f'[放行] {reason} ⇒ 還在倒數的放行全部取消（UAV-{cancelled} 不會被送出起飛）；'
           f'確認新任務後再按「依時程放行」')
    logger.error(msg)
    try:
        win.statusBar().showMessage(msg, 15000)
    except Exception:          # noqa: BLE001
        pass
    return cancelled


def record_release_schedule(win, hv, pairs, *, mapping: bool):
    """上傳了 HARVEST 任務 ⇒ 在 ``win._hv_release_schedule`` 記下飛控上這些任務的起飛時程。
    pairs：``[(sysid, drone_id), ...]``（真的送出寫入的）。同一份結果（同一個 hv、同一個 last_result）分批上傳
    ⇒ 合併；換了結果 ⇒ 取代。回傳 ``(entries, 來源)``，entries ＝ ``{sysid: {'sysid','uav','offset_s','callsign'}}``。
    """
    offsets, source = harvest_takeoff_offsets(hv)
    _cancel_active_release(win, '重新上傳了 HARVEST 任務')
    result = getattr(hv, 'last_result', None)
    old = getattr(win, '_hv_release_schedule', None)
    same = bool(old) and old.get('hv') is hv and old.get('result') is result
    entries = dict(old['entries']) if same else {}
    upload_ok = dict(old.get('upload_ok') or {}) if same else {}
    if old and not same:
        logger.info('[起飛時程] 新的 HARVEST 結果上傳，取代舊的起飛時程')
    missing = []
    for sysid, did in pairs:
        off = offsets.get(int(did))
        if off is None:
            missing.append(int(did))
            continue
        entries[int(sysid)] = {'sysid': int(sysid), 'uav': int(did), 'offset_s': float(off),
                               'callsign': (getattr(hv, 'id_map', None) or {}).get(int(did), '?')}
        upload_ok[int(sysid)] = None
    if missing:
        # 沒有時程 = 不知道該何時起飛 ⇒ 不列入（不能依時程放行）；一鍵起飛仍會被其他機的時程擋
        logger.error(f'[起飛時程] 機 {missing} 沒有起飛時程（{source}）⇒ 不能依時程放行')
    win._hv_release_schedule = {'hv': hv, 'result': result, 'source': source, 'mapping': bool(mapping),
                                'entries': entries, 'upload_ok': upload_ok}
    logger.warning('[起飛時程] 飛控上的 HARVEST 任務：' + '；'.join(
        f'UAV-{e["sysid"]}（機 {e["uav"]} {e["callsign"]}）T+{e["offset_s"]:.1f} s'
        for e in sorted(entries.values(), key=lambda x: (x['offset_s'], x['sysid']))) + f'（來源：{source}）')
    _refresh_countdown_of(win)
    return entries, source


def forget_release_schedule(win, sysids=None, reason: str = '') -> None:
    """這些飛控的任務被別的上傳路徑換掉了 ⇒ 移出 HARVEST 起飛時程（None ＝ 全部清掉）。"""
    sched = getattr(win, '_hv_release_schedule', None)
    if not sched:
        return
    if sysids is None:
        win._hv_release_schedule = None
        logger.info(f'[起飛時程] 清除起飛時程（{reason}）')
        _cancel_active_release(win, f'起飛時程清除（{reason}）')
    else:
        gone = [int(s) for s in sysids if int(s) in sched['entries']]
        for s in gone:
            sched['entries'].pop(s, None)
            sched.get('upload_ok', {}).pop(s, None)
        if gone:
            logger.info(f'[起飛時程] UAV-{gone} 的任務已被換掉（{reason}），移出起飛時程')
            _cancel_active_release(win, f'UAV-{gone} 的任務已被換掉（{reason}）')
    _refresh_countdown_of(win)


def staggered_start_block(schedule, links):
    """一鍵起飛（auto_start／mission_start）要不要擋 → 拒絕訊息或 None。

    擋：飛控上的 HARVEST 任務起飛時程不同（差 > TAKEOFF_STAGGER_TOL_S），而這次的指令對象裡有其中任何一架
    （包括只選一架：單獨提早或延後起飛同樣破壞相對時程）。時程相同、或對象都不是 HARVEST 任務 ⇒ None（行為不變）。
    """
    if not schedule or not schedule.get('entries'):
        return None
    entries = list(schedule['entries'].values())
    if schedule_spread_s(entries) <= TAKEOFF_STAGGER_TOL_S:
        return None
    hit = sorted(int(getattr(l, 'sysid_label', 0)) for l in links
                 if int(getattr(l, 'sysid_label', 0)) in schedule['entries'])
    if not hit:
        return None
    return ('飛控上的 HARVEST 任務起飛時程不同（4D 去衝突有一部分靠延後起飛錯開；'
            + ('建圖模式不分層，時間是唯一的隔離' if schedule.get('mapping') else '同時起飛會失去這段隔離')
            + '）。\n一鍵起飛會讓這些飛機同時起飛，不允許：\n  '
            + '\n  '.join(release_schedule_lines(entries))
            + f'\n\n（這次指令對象裡受影響的：UAV-{hit}）\n'
            + '請改用 HUD 的「依時程放行」：時程 T+0 的立刻起飛，其他的依時程倒數後自動起飛，可「取消放行」。')


def release_report_text(schedule, entries, source: str, *, mapping: bool) -> str:
    """上傳報告的「起飛時程」段（每架 T+秒）：時程不同 ⇒ 大聲說一鍵起飛會被擋、要用「依時程放行」。
    schedule ＝ 合併後的 ``_hv_release_schedule``（分批上傳時列全部）；沒有就用這次的 entries。"""
    all_entries = list(((schedule or {}).get('entries') or entries or {}).values())
    if not all_entries:
        return '起飛時程：沒有（這些任務不能依時程放行）'
    spread = schedule_spread_s(all_entries)
    head = f'起飛時程（相對全隊最早起飛；來源 {source}）：\n  ' + '\n  '.join(release_schedule_lines(all_entries))
    if spread > TAKEOFF_STAGGER_TOL_S:
        return (head + '\n各機起飛時間不同：4D 去衝突有一部分靠延後起飛錯開'
                + ('（建圖模式不分層，時間是唯一的隔離）' if mapping else '')
                + ' ⇒ 「一鍵起飛」會被擋，請用 HUD 的「依時程放行」。')
    return head + f'\n各機起飛時程相同（差 ≤ {TAKEOFF_STAGGER_TOL_S:g} s），可以一鍵起飛。'


def hv_upload_gate(hv, swarm_coordinator) -> dict:
    """建圖上傳閘門 = 轉接層 ``mapping_gate()``（未覆蓋／mapping_incomplete／NO_GO）＋ validate 的原因（R1 #8）
    ＋ 飛前曝光驗證（B3，``mapping_exposure_block_reasons``：內部有視數不足的破洞、或驗證沒跑到／沒跑成功）。

    validate 與曝光驗證的結果都在**產生這個 hv 的** SwarmCoordinator（``hv_validation_reasons``、
    ``hv_exposure_summary``）。對不上（不是同一個 hv）、或沒跑到 ⇒ 不可上傳（沒驗過不能當驗過）。
    接縫最小重疊、邊緣帶視數不足**不在這裡**（只顯示，見 :func:`mapping_exposure_lines`）。
    """
    from mission.swarm_coordinator import mapping_exposure_block_reasons
    gate = hv.mapping_gate()
    reasons = list(gate.get('reasons') or [])
    sc = swarm_coordinator
    if sc is not None and getattr(sc, 'hv_coordinator', None) is hv:
        vr = getattr(sc, 'hv_validation_reasons', None)
        if vr is None:
            reasons.append('validate 沒有跑到（這份計畫沒有經過時空去衝突與能量驗證）')
        else:
            reasons.extend(str(r) for r in vr)
        reasons.extend(mapping_exposure_block_reasons(getattr(sc, 'hv_exposure_summary', None)))
    else:
        reasons.append('找不到這份計畫的 validate 結果（時空去衝突／能量驗證）')
        reasons.append('找不到這份計畫的飛前曝光驗證結果（視數）')
    return {'uploadable': not reasons, 'reasons': reasons}


def mapping_exposure_lines(summary) -> list:
    """飛前曝光驗證摘要（``SwarmCoordinator.hv_exposure_summary``）→ 偵察頁「航測建圖」狀態的幾行字。

    標明這是**曝光層級（視數）**：每格被幾張模擬照片看到；與「覆蓋率驗算」按鈕的航跡 ± 刈幅幾何覆蓋率是不同的量。
    內部破洞會擋上傳；邊緣帶與跨機接縫只顯示（驗證器不建模側傾門檻與轉彎動態）。用了哪些值、沒建模什麼也列出來。
    """
    from mission.swarm_coordinator import MAPPING_EXPOSURE_SEAM_CAVEAT, mapping_exposure_block_reasons
    head = '曝光層級（視數）—— 飛前模擬 AP_Camera 距離觸發、每格被幾張照片看到；與「覆蓋率驗算」的航跡 ± 刈幅覆蓋率不同'
    if summary is None or summary.get('ok') is not True:
        return [head + '：', '  ' + mapping_exposure_block_reasons(summary)[0] + ' ⇒ 不可上傳']
    k = summary['min_views']
    eb = summary.get('edge_band_m')
    drone_of = summary.get('drone_of') or {}

    def _who(u):
        return f'機 {drone_of[u]}（{u}）' if u in drone_of else str(u)
    lines = [head + f'（格 {summary["cell_size_m"]:g} m）：']
    inter = summary.get('interior')
    eb_txt = f'離測區邊界 > {float(eb):.1f} m＝半刈幅' if eb is not None else '沒有照片 ⇒ 不分邊緣帶'
    if inter is not None:
        mv = inter.get('min_views')
        lines.append(f'  內部（{eb_txt}）：視數 ≥{k} {float(inter["frac_ge"].get(k, 0.0)):.2%}'
                     + ('' if mv is None else f'、最少 {mv} 視'))
    n_hole, a_hole = summary.get('interior_hole_count', 0), float(summary.get('undercovered_interior_area_m2') or 0.0)
    if a_hole > 0.0:
        lines.append(f'  內部視數 < {k} 的破洞 {n_hole} 處 {a_hole:,.0f} m² ⇒ 擋上傳：'
                     + '、'.join(f'{h["area_m2"]:,.0f} m²（最少 {h["min_views"]} 視，中心 '
                                 f'{h["centroid_latlon"][0]:.6f}, {h["centroid_latlon"][1]:.6f}）'
                                 for h in (summary.get('interior_holes') or [])[:3]))
    else:
        lines.append(f'  內部視數 < {k} 的破洞：0 處（有就擋上傳）')
    edge = summary.get('edge_band')
    if edge is not None:
        lines.append(f'  邊緣帶（≤ {float(eb):.1f} m）：視數 < {k} 共 {summary.get("edge_band_short_count", 0)} 處 '
                     f'{float(summary.get("undercovered_edge_band_area_m2") or 0.0):,.0f} m²'
                     f'（佔邊緣帶 {float(edge["undercovered_frac"]):.2%}'
                     + ('' if edge.get('min_views') is None else f'，最少 {edge["min_views"]} 視')
                     + '）—— 只顯示不擋')
    seams = summary.get('seams') or []
    if seams:
        lines.append(f'  跨機接縫最窄 {float(summary["seam_overlap_min"]):.2f} 刈幅（'
                     + '、'.join(f'{s["uav_a"]}/{s["uav_b"]} {s["frac_min"]:.2f}'
                                 + (f'（隔 {float(s["gap_m"]):.0f} m）' if s.get('gap_m') else '') for s in seams)
                     + f'）—— 只顯示不擋（{MAPPING_EXPOSURE_SEAM_CAVEAT}）')
    else:
        lines.append('  跨機接縫：沒有相鄰的兩架（單機或區塊不相接）')
    ppu = summary.get('photos_per_uav') or {}
    if ppu:
        lines.append('  模擬張數：' + '、'.join(f'{_who(u)} {n} 張' for u, n in sorted(
            ppu.items(), key=lambda kv: drone_of.get(kv[0], 0))))
    eff = summary.get('effective') or {}
    spd = eff.get('airspeed_initial_mps') or {}
    lines.append('  有效值：' + '；'.join(x for x in (
        (f'風 {eff["wind"][0]:g} m/s 來自 {eff["wind"][1]:g}°（{eff.get("wind_source", "")}）' if eff.get('wind') else ''),
        f'地形 {eff.get("terrain", "?")}',
        f'測區 {eff.get("polygon_source", "?")}',
        ('初始空速 ' + '、'.join(f'{u} {v:g}' for u, v in spd.items()) + ' m/s') if spd else '',
        (f'CAM1_INTRVAL_MIN {float(eff["intrval_min_s"]):.2f} s、CAM1_DURATION {float(eff["trigger_duration_s"]):g} s'
         if eff.get('intrval_min_s') is not None else '')) if x))
    lines.append('  假設：' + '；'.join(summary.get('assumptions') or []))
    return lines


def gcp_blocks_provider(win):
    """「GCP／檢核點規劃」要的 UAV 區塊：回傳一個**呼叫時才讀**最新 HARVEST 規劃的函式（對話框是非模態的）。

    回傳函式的結果：``(區塊, 說明, 來源說明)``；沒有 HARVEST 規劃 ⇒ ``(None, {}, 說明)``。
    區塊由 ``survey.gcp_plan.blocks_from_hv`` 從 ``win._hv_coordinator`` 的 last_plan／last_ctx **唯讀**取出；
    規劃結果有但讀不出來時讓例外往上丟（對話框會照實顯示，不默默改成不分區塊）。
    """
    def _provider():
        from survey import gcp_plan
        hv = getattr(win, '_hv_coordinator', None)
        if hv is None or getattr(hv, 'last_plan', None) is None:
            return None, {}, '沒有 HARVEST 規劃：不分 UAV 區塊（每區塊 GCP 與接縫檢核點對不會產生）'
        blocks, info = gcp_plan.blocks_from_hv(hv)
        names = '、'.join(f"{k}（#{v.get('drone_id')}）" for k, v in sorted(info.items()))
        return blocks, info, f'HARVEST 最近一次規劃：{len(blocks)} 架 {names}'
    return _provider


# ═════════════════════════════════════════════════════════════════════
#  飛行計畫快照（survey.plan_snapshot，2026-10-20 Step 0）：建圖任務上傳成功時把「真的飛的那一份」存檔，
#  後處理（航測處理對話框）可以改讀它，不再依賴即時的規劃器／地圖狀態。
#  兩步：送出寫入後先在記憶體建好（prepare_…），ACK 回來第一個成功才寫檔、之後就地更新（note_…）。
#  模組層級 ⇒ 假 self 也能跑（見 tests/test_survey_process_wiring.py）。
# ═════════════════════════════════════════════════════════════════════
#: 同時等 ACK 的批數上限（每批一份快照；超過就丟最舊的——那批的 ACK 已經不可能還沒回來）
FLOWN_PLAN_PENDING_MAX = 8
#: 建圖「匯出」的說明（審查：這條路不產生快照，要明講）
FLOWN_PLAN_EXPORT_NOTE = ('注意：匯出的檔案不會產生飛行計畫快照——快照只在 AeroPlan 自己把建圖任務上傳、飛控確認寫入成功時才存。'
                          '用 Mission Planner 等外部工具上傳這些檔案時，航測後處理只能用目前規劃器的即時狀態'
                          '（重新規劃、改作業區或重開程式之後就不一定是飛的那一份）。')


def _status_message(win, msg: str, ms: int = 15000) -> None:
    try:
        win.statusBar().showMessage(msg, ms)
    except Exception:          # noqa: BLE001 — 假 self／視窗正在關
        pass


def prepare_flown_plan_snapshot(win, hv, jobs_sent, *, takeoff_offsets, takeoff_source, readback_info=None,
                                skipped=(), not_targeted=(), tlog=None):
    """建圖任務**送出寫入之後**：把這批（只含真的送出的機）的飛行計畫快照建在記憶體 ``win._flown_plan_pending``。

    還不寫檔——要等飛控的 ACK（:func:`note_flown_plan_upload`），沒有任何一架寫成功就不寫。
    回傳問題字串（放進上傳報告）或 None；**永遠不丟例外**：快照壞了不能擋上傳，但要大聲說。
    """
    jobs_sent = list(jobs_sent or [])
    if not jobs_sent:
        return None
    try:
        from survey import plan_snapshot as _ps
        snap = _ps.snapshot_from_hv(hv, _win_attr(win, '_hv_swarm_coordinator'), jobs_sent,
                                    takeoff_offsets=takeoff_offsets, takeoff_source=takeoff_source,
                                    readback_info=readback_info, skipped=list(skipped or ()),
                                    not_targeted=list(not_targeted or ()), tlog=tlog)
    except Exception as e:     # noqa: BLE001 — 照實說，不擋上傳
        msg = (f'飛行計畫快照沒建成（任務照常上傳；後處理只能用目前規劃器的即時狀態）：{type(e).__name__}: {e}')
        logger.error(f'[航測] {msg}', exc_info=True)
        return msg
    sysids = {int(j['sysid']) for j in jobs_sent}
    for _sid in sorted(sysids):                 # 舊批次還在等這幾架 ⇒ 那些 ACK 不會再來（被這次的寫入取代）
        abandon_flown_plan_sysid(win, _sid, '同一架又送出了新的建圖任務')
    pend = list(_win_attr(win, '_flown_plan_pending') or [])
    pend.append({'snapshot': snap, 'path': None, 'sysids': sysids, 'reported': set(), 'error': None})
    win._flown_plan_pending = pend[-FLOWN_PLAN_PENDING_MAX:]
    logger.info(f'[航測] 飛行計畫快照已建好（plan {str(snap.get("plan_sha256"))[:12]}，UAV-{sorted(sysids)}）；'
                f'等第一個寫入成功的 ACK 才存檔')
    return None


def note_flown_plan_upload(win, sysid, ok, message):
    """``_on_mission_uploaded`` 的 ACK → 對應那批的飛行計畫快照：第一個成功才寫檔（狀態列寫出檔名），之後就地更新
    每架的結果。寫檔／更新失敗：log＋狀態列＋警告視窗，**不影響**任務上傳本身。回傳寫出／更新的路徑或 None。"""
    pend = _win_attr(win, '_flown_plan_pending')
    if not pend:
        return None
    sid = int(sysid)
    entry = next((e for e in reversed(pend) if sid in e['sysids'] and sid not in e['reported']), None)
    if entry is None:
        return None             # 不是建圖這批的寫入（Write WPs 等），或這架的結果已記過
    return _record_flown_plan_ack(win, entry, sid, ok, message)


def abandon_flown_plan_sysid(win, sysid, reason: str) -> int:
    """這架的建圖任務寫入結果不會（或不該）再對到快照：連線中斷（ACK 永遠不會來）、或這架開始寫別的任務
    （Write WPs／新的建圖上傳）⇒ 每批還在等它的快照都記成 ok=False（訊息＝原因），之後同一架的 ACK 不會被誤記成
    「建圖任務寫入成功」（審查：只靠 sysid 對 ACK 時，連線中斷後的 Write WPs ACK 會被當成建圖那批的結果）。
    回傳處理了幾批。永遠不丟例外。"""
    pend = _win_attr(win, '_flown_plan_pending')
    if not pend:
        return 0
    try:
        sid = int(sysid)
    except (TypeError, ValueError):
        return 0
    hit = [e for e in list(pend) if sid in e['sysids'] and sid not in e['reported']]
    for entry in hit:
        logger.warning(f'[航測] 飛行計畫快照：UAV-{sid} 沒有收到建圖任務的寫入結果（{reason}）⇒ 記為未確認')
        _record_flown_plan_ack(win, entry, sid, False, f'沒有收到建圖任務的寫入結果：{reason}')
    return len(hit)


def _record_flown_plan_ack(win, entry, sid, ok, message):
    """把一架的寫入結果記進那一批的快照：第一個成功才寫檔，之後就地更新；這批全部回報完就收掉。"""
    entry['reported'].add(sid)
    path = None
    try:
        from survey import plan_snapshot as _ps
        if entry['path'] is None:
            entry['snapshot'] = _ps.set_upload_status(entry['snapshot'], sid, ok=bool(ok), message=message)
            if _ps.any_upload_ok(entry['snapshot']):
                path = entry['path'] = _ps.write_snapshot(_ps.default_root(), entry['snapshot'])
                logger.info(f'[航測] 飛行計畫快照已存：{path}（plan {str(entry["snapshot"].get("plan_sha256"))[:12]}）')
                _status_message(win, f'✅ UAV-{sid} 任務寫入完成；飛行計畫快照已存：{path}', 20000)
        else:
            entry['snapshot'] = _ps.update_upload_status(entry['path'], sid, ok=bool(ok), message=message)
            path = entry['path']
            logger.info(f'[航測] 飛行計畫快照 {Path(path).name}：記下 UAV-{sid} 寫入{"成功" if ok else "失敗"}')
    except Exception as e:      # noqa: BLE001 — 快照寫不成不能讓上傳看起來失敗，但一定要大聲
        what = '更新' if entry['path'] is not None else '存檔'
        msg = (f'飛行計畫快照{what}失敗（任務上傳不受影響；後處理只能用目前規劃器的即時狀態）：'
               f'{type(e).__name__}: {e}')
        entry['error'] = msg
        logger.error(f'[航測] {msg}', exc_info=True)
        _status_message(win, f'⚠ {msg}', 30000)
        try:
            QMessageBox.warning(win, '飛行計畫快照', msg)
        except Exception:       # noqa: BLE001 — 假 self（不是 QWidget）
            pass
    finally:
        if entry['reported'] >= entry['sysids']:
            win._flown_plan_pending = [e for e in (_win_attr(win, '_flown_plan_pending') or []) if e is not entry]
            if entry['path'] is None and entry['error'] is None:
                logger.warning('[航測] 這批建圖任務沒有任何一架寫入成功 ⇒ 不存飛行計畫快照')
    return path


# ═════════════════════════════════════════════════════════════════════
#  關主視窗時航測處理對話框的背景工作（處理、ODM 上傳／等待／下載、接回、取消遠端任務）
# ═════════════════════════════════════════════════════════════════════
#: 「停止並關閉」時合計最多等幾毫秒（ODM 上傳一個 16 KiB 區塊、輪詢 ≤ 0.5 s＋一次 API、下載一個 1 MiB 區塊就會停；
#  處理的單一步驟——讀大日誌、RTKLIB——停不下來，等不到就寄放）
SURVEY_CLOSE_WAIT_MS = 10000


def _park_orphan_thread(worker, tag: str) -> None:
    """還在跑的 QThread（信號已拆、parent 已是 None）寄放到模組層級，程式結束前由 ``_hv_wait_for_orphan_workers`` 統一等。"""
    _HV_ORPHAN_WORKERS.add(worker)
    try:
        worker.finished.connect(lambda w=worker: _HV_ORPHAN_WORKERS.discard(w))
        if worker.isFinished():                              # 剛好在 connect 之前結束
            _HV_ORPHAN_WORKERS.discard(worker)
            return
    except Exception:          # noqa: BLE001
        pass
    try:
        from PyQt6.QtWidgets import QApplication
        from PyQt6.QtCore import Qt as _Qt
        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(_hv_wait_for_orphan_workers, _Qt.ConnectionType.UniqueConnection)
    except Exception:          # noqa: BLE001
        pass
    logger.warning(f'[{tag}] 視窗關閉時背景工作仍在執行，已寄放該執行緒等它自行結束')


def _ask_close_survey(win, title: str, text: str) -> bool:
    r = QMessageBox.question(win, title, text, QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                             QMessageBox.StandardButton.No)
    return r == QMessageBox.StandardButton.Yes


def _survey_dialog_busy(win):
    """(對話框, 忙不忙, 說明)；對話框不在或 C++ 物件已刪 ⇒ (None, False, '')。"""
    dlg = _win_attr(win, '_survey_process_dialog')
    if dlg is None:
        return None, False, ''
    try:
        busy = bool(dlg.is_busy())
        desc = dlg.busy_description() if hasattr(dlg, 'busy_description') else '航測處理／ODM'
    except (RuntimeError, AttributeError):
        return None, False, ''  # 對話框的 C++ 物件已不在（它的執行緒在對話框關閉時就處理過了）
    return dlg, busy, desc


def survey_close_ask(win, *, ask=None):
    """關主視窗前**只問、不動任何東西**：航測處理對話框還有背景工作 ⇒ 問「停止並關閉／不關」。

    回傳 None ＝ 沒有背景工作（不用問）、True ＝ 使用者選停止並關閉、False ＝ 不關（呼叫端 event.ignore()）。
    真的停止要等關閉**定案**（後面的 STOT、未儲存變更的提問都沒有否決）才由 :func:`survey_close_finish` 做——
    選了「是」之後被後面的提問否決時，本機工作（尤其是還沒提交、不能接續的上傳）不能已經被停掉（審查）。
    """
    dlg, busy, desc = _survey_dialog_busy(win)
    if not busy:
        return None
    pend = None
    try:
        pend = dlg.pending_remote_cancel_text() if hasattr(dlg, 'pending_remote_cancel_text') else None
    except RuntimeError:
        pend = None
    text = (f'航測處理對話框還有背景工作在跑：{desc}。\n\n'
            '「是」＝停止本機的工作再關閉：處理會停在目前的步驟（輸出資料夾可能只寫了一部分）；'
            'ODM 已提交的遠端任務會在 NodeODX 上繼續跑，之後開「航測處理…」用「接回既有任務…」繼續等待／下載；'
            '還沒提交的上傳會被放棄（節點 48 h 後自清暫存）。\n'
            + (f'⚠ {pend}：關閉後**不會**送出，遠端任務會繼續跑——要取消請選「否」，等取消送出後再關。\n' if pend else '')
            + '「否」＝不關閉。')
    if not (ask or _ask_close_survey)(win, '航測處理仍在執行', text):
        logger.info('[航測處理] 關閉主視窗：使用者選擇不關（背景工作繼續）')
        return False
    logger.info(f'[航測處理] 關閉主視窗：使用者選擇停止 {desc}（等關閉定案才停）')
    return True


def survey_close_finish(win, *, wait_ms: int = SURVEY_CLOSE_WAIT_MS) -> list:
    """關閉**已定案**：要求停止、合計最多等 ``wait_ms``；停不下來的執行緒拆信號、parent=None、寄放到模組層級
    （在還在跑的 QThread 上解構是 qFatal）。問的時候不忙、之後才開始的工作（例如等著送的取消遠端任務）也一樣要停。
    回傳寄放的執行緒。"""
    dlg, busy, desc = _survey_dialog_busy(win)
    if not busy:
        return []
    left = dlg.shutdown_workers(wait_ms)
    for w in left:
        _park_orphan_thread(w, '航測處理')
    logger.warning(f'[航測處理] 關閉主視窗：已要求停止 {desc}'
                   + (f'；{len(left)} 個執行緒還沒停下來，已寄放' if left else ''))
    return left


def survey_close_decision(win, *, wait_ms: int = SURVEY_CLOSE_WAIT_MS, ask=None) -> bool:
    """問＋（選「是」就）立刻停止：:func:`survey_close_ask` 再 :func:`survey_close_finish`。回傳 False ＝ 不關。
    closeEvent 不用這支（它要等所有可以否決關閉的提問都問完才停）；留給只需要這一個問題的呼叫端。"""
    ans = survey_close_ask(win, ask=ask)
    if ans is False:
        return False
    if ans:
        survey_close_finish(win, wait_ms=wait_ms)
    return True


def _qt_single_shot(delay_s: float, callback):
    """預設計時器：PreciseTimer（⚠ 預設的 CoarseTimer 誤差可達間隔的 5%，1516 s 就是 ±76 s）。"""
    from PyQt6.QtCore import QTimer, Qt as _Qt
    t = QTimer()
    t.setSingleShot(True)
    t.setTimerType(_Qt.TimerType.PreciseTimer)
    t.timeout.connect(callback)
    t.start(max(0, int(round(float(delay_s) * 1000.0))))
    return t


class StaggeredRelease:
    """依起飛時程逐機放行：時程 0 的立刻送、其他的用計時器在各自的時程送（同一支 ``start_fn``）。

    ``start_fn(entry)``：真的送出起飛（MainWindow 用與「一鍵起飛」同一支的逐機 auto_start）；丟例外 ⇒ 該機
    記為放行失敗（例外訊息就是原因）。``timer_factory(delay_s, callback)`` 回傳有 ``stop()`` 的物件；
    ``clock()`` 單調秒數；``wall()`` 牆上時間（log 用）。三者都可注入（測試用假時鐘／假計時器）。
    每一次放行（成功／失敗／取消）都記進 ``events`` 並寫 log。
    """

    def __init__(self, start_fn, *, timer_factory=None, clock=None, wall=None, on_change=None):
        import time as _time
        import datetime as _dt
        self._start_fn = start_fn
        self._timer_factory = timer_factory or _qt_single_shot
        self._clock = clock or _time.monotonic
        self._wall = wall or _dt.datetime.now
        self._on_change = on_change
        self._t0 = None
        self._pending = {}          # sysid → {'entry', 'delay_s', 'timer'}
        self.events = []

    @property
    def active(self) -> bool:
        return bool(self._pending)

    def start(self, entries) -> list:
        """entries：``[{'sysid', 'uav', 'offset_s', ...}]``；延遲 = offset − 這批最小的 offset。回傳延遲表。"""
        if self._pending:
            raise RuntimeError('上一批放行還沒結束，請先「取消放行」')
        entries = [dict(e) for e in entries]
        if not entries:
            raise ValueError('沒有要放行的飛機')
        base = min(float(e['offset_s']) for e in entries)
        self._t0 = float(self._clock())
        plan = []
        for e in sorted(entries, key=lambda x: (float(x['offset_s']), int(x['sysid']))):
            delay = float(e['offset_s']) - base
            e['delay_s'] = delay
            plan.append((int(e['sysid']), delay))
        logger.warning('[放行] 依時程放行開始（T0 = ' + self._wall().strftime('%H:%M:%S') + '）：' + '；'.join(
            f'UAV-{s} T+{d:.1f} s' for s, d in plan))
        for e in sorted(entries, key=lambda x: float(x['delay_s'])):
            if e['delay_s'] <= TAKEOFF_STAGGER_TOL_S:
                self._fire(e)
            else:
                sysid = int(e['sysid'])
                self._pending[sysid] = {'entry': e, 'delay_s': e['delay_s'], 'timer': None}
                self._pending[sysid]['timer'] = self._timer_factory(e['delay_s'],
                                                                    lambda s=sysid: self._on_timer(s))
        self._changed()
        return plan

    def _on_timer(self, sysid: int) -> None:
        p = self._pending.pop(int(sysid), None)
        if p is None:
            return              # 已取消
        self._fire(p['entry'])
        self._changed()

    def _fire(self, e: dict) -> None:
        now = float(self._clock())
        actual = now - (self._t0 if self._t0 is not None else now)
        ev = {'sysid': int(e['sysid']), 'uav': e.get('uav'), 'planned_s': float(e.get('delay_s', 0.0)),
              'actual_s': actual, 'wall': self._wall().strftime('%H:%M:%S'), 'ok': True, 'error': None}
        try:
            self._start_fn(e)
        except Exception as ex:      # noqa: BLE001 — 放不了要說出原因，其他機照時程
            ev.update(ok=False, error=str(ex))
        self.events.append(ev)
        if ev['ok']:
            logger.warning(f'[放行] UAV-{ev["sysid"]}（機 {ev["uav"]}）已送出 AUTO+ARM+START：預定 T+{ev["planned_s"]:.1f} s、'
                           f'實際 T+{actual:.2f} s（{ev["wall"]}）')
        else:
            logger.error(f'[放行] UAV-{ev["sysid"]}（機 {ev["uav"]}）放行失敗：{ev["error"]}（預定 T+{ev["planned_s"]:.1f} s、'
                         f'{ev["wall"]}）')

    def cancel(self) -> list:
        """停掉所有還沒放行的計時器；回傳被取消的 sysid。"""
        cancelled = sorted(self._pending)
        for s in cancelled:
            t = self._pending[s].get('timer')
            try:
                if t is not None:
                    t.stop()
            except Exception:          # noqa: BLE001
                pass
            self.events.append({'sysid': s, 'uav': self._pending[s]['entry'].get('uav'),
                                'planned_s': self._pending[s]['delay_s'], 'actual_s': None,
                                'wall': self._wall().strftime('%H:%M:%S'), 'ok': False, 'error': '已取消'})
        self._pending.clear()
        if cancelled:
            logger.warning('[放行] 取消放行：' + '、'.join(f'UAV-{s}' for s in cancelled) + '（這些飛機不會被送出起飛）')
        self._changed()
        return cancelled

    def remaining(self) -> list:
        """``[(sysid, uav, 剩餘秒數), ...]``（依剩餘時間排序）。"""
        now = float(self._clock())
        t0 = self._t0 if self._t0 is not None else now
        return sorted(((s, p['entry'].get('uav'), max(0.0, p['delay_s'] - (now - t0)))
                       for s, p in self._pending.items()), key=lambda x: (x[2], x[0]))

    def countdown_text(self) -> str:
        lines = []
        for ev in self.events:
            if ev['ok']:
                lines.append(f'UAV-{ev["sysid"]}：已放行（{ev["wall"]}，T+{ev["actual_s"]:.0f} s）')
            elif ev['error'] == '已取消':
                lines.append(f'UAV-{ev["sysid"]}：已取消')
            else:
                lines.append(f'UAV-{ev["sysid"]}：放行失敗（{ev["error"]}）')
        for s, uav, rem in self.remaining():
            m, sec = divmod(int(math.ceil(rem)), 60)
            lines.append(f'UAV-{s}：倒數 {m:d}:{sec:02d} 後放行')
        return '\n'.join(lines)

    def _changed(self) -> None:
        if self._on_change is not None:
            try:
                self._on_change()
            except Exception as e:      # noqa: BLE001
                logger.error(f'[放行] 更新畫面失敗：{e}')


def _stop_link_tlog(link):
    """鏈路斷線時停 tlog（沒在記錄也可以呼叫）；回傳 tlog_status（沒有 tlog 能力的假鏈路 ⇒ None）。"""
    stop = getattr(link, 'stop_tlog', None)
    if stop is None:
        return None
    try:
        st = stop()
    except Exception as e:      # noqa: BLE001 — 斷線清理不可以被 tlog 帶掉
        logger.error(f'[航測] tlog 關檔失敗 sysid={getattr(link, "sysid_label", "?")}: {e}')
        return None
    if isinstance(st, dict) and st.get('error'):
        logger.error(f'[航測] tlog 錯誤 sysid={getattr(link, "sysid_label", "?")}: {st["error"]}')
    return st


# ── SITL 頁「群控／單機」（2026-10-22）：HUD 單機選台 ⇄ Command 分頁機卡列 ──────────────
#   模組層級函式（不是方法）：假 self 的測試只綁固定一串方法名，新增方法會讓它們整串炸掉。
#   ⚠ 不碰群控的指令對象（SITLHud._selected_sysid）：在 Command 分頁點一台只換「單機看哪台」
#     （2026-09-02 教訓：點一下想看它，結果指令對象被換掉）。單機模式下看哪台＝指令對象，HUD 會留一行說明。
def uav_callsign_for_sysid(sysid) -> str:
    """sysid → FleetRegistry 的 callsign（依鏈路的 sysid_label）；找不到 ⇒ 'UAV-<sysid>'。"""
    try:
        from mission.fleet_registry import FleetRegistry
        reg = FleetRegistry.instance()
        for cs in reg.callsigns():
            link = reg.get_link(cs)
            if link is not None and int(getattr(link, 'sysid_label', -1)) == int(sysid):
                return cs
    except Exception:           # noqa: BLE001
        pass
    return f'UAV-{int(sysid)}'


def uav_sysid_for_callsign(callsign):
    """callsign → sysid（FleetRegistry 鏈路的 sysid_label；沒註冊就認 'UAV-<n>'）；認不出 ⇒ None。"""
    try:
        from mission.fleet_registry import FleetRegistry
        link = FleetRegistry.instance().get_link(str(callsign))
        if link is not None:
            return int(getattr(link, 'sysid_label'))
    except Exception:           # noqa: BLE001
        pass
    m = re.match(r'^UAV-(\d+)$', str(callsign or ''))
    return int(m.group(1)) if m else None


def sync_rail_to_hud(win, sysid) -> bool:
    """HUD 單機選台變了 → Command 分頁機卡列跟著選（< 0 ＝ 被清掉，不動 rail）。

    Command 分頁沒顯示時**先記著**、切過去才套用（apply_pending_rail_sync）：rail 換選會讓單機詳情面板
    向那台要全部參數（PARAM_REQUEST_LIST，數傳上要好幾秒頻寬）—— 不該因為在 HUD 頁換看一台就在背景送。
    """
    if sysid is None or int(sysid) < 0 or win.__dict__.get('_uav_sync_busy'):
        return False
    hud = getattr(win, 'sitl_hud', None)
    src = getattr(hud, 'single_change_source', None)
    view = getattr(hud, 'view_mode', None)
    if callable(src) and src() == 'auto' and callable(view) and view() != 'single':
        # 群控時 HUD「第一台有遙測就自動看它」不帶 rail：rail 自己也自動選過了，再帶一次只是讓詳情面板
        # 多換一次台、多要一次參數（2026-10-22 審查）。單機時自動選的那台就是指令對象 ⇒ 照樣對齊。
        return False
    panel = getattr(win, 'sitl_command_panel', None)
    rail = getattr(panel, 'rail', None)
    if rail is None:
        return False
    visible = getattr(panel, 'isVisible', None)
    if callable(visible) and not visible():
        win.__dict__['_uav_sync_pending'] = int(sysid)
        return False
    win.__dict__.pop('_uav_sync_pending', None)
    win.__dict__['_uav_sync_busy'] = True
    try:
        rail.set_current(uav_callsign_for_sysid(sysid))
    finally:
        win.__dict__['_uav_sync_busy'] = False
    return True


def apply_pending_rail_sync(win) -> bool:
    """Command 分頁顯示出來：rail 對齊 HUD **現在**的單機選台。

    對齊的條件：分頁沒顯示時 HUD 有被使用者換過台（記著 _uav_sync_pending），或 HUD 在單機模式（那台就是指令對象，
    Command 分頁的詳情面板也會對它下指令，兩邊一定要同一台）。記著之後 rail 自己自動改選過（別台斷線）也照樣對齊 ——
    以前自動改選會把記著的那筆清掉，Command 分頁就停在另一台（2026-10-22 審查）。HUD 沒選定（斷線停住）⇒ 不動。
    """
    pending = win.__dict__.pop('_uav_sync_pending', None)
    hud = getattr(win, 'sitl_hud', None)
    if hud is None:
        return False
    sid = getattr(hud, 'single_sysid', lambda: None)()
    view = getattr(hud, 'view_mode', None)
    single = callable(view) and view() == 'single'
    if sid is None or (pending is None and not single):
        return False
    rail = getattr(getattr(win, 'sitl_command_panel', None), 'rail', None)
    cur = getattr(rail, 'current_callsign', None)
    if win.__dict__.get('_uav_sync_busy') or (callable(cur) and cur() == uav_callsign_for_sysid(sid)):
        return False
    return _force_rail_sync(win, sid)


def _force_rail_sync(win, sid) -> bool:
    """rail 直接選 sid 那台（分頁已顯示；不看 HUD 的變更來源）。busy 旗標擋住 rail → HUD 的回彈。"""
    rail = getattr(getattr(win, 'sitl_command_panel', None), 'rail', None)
    if rail is None:
        return False
    win.__dict__['_uav_sync_busy'] = True
    try:
        rail.set_current(uav_callsign_for_sysid(sid))
    finally:
        win.__dict__['_uav_sync_busy'] = False
    return True


def sync_hud_to_rail(win, callsign) -> bool:
    """Command 分頁機卡列被使用者點選 → HUD 單機改看那一台。

    rail 自己「自動選」的（第一台註冊、選中那台斷線改選下一台）不跟：單機模式下那等於靜默換掉指令對象。
    """
    if win.__dict__.get('_uav_sync_busy'):
        return False
    rail = getattr(getattr(win, 'sitl_command_panel', None), 'rail', None)
    auto = getattr(rail, 'is_auto_pick', None)
    if callable(auto) and auto():
        return False                                  # ⚠ 先判斷再清 pending：自動改選不能讓記著的那筆作廢
    win.__dict__.pop('_uav_sync_pending', None)      # 使用者在 rail 上選了 ⇒ 之前記著的作廢
    hud = getattr(win, 'sitl_hud', None)
    sid = uav_sysid_for_callsign(callsign)
    if hud is None or sid is None or not hasattr(hud, 'set_single_sysid'):
        return False
    win.__dict__['_uav_sync_busy'] = True
    try:
        return bool(hud.set_single_sysid(sid, source='rail'))
    finally:
        win.__dict__['_uav_sync_busy'] = False


def wire_uav_selection_sync(win) -> None:
    """接 HUD.single_uav_changed ⇄ SitlCommandPanel.current_uav_changed。

    不會無限迴圈，兩個獨立理由：兩端都只在「真的變了」才發信號；_uav_sync_busy 擋住回彈。
    用 lambda 接：假 self（SimpleNamespace）的 bound method 會讓 PyQt 的 weakref 出問題。
    """
    hud = getattr(win, 'sitl_hud', None)
    panel = getattr(win, 'sitl_command_panel', None)
    if hud is None or panel is None or not hasattr(hud, 'single_uav_changed'):
        return
    hud.single_uav_changed.connect(lambda sid, w=win: sync_rail_to_hud(w, sid))
    panel.current_uav_changed.connect(lambda cs, w=win: sync_hud_to_rail(w, cs))
    if hasattr(panel, 'user_picked'):
        # 使用者點了 rail 上**已經選著**的那一格（rail 自動改選之後想把 HUD 單機也對過來）：current_uav_changed
        # 不會發，靠這條。點別格時兩條都會到，第二次 set_single_sysid 沒變 ⇒ no-op。
        panel.user_picked.connect(lambda cs, w=win: sync_hud_to_rail(w, cs))
    if hasattr(panel, 'shown'):
        panel.shown.connect(lambda w=win: apply_pending_rail_sync(w))


def single_mode_sysid(win):
    """HUD 在「單機」模式 ⇒ 選定那台的 sysid（還沒選／選定那台斷線 ⇒ SITLHud.NO_TARGET＝-1）；群控 ⇒ None。

    給「整批」類的動作（上傳任務、依時程放行、下載全部 .bin、寫入每架匯入）判斷範圍：單機 chip 說明寫的是
    「FLIGHT／MISSION／LOG 的按鈕只對這一台」，這些動作以前照樣送給每一台（2026-10-22 審查）。"""
    hud = getattr(win, 'sitl_hud', None)
    view = getattr(hud, 'view_mode', None)
    if not callable(view) or view() != 'single':
        return None
    try:
        sid = hud.get_target_sysid()
    except Exception:           # noqa: BLE001
        return -1
    return -1 if sid is None else int(sid)


def single_mode_refusal(sid) -> str:
    """單機模式但沒有可用對象時的說明（與 _sitl_broadcast 同一句話）。"""
    if sid is None or int(sid) >= 0:
        return ''
    return '單機模式還沒選定飛機（在「單機」區上方選一台），或切回「群控」對全部下指令。'


def hud_uav_message(win, sysid, text: str, severity: int = 4, kind: str = 'gcs') -> bool:
    """對某一台的 HUD 訊息（帶 sysid：群控標 UAVn、單機只在看那台時列出；kind='link' 斷線警示每一台都列）；
    舊 HUD／假 HUD 退回 lbl_status。"""
    hud = getattr(win, 'sitl_hud', None)
    if hud is None:
        return False
    fn = getattr(hud, 'on_uav_message', None)
    if callable(fn):
        fn(int(sysid), str(text), int(severity), kind)
        return True
    lbl = getattr(hud, 'lbl_status', None)
    if lbl is not None:
        lbl.setText(text)
        return True
    return False


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

        # ── HARVEST（異構雙構型 VTOL）狀態 ──
        self._hv_worker = None        # HarvestPlanningWorker（背景規劃執行緒）
        self._hv_coordinator = None   # hv_dccpp 的 HvDccppCoordinator（匯出／上傳／重規劃）
        # 產生 _hv_coordinator 的那個 SwarmCoordinator（讀 validate 結果 hv_validation_reasons 用；R1 #8）
        self._hv_swarm_coordinator = None
        # 高精地圖預計成果（規劃預測）：背景 worker、結果、送地圖的 payload 快取、圖層是否畫過、非模態對話框
        self._hdmap_preview_worker = None
        self._hdmap_preview = None
        self._hdmap_preview_payload = None
        self._hdmap_preview_drawn = False
        self._hdmap_preview_draw_error = None     # 地圖圖層畫不出來的原因（有值 ⇒ 再勾「地圖顯示預計成果」會重畫）
        self._hdmap_preview_error = None          # 上一趟預計成果算不出來的原因（工具選單開對話框時說明用）
        self._hdmap_preview_dialog = None
        # 目前飛控上的 HARVEST 任務的起飛時程（上傳時記；一鍵起飛閘門與「依時程放行」用；R1 #1）
        self._hv_release_schedule = None
        self._release = None                  # StaggeredRelease（進行中的依時程放行）
        self._release_timer_factory = None    # 測試注入：None ⇒ QTimer（PreciseTimer）
        self._release_clock = None            # 測試注入：None ⇒ time.monotonic
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
        # 2026-10-17 使用者定調：蜂群攻擊這一系先從 UI 拿掉、管線先斷開（config/features.STRIKE_ENABLED，
        # 預設關；AEROPLAN_STRIKE_UI=1 暫時開）。關閉時這個儀表板不建、下面的 launch/abort 信號也就沒接。
        self.strike_ttt_dashboard = None
        if _features.strike_enabled():
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
        else:
            logger.info('[TTT Dashboard] 未建立：蜂群攻擊 UI 已停用（config/features.STRIKE_ENABLED=False）')

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
        if _features.strike_enabled():     # 嵌入式蜂群（攻擊那一系）：預設不接、HUD 也不放那顆鈕
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
        # Mission Planner 風格航點檔三兄弟（Load / Write / Save）；
        # Write 是安全關鍵路徑，刻意不重用 on_sitl_upload_mission 的機制。
        self.sitl_hud.cmd_load_wp.connect(self.on_sitl_load_wp_file)
        self.sitl_hud.cmd_write_wp.connect(self.on_sitl_write_wp)
        self.sitl_hud.cmd_save_wp.connect(self.on_sitl_save_wp_file)
        # 多檔匯入（每檔指派一架）→ 逐機寫入；寫入仍走 on_sitl_write_wp 同一套守衛
        self.sitl_hud.cmd_import_wps.connect(self.on_import_waypoints)
        self.sitl_hud.cmd_write_imported.connect(self.on_sitl_write_imported)
        # 飛完後下載每台的 .bin（逐機 / 全部 / 取消）
        self.sitl_hud.cmd_log_download.connect(self._start_log_download)
        self.sitl_hud.cmd_log_download_all.connect(self._start_log_download_all)
        self.sitl_hud.cmd_log_cancel.connect(self._cancel_log_download)
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
        if _features.strike_enabled():     # 投放（攻擊那一系）：預設不接；HUD 的投放列也不進版面
            self.sitl_hud.cmd_release_store.connect(self.on_release_store)
        self.sitl_hud.cmd_vtol_transition.connect(
            lambda state: self._sitl_broadcast('vtol_transition', state)
        )
        # 「依時程放行」／「取消放行」＋逐機倒數：放在 HUD 的「一鍵起飛」正下方（同一個起飛控制區）
        self._install_release_controls()

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
        # DroneDetailPanel 的「↥」鈕（原本沒接東西——不知道要送哪份任務）
        # → 走同一套 Write WPs 守衛，對象換成這個面板選定的單機。
        if hasattr(self.sitl_command_panel, 'detail'):
            self.sitl_command_panel.detail.write_wp_requested.connect(
                self._on_detail_panel_write_wp)
        # 「HUD / Servo」單機選台 ⇄ Command 機卡列互相同步（不碰群控的指令對象；2026-10-22）
        wire_uav_selection_sync(self)

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
        # 起降場 = home_point（與固定翼「起飛點」同一份狀態）；VTOL 偵察規劃前必選
        self.vtol_recon_panel.pick_launch_site_requested.connect(self.on_pick_home_point)
        self.vtol_recon_panel.clear_launch_site_requested.connect(self.on_clear_home_point)
        self.vtol_recon_panel.launch_site_preset_requested.connect(self._on_launch_site_preset)
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
        self.vtol_recon_panel.find_flat_requested.connect(self._on_vtol_recon_find_flat)
        self.vtol_recon_panel.flat_apply_requested.connect(self._on_vtol_recon_apply_flat)
        self.vtol_recon_panel.flat_undo_requested.connect(self._on_vtol_recon_undo_flat)
        # 風場顯示（規劃風／飛機即時風／氣象網格 → 2D+3D 風場圖層）
        self.vtol_recon_panel.wind_display_changed.connect(self._on_wind_display_changed)
        self.vtol_recon_panel.wind_grid_requested.connect(self._on_wind_grid_requested)
        self.vtol_recon_panel.wind_grid_clear_requested.connect(self._on_wind_grid_clear)
        self.vtol_recon_panel.wind_cwa_requested.connect(self._on_wind_cwa_requested)
        self.vtol_recon_panel.wind_profile_view_changed.connect(self._on_wind_profile_view_changed)
        # 航高變了：分層風的 2D 預設層與寫入規劃風的那一層都要依新高度重挑（或標明過時）
        self.vtol_recon_panel.planned_altitude_changed.connect(self._on_planned_altitude_changed)
        # 高精地圖預計成果（規劃預測）：開對話框／地圖圖層開關
        self.vtol_recon_panel.hdmap_preview_requested.connect(self._on_hdmap_preview_open)
        self.vtol_recon_panel.hdmap_preview_layer_toggled.connect(self._on_hdmap_preview_layer_toggled)
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
        # 最近一次從檔案載入的航點檔（LoadedMission；見 mission/waypoint_file.py）。
        # 「寫入飛控」永遠送這一份，不是 self._fc_mission —— 兩者用途相反
        # （一個是要寫進去的、一個是剛讀回來的），混用會把讀回結果當成要寫的東西。
        self._loaded_mission = None
        # 正在等待 mission_uploaded 回報的 sysid 集合：寫入期間該連線的
        # 「寫入飛控」鈕會停用，避免連點造成兩筆寫入互相競爭（見 on_sitl_write_wp）。
        self._write_pending_sysids: set = set()
        # 上次「載入/存成航點檔」對話框停留的目錄；純 UI 便利，不是任何正確性契約。
        self._last_wp_dir: str = ''
        # 「多檔匯入」的結果：{sysid: ImportedMission}。跟 _loaded_mission 分開放——
        # 那份是「HUD 指令對象是誰就寫誰」，這裡每份都綁死一個 sysid（見 on_sitl_write_imported）。
        self._imported_missions: dict = {}
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
        # 2026-10-17：整段只在 config/features.STRIKE_ENABLED 時接（預設關）。參數面板的「蜂群打擊」分頁
        # 同一個開關不建；StrikeControllerMixin／StotRos2ControllerMixin 的方法留著，測試仍直接呼叫得到。
        if _features.strike_enabled():
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
        else:
            logger.info('[功能] 蜂群攻擊 UI 與管線已停用（config/features.STRIKE_ENABLED=False；'
                        'AEROPLAN_STRIKE_UI=1 可暫時打開）')

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
        if _features.strike_enabled():     # 2026-10-17：攻擊那一系先從 UI 拿掉（config/features）
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

        action = file_menu.addAction("匯入航點檔（多檔 → 每架一份）…")
        action.setShortcut(QKeySequence("Ctrl+I"))
        action.setStatusTip("選多個 .waypoints / .plan，每檔指派給一架 UAV 預覽；寫入飛控要另外按 HUD 的「寫入每架匯入的任務」")
        action.triggered.connect(self.on_import_waypoints)

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

        # 航測後處理（飛控日誌＋樹莓派 session → 照片位姿 → ODM → 正射／位姿圖層）
        action = tools_menu.addAction("航測處理…")
        action.setStatusTip("把一個架次的飛控日誌與相機 session 處理成照片位姿與正射圖層")
        action.triggered.connect(self.on_survey_process)

        # 高精地圖預計成果（規劃預測）：建圖規劃完成後在背景算好，這裡跟偵察頁的按鈕開同一個非模態對話框
        action = tools_menu.addAction("預覽高精地圖成果…")
        action.setStatusTip("建圖模式規劃的預計成果：分級圖、可成圖範圍、內部破洞、張數與資料量（規劃預測，非實際成果）")
        action.triggered.connect(self._on_hdmap_preview_open)

        # GCP／檢核點佈設（交叉航帶 vs 加密 GCP 兩方案；作業區＋HARVEST 每架區塊 → 外業表、gcp_list 樣板、地圖點位）
        action = tools_menu.addAction("GCP／檢核點規劃…")
        action.setStatusTip("依作業區（與 HARVEST 規劃的每架區塊）產生 GCP 與檢核點的建議位置，匯出外業表並顯示在地圖上")
        action.triggered.connect(self.on_gcp_plan)

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
        self.statusBar().showMessage(
            "點擊地圖設定起降場／起飛點（VTOL：各機起降坪排在這裡；固定翼：跑道位置）...", 0)
        logger.info("進入起飛點選模式")

    def _sync_launch_site_display(self):
        """VTOL 偵察面板的「起降場」顯示跟著 home_point 走（狀態只有這一份）。"""
        # ⚠ 面板還沒建（__init__ 途中的 on_clear_corners）或測試用 cls.__new__ 建的 MainWindow：
        #   sip 物件在 Qt __init__ 沒跑過時，讀不存在的屬性丟的是 RuntimeError 而不是
        #   AttributeError ⇒ getattr(..., None) 的預設值接不住（tests/test_clear_all_wipes_map.py）。
        try:
            panel = self.vtol_recon_panel
        except (AttributeError, RuntimeError):
            return
        if panel is None:
            return
        if self.home_point is not None:
            panel.set_launch_site(*self.home_point)
        else:
            panel.clear_launch_site()

    def on_home_point_set(self, lat: float, lon: float):
        """設定起飛點並更新 UI"""
        self.home_point = (lat, lon)
        self.picking_home_point = False
        self.parameter_panel.set_home_point_display(lat, lon)
        self._sync_launch_site_display()
        self.map_widget.set_home_point_overlay(lat, lon)
        # 假 self（tests/test_vtol_launch_site.py 的 SimpleNamespace）沒有這個方法 ⇒ 有才呼叫
        _reset = getattr(self, '_reset_sitl_home_preset', None)
        if _reset is not None:
            _reset()
        self.statusBar().showMessage(f"起飛點已設定: ({lat:.6f}, {lon:.6f})", 5000)
        logger.info(f"起飛點設定: ({lat:.6f}, {lon:.6f})")

    def _reset_sitl_home_preset(self) -> None:
        """起降場被（地圖點選／清除）改動 ⇒ SITL 出生點預設改回「自動」（無任務時生在 home_point），
        否則之前套用的場地預設會讓 SITL 生在舊場地、起降場卻在新位置。"""
        try:
            from config.settings import get_settings
            get_settings().sitl_launch.home_preset = ''
        except Exception as e:                               # noqa: BLE001
            logger.warning(f'[起降場] 清除 SITL 出生點預設失敗：{e}')

    def on_clear_home_point(self):
        """清除起飛點"""
        self.home_point = None
        self.picking_home_point = False
        self.parameter_panel.clear_home_point_display()
        self._sync_launch_site_display()
        if hasattr(self.map_widget, 'clear_home_point_overlay'):
            self.map_widget.clear_home_point_overlay()
        # 假 self（tests/test_vtol_launch_site.py 的 SimpleNamespace）沒有這個方法 ⇒ 有才呼叫
        _reset = getattr(self, '_reset_sitl_home_preset', None)
        if _reset is not None:
            _reset()
        self.statusBar().showMessage("起飛點已清除", 3000)
        logger.info("起飛點已清除")

    SITE_PRESET_VIEW_RANGE_M = 1500.0   # 套用預設場地後地圖飛過去的視距

    def _on_launch_site_preset(self, key: str) -> None:
        """面板「預設場地 → 套用為起降場」：設 home_point、地圖飛過去，SITL 出生點預設也同步成同一場地。

        場地清單只有一份（config.spawn_presets.SPAWN_PRESETS）；不認得的 key 不做任何事、大聲說。
        """
        from config.spawn_presets import get_preset_by_key
        p = get_preset_by_key(key)
        if p is None:
            self.statusBar().showMessage(f'沒有這個場地預設：{key!r}（起降場未變）', 8000)
            logger.warning(f'[場地預設] 找不到 key={key!r}，起降場未變')
            return
        self.on_home_point_set(p.lat, p.lon)
        moved = False
        fly = getattr(self.map_widget, 'fly_to_position', None)
        if fly is not None:
            try:
                moved = bool(fly(p.lat, p.lon, 0.0, self.SITE_PRESET_VIEW_RANGE_M))
            except Exception as e:                           # noqa: BLE001 - 地圖飛不過去不該讓起降場設定失敗
                logger.warning(f'[場地預設] 地圖飛到 {p.label} 失敗：{e}')
        # SITL 啟動對話框的「出生地點」預設跟著同一場地（on_home_point_set 剛把它清成「自動」，這裡再設成本場地；
        # 之後用地圖點選／清除起降場會再清回「自動」，SITL 就生在 home_point）。對話框「儲存為預設」勾著時會存進 settings.json。
        try:
            from config.settings import get_settings
            get_settings().sitl_launch.home_preset = p.key
        except Exception as e:                               # noqa: BLE001
            logger.warning(f'[場地預設] 同步 SITL 出生點預設失敗：{e}')
        self.statusBar().showMessage(f'起降場已設為「{p.label}」：{p.lat:.6f}, {p.lon:.6f}；{p.note}'
                                     + ('' if moved else '（地圖未移動）'), 10000)
        logger.info(f'[場地預設] {p.key} → 起降場 ({p.lat:.6f}, {p.lon:.6f}) 海拔 {p.alt_m:.0f} m 航向 {p.heading:.1f}°；'
                    f'來源：{p.source or "未註明"}')

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

    # ══════════════════════════════════════════════════════════════
    #  找平坦位置（World Terrain 掃描 → 平移作業區，可復原）
    # ══════════════════════════════════════════════════════════════
    def _on_vtol_recon_find_flat(self, req: dict) -> None:
        """按下「找平坦位置」：補上作業區與 token，丟到背景執行緒。"""
        from ui.workers.flat_search_worker import FlatSearchWorker
        from utils.cesium_ion import configured_ion_token
        panel = self.vtol_recon_panel
        areas = self.survey_area_polygons()
        if not areas:
            panel.set_flat_busy(False, "請先圈好作業區")
            return
        token, _src = configured_ion_token()
        if not token:
            panel.set_flat_busy(False, "沒有 Cesium ion token（config/settings.json 的 map.cesium_ion_token 或環境變數 CESIUM_ION_TOKEN）")
            return
        old = getattr(self, '_flat_worker', None)
        if old is not None and old.isRunning():
            self.statusBar().showMessage('找平坦位置還在執行，請稍候', 4000)
            return
        if old is not None:
            try:
                old.deleteLater()
            except RuntimeError:
                pass
        full = dict(req)
        full['areas'] = [list(a) for a in areas]
        panel.set_flat_busy(True, "掃描中…（下載地形圖磚，約 10–20 秒）")
        # parent=None：視窗關閉時由 closeEvent 等它跑完（在還在跑的 QThread 上跑解構子是 qFatal）
        self._flat_worker = FlatSearchWorker(token, full, parent=None)
        self._flat_worker.finished_ok.connect(self._on_flat_search_done)
        self._flat_worker.failed.connect(self._on_flat_search_failed)
        self._flat_worker.start()
        logger.info(f"[找平坦位置] 開始：{len(areas)} 塊作業區、半徑 {req.get('radius_km')} km、大小 {req.get('scale', 1.0) * 100:.0f}%")

    def _on_flat_search_done(self, payload: dict) -> None:
        logger.info(f"[找平坦位置] 完成：{len(payload.get('candidates') or [])} 個候選，"
                    f"不用豁免可行 {payload.get('any_fits')}")
        self.vtol_recon_panel.show_flat_result(payload, can_undo=bool(getattr(self, '_flat_undo', None)))

    def _on_flat_search_failed(self, msg: str) -> None:
        self.vtol_recon_panel.set_flat_busy(False, f"⚠ {msg}")
        self.statusBar().showMessage(f'找平坦位置失敗：{msg}', 10000)

    # ══════════════════════════════════════════════════════════════
    #  風場顯示（規劃風／飛機即時風／外部氣象網格 → 2D+3D 風場圖層）
    #
    #  流動層的來源只有一個（優先序）：分層風（10／80／120 m）＞ 氣象網格 ＞ 勾了「規劃風」的均勻場 ＞ 無。
    #  分層風與單層網格互相取代（後抓的那個留下）；分層風在地圖上時，單層流動層由地圖自己藏起來。
    #  箭頭每架一支（飛控 WIND，太舊的不畫）＋ 規劃風在作業區中心一支（sysid 0）。
    #  資料格式與圖例文字在 core.wind_field；頁面端在 ui.widgets.wind_layer。
    # ══════════════════════════════════════════════════════════════
    WIND_LIVE_PUSH_INTERVAL_S = 0.5      # 即時風箭頭最多每 0.5 s 推一次地圖（WIND 約 1–2 Hz × N 架）
    WIND_LIVE_STALE_S = 10.0             # 超過這麼久沒收到 WIND 的機不再畫箭頭
    WIND_PLAN_SYSID = 0                  # 規劃風箭頭用的代號（真機 sysid 從 1 起）
    WIND_GRID_MARGIN_M = 500.0           # 網格／均勻場範圍 = 作業區外擴這麼多
    WIND_CWA_SYSID = -1                  # CWA 測站觀測箭頭用的代號（真機 sysid 從 1 起、規劃風 0）
    WIND_PROFILE_PARTICLES_TOTAL = 2400  # 分層風三層合計的粒子數（每層 = 總數 ÷ 層數，總工作量不變）

    def _wind_state(self) -> dict:
        st = self.__dict__.get('_wind')
        if st is None:
            st = self.__dict__['_wind'] = {'live': {}, 'grid': None, 'grid_text': '', 'cwa': None, 'cwa_text': '',
                                           'plan_text': '', 'plan_applied': None, 'plan_source': '',
                                           'plan_writing': False, 'note_text': '', 'last_push': 0.0,
                                           'profile': None, 'profile_text': '', 'profile_payload': None,
                                           'profile_sent': None,
                                           # 規劃風是分層風依規劃高度挑的那一層寫入的：{profile, column, alt, used_h,
                                           #   eff_text}；改航高時要跟著重挑（或標明已過時）。別的來源寫入／手改 ⇒ None
                                           'plan_profile': None}
        return st

    def _wind_note(self, text: str) -> None:
        """抓取中／失敗／拒絕這類一次性的提示：放在狀態列第一行，不會被下一次即時風推送蓋掉。"""
        self._wind_state()['note_text'] = str(text or '')
        self._refresh_wind_status()

    def _wind_loaded(self) -> bool:
        st = self._wind_state()
        return st.get('grid') is not None or st.get('cwa') is not None or st.get('profile') is not None

    def _wind_display(self) -> dict:
        panel = getattr(self, 'vtol_recon_panel', None)
        get = getattr(panel, 'get_wind_display', None)
        if get is None:
            return {'plan': False, 'live': True, 'speed_mps': 0.0, 'dir_from_deg': 0.0}
        return get()

    def _wind_limit_mps(self) -> float:
        """箭頭／圖例上色門檻 = 面板的「建圖作業風速上限」；讀不到就用首飛預設 3 m/s。"""
        from ui.widgets.wind_layer import WIND_LIMIT_MPS_DEFAULT
        spin = getattr(getattr(self, 'vtol_recon_panel', None), 'hv_map_max_wind_spin', None)
        try:
            v = float(spin.value()) if spin is not None else WIND_LIMIT_MPS_DEFAULT
        except Exception:
            v = WIND_LIMIT_MPS_DEFAULT
        return v if v > 0 else WIND_LIMIT_MPS_DEFAULT

    def _wind_area_bbox(self):
        """作業區外擴 WIND_GRID_MARGIN_M 的 bbox；沒圈作業區回 None。"""
        from core.wind_field import area_bbox
        polys = self.survey_area_polygons()
        return area_bbox(polys, margin_m=self.WIND_GRID_MARGIN_M) if polys else None

    def _on_wind(self, sysid: int, w: dict) -> None:
        """飛控 WIND → 記最新值；依節流推箭頭到地圖。"""
        st = self._wind_state()
        st['live'][int(sysid)] = dict(w)
        now = time.monotonic()
        if now - st['last_push'] >= self.WIND_LIVE_PUSH_INTERVAL_S:
            self._push_wind_arrows(now)

    def _wind_arrow_rows(self, now: float) -> list:
        st = self._wind_state()
        disp = self._wind_display()
        rows = []
        if disp.get('live', True):
            for sid, w in sorted(st['live'].items()):
                if now - float(w.get('t', 0.0)) > self.WIND_LIVE_STALE_S:
                    continue
                rows.append({'sysid': sid, 'lat': w.get('lat'), 'lon': w.get('lon'), 'alt_rel': w.get('alt_rel'),
                             'dir_from_deg': w['dir_from_deg'], 'speed_mps': w['speed_mps'],
                             'label': f'UAV-{sid} 即時風（EKF 估計）'})
        if disp.get('plan'):
            bbox = self._wind_area_bbox()
            if bbox is not None:
                rows.append({'sysid': self.WIND_PLAN_SYSID, 'lat': 0.5 * (bbox[0] + bbox[2]),
                             'lon': 0.5 * (bbox[1] + bbox[3]), 'alt_rel': 0.0,
                             'dir_from_deg': disp['dir_from_deg'], 'speed_mps': disp['speed_mps'],
                             'label': '規劃風（面板假設）'})
        obs = st.get('cwa')
        if obs is not None and obs.has_wind():
            rows.append(obs.to_arrow(self.WIND_CWA_SYSID))
        return rows

    def _push_wind_arrows(self, now: float | None = None) -> None:
        """即時風（每架一支，太舊的不畫）＋ 規劃風（作業區中心一支）→ 地圖；面板狀態列列出最近的即時風。"""
        st = self._wind_state()
        now = time.monotonic() if now is None else now
        st['last_push'] = now
        rows = self._wind_arrow_rows(now)
        fn = getattr(self.map_widget, 'set_wind_arrows', None)
        if fn is not None:
            if rows:
                fn(rows)
            else:
                self.map_widget.clear_wind_arrows()
        live = [f"UAV-{r['sysid']} {r['speed_mps']:.1f} m/s 來自 {r['dir_from_deg']:.0f}°"
                for r in rows if r['sysid'] > 0]
        st['live_text'] = ('即時風（EKF 估計）：' + '；'.join(live)) if live else ''
        self._refresh_wind_status()

    def _refresh_wind_status(self) -> None:
        panel = getattr(self, 'vtol_recon_panel', None)
        set_status = getattr(panel, 'set_wind_status', None)
        if set_status is None:
            return
        st = self._wind_state()
        # scope_text：網格／分層風實際抓的範圍（2026-10-21）；重估：目前 HARVEST 計畫的網格風重估（已作廢會改寫）
        set_status('\n'.join(t for t in (st.get('note_text'), st.get('profile_text'), st.get('grid_text'),
                                         st.get('scope_text'), st.get('cwa_text'), st.get('plan_text'),
                                         grid_wind_estimate_status_text(self), st.get('live_text')) if t))

    def _refresh_wind_field(self) -> None:
        """流動層：有分層風就畫分層風；否則有網格就畫網格；否則勾了規劃風就畫均勻場；否則清掉。

        分層風在地圖上時不碰單層流動層（地圖自己藏起來，清掉分層風時再由這裡重推正確的單層）。"""
        from core.wind_field import uniform_field
        st = self._wind_state()
        if st.get('profile') is not None and getattr(self.map_widget, 'set_wind_profile', None) is not None:
            self._push_wind_profile()
            return
        disp = self._wind_display()
        field = None
        if st['grid'] is not None:
            field = st['grid']
        elif disp.get('plan'):
            bbox = self._wind_area_bbox()
            if bbox is None:
                self.statusBar().showMessage('規劃風要先圈好作業區才畫得出範圍', 5000)
            else:
                field = uniform_field(bbox, disp['speed_mps'], disp['dir_from_deg'])
        fn = getattr(self.map_widget, 'set_wind_field', None)
        if fn is None:
            return
        if field is None:
            self.map_widget.clear_wind_field()
        else:
            fn(field)

    def _apply_wind_options(self) -> None:
        fn = getattr(self.map_widget, 'set_wind_options', None)
        if fn is not None:
            fn(limitMps=self._wind_limit_mps())

    def _on_wind_display_changed(self, _d: dict | None = None) -> None:
        """面板勾選或規劃風值變了：選項、流動層、箭頭全部重推。

        規劃風曾由外部來源寫入、之後使用者手改欄位 ⇒ 狀態列不能再宣稱「已寫入 X 來自 CWA」（有效值契約：
        回報的必須是生效值）；改寫成「已手動改為…（原寫入值已被覆蓋）」。寫入當下 setValue 的同步回呼用 plan_writing 擋掉。
        """
        st = self._wind_state()
        applied = st.get('plan_applied')
        if applied is not None and not st.get('plan_writing'):
            disp = self._wind_display()
            try:
                s, d = float(disp.get('speed_mps', 0.0)), float(disp.get('dir_from_deg', 0.0))
            except (TypeError, ValueError):
                s, d = float('nan'), float('nan')
            if not (abs(s - applied[0]) <= 1e-9 and abs(d - applied[1]) <= 1e-9):
                st['plan_text'] = (f"規劃風已手動改為 {s:.1f} m/s 來自 {d:.0f}°（原由 {st.get('plan_source') or '外部來源'} 寫入的 "
                                   f"{applied[0]:.1f} m/s／{applied[1]:.0f}° 已被覆蓋）")
                st['plan_applied'] = None
                st['plan_profile'] = None            # 手改過：改航高時不再依分層風重挑／重寫
                logger.info(f"[風場] {st['plan_text']}")
        self._apply_wind_options()
        self._refresh_wind_field()
        self._push_wind_arrows()

    def _on_wind_grid_requested(self, req: dict) -> None:
        """按下「抓氣象網格」：範圍 → 背景執行緒抓 Open-Meteo（'profile' ＝ 10／80／120 m 分層）。

        範圍（wind_fetch_scope）：畫面上有 HARVEST 計畫 ⇒ 作業區＋全部航點＋起降場外擴 WIND_GRID_MARGIN_M（網格風重估要
        涵蓋起降轉移）；沒有 ⇒ 作業區外擴 WIND_GRID_MARGIN_M（原狀）。範圍來源寫進狀態列。"""
        from ui.workers.wind_grid_worker import WindGridWorker
        panel = self.vtol_recon_panel
        scope = wind_fetch_scope(self, self.WIND_GRID_MARGIN_M)
        if scope is None:
            panel.set_wind_grid_busy(False)
            self._wind_note(f"請先圈好作業區（網格範圍 = 作業區外擴 {self.WIND_GRID_MARGIN_M:.0f} m）")
            return
        bbox = scope['bbox']
        # 單層網格與分層風共用同一顆按鈕：任一個還在抓就擋
        for _wname in ('_wind_worker', '_wind_profile_worker'):
            _busy = getattr(self, _wname, None)
            if _busy is not None and _busy.isRunning():
                self.statusBar().showMessage('氣象網格還在抓，請稍候', 4000)
                return
        self._wind_state()['fetch_scope'] = scope            # 結果到了再轉成 src_scope／scope_text（_take_fetch_scope）
        height = str((req or {}).get('height') or '10m')
        if height == 'profile':
            self._start_wind_profile_fetch(bbox)
            return
        old = getattr(self, '_wind_worker', None)
        if old is not None:
            try:
                old.deleteLater()
            except RuntimeError:
                pass
        panel.set_wind_grid_busy(True)
        self._wind_note(f"抓取 Open-Meteo {height} 風場中…（要網路，約 1–3 秒）；{scope['text']}")
        # parent=None：視窗關閉時由 closeEvent 等它跑完（在還在跑的 QThread 上跑解構子是 qFatal）
        self._wind_worker = WindGridWorker(bbox, height=height, parent=None)
        self._wind_worker.finished_ok.connect(self._on_wind_grid_done)
        self._wind_worker.failed.connect(self._on_wind_grid_failed)
        self._wind_worker.start()
        logger.info(f"[氣象網格] 開始：{height}，bbox={tuple(round(x, 5) for x in bbox)}；{scope['text']}")

    def _on_wind_grid_done(self, field) -> None:
        st = self._wind_state()
        if st.get('profile') is not None:            # 後抓的單層網格取代分層風（同一個「氣象網格」列，只留一個）
            self._drop_wind_profile()
        st['grid'] = field
        st['grid_text'] = field.label + (('；' + '；'.join(field.notes)) if field.notes else '')
        st['note_text'] = ''
        # 範圍：格點的頭尾就是請求的 bbox（grid_points 含邊界）；對不上 ⇒ 不知道是哪次抓的，不寫範圍
        _ext = ((field.lats[0], field.lons[0], field.lats[-1], field.lons[-1])
                if len(field.lats) > 1 and len(field.lons) > 1 else getattr(field, 'bbox', None))
        scope = _take_fetch_scope(st, _ext)
        st['src_scope'] = scope
        st['scope_text'] = scope['text'] if scope else ''
        logger.info(f"[氣象網格] 完成：{field.label}；最大 {field.max_speed():.1f} m/s")
        self.vtol_recon_panel.set_wind_grid_busy(False)
        self.vtol_recon_panel.set_wind_grid_loaded(True)
        self._refresh_wind_status()
        self._apply_wind_options()
        self._refresh_wind_field()
        # 使用者 2026-10-16：讀到的風也要寫進規劃器 ⇒ 網格的向量平均寫進「平均風速／風向」（面板勾選控制）
        if self._wind_display().get('apply_plan'):
            from core.wind_field import field_mean_wind
            m = field_mean_wind(field)
            if m is None:
                self.statusBar().showMessage('氣象網格沒有有效格點，規劃風未寫入', 8000)
            else:
                # 範圍涵蓋整條航線時，向量平均也是整個範圍的（不只作業區）⇒ 來源照實寫
                _where = '（範圍含整條 HARVEST 航線與起降場，不只作業區）' if (scope or {}).get('kind') == 'plan' else ''
                self._apply_wind_to_plan(m[0], m[1], f"Open-Meteo {field.height_m:.0f} m 網格 {m[2]} 格向量平均"
                                                      f"{_where}，{field.time_utc or '時間未知'} UTC（模式預報，非實測）")
        # 網格風重估（畫面上有 HARVEST 計畫才算；計畫不動）
        refresh_grid_wind_estimate(self, '氣象網格抵達')

    def _on_wind_grid_failed(self, msg: str) -> None:
        self._wind_state().pop('fetch_scope', None)          # 這次抓取結束了（範圍不能留給之後的結果）
        self.vtol_recon_panel.set_wind_grid_busy(False)
        self.vtol_recon_panel.set_wind_grid_loaded(self._wind_loaded())
        self._wind_note(f'⚠ 氣象網格：{msg}')
        self.statusBar().showMessage(f'氣象網格抓取失敗：{msg}', 10000)

    def _on_wind_grid_clear(self) -> None:
        """清掉氣象網格與 CWA 測站觀測（規劃風欄位不改回去：那是使用者看得到、可以自己改的值；狀態列留著來源，
        之後手改欄位會自動改寫成「已手動改為」）。"""
        st = self._wind_state()
        if st.get('profile') is not None or st.get('profile_sent') is not None:
            self._drop_wind_profile()
        st['grid'] = None
        st['grid_text'] = ''
        st['cwa'] = None
        st['cwa_text'] = ''
        st['note_text'] = ''
        st['src_scope'] = None
        st['scope_text'] = ''
        refresh_grid_wind_estimate(self, '清除氣象網格')        # 沒有網格風了 ⇒ 重估清掉（不留舊數字）
        self.vtol_recon_panel.set_wind_grid_loaded(False)
        self._refresh_wind_status()
        self._refresh_wind_field()
        self._push_wind_arrows()

    # ── 分層風（Open-Meteo 同一模式、同一個逐時時間步的 10／80／120 m）────────────────
    #   使用者 2026-10-19：3D 要看不同高度的風，先只做 10／80／120 m。只有這三層：層間不內插、層外不外推；
    #   全是模式預報（非實測），飛控 WIND 即時風仍是另一個、標明是估計的來源（箭頭，不受影響）。
    def _start_wind_profile_fetch(self, bbox) -> None:
        from ui.workers.wind_grid_worker import WindProfileWorker
        panel = self.vtol_recon_panel
        old = getattr(self, '_wind_profile_worker', None)
        if old is not None:
            try:
                old.deleteLater()
            except RuntimeError:
                pass
        panel.set_wind_grid_busy(True)
        _sc = self._wind_state().get('fetch_scope')
        _where = f"；{_sc['text']}" if isinstance(_sc, dict) and _same_bbox(_sc.get('bbox'), bbox) else ''
        self._wind_note(f"抓取 Open-Meteo 分層風（10／80／120 m，同一模式、同一小時）中…（要網路，約 1–3 秒）{_where}")
        # parent=None：視窗關閉時由 closeEvent 等它跑完（在還在跑的 QThread 上跑解構子是 qFatal）
        self._wind_profile_worker = WindProfileWorker(bbox, parent=None)
        self._wind_profile_worker.finished_ok.connect(self._on_wind_profile_done)
        self._wind_profile_worker.failed.connect(self._on_wind_profile_failed)
        self._wind_profile_worker.start()
        logger.info(f"[分層風] 開始：bbox={tuple(round(x, 5) for x in bbox)}")

    def _wind_profile_heights_of(self, profile) -> list:
        """這份分層風**實際有**的高度（遞增）；模式沒給值的層已被 core 丟掉，不在這裡。"""
        try:
            hs = [float(h) for h in profile.heights()]
        except Exception:                                    # noqa: BLE001
            return []
        return sorted(h for h in hs if math.isfinite(h))

    def _wind_planned_alt_m(self):
        """規劃高度＝面板的生效偵察航高（effective_altitude_m）；讀不到回 None（不猜）。"""
        get = getattr(getattr(self, 'vtol_recon_panel', None), 'effective_altitude_m', None)
        try:
            a = float(get()) if get is not None else math.nan
        except Exception:                                    # noqa: BLE001 - GSD 反算可能丟錯
            a = math.nan
        return a if math.isfinite(a) else None

    def _wind_profile_h2d(self, profile, requested=None):
        """(2D 實際畫的層, 要求的高度, 要求從哪來)。

        2D 只畫一層 ＝ 最接近要求高度的**可用**層（等距取較低層；不內插），**只挑一次**。要求從哪來：
          'choice'：呼叫端給的或面板上手動選的那一層（缺層時狀態寫「選的 X m 沒有資料」）；
          'auto'  ：面板還是自動預設（profile_height_2d_user=False）⇒ 直接拿規劃高度對可用層挑（不先對固定的
                    10／80／120 挑、再對可用層挑第二次；缺層時兩次挑選會挑到離航高較遠的層，也不是使用者選的）；
          'other' ：都讀不到 ⇒ 面板下拉的值或最低層。
        舊面板沒有 profile_height_2d_user ⇒ 當成手動選的（舊行為）。"""
        from ui.widgets.vtol_recon_panel import nearest_profile_height, wind_profile_heights
        hs = self._wind_profile_heights_of(profile)
        req, kind = requested, 'choice'
        if req is None:
            disp = self._wind_display()
            if disp.get('profile_height_2d_user', True):
                req = disp.get('profile_height_2d_m')
            else:
                req, kind = self._wind_planned_alt_m(), 'auto'
                if req is None:
                    req, kind = disp.get('profile_height_2d_m'), 'other'
        if req is None:
            req, kind = self._wind_planned_alt_m(), 'auto'
        if req is None and hs:
            req, kind = hs[0], 'other'
        try:
            req = float(req) if req is not None else None
        except (TypeError, ValueError):
            req = None
        if req is not None and not math.isfinite(req):
            req = None
        if kind == 'choice' and (req is None or not any(abs(req - float(h)) < 1e-6 for h in wind_profile_heights())):
            kind = 'other'
        return nearest_profile_height(req, hs), req, kind

    def _wind_profile_centre(self, profile):
        """剖面柱的位置 ＝ 抓取時的**作業區**範圍（作業區外擴 bbox）中心；沒有就用分層風自己的範圍中心，再沒有就用目前作業區。

        2026-10-21：有 HARVEST 計畫時抓取範圍會涵蓋整條航線（wind_fetch_scope），範圍中心不再是作業區中心 ⇒ 先用那次抓取
        記下的作業區範圍（area_bbox；只認 bbox 跟這份分層風相同的那次）。寫入規劃風的「作業區中心」因此仍是作業區中心。"""
        def _boxes():
            st = self._wind_state()
            pb = getattr(profile, 'bbox', None)
            for sc in (st.get('fetch_scope'), st.get('src_scope')):
                if isinstance(sc, dict) and _same_bbox(sc.get('bbox'), pb):
                    yield sc.get('area_bbox')
            yield pb
            try:
                yield self._wind_area_bbox()
            except (TypeError, ValueError):
                yield None
        for bb in _boxes():
            try:
                la0, lo0, la1, lo1 = (float(x) for x in bb)
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(x) for x in (la0, lo0, la1, lo1)):
                return 0.5 * (la0 + la1), 0.5 * (lo0 + lo1)
        return None

    def _wind_profile_payload(self, profile) -> dict:
        """WindProfile → 地圖 set_wind_profile 的 payload（JSON-safe；層用 WindField.to_js()，剖面柱在作業區中心）。"""
        from core.wind_field import profile_column
        centre = self._wind_profile_centre(profile)
        if centre is None:
            raise ValueError('分層風沒有範圍（bbox），不知道剖面柱要放哪')
        h2d, _req, _choice = self._wind_profile_h2d(profile)
        if h2d is None:
            raise ValueError('分層風沒有任何一層')
        return {'layers': [f.to_js() for f in profile.layers],
                'column': {'lat': float(centre[0]), 'lon': float(centre[1]),
                           'levels': [dict(lv) for lv in profile_column(profile, centre[0], centre[1])]},
                'label': str(profile.label),
                'height_2d_m': float(h2d),
                'particles_total': int(self.WIND_PROFILE_PARTICLES_TOTAL),
                'limit_mps': float(self._wind_limit_mps())}

    def _wind_profile_status(self, profile, h2d, req, kind) -> str:
        """狀態列（兩行）：① 模式、有效時間（UTC）、可用層、缺層、2D 畫的是哪一層（與為什麼是這層）；② core 的說明
        （同一時間步、固定模式、不內插不外推、格點吸附與間距、缺層原因、時鐘差）。kind 見 _wind_profile_h2d。"""
        from ui.widgets.vtol_recon_panel import wind_profile_heights
        hs = self._wind_profile_heights_of(profile)
        model = str(getattr(profile, 'model', '') or '模式未知')
        t = str(getattr(profile, 'time_utc', '') or '時間未知')
        parts = [f"分層風：Open-Meteo {model} 預報（模式資料，非實測），{t} UTC",
                 f"可用 {'／'.join(f'{h:g}' for h in hs) or '無'} m（只有這幾層，不內插、不外推）"]
        missing = [float(h) for h in wind_profile_heights() if all(abs(float(h) - x) > 1e-6 for x in hs)]
        if missing:
            parts.append(f"缺 {'／'.join(f'{h:g}' for h in missing)} m（這個模式這小時沒有有效值，已略過）")
        if h2d is not None:
            s = f"2D 顯示 {h2d:g} m"
            if kind == 'choice' and req is not None and abs(float(req) - h2d) > 1e-6:
                s += f"（選的 {float(req):g} m 沒有資料，改畫最接近的一層）"
            elif kind == 'auto' and req is not None:
                why, _w = profile_layer_choice(hs, [{'h_m': h, 'speed_mps': 0.0, 'dir_from_deg': 0.0} for h in hs],
                                               float(req), h2d)
                s += f"（預設：{why}）" if '最接近規劃高度' not in why else f"（預設：最接近規劃高度 {fmt_alt_m(req)} m 的可用層）"
            elif kind == 'other' and req is not None and abs(float(req) - h2d) > 1e-6:
                s += f"（{float(req):g} m 沒有資料，改畫最接近的一層）"
            parts.append(s)
        notes = [str(n) for n in (getattr(profile, 'notes', None) or ()) if str(n)]
        return '；'.join(parts) + (('\n' + '；'.join(notes)) if notes else '')

    def _push_wind_profile(self, requested_h2d=None) -> None:
        """分層風 → 地圖。內容沒變就不重送（勾選／規劃風值每變一次都會走到這裡，重送會讓粒子整個重來）；
        只有 2D 那一層換了 ⇒ set_wind_profile_view。"""
        import json
        st = self._wind_state()
        prof, base = st.get('profile'), st.get('profile_payload')
        fn = getattr(self.map_widget, 'set_wind_profile', None)
        if prof is None or base is None or fn is None:
            return
        try:
            h2d, _req, _choice = self._wind_profile_h2d(prof, requested_h2d)
            payload = dict(base, limit_mps=float(self._wind_limit_mps()), height_2d_m=float(h2d))
            sig = json.dumps({k: v for k, v in payload.items() if k != 'height_2d_m'}, sort_keys=True,
                             allow_nan=False, ensure_ascii=False)
            if sig != st.get('profile_sent'):
                fn(payload)
                st['profile_sent'] = sig
                st['profile_h2d'] = payload['height_2d_m']
            elif payload['height_2d_m'] != st.get('profile_h2d'):
                view = getattr(self.map_widget, 'set_wind_profile_view', None)
                if view is not None:
                    view(height_2d_m=payload['height_2d_m'])
                st['profile_h2d'] = payload['height_2d_m']
        except Exception as e:                               # noqa: BLE001 - slot 裡不能帶例外回 Qt（會 qFatal）
            logger.error(f"[分層風] 送到地圖失敗：{type(e).__name__}: {e}")
            self._wind_note(f"⚠ 分層風畫不出來：{type(e).__name__}: {e}")

    def _on_wind_profile_done(self, profile) -> None:
        """分層風到了：成為流動層來源（取代舊的單層網格）；狀態列寫明模式、有效時間、可用層與缺層。"""
        panel = self.vtol_recon_panel
        try:
            payload = self._wind_profile_payload(profile)
            h2d, req, is_choice = self._wind_profile_h2d(profile)
            text = self._wind_profile_status(profile, h2d, req, is_choice)
        except Exception as e:                               # noqa: BLE001 - slot 裡不能帶例外回 Qt（會 qFatal）
            logger.error(f"[分層風] 資料無法顯示：{type(e).__name__}: {e}")
            self._on_wind_profile_failed(f"資料無法顯示（{type(e).__name__}: {e}）")
            return
        st = self._wind_state()
        st['grid'] = None                            # 同一個「氣象網格」列只留一個：分層風取代單層網格
        st['grid_text'] = ''
        st['profile'] = profile
        st['profile_payload'] = payload
        st['profile_sent'] = None
        st['profile_h2d'] = None
        st['profile_text'] = text
        st['note_text'] = ''
        scope = _take_fetch_scope(st, getattr(profile, 'bbox', None))   # 對不上 ⇒ 不知道是哪次抓的，不寫範圍
        st['src_scope'] = scope
        st['scope_text'] = scope['text'] if scope else ''
        if getattr(self.map_widget, 'set_wind_profile', None) is None:
            st['note_text'] = '⚠ 這個地圖元件不支援分層風顯示（只有狀態文字）'
        logger.info(f"[分層風] 完成：{text}")
        panel.set_wind_grid_busy(False)
        panel.set_wind_grid_loaded(True)
        set_loaded = getattr(panel, 'set_wind_profile_loaded', None)
        if set_loaded is not None:
            set_loaded(True)
        self._refresh_wind_status()
        self._apply_wind_options()
        self._refresh_wind_field()
        if self._wind_display().get('apply_plan'):
            self._apply_profile_wind_to_plan(profile, payload['column'])
        # 網格風重估（畫面上有 HARVEST 計畫才算；計畫不動）
        refresh_grid_wind_estimate(self, '分層風抵達')

    def _on_wind_profile_failed(self, msg: str) -> None:
        self._wind_state().pop('fetch_scope', None)          # 這次抓取結束了（範圍不能留給之後的結果）
        panel = self.vtol_recon_panel
        panel.set_wind_grid_busy(False)
        panel.set_wind_grid_loaded(self._wind_loaded())
        self._wind_note(f'⚠ 分層風：{msg}')
        self.statusBar().showMessage(f'分層風抓取失敗：{msg}', 10000)

    def _drop_wind_profile(self) -> None:
        """把分層風從狀態與地圖拿掉（清除鈕，或後抓的單層網格取代它）；之後由呼叫端重推單層流動層。"""
        st = self._wind_state()
        had = st.get('profile') is not None or st.get('profile_sent') is not None
        for k in ('profile', 'profile_payload', 'profile_sent', 'profile_h2d'):
            st[k] = None
        st['profile_text'] = ''
        fn = getattr(self.map_widget, 'clear_wind_profile', None)
        if had and fn is not None:
            try:
                fn()
            except Exception as e:                           # noqa: BLE001
                logger.error(f"[分層風] 清除地圖失敗：{type(e).__name__}: {e}")
        set_loaded = getattr(self.vtol_recon_panel, 'set_wind_profile_loaded', None)
        if set_loaded is not None:
            set_loaded(False)

    def _on_wind_profile_view_changed(self, d: dict | None = None) -> None:
        """「2D 顯示高度」換了：只換 2D 那一層（set_wind_profile_view），3D 不動、不重送整份分層風。"""
        st = self._wind_state()
        prof = st.get('profile')
        if prof is None:
            return
        req = None
        if (d or {}).get('user', True):                      # 面板自動預設（依航高）換的 ⇒ 不當成「選的」，由規劃高度挑可用層
            try:
                req = float((d or {}).get('height_2d_m'))
            except (TypeError, ValueError):
                req = None
        try:
            h2d, req2, is_choice = self._wind_profile_h2d(prof, req)
            st['profile_text'] = self._wind_profile_status(prof, h2d, req2, is_choice)
        except Exception as e:                               # noqa: BLE001
            logger.error(f"[分層風] 換 2D 顯示高度失敗：{type(e).__name__}: {e}")
            return
        self._refresh_wind_status()
        self._push_wind_profile(req)

    def _apply_profile_wind_to_plan(self, profile, column: dict) -> None:
        """分層風寫入規劃風：規劃器只吃**一個**風 ⇒ 作業區中心、最接近規劃高度的那一層（不內插）。

        說明照實際情況寫（profile_plan_note）：超出最低／最高層 ⇒ ⚠ 不代表規劃高度的風（不外推）；兩層一樣近 ⇒ 依規則
        取較低層（較高層較強時 ⚠）；最接近的層在中心沒有值 ⇒ 寫明改用哪層。寫不進去的原因放狀態列第一行（不只閃 8 秒）。
        記下 plan_profile：之後改航高時依新高度重挑（_on_planned_altitude_changed）。"""
        from core.wind_field import profile_wind_at
        alt = self._wind_planned_alt_m()
        if alt is None:
            self._profile_plan_not_written('讀不到規劃高度（偵察航高）')
            return
        try:
            hit = profile_wind_at(profile, float(column['lat']), float(column['lon']), alt)
        except Exception as e:                               # noqa: BLE001
            logger.error(f"[分層風] 取規劃高度的那一層失敗：{type(e).__name__}: {e}")
            hit = None
        if hit is None:
            self._profile_plan_not_written('作業區中心沒有有效的分層預報風')
            return
        s, d, used_h = (float(x) for x in hit)
        model = str(getattr(profile, 'model', '') or '模式未知')
        t = str(getattr(profile, 'time_utc', '') or '時間未知')
        note = profile_plan_note(self._wind_profile_heights_of(profile), column.get('levels') or [], alt, used_h)
        eff = self._apply_wind_to_plan(s, d, f"Open-Meteo {model} 分層預報 {used_h:g} m（作業區中心），{t} UTC（模式預報，非實測）",
                                       note=note)
        if eff is not None:
            st = self._wind_state()
            st['plan_profile'] = {'profile': profile, 'column': column, 'alt': float(alt), 'used_h': used_h,
                                  'eff_text': eff['text'], 'text': st.get('plan_text')}

    def _profile_plan_not_written(self, reason: str) -> None:
        """分層風沒寫進規劃風：原因放狀態列**第一行**（note_text，留著）＋狀態列 8 秒；說清楚規劃器現在用的是哪個值。"""
        st = self._wind_state()
        src = st.get('plan_source')
        keep = (f"規劃風仍是先前由 {src} 寫入的值" if st.get('plan_applied') is not None and src
                else "規劃風欄位沒有改（仍是面板上的值）")
        msg = f"⚠ 分層風未寫入規劃風：{reason}；{keep}"
        st['plan_profile'] = None                            # 規劃風不是這份分層風寫的：改航高時不跟著它重挑
        logger.warning(f"[分層風] {msg}")
        self.statusBar().showMessage(msg, 8000)
        self._wind_note(msg)

    def _on_planned_altitude_changed(self, _alt=None) -> None:
        """偵察航高（含 GSD 反算）變了：分層風的 2D 預設層與寫入規劃風挑的那一層都是依規劃高度挑的，不能停在舊高度。

        * 2D：「2D 顯示高度」還是自動預設 ⇒ 依新高度重挑**可用**層（只送 view、不重送整份分層風）。
        * 規劃風是分層風寫入、之後沒被手改：分層風還在且勾著「抓到就寫入規劃風」⇒ 依新高度重寫（值與說明都換）；
          否則數值不動，狀態改寫成「規劃高度已改為 X m…；規劃風仍是寫入時選的 Y m 層」（有效值契約：說明不能過時）。"""
        from core.wind_field import profile_wind_at
        st = self._wind_state()
        try:
            prof = st.get('profile')
            if prof is not None and not self._wind_display().get('profile_height_2d_user', True):
                h2d, req, kind = self._wind_profile_h2d(prof)
                st['profile_text'] = self._wind_profile_status(prof, h2d, req, kind)
                self._push_wind_profile()
                self._refresh_wind_status()
            pp = st.get('plan_profile')
            if pp is None or st.get('plan_applied') is None:
                return
            alt = self._wind_planned_alt_m()
            if alt is None:
                st['plan_text'] = (f"{pp['eff_text']}；⚠ 讀不到新的規劃高度：規劃風仍是寫入時（規劃高度 "
                                   f"{fmt_alt_m(pp['alt'])} m）選的 {pp['used_h']:g} m 層的值")
                self._refresh_wind_status()
                return
            if abs(alt - pp['alt']) < 1e-9:                  # 改回寫入時的高度：說明恢復成寫入當下那一句
                if pp.get('text') and st.get('plan_text') != pp['text']:
                    st['plan_text'] = pp['text']
                    self._refresh_wind_status()
                return
            if prof is not None and prof is pp['profile'] and self._wind_display().get('apply_plan'):
                self._apply_profile_wind_to_plan(prof, pp['column'])
                return
            col = pp['column']
            levels = col.get('levels') or []
            hit = profile_wind_at(pp['profile'], float(col['lat']), float(col['lon']), alt)
            why_not = '分層風已清除' if prof is not pp['profile'] else '「抓到就寫入規劃風」沒勾'
            heights = self._wind_profile_heights_of(pp['profile'])
            old = fmt_alt_m(pp['alt'])
            if hit is None:
                tail = f"⚠ 規劃高度已改為 {fmt_alt_m(alt)} m：作業區中心沒有可用的分層"
            elif abs(float(hit[2]) - pp['used_h']) > 1e-6:
                why, warns = profile_layer_choice(heights, levels, alt, float(hit[2]))
                tail = (f"⚠ 規劃高度已改為 {fmt_alt_m(alt)} m，依新高度該用 {float(hit[2]):g} m 層（{why}）"
                        + ''.join('；' + w for w in warns))
            else:
                why, warns = profile_layer_choice(heights, levels, alt, pp['used_h'])
                tail = f"規劃高度已改為 {fmt_alt_m(alt)} m，仍是 {pp['used_h']:g} m 層（{why}）" + ''.join('；' + w for w in warns)
            st['plan_text'] = (f"{pp['eff_text']}；{tail}；規劃風仍是寫入時（規劃高度 {old} m）選的 {pp['used_h']:g} m 層的值，"
                               f"沒有改寫（{why_not}）")
            logger.info(f"[分層風] {st['plan_text']}")
            self._refresh_wind_status()
        except Exception as e:                               # noqa: BLE001 - slot 裡不能帶例外回 Qt（會 qFatal）
            logger.error(f"[分層風] 規劃高度變更處理失敗：{type(e).__name__}: {e}")
            self._wind_note(f"⚠ 規劃高度變更後分層風沒更新：{type(e).__name__}: {e}")

    # ── CWA 自動氣象站觀測（真的量測）→ 測站箭頭 ＋ 寫入規劃風 ──────────────────
    def _wind_near_point(self):
        """找最近測站的基準點：作業區中心，否則起降場；都沒有回 None。"""
        bbox = self._wind_area_bbox()
        if bbox is not None:
            return (0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3]))
        hp = getattr(self, 'home_point', None)
        try:
            if hp is not None and len(hp) >= 2:
                la, lo = float(hp[0]), float(hp[1])
                if math.isfinite(la) and math.isfinite(lo):
                    return (la, lo)
        except (TypeError, ValueError):
            pass
        return None

    def _apply_wind_to_plan(self, speed_mps: float, dir_from_deg: float, source: str, *, note: str = ''):
        """抓到的風 → 面板「平均風速／風向」（規劃器讀的就是那兩格）；狀態列與 log 寫明來源。
        ``note``：接在寫入說明後面的補充（例如分層風用的是哪一層）。"""
        panel = self.vtol_recon_panel
        st = self._wind_state()
        st['plan_writing'] = True            # setValue 會同步觸發 wind_display_changed：那不是使用者手改
        try:
            eff = panel.apply_wind_to_plan(speed_mps, dir_from_deg, source)
        except (ValueError, TypeError) as e:
            logger.error(f'[風場] 規劃風未寫入：{e}')
            self.statusBar().showMessage(f'規劃風未寫入：{e}', 10000)
            return None
        finally:
            st['plan_writing'] = False
        text = eff['text'] + (f"；{note}" if note else '')
        st['plan_text'] = text
        st['plan_applied'] = (float(eff['speed_mps']), float(eff['dir_from_deg']))   # 生效值，之後手改用它比對
        st['plan_source'] = source
        st['plan_profile'] = None            # 分層風寫入的由呼叫端（_apply_profile_wind_to_plan）寫回；其他來源不跟航高
        logger.info(f"[風場] {text}")
        self.statusBar().showMessage(text, 10000)
        self._refresh_wind_status()
        return eff

    def _on_wind_cwa_requested(self, req: dict) -> None:
        """按下「抓 CWA 測站觀測」：授權碼只從 settings.json／環境變數拿，不進 log；背景執行緒抓。"""
        from core.wind_field import short_text
        from ui.workers.wind_grid_worker import CwaStationWorker
        from utils.cwa_opendata import configured_cwa_api_key, looks_like_cwa_api_key
        panel = self.vtol_recon_panel
        station = str((req or {}).get('station') or '').strip()
        # 授權碼貼錯到「測站」欄：不回顯、不進 log（redact 只認 Authorization=，擋不到這條）
        if looks_like_cwa_api_key(station):
            try:
                panel.wind_cwa_station_edit.clear()
            except Exception:
                pass
            panel.set_wind_cwa_busy(False)
            self._wind_note("「CWA 測站」欄填的是授權碼，不是測站（欄位已清空）；授權碼請放 config/settings.json 的 "
                            "weather.cwa_api_key 或環境變數 CWA_API_KEY")
            logger.warning(f"[CWA 測站] 測站欄疑似授權碼（長度 {len(station)}），已拒絕")
            return
        try:
            key, src = configured_cwa_api_key()
        except Exception as e:                               # noqa: BLE001 - slot 裡不能帶例外回 Qt（會 qFatal）
            logger.error(f"[CWA 測站] 讀取授權碼失敗：{type(e).__name__}")
            panel.set_wind_cwa_busy(False)
            self._wind_note(f"讀取 CWA 授權碼失敗：{type(e).__name__}")
            return
        if not key:
            why = f"CWA 授權碼讀取失敗（{src[len('invalid:'):]}）" if src.startswith('invalid:') else "沒有 CWA API 授權碼"
            panel.set_wind_cwa_busy(False)
            self._wind_note(f"{why}：放 config/settings.json 的 weather.cwa_api_key 或環境變數 CWA_API_KEY")
            return
        near = self._wind_near_point()
        if not station and near is None:
            panel.set_wind_cwa_busy(False)
            self._wind_note("請填測站代碼／名稱，或先圈好作業區（才能找最近測站）")
            return
        old = getattr(self, '_cwa_worker', None)
        if old is not None and old.isRunning():
            self.statusBar().showMessage('CWA 測站觀測還在抓，請稍候', 4000)
            return
        if old is not None:
            try:
                old.deleteLater()
            except RuntimeError:
                pass
        panel.set_wind_cwa_busy(True)
        self._wind_note(f"抓 CWA 測站觀測中…（{short_text(station) or '離作業區最近的測站'}；要網路）")
        # parent=None：視窗關閉時由 closeEvent 等它跑完（在還在跑的 QThread 上跑解構子是 qFatal）
        self._cwa_worker = CwaStationWorker(key, station=station, near=near, parent=None)
        self._cwa_worker.finished_ok.connect(self._on_wind_cwa_done)
        self._cwa_worker.failed.connect(self._on_wind_cwa_failed)
        self._cwa_worker.start()
        logger.info(f"[CWA 測站] 開始：{short_text(station) or '最近測站'}；授權碼來源 {src}（長度 {len(key)}）")

    def _on_wind_cwa_done(self, obs) -> None:
        st = self._wind_state()
        st['cwa'] = obs
        st['cwa_text'] = obs.summary()
        st['note_text'] = ''
        logger.info(f"[CWA 測站] {st['cwa_text']}")
        panel = self.vtol_recon_panel
        panel.set_wind_cwa_busy(False)
        panel.set_wind_grid_loaded(True)
        self._refresh_wind_status()
        self._apply_wind_options()
        self._push_wind_arrows()
        if self._wind_display().get('apply_plan'):
            self._apply_wind_to_plan(obs.speed_mps, obs.dir_from_deg,
                                     f"CWA 測站 {obs.title()} 觀測 {obs.obs_time or '時間未知'}，10 分鐘平均地面測站風"
                                     f"（非作業高度）")

    def _on_wind_cwa_failed(self, msg: str) -> None:
        self.vtol_recon_panel.set_wind_cwa_busy(False)
        self.vtol_recon_panel.set_wind_grid_loaded(self._wind_loaded())
        self._wind_note(f'⚠ CWA 測站：{msg}')
        self.statusBar().showMessage(f'CWA 測站觀測抓取失敗：{msg}', 10000)

    def _on_vtol_recon_apply_flat(self, lat: float, lon: float, scale: float) -> None:
        """把所有作業區（含正在畫的角點與起降場）平移到 (lat, lon)，並縮放 ``scale`` 倍。可復原。"""
        from core.terrain.flat_finder import areas_window, transform_point
        areas = self.survey_area_polygons()
        if not areas:
            return
        _h, _w, src = areas_window(areas)
        dst = (float(lat), float(lon))

        def _t(p):
            return transform_point(p, src, dst, scale)

        self._flat_undo = {'survey_areas': [list(a) for a in self.survey_areas],
                           'corners': list(self.corners or []), 'home': self.home_point}
        self.survey_areas = [[_t(p) for p in a] for a in self.survey_areas]
        self.corners = [_t(p) for p in (self.corners or [])]
        self._flat_redraw_areas()
        if self._flat_undo['home'] is not None:
            self.on_home_point_set(*_t(self._flat_undo['home']))
        self.on_clear_paths()        # 舊的規劃結果已不適用（位置變了）
        try:
            self.map_widget.fly_to_position(dst[0], dst[1], 0.0, 4000.0)
        except Exception as e:       # noqa: BLE001 - 2D 模式沒有 fly_to 也不影響平移
            logger.debug(f"[找平坦位置] fly_to 略過：{e}")
        self.vtol_recon_panel.set_flat_can_undo(True)
        self.statusBar().showMessage(
            f'作業區已平移到 ({dst[0]:.4f}, {dst[1]:.4f})' + (f'、縮成 {scale * 100:.0f}%' if scale < 0.999 else '')
            + '；請確認起降場後重新規劃（可在對話框按「復原上次平移」）', 12000)
        logger.info(f"[找平坦位置] 平移：{src} → {dst}，縮放 {scale:.2f}")

    def _on_vtol_recon_undo_flat(self) -> None:
        snap = getattr(self, '_flat_undo', None)
        if not snap:
            return
        self.survey_areas = [list(a) for a in snap['survey_areas']]
        self.corners = list(snap['corners'])
        self._flat_redraw_areas()
        if snap['home'] is not None:
            self.on_home_point_set(*snap['home'])
        else:
            self.on_clear_home_point()
        self.on_clear_paths()
        self._flat_undo = None
        self.vtol_recon_panel.set_flat_can_undo(False)
        self.statusBar().showMessage('已復原平移', 6000)
        logger.info("[找平坦位置] 已復原平移")

    def _flat_redraw_areas(self) -> None:
        """依 self.corners / self.survey_areas 重畫兩張圖與面板計數（平移與復原共用）。"""
        self.map_widget.clear_corners()
        for (la, lo) in self.corners:
            self.map_widget.add_corner(la, lo)         # 不發信號（與手動輸入角點同一條路）
        self._render_committed_areas()
        self.parameter_panel.update_corner_count(len(self.corners))
        self._sync_area_counts()
        self.update_statusbar()

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
            # 在 3D 點的話，Cesium 自己也加了一顆角點標記（DualMapWidget 只把它鏡射進 2D
            # 再由上面 pop 掉）⇒ 3D 那顆不收的話，起降場位置上會留一個作業區角點的紅點。
            _m3 = getattr(self.map_widget, 'map_3d', None)
            try:
                if _m3 is not None and _m3.corners and tuple(_m3.corners[-1]) == (lat, lon):
                    _m3.remove_last_corner()
            except Exception as _e:
                logger.warning(f"[起降場] 移除 3D 暫時角點失敗: {_e}")
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
                # 預計成果也是舊計畫的（含還在背景算的那一趟）
                hdmap_preview_reset(self, f"載具改為 {params['vehicle_type']}")
                refresh_grid_wind_estimate(self, '換載具')       # 網格風重估也是舊計畫的
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
        # 預計成果是那份計畫的 ⇒ 一起清（地圖層、結果、面板；還在算的放棄）
        hdmap_preview_reset(self, '清除路徑')
        refresh_grid_wind_estimate(self, '清除路徑')         # 網格風重估也是那份計畫的
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
        # 同時清除起飛點設定（⚠ VTOL 偵察的「起降場」也是這一份 ⇒ 清區之後要重選）
        self.home_point = None
        self.picking_home_point = False
        if hasattr(self, 'parameter_panel'):
            self.parameter_panel.clear_home_point_display()
        self._sync_launch_site_display()
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
        # 預計成果圖層（on_clear_paths 也會清；這裡明寫一次，免得 on_clear_paths 被換掉時漏清）
        hdmap_preview_reset(self, '清除全部')
        # 真正把兩張圖都清乾淨（含圍籬、VTOL 軌跡、飛控航線等非路徑疊圖）
        if hasattr(self.map_widget, 'clear_all'):
            self.map_widget.clear_all()
        # 多檔匯入的預覽也被上面清掉了 ⇒ 狀態要跟著走，否則「寫入每架匯入的任務」鈕還亮著、
        # 寫的是畫面上已經看不到的東西（審查 2026-10-17）。單檔 _loaded_mission 照舊不動。
        if self.__dict__.get('_imported_missions'):
            n = len(self._imported_missions)
            self._imported_missions = {}
            self._update_write_wp_enabled()
            logger.info(f'[WP] 清除全部：丟掉 {n} 份多檔匯入的指派')
        # 攻擊那一系的隱藏狀態（開關關著時 UI 上沒有任何入口能清它）：清除全部一併歸零
        if self.__dict__.get('_strike_targets') or self.__dict__.get('_strike_launch_base') is not None:
            self._strike_targets = []
            self._strike_launch_base = None
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
        """相機配置（工具 → 相機配置）。

        按對話框的「套用到航測建圖」⇒ 焦距／像素／影像尺寸／前向與側向重疊填進 VTOL 偵察頁的「航測建圖」欄位
        （見 _apply_camera_config_values）。以前這裡只開對話框、結果丟掉，而對話框還說「相機設定已應用」。
        """
        try:
            from ui.dialogs.camera_config import CameraConfigDialog
        except ImportError:
            QMessageBox.information(self, "相機配置", "相機配置功能開發中")
            return
        dialog = CameraConfigDialog(self)
        if not dialog.exec():
            return                      # 關閉／取消：什麼都不套用
        self._apply_camera_config_values(dialog.applied_values())

    def _apply_camera_config_values(self, values: dict):
        """相機配置對話框的值 → 偵察頁「航測建圖」欄位。

        建圖模式還沒開時先問（是 = 打開並套用；否 = 只填欄位；取消 = 不動）。套用後把**欄位讀回**的有效值
        與「沒有套用」的對話框欄位（飛行高度、速度）一起列出來；超出欄位範圍 ⇒ 拒絕並說明（不讓 spinbox 夾值）。
        """
        panel = getattr(self, 'vtol_recon_panel', None)
        if panel is None:
            QMessageBox.warning(self, '相機配置', '找不到 VTOL 偵察頁，無法套用。')
            return
        if panel.mapping_enabled():
            enable = True
        else:
            ret = QMessageBox.question(
                self, '相機配置',
                '要把這台相機套用到 VTOL 偵察頁的「航測建圖」嗎？\n\n'
                '是：打開建圖模式並填入欄位（規劃演算法會切到 HARVEST）\n'
                '否：只填入欄位，不打開建圖模式\n'
                '取消：什麼都不套用',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
                | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel)
            if ret == QMessageBox.StandardButton.Cancel:
                logger.info('[相機配置] 使用者取消，沒有套用任何值')
                return
            enable = ret == QMessageBox.StandardButton.Yes
        try:
            eff = panel.apply_camera_values(values, enable_mapping=enable)
        except ValueError as e:
            logger.error(f'[相機配置] 套用失敗：{e}')
            QMessageBox.critical(self, '相機配置', f'沒有套用：{e}')
            return
        lines = [f'相機：{values.get("camera_name", "?")}', '', '已套用（欄位讀回的值）：']
        lines += [f'  {k}：{v:g}' if isinstance(v, float) else f'  {k}：{v}' for k, v in eff.items()]
        skipped = values.get('not_applied') or {}
        if skipped:
            lines += ['', '沒有套用（偵察頁另有自己的欄位）：']
            lines += [f'  {k}：{v}' for k, v in skipped.items()]
        logger.info(f'[相機配置] 套用到航測建圖：{eff}；未套用 {skipped}')
        QMessageBox.information(self, '相機配置已套用', '\n'.join(lines))

    def on_survey_process(self):
        """工具 → 航測處理…（對話框由 ui.dialogs.survey_process_dialog 提供，延遲匯入）。

        已經有一個（非模態，可能只是被關掉＝隱藏）⇒ 叫回前面、不另建第二個：兩個對話框各跑各的 worker、寫同一個
        輸出資料夾與 odm/submission.json 會互相踩，而且舊的那個的處理結果／ODM 任務狀態會「消失」。
        """
        old = _win_attr(self, '_survey_process_dialog')
        if old is not None and _qt_object_alive(old):
            try:
                old.polygons_source = self.survey_area_polygons()     # 即時來源跟著主視窗（建構時是當下的值）
                if hasattr(old, 'gcp_blocks_source'):
                    old.gcp_blocks_source = gcp_blocks_provider(self)
                    if hasattr(old, '_sync_gcp_dialog_sources'):
                        old._sync_gcp_dialog_sources()
                # 已經顯示中的對話框 show() 不會再發 showEvent ⇒ 測區標籤要自己更新（否則顯示舊的塊數；審查）
                if hasattr(old, '_refresh_polygon_label'):
                    old._refresh_polygon_label()
                old.show()
                old.raise_()
                old.activateWindow()
                return
            except RuntimeError:
                pass                                   # C++ 物件剛好被刪 ⇒ 重建
        try:
            from ui.dialogs.survey_process_dialog import SurveyProcessDialog
        except ImportError as e:
            logger.error(f'[航測] 航測處理對話框載入失敗：{e}')
            QMessageBox.warning(self, '航測處理', f'航測處理對話框無法載入：{e}')
            return
        # R2：「以測區計算視數」要的是**已完成的測區**（完成本區後地圖上的角點會清空）⇒ 把測區多邊形交給對話框
        polygons = self.survey_area_polygons()
        import inspect as _inspect
        _takes_blocks = False
        try:
            _ps = _inspect.signature(SurveyProcessDialog).parameters
            _ok = 'polygons' in _ps or any(p.kind is p.VAR_KEYWORD for p in _ps.values())
            _takes_blocks = 'gcp_blocks' in _ps    # 新版對話框才收：「GCP／檢核點規劃…」按鈕要的 HARVEST 區塊
        except (TypeError, ValueError):
            _ok = True                         # 看不到簽章（C 擴充之類）就照常呼叫（不送新參數）
        if not _ok:
            # 對話框還不收 polygons=（另一個套件正在加）⇒ 說出來，不默默改用地圖角點
            logger.error('[航測] 航測處理對話框不接受 polygons=，測區多邊形交不過去')
            QMessageBox.warning(self, '航測處理',
                                f'航測處理對話框這個版本不接受測區多邊形（polygons=）；目前 {len(polygons)} 個測區'
                                '無法交給它。請更新 ui/dialogs/survey_process_dialog.py。')
            return
        kw = {'gcp_blocks': gcp_blocks_provider(self)} if _takes_blocks else {}
        dlg = SurveyProcessDialog(parent=self, map_widget=self.map_widget, polygons=polygons, **kw)
        self._survey_process_dialog = dlg      # 非模態：留參考，避免被 GC 收掉
        dlg.show()

    def on_gcp_plan(self):
        """工具 → GCP／檢核點規劃…（ui.dialogs.gcp_plan_dialog，延遲匯入；非模態）。

        作業區每次按「產生」時才向 survey_area_polygons 取；HARVEST 區塊由 ``gcp_blocks_provider`` 從
        ``self._hv_coordinator`` 最近一次規劃**唯讀**取出（沒有 HARVEST 規劃 ⇒ 不分區塊，對話框會說明）。
        """
        try:
            from ui.dialogs.gcp_plan_dialog import GcpPlanDialog
        except ImportError as e:
            logger.error(f'[航測] GCP／檢核點規劃對話框載入失敗：{e}')
            QMessageBox.warning(self, 'GCP／檢核點規劃', f'對話框無法載入：{e}')
            return
        dlg = GcpPlanDialog(parent=self, map_widget=self.map_widget, polygons=self.survey_area_polygons,
                            blocks=gcp_blocks_provider(self))
        self._gcp_plan_dialog = dlg            # 非模態：留參考，避免被 GC 收掉
        dlg.show()
    
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
            run_async: HARVEST 分支是否丟到背景執行緒。
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
                    self, "HARVEST",
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
                        '覆蓋規劃：沒有任何 DEM ⇒ HARVEST 會把地形當成海平面 0 m 的'
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

            # ── 預計成果（規劃預測）是上一份計畫的：clear_all_display_layers_js 刻意不清它的圖層，這裡自己清 ──
            hdmap_preview_reset(self, '新的規劃開始')
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

            # ── HARVEST（異構雙構型 VTOL）分支 ──
            #   面板勾選且 vehicle_type == 'vtol' 才接手；其餘一律走既有 DCCPP。
            #   見 docs/AEROPLAN_接入手冊.md §5 與 docs/adr/0007-hv-dccpp-planner.md。
            if self._configure_hv_dccpp(coordinator, params, vehicle_type) and run_async:
                self._hv_on_done = on_done
                self._start_hv_planning(coordinator, plan_kwargs, display_ctx)
                return          # ★ 後處理與 on_done 改由 finished_ok 觸發

            dccpp_result = coordinator.plan_coverage_dccpp(**plan_kwargs)
            # HV 分支同步跑時也要把轉接器留下來（匯出／上傳／重規劃要用）
            self._hv_coordinator = getattr(coordinator, 'hv_coordinator', None)
            self._hv_swarm_coordinator = coordinator      # validate 結果（上傳閘門用）
            self._display_dccpp_result(dccpp_result, **display_ctx)
            # 網格風重估：HARVEST 計畫＋已有網格／分層風才算；DCCPP 計畫 ⇒ 清掉舊計畫的重估
            refresh_grid_wind_estimate(self, '規劃完成')
            if on_done:
                on_done(True)

        except Exception as e:
            logger.error(f"DCCPP 最佳化失敗: {e}", exc_info=True)
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "DCCPP 失敗", f"DCCPP 最佳化路徑規劃失敗：\n{e}")
            if on_done:
                on_done(False)

    def _notify_avoidance_problem(self, text: str) -> None:
        """狀態列(第一行)+ 警告對話框(全文)。"""
        self.statusBar().showMessage("⚠ " + text.splitlines()[0], 20000)
        QMessageBox.warning(self, "避撞未完成", text)

    def _display_dccpp_result(self, dccpp_result: dict, *, params: dict,
                              corners_snapshot: list,
                              centroid_lat: float, centroid_lon: float,
                              altitude: float, speed: float, turn_radius: float,
                              vehicle_type: str, auto_scan: bool):
        """把 DCCPP 規劃結果套用到 UI：避讓後處理、地圖、統計、metrics。

        由 `_on_dccpp_coverage_requested` 直接呼叫（同步路徑），或由
        `HarvestPlanningWorker.finished_ok` 觸發（HARVEST 背景執行緒路徑）。
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
            _collision_conflicts, _collision_actions = run_collision_avoidance(
                params, speed, assembled_paths,
                status=lambda *a: self.statusBar().showMessage(*a),
                notify=lambda t: self._notify_avoidance_problem(t))

            drones_data = []
            total_wps = 0
            total_dist = 0.0

            # ── 優先使用 assembled_paths（含 Dubins 曲線）──
            if assembled_paths:
                for uav_id, apath in assembled_paths.items():
                    # 按 SegmentLabel 切段；⚠ 邊界那條腿要依來源接到前段或後段
                    # （HARVEST 腿歸出發點、舊版 DCCPP 腿歸終點），否則 1 點的段被丟掉
                    # 或腿被歸錯類型（見 mission/path_segments.py）
                    from mission.path_segments import segments_for_path
                    _segs = segments_for_path(apath, altitude)
                    takeoff_pts = _segs['takeoff_paths']
                    entry_pts = _segs['entry_paths']
                    op_pts = _segs['operation_paths']
                    xfer_pts = _segs['transfer_paths']
                    landing_pts = _segs['landing_paths']

                    # 各段分別保留，讓 map_widget 以不同線型/顏色繪製
                    total_wps += len(apath.waypoints)
                    start_pos = (apath.waypoints[0].lat, apath.waypoints[0].lon) if apath.waypoints else (centroid_lat, centroid_lon)
                    # 構型（只有 HARVEST 有）：讓 2D/3D 一眼看得出哪幾架是
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

            # 規劃風（有效值）：HARVEST 計畫讀規劃器生效的風；DCCPP 不考慮風（評估_2026-10-20 §5.1）
            _hv_w = _win_attr(self, '_hv_coordinator')
            _wind_txt = (harvest_wind_text(harvest_plan_wind(_hv_w))
                         if _hv_w is not None and getattr(_hv_w, 'last_result', None) is dccpp_result
                         else '規劃風：DCCPP 不考慮風')
            self.statusBar().showMessage(
                f"DCCPP 最佳化完成 | {vtype_label} × {len(drones_data)} 台 | "
                f"共 {total_wps} 航點 | 總距離 {total_dist:.0f}m | "
                f"Makespan {total_makespan:.1f}s ({total_makespan / 60:.1f}min) | {_wind_txt}",
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
    # HARVEST（異構雙構型 VTOL 覆蓋規劃）
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
            logger.warning(f'[HARVEST] 取不到 fence zone：{e}')
            return []

    def _configure_hv_dccpp(self, coordinator, params: dict, vehicle_type: str) -> bool:
        """決定這一次規劃要不要走 HARVEST，要的話把旗標設到 coordinator 上。

        Returns:
            True 表示已切到 HV 分支（coordinator.use_harvest 已打開）。
        """
        enabled = (str(vehicle_type).lower() == 'vtol'
                   and bool(params.get('hv_dccpp', False)))
        coordinator.use_harvest = enabled
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
                    f'[HARVEST] 台數對不上：Pioneer {n_a} + HERO {n_b} = {n_a + n_b}，'
                    f'但 num_drones={n_total}；以相加值為準')
        # {drone_id: 'A'|'B'} — 前 n_a 台是構型 A（Pioneer + LiDAR），其餘是 B（HERO + 相機）
        self._hv_classes = {i + 1: ('A' if i < n_a else 'B')
                            for i in range(n_a + n_b)}

        coordinator.hv_class_of = self._hv_class_of
        coordinator.harvest_params = dict(params)
        coordinator.hv_home = self.home_point
        coordinator.hv_alternates = getattr(self, '_hv_alternates', None)
        coordinator.hv_wind = params.get('hv_wind')
        # ★ 在 GUI 執行緒先把禁飛區取好再交給規劃器：FenceZoneRegistry 是主執行緒的
        #   QObject 單例，規劃跑在背景執行緒時不該由那邊去讀它。
        coordinator.hv_fence_zones = self._hv_fence_zones()
        logger.info(
            f'[HARVEST] 啟用：構型 A {n_a} 台 / 構型 B {n_total - n_a} 台，'
            f'home={self.home_point}, 風={params.get("hv_wind")}, '
            f'C2={params.get("hv_c2_range_km")} km')
        if not self._hv_classes:
            logger.warning('[HARVEST] num_drones=0，沒有任何飛機可規劃')
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
        """把 HARVEST 規劃丟到背景執行緒（大情境數十秒，同步跑會讓視窗「沒有回應」）。"""
        from ui.workers.planning_worker import HarvestPlanningWorker

        if self._hv_planning_in_flight():
            # 正常情況下 `_on_dccpp_coverage_requested` 的閘門已經先擋掉了；
            # 這裡是防止有人日後從別的地方直接呼叫進來。
            logger.warning('[HARVEST] 已有規劃在執行，忽略這次請求')
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
        # _on_hv_report 是累加的 ⇒ 每次新規劃要從空的開始（否則上一趟的未覆蓋／警示會帶到這一趟）
        self._hv_last_report = {}
        # ★ 必須存成成員：區域變數會被 GC 回收，QThread 直接消失。
        # ★ parent=None（不掛在 MainWindow 底下）：規劃沒跑完就關視窗時，
        #   Qt 會連同子物件一起銷毀，而在「還在跑的 QThread」上呼叫解構子是 qFatal
        #   ⇒ 整個程式 abort 而不是正常關閉。改由 closeEvent 的 _hv_park_running_worker
        #   接手（見該函式）。
        self._hv_worker = HarvestPlanningWorker(coordinator, plan_kwargs, parent=None)
        self._hv_worker.progress.connect(self._on_hv_progress)
        self._hv_worker.report.connect(self._on_hv_report)
        self._hv_worker.failed.connect(self._on_hv_failed)
        self._hv_worker.finished_ok.connect(self._on_hv_finished)
        self._hv_worker.cancelled.connect(self._on_hv_cancelled)
        self.statusBar().showMessage('HARVEST 規劃中…', 0)
        self._hv_worker.start()

    def _on_hv_progress(self, frac: float, msg: str):
        """進度回報。⚠ 0.40 → 0.90 之間沒有中間進度（plan_mission 整段無回呼）。"""
        self.statusBar().showMessage(f'HARVEST {frac * 100:.0f}%：{msg}', 0)

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
        # ⚠⚠⚠ 「畫一大區、系統自動切」這條路唯一已量到的靜默失效：禁航區在
        #   步進軸方向太寬 ⇒ 子區被吃穿 ⇒ 整胞 no_feasible_class，而 status
        #   仍是 GO_WITH_WARNINGS、errors 仍是空的。實測 1560×800 切 4 塊
        #   （門檻 61.4 m）：跨度 70 m ⇒ makespan +60%、100 m ⇒ 只剩 1/4 台。
        'nfz_too_wide_for_auto_split',
        # 多場坪自動指派：採用或被守衛擋下，兩者都要讓操作員看見
        # （靜靜換了降落點跟靜靜沒換一樣糟）。
        'recovery_auto_applied',
        'recovery_auto_rejected',
        # 規劃器沒把某架降回它自己的起飛點（能量吃不下等）⇒ 使用者要看見，不能在 3D 圖上才發現
        'landing_not_at_requested_site',
        # World Terrain：地形來源與授權（個人/非商用）要讓操作員看見；起伏吃掉高度帶時要說出需要多高的 AGL 上限
        'terrain_source',
        'terrain_relief_exceeds_band',
        # 高程基準（A6）：DEM 是正高還是橢球高、大地起伏 N 已知或未知（未知 ⇒ 只准相對高度、MSL 匯出被拒）；
        # N 未知時禁航區海拔上下限被改成整柱（路由會變）；坪高與 DEM 地面差 > 10 m（基準混用 ⇒ 高飛／低飛 ~N）
        'terrain_datum',
        'nfz_vertical_limits_ignored',
        'site_alt_vs_dem',
        # 航測建圖：實際生效的相機／觸發間隔／重疊／引入段（有效值契約），與「這份計畫不能上傳」
        'mapping_setup',
        'mapping_upload_blocked',
        # 交叉航帶鄰機空域（使用者 2026-10-15：多機預設禁止，可明確放行）：被禁止而沒排的組、與放行後伸進鄰機區塊
        # 的面積，兩者都會改變「這份計畫實際飛了什麼」⇒ 要讓操作員看見
        'mapping_cross_forbidden',
        'mapping_cross_airspace',
        # 4D 去衝突收尾重排（vendored 修改 #11）為解殘留衝突改了起飛時刻；cap=relaxed 時工期比原本長
        # （實測 +3.1%）⇒ 要讓操作員看見，不能只躺在 log 檔
        'reschedule_resolved_residual',
        # 上傳那份任務（已縮點）沿線離地高低於 h_clr／固定翼 h_minfw、或取樣點在 DEM 外（core.terrain_clearance；
        # validate 只驗密集航點，縮點的高度剖面容差 10 m）。摘要 terrain_clearance(…) 只寫 log
        'terrain_clearance_failed',
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
            f"[HARVEST] status={payload.get('status')} "
            f"errors={len(payload.get('errors') or [])} "
            f"warnings={len(warnings)} notes={len(notes)} "
            f"total={float(timing.get('total', 0.0)):.0f} ms")
        for w in warnings:
            logger.warning(f'[HARVEST] {w}')
        loud = []
        for n in notes:
            text = str(n)
            if text.startswith(self._HV_LOUD_NOTES):
                loud.append(text)
                logger.warning(f'[HARVEST] {text}')
            else:
                logger.info(f'[HARVEST] note: {text}')
        # 存起來給 _on_hv_finished 在後處理**之後**再顯示一次
        # （後處理最後一定會用「DCCPP 最佳化完成」蓋掉狀態列）
        # ⚠⚠ 同一次規劃會送好幾則（SwarmCoordinator：前置 validation_notes → 轉接層 status/notes/uncovered
        #   → 尾端 validation_notes），所以要**累加**，不能整份覆寫：覆寫的話尾端那則（沒有 uncovered／
        #   status）會把轉接層的未覆蓋吃掉 ⇒「HARVEST 覆蓋不完整」不跳。新規劃開始時由 _start_hv_planning 清空。
        rep = getattr(self, '_hv_last_report', None) or {}
        acc_loud = list(rep.get('loud_notes') or [])
        acc_unc = list(rep.get('uncovered') or [])
        for n in loud:                       # 尾端會原樣轉送轉接層的 terrain_datum(…) 等 ⇒ 去重
            if n not in acc_loud:
                acc_loud.append(n)
        new_unc = []
        for u in (payload.get('uncovered') or []):     # 元素是 list（不可雜湊）⇒ 用 in 比對
            if u not in acc_unc:
                acc_unc.append(u)
                new_unc.append(u)
        self._hv_last_report = {'loud_notes': acc_loud,
                                'uncovered': acc_unc,
                                'status': payload.get('status') or rep.get('status')}
        for u in new_unc:
            logger.error(f'[HARVEST] 未覆蓋：{u}')

    def _hv_show_report_banner(self):
        """把 `_on_hv_report` 收集到的重點顯示出來（要在後處理之後叫，否則會被蓋掉）。"""
        rep = getattr(self, '_hv_last_report', None) or {}
        uncovered, loud = rep.get('uncovered') or [], rep.get('loud_notes') or []
        if uncovered:
            from PyQt6.QtWidgets import QMessageBox
            self.statusBar().showMessage(f'⚠ HARVEST 有未覆蓋區域：{uncovered}', 0)
            QMessageBox.warning(
                self, 'HARVEST 覆蓋不完整',
                '下列區域沒有被完全覆蓋：\n  ' + '\n  '.join(str(u) for u in uncovered)
                + '\n\n（多半要改機隊組成或放寬約束，見接入手冊 §15.2 第 13 條）')
        elif loud:
            self.statusBar().showMessage('HARVEST：' + '；'.join(loud[:3]), 15000)

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
            logger.error(f'[HARVEST] on_done 回呼失敗：{e}', exc_info=True)

    def _on_hv_failed(self, msg: str):
        """⚠⚠ 失敗對話框一定要帶上 `_on_hv_report` 收到的重點。

        上游的 NO_GO 只寫「所有區域皆無可行構型（感測相容／高度帶／鏈路／方向）」,
        四個原因擠在同一句話裡；真正說得出「AGL 上限 120 − 最低作業高度 120 = 0 m」
        的是我們自己補的那些 note。只記進 log 檔的話,使用者在畫面上只會看到那句
        沒有資訊量的話。
        """
        from PyQt6.QtWidgets import QMessageBox
        self.statusBar().clearMessage()
        logger.error(f'[HARVEST] 規劃失敗：{msg}')
        loud = list((getattr(self, '_hv_last_report', None) or {}).get('loud_notes') or [])
        text = msg if not loud else (msg + '\n\n' + '\n'.join(f'• {n}' for n in loud))
        hdmap_preview_reset(self, '規劃失敗')
        QMessageBox.critical(self, 'HARVEST 失敗', text)
        self._hv_notify_done(False)

    def _on_hv_cancelled(self):
        """規劃跑完了但結果已被放棄（呼叫過 stop()，多半是關視窗）。"""
        logger.info('[HARVEST] 規劃結果已放棄（使用者取消）')
        try:
            self.statusBar().clearMessage()
        except RuntimeError:
            pass          # 視窗已經沒了
        try:
            hdmap_preview_reset(self, '規劃取消')
        except RuntimeError:
            pass          # 視窗已經沒了
        self._hv_notify_done(False)

    def _on_hv_finished(self, result: dict):
        """背景規劃完成 → 回到 GUI 執行緒做後處理（與同步路徑同一支）。"""
        worker = getattr(self, '_hv_worker', None)
        coordinator = getattr(worker, 'coordinator', None)
        self._hv_coordinator = getattr(coordinator, 'hv_coordinator', None)
        self._hv_swarm_coordinator = coordinator      # validate 結果（上傳閘門用）
        ctx = getattr(self, '_hv_display_ctx', None) or {}
        self._display_dccpp_result(result, **ctx)
        # 網格風重估（已有網格／分層風才算；計畫不動）：問題要在橫幅之前併進 _hv_last_report 的 loud note
        refresh_grid_wind_estimate(self, '規劃完成')
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

        # 航測建圖：走 hv 匯出器 export_all(camera_cfg)，匯出的 .waypoints／.param 與上傳的逐項相同
        if self._hv_mapping_upload_applies():
            return self._export_hv_mapping(export_dir)

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
                    from ui.widgets.cesium_map_widget import swarm_drone_colors
                    vtol_data = exporter.to_cesium_data(
                        assembled_paths, altitude,
                        colors=swarm_drone_colors(
                            (uid, getattr(ap, 'cls', None))
                            for uid, ap in assembled_paths.items()))
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

    def _export_hv_mapping(self, export_dir: str):
        """建圖模式匯出：hv_dccpp.ardupilot.export_all(..., camera_cfg) ⇒ 每架 .waypoints／.param／.plan
        與 mission_summary.json。項目與上傳的 hv.mission_items 逐項相同（同一支 plan_to_mission_items、同一份
        camera_cfg）；建圖閘門不過時照樣匯出（給人檢查），但要大聲說這份計畫不允許上傳。
        """
        hv = self._hv_coordinator
        cc = getattr(hv, 'camera_cfg', None)
        if cc is None:
            QMessageBox.critical(self, '航測建圖：無法匯出', '這份建圖計畫沒有相機觸發設定（camera_cfg）。請重新規劃。')
            return
        try:
            from hv_dccpp.ardupilot import export_all
            summary = export_all(hv.last_plan, hv.last_ctx, hv.cfg, export_dir, camera_cfg=cc)
        except Exception as e:      # noqa: BLE001
            logger.error(f'[航測] 建圖匯出失敗：{e}', exc_info=True)
            QMessageBox.warning(self, '匯出失敗', f'航測建圖任務匯出失敗：\n{e}')
            return
        lead = float(hv.mapping_mode.lead_in_m)
        lines = [f'目錄：{export_dir}',
                 f'CAM1_INTRVAL_MIN {cc.interval_min_s:.2f} s（規劃拍照率 {1.0 / cc.interval_min_s:.3f} Hz）、'
                 f'CAM1_DURATION {cc.duration_s:g} s、引入段 {lead:g} m、後端 {cc.backend}'
                 f'（韌體 {cc.firmware or "未填"}）', '']
        for uid, u in sorted((summary.get('uavs') or {}).items()):
            lines.append(f'{uid}：{u.get("n_items")} 項 → {Path(u["files"]["waypoints"]).name}、'
                         f'{Path(u["files"]["param"]).name}')
        lines.append(f'摘要：{Path(summary["files"]["summary"]).name}')
        lines += ['', FLOWN_PLAN_EXPORT_NOTE]
        logger.info(f'[航測] 建圖匯出完成：{export_dir}（{len(summary.get("uavs") or {})} 架）；{FLOWN_PLAN_EXPORT_NOTE}')
        gate = hv_upload_gate(hv, getattr(self, '_hv_swarm_coordinator', None))
        if not gate['uploadable']:
            lines += ['', '注意：這份計畫不允許上傳（' + '；'.join(gate['reasons']) + '）；匯出的檔案只供檢查。']
            QMessageBox.warning(self, '航測建圖：已匯出，但不允許上傳', '\n'.join(lines))
        else:
            QMessageBox.information(self, '航測建圖匯出成功（與上傳內容相同）', '\n'.join(lines))
        self.statusBar().showMessage(f'已匯出航測建圖任務至 {export_dir}', 5000)

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
        # 飛控／鏈路訊息帶 sysid 進 HUD（群控標 UAVn、單機只列選定那台）。⚠ 只接帶 sysid 的這條：
        #   舊的 status_text 同一則也會發，兩條都接 HUD 每則就重複兩次（2026-10-22）。
        link.status_text_sysid.connect(self.sitl_hud.on_status_text_sysid)
        link.error.connect(self.sitl_hud.on_error)
        # SERVO 參數（FUNCTION/MIN/TRIM/MAX/REVERSED）→ HUD 依 sysid 快取 → 單機模式的 ServoOutputPanel
        link.servo_param_sysid.connect(self.sitl_hud.on_servo_param_sysid)
        # 任務下載結果 → 畫到地圖。
        # ⚠️ 接在**建立 link 時**，不是接在按鈕的處理函式裡：這樣不論觸發來源
        #    是 HUD 的按鈕、DroneDetailPanel 的 ↧、還是程式自己呼叫
        #    link.download_mission()，結果都會畫出來，而且每條 link 只接一次。
        #    接在處理函式裡會每按一次多接一條 ⇒ 第 N 次點擊畫 N 遍。
        link.mission_downloaded.connect(self._on_mission_downloaded)
        link.mission_download_failed.connect(self._on_mission_download_failed)
        # 任務**寫入**結果 → 狀態列 + 解除「寫入飛控」鈕的停用。
        # ⚠️ 接在這裡（建立 link 時）、只接一次，理由與上面 mission_downloaded
        #    的註解相同：不論觸發來源是 HUD 按鈕、DroneDetailPanel 的 ↥、還是
        #    程式自己呼叫 link.upload_mission_items()，結果都要被聽到；
        #    接在按鈕處理函式裡會每次呼叫多接一條，第 N 次寫入就回報 N 次。
        link.mission_uploaded.connect(self._on_mission_uploaded)
        # 相機回授（CAMERA_FEEDBACK）→ 航測建圖的「已拍 N / 預估 M」。只有建圖上傳過的機才顯示。
        link.camera_feedback.connect(self._on_camera_feedback)
        # 建圖條帶監測（A5，survey.strip_monitor）：任務進度 → 逐條帶 0 張／中斷沒倒帶就接續／跳號警報。
        # 只有建圖上傳過、有監測器的機才處理；同樣在建立 link 時只接一次。
        link.mission_current.connect(self._on_mapping_mission_current)
        link.mission_item_started.connect(self._on_mapping_item_started)
        # 飛控 EKF 風估計（WIND）→ 地圖上跟著飛機走的風箭頭（面板「飛機即時風」勾著才畫）
        link.wind.connect(self._on_wind)
        # 飛控 DataFlash 日誌（.bin）下載：清單 → 挑最新 → 下載；進度／完成／失敗各走自己的信號。
        # 同樣只在建立 link 時接一次（理由同上面 mission_downloaded 的註解）。
        link.log_list.connect(self._on_log_list)
        link.log_list_failed.connect(self._on_log_list_failed)
        link.log_download_progress.connect(self._on_log_download_progress)
        link.log_download_done.connect(self._on_log_download_done)
        link.log_download_failed.connect(self._on_log_download_failed)
        link.start()
        self._sitl_links.append(link)
        if self._sitl_link is None:
            self._sitl_link = link
        self._update_write_wp_enabled()

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
            _stop_link_tlog(link)          # 航測 tlog（沒在記錄就什麼都不做）
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
        # 使用者自己按「全部斷線」：之後重連是新的一輪 ⇒ 單機模式「選定那台斷線、等重選」解除
        # （自然斷線到最後一台時不解除：見 SITLHud.clear_cards）
        _release = getattr(self.sitl_hud, 'release_single_hold', None)
        if callable(_release):
            _release()
        if hasattr(self.map_widget, 'clear_uav'):
            self.map_widget.clear_uav()
        # 全域斷線 ⇒ 任何等待中的寫入都已經不會有回報了（link 已經停止）；
        # 清掉待回報集合並停用「寫入飛控」鈕，否則它會卡在「寫入中」的停用狀態，
        # 而使用者重新連線後那個 sysid 早就不對了。
        self._write_pending_sysids.clear()
        self._update_write_wp_enabled()
        # 飛行計畫快照：還在等 ACK 的每一架都不會有回報了（全部斷線）
        try:
            for _e in list(_win_attr(self, '_flown_plan_pending') or []):
                for _sid in sorted(_e['sysids'] - _e['reported']):
                    abandon_flown_plan_sysid(self, _sid, '全部連線已中斷，寫入結果不明')
        except Exception:           # noqa: BLE001
            pass
        # 日誌下載等待狀態／解鎖紀錄：這條路先把 disconnected 信號切掉了，
        # _on_sitl_link_disconnected 不會跑 ⇒ 要在這裡清，否則重連後該機永遠被拒「還在進行（list）」、
        # 舊的 armed 紀錄會對換上來的另一台機誤判「落地」而自動下載（審查 2026-10-17）。
        _forget_log_state(self, None)
        _forget_tele_state(self, None)      # 已排的 flush 不會再把斷線的飛機畫回來

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
            _stop_link_tlog(link)          # 航測 tlog（沒在記錄就什麼都不做）
            try:
                link.stop()
                link.wait(2000)
            except Exception:
                pass
            try:
                self._sitl_links.remove(link)
            except ValueError:
                pass
            # 同 on_sitl_disconnect：disconnected 已切斷，逐機清理不會跑 ⇒ 這裡清日誌狀態與合幀快取
            _forget_log_state(self, int(getattr(link, 'sysid_label', 1)))
            _forget_tele_state(self, int(getattr(link, 'sysid_label', 1)))
            # ⚠️ finding（major，ui_safety）：這裡先切斷了 disconnected 信號
            # （上面那段），所以 _on_sitl_link_disconnected 不會為這些 link 執行
            # ——它原本負責的「該機若正等待 mission_uploaded 回報，回報永遠不會
            # 來了，要把它從待回報集合裡拿掉」這件事，在 HIL 斷線這條路上要自己
            # 補做一次，否則這個 sysid 會永遠卡在 _write_pending_sysids 裡：
            # _update_write_wp_enabled() 的 enabled 條件是「集合是空的」，卡住
            # 一台會連帶讓所有機的「寫入飛控」鈕都停用；若同一 sysid 之後又用
            # HIL 重連，on_sitl_write_wp() 的併發守衛（見那裡 (2.5)）會誤判成
            # 「已經有一筆寫入在進行中」而永遠拒絕對它寫入。
            self._write_pending_sysids.discard(
                int(getattr(link, 'sysid_label', 1)))
            try:                    # 同理：飛行計畫快照等的這架 ACK 不會來了
                abandon_flown_plan_sysid(self, int(getattr(link, 'sysid_label', 1)), 'HIL 鏈路已斷開，寫入結果不明')
            except Exception:       # noqa: BLE001
                pass
        gone = sorted({int(getattr(l, 'sysid_label', 1)) for l in self._hil_links})
        self._hil_links.clear()
        if self._sitl_link is not None and self._sitl_link not in self._sitl_links:
            self._sitl_link = self._sitl_links[0] if self._sitl_links else None
        # HUD／地圖：同 _on_sitl_link_disconnected 的逐機清理（disconnected 已切斷，那條不會跑）。以前這裡完全沒碰
        # HUD ⇒ 機卡凍在最後一幀、單機模式停在已死的那台（指令對象仍是它）、SERVO 快取留著 —— 同 sysid 換一顆板子
        # 重連只讀回 3 筆，標題卻寫「參數 80/80」、其餘 77 筆是上一顆板子的值（2026-10-22 審查）。
        still = {int(getattr(l, 'sysid_label', -1)) for l in self._sitl_links}
        hud = getattr(self, 'sitl_hud', None)
        for sid in gone:
            if sid in still:        # 同 sysid 還有別條（SITL）連線：機卡是它的，不動
                continue
            try:
                hud.remove_card(sid)
            except Exception:       # noqa: BLE001
                pass
            mw_ = getattr(self, 'map_widget', None)
            if hasattr(mw_, 'clear_uav'):
                try:
                    mw_.clear_uav(sid)
                except Exception:   # noqa: BLE001
                    pass
            n_left = len(self._sitl_links)
            hud_uav_message(self, sid, f'🔌 UAV-{sid} 已斷線（HIL 全部斷開；'
                            + (f'剩餘 {n_left} 台連線）' if n_left else '沒有其他連線）'), 4, kind='link')
        if gone and not self._sitl_links and hud is not None:
            hud.on_disconnected('HIL 全部斷開')
            _release = getattr(hud, 'release_single_hold', None)
            if callable(_release):  # 使用者自己按的「全部斷開」⇒ 之後重連是新的一輪（同 on_sitl_disconnect）
                _release()
        self.hil_panel.on_disconnected()
        self._update_write_wp_enabled()
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
        # 航測 tlog：鏈路結束就關檔（run() 自己也會關；這裡多報一次錯誤狀態）
        _tl = _stop_link_tlog(link)
        if isinstance(_tl, dict) and _tl.get('error'):
            self.statusBar().showMessage(f'UAV-{sysid} tlog 錯誤：{_tl["error"]}', 10000)
        # 該機若正等待 mission_uploaded 回報 → 這條 link 已經停止，回報永遠不會來了
        # （零次違反「恰好一次」的期待，但 link 本身已死，UI 不能繼續卡在「寫入中」）。
        # ⚠️ finding（ui_safety）：只默默拿掉、不額外提示的話，「寫入被連線中斷
        # 打斷」跟「單純斷線」在使用者眼裡長得一模一樣——只有下面那行通用的
        # 斷線狀態列文字，沒有 `_on_mission_uploaded` 失敗分支那種大聲警告。
        # 飛控端這時可能只寫進一半的任務，必須跟那裡用同一套「不要假設剛才
        # 的寫入已經生效」措辭提醒使用者，不能讓這種失敗看起來不像失敗。
        interrupted_write = sysid in self._write_pending_sysids
        self._write_pending_sysids.discard(sysid)
        # 飛行計畫快照：還在等這架建圖寫入結果的批次 ⇒ 結果永遠不會來了，記成未確認（之後這架的 Write WPs ACK
        # 不能被當成建圖任務寫入成功）
        try:
            abandon_flown_plan_sysid(self, sysid, f'連線中斷（{reason}），寫入結果不明')
        except Exception as e:      # noqa: BLE001
            logger.error(f'[航測] 飛行計畫快照：記下 UAV-{sysid} 斷線失敗：{e}', exc_info=True)
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
        # 該機的即時風箭頭也拿掉（不然會留一支不會動的箭頭）；日誌下載等待狀態與解鎖紀錄一併清
        #   （進行中的下載 link 自己會發 log_download_failed('連線結束')）。
        #   ⚠ 直接讀 __dict__：tests/test_mission_write.py 的假 self 沒有 _wind_state 等輔助方法，
        #     用方法呼叫會讓「斷線清理」整段在測試裡炸掉（2026-10-17 抓到）。
        _wind = self.__dict__.get('_wind')
        if _wind and (_wind.get('live') or {}).pop(sysid, None) is not None:
            push = getattr(self, '_push_wind_arrows', None)
            if push is not None:
                push()
        _forget_log_state(self, sysid)
        # 合幀快取：這架最後一幀丟掉（不然 flush 還會再畫一次已斷線的機）
        _forget_tele_state(self, sysid)
        # 維護主鏈路指標（向後相容）
        if self._sitl_link is link:
            self._sitl_link = self._sitl_links[0] if self._sitl_links else None
        # 斷線那行：帶 sysid、kind='link'（全隊警示）⇒ 群控標 UAVn；單機**看哪一台都列**（以前單機只在看那台時才列，
        # 別台在空中斷線畫面上只是一顆 chip 不見了；看的那台斷線連它自己這行都被濾掉；2026-10-22 審查）。
        # 原因（reason）也寫進去。最後一台也寫（之後 on_disconnected 再還原全域連線狀態）。假 HUD 退回 lbl_status。
        n_left = len(self._sitl_links)
        hud_uav_message(self, sysid,
                        f'🔌 UAV-{sysid} 已斷線（{reason or "原因不明"}；'
                        + (f'剩餘 {n_left} 台連線）' if n_left else '沒有其他連線）'), 4, kind='link')
        if not self._sitl_links:
            self.sitl_hud.on_disconnected(reason)
        self._update_write_wp_enabled()
        if interrupted_write:
            logger.error(
                f'[WP] UAV-{sysid} 寫入任務途中斷線（{reason}），無法確定寫入是否完成')
            QMessageBox.critical(
                self, 'Write WPs 被中斷',
                f'UAV-{sysid} 在寫入任務的過程中斷線（原因：{reason}），'
                '這筆寫入**沒有被可靠地完成**。\n\n'
                '飛控上可能只寫進一半的任務。請重新連線後先按「📥 從飛控讀回'
                '任務航線」確認飛控上實際存著的內容，不要假設剛才的寫入已經生效。')

    def _on_sitl_telemetry(self, frame):
        """收到一幀遙測：便宜的狀態觀察立刻做；畫面更新（HUD 重繪、2D/3D 地圖 JS、戰術光錐）合幀。

        2026-10-17 效能：每條鏈路每秒 emit 幾十幀，6 架就是每秒一兩百次「重繪卡片＋兩段 2D JS＋一段 3D JS」。
        現在只記「每架最新一幀」，由 `_flush_telemetry` 以 TELEMETRY_FLUSH_HZ（預設 30 Hz）一次處理全部架：
        地圖一個批次一次 runJavaScript、HUD 機卡另以 HUD_MAX_HZ（15）限頻（離散狀態改變不限）。
        `AEROPLAN_TELEMETRY_HZ=0` ⇒ 不合幀（舊行為，除錯用）。
        """
        # 落地（解鎖→上鎖）⇒ 提示可下載這台的 .bin；勾了自動就排程下載 —— 每幀都看，不能漏掉轉換
        self._note_armed_transition(frame)
        # 建圖條帶監測：模式／解鎖 → 中斷、沒倒帶就接續、落地收尾（只有建圖上傳過的機）
        if getattr(self, '_map_photo_stats', None):
            self._strip_monitor_on_frame(frame)
        # 投放讀回確認：PWM 真的動了才播落點模擬（見 on_release_store）
        if getattr(self, '_pending_releases', None):
            self._confirm_pending_release(frame)
        st = self._tele_state()
        try:
            sysid = int(frame.sysid)
        except (TypeError, ValueError, AttributeError):
            return
        st['latest'][sysid] = frame
        st['dirty'].add(sysid)
        _note_att_sample(st, sysid, frame)      # 每一則姿態都記（合幀只送最新一幀，3D 要每一筆）
        if TELEMETRY_FLUSH_HZ <= 0:
            self._flush_telemetry()
            return
        if not st['pending']:
            st['pending'] = True
            # lambda 而不是綁定方法：PyQt 對綁定方法要拿 weakref，假 self（SimpleNamespace）拿不到會 TypeError。
            # PreciseTimer：Windows 的粗粒度計時器 33 ms 會被拉到 ~48 ms（實測 flush 只有 20 Hz 而不是 30）。
            QTimer.singleShot(int(1000.0 / TELEMETRY_FLUSH_HZ), Qt.TimerType.PreciseTimer,
                              lambda: self._flush_telemetry())

    def _tele_state(self) -> dict:
        """合幀狀態：{'latest': {sysid: frame}, 'dirty': set, 'pending': bool}（__dict__：假 self 也能跑）。"""
        st = self.__dict__.get('_tele')
        if st is None:
            st = self.__dict__['_tele'] = {'latest': {}, 'dirty': set(), 'pending': False}
        return st

    @staticmethod
    def _uav_batch_item(frame) -> dict:
        """TelemetryFrame → 地圖批次的一筆（鍵與 DualMapWidget/MapWidget/CesiumMapWidget.update_uav_positions 一致）。

        hdg：機頭用 ATTITUDE 的 yaw（浮點、與姿態同頻），不是 VFR_HUD 的整數 heading（4 Hz、整數度 ⇒ 機頭一格一格跳、
        轉彎時慢半拍——使用者 2026-10-17：「頭沒有照著航線轉」）。收過 ATTITUDE（att_t > 0）就一定用 yaw——
        **0.0（正北）也是合法值**；沒收過才退回 heading（沒有 att_t 的舊幀：yaw 0 而 heading 非 0 ⇒ 退回 heading）。
        crs：航跡方向（由 GLOBAL_POSITION_INT 的 NED 速度算）。3D 模型機鼻**不用**它（模型 = 飛控姿態，2026-10-22
        使用者：「模型要忠實反映 SITL 的姿態」）——只顯示在狀態列，明確 setUavNoseFollowsTrack(true) 才拿來過渡機鼻；
        2D 圖示在速度 > 2 m/s 時用它轉向。
        """
        yaw = float(getattr(frame, 'yaw', 0.0) or 0.0)
        heading = float(getattr(frame, 'heading', 0.0) or 0.0)
        att_t = float(getattr(frame, 'att_t', 0.0) or 0.0)
        # ⚠ 以前只看「yaw != 0」：真的朝正北（ATTITUDE yaw = 0.0）時會被換成 VFR_HUD／GLOBAL_POSITION_INT 的 heading
        #   （整數度、別的時刻收的）。有 att_t 就知道 ATTITUDE 收過了，yaw 照用。
        have_att = att_t > 0.0 and math.isfinite(att_t)
        hdg = (yaw % 360.0) if (math.isfinite(yaw) and (have_att or yaw != 0.0 or heading == 0.0)) else heading
        vn, ve = float(getattr(frame, 'vel_n', 0.0) or 0.0), float(getattr(frame, 'vel_e', 0.0) or 0.0)
        crs = (math.degrees(math.atan2(ve, vn)) % 360.0) if (vn * vn + ve * ve) > 0.25 else hdg
        item = {'sysid': int(frame.sysid), 'lat': float(frame.lat), 'lon': float(frame.lon),
                'alt': float(frame.alt_rel), 'hdg': hdg, 'crs': crs, 'spd': float(frame.ground_speed),
                'mode': str(frame.mode or ''), 'armed': bool(frame.armed),
                'vt': str(getattr(frame, 'vehicle_type', '') or ''),
                'pitch': float(getattr(frame, 'pitch', 0.0) or 0.0), 'roll': float(getattr(frame, 'roll', 0.0) or 0.0)}
        # pt：鏈路收到這則位置的牆鐘時間（epoch ms）；at：這則姿態的取樣時刻 —— 飛控 time_boot_ms 換到牆鐘的 att_ts
        # （2026-10-22：收到時刻有 0～32 ms 抖動），沒有才退回收到時刻 att_t。3D 用它們當插值樣本時間、同一則重送
        # 不加樣本；沒收過（0）就不帶，地圖端退回自己的現在時刻。
        v = float(getattr(frame, 'pos_t', 0.0) or 0.0)
        if v > 0.0 and math.isfinite(v):
            item['pt'] = int(round(v * 1000.0))
        ams = _att_sample_ms(frame)
        if ams is not None:
            item['at'] = int(round(ams))
        return item

    def _flush_telemetry(self) -> int:
        """把每架最新一幀推到 HUD、地圖（一個批次）、戰術模組。回處理的架數。"""
        st = self._tele_state()
        st['pending'] = False
        dirty = st['dirty']
        st['dirty'] = set()
        # 只畫還連著的機：斷線後鏈路執行緒排隊中的最後幾幀仍會送到這裡，不過濾就是幽靈卡片
        links = self.__dict__.get('_sitl_links')
        if links is not None:
            live = {int(getattr(l, 'sysid_label', 1)) for l in links}
            for dead in [s for s in st['latest'] if s not in live]:
                st['latest'].pop(dead, None)
            dirty = {s for s in dirty if s in live}
        frames = [st['latest'][s] for s in sorted(dirty) if s in st['latest']]
        # 累積的姿態樣本這一輪一定取走（GPS 無效、沒進地圖批次的也丟掉，不然會越積越多）
        att_q = st.get('atts', {})
        for s in [s for s in att_q if s not in st['latest']]:
            att_q.pop(s, None)
        if not frames:
            return 0
        hud = self.sitl_hud
        now = time.monotonic()
        hud_t = st.setdefault('hud_t', {})          # sysid → 上次送進 HUD 的時間
        hud_state = st.setdefault('hud_state', {})  # sysid → (mode, armed) 上次送進 HUD 的離散狀態
        min_dt = (1.0 / HUD_MAX_HZ) if HUD_MAX_HZ > 0 else 0.0
        for frame in frames:
            sysid = int(frame.sysid)
            discrete = (str(getattr(frame, 'mode', '')), bool(getattr(frame, 'armed', False)))
            if min_dt > 0 and discrete == hud_state.get(sysid) and (now - hud_t.get(sysid, -1e9)) < min_dt:
                continue                            # 卡片重繪限頻；離散狀態變了就不限
            hud_t[sysid] = now
            hud_state[sysid] = discrete
            # 所有 sysid 都送進 HUD（多機卡片）
            try:
                hud.on_telemetry(frame)
            except Exception as e:      # noqa: BLE001 — 一張卡壞不能讓其他架也不更新
                logger.debug(f'[遙測] HUD 更新失敗 sysid={sysid}: {e}')
        gps = [f for f in frames if f.is_valid_gps()]
        if gps:
            # ⚠ 地圖或戰術模組丟例外不能讓這一輪 flush 中斷（pending 已經在上面復位，下一幀會再排；
            #   但同一輪裡其他架的戰術更新與這次的地圖批次不能因為一架壞掉就全沒了）。
            mw = self.map_widget
            try:
                if hasattr(mw, 'update_uav_positions'):
                    batch = []
                    for f in gps:
                        item = self._uav_batch_item(f)
                        _take_att_samples(st, int(f.sysid), item)   # 上次送出後、比這一筆舊的姿態 ⇒ item['atts']
                        batch.append(item)
                    mw.update_uav_positions(batch)
                elif hasattr(mw, 'update_uav_position'):
                    for f in gps:
                        mw.update_uav_position(
                            f.lat, f.lon, f.alt_rel, f.heading, f.ground_speed,
                            sysid=f.sysid, mode=f.mode, armed=f.armed,
                            vehicle_type=getattr(f, 'vehicle_type', ''),
                            pitch_deg=getattr(f, 'pitch', 0.0), roll_deg=getattr(f, 'roll', 0.0))
            except Exception as e:      # noqa: BLE001
                logger.error(f'[遙測] 地圖批次更新失敗（{len(gps)} 架）: {e}', exc_info=True)
            # 同步更新戰術模組（FOV 光錐、熱力圖、RCS）—— 沒開時函式本身就是幾個 if
            for f in gps:
                try:
                    self._update_tactical_on_uav_move(
                        f.lat, f.lon, f.alt_rel, f.heading,
                        getattr(f, 'pitch', 0.0), getattr(f, 'roll', 0.0), sysid=f.sysid)
                except Exception as e:  # noqa: BLE001
                    logger.error(f'[遙測] 戰術疊圖更新失敗 sysid={f.sysid}: {e}', exc_info=True)
        for f in frames:                # 沒進批次的姿態樣本（GPS 無效、舊 API 地圖、批次例外）也清掉，不跨批累積
            att_q.pop(int(f.sysid), None)
        # 串流進行中就以固定節奏續排下一次（不等下一幀來才排）：否則週期＝計時器＋等下一幀，
        # 實測 40 Hz 餵幀只拿到 20 次/秒。沒處理到幀（閒置／全斷線）就不續排，下一幀再觸發。
        if TELEMETRY_FLUSH_HZ > 0 and not st['pending']:
            st['pending'] = True
            QTimer.singleShot(int(1000.0 / TELEMETRY_FLUSH_HZ), Qt.TimerType.PreciseTimer,
                              lambda: self._flush_telemetry())
        return len(frames)

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
            msg = '所選的指令對象 UAV 已離線，請改選其他機或「全部」'
            try:
                if self.sitl_hud.get_target_sysid() == -1:      # SITLHud.NO_TARGET
                    msg = '單機模式還沒選定飛機（在「單機」區上方選一台），或切回「群控」對全部下指令。'
            except Exception:       # noqa: BLE001
                pass
            QMessageBox.information(self, 'SITL', msg)
            return
        # R1 #1：飛控上是起飛時程不同的 HARVEST 任務 ⇒ 不准一起起飛（4D 去衝突靠起飛時間錯開）
        if cmd in ('auto_start', 'mission_start'):
            block = staggered_start_block(getattr(self, '_hv_release_schedule', None), links)
            if block:
                logger.error(f'[起飛時程] 擋下一鍵起飛：{block}')
                QMessageBox.critical(self, '一鍵起飛：已拒絕（起飛時程不同）', block)
                return
        for link in links:
            try:
                send_link_command(link, cmd, arg)
            except Exception as e:
                logger.error(f'[SITL] {cmd} 失敗: {e}')

    # ─────────────────────────────────────────────────────────────────
    # HARVEST 起飛時程：記錄（上傳時）→ 擋一鍵起飛 → 依時程放行（R1 #1）
    #   記錄／清除／判斷都是模組層級函式（假 self 的測試也走得到），這裡只是薄包裝
    # ─────────────────────────────────────────────────────────────────
    def _record_release_schedule(self, hv, pairs, *, mapping: bool):
        return record_release_schedule(self, hv, pairs, mapping=mapping)

    def _forget_release_schedule(self, sysids=None, reason: str = ''):
        forget_release_schedule(self, sysids, reason)

    @staticmethod
    def _schedule_spread_s(entries) -> float:
        return schedule_spread_s(entries)

    @staticmethod
    def _release_schedule_lines(entries) -> list:
        return release_schedule_lines(entries)

    def _staggered_start_block(self, links):
        return staggered_start_block(getattr(self, '_hv_release_schedule', None), links)

    def _install_release_controls(self):
        """在 HUD「一鍵起飛」下面插入「依時程放行」「取消放行」與逐機倒數（sitl_hud.py 不動，從外面插）。"""
        from PyQt6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
        box = QWidget()
        v = QVBoxLayout(box)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(2)
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        self.btn_release_schedule = QPushButton('依時程放行')
        self.btn_release_schedule.setToolTip(
            'HARVEST 任務的起飛時程不同時用（一鍵起飛會被擋）：時程 T+0 的立刻 AUTO+ARM+START，\n'
            '其他的倒數到各自的時程再送同一個指令。已解鎖、AUTO 中、沒連線或任務寫入未確認的飛機不放行。')
        self.btn_release_schedule.clicked.connect(self.on_release_by_schedule)
        self.btn_release_cancel = QPushButton('取消放行')
        self.btn_release_cancel.setToolTip('停掉還沒到時間的放行（已經放行的飛機不受影響）。')
        self.btn_release_cancel.clicked.connect(self.on_cancel_release)
        self.btn_release_cancel.setEnabled(False)
        row.addWidget(self.btn_release_schedule, 1)
        row.addWidget(self.btn_release_cancel, 1)
        v.addLayout(row)
        self._release_label = QLabel('')
        self._release_label.setWordWrap(True)
        v.addWidget(self._release_label)
        self._release_widget = box
        try:
            btn = self.sitl_hud.btn_auto_start
            lay = btn.parentWidget().layout()
            lay.insertWidget(lay.indexOf(btn) + 1, box)
        except Exception as e:      # noqa: BLE001 — 插不進 HUD 就要說，不然按鈕不見了沒人知道
            logger.error(f'[放行] 「依時程放行」沒辦法放進 HUD（{type(e).__name__}: {e}）')

    def _release_start_one(self, entry):
        """放行一架（計時器到時間或 T+0 立刻）：放行**當下**重新確認還連著、沒解鎖、不在 AUTO、任務寫入沒失敗，
        再送與一鍵起飛同一支的 auto_start。不合 ⇒ 丟例外（StaggeredRelease 記為放行失敗並寫 log）。"""
        sysid = int(entry['sysid'])
        link = next((l for l in self._sitl_links if int(getattr(l, 'sysid_label', -1)) == sysid), None)
        if link is None:
            raise RuntimeError(f'UAV-{sysid} 已斷線')
        sched = getattr(self, '_hv_release_schedule', None) or {}
        if sysid not in (sched.get('entries') or {}):
            raise RuntimeError(f'UAV-{sysid} 的任務已被換掉，不再屬於這份起飛時程')
        if (sched.get('upload_ok') or {}).get(sysid) is False:
            raise RuntimeError(f'UAV-{sysid} 的任務寫入失敗過')
        prob = link_flight_state_problem(link)
        if prob:
            raise RuntimeError(prob)
        send_link_command(link, 'auto_start')

    def on_release_by_schedule(self):
        """「依時程放行」：時程 T+0 的立刻送 auto_start，其他的用計時器在各自的時程送；倒數顯示在 HUD。"""
        _single = single_mode_sysid(self)
        if _single is not None:
            # 整批起飛（各機依時程相對起飛）：單機模式只對一台下指令，不送（2026-10-22 審查：以前單機也照樣放行全部）
            QMessageBox.information(
                self, '依時程放行：單機模式不送',
                '「依時程放行」會讓時程裡的**每一架**依各自的時程起飛，是整批的動作；'
                + (f'單機模式只對 UAV{_single} 下指令。' if _single >= 0 else '單機模式只對選定的一台下指令。')
                + '\n要放行請切回「群控」。')
            return
        sched = getattr(self, '_hv_release_schedule', None)
        if not sched or not sched.get('entries'):
            QMessageBox.information(self, '依時程放行',
                                    '飛控上沒有已上傳的 HARVEST 任務（先規劃 HARVEST、按「上傳飛控」）。')
            return
        rel = getattr(self, '_release', None)
        if rel is not None and rel.active:
            QMessageBox.information(self, '依時程放行', '上一批放行還在倒數；要重來請先按「取消放行」。')
            return
        entries = sorted(sched['entries'].values(), key=lambda e: (e['offset_s'], e['sysid']))
        armed, other = [], []
        for e in entries:
            sysid = e['sysid']
            link = next((l for l in self._sitl_links if int(getattr(l, 'sysid_label', -1)) == sysid), None)
            if link is None:
                other.append((e, f'UAV-{sysid} 沒有連線'))
                continue
            if sysid in getattr(self, '_write_pending_sysids', set()):
                other.append((e, f'UAV-{sysid} 任務寫入還沒確認'))
                continue
            if (sched.get('upload_ok') or {}).get(sysid) is False:
                other.append((e, f'UAV-{sysid} 任務寫入失敗'))
                continue
            code, prob = link_flight_state(link)
            if code is not None:
                # 已解鎖／AUTO／狀態不明：可能已經在飛 ⇒ 其他機相對它的時間不明，整批不放行
                armed.append((e, prob))
        if armed:
            text = ('下列飛機已解鎖、在 AUTO 或狀態不明（可能已經在飛），其他飛機相對它的起飛時間無法確定 ⇒ '
                    '整批不放行：\n  ' + '\n  '.join(p for _e, p in armed))
            logger.error(f'[放行] 拒絕：{text}')
            QMessageBox.critical(self, '依時程放行：已拒絕', text)
            return
        ok_entries = [e for e in entries if all(e is not x for x, _p in other)]
        if not ok_entries:
            text = '沒有可以放行的飛機：\n  ' + '\n  '.join(p for _e, p in other)
            logger.error(f'[放行] 拒絕：{text}')
            QMessageBox.critical(self, '依時程放行：已拒絕', text)
            return
        base = min(e['offset_s'] for e in ok_entries)
        lines = [f'UAV-{e["sysid"]}（機 {e["uav"]} {e.get("callsign", "?")}）：'
                 + ('立刻' if e['offset_s'] - base <= TAKEOFF_STAGGER_TOL_S else f'{e["offset_s"] - base:.0f} s 後')
                 + f'（計畫 T+{e["offset_s"]:.0f} s）' for e in ok_entries]
        text = '依起飛時程送出 AUTO + ARM + MISSION_START：\n  ' + '\n  '.join(lines)
        if other:
            text += ('\n\n下列飛機**不放行**（不會起飛；其他飛機的相對時程不變）：\n  '
                     + '\n  '.join(p for _e, p in other))
        text += '\n\n倒數中的飛機可按「取消放行」停掉。確定開始？'
        ret = QMessageBox.question(self, '依時程放行', text,
                                   QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                                   QMessageBox.StandardButton.Cancel)
        if ret != QMessageBox.StandardButton.Yes:
            logger.info('[放行] 使用者取消依時程放行')
            return
        if other:
            logger.warning('[放行] 不放行：' + '；'.join(p for _e, p in other))
        self._release_failures_shown = 0          # 新的一批：失敗訊息從頭算
        self._release = StaggeredRelease(self._release_start_one,
                                         timer_factory=getattr(self, '_release_timer_factory', None),
                                         clock=getattr(self, '_release_clock', None),
                                         on_change=self._on_release_changed)
        self._release.start(ok_entries)
        self._ensure_release_ticker()

    def on_cancel_release(self):
        rel = getattr(self, '_release', None)
        if rel is None or not rel.active:
            return
        cancelled = rel.cancel()
        self.statusBar().showMessage(f'已取消放行：UAV-{cancelled}（這些飛機不會被送出起飛）', 10000)

    def _on_release_changed(self):
        """放行狀態變了（放出一架／失敗／取消）：更新倒數；失敗要大聲。"""
        rel = getattr(self, '_release', None)
        self._refresh_release_countdown()
        if rel is None:
            return
        shown = getattr(self, '_release_failures_shown', 0)
        fails = [ev for ev in rel.events if not ev['ok'] and ev['error'] != '已取消']
        if len(fails) > shown:
            self._release_failures_shown = len(fails)
            new = fails[shown:]
            QMessageBox.warning(self, '依時程放行：有飛機沒有放行',
                                '\n'.join(f'UAV-{ev["sysid"]}：{ev["error"]}' for ev in new)
                                + '\n\n其他飛機照時程繼續；要全部停下請按「取消放行」。')

    def _ensure_release_ticker(self):
        """每秒更新倒數（只在有飛機倒數時跑）。"""
        from PyQt6.QtCore import QTimer
        t = getattr(self, '_release_ticker', None)
        if t is None:
            t = QTimer(self)
            t.setInterval(1000)
            t.timeout.connect(self._refresh_release_countdown)
            self._release_ticker = t
        if not t.isActive():
            t.start()

    def _refresh_release_countdown(self):
        rel = getattr(self, '_release', None)
        sched = getattr(self, '_hv_release_schedule', None)
        if rel is not None and (rel.active or rel.events):
            text = rel.countdown_text()
        elif sched and sched.get('entries'):
            spread = schedule_spread_s(sched['entries'].values())
            text = ('起飛時程：' + '、'.join(release_schedule_lines(sched['entries'].values()))
                    + ('（不同 ⇒ 用「依時程放行」）' if spread > TAKEOFF_STAGGER_TOL_S else ''))
        else:
            text = ''
        self._release_countdown_text = text
        lbl = getattr(self, '_release_label', None)
        if lbl is not None:
            try:
                lbl.setText(text)
            except RuntimeError:
                pass
        btn = getattr(self, 'btn_release_cancel', None)
        if btn is not None:
            try:
                btn.setEnabled(bool(rel is not None and rel.active))
            except RuntimeError:
                pass
        t = getattr(self, '_release_ticker', None)
        if t is not None and (rel is None or not rel.active):
            try:
                t.stop()
            except RuntimeError:
                pass

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
        if int(sid) < 0:                      # 單機模式還沒選飛機（SITLHud.NO_TARGET）：後面解析不到對象會中止
            return True
        if len(self._sitl_links) <= 1:        # 本來就只有一台，沒有歧義
            return True
        # 「單機」模式是使用者明確選的（FLIGHT 標頭寫著「UAVn（單機模式）」）：這個對話框是給 09-02「點機卡只是想看」
        # 的誤觸情境用的，單機模式再問一次、而且預設鈕（Enter）是「改成全部 UAV」⇒ 一按 Enter 就把刻意的單機起飛
        # 變成整群分階段起飛、還靜默切回群控（2026-10-22 審查）。單機模式直接照單機送。
        _view = getattr(self.sitl_hud, 'view_mode', None)
        if callable(_view) and _view() == 'single':
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
        # ⚠️ 地圖自 2026-10-17 起每架一層（讀回 UAV-2 不會蓋掉 UAV-1），但「💾 存成航點檔」
        #    只保留**最後一份**讀回結果（self._fc_mission）；多架一起讀回會讓「存的是哪一架」
        #    變成靠回報順序決定 ⇒ 仍然要求逐台讀回。
        if len(links) > 1:
            QMessageBox.information(
                self, 'SITL',
                f'目前指令對象是「全部」（{len(links)} 架）。\n'
                '地圖上每架各畫一層，但「存成航點檔」只保留最後一份讀回結果，\n'
                '多架一起讀回會讓存的是哪一架變成靠回報順序決定。\n\n'
                '請先在 HUD 上點選單一 UAV 卡片，逐台讀回。')
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

            # 每架一層（sysid=…）：讀回 UAV-2 只換 UAV-2 那層，UAV-1 的讀回／匯入預覽留著。
            # ⚠ 不帶 sysid 的舊呼叫法會先清掉**全部**層（2026-10-17 2D/3D 改多層後的語意）。
            if hasattr(self.map_widget, 'display_fc_mission'):
                self.map_widget.display_fc_mission(path2d, path3d, loiters,
                                                   home=home, sysid=int(sysid),
                                                   label=f'UAV-{int(sysid)} 讀回')
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

    # ══════════════════════════════════════════════════════════════
    #  飛控 DataFlash 日誌（.bin）下載：每台飛完後把那一趟的日誌抓下來
    #
    #  鏈路層（SITLLink.request_log_list / download_log / cancel_log_download）是非阻塞狀態機，
    #  這裡只做：找 link → 擋「還在解鎖」→ 要清單 → 挑最新一份 → 給路徑 → 開始；
    #  進度丟給 HUD 卡片；完成／失敗寫狀態列與 HUD，並記住每台的檔案路徑（航測處理可直接帶入）。
    #  ⚠ 飛控已解鎖時一律拒絕（ArduPilot 自己也拒絕：'Disarm for log download'）。
    #  ⚠ 數傳（57600）很慢（實測約 8 KiB/s、數十 MB 要數十分鐘到數小時），所以預設不自動下載，
    #    落地只提示；HUD 勾「落地後自動下載」才會在上鎖 LOG_DOWNLOAD_AUTO_DELAY_S 秒後自動開始。
    # ══════════════════════════════════════════════════════════════
    LOG_DOWNLOAD_AUTO_DELAY_S = 8.0      # 落地上鎖後等這麼久再抓（讓飛控把日誌關檔、寫完尾巴）

    def _log_dl_state(self) -> dict:
        """sysid → {'stage': 'list'|'download', 'id', 'size', 'path', 'started'}"""
        st = self.__dict__.get('_log_dl')
        if st is None:
            st = self.__dict__['_log_dl'] = {}
        return st

    def _flight_logs(self) -> dict:
        """sysid → 最近一次下載完成的 .bin 路徑（字串）"""
        st = self.__dict__.get('_flight_log_paths')
        if st is None:
            st = self.__dict__['_flight_log_paths'] = {}
        return st

    def _armed_seen(self) -> dict:
        st = self.__dict__.get('_armed_state')
        if st is None:
            st = self.__dict__['_armed_state'] = {}
        return st

    def _link_for_sysid(self, sysid: int):
        for l in list(getattr(self, '_sitl_links', None) or []):
            if int(getattr(l, 'sysid_label', -1)) == int(sysid):
                return l
        return None

    def _hud_log_msg(self, sysid: int, text: str, severity: int = 6) -> None:
        hud = getattr(self, 'sitl_hud', None)
        fn = getattr(hud, 'on_log_message', None)
        if fn is not None:
            fn(int(sysid), text, severity)

    def _start_log_download(self, sysid: int) -> bool:
        """HUD 卡片「⬇ .bin」或落地自動：對這一台要清單，之後 _on_log_list 挑最新一份開始下載。"""
        sysid = int(sysid)
        link = self._link_for_sysid(sysid)
        if link is None:
            msg = f'UAV-{sysid} 沒有連線，無法下載日誌'
            self.statusBar().showMessage(msg, 6000)
            self._hud_log_msg(sysid, msg, 4)
            return False
        code, why = link_flight_state(link)
        if code in ('armed', 'unknown'):
            msg = f'日誌下載拒絕：{why}（飛控只在上鎖後才給日誌）'
            self.statusBar().showMessage(msg, 8000)
            self._hud_log_msg(sysid, msg, 4)
            logger.warning(f'[日誌] {msg}')
            return False
        st = self._log_dl_state()
        if sysid in st:
            msg = f'UAV-{sysid} 的日誌下載還在進行（{st[sysid].get("stage")}）'
            self.statusBar().showMessage(msg, 6000)
            self._hud_log_msg(sysid, msg, 5)
            return False
        st[sysid] = {'stage': 'list', 'started': time.monotonic()}
        link.request_log_list()
        # ⚠ 鏈路「立即失敗」（尚未連線）是在呼叫端執行緒、return 之前就 emit log_list_failed
        #   ⇒ 狀態此時已被 _on_log_list_failed 清掉；不可以再蓋一句「索取中…」在失敗訊息上。
        if sysid not in st:
            return False
        self._hud_log_msg(sysid, f'向 UAV-{sysid} 索取日誌清單…')
        logger.info(f'[日誌] UAV-{sysid} 索取日誌清單')
        return True

    def _start_log_download_all(self) -> None:
        """HUD「下載全部」：對每一條連線各自走 _start_log_download（各自回報）。
        HUD「單機」模式：只下載選定那一台（單機 chip 說明的承諾；2026-10-22 審查）。"""
        links = list(getattr(self, '_sitl_links', None) or [])
        if not links:
            self.statusBar().showMessage('沒有連線中的飛機，無法下載日誌', 6000)
            return
        single = single_mode_sysid(self)
        if single is not None:
            if single < 0 or self._link_for_sysid(single) is None:
                msg = single_mode_refusal(single) or f'單機模式選定的 UAV{single} 已離線，沒有下載日誌'
                self.statusBar().showMessage(msg, 8000)
                return
            ok = self._start_log_download(single)
            self.statusBar().showMessage(
                f'單機模式：只對 UAV{single} 發出日誌下載（其他 {len(links) - 1} 台未動）' if ok
                else f'單機模式：UAV{single} 的日誌下載沒有開始（原因見訊息區）', 6000)
            return
        n = sum(1 for l in links if self._start_log_download(int(getattr(l, 'sysid_label', 1))))
        self.statusBar().showMessage(f'已對 {n}/{len(links)} 台發出日誌下載', 6000)

    def _cancel_log_download(self, sysid: int) -> None:
        link = self._link_for_sysid(int(sysid))
        if link is not None:
            link.cancel_log_download()
        self._hud_log_msg(int(sysid), f'UAV-{sysid} 日誌下載取消中…', 5)

    def _on_log_list(self, sysid: int, entries: list) -> None:
        """清單回來：挑最新一份、算路徑、開始下載。沒在等清單的（例如別處要的）只記 log。"""
        sysid = int(sysid)
        st = self._log_dl_state().get(sysid)
        if not st or st.get('stage') != 'list':
            logger.info(f'[日誌] UAV-{sysid} 清單 {len(entries)} 份（非本流程索取，略過）')
            return
        latest = pick_latest_log(entries)
        if latest is None:
            self._log_dl_state().pop(sysid, None)
            msg = f'UAV-{sysid} 飛控沒有任何日誌（清單為空）'
            self.statusBar().showMessage(msg, 8000)
            self._hud_log_msg(sysid, msg, 4)
            return
        link = self._link_for_sysid(sysid)
        if link is None:
            self._log_dl_state().pop(sysid, None)
            return
        path = flight_log_path(f'UAV-{sysid}', int(latest['id']), int(latest.get('time_utc', 0) or 0))
        size = int(latest.get('size', 0) or 0)
        st.update({'stage': 'download', 'id': int(latest['id']), 'size': size, 'path': str(path),
                   'n_logs': len(entries)})
        link.download_log(int(latest['id']), str(path), size)
        # 立即失敗（armed／size 0／已有下載／沒連線）在 return 前就 emit failed ⇒ 狀態已清，別再報「下載中」
        if sysid not in self._log_dl_state():
            return
        msg = (f'UAV-{sysid} 下載日誌 #{latest["id"]}（{size / 1048576:.1f} MB，清單共 {len(entries)} 份）'
               f' → {path.name}')
        self.statusBar().showMessage(msg, 10000)
        self._hud_log_msg(sysid, msg)
        logger.info(f'[日誌] {msg}；完整路徑 {path}')

    def _on_log_list_failed(self, sysid: int, reason: str) -> None:
        sysid = int(sysid)
        self._log_dl_state().pop(sysid, None)
        msg = f'UAV-{sysid} 日誌清單失敗：{reason}'
        self.statusBar().showMessage(msg, 10000)
        self._hud_log_msg(sysid, msg, 3)
        logger.error(f'[日誌] {msg}')

    def _on_log_download_progress(self, sysid: int, info: dict) -> None:
        hud = getattr(self, 'sitl_hud', None)
        fn = getattr(hud, 'on_log_progress', None)
        if fn is not None:
            fn(int(sysid), dict(info))

    def _on_log_download_done(self, sysid: int, info: dict) -> None:
        sysid = int(sysid)
        self._log_dl_state().pop(sysid, None)
        path = str(info.get('path', ''))
        self._flight_logs()[sysid] = path
        size = int(info.get('size', 0) or 0)
        secs = float(info.get('seconds', 0.0) or 0.0)
        rate = (size / secs / 1024.0) if secs > 0 else 0.0
        import os as _os
        msg = (f'UAV-{sysid} 日誌下載完成：{_os.path.basename(path)}（{size / 1048576:.1f} MB，'
               f'{secs:.0f} s，{rate:.0f} KiB/s，sha256 {str(info.get("sha256", ""))[:12]}）')
        self.statusBar().showMessage(msg, 15000)
        self._hud_log_msg(sysid, msg)
        hud = getattr(self, 'sitl_hud', None)
        fn = getattr(hud, 'on_log_done', None)
        if fn is not None:
            fn(sysid, dict(info))
        logger.info(f'[日誌] {msg}；完整路徑 {path}')

    def _on_log_download_failed(self, sysid: int, reason: str) -> None:
        sysid = int(sysid)
        self._log_dl_state().pop(sysid, None)
        msg = f'UAV-{sysid} 日誌下載失敗：{reason}'
        self.statusBar().showMessage(msg, 15000)
        self._hud_log_msg(sysid, msg, 3)
        hud = getattr(self, 'sitl_hud', None)
        fn = getattr(hud, 'on_log_failed', None)
        if fn is not None:
            fn(sysid, str(reason))
        logger.error(f'[日誌] {msg}')

    def _note_armed_transition(self, frame) -> None:
        """每幀遙測：解鎖→上鎖＝落地 ⇒ HUD 提示可下載；勾了自動就延遲後自動開始。"""
        try:
            sysid = int(getattr(frame, 'sysid', -1))
            armed = bool(getattr(frame, 'armed', False))
        except (TypeError, ValueError):
            return
        seen = self._armed_seen()
        prev = seen.get(sysid)
        seen[sysid] = armed
        if prev is True and armed is False:
            msg = f'UAV-{sysid} 已上鎖（落地）：可下載這一趟的飛行日誌（.bin）'
            self.statusBar().showMessage(msg, 15000)
            self._hud_log_msg(sysid, msg, 5)
            logger.info(f'[日誌] {msg}')
            hud = getattr(self, 'sitl_hud', None)
            auto = getattr(hud, 'auto_log_download_enabled', None)
            if auto is not None and auto():
                delay_ms = int(self.LOG_DOWNLOAD_AUTO_DELAY_S * 1000)
                self._hud_log_msg(sysid, f'{self.LOG_DOWNLOAD_AUTO_DELAY_S:.0f} 秒後自動下載 UAV-{sysid} 的最新日誌…')

                def _fire(s=sysid):
                    # 到點**再查一次**勾選：這 8 秒內使用者取消勾選就是不要下載（數傳上是數十分鐘的傳輸）
                    hud2 = getattr(self, 'sitl_hud', None)
                    auto2 = getattr(hud2, 'auto_log_download_enabled', None)
                    if auto2 is None or not auto2():
                        self._hud_log_msg(s, f'UAV-{s} 自動下載已取消（勾選已關閉）', 5)
                        logger.info(f'[日誌] UAV-{s} 自動下載取消：勾選在延遲期間被關閉')
                        return
                    self._start_log_download(s)
                QTimer.singleShot(delay_ms, _fire)

    def _on_mission_download_failed(self, sysid: int, reason: str):
        """讀回失敗 —— **不可以**靜默，也不可以畫成空航線。

        空航線是一個合法答案（飛控裡真的沒有任務），把失敗畫成空的，
        使用者就會以為「AUTO 沒有上傳成功」而去查上傳側 —— 查錯地方。
        """
        logger.warning(f'[SITL] UAV-{sysid} 任務讀回失敗: {reason}')
        self.statusBar().showMessage(f'UAV-{sysid} 任務讀回失敗：{reason}', 10000)

    # ── 航點檔 Load / Write / Save（Mission Planner 風格；Read = 上面那組）──
    def _update_write_wp_enabled(self) -> None:
        """『寫入飛控』鈕是否可按的**唯一**決定點。

        兩個條件：至少有一條連線、而且沒有任何 sysid 正在等待 mission_uploaded
        回報。任何會改變這兩者的地方（連線/斷線、送出寫入前、回報後）都要呼叫
        這裡，而不是自己重算一次 setEnabled —— 否則按鈕狀態遲早會跟真實狀態
        脫節（例如寫入中被使用者連點兩次，開兩筆互相競爭的交易）。
        """
        enabled = bool(self._sitl_links) and not self._write_pending_sysids
        try:
            self.sitl_hud.btn_write_wp.setEnabled(enabled)
        except Exception:
            pass
        # 「寫入每架匯入的任務」多一個條件：真的有匯入過東西（__dict__：假 self 可能沒這欄）
        try:
            self.sitl_hud.btn_write_imported.setEnabled(
                enabled and bool(self.__dict__.get('_imported_missions')))
        except Exception:
            pass

    def on_sitl_load_wp_file(self):
        """Load WP File —— 從檔案載入一份任務並預覽到地圖。

        純讀檔 + 畫圖，**不需要連線**、也**不會**自動寫入飛控（要寫入另外按
        「✍️ 寫入飛控」，中間一定會再跳一次確認對話框）。
        """
        import os
        from mission.waypoint_file import MissionFileError, load_mission_file
        path, _flt = QFileDialog.getOpenFileName(
            self, '載入航點檔', self._last_wp_dir,
            'Mission Planner / QGC waypoints (*.waypoints *.txt);;'
            'QGroundControl plan (*.plan);;All files (*)')
        if not path:
            return
        self._last_wp_dir = os.path.dirname(path)
        try:
            lm = load_mission_file(path)
        except MissionFileError as e:
            # ⚠️ 不可以靜默 return：使用者按了載入卻什麼都沒發生，
            # 會以為程式沒反應而不是「這份檔案讀不進來」。
            logger.error(f'[WP] 載入航點檔失敗: {e}')
            QMessageBox.critical(self, '載入航點檔失敗', str(e))
            return
        if lm.has_errors:
            # 有錯誤的檔案完全不進 self._loaded_mission —— 保留使用者先前
            # 已經載入、驗證過的那一份，不能被一次失敗的載入悄悄蓋掉。
            QMessageBox.critical(
                self, '航點檔有錯誤，無法寫入',
                lm.summary_text() + '\n\n請修正檔案後重新載入；'
                '目前不會覆蓋先前已載入的任務。')
            return
        self._loaded_mission = lm
        # 預覽走 display_fc_mission（跟「讀回」共用同一條路）——
        # 不可以用 display_path/display_paths/display_fw_paths：那幾條會
        # 連帶重建「上傳電子圍籬」出發的圍籬，而這裡只是單純的載入預覽。
        try:
            from mission import mission_view as _mv
            path2d = _mv.mission_to_path_2d(lm.items)
            path3d = _mv.mission_to_path_3d(lm.items)
            loiters = _mv.mission_loiters(lm.items)
            home = _mv.mission_home(lm.items)
            # kind='loaded'：自己一層，不清掉其他架的讀回／匯入預覽（不帶 kind 的 sysid=None 是「先清全部層」）
            if hasattr(self.map_widget, 'display_fc_mission'):
                self.map_widget.display_fc_mission(path2d, path3d, loiters,
                                                    home=home, kind='loaded',
                                                    label=f'載入：{os.path.basename(path)}')
        except Exception as e:
            logger.error(f'[WP] 預覽載入的航點檔失敗: {e}', exc_info=True)
        self.statusBar().showMessage(
            f'已載入 {os.path.basename(path)}：{len(lm.items)} 個項目', 8000)
        logger.info(f'[WP] 已載入航點檔 {path}: {len(lm.items)} 項、'
                    f'{len(lm.warnings)} 個警告')
        QMessageBox.information(self, '航點檔已載入', lm.summary_text())

    # ── 多檔匯入：每個 .waypoints／.plan 明確指派給一架 → 逐機預覽 → 逐機寫入 ──
    #   規則只有 mission/import_missions.py 一份（載入≠指派、sysid 不用猜、問題清單擋確定）；
    #   這裡只做：開對話框 → 收結果 → 畫圖 → 逐機走 on_sitl_write_wp 同一套守衛。
    def _run_import_dialog(self):
        """開「多檔匯入」對話框；回 list[ImportedMission]（使用者取消 ⇒ None）。

        獨立成一小段：測試可以整段換掉，不必真的開 QDialog。
        """
        from ui.dialogs.import_waypoints_dialog import FILE_FILTER, ImportWaypointsDialog
        # 先選檔（可多選）再開指派表；一個都沒選就當取消。對話框裡還能再「加入檔案…」。
        paths, _flt = QFileDialog.getOpenFileNames(
            self, '匯入航點檔（可多選，每檔指派一架）', self._last_wp_dir, FILE_FILTER)
        if not paths:
            return None
        known = sorted({int(getattr(l, 'sysid_label', 1)) for l in self._sitl_links})
        dlg = ImportWaypointsDialog(self, paths=list(paths), links_sysids=known,
                                    last_dir=self._last_wp_dir)
        if not dlg.exec():
            return None
        # ⚠ dlg.result() 被對話框改成回 list[ImportedMission]（取消＝空清單），不是 QDialog 的代碼
        return dlg.result() or None

    def on_import_waypoints(self):
        """📂 多檔匯入 → 每架一份（HUD 鈕與「檔案」選單共用）。

        純讀檔 + 畫圖，**不需要連線、不會寫飛控**；寫入另外按「✍️ 寫入每架匯入的任務」。
        """
        missions = self._run_import_dialog()
        if not missions:
            return
        self._apply_imported_missions(missions)

    def _apply_imported_missions(self, missions) -> dict:
        """收下對話框確認過的指派結果：再驗一次 → 換掉上一批 → 逐機預覽 → 更新鈕。

        回 `{sysid: ImportedMission}`；被擋下就回 {} 且**什麼都不改**（上一批保留）。

        ⚠ 對話框已經擋過的事這裡再驗一次（`assignment_problems`）：這是「真的會被
          寫進飛控的那份」最後一道，不信任 UI 層。
        """
        import os
        from mission.import_missions import assignment_problems
        missions = list(missions or [])
        probs = assignment_problems(missions)
        if probs:
            logger.error(f'[WP] 多檔匯入被擋下：{probs}')
            QMessageBox.critical(
                self, '匯入航點檔',
                '指派結果有問題，**沒有匯入任何檔**（上一批匯入保留）：\n\n' + '\n'.join(probs))
            return {}
        # 上一批匯入的層只清「匯入的那一種」（kind='import'）——同一架的「📥 讀回」層（鍵 uav<n>）
        # 與單檔載入層（'loaded'）都不動。
        old = self.__dict__.get('_imported_missions') or {}
        for sysid in list(old):
            try:
                self.map_widget.clear_fc_mission(sysid=int(sysid), kind='import')
            except Exception as e:
                logger.warning(f'[WP] 清除 UAV-{sysid} 舊匯入層失敗：{e}')
        by_sysid = {int(m.sysid): m for m in missions}
        self._imported_missions = by_sysid
        try:
            self._last_wp_dir = os.path.dirname(missions[0].path)
        except Exception:
            pass
        drawn = self._preview_imported_missions(by_sysid)
        self._update_write_wp_enabled()

        connected = {int(getattr(l, 'sysid_label', 1)) for l in self._sitl_links}
        missing = sorted(s for s in by_sysid if s not in connected)
        # m.label：load_many 在同名檔時會加上父目錄（siteA/mission.waypoints），不要再用 basename
        parts = [f'匯入 {len(by_sysid)} 份：' + '、'.join(
            f'UAV-{s}←{m.label or os.path.basename(m.path)}' for s, m in sorted(by_sysid.items()))]
        if drawn != len(by_sysid):
            parts.append(f'⚠ 只畫出 {drawn}/{len(by_sysid)} 份（看 log）')
        if missing:
            parts.append('尚未連線：' + '、'.join(f'UAV-{s}' for s in missing))
        self.statusBar().showMessage('；'.join(parts), 12000)
        logger.info('[WP] ' + '；'.join(parts))
        return by_sysid

    def _preview_imported_missions(self, by_sysid) -> int:
        """每架一層疊到 2D/3D（顏色依 sysid）；只對第一架 fit 視圖。回畫成功的份數。

        ⚠ 走 `display_fc_mission(sysid=…)`，不可以用 display_path*（會重建圍籬，見
          on_sitl_load_wp_file 的註解）。一架畫失敗不影響其他架，但要留 log。
        """
        import os
        from mission import mission_view as _mv
        n = 0
        for i, (sysid, m) in enumerate(sorted(by_sysid.items())):
            try:
                items = list(m.items)
                self.map_widget.display_fc_mission(
                    _mv.mission_to_path_2d(items), _mv.mission_to_path_3d(items),
                    _mv.mission_loiters(items), home=_mv.mission_home(items),
                    sysid=int(sysid), kind='import',     # 鍵 uav<n>-import：與讀回層（uav<n>）並存
                    label=f'UAV-{int(sysid)}：{m.label or os.path.basename(m.path)}',
                    fit=(i == 0))
                n += 1
            except Exception as e:
                logger.error(f'[WP] 預覽 UAV-{sysid} 的匯入任務失敗：{e}', exc_info=True)
        return n

    def on_sitl_write_imported(self):
        """✍️ 寫入每架匯入的任務：每個 sysid 拿**自己那份**，各自走 `on_sitl_write_wp`。

        對象不看 HUD「指令對象」——UAV-1 就寫 UAV-1 的 link（sysid_label 相等）。
        每架各跳一次確認對話框；取消只跳過那架。指派了但沒連線的 sysid 最後列出，
        **絕不**寫到別台。
        """
        by_sysid = self.__dict__.get('_imported_missions') or {}
        if not by_sysid:
            QMessageBox.information(
                self, '寫入每架匯入的任務',
                '尚未匯入任何航點檔。\n請先按「📂 多檔匯入 → 每架一份」選檔並指派 sysid。')
            return
        # HUD「單機」模式：只寫選定那一台的那一份（單機 chip 說明的承諾；2026-10-22 審查：以前照樣逐架寫全部）
        single = single_mode_sysid(self)
        if single is not None:
            if single < 0:
                QMessageBox.information(self, '寫入每架匯入的任務', single_mode_refusal(single))
                return
            if int(single) not in {int(s) for s in by_sysid}:
                QMessageBox.information(
                    self, '寫入每架匯入的任務',
                    f'單機模式只寫 UAV{single}，但沒有指派給 UAV{single} 的匯入檔（已指派：'
                    + '、'.join(f'UAV-{s}' for s in sorted(by_sysid)) + '）。\n'
                    '沒有寫入任何一台；要寫其他架請切回「群控」。')
                return
            by_sysid = {s: m for s, m in by_sysid.items() if int(s) == int(single)}
        missing = []
        ambiguous = []
        for sysid in sorted(by_sysid):
            links = [l for l in self._sitl_links
                     if int(getattr(l, 'sysid_label', 1)) == int(sysid)]
            if not links:
                missing.append(int(sysid))
                continue
            if len(links) > 1:
                # 兩條連線掛同一個 sysid（HIL 兩列都填 2、或 SITL＋HIL）：on_sitl_write_wp 會以
                # 「多於一台」拒絕並叫人去點卡片——但這條路本來就不看卡片，點了也沒用 ⇒ 這裡把真正原因講出來
                conns = '、'.join(str(getattr(l, 'conn_str', '?')) for l in links)
                ambiguous.append((int(sysid), conns))
                logger.error(f'[WP] UAV-{sysid} 有 {len(links)} 條連線（{conns}），跳過寫入')
                continue
            # 同一套守衛：有錯誤拒絕、恰好一台、併發、確認對話框、快照
            self.on_sitl_write_wp(target_links=links, mission=by_sysid[sysid].loaded)
        notes = []
        if missing:
            notes.append('以下 sysid 有指派檔案但目前沒有連線，**沒有寫入**（也不會寫到別台）：\n'
                         + '、'.join(f'UAV-{s}' for s in missing))
        if ambiguous:
            notes.append('以下 sysid 同時有多條連線，分不出要寫哪一條，**沒有寫入**；請先斷開其中一條再試：\n'
                         + '\n'.join(f'UAV-{s}：{c}' for s, c in ambiguous))
        if notes:
            QMessageBox.information(
                self, '寫入每架匯入的任務',
                '\n\n'.join(notes)
                + '\n\n處理好之後再按一次即可；已寫入的那幾架不會重寫（會再各跳一次確認）。')

    def on_sitl_write_wp(self, target_links=None, mission=None):
        """Write WPs（安全關鍵）—— 把已載入的航點檔寫入**恰好一台**飛控。

        target_links: 通常留 None，此時對象由 HUD 的「指令對象」
        （`_target_links()`）決定；DroneDetailPanel 的「↥」鈕會直接帶入該
        面板 callsign 對應的單一 link（見 `_on_detail_panel_write_wp`），
        但**不會**因此少走任何一道下面的守衛——換的只是「對象怎麼決定」，
        不是「要不要檢查」。
        mission: 通常留 None ⇒ 寫 `self._loaded_mission`（📂 載入航點檔那份）；
        「多檔匯入」逐機寫入（`on_sitl_write_imported`）會帶入該 sysid 指派到的
        那份 `LoadedMission`——同樣只換「資料來源」，守衛、確認對話框、快照、
        併發檢查一道都不少。

        ⚠️ 刻意不重用 `on_sitl_upload_mission`：那條走的是「當前規劃路徑」
        （地圖 paths / DCCPP 結果），資料來源、語意、失敗處理都不同，
        混用只會讓兩條路都變得難懂、難測。
        """
        import os
        # (1) 檔案面：沒載入、或載入了但有錯誤 —— 一律拒絕，不猜、不部分寫。
        lm = mission if mission is not None else self._loaded_mission
        if lm is None:
            QMessageBox.information(
                self, 'Write WPs',
                '尚未載入任何航點檔。\n請先按「📂 載入航點檔」選擇 '
                '.waypoints / .plan 檔案。')
            return
        if lm.has_errors:
            QMessageBox.critical(
                self, 'Write WPs：航點檔有錯誤',
                '這份航點檔有錯誤，寫入飛控前必須先修正：\n\n'
                + lm.summary_text())
            return

        # (2) 對象面：必須恰好一台。這個專案已經撞過「兩架飛機拿到同一份
        #     任務」的事故（on_sitl_upload_mission 的 per_uav_wps 註解），
        #     這裡把「多於一台」與「零台」都當拒絕理由，不猜使用者想寫哪台。
        links = list(target_links) if target_links is not None else self._target_links()
        if len(links) > 1:
            QMessageBox.warning(
                self, 'Write WPs',
                f'目前指令對象是 {len(links)} 台 UAV。\n'
                '「寫入飛控」只能對**恰好一台**執行——這個專案曾經發生過'
                '「兩架飛機拿到同一份任務」的事故，這裡刻意拒絕多機寫入。\n\n'
                '請先在 HUD 上點選單一 UAV 卡片，再重試。')
            return
        if not links:
            QMessageBox.information(
                self, 'Write WPs',
                '所選的指令對象 UAV 已離線（或尚未連線任何飛控），無法寫入。')
            return
        link = links[0]
        sysid = int(getattr(link, 'sysid_label', 1))

        # (2.5) 併發面：這台 UAV 已經有一筆寫入在跑——不論是從 HUD 按鈕、
        #     還是從 DroneDetailPanel 的「↥」鈕（見 _on_detail_panel_write_wp）
        #     進來的都一樣要擋。_update_write_wp_enabled() 只會關閉 HUD 那顆
        #     按鈕，DroneDetailPanel 的按鈕沒有對應的 enabled 狀態，所以「對象
        #     是否已經在寫」不能只靠某一個 widget 被 disable 來防，必須在這個
        #     函式的入口自己查 _write_pending_sysids——否則兩條路徑各自排一筆
        #     upload_mission_items() 到同一台飛控，先回來的 mission_uploaded
        #     會把 pending 清掉、兩顆按鈕都重新可按，而另一筆其實還沒有結果。
        if sysid in self._write_pending_sysids:
            QMessageBox.information(
                self, 'Write WPs',
                f'UAV-{sysid} 已經有一筆寫入在進行中，請等它完成（成功或失敗）'
                '後再試一次。\n\n'
                '（這個檢查涵蓋所有入口——HUD 的「✍️ 寫入飛控」按鈕，和 '
                'DroneDetailPanel 的「↥」快速鈕，寫的是同一台飛控就會互相'
                '擋，不管是哪一個先按下去。）')
            return

        # (3) 確認對話框——顯示的必須是「等一下真的會送出去的那份」。
        conn_str = str(getattr(link, 'conn_str', '?'))
        is_real_board = looks_like_real_board_connection(conn_str)
        items = lm.items
        first_wp = items[1] if len(items) > 1 else (items[0] if items else None)
        last_wp = items[-1] if items else None

        lines = [
            f'目標：UAV-{sysid}（sysid={sysid}）',
            f'連線：{conn_str}' + ('　⚠️ 這看起來是「真實飛控」（序列埠）'
                                  if is_real_board else '　（SITL / 網路連線）'),
            # 完整路徑：兩個資料夾裡同名的 mission.waypoints 只看檔名分不出要寫哪一份（審查 2026-10-17）
            f'檔案：{lm.path if lm.path else "?"}',
            f'項目數：{len(items)}',
        ]
        if first_wp is not None:
            lines.append(
                f'第一個航點：#{int(first_wp["seq"])} cmd={int(first_wp["command"])} '
                f'({first_wp["lat"]:.6f}, {first_wp["lon"]:.6f}, {first_wp["alt"]:.1f}m)')
        if last_wp is not None:
            lines.append(
                f'最後一個航點：#{int(last_wp["seq"])} cmd={int(last_wp["command"])} '
                f'({last_wp["lat"]:.6f}, {last_wp["lon"]:.6f}, {last_wp["alt"]:.1f}m)')
        if lm.warnings:
            lines.append('')
            lines.append(f'{len(lm.warnings)} 個警告（仍允許寫入，但請先看過）：')
            for w in lm.warnings:
                where = '' if w.seq is None else f'#{w.seq} '
                lines.append(f'  [警告] {where}{w.message}')

        frame = None
        try:
            frame = link.get_latest_telemetry()
        except Exception:
            frame = None
        in_flight_auto = bool(
            frame is not None and getattr(frame, 'armed', False)
            and str(getattr(frame, 'mode', '')).upper() == 'AUTO')
        if in_flight_auto:
            lines.insert(
                0, '⚠️⚠️ 這台 UAV 目前 ARMED 且在 AUTO 模式——這將取代一份'
                   '**飛行中的任務**！請務必確認這是你要做的事。')

        # ⚠️ 用 QMessageBox.warning/.question 這兩個**類別方法**而不是自己
        #    `QMessageBox(self)` 建實體——一來圖示照案例自動對，二來這是
        #    本專案既有的整體替換測法（見 test_dccpp_reroute_altitude.py 的
        #    `_QtW.QMessageBox.warning = staticmethod(...)`），不必為了測試
        #    另外發明一套攔截機制。
        #    真實飛控 / 飛行中 AUTO 兩種都用 warning()（警告圖示，措辭更重）；
        #    其餘情況用 question()（詢問圖示）。
        msg_fn = QMessageBox.warning if (is_real_board or in_flight_auto) \
            else QMessageBox.question
        # 真實飛控預設停在「取消」——多一道刻意的摩擦，逼使用者主動選「是」。
        default_btn = (QMessageBox.StandardButton.Cancel if is_real_board
                       else QMessageBox.StandardButton.Yes)
        ret = msg_fn(
            self,
            '⚠️ 寫入真實飛控 —— Write WPs' if is_real_board else 'Write WPs 確認',
            '\n'.join(lines),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            default_btn,
        )
        if ret != QMessageBox.StandardButton.Yes:
            # 唯一允許的「什麼都不送」路徑：Cancel（含叉掉視窗）一律不送。
            logger.info(f'[WP] 使用者取消寫入 UAV-{sysid}')
            return

        # (4) 快照——在**這一刻**複製。對話框開著的時候另外載入別的檔案
        #     不可以改到已經排隊要送出的內容；確認對話框顯示的就是實際
        #     送出的內容，這是本函式唯一的正確性承諾。
        snapshot = list(lm.items)
        # 飛行計畫快照：這架要寫別的任務了 ⇒ 還在等它建圖寫入結果的批次不能再拿之後的 ACK（那是這筆 Write WPs 的）
        try:
            abandon_flown_plan_sysid(self, sysid, '改寫入別的任務（Write WPs），建圖任務的寫入結果沒有收到')
        except Exception:           # noqa: BLE001
            pass
        self._write_pending_sysids.add(sysid)
        self._update_write_wp_enabled()
        try:
            link.upload_mission_items(snapshot, verify=True)
        except Exception as e:
            logger.error(f'[WP] 送出寫入請求失敗 UAV-{sysid}: {e}', exc_info=True)
            self._write_pending_sysids.discard(sysid)
            self._update_write_wp_enabled()
            QMessageBox.critical(self, 'Write WPs 失敗', f'送出寫入請求失敗：{e}')
            return
        # 這架的任務換成載入檔了 ⇒ 不再屬於 HARVEST 起飛時程
        forget_release_schedule(self, [sysid], reason='Write WPs 寫入載入檔')
        self.statusBar().showMessage(
            f'正在寫入 UAV-{sysid}（{len(snapshot)} 項）…', 5000)
        logger.info(f'[WP] 已送出寫入請求 UAV-{sysid}: {len(snapshot)} 項')

    def _on_mission_uploaded(self, sysid: int, ok: bool, message: str):
        """任務**寫入**結果——`SITLLink.mission_uploaded` 的**唯一**接收端。

        失敗必須夠大聲：一個被拒絕 / 逾時 / 只寫一半 / 沒驗證的寫入絕對不能
        看起來像成功——那會讓使用者帶著一份沒生效的任務去飛。
        """
        self._write_pending_sysids.discard(sysid)
        self._update_write_wp_enabled()
        # HARVEST 起飛時程：記下這架的任務有沒有寫成功（寫失敗的不放行）
        _sched = getattr(self, '_hv_release_schedule', None)
        if _sched and int(sysid) in (_sched.get('entries') or {}):
            _sched.setdefault('upload_ok', {})[int(sysid)] = bool(ok)
        if ok:
            logger.info(f'[WP] UAV-{sysid} 寫入成功: {message}')
            self.statusBar().showMessage(
                f'✅ UAV-{sysid} 任務寫入完成（已驗證）：{message}', 10000)
        else:
            logger.error(f'[WP] UAV-{sysid} 寫入失敗: {message}')
            # 建圖條帶監測器的段表是照「這份任務」建的；沒寫進去就不能拿它去對飛控上的舊任務
            _st = (getattr(self, '_map_photo_stats', None) or {}).get(int(sysid))
            if _st is not None and _st.get('monitor') is not None:
                _st['monitor'] = None
                logger.error(f'[航測] UAV-{sysid} 任務沒寫成功 ⇒ 停用這架的條帶監測器')
            self.statusBar().showMessage(
                f'❌ UAV-{sysid} 任務寫入失敗：{message}', 10000)
            QMessageBox.critical(
                self, 'Write WPs 失敗',
                f'UAV-{sysid} 的飛控任務**沒有被可靠地更新**：\n\n{message}\n\n'
                '飛行前請先按「📥 從飛控讀回任務航線」確認飛控上實際存著的內容，'
                '不要假設剛才的寫入已經生效。')
        # 飛行計畫快照（只有建圖上傳那批的 ACK 會對上）：第一個成功才存檔；存不成大聲說，但不影響這次寫入的結果
        note_flown_plan_upload(self, sysid, ok, message)

    def on_sitl_save_wp_file(self):
        """Save WP File —— 存的是**剛從飛控讀回**的那份任務，不是載入檔。

        ⚠️ 這顆鈕永遠讀 `self._fc_mission`（`_on_mission_downloaded` 填的），
        不是 `self._loaded_mission`——後者是「要寫進去的」，前者才是「飛控
        上實際有的」；用錯來源會讓使用者以為自己存到了飛控現況，實際存的
        卻是還沒（或根本沒打算）寫入的東西。
        """
        import os
        from mission.waypoint_file import MissionFileError, save_wpl
        fc = self._fc_mission
        if not fc or not fc.get('items'):
            QMessageBox.information(
                self, 'Save WP File',
                '尚未從飛控讀回任何任務。\n請先按「📥 從飛控讀回任務航線」，'
                '再用這顆鈕把讀回的結果存成檔案。')
            return
        default_name = f'UAV-{fc["sysid"]}_readback.waypoints'
        start = (os.path.join(self._last_wp_dir, default_name)
                 if self._last_wp_dir else default_name)
        path, _flt = QFileDialog.getSaveFileName(
            self, '存成航點檔', start,
            'Mission Planner / QGC waypoints (*.waypoints)')
        if not path:
            return
        if not path.lower().endswith('.waypoints'):
            path += '.waypoints'
        self._last_wp_dir = os.path.dirname(path)
        try:
            save_wpl(fc['items'], path)
        except MissionFileError as e:
            logger.error(f'[WP] 存檔失敗: {e}')
            QMessageBox.critical(self, '存檔失敗', str(e))
            return
        self.statusBar().showMessage(f'已存成 {os.path.basename(path)}', 6000)
        logger.info(f'[WP] UAV-{fc["sysid"]} 讀回結果已存成 {path}')

    def _on_detail_panel_write_wp(self, callsign: str):
        """DroneDetailPanel 的「↥」鈕 → 單機版 Write WPs。

        ⚠️ 不直接呼叫 `link.upload_mission_items`——那會跳過『已載入且沒有
        錯誤』檢查與確認對話框。真正的守衛邏輯只有 `on_sitl_write_wp` 一份，
        這裡只是把「對象」從 HUD 的指令對象換成這個面板選定的 callsign
        （格式固定 'UAV-<sysid>'，與 `on_sitl_connect` 註冊 FleetRegistry
        時一致）。
        """
        try:
            sysid = int(str(callsign).rsplit('-', 1)[-1])
        except (ValueError, IndexError):
            QMessageBox.warning(self, 'Write WPs', f'無法從 {callsign!r} 解析 sysid')
            return
        links = [l for l in self._sitl_links
                if int(getattr(l, 'sysid_label', 1)) == sysid]
        self.on_sitl_write_wp(target_links=links)

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
        """把當前路徑上傳到所有連線的 SITL（支援 Diamond 蜂群打擊 / DCCPP / 一般路徑）。

        HUD「單機」模式：只寫入選定那一台（每架各自路徑時，配對照樣用全部連線算 ⇒ 那台拿到的就是群控上傳時它會
        拿到的那一份）；整批才有意義的建圖上傳（參數讀回閘門＋飛行計畫快照）與菱形打擊任務在單機模式不送。
        以前單機模式照樣寫進每一台 —— 對正在 AUTO 的別架等於換掉它的航線（2026-10-22 審查）。
        """
        if not self._sitl_links:
            QMessageBox.information(self, 'SITL', '尚未連線任何 SITL')
            return
        single = single_mode_sysid(self)
        scope = None                    # None ＝ 全部連線；單機 ⇒ 只有選定那台的 link
        if single is not None:
            scope = [l for l in self._sitl_links if int(getattr(l, 'sysid_label', -1)) == single] \
                if single >= 0 else []
            if not scope:
                QMessageBox.information(self, '上傳任務', single_mode_refusal(single)
                                        or f'單機模式選定的 UAV{single} 已離線，沒有上傳。')
                return
            if getattr(self, '_diamond_plans', None) or self._hv_mapping_upload_applies():
                QMessageBox.information(
                    self, '上傳任務：單機模式不送整批任務',
                    '這份是整批上傳的任務（建圖：上傳前參數讀回閘門＋飛行計畫快照；或多機打擊任務），'
                    f'單機模式只對 UAV{single} 下指令，沒有上傳任何一台。\n要上傳請切回「群控」。')
                return
        # ── 優先：菱形蜂群打擊（每架獨立任務 + 友善參數）──────
        if getattr(self, '_diamond_plans', None):
            ret = self._on_diamond_strike_sitl_upload()
            # 菱形打擊任務送到每一條連線 ⇒ 飛控上不再是 HARVEST 任務（它有自己的 KAMIKAZE LAUNCH 時程）
            forget_release_schedule(self, [int(getattr(l, 'sysid_label', 0)) for l in self._sitl_links],
                                    reason='菱形打擊任務上傳')
            return ret
        # ── 航測建圖（HARVEST 建圖模式）：hv 匯出器的任務（含 206 拍照指令）+ 上傳前參數讀回閘門 ──
        if self._hv_mapping_upload_applies():
            return self._on_hv_mapping_upload()
        # 從 map_widget.paths 收集所有航點；若無則改抓 DCCPP / swarm 結果
        wps = []
        hv_used = None          # 這次上傳的是 HARVEST 結果（upload_tuples）⇒ 要記起飛時程（R1 #1）
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

                # ── HARVEST：改用轉接層的 8-tuple（手冊 §12.3）──
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
                            f'[SITL] HARVEST upload_tuples: '
                            f'{ {k: len(v) for k, v in per_uav_wps.items()} }')
                        hv_used = _hv
                    except Exception as e:      # noqa: BLE001
                        logger.error(
                            f'[SITL] HARVEST upload_tuples 失敗，退回 _build_wps_for_path：{e}',
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
                if scope is not None and link not in scope:
                    continue            # 單機：配對照算（別台的配對不變），只是不寫入別台
                link.upload_mission(path_wps)
                uploaded.append((sysid, target_uav_id, len(path_wps)))
                logger.info(
                    f'[SITL] sysid={sysid} ← UAV-{target_uav_id} ({len(path_wps)} 點)'
                )
            self.statusBar().showMessage(
                (f'單機模式：只為 UAV{single} 上傳（其他 {len(self._sitl_links) - len(scope)} 台未動）；'
                 if scope is not None else '')
                + f'已為 {len(uploaded)}/{len(self._sitl_links)} 台 SITL 上傳獨立 '
                f'DCCPP 路徑（{len(skipped)} 跳過）', 6000
            )
            if hv_used is not None and uploaded:
                # R1 #1（非建圖的 HARVEST 也一樣）：上傳報告列每架起飛時程；時程不同 ⇒ 一鍵起飛會被擋
                entries, src = record_release_schedule(
                    self, hv_used, [(s, u) for s, u, _n in uploaded], mapping=False)
                text = (f'已為 {len(uploaded)}/{len(self._sitl_links)} 台送出 HARVEST 任務：\n  '
                        + '\n  '.join(f'UAV-{s} ← 機 {u}：{n} 點' for s, u, n in uploaded)
                        + '\n\n' + release_report_text(getattr(self, '_hv_release_schedule', None), entries, src,
                                                       mapping=False))
                QMessageBox.information(self, 'HARVEST：已送出上傳', text)
            else:
                forget_release_schedule(self, [s for s, _u, _n in uploaded], reason='非 HARVEST 任務上傳')
        else:
            targets = list(self._sitl_links) if scope is None else list(scope)
            for link in targets:
                link.upload_mission(wps)
            forget_release_schedule(self, [int(getattr(l, 'sysid_label', 0)) for l in targets],
                                    reason='一般航點上傳')
            self.statusBar().showMessage(
                (f'單機模式：只對 UAV{single} 上傳 {len(wps)} 個航點'
                 f'（其他 {len(self._sitl_links) - len(targets)} 台未動）' if scope is not None
                 else f'已對 {len(self._sitl_links)} 個 SITL 上傳 {len(wps)} 個航點'), 5000
            )

    # ─────────────────────────────────────────────────────────────────
    # 航測建圖：上傳（閘門 → 工作執行緒讀回參數 → 不符就擋／寫入後重讀 → upload_mission_items）
    # ─────────────────────────────────────────────────────────────────
    def _hv_mapping_upload_applies(self) -> bool:
        """這份結果是 HARVEST **建圖模式**規劃出來的（看規劃當時的 coordinator，不看面板現在勾不勾）。"""
        hv = getattr(self, '_hv_coordinator', None)
        res = getattr(hv, 'last_result', None) if hv is not None else None
        return (res is not None and res is getattr(self, '_dccpp_result', None)
                and getattr(hv, 'mapping_mode', None) is not None)

    def _on_hv_mapping_upload(self):
        """建圖上傳的第一步：閘門 → 產生每架任務項目 → 背景讀回飛控參數（結果在 _on_mapping_readback_done）。

        任務項目 = hv 匯出器（hv.mission_items，含每條帶 206 開／關、DO_LAND_START 後 206(0)）；
        與「匯出航點」寫出的 .waypoints 逐項相同。
        """
        hv = self._hv_coordinator
        cc = getattr(hv, 'camera_cfg', None)
        if cc is None:
            QMessageBox.critical(self, '航測建圖：拒絕上傳',
                                 '這份建圖計畫沒有相機觸發設定（camera_cfg），產生不了拍照指令。請重新規劃。')
            return
        gate = hv_upload_gate(hv, getattr(self, '_hv_swarm_coordinator', None))
        if not gate['uploadable']:
            logger.error(f'[航測] 建圖閘門擋下上傳：{gate["reasons"]}')
            QMessageBox.critical(
                self, '航測建圖：拒絕上傳',
                '這份計畫不能上傳：\n  ' + '\n  '.join(gate['reasons'])
                + '\n\npreflight 的 GO_WITH_WARNINGS 不代表整區都有人拍，也不代表通過時空去衝突／能量驗證（validate）。'
                  '請調整機隊、作業區或風之後重新規劃。')
            return
        worker = getattr(self, '_map_readback_worker', None)
        if worker is not None and worker.isRunning():
            QMessageBox.information(self, '航測建圖', '上一次的參數讀回還在進行，請等它完成。')
            return
        # R1 #10：只送 HUD「指令對象」選到的飛機。對應規則用**全部**連線算（選單機時機號對應不變），再篩出目標。
        targets = list(self._target_links())
        if not targets:
            QMessageBox.information(self, '航測建圖', '所選的指令對象 UAV 已離線，請改選其他機或「全部」')
            return
        target_ids = {int(getattr(l, 'sysid_label', -1)) for l in targets}
        uav_ids = sorted(int(k) for k in ((self._dccpp_result or {}).get('assembled_paths') or {}))
        all_pairs, skipped = assign_links_to_uavs(self._sitl_links, uav_ids)
        pairs = [(l, d) for l, d in all_pairs if int(l.sysid_label) in target_ids]
        not_targeted = sorted(int(l.sysid_label) for l, _d in all_pairs if int(l.sysid_label) not in target_ids)
        skipped = [s for s in skipped if s in target_ids]
        busy = [int(l.sysid_label) for l, _d in pairs if int(l.sysid_label) in self._write_pending_sysids]
        if busy:
            QMessageBox.information(self, '航測建圖', f'UAV-{busy} 已經有一筆寫入在進行中，請等它完成後再試。')
            return
        # R1 #10：已解鎖／AUTO／狀態不明的飛控不換任務（可能正在飛）
        states = [p for p in (link_flight_state_problem(l) for l, _d in pairs) if p]
        if states:
            logger.error(f'[航測] 擋下上傳（飛控狀態）：{states}')
            QMessageBox.critical(self, '航測建圖：拒絕上傳',
                                 '下列飛控不能換任務（已解鎖可能正在飛、AUTO 會改到正在執行的任務、狀態不明不猜）：\n  '
                                 + '\n  '.join(states) + '\n\n請先上鎖、切出 AUTO 並等遙測進來後再上傳。')
            return
        jobs = []
        try:
            for link, did in pairs:
                items = hv.mission_items(did, camera_cfg=cc)
                jobs.append({'sysid': int(link.sysid_label), 'link': link, 'uav_id': int(did), 'items': items,
                             'expected': hv.expected_camera_params(did, camera_cfg=cc),
                             'airspeed_expected': mapping_expected_airspeed(hv, did, cc),
                             'speeds': mapping_airspeed_commands(items)})
        except Exception as e:      # noqa: BLE001 — 例如引入段值鎖不符：要說出來，不能送一份錯的任務
            logger.error(f'[航測] 產生任務項目失敗：{e}', exc_info=True)
            QMessageBox.critical(self, '航測建圖：拒絕上傳', f'產生任務項目失敗：{e}')
            return
        if not jobs:
            QMessageBox.warning(self, '航測建圖', '沒有任何連線對得上有任務的 UAV（sysid 與機號對不起來）。')
            return
        if skipped:
            logger.warning(f'[航測] 這些連線沒有對應的任務，不上傳：sysid {skipped}')
        if not_targeted:
            logger.info(f'[航測] 這些連線不是指令對象，這次不上傳：sysid {not_targeted}')
        self._map_upload_jobs = jobs
        self._map_upload_skipped = list(skipped)
        self._map_upload_not_targeted = list(not_targeted)
        # R1 #9：記下這批任務是哪一份計畫產生的；讀回是非同步的，回來時計畫換了就不能送
        self._map_upload_ident = {'hv': hv, 'result': getattr(hv, 'last_result', None)}
        self._map_write_round = 0
        self._map_reboot_pending = {}
        logger.info(f'[航測] 上傳前讀回 {len(jobs)} 架：' + '；'.join(
            f'UAV-{j["sysid"]}←機 {j["uav_id"]}（{len(j["expected"])} 個相機參數＋AIRSPEED_MIN/MAX＋LOG_BITMASK／'
            f'LOG_DISARMED）' for j in jobs))
        self.statusBar().showMessage(f'航測建圖：讀回 {len(jobs)} 架飛控的相機、空速與日誌參數中…（不用 MAVFTP）', 0)
        self._start_mapping_readback(jobs)

    def _hv_upload_gate(self, hv) -> dict:
        """建圖上傳閘門（見模組層級 :func:`hv_upload_gate`：mapping_gate ＋ validate 的原因）。"""
        return hv_upload_gate(hv, getattr(self, '_hv_swarm_coordinator', None))

    def _mapping_upload_stale_reason(self):
        """R1 #9：讀回（非同步，最壞約 27 s）回來時，這批任務的計畫還是不是目前的計畫？是 ⇒ None；否則回原因。"""
        ident = getattr(self, '_map_upload_ident', None) or {}
        hv = getattr(self, '_hv_coordinator', None)
        if not ident or hv is None or hv is not ident.get('hv'):
            return '讀回期間重新規劃過（規劃器物件已換）'
        if getattr(hv, 'last_result', None) is not ident.get('result'):
            return '讀回期間重新規劃過（計畫結果已換）'
        if not self._hv_mapping_upload_applies():
            return '目前畫面上的結果已不是這份建圖計畫'
        return None

    def _abort_stale_mapping_upload(self) -> bool:
        """計畫在讀回期間換了 ⇒ 取消這批上傳（大聲說）並回 True。"""
        why = self._mapping_upload_stale_reason()
        if not why:
            return False
        logger.error(f'[航測] 取消上傳：{why}；讀回開始時的那份任務不再送出')
        self._map_upload_jobs = []
        self.statusBar().showMessage('航測建圖：計畫已變更，這次上傳取消', 10000)
        QMessageBox.critical(self, '航測建圖：計畫已變更，取消上傳',
                             f'{why}。\n\n讀回開始時的那份任務**沒有**送出（送出去會跟畫面上的新計畫不一致）。'
                             '請確認新計畫後再按一次「上傳飛控」。')
        return True

    def _start_mapping_readback(self, jobs):
        """讀回在工作執行緒跑（read_params 會同步等飛控回覆，最壞 9 s／架）；結果交回 GUI 執行緒。"""
        w = MappingReadbackWorker(jobs, parent=None)
        w.done.connect(self._on_mapping_readback_done)
        self._map_readback_worker = w
        w.start()

    def _on_mapping_readback_done(self, results: dict):
        """讀回比對完成：全部相符 ⇒ 上傳；不符 ⇒ 擋下、列出、問要不要寫入（寫完重讀重比一次）。"""
        jobs = list(getattr(self, '_map_upload_jobs', None) or [])
        for s, r in sorted(results.items()):
            logger.info(f'[航測] UAV-{s} 讀回：' + '、'.join(
                f'{row["name"]}={_fmt_param(row["actual"])}{"" if row["ok"] else "（不符）"}'
                for row in r.get('rows', [])) + (f'；讀回失敗 {r["error"]}' if r.get('error') else ''))
        # R1 #9：讀回期間重新規劃過 ⇒ 這批任務是舊計畫的，不送
        if self._abort_stale_mapping_upload():
            return
        # LOG_DISARMED 等資訊列（不擋）：上傳報告要顯示
        self._map_readback_info = {int(s): [row['info'] for row in r.get('rows', []) if row.get('info')]
                                   for s, r in results.items()}
        bad = {s: r for s, r in results.items()
               if r.get('error') or any(not row['ok'] for row in r.get('rows', []))}
        if not bad:
            reboot = getattr(self, '_map_reboot_pending', None) or {}
            if reboot:
                names = sorted({n for v in reboot.values() for n in v})
                self._map_reboot_pending = {}
                QMessageBox.warning(
                    self, '航測建圖：要重開飛控',
                    f'參數已寫入並讀回相符，但 {"、".join(names)} 要**重開飛控**才生效'
                    f'（UAV-{sorted(reboot)}）。\n\n這次不上傳任務：請重開飛控、重新連線後再按「上傳飛控」。')
                self.statusBar().showMessage('航測建圖：參數已寫入，等待重開飛控後再上傳', 10000)
                return
            self._upload_mapping_missions(jobs)
            return
        text = mapping_mismatch_text(bad)
        logger.error(f'[航測] 讀回不符，擋下上傳：\n{text}')
        self.statusBar().showMessage('航測建圖：飛控參數與任務不符，已擋下上傳', 10000)
        if int(getattr(self, '_map_write_round', 0) or 0) >= 1:
            QMessageBox.critical(self, '航測建圖：寫入後讀回仍不符',
                                 '已寫入一次但讀回仍不符，上傳取消：\n\n' + text)
            return
        if not self._ask_write_mapping_params(bad):
            logger.info('[航測] 使用者不寫入參數，上傳取消')
            return
        # 對話框開著時規劃可能在背景完成 ⇒ 寫參數前再確認一次計畫沒換
        if self._abort_stale_mapping_upload():
            return
        written = self._write_mapping_params(bad)
        if not written:
            QMessageBox.warning(self, '航測建圖：無法自動修正',
                                '不符的項目都不能自動寫入，上傳取消：\n\n' + text)
            return
        self._map_write_round = 1
        self._map_reboot_pending = mapping_reboot_names(bad)
        self.statusBar().showMessage(f'航測建圖：已寫入 {len(written)} 個參數，重新讀回比對中…', 0)
        self._start_mapping_readback(jobs)

    def _ask_write_mapping_params(self, bad: dict) -> bool:
        """不符清單對話框：「寫入這些參數」（只有可寫的不符時才有）／「取消上傳」。回傳是否寫入。"""
        writable = any(not row['ok'] and row['writable'] for r in bad.values() for row in r.get('rows', []))
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning)
        box.setWindowTitle('航測建圖：飛控參數不符，已擋下上傳')
        box.setText('下列參數與這份任務要求的不一致（名稱／預期／實際）：')
        box.setInformativeText(mapping_mismatch_text(bad))
        write_btn = box.addButton('寫入這些參數', QMessageBox.ButtonRole.AcceptRole) if writable else None
        box.addButton('取消上傳', QMessageBox.ButtonRole.RejectRole)
        box.exec()
        return write_btn is not None and box.clickedButton() is write_btn

    def _write_mapping_params(self, bad: dict) -> list:
        """把可寫的不符項寫成 ``write_value``（相機參數 = 預期值；LOG_BITMASK = 讀回值 OR bit 12）（既有 set_param
        路徑）；``writable=False`` 的（空速包絡、在用的 SERVOn_FUNCTION、讀不到的 LOG_BITMASK）一律不寫。
        回傳 [(sysid, 名稱, 值), ...]。"""
        links = {int(j['sysid']): j['link'] for j in (getattr(self, '_map_upload_jobs', None) or [])}
        written = []
        for sysid, r in sorted(bad.items()):
            link = links.get(int(sysid))
            if link is None:
                continue
            for row in r.get('rows', []):
                if row['ok'] or not row['writable']:
                    continue
                value = float(row.get('write_value', row['expected']))
                ptype = _PTYPE_NAMES.get(int(row.get('ptype') or 0), 'REAL32')
                link.set_param(row['name'], value, ptype)
                written.append((int(sysid), row['name'], value))
                logger.warning(f'[航測] UAV-{sysid} 寫入 {row["name"]} = {value:g}'
                               f'（原本 {_fmt_param(row["actual"])}）')
        return written

    def _upload_mapping_missions(self, jobs):
        """讀回全部相符 ⇒ 逐架 upload_mission_items(verify=True)；勾了 tlog 就先開始記錄。"""
        params = {}
        try:
            params = self.vtol_recon_panel.get_params() or {}
        except Exception:                  # noqa: BLE001
            params = {}
        tlog_on = bool(params.get('hv_map_tlog'))
        from hv_dccpp.integration.aeroplan_adapter import expected_photo_count
        hv = self._hv_coordinator
        # R1 #10：讀回花了好幾秒 ⇒ 送出前再看一次飛控狀態（這期間有人解鎖／切 AUTO 就不換任務）
        states = [p for p in (link_flight_state_problem(j['link']) for j in jobs) if p]
        if states:
            logger.error(f'[航測] 送出前飛控狀態變了，取消上傳：{states}')
            QMessageBox.critical(self, '航測建圖：取消上傳',
                                 '讀回完成時下列飛控已不能換任務：\n  ' + '\n  '.join(states)
                                 + '\n\n這次沒有送出任何任務。')
            return
        offsets, off_src = harvest_takeoff_offsets(hv)
        info = getattr(self, '_map_readback_info', None) or {}
        self._map_photo_stats = {}
        lines, problems, sent = [], [], []
        for job in jobs:
            link, sysid, did, items = job['link'], int(job['sysid']), int(job['uav_id']), job['items']
            if tlog_on:
                try:
                    path = link.start_tlog(mapping_tlog_path(f'UAV-{sysid}'))
                    logger.info(f'[航測] UAV-{sysid} tlog 開始：{path}')
                except Exception as e:      # noqa: BLE001
                    problems.append(f'UAV-{sysid} tlog 開不了：{e}')
            self._write_pending_sysids.add(sysid)
            try:
                link.upload_mission_items(items, verify=True)
            except Exception as e:          # noqa: BLE001
                self._write_pending_sysids.discard(sysid)
                problems.append(f'UAV-{sysid} 送出寫入請求失敗：{e}')
                logger.error(f'[航測] UAV-{sysid} 送出寫入請求失敗：{e}', exc_info=True)
                continue
            n_on = sum(1 for it in items if int(it['command']) == 206 and float(it['param1']) > 0.0)
            n_off = sum(1 for it in items if int(it['command']) == 206 and float(it['param1']) == 0.0)
            n_exp = expected_photo_count(items)
            monitor, mon_err = build_strip_monitor(hv, did, items)
            if mon_err:
                problems.append(f'UAV-{sysid} 條帶監測器沒建（只顯示總張數，沒有逐條帶警報）：{mon_err}')
            self._map_photo_stats[sysid] = {'expected': n_exp, 'taken': 0, 'fb_missing': 0, 'resets': 0,
                                            'uav': did, 'link': link, 'monitor': monitor, 'alarms': [],
                                            'was_armed': False}
            off = offsets.get(did)
            off_txt = f'起飛時程 T+{off:.0f} s' if off is not None else '起飛時程：未知（不能依時程放行）'
            extra = ''.join(f'；{t}' for t in info.get(sysid, []))
            lines.append(f'UAV-{sysid} ← 機 {did}（{hv.id_map.get(did, "?")}）：{off_txt}、{len(items)} 項、'
                         f'206 開 {n_on} 筆／關 {n_off} 筆、預估 {n_exp} 張{extra}')
            sent.append((sysid, did))
        self._update_write_wp_enabled()
        self._refresh_mapping_live_status()
        # 飛行計畫快照：只放真的送出寫入的那幾架；先建在記憶體，第一個寫入成功的 ACK（_on_mission_uploaded）才存檔
        if sent:
            _sent = {(int(s), int(d)) for s, d in sent}
            _snap_problem = prepare_flown_plan_snapshot(
                self, hv, [j for j in jobs if (int(j['sysid']), int(j['uav_id'])) in _sent],
                takeoff_offsets=offsets, takeoff_source=off_src, readback_info=info,
                skipped=getattr(self, '_map_upload_skipped', None) or [],
                not_targeted=getattr(self, '_map_upload_not_targeted', None) or [], tlog=tlog_on)
            if _snap_problem:
                problems.append(_snap_problem)
        for p in problems:
            logger.error(f'[航測] {p}')
        logger.info('[航測] 已送出建圖任務：' + '；'.join(lines))
        entries, _src = record_release_schedule(self, hv, sent, mapping=True) if sent else ({}, off_src)
        skipped = getattr(self, '_map_upload_skipped', None) or []
        not_targeted = getattr(self, '_map_upload_not_targeted', None) or []
        text = ('飛控相機、空速與日誌參數讀回相符，已送出寫入（寫完會逐項讀回驗證，結果看狀態列）：\n  '
                + '\n  '.join(lines))
        text += '\n\n' + release_report_text(getattr(self, '_hv_release_schedule', None), entries, off_src,
                                               mapping=True)
        if skipped:
            text += f'\n\n沒有對應任務、沒上傳的連線：sysid {skipped}'
        if not_targeted:
            text += f'\n\n不是 HUD 指令對象、這次沒上傳的連線：sysid {not_targeted}'
        if problems:
            text += '\n\n' + '\n'.join(problems)
        self.statusBar().showMessage(f'航測建圖：已送出 {len(lines)} 架任務寫入', 10000)
        QMessageBox.information(self, '航測建圖：已送出上傳', text)

    def _on_camera_feedback(self, sysid: int, payload: dict):
        """CAMERA_FEEDBACK → 「已拍 N / 預估 M」。

        ⚠ 張數用 payload['new_photos']（以 img_idx 增量算，鏈路忙時多則訊息會合併）加總，**不是**訊號次數；
        payload['gap'] 是**沒收到的回授訊息**，照片照常計入觸發數，不是漏拍。
        只有建圖上傳過的機才記（不建圖的使用者畫面不變）。逐條帶歸屬交給條帶監測器（survey.strip_monitor）。
        """
        st = (getattr(self, '_map_photo_stats', None) or {}).get(int(sysid))
        if st is None:
            return
        n_new = int(payload.get('new_photos') or 0)
        st['taken'] += n_new
        gap = payload.get('gap')
        if gap:
            st['fb_missing'] += (int(gap[1]) - int(gap[0])) % 0x10000 + 1
        if payload.get('reset'):
            st['resets'] += 1
        mon = st.get('monitor')
        if mon is not None:
            mon.photos(n_new, time.monotonic(), reset=bool(payload.get('reset')))
            self._poll_strip_monitor(int(sysid))
        self._refresh_mapping_live_status()

    def _on_mapping_mission_current(self, sysid: int, mc: dict):
        """MISSION_CURRENT（正飛向的 NAV）→ 條帶監測器（STATUSTEXT 掉了時的備援）。"""
        st = (getattr(self, '_map_photo_stats', None) or {}).get(int(sysid))
        mon = st.get('monitor') if st else None
        if mon is None:
            return
        mon.mission_current(int(mc.get('seq', 0)), time.monotonic())
        self._poll_strip_monitor(int(sysid))

    def _on_mapping_item_started(self, sysid: int, seq: int, _cmd_type: str):
        """STATUSTEXT「Mission: N」（飛控開始執行第 N 項）→ 條帶監測器（206 開／關的精確時刻）。"""
        st = (getattr(self, '_map_photo_stats', None) or {}).get(int(sysid))
        mon = st.get('monitor') if st else None
        if mon is None:
            return
        mon.item_started(int(seq), time.monotonic())
        self._poll_strip_monitor(int(sysid))

    def _strip_monitor_on_frame(self, frame) -> None:
        """遙測幀的模式／解鎖 → 條帶監測器；解鎖→上鎖（落地）時收尾（沒飛到的條帶）。"""
        st = (getattr(self, '_map_photo_stats', None) or {}).get(int(getattr(frame, 'sysid', -1)))
        mon = st.get('monitor') if st else None
        if mon is None:
            return
        t = time.monotonic()
        armed = bool(getattr(frame, 'armed', False))
        mon.mode(str(getattr(frame, 'mode', '') or ''), armed, t)
        if st.get('was_armed') and not armed:
            self._show_strip_alarms(int(frame.sysid), mon.finish(t))
        st['was_armed'] = armed
        self._poll_strip_monitor(int(frame.sysid), t)

    def _poll_strip_monitor(self, sysid: int, t: float | None = None) -> None:
        st = (getattr(self, '_map_photo_stats', None) or {}).get(int(sysid))
        mon = st.get('monitor') if st else None
        if mon is None:
            return
        self._show_strip_alarms(int(sysid), mon.poll(time.monotonic() if t is None else t))

    def _show_strip_alarms(self, sysid: int, alarms: list) -> None:
        """條帶警報：log＋狀態列＋HUD 訊息＋面板（飛行中**不**跳強制回應的對話框）。"""
        if not alarms:
            return
        st = (getattr(self, '_map_photo_stats', None) or {}).get(int(sysid))
        if st is not None:
            st['alarms'].extend(alarms)
        for a in alarms:
            text = f'UAV-{sysid} {a.message}'
            if a.severity == 'critical':
                logger.error(f'[航測] {text}')
            else:
                logger.warning(f'[航測] {text}')
            self.statusBar().showMessage(('⚠⚠ ' if a.severity == 'critical' else '⚠ ') + text, 30000)
            hud = getattr(self, 'sitl_hud', None)
            if hud is not None:
                try:
                    sev = 2 if a.severity == 'critical' else 4
                    if hasattr(hud, 'on_uav_message'):      # 帶 sysid：群控標 UAVn、單機只在看那台時列
                        hud.on_uav_message(int(sysid), a.message, sev, kind='strip')
                    elif hasattr(hud, 'on_status_text'):
                        hud.on_status_text(sev, f'[條帶] {text}')
                except Exception:          # noqa: BLE001
                    logger.debug('[航測] HUD 顯示條帶警報失敗', exc_info=True)
        self._refresh_mapping_live_status()

    def _refresh_mapping_live_status(self):
        stats = getattr(self, '_map_photo_stats', None) or {}
        lines = []
        for sysid in sorted(stats):
            s = stats[sysid]
            line = f'UAV-{sysid}：已拍 {s["taken"]} / 預估 {s["expected"]}'
            if s['fb_missing']:
                line += (f'（回授訊息漏收 {s["fb_missing"]} 則：CAMERA_FEEDBACK 沒收到，'
                         f'照片照常計入觸發數，不是漏拍）')
            if s['resets']:
                line += f'（飛控重開機 {s["resets"]} 次，觸發計數從頭算）'
            mon = s.get('monitor')
            if mon is not None:
                line += f'；{mon.progress_text()}'
            for a in (s.get('alarms') or [])[-3:]:
                line += f'\n    {"⚠⚠" if a.severity == "critical" else "⚠"} {a.message}'
            link = s.get('link')
            if link is not None and hasattr(link, 'tlog_status'):
                try:
                    err = (link.tlog_status() or {}).get('error')
                except Exception:          # noqa: BLE001
                    err = None
                if err:
                    line += f'（tlog 錯誤：{err}）'
            lines.append(line)
        panel = getattr(self, 'vtol_recon_panel', None)
        if panel is not None and hasattr(panel, 'set_mapping_live_status'):
            panel.set_mapping_live_status('\n'.join(lines))

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
        # 起降場由使用者決定（2026-10-08）：沒選就不規劃。以前會默默用作業區質心，
        # 結果起降坪在作業區正中央、三台的進場與返場全擠在中間（2026-10-07 回報）。
        if getattr(self, 'home_point', None) is None:
            QMessageBox.warning(
                self, "尚未選擇起降場",
                "請先在「1. 偵察區域」按「從地圖選起降場」，再到地圖上點出起降場位置。\n"
                "各機的起降坪會排在你點的位置，任務結束也降回那裡。")
            panel.set_plan_status("尚未選擇起降場")
            return

        use_hv = bool(params.get('hv_dccpp'))
        logger.info(f"[VTOL 偵察] 規劃演算法 = {params.get('planner', 'dccpp')}"
                    f"（{'背景執行緒' if use_hv else '同步'}）")
        panel.set_plan_status(
            ("HARVEST 規劃中 …（大情境需數十秒）" if use_hv else "DCCPP 規劃中 …"))
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
            hdmap_preview_reset(self, '規劃未產生結果')
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
                from ui.widgets.cesium_map_widget import swarm_drone_colors
                _ap = result.get('assembled_paths') or {}
                cesium.draw_vtol_swarm_paths(
                    _viz.to_cesium_data(
                        _ap, float(params.get('altitude', 100.0)),
                        # 與 3D 圖例同一份配色，否則條帶顏色和圖例對不上
                        colors=swarm_drone_colors(
                            (uid, getattr(ap, 'cls', None)) for uid, ap in _ap.items()))
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
        # 規劃風（有效值：規劃器生效的那一個，不是面板現在的值）。一般模式原本沒有任何一行寫「用了多少風」，
        #   而面板預設 0 m/s ⇒ 沒輸入就是無風規劃、畫面上看不出來（評估_2026-10-20 §5.1）。DCCPP 不吃風 ⇒ 明說。
        _hv = _win_attr(self, '_hv_coordinator')
        if _hv is not None and getattr(_hv, 'last_result', None) is result:
            wind_txt = harvest_wind_text(harvest_plan_wind(_hv))
        else:
            wind_txt = '規劃風：DCCPP 不考慮風（無風規劃）'
        # 網格風重估（規劃完成時已算好；全文在風場狀態列）：一句話放進摘要，沒有就不加
        _gw = grid_wind_estimate_digest(self)
        summary = (
            f"規劃完成：{len(rows)}/{asked} 架{cls_txt} ｜ 總航程 "
            f"{sum(r[2] for r in rows):.2f} km ｜ makespan {makespan / 60.0:.1f} min"
            f"{idle_txt} ｜ {wind_txt}" + (f" ｜ {_gw}" if _gw else '')
        )
        panel.set_plan_result(rows, summary)
        logger.info(f'[VTOL 偵察] {summary}')

        # ── 航測建圖：有效值與上傳閘門回填到「航測建圖」群組（只在這份結果是建圖模式規劃的時候）──
        if (_hv is not None and getattr(_hv, 'last_result', None) is result
                and getattr(_hv, 'mapping_mode', None) is not None):
            self._show_mapping_plan_status(_hv)
            # 預計成果（規劃預測）：閘門狀態顯示完才在背景算（用跟上傳閘門同一份有效輸入）；失敗不影響規劃結果
            try:
                self._start_hdmap_preview(_hv)
            except Exception as e:      # noqa: BLE001
                logger.error(f'[高精地圖預覽] 無法開始計算：{e}', exc_info=True)
                if hasattr(panel, 'set_hdmap_preview_state'):
                    panel.set_hdmap_preview_state('failed')
                    panel.set_hdmap_preview_status(f'預計成果無法開始計算：{e}')

    # ─────────────────────────────────────────────────────────────────
    # 高精地圖預計成果（規劃預測，非實際成果）
    #   規劃完成 → HdmapPreviewWorker（survey.preview.compute_hdmap_preview）→ 地圖圖層＋偵察頁按鈕＋對話框
    # ─────────────────────────────────────────────────────────────────
    def _start_hdmap_preview(self, hv):
        """建圖計畫顯示完成後：在背景算預計成果（stamp＝這份計畫；回來時計畫換了就丟）。"""
        from ui.workers.hdmap_preview_worker import HdmapPreviewWorker
        hdmap_preview_reset(self, '重新計算預計成果')       # 上一趟（若有）放棄、圖層清掉
        sc = _win_attr(self, '_hv_swarm_coordinator')
        stamp = (hv, getattr(hv, 'last_result', None))
        # parent=None：視窗關閉時由 closeEvent 寄放（在還在跑的 QThread 上解構是 qFatal）
        w = HdmapPreviewWorker(hv, sc, stamp=stamp, parent=None)
        w.finished_ok.connect(self._on_hdmap_preview_done)
        w.failed.connect(self._on_hdmap_preview_failed)
        self._hdmap_preview_worker = w
        panel = _win_attr(self, 'vtol_recon_panel')
        if panel is not None and hasattr(panel, 'set_hdmap_preview_state'):
            panel.set_hdmap_preview_state('computing')
        logger.info('[高精地圖預覽] 開始計算預計成果（規劃預測）')
        w.start()

    def _hdmap_preview_signal_worker(self, *, result=None, error=None):
        """信號是不是目前這個 worker 發的（排隊中的舊信號在換 worker／重設之後才送到 ⇒ 丟）。"""
        w = _win_attr(self, '_hdmap_preview_worker')
        if w is None:
            return None
        if result is not None and getattr(w, 'result', None) is not result:
            return None
        if error is not None and getattr(w, 'error_text', None) != error:
            return None
        return w

    def _on_hdmap_preview_done(self, preview):
        """背景算完：計畫還是同一份 ⇒ 存起來、畫圖層（有勾的話）、偵察頁按鈕可按、狀態列。"""
        w = self._hdmap_preview_signal_worker(result=preview)
        if w is None:
            logger.info('[高精地圖預覽] 收到舊的計算結果（已重設或換了一趟），丟掉')
            return
        why = hdmap_preview_stale_reason(self, getattr(w, 'stamp', None))
        if why:
            logger.warning(f'[高精地圖預覽] 結果不採用：{why}')
            hdmap_preview_reset(self, why)
            return
        if not bool(getattr(preview, 'ok', False)):        # worker 已把 ok=False 轉成 failed；這裡是第二道
            self._hdmap_preview_show_failure(str(getattr(preview, 'reason', None) or '預覽沒有產生結果'))
            return
        self._hdmap_preview_retire_worker()
        self._hdmap_preview = preview
        self._hdmap_preview_payload = None
        self._hdmap_preview_drawn = False
        self._hdmap_preview_draw_error = None
        self._hdmap_preview_error = None
        summary = hdmap_preview_summary(preview)
        panel = _win_attr(self, 'vtol_recon_panel')
        if panel is not None and hasattr(panel, 'set_hdmap_preview_state'):
            panel.set_hdmap_preview_state('ready')
        layer_on = True
        if panel is not None and hasattr(panel, 'hdmap_preview_layer_enabled'):
            layer_on = bool(panel.hdmap_preview_layer_enabled())
        if layer_on:
            self._hdmap_preview_draw()
        # 狀態字在畫完之後才寫：圖層畫不出來要接在摘要後面說（不能被成功的摘要蓋掉）
        hdmap_preview_show_status(self, summary)
        logger.info(f'[高精地圖預覽] {summary}')
        # 狀態列只在空著（或是可蓋掉的訊息）時才用：HARVEST 的重點（loud note、未覆蓋區域）不能被這則蓋掉
        err = _win_attr(self, '_hdmap_preview_draw_error')
        hdmap_preview_status_message(
            self, summary + (f'；⚠ 地圖圖層畫不出來：{err}' if err else '')
            + ' ⇒ 按偵察頁「預覽高精地圖成果…」看細節', 15000)

    def _on_hdmap_preview_failed(self, msg: str):
        """背景算不出來：偵察頁顯示原因（不跳對話框：這是輔助功能，不擋上傳）。"""
        w = self._hdmap_preview_signal_worker(error=msg)
        if w is None:
            logger.info(f'[高精地圖預覽] 舊的一趟失敗（已重設或換了一趟），不顯示：{msg}')
            return
        why = hdmap_preview_stale_reason(self, getattr(w, 'stamp', None))
        if why:
            logger.info(f'[高精地圖預覽] 失敗訊息不顯示（{why}）：{msg}')
            hdmap_preview_reset(self, why)
            return
        self._hdmap_preview_show_failure(msg)

    def _hdmap_preview_retire_worker(self):
        """結果（或失敗）已經收到：不再需要這個 worker。⚠ 信號是排隊送來的，這一刻 run() 可能還沒 return
        ⇒ 不能直接丟參考（在還在跑的 QThread 上解構是 qFatal），交給 _hdmap_preview_abandon 寄放／deleteLater。"""
        w = _win_attr(self, '_hdmap_preview_worker')
        self._hdmap_preview_worker = None
        if w is not None:
            _hdmap_preview_abandon(w, retired=True)

    def _hdmap_preview_show_failure(self, msg: str):
        self._hdmap_preview_retire_worker()
        self._hdmap_preview = None
        self._hdmap_preview_error = str(msg)            # 工具選單開對話框時要說得出「上一趟為什麼沒有」
        logger.error(f'[高精地圖預覽] 無法產生預計成果：{msg}')
        panel = _win_attr(self, 'vtol_recon_panel')
        if panel is not None and hasattr(panel, 'set_hdmap_preview_state'):
            panel.set_hdmap_preview_state('failed')
            panel.set_hdmap_preview_status(f'預計成果無法產生：{msg}')
        hdmap_preview_status_message(self, f'預計成果無法產生：{msg}', 10000)

    def _hdmap_preview_draw(self) -> bool:
        """把目前的預計成果畫到 2D／3D（圖層 id = HDMAP_PREVIEW_LAYER_ID）。成功（至少一邊 ok）回 True。

        任何一邊失敗（丟例外、或回 ok=False）⇒ 原因記在 ``_hdmap_preview_draw_error``（狀態字接在摘要後面說；
        再勾「地圖顯示預計成果」會整份重送一次）。``_hdmap_preview_drawn`` 照樣設 True：另一邊可能畫上了，重設時要清。"""
        preview = _win_attr(self, '_hdmap_preview')
        mwid = _win_attr(self, 'map_widget')
        if preview is None or mwid is None or not hasattr(mwid, 'set_hdmap_preview'):
            return False
        try:
            payload = _win_attr(self, '_hdmap_preview_payload')
            if payload is None:
                payload = preview.to_js_payload()
                self._hdmap_preview_payload = payload
            self._hdmap_preview_drawn = True            # 先記：就算一邊失敗，另一邊可能畫上了 ⇒ 重設時要清
            res = mwid.set_hdmap_preview(HDMAP_PREVIEW_LAYER_ID, payload)
        except Exception as e:                          # noqa: BLE001 - 畫不出來要說，但不影響計畫
            logger.error(f'[高精地圖預覽] 地圖圖層失敗：{e}', exc_info=True)
            self._hdmap_preview_draw_error = f'{type(e).__name__}: {e}'
            return False
        if isinstance(res, dict) and 'ok' in res:
            parts = {'': res}
        elif isinstance(res, dict):
            parts = {k: v for k, v in res.items() if isinstance(v, dict)}
        else:
            parts = {}
        bad = {k: v for k, v in parts.items() if not v.get('ok')}
        if bad:
            logger.warning(f'[高精地圖預覽] 地圖圖層部分失敗：{res}')
            self._hdmap_preview_draw_error = '；'.join(
                (f'{k.upper()}：' if k else '') + str(v.get('error') or '頁面回報失敗') for k, v in bad.items())
        elif not parts:
            self._hdmap_preview_draw_error = f'地圖沒有回報結果（{res!r}）'
        else:
            self._hdmap_preview_draw_error = None
        if hasattr(mwid, 'set_hdmap_preview_visible'):
            try:
                mwid.set_hdmap_preview_visible(HDMAP_PREVIEW_LAYER_ID, True)
            except Exception as e:                      # noqa: BLE001
                logger.warning(f'[高精地圖預覽] 圖層顯示切換失敗：{e}')
        return bool(parts) and len(bad) < len(parts)

    def _on_hdmap_preview_layer_toggled(self, on: bool):
        """偵察頁「地圖顯示預計成果」：勾 ⇒ 沒畫過（或上次畫失敗）就畫、畫過就顯示；取消 ⇒ 隱藏（資料留著）。"""
        if _win_attr(self, '_hdmap_preview') is None:
            return
        mwid = _win_attr(self, 'map_widget')
        if on and (not _win_attr(self, '_hdmap_preview_drawn', False)
                   or _win_attr(self, '_hdmap_preview_draw_error')):
            self._hdmap_preview_draw()
            hdmap_preview_show_status(self)             # 重試的結果（成功就把錯誤字拿掉）
            return
        if mwid is not None and hasattr(mwid, 'set_hdmap_preview_visible'):
            try:
                mwid.set_hdmap_preview_visible(HDMAP_PREVIEW_LAYER_ID, bool(on))
            except Exception as e:                      # noqa: BLE001
                logger.warning(f'[高精地圖預覽] 圖層顯示切換失敗：{e}')

    def _on_hdmap_preview_open(self, *_a):
        """偵察頁「預覽高精地圖成果…」／工具選單：開非模態對話框（留參考，避免被 GC 收掉）。"""
        preview = _win_attr(self, '_hdmap_preview')
        if preview is None:
            QMessageBox.information(self, '預覽高精地圖成果', hdmap_preview_unavailable_reason(self))
            return
        old = _win_attr(self, '_hdmap_preview_dialog')
        try:
            if old is not None and getattr(old, 'preview', None) is preview:
                old.show()
                old.raise_()
                old.activateWindow()
                return
        except RuntimeError:
            pass                                        # 舊對話框已被 Qt 刪掉
        try:
            from ui.dialogs.hdmap_preview_dialog import HdmapPreviewDialog
            dlg = HdmapPreviewDialog(preview, parent=self)
        except Exception as e:                          # noqa: BLE001
            logger.error(f'[高精地圖預覽] 對話框開不起來：{e}', exc_info=True)
            QMessageBox.warning(self, '預覽高精地圖成果', f'對話框無法開啟：{e}')
            return
        if old is not None:
            try:
                old.close()
                old.deleteLater()                        # 上一份預覽的視窗：parent 是本視窗，不刪會一直掛著
            except RuntimeError:
                pass
        self._hdmap_preview_dialog = dlg                 # 非模態：留參考
        dlg.show()

    def _show_mapping_plan_status(self, hv):
        """建圖有效值（觸發間隔、引入段、逐構型條帶間距與偏流、起飛時程）＋ 飛前曝光驗證（視數）＋ 上傳閘門 → 偵察頁；
        不可上傳時大聲說。"""
        _sc = getattr(self, '_hv_swarm_coordinator', None)
        gate = hv_upload_gate(hv, _sc)
        cc = getattr(hv, 'camera_cfg', None)
        parts = []
        if cc is not None:
            parts.append(f'CAM1_INTRVAL_MIN {cc.interval_min_s:.2f} s（規劃拍照率 {1.0 / cc.interval_min_s:.3f} Hz）、'
                         f'引入段 {float(hv.mapping_mode.lead_in_m):g} m、後端 {cc.backend}'
                         + ('（只適用 SITL；實機請用 relay）' if cc.backend == 'servo' else ''))
        for c, m in sorted(((getattr(hv, 'mapping_effective', None) or {}).get('per_class') or {}).items()):
            if m:
                parts.append(f'構型 {c}：條帶間距 {float(m.get("s_m", float("nan"))):.1f} m、'
                             f'偏流 {float(m.get("crab_deg", 0.0)):.1f}°')
        rule = (getattr(hv, 'mapping_effective', None) or {}).get('cross_rule')
        if rule:
            flown = rule.get('flown_cross_strips_by_uav') or {}
            parts.append('交叉航帶：鄰機空域 ' + ('已明確放行' if rule.get('allow_neighbor_airspace') else '禁止（預設）')
                         + '；排進 ' + ('、'.join(f'{u} {n} 條' for u, n in flown.items()) or '0 條')
                         + (f'；因鄰機空域不排 {rule["forbidden_sets"]} 組' if rule.get('forbidden_sets') else ''))
        try:
            offs, _src = harvest_takeoff_offsets(hv)
        except Exception as e:      # noqa: BLE001
            offs, _src = {}, f'取不到（{e}）'
        if offs:
            spread = max(offs.values()) - min(offs.values())
            parts.append('起飛時程：' + '、'.join(f'機 {d} T+{o:.0f} s' for d, o in sorted(offs.items(),
                                                                                    key=lambda kv: (kv[1], kv[0])))
                         + ('（不同 ⇒ 上傳後用 HUD 的「依時程放行」，一鍵起飛會被擋）'
                            if spread > TAKEOFF_STAGGER_TOL_S else ''))
        # B3：飛前曝光驗證（視數）。橫幅只顯示前 3 則 loud note ⇒ 摘要一定要寫在這裡
        _xs = getattr(_sc, 'hv_exposure_summary', None) if getattr(_sc, 'hv_coordinator', None) is hv else None
        parts.extend(mapping_exposure_lines(_xs))
        # HARVEST 的重點（loud note、未覆蓋區域）：成功時原本只在狀態列 15 s，之後會被別的訊息蓋掉 ⇒ 這裡留一份
        parts.extend(harvest_report_lines(_win_attr(self, '_hv_last_report')))
        if gate['uploadable']:
            parts.append('建圖閘門：可上傳')
        else:
            parts.append('建圖閘門：不可上傳（' + '；'.join(gate['reasons']) + '）')
        text = '\n'.join(parts)
        logger.info(f'[航測] 規劃結果：{text}')
        panel = getattr(self, 'vtol_recon_panel', None)
        if panel is not None and hasattr(panel, 'set_mapping_plan_status'):
            panel.set_mapping_plan_status(text, ok=gate['uploadable'])
        if not gate['uploadable']:
            QMessageBox.warning(self, '航測建圖：這份計畫不能上傳',
                                '這份計畫不能上傳：\n  ' + '\n  '.join(gate['reasons'])
                                + '\n\n（preflight 的 GO_WITH_WARNINGS 不代表整區都有人拍，'
                                  '也不代表通過時空去衝突／能量驗證）')

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
        # ⚠ 只取**相機真的開著**的航點。論文 §2.3 定義 coverage 只在 Operation
        #   Path 上有效，把全部航點拿去驗算等於把轉場與爬升也算成掃過、覆蓋率虛高。
        #
        # ⚠⚠⚠ 但判準必須是 `sensor_on`，**不是** `segment_type == OPERATION`。
        #   兩個規劃器的段落標籤慣例不同，實測同一份幾何：
        #       DCCPP  73 個航點 → OPERATION 14 個（19.2%）
        #       HARVEST 306 個   → OPERATION 16 個（**5.2%**，170 個標成 TRANSFER）
        #   用 OPERATION 篩選驗算，HARVEST 從 **100.00%** 掉到 **75.10%**，
        #   而 DCCPP 只從 76.68% 掉到 74.04% ⇒ **同一把尺對兩個規劃器不對等**。
        #   使用者因此在真實機場幾何上量到 66.07%，差點據此判定演算法有問題。
        #   相機開沒開才是語意上正確且與規劃器無關的判準（vendored 修改 #5 把
        #   `sensor_on` 帶進 assembled waypoints）。
        #   ⚠ 舊規劃器不帶這個欄位 ⇒ 沒有任何一點有 `sensor_on` 時退回 OPERATION，
        #     並在 log 說明用了哪一種判準（靜默退回會讓人以為兩邊是同一把尺量的）。
        # ⚠⚠⚠ **一定要量會被飛出去的那條路徑**（assembled_paths＝縮點後），
        #   不可以改讀規劃器原生的未縮點計畫。我一度那樣改，但那會把一個**真的**
        #   缺陷變成看不見的：
        #     `deconflict.py:934` 插入的延遲盤旋，其 ORBIT 航點是條帶終點的複本；
        #     `ardupilot.py:394` 直線段刻意不輸出自己的終點（指望那個複本）；
        #     `ardupilot.py:361` 卻在轉 loiter 指令時把複本座標改成盤旋**圓心**。
        #   ⇒ 上傳序列變成「條帶西端 → 圓心 → 條帶東端」，條帶被斜掉 r_orb=120 m，
        #     實測 (c) 格漏掃 87 851 m²（0.4393%），而原生計畫是精確的 100.0000%。
        #   量原生計畫會回報 100% ⇒ 假綠燈。**驗算的對象必須是產物，不是意圖。**
        # ⚠⚠⚠ 成像航點要當成**腿**量，不是把航點連起來（2026-10-09 使用者問「為何 HARVEST 覆蓋率驗算
        #   都那麼差」）：縮點後每條條帶只剩起點帶 `sensor_on`、終點是別的標籤 ⇒ 只取 `sensor_on`
        #   的航點＝每條條帶一個孤立點，驗算再把這些點連成一條跨過整區的鋸齒線。同一份計畫（4 角點、
        #   1A+1B）：GUI 75.06%（10 個「成像航點」）、實際飛出去的路徑 **100.00%**（3585/3585 格）。
        #   `imaging_path` 依「腿歸出發點」取 sensor_on 航點＋它的下一個航點，不相連的條帶以 None 斷開。
        from mission.path_segments import imaging_path
        aps = (result.get('assembled_paths') or {})
        paths = {}
        criterion = ''
        for uid, ap in aps.items():
            paths[int(uid)], criterion = imaging_path(ap)
        n_pts = sum(1 for v in paths.values() for p in v if p is not None)
        n_runs = sum(1 for v in paths.values() if any(p is not None for p in v)) + \
            sum(1 for v in paths.values() for p in v if p is None)
        logger.info(f'[VTOL 偵察] 覆蓋率驗算判準：{criterion}；'
                    f'取到 {n_runs} 段成像線（{n_pts} 個點）')
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
            # 「帶入規劃風」：目前 HARVEST 計畫的生效風（否則偵察頁風格子），對話框預設勾選並寫明值與方向
            plan_wind = sitl_plan_wind_offer(self)
            dlg_result = SITLLaunchDialog.get_config(
                vehicle=vehicle, count=count, parent=self, plan_wind=plan_wind,
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

            # 模擬風：對話框勾了才帶（sitl_launcher 寫 sim_wind.parm 並讀回比對）；沒帶而規劃風 > 0 ⇒ 大聲說不一致
            sim_wind = dlg_result.get('sim_wind')
            if sim_wind is not None:
                logger.info(f'[SITL] 模擬風：帶入規劃風 {float(sim_wind[0]):g} m/s 來自 {float(sim_wind[1]):g}°'
                            f'（{dlg_result.get("sim_wind_source", "")}）')
            elif plan_wind is not None and float(plan_wind['speed_mps']) >= _PLAN_WIND_CALM_MPS:
                logger.warning(f'[SITL] ⚠ 模擬風：未帶入 ⇒ SITL 無風，但{plan_wind["text"]}'
                               f'（{plan_wind["source"]}）——計畫與模擬不一致')
            else:
                logger.info('[SITL] 模擬風：無')

            results = self._sitl_launcher.start_multi(
                vehicle=vehicle, count=count,
                lat=lat, lon=lon, alt=0.0, heading=spawn_heading,
                spacing_deg=0.0008,
                # ⚠ 共用起降場時四台的起飛點是同一個座標 ⇒ 先排成一列，
                #   否則四台 SITL 會生在同一點上互相重疊（見 spread_colocated_homes）。
                homes=(spread_colocated_homes(per_uav_homes, logger_=logger)
                       if per_uav_homes else None),
                instance_configs=instance_configs,
                auto_firewall=auto_firewall,
                sim_wind=sim_wind,
            )
            first_conn = results[0][1]
            self.sitl_hud.on_sitl_launched(f'{vehicle} x{count}', first_conn)
            logger.info(f'[SITL] 內建 {vehicle} x{count} 已啟動')
            self.statusBar().showMessage(
                f'SITL {vehicle} x{count} 已啟動｜模擬風 '
                + (f'{float(sim_wind[0]):g} m/s 來自 {float(sim_wind[1]):g}°（帶入規劃風）'
                   if sim_wind is not None else '無'), 10000)

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
        # 航測處理對話框的背景工作（處理、ODM 上傳／等待／下載、接回、取消遠端任務）：先**只問**「停止並關閉／不關」。
        # 它們的 parent 是對話框（對話框的 parent 是本視窗）⇒ 不處理就是在還在跑的 QThread 上解構（qFatal）。
        # 放在 STOT 之前：這裡選「不關」沒有任何副作用；STOT 的「中止航次」不能復原，不可以先中止了航次，
        # 再因為這裡選「不關」而留下視窗。真的停止在最後（關閉定案、event.accept() 前）才做：選了「是」之後
        # 被 STOT 的「不要關閉」或「未儲存的變更」否決時，本機工作（例如還沒提交、不能接續的上傳）不能已經停了（審查）。
        try:
            if survey_close_ask(self) is False:
                event.ignore()
                return
        except Exception as e:      # noqa: BLE001 — 這段壞了也要能關視窗，但要留下紀錄
            logger.error(f'[航測處理] 關閉時詢問背景工作失敗：{type(e).__name__}: {e}', exc_info=True)
        # ROS2 STOT 航次還在跑時，要由使用者決定「中止 / 留著繼續跑 / 不關」。
        # 靜默殺掉會製造不可歸因的失敗；靜默留著則會變成沒人監看的孤兒航次。
        try:
            if self._stot_close_decision() is False:
                event.ignore()
                return
        except Exception:
            pass
        # 遙測合幀：已排的 flush 不可以在 widget 開始銷毀後還去畫 HUD／地圖
        _forget_tele_state(self, None)
        # HARVEST 規劃執行緒：不能殺、不能丟著不管，只能寄放（見 _hv_park_running_worker）。
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
        # 高精地圖預計成果的背景執行緒：算圖沒有取消點 ⇒ 放棄結果、等 3 s，還沒完就寄放（同 HARVEST）
        try:
            _pw = _win_attr(self, '_hdmap_preview_worker')
            if _pw is not None:
                self._hdmap_preview_worker = None
                _hdmap_preview_abandon(_pw, closing=True)
        except Exception:
            pass
        # 找平坦位置的背景執行緒：下載/掃描停不下來，放棄結果並等它跑完（在還在跑的 QThread 上解構是 qFatal）
        try:
            _fw = getattr(self, '_flat_worker', None)
            if _fw is not None and _fw.isRunning():
                _fw.stop()
                _fw.wait(30000)
        except Exception:
            pass
        # 氣象網格／分層風／CWA 測站的背景執行緒：同上（urllib 沒有取消點，逾時 20 s）
        for _name in ('_wind_worker', '_wind_profile_worker', '_cwa_worker'):
            try:
                _ww = getattr(self, _name, None)
                if _ww is not None and _ww.isRunning():
                    _ww.stop()
                    _ww.wait(30000)
            except Exception:
                pass
        # 依時程放行還有飛機在倒數 ⇒ 關視窗就不會再放行了，取消並寫 log（不留一個看不到的計時器）
        try:
            _rel = getattr(self, '_release', None)
            if _rel is not None and _rel.active:
                _rel.cancel()
                logger.warning('[放行] 視窗關閉：還在倒數的飛機已取消放行（不會起飛）')
        except Exception:
            pass
        # 確保 SITL 執行緒安全停止
        for _link in self._sitl_links:
            _stop_link_tlog(_link)
            try:
                _link.stop()
                _link.wait(2000)
            except Exception:
                pass
        self._sitl_links.clear()
        # 航測建圖的參數讀回執行緒：鏈路一停 read_params 就醒（≤ 0.25 s），等它結束再讓 Qt 解構
        #   （在還在跑的 QThread 上解構是 qFatal）
        try:
            _mw = getattr(self, '_map_readback_worker', None)
            if _mw is not None:
                try:
                    _mw.done.disconnect()
                except (TypeError, RuntimeError):
                    pass
                if _mw.isRunning():
                    _mw.wait(15000)
        except Exception:
            pass
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
        # 關閉已定案（上面所有可以否決的提問都過了）⇒ 現在才停航測處理／ODM 的背景工作
        try:
            survey_close_finish(self)
        except Exception as e:      # noqa: BLE001 — 這段壞了也要能關視窗，但要留下紀錄
            logger.error(f'[航測處理] 關閉時停止背景工作失敗：{type(e).__name__}: {e}', exc_info=True)

        logger.info("應用程式關閉")
        event.accept()