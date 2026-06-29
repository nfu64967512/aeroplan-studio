"""
AeroPlan Studio - 主程式入口
Collaborative UAV Mission Planning Suite

整合 2D/3D 地圖、覆蓋路徑規劃、DCCPP 多機協同、Dubins 軌跡、
ArduPilot SITL 多機模擬與 Mission Planner 相容 MAVLink 匯出的一站式平台。

使用方式
--------
    python main.py                      # 啟動 GUI（預設）
    python main.py --vehicle surfer     # 指定飛行器設定檔
    python main.py --log-level DEBUG    # 指定日誌等級
    python main.py --config my.yaml     # 自訂全域設定檔

典型流程
--------
    1. 地圖上左鍵點選 ≥3 個角點定義作業區域
    2. 右側面板選擇飛行器 / 演算法 / 高度 / 速度 / 重疊率
    3. 按 Enter 或「預覽」產生路徑，可切換 2D↔3D Cesium 檢視
    4. DCCPP 分頁：設定無人機數量後點「DCCPP 最佳化」
    5. SITL 分頁：按「🚀 啟動」會依 DCCPP 起飛點生成 N 台 ArduPilot SITL
    6. 按「上傳任務」將每架 UAV 對應路徑分別上傳到對應 SITL 實例
    7. Ctrl+E 匯出 QGC WPL 110 航點檔供 Mission Planner / QGC 使用
"""

import sys
import argparse
from pathlib import Path

# 添加專案根目錄到路徑
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# ── 強制標準輸出/錯誤為 UTF-8 ────────────────────────────────────────
# Windows 預設 console 編碼為 cp950（繁中）。本 App 各處 print() 含 emoji
# （Cesium JS console 回報、地圖點擊 log 等）；當 stdout 被導向檔案、或在
# cp950 終端執行時，輸出非 cp950 字元會拋 UnicodeEncodeError。若該例外
# 發生在 Qt callback（如 ClickCapturePage.javaScriptConsoleMessage）內，
# PyQt6 會直接 abort（程式以 exit code 9 崩潰）。提早於進入點把標準輸出
# 重設為 UTF-8 + errors='replace'，根治整類編碼崩潰（含子行程繼承）。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding='utf-8', errors='replace')  # type: ignore[union-attr]
    except (AttributeError, ValueError):
        pass  # 已被包裝/重導向、無 reconfigure 的串流：略過即可

from config import init_settings
from utils import setup_logger


def parse_arguments():
    """解析命令列參數"""
    parser = argparse.ArgumentParser(
        prog='aeroplan-studio',
        description='AeroPlan Studio — Collaborative UAV Mission Planning Suite',
        epilog='範例: python main.py --vehicle surfer --log-level DEBUG',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='配置文件路徑'
    )
    
    parser.add_argument(
        '--log-level',
        type=str,
        default='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
        help='日誌等級'
    )
    
    parser.add_argument(
        '--no-ui',
        action='store_true',
        help='無界面模式（僅命令列）'
    )
    
    parser.add_argument(
        '--vehicle',
        type=str,
        default='generic_quadcopter',
        help='飛行器配置檔案名稱'
    )

    parser.add_argument(
        '--shell',
        type=str,
        default='legacy',
        choices=['legacy', 'milstd'],
        help=(
            "UI 外殼："
            "'legacy' 使用既有 MainWindow； "
            "'milstd' 啟動 MIL-STD-1472H 合規之 AeroPlanMainWindow 框架"
        ),
    )

    # AeroPlan Studio Design System 新版主題（預設）vs 舊版 tactical_theme（除錯 fallback）
    parser.add_argument(
        '--legacy-theme',
        action='store_true',
        help='使用舊版 tactical_theme（新版套用失敗時的 fallback；正常情況不需指定）',
    )

    return parser.parse_args()


def initialize_system(args):
    """初始化系統"""
    # 初始化日誌
    logger = setup_logger(
        name='AeroPlanStudio',
        level=args.log_level,
        log_to_file=True,
        log_to_console=True
    )

    logger.info("=" * 60)
    logger.info("AeroPlan Studio — Collaborative UAV Mission Planning Suite")
    logger.info("版本: 2.8.1")
    logger.info("=" * 60)
    
    # 初始化配置
    settings = init_settings(args.config)
    logger.info(f"配置文件: {args.config or '使用預設配置'}")
    
    # 載入飛行器配置
    from utils.file_io import read_yaml
    import os
    
    vehicle_config_path = os.path.join(
        settings.paths.config_dir,
        'vehicle_profiles.yaml'
    )
    
    if os.path.exists(vehicle_config_path):
        vehicle_profiles = read_yaml(vehicle_config_path)
        logger.info(f"飛行器配置載入成功: {len(vehicle_profiles or {})} 種類型")
    else:
        logger.warning(f"飛行器配置文件不存在: {vehicle_config_path}")
        vehicle_profiles = None

    # ── 清理暫存 HTML 快取 (Folium / Cesium / polygon_editor) ──
    # 避免每次啟動累積 tmp*.html 於專案根目錄
    try:
        from utils.temp_cache import clear_temp_html_dir, purge_legacy_temp_html_in_root
        purge_legacy_temp_html_in_root()   # 移除舊版散落在根目錄的 tmp*.html
        clear_temp_html_dir()              # 清空 data/tmp_maps/
    except Exception as e:
        logger.warning(f"暫存 HTML 清理失敗: {e}")

    return logger, settings, vehicle_profiles


def run_gui_mode(logger, settings, vehicle_profiles, args=None):
    """運行GUI模式

    Parameters
    ----------
    args : argparse.Namespace | None
        命令列參數；若提供且 args.shell == 'milstd'，啟動新版
        MIL-STD-1472H 合規主視窗 (AeroPlanMainWindow)，否則沿用既有 MainWindow。
    """
    try:
        logger.info("啟動 GUI 模式...")
        
        # 檢查PyQt6是否可用
        try:
            from PyQt6.QtWidgets import QApplication
            from PyQt6.QtCore import Qt
            
            # 重要：在創建 QApplication 之前導入 WebEngine
            # 這是 PyQt6 WebEngine 的已知要求
            try:
                from PyQt6.QtWebEngineWidgets import QWebEngineView  # noqa: F401 — 可用性偵測
                logger.info("QtWebEngineWidgets 載入成功")
            except ImportError:
                logger.warning("QtWebEngineWidgets 未安裝，地圖功能可能受限")
                
        except ImportError:
            logger.error("PyQt6 未安裝，無法啟動 GUI 模式")
            logger.info("請安裝: pip install PyQt6 PyQt6-WebEngine")
            return 1
        
        # 設置 OpenGL 共享上下文（WebEngine 需要）
        try:
            QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
        except Exception as e:
            logger.warning(f"設置 OpenGL 共享上下文失敗: {e}")

        # ── 高 DPI 支援：4K 螢幕字型/圖示等比例放大，避免視覺過小 ──
        # Qt 6 預設已啟用 HiDPI，只需設定 rounding policy 讓比例連續而非階梯式
        try:
            from PyQt6.QtCore import Qt as _QtCore
            QApplication.setHighDpiScaleFactorRoundingPolicy(
                _QtCore.HighDpiScaleFactorRoundingPolicy.PassThrough
            )
        except Exception as e:
            logger.warning(f"HiDPI rounding policy 設定失敗: {e}")
        
        # 創建應用程式
        app = QApplication(sys.argv)
        app.setApplicationName("AeroPlan Studio")
        app.setApplicationDisplayName("AeroPlan Studio — Collaborative UAV Mission Planning Suite")
        app.setOrganizationName("AeroPlan")

        # ── 套用 AeroPlan Studio 主題 ─────────────────────────────
        # 新版設計系統：MIL-STD-1472H 對齊、SVG 圖示替換 emoji、
        # JetBrains Mono / Rajdhani / Inter 字型 stack、全域 QSS。
        # 必須在任何 QWidget 建立前呼叫，否則已建立的視窗不會收到全域 QSS。
        #
        # 載入順序：
        #   1. --legacy-theme 旗標 → 直接用舊版 tactical_theme（除錯 / fallback 用）
        #   2. 預設 → 套新版 aeroplan_theme；失敗自動 fallback 到舊版
        #   3. 兩者皆失敗 → 用系統預設樣式（不擋啟動）
        use_legacy = getattr(args, 'legacy_theme', False) if args is not None else False
        if use_legacy:
            try:
                from ui.resources.tactical_theme import apply_tactical_theme
                apply_tactical_theme(app)
                logger.info("舊版戰術主題 (legacy tactical_theme) 已套用")
            except Exception as e:
                logger.warning(f"套用舊版主題失敗，使用系統預設樣式: {e}")
        else:
            try:
                from ui.resources.aeroplan_theme import apply_theme
                apply_theme(app)
                logger.info("AeroPlan Studio 設計系統主題已套用")
            except Exception as e:
                logger.warning(
                    f"套用新版主題失敗，退回舊版 tactical_theme: {e}"
                )
                try:
                    from ui.resources.tactical_theme import apply_tactical_theme
                    apply_tactical_theme(app)
                    logger.info("已 fallback 至舊版戰術主題")
                except Exception as e2:
                    logger.warning(f"舊版主題亦失敗，使用系統預設樣式: {e2}")

        # 導入並創建主視窗
        # 依 --shell 選擇外殼：
        #   legacy — 既有 MainWindow（完整業務邏輯）
        #   milstd — 全新 AeroPlanMainWindow（MIL-STD-1472H 合規骨架）
        shell = getattr(args, 'shell', 'legacy') if args is not None else 'legacy'
        if shell == 'milstd':
            from ui.aeroplan_main_window import AeroPlanMainWindow
            window = AeroPlanMainWindow()
            logger.info("啟用 MIL-STD-1472H 合規主視窗 (AeroPlanMainWindow)")
        else:
            from ui.main_window import MainWindow
            window = MainWindow()
        window.show()

        logger.info("GUI 啟動成功")

        return app.exec()
    
    except Exception as e:
        logger.error(f"GUI 啟動失敗: {e}")
        import traceback
        traceback.print_exc()
        return 1


def run_cli_mode(logger, settings, vehicle_profiles):
    """運行命令列模式"""
    logger.info("啟動 CLI 模式...")
    logger.warning("CLI 模式尚未完整實現")

    # TODO: 實現命令列模式功能
    print("AeroPlan Studio — CLI 模式")
    print("此功能正在開發中...")
    
    return 0


def _run_subcommand() -> int:
    """ADOS 風格子指令分派（aeroplan sitl / aeroplan demo）。

    無子指令時 sys.argv 不會走到此分支；本函式只在 argv[1] 為已知子指令時被呼叫。
    """
    # 為子指令各建立獨立 argparse；不繼承 root 旗標，避免衝突
    parser = argparse.ArgumentParser(prog="aeroplan-studio")
    subparsers = parser.add_subparsers(dest="subcmd", required=True)

    from cli.sitl_cmd import add_subparser as add_sitl
    from cli.demo_cmd import add_subparser as add_demo
    add_sitl(subparsers)
    add_demo(subparsers)

    args = parser.parse_args()

    # 最簡 logger 初始化（不需 full settings stack）
    import logging
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        return 130


def main():
    """主函式。

    若第一個非旗標參數為已知子指令（sitl / demo），分派到 CLI；
    否則沿用既有 GUI 流程（無破壞性變更）。
    """
    # 偵測子指令
    _SUBCMDS = {"sitl", "demo"}
    if len(sys.argv) > 1 and sys.argv[1] in _SUBCMDS:
        return _run_subcommand()

    # 解析參數
    args = parse_arguments()

    # 初始化系統
    try:
        logger, settings, vehicle_profiles = initialize_system(args)
    except Exception as e:
        print(f"系統初始化失敗: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # 選擇運行模式
    try:
        if args.no_ui:
            return run_cli_mode(logger, settings, vehicle_profiles)
        else:
            return run_gui_mode(logger, settings, vehicle_profiles, args)

    except KeyboardInterrupt:
        logger.info("用戶中斷程式")
        return 0

    except Exception as e:
        logger.error(f"程式異常終止: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        logger.info("AeroPlan Studio 已退出")


if __name__ == '__main__':
    sys.exit(main())