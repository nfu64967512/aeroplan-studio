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

from config import init_settings, get_settings
from utils import get_logger, setup_logger


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
    logger.info("版本: 2.5.0")
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


def run_gui_mode(logger, settings, vehicle_profiles):
    """運行GUI模式"""
    try:
        logger.info("啟動 GUI 模式...")
        
        # 檢查PyQt6是否可用
        try:
            from PyQt6.QtWidgets import QApplication
            from PyQt6.QtCore import Qt
            
            # 重要：在創建 QApplication 之前導入 WebEngine
            # 這是 PyQt6 WebEngine 的已知要求
            try:
                from PyQt6.QtWebEngineWidgets import QWebEngineView
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
        
        # 導入並創建主視窗
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


def main():
    """主函式"""
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
            return run_gui_mode(logger, settings, vehicle_profiles)
    
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