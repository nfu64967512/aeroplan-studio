# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec — AeroPlan Studio

打包成 one-folder 模式（dist/AeroPlanStudio/）。
WebEngine + Cesium + ArduPilot SITL 等資源較大且需執行子程序，
**不建議** 使用 one-file，因為解壓到 _MEIxxxx 會造成首次啟動極慢
且 WebEngine 子程序找不到資源。

build:
    pyinstaller aeroplan_studio.spec --noconfirm
    # 或：python -m PyInstaller aeroplan_studio.spec --noconfirm

執行：
    dist/AeroPlanStudio/AeroPlanStudio.exe
"""

import os
import sys
from pathlib import Path
from PyInstaller.utils.hooks import collect_all, collect_data_files, collect_submodules

# ── 專案根目錄 ──────────────────────────────────────────────
PROJECT_ROOT = Path(os.path.abspath(SPECPATH)).resolve()

# ── 資源/資料檔（src, dst_dir_in_bundle）───────────────────
# dst_dir 用相對路徑（相對於最終 exe 所在資料夾）
datas = []

def _add_dir(src_rel: str, dst_rel: str = None, exclude_dirs=None):
    """把整個資料夾遞迴加入；自動跳過 __pycache__；exclude_dirs 為頂層子資料夾名稱。"""
    src = PROJECT_ROOT / src_rel
    if not src.exists():
        print(f'[spec] skip missing dir: {src}')
        return
    dst = dst_rel if dst_rel is not None else src_rel
    excl = set(exclude_dirs or [])
    for p in src.rglob('*'):
        if not p.is_file():
            continue
        if '__pycache__' in p.parts:
            continue
        rel_inside = p.relative_to(src)
        # 比對任一層路徑符合排除清單即跳過
        if any(part in excl for part in rel_inside.parts):
            continue
        datas.append((str(p), str(Path(dst) / rel_inside.parent)))

def _add_file(src_rel: str, dst_rel: str = None):
    src = PROJECT_ROOT / src_rel
    if not src.exists():
        print(f'[spec] skip missing file: {src}')
        return
    dst = dst_rel if dst_rel is not None else str(Path(src_rel).parent)
    if dst == '':
        dst = '.'
    datas.append((str(src), dst))

# Cesium 3D 引擎（必要，~50MB）
_add_dir('assets/cesium', 'assets/cesium')
# Leaflet 2D 地圖
_add_dir('assets/leaflet', 'assets/leaflet')
# 3D 地圖 HTML 模板
_add_dir('ui/widgets/cesium_templates', 'ui/widgets/cesium_templates')
# UI 資源（GLB 模型、QSS 主題、圖示）
_add_dir('ui/resources', 'ui/resources')
# 設定檔
_add_file('config/vehicle_profiles.yaml', 'config')
# DEM 地形資料
_add_file('huwei_dem.tif', '.')
# 預設角點
_add_file('polygon_corners.json', '.')
# ArduPilot SITL 執行檔 + cygwin DLL + 預設參數 + 模型
# ⚠️ 排除 sitl/plane/ + sitl/+/ + 各 instance 的 logs/eeprom/terrain
#    這些是 SITL 執行期累積資料（會佔 GB 級），首次啟動會自動建立
_add_dir('sitl', 'sitl', exclude_dirs=[
    'plane',     # sitl/plane/iN/ 為各機 SITL 工作目錄（runtime 產生）
    '+',         # sitl/+ 為 cygwin link 暫存目錄
    'logs',
    'terrain',
    'eeprom.bin',
])
# 文件（可選；註解掉減少體積）
# _add_dir('docs', 'docs')

# pyproj 座標轉換需要的 PROJ 資料庫
datas += collect_data_files('pyproj')
# folium 內建 HTML/JS 模板
datas += collect_data_files('folium')
# pymavlink 訊息定義
datas += collect_data_files('pymavlink')
# shapely / matplotlib 資源
datas += collect_data_files('shapely')
datas += collect_data_files('matplotlib')

# ── 隱式匯入（PyInstaller 靜態分析抓不到的模組）──────────────
hiddenimports = []

# pymavlink 動態載入 dialect 模組
hiddenimports += collect_submodules('pymavlink.dialects')
hiddenimports += [
    'pymavlink.dialects.v20.ardupilotmega',
    'pymavlink.dialects.v20.common',
    'pymavlink.dialects.v20.standard',
    'pymavlink.dialects.v10.ardupilotmega',
    'pymavlink.dialects.v10.common',
]

# PyQt6 WebEngine（一定要顯式宣告，hook 有時抓不全）
hiddenimports += [
    'PyQt6.QtCore',
    'PyQt6.QtGui',
    'PyQt6.QtWidgets',
    'PyQt6.QtNetwork',
    'PyQt6.QtPrintSupport',
    'PyQt6.QtWebEngineCore',
    'PyQt6.QtWebEngineWidgets',
    'PyQt6.QtWebChannel',
    'PyQt6.sip',
]

# numba JIT 執行期相依
hiddenimports += [
    'numba.core.typing.builtins',
    'numba.core.imputils',
]

# 其他常見動態匯入
hiddenimports += [
    'pyproj.datadir',
    'shapely.geometry',
    'shapely.ops',
    'yaml',
    'json',
]

# 確保專案內所有套件都被打包
hiddenimports += collect_submodules('core')
hiddenimports += collect_submodules('mission')
hiddenimports += collect_submodules('ui')
hiddenimports += collect_submodules('utils')
hiddenimports += collect_submodules('config')
hiddenimports += collect_submodules('sensors')

# ── 排除模組（縮小體積）──────────────────────────────────────
excludes = [
    'tkinter',           # 雖有 tkinter_map_widget.py，但在 PyQt 環境下不使用
    'unittest',          # 測試框架
    'pytest',
    'pytest_cov',
    'pytest_mock',
    'sphinx',
    'sphinx_rtd_theme',
    'mypy',
    'black',
    'flake8',
    'IPython',
    'jupyter',
    'notebook',
    'pandas',            # 沒用到
    'PyQt5',
    'PySide2',
    'PySide6',
]

# ── 圖示（如有 .ico）─────────────────────────────────────────
icon_path = PROJECT_ROOT / 'ui' / 'resources' / 'app.ico'
icon_arg = str(icon_path) if icon_path.exists() else None


# ══════════════════════════════════════════════════════════════════════
# Analysis
# ══════════════════════════════════════════════════════════════════════
a = Analysis(
    ['main.py'],
    pathex=[str(PROJECT_ROOT)],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=excludes,
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='AeroPlanStudio',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,                  # WebEngine 二進位用 UPX 壓會 crash
    console=False,              # GUI 應用：不顯示黑色 console 視窗
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=icon_arg,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='AeroPlanStudio',
)
