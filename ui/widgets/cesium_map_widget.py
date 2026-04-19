"""
Cesium 3D 地圖組件
使用 Cesium.js + QWebEngineView 實現 3D 互動式地圖

特色：
- 完整 3D 地形 + 衛星影像
- 固定翼三階段路徑含高度視覺化（起飛爬升 / 掃描 / 降落）
- 多旋翼路徑等高度顯示
- SITL 即時飛行位置更新預留接口
- 禁航區 3D 柱體顯示
- 與 2D MapWidget 相同的 pyqt:// URL bridge
"""

import json
import math
from typing import List, Tuple, Optional

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage, QWebEngineSettings
from PyQt6.QtCore import pyqtSignal, QUrl, Qt
from urllib.parse import unquote

from utils.logger import get_logger

logger = get_logger()

# ── 顏色常數（與 MapWidget 一致）─────────────────────────────────────
_REGION_COLORS = ['#08EC91', '#FF6B35', '#3D87FF', '#FFD700', '#FF69B4', '#9B59B6']
_DRONE_COLORS  = ['#E53935', '#1E88E5', '#43A047', '#FB8C00',
                  '#8E24AA', '#00ACC1', '#F4511E', '#3949AB']



# ══════════════════════════════════════════════════════════════════════
#  Cesium HTML 模板 — 從 ui/widgets/cesium_templates/main.html 載入
#  (2026 重構：原本 3234 行的 inline HTML 抽到靜態檔，Python 檔瘦身 76%)
# ══════════════════════════════════════════════════════════════════════
import os as _os
_TEMPLATE_PATH = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                "cesium_templates", "main.html")

def _load_cesium_html() -> str:
    """讀取 Cesium HTML 模板；若檔案遺失則回報清楚錯誤"""
    try:
        with open(_TEMPLATE_PATH, "r", encoding="utf-8") as _fh:
            return _fh.read()
    except FileNotFoundError:
        raise FileNotFoundError(
            f"Cesium HTML 模板遺失：{_TEMPLATE_PATH}\n"
            f"此檔為 AeroPlan Studio 3D 地圖必要資源，不可刪除。"
        )

_CESIUM_HTML = _load_cesium_html()




# ══════════════════════════════════════════════════════════════════════
#  URL Bridge（重用 ClickCapturePage 邏輯）
# ══════════════════════════════════════════════════════════════════════
class CesiumPage(QWebEnginePage):
    def __init__(self, parent, widget):
        super().__init__(parent)
        self.widget = widget

    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        prefix = {0: 'ℹ️', 1: '⚠️', 2: '❌'}.get(level, '🔹')
        print(f"{prefix} [Cesium JS] {message}")

    def acceptNavigationRequest(self, url, nav_type, is_main_frame):
        url_str = url.toString()
        if url_str.startswith('pyqt://click/'):
            try:
                parts = url_str.replace('pyqt://click/', '').split('/')
                lat, lon = float(parts[0]), float(parts[1])
                self.widget.on_map_clicked(lat, lon)
            except Exception as e:
                print(f'❌ [Cesium] 解析點擊失敗: {e}')
            return False
        if url_str.startswith('pyqt://circle_draw/'):
            try:
                parts = url_str.replace('pyqt://circle_draw/', '').split('/')
                lat, lon, radius = float(parts[0]), float(parts[1]), float(parts[2])
                self.widget.on_circle_draw_complete(lat, lon, radius)
            except Exception as e:
                print(f'❌ [Cesium] 解析圓形失敗: {e}')
            return False
        if url_str.startswith('pyqt://nfz_poly_done/'):
            encoded = unquote(url_str[len('pyqt://nfz_poly_done/'):])
            vertices = []
            for pair in encoded.split('|'):
                parts = pair.split(',')
                if len(parts) == 2:
                    try:
                        vertices.append((float(parts[0]), float(parts[1])))
                    except ValueError:
                        pass
            self.widget.on_nfz_poly_done(vertices)
            return False
        return True


# ══════════════════════════════════════════════════════════════════════
#  CesiumMapWidget
# ══════════════════════════════════════════════════════════════════════
class CesiumMapWidget(QWidget):
    """3D 地圖組件，介面與 MapWidget 相容"""

    # ── 與 MapWidget 相同的信號 ───────────────────────────────────────
    corner_added       = pyqtSignal(float, float)
    corner_moved       = pyqtSignal(int, float, float)
    circle_defined     = pyqtSignal(float, float, float)
    nfz_polygon_drawn  = pyqtSignal(list)
    nfz_circle_drawn   = pyqtSignal(float, float, float)
    strike_target_added = pyqtSignal(float, float)  # 打擊目標標記

    # ── Cesium Ion Token（可在設定中覆寫）────────────────────────────
    CESIUM_TOKEN: str = ''   # 留空 = 無地形 Token，衛星影像仍正常

    def __init__(self, parent=None):
        super().__init__(parent)
        self.corners: List[Tuple[float, float]] = []
        self.paths: list = []
        self.path_colors: list = []
        self._path_tooltips: list = []
        self.transit_paths: list = []
        self._circle_center: Optional[Tuple[float, float]] = None
        self._circle_radius_m: float = 0.0
        self._home_point: Optional[Tuple[float, float]] = None
        self._nfz_zones: list = []
        self._swarm_data: Optional[dict] = None
        self._fw_result: Optional[dict] = None   # fw_mission_result，用於 3D 高度
        self._last_altitude: float = 50.0        # 多旋翼路徑高度
        self._clamp_basic: bool = False          # 基本演算法 grid 是否貼地
        self._strike_marking: bool = False       # 打擊目標標記模式

        self._cesium_html_file = None

        self._init_ui()
        self._load_cesium()
        logger.info('Cesium 3D 地圖組件初始化完成')

    def __del__(self):
        if self._cesium_html_file:
            try:
                import os
                os.unlink(self._cesium_html_file.name)
            except Exception:
                pass

    # ─────────────────────────────────────────────────────────────────
    # UI 初始化
    # ─────────────────────────────────────────────────────────────────
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.web_view = QWebEngineView()
        self.web_view.setMinimumSize(400, 400)
        self.web_view.setSizePolicy(
            self.web_view.sizePolicy().horizontalPolicy(),
            self.web_view.sizePolicy().verticalPolicy(),
        )
        self.web_view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)

        self._page = CesiumPage(self.web_view, self)
        self.web_view.setPage(self._page)

        ws = self._page.settings()
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalStorageEnabled, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.WebGLEnabled, True)

        layout.addWidget(self.web_view)

    def _load_cesium(self):
        import os
        from pathlib import Path

        # 讀取 2D 地圖預設座標保持一致
        try:
            from config.settings import get_settings
            s = get_settings()
            init_lat = s.map.default_lat
            init_lon = s.map.default_lon
        except Exception:
            init_lat, init_lon = 23.702732, 120.419333
        init_height = 2000.0

        # ── 本地 / CDN 路徑解析 ──
        project_root = Path(__file__).resolve().parent.parent.parent
        local_cesium = project_root / 'assets' / 'cesium' / 'Build' / 'Cesium'
        local_js     = local_cesium / 'Cesium.js'
        local_css    = local_cesium / 'Widgets' / 'widgets.css'

        if local_js.exists() and local_css.exists():
            cesium_js  = local_js.as_uri()
            cesium_css = local_css.as_uri()
            base_url   = QUrl.fromLocalFile(str(local_cesium) + os.sep)
            logger.info(f'[Cesium] 使用本地資源: {local_cesium}')
        else:
            cesium_js  = 'https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Cesium.js'
            cesium_css = 'https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Widgets/widgets.css'
            base_url   = QUrl('https://cesium.com/')
            logger.info('[Cesium] 使用 CDN 線上資源')

        # ── GLB 3D 模型路徑 ──
        plane_glb = project_root / 'ui' / 'resources' / 'P25-ID.glb'
        plane_model_uri = plane_glb.as_uri() if plane_glb.exists() else ''
        logger.info(f'[Cesium] 固定翼 GLB 模型: {plane_model_uri or "未找到，使用 fallback"}')

        copter_glb = project_root / 'ui' / 'resources' / 'copter.glb'
        copter_model_uri = copter_glb.as_uri() if copter_glb.exists() else ''
        logger.info(f'[Cesium] 多旋翼 GLB 模型: {copter_model_uri or "未找到，使用 fallback"}')

        html = (_CESIUM_HTML
                .replace('__CESIUM_JS__',   cesium_js)
                .replace('__CESIUM_CSS__',  cesium_css)
                .replace('__CESIUM_TOKEN__', self.CESIUM_TOKEN)
                .replace('__INIT_LAT__',    str(init_lat))
                .replace('__INIT_LON__',    str(init_lon))
                .replace('__INIT_HEIGHT__', str(init_height))
                .replace('__PLANE_MODEL_URI__', plane_model_uri)
                .replace('__COPTER_MODEL_URI__', copter_model_uri))

        # 將 HTML 寫入 data/tmp_maps/cesium_NNNN.html (每次覆蓋舊檔)
        # 所有 Cesium 資源 (JS/CSS/GLB) 皆為絕對 file:// URI，因此 HTML
        # 放在任何位置都能解析，不受子資料夾影響
        from utils.temp_cache import new_temp_html_path
        out_path = new_temp_html_path(prefix='cesium', keep_last=1)
        with open(out_path, 'w', encoding='utf-8') as fh:
            fh.write(html)
        # 保留 file 屬性以供後續判斷 (不再是 NamedTemporaryFile 物件)
        self._cesium_html_file = type('Dummy', (), {'name': str(out_path)})()
        html_url = QUrl.fromLocalFile(str(out_path))
        logger.info(f'[Cesium] 載入 HTML: {html_url.toString()}')
        self.web_view.setUrl(html_url)

    # ─────────────────────────────────────────────────────────────────
    # JS 橋接：Python → Cesium
    # ─────────────────────────────────────────────────────────────────
    def _js(self, script: str):
        """執行 JavaScript（非同步）"""
        self._page.runJavaScript(script)

    @staticmethod
    def _pts_to_json(pts, default_alt: float = 50.0) -> str:
        """將 [(lat,lon)] 或 [(lat,lon,alt)] 序列化為 JSON"""
        result = []
        for p in pts:
            if len(p) >= 3:
                result.append({'lat': p[0], 'lon': p[1], 'alt': p[2]})
            else:
                result.append({'lat': p[0], 'lon': p[1], 'alt': default_alt})
        return json.dumps(result)

    # ─────────────────────────────────────────────────────────────────
    # URL Bridge 回調（由 CesiumPage 呼叫）
    # ─────────────────────────────────────────────────────────────────
    def on_map_clicked(self, lat: float, lon: float):
        if self._strike_marking:
            self.strike_target_added.emit(lat, lon)
            return
        self.corner_added.emit(lat, lon)

    def on_circle_draw_complete(self, lat: float, lon: float, radius: float):
        self.circle_defined.emit(lat, lon, radius)

    def on_nfz_poly_done(self, vertices):
        self.nfz_polygon_drawn.emit(vertices)

    # ─────────────────────────────────────────────────────────────────
    # 狀態同步（由 DualMapWidget 呼叫，切換到 3D 時同步 2D 狀態）
    # ─────────────────────────────────────────────────────────────────
    def sync_state(self, corners, paths, path_colors, path_tooltips,
                   transit_paths, circle_center, circle_radius,
                   home_point, nfz_zones, swarm_data):
        """同步 MapWidget 狀態到 3D 地圖"""
        self.corners       = list(corners)
        self.paths         = list(paths)
        self.path_colors   = list(path_colors)
        self._path_tooltips = list(path_tooltips)
        self.transit_paths  = list(transit_paths)
        self._circle_center = circle_center
        self._circle_radius_m = circle_radius
        self._home_point    = home_point
        self._nfz_zones     = list(nfz_zones)
        self._swarm_data    = swarm_data
        self._refresh_all()

    def _refresh_all(self):
        """重繪 3D 地圖上的所有元素"""
        self._refresh_corners()
        self._refresh_paths()
        self._refresh_overlays()
        self._refresh_nfz()

    def _refresh_corners(self):
        corners_data = [{'lat': lat, 'lon': lon} for lat, lon in self.corners]
        corners_json = json.dumps(corners_data)
        self._js(f'setCorners({json.dumps(corners_json)})')

    def _refresh_paths(self):
        self._js('clearPaths()')
        for idx, path in enumerate(self.paths):
            if len(path) < 2:
                continue
            color = (self.path_colors[idx]
                     if idx < len(self.path_colors) else _REGION_COLORS[0])
            tooltip = (self._path_tooltips[idx]
                       if idx < len(self._path_tooltips) else '')

            # 取得高度（固定翼用 _fw_result；多旋翼用 _last_altitude）
            alt = self._resolve_altitude_for_path(idx, tooltip)
            pts_json = self._pts_to_json(path, default_alt=alt)

            dashed = '轉移' in tooltip or 'transfer' in tooltip.lower()
            label_escaped = json.dumps(tooltip)
            clamp = 'true' if getattr(self, '_clamp_basic', False) else 'false'
            self._js(f'addPath({json.dumps(pts_json)},{json.dumps(color)},10,{label_escaped},{str(dashed).lower()},{clamp})')

        # 轉場路徑（虛線）
        for tp in self.transit_paths:
            t_path = tp.get('path', [])
            t_alt  = tp.get('altitude', self._last_altitude)
            t_idx  = tp.get('region_index', 0)
            if len(t_path) < 2:
                continue
            color = _REGION_COLORS[t_idx % len(_REGION_COLORS)]
            pts_json = self._pts_to_json(t_path, default_alt=t_alt)
            label = f'轉場→區域{t_idx+1}'
            self._js(f'addPath({json.dumps(pts_json)},{json.dumps(color)},7,{json.dumps(label)},true)')

    def _resolve_altitude_for_path(self, idx: int, tooltip: str) -> float:
        """從 fw_result 解析高度（固定翼），否則回傳 _last_altitude"""
        if self._fw_result:
            if '起飛' in tooltip:
                tp = self._fw_result.get('takeoff_path', [])
                if tp and len(tp[0]) >= 3:
                    return tp[0][2]  # 起飛起始高度
            elif '降落' in tooltip or '進場' in tooltip:
                lp = self._fw_result.get('landing_path', [])
                if lp and len(lp[0]) >= 3:
                    return lp[0][2]
            else:
                mp = self._fw_result.get('mission_path', [])
                if mp and len(mp[0]) >= 3:
                    return mp[0][2]
        return self._last_altitude

    def _refresh_overlays(self):
        if self._circle_center:
            lat, lon = self._circle_center
            self._js(f'setCircleOverlay({lat},{lon},{self._circle_radius_m},"#2196F3")')
        if self._home_point:
            h_lat, h_lon = self._home_point
            self._js(f'setHomePoint({h_lat},{h_lon})')

    def _refresh_nfz(self):
        self._js('clearNFZ()')
        for nfz in self._nfz_zones:
            name = json.dumps(nfz.get('name', 'NFZ'))
            if nfz['type'] == 'polygon':
                verts = [{'lat': la, 'lon': lo} for la, lo in nfz['vertices']]
                self._js(f'addNFZPolygon({json.dumps(json.dumps(verts))},{name})')
            elif nfz['type'] == 'circle':
                clat, clon = nfz['center']
                r = nfz['radius']
                self._js(f'addNFZCircle({clat},{clon},{r},{name})')

    # ─────────────────────────────────────────────────────────────────
    # 固定翼 3D 路徑（高度完整資訊）
    # ─────────────────────────────────────────────────────────────────
    def set_fw_result(self, result: dict):
        """接收 fw_mission_result，用於提取 (lat,lon,alt) 3D 路徑"""
        self._fw_result = result
        self._render_fw_3d(result)

    def _render_fw_3d(self, result: dict):
        """在 3D 地圖上用完整高度資訊繪製固定翼三階段路徑

        為避免段與段之間出現視覺斷層：
        - 在 takeoff 末端附加 mission 起點
        - 在 mission 末端附加 landing 起點
        """
        self._js('clearPaths()')

        takeoff = list(result.get('takeoff_path', []) or [])
        mission = list(result.get('mission_path', []) or [])
        landing = list(result.get('landing_path', []) or [])

        # 銜接：takeoff → mission
        if takeoff and mission:
            takeoff = takeoff + [mission[0]]
        # 銜接：mission → landing
        if mission and landing:
            mission = mission + [landing[0]]

        # 起飛段（金黃）
        if len(takeoff) >= 2:
            pts_json = self._pts_to_json(takeoff, default_alt=50.0)
            self._js(f'addPath({json.dumps(pts_json)},"#FFD700",12,"起飛爬升",false)')

        # 任務段（藍）
        if len(mission) >= 2:
            pts_json = self._pts_to_json(mission, default_alt=100.0)
            self._js(f'addPath({json.dumps(pts_json)},"#3D87FF",10,"任務掃描",false)')

        # 降落段（橙）
        if len(landing) >= 2:
            pts_json = self._pts_to_json(landing, default_alt=80.0)
            self._js(f'addPath({json.dumps(pts_json)},"#FF6B35",10,"五邊進場",false)')

        # 飛向路徑
        self._js('flyToScene()')

    # ─────────────────────────────────────────────────────────────────
    # 與 MapWidget 相容的公開方法
    # ─────────────────────────────────────────────────────────────────
    def display_path(self, path: List[Tuple], altitude: float = 50.0):
        self._last_altitude = altitude
        self.paths = [path]
        self.path_colors = [_REGION_COLORS[0]]
        self._path_tooltips = []
        self._fw_result = None
        self._clamp_basic = True
        self._refresh_paths()
        self._clamp_basic = False
        self._js('flyToScene()')

    def display_paths(self, paths_list: List[List[Tuple]], altitude: float = 50.0):
        self._last_altitude = altitude
        self.paths = [p for p in paths_list if p and len(p) >= 2]
        self.path_colors = [_REGION_COLORS[i % len(_REGION_COLORS)]
                            for i in range(len(self.paths))]
        self._path_tooltips = [f'掃描區域 {i+1}' for i in range(len(self.paths))]
        self._fw_result = None
        self._clamp_basic = True
        self._refresh_paths()
        self._clamp_basic = False
        self._js('flyToScene()')

    def display_fw_paths(self,
                         takeoff: List[Tuple],
                         mission,
                         landing: List[Tuple]):
        """固定翼三階段（2D 座標版；高度用 _fw_result 補充，若無則用常數）"""
        # 正規化 mission
        if not mission:
            mission_paths = []
        elif mission and isinstance(mission[0], tuple):
            mission_paths = [mission]
        else:
            mission_paths = [m for m in mission if m and len(m) >= 2]

        _SCAN_COLORS = ['#3D87FF', '#08EC91', '#FF69B4', '#9B59B6', '#00BCD4', '#FF4500']

        self.paths = []
        self.path_colors = []
        self._path_tooltips = []

        if takeoff and len(takeoff) >= 2:
            self.paths.append(takeoff)
            self.path_colors.append('#FFD700')
            self._path_tooltips.append('起飛爬升路徑')

        for i, mp in enumerate(mission_paths):
            label = f'掃描區域 {i+1}' if len(mission_paths) > 1 else '任務掃描路徑'
            self.paths.append(mp)
            self.path_colors.append(_SCAN_COLORS[i % len(_SCAN_COLORS)])
            self._path_tooltips.append(label)

        if landing and len(landing) >= 2:
            self.paths.append(landing)
            self.path_colors.append('#FF6B35')
            self._path_tooltips.append('五邊進場路徑')

        # 如果已有 fw_result，用高度完整版渲染
        if self._fw_result:
            self._render_fw_3d(self._fw_result)
        else:
            self._refresh_paths()
            self._js('flyToScene()')

    def display_nfz_zones(self, nfz_zones: list):
        self._nfz_zones = list(nfz_zones) if nfz_zones else []
        self._refresh_nfz()

    def draw_circle_overlay(self, center_lat: float, center_lon: float,
                             radius_m: float, color: str = '#2196F3'):
        self._circle_center   = (center_lat, center_lon)
        self._circle_radius_m = radius_m
        self._js(f'setCircleOverlay({center_lat},{center_lon},{radius_m},{json.dumps(color)})')

    def clear_circle_overlay(self):
        self._circle_center   = None
        self._circle_radius_m = 0.0
        self._js('if(circleEntity){viewer.entities.remove(circleEntity);circleEntity=null;}')

    def set_home_point_overlay(self, lat: float, lon: float):
        self._home_point = (lat, lon)
        self._js(f'setHomePoint({lat},{lon})')

    def clear_home_point_overlay(self):
        self._home_point = None
        self._js('if(homeEntity){viewer.entities.remove(homeEntity);homeEntity=null;}')

    def display_swarm_coverage(self, swarm_mission, coverage_paths=None):
        """群飛路徑顯示（3D 版）"""
        try:
            self._js('clearPaths()')
            for drone in swarm_mission.drones:
                drone_id = drone.drone_id
                color    = _DRONE_COLORS[(drone_id - 1) % len(_DRONE_COLORS)]
                if not drone.mission or not drone.mission.waypoints:
                    continue
                nav_wps = [wp for wp in drone.mission.waypoints.waypoints if wp.command == 16]
                if len(nav_wps) < 2:
                    continue
                pts = [{'lat': wp.lat, 'lon': wp.lon, 'alt': getattr(wp, 'alt', 50.0)}
                       for wp in nav_wps]
                pts_json = json.dumps(pts)
                label    = f'Drone {drone_id}'
                self._js(f'addPath({json.dumps(pts_json)},{json.dumps(color)},6,{json.dumps(label)},false)')
            self._js('flyToScene()')
        except Exception as e:
            logger.error(f'Cesium 群飛顯示失敗: {e}')

    def display_swarm_raw(self, swarm_data: dict):
        """群飛原始資料顯示（3D 版）— 支援 DCCPP 五階段分色：
        TAKEOFF（綠）/ ENTRY（青）/ OPERATION（無人機主色）/
        TRANSFER（主色長虛線）/ LANDING（橘 dash-dot）。
        點可為 (lat, lon) 或 (lat, lon, alt)。
        """
        try:
            self._js('clearPaths()')
            drones = swarm_data.get('drones', [])

            def _to_pts(seg, default_alt):
                return [{'lat': p[0], 'lon': p[1],
                         'alt': (p[2] if len(p) > 2 else default_alt)}
                        for p in seg]

            def _add(seg, color, width, label, dashed, default_alt=80.0):
                if not seg or len(seg) < 2:
                    return
                pts = _to_pts(seg, default_alt)
                self._js(
                    f'addPath({json.dumps(json.dumps(pts))},'
                    f'{json.dumps(color)},{width},{json.dumps(label)},'
                    f'{"true" if dashed else "false"})'
                )

            for drone_info in drones:
                drone_id = drone_info.get('drone_id', 1)
                color    = _DRONE_COLORS[(drone_id - 1) % len(_DRONE_COLORS)]

                # TAKEOFF — 金黃實線
                for seg in drone_info.get('takeoff_paths', []) or []:
                    _add(seg, '#FFD700', 8, f'D{drone_id} 起飛', False, 30.0)

                # ENTRY — 青色實線
                for seg in drone_info.get('entry_paths', []) or []:
                    _add(seg, '#26C6DA', 6, f'D{drone_id} 進入', False)

                # OPERATION — 無人機主色，最粗實線
                for seg in drone_info.get('operation_paths', []) or []:
                    _add(seg, color, 8, f'D{drone_id} 作業', False)

                # TRANSFER — 主色虛線
                for seg in drone_info.get('transfer_paths', []) or []:
                    _add(seg, color, 5, f'D{drone_id} 轉移', True)

                # LANDING — 橘色實線
                for seg in drone_info.get('landing_paths', []) or []:
                    _add(seg, '#FF6B35', 7, f'D{drone_id} 降落', False, 40.0)

            self._js('flyToScene()')
        except Exception as e:
            logger.error(f'Cesium 群飛原始顯示失敗: {e}')

    def display_transit_paths(self, transit_paths: list):
        self.transit_paths = list(transit_paths)
        self._refresh_paths()

    def clear_transit_paths(self):
        self.transit_paths = []
        self._refresh_paths()

    def set_edit_mode(self, enabled: bool):
        """編輯模式（3D 地圖無需特別處理）"""
        pass

    def add_corner(self, lat: float, lon: float):
        if len(self.corners) < 100:
            self.corners.append((lat, lon))
            self._refresh_corners()

    def move_corner(self, index: int, lat: float, lon: float):
        if 0 <= index < len(self.corners):
            self.corners[index] = (lat, lon)
            self._refresh_corners()

    def clear_paths(self):
        """僅清除路徑（保留角點 / 起飛點 / NFZ 等其他標記）。

        對應主視窗「清除路徑」按鈕（Esc 快捷鍵）；之前這個方法不存在，
        透過 DualMapWidget.__getattr__ 會 fallback 到 2D 地圖，導致 3D
        路徑殘留在畫面上。
        """
        self.paths.clear()
        self.path_colors.clear()
        self._path_tooltips.clear()
        self.transit_paths.clear()
        # 固定翼/DCCPP 組裝結果也一併清掉
        self._fw_result = None
        # JS 端清除所有路徑 entities（含 DCCPP 群飛、固定翼三段等）
        self._js('clearPaths()')
        # 若有 swarm 原始資料也清除
        if hasattr(self, '_swarm_data') and self._swarm_data is not None:
            self._swarm_data = None
            self._js('if (typeof clearSwarmRaw === "function") clearSwarmRaw();')
        # 末端打擊 / 俯衝視覺化
        self._js('if (typeof strikeClearAll === "function") strikeClearAll();')
        self._js('if (typeof clearAllTerminalDives === "function") clearAllTerminalDives();')

    def clear_all(self):
        self.corners.clear()
        self.paths.clear()
        self.path_colors.clear()
        self._path_tooltips.clear()
        self.transit_paths.clear()
        self._circle_center   = None
        self._circle_radius_m = 0.0
        self._home_point      = None
        self._nfz_zones.clear()
        self._swarm_data      = None
        self._fw_result       = None
        self._js('clearCorners(); clearPaths(); clearNFZ();')
        self._js('if(circleEntity){viewer.entities.remove(circleEntity);circleEntity=null;}')
        self._js('if(homeEntity){viewer.entities.remove(homeEntity);homeEntity=null;}')
        self._js('clearDemOverlay()')
        # 末端打擊 / 俯衝視覺化
        self._js('if (typeof strikeClearAll === "function") strikeClearAll();')
        self._js('if (typeof clearAllTerminalDives === "function") clearAllTerminalDives();')

    # ─────────────────────────────────────────────────────────────────
    # DEM 地形視覺化
    # ─────────────────────────────────────────────────────────────────
    def show_dem_overlay(self, dem_manager, alpha: float = 0.65,
                          max_size: int = 256) -> bool:
        """
        將 DEM 資料以高度 heatmap 方式疊加在 3D 地圖上。
        dem_manager: 已載入的 DEMTerrainManager
        """
        try:
            import numpy as np
            import base64
            from io import BytesIO
            data = getattr(dem_manager, '_data', None)
            if data is None or not getattr(dem_manager, '_loaded', False):
                logger.warning('[DEM] 尚未載入任何 DEM 資料')
                return False

            lat_min = float(dem_manager._lat_min)
            lat_max = float(dem_manager._lat_max)
            lon_min = float(dem_manager._lon_min)
            lon_max = float(dem_manager._lon_max)

            # 過濾 nodata
            arr = np.array(data, dtype=np.float64)
            nodata = float(getattr(dem_manager, '_nodata', -9999.0))
            mask = (arr > nodata + 1) & np.isfinite(arr)
            if not mask.any():
                logger.warning('[DEM] 全為 nodata')
                return False
            min_e = float(arr[mask].min())
            max_e = float(arr[mask].max())
            rng = max(max_e - min_e, 1.0)

            # 下採樣
            h, w = arr.shape
            step_r = max(1, h // max_size)
            step_c = max(1, w // max_size)
            sub = arr[::step_r, ::step_c]
            sub_mask = (sub > nodata + 1) & np.isfinite(sub)

            # 正規化 0~1 → 彩虹色帶
            norm = np.clip((sub - min_e) / rng, 0.0, 1.0)
            rgba = np.zeros((*sub.shape, 4), dtype=np.uint8)
            # 色帶：藍→青→綠→黃→紅
            def ramp(t):
                if t < 0.25:
                    k = t / 0.25;   return (0, int(255 * k), 255)
                if t < 0.5:
                    k = (t - 0.25) / 0.25; return (0, 255, int(255 * (1 - k)))
                if t < 0.75:
                    k = (t - 0.5) / 0.25; return (int(255 * k), 255, 0)
                k = (t - 0.75) / 0.25; return (255, int(255 * (1 - k)), 0)
            it = np.nditer(norm, flags=['multi_index'])
            while not it.finished:
                r, g, b = ramp(float(it[0]))
                i, j = it.multi_index
                rgba[i, j] = (r, g, b, 230 if sub_mask[i, j] else 0)
                it.iternext()

            # 轉 PNG base64
            try:
                from PIL import Image
                img = Image.fromarray(rgba, 'RGBA')
                buf = BytesIO()
                img.save(buf, format='PNG')
                png_bytes = buf.getvalue()
            except ImportError:
                # 退而求其次：用 numpy 自行寫 PNG — 需 PIL 才可行
                logger.error('[DEM] 需要 Pillow 才能生成 heatmap: pip install pillow')
                return False

            b64 = base64.b64encode(png_bytes).decode('ascii')
            data_url = f'data:image/png;base64,{b64}'

            self._js(
                f'showDemOverlay({lat_min},{lat_max},{lon_min},{lon_max},'
                f'{json.dumps(data_url)},{min_e},{max_e},{alpha})'
            )
            logger.info(
                f'[DEM] 疊加完成: {sub.shape[1]}x{sub.shape[0]} 點, '
                f'範圍 {min_e:.1f}~{max_e:.1f}m'
            )
            return True
        except Exception as e:
            logger.error(f'[DEM] 疊加失敗: {e}', exc_info=True)
            return False

    def clear_dem_overlay(self):
        self._js('clearDemOverlay()')

    def show_dem_terrain_3d(self, dem_manager, max_size: int = 256) -> bool:
        """把 DEM 以平滑 3D 地形顯示 (Cesium CustomHeightmapTerrainProvider)。

        整個地球地形會被替換成 DEM 資料，用雙線性插值採樣。
        max_size: 下採樣後每邊最大解析度（256×256 ≈ 夠平滑）
        """
        try:
            import numpy as np
            data = getattr(dem_manager, '_data', None)
            if data is None or not getattr(dem_manager, '_loaded', False):
                logger.warning('[DEM3D] 尚未載入任何 DEM 資料')
                return False

            lat_min = float(dem_manager._lat_min)
            lat_max = float(dem_manager._lat_max)
            lon_min = float(dem_manager._lon_min)
            lon_max = float(dem_manager._lon_max)

            arr = np.array(data, dtype=np.float32)
            nodata = float(getattr(dem_manager, '_nodata', -9999.0))
            arr = np.where((arr > nodata + 1) & np.isfinite(arr), arr, 0.0)

            # 下採樣
            h, w = arr.shape
            step_r = max(1, h // max_size)
            step_c = max(1, w // max_size)
            sub = arr[::step_r, ::step_c]
            sh, sw = sub.shape

            flat = sub.flatten().tolist()

            self._js(
                f'showDemTerrainHeightmap({lat_min},{lat_max},{lon_min},{lon_max},'
                f'{sh},{sw},{json.dumps(flat)})'
            )
            logger.info(
                f'[DEM3D] CustomHeightmapTerrainProvider 已載入 {sh}x{sw} 解析度'
            )
            return True
        except Exception as e:
            logger.error(f'[DEM3D] 建立 3D 地形失敗: {e}', exc_info=True)
            return False

    def clear_dem_terrain_3d(self):
        self._js('clearDemTerrain()')

    # ─────────────────────────────────────────────────────────────────
    # SITL 接口（公開方法，供外部 MAVLink 執行緒呼叫）
    # ─────────────────────────────────────────────────────────────────
    def update_uav_position(self, lat: float, lon: float, alt: float,
                             heading_deg: float = 0.0, speed_ms: float = 0.0,
                             sysid: int = 1, mode: str = '', armed: bool = False,
                             vehicle_type: str = '',
                             pitch_deg: float = 0.0, roll_deg: float = 0.0):
        """更新 UAV 即時位置（多機支援，含姿態同步）"""
        mode_js = json.dumps(mode or '')
        vt_js = json.dumps(vehicle_type or '')
        armed_js = 'true' if armed else 'false'
        self._js(
            f'updateUAVPosition({lat},{lon},{alt},{heading_deg},{speed_ms},'
            f'{sysid},{mode_js},{armed_js},{vt_js},{pitch_deg},{roll_deg})'
        )

    def clear_uav(self, sysid: int = None):
        """清除 UAV 位置標記（不指定 sysid 則清全部）"""
        if sysid is None:
            self._js('clearUAV()')
        else:
            self._js(f'clearUAV({sysid})')

    def fly_to_position(self, lat: float, lon: float,
                         alt: float = 0.0, range_m: float = 600.0):
        """相機飛向指定位置"""
        self._js(f'flyToPos({lat},{lon},{alt},{range_m})')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組一：FSDM 高程切片分析
    # ─────────────────────────────────────────────────────────────────
    def update_elevation_slicer(self, min_alt: float, max_alt: float):
        """更新高程切片視覺化（FSDM 地形盲區走廊）"""
        self._js(f'updateElevationSlicer({min_alt},{max_alt})')

    def clear_elevation_slicer(self):
        """清除高程切片"""
        self._js('clearElevationSlicer()')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組二：FOV 光錐 + SAR 搜救機率熱力圖
    # ─────────────────────────────────────────────────────────────────
    def update_fov_cone(self, lat: float, lon: float, alt: float,
                        fov_radius: float = 50.0,
                        heading_deg: float = 0.0,
                        pitch_deg: float = 0.0,
                        roll_deg: float = 0.0):
        """更新 FOV 光錐位置與姿態"""
        self._js(
            f'updateFOVCone({lat},{lon},{alt},{fov_radius},'
            f'{heading_deg},{pitch_deg},{roll_deg})'
        )

    def clear_fov_cone(self):
        """清除 FOV 光錐"""
        self._js('clearFOVCone()')

    def init_sar_heatmap(self, lat_min: float, lat_max: float,
                         lon_min: float, lon_max: float,
                         rows: int = 20, cols: int = 20,
                         sweep_width: float = 50.0,
                         quality: float = 0.8):
        """初始化搜救機率熱力圖"""
        self._js(
            f'initSARHeatmap({lat_min},{lat_max},{lon_min},{lon_max},'
            f'{rows},{cols},{sweep_width},{quality})'
        )

    def update_heatmap(self, uav_lat: float, uav_lon: float,
                       fov_radius: float = 50.0):
        """更新搜救熱力圖（UAV 光錐掃過時更新 COS）"""
        self._js(f'updateHeatmap({uav_lat},{uav_lon},{fov_radius})')

    def clear_sar_heatmap(self):
        """清除搜救熱力圖"""
        self._js('clearSARHeatmap()')

    def reset_sar_heatmap(self):
        """重置搜救熱力圖 COS"""
        self._js('resetSARHeatmap()')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組三：3D 雷達威脅穹頂 + RCS 敏感度
    # ─────────────────────────────────────────────────────────────────
    def add_radar_dome(self, lat: float, lon: float, alt: float = 0.0,
                       radius: float = 5000.0, name: str = ''):
        """新增雷達威脅穹頂"""
        name_js = json.dumps(name or '')
        self._js(f'addRadarDome({lat},{lon},{alt},{radius},{name_js})')

    def clear_radar_domes(self):
        """清除所有雷達穹頂"""
        self._js('clearRadarDomes()')

    def update_rcs_sensitivity(self, uav_lat: float, uav_lon: float,
                               uav_alt: float, uav_heading: float = 0.0,
                               sysid: int = 1):
        """更新 UAV RCS 敏感度渲染"""
        self._js(
            f'updateRCSSensitivity({uav_lat},{uav_lon},{uav_alt},'
            f'{uav_heading},{sysid})'
        )

    def clear_rcs_sensitivity(self, sysid: int = 1):
        """清除 RCS 渲染"""
        self._js(f'clearRCSSensitivity({sysid})')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組四：蜂群打擊 (Swarm Strike) 前端接口
    # ─────────────────────────────────────────────────────────────────
    def strike_set_marking_mode(self, enabled: bool):
        """切換打擊目標標記模式"""
        self._strike_marking = enabled
        self._js(f'strikeSetMarkingMode({"true" if enabled else "false"})')

    def strike_add_target(self, lat: float, lon: float, idx: int):
        """在 3D 地圖上新增打擊目標標記"""
        self._js(f'strikeAddTarget({lat},{lon},{idx})')

    def strike_execute_animation(self, data_json: str, anim_speed: float = 3.0):
        """執行蜂群打擊動畫（舊版：按時間播放 UAV 3D 模型飛行）"""
        self._js(f'strikeExecuteAnimation({json.dumps(data_json)},{anim_speed})')

    def strike_render_path(self, data_json: str):
        """渲染靜態蜂群打擊路徑（起飛=黃 / 巡航=綠 / 俯衝=紅）。

        不播放動畫，直接將 N 架 UCAV 的完整三段飛行路徑顯示為 3D polyline
        供操作員檢視預定航線與俯衝剖面。
        """
        self._js(f'strikeRenderPath({json.dumps(data_json)})')

    def strike_clear_all(self):
        """清除所有打擊視覺化"""
        self._strike_marking = False
        self._js('strikeClearAll()')

    # ── VTOL 3D 軌跡視覺化 ────────────────────────────────────────
    def draw_vtol_swarm_paths(self, uav_data_list: list):
        """繪製 VTOL 群飛三階段 3D 軌跡（垂直黃線+巡航虛線+轉換點閃爍）

        Parameters
        ----------
        uav_data_list : list[dict]
            VTOLMissionExporter.to_cesium_data() 回傳的資料列表。
        """
        import json as _json
        self._js(f'drawVTOLSwarmPaths({_json.dumps(uav_data_list)})')

    def vtol_clear_all(self):
        """清除所有 VTOL 軌跡視覺化"""
        self._js('vtolClearAll()')

    def animate_radar_scan(self, radar_idx: int = 0, duration_ms: int = 2000):
        """播放雷達掃描動畫"""
        self._js(f'animateRadarScan({radar_idx},{duration_ms})')

    # ── Terminal Dive 末端俯衝視覺化 ──────────────────────────
    def render_terminal_dive(self,
                             uav_id: int,
                             trajectory: list,
                             target_lat: float,
                             target_lon: float,
                             target_alt: float = 0.0,
                             dive_threshold: float = 500.0):
        """渲染末端俯衝全套視覺化（俯衝姿態 + 軌跡變色 + 鎖定射線 + 爆炸）

        Parameters
        ----------
        uav_id : int
            UAV 識別碼
        trajectory : list
            航跡資料，每筆格式:
              { 'time': 'ISO8601', 'lat': float, 'lon': float, 'alt': float }
            或 tuple: (iso_time_str, lat, lon, alt)
        target_lat, target_lon, target_alt : float
            地面目標座標（WGS84 度、公尺）
        dive_threshold : float
            俯衝觸發距離門檻（公尺），預設 500m
        """
        # 將 trajectory 正規化為 JS 可用的 JSON 陣列
        traj_data = []
        for pt in trajectory:
            if isinstance(pt, dict):
                traj_data.append({
                    'time': pt['time'],
                    'lat':  float(pt['lat']),
                    'lon':  float(pt['lon']),
                    'alt':  float(pt.get('alt', 0)),
                })
            elif isinstance(pt, (list, tuple)) and len(pt) >= 4:
                traj_data.append({
                    'time': str(pt[0]),
                    'lat':  float(pt[1]),
                    'lon':  float(pt[2]),
                    'alt':  float(pt[3]),
                })

        traj_json = json.dumps(traj_data, ensure_ascii=False)
        target_json = json.dumps({
            'lat': target_lat, 'lon': target_lon, 'alt': target_alt
        })

        self._js(
            f'renderTerminalDive({uav_id}, {traj_json}, {target_json}, {dive_threshold})'
        )
        logger.info(
            f'[TerminalDive] UAV-{uav_id}: {len(traj_data)} 航點, '
            f'目標=({target_lat:.5f},{target_lon:.5f}), 俯衝門檻={dive_threshold}m'
        )

    def clear_terminal_dive(self, uav_id: int):
        """清除指定 UAV 的末端俯衝視覺化"""
        self._js(f'clearTerminalDive({uav_id})')

    def clear_all_terminal_dives(self):
        """清除所有末端俯衝視覺化"""
        self._js('clearAllTerminalDives()')
