"""
地圖組件模組
使用 folium + PyQt6 WebEngine 實現互動式地圖

修正重點：
1. 使用 folium.Map.save() 產生正確 HTML（非 _repr_html_ 的 iframe）
2. JS 注入到 </html> 前（folium 把 Leaflet script 放在 </body> 之後）
3. 用 re 提取確切的 folium map 變數名，直接綁定 Leaflet click 事件
4. 左鍵點擊=添加角點，拖動=移動地圖，右鍵=工具選單
"""

import os
import re
import io
import tempfile
from typing import List, Tuple, Optional

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QMessageBox, QSizePolicy
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage, QWebEngineSettings
from PyQt6.QtCore import pyqtSignal, QUrl, Qt
from urllib.parse import unquote

import folium
from folium import plugins

from config import get_settings
from utils.logger import get_logger

settings = get_settings()
logger = get_logger()

MAX_CORNERS = 100
MIN_CORNERS_FOR_POLYGON = 3


class ClickCapturePage(QWebEnginePage):
    """自定義 WebEngine 頁面，透過 URL scheme 攔截接收 JS 點擊事件"""

    def __init__(self, parent, widget):
        super().__init__(parent)
        self.widget = widget

    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        prefix = {0: 'ℹ️', 1: '⚠️', 2: '❌'}.get(level, '🔹')
        print(f"{prefix} [JS] {message}")

    def acceptNavigationRequest(self, url, nav_type, is_main_frame):
        url_str = url.toString()
        if url_str.startswith('pyqt://click/'):
            try:
                parts = url_str.replace('pyqt://click/', '').split('/')
                lat, lon = float(parts[0]), float(parts[1])
                print(f"📥 [Python] 收到點擊: lat={lat:.6f}, lon={lon:.6f}")
                self.widget.on_map_clicked(lat, lon)
            except Exception as e:
                print(f"❌ [Python] 解析座標失敗: {e}")
            return False
        if url_str.startswith('pyqt://circle_draw/'):
            try:
                parts = url_str.replace('pyqt://circle_draw/', '').split('/')
                lat, lon, radius = float(parts[0]), float(parts[1]), float(parts[2])
                print(f"📥 [Python] 收到圓形定義: lat={lat:.6f}, lon={lon:.6f}, r={radius:.1f}m")
                self.widget.on_circle_draw_complete(lat, lon, radius)
            except Exception as e:
                print(f"❌ [Python] 解析圓形座標失敗: {e}")
            return False
        if url_str.startswith('pyqt://nfz_poly_done/'):
            # 頂點以 "lat,lon|lat,lon|..." 編碼在路徑中
            # Qt/Chromium 會將 '|' URL 編碼為 '%7C'，需先 unquote 才能正確 split
            encoded = unquote(url_str[len('pyqt://nfz_poly_done/'):])
            vertices = []
            if encoded:
                for pair in encoded.split('|'):
                    parts = pair.split(',')
                    if len(parts) == 2:
                        try:
                            vertices.append((float(parts[0]), float(parts[1])))
                        except ValueError:
                            pass
            self.widget.on_nfz_poly_done(vertices)
            return False
        if url_str.startswith('pyqt://view/'):
            try:
                parts = url_str.replace('pyqt://view/', '').split('/')
                lat, lon, zoom = float(parts[0]), float(parts[1]), float(parts[2])
                self.widget._view_state = {'lat': lat, 'lon': lon, 'zoom': zoom}
            except Exception:
                pass
            return False
        if url_str.startswith('pyqt://nfz_circle/'):
            try:
                parts = url_str.replace('pyqt://nfz_circle/', '').split('/')
                lat, lon, radius = float(parts[0]), float(parts[1]), float(parts[2])
                print(f"📥 [Python] 收到 NFZ 圓形: lat={lat:.6f}, lon={lon:.6f}, r={radius:.1f}m")
                self.widget.on_nfz_circle_complete(lat, lon, radius)
            except Exception as e:
                print(f"❌ [Python] 解析 NFZ 圓形失敗: {e}")
            return False
        return True


class MapWidget(QWidget):
    """地圖組件 - folium + PyQt6 WebEngine"""

    corner_added = pyqtSignal(float, float)
    corner_moved = pyqtSignal(int, float, float)
    # 拖曳定義圓形完成：發送 (lat, lon, radius_m)
    circle_defined = pyqtSignal(float, float, float)
    # NFZ 地圖繪製完成信號
    nfz_polygon_drawn = pyqtSignal(list)           # list of (lat, lon) tuples
    nfz_circle_drawn = pyqtSignal(float, float, float)  # lat, lon, radius_m

    def __init__(self, parent=None):
        super().__init__(parent)
        self.corners: List[Tuple[float, float]] = []
        self.markers = []
        self.paths = []
        self.path_colors = []   # 與 self.paths 同長，每條路徑對應一個 hex 顏色
        self._path_tooltips: list = []  # 可選：每條路徑的懸停標籤
        # 轉場路徑：list of dict {'path': [(lat,lon),...], 'altitude': float, 'region_index': int}
        self.transit_paths: list = []
        self.current_map = None
        self.temp_html_file = None
        self.edit_mode = True
        # 圓心掃描圓形覆蓋（持久化，隨 _render_map 一起重建）
        self._circle_center: Optional[Tuple[float, float]] = None
        self._circle_radius_m: float = 0.0
        # 起飛點 Home Point（持久化）
        self._home_point: Optional[Tuple[float, float]] = None
        # 禁航區清單（持久化，由 display_nfz_zones() 設定）
        self._nfz_zones: list = []
        # NFZ 地圖繪製模式
        self._nfz_poly_draw_mode: bool = False
        self._nfz_poly_vertices: list = []  # [(lat, lon), ...]
        # 群飛覆蓋路徑資料（持久化，隨 _render_map 重建）
        self._swarm_data: Optional[dict] = None
        # 使用者目前視角（lat, lon, zoom）— 由 JS moveend 事件即時上報
        # 重新渲染地圖時用此值當中心，避免每次都跳回預設位置
        self._view_state: Optional[dict] = None

        self._init_ui()
        self._init_map()
        logger.info("地圖組件初始化完成")

    # ─────────────────────────────────────
    # UI 初始化
    # ─────────────────────────────────────
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.web_view = QWebEngineView()
        self.web_view.setMinimumSize(400, 400)
        self.web_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.web_view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)

        self.custom_page = ClickCapturePage(self.web_view, self)
        self.web_view.setPage(self.custom_page)

        ws = self.custom_page.settings()
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalStorageEnabled, True)

        layout.addWidget(self.web_view)

    # ─────────────────────────────────────
    # 地圖初始化 & 渲染
    # ─────────────────────────────────────
    def _init_map(self):
        try:
            self.current_map = self._create_base_map()
            self._render_map()
            logger.info("地圖初始化成功")
        except Exception as e:
            logger.error(f"地圖初始化失敗: {e}")
            QMessageBox.critical(self, "地圖錯誤", f"地圖初始化失敗：\n{str(e)}")

    def _create_base_map(self) -> folium.Map:
        """建立基礎 folium Map（含圖層、控制項）

        中心點優先順序：
        1. 使用者目前視角（_view_state）— 由 JS moveend 即時上報
        2. settings 預設座標
        """
        if self._view_state:
            center = (self._view_state['lat'], self._view_state['lon'])
            zoom   = int(self._view_state['zoom'])
        else:
            center = (settings.map.default_lat, settings.map.default_lon)
            zoom   = 20
        m = folium.Map(
            location=center,
            zoom_start=zoom, tiles=None, control_scale=True,
            prefer_canvas=True, max_zoom=22, min_zoom=3,
        )
        folium.TileLayer(
            tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
            attr='Google Satellite', name='Google 衛星',
            overlay=False, control=True, show=True,
        ).add_to(m)
        folium.TileLayer(
            tiles='https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}',
            attr='Google Maps', name='Google 地圖',
            overlay=False, control=True, show=False,
        ).add_to(m)
        folium.TileLayer(
            tiles='OpenStreetMap', name='OpenStreetMap',
            overlay=False, control=True, show=False,
        ).add_to(m)
        folium.LayerControl().add_to(m)
        plugins.Fullscreen().add_to(m)
        plugins.MousePosition().add_to(m)
        plugins.MeasureControl().add_to(m)
        return m

    def _build_map(self) -> folium.Map:
        """重建含所有角點/多邊形的 folium Map"""
        m = self._create_base_map()

        for i, (lat, lon) in enumerate(self.corners):
            folium.Marker(
                location=[lat, lon],
                popup=f'📍 角點 {i + 1}<br>({lat:.6f}, {lon:.6f})',
                icon=folium.DivIcon(
                    html=f'<div style="background:#E53935;color:#fff;border-radius:50%;'
                         f'width:24px;height:24px;display:flex;align-items:center;'
                         f'justify-content:center;font-weight:bold;font-size:11px;'
                         f'border:2px solid #fff;box-shadow:0 2px 6px rgba(0,0,0,.4);">'
                         f'{i + 1}</div>',
                    icon_size=(24, 24), icon_anchor=(12, 12),
                ),
            ).add_to(m)

        if len(self.corners) >= 3:
            folium.Polygon(
                locations=self.corners,
                color='#2196F3', weight=3,
                fill=True, fill_color='#2196F3', fill_opacity=0.15,
            ).add_to(m)
        elif len(self.corners) >= 2:
            folium.PolyLine(
                locations=self.corners,
                color='#2196F3', weight=2, dash_array='6,4',
            ).add_to(m)

        # 渲染已儲存的飛行路徑（支援多區域不同顏色、固定翼三階段 tooltip）
        _default_color = '#08EC91'
        for idx, path in enumerate(self.paths):
            if len(path) >= 2:
                color = (self.path_colors[idx]
                         if idx < len(self.path_colors)
                         else _default_color)
                tooltip = (self._path_tooltips[idx]
                           if hasattr(self, '_path_tooltips') and idx < len(self._path_tooltips)
                           else None)
                poly_kw = dict(locations=path, color=color, weight=3, opacity=0.9)
                if tooltip:
                    poly_kw['tooltip'] = tooltip
                folium.PolyLine(**poly_kw).add_to(m)

                # 起訖標記：依 tooltip 內容決定，支援分區多段
                tip = (self._path_tooltips[idx]
                       if hasattr(self, '_path_tooltips') and idx < len(self._path_tooltips)
                       else '')
                if tip == '起飛爬升路徑':
                    start_lbl, end_lbl = '起飛起點', '爬升完成'
                elif tip == '五邊進場路徑':
                    start_lbl, end_lbl = '進五邊', '觸地點'
                elif tip:
                    start_lbl, end_lbl = f'{tip} 起點', f'{tip} 終點'
                else:
                    start_lbl, end_lbl = f'區域{idx+1} 起點', f'區域{idx+1} 終點'
                folium.Marker(
                    location=path[0],
                    popup=start_lbl,
                    icon=folium.Icon(color='green', icon='play'),
                ).add_to(m)
                folium.Marker(
                    location=path[-1],
                    popup=end_lbl,
                    icon=folium.Icon(color='red', icon='stop'),
                ).add_to(m)
                for pt in path[1:-1]:
                    folium.CircleMarker(
                        location=pt, radius=3, color=color,
                        fill=True, fill_color=color, fill_opacity=0.7,
                    ).add_to(m)

        # 渲染轉場路徑（虛線＋高度標籤，避免與覆蓋路徑混淆）
        for tp in self.transit_paths:
            t_path = tp.get('path', [])
            t_alt = tp.get('altitude', 0)
            t_idx = tp.get('region_index', 0)
            if len(t_path) < 2:
                continue
            t_color = self._REGION_COLORS[t_idx % len(self._REGION_COLORS)]
            folium.PolyLine(
                locations=t_path,
                color=t_color, weight=2, opacity=0.7,
                dash_array='8,5',
                tooltip=f'轉場路徑 → 區域{t_idx+1}（高度 {t_alt:.0f}m）',
            ).add_to(m)
            # 起飛點標記（僅第一條轉場路徑畫起飛點，避免重複）
            if t_idx == 0:
                folium.Marker(
                    location=t_path[0],
                    popup='起飛點 (Home)',
                    icon=folium.Icon(color='black', icon='home', prefix='fa'),
                ).add_to(m)
            # 各子區域進入點
            folium.CircleMarker(
                location=t_path[-1],
                radius=6, color=t_color,
                fill=True, fill_color=t_color, fill_opacity=0.9,
                tooltip=f'區域{t_idx+1} 進入點（{t_alt:.0f}m）',
            ).add_to(m)

        # 圓心掃描圓形覆蓋（folium.Circle 在 _build_map 中渲染，隨頁面持久存在）
        if self._circle_center is not None:
            c_lat, c_lon = self._circle_center
            folium.Circle(
                location=[c_lat, c_lon],
                radius=self._circle_radius_m,
                color='#2196F3', weight=2,
                fill=True, fill_color='#2196F3', fill_opacity=0.06,
                dash_array='6 4',
                popup=f'掃描範圍：半徑 {self._circle_radius_m:.0f} m',
            ).add_to(m)
            folium.CircleMarker(
                location=[c_lat, c_lon],
                radius=7, color='#2196F3',
                fill=True, fill_color='#2196F3', fill_opacity=0.9,
                popup=f'圓心：{c_lat:.6f}, {c_lon:.6f}',
                tooltip='螺旋/圓心掃描中心',
            ).add_to(m)

        # 禁航區 NFZ 視覺化（紅色多邊形/圓形）
        for nfz in self._nfz_zones:
            nfz_name = nfz.get('name', 'NFZ')
            if nfz['type'] == 'polygon':
                verts = nfz['vertices']
                folium.Polygon(
                    locations=[[lat, lon] for lat, lon in verts],
                    color='#D32F2F', weight=2,
                    fill=True, fill_color='#EF5350', fill_opacity=0.25,
                    popup=f'🚫 禁航區：{nfz_name}（多邊形，{len(verts)} 頂點）',
                    tooltip=f'NFZ: {nfz_name}',
                ).add_to(m)
            elif nfz['type'] == 'circle':
                clat, clon = nfz['center']
                folium.Circle(
                    location=[clat, clon],
                    radius=nfz['radius'],
                    color='#D32F2F', weight=2,
                    fill=True, fill_color='#EF5350', fill_opacity=0.25,
                    popup=f'🚫 禁航區：{nfz_name}（圓形，r={nfz["radius"]:.0f}m）',
                    tooltip=f'NFZ: {nfz_name}',
                ).add_to(m)

        # 起飛點 Home Point 標記
        if self._home_point is not None:
            h_lat, h_lon = self._home_point
            folium.Marker(
                location=[h_lat, h_lon],
                popup=f'🏠 起飛/降落點：{h_lat:.6f}, {h_lon:.6f}',
                tooltip='起飛點 / 降落點',
                icon=folium.Icon(color='black', icon='home', prefix='fa'),
            ).add_to(m)

        # === 群飛覆蓋路徑渲染 ===
        if self._swarm_data:
            for drone_info in self._swarm_data.get('drones', []):
                drone_id = drone_info['drone_id']
                color = self._DRONE_COLORS[(drone_id - 1) % len(self._DRONE_COLORS)]

                # 起飛段（粗綠實線）
                for tk_path in drone_info.get('takeoff_paths', []):
                    if len(tk_path) >= 2:
                        folium.PolyLine(
                            locations=tk_path,
                            color='#2e7d32', weight=5, opacity=0.95,
                            tooltip=f'Drone {drone_id} 起飛',
                        ).add_to(m)

                # 進入段（淺色點虛線）
                for en_path in drone_info.get('entry_paths', []):
                    if len(en_path) >= 2:
                        folium.PolyLine(
                            locations=en_path,
                            color='#26c6da', weight=3, opacity=0.9,
                            dash_array='2,6',
                            tooltip=f'Drone {drone_id} 進入段',
                        ).add_to(m)

                # 作業段（實粗線，主色）
                for op_path in drone_info.get('operation_paths', []):
                    if len(op_path) >= 2:
                        folium.PolyLine(
                            locations=op_path,
                            color=color, weight=5, opacity=0.95,
                            tooltip=f'Drone {drone_id} 作業段',
                        ).add_to(m)

                # 轉移段（長虛線，主色 淡化）
                for tr_path in drone_info.get('transfer_paths', []):
                    if len(tr_path) >= 2:
                        folium.PolyLine(
                            locations=tr_path,
                            color=color, weight=2, opacity=0.55,
                            dash_array='12,8',
                            tooltip=f'Drone {drone_id} 轉移段',
                        ).add_to(m)

                # 降落段（橘色 dash-dot）
                for ld_path in drone_info.get('landing_paths', []):
                    if len(ld_path) >= 2:
                        folium.PolyLine(
                            locations=ld_path,
                            color='#ef6c00', weight=4, opacity=0.95,
                            dash_array='10,5,2,5',
                            tooltip=f'Drone {drone_id} 降落',
                        ).add_to(m)

                # 無人機起點標記
                start_pos = drone_info.get('start_position')
                if start_pos:
                    folium.Marker(
                        location=start_pos,
                        popup=f'Drone {drone_id} 起點',
                        icon=folium.DivIcon(
                            html=(
                                f'<div style="background:{color};color:#fff;'
                                f'border-radius:50%;width:28px;height:28px;'
                                f'display:flex;align-items:center;justify-content:center;'
                                f'font-weight:bold;font-size:12px;'
                                f'border:2px solid #fff;box-shadow:0 2px 6px rgba(0,0,0,.4);">'
                                f'U{drone_id}</div>'
                            ),
                            icon_size=(28, 28), icon_anchor=(14, 14),
                        ),
                    ).add_to(m)

            # 渲染區域邊界
            for area_info in self._swarm_data.get('areas', []):
                poly = area_info.get('polygon', [])
                if len(poly) >= 3:
                    folium.Polygon(
                        locations=poly,
                        color='#FFC107', weight=2,
                        fill=True, fill_color='#FFC107', fill_opacity=0.08,
                        tooltip=f"區域 {area_info.get('area_id', '?')}",
                    ).add_to(m)

            # 圖例
            legend_html = self._build_swarm_legend(self._swarm_data)
            m.get_root().html.add_child(folium.Element(legend_html))

        if self.corners:
            m.fit_bounds(
                [[min(c[0] for c in self.corners), min(c[1] for c in self.corners)],
                 [max(c[0] for c in self.corners), max(c[1] for c in self.corners)]],
                padding=[50, 50],
            )
        elif self._circle_center is not None:
            # 無多邊形時，以圓形範圍自動縮放
            c_lat, c_lon = self._circle_center
            r_deg = self._circle_radius_m / 111320.0
            m.fit_bounds(
                [[c_lat - r_deg, c_lon - r_deg],
                 [c_lat + r_deg, c_lon + r_deg]],
                padding=[40, 40],
            )

        return m

    def _rewrite_leaflet_to_local(self, html: str) -> str:
        """將 folium 產生的 leaflet CDN URL 換為本地 assets/leaflet/ 路徑（離線可用）

        若本地檔案不存在則保留原 CDN 路徑（線上模式）。
        """
        from pathlib import Path
        project_root = Path(__file__).resolve().parent.parent.parent
        local_js  = project_root / 'assets' / 'leaflet' / 'leaflet.js'
        local_css = project_root / 'assets' / 'leaflet' / 'leaflet.css'
        if not (local_js.exists() and local_css.exists()):
            return html

        js_uri  = local_js.as_uri()
        css_uri = local_css.as_uri()

        # folium 用 jsdelivr / unpkg / cdnjs 等 CDN，全部換掉
        patterns_js = [
            r'https?://cdn\.jsdelivr\.net/npm/leaflet@[^"\'>]*?leaflet\.js',
            r'https?://unpkg\.com/leaflet@[^"\'>]*?leaflet\.js',
            r'https?://cdnjs\.cloudflare\.com/ajax/libs/leaflet/[^"\'>]*?leaflet\.js',
        ]
        patterns_css = [
            r'https?://cdn\.jsdelivr\.net/npm/leaflet@[^"\'>]*?leaflet\.css',
            r'https?://unpkg\.com/leaflet@[^"\'>]*?leaflet\.css',
            r'https?://cdnjs\.cloudflare\.com/ajax/libs/leaflet/[^"\'>]*?leaflet\.css',
        ]
        for pat in patterns_js:
            html = re.sub(pat, js_uri, html)
        for pat in patterns_css:
            html = re.sub(pat, css_uri, html)
        return html

    def _render_map(self):
        """渲染地圖到 WebView"""
        try:
            m = self._build_map()

            # 用 save() 產生完整 HTML（非 _repr_html_ 的 iframe 格式）
            buf = io.BytesIO()
            m.save(buf, close_file=False)
            html = buf.getvalue().decode('utf-8')

            # 將 folium 預設 CDN 換為本地 leaflet（離線可用）
            html = self._rewrite_leaflet_to_local(html)

            # 從 HTML 提取確切的 folium 地圖變數名
            match = re.search(r'var\s+(map_[a-f0-9]+)\s*=\s*L\.map', html)
            map_var_name = match.group(1) if match else None

            if not map_var_name:
                logger.error("無法從 HTML 提取 folium 地圖變數名！")
                return

            # 注入互動 JS
            html = self._inject_javascript(html, map_var_name)

            # 清理舊檔案
            if self.temp_html_file:
                try:
                    os.unlink(self.temp_html_file)
                except OSError:
                    pass

            with tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False, encoding='utf-8') as f:
                f.write(html)
                self.temp_html_file = f.name

            self.web_view.setUrl(QUrl.fromLocalFile(self.temp_html_file))

        except Exception as e:
            logger.error(f"渲染地圖失敗: {e}")

    # ─────────────────────────────────────
    # JavaScript 注入（核心修正）
    # ─────────────────────────────────────
    def _inject_javascript(self, html: str, map_var_name: str) -> str:
        """
        注入互動 JS - 直接用確切的 folium 地圖變數名綁定 Leaflet click

        關鍵：folium save() 的 HTML 結構是：
            <body> div </body>
            <script> var map_xxx = L.map(...) </script>
            </html>
        所以我們的 JS 必須注入在 </html> 前面（Leaflet 初始化之後）。
        
        使用 Leaflet 原生 click 事件，不用 DOM mousedown/mouseup
        （DOM 事件會被 Leaflet dragging handler 攔截）。
        """
        corner_count = len(self.corners)

        js_code = f"""
<style>
html, body {{ margin:0; padding:0; width:100%; height:100%; overflow:hidden; }}
.folium-map {{ width:100% !important; height:100vh !important; }}
body > div {{ width:100% !important; height:100% !important; }}
.leaflet-control, .leaflet-control * {{ cursor:pointer !important; }}

.corner-counter {{
    position:absolute; bottom:30px; left:10px;
    background:rgba(33,150,243,.95); color:#fff;
    padding:10px 15px; border-radius:6px;
    font-size:14px; font-weight:bold;
    z-index:1000; pointer-events:none;
    box-shadow:0 2px 10px rgba(0,0,0,.3);
}}
.ctx-menu {{
    position:fixed; z-index:10000; background:#fff;
    border-radius:8px; box-shadow:0 4px 20px rgba(0,0,0,.25);
    min-width:220px; padding:6px 0;
    font-family:'Segoe UI',sans-serif; font-size:13px; display:none;
}}
.ctx-menu .ctx-header {{
    padding:8px 14px; font-size:11px; color:#888;
    border-bottom:1px solid #eee; user-select:text; font-family:monospace;
}}
.ctx-menu .ctx-item {{
    padding:8px 14px; cursor:pointer;
    display:flex; align-items:center; gap:8px;
}}
.ctx-menu .ctx-item:hover {{ background:#e3f2fd; }}
.ctx-menu .ctx-sep {{ height:1px; background:#eee; margin:4px 0; }}

.usage-hint {{
    position:absolute; top:10px; left:50%; transform:translateX(-50%);
    background:rgba(76,175,80,.92); color:#fff;
    padding:8px 16px; border-radius:5px; font-size:13px;
    z-index:1000; pointer-events:none;
    box-shadow:0 2px 8px rgba(0,0,0,.3);
    animation:fadeHint 6s forwards;
}}
@keyframes fadeHint {{ 0%,75%{{opacity:1}} 100%{{opacity:0}} }}
.cd-radius-tip {{
    background: rgba(255,87,34,.92) !important;
    border: none !important;
    color: #fff !important;
    font-size: 12px !important;
    font-weight: bold !important;
    padding: 4px 8px !important;
    border-radius: 4px !important;
    white-space: nowrap !important;
    box-shadow: 0 2px 6px rgba(0,0,0,.3) !important;
}}
.cd-radius-tip::before {{ display: none !important; }}
</style>

<script>
// ═══════════════════════════════════════════════════════════
// 直接引用 folium 生成的地圖變數（不再動態搜尋 window）
// ═══════════════════════════════════════════════════════════
(function() {{
    var cornerCount = {corner_count};
    var maxCorners = {MAX_CORNERS};
    var _ctxMenu = null;
    var _lastCtxLatLng = null;
    var _measureStart = null;
    var _tempPins = [];

    // 直接引用 folium 的地圖變數
    var mapObj = {map_var_name};

    if (!mapObj || typeof mapObj.on !== 'function') {{
        console.error('❌ 地圖物件無效: {map_var_name}');
        return;
    }}
    console.log('✅ 直接引用地圖物件: {map_var_name}');

    window.currentMap = mapObj;
    window._cornerMarkers = [];
    window._boundaryLayer = null;
    mapObj.invalidateSize();

    // ─── UI 元素 ───
    var hint = document.createElement('div');
    hint.className = 'usage-hint';
    hint.innerHTML = '💡 左鍵點擊=添加角點 | 拖動=移動地圖 | 右鍵=工具選單';
    mapObj._container.appendChild(hint);

    var counter = document.createElement('div');
    counter.className = 'corner-counter';
    counter.id = 'corner-counter';
    mapObj._container.appendChild(counter);
    updateCounter();

    // 右鍵選單 DOM
    _ctxMenu = document.createElement('div');
    _ctxMenu.className = 'ctx-menu';
    document.body.appendChild(_ctxMenu);
    document.addEventListener('mousedown', function(e) {{
        if (_ctxMenu && !_ctxMenu.contains(e.target)) _ctxMenu.style.display = 'none';
    }});

    // ═══════════════════════════════════════════════════════════
    // 核心：使用 Leaflet 原生 click 事件（最可靠）
    // Leaflet 內部已處理 click vs drag 的區分
    // ═══════════════════════════════════════════════════════════
    // ═══════════════════════════════════════════════════════════
    // 拖曳定義圓形模式（Mission Planner 風格：按住拖曳設定圓心+半徑）
    // ═══════════════════════════════════════════════════════════
    window.circleDrawMode = false;
    var _cdStart = null;
    var _cdDragging = false;
    var _cdCirclePreview = null;
    var _cdCenterMarker = null;
    var _cdRadiusLine = null;
    var _cdRadiusLabel = null;

    function _cleanupCircleDraw() {{
        if (_cdCirclePreview) {{ mapObj.removeLayer(_cdCirclePreview); _cdCirclePreview = null; }}
        if (_cdCenterMarker)  {{ mapObj.removeLayer(_cdCenterMarker);  _cdCenterMarker = null; }}
        if (_cdRadiusLine)    {{ mapObj.removeLayer(_cdRadiusLine);     _cdRadiusLine = null; }}
        if (_cdRadiusLabel)   {{ mapObj.removeLayer(_cdRadiusLabel);    _cdRadiusLabel = null; }}
    }}

    window.enableCircleDraw = function() {{
        window.circleDrawMode = true;
        _cdStart = null; _cdDragging = false;
        mapObj._container.style.cursor = 'crosshair';
        toast('🎯 按住滑鼠並拖曳以定義圓形掃描區域');
        console.log('✅ 圓形拖曳模式已啟用');
    }};
    window.disableCircleDraw = function() {{
        window.circleDrawMode = false;
        _cdStart = null; _cdDragging = false;
        mapObj._container.style.cursor = '';
        _cleanupCircleDraw();
        console.log('✅ 圓形拖曳模式已停用');
    }};

    mapObj.on('mousedown', function(e) {{
        if (!window.circleDrawMode) return;
        if (e.originalEvent.button !== 0) return;
        _cdStart = e.latlng;
        _cdDragging = false;
        mapObj.dragging.disable();
        if (_cdCenterMarker) mapObj.removeLayer(_cdCenterMarker);
        _cdCenterMarker = L.circleMarker([_cdStart.lat, _cdStart.lng], {{
            radius: 7, color: '#2196F3', fillColor: '#2196F3', fillOpacity: 1, weight: 2
        }}).addTo(mapObj);
        L.DomEvent.stopPropagation(e);
    }});

    mapObj.on('mousemove', function(e) {{
        if (!window.circleDrawMode || !_cdStart) return;
        _cdDragging = true;
        var dist = mapObj.distance(_cdStart, e.latlng);
        // 即時更新圓形預覽
        if (_cdCirclePreview) mapObj.removeLayer(_cdCirclePreview);
        _cdCirclePreview = L.circle([_cdStart.lat, _cdStart.lng], {{
            radius: Math.max(dist, 1),
            color: '#2196F3', weight: 2,
            fillColor: '#2196F3', fillOpacity: 0.07,
            dashArray: '7 4'
        }}).addTo(mapObj);
        // 半徑指示線
        if (_cdRadiusLine) mapObj.removeLayer(_cdRadiusLine);
        _cdRadiusLine = L.polyline([[_cdStart.lat, _cdStart.lng],
                                    [e.latlng.lat, e.latlng.lng]], {{
            color: '#FF5722', weight: 1.5, dashArray: '5 4', opacity: 0.85
        }}).addTo(mapObj);
        // 半徑標籤
        if (_cdRadiusLabel) mapObj.removeLayer(_cdRadiusLabel);
        var label = dist < 1000 ? dist.toFixed(0) + ' m' : (dist/1000).toFixed(2) + ' km';
        var midLat = (_cdStart.lat + e.latlng.lat) / 2;
        var midLng = (_cdStart.lng + e.latlng.lng) / 2;
        _cdRadiusLabel = L.tooltip({{ permanent: true, direction: 'center',
                                       className: 'cd-radius-tip' }})
            .setLatLng([midLat, midLng])
            .setContent('R: ' + label)
            .addTo(mapObj);
    }});

    mapObj.on('mouseup', function(e) {{
        if (!window.circleDrawMode || !_cdStart) return;
        mapObj.dragging.enable();
        var dist = _cdDragging ? mapObj.distance(_cdStart, e.latlng) : 0;
        var startLat = _cdStart.lat, startLng = _cdStart.lng;
        window.circleDrawMode = false;
        mapObj._container.style.cursor = '';
        _cdStart = null; _cdDragging = false;
        _cleanupCircleDraw();
        // 送出給 Python（radius=0 表示僅設圓心不更新半徑）
        window.location.href = 'pyqt://circle_draw/' + startLat + '/' + startLng + '/' + dist.toFixed(1);
    }});

    mapObj.on('click', function(e) {{
        // 圓形拖曳模式中，mouseup 已處理，忽略後續觸發的 click
        if (window.circleDrawMode) {{ L.DomEvent.stopPropagation(e); return; }}
        if (window.nfzCircleDrawMode) {{ L.DomEvent.stopPropagation(e); return; }}

        var lat = e.latlng.lat;
        var lng = e.latlng.lng;

        // NFZ 多邊形繪製模式：將點加入 NFZ 頂點（僅存 JS，不發 URL 避免閃爍）
        if (window.nfzPolyDrawMode) {{
            // 忽略雙擊的第二次 click 事件
            if (e.originalEvent && e.originalEvent.detail >= 2) return;
            window._nfzPolyVerts.push([lat, lng]);
            var mk = L.circleMarker([lat, lng], {{
                radius: 6, color: '#B71C1C', fillColor: '#B71C1C', fillOpacity: 1, weight: 2
            }}).addTo(mapObj);
            window._nfzPolyLayers.push(mk);
            if (window._nfzPolyOutline) {{ mapObj.removeLayer(window._nfzPolyOutline); window._nfzPolyOutline = null; }}
            var verts = window._nfzPolyVerts;
            if (verts.length >= 3) {{
                window._nfzPolyOutline = L.polygon(verts, {{
                    color: '#B71C1C', weight: 2, fillColor: '#B71C1C', fillOpacity: 0.08, dashArray: '6,3'
                }}).addTo(mapObj);
            }} else if (verts.length >= 2) {{
                window._nfzPolyOutline = L.polyline(verts, {{color: '#B71C1C', weight: 2, dashArray: '6,3'}}).addTo(mapObj);
            }}
            // 更新計數器顯示（不用 toast，避免深色疊加）
            var ctr = document.getElementById('corner-counter');
            if (ctr) {{
                ctr.innerHTML = '🚫 NFZ 頂點: ' + verts.length +
                    '<br><small style="opacity:.8">雙擊或按「完成」結束</small>';
                ctr.style.background = 'rgba(183,28,28,.92)';
            }}
            return;
        }}

        console.log('🖱️ [Leaflet click] lat=' + lat.toFixed(6) + ', lng=' + lng.toFixed(6));

        // 通知 Python
        window.location.href = 'pyqt://click/' + lat + '/' + lng;

        // 更新計數
        cornerCount++;
        updateCounter();

        // 脈衝反饋
        var pulse = L.circleMarker([lat, lng], {{
            radius: 12, color: '#2196F3', fillColor: '#2196F3',
            fillOpacity: 0.6, weight: 3
        }}).addTo(mapObj);
        setTimeout(function() {{ mapObj.removeLayer(pulse); }}, 500);
    }});

    // 雙擊完成 NFZ 多邊形
    mapObj.on('dblclick', function(e) {{
        if (!window.nfzPolyDrawMode) return;
        L.DomEvent.stopPropagation(e);
        window.finishNFZPoly();
    }});
    console.log('✅ Leaflet click 事件已綁定');

    // ─── 視角同步：每次平移/縮放完成後上報 Python ───
    function _reportView() {{
        try {{
            var c = mapObj.getCenter();
            var z = mapObj.getZoom();
            // 用 fetch 而非 location.href 避免打斷使用者操作
            var img = new Image();
            img.src = 'pyqt://view/' + c.lat + '/' + c.lng + '/' + z;
        }} catch(e) {{}}
    }}
    mapObj.on('moveend', _reportView);
    mapObj.on('zoomend', _reportView);
    setTimeout(_reportView, 300);  // 初次載入也回報一次

    // ─── 右鍵：工具選單（用 Leaflet contextmenu） ───
    mapObj.on('contextmenu', function(e) {{
        L.DomEvent.preventDefault(e.originalEvent);
        L.DomEvent.stopPropagation(e.originalEvent);
        _lastCtxLatLng = e.latlng;
        showCtxMenu(e.originalEvent.clientX, e.originalEvent.clientY, e.latlng);
    }});
    console.log('✅ 右鍵選單事件已綁定');

    // ─── 右鍵選單函數 ───
    function showCtxMenu(x, y, ll) {{
        var lat = ll.lat.toFixed(8), lng = ll.lng.toFixed(8);
        _ctxMenu.innerHTML =
            '<div class="ctx-header">📍 ' + lat + ', ' + lng + '</div>' +
            '<div class="ctx-item" id="ctx-copy">📋 複製座標</div>' +
            '<div class="ctx-item" id="ctx-add">📌 在此添加角點</div>' +
            '<div class="ctx-sep"></div>' +
            '<div class="ctx-item" id="ctx-center">🎯 將此處置中</div>' +
            '<div class="ctx-item" id="ctx-zin">🔍 放大</div>' +
            '<div class="ctx-item" id="ctx-zout">🔎 縮小</div>' +
            '<div class="ctx-sep"></div>' +
            '<div class="ctx-item" id="ctx-measure">📏 測距</div>' +
            '<div class="ctx-item" id="ctx-pin">📍 臨時標記</div>';

        var w = window.innerWidth, h = window.innerHeight;
        if (x + 230 > w) x = w - 235;
        if (y + 320 > h) y = h - 325;
        _ctxMenu.style.left = Math.max(0, x) + 'px';
        _ctxMenu.style.top = Math.max(0, y) + 'px';
        _ctxMenu.style.display = 'block';

        document.getElementById('ctx-copy').onclick = function() {{
            _ctxMenu.style.display = 'none';
            var t = _lastCtxLatLng.lat.toFixed(8) + ', ' + _lastCtxLatLng.lng.toFixed(8);
            navigator.clipboard.writeText(t).then(function() {{ toast('✅ 已複製: ' + t); }})
                .catch(function() {{ prompt('座標:', t); }});
        }};
        document.getElementById('ctx-add').onclick = function() {{
            _ctxMenu.style.display = 'none';
            window.location.href = 'pyqt://click/' + _lastCtxLatLng.lat + '/' + _lastCtxLatLng.lng;
            cornerCount++; updateCounter();
            pulse(_lastCtxLatLng, '#4CAF50');
        }};
        document.getElementById('ctx-center').onclick = function() {{
            _ctxMenu.style.display = 'none';
            mapObj.panTo(_lastCtxLatLng);
        }};
        document.getElementById('ctx-zin').onclick = function() {{
            _ctxMenu.style.display = 'none'; mapObj.zoomIn();
        }};
        document.getElementById('ctx-zout').onclick = function() {{
            _ctxMenu.style.display = 'none'; mapObj.zoomOut();
        }};
        document.getElementById('ctx-measure').onclick = function() {{
            _ctxMenu.style.display = 'none';
            if (!_measureStart) {{
                _measureStart = _lastCtxLatLng;
                L.circleMarker([_measureStart.lat, _measureStart.lng], {{
                    radius: 6, color: '#FF5722', fillColor: '#FF5722', fillOpacity: 1
                }}).addTo(mapObj);
                toast('📏 已標記起點，右鍵另一處再選測距');
            }} else {{
                var d = mapObj.distance(_measureStart, _lastCtxLatLng);
                L.polyline([[_measureStart.lat, _measureStart.lng],
                            [_lastCtxLatLng.lat, _lastCtxLatLng.lng]],
                    {{ color: '#FF5722', weight: 2, dashArray: '6,4' }}).addTo(mapObj);
                var mid = L.latLng(
                    (_measureStart.lat + _lastCtxLatLng.lat) / 2,
                    (_measureStart.lng + _lastCtxLatLng.lng) / 2
                );
                var label = d < 1000 ? d.toFixed(1) + ' m' : (d / 1000).toFixed(3) + ' km';
                L.tooltip({{ permanent: true, direction: 'center' }})
                    .setLatLng(mid).setContent('📏 ' + label).addTo(mapObj);
                toast('📏 距離: ' + label);
                _measureStart = null;
            }}
        }};
        document.getElementById('ctx-pin').onclick = function() {{
            _ctxMenu.style.display = 'none';
            var mk = L.marker([_lastCtxLatLng.lat, _lastCtxLatLng.lng], {{
                icon: L.divIcon({{
                    html: '<div style="font-size:24px">📍</div>',
                    iconSize: [24, 24], iconAnchor: [12, 24], className: ''
                }})
            }}).addTo(mapObj);
            mk.bindPopup('臨時標記<br>' + _lastCtxLatLng.lat.toFixed(6) + ', ' + _lastCtxLatLng.lng.toFixed(6));
            _tempPins.push(mk);
        }};
    }}

    function pulse(ll, color) {{
        var p = L.circleMarker([ll.lat, ll.lng], {{
            radius: 12, color: color, fillColor: color, fillOpacity: .6, weight: 3
        }}).addTo(mapObj);
        setTimeout(function() {{ mapObj.removeLayer(p); }}, 500);
    }}

    function toast(msg) {{
        var t = document.createElement('div');
        t.style.cssText = 'position:fixed;bottom:60px;left:50%;transform:translateX(-50%);' +
            'background:rgba(0,0,0,.8);color:#fff;padding:8px 18px;border-radius:6px;' +
            'font-size:13px;z-index:99999;pointer-events:none;';
        t.textContent = msg;
        document.body.appendChild(t);
        setTimeout(function() {{ t.style.opacity = '0'; t.style.transition = 'opacity .3s'; }}, 2000);
        setTimeout(function() {{ document.body.removeChild(t); }}, 2500);
    }}

    function updateCounter() {{
        var el = document.getElementById('corner-counter');
        if (!el) return;
        el.innerHTML = '📍 角點: ' + cornerCount + ' / ' + maxCorners +
            '<br><small style="opacity:.8">🖱️ 左鍵點擊添加</small>';
        if (cornerCount >= maxCorners) el.style.background = 'rgba(244,67,54,.95)';
        else if (cornerCount >= 3) el.style.background = 'rgba(76,175,80,.95)';
        else el.style.background = 'rgba(33,150,243,.95)';
    }}

    // 全域函數供外部調用
    window.resetCornerCounter = function() {{ cornerCount = 0; updateCounter(); }};
    window.setCornerCount = function(n) {{ cornerCount = n; updateCounter(); }};

    // ═══════════════════════════════════════════════════════════
    // NFZ 多邊形繪製模式
    // ═══════════════════════════════════════════════════════════
    window.nfzPolyDrawMode = false;
    window._nfzPolyVerts = [];
    window._nfzPolyLayers = [];
    window._nfzPolyOutline = null;

    function _cleanupNFZPoly() {{
        window._nfzPolyVerts = [];
        window._nfzPolyLayers.forEach(function(l) {{ mapObj.removeLayer(l); }});
        window._nfzPolyLayers = [];
        if (window._nfzPolyOutline) {{ mapObj.removeLayer(window._nfzPolyOutline); window._nfzPolyOutline = null; }}
    }}

    window.enableNFZPolyDraw = function() {{
        window.nfzPolyDrawMode = true;
        _cleanupNFZPoly();
        mapObj._container.style.cursor = 'crosshair';
        var ctr = document.getElementById('corner-counter');
        if (ctr) {{
            ctr.innerHTML = '🚫 NFZ 頂點: 0<br><small style="opacity:.8">點擊新增，雙擊完成</small>';
            ctr.style.background = 'rgba(183,28,28,.92)';
        }}
        console.log('✅ NFZ 多邊形繪製模式已啟用');
    }};
    window.disableNFZPolyDraw = function() {{
        window.nfzPolyDrawMode = false;
        mapObj._container.style.cursor = '';
        _cleanupNFZPoly();
        // 恢復角點計數器
        if (typeof window.setCornerCount === 'function') window.setCornerCount(cornerCount);
        var ctr = document.getElementById('corner-counter');
        if (ctr) ctr.style.background = cornerCount >= 3 ? 'rgba(76,175,80,.95)' : 'rgba(33,150,243,.95)';
        console.log('✅ NFZ 多邊形繪製模式已停用');
    }};
    window.finishNFZPoly = function() {{
        if (window._nfzPolyVerts.length < 3) {{
            toast('⚠️ 至少需要 3 個頂點才能完成 NFZ 多邊形');
            return;
        }}
        // 先把頂點編碼（_cleanupNFZPoly 會清空陣列）
        var encoded = window._nfzPolyVerts.map(function(v) {{
            return v[0].toFixed(8) + ',' + v[1].toFixed(8);
        }}).join('|');
        window.nfzPolyDrawMode = false;
        mapObj._container.style.cursor = '';
        _cleanupNFZPoly();
        // 恢復計數器
        var ctr = document.getElementById('corner-counter');
        if (ctr) {{
            if (typeof window.setCornerCount === 'function') window.setCornerCount(cornerCount);
            ctr.style.background = cornerCount >= 3 ? 'rgba(76,175,80,.95)' : 'rgba(33,150,243,.95)';
        }}
        // 一次性發送所有頂點
        window.location.href = 'pyqt://nfz_poly_done/' + encoded;
    }};

    // ═══════════════════════════════════════════════════════════
    // NFZ 圓形拖曳模式（紅色，獨立於 mission 圓形模式）
    // ═══════════════════════════════════════════════════════════
    window.nfzCircleDrawMode = false;
    var _ncdStart = null, _ncdDragging = false;
    var _ncdPreview = null, _ncdCenterMk = null, _ncdLine = null, _ncdLabel = null;

    function _cleanupNFZCircle() {{
        if (_ncdPreview)  {{ mapObj.removeLayer(_ncdPreview);  _ncdPreview = null; }}
        if (_ncdCenterMk) {{ mapObj.removeLayer(_ncdCenterMk); _ncdCenterMk = null; }}
        if (_ncdLine)     {{ mapObj.removeLayer(_ncdLine);     _ncdLine = null; }}
        if (_ncdLabel)    {{ mapObj.removeLayer(_ncdLabel);    _ncdLabel = null; }}
    }}

    window.enableNFZCircleDraw = function() {{
        window.nfzCircleDrawMode = true;
        _ncdStart = null; _ncdDragging = false;
        mapObj._container.style.cursor = 'crosshair';
        toast('🚫 按住滑鼠拖曳定義 NFZ 圓形禁航區');
        console.log('✅ NFZ 圓形拖曳模式已啟用');
    }};
    window.disableNFZCircleDraw = function() {{
        window.nfzCircleDrawMode = false;
        _ncdStart = null; _ncdDragging = false;
        mapObj._container.style.cursor = '';
        _cleanupNFZCircle();
        console.log('✅ NFZ 圓形拖曳模式已停用');
    }};

    mapObj.on('mousedown', function(e) {{
        if (!window.nfzCircleDrawMode) return;
        if (e.originalEvent.button !== 0) return;
        _ncdStart = e.latlng; _ncdDragging = false;
        mapObj.dragging.disable();
        _ncdCenterMk = L.circleMarker([_ncdStart.lat, _ncdStart.lng], {{
            radius: 7, color: '#B71C1C', fillColor: '#B71C1C', fillOpacity: 1, weight: 2
        }}).addTo(mapObj);
        L.DomEvent.stopPropagation(e);
    }});

    mapObj.on('mousemove', function(e) {{
        if (!window.nfzCircleDrawMode || !_ncdStart) return;
        _ncdDragging = true;
        var dist = mapObj.distance(_ncdStart, e.latlng);
        if (_ncdPreview) mapObj.removeLayer(_ncdPreview);
        _ncdPreview = L.circle([_ncdStart.lat, _ncdStart.lng], {{
            radius: Math.max(dist, 1), color: '#B71C1C', weight: 2,
            fillColor: '#B71C1C', fillOpacity: 0.1, dashArray: '7 4'
        }}).addTo(mapObj);
        if (_ncdLine) mapObj.removeLayer(_ncdLine);
        _ncdLine = L.polyline([[_ncdStart.lat, _ncdStart.lng], [e.latlng.lat, e.latlng.lng]], {{
            color: '#E53935', weight: 1.5, dashArray: '5 4', opacity: 0.85
        }}).addTo(mapObj);
        if (_ncdLabel) mapObj.removeLayer(_ncdLabel);
        var label = dist < 1000 ? dist.toFixed(0) + ' m' : (dist/1000).toFixed(2) + ' km';
        _ncdLabel = L.tooltip({{ permanent: true, direction: 'center', className: 'cd-radius-tip' }})
            .setLatLng([(_ncdStart.lat + e.latlng.lat)/2, (_ncdStart.lng + e.latlng.lng)/2])
            .setContent('NFZ R: ' + label).addTo(mapObj);
    }});

    mapObj.on('mouseup', function(e) {{
        if (!window.nfzCircleDrawMode || !_ncdStart) return;
        mapObj.dragging.enable();
        var dist = _ncdDragging ? mapObj.distance(_ncdStart, e.latlng) : 0;
        var sLat = _ncdStart.lat, sLng = _ncdStart.lng;
        window.nfzCircleDrawMode = false;
        mapObj._container.style.cursor = '';
        _ncdStart = null; _ncdDragging = false;
        _cleanupNFZCircle();
        if (dist < 10) {{ toast('⚠️ 拖曳距離太短，請重試'); return; }}
        window.location.href = 'pyqt://nfz_circle/' + sLat + '/' + sLng + '/' + dist.toFixed(1);
    }});
}})();
</script>
"""
        # 注入到 </html> 前（在 folium 的 Leaflet script 之後）
        if '</html>' in html:
            html = html.replace('</html>', js_code + '\n</html>')
        else:
            html += js_code

        return html

    # ─────────────────────────────────────
    # 地圖點擊處理
    # ─────────────────────────────────────
    def on_map_clicked(self, lat: float, lon: float):
        """處理地圖點擊事件"""
        logger.info(f"🖱️ 收到地圖點擊: ({lat:.6f}, {lon:.6f}), 編輯模式={self.edit_mode}")
        if self.edit_mode:
            if self.add_corner(lat, lon):
                self.corner_added.emit(lat, lon)
                logger.info(f"✅ 添加角點成功 (總數: {len(self.corners)})")

    def on_circle_draw_complete(self, lat: float, lon: float, radius_m: float):
        """處理拖曳定義圓形完成事件"""
        self._circle_center = (lat, lon)
        if radius_m > 1.0:
            self._circle_radius_m = radius_m
        self.circle_defined.emit(lat, lon, radius_m)
        self._render_map()
        logger.info(f"圓形定義完成: ({lat:.6f}, {lon:.6f}), r={radius_m:.1f}m")

    def set_circle_draw_mode(self, enabled: bool):
        """啟用/停用地圖拖曳定義圓形模式（透過 JS）"""
        if enabled:
            self.custom_page.runJavaScript("if(typeof window.enableCircleDraw==='function') window.enableCircleDraw();")
        else:
            self.custom_page.runJavaScript("if(typeof window.disableCircleDraw==='function') window.disableCircleDraw();")
        logger.info(f"圓形拖曳模式: {'啟用' if enabled else '停用'}")

    # ─────────────────────────────────────
    # NFZ 地圖繪製模式
    # ─────────────────────────────────────
    def set_nfz_polygon_draw_mode(self, enabled: bool):
        """啟用/停用 NFZ 多邊形繪製模式"""
        self._nfz_poly_draw_mode = enabled
        if enabled:
            self._nfz_poly_vertices = []
            self.custom_page.runJavaScript("if(typeof window.enableNFZPolyDraw==='function') window.enableNFZPolyDraw();")
        else:
            self._nfz_poly_vertices = []
            self.custom_page.runJavaScript("if(typeof window.disableNFZPolyDraw==='function') window.disableNFZPolyDraw();")
        logger.info(f"NFZ 多邊形繪製模式: {'啟用' if enabled else '停用'}")

    def finish_nfz_polygon(self):
        """從 Python 側觸發完成 NFZ 多邊形（等同雙擊）"""
        self.custom_page.runJavaScript("if(typeof window.finishNFZPoly==='function') window.finishNFZPoly();")

    def on_nfz_poly_done(self, vertices: list):
        """
        NFZ 多邊形完成，頂點由 JS 一次性傳入（編碼於 pyqt://nfz_poly_done/ URL）。
        不再依賴 per-click on_nfz_poly_vertex() 累積。
        """
        self._nfz_poly_draw_mode = False
        self._nfz_poly_vertices = []
        if len(vertices) >= 3:
            self.nfz_polygon_drawn.emit(vertices)
            logger.info(f"NFZ 多邊形完成: {len(vertices)} 頂點")
        else:
            logger.warning(f"NFZ 多邊形頂點不足 ({len(vertices)} < 3)，放棄")

    def set_nfz_circle_draw_mode(self, enabled: bool):
        """啟用/停用 NFZ 圓形拖曳繪製模式"""
        if enabled:
            self.custom_page.runJavaScript("if(typeof window.enableNFZCircleDraw==='function') window.enableNFZCircleDraw();")
        else:
            self.custom_page.runJavaScript("if(typeof window.disableNFZCircleDraw==='function') window.disableNFZCircleDraw();")
        logger.info(f"NFZ 圓形拖曳模式: {'啟用' if enabled else '停用'}")

    def on_nfz_circle_complete(self, lat: float, lon: float, radius_m: float):
        """接收 NFZ 圓形定義完成"""
        self.nfz_circle_drawn.emit(lat, lon, radius_m)
        self._render_map()
        logger.info(f"NFZ 圓形定義完成: ({lat:.6f}, {lon:.6f}), r={radius_m:.1f}m")

    def add_corner(self, lat: float, lon: float) -> bool:
        """新增邊界點"""
        if len(self.corners) >= MAX_CORNERS:
            logger.warning(f"已達最大角點數量 ({MAX_CORNERS})")
            QMessageBox.warning(self, "已達上限", f"已達最大角點數量 ({MAX_CORNERS} 個)！")
            return False

        self.corners.append((lat, lon))
        self._add_corner_via_js(len(self.corners), lat, lon)
        logger.info(f"新增角點 #{len(self.corners)}: ({lat:.6f}, {lon:.6f})")
        return True

    def _add_corner_via_js(self, index: int, lat: float, lon: float):
        """透過 JavaScript 動態添加角點標記並更新多邊形（不重載頁面）"""
        import json
        corners_json = json.dumps([[c[0], c[1]] for c in self.corners])
        js = f"""
        (function() {{
            var map = window.currentMap;
            if (!map) {{ console.error('地圖物件不存在'); return; }}

            // 添加角點標記
            var icon = L.divIcon({{
                html: '<div style="background:#E53935;color:#fff;border-radius:50%;'
                    + 'width:24px;height:24px;display:flex;align-items:center;'
                    + 'justify-content:center;font-weight:bold;font-size:11px;'
                    + 'border:2px solid #fff;box-shadow:0 2px 6px rgba(0,0,0,.4);">'
                    + '{index}</div>',
                iconSize: [24, 24],
                iconAnchor: [12, 12],
                className: ''
            }});
            var marker = L.marker([{lat}, {lon}], {{icon: icon}})
                .bindPopup('📍 角點 {index}<br>({lat:.6f}, {lon:.6f})')
                .addTo(map);

            if (!window._cornerMarkers) window._cornerMarkers = [];
            window._cornerMarkers.push(marker);

            // 更新邊界多邊形
            var corners = {corners_json};

            if (window._boundaryLayer) {{
                map.removeLayer(window._boundaryLayer);
                window._boundaryLayer = null;
            }}

            if (corners.length >= 3) {{
                window._boundaryLayer = L.polygon(corners, {{
                    color: '#2196F3', weight: 3,
                    fill: true, fillColor: '#2196F3', fillOpacity: 0.15
                }}).addTo(map);
            }} else if (corners.length >= 2) {{
                window._boundaryLayer = L.polyline(corners, {{
                    color: '#2196F3', weight: 2, dashArray: '6,4'
                }}).addTo(map);
            }}
        }})();
        """
        self.custom_page.runJavaScript(js)

    def move_corner(self, index: int, lat: float, lon: float):
        """移動邊界點"""
        if 0 <= index < len(self.corners):
            self.corners[index] = (lat, lon)
            self._render_map()
            logger.info(f"移動角點 #{index + 1}: ({lat:.6f}, {lon:.6f})")

    # ─────────────────────────────────────
    # 路徑顯示
    # ─────────────────────────────────────
    # 各子區域的顯示顏色（最多 6 個）
    _REGION_COLORS = [
        '#08EC91',  # 區域 1：綠
        '#FF6B35',  # 區域 2：橙
        '#3D87FF',  # 區域 3：藍
        '#FFD700',  # 區域 4：金
        '#FF69B4',  # 區域 5：粉
        '#9B59B6',  # 區域 6：紫
    ]

    # 各無人機分配色板（最多 8 台）
    _DRONE_COLORS = [
        '#E53935',  # Drone 1: 紅
        '#1E88E5',  # Drone 2: 藍
        '#43A047',  # Drone 3: 綠
        '#FB8C00',  # Drone 4: 橙
        '#8E24AA',  # Drone 5: 紫
        '#00ACC1',  # Drone 6: 青
        '#F4511E',  # Drone 7: 深橙
        '#3949AB',  # Drone 8: 深藍
    ]

    # ─────────────────────────────────────────────────────────────────
    # SITL 即時 UAV 標記（2D Leaflet）
    # ─────────────────────────────────────────────────────────────────
    def update_uav_position(self, lat: float, lon: float, alt: float = 0.0,
                             heading_deg: float = 0.0, speed_ms: float = 0.0,
                             sysid: int = 1, mode: str = '', armed: bool = False,
                             vehicle_type: str = ''):
        """在 2D 地圖上更新 UAV 標記（依機型顯示不同 SVG 圖示）"""
        try:
            import json as _json
            vt = (vehicle_type or '').upper()
            is_plane = ('PLANE' in vt) or ('FIXED' in vt) or ('VTOL' in vt)
            colors = ['#FFD700','#E53935','#43A047','#1E88E5','#8E24AA','#00ACC1','#FB8C00','#3949AB']
            color = colors[(sysid - 1) % len(colors)]

            if is_plane:
                svg = (
                    '<svg xmlns="http://www.w3.org/2000/svg" width="44" height="44" viewBox="0 0 48 48">'
                    f'<polygon points="24,2 28,22 46,28 46,32 28,30 28,40 34,44 34,46 24,44 14,46 14,44 20,40 20,30 2,32 2,28 20,22" '
                    f'fill="{color}" stroke="#1a1a1a" stroke-width="1.6" stroke-linejoin="round"/>'
                    '<circle cx="24" cy="14" r="2" fill="#FFF"/></svg>'
                )
            else:
                svg = (
                    '<svg xmlns="http://www.w3.org/2000/svg" width="44" height="44" viewBox="0 0 48 48">'
                    '<line x1="8" y1="8" x2="40" y2="40" stroke="#222" stroke-width="3"/>'
                    '<line x1="40" y1="8" x2="8" y2="40" stroke="#222" stroke-width="3"/>'
                    f'<circle cx="8" cy="8" r="6" fill="{color}" stroke="#222" stroke-width="1.5"/>'
                    f'<circle cx="40" cy="8" r="6" fill="{color}" stroke="#222" stroke-width="1.5"/>'
                    f'<circle cx="8" cy="40" r="6" fill="{color}" stroke="#222" stroke-width="1.5"/>'
                    f'<circle cx="40" cy="40" r="6" fill="{color}" stroke="#222" stroke-width="1.5"/>'
                    '<rect x="18" y="18" width="12" height="12" rx="2" fill="#FFF" stroke="#222" stroke-width="1.5"/>'
                    '<polygon points="24,12 28,20 20,20" fill="#E53935"/></svg>'
                )

            label = ('✈ ' if is_plane else '🚁 ') + f'UAV{sysid}' + (f' [{mode}]' if mode else '') + (' ●ARM' if armed else '')
            svg_b64_url = "data:image/svg+xml;utf8," + svg.replace('#', '%23').replace('"', "'")
            label_js = _json.dumps(label)
            js = f"""
            (function(){{
              if (!window.currentMap) return;
              if (!window._uavMarkers) window._uavMarkers = {{}};
              var sysid = {sysid};
              var lat = {lat}, lon = {lon}, hdg = {heading_deg};
              var iconUrl = "{svg_b64_url}";
              var lbl = {label_js};
              var m = window._uavMarkers[sysid];
              if (!m) {{
                var icon = L.divIcon({{
                  html: '<div style="transform:rotate(' + hdg + 'deg);transform-origin:center;">' +
                        '<img src="' + iconUrl + '" width="44" height="44"/></div>' +
                        '<div style="position:absolute;top:46px;left:50%;transform:translateX(-50%);' +
                        'background:rgba(0,0,0,0.7);color:#fff;padding:1px 5px;border-radius:3px;' +
                        'font-size:10px;white-space:nowrap;font-weight:bold;">' + lbl + '</div>',
                  iconSize: [44, 44], iconAnchor: [22, 22], className: 'uav-icon'
                }});
                m = L.marker([lat, lon], {{ icon: icon, zIndexOffset: 1000 }}).addTo(window.currentMap);
                window._uavMarkers[sysid] = m;
              }} else {{
                m.setLatLng([lat, lon]);
                var newIcon = L.divIcon({{
                  html: '<div style="transform:rotate(' + hdg + 'deg);transform-origin:center;">' +
                        '<img src="' + iconUrl + '" width="44" height="44"/></div>' +
                        '<div style="position:absolute;top:46px;left:50%;transform:translateX(-50%);' +
                        'background:rgba(0,0,0,0.7);color:#fff;padding:1px 5px;border-radius:3px;' +
                        'font-size:10px;white-space:nowrap;font-weight:bold;">' + lbl + '</div>',
                  iconSize: [44, 44], iconAnchor: [22, 22], className: 'uav-icon'
                }});
                m.setIcon(newIcon);
              }}
            }})();
            """
            self.custom_page.runJavaScript(js)
            # 軌跡尾跡（每架最多 800 點）
            trail_js = f"""
            (function(){{
              if (!window.currentMap) return;
              if (!window._uavTrails) window._uavTrails = {{}};
              if (!window._uavTrailPts) window._uavTrailPts = {{}};
              var sid = {sysid};
              var c = "{color}";
              if (!window._uavTrailPts[sid]) window._uavTrailPts[sid] = [];
              var pts = window._uavTrailPts[sid];
              pts.push([{lat}, {lon}]);
              if (pts.length > 800) pts.shift();
              if (!window._uavTrails[sid]) {{
                window._uavTrails[sid] = L.polyline(pts, {{
                  color: c, weight: 3, opacity: 0.85, dashArray: null
                }}).addTo(window.currentMap);
              }} else {{
                window._uavTrails[sid].setLatLngs(pts);
              }}
            }})();
            """
            self.custom_page.runJavaScript(trail_js)
        except Exception as e:
            logger.debug(f'2D update_uav_position 失敗: {e}')

    def clear_uav(self, sysid: int = None):
        """清除 2D UAV 標記與軌跡"""
        try:
            if sysid is None:
                js = ("(function(){"
                      "if(window._uavMarkers){Object.values(window._uavMarkers).forEach(m=>{try{window.currentMap.removeLayer(m);}catch(_){}});window._uavMarkers={};}"
                      "if(window._uavTrails){Object.values(window._uavTrails).forEach(t=>{try{window.currentMap.removeLayer(t);}catch(_){}});window._uavTrails={};}"
                      "window._uavTrailPts={};"
                      "})();")
            else:
                js = (f"(function(){{"
                      f"if(window._uavMarkers && window._uavMarkers[{sysid}]){{try{{window.currentMap.removeLayer(window._uavMarkers[{sysid}]);}}catch(_){{}}delete window._uavMarkers[{sysid}];}}"
                      f"if(window._uavTrails && window._uavTrails[{sysid}]){{try{{window.currentMap.removeLayer(window._uavTrails[{sysid}]);}}catch(_){{}}delete window._uavTrails[{sysid}];}}"
                      f"if(window._uavTrailPts) delete window._uavTrailPts[{sysid}];"
                      f"}})();")
            self.custom_page.runJavaScript(js)
        except Exception:
            pass

    def display_path(self, path: List[Tuple[float, float]], altitude: float = 50.0):
        """顯示單條飛行路徑"""
        if not path or len(path) < 2:
            return
        try:
            self.paths = [path]
            self.path_colors = [self._REGION_COLORS[0]]
            self._path_tooltips = []
            self._render_map()
            logger.info(f"顯示路徑: {len(path)} 個航點")
        except Exception as e:
            logger.error(f"顯示路徑失敗: {e}")

    def display_paths(self, paths_list: List[List[Tuple[float, float]]],
                      altitude: float = 50.0):
        """顯示多條飛行路徑（各子區域不同顏色）"""
        valid = [p for p in paths_list if p and len(p) >= 2]
        if not valid:
            return
        try:
            self.paths = valid
            self.path_colors = [
                self._REGION_COLORS[i % len(self._REGION_COLORS)]
                for i in range(len(valid))
            ]
            self._path_tooltips = []
            self._render_map()
            logger.info(f"顯示 {len(valid)} 條子區域路徑")
        except Exception as e:
            logger.error(f"顯示多條路徑失敗: {e}")

    def display_fw_paths(self,
                         takeoff: List[Tuple[float, float]],
                         mission,
                         landing: List[Tuple[float, float]]):
        """
        固定翼三階段路徑顯示（支援多分區掃描）

        takeoff : [(lat,lon), ...] 起飛爬升路徑（金黃）
        mission : 單條路徑 [(lat,lon),...] 或多條 [[(lat,lon),...], ...]
                  單條 → 藍色；多條 → 每條獨立顏色
        landing : [(lat,lon), ...] 五邊降落路徑（橙）
        """
        # 掃描段顏色（避開起飛金黃/降落橙）
        _SCAN_COLORS = ['#3D87FF', '#08EC91', '#FF69B4', '#9B59B6', '#00BCD4', '#FF4500']

        # 正規化 mission → list of paths
        if not mission:
            mission_paths = []
        elif isinstance(mission[0], tuple):   # 單條路徑
            mission_paths = [mission]
        else:                                 # 多條路徑列表
            mission_paths = [m for m in mission if m and len(m) >= 2]

        self.paths = []
        self.path_colors = []
        self._path_tooltips = []

        if takeoff and len(takeoff) >= 2:
            self.paths.append(takeoff)
            self.path_colors.append('#FFD700')
            self._path_tooltips.append('起飛爬升路徑')

        for i, mp in enumerate(mission_paths):
            label = f'掃描區域 {i + 1}' if len(mission_paths) > 1 else '任務掃描路徑'
            self.paths.append(mp)
            self.path_colors.append(_SCAN_COLORS[i % len(_SCAN_COLORS)])
            self._path_tooltips.append(label)

        if landing and len(landing) >= 2:
            self.paths.append(landing)
            self.path_colors.append('#FF6B35')
            self._path_tooltips.append('五邊進場路徑')

        if self.paths:
            self._render_map()

    def display_nfz_zones(self, nfz_zones: list):
        """
        更新禁航區清單並重建地圖。

        Args:
            nfz_zones: 禁航區列表，格式同 MainWindow.nfz_zones
        """
        self._nfz_zones = list(nfz_zones) if nfz_zones else []
        self._render_map()

    def draw_circle_overlay(self, center_lat: float, center_lon: float,
                             radius_m: float, color: str = '#2196F3'):
        """儲存圓心掃描範圍並重建地圖（folium.Circle，隨頁面持久存在）"""
        self._circle_center = (center_lat, center_lon)
        self._circle_radius_m = radius_m
        self._render_map()

    def clear_circle_overlay(self):
        """清除圓心掃描範圍並重建地圖"""
        self._circle_center = None
        self._circle_radius_m = 0.0
        self._render_map()

    def set_home_point_overlay(self, lat: float, lon: float):
        """儲存起飛點並重建地圖（folium home 圖標，隨頁面持久存在）"""
        self._home_point = (lat, lon)
        self._render_map()

    def clear_home_point_overlay(self):
        """清除起飛點並重建地圖"""
        self._home_point = None
        self._render_map()

    def display_survey(self, survey_mission):
        """顯示 Survey 任務"""
        try:
            waypoint_seq = survey_mission.waypoint_sequence
            if not waypoint_seq or len(waypoint_seq.waypoints) < 2:
                return
            path_coords = [(wp.lat, wp.lon) for wp in waypoint_seq.waypoints if wp.command in [16, 22]]
            if len(path_coords) >= 2:
                self.display_path(path_coords)
        except Exception as e:
            logger.error(f"顯示 Survey 失敗: {e}")

    def display_swarm_coverage(
        self,
        swarm_mission,
        coverage_paths=None,
    ):
        """
        顯示群飛協同覆蓋路徑（按無人機分色，區分 Operation/Transfer）

        視覺規則：
        - 每架無人機分配一個獨立顏色（使用 _DRONE_COLORS）
        - Operation Path（作業段）：實線，weight=4，opacity=0.9
        - Transfer Path（轉移段）：虛線 dash_array='8,5'，weight=2，opacity=0.6
        - 每架無人機的起點標記為帶編號的圓形圖標
        - 每個區域邊界用半透明多邊形標記

        參數:
            swarm_mission: SwarmMission 實例
            coverage_paths: 可選的 CoveragePath 列表（用於區分路段類型）
        """
        try:
            drones_data = []
            for drone in swarm_mission.drones:
                drone_id = drone.drone_id
                start_pos = drone.position

                # 從任務航點中提取路徑座標
                operation_paths = []
                transfer_paths = []

                if drone.mission and drone.mission.waypoints:
                    # 取出所有 NAV_WAYPOINT 座標
                    nav_wps = [
                        wp for wp in drone.mission.waypoints.waypoints
                        if wp.command == 16  # NAV_WAYPOINT
                    ]
                    if len(nav_wps) >= 2:
                        # 每兩個相鄰 NAV_WAYPOINT 視為一條作業段
                        # 奇數索引→偶數索引為作業段，偶數→奇數為轉移段
                        for i in range(0, len(nav_wps) - 1, 2):
                            seg = [(nav_wps[i].lat, nav_wps[i].lon),
                                   (nav_wps[i + 1].lat, nav_wps[i + 1].lon)]
                            operation_paths.append(seg)
                        # 相鄰作業段之間的轉移
                        for i in range(1, len(nav_wps) - 2, 2):
                            trans = [(nav_wps[i].lat, nav_wps[i].lon),
                                     (nav_wps[i + 1].lat, nav_wps[i + 1].lon)]
                            transfer_paths.append(trans)

                drones_data.append({
                    'drone_id': drone_id,
                    'start_position': start_pos,
                    'operation_paths': operation_paths,
                    'transfer_paths': transfer_paths,
                })

            # 區域邊界
            areas_data = []
            if coverage_paths:
                for cp in coverage_paths:
                    areas_data.append({
                        'area_id': cp.area_id,
                        'polygon': cp.area_polygon,
                    })

            # 統計資訊
            stats = getattr(swarm_mission, 'stats', {})

            self._swarm_data = {
                'drones': drones_data,
                'areas': areas_data,
                'stats': stats,
            }
            # 使用 JS 動態注入路徑，不重載頁面（避免灰底閃爍）
            self._display_swarm_via_js()
            logger.info(f"顯示群飛覆蓋: {len(drones_data)} 台無人機")

        except Exception as e:
            logger.error(f"顯示群飛覆蓋失敗: {e}")

    def _display_swarm_via_js(self):
        """
        透過 runJavaScript() 動態把群飛路徑加到 Leaflet 地圖，
        不觸發 setUrl() 頁面重載，解決灰底 + 地圖不可操作問題。
        """
        if not self._swarm_data:
            return
        import json as _json

        drones = self._swarm_data.get('drones', [])
        areas  = self._swarm_data.get('areas', [])
        stats  = self._swarm_data.get('stats', {})

        js_parts = ["""
(function(){
    var map = window.currentMap;
    if(!map){console.error('❌ 地圖未就緒，無法顯示群飛路徑');return;}

    // ── 清除舊群飛圖層 ──
    if(window._swarmLayers){
        window._swarmLayers.forEach(function(l){try{map.removeLayer(l);}catch(e){}});
    }
    window._swarmLayers=[];
    var oldLeg=document.getElementById('swarm-legend');
    if(oldLeg) oldLeg.parentNode.removeChild(oldLeg);
"""]

        for drone_info in drones:
            drone_id = drone_info['drone_id']
            color = self._DRONE_COLORS[(drone_id - 1) % len(self._DRONE_COLORS)]

            # 起飛（粗綠）
            for tk_path in drone_info.get('takeoff_paths', []):
                if len(tk_path) >= 2:
                    ll = _json.dumps([[p[0], p[1]] for p in tk_path])
                    js_parts.append(f"""
    var _tk=L.polyline({ll},{{color:'#2e7d32',weight:5,opacity:0.95}});
    _tk.bindTooltip('Drone {drone_id} 起飛');_tk.addTo(map);
    window._swarmLayers.push(_tk);
""")

            # 進入段（青色點虛線）
            for en_path in drone_info.get('entry_paths', []):
                if len(en_path) >= 2:
                    ll = _json.dumps([[p[0], p[1]] for p in en_path])
                    js_parts.append(f"""
    var _en=L.polyline({ll},{{color:'#26c6da',weight:3,opacity:0.9,dashArray:'2,6'}});
    _en.bindTooltip('Drone {drone_id} 進入段');_en.addTo(map);
    window._swarmLayers.push(_en);
""")

            # 作業段（主色實線，最粗）
            for op_path in drone_info.get('operation_paths', []):
                if len(op_path) >= 2:
                    ll = _json.dumps([[p[0], p[1]] for p in op_path])
                    js_parts.append(f"""
    var _op=L.polyline({ll},{{color:'{color}',weight:5,opacity:0.95}});
    _op.bindTooltip('Drone {drone_id} 作業段');_op.addTo(map);
    window._swarmLayers.push(_op);
""")

            # 轉移段（主色長虛線，淡）
            for tr_path in drone_info.get('transfer_paths', []):
                if len(tr_path) >= 2:
                    ll = _json.dumps([[p[0], p[1]] for p in tr_path])
                    js_parts.append(f"""
    var _tr=L.polyline({ll},{{color:'{color}',weight:2,opacity:0.55,dashArray:'12,8'}});
    _tr.bindTooltip('Drone {drone_id} 轉移段');_tr.addTo(map);
    window._swarmLayers.push(_tr);
""")

            # 降落（橘色 dash-dot）
            for ld_path in drone_info.get('landing_paths', []):
                if len(ld_path) >= 2:
                    ll = _json.dumps([[p[0], p[1]] for p in ld_path])
                    js_parts.append(f"""
    var _ld=L.polyline({ll},{{color:'#ef6c00',weight:4,opacity:0.95,dashArray:'10,5,2,5'}});
    _ld.bindTooltip('Drone {drone_id} 降落');_ld.addTo(map);
    window._swarmLayers.push(_ld);
""")

            start_pos = drone_info.get('start_position')
            if start_pos:
                icon_html = (
                    f'<div style="background:{color};color:#fff;border-radius:50%;'
                    f'width:28px;height:28px;display:flex;align-items:center;'
                    f'justify-content:center;font-weight:bold;font-size:12px;'
                    f'border:2px solid #fff;box-shadow:0 2px 6px rgba(0,0,0,.4);">U{drone_id}</div>'
                )
                icon_html_js = _json.dumps(icon_html)
                js_parts.append(f"""
    var _ico=L.divIcon({{html:{icon_html_js},iconSize:[28,28],iconAnchor:[14,14],className:''}});
    var _mk=L.marker([{start_pos[0]},{start_pos[1]}],{{icon:_ico}});
    _mk.bindPopup('Drone {drone_id} 起點');_mk.addTo(map);
    window._swarmLayers.push(_mk);
""")

        for area_info in areas:
            poly = area_info.get('polygon', [])
            if len(poly) >= 3:
                ll = _json.dumps([[p[0], p[1]] for p in poly])
                area_id = area_info.get('area_id', '?')
                js_parts.append(f"""
    var _ar=L.polygon({ll},{{color:'#FFC107',weight:2,fillColor:'#FFC107',fillOpacity:0.08}});
    _ar.bindTooltip('區域 {area_id}');_ar.addTo(map);
    window._swarmLayers.push(_ar);
""")

        # 圖例 HTML
        legend_html = self._build_swarm_legend(self._swarm_data)
        legend_js = _json.dumps(legend_html)
        js_parts.append(f"""
    var _leg=document.createElement('div');
    _leg.id='swarm-legend';
    _leg.innerHTML={legend_js};
    document.body.appendChild(_leg);
""")

        js_parts.append("})();")

        self.custom_page.runJavaScript("".join(js_parts))
        logger.info("群飛路徑已透過 JS 注入（無頁面重載）")

    def _build_swarm_legend(self, swarm_data: dict) -> str:
        """
        建構群飛路徑圖例的 HTML

        圖例內容：
        - 各無人機的顏色色塊 + 名稱
        - 實線 = 作業段，虛線 = 轉移段
        - 統計資訊（若有）

        返回:
            HTML 字串（固定定位在右下角）
        """
        drones = swarm_data.get('drones', [])
        stats = swarm_data.get('stats', {})

        drone_items = ''
        for di in drones:
            did = di['drone_id']
            color = self._DRONE_COLORS[(did - 1) % len(self._DRONE_COLORS)]
            drone_items += (
                f'<div style="display:flex;align-items:center;margin:3px 0;">'
                f'<div style="background:{color};width:14px;height:14px;'
                f'border-radius:50%;margin-right:8px;flex-shrink:0;"></div>'
                f'<span>Drone {did}</span></div>'
            )

        # 統計行
        stat_lines = ''
        if stats.get('total_distance'):
            stat_lines += (
                f'<div style="border-top:1px solid rgba(255,255,255,.3);'
                f'margin-top:6px;padding-top:6px;font-size:11px;opacity:.8;">'
                f'總距離: {stats["total_distance"]:.0f} m<br>'
                f'預估時間: {stats.get("estimated_time", 0) / 60:.1f} min'
                f'</div>'
            )

        return f'''
<div style="position:fixed;bottom:30px;right:10px;z-index:9999;
    background:rgba(30,30,30,.88);color:#fff;padding:10px 14px;
    border-radius:8px;font-size:12px;min-width:130px;
    box-shadow:0 2px 10px rgba(0,0,0,.5);">
  <div style="font-weight:bold;margin-bottom:6px;font-size:13px;">群飛路徑圖例</div>
  {drone_items}
  <div style="margin-top:6px;font-size:11px;opacity:.8;">
    <span style="border-bottom:2px solid #aaa;padding-bottom:1px;">──</span> 作業段&nbsp;&nbsp;
    <span style="border-bottom:2px dashed #aaa;padding-bottom:1px;">- -</span> 轉移段
  </div>
  {stat_lines}
</div>'''

    def display_swarm_raw(self, swarm_data: dict):
        """
        直接接受預建好的 swarm_data dict 並顯示。
        用於螺旋/圓形模式的群飛顯示，不依賴 SwarmMission 結構。

        swarm_data 格式:
            {
                'drones': [{'drone_id': int, 'start_position': (lat,lon),
                            'operation_paths': [[(lat,lon),...]], 'transfer_paths': [...]}, ...],
                'areas': [{'area_id': int, 'polygon': [(lat,lon),...]}],
                'stats': {'total_drones':int, 'total_waypoints':int,
                          'total_distance':float, 'estimated_time':float}
            }
        """
        self._swarm_data = swarm_data
        self._display_swarm_via_js()

    def clear_swarm_display(self):
        """清除群飛覆蓋顯示（JS 移除圖層，不重載頁面）"""
        self._swarm_data = None
        self.custom_page.runJavaScript("""
(function(){
    if(window._swarmLayers){
        window._swarmLayers.forEach(function(l){
            try{window.currentMap.removeLayer(l);}catch(e){}
        });
        window._swarmLayers=[];
    }
    var leg=document.getElementById('swarm-legend');
    if(leg) leg.parentNode.removeChild(leg);
})();
""")
        logger.info("群飛覆蓋顯示已清除")

    # ─────────────────────────────────────
    # 清除操作
    # ─────────────────────────────────────
    def clear_corners(self):
        """清除邊界點"""
        self.corners.clear()
        self.markers.clear()
        self._render_map()
        logger.info("已清除邊界點")

    def display_transit_paths(self, transit_paths: list):
        """
        顯示各無人機從起飛點到各子區域的轉場路徑（虛線）

        參數:
            transit_paths: list of dict
                {'path': [(lat, lon), ...], 'altitude': float, 'region_index': int}
        """
        self.transit_paths = [tp for tp in transit_paths if tp.get('path') and len(tp['path']) >= 2]
        self._render_map()
        logger.info(f"顯示 {len(self.transit_paths)} 條轉場路徑")

    def clear_transit_paths(self):
        """清除轉場路徑"""
        self.transit_paths.clear()
        self._render_map()

    def clear_paths(self):
        """清除路徑（保留角點），同時清除群飛覆蓋圖層"""
        self.paths.clear()
        self.transit_paths.clear()
        # 同時清除群飛/DCCPP 圖層，避免切換模式時殘留舊路徑
        if self._swarm_data is not None:
            self.clear_swarm_display()
        self._render_map()
        logger.info("已清除路徑")

    def clear_all_display_layers_js(self):
        """
        僅透過 JS 清除地圖上的所有顯示圖層（路徑 + 群飛），
        不觸發頁面重載。用於切換到群飛/DCCPP 模式前的清除。
        """
        self.paths.clear()
        self.transit_paths.clear()
        self._swarm_data = None
        # JS：移除群飛圖層 + 所有非底圖的 polyline/marker/polygon 圖層
        self.custom_page.runJavaScript("""
(function(){
    var map = window.currentMap;
    if(!map) return;
    // 清除群飛圖層
    if(window._swarmLayers){
        window._swarmLayers.forEach(function(l){try{map.removeLayer(l);}catch(e){}});
        window._swarmLayers=[];
    }
    var leg=document.getElementById('swarm-legend');
    if(leg) leg.parentNode.removeChild(leg);
    // 清除 folium 產生的路徑/航點 polyline, circleMarker, marker（保留底圖）
    map.eachLayer(function(layer){
        if(layer instanceof L.Polyline || layer instanceof L.CircleMarker){
            // 保留邊界多邊形（藍色 fill）
            if(layer instanceof L.Polygon && layer.options && layer.options.fillColor === '#2196F3') return;
            try{map.removeLayer(layer);}catch(e){}
        }
        // 移除 folium 產生的 Marker（帶 icon 的起訖點標記），保留角點 DivIcon
        if(layer instanceof L.Marker && layer.options && layer.options.icon){
            var ico = layer.options.icon;
            if(ico instanceof L.Icon){
                try{map.removeLayer(layer);}catch(e){}
            }
        }
    });
})();
""")
        logger.info("已透過 JS 清除所有顯示圖層（無頁面重載）")

    def reset_view(self):
        """重置視圖"""
        self._render_map()
        logger.info("視圖已重置")

    def fit_bounds(self, coordinates):
        """調整視圖以包含所有座標"""
        if coordinates:
            self._render_map()

    # ─────────────────────────────────────
    # 編輯模式 & 工具方法
    # ─────────────────────────────────────
    def set_edit_mode(self, enabled: bool):
        self.edit_mode = enabled
        logger.info(f"編輯模式: {'啟用' if enabled else '停用'}")

    def get_corner_count(self) -> int:
        return len(self.corners)

    def get_max_corners(self) -> int:
        return MAX_CORNERS

    def get_remaining_corners(self) -> int:
        return MAX_CORNERS - len(self.corners)

    def can_add_corner(self) -> bool:
        return len(self.corners) < MAX_CORNERS

    def on_marker_moved(self, index: int, lat: float, lon: float):
        self.move_corner(index, lat, lon)
        self.corner_moved.emit(index, lat, lon)

    def change_tile_layer(self, tile_name: str):
        self._render_map()
        logger.info(f"切換圖層: {tile_name}")

    def closeEvent(self, event):
        if self.temp_html_file:
            try:
                os.unlink(self.temp_html_file)
            except OSError:
                pass
        event.accept()