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
from typing import List, Tuple, Optional

from PyQt6.QtWidgets import QVBoxLayout
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage, QWebEngineSettings
from PyQt6.QtCore import pyqtSignal, QUrl, Qt
from urllib.parse import unquote

from utils.logger import get_logger
from ui.widgets.map_base import MapWidgetBase
from ui.resources.tactical_theme import TacticalColors as TC

logger = get_logger()

# ── 顏色常數（取自 TacticalColors 識別色板）──────────────────────
# MIL-STD-1472H 5.8.5 允許多目標識別色板；統一從中央 palette 取得
_REGION_COLORS = list(TC.REGION_PALETTE)
_DRONE_COLORS  = list(TC.UAV_PALETTE)



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
        if url_str.startswith('pyqt://nfz_circle_done/'):
            try:
                parts = url_str.replace('pyqt://nfz_circle_done/', '').split('/')
                lat, lon, radius = float(parts[0]), float(parts[1]), float(parts[2])
                self.widget.on_nfz_circle_done(lat, lon, radius)
            except Exception as e:
                print(f'❌ [Cesium] 解析 NFZ 圓形失敗: {e}')
            return False
        if url_str.startswith('pyqt://rclick/'):
            try:
                parts = url_str.replace('pyqt://rclick/', '').split('/')
                if len(parts) >= 2:
                    self.widget.on_map_rclick(float(parts[0]), float(parts[1]))
            except Exception as e:
                print(f'❌ [Cesium] 解析右鍵 GUIDED 失敗: {e}')
            return False
        return True


# ══════════════════════════════════════════════════════════════════════
#  CesiumMapWidget
# ══════════════════════════════════════════════════════════════════════
class CesiumMapWidget(MapWidgetBase):
    # ── VTOL GLB 模型校正常數 ────────────────────────────────────
    # 綁定 ui/resources/vtol.glb 這一個檔的匯出慣例（單位/朝向），
    # 換模型檔必須一起重校 —— 同 Barracuda 的教訓：靜默換檔只會畫出
    # 一架姿態錯誤的飛機。scale 依該檔單位設定（公尺=1.0、公分=0.01）。
    # vtol.glb = P25-ID.glb（2026-08-25 使用者確認：900 STEP 那顆渲染出來
    # 是六軸旋翼機，不是 VTOL；全機唯一有翼的候選是 P25-ID）。
    # 校正沿用 P25-ID 當年在固定翼通道的慣例（main.html 註解）：
    # 單位公分 ⇒ scale 0.01；模型躺著且上下顛倒 ⇒ pitch +90 / roll 180；
    # 機鼻朝 +X(東) ⇒ heading -90。歪了就目視調這四個值。
    VTOL_MODEL_SCALE = 0.01
    VTOL_HEADING_OFFSET = -90
    VTOL_PITCH_OFFSET = 90
    VTOL_ROLL_OFFSET = 180

    """3D 地圖組件，介面與 MapWidget 相容"""

    # (1-1) 5 個交集 signal（corner_added/corner_moved/circle_defined/
    #       nfz_polygon_drawn/nfz_circle_drawn）已上移至 MapWidgetBase；
    #       以下為本類專屬 signal。
    strike_target_added = pyqtSignal(float, float)  # 打擊目標標記
    guided_goto_requested = pyqtSignal(float, float)  # 右鍵 GUIDED 飛到此點（lat, lon）

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
        # ── 地形跟隨（terrain-following）─────────────────────────────
        # _terrain_follow_agl > 0 表示啟用：每個航點高度改寫為
        #     alt = DEM(lat, lon) + _terrain_follow_agl
        # 若 _dem_manager 為 None 則此模式無效（fallback 到原始 alt）
        self._dem_manager = None
        self._terrain_follow_agl: float = 0.0    # 0 = 停用
        # Geofence — 強制飛安：每次顯示路徑時自動建立並渲染
        self._geofence = None
        self._strike_marking: bool = False       # 打擊目標標記模式
        # ── TOT 扇形閘門（紅心）疊圖開關 ──────────────────────────────
        # ⚠️ 這兩個原本寫成 getattr(self, '_tot_gate_auto', True) 而該屬性
        #    **從未被指派**，等於一個關不掉的開關。改成真的欄位。
        #   * 右鍵「飛到此點」不該畫紅心 —— 那是 STOT 的東西，而 STOT 入口
        #     已於 2026-09-02 關閉（main_window.ENABLE_STOT_RCLICK_STRIKE）。
        #     兩個要一起關，否則選單沒了、疊圖還在。
        #   * 蜂群打擊分頁的「標記目標」模式是使用者明確進入的流程，維持原樣。
        self._tot_gate_on_rclick: bool = False   # 右鍵 → 不畫
        self._tot_gate_auto: bool = True         # 標記目標模式 → 照畫
        self._tot_approach: float = 180.0

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
        from pathlib import Path

        # 讀取 2D 地圖預設座標保持一致
        try:
            from config.settings import get_settings
            s = get_settings()
            init_lat = s.map.default_lat
            init_lon = s.map.default_lon
        except Exception:
            init_lat, init_lon = 25.0777, 121.2325   # 桃園國際機場 RCTP（fallback）
        init_height = 2000.0

        # ── 本地 / CDN 路徑解析 ──
        project_root = Path(__file__).resolve().parent.parent.parent
        local_cesium = project_root / 'assets' / 'cesium' / 'Build' / 'Cesium'
        local_js     = local_cesium / 'Cesium.js'
        local_css    = local_cesium / 'Widgets' / 'widgets.css'

        if local_js.exists() and local_css.exists():
            cesium_js  = local_js.as_uri()
            cesium_css = local_css.as_uri()
            logger.info(f'[Cesium] 使用本地資源: {local_cesium}')
        else:
            cesium_js  = 'https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Cesium.js'
            cesium_css = 'https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Widgets/widgets.css'
            logger.info('[Cesium] 使用 CDN 線上資源')

        # ── GLB 3D 模型路徑 ──
        # 固定翼 = Barracuda 500，由 Fusion 直接匯出（Barracuda500/export_glb.py）。
        # 刻意不自動退回 P25-ID.glb：兩者的單位與朝向慣例不同，main.html 裡的 scale 與
        # _PLANE_*_OFFSET 是綁定 Barracuda 的，靜默換檔只會畫出一架姿態錯誤的飛機。
        # 找不到檔案時 plane_model_uri 留空，Cesium 端自動退回 2D 圖示。
        plane_glb = project_root / 'ui' / 'resources' / 'barracuda500.glb'
        plane_model_uri = plane_glb.as_uri() if plane_glb.exists() else ''
        logger.info(f'[Cesium] 固定翼 GLB 模型: {plane_model_uri or "未找到，退回 2D 圖示"}')

        copter_glb = project_root / 'ui' / 'resources' / 'copter.glb'
        copter_model_uri = copter_glb.as_uri() if copter_glb.exists() else ''
        logger.info(f'[Cesium] 多旋翼 GLB 模型: {copter_model_uri or "未找到，使用 fallback"}')

        # VTOL (4+1 QuadPlane) 專屬模型。沒有這個檔時 VTOL 沿用固定翼模型
        # （舊行為），所以缺檔不是錯誤，只記 log。
        vtol_glb = project_root / 'ui' / 'resources' / 'vtol.glb'
        vtol_model_uri = vtol_glb.as_uri() if vtol_glb.exists() else ''
        logger.info(f'[Cesium] VTOL GLB 模型: {vtol_model_uri or "未找到，VTOL 沿用固定翼模型"}')

        html = (_CESIUM_HTML
                .replace('__CESIUM_JS__',   cesium_js)
                .replace('__CESIUM_CSS__',  cesium_css)
                .replace('__CESIUM_TOKEN__', self.CESIUM_TOKEN)
                .replace('__INIT_LAT__',    str(init_lat))
                .replace('__INIT_LON__',    str(init_lon))
                .replace('__INIT_HEIGHT__', str(init_height))
                .replace('__PLANE_MODEL_URI__', plane_model_uri)
                .replace('__COPTER_MODEL_URI__', copter_model_uri)
                .replace('__VTOL_MODEL_URI__', vtol_model_uri)
                .replace('__VTOL_MODEL_SCALE__', str(self.VTOL_MODEL_SCALE))
                .replace('__VTOL_HEADING_OFFSET__', str(self.VTOL_HEADING_OFFSET))
                .replace('__VTOL_PITCH_OFFSET__', str(self.VTOL_PITCH_OFFSET))
                .replace('__VTOL_ROLL_OFFSET__', str(self.VTOL_ROLL_OFFSET)))

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

    def _pts_to_json(self, pts, default_alt: float = 50.0) -> str:
        """
        將 [(lat,lon)] 或 [(lat,lon,alt)] 序列化為 JSON。

        若啟用地形跟隨（_terrain_follow_agl > 0 且 _dem_manager 存在）：
        每個航點高度改寫為 DEM(lat, lon) + AGL，保持固定離地高度。
        否則使用原始 alt（或 default_alt fallback）。
        """
        agl = float(self._terrain_follow_agl)
        use_tf = (agl > 0.0 and self._dem_manager is not None
                   and getattr(self._dem_manager, '_loaded', False))
        result = []
        for p in pts:
            lat, lon = p[0], p[1]
            base_alt = p[2] if len(p) >= 3 else default_alt
            if use_tf:
                terrain = self._dem_manager.get_elevation(lat, lon)
                alt = terrain + agl
            else:
                alt = base_alt
            result.append({'lat': lat, 'lon': lon, 'alt': alt})
        return json.dumps(result)

    # ─────────────────────────────────────────────────────────────────
    # Geofence（電子圍籬）— 強制飛安政策
    # ─────────────────────────────────────────────────────────────────
    def set_geofence(self, geofence) -> None:
        """
        顯示矩形電子圍籬。
        geofence: mission.geofence_manager.Geofence（4 vertices + alt_min/max）
        """
        if geofence is None:
            self.clear_geofence()
            return
        verts = [{'lat': float(la), 'lon': float(lo)}
                 for (la, lo) in geofence.vertices]
        payload = {
            'vertices': verts,
            'alt_min':  float(geofence.alt_min_m),
            'alt_max':  float(geofence.alt_max_m),
            'method':   getattr(geofence, 'method', 'aabb'),
        }
        self._geofence = geofence
        self._js(f'setGeofence({json.dumps(json.dumps(payload))})')

    def clear_geofence(self) -> None:
        self._geofence = None
        self._js('clearGeofence()')

    # ─────────────────────────────────────────────────────────────────
    # TOT 扇形閘門（蜂群同時抵達；與 sar_swarm/tot_node 同幾何）
    # ─────────────────────────────────────────────────────────────────
    def set_tot_gate(self, target_lat: float, target_lon: float,
                     target_alt: float = 130.0, r_fan: float = 4000.0,
                     half_angle: float = 25.0, approach_from: float = 180.0,
                     n: int = 6) -> None:
        """在地圖上畫目標前的扇形閘門 + N 個 slot + 切線(含 STOT 紅心靶)。
        slot 幾何與 tot_node 完全一致（以 T 為圓心、r_fan、approach±half）。
        預設值對齊噴射尺度(plane-jet 為標準機體:R=4km ±25°、巡航層 130m);
        泡棉尺度請自行傳 r_fan=2000, half_angle=30, target_alt=90。"""
        import math
        EARTH = 111320.0
        slots = []
        for k in range(n):
            off = -half_angle + (2 * half_angle) * k / (n - 1)
            brg = math.radians((approach_from + off) % 360.0)
            slots.append({
                'id': k,
                'lat': target_lat + r_fan * math.cos(brg) / EARTH,
                'lon': target_lon + r_fan * math.sin(brg)
                       / (EARTH * math.cos(math.radians(target_lat))),
            })
        payload = {
            'target': {'lat': target_lat, 'lon': target_lon, 'alt': target_alt},
            'r_fan': r_fan, 'half_angle': half_angle,
            'approach_from': approach_from, 'slots': slots,
        }
        self._js(f'setTotGate({json.dumps(json.dumps(payload))})')
        # 記住紅心位置並清空命中記錄 → check_bullseye_hit 才知道要炸在哪
        self._tot_target = (target_lat, target_lon)
        self._hit_done = set()

    def clear_tot_gate(self) -> None:
        self._js('clearTotGate()')
        self._tot_target = None
        self._hit_done = set()

    # ── 命中爆炸動畫 ────────────────────────────────────────────────
    def explode_at(self, lat: float, lon: float, alt: float = 0.0,
                   label: str = 'HIT') -> None:
        """在指定點放一發爆炸動畫(火球 + 地面震波環 + 命中標籤)。"""
        self._js(f'explodeAt({float(lat)}, {float(lon)}, {float(alt)}, '
                 f'{json.dumps(str(label))})')

    def clear_explosions(self) -> None:
        self._js('clearExplosions()')

    # ── 投放彈體：即時下落 + 落地特效 ──────────────────────────────────
    def drop_store(self, traj, label: str = '') -> None:
        """在 3D 地圖上即時播放一發投放彈體的下落與落地。

        traj 可以是 mission.ballistics.Trajectory，或它的 as_dict() 結果。
        軌跡是 Python 端算好的（mission/ballistics.py），這裡只負責播放 ——
        算錯和畫錯因此不會混在一起，落點與飛行時間可以離線單元測試。
        """
        d = traj.as_dict() if hasattr(traj, 'as_dict') else traj
        self._js(f'dropStore({json.dumps(json.dumps(d))}, '
                 f'{json.dumps(str(label))})')

    def set_view_position(self, lat: float, lon: float,
                          alt: float = 0.0, range_m: float = 800.0) -> None:
        """立即把相機擺到指定位置（不做補間）。

        程式化截圖時要用這一支而不是 fly_to_position：flyTo 的補間由
        scene.tweens 在算圖迴圈裡推進，視窗被遮蔽時 rAF 被節流，補間永遠飛不到。
        """
        self._js(f'setViewPos({float(lat)},{float(lon)},'
                 f'{float(alt)},{float(range_m)})')

    def capture_png(self, path, callback=None) -> None:
        """把目前的 3D 畫面存成 PNG（非同步）。

        直接向 Cesium 的 canvas 要圖，不是螢幕擷取 —— QWebEngine 在視窗沒有
        真正前景時會把算圖節流，螢幕擷取會拿到一模一樣的舊畫面
        （實測連拍四張 md5 完全相同）。這條路不受視窗可見性影響。

        callback(ok: bool, detail: str) 在寫檔完成後被呼叫。
        """
        import base64

        def _done(data_url):
            ok, detail = False, ''
            try:
                if not data_url:
                    detail = 'JS 沒有回傳（頁面還沒載入完？）'
                elif str(data_url).startswith('ERR:'):
                    detail = str(data_url)
                else:
                    _head, b64 = str(data_url).split(',', 1)
                    with open(path, 'wb') as f:
                        f.write(base64.b64decode(b64))
                    ok, detail = True, str(path)
            except Exception as e:
                detail = str(e)
            if callback:
                callback(ok, detail)

        self._page.runJavaScript('grabCanvas()', _done)

    def clear_stores(self) -> None:
        """清掉還在空中的彈體與煙柱（換場景/重置時用）。"""
        self._js('if (typeof clearStores === "function") clearStores();')

    def check_bullseye_hit(self, sysid: int, lat: float, lon: float,
                           alt: float = 0.0, radius: float = 60.0) -> bool:
        """UAV 位置更新時呼叫:靠近已標記的紅心就自動放爆炸(每機只放一次)。

        radius 用「已命中判定半徑」而非 0,因為 GPS/更新率下不可能剛好取樣在
        正中心;實測 tot_node 的命中最近距是 6-17m,60m 門檻可穩定觸發一次。"""
        tgt = getattr(self, '_tot_target', None)
        if not tgt:
            return False
        if sysid in getattr(self, '_hit_done', set()):
            return False
        EARTH = 111320.0
        dy = (lat - tgt[0]) * EARTH
        dx = (lon - tgt[1]) * EARTH * math.cos(math.radians(tgt[0]))
        if math.hypot(dx, dy) > radius:
            return False
        if not hasattr(self, '_hit_done'):
            self._hit_done = set()
        self._hit_done.add(sysid)
        self.explode_at(lat, lon, alt, f'UAV{sysid} 命中')
        return True

    # ─────────────────────────────────────────────────────────────────
    # 地形跟隨（Terrain Following at constant AGL）
    # ─────────────────────────────────────────────────────────────────
    def set_dem_manager(self, dem_manager):
        """注入 DEMTerrainManager（由 main_window 在 DEM 載入後呼叫）。"""
        self._dem_manager = dem_manager

    def set_terrain_following(self, agl_meters: float):
        """
        啟用地形跟隨：每個航點高度改寫為 DEM + AGL。
        agl_meters <= 0 → 停用（沿用原始 alt 或固定海拔）。
        必須先呼叫 set_dem_manager() 才會生效。
        """
        self._terrain_follow_agl = max(0.0, float(agl_meters))
        # 啟用時強制關閉舊的「貼地」模式，避免兩者衝突（貼地會覆蓋我們的 AGL alt）
        if self._terrain_follow_agl > 0:
            self._js('clampToGround = false;')
            self._clamp_basic = False
        self._refresh_paths()

    def clear_terrain_following(self):
        """關閉地形跟隨。"""
        self._terrain_follow_agl = 0.0
        self._refresh_paths()

    @property
    def terrain_following_agl(self) -> float:
        return self._terrain_follow_agl

    # ─────────────────────────────────────────────────────────────────
    # URL Bridge 回調（由 CesiumPage 呼叫）
    # ─────────────────────────────────────────────────────────────────
    def on_map_clicked(self, lat: float, lon: float):
        if self._strike_marking:
            self.strike_target_added.emit(lat, lon)
            # 標記蜂群打擊/TOT 目標時,順手畫出目標前的扇形閘門 + 6 slot + 切線
            # (與 sar_swarm/tot_node 同幾何;純疊圖,不影響既有打擊規劃)。
            if self._tot_gate_auto:
                try:
                    self.set_tot_gate(lat, lon, approach_from=self._tot_approach)
                except Exception:
                    pass
            return
        # 在 3D 地圖上即時畫出角點（與 2D MapWidget 行為一致，避免點擊後看不到標記）
        self.add_corner(lat, lon)
        self.corner_added.emit(lat, lon)

    def on_map_rclick(self, lat: float, lon: float):
        """3D 地圖右鍵 → GUIDED 飛到此點（Mission Planner 風格，由上層跳高度框）。

        ⚠️ 2026-09-02：右鍵**不再**畫 TOT 扇形閘門（紅心）。那是 STOT 的疊圖，
           而 STOT 右鍵入口已關閉；只關選單、疊圖照畫，看起來就像沒關。
           要開回來把 _tot_gate_on_rclick 設 True 即可（幾何程式碼零改動）。
        """
        if self._tot_gate_on_rclick:
            try:
                self.set_tot_gate(lat, lon, approach_from=self._tot_approach)
            except Exception:
                pass
        else:
            # 關閉時順手清掉上一次留下的疊圖，否則畫面上還有紅心，
            # 看起來像沒關掉。
            try:
                self._js('clearTotGate();')
            except Exception:
                pass
        self.guided_goto_requested.emit(lat, lon)

    def on_circle_draw_complete(self, lat: float, lon: float, radius: float):
        self.circle_defined.emit(lat, lon, radius)

    def on_nfz_poly_done(self, vertices):
        self.nfz_polygon_drawn.emit(vertices)

    def on_nfz_circle_done(self, lat: float, lon: float, radius_m: float):
        """JS 端拖曳圓形完成 → 發 nfz_circle_drawn signal"""
        self.nfz_circle_drawn.emit(lat, lon, radius_m)

    # ── NFZ 繪製模式（給 MainWindow / DualMapWidget 統一介面）─────────
    def set_nfz_polygon_draw_mode(self, enabled: bool):
        """3D Cesium 端進入多邊形繪製模式：左鍵點頂點、雙擊完成。"""
        self._js(f'setNFZPolygonDrawMode({"true" if enabled else "false"})')

    def finish_nfz_polygon(self):
        """外部觸發完成多邊形（給「完成多邊形」按鈕用）"""
        self._js('finishNFZPolygon()')

    def set_nfz_circle_draw_mode(self, enabled: bool):
        """3D Cesium 端進入圓形拖曳模式：按住左鍵從圓心拖曳、放開完成。"""
        self._js(f'setNFZCircleDrawMode({"true" if enabled else "false"})')

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
            # forceClamp 三態：
            #   TF 啟用 → '"off"' 強制不貼地（航點 alt 已含 DEM+AGL）
            #   _clamp_basic → 'true' 強制貼地（舊版基本演算法）
            #   其餘    → 'false' = 跟隨 JS 端 clampToGround 全域設定
            if self._terrain_follow_agl > 0:
                clamp = '"off"'
            elif getattr(self, '_clamp_basic', False):
                clamp = 'true'
            else:
                clamp = 'false'
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
            self._js(f'setCircleOverlay({lat},{lon},{self._circle_radius_m},{json.dumps(TC.NEUTRAL)})')
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
            self._js(f'addPath({json.dumps(pts_json)},{json.dumps(TC.FRIENDLY)},12,"起飛爬升",false)')

        # 任務段（藍）
        if len(mission) >= 2:
            pts_json = self._pts_to_json(mission, default_alt=100.0)
            self._js(f'addPath({json.dumps(pts_json)},{json.dumps(TC.NEUTRAL)},10,"任務掃描",false)')

        # 降落段（橙）
        if len(landing) >= 2:
            pts_json = self._pts_to_json(landing, default_alt=80.0)
            self._js(f'addPath({json.dumps(pts_json)},{json.dumps(TC.WARNING)},10,"五邊進場",false)')

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

        _SCAN_COLORS = list(TC.REGION_PALETTE)

        self.paths = []
        self.path_colors = []
        self._path_tooltips = []

        if takeoff and len(takeoff) >= 2:
            self.paths.append(takeoff)
            self.path_colors.append(TC.FG_EMPHASIS)
            self._path_tooltips.append('起飛爬升路徑')

        for i, mp in enumerate(mission_paths):
            label = f'掃描區域 {i+1}' if len(mission_paths) > 1 else '任務掃描路徑'
            self.paths.append(mp)
            self.path_colors.append(_SCAN_COLORS[i % len(_SCAN_COLORS)])
            self._path_tooltips.append(label)

        if landing and len(landing) >= 2:
            self.paths.append(landing)
            self.path_colors.append(TC.WARNING)
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
                             radius_m: float, color: str = None):
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
        """群飛原始資料顯示（3D 版）— **1 架 UAV = 1 種顏色、全實線**。

        segment 類型僅透過「線寬」區分，不再用虛線（避免視覺凌亂）：
            TAKEOFF / LANDING           → UAV 色，較粗實線（起降段）
            OPERATION                   → UAV 色，主寬實線（偵蒐主體）
            ENTRY / TRANSFER            → UAV 色，較細實線（dead-heading）

        點可為 (lat, lon) 或 (lat, lon, alt)。
        """
        try:
            self._js('clearPaths()')
            drones = swarm_data.get('drones', [])

            def _to_pts(seg, default_alt):
                return [{'lat': p[0], 'lon': p[1],
                         'alt': (p[2] if len(p) > 2 else default_alt)}
                        for p in seg]

            def _add(seg, color, width, label, default_alt=80.0):
                """全實線繪製單一 segment（dashed 永為 False）"""
                if not seg or len(seg) < 2:
                    return
                pts = _to_pts(seg, default_alt)
                self._js(
                    f'addPath({json.dumps(json.dumps(pts))},'
                    f'{json.dumps(color)},{width},{json.dumps(label)},false)'
                )

            for drone_info in drones:
                drone_id = drone_info.get('drone_id', 1)
                # 整架 UAV 統一用一種色 — 由 drone_id 決定 (UAV 1 = palette[0], etc.)
                color = _DRONE_COLORS[(drone_id - 1) % len(_DRONE_COLORS)]

                # TAKEOFF — 較粗（起飛段視覺強調）
                for seg in drone_info.get('takeoff_paths', []) or []:
                    _add(seg, color, 9, f'D{drone_id} 起飛', 30.0)

                # ENTRY — 較細（dead-heading 段）
                for seg in drone_info.get('entry_paths', []) or []:
                    _add(seg, color, 5, f'D{drone_id} 進入')

                # OPERATION — 主寬（偵蒐主體）
                for seg in drone_info.get('operation_paths', []) or []:
                    _add(seg, color, 8, f'D{drone_id} 作業')

                # TRANSFER — 較細（U-turn dead-heading）
                for seg in drone_info.get('transfer_paths', []) or []:
                    _add(seg, color, 5, f'D{drone_id} 轉移')

                # LANDING — 中等寬（降落段收束）
                for seg in drone_info.get('landing_paths', []) or []:
                    _add(seg, color, 7, f'D{drone_id} 降落', 40.0)

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

    def remove_last_corner(self):
        """刪除最後一個邊界點並重繪 3D 標記。回傳被刪的 (lat, lon) 或 None。

        ⚠ 舊版沒有這個方法：main_window 的刪除流程只 pop 了 2D 的 corners、
        只重繪 2D，3D 這邊的 ``corners`` 與 Cesium 標記完全沒動 ⇒ 切到 3D
        還看得到那顆被刪掉的點，而且兩邊索引從此對不上（再刪一次就刪錯點）。
        """
        if not self.corners:
            return None
        removed = self.corners.pop()
        self._refresh_corners()
        return removed

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
        """清掉 3D 圖上所有**規劃疊圖**。

        ⚠️ 舊版只清 corners / paths / NFZ / circle / home / DEM / 打擊，
        而 Cesium 這邊實際上有十幾組互相獨立的 entity 陣列。沒被列進來的
        那幾組（`geofenceEntities` 黃色電子圍籬、`_vtolEntities` VTOL 起降
        黃線 + 巡航虛線、`_missionEntities` 飛控讀回的航線、TOT 扇形閘門、
        爆炸特效、漏掃格心）**沒有任何 Python 端會去清**，只能等下一次
        重畫自己蓋掉自己 ⇒ 使用者按「清除全部」，圖上還是一片東西。

        不動的東西：即時 UAV 與它衍生的碰撞預警連線（另有「清除 UAV」）、
        使用者在「管理 Fence」建立的圍籬區（那是飛安設定，有自己的管理介面，
        不該被一顆泛用清除鈕靜靜刪掉）。
        """
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
        self._geofence        = None
        self._js('clearCorners(); clearPaths(); clearNFZ();')
        self._js('if(circleEntity){viewer.entities.remove(circleEntity);circleEntity=null;}')
        self._js('if(homeEntity){viewer.entities.remove(homeEntity);homeEntity=null;}')
        self._js('clearDemOverlay()')
        # 電子圍籬（黃色矩形 + 高度帶）
        self._js('if (typeof clearGeofence === "function") clearGeofence();')
        # VTOL 三段軌跡（垂直黃線 / 巡航虛線 / 轉場箭頭）
        self._js('if (typeof vtolClearAll === "function") vtolClearAll();')
        # 從飛控讀回來的 AUTO 航線疊圖
        self._js('if (typeof clearMissionOverlay === "function") clearMissionOverlay();')
        # 戰術疊圖 / TOT 扇形閘門 / 爆炸特效 / 漏掃格心
        self._js('if (typeof clearTacticalOverlay === "function") clearTacticalOverlay();')
        self._js('if (typeof clearTotGate === "function") clearTotGate();')
        self._js('if (typeof clearExplosions === "function") clearExplosions();')
        self._js('if (typeof clearCoverageGaps === "function") clearCoverageGaps();')
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
        # 有標記紅心時,飛到靶心就自動放一發爆炸(每機一次)
        try:
            self.check_bullseye_hit(sysid, lat, lon, alt)
        except Exception:
            pass

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
    # 跟隨相機（Chase Camera / 模擬飛行視角）
    # ─────────────────────────────────────────────────────────────────
    def set_chase_camera(self, sysid: int):
        """
        啟用或關閉跟隨相機。
        sysid > 0 鎖定指定 UAV；sysid <= 0 關閉並釋放相機控制權。
        """
        self._js(f'setChaseCamera({int(sysid)})')

    def set_chase_camera_params(self, distance: float = None, height: float = None,
                                  pitch_deg: float = None, smoothing: float = None):
        """
        調整跟隨相機參數。None 參數沿用現值。新值會在下一幀把鏡頭歸零回後方視角。
        distance  ── 相機距飛機後方公尺（預設 80）
        height    ── 相機高於飛機公尺（預設 20）
        pitch_deg ── 相機首幀俯角（負值向下看，預設 -10）
        smoothing ── 預留參數（lookAtTransform 模式下由 Cesium 內部平滑處理）
        """
        def _fmt(v):
            return 'null' if v is None else repr(float(v))
        self._js(
            f'setChaseCameraParams({_fmt(distance)},{_fmt(height)},'
            f'{_fmt(pitch_deg)},{_fmt(smoothing)})'
        )

    def reset_chase_view(self):
        """把跟隨鏡頭歸零回「飛機正後方」的預設視角（使用者滑鼠旋轉後可用）。"""
        self._js('resetChaseView()')

    # ─────────────────────────────────────────────────────────────────
    # FPV 第一人稱相機（機上相機 / 雲台偵查視角）
    # ─────────────────────────────────────────────────────────────────
    def set_fpv_camera(self, sysid: int, gimbal_pitch_deg: float = None,
                        fov_deg: float = None, roll_follow: bool = None):
        """
        啟用或關閉 FPV 第一人稱視角（機上相機）。與 chase 互斥。
        sysid > 0 鎖定；sysid <= 0 關閉並還原 FOV。
        gimbal_pitch_deg ── 0 = 平視前方（飛行 FPV）；-90 = 正下方（偵查鏡頭）
        fov_deg          ── 水平視角度數，典型 70~90°；廣角 FPV 100~130°
        roll_follow      ── True 相機跟飛機 roll（沉浸感）；False 雲台穩定
        """
        def _fmt(v):
            if v is None:
                return 'undefined'
            if isinstance(v, bool):
                return 'true' if v else 'false'
            return repr(float(v))
        self._js(
            f'setFpvCamera({int(sysid)},{_fmt(gimbal_pitch_deg)},'
            f'{_fmt(fov_deg)},{_fmt(roll_follow)})'
        )

    def set_fpv_camera_params(self, gimbal_pitch_deg: float = None,
                               fov_deg: float = None, roll_follow: bool = None,
                               forward_offset: float = None):
        """微調 FPV 參數（雲台俯角 / FOV / 是否跟 roll / 機頭前偏移）。"""
        def _fmt(v):
            if v is None:
                return 'undefined'
            if isinstance(v, bool):
                return 'true' if v else 'false'
            return repr(float(v))
        self._js(
            f'setFpvCameraParams({_fmt(gimbal_pitch_deg)},{_fmt(fov_deg)},'
            f'{_fmt(roll_follow)},{_fmt(forward_offset)})'
        )

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
                        roll_deg: float = 0.0,
                        sysid: int = 1,
                        hfov_deg: float = 0.0,
                        vfov_deg: float = 0.0,
                        mount_angle_deg: float = 0.0):
        """更新指定 UAV 的 FOV 光錐位置與姿態（多機支援）

        Args:
            lat / lon / alt   : UAV 位置（度、度、公尺）
            fov_radius        : Legacy 圓錐地面投影半徑（公尺），HFOV/VFOV=0 時使用
            heading_deg       : 航向角（度，順時針自北）
            pitch_deg         : 俯仰角（度），legacy 模式用於地面投影偏移
            roll_deg          : 滾轉角（度）
            sysid             : UAV 系統 ID（1-12）
            hfov_deg          : 水平視場角全角（度）；> 0 啟用梯形角錐 frustum 模式
            vfov_deg          : 垂直視場角全角（度）；> 0 啟用梯形角錐 frustum 模式
            mount_angle_deg   : 感測器掛載角 α_m（度）；0 = nadir、> 0 前傾安裝
        """
        self._js(
            f'updateFOVCone({int(sysid)},{lat},{lon},{alt},{fov_radius},'
            f'{heading_deg},{pitch_deg},{roll_deg},'
            f'{hfov_deg},{vfov_deg},{mount_angle_deg})'
        )

    def clear_fov_cone(self, sysid: int = None):
        """清除 FOV 光錐 (sysid=None 清全部，否則只清指定 UAV)"""
        if sysid is None:
            self._js('clearFOVCone()')
        else:
            self._js(f'clearFOVCone({int(sysid)})')

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
                       fov_radius: float = 50.0,
                       sysid: int = 0):
        """更新搜救熱力圖（UAV 光錐掃過時更新 COS）

        Args:
            uav_lat / uav_lon : UAV 經緯度
            fov_radius        : Legacy 圓形覆蓋半徑（公尺），sysid=0 時用
            sysid             : > 0 時改用該機 frustum 真實梯形覆蓋判定
                                （依 HFOV/VFOV/掛載角即時投影，視角變寬則
                                掃描範圍同步變寬）
        """
        self._js(f'updateHeatmap({uav_lat},{uav_lon},{fov_radius},{int(sysid)})')

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

    # ── Fence Zone（統一 NFZ / 威脅 / 作業圍籬視覺化）───────────────
    def add_fence_zone(self, zone_id: str, vertices: list,
                       alt_min: float, alt_max: float,
                       name: str, color_hex: str,
                       category: str, inclusion: bool):
        """新增 / 更新 fence-style 區域視覺。

        Args:
            zone_id      : 唯一 ID（從 FenceZone.id 來）
            vertices     : [(lat, lon), ...]  圓形請先在 Python 端展開為 N 邊形
            alt_min/max  : 海拔上下限（公尺）
            name         : 顯示名稱
            color_hex    : '#RRGGBB' 顏色（依 category 而定）
            category     : 'NFZ' | 'THREAT' | 'GEOFENCE'
            inclusion    : True=圍住飛機在內、False=禁止進入
        """
        verts_js = json.dumps([
            {'lat': float(lat), 'lon': float(lon)} for lat, lon in vertices
        ])
        self._js(
            f'addFenceZone({json.dumps(zone_id)},{verts_js},'
            f'{float(alt_min)},{float(alt_max)},'
            f'{json.dumps(name)},{json.dumps(color_hex)},'
            f'{json.dumps(category)},{"true" if inclusion else "false"})'
        )

    def remove_fence_zone(self, zone_id: str):
        self._js(f'removeFenceZone({json.dumps(zone_id)})')

    def clear_fence_zones(self):
        self._js('clearAllFenceZones()')

    # ── Strike Visual Overlay — 攻擊執行階段即時視覺化 ──────────────
    def strike_viz_begin(self, target_lat: float, target_lon: float,
                         target_alt: float = 0.0):
        """啟動攻擊視覺化（清空前一輪殘留 + 開始接收 telemetry 更新）。"""
        self._js(
            f'strikeVizBegin({float(target_lat)},{float(target_lon)},'
            f'{float(target_alt)})'
        )

    def strike_viz_update_uav(self, sysid: int,
                              lat: float, lon: float, alt: float,
                              heading_deg: float,
                              callsign: str = '',
                              planned_eta_s: float = 0.0,
                              loiter_lat: float = 0.0,
                              loiter_lon: float = 0.0,
                              loiter_radius_m: float = 0.0):
        """每筆 telemetry 呼叫一次 — 階段判定 + 軌跡 + HUD label。

        Args:
            sysid              : UAV 系統 ID
            lat/lon/alt        : UAV 目前 ENU 位置
            heading_deg        : 航向角（度）
            callsign           : 顯示用呼號（'UAV-3' 等）
            planned_eta_s      : 規劃命中時刻（從 strike_viz_begin 後幾秒）
            loiter_lat/lon     : 該 UAV 規劃 loiter 點（>0 → 繪製軌道環）
            loiter_radius_m    : loiter 半徑（公尺）
        """
        cs = json.dumps(callsign or f'UAV-{int(sysid)}')
        self._js(
            f'strikeVizUpdateUav({int(sysid)},'
            f'{float(lat)},{float(lon)},{float(alt)},{float(heading_deg)},'
            f'{cs},{float(planned_eta_s)},'
            f'{float(loiter_lat)},{float(loiter_lon)},{float(loiter_radius_m)})'
        )

    def strike_viz_end(self, callback=None):
        """結束攻擊視覺化，可選 callback(stats_json: str) 接收 BDA 統計。

        若提供 callback，會 async 取回 JS 端 `strikeVizEnd()` 的 JSON 結果
        並轉成 dict 後呼叫 callback({'uavs': [...], 'summary': {...}})。
        """
        if callback is None:
            self._js('strikeVizEnd()')
            return
        import json as _json

        def _wrap(result_json):
            try:
                if isinstance(result_json, str) and result_json:
                    callback(_json.loads(result_json))
                else:
                    callback({'uavs': [], 'summary': {}})
            except Exception:
                callback({'uavs': [], 'summary': {}})

        self._page.runJavaScript('strikeVizEnd()', _wrap)

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

    # ── 飛控任務航線（讀回的那一份）────────────────────────────────
    def display_fc_mission(self, path3d, loiters=None,
                           color: str = '#00E5FF', label: str = 'FC Mission',
                           home=None):
        """畫從飛控讀回來的任務航線 + 盤旋圈。

        Parameters
        ----------
        path3d : list[tuple[float, float, float]]
            [(lat, lon, alt), ...]，由 `mission.mission_view.mission_to_path_3d`
            產生。
        loiters : list[dict] | None
            `mission.mission_view.mission_loiters` 的輸出。

        ⚠️ 這條路徑**不碰** `_tacticalEntities`，也不觸發任何圍籬重建。
           對應的清除是 `clear_fc_mission()`，不是 `clearTacticalOverlay()`。
        """
        import json as _json
        self._js('clearMissionOverlay()')
        pts = [[float(p[0]), float(p[1]), float(p[2] if len(p) > 2 else 0.0)]
               for p in (path3d or [])]
        if pts:
            self._js(f'addMissionRoute({_json.dumps(pts)},'
                     f'{_json.dumps(color)},{_json.dumps(label)})')
        if home:
            self._js('addMissionHome(%r,%r,%r)' % (
                float(home['lat']), float(home['lon']),
                float(home.get('alt', 0.0))))
        for lo in (loiters or []):
            self._js(
                'addMissionLoiter(%r,%r,%r,%r,%s)' % (
                    float(lo['lat']), float(lo['lon']),
                    float(lo['radius_m']), float(lo.get('alt', 0.0)),
                    _json.dumps(color))
            )

    def clear_fc_mission(self):
        """清除飛控任務航線疊加。"""
        self._js('clearMissionOverlay()')

    def draw_coverage_gaps(self, missed_points: list):
        """標出偵察覆蓋率驗算找出的漏掃格心。

        Parameters
        ----------
        missed_points : list[tuple[float, float]]
            [(lat, lon), ...]，來自 core.recon.CoverageReport.missed_points。
        """
        import json as _json
        pts = [{'lat': float(la), 'lon': float(lo)} for la, lo in missed_points]
        self._js(f'drawCoverageGaps({_json.dumps(pts)})')

    def clear_coverage_gaps(self):
        """清除漏掃格標示"""
        self._js('clearCoverageGaps()')

    def render_coverage_heatmap(self, display_grid: dict):
        """用 SAR 熱力圖呈現偵察覆蓋率分布（紅=漏掃，藍=已充分掃過）。

        Parameters
        ----------
        display_grid : dict
            core.recon.CoverageReport.to_display_grid() 的回傳值。
            與 initSARHeatmap 共用同一組 entity，故 clear_sar_heatmap() 可清除。
        """
        import json as _json
        self._js(f'renderCoverageHeatmap({_json.dumps(display_grid)})')

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
