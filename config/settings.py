"""
全局配置管理模組
提供系統級配置、路徑配置、地圖配置等
"""

import os
import json
from pathlib import Path
from dataclasses import dataclass, asdict, field
from typing import Dict, Any, Optional


def _rebase_saved_paths(saved: Dict[str, Any]) -> Dict[str, Any]:
    """把存檔裡**搬家前**的絕對路徑校正到目前的專案根目錄。

    ⚠⚠⚠ 為什麼需要：`GlobalSettings.load` 是 `PathSettings(**config_data['paths'])`，
    也就是**存檔的絕對路徑會蓋掉計算值**。專案一搬家（或換一台機器 clone）之後，
    settings.json 裡那八條路徑就全部指向不存在的地方，而症狀只有一則
    「飛行器配置文件不存在」的警告 —— 其餘七條**完全靜默**：

    * `vehicle_profiles` 永遠是 None（載不到任何機型設定）
    * `icons_dir` / `styles_dir` 指向不存在的目錄
    * `PathSettings._ensure_dirs` 還會在**舊位置**建出一整組空目錄

    2026-09-09 實際發生：專案從 `Documents/aeroplan-studio` 搬到
    `Documents/Projects/aeroplan-studio`，八條路徑全部留在舊位置。

    作法：只在存檔的 `project_root` 與實際不同時動作，而且**只校正那些原本就在
    舊根目錄底下**的路徑 —— 使用者刻意指到別處的（例如把 export_dir 設在 D 槽）
    保持原樣，不要好心辦壞事。
    """
    if not saved:
        return saved
    real_root = str(Path(__file__).resolve().parent.parent)
    old_root = str(saved.get("project_root") or "")
    if not old_root or os.path.normcase(os.path.normpath(old_root)) ==             os.path.normcase(os.path.normpath(real_root)):
        return saved

    old_norm = os.path.normcase(os.path.normpath(old_root))
    out, moved = dict(saved), []
    for k, v in saved.items():
        if not isinstance(v, str) or not v:
            continue
        norm = os.path.normcase(os.path.normpath(v))
        if norm == old_norm:
            out[k] = real_root
        elif norm.startswith(old_norm + os.sep):
            rel = os.path.normpath(v)[len(os.path.normpath(old_root)):]
            out[k] = os.path.join(real_root, rel.lstrip(os.sep).lstrip("/"))
        else:
            continue                       # 使用者自訂到別處 ⇒ 不動
        moved.append(k)
    if moved:
        print(f"[settings] ⚠ 專案已搬家（{old_root} → {real_root}）；"
              f"已校正存檔路徑 {sorted(moved)}。"
              f"不校正的話飛行器設定、圖示與樣式都會靜靜載不到。")
    return out


@dataclass
class PathSettings:
    """路徑配置"""
    # 專案根目錄
    project_root: str = str(Path(__file__).parent.parent)
    
    # 資料目錄
    data_dir: str = None
    config_dir: str = None
    cache_dir: str = None
    export_dir: str = None
    log_dir: str = None
    
    # 資源目錄
    icons_dir: str = None
    styles_dir: str = None
    
    def __post_init__(self):
        """初始化後自動設定路徑"""
        if self.data_dir is None:
            self.data_dir = os.path.join(self.project_root, "data")
        if self.config_dir is None:
            self.config_dir = os.path.join(self.project_root, "config")
        if self.cache_dir is None:
            self.cache_dir = os.path.join(self.data_dir, "cache")
        if self.export_dir is None:
            self.export_dir = os.path.join(self.data_dir, "exports")
        if self.log_dir is None:
            self.log_dir = os.path.join(self.data_dir, "logs")
        if self.icons_dir is None:
            self.icons_dir = os.path.join(self.project_root, "ui", "resources", "icons")
        if self.styles_dir is None:
            self.styles_dir = os.path.join(self.project_root, "ui", "resources", "styles")
        
        # 確保目錄存在
        self._ensure_dirs()
    
    def _ensure_dirs(self):
        """確保所有目錄存在"""
        for dir_path in [self.data_dir, self.config_dir, self.cache_dir, 
                        self.export_dir, self.log_dir]:
            os.makedirs(dir_path, exist_ok=True)


@dataclass
class MapSettings:
    """地圖配置"""
    # 預設位置（2D Leaflet 與 3D Cesium 初始置中點）— 桃園國際機場 RCTP
    default_lat: float = 25.0777
    default_lon: float = 121.2325
    default_zoom: int = 20  # 預設縮放 20 倍
    max_zoom: int = 22      # 最大縮放
    
    # 地圖伺服器
    map_servers: list = None
    
    # 快取設定
    enable_cache: bool = True
    cache_size_mb: int = 500

    # Cesium ion access token（3D 地圖用）。⚠ 這是祕密：只放在 config/settings.json
    # （已被 .gitignore），不要寫進原始碼。環境變數 CESIUM_ION_TOKEN 優先於這個值。
    cesium_ion_token: str = ''

    # 地圖起始中心改用「場地預設」：填 config.spawn_presets.SPAWN_PRESETS 的 key
    # （CHIAYI_AUIRC / HUWEI / TAOYUAN）就會在載入設定時把 default_lat/lon 換成該場地；
    # 空字串＝用上面的 default_lat/lon；不認得的 key 不會靜默，會印出警告並維持原值。
    default_site: str = ''
    
    def __post_init__(self):
        """初始化地圖伺服器列表"""
        if self.map_servers is None:
            self.map_servers = [
                {
                    "name": "Google衛星",
                    "url": "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                    "max_zoom": 22
                },
                {
                    "name": "OpenStreetMap",
                    "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                    "max_zoom": 19
                },
                {
                    "name": "Google地圖",
                    "url": "https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}",
                    "max_zoom": 22
                }
            ]


@dataclass
class ExportSettings:
    """匯出配置"""
    # MAVLink 設定
    mavlink_version: str = "QGC WPL 110"
    include_home_point: bool = True
    include_rtl: bool = True
    
    # 檔案格式
    waypoint_extension: str = ".waypoints"
    mission_extension: str = ".mission"
    
    # 匯出選項
    auto_generate_briefing: bool = True
    include_metadata: bool = True
    compress_output: bool = False
    
    # 座標精度
    coordinate_precision: int = 6  # 小數點後6位
    altitude_precision: int = 2    # 小數點後2位


@dataclass
class PerformanceSettings:
    """效能配置"""
    # 渲染設定
    max_waypoints_display: int = 5000
    path_simplification_tolerance: float = 0.5  # 公尺
    
    # 計算設定
    max_iterations: int = 10000
    convergence_threshold: float = 0.001
    
    # 多執行緒
    enable_multiprocessing: bool = True
    max_workers: int = 4

    # GPU 加速（SITL 3D 視覺化 / WebEngine 渲染管線）
    # 啟用時在 QApplication 建立前注入 Chromium GPU 旗標：
    # GPU rasterization / zero-copy / WebGL high-performance context，
    # 讓 Cesium 3D 地圖與 SITL 即時追蹤走獨顯硬體渲染。
    # 注意：ArduPilot SITL 物理模擬本身為 CPU 程序，不受此旗標影響。
    enable_gpu_acceleration: bool = True


@dataclass
class SafetySettings:
    """安全配置"""
    # 高度限制
    min_altitude_m: float = 5.0
    max_altitude_m: float = 500.0
    
    # 速度限制
    min_speed_mps: float = 0.1
    max_speed_mps: float = 30.0
    
    # 間距限制
    min_spacing_m: float = 0.5
    max_spacing_m: float = 100.0
    
    # 安全距離
    default_safety_distance_m: float = 5.0
    min_safety_distance_m: float = 1.0
    
    # 群飛設定
    swarm_safety_distance_m: float = 10.0
    rtl_altitude_increment_m: float = 3.0


@dataclass
class UISettings:
    """UI配置"""
    # 視窗設定
    window_title: str = "無人機網格航線規劃工具"
    window_width: int = 1600
    window_height: int = 1000
    min_window_width: int = 1000
    min_window_height: int = 700
    
    # 主題
    theme: str = "modern"  # modern, dark, light
    
    # 路徑顯示
    default_path_width: int = 4
    path_colors: list = None
    
    # 區域顯示
    region_border_width: int = 2
    region_border_color: str = "#6aa84f"
    region_fill_colors: list = None
    
    def __post_init__(self):
        """初始化顏色列表"""
        if self.path_colors is None:
            self.path_colors = [
                "#08EC91", "#2DB7F5", "#F76560", "#F7BA1E",
                "#8A2BE2", "#00C2B8", "#FF8F1F", "#6C8CFF"
            ]
        if self.region_fill_colors is None:
            self.region_fill_colors = [
                "#cfe8cf", "#cfe3f6", "#ffd9d6", "#fbe7c3",
                "#e3d7f7", "#d5f3ef", "#ffe4cc", "#dee4ff"
            ]


@dataclass
class SITLLaunchSettings:
    """SITL 啟動彈窗的持久化設定。
    用途：記住使用者上次填的「每台 SITL 的 sysid + 嵌入式 fan-out 目標」，
         下次啟動時自動回填到 SITLLaunchDialog，免重打。
    """
    # 是否啟用嵌入式 fan-out（取消 → 純本機 SITL，沿用舊行為）
    enable_fanout: bool = False
    # 啟動 SITL 時是否自動嘗試新增 Windows Firewall 入站規則（首次會跳 UAC）
    auto_firewall: bool = True
    # 上次選的「出生地點」預設 key（config.spawn_presets.SPAWN_PRESETS）；
    # 空字串 = 自動（任務原點 / settings.map 預設），不覆寫生成座標。
    home_preset: str = ''
    # 每台 SITL 的設定列表，list[dict]，每個 dict 包含：
    #   sysid: int           — 覆寫 SYSID_THISMAV，讓嵌入式 tgt_system 對得上
    #   embedded_ip: str     — 嵌入式裝置 LAN IP，例如 '192.168.1.50'
    #   embedded_port: int   — 嵌入式 MAVROS apm.launch 監聽的 UDP port
    # 注意：刻意存 list[dict] 而非 nested dataclass，避免 asdict 序列化遞迴複雜度
    instances: list = field(default_factory=list)


@dataclass
class EmbeddedSwarmSettings:
    """嵌入式蜂群連線的持久化設定。

    用途：記住「連線到嵌入式裝置（Jetson 原生 SITL）監看 N 機」
         的上次設定，下次開啟對話框時自動回填，免重打。

    對應固定翼蜂群文件 §4.1（模式 A1）：
        AeroPlan Studio 當監看 GCS，透過 TCP 連原生 ArduPlane SITL 的 TCP server
        （每個實例 -I<N> 開 `5760 + 10*N` → 5760 / 5770 / 5780 / 5790，每機一條），
        導引由嵌入式 ROS 蜂群負責，本端只做顯示。
    """
    # 嵌入式裝置（Jetson）LAN IP — direction='dial' 時 AeroPlan 主動連往此位址
    host: str = ''
    # 監看機數（SITL 實例數量）
    count: int = 4
    # 基準 TCP port（第 0 機）；第 i 機 = base_port + i * port_stride
    base_port: int = 5760
    # 相鄰機之間的 port 間隔（SITL 慣例：5760 / 5770 / 5780 → stride=10）
    port_stride: int = 10
    # SYSID 起始值（原生 SITL -I0..N 慣例：sysid = 1..N+1 → sysid_base=1）
    sysid_base: int = 1
    # 連線方向：
    #   'dial'   = AeroPlan 主動連往對端 TCP server (tcp:<host>:<port>)，
    #              對端為原生 ArduPlane SITL -I<N>，或 mavproxy --out tcpin:0.0.0.0:<port>
    #   'listen' = AeroPlan 當 TCP server (tcpin:0.0.0.0:<port>)，
    #              需 mavproxy 端 --out tcp:<Windows_IP>:<port> 主動連入
    direction: str = 'dial'


@dataclass
class WeatherSettings:
    """氣象資料來源（風場顯示／寫入規劃風）。"""
    # 中央氣象署開放資料平臺 API 授權碼（自動氣象站觀測 O-A0001-001）。⚠ 這是祕密：只放在
    # config/settings.json（已被 .gitignore），不要寫進原始碼；環境變數 CWA_API_KEY 優先於這個值。
    cwa_api_key: str = ''
    # 預設測站（代碼如 466920 或名稱）；空白＝離作業區最近的測站
    cwa_station: str = ''


def _load_weather(raw) -> tuple:
    """weather 區塊單獨處理：未知鍵只警告並忽略、已知鍵照收 —— 不讓一個打錯的鍵名（cwa_apikey）把整段（含授權碼）
    靜默歸零，再讓面板誤報「沒有授權碼」。回傳 (WeatherSettings, 警告文字)；只記鍵名，絕不記值。"""
    from dataclasses import fields as _fields
    if not isinstance(raw, dict):
        warn = f"settings.json 的 weather 區塊不是物件（{type(raw).__name__}），整段忽略"
        print(warn)
        return WeatherSettings(), warn
    known = {f.name for f in _fields(WeatherSettings)}
    unknown = sorted(set(raw) - known)
    ws = WeatherSettings(**{k: v for k, v in raw.items() if k in known})
    warn = f"settings.json 的 weather 區塊有未知鍵 {unknown}（已忽略；已知鍵 {sorted(known)}）" if unknown else ''
    if warn:
        print(warn)
    return ws, warn


@dataclass
class Model3DSettings:
    """3D 圖上 VTOL 畫哪個機體模型（**只換外觀**：飛控、規劃、SITL 動力學都不看這個）。

    vtol_model     沒逐架指定時的預設：'hero' ＝ ui/resources/hero.glb（MFE 逆行者，tools/build_hero_glb.py 建）；
                   'pioneer' ＝ ui/resources/pioneer.glb（MFE 開拓者，tools/build_pioneer_glb.py 建）；'' ＝通用 vtol.glb。
                   沒有那個 GLB ⇒ 記 warning、畫通用的。
                   程式預設 ''（不改既有行為；姿態探針用預設設定隔離，它的閘門針對通用 vtol.glb 的校正）。
    uav_airframes  逐架指定 {"sysid": "hero"|"pioneer"|""}，例如 HIL 三顆板 {"1": "pioneer", "2": "hero", "3": "hero"}。
    HARVEST 任務上傳後，該計畫的構型 A／B 會把那幾架指定成 pioneer／hero（執行期，優先於這裡；重開／停 SITL 時取消）。
    ⚠ 放在自己的頂層區塊、不放 map：舊版本只讀認得的區塊（看到 model3d 會跳過），但舊版 MapSettings(**...) 遇到
      不認得的鍵會丟 TypeError ⇒ map 以後的區塊全部回預設（2026-10-23 審查實測）。
    """
    vtol_model: str = ''
    uav_airframes: dict = field(default_factory=dict)


# 2026-10-23 第一版把這兩個鍵放在 map 底下；讀到就搬到 model3d（存檔時只寫 model3d）
_MODEL3D_KEYS_FORMERLY_IN_MAP = ('vtol_model', 'uav_airframes')


def _known_fields_only(cls, raw, section: str) -> tuple:
    """區塊裡只留 cls 認得的鍵；其餘只記鍵名（絕不記值）並忽略 —— 不讓一個新版本才有的鍵把整段（和後面每一段）歸零。
    回傳 (kwargs, 警告文字)。"""
    from dataclasses import fields as _fields
    if not isinstance(raw, dict):
        warn = f"settings.json 的 {section} 區塊不是物件（{type(raw).__name__}），整段忽略"
        print(warn)
        return {}, warn
    known = {f.name for f in _fields(cls)}
    unknown = sorted(set(raw) - known)
    warn = f"settings.json 的 {section} 區塊有未知鍵 {unknown}（已忽略）" if unknown else ''
    if warn:
        print(warn)
    return {k: v for k, v in raw.items() if k in known}, warn


def _load_model3d(raw) -> tuple:
    """model3d 區塊：未知鍵只警告；uav_airframes 不是物件 ⇒ 警告、當成空的。回傳 (Model3DSettings, 警告文字)。"""
    kw, warn = _known_fields_only(Model3DSettings, raw, 'model3d')
    if 'uav_airframes' in kw and not isinstance(kw['uav_airframes'], dict):
        w = f"settings.json 的 model3d.uav_airframes 應為 {{sysid: 機體}}，得 {type(kw['uav_airframes']).__name__}（已忽略）"
        print(w)
        warn = (warn + '；' if warn else '') + w
        kw.pop('uav_airframes')
    if 'vtol_model' in kw and not isinstance(kw['vtol_model'], str):
        kw['vtol_model'] = str(kw['vtol_model'])
    return Model3DSettings(**kw), warn


@dataclass
class HILSettings:
    """HIL（Hardware-in-the-Loop）連線面板的持久化設定。
    用途：記住使用者上次填的每顆飛控連線埠 / 鮑率 / SYSID，
         下次開啟 SITL 分頁的 HIL 頁時自動回填。
    """
    # 每顆飛控一列，list[dict]，每個 dict 包含：
    #   enabled: bool        — 此列是否參與「全部連線」
    #   port: str            — 'COM7' 或 'udp:0.0.0.0:14550' / 'tcp:...' 完整連線字串
    #   baud: int            — 序列埠鮑率（udp/tcp 連線忽略）
    #   sysid: int           — 期望的 MAVLink system id（顯示用 callsign）
    #   vehicle_hint: str    — '' 自動 / 'PLANE' / 'COPTER' / 'VTOL'
    # 與 SITLLaunchSettings 相同，刻意存 list[dict] 避免 asdict 遞迴複雜度
    instances: list = field(default_factory=list)


class GlobalSettings:
    """全局配置管理器"""
    
    def __init__(self, config_file: Optional[str] = None):
        """
        初始化全局配置
        
        參數:
            config_file: 配置文件路徑（可選）
        """
        # 初始化各項配置
        self.paths = PathSettings()
        self.map = MapSettings()
        self.export = ExportSettings()
        self.performance = PerformanceSettings()
        self.safety = SafetySettings()
        self.ui = UISettings()
        self.sitl_launch = SITLLaunchSettings()
        self.embedded_swarm = EmbeddedSwarmSettings()
        self.hil = HILSettings()
        self.weather = WeatherSettings()
        self.model3d = Model3DSettings()
        self.model3d_load_warning = ''     # model3d 區塊讀取時的警告（未知鍵、型別不對）
        self.map_load_warning = ''         # map 區塊的未知鍵（只記鍵名）
        self.weather_load_warning = ''      # weather 區塊讀取時的警告（未知鍵等），給 utils.cwa_opendata 回報
        self.map_default_site_warning = ''  # map.default_site 不認得時的警告
        self._map_default_latlon_raw = None # 檔案裡使用者自己填的 default_lat/lon（被 default_site 取代時保留，save 寫回原值）

        # 配置文件路徑
        self.config_file = config_file or os.path.join(
            self.paths.config_dir, "settings.json"
        )
        
        # 載入配置（如果存在）
        self.load()
    
    def load(self) -> bool:
        """
        從文件載入配置
        
        返回:
            是否成功載入
        """
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                
                # 更新各項配置
                if 'paths' in config_data:
                    # ⚠ 存檔路徑會蓋掉計算值 ⇒ 搬家後必須先校正，
                    #   否則八條路徑全部指向舊位置（見 _rebase_saved_paths）。
                    self.paths = PathSettings(**_rebase_saved_paths(config_data['paths']))
                legacy = {}
                if 'map' in config_data:
                    raw_map = config_data['map']
                    if isinstance(raw_map, dict):     # 第一版放在 map 底下的 3D 機體鍵 ⇒ 搬到 model3d
                        raw_map = dict(raw_map)
                        legacy = {k: raw_map.pop(k) for k in _MODEL3D_KEYS_FORMERLY_IN_MAP if k in raw_map}
                    kw, self.map_load_warning = _known_fields_only(MapSettings, raw_map, 'map')
                    self.map = MapSettings(**kw)
                    self._apply_default_site()
                    if legacy and 'model3d' not in config_data:
                        self.model3d, self.model3d_load_warning = _load_model3d(legacy)
                if 'model3d' in config_data:
                    self.model3d, self.model3d_load_warning = _load_model3d(config_data['model3d'])
                    if legacy:          # 兩處都有 ⇒ 以 model3d 為準；只記鍵名
                        w = (f"settings.json 的 map 區塊仍有舊鍵 {sorted(legacy)}，但已有 model3d 區塊 ⇒ 以 model3d 為準、"
                             f"舊鍵已忽略（下次存檔會移除）")
                        print(w)
                        self.model3d_load_warning = (self.model3d_load_warning + '；' if self.model3d_load_warning else '') + w
                if 'export' in config_data:
                    self.export = ExportSettings(**config_data['export'])
                if 'performance' in config_data:
                    self.performance = PerformanceSettings(**config_data['performance'])
                if 'safety' in config_data:
                    self.safety = SafetySettings(**config_data['safety'])
                if 'ui' in config_data:
                    self.ui = UISettings(**config_data['ui'])
                if 'sitl_launch' in config_data:
                    # SITL 啟動彈窗設定 — instances 維持 list[dict]，不再包裝
                    self.sitl_launch = SITLLaunchSettings(**config_data['sitl_launch'])
                if 'embedded_swarm' in config_data:
                    # 嵌入式蜂群連線設定（模式 A1：AeroPlan 當監看 GCS）
                    self.embedded_swarm = EmbeddedSwarmSettings(
                        **config_data['embedded_swarm']
                    )
                if 'hil' in config_data:
                    self.hil = HILSettings(**config_data['hil'])
                if 'weather' in config_data:
                    self.weather, self.weather_load_warning = _load_weather(config_data['weather'])

                return True
        except Exception as e:
            print(f"載入配置失敗: {e}")
        
        return False
    
    def _apply_default_site(self) -> None:
        """map.default_site（場地預設 key）⇒ 生效的 map.default_lat/lon 換成該場地；不認得的 key 印警告、維持原值。

        檔案裡使用者自己填的 default_lat/lon 留在 _map_default_latlon_raw，save() 寫回的是原值
        （否則按一次 SITL 對話框的「儲存為預設」就把原值靜默換掉，之後清空 default_site 也回不去）。
        """
        self.map_default_site_warning = ''
        self._map_default_latlon_raw = None
        key = str(getattr(self.map, 'default_site', '') or '').strip()
        if not key:
            return
        from config.spawn_presets import SPAWN_PRESETS, get_preset_by_key
        p = get_preset_by_key(key)
        if p is None:
            self.map_default_site_warning = (f"settings.json map.default_site={key!r} 不是已知場地預設"
                                             f"（可用：{[q.key for q in SPAWN_PRESETS]}），地圖中心維持 default_lat/lon")
            print(self.map_default_site_warning)
            return
        self._map_default_latlon_raw = (self.map.default_lat, self.map.default_lon)
        self.map.default_lat, self.map.default_lon = float(p.lat), float(p.lon)
        print(f"settings.json map.default_site={p.key}：地圖中心用 {p.lat:.6f}, {p.lon:.6f}"
              f"（檔案的 default_lat/lon {self._map_default_latlon_raw} 保留不改）")

    def _map_for_save(self) -> dict:
        """asdict(self.map)，但 default_lat/lon 寫回使用者原值（default_site 生效時）。"""
        d = asdict(self.map)
        raw = getattr(self, '_map_default_latlon_raw', None)
        if raw is not None:
            d['default_lat'], d['default_lon'] = raw
        return d

    def save(self) -> bool:
        """
        儲存配置到文件
        
        返回:
            是否成功儲存
        """
        try:
            config_data = {
                'paths': asdict(self.paths),
                'map': self._map_for_save(),
                'export': asdict(self.export),
                'performance': asdict(self.performance),
                'safety': asdict(self.safety),
                'ui': asdict(self.ui),
                'sitl_launch': asdict(self.sitl_launch),
                'embedded_swarm': asdict(self.embedded_swarm),
                'hil': asdict(self.hil),
                'weather': asdict(self.weather),
                'model3d': asdict(self.model3d),
            }

            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            
            return True
        except Exception as e:
            print(f"儲存配置失敗: {e}")
            return False
    
    def reset_to_default(self):
        """重設為預設配置"""
        self.paths = PathSettings()
        self.map = MapSettings()
        self.export = ExportSettings()
        self.performance = PerformanceSettings()
        self.safety = SafetySettings()
        self.ui = UISettings()
        self.sitl_launch = SITLLaunchSettings()
        self.embedded_swarm = EmbeddedSwarmSettings()
        self.hil = HILSettings()
        self.model3d = Model3DSettings()

    def get_dict(self) -> Dict[str, Any]:
        """獲取配置字典"""
        return {
            'paths': asdict(self.paths),
            'map': self._map_for_save(),
            'export': asdict(self.export),
            'performance': asdict(self.performance),
            'safety': asdict(self.safety),
            'ui': asdict(self.ui),
            'sitl_launch': asdict(self.sitl_launch),
            'embedded_swarm': asdict(self.embedded_swarm),
            'hil': asdict(self.hil),
            'model3d': asdict(self.model3d),
        }


# 全局配置實例
_global_settings: Optional[GlobalSettings] = None


def get_settings() -> GlobalSettings:
    """
    獲取全局配置實例（單例模式）
    
    返回:
        GlobalSettings實例
    """
    global _global_settings
    if _global_settings is None:
        _global_settings = GlobalSettings()
    return _global_settings


def init_settings(config_file: Optional[str] = None) -> GlobalSettings:
    """
    初始化全局配置
    
    參數:
        config_file: 配置文件路徑（可選）
    
    返回:
        GlobalSettings實例
    """
    global _global_settings
    _global_settings = GlobalSettings(config_file)
    return _global_settings
