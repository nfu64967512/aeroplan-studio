"""config/spawn_presets.py — 常用「場地」預設（單一事實來源）：SITL 出生點、規劃器起降場、地圖起始中心共用。

「出生地點」= SITL 飛機在模擬世界的生成點（home）。這裡集中維護幾個常用地點，
讓 SITL 啟動對話框可以下拉選取，免每次手打經緯度。座標與 WSL 端
`~/fixwing_ws/sitl_run/swarm_column_up.sh` 的 LOC 預設保持一致，兩邊同址。
2026-10-17 起同一份清單也給 VTOL 偵察面板「預設場地 → 套用為起降場」與
settings.json 的 ``map.default_site``（地圖起始中心）用，所以每筆都要寫 ``source``（座標從哪裡來）。

每個預設欄位：
    key        : str   — 程式用鍵值（英數，與 WSL 腳本 LOC 預設同名，大寫）
    label      : str   — UI 顯示名稱（中文）
    lat, lon   : float — WGS84 生成座標（度）
    alt_m      : float — 生成海拔（公尺），SITL 地面高度
    heading    : float — 生成機頭航向（度，0=N 順時針），跑道方向
    note       : str   — 說明（跑道 / 用途）
    source     : str   — 座標與高程的出處（OSM way id、日期、DEM…）；沒有出處的值不要加進來
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class SpawnPreset:
    key: str
    label: str
    lat: float
    lon: float
    alt_m: float
    heading: float
    note: str = ""
    source: str = ""

    def as_tuple(self) -> tuple:
        """(lat, lon, heading) — on_sitl_launch 需要的最小組合。"""
        return (self.lat, self.lon, self.heading)


# ── 常用場地（使用者 2026-10-17 定：嘉義亞創中心跑道北端、虎尾、桃園機場）──────────
# 嘉義：亞洲無人機AI創新應用研發中心（亞創中心，朴子市學府路二段 52-16，台體大舊嘉義校區）的無人機跑道
#       **北端**；跑道由 OpenStreetMap 標為 aeroway=runway、混凝土、寬 20 m，兩端點
#       (23.449654, 120.285965)–(23.451785, 120.286324)，長約 240 m、方位 8.8°/188.8°。
# 虎尾：專案既有預設作業區（與 huwei_dem.tif 同址，國立虎尾科技大學一帶；座標是專案既有值，未另行查證）。
# 桃園機場：RCTP 跑道 05，與 WSL swarm 腳本同址。
SPAWN_PRESETS: List[SpawnPreset] = [
    SpawnPreset(
        key="CHIAYI_AUIRC",
        label="嘉義 亞創中心跑道北端（亞洲無人機AI創新應用研發中心）",
        lat=23.451785,
        lon=120.286324,
        alt_m=8.0,
        heading=188.8,
        note="朴子市學府路二段 52-16 無人機跑道北端；跑道混凝土、寬 20 m、長約 240 m、方位 9°/189°，由北端往南起飛",
        source="OpenStreetMap way 1413073131（aeroway=runway）北端節點，2026-10-17 經 api.openstreetmap.org 讀取；"
               "高程 Open-Meteo elevation（Copernicus DEM 90 m）8 m",
    ),
    SpawnPreset(
        key="HUWEI",
        label="虎尾（虎尾科大 / 專案預設）",
        lat=23.702732,
        lon=120.419333,
        alt_m=0.0,
        heading=0.0,
        note="專案預設作業區，對應 huwei_dem.tif 地形",
        source="專案既有值（huwei_dem.tif 同址）；高程未查證（Open-Meteo 此點約 24 m，SITL 沿用 0）",
    ),
    SpawnPreset(
        key="TAOYUAN",
        label="桃園國際機場（RCTP 跑道 05）",
        lat=25.0777,
        lon=121.2325,
        alt_m=32.0,
        heading=52.0,
        note="RCTP/TPE 跑道 05，與 WSL swarm_column_up.sh 同址",
        source="專案既有值（與 config.MapSettings.default_lat/lon、WSL swarm_column_up.sh LOC 同址）；Open-Meteo 高程 32 m 相符",
    ),
]


def preset_labels() -> List[str]:
    """UI 下拉用的顯示名稱列表（依 SPAWN_PRESETS 順序）。"""
    return [p.label for p in SPAWN_PRESETS]


def get_preset_by_key(key: Optional[str]) -> Optional[SpawnPreset]:
    """依 key 取預設；找不到（含 None / 空字串）回 None。"""
    if not key:
        return None
    k = str(key).strip().upper()
    for p in SPAWN_PRESETS:
        if p.key == k:
            return p
    return None


def get_preset_by_label(label: Optional[str]) -> Optional[SpawnPreset]:
    """依 UI 顯示名稱取預設；找不到回 None。"""
    if not label:
        return None
    for p in SPAWN_PRESETS:
        if p.label == label:
            return p
    return None
