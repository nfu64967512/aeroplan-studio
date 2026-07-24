"""config/spawn_presets.py — SITL / 實機「出生地點」常用預設（單一事實來源）。

「出生地點」= SITL 飛機在模擬世界的生成點（home）。這裡集中維護幾個常用地點，
讓 SITL 啟動對話框可以下拉選取，免每次手打經緯度。座標與 WSL 端
`~/fixwing_ws/sitl_run/swarm_column_up.sh` 的 LOC 預設保持一致，兩邊同址。

每個預設欄位：
    key        : str   — 程式用鍵值（英數，與 WSL 腳本 LOC 預設同名，大寫）
    label      : str   — UI 顯示名稱（中文）
    lat, lon   : float — WGS84 生成座標（度）
    alt_m      : float — 生成海拔（公尺），SITL 地面高度
    heading    : float — 生成機頭航向（度，0=N 順時針），跑道方向
    note       : str   — 說明（跑道 / 用途）
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

    def as_tuple(self) -> tuple:
        """(lat, lon, heading) — on_sitl_launch 需要的最小組合。"""
        return (self.lat, self.lon, self.heading)


# ── 常用出生地點（依使用頻率排序）──────────────────────────────────────
# 虎尾：專案預設作業區（與 config.MapSettings.default_lat/lon、huwei_dem.tif 同址，
#       國立虎尾科技大學一帶）。桃園機場：RCTP 跑道 05，與 WSL swarm 腳本同址。
SPAWN_PRESETS: List[SpawnPreset] = [
    SpawnPreset(
        key="HUWEI",
        label="虎尾（虎尾科大 / 專案預設）",
        lat=23.702732,
        lon=120.419333,
        alt_m=0.0,
        heading=0.0,
        note="專案預設作業區，對應 huwei_dem.tif 地形",
    ),
    SpawnPreset(
        key="TAOYUAN",
        label="桃園國際機場（RCTP 跑道 05）",
        lat=25.0777,
        lon=121.2325,
        alt_m=32.0,
        heading=52.0,
        note="RCTP/TPE 跑道 05，與 WSL swarm_column_up.sh 同址",
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
