"""
AeroPlan Studio 專案檔保存/讀取 (.aeroplan)
=============================================

序列化當前 GUI 狀態為 JSON，方便使用者儲存「正在規劃中的情境」：
    - 作業區域邊界 (corners)
    - 已標記的打擊目標
    - STOT 發射基地
    - 參數面板所有設定
    - 最近一次 strike_result (不含大型 trajectories，只存 metadata)

檔案格式：JSON (.aeroplan 副檔名)，人類可讀、版本可演進。
未涵蓋：DCCPP 原始規劃結果 (太大，且依賴 numpy ndarray，另存 .npz)
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# 檔案格式版本 — 日後可據此做向後相容遷移
AEROPLAN_PROJECT_VERSION = 1


@dataclass
class AeroPlanProject:
    """一份 AeroPlan Studio 專案的完整 JSON schema

    儲存的是「UI 層的使用者輸入」，不存規劃結果（那些可重算）。
    """
    version: int = AEROPLAN_PROJECT_VERSION
    saved_at: str = field(default_factory=lambda: datetime.now().isoformat(timespec='seconds'))
    app_version: str = '2.6.0'

    # ── 作業區域 ────────────────────────────────────
    corners: List[Tuple[float, float]] = field(default_factory=list)
    home_point: Optional[Tuple[float, float]] = None
    circle_center: Optional[Tuple[float, float]] = None
    circle_radius_m: float = 0.0
    nfz_zones: List[Dict[str, Any]] = field(default_factory=list)  # 禁航區

    # ── 蜂群打擊 ────────────────────────────────────
    strike_targets: List[Tuple[float, float]] = field(default_factory=list)
    strike_launch_base: Optional[Tuple[float, float]] = None
    strike_params: Dict[str, Any] = field(default_factory=dict)

    # ── 飛行參數 ────────────────────────────────────
    flight_params: Dict[str, Any] = field(default_factory=dict)
    vehicle_type: str = '多旋翼'

    # ── 備註 ──────────────────────────────────────
    notes: str = ''


# ═══════════════════════════════════════════════════════════════════════
#  保存 / 讀取
# ═══════════════════════════════════════════════════════════════════════

def save_project(project: AeroPlanProject, path: str) -> None:
    """寫出專案檔 (.aeroplan JSON)"""
    p = Path(path)
    if p.suffix == '':
        p = p.with_suffix('.aeroplan')
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(asdict(project), f, ensure_ascii=False, indent=2)


def load_project(path: str) -> AeroPlanProject:
    """讀入專案檔並回傳 AeroPlanProject"""
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # 簡易版本遷移
    version = data.get('version', 0)
    if version > AEROPLAN_PROJECT_VERSION:
        raise ValueError(
            f'專案版本 {version} > 支援最高版本 {AEROPLAN_PROJECT_VERSION}；'
            f'請升級 AeroPlan Studio'
        )

    # tuple 欄位 JSON 反序列化會變 list，手動轉回
    def _to_tup(v, n=2):
        if v is None:
            return None
        if isinstance(v, (list, tuple)) and len(v) >= n:
            return tuple(v[:n])
        return None

    return AeroPlanProject(
        version=data.get('version', AEROPLAN_PROJECT_VERSION),
        saved_at=data.get('saved_at', ''),
        app_version=data.get('app_version', ''),
        corners=[tuple(c) for c in data.get('corners', [])],
        home_point=_to_tup(data.get('home_point')),
        circle_center=_to_tup(data.get('circle_center')),
        circle_radius_m=float(data.get('circle_radius_m', 0.0)),
        nfz_zones=data.get('nfz_zones', []),
        strike_targets=[tuple(t) for t in data.get('strike_targets', [])],
        strike_launch_base=_to_tup(data.get('strike_launch_base')),
        strike_params=data.get('strike_params', {}),
        flight_params=data.get('flight_params', {}),
        vehicle_type=data.get('vehicle_type', '多旋翼'),
        notes=data.get('notes', ''),
    )


# ═══════════════════════════════════════════════════════════════════════
#  從 MainWindow 建立 / 寫回 Project
# ═══════════════════════════════════════════════════════════════════════

def build_project_from_main_window(win) -> AeroPlanProject:
    """從執行中的 MainWindow 萃取當前狀態"""
    return AeroPlanProject(
        corners=list(getattr(win, 'corners', [])),
        home_point=getattr(win, 'home_point', None),
        circle_center=getattr(win, 'circle_center', None),
        circle_radius_m=float(getattr(win, 'flight_params', {}).get(
            'circle_max_radius', 0.0
        )),
        nfz_zones=[],   # 簡化版；完整 NFZ 序列化另外實作
        strike_targets=list(getattr(win, '_strike_targets', [])),
        strike_launch_base=getattr(win, '_strike_launch_base', None),
        strike_params={},
        flight_params=dict(getattr(win, 'flight_params', {})),
        vehicle_type=getattr(win, 'current_vehicle_type', '多旋翼'),
    )


def apply_project_to_main_window(project: AeroPlanProject, win) -> None:
    """把 Project 內容套用回 MainWindow (讀檔後復原狀態)"""
    # 邊界
    if project.corners and hasattr(win, 'map_widget'):
        # 清空再重加，確保 UI 同步
        if hasattr(win, 'on_clear_corners'):
            win.on_clear_corners()
        for lat, lon in project.corners:
            win.map_widget.add_corner(lat, lon)

    # 打擊目標
    if project.strike_targets:
        win._strike_targets = list(project.strike_targets)
        cesium = getattr(win, '_get_cesium_widget', lambda: None)()
        if cesium:
            for idx, (lat, lon) in enumerate(project.strike_targets, 1):
                try:
                    cesium.strike_add_target(lat, lon, idx)
                except Exception:
                    pass
        if hasattr(win, 'parameter_panel'):
            win.parameter_panel.update_strike_target_count(
                len(project.strike_targets)
            )

    # STOT 基地
    if project.strike_launch_base is not None:
        win._strike_launch_base = project.strike_launch_base
        if hasattr(win, 'parameter_panel'):
            win.parameter_panel.update_strike_base_label(
                *project.strike_launch_base
            )

    # 飛行參數
    if project.flight_params:
        win.flight_params.update(project.flight_params)


if __name__ == '__main__':
    # 冒煙測試
    import tempfile, os as _os
    proj = AeroPlanProject(
        corners=[(25.05, 121.55), (25.06, 121.56), (25.05, 121.57)],
        strike_targets=[(25.033, 121.5654)],
        strike_launch_base=(25.0, 121.5),
        vehicle_type='固定翼',
        notes='測試情境',
    )
    with tempfile.TemporaryDirectory() as td:
        path = _os.path.join(td, 'demo.aeroplan')
        save_project(proj, path)
        loaded = load_project(path)
        assert loaded.corners == proj.corners
        assert loaded.strike_targets == proj.strike_targets
        print('[OK] save/load round-trip 驗證通過')
        print(f'檔案大小: {_os.path.getsize(path)} bytes')
