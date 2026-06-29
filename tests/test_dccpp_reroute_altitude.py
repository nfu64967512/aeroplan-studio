"""tests/test_dccpp_reroute_altitude.py — DCCPP NFZ 繞行段「路徑貼地」修正回歸測試。

問題
----
NFZ 避障繞行的航點，在 Dubins fillet 倒角 / kink 修復重新離散化後，原 waypoint
索引失效，改用「幾何最近原始航點」推算高度。但繞 NFZ 的 detour 點在幾何上往往
最靠近低高度的 takeoff 爬升 / landing 下降航點，於是繼承到接近地面的高度
→ 繞行段「路徑貼地」。

修正
----
在 fillet 前擷取正確高度剖面 (corrected_path + ref_indices)，最後用
``_resample_alts_by_arclength`` 沿累積弧長比例還原各點高度，使繞行 detour 點維持
其所屬巡航段的高度。

本測試：
  1. 純函式驗證 _resample_alts_by_arclength（弧長內插）。
  2. 整合驗證：跑完整 _apply_fence_avoidance_to_dccpp_result，斷言繞 NFZ 的航點
     維持巡航高度（不貼地），且降落觸地點仍為 0。
"""
from __future__ import annotations

import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import math
import types

import pytest

# ── Qt / 主視窗匯入（headless）。失敗則跳過整合測試，純函式測試仍可跑。──
try:
    from ui.main_window import MainWindow            # 先匯入（載入 QtWebEngine）
    from PyQt6.QtWidgets import QApplication
    import PyQt6.QtWidgets as _QtW
    _QtW.QMessageBox.warning = staticmethod(lambda *a, **k: 0)   # 避免彈窗阻塞
    _APP = QApplication.instance() or QApplication([])
    from core.trajectory.dccpp_path_assembler import (
        AssembledPath, AssembledWaypoint, SegmentLabel,
    )
    import mission.fence_zone as _fz
    _QT_OK = True
except Exception as _e:        # pragma: no cover - 環境相依
    _QT_OK = False
    _IMPORT_ERR = _e


# ──────────────────────────────────────────────────────────────────────
#  純函式：弧長內插
# ──────────────────────────────────────────────────────────────────────
@pytest.mark.skipif(not _QT_OK, reason="ui.main_window import failed")
class TestResampleAltsByArclength:
    def test_flat_cruise_preserved(self):
        # ref 全在巡航 100；query 任意取樣 → 全 100
        ref_xy = [(0, 0), (100, 0), (200, 0)]
        ref_alt = [100.0, 100.0, 100.0]
        query = [(0, 0), (50, 0), (150, 0), (200, 0)]
        out = MainWindow._resample_alts_by_arclength(query, ref_xy, ref_alt)
        assert out == pytest.approx([100.0, 100.0, 100.0, 100.0])

    def test_climb_profile_midpoint(self):
        # ref：0m → 100m（線性爬升）；query 中點 → 50m
        ref_xy = [(0, 0), (100, 0)]
        ref_alt = [0.0, 100.0]
        out = MainWindow._resample_alts_by_arclength([(0, 0), (50, 0), (100, 0)], ref_xy, ref_alt)
        assert out == pytest.approx([0.0, 50.0, 100.0])

    def test_detour_longer_path_keeps_cruise(self):
        # ref 直線兩端皆巡航 100；query 為「繞行」較長折線（中段外凸）
        # → 內插後中段仍為 100（不會掉到地面）
        ref_xy = [(0, 0), (200, 0)]
        ref_alt = [100.0, 100.0]
        query = [(0, 0), (50, 80), (100, 80), (150, 80), (200, 0)]  # 繞行外凸
        out = MainWindow._resample_alts_by_arclength(query, ref_xy, ref_alt)
        assert all(a == pytest.approx(100.0) for a in out)

    def test_empty_ref_returns_zeros(self):
        out = MainWindow._resample_alts_by_arclength([(0, 0), (1, 1)], [], [])
        assert out == [0.0, 0.0]


# ──────────────────────────────────────────────────────────────────────
#  整合：完整 _apply_fence_avoidance_to_dccpp_result（繞行段不貼地）
# ──────────────────────────────────────────────────────────────────────
@pytest.mark.skipif(not _QT_OK, reason="ui.main_window import failed")
class TestRerouteNotGroundHugging:
    CRUISE = 100.0
    CEN = (23.72, 120.43)
    HOME = (23.700, 120.400)

    def _wp(self, lat, lon, alt, seg):
        return AssembledWaypoint(lat=lat, lon=lon, alt=alt, heading_deg=0.0, segment_type=seg)

    def _build(self):
        """掃描線在 NFZ 北側、arrival 在西南 → 回程橫越 NFZ。
        arrival(场周入口) 設在巡航高度（builder 行為）→ 回程繞行應全程維持巡航高度。"""
        C = self.CRUISE
        h_lat, h_lon = self.HOME
        wps = [
            self._wp(h_lat, h_lon, 0.0, SegmentLabel.TAKEOFF),
            self._wp(h_lat + 0.0008, h_lon + 0.0008, 30.0, SegmentLabel.TAKEOFF),
            self._wp(23.740, 120.438, C, SegmentLabel.ENTRY),
            self._wp(23.740, 120.438, C, SegmentLabel.OPERATION),
            self._wp(23.745, 120.445, C, SegmentLabel.OPERATION),   # 最後掃描出口（NFZ 北側）
            self._wp(23.703, 120.403, C, SegmentLabel.LANDING),     # arrival @ 巡航高度
            self._wp(23.701, 120.401, 40.0, SegmentLabel.LANDING),
            self._wp(h_lat, h_lon, 0.0, SegmentLabel.LANDING),      # 觸地 NAV_LAND
        ]
        p = AssembledPath(uav_id=1)
        p.waypoints = wps
        return p

    def _run(self, path):
        _fz.FenceZoneRegistry._instance = None
        reg = _fz.FenceZoneRegistry.instance()
        reg.add(_fz.FenceZone(
            name='NFZ-1', category=_fz.ZoneCategory.NFZ, shape=_fz.ZoneShape.POLYGON,
            vertices=[(23.715, 120.418), (23.732, 120.418),
                      (23.732, 120.450), (23.715, 120.450)],
            alt_min_m=0.0, alt_max_m=200.0,
        ))
        mw = MainWindow.__new__(MainWindow)
        mw.statusBar = lambda: types.SimpleNamespace(showMessage=lambda *a, **k: None)
        mw._apply_fence_avoidance_to_dccpp_result(
            {'assembled_paths': {1: path}}, self.CEN[0], self.CEN[1], turn_radius=80.0,
        )
        return path

    @staticmethod
    def _dist_m(a_lat, a_lon, b_lat, b_lon):
        dlat = (a_lat - b_lat) * 111320.0
        dlon = (a_lon - b_lon) * 111320.0 * math.cos(math.radians(a_lat))
        return math.hypot(dlat, dlon)

    def test_reroute_happened(self):
        path = self._run(self._build())
        # 繞行應插入額外航點（原 8 點）
        assert len(path.waypoints) > 8

    def test_reroute_points_not_ground_hugging(self):
        path = self._run(self._build())
        h_lat, h_lon = self.HOME
        # 遠離 home（> 800m）且非觸地的航點 = NFZ 繞行/巡航段 → 應維持巡航高度
        far_pts = [
            w for w in path.waypoints
            if self._dist_m(w.lat, w.lon, h_lat, h_lon) > 800.0
        ]
        assert far_pts, "應有遠離 home 的繞行航點"
        min_far_alt = min(w.alt for w in far_pts)
        # 修正前這些 detour 點會繼承到 takeoff(0~30m)/landing 低高度而貼地；
        # 修正後沿弧長維持巡航高度。容許小誤差。
        assert min_far_alt >= self.CRUISE - 5.0, (
            f"繞行段貼地：遠離 home 的最低高度 {min_far_alt:.1f}m « 巡航 {self.CRUISE}m"
        )

    def test_touchdown_preserved_zero_alt(self):
        path = self._run(self._build())
        last = path.waypoints[-1]
        assert last.segment_type.name == 'LANDING'
        assert last.alt == pytest.approx(0.0)
        assert self._dist_m(last.lat, last.lon, *self.HOME) < 30.0
