"""tests/test_dccpp_coverage_retention.py — NFZ 避障的「偵察覆蓋率保留」回歸測試。

偵察任務以覆蓋率為優先：穿越 NFZ 的掃描線在 NFZ 外的「兩側」覆蓋都必須保留，
不可因避障而丟整條尾段。

背景 bug（已修）：_segment_aware_fence_avoidance 舊的 Case A 在「掃描線中段/遠端
落在 NFZ 內」時，會 `while _pt_in_any_poly: j += 1` 跳過所有 NFZ 內航點，連帶把該
掃描線（甚至後續多條）在 NFZ 另一側的覆蓋整段丟棄 → 偵察區大量空洞。

修法：改用 difference-based 邊界截斷 —— 對每個 OPERATION 線段取
LineString(lane).difference(buffered_NFZ)，保留「所有」NFZ 外子段，缺口才用
correct_path 繞行。不重排（保留起飛順序 / Dubins R_min / geofence）。
"""
from __future__ import annotations

import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import math
import pytest

try:
    from ui.main_window import MainWindow            # 先匯入（載入 QtWebEngine）
    from PyQt6.QtWidgets import QApplication
    import PyQt6.QtWidgets as _QtW
    _QtW.QMessageBox.warning = staticmethod(lambda *a, **k: 0)
    _APP = QApplication.instance() or QApplication([])
    from shapely.geometry import LineString, Polygon
    from core.trajectory.dccpp_path_assembler import (
        AssembledWaypoint, AssembledPath, SegmentLabel,
    )
    from core.global_planner.nfz_planner import FixedWingNFZPlanner
    from core.base.fixed_wing_constraints import FixedWingConstraints
    import mission.fence_zone as _fz
    import types as _types
    _QtW.QMessageBox.information = staticmethod(lambda *a, **k: 0)
    _QT_OK = True
except Exception:
    _QT_OK = False


# NFZ 矩形（公尺）：x,y ∈ [-1500, 1500]，置中
_NFZ_XY = [(-1500.0, -1500.0), (1500.0, -1500.0), (1500.0, 1500.0), (-1500.0, 1500.0)]


def _wp(seg):
    return AssembledWaypoint(lat=0.0, lon=0.0, alt=100.0, heading_deg=0.0, segment_type=seg)


def _planner():
    c = FixedWingConstraints(cruise_airspeed_mps=18.0, max_bank_angle_deg=45.0)
    p = FixedWingNFZPlanner(c, buffer_factor=1.5)
    p.add_polygon_nfz(_NFZ_XY, name='NFZ-1', coord_type='metric')
    p._rebuild_merged()
    return p


def _penetrates(pts, poly, tol=1.0):
    for k in range(len(pts) - 1):
        seg = LineString([pts[k], pts[k + 1]])
        if seg.intersects(poly):
            it = seg.intersection(poly)
            if not it.is_empty and it.geom_type not in ('Point', 'MultiPoint') \
                    and getattr(it, 'length', 0.0) > tol:
                return True
    return False


@pytest.mark.skipif(not _QT_OK, reason="ui.main_window import failed")
class TestCoverageRetention:
    def test_through_crossing_keeps_both_sides(self):
        """穿越 NFZ 的掃描線（兩端在外）→ 左右兩側覆蓋都保留。"""
        wps = [_wp(SegmentLabel.OPERATION), _wp(SegmentLabel.OPERATION)]
        metric = [(-5000.0, 0.0), (5000.0, 0.0)]   # 一條橫貫 NFZ 的掃描線
        out, refs = MainWindow._segment_aware_fence_avoidance(wps, metric, _planner())
        xs = [p[0] for p in out]
        assert min(xs) <= -1500.0, f"左側覆蓋丟失 (min_x={min(xs):.0f})"
        assert max(xs) >= 1500.0, f"右側覆蓋丟失 (max_x={max(xs):.0f})"
        assert not _penetrates(out, Polygon(_NFZ_XY)), "輸出仍穿越 NFZ"

    def test_lane_with_inside_midpoint_keeps_far_side(self):
        """掃描線中段落在 NFZ 內（舊 Case A 會丟遠端）→ 遠端覆蓋仍保留。"""
        # 三點掃描線：左(外) → 中(NFZ 內) → 右(外)
        wps = [_wp(SegmentLabel.OPERATION), _wp(SegmentLabel.OPERATION), _wp(SegmentLabel.OPERATION)]
        metric = [(-5000.0, 0.0), (0.0, 0.0), (5000.0, 0.0)]
        out, refs = MainWindow._segment_aware_fence_avoidance(wps, metric, _planner())
        xs = [p[0] for p in out]
        # 關鍵：右側(遠端) 覆蓋必須保留 —— 舊 Case A 會在此整段丟棄
        assert max(xs) >= 1500.0, f"遠端(右側)覆蓋被丟棄 (max_x={max(xs):.0f})"
        assert min(xs) <= -1500.0, f"近端(左側)覆蓋丟失 (min_x={min(xs):.0f})"
        assert not _penetrates(out, Polygon(_NFZ_XY)), "輸出仍穿越 NFZ"
        assert len(refs) == len(out)

    def test_non_crossing_lane_unchanged(self):
        """不穿越 NFZ 的掃描線 → 原樣保留。"""
        wps = [_wp(SegmentLabel.OPERATION), _wp(SegmentLabel.OPERATION)]
        metric = [(-5000.0, 4000.0), (5000.0, 4000.0)]   # 在 NFZ 上方，不穿越
        out, _ = MainWindow._segment_aware_fence_avoidance(wps, metric, _planner())
        assert out == metric

    def test_fully_inside_lane_dropped_cleanly(self):
        """完全在 NFZ 內的掃描線 → 不保留覆蓋、且不穿越（不可亂連）。"""
        wps = [_wp(SegmentLabel.OPERATION), _wp(SegmentLabel.OPERATION), _wp(SegmentLabel.OPERATION)]
        metric = [(-5000.0, 0.0), (-500.0, 0.0), (500.0, 0.0)]  # 後兩點在 NFZ 內
        out, _ = MainWindow._segment_aware_fence_avoidance(wps, metric, _planner())
        assert min(p[0] for p in out) <= -1500.0   # 左側外部覆蓋保留
        assert not _penetrates(out, Polygon(_NFZ_XY))


@pytest.mark.skipif(not _QT_OK, reason="ui.main_window import failed")
class TestNoCrossingTiltedNFZ:
    """端到端：傾斜 NFZ + 跨越航線跑完整 _apply_fence_avoidance_to_dccpp_result，
    斷言任何航段都不穿越 NFZ（涵蓋最終 gate 的 difference 裁切 + 邊界走線後備）。

    背景 bug（已修）：傾斜/壓邊界 NFZ 下，掃描線深入 NFZ 使航點落在 buffered NFZ
    內 → correct_path 的 VG 無法從內部點起算 → gate 軟性保留穿越。修法：gate 改
    difference 裁切 + 沿外緣走邊界後備，保證不穿越。
    """
    CEN = (23.74, 120.45)
    TH = math.radians(28.0)

    def _m2ll(self, x, y):
        return (self.CEN[0] + y / 111320.0,
                self.CEN[1] + x / (111320.0 * math.cos(math.radians(self.CEN[0]))))

    def _M(self, lat, lon):
        return ((lon - self.CEN[1]) * 111320.0 * math.cos(math.radians(self.CEN[0])),
                (lat - self.CEN[0]) * 111320.0)

    def _rot(self, x, y):
        c, s = math.cos(self.TH), math.sin(self.TH)
        return (x * c - y * s, x * s + y * c)

    def _run(self):
        # 傾斜 NFZ（旋轉矩形）+ 水平 boustrophedon 掃描線深入穿越
        nfz_xy = [self._rot(-1200, -1000), self._rot(1200, -1000),
                  self._rot(1200, 1000), self._rot(-1200, 1000)]
        NFZ = [self._m2ll(x, y) for x, y in nfz_xy]
        home = self._m2ll(-6000, -3000)

        def wp(lat, lon, seg, alt=100.0):
            return AssembledWaypoint(lat=lat, lon=lon, alt=alt, heading_deg=0.0, segment_type=seg)

        wps = [wp(home[0], home[1], SegmentLabel.TAKEOFF, 0.0)]
        for i in range(13):
            yy = -3000 + i * 500
            s, e = ((-5000, yy), (5000, yy)) if i % 2 == 0 else ((5000, yy), (-5000, yy))
            sll, ell = self._m2ll(*s), self._m2ll(*e)
            wps.append(wp(sll[0], sll[1], SegmentLabel.ENTRY if i == 0 else SegmentLabel.TRANSFER))
            wps.append(wp(sll[0], sll[1], SegmentLabel.OPERATION))
            wps.append(wp(ell[0], ell[1], SegmentLabel.OPERATION))
        wps.append(wp(home[0], home[1], SegmentLabel.LANDING, 0.0))
        path = AssembledPath(uav_id=1); path.waypoints = wps

        _fz.FenceZoneRegistry._instance = None
        reg = _fz.FenceZoneRegistry.instance()
        reg.add(_fz.FenceZone(name='NFZ-1', category=_fz.ZoneCategory.NFZ,
                              shape=_fz.ZoneShape.POLYGON, vertices=NFZ,
                              alt_min_m=0.0, alt_max_m=200.0))
        mw = MainWindow.__new__(MainWindow)
        mw.statusBar = lambda: _types.SimpleNamespace(showMessage=lambda *a, **k: None)
        mw._apply_fence_avoidance_to_dccpp_result(
            {'assembled_paths': {1: path}}, self.CEN[0], self.CEN[1], turn_radius=80.0,
        )
        return path, Polygon([self._M(la, lo) for la, lo in NFZ])

    def test_no_segment_crosses_nfz(self):
        path, nfz_poly = self._run()
        crossing = 0
        for i in range(len(path.waypoints) - 1):
            a, b = path.waypoints[i], path.waypoints[i + 1]
            seg = LineString([self._M(a.lat, a.lon), self._M(b.lat, b.lon)])
            if seg.intersects(nfz_poly):
                it = seg.intersection(nfz_poly)
                if not it.is_empty and it.geom_type not in ('Point', 'MultiPoint') \
                        and getattr(it, 'length', 0.0) > 1.0:
                    crossing += 1
        assert crossing == 0, f"仍有 {crossing} 段穿越傾斜 NFZ"

    def test_both_sides_still_covered(self):
        path, nfz_poly = self._run()
        xs = [self._M(w.lat, w.lon)[0] for w in path.waypoints]
        assert min(xs) < -1500.0 and max(xs) > 1500.0, "傾斜 NFZ 兩側覆蓋未保留"
