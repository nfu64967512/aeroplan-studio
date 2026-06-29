"""tests/test_survey_return_leg_nfz.py — 巡航/多旋翼回程 NFZ 繞行修正回歸測試。

背景
----
DCCPP 固定翼路徑的回程穿越 NFZ 已於前次修正（`_split_landing_tail` +
`_apply_fence_avoidance_to_dccpp_result`）。本次關閉另外兩條回程缺口：

  1) Legacy 固定翼 survey（`_auto_generate_path`）：先前 `takeoff_ll`/`landing_ll`
     離場/回程段跳過 NFZ 檢查；改為把「離場 + 掃描 + 進場入口」整條巡航包絡一起
     繞行，僅豁免最終下降進場型態。
  2) 多旋翼 / grid survey：回程僅靠匯出端 bare RTL（cmd 20，韌體直線飛回 home →
     穿越 NFZ）；改以 `_append_nfz_safe_return` 在 `self.waypoints`/`sub_paths` 尾端
     顯式接上 NFZ 繞行回程。

本測試直接在 `MainWindow.__new__` 裸實例上呼叫 `_append_nfz_safe_return`（只需設定
`self.nfz_zones` 與 `self.flight_params`），於公尺座標以 shapely 驗證回程不穿越 NFZ。
"""
from __future__ import annotations

import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest

try:
    from ui.main_window import MainWindow            # 先匯入（載入 QtWebEngine）
    from PyQt6.QtWidgets import QApplication
    import PyQt6.QtWidgets as _QtW
    _QtW.QMessageBox.warning = staticmethod(lambda *a, **k: 0)
    _QtW.QMessageBox.information = staticmethod(lambda *a, **k: 0)
    _APP = QApplication.instance() or QApplication([])
    from shapely.geometry import LineString, Polygon
    from utils.math_utils import latlon_to_meters
    _QT_OK = True
except Exception as _e:        # pragma: no cover
    _QT_OK = False


# NFZ 矩形（lat/lon）介於 home(西) 與最後掃描點(東) 之間
_NFZ_LL = [(23.695, 120.440), (23.705, 120.440),
           (23.705, 120.460), (23.695, 120.460)]
_HOME = (23.700, 120.400)
_REF = (23.700, 120.450)   # 投影參考原點（接近場景中心）


def _nfz_polygon_metric():
    verts = [latlon_to_meters(lat, lon, _REF[0], _REF[1]) for lat, lon in _NFZ_LL]
    return Polygon(verts)


def _penetrates(seg_latlon, poly_m, tol_m=0.5):
    """latlon 折線是否穿入 NFZ（公尺投影；沿邊/觸點不算）。"""
    pts_m = [latlon_to_meters(lat, lon, _REF[0], _REF[1]) for lat, lon in seg_latlon]
    if len(pts_m) < 2:
        return False
    line = LineString(pts_m)
    if not line.intersects(poly_m):
        return False
    inter = line.intersection(poly_m)
    if inter.is_empty or inter.geom_type in ('Point', 'MultiPoint'):
        return False
    return getattr(inter, 'length', 0.0) > tol_m


def _make_mw(nfz_zones):
    mw = MainWindow.__new__(MainWindow)
    mw.nfz_zones = nfz_zones
    # _apply_nfz_correction_latlon → _build_fw_params 需要 flight_params
    mw.flight_params = {'speed': 18.0, 'altitude': 100.0, 'turn_radius': 50.0,
                        'fw_max_bank_deg': 45.0}
    return mw


def _nfz_dict():
    return [{'type': 'polygon', 'name': 'NFZ-1', 'vertices': list(_NFZ_LL)}]


@pytest.mark.skipif(not _QT_OK, reason="ui.main_window import failed")
class TestAppendNfzSafeReturn:
    # 掃描路徑：home(西) → 中段 → 最後掃描點(東)。回程 last→home 會橫越 NFZ。
    PATH = [_HOME, (23.700, 120.420), (23.700, 120.500)]

    def test_no_nfz_returns_unchanged(self):
        mw = _make_mw([])
        out = mw._append_nfz_safe_return(list(self.PATH), _REF[0], _REF[1])
        assert out == self.PATH

    def test_clear_return_returns_unchanged(self):
        # NFZ 遠在北方，直線回程不穿越 → 不附加
        far_nfz = [{'type': 'polygon', 'name': 'NFZ-far',
                    'vertices': [(23.80, 120.40), (23.81, 120.40),
                                 (23.81, 120.41), (23.80, 120.41)]}]
        mw = _make_mw(far_nfz)
        out = mw._append_nfz_safe_return(list(self.PATH), _REF[0], _REF[1])
        assert out == self.PATH

    def test_crossing_return_routed_multirotor(self):
        mw = _make_mw(_nfz_dict())
        out = mw._append_nfz_safe_return(list(self.PATH), _REF[0], _REF[1],
                                         fixed_wing=False)
        # 應附加繞行回程航點
        assert len(out) > len(self.PATH)
        # 回程（從原最後掃描點起）不得穿越 NFZ
        return_leg = out[len(self.PATH) - 1:]   # 含原 last 與後續繞行點
        assert not _penetrates(return_leg, _nfz_polygon_metric()), \
            "多旋翼回程仍穿越 NFZ"
        # 回程終點 == home
        assert out[-1] == pytest.approx(_HOME)

    def test_crossing_return_routed_fixedwing(self):
        mw = _make_mw(_nfz_dict())
        out = mw._append_nfz_safe_return(list(self.PATH), _REF[0], _REF[1],
                                         fixed_wing=True)
        assert len(out) > len(self.PATH)
        return_leg = out[len(self.PATH) - 1:]
        assert not _penetrates(return_leg, _nfz_polygon_metric()), \
            "固定翼回程仍穿越 NFZ"
        assert out[-1] == pytest.approx(_HOME)

    def test_straight_return_would_cross_baseline(self):
        # 前提檢查：未修正前，直線 last→home 確實穿越 NFZ（否則測試無意義）
        assert _penetrates([self.PATH[-1], _HOME], _nfz_polygon_metric())

    def test_short_path_returns_unchanged(self):
        mw = _make_mw(_nfz_dict())
        assert mw._append_nfz_safe_return([_HOME], _REF[0], _REF[1]) == [_HOME]
