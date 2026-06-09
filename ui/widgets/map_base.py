"""ui.widgets.map_base — 地圖 widget 共同基底
=================================================

集中宣告 2D(MapWidget) / 3D(CesiumMapWidget) / 雙模(DualMapWidget) 三者「交集」的
5 個互動 Qt signal 一次，避免三檔各抄一份（單一事實來源）。

各子類仍可在自己的 class body 宣告專屬 signal（如 Cesium / Dual 的
strike_target_added、Dual 的 fence_built / fence_zone_*）。
"""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget


class MapWidgetBase(QWidget):
    """所有地圖 widget 的共同基底（只宣告交集 signal，不加抽象方法以避免
    QWidget 的 sip metaclass 與 ABCMeta 衝突）。

    (1-1 去重) 三者交集的 5 個互動 signal 在此宣告一次。繼承後仍為 per-instance
    pyqtBoundSignal，connect/emit 行為與原本各自宣告完全相同。
    """

    corner_added      = pyqtSignal(float, float)
    corner_moved      = pyqtSignal(int, float, float)
    circle_defined    = pyqtSignal(float, float, float)   # (lat, lon, radius_m)
    nfz_polygon_drawn = pyqtSignal(list)                  # list of (lat, lon)
    nfz_circle_drawn  = pyqtSignal(float, float, float)   # (lat, lon, radius_m)
