"""
GeofenceConstraintManager — 強制綁定於路徑生成工作流的矩形電子圍籬管理器

依飛安最高原則：任何生成飛行路徑的函式皆必須自動綁定一組涵蓋全部航點、且
頂點數量鎖定在 4～5 個的矩形電子圍籬（Geofence）。本模組提供：

    1.  ``GeofenceConstraintManager``
        - 以 pyproj AEQD 投影將 WGS84 座標轉至公尺平面
        - 使用 shapely ``.envelope`` 或 ``.minimum_rotated_rectangle`` 產出
          4 頂點（+閉合點 = 5 座標）的矩形包絡線
        - 於平面上外擴 ``buffer_radius_m`` 公尺
        - 反投影回 WGS84，確保 ArduPilot ``AC_PolyFence_loader`` 所需
          之環向（外框逆時針為「內含」，題目簡化 → 我們輸出順時針閉合環，
          並將 inclusion 旗標標為 True）

    2.  ``MissionBundle``
        將「航點 + 圍籬頂點 + 飛控參數」封裝在同一物件中，
        杜絕「產生了路徑卻忘了產圍籬」的工作流漏洞。

    3.  ``@enforce_geofence`` 裝飾器
        強制攔截任何回傳航點的生成函式，自動附加 Geofence，
        回傳 ``MissionBundle`` 而非原始航點串列。

    4.  ``export_qgc_plan() / export_mission_planner_fen()``
        匯出至 QGroundControl ``.plan`` 與 Mission Planner ``.fen`` 格式。

MIL-STD-1472H §5.17.25 色彩語意僅處理 UI，本模組為純運算層，
所有異常皆以建設性訊息拋出 ``GeofenceError``，供上層以
``MilStdMessageBox.warning`` 顯示。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import wraps
from pathlib import Path
from typing import (
    Any,
    Callable,
    Iterable,
    List,
    Optional,
    Sequence,
    Tuple,
    TypeVar,
    Union,
)

import pyproj
from shapely import affinity
from shapely.geometry import MultiPoint, Polygon


# ═════════════════════════════════════════════════════════════════════
#  型別別名
# ═════════════════════════════════════════════════════════════════════
LatLon = Tuple[float, float]           # (lat, lon) 度
LatLonAlt = Tuple[float, float, float]  # (lat, lon, alt_m)
# 本模組所有「座標輸入」統一採 (lat, lon[, alt]) 順序，避免 lon/lat 互換錯誤。

T = TypeVar("T")


# ═════════════════════════════════════════════════════════════════════
#  自訂例外 — 建設性訊息以利 MIL-STD-1472H §5.17.10.7.3 UI 顯示
# ═════════════════════════════════════════════════════════════════════
class GeofenceError(RuntimeError):
    """圍籬計算失敗。訊息必須為建設性、可操作的指示。"""


# ═════════════════════════════════════════════════════════════════════
#  MissionBundle — 航點 + 圍籬 + 飛控參數 單一事實來源
# ═════════════════════════════════════════════════════════════════════
@dataclass
class Geofence:
    """單一矩形電子圍籬。

    Attributes
    ----------
    vertices : list[LatLon]
        矩形 4 個角的 (lat, lon) — 不含閉合點。
    alt_min_m : float
        最低允許高度（AGL 或 AMSL，依任務約定）。
    alt_max_m : float
        最高允許高度。
    buffer_radius_m : float
        當初用於外擴的半徑。
    method : str
        產生方式：'aabb'（軸對齊）、'mrr'（最小旋轉矩形）、'scan'（對齊掃描角）或 'hull'（貼合航線的凸多邊形）。
    inclusion : bool
        ArduPilot Polygon Fence 方向旗標：
        True = 內含（航機需留在內部）；False = 排除區（禁飛區）。
    """

    vertices: List[LatLon]
    alt_min_m: float
    alt_max_m: float
    buffer_radius_m: float
    method: str = "aabb"
    inclusion: bool = True

    def closed_ring(self) -> List[LatLon]:
        """回傳頭尾閉合的環（5 個座標，第 5 = 第 1）— QGC/MP 格式用。"""
        if not self.vertices:
            return []
        return [*self.vertices, self.vertices[0]]

    def vertex_count(self) -> int:
        """回傳獨立頂點數（不含閉合點）。恆為 4。"""
        return len(self.vertices)


@dataclass
class MissionBundle:
    """任務封裝：航點 + 圍籬 + 飛控參數。

    任何路徑生成函式經 ``@enforce_geofence`` 裝飾後，回傳的不再是單純
    航點串列，而是本封裝物件 — 強制確保圍籬不會被遺忘。
    """

    waypoints: List[LatLonAlt]
    geofence: Geofence
    fence_params: dict[str, Any]
    raw_result: Any = None  # 原始函式回傳值（供上層需要時使用）

    def summary(self) -> str:
        """回傳單行摘要（供 log / status bar 使用，建設性語氣）。"""
        g = self.geofence
        return (
            f"[MissionBundle] waypoints={len(self.waypoints)}  "
            f"fence={g.vertex_count()}-vtx {g.method.upper()} "
            f"alt=[{g.alt_min_m:.1f},{g.alt_max_m:.1f}]m  "
            f"buffer={g.buffer_radius_m:.0f}m"
        )


# ═════════════════════════════════════════════════════════════════════
#  GeofenceConstraintManager
# ═════════════════════════════════════════════════════════════════════
class GeofenceConstraintManager:
    """電子圍籬管理器 —— **預設產生貼合航線的凸多邊形**（'hull'）。

    ⭐ 2026-10-08 使用者定調：「規劃器出來的圍籬都要貼合路徑，這是底層規劃的」⇒ 底層預設就是
    'hull'，所有規劃器（基本演算法、固定翼、DCCPP、HARVEST…）不用各自選。矩形（aabb/mrr/scan）
    保留為**明確指定**才用的選項。

    Parameters
    ----------
    buffer_radius_m : float
        矩形外擴的緩衝半徑（公尺）。預設 30 m — 對多旋翼保守；
        固定翼建議 ≥ 100 m（與轉彎半徑匹配）。
    alt_margin_m : float
        高度上下限的安全容裕。FENCE_ALT_MAX = 航點最高 + margin；
        FENCE_ALT_MIN = max(航點最低 - margin, 2.0) — 至少保留 2 m 地面淨空。
    method : {'aabb', 'mrr', 'scan'}
        'aabb' — shapely ``.envelope``（軸對齊矩形；ArduPilot 操作員
                最直觀、RTL 行為可預期）。
        'mrr'  — ``.minimum_rotated_rectangle``（最小面積旋轉矩形；若航線
                呈對角狹長帶狀則節省空域）。
        'scan' — 對齊掃描角（見 ``build(scan_angle_deg=)``）。選了這個就
                **必須**在 build 時給角度，否則拋 ``GeofenceError``。
                一般不需要自己指定：只要 build 帶了 ``scan_angle_deg``
                就會自動走這一種。
        'hull' — **貼合航線的凸多邊形**（2026-10-08 使用者定調：圍籬要「平均覆蓋全部
                飛行路徑」）。所有點的凸包四周平均外擴 buffer，頂點數 ≤ ``max_vertices``。
                任何矩形在起降場位於作業區外、或作業區不是長方形時都會在角落留下大片
                空白（實測使用者 2026-10-08 那份規劃：軸對齊 6.10 km²、對齊條帶 6.43、
                MRR 5.53、凸多邊形 4.32 km²）。與 ``scan_angle_deg`` 互斥。
    max_vertices : int
        僅 'hull'：多邊形頂點上限（ArduPilot 圍籬點存在飛控儲存區，別讓它無限長）。
        ``FENCE_ACTION`` 值。預設 1 = RTL/QRTL（題目規格）。
        2=Always Land, 3=SmartRTL, 4=Brake, 5=SmartRTL or Land。
    fence_margin_m : float
        ``FENCE_MARGIN``：飛控提前告警距離（公尺）。

    範例
    ----
    >>> mgr = GeofenceConstraintManager(buffer_radius_m=50)
    >>> bundle = mgr.build(waypoints=[(24.78, 120.99, 50), (24.79, 121.00, 60)])
    >>> print(bundle.geofence.vertices)
    """

    # ArduPilot FENCE_TYPE bitmask（Copter/Plane 4.2+）
    #   bit 0 (1)  = Max altitude
    #   bit 1 (2)  = Circle
    #   bit 2 (4)  = Polygon
    #   bit 3 (8)  = Min altitude
    # 題目指定 FENCE_TYPE=7 (= MaxAlt + Circle + Polygon)。
    # 若需 Polygon+Max+Min (最嚴格)，改為 13 (= 0b1101)。
    FENCE_TYPE_DEFAULT: int = 7

    def __init__(
        self,
        buffer_radius_m: float = 30.0,
        alt_margin_m: float = 20.0,
        method: str = "hull",
        fence_action: int = 1,
        fence_margin_m: float = 5.0,
        max_vertices: int = 12,
    ) -> None:
        if buffer_radius_m <= 0:
            raise GeofenceError(
                "緩衝半徑必須為正值。"
                "建議：多旋翼 30 m，固定翼 ≥ 100 m。"
            )
        if alt_margin_m <= 0:
            raise GeofenceError(
                "高度容裕必須為正值。建議 ≥ 10 m 以吸收氣壓計漂移。"
            )
        if method not in ("aabb", "mrr", "scan", "hull"):
            raise GeofenceError(
                f"未知的圍籬產生方式 '{method}'。"
                "請改為 'aabb'（軸對齊）、'mrr'（最小旋轉矩形）、"
                "'scan'（對齊掃描角，需在 build 時給 scan_angle_deg）"
                "或 'hull'（貼合航線的凸多邊形）。"
            )
        if int(max_vertices) < 3:
            raise GeofenceError(f"max_vertices 至少要 3，收到 {max_vertices!r}。")

        self.buffer_radius_m: float = buffer_radius_m
        self.alt_margin_m: float = alt_margin_m
        self.method: str = method
        self.fence_action: int = fence_action
        self.fence_margin_m: float = fence_margin_m
        self.max_vertices: int = int(max_vertices)

    # ------------------------------------------------------------------
    # 投影轉換：WGS84 <-> 局部 AEQD 公尺平面
    # ------------------------------------------------------------------
    @staticmethod
    def _build_transformers(
        lat0: float, lon0: float
    ) -> Tuple[pyproj.Transformer, pyproj.Transformer]:
        """依任務中心建立 forward / inverse 投影。

        使用 Azimuthal Equidistant (AEQD)：以 (lat0, lon0) 為切點、
        任意方向距離保真（公尺），對 <200 km 任務誤差 <1 m，
        且不受 UTM zone 邊界影響。
        """
        wgs84 = pyproj.CRS.from_epsg(4326)
        aeqd = pyproj.CRS.from_proj4(
            f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} "
            "+datum=WGS84 +units=m +no_defs"
        )
        fwd = pyproj.Transformer.from_crs(wgs84, aeqd, always_xy=True)
        inv = pyproj.Transformer.from_crs(aeqd, wgs84, always_xy=True)
        return fwd, inv

    # ------------------------------------------------------------------
    # 掃描角：羅盤 → 平面數學角
    # ------------------------------------------------------------------
    @staticmethod
    def scan_angle_to_math_deg(scan_angle_deg: float) -> float:
        """羅盤方位（0=北、順時針）→ 平面數學角（0=+x=東、逆時針）。

        ``φ = 90 − θ``。

        ⚠⚠⚠ 這一行是整個功能唯一容易寫錯、而且**驗不出來**的地方。
        最常見的錯誤是直接把羅盤角當數學角用（``φ = θ``），兩者差 ``90 − 2θ``；
        而「在旋轉框架裡取軸對齊包絡」是 **90° 週期**的
        ⇒ θ 是 45 的倍數時兩種寫法給出**同一個矩形**。
        也就是說 0°/45°/90°/135° 全都驗不出這個錯，而 45° 正是浪費最多、
        最像好驗收題目的那一格。θ=30° 時錯誤寫法算出來的矩形甚至跟
        「完全沒轉」一模一樣，卻仍然回報 ``method='scan'``。
        鎖在 ``tests/test_fence_scan_angle.py::test_multiples_of_45_cannot_detect_the_missing_conversion``
        與同檔的對照組。
        """
        return 90.0 - float(scan_angle_deg)

    def _resolve_mode(
        self, scan_angle_deg: Optional[float]
    ) -> Tuple[str, Optional[float]]:
        """決定這一次要用哪種矩形，並擋掉矛盾組合。

        ⚠ 矛盾組合一律**拋例外**，不做仲裁：靜靜挑一個的話，畫面上是一個
        看起來很正常、但方向不是操作員以為的那個矩形。

        Returns
        -------
        (mode, angle) : ('aabb' | 'mrr' | 'scan', 角度或 None)
        """
        if scan_angle_deg is None:
            if self.method == "scan":
                raise GeofenceError(
                    "method='scan' 需要 scan_angle_deg，但沒有提供。"
                    "請在 build() 傳入掃描角（羅盤度），"
                    "或把 method 改回 'aabb' / 'mrr'。"
                )
            return self.method, None

        angle = float(scan_angle_deg)
        if not math.isfinite(angle):
            raise GeofenceError(
                f"掃描角必須是有限數值，收到 {scan_angle_deg!r}。"
                "上游算不出角度時請傳 None（維持軸對齊），不要傳 NaN/Inf。"
            )
        if self.method == "hull":
            raise GeofenceError(
                "method='hull' 與 scan_angle_deg 不能同時給：'hull' 直接貼合航線外形，"
                "沒有「方向」可言（而 'hull' 是預設）。真的要對齊掃描角的矩形，"
                "請明確指定 method='aabb' 再傳 scan_angle_deg。"
            )
        if self.method == "mrr":
            raise GeofenceError(
                "method='mrr' 與 scan_angle_deg 是兩種不同的目標，不能同時給："
                "'mrr' 求**面積最小**（方向跟著點集外形走），"
                "scan_angle_deg 求**方向對齊掃描線**。"
                "兩者在作業區外形與掃描方向不一致時會給出差很多的答案，"
                "請自己決定要哪一個。"
            )
        return "scan", angle

    # ------------------------------------------------------------------
    # 主建構方法：從航點集合 → MissionBundle
    # ------------------------------------------------------------------
    def build(
        self,
        waypoints: Sequence[Union[LatLon, LatLonAlt]],
        *,
        scan_angle_deg: Optional[float] = None,
        takeoff: Optional[Union[LatLon, LatLonAlt]] = None,
        targets: Optional[Sequence[Union[LatLon, LatLonAlt]]] = None,
        raw_result: Any = None,
    ) -> MissionBundle:
        """產生涵蓋所有點的矩形圍籬並封裝至 ``MissionBundle``。

        Parameters
        ----------
        waypoints : Sequence
            任務航點（含高度最佳，若未提供則預設 0）。
        scan_angle_deg : Optional[float]
            掃描線方向（**羅盤**度，0=北、順時針）。給了就把矩形轉到與掃描線
            平行／垂直（``method`` 變成 ``'scan'``）；``None`` 維持原本的軸對齊。

            ⚠ 對齊掃描角**不保證面積變小**。使用者要的是「方向跟航線一致」，
            不是「面積最小」（面積最小是 ``'mrr'``，而 MRR 幾乎垂直於航線）。
            手動掃描角與作業區外形打架時，圍籬會比軸對齊**大**
            —— 這是已知且刻意的取捨，鎖在
            ``tests/test_fence_scan_angle.py::test_manual_angle_fighting_the_area_can_enlarge_the_fence``。

            ⚠ 上游算不出角度時請傳 ``None``，不要傳 NaN/Inf（會拋 ``GeofenceError``）。
            本專案有**三套**掃描角慣例，換算見 ``core/geometry/polygon.py`` 尾端。
        takeoff : Optional
            起飛點。若與 waypoints 不同點需獨立確保納入。
        targets : Optional
            目標點（如打擊任務的終端瞄準點）。
        raw_result : Any
            原始生成函式的回傳值，一併夾帶至 bundle。

        Returns
        -------
        MissionBundle
        """
        # 0. 決定矩形種類（矛盾組合在這裡就吵，不要走到一半才發現）
        mode, angle = self._resolve_mode(scan_angle_deg)

        # 1. 彙整所有需被包絡的點
        all_pts: List[LatLonAlt] = []
        all_pts.extend(self._normalize(waypoints))
        if takeoff is not None:
            all_pts.append(self._normalize_single(takeoff))
        if targets is not None:
            all_pts.extend(self._normalize(targets))

        if len(all_pts) == 0:
            raise GeofenceError(
                "航點清單為空，無法計算圍籬。"
                "請先提供至少 1 個航點或起飛點。"
            )

        # 2. 建立投影 — 以所有點的幾何中心為切點（誤差最小）
        lat0 = sum(p[0] for p in all_pts) / len(all_pts)
        lon0 = sum(p[1] for p in all_pts) / len(all_pts)
        fwd, inv = self._build_transformers(lat0, lon0)

        # 3. 投影至 AEQD 平面（公尺）
        xy: List[Tuple[float, float]] = [
            fwd.transform(lon, lat) for lat, lon, _alt in all_pts
        ]

        # 4-5. 依方法取矩形並外擴 buffer_radius_m 公尺
        if mode == "hull":
            final_rect_planar = self._hull_polygon(xy)
        elif mode == "scan":
            # ⚠ buffer 必須做在**旋轉後的框架**裡：先轉回來再 buffer 的話，
            #   後面那一次 envelope 會把矩形收斂回軸對齊，方向整個掉光
            #   （鎖在 test_buffer_is_applied_in_the_rotated_frame）。
            final_rect_planar = self._scan_aligned_rect(xy, angle)
        else:
            rect_planar = self._rectangle_from_points(xy)
            #    cap_style=square / join_style=mitre — 確保 buffer 為直角矩形
            buffered = rect_planar.buffer(
                self.buffer_radius_m,
                cap_style="square",
                join_style="mitre",
            )
            # buffer 後可能是 Polygon 或 MultiPolygon；再取一次 envelope 收斂
            # 回 4 頂點的軸對齊矩形（若一開始是 mrr，此處改用 oriented bbox 以保旋轉）
            if mode == "mrr":
                final_rect_planar = buffered.minimum_rotated_rectangle
            else:
                final_rect_planar = buffered.envelope

        # 6. 反投影回 WGS84，保證 4 個獨立頂點
        ring_lonlat = list(final_rect_planar.exterior.coords)
        # shapely 閉合環最後一點 == 第一點 — 移除重覆
        if len(ring_lonlat) >= 2 and ring_lonlat[0] == ring_lonlat[-1]:
            ring_lonlat = ring_lonlat[:-1]
        if mode == "hull":
            if not 3 <= len(ring_lonlat) <= self.max_vertices:
                raise GeofenceError(
                    f"凸多邊形圍籬頂點數 {len(ring_lonlat)} 不在 3..{self.max_vertices}。")
        elif len(ring_lonlat) != 4:
            # 理論上 envelope/MRR 恆為 4 頂點；若不是則代表退化（如單點）
            # — 此時 buffer 後會是圓，envelope 仍會取 4 頂點；但保險起見檢查
            raise GeofenceError(
                f"矩形頂點數異常（得到 {len(ring_lonlat)} 個），"
                "通常導因於所有航點退化為同一點。"
                "請至少提供 2 個不同航點，或增加 buffer_radius_m。"
            )

        vertices_latlon: List[LatLon] = [
            (lat, lon)  # 統一 (lat, lon) 順序
            for lon, lat in (inv.transform(x, y) for x, y in ring_lonlat)
        ]

        # 7. 強制順時針（ArduPilot PolyFence inclusion 習慣；亦方便人類檢視）
        if self._ring_is_ccw(ring_lonlat):
            vertices_latlon = list(reversed(vertices_latlon))

        # 8. 高度上下限
        alts = [p[2] for p in all_pts]
        alt_max = max(alts) + self.alt_margin_m
        alt_min = max(min(alts) - self.alt_margin_m, 2.0)  # 至少 2 m 淨空

        geofence = Geofence(
            vertices=vertices_latlon,
            alt_min_m=alt_min,
            alt_max_m=alt_max,
            buffer_radius_m=self.buffer_radius_m,
            method=mode,          # ★ 這一次實際用的，不是建構子設定的
            inclusion=True,
        )

        # 9. ArduPilot FENCE 參數
        fence_params = self._build_fence_params(geofence, all_pts)

        return MissionBundle(
            waypoints=list(all_pts),
            geofence=geofence,
            fence_params=fence_params,
            raw_result=raw_result,
        )

    # ------------------------------------------------------------------
    # 矩形產生（在平面座標系上）
    # ------------------------------------------------------------------
    def _scan_aligned_rect(
        self, xy: List[Tuple[float, float]], scan_angle_deg: float
    ) -> Polygon:
        """對齊掃描角的矩形（**含 buffer**，回傳已轉回原框架的多邊形）。

        做法：把點集轉到「掃描線平行 x 軸」的框架 → 取軸對齊包絡 → 在**那個框架裡**
        外擴 buffer → 轉回來。

        ⚠ 三個一旦寫錯就會靜默失效的地方：
          1. ``φ = 90 − θ``（見 :meth:`scan_angle_to_math_deg`）；
          2. buffer 要在旋轉後的框架做，否則轉回來之後那一次 envelope 會把
             矩形收斂回軸對齊，方向白轉；
          3. 轉回來要用**同一個原點**（這裡一律用 (0, 0)）。
        """
        phi = self.scan_angle_to_math_deg(scan_angle_deg)
        mp = MultiPoint(xy)
        if mp.is_empty:
            raise GeofenceError(
                "無法從空的點集建立矩形。"
                "請確認航點已正確傳入。"
            )

        # 轉到掃描框架：−φ 讓掃描方向落在 +x
        rot = affinity.rotate(mp, -phi, origin=(0.0, 0.0))
        rect_rot = rot.envelope
        # 退化（單點／共線）→ 與 aabb 路徑同樣先撐開再取包絡
        if rect_rot.area < 1e-6:
            rect_rot = rot.buffer(1.0, cap_style="square").envelope

        buffered_rot = rect_rot.buffer(
            self.buffer_radius_m,
            cap_style="square",
            join_style="mitre",
        ).envelope          # 在掃描框架裡收斂回 4 頂點矩形

        return affinity.rotate(buffered_rot, phi, origin=(0.0, 0.0))

    def _hull_polygon(self, xy: List[Tuple[float, float]]) -> Polygon:
        """貼合航線的凸多邊形（**含 buffer**，平面座標）：頂點 ≤ max_vertices。

        做法：凸包 H → Douglas–Peucker 簡化成 S（容差 tol）→ S 外擴 ``buffer + tol``（mitre 角）。
        S 的頂點是 H 頂點的子集，H 的每一點離 S 不超過 tol ⇒ ``H ⊕ buffer ⊆ S ⊕ (buffer+tol)``，
        也就是**每一段航線離圍籬邊至少 buffer**；邊到航線的距離介於 buffer 與 buffer+tol 之間，
        所以四周是平均的。頂點太多就把 tol 放大再試。最後再用數值檢查一次包含關係。
        """
        mp = MultiPoint(xy)
        if mp.is_empty:
            raise GeofenceError("無法從空的點集建立圍籬。請確認航點已正確傳入。")
        hull = mp.convex_hull
        if hull.geom_type != "Polygon" or hull.area < 1e-6:
            # 單點或全共線：先撐成很窄的方塊再取凸包，後面照常外擴
            hull = mp.buffer(1.0, cap_style="square").convex_hull

        need = hull.buffer(self.buffer_radius_m - 0.05, resolution=32)   # 理想圍籬（圓角）略縮
        tol = 0.0
        for _ in range(60):
            simp = hull.simplify(tol, preserve_topology=True) if tol > 0 else hull
            poly = simp.buffer(self.buffer_radius_m + tol, join_style="mitre", mitre_limit=2.0)
            n = len(poly.exterior.coords) - 1
            if n <= self.max_vertices:
                if not poly.contains(need):
                    raise GeofenceError(
                        "凸多邊形圍籬沒有包住全部航線加緩衝（簡化出錯），拒絕產生不完整的圍籬。")
                return poly
            tol = 1.0 if tol == 0.0 else tol * 1.5
        raise GeofenceError(
            f"凸多邊形圍籬無法壓到 {self.max_vertices} 個頂點以內；請加大 max_vertices。")

    def _rectangle_from_points(
        self, xy: List[Tuple[float, float]]
    ) -> Polygon:
        """由平面點集產生矩形（單點/共線時退化成 buffer 後再包絡）。"""
        mp = MultiPoint(xy)

        # 單點或全共線 → 面積為 0，先 buffer 成方塊
        if mp.is_empty or not hasattr(mp, "envelope"):
            raise GeofenceError(
                "無法從空的點集建立矩形。"
                "請確認航點已正確傳入。"
            )

        if self.method == "mrr":
            rect = mp.minimum_rotated_rectangle
        else:
            rect = mp.envelope

        # 退化檢查：面積為 0 代表所有點共線或重合
        if rect.area < 1e-6:
            # 先以小半徑 buffer 撐開為方塊，再取 envelope
            rect = mp.buffer(1.0, cap_style="square").envelope

        return rect

    # ------------------------------------------------------------------
    # ArduPilot FENCE 參數生成
    # ------------------------------------------------------------------
    def _build_fence_params(
        self,
        fence: Geofence,
        points: Sequence[LatLonAlt],
    ) -> dict[str, Any]:
        """產出 ArduPilot FENCE_* 參數字典。

        參考：https://ardupilot.org/copter/docs/common-ap-fence.html
        """
        # FENCE_RADIUS 僅當 Circle bit (bit1=2) 啟用時生效。
        # ⚠⚠ ArduPilot 的圓形圍籬以 **Home** 為圓心，不是航點中心。舊算法用「航點中心 → 最遠航點」，
        #   起降坪在作業區中心時剛好差不多；2026-10-08 起起降場由使用者自選、常在作業區外 ⇒
        #   實測使用者那份：起降坪到最遠航點 2.81–2.92 km，舊半徑只有 1.66 km ⇒ 任務途中圓形越界、
        #   FENCE_ACTION=RTL，飛到一半自己返航。
        # 修法：Home 候選 = 貼地航點（VTOL 起降坪 alt≈0；含 build 的 takeoff），半徑 = 任一候選到
        #   任一航點的最遠距離 + buffer。沒有貼地航點時不知道 Home 在哪 ⇒ 用整個點集的直徑（保守，
        #   Home 落在任何航點上都不會誤觸；主要的包圍仍由多邊形圍籬負責）。
        lat0 = sum(p[0] for p in points) / len(points)
        lon0 = sum(p[1] for p in points) / len(points)
        fwd, _ = self._build_transformers(lat0, lon0)
        xy = [fwd.transform(p[1], p[0]) for p in points]
        homes = [q for q, p in zip(xy, points) if p[2] <= 1.0] or xy
        far = MultiPoint(xy).convex_hull
        far_pts = list(far.exterior.coords) if far.geom_type == "Polygon" else list(far.coords)
        max_r = max(math.hypot(hx - x, hy - y) for hx, hy in homes for x, y in far_pts)
        fence_radius = max_r + self.buffer_radius_m

        return {
            # 核心啟用（題目強制規格）
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": self.FENCE_TYPE_DEFAULT,   # 7 = MaxAlt+Circle+Polygon
            "FENCE_ACTION": self.fence_action,        # 1 = RTL/QRTL

            # 高度與距離限制
            "FENCE_ALT_MAX": round(fence.alt_max_m, 1),
            "FENCE_ALT_MIN": round(fence.alt_min_m, 1),
            "FENCE_MARGIN": round(self.fence_margin_m, 1),
            "FENCE_RADIUS": round(fence_radius, 1),

            # 多邊形選項
            "FENCE_TOTAL": len(fence.vertices),   # polygon 頂點數（矩形 4；'hull' 3..max_vertices）
            "FENCE_OPTIONS": 0,        # 0 = 不禁用位置外推等進階選項
        }

    # ------------------------------------------------------------------
    # MAVLink / QGC 格式匯出
    # ------------------------------------------------------------------
    @staticmethod
    def export_qgc_plan(
        bundle: MissionBundle,
        output: Union[str, Path],
    ) -> Path:
        """匯出為 QGroundControl ``.plan`` JSON 檔（含 geoFence 區段）。

        Parameters
        ----------
        bundle : MissionBundle
        output : path
            ``.plan`` 檔路徑。

        Returns
        -------
        pathlib.Path
            實際寫入之檔案路徑。
        """
        g = bundle.geofence

        plan = {
            "fileType": "Plan",
            "version": 1,
            "groundStation": "AeroPlan Studio",
            "geoFence": {
                "version": 2,
                "circles": [],
                "polygons": [
                    {
                        "inclusion": g.inclusion,
                        "version": 1,
                        "polygon": [
                            [lat, lon] for (lat, lon) in g.vertices
                        ],
                    }
                ],
            },
            "mission": {
                "version": 2,
                "firmwareType": 3,
                "vehicleType": 2,
                "cruiseSpeed": 15,
                "hoverSpeed": 5,
                "plannedHomePosition": [
                    bundle.waypoints[0][0],
                    bundle.waypoints[0][1],
                    bundle.waypoints[0][2],
                ],
                "items": [],
            },
            "rallyPoints": {"version": 2, "points": []},
        }

        out = Path(output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(plan, indent=2), encoding="utf-8")
        return out

    @staticmethod
    def export_mission_planner_fen(
        bundle: MissionBundle,
        output: Union[str, Path],
    ) -> Path:
        """匯出為 Mission Planner ``.fen`` 純文字檔。

        格式（第一行為 breach point，之後為 fence ring，最後閉合重覆頂點）::

            #saved by AeroPlan Studio (MIL-STD-1472H compliant)
            <breach_lat> <breach_lon>
            <v1_lat> <v1_lon>
            <v2_lat> <v2_lon>
            <v3_lat> <v3_lon>
            <v4_lat> <v4_lon>
            <v1_lat> <v1_lon>
        """
        g = bundle.geofence
        # breach point：違規時返航點，取航點第一點（起飛點慣例）
        breach_lat, breach_lon = bundle.waypoints[0][0], bundle.waypoints[0][1]

        lines: List[str] = [
            "#saved by AeroPlan Studio (MIL-STD-1472H compliant)",
            f"{breach_lat:.7f} {breach_lon:.7f}",
        ]
        ring = g.closed_ring()  # 頂點數 + 1（閉合）
        for lat, lon in ring:
            lines.append(f"{lat:.7f} {lon:.7f}")

        out = Path(output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return out

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_single(p: Union[LatLon, LatLonAlt]) -> LatLonAlt:
        """將 (lat,lon) 或 (lat,lon,alt) 統一為 (lat,lon,alt)。"""
        if len(p) == 2:
            return (float(p[0]), float(p[1]), 0.0)
        if len(p) >= 3:
            return (float(p[0]), float(p[1]), float(p[2]))
        raise GeofenceError(
            f"航點格式無效：{p!r}。請使用 (lat, lon) 或 (lat, lon, alt)。"
        )

    @classmethod
    def _normalize(
        cls, pts: Iterable[Union[LatLon, LatLonAlt]]
    ) -> List[LatLonAlt]:
        return [cls._normalize_single(p) for p in pts]

    @staticmethod
    def _ring_is_ccw(ring: Sequence[Tuple[float, float]]) -> bool:
        """以 shoelace 公式判斷平面環是否逆時針。"""
        s = 0.0
        n = len(ring)
        for i in range(n):
            x1, y1 = ring[i]
            x2, y2 = ring[(i + 1) % n]
            s += (x2 - x1) * (y2 + y1)
        return s < 0.0  # Shoelace 正負與座標系 y 方向相關；pyproj 為東北為正


# ═════════════════════════════════════════════════════════════════════
#  @enforce_geofence — 強制攔截裝飾器
# ═════════════════════════════════════════════════════════════════════
def enforce_geofence(
    *,
    buffer_radius_m: float = 30.0,
    alt_margin_m: float = 20.0,
    method: str = "hull",
    fence_action: int = 1,
    coord_extractor: Optional[Callable[[Any], Sequence[LatLonAlt]]] = None,
) -> Callable[[Callable[..., T]], Callable[..., MissionBundle]]:
    """裝飾器：強制攔截任何路徑生成函式，自動附加矩形圍籬。

    使用方式
    --------
    ::

        @enforce_geofence(buffer_radius_m=50)
        def plan_coverage(area_corners, altitude):
            # 原本回傳 List[(lat, lon, alt)]
            return [...]

        bundle = plan_coverage(corners, 80)   # 回傳 MissionBundle
        bundle.waypoints       # 原航點
        bundle.geofence        # 自動算出的矩形圍籬
        bundle.fence_params    # ArduPilot FENCE_* 字典

    Parameters
    ----------
    buffer_radius_m, alt_margin_m, method, fence_action
        傳遞給 :class:`GeofenceConstraintManager`。
    coord_extractor : callable, optional
        若被裝飾函式的回傳值不是直接的座標串列（例如是
        ``MissionPlanResult`` 之類的 dataclass），提供此函式以萃取出
        座標。預設假設回傳值即為可迭代的 (lat, lon[, alt]) 序列。

    Notes
    -----
    裝飾器**強制不可關閉**（遵循題目「圍籬不能是選配」規範）。
    即便原函式拋例外，亦會傳遞原例外給上層（不壓制）。
    """

    def decorator(fn: Callable[..., T]) -> Callable[..., MissionBundle]:
        @wraps(fn)
        def wrapper(*args: Any, **kwargs: Any) -> MissionBundle:
            raw = fn(*args, **kwargs)

            # 萃取座標
            if coord_extractor is not None:
                coords = list(coord_extractor(raw))
            else:
                coords = _default_extract(raw)

            if not coords:
                raise GeofenceError(
                    f"函式 {fn.__qualname__} 未回傳任何可識別的座標。"
                    "請確認回傳值為 (lat, lon[, alt]) 的序列，"
                    "或於 @enforce_geofence 提供 coord_extractor。"
                )

            mgr = GeofenceConstraintManager(
                buffer_radius_m=buffer_radius_m,
                alt_margin_m=alt_margin_m,
                method=method,
                fence_action=fence_action,
            )
            return mgr.build(coords, raw_result=raw)

        # 標註讓上層可以識別已受裝飾
        wrapper.__enforced_geofence__ = True  # type: ignore[attr-defined]
        return wrapper

    return decorator


def _default_extract(raw: Any) -> List[LatLonAlt]:
    """預設座標萃取器：嘗試多種常見回傳型態。"""
    if raw is None:
        return []
    # dict with 'waypoints' key
    if isinstance(raw, dict):
        for key in ("waypoints", "points", "path", "route"):
            if key in raw:
                return [GeofenceConstraintManager._normalize_single(p)
                        for p in raw[key]]
        return []
    # iterable of tuples / Waypoint-like objects
    try:
        out: List[LatLonAlt] = []
        for item in raw:  # type: ignore[union-attr]
            # 若是具 lat/lon 屬性的物件
            if hasattr(item, "lat") and hasattr(item, "lon"):
                alt = getattr(item, "alt", getattr(item, "altitude", 0.0))
                out.append((float(item.lat), float(item.lon), float(alt)))
            else:
                out.append(
                    GeofenceConstraintManager._normalize_single(item)
                )
        return out
    except TypeError:
        return []


# ═════════════════════════════════════════════════════════════════════
#  Main Block — 10 個不規則航點示範
# ═════════════════════════════════════════════════════════════════════
def _demo() -> None:
    """示範：10 個隨機不規則航點 → 自動矩形圍籬 → 匯出 QGC/MP。"""
    import random
    import tempfile

    # 以新竹縣虎尾地區 (≈北緯 23.7°, 東經 120.43°) 作為測試中心
    # 模擬戰術偵察任務 — 在 2 km × 2 km 範圍內散佈 10 個不規則航點
    random.seed(20260424)  # 固定隨機種子以確保示範可重現
    CENTER_LAT, CENTER_LON = 23.7050, 120.4320
    BASE_ALT = 80.0

    waypoints: List[LatLonAlt] = []
    for _ in range(10):
        # 以約 0.01° ≈ 1.1 km 的尺度隨機散佈
        dlat = (random.random() - 0.5) * 0.02
        dlon = (random.random() - 0.5) * 0.02
        dalt = (random.random() - 0.5) * 40.0  # ±20 m 高度變化
        waypoints.append(
            (CENTER_LAT + dlat, CENTER_LON + dlon, BASE_ALT + dalt)
        )

    takeoff = (CENTER_LAT, CENTER_LON, 0.0)
    targets = [(CENTER_LAT + 0.005, CENTER_LON + 0.007, 100.0)]

    # 建立管理器 — 50 m 緩衝、20 m 高度容裕、軸對齊矩形
    mgr = GeofenceConstraintManager(
        buffer_radius_m=50.0,
        alt_margin_m=20.0,
        method="aabb",
        fence_action=1,  # RTL/QRTL
    )

    bundle = mgr.build(
        waypoints=waypoints,
        takeoff=takeoff,
        targets=targets,
        raw_result={"mission_name": "demo-recon"},
    )

    # ── 列印結果 ────────────────────────────────────────────
    print("═" * 64)
    print("  AeroPlan Studio  —  Geofence Constraint Manager Demo")
    print("═" * 64)
    print(bundle.summary())
    print()

    print("▶ 10 個不規則航點（lat, lon, alt m）:")
    for i, (lat, lon, alt) in enumerate(waypoints, 1):
        print(f"  WP-{i:02d}  {lat:10.6f}  {lon:11.6f}  {alt:6.1f} m")
    print(f"  TAKEOFF {takeoff[0]:10.6f}  {takeoff[1]:11.6f}  "
          f"{takeoff[2]:6.1f} m")
    print(f"  TARGET  {targets[0][0]:10.6f}  {targets[0][1]:11.6f}  "
          f"{targets[0][2]:6.1f} m")
    print()

    print("▶ 矩形圍籬 4 頂點（順時針，WGS84）:")
    for i, (lat, lon) in enumerate(bundle.geofence.vertices, 1):
        print(f"  V{i}  {lat:10.6f}  {lon:11.6f}")
    print(f"  方法     : {bundle.geofence.method.upper()}")
    print(f"  ALT_MIN  : {bundle.geofence.alt_min_m:.1f} m")
    print(f"  ALT_MAX  : {bundle.geofence.alt_max_m:.1f} m")
    print(f"  BUFFER   : {bundle.geofence.buffer_radius_m:.1f} m")
    print()

    print("▶ ArduPilot FENCE 參數:")
    for k, v in bundle.fence_params.items():
        print(f"  {k:<18s} = {v}")
    print()

    # ── 匯出格式示範 ─────────────────────────────────────────
    with tempfile.TemporaryDirectory(prefix="aeroplan_fence_") as tmp:
        tmp_path = Path(tmp)
        plan_path = mgr.export_qgc_plan(bundle, tmp_path / "demo.plan")
        fen_path = mgr.export_mission_planner_fen(bundle, tmp_path / "demo.fen")

        print(f"▶ QGC .plan 匯出至: {plan_path}")
        print("  前 12 行預覽:")
        for line in plan_path.read_text(encoding="utf-8").splitlines()[:12]:
            print(f"    {line}")
        print()
        print(f"▶ Mission Planner .fen 匯出至: {fen_path}")
        print("  完整內容:")
        for line in fen_path.read_text(encoding="utf-8").splitlines():
            print(f"    {line}")
    print()

    # ── 裝飾器示範 ─────────────────────────────────────────
    print("▶ @enforce_geofence 裝飾器示範:")

    @enforce_geofence(buffer_radius_m=30.0, alt_margin_m=15.0)
    def plan_coverage(area_center: LatLon, altitude: float) -> List[LatLonAlt]:
        """模擬的覆蓋規劃函式：回傳一圈繞圓航點。"""
        return [
            (
                area_center[0] + 0.003 * math.cos(math.radians(a)),
                area_center[1] + 0.003 * math.sin(math.radians(a)),
                altitude,
            )
            for a in range(0, 360, 45)
        ]

    decorated_result = plan_coverage((CENTER_LAT, CENTER_LON), 75.0)
    assert isinstance(decorated_result, MissionBundle), \
        "裝飾器應回傳 MissionBundle"
    print(f"  {decorated_result.summary()}")
    print(f"  裝飾後圍籬 {decorated_result.geofence.vertex_count()} 頂點:")
    for i, (lat, lon) in enumerate(decorated_result.geofence.vertices, 1):
        print(f"    V{i}  {lat:10.6f}  {lon:11.6f}")
    print()

    # ── 最終斷言 ─────────────────────────────────────────
    assert 3 <= bundle.geofence.vertex_count() <= 12, "頂點數必須在 3..12"
    assert bundle.fence_params["FENCE_ENABLE"] == 1
    assert bundle.fence_params["FENCE_TYPE"] == 7
    assert bundle.fence_params["FENCE_ACTION"] == 1
    print("✓ All assertions passed — Geofence is mandatory, minimal (4-vtx), "
          "and ArduPilot-compatible.")


if __name__ == "__main__":
    _demo()


__all__ = [
    "GeofenceConstraintManager",
    "Geofence",
    "MissionBundle",
    "GeofenceError",
    "enforce_geofence",
]
