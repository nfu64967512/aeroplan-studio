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
from dataclasses import dataclass, field, asdict
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
        產生方式：'aabb'（軸對齊）或 'mrr'（最小旋轉矩形）。
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
    """強制矩形包絡線電子圍籬管理器。

    Parameters
    ----------
    buffer_radius_m : float
        矩形外擴的緩衝半徑（公尺）。預設 30 m — 對多旋翼保守；
        固定翼建議 ≥ 100 m（與轉彎半徑匹配）。
    alt_margin_m : float
        高度上下限的安全容裕。FENCE_ALT_MAX = 航點最高 + margin；
        FENCE_ALT_MIN = max(航點最低 - margin, 2.0) — 至少保留 2 m 地面淨空。
    method : {'aabb', 'mrr'}
        'aabb' — shapely ``.envelope``（軸對齊矩形，預設；ArduPilot 操作員
                最直觀、RTL 行為可預期）。
        'mrr'  — ``.minimum_rotated_rectangle``（最小面積旋轉矩形；若航線
                呈對角狹長帶狀則節省空域）。
    fence_action : int
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
        method: str = "aabb",
        fence_action: int = 1,
        fence_margin_m: float = 5.0,
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
        if method not in ("aabb", "mrr"):
            raise GeofenceError(
                f"未知的矩形產生方式 '{method}'。"
                "請改為 'aabb'（軸對齊）或 'mrr'（最小旋轉矩形）。"
            )

        self.buffer_radius_m: float = buffer_radius_m
        self.alt_margin_m: float = alt_margin_m
        self.method: str = method
        self.fence_action: int = fence_action
        self.fence_margin_m: float = fence_margin_m

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
    # 主建構方法：從航點集合 → MissionBundle
    # ------------------------------------------------------------------
    def build(
        self,
        waypoints: Sequence[Union[LatLon, LatLonAlt]],
        *,
        takeoff: Optional[Union[LatLon, LatLonAlt]] = None,
        targets: Optional[Sequence[Union[LatLon, LatLonAlt]]] = None,
        raw_result: Any = None,
    ) -> MissionBundle:
        """產生涵蓋所有點的矩形圍籬並封裝至 ``MissionBundle``。

        Parameters
        ----------
        waypoints : Sequence
            任務航點（含高度最佳，若未提供則預設 0）。
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

        # 4. 依方法取矩形
        rect_planar = self._rectangle_from_points(xy)

        # 5. 外擴 buffer_radius_m 公尺
        #    cap_style=square / join_style=mitre — 確保 buffer 為直角矩形
        buffered = rect_planar.buffer(
            self.buffer_radius_m,
            cap_style="square",
            join_style="mitre",
        )
        # buffer 後可能是 Polygon 或 MultiPolygon；再取一次 envelope 收斂
        # 回 4 頂點的軸對齊矩形（若一開始是 mrr，此處改用 oriented bbox 以保旋轉）
        if self.method == "mrr":
            final_rect_planar = buffered.minimum_rotated_rectangle
        else:
            final_rect_planar = buffered.envelope

        # 6. 反投影回 WGS84，保證 4 個獨立頂點
        ring_lonlat = list(final_rect_planar.exterior.coords)
        # shapely 閉合環最後一點 == 第一點 — 移除重覆
        if len(ring_lonlat) >= 2 and ring_lonlat[0] == ring_lonlat[-1]:
            ring_lonlat = ring_lonlat[:-1]
        if len(ring_lonlat) != 4:
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
            method=self.method,
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
        # 計算航點中心 → 最遠航點的距離作為圓心半徑；
        # + buffer_radius_m 作為安全裕度。
        lat0 = sum(p[0] for p in points) / len(points)
        lon0 = sum(p[1] for p in points) / len(points)
        fwd, _ = self._build_transformers(lat0, lon0)
        max_r = max(
            math.hypot(*fwd.transform(p[1], p[0])) for p in points
        )
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
            "FENCE_TOTAL": 4,          # polygon 頂點數（本模組恆為 4）
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
        ring = g.closed_ring()  # 5 座標
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
    method: str = "aabb",
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
    print(f"  裝飾後圍籬 4 頂點:")
    for i, (lat, lon) in enumerate(decorated_result.geofence.vertices, 1):
        print(f"    V{i}  {lat:10.6f}  {lon:11.6f}")
    print()

    # ── 最終斷言 ─────────────────────────────────────────
    assert bundle.geofence.vertex_count() == 4, "頂點數必須為 4"
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
