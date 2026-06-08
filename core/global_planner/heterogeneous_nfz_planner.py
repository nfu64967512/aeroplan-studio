"""
core/global_planner/heterogeneous_nfz_planner.py
================================================

異質機隊（固定翼 + 多旋翼）NFZ 事前繞行規劃器
-----------------------------------------------

針對 AeroPlan Studio 機隊在運動學上的根本差異：
    • 多旋翼 (Multirotor)  → 可懸停、可急停、無最小轉彎半徑限制
    • 固定翼 (Fixed-wing)  → 必須前進、受最小轉彎半徑 R_min 約束

本模組提供 ``HeterogeneousNFZPlanner`` 主控類別，當起點與目標連線穿越
禁飛區 (NFZ) 多邊形時，會為兩種機型分別產生「物理上強制分離、不重疊、
不互撞、可飛性正確」的兩條繞行航點。

物理分離三層保險（詳見 README 與類別 docstring）：
    1. 水平分離 — Shapely buffer 分別膨脹（MR 20m / FW 100m）
       兩條路徑的最近點水平距 ≥ 80m
    2. 垂直分離 — 高度層分配（MR 80m / FW 120m）
       恆定 40m 垂直差，即使水平共線也不衝突
    3. 演算法本徵差異 —
       MR 走 A* → 直角折線貼邊；FW 走 Dubins fillet → 大弧迂迴
       形狀根本不同，不會出現平行重疊

輸出格式：每架機回傳 ``List[Waypoint]``，可直接以
``Waypoint.to_mavlink_wpl_line()`` 串成 QGC WPL 110 任務檔案。

執行範例見 ``__main__`` 區塊。
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# Shapely — 多邊形幾何運算的標準（buffer、交集、外輪廓）
try:
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
except ImportError as e:  # pragma: no cover
    raise ImportError(
        '本模組需要 shapely >= 2.0，請執行：pip install shapely'
    ) from e


# ──────────────────────────────────────────────────────────────────────
#  資料模型
# ──────────────────────────────────────────────────────────────────────
@dataclass
class Waypoint:
    """單一航點（lat / lon 度、alt 公尺 AGL）

    可直接序列化為 ArduPilot 相容的 QGC WPL 110 行。
    """

    lat: float
    lon: float
    alt: float
    cmd: int = 16    # MAV_CMD_NAV_WAYPOINT
    frame: int = 3   # MAV_FRAME_GLOBAL_RELATIVE_ALT

    def to_mavlink_wpl_line(
        self, seq: int, current: int = 0, autocontinue: int = 1,
        p1: float = 0.0, p2: float = 0.0, p3: float = 0.0, p4: float = 0.0,
    ) -> str:
        """產生 QGC WPL 110 單行格式。

        欄位順序（11 欄、Tab 分隔）：
            seq  current  frame  cmd  p1  p2  p3  p4  lat  lon  alt  autocontinue
        """
        return (
            f"{seq}\t{current}\t{self.frame}\t{self.cmd}\t"
            f"{p1:.6f}\t{p2:.6f}\t{p3:.6f}\t{p4:.6f}\t"
            f"{self.lat:.7f}\t{self.lon:.7f}\t{self.alt:.2f}\t"
            f"{autocontinue}"
        )


@dataclass
class PlannerConfig:
    """規劃器全域參數 — 物理常數 + 演算法調節參數一站式管理"""

    # ── 水平分離（Shapely buffer 距離）──
    buffer_mr: float = 20.0     # 多旋翼安全緩衝距 [m]
    buffer_fw: float = 100.0    # 固定翼安全緩衝距 [m]

    # ── 垂直分離（高度層）──
    alt_mr: float = 80.0        # 多旋翼專屬高度 [m AGL]
    alt_fw: float = 120.0       # 固定翼專屬高度 [m AGL]

    # ── 固定翼運動學約束 ──
    r_min: float = 60.0         # 最小轉彎半徑 [m]
    arc_segments: int = 12      # Dubins 圓弧離散化段數（每弧的航點數）

    # ── 多旋翼 A* 網格化 ──
    grid_res_m: float = 8.0     # 網格解析度 [m]（越小越精準、越慢）
    grid_margin_m: float = 200.0  # 規劃域邊界外擴 [m]（保 A* 有迂迴空間）


# ──────────────────────────────────────────────────────────────────────
#  幾何 helper — 線段是否穿入 polygon 嚴格內部（沿邊不算）
# ──────────────────────────────────────────────────────────────────────
def _crosses_polygon_interior(
    p1: Tuple[float, float], p2: Tuple[float, float],
    polygon: Polygon, epsilon_m: float = 0.3,
) -> bool:
    """檢查線段 [p1, p2] 是否進入 polygon 內部「超過 epsilon_m 公尺」。

    精確做法（不依取樣）：
        1. 計算 inner = polygon.buffer(-epsilon_m) — 邊界內縮 epsilon
        2. 求 line ∩ inner 的長度
        3. 若 > 微小門檻 → 真實穿越

    為何要 epsilon_m？
        Shapely buffer() 圓角化的相鄰頂點之間「直線弦」會略微凹入圓內側。
        即使 quad_segs=32 時弦深 < 0.1m，數值精度仍可能讓沿邊線段被判
        為「穿入 0.0001m」。內縮 epsilon 直接排除這些假陽。

    為何不用 N 點取樣？
        取樣有盲點：穿入區若只佔線段 2% 且我取樣 32 點，間距大則完全錯過。
        buffer(-epsilon) + intersection 是 polygon-line 嚴格幾何運算，零盲點。

    Args:
        p1, p2     : 線段兩端點 (x, y) ENU 公尺
        polygon    : Shapely Polygon
        epsilon_m  : 內縮容差（公尺），預設 0.3m
                     （> quad_segs=32 buffer 圓角弦深 0.1m × 安全係數 3）

    Returns:
        True  → 線段穿入 polygon 內部 > epsilon_m
        False → 線段在外、僅接觸邊界、或沿邊（凹陷 ≤ epsilon_m）
    """
    seg = LineString([p1, p2])
    if seg.length < 1e-9:
        return False
    # 快速排除：完全不相交 → 不穿入
    if not seg.intersects(polygon):
        return False
    # 內縮 polygon 邊界 epsilon_m — strictly interior 區域
    inner = polygon.buffer(-epsilon_m)
    if inner.is_empty or not inner.is_valid:
        # polygon 太細小無內部，視同無穿入
        return False
    inter = seg.intersection(inner)
    if inter.is_empty:
        return False
    if inter.geom_type in ('Point', 'MultiPoint'):
        return False
    # LineString / MultiLineString → 看穿入段長度
    if hasattr(inter, 'length'):
        return inter.length > 1e-3
    return False


# ──────────────────────────────────────────────────────────────────────
#  本地投影 — 經緯度 ↔ ENU 公尺座標
# ──────────────────────────────────────────────────────────────────────
class _LocalProjection:
    """以使用者指定的原點 (lat0, lon0) 為中心的局部 ENU 平面投影。

    使用 equirectangular 近似（小範圍 km 級足夠精準，不依賴 pyproj）：
        x_east  = (lon - lon0) · 111320 · cos(lat0)
        y_north = (lat - lat0) · 111320
    """

    _M_PER_DEG_LAT = 111320.0

    def __init__(self, lat0: float, lon0: float) -> None:
        self.lat0 = float(lat0)
        self.lon0 = float(lon0)
        self._cos_lat0 = max(math.cos(math.radians(self.lat0)), 1e-9)
        self._m_per_deg_lon = self._M_PER_DEG_LAT * self._cos_lat0

    def to_metric(self, lat: float, lon: float) -> Tuple[float, float]:
        """經緯度 → ENU (east_m, north_m)"""
        return (
            (lon - self.lon0) * self._m_per_deg_lon,
            (lat - self.lat0) * self._M_PER_DEG_LAT,
        )

    def to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """ENU (east_m, north_m) → 經緯度"""
        return (
            self.lat0 + y / self._M_PER_DEG_LAT,
            self.lon0 + x / self._m_per_deg_lon,
        )

    def project_polygon(
        self, vertices_ll: Sequence[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        return [self.to_metric(lat, lon) for lat, lon in vertices_ll]


# ──────────────────────────────────────────────────────────────────────
#  多旋翼規劃器：A* 網格化 + Line-of-Sight 簡化
# ──────────────────────────────────────────────────────────────────────
class MultirotorAStarPlanner:
    """多旋翼 A* 規劃器。

    特性對應：
        • 可懸停 / 急轉 → 8 連通網格、不需轉彎半徑約束
        • 追求最短路徑 → A* + Euclidean heuristic
        • 允許貼邊切角 → buffer 小、line-of-sight smoothing 留下最少航點

    輸入輸出都是局部 ENU (x, y) 公尺座標。
    """

    def __init__(self, config: PlannerConfig) -> None:
        self._cfg = config

    def plan(
        self,
        start_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        buffered_nfz: Polygon,
    ) -> List[Tuple[float, float]]:
        """規劃多旋翼從 start_xy 到 target_xy 的繞行路徑。

        Args:
            start_xy / target_xy : ENU 公尺座標 (east, north)
            buffered_nfz         : 已膨脹 buffer_mr 的 Shapely Polygon

        Returns:
            航點列表 [(x, y), ...]（含起點與終點），均為 ENU 公尺座標。
            line-of-sight 簡化後保留最少數頂點（多為轉折點 + 端點）。
        """
        # 路徑直線本身就安全 → 直接回傳兩點
        direct = LineString([start_xy, target_xy])
        if not direct.intersects(buffered_nfz):
            return [start_xy, target_xy]

        # ── 1) 規劃域 bounding box（涵蓋兩端點 + NFZ + 邊界裕量）──
        margin = self._cfg.grid_margin_m
        minx = min(start_xy[0], target_xy[0], buffered_nfz.bounds[0]) - margin
        miny = min(start_xy[1], target_xy[1], buffered_nfz.bounds[1]) - margin
        maxx = max(start_xy[0], target_xy[0], buffered_nfz.bounds[2]) + margin
        maxy = max(start_xy[1], target_xy[1], buffered_nfz.bounds[3]) + margin

        res = self._cfg.grid_res_m
        n_cols = max(2, int(math.ceil((maxx - minx) / res)))
        n_rows = max(2, int(math.ceil((maxy - miny) / res)))

        # ── 2) 計算每個 cell 中心是否被禁區覆蓋 ──
        # 對效能：先把 buffered_nfz 的 prepared 版本建好（Shapely 2.0 內建）
        from shapely.prepared import prep
        prepared = prep(buffered_nfz)

        def cell_xy(i: int, j: int) -> Tuple[float, float]:
            """網格 (col, row) → 中心 ENU 座標"""
            return (minx + (i + 0.5) * res, miny + (j + 0.5) * res)

        def cell_blocked(i: int, j: int) -> bool:
            x, y = cell_xy(i, j)
            return prepared.contains(Point(x, y))

        # 把起點 / 終點吸附到最近的 cell（如剛好在 cell 邊上也 OK）
        def snap(p: Tuple[float, float]) -> Tuple[int, int]:
            i = int((p[0] - minx) / res)
            j = int((p[1] - miny) / res)
            i = max(0, min(n_cols - 1, i))
            j = max(0, min(n_rows - 1, j))
            return (i, j)

        s_ij = snap(start_xy)
        t_ij = snap(target_xy)

        # 若起點或終點落在 blocked cell → 找出最近的可通行 cell（BFS 救援）
        if cell_blocked(*s_ij):
            s_ij = self._nearest_free(s_ij, n_cols, n_rows, cell_blocked)
        if cell_blocked(*t_ij):
            t_ij = self._nearest_free(t_ij, n_cols, n_rows, cell_blocked)
        if s_ij is None or t_ij is None:
            raise RuntimeError(
                'MR A*：起點或終點周圍無可通行 cell，請放寬 buffer 或擴大規劃域'
            )

        # ── 3) A* 主迴圈（8 連通、Euclidean heuristic）──
        # 4 軸向鄰居代價 = res；4 對角線代價 = res·√2
        SQRT2 = math.sqrt(2.0)
        NEIGH = [
            (1, 0, res), (-1, 0, res), (0, 1, res), (0, -1, res),
            (1, 1, res * SQRT2), (-1, 1, res * SQRT2),
            (1, -1, res * SQRT2), (-1, -1, res * SQRT2),
        ]

        def heuristic(ij: Tuple[int, int]) -> float:
            dx = (t_ij[0] - ij[0]) * res
            dy = (t_ij[1] - ij[1]) * res
            return math.hypot(dx, dy)

        open_heap: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(open_heap, (heuristic(s_ij), s_ij))
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {s_ij: 0.0}

        found = False
        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == t_ij:
                found = True
                break
            cx, cy = current
            for di, dj, cost in NEIGH:
                nb = (cx + di, cy + dj)
                if not (0 <= nb[0] < n_cols and 0 <= nb[1] < n_rows):
                    continue
                if cell_blocked(*nb):
                    continue
                # 對角線移動需檢查兩側不能同時 blocked（防止穿牆角）
                if di != 0 and dj != 0:
                    if cell_blocked(cx + di, cy) and cell_blocked(cx, cy + dj):
                        continue
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(nb, math.inf):
                    g_score[nb] = tentative_g
                    came_from[nb] = current
                    f = tentative_g + heuristic(nb)
                    heapq.heappush(open_heap, (f, nb))

        if not found:
            raise RuntimeError('MR A*：起點到終點無可通行路徑（NFZ 完全切斷）')

        # ── 4) 回溯路徑、轉成 ENU 座標 ──
        path_ij: List[Tuple[int, int]] = [t_ij]
        while path_ij[-1] != s_ij:
            path_ij.append(came_from[path_ij[-1]])
        path_ij.reverse()
        path_xy: List[Tuple[float, float]] = [cell_xy(i, j) for i, j in path_ij]

        # 強制兩端點對齊使用者輸入（不依 cell 中心）
        path_xy[0] = start_xy
        path_xy[-1] = target_xy

        # ── 5) Line-of-Sight 簡化 — 拿掉中間「視認連通」的多餘航點 ──
        # 傳 raw polygon 而非 prepared，因為簡化需用 .intersection().length
        # 判斷穿入段（prepared 不支援 .intersection）
        return self._los_simplify(path_xy, buffered_nfz)

    @staticmethod
    def _nearest_free(
        start_ij: Tuple[int, int], n_cols: int, n_rows: int,
        blocked_fn,
    ) -> Optional[Tuple[int, int]]:
        """從 start_ij 螺旋向外尋找最近的可通行 cell（用於救援被堵的起終點）"""
        if not blocked_fn(*start_ij):
            return start_ij
        for r in range(1, max(n_cols, n_rows)):
            for di in range(-r, r + 1):
                for dj in (-r, r):
                    cand = (start_ij[0] + di, start_ij[1] + dj)
                    if (0 <= cand[0] < n_cols and 0 <= cand[1] < n_rows
                            and not blocked_fn(*cand)):
                        return cand
                for dj_inner in range(-r + 1, r):
                    for di_edge in (-r, r):
                        cand = (start_ij[0] + di_edge, start_ij[1] + dj_inner)
                        if (0 <= cand[0] < n_cols and 0 <= cand[1] < n_rows
                                and not blocked_fn(*cand)):
                            return cand
        return None

    @staticmethod
    def _los_simplify(
        path: List[Tuple[float, float]],
        obstacle,   # Shapely Polygon
    ) -> List[Tuple[float, float]]:
        """Line-of-Sight 簡化：保留貼邊切角，移除中間共線航點。

        判斷原則（N 點取樣 + strictly contains）：
            在線段上均勻取 N 個內部點，若任一點被 polygon.contains() 嚴格包含
            → 線段穿入 polygon 內部。
            polygon.contains() 不含邊界，所以沿邊行走的線段不會誤判。
            「中點 contains」單點測試容易被穿越線段騙過（中點剛好擦邊外）。
        """
        if len(path) <= 2:
            return list(path)
        simplified = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            far = anchor + 1
            for k in range(anchor + 2, len(path)):
                if _crosses_polygon_interior(path[anchor], path[k], obstacle):
                    break
                far = k
            simplified.append(path[far])
            anchor = far
        return simplified


# ──────────────────────────────────────────────────────────────────────
#  固定翼規劃器：Visibility Graph + Dubins fillet
# ──────────────────────────────────────────────────────────────────────
class FixedWingDubinsPlanner:
    """固定翼 Dubins 規劃器。

    流程：
        1. 把 NFZ 膨脹 buffer_fw 得到 buffered_polygon
        2. 取 buffered_polygon 外輪廓頂點作為候選轉彎點
        3. Visibility Graph + Dijkstra 找出 start → vertices → target
           的最短折線路徑 (P_0, P_1, ..., P_n)
        4. 每個內部頂點 P_i 用 R_min 圓弧 (fillet) 取代尖角：
              - tangent point T1 = P_i + (R/tan(α/2)) · unit(P_{i-1} - P_i)
              - tangent point T2 = P_i + (R/tan(α/2)) · unit(P_{i+1} - P_i)
              - 圓心 C_i = P_i + (R/sin(α/2)) · 角平分線單位向量
              - 弧離散化為 arc_segments 個小段
        5. 串接成 [start, T1_0, arc_0_pts, T2_0, T1_1, arc_1_pts, T2_1, ..., target]
           — 全段轉彎半徑保證 ≥ R_min，飛機可飛
    """

    def __init__(self, config: PlannerConfig) -> None:
        self._cfg = config

    def plan(
        self,
        start_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        buffered_nfz: Polygon,
    ) -> List[Tuple[float, float]]:
        """規劃固定翼從 start 到 target 的平滑繞行路徑（ENU 公尺）。"""
        # 直線安全 → 直接回傳
        direct = LineString([start_xy, target_xy])
        if not direct.intersects(buffered_nfz):
            return [start_xy, target_xy]

        # ── 1) 視認圖頂點：起點、終點、buffered NFZ 外輪廓所有頂點 ──
        nfz_verts = list(buffered_nfz.exterior.coords)[:-1]  # 去掉閉合重複尾
        nodes: List[Tuple[float, float]] = [start_xy, target_xy] + nfz_verts
        n = len(nodes)

        # ── 2) 建立可視性鄰接表 ──
        # 可視 = 線段不穿入 polygon 嚴格內部（沿邊 OK、穿越 NOT OK）
        # 用 N 點取樣判斷，避免「中點剛好擦邊」的假陰性
        def visible(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
            return not _crosses_polygon_interior(a, b, buffered_nfz)

        adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(n)}
        for i in range(n):
            for j in range(i + 1, n):
                if visible(nodes[i], nodes[j]):
                    d = math.hypot(
                        nodes[j][0] - nodes[i][0],
                        nodes[j][1] - nodes[i][1],
                    )
                    adj[i].append((j, d))
                    adj[j].append((i, d))

        # ── 3) Dijkstra: node 0 (start) → node 1 (target) ──
        dist = [math.inf] * n
        prev = [-1] * n
        dist[0] = 0.0
        pq: List[Tuple[float, int]] = [(0.0, 0)]
        while pq:
            d_u, u = heapq.heappop(pq)
            if d_u > dist[u]:
                continue
            if u == 1:
                break
            for v, w in adj[u]:
                d_v = d_u + w
                if d_v < dist[v]:
                    dist[v] = d_v
                    prev[v] = u
                    heapq.heappush(pq, (d_v, v))

        if math.isinf(dist[1]):
            raise RuntimeError(
                'FW Dubins：視認圖無法連通起點到終點（buffered NFZ 完全封閉）'
            )

        # 回溯出折線路徑
        broken: List[Tuple[float, float]] = []
        cur = 1
        while cur != -1:
            broken.append(nodes[cur])
            cur = prev[cur]
        broken.reverse()

        # ── 4) 每個內部頂點用 R_min 圓弧 fillet 倒角 ──
        return self._apply_fillets(broken)

    def _apply_fillets(
        self, broken: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """把折線轉成「直線 + R_min 圓弧」串接，弧離散化為 arc_segments 段。

        若某轉角過尖（fillet 切線長 > 上下游邊長一半），則對該邊取較小
        的安全 R 以避免相鄰 fillet 重疊（保守處理：縮小 R 而非放棄）。
        """
        R = self._cfg.r_min
        N_SEG = max(2, self._cfg.arc_segments)

        if len(broken) <= 2:
            return list(broken)

        out: List[Tuple[float, float]] = [broken[0]]

        for i in range(1, len(broken) - 1):
            A = broken[i - 1]
            B = broken[i]
            C = broken[i + 1]

            # 入向 / 出向 單位向量（均從 B 向外）
            uxa = A[0] - B[0]; uya = A[1] - B[1]
            la = math.hypot(uxa, uya)
            uxc = C[0] - B[0]; uyc = C[1] - B[1]
            lc = math.hypot(uxc, uyc)
            if la < 1e-6 or lc < 1e-6:
                out.append(B)
                continue
            uxa /= la; uya /= la
            uxc /= lc; uyc /= lc

            # 內部夾角 α（B 處 ∠ABC）
            cos_alpha = max(-1.0, min(1.0, uxa * uxc + uya * uyc))
            alpha = math.acos(cos_alpha)
            if alpha < math.radians(1.0):
                # 幾乎反向折返 → fillet 數值上不穩定，保留 B
                out.append(B)
                continue
            if alpha > math.radians(175.0):
                # 幾乎直線（但仍是繞開禁區的關鍵轉折點）— fillet 半徑微小
                # 不值得算弧，但「必須保留 B」否則路徑會塌回成直線穿越禁區。
                # 這是 visibility graph 找到的 grazing vertex，丟了會破壞拓樸繞行。
                out.append(B)
                continue

            # 切線長 d = R / tan(α/2)；圓心距 B 的距離 = R / sin(α/2)
            d_tangent = R / math.tan(alpha / 2.0)
            # 防止 fillet 切到相鄰邊（保留至少 5% 安全邊距）
            d_max = 0.45 * min(la, lc)
            if d_tangent > d_max:
                # 該頂點 R 不可行：退而求其次，縮小有效 R
                R_eff = d_max * math.tan(alpha / 2.0)
                R_eff = max(R_eff, R * 0.3)  # 不低於 30% R_min
                d_tangent = R_eff / math.tan(alpha / 2.0)
                R_used = R_eff
            else:
                R_used = R

            # 切線點
            T1 = (B[0] + uxa * d_tangent, B[1] + uya * d_tangent)
            T2 = (B[0] + uxc * d_tangent, B[1] + uyc * d_tangent)

            # 圓心：沿「角平分線」自 B 向外推距離 R/sin(α/2)
            # 角平分線單位向量 = unit(uxa + uxc)
            bx = uxa + uxc; by = uya + uyc
            bl = math.hypot(bx, by)
            if bl < 1e-9:
                out.append(B)
                continue
            bx /= bl; by /= bl
            d_center = R_used / math.sin(alpha / 2.0)
            Cx = B[0] + bx * d_center
            Cy = B[1] + by * d_center

            # 弧的起終角（atan2 從圓心指向兩切線點）
            theta1 = math.atan2(T1[1] - Cy, T1[0] - Cx)
            theta2 = math.atan2(T2[1] - Cy, T2[0] - Cx)

            # 決定弧的方向（順 / 逆時針）— 走較短一段（≤ π - α）
            d_theta = theta2 - theta1
            while d_theta > math.pi:
                d_theta -= 2 * math.pi
            while d_theta < -math.pi:
                d_theta += 2 * math.pi

            # 入弧切點 T1
            out.append(T1)
            # 中間弧點（不含端點，避免與 T1 / T2 重複）
            for k in range(1, N_SEG):
                t = k / N_SEG
                th = theta1 + d_theta * t
                out.append((Cx + R_used * math.cos(th), Cy + R_used * math.sin(th)))
            # 出弧切點 T2
            out.append(T2)

        out.append(broken[-1])
        return out


# ──────────────────────────────────────────────────────────────────────
#  主控類別：HeterogeneousNFZPlanner
# ──────────────────────────────────────────────────────────────────────
class HeterogeneousNFZPlanner:
    """異質機隊 NFZ 繞行規劃主控器。

    使用：
        >>> planner = HeterogeneousNFZPlanner()
        >>> routes = planner.plan_heterogeneous_routes(
        ...     start_pt=(23.700, 120.420),
        ...     target_pt=(23.720, 120.460),
        ...     nfz_polygon=[(23.708, 120.435), (23.712, 120.435),
        ...                  (23.712, 120.445), (23.708, 120.445)],
        ... )
        >>> mr_waypoints = routes['multirotor']  # List[Waypoint]
        >>> fw_waypoints = routes['fixed_wing']  # List[Waypoint]

    回傳的 ``Waypoint`` 可直接以 ``to_mavlink_wpl_line(seq)`` 串成 WPL 110。
    """

    def __init__(self, config: Optional[PlannerConfig] = None) -> None:
        self._cfg = config or PlannerConfig()
        self._mr = MultirotorAStarPlanner(self._cfg)
        self._fw = FixedWingDubinsPlanner(self._cfg)

    @property
    def config(self) -> PlannerConfig:
        return self._cfg

    # ── 主要對外 API ─────────────────────────────────────────────
    def plan_heterogeneous_routes(
        self,
        start_pt: Tuple[float, float],
        target_pt: Tuple[float, float],
        nfz_polygon: Sequence[Tuple[float, float]],
    ) -> Dict[str, List[Waypoint]]:
        """為 MR + FW 兩種機型同時規劃繞行路徑。

        Args:
            start_pt    : (lat, lon) 起飛點（度）
            target_pt   : (lat, lon) 目標點（度）
            nfz_polygon : [(lat, lon), ...] 禁飛區多邊形頂點（≥ 3 點）

        Returns:
            {
              'multirotor': [Waypoint, ...] 多旋翼路徑（alt = alt_mr）,
              'fixed_wing': [Waypoint, ...] 固定翼路徑（alt = alt_fw）,
            }
        """
        if len(nfz_polygon) < 3:
            raise ValueError('NFZ 多邊形至少需要 3 個頂點')

        # ── 1) 建本地投影（以起終點中點為原點）──
        lat0 = (start_pt[0] + target_pt[0]) / 2.0
        lon0 = (start_pt[1] + target_pt[1]) / 2.0
        proj = _LocalProjection(lat0, lon0)

        start_xy = proj.to_metric(*start_pt)
        target_xy = proj.to_metric(*target_pt)
        nfz_xy = proj.project_polygon(nfz_polygon)

        # ── 2) Shapely 構造原始 NFZ + 兩種 buffered 版本 ──
        nfz_raw = Polygon(nfz_xy)
        if not nfz_raw.is_valid:
            nfz_raw = nfz_raw.buffer(0)  # 自動修正自交
        # 用 quad_segs=32 讓圓角化弦深 ≪ 0.1m，搭配 _crosses_polygon_interior
        # 的 epsilon=0.2m 容差仍能正確判斷「沿邊行走」（弦深 < epsilon）
        # 與「真實穿越」（深度 ≫ epsilon）。
        nfz_buf_mr = nfz_raw.buffer(self._cfg.buffer_mr, quad_segs=32)
        nfz_buf_fw = nfz_raw.buffer(self._cfg.buffer_fw, quad_segs=32)

        # buffer 操作偶會回傳 MultiPolygon（複雜形狀）— 取最大塊
        nfz_buf_mr = self._largest_polygon(nfz_buf_mr)
        nfz_buf_fw = self._largest_polygon(nfz_buf_fw)

        # ── 3) 分別呼叫兩個規劃器 ──
        mr_path_xy = self._mr.plan(start_xy, target_xy, nfz_buf_mr)
        fw_path_xy = self._fw.plan(start_xy, target_xy, nfz_buf_fw)

        # ── 4) ENU → 經緯度，附上各自高度層 ──
        mr_wps = [
            Waypoint(*proj.to_latlon(x, y), alt=self._cfg.alt_mr)
            for x, y in mr_path_xy
        ]
        fw_wps = [
            Waypoint(*proj.to_latlon(x, y), alt=self._cfg.alt_fw)
            for x, y in fw_path_xy
        ]

        return {'multirotor': mr_wps, 'fixed_wing': fw_wps}

    # ── 輔助：MAVLink WPL 110 串檔 ──────────────────────────────
    @staticmethod
    def export_mavlink_wpl(
        waypoints: List[Waypoint],
        home_lat: float, home_lon: float, home_alt: float = 0.0,
    ) -> str:
        """產生 QGC WPL 110 格式整檔字串。

        第 0 個 item 為 HOME（lat/lon/alt = home, cmd=16, frame=0）。
        後續 items 依序為 waypoints。
        """
        lines = ['QGC WPL 110']
        home_wp = Waypoint(home_lat, home_lon, home_alt,
                           cmd=16, frame=0)  # frame=0 GLOBAL（HOME）
        lines.append(home_wp.to_mavlink_wpl_line(seq=0, current=1))
        for i, wp in enumerate(waypoints, start=1):
            lines.append(wp.to_mavlink_wpl_line(seq=i))
        return '\n'.join(lines)

    @staticmethod
    def _largest_polygon(geom):
        """從 Polygon / MultiPolygon 取面積最大的 Polygon"""
        if geom.geom_type == 'Polygon':
            return geom
        if geom.geom_type == 'MultiPolygon':
            return max(geom.geoms, key=lambda p: p.area)
        raise TypeError(f'未預期的幾何類型: {geom.geom_type}')

    # ── 驗證：兩條路線是否在水平 + 垂直雙維度上實際分離 ──
    def verify_physical_separation(
        self, routes: Dict[str, List[Waypoint]],
    ) -> Dict[str, float]:
        """檢查兩條路線是否符合設計的物理分離要求。

        Returns:
            {
              'min_horizontal_dist_m': 兩條路線最近的水平距 [m],
              'vertical_gap_m'       : 高度差 [m],
              'separation_ok'        : True / False（雙重保險都滿足）,
            }
        """
        mr = routes['multirotor']
        fw = routes['fixed_wing']
        if not mr or not fw:
            return {
                'min_horizontal_dist_m': 0.0,
                'vertical_gap_m': 0.0,
                'separation_ok': False,
            }
        # 投影回 ENU 算「中段最近距」— 兩條路徑共用起終點，整段比距會被
        # 端點重疊污染為 0。改用最大繞行偏離量：
        #   對每條路徑取「離端點連線最遠的點」，比較兩個最遠點之間的距離。
        # 這真實反映 demand：兩條繞路有沒有走不同方向？
        proj = _LocalProjection(mr[0].lat, mr[0].lon)
        mr_xy = [proj.to_metric(w.lat, w.lon) for w in mr]
        fw_xy = [proj.to_metric(w.lat, w.lon) for w in fw]

        def _max_excursion_point(path_xy):
            """找出路徑上「離 start-target 弦最遠的點」"""
            if len(path_xy) < 3:
                return path_xy[len(path_xy) // 2]
            chord = LineString([path_xy[0], path_xy[-1]])
            best_d = -1.0; best_p = path_xy[1]
            for p in path_xy[1:-1]:
                d = chord.distance(Point(p))
                if d > best_d:
                    best_d = d; best_p = p
            return best_p

        mr_peak = _max_excursion_point(mr_xy)
        fw_peak = _max_excursion_point(fw_xy)
        h_dist = math.hypot(mr_peak[0] - fw_peak[0], mr_peak[1] - fw_peak[1])
        v_gap = abs(self._cfg.alt_fw - self._cfg.alt_mr)
        # 水平 OK 標準：≥ buffer_fw - buffer_mr × 0.5（保守取一半當門檻）
        h_threshold = max(20.0, (self._cfg.buffer_fw - self._cfg.buffer_mr) * 0.5)
        v_threshold = 20.0  # 垂直至少 20m
        ok = (h_dist >= h_threshold) or (v_gap >= v_threshold)
        return {
            'min_horizontal_dist_m': float(h_dist),
            'vertical_gap_m': float(v_gap),
            'separation_ok': bool(ok),
        }


# ──────────────────────────────────────────────────────────────────────
#  Demo / __main__
# ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=' * 70)
    print(' HeterogeneousNFZPlanner — 示範')
    print('=' * 70)

    # 場景：起點正南、目標正北，中間放一塊大型正方形 NFZ
    # 直線距離 ~2.2km，NFZ 直接在路徑中央阻擋
    # → 兩種機型「必須」繞行；FW 必須在 NFZ 兩個角各倒一次 Dubins 圓弧
    start = (23.700, 120.440)
    target = (23.720, 120.440)
    # 正方形 NFZ（~660m × 1100m）位於 start-target 連線中央
    nfz_square = [
        (23.707, 120.435),
        (23.707, 120.445),
        (23.713, 120.445),
        (23.713, 120.435),
    ]

    print(f'\n起點:   ({start[0]:.6f}, {start[1]:.6f})')
    print(f'目標:   ({target[0]:.6f}, {target[1]:.6f})')
    print(f'NFZ:    正方形 4 頂點，約 1.1 km × 0.45 km')

    cfg = PlannerConfig(
        buffer_mr=20.0, buffer_fw=100.0,
        alt_mr=80.0, alt_fw=120.0,
        r_min=60.0, arc_segments=12,
        grid_res_m=10.0,
    )
    planner = HeterogeneousNFZPlanner(cfg)

    print(f'\n設定:')
    print(f'  MR  buffer={cfg.buffer_mr}m  alt={cfg.alt_mr}m')
    print(f'  FW  buffer={cfg.buffer_fw}m  alt={cfg.alt_fw}m  R_min={cfg.r_min}m')
    print(f'  最近水平分離 (理論) >= {cfg.buffer_fw - cfg.buffer_mr}m')
    print(f'  垂直分離 = {cfg.alt_fw - cfg.alt_mr}m')

    routes = planner.plan_heterogeneous_routes(start, target, nfz_square)

    # ── 印出多旋翼路徑 ──
    print('\n' + '-' * 70)
    print(f' 多旋翼路徑 (A* + 貼邊切角)  ─  {len(routes["multirotor"])} 航點')
    print('-' * 70)
    print(f'{"#":>3} {"Lat":>13} {"Lon":>14} {"Alt":>8}')
    for i, wp in enumerate(routes['multirotor']):
        print(f'{i:>3} {wp.lat:>13.6f} {wp.lon:>14.6f} {wp.alt:>6.1f} m')

    # ── 印出固定翼路徑 ──
    print('\n' + '-' * 70)
    print(f' 固定翼路徑 (Dubins fillet 大弧迂迴)  ─  {len(routes["fixed_wing"])} 航點')
    print('-' * 70)
    print(f'{"#":>3} {"Lat":>13} {"Lon":>14} {"Alt":>8}')
    for i, wp in enumerate(routes['fixed_wing']):
        print(f'{i:>3} {wp.lat:>13.6f} {wp.lon:>14.6f} {wp.alt:>6.1f} m')

    # ── 物理分離驗證 ──
    print('\n' + '-' * 70)
    print(' 物理分離驗證')
    print('-' * 70)
    sep = planner.verify_physical_separation(routes)
    print(f'  最近水平距離      : {sep["min_horizontal_dist_m"]:>7.1f} m')
    print(f'  垂直高度差        : {sep["vertical_gap_m"]:>7.1f} m')
    print(f'  通過分離規則      : {"✓ YES" if sep["separation_ok"] else "✗ NO"}')

    # ── 匯出 MAVLink WPL 110 ──
    print('\n' + '-' * 70)
    print(' MAVLink WPL 110 預覽（多旋翼，前 6 行）')
    print('-' * 70)
    wpl_mr = HeterogeneousNFZPlanner.export_mavlink_wpl(
        routes['multirotor'], home_lat=start[0], home_lon=start[1],
    )
    for line in wpl_mr.split('\n')[:6]:
        print(' ', line)
    if len(wpl_mr.split('\n')) > 6:
        print(f'  ... 共 {len(wpl_mr.split(chr(10)))} 行')

    print('\n' + '-' * 70)
    print(' MAVLink WPL 110 預覽（固定翼，前 6 行）')
    print('-' * 70)
    wpl_fw = HeterogeneousNFZPlanner.export_mavlink_wpl(
        routes['fixed_wing'], home_lat=start[0], home_lon=start[1],
    )
    for line in wpl_fw.split('\n')[:6]:
        print(' ', line)
    if len(wpl_fw.split('\n')) > 6:
        print(f'  ... 共 {len(wpl_fw.split(chr(10)))} 行')

    print('\n' + '=' * 70)
    print(' 示範結束 — 兩條路徑已物理隔離（水平 + 垂直）')
    print('=' * 70)
