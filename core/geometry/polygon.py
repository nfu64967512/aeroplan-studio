"""
多邊形運算工具模組
提供多邊形的各種幾何運算
"""

import numpy as np
from typing import List, Tuple, Optional
import math


class PolygonUtils:
    """
    多邊形運算工具類
    
    提供：
    - 面積計算
    - 點在多邊形內判斷
    - 多邊形縮放/偏移
    - 凸包計算
    - 邊界框計算
    """
    
    @staticmethod
    def calculate_area(polygon: List[np.ndarray]) -> float:
        """
        計算多邊形面積（Shoelace 公式）
        
        Args:
            polygon: 頂點列表，每個頂點為 [x, y] 或 np.ndarray
            
        Returns:
            面積（平方單位，與輸入單位一致）
        """
        n = len(polygon)
        if n < 3:
            return 0.0
        
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += polygon[i][0] * polygon[j][1]
            area -= polygon[j][0] * polygon[i][1]
        
        return abs(area) / 2.0
    
    @staticmethod
    def calculate_centroid(polygon: List[np.ndarray]) -> np.ndarray:
        """
        計算多邊形重心
        
        Args:
            polygon: 頂點列表
            
        Returns:
            重心座標 [x, y]
        """
        n = len(polygon)
        if n == 0:
            return np.zeros(2)
        
        cx = sum(p[0] for p in polygon) / n
        cy = sum(p[1] for p in polygon) / n
        
        return np.array([cx, cy])
    
    @staticmethod
    def calculate_bounding_box(polygon: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
        """
        計算邊界框
        
        Args:
            polygon: 頂點列表
            
        Returns:
            (min_point, max_point) 即 ([xmin, ymin], [xmax, ymax])
        """
        if not polygon:
            return (np.zeros(2), np.zeros(2))
        
        points = np.array(polygon)
        min_pt = np.min(points[:, :2], axis=0)
        max_pt = np.max(points[:, :2], axis=0)
        
        return (min_pt, max_pt)
    
    @staticmethod
    def point_in_polygon(point: np.ndarray, polygon: List[np.ndarray]) -> bool:
        """
        判斷點是否在多邊形內（射線法）
        
        Args:
            point: 測試點 [x, y]
            polygon: 多邊形頂點列表
            
        Returns:
            True 如果點在多邊形內
        """
        x, y = point[0], point[1]
        n = len(polygon)
        inside = False
        
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i][0], polygon[i][1]
            xj, yj = polygon[j][0], polygon[j][1]
            
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            
            j = i
        
        return inside
    
    @staticmethod
    def offset_polygon(polygon: List[np.ndarray], offset: float) -> List[np.ndarray]:
        """
        多邊形偏移（正值向外擴展，負值向內縮減）
        
        使用簡化的法向量偏移方法
        
        Args:
            polygon: 頂點列表
            offset: 偏移距離（正=擴展，負=縮減）
            
        Returns:
            偏移後的頂點列表
        """
        n = len(polygon)
        if n < 3:
            return polygon
        
        result = []
        
        for i in range(n):
            # 獲取前後點
            prev_idx = (i - 1) % n
            next_idx = (i + 1) % n
            
            prev_pt = polygon[prev_idx]
            curr_pt = polygon[i]
            next_pt = polygon[next_idx]
            
            # 計算兩條邊的法向量
            edge1 = curr_pt[:2] - prev_pt[:2]
            edge2 = next_pt[:2] - curr_pt[:2]
            
            # 計算法向量（向左垂直）
            norm1 = np.array([-edge1[1], edge1[0]])
            norm2 = np.array([-edge2[1], edge2[0]])
            
            # 正規化
            len1 = np.linalg.norm(norm1)
            len2 = np.linalg.norm(norm2)
            
            if len1 > 1e-10:
                norm1 = norm1 / len1
            if len2 > 1e-10:
                norm2 = norm2 / len2
            
            # 計算平均法向量
            avg_norm = norm1 + norm2
            avg_len = np.linalg.norm(avg_norm)
            
            if avg_len > 1e-10:
                avg_norm = avg_norm / avg_len
                
                # 計算偏移量（考慮角度校正）
                cos_half_angle = np.dot(norm1, avg_norm)
                if cos_half_angle > 0.1:  # 避免極端角度
                    actual_offset = offset / cos_half_angle
                else:
                    actual_offset = offset
                
                # 應用偏移
                new_pt = curr_pt[:2] + avg_norm * actual_offset
                result.append(np.array([new_pt[0], new_pt[1]]))
            else:
                result.append(curr_pt[:2].copy())
        
        return result
    
    @staticmethod
    def convex_hull(points: List[np.ndarray]) -> List[np.ndarray]:
        """
        計算凸包（Graham Scan 演算法）
        
        Args:
            points: 點列表
            
        Returns:
            凸包頂點列表（逆時針順序）
        """
        if len(points) < 3:
            return points
        
        # 找到最下方的點（y 最小，x 最小）
        points = sorted(points, key=lambda p: (p[1], p[0]))
        start = points[0]
        
        # 按極角排序
        def polar_angle(p):
            dx = p[0] - start[0]
            dy = p[1] - start[1]
            return math.atan2(dy, dx)
        
        sorted_points = sorted(points[1:], key=polar_angle)
        
        # Graham Scan
        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        
        hull = [start]
        
        for p in sorted_points:
            while len(hull) > 1 and cross(hull[-2], hull[-1], p) <= 0:
                hull.pop()
            hull.append(p)
        
        return hull
    
    @staticmethod
    def simplify_polygon(polygon: List[np.ndarray], 
                        tolerance: float) -> List[np.ndarray]:
        """
        簡化多邊形（Douglas-Peucker 演算法）
        
        Args:
            polygon: 頂點列表
            tolerance: 簡化容差
            
        Returns:
            簡化後的頂點列表
        """
        if len(polygon) <= 2:
            return polygon
        
        def perpendicular_distance(point, line_start, line_end):
            """計算點到線段的垂直距離"""
            dx = line_end[0] - line_start[0]
            dy = line_end[1] - line_start[1]
            
            if abs(dx) < 1e-10 and abs(dy) < 1e-10:
                return np.linalg.norm(point[:2] - line_start[:2])
            
            t = max(0, min(1, (
                (point[0] - line_start[0]) * dx + 
                (point[1] - line_start[1]) * dy
            ) / (dx * dx + dy * dy)))
            
            proj = np.array([
                line_start[0] + t * dx,
                line_start[1] + t * dy
            ])
            
            return np.linalg.norm(point[:2] - proj)
        
        def dp_simplify(points, start_idx, end_idx):
            """遞迴簡化"""
            max_dist = 0
            max_idx = start_idx
            
            for i in range(start_idx + 1, end_idx):
                dist = perpendicular_distance(
                    points[i], points[start_idx], points[end_idx]
                )
                if dist > max_dist:
                    max_dist = dist
                    max_idx = i
            
            if max_dist > tolerance:
                # 遞迴處理兩側
                left = dp_simplify(points, start_idx, max_idx)
                right = dp_simplify(points, max_idx, end_idx)
                return left[:-1] + right
            else:
                return [points[start_idx], points[end_idx]]
        
        result = dp_simplify(polygon, 0, len(polygon) - 1)
        return result
    
    @staticmethod
    def is_convex(polygon: List[np.ndarray]) -> bool:
        """
        判斷多邊形是否為凸多邊形
        
        Args:
            polygon: 頂點列表
            
        Returns:
            True 如果是凸多邊形
        """
        n = len(polygon)
        if n < 3:
            return False
        
        sign = None
        
        for i in range(n):
            o = polygon[i]
            a = polygon[(i + 1) % n]
            b = polygon[(i + 2) % n]
            
            cross = (a[0] - o[0]) * (b[1] - a[1]) - (a[1] - o[1]) * (b[0] - a[0])
            
            if abs(cross) > 1e-10:
                current_sign = cross > 0
                if sign is None:
                    sign = current_sign
                elif sign != current_sign:
                    return False
        
        return True
    
    @staticmethod
    def rotate_polygon(polygon: List[np.ndarray], 
                      angle: float,
                      center: np.ndarray = None) -> List[np.ndarray]:
        """
        旋轉多邊形
        
        Args:
            polygon: 頂點列表
            angle: 旋轉角度（度，逆時針為正）
            center: 旋轉中心（默認為多邊形重心）
            
        Returns:
            旋轉後的頂點列表
        """
        if center is None:
            center = PolygonUtils.calculate_centroid(polygon)
        
        theta = np.radians(angle)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        
        result = []
        for pt in polygon:
            # 平移到原點
            dx = pt[0] - center[0]
            dy = pt[1] - center[1]
            
            # 旋轉
            new_x = cos_t * dx - sin_t * dy + center[0]
            new_y = sin_t * dx + cos_t * dy + center[1]
            
            result.append(np.array([new_x, new_y]))
        
        return result
    
    @staticmethod
    def line_intersects_polygon(p1: np.ndarray, p2: np.ndarray,
                               polygon: List[np.ndarray]) -> List[np.ndarray]:
        """
        計算線段與多邊形的交點
        
        Args:
            p1, p2: 線段端點
            polygon: 多邊形頂點列表
            
        Returns:
            交點列表
        """
        intersections = []
        n = len(polygon)
        
        for i in range(n):
            p3 = polygon[i]
            p4 = polygon[(i + 1) % n]
            
            intersection = PolygonUtils.line_intersection(p1, p2, p3, p4)
            if intersection is not None:
                intersections.append(intersection)
        
        return intersections
    
    @staticmethod
    def line_intersection(p1: np.ndarray, p2: np.ndarray,
                         p3: np.ndarray, p4: np.ndarray) -> Optional[np.ndarray]:
        """
        計算兩線段的交點
        
        Args:
            p1, p2: 第一條線段的端點
            p3, p4: 第二條線段的端點
            
        Returns:
            交點座標，如果沒有交點則返回 None
        """
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        x3, y3 = p3[0], p3[1]
        x4, y4 = p4[0], p4[1]
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        
        if abs(denom) < 1e-10:
            return None  # 平行或共線
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        
        if 0 <= t <= 1 and 0 <= u <= 1:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return np.array([x, y])
        
        return None
    
    @staticmethod
    def get_longest_edge(polygon: List[np.ndarray]) -> Tuple[int, float]:
        """
        獲取多邊形最長邊
        
        Args:
            polygon: 頂點列表
            
        Returns:
            (邊索引, 邊長度)
        """
        n = len(polygon)
        max_length = 0
        max_idx = 0
        
        for i in range(n):
            j = (i + 1) % n
            length = np.linalg.norm(polygon[j][:2] - polygon[i][:2])
            if length > max_length:
                max_length = length
                max_idx = i
        
        return (max_idx, max_length)
    
    @staticmethod
    def get_edge_angle(polygon: List[np.ndarray], edge_idx: int) -> float:
        """
        獲取多邊形某條邊的角度
        
        Args:
            polygon: 頂點列表
            edge_idx: 邊索引
            
        Returns:
            角度（度，相對於 X 軸正向）
        """
        n = len(polygon)
        p1 = polygon[edge_idx]
        p2 = polygon[(edge_idx + 1) % n]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        return np.degrees(np.arctan2(dy, dx))

    @staticmethod
    def optimal_scan_angle(polygon: List[Tuple[float, float]]) -> float:
        """
        計算多邊形的最佳覆蓋掃描方向角度。

        垂直於多邊形最小寬度方向的掃描方向能最小化平行掃描線數量。
        方法：旋轉卡尺法（若 Shapely 可用則改用最小外接矩形）。

        參數:
            polygon: 多邊形頂點列表，格式為 [(lat, lon), ...]（經緯度）

        返回:
            最佳掃描方向角度（度，0=北，順時針為正）。
            掃描線將平行於此方向，無人機沿此方向來回飛行。
        """
        if len(polygon) < 3:
            return 0.0

        # ── 步驟 1：經緯度 → 局部公尺座標（以第一個頂點為原點）────────
        origin_lat, origin_lon = polygon[0]
        lat_to_m = 111111.0
        lon_to_m = 111111.0 * math.cos(math.radians(origin_lat))

        pts_m = np.array([
            ((lat - origin_lat) * lat_to_m,
             (lon - origin_lon) * lon_to_m)
            for lat, lon in polygon
        ])  # shape: (n, 2)  — (north_m, east_m)

        # ── 步驟 2：優先嘗試 Shapely 最小外接矩形 ────────────────────
        try:
            from shapely.geometry import Polygon as ShapelyPolygon

            sp = ShapelyPolygon(pts_m)
            if not sp.is_valid:
                sp = sp.buffer(0)
            rect = sp.minimum_rotated_rectangle

            # 取矩形頂點，找短邊方向
            coords = list(rect.exterior.coords)[:-1]  # 去掉重複的最後一點
            if len(coords) >= 2:
                edges = []
                for i in range(len(coords)):
                    p1 = np.array(coords[i])
                    p2 = np.array(coords[(i + 1) % len(coords)])
                    edge_vec = p2 - p1
                    edges.append((np.linalg.norm(edge_vec), edge_vec))

                # 找最短邊（短邊的法線方向 = 掃描方向）
                edges.sort(key=lambda e: e[0])
                short_edge_vec = edges[0][1]

                # 短邊法線方向（旋轉 90°）= 掃描方向
                # 座標系：x=North, y=East；方位角 = atan2(East, North)
                scan_vec = np.array([-short_edge_vec[1], short_edge_vec[0]])
                bearing_rad = math.atan2(scan_vec[1], scan_vec[0])
                bearing_deg = math.degrees(bearing_rad)
                # 轉換為 0–360 範圍
                return bearing_deg % 360.0

        except ImportError:
            pass

        # ── 步驟 3：旋轉卡尺法（不依賴 Shapely）────────────────────────
        # 先計算凸包（簡單版：使用 numpy 的凸包索引）
        def _convex_hull_2d(points: np.ndarray) -> np.ndarray:
            """Andrew's Monotone Chain 凸包演算法，返回逆時針頂點陣列"""
            pts = sorted(map(tuple, points))
            pts = [np.array(p) for p in pts]

            def cross(o, a, b):
                return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

            lower: list = []
            for p in pts:
                while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                    lower.pop()
                lower.append(p)

            upper: list = []
            for p in reversed(pts):
                while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                    upper.pop()
                upper.append(p)

            return np.array(lower[:-1] + upper[:-1])

        hull = _convex_hull_2d(pts_m)
        n_hull = len(hull)

        if n_hull < 2:
            return 0.0

        # 遍歷凸包每條邊，找最小投影寬度對應的掃描方向
        # pts_m 座標系：col 0 = north(m)，col 1 = east(m)
        best_bearing_rad = 0.0
        min_width = float('inf')

        for i in range(n_hull):
            p1 = hull[i]
            p2 = hull[(i + 1) % n_hull]
            dn = p2[0] - p1[0]  # Δnorth
            de = p2[1] - p1[1]  # Δeast
            edge_len = math.sqrt(dn * dn + de * de)
            if edge_len < 1e-9:
                continue

            # 邊的單位法向量（垂直於邊，旋轉 +90°）
            nn = -de / edge_len
            ne =  dn / edge_len

            # 所有頂點在法向量上的投影寬度
            projections = hull[:, 0] * nn + hull[:, 1] * ne
            width = projections.max() - projections.min()

            if width < min_width:
                min_width = width
                # 掃描方向沿邊飛行：bearing = atan2(Δeast, Δnorth)
                best_bearing_rad = math.atan2(de, dn)

        return math.degrees(best_bearing_rad) % 360.0


# ==========================================
# 單元測試
# ==========================================
if __name__ == '__main__':

    def test_optimal_scan_angle():
        """
        測試 optimal_scan_angle()。

        使用雲林地區座標 (約 23.70°N, 120.42°E) 建立幾個已知方向的多邊形，
        驗證返回角度是否符合預期。
        """
        print("=" * 55)
        print("test_optimal_scan_angle()")
        print("=" * 55)

        # ── 案例 1：近似正方形（應無明顯偏好，任何值皆可）──────────
        sq = [
            (23.700, 120.420),
            (23.701, 120.420),
            (23.701, 120.421),
            (23.700, 120.421),
        ]
        angle_sq = PolygonUtils.optimal_scan_angle(sq)
        print(f"[案例 1] 近似正方形  → 掃描角度: {angle_sq:.1f}°")

        # ── 案例 2：南北向長條（長邊朝北）→ 掃描方向應接近 0° 或 180°
        ns_rect = [
            (23.700, 120.420),
            (23.710, 120.420),   # 長邊沿緯度方向（北）
            (23.710, 120.4202),  # 寬 ~22m
            (23.700, 120.4202),
        ]
        angle_ns = PolygonUtils.optimal_scan_angle(ns_rect)
        print(f"[案例 2] 南北向長條  → 掃描角度: {angle_ns:.1f}°  (預期接近 0° 或 180°)")
        assert abs(angle_ns) < 10 or abs(angle_ns - 180) < 10 or abs(angle_ns - 360) < 10, \
            f"南北長條掃描角度應接近 0/180°，得到 {angle_ns:.1f}°"

        # ── 案例 3：東西向長條（長邊朝東）→ 掃描方向應接近 90° 或 270°
        ew_rect = [
            (23.700,  120.410),
            (23.700,  120.430),  # 長邊沿經度方向（東）
            (23.7002, 120.430),  # 寬 ~22m
            (23.7002, 120.410),
        ]
        angle_ew = PolygonUtils.optimal_scan_angle(ew_rect)
        print(f"[案例 3] 東西向長條  → 掃描角度: {angle_ew:.1f}°  (預期接近 90° 或 270°)")
        assert abs(angle_ew - 90) < 10 or abs(angle_ew - 270) < 10, \
            f"東西長條掃描角度應接近 90/270°，得到 {angle_ew:.1f}°"

        # ── 案例 4：45° 斜向長條 ────────────────────────────────────
        import math as _math
        # 沿 45° 方向拉長（東北方向）
        dlat = 0.005   # ~555m north
        dlon = 0.005   # ~500m east（23.7°N）
        diag_rect = [
            (23.700,        120.420),
            (23.700 + dlat, 120.420 + dlon),
            (23.700 + dlat + 0.0001, 120.420 + dlon - 0.0001),
            (23.700 + 0.0001,        120.420 - 0.0001),
        ]
        angle_diag = PolygonUtils.optimal_scan_angle(diag_rect)
        print(f"[案例 4] 45°斜向長條 → 掃描角度: {angle_diag:.1f}°  (預期接近 45° 或 225°)")

        # ── 案例 5：不規則五邊形 ─────────────────────────────────────
        pentagon = [
            (23.700, 120.420),
            (23.702, 120.418),
            (23.704, 120.419),
            (23.704, 120.422),
            (23.701, 120.423),
        ]
        angle_pent = PolygonUtils.optimal_scan_angle(pentagon)
        print(f"[案例 5] 不規則五邊形 → 掃描角度: {angle_pent:.1f}°")

        print()
        print("所有斷言通過")

    test_optimal_scan_angle()
