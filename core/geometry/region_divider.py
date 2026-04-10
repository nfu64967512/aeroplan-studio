"""
區域分割器模組
將多邊形區域分割為子區域，支援最多 6 台無人機

分割策略：
  n=1        : 不分割，直接返回原區域
  n=2,3,5    : 水平條帶（均等寬度，可設間隔）
  n=4        : 2×2 網格
  n=6        : 3×2 網格（3 列 × 2 行）

參考 RegionDivider (region_divider.py) 並擴充至 6 台
"""

import math
from typing import List, Tuple, Optional, Dict, Any


class RegionDivider:
    """區域分割器 — 支援四邊形雙線性插值分割與任意多邊形水平條帶分割"""

    # ------------------------------------------------------------------
    # 雙線性插值（四邊形專用）
    # ------------------------------------------------------------------
    @staticmethod
    def bilinear_interpolation(
        corners: List[Tuple[float, float]], u: float, v: float
    ) -> Tuple[float, float]:
        """
        四邊形雙線性插值

        corners 順序：左下(p0), 右下(p1), 右上(p2), 左上(p3)
        u ∈ [0,1]：水平方向（左→右）
        v ∈ [0,1]：垂直方向（下→上）
        """
        if len(corners) != 4:
            raise ValueError("雙線性插值需要 4 個角點")

        p0, p1, p2, p3 = corners
        x = (1-u)*(1-v)*p0[0] + u*(1-v)*p1[0] + u*v*p2[0] + (1-u)*v*p3[0]
        y = (1-u)*(1-v)*p0[1] + u*(1-v)*p1[1] + u*v*p2[1] + (1-u)*v*p3[1]
        return (x, y)

    # ------------------------------------------------------------------
    # 四邊形分割（雙線性插值）
    # ------------------------------------------------------------------
    @staticmethod
    def subdivide_rectangle(
        corners: List[Tuple[float, float]],
        n: int,
        spacing_m: float = 0.0,
        gap_spacings_m: Optional[List[float]] = None,
        v_spacing_m: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """
        分割四邊形區域為 n 個子區域

        n=1        : 原樣返回
        n=2,3,5    : 水平條帶（沿 u 軸等分）
        n=4        : 2×2 網格
        n=6        : 3×2 網格（3 列 × 2 行）

        參數:
            corners          : 四個角點 [(lat,lon)×4]，順序：左下、右下、右上、左上
            n                : 分割數量（1–6）
            spacing_m        : 預設間隔（公尺），作為所有間隔的回退值
            gap_spacings_m   : 條帶模式（n=2,3,5）各邊界間隔列表，長度 = n-1
            v_spacing_m      : 網格模式（n=4,6）垂直方向間隔（公尺）；
                               水平方向使用 spacing_m

        返回:
            子區域列表，每個子區域為 4 個角點
        """
        if len(corners) != 4:
            raise ValueError("矩形分割需要 4 個角點")
        if not 1 <= n <= 6:
            raise ValueError(f"n 必須在 1–6 之間，收到 {n}")

        if n == 1:
            return [corners]

        # 計算參考寬度與高度
        w1 = RegionDivider._distance(corners[0], corners[1])
        w2 = RegionDivider._distance(corners[3], corners[2])
        avg_w = (w1 + w2) / 2 if (w1 + w2) > 0 else 1.0

        h1 = RegionDivider._distance(corners[0], corners[3])
        h2 = RegionDivider._distance(corners[1], corners[2])
        avg_h = (h1 + h2) / 2 if (h1 + h2) > 0 else 1.0

        # 預設水平間隔比例（最多 15%）
        default_h_ratio = min(0.15, spacing_m / avg_w) if spacing_m > 0 else 0.0
        # 垂直間隔比例（網格模式）
        v_ratio = min(0.15, v_spacing_m / avg_h) if v_spacing_m and v_spacing_m > 0 else default_h_ratio

        bi = RegionDivider.bilinear_interpolation
        regions: List[List[Tuple[float, float]]] = []

        if n in (2, 3, 5):
            # ── 水平條帶：支援每條邊界各自設定間隔 ─────────────
            n_gaps = n - 1
            if gap_spacings_m and len(gap_spacings_m) >= n_gaps:
                gap_ratios = [min(0.15, g / avg_w) if avg_w > 0 else 0.0
                              for g in gap_spacings_m[:n_gaps]]
            else:
                gap_ratios = [default_h_ratio] * n_gaps

            total_gap = sum(gap_ratios)
            seg = max(0.01, (1.0 - total_gap) / n)

            u_pos = 0.0
            for i in range(n):
                u0 = max(0.0, min(1.0, u_pos))
                u1 = max(0.0, min(1.0, u_pos + seg))
                if u0 < u1:
                    regions.append([
                        bi(corners, u0, 0),
                        bi(corners, u1, 0),
                        bi(corners, u1, 1),
                        bi(corners, u0, 1),
                    ])
                u_pos = u1
                if i < n_gaps:
                    u_pos += gap_ratios[i]

        elif n == 4:
            # ── 2×2 網格：水平用 spacing_m，垂直用 v_spacing_m ──
            regions = RegionDivider._grid(corners, cols=2, rows=2,
                                          h_spacing_ratio=default_h_ratio,
                                          v_spacing_ratio=v_ratio)

        elif n == 6:
            # ── 3×2 網格（3 列 × 2 行）───────────────────────────
            regions = RegionDivider._grid(corners, cols=3, rows=2,
                                          h_spacing_ratio=default_h_ratio,
                                          v_spacing_ratio=v_ratio)

        return regions

    @staticmethod
    def _grid(
        corners: List[Tuple[float, float]],
        cols: int,
        rows: int,
        spacing_ratio: float = 0.0,
        h_spacing_ratio: Optional[float] = None,
        v_spacing_ratio: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """
        通用矩形網格分割輔助函式，支援水平/垂直獨立間隔

        分割順序：row 0 col 0, row 0 col 1, ..., row 1 col 0, ...
        """
        bi = RegionDivider.bilinear_interpolation
        h_sr = h_spacing_ratio if h_spacing_ratio is not None else spacing_ratio
        v_sr = v_spacing_ratio if v_spacing_ratio is not None else spacing_ratio

        total_u_gap = h_sr * (cols - 1)
        total_v_gap = v_sr * (rows - 1)
        cell_u = max(0.01, (1.0 - total_u_gap) / cols)
        cell_v = max(0.01, (1.0 - total_v_gap) / rows)

        regions: List[List[Tuple[float, float]]] = []
        for row in range(rows):
            for col in range(cols):
                u0 = max(0.0, min(1.0, col * (cell_u + h_sr)))
                u1 = max(0.0, min(1.0, u0 + cell_u))
                v0 = max(0.0, min(1.0, row * (cell_v + v_sr)))
                v1 = max(0.0, min(1.0, v0 + cell_v))
                if u0 >= u1 or v0 >= v1:
                    continue
                regions.append([
                    bi(corners, u0, v0),
                    bi(corners, u1, v0),
                    bi(corners, u1, v1),
                    bi(corners, u0, v1),
                ])
        return regions

    # ------------------------------------------------------------------
    # 任意多邊形分割（水平條帶）
    # ------------------------------------------------------------------
    @staticmethod
    def subdivide_polygon(
        corners: List[Tuple[float, float]],
        n: int,
        spacing_m: float = 0.0
    ) -> List[List[Tuple[float, float]]]:
        """
        分割任意多邊形為 n 條水平條帶

        適用於非四邊形多邊形；座標單位為經緯度，
        spacing_m 以公尺計並自動換算為緯度度數。

        參數:
            corners   : 多邊形頂點列表（經緯度）
            n         : 分割數量（1–6）
            spacing_m : 子區域間隔（公尺）

        返回:
            子區域頂點列表
        """
        if not 1 <= n <= 6:
            raise ValueError(f"n 必須在 1–6 之間，收到 {n}")

        if n == 1:
            return [corners]

        min_y = min(p[1] for p in corners)
        max_y = max(p[1] for p in corners)
        total_h = max_y - min_y

        # 間隔換算（公尺 → 度，1° ≈ 111111 m）
        if spacing_m > 0:
            sp_deg = spacing_m / 111111.0
            total_sp = sp_deg * (n - 1)
            if total_sp >= total_h * 0.5:
                sp_deg = total_h * 0.1 / (n - 1)
                total_sp = sp_deg * (n - 1)
        else:
            sp_deg = 0.0
            total_sp = 0.0

        strip_h = (total_h - total_sp) / n
        regions: List[List[Tuple[float, float]]] = []

        for i in range(n):
            y0 = max(min_y, min_y + i * (strip_h + sp_deg))
            y1 = min(max_y, y0 + strip_h)
            if y0 >= y1:
                continue

            strip_pts: List[Tuple[float, float]] = []
            for j in range(len(corners)):
                x1, y1_ = corners[j]
                x2, y2 = corners[(j + 1) % len(corners)]

                for y_cut in (y0, y1):
                    if (y1_ <= y_cut <= y2) or (y2 <= y_cut <= y1_):
                        if abs(y2 - y1_) > 1e-10:
                            xi = x1 + (y_cut - y1_) * (x2 - x1) / (y2 - y1_)
                            strip_pts.append((xi, y_cut))

                if y0 <= y1_ <= y1:
                    strip_pts.append((x1, y1_))

            if len(strip_pts) >= 3:
                cx = sum(p[0] for p in strip_pts) / len(strip_pts)
                cy = sum(p[1] for p in strip_pts) / len(strip_pts)
                strip_pts.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
                regions.append(strip_pts)
            else:
                regions.append(corners)

        return regions if regions else [corners]

    # ------------------------------------------------------------------
    # 基於 FOV 的作業路徑分解
    # ------------------------------------------------------------------
    @staticmethod
    def decompose_by_fov(
        polygon: List[Tuple[float, float]],
        scan_angle_deg: float,
        coverage_width_m: float,
        overlap_rate: float = 0.1,
    ) -> List[Dict[str, Any]]:
        """
        基於 FOV 覆蓋寬度將多邊形區域分解為平行作業路徑。

        參數:
            polygon: 多邊形頂點 [(lat, lon), ...]
            scan_angle_deg: 掃描方向角度（度，0=北，順時針為正，可用 optimal_scan_angle() 的結果）
            coverage_width_m: 單條掃描覆蓋寬度（公尺），即 FOV 地面投影寬度
            overlap_rate: 相鄰掃描線重疊率 (0–1)

        返回:
            List[Dict]，每個元素代表一條作業路徑：
            {
                'index': int,               # 路徑編號（從 0 開始）
                'left_point': (lat, lon),   # 左端點（掃描線起點）
                'right_point': (lat, lon),  # 右端點（掃描線終點）
                'length_m': float,          # 作業路徑長度（公尺）
                'direction_deg': float,     # 飛行方向角度（同 scan_angle_deg）
            }
        """
        if len(polygon) < 3 or coverage_width_m <= 0:
            return []

        # 有效掃描間距（考慮重疊率）
        overlap_rate = max(0.0, min(0.99, overlap_rate))
        step_m = coverage_width_m * (1.0 - overlap_rate)
        if step_m <= 0:
            return []

        # ── 步驟 1：經緯度 → 局部公尺座標（east, north）────────────
        origin_lat, origin_lon = polygon[0]
        lat_to_m = 111111.0
        lon_to_m = 111111.0 * math.cos(math.radians(origin_lat))

        # pts_en[i] = (east_m, north_m)
        pts_en = [
            ((lon - origin_lon) * lon_to_m, (lat - origin_lat) * lat_to_m)
            for lat, lon in polygon
        ]

        # ── 步驟 2：旋轉座標系使 x' 軸對齊掃描方向 ──────────────────
        # scan_angle_deg = bearing（0=北，順時針為正）
        # 旋轉矩陣（正交，自逆）：
        #   xp = east*sin_t + north*cos_t   ← 沿掃描方向
        #   yp = east*cos_t - north*sin_t   ← 垂直掃描方向（掃描推進軸）
        theta = math.radians(scan_angle_deg)
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)

        def to_rot(east: float, north: float) -> Tuple[float, float]:
            return east * sin_t + north * cos_t, east * cos_t - north * sin_t

        def from_rot(xp: float, yp: float) -> Tuple[float, float]:
            # 逆旋轉（矩陣自逆）
            return xp * sin_t + yp * cos_t, xp * cos_t - yp * sin_t

        rot_pts = [to_rot(e, n) for e, n in pts_en]

        # ── 步驟 3：在旋轉空間中決定掃描線 y' 位置 ──────────────────
        yp_vals = [p[1] for p in rot_pts]
        yp_min, yp_max = min(yp_vals), max(yp_vals)

        n_poly = len(rot_pts)
        results: List[Dict[str, Any]] = []
        idx = 0

        # 第一條掃描線置於 yp_min + step/2，之後每步 step_m
        scan_yp = yp_min + step_m / 2.0

        while scan_yp <= yp_max + step_m * 1e-6:
            # ── 步驟 4：scanline 交點（半開區間避免頂點重複計數）──
            x_ints: List[float] = []
            for i in range(n_poly):
                xp1, yp1 = rot_pts[i]
                xp2, yp2 = rot_pts[(i + 1) % n_poly]
                lo, hi = (yp1, yp2) if yp1 < yp2 else (yp2, yp1)
                if lo >= hi:
                    continue  # 水平邊略過
                if lo < scan_yp <= hi:
                    t = (scan_yp - yp1) / (yp2 - yp1)
                    x_ints.append(xp1 + t * (xp2 - xp1))

            if len(x_ints) >= 2:
                x_ints.sort()
                xp_left, xp_right = x_ints[0], x_ints[-1]

                # ── 步驟 5：轉回局部公尺座標再轉為經緯度 ──────────
                e_l, n_l = from_rot(xp_left,  scan_yp)
                e_r, n_r = from_rot(xp_right, scan_yp)

                lat_l = origin_lat + n_l / lat_to_m
                lon_l = origin_lon + e_l / lon_to_m
                lat_r = origin_lat + n_r / lat_to_m
                lon_r = origin_lon + e_r / lon_to_m

                length_m = math.sqrt((e_r - e_l) ** 2 + (n_r - n_l) ** 2)

                if length_m > 0.1:  # 過濾極短殘段
                    results.append({
                        'index':        idx,
                        'left_point':   (lat_l, lon_l),
                        'right_point':  (lat_r, lon_r),
                        'length_m':     length_m,
                        'direction_deg': scan_angle_deg,
                    })
                    idx += 1

            scan_yp += step_m

        return results

    # ------------------------------------------------------------------
    # 輔助
    # ------------------------------------------------------------------
    @staticmethod
    def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """兩點近似距離（公尺）"""
        dlat = p2[0] - p1[0]
        dlon = p2[1] - p1[1]
        return math.sqrt(dlat**2 + dlon**2) * 111111.0


# ==========================================
# 單元測試
# ==========================================
if __name__ == '__main__':

    def test_decompose_by_fov():
        """測試 decompose_by_fov()，使用雲林地區矩形多邊形。"""
        print("=" * 55)
        print("test_decompose_by_fov()")
        print("=" * 55)

        # 雲林近似矩形：約 556m(N-S) × 611m(E-W)
        poly = [
            (23.700, 120.420),
            (23.705, 120.420),
            (23.705, 120.426),
            (23.700, 120.426),
        ]
        coverage_w = 100.0   # m
        overlap    = 0.1     # 10%
        # 有效間距 = 100 * 0.9 = 90m

        # ── 案例 1：向北掃描（scan_angle=0）→ 掃描線為 N-S，E-W 間距
        paths_ns = RegionDivider.decompose_by_fov(poly, 0.0, coverage_w, overlap)
        n_expected = int((611.0 / 90.0))  # 約 6~7 條
        print(f"[案例 1] 向北掃描  → {len(paths_ns)} 條作業路徑（預期 ~{n_expected}）")
        for p in paths_ns:
            print(f"  idx={p['index']}  length={p['length_m']:.1f}m  "
                  f"left={p['left_point']}  right={p['right_point']}")
        assert 5 <= len(paths_ns) <= 8, f"條數異常: {len(paths_ns)}"

        # ── 案例 2：向東掃描（scan_angle=90）→ 掃描線為 E-W，N-S 間距
        paths_ew = RegionDivider.decompose_by_fov(poly, 90.0, coverage_w, overlap)
        print(f"\n[案例 2] 向東掃描  → {len(paths_ew)} 條作業路徑")
        for p in paths_ew:
            print(f"  idx={p['index']}  length={p['length_m']:.1f}m")
        assert len(paths_ew) >= 4, f"條數異常: {len(paths_ew)}"

        # ── 案例 3：不規則五邊形
        pent = [
            (23.700, 120.420),
            (23.702, 120.418),
            (23.704, 120.419),
            (23.704, 120.422),
            (23.701, 120.423),
        ]
        paths_pent = RegionDivider.decompose_by_fov(pent, 45.0, 80.0, 0.0)
        print(f"\n[案例 3] 五邊形 45° → {len(paths_pent)} 條作業路徑")
        for p in paths_pent:
            print(f"  idx={p['index']}  length={p['length_m']:.1f}m")
        assert len(paths_pent) >= 1

        print("\n所有斷言通過")

    test_decompose_by_fov()
