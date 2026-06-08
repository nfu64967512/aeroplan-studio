"""tests/test_heterogeneous_nfz_planner.py — HeterogeneousNFZPlanner 完整內部測試

每個場景驗證 6 個條件：
    1. MR 不穿越原始 NFZ 內部
    2. FW 不穿越原始 NFZ 內部
    3. MR 路徑與 MR-buffered NFZ 的安全距離 OK
    4. FW 路徑與 FW-buffered NFZ 的安全距離 OK
    5. 兩條路徑除了共用端點外不交叉（topologically not crossing）
    6. 物理分離：垂直 ≥ 20m  OR  水平峰值繞行差 ≥ 30m
       (符合 demo 雙重保險：altitude or horizontal separation)

額外驗證：
    7. 無連續重複航點
    8. FW 每三點構成的局部圓弧半徑 ≥ 0.85 × R_min
       (容許 fillet 縮減處最多 15% 誤差)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Tuple

# 把 project root 加入 sys.path（測試從 tests/ 目錄執行）
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from shapely.geometry import LineString, Point, Polygon

from core.global_planner.heterogeneous_nfz_planner import (
    HeterogeneousNFZPlanner, PlannerConfig, Waypoint, _LocalProjection,
)


# ──────────────────────────────────────────────────────────────────────
#  驗證 helpers — 全部以 ENU 公尺座標執行（精準、與規劃器一致）
# ──────────────────────────────────────────────────────────────────────
def _wps_to_xy(wps: List[Waypoint], proj: _LocalProjection) -> List[Tuple[float, float]]:
    return [proj.to_metric(w.lat, w.lon) for w in wps]


def check_no_intrusion(
    path_xy: List[Tuple[float, float]], polygon_xy: List[Tuple[float, float]],
) -> Tuple[bool, str]:
    """路徑不能穿越多邊形「內部」（容許貼邊）。

    判斷：任一線段中點是否落在多邊形內部 (contains，嚴格內部，不含邊界)
    """
    if len(path_xy) < 2:
        return False, '路徑點數不足'
    poly = Polygon(polygon_xy)
    line = LineString(path_xy)
    # 直接檢查線段是否真的穿越多邊形內部
    intersection = line.intersection(poly)
    if intersection.is_empty:
        return True, '完全在外'
    # 求交集區域 — 若僅是端點接觸 (零長度 / 點)，OK
    if intersection.geom_type == 'Point':
        return True, '僅單點接觸'
    if intersection.geom_type == 'MultiPoint':
        return True, '多點接觸但無線段穿入'
    # 若是 LineString / MultiLineString → 確實有段穿入內部
    overlap_len = intersection.length if hasattr(intersection, 'length') else 0
    return False, f'穿入內部 {overlap_len:.1f}m'


def check_buffer_respected(
    path_xy: List[Tuple[float, float]],
    polygon_xy: List[Tuple[float, float]],
    buffer_m: float,
    tolerance_m: float = 1.0,
) -> Tuple[bool, str]:
    """路徑與 buffered NFZ 的安全距離應 >= 0（可貼邊）。

    用 polygon.buffer 後再檢查穿入；如果穿入深度 > tolerance_m 視為違反。
    """
    poly = Polygon(polygon_xy)
    buffered = poly.buffer(buffer_m)
    line = LineString(path_xy)
    intersection = line.intersection(buffered)
    if intersection.is_empty:
        return True, '路徑完全在 buffer 外'
    # 計算「穿入」深度 — 最大向 buffered 內部的距離
    # 線段內部到 buffered 邊界的最大距離
    max_depth = 0.0
    for x, y in path_xy:
        pt = Point(x, y)
        if buffered.contains(pt):
            # 距離 buffered 邊界 (外緣)
            d = buffered.boundary.distance(pt)
            max_depth = max(max_depth, d)
    if max_depth > tolerance_m:
        return False, f'穿入 {max_depth:.1f}m (容許 {tolerance_m}m)'
    return True, f'貼邊 (深度 {max_depth:.2f}m ≤ {tolerance_m}m)'


def check_paths_dont_cross(
    mr_xy: List[Tuple[float, float]],
    fw_xy: List[Tuple[float, float]],
    vertical_gap_m: float,
    endpoint_tol: float = 1.0,
) -> Tuple[bool, str]:
    """兩條 2D 路徑除了共用端點外不應「意外」交叉。

    交叉類型：
      • 重疊（共線同方向）→ 高度層分開 ≥20m 就 OK（如同高速公路上下層車道）
      • X 形交叉（非端點點接觸）→ 失敗，存在碰撞風險
      • 僅端點接觸 → OK
    """
    if len(mr_xy) < 2 or len(fw_xy) < 2:
        return True, '路徑點不足，跳過'
    mr_line = LineString(mr_xy)
    fw_line = LineString(fw_xy)
    inter = mr_line.intersection(fw_line)
    if inter.is_empty:
        return True, '完全分離'

    # 重疊類型：共線
    if inter.geom_type in ('LineString', 'MultiLineString'):
        overlap_len = inter.length
        if vertical_gap_m >= 20.0:
            return True, (
                f'共線重疊 {overlap_len:.0f}m 但高度層差 '
                f'{vertical_gap_m:.0f}m 安全'
            )
        return False, f'共線重疊 {overlap_len:.0f}m 且垂直分離不足'

    # 點交叉類型
    if inter.geom_type == 'Point':
        pts = [inter]
    elif inter.geom_type == 'MultiPoint':
        pts = list(inter.geoms)
    elif inter.geom_type == 'GeometryCollection':
        # 混合：先看有沒有 LineString
        for g in inter.geoms:
            if g.geom_type in ('LineString', 'MultiLineString'):
                if vertical_gap_m >= 20.0:
                    return True, (
                        f'混合幾何含重疊 {g.length:.0f}m '
                        f'但高度差 {vertical_gap_m:.0f}m 安全'
                    )
                return False, f'混合交集含重疊 {g.length:.0f}m'
        pts = [g for g in inter.geoms if g.geom_type == 'Point']
    else:
        return False, f'未預期幾何 {inter.geom_type}'

    endpoints = [mr_xy[0], mr_xy[-1], fw_xy[0], fw_xy[-1]]
    non_endpoint = []
    for p in pts:
        is_ep = any(
            math.hypot(p.x - ex, p.y - ey) < endpoint_tol
            for ex, ey in endpoints
        )
        if not is_ep:
            non_endpoint.append((p.x, p.y))

    if non_endpoint:
        return False, f'X-形非端點交叉 {len(non_endpoint)} 處'
    return True, f'僅端點接觸 ({len(pts)} 點)'


def check_no_duplicate_consecutive(
    wps: List[Waypoint], min_dist_m: float = 0.5, proj: _LocalProjection = None,
) -> Tuple[bool, str]:
    """連續航點間距離 >= min_dist_m（避免飛控 disarm）。"""
    if proj is None:
        proj = _LocalProjection(wps[0].lat, wps[0].lon)
    xy = _wps_to_xy(wps, proj)
    for i in range(1, len(xy)):
        d = math.hypot(xy[i][0] - xy[i-1][0], xy[i][1] - xy[i-1][1])
        if d < min_dist_m:
            return False, f'wp[{i-1}]→wp[{i}] 距離 {d:.3f}m < {min_dist_m}m'
    return True, f'所有連續間距 ≥ {min_dist_m}m'


def check_fw_turning_radius(
    fw_xy: List[Tuple[float, float]], r_min: float, tolerance: float = 0.85,
) -> Tuple[bool, str]:
    """固定翼路徑：每三連續點構成的局部曲率半徑 >= r_min × tolerance。

    公式：三角形外接圓半徑 R = abc / (4·Area)
    """
    if len(fw_xy) < 3:
        return True, '路徑點不足 3，跳過'
    violations = []
    threshold = r_min * tolerance
    for i in range(1, len(fw_xy) - 1):
        A, B, C = fw_xy[i-1], fw_xy[i], fw_xy[i+1]
        ab = math.hypot(B[0]-A[0], B[1]-A[1])
        bc = math.hypot(C[0]-B[0], C[1]-B[1])
        ac = math.hypot(C[0]-A[0], C[1]-A[1])
        if ab < 0.5 or bc < 0.5 or ac < 0.5:
            continue
        # cross product = 2 × 面積
        cross = (B[0]-A[0])*(C[1]-A[1]) - (B[1]-A[1])*(C[0]-A[0])
        area = abs(cross) / 2.0
        if area < 0.5:
            # 共線（直線段）— 半徑無窮大，OK
            continue
        r = ab * bc * ac / (4.0 * area)
        if r < threshold:
            violations.append((i, r, A, B, C))
    if violations:
        worst = min(violations, key=lambda v: v[1])
        return False, (
            f'{len(violations)} 處違反 R_min（最小 R={worst[1]:.1f}m '
            f'@ wp[{worst[0]}]，門檻 {threshold:.1f}m）'
        )
    return True, '所有局部圓弧半徑 ≥ {:.1f}m (={:.0f}% R_min)'.format(
        threshold, tolerance * 100
    )


def check_physical_separation(
    routes, cfg: PlannerConfig,
) -> Tuple[bool, str]:
    """物理分離雙重保險：垂直 ≥ 20m  OR  水平峰值繞行差 ≥ 30m"""
    mr = routes['multirotor']
    fw = routes['fixed_wing']
    v_gap = abs(cfg.alt_fw - cfg.alt_mr)
    if v_gap >= 20.0:
        return True, f'垂直 {v_gap:.0f}m ≥ 20m'

    # 水平：計算各自最大繞行偏離點之間的距離
    proj = _LocalProjection(mr[0].lat, mr[0].lon)
    mr_xy = _wps_to_xy(mr, proj)
    fw_xy = _wps_to_xy(fw, proj)
    chord = LineString([mr_xy[0], mr_xy[-1]])

    def peak(path):
        if len(path) < 3:
            return path[len(path)//2]
        best_d = -1.0; best_p = path[1]
        for p in path[1:-1]:
            d = chord.distance(Point(p))
            if d > best_d:
                best_d = d; best_p = p
        return best_p

    p_mr = peak(mr_xy)
    p_fw = peak(fw_xy)
    h = math.hypot(p_mr[0]-p_fw[0], p_mr[1]-p_fw[1])
    if h >= 30.0:
        return True, f'水平峰值差 {h:.0f}m ≥ 30m'
    return False, f'垂直 {v_gap:.0f}m  水平 {h:.0f}m 均不足'


# ──────────────────────────────────────────────────────────────────────
#  測試場景
# ──────────────────────────────────────────────────────────────────────
SCENARIOS = [
    {
        'name': 'A. NFZ 正中阻擋（S→N 對穿）',
        'start': (23.700, 120.440),
        'target': (23.720, 120.440),
        'nfz': [(23.707, 120.435), (23.707, 120.445),
                (23.713, 120.445), (23.713, 120.435)],
    },
    {
        'name': 'B. NFZ 偏 E（部分阻擋斜線）',
        'start': (23.700, 120.420),
        'target': (23.720, 120.460),
        'nfz': [(23.707, 120.434), (23.707, 120.446),
                (23.713, 120.446), (23.713, 120.434)],
    },
    {
        'name': 'C. 大型 NFZ（2 km × 1 km）',
        'start': (23.685, 120.425),
        'target': (23.735, 120.475),
        'nfz': [(23.702, 120.435), (23.702, 120.465),
                (23.720, 120.465), (23.720, 120.435)],
    },
    {
        'name': 'D. 三角形 NFZ',
        'start': (23.700, 120.440),
        'target': (23.720, 120.440),
        'nfz': [(23.708, 120.430), (23.708, 120.450), (23.713, 120.440)],
    },
    {
        'name': 'E. NFZ 偏離路徑（不該繞行）',
        'start': (23.700, 120.440),
        'target': (23.720, 120.440),
        'nfz': [(23.707, 120.460), (23.707, 120.465),
                (23.713, 120.465), (23.713, 120.460)],
    },
    {
        'name': 'F. 長條形 NFZ（橫切）',
        'start': (23.700, 120.440),
        'target': (23.720, 120.440),
        'nfz': [(23.709, 120.420), (23.709, 120.460),
                (23.711, 120.460), (23.711, 120.420)],
    },
]


# ──────────────────────────────────────────────────────────────────────
#  Test runner
# ──────────────────────────────────────────────────────────────────────
def run_all() -> int:
    cfg = PlannerConfig(
        buffer_mr=20.0, buffer_fw=100.0,
        alt_mr=80.0, alt_fw=120.0,
        r_min=60.0, arc_segments=12, grid_res_m=10.0,
    )
    planner = HeterogeneousNFZPlanner(cfg)

    summary: list = []   # [(scenario_name, [(check_name, ok, msg), ...])]
    overall_pass = 0
    overall_fail = 0

    for sc in SCENARIOS:
        name = sc['name']
        print('\n' + '═' * 78)
        print(f' 場景：{name}')
        print('═' * 78)
        try:
            routes = planner.plan_heterogeneous_routes(
                sc['start'], sc['target'], sc['nfz'],
            )
        except Exception as e:
            print(f'  ❌ 規劃失敗：{e}')
            summary.append((name, [('規劃', False, str(e))]))
            overall_fail += 1
            continue

        mr_wps = routes['multirotor']
        fw_wps = routes['fixed_wing']
        print(f'  MR 路徑：{len(mr_wps)} 點   FW 路徑：{len(fw_wps)} 點')

        # 投影到本地 ENU 做檢查（與規劃器內部一致）
        lat0 = (sc['start'][0] + sc['target'][0]) / 2.0
        lon0 = (sc['start'][1] + sc['target'][1]) / 2.0
        proj = _LocalProjection(lat0, lon0)
        nfz_xy = proj.project_polygon(sc['nfz'])
        mr_xy = _wps_to_xy(mr_wps, proj)
        fw_xy = _wps_to_xy(fw_wps, proj)

        # ── 執行所有檢查 ──
        checks = []
        checks.append(('MR 不穿越原 NFZ',
                       *check_no_intrusion(mr_xy, nfz_xy)))
        checks.append(('FW 不穿越原 NFZ',
                       *check_no_intrusion(fw_xy, nfz_xy)))
        checks.append(('MR 尊重 buffer_mr',
                       *check_buffer_respected(mr_xy, nfz_xy, cfg.buffer_mr)))
        checks.append(('FW 尊重 buffer_fw',
                       *check_buffer_respected(fw_xy, nfz_xy, cfg.buffer_fw)))
        v_gap = abs(cfg.alt_fw - cfg.alt_mr)
        checks.append(('兩條路徑不交叉',
                       *check_paths_dont_cross(mr_xy, fw_xy, v_gap)))
        checks.append(('物理分離雙保險',
                       *check_physical_separation(routes, cfg)))
        checks.append(('MR 無連續重複航點',
                       *check_no_duplicate_consecutive(mr_wps, proj=proj)))
        checks.append(('FW 無連續重複航點',
                       *check_no_duplicate_consecutive(fw_wps, proj=proj)))
        checks.append(('FW R_min 約束',
                       *check_fw_turning_radius(fw_xy, cfg.r_min)))

        # 印出結果
        for chk_name, ok, msg in checks:
            mark = '✓' if ok else '✗'
            print(f'    [{mark}] {chk_name:<22s} — {msg}')
            if ok:
                overall_pass += 1
            else:
                overall_fail += 1
        summary.append((name, checks))

    # ── 最終摘要 ──
    print('\n\n' + '═' * 78)
    print(' 最終摘要')
    print('═' * 78)
    for name, checks in summary:
        n_pass = sum(1 for _, ok, _ in checks if ok)
        n_total = len(checks)
        status = '✓ PASS' if n_pass == n_total else f'✗ FAIL ({n_total - n_pass}/{n_total})'
        print(f'  {name:<35s}  {n_pass}/{n_total}  {status}')
    print('-' * 78)
    print(f'  總計：{overall_pass} 通過 / {overall_pass + overall_fail} '
          f'({100 * overall_pass / max(1, overall_pass + overall_fail):.1f}%)')
    print('═' * 78)
    return 0 if overall_fail == 0 else 1


if __name__ == '__main__':
    sys.exit(run_all())
