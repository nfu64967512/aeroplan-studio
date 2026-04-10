"""
多無人機動態旅行商問題 (MDTSP) 求解器
==========================================
對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas》之核心演算法：

- Section 2.3.4: 路徑長度矩陣 D_{i,k} 與決策矩陣 E_{i,k} (Eq. 13-14)
- Section 2.3.4: MDTSP 優化模型 (Eq. 17-26)
- Algorithm 1: 多區域 UAV 分配 — 貪心演算法 (Eq. 30-31)
- Algorithm 2: 改進動態規劃 IDP (Eq. 32-34)
- Algorithm 3: 高度下降演算法 (Eq. 35-39) 含 GDA 平滑

同時支援多旋翼（自由轉向）與固定翼（Dubins 曲線約束）。

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
import itertools
from dataclasses import dataclass, field
from typing import (
    List, Tuple, Optional, Dict, Any, Sequence, Callable,
)
from enum import Enum, auto

from mission.coverage_path import CoveragePath, OperationSegment


# ============================================================
# 列舉與基礎資料結構
# ============================================================

class VehicleType(Enum):
    """載具類型"""
    MULTIROTOR = auto()
    FIXED_WING = auto()


@dataclass
class UAVState:
    """
    無人機狀態，對應論文 S_i = (x_i, y_i, χ_i, r_i)

    Attributes:
        uav_id   : 無人機 ID
        position : 當前位置 (lat, lon)
        heading  : 航向角（度，0=北，順時針為正）
        turn_radius : 最小轉彎半徑（公尺）；多旋翼設 0
        speed    : 巡航速度（m/s）
        vehicle_type : 載具類型
        fov_width: FOV 地面覆蓋寬度（公尺）
    """
    uav_id: int
    position: Tuple[float, float]
    heading: float = 0.0
    turn_radius: float = 0.0
    speed: float = 15.0
    vehicle_type: VehicleType = VehicleType.MULTIROTOR
    fov_width: float = 100.0


@dataclass
class PathTransition:
    """
    路徑轉移記錄

    表示無人機從一條作業路徑到另一條的轉移資訊，
    對應論文中 d^{trans}_{i,k,l,l+s}。
    """
    from_op_idx: int        # 起始作業路徑編號
    to_op_idx: int          # 目標作業路徑編號
    distance: float         # 轉移距離（公尺）
    entry_direction: int    # 進入方向：+1=從左到右, -1=從右到左


@dataclass
class MDTSPResult:
    """
    MDTSP 求解結果

    Attributes:
        area_id          : 區域 ID
        uav_assignments  : {uav_id: [OperationSegment 有序列表]}
        path_matrix      : 路徑長度矩陣 D_{i,k}
        decision_matrix  : 決策矩陣 E_{i,k}
        total_distances  : {uav_id: 總路徑長度}
        makespan         : 最大完工時間（秒）
    """
    area_id: int
    uav_assignments: Dict[int, List[OperationSegment]]
    path_matrix: Optional[Dict[int, List[List[float]]]] = None
    decision_matrix: Optional[Dict[int, List[List[int]]]] = None
    total_distances: Dict[int, float] = field(default_factory=dict)
    makespan: float = 0.0
    assembled_paths: Optional[Dict[int, Any]] = None  # {uav_id: AssembledPath}


# ============================================================
# 距離計算工具
# ============================================================

def _haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Haversine 距離（公尺）"""
    R = 6_371_000.0
    lat1, lat2 = math.radians(p1[0]), math.radians(p2[0])
    dlat = lat2 - lat1
    dlon = math.radians(p2[1] - p1[1])
    a = (math.sin(dlat / 2) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    return R * 2.0 * math.asin(min(1.0, math.sqrt(a)))


def _dubins_estimate(
    p1: Tuple[float, float], h1: float,
    p2: Tuple[float, float], h2: float,
    r_min: float,
) -> float:
    """
    估算 Dubins 路徑長度（近似值）。

    精確計算需要呼叫 DubinsTrajectoryGenerator，此處用解析近似
    以加速矩陣建構。公式：直線距離 + 額外轉彎補償。
    """
    d = _haversine(p1, p2)
    if r_min <= 0 or d <= 0:
        return d
    # 航向差的轉彎補償
    dh = abs(h2 - h1) % 360
    if dh > 180:
        dh = 360 - dh
    # 估算額外弧長 ≈ R * |Δθ|（弧度）
    extra = r_min * math.radians(dh)
    # 當直線距離 < 2R 時，需要額外的 bulb turn
    if d < 2 * r_min:
        extra += math.pi * r_min - d
    return max(d, d + extra)


def _transit_distance(
    uav: UAVState,
    from_pos: Tuple[float, float], from_heading: float,
    to_pos: Tuple[float, float], to_heading: float,
) -> float:
    """
    計算轉移距離，自動根據載具類型選擇直線或 Dubins 估算。
    """
    if uav.vehicle_type == VehicleType.FIXED_WING and uav.turn_radius > 0:
        return _dubins_estimate(
            from_pos, from_heading, to_pos, to_heading, uav.turn_radius
        )
    return _haversine(from_pos, to_pos)


def _scan_heading(op: OperationSegment) -> float:
    """計算作業路徑的掃描航向（度）"""
    dlat = op.right_point[0] - op.left_point[0]
    dlon = op.right_point[1] - op.left_point[1]
    bearing = math.degrees(math.atan2(dlon, dlat)) % 360
    return bearing


# ============================================================
# 路徑長度矩陣與決策矩陣建構
# (論文 Eq. 13-14)
# ============================================================

class PathMatrixBuilder:
    """
    建構路徑長度矩陣 D_{i,k} 與決策矩陣 E_{i,k}。

    D_{i,k} 為 (n_{k,l}+1) × n_{k,l} 矩陣，
    第 0 列 = 從 UAV 初始位置到各作業路徑的進入距離（含方向），
    其餘列 = 從 L_{k,l} 到 L_{k,l+s} 的轉移距離。

    E_{i,k} 為對應的二元方向決策矩陣（+1 或 -1）。
    """

    @staticmethod
    def build(
        uav: UAVState,
        coverage: CoveragePath,
    ) -> Tuple[List[List[float]], List[List[int]]]:
        """
        建構路徑長度矩陣 D_{i,k} 和決策矩陣 E_{i,k}

        參數:
            uav       : 無人機狀態
            coverage  : 區域覆蓋路徑

        返回:
            (D_matrix, E_matrix)
            D_matrix[t][l] = 從狀態 t 到作業路徑 l 的距離
            E_matrix[t][l] = 對應的進入方向 (+1 = L→R, -1 = R→L)
        """
        ops = coverage.operations
        n = len(ops)
        if n == 0:
            return [], []

        # D[0][l] = 從 UAV 初始位置到 L_{k,l} 的進入距離
        # D[t+1][l] = 從 L_{k,t} 到 L_{k,l} 的轉移距離
        D = [[0.0] * n for _ in range(n + 1)]
        E = [[0] * n for _ in range(n + 1)]

        # ── 第 0 列：進入路徑（Entry Path, Eq. 11）──────────
        for l in range(n):
            op = ops[l]
            h_op = _scan_heading(op)
            # 計算從左端點進入 vs 從右端點進入
            d_left = _transit_distance(
                uav, uav.position, uav.heading,
                op.left_point, h_op
            )
            d_right = _transit_distance(
                uav, uav.position, uav.heading,
                op.right_point, (h_op + 180) % 360
            )
            if d_left <= d_right:
                D[0][l] = d_left + op.length_m
                E[0][l] = 1    # L→R
            else:
                D[0][l] = d_right + op.length_m
                E[0][l] = -1   # R→L

        # ── 第 1..n 列：轉移路徑（Transfer Path, Eq. 12）──
        for t in range(n):
            op_t = ops[t]
            h_t = _scan_heading(op_t)
            # 從 L_{k,t} 出口端點出發
            for l in range(n):
                if l == t:
                    D[t + 1][l] = float('inf')
                    E[t + 1][l] = 0
                    continue
                op_l = ops[l]
                h_l = _scan_heading(op_l)

                # 論文 Eq. 25：連續兩條作業路徑方向交替
                # 試兩種出口→入口組合，取最短
                combos = [
                    (op_t.right_point, h_t, op_l.left_point, h_l, 1),
                    (op_t.right_point, h_t, op_l.right_point, (h_l + 180) % 360, -1),
                    (op_t.left_point, (h_t + 180) % 360, op_l.left_point, h_l, 1),
                    (op_t.left_point, (h_t + 180) % 360, op_l.right_point, (h_l + 180) % 360, -1),
                ]
                best_d = float('inf')
                best_e = 1
                for from_p, from_h, to_p, to_h, direction in combos:
                    d = _transit_distance(uav, from_p, from_h, to_p, to_h)
                    d_total = d + op_l.length_m
                    if d_total < best_d:
                        best_d = d_total
                        best_e = direction
                D[t + 1][l] = best_d
                E[t + 1][l] = best_e

        return D, E


# ============================================================
# 多區域 UAV 分配 — 貪心演算法
# (論文 Algorithm 1, Eq. 30-31)
# ============================================================

class GreedyAllocator:
    """
    多區域多 UAV 分配器。

    基於論文 Eq. 30 的加權評估函數 J_k，
    以貪心策略依序為每個區域分配最適當數量的 UAV。

    加權因子（預設均 0.25）：
        ω₁: 最短進入路徑距離成本
        ω₂: 作業路徑數量成本
        ω₃: 作業路徑長度成本
        ω₄: 區域威脅等級成本
    """

    def __init__(
        self,
        w1: float = 0.25,
        w2: float = 0.25,
        w3: float = 0.25,
        w4: float = 0.25,
    ):
        self.weights = (w1, w2, w3, w4)

    def evaluate_area(
        self,
        uavs: List[UAVState],
        coverage: CoveragePath,
    ) -> float:
        """
        計算區域評估指標 J_k（論文 Eq. 30）。

        J_k = ω₁·(Σ min entry dist) / n_U
            + ω₂·(n_{k,l} / Σ n_{k,l})
            + ω₃·(Σ|op_length|) / (Σ all op_length)
            + ω₄·(val_k / Σ val_j)
        """
        w1, w2, w3, w4 = self.weights
        ops = coverage.operations
        n_ops = len(ops)
        if n_ops == 0:
            return float('inf')

        # ω₁: 最短進入路徑距離
        entry_dists = []
        for uav in uavs:
            min_d = min(
                min(_haversine(uav.position, op.left_point),
                    _haversine(uav.position, op.right_point))
                for op in ops
            )
            entry_dists.append(min_d)
        avg_entry = sum(entry_dists) / max(len(entry_dists), 1)

        # ω₂: 作業路徑數量
        count_cost = n_ops

        # ω₃: 作業路徑總長度
        total_op_len = sum(op.length_m for op in ops)

        # ω₄: 區域優先級（威脅等級）
        val_k = coverage.priority

        J_k = (w1 * avg_entry
               + w2 * count_cost
               + w3 * total_op_len
               + w4 * val_k)
        return J_k

    def allocate(
        self,
        uavs: List[UAVState],
        coverages: List[CoveragePath],
    ) -> Dict[int, List[int]]:
        """
        貪心分配：為每個區域分配 UAV 數量（論文 Algorithm 1, Eq. 31）。

        ℓ_k = round(n_U × J_k / Σ J_j)

        參數:
            uavs      : 所有可用 UAV
            coverages : 所有區域的覆蓋路徑

        返回:
            {area_id: [uav_id, ...]} 分配結果
        """
        n_U = len(uavs)
        n_Z = len(coverages)
        if n_U == 0 or n_Z == 0:
            return {}

        # 步驟 1：計算各區域 J_k
        Jk_values = [self.evaluate_area(uavs, cp) for cp in coverages]
        J_sum = sum(Jk_values) or 1.0

        # 步驟 2：按 J_k 從大到小排序區域
        area_order = sorted(range(n_Z), key=lambda k: Jk_values[k], reverse=True)

        # 步驟 3：依公式分配 UAV 數量，保證至少 1 架
        ell = [0] * n_Z
        assigned_total = 0
        for k in area_order:
            raw = n_U * Jk_values[k] / J_sum
            ell[k] = max(1, round(raw))
            assigned_total += ell[k]

        # 修正：若分配總數超出，從 J_k 最小的區域減少
        while assigned_total > n_U:
            for k in reversed(area_order):
                if ell[k] > 1 and assigned_total > n_U:
                    ell[k] -= 1
                    assigned_total -= 1
        # 若不足，從 J_k 最大的區域增加
        while assigned_total < n_U:
            for k in area_order:
                if assigned_total < n_U:
                    ell[k] += 1
                    assigned_total += 1

        # 步驟 4：為每個區域選擇最近的 ℓ_k 架 UAV
        available = list(range(n_U))
        result: Dict[int, List[int]] = {}

        for k in area_order:
            cp = coverages[k]
            center = _polygon_centroid(cp.area_polygon) if cp.area_polygon else (0, 0)
            # 按距離排序可用 UAV
            available.sort(
                key=lambda i: _haversine(uavs[i].position, center)
            )
            chosen = available[:ell[k]]
            result[cp.area_id] = [uavs[i].uav_id for i in chosen]
            available = available[ell[k]:]

        return result


def _polygon_centroid(
    polygon: List[Tuple[float, float]],
) -> Tuple[float, float]:
    """計算多邊形質心"""
    if not polygon:
        return (0.0, 0.0)
    lat = sum(p[0] for p in polygon) / len(polygon)
    lon = sum(p[1] for p in polygon) / len(polygon)
    return (lat, lon)


# ============================================================
# 改進動態規劃 (IDP) 求解器
# (論文 Algorithm 2, Eq. 32-34)
# ============================================================

class IDPSolver:
    """
    改進動態規劃 (Improved Dynamic Programming) 求解 MDTSP。

    核心思路：
    - 將 MDTSP 分解為子問題：每次狀態轉移時，
      從當前作業路徑集合 L^{CUR}_k(s) 中選擇最小成本的下一步
    - 狀態方程 (Eq. 34)：
      dp(S^{CUR}_k(s+1), L^{NAD}_k - L^{SEL}_k(s))
        = Σ W(S_i(s), L_{k,l_{s+1}})
        + dp(S^{CUR}_k(s), L^{NAD}_k(s))
    - W(S_i(s), L_{k,l_{s+1}}) = |d^{trans} + d^{oper}| / dis_{i,k}(s)  (Eq. 33)

    支援多旋翼（Haversine 直線距離）和固定翼（Dubins 估算距離）。
    """

    def __init__(self, max_permutation_size: int = 6):
        """
        參數:
            max_permutation_size: 當候選集合 > 此值時改用啟發式剪枝
        """
        self._max_perm = max_permutation_size

    def solve(
        self,
        uavs: List[UAVState],
        coverage: CoveragePath,
    ) -> MDTSPResult:
        """
        求解單區域 MDTSP。

        參數:
            uavs     : 分配給此區域的 UAV 列表（ℓ_k 架）
            coverage : 區域覆蓋路徑

        返回:
            MDTSPResult 含各 UAV 的有序作業路徑分配
        """
        ops = coverage.operations
        n = len(ops)
        ell = len(uavs)
        if n == 0 or ell == 0:
            return MDTSPResult(
                area_id=coverage.area_id,
                uav_assignments={u.uav_id: [] for u in uavs},
            )

        # ── 建構路徑矩陣（每架 UAV 獨立）─────────────────
        D_all: Dict[int, List[List[float]]] = {}
        E_all: Dict[int, List[List[int]]] = {}
        for uav in uavs:
            D, E = PathMatrixBuilder.build(uav, coverage)
            D_all[uav.uav_id] = D
            E_all[uav.uav_id] = E

        # ── 初始化 IDP 狀態（Eq. 32）───────────────────────
        # R_{i,k}: 各 UAV 的路徑轉移結果序列
        R: Dict[int, List[int]] = {u.uav_id: [] for u in uavs}
        # dis_{i}: 各 UAV 已累計路徑長度
        dis: Dict[int, float] = {u.uav_id: 0.0 for u in uavs}
        # 各 UAV 當前狀態（位置 + 航向）
        states: Dict[int, UAVState] = {u.uav_id: u for u in uavs}

        # L^{NAD}_k: 尚未分配的作業路徑索引集合
        L_nad: set = set(range(n))
        # S^{CUR}_k: 各 UAV 當前所在的狀態索引（-1 = 初始位置）
        S_cur: Dict[int, int] = {u.uav_id: -1 for u in uavs}

        s = 0  # 狀態轉移計數

        # ── 主迴圈：直到所有作業路徑都被分配 ────────────
        while L_nad:
            s += 1
            # 對每架 UAV 選擇下一條最佳作業路徑
            candidates = list(L_nad)
            n_cand = len(candidates)

            if n_cand <= ell:
                # 剩餘路徑 ≤ UAV 數量：每架 UAV 各取一條
                sorted_cands = sorted(candidates)
                assignments_this_step = {}
                remaining_uavs = list(uavs)
                for l_idx in sorted_cands:
                    best_uav = min(
                        remaining_uavs,
                        key=lambda u: self._transition_cost(
                            u.uav_id, S_cur[u.uav_id], l_idx,
                            D_all, dis
                        ),
                    )
                    assignments_this_step[best_uav.uav_id] = l_idx
                    remaining_uavs.remove(best_uav)
                # 執行分配
                for uid, l_idx in assignments_this_step.items():
                    R[uid].append(l_idx)
                    dis[uid] += D_all[uid][S_cur[uid] + 1][l_idx]
                    S_cur[uid] = l_idx
                    L_nad.discard(l_idx)
            else:
                # 候選集合較大：使用 IDP 狀態轉移
                if n_cand <= self._max_perm and ell <= 4:
                    # 精確：嘗試所有 ℓ_k 元排列，選最小成本
                    best_assign, best_cost = self._exact_assignment(
                        uavs, candidates, S_cur, D_all, dis
                    )
                else:
                    # 啟發式：每架 UAV 貪心選取最小 W 值的路徑
                    best_assign = self._greedy_assignment(
                        uavs, candidates, S_cur, D_all, dis
                    )

                for uid, l_idx in best_assign.items():
                    R[uid].append(l_idx)
                    dis[uid] += D_all[uid][S_cur[uid] + 1][l_idx]
                    S_cur[uid] = l_idx
                    L_nad.discard(l_idx)

        # ── 組裝結果 ──────────────────────────────────────
        assignments: Dict[int, List[OperationSegment]] = {}
        for uav in uavs:
            uid = uav.uav_id
            assignments[uid] = [ops[i] for i in R[uid]]

        return MDTSPResult(
            area_id=coverage.area_id,
            uav_assignments=assignments,
            path_matrix=D_all,
            decision_matrix=E_all,
            total_distances=dict(dis),
            makespan=max(
                (dis[u.uav_id] / max(u.speed, 0.1) for u in uavs),
                default=0.0,
            ),
        )

    # ── 狀態轉移成本 W (Eq. 33) ───────────────────────────

    def _transition_cost(
        self,
        uav_id: int,
        s_cur: int,         # 當前狀態索引（-1=初始位置）
        l_next: int,        # 候選作業路徑索引
        D_all: Dict[int, List[List[float]]],
        dis: Dict[int, float],
    ) -> float:
        """
        計算轉移機率/成本 W(S_i(s), L_{k,l_{s+1}})（論文 Eq. 33）

        W = |d^{trans}_{i,k,l_s,l_{s+1}} + d^{oper}_{i,k,l_{s+1}}| / dis_{i,k}(s)

        dis_{i,k}(s) = 0 時退化為純距離。
        """
        D = D_all[uav_id]
        row = s_cur + 1  # D 的第 0 列對應初始位置
        d_total = D[row][l_next]
        if d_total == float('inf'):
            return float('inf')
        total_so_far = dis[uav_id]
        if total_so_far > 0:
            return d_total / total_so_far
        return d_total

    # ── 精確分配（小規模排列枚舉）────────────────────────

    def _exact_assignment(
        self,
        uavs: List[UAVState],
        candidates: List[int],
        S_cur: Dict[int, int],
        D_all: Dict[int, List[List[float]]],
        dis: Dict[int, float],
    ) -> Tuple[Dict[int, int], float]:
        """對 ℓ_k 架 UAV 嘗試所有候選路徑的排列，選最小成本分配"""
        ell = len(uavs)
        best_cost = float('inf')
        best_assign: Dict[int, int] = {}

        for perm in itertools.permutations(candidates, ell):
            cost = 0.0
            for i, uav in enumerate(uavs):
                uid = uav.uav_id
                c = self._transition_cost(uid, S_cur[uid], perm[i], D_all, dis)
                cost += c
                if cost >= best_cost:
                    break
            if cost < best_cost:
                best_cost = cost
                best_assign = {uavs[i].uav_id: perm[i] for i in range(ell)}

        return best_assign, best_cost

    # ── 啟發式分配（貪心）──────────────────────────────

    def _greedy_assignment(
        self,
        uavs: List[UAVState],
        candidates: List[int],
        S_cur: Dict[int, int],
        D_all: Dict[int, List[List[float]]],
        dis: Dict[int, float],
    ) -> Dict[int, int]:
        """每架 UAV 貪心選取成本最低的候選路徑"""
        used: set = set()
        result: Dict[int, int] = {}

        # 按 UAV 距離最短的候選排序
        uav_order = sorted(
            uavs,
            key=lambda u: min(
                self._transition_cost(u.uav_id, S_cur[u.uav_id], c, D_all, dis)
                for c in candidates
            ),
        )

        for uav in uav_order:
            uid = uav.uav_id
            best_l = -1
            best_c = float('inf')
            for c in candidates:
                if c in used:
                    continue
                cost = self._transition_cost(uid, S_cur[uid], c, D_all, dis)
                if cost < best_c:
                    best_c = cost
                    best_l = c
            if best_l >= 0:
                result[uid] = best_l
                used.add(best_l)

        return result


# ============================================================
# 高度下降演算法 + GDA 平滑
# (論文 Algorithm 3, Eq. 35-39)
# ============================================================

class AltitudePlanner:
    """
    2D → 3D 路徑高度規劃器。

    步驟：
    1. 沿 2D 路徑以均勻步長 δd 插值離散點 P^{psd}_{i,k}
    2. 利用雙線性插值從 DEM（G_lla）取得地面高度 h^{psd}_{i,k,j}
    3. 根據覆蓋高度約束 h^{cov}_k 計算期望 Z 座標 z^{psd}_{i,k,j}（Eq. 35）
    4. 依爬升角約束迭代修正實際 Z 座標（Eq. 36）
    5. 使用 GDA（Gradient Descent Algorithm）平滑高度曲線（Eq. 37-38）
    6. 刪除共線冗餘點（Eq. 39）
    """

    def __init__(
        self,
        coverage_altitude: float = 100.0,
        climb_angle_max_deg: float = 15.0,
        step_distance_m: float = 50.0,
        gda_c1: float = 1.0,
        gda_c2: float = 0.5,
        gda_iterations: int = 50,
        gda_learning_rate: float = 0.01,
        collinear_epsilon: float = 0.01,
        earth_radius: float = 6_371_000.0,
    ):
        """
        參數:
            coverage_altitude  : 覆蓋高度 h^{cov}_k（公尺）
            climb_angle_max_deg: 最大爬升/下降角（度）
            step_distance_m    : 路徑離散步長 δd（公尺）
            gda_c1             : GDA 距離因子
            gda_c2             : GDA 平滑因子
            gda_iterations     : GDA 迭代次數
            gda_learning_rate  : GDA 學習率
            collinear_epsilon  : 共線判定閾值
            earth_radius       : 地球半徑
        """
        self.h_cov = coverage_altitude
        self.gamma_max = math.radians(climb_angle_max_deg)
        self.delta_d = step_distance_m
        self.c1 = gda_c1
        self.c2 = gda_c2
        self.gda_iter = gda_iterations
        self.gda_lr = gda_learning_rate
        self.eps = collinear_epsilon
        self.R0 = earth_radius

    def plan_altitude(
        self,
        path_2d: List[Tuple[float, float]],
        terrain_func: Optional[Callable[[float, float], float]] = None,
    ) -> List[Tuple[float, float, float]]:
        """
        將 2D 路徑升維為 3D 路徑。

        參數:
            path_2d      : 2D 路徑 [(lat, lon), ...]
            terrain_func : 地形查詢函數 (lat, lon) → 地面海拔（公尺）；
                           None 時假設地面海拔 = 0

        返回:
            3D 路徑 [(lat, lon, alt), ...]
        """
        if len(path_2d) < 2:
            return [(p[0], p[1], self.h_cov) for p in path_2d]

        # ── 步驟 1：均勻插值離散點 ────────────────────────
        discrete_pts = self._interpolate_path(path_2d)
        n = len(discrete_pts)

        # ── 步驟 2：查詢地面高度 ──────────────────────────
        ground_h = [0.0] * n
        if terrain_func is not None:
            for i, (lat, lon) in enumerate(discrete_pts):
                try:
                    ground_h[i] = terrain_func(lat, lon)
                except Exception:
                    ground_h[i] = 0.0

        # ── 步驟 3：期望 Z 座標（Eq. 35）─────────────────
        desired_z = [0.0] * n
        for i in range(n):
            lat, lon = discrete_pts[i]
            # 地球曲率補償
            rho = math.sqrt(
                (lat - discrete_pts[0][0]) ** 2
                + (lon - discrete_pts[0][1]) ** 2
            ) * 111111.0
            delta_z = abs(rho) - self.R0
            # z^{psd} = h^{cov} - Δz（曲率修正通常可忽略）
            desired_z[i] = self.h_cov + ground_h[i] - max(0, delta_z)

        # ── 步驟 4：爬升角約束修正（Eq. 36）───────────────
        actual_z = list(desired_z)
        delta_h_max = self.delta_d * math.tan(self.gamma_max)

        # 正向掃描
        for j in range(1, n):
            diff = actual_z[j] - actual_z[j - 1]
            if abs(diff) > delta_h_max:
                actual_z[j] = actual_z[j - 1] + math.copysign(delta_h_max, diff)

        # 反向掃描（確保雙向一致）
        for j in range(n - 2, -1, -1):
            diff = actual_z[j] - actual_z[j + 1]
            if abs(diff) > delta_h_max:
                actual_z[j] = actual_z[j + 1] + math.copysign(delta_h_max, diff)

        # ── 步驟 5：GDA 平滑（Eq. 37-38）─────────────────
        smoothed = list(actual_z)
        for _iteration in range(self.gda_iter):
            grad = [0.0] * n
            for j in range(1, n - 1):
                # J(h) = c1 |h_j - h_{j-1}| + c2 |h_{j+1} - 2h_j + h_{j-1}|
                # ∂J/∂h_j ≈ c1·sign(h_j - h_{j-1}) + 2·c2·(2h_j - h_{j-1} - h_{j+1})
                d1 = smoothed[j] - smoothed[j - 1]
                d2 = smoothed[j + 1] - 2 * smoothed[j] + smoothed[j - 1]
                sign_d1 = (1.0 if d1 > 0 else (-1.0 if d1 < 0 else 0.0))
                grad[j] = self.c1 * sign_d1 + 2.0 * self.c2 * (2.0 * smoothed[j]
                           - smoothed[j - 1] - smoothed[j + 1])

            # 更新（固定端點）
            max_change = 0.0
            for j in range(1, n - 1):
                delta = -self.gda_lr * grad[j]
                smoothed[j] += delta
                max_change = max(max_change, abs(delta))
                # 保持不低於地面高度 + 安全餘量
                smoothed[j] = max(smoothed[j], ground_h[j] + 10.0)

            # 收斂判定
            if max_change < 0.01:
                break

        # ── 步驟 6：刪除共線冗餘點（Eq. 39）───────────────
        result = self._reduce_collinear(discrete_pts, smoothed)
        return result

    # ── 輔助方法 ──────────────────────────────────────────

    def _interpolate_path(
        self,
        path: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """以均勻步長 δd 沿路徑插值離散點"""
        result = [path[0]]
        accumulated = 0.0
        for i in range(1, len(path)):
            seg_len = _haversine(path[i - 1], path[i])
            if seg_len < 1e-3:
                continue
            num_steps = max(1, int(seg_len / self.delta_d))
            for s in range(1, num_steps + 1):
                t = s / num_steps
                lat = path[i - 1][0] + t * (path[i][0] - path[i - 1][0])
                lon = path[i - 1][1] + t * (path[i][1] - path[i - 1][1])
                result.append((lat, lon))
        return result

    def _reduce_collinear(
        self,
        pts: List[Tuple[float, float]],
        alts: List[float],
    ) -> List[Tuple[float, float, float]]:
        """
        刪除共線冗餘點（Eq. 39）。

        判定條件：
        |(P_{j} - P_{j-1}) × (P_{j+1} - P_{j})| /
        (|P_{j} - P_{j-1}| · |P_{j+1} - P_{j}|) - 1 < ε
        """
        n = len(pts)
        if n <= 2:
            return [(pts[i][0], pts[i][1], alts[i]) for i in range(n)]

        keep = [True] * n
        for j in range(1, n - 1):
            # 3D 向量
            dx1 = pts[j][0] - pts[j - 1][0]
            dy1 = pts[j][1] - pts[j - 1][1]
            dz1 = alts[j] - alts[j - 1]
            dx2 = pts[j + 1][0] - pts[j][0]
            dy2 = pts[j + 1][1] - pts[j][1]
            dz2 = alts[j + 1] - alts[j]
            # 向量夾角餘弦
            dot = dx1 * dx2 + dy1 * dy2 + dz1 * dz2
            mag1 = math.sqrt(dx1 ** 2 + dy1 ** 2 + dz1 ** 2)
            mag2 = math.sqrt(dx2 ** 2 + dy2 ** 2 + dz2 ** 2)
            if mag1 > 1e-12 and mag2 > 1e-12:
                cos_angle = dot / (mag1 * mag2)
                if abs(cos_angle - 1.0) < self.eps:
                    keep[j] = False

        return [
            (pts[i][0], pts[i][1], alts[i])
            for i in range(n) if keep[i]
        ]


# ============================================================
# 整合介面：DCCPPSolver
# ============================================================

class DCCPPSolver:
    """
    動態完全覆蓋路徑規劃 (DCCPP) 完整求解器。

    整合 Algorithm 1（UAVTaskAllocator）+ Algorithm 2（IDP_Solver）
    + Algorithm 3（AltitudePlanner），提供從輸入到輸出的一站式 API。

    優先使用 core/dccpp/ 的 numpy 向量化實作；若導入失敗則退化為舊版。

    典型使用流程：
        solver = DCCPPSolver()
        results = solver.solve(uavs, coverages)
    """

    def __init__(
        self,
        allocator=None,
        idp_solver=None,
        altitude_planner=None,
        path_assembler: Optional[Any] = None,
        coordinated_planner: Optional[Any] = None,
    ):
        # 優先使用 core/dccpp/ 的 numpy 向量化實作
        try:
            from core.dccpp.task_allocator import UAVTaskAllocator as _Alloc
            from core.dccpp.idp_solver import IDP_Solver as _IDP
            from core.dccpp.altitude_planner import AltitudePlanner as _AltPlan
            self._use_new_dccpp = True
        except ImportError:
            _Alloc = GreedyAllocator
            _IDP = IDPSolver
            _AltPlan = AltitudePlanner
            self._use_new_dccpp = False

        self.allocator = allocator or _Alloc()
        self.idp = idp_solver or _IDP()
        self.alt_planner = altitude_planner or _AltPlan()
        self._path_assembler = path_assembler
        self._coordinated_planner = coordinated_planner

    def solve(
        self,
        uavs: List[UAVState],
        coverages: List[CoveragePath],
        terrain_func: Optional[Callable[[float, float], float]] = None,
        enable_altitude: bool = True,
        enable_dubins_assembly: bool = True,
        coordination_mode: str = "uncoordinated",
    ) -> Dict[int, MDTSPResult]:
        """
        完整求解多區域多 UAV DCCPP。

        步驟：
        1. 貪心分配 UAV 至各區域（Algorithm 1, Eq. 30-31）
        2. 對每個區域用 IDP 求解路徑序列（Algorithm 2, Eq. 33-34）
        3. 組裝含 Dubins 曲線的完整航點路徑
        4. 可選：對每架 UAV 的路徑進行高度規劃（Algorithm 3, Eq. 35-39）
        5. 可選：協調模式下同步各 UAV 進入時間

        參數:
            uavs                  : 所有 UAV 狀態
            coverages             : 所有區域覆蓋路徑
            terrain_func          : 地形查詢函數；None 表示平地
            enable_altitude       : 是否啟用高度規劃
            enable_dubins_assembly: 是否組裝 Dubins 曲線航點
            coordination_mode     : "uncoordinated" 或 "coordinated"

        返回:
            {area_id: MDTSPResult}
        """
        # ── 步驟 1：區域分配（Algorithm 1）────────────────
        alloc_result = self.allocator.allocate(uavs, coverages)

        # 相容新舊介面：新版返回 AllocationResult，舊版返回 Dict
        if hasattr(alloc_result, 'area_assignments'):
            allocation = alloc_result.area_assignments
        else:
            allocation = alloc_result

        # 建立 uav_id → UAVState 查找表
        uav_map = {u.uav_id: u for u in uavs}

        # ── 步驟 2 + 3 + 4：逐區域求解 ──────────────────
        results: Dict[int, MDTSPResult] = {}
        for cp in coverages:
            area_uav_ids = allocation.get(cp.area_id, [])
            area_uavs = [uav_map[uid] for uid in area_uav_ids if uid in uav_map]

            if not area_uavs:
                results[cp.area_id] = MDTSPResult(
                    area_id=cp.area_id,
                    uav_assignments={},
                )
                continue

            # IDP 求解（Algorithm 2）
            mdtsp_result = self.idp.solve(area_uavs, cp)

            # ── 步驟 3：Dubins 航點組裝 ─────────────────
            if enable_dubins_assembly and self._path_assembler is not None:
                try:
                    assembled = self._path_assembler.assemble_all(
                        area_uavs, mdtsp_result
                    )
                    mdtsp_result.assembled_paths = assembled
                except Exception as e:
                    import logging as _log
                    _log.getLogger(__name__).warning(
                        f"Dubins 航點組裝失敗 (area {cp.area_id}): {e}"
                    )

            # ── 步驟 4：高度規劃（Algorithm 3）──────────
            if enable_altitude:
                self._apply_altitude_planning(
                    mdtsp_result, terrain_func
                )

            results[cp.area_id] = mdtsp_result

        # ── 步驟 5：協調模式（同步進入時間）──────────────
        if coordination_mode == "coordinated" and self._coordinated_planner is not None:
            try:
                self._coordinated_planner.synchronize(results, uavs)
            except Exception as e:
                import logging as _log
                _log.getLogger(__name__).warning(f"協調同步失敗: {e}")

        return results

    def _apply_altitude_planning(
        self,
        mdtsp_result: MDTSPResult,
        terrain_func: Optional[Callable[[float, float], float]],
    ):
        """對 MDTSPResult 中的路徑進行高度規劃（Algorithm 3, Eq. 35-39）"""
        # 優先使用 assembled_paths（含 Dubins 曲線）
        if mdtsp_result.assembled_paths:
            for uid, apath in mdtsp_result.assembled_paths.items():
                path_2d = apath.to_latlon_list()
                if len(path_2d) >= 2:
                    alt_result = self.alt_planner.plan_altitude(
                        path_2d, terrain_func
                    )
                    # 相容新舊介面：新版返回 AltitudePlanResult，舊版返回 List
                    if hasattr(alt_result, 'waypoints_3d'):
                        path_3d = alt_result.waypoints_3d
                    else:
                        path_3d = alt_result
                    # 回寫高度到 assembled_paths
                    for i, wp in enumerate(apath.waypoints):
                        if i < len(path_3d):
                            wp.alt = path_3d[i][2]
        else:
            # 退化：僅用 left/right 端點
            for uid, ops in mdtsp_result.uav_assignments.items():
                path_2d = []
                for op in ops:
                    path_2d.append(op.left_point)
                    path_2d.append(op.right_point)
                if len(path_2d) >= 2:
                    self.alt_planner.plan_altitude(path_2d, terrain_func)

    def solve_dynamic_event(
        self,
        uavs: List[UAVState],
        coverages: List[CoveragePath],
        event_type: str = "uav_exit",
        event_uav_id: Optional[int] = None,
        new_area: Optional[CoveragePath] = None,
        terrain_func: Optional[Callable[[float, float], float]] = None,
    ) -> Dict[int, MDTSPResult]:
        """
        動態事件處理（論文 Section 4.3）。

        支援事件類型：
        - "uav_exit"  : UAV 退出任務（故障/低電量）
        - "new_area"  : 新的未知區域出現

        參數:
            uavs           : 當前所有 UAV 狀態（含位置更新）
            coverages      : 當前所有區域（含已覆蓋進度）
            event_type     : 事件類型
            event_uav_id   : 退出的 UAV ID（僅 uav_exit 使用）
            new_area       : 新增區域（僅 new_area 使用）
            terrain_func   : 地形查詢函數

        返回:
            重新規劃的 {area_id: MDTSPResult}
        """
        # ── 處理 UAV 退出 ─────────────────────────────────
        if event_type == "uav_exit" and event_uav_id is not None:
            # 取消該 UAV 的所有未完成路徑分配
            for cp in coverages:
                cp.unassign_drone(event_uav_id)
            # 從可用 UAV 列表中移除
            active_uavs = [u for u in uavs if u.uav_id != event_uav_id]
        else:
            active_uavs = list(uavs)

        # ── 處理新區域出現 ────────────────────────────────
        active_coverages = list(coverages)
        if event_type == "new_area" and new_area is not None:
            active_coverages.append(new_area)

        # ── 僅重新規劃包含未覆蓋路徑的區域 ────────────────
        replanning = [
            cp for cp in active_coverages
            if cp.get_uncovered_operations()
        ]

        return self.solve(
            active_uavs, replanning,
            terrain_func=terrain_func,
        )


# ============================================================
# 協調覆蓋規劃器
# (論文 Section 4.1.1: 協調模式下同步各 UAV 進入時間)
# ============================================================

class CoordinatedPlanner:
    """
    協調覆蓋規劃器。

    在協調模式下，所有 UAV 需要同時抵達並開始覆蓋。
    對於距離較近的 UAV，插入等待轉彎（S-turn / racetrack）
    使所有 UAV 的進入時間同步。

    使用方式:
        cp = CoordinatedPlanner(cruise_speed=230.0)
        cp.synchronize(results, uavs)
    """

    def __init__(self, cruise_speed: float = 15.0, turn_radius: float = 50.0):
        self._cruise_speed = cruise_speed
        self._turn_radius = turn_radius

    def synchronize(
        self,
        results: Dict[int, MDTSPResult],
        uavs: List[UAVState],
    ):
        """
        同步各區域中 UAV 的進入時間。

        對每個區域:
        1. 計算各 UAV 的進入路徑時間
        2. 找到最長的進入時間 T_max
        3. 為其他 UAV 在進入路徑前插入等待航點，使總時間 = T_max
        """
        for area_id, mdtsp_result in results.items():
            if not mdtsp_result.assembled_paths:
                continue

            assembled = mdtsp_result.assembled_paths

            # 計算各 UAV 的進入路徑時間
            entry_times: Dict[int, float] = {}
            for uid, apath in assembled.items():
                uav = next((u for u in uavs if u.uav_id == uid), None)
                speed = uav.speed if uav else self._cruise_speed
                entry_times[uid] = apath.entry_length_m / max(speed, 0.1)

            if not entry_times:
                continue

            # 最長進入時間
            t_max = max(entry_times.values())

            # 為較短路徑的 UAV 插入等待航點
            for uid, apath in assembled.items():
                t_entry = entry_times[uid]
                delta_t = t_max - t_entry

                if delta_t < 1.0:
                    continue  # 差距小於 1 秒，不需等待

                uav = next((u for u in uavs if u.uav_id == uid), None)
                speed = uav.speed if uav else self._cruise_speed
                r = uav.turn_radius if uav and uav.turn_radius > 0 else self._turn_radius

                # 計算需要插入的額外距離
                extra_dist = delta_t * speed

                # 在進入路徑之前插入 S-turn 等待航點
                self._insert_holding_pattern(apath, extra_dist, r)

    def _insert_holding_pattern(
        self,
        apath,
        extra_dist: float,
        turn_radius: float,
    ):
        """
        在 AssembledPath 的進入路徑起點前插入保持航線（S-turn）。

        S-turn: 左轉半圈 + 右轉半圈 = 一個完整 S 形，
        長度 = 2 × π × R，可疊加多個。
        """
        from core.trajectory.dccpp_path_assembler import (
            AssembledWaypoint, SegmentLabel,
        )

        if not apath.waypoints:
            return

        # 單個 S-turn 長度
        s_turn_len = 2.0 * math.pi * turn_radius
        n_turns = max(1, int(extra_dist / s_turn_len))

        # S-turn 起點 = 第一個航點
        first_wp = apath.waypoints[0]
        base_lat, base_lon = first_wp.lat, first_wp.lon
        heading = first_wp.heading_deg

        # 生成 S-turn 航點（左-右交替圓弧）
        holding_wps = []
        h_rad = math.radians(heading)
        lat_per_m = 1.0 / 111111.0
        lon_per_m = 1.0 / (111111.0 * math.cos(math.radians(base_lat)))

        for t in range(n_turns):
            # 左轉半圈
            for step in range(12):
                angle = h_rad + math.pi / 2 + step * math.pi / 12
                offset_n = turn_radius * (math.cos(angle) - math.cos(h_rad + math.pi / 2))
                offset_e = turn_radius * (math.sin(angle) - math.sin(h_rad + math.pi / 2))
                wp_lat = base_lat + offset_n * lat_per_m
                wp_lon = base_lon + offset_e * lon_per_m
                wp_hdg = (heading + (step + 1) * 15.0) % 360
                holding_wps.append(AssembledWaypoint(
                    lat=wp_lat, lon=wp_lon,
                    alt=first_wp.alt,
                    heading_deg=wp_hdg,
                    segment_type=SegmentLabel.ENTRY,
                ))

        # 插入到路徑起點
        apath.waypoints = holding_wps + apath.waypoints
        apath.entry_length_m += n_turns * s_turn_len
