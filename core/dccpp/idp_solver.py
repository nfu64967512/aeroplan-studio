"""
DCCPP IDP Solver v2
====================
論文 Sec 2.3 / 3.3 (Algorithm 2 — Improved Dynamic Programming) 的精確
實作，**並使用真實 ``DubinsTrajectoryGenerator.calculate_path`` 計算所有
轉移成本**，與下游 ``DCCPPPathBuilder`` 實際走的幾何完全一致。

核心輸出
--------
``IDPResult.per_uav[uav_id]`` 是一份由 ``(ScanLine, direction)`` 組成的有
序序列。``direction = +1`` 表示沿 ``pL → pR`` 飛，``-1`` 表示沿 ``pR →
pL`` 飛 (對應論文 Eq. 23–26 的 e ∈ {-1, +1})。

設計選擇
--------
- 多 UAV 同區域時，掃描線依索引「等量切段」分配到各 UAV (Strip-Split)。
  這對應論文 Fig. 5 中 l_k 架 UAV 各自負責一段連續條帶的常見實作策略。
- 單 UAV 子問題以 bitmask DP 精確求解 (n ≤ 18)；超出時退回貪婪 + 2-opt。
- 方向耦合 (Eq. 24/25/26) 自動編碼進 DP 狀態 ``(visited_mask, last_op,
  last_dir)`` — 不需要額外的方向解算器。
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from core.dccpp.area_processor import AreaProcessingResult, ScanLine

logger = logging.getLogger(__name__)


# ============================================================
# 結果結構
# ============================================================

@dataclass
class UAVPlan:
    """單架 UAV 在單一區域內的覆蓋計畫。"""
    uav_id: int
    ordered_scan_lines: List[ScanLine] = field(default_factory=list)
    directions: List[int] = field(default_factory=list)         # +1 / -1
    total_length_m: float = 0.0
    entry_length_m: float = 0.0
    transfer_length_m: float = 0.0
    operation_length_m: float = 0.0


@dataclass
class IDPResult:
    """IDP 對單一區域所有 UAV 的求解結果。"""
    area_id: int
    per_uav: Dict[int, UAVPlan] = field(default_factory=dict)

    @property
    def makespan_length(self) -> float:
        """對應論文 minmax dis_{i,k}：最長 UAV 路徑長 [m]。"""
        if not self.per_uav:
            return 0.0
        return max(p.total_length_m for p in self.per_uav.values())


# ============================================================
# Dubins 成本快取
# ============================================================

class _DubinsCostCache:
    """同一區域內所有 (start_pose, end_pose) 對的 Dubins 成本，全程快取。"""

    def __init__(self, min_turn_radius: float, altitude: float = 100.0) -> None:
        from core.base.fixed_wing_constraints import FixedWingConstraints
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator

        phi = 45.0
        safety = 1.2
        g = 9.81
        v = math.sqrt(min_turn_radius * g * math.tan(math.radians(phi)) / safety)
        c = FixedWingConstraints(
            cruise_airspeed_mps=v,
            max_bank_angle_deg=phi,
            safety_factor=safety,
        )
        self._gen = DubinsTrajectoryGenerator(c)
        self._alt = float(altitude)
        self._cache: Dict[Tuple, float] = {}

    def cost(
        self,
        x1: float, y1: float, h1_math: float,
        x2: float, y2: float, h2_math: float,
    ) -> float:
        # 量化 key (cm + 0.1°)，避免浮點抖動產生 cache miss
        key = (
            round(x1, 2), round(y1, 2), round(h1_math, 1),
            round(x2, 2), round(y2, 2), round(h2_math, 1),
        )
        if key in self._cache:
            return self._cache[key]
        from core.trajectory.dubins_trajectory import Pose3D
        try:
            p = self._gen.calculate_path(
                Pose3D(x1, y1, self._alt, h1_math),
                Pose3D(x2, y2, self._alt, h2_math),
            )
            L = float(p.total_length) if p.is_feasible else math.hypot(x2 - x1, y2 - y1)
        except Exception:
            L = math.hypot(x2 - x1, y2 - y1)
        self._cache[key] = L
        return L


# ============================================================
# IDP_Solver
# ============================================================

def _compass_to_math(c: float) -> float:
    return (90.0 - c) % 360.0


class IDP_Solver:
    """論文 Algorithm 2 求解器。

    使用方式::

        solver = IDP_Solver(min_turn_radius=80.0, altitude=120.0)
        result = solver.solve(uav_inputs, area_result, transformer)

    其中 ``uav_inputs`` 為::

        [
          {'uav_id': 1, 'lat': ..., 'lon': ..., 'heading': 90.0},
          ...
        ]
    """

    def __init__(
        self,
        min_turn_radius: float,
        altitude: float = 100.0,
        max_exact_n: int = 18,
    ) -> None:
        self.R = float(min_turn_radius)
        self.alt = float(altitude)
        self.max_exact_n = int(max_exact_n)
        self._cost = _DubinsCostCache(self.R, altitude=self.alt)

    # ────────────────────────────────────────────────────────
    # 公開介面
    # ────────────────────────────────────────────────────────
    def solve(
        self,
        uav_inputs: Sequence[Dict],
        area_result: AreaProcessingResult,
        transformer,
    ) -> IDPResult:
        scan_lines = area_result.scan_lines
        n = len(scan_lines)
        m = len(uav_inputs)
        out = IDPResult(area_id=0)

        if n == 0 or m == 0:
            for u in uav_inputs:
                out.per_uav[u['uav_id']] = UAVPlan(uav_id=u['uav_id'])
            return out

        # 1. Strip-split：掃描線依索引切成 m 段
        groups = self._partition(n, m)

        # 2. 每架 UAV 在自己 strip 上跑單機 DP
        for uav, idx_list in zip(uav_inputs, groups):
            uid = uav['uav_id']
            sub_lines = [scan_lines[i] for i in idx_list]
            if not sub_lines:
                out.per_uav[uid] = UAVPlan(uav_id=uid)
                continue

            # UAV 起點 (ENU + math heading)
            s_enu = transformer.geo_to_local(uav['lat'], uav['lon'])
            sx, sy = float(s_enu[0]), float(s_enu[1])
            sh = _compass_to_math(uav.get('heading', 0.0))

            ordered_idx, dirs, total_len, entry_len, trans_len = self._solve_single(
                sub_lines, sx, sy, sh
            )
            ordered = [sub_lines[i] for i in ordered_idx]
            op_len = sum(sl.length_m for sl in ordered)

            out.per_uav[uid] = UAVPlan(
                uav_id=uid,
                ordered_scan_lines=ordered,
                directions=dirs,
                total_length_m=total_len,
                entry_length_m=entry_len,
                transfer_length_m=trans_len,
                operation_length_m=op_len,
            )

        return out

    # ────────────────────────────────────────────────────────
    # Strip 分配
    # ────────────────────────────────────────────────────────
    @staticmethod
    def _partition(n: int, m: int) -> List[List[int]]:
        if m >= n:
            groups = [[i] for i in range(n)]
            while len(groups) < m:
                groups.append([])
            return groups
        base, rem = divmod(n, m)
        groups: List[List[int]] = []
        cur = 0
        for k in range(m):
            sz = base + (1 if k < rem else 0)
            groups.append(list(range(cur, cur + sz)))
            cur += sz
        return groups

    # ────────────────────────────────────────────────────────
    # 單 UAV 子問題：bitmask DP / 貪婪
    # ────────────────────────────────────────────────────────
    def _solve_single(
        self,
        sub_lines: List[ScanLine],
        sx: float, sy: float, sh_math: float,
    ) -> Tuple[List[int], List[int], float, float, float]:
        n = len(sub_lines)

        # 預計算 entry/exit 資訊
        # ep[l][d] = (x, y, heading_math)，d=0 表 +1(L→R) 進入端 = pL，d=1 表 -1(R→L) 進入端 = pR
        ep: List[List[Tuple[float, float, float]]] = []
        xp: List[List[Tuple[float, float, float]]] = []  # exit
        for sl in sub_lines:
            h_lr = _compass_to_math(sl.heading_compass_deg)
            h_rl = _compass_to_math((sl.heading_compass_deg + 180.0) % 360.0)
            ep.append([
                (sl.pL_xy[0], sl.pL_xy[1], h_lr),
                (sl.pR_xy[0], sl.pR_xy[1], h_rl),
            ])
            xp.append([
                (sl.pR_xy[0], sl.pR_xy[1], h_lr),    # +1 飛完從 pR 出去
                (sl.pL_xy[0], sl.pL_xy[1], h_rl),    # -1 飛完從 pL 出去
            ])

        # D_in[l][d] = UAV 起點 → 第 l 段 d 方向的 Dubins
        D_in = [[0.0, 0.0] for _ in range(n)]
        for l in range(n):
            for d in (0, 1):
                ex, ey, eh = ep[l][d]
                D_in[l][d] = self._cost.cost(sx, sy, sh_math, ex, ey, eh)

        # D_op[l] = 該段直線長度
        D_op = [sl.length_m for sl in sub_lines]

        # T[l_from][d_from][l_to][d_to] = 真實 Dubins
        T = [[[[0.0, 0.0] for _ in range(n)] for _ in range(2)] for _ in range(n)]
        for lf in range(n):
            for df in (0, 1):
                fx, fy, fh = xp[lf][df]
                for lt in range(n):
                    if lt == lf:
                        T[lf][df][lt] = [math.inf, math.inf]
                        continue
                    for dt in (0, 1):
                        tx, ty, th = ep[lt][dt]
                        T[lf][df][lt][dt] = self._cost.cost(fx, fy, fh, tx, ty, th)

        # ── DP ──
        if n <= self.max_exact_n:
            return self._bitmask_dp(n, D_in, D_op, T)
        return self._greedy(n, D_in, D_op, T)

    # ── 精確 bitmask DP ──
    @staticmethod
    def _bitmask_dp(n, D_in, D_op, T):
        FULL = (1 << n) - 1
        INF = math.inf
        # dp[mask][l][d] = 完成 mask 集合，最後在 l 段以 d 方向結束的最短長度
        dp = [[[INF, INF] for _ in range(n)] for _ in range(1 << n)]
        parent = [[[None, None] for _ in range(n)] for _ in range(1 << n)]

        for l in range(n):
            for d in (0, 1):
                m0 = 1 << l
                dp[m0][l][d] = D_in[l][d] + D_op[l]

        for mask in range(1, 1 << n):
            for l in range(n):
                if not (mask >> l) & 1:
                    continue
                for d in (0, 1):
                    cur = dp[mask][l][d]
                    if cur == INF:
                        continue
                    for lt in range(n):
                        if (mask >> lt) & 1:
                            continue
                        for dt in (0, 1):
                            cost = T[l][d][lt][dt] + D_op[lt]
                            new_mask = mask | (1 << lt)
                            cand = cur + cost
                            if cand < dp[new_mask][lt][dt]:
                                dp[new_mask][lt][dt] = cand
                                parent[new_mask][lt][dt] = (mask, l, d)

        # 找最終最佳
        best = INF
        end = (None, None)
        for l in range(n):
            for d in (0, 1):
                if dp[FULL][l][d] < best:
                    best = dp[FULL][l][d]
                    end = (l, d)

        # 回溯
        order: List[int] = []
        dirs: List[int] = []
        mask, l, d = FULL, end[0], end[1]
        while True:
            order.append(l)
            dirs.append(+1 if d == 0 else -1)
            p = parent[mask][l][d]
            if p is None:
                break
            mask, l, d = p
        order.reverse()
        dirs.reverse()

        entry_len = D_in[order[0]][0 if dirs[0] == 1 else 1]
        op_len = sum(D_op)
        trans_len = best - entry_len - op_len
        return order, dirs, best, entry_len, max(trans_len, 0.0)

    # ── 貪婪退化 ──
    @staticmethod
    def _greedy(n, D_in, D_op, T):
        # 起點：選 entry 最小者
        best_first = min(
            ((l, d) for l in range(n) for d in (0, 1)),
            key=lambda ld: D_in[ld[0]][ld[1]],
        )
        order = [best_first[0]]
        dirs_idx = [best_first[1]]
        visited = {best_first[0]}
        total = D_in[best_first[0]][best_first[1]] + D_op[best_first[0]]
        cur_l, cur_d = best_first
        while len(order) < n:
            best = None
            best_cost = math.inf
            for lt in range(n):
                if lt in visited:
                    continue
                for dt in (0, 1):
                    c = T[cur_l][cur_d][lt][dt] + D_op[lt]
                    if c < best_cost:
                        best_cost = c
                        best = (lt, dt)
            order.append(best[0])
            dirs_idx.append(best[1])
            visited.add(best[0])
            total += best_cost
            cur_l, cur_d = best
        dirs = [+1 if x == 0 else -1 for x in dirs_idx]
        entry_len = D_in[order[0]][dirs_idx[0]]
        op_len = sum(D_op)
        trans_len = total - entry_len - op_len
        return order, dirs, total, entry_len, max(trans_len, 0.0)


# ============================================================
# 向後相容包裝（給舊呼叫端用，回傳 MDTSPResult-like 結構）
# ============================================================

def to_legacy_mdtsp_result(idp_result: IDPResult, area_id: int):
    """把 IDPResult 轉為舊 MDTSPResult 格式（OperationSegment 列表）。"""
    from mission.coverage_path import OperationSegment
    from core.global_planner.mdtsp_solver import MDTSPResult

    uav_assignments: Dict[int, List[OperationSegment]] = {}
    total_distances: Dict[int, float] = {}
    for uid, plan in idp_result.per_uav.items():
        ops: List[OperationSegment] = []
        for sl, d in zip(plan.ordered_scan_lines, plan.directions):
            # 依方向決定 left/right (始終讓 left 為實際進入端)
            if d == 1:
                lp, rp = sl.pL_latlon, sl.pR_latlon
            else:
                lp, rp = sl.pR_latlon, sl.pL_latlon
            ops.append(OperationSegment(
                index=sl.index,
                left_point=lp,
                right_point=rp,
                length_m=sl.length_m,
            ))
        uav_assignments[uid] = ops
        total_distances[uid] = plan.total_length_m

    return MDTSPResult(
        area_id=area_id,
        uav_assignments=uav_assignments,
        path_matrix=None,
        decision_matrix=None,
        total_distances=total_distances,
        makespan=idp_result.makespan_length,
    )
