"""剩餘時間 (t_go) 協商 — 協同變數制導律（Coordination Variables）。

機群進入決斷圈時，透過共享黑板（[[fleet_registry]] = Mesh 等價）即時交換各自的
剩餘飛行時間 t_go，協商出一個「使整體能量消耗最小」的共同命中時刻 t*（協同變數）。

牧羊犬網格（Shepherd Grid）執行：以協商出的 t* 為錨，距離遠者全速直線衝刺、距離近者
盤旋 / S 機動耗時，最終全機在同一刻、由不同軸線精準抵達目標。

── 能量模型 ─────────────────────────────────────────────────────────────────
單位距離能耗 g(v) = ½(v/v_eff)² + ½(v_eff/v)²，在最佳續航速度 v_eff 取最小值 1；
飛得比 v_eff 快或慢，每單位距離都更耗能（阻力功 + 誘導阻力的折衷）。
  • 直線段（v=r/τ ≥ v_min）：E = r · g(v)。τ 越大 → v 越小 → 趨近/低於 v_eff。
  • 耗時段（v=r/τ < v_min，太近）：以 v_min 飛「加長路徑 L=v_min·τ」(S 機動/盤旋)：
        E = L · g(v_min) = v_min·τ·g(v_min)，隨 τ 增加 → 耗時越久越耗能。
遠機（大 r）屬直線段（E 隨 τ 遞減），近機（小 r）屬耗時段（E 隨 τ 遞增）→ 總能量
對 τ 有內部極小 → 即「整體能量消耗最小」的協商共同時間 τ*。

註：協商在 GCS 端集中求解，但其結果（單一協同變數 t*）與分散式 consensus 等價——
每個 mesh 節點以共享 t_go 跑相同最小化都會收斂到同一 t*。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

# 角色（牧羊犬網格中各機的終端行為）
ROLE_SPRINT = 'sprint'    # 距離遠 → 全速直線衝刺（v 顯著 > v_eff）
ROLE_CRUISE = 'cruise'    # 距離適中 → 近 v_eff 巡航
ROLE_BURN = 'burn'        # 距離近 → 盤旋耗時（v_req < v_min，先不撲；舊式盤旋）
ROLE_WEAVE = 'weave'      # 距離近 → S 型機動耗時（定速 v_eff，航向偏角；精準對時）


@dataclass
class TgoPlan:
    """協商結果：共同剩餘時間 + 各機角色/速度 + 能量。"""
    tgo_common_s: float                       # 協商出的共同剩餘時間 τ*
    roles: Dict[int, str] = field(default_factory=dict)     # sysid → ROLE_*
    speeds: Dict[int, float] = field(default_factory=dict)  # sysid → 指令空速（burn 機=v_min）
    energy: float = 0.0                        # 該 τ* 的總能量（相對值）
    tgo_floor_s: float = 0.0                   # 可行下界 = max(r/v_max)（最遠機全速）


def energy_per_distance(v: float, v_eff: float) -> float:
    """g(v)：單位距離能耗，於 v_eff 取最小值 1。v、v_eff 需 > 0。"""
    if v <= 0.0 or v_eff <= 0.0:
        return float('inf')
    rv = v / v_eff
    return 0.5 * rv * rv + 0.5 / (rv * rv)


def _aircraft_energy(r: float, tau: float, v_min: float, v_eff: float) -> float:
    """單機在共同剩餘時間 τ 下的能量（直線段 vs 耗時段）。"""
    if r <= 0.0:
        return 0.0                       # 已在目標 → 零距離零能量（勿落入 burn 分支）
    if tau <= 0.0:
        return float('inf')
    v = r / tau
    if v >= v_min:                       # 直線段（含 sprint / cruise）
        return r * energy_per_distance(v, v_eff)
    # 耗時段：以 v_min 飛加長路徑 L = v_min·τ（S 機動/盤旋）
    return (v_min * tau) * energy_per_distance(v_min, v_eff)


def total_energy(tau: float, rs: List[float], v_min: float, v_eff: float) -> float:
    return sum(_aircraft_energy(r, tau, v_min, v_eff) for r in rs)


def _golden_min(f, lo: float, hi: float, iters: int = 80) -> float:
    """黃金分割搜尋一維極小（確定性、不用亂數；τ 區間為單峰）。"""
    if hi <= lo:
        return lo
    gr = (5.0 ** 0.5 - 1.0) / 2.0        # 0.618…
    c = hi - gr * (hi - lo)
    d = lo + gr * (hi - lo)
    fc, fd = f(c), f(d)
    for _ in range(iters):
        if fc < fd:
            hi, d, fd = d, c, fc
            c = hi - gr * (hi - lo)
            fc = f(c)
        else:
            lo, c, fc = c, d, fd
            d = lo + gr * (hi - lo)
            fd = f(d)
    return 0.5 * (lo + hi)


def negotiate_tgo(rs: Dict[int, float], v_min: float, v_max: float,
                  v_eff: float, *, tau_hi_factor: float = 1.3) -> TgoPlan:
    """協商「使整體能量消耗最小」的共同剩餘時間 τ*，並指派各機角色/速度。

    rs    : {sysid: 到目標剩餘距離 r_i (m)}（即 t_go = r_i/v 的距離來源）
    v_min : 最小可維持空速（低於此須盤旋/S 機動耗時）
    v_max : 最大可衝刺空速
    v_eff : 最佳續航速度（能耗最小點；通常取巡航速度）
    回傳 TgoPlan：τ*、各機 ROLE_*、指令速度（burn 機=v_min）、總能量。
    """
    if not rs:
        raise ValueError("rs 不可為空")
    if not (0 < v_min < v_max):
        raise ValueError(f"need 0 < v_min < v_max, got {v_min}, {v_max}")
    if any(r < 0 for r in rs.values()):
        raise ValueError(f"剩餘距離 r 需 >= 0，收到 {dict(rs)}")
    # v_eff 夾進 [v_min, v_max]（能耗最小點須落在可行速度域內；驅動傳巡航速通常即在域內）
    v_eff = min(max(v_eff, v_min * 1.001), v_max * 0.999)

    r_list = list(rs.values())
    # 全機都已在目標（皆 r=0）→ 退化解：τ*=0、零能量
    if max(r_list) <= 0.0:
        return TgoPlan(tgo_common_s=0.0,
                       roles={s: ROLE_BURN for s in rs},
                       speeds={s: v_min for s in rs},
                       energy=0.0, tgo_floor_s=0.0)
    # 可行下界：最遠機都得能（全速）抵達 → τ ≥ max(r/v_max)
    tau_floor = max(r / v_max for r in r_list)
    # 上界：略超過「最遠機以 v_min 飛」的時間（再大全機都在耗時段、能量續增）
    tau_hi = max(max(r / v_min for r in r_list) * tau_hi_factor, tau_floor + 1.0)

    tau_star = _golden_min(
        lambda t: total_energy(t, r_list, v_min, v_eff), tau_floor, tau_hi)
    tau_star = max(tau_star, tau_floor)      # 夾在可行域

    roles, speeds = {}, {}
    for sid, r in rs.items():
        v = r / tau_star
        if v < v_min:
            roles[sid] = ROLE_BURN
            speeds[sid] = v_min
        elif v > v_eff * 1.02:
            roles[sid] = ROLE_SPRINT
            speeds[sid] = min(v, v_max)
        else:
            roles[sid] = ROLE_CRUISE
            speeds[sid] = min(v, v_max)

    return TgoPlan(
        tgo_common_s=tau_star, roles=roles, speeds=speeds,
        energy=total_energy(tau_star, r_list, v_min, v_eff),
        tgo_floor_s=tau_floor)
