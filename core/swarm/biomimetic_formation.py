"""仿生編隊演算法（Biomimetic Formation）— 去中心化推拉力 + 相鄰追蹤 + 動態長機選舉。

取代「所有僚機死盯一台長機」的星型拓樸，改用候鳥式的局部規則，讓大間距 V 型編隊
自然湧現、遇障自動分裂穿透、過障無縫重組，並在長機失效時毫秒級接管指揮鏈。

純邏輯（2D 局部 ENU 公尺座標，無 ROS / 無 MAVLink）→ 可單元測試 + 模擬，亦可直接
搬進嵌入式 ROS2 蜂群節點（wingman_node 以鄰機 GeoPose 餵 step()，輸出速度指令）。

三大機制：
  1) 相鄰單元追蹤 Adjacent Unit Tracking：僚機不訂閱長機座標，而是挑「比自己更靠近
     目標、空間最近」的相鄰僚機作相對參考點，維持 V 翼偏置 → V 由局部鏈結湧現。
  2) 動態重構 Dynamic Reshaping：推拉力（分離 push / slot 吸引 pull / 對齊 / 障礙排斥
     / 趨目標）疊加；遇障時障礙排斥壓過 slot 力 → 分裂繞行，過障後 slot 力主導 → 重組。
  3) 動態長機選舉 Dynamic Leader Selection：長機週期送心跳；心跳逾時（失效）→ 距目標
     最近的存活機接管成新臨時長機（Raft 風格：term+1），確保指揮鏈不中斷。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class Agent:
    """單機狀態（局部 ENU 公尺）。嵌入式端由鄰機 GeoPose 轉入、step() 後取 (vx,vy) 下令。"""
    sysid: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    is_leader: bool = False
    alive: bool = True
    last_heartbeat_t: float = 0.0
    term: int = 0


@dataclass
class FormationParams:
    shape: str = 'V'                 # 'V' | 'GRID' | 'LINE'
    spacing_m: float = 50.0          # 編隊間距（候鳥式大間距）
    sep_radius_m: float = 25.0       # 分離（避撞）作用半徑
    obstacle_margin_m: float = 60.0  # 障礙感知邊際（障礙半徑外再加此距離）
    max_speed: float = 20.0          # 速度上限（m/s）
    w_sep: float = 1.8               # 分離 push 權重
    w_slot: float = 1.0              # slot 吸引 pull 權重
    w_align: float = 0.35            # 對齊（match 鄰機速度）權重
    w_obstacle: float = 3.0          # 障礙排斥權重（強 → 觸發分裂）
    w_goal: float = 1.0              # 趨目標權重
    heartbeat_timeout_s: float = 1.0  # 長機心跳逾時 → 觸發選舉


def _hypot(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


class BiomimeticFormation:
    """無狀態演算法物件（狀態都在 Agent 上）；同一 params 可重用於整群。"""

    def __init__(self, params: Optional[FormationParams] = None) -> None:
        self.p = params or FormationParams()

    # ── 1) 相鄰單元追蹤 ───────────────────────────────────────────────────
    def forward_reference(self, agent: Agent, agents: List[Agent],
                          goal: Tuple[float, float]) -> Optional[Agent]:
        """挑相對參考機：在「比自己更靠近目標」的存活鄰機中，取空間最近者。

        非死盯長機 → 去中心化；frontmost（含長機）無此參考 → 回 None（改趨目標）。
        """
        gx, gy = goal
        d_self = _hypot(agent.x, agent.y, gx, gy)
        best, best_d = None, float('inf')
        for a in agents:
            if not a.alive or a.sysid == agent.sysid:
                continue
            if _hypot(a.x, a.y, gx, gy) >= d_self:
                continue                       # 不比自己前面 → 不當參考
            d = _hypot(a.x, a.y, agent.x, agent.y)
            if d < best_d:
                best, best_d = a, d
        return best

    def desired_slot(self, agent: Agent, ref: Agent,
                     goal: Tuple[float, float]) -> Tuple[float, float]:
        """相對參考機的 slot：沿「參考機→目標」方向後退 spacing，並側向偏置。

        V：依 sysid 奇偶分左右翼（去中心化、免全域排序）；LINE：純後退；GRID：側向較密。
        """
        gx, gy = goal
        fx, fy = gx - ref.x, gy - ref.y
        n = math.hypot(fx, fy) or 1.0
        fx, fy = fx / n, fy / n                # forward unit（朝目標）
        px, py = -fy, fx                       # 左垂直
        side = 1.0 if (agent.sysid % 2 == 0) else -1.0
        s = self.p.spacing_m
        if self.p.shape == 'LINE':
            lat = 0.0
        elif self.p.shape == 'GRID':
            lat = side * s * 0.6
        else:                                  # 'V'
            lat = side * s
        sx = ref.x - fx * s + px * lat
        sy = ref.y - fy * s + py * lat
        return sx, sy

    # ── 2) 動態重構：推拉力疊加 ───────────────────────────────────────────
    def compute_velocity(self, agent: Agent, agents: List[Agent],
                         goal: Tuple[float, float],
                         obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float]:
        """回傳該機速度指令 (vx, vy)。各分量：分離 / slot / 對齊 / 障礙 / 趨目標。"""
        p = self.p
        vmax = p.max_speed
        ax = ay = 0.0

        # 分離（push，m/s）：被過近鄰機反向推開（避撞）；越近推力越強（→ vmax）
        for a in agents:
            if not a.alive or a.sysid == agent.sysid:
                continue
            dx, dy = agent.x - a.x, agent.y - a.y
            d = math.hypot(dx, dy)
            if 0.0 < d < p.sep_radius_m:
                k = p.w_sep * (p.sep_radius_m - d) / p.sep_radius_m * vmax
                ax += k * dx / d
                ay += k * dy / d

        if agent.is_leader:
            # 長機：以 vmax 領航趨目標
            gx, gy = goal
            dx, dy = gx - agent.x, gy - agent.y
            n = math.hypot(dx, dy) or 1.0
            ax += p.w_goal * vmax * dx / n
            ay += p.w_goal * vmax * dy / n
        else:
            ref = self.forward_reference(agent, agents, goal)
            if ref is not None:
                sx, sy = self.desired_slot(agent, ref, goal)
                dx, dy = sx - agent.x, sy - agent.y
                n = math.hypot(dx, dy) or 1.0
                speed = min(n, vmax)                       # 比例趨近 slot（遠→快、近→慢不過衝）
                ax += p.w_slot * speed * dx / n
                ay += p.w_slot * speed * dy / n
                ax += p.w_align * ref.vx                   # 對齊：跟參考機同向（已 m/s）
                ay += p.w_align * ref.vy
            else:
                gx, gy = goal                              # frontmost 僚機 → 以 vmax 趨目標
                dx, dy = gx - agent.x, gy - agent.y
                n = math.hypot(dx, dy) or 1.0
                ax += p.w_goal * vmax * dx / n
                ay += p.w_goal * vmax * dy / n

        # 障礙排斥（push，強，m/s）：< 感知邊際時強力側推 → 分裂繞行
        for ox, oy, orad in obstacles:
            dx, dy = agent.x - ox, agent.y - oy
            d = math.hypot(dx, dy)
            reach = orad + p.obstacle_margin_m
            if 0.0 < d < reach:
                k = p.w_obstacle * (reach - d) / reach * vmax
                ax += k * dx / d
                ay += k * dy / d

        spd = math.hypot(ax, ay)
        if spd > vmax:
            ax = ax / spd * vmax
            ay = ay / spd * vmax
        return ax, ay

    # ── 3) 動態長機選舉 ───────────────────────────────────────────────────
    def select_leader(self, agents: List[Agent], goal: Tuple[float, float],
                      t_now: float) -> Optional[int]:
        """回傳應任長機的 sysid。長機存活且心跳新鮮 → 續任；否則距目標最近存活機接管。"""
        alive = [a for a in agents if a.alive]
        if not alive:
            return None
        cur = next((a for a in alive if a.is_leader), None)
        if cur is not None and (t_now - cur.last_heartbeat_t) <= self.p.heartbeat_timeout_s:
            return cur.sysid                   # 長機健康 → 續任
        gx, gy = goal
        new = min(alive, key=lambda a: _hypot(a.x, a.y, gx, gy))  # 最靠近目標者接管
        return new.sysid

    # ── 一步推進（模擬 / 嵌入式 tick）─────────────────────────────────────
    def step(self, agents: List[Agent], goal: Tuple[float, float],
             obstacles: List[Tuple[float, float, float]],
             t_now: float, dt: float) -> int:
        """選舉 → 計算各機速度 → 積分位置。回傳目前長機 sysid（嵌入式可據此送心跳）。"""
        lid = self.select_leader(agents, goal, t_now)
        max_term = max((a.term for a in agents), default=0)
        for a in agents:
            was = a.is_leader
            a.is_leader = (a.sysid == lid)
            if a.is_leader and not was:
                a.term = max_term + 1          # 新長機升 term（Raft 風格）
            if a.is_leader and a.alive:
                a.last_heartbeat_t = t_now      # 長機送心跳

        cmds: Dict[int, Tuple[float, float]] = {}
        for a in agents:
            if a.alive:
                cmds[a.sysid] = self.compute_velocity(a, agents, goal, obstacles)
        for a in agents:
            if not a.alive:
                continue
            a.vx, a.vy = cmds[a.sysid]
            a.x += a.vx * dt
            a.y += a.vy * dt
        return lid if lid is not None else -1
