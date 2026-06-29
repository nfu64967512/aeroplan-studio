"""core/swarm/consensus.py — 通用心跳失效切換(Raft 風格 leader failover)核心。

本模組是「網路拓樸韌性」用的純邏輯共識核心,專為搜救(SAR)等去中心化蜂群設計:
**長機(leader)心跳消失 → 存活僚機在數秒內自動接管指揮鏈**,確保鏈路不中斷。

刻意與任何任務語意「解耦」:
  - 不引入「距目標距離」加權(舊版 biomimetic_formation.select_leader 把選舉綁死在
    幾何距離上;本核心改用可注入的健康度優先序,預設最低 sysid 的經典 Raft tiebreak)。
  - 不依賴座標、不依賴打擊目標、不依賴 ToT/STOT 等戰術概念。
  - 無 Qt、無 MAVLink、無 ROS → 可純單元測試,亦可直接搬進嵌入式 ROS2 蜂群節點。

設計取向:**GCS 端集中式容錯監督者(centralized failover supervisor)**。
GCS 以 `FleetRegistry` 為共享黑板,本身即是權威決策點;故這裡用「確定性」單一決策,
而非分散式 RequestVote 投票(分散式 peer 版另由 Jetson `fleet_consensus/raft_node` 負責)。
集中式換來:(1) 可確定性測試、(2) 接管延遲有上界、(3) 不需隨機選舉逾時避免分裂投票。

核心語意(Raft 對應):
  - term(任期):每次長機「更替」單調 +1,作為新舊指令的判別(舊 term 心跳被忽略)。
  - leader liveness:長機心跳新鮮(age ≤ heartbeat_timeout_s)即「續任」,不被搶位 →
    防抖動(anti-flapping);唯有長機自己心跳逾時(失效)才觸發重新選舉。
  - failover:長機失效 → 存活候選中依優先序選出接管者,term+1,發 LEADER_CHANGED。
  - 接管上界:worst_case_takeover ≈ heartbeat_timeout_s + tick_period_s(設定階段驗證 ≤ 3 秒)。
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

# 排序鍵型別:priority_key(node) 回傳的可比較鍵(min() 取最優先)。
SortKey = Tuple[object, ...]
PriorityKey = Callable[["NodeRecord"], object]

_NEG_INF = float("-inf")


# ═══════════════════════════════════════════════════════════════════════════
#  設定 / 資料結構
# ═══════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class FailoverConfig:
    """容錯核心參數。

    Attributes:
        heartbeat_timeout_s: 心跳逾時門檻(秒)。節點最後一次心跳距今超過此值 → 視為失效。
        tick_period_s:       監督者預期的 tick 週期(秒);僅用於計算接管延遲上界。
        takeover_deadline_s: 接管延遲上界需求(秒)。SAR 要求「3 秒內接管」→ 預設 3.0。
    """
    heartbeat_timeout_s: float = 1.0
    tick_period_s: float = 0.2
    takeover_deadline_s: float = 3.0

    def __post_init__(self) -> None:
        # 防呆:任何非正值都會讓 liveness 判定失去意義。
        if self.heartbeat_timeout_s <= 0.0:
            raise ValueError("heartbeat_timeout_s 必須為正數")
        if self.tick_period_s <= 0.0:
            raise ValueError("tick_period_s 必須為正數")
        if self.takeover_deadline_s <= 0.0:
            raise ValueError("takeover_deadline_s 必須為正數")
        # 核心安全保證:最壞接管延遲(逾時 + 一個 tick)必須「嚴格小於」需求上界,
        # 否則「3 秒接管」承諾不成立 → 設定階段即拒絕,不讓它默默違規。
        # 用 >= 而非 >:worst == deadline 表示零餘裕,Qt 排程抖動 / 下游致動延遲
        # 必定把端到端接管推過上界,故一律拒絕,強制保留餘裕。
        worst = self.heartbeat_timeout_s + self.tick_period_s
        if worst >= self.takeover_deadline_s:
            raise ValueError(
                f"最壞接管延遲 {worst:.3f}s 已達/超過上界 {self.takeover_deadline_s:.3f}s(需保留餘裕);"
                f"請調小 heartbeat_timeout_s/tick_period_s 或放寬 takeover_deadline_s"
            )


@dataclass(frozen=True)
class NodeHealth:
    """節點健康度 — 選舉優先序的「非任務」輸入(與打擊目標、距離完全無關)。

    Attributes:
        battery_pct:   電量百分比(0~100),越高越優先接管。
        gps_ok:        GPS 定位是否健康(3D fix 以上)。失鎖者不宜當長機。
        battery_known: 電量是否「已知」。遙測未回報電量時應設 False —— 未知電量
                       絕不可被當成滿電而優先當選(否則遙測失明的機反而搶到指揮鏈)。
    """
    battery_pct: float = 100.0
    gps_ok: bool = True
    battery_known: bool = True


@dataclass
class NodeRecord:
    """單一節點在容錯核心內的狀態。"""
    node_id: int
    last_heartbeat_s: float = _NEG_INF   # 最後一次心跳的時戳(單調時鐘秒);_NEG_INF = 從未收到
    health: NodeHealth = field(default_factory=NodeHealth)
    alive: bool = False                  # 由 tick() 依心跳新鮮度更新


class EventKind(enum.Enum):
    """容錯事件種類。"""
    LEADER_CHANGED = "leader_changed"     # 長機更替(含初次選出);term 已 +1
    LEADER_LOST = "leader_lost"           # 全機失聯 → 暫時無長機
    NODE_LOST = "node_lost"               # 某節點心跳逾時(失聯)
    NODE_RECOVERED = "node_recovered"     # 某節點心跳恢復


@dataclass(frozen=True)
class FailoverEvent:
    """一次 tick() 產出的事件;供上層(UI / 指令層)反應。"""
    kind: EventKind
    node_id: int                          # 事件主體節點(LEADER_LOST 時為舊長機)
    term: int                             # 事件當下的任期
    t: float                              # 事件時戳(傳入 tick 的 now)
    old_leader: Optional[int] = None      # 僅 LEADER_CHANGED / LEADER_LOST 有意義
    new_leader: Optional[int] = None      # 僅 LEADER_CHANGED 有意義


# ═══════════════════════════════════════════════════════════════════════════
#  優先序策略(可注入)
# ═══════════════════════════════════════════════════════════════════════════
def lowest_id_priority_key(node: "NodeRecord") -> object:
    """預設策略:最低 node_id 優先(經典 Raft 確定性 tiebreak,完全與任務無關)。"""
    return node.node_id


def health_priority_key(node: "NodeRecord") -> SortKey:
    """SAR 推薦策略:健康度優先 — GPS 正常 > 電量已知 > 電量高 > id 小。

    回傳可比較 tuple,`min()` 取最優先:
      (0 if gps_ok else 1, 0 if battery_known else 1, -battery_pct, node_id)
      → GPS 正常者排前;電量「已知」者排前(未知電量不得冒充滿電搶位);
        電量高者排前;最後以最低 id 作確定性 tiebreak。
    刻意「不含」任何距目標距離 / 戰術權重。
    """
    h = node.health
    return (0 if h.gps_ok else 1, 0 if h.battery_known else 1, -float(h.battery_pct), node.node_id)


# ═══════════════════════════════════════════════════════════════════════════
#  容錯核心
# ═══════════════════════════════════════════════════════════════════════════
class RaftFailover:
    """集中式心跳失效切換核心(純邏輯,確定性)。

    用法:
        core = RaftFailover(priority_key=health_priority_key)
        # 每收到一架的遙測/心跳:
        core.heartbeat(sysid, t=now, health=NodeHealth(battery_pct=..., gps_ok=...))
        # 每個 tick(固定週期)推進並取事件:
        for ev in core.tick(now):
            ... # 反應 LEADER_CHANGED / NODE_LOST ...

    不變式(invariants):
        - 健康長機不被搶位:只要長機心跳新鮮就續任 → 防抖動。
        - term 單調遞增:每次長機「更替」+1;絕不回退。
        - 接管有上界:worst_case_takeover_s ≤ config.takeover_deadline_s(設定階段已驗證)。
    """

    def __init__(
        self,
        config: Optional[FailoverConfig] = None,
        priority_key: Optional[PriorityKey] = None,
    ) -> None:
        self.cfg: FailoverConfig = config or FailoverConfig()
        # 預設最低 id;SAR 可傳 health_priority_key。刻意不接受任何「目標座標」。
        self._priority_key: PriorityKey = priority_key or lowest_id_priority_key
        self._nodes: Dict[int, NodeRecord] = {}
        self._leader_id: Optional[int] = None
        self._term: int = 0

    # ── 唯讀狀態 ────────────────────────────────────────────────────────────
    @property
    def leader_id(self) -> Optional[int]:
        """目前長機 sysid;全機失聯時為 None。"""
        return self._leader_id

    @property
    def term(self) -> int:
        """目前任期(每次長機更替 +1)。"""
        return self._term

    @property
    def worst_case_takeover_s(self) -> float:
        """最壞接管延遲(秒)= 心跳逾時 + 一個 tick 週期。

        注意:這是「純邏輯」延遲 —— 從長機靜默到核心**發出** LEADER_CHANGED 為止,
        不含 Qt 排程抖動與下游致動傳遞。端到端接管需在此之上再預留餘裕(故 config
        驗證採嚴格不等式)。實際 tick 由 FailoverMonitor 驅動,其 QTimer 週期必須與
        cfg.tick_period_s 一致,否則此值會低估真實延遲。
        """
        return self.cfg.heartbeat_timeout_s + self.cfg.tick_period_s

    def node_ids(self) -> List[int]:
        """目前已知的所有節點 id(含已失聯但尚未移除者)。"""
        return list(self._nodes.keys())

    def alive_nodes(self) -> List[int]:
        """目前判定為存活(心跳新鮮)的節點 id。"""
        return [n.node_id for n in self._nodes.values() if n.alive]

    def fleet_liveness(self) -> Dict[int, bool]:
        """全機存活快照 {node_id: alive};供 UI 顯示健康狀態。"""
        return {nid: n.alive for nid, n in self._nodes.items()}

    # ── 節點管理 ────────────────────────────────────────────────────────────
    def add_node(self, node_id: int, health: Optional[NodeHealth] = None) -> None:
        """登記一個節點(若已存在則僅更新健康度)。新節點預設未存活,待心跳判定。"""
        node_id = int(node_id)
        rec = self._nodes.get(node_id)
        if rec is None:
            self._nodes[node_id] = NodeRecord(node_id=node_id, health=health or NodeHealth())
        elif health is not None:
            rec.health = health

    def remove_node(self, node_id: int) -> None:
        """移除節點(如已退役 / 解除註冊)。若移除的是現任長機,下個 tick 會重新選舉。"""
        self._nodes.pop(int(node_id), None)
        # 注意:不在此清掉 _leader_id;保留它讓 tick() 能在事件中回報「舊長機是誰」。

    def heartbeat(
        self, node_id: int, t: float, health: Optional[NodeHealth] = None
    ) -> None:
        """記錄一次心跳(收到該機遙測即呼叫)。未知節點自動登記。

        Args:
            node_id: 節點 sysid。
            t:       心跳時戳(單調時鐘秒);應為「該筆遙測實際抵達時刻」。
            health:  最新健康度(可選);提供則更新,供下一次選舉參考。
        """
        node_id = int(node_id)
        rec = self._nodes.get(node_id)
        if rec is None:
            rec = NodeRecord(node_id=node_id, health=health or NodeHealth())
            self._nodes[node_id] = rec
        # 單調保護:不讓較舊的時戳覆蓋較新的(亂序遙測時保險)。
        if t > rec.last_heartbeat_s:
            rec.last_heartbeat_s = t
        if health is not None:
            rec.health = health

    def set_leader(self, node_id: int, t: float = 0.0) -> Optional[FailoverEvent]:
        """強制指定長機(操作員覆寫 / 初始種子)。若有更替則 term+1 並回傳事件。

        Args:
            node_id: 欲指定的長機 sysid。
            t:       指定當下的時戳(單調時鐘秒);用於心跳新鮮度檢查與事件時戳。

        Raises:
            ValueError: 目標節點不存在,或其心跳已逾時(相對 t)。拒絕指定「不新鮮」
                的節點當長機 —— 否則它會在下一拍立刻被廢黜,造成雙重 term 抖動(幽靈長機)。
        """
        node_id = int(node_id)
        rec = self._nodes.get(node_id)
        if rec is None or (t - rec.last_heartbeat_s) > self.cfg.heartbeat_timeout_s:
            raise ValueError(
                f"無法指定 sysid={node_id} 為長機:該節點不存在或心跳已逾時(相對 t={t:.3f})"
            )
        if node_id == self._leader_id:
            return None
        old = self._leader_id
        self._term += 1
        self._leader_id = node_id
        rec.alive = True   # 已確認新鮮 → 標記存活,使下一拍直接續任、不再重選
        return FailoverEvent(
            kind=EventKind.LEADER_CHANGED, node_id=node_id, term=self._term,
            t=t, old_leader=old, new_leader=node_id,
        )

    # ── 推進一拍 ────────────────────────────────────────────────────────────
    def tick(self, now: float) -> List[FailoverEvent]:
        """推進容錯狀態機一拍,回傳本拍所有事件。

        步驟:
          1) 依心跳新鮮度更新每個節點 alive,發 NODE_LOST / NODE_RECOVERED。
          2) 若長機不存在或已失效 → 在存活候選中依優先序選新長機(term+1, LEADER_CHANGED);
             若無任何存活候選 → LEADER_LOST,長機設為 None。
             健康長機則「續任」,不被任何更高優先序節點搶位(防抖動)。
        """
        events: List[FailoverEvent] = []

        # 1) liveness 更新 ----------------------------------------------------
        for node in self._nodes.values():
            fresh = (now - node.last_heartbeat_s) <= self.cfg.heartbeat_timeout_s
            if fresh and not node.alive:
                node.alive = True
                events.append(FailoverEvent(
                    EventKind.NODE_RECOVERED, node_id=node.node_id, term=self._term, t=now))
            elif not fresh and node.alive:
                node.alive = False
                events.append(FailoverEvent(
                    EventKind.NODE_LOST, node_id=node.node_id, term=self._term, t=now))

        # 2) 長機健康檢查 → 必要時選舉 ----------------------------------------
        leader = self._nodes.get(self._leader_id) if self._leader_id is not None else None
        leader_ok = leader is not None and leader.alive
        if leader_ok:
            return events  # 長機健康 → 續任,任何更高優先序都不搶位(防抖動)

        old_leader = self._leader_id
        candidates = [n for n in self._nodes.values() if n.alive]
        if not candidates:
            # 全機失聯 → 暫時無長機
            if self._leader_id is not None:
                self._leader_id = None
                events.append(FailoverEvent(
                    EventKind.LEADER_LOST, node_id=old_leader if old_leader is not None else -1,
                    term=self._term, t=now, old_leader=old_leader, new_leader=None))
            return events

        # 存活候選中依優先序(預設最低 id;SAR 用健康度)選出接管者 —— 不含距目標距離。
        winner = min(candidates, key=self._priority_key)
        if winner.node_id != old_leader:
            self._term += 1
            self._leader_id = winner.node_id
            events.append(FailoverEvent(
                EventKind.LEADER_CHANGED, node_id=winner.node_id, term=self._term,
                t=now, old_leader=old_leader, new_leader=winner.node_id))
        else:
            # 舊長機仍是最優先且已恢復存活 → 沿用,不更替、不升 term。
            self._leader_id = winner.node_id
        return events


__all__ = [
    "FailoverConfig",
    "NodeHealth",
    "NodeRecord",
    "EventKind",
    "FailoverEvent",
    "RaftFailover",
    "lowest_id_priority_key",
    "health_priority_key",
]
