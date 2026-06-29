"""RaftFailover 容錯核心單元測試 —— 保障 SAR 鏈路韌性不會默默失效。

涵蓋:初次選舉、健康長機續任、3 秒內接管、健康度選舉(非距目標距離)、
recovered 不搶位(防抖動)、term 單調遞增、全機失聯後恢復、多機同時失效、
設定驗證、確定性。

執行:pytest tests/test_consensus_failover.py -v
"""
import pytest

from core.swarm.consensus import (
    EventKind,
    FailoverConfig,
    NodeHealth,
    RaftFailover,
    health_priority_key,
)


# ═══════════════════════════════════════════════════════════════════════════
#  輔助
# ═══════════════════════════════════════════════════════════════════════════
def beat(core: RaftFailover, ids, t, health=None):
    """讓一組節點在時刻 t 各送一次心跳。"""
    for i in ids:
        core.heartbeat(i, t, health.get(i) if health else None)


def kinds(events):
    return [e.kind for e in events]


# ═══════════════════════════════════════════════════════════════════════════
#  初次選舉 / 續任
# ═══════════════════════════════════════════════════════════════════════════
def test_initial_election_picks_lowest_id():
    """預設策略:三機上線 → 最低 id 當選,term 由 0 升 1。"""
    core = RaftFailover()
    beat(core, [1, 2, 3], 0.0)
    ev = core.tick(0.0)
    assert core.leader_id == 1
    assert core.term == 1
    assert EventKind.LEADER_CHANGED in kinds(ev)
    # 三機皆由「未存活」轉「存活」。
    assert sum(k is EventKind.NODE_RECOVERED for k in kinds(ev)) == 3


def test_healthy_leader_retained_no_flapping():
    """長機持續送心跳 → 連續多拍續任,term 不變。"""
    core = RaftFailover()
    for t in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5):
        beat(core, [1, 2, 3], t)
        core.tick(t)
    assert core.leader_id == 1
    assert core.term == 1


# ═══════════════════════════════════════════════════════════════════════════
#  3 秒內接管
# ═══════════════════════════════════════════════════════════════════════════
def test_failover_within_three_seconds():
    """長機心跳消失 → 僚機在 3 秒內接管(本設定實測 1.2 秒)。"""
    core = RaftFailover()
    beat(core, [1, 2, 3], 0.0)
    core.tick(0.0)
    assert core.leader_id == 1
    last_leader_beat = 0.0

    # 長機 1 不再送心跳;2、3 持續。t=1.0 時 age=1.0 == 逾時門檻 → 仍續任。
    beat(core, [2, 3], 1.0)
    core.tick(1.0)
    assert core.leader_id == 1

    # t=1.2(逾時後一拍)→ age=1.2 > 1.0 → 失效 → 接管。
    beat(core, [2, 3], 1.2)
    ev = core.tick(1.2)
    assert core.leader_id == 2
    assert EventKind.LEADER_CHANGED in kinds(ev)
    assert any(e.kind is EventKind.NODE_LOST and e.node_id == 1 for e in ev)

    takeover_latency = 1.2 - last_leader_beat
    assert takeover_latency <= 3.0
    assert core.worst_case_takeover_s <= 3.0


# ═══════════════════════════════════════════════════════════════════════════
#  健康度選舉(非距目標距離)
# ═══════════════════════════════════════════════════════════════════════════
def test_health_priority_not_distance():
    """注入健康度策略:電量最高且 GPS 正常者當選,而非最低 id、亦非任何距離。"""
    core = RaftFailover(priority_key=health_priority_key)
    health = {
        1: NodeHealth(battery_pct=20.0, gps_ok=True),   # 低電量
        2: NodeHealth(battery_pct=90.0, gps_ok=True),   # 高電量 → 應當選
        3: NodeHealth(battery_pct=80.0, gps_ok=False),  # GPS 失鎖 → 不宜
    }
    beat(core, [1, 2, 3], 0.0, health)
    core.tick(0.0)
    assert core.leader_id == 2  # 非最低 id(1),純由健康度決定


def test_gps_lock_outranks_battery():
    """GPS 正常優先於電量:GPS 失鎖但滿電者,不得勝過 GPS 正常者。"""
    core = RaftFailover(priority_key=health_priority_key)
    health = {
        1: NodeHealth(battery_pct=100.0, gps_ok=False),  # 滿電但失鎖
        2: NodeHealth(battery_pct=40.0, gps_ok=True),    # 半電但正常 → 應當選
    }
    beat(core, [1, 2], 0.0, health)
    core.tick(0.0)
    assert core.leader_id == 2


# ═══════════════════════════════════════════════════════════════════════════
#  防抖動:recovered 舊長機不搶位
# ═══════════════════════════════════════════════════════════════════════════
def test_recovered_leader_does_not_preempt():
    """舊長機失效→2 接管後,舊長機恢復心跳也不搶回指揮鏈(term 不變)。"""
    core = RaftFailover()
    beat(core, [1, 2, 3], 0.0)
    core.tick(0.0)            # leader 1

    beat(core, [2, 3], 1.2)
    core.tick(1.2)           # 1 失效 → leader 2, term 2
    assert core.leader_id == 2 and core.term == 2

    # 1 恢復心跳,全員存活。
    beat(core, [1, 2, 3], 1.4)
    ev = core.tick(1.4)
    assert core.leader_id == 2          # 不被搶回
    assert core.term == 2               # 不升 term
    assert any(e.kind is EventKind.NODE_RECOVERED and e.node_id == 1 for e in ev)


# ═══════════════════════════════════════════════════════════════════════════
#  term 單調遞增 / 連續換手
# ═══════════════════════════════════════════════════════════════════════════
def test_term_monotonic_across_failovers():
    """連續換手:每次更替 term 嚴格 +1,絕不回退。"""
    core = RaftFailover()
    beat(core, [1, 2, 3], 0.0)
    core.tick(0.0)
    assert core.term == 1            # leader 1

    # 1 死 → 2
    beat(core, [2, 3], 1.2)
    core.tick(1.2)
    assert core.leader_id == 2 and core.term == 2

    # 2 死 → 3
    beat(core, [3], 2.4)
    core.tick(2.4)
    assert core.leader_id == 3 and core.term == 3


# ═══════════════════════════════════════════════════════════════════════════
#  全機失聯後恢復
# ═══════════════════════════════════════════════════════════════════════════
def test_all_lost_then_recover():
    """全機失聯 → LEADER_LOST、無長機;之後一架恢復 → 它接管,term 續增。"""
    core = RaftFailover()
    beat(core, [1, 2], 0.0)
    core.tick(0.0)
    assert core.leader_id == 1 and core.term == 1

    # 無人送心跳,推進過逾時 → 全失聯。
    ev = core.tick(2.0)
    assert core.leader_id is None
    assert EventKind.LEADER_LOST in kinds(ev)

    # 2 恢復 → 接管。
    beat(core, [2], 3.0)
    ev = core.tick(3.0)
    assert core.leader_id == 2
    assert core.term == 2
    assert EventKind.LEADER_CHANGED in kinds(ev)


# ═══════════════════════════════════════════════════════════════════════════
#  多機同時失效
# ═══════════════════════════════════════════════════════════════════════════
def test_simultaneous_failures():
    """長機與另一機同拍失效 → 由存活者中選新長機,兩者皆發 NODE_LOST。"""
    core = RaftFailover()
    beat(core, [1, 2, 3, 4], 0.0)
    core.tick(0.0)           # leader 1

    # 只有 3、4 續送;1、2 同時失效。
    beat(core, [3, 4], 1.2)
    ev = core.tick(1.2)
    assert core.leader_id == 3            # 存活 {3,4} 中最低 id
    lost = {e.node_id for e in ev if e.kind is EventKind.NODE_LOST}
    assert lost == {1, 2}


# ═══════════════════════════════════════════════════════════════════════════
#  設定驗證
# ═══════════════════════════════════════════════════════════════════════════
def test_config_rejects_takeover_exceeding_deadline():
    """逾時 + tick 週期 > 接管上界 → 設定階段即拒絕,不讓 3 秒承諾默默違規。"""
    with pytest.raises(ValueError):
        FailoverConfig(heartbeat_timeout_s=3.0, tick_period_s=0.5, takeover_deadline_s=3.0)


def test_config_rejects_nonpositive():
    with pytest.raises(ValueError):
        FailoverConfig(heartbeat_timeout_s=0.0)


# ═══════════════════════════════════════════════════════════════════════════
#  確定性
# ═══════════════════════════════════════════════════════════════════════════
def test_deterministic_same_inputs_same_outcome():
    """相同輸入序列 → 兩個獨立核心得到相同長機與 term(確定性,可重現)。"""
    def run():
        c = RaftFailover()
        beat(c, [1, 2, 3], 0.0)
        c.tick(0.0)
        beat(c, [2, 3], 1.2)
        c.tick(1.2)
        return c.leader_id, c.term

    assert run() == run() == (2, 2)


# ═══════════════════════════════════════════════════════════════════════════
#  set_leader(操作員覆寫 / 初始種子)
# ═══════════════════════════════════════════════════════════════════════════
def test_set_leader_seeds_and_bumps_term():
    """對新鮮節點下 set_leader → 成為長機、term+1、事件帶正確 old/new/t。"""
    core = RaftFailover()
    core.heartbeat(2, 0.0)
    ev = core.set_leader(2, t=0.0)
    assert core.leader_id == 2 and core.term == 1
    assert ev is not None and ev.kind is EventKind.LEADER_CHANGED
    assert ev.old_leader is None and ev.new_leader == 2 and ev.t == 0.0


def test_set_leader_same_leader_is_noop():
    """對現任長機重複 set_leader → 回 None、term 不變。"""
    core = RaftFailover()
    core.heartbeat(2, 0.0)
    core.set_leader(2, t=0.0)
    assert core.set_leader(2, t=0.0) is None
    assert core.term == 1


def test_set_leader_override_reports_old_leader():
    """選出長機 1 後覆寫指定 3 → 事件 old_leader==1、new==3、term+1。"""
    core = RaftFailover()
    beat(core, [1, 3], 0.0)
    core.tick(0.0)            # leader 1
    ev = core.set_leader(3, t=0.0)
    assert ev.old_leader == 1 and ev.new_leader == 3
    assert core.leader_id == 3 and core.term == 2


def test_set_leader_rejects_stale_or_unknown_node():
    """拒絕指定不存在或心跳逾時的節點當長機(避免幽靈長機雙重抖動)。"""
    core = RaftFailover()
    with pytest.raises(ValueError):
        core.set_leader(99, t=0.0)          # 從未送心跳
    core.heartbeat(1, 0.0)
    with pytest.raises(ValueError):
        core.set_leader(1, t=5.0)           # age=5s > 逾時 1s


# ═══════════════════════════════════════════════════════════════════════════
#  亂序心跳單調保護
# ═══════════════════════════════════════════════════════════════════════════
def test_out_of_order_heartbeat_ignored():
    """較舊時戳的心跳不得倒退 last_heartbeat(避免把存活機誤判失效)。"""
    core = RaftFailover()
    core.heartbeat(1, 5.0)
    core.heartbeat(1, 2.0)               # 較舊 → 應被忽略
    core.heartbeat(2, 5.5)
    core.tick(5.5)
    assert 1 in core.alive_nodes()       # 若被倒退到 2.0,now=5.5 時 age=3.5 會誤判失效


# ═══════════════════════════════════════════════════════════════════════════
#  LEADER_LOST 事件值
# ═══════════════════════════════════════════════════════════════════════════
def test_leader_lost_reports_old_leader_value():
    """全機失聯 → LEADER_LOST 帶正確 old_leader/node_id,new_leader 為 None。"""
    core = RaftFailover()
    beat(core, [1, 2], 0.0)
    core.tick(0.0)                       # leader 1
    lost = [e for e in core.tick(2.0) if e.kind is EventKind.LEADER_LOST]
    assert len(lost) == 1
    assert lost[0].old_leader == 1 and lost[0].node_id == 1 and lost[0].new_leader is None
    assert core.leader_id is None


# ═══════════════════════════════════════════════════════════════════════════
#  健康度 tiebreak / 未知電量
# ═══════════════════════════════════════════════════════════════════════════
def test_health_tiebreak_falls_back_to_lowest_id():
    """健康度全同(GPS、電量)→ 退回最低 id 的確定性 tiebreak。"""
    core = RaftFailover(priority_key=health_priority_key)
    same = {i: NodeHealth(battery_pct=100.0, gps_ok=True) for i in (3, 5, 2)}
    for i in (3, 5, 2):                   # 刻意非排序插入
        core.heartbeat(i, 0.0, same[i])
    core.tick(0.0)
    assert core.leader_id == 2


def test_unknown_battery_not_preferred():
    """電量未知者不得冒充滿電而優先;已知健康電量(雖 id 較大)勝出。"""
    core = RaftFailover(priority_key=health_priority_key)
    core.heartbeat(1, 0.0, NodeHealth(battery_pct=0.0, gps_ok=True, battery_known=False))
    core.heartbeat(2, 0.0, NodeHealth(battery_pct=80.0, gps_ok=True, battery_known=True))
    core.tick(0.0)
    assert core.leader_id == 2


# ═══════════════════════════════════════════════════════════════════════════
#  單機艦隊 / 節點移除與重加入
# ═══════════════════════════════════════════════════════════════════════════
def test_single_node_fleet_elects_itself():
    """單一倖存機自選為長機;之後失聯 → LEADER_LOST、無長機。"""
    core = RaftFailover()
    core.heartbeat(9, 0.0)
    core.tick(0.0)
    assert core.leader_id == 9 and core.term == 1
    ev = core.tick(2.0)
    assert core.leader_id is None
    assert EventKind.LEADER_LOST in kinds(ev)


def test_remove_leader_node_triggers_reelection():
    """移除現任長機節點 → 下一拍由存活者接管(舊長機在 tick 前為幽靈)。"""
    core = RaftFailover()
    beat(core, [1, 2], 0.0)
    core.tick(0.0)                       # leader 1
    core.remove_node(1)
    assert core.leader_id == 1           # 幽靈長機(tick 前尚未重選)
    beat(core, [2], 0.1)
    ev = core.tick(0.1)
    assert core.leader_id == 2
    assert any(e.kind is EventKind.LEADER_CHANGED and e.old_leader == 1 for e in ev)


def test_node_readd_after_removal():
    """移除後再送心跳 → 節點乾淨重現,可再被選。"""
    core = RaftFailover()
    core.heartbeat(1, 0.0)
    core.tick(0.0)
    core.remove_node(1)
    assert 1 not in core.node_ids()
    core.heartbeat(1, 1.0)
    assert 1 in core.node_ids()


def test_remove_non_leader_no_failover():
    """移除非長機節點 → 長機與 term 不變、不發 LEADER_CHANGED/LEADER_LOST。"""
    core = RaftFailover()
    beat(core, [1, 2, 3], 0.0)
    core.tick(0.0)                       # leader 1
    core.remove_node(3)
    beat(core, [1, 2], 0.1)
    ev = core.tick(0.1)
    assert core.leader_id == 1 and core.term == 1
    assert not any(e.kind in (EventKind.LEADER_CHANGED, EventKind.LEADER_LOST) for e in ev)


# ═══════════════════════════════════════════════════════════════════════════
#  逾時邊界 inclusivity / 邊界抖動
# ═══════════════════════════════════════════════════════════════════════════
def test_freshness_boundary_inclusive():
    """age == 逾時門檻仍新鮮(inclusive);剛超過才失效。"""
    core = RaftFailover()
    core.heartbeat(1, 0.0)
    core.tick(0.0)
    ev = core.tick(1.0)                  # age == 1.0 == 門檻 → 仍存活
    assert 1 in core.alive_nodes()
    assert not any(e.kind is EventKind.NODE_LOST for e in ev)
    ev = core.tick(1.0 + 1e-6)           # 剛超過 → 失效
    assert 1 not in core.alive_nodes()
    assert any(e.kind is EventKind.NODE_LOST for e in ev)


def test_marginal_link_follower_flap_does_not_move_leader():
    """僚機在邊界抖動(失聯→恢復)各觸發一次事件,但長機不換、term 不變。"""
    core = RaftFailover()
    beat(core, [1, 2], 0.0)
    core.tick(0.0)                       # leader 1
    beat(core, [1], 1.2)                 # 長機 1 續送;僚機 2 失聯
    ev1 = core.tick(1.2)
    assert any(e.kind is EventKind.NODE_LOST and e.node_id == 2 for e in ev1)
    assert core.leader_id == 1 and core.term == 1
    beat(core, [1, 2], 1.4)             # 僚機 2 恢復
    ev2 = core.tick(1.4)
    assert any(e.kind is EventKind.NODE_RECOVERED and e.node_id == 2 for e in ev2)
    assert core.leader_id == 1 and core.term == 1


def test_config_rejects_worst_equals_deadline():
    """worst == deadline(零餘裕)亦拒絕,強制保留 Qt 排程/致動餘裕。"""
    with pytest.raises(ValueError):
        FailoverConfig(heartbeat_timeout_s=2.8, tick_period_s=0.2, takeover_deadline_s=3.0)
