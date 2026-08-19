# -*- coding: utf-8 -*-
"""打擊期間的**控制權交接** —— 編隊層讓權給 tot_node。

═══ 為什麼需要這個(2026-08-07,AeroPlan 互動航次 AP2)═══

批次路徑(`stot_trial.sh:425`)交棒的做法是**把編隊層整個收掉**:

    bash "$SR/swarm_ros2_down.sh"   # 交棒前收掉編隊層,避免和 tot_node 搶控制
    sleep 3
    # …然後才起航次用的 tot_node

互動路徑(AeroPlan 地圖右鍵)**沒有這一步** —— 兩層同時活著,於是同一架
飛機被兩個控制器下相反的令。實測 AP2:

    tot_node   : ALIGN,飛 48 km 向西、爬到 perch 400 m
    master_node: `master -> leg 2 (25.07770,121.24738) @ 80 m`(在起飛點繞航線)
                 `leader slowing to 10 m/s [max-consensus]`
    backup_node: `UAV2 -> V-tail slot -> (25.09192,121.24716) @ 50 m`(1 Hz 重發)

編隊層以 1 Hz 重發,壓過 tot_node 的爬升需求 ⇒ 高度一路掉(122→74 m),
飛機在起飛點附近繞圈,`d` 幾乎不減少,整趟必然逾時或觸地。

═══ 設計:軟交接(使用者 2026-08-07 定調)═══

不收掉編隊層,改成**讓權**:

  * tot_node 進 ALIGN/PERCH/DIVE ⇒ 編隊層**停止下令**
  * 但**保持存活**:遙測、心跳、formation_status 照發
  * tot_node 回 STANDBY/DONE ⇒ 編隊層自動收回控制

文獻上這是標準做法:多操作員/多站台系統用**明確的交接協定**移轉載具控制權
(NASA MOMU TP-2018-219875 的 "standardized methods for vehicle handovers";
UAV Navigation 的 GCS 間 handover),而終端導引段本來就該把
「巡航效率」與「近目標調節」的權限分開(arXiv 2606.26315)。

⚠️⚠️ **失效方向刻意選「恢復下令」。**
   狀態逾時(tot_node 掛了、ROS2 圖斷了)⇒ 編隊層**收回**控制。
   反過來設計(逾時就繼續讓權)會讓飛機在沒有任何控制器的情況下滑走 ——
   本專案已經有 [[leader-state-freeze]] 那個「沒人發現的靜默失控」教訓。
   讓權是**主動、可觀測、會自己過期**的,不是 latch。

⚠️ 只擋「下令」,不擋「發布」。編隊層的遙測是 AeroPlan HUD 與離線評分的
   來源之一,停掉會讓打擊全程變成黑箱。

⚠️ 這一份是**唯一**的實作,三個節點都 import 它。抄三份會漂移,而漂移是靜默的。
"""

import json
import os

#: tot_node 的狀態話題(來源:tot_node.py 的 status_pub)。
STATUS_TOPIC = "/swarm/tot_status"

#: 這些相位代表 **tot_node 正在指揮飛機** ⇒ 編隊層必須讓權。
#: 取值與 integrations/rclick_strike.py 的 PHASE_BUSY 相同,改一邊要改兩邊
#: (tests/test_strike_authority.py 有一條迴歸鎖比對)。
BUSY_PHASES = ("ALIGN", "PERCH", "DIVE")

#: 這些相位代表 tot_node 沒有在指揮 ⇒ 編隊層照常。
IDLE_PHASES = ("STANDBY", "INIT", "DONE")

#: 狀態多久沒更新就視為「tot_node 不在了」⇒ 收回控制。
#: 預設 5.0 s:tot_node 待命時 1 Hz、活躍時 20 Hz,5 s 等於連漏 5 筆待命心跳。
#: ⚠️ 不可以設得比 1 s 小 —— 待命是 1 Hz,會一直在讓權/收回之間抖。
DEFAULT_MAX_AGE_S = 5.0


def max_age_s():
    """讓權狀態的有效期。`STRIKE_HANDOFF_MAX_AGE=0` ⇒ **完全關掉讓權**(回退開關)。"""
    try:
        return float(os.environ.get("STRIKE_HANDOFF_MAX_AGE", DEFAULT_MAX_AGE_S))
    except (TypeError, ValueError):
        return DEFAULT_MAX_AGE_S


def parse_phase(data):
    """從 `/swarm/tot_status` 的 String.data 取出 phase。取不到回 None。

    ⚠️ 壞訊息一律回 None ⇒ 上游會當成「沒有讓權」⇒ 編隊層繼續下令。
       這和逾時同一個安全方向:寧可兩個控制器打架(看得見),
       也不要沒有控制器(看不見)。
    """
    try:
        obj = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(obj, dict):
        return None
    ph = obj.get("phase")
    return ph if isinstance(ph, str) else None


def should_yield(phase, age_s, max_age=None):
    """編隊層現在該不該讓權?**純述詞,單元測試打這個。**

    phase   最後一次收到的 tot_node 相位(None = 從沒收過)
    age_s   那一筆的年齡(秒)
    """
    ma = max_age_s() if max_age is None else float(max_age)
    if ma <= 0.0:                 # 回退開關:當作 tot_node 不存在
        return False
    if phase is None:             # 從沒收過 ⇒ 沒有人在打擊
        return False
    if age_s > ma:                # 過期 ⇒ 收回控制(安全方向)
        return False
    return phase in BUSY_PHASES


class StrikeAuthority:
    """把「有沒有讓權」包成一個物件,三個節點共用。

    用法(節點端):

        self._auth = StrikeAuthority(self)              # 自己建訂閱
        ...
        if self._auth.yielded():
            return                                      # 這一 tick 不下令
    """

    def __init__(self, node, on_change=None):
        """`on_change(yielded: bool)` 在狀態翻轉時各叫一次(可選)。"""
        self.node = node
        self._phase = None
        self._t = 0.0
        self._yielded = False
        self._on_change = on_change
        from std_msgs.msg import String
        self._sub = node.create_subscription(
            String, STATUS_TOPIC, self._on_status, 10)

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, msg):
        ph = parse_phase(msg.data)
        if ph is None:
            return                      # 壞訊息不更新時戳,讓它自然過期
        self._phase = ph
        self._t = self._now()

    def yielded(self):
        """現在是不是該停止下令。**每個下令點都要先問這一句。**"""
        age = self._now() - self._t if self._t else 1e9
        y = should_yield(self._phase, age)
        if y != self._yielded:
            self._yielded = y
            self._log(y)
            if self._on_change is not None:
                try:
                    self._on_change(y)
                except Exception:       # 回呼壞掉不該拖垮飛行迴圈
                    pass
        return y

    def phase(self):
        return self._phase

    def _log(self, y):
        lg = self.node.get_logger()
        if y:
            lg.warn(
                "⏸ 讓權給 tot_node(相位 %s)—— 本節點**停止下令**,"
                "但遙測 / 心跳 / formation_status 照發。"
                "tot_node 回 STANDBY/DONE 或狀態逾時 %.1f s 就自動收回。"
                % (self._phase, max_age_s()))
        else:
            why = ("tot_node 相位 %s" % self._phase) if self._phase else "沒有狀態"
            lg.warn("▶ 收回控制(%s)—— 本節點恢復下令。" % why)
