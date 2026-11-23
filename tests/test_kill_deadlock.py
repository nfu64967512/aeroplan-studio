# -*- coding: utf-8 -*-
"""結案前被終結的機體不可以鎖死整趟航次(ALIGNW1,2026-08-24)。

═══════════════════════════════════════════════════════════════════════════
 實際發生的事
═══════════════════════════════════════════════════════════════════════════
ALIGNW1 收到目標後:

    +3.7 s   ☠ UAV4 觸水終結:agl=-0.7m ⇒ 強制 disarm
    +27.7 s  ☠ UAV5 觸水終結:agl=-1.5m ⇒ 強制 disarm
    +261 s   UAV1/2/3/6 全部抵達 slot
    ...
    +1500 s  ✗ 逾時,沒有結案 —— 記為失敗

四架在 261 秒就到齊了。節點又等了 **1239 秒**,等的是兩具已經 disarm、
地速 0.1 m/s、停在海面上不動的殘骸飛完剩下的 30 km。

⚠️⚠️ **終結機制沒有壞**。解除保險正常(+56/+55 m ≥ kill_arm_agl=50)、
   `☠` 兩行都在、disarm 送出去了。壞的是 `self.killed` 寫進去之後
   **全檔沒有任何一個地方讀它**(除了防重複終結),於是四道 all-n 閘門:

       len(self.arr) == self.n          ALIGN → PERCH      ← 這趟死在這
       len(self.ready) == self.n        PERCH 助跑
       len(self.perch_arr) == self.n    進環
       done == self.n                   DONE

   全部變成**數學上不可能成立**的等式。俯衝之前的任何一次終結都會鎖死整趟。

⚠️ 為什麼 CSV 看不出來:那兩架照樣每拍出現、mode 從 straight 升到 max、
   v_cmd 從 104 爬到 150、align_vreq_mps 最後是 298470 m/s。節點一路對著
   殘骸下指令,而每一列都合法。

═══════════════════════════════════════════════════════════════════════════
 這個檔的紅線
═══════════════════════════════════════════════════════════════════════════
1. 零終結時**行為逐位元不變** —— 修法不可以動到健康航次。
2. 全滅時**不可以**推進相位(`len(x) >= 0` 恆真的陷阱)。
3. 退化航次的摘要**必須**帶損失橫幅 —— 4/6 起飛的航次會印出
   「4/4 命中、散布 1.8 s」,看起來比健康航次還好。
4. 存活低於下限要**明確中止**,不是飛一趟 1 架的航次(散布恆 0.0 s)。
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_tot_standby import TotNode, FakeLogger, SRC   # noqa: E402


def mk(n=6, phase="ALIGN", killed=(), impact=(), missed=(), min_live_n=2):
    """造一個只塞受測方法會碰到的欄位的節點。

    ⚠️ 用 __new__ 而不是 __init__ —— 真的 __init__ 要 ROS。這是本 repo
       既有的模式(見 test_tot_standby)。
    """
    o = TotNode.__new__(TotNode)
    o.n = n
    o.phase = phase
    o.pos = {i: (0.0, 0.0, 100.0) for i in range(1, n + 1)}
    o.killed = dict.fromkeys(killed, 1.0)
    o.impact = dict.fromkeys(impact, 1.0)
    o.missed = dict.fromkeys(missed, 1.0)
    o.lost_at = {}
    o.lost_phase = {}
    o.min_live_n = min_live_n
    o.standby_mode = False
    # ⚠️ _impact_summary 會碰到的結算 dict。全部給值而不是靠 getattr 退回 ——
    #    退回會讓摘要在測試裡走一條**生產路徑不會走的分支**。
    o.dive_go = 0.0
    o.dmin = {i: 3.0 for i in impact}
    o.hit_alt = {i: 1.0 for i in impact}
    o.hit_d = {i: 3.0 for i in impact}
    o.hit_gs = {i: 180.0 for i in impact}
    o.miss_why = {}
    o.impact_alt_ok = 30.0
    o.dive_speed = 210.0
    o.dive_angle = 27.0
    o.perch_alt = 3100.0
    o.gate_r = 4700.0
    o.T = (25.0, 121.0)
    o.miss_max = 30.0
    o.impact_reach = 25.0
    o.truth = {}
    o.truth_dmin = {}
    o.truth_hit = {}
    o._log = FakeLogger()
    o.get_logger = lambda: o._log
    return o


# ══════════════════════════════════════════════════════════════════════
#  紅線 1:健康航次一個位元都不能變
# ══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("got_n", range(0, 7))
def test_zero_kill_is_bit_identical_to_the_old_equality(got_n):
    """⭐⭐ 沒有任何終結時,`_gate_all(got)` 必須等同舊的 `len(got) == n`。

    ⚠️ 這一條是這次修改的**安全帶**。四道閘門決定整個相位機的推進,
       改錯了不會報錯 —— 只會讓某一趟提早或延後推進,而那看起來就像
       一次正常的航次變異。
    """
    o = mk()
    got = {i: 1.0 for i in range(1, got_n + 1)}
    assert o._gate_all(got) == (len(got) == o.n), (
        "零終結時 got=%d/%d 的判定與舊等式不一致" % (got_n, o.n))


# ══════════════════════════════════════════════════════════════════════
#  紅線 2:全滅不可以推進
# ══════════════════════════════════════════════════════════════════════

def test_all_dead_does_not_advance_the_phase():
    """⚠️⚠️ `len(got) >= self._n_live()` 在全滅時是 `len(got) >= 0` = **恆真**。

    那會讓相位在**零架飛機**的情況下推進,而每一個下游計數都是 0/0。
    比逾時更糟:逾時至少會被記成失敗。
    """
    o = mk(killed=range(1, 7))
    assert o._n_live() == 0
    assert not o._gate_all({}), "全滅時空的 got 竟然過了閘門"
    assert not o._gate_all({1: 1.0}), "全滅時閘門竟然會過"


def test_a_lost_airframe_shrinks_the_denominator():
    o = mk(killed=(4, 5))
    assert o._n_live() == 4
    assert o._gate_all({1: 1, 2: 1, 3: 1, 6: 1}), (
        "ALIGNW1 的形狀:四架到齊而兩架已終結 —— 這正是卡住 1239 s 的那一刻")
    assert not o._gate_all({1: 1, 2: 1, 3: 1})


def test_impact_and_missed_are_not_losses():
    """⚠️ 俯衝末端觸地被終結是**正常結局**,不是損失。

    分不清楚的話,每一趟成功的航次都會被判成退化。
    """
    o = mk(killed=(1, 2), impact=(1,), missed=(2,))
    assert o._n_live() == 6, "結案的機體被誤算成損失"
    assert not o._lost(1) and not o._lost(2)


# ══════════════════════════════════════════════════════════════════════
#  紅線 3:退化航次不可以讀起來像乾淨的
# ══════════════════════════════════════════════════════════════════════

def test_the_summary_carries_the_loss_banner():
    """⚠️⚠️ 4/6 起飛成功的航次會印出「4/4 命中、散布 1.8 s」。

    沒有橫幅的話,那一行看起來**比健康航次還好**,而且會被跨趟比較拿去用。
    """
    o = mk(phase="DIVE", killed=(4, 5), impact=(1, 2, 3, 6))
    o.lost_at = {4: 1.0, 5: 2.0}
    o.impact = {1: 10.0, 2: 10.5, 3: 11.0, 6: 11.8}
    lines = o._impact_summary()
    assert lines, "摘要是空的"
    # ⚠️ 橫幅在**哨兵之後**([1]),不是 [0]。`_impact_summary()[0]` 是既有契約:
    #    47 個測試與外部 grep 消費者都認第 0 列是哨兵。
    assert lines[0].startswith("★★ 俯衝結案:"), "第 0 列不再是哨兵:%r" % lines[0]
    head = lines[1]
    assert "退化航次" in head, "第 1 列不是損失橫幅:%r" % head
    assert "4" in head and "6" in head, "橫幅沒有寫出 4/6:%r" % head
    assert "不可與健康航次比較" in head


def test_the_closeout_sentinel_keeps_the_full_denominator():
    """⚠️ 哨兵那一行的分母**維持 self.n**。

    既有的 grep 消費者讀的是「6 架裡命中幾架」。把它改成存活數會讓
    一趟 4/6 損失的航次在歷史紀錄裡變成滿分。
    """
    o = mk(phase="DIVE", killed=(4, 5), impact=(1, 2, 3, 6))
    o.lost_at = {4: 1.0, 5: 2.0}
    o.impact = {1: 10.0, 2: 10.5, 3: 11.0, 6: 11.8}
    # ⚠️⚠️ 必須釘在**哨兵那一列本身**([0]),不能對整段 join 之後找 "/6"。
    #    第一版寫成 `"\n".join(lines[1:])` 然後找 "/6" —— 而損失橫幅裡剛好
    #    也有「4/6」⇒ 把哨兵分母改成存活數之後**測試照樣是綠的**。
    #    突變實驗(scratchpad/bite_kill_deadlock.py)抓到了這個假綠燈。
    head = o._impact_summary()[0]
    assert "4/6 命中" in head, (
        "哨兵行的分母不是 self.n:%r\n"
        "    既有的 grep 消費者讀的是「6 架裡命中幾架」;改成存活數會讓\n"
        "    一趟 4/6 損失的航次在歷史紀錄裡變成滿分。" % head)


# ══════════════════════════════════════════════════════════════════════
#  紅線 4:存活不足要明確中止
# ══════════════════════════════════════════════════════════════════════

def test_too_few_survivors_aborts_instead_of_flying_a_meaningless_sortie():
    """⚠️ 1 架的散布**恆為 0.0 s** —— 那是一個看起來完美的數字。"""
    o = mk(killed=(2, 3, 4, 5, 6))
    assert o._n_live() == 1
    assert o._abort_if_too_few(0.0) is True
    assert o.phase == "DONE"
    txt = "\n".join(o._log.lines)
    assert "中止" in txt
    assert "不是" in txt and "逾時" in txt, (
        "中止訊息沒有把「這不是逾時」講清楚 ⇒ 下一個人會去查導引層:\n%s" % txt)


def test_enough_survivors_keeps_flying():
    o = mk(killed=(4, 5))
    assert o._abort_if_too_few(0.0) is False
    assert o.phase == "ALIGN"


def test_abort_does_not_fire_in_standby_or_done():
    """⚠️ STANDBY 沒有『這一發』可中止;DONE 已經結案了。"""
    for ph in ("STANDBY", "DONE"):
        o = mk(phase=ph, killed=range(1, 7))
        assert o._abort_if_too_few(0.0) is False, ph


def test_the_loss_is_announced_once_not_every_tick():
    """⚠️ 這是**整趟航次品質的分水嶺**,不是逐拍狀態。洗版=變成背景雜訊。"""
    o = mk(killed=(4,))
    for _ in range(50):
        o._note_loss(4, 1.0)
    hits = [ln for ln in o._log.lines if "退出這一發" in ln]
    assert len(hits) == 1, "印了 %d 次" % len(hits)
    assert "5/6" in hits[0], "沒有寫出剩餘數:%r" % hits[0]


# ══════════════════════════════════════════════════════════════════════
#  源碼層:確認四道閘門真的都換掉了
# ══════════════════════════════════════════════════════════════════════

def test_no_all_n_gate_survived_the_change():
    """⚠️ 漏掉任何一道,那個相位就還是會死結 —— 而且只在有終結時才發作。

    ⚠️⚠️ 這是**源碼層**的檢查,它只證明「那幾行長什麼樣」
       (見 [[value-locks-beat-source-locks]])。真正的證據是上面的行為測試
       加上 scratchpad/bite_kill_deadlock.py 的突變實驗。
    """
    import re
    # ⚠️ 必須釘在 **if/elif 的條件位置**,不能整檔 grep 字串。
    #    修法旁邊留了「曾經是 `len(self.arr) == self.n`」的說明註解,
    #    整檔 grep 會把那段解釋抓成假紅燈 —— 第一版就是這樣紅的。
    #    ⇒ 一道會被自己的說明文字觸發的守衛,下一個人只會把它刪掉。
    gate = re.compile(
        r"^\s*(?:el)?if\s+.*len\(self\.(arr|ready|perch_arr)\)\s*==\s*self\.n",
        re.M)
    bad = gate.findall(SRC)
    assert not bad, "還有 all-n 閘門沒換:%s" % bad
    done = re.compile(r"^\s*(?:el)?if\s+.*done == self\.n", re.M)
    assert not done.findall(SRC), "DONE 閘門還在用 self.n 當分母"


def test_init_really_creates_the_state_dicts():
    """⭐⭐⭐ `_lost()` 用 getattr 退回空 —— 這一條就是那個退回的**代價**。

    退回是為了測試樁（5 個獨立的樁工廠，逐一補欄位撐不了多久）。但退回
    **正好會把我正在修的失效重造一次**：`killed` 讀不到 ⇒ 永遠沒有損失
    ⇒ 四道閘門又變回 all-n ⇒ 又是 1551 秒的死結，而且**完全靜默**
    （每一趣都會「成功」地不中止）。

    ⇒ 所以退回必須有東西守著。這一條對**真的 __init__ 源碼**下斷言。
    ⚠️ 它是源碼層的（[[value-locks-beat-source-locks]]：改名/搬家會繞過），
       但這裡沒有更好的選擇 —— 要在行為層測它就得跑真的 __init__，而那要 ROS。
    """
    body = SRC.split("def __init__", 1)
    assert len(body) == 2, "找不到 __init__"
    lines = body[1].splitlines()
    for name in ("killed", "lost_at", "impact", "missed"):
        # 真正出現的兩種寫法：
        #     self.killed = {}
        #     self.perch_arr, self.impact = {}, {}      <- 元組賦值
        # ⚠️ 逐行掃描而不寫正規式：窄的模式（只認 `self.x = {}`）會把
        #    元組賦值判成「沒建」—— 那是**假紅燈**，而守衛指著一個沒有
        #    問題的地方時，下一個人會直接把守衛刪掉。
        hit = any(("self." + name) in ln and "= {}" in ln for ln in lines)
        assert hit, (
            "⚠️⚠️ __init__ 沒有建 self.%s ⇒ _lost() 的 getattr 會**永遠**退回空。"
            "  損失偵測靜靜失效,四道閘門變回 all-n,而每一趟都會「成功」地不中止。"
            % name)


def test_killed_and_lost_at_survive_a_target_change():
    """⭐⭐ 殘骸**跨發持續** —— `killed`/`lost_at` 刻意不在 RESET_DICTS 裡。

    被終結的機體是強制 disarm 且停在海面上的,換一個目標不會讓它飛起來。
    清掉的話,第二發會把殘骸重新算進 `_n_live()` ⇒ all-n 閘門又在等一架
    永遠不會到的飛機。**同一個 1551 秒死結,只是換到第二發才發作** ——
    而那更難查,因為第一發看起來完全正常。

    ⚠️ 對照:`impact`/`missed` **要**清,那是這一發的結案判定。
    """
    import tot_node
    for name in ("killed", "lost_at"):
        assert name not in tot_node.RESET_DICTS, (
            "⚠️⚠️ %s 被加進 RESET_DICTS 了。看起來像在補漏,實際是把殘骸\n"
            "    在第二發復活 —— 見本函式的說明。" % name)
    for name in ("impact", "missed"):
        assert name in tot_node.RESET_DICTS, "%s 應該逐發清" % name


# ══════════════════════════════════════════════════════════════════════
#  紅線 5:有結果的航次不可以被自己的守衛丟掉(FLOG1,2026-08-24)
# ══════════════════════════════════════════════════════════════════════

def test_a_sortie_with_results_closes_out_instead_of_aborting():
    """⭐⭐⭐ FLOG1 實測:UAV2 在 +67.6 s **命中紅心**(誤差 20.1 m),
    UAV1 在 +99.6 s 的 DIVE 段觸水終結。

    我的第一版只看 `_n_live() < min_live_n` ⇒ 判成中止、**跳過結案摘要**、
    直接回 STANDBY ⇒ 外層 `rep_interactive.sh` 看不到 `★★ 俯衝結案`,
    把一趟**有命中的航次記成「✗ 逾時,沒有結案」**。

    ⚠️⚠️ 結果被自己的守衛丟掉,**比不加守衛更糟** —— 不加的話至少會逾時,
       而逾時會留下 CSV 讓人事後撈;判成中止則連摘要都沒有。
    """
    o = mk(n=2, phase="DIVE", killed=(1, 2), impact=(2,))
    o.lost_at = {1: 99.6}
    o.lost_phase = {1: "DIVE"}
    o.impact = {2: 67.6}
    o.dmin = {2: 20.1}
    o.hit_alt = {2: 16.0}
    o.hit_d = {2: 20.1}
    o.hit_gs = {2: 173.0}
    assert o._n_live() == 1 < o.min_live_n     # 中止條件成立
    assert o._abort_if_too_few(0.0) is True
    txt = "\n".join(o._log.lines)
    assert "打完了" in txt, "沒有走正常結案路徑:\n%s" % txt
    assert "★★ 俯衝結案:" in txt, (
        "⚠️⚠️ 沒有印結案摘要 ⇒ 外層會把一趟有命中的航次記成逾時:\n%s" % txt)
    assert "1/2 命中" in txt, "哨兵行的分母不對:\n%s" % txt


def test_a_real_abort_still_prints_whatever_results_exist():
    """⚠️ 就算是真正的中止(還有機體沒有終局),已經拿到的結果也要印出來。"""
    o = mk(n=6, phase="ALIGN", killed=(2, 3, 4, 5, 6), impact=(1,))
    o.lost_at = {i: 1.0 for i in (2, 3, 4, 5, 6)}
    o.lost_phase = {i: "ALIGN" for i in (2, 3, 4, 5, 6)}
    o.impact = {1: 10.0}
    o.dmin = {1: 3.0}
    o.hit_alt = {1: 1.0}
    o.hit_d = {1: 3.0}
    o.hit_gs = {1: 180.0}
    assert o._abort_if_too_few(0.0) is True
    txt = "\n".join(o._log.lines)
    assert "中止" in txt
    assert "★★ 俯衝結案:" in txt, "中止也要留下已拿到的結果:\n%s" % txt


def test_the_abort_message_points_at_the_phase_where_the_loss_happened():
    """⚠️⚠️ 第一版寫死「機體在 phase 推進前就損失掉了,請查起飛/編隊層」。

    而 FLOG1 的損失發生在 **DIVE** —— 那句話會把下一個人指向完全錯誤的
    地方,而且它讀起來非常確定。訊息要跟著 `lost_phase` 走。
    """
    o = mk(n=6, phase="DIVE", killed=(2, 3, 4, 5, 6))
    o.lost_at = {i: 1.0 for i in (2, 3, 4, 5, 6)}
    o.lost_phase = {i: "DIVE" for i in (2, 3, 4, 5, 6)}
    o._abort_if_too_few(0.0)
    txt = "\n".join(o._log.lines)
    assert "DIVE" in txt, "沒有指出損失發生在 DIVE:\n%s" % txt
    assert "起飛/編隊層" not in txt, (
        "⚠️ DIVE 段的損失仍然叫人去查起飛/編隊層:\n%s" % txt)

    o2 = mk(n=6, phase="ALIGN", killed=(2, 3, 4, 5, 6))
    o2.lost_at = {i: 1.0 for i in (2, 3, 4, 5, 6)}
    o2.lost_phase = {i: "ALIGN" for i in (2, 3, 4, 5, 6)}
    o2._abort_if_too_few(0.0)
    assert "起飛/編隊層" in "\n".join(o2._log.lines)


def test_lost_phase_survives_a_target_change_like_the_other_two():
    """⚠️ `lost_phase` 與 `killed`/`lost_at` 是同一組事實,清一半更糟。"""
    import tot_node
    assert "lost_phase" not in tot_node.RESET_DICTS
