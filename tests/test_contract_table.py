# -*- coding: utf-8 -*-
"""生效值契約層的迴歸鎖。

═══ 這一層要擋的是什麼 ═══

使用者 2026-08-11 定調:「這些問題一定不能再發生」。單日七次靜默失敗的
**共同結構是同一個**:生效值與意圖值不同,而沒有任何東西在檢查。

⚠️⚠️ 契約層自己也可能靜默失敗,而且更難發現 —— 一張**放得太寬的契約表**
會給出「✅ 全部通過」,比沒有契約更糟:它把「沒檢查」偽裝成「檢查過了」。
所以本檔的重點不是測 `evaluate()` 會不會跑,是測**這張表真的擋得住它宣稱
擋得住的東西**。

最硬的一條是 `TestDeclaredDefaultsAreRejected`:
**節點的 declare_parameter 預設值必須被契約拒絕。**
因為那些預設值全是泡棉時代的,只要任何一條餵值路徑斷掉就會靜靜退回去 ——
若契約放行它們,整層就白做了。
"""
import ast
import math
import json
import os
import pathlib
import re
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCHES = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
sys.path.insert(0, str(PATCHES))

from contract_table import (  # noqa: E402
    BY_NAME, ENV_FOR, INVARIANTS, JET_REFERENCE, SOURCE_FC, SOURCE_NODE,
    TABLE, UNPROVEN, evaluate, rerun_hint, turn_radius_m,
)

TOT = PATCHES / "tot_node.py"
VERIFY = ROOT / "scripts" / "wsl" / "contract_verify.py"
NODE_FILES = {
    "tot_node": TOT,
    "wingman_node": PATCHES / "wingman_node.py",
    "master_node": PATCHES / "master_node.py",
    "backup_node": PATCHES / "backup_node.py",
}


# ── 從節點原始碼抽出 declare_parameter 的預設值 ────────────────────────
# ⚠️ 用 AST 而不是 import:ROS2 在 Windows 上不可用,而這些鎖必須能在開發機
#    上跑。走原始碼也順便把「宣告值」與「生效值」的區別釘死 —— 本檔測的是
#    前者(表的正確性),生效值那一側由 contract_verify.py 現場讀回。
def _declared_defaults(path):
    out = {}
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        fn = node.func
        name = getattr(fn, "id", None) or getattr(fn, "attr", None)
        if name not in ("g", "declare_parameter"):
            continue
        key, val = node.args[0], node.args[1]
        if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
            continue
        if isinstance(val, ast.Constant) and isinstance(val.value, (int, float)) \
                and not isinstance(val.value, bool):
            out[key.value] = float(val.value)
    return out


ALL_DECLARED = {}
for _n, _p in NODE_FILES.items():
    ALL_DECLARED.update(_declared_defaults(_p))


class TestDeclaredDefaultsAreRejected:
    """⭐⭐ 最硬的一條:節點預設值必須被契約拒絕。

    `tot_node.py` 的 declare_parameter 預設值**全部是泡棉時代的**
    (`vmin=14`、`vmax=30`、`dive_speed=42`、`perch_alt=280`、`r_fan=2000`),
    `wingman_node`/`master_node` 的 `formation_alt=50`/`patrol_alt=80` 亦然。

    它們合法、型別正確、不噴任何錯誤。只要任何一條餵值路徑斷掉(env 沒傳、
    launch 少一行、腳本改名),節點就**靜靜退回**去跑原廠常數 —— 這正是
    jet-conversion-never-applied 那幾十趟報廢的形狀。

    ⇒ 契約若放行預設值,整層等於白做。
    """

    #: ⚠️ 白名單只准放「預設值本身就是**現行生產組態**」的參數,
    #:    而且必須寫清楚為什麼。任何其他理由都不可以進這裡。
    INTENTIONAL = {
        "dive_pace": "預設 0.0 的語意是『交給節點自動取 0.8×dive_speed』,"
                     "而那正是現行組態(handoff 缺陷#5 修正後)。"
                     "真正的泡棉禁區是被批次腳本寫死過的 50.0,已列在表上。",
        "cruise_alt_threshold": "預設 3000.0 就是 swarm_ros2_up.sh:532 的生產值。",
        "dive_angle_max": "預設 0.0(= 守門員關閉)就是現行生產組態,而且是"
                          "**刻意**的:它掛在 phase==DIVE 分支、不在 PERCH 段跑,"
                          "而 RPY3 實測 UAV1 在 phase 翻 DIVE 時 γ_req 已 41.5°、"
                          "接下來在 wait 模式外飛到 33.5° 才放行 ⇒ 設 34 會在 41.5° "
                          "立刻觸發,把它放進**比等待更差**的幾何。"
                          "列進契約表是為了讓它從此可見(先前完全沒納管),"
                          "不是為了擋預設值。"
                          "⚠️ γ_req 檢查前移到 PERCH 之後,這條白名單要撤掉、下界改 28。",
        "perch_band": "預設 40.0 就是 swarm_ros2_up.sh:292 的 PERCH_BAND:-40.0 —— 餵值路徑斷掉時退回的值與生產值相同 ⇒ 沒有靜默行為差。列進契約表是為了讓它可見:它量的是 |z − perch_of(i)| 而飛機被指到的是 hold_alt_of(i),HOLD_ALT>0 時這個閘門永遠不成立且完全無聲。",
        "perch_settle": "預設 25.0 就是 swarm_ros2_up.sh:293 的 PERCH_SETTLE:-25.0,同上。列進契約表是因為 dive_from_runup 開著時它同時是**陡俯角參數的重送視窗長度**(視窗 × 20 Hz = 重送次數),而那組參數送不進去是靜默的。",
        "release_at_ring": "預設 False(0.0)= 舊行為,而 S1 之外的所有批次都應該跑舊行為 ⇒ 預設值本身就是生產組態。開關要顯式打開才生效,契約 log 會印出來。",
        "dive_from_runup": "同 release_at_ring:預設 False = 逐位元舊行為。",
        "wait_goto_radius": "同上:預設 False = 逐位元同 RRD 組態。",
        "mpng_k_eps": "預設 0.0 是**合法值**而不是「沒設」:它代表走論文建議式 "
                      "K = 40/(r₀·tgo₀)(放行當拍鎖存),而那正是本輪的生產組態 "
                      "⇒ 餵值路徑斷掉時退回的值與生產值相同,沒有靜默行為差。"
                      "⚠️ 但這條白名單只在「本輪用自動推導」時成立:一旦有人開始"
                      "**掃增益**,餵值路徑斷掉就會靜默退回自動值。那時的保護不是"
                      "這個區間,而是 contract_verify 的讀回(NODE_PARAMS 已列入"
                      "mpng_k_eps/mpng_k_s ⇒ 生效值會被印出來對照)。"
                      "區間 [0, 1e-2] 擋的是量級/單位寫錯:1e-2 代進 N=4、V=161.7 "
                      "⇒ 1046 m/s²,比 1 g 夾制大兩個數量級。",
        "mpng_k_s": "同 mpng_k_eps。",
        "tgo_predict_dt": "預設 0.5 就是 swarm_ros2_up.sh 的 TGO_PREDICT_DT:-0.5。"
                          "它是**節流間隔**,餵值路徑斷掉時退回的值與生產值相同 ⇒ "
                          "沒有靜默行為差。列進契約表是為了讓它在事後可見"
                          "(RPY3 的時基污染就是經由這個快取傳播的),不是為了擋預設值。",
    }

    @pytest.mark.parametrize("c", [c for c in TABLE if c.source == SOURCE_NODE],
                             ids=lambda c: c.name)
    def test_the_declared_default_does_not_pass(self, c):
        if c.name not in ALL_DECLARED:
            pytest.skip(f"{c.name} 不是節點 declare_parameter 參數")
        default = ALL_DECLARED[c.name]
        rep = evaluate({c.name: default}, require=[c.name])
        offending = [v for v in rep.violations if v.name == c.name]
        if c.name in self.INTENTIONAL:
            assert not offending, (
                f"{c.name} 在白名單裡(理由:{self.INTENTIONAL[c.name]}),"
                f"預設值 {default} 卻被擋下 —— 白名單或契約其中之一過期了")
            return
        assert offending, (
            f"⚠️⚠️ {c.name} 的節點預設值 {default} **通過了契約**。\n"
            f"    區間 [{c.lo}, {c.hi}]、泡棉禁區 {c.foam}。\n"
            f"    代表餵值路徑斷掉時契約層不會吭聲 —— 這一層就白做了。\n"
            f"    修法:收緊區間,或把 {default} 填進該列的 foam 欄。")

    def test_foam_column_tracks_the_real_default(self):
        """foam 欄若填的是節點預設值,兩邊必須同步。

        ⚠️ 有人改了 declare_parameter 的預設而沒改表 ⇒ 禁區指向一個
           不存在的值 ⇒ **禁區檢查靜默失效**。這是契約層自己的靜默失敗。
        """
        drift = []
        for c in TABLE:
            if c.foam is None or c.name not in ALL_DECLARED:
                continue
            default = ALL_DECLARED[c.name]
            # dive_pace 的 foam 是歷史寫死值而非預設值,不在此鎖範圍
            if c.name in ("dive_pace",):
                continue
            if abs(c.foam - default) > 1e-9:
                drift.append(f"{c.name}: 表填 foam={c.foam},節點預設卻是 {default}")
        assert not drift, "泡棉禁區與節點預設脫鉤:\n  " + "\n  ".join(drift)


class TestFoamIsCaughtEvenInsideTheBand:
    """⚠️ 三個泡棉值**落在噴射區間內**:dive_angle 25(區間 20–60)、
    impact_reach 40(10–60)、half_angle 30(15–40)。

    單靠區間檢查對它們是瞎的 —— 這正是「禁區」要獨立成一欄的理由。
    dive_angle 尤其兇險:泡棉 25 與噴射 27 只差 2 度。
    """

    INSIDE_BAND = ["dive_angle", "impact_reach", "half_angle"]

    @pytest.mark.parametrize("name", INSIDE_BAND)
    def test_the_band_alone_would_let_it_through(self, name):
        c = BY_NAME[name]
        assert c.lo is not None and c.hi is not None
        assert c.lo <= c.foam <= c.hi, (
            f"{name} 的泡棉值 {c.foam} 已不在區間 [{c.lo}, {c.hi}] 內 —— "
            f"這條測試的前提沒了,請改用區間檢查並更新 INSIDE_BAND")

    @pytest.mark.parametrize("name", INSIDE_BAND)
    def test_but_the_forbidden_column_catches_it(self, name):
        rep = evaluate({name: BY_NAME[name].foam}, require=[name])
        kinds = {v.kind for v in rep.violations if v.name == name}
        assert "foam" in kinds, f"{name} 的泡棉值沒有被禁區欄擋下"


class TestReferenceValuesSatisfyTheirOwnContract:
    """建議值必須自己過得了契約。

    ⚠️ 否則違規訊息會印出一段**契約自己會拒絕**的重跑指令,
       使用者照貼、再被擋一次,然後合理地開始不信任這一層。
    """

    @pytest.mark.parametrize("name", sorted(JET_REFERENCE))
    def test_the_suggested_value_passes(self, name):
        if name not in BY_NAME:
            pytest.skip(f"{name} 不在契約表")
        rep = evaluate({name: JET_REFERENCE[name]}, require=[name])
        bad = [v for v in rep.violations if v.name == name]
        assert not bad, (f"建議值 {name}={JET_REFERENCE[name]} 自己違反契約:"
                         f"{[v.want for v in bad]}")

    def test_unproven_suggestions_are_flagged(self):
        """沒有實測背書的建議值必須標記出來。

        formation_alt / patrol_alt 目前**沒有任何達標航次**背書
        (現行組態就是違規的)。把它們與已驗證值混在一起呈現,
        等於製造下一個「以為修好了」。
        """
        hint = rerun_hint(evaluate({"formation_alt": 50.0},
                                   require=["formation_alt"]).violations)
        assert "沒有實測背書" in hint
        assert UNPROVEN, "UNPROVEN 空了 —— 是真的都驗過了,還是有人順手清掉?"


class TestMissingIsAViolation:
    """⚠️ 讀不到 ≠ 跳過。

    `preflight_check.py` 檔頭第 1 條就是這個形狀:`JET_CDP` 為空 ⇒ 整段
    高速改裝被跳過 ⇒ 幾十趟跑原廠阻力而毫無訊息
    (jet-conversion-never-applied)。「沒讀到所以算過」是這一整類 bug 的核心。
    """

    def test_empty_input_fails_everything(self):
        rep = evaluate({})
        assert not rep.ok
        assert {v.kind for v in rep.violations} >= {"missing"}
        assert len(rep.missing) == len(TABLE)

    def test_nan_counts_as_missing(self):
        """⚠️ `.nan` 在本專案是**合法的參數值**(target_amsl 用它表示未給)。
        它絕不可以被當成「有讀到」而放行。"""
        rep = evaluate({"vmin": float("nan")}, require=["vmin"])
        assert [v.kind for v in rep.violations if v.name == "vmin"] == ["missing"]

    def test_a_passing_config_really_passes(self):
        """反向鎖:全套正確值必須乾淨通過,否則這層會被當成雜訊關掉。"""
        good = dict(JET_REFERENCE)
        good.update({"formation_alt": 3100.0, "patrol_alt": 3100.0,
                     "pn_los_floor": 1.0, "pn_nz_min": -1.0,
                     "AIRSPEED_MIN": 25.0, "AIRSPEED_MAX": 230.0, "THR_MAX": 100.0,
                     "hold_radius": 1300.0, "gate_r_min": 2500.0,
                     "gate_r_max": 6000.0})
        rep = evaluate(good)
        assert rep.ok, "全套正確值卻被擋:\n" + rep.text()


class TestInvariantsFireOnHistoricalFailures:
    """每條不變式都對照一次**實測踩過**的組態。

    ⚠️ 不變式若只是「看起來合理」而沒有實測對照,它遲早會因為誤報而被關掉。
    """

    BASE = dict(vmin=95.0, vmax=150.0, dive_speed=210.0, dive_pace=0.0,
                perch_alt=3100.0, dive_angle=27.0,
                # 2026-08-13:5600 當日被 RPO 實測反證(gate 落進守環擺盪帶),
                # 4800 在名目環 6084 − hold 1300 = 4784 之下只差 −16 m ⇒ 取 4600。
                gate_r=4700.0,
                gate_r_min=2500.0,
                gate_r_max=6000.0, AIRSPEED_MIN=25.0, AIRSPEED_MAX=230.0,
                hold_radius=1300.0, formation_alt=3100.0, READY_AGL=30.0)

    def _fire(self, **over):
        v = dict(self.BASE); v.update(over)
        return {x.name for x in evaluate(v, require=[]).violations}

    def test_base_config_is_clean(self):
        assert self._fire() == set(), "基準組態不該觸發任何不變式"

    def test_foam_dive_pace_saturates_the_sync_loop(self):
        """handoff 缺陷#5:DIVE_PACE=50.0 ≪ vmin 95 ⇒ 同步迴路全程飽和。
        修掉之後散布 25.00 → 0.78/2.40 s。"""
        assert "dive_pace ≥ vmin" in self._fire(dive_pace=50.0)

    def test_pace_pinned_to_the_ceiling_also_loses_authority(self):
        """handoff §4.5:修配速後全部貼上限 210 ——『想更快但追不上』,
        同步迴路一樣沒有權限。貼頂與貼底都要擋。"""
        assert "dive_pace ≤ dive_speed(留餘裕)" in self._fire(dive_pace=209.0)

    def test_rpl3_hold_radius_is_smaller_than_the_turn_circle(self):
        """⚠️⚠️ RPL3 實測 hold_radius=400 vs 迴轉半徑 1252 m。
        tot_node **印了 ERROR 然後照跑** —— 這就是契約層存在的理由。"""
        assert "hold_radius ≥ 物理迴轉半徑" in self._fire(hold_radius=400.0)

    def test_the_turn_radius_matches_what_the_node_prints(self):
        """數字必須與 tot_node 自己印的一致,否則兩邊會各說各話。"""
        got = turn_radius_m(95.0, 3100.0)
        assert abs(got - 1252.0) < 5.0, f"迴轉半徑 {got:.0f} 與節點印的 1252 不符"

    def test_speeds_below_the_flight_controller_floor_are_rejected(self):
        """catch-speed-rejected-by-fc:低於 AIRSPEED_MIN 的指令被
        MAV_RESULT=4 **整發拒收**(不是夾住),而且零告警。"""
        assert "vmin ≥ AIRSPEED_MIN" in self._fire(vmin=14.0, AIRSPEED_MIN=25.0)

    def test_speeds_above_the_ceiling_are_rejected(self):
        """catch_speed 157 > AIRSPEED_MAX 150 ⇒ 追趕通道從未生效。"""
        assert "vmax ≤ AIRSPEED_MAX" in self._fire(vmax=157.0, AIRSPEED_MAX=150.0)

    def test_an_unreachable_ready_gate_times_the_whole_trial_out(self):
        """⚠️ 門檻不可達整趟逾時且無訊息(RPB1/RPC1)。
        與 handoff 缺陷#4(門檻太低、停在跑道上就放行)是同一條的兩端。"""
        assert "READY_AGL < formation_alt" in self._fire(READY_AGL=200.0,
                                                         formation_alt=50.0)

    def test_gate_r_outside_its_own_bounds(self):
        """u1-descent-authority:gate_r/環半徑 ≤0.446 的十趟殘餘 506–2499 m。"""
        assert "gate_r ≥ gate_r_min" in self._fire(gate_r=1200.0)


class TestHandoverGeometry:
    """⭐ 交接幾何:拉大 gate_r 的**目的**必須寫成契約。

    ⚠️⚠️ 這裡擋的不是 gate_r 本身,是**關係**:有人調高 perch_alt 或調小
    dive_angle 之後,需求俯角會無聲地飄回失敗帶,而 gate_r 看起來完全沒動。
    又是一次「單看每個參數都合法,壞的是關係」。
    """

    BASE = dict(perch_alt=3100.0, dive_angle=27.0, gate_r=4700.0,
                gate_r_min=2500.0, gate_r_max=6000.0)

    def _fire(self, **over):
        v = dict(self.BASE); v.update(over)
        return {x.name for x in evaluate(v, require=[]).violations}

    def test_the_chosen_value_is_clean(self):
        assert self._fire() == set(), "gate_r=4700 的採用值不該觸發任何不變式"

    def test_pushing_gate_r_onto_the_ring_is_rejected(self):
        """⚠️ 拉大 gate_r **有上界**,不是越大越好:貼上放行距離會讓 PN
        在放行瞬間就接手而退化(u1-descent-authority 用 AD1 佐證)。
        這是本組唯一的**硬**約束 —— 它有物理意義而且擋得住真東西。"""
        assert "gate_r < 環半徑(留餘裕)" in self._fire(gate_r=6000.0)

    def test_the_feasible_window_is_not_empty(self):
        """契約必須有解 —— 否則使用者會被逼著關掉它。"""
        ok = [g for g in range(3000, 6200, 20)
              if not self._fire(gate_r=float(g), hold_radius=1300.0)]
        assert ok, "可行窗口是空的"

    def test_gate_inside_the_loiter_swing_is_rejected(self):
        """⚠️⚠️ RPO 實測(2026-08-13)燒出來的一條,代價是三趟航次。

        gate 5600 與 hold_radius 1300 **各自都通過所有既有檢查**,相乘卻毀掉
        交棒幾何:守環擺盪 4847–7447 m 把 gate 包在裡面 ⇒ 放行時飛機常已在
        gate 內側 ⇒ 當場交棒 ⇒ 交棒距離從 ±11 m 炸成 ±1075 m,真落地 5/12 → 1/6。
        """
        assert "gate_r < 環半徑 − hold_radius" in \
            self._fire(gate_r=5600.0, hold_radius=1300.0)

    def test_the_old_combination_would_have_passed(self):
        """反向鎖:舊組態(gate 4800 / hold 400)每架都乾淨穿越,不可被誤擋。"""
        assert "gate_r < 環半徑 − hold_radius" not in \
            self._fire(gate_r=4800.0, hold_radius=400.0)

    def test_the_recommended_combination_passes(self):
        """hold_radius=1300(可飛的守環圈)配 gate_r=4700 必須通過 ——
        1300 讓交接高度散布腰斬(1224→631 m),那一項是有效的,要留住。"""
        assert self._fire(gate_r=4700.0, hold_radius=1300.0) == set()

    def test_neither_parameter_alone_looks_wrong(self):
        """⭐ 本條是這一整層存在的理由:**兩個單獨看都合法的值**。

        gate 5600 單獨看:在 [2500, 6000] 內、< 0.95×環(5780)⇒ 全部通過。
        hold 1300 單獨看:≥ 物理迴轉半徑 1253 ⇒ 通過。
        只有把兩者放在一起才看得出災難。單參數檢查對這種事永遠是瞎的。
        """
        assert self._fire(gate_r=5600.0, hold_radius=400.0) == set()
        assert self._fire(gate_r=4700.0, hold_radius=1300.0) == set()
        assert self._fire(gate_r=5600.0, hold_radius=1300.0) != set()

    def test_gamma_is_reported_but_never_blocks(self):
        """⚠️⚠️ 本組最重要的一條 —— 記錄一個**被撤掉**的設計決定。

        一度想把「需求俯角 ≤ 上限」做成硬性不變式。**不能做**:

            TWO1     gate 4800 / perch 3100 ⇒ γ_nominal 32.9° ⇒ 2/2 達標
            RPK/RPL  gate 4800 / perch 3100 ⇒ γ_nominal 32.9° ⇒ 2/6

        同一組設計幾何、結果天差地遠 ⇒ γ_nominal 零鑑別力。
        門檻設 29.1° 會**誤擋 TWO1**(實測成功過的組態);設 33° 則兩者都過。
        而這一層最不能有的就是誤報 —— 擋掉已知可行的組態,人就會關掉它。

        ⇒ γ 留作報告量寫進快照,不進 INVARIANTS。
        """
        from contract_table import INVARIANTS, required_gamma_deg
        # 就算 γ 大到離譜,也不可以有任何不變式因此觸發
        wild = self._fire(perch_alt=3600.0, gate_r=3000.0)
        assert not any("俯角" in n for n in wild), \
            f"需求俯角不該成為阻擋條件,卻觸發了:{wild}"
        assert not any("gamma" in getattr(f, "__name__", "") for f in INVARIANTS)
        # 但它必須算得出來(供快照與事後判讀)
        assert required_gamma_deg(3100.0, 5600.0) < required_gamma_deg(3100.0, 4800.0)

    def test_raising_gate_r_lowers_the_required_angle(self):
        """拉大 gate_r 的作用機制:不是多給時間,是**降低陡度**。
        這條鎖住方向 —— 若哪天式子被改成反的,這裡會紅。"""
        from contract_table import required_gamma_deg
        angles = [required_gamma_deg(3100.0, g) for g in (4800, 5200, 5600, 5780)]
        assert angles == sorted(angles, reverse=True), f"γ 沒有隨 gate_r 單調下降:{angles}"

    def test_the_measured_gamma_split_is_recorded(self):
        """實測分界要留在程式碼裡,不是只留在對話或交接文件裡 ——
        即使它現在不用來阻擋,事後判讀仍然要靠它。"""
        from contract_table import PERCH_OVERSHOOT, PN_GAMMA_MAX_DEG
        assert 29.1 < PN_GAMMA_MAX_DEG < 33.8, (
            "上限必須落在『全中最高 29.1°』與『全敗最低 33.8°』之間")
        assert PERCH_OVERSHOOT > 1.0, (
            "交接高度實測 2286–3510 會超過名目 perch_alt 3100,超調係數不可為 1")


class TestTheHistoricalConfigWouldHaveBeenCaught:
    """⚠️⚠️ RPK/RPL 那九趟 n=2 跑的組態**本身就違規**,而且沒有任何東西擋。

    這不是假想案例,是 2026-08-13 之前 `swarm_ros2_up.sh` 的實際生效值:
    formation_alt=50 / patrol_alt=80 是泡棉高度(噴射機巡航在 3100 m),
    hold_radius=400 小於物理迴轉半徑 1253 —— 而 tot_node **每次開機都印
    ERROR 吼 hold_radius**,九趟照跑。

    ⇒ 這一組值留在測試裡當**歷史證據**:契約層若哪天放行它們就是退化了。
    """

    LIVE = dict(vmin=95.0, vmax=150.0, dive_speed=210.0, dive_pace=0.0,
                # 2026-08-13:GATE_R 預設已由 4800 改為 5600。
                # 舊值被擋的迴歸鎖在 TestHandoverGeometry::test_the_old_4800_is_rejected。
                gate_r=4700.0, gate_r_min=2500.0, gate_r_max=6000.0,
                perch_alt=3100.0, r_fan=35000.0, dive_angle=27.0, reach=2400.0,
                perch_reach=260.0, impact_reach=25.0, half_angle=25.0,
                formation_alt=50.0, patrol_alt=80.0, cruise_alt_threshold=3000.0,
                READY_AGL=30.0, AIRSPEED_MIN=25.0, AIRSPEED_MAX=230.0,
                THR_MAX=100.0, hold_radius=400.0,
                # ⭐ 2026-08-14:互動路徑的兩個陡俯角開關都是關的,
                #    而 TWO1 走的 run_west.sh 是 1.0 / −1.0。
                pn_los_floor=0.0, pn_nz_min=0.05,
                # ⭐ 2026-08-14 補。tgo_from_predictor 在 2026-08-13 還不存在
                #    (T1 是隔天做的),用 0.0 = 「等同沒這個功能」才是史實。
                #    predict_hz=0.0 則是**一路到 RPY3 都沒變過**:S9 做好之後
                #    從沒接線,所以歷史組態這一欄本來就是 0。
                tgo_from_predictor=0.0, tgo_predict_dt=0.5, predict_hz=0.0,
                # ⭐ 幾何守門員 RPY3 當時是關的 —— 78.5 s 散布的直接成因。
                dive_angle_max=0.0,
                # ⭐ 2026-08-15:進環時間與 ITCG 都是 RPZ1/RQA 之後才有的機制,
                #    歷史組態全部是 0(= 不存在)。
                tgo_ring_transit=0.0,
                itcg_on=0.0, itcg_gain=0.0, itcg_a_max_g=0.0,
                # ⚠️ 兩個先前完全隱形的參數。歷史組態的實際生效值:
                #    pn_vmax 有被餵(210),SCALING_SPEED 從沒設過(原廠 15)。
                pn_vmax=210.0, SCALING_SPEED=15.0,
                # ⭐ 2026-08-18 加入:S1 的兩顆開關當時不存在 ⇒ 史實是 0。
                #    ⚠️ 它們**不會**出現在違規集合裡:契約區間是 [0,1],
                #    0 合法。列進表是為了讓生效值 log 印得出來(可見性),
                #    不是為了擋值 —— 前三批白跑有兩批的症狀就是
                #    「開關看起來開著,但沒有任何一行證明它生效」。
                release_at_ring=0.0, dive_from_runup=0.0, wait_goto_radius=0.0,
                # ⭐ 2026-08-19 加入:MPNG(Jiang 2021 Eq.41)當時不存在
                #    ⇒ 史實是 0。⚠️ 它**不會**出現在違規集合裡:區間 [0,1],
                #    0 合法。列進來只是為了讓「這一趟跑的是哪一條律」可見 ——
                #    整個 ITCG 通道 80,760 個樣本全零,而先前沒有任何一欄
                #    能告訴你那是「不需要出力」還是「根本沒跑」。
                mpng_on=0.0,
                # ⚠️ 兩個增益的史實也是 0(當時不存在)。它們**不會**出現在
                #    違規集合裡:0.0 = 走論文建議式,是合法值,不是泡棉禁區
                #    (與 itcg_gain 的 0.0 相反 —— 那一列 0 代表「開了但沒效果」)。
                mpng_k_eps=0.0, mpng_k_s=0.0,
                # ⭐ 2026-08-18 加入:這兩個一路都是 40/25,歷史組態不違規。
                perch_band=40.0, perch_settle=25.0)

    def test_it_does_not_pass(self):
        assert not evaluate(self.LIVE).ok

    def test_the_three_known_violations_are_named(self):
        names = {v.name for v in evaluate(self.LIVE).violations}
        assert "formation_alt" in names
        assert "patrol_alt" in names
        assert "hold_radius ≥ 物理迴轉半徑" in names

    def test_everything_else_is_clean(self):
        """⚠️ 反向鎖:若這裡開始冒出第四項,不是表變嚴了就是這組歷史值被改了 ——
        兩種都必須有人看一眼,不可以默默接受。"""
        names = {v.name for v in evaluate(self.LIVE).violations}
        assert names == {"formation_alt", "patrol_alt", "hold_radius ≥ 物理迴轉半徑",
                         # ⭐ 2026-08-14 加入:互動路徑的兩個陡俯角開關都是關的,
                         #    而唯一達標的 TWO1 走 run_west.sh(1.0 / −1.0)。
                         #    ⇒ RPK/RPL/RPO/RPQ 二十四架次全部在關閉狀態下飛。
                         "pn_los_floor", "pn_nz_min",
                         # ⭐ 2026-08-14 加入:影子預測自 S9 之後就沒接線
                         #    (swarm_ros2_up.sh 從來沒有 -p predict_hz),
                         #    所以歷史組態這一欄一直是 0 ⇒ 校準過的預測器
                         #    到 RPY3 為止一趟都沒真的跑過,而且全程無告警。
                         # ⭐ 2026-08-14 加入:幾何守門員預設 0.0(關閉)⇒
                         #    放行只剩 perch_settle 25 s 逾時、完全不看幾何。
                         #    RPY3 的 78.5 s 散布由此而來。
                         "predict_hz",
                         # ⭐ 2026-08-15 加入:把進環時間併入 t_go 是 RPZ1
                         #    之後才有的機制,歷史組態沒有 ⇒ 必然違規。
                         "tgo_ring_transit",
                         # ⭐ 2026-08-15 加入:ITCG 的兩個增益在歷史組態是 0。
                         #    「開了但沒效果」是禁區 —— 這是最難查的一種失效,
                         #    契約會顯示啟用、log 一切正常、飛出來只是沒改善。
                         "itcg_gain", "itcg_a_max_g"}, \
            f"歷史組態的違規集合變了:{sorted(names)}"

    def test_the_fixed_config_now_passes(self):
        """2026-08-13 的三項修正必須讓契約乾淨通過 ——
        否則使用者改了還是跑不了,這一層就從守門變成路障。"""
        fixed = dict(self.LIVE)
        fixed.update({"formation_alt": 3100.0, "patrol_alt": 3100.0,
                      "hold_radius": 1300.0,
                      "pn_los_floor": 1.0, "pn_nz_min": -1.0,
                      "predict_hz": 2.0, "tgo_ring_transit": 1.0,
                      "itcg_gain": 1.0, "itcg_a_max_g": 0.4})
        rep = evaluate(fixed)
        assert rep.ok, "修正後仍不通過:\n" + rep.text()

    @pytest.mark.parametrize("var,want", [
        ("FORM_ALT", 3100.0), ("PATROL_ALT", 3100.0), ("HOLD_RADIUS", 1300.0),
    ])
    def test_the_launch_script_actually_carries_the_fix(self, var, want):
        """⚠️⚠️ 契約通過不代表**腳本真的改了** —— 這條直接讀啟動腳本。

        `hold_radius` 尤其要鎖:2026-08-05 有人補了 `-p hold_radius` 通路
        卻把預設留在 400(理由寫「不改行為」)⇒ 通路補了、值沒改,
        節點的 ERROR 又繼續吼了八天。**補通路不等於修好。**
        """
        up = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(
            encoding="utf-8")
        # ⚠️ PATROL_ALT 的預設是**巢狀**的:`${PATROL_ALT:-${FORM_ALT:-3100.0}}`
        #    ⇒ 只抓 `VAR:-數字` 會抓不到,要允許中間夾一層 `${...:-`。
        m = re.search(rf"{var}:-(?:\$\{{[A-Z_]+:-)?([0-9.]+)", up)
        assert m, f"swarm_ros2_up.sh 找不到 {var} 的預設值"
        assert float(m.group(1)) == want, \
            f"{var} 預設是 {m.group(1)},應為 {want}"

    def test_hold_radius_is_actually_passed_to_the_node(self):
        """⚠️ 先前 swarm_ros2_up.sh **完全沒有傳 hold_radius** ⇒ 互動路徑
        一路吃 tot_node 的宣告預設 400。有值但沒通路 = 沒有值。"""
        up = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(
            encoding="utf-8")
        assert "-p hold_radius:=" in up, "沒有把 hold_radius 傳給 tot_node"


class TestSentinelsAgree:
    """⚠️ 哨兵字串改一邊沒改另一邊 ⇒ 契約行永遠讀不到 ⇒ 全部判 missing,
    而「全部讀不到」看起來像「節點沒起來」—— 歸因會歪到完全錯的地方。"""

    def _const(self, path, name):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
                for t in node.targets:
                    if isinstance(t, ast.Name) and t.id == name:
                        return node.value.value
        raise AssertionError(f"{path.name} 找不到 {name}")

    def test_both_sides_use_the_same_string(self):
        assert self._const(TOT, "CONTRACT_SENTINEL") == \
               self._const(VERIFY, "CONTRACT_SENTINEL")

    def test_the_sentinel_has_no_parentheses(self):
        """⚠️ tot_node.py:91 已經記取過教訓:全形/半形括號不一致是
        **看不出來**的失配。不用括號就沒有這個失效模式。"""
        s = self._const(TOT, "CONTRACT_SENTINEL")
        for ch in "()()[]【】":
            assert ch not in s, f"哨兵含括號 {ch!r} —— 全半形失配風險"

    def test_make_artifacts_still_scrapes_it(self):
        """`make_artifacts.sh` 用 `grep -E '生效值|抵達判定'` 撈進 summary.md。
        哨兵必須仍被那個式子撈到,否則契約行不會進產物。"""
        s = self._const(TOT, "CONTRACT_SENTINEL")
        script = (ROOT / "scripts" / "wsl" / "make_artifacts.sh").read_text(
            encoding="utf-8")
        assert "生效值" in script and "生效值" in s


class TestNodeEmitsEveryNodeOwnedContractValue:
    """契約層只能檢查**讀得到**的東西。

    ⚠️ 舊的「tot_node 生效值:」那一行涵蓋不全(缺 dive_speed / dive_pace /
       gate_r / dive_angle)—— 而那幾項恰好是 dive-pace-saturates-vmin 與
       u1-descent-authority 的主角。**想比對也讀不到** = 契約對它們是瞎的。
    """

    def _emitted_keys(self):
        tree = ast.parse(TOT.read_text(encoding="utf-8"), filename=str(TOT))
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == "_contract_values":
                return {e.elts[0].value for e in ast.walk(node)
                        if isinstance(e, ast.Tuple) and len(e.elts) == 2
                        and isinstance(e.elts[0], ast.Constant)
                        and isinstance(e.elts[0].value, str)}
        raise AssertionError("tot_node.py 找不到 _contract_values()")

    def test_every_tot_node_contract_parameter_is_emitted(self):
        emitted = self._emitted_keys()
        owned = {c.name for c in TABLE
                 if c.source == SOURCE_NODE and c.name in ALL_DECLARED
                 and c.name in _declared_defaults(TOT)}
        missing = owned - emitted
        assert not missing, (
            f"tot_node 擁有但沒印進契約行:{sorted(missing)} ⇒ "
            f"contract_verify 讀不到 ⇒ 一律判 missing")

    def test_the_invariant_helpers_are_emitted_too(self):
        """不變式需要的輔助量(本身不在契約表裡)同樣必須印出來。"""
        emitted = self._emitted_keys()
        for aux in ("hold_radius", "gate_r_min", "gate_r_max"):
            assert aux in emitted, f"{aux} 沒印 ⇒ 對應的不變式永遠不會判定"


class TestEnvMappingIsComplete:
    """違規訊息要能印出**可以直接貼**的重跑指令。

    ⚠️ 「印了錯誤但不告訴你怎麼修」等於逼人去翻腳本,而翻錯就是下一次靜默失敗。
    """

    def test_every_suggestible_parameter_has_an_env_var(self):
        missing = [n for n in JET_REFERENCE if n not in ENV_FOR]
        assert not missing, f"有建議值卻沒有對應環境變數:{missing}"

    def test_the_env_vars_actually_exist_in_the_launch_script(self):
        """⚠️ 環境變數名打錯 = 設了沒有用,而且**零告警** ——
        正是 effective-value-contract 記的那一整類。所以要對著腳本驗。"""
        # ⚠️ 必須掃**三支**。節點參數在 swarm_ros2_up.sh,但**飛控 PARM**
        #    (SOURCE_FC,例如 SCALING_SPEED)是由 swarm_sitl_up.sh 寫進
        #    jet_fast.parm 的 —— 只掃 ros2 那支會把「其實有設」誤報成
        #    「環境變數不存在」,而那正是這條測試要防的相反錯誤。
        blob = "".join(
            (ROOT / "scripts" / "wsl" / f).read_text(encoding="utf-8")
            for f in ("swarm_ros2_up.sh", "swarm_sitl_up.sh", "rep_interactive.sh"))
        unknown = [f"{k}→{v}" for k, v in ENV_FOR.items() if v not in blob]
        assert not unknown, (
            f"這些環境變數在啟動腳本裡找不到 ⇒ 建議指令是無效的:{unknown}")


class TestVerifierEndToEnd:
    """把 contract_verify.py 當黑箱跑一次。

    ⚠️ 單元測 evaluate() 過了不代表 CLI 會過:哨兵、正則、退出碼、
       降級路徑都在 CLI 那一側,而那正是實際被批次腳本呼叫的介面。
    """

    LINE = ("[WARN] [1786451084.688] [tot_node]: tot_node 生效值契約:"
            "vmin=95 vmax=150 dive_speed=210 dive_pace=0 gate_r=4700 "
            "perch_alt=3100 r_fan=35000 dive_angle=27 reach=2400 "
            "perch_reach=260 impact_reach=25 half_angle=25 hold_radius=1300 "
            "goto_radius=30 gate_r_min=2500 gate_r_max=6000 n=2")

    NODE_ONLY = ("vmin,vmax,dive_speed,dive_pace,gate_r,perch_alt,r_fan,"
                 "dive_angle,reach,perch_reach,impact_reach,half_angle,READY_AGL")

    def _run(self, tmp_path, line, env=None, extra=()):
        log = tmp_path / "t.log"
        log.write_text(line + "\n", encoding="utf-8")
        e = dict(os.environ, PYTHONIOENCODING="utf-8", READY_AGL="30")
        e.update(env or {})
        return subprocess.run(
            [sys.executable, str(VERIFY), "--from-log", str(log),
             "--require", self.NODE_ONLY, *extra],
            capture_output=True, text=True, env=e, encoding="utf-8")

    def test_a_good_line_exits_zero(self, tmp_path):
        p = self._run(tmp_path, self.LINE)
        assert p.returncode == 0, p.stdout + p.stderr

    def test_a_foam_value_exits_nonzero(self, tmp_path):
        p = self._run(tmp_path, self.LINE.replace("dive_angle=27", "dive_angle=25"))
        assert p.returncode == 1
        assert "泡棉禁區" in p.stdout

    def test_a_log_without_the_contract_line_is_not_silently_passed(self, tmp_path):
        """⚠️ 舊產物沒有契約行。那必須是**明確的 missing**,
        絕不可以因為「解析不到就跳過」而回報通過。"""
        p = self._run(tmp_path, "[WARN] [tot_node]: 沒有契約行的舊 log")
        assert p.returncode == 1
        assert "沒有契約行" in p.stdout

    def test_the_snapshot_records_what_actually_ran(self, tmp_path):
        """規格第 4 部分:每趟要留生效值快照,事後才能回答
        『那一趟到底跑的是什麼』而不是靠記憶或猜。"""
        snap = tmp_path / "snap.json"
        self._run(tmp_path, self.LINE, extra=("--snapshot", str(snap)))
        data = json.loads(snap.read_text(encoding="utf-8"))
        assert data["ok"] is True
        assert float(data["values"]["vmin"]) == 95.0
        assert data["sources"]["vmin"] == "log"
        assert data["sources"]["READY_AGL"] == "env"

    def test_downgrading_is_loud(self, tmp_path):
        """⚠️ STRICT=0 要能繼續,但**必須大聲** ——
        靜默降級正是這一整類 bug 的成因。"""
        p = self._run(tmp_path, self.LINE.replace("dive_angle=27", "dive_angle=25"),
                      env={"STRICT": "0"})
        assert p.returncode == 0
        assert "STRICT=0" in p.stderr and "不可用來下結論" in p.stderr


class TestBatchGateIsWired:
    """⚠️⚠️ 一支沒有人呼叫的驗證器不是閘門,是裝飾品。

    這一整類 bug 的歷史就是這樣長的:`rep_interactive.sh` **從不部署**
    (handoff 缺陷#3)、`stot_fast.env` **從未被 source**、`preflight_check.py`
    列進清單卻沒被複製過。工具存在 ≠ 工具生效。
    """

    REP = (ROOT / "scripts" / "wsl" / "rep_interactive.sh").read_text(encoding="utf-8")
    DEPLOY = (ROOT / "scripts" / "wsl" / "deploy_build.sh").read_text(encoding="utf-8")

    def test_the_batch_actually_calls_the_verifier(self):
        assert "contract_verify.py" in self.REP

    def test_the_gate_runs_before_takeoff(self):
        """契約要在**起飛前**擋下來。飛起來之後才發現組態錯了,
        代價是一整趟航次外加一份會被誤讀的資料。

        ⚠️ 比對的是**實際呼叫形式**而不是裸檔名:本檔的註解本身就提到
           swarm_takeoff.sh(第 138 行),用裸檔名會抓到註解而給出假失敗。
           deploy_build.sh 檔頭記過同一個坑(清單變數被註解搶先匹配)。
        """
        gate = self.REP.index('python3 "$SR/contract_verify.py"')
        takeoff = self.REP.index('bash "$SR/swarm_takeoff.sh"')
        assert gate < takeoff, "契約閘門排在起飛之後 —— 擋不住任何東西"

    def test_the_exit_code_is_not_swallowed_by_the_pipe(self):
        """⚠️⚠️ `cmd | tee` 的 rc 是 **tee** 的,永遠是 0。
        直接 `if ! cmd | tee` 會讓閘門看起來永遠通過 —— 靜默失敗本身。"""
        seg = self.REP[self.REP.index("contract_verify.py"):][:900]
        assert "PIPESTATUS" in seg, "沒有用 PIPESTATUS 取回驗證器的 rc"

    def test_a_snapshot_lands_in_the_artifact_dir(self):
        """規格第 4 部分:生效值快照與 CSV/log/geom 同級。"""
        seg = self.REP[self.REP.index("contract_verify.py"):][:900]
        assert "--snapshot" in seg and "$OUT/" in seg

    @pytest.mark.parametrize("f", ["contract_verify.py", "contract_table.py"])
    def test_the_new_files_are_in_the_deploy_manifest(self, f):
        """⚠️ 漏列 = WSL 端永遠跑舊副本,而 deploy 仍回報「✅ 部署一致」
        (它只驗 ROS2 package 那一層)。preflight_check.py 與 _kpi_parse.py
        都因此踩過,deploy_build.sh 檔頭記著兩次。"""
        assert f in self.DEPLOY

    def test_the_contract_table_is_importable_as_an_installed_module(self):
        """`contract_verify.py` 優先 import 已安裝的 `sar_swarm.contract_table`
        (deploy_build.sh 用 md5 驗過的那一份)。冒煙測試要涵蓋它,
        否則 import 壞掉時會**靜默退回**去讀未驗證的 repo 副本。"""
        assert "sar_swarm.contract_table" in self.DEPLOY

    def test_the_verifier_prefers_the_deployed_package(self):
        """⚠️ 部署後 contract_verify.py 與 contract_table.py **不在同一層**
        (一個進 sitl_run/,一個進 ROS2 package)。只寫相對路徑會在 WSL 端
        ImportError —— 防靜默失敗的工具自己踩靜默失敗。"""
        src = VERIFY.read_text(encoding="utf-8")
        pkg = src.index("from sar_swarm.contract_table")
        # ⚠️ 同上:比對實際的 sys.path 操作,不是註解裡提到的目錄名。
        rel = src.index("sys.path.insert")
        assert pkg < rel, "相對路徑排在已安裝 package 前面,會蓋過部署版"


class TestTableIsWellFormed:
    """表本身的健全性。"""

    def test_every_row_cites_evidence(self):
        """⚠️ 沒有反證來源的契約條目 = 我猜的。
        猜出來的區間會誤報,誤報會讓人把整層關掉。"""
        thin = [c.name for c in TABLE if len(c.evidence) < 20]
        assert not thin, f"這些條目沒有實質反證來源:{thin}"

    def test_bands_are_ordered(self):
        for c in TABLE:
            if c.lo is not None and c.hi is not None:
                assert c.lo < c.hi, f"{c.name} 區間顛倒"

    def test_sources_are_known(self):
        assert {c.source for c in TABLE} <= {SOURCE_NODE, SOURCE_FC, "shell"}

    def test_names_are_unique(self):
        names = [c.name for c in TABLE]
        assert len(names) == len(set(names))

    def test_the_eight_previously_uncovered_parameters_are_now_covered(self):
        """接手時的缺口:這 8 項**完全沒有任何檢查覆蓋**
        (preflight_check.py 走 pymavlink,構不到節點層參數)。"""
        gap = {"vmin", "vmax", "dive_speed", "dive_pace", "gate_r",
               "patrol_alt", "formation_alt", "READY_AGL"}
        assert gap <= set(BY_NAME), f"仍未覆蓋:{sorted(gap - set(BY_NAME))}"
