"""DO_REPOSITION 的 CHANGE_MODE 旗標:**只有打擊權威層可以帶,編隊層不可以。**

ArduPlane 的閘門(`ArduPlane/GCS_MAVLink_Plane.cpp:584-605`,已對原始碼確認):

    if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
        (plane.control_mode == &plane.mode_guided)) {
        plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND);
        ...
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;          // param2=0 且不在 GUIDED ⇒ 整發丟掉

⚠️ 兩個方向都會出事,而且**兩種都實測撞過**:

┌─ 帶太少(param2=0)——────────────────────────────────────────────────
│ RPF1:互動路徑用 MANUAL=1 + swarm_takeoff.sh 起飛,飛控停在 TAKEOFF;
│ backup_node 讓權後沒人維持 GUIDED ⇒ tot_node 的位置指令整趟被丟且**零告警**
│ (ACK 走沒人排空的 WP 連線)。而 DO_CHANGE_SPEED 不吃這個閘門照樣生效
│ ⇒ 地速 148-164 m/s 看起來很健康,飛機卻在桃園上空繞圈 1250 s。
└──────────────────────────────────────────────────────────────────────

┌─ 帶太多(編隊層也帶)——──────────────────────────────────────────────
│ RPG/RPH/RPI 三批報廢。nodes.log 時序:
│     463.776  UAV1 master: GUIDED takeoff, then patrol the box
│     465.777  master airborne in GUIDED — leading the patrol   ← 還在地上、還沒 arm
│     465.777  master -> leg 0 (…) @ 80 m
│ master_node 啟動 2 秒就自稱 airborne 並以 1 Hz 送巡邏航點;帶了旗標之後
│ 這些航點**每秒把長機從 TAKEOFF 拽回 GUIDED** ⇒ 起飛被取消
│ ⇒ UAV1 gs_max=0.0、alt_max=0.1,整趟停在跑道上(樣本 6290 筆,遙測沒斷)。
└──────────────────────────────────────────────────────────────────────

⇒ 正解是**分層**:
     tot_node._reposition()              帶旗標 —— 它是打擊權威,建立 GUIDED 是它的職責
     mavros_interface.send_guided_waypoint()  不帶 —— 模式歸飛控,直到
                                          `_keep_guided()`(有 guided_min_alt 守衛)接手
   而 RPF 那個「讓權後沒人切模式」的洞,補在 backup/wingman:
   讓 `_keep_guided()` 在讓權期間**繼續跑**(見 test_keep_guided_survives_yield)。

本檔讀的是 **AST 的實際呼叫引數**,不是註解 —— 本專案已有三次
「測試讀到註解而誤判通過」的前科。
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

PATCHES = Path(__file__).resolve().parents[1] / "scripts" / "wsl" / "sar_swarm_patches"

# command_int_send(target_system, target_component, frame, command, current,
#                  autocontinue, param1, param2, param3, param4, x, y, z)
PARAM2_INDEX = 7
FLAG_NAME = "MAV_DO_REPOSITION_FLAGS_CHANGE_MODE"

STRIKE_AUTHORITY = "tot_node.py"        # 可以帶旗標
FORMATION_LAYER = "mavros_interface.py"  # 不可以帶旗標


def _attr_tail(node: ast.AST) -> str:
    if isinstance(node, ast.Attribute):
        return node.attr
    if isinstance(node, ast.Name):
        return node.id
    return ""


def _reposition_calls():
    out = []
    for path in sorted(PATCHES.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if _attr_tail(node.func) != "command_int_send":
                continue
            if any(_attr_tail(a) == "MAV_CMD_DO_REPOSITION" for a in node.args):
                out.append((path.name, node))
    return out


def _param2(call: ast.Call) -> ast.AST:
    assert len(call.args) >= PARAM2_INDEX + 1, (
        f"command_int_send 引數不足({len(call.args)}),無法定位 param2"
    )
    return call.args[PARAM2_INDEX]


class TestCallSitesAreKnown:
    def test_exactly_the_two_expected_files(self):
        """先確認測試找得到東西 —— 否則下面全部空跑照樣綠燈。"""
        names = {n for n, _ in _reposition_calls()}
        assert names == {STRIKE_AUTHORITY, FORMATION_LAYER}, (
            f"DO_REPOSITION 送出點變了:{sorted(names)}。"
            "新增送出點必須先想清楚它屬於哪一層,再更新這份清單。"
        )


class TestStrikeAuthorityCarriesTheFlag:
    """tot_node 帶旗標:讓權當下飛機可能不在 GUIDED,它得自己把模式建起來。"""

    def test_param2_sets_change_mode(self):
        calls = [c for n, c in _reposition_calls() if n == STRIKE_AUTHORITY]
        assert calls, f"{STRIKE_AUTHORITY} 找不到 DO_REPOSITION 呼叫"
        for call in calls:
            assert FLAG_NAME in ast.dump(_param2(call)), (
                f"{STRIKE_AUTHORITY}:{call.lineno} 的 param2 沒帶 {FLAG_NAME}。"
                "非 GUIDED 時整發被丟回 MAV_RESULT_FAILED,而且零告警。"
            )

    def test_param2_is_not_a_bare_constant(self):
        for call in [c for n, c in _reposition_calls() if n == STRIKE_AUTHORITY]:
            p2 = _param2(call)
            if isinstance(p2, ast.Constant):
                pytest.fail(
                    f"{STRIKE_AUTHORITY}:{call.lineno} 的 param2 是常數 {p2.value!r};"
                    f"必須用 mavutil.mavlink.{FLAG_NAME}"
                )


class TestFormationLayerMustNotCarryTheFlag:
    """⭐ 編隊層**不可以**帶 —— 帶了會在起飛滾行時把飛機拽出 TAKEOFF。"""

    def test_param2_is_zero(self):
        calls = [c for n, c in _reposition_calls() if n == FORMATION_LAYER]
        assert calls, f"{FORMATION_LAYER} 找不到 DO_REPOSITION 呼叫"
        for call in calls:
            p2 = _param2(call)
            assert FLAG_NAME not in ast.dump(p2), (
                f"{FORMATION_LAYER}:{call.lineno} 帶了 {FLAG_NAME}。"
                "RPG/RPH/RPI 三批就是這樣報廢的:master_node 啟動 2 秒就開始送"
                "巡邏航點,帶旗標等於每秒取消一次起飛(UAV1 gs_max=0.0)。"
            )
            assert isinstance(p2, ast.Constant) and float(p2.value) == 0.0, (
                f"{FORMATION_LAYER}:{call.lineno} 的 param2 應該是 0.0,"
                f"實際是 {ast.dump(p2)}"
            )


class TestKeepGuidedSurvivesYield:
    """RPF 那個洞的正解:讓權讓的是**位置權**,模式維持必須繼續。

    `_keep_guided()` 只在 `_formation_control()` 的最後一行被呼叫,而讓權的
    `return` 排在它前面 ⇒ 一讓權模式維持就整個停掉。修法是在每個讓權
    `return` 之前補一次 `_keep_guided()`。
    ⚠️ n-agnostic 是硬要求:wingman(n>2 才用到)也要修,不能只修 n=2 的路徑。
    """

    @pytest.mark.parametrize("fname", ["backup_node.py", "wingman_node.py"])
    def test_every_yield_return_in_a_control_path_keeps_guided(self, fname):
        src = (PATCHES / fname).read_text(encoding="utf-8")
        tree = ast.parse(src, filename=fname)

        # 找出所有 `if <...>.yielded(): ...` 的 if 節點
        yields = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.If):
                continue
            test = node.test
            if isinstance(test, ast.Call) and _attr_tail(test.func) == "yielded":
                yields.append(node)
        assert yields, f"{fname} 找不到任何讓權判斷"

        # 控制路徑 = 該 if 的 body 裡有 _keep_guided;看門狗路徑不需要。
        # 至少要有一個讓權點維持模式,否則就是回歸到 RPF 的狀態。
        keeps = [n for n in yields
                 if any(_attr_tail(c.func) == "_keep_guided"
                        for c in ast.walk(n) if isinstance(c, ast.Call))]
        assert keeps, (
            f"{fname} 的讓權路徑沒有任何一處呼叫 _keep_guided —— "
            "讓權期間沒人維持 GUIDED,tot_node 的位置指令會被飛控整發丟掉且零告警"
        )


class TestFlagValueMatchesFirmwareGate:
    def test_flag_is_bit_zero(self):
        mavutil = pytest.importorskip(
            "pymavlink.mavutil",
            reason="pymavlink 只在 WSL 端裝;Windows 端跳過這條",
        )
        assert getattr(mavutil.mavlink, FLAG_NAME) == 1
