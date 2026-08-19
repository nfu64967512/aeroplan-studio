"""直連 pymavlink 鏈路必須被排空 —— 只寫不讀會讓整台 MAVProxy 凍結。

背景(RPG1,2026-08-10 實測):

    MAVProxy 是**單執行緒**。`--out=tcpin:` 的任何一個客戶端不讀,
    核心收方緩衝就會塞滿 → TCP 視窗關閉 → MAVProxy 卡在那個 write 上
    ⇒ **該機所有輸出一起凍結**。

    UAV1 vs UAV2 的位元組計數器(同一時刻、同一批次):

        埠      角色        UAV1        UAV2
        58x0    MAVROS       9,279   2,218,582
        68x0    tot_node    16,032   2,202,329
        69x0    編隊 WP      1,983   1,937,646

    而 SITL→MAVProxy 的 master 埠兩機都送了 2.4 MB —— 資料有進來,出不去。

    後果:tot_node 眼中 UAV1 的位姿**永遠停在 t=0**,`d` 逐位元固定在 34148,
    抵達判定永遠不成立,整趟 1500 s 逾時。

    ⚠️ 症狀極具欺騙性:飛機真的在飛,`ros2 topic echo .../state` 也回得出
      `mode: GUIDED` —— 那是**鎖存的舊值**。只有位元組計數器看得出來。

    這與記憶 master-node-stall-diagnosis 記載的是同一個根因;當時定位了,
    但從來沒有在 mavros_interface.py 修掉。

⚠️ 排空器必須掛在**自己的 timer**,不能掛在節點的 tick:
   tick 在讓權(strike_authority)時會整段 return,而讓權期間正是打擊進行中,
   最不能凍結的時候。這支測試把這個性質也鎖住。
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

SRC = (Path(__file__).resolve().parents[1]
       / "scripts" / "wsl" / "sar_swarm_patches" / "mavros_interface.py")

RECV_CALLS = {"recv_match", "recv_msg"}


@pytest.fixture(scope="module")
def tree() -> ast.Module:
    return ast.parse(SRC.read_text(encoding="utf-8"), filename=str(SRC))


def _funcs(tree: ast.Module) -> dict[str, ast.FunctionDef]:
    return {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}


def _called_attrs(node: ast.AST) -> set[str]:
    out = set()
    for sub in ast.walk(node):
        if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute):
            out.add(sub.func.attr)
    return out


class TestLinkIsDrained:
    def test_a_drain_method_exists(self, tree):
        assert "_drain_wp" in _funcs(tree), (
            "mavros_interface.py 少了 _drain_wp():直連鏈路只寫不讀,"
            "MAVProxy 會被塞住,該機所有下游一起斷流。"
        )

    def test_drain_actually_reads_the_link(self, tree):
        called = _called_attrs(_funcs(tree)["_drain_wp"])
        assert called & RECV_CALLS, (
            f"_drain_wp() 沒有呼叫 {sorted(RECV_CALLS)} 中的任何一個 —— "
            "它沒有在排空,只是看起來像。"
        )

    def test_module_is_not_write_only(self, tree):
        """整支檔案至少要有一處讀 —— 擋住「排空器被刪掉」的回歸。"""
        assert _called_attrs(tree) & RECV_CALLS, (
            "整支 mavros_interface.py 找不到任何 recv —— 又變回只寫不讀了。"
        )

    def test_drain_is_bounded(self, tree):
        """不能無上限地讀,否則排空器自己會佔住 executor。"""
        fn = _funcs(tree)["_drain_wp"]
        has_loop = any(isinstance(n, (ast.For, ast.While)) for n in ast.walk(fn))
        assert has_loop, "_drain_wp() 應該用有界迴圈一次讀多筆"
        for n in ast.walk(fn):
            if isinstance(n, ast.While):
                pytest.fail("_drain_wp() 用了 while —— 要有明確上限,不可能無界")


class TestDrainSurvivesYield:
    """讓權時 tick 會整段 return;排空**不可以**跟著停。"""

    def test_drain_is_on_its_own_timer(self, tree):
        init = _funcs(tree)["__init__"]
        timers = [
            n for n in ast.walk(init)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == "create_timer"
        ]
        assert timers, "__init__ 沒有為排空建立 timer"
        targets = set()
        for call in timers:
            for a in call.args:
                if isinstance(a, ast.Attribute):
                    targets.add(a.attr)
                elif isinstance(a, ast.Name):
                    targets.add(a.id)
        assert "_drain_wp" in targets, (
            f"create_timer 的回呼是 {sorted(targets)},沒有 _drain_wp。"
            "掛在節點 tick 上會在讓權時一起停掉 —— 那正是最不能停的時候。"
        )

    def test_drain_is_not_called_from_node_tick_only(self, tree):
        """排空不可以**只**由 send_guided_waypoint 之類的下令路徑觸發。"""
        for name in ("send_guided_waypoint", "change_speed_async"):
            fn = _funcs(tree).get(name)
            if fn is None:
                continue
            assert "_drain_wp" not in _called_attrs(fn), (
                f"{name}() 裡呼叫了 _drain_wp —— 停止下令時排空也會停,"
                "正好在讓權期間失效。"
            )
