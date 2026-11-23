# -*- coding: utf-8 -*-
"""僚機在爬升脫離前不可以下編隊指令 —— 否則噴射機會被拒收到沉回地面。

═══ RPM1(n=3 首航,2026-08-11)═══

n=2 的六趟(RPK/RPL)**從未執行過 wingman_node** —— 那時只有
「長機 + 備長」。n=3 第一次啟用僚機路徑,當場翻車:

    [wingman_node] 長機廣播中斷 5.2 s → 啟動 Dead Reckoning (speed 12 m/s)
    ⇒ 噴射機 AIRSPEED_MIN = 25,每一發 DO_CHANGE_SPEED 被 MAV_RESULT=4 整發拒收
    ⇒ UAV3 沉回地面(rel_alt −3.96)
    ⇒ tot_node 等不到全員 ⇒ 整趟 1500 s 逾時

⇒ 起飛爬升交給飛控自己的 TAKEOFF 模式(TKOFF_ALT=300),
  本層在脫離高度之前只維持 GUIDED、不下任何指令。

⚠️ 這個閘門**失敗是安全的**(繼續放手,飛控照樣爬),
   與 rep_interactive.sh 的就緒閘門不同(那個沒過整趟逾時)。
   「不可達門檻」已經害過 RPB1/RPC1,兩者不可混為一談。
"""
import ast
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
P = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "wingman_node.py"
SRC = P.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(P))


def _fn(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError("找不到 %s()" % name)


def _calls(node):
    return {c.func.attr for c in ast.walk(node)
            if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}


class TestClimboutHold:
    def test_parameter_declared(self):
        assert re.search(r'declare_parameter\(\s*"climbout_hold_alt"\s*,\s*([0-9.]+)',
                         SRC), "climbout_hold_alt 沒有宣告"

    def test_threshold_is_reachable_and_above_formation_alt(self):
        """門檻要夾在 formation_alt(50) 與 TKOFF_ALT(300) 之間。

        低於 formation_alt ⇒ 編隊層一接手就把飛機帶回門檻以下(來回跳)。
        高於 TKOFF_ALT     ⇒ 永遠脫離不了(雖然失敗是安全的,但等於本層停擺)。
        """
        m = re.search(r'declare_parameter\(\s*"climbout_hold_alt"\s*,\s*([0-9.]+)', SRC)
        v = float(m.group(1))
        assert 60.0 <= v <= 280.0, f"climbout_hold_alt={v} 不在 (50, 300) 的安全帶內"

    def test_fly_holds_before_climbout(self):
        fly = _fn("_fly")
        src = ast.dump(fly)
        assert "_climbout_hold_alt" in src, "_fly() 沒有檢查脫離高度"
        assert "_climbout_done" in src, "_fly() 沒有用鎖存旗標"

    def test_hold_keeps_guided_not_just_returns(self):
        """放手 ≠ 放生:模式維持必須繼續,否則 tot_node 的指令會被飛控丟掉。

        ⚠️ 用 AST 定位,不要用文字切片 —— 本專案已經被「固定長度切片
           讓待驗的東西掉出視窗而假性失敗」咬過兩次。
        """
        fly = _fn("_fly")
        holds = [n for n in ast.walk(fly)
                 if isinstance(n, ast.If) and "_climbout_done" in ast.dump(n.test)]
        assert holds, "_fly() 裡找不到以 _climbout_done 為條件的放手區塊"
        assert any("_keep_guided" in _calls(h) for h in holds), (
            "爬升脫離的放手路徑沒有呼叫 _keep_guided —— "
            "放手期間沒人維持 GUIDED,tot_node 的 DO_REPOSITION 會被整發丟掉且零告警"
        )

    def test_latched_not_re_entrant(self):
        """⚠️ 必須鎖存。編隊層會把飛機帶回 formation_alt(50 < 門檻 120),
        沒鎖存就會在「放手/接手」之間來回跳 —— 就是 stot-altitude-gate-chatter
        記載的那種沒有遲滯的閘門。"""
        assert re.search(r"self\._climbout_done\s*=\s*True", SRC), \
            "脫離後沒有把旗標鎖成 True"
        assert re.search(r"if not self\._climbout_done", SRC), \
            "沒有用鎖存旗標短路,代表每 tick 都會重判"


class TestStrikeYieldStillKeepsGuided:
    """n-agnostic:讓權時模式維持不可以跟著停(與 backup_node 同一個修正)。"""

    def test_yield_path_keeps_guided(self):
        fly = _fn("_fly")
        for node in ast.walk(fly):
            if not isinstance(node, ast.If):
                continue
            t = node.test
            if isinstance(t, ast.Call) and getattr(t.func, "attr", "") == "yielded":
                assert "_keep_guided" in _calls(node), (
                    "讓權路徑沒有 _keep_guided —— RPF 那個「繞桃園 1250 s」的形態"
                )
                return
        pytest.fail("_fly() 裡找不到讓權判斷")
