# -*- coding: utf-8 -*-
"""不變式:俯衝配速 ≥ vmin,否則同步迴路等於不存在。

俯衝段的速度指令(tot_node.py):

    t_rem = dive_T - now
    v_req = d / max(t_rem, 0.05)
    v = max(vmin, min(dive_speed, v_req))

而 `dive_T` 是用 `_pace_of()`(≈ dive_pace 的幾何正規化)訂的。
⇒ **pace < vmin 時,v_req 恆低於 vmin,整段被 max() 夾住**:

    ① 近機無法慢下來等遠機 —— 同步權限歸零
    ② dive_speed 永遠不生效 —— 終端速度被壓在 vmin

⚠️ 這是**靜默**失效:飛機照飛、命中照樣算,只有時刻表對不上。

實測(RPJ 三趟,2026-08-11):`DIVE_PACE` 預設 50.0 是**泡棉機時代**的常數
(當年 vmin=45,50 ≈ vmin,符合「配保守一點」的設計意圖)。
噴射組態 vmin=95 之後 50 ≪ 95 ⇒ 早到 25–52 s、散布 25.00 s、
終端只有 115–120 m/s(指令 dive_speed=210 從未生效)。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
W = ROOT / "scripts" / "wsl"
UP = (W / "swarm_ros2_up.sh").read_text(encoding="utf-8")
TOT = (W / "sar_swarm_patches" / "tot_node.py").read_text(encoding="utf-8")


def _sh_default(src, var):
    m = re.search(r'%s="\$\(_f "\$\{%s:-([^}]*)\}"\)"' % (var, var), src)
    if m is None:
        m = re.search(r'%s="\$\{%s:-([^}]*)\}"' % (var, var), src)
    assert m, "找不到 %s 的預設值" % var
    return m.group(1)


def _code(src):
    return "\n".join(ln.split("#", 1)[0] for ln in src.splitlines())


class TestPaceIsNotTheFoamConstant:
    def test_dive_pace_default_is_auto(self):
        """0 = 自動,取 min(vmax, 0.8·dive_speed);50.0 是泡棉常數。"""
        v = float(_sh_default(UP, "DIVE_PACE"))
        assert v == 0.0, (
            f"DIVE_PACE 預設是 {v} —— 應該是 0(自動)。"
            "50.0 是 vmin=45 時代的值,噴射組態(vmin=95)下會讓整段俯衝夾在下限。"
        )

    def test_auto_pace_lands_between_vmin_and_dive_speed(self):
        """自動值必須落在 [vmin, dive_speed] 之間,兩個方向才都有調節權限。"""
        vmin = float(_sh_default(UP, "VMIN"))
        vmax = float(_sh_default(UP, "VMAX"))
        dive_speed = float(_sh_default(UP, "DIVE_SPEED"))
        auto = min(vmax, 0.8 * dive_speed)
        assert vmin <= auto <= dive_speed, (
            f"自動配速 {auto} 不在 [{vmin}, {dive_speed}] 內 —— "
            "低於 vmin 就沒有同步,高於 dive_speed 則永遠追不上"
        )
        assert auto > vmin, (
            f"自動配速 {auto} 等於 vmin {vmin} —— 近機沒有可以慢下來的空間"
        )


class TestNodeGuardsTheInvariant:
    def test_tot_node_warns_when_pace_below_vmin(self):
        """靜默失效必須變成大聲失效。"""
        code = _code(TOT)
        assert re.search(r"self\.dive_pace\s*<\s*self\.vmin", code), (
            "tot_node 沒有檢查 pace < vmin —— 這是靜默失效,"
            "飛機照飛、命中照算,只有時刻表對不上"
        )

    def test_the_guard_is_an_error_not_a_debug_line(self):
        i = TOT.index("self.dive_pace < self.vmin")
        window = TOT[i:i + 600]
        assert "get_logger().error" in window, \
            "pace < vmin 應該用 error 等級 —— info 會被淹沒"

    def test_speed_command_still_clamps_to_vmin(self):
        """守衛是為了讓夾制不再發生,不是把夾制拿掉 —— vmin 仍是飛行下限。"""
        code = _code(TOT)
        assert re.search(r"max\(\s*self\.vmin\s*,\s*min\(\s*self\.dive_speed",
                         code), "俯衝速度指令的夾制式被改動了,請重新檢視本不變式"
