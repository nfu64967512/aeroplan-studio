# -*- coding: utf-8 -*-
"""動態 PN 交棒距離。

═══ 為什麼不綁環半徑的固定比例 ═══
全檔案庫掃描(169 份 trial log / 828 架次,scratchpad/scan_gate.py):
    Spearman(殘餘高度, 交棒視線俯角 γ=atan(h/d)) = **+0.892**(8 個箱全單調)
    Spearman(殘餘高度, 比值 f=gate/環半徑)       = **+0.103**(幾乎無關)
且 f≈0.39 的 66 架次殘餘從 3 m 到 1879 m(626 倍)——γ 解釋得了、f 解釋不了。
先前 n=17 的「≥0.566 成功 / ≤0.446 失敗」判別帶,在 828 架次上被反證:
f 落在 0.393–0.49 的 366 架次殘餘中位數 10.0 m、81% 在門檻內。

═══ 公式(幾何,不是文獻結論)═══
    R_turn = V² / (g·(cos γ_now − n_z))
    d_min  = R_turn·sinθ + (h − R_turn(1−cosθ))/tanθ    (h > 圓弧能卸的高度)
           = √(h·(2R_turn − h))                          (圓弧就夠)
    gate   = clamp(d_min/Φ, 下限, 上限, 環半徑)

⭐ 式子裡**沒有設計俯角** —— 環的俯角只決定環半徑,不決定交棒距。
   這是「先訂角度、再由它決定 PN 段距離」的正確形式:比例式做不到,
   因為同一個 f=0.781 在 27° 是 γ=33°,在 45° 就變成 γ=52°。

═══ 證據等級 ═══
[幾何]  d_min 的兩支式子與單調性。
[文獻]  「交棒門檻 = 動態幾何量的倍率」「先訂角度、釘一個座標、讓距離浮動」的**結構**:
        NASA TM-104744 §3.9(TAEM `RCIR < 1.1·RTURN`)、
        USA005512 p.4-7(all glide slopes share one pullup altitude)。倍率原文自承 empirical。
[文獻缺口] 「由待卸高度反推交棒距離」文獻上不存在;架構最像的 Yu 2020
        (CJA 33(3):990–1005)把它列為 future work(Remark 4、Conclusions)。
[相關]  θ_max=35 與 Φ=0.80 是用我們自己三趟六架次校準的,無受控實驗。
"""
import math
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE = (ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py").read_text(encoding="utf-8")
TRIAL_P = ROOT / "scripts" / "wsl" / "stot_trial.sh"
TRIAL = TRIAL_P.read_text(encoding="utf-8")
WEST = (ROOT / "scripts" / "wsl" / "run_west.sh").read_text(encoding="utf-8")

G0 = 9.80665


def gate_of(h, V, *, theta_max=35.0, phi=0.80, nz=-1.0, gam_now=0.0,
            ring=6147.0, gate_r=4800.0, lo=2500.0, hi=6000.0):
    """把節點那條式子獨立實作一次(不 import 節點,免得同源錯誤)。"""
    if theta_max <= 0.0 or h <= 0.0:
        return gate_r
    den = math.cos(gam_now) - nz
    if V <= 1.0 or den <= 1e-6:
        return gate_r
    th = math.radians(theta_max)
    r_turn = V * V / (G0 * den)
    h_arc = r_turn * (1.0 - math.cos(th))
    if h > h_arc:
        d_min = r_turn * math.sin(th) + (h - h_arc) / math.tan(th)
    else:
        d_min = math.sqrt(max(h * (2.0 * r_turn - h), 0.0))
    return max(lo, min(d_min / phi, hi, ring))


class TestFallback:
    def test_theta_zero_returns_absolute_gate(self):
        """θmax<=0 ⇒ 完全退回絕對 gate_r,逐位元舊行為。這是回退開關。"""
        assert gate_of(3100.0, 150.0, theta_max=0.0) == 4800.0
        assert gate_of(3100.0, 150.0, theta_max=-1.0) == 4800.0

    def test_degenerate_inputs_fall_back(self):
        """高度 ≤0、速度過小、分母非正 ⇒ 退回,不得回傳 NaN 或負值。"""
        assert gate_of(0.0, 150.0) == 4800.0
        assert gate_of(-5.0, 150.0) == 4800.0
        assert gate_of(3100.0, 0.5) == 4800.0
        assert gate_of(3100.0, 150.0, nz=1.0, gam_now=0.0) == 4800.0


class TestIndependentOfDesignDiveAngle:
    """⭐ 本次改動的核心:交棒距不隨設計俯角變。"""

    def test_same_gate_for_27_and_45_degree_rings(self):
        """27° 的環是 6147 m、45° 的環是 3132 m,但交棒距應該一樣
        (只要沒被環半徑夾住)。固定比例做不到這件事。"""
        g27 = gate_of(3100.0, 150.0, ring=6147.0)
        g45 = gate_of(3100.0, 150.0, ring=9999.0)   # 不被環夾住的對照
        assert g27 == pytest.approx(g45, rel=1e-9)

    def test_fixed_fraction_would_not_be_invariant(self):
        """反證:固定比例 f 在不同俯角下給出不同的視線俯角需求。"""
        f = 0.781
        g27 = math.degrees(math.atan(math.tan(math.radians(27.0)) / f))
        g45 = math.degrees(math.atan(math.tan(math.radians(45.0)) / f))
        assert g27 == pytest.approx(33.1, abs=0.3)
        assert g45 == pytest.approx(52.0, abs=0.5)
        assert abs(g45 - g27) > 15.0


class TestMonotonicity:
    def test_higher_arrival_hands_over_earlier(self):
        """抵環時偏高的機要**更早**交棒(需要更多航程卸高度)。

        KIL3 的 UAV1 實測 h=3228 m,在現行固定 gate 之下可行性餘裕只剩 0.4%。
        """
        assert gate_of(3228.0, 102.0) > gate_of(2073.0, 160.0)

    def test_steeper_theta_allows_later_handover(self):
        """允許 PN 飛更陡 ⇒ 需要的航程變少 ⇒ 交棒可以更晚(更近)。"""
        assert gate_of(3100.0, 150.0, theta_max=45.0) < \
               gate_of(3100.0, 150.0, theta_max=35.0)

    def test_smaller_phi_hands_over_earlier(self):
        """Φ 越小餘裕越大 ⇒ 交棒越早。"""
        assert gate_of(3100.0, 150.0, phi=0.70) > gate_of(3100.0, 150.0, phi=0.90)


class TestClamps:
    def test_never_exceeds_ring(self):
        """交棒不得發生在抵環之前,否則放行與交接的先後會反轉。"""
        assert gate_of(3100.0, 200.0, ring=3000.0) <= 3000.0

    def test_respects_bounds(self):
        assert gate_of(50.0, 60.0, lo=2500.0) >= 2500.0
        assert gate_of(9000.0, 260.0, hi=6000.0, ring=99999.0) <= 6000.0


class TestNodeWiring:
    def test_params_declared_and_read(self):
        for n in ("gate_theta_max", "gate_phi", "gate_r_min",
                  "gate_r_max", "gate_hold_n"):
            assert 'g("%s"' % n in NODE, n
            assert 'p("%s").value' % n in NODE, n

    def test_default_is_off(self):
        """預設必須是關 —— 程式改動本身零行為變化,單變數紀律。"""
        assert re.search(r'g\("gate_theta_max",\s*0\.0\)', NODE)

    def test_uses_total_speed_not_ground_speed(self):
        """⚠️ CSV 的 gs 只有水平分量。用它會低估 R_turn ⇒ 交棒偏近 ⇒ 偏不安全。
        vz 早就存在 self.vel 裡,只是從沒被用過。"""
        m = re.search(r"def gate_of\(self, i\).*?(?=\n    def )", NODE, re.S)
        assert m, "找不到 gate_of"
        assert "math.hypot(gs, vz)" in m.group(0)

    def test_hold_n_debounce(self):
        """動態 gate 會隨 h/V 抖動,單拍觸發等於把雜訊接成指令。"""
        assert "_gate_hits" in NODE
        assert "self.gate_hold_n" in NODE

    def test_handover_log_records_the_discriminant(self):
        """γ 是 828 架次裡唯一單調的量,以前完全沒進 log(只能事後從 d 反推)。"""
        assert "視線俯角" in NODE


class TestTrialPlumbing:
    def test_f_helper_defined_before_every_use(self):
        """⚠️⚠️ bash 函式必須先定義才能呼叫,而 `bash -n` **檢查不出來**。

        實際踩過:GATE_* 區塊放在 _f() 定義之前 ⇒ `_f: command not found`
        ⇒ 參數變空字串 ⇒ `-p gate_theta_max:=` 傳給 ROS2。
        該檔自己的註解寫著「這個陷阱已經踩過兩次」——這是第三次,所以鎖起來。
        """
        lines = TRIAL_P.read_text(encoding="utf-8").splitlines()
        dfn = next((i for i, l in enumerate(lines, 1) if l.startswith("_f()")), None)
        assert dfn, "找不到 _f() 定義"
        uses = [i for i, l in enumerate(lines, 1) if "$(_f " in l]
        early = [u for u in uses if u < dfn]
        assert not early, "這些行在 _f() 定義前就呼叫它:%s" % early

    def test_params_forwarded(self):
        for n in ("gate_theta_max", "gate_phi", "gate_r_min",
                  "gate_r_max", "gate_hold_n"):
            assert "-p %s:=" % n in TRIAL, n

    def test_float_params_go_through_f(self):
        """ROS2 會把 "35" 判成 INTEGER 而參數宣告是 DOUBLE ⇒ 節點啟動即崩。"""
        for n in ("GATE_THETA_MAX", "GATE_PHI", "GATE_R_MIN", "GATE_R_MAX"):
            assert re.search(r'%s="\$\(_f ' % n, TRIAL), n

    def test_west_exposes_the_switch(self):
        assert 'export GATE_THETA_MAX="${GATE_THETA_MAX:-0.0}"' in WEST
        assert 'export GATE_PHI="${GATE_PHI:-0.80}"' in WEST
