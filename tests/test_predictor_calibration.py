# -*- coding: utf-8 -*-
"""點質量預測器與 c_D0 校準的迴歸鎖。

═══ 為什麼要鎖 ═══

`scripts/research/predictor/` 是 Hybrid Trajectory Shaping 線上預測器的地基。
它的兩個數字撐著後面所有的可行性判斷:

    c_D0 = 0.01500          ← 決定「早降還能不能達到終端速度」
    預測 RMS 5.4 m/s        ← 決定「線上可達性判斷可不可信」

⚠️ 這些檔原本只存在於 session scratchpad,差一點就跟著 session 一起消失。
   搬進 repo 之後,這條測試確保它們**還能跑、而且還給出同樣的數字**。

⚠️⚠️ 交叉驗證(這一點比單一數字重要):
   `jet_design.py` 的未校正解 0.01642/1.087 = 0.01511,與實測反解差 **0.7%**。
   兩條完全獨立的路徑 —— 一條從設計反推、一條從飛行資料反解。
   若哪天兩者開始分歧,代表機體模型或推力模型有一邊變了。
"""
import pathlib
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PRED = ROOT / "scripts" / "research" / "predictor"


def _run(script, timeout=300):
    """在 predictor 目錄下跑,因為那些檔互相 import(sim.py)。"""
    import os
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    p = subprocess.run([sys.executable, script], cwd=str(PRED), env=env,
                       capture_output=True, text=True, encoding="utf-8",
                       errors="replace", timeout=timeout)
    return p


class TestAssetsArePresent:
    """⚠️ 這一整組資產先前只活在 scratchpad。少一個檔,後面的分析就重建不起來。"""

    CORE = ["sim.py", "extract.py", "calib.py", "validate2.py", "README.md"]

    @pytest.mark.parametrize("name", CORE)
    def test_core_file_exists(self, name):
        assert (PRED / name).is_file(), f"scripts/research/predictor/{name} 不見了"

    def test_no_hardcoded_absolute_paths(self):
        """⚠️ 寫死 `C:/Users/joy46/...` 會讓別台機器或 CI 上直接壞掉,
        而且壞的方式是「找不到資料」而不是「路徑錯」—— 又是一個誤導性錯誤。"""
        bad = [p.name for p in PRED.glob("*.py")
               if "C:/Users/joy46" in p.read_text(encoding="utf-8", errors="replace")]
        assert not bad, f"這些檔仍寫死絕對路徑:{bad}"

    def test_line_endings_are_lf(self):
        """⚠️ 與 tests/test_shell_line_endings.py 同源的坑,這裡再守一次。"""
        bad = [p.name for p in PRED.glob("*.py") if p.read_bytes().count(b"\r")]
        assert not bad, f"這些檔含 CR:{bad}"


@pytest.mark.slow
class TestNumbersStillReproduce:
    """⚠️ 這幾條會實際跑積分,比較慢(標 slow)。

    但它們是唯一能證明「搬家沒搬壞」的東西 —— 檔案存在不等於還算得出同樣的答案。
    """

    @pytest.fixture(scope="class", autouse=True)
    def _traj(self):
        """calib / validate2 都吃 extract.py 產生的 traj.pkl。"""
        p = _run("extract.py")
        assert p.returncode == 0, f"extract.py 失敗:\n{p.stdout[-2000:]}\n{p.stderr[-2000:]}"
        assert (PRED / "traj.pkl").is_file(), "extract.py 沒有產生 traj.pkl"

    def test_drag_coefficient(self):
        """c_D0 = 0.01500(POOLED n=1617、sd 0.00058)。容差 ±1 sd。

        ⚠️ 鎖的是 **POOLED median 那一行**,不是輸出裡任何一個看起來像 0.015 的數字。
           第一版正則抓到了同一份輸出裡的 `jet_design 0.01642`(那是 SITL 的
           c_drag_p 不是有效 c_D0),測試因此假紅 —— 記在這裡當反例。
        """
        p = _run("calib.py")
        assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
        import re
        m = re.search(r"POOLED\s+n=(\d+)\s+median=([\d.]+)\s+mean=[\d.]+\s+sd=([\d.]+)",
                      p.stdout)
        assert m, f"calib.py 沒有 POOLED 匯總行:\n{p.stdout[-1500:]}"
        n, med, sd = int(m.group(1)), float(m.group(2)), float(m.group(3))
        assert n > 1400, f"樣本數掉到 {n}(原本 1617)—— 是不是少了航次資料?"
        assert abs(med - 0.01500) <= 0.0006, f"c_D0 median = {med},偏離 0.01500 超過 1 sd"
        assert sd < 0.0010, f"離散度 sd = {sd} 變大(原本 0.00058)"

    def test_lift_to_drag_at_terminal_speed(self):
        """⚠️ L/D 在 204 m/s 只有 1.25 —— 這是「不是滑翔機是磚頭」的量化依據,
        也是「早降的代價只有 1–7 m/s」那個結論的來源。"""
        p = _run("calib.py")
        import re
        m = re.search(r"^\s*204\s+[\d.]+\s+[\d.]+\s+([\d.]+)", p.stdout, re.M)
        assert m, f"calib.py 沒有 204 m/s 那一列:\n{p.stdout[-1500:]}"
        ld = float(m.group(1))
        assert 1.1 <= ld <= 1.4, f"204 m/s 的 L/D = {ld},偏離實測 1.25"

    def test_integrator_still_tracks_the_measured_dives(self):
        """預測器在 PN 段的重放誤差。RMS 5.4 m/s、終點偏差 +0.2±1.0。

        ⚠️ 這條是「線上可達性判斷可不可信」的唯一依據。若它退步,
           後面所有基於預測的觸發邏輯都要重新評估。
        """
        p = _run("validate2.py")
        assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
        assert "RPK1" in p.stdout and "RPL3" in p.stdout, \
            f"validate2.py 沒有輸出逐架次結果:\n{p.stdout[-1500:]}"


class TestCrossValidationAgainstDesignTool:
    """⭐ 兩條獨立路徑必須持續一致。

    `jet_design.py` 由設計需求反推 c_drag_p;predictor 由飛行資料反解。
    兩者差 0.7% 是**強證據**;若哪天分歧變大,代表其中一邊的假設變了
    (推力模型?機體?SITL build?),必須查清楚而不是取平均。
    """

    def test_design_tool_still_declares_the_same_drag(self):
        west = (ROOT / "scripts" / "wsl" / "run_west.sh").read_text(encoding="utf-8")
        import re
        m = re.search(r"JET_CDP:-([\d.]+)", west)
        assert m, "run_west.sh 找不到 JET_CDP"
        cdp = float(m.group(1))
        # SITL 的 c_drag_p 與有效 c_D0 差一個 1.087 的構型因子(見 predictor/README.md)
        implied = cdp / 1.087
        assert abs(implied - 0.01500) / 0.01500 < 0.05, (
            f"JET_CDP={cdp} ⇒ 隱含 c_D0={implied:.5f},與實測 0.01500 差 "
            f"{abs(implied - 0.015) / 0.015:.1%} —— 超過 5%,兩條路徑開始分歧了,"
            f"要查是推力模型、機體、還是 SITL build 變了")
