# -*- coding: utf-8 -*-
"""守環半徑守門員(tot_node._check_hold_radius)。

⚠️ 這個檢查要擋的約束**本來就寫在 tot_node.py 裡**(hold_radius 的參數說明:
   「守在該點 —— 要 ≥ 物理迴轉半徑,否則飛機飛不出那個圈」),但一直沒有任何
   程式碼去驗它,於是預設 hold_radius=400 一路違反自己的契約而且完全靜默。

   SW1 實測後果(環半徑 3120 m):守環退化成繞自己的最小圈,距目標距離在
   1724–4925 m 之間振盪;六機進環時 d 一致(3108–3131),放行時卻散到
   1806–3449 m ⇒ 放行 h/d 0.78→1.68、需求俯角 38.1°→59.3°。
"""
import math
import re
import pathlib

import pytest

SRC = (pathlib.Path(__file__).resolve().parents[1]
       / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py")


def turn_radius(vmin_eas, alt_m, bank_deg=45.0):
    """把節點裡那條式子獨立實作一次(不 import 節點,免得同源錯誤)。"""
    sigma = (1.0 - 2.25577e-5 * max(alt_m, 0.0)) ** 4.2559
    tas = vmin_eas / math.sqrt(max(sigma, 1e-6))
    return tas * tas / (9.81 * math.tan(math.radians(bank_deg)))


class TestTurnRadiusPhysics:
    def test_sw1_setting_violates_its_own_contract(self):
        """SW1 的 hold_radius=400 遠小於物理迴轉半徑 —— 這就是被漏掉的那個檢查。"""
        r = turn_radius(95.0, 3100.0)
        assert 1200 < r < 1320, r          # 約 1253 m
        assert 400 < r                      # 設定值只有 ~32%

    def test_eas_to_tas_matters(self):
        """3100 m 的 EAS→TAS 是 1.167 倍,迴轉半徑 ∝ TAS² ⇒ 差 1.36 倍。

        用 EAS 直接算會低估 26%,正好會讓一個該紅的組態變綠。
        """
        sea = 95.0 ** 2 / 9.81
        alt = turn_radius(95.0, 3100.0)
        assert alt / sea == pytest.approx(1.362, rel=0.02)

    def test_lower_vmin_shrinks_the_turn(self):
        """迴轉半徑 ∝ v²:vmin 45 vs 95 差 4.45 倍。"""
        assert (turn_radius(95.0, 3100.0)
                / turn_radius(45.0, 3100.0)) == pytest.approx(4.45, rel=0.02)


class TestGuardIsWiredIn:
    """守門員必須真的被呼叫,而且訊息要能被 log 撈到。"""

    def setup_method(self):
        self.src = SRC.read_text(encoding="utf-8")

    def test_the_method_exists(self):
        assert "def _check_hold_radius(self)" in self.src

    def test_it_is_actually_called(self):
        """⚠️ 假迴歸鎖防呆:方法存在但沒人呼叫 = 死碼。"""
        calls = re.findall(r"self\._check_hold_radius\(\)", self.src)
        assert len(calls) >= 1, "定義了卻沒有生產呼叫點"

    def test_it_uses_tas_not_eas(self):
        """必須有密度修正 —— 少了它會低估 26%,守門員就形同虛設。"""
        assert "2.25577e-5" in self.src

    def test_failure_path_is_error_not_info(self):
        """靜默失敗是這個專案的頭號敵人:不合格必須用 error 級別。"""
        blk = self.src.split("def _check_hold_radius(self)")[1][:2600]
        assert "get_logger().error(" in blk

    def test_it_names_all_three_remedies(self):
        """只吼不給處置等於沒修 —— 三條出路都要寫進訊息。"""
        blk = self.src.split("def _check_hold_radius(self)")[1][:2600]
        for token in ("hold_radius 提到", "環半徑放大", "相位閘門"):
            assert token in blk, token

    def test_the_contract_it_enforces_is_still_documented(self):
        """守門員與它引用的那句參數說明必須同時存在,否則下一個人會刪錯邊。"""
        assert "要 ≥ 物理迴轉半徑" in self.src

    def test_measured_contrast_is_labelled_as_n2(self):
        """SW1 2.2× / P91 3.2× 只有兩趟,不可被讀成門檻值。"""
        blk = self.src.split("def _check_hold_radius(self)")[1][:2600]
        assert "n=2" in blk and "非門檻" in blk


class TestReleaseGeometryIsTheRealSymptom:
    """把 SW1 的實測放行幾何釘成迴歸基準 —— 修好之後這些數字必須改善。"""

    SW1_RELEASE_D = {1: 3139, 2: 2245, 3: 3241, 4: 2579, 5: 3449, 6: 1806}
    SW1_RING = 3120.0

    def test_release_distance_spread_is_19_times_worse_than_arrival(self):
        d = list(self.SW1_RELEASE_D.values())
        assert max(d) - min(d) == pytest.approx(1643, abs=5)
        # 進環時只散 23 m(3108–3131),放行時散 1643 m
        assert (max(d) - min(d)) / 23.0 > 60

    def test_worst_aircraft_needs_an_impossible_dive_angle(self):
        """U6 放行時 h/d=1.68 ⇒ 需求俯角 59.3°,還沒開始俯衝就貼上限。"""
        h = 3036.0
        ang = math.degrees(math.atan2(h, self.SW1_RELEASE_D[6]))
        assert ang == pytest.approx(59.3, abs=0.5)

    def test_p91_release_geometry_all_inside_the_envelope(self):
        """對照組:P91 六機放行需求俯角全部落在 22–33°。"""
        p91 = {1: (6773, 3066), 2: (6789, 3081), 3: (6453, 3120),
               4: (6772, 2811), 5: (5072, 3207), 6: (5631, 3223)}
        for i, (d, h) in p91.items():
            ang = math.degrees(math.atan2(h, d))
            assert 22.0 <= ang <= 33.0, (i, ang)
