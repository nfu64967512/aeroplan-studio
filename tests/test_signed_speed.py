"""geodesy.speed_for_along_error 單元測試 —— 縱向速度律的正負號。

背景(S31 實測):舊的 `speed_for_error(err, ...)` 吃**無號距離**,
僚機超前 slot 10–28 km 時仍命令 `catch_speed` ⇒ 加速、超前更多。
四架僚機的 along 全部為正 +10383 ~ +28014 m,佔總誤差 98%,單調發散。

⚠️ 這個 bug 先前踩不到:機體推力不足時僚機一律落後(只用到加速那一側),
   而且實驗 A 之前速度指令零權限(TECS 走 no-airspeed 路徑),兩側都沒有執行器。
   **修好速度通道等於同時解鎖了這個 bug。**
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / "scripts" / "wsl" / "sar_swarm_patches"))

from geodesy import speed_for_along_error, speed_for_error  # noqa: E402

NOMINAL, CATCH, LO, CATCH_R = 135.0, 157.0, 90.0, 528.0


def cmd(along):
    return speed_for_along_error(along, CATCH_R, NOMINAL, CATCH, LO)


class TestSign:
    def test_behind_accelerates(self):
        """slot 在前方(我落後)⇒ 加速。"""
        assert cmd(+CATCH_R) == pytest.approx(CATCH)
        assert NOMINAL < cmd(+264.0) < CATCH

    def test_ahead_decelerates(self):
        """我超前 ⇒ **減速**。這是舊實作完全沒有的那一側。"""
        assert cmd(-CATCH_R) == pytest.approx(LO)
        assert LO < cmd(-264.0) < NOMINAL

    def test_on_station_is_nominal(self):
        assert cmd(0.0) == pytest.approx(NOMINAL)

    def test_saturates_both_sides(self):
        assert cmd(+28014.0) == pytest.approx(CATCH)
        assert cmd(-28014.0) == pytest.approx(LO)

    def test_monotone_in_along(self):
        xs = [-20000, -1000, -264, 0, 264, 1000, 20000]
        vs = [cmd(x) for x in xs]
        assert vs == sorted(vs), "指令必須隨 along 單調遞增"

    def test_none_is_nominal(self):
        assert cmd(None) == pytest.approx(NOMINAL)


class TestRegressionVsOldUnsigned:
    """用 S31 的實際數字釘住「舊實作在超前時會加速」這個 bug。"""

    @pytest.mark.parametrize("along", [10383.0, 13206.0, 23526.0, 28014.0])
    def test_old_unsigned_wrongly_accelerates_when_ahead(self, along):
        """反證:舊實作拿 |along| 當 err ⇒ 一律回 catch_speed。"""
        old = speed_for_error(abs(along), 200.0, CATCH_R, NOMINAL, CATCH)
        assert old == pytest.approx(CATCH), "舊實作在超前時仍下 catch"

    @pytest.mark.parametrize("along", [10383.0, 13206.0, 23526.0, 28014.0])
    def test_new_decelerates_when_ahead(self, along):
        """正證:同樣的量,新實作下減速到下限。"""
        assert cmd(-along) == pytest.approx(LO)


class TestClosureGuarantee:
    """⚠️ 診斷報告預警的陷阱:減速下限若等於長機速度,誤差會**凍結**而非收斂。"""

    def test_decel_floor_must_be_below_leader(self):
        """長機被 max-consensus 壓在 90 等人時,僚機超前也降到 90 ⇒ 同速 ⇒ 不收斂。"""
        leader = 90.0
        bad = speed_for_along_error(-5000.0, CATCH_R, leader, CATCH, leader)
        assert bad == pytest.approx(leader), "下限=長機速度時確實凍結(反證)"
        good = speed_for_along_error(-5000.0, CATCH_R, leader, CATCH,
                                     max(25.0, leader * 0.67))
        assert good < leader, "下限要低於長機速度才有閉合率"

    def test_closure_rate_is_positive_when_ahead(self):
        """實務組態:長機 90、僚機超前 ⇒ 指令必須明顯低於 90。"""
        leader = 90.0
        v = speed_for_along_error(-5000.0, CATCH_R, leader, CATCH,
                                  max(25.0, leader * 0.67))
        assert leader - v >= 25.0, f"閉合率只有 {leader - v:.0f} EAS,太弱"

    def test_deceleration_authority_exceeds_acceleration(self):
        """減速權限(−45)本來就是加速(+21)的 2 倍 —— 這一側先前完全沒用到。"""
        up = cmd(+CATCH_R) - NOMINAL
        down = NOMINAL - cmd(-CATCH_R)
        assert down > up
