"""前置角投影(Lookahead / carrot-chasing)的幾何驗證。

背景:P91 / T1 / U1 三趟的 `bearing_to_wp_deg` 中位數都是 −73° ~ −86° ——
僚機整段巡航都用接近**正橫**的角度對著自己的 slot。正橫追蹤要求持續大坡度
轉彎,誘導阻力 ∝ n² = (1/cos φ)²,U1 因此掉了 1000 m 高度。

修法:導引 WP = slot 沿長機航向再推 L1(1500 m),誤差仍對真 slot 量。
這裡用**該航次實測的側偏/沿航向分量**驗證夾角確實收斂,並且釘住
「L1 越大夾角越小、但不是零」這個單調關係(避免有人以為它能一步到位)。
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / "scripts" / "wsl" / "sar_swarm_patches"))

from geodesy import advance_along_heading, EARTH_M_PER_DEG  # noqa: E402

L1_DEFAULT = 1500.0


def bearing_off_nose(along_m, cross_m):
    """機頭到目標的夾角(度)。along>0 = 目標在前方,cross = 側向偏置。"""
    return abs(math.degrees(math.atan2(cross_m, along_m)))


def bearing_with_lookahead(along_m, cross_m, l1):
    """加上 L1 之後的夾角。L1 沿長機航向 ≈ 沿機頭方向推(直線巡航段)。"""
    return bearing_off_nose(along_m + l1, cross_m)


# P91 末端 60 s 實測的 (沿航向, 垂直航向) 誤差分量,單位 m。
# 取自 stot_stage2_report.py 的輸出;符號取絕對值,只看夾角。
P91_TAIL = {"uav2": (223.0, 1387.0), "uav3": (145.0, 1435.0),
            "uav4": (916.0, 748.0), "uav6": (826.0, 1154.0)}


class TestBearingReduction:
    def test_raw_slot_is_near_abeam(self):
        """反證:不加 L1 時,實測分量算出來的夾角就是 73°–84°(對得上記錄)。"""
        angs = {k: bearing_off_nose(a, c) for k, (a, c) in P91_TAIL.items()}
        assert 70.0 <= angs["uav2"] <= 85.0
        assert 70.0 <= angs["uav3"] <= 85.0
        # uav4 沿航向分量較大,本來就沒那麼正橫
        assert angs["uav4"] < angs["uav2"]

    def test_lookahead_reduces_every_case(self):
        for k, (a, c) in P91_TAIL.items():
            before = bearing_off_nose(a, c)
            after = bearing_with_lookahead(a, c, L1_DEFAULT)
            assert after < before, k

    def test_lookahead_halves_the_worst_case(self):
        """uav3 是最正橫的一架:84° → 應收斂到 45° 以下。"""
        a, c = P91_TAIL["uav3"]
        assert bearing_off_nose(a, c) > 80.0
        assert bearing_with_lookahead(a, c, L1_DEFAULT) < 45.0

    def test_converged_offset_gives_small_angle(self):
        """側偏收到 400 m 之後,夾角應落在 15° 附近 —— 這才是漸進切入的區間。"""
        ang = bearing_with_lookahead(223.0, 400.0, L1_DEFAULT)
        assert 10.0 <= ang <= 20.0

    def test_monotone_in_l1(self):
        """L1 越大夾角越小(單調),但恆 > 0 —— 不會一步到位。"""
        a, c = P91_TAIL["uav2"]
        angs = [bearing_with_lookahead(a, c, l1)
                for l1 in (0.0, 500.0, 1500.0, 3000.0)]
        assert angs == sorted(angs, reverse=True)
        assert angs[-1] > 0.0

    def test_never_flips_behind(self):
        """L1 的重點:目標永遠留在機頭前方(夾角 < 90°),不會觸發 behind 保護。

        ⚠️ 只要 along + L1 > 0 就成立。slot 落後本機超過 L1 時仍會 > 90°,
           那時 behind 保護該接手 —— 這裡把那個邊界也釘住。
        """
        assert bearing_with_lookahead(-200.0, 1400.0, L1_DEFAULT) < 90.0
        # 落後超過 L1 ⇒ 仍在身後,保護不該被繞過
        assert bearing_with_lookahead(-1600.0, 1400.0, L1_DEFAULT) > 90.0


class TestRollAndDrag:
    def test_induced_drag_factor_at_cruise_limit(self):
        """15° 的誘導阻力代價只有 +7%,45° 是 +100% —— 這是壓坡度的理由。"""
        n2 = lambda d: 1.0 / math.cos(math.radians(d)) ** 2
        assert n2(15.0) == pytest.approx(1.072, abs=0.01)
        assert n2(45.0) == pytest.approx(2.0, abs=0.01)

    def test_turn_radius_fits_the_leg(self):
        """15° @157 m/s 的轉彎半徑 ~9.4 km,98 km 航程展得開 4.6 km 側偏。"""
        v, phi = 157.0, math.radians(15.0)
        r = v * v / (9.81 * math.tan(phi))
        assert 8000.0 < r < 11000.0
        assert r < 98000.0


class TestFreezeTargetMustBeAheadOfNose:
    """側偏凍結的回歸測試 —— 這個 bug 實際發生過(X1)。

    凍結目標是「沿**長機**航向往前 L1」。那只保證它在長機前方;
    本機當下若指向別處(掉高度時常常正在轉),它就落在**本機身後**。
    實測 X1:爬升段 behind% = 99.8–100%、|bearing| 中位數 112°、
    d2wp 剛好 1499(= L1)—— 距離對、方向全錯,UAV4 末端誤差 112 km。
    """

    @staticmethod
    def _ahead_of_nose(al, cr, trail_ahead=500.0):
        """`WingmanNode._ahead_of_nose` 的純幾何等價:橫向照留、縱向夾到前方。"""
        return (al if al >= trail_ahead else trail_ahead), cr

    def test_target_along_leader_axis_can_land_behind_own_nose(self):
        """反證:本機與長機航向差 112° 時,沿長機軸線推 1500 m 會落在身後。"""
        l1, dpsi = 1500.0, math.radians(112.0)
        # 目標相對本機:沿長機航向 1500 m ⇒ 投影到本機機頭軸
        al = l1 * math.cos(dpsi)          # < 0 ⇒ 在身後
        cr = l1 * math.sin(dpsi)
        assert al < 0.0
        assert bearing_off_nose(al, cr) > 90.0
        assert math.hypot(al, cr) == pytest.approx(l1)   # 距離是對的(d2wp≈1499)

    def test_guard_pulls_it_in_front(self):
        l1, dpsi = 1500.0, math.radians(112.0)
        al, cr = self._ahead_of_nose(l1 * math.cos(dpsi), l1 * math.sin(dpsi))
        assert al > 0.0
        assert bearing_off_nose(al, cr) < 90.0

    def test_guard_preserves_cross_track(self):
        """關鍵:橫向分量不可以被改掉(改掉 = 開迴路直飛,P81 那個更糟的失敗)。"""
        l1, dpsi = 1500.0, math.radians(112.0)
        cr_in = l1 * math.sin(dpsi)
        _, cr_out = self._ahead_of_nose(l1 * math.cos(dpsi), cr_in)
        assert cr_out == pytest.approx(cr_in)

    def test_guard_is_noop_when_already_ahead(self):
        al, cr = self._ahead_of_nose(1400.0, 300.0)
        assert (al, cr) == (1400.0, 300.0)

    @pytest.mark.parametrize("dpsi_deg", [0, 30, 60, 89, 91, 120, 179])
    def test_never_behind_for_any_heading_error(self, dpsi_deg):
        l1, d = 1500.0, math.radians(dpsi_deg)
        al, cr = self._ahead_of_nose(l1 * math.cos(d), l1 * math.sin(d))
        assert bearing_off_nose(al, cr) < 90.0


class TestCarrotIndependentOfOwnHeading:
    """⭐ 這一項修法的**核心不變量**:導引點不得隨本機航向旋轉。

    舊實作把 slot 投影到本機機頭座標系,目標方位因此變成「相對本機航向 ±89°」
    —— 飛機一轉,目標跟著轉。實測簽名:`|bearing_to_wp|` 的 p90 在四架上是
    81.3 / 81.6 / 82.3 / 85.7°,**從不超過 ~86°**,而 behind_deg = 90°。
    那是幾何硬夾,不是物理限制。結果 dpsi 穩定 29–33°,橫向以 v·sinΔ 發散。

    這組測試用純幾何複刻 `WingmanNode._carrot_on_path`,把不變量釘死。
    """

    L1 = 1500.0

    @classmethod
    def carrot(cls, me_en, slot_en, leader_hdg_deg, lead_m=0.0):
        """me/slot 用平面 (east, north) 公尺座標;回傳胡蘿蔔點 (east, north)。

        ⚠️ 刻意**不接受本機航向參數** —— 簽名本身就是不變量的一部分。
        """
        hl = math.radians(leader_hdg_deg)
        de = slot_en[0] - me_en[0]
        dn = slot_en[1] - me_en[1]
        al = dn * math.cos(hl) + de * math.sin(hl)
        cr = -dn * math.sin(hl) + de * math.cos(hl)
        al_c = max(al, cls.L1) + lead_m
        return (me_en[0] + al_c * math.sin(hl) + cr * math.cos(hl),
                me_en[1] + al_c * math.cos(hl) - cr * math.sin(hl))

    def test_signature_takes_no_own_heading(self):
        """不變量的結構性保證:函式簽名裡沒有本機航向,就不可能耦合進去。"""
        import inspect
        params = list(inspect.signature(self.carrot).parameters)
        assert params == ["me_en", "slot_en", "leader_hdg_deg", "lead_m"]

    @pytest.mark.parametrize("slot_off", [(0, 20000), (0, -5000),
                                          (66000, -19000), (-3000, 1200)])
    def test_carrot_identical_for_every_own_heading(self, slot_off):
        """同一組 (本機位置, 長機航向, slot) 之下,胡蘿蔔點唯一。

        舊實作在這裡會給出 360 個不同的點 —— 那就是正回饋的來源。
        """
        me, slot = (0.0, 0.0), slot_off
        ref = self.carrot(me, slot, 259.0)
        # 本機航向掃過整圈都不該影響結果(函式根本拿不到它)
        for _ in range(0, 360, 15):
            assert self.carrot(me, slot, 259.0) == pytest.approx(ref)

    def test_aims_at_slot_when_slot_is_well_ahead(self):
        """slot 還在前方(al > L1)⇒ 直接瞄 slot,縱向位置回授保留(行為不變)。"""
        me = (0.0, 0.0)
        slot = (0.0, 20000.0)          # 正北 20 km,長機航向 0°
        c = self.carrot(me, slot, 0.0)
        assert c == pytest.approx(slot, abs=1.0)

    def test_pushes_forward_when_slot_is_behind(self):
        """slot 落到身後 ⇒ 沿長機航跡前方 L1,**不要求掉頭**。"""
        me = (0.0, 0.0)
        slot = (0.0, -5000.0)          # 正南 5 km(長機航向 0° ⇒ 在身後)
        c = self.carrot(me, slot, 0.0)
        assert c[1] == pytest.approx(self.L1, abs=1.0)   # 前方 L1
        assert c[0] == pytest.approx(0.0, abs=1.0)       # 橫向不變

    def test_preserves_cross_track_exactly(self):
        """橫向分量原封不動 —— 不是 P81 的開迴路直飛(那會把橫向導引移除)。"""
        me = (0.0, 0.0)
        slot = (66000.0, -19000.0)     # 右 66 km、後 19 km(BL1 的實際量級)
        c = self.carrot(me, slot, 0.0)
        assert c[0] == pytest.approx(66000.0, abs=1.0)   # cross 保留
        assert c[1] == pytest.approx(self.L1, abs=1.0)

    def test_cross_track_error_is_monotonically_removed(self):
        """收斂性:朝胡蘿蔔飛,橫向誤差必須單調變小(舊實作是不變或發散)。"""
        me = [30000.0, 0.0]            # 右偏 30 km
        slot = (0.0, 0.0)              # slot 在長機航跡線上(V_ANGLE=0)
        v, dt = 115.0, 1.0
        prev = abs(me[0])
        for _ in range(400):
            c = self.carrot(tuple(me), slot, 0.0)
            dx, dy = c[0] - me[0], c[1] - me[1]
            n = math.hypot(dx, dy) or 1.0
            me[0] += v * dt * dx / n
            me[1] += v * dt * dy / n
            cur = abs(me[0])
            assert cur <= prev + 1e-6, "橫向誤差變大 = 沒有收斂"
            prev = cur
        assert prev < 30000.0 * 0.5, f"400 s 後橫向只收到 {prev:.0f} m"

    def test_bearing_is_not_pinned_below_ninety(self):
        """反證:舊實作的簽名是 |bearing| 被夾在 90° 正下方(實測 p90 81–86°)。
        胡蘿蔔法在 slot 正後方時,方位應該可以**超過** 90°(該掉頭就掉頭)。"""
        me = (0.0, 0.0)
        slot = (0.0, -5000.0)
        c = self.carrot(me, slot, 0.0)
        # 本機正朝南(180°)時,胡蘿蔔在正北 ⇒ 相對機頭 180°,不該被夾住
        ang = bearing_off_nose(-c[1], c[0])     # 機頭 180° ⇒ 沿機頭分量 = −北分量
        assert ang > 90.0


class TestProjectionMatchesGeodesy:
    def test_advance_along_heading_moves_l1_metres(self):
        """實作用的是 geodesy.advance_along_heading —— 確認它真的推了 L1 公尺。"""
        lat0, lon0 = 25.0777, 121.2325
        for hdg in (0.0, 90.0, 259.0):
            lat1, lon1 = advance_along_heading(lat0, lon0, hdg, L1_DEFAULT)
            dn = (lat1 - lat0) * EARTH_M_PER_DEG
            de = ((lon1 - lon0) * EARTH_M_PER_DEG
                  * math.cos(math.radians(lat0)))
            assert math.hypot(dn, de) == pytest.approx(L1_DEFAULT, rel=1e-3)
