"""逐機轉彎半徑（B 階段）—— 值層級鎖。

⚠️⚠️⚠️ **改之前**整支機隊收斂成一個半徑,實測混編 [Pioneer 160, HERO 100]:
``IDP_Solver`` 與 ``DCCPPPathBuilder`` 都收到 **160.0**,HERO 較緊的機動能力
在規劃階段就被丟掉。

⚠️⚠️⚠️ **這一檔最重要的一件事**:`DCCPPPathBuilder` 的半徑
**改不到航點座標**。ENTRY / TRANSFER 都走 ``_dubins_waypoints(simplify=True)``,
只留端點、把弧丟掉,所以換半徑只改 ``entry_length_m`` / ``transfer_length_m``,
航點列逐位元相同(R=40/100/160 實測 md5 一樣)。

⇒ **任何寫在航點座標上的驗收,對 builder 半徑結構上不可能紅。**
   真正會動到飛出來的航線的,是 IDP 排出的**條帶順序與方向**。
   本檔因此把斷言放在:收據、長度、排序,以及一條「航點測不到」的負控制。

範圍(使用者定調):只做**路徑生成**。誰拿哪些條帶(`_partition_*`)仍用機隊
半徑,`allocation` 必須維持不變 —— 本檔有一條鎖住它。
"""

import logging
import math

import pytest

from core.base.airframe import HERO_PNP, PIONEER_PNP
from core.dccpp.dccpp_path_builder import DCCPPPathBuilder
from core.dccpp.idp_solver import IDP_Solver
import mission.swarm_coordinator as SC

LAT, LON = 25.05, 121.20
R_PIONEER = PIONEER_PNP.turn_radius_m      # 160.0
R_HERO = HERO_PNP.turn_radius_m            # 100.0


def _area():
    return [{"area_id": 1,
             "polygon": [(LAT, LON), (LAT, LON + 0.030),
                         (LAT + 0.020, LON + 0.030), (LAT + 0.020, LON)],
             "priority": 1.0, "scan_angle_deg": 0.0}]


def _drone(did, radius, speed, lon_off=0.0):
    return {"drone_id": did, "position": (LAT - 0.005, LON + lon_off),
            "vehicle_type": "vtol", "speed": speed, "turn_radius": radius}


def _plan(drones, areas=None):
    return SC.SwarmCoordinator().plan_coverage_dccpp(
        areas=areas or _area(), drones=drones, coverage_width_m=300.0,
        coverage_altitude=120.0, overlap_rate=0.1, auto_scan_angle=True,
        vtol_transition_alt_m=100.0)


def _polyline_m(built):
    """航點折線實長 —— 這才是會飛出來的東西。

    ⚠️ 不要用 ``BuiltPath.total_length_m``:那個量含 Dubins 弧長,而航點只
       留端點,兩者在實測中差 −2,426 到 +1,004 m,不是同一件事。
    """
    R = 6371000.0
    tot = 0.0
    pts = [(w.lat, w.lon) for w in built.waypoints]
    for (a1, o1), (a2, o2) in zip(pts, pts[1:]):
        p1, p2 = math.radians(a1), math.radians(a2)
        dp, dl = p2 - p1, math.radians(o2 - o1)
        h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        tot += 2 * R * math.asin(min(1.0, math.sqrt(h)))
    return tot


def _wp_key(built):
    return tuple((round(w.lat, 7), round(w.lon, 7)) for w in built.waypoints)


MIXED = [_drone(1, R_PIONEER, PIONEER_PNP.cruise_airspeed_mps, 0.000),
         _drone(2, R_HERO, HERO_PNP.cruise_airspeed_mps, 0.002)]


# ═══════════════════════════════════════════════════════════════════
#  0. 可觸發性 —— 沒有這一節，下面的斷言可能只是在量一組本來就相等的值
# ═══════════════════════════════════════════════════════════════════
def test_the_fixture_really_holds_two_distinct_radii():
    """兩台若半徑相同，逐機與機隊收斂在數值上無從區分。"""
    assert R_PIONEER != R_HERO
    assert {d["turn_radius"] for d in MIXED} == {R_PIONEER, R_HERO}
    # 而且機隊收斂值(max)恰好等於其中一台 ⇒ 只有另一台看得出差別
    assert max(R_PIONEER, R_HERO) == R_PIONEER


def test_the_collapsed_radius_really_would_change_the_answer():
    """證明「用 160 排 HERO」與「用 100 排 HERO」給出不同的航線。

    ⚠️ 這是整檔的地基。若這兩者恰好相同，所有下游斷言都是恆綠的。
    直接單飛 HERO 兩次:一次宣告 100(它自己)、一次宣告 160(機隊收斂值)。
    """
    own = _plan([_drone(1, R_HERO, HERO_PNP.cruise_airspeed_mps)])
    collapsed = _plan([_drone(1, R_PIONEER, HERO_PNP.cruise_airspeed_mps)])
    a = own["assembled_paths"][1]
    b = collapsed["assembled_paths"][1]
    assert _wp_key(a) != _wp_key(b), (
        "兩個半徑給出相同航點 —— 這組幾何分不出差別，換一組再驗收")
    # 方向:自己的較小半徑應該飛得比較短
    assert _polyline_m(a) < _polyline_m(b)


# ═══════════════════════════════════════════════════════════════════
#  1. 負控制 —— 明確記錄「航點座標測不到 builder 半徑」
# ═══════════════════════════════════════════════════════════════════
def test_builder_radius_alone_cannot_move_a_single_waypoint():
    """⚠️ 這條**故意**斷言「沒有變化」。

    它不是在驗收功能，是在鎖住一個會讓人寫出恆綠測試的事實:
    ``simplify=True`` 讓 ENTRY/TRANSFER 只留端點，所以單獨換 builder 半徑
    只改長度、不動座標。哪天 simplify 被拿掉，這條會紅 —— 那時要回來
    把上面那些「斷言放在長度」的註解一起改掉。
    """
    from core.dccpp.area_processor import AreaProcessor
    from core.geometry.coordinate import CoordinateTransformer

    ar = AreaProcessor().process_latlon(_area()[0]["polygon"], 300.0, 30.0,
                                        scan_angle_deg=None)
    tf = CoordinateTransformer(*ar.centroid_latlon)
    lines = ar.scan_lines[:6]
    dirs = [1] * len(lines)

    outs = {}
    for R in (40.0, 100.0, 160.0):
        b = DCCPPPathBuilder(min_turn_radius=R, default_altitude=120.0, step_size=10.0)
        built = b.build(start_lat=LAT - 0.005, start_lon=LON,
                        start_heading_compass_deg=90.0,
                        ordered_scan_lines=lines, directions=dirs,
                        transformer=tf, uav_id=1)
        outs[R] = built

    keys = {R: _wp_key(v) for R, v in outs.items()}
    assert keys[40.0] == keys[100.0] == keys[160.0], (
        "航點隨半徑變了 —— simplify=True 的行為改了，本檔的驗收策略要重寫")
    # 但長度必須有變，否則半徑根本沒送到
    lens = {R: v.transfer_length_m for R, v in outs.items()}
    assert lens[40.0] < lens[100.0] < lens[160.0], (
        f"transfer 長度沒有隨半徑單調變化: {lens}")


# ═══════════════════════════════════════════════════════════════════
#  2. 收據 —— 每架實際用到的半徑
# ═══════════════════════════════════════════════════════════════════
def test_each_aircraft_gets_its_own_generation_radius():
    res = _plan(MIXED)
    got = {uid: bp.turn_radius_m for uid, bp in res["assembled_paths"].items()}
    assert got == {1: pytest.approx(R_PIONEER), 2: pytest.approx(R_HERO)}, (
        f"收據 {got}，應為 {{1: {R_PIONEER}, 2: {R_HERO}}}")


def test_the_receipt_ignores_a_radius_the_builder_ignores():
    """宣告一個 builder 會忽略的半徑(<=0)，收據必須回報建構子的機隊值。

    ⚠️ **訂正**:這條原名 `..._is_not_a_copy_of_the_input`，宣稱能證明
    「收據來自生成器而不是輸入」——**證明不了**。當 `turn_radius=0.0` 被忽略時，
    `float(gen._radius)` 與 `self.R` 本來就相等，兩種寫法給同一個數。
    真正擋住「照抄輸入」的是 §6 的 `test_the_two_layer_consistency_check_actually_fires`
    (強迫兩層不一致)。這條只鎖「被忽略的輸入不會漏進收據」。
    """
    from core.dccpp.area_processor import AreaProcessor
    from core.geometry.coordinate import CoordinateTransformer
    ar = AreaProcessor().process_latlon(_area()[0]["polygon"], 300.0, 30.0,
                                        scan_angle_deg=None)
    tf = CoordinateTransformer(*ar.centroid_latlon)
    b = DCCPPPathBuilder(min_turn_radius=123.0, default_altitude=120.0)
    built = b.build(start_lat=LAT, start_lon=LON, start_heading_compass_deg=0.0,
                    ordered_scan_lines=ar.scan_lines[:3], directions=[1, 1, 1],
                    transformer=tf, uav_id=7, turn_radius=0.0)
    assert built.turn_radius_m == pytest.approx(123.0), (
        "收據照抄了輸入的 0.0 —— 那樣就分不出「有傳」與「有用到」")


def test_the_ordering_radius_is_recorded_too():
    """排序層(IDP)也要留收據，否則只證明了幾何層。"""
    from core.dccpp.area_processor import AreaProcessor
    from core.geometry.coordinate import CoordinateTransformer
    ar = AreaProcessor().process_latlon(_area()[0]["polygon"], 300.0, 30.0,
                                        scan_angle_deg=None)
    tf = CoordinateTransformer(*ar.centroid_latlon)
    solver = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    out = solver.solve(
        [{"uav_id": 1, "lat": LAT - 0.005, "lon": LON, "heading": 90.0,
          "turn_radius": R_PIONEER},
         {"uav_id": 2, "lat": LAT - 0.005, "lon": LON + 0.002, "heading": 90.0,
          "turn_radius": R_HERO}],
        ar, tf)
    got = {uid: p.turn_radius_m for uid, p in out.per_uav.items()
           if p.ordered_scan_lines}
    assert got.get(2) == pytest.approx(R_HERO), (
        f"IDP 沒有用 HERO 自己的半徑排序: {got}")


# ═══════════════════════════════════════════════════════════════════
#  3. 綁住兩層的不變式
# ═══════════════════════════════════════════════════════════════════
def test_ordering_radius_equals_generation_radius():
    """排條帶順序的半徑，必須等於生成幾何的半徑。

    ⚠️ 兩者不一致**不會有任何錯誤訊息**，而實測不一致比兩邊都用機隊值還糟
    (34,576 vs 34,462 m)。這條是唯一擋得住「只修一層」的鎖。

    ⚠️ 第一版這條是**恆綠**的:它去 `res["results"]` 找 `per_uav`，
    而那裡裝的是 `MDTSPResult`（沒有這個屬性），迴圈一次都沒跑。
    現在改讀 `turn_radius_by_uav` 收據，並先斷言它非空。
    """
    res = _plan(MIXED)
    receipt = res.get("turn_radius_by_uav")
    assert receipt, "收據是空的 —— 這條測試量不到任何東西"
    assert set(receipt) == {1, 2}, f"收據沒有涵蓋兩台: {sorted(receipt)}"
    for uid, rec in receipt.items():
        assert rec["ordering_m"] == pytest.approx(rec["generation_m"]), (
            f"UAV {uid}: 排序用 {rec['ordering_m']}、生成用 {rec['generation_m']}")
    # 而且兩台的值必須真的不同，否則「相等」是因為只有一個值
    assert receipt[1]["generation_m"] != pytest.approx(receipt[2]["generation_m"])


def test_distinct_generators_match_distinct_radii():
    """兩個相異半徑都必須各自建出生成器。

    ⚠️ **訂正**:原本的標題與註解說這條能擋「每次都重建」——**做不到**。
    下面用的是 `set(made)`，集合丟掉次數,只看得到「出現過哪些半徑」。
    而且正確的程式碼本來就會建 5 個生成器(builder 與 cost cache 各有一份、
    起降段還有),不是 2 個,所以「數量相等」這個等式在正確碼上就是假的。
    「重建太多次」是純效能問題,不動任何航點、長度或收據,故不在此鎖。
    這條能擋的是:提到迴圈外、memo key 撞格 —— 兩者都會讓某個半徑**不出現**。
    """
    import core.trajectory.dubins_trajectory as DT
    made = []
    orig = DT.DubinsTrajectoryGenerator.__init__

    def spy(self, *a, **k):
        r = orig(self, *a, **k)
        made.append(round(float(self._radius), 6))
        return r
    DT.DubinsTrajectoryGenerator.__init__ = spy
    try:
        _plan(MIXED)
    finally:
        DT.DubinsTrajectoryGenerator.__init__ = orig
    distinct = set(made)
    assert {R_PIONEER, R_HERO} <= distinct, (
        f"沒有為兩個半徑各建生成器，實際建了 {sorted(distinct)}")


def test_cost_caches_are_separate_objects_per_radius():
    """⚠️ `_DubinsCostCache.cost` 的 memo key 只有幾何、沒有半徑。

    所以逐機半徑**一定**要用不同的 instance;共用一份 `_cache` 再加 key
    的話，A 機的成本會被靜默餵給 B 機。
    """
    s = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    a = s.cost_cache_for(R_PIONEER)
    b = s.cost_cache_for(R_HERO)
    assert a is not b
    assert a.R == pytest.approx(R_PIONEER)
    assert b.R == pytest.approx(R_HERO)
    assert a._cache is not b._cache
    # 同一個半徑要拿到同一個物件（否則等於沒有快取）
    assert s.cost_cache_for(R_HERO) is b


# ⚠️⚠️⚠️ 這裡原本還有第二個 `test_solve_single_refuses_to_default_its_cost`，
#    與 §6 那個**同名**。Python 後定義覆蓋前定義 ⇒ 這一份是**死碼**，
#    pytest 收 22 個而檔案裡有 23 個 `def test_*`，而且**完全沒有警告**。
#    更糟的是這一份用空的 scan_lines，函式在碰到 cost 之前就回傳，
#    `pytest.raises(TypeError)` 是被別的原因滿足的 —— 就算它有跑也是恆綠。
#    已刪。守衛見 `test_no_duplicate_test_names_in_this_file`。


def test_no_duplicate_test_names_in_this_file():
    """同名測試會被靜默覆蓋，pytest 不會說一個字。

    ⚠️ 這正是本 repo 記過的 [[promised-guards-that-do-not-exist]]:
    「註解說有鎖」不等於那條鎖會執行。判別法是比對**定義數**與**收集數**。
    """
    import ast
    import pathlib
    src = pathlib.Path(__file__).read_text(encoding="utf-8")
    names = [n.name for n in ast.parse(src).body
             if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    dup = sorted({n for n in names if names.count(n) > 1})
    assert not dup, f"同名測試會被覆蓋成死碼: {dup}"


# ═══════════════════════════════════════════════════════════════════
#  4. 範圍守衛 —— 分配不可以被動到
# ═══════════════════════════════════════════════════════════════════
def test_allocation_is_unchanged_by_per_aircraft_radius():
    """B 階段只做路徑生成。誰拿哪一區必須跟收斂成一個半徑時一樣。

    ⚠️ `UAVTaskAllocator` 本身就會讀 `uav.turn_radius`，所以這條不是多餘的。
    """
    hetero = _plan(MIXED)
    homo = _plan([_drone(1, R_PIONEER, PIONEER_PNP.cruise_airspeed_mps, 0.000),
                  _drone(2, R_PIONEER, HERO_PNP.cruise_airspeed_mps, 0.002)])
    assert hetero["allocation"] == homo["allocation"], (
        f"分配被動到了: {hetero['allocation']} vs {homo['allocation']}")


def test_a_homogeneous_fleet_uses_the_fleet_radius():
    """同質機隊下，每一架的收據都必須等於那個共同半徑。

    ⚠️ **訂正**:這條原名 `..._is_bit_identical_to_before`，並宣稱
    「保護既有的 golden」——它做不到,因為它一個航點、一個長度都沒看。
    既有 golden 由 tests/test_dccpp_pipeline.py 保護;這條只鎖收據。
    下面補一條真的比航點的。
    """
    a = _plan([_drone(1, R_PIONEER, 21.5, 0.000),
               _drone(2, R_PIONEER, 21.5, 0.002)])
    for uid, bp in a["assembled_paths"].items():
        assert bp.turn_radius_m == pytest.approx(R_PIONEER)


def test_a_homogeneous_fleet_plans_the_same_route_as_the_collapsed_radius():
    """同質機隊:逐機路徑必須與「整隊收斂成一個半徑」**逐航點相同**。

    這才是「不影響既有同質 fixture」的真正驗收 —— 比航點，不是比收據。
    做法:同一組機隊，一次讓每台宣告自己的半徑(值相同)，
    一次把兩台都設成機隊最大值。兩者的航點列必須一致。
    """
    own = _plan([_drone(1, R_PIONEER, 21.5, 0.000),
                 _drone(2, R_PIONEER, 21.5, 0.002)])
    for uid, bp in own["assembled_paths"].items():
        assert bp.waypoints, f"UAV {uid} 沒有航點，這條測試量不到東西"
    # 異質時 uav1 的半徑本來就是機隊最大值 ⇒ 它的航線不該被這次改動動到
    mixed = _plan(MIXED)
    assert _wp_key(mixed["assembled_paths"][1]) == _wp_key(
        _plan([_drone(1, R_PIONEER, PIONEER_PNP.cruise_airspeed_mps, 0.000),
               _drone(2, R_PIONEER, HERO_PNP.cruise_airspeed_mps, 0.002)]
              )["assembled_paths"][1]), (
        "半徑等於機隊最大值的那一架，航線被改動到了")


# ═══════════════════════════════════════════════════════════════════
#  5. 退化輸入
# ═══════════════════════════════════════════════════════════════════
def test_a_radius_below_the_floor_names_itself():
    """⚠️ 半徑 < 約 6 m 時，反解出的虛構空速掉到失速以下，
    `FixedWingConstraints` 的訊息**不會提到半徑**，看起來像不相干的空速錯誤。

    實測:低於門檻時整條 v2 管線靜默退化，產出一個看起來合理的錯誤答案
    (43,406 m)。所以錯誤訊息一定要指名半徑與機號。
    """
    b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=120.0)
    with pytest.raises(ValueError) as ei:
        b._dubins_for(1.0, uav_id=9)
    msg = str(ei.value)
    assert "1.0" in msg and "9" in msg, f"訊息沒指名半徑與機號: {msg}"


def test_multirotors_still_take_the_fleet_fallback():
    """多旋翼(turn_radius=0)在本階段**維持**既有行為:拿機隊半徑。

    ⚠️ 這是刻意保留的,不是遺漏。改成直線會動到既有 golden,屬於另一個
    獨立的行為變更。這條鎖住現況,讓那個變更發生時看得見。
    """
    res = _plan([_drone(1, R_PIONEER, 21.5, 0.000),
                 {"drone_id": 2, "position": (LAT - 0.005, LON + 0.002),
                  "vehicle_type": "multirotor", "speed": 10.0, "turn_radius": 0.0}])
    assert res["assembled_paths"][2].turn_radius_m == pytest.approx(R_PIONEER)


def test_legacy_collapse_no_longer_depends_on_list_order():
    """⚠️ 舊管線原本取「清單第一台」，v2 取「最大值」——同一個函式兩套規則。

    實測同一組機隊只換 drones 清單順序，`DCCPPPathAssembler` 就從 160 變 100。
    """
    import core.trajectory.dccpp_path_assembler as DPA
    seen = []
    orig = DPA.DCCPPPathAssembler.__init__

    def spy(self, *a, **k):
        seen.append(k.get("turn_radius"))
        return orig(self, *a, **k)
    DPA.DCCPPPathAssembler.__init__ = spy
    try:
        _plan(MIXED)
        first = list(seen)
        seen.clear()
        _plan(list(reversed(MIXED)))
        second = list(seen)
    finally:
        DPA.DCCPPPathAssembler.__init__ = orig
    assert first == second, f"換順序就變了: {first} vs {second}"


def test_heterogeneous_fleet_is_announced():
    """異質機隊要留下一行紀錄，說明分配階段仍用機隊值。

    ⚠️ 這條只鎖 log，所以**不能**當作功能驗收 —— 功能驗收在上面的收據那幾條。
    """
    logger = logging.getLogger("mission.swarm_coordinator")
    recs = []

    class H(logging.Handler):
        def emit(self, r):
            recs.append(r.getMessage())
    h = H()
    logger.addHandler(h)
    old = logger.level
    logger.setLevel(logging.INFO)
    try:
        _plan(MIXED)
    finally:
        logger.removeHandler(h)
        logger.setLevel(old)
    assert any("異質機隊" in m for m in recs), "沒有宣告異質機隊"


# ═══════════════════════════════════════════════════════════════════
#  6. 收據不可以說謊 —— 突變實驗補上的四條
# ═══════════════════════════════════════════════════════════════════
def _idp_fixture():
    from core.dccpp.area_processor import AreaProcessor
    from core.geometry.coordinate import CoordinateTransformer
    ar = AreaProcessor().process_latlon(_area()[0]["polygon"], 300.0, 30.0,
                                        scan_angle_deg=None)
    return ar, CoordinateTransformer(*ar.centroid_latlon)


#: 混編 [Pioneer 160, HERO 100]、9 條帶下 IDP 的實測值。
#: ⚠️ 這兩個數字是**唯一**能分辨「真的用了 100」與「只是收據寫 100」的證據。
_HERO_LEN_AT_100 = 15767.475
_HERO_LEN_AT_160 = 16583.211


def test_the_length_matches_the_radius_the_receipt_claims():
    """收據說用了 100，長度就必須是 100 排出來的長度。

    ⚠️⚠️ 這條是突變實驗補上的。原本若 `_solve_single` 忽略傳進來的 cost、
    改用機隊快取，**收據照樣回報 100**（因為收據讀的是 `cost.R`），
    而實際是用 160 排的 —— 收據說謊，其餘 16 條測試全綠。
    """
    ar, tf = _idp_fixture()
    s = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    out = s.solve(
        [{"uav_id": 1, "lat": LAT - 0.005, "lon": LON, "heading": 90.0,
          "turn_radius": R_PIONEER},
         {"uav_id": 2, "lat": LAT - 0.005, "lon": LON + 0.002, "heading": 90.0,
          "turn_radius": R_HERO}],
        ar, tf)
    hero = out.per_uav[2]
    assert hero.turn_radius_m == pytest.approx(R_HERO)
    assert hero.total_length_m == pytest.approx(_HERO_LEN_AT_100, rel=1e-6), (
        f"收據說 {hero.turn_radius_m} 但長度 {hero.total_length_m} 對不上；"
        f"用機隊 160 排會得到 {_HERO_LEN_AT_160}")
    # 而且必須**不等於**機隊半徑排出來的值，否則這條沒有鑑別力
    assert hero.total_length_m != pytest.approx(_HERO_LEN_AT_160, rel=1e-6)


def test_solve_single_really_uses_the_cost_it_is_handed():
    """把兩個不同半徑的快取分別餵進去，長度必須不同。

    ⚠️ 直接打在 `_solve_single` 上，不經過 solve()，所以「有沒有真的用」
    與「有沒有傳」分得開。
    """
    ar, _ = _idp_fixture()
    s = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    lines = ar.scan_lines[:5]
    a = s._solve_single(lines, 0.0, 0.0, 0.0, s.cost_cache_for(R_HERO))[2]
    b = s._solve_single(lines, 0.0, 0.0, 0.0, s.cost_cache_for(R_PIONEER))[2]
    assert a != pytest.approx(b), (
        f"換了成本快取長度卻一樣（{a}）—— cost 參數沒被用到")
    assert a < b, "較小半徑應該給出較短的成本"


def test_solve_single_refuses_to_default_its_cost():
    """漏傳 cost 必須是 TypeError，不可以靜默退回機隊快取。

    ⚠️ 用**非空**的 scan lines，否則函式可能在碰到 cost 之前就回傳，
    TypeError 會是別的原因造成的（第一版就是這樣，恆綠）。
    """
    ar, _ = _idp_fixture()
    s = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    with pytest.raises(TypeError):
        s._solve_single(ar.scan_lines[:3], 0.0, 0.0, 0.0)


def test_the_two_layer_consistency_check_actually_fires():
    """可觸發性:強迫兩層不一致，協調器必須拋錯而不是默默照做。

    ⚠️ 沒有這一條，「拿掉一致性檢查」的突變完全偵測不到 —— 因為正確的
    程式碼裡兩層本來就相等，那個檢查永遠不會被觸發。
    """
    from core.dccpp.dccpp_path_builder import DCCPPPathBuilder
    orig = DCCPPPathBuilder._dubins_for

    def always_fleet(self, radius, uav_id=0):
        return orig(self, self.R, uav_id)      # 忽略逐機半徑 ⇒ 收據變機隊值
    DCCPPPathBuilder._dubins_for = always_fleet
    try:
        with pytest.raises(RuntimeError, match="不一致"):
            _plan(MIXED)
    finally:
        DCCPPPathBuilder._dubins_for = orig


def test_the_generator_readback_guard_fires():
    """可觸發性:讓反解空速壞掉，成本快取必須拒收而不是照抄。"""
    from core.dccpp.idp_solver import _DubinsCostCache
    import core.base.fixed_wing_constraints as FWC
    orig = FWC.FixedWingConstraints.get_min_turn_radius

    def wrong(self, *a, **k):
        return orig(self, *a, **k) * 2.0
    FWC.FixedWingConstraints.get_min_turn_radius = wrong
    try:
        with pytest.raises(ValueError, match="反解空速"):
            _DubinsCostCache(100.0, altitude=120.0)
    finally:
        FWC.FixedWingConstraints.get_min_turn_radius = orig


def test_strip_partition_is_pinned():
    """分配(誰拿幾條)必須釘死。

    ⚠️ 只比「異質 vs 同質」不夠:若分配階段被改成對**所有**機體都用某個
    固定半徑，兩邊會一起變，比較仍然相等。所以這裡釘絕對值。
    """
    ar, tf = _idp_fixture()
    assert len(ar.scan_lines) == 9, "條帶數變了，下面的 golden 要重新量"
    s = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    out = s.solve(
        [{"uav_id": 1, "lat": LAT - 0.005, "lon": LON, "heading": 90.0,
          "turn_radius": R_PIONEER},
         {"uav_id": 2, "lat": LAT - 0.005, "lon": LON + 0.002, "heading": 90.0,
          "turn_radius": R_HERO}],
        ar, tf)
    counts = {u: len(p.ordered_scan_lines) for u, p in sorted(out.per_uav.items())}
    assert counts == {1: 5, 2: 4}, f"分配被動到了: {counts}"


def test_allocation_search_still_runs_at_the_fleet_radius():
    """分配搜尋必須跑在**機隊**半徑上，只有生成那一次用逐機半徑。

    ⚠️⚠️ 這條是突變實驗第三輪補上的。前兩個分配守衛都沒有鑑別力:
    這組幾何下 joint 搜尋不論用哪個半徑都收斂到同一組切分，
    所以「條帶數」與「條帶歸屬」兩個量都測不出分配被改掉。

    真正分得出來的是**呼叫結構**:正常是 {160: 45 次, 100: 1 次}
    （45 次是搜尋、1 次是 HERO 的生成）；把分配也改成逐機之後
    變成 {100: 46 次}，機隊半徑一次都沒被用到。
    """
    import collections
    import core.dccpp.idp_solver as M

    ar, tf = _idp_fixture()
    counts = collections.Counter()
    orig = M.IDP_Solver._solve_single

    def spy(self, sub, sx, sy, sh, cost):
        counts[round(cost.R)] += 1
        return orig(self, sub, sx, sy, sh, cost)
    M.IDP_Solver._solve_single = spy
    try:
        M.IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0).solve(
            [{"uav_id": 1, "lat": LAT - 0.005, "lon": LON, "heading": 90.0,
              "turn_radius": R_PIONEER},
             {"uav_id": 2, "lat": LAT - 0.005, "lon": LON + 0.002, "heading": 90.0,
              "turn_radius": R_HERO}],
            ar, tf)
    finally:
        M.IDP_Solver._solve_single = orig

    fleet_calls = counts[round(R_PIONEER)]
    hero_calls = counts[round(R_HERO)]
    assert fleet_calls >= 10, (
        f"機隊半徑只被用了 {fleet_calls} 次 —— 分配搜尋被改成逐機了。實測 {dict(counts)}")
    assert hero_calls == 1, (
        f"HERO 半徑被用了 {hero_calls} 次，應該只有生成那一次。實測 {dict(counts)}")


# ═══════════════════════════════════════════════════════════════════
#  7. 對抗式審查補上的三條
# ═══════════════════════════════════════════════════════════════════
def test_an_unusable_radius_names_itself_on_the_solver_path_too():
    """⚠️⚠️ builder 那一側早就有具名錯誤，**solver 這一側漏了**。

    `cost_cache_for(3.0)` 會去建 `_DubinsCostCache`，反解出的空速 4.95 m/s
    掉到失速 7.0 以下，`FixedWingConstraints` 丟的訊息是
    「失速速度必須 < 巡航空速」—— **完全沒提到半徑**，看起來像不相干的空速錯誤，
    而且會被 v2 管線外層那個 `except Exception` 吞掉，靜默退回舊 solver。
    """
    from core.dccpp.idp_solver import TurnRadiusUnusable
    s = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    with pytest.raises(TurnRadiusUnusable) as ei:
        s.cost_cache_for(3.0)
    assert "3.0" in str(ei.value), f"訊息沒指名半徑: {ei.value}"


def test_an_unusable_radius_does_not_degrade_the_pipeline_silently():
    """半徑不能用時必須拋出去，不可以退化成一份看起來正常的舊 solver 計畫。"""
    from core.dccpp.idp_solver import TurnRadiusUnusable
    bad = [_drone(1, R_PIONEER, 21.5, 0.000), _drone(2, 3.0, 19.0, 0.002)]
    with pytest.raises(TurnRadiusUnusable):
        _plan(bad)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_radius_is_refused(bad):
    """NaN 特別危險:`abs(NaN - x) > tol` 是 False，所有讀回檢查都會放行。"""
    from core.dccpp.idp_solver import TurnRadiusUnusable
    s = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0)
    with pytest.raises(TurnRadiusUnusable):
        s.cost_cache_for(bad)


def test_an_aircraft_with_no_strips_still_reports_a_real_radius():
    """⚠️ 沒分到條帶的 UAV，`UAVPlan.turn_radius_m` 以前留預設 **0.0**，
    而下游 builder 仍寫入真實半徑 ⇒ 收據變成 (0.0, 160.0) 這種自相矛盾的一組。

    後果:`ordering_m == generation_m` 那條不變式會在**合法**的計畫上假紅 ——
    只要機隊台數多於條帶數就會發生，而面板允許 2–12 台。
    """
    from core.dccpp.area_processor import AreaProcessor
    from core.geometry.coordinate import CoordinateTransformer
    ar = AreaProcessor().process_latlon(
        [(LAT, LON), (LAT, LON + 0.002),
         (LAT + 0.002, LON + 0.002), (LAT + 0.002, LON)],
        300.0, 30.0, scan_angle_deg=None)
    assert len(ar.scan_lines) < 3, (
        f"這塊區域有 {len(ar.scan_lines)} 條帶，撐得起 3 台，測不到閒置機")
    tf = CoordinateTransformer(*ar.centroid_latlon)
    out = IDP_Solver(min_turn_radius=R_PIONEER, altitude=120.0).solve(
        [{"uav_id": i, "lat": LAT, "lon": LON, "heading": 90.0, "turn_radius": r}
         for i, r in ((1, R_PIONEER), (2, R_HERO), (3, R_HERO))], ar, tf)
    idle = [p for p in out.per_uav.values() if not p.ordered_scan_lines]
    assert idle, "沒有閒置的 UAV —— 這條測試量不到東西"
    for p in idle:
        assert p.turn_radius_m > 0.0, (
            f"UAV {p.uav_id} 沒分到條帶就回報半徑 {p.turn_radius_m}，"
            "會讓一致性不變式假紅")
        assert p.turn_radius_m == pytest.approx(R_HERO)
