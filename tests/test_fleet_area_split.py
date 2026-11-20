"""模式一：使用者只畫一大塊，`split_area_for_fleet` 自動切給機隊。

三個 2026-09-08 抓到並修掉的缺陷，每一條都附**會失敗的具體輸入**：

1. `_min_width_dir` 的旋轉投影符號寫反 ⇒ 對 180° 裡的 90° 方位選到**長軸**當
   最小寬度方向，1560×800 的區塊被切成 390×800 的短塊而不是 1560×200 的長條，
   條帶剁成 1/4 長、掉頭翻倍。
   ⚠⚠⚠ sin 項在 0°、cos 項在 90° 各自消失 ⇒ **軸對齊的區塊剛好免疫**，
   而既有測試（與飛過的 HV4SITL9）用的都是軸對齊矩形，所以一直沒被抓到。
   這正是「守衛要測在會改變答案的地方」的又一例。
2. 自交多邊形被 `buffer(0)` 靜默修成一半，而「面積守恆」守衛的基準線是
   **修好之後**的形狀 ⇒ 結構上看不見那 50%，還照樣印成功。
3. 份數只做四捨五入 ⇒ 3 塊等面積區配 4 台時每塊 k=round(1.33)=1，完全不切，
   第 4 台分不到工作。
"""
from __future__ import annotations

import logging as _logging
import math

import pytest

pytest.importorskip("shapely")

from shapely.geometry import Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

from mission.swarm_coordinator import (_min_width_dir,  # noqa: E402
                                       split_area_for_fleet)

LAT0, LON0 = 23.0, 120.0
M_LAT = 111132.0
M_LON = 111412.0 * math.cos(math.radians(LAT0))


def _ll(e: float, n: float) -> tuple[float, float]:
    return (LAT0 + n / M_LAT, LON0 + e / M_LON)


def _xy(p) -> tuple[float, float]:
    return ((p[1] - LON0) * M_LON, (p[0] - LAT0) * M_LAT)


def _rect(deg: float, half_e: float = 780.0, half_n: float = 400.0,
          cx: float = 0.0, cy: float = 0.0):
    """以原點為中心、旋轉 `deg` 度的矩形（HV4SITL9 的 1560×800 幾何）。"""
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return [_ll(cx + c * x - s * y, cy + s * x + c * y)
            for x, y in ((-half_e, -half_n), (half_e, -half_n),
                         (half_e, half_n), (-half_e, half_n))]


def _piece_size(area) -> tuple[float, float]:
    """(長邊, 短邊) 公尺 —— 用最小旋轉外接矩形量，不受方位影響。"""
    q = Polygon([_xy(p) for p in area["polygon"]]).minimum_rotated_rectangle
    xs, ys = q.exterior.coords.xy
    e = [math.dist((xs[i], ys[i]), (xs[i + 1], ys[i + 1])) for i in range(4)]
    return max(e[:2]), min(e[:2])


# ── 1. 最小寬度方向 ──────────────────────────────────────────────────
def test_min_width_dir_matches_brute_force():
    """對每個方位，`_min_width_dir` 給的方向的實際寬度要等於暴力掃出來的最小值。

    ⚠ 直接比「回傳角度」會被 180° 週期與等寬邊咬到，所以比**寬度**。
    """
    for deg in range(0, 180, 7):
        pts = [_xy(p) for p in _rect(deg)]
        th = _min_width_dir(pts)
        got = _width_along(pts, th)
        best = min(_width_along(pts, math.radians(d / 4.0)) for d in range(720))
        assert got == pytest.approx(best, abs=1.0), \
            f"方位 {deg}°：選到的方向寬 {got:.1f} m，最小其實是 {best:.1f} m"


def _width_along(pts, th: float) -> float:
    """把點旋轉 −th 之後在 y 方向的跨距（＝垂直於 th 的寬度）。"""
    c, s = math.cos(-th), math.sin(-th)
    ys = [s * x + c * y for x, y in pts]
    return max(ys) - min(ys)


def test_the_cut_keeps_strips_long_at_every_orientation():
    """⚠⚠⚠ 這一條是本檔的主角。

    1560×800 的區塊切成 4 塊時，每塊必須是 **1560×200**（沿掃描步進軸切，
    條帶方向與長度不變），不可以是 390×800（沿另一軸切，條帶剁成 1/4）。

    修掉符號錯誤之前：0/15/75/90 度是對的，30/45/60 度全錯。
    只測軸對齊等於量不到東西 —— 飛過的 HV4SITL9 就是軸對齊的。
    """
    for deg in (0, 15, 30, 45, 60, 75, 90, 120, 150, 179):
        out = split_area_for_fleet(
            [{"area_id": 1, "priority": 1.0, "polygon": _rect(deg)}], 4)
        assert len(out) == 4, (deg, len(out))
        for a in out:
            lo_, hi_ = sorted(_piece_size(a))
            assert hi_ == pytest.approx(1560.0, abs=2.0), \
                f"方位 {deg}°：長邊 {hi_:.0f} m，條帶被剁短了"
            assert lo_ == pytest.approx(200.0, abs=2.0), (deg, lo_)


# ── 2. 面積守恆與不合法輸入 ──────────────────────────────────────────
def test_area_is_conserved_at_every_orientation():
    for deg in (0, 30, 45, 90, 137):
        polys = _rect(deg)
        out = split_area_for_fleet(
            [{"area_id": 1, "priority": 1.0, "polygon": polys}], 4)
        par = Polygon([_xy(p) for p in polys])
        ps = [Polygon([_xy(p) for p in a["polygon"]]) for a in out]
        u = unary_union(ps)
        assert par.area - u.area == pytest.approx(0.0, abs=1.0), deg
        assert sum(p.area for p in ps) - u.area == pytest.approx(0.0, abs=1.0), deg


def test_self_intersecting_area_is_refused_not_silently_halved():
    """⚠⚠⚠ 蝴蝶結：兩瓣共 500,000 m²，`buffer(0)` 只留 250,000。

    舊版靜默接受，而「面積守恆」守衛的基準線是修好之後的形狀
    ⇒ 結構上看不見那 50%，還印 `fleet_split(...)` 成功訊息。
    """
    bow = [_ll(0, 0), _ll(1000, 1000), _ll(1000, 0), _ll(0, 1000)]
    with pytest.raises(ValueError, match="不合法"):
        split_area_for_fleet([{"area_id": 1, "priority": 1.0, "polygon": bow}], 4)


def test_the_refusal_says_how_much_would_have_been_lost():
    """只說「不合法」不夠 —— 要讓使用者看到修補會吃掉多少。"""
    bow = [_ll(0, 0), _ll(1000, 1000), _ll(1000, 0), _ll(0, 1000)]
    with pytest.raises(ValueError) as ei:
        split_area_for_fleet([{"area_id": 1, "priority": 1.0, "polygon": bow}], 4)
    msg = str(ei.value)
    assert "249,992" in msg or "250,0" in msg, msg      # 修補後
    assert "area_id=1" in msg, msg


def test_a_valid_concave_area_still_splits():
    """反向臂：凹多邊形（合法）不可以被上面那道守衛擋掉。"""
    ell = [_ll(0, 0), _ll(1600, 0), _ll(1600, 500), _ll(700, 500),
           _ll(700, 1500), _ll(0, 1500)]
    out = split_area_for_fleet([{"area_id": 1, "priority": 1.0, "polygon": ell}], 4)
    assert len(out) >= 2
    par = Polygon([_xy(p) for p in ell])
    u = unary_union([Polygon([_xy(p) for p in a["polygon"]]) for a in out])
    assert par.area - u.area == pytest.approx(0.0, abs=1.0)


# ── 3. 份數要餵飽整個機隊 ────────────────────────────────────────────
def test_three_equal_areas_still_feed_four_aircraft():
    """⚠ 只做四捨五入時 k=round(4×1/3)=1 ⇒ 完全不切，3 個胞餵 4 台。"""
    areas = [{"area_id": i + 1, "priority": 1.0,
              "polygon": _rect(0, 300, 300, cx=3000 * i)} for i in range(3)]
    out = split_area_for_fleet(areas, 4)
    assert len(out) >= 4, f"3 塊配 4 台只切出 {len(out)} 塊"
    assert len({a["area_id"] for a in out}) == len(out), "子區 id 撞號"


@pytest.mark.parametrize("n_area,n_uav", [(1, 4), (1, 2), (2, 4), (3, 4), (4, 4), (5, 4)])
def test_piece_count_reaches_the_fleet_size(n_area: int, n_uav: int):
    areas = [{"area_id": i + 1, "priority": 1.0,
              "polygon": _rect(0, 300, 300, cx=3000 * i)} for i in range(n_area)]
    out = split_area_for_fleet(areas, n_uav)
    assert len(out) >= min(n_uav, n_area * 16), (n_area, n_uav, len(out))


def test_parent_keys_survive_the_split():
    a = {"area_id": 1, "priority": 2.5, "polygon": _rect(0),
         "scan_angle_deg": 37.0, "sensor_req": "rgb", "payload_req_id": "default"}
    for p in split_area_for_fleet([a], 4):
        for k in ("priority", "scan_angle_deg", "sensor_req", "payload_req_id"):
            assert p[k] == a[k], k
        assert p["_src_area_id"] == 1


def test_degenerate_inputs_pass_through():
    assert split_area_for_fleet([], 4) == []
    one = [{"area_id": 1, "priority": 1.0, "polygon": _rect(0)}]
    assert split_area_for_fleet(one, 1) == one
    assert split_area_for_fleet(one, 0) == one
    short = [{"area_id": 1, "priority": 1.0, "polygon": [_ll(0, 0), _ll(10, 0)]}]
    assert split_area_for_fleet(short, 4) == short


def test_the_split_is_announced(caplog):
    with caplog.at_level(_logging.WARNING, logger="mission.swarm_coordinator"):
        split_area_for_fleet([{"area_id": 1, "priority": 1.0, "polygon": _rect(0)}],
                             4, logger=_logging.getLogger("mission.swarm_coordinator"))
    assert [r for r in caplog.records if "fleet_area_split" in r.getMessage()]


# ── 4. 禁航區切區的 id 要與機隊切區一致（都用整數） ────────────────────
def _zone(vertices):
    from mission.fence_zone import FenceZone, ZoneCategory, ZoneShape
    return FenceZone(name="N1", category=ZoneCategory.NFZ, shape=ZoneShape.POLYGON,
                     vertices=vertices, alt_min_m=0.0, alt_max_m=500.0)


def test_nfz_split_uses_integer_ids_not_underscored_strings():
    """⚠⚠⚠ 舊版寫 `f"{area_id}_{k+1}"` ⇒ 轉接層 `int()` 把底線吃掉，

    "3_1" 變成 31，與使用者真的畫的第 31 塊區、或機隊切區產生的 31 號子區
    撞號，而且撞號之後**完全靜默**。機隊切區早就改成整數 id 了，這裡跟進。
    """
    from mission.swarm_coordinator import split_areas_around_nfz
    sp = split_area_for_fleet(
        [{"area_id": 1, "priority": 1.0, "polygon": _rect(0)}], 4)
    out = split_areas_around_nfz(
        sp, [_zone([_ll(-120, -120), _ll(120, -120), _ll(120, 120), _ll(-120, 120)])],
        inset_m=70.0)
    ids = [a["area_id"] for a in out]
    assert all(isinstance(i, int) for i in ids), f"還有非整數 id：{ids}"
    assert len(ids) == len(set(ids)), f"子區 id 撞號：{ids}"
    assert "_" not in "".join(str(i) for i in ids)


def test_nfz_split_keeps_the_original_area_as_the_source():
    """兩層切完之後，每一塊都要能指回**使用者畫的那一塊**（不是中間層）。"""
    from mission.swarm_coordinator import split_areas_around_nfz
    sp = split_area_for_fleet(
        [{"area_id": 1, "priority": 1.0, "polygon": _rect(0)}], 4)
    out = split_areas_around_nfz(
        sp, [_zone([_ll(-120, -120), _ll(120, -120), _ll(120, 120), _ll(-120, 120)])],
        inset_m=70.0)
    assert {a.get("_src_area_id") for a in out} == {1}, \
        [a.get("_src_area_id") for a in out]
