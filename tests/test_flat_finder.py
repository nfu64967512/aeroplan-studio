"""「找平坦位置」核心（core/terrain/flat_finder.py）與取樣器的向量化 `sample_grid`。

用假地形（解析函數）斷言**值**：每個情境先證明「不做這件事就會選錯」。真實資料的對照在 evidence 腳本
（桃園實測：同尺寸 0 個、70% 少數、50% 很多；預測可行者規劃器 0 次 NO_GO）。
"""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

from core.terrain.flat_finder import (FlatSearchError, areas_window, find_flat_windows, required_ceiling_m,
                                      transform_areas, transform_point, translate_areas)

LAT0, LON0 = 25.0, 121.0
M_LAT = 111320.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))


def xy(la, lo):
    """緯經度（可為 numpy 陣列）→ 以 (LAT0, LON0) 為原點的 (東 m, 北 m)。"""
    return (np.asarray(lo) - LON0) * M_LON, (np.asarray(la) - LAT0) * M_LAT


def hills(la, lo, amp=60.0):
    x, y = xy(la, lo)
    return 100.0 + amp * np.sin(x / 700.0) * np.cos(y / 900.0) + 25.0 * np.sin((x + y) / 300.0)


class Terr:
    """假地形：``fn(la, lo)``（支援純量與陣列）；可選有/無 sample_grid；記錄 prefetch。"""

    def __init__(self, fn, *, grid=True, none_where=None):
        self.fn, self.none_where, self.prefetched, self.failures = fn, none_where, [], []
        if grid:
            self.sample_grid = self._grid

    def prefetch(self, bounds, **kw):
        self.prefetched.append(bounds)

    def _bad(self, la, lo):
        return self.none_where is not None and self.none_where(la, lo)

    def get_elevation(self, la, lo):
        return None if self._bad(la, lo) else float(self.fn(la, lo))

    def _grid(self, lats, lons):
        LA, LO = np.meshgrid(lats, lons, indexing="ij")
        Z = np.asarray(self.fn(LA, LO), dtype=float)
        if self.none_where is not None:
            Z = np.where(self.none_where(LA, LO), np.nan, Z)
        return Z


def with_flat_block(cx=1500.0, cy=-900.0, radius=1300.0, z=80.0):
    """平坦區塊：有 ~5 m 的細微起伏（真實平原不會是完全固定的高度；固定高度會被當成水面排除）。"""
    def fn(la, lo):
        x, y = xy(la, lo)
        inside = np.hypot(x - cx, y - cy) < radius
        return np.where(inside, z + 2.5 * np.sin(x / 137.0) * np.cos(y / 151.0), hills(la, lo))
    return fn


CENTER = (LAT0, LON0)
KW = dict(min_fw_agl_m={"A": 100.0, "B": 90.0}, band_min_m=5.0, ceil_m=120.0, step_m=30.0, top_n=5)
WIN = (600.0, 600.0)


def _find(terr, **over):
    kw = {**KW, **over}
    return find_flat_windows(terr, CENTER, over.pop("radius_km", 4.0) if "radius_km" in over else 4.0, *WIN,
                             **{k: v for k, v in kw.items() if k != "radius_km"})


# ── 前提：沒有平坦區塊時，丘陵地沒有任何位置能不用豁免 ─────────────────────────
def test_precondition_plain_hills_have_nothing_flat_enough():
    r = _find(Terr(hills))
    assert r.candidates and not any(r.any_fits.values()), [c.relief_m for c in r.candidates]
    assert min(c.relief_m for c in r.candidates) > 25.0


# ── 找得到平坦區塊 ───────────────────────────────────────────────────────────
@pytest.mark.parametrize("grid", [True, False], ids=["sample_grid", "per-point"])
def test_finds_the_flat_block_and_says_it_fits(grid):
    r = _find(Terr(with_flat_block(), grid=grid))
    best = r.candidates[0]
    x, y = xy(best.lat, best.lon)
    assert math.hypot(x - 1500.0, y + 900.0) < 700.0, (x, y)           # 在平坦區塊內（窗口完整落在圓內）
    assert best.relief_m < 6.0
    assert best.fits == {"A": True, "B": True} and r.any_fits == {"A": True, "B": True}
    assert best.need_m["A"] == pytest.approx(100.0 + 5.0 + best.relief_m)
    assert best.need_m["B"] == pytest.approx(90.0 + 5.0 + best.relief_m)
    assert best.slack_m["B"] == pytest.approx(120.0 - best.need_m["B"])


def test_vectorised_and_per_point_sampling_give_the_same_answer():
    a = _find(Terr(with_flat_block(), grid=True))
    b = _find(Terr(with_flat_block(), grid=False))
    assert [(round(c.lat, 5), round(c.lon, 5), round(c.relief_m, 2)) for c in a.candidates] == \
           [(round(c.lat, 5), round(c.lon, 5), round(c.relief_m, 2)) for c in b.candidates]


def test_candidates_that_fit_come_before_flatter_looking_ones_that_do_not():
    """排序：能不用豁免者優先，其次起伏（2 m 一級）、再近的。"""
    r = _find(Terr(with_flat_block()), top_n=6)
    fits = [any(c.fits.values()) for c in r.candidates]
    assert fits == sorted(fits, reverse=True), fits                  # True 全在前
    assert fits[0] is True


def two_tilted_plateaus(near_relief, far_relief):
    """兩塊傾斜的平台（平面斜坡 ⇒ 窗口起伏 = 斜率 × 窗口長，可精確控制）：近的在 (-1300, 600)、遠的在 (2600, -800)。"""
    span = 960.0                                           # 33 格 × 30 m 的窗口跨度（含外擴 200 m）
    def fn(la, lo):
        x, y = xy(la, lo)
        near = (np.abs(x + 1300.0) < 1250.0) & (np.abs(y - 600.0) < 1250.0)
        far = (np.abs(x - 2600.0) < 1250.0) & (np.abs(y + 800.0) < 1250.0)
        return np.where(near, 100.0 + near_relief / span * x, np.where(far, 100.0 + far_relief / span * x, hills(la, lo)))
    return fn


def test_within_one_relief_bucket_a_window_that_fits_beats_a_nearer_one_that_does_not():
    """B 的門檻 = 120 − 90 − 5 = 25 m。近的平台起伏 ~25.9（容不下）、遠的 ~24.1（容得下），兩者同在 2 m 一級（24–26）：
    只比「級距 + 距離」會讓近的先；必須先看「能不能不用豁免」。"""
    r = _find(Terr(two_tilted_plateaus(25.9, 24.1)), radius_km=4.0, top_n=60)
    near = [c for c in r.candidates if xy(c.lat, c.lon)[0] < 0.0]
    far = [c for c in r.candidates if xy(c.lat, c.lon)[0] > 1500.0]
    assert near and far, [(round(float(xy(c.lat, c.lon)[0])), round(c.relief_m, 1)) for c in r.candidates]
    assert 25.0 < near[0].relief_m < 26.5 and not near[0].fits["B"]
    assert 23.5 < far[0].relief_m < 25.0 and far[0].fits["B"]
    assert near[0].distance_km < min(c.distance_km for c in far), "前提：近的平台確實比較近"
    assert r.candidates.index(far[0]) < r.candidates.index(near[0]), "容得下的要排在容不下的前面"


def test_a_big_nearby_area_that_does_not_fit_cannot_crowd_the_fitting_ones_out_of_the_pool():
    """近處有一大片（~36 個窗口）起伏 25.9 m 的平台（B 的門檻 25 m ⇒ 容不下），遠處才有一小塊 24.1 m（容得下）：
    只取前幾名時，容得下的不能被擠掉。"""
    span = 960.0
    def fn(la, lo):
        x, y = xy(la, lo)
        near = (np.abs(x + 1800.0) < 1900.0) & (np.abs(y) < 1900.0)
        far = (np.abs(x - 4300.0) < 1100.0) & (np.abs(y) < 1100.0)
        return np.where(near, 100.0 + 25.9 / span * x, np.where(far, 100.0 + 24.1 / span * x, hills(la, lo)))
    r = _find(Terr(fn), radius_km=6.0, top_n=2)
    assert r.any_fits["B"], [(round(float(xy(c.lat, c.lon)[0])), round(c.relief_m, 1)) for c in r.candidates]
    assert r.candidates[0].fits["B"] and xy(r.candidates[0].lat, r.candidates[0].lon)[0] > 3000.0


def test_candidates_are_spread_out():
    r = _find(Terr(with_flat_block()), top_n=6)
    min_sep = 0.5 * 600.0 + 200.0
    pts = [xy(c.lat, c.lon) for c in r.candidates]
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            assert math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]) >= min_sep - 1.0


def test_only_looks_inside_the_radius():
    """平坦區塊在 ~1.7 km 外：半徑 1 km 找不到、4 km 找得到。"""
    near = _find(Terr(with_flat_block()), radius_km=1.0)
    far = _find(Terr(with_flat_block()), radius_km=4.0)
    assert not any(near.any_fits.values())
    assert any(far.any_fits.values())
    for c in near.candidates:
        assert c.distance_km <= 1.0 + 1e-6


def test_the_search_downloads_the_radius_plus_window_plus_margin():
    t = Terr(hills)
    _find(t, radius_km=4.0)
    (la0, lo0, la1, lo1), = t.prefetched
    need_m = 4000.0 + 300.0 + 200.0
    assert (LAT0 - la0) * M_LAT >= need_m - 1.0 and (la1 - LAT0) * M_LAT >= need_m - 1.0
    assert (LON0 - lo0) * M_LON >= need_m - 1.0 and (lo1 - LON0) * M_LON >= need_m - 1.0


# ── 外擴（規劃器的胞外擴一個轉彎距離才量地形）─────────────────────────────────
def test_margin_rejects_a_flat_patch_that_is_ringed_by_hills():
    """窗口剛好落在一小塊平地、但外圍 200 m 內是高山：margin=0 以為可行，margin=200 才看得到真相。"""
    def fn(la, lo):
        x, y = xy(la, lo)
        inside = (np.abs(x - 1500.0) < 320.0) & (np.abs(y + 900.0) < 320.0)
        flat = 80.0 + 1.0 * np.sin(x / 90.0) * np.cos(y / 110.0)
        return np.where(inside, flat, 80.0 + 60.0 * (1.0 - np.exp(-((np.hypot(x - 1500.0, y + 900.0) - 320.0) / 150.0) ** 2)))
    t = Terr(fn)
    lax = _find(t, margin_m=0.0)
    strict = _find(Terr(fn), margin_m=200.0)
    assert any(lax.any_fits.values()), "前提：不外擴時看起來可行"
    assert not any(strict.any_fits.values()), "外擴 200 m 後應看出外圍有山"


# ── 水面/海面 ────────────────────────────────────────────────────────────────
def test_sea_level_cells_are_not_offered_as_flat_land():
    """左半是海（高度固定 18 m，占 >3%）、右半是丘陵＋一小塊平地：平坦候選必須在陸地的平坦區塊，不是海。"""
    def fn(la, lo):
        x, y = xy(la, lo)
        land = np.where(np.hypot(x - 2200.0, y + 400.0) < 1300.0, 90.0, hills(la, lo))
        return np.where(x < -500.0, 18.0, land)
    r = _find(Terr(fn))
    assert r.water_fraction > 0.2 and any("水面" in n for n in r.notes)
    for c in r.candidates:
        x, _ = xy(c.lat, c.lon)
        assert x > -500.0 + 300.0 + 200.0 - 1.0, "候選窗口含海面格"
    assert r.candidates[0].fits["B"] and r.candidates[0].z_min > 50.0


def test_a_huge_textured_plain_is_not_mistaken_for_water():
    """搜尋範圍 >50% 是略有起伏的大平原（取整後眾數占比遠超 8%）：它是陸地，不能被當水面排除。"""
    def fn(la, lo):
        x, y = xy(la, lo)
        plain = 80.0 + 2.5 * np.sin(x / 137.0) * np.cos(y / 151.0)
        return np.where(x > -1500.0, plain, hills(la, lo))
    r = _find(Terr(fn))
    assert r.water_fraction < 0.01, r.notes
    assert r.candidates[0].fits["B"] and 70.0 < r.candidates[0].z_min


def test_water_exclusion_can_be_switched_off():
    def fn(la, lo):
        x, _ = xy(la, lo)
        return np.where(x < -500.0, 18.0, hills(la, lo))
    on = _find(Terr(fn), exclude_water=True)
    off = _find(Terr(fn), exclude_water=False)
    assert not any(c.fits["B"] for c in on.candidates)
    assert off.candidates[0].fits["B"] and off.candidates[0].z_max < 19.0


# ── 數字 ─────────────────────────────────────────────────────────────────────
def test_required_ceiling_uses_the_clearance_floor():
    assert required_ceiling_m(10.0, 100.0, 15.0) == pytest.approx(125.0)
    assert required_ceiling_m(10.0, 30.0, 5.0, clearance_m=36.0) == pytest.approx(51.0)   # 淨空 36 > 最低作業高度 30


def test_a_raised_ceiling_turns_a_miss_into_a_fit():
    plain = _find(Terr(hills), ceil_m=120.0)
    waived = _find(Terr(hills), ceil_m=250.0)
    assert not any(plain.any_fits.values()) and all(waived.any_fits.values())
    assert waived.candidates[0].ceil_m == 250.0


def test_result_is_json_serialisable_for_the_gui_signal():
    r = _find(Terr(with_flat_block()))
    d = json.loads(json.dumps(r.to_dict()))
    assert d["candidates"][0]["fits"] == {"A": True, "B": True} and d["any_fits"] == {"A": True, "B": True}
    assert d["window_h_m"] == 600.0 and "here_relief_m" in d


# ── 失敗與參數 ───────────────────────────────────────────────────────────────
def test_too_many_failed_samples_raise_instead_of_guessing():
    t = Terr(with_flat_block(), none_where=lambda la, lo: np.asarray(lo) > LON0)        # 一半取不到
    with pytest.raises(FlatSearchError) as e:
        _find(t)
    assert "取樣失敗" in str(e.value)


def test_a_few_failed_samples_only_disqualify_the_windows_that_contain_them():
    """先找出不含洞時的第一名，再把洞打在它正中央：新的第一名必須離開那個洞（窗口 + 外擴）以外。"""
    # 平地高度 = 丘陵的中位數（100 m）：否則含洞的窗口會因為「用中位數補值」而自然變崎嶇，測不到「含洞即出局」本身
    base = _find(Terr(with_flat_block(z=100.0)))
    bx, by = xy(base.candidates[0].lat, base.candidates[0].lon)
    hole = lambda la, lo: (np.abs(xy(la, lo)[0] - bx) < 40.0) & (np.abs(xy(la, lo)[1] - by) < 40.0)
    r = _find(Terr(with_flat_block(z=100.0), none_where=hole))
    for c in r.candidates:
        x, y = xy(c.lat, c.lon)
        assert max(abs(x - bx), abs(y - by)) > 300.0 + 200.0 - 31.0, "候選窗口含取樣失敗的格"
    assert r.candidates[0].fits["B"], "洞只該讓含洞的窗口出局，周圍的平地仍可用"


def test_the_grid_is_coarsened_when_it_would_be_too_big():
    r = _find(Terr(hills), max_nodes=5000)
    assert r.step_m > 30.0 and r.cells <= 5000


@pytest.mark.parametrize("bad", [dict(radius_km=0.0), dict(height_m=0.0), dict(width_m=-5.0)])
def test_nonsense_arguments_are_refused(bad):
    a = dict(radius_km=4.0, height_m=600.0, width_m=600.0)
    a.update(bad)
    with pytest.raises(FlatSearchError):
        find_flat_windows(Terr(hills), CENTER, a["radius_km"], a["height_m"], a["width_m"], **{k: v for k, v in KW.items() if k != "top_n"})


def test_all_water_is_reported_not_returned_as_flat():
    with pytest.raises(FlatSearchError):
        _find(Terr(lambda la, lo: np.full(np.shape(la), 18.0)), radius_km=1.0)


# ── 幾何 ─────────────────────────────────────────────────────────────────────
POLY = [(25.0, 121.0), (25.0, 121.02), (25.01, 121.02), (25.01, 121.0)]


def test_areas_window_is_the_union_bounding_box():
    h, w, c = areas_window([POLY, [(25.05, 121.1), (25.05, 121.12), (25.06, 121.12)]])
    assert h == pytest.approx(0.06 * M_LAT, rel=1e-6) and c == pytest.approx((25.03, 121.06))
    assert w == pytest.approx(0.12 * 111320.0 * math.cos(math.radians(25.03)), rel=1e-6)
    with pytest.raises(FlatSearchError):
        areas_window([[(25.0, 121.0), (25.1, 121.1)]])


def test_transform_moves_the_centre_scales_about_it_and_keeps_the_shape():
    src = (25.005, 121.01)
    out = transform_areas([POLY], src, (24.0, 120.0), 0.5)[0]
    assert np.mean([p[0] for p in out]) == pytest.approx(24.0) and np.mean([p[1] for p in out]) == pytest.approx(120.0)
    assert (out[2][0] - out[0][0]) == pytest.approx(0.5 * (POLY[2][0] - POLY[0][0]))
    assert (out[1][1] - out[0][1]) == pytest.approx(0.5 * (POLY[1][1] - POLY[0][1]))
    assert translate_areas([POLY], src, src) == [list(map(tuple, POLY))]
    assert transform_point((25.0, 121.0), src, src, 1.0) == (25.0, 121.0)
    with pytest.raises(FlatSearchError):
        transform_areas([POLY], src, src, 0.0)


# ═════════════ 取樣器 sample_grid（向量化）═════════════
from tests.test_cesium_world_terrain import (EP_TOKEN, USER_TOKEN, FakeSession, plane_mesh)  # noqa: E402
from core.terrain import CesiumWorldTerrain  # noqa: E402
import core.terrain.cesium_world_terrain as cwt  # noqa: E402


def _tiles_for(levels_x, levels_y, level=4):
    d, _ = plane_mesh(h0=100.0, du=40.0, dv=10.0)
    d2, _ = plane_mesh(h0=300.0, du=-20.0, dv=60.0)
    return {(level, x, y): (d if (x + y) % 2 == 0 else d2) for x in levels_x for y in levels_y}


def test_sample_grid_equals_per_point_sampling_across_several_tiles():
    L = 4
    x0, y0 = cwt.tile_xy(25.0, 121.0, L)
    sess = FakeSession(_tiles_for(range(x0 - 1, x0 + 3), range(y0 - 1, y0 + 3)), levels=5)
    w = CesiumWorldTerrain(USER_TOKEN, session=sess)
    wb, sb, eb, nb = cwt.tile_bounds(L, x0, y0)
    lats = np.linspace(sb - 5.0, nb + 12.0, 23)                       # 跨好幾列/欄圖磚
    lons = np.linspace(wb - 8.0, eb + 20.0, 31)
    G = w.sample_grid(lats, lons)
    S = np.array([[w.get_elevation(float(la), float(lo)) for lo in lons] for la in lats])
    assert G.shape == (23, 31) and np.isfinite(G).all()
    assert np.abs(G - S).max() < 1e-3          # 圖磚點陣是 float32，兩條路徑的捨入略有不同
    assert len({round(float(v), 3) for v in G.ravel()}) > 50          # 不是一整片同值（確實取到各張圖磚）


def test_sample_grid_marks_a_failed_tile_as_nan_and_records_it():
    L = 4
    x0, y0 = cwt.tile_xy(25.0, 121.0, L)
    sess = FakeSession(_tiles_for(range(x0 - 1, x0 + 2), range(y0 - 1, y0 + 2)), levels=5, tile_status={(L, x0, y0): 500})
    w = CesiumWorldTerrain(USER_TOKEN, session=sess)
    wb, sb, eb, nb = cwt.tile_bounds(L, x0, y0)
    G = w.sample_grid(np.linspace(sb - 3, nb + 3, 9), np.linspace(wb - 3, eb + 3, 9))
    assert np.isnan(G).any() and np.isfinite(G).any()
    assert w.failures and "500" in w.failures[0]


def test_sample_grid_does_not_leak_the_token_and_reuses_the_cache():
    L = 4
    x0, y0 = cwt.tile_xy(25.0, 121.0, L)
    sess = FakeSession(_tiles_for(range(x0 - 1, x0 + 2), range(y0 - 1, y0 + 2)), levels=5)
    w = CesiumWorldTerrain(USER_TOKEN, session=sess)
    wb, sb, eb, nb = cwt.tile_bounds(L, x0, y0)
    args = (np.linspace(sb, nb, 12), np.linspace(wb, eb, 12))
    w.sample_grid(*args)
    n = len(sess.calls)
    w.sample_grid(*args)
    assert len(sess.calls) == n
    assert all(USER_TOKEN not in u and EP_TOKEN not in u for u, _ in sess.calls)
