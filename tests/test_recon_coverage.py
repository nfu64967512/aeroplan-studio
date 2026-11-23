"""core.recon 的感測器足跡與覆蓋率驗算測試。

原則：每條測試都要能在邏輯寫錯時**變紅**。所以除了「算對」的案例，
也刻意放進「故意掃不完」「故意重疊」「幾何無解」的案例 ——
若驗算器永遠回傳 100%，這些會立刻抓到。
"""

from __future__ import annotations

import math

import pytest

from core.recon import (
    CameraSpec,
    FootprintError,
    altitude_for_gsd_m,
    compute_footprint,
    effective_sweep_width_m,
    gsd_m_per_px,
    swath_width_m,
    verify_coverage,
)


# ──────────────────────────────────────────────────────────────
#  感測器足跡
# ──────────────────────────────────────────────────────────────

def test_nadir_swath_matches_closed_form():
    """正下視：W = 2·h·tan(HFOV/2)。"""
    cam = CameraSpec(hfov_deg=60.0, vfov_deg=45.0, image_width_px=4000,
                     mount_angle_deg=0.0)
    h = 120.0
    expected = 2.0 * h * math.tan(math.radians(60.0) / 2.0)
    assert swath_width_m(cam, h) == pytest.approx(expected, rel=1e-12)


def test_swath_scales_linearly_with_altitude():
    cam = CameraSpec()
    assert swath_width_m(cam, 200.0) == pytest.approx(2.0 * swath_width_m(cam, 100.0))


def test_tilt_widens_swath_via_slant_range():
    """傾斜會拉長斜距 ⇒ 掃描寬度變寬（除以 cos θ）。"""
    flat = CameraSpec(mount_angle_deg=0.0)
    tilt = CameraSpec(mount_angle_deg=30.0)
    ratio = swath_width_m(tilt, 100.0) / swath_width_m(flat, 100.0)
    assert ratio == pytest.approx(1.0 / math.cos(math.radians(30.0)), rel=1e-12)


def test_gsd_is_swath_over_pixels():
    cam = CameraSpec(hfov_deg=50.0, image_width_px=5000)
    h = 300.0
    assert gsd_m_per_px(cam, h) == pytest.approx(swath_width_m(cam, h) / 5000)


def test_altitude_for_gsd_roundtrips():
    """由 GSD 反算航高，再算回 GSD 要一致 —— 反解沒寫反的證據。"""
    cam = CameraSpec(hfov_deg=55.0, image_width_px=6000, mount_angle_deg=15.0)
    target = 0.05
    h = altitude_for_gsd_m(cam, target)
    assert gsd_m_per_px(cam, h) == pytest.approx(target, rel=1e-10)
    assert h > 0


def test_finer_gsd_needs_lower_altitude():
    """要看得更細就得飛更低 —— 方向寫反的話這條會紅。"""
    cam = CameraSpec()
    assert altitude_for_gsd_m(cam, 0.02) < altitude_for_gsd_m(cam, 0.10)


def test_worst_gsd_is_never_better_than_centre():
    tilt = CameraSpec(mount_angle_deg=25.0, vfov_deg=40.0)
    fp = compute_footprint(tilt, 150.0)
    assert fp.worst_gsd_m_per_px >= fp.gsd_m_per_px
    # 正下視時兩者仍有差（邊緣斜距較遠），但傾斜時差距必須更大
    flat = compute_footprint(CameraSpec(mount_angle_deg=0.0, vfov_deg=40.0), 150.0)
    assert (fp.worst_gsd_m_per_px / fp.gsd_m_per_px) > (
        flat.worst_gsd_m_per_px / flat.gsd_m_per_px
    )


def test_horizon_geometry_raises_instead_of_returning_garbage():
    """視線角逼近 90° 必須丟錯，不可回傳一個發散的大數字。"""
    cam = CameraSpec(mount_angle_deg=70.0, vfov_deg=40.0)   # 70 + 20 = 90
    with pytest.raises(FootprintError, match="地平線"):
        compute_footprint(cam, 100.0)
    with pytest.raises(FootprintError):
        swath_width_m(cam, 100.0)


@pytest.mark.parametrize("bad", [0.0, -10.0])
def test_non_positive_altitude_rejected(bad):
    with pytest.raises(FootprintError):
        swath_width_m(CameraSpec(), bad)


@pytest.mark.parametrize("kwargs", [
    {"hfov_deg": 0.0}, {"hfov_deg": 180.0}, {"vfov_deg": -5.0},
    {"image_width_px": 0}, {"mount_angle_deg": 90.0},
])
def test_invalid_camera_spec_rejected(kwargs):
    with pytest.raises(FootprintError):
        CameraSpec(**kwargs)


def test_effective_sweep_applies_overlap():
    assert effective_sweep_width_m(100.0, 0.0) == pytest.approx(100.0)
    assert effective_sweep_width_m(100.0, 0.3) == pytest.approx(70.0)


@pytest.mark.parametrize("bad", [-0.1, 1.0, 1.5])
def test_invalid_overlap_rejected(bad):
    with pytest.raises(FootprintError):
        effective_sweep_width_m(100.0, bad)


# ──────────────────────────────────────────────────────────────
#  覆蓋率驗算
# ──────────────────────────────────────────────────────────────

# 約 200 m × 200 m 的方形區（緯度 25° 附近）
_D = 0.0009           # ~100 m
SQUARE = [(25.0 - _D, 121.0 - _D), (25.0 - _D, 121.0 + _D),
          (25.0 + _D, 121.0 + _D), (25.0 + _D, 121.0 - _D)]


def _lawnmower(lat_lo, lat_hi, lon_lo, lon_hi, n_lines):
    """在區域內產生 n_lines 條南北向掃描線。"""
    path = []
    for i in range(n_lines):
        lon = lon_lo + (lon_hi - lon_lo) * (i + 0.5) / n_lines
        pts = [(lat_lo, lon), (lat_hi, lon)]
        path.extend(pts if i % 2 == 0 else pts[::-1])
    return path


def test_dense_lawnmower_covers_everything():
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    rep = verify_coverage({1: path}, SQUARE, sweep_width_m=30.0, cell_size_m=10.0)
    assert rep.total_cells > 100
    assert rep.coverage_ratio > 0.99, rep.summary()


def test_sparse_lawnmower_leaves_gaps():
    """條帶太疏就該掃不完 —— 驗算器若永遠回 100%，這條會紅。"""
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=3)
    rep = verify_coverage({1: path}, SQUARE, sweep_width_m=10.0, cell_size_m=10.0)
    assert rep.coverage_ratio < 0.6, rep.summary()
    assert rep.missed_points, "有漏掃卻沒回報座標"


def test_empty_path_covers_nothing():
    rep = verify_coverage({1: []}, SQUARE, sweep_width_m=50.0, cell_size_m=10.0)
    assert rep.covered_cells == 0
    assert rep.coverage_ratio == 0.0
    assert rep.per_uav_cells == {1: 0}


def test_two_uavs_splitting_area_beats_one_of_them():
    """兩架分工的覆蓋率要高於任一架單獨飛。"""
    left = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0, n_lines=6)
    right = _lawnmower(25.0 - _D, 25.0 + _D, 121.0, 121.0 + _D, n_lines=6)
    both = verify_coverage({1: left, 2: right}, SQUARE, 30.0, cell_size_m=10.0)
    only = verify_coverage({1: left}, SQUARE, 30.0, cell_size_m=10.0)
    assert both.coverage_ratio > only.coverage_ratio
    assert both.per_uav_cells[1] > 0 and both.per_uav_cells[2] > 0


def test_overlap_histogram_detects_double_scanning():
    """兩架飛同一條航跡 ⇒ 幾乎每格都被掃 2 次，重掃比例應該很高。"""
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    rep = verify_coverage({1: path, 2: path}, SQUARE, 30.0, cell_size_m=10.0)
    assert rep.redundant_ratio > 0.95, rep.overlap_histogram
    assert 2 in rep.overlap_histogram


def test_single_uav_has_no_multi_aircraft_overlap():
    """單機 ⇒ `redundant_ratio`（多**機**重疊）為 0。

    ⚠⚠⚠ 這條測試**不是**在說「單機不會跑重複路徑」。`redundant_ratio` 以
    「不同 UAV」計數，所以單機情形下它在**數學上**恆為 0，量不到任何東西。
    舊版本把這件事寫成 `test_no_overlap_when_single_uav` ⇒ 把「量不到」寫成了
    「應該是 0」。真正的重複路徑由 `repeat_ratio` / `overcoverage_x` 負責，
    見下面兩條。
    """
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    rep = verify_coverage({1: path}, SQUARE, 30.0, cell_size_m=10.0)
    assert rep.redundant_ratio == 0.0
    assert set(rep.overlap_histogram) == {1}
    # 同一份資料，新指標必須看得見這是「一趟」而不是「零趟」
    assert rep.overcoverage_x > 0.0
    assert set(rep.pass_histogram) != set()


def test_one_uav_flying_the_same_path_three_times_is_reported_as_three():
    """⭐ 單機把**同一條**航跡飛三遍 ⇒ 過度覆蓋倍率必須 ≈3，不是 0。

    這正是使用者要的「有沒有跑重複的路徑」。舊指標在這個輸入上回 0.0000、
    直方圖 {1: 360} —— 那不是「沒有重複」，是**量不到**。
    """
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    once = verify_coverage({1: path}, SQUARE, 30.0, cell_size_m=10.0)
    thrice = verify_coverage({1: path + path + path}, SQUARE, 30.0, cell_size_m=10.0)

    # 前置斷言：兩者覆蓋的地一模一樣 ⇒ 差別只在「掃了幾趟」
    assert thrice.covered_cells == once.covered_cells

    assert thrice.overcoverage_x > 2.7 * once.overcoverage_x, (
        once.overcoverage_x, thrice.overcoverage_x, thrice.pass_histogram)
    assert thrice.repeat_ratio > 0.9, thrice.pass_histogram
    # 而舊指標在同一份資料上仍然是 0 —— 保留這個對比，別讓人再誤用它
    assert thrice.redundant_ratio == 0.0


def test_a_single_straight_pass_is_exactly_one_times():
    """反向臂：真正只掃一趟的航跡，倍率必須**恰好** 1.0。

    少了這一條，上面那條「三遍 ⇒ 3×」就可能只是在量雜訊。
    """
    lat, lo, hi = 25.0, 121.0 - _D, 121.0 + _D
    rep = verify_coverage({1: [(lat, lo + (hi - lo) * i / 40.0) for i in range(41)]},
                          SQUARE, 30.0, cell_size_m=10.0)
    assert rep.covered_cells > 0
    assert rep.overcoverage_x == 1.0, (rep.overcoverage_x, rep.pass_histogram)
    assert rep.repeat_ratio == 0.0


def test_adjacent_lawnmower_lines_are_seen_as_over_coverage():
    """⭐ 側向重疊過大也是「重複」：12 線 × 刈幅 30 m 的相鄰航線互相蓋到。

    實測 1.817×（294/360 格被掃 2 趟）。舊指標對這件事回 0.0%，因為它只數飛機。
    這條同時是「新指標看得見側向過度重疊」的證據。
    """
    dense = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    sparse = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=4)
    r_dense = verify_coverage({1: dense}, SQUARE, 30.0, cell_size_m=10.0)
    r_sparse = verify_coverage({1: sparse}, SQUARE, 30.0, cell_size_m=10.0)
    assert r_dense.overcoverage_x > r_sparse.overcoverage_x, (
        r_dense.overcoverage_x, r_sparse.overcoverage_x)
    assert r_dense.overcoverage_x > 1.5, r_dense.pass_histogram
    assert r_dense.redundant_ratio == r_sparse.redundant_ratio == 0.0   # 舊指標全瞎


def test_out_and_back_on_the_same_line_counts_as_two_passes():
    """去一趟回一趟 ⇒ 2 趟。判準是『離開再回來』，掉頭會跨過許多航段。"""
    lat, lo, hi = 25.0, 121.0 - _D, 121.0 + _D
    fwd = [(lat, lo + (hi - lo) * i / 40.0) for i in range(41)]
    rep = verify_coverage({1: fwd + list(reversed(fwd))}, SQUARE, 30.0, cell_size_m=10.0)
    assert max(rep.pass_histogram) >= 2, rep.pass_histogram
    assert rep.repeat_ratio > 0.9, rep.pass_histogram


def test_wider_sweep_covers_more():
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=4)
    narrow = verify_coverage({1: path}, SQUARE, 10.0, cell_size_m=10.0)
    wide = verify_coverage({1: path}, SQUARE, 60.0, cell_size_m=10.0)
    assert wide.coverage_ratio > narrow.coverage_ratio


def test_concave_polygon_excludes_the_notch():
    """凹多邊形的凹口不能被算進總格數（ray-casting 寫錯就會多算）。"""
    notched = [(25.0, 121.0), (25.0, 121.002), (25.001, 121.002),
               (25.001, 121.0012), (25.0004, 121.0012), (25.0004, 121.0),
               ]
    full = [(25.0, 121.0), (25.0, 121.002), (25.001, 121.002), (25.001, 121.0)]
    rep_notch = verify_coverage({1: []}, notched, 30.0, cell_size_m=10.0)
    rep_full = verify_coverage({1: []}, full, 30.0, cell_size_m=10.0)
    assert rep_notch.total_cells < rep_full.total_cells


def test_missed_points_are_truncated_and_flagged():
    rep = verify_coverage({1: []}, SQUARE, 30.0, cell_size_m=2.0,
                          max_missed_points=50)
    assert rep.missed_truncated is True
    assert len(rep.missed_points) <= 51


def test_cell_budget_is_enforced_loudly():
    """格距太小要明確報錯，不可默默跑到卡死。"""
    with pytest.raises(ValueError, match="上限"):
        verify_coverage({1: []}, SQUARE, 30.0, cell_size_m=0.01)


@pytest.mark.parametrize("kwargs,match", [
    ({"sweep_width_m": 0.0}, "掃描寬度"),
    ({"cell_size_m": 0.0}, "格距"),
])
def test_invalid_verify_args_rejected(kwargs, match):
    args = {"sweep_width_m": 30.0, "cell_size_m": 10.0}
    args.update(kwargs)
    with pytest.raises(ValueError, match=match):
        verify_coverage({1: []}, SQUARE, **args)


def test_degenerate_polygon_rejected():
    with pytest.raises(ValueError, match="3 個點"):
        verify_coverage({1: []}, [(25.0, 121.0), (25.0, 121.001)], 30.0)


# ──────────────────────────────────────────────────────────────
#  熱力圖顯示格網
# ──────────────────────────────────────────────────────────────

def test_display_grid_shape_and_bounds():
    rep = verify_coverage({1: []}, SQUARE, 30.0, cell_size_m=10.0)
    g = rep.to_display_grid(max_side=60)
    assert g is not None
    assert g["rows"] > 0 and g["cols"] > 0
    assert len(g["values"]) == g["rows"] * g["cols"]
    assert g["lat_min"] < g["lat_max"] and g["lon_min"] < g["lon_max"]
    # 顯示格網要涵蓋整個偵察區
    assert g["lat_min"] <= min(p[0] for p in SQUARE) + 1e-6
    assert g["lat_max"] >= max(p[0] for p in SQUARE) - 1e-6


def test_display_grid_is_capped_so_the_browser_survives():
    """細格可以幾十萬個，但送去畫的絕不能。"""
    rep = verify_coverage({1: []}, SQUARE, 30.0, cell_size_m=1.0)
    assert rep.total_cells > 20000, "這個案例本來就該產生很多細格"
    g = rep.to_display_grid(max_side=40)
    assert g["rows"] <= 40 and g["cols"] <= 40
    assert len(g["values"]) <= 1600


def test_display_grid_marks_uncovered_as_zero_and_covered_as_one():
    """全漏掃 → 值全 0；全覆蓋 → 值全 1。若聚合寫反這條會紅。"""
    empty = verify_coverage({1: []}, SQUARE, 30.0, cell_size_m=10.0)
    vals = [v for v in empty.to_display_grid()["values"] if v is not None]
    assert vals and all(v == 0.0 for v in vals)

    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    full = verify_coverage({1: path}, SQUARE, 60.0, cell_size_m=10.0)
    assert full.coverage_ratio == pytest.approx(1.0)
    vals = [v for v in full.to_display_grid()["values"] if v is not None]
    assert vals and all(v == pytest.approx(1.0) for v in vals)


def test_display_grid_mean_matches_overall_coverage():
    """顯示格的平均值應該貼近整體覆蓋率 —— 聚合沒有把資料弄丟的證據。"""
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=4)
    rep = verify_coverage({1: path}, SQUARE, 25.0, cell_size_m=5.0)
    g = rep.to_display_grid(max_side=50)
    vals = [v for v in g["values"] if v is not None]
    assert abs(sum(vals) / len(vals) - rep.coverage_ratio) < 0.05, (
        f"聚合平均 {sum(vals)/len(vals):.3f} vs 實際 {rep.coverage_ratio:.3f}"
    )


def test_display_grid_uses_none_outside_the_polygon():
    """凹口/區外的顯示格要是 None，不能被畫成「漏掃」的紅塊。"""
    notched = [(25.0, 121.0), (25.0, 121.002), (25.001, 121.002),
               (25.001, 121.0012), (25.0004, 121.0012), (25.0004, 121.0)]
    rep = verify_coverage({1: []}, notched, 30.0, cell_size_m=5.0)
    g = rep.to_display_grid(max_side=40)
    assert any(v is None for v in g["values"]), "凹口沒有被標成區外"
    assert any(v is not None for v in g["values"])


def test_display_grid_is_json_serialisable():
    """要經 JSON 送進 Cesium，numpy 型別會炸掉。"""
    import json
    rep = verify_coverage({1: []}, SQUARE, 30.0, cell_size_m=10.0)
    s = json.dumps(rep.to_display_grid())
    assert '"rows"' in s and '"values"' in s


def test_summary_reports_real_numbers():
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    rep = verify_coverage({1: path}, SQUARE, 30.0, cell_size_m=10.0)
    s = rep.summary()
    assert "覆蓋率" in s and "%" in s
    assert f"{rep.covered_cells}/{rep.total_cells}" in s


# ──────────────────────────────────────────────────────────────
#  路徑中的 None = 斷點（不相連的成像段不連線）
#  2026-10-09：HARVEST 縮點後每條條帶只剩起點帶 sensor_on，GUI 只取這些「成像航點」並連成一條
#  折線 ⇒ 量的是一條跨過整區的鋸齒線，不是飛出去的條帶（同一份計畫 GUI 75.06%、實際 100.00%）。
# ──────────────────────────────────────────────────────────────

# 細長區（±300 m × ±1000 m）：條帶長 2 km、間距 100 m ⇒ 起點之間的斜線蓋不到條帶（貼近 HARVEST 的 1.6 km 條帶）
_LAT0, _LON0, _DLAT, _DLON = 25.0, 121.0, 0.0027, 0.0105
LONG = [(_LAT0 - _DLAT, _LON0 - _DLON), (_LAT0 - _DLAT, _LON0 + _DLON),
        (_LAT0 + _DLAT, _LON0 + _DLON), (_LAT0 + _DLAT, _LON0 - _DLON)]
_SW = 110.0          # 感測器刈幅；6 條帶間距 100 m ⇒ 剛好鋪滿


def _strips(n_lines=6):
    """n 條東西向條帶，各自一個 (起點, 終點) 對（來回掃，不含條帶之間的連接腿）。"""
    out = []
    for i in range(n_lines):
        lat = _LAT0 - _DLAT + 2 * _DLAT * (i + 0.5) / n_lines
        w, e = (lat, _LON0 - _DLON), (lat, _LON0 + _DLON)
        out.append([w, e] if i % 2 == 0 else [e, w])
    return out


def _with_breaks(strips):
    path = []
    for k, st in enumerate(strips):
        if k:
            path.append(None)
        path.extend(st)
    return path


def _cov(path, **kw):
    return verify_coverage({1: path}, LONG, kw.pop("sw", _SW), cell_size_m=10.0, **kw)


def test_none_break_keeps_separate_strips_separate():
    """⭐ 同樣的 6 條帶：用 None 隔開 ⇒ 100%；只留條帶起點再連起來（HARVEST 縮點後 GUI 的舊量法）⇒ 掉到 7 成。

    前置：舊量法真的會掉（否則這條量不到斷點的作用）。
    """
    strips = _strips()
    starts_glued = _cov([st[0] for st in strips])              # 舊：只取起點、連成一條折線
    broken = _cov(_with_breaks(strips))                        # 新：每條帶一段
    fully_glued = _cov([p for st in strips for p in st])      # 舊呼叫端的行為（連續折線）

    assert starts_glued.coverage_pct < 80.0, starts_glued.coverage_pct
    assert broken.coverage_pct == pytest.approx(100.0, abs=1e-9)
    assert fully_glued.coverage_pct == pytest.approx(100.0, abs=1e-9)


def test_none_break_does_not_credit_the_gap_between_strips():
    """最遠的兩條帶：連起來會被連接線蓋到東邊緣一整條，斷開就不會。"""
    st = _strips()
    left, right = st[0], st[-1]
    joined = _cov(left + right)
    broken = _cov(left + [None] + right)
    assert broken.covered_cells < joined.covered_cells, (broken.covered_cells, joined.covered_cells)
    # 斷開版 = 兩條各自獨立的聯集（用兩架各飛一條的結果交叉驗證）
    both = verify_coverage({1: left, 2: right}, LONG, _SW, cell_size_m=10.0)
    assert broken.covered_cells == both.covered_cells


def test_each_broken_run_counts_as_its_own_pass():
    """同一架飛兩段**重疊**的成像：斷開 ⇒ 2 趟；不斷開（連續）⇒ 1 趟。"""
    seg = [(25.0 - _D, 121.0), (25.0 + _D, 121.0)]
    rep_two = verify_coverage({1: seg + [None] + seg}, SQUARE, 30.0, cell_size_m=10.0)
    rep_one = verify_coverage({1: seg}, SQUARE, 30.0, cell_size_m=10.0)
    assert rep_two.covered_cells == rep_one.covered_cells
    assert rep_two.overcoverage_x == pytest.approx(2.0 * rep_one.overcoverage_x)
    assert rep_two.repeat_ratio > 0.99 and rep_one.repeat_ratio == 0.0


def test_breaks_at_the_edges_and_doubled_breaks_are_harmless():
    strips = _strips()
    clean = _cov(_with_breaks(strips))
    messy = _cov([None, None] + _with_breaks(strips) + [None])
    doubled = _cov([p for st in strips for p in st + [None, None]])
    assert messy.covered_cells == clean.covered_cells == doubled.covered_cells
    assert messy.pass_histogram == clean.pass_histogram
    only_none = verify_coverage({1: [None, None]}, SQUARE, 30.0, cell_size_m=10.0)
    assert only_none.covered_cells == 0 and only_none.per_uav_cells == {1: 0}


def test_a_path_without_none_is_unchanged():
    """沒有 None ⇒ 只有一段 ⇒ 與舊行為逐位相同（舊呼叫端不受影響）。"""
    path = _lawnmower(25.0 - _D, 25.0 + _D, 121.0 - _D, 121.0 + _D, n_lines=12)
    a = verify_coverage({1: path}, SQUARE, 30.0, cell_size_m=10.0)
    assert a.covered_cells == a.total_cells
    assert a.pass_histogram == verify_coverage({1: list(path)}, SQUARE, 30.0, cell_size_m=10.0).pass_histogram
