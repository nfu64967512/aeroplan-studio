"""survey.qa：間距、側傾比例、足跡視數格網的數值。"""

import math

import numpy as np
import pytest

from survey import datum, qa
from survey.dflog import StripWindow

HQ6 = dict(focal_mm=6.0, pixel_um=1.55, width_px=4056, height_px=3040)
ACROSS = 4056 * 1.55e-3 / 6.0 * 120.0      # 125.73 m（影像寬 ⟂ 機頭）
ALONG = 3040 * 1.55e-3 / 6.0 * 120.0       # 94.24 m


def _north_events(n, spacing, t0_us, dt_us, lat0=25.1, lon0=121.2, rolls=None):
    m, _ = datum.wgs84_radii(lat0)
    lat = lat0 + np.degrees(np.arange(n) * spacing / m)
    return {"TimeUS": t0_us + np.arange(n) * dt_us, "Lat": lat, "Lng": np.full(n, lon0),
            "R": np.zeros(n) if rolls is None else np.asarray(rolls, float)}


def test_spacing_stats_and_roll_share():
    a = _north_events(20, 18.85, 1_000_000, 900_000, rolls=[25.0] + [0.0] * 18 + [12.0])
    b = _north_events(10, 18.85 * 1.25, 100_000_000, 900_000, lat0=25.11, rolls=[0.0] * 10)
    fc = {k: np.concatenate([a[k], b[k]]) for k in a}
    w = [StripWindow(1, 1_000_000, 50_000_000, 18.85, 3, 5), StripWindow(2, 100_000_000, 200_000_000, 18.85, 8, 9)]
    sp = qa.spacing_stats(fc, w)
    assert sp[0]["n_shots"] == 20 and sp[0]["n_intervals"] == 19
    assert sp[0]["median_m"] == pytest.approx(18.85, abs=1e-3) and sp[0]["median_ok"] and sp[0]["p99_ok"]
    assert sp[1]["median_m"] == pytest.approx(23.5625, abs=1e-3)
    assert sp[1]["median_ok"] is False and sp[1]["p99_ok"] is False      # 25% 過長
    r = qa.roll_share(fc, w, limit_deg=10.0)
    # 每條帶第一張（25°）不計；第 1 帶最後一張 12° 算
    assert (r["n"], r["n_gt"]) == (28, 1) and r["share"] == pytest.approx(1 / 28)


def _photo(lat, lon, yaw, h=150.0, roll=0.0, pitch=0.0):
    return {"lat_deg": lat, "lon_deg": lon, "yaw_deg": yaw, "roll_deg": roll, "pitch_deg": pitch,
            "h_orth_m": h, "h_ell_m": None}


def _extent(grid):
    ii, jj = np.nonzero(grid.counts)
    return (jj.max() - jj.min() + 1) * grid.cell_m, (ii.max() - ii.min() + 1) * grid.cell_m


def test_single_footprint_size_and_orientation():
    g = qa.footprint_multiplicity([_photo(25.1, 121.2, 0.0)], ground_h_m=30.0, height="orthometric",
                                  cell_m=1.0, **HQ6)
    assert g.n_footprints == 1 and g.counts.max() == 1
    assert g.counts.sum() == pytest.approx(ACROSS * ALONG, rel=0.02)
    ew, ns = _extent(g)
    assert ew == pytest.approx(ACROSS, abs=2.0) and ns == pytest.approx(ALONG, abs=2.0)
    g90 = qa.footprint_multiplicity([_photo(25.1, 121.2, 90.0)], ground_h_m=30.0, height="orthometric",
                                    cell_m=1.0, **HQ6)
    ew, ns = _extent(g90)
    assert ew == pytest.approx(ALONG, abs=2.0) and ns == pytest.approx(ACROSS, abs=2.0)


def test_overlap_counts_and_fraction():
    m, _ = datum.wgs84_radii(25.1)
    p2 = 25.1 + math.degrees(18.85 / m)
    g = qa.footprint_multiplicity([_photo(25.1, 121.2, 0.0), _photo(p2, 121.2, 0.0)], ground_h_m=30.0,
                                  height="orthometric", cell_m=1.0, **HQ6)
    assert (g.counts == 2).sum() == pytest.approx((ALONG - 18.85) * ACROSS, rel=0.03)
    assert g.fraction_at_least(2) == pytest.approx((ALONG - 18.85) / (ALONG + 18.85), rel=0.03)
    assert g.fraction_at_least(3) == 0.0


def test_attitude_shifts_footprint_and_bad_inputs():
    base = qa.footprint_multiplicity([_photo(25.1, 121.2, 0.0)], ground_h_m=30.0, height="orthometric",
                                     cell_m=1.0, origin=(25.0995, 121.199), **HQ6)
    rolled = qa.footprint_multiplicity([_photo(25.1, 121.2, 0.0, roll=10.0)], ground_h_m=30.0,
                                       height="orthometric", cell_m=1.0, origin=(25.0995, 121.199), **HQ6)
    _, nr = datum.wgs84_radii(25.1)

    def centroid_e(g):
        ii, jj = np.nonzero(g.counts)
        x0 = math.radians(g.lon0 - 121.199) * nr * math.cos(math.radians(25.0995))
        return x0 + (jj.mean() + 0.5) * g.cell_m
    # 右翼下 10°：光軸偏向左（機頭朝北 ⇒ 往西）AGL·tan10° = 21.16 m
    assert centroid_e(base) - centroid_e(rolled) == pytest.approx(120.0 * math.tan(math.radians(10.0)), abs=1.0)
    g = qa.footprint_multiplicity([_photo(25.1, 121.2, 0.0), dict(_photo(25.1, 121.2, 0.0), h_orth_m=None),
                                   _photo(25.1, 121.2, 0.0, h=20.0)], ground_h_m=30.0, height="orthometric", **HQ6)
    assert (g.n_footprints, g.n_skipped) == (1, 2)
    with pytest.raises(ValueError, match="height"):
        qa.footprint_multiplicity([], ground_h_m=0.0, height="msl", **HQ6)
    empty = qa.footprint_multiplicity([], ground_h_m=0.0, height="ellipsoidal", **HQ6)
    assert empty.n_footprints == 0 and empty.fraction_at_least(1) == 0.0
