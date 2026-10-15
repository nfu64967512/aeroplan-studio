"""survey.gcp_plan：GCP／檢核點佈設 —— 值層級的鎖。

幾何真值用 **UTM 51N（EPSG:32651）** 建（pyproj），距離也在 UTM 上量（此處尺度誤差 ~5 ppm），
與 gcp_plan 內部的橫麥卡托互相獨立。作業區 1200 × 800 m、三個 UAV 區塊（A1／B2／B3 縱向三等分，
相鄰兩塊重疊 88 m ≈ 0.7 × 125.7 m 的接縫），足跡 125.7 × 94.2 m（HQ6 @120 m）。
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
import pytest
from pyproj import CRS, Transformer
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from survey import gcp_plan as gp
from survey.gcp_plan import GcpPlanError, LayoutParams

UTM = "EPSG:32651"
TO_UTM = Transformer.from_crs("EPSG:4326", UTM, always_xy=True)
TO_LL = Transformer.from_crs(UTM, "EPSG:4326", always_xy=True)
E0, N0 = TO_UTM.transform(121.21, 25.07)
W, H = 1200.0, 800.0
INSET = 125.7 / 2.0                       # max(30, w/2)


def ll(x, y):
    lon, lat = TO_LL.transform(E0 + x, N0 + y)
    return (lat, lon)


def xy(lat, lon):
    e, n = TO_UTM.transform(lon, lat)
    return e - E0, n - N0


def rect(x0, x1, y0, y1):
    return [ll(x0, y0), ll(x1, y0), ll(x1, y1), ll(x0, y1)]


AREA = rect(-W / 2, W / 2, -H / 2, H / 2)
BLOCK_X = {"A1": (-660.0, -156.0), "B2": (-244.0, 244.0), "B3": (156.0, 660.0)}
BLOCKS = {k: [rect(a, b, -H / 2 - 60, H / 2 + 60)] for k, (a, b) in BLOCK_X.items()}
AREA_POLY = Polygon([(-W / 2, -H / 2), (W / 2, -H / 2), (W / 2, H / 2), (-W / 2, H / 2)])
BLOCK_POLY = {k: Polygon([(a, -H / 2), (b, -H / 2), (b, H / 2), (a, H / 2)]) for k, (a, b) in BLOCK_X.items()}


def pxy(p):
    return np.asarray(xy(p.lat, p.lon))


@pytest.fixture(scope="module")
def cross():
    return gp.plan_layout([AREA], BLOCKS, LayoutParams(preset="cross_strips"))


@pytest.fixture(scope="module")
def dense():
    return gp.plan_layout([AREA], BLOCKS, LayoutParams(preset="dense_gcp"))


# ─────────────────────────────────────────────────────────────────────────────
# 規則
# ─────────────────────────────────────────────────────────────────────────────

def test_preset_counts_cross_is_5_plus_one_per_block_dense_is_9_plus_blocks_plus_seams(cross, dense):
    assert cross.counts["GCP"] == 5 + 3
    assert cross.counts["GCP_by_purpose"] == {"block": 3, "centre": 1, "corner": 4}
    assert dense.counts["GCP"] == 9 + 3 + 2
    assert dense.counts["GCP_by_purpose"] == {"block": 3, "centre": 1, "corner": 4, "edge_mid": 4, "seam": 2}
    for plan in (cross, dense):
        assert plan.counts["CHK"] == 20 and plan.counts["seams"] == 2 and plan.counts["seam_pairs"] == 2
        assert not plan.warnings, plan.warnings
    assert gp.compare_presets([AREA], BLOCKS)["dense_gcp"]["GCP"] == 14


def test_corners_and_centre_are_inset_outside_the_low_view_edge_band(dense):
    assert dense.effective["inset_m"] == pytest.approx(INSET) and dense.effective["edge_band_m"] == pytest.approx(INSET)
    corners = [pxy(p) for p in dense.gcps() if p.purpose == "corner"]
    exp = [np.array([sx * (W / 2 - INSET), sy * (H / 2 - INSET)]) for sx in (-1, 1) for sy in (-1, 1)]
    for c in exp:
        assert min(np.hypot(*(c - q)) for q in corners) < 0.3        # 角落 GCP 正好在內縮矩形的角
    centre = next(pxy(p) for p in dense.gcps() if p.purpose == "centre")
    assert np.hypot(*centre) < 0.3
    mids = sorted((round(float(q[0])), round(float(q[1]))) for q in (pxy(p) for p in dense.gcps()
                                                                       if p.purpose == "edge_mid"))
    assert mids == sorted([(0, round(H / 2 - INSET)), (0, -round(H / 2 - INSET)),
                           (round(W / 2 - INSET), 0), (-round(W / 2 - INSET), 0)])
    for p in dense.points:                                           # 每個點都在邊緣帶之外
        assert AREA_POLY.exterior.distance(Point(*pxy(p))) >= INSET - 0.3, p.id


def test_every_uav_block_has_its_own_gcp_inside(cross):
    for k, poly in BLOCK_POLY.items():
        own = [p for p in cross.gcps() if p.purpose == "block" and p.note.endswith(f"{k} 的控制點")]
        assert len(own) == 1
        q = Point(*pxy(own[0]))
        assert poly.buffer(0.5).contains(q)
        others = [BLOCK_POLY[o] for o in BLOCK_POLY if o != k]
        assert not any(o.contains(q) for o in others), "區塊 GCP 放在只有這架飛的地方"
        assert k in own[0].block.split("|")


def test_gcp_spacing_and_checkpoint_separation_follow_the_footprint(cross, dense):
    for plan in (cross, dense):
        g = np.asarray([pxy(p) for p in plan.gcps()])
        c = np.asarray([pxy(p) for p in plan.checkpoints()])
        dg = np.hypot(g[:, None, 0] - g[None, :, 0], g[:, None, 1] - g[None, :, 1])
        dg[np.diag_indices(len(g))] = np.inf
        assert dg.min() >= 125.7 - 0.5                               # max(w, l)
        dc = np.hypot(c[:, None, 0] - g[None, :, 0], c[:, None, 1] - g[None, :, 1])
        assert dc.min() >= 94.2 - 0.5                                # 一個足跡長
        assert all(ok for name, ok, _ in plan.checks if not name.startswith("ASPRS")), plan.checks


def test_each_seam_gets_a_checkpoint_pair_straddling_it_at_the_offset(cross):
    pairs = {}
    for p in cross.checkpoints():
        if p.purpose == "seam_pair":
            pairs.setdefault(p.seam, []).append(p)
    assert sorted(pairs) == ["A1|B2", "B2|B3"]
    for seam, (pa, pb) in pairs.items():
        a, b = seam.split("|")
        qa, qb = Point(*pxy(pa)), Point(*pxy(pb))
        assert BLOCK_POLY[a].buffer(0.5).contains(qa) and not BLOCK_POLY[b].contains(qa)
        assert BLOCK_POLY[b].buffer(0.5).contains(qb) and not BLOCK_POLY[a].contains(qb)
        assert BLOCK_POLY[b].distance(qa) >= INSET - 0.5 and BLOCK_POLY[a].distance(qb) >= INSET - 0.5
        # 離接縫最近的合格位置：x 離「對方區塊 + w/2」不到一個候選格距
        step = cross.effective["candidate_step_m"]
        if seam == "A1|B2":
            assert -244 - INSET - 1.5 * step <= pxy(pa)[0] <= -244 - INSET + 0.5
            assert -156 + INSET - 0.5 <= pxy(pb)[0] <= -156 + INSET + 1.5 * step
        assert pa.block == a and pb.block == b
    rec = {"|".join(s["blocks"]): s for s in cross.seams}
    assert rec["A1|B2"]["kind"] == "overlap" and rec["A1|B2"]["width_m"] == pytest.approx(88.0, abs=0.5)
    assert rec["A1|B2"]["length_m"] == pytest.approx(800.0, abs=0.5)
    assert "A1|B3" not in rec


def test_stratified_checkpoints_cover_every_quadrant(cross):
    c = np.asarray([pxy(p) for p in cross.checkpoints()])
    quad = {(bool(x > 0), bool(y > 0)) for x, y in c}
    assert len(quad) == 4
    for sx in (False, True):
        for sy in (False, True):
            assert sum((x > 0) == sx and (y > 0) == sy for x, y in c) >= 3


def test_dense_puts_a_gcp_inside_each_seam_overlap(dense):
    seam_g = [pxy(p) for p in dense.gcps() if p.purpose == "seam"]
    ov = {"A1|B2": BLOCK_POLY["A1"].intersection(BLOCK_POLY["B2"]),
          "B2|B3": BLOCK_POLY["B2"].intersection(BLOCK_POLY["B3"])}
    for k, poly in ov.items():
        assert sum(poly.buffer(0.5).contains(Point(*q)) for q in seam_g) == 1, k
    assert {s["gcp"] for s in dense.seams} == {p.id for p in dense.gcps() if p.purpose == "seam"}


def test_layout_is_deterministic_for_a_seed_and_changes_with_it():
    a = gp.layout_csv_text(gp.plan_layout([AREA], BLOCKS, LayoutParams(seed=3)))
    b = gp.layout_csv_text(gp.plan_layout([AREA], BLOCKS, LayoutParams(seed=3)))
    c = gp.plan_layout([AREA], BLOCKS, LayoutParams(seed=4))
    assert a == b
    assert a != gp.layout_csv_text(c)
    base = gp.plan_layout([AREA], BLOCKS, LayoutParams(seed=3))
    assert [(p.id, p.lat, p.lon) for p in base.gcps() if p.purpose in ("corner", "centre")] == \
           [(p.id, p.lat, p.lon) for p in c.gcps() if p.purpose in ("corner", "centre")]
    # 種子也決定候選點的抖動 ⇒ 「離既有 GCP 最遠」挑出來的區塊 GCP 會跟著變（不是只有分層檢核點在變）
    assert [(p.lat, p.lon) for p in base.gcps() if p.purpose == "block"] != \
           [(p.lat, p.lon) for p in c.gcps() if p.purpose == "block"]


def test_asprs_30_and_without_blocks():
    p30 = gp.plan_layout([AREA], BLOCKS, LayoutParams(n_checkpoints=30))
    assert p30.counts["CHK"] == 30 and dict((n, ok) for n, ok, _ in p30.checks)[
        "ASPRS 第 2 版：檢核點 ≥ 30（否則附小專案聲明）"] is True
    bare = gp.plan_layout([AREA], None, LayoutParams())
    assert bare.counts["GCP"] == 5 and bare.counts["seams"] == 0 and bare.counts["CHK"] == 20
    assert all(p.block == "" for p in bare.points)
    assert any("沒有 UAV 區塊資訊" in n for n in bare.notes)
    # 1200 × 800 m = 0.96 km² < 1 km²：不是大案；放大到 1.2 km² 就該警告 ODM 8–10
    big = gp.plan_layout([rect(-700, 700, -450, 450)], None, LayoutParams())
    assert big.effective["is_large_area"] is True and big.counts["GCP"] == 5
    assert any("ODM 建議 8–10" in w and "交叉航帶方案刻意少 GCP" in w for w in big.warnings)


def test_user_values_win_and_are_reported_with_their_source():
    p = gp.plan_layout([AREA], BLOCKS, LayoutParams(edge_band_m=20.0, gcp_min_spacing_m=150.0, chk_gcp_min_m=60.0,
                                                    per_block=False, seam_pair_offset_m=30.0))
    e = p.effective
    assert (e["edge_band_m"], e["inset_m"], e["gcp_min_spacing_m"], e["chk_gcp_min_m"]) == (20.0, 30.0, 150.0, 60.0)
    assert e["sources"]["edge_band_m"] == "使用者" and e["sources"]["per_block"] == "使用者"
    assert e["sources"]["chk_inset_m"].startswith("預設")
    assert p.counts["GCP_by_purpose"].get("block", 0) == 0 and p.counts["GCP"] == 5
    pairs = [q for q in p.checkpoints() if q.seam == "A1|B2"]
    assert max(pxy(q)[0] for q in pairs if q.block == "A1") >= -244 - 30 - 1.5 * e["candidate_step_m"]
    corner = min((pxy(q) for q in p.gcps() if q.purpose == "corner"), key=lambda v: v[0] + v[1])
    assert corner == pytest.approx([-W / 2 + 30, -H / 2 + 30], abs=0.3)


def test_small_area_warns_and_reports_effective_counts_instead_of_silently_relaxing():
    p = gp.plan_layout([rect(-200, 200, -150, 150)], None, LayoutParams(preset="dense_gcp"))
    assert p.counts["GCP"] == 8                                      # 方案下限 8 照樣補齊
    assert any("未放" in w for w in p.warnings)                      # 邊中點／中央太近而沒放
    assert any("< 間距" in w for w in p.warnings)                    # 補的點也只能違反間距 ⇒ 說出來
    assert dict((n.split("（")[0], ok) for n, ok, _ in p.checks)["GCP 間距 ≥ 125.7 m"] is False


def test_seam_pairs_beyond_the_requested_count_raise_it_loudly():
    p = gp.plan_layout([AREA], BLOCKS, LayoutParams(n_checkpoints=3))
    assert p.counts["CHK"] == 4 and p.counts["CHK_by_purpose"] == {"seam_pair": 4}
    assert any("接縫規則優先" in w for w in p.warnings)


def test_split_submodels_each_get_three_gcps():
    p = gp.plan_layout([AREA], None, LayoutParams(split_submodels=3))
    cent = [np.asarray(xy(s["centre_lat"], s["centre_lon"])) for s in p.effective["split"]]
    g = [pxy(q) for q in p.gcps()]
    lab = [int(np.argmin([np.hypot(*(q - c)) for c in cent])) for q in g]
    assert sorted(set(lab)) == [0, 1, 2]
    assert all(lab.count(j) >= 3 for j in range(3)), lab
    assert p.counts["GCP_by_purpose"].get("split", 0) >= 1
    assert any("以作業區面積的 k-means 近似" in n for n in p.notes)


def test_bad_inputs_are_refused_in_chinese():
    with pytest.raises(GcpPlanError, match="作業區太小"):
        gp.plan_layout([rect(-50, 50, -40, 40)], None)
    with pytest.raises(GcpPlanError, match="方案"):
        gp.plan_layout([AREA], None, LayoutParams(preset="x"))
    with pytest.raises(GcpPlanError, match="自相交"):
        gp.plan_layout([[ll(0, 0), ll(500, 500), ll(500, 0), ll(0, 500)]], None)
    with pytest.raises(GcpPlanError, match="至少要 3"):
        gp.plan_layout([[ll(0, 0), ll(1, 1)]], None)
    with pytest.raises(GcpPlanError, match="split_submodels = 1"):
        gp.plan_layout([AREA], None, LayoutParams(split_submodels=1))
    with pytest.raises(GcpPlanError, match="n_checkpoints"):
        gp.plan_layout([AREA], None, LayoutParams(n_checkpoints=-1))
    p = gp.plan_layout([AREA], {"Z9": [rect(5000, 5600, 0, 100)]}, LayoutParams())
    assert any("Z9 與作業區沒有交集" in w for w in p.warnings) and p.effective["blocks"] == []


def test_blocks_that_do_not_cover_the_area_are_flagged():
    p = gp.plan_layout([AREA], {"A1": BLOCKS["A1"]}, LayoutParams())
    assert any("只蓋到作業區的" in w for w in p.warnings)


def test_edge_gaps_narrower_than_half_a_footprint_are_filled_not_flagged():
    """真的 HARVEST 規劃：條帶帶寬只到最外條帶 ± s/2，外緣 25 m 不在任何帶內，但影像（± w/2）有拍到。"""
    gap = {k: [rect(a, b, -H / 2 + 25, H / 2 - 25)] for k, (a, b) in
           {"A1": (-W / 2 + 25, -156.0), "B2": (-244.0, 244.0), "B3": (156.0, W / 2 - 25)}.items()}
    p = gp.plan_layout([AREA], gap, LayoutParams())
    assert not any("只蓋到" in w for w in p.warnings), p.warnings
    assert p.effective["blocks_coverage"] == pytest.approx(1.0)
    assert p.effective["blocks_band_coverage"] == pytest.approx((W - 50) * (H - 50) / (W * H), rel=1e-3)
    assert p.effective["blocks_gap_filled_m2"] == pytest.approx(W * H - (W - 50) * (H - 50), rel=1e-3)
    assert all(p_.block for p_ in p.points)                           # 每個點都有所屬區塊
    filled = unary_union([Polygon([xy(*q) for q in r]) for r in p.block_rings["A1"]])
    assert filled.bounds[0] == pytest.approx(-W / 2, abs=0.5)          # A1 補到西邊界
    east = unary_union([Polygon([xy(*q) for q in r]) for r in p.block_rings["B3"]])
    assert east.bounds[2] == pytest.approx(W / 2, abs=0.5) and east.bounds[0] > 100   # 東邊那一圈給 B3、不給 A1
    half = gp.plan_layout([AREA], {"A1": BLOCKS["A1"], "B2": [rect(-244, 0, -H / 2 - 60, H / 2 + 60)]}, LayoutParams())
    assert any("只蓋到作業區的" in w for w in half.warnings)          # 東邊 600 m 沒人飛 ⇒ 照樣警告


def test_adjacent_areas_get_corner_gcps_as_one_connected_area(cross):
    """使用者畫 3 塊相鄰作業區（同一個 1200×800 m）：角落／中央 GCP 以連通區為單位 ⇒ 與單一作業區相同的 8 個，
    不是逐塊套用的 18 個（審查探針 2026-10-15；18 個正落在 Gerke 說差異消失的 17–18 個區間）。"""
    thirds = [rect(-W / 2 + k * W / 3, -W / 2 + (k + 1) * W / 3, -H / 2, H / 2) for k in range(3)]
    p = gp.plan_layout(thirds, BLOCKS, LayoutParams(preset="cross_strips"))
    assert p.counts["GCP"] == cross.counts["GCP"] == 8
    assert p.counts["GCP_by_purpose"] == {"block": 3, "centre": 1, "corner": 4}
    assert any("相接成 1 塊連通區" in n for n in p.notes)
    assert not any("不是「少 GCP」" in w for w in p.warnings)
    # 真的放了很多 GCP（使用者把下限拉到 12）⇒ 交叉航帶方案大聲警告
    many = gp.plan_layout([AREA], BLOCKS, LayoutParams(preset="cross_strips", min_total=12))
    assert many.counts["GCP"] == 12 and any("不是「少 GCP」" in w for w in many.warnings)
    # 不相接的多塊作業區照舊逐塊放（行為不變）
    apart = gp.plan_layout([rect(-1500, -900, -H / 2, H / 2), rect(900, 1500, -H / 2, H / 2)], None,
                           LayoutParams(preset="cross_strips"))
    assert apart.counts["GCP_by_purpose"]["corner"] == 8 and not any("連通區" in n for n in apart.notes)


# ─────────────────────────────────────────────────────────────────────────────
# 由 HARVEST 規劃取區塊
# ─────────────────────────────────────────────────────────────────────────────

class _Frame:
    def __init__(self):
        self.t = Transformer.from_crs(UTM, "EPSG:4326", always_xy=True)

    def to_wgs84(self, enu):
        enu = np.asarray(enu, float)
        lon, lat = self.t.transform(E0 + enu[:, 0], N0 + enu[:, 1])
        return np.column_stack([lat, lon, enu[:, 2]])


def _fake_hv():
    strips, plans = {}, {}
    for uid, x_list in (("A1", (-580, -540, -500)), ("B2", (-100, -60))):
        seq = []
        for k, x in enumerate(x_list):
            sid = f"{uid}_s{k}"
            strips[sid] = SimpleNamespace(pL=np.array([x, -500.0]), pR=np.array([x, 500.0]), spacing_m=40.0)
            seq.append((sid, k % 2))
        plans[uid] = SimpleNamespace(strips=seq, cls="A" if uid == "A1" else "B")
    plan = SimpleNamespace(plans=plans, strips=strips)
    return SimpleNamespace(last_plan=plan, last_ctx=SimpleNamespace(frame=_Frame()), id_map={1: "A1", 2: "B2"})


def test_blocks_from_hv_are_the_union_of_strip_bands():
    blocks, info = gp.blocks_from_hv(_fake_hv())
    assert sorted(blocks) == ["A1", "B2"]
    assert info["A1"] == {"uav_id": "A1", "drone_id": 1, "cls": "A", "n_strips": 3, "n_cross_excluded": 0}
    a = Polygon([xy(*q) for q in blocks["A1"][0]])
    assert a.area == pytest.approx(120.0 * 1000.0, rel=1e-4)      # 3 條 × 40 m × 1000 m（相鄰帶無縫相接）
    assert a.bounds == pytest.approx((-600.0, -500.0, -480.0, 500.0), abs=0.05)
    assert gp.blocks_from_hv(_fake_hv())[0] == blocks              # 唯讀、可重複
    with pytest.raises(GcpPlanError, match="沒有 HARVEST 規劃"):
        gp.blocks_from_hv(SimpleNamespace(last_plan=None, last_ctx=None))


def test_cross_strips_do_not_enlarge_the_block():
    """交叉航帶兩端伸進鄰機區塊（審查探針：區塊 +10–50%）⇒ blocks_from_hv 只算主條帶。"""
    from hv_dccpp.geometry import cross_strip_id
    hv = _fake_hv()
    base, _ = gp.blocks_from_hv(hv)
    sid = cross_strip_id("Z1", "A", "A1", 0)
    # 東西向、從 A1 一路伸到 B2 那一側（−700 → −20 m）
    hv.last_plan.strips[sid] = SimpleNamespace(pL=np.array([-700.0, 0.0]), pR=np.array([-20.0, 0.0]), spacing_m=40.0)
    hv.last_plan.plans["A1"].strips.append((sid, 0))
    blocks, info = gp.blocks_from_hv(hv)
    assert blocks == base
    assert info["A1"]["n_strips"] == 3 and info["A1"]["n_cross_excluded"] == 1


def test_blocks_from_a_real_harvest_plan_give_one_gcp_per_uav():
    pytest.importorskip("hv_dccpp")
    from tests.test_hv_dccpp_adapter import _areas, _hv_coordinator, _plan_kwargs
    coord = _hv_coordinator()
    coord.plan_coverage_dccpp(**_plan_kwargs())
    hv = coord.hv_coordinator
    blocks, info = gp.blocks_from_hv(hv)
    assert len(blocks) >= 2 and {i["drone_id"] for i in info.values()} <= {1, 2}
    area = _areas()[0]["polygon"]
    plan = gp.plan_layout([area], blocks, LayoutParams(), blocks_source="HARVEST")
    assert plan.effective["blocks_coverage"] > 0.995                 # 條帶（含 w/2 影像）蓋滿作業區
    assert not any("只蓋到" in w for w in plan.warnings), plan.warnings
    for lb in blocks:
        assert any(lb in p.block.split("|") for p in plan.gcps() if p.purpose == "block"), lb
    # 每架飛的條帶中點都落在自己的區塊裡（區塊真的是「這架飛的地方」）
    for uid, up in hv.last_plan.plans.items():
        poly = gp.LocalFrame(*np.mean(np.asarray(area), axis=0))
        bpoly = unary_union([Polygon(np.column_stack(poly.en(*np.asarray(r).T))) for r in blocks[uid]])
        for sid, _d in up.strips:
            st = hv.last_plan.strips[sid]
            mid = (np.asarray(st.pL)[:2] + np.asarray(st.pR)[:2]) / 2
            lat, lon, _ = hv.last_ctx.frame.to_wgs84(np.array([[mid[0], mid[1], 0.0]]))[0]
            assert bpoly.buffer(1.0).contains(Point(*poly.en(lat, lon))), (uid, sid)


# ─────────────────────────────────────────────────────────────────────────────
# ODM gcp_list.txt（docs.opendronemap.org/gcp 與 opendm/gcp.py、location.py，2026-10-15 核對）
# ─────────────────────────────────────────────────────────────────────────────

def test_odm_header_rules_match_opendm_location_parse_srs_header():
    utm = gp.parse_odm_srs_header("WGS84 UTM 51N")
    t = Transformer.from_crs("EPSG:4326", utm, always_xy=True)
    assert t.transform(121.21, 25.07) == pytest.approx(TO_UTM.transform(121.21, 25.07), abs=1e-6)
    south = Transformer.from_crs("EPSG:4326", gp.parse_odm_srs_header("WGS84 UTM 51S"), always_xy=True)
    assert south.transform(121.21, -25.07) == pytest.approx(
        Transformer.from_crs("EPSG:4326", "EPSG:32751", always_xy=True).transform(121.21, -25.07), abs=1e-6)
    assert gp.parse_odm_srs_header("EPSG:4326").is_geographic
    assert gp.parse_odm_srs_header("+proj=utm +zone=51 +datum=WGS84 +units=m +no_defs").is_projected
    for bad in ("UTM 51N", "WGS84 UTM 61N", "WGS84  UTM 51N", "TWD97", "epsg:abc", ""):
        with pytest.raises(GcpPlanError):
            gp.parse_odm_srs_header(bad)
    assert gp.utm_header_for(25.07, 121.21) == "WGS84 UTM 51N"


def test_template_has_header_comments_and_one_line_per_gcp_in_odm_order(cross):
    txt = gp.gcp_template_text(cross, projection="WGS84 UTM 51N")
    lines = txt.splitlines()
    assert lines[0] == "WGS84 UTM 51N"
    data = [ln for ln in lines[1:] if ln and not ln.startswith("#")]
    assert len(data) == cross.counts["GCP"] == 8
    for ln, p in zip(data, cross.gcps()):
        parts = ln.split()
        assert len(parts) == 7                                       # geo_x geo_y geo_z im_x im_y image_name gcp_name
        assert parts[2:6] == ["<geo_z>", "<im_x>", "<im_y>", "<image_name>"] and parts[6] == p.id
        e, n = TO_UTM.transform(p.lon, p.lat)
        assert (float(parts[0]), float(parts[1])) == pytest.approx((e, n), abs=0.002)
    assert "CHK" not in "\n".join(data)
    geo = gp.gcp_template_text(cross, projection="EPSG:4326").splitlines()
    first = [ln for ln in geo[1:] if not ln.startswith("#")][0].split()
    g1 = cross.gcps()[0]
    assert (float(first[0]), float(first[1])) == pytest.approx((g1.lon, g1.lat), abs=1e-8)   # x = 經度
    assert any("橢球高" in ln and "e-GNSS[2025]" in ln for ln in geo if ln.startswith("#"))


def _fill(template: str, images=("IMG_1.jpg", "IMG_2.jpg", "IMG_3.jpg", "IMG_4.jpg", "IMG_5.jpg"), z=50.0):
    out = []
    for ln in template.splitlines():
        if not ln or ln.startswith("#") or "<" not in ln:
            out.append(ln)
            continue
        x, y, _z, _ix, _iy, _img, name = ln.split()
        for k, im in enumerate(images):
            out.append(f"{x} {y} {z} {100 + k} {200 + k} {im} {name}")
    return "\n".join(out) + "\n"


def test_validate_refuses_templates_unknown_images_mixed_datums_and_checkpoints(cross, tmp_path):
    tpl = gp.gcp_template_text(cross, projection="WGS84 UTM 51N")
    raw = tmp_path / "gcp_list.txt"
    raw.write_text(tpl, encoding="utf-8")
    names = [f"IMG_{k}.jpg" for k in range(1, 6)]
    rep = gp.validate_gcp_list(raw, image_names=names, gcp_vertical="ellipsoidal", height="ellipsoidal")
    assert rep["errors"] and all("樣板佔位" in e for e in rep["errors"][:-1])
    raw.write_text(_fill(tpl), encoding="utf-8")
    ok = gp.validate_gcp_list(raw, image_names=names, image_sizes={n: (4056, 3040) for n in names},
                              gcp_vertical="ellipsoidal", height="ellipsoidal")
    assert ok["errors"] == [] and ok["n_gcps"] == 8 and ok["n_lines"] == 40 and ok["warnings"] == []
    assert set(ok["images_per_gcp"].values()) == {5}
    bad = gp.validate_gcp_list(raw, image_names=names[:4], gcp_vertical="ellipsoidal", height="ellipsoidal")
    assert sum("IMG_5.jpg 不在本次要上傳的影像裡" in e for e in bad["errors"]) == 8
    mix = gp.validate_gcp_list(raw, image_names=names, gcp_vertical="orthometric", height="ellipsoidal")
    assert any("混用" in e for e in mix["errors"])
    none = gp.validate_gcp_list(raw, image_names=names, gcp_vertical=None, height="ellipsoidal")
    assert any("明確指定" in e for e in none["errors"])
    chk = gp.validate_gcp_list(raw, image_names=names, gcp_vertical="ellipsoidal", forbidden_names=["GCP03"])
    assert any("檢核點 ['GCP03']" in e for e in chk["errors"])
    px = gp.validate_gcp_list(raw, image_names=names, image_sizes={n: (64, 48) for n in names},
                              gcp_vertical="ellipsoidal")
    assert any("超出影像 64×48" in e for e in px["errors"])
    raw.write_text(_fill(tpl, images=names[:3], z=0.0), encoding="utf-8")
    few = gp.validate_gcp_list(raw, image_names=names, gcp_vertical="ellipsoidal")
    assert few["errors"] == [] and sum("只標在 3 張" in w for w in few["warnings"]) == 8
    assert any("geo_z = 0.0" in w for w in few["warnings"])
    geo = tmp_path / "geo.txt"
    geo.write_text(_fill(tpl), encoding="utf-8")
    assert any("geo.txt" in e for e in gp.validate_gcp_list(geo, gcp_vertical="ellipsoidal")["errors"])


def test_gcp_list_from_survey_uses_surveyed_values_and_refuses_unknown_frames(cross, tmp_path):
    txt = gp.survey_template_csv_text(cross)
    rows = [ln.split(",") for ln in txt.splitlines()]
    head = rows[0]
    assert tuple(head) == gp.SURVEY_COLUMNS
    i = {c: k for k, c in enumerate(head)}
    out = [",".join(head)]
    for r in rows[1:]:
        r = list(r)
        r[i["lat"]], r[i["lon"]], r[i["h"]] = r[i["planned_lat"]], r[i["planned_lon"]], "48.512"
        r[i["frame"]] = "EPSG:4326"
        out.append(",".join(r))
    p = tmp_path / "survey.csv"
    p.write_text("\n".join(out) + "\n", encoding="utf-8")
    text = gp.gcp_list_from_survey(p, projection="WGS84 UTM 51N", vertical="ellipsoidal")
    data = [ln.split() for ln in text.splitlines()[1:] if not ln.startswith("#")]
    assert len(data) == 8 and all(d[2] == "48.512" and d[3] == "<im_x>" for d in data)
    g1 = cross.gcps()[0]
    assert (float(data[0][0]), float(data[0][1])) == pytest.approx(TO_UTM.transform(g1.lon, g1.lat), abs=0.002)
    with pytest.raises(GcpPlanError, match="不是正高|高程基準"):
        gp.gcp_list_from_survey(p, projection="WGS84 UTM 51N", vertical="orthometric")
    p.write_text(p.read_text(encoding="utf-8").replace("EPSG:4326", "e-GNSS[2025]"), encoding="utf-8")
    with pytest.raises(GcpPlanError, match="不是 PROJ 可解析"):
        gp.gcp_list_from_survey(p, projection="WGS84 UTM 51N", vertical="ellipsoidal")
    t2 = gp.gcp_list_from_survey(p, projection="EPSG:4326", vertical="ellipsoidal",
                                 frame_crs="+proj=longlat +ellps=GRS80 +no_defs")
    first = [ln for ln in t2.splitlines()[1:] if not ln.startswith("#")][0].split()
    assert float(first[0]) == pytest.approx(g1.lon, abs=1e-7) and float(first[1]) == pytest.approx(g1.lat, abs=1e-7)


def test_gcp_list_from_survey_refuses_mixed_or_unnamed_orthometric_systems(cross, tmp_path):
    """EGM96 與 TWVD2001 都是「正高」，但不能混在同一份 gcp_list（審查探針 2026-10-15）；只寫「正高」也不收。"""
    txt = gp.survey_template_csv_text(cross)
    rows = [ln.split(",") for ln in txt.splitlines()]
    head = rows[0]
    i = {c: k for k, c in enumerate(head)}

    def write(datums):
        out = [",".join(head)]
        for k, r in enumerate(rows[1:]):
            r = list(r)
            r[i["lat"]], r[i["lon"]], r[i["h"]] = r[i["planned_lat"]], r[i["planned_lon"]], "30.000"
            r[i["frame"]] = "EPSG:4326"
            r[i["vertical_datum"]] = datums[k % len(datums)]
            out.append(",".join(r))
        p = tmp_path / "survey.csv"
        p.write_text("\n".join(out) + "\n", encoding="utf-8")
        return p

    with pytest.raises(GcpPlanError, match="混用了不同正高系統"):
        gp.gcp_list_from_survey(write(["TWVD2001", "EGM96"]), projection="WGS84 UTM 51N", vertical="orthometric")
    with pytest.raises(GcpPlanError, match="要寫明是 EGM96 還是 TWVD2001"):
        gp.gcp_list_from_survey(write(["正高"]), projection="WGS84 UTM 51N", vertical="orthometric")
    text = gp.gcp_list_from_survey(write(["TWVD2001"]), projection="WGS84 UTM 51N", vertical="orthometric")
    g = tmp_path / "gcp_list.txt"
    g.write_text(text, encoding="utf-8")
    assert gp.declared_ortho_system(g) == "TWVD2001"
    g.write_text(gp.gcp_list_from_survey(write(["ellipsoidal"]), projection="WGS84 UTM 51N", vertical="ellipsoidal"),
                 encoding="utf-8")
    assert gp.declared_ortho_system(g) is None


# ─────────────────────────────────────────────────────────────────────────────
# 輸出
# ─────────────────────────────────────────────────────────────────────────────

def test_export_writes_every_file_deterministically_and_refuses_overwrite(cross, tmp_path):
    paths = gp.export_layout(cross, tmp_path / "a", projection="WGS84 UTM 51N")
    assert sorted(paths) == ["csv", "effective", "field_sheet", "gcp_template", "geojson", "kml", "survey_csv"]
    csv_text = paths["csv"].read_text(encoding="utf-8-sig")
    assert csv_text.splitlines()[0] == "id,type,lat,lon,block,purpose,note"
    assert paths["csv"].read_bytes().startswith(b"\xef\xbb\xbf") and b"\r\n" not in paths["csv"].read_bytes()
    rows = gp.read_layout_csv(paths["csv"])
    assert [r["id"] for r in rows] == [p.id for p in cross.points]
    seam = [r for r in rows if r["purpose"] == "seam_pair"]
    assert {(r["seam"], r["pair"]) for r in seam} == {("A1|B2", 1), ("B2|B3", 1)}
    kml = ET.fromstring(paths["kml"].read_text(encoding="utf-8"))
    ns = {"k": "http://www.opengis.net/kml/2.2"}
    marks = kml.findall(".//k:Placemark", ns)
    assert len(marks) == 28 + 1                                       # 點＋作業區
    gj = json.loads(paths["geojson"].read_text(encoding="utf-8"))
    pts = [f for f in gj["features"] if f["geometry"]["type"] == "Point"]
    assert len(pts) == 28 and pts[0]["geometry"]["coordinates"] == [round(cross.points[0].lon, 8),
                                                                     round(cross.points[0].lat, 8)]
    sheet = paths["field_sheet"].read_text(encoding="utf-8")
    assert "e-GNSS[2025]" in sheet and "橢球高" in sheet and "天線高" in sheet and "GCP08" in sheet
    eff = json.loads(paths["effective"].read_text(encoding="utf-8"))
    assert eff["export"] == {"projection": "WGS84 UTM 51N", "vertical": "ellipsoidal", "survey_frame": "e-GNSS[2025]"}
    with pytest.raises(GcpPlanError, match="已有"):
        gp.export_layout(cross, tmp_path / "a", projection="WGS84 UTM 51N")
    again = gp.export_layout(gp.plan_layout([AREA], BLOCKS, LayoutParams()), tmp_path / "b", projection="WGS84 UTM 51N")
    for k in paths:
        assert paths[k].read_bytes() == again[k].read_bytes(), k
    with pytest.raises(GcpPlanError):
        gp.export_layout(cross, tmp_path / "c", projection="TWD97")


def test_map_points_carry_type_and_label(cross):
    pts = gp.map_points(cross)
    assert len(pts) == 28 and {p["type"] for p in pts} == {"GCP", "CHK"}
    assert pts[0]["label"].startswith("GCP01")


# ═════════════════════════════════════════════════════════════════════════════
# 流水線：使用者的 gcp_list.txt 送 NodeODM（survey/pipeline.py）＋ 風旗標
# ═════════════════════════════════════════════════════════════════════════════
from survey import dflog, pipeline  # noqa: E402
from survey.pipeline import PipelineError  # noqa: E402
from tests.test_survey_pipeline import FakeNodeOdm, fleet, run, sortie  # noqa: E402,F401  (fleet 是 fixture)
from tests.test_survey_synthlog import DFWriter  # noqa: E402

JOB_KEYS_WITHOUT_GCP = {"node_url", "images", "geo_txt", "options", "name", "qa_passed", "qa_override", "qa_reasons",
                        "rolling_shutter", "out_dir"}


def _gcp_lines(images, *, n_gcp=5, ix=30.0, iy=20.0):
    out = ["EPSG:4326", "# 測試"]
    for g in range(n_gcp):
        for im in images[:5]:
            out.append(f"{121.2 + g * 1e-3:.6f} {25.1:.6f} {48.5 + g:.3f} {ix} {iy} {im} GCP{g + 1:02d}")
    return "\n".join(out) + "\n"


@pytest.fixture(scope="module")
def fc_result(fleet, tmp_path_factory):
    return run(tmp_path_factory.mktemp("gcpodm") / "out", [sortie(fleet["h1"], pos=False)])


def test_odm_job_without_gcp_is_unchanged_and_with_gcp_carries_the_checked_file(fc_result, tmp_path):
    from survey.odm_client import NodeOdmClient
    res = fc_result
    job0 = pipeline.odm_job(res, node_url="http://127.0.0.1:3000", orthophoto_resolution_cm=2.5, rolling_shutter=False)
    assert set(job0) == JOB_KEYS_WITHOUT_GCP                            # 沒給 GCP：任務內容與加入功能前相同
    fake0 = FakeNodeOdm()
    pipeline.run_odm_job(job0, client=NodeOdmClient(job0["node_url"], session=fake0), poll_s=1.0, sleep=lambda s: None)
    assert sorted(fake0.posted[0]["names"]) == sorted([p.name for p in res.images] + ["geo.txt"])
    names = [p.name for p in res.images]
    g = tmp_path / "gcp_list.txt"
    g.write_text(_gcp_lines(names), encoding="utf-8")
    job = pipeline.odm_job(res, node_url="http://127.0.0.1:3000", orthophoto_resolution_cm=2.5, rolling_shutter=False,
                           gcp_txt=g, gcp_vertical="ellipsoidal")
    assert set(job) == JOB_KEYS_WITHOUT_GCP | {"gcp_txt", "gcp"}
    assert job["gcp_txt"] == str(g) and job["gcp"]["n_gcps"] == 5 and job["gcp"]["n_lines"] == 25
    assert job["gcp"]["header"] == "EPSG:4326" and job["gcp"]["gcp_vertical"] == "ellipsoidal"
    assert set(job["gcp"]["images_per_gcp"].values()) == {5} and job["gcp"]["warnings"] == []
    fake = FakeNodeOdm()
    pipeline.run_odm_job(job, client=NodeOdmClient(job["node_url"], session=fake), poll_s=1.0, sleep=lambda s: None)
    assert sorted(fake.posted[0]["names"]) == sorted(names + ["geo.txt", "gcp_list.txt"])
    rec = json.loads((res.out_dir / "odm" / "submission.json").read_text(encoding="utf-8"))
    assert rec["gcp_txt"] == str(g) and rec["gcp"]["n_gcps"] == 5


def test_odm_job_refuses_bad_gcp_files(fc_result, tmp_path):
    res = fc_result
    names = [p.name for p in res.images]
    g = tmp_path / "gcp_list.txt"
    kw = dict(node_url="http://127.0.0.1:3000", orthophoto_resolution_cm=2.5, rolling_shutter=False)
    g.write_text(_gcp_lines(names, ix=100.0), encoding="utf-8")                # 影像只有 64×48
    with pytest.raises(PipelineError, match="超出影像 64×48"):
        pipeline.odm_job(res, gcp_txt=g, gcp_vertical="ellipsoidal", **kw)
    g.write_text(_gcp_lines(names), encoding="utf-8")
    with pytest.raises(PipelineError, match="混用"):
        pipeline.odm_job(res, gcp_txt=g, gcp_vertical="orthometric", **kw)
    with pytest.raises(PipelineError, match="明確指定"):
        pipeline.odm_job(res, gcp_txt=g, **kw)
    with pytest.raises(PipelineError, match="檢核點"):
        pipeline.odm_job(res, gcp_txt=g, gcp_vertical="ellipsoidal", checkpoint_ids=["GCP02"], **kw)
    with pytest.raises(PipelineError, match="卻沒有 gcp_list.txt"):
        pipeline.odm_job(res, gcp_vertical="ellipsoidal", **kw)
    g.write_text(_gcp_lines(["NOPE.jpg"] * 5), encoding="utf-8")
    with pytest.raises(PipelineError, match="NOPE.jpg 不在本次要上傳的影像裡"):
        pipeline.odm_job(res, gcp_txt=g, gcp_vertical="ellipsoidal", **kw)
    tpl = tmp_path / "tpl.txt"
    tpl.write_text("EPSG:4326\n121.2 25.1 <geo_z> <im_x> <im_y> <image_name> GCP01\n", encoding="utf-8")
    with pytest.raises(PipelineError, match="樣板佔位"):
        pipeline.odm_job(res, gcp_txt=tpl, gcp_vertical="ellipsoidal", **kw)


def test_check_gcp_for_job_refuses_a_different_orthometric_system(tmp_path):
    """geo.txt 用 EGM96、gcp_list 宣告 TWVD2001 ⇒ 擋；沒宣告 ⇒ 警告（不猜）。"""
    names = [f"IMG_{k:04d}.jpg" for k in range(5)]
    res = SimpleNamespace(images=[tmp_path / n for n in names], sorties=[],
                          effective={"height": "orthometric", "geoid": {"datum": "EGM96"}})
    g = tmp_path / "gcp_list.txt"
    body = _gcp_lines(names).splitlines()
    g.write_text("\n".join([body[0], "# geo_z = 正高（TWVD2001）"] + body[1:]) + "\n", encoding="utf-8")
    with pytest.raises(PipelineError, match="TWVD2001.*EGM96.*不混用"):
        pipeline.check_gcp_for_job(res, g, gcp_vertical="orthometric")
    g.write_text("\n".join([body[0], "# geo_z = 正高（EGM96）"] + body[1:]) + "\n", encoding="utf-8")
    assert pipeline.check_gcp_for_job(res, g, gcp_vertical="orthometric")["errors"] == []
    g.write_text(_gcp_lines(names), encoding="utf-8")
    rep = pipeline.check_gcp_for_job(res, g, gcp_vertical="orthometric")
    assert any("沒宣告是哪一個正高系統" in w for w in rep["warnings"])


def test_wind_rows_survive_no_photo_with_a_wind_estimate():
    """EKF 有風、但沒有任何照片取得到風值（XKF2 間隔 > 2 s 或零照片）⇒ 不能丟 TypeError（審查探針）。"""
    wind = {"limit_mps": 3.0, "available": True, "share_above": None, "flagged": False, "n_photos_above": 0,
            "n_photos_with_wind": 0, "photo_speed_median_mps": None, "photo_speed_max_mps": None,
            "core_selection": "XKF4.PI", "per_strip": []}
    rows = pipeline._wind_rows("A1", wind)
    assert len(rows) == 1 and "中位 —、最大 — m/s" in rows[0][2] and "沒有任何照片取得到風估計" in rows[0][2]


def _with_wind(src_bin, dst_bin, vn=0.0, ve=5.0):
    """在合成 .BIN 後面接 XKF2（EKF3 風估計，'c' 格式）與 PARM AHRS_EKF_TYPE=3。"""
    log = dflog.read_dflog(src_bin)
    t0, t1 = log.time_range_us
    w = DFWriter()
    w.next_id = 200                                                      # 不跟原檔的訊息代號撞
    w.define("XKF2", XKF2_FMT_REAL, XKF2_COLS_REAL)
    w.define("PRM2", "QNff", ["TimeUS", "Name", "Value", "Default"])
    for t in range(int(t0), int(t1), 200_000):
        for c in (0, 1):
            w.write("XKF2", TimeUS=t, C=c, VWN=vn, VWE=ve)
    dst_bin.write_bytes(src_bin.read_bytes() + bytes(w.buf))
    return dst_bin


XKF2_COLS_REAL = ["TimeUS", "C", "AX", "AY", "AZ", "VWN", "VWE", "MN", "ME", "MD", "MX", "MY", "MZ", "IDX", "IDY", "IS"]
XKF2_FMT_REAL = "QBccccchhhhhhfff"


def test_pipeline_flags_wind_above_the_mapping_limit_without_failing_qa(fleet, tmp_path):
    s = fleet["h1"]
    b = _with_wind(s["bin"], tmp_path / "wind.BIN")
    res = run(tmp_path / "w", [pipeline.SortieInput(s["uav"], b, s["sess"])])
    assert res.passed is True                                            # 風只是旗標
    sr = res.sorties[0]
    assert sr.wind["available"] and sr.wind["flagged"] and sr.wind["share_above"] == 1.0
    assert sr.wind["core_selection"].startswith("核心 0")                # 沒有 XKF4 ⇒ 用核心 0 並寫明
    assert any("EKF 估計風速 > 3 m/s" in w_ for w_ in res.warnings)
    q = json.loads((res.out_dir / "qa.json").read_text(encoding="utf-8"))
    assert q["effective"]["wind_qa"]["limit_mps"] == 3.0 and q["qa"]["per_sortie"][s["uav"]]["wind"]["flagged"]
    assert "wind_qa" not in (res.out_dir / "photos.meta.json").read_text(encoding="utf-8")
    rows = [r for r in pipeline.qa_table_rows(res) if r[1].startswith(("EKF 風速", "風 第"))]
    assert rows[0][1] == "EKF 風速（上限 3 m/s）" and rows[0][2].startswith("⚠ 照片 100.0% 超過") and rows[0][3] is None
    assert len(rows) == 1 + len(sr.wind["per_strip"]) and "來自 270°" in rows[1][2]
    ok = run(tmp_path / "w6", [pipeline.SortieInput(s["uav"], b, s["sess"])], wind_limit_mps=6.0)
    assert not ok.sorties[0].wind["flagged"] and not any("EKF 估計風速" in w_ for w_ in ok.warnings)
    plain = run(tmp_path / "p", [sortie(s, pos=False)])                 # 合成日誌沒有 XKF2 ⇒ 說「無法確認」
    assert not plain.sorties[0].wind["available"] and any("無法確認作業時的風速" in w_ for w_ in plain.warnings)
    for bad in (0.0, -1.0, float("nan"), "3", True, 99.0):
        with pytest.raises(PipelineError, match="風速上限"):
            run(tmp_path / f"bad{bad}", [sortie(s, pos=False)], wind_limit_mps=bad)
    assert not (tmp_path / "bad0.0").exists()
