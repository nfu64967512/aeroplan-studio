"""交叉航帶（建圖模式 cross_strips）的 GUI／協調器／轉接層／SITL 旋鈕／曝光驗證器接線（值層級）。

核心規劃器的幾何與排程鎖在 tests/hv_dccpp/test_cross_strips.py；這裡鎖「面板 → build_mapping_setup →
SwarmCoordinator → HvDccppCoordinator → 計畫／任務項目」的值一路不變、沒勾時完全不出現，以及
scripts/hv_sitl_run.py 的 HV_CROSS* 旋鈕、core/recon/exposure_verifier.py 的方向分視數。

真規劃情境（與 test_hdmap_gui_coordinator.py 同一個）：1.2 × 1.0 km 平地、1 Pioneer（min_fw 100）+ 2 HERO、
HQ IMX477 + 6 mm、120 m、無豁免、無風。
"""
from __future__ import annotations

import math
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from mission.swarm_coordinator import SwarmCoordinator, build_mapping_setup, mapping_setup_note  # noqa: E402

pytest.importorskip("hv_dccpp")

from hv_dccpp.ardupilot import MAV_CMD_DO_SET_CAM_TRIGG_DIST  # noqa: E402
from hv_dccpp.geometry import is_cross_strip  # noqa: E402
from hv_dccpp.mission import MappingModeConfig  # noqa: E402

from core.recon import exposure_verifier as ev  # noqa: E402

runner = pytest.importorskip("scripts.hv_sitl_run")

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
CLASS_OF = {1: "A", 2: "B", 3: "B"}
F_MM, PIX_UM, W_PX, H_PX = 6.0, 1.55, 4056, 3040
ALT = 120.0
SIDE = 0.70
SWATH = ALT * W_PX * PIX_UM * 1e-3 / F_MM


def ll(east_m, north_m):
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def params(**over) -> dict:
    w_mm, h_mm = W_PX * PIX_UM * 1e-3, H_PX * PIX_UM * 1e-3
    p = {
        "hfov_deg": 2.0 * math.degrees(math.atan(w_mm / (2.0 * F_MM))),
        "vfov_deg": 2.0 * math.degrees(math.atan(h_mm / (2.0 * F_MM))),
        "hv_c2_range_km": 25.0, "hv_waiver_max_agl_m": None, "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0},
        "hv_dh_band_min_m": 15.0, "hv_dem_margin_m": 3000.0, "target_gsd": 0.05, "hv_wind": (0.0, 0.0),
        "hv_reduce_waypoints": True,
        "hv_map_enabled": True, "hv_map_camera_preset": "rpi_hq_6mm", "hv_map_focal_mm": F_MM,
        "hv_map_pixel_um": PIX_UM, "hv_map_width_px": W_PX, "hv_map_height_px": H_PX,
        "hv_map_trigger_hz": 2.0, "hv_map_trigger_verified": True, "hv_map_readout_ms": None,
        "hv_map_forward_overlap": 0.80, "hv_map_side_overlap": SIDE, "hv_map_backend": "servo",
        "hv_map_firmware": None, "hv_map_relay_pin": None, "hv_map_relay_aux_servo": None,
        "hv_map_feedback_pin": -1, "hv_map_feedback_servo": None, "hv_map_max_roll_deg": 15.0,
        "hv_map_lead_in_m": 50.0, "hv_map_terrain_retrigger_tol": 0.10, "hv_map_tlog": False,
        "hv_map_cross_enabled": False, "hv_map_cross_fraction": None, "hv_map_cross_alt_offset_m": None,
        "hv_map_max_wind_mps": 3.0,
    }
    p.update(over)
    return p


def plan(p: dict) -> SwarmCoordinator:
    coord = SwarmCoordinator()
    coord.use_harvest = True
    coord.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    coord.hv_home = ll(0.0, -400.0)
    coord.hv_wind = tuple(p.get("hv_wind") or (0.0, 0.0))
    coord.hv_fence_zones = []
    coord.harvest_params = dict(p)
    areas = [{"area_id": 1, "priority": 1.0, "polygon": [ll(-600, 0), ll(600, 0), ll(600, 1000), ll(-600, 1000)]}]
    drones = [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": ll(-60.0 + 60.0 * i, -400.0), "speed": 20.0,
               "heading": 0.0, "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": SWATH * (1.0 - SIDE)}
              for i in range(3)]
    coord.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=SWATH * (1.0 - SIDE), overlap_rate=0.0,
                              coverage_altitude=ALT, mounting_angle_deg=0.0)
    assert coord.hv_coordinator is not None
    return coord


CROSS = dict(hv_map_cross_enabled=True, hv_map_cross_fraction=0.3, hv_map_cross_alt_offset_m=-20.0)


@pytest.fixture(scope="module")
def crossed():
    return plan(params(**CROSS))


# ═══════════════════════ 1. build_mapping_setup ═══════════════════════
def test_setup_passes_cross_values_into_mapping_mode():
    s = build_mapping_setup(params(**CROSS))
    mm = s["mapping_mode"]
    assert (mm.cross_strips, mm.cross_fraction, mm.cross_alt_offset_m, mm.cross_angle_offset_deg) == (True, 0.3, -20.0, 90.0)
    assert mm.lead_in_m == s["camera_cfg"].camera_lead_in_m == 50.0
    e = s["effective"]
    assert (e["cross_strips"], e["cross_fraction"], e["cross_alt_offset_m"]) == (True, 0.3, -20.0)
    note = mapping_setup_note(s)
    assert "交叉航帶 開（面積比例 0.3、高度差 -20 m）" in note
    assert any("交叉航帶" in w and "Gerke" in w for w in s["warnings"])


def test_setup_without_cross_is_the_old_mapping_mode():
    s = build_mapping_setup(params())
    assert s["mapping_mode"] == MappingModeConfig(lead_in_m=50.0)
    assert s["effective"]["cross_strips"] is False and s["effective"]["cross_fraction"] is None
    assert "交叉航帶 關" in mapping_setup_note(s)
    p = params()
    for k in ("hv_map_cross_enabled", "hv_map_cross_fraction", "hv_map_cross_alt_offset_m"):
        p.pop(k)
    assert build_mapping_setup(p)["mapping_mode"] == MappingModeConfig(lead_in_m=50.0)     # 缺鍵 ＝ 沒勾


@pytest.mark.parametrize("over, frag", [
    (dict(hv_map_cross_fraction=0.3), "沒勾交叉航帶卻給了 hv_map_cross_fraction"),
    (dict(hv_map_cross_alt_offset_m=-20.0), "沒勾交叉航帶卻給了 hv_map_cross_alt_offset_m"),
    (dict(hv_map_cross_enabled=True, hv_map_cross_fraction=0.0, hv_map_cross_alt_offset_m=0.0), "面積比例"),
    (dict(hv_map_cross_enabled=True, hv_map_cross_fraction=1.5, hv_map_cross_alt_offset_m=0.0), "面積比例"),
    (dict(hv_map_cross_enabled=True, hv_map_cross_fraction=None, hv_map_cross_alt_offset_m=0.0), "面積比例"),
    (dict(hv_map_cross_enabled=True, hv_map_cross_fraction=0.2, hv_map_cross_alt_offset_m=None), "高度差"),
    (dict(hv_map_cross_enabled=True, hv_map_cross_fraction=0.2, hv_map_cross_alt_offset_m=float("nan")), "高度差"),
    (dict(hv_map_cross_enabled="yes"), "hv_map_cross_enabled"),
])
def test_setup_rejects_bad_cross_values(over, frag):
    with pytest.raises(ValueError, match=frag):
        build_mapping_setup(params(**over))


# ═══════════════════════ 2. 面板 ═══════════════════════
try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from PyQt6.QtWidgets import QApplication
    _QT = True
except Exception:                                  # pragma: no cover
    _QT = False


@pytest.fixture(scope="module")
def qt_app():
    if not _QT:
        pytest.skip("Qt 匯入失敗")
    yield QApplication.instance() or QApplication([])


@pytest.fixture()
def panel(qt_app):
    pnl = VTOLReconPanel()
    pnl.algo_combo.setCurrentIndex(pnl.algo_combo.findData("hv_dccpp"))
    return pnl


def test_panel_cross_fields_default_off_and_locked(panel):
    p = panel.get_params()
    assert (p["hv_map_cross_enabled"], p["hv_map_cross_fraction"], p["hv_map_cross_alt_offset_m"]) == (False, None, None)
    assert not panel.hv_map_cross_check.isEnabled()                  # 建圖沒開 ⇒ 交叉航帶也鎖
    panel.hv_map_check.setChecked(True)
    assert panel.hv_map_cross_check.isEnabled()
    assert not panel.hv_map_cross_frac_spin.isEnabled() and not panel.hv_map_cross_dh_spin.isEnabled()
    assert panel.hv_map_cross_frac_spin.value() == pytest.approx(0.20)
    assert panel.hv_map_cross_dh_spin.value() == 0.0


def test_panel_cross_values_reach_mapping_mode(panel):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_cross_check.setChecked(True)
    assert panel.hv_map_cross_frac_spin.isEnabled() and panel.hv_map_cross_dh_spin.isEnabled()
    panel.hv_map_cross_frac_spin.setValue(0.35)
    panel.hv_map_cross_dh_spin.setValue(-15.0)
    panel.hv_map_backend_combo.setCurrentIndex(panel.hv_map_backend_combo.findData("servo"))   # relay 要韌體版本
    p = panel.get_params()
    assert (p["hv_map_cross_enabled"], p["hv_map_cross_fraction"], p["hv_map_cross_alt_offset_m"]) == (True, 0.35, -15.0)
    mm = build_mapping_setup(p)["mapping_mode"]
    assert (mm.cross_strips, mm.cross_fraction, mm.cross_alt_offset_m) == (True, 0.35, -15.0)
    panel.hv_map_cross_check.setChecked(False)                        # 取消勾 ⇒ 送 None（不送會被忽略的值）
    p = panel.get_params()
    assert (p["hv_map_cross_fraction"], p["hv_map_cross_alt_offset_m"]) == (None, None)
    assert build_mapping_setup(p)["mapping_mode"].cross_strips is False


# ═══════════════════════ 3. SwarmCoordinator → 轉接層 → 計畫／任務 ═══════════════════════
def test_coordinator_plans_cross_strips_with_the_panel_values(crossed):
    hv = crossed.hv_coordinator
    mm = hv.mapping_mode
    assert (mm.cross_strips, mm.cross_fraction, mm.cross_alt_offset_m) == (True, 0.3, -20.0)
    p = hv.last_plan
    xs = [s for up in p.plans.values() for s, _d in up.strips if is_cross_strip(s)]
    assert xs and all(p.strips[s].h_op_m == pytest.approx(100.0) for s in xs)
    eff = hv.mapping_effective
    assert eff["config"]["cross_fraction"] == 0.3
    assert any(w.startswith("mapping_cross(") and "高度差 -20 m" in w for w in eff["cross"])
    # 多區塊的機（A1 飛三個子區的 A 殘帶）：每組交叉帶是一段連續的、緊鄰同一母胞的主條帶（之前或之後）
    multi = 0
    for u, up in p.plans.items():
        ids = [s for s, _d in up.strips]
        cells = {p.strips[s].cell_id for s in ids if not is_cross_strip(s)}
        multi += len(cells) > 1
        for cid in {p.strips[s].cell_id for s in ids if is_cross_strip(s)}:
            pos = [i for i, s in enumerate(ids) if is_cross_strip(s) and p.strips[s].cell_id == cid]
            assert pos == list(range(pos[0], pos[-1] + 1)), (u, cid, pos)       # 連續
            nb = [ids[i] for i in (pos[0] - 1, pos[-1] + 1) if 0 <= i < len(ids)]
            assert any(not is_cross_strip(s) and p.strips[s].cell_id == cid for s in nb), (u, cid, nb)
    assert multi >= 1


def test_coordinator_mission_items_have_206_per_cross_strip(crossed):
    hv = crossed.hv_coordinator
    cc = crossed.hv_mapping_setup["camera_cfg"]
    p = hv.last_plan
    for did, uid in hv.id_map.items():
        up = p.plans.get(uid)
        if up is None or not up.strips:
            continue
        items = hv.mission_items(did, camera_cfg=cc)
        on, prev = [], False
        for it in items:
            if it["command"] == MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                o = it["param1"] > 0
                if o and not prev:
                    on.append(it["param1"])
                prev = o
        assert len(on) == len(up.strips)
        for (sid, _d), d in zip(up.strips, on):
            want = (100.0 if is_cross_strip(sid) else 120.0) * H_PX * PIX_UM * 1e-3 / F_MM * 0.2
            assert d == pytest.approx(want, rel=5e-3), (uid, sid, d, want)


def _scan_angles(hv, plan_):
    import hv_dccpp.integration.aeroplan_adapter as ad
    areas = [{"area_id": int(a.area_id[1:]), "priority": 1.0, "polygon": list(a.polygon_wgs84)}
             for a in hv.last_inputs.areas]
    drones = [{"drone_id": d, "name": f"UAV-{d}", "position": ll(0.0, -400.0), "speed": 20.0, "heading": 0.0,
               "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": SWATH * (1.0 - SIDE)} for d in hv.id_map]
    res = ad.from_mission_plan(plan_, hv.last_ctx, areas=areas, drones=drones, id_map=hv.id_map, types=hv.types)
    return {int(getattr(cp, "area_id")): float(getattr(cp, "scan_angle_deg")) for cp in res["coverage_paths"]}


def test_scan_angle_ignores_cross_strips(crossed, monkeypatch):
    """轉接層 coverage_paths 的掃描角是主條帶方向；交叉帶（比例大時可能比主條帶多）不參與中位數。
    做法：在真計畫上把每架的交叉帶複製到比主條帶多，掃描角仍是主條帶方向；拿掉排除（is_cross_strip → False）就翻過去。"""
    import dataclasses
    import hv_dccpp.integration.aeroplan_adapter as ad
    hv = crossed.hv_coordinator
    p = hv.last_plan
    strips = dict(p.strips)
    plans = {}
    for u, up in p.plans.items():
        mains = [x for x in up.strips if not is_cross_strip(x[0])]
        xs = [x for x in up.strips if is_cross_strip(x[0])]
        extra = []
        for k in range(len(mains) // max(1, len(xs)) + 2):
            for sid, d in xs:
                nid = f"{sid}_dup{k}"
                strips[nid] = dataclasses.replace(p.strips[sid], strip_id=nid)
                extra.append((nid, d))
        assert len(xs) + len(extra) > len(mains)
        plans[u] = dataclasses.replace(up, strips=list(up.strips) + extra)
    p2 = dataclasses.replace(p, strips=strips, plans=plans)
    main_hdg = {a: None for a in range(1, 10)}
    for up in p.plans.values():
        for sid, _d in up.strips:
            st = p.strips[sid]
            if not is_cross_strip(sid):
                main_hdg[int(next(c.area_id for c in p.cells if c.cell_id == st.cell_id)[1:])] = \
                    math.degrees(math.atan2(st.pR[0] - st.pL[0], st.pR[1] - st.pL[1])) % 180.0

    def d180(a, b):
        x = abs(a - b) % 180.0
        return min(x, 180.0 - x)

    got = _scan_angles(hv, p2)
    assert got and all(d180(v, main_hdg[a]) < 1.0 for a, v in got.items()), (got, main_hdg)
    monkeypatch.setattr(ad, "is_cross_strip", lambda _s: False)              # 對照：不排除 ⇒ 中位數翻成交叉方向
    bad = _scan_angles(hv, p2)
    assert any(d180(v, main_hdg[a]) > 45.0 for a, v in bad.items()), bad


# ═══════════════════════ 4. SITL 旋鈕 ═══════════════════════
def test_sitl_knobs_cross_values_and_record():
    k = runner.survey_knobs({"HV_MAPPING": "1", "HV_CROSS": "1", "HV_CROSS_FRACTION": "0.3",
                             "HV_CROSS_ALT_OFFSET": "-20"})
    assert (k.cross, k.cross_fraction, k.cross_alt_offset_m) == (True, 0.3, -20.0)
    assert k.mapping_config() == MappingModeConfig(lead_in_m=50.0, cross_strips=True, cross_fraction=0.3,
                                                   cross_alt_offset_m=-20.0)
    d = k.to_dict()
    assert (d["cross"], d["cross_fraction"], d["cross_alt_offset_m"]) == (True, 0.3, -20.0)
    assert d["env"]["HV_CROSS"] == "1" and d["env"]["HV_CROSS_FRACTION"] == "0.3"
    k2 = runner.survey_knobs({"HV_MAPPING": "1", "HV_CROSS": "1"})
    assert (k2.cross_fraction, k2.cross_alt_offset_m) == (0.2, 0.0)


def test_sitl_knobs_default_off_is_unchanged():
    assert runner.survey_knobs({}) is None
    assert runner.survey_knobs({"HV_CROSS": "0"}) is None                 # 只給 HV_CROSS=0 ＝ 沒給
    k = runner.survey_knobs({"HV_MAPPING": "1"})
    assert k.cross is False and k.mapping_config() == MappingModeConfig(lead_in_m=50.0)
    assert not {"cross", "cross_fraction", "cross_alt_offset_m"} & set(k.to_dict())   # survey_knobs.json 不變


@pytest.mark.parametrize("env, frag", [
    ({"HV_CROSS": "1"}, "需要 HV_MAPPING=1"),
    ({"HV_MAPPING": "1", "HV_CROSS_FRACTION": "0.3"}, "只在 HV_CROSS=1"),
    ({"HV_MAPPING": "1", "HV_CROSS": "0", "HV_CROSS_ALT_OFFSET": "-20"}, "只在 HV_CROSS=1"),
    ({"HV_MAPPING": "1", "HV_CROSS": "2"}, "HV_CROSS 只能是 0 或 1"),
    ({"HV_MAPPING": "1", "HV_CROSS": "1", "HV_CROSS_FRACTION": "0"}, "HV_CROSS_FRACTION"),
    ({"HV_MAPPING": "1", "HV_CROSS": "1", "HV_CROSS_FRACTION": "1.5"}, "HV_CROSS_FRACTION"),
    ({"HV_MAPPING": "1", "HV_CROSS": "1", "HV_CROSS_FRACTION": "abc"}, "HV_CROSS_FRACTION"),
    ({"HV_MAPPING": "1", "HV_CROSS": "1", "HV_CROSS_ALT_OFFSET": "nan"}, "HV_CROSS_ALT_OFFSET"),
])
def test_sitl_knobs_reject(env, frag):
    with pytest.raises(SystemExit, match=frag):
        runner.survey_knobs(env)


# ═══════════════════════ 5. 曝光驗證器：方向分視數 ═══════════════════════
def _photos(uav, strips, d=18.0, h=120.0):
    """strips: [(strip_key, (x0, y0), (x1, y1))]（公尺，本地）→ PhotoPose（每 d 公尺一張，航向沿帶）。"""
    out, seq = [], 0
    for key, a, b in strips:
        a, b = np.asarray(a, float), np.asarray(b, float)
        L = float(np.hypot(*(b - a)))
        yaw = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360.0
        for s in np.arange(0.0, L + 1e-9, d):
            x, y = a + (b - a) * (s / L)
            seq += 1
            out.append(ev.PhotoPose(uav=uav, strip=key, seq=seq, lat_deg=HOME[0] + y / _M_LAT,
                                    lon_deg=HOME[1] + x / _M_LON, h_m=h, yaw_deg=yaw))
    return out


CAM = ev.FrameCamera(F_MM, PIX_UM, W_PX, H_PX)
POLY = [ll(0, 0), ll(400, 0), ll(400, 600), ll(0, 600)]


def _scene(with_cross=True):
    main = [(str(i + 1), (x, -60.0), (x, 660.0)) for i, x in enumerate(np.arange(0.0, 401.0, 37.7))]
    cross = [(str(len(main) + 1 + j), (-60.0, y), (460.0, y)) for j, y in enumerate((280.0, 317.7))]
    return _photos("U1", main + (cross if with_cross else [])), len(main)


def test_exposure_direction_stats_auto_and_mapping_agree():
    photos, n_main = _scene()
    rep = ev.exposure_metrics(photos, POLY, CAM, terrain=0.0, cell_size_m=4.0)
    d = rep.directions
    assert d["method"] == "axis" and d["photos"]["cross"] > 0 and d["photos"]["unclassified"] == 0
    assert d["per_uav"]["U1"]["main_axis_deg"] == pytest.approx(0.0, abs=1.0)
    # 兩條東西向交叉帶（相距 37.7 m、刈幅 125.7 m）＋前後足跡 ⇒ 交叉帶看到的是中央一條橫帶
    fx = d["frac_seen"]["cross"]
    band = (37.7 + 125.7) / 600.0                                        # 兩條帶聯集的南北寬 / 測區南北長
    assert band * 0.9 < fx < band * 1.2
    assert d["frac_seen"]["both"] == pytest.approx(fx, abs=1e-9)          # 主條帶蓋滿 ⇒ both = cross
    assert d["frac_seen"]["main"] == pytest.approx(1.0, abs=1e-9)
    assert d["both_ge"][2] <= d["both_ge"][1] and d["cross_ge"][1] == pytest.approx(fx)
    labels = {("U1", str(k)): ("cross" if k > n_main else "main") for k in range(1, n_main + 3)}
    rep2 = ev.exposure_metrics(photos, POLY, CAM, terrain=0.0, cell_size_m=4.0, direction_of=labels)
    assert rep2.directions["method"] == "mapping"
    assert rep2.directions["photos"] == d["photos"] and rep2.directions["frac_seen"] == d["frac_seen"]


def test_exposure_direction_stats_leave_other_outputs_unchanged():
    photos, _n = _scene(with_cross=False)
    rep = ev.exposure_metrics(photos, POLY, CAM, terrain=0.0, cell_size_m=4.0)
    assert rep.directions["photos"]["cross"] == 0 and rep.directions["frac_seen"]["cross"] == 0.0
    # 只差 directions：其餘欄位與明確給「全部是主條帶」的對照完全相同
    rep_m = ev.exposure_metrics(photos, POLY, CAM, terrain=0.0, cell_size_m=4.0,
                                direction_of=lambda ph: "main")
    for f in ("frac_ge", "views_histogram", "photos_per_uav", "undercovered_area_m2", "seam_overlap_min"):
        assert getattr(rep, f) == getattr(rep_m, f), f
    assert rep.forward_overlap == rep_m.forward_overlap and rep.side_overlap == rep_m.side_overlap
    assert "directions" not in rep.effective                              # effective 不動


def test_window_direction_labels_maps_206_windows_to_strip_ids():
    lab = ev.window_direction_labels({"A1": ["Z1_1_A_1", "Z1_1_A_cross_A1_1", "Z1_1_A_2"]}, is_cross_strip)
    assert lab == {("A1", "1"): "main", ("A1", "2"): "cross", ("A1", "3"): "main"}
