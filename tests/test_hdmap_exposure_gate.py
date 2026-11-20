"""B3（2026-10-16）：飛前曝光驗證（core.recon.exposure_verifier.verify_preflight）接進建圖上傳閘門 —— 協調器端的值測試。

不建 MainWindow、不起 SITL、不開 COM 埠；規劃是真的（SwarmCoordinator HARVEST 建圖分支，1.2 × 1.0 km、
1 Pioneer + 2 HERO，見 tests/test_hdmap_gui_coordinator.py）。

鎖住的事：
* 摘要的輸入就是上傳／規劃生效的值：上傳那份 hv.mission_items、規劃風、規劃 DEM、匯出 .param 的空速、camera_cfg 的
  CAM1_INTRVAL_MIN／CAM1_DURATION、格 4 m、視數門檻 5；
* 真規劃：內部 0 破洞；206 的 50 Hz 量化保護（VENDOR #12，預設開）之後邊緣帶也 ≥ 5 視；把保護關掉 ⇒ 邊緣帶有
  視數不足 ⇒ 不擋；挖掉一架的相機 ⇒ 內部破洞 ⇒ 擋；
* 摘要是純資料（不留報告／全格網）；
* 驗證本身壞掉 ⇒ 協調器的建圖閘門擋（沒驗過不能當驗過），轉接層自己的 mapping_gate 不受影響；
* 測區扣掉禁航區（不扣的話區中央的禁航區會被當成內部破洞）；多層風取最大層。
GUI 端（hv_upload_gate、面板文字、上傳被擋）在 tests/test_hdmap_fix_gui.py 的 (m)。
"""
from __future__ import annotations

import dataclasses
import pickle
import types

import numpy as np
import pytest

pytest.importorskip("hv_dccpp")

import shapely  # noqa: E402

from hv_dccpp.ardupilot import recommended_params  # noqa: E402
from hv_dccpp.geo import GeoFrame  # noqa: E402
from hv_dccpp.mission import WindLayer  # noqa: E402
from mission import swarm_coordinator as SC  # noqa: E402
from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402
from tests.test_hdmap_gui_coordinator import (  # noqa: E402
    ALT, CLASS_OF, SIDE, SWATH, areas, drones, ll, mapping_params, plan)


@pytest.fixture(scope="module")
def mapped():
    return plan(mapping_params())


@pytest.fixture(scope="module")
def mapped_noguard():
    """同一份規劃，但 206 不扣 50 Hz 量化保護（舊匯出行為）⇒ 邊界只有最外側條帶的那一帶有一部分只剩 4 視。"""
    orig = SC.build_mapping_setup

    def no_guard(p):
        out = orig(p)
        out["camera_cfg"] = dataclasses.replace(out["camera_cfg"], trigger_overshoot_guard=False)
        return out
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(SC, "build_mapping_setup", no_guard)
        coord = plan(mapping_params())
    assert coord.hv_mapping_setup["camera_cfg"].trigger_overshoot_guard is False
    return coord


def _items_206(items):
    return [it for it in items if it["command"] == 206]


# ═══════════════════════════ 真規劃：量化保護之後邊緣帶也 ≥ 5 視 ═══════════════════════════
def test_real_plan_with_trigger_guard_has_no_edge_shortfall(mapped):
    """VENDOR #12：206 的 d × (1 − f_s/50) ⇒ 實際間距 ≤ l(1 − o_fwd)，最外側條帶單獨負責的那一帶也有 5 視。"""
    s = mapped.hv_exposure_summary
    assert mapped.hv_mapping_setup["camera_cfg"].trigger_overshoot_guard is True
    assert s["ok"] is True and s["undercovered_interior_area_m2"] == 0.0
    assert s["undercovered_edge_band_area_m2"] == 0.0 and s["edge_band_short_count"] == 0
    assert s["edge_band"]["min_views"] >= 5
    assert SC.mapping_exposure_block_reasons(s) == []
    assert mapped.hv_mapping_gate == {"uploadable": True, "reasons": []}


# ═══════════════════════════ 真規劃（保護關掉）：只有邊緣 ⇒ 不擋 ═══════════════════════════
def test_real_plan_only_edge_band_is_short_and_nothing_blocks(mapped_noguard):
    mapped = mapped_noguard
    s = mapped.hv_exposure_summary
    assert s["ok"] is True and s["level"] == "曝光層級（視數）"
    assert s["cell_size_m"] == SC.MAPPING_EXPOSURE_CELL_M == 4.0 and s["min_views"] == SC.MAPPING_EXPOSURE_MIN_VIEWS == 5
    assert s["undercovered_interior_area_m2"] == 0.0 and s["interior_hole_count"] == 0 and s["interior_holes"] == []
    assert s["interior"]["frac_ge"][5] == 1.0 and s["interior"]["min_views"] >= 5
    # 邊緣帶真的有視數不足（最外側條帶只蓋一次）——這就是「只有邊緣」的情境
    assert s["undercovered_edge_band_area_m2"] > 0.0 and s["edge_band_short_count"] >= 1
    assert s["edge_band"]["min_views"] < 5
    assert 40.0 < s["edge_band_m"] < 90.0                                      # w/2（照片橫向足跡寬一半）
    assert SC.mapping_exposure_block_reasons(s) == []
    assert mapped.hv_mapping_gate == {"uploadable": True, "reasons": []}
    # 每架都有模擬照片；接縫有列出（只顯示）
    hv = mapped.hv_coordinator
    assert set(s["photos_per_uav"]) == set(hv.last_plan.plans) and all(n > 0 for n in s["photos_per_uav"].values())
    assert s["drone_of"] == {u: d for d, u in hv.id_map.items() if u in hv.last_plan.plans}
    assert s["seams"] and 0.0 < s["seam_overlap_min"] < 1.0


def test_real_plan_note_states_level_and_assumptions(mapped_noguard):
    s = mapped_noguard.hv_exposure_summary
    note = SC.mapping_exposure_note(s)
    assert note.startswith("mapping_exposure(曝光層級（視數），格 4 m")
    for frag in ("內部破洞 0 處", "（不擋）", "規劃風 0 m/s 來自 0°", "CAM_MAX_ROLL 15° 未建模", "轉彎切角與轉彎動態未建模"):
        assert frag in note, frag
    # 驗證器自己的 notes 原樣保留（含它的未建模清單與邊緣帶統計）
    assert any("CAM_MAX_ROLL" in n for n in s["verifier_notes"])
    assert any(n.startswith("視數 < 5：邊緣帶") for n in s["verifier_notes"])


def test_summary_is_plain_data_without_the_report_or_grid(mapped):
    s = mapped.hv_exposure_summary

    def walk(v):
        assert not isinstance(v, np.ndarray), type(v)
        if isinstance(v, dict):
            for k, x in v.items():
                walk(k)
                walk(x)
        elif isinstance(v, (list, tuple)):
            for x in v:
                walk(x)
        else:
            assert v is None or isinstance(v, (str, int, float, bool)), type(v)
    walk(s)
    assert len(pickle.dumps(s)) < 100_000


# ═══════════════════════════ 真規劃：挖掉一架的相機 ⇒ 內部破洞 ⇒ 擋 ═══════════════════════════
def test_one_camera_off_makes_an_interior_hole_that_blocks(mapped, monkeypatch):
    hv = mapped.hv_coordinator
    real = hv.mission_items
    dark = sorted(hv.id_map)[0]

    def items(did, **kw):
        out = real(did, **kw)
        if int(did) == dark:                       # 這架所有 206 的距離改 0 ⇒ 一張都不拍
            out = [dict(it, param1=0.0) if it["command"] == 206 else it for it in out]
        return out

    monkeypatch.setattr(hv, "mission_items", items)
    s = SC.mapping_exposure_summary(hv, mapped.hv_mapping_setup["camera"])
    uid = hv.id_map[dark]
    assert s["ok"] is True and s["photos_per_uav"].get(uid, 0) == 0
    assert s["undercovered_interior_area_m2"] > 5.0e4
    assert s["interior_hole_count"] >= 1 and s["interior_holes"][0]["min_views"] == 0
    r = SC.mapping_exposure_block_reasons(s)
    assert len(r) == 1 and r[0].startswith("exposure_interior_hole(曝光層級（視數）：內部（離測區邊界 > ")
    assert f"{s['undercovered_interior_area_m2']:,.0f} m²" in r[0]


# ═══════════════════════════ 輸入＝生效值（有效值契約） ═══════════════════════════
class _Proxy:
    """把某幾個屬性換掉、其餘照原物件（ctx.inputs.wind 換成有風）。"""

    def __init__(self, base, **over):
        self._b, self._o = base, over

    def __getattr__(self, n):
        return self._o[n] if n in self._o else getattr(self._b, n)


def test_inputs_are_the_effective_planning_and_upload_values(mapped, monkeypatch):
    import core.recon.exposure_verifier as ev
    hv = mapped.hv_coordinator
    ctx, cc = hv.last_ctx, hv.camera_cfg
    upload = {hv.id_map[d]: hv.mission_items(d, camera_cfg=cc) for d in sorted(hv.id_map)
              if hv.id_map[d] in hv.last_plan.plans}
    seen = {}
    real = ev.verify_preflight

    def spy(missions, polygons, camera, **kw):
        seen.update(kw, missions=missions, polygons=polygons, camera=camera)
        return real(missions, polygons, camera, **kw)

    monkeypatch.setattr(ev, "verify_preflight", spy)
    windy = (WindLayer(0.0, 3000.0, 3.0, 270.0, z_ref="agl"),)
    monkeypatch.setattr(hv, "last_ctx", _Proxy(ctx, inputs=_Proxy(ctx.inputs, wind=windy)))
    s = SC.mapping_exposure_summary(hv, mapped.hv_mapping_setup["camera"])
    assert seen["missions"] == upload                                        # 上傳那一份，逐項相同
    assert seen["camera"] is mapped.hv_mapping_setup["camera"]
    assert seen["wind"] == (3.0, 270.0) and s["effective"]["wind"] == (3.0, 270.0)
    assert s["assumptions"][0].startswith("規劃風 3 m/s 來自 270°")
    assert seen["cell_size_m"] == 4.0 and seen["undercover_k"] == 5 and tuple(seen["ks"]) == (5, 9)
    assert seen["intrval_min_s"] == cc.interval_min_s and seen["trigger_duration_s"] == cc.duration_s
    assert seen["accept_radius"] == "mission"
    for uid, up in hv.last_plan.plans.items():
        p = recommended_params(ctx.specs[up.cls], None, hv.cfg, sensor_kinds=("rgb",), camera_cfg=cc, warnings=[])
        assert seen["airspeed_mps"][uid] == float(p["AIRSPEED_CRUISE"])
        assert seen["airspeed_limits_mps"][uid] == (float(p["AIRSPEED_MIN"]), float(p["AIRSPEED_MAX"]))
    lat, lon = upload[sorted(upload)[0]][0]["lat"], upload[sorted(upload)[0]][0]["lon"]
    assert seen["terrain"](lat, lon) == float(ctx.inputs.dem.sample_msl(lat, lon)[0])
    assert "規劃 DEM" in s["effective"]["terrain"] and s["effective"]["cam_max_roll_deg"] == cc.max_roll_deg


@pytest.mark.parametrize("layers, want", [
    ((WindLayer(0.0, 3000.0, 3.0, 270.0, z_ref="agl"),), ((3.0, 270.0), "規劃風（單層）")),
    ((WindLayer(0.0, 100.0, 2.0, 90.0, z_ref="agl"), WindLayer(100.0, 3000.0, 5.0, 540.0, z_ref="agl")),
     ((5.0, 180.0), "規劃風（2 層取最大）")),
    ((WindLayer(0.0, 3000.0, 4.0, 45.0, gust_mps=2.0, z_ref="agl"),), ((4.0, 45.0), "規劃風（單層）；陣風不計")),
    ((), ((0.0, 0.0), "規劃沒有風層 ⇒ 無風")),
])
def test_planning_wind_is_the_strongest_layer(layers, want):
    ctx = types.SimpleNamespace(inputs=types.SimpleNamespace(wind=layers))
    assert SC._mapping_planning_wind(ctx) == want


def test_survey_polygons_subtract_the_no_fly_zones():
    fr = GeoFrame(23.96, 120.90, 30.0)
    sq, nf = shapely.box(-500.0, -500.0, 500.0, 500.0), shapely.box(-50.0, -50.0, 50.0, 50.0)

    def area(polys):
        rings = []
        for p in polys:
            # 換回 ENU 用框架原點高（30 m；轉出時 ENU z = 0）：高度給 0 會有 30/R 的尺度差（1e-5）
            ext = fr.to_enu_xy([a for a, _ in p["exterior"]], [b for _, b in p["exterior"]], 30.0)
            holes = [fr.to_enu_xy([a for a, _ in h], [b for _, b in h], 30.0) for h in p["holes"]]
            rings.append(shapely.Polygon(ext, holes))
        return sum(r.area for r in rings)

    polys, n = SC._mapping_survey_polygons(types.SimpleNamespace(areas_enu={"Z1": sq}, nofly_buffered=[], frame=fr))
    assert n == 0 and len(polys) == 1 and polys[0]["holes"] == []
    assert area(polys) == pytest.approx(1.0e6, rel=1e-6)
    polys, n = SC._mapping_survey_polygons(types.SimpleNamespace(areas_enu={"Z1": sq}, nofly_buffered=[nf], frame=fr))
    assert n == 1 and len(polys) == 1 and len(polys[0]["holes"]) == 1
    assert area(polys) == pytest.approx(1.0e6 - 1.0e4, rel=1e-6)


# ═══════════════════════════ 協調器：驗證壞掉 ⇒ 擋（沒驗過不能當驗過） ═══════════════════════════
def test_coordinator_blocks_when_exposure_verification_crashes(monkeypatch):
    def boom(*a, **k):
        raise RuntimeError("boom-exposure")
    monkeypatch.setattr(SC, "mapping_exposure_summary", boom)
    seen: list = []
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    c.hv_home = ll(0.0, -400.0)
    c.hv_wind = (0.0, 0.0)
    c.hv_fence_zones = []
    c.hv_on_report = seen.append
    c.harvest_params = mapping_params()
    c.plan_coverage_dccpp(areas=areas(), drones=drones(), coverage_width_m=SWATH * (1.0 - SIDE), overlap_rate=0.0,
                          coverage_altitude=ALT, mounting_angle_deg=0.0)
    assert c.hv_exposure_summary == {"ok": False, "error": "RuntimeError: boom-exposure"}
    assert c.hv_coordinator.mapping_gate()["uploadable"] is True              # 轉接層自己看不到曝光驗證
    g = c.hv_mapping_gate
    assert g["uploadable"] is False
    assert any(r.startswith("exposure_verify_failed(") and "boom-exposure" in r for r in g["reasons"])
    notes = [n for p in seen for n in (p.get("validation_notes") or [])]
    assert any(n.startswith("mapping_upload_blocked(") and "boom-exposure" in n for n in notes)
    assert any(n.startswith("mapping_exposure(exposure_verify_failed(") for n in notes)
