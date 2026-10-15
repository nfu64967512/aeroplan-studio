"""SITL S1 航測驗證：`scripts/hv_sitl_run.py` 的旋鈕解析與注入、`scripts/survey_sitl_s1.py` 的出口條件。

不起 SITL、不開任何連線。真實日誌那一組讀本機 sitl/vtol/i100–i102 的 00000019.BIN（舊匯出器飛的：
CAM1_DURATION 1.0、CAM_MAX_ROLL 0、DO_LAND_START 在最後一條開相機的航段裡），沒有就 skip；
它要顯示「舊設定該不過的不過、張數與間距該過的過」，數字對 phase-1 的量測（CAM 170/113/226、d 29.699 m）。
其餘用 survey.dflog.DFLog 直接組的合成日誌，逐項鎖住每個條件的**值**。
"""

from __future__ import annotations

import dataclasses
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from survey import datum
from survey.dflog import DFLog

REPO = Path(__file__).resolve().parents[1]
runner = pytest.importorskip("scripts.hv_sitl_run")
s1 = pytest.importorskip("scripts.survey_sitl_s1")
scenarios = pytest.importorskip("scripts.hv_sitl_scenarios")

FULL_ENV = {"HV_MAPPING": "1", "HV_CAMERA": "hq6", "HV_SHUTTER_HZ": "1.8", "HV_SIDE_OVERLAP": "0.7",
            "HV_FWD_OVERLAP": "0.8"}


# ══════════════════════════════════════════════════════════════════════
#  hv_sitl_run：旋鈕解析
# ══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("env", [{}, {"HV_MAPPING": "0"}, {"HV_MAPPING": ""}, {"HV_TRIAL": "X", "HV_SPEEDUP": "5"}])
def test_no_knob_means_none(env):
    """預設關閉：沒給（或只給 HV_MAPPING=0）⇒ None ⇒ 走舊版路徑。"""
    assert runner.survey_knobs(env) is None


def test_full_knob_set_values():
    k = runner.survey_knobs(FULL_ENV)
    assert (k.mapping, k.lead_in_m, k.camera) == (True, 50.0, "hq6")
    assert (k.side_overlap, k.fwd_overlap) == (0.7, 0.8)
    assert (k.shutter_hz, k.shutter_source) == (1.8, "HV_SHUTTER_HZ")
    assert k.cam_max_roll_deg == 15.0
    assert k.interval_min_s == 0.56                      # ceil_0.02(1/1.8 = 0.5556)
    assert k.env == FULL_ENV
    d = k.to_dict()
    assert d["fc_max_rate_hz"] == pytest.approx(1 / 0.56)
    assert len(d["notes"]) == 1 and "0.80%" in d["notes"][0]     # 1.8 × 0.56 − 1 = 0.8%
    assert d["camera_spec"] == {"focal_length_mm": 6.0, "sensor_width_mm": 4056 * 1.55e-3,
                                "sensor_height_mm": 3040 * 1.55e-3, "pixel_pitch_um": 1.55}
    cc = k.trigger_config()
    assert (cc.backend, cc.duration_s, cc.interval_min_s, cc.auto_only, cc.max_roll_deg, cc.camera_lead_in_m) == \
        ("servo", 0.1, 0.56, True, 15.0, 50.0)
    # 每帶第一張也走距離觸發（受 CAM_MAX_ROLL 擋），與 GUI 建圖模式相同（S1 有風那趟：立即拍那張在 20–25° 側傾）
    assert cc.first_shot_immediate is False and cc.firmware is None
    from hv_dccpp.mission import MappingModeConfig
    assert k.mapping_config() == MappingModeConfig(lead_in_m=50.0)


@pytest.mark.parametrize("hz, want", [(2.0, 0.5), (1.8, 0.56), (1.0, 1.0), (3.0, 0.34), (50.0, 0.02),
                                      (0.1, 10.0), (1.25, 0.8), (1.7, 0.6)])
def test_interval_min_is_ceil_to_20ms(hz, want):
    assert runner._ceil_002(1.0 / hz) == want


def test_defaults_when_only_mapping_given():
    from hv_dccpp.platform import CameraHardware
    k = runner.survey_knobs({"HV_MAPPING": "1"})
    assert k.shutter_hz == CameraHardware().max_shutter_rate_hz == 1.0
    assert k.shutter_source == "CameraHardware 預設"
    assert (k.camera, k.lead_in_m, k.interval_min_s, k.cam_max_roll_deg) == ("panel", 50.0, 1.0, 15.0)
    assert (k.side_overlap, k.fwd_overlap) == (None, None)
    assert k.camera_hw() is None and k.fov_deg() is None
    assert k.to_dict()["notes"] == []


def test_camera_only_without_mapping_has_no_lead_in():
    k = runner.survey_knobs({"HV_CAMERA": "GS8", "HV_CAM_MAX_ROLL": "0"})
    assert (k.mapping, k.lead_in_m, k.camera, k.cam_max_roll_deg) == (False, 0.0, "gs8", 0.0)
    assert k.mapping_config() is None and k.trigger_config().camera_lead_in_m == 0.0
    hw = k.camera_hw()
    assert (hw.focal_length_mm, hw.sensor_width_mm, hw.sensor_height_mm, hw.pixel_pitch_um,
            hw.max_shutter_rate_hz, hw.is_default_example) == (8.0, 1456 * 3.45e-3, 1088 * 3.45e-3, 3.45, 1.0, False)
    h, v = k.fov_deg()
    assert h == pytest.approx(math.degrees(2 * math.atan(1456 * 3.45e-3 / 16.0)))
    assert v == pytest.approx(math.degrees(2 * math.atan(1088 * 3.45e-3 / 16.0)))


@pytest.mark.parametrize("env, msg", [
    ({"HV_MAPPING": "2"}, "HV_MAPPING 只能是 0 或 1"),
    ({"HV_MAPPING": "yes"}, "HV_MAPPING 只能是 0 或 1"),
    ({"HV_LEAD_IN_M": "40"}, "只在 HV_MAPPING=1 時有作用"),
    ({"HV_MAPPING": "0", "HV_LEAD_IN_M": "40", "HV_CAMERA": "hq6"}, "只在 HV_MAPPING=1 時有作用"),
    ({"HV_MAPPING": "1", "HV_LEAD_IN_M": "-5"}, "HV_LEAD_IN_M=-5 超出範圍"),
    ({"HV_CAMERA": "sony"}, "HV_CAMERA 只能是"),
    ({"HV_SIDE_OVERLAP": "1.0"}, "HV_SIDE_OVERLAP=1.0 超出範圍 (0, 1)"),
    ({"HV_SIDE_OVERLAP": "0"}, "HV_SIDE_OVERLAP=0 超出範圍"),
    ({"HV_SIDE_OVERLAP": "abc"}, "HV_SIDE_OVERLAP 必須是數字"),
    ({"HV_FWD_OVERLAP": "nan"}, "HV_FWD_OVERLAP=nan 超出範圍"),
    ({"HV_FWD_OVERLAP": "-0.1"}, "HV_FWD_OVERLAP=-0.1 超出範圍"),
    ({"HV_SHUTTER_HZ": "0.05"}, "HV_SHUTTER_HZ=0.05 超出範圍 [0.1, 50]"),
    ({"HV_SHUTTER_HZ": "60"}, "HV_SHUTTER_HZ=60 超出範圍"),
    ({"HV_SHUTTER_HZ": "inf"}, "HV_SHUTTER_HZ=inf 超出範圍"),
    ({"HV_CAM_MAX_ROLL": "181"}, "HV_CAM_MAX_ROLL=181 超出範圍 [0, 180]"),
    ({"HV_CAM_MAX_ROLL": "-1"}, "HV_CAM_MAX_ROLL=-1 超出範圍"),
])
def test_bad_knobs_are_refused(env, msg):
    with pytest.raises(SystemExit, match=msg.replace("(", r"\(").replace(")", r"\)").replace("[", r"\[")
                       .replace("]", r"\]")):
        runner.survey_knobs(env)


def test_edges_accepted():
    k = runner.survey_knobs({"HV_SHUTTER_HZ": "0.1", "HV_CAM_MAX_ROLL": "180", "HV_MAPPING": "1",
                             "HV_LEAD_IN_M": "0"})
    assert (k.shutter_hz, k.interval_min_s, k.cam_max_roll_deg, k.lead_in_m) == (0.1, 10.0, 180.0, 0.0)
    # 觸發率上限（servo、CAM1_DURATION 0.1 s ⇒ 兩次觸發至少隔 0.14 s）：8.33 Hz 取整後 0.14 s 還行，8.34 Hz 取整成 0.12 s 就不行
    assert runner.survey_knobs({"HV_SHUTTER_HZ": "8.33"}).interval_min_s == 0.14


@pytest.mark.parametrize("hz", ["8.34", "12.5", "20", "50"])
def test_trigger_rate_above_the_servo_duration_cap_is_refused(hz):
    """R1 #7：CAM1_DURATION 0.1 s 時飛控給不出 > 7.14 Hz 的乾淨觸發；以前 HV_SHUTTER_HZ 照收、飛控靜默拉長間距。"""
    with pytest.raises(SystemExit, match=r"上限 7\.143 Hz"):
        runner.survey_knobs({"HV_SHUTTER_HZ": hz})


def test_sitl_trigger_spacing_formula_equals_the_gui_one():
    """腳本裡複製的公式與 GUI 路徑（swarm_coordinator.trigger_min_spacing_s）必須是同一個數（值鎖）。"""
    from mission.swarm_coordinator import trigger_min_spacing_s
    for d in (0.0, 0.02, 0.05, 0.1, 0.11, 0.2, 0.5, 1.0):
        assert runner._min_trigger_spacing_s(d) == trigger_min_spacing_s(d), d
    assert runner._min_trigger_spacing_s(runner.SURVEY_TRIGGER_DURATION_S) == 0.14


def test_sitl_first_flight_wind_limit_equals_the_gui_default():
    """SITL 印的 GUI 閘門風速上限與 GUI／面板的預設值要是同一個數（值鎖）。"""
    from mission.swarm_coordinator import MAPPING_MAX_WIND_DEFAULT_MPS
    assert runner.MAPPING_FIRST_FLIGHT_MAX_WIND_MPS == MAPPING_MAX_WIND_DEFAULT_MPS == 3.0


def test_cross_strip_plan_that_fails_validate_is_not_exported(monkeypatch, tmp_path):
    """交叉航帶 validate 不過 ⇒ SystemExit、不匯出（只寫 validation.json）；HV_ALLOW_CONFLICTS=1 才放行並記錄。"""
    import types
    k = runner.survey_knobs({**FULL_ENV, "HV_CROSS": "1"})
    assert k is not None and k.cross
    sc = scenarios.SCENARIOS["gui_legal_120_1a2b"]
    up = dict(strips=[1], waypoints=[], distance_m=0.0, t_takeoff_s=0.0, t_land_s=0.0, energy_used_wh=0.0,
              energy_margin_wh=0.0)
    plan = types.SimpleNamespace(plans={u: types.SimpleNamespace(cls=u[0], **up) for u in ("A1", "B2", "B3")},
                                 makespan_s=0.0, uncovered=[])
    assert len(sc.fleet) == 3
    ctx = types.SimpleNamespace(sensors={})
    hv = types.SimpleNamespace(last_plan=plan, last_ctx=ctx, cfg=None,
                               last_inputs=types.SimpleNamespace(payload_reqs={}),
                               mapping_gate=lambda **kw: {"uploadable": True, "reasons": []})
    exported = []
    fake_hvlib = types.ModuleType("hv_dccpp")
    fake_hvlib.build_validation_cache = lambda *a, **kw: None
    fake_hvlib.validate = lambda *a, **kw: types.SimpleNamespace(passed=False, conflicts=[1, 2], coverage_ok=True,
                                                                 energy_ok=True, min_sep_h_m=10.0, min_sep_v_m=0.0)
    fake_hvlib.export_all = lambda *a, **kw: exported.append(1) or {}
    monkeypatch.setitem(sys.modules, "hv_dccpp", fake_hvlib)
    monkeypatch.setattr(runner, "apply_survey_knobs", lambda sc_, k_: sc_)
    monkeypatch.setattr(runner, "out_dir", lambda sc_: tmp_path)
    monkeypatch.setattr(runner, "_plan_gui", lambda sc_, k_: (hv, {"status": "GO"}))
    monkeypatch.setattr(runner, "_plan_direct", lambda sc_: (hv, {"status": "GO"}))
    monkeypatch.delenv("HV_ALLOW_CONFLICTS", raising=False)
    with pytest.raises(SystemExit, match="validate 不過.*HV_ALLOW_CONFLICTS=1"):
        runner.plan_and_export(sc, k)
    assert exported == []
    rec = json.loads((tmp_path / "validation.json").read_text(encoding="utf-8"))
    assert rec["passed"] is False and rec["conflicts"] == 2 and rec["allow_conflicts"] is False
    assert rec["gui_mapping_gate"] == {"max_wind_mps": 3.0, "uploadable": True, "reasons": []}


# ── 情境層 ──────────────────────────────────────────────────────────

def test_apply_knobs_replaces_fov_and_side_overlap_only():
    sc = scenarios.SCENARIOS["gui_legal_120_1a2b"]
    k = runner.survey_knobs(FULL_ENV)
    sc2 = runner.apply_survey_knobs(sc, k)
    assert (sc2.hfov_deg, sc2.vfov_deg) == pytest.approx(k.fov_deg())
    assert sc2.hfov_deg == pytest.approx(55.30008601781524) and sc2.vfov_deg == pytest.approx(42.876562766137766)
    assert sc2.overlap_rate == 0.7
    changed = {f.name for f in dataclasses.fields(sc) if getattr(sc, f.name) != getattr(sc2, f.name)}
    assert changed == {"hfov_deg", "vfov_deg", "overlap_rate"}
    assert (sc.hfov_deg, sc.overlap_rate) == (60.0, 0.30)          # 原情境沒被改
    # panel 相機、不給側向重疊 ⇒ 情境原封不動
    assert runner.apply_survey_knobs(sc, runner.survey_knobs({"HV_MAPPING": "1"})) is sc


def test_direct_path_is_refused():
    sc = scenarios.SCENARIOS["lidar_rgb_1a3b"]
    with pytest.raises(SystemExit, match="只支援 path='gui'"):
        runner.apply_survey_knobs(sc, runner.survey_knobs({"HV_MAPPING": "1"}))


# ── 注入點 ──────────────────────────────────────────────────────────

def test_injection_reaches_coordinator_and_is_restored():
    import hv_dccpp.integration.aeroplan_adapter as ad
    import mission.swarm_coordinator as swc
    from hv_dccpp.mission import MappingModeConfig, PayloadRequirement
    base_coord, base_cam = ad.HvDccppCoordinator, swc._uniform_camera
    k = runner.survey_knobs(FULL_ENV)
    reqs = {"default": PayloadRequirement("default", "rgb", gsd_required_m=0.05, side_overlap=0.7,
                                          forward_overlap=0.70, h_target_agl_m=120.0),
            "lid": PayloadRequirement("lid", "lidar", density_pt_m2=20.0, side_overlap=0.25)}
    with runner._inject_survey(k):
        c = ad.HvDccppCoordinator(payload_reqs=reqs)
        assert c.payload_reqs["default"].forward_overlap == 0.8
        assert c.payload_reqs["lid"].forward_overlap == 0.70             # 只改 rgb
        assert c.mapping_mode == MappingModeConfig(lead_in_m=50.0)
        assert swc._uniform_camera(hfov_deg=60.0, vfov_deg=45.0) == k.camera_hw()
        with pytest.raises(SystemExit, match="mapping_mode"):
            ad.HvDccppCoordinator(mapping_mode=MappingModeConfig(lead_in_m=10.0))
    assert ad.HvDccppCoordinator is base_coord and swc._uniform_camera is base_cam
    # 例外時也要還原
    with pytest.raises(RuntimeError):
        with runner._inject_survey(k):
            raise RuntimeError("boom")
    assert ad.HvDccppCoordinator is base_coord and swc._uniform_camera is base_cam


def test_panel_camera_only_gets_the_shutter_rate():
    import mission.swarm_coordinator as swc
    k = runner.survey_knobs({"HV_SHUTTER_HZ": "2"})
    want = dataclasses.replace(swc._uniform_camera(hfov_deg=60.0, vfov_deg=45.0), max_shutter_rate_hz=2.0)
    with runner._inject_survey(k):
        got = swc._uniform_camera(hfov_deg=60.0, vfov_deg=45.0)
    assert got == want and got.focal_length_mm == 16.0


def test_injection_none_is_a_no_op():
    import hv_dccpp.integration.aeroplan_adapter as ad
    base = ad.HvDccppCoordinator
    with runner._inject_survey(None):
        assert ad.HvDccppCoordinator is base


# ── 規劃後讀回 ──────────────────────────────────────────────────────

def _fake_hv(k, *, fwd=0.8, side=0.7, hz=1.8, hw=None, mapping="same"):
    from hv_dccpp.mission import MappingModeConfig
    hw = hw if hw is not None else k.camera_hw()
    sensor = SimpleNamespace(hw=hw, r=SimpleNamespace(forward_overlap=fwd, side_overlap=side, shutter_rate_hz=hz),
                             footprint_len_m=lambda h: h * hw.sensor_height_mm / hw.focal_length_mm)
    mm = k.mapping_config() if mapping == "same" else mapping
    return SimpleNamespace(last_ctx=SimpleNamespace(sensors={("A", "default"): sensor, ("B", "default"): sensor}),
                           mapping_mode=mm, mapping_effective={"config": mm.to_dict() if mm else None}), MappingModeConfig


def test_survey_effective_records_values():
    k = runner.survey_knobs(FULL_ENV)
    hv, _ = _fake_hv(k)
    sc = runner.apply_survey_knobs(scenarios.SCENARIOS["gui_legal_120_1a2b"], k)
    eff = runner.survey_effective(hv, k, k.trigger_config(), sc)
    a = eff["sensors"]["A:default"]
    assert (a["forward_overlap"], a["side_overlap"], a["shutter_rate_hz"]) == (0.8, 0.7, 1.8)
    assert a["trigger_distance_m"] == pytest.approx(120.0 * 3040 * 1.55e-3 / 6.0 * 0.2)     # 18.848 m
    assert eff["mapping_mode"]["lead_in_m"] == 50.0


@pytest.mark.parametrize("kw, msg", [
    ({"fwd": 0.7}, "前向重疊 0.7 ≠ HV_FWD_OVERLAP 0.8"),
    ({"side": 0.3}, "側向重疊 0.3 ≠ HV_SIDE_OVERLAP 0.7"),
    ({"hz": 1.0}, "拍照率 1.0 Hz ≠ 1.8 Hz"),
    ({"mapping": None}, "建圖模式 None"),
])
def test_survey_effective_refuses_mismatch(kw, msg):
    k = runner.survey_knobs(FULL_ENV)
    hv, _ = _fake_hv(k, **kw)
    with pytest.raises(SystemExit, match=msg):
        runner.survey_effective(hv, k, k.trigger_config(), scenarios.SCENARIOS["gui_legal_120_1a2b"])


def test_survey_effective_refuses_wrong_camera_and_lead_in():
    from hv_dccpp.platform import CameraHardware
    k = runner.survey_knobs(FULL_ENV)
    hv, _ = _fake_hv(k, hw=CameraHardware(max_shutter_rate_hz=1.8))
    with pytest.raises(SystemExit, match="HV_CAMERA=hq6"):
        runner.survey_effective(hv, k, k.trigger_config(), scenarios.SCENARIOS["gui_legal_120_1a2b"])
    hv, _ = _fake_hv(k)
    cc = dataclasses.replace(k.trigger_config(), camera_lead_in_m=40.0)
    with pytest.raises(SystemExit, match="引入段 40.0 m ≠"):
        runner.survey_effective(hv, k, cc, scenarios.SCENARIOS["gui_legal_120_1a2b"])
    hv.last_ctx.sensors = {}
    with pytest.raises(SystemExit, match="沒有任何 rgb 相機"):
        runner.survey_effective(hv, k, k.trigger_config(), scenarios.SCENARIOS["gui_legal_120_1a2b"])


def test_record_sitl_logs(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "_instance_dir", lambda i: tmp_path / f"i{i}")
    for i, n in ((100, "21"), (101, "7")):
        (tmp_path / f"i{i}" / "logs").mkdir(parents=True)
        (tmp_path / f"i{i}" / "logs" / "LASTLOG.TXT").write_text(n + "\n", encoding="utf-8")
    (tmp_path / "survey_knobs.json").write_text(json.dumps({"knobs": {}}), encoding="utf-8")
    procs = [{"uav": "A1", "instance": 100}, {"uav": "B2", "instance": 101}]
    rows = runner._record_sitl_logs(tmp_path, procs, {"A1": 20, "B2": 7})
    assert rows["A1"]["bin"] == str(tmp_path / "i100" / "logs" / "00000021.BIN")
    assert rows["B2"]["bin"] is None                     # LASTLOG 沒變 ⇒ 這一趟沒有新日誌，不拿舊檔充數
    assert json.loads((tmp_path / "survey_knobs.json").read_text(encoding="utf-8"))["sitl_logs"] == rows
    assert runner._lastlog(102) is None


# ══════════════════════════════════════════════════════════════════════
#  survey_sitl_s1：合成日誌逐項
# ══════════════════════════════════════════════════════════════════════
LAT0, LON0, V, D = 23.95, 120.90, 20.0, 20.0
M0, _N0 = datum.wgs84_radii(LAT0)


def _lat(north_m):
    return LAT0 + np.degrees(np.asarray(north_m, float) / M0)


def _mission(land_start_ok=True, d=D):
    """seq: 0 home、1 VTOL 起飛、2 NAV、3 206(d,0,1)、4 NAV、5 206(0)、6 189、7 206(0)|NAV、8 85。"""
    rows = [(0, 16, 0.0, 100.0), (1, 84, 0.0, 60.0), (2, 16, 0.0, 100.0), (3, 206, d, 0.0), (4, 16, 0.0, 100.0),
            (5, 206, 0.0, 0.0), (6, 189, 0.0, 0.0), (7, 206 if land_start_ok else 16, 0.0, 100.0), (8, 85, 0.0, 0.0)]
    return rows


def _fake_log(*, cam_t=None, cam_roll=None, params=None, messages=None, mode=None, land_start_ok=True,
              cam_north=None, t_on=20.0, t_off=60.5, extra_dumps=(), v=V, d=D, t_end=100.0, t_land=80.0,
              att=None, dump=True):
    """北飛 v m/s（預設 20），10–t_end s；206 開 t_on、關 t_off；85 在 t_land。距離觸發每 1 s（d = 20 m）。

    ``att``：(TimeUS 秒陣列, Roll 度陣列) 給了才有 ATT 表；``dump=False``：沒有 CMD 傾印。
    """
    if cam_t is None:
        cam_t = np.arange(t_on, t_off, 1.0)
    cam_t = np.asarray(cam_t, float)
    north = v * (cam_t - 10.0) if cam_north is None else np.asarray(cam_north, float)
    roll = np.zeros(cam_t.size) if cam_roll is None else np.asarray(cam_roll, float)
    us = lambda s: (np.asarray(s, float) * 1e6).astype(np.int64)      # noqa: E731
    pos_t = np.arange(10.0, t_end + 1e-9, 0.2)
    mis = _mission(land_start_ok, d=d)
    dumps = ([(5.0, mis)] if dump else []) + list(extra_dumps)
    cmd = {c: [] for c in ("TimeUS", "CTot", "CNum", "CId", "Prm1", "Prm2", "Prm3", "Prm4", "Lat", "Lng", "Alt")}
    for td, rows in dumps:
        for seq, cid, p1, alt in rows:
            for c, x in zip(cmd, (int(td * 1e6), len(rows), seq, cid, p1, 0.0, 1.0 if cid == 206 and p1 else 0.0,
                                  0.0, LAT0, LON0, alt)):
                cmd[c].append(x)
    mise_rows = [(t_on, 3, 206, d), (t_off, 5, 206, 0.0), (t_land, 8, 85, 0.0)]
    tables = {
        "CAM": {"TimeUS": us(cam_t), "I": np.zeros(cam_t.size, int), "Img": np.arange(1, cam_t.size + 1),
                "Lat": _lat(north), "Lng": np.full(cam_t.size, LON0), "R": roll},
        "POS": {"TimeUS": us(pos_t), "Lat": _lat(v * (pos_t - 10.0)), "Lng": np.full(pos_t.size, LON0)},
        "CMD": {c: np.asarray(x) for c, x in cmd.items()},
        "MISE": {"TimeUS": us([r[0] for r in mise_rows]), "CNum": np.array([r[1] for r in mise_rows]),
                 "CId": np.array([r[2] for r in mise_rows]), "Prm1": np.array([r[3] for r in mise_rows], float)},
        "MODE": {"TimeUS": us([m[0] for m in (mode or [(1.0, 10)])]),
                 "ModeNum": np.array([m[1] for m in (mode or [(1.0, 10)])])},
    }
    if not tables["CMD"]["TimeUS"].size:
        del tables["CMD"]
    if att is not None:
        tables["ATT"] = {"TimeUS": us(att[0]), "Roll": np.asarray(att[1], float)}
    p = {"CAM1_TYPE": 1.0, "CAM1_DURATION": 0.1, "CAM1_INTRVAL_MIN": 0.5, "CAM_AUTO_ONLY": 1.0,
         "CAM_MAX_ROLL": 15.0}
    p.update(params or {})
    msgs = [(int(1e6), "ArduPlane V4.8.0-dev (85d417ee)"), (int(15e6), "Transition done")] + list(messages or [])
    return DFLog(path="synthetic", tables=tables, formats={}, angle_encoding={}, params=p, param_changes=[],
                 messages=msgs, firmware="ArduPlane V4.8.0-dev (85d417ee)")


def _crit(rep, key):
    return next(c for c in rep["criteria"] if c["key"] == key)


def test_clean_synthetic_sortie_passes_every_fc_criterion():
    rep = s1.analyze_fc(_fake_log())
    c = _crit(rep, "cam_count")
    st = c["detail"]["strips"][0]
    assert st["track_m"] == pytest.approx(40.5 * V, rel=1e-6)        # POS 內插端點：(60.5 − 20) × 20
    assert (st["expected"], st["n_cam"], st["diff"]) == (41, 41, 0)
    assert (st["expected_range"], st["expected_naive"]) == ([41, 41], 41)
    assert st["model"]["immediate"] is True and st["model"]["roll_gate_modelled"] is False   # 沒有 ATT
    assert c["passed"] and c["value"] == 0 and c["detail"]["n_on_in_mission"] == 1
    for key in ("spacing_median", "spacing_p99", "outside_windows", "transition_landing", "roll_share",
                "min_interval", "land_start_then_off", "cam_params"):
        assert _crit(rep, key)["passed"] is True, key
    assert _crit(rep, "spacing_median")["value"] == pytest.approx(0.0, abs=1e-6)
    assert _crit(rep, "spacing_p99")["value"] == pytest.approx(1.0, abs=1e-5)
    assert _crit(rep, "min_interval")["value"] == pytest.approx(1.0)
    assert _crit(rep, "min_interval")["threshold"] == pytest.approx(0.499)
    assert _crit(rep, "mission_readback")["passed"] is None          # 沒給匯出檔 ⇒ 未評估
    assert rep["summary"]["n_cam"] == 41 and rep["summary"]["trigg_dist_m"] == [D]


def test_count_off_by_two_fails_and_by_one_passes():
    t = np.arange(20.0, 60.5, 1.0)
    rep = s1.analyze_fc(_fake_log(cam_t=t[:-1]))
    assert _crit(rep, "cam_count")["passed"] and _crit(rep, "cam_count")["value"] == 1
    rep = s1.analyze_fc(_fake_log(cam_t=t[:-2]))
    assert not _crit(rep, "cam_count")["passed"] and _crit(rep, "cam_count")["value"] == 2


def test_missing_window_fails_count():
    """任務有兩個 206 開啟、日誌只有一條時間窗 ⇒ 一整條帶沒拍（張數對不上任務）。"""
    rows = _mission() + [(9, 206, D, 0.0)]
    log = _fake_log(extra_dumps=[(6.0, rows)])
    rep = s1.analyze_fc(log)
    c = _crit(rep, "cam_count")
    assert (c["detail"]["n_windows"], c["detail"]["n_on_in_mission"]) == (1, 2) and not c["passed"]


# ── 張數期望值：飛控 50 Hz tick 的距離觸發（R2：floor(L/d)+1 忽略 tick，HERO 長帶少算） ──────────
HERO_V, HERO_D = 19.0, 18.848       # 0.38 m/tick ⇒ 第 ceil(18.848/0.38) = 50 個 tick 才到 d ⇒ 實際間距 19.0 m


def _perfect_strip(length_m, *, v=HERO_V, d=HERO_D, t_on=20.0):
    """完美的飛控：206 開啟當下立刻拍，之後每 ceil(d/(v·0.02)) 個 50 Hz tick 一張（等速直線）。"""
    step_s = math.ceil(d / (v * 0.02)) * 0.02
    t_off = t_on + length_m / v
    cam_t = t_on + step_s * np.arange(int((t_off - t_on) / step_s) + 2)
    cam_t = cam_t[cam_t < t_off]
    return _fake_log(cam_t=cam_t, v=v, d=d, t_on=t_on, t_off=t_off, t_end=t_off + 30.0, t_land=t_off + 10.0)


def test_cam_count_expectation_follows_the_50hz_tick_not_floor_l_over_d():
    """R2 的 HERO 帶（1152.2 m、19 m/s、d 18.848 m）：完美飛行 61 張，floor(L/d)+1 = 62 ⇒ 舊判準差 −1（卡在 ±1 邊緣）。"""
    rep = s1.analyze_fc(_perfect_strip(1152.2))
    c = _crit(rep, "cam_count")
    st = c["detail"]["strips"][0]
    assert st["track_m"] == pytest.approx(1152.2, abs=1e-3)          # t_off 以 µs 截斷
    assert (st["n_cam"], st["expected_naive"]) == (61, 62)
    assert (st["expected"], st["expected_range"], st["diff"]) == (61, [61, 61], 0)
    assert c["passed"] is True and c["value"] == 0
    assert c["detail"]["expected_total"] == 61 and c["detail"]["expected_total_naive"] == 62
    m = st["model"]
    assert m["immediate"] is True and m["intrval_min_s"] == 0.5 and m["duration_s"] == 0.1
    assert m["spacing_median_m"] == pytest.approx(19.0, abs=1e-4)    # d + (50·0.38 − d)：tick 量化後的間距
    sp = _crit(rep, "spacing_median")["detail"]["tick_model_spacing_m"]
    assert sp == {"1": pytest.approx(19.0, abs=1e-4)}


def test_long_perfect_strip_passes_and_missing_photos_still_fail():
    """4.8 km：完美飛控 253 張，floor(L/d)+1 = 255 ⇒ 舊判準差 −2、把正確的飛行判成未通過。"""
    log = _perfect_strip(4800.0)
    c = _crit(s1.analyze_fc(log), "cam_count")
    st = c["detail"]["strips"][0]
    assert (st["n_cam"], st["expected_naive"], st["expected"], st["diff"]) == (253, 255, 253, 0)
    assert c["passed"] is True
    cam = log.tables["CAM"]                                          # 判準照樣咬得到：少兩張 ⇒ 差 −2
    keep = np.ones(cam["TimeUS"].size, bool)
    keep[[100, 200]] = False
    for k in list(cam):
        cam[k] = cam[k][keep]
    c = _crit(s1.analyze_fc(log), "cam_count")
    assert (c["detail"]["strips"][0]["diff"], c["value"], c["passed"]) == (-2, 2, False)
    cam["TimeUS"] = np.r_[cam["TimeUS"], [int(21.5e6), int(22.5e6), int(23.5e6)]]     # 多三張 ⇒ +1
    for k in ("I", "Img", "Lat", "Lng", "R"):
        cam[k] = np.r_[cam[k], cam[k][:3]]
    c = _crit(s1.analyze_fc(log), "cam_count")
    assert (c["detail"]["strips"][0]["diff"], c["passed"]) == (1, True)


def test_roll_gate_from_att_delays_the_distance_trigger():
    """CAM_MAX_ROLL 15°、30.5–31.5 s |roll| 20° ⇒ 31.0 s 那一張等到 31.52 s，之後全部後移 ⇒ 40 張（不是 41）。"""
    att_t = np.arange(10.0, 100.0, 0.02)
    roll = np.where((att_t > 30.49) & (att_t < 31.51), 20.0, 0.0)
    cam_t = np.r_[np.arange(20.0, 30.5, 1.0), np.arange(31.52, 60.5, 1.0)]
    assert cam_t.size == 40
    st = _crit(s1.analyze_fc(_fake_log(cam_t=cam_t, att=(att_t, roll))), "cam_count")["detail"]["strips"][0]
    assert (st["expected"], st["expected_range"], st["diff"]) == (40, [40, 40], 0)
    assert st["model"]["roll_gate_modelled"] is True and st["model"]["cam_max_roll_deg"] == 15.0
    # 沒有 ATT ⇒ 無法建模側傾閘門，照實標示（期望值 41，差 −1）
    st = _crit(s1.analyze_fc(_fake_log(cam_t=cam_t)), "cam_count")["detail"]["strips"][0]
    assert (st["expected"], st["diff"], st["model"]["roll_gate_modelled"]) == (41, -1, False)
    # CAM_MAX_ROLL = 0 ⇒ 閘門關閉 ⇒ 不受 ATT 影響
    st = _crit(s1.analyze_fc(_fake_log(cam_t=cam_t, att=(att_t, roll), params={"CAM_MAX_ROLL": 0.0})),
               "cam_count")["detail"]["strips"][0]
    assert (st["expected"], st["model"]["roll_gate_modelled"]) == (41, False)


def test_intrval_min_pending_shot_is_modelled():
    """CAM1_INTRVAL_MIN 1.5 s、距離每 1 s 就到 ⇒ 每張都 pending 到 1.5 s ⇒ 20, 21.5, …, 59.0 共 27 張。"""
    cam_t = 20.0 + 1.5 * np.arange(27)
    rep = s1.analyze_fc(_fake_log(cam_t=cam_t, params={"CAM1_INTRVAL_MIN": 1.5}))
    st = _crit(rep, "cam_count")["detail"]["strips"][0]
    assert (st["expected"], st["expected_naive"], st["diff"]) == (27, 41, 0)
    assert st["model"]["intrval_min_s"] == 1.5


def test_pending_shot_is_not_blocked_by_the_roll_gate():
    """AP_Camera_Backend::update 先補拍 pending、之後才查 CAM_MAX_ROLL：21.0 s 距離到了但 INTRVAL_MIN 1.5 s
    ⇒ pending；21.2–21.6 s |roll| 20° 也照樣在 21.5 s 拍 ⇒ 20 + 1.5k 到 59.0 共 27 張（帶在 59.1 s 關）。
    若 pending 被側傾擋住，就要等到 21.62 s 才由距離觸發，之後全部後移 0.12 s ⇒ 只剩 26 張。"""
    att_t = np.arange(10.0, 100.0, 0.02)
    roll = np.where((att_t > 21.19) & (att_t < 21.61), 20.0, 0.0)
    cam_t = 20.0 + 1.5 * np.arange(27)
    rep = s1.analyze_fc(_fake_log(cam_t=cam_t, t_off=59.1, att=(att_t, roll), params={"CAM1_INTRVAL_MIN": 1.5}))
    st = _crit(rep, "cam_count")["detail"]["strips"][0]
    assert (st["n_cam"], st["expected"], st["expected_range"], st["diff"]) == (27, 27, [27, 27], 0)
    assert st["model"]["roll_gate_modelled"] is True


def test_unknown_first_shot_semantics_gives_a_range():
    """沒有 CMD 傾印 ⇒ 不知道 206 p3（立即拍）⇒ 兩種都算、取範圍 [40, 41]，不替飛控猜。"""
    c = _crit(s1.analyze_fc(_fake_log(dump=False)), "cam_count")
    st = c["detail"]["strips"][0]
    assert st["model"]["immediate"] is None and st["expected_range"] == [40, 41] and st["diff"] == 0
    assert c["passed"] is False and c["detail"]["n_on_in_mission"] == 0      # 任務對不上，照樣不通過


def test_replay_matches_ardupilot_semantics_directly():
    """重播本身：206 立即拍＋50 Hz 距離觸發；p3=0 時第一個 tick 只記位置。"""
    log = _perfect_strip(1152.2)
    pos = log.table("POS")
    t_on, t_off = int(20e6), int(round((20.0 + 1152.2 / HERO_V) * 1e6))
    shots = s1.replay_distance_trigger(pos, t_on, t_off, HERO_D, immediate=True)
    t = np.asarray([s[0] for s in shots])
    assert t.size == 61 and t[0] == t_on and np.all(np.diff(t) == 1_000_000)
    shots = s1.replay_distance_trigger(pos, t_on, t_off, HERO_D, immediate=False, phase_us=8_000)
    assert len(shots) == 60 and shots[0][0] == t_on + 8_000 + 1_000_000
    with pytest.raises(ValueError, match="phase_us"):
        s1.replay_distance_trigger(pos, t_on, t_off, HERO_D, immediate=True, phase_us=20_000)
    with pytest.raises(ValueError, match="d_m"):
        s1.replay_distance_trigger(pos, t_on, t_off, 0.0, immediate=True)


def test_analyze_hero_speed_synthetic_bin_counts_match_tick_model(tmp_path):
    """R2 重現（以 E1 的 .BIN 產生器生成，不讀 scratch 檔）：HERO 19 m/s、d 18.848 m、1152.2 m 的帶。"""
    from tests.test_survey_synthlog import SurveyTruth, write_survey_log
    info = write_survey_log(tmp_path / "h.BIN", SurveyTruth(n_strips=2, strip_len=1152.2, speed=19.0), d=18.848)
    assert len(info["cam_t_us"]) == 122
    rep = s1.analyze(bins={"B2": str(tmp_path / "h.BIN")}, out=tmp_path / "out", run_verifier=False)
    c = _crit(rep["uavs"]["B2"], "cam_count")
    strips = c["detail"]["strips"]
    assert [s["n_cam"] for s in strips] == [61, 61]
    assert [s["expected_naive"] for s in strips] == [62, 62]
    assert [(s["expected"], s["diff"]) for s in strips] == [(61, 0), (61, 0)]
    assert c["passed"] is True


def test_spacing_median_is_judged_against_the_50hz_tick_model_not_d():
    """S1 3 m/s：HERO 順風 21.9 m/s ⇒ 0.438 m/tick，d/(v·0.02) = 43.03 ⇒ 第 44 個 tick 才拍 ⇒ 間距 19.27 m（+2.3%）。
    飛控完全照 50 Hz 語意飛（與重播模型差 ≤ 1 cm），舊判準對 d 比 ⇒ 誤判未通過；改對重播模型比 ⇒ 通過。"""
    v = 21.9
    rep = s1.analyze_fc(_perfect_strip(1152.2, v=v))
    c = _crit(rep, "spacing_median")
    st = c["detail"]["strips"][0]
    assert st["median_m"] == pytest.approx(44 * v * 0.02, abs=1e-3)               # 19.272 m
    assert st["median_m"] / HERO_D - 1.0 > 0.02                                    # 對 d 偏 2.3%（超過 2%）
    assert st["median_ref"] == "tick_model" and st["median_ref_m"] == pytest.approx(19.272, abs=1e-3)
    assert c["passed"] is True and c["value"] == pytest.approx(0.0, abs=1e-4)
    assert c["detail"]["rel_to_d_max"] == pytest.approx(44 * v * 0.02 / HERO_D - 1.0, abs=1e-4)


def test_spacing_median_and_p99():
    t = np.arange(20.0, 60.5, 1.0)
    north = V * (t - 10.0)
    north_far = north * 1.03 - 0.03 * north[0]                          # 每段 20.6 m（+3%）
    rep = s1.analyze_fc(_fake_log(cam_t=t, cam_north=north_far))
    c = _crit(rep, "spacing_median")
    assert c["value"] == pytest.approx(0.03, abs=1e-5) and not c["passed"]
    gap = north.copy()
    gap[20:] += 6.0                                                   # 兩段 26 m = 1.3 d
    gap[30:] += 6.0
    rep = s1.analyze_fc(_fake_log(cam_t=t, cam_north=gap))
    assert _crit(rep, "spacing_median")["passed"]
    c = _crit(rep, "spacing_p99")
    assert c["value"] == pytest.approx(np.percentile(np.r_[np.full(38, 20.0), 26.0, 26.0], 99) / 20.0, abs=1e-5)
    assert c["value"] > 1.2 and not c["passed"]


def test_photo_outside_window_and_in_landing():
    t = np.r_[np.arange(20.0, 60.5, 1.0), 70.0]                      # 70 s：206 關了之後
    rep = s1.analyze_fc(_fake_log(cam_t=t))
    c = _crit(rep, "outside_windows")
    assert c["value"] == 1 and c["detail"]["t_us"] == [70_000_000] and not c["passed"]
    assert _crit(rep, "transition_landing")["passed"]                 # 70 s 還沒進降落
    t = np.r_[np.arange(20.0, 60.5, 1.0), 85.0, 90.0]                 # 85 在 80 s 開始執行
    rep = s1.analyze_fc(_fake_log(cam_t=t))
    c = _crit(rep, "transition_landing")
    assert c["value"] == 2 and c["detail"]["by_phase"] == {"landing": 2} and not c["passed"]


def test_photo_during_takeoff_transition_and_non_auto():
    t = np.r_[12.0, np.arange(20.0, 60.5, 1.0)]                       # 12 s：Transition done（15 s）之前
    rep = s1.analyze_fc(_fake_log(cam_t=t))
    assert _crit(rep, "transition_landing")["detail"]["by_phase"] == {"takeoff_transition": 1}
    # 中途切 RTL（11）30–35 s
    rep = s1.analyze_fc(_fake_log(mode=[(1.0, 10), (30.0, 11), (35.0, 10)]))
    c = _crit(rep, "transition_landing")
    assert c["value"] == 5 and c["detail"]["by_phase"] == {"mode_11": 5}
    # 第二次轉換 40–42 s（不含 42）
    rep = s1.analyze_fc(_fake_log(messages=[(int(40e6), "Transition started airspeed 5"),
                                            (int(42e6), "Transition done")]))
    assert _crit(rep, "transition_landing")["detail"]["by_phase"] == {"transition": 2}
    # 沒有 Transition done 訊息 ⇒ 全程視為未完成轉換
    log = _fake_log()
    log.messages[:] = [m for m in log.messages if m[1] != "Transition done"]
    c = _crit(s1.analyze_fc(log), "transition_landing")
    assert c["value"] == 41 and c["detail"]["by_phase"] == {"no_transition_done_msg": 41}


def test_landing_detected_from_text_without_cmd_lookup():
    log = _fake_log(messages=[(int(50e6), "Mission: 8 VTOLLand")])
    phases = s1.phase_intervals(log)
    assert min(a for a, _b, lab in phases if lab == "landing") == 50_000_000


def test_roll_share_excludes_first_shot_and_gate():
    roll = np.zeros(41)
    roll[0] = 25.0                                                    # 第一張（p3=1）不計
    roll[5] = 11.0                                                    # 1 / 40 = 2.5% > 1%
    rep = s1.analyze_fc(_fake_log(cam_roll=roll))
    c = _crit(rep, "roll_share")
    assert (c["value"], c["detail"]["n"], c["detail"]["n_gt"]) == (1 / 40, 40, 1) and not c["passed"]
    assert c["detail"]["n_over_cam_max_roll"] == 0
    roll[5] = 0.0
    assert _crit(s1.analyze_fc(_fake_log(cam_roll=roll)), "roll_share")["passed"]
    # 超過 CAM_MAX_ROLL 的距離觸發 ⇒ 閘門沒作用，即使比例 ≤ 1% 也不過
    roll = np.zeros(41)
    roll[7] = 16.0
    rep = s1.analyze_fc(_fake_log(cam_roll=roll, cam_t=None, params={}))
    c = _crit(rep, "roll_share")
    big = s1.analyze_fc(_fake_log(cam_roll=np.r_[roll, np.zeros(100)],
                                  cam_t=np.r_[np.arange(20.0, 60.5, 1.0), np.arange(20.25, 45.25, 0.25)]))
    assert c["detail"]["n_over_cam_max_roll"] == 1 and not c["passed"]
    cb = _crit(big, "roll_share")
    assert cb["value"] <= 0.01 and cb["detail"]["n_over_cam_max_roll"] == 1 and not cb["passed"]
    # CAM_MAX_ROLL = 0 ⇒ 不檢查閘門
    rep = s1.analyze_fc(_fake_log(cam_roll=np.r_[roll, np.zeros(100)],
                                  cam_t=np.r_[np.arange(20.0, 60.5, 1.0), np.arange(20.25, 45.25, 0.25)],
                                  params={"CAM_MAX_ROLL": 0.0}))
    assert _crit(rep, "roll_share")["detail"]["n_over_cam_max_roll"] is None
    assert _crit(rep, "roll_share")["passed"]


def test_min_interval_against_fc_param():
    t = np.r_[np.arange(20.0, 40.0, 1.0), 40.0, 40.4, np.arange(41.0, 60.5, 1.0)]
    rep = s1.analyze_fc(_fake_log(cam_t=t))
    c = _crit(rep, "min_interval")
    assert c["value"] == pytest.approx(0.4) and c["threshold"] == pytest.approx(0.499) and not c["passed"]
    rep = s1.analyze_fc(_fake_log(cam_t=t, params={"CAM1_INTRVAL_MIN": 0.4}))
    assert _crit(rep, "min_interval")["passed"]                      # 0.4 ≥ 0.4 − 1 ms
    rep = s1.analyze_fc(_fake_log(cam_t=t, params={"CAM1_INTRVAL_MIN": 0.402}))
    assert not _crit(rep, "min_interval")["passed"]


def test_land_start_check_on_fc_mission():
    rep = s1.analyze_fc(_fake_log(land_start_ok=False))
    c = _crit(rep, "land_start_then_off")
    assert c["value"] == [6] and not c["passed"]
    rows = [{"seq": 0, "command": 16, "p1": 0.0}, {"seq": 1, "command": 189, "p1": 0.0},
            {"seq": 2, "command": 206, "p1": 5.0}]
    assert s1.land_start_offenders(rows) == [1]                      # 206 但不是 206(0)
    assert s1.land_start_offenders(rows[:2] + [{"seq": 2, "command": 206, "p1": 0.0}]) == []
    assert s1.land_start_offenders([{"seq": 0, "command": 206, "p1": 5.0},
                                    {"seq": 1, "command": 189, "p1": 0.0}]) == [1]     # 在最後
    assert s1.land_start_offenders([{"seq": 0, "command": 189, "p1": 0.0}]) is None     # 沒用相機


def test_fc_mission_picks_dump_before_first_on():
    later = [(0, 16, 0.0, 0.0), (1, 206, 99.0, 0.0)]
    log = _fake_log(extra_dumps=[(30.0, later)])                      # 30 s 的傾印在 206 開（20 s）之後
    m = s1.fc_mission(log)
    assert len(m) == 9 and m[3]["command"] == 206 and m[3]["p1"] == D and m[3]["ctot"] == 9


def test_cam_params_s1_rules_and_exact_expectation():
    rep = s1.analyze_fc(_fake_log(params={"CAM1_DURATION": 1.0, "CAM_AUTO_ONLY": 0.0, "CAM_MAX_ROLL": 0.0,
                                          "CAM1_INTRVAL_MIN": 0.0}))
    c = _crit(rep, "cam_params")
    assert not c["passed"] and c["detail"]["mismatched"] == ["CAM1_DURATION", "CAM1_INTRVAL_MIN", "CAM_AUTO_ONLY",
                                                             "CAM_MAX_ROLL"]
    exp = {"backend": "servo", "duration_s": 0.1, "interval_min_s": 0.5, "auto_only": True, "max_roll_deg": 15.0}
    assert s1.analyze_fc(_fake_log(), expect_cam=exp)["criteria"][-2]["passed"] is True
    c = _crit(s1.analyze_fc(_fake_log(params={"CAM1_INTRVAL_MIN": 0.56}), expect_cam=exp), "cam_params")
    assert c["detail"]["mismatched"] == ["CAM1_INTRVAL_MIN"] and c["detail"]["source"] == "survey_knobs.json"
    c = _crit(s1.analyze_fc(_fake_log(params={"CAM_MAX_ROLL": 10.0}), expect_cam=exp), "cam_params")
    assert c["detail"]["mismatched"] == ["CAM_MAX_ROLL"]


def test_mission_readback_float32_bit_identity():
    def items(p1):
        return [SimpleNamespace(seq=s, command=c, param1=(p1 if c == 206 and p else p)) for s, c, p, _a in _mission()]
    assert _crit(s1.analyze_fc(_fake_log(), exported_items=items(D)), "mission_readback")["passed"]
    c = _crit(s1.analyze_fc(_fake_log(), exported_items=items(20.000002)), "mission_readback")
    assert c["passed"] is False and c["value"] == 1 and "206 p1" in c["detail"]["mismatches"][0]
    # float32 相同的寫法（匯出檔小數 8 位）仍算一致
    assert _crit(s1.analyze_fc(_fake_log(), exported_items=items(float(np.float32(D)) + 1e-7)),
                 "mission_readback")["passed"]
    c = _crit(s1.analyze_fc(_fake_log(), exported_items=items(D)[:-1]), "mission_readback")
    assert not c["passed"] and "項數" in c["detail"]["mismatches"][0]


def test_track_length_interpolates_endpoints():
    log = _fake_log()
    assert s1.track_length_m(log.table("POS"), 30_100_000, 30_900_000) == pytest.approx(16.0, rel=1e-6)
    assert s1.track_length_m(log.table("POS"), 20_000_000, 60_500_000) == pytest.approx(810.0, rel=1e-6)


def test_parse_bins_and_out_guard(tmp_path):
    assert s1._parse_bins(["A1=x.BIN", " B2 = y.BIN "]) == {"A1": "x.BIN", "B2": "y.BIN"}
    for bad in (["A1"], ["=x"], ["A1="]):
        with pytest.raises(SystemExit, match="UAV=路徑"):
            s1._parse_bins(bad)
    target = s1.ROOT / "data" / "s1_should_not_exist"
    with pytest.raises(SystemExit, match="data/"):
        s1.analyze(bins={"A1": "nope.BIN"}, out=target)
    assert not target.exists()
    with pytest.raises(SystemExit, match="data/"):
        s1._check_out(s1.ROOT / "data")
    assert s1._check_out(tmp_path / "x") == (tmp_path / "x").resolve()


def test_resolve_inputs(tmp_path):
    run = tmp_path / "run"
    (run / "plan").mkdir(parents=True)
    for u in ("A1", "B2"):
        (run / "plan" / f"{u}.waypoints").write_text("QGC WPL 110\n", encoding="utf-8")
    with pytest.raises(SystemExit, match="沒有任何 .BIN"):
        s1.resolve_inputs(run, {})
    b = tmp_path / "a.BIN"
    b.write_bytes(b"")
    (run / "survey_knobs.json").write_text(json.dumps({"sitl_logs": {"A1": {"bin": str(b)}, "B2": {"bin": None}}}),
                                           encoding="utf-8")
    inp = s1.resolve_inputs(run, {})
    assert inp["bins"] == {"A1": b} and inp["missing_bin"] == ["B2"]
    with pytest.raises(SystemExit, match="不存在"):
        s1.resolve_inputs(run, {"B2": str(tmp_path / "nope.BIN")})
    with pytest.raises(SystemExit, match="找不到產物目錄"):
        s1.resolve_inputs(tmp_path / "missing", {})


def test_cli_requires_input():
    with pytest.raises(SystemExit):
        s1.main([])


def test_markdown_escapes_pipes():
    rep = {"run_dir": None, "passed": False, "missing_bin": [], "thresholds": {}, "synth": {}, "exposure": {},
           "uavs": {"A1": {"bin": "x", "passed": False, "summary": {"firmware": "fw", "n_cam": 1, "n_trig": 0,
                                                                     "n_windows": 1, "trigg_dist_m": [20.0]},
                           "criteria": [s1._crit("roll_share", "a |roll| b", 0.5, 0.01, False)]}}}
    md = s1.render_markdown(rep)
    assert r"| a \|roll\| b | 0.5 | 0.01 | **未通過** |" in md


def test_analyze_end_to_end_on_synthetic_bin(tmp_path):
    """不靠本機 SITL 日誌：E1 的合成 .BIN（已知真值）走完整條 analyze()。"""
    from tests.test_survey_synthlog import SurveyTruth, write_survey_log
    info = write_survey_log(tmp_path / "s.BIN", SurveyTruth(n_strips=3, strip_len=1500.0))
    n = len(info["cam_t_us"])
    rep = s1.analyze(bins={"U1": str(tmp_path / "s.BIN")}, out=tmp_path / "out")
    u = rep["uavs"]["U1"]
    assert u["summary"]["n_cam"] == n and u["summary"]["n_windows"] == 3
    for key in ("cam_count", "spacing_median", "spacing_p99", "outside_windows", "land_start_then_off",
                "ingest_recovery"):
        assert _crit(u, key)["passed"] is True, (key, _crit(u, key))
    # 合成日誌沒有「Transition done」⇒ 不知道何時進入定翼 ⇒ 全部算在轉換中（不放行）
    assert _crit(u, "transition_landing")["detail"]["by_phase"] == {"no_transition_done_msg": n}
    assert _crit(u, "min_interval")["passed"] is None                # 日誌沒有 CAM1_INTRVAL_MIN
    assert _crit(u, "cam_params")["detail"]["mismatched"] == ["CAM1_INTRVAL_MIN", "CAM_AUTO_ONLY", "CAM_MAX_ROLL"]
    assert u["not_evaluated"] == ["min_interval", "mission_readback"]
    ing = u["ingest"]
    assert ing["recovered"]["unmatched_fc"] == ing["injected"]["drop_index"]
    assert ing["n_poses"] == n - 2 - 1 + 1
    assert rep["exposure"]["skipped"].startswith("沒有測區")
    assert rep["passed"] is False and rep["missing_bin"] == []
    saved = json.loads((tmp_path / "out" / "s1_report.json").read_text(encoding="utf-8"))
    assert _crit(saved["uavs"]["U1"], "spacing_median")["detail"]["strips"][0]["median_ok"] is True   # 不是字串


def test_too_few_photos_is_recorded_not_fatal(tmp_path):
    """CAM 太少、注入不了故障 ⇒ ingest 這一項記為未通過，其他條件照算，不中斷。"""
    from tests.test_survey_synthlog import SurveyTruth, write_survey_log
    write_survey_log(tmp_path / "s.BIN", SurveyTruth(n_strips=1, strip_len=60.0, lead=40.0))
    rep = s1.analyze(bins={"U1": str(tmp_path / "s.BIN")}, out=tmp_path / "out", run_verifier=False)
    c = _crit(rep["uavs"]["U1"], "ingest_recovery")
    assert c["passed"] is False and "不夠注入" in c["detail"]["error"]
    assert _crit(rep["uavs"]["U1"], "cam_count")["passed"] is not None
    assert rep["exposure"] == {"skipped": "run_verifier=False"}


# ══════════════════════════════════════════════════════════════════════
#  survey_sitl_s1：本機真實 SITL 日誌（舊匯出器飛的那三架）
# ══════════════════════════════════════════════════════════════════════
REAL = {"A1": REPO / "sitl/vtol/i100/logs/00000019.BIN", "B2": REPO / "sitl/vtol/i101/logs/00000019.BIN",
        "B3": REPO / "sitl/vtol/i102/logs/00000019.BIN"}
RUN_DIR = REPO / "runs/artifacts/HV4W55S"          # 同一趟（log 19 = HV4W55S，gitignored；沒有就只用日誌）


@pytest.fixture(scope="module")
def real_report(tmp_path_factory):
    if not all(p.exists() for p in REAL.values()):
        pytest.skip("本機沒有 SITL 日誌 sitl/vtol/i100–i102/logs/00000019.BIN")
    out = tmp_path_factory.mktemp("s1")
    run_dir = RUN_DIR if (RUN_DIR / "plan" / "A1.waypoints").exists() else None
    rep = s1.analyze(run_dir=run_dir, bins={u: str(p) for u, p in REAL.items()}, out=out)
    return rep, out, run_dir


def test_real_counts_and_trigger_distance(real_report):
    rep, _out, _r = real_report
    assert {u: rep["uavs"][u]["summary"]["n_cam"] for u in REAL} == {"A1": 170, "B2": 113, "B3": 226}
    assert {u: rep["uavs"][u]["summary"]["n_windows"] for u in REAL} == {"A1": 3, "B2": 2, "B3": 4}
    a1 = _crit(rep["uavs"]["A1"], "cam_count")
    assert all(s["d_m"] == pytest.approx(29.699, abs=5e-4) for s in a1["detail"]["strips"])
    assert [s["n_cam"] for s in a1["detail"]["strips"]] == [56, 57, 57]
    # 真的 ArduPlane SITL：tick 重播的期望值逐帶完全相同；floor(L/d)+1 有兩帶多算一張
    assert [s["expected"] for s in a1["detail"]["strips"]] == [56, 57, 57]
    assert [s["expected_naive"] for s in a1["detail"]["strips"]] == [57, 57, 58]
    for u in REAL:
        strips = _crit(rep["uavs"][u], "cam_count")["detail"]["strips"]
        assert [s["diff"] for s in strips] == [0] * len(strips), u
        assert all(s["expected_range"][0] == s["expected_range"][1] for s in strips), u
    b3 = _crit(rep["uavs"]["B3"], "cam_count")["detail"]["strips"]
    assert [(s["n_cam"], s["expected_naive"]) for s in b3] == [(56, 57), (56, 56), (57, 57), (57, 57)]


def test_real_old_config_passes_count_and_spacing(real_report):
    rep, _out, _r = real_report
    for u in REAL:
        cr = rep["uavs"][u]
        for key in ("cam_count", "spacing_median", "spacing_p99", "outside_windows", "transition_landing",
                    "min_interval", "ingest_recovery"):
            assert _crit(cr, key)["passed"] is True, (u, key, _crit(cr, key))
        assert _crit(cr, "outside_windows")["value"] == 0 and _crit(cr, "transition_landing")["value"] == 0
        assert _crit(cr, "spacing_median")["value"] < 0.02 and _crit(cr, "spacing_p99")["value"] < 1.2
    # 舊設定 INTRVAL_MIN = 0 ⇒ 這一項沒有約束力，但最小間隔照實記
    c = _crit(rep["uavs"]["A1"], "min_interval")
    assert c["detail"]["fc_intrval_min_s"] == 0.0 and c["value"] == pytest.approx(1.32, abs=0.005)


def test_real_old_config_fails_what_it_should(real_report):
    rep, _out, _r = real_report
    assert rep["passed"] is False
    assert {u: _crit(rep["uavs"][u], "land_start_then_off")["value"] for u in REAL} == \
        {"A1": [61], "B2": [37], "B3": [61]}
    for u in REAL:
        assert _crit(rep["uavs"][u], "land_start_then_off")["passed"] is False
        c = _crit(rep["uavs"][u], "cam_params")
        assert c["passed"] is False and c["value"]["CAM1_DURATION"] == 1.0 and c["value"]["CAM_MAX_ROLL"] == 0.0
        assert rep["uavs"][u]["passed"] is False
    roll = {u: _crit(rep["uavs"][u], "roll_share") for u in REAL}
    assert {u: (c["detail"]["n_gt"], c["detail"]["n"]) for u, c in roll.items()} == \
        {"A1": (5, 167), "B2": (1, 111), "B3": (3, 222)}
    assert {u: c["passed"] for u, c in roll.items()} == {"A1": False, "B2": True, "B3": False}
    pooled = sum(c["detail"]["n_gt"] for c in roll.values()) / sum(c["detail"]["n"] for c in roll.values())
    assert pooled == pytest.approx(9 / 500)


def test_real_ingest_recovers_injected_faults(real_report):
    rep, out, _r = real_report
    for k, u in enumerate(sorted(REAL)):
        ing = rep["uavs"][u]["ingest"]
        assert ing["synth"]["seed"] == 1 + k
        assert ing["recovered"]["unmatched_fc"] == ing["injected"]["drop_index"] and len(ing["injected"]["drop_index"]) == 2
        assert ing["recovered"]["unmatched_pi_seq"] == ing["injected"]["extra_seq"]
        assert ing["fit"]["drift_ppm"] == pytest.approx(37.0, abs=1.0)
        assert all(ing["criterion"]["detail"]["checks"].values())
        n_cam = rep["uavs"][u]["summary"]["n_cam"]
        assert ing["n_poses"] == n_cam - 2 - 1 + 1                   # 漏 2、沒影像 1、雜訊 1
    rows = (out / "photos.csv").read_text(encoding="utf-8").splitlines()
    assert len(rows) == 1 + sum(170 + 113 + 226 - 2 * 3 for _ in [0])
    assert (out / "s1_report.json").exists() and (out / "s1_report.md").exists()
    assert sorted(p.name for p in out.iterdir()) == ["photos.csv", "s1_report.json", "s1_report.md", "sessions"]


def test_real_mission_readback_and_exposure(real_report):
    rep, _out, run_dir = real_report
    if run_dir is None:
        pytest.skip("本機沒有 runs/artifacts/HV4W55S（gitignored）")
    for u in REAL:
        c = _crit(rep["uavs"][u], "mission_readback")
        assert c["passed"] is True and c["value"] == 0
    ex = rep["exposure"]
    assert ex["inputs"]["fc_params"]["A1"]["SIM_WIND_SPD"] == 5.0
    assert ex["preflight"]["wind"] == [5.0, 180.0] and "SIM_WIND" in ex["preflight"]["wind_source"]
    assert {u: v["fc_cam"] for u, v in ex["preflight"]["fc_cam_vs_simulated"].items()} == \
        {"A1": 170, "B2": 113, "B3": 226}
    assert ex["postflight"]["photos_per_uav"] == {"A1": 168, "B2": 111, "B3": 224}
    assert ex["postflight"]["undulation_m"] == 0.0
    assert 0.0 < ex["postflight"]["frac_ge"]["5"] <= 1.0


# ══════════════════════════════════════════════════════════════════════
#  hv_sitl_run：真的規劃一次（GUI 路徑、旋鈕全開；不起 SITL，輸出到 tmp）
# ══════════════════════════════════════════════════════════════════════
S1_ENV = {"HV_MAPPING": "1", "HV_CAMERA": "hq6", "HV_SHUTTER_HZ": "2", "HV_SIDE_OVERLAP": "0.7",
          "HV_FWD_OVERLAP": "0.8"}


@pytest.fixture(scope="module")
def planned_s1(tmp_path_factory):
    import logging
    out = tmp_path_factory.mktemp("s1_plan")
    orig = runner.out_dir
    runner.out_dir = lambda sc: out
    logging.disable(logging.WARNING)
    try:
        k = runner.survey_knobs(S1_ENV)
        res = runner.plan_and_export(scenarios.SCENARIOS["gui_legal_120_1a2b"], k)
    finally:
        runner.out_dir = orig
        logging.disable(logging.NOTSET)
    return k, res, out


@pytest.mark.slow
def test_plan_with_knobs_reaches_planner_and_export(planned_s1):
    from hv_dccpp.ardupilot import read_qgc_wpl
    k, (hv, plan, _ctx, _cfg, summary), out = planned_s1
    ev = json.loads((out / "survey_knobs.json").read_text(encoding="utf-8"))
    assert ev["knobs"]["env"] == S1_ENV and ev["knobs"]["interval_min_s"] == 0.5
    assert ev["camera_trigger_config"] == dataclasses.asdict(k.trigger_config()) == summary["camera_cfg"]
    for tag, s in ev["planner_readback"]["sensors"].items():
        assert (s["forward_overlap"], s["side_overlap"], s["shutter_rate_hz"]) == (0.8, 0.7, 2.0), tag
        assert s["camera"]["focal_length_mm"] == 6.0
        assert s["trigger_distance_m"] == pytest.approx(18.848)
    assert ev["planner_readback"]["mapping_mode"]["lead_in_m"] == 50.0 and hv.mapping_mode.lead_in_m == 50.0
    assert "LOG_DISARMED" in ev["logging"]
    scen = json.loads((out / "scenario.json").read_text(encoding="utf-8"))
    assert (scen["hfov_deg"], scen["vfov_deg"]) == pytest.approx(k.fov_deg()) and scen["overlap_rate"] == 0.7
    for uav in plan.plans:
        par = dict(ln.split(",", 1) for ln in (out / "plan" / f"{uav}.param").read_text(encoding="utf-8").splitlines()
                   if ln and not ln.startswith("#") and "," in ln)
        assert {n: float(par[n]) for n in ("CAM1_TYPE", "CAM1_DURATION", "CAM1_INTRVAL_MIN", "CAM_AUTO_ONLY",
                                           "CAM_MAX_ROLL")} == {"CAM1_TYPE": 1.0, "CAM1_DURATION": 0.1,
                                                                "CAM1_INTRVAL_MIN": 0.5, "CAM_AUTO_ONLY": 1.0,
                                                                "CAM_MAX_ROLL": 15.0}
        items = read_qgc_wpl(out / "plan" / f"{uav}.waypoints")
        rows = [{"seq": it.seq, "command": it.command, "p1": it.param1} for it in items]
        assert s1.land_start_offenders(rows) == []
        ons = [i for i, it in enumerate(items) if it.command == 206 and it.param1 > 0]
        assert ons and all(np.float32(items[i].param1) == np.float32(18.848) for i in ons)
        for i in ons:                                     # 開相機前一個 NAV 是 50 m 引入點
            a, b = items[i - 2], items[i - 1]
            assert (a.command, b.command) == (16, 16)
            e, n = datum.local_en_m(a.x, a.y, b.x, b.y)
            assert math.hypot(float(e), float(n)) == pytest.approx(50.0, abs=0.5)


@pytest.mark.slow
def test_upload_items_equal_exported_mission(planned_s1):
    """上傳＝匯出：fly() 上傳的項目與匯出檔逐項同指令、206 p1 float32 一致（含 50 m 引入點與 189 → 206(0)）。"""
    from hv_dccpp.ardupilot import read_qgc_wpl
    k, (hv, plan, ctx, cfg, summary), out = planned_s1
    for uav in plan.plans:
        items = runner._mission_for_upload(plan, uav, ctx, cfg, k.trigger_config(), summary)
        exported = read_qgc_wpl(out / "plan" / f"{uav}.waypoints")
        assert [(i.seq, i.command) for i in items] == [(i.seq, i.command) for i in exported]
        assert all(np.float32(a.param1) == np.float32(b.param1) for a, b in zip(items, exported) if a.command == 206)
        assert max(abs(a.x - b.x) + abs(a.y - b.y) for a, b in zip(items, exported) if a.seq) < 2e-7
        # 沒有 camera_cfg ⇒ 舊版任務（沒有引入點），項數必不相同 ⇒ 比對會擋下
        old = runner._mission_for_upload(plan, uav, ctx, cfg)
        assert len(old) < len(items)
        with pytest.raises(SystemExit, match="上傳 .* 項 ≠ 匯出檔"):
            runner._mission_for_upload(plan, uav, ctx, cfg, None, summary)
