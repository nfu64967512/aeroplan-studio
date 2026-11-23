"""206 觸發距離的 50 Hz 量化保護（vendored 2026-10-21：``hv_dccpp/ardupilot.py``；docs/hv_dccpp/VENDOR.md #12）。

背景（docs/評估_2026-10-21_HARVEST邊界帶視數不足.md）：飛控每 20 ms 才比一次「離上一張 ≥ d」，實際間距 ＝ d 再加
0 到 v_g/50。前向重疊 0.8 時一條帶剛好 l/d = 5 視，邊界只有最外側條帶拍到的那一帶因此有一部分只剩 4 視
（2026-10-20 四趟實飛 724–5,136 m²，內部 0）。

鎖住的是**值**，不是「沒丟例外」：

* d_cmd ＝ l(1 − o_fwd)·(1 − f_s/50)，f_s ＝ 規劃拍照率；沒有拍照率 → 不扣、警告一次；f_s ≥ 50 Hz → ValueError；
* 給 camera_cfg（``trigger_overshoot_guard`` 預設 True）時每個 206 開與地形重發都是 d_cmd；guard=False 或不給
  camera_cfg ＝ 舊值；guard 只改 206 的 p1，其餘任務項目逐欄相同；
* 物理：飛前模擬（``core.recon.exposure_verifier``，AP_Camera 50 Hz 逐 tick 重現）掃空速，guard 開時帶內每一段間距
  ≤ l(1 − o_fwd)；關時有超出（證明這個量測抓得到）。
"""
from __future__ import annotations

import dataclasses
import logging
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

os.environ.setdefault("MAVLINK20", "1")

from hv_dccpp import ardupilot as ap  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.scenario import build_mini_example  # noqa: E402

CAM = 206


@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    mp = plan_mission(inputs, cfg, ctx=ctx)
    return SimpleNamespace(inputs=inputs, cfg=cfg, ctx=ctx, mp=mp, sensor=ctx.sensors[("B", "default_rgb")])


def _with_rate(sensor, f_s):
    return dataclasses.replace(sensor, r=dataclasses.replace(sensor.r, shutter_rate_hz=f_s))


def _on(items):
    return [it for it in items if it.command == CAM and it.param1 > 0]


def test_commanded_distance_value(mini):
    s = mini.sensor
    assert s.r.shutter_rate_hz == pytest.approx(1.0) and s.r.forward_overlap == pytest.approx(0.7)
    assert ap.CAMERA_UPDATE_HZ == 50
    assert ap.trigger_overshoot_fraction(s) == pytest.approx(1.0 / 50)
    for h in (80.0, 100.0, 130.0):
        assert ap.commanded_trigger_distance_m(s, h) == pytest.approx(ap.trigger_distance_m(s, h) * (1 - 1.0 / 50),
                                                                      rel=1e-12)
    # 首飛建圖的相機（HQ6、120 m、前向 0.8、2 Hz）：18.848 → 18.094 m
    s2 = _with_rate(dataclasses.replace(s, r=dataclasses.replace(s.r, forward_overlap=0.8)), 2.0)
    l120 = s2.footprint_len_m(120.0)
    assert ap.commanded_trigger_distance_m(s2, 120.0) == pytest.approx(l120 * 0.2 * (1 - 2.0 / 50), rel=1e-12)
    # 沒有拍照率 ⇒ 無從界定 V_g,cap ⇒ 不扣
    s0 = _with_rate(s, None)
    assert ap.trigger_overshoot_fraction(s0) == 0.0
    assert ap.commanded_trigger_distance_m(s0, 100.0) == ap.trigger_distance_m(s0, 100.0)
    with pytest.raises(ValueError, match="50 Hz"):
        ap.trigger_overshoot_fraction(_with_rate(s, 50.0))


def test_guard_changes_only_206_p1_and_can_be_turned_off(mini):
    up = mini.mp.plans["B1"]
    on_ = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig())
    off = ap.plan_to_mission_items(up, mini.ctx, mini.cfg,
                                   camera_cfg=ap.CameraTriggerConfig(trigger_overshoot_guard=False))
    plain = ap.plan_to_mission_items(up, mini.ctx, mini.cfg)
    assert ap.CameraTriggerConfig().trigger_overshoot_guard is True
    assert len(on_) == len(off) and len(_on(on_)) == 3
    for a, b in zip(on_, off):
        if a.command == CAM and a.param1 > 0:
            assert np.float32(a.param1) == np.float32(b.param1 * (1 - 1.0 / 50))
            assert dataclasses.replace(a, param1=0.0) == dataclasses.replace(b, param1=0.0)
        else:
            assert a == b
    # guard=False ＝ 吸附到作業高度的舊值；不給 camera_cfg ＝ 舊行為（該點 AGL、不扣）
    assert {np.float32(it.param1) for it in _on(off)} == {np.float32(ap.trigger_distance_m(mini.sensor, 100.0))}
    starts = [w for w in up.waypoints if w.sensor_on and w.phase.name == "OPERATE"]
    first = {}
    for w in starts:
        first.setdefault(w.strip_id, w)
    assert [np.float32(it.param1) for it in _on(plain)] == \
        [np.float32(ap.trigger_distance_m(mini.sensor, first[s].agl_m)) for s, _d in up.strips]


def test_guard_flag_must_be_bool():
    with pytest.raises(ValueError, match="trigger_overshoot_guard"):
        ap.CameraTriggerConfig(trigger_overshoot_guard=1)


def test_no_shutter_rate_warns_once_and_keeps_nominal(mini, caplog):
    up = mini.mp.plans["B1"]
    s0 = _with_rate(mini.sensor, None)
    sens = {sid: s0 for sid, _d in up.strips}
    with caplog.at_level(logging.WARNING, logger="hv_dccpp.ardupilot"):
        items = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, strip_sensors=sens,
                                         camera_cfg=ap.CameraTriggerConfig())
    assert {np.float32(it.param1) for it in _on(items)} == {np.float32(ap.trigger_distance_m(s0, 100.0))}
    msgs = [r.getMessage() for r in caplog.records if "沒有拍照率" in r.getMessage()]
    assert len(msgs) == 1 and "B1" in msgs[0]


def _spacings(items, airspeed):
    """飛前模擬（50 Hz 逐 tick 的 AP_Camera 重現）：DO_CHANGE_SPEED 改成 ``airspeed``，回傳每條帶內相鄰照片間距。"""
    from core.recon import exposure_verifier as ev
    its = [dataclasses.replace(it, param2=airspeed) if (it.command == 178 and int(it.param1) == 0) else it
           for it in items]
    sim = ev.simulate_mission_photos(its, uav="B1", airspeed_mps=airspeed, wind=(0.0, 0.0))
    fr = ev.LocalFrame(sim.photos[0].lat_deg, sim.photos[0].lon_deg)
    out = []
    for s in sorted({p.strip for p in sim.photos if p.strip}):
        ph = [p for p in sim.photos if p.strip == s]
        x, y = fr.to_xy(np.array([p.lat_deg for p in ph]), np.array([p.lon_deg for p in ph]))
        out += list(np.hypot(np.diff(x), np.diff(y)))
    return np.array(out)


def test_realized_spacing_never_exceeds_planned(mini):
    """掃空速 15–24 m/s（≤ V_g,cap = 29.25 m/s）：guard 開 ⇒ 每一段間距 ≤ l(1 − o_fwd)；guard 關 ⇒ 有超出。"""
    up = mini.mp.plans["B1"]
    d_plan = ap.trigger_distance_m(mini.sensor, 100.0)
    v_cap = mini.sensor.max_ground_speed_mps(100.0)
    assert v_cap == pytest.approx(d_plan * 1.0)
    on_ = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig())
    off = ap.plan_to_mission_items(up, mini.ctx, mini.cfg,
                                   camera_cfg=ap.CameraTriggerConfig(trigger_overshoot_guard=False))
    worst_on, worst_off = 0.0, 0.0
    for va in np.arange(15.0, 24.0 + 1e-9, 0.05):
        assert va < v_cap
        sp_on, sp_off = _spacings(on_, float(va)), _spacings(off, float(va))
        assert sp_on.size > 50
        worst_on = max(worst_on, float(sp_on.max()))
        worst_off = max(worst_off, float(sp_off.max()))
    assert worst_on <= d_plan + 1e-6
    assert worst_off > d_plan + 0.2                  # 舊值：最多多走一個 tick（v/50 ≈ 0.3–0.48 m）
