"""survey.plan_snapshot：飛行計畫快照 —— 值層級的鎖。

假 HARVEST 計畫（不跑規劃器）：幾何用 **UTM 51N（EPSG:32651）** 當 ENU（與 tests/test_survey_gcp_plan.py 同法），
建圖設定用真的 ``mission.swarm_coordinator.build_mapping_setup``、風層／站點用真的 hv_dccpp 資料類別，
GCP 區塊用真的 ``survey.gcp_plan.blocks_from_hv``。所有檔案只寫在 tmp_path。
"""
from __future__ import annotations

import datetime as dt
import json
import math
import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pyproj import Transformer
from shapely.geometry import box

from survey import gcp_plan as gp
from survey import plan_snapshot as ps
from survey.plan_snapshot import PlanSnapshotError

UTM = "EPSG:32651"
TO_UTM = Transformer.from_crs("EPSG:4326", UTM, always_xy=True)
TO_LL = Transformer.from_crs(UTM, "EPSG:4326", always_xy=True)
E0, N0 = TO_UTM.transform(121.21, 25.07)
T_UP = "2026-10-20T03:04:05Z"
SW = {"aeroplan_git_describe": "test", "aeroplan_git_error": None, "python": "3.x"}
SECRETS = ("SECRET-TOKEN-123", "SECRET-KEY-456", "SECRET-CONN-789", "hunter2")

PARAMS = dict(hv_map_focal_mm=8.8, hv_map_pixel_um=2.4, hv_map_width_px=5472, hv_map_height_px=3648,
              hv_map_trigger_hz=1.0, hv_map_forward_overlap=0.8, hv_map_side_overlap=0.7, hv_map_lead_in_m=30.0,
              hv_map_max_roll_deg=15.0, hv_map_backend="servo", hv_map_trigger_verified=True,
              hv_map_readout_ms=20.0, hv_map_camera_preset="HQ6", hv_wind=(3.0, 45.0))


def ll(x, y):
    lon, lat = TO_LL.transform(E0 + x, N0 + y)
    return (float(lat), float(lon))


class _Frame:
    def to_wgs84(self, enu):
        enu = np.asarray(enu, float).reshape(-1, 3)
        lon, lat = TO_LL.transform(E0 + enu[:, 0], N0 + enu[:, 1])
        return np.column_stack([lat, lon, enu[:, 2]])


AREA_XY = (-700.0, -560.0, 100.0, 560.0)        # x0, y0, x1, y1


def _items(home, strips):
    """建圖任務項目（mission_item_dicts 的形狀）：home、VTOL 起飛、空速、每條帶 引入點→206 開→終點→206 關、降落。"""
    rows = [dict(command=16, frame=0, lat=home[0], lon=home[1], alt=32.0)]
    rows += [dict(command=84, frame=3, lat=home[0], lon=home[1], alt=60.0),
             dict(command=178, frame=3, param1=0.0, param2=22.0, param3=-1.0)]
    for a, b in strips:
        rows += [dict(command=16, frame=3, lat=a[0], lon=a[1], alt=120.0, param2=1.0),
                 dict(command=206, frame=3, param1=26.3),
                 dict(command=16, frame=3, lat=b[0], lon=b[1], alt=120.0, param2=30.0),
                 dict(command=206, frame=3, param1=0.0)]
    rows += [dict(command=189, frame=3), dict(command=206, frame=3, param1=0.0),
             dict(command=85, frame=3, lat=home[0], lon=home[1], alt=0.0)]
    out = []
    for k, r in enumerate(rows):
        it = {"seq": k, "command": r["command"], "frame": r["frame"], "lat": float(r.get("lat", 0.0)),
              "lon": float(r.get("lon", 0.0)), "alt": float(r.get("alt", 0.0)), "param1": float(r.get("param1", 0.0)),
              "param2": float(r.get("param2", 0.0)), "param3": float(r.get("param3", 0.0)), "param4": 0.0,
              "current": 0, "autocontinue": 1}
        out.append(it)
    return out


def make_plan():
    """(hv, swarm, jobs)：兩架（A1 三條主帶、B2 兩條主帶＋一條交叉帶）、兩條連線 sysid 11／12。"""
    from hv_dccpp.geometry import cross_strip_id
    from hv_dccpp.mission import Site, WindLayer
    from mission.swarm_coordinator import build_mapping_setup

    setup = build_mapping_setup(PARAMS)
    strips, plans = {}, {}
    cross = cross_strip_id("Z1_1", "B", "B2", 1)
    for uid, xs, cls, site in (("A1", (-580.0, -540.0, -500.0), "A", "TO_1"), ("B2", (-100.0, -60.0), "B", "TO_2")):
        seq = []
        for k, x in enumerate(xs):
            sid = f"Z1_1_{cls}_{uid}_{k}"
            strips[sid] = SimpleNamespace(pL=np.array([x, -500.0]), pR=np.array([x, 500.0]), spacing_m=40.0,
                                          width_m=57.0, h_op_m=120.0, z_op_m=150.0, length_m=1000.0, cls=cls)
            seq.append((sid, -1 if k % 2 == 0 else 1))
        if uid == "B2":
            strips[cross] = SimpleNamespace(pL=np.array([-300.0, 0.0]), pR=np.array([0.0, 0.0]), spacing_m=40.0,
                                            width_m=57.0, h_op_m=130.0, z_op_m=160.0, length_m=300.0, cls=cls)
            seq.append((cross, -1))
        plans[uid] = SimpleNamespace(strips=seq, cls=cls, t_takeoff_s=0.0 if uid == "A1" else 95.0,
                                     t_land_s=1700.0, takeoff_site=site, landing_site=site)
    home = Site("home", *ll(-200.0, -650.0), alt_msl_m=30.0, kind="home")
    to1 = Site("TO_1", *ll(-560.0, -650.0), alt_msl_m=32.0)
    to2 = Site("TO_2", *ll(-80.0, -650.0), alt_msl_m=31.0)
    x0, y0, x1, y1 = AREA_XY
    area = SimpleNamespace(area_id="Z1", polygon_wgs84=(ll(x0, y0), ll(x1, y0), ll(x1, y1), ll(x0, y1)),
                           holes_wgs84=())
    inputs = SimpleNamespace(areas=(area,), wind=(WindLayer(0.0, math.inf, 3.0, 45.0),), home=home,
                             sites={"home": home, "TO_1": to1, "TO_2": to2})
    ctx = SimpleNamespace(frame=_Frame(), specs={"A": SimpleNamespace(name="Pioneer"), "B": SimpleNamespace(name="Hero")},
                          inputs=inputs, areas_enu={"Z1": box(x0, y0, x1, y1)}, nofly_buffered=[])
    plan = SimpleNamespace(plans=plans, strips=strips, makespan_s=1800.0, uncovered=[],
                           feasibility=SimpleNamespace(status="GO_WITH_WARNINGS"), timing_ms={"total": 12.3})
    paths = {1: SimpleNamespace(alt_ref_site="home", alt_ref_msl_m=30.0, fc_home_site="TO_1", fc_home_msl_m=32.0,
                                fc_alt_offset_m=2.0, land_alt_m=0.0),
             2: SimpleNamespace(alt_ref_site="home", alt_ref_msl_m=30.0, fc_home_site="TO_2", fc_home_msl_m=31.0,
                                fc_alt_offset_m=1.0, land_alt_m=0.0)}
    hv = SimpleNamespace(last_plan=plan, last_ctx=ctx, last_inputs=inputs, id_map={1: "A1", 2: "B2"},
                         mapping_mode=setup["mapping_mode"], camera_cfg=setup["camera_cfg"],
                         mapping_effective={"config": setup["mapping_mode"].to_dict(),
                                            "per_class": {"B": {"timing_ms": 4.0, "seam_m": 12.5}}},
                         mapping_max_wind_mps=3.0, vertical_datum="msl", geoid_undulation_m=None, dem_step_m=25.0,
                         notes=["terrain_source(flat)", "mapping_mode_effective(...)"],
                         last_result={"assembled_paths": paths})
    swarm = SimpleNamespace(hv_mapping_setup=setup, hv_coordinator=hv,
                            harvest_params={**PARAMS, "num_drones": 2, "hv_num_a": 1, "odm_token": SECRETS[0],
                                            "hv_map_api_key": SECRETS[1], "hv_fleet_label": object()},
                            hv_validation_reasons=[], hv_mapping_gate={"uploadable": True, "reasons": []},
                            hv_exposure_summary={"ok": True, "elapsed_s": 1.23, "frac_ge": {5: 0.99},
                                                 "drone_of": {"A1": 1, "B2": 2}})

    def strip_ll(uid):
        out = []
        for sid, d in plans[uid].strips:
            st = strips[sid]
            a, b = (st.pL, st.pR) if d == -1 else (st.pR, st.pL)
            out.append((ll(*a), ll(*b)))
        return out
    links = {11: SimpleNamespace(conn_str=f"tcp://pilot:{SECRETS[3]}@127.0.0.1:5760?token={SECRETS[2]}",
                                 sysid_label=11),
             12: SimpleNamespace(conn_str="udpin:0.0.0.0:14560", sysid_label=12)}
    jobs = []
    for sysid, did, uid, site in ((11, 1, "A1", to1), (12, 2, "B2", to2)):
        items = _items(site.latlon, strip_ll(uid))
        jobs.append({"sysid": sysid, "link": links[sysid], "uav_id": did, "items": items,
                     "expected": {"CAM1_TYPE": 1.0, "CAM1_INTRVAL_MIN": 1.0, "MIS_RESTART": 0.0},
                     "airspeed_expected": {"AIRSPEED_MIN": 18.0, "AIRSPEED_MAX": 30.0}, "speeds": [22.0]})
    return hv, swarm, jobs


def snap_of(hv=None, swarm=None, jobs=None, *, when=T_UP, **kw):
    if hv is None:
        hv, swarm, jobs = make_plan()
    return ps.snapshot_from_hv(hv, swarm, jobs, takeoff_offsets={1: 0.0, 2: 95.0},
                               takeoff_source="HvDccppCoordinator.takeoff_schedule()",
                               readback_info={11: ["CAM1_TYPE 相符"]}, skipped=[13], not_targeted=[14], tlog=True,
                               uploaded_at_utc=when, software=SW, **kw)


def acked(snap, *sysids):
    for s in sysids:
        snap = ps.set_upload_status(snap, s, ok=True, message="ACCEPTED＋讀回相符")
    return snap


@pytest.fixture(autouse=True)
def _snapshot_dir(tmp_path, monkeypatch):
    monkeypatch.setenv(ps.ENV_DIR, str(tmp_path / "default_root"))


# ─────────────────────────────────────────────────────────────────────────────
# 來回、決定性、唯讀
# ─────────────────────────────────────────────────────────────────────────────

def test_round_trip_write_then_read_is_identical(tmp_path):
    snap = acked(snap_of(), 11)
    path = ps.write_snapshot(tmp_path, snap)
    local = dt.datetime(2026, 10, 20, 3, 4, 5, tzinfo=dt.timezone.utc).astimezone()
    assert path == tmp_path / local.strftime("%Y%m%d") / f"flown_plan_{local:%H%M%S}_{snap['plan_sha256'][:8]}.json"
    raw = path.read_bytes()
    assert b"\r" not in raw and raw.endswith(b"}\n")
    back = ps.read_snapshot(path)
    assert back == snap
    assert back["format"] == "aeroplan-plan-snapshot/1" and back["uploaded_at_utc"] == T_UP
    assert back["upload"]["aircraft"]["11"]["ok"] is True and back["upload"]["aircraft"]["12"]["ok"] is None
    assert back["upload"]["skipped_sysids"] == [13] and back["upload"]["not_targeted_sysids"] == [14]
    assert back["upload"]["tlog"] is True and back["upload"]["readback_info"] == {"11": ["CAM1_TYPE 相符"]}
    # 內容：兩架、作業區、建圖設定、風、起飛時程、高度基準
    plan = back["plan"]
    assert sorted(plan["aircraft"]) == ["1", "2"] and sorted(plan["mission_uploads"]) == ["11", "12"]
    a1 = plan["aircraft"]["1"]
    assert (a1["uav_id"], a1["cls"], a1["platform_name"], a1["uploaded_to_sysids"]) == ("A1", "A", "Pioneer", [11])
    assert a1["takeoff_site"]["site_id"] == "TO_1" and a1["takeoff_site"]["alt_msl_m"] == 32.0
    assert a1["altitude"]["fc_alt_offset_m"] == 2.0 and a1["altitude"]["fc_home_site"] == "TO_1"
    assert plan["takeoff_schedule"] == {"source": "HvDccppCoordinator.takeoff_schedule()",
                                        "offsets_s": {"1": 0.0, "2": 95.0}}
    eff = plan["mapping"]["setup_effective"]
    assert (eff["focal_mm"], eff["forward_overlap"], eff["side_overlap"], eff["lead_in_m"], eff["readout_ms"]) == \
        (8.8, 0.8, 0.7, 30.0, 20.0)
    assert plan["mapping"]["camera_trigger"]["camera_lead_in_m"] == 30.0
    assert plan["mapping"]["mapping_mode"]["lead_in_m"] == 30.0
    assert plan["wind"]["planning_speed_mps"] == 3.0 and plan["wind"]["planning_dir_from_deg"] == 45.0
    assert plan["wind"]["source"].startswith("規劃風") and plan["wind"]["panel_hv_wind"] == [3.0, 45.0]
    assert plan["terrain"]["vertical_datum"] == "msl" and plan["terrain"]["notes"] == ["terrain_source(flat)"]
    assert plan["survey_polygons"]["n_nofly"] == 0 and len(plan["survey_polygons"]["items"]) == 1
    assert plan["planner"]["name"] == "HARVEST" and plan["planner"]["feasibility_status"] == "GO_WITH_WARNINGS"
    mu = plan["mission_uploads"]["11"]
    assert mu["mission_items"] == next(j["items"] for j in make_plan()[2] if j["sysid"] == 11)
    assert (mu["n_206_on"], mu["n_206_off"], mu["trigger_distances_m"]) == (3, 4, [26.3])
    assert mu["expected_photos"] > 0 and mu["callsign"] == "UAV-11"


def test_same_input_gives_same_bytes_except_upload_time(tmp_path):
    a = acked(snap_of(), 11)
    b = acked(snap_of(), 11)
    pa, pb = ps.write_snapshot(tmp_path / "a", a), ps.write_snapshot(tmp_path / "b", b)
    assert pa.read_bytes() == pb.read_bytes()
    c = acked(snap_of(when=dt.datetime(2026, 10, 21, 7, 0, 0, tzinfo=dt.timezone.utc)), 11)
    assert c["plan"] == a["plan"] and c["plan_sha256"] == a["plan_sha256"]
    diff = sorted(k for k in a if a[k] != c[k])
    assert diff == ["digest_sha256", "uploaded_at_utc"]
    # 另一個 ACK 結果只改 upload 段：plan_sha256 不變
    d = ps.set_upload_status(a, 12, ok=False, message="寫入逾時")
    assert d["plan_sha256"] == a["plan_sha256"] and d["digest_sha256"] != a["digest_sha256"]
    assert a["upload"]["aircraft"]["12"]["ok"] is None            # 原 dict 不動


def test_build_does_not_mutate_the_planner_state():
    hv, swarm, jobs = make_plan()
    before = (repr(hv.last_plan.plans), repr(hv.last_plan.strips), gp.blocks_from_hv(hv), repr(swarm.harvest_params),
              json.dumps(jobs, default=repr, sort_keys=True))
    snap_of(hv, swarm, jobs)
    after = (repr(hv.last_plan.plans), repr(hv.last_plan.strips), gp.blocks_from_hv(hv), repr(swarm.harvest_params),
             json.dumps(jobs, default=repr, sort_keys=True))
    assert before == after


def test_strip_rows_follow_direction_and_mark_cross_strips():
    snap = snap_of()
    a1 = snap["plan"]["aircraft"]["1"]["strips"]
    assert [r["strip_id"] for r in a1] == ["Z1_1_A_A1_0", "Z1_1_A_A1_1", "Z1_1_A_A1_2"]
    # dir −1：pL(y=−500) → pR(y=+500)，往北；+1 反過來
    assert a1[0]["direction"] == -1 and a1[0]["start"] == pytest.approx(list(ll(-580.0, -500.0)), abs=1e-12)
    assert a1[0]["end"] == pytest.approx(list(ll(-580.0, 500.0)), abs=1e-12)
    assert a1[1]["start"] == pytest.approx(list(ll(-540.0, 500.0)), abs=1e-12)

    def ang(a, b):
        return abs((a - b + 180.0) % 360.0 - 180.0)
    assert ang(a1[0]["heading_deg"], 0.0) < 2.0 and ang(a1[1]["heading_deg"], 180.0) < 2.0   # UTM 子午線收斂 < 1°
    assert a1[0]["endpoint_distance_m"] == pytest.approx(1000.0, rel=1e-3)
    assert (a1[0]["h_op_m"], a1[0]["z_op_m"], a1[0]["spacing_m"], a1[0]["cross"]) == (120.0, 150.0, 40.0, False)
    b2 = snap["plan"]["aircraft"]["2"]["strips"]
    assert [r["cross"] for r in b2] == [False, False, True]
    assert ang(b2[2]["heading_deg"], 90.0) < 2.0


# ─────────────────────────────────────────────────────────────────────────────
# 竄改、驗證
# ─────────────────────────────────────────────────────────────────────────────

def _rewrite(path: Path, doc) -> None:
    path.write_bytes((json.dumps(doc, ensure_ascii=False, indent=1, sort_keys=True) + "\n").encode("utf-8"))


def test_tampered_or_truncated_file_is_rejected(tmp_path):
    path = ps.write_snapshot(tmp_path, acked(snap_of(), 11))
    doc = json.loads(path.read_text(encoding="utf-8"))
    doc["plan"]["aircraft"]["1"]["strips"][0]["start"][0] += 1e-6          # 改一個座標、不改雜湊
    _rewrite(path, doc)
    with pytest.raises(PlanSnapshotError, match="digest 不符"):
        ps.read_snapshot(path)
    doc["digest_sha256"] = ps.content_digest(doc)                         # 補 digest 但 plan_sha256 沒跟著改
    _rewrite(path, doc)
    with pytest.raises(PlanSnapshotError, match="plan_sha256 與 plan 段內容不符"):
        ps.read_snapshot(path)
    good = ps.write_snapshot(tmp_path / "g", acked(snap_of(), 11))
    raw = good.read_bytes()
    good.write_bytes(raw[: len(raw) // 2])
    with pytest.raises(PlanSnapshotError, match="不是合法的 UTF-8 JSON"):
        ps.read_snapshot(good)
    with pytest.raises(PlanSnapshotError, match="讀不到"):
        ps.read_snapshot(tmp_path / "nope.json")


def _resealed(snap, mutate):
    doc = json.loads(json.dumps(snap))
    mutate(doc)
    return ps._seal(doc)


@pytest.mark.parametrize("mutate, msg", [
    (lambda d: d.update(format="aeroplan-gcp-plan/1"), "不是飛行計畫快照"),
    (lambda d: d.update(format="aeroplan-plan-snapshot/2"), "版本 'aeroplan-plan-snapshot/2' 不支援"),
    (lambda d: d.update(extra=1), "不認得的欄位"),
    (lambda d: d.update(uploaded_at_utc="2026-10-20 03:04:05"), "YYYY-MM-DDTHH:MM:SSZ"),
    (lambda d: d["plan"].pop("gcp_blocks"), "plan 缺少欄位"),
    (lambda d: d["plan"]["areas"]["items"][0].update(polygon=[[25.0, 121.0], [25.1, 121.0]]), "不是合法的多邊形"),
    (lambda d: d["plan"]["areas"]["items"][0]["polygon"][0].__setitem__(0, 95.0), "不是合法的多邊形"),
    (lambda d: d["plan"]["mission_uploads"]["11"]["mission_items"][3].update(param1=99.0), "任務項目與 n_items／sha256 不符"),
    (lambda d: d["plan"]["mission_uploads"]["11"].update(drone_id=7), "不在 plan.aircraft 裡"),
    (lambda d: d["plan"]["gcp_blocks"]["blocks"].update({"A|1": [[[25.0, 121.0]] * 3]}), "區塊標籤不能"),
    (lambda d: d["plan"]["wind"].pop("source"), "plan.wind 沒有來源說明"),
    (lambda d: d["upload"]["aircraft"].pop("12"), "sysid 要與 plan.mission_uploads 完全相同"),
    (lambda d: d["upload"]["aircraft"]["11"].update(ok="yes"), "ok 要是 true／false／null"),
    (lambda d: d["upload"]["aircraft"]["11"].update(ok=None), "沒有任何一架回報上傳成功"),
])
def test_strict_validation_messages(tmp_path, mutate, msg):
    doc = _resealed(acked(snap_of(), 11), mutate)
    path = tmp_path / "x.json"
    _rewrite(path, doc)
    with pytest.raises(PlanSnapshotError, match=re.escape(msg)):
        ps.read_snapshot(path)


def test_cannot_write_before_any_upload_ack(tmp_path):
    snap = snap_of()
    with pytest.raises(PlanSnapshotError, match="沒有任何一架回報上傳成功"):
        ps.write_snapshot(tmp_path, snap)
    assert not any(tmp_path.rglob("*"))
    failed = ps.set_upload_status(snap, 11, ok=False, message="NACK")
    with pytest.raises(PlanSnapshotError, match="沒有任何一架回報上傳成功"):
        ps.write_snapshot(tmp_path, failed)
    assert ps.read_snapshot  # noqa: B018 - 只是確認 API 存在
    assert ps.validate_snapshot(snap, require_upload_ok=False) is snap


def test_uploaded_at_needs_a_timezone():
    with pytest.raises(PlanSnapshotError, match="沒有時區"):
        snap_of(when=dt.datetime(2026, 10, 20, 3, 4, 5))
    with pytest.raises(PlanSnapshotError, match="YYYY-MM-DDTHH:MM:SSZ"):
        snap_of(when="2026/10/20 03:04")
    s = snap_of(when=dt.datetime(2026, 10, 20, 11, 4, 5, tzinfo=dt.timezone(dt.timedelta(hours=8))))
    assert s["uploaded_at_utc"] == T_UP


# ─────────────────────────────────────────────────────────────────────────────
# 寫檔：永不覆蓋、ACK 更新
# ─────────────────────────────────────────────────────────────────────────────

def test_write_never_overwrites_and_leaves_no_temp_files(tmp_path):
    a = acked(snap_of(), 11)
    p1 = ps.write_snapshot(tmp_path, a)
    raw1 = p1.read_bytes()
    b = acked(snap_of(), 11, 12)                                   # 同計畫、同上傳時刻 ⇒ 同檔名
    p2 = ps.write_snapshot(tmp_path, b)
    p3 = ps.write_snapshot(tmp_path, b)
    assert p2.name == p1.stem + "_2.json" and p3.name == p1.stem + "_3.json"
    assert p1.read_bytes() == raw1 and ps.read_snapshot(p2) == b
    assert sorted(p.name for p in p1.parent.iterdir()) == sorted([p1.name, p2.name, p3.name])


def test_update_upload_status_rewrites_only_the_upload_section(tmp_path):
    first = acked(snap_of(), 11)
    path = ps.write_snapshot(tmp_path, first)
    new = ps.update_upload_status(path, 12, ok=False, message="MISSION_ACK 錯誤 13")
    back = ps.read_snapshot(path)
    assert back == new and back["plan"] == first["plan"] and back["plan_sha256"] == first["plan_sha256"]
    assert back["upload"]["aircraft"]["12"] == {**first["upload"]["aircraft"]["12"], "ok": False,
                                               "message": "MISSION_ACK 錯誤 13"}
    raw = path.read_bytes()
    ps.update_upload_status(path, 12, ok=False, message="MISSION_ACK 錯誤 13")   # 同結果：不重寫
    assert path.read_bytes() == raw
    with pytest.raises(PlanSnapshotError, match="已記錄為 ok=True"):
        ps.update_upload_status(path, 11, ok=False, message="另一次上傳")
    with pytest.raises(PlanSnapshotError, match="不在這份快照的上傳清單裡"):
        ps.update_upload_status(path, 99, ok=True)
    assert path.read_bytes() == raw and len(list(path.parent.iterdir())) == 1


def test_default_root_env_and_pytest_tripwire(monkeypatch, tmp_path):
    assert ps.default_root() == tmp_path / "default_root"
    monkeypatch.delenv(ps.ENV_DIR)
    with pytest.raises(PlanSnapshotError, match="測試中沒有設 AEROPLAN_PLAN_SNAPSHOT_DIR"):
        ps.default_root()
    monkeypatch.delenv("PYTEST_CURRENT_TEST")
    assert ps.default_root() == Path(ps.__file__).resolve().parent.parent / "runs" / "survey"


# ─────────────────────────────────────────────────────────────────────────────
# 沒有機密、不是有限數、計時欄位、大量任務項目
# ─────────────────────────────────────────────────────────────────────────────

def test_no_secrets_in_the_file(tmp_path):
    path = ps.write_snapshot(tmp_path, acked(snap_of(), 11))
    text = path.read_text(encoding="utf-8")
    for s in SECRETS:
        assert s not in text
    snap = ps.read_snapshot(path)
    conn = snap["upload"]["aircraft"]["11"]["connection"]
    assert conn == "tcp://<已移除>@127.0.0.1:5760?<已移除>"
    assert snap["upload"]["aircraft"]["12"]["connection"] == "udpin:0.0.0.0:14560"
    params = snap["plan"]["params_plan_time"]
    assert "odm_token" not in params["values"] and "hv_map_api_key" not in params["values"]
    assert params["not_recorded"] == ["hv_fleet_label（object）", "hv_map_api_key"]
    assert params["values"]["hv_map_focal_mm"] == 8.8 and params["values"]["num_drones"] == 2


def test_nonfinite_and_clock_fields_are_recorded_not_hidden():
    snap = snap_of()
    plan = snap["plan"]
    assert plan["wind"]["layers"][0]["z_top_m"] == "Infinity"
    assert plan["data_issues"]["nonfinite_as_string"] == ["plan.wind.layers[0].z_top_m"]
    assert "elapsed_s" not in plan["exposure_prediction"] and plan["exposure_prediction"]["frac_ge"] == {"5": 0.99}
    assert plan["data_issues"]["dropped_clock_fields"] == ["plan.exposure_prediction.elapsed_s",
                                                           "plan.mapping.mapping_effective.per_class.B.timing_ms"]
    assert plan["data_issues"]["unserializable"] == []


def test_huge_missions_keep_digest_and_count_and_say_so(monkeypatch, tmp_path):
    monkeypatch.setattr(ps, "MISSION_ITEMS_INLINE_MAX", 5)
    snap = acked(snap_of(), 11)
    mu = snap["plan"]["mission_uploads"]["11"]
    assert mu["mission_items"] is None and "內嵌上限 5" in mu["mission_items_omitted"]
    items = next(j["items"] for j in make_plan()[2] if j["sysid"] == 11)
    assert mu["n_items"] == len(items) and mu["mission_items_sha256"] == ps._sha256(items)
    assert any("UAV-11 任務項目沒有內嵌" in w for w in snap["plan"]["warnings"])
    assert ps.read_snapshot(ps.write_snapshot(tmp_path, snap)) == snap


# ─────────────────────────────────────────────────────────────────────────────
# 給對話框：作業區、GCP 區塊
# ─────────────────────────────────────────────────────────────────────────────

def test_polygons_feed_normalize_polygons_and_the_gcp_dialog(tmp_path):
    from survey.pipeline import normalize_polygons
    from ui.dialogs.gcp_plan_dialog import GcpPlanDialog
    path = ps.write_snapshot(tmp_path, acked(snap_of(), 11))
    snap = ps.read_snapshot(path)
    x0, y0, x1, y1 = AREA_XY
    want = [[ll(x0, y0), ll(x1, y0), ll(x1, y1), ll(x0, y1)]]
    rings = ps.snapshot_polygons(snap)
    assert rings == want and normalize_polygons(polygons=rings) == want
    assert ps.polygons_provider(snap)() == want
    assert gp.normalize_areas(rings) == want
    fake = SimpleNamespace(polygons_source=ps.polygons_provider(snap), map_widget=None)
    got, _src, err = GcpPlanDialog.areas(fake)
    assert err is None and got == want
    assert "作業區 1 塊" in ps.polygons_label(snap, path) and "內洞" not in ps.polygons_label(snap, path)


def test_gcp_blocks_match_the_live_provider_and_drive_the_layout(tmp_path):
    from ui.dialogs.gcp_plan_dialog import GcpPlanDialog
    hv, swarm, jobs = make_plan()
    live_blocks, live_info = gp.blocks_from_hv(hv)
    path = ps.write_snapshot(tmp_path, acked(snap_of(hv, swarm, jobs), 11))
    snap = ps.read_snapshot(path)
    blocks, info, label = ps.snapshot_gcp_blocks(snap, path)
    assert blocks == live_blocks and info == live_info                     # 逐值相同（tuple 形狀也一樣）
    assert label.startswith(f"飛行計畫快照 {path.name}（plan {snap['plan_sha256'][:12]}") and "A1（#1）" in label
    fake = SimpleNamespace(chk_use_blocks=SimpleNamespace(isChecked=lambda: True),
                           blocks_source=ps.blocks_provider(snap, path))
    b, i, src, err = GcpPlanDialog.blocks(fake)
    assert err is None and b == live_blocks and i == live_info and src == label
    layout = gp.plan_layout(ps.snapshot_polygons(snap), b, gp.LayoutParams(preset="cross_strips"), blocks_source=src)
    assert layout.counts["GCP"] >= 5 and set(layout.block_rings) == set(b)
    per_block = {p.block for p in layout.gcps() if p.purpose == "block"}
    assert per_block and per_block <= set(b)


def test_missing_optional_parts_are_reported_loudly(monkeypatch, tmp_path):
    from ui.dialogs.gcp_plan_dialog import GcpPlanDialog

    def boom(_hv):
        raise gp.GcpPlanError("HARVEST 規劃裡沒有任何一架有條帶")
    monkeypatch.setattr(gp, "blocks_from_hv", boom)
    hv, swarm, jobs = make_plan()
    del hv.last_result["assembled_paths"][2]
    snap = acked(snap_of(hv, swarm, jobs), 11)
    g = snap["plan"]["gcp_blocks"]
    assert "blocks" not in g and g["error"] == "GcpPlanError: HARVEST 規劃裡沒有任何一架有條帶"
    warns = snap["plan"]["warnings"]
    assert any(w.startswith("GCP 區塊取不到") for w in warns)
    assert any("機 2（B2） 沒有組裝路徑" in w for w in warns)
    assert snap["plan"]["aircraft"]["2"]["altitude"] == {"error": "last_result['assembled_paths'] 沒有這架"}
    with pytest.raises(PlanSnapshotError, match="上傳時就取不到"):
        ps.blocks_provider(snap)()
    fake = SimpleNamespace(chk_use_blocks=SimpleNamespace(isChecked=lambda: True), blocks_source=ps.blocks_provider(snap))
    b, _i, _src, err = GcpPlanDialog.blocks(fake)
    assert b is None and "上傳時就取不到" in err
    assert any("GCP 區塊：✘" in line for line in ps.summary_lines(snap))
    assert ps.read_snapshot(ps.write_snapshot(tmp_path, snap)) == snap     # 照樣是合法快照


def test_refuses_non_mapping_plans_and_empty_batches():
    hv, swarm, jobs = make_plan()
    hv.mapping_mode = None
    with pytest.raises(PlanSnapshotError, match="不是航測建圖模式"):
        snap_of(hv, swarm, jobs)
    hv, swarm, jobs = make_plan()
    swarm.hv_coordinator = object()
    with pytest.raises(PlanSnapshotError, match="計畫已換成另一份"):
        snap_of(hv, swarm, jobs)
    hv, swarm, _jobs = make_plan()
    with pytest.raises(PlanSnapshotError, match="這批沒有任何一架真的送出任務"):
        snap_of(hv, swarm, [])
    hv, swarm, jobs = make_plan()
    jobs[0]["uav_id"] = 9
    with pytest.raises(PlanSnapshotError, match="UAV-11 上傳的是機 9"):
        snap_of(hv, swarm, jobs)


def test_build_snapshot_from_plain_data_only(tmp_path):
    """不經 hv：純資料也能建（給以後的匯出路徑或其他規劃器）。"""
    ring = [list(ll(0, 0)), list(ll(100, 0)), list(ll(100, 100))]
    snap = ps.build_snapshot(
        planner={"name": "手動"}, areas={"source": "測試", "items": [ring]},
        aircraft={3: {"uav_id": "X", "cls": "B", "strips": []}},
        mission_uploads=[{"sysid": 5, "drone_id": 3, "items": [{"command": 16, "param1": 0.0}],
                          "connection": "COM36"}],
        mapping={"camera": "HQ6"}, wind={"source": "無風", "planning_speed_mps": 0.0},
        gcp_blocks={"source": "無", "blocks": {}, "info": {}}, uploaded_at_utc=T_UP, software=SW)
    assert snap["plan"]["areas"]["items"][0] == {"area_id": "area_1", "polygon": ring, "holes": []}
    assert snap["plan"]["terrain"] is None and snap["plan"]["takeoff_schedule"] is None
    snap = ps.set_upload_status(snap, 5, ok=True)
    assert ps.read_snapshot(ps.write_snapshot(tmp_path, snap)) == snap
    with pytest.raises(PlanSnapshotError, match="計畫裡沒有這架"):
        ps.build_snapshot(planner={"name": "手動"}, areas={"source": "測試", "items": [ring]},
                          aircraft={3: {"uav_id": "X", "cls": "B", "strips": []}},
                          mission_uploads=[{"sysid": 5, "drone_id": 4, "items": [{"command": 16}]}],
                          mapping={"camera": "HQ6"}, wind={"source": "無風"},
                          gcp_blocks={"source": "無", "blocks": {}, "info": {}}, software=SW)
    with pytest.raises(PlanSnapshotError, match="沒有來源說明"):
        ps.build_snapshot(planner={"name": "手動"}, areas={"items": [ring]},
                          aircraft={3: {"uav_id": "X", "cls": "B", "strips": []}},
                          mission_uploads=[{"sysid": 5, "drone_id": 3, "items": [{"command": 16}]}],
                          mapping={"camera": "HQ6"}, wind={"source": "無風"},
                          gcp_blocks={"source": "無", "blocks": {}, "info": {}}, software=SW)


def test_source_record_and_summary_for_the_dialog(tmp_path):
    snap = acked(snap_of(), 11)
    path = ps.write_snapshot(tmp_path, snap)
    rec = ps.source_record(snap, path)
    assert rec == {"kind": "plan_snapshot", "format": ps.FORMAT, "file": path.name,
                   "plan_sha256": snap["plan_sha256"], "digest_sha256": snap["digest_sha256"],
                   "uploaded_at_utc": T_UP, "n_areas": 1, "n_area_holes_ignored": 0,
                   "upload_ok_sysids": [11], "upload_failed_sysids": [], "upload_unreported_sysids": [12]}
    assert str(tmp_path) not in json.dumps(rec, ensure_ascii=False)        # 只放檔名，不放絕對路徑
    lines = ps.summary_lines(snap, path)
    assert lines[0] == ps.source_label(snap, path)
    assert "UAV-11←機 1 ✔" in lines[2] and "UAV-12←機 2 未回報" in lines[2]
    assert lines[3] == "GCP 區塊：2 架"


def test_software_version_is_recorded_without_touching_the_index():
    sw = ps.software_version()
    assert set(sw) == {"aeroplan_git_describe", "aeroplan_git_error", "python"}
    assert (sw["aeroplan_git_describe"] is None) != (sw["aeroplan_git_error"] is None)
    snap = ps.snapshot_from_hv(*make_plan(), takeoff_offsets={}, takeoff_source="t", uploaded_at_utc=T_UP)
    assert snap["software"] == sw and "software" not in snap["plan"]
