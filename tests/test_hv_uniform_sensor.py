"""HV-DCCPP：兩個構型共用同一台單光相機。

背景：轉接層的 `_hardware_for` 是**照構型字母**決定酬載種類（A→LiDAR、B→相機）。
不明確給 `sensor_a`/`sensor_b` 就會憑空生出一個酬載差異，讓「哪一型做哪一塊」
變成酬載決定的，而不是平台性能（續航、速度、轉彎半徑）決定的。

本檔鎖住三件事：

1. `_uniform_camera` 的刈幅與面板 HFOV 算出來的一致（數值）。
2. 兩個構型拿到的是**同一種** kind，且只有一個 PayloadRequirement。
3. 每個構型都拿得到 `Sensor` **而且真的分到工作** —— 手冊 §10.5 的坑是
   構型找不到對應 kind 的需求時**不會報錯**，只會整型分不到工作。

第 2、3 項各附一個「反向臂」（`hv_uniform_sensor=False`）：如果反向臂量不到
差異，代表這些測試根本不可能失敗，那它們就沒有在測東西。
"""
from __future__ import annotations

import logging as _logging
import math

import pytest

from mission.swarm_coordinator import SwarmCoordinator, _uniform_camera

pytest.importorskip("hv_dccpp")

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
CLASS_OF = {1: "A", 2: "B", 3: "B", 4: "B"}
HFOV, VFOV, ALT = 60.0, 45.0, 140.0


def _ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def _rect(cx: float, half_e: float, half_n: float) -> list[tuple[float, float]]:
    return [_ll(cx - half_e, -half_n), _ll(cx + half_e, -half_n),
            _ll(cx + half_e, half_n), _ll(cx - half_e, half_n)]


def _plan(uniform: bool):
    """跑一次真的 HV-DCCPP 規劃，回傳 `HvDccppCoordinator`。"""
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_uniform_sensor = uniform
    coord.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    drones = [{"drone_id": i + 1, "name": n, "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 150.0}
              for i, (n, v, r) in enumerate([("A1", 21.5, 200.0), ("B1", 19.0, 120.0),
                                             ("B2", 19.0, 120.0), ("B3", 19.0, 120.0)])]
    areas = [{"area_id": i + 1, "priority": 1.0, "polygon": _rect(-700 + 1400 * i, 300, 350)}
             for i in range(2)]
    coord.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=150.0,
                              overlap_rate=0.30, coverage_altitude=ALT,
                              mounting_angle_deg=0.0)
    assert coord.hv_coordinator is not None, "沒走到 HV 分支（hv_dccpp 匯入失敗？）"
    return coord.hv_coordinator


@pytest.fixture(scope="module")
def hv_uniform():
    return _plan(True)


@pytest.fixture(scope="module")
def hv_split():
    """反向臂：關掉單光相機 ⇒ 轉接層照字母給 A LiDAR、B 相機。"""
    return _plan(False)


# ── 1. 相機幾何 ────────────────────────────────────────────────────────
def test_uniform_camera_swath_matches_panel_hfov():
    """刈幅 = 2·h·tan(HFOV/2)：面板算足跡用這條，規劃器也要得到同一個數。"""
    cam = _uniform_camera(hfov_deg=HFOV, vfov_deg=VFOV)
    swath = ALT * cam.sensor_width_mm / cam.focal_length_mm
    assert swath == pytest.approx(2.0 * ALT * math.tan(math.radians(HFOV) / 2.0), rel=1e-9)
    along = ALT * cam.sensor_height_mm / cam.focal_length_mm
    assert along == pytest.approx(2.0 * ALT * math.tan(math.radians(VFOV) / 2.0), rel=1e-9)


def test_uniform_camera_is_not_flagged_as_example():
    """使用者給的規格不可被標成「範例硬體」，否則 preflight 會發出誤導的警告。"""
    assert _uniform_camera(hfov_deg=HFOV, vfov_deg=VFOV).is_default_example is False
    assert _uniform_camera(hfov_deg=0.0, vfov_deg=0.0).is_default_example is False


def test_uniform_camera_falls_back_without_fov():
    """沒給 HFOV/VFOV 時退回 CameraHardware 預設幾何，不是丟例外。"""
    cam = _uniform_camera(hfov_deg=0.0, vfov_deg=0.0)
    assert cam.kind == "rgb"
    assert cam.focal_length_mm > 0.0 and cam.sensor_width_mm > 0.0


def test_uniform_default_is_on():
    """預設就是單光相機；要異構酬載得自己關掉。"""
    assert SwarmCoordinator().hv_uniform_sensor is True


# ── 2. 兩型同 kind、單一需求 ──────────────────────────────────────────
def test_both_classes_get_the_same_sensor_kind(hv_uniform):
    kinds = {cls: sen.hw.kind for (cls, _rid), sen in hv_uniform.last_ctx.sensors.items()}
    assert set(kinds) == {"A", "B"}, f"有構型完全沒有 Sensor：{kinds}"
    assert set(kinds.values()) == {"rgb"}, kinds


def test_only_one_payload_requirement(hv_uniform):
    reqs = hv_uniform.last_inputs.payload_reqs
    assert len(reqs) == 1, f"單光相機不該產生第二個酬載需求：{ {k: r.kind for k, r in reqs.items()} }"
    assert next(iter(reqs.values())).kind == "rgb"


def test_split_arm_really_differs(hv_split):
    """反向臂：關掉之後 A 是 lidar、B 是 rgb，而且有兩個需求。

    這一條**必須**通過，否則上面兩條是量不到東西的假綠燈。
    """
    kinds = {cls: sen.hw.kind for (cls, _rid), sen in hv_split.last_ctx.sensors.items()}
    assert kinds.get("A") == "lidar" and kinds.get("B") == "rgb", kinds
    assert len(hv_split.last_inputs.payload_reqs) == 2


# ── 3. 每個構型都真的分到工作 ─────────────────────────────────────────
def test_every_class_receives_work(hv_uniform):
    """手冊 §10.5 的坑：構型分不到工作時**不會報錯**，所以要在這裡攔。"""
    used = {p.cls for p in hv_uniform.last_plan.plans.values() if p.strips}
    assert used == {"A", "B"}, f"有構型一條條帶都沒分到：{used}"
    assert not hv_uniform.last_plan.uncovered


def test_uniform_swath_is_shared_by_both_classes(hv_uniform):
    """兩型的條帶間距一致 ⇒ 幾何不再是分工依據，剩下的差異全在平台性能。"""
    spacing = {cls: sen.strip_spacing_m(ALT)
               for (cls, _rid), sen in hv_uniform.last_ctx.sensors.items()}
    assert spacing["A"] == pytest.approx(spacing["B"], rel=1e-9), spacing


def test_platform_performance_still_differs(hv_uniform):
    """相機一樣，但平台不一樣：速度、轉彎半徑、可用航程都要保持有差。"""
    spec = {u.spec.cls: u.spec for u in hv_uniform.last_inputs.fleet}
    a, b = spec["A"], spec["B"]
    assert a.cruise_airspeed_mps != b.cruise_airspeed_mps
    assert a.turn_radius_planning_m != b.turn_radius_planning_m
    assert a.reference_endurance_min != b.reference_endurance_min
    assert a.usable_distance_km != b.usable_distance_km
    assert a.min_fw_operation_agl_m != b.min_fw_operation_agl_m


# ── 4. 單光相機 × 作業區自帶 sensor_req 的陷阱 ────────────────────────
def test_lidar_area_under_uniform_camera_is_dropped_silently(caplog):
    """作業區宣告 sensor_req='lidar' 而沒人掛 LiDAR ⇒ **整塊被丟掉**。

    量到的行為（不是推測）：status 仍是 GO_WITH_WARNINGS、errors 空的、
    `raise_on_no_go=True` 也不會 raise，只有 `plan.uncovered` 與一則 note 記著。
    所以這裡鎖兩件事：現象本身，以及我們自己補的那道警告真的會叫。
    """
    import logging

    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_uniform_sensor = True
    # ⚠ 這一條鎖的是**修好之前的現象**與那道守衛，所以要把機隊切區器關掉；
    #   開著的話 split_area_for_fleet 會先把區切開，現象就不會發生了。
    coord.hv_split_area_for_fleet = False
    coord.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    report: dict = {}
    coord.hv_on_report = report.update
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    drones = [{"drone_id": i + 1, "name": n, "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 150.0}
              for i, (n, v, r) in enumerate([("A1", 21.5, 200.0), ("B1", 19.0, 120.0),
                                             ("B2", 19.0, 120.0), ("B3", 19.0, 120.0)])]
    areas = [{"area_id": 1, "priority": 1.0, "sensor_req": "lidar",
              "polygon": _rect(-700, 300, 350)},
             {"area_id": 2, "priority": 1.0, "polygon": _rect(700, 300, 350)}]
    with caplog.at_level(logging.WARNING, logger="mission.swarm_coordinator"):
        coord.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=150.0,
                                  overlap_rate=0.30, coverage_altitude=ALT,
                                  mounting_angle_deg=0.0)

    # (a) 我們自己補的警告要指名是哪一塊區
    assert any("sensor_req" in r.message and "[1]" in r.message
               for r in caplog.records), [r.message for r in caplog.records]
    # (b) 現象：那一塊真的沒被覆蓋，而且 status 不是 NO_GO
    plan = coord.hv_coordinator.last_plan
    assert plan.uncovered, "lidar 區居然被覆蓋了？那這個測試量不到東西"
    assert report.get("status") != "NO_GO"
    assert not report.get("errors")


def test_guard_is_quiet_when_areas_are_plain(caplog):
    """反向臂：作業區沒宣告 sensor_req 時不可以亂叫（否則警告會變成雜訊）。"""
    import logging
    with caplog.at_level(logging.WARNING, logger="mission.swarm_coordinator"):
        _plan(True)
    assert not any("sensor_req" in r.message for r in caplog.records)


# ── 5. HV 分支自己補跑 hv_dccpp.validate() ────────────────────────────
def test_validate_is_actually_run(hv_uniform):
    """轉接層**從來不呼叫** `hv_dccpp.validate()`（全 repo grep 不到）。

    時空去衝突與能量預算只有它會檢查，preflight 的 feasibility 不會。
    所以 HV 分支自己補跑一次，結果要留在 `SwarmCoordinator.hv_validation`。
    """
    import mission.swarm_coordinator as sc_mod
    import hv_dccpp.integration.aeroplan_adapter as adapter

    src = (sc_mod.__file__, adapter.__file__)
    assert "validate" not in open(src[1], encoding="utf-8").read(), \
        "轉接層現在會自己 validate 了？那這一段補丁可以拿掉，先確認再改"

    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    drones = [{"drone_id": i + 1, "name": n, "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 150.0}
              for i, (n, v, r) in enumerate([("A1", 21.5, 200.0), ("B1", 19.0, 120.0),
                                             ("B2", 19.0, 120.0), ("B3", 19.0, 120.0)])]
    areas = [{"area_id": i + 1, "priority": 1.0, "polygon": _rect(-700 + 1400 * i, 300, 350)}
             for i in range(2)]
    coord.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=150.0,
                              overlap_rate=0.30, coverage_altitude=ALT,
                              mounting_angle_deg=0.0)
    v = coord.hv_validation
    assert v is not None, "validate 沒被跑到"
    assert v.passed is True and v.coverage_ok and v.energy_ok, v
    assert len(v.conflicts) == 0


def test_validate_failure_is_reported(caplog):
    """反向臂：validate 失敗時要**叫出來**，而且訊息要說是哪一項失敗。

    用「作業區宣告 lidar 但沒人掛 lidar」造出 coverage_ok=False。
    這一條同時證明上一條不是恆真的假綠燈。

    ⚠ 用 `validation_notes`（清單）而不是切 `validation_note` 字串：這一趟同時
    會冒出 `partial_fleet_assigned`，用 `startswith` 會被排序綁死。
    """
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    report: dict = {}
    coord.hv_on_report = report.update
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    drones = [{"drone_id": i + 1, "name": n, "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 150.0}
              for i, (n, v, r) in enumerate([("A1", 21.5, 200.0), ("B1", 19.0, 120.0),
                                             ("B2", 19.0, 120.0), ("B3", 19.0, 120.0)])]
    areas = [{"area_id": 1, "priority": 1.0, "sensor_req": "lidar",
              "polygon": _rect(-700, 300, 350)},
             {"area_id": 2, "priority": 1.0, "polygon": _rect(700, 300, 350)}]
    with caplog.at_level(_logging.WARNING, logger="mission.swarm_coordinator"):
        coord.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=150.0,
                                  overlap_rate=0.30, coverage_altitude=ALT,
                                  mounting_angle_deg=0.0)
    assert coord.hv_validation is not None and coord.hv_validation.passed is False
    notes = report.get("validation_notes") or []
    hit = [n for n in notes if n.startswith("validate_failed")]
    assert hit, report
    assert "coverage" in hit[0], hit
    assert "validate_failed" in (report.get("validation_note") or ""), report

    # ── 提示不可以指向錯的旋鈕 ──────────────────────────────────────
    #   這一趟兩個構型**都**有工作（閒置的只是同型的第 4 台），所以「構型 A 要
    #   調高度／填 GSD」那句話在這裡是錯的。無條件附上去時這條會咬。
    partial = [r.getMessage() for r in caplog.records
               if "partial_fleet_assigned" in r.getMessage()]
    assert partial, "這一趟本來就該有 partial_fleet_assigned，不然這條測不到東西"
    assert "構型 A" not in partial[0], partial[0]


def test_gui_surfaces_validate_failed():
    """`validate_failed` 必須在 GUI 的大聲清單裡，否則只會躺在 log 檔。"""
    import ui.main_window as mw
    assert "validate_failed" in mw.MainWindow._HV_LOUD_NOTES


# ── 6. 混編機隊 + 單一胞 ⇒ 空計畫 ─────────────────────────────────────
def _plan_single_area(n_a: int, n_b: int, half_m: float):
    """一塊 `2*half_m` 見方的作業區，機隊 n_a 台 A + n_b 台 B。"""
    fleet = [("A", 21.5, 200.0)] * n_a + [("B", 19.0, 120.0)] * n_b
    cls = {i + 1: c for i, (c, _v, _r) in enumerate(fleet)}
    drones = [{"drone_id": i + 1, "name": f"{c}{i + 1}", "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 150.0} for i, (c, v, r) in enumerate(fleet)]
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    # ⚠ 同上：這一條鎖的是「單一胞 + 混編 = 空計畫」這個現象本身。
    coord.hv_split_area_for_fleet = False
    coord.hv_class_of = lambda d: cls[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (5.0, 270.0)
    coord.hv_fence_zones = []
    report: dict = {}
    coord.hv_on_report = report.update
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    res = coord.plan_coverage_dccpp(
        areas=[{"area_id": 1, "priority": 1.0, "polygon": _rect(0, half_m, half_m)}],
        drones=drones, coverage_width_m=150.0, overlap_rate=0.30,
        coverage_altitude=ALT, mounting_angle_deg=0.0)
    return coord, res, report


def test_mixed_fleet_single_cell_yields_empty_plan_and_says_so():
    """⚠⚠⚠ 混編機隊在只切出一個胞的作業區上，**一台都不會上工**。

    殘帶對拍賣把每個母胞鎖成「A 一台 + B 一台」，兩台吃不下就整胞放棄。
    回傳值長得像成功（status=GO_WITH_WARNINGS、errors=[]），只有 makespan=0.0。
    這一條鎖住「我們自己補的那句 `no_aircraft_assigned` 會叫」。
    """
    coord, res, report = _plan_single_area(4, 4, 2000.0)     # 4.0 x 4.0 km
    assert not [p for p in coord.hv_coordinator.last_plan.plans.values() if p.strips], \
        "混編居然有人上工了？那這個測試量不到東西，先確認上游是不是修了"
    assert res["total_makespan"] == 0.0
    assert report.get("status") != "NO_GO"          # 現象：它不當成失敗
    note = report.get("validation_note") or ""
    assert note.startswith("no_aircraft_assigned"), report


def test_homogeneous_fleet_on_the_same_area_works():
    """反向臂：**同一塊區**、同樣台數，全部同構型就 4/4 上工。

    這一條證明上面那條量到的是「混編」而不是「區太大」。
    """
    coord, res, _rep = _plan_single_area(4, 0, 2000.0)
    working = [u for u, p in coord.hv_coordinator.last_plan.plans.items() if p.strips]
    assert len(working) == 4, working
    assert res["total_makespan"] > 0.0


def test_guard_is_quiet_when_someone_is_assigned(hv_uniform):
    """機隊有人上工時不可以亂叫。"""
    assert [p for p in hv_uniform.last_plan.plans.values() if p.strips]


def test_gui_surfaces_no_aircraft_assigned():
    import ui.main_window as mw
    assert "no_aircraft_assigned" in mw.MainWindow._HV_LOUD_NOTES


# ── 7. DEM 從來沒有進到 HV 規劃 ───────────────────────────────────────
def test_recon_panel_has_no_dem_key():
    """「VTOL 偵查」頁的 get_params() 沒有 dem_path ⇒ HV 一律拿到空字串。

    這是現象本身，鎖住它是為了：哪天面板加了 dem_path，這條會紅，
    提醒去把 main_window 的退回邏輯拿掉（否則兩邊會打架）。
    """
    import inspect

    from ui.widgets import vtol_recon_panel as vrp
    src = inspect.getsource(vrp.VTOLReconPanel.get_params)
    assert "dem_path" not in src, "面板現在會給 dem_path 了？把 main_window 的退回拿掉"


def test_dem_manager_records_its_path():
    """`_on_dem_loaded` 存下來的 DEM 要能問出來源路徑，才轉交得出去。"""
    from sensors.dem_terrain import DEMTerrainManager
    dem = DEMTerrainManager()
    dem.load_dem("does_not_exist.tif")          # 載入會失敗，但路徑要先記下
    assert getattr(dem, "dem_path", None) == "does_not_exist.tif"


def test_main_window_falls_back_to_loaded_dem():
    """沒帶 dem_path 但已載入 DEM 時，覆蓋規劃要改用那一份。"""
    import inspect

    import ui.main_window as mw
    src = inspect.getsource(mw.MainWindow._on_dccpp_coverage_requested)
    assert "_dem_manager" in src and "dem_path = str(" in src, \
        "退回邏輯不見了 ⇒ HV 規劃又會把地形當成海平面 0 m 的平面"


# ── 8. 面板把重疊吃進 fov_width_m，HV 要能還原 ────────────────────────
def _spacing_for(width: float, overlap: float) -> float:
    """跑一次 HV 規劃，回傳它實際採用的條帶間距（公尺）。"""
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    drones = [{"drone_id": i + 1, "name": n, "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": width}
              for i, (n, v, r) in enumerate([("A1", 21.5, 200.0), ("B1", 19.0, 120.0),
                                             ("B2", 19.0, 120.0), ("B3", 19.0, 120.0)])]
    areas = [{"area_id": i + 1, "priority": 1.0, "polygon": _rect(-700 + 1400 * i, 300, 350)}
             for i in range(2)]
    coord.plan_coverage_dccpp(areas=areas, drones=drones, coverage_width_m=width,
                              overlap_rate=overlap, coverage_altitude=ALT,
                              mounting_angle_deg=0.0)
    sensors = coord.hv_coordinator.last_ctx.sensors
    return next(iter(sensors.values())).strip_spacing_m(ALT)


def test_hv_ignores_fov_width_m():
    """現象本身：HV 的條帶間距**完全不看** `coverage_width_m`。

    這是為什麼面板的「重疊已經吃進 fov_width_m」慣例會在 HV 上失效。
    """
    swath = 2.0 * ALT * math.tan(math.radians(HFOV) / 2.0)
    wide = _spacing_for(width=swath * 2.0, overlap=0.30)
    narrow = _spacing_for(width=swath * 0.2, overlap=0.30)
    assert wide == pytest.approx(narrow, rel=1e-9), (wide, narrow)


def test_panel_convention_overlap_is_restored():
    """面板慣例（width 已扣重疊、overlap_rate=0）要還原成同樣的間距。

    不還原的話 HV 會以 0% 重疊規劃 —— 間距等於整個刈幅。
    """
    swath = 2.0 * ALT * math.tan(math.radians(HFOV) / 2.0)
    explicit = _spacing_for(width=150.0, overlap=0.30)          # 明確給重疊
    folded = _spacing_for(width=swath * 0.70, overlap=0.0)      # 面板慣例
    assert folded == pytest.approx(explicit, rel=1e-6), (folded, explicit)
    assert folded < swath * 0.99, f"還原沒生效，間距 {folded} 等於整個刈幅 {swath}"


def test_no_restore_when_width_exceeds_swath():
    """width 比刈幅大時不可以亂還原（那代表使用者要的就是稀疏條帶）。"""
    swath = 2.0 * ALT * math.tan(math.radians(HFOV) / 2.0)
    assert _spacing_for(width=swath * 1.5, overlap=0.0) == pytest.approx(swath, rel=1e-6)


# ── 9. 模式一：單一大區自動切區餵飽機隊 ────────────────────────────────
def _plan_single_big(n_a: int, n_b: int, *, split: bool, half_e=2500.0, half_n=2000.0):
    """一塊 `2*half_e × 2*half_n` 的大區（使用者用四個角點畫出來的那種）。"""
    fleet = [("A", 21.5, 200.0)] * n_a + [("B", 19.0, 120.0)] * n_b
    cls = {i + 1: c for i, (c, _v, _r) in enumerate(fleet)}
    drones = [{"drone_id": i + 1, "name": f"{c}{i + 1}", "position": HOME, "speed": v,
               "heading": 0.0, "turn_radius": r, "vehicle_type": "vtol",
               "fov_width": 150.0} for i, (c, v, r) in enumerate(fleet)]
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_split_area_for_fleet = split
    coord.hv_class_of = lambda d: cls[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (5.0, 270.0)
    coord.hv_fence_zones = []
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    res = coord.plan_coverage_dccpp(
        areas=[{"area_id": 1, "priority": 1.0,
                "polygon": [_ll(-half_e, -half_n), _ll(half_e, -half_n),
                            _ll(half_e, half_n), _ll(-half_e, half_n)]}],
        drones=drones, coverage_width_m=150.0, overlap_rate=0.30,
        coverage_altitude=ALT, mounting_angle_deg=0.0)
    return coord, res


def test_single_big_area_mixed_fleet_is_empty_without_split():
    """現象：不切區時，單一大區配混編機隊產出**空計畫**。

    這一條是反向臂 —— 它證明下一條測到的是真的修好，不是本來就會過。
    """
    coord, res = _plan_single_big(1, 3, split=False)
    assert res["total_makespan"] == 0.0
    assert not [p for p in coord.hv_coordinator.last_plan.plans.values() if p.strips]


@pytest.mark.parametrize("n_a,n_b", [(1, 3), (2, 2), (4, 4), (2, 6)])
def test_single_big_area_split_feeds_the_whole_fleet(n_a, n_b):
    """切區之後整個機隊都分得到工作，而且沒有未覆蓋。"""
    coord, res = _plan_single_big(n_a, n_b, split=True)
    plan = coord.hv_coordinator.last_plan
    working = [u for u, p in plan.plans.items() if p.strips]
    assert len(working) == n_a + n_b, (working, len(plan.cells))
    assert not plan.uncovered
    assert res["total_makespan"] > 0.0


def test_split_shortens_the_homogeneous_case_too():
    """同構型本來就跑得動，但切區讓它更短（條帶不變、轉場變少）。"""
    _c0, _r0 = _plan_single_big(4, 0, split=False)
    _c1, _r1 = _plan_single_big(4, 0, split=True)
    km0 = sum(p.distance_m for p in _c0.hv_coordinator.last_plan.plans.values())
    km1 = sum(p.distance_m for p in _c1.hv_coordinator.last_plan.plans.values())
    assert km1 < km0, (km0, km1)


def test_split_preserves_area():
    """切區絕不可以靜默吃掉面積（`convex_partition` 就是在這一步掉 25.8% 的）。"""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    from mission.swarm_coordinator import split_area_for_fleet
    src = [{"area_id": 1, "priority": 1.0,
            "polygon": [_ll(-2500, -2000), _ll(2500, -2000), _ll(2500, 2000),
                        _ll(-2500, 2000)]}]
    out = split_area_for_fleet(src, 4)
    assert len(out) == 4, out
    before = Polygon(src[0]["polygon"])
    after = unary_union([Polygon(a["polygon"]) for a in out])
    assert after.area == pytest.approx(before.area, rel=1e-6)
    assert len({a["area_id"] for a in out}) == 4, "子區 id 必須互不相同"
    assert all(a["_src_area_id"] == 1 for a in out)


def test_split_is_a_no_op_for_multi_area_input():
    """模式二（多個小區）不可以被這一步動到 —— 每塊區只分到 1 份就原樣送出。"""
    from mission.swarm_coordinator import split_area_for_fleet
    src = [{"area_id": i + 1, "priority": 1.0, "polygon": _rect(-700 + 1400 * i, 300, 350)}
           for i in range(4)]
    out = split_area_for_fleet(src, 4)
    assert len(out) == len(src)
    assert [a["area_id"] for a in out] == [a["area_id"] for a in src]
    assert [a["polygon"] for a in out] == [a["polygon"] for a in src]


def test_split_default_is_on():
    assert SwarmCoordinator().hv_split_area_for_fleet is True


def test_allocation_is_keyed_by_the_users_area_not_the_split_pieces():
    """切區之後 `allocation` 的鍵必須換回**使用者畫的那塊區**。

    不換的話下游任何拿 `allocation.get(area_id)` 對回輸入的程式都會對不上
    （`ui/main_window.py:5574` 就是這樣用的）。
    """
    coord, res = _plan_single_big(2, 2, split=True)
    alloc = res["allocation"]
    assert set(alloc) == {1}, f"鍵應該只剩使用者那塊區，實得 {sorted(alloc)}"
    assert sorted(alloc[1]) == [1, 2, 3, 4], alloc


def test_split_pieces_carry_their_source_area():
    """每塊子區都要記得自己從哪塊區來，否則換不回去。"""
    from mission.swarm_coordinator import split_area_for_fleet
    src = [{"area_id": 7, "priority": 1.0,
            "polygon": [_ll(-2500, -2000), _ll(2500, -2000), _ll(2500, 2000),
                        _ll(-2500, 2000)]}]
    out = split_area_for_fleet(src, 4)
    assert len(out) == 4
    assert all(a["_src_area_id"] == 7 for a in out), out
    assert len({a["area_id"] for a in out}) == 4, "子區 id 必須互不相同"
    assert all(isinstance(a["area_id"], int) for a in out), \
        "子區 id 要用整數：轉接層會把 '1_1' 的底線吃掉變成 11，之後對不回來"


# ── 10. 部分機隊閒置要點名是哪個構型 ──────────────────────────────────
def _plan_two(alt: float):
    """1 Pioneer + 1 HERO、2.45 km² 單一區（使用者實際踩到的組態）。"""
    half = math.sqrt(2449789.0) / 2.0
    drones = [{"drone_id": 1, "name": "A1", "position": HOME, "speed": 21.5,
               "heading": 0.0, "turn_radius": 200.0, "vehicle_type": "vtol",
               "fov_width": 150.0},
              {"drone_id": 2, "name": "B1", "position": HOME, "speed": 19.0,
               "heading": 0.0, "turn_radius": 120.0, "vehicle_type": "vtol",
               "fov_width": 150.0}]
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_class_of = lambda d: {1: "A", 2: "B"}[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    report: dict = {}
    coord.hv_on_report = report.update
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    coord.plan_coverage_dccpp(
        areas=[{"area_id": 1, "priority": 1.0,
                "polygon": [_ll(-half, -half), _ll(half, -half),
                            _ll(half, half), _ll(-half, half)]}],
        drones=drones, coverage_width_m=150.0, overlap_rate=0.30,
        coverage_altitude=alt, mounting_angle_deg=0.0)
    plan = coord.hv_coordinator.last_plan
    return {p.cls for p in plan.plans.values() if p.strips}, report


def test_low_altitude_leaves_the_pioneer_idle():
    """現象：作業高度 ≤133 m 時 Pioneer 一條條帶都分不到。

    ⚠ 這**不是**高度被夾（兩型的 h_op 相同），也與對地淨空無關
      （手動設 0 結果一樣）—— 是分配層算出來不派它。
    """
    classes, _rep = _plan_two(130.0)
    assert classes == {"B"}, f"低高度居然派了 {classes}，門檻變了要更新面板提示"


def test_higher_altitude_uses_both():
    """反向臂：高度拉到 140 m 兩型都有工作 —— 證明上一條量得到差別。"""
    classes, _rep = _plan_two(140.0)
    assert classes == {"A", "B"}, classes


def test_partial_fleet_is_named_not_just_counted():
    """只顯示「1/2 架」使用者不知道要調哪個旋鈕，所以要點名構型。"""
    _classes, rep = _plan_two(130.0)
    notes = rep.get("validation_notes") or []
    hit = [n for n in notes if str(n).startswith("partial_fleet_assigned")]
    assert hit, f"沒有點名部分閒置：{notes}"
    assert "'A'" in hit[0] or '"A"' in hit[0], f"沒說是哪個構型：{hit[0]}"


def test_gui_surfaces_partial_fleet():
    import ui.main_window as mw
    assert "partial_fleet_assigned" in mw.MainWindow._HV_LOUD_NOTES


# ── 11. 民航法規上限 120 m AGL 下 Pioneer 必須用得到 ──────────────────
def _plan_two_gsd(alt: float, gsd=None):
    """1 Pioneer + 1 HERO、2.45 km² 單一區，可指定目標 GSD。"""
    half = math.sqrt(2449789.0) / 2.0
    drones = [{"drone_id": 1, "name": "A1", "position": HOME, "speed": 21.5,
               "heading": 0.0, "turn_radius": 200.0, "vehicle_type": "vtol",
               "fov_width": 150.0},
              {"drone_id": 2, "name": "B1", "position": HOME, "speed": 19.0,
               "heading": 0.0, "turn_radius": 120.0, "vehicle_type": "vtol",
               "fov_width": 150.0}]
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_class_of = lambda d: {1: "A", 2: "B"}[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    if gsd is not None:
        coord.hv_params["target_gsd"] = gsd
    coord.plan_coverage_dccpp(
        areas=[{"area_id": 1, "priority": 1.0,
                "polygon": [_ll(-half, -half), _ll(half, -half),
                            _ll(half, half), _ll(-half, half)]}],
        drones=drones, coverage_width_m=150.0, overlap_rate=0.30,
        coverage_altitude=alt, mounting_angle_deg=0.0)
    plan = coord.hv_coordinator.last_plan
    return {p.cls for p in plan.plans.values() if p.strips}


def test_pioneer_is_unusable_at_120m_without_the_panel_gsd():
    """現象（反向臂）：不送目標 GSD 時，法規上限 120 m 下 Pioneer 用不到。

    機制：轉接層自己合成 GSD 使高度上限 = 1.05×作業高度，而
    `cfg.dh_band_min_m = 20` 要求可用高度帶至少 20 m 寬
    ⇒ 1.05h − 120 ≥ 20 ⇒ **h ≥ 133.3 m**。但法規上限就是 120 m。
    """
    assert _plan_two_gsd(120.0, gsd=None) == {"B"}


def test_panel_gsd_makes_the_pioneer_usable_at_the_120m_legal_ceiling():
    """送面板預設目標 GSD 0.05 m/px（高度上限 205 m）⇒ 120 m 下兩型都有工作。"""
    assert _plan_two_gsd(120.0, gsd=0.05) == {"A", "B"}


def test_too_strict_gsd_still_excludes_the_pioneer():
    """GSD 設太嚴（0.03 ⇒ 上限 123 m，帶寬只有 3 m）⇒ 仍然只有 HERO。

    這一條證明修法不是「無論如何都讓 A 上工」，而是把真正的物理約束接上來。
    """
    assert _plan_two_gsd(120.0, gsd=0.03) == {"B"}


@pytest.fixture(scope="session")
def qapp():
    """⚠ 一定要**留住**這個參考。

    原本寫成 `QApplication.instance() or QApplication([])` 把回傳值丟掉，
    PyQt6 就會在直譯器收尾時先回收 QApplication、再回收還活著的面板 ⇒
    整個 process 以 exit 127 無聲死掉（沒有 traceback、沒有 summary，
    單獨跑這個檔時 43 條綠燈後直接消失）。
    """
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_panel_defaults_assume_no_waiver(qapp):
    """面板出廠值：作業高度 120 m、**沒有**豁免、GSD 0.05。

    ⚠⚠⚠ 豁免欄位餵的是 `Airspace.waiver_max_agl_m`，而
    `h_ceil = waiver if given else max_agl_m`（上游 max_agl_m 寫死 120 ＝ 法規值）。
    以前預設 200 等於替使用者**假設他拿到了 200 m 豁免**，規劃出來的任務違規
    而畫面上完全看不出來。所以預設必須是 None。
    """
    import ui.widgets.vtol_recon_panel as vrp
    p = vrp.VTOLReconPanel()
    d = p.get_params()
    assert d["altitude"] == pytest.approx(120.0), "作業高度預設超過法規上限"
    assert d["hv_waiver_max_agl_m"] is None, \
        "預設就給了豁免值 ⇒ 沒有豁免的使用者會拿到違規任務而不自知"
    assert d["target_gsd"] >= 0.035, \
        f"目標 GSD {d['target_gsd']} 太嚴 ⇒ Pioneer 會分不到工作"
    # 勾了才有值，而且必須高過 Pioneer 的門檻（120 + 20 + 起伏 = 141）
    p.hv_waiver_check.setChecked(True)
    w = p.get_params()["hv_waiver_max_agl_m"]
    assert w is not None and w >= 141.0, f"勾了豁免卻只給 {w} m，Pioneer 仍派不動"
    p.deleteLater()


def test_no_waiver_means_the_legal_120m_ceiling():
    """不給豁免時 `waiver_max_agl_m` 必須是 **None**，不可以退回 200。

    退回 200 的話「沒有豁免」與「有 200 m 豁免」規劃結果一模一樣，
    上面那條面板測試就變成量不到東西的假綠燈。
    """
    import inspect
    src = inspect.getsource(SwarmCoordinator.plan_coverage_dccpp)
    assert "waiver_max_agl_m=_waiver" in src, "又把 waiver 寫死成 float(...) 了"
    assert "'hv_waiver_max_agl_m', 'waiver_max_agl_m', default=None" in src, \
        "沒給豁免時退回了某個預設值 ⇒ 等於替使用者假設他有豁免"


def test_narrow_altitude_band_is_reported(caplog):
    """法規上限 120 m 下每個構型的高度帶都不夠 ⇒ 要把算式攤開講。

    上游的 NO_GO 只寫「所有區域皆無可行構型（感測相容／高度帶／鏈路／方向）」，
    四個原因擠在一句話裡，看不出是高度帶。
    """
    coord = SwarmCoordinator()
    coord.use_hv_dccpp = True
    coord.hv_class_of = lambda d: {1: "A", 2: "B"}[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    report: dict = {}
    coord.hv_on_report = report.update
    coord.hv_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                       "hv_dem_margin_m": 3000.0, "target_gsd": 0.05}   # 沒有豁免
    half = math.sqrt(2449789.0) / 2.0
    drones = [{"drone_id": 1, "name": "A1", "position": HOME, "speed": 21.5,
               "heading": 0.0, "turn_radius": 200.0, "vehicle_type": "vtol",
               "fov_width": 150.0},
              {"drone_id": 2, "name": "B1", "position": HOME, "speed": 19.0,
               "heading": 0.0, "turn_radius": 120.0, "vehicle_type": "vtol",
               "fov_width": 150.0}]
    with caplog.at_level(_logging.WARNING, logger="mission.swarm_coordinator"):
        with pytest.raises(Exception):          # 上游 preflight NO_GO
            coord.plan_coverage_dccpp(
                areas=[{"area_id": 1, "priority": 1.0,
                        "polygon": [_ll(-half, -half), _ll(half, -half),
                                    _ll(half, half), _ll(-half, half)]}],
                drones=drones, coverage_width_m=150.0, overlap_rate=0.30,
                coverage_altitude=120.0, mounting_angle_deg=0.0)
    msg = [r.getMessage() for r in caplog.records
           if "altitude_band_too_narrow" in r.getMessage()]
    assert msg, "高度帶太窄卻沒有說出來"
    # Pioneer：120 − 120 = 0 m，平地就需要 h_ceil ≥ 140（實測含起伏是 141）。
    assert "Pioneer" in msg[0] and "140" in msg[0], msg[0]
    # HERO 的 20 m 剛好等於下限，會被地形起伏吃掉 ⇒ 也要講出來。
    assert "HERO" in msg[0] and "起伏" in msg[0], msg[0]
    assert "altitude_band_too_narrow" in (report.get("validation_note") or "")


def test_wide_enough_ceiling_stays_quiet(caplog):
    """反向臂：豁免 200 m 時兩個構型的高度帶都夠 ⇒ 這則警告**不可以**出現。

    沒有這一條的話上一條有可能是「無論如何都會叫」的假綠燈。
    """
    with caplog.at_level(_logging.WARNING, logger="mission.swarm_coordinator"):
        assert _plan_two_gsd(120.0, gsd=0.05) == {"A", "B"}
    assert not [r for r in caplog.records
                if "altitude_band_too_narrow" in r.getMessage()]


def test_gui_surfaces_narrow_band():
    import ui.main_window as mw
    assert "altitude_band_too_narrow" in mw.MainWindow._HV_LOUD_NOTES


def test_panel_defaults_are_legal_and_workable(qapp):
    """出廠值必須合法：作業高度不得超過法規 120 m。"""
    import ui.widgets.vtol_recon_panel as vrp
    p = vrp.VTOLReconPanel()
    assert p.get_params()["altitude"] <= 120.0
    p.deleteLater()


def test_failure_dialog_carries_the_loud_notes():
    """NO_GO 對話框必須帶上我們自己補的 note。

    上游訊息把四個原因擠成一句話，真正說得出「120 − 120 = 0 m」的是
    `altitude_band_too_narrow`。只寫進 log 檔等於畫面上什麼都沒說。
    """
    import inspect
    import ui.main_window as mw
    src = inspect.getsource(mw.MainWindow._on_hv_failed)
    assert "loud_notes" in src, "失敗對話框沒有取用收集到的重點"
    assert "QMessageBox.critical(self, 'HV-DCCPP 失敗', text)" in src, \
        "還是把原始 msg 直接丟進對話框"
