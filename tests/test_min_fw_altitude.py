"""最低固定翼作業高度（`min_fw_operation_agl_m`）是**可調的規劃值**。

依據（全部逐字查證過）：

* `docs/hv_dccpp/平台參數表.md:34` —— 這一列的使用方式欄是
  「任務前需依地形、感測器與法規調整」，而且它**沒有**像旁邊的
  `transition_start_agl_m` / `transition_complete_agl_m` 那樣標「暫定」。
  它是使用者給的規劃值，全 repo 找不到任何原廠或法規出處。
* 規格 D35：`h_minfw` 只對 ENTRY/OPER/TRANS/RTB 生效；CLIMB/DESCENT 的
  `z_lo` 不含它，ACCEL/DECEL 更是落在 H_tc（A 80–100 m）
  ⇒ **規劃器自己就讓飛機飛在 120 m 以下**。
* 全 repo 沒有任何一句話描述低於它的機體後果（失速／操縱權限／適航）。

但它有真實下游：`Q_RTL_ALT`（`hv_dccpp/ardupilot.py:922`）會進**匯出的 .param**，
每條路徑都生效（HV4SITL8 讀回 `Q_RTL_ALT,100` 已確認）。`FENCE_ALT_MIN`
（`hv_dccpp/mavlink_link.py:996`）也綁著它，但那行只在 `suggest_params` 裡，
而 `suggest_params` 只被 `mavlink_link.upload_plan(write_params/set_params)` 呼叫
—— AeroPlan 的 SITL 與匯出流程都不走那條路（`A1.param` 裡一個都沒有）。
所以調低要大聲講，而且要分開講清楚哪個真的會生效。

⚠⚠⚠ 本檔每一條會規劃的測試都**先斷言旋鈕真的送到規劃器**。第一版探針改的是
`hv_dccpp.platform.PLATFORM_PIONEER` 這個模組屬性，而轉接層在匯入時就把它綁進
`_SPEC_OF`（:500）與 `platform_a` 的預設值（:1653）⇒ 七個不同的 min_fw 跑出
一模一樣的結果。那不是「沒有影響」，那是**量不到東西**。
"""
from __future__ import annotations

import logging as _logging
import math

import pytest

from mission.swarm_coordinator import SwarmCoordinator

pytest.importorskip("hv_dccpp")

from hv_dccpp.integration.aeroplan_adapter import (  # noqa: E402
    AeroPlanAdapterError as _AdapterError)

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
HFOV, VFOV = 60.0, 45.0
_HALF = math.sqrt(2449789.0) / 2.0          # 2.45 km² 的正方形


def _ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


AREA = [_ll(-_HALF, -_HALF), _ll(_HALF, -_HALF), _ll(_HALF, _HALF), _ll(-_HALF, _HALF)]


def _plan(*, minfw_a=None, minfw_b=None, band=None, ceil=None,
          alt=120.0, gsd=0.05, report=None):
    """跑一趟真的規劃。回傳 (上工構型集合, makespan, coordinator)。

    NO_GO 時回傳 (None, 0.0, coordinator)。
    """
    coord = SwarmCoordinator()
    coord.use_harvest = True
    coord.hv_class_of = lambda d: {1: "A", 2: "B"}[int(d["drone_id"])]
    coord.hv_home = HOME
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    if report is not None:
        coord.hv_on_report = report.update
    params: dict = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                    "hv_dem_margin_m": 3000.0, "target_gsd": gsd}
    if ceil is not None:
        params["hv_waiver_max_agl_m"] = ceil
    mf = {}
    if minfw_a is not None:
        mf["A"] = minfw_a
    if minfw_b is not None:
        mf["B"] = minfw_b
    if mf:
        params["hv_min_fw_agl_m"] = mf
    if band is not None:
        params["hv_dh_band_min_m"] = band
    coord.harvest_params = params
    drones = [{"drone_id": 1, "name": "A1", "position": HOME, "speed": 21.5,
               "heading": 0.0, "turn_radius": 200.0, "vehicle_type": "vtol",
               "fov_width": 150.0},
              {"drone_id": 2, "name": "B1", "position": HOME, "speed": 19.0,
               "heading": 0.0, "turn_radius": 120.0, "vehicle_type": "vtol",
               "fov_width": 150.0}]
    try:
        res = coord.plan_coverage_dccpp(
            areas=[{"area_id": 1, "priority": 1.0, "polygon": AREA}],
            drones=drones, coverage_width_m=150.0, overlap_rate=0.30,
            coverage_altitude=alt, mounting_angle_deg=0.0)
    except _AdapterError:
        # 只吞上游的 NO_GO。我們自己的參數守衛丟 ValueError，要讓它冒上去，
        # 不然「拒收不合理的值」那條測試會變成量不到東西的假綠燈。
        return None, 0.0, coord
    plan = coord.hv_coordinator.last_plan
    used = {p.cls for p in plan.plans.values() if p.strips}
    return used, float(res.get("total_makespan") or 0.0), coord


def _assert_fired(coord, *, minfw_a=None, minfw_b=None, band=None):
    """旋鈕真的到得了規劃器嗎？—— 每條測試的前置條件。"""
    hv = coord.hv_coordinator
    assert hv is not None, "沒走到 HV 分支"
    if minfw_a is not None:
        assert hv.platform_a.min_fw_operation_agl_m == pytest.approx(minfw_a), \
            "min_fw(A) 沒送到規劃器 ⇒ 這條測試量不到東西"
    if minfw_b is not None:
        assert hv.platform_b.min_fw_operation_agl_m == pytest.approx(minfw_b), \
            "min_fw(B) 沒送到規劃器"
    if band is not None:
        assert hv.cfg.dh_band_min_m == pytest.approx(band), \
            "dh_band_min_m 沒送到規劃器"


# ── 1. 表 1 不可以被改掉 ──────────────────────────────────────────────
def test_the_authoritative_table_still_says_120_and_100():
    """值層級鎖：這件事的正解是**覆寫**，不是去改權威表。

    有人如果「順手」把表 1 的 120 改成 100，本檔其他測試會全部照樣綠 ——
    因為它們只驗行為。所以這裡直接釘住檔案內容。
    """
    from pathlib import Path
    src = Path(__file__).resolve().parent.parent / "docs" / "hv_dccpp" / "平台參數表.md"
    row = [ln for ln in src.read_text(encoding="utf-8").splitlines()
           if "min_fw_operation_agl_m" in ln]
    assert len(row) == 1, row
    assert "120" in row[0] and "100" in row[0], row[0]
    assert "任務前需依地形、感測器與法規調整" in row[0], \
        "使用方式欄被改掉了 —— 「可依法規調整」正是我們調它的依據"


def test_upstream_only_allows_raising_it():
    """上游的 `platform_overrides` 只准調高，所以我們走 `platform_a=` 那條路。

    這一條記住的是「為什麼不用 platform_overrides」。哪天上游放寬了，
    它會紅，那時候就該回頭簡化。
    """
    import dataclasses

    from hv_dccpp.context import apply_platform_overrides
    from hv_dccpp.mission import MissionInputError
    from hv_dccpp.platform import PLATFORM_PIONEER

    base = {"A": PLATFORM_PIONEER}
    with pytest.raises(MissionInputError, match="只允許調高"):
        apply_platform_overrides(base, {"A": {"min_fw_operation_agl_m": 100.0}})
    out, _w = apply_platform_overrides(base, {"A": {"min_fw_operation_agl_m": 140.0}})
    assert out["A"].min_fw_operation_agl_m == 140.0
    assert dataclasses.is_dataclass(out["A"])


# ── 2. 旋鈕真的送到規劃器 ────────────────────────────────────────────
def test_panel_value_reaches_the_planner():
    """⚠ 這條是所有行為測試的前置條件，見檔頭那段「量不到東西」的教訓。"""
    _used, _ms, coord = _plan(minfw_a=100.0, minfw_b=100.0, band=15.0, ceil=200.0)
    _assert_fired(coord, minfw_a=100.0, minfw_b=100.0, band=15.0)


def test_default_specs_are_the_table_values():
    """不給覆寫時就是表 1 的 120／100 與規格的 20。"""
    _used, _ms, coord = _plan(ceil=200.0)
    _assert_fired(coord, minfw_a=120.0, minfw_b=100.0, band=20.0)


# ── 3. 法規上限下的實際結果 ──────────────────────────────────────────
def test_table_values_are_unflyable_at_the_legal_ceiling():
    """反向臂：120／100／20 在 120 m 上限下整趟 NO_GO。"""
    used, _ms, coord = _plan(ceil=120.0)
    _assert_fired(coord, minfw_a=120.0, minfw_b=100.0, band=20.0)
    assert used is None, f"本來預期 NO_GO，卻規劃出 {used}"


def test_lowering_to_100_alone_is_still_not_enough():
    """⚠ 只把 Pioneer 調到 100 **還是不夠** —— 窗恰好 20 m，起伏一減就破。

    這一條擋掉「調成 100 就好了」這個直覺。實測差的是不到 1 m 的起伏。
    """
    used, _ms, coord = _plan(minfw_a=100.0, minfw_b=100.0, band=20.0, ceil=120.0)
    _assert_fired(coord, minfw_a=100.0, minfw_b=100.0, band=20.0)
    assert used is None, f"本來預期仍 NO_GO，卻規劃出 {used}"


def test_100_plus_relaxed_band_flies_both_types():
    """面板預設組合（100／100／15）在法規上限下兩型都有工作。"""
    used, ms, coord = _plan(minfw_a=100.0, minfw_b=100.0, band=15.0, ceil=120.0)
    _assert_fired(coord, minfw_a=100.0, minfw_b=100.0, band=15.0)
    assert used == {"A", "B"}, used
    assert 1000.0 < ms < 1800.0, ms          # 實測 1423.8 s（表 1 值是 2085 s）


def test_no_legal_min_fw_fixes_it_without_relaxing_the_band():
    """⚠⚠ 在 120 m 上限下，**光調最低作業高度是解不掉的**。

    Pioneer 的合法下界是它的轉換完成高度上緣 100 m，而 120 − 100 = 20 恰好
    等於規格的高度帶下限 ⇒ 容許的地形起伏是 0。所以合法範圍內的每一個值都
    NO_GO，唯一的槓桿是高度帶下限。

    （實測 99/99/20 確實會過，但 99 低於轉換完成高度上緣，已被上面那條擋掉，
    不是可以用的組態。）
    """
    for mf_a in (100.0, 105.0, 110.0, 120.0):
        used, _ms, coord = _plan(minfw_a=mf_a, minfw_b=100.0, band=20.0, ceil=120.0)
        _assert_fired(coord, minfw_a=mf_a, minfw_b=100.0, band=20.0)
        assert used is None, f"min_fw(A)={mf_a} 竟然規劃得出 {used}"


# ── 4. 下界：不可低於轉換完成高度上緣 ────────────────────────────────
def test_below_transition_complete_is_refused():
    """低於轉換完成高度上緣 ⇒ 等於要求轉換沒完成就固定翼巡航，直接拒收。

    表 1：A 的 transition_complete_agl_m 是 80–100，B 是 70–90。
    """
    with pytest.raises(ValueError, match="轉換完成高度上緣"):
        _plan(minfw_a=95.0, ceil=200.0)
    with pytest.raises(ValueError, match="轉換完成高度上緣"):
        _plan(minfw_b=85.0, ceil=200.0)
    # 邊界上剛好可以
    _used, _ms, coord = _plan(minfw_a=100.0, minfw_b=90.0, band=15.0, ceil=200.0)
    _assert_fired(coord, minfw_a=100.0, minfw_b=90.0)


# ── 5. 調低要說出來 ──────────────────────────────────────────────────
def test_lowering_is_announced(caplog):
    report: dict = {}
    with caplog.at_level(_logging.WARNING, logger="mission.swarm_coordinator"):
        _plan(minfw_a=100.0, minfw_b=100.0, band=15.0, ceil=200.0, report=report)
    msg = [r.getMessage() for r in caplog.records
           if "min_fw_lowered" in r.getMessage()]
    assert msg, "調低了卻沒說"
    # ⚠ 兩個下游的**生效條件不同**，訊息必須分開講：
    #   Q_RTL_ALT 進匯出的 .param，每條路徑都生效（HV4SITL8 讀回 100 已確認）；
    #   FENCE_ALT_MIN 只在 mavlink_link.upload_plan 寫參數那條路才送出，
    #   而 AeroPlan 的 SITL／匯出流程都不走那條路。
    assert "Q_RTL_ALT" in msg[0], f"沒講出真正會生效的下游：{msg[0]}"
    assert "FENCE_ALT_MIN" in msg[0] and "mavlink_link" in msg[0], \
        f"把 FENCE_ALT_MIN 講成無條件生效了：{msg[0]}"
    assert "min_fw_lowered" in (report.get("validation_note") or "")
    band = [r.getMessage() for r in caplog.records
            if "dh_band_min_relaxed" in r.getMessage()]
    assert band, "高度帶下限調小了卻沒說"


def test_table_values_stay_quiet(caplog):
    """反向臂：用表 1 值時這兩則警告都不可以出現。"""
    with caplog.at_level(_logging.WARNING, logger="mission.swarm_coordinator"):
        _plan(minfw_a=120.0, minfw_b=100.0, band=20.0, ceil=200.0)
    assert not [r for r in caplog.records
                if "min_fw_lowered" in r.getMessage()
                or "dh_band_min_relaxed" in r.getMessage()]


def test_gui_surfaces_both_notes():
    import ui.main_window as mw
    assert "min_fw_lowered" in mw.MainWindow._HV_LOUD_NOTES
    assert "dh_band_min_relaxed" in mw.MainWindow._HV_LOUD_NOTES


# ── 6. 面板 ──────────────────────────────────────────────────────────
@pytest.fixture(scope="session")
def qapp():
    """⚠ 一定要留住這個參考，否則收尾時 process 會 exit 127 無聲死掉。"""
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_panel_defaults_fly_at_the_legal_ceiling(qapp):
    """面板出廠值必須是「不勾豁免也飛得起來」的那一組。"""
    import ui.widgets.vtol_recon_panel as vrp
    p = vrp.VTOLReconPanel()
    d = p.get_params()
    assert d["hv_waiver_max_agl_m"] is None
    assert d["hv_min_fw_agl_m"] == {"A": pytest.approx(100.0), "B": pytest.approx(100.0)}
    assert d["hv_dh_band_min_m"] == pytest.approx(15.0)
    p.deleteLater()


def test_panel_cannot_go_below_transition_complete(qapp):
    """面板下界就擋住，不要等到規劃才丟例外。"""
    import ui.widgets.vtol_recon_panel as vrp
    p = vrp.VTOLReconPanel()
    p.hv_minfw_a_spin.setValue(80.0)
    assert p.hv_minfw_a_spin.value() == pytest.approx(100.0), "A 的下界不是 100"
    p.hv_minfw_b_spin.setValue(50.0)
    assert p.hv_minfw_b_spin.value() == pytest.approx(90.0), "B 的下界不是 90"
    p.deleteLater()
