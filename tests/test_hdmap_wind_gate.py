"""建圖作業風速上限（2026-10-15 使用者定調：首次建圖只在風速 ≤ 3 m/s 飛）的值層級守衛。

鏈：面板「建圖作業風速上限」（hv_map_max_wind_mps，預設 3.0）→ build_mapping_setup（有效值、警告）→
SwarmCoordinator 把它放進 HvDccppCoordinator(mapping_max_wind_mps=…) → 轉接層 mapping_gate()（MainWindow 的
hv_upload_gate 呼叫它時**不帶參數**，所以上限必須放在建構參數）比較「規劃實際用的風速」（last_ctx.wind 各層最大值）。
3.0 m/s 過、3.1 m/s 擋；上限指現場實際風速，起飛前要實測（每次都提醒）。

真規劃情境同 test_hdmap_gui_coordinator.py：1.2 × 1.0 km 平地、1 Pioneer + 2 HERO、HQ6、120 m、無豁免。
"""
from __future__ import annotations

import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from mission.swarm_coordinator import (  # noqa: E402
    MAPPING_MAX_WIND_DEFAULT_MPS, MAPPING_WIND_SITE_NOTE, build_mapping_setup, mapping_setup_note,
)

pytest.importorskip("hv_dccpp")

from hv_dccpp.integration.aeroplan_adapter import AeroPlanAdapterError, HvDccppCoordinator  # noqa: E402
from hv_dccpp.mission import MappingModeConfig  # noqa: E402

from tests.test_hdmap_cross_strips import params, plan  # noqa: E402


@pytest.fixture(scope="module")
def at_limit():
    return plan(params(hv_wind=(3.0, 270.0)))


@pytest.fixture(scope="module")
def over_limit():
    return plan(params(hv_wind=(3.1, 270.0)))


# ═══════════════════════ 1. build_mapping_setup ═══════════════════════
def test_default_limit_is_three_and_recorded():
    assert MAPPING_MAX_WIND_DEFAULT_MPS == 3.0
    p = params()
    p.pop("hv_map_max_wind_mps")
    s = build_mapping_setup(p)                                               # 缺鍵 ⇒ 預設 3.0（寫進有效值）
    assert s["max_wind_mps"] == 3.0 and s["effective"]["max_wind_mps"] == 3.0
    assert "建圖作業風速上限 3 m/s（指現場實際風速，起飛前實測）" in mapping_setup_note(s)
    assert build_mapping_setup(params(hv_map_max_wind_mps=4.5))["max_wind_mps"] == 4.5


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf"), "3", True])
def test_bad_limit_is_refused(bad):
    with pytest.raises(ValueError, match="建圖作業風速上限"):
        build_mapping_setup(params(hv_map_max_wind_mps=bad))


def test_setup_always_reminds_site_wind_and_flags_planning_wind_over_limit():
    ok = build_mapping_setup(params(hv_wind=(3.0, 270.0)))
    assert any(w.startswith(MAPPING_WIND_SITE_NOTE) and "上限 3 m/s" in w for w in ok["warnings"])
    assert not any("超過建圖作業風速上限" in w for w in ok["warnings"])
    over = build_mapping_setup(params(hv_wind=(3.1, 270.0)))
    assert any("規劃風速 3.1 m/s 超過建圖作業風速上限 3 m/s" in w for w in over["warnings"])


# ═══════════════════════ 2. 轉接層 ═══════════════════════
def test_adapter_refuses_limit_without_mapping_and_bad_values():
    with pytest.raises(AeroPlanAdapterError, match="只在建圖模式"):
        HvDccppCoordinator(mapping_max_wind_mps=3.0)
    for bad in (0.0, -2.0, float("nan")):
        with pytest.raises(AeroPlanAdapterError, match="建圖作業風速上限"):
            HvDccppCoordinator(mapping_mode=MappingModeConfig(), mapping_max_wind_mps=bad)
    hv = HvDccppCoordinator(mapping_mode=MappingModeConfig(), mapping_max_wind_mps=3)
    assert hv.mapping_max_wind_mps == 3.0
    assert hv.mapping_gate() == {"uploadable": False, "reasons": ["尚未規劃"]}       # 形狀不變


def test_gate_passes_at_limit(at_limit):
    hv = at_limit.hv_coordinator
    assert hv.mapping_max_wind_mps == 3.0
    assert hv.last_ctx.wind.max_speed() == pytest.approx(3.0, abs=1e-12)
    g = hv.mapping_gate()
    assert not any("建圖作業風速上限" in r for r in g["reasons"]), g
    assert hv.mapping_effective["wind"] == {"limit_mps": 3.0, "planning_wind_mps": pytest.approx(3.0)}
    assert any(n.startswith("mapping_wind(規劃風速 3 m/s ≤ 建圖作業風速上限 3 m/s") and "現場實際風速" in n
               for n in hv.notes)
    assert at_limit.hv_mapping_gate["uploadable"] == g["uploadable"]


def test_gate_blocks_over_limit_with_reason(over_limit):
    hv = over_limit.hv_coordinator
    g = hv.mapping_gate()
    assert g["uploadable"] is False
    r = [x for x in g["reasons"] if "建圖作業風速上限" in x]
    assert len(r) == 1 and "3 m/s" in r[0] and "3.1 m/s" in r[0] and "現場實際風速" in r[0]
    assert over_limit.hv_mapping_gate["uploadable"] is False
    assert any("建圖作業風速上限" in x for x in over_limit.hv_mapping_gate["reasons"])
    assert any(n.startswith("mapping_wind(規劃風速 3.1 m/s ＞") and "不可上傳" in n for n in hv.notes)
    # 呼叫時給的上限優先（例如之後放寬到 5 m/s 再比）
    assert not any("建圖作業風速上限" in x for x in hv.mapping_gate(max_wind_mps=5.0)["reasons"])


def test_mainwindow_upload_gate_sees_the_wind_reason(over_limit, at_limit):
    """MainWindow 的上傳拒絕路徑（hv_upload_gate）不帶參數呼叫 mapping_gate()：上限在建構參數裡才擋得到。"""
    from ui.main_window import hv_upload_gate
    g = hv_upload_gate(over_limit.hv_coordinator, over_limit)
    assert g["uploadable"] is False and any("建圖作業風速上限" in x for x in g["reasons"])
    g2 = hv_upload_gate(at_limit.hv_coordinator, at_limit)
    assert not any("建圖作業風速上限" in x for x in g2["reasons"])


def test_unmapped_plan_has_no_wind_gate():
    c = plan(params(hv_map_enabled=False, hv_wind=(3.1, 270.0)))
    hv = c.hv_coordinator
    assert hv.mapping_mode is None and hv.mapping_max_wind_mps is None
    assert not any("建圖作業風速上限" in r for r in hv.mapping_gate()["reasons"])
    assert not any(str(n).startswith("mapping_wind(") for n in hv.notes)


# ═══════════════════════ 3. 面板 ═══════════════════════
try:
    from ui.main_window import MainWindow          # noqa: F401
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


def test_panel_wind_limit_field_and_warning(qt_app):
    pnl = VTOLReconPanel()
    pnl.algo_combo.setCurrentIndex(pnl.algo_combo.findData("hv_dccpp"))
    assert pnl.hv_map_max_wind_spin.value() == 3.0 and pnl.get_params()["hv_map_max_wind_mps"] == 3.0
    assert not pnl.hv_map_max_wind_spin.isEnabled()
    pnl.hv_map_check.setChecked(True)
    pnl.hv_map_backend_combo.setCurrentIndex(pnl.hv_map_backend_combo.findData("servo"))
    assert pnl.hv_map_max_wind_spin.isEnabled()
    pnl.hv_wind_speed_spin.setValue(3.0)
    txt = pnl.hv_map_warn_label.text()
    assert "現場實際風速" in txt and "超過建圖作業風速上限" not in txt
    pnl.hv_wind_speed_spin.setValue(3.5)
    assert "規劃風速 3.5 m/s 超過建圖作業風速上限 3 m/s" in pnl.hv_map_warn_label.text()
    pnl.hv_map_max_wind_spin.setValue(4.0)
    assert "超過建圖作業風速上限" not in pnl.hv_map_warn_label.text()
    assert pnl.get_params()["hv_map_max_wind_mps"] == 4.0
    assert math.isclose(build_mapping_setup(pnl.get_params())["max_wind_mps"], 4.0)
