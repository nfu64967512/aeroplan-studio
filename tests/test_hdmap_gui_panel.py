"""航測建圖：VTOL 偵察面板「航測建圖」群組、相機配置對話框、main.py 的 MAVLINK20。

* 預設關閉 ⇒ get_params 的 hv_map_enabled=False（不建圖的使用者完全不變）；
* 每個欄位都有明確的鍵、逐值送出；用不到的欄位送 None（不送一個會被默默忽略的值）；
* 相機預設只填規格值，觸發率是佔位 ⇒ 顯示「未經台架驗證」直到勾選；
* 建圖時感測器足跡的 HFOV／VFOV／影像寬度／掛載角是這台相機的唯讀鏡子，關掉就還原；
* 驗證與規劃器同一支 build_mapping_setup（relay 沒填韌體 ⇒ 擋規劃並原樣顯示訊息）；
* 相機配置對話框不再自稱「已應用」，而是交出值給 MainWindow 填欄位。
"""
from __future__ import annotations

import ast
import math
import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine
    from ui.widgets.vtol_recon_panel import (
        MAPPING_CAMERA_PRESETS, MAPPING_TRIGGER_HZ_PLACEHOLDER, VTOLReconPanel,
    )
    from PyQt6.QtWidgets import QApplication
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

ROOT = Path(__file__).resolve().parent.parent
MAP_KEYS = {
    "hv_map_enabled", "hv_map_camera_preset", "hv_map_focal_mm", "hv_map_pixel_um", "hv_map_width_px",
    "hv_map_height_px", "hv_map_trigger_hz", "hv_map_trigger_verified", "hv_map_readout_ms",
    "hv_map_forward_overlap", "hv_map_side_overlap", "hv_map_backend", "hv_map_firmware", "hv_map_relay_pin",
    "hv_map_relay_aux_servo", "hv_map_feedback_pin", "hv_map_feedback_servo", "hv_map_max_roll_deg",
    "hv_map_lead_in_m", "hv_map_terrain_retrigger_tol", "hv_map_tlog",
    # 2026-10-15：交叉航帶（沒勾 ⇒ 比例／高度差送 None）＋建圖作業風速上限（預設 3.0 m/s）
    "hv_map_cross_enabled", "hv_map_cross_fraction", "hv_map_cross_alt_offset_m", "hv_map_max_wind_mps",
}


@pytest.fixture(scope="module")
def qt_app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture()
def panel(qt_app):
    return VTOLReconPanel()


def _map_params(p):
    return {k: v for k, v in p.get_params().items() if k.startswith("hv_map_")}


# ═══════════════════════════ 預設 ═══════════════════════════
def test_mapping_is_off_by_default_and_every_key_is_explicit(panel):
    p = _map_params(panel)
    assert set(p) == MAP_KEYS
    assert p["hv_map_enabled"] is False
    assert p == {
        "hv_map_enabled": False, "hv_map_camera_preset": "rpi_hq_6mm",
        "hv_map_focal_mm": 6.0, "hv_map_pixel_um": 1.55, "hv_map_width_px": 4056, "hv_map_height_px": 3040,
        "hv_map_trigger_hz": MAPPING_TRIGGER_HZ_PLACEHOLDER, "hv_map_trigger_verified": False,
        "hv_map_readout_ms": None, "hv_map_forward_overlap": 0.80, "hv_map_side_overlap": 0.20,
        "hv_map_backend": "relay", "hv_map_firmware": None, "hv_map_relay_pin": None,
        "hv_map_relay_aux_servo": None, "hv_map_feedback_pin": -1, "hv_map_feedback_servo": None,
        "hv_map_max_roll_deg": 15.0, "hv_map_lead_in_m": 50.0, "hv_map_terrain_retrigger_tol": 0.10,
        "hv_map_tlog": False,
        "hv_map_cross_enabled": False, "hv_map_cross_fraction": None, "hv_map_cross_alt_offset_m": None,
        "hv_map_max_wind_mps": 3.0,
    }


def test_mapping_off_leaves_the_sensor_fields_alone(panel):
    before = (panel.hfov_spin.value(), panel.vfov_spin.value(), panel.img_width_spin.value(),
              panel.mount_spin.value(), panel.get_params()["fov_width_m"])
    assert before[:4] == (60.0, 45.0, 4000, 0.0)
    assert panel.hfov_spin.isEnabled() and panel.mount_spin.isEnabled()
    assert not panel.hv_map_focal_spin.isEnabled()                   # 建圖欄位鎖著
    assert panel.hv_map_warn_label.text() == ""


def test_mapping_needs_the_harvest_planner(panel):
    panel.hv_map_check.setChecked(True)
    assert panel.get_params()["hv_map_enabled"] is True
    panel.algo_combo.setCurrentIndex(panel.algo_combo.findData("dccpp"))
    assert panel.hv_map_check.isChecked() is False                   # 切到 DCCPP ⇒ 關掉並還原
    assert panel.get_params()["hv_map_enabled"] is False
    assert panel.hfov_spin.value() == 60.0 and panel.hfov_spin.isEnabled()
    # 防呆第二層：就算勾選框被程式勾上，DCCPP 規劃也不會收到 hv_map_enabled=True
    panel.hv_map_check.setChecked(True)
    assert panel.get_params()["planner"] == "dccpp"
    assert panel.get_params()["hv_map_enabled"] is False and panel.mapping_enabled() is False


# ═══════════════════════════ 鏡子：感測器足跡 = 這台相機 ═══════════════════════════
def test_mapping_mirrors_the_camera_into_the_footprint_and_restores(panel):
    panel.mount_spin.setValue(10.0)
    panel.hv_map_check.setChecked(True)
    w_mm, h_mm = 4056 * 1.55e-3, 3040 * 1.55e-3
    assert panel.hfov_spin.value() == pytest.approx(2 * math.degrees(math.atan(w_mm / 12.0)), abs=0.006)
    assert panel.vfov_spin.value() == pytest.approx(2 * math.degrees(math.atan(h_mm / 12.0)), abs=0.006)
    assert panel.img_width_spin.value() == 4056 and panel.mount_spin.value() == 0.0
    assert not panel.hfov_spin.isEnabled() and not panel.mount_spin.isEnabled()
    # 掃描寬度 = h·w_mm/f（120 m ⇒ 125.7 m）
    assert panel.get_params()["raw_swath_width_m"] == pytest.approx(120.0 * w_mm / 6.0, rel=2e-4)
    panel.hv_map_preset_combo.setCurrentIndex(panel.hv_map_preset_combo.findData("rpi_gs_8mm"))
    assert panel.img_width_spin.value() == 1456
    panel.hv_map_check.setChecked(False)
    assert (panel.hfov_spin.value(), panel.vfov_spin.value(), panel.img_width_spin.value(),
            panel.mount_spin.value()) == (60.0, 45.0, 4000, 10.0)
    assert panel.hfov_spin.isEnabled()


# ═══════════════════════════ 預設相機／自訂／台架驗證 ═══════════════════════════
@pytest.mark.parametrize("key", list(MAPPING_CAMERA_PRESETS))
def test_presets_fill_the_spec_values(panel, key):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_preset_combo.setCurrentIndex(panel.hv_map_preset_combo.findData(key))
    pre = MAPPING_CAMERA_PRESETS[key]
    p = _map_params(panel)
    assert (p["hv_map_focal_mm"], p["hv_map_pixel_um"], p["hv_map_width_px"], p["hv_map_height_px"]) == \
        pytest.approx((pre["focal_mm"], pre["pixel_um"], pre["width_px"], pre["height_px"]))
    assert p["hv_map_camera_preset"] == key


def test_preset_numbers_reproduce_the_feasibility_footprints():
    """評估報告 §5 的足跡（120 m）：HQ6 125.7 m、CM3 163 m、GS8 75 m。"""
    def swath(k):
        pre = MAPPING_CAMERA_PRESETS[k]
        return 120.0 * pre["width_px"] * pre["pixel_um"] * 1e-3 / pre["focal_mm"]
    assert swath("rpi_hq_6mm") == pytest.approx(125.7, abs=0.1)
    assert swath("rpi_cm3") == pytest.approx(163.3, abs=0.1)
    assert swath("rpi_gs_8mm") == pytest.approx(75.3, abs=0.1)


def test_editing_a_camera_value_switches_to_custom(panel):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_focal_spin.setValue(6.3)
    assert panel.get_params()["hv_map_camera_preset"] == "custom"
    assert panel.get_params()["hv_map_focal_mm"] == pytest.approx(6.3)


def test_trigger_rate_is_unverified_until_ticked_and_edits_reset_it(panel):
    panel.hv_map_check.setChecked(True)
    assert panel.hv_map_rate_status_label.text() == "未經台架驗證"
    panel.hv_map_rate_verified_check.setChecked(True)
    assert panel.hv_map_rate_status_label.text() == "已台架驗證"
    assert panel.get_params()["hv_map_trigger_verified"] is True
    panel.hv_map_rate_spin.setValue(1.8)                             # 改了數字 ⇒ 舊的驗證不算
    assert panel.get_params()["hv_map_trigger_verified"] is False
    assert panel.hv_map_rate_status_label.text() == "未經台架驗證"
    panel.hv_map_rate_verified_check.setChecked(True)
    panel.hv_map_preset_combo.setCurrentIndex(panel.hv_map_preset_combo.findData("rpi_cm3"))
    assert panel.get_params()["hv_map_trigger_verified"] is False     # 換相機 ⇒ 也不算
    assert panel.get_params()["hv_map_trigger_hz"] == MAPPING_TRIGGER_HZ_PLACEHOLDER


# ═══════════════════════════ 逐值送出 ═══════════════════════════
def test_values_reach_get_params(panel):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_rate_spin.setValue(1.7)
    panel.hv_map_rate_verified_check.setChecked(True)
    panel.hv_map_readout_spin.setValue(84.0)
    panel.hv_map_fwd_spin.setValue(0.85)
    panel.overlap_spin.setValue(0.70)
    panel.hv_map_roll_spin.setValue(10.0)
    panel.hv_map_lead_spin.setValue(40.0)
    panel.hv_map_retrig_spin.setValue(0.0)                           # 0 ⇒ 關閉 ⇒ None
    panel.hv_map_tlog_check.setChecked(True)
    panel.hv_map_firmware_edit.setText("  4.5.7 ")
    p = _map_params(panel)
    assert p["hv_map_trigger_hz"] == pytest.approx(1.7) and p["hv_map_trigger_verified"] is True
    assert p["hv_map_readout_ms"] == pytest.approx(84.0)
    assert p["hv_map_forward_overlap"] == pytest.approx(0.85)
    assert p["hv_map_side_overlap"] == pytest.approx(0.70)          # 側向重疊就是「重疊率」
    assert p["hv_map_max_roll_deg"] == 10.0 and p["hv_map_lead_in_m"] == 40.0
    assert p["hv_map_terrain_retrigger_tol"] is None and p["hv_map_tlog"] is True
    assert p["hv_map_firmware"] == "4.5.7"


def test_relay_fields_only_count_for_the_relay_backend(panel):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_backend_combo.setCurrentIndex(panel.hv_map_backend_combo.findData("servo"))
    panel.hv_map_relay_pin_spin.setValue(50)
    panel.hv_map_relay_aux_spin.setValue(9)
    assert not panel.hv_map_relay_pin_spin.isEnabled()               # servo：relay 欄位鎖著
    p = _map_params(panel)
    assert p["hv_map_relay_pin"] is None and p["hv_map_relay_aux_servo"] is None
    panel.hv_map_backend_combo.setCurrentIndex(panel.hv_map_backend_combo.findData("relay"))
    assert panel.hv_map_relay_pin_spin.isEnabled()
    p = _map_params(panel)
    assert (p["hv_map_backend"], p["hv_map_relay_pin"], p["hv_map_relay_aux_servo"]) == ("relay", 50, 9)


def test_feedback_channel_needs_a_feedback_pin(panel):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_fb_servo_spin.setValue(11)
    assert _map_params(panel)["hv_map_feedback_servo"] is None
    assert not panel.hv_map_fb_servo_spin.isEnabled()
    panel.hv_map_fb_pin_spin.setValue(51)
    assert panel.hv_map_fb_servo_spin.isEnabled()
    p = _map_params(panel)
    assert (p["hv_map_feedback_pin"], p["hv_map_feedback_servo"]) == (51, 11)


# ═══════════════════════════ 驗證（同一支 build_mapping_setup） ═══════════════════════════
def test_relay_without_firmware_blocks_the_plan_with_the_config_message(panel, qt_app):
    panel.set_corner_count(4)
    panel.hv_map_check.setChecked(True)
    panel.hv_map_backend_combo.setCurrentIndex(panel.hv_map_backend_combo.findData("relay"))
    panel.hv_map_relay_pin_spin.setValue(50)
    sent = []
    panel.plan_requested.connect(sent.append)
    panel._emit_plan()
    assert sent == []
    assert "firmware" in panel.plan_status.text() and "航測建圖設定有誤" in panel.plan_status.text()
    assert "無法規劃" in panel.hv_map_warn_label.text()
    panel.hv_map_firmware_edit.setText("4.5.7")
    panel._emit_plan()
    assert len(sent) == 1 and sent[0]["hv_map_firmware"] == "4.5.7"


def test_mavlink_v2_without_firmware_is_refused(panel):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_backend_combo.setCurrentIndex(panel.hv_map_backend_combo.findData("mavlink_v2"))
    errs, _w = panel.mapping_problems()
    assert errs and "firmware" in errs[0]


def test_panel_params_feed_the_planner_setup_value_for_value(panel):
    """面板 get_params → build_mapping_setup（規劃器用的那支）：鍵名對得上、值逐一相同。"""
    from mission.swarm_coordinator import build_mapping_setup
    panel.hv_map_check.setChecked(True)
    panel.hv_map_preset_combo.setCurrentIndex(panel.hv_map_preset_combo.findData("rpi_cm3"))
    panel.hv_map_rate_spin.setValue(1.9)
    panel.hv_map_fwd_spin.setValue(0.85)
    panel.overlap_spin.setValue(0.65)
    panel.hv_map_lead_spin.setValue(60.0)
    panel.hv_map_roll_spin.setValue(12.0)
    panel.hv_map_backend_combo.setCurrentIndex(panel.hv_map_backend_combo.findData("relay"))
    panel.hv_map_firmware_edit.setText("4.5.7")
    panel.hv_map_relay_pin_spin.setValue(50)
    panel.hv_map_relay_aux_spin.setValue(9)
    s = build_mapping_setup(panel.get_params())
    cam, cc, mm = s["camera"], s["camera_cfg"], s["mapping_mode"]
    assert (cam.focal_length_mm, cam.pixel_pitch_um) == pytest.approx((4.74, 1.40))
    assert cam.sensor_width_mm == pytest.approx(4608 * 1.40e-3) and cam.sensor_height_mm == pytest.approx(2592 * 1.40e-3)
    assert cc.interval_min_s == pytest.approx(0.54) and cam.max_shutter_rate_hz == pytest.approx(1 / 0.54)
    assert s["forward_overlap"] == pytest.approx(0.85) and s["side_overlap"] == pytest.approx(0.65)
    assert mm.lead_in_m == cc.camera_lead_in_m == 60.0
    assert (cc.backend, cc.firmware, cc.relay_pin, cc.aux_servo_channel_for_relay, cc.max_roll_deg) == \
        ("relay", "4.5.7", 50, 9, 12.0)
    assert cc.terrain_retrigger_tol == pytest.approx(0.10)


def test_side_overlap_below_060_and_unverified_rate_warn(panel):
    panel.hv_map_check.setChecked(True)
    panel.hv_map_firmware_edit.setText("4.5.7")                      # 預設 relay：要韌體版本與腳位才規劃得了
    panel.hv_map_relay_pin_spin.setValue(50)
    panel.overlap_spin.setValue(0.30)
    errs, warns = panel.mapping_problems()
    assert errs == []
    assert any("0.30" in w and "0.60" in w for w in warns)
    assert any("未經台架驗證" in w for w in warns)
    assert "0.60" in panel.hv_map_warn_label.text()
    panel.overlap_spin.setValue(0.70)
    _e, warns2 = panel.mapping_problems()
    assert not any("側向重疊" in w for w in warns2)


def test_mapping_group_keeps_the_page_rules(panel):
    """沒有裝飾符號（⚠ 在 0x2600–0x27BF）、群組編號規則不被新群組打亂、表單仍是全寬。"""
    from PyQt6.QtWidgets import QFormLayout, QGroupBox, QLabel
    panel.hv_map_check.setChecked(True)
    panel.overlap_spin.setValue(0.3)
    texts = [panel.hv_map_warn_label.text(), panel.hv_map_rate_status_label.text()]
    texts += [w.text() for w in panel.findChildren(QLabel)]
    assert not [t for t in texts for ch in t if 0x2600 <= ord(ch) <= 0x27BF]
    titles = [g.title() for g in panel.findChildren(QGroupBox)]
    assert "航測建圖" in titles
    for f in panel.findChildren(QFormLayout):
        assert f.fieldGrowthPolicy() == QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow


def test_live_and_plan_status_setters(panel):
    panel.set_mapping_live_status("UAV-1：已拍 3 / 預估 120")
    assert panel.hv_map_live_label.text() == "UAV-1：已拍 3 / 預估 120"
    panel.set_mapping_plan_status("建圖閘門：不可上傳", ok=False)
    assert "不可上傳" in panel.hv_map_status_label.text()


# ═══════════════════════════ 相機配置 → 航測建圖欄位 ═══════════════════════════
def test_apply_camera_values_fills_fields_and_turns_mapping_on(panel):
    panel.hv_map_rate_verified_check.setChecked(True)
    panel.algo_combo.setCurrentIndex(panel.algo_combo.findData("dccpp"))
    eff = panel.apply_camera_values({"focal_mm": 24.0, "pixel_um": 17.3 / 5280 * 1000.0, "width_px": 5280,
                                     "height_px": 3956, "front_overlap": 0.80, "side_overlap": 0.60},
                                    enable_mapping=True)
    p = panel.get_params()
    assert p["planner"] == "hv_dccpp" and p["hv_map_enabled"] is True
    assert p["hv_map_camera_preset"] == "custom"
    assert p["hv_map_focal_mm"] == 24.0 and p["hv_map_width_px"] == 5280 and p["hv_map_height_px"] == 3956
    assert p["hv_map_pixel_um"] == pytest.approx(3.2765, abs=1e-4)
    assert p["hv_map_forward_overlap"] == 0.80 and p["hv_map_side_overlap"] == 0.60
    assert eff["建圖模式"] is True and "規劃演算法" in eff
    assert eff["像素 µm"] == pytest.approx(p["hv_map_pixel_um"])
    assert p["hv_map_trigger_verified"] is False                     # 換相機 ⇒ 驗證不算數
    assert "未經台架驗證" in eff["實測最大觸發率"]


def test_apply_camera_values_without_enabling(panel):
    eff = panel.apply_camera_values({"focal_mm": 8.8, "pixel_um": 2.41, "width_px": 5472, "height_px": 3648,
                                     "front_overlap": 0.75, "side_overlap": 0.65}, enable_mapping=False)
    assert panel.hv_map_check.isChecked() is False and eff["建圖模式"] is False
    assert panel.get_params()["hv_map_focal_mm"] == pytest.approx(8.8)


def test_apply_camera_values_refuses_out_of_range(panel):
    with pytest.raises(ValueError, match="側向重疊"):
        panel.apply_camera_values({"focal_mm": 24.0, "pixel_um": 3.3, "width_px": 5280, "height_px": 3956,
                                   "front_overlap": 0.80, "side_overlap": 0.95}, enable_mapping=True)
    assert panel.hv_map_check.isChecked() is False                  # 什麼都沒改
    assert panel.get_params()["hv_map_focal_mm"] == 6.0


def test_camera_config_dialog_hands_over_values_instead_of_claiming(qt_app, monkeypatch):
    from PyQt6 import QtWidgets as _QtW
    from ui.dialogs.camera_config import CameraConfigDialog
    from sensors import CameraDatabase
    calls = []
    monkeypatch.setattr(_QtW.QMessageBox, "information", staticmethod(lambda *a, **k: calls.append(a)))
    dlg = CameraConfigDialog()
    v = dlg.applied_values()
    cam = CameraDatabase.get_camera(v["camera_name"])
    assert v["focal_mm"] == pytest.approx(cam.focal_length)
    assert v["pixel_um"] == pytest.approx(cam.sensor_width / cam.image_width * 1000.0)
    assert (v["width_px"], v["height_px"]) == (cam.image_width, cam.image_height)
    assert (v["front_overlap"], v["side_overlap"]) == (0.80, 0.60)
    assert set(v["not_applied"]) == {"飛行高度", "飛行速度"}
    dlg.apply_settings()
    assert dlg.result() == _QtW.QDialog.DialogCode.Accepted
    assert calls == []                                               # 不再跳「相機設定已應用」


def test_camera_config_calculation_does_not_mutate_the_database(qt_app, monkeypatch):
    from ui.dialogs.camera_config import CameraConfigDialog
    from sensors import CameraDatabase
    dlg = CameraConfigDialog()
    name = dlg.current_camera.name
    f0 = CameraDatabase.get_camera(name).focal_length
    dlg.focal_length_spin.setValue(f0 + 5.0)
    dlg.calculate_parameters()
    assert CameraDatabase.get_camera(name).focal_length == f0


def test_camera_config_apply_no_longer_claims_success():
    import inspect
    from ui.dialogs.camera_config import CameraConfigDialog
    src = inspect.getsource(CameraConfigDialog.apply_settings)
    assert "相機設定已應用" not in src and "TODO" not in src and "information" not in src


# ═══════════════════════════ main.py：MAVLINK20 在 pymavlink 之前 ═══════════════════════════
_STDLIB = {"os", "sys", "argparse", "pathlib"}


def test_main_sets_mavlink20_before_any_project_or_pymavlink_import():
    tree = ast.parse((ROOT / "main.py").read_text(encoding="utf-8"))
    body = tree.body
    idx_set = next(i for i, n in enumerate(body)
                   if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call)
                   and ast.unparse(n.value) == "os.environ.setdefault('MAVLINK20', '1')")
    for i, n in enumerate(body[:idx_set]):
        if isinstance(n, ast.Import):
            assert all(a.name.split(".")[0] in _STDLIB for a in n.names), ast.unparse(n)
        if isinstance(n, ast.ImportFrom):
            assert (n.module or "").split(".")[0] in _STDLIB, ast.unparse(n)
    first_project = next(i for i, n in enumerate(body)
                         if isinstance(n, (ast.Import, ast.ImportFrom))
                         and not ((isinstance(n, ast.Import) and all(a.name in _STDLIB for a in n.names))
                                  or (isinstance(n, ast.ImportFrom) and (n.module or "") in _STDLIB)))
    assert idx_set < first_project
    assert "pymavlink" not in ast.unparse(ast.Module(body=body[:idx_set], type_ignores=[]))


def test_main_entry_import_chain_has_no_pymavlink_before_the_setdefault():
    """setdefault 之前只有標準庫；之後第一批專案模組（config、utils）本身也不 import pymavlink（靜態檢查）。"""
    for rel in ("config/__init__.py", "config/settings.py", "utils/__init__.py", "utils/math_utils.py",
                "utils/file_io.py", "utils/logger.py"):
        src = (ROOT / rel).read_text(encoding="utf-8")
        assert "pymavlink" not in src, rel
