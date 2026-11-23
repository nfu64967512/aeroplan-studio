"""航測後處理 GUI 的「既有 .pos」模式（2026-10-21 Emlid Studio 評估 §4 M4）。

* .pos 模式也啟用並要求基站座標（流水線會核對 .pos 表頭的 % ref pos），說明文字跟著模式換；
  基站觀測檔與 RTKLIB 執行檔只在 RTKLIB 模式可填。
* 架次表填了 Emlid Studio 的 ``_events.pos`` ⇒ 直接擋並說明要給整條軌跡。
* 選檔的篩選字串是「RTKLIB／Emlid Studio 軌跡 (*.pos)」。
* 從對話框收集的設定直接跑真的流水線：基站座標與 .pos 一致才採用 PPK。

合成數值（tests/test_survey_pipeline 的合成架次）；不放論壇下載的 Emlid Studio 檔。
"""
from __future__ import annotations

import pytest

from tests.test_survey_dialog import _QT_OK, filled_dialog, no_rtklib, qt_app  # noqa: F401  （fixtures）

pytestmark = pytest.mark.skipif(not _QT_OK, reason="Qt 匯入失敗")

if _QT_OK:
    from ui.dialogs import survey_process_dialog as spd

from survey import pipeline  # noqa: E402
from survey.ppk import rtklib  # noqa: E402

BASE_FIELDS = (("ed_base_lat", "25.0995"), ("ed_base_lon", "121.1995"), ("ed_base_h", "52.3"), ("ed_base_ant", "1.234"),
               ("ed_base_frame", "e-GNSS[2025]"))


def _pos_mode(dlg):
    dlg.rb_exposure_start.setChecked(True)
    dlg.grp_ppk.setChecked(True)
    dlg.rb_ppk_pos.setChecked(True)


def test_pos_mode_enables_base_fields_and_explains_the_ref_pos_check(qt_app):
    dlg = spd.SurveyProcessDialog()
    assert dlg.rb_ppk_rtklib.isChecked() and dlg.lbl_base_hint.text() == spd.BASE_HINT_RTKLIB
    assert dlg.box_base_rinex.isEnabled() and dlg.ed_convbin.isEnabled()
    dlg.rb_ppk_pos.setChecked(True)
    for name, _ in BASE_FIELDS:
        assert getattr(dlg, name).isEnabled(), name
    for w in (dlg.box_base_rinex, dlg.box_base_ubx, dlg.ed_convbin, dlg.ed_rnx2rtkp, dlg.btn_rtklib_detect):
        assert not w.isEnabled()
    hint = dlg.lbl_base_hint.text()
    assert hint == spd.BASE_HINT_POS and "% ref pos" in hint and "Emlid Studio" in hint and "_events.pos" in hint
    dlg.rb_ppk_rtklib.setChecked(True)
    assert dlg.box_base_rinex.isEnabled() and dlg.lbl_base_hint.text() == spd.BASE_HINT_RTKLIB


def test_pos_mode_requires_base_station_and_builds_it(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    _pos_mode(dlg)
    dlg._set_cell(0, 4, str(tmp_path / "rover.pos"))
    dlg.ed_base_frame.setText("e-GNSS[2025]")
    _, errors = dlg.collect_inputs()
    want = [f"PPK：{n}必填（要與解算 .pos 時輸入的一致，會核對 % ref pos）" for n in ("基站緯度", "基站經度", "基站橢球高", "天線高")]
    assert errors == want
    for name, v in BASE_FIELDS:
        getattr(dlg, name).setText(v)
    inp, errors = dlg.collect_inputs()
    assert errors == []                                         # 不需要基站觀測檔、也不需要裝 RTKLIB
    p = inp["kwargs"]["ppk"]
    assert not p.run_rtklib and p.base_obs is None and p.base_ubx is None
    assert p.base == rtklib.BaseStation(25.0995, 121.1995, 52.3, "e-GNSS[2025]", antenna_height_m=1.234)
    dlg.ed_base_lat.setText("95")
    _, errors = dlg.collect_inputs()
    assert any("超出範圍" in e for e in errors)


@pytest.mark.parametrize("name", ["rover_raw_20261020_events.pos", "ROVER_EVENTS.POS"])
def test_events_pos_is_refused_in_the_sortie_table(qt_app, no_rtklib, tmp_path, name):
    dlg = filled_dialog(tmp_path)
    _pos_mode(dlg)
    for n, v in BASE_FIELDS:
        getattr(dlg, n).setText(v)
    ev = tmp_path / name
    ev.write_text("% x\n", encoding="utf-8")
    dlg._set_cell(0, 4, str(ev))
    inp, errors = dlg.collect_inputs()
    assert inp is None and errors == [f"第 1 列：{name}：{pipeline.EVENTS_POS_REFUSAL}"]
    assert "整條軌跡" in errors[0]
    dlg._set_cell(0, 4, str(tmp_path / "rover.pos"))               # 同一次解算的軌跡檔 ⇒ 通過
    assert dlg.collect_inputs()[1] == []


def test_pos_browse_filter_names_emlid_studio(qt_app, monkeypatch):
    dlg = spd.SurveyProcessDialog()
    seen = []
    monkeypatch.setattr(spd.QFileDialog, "getOpenFileName",
                        staticmethod(lambda parent, cap, start, flt: (seen.append(flt), ("", ""))[1]))
    dlg.btn_browse_pos.click()
    assert seen == ["RTKLIB／Emlid Studio 軌跡 (*.pos)"] == [spd.POS_FILE_FILTER]


def test_dialog_pos_mode_runs_the_real_pipeline_and_checks_ref_pos(qt_app, no_rtklib, tmp_path):
    """對話框收集的設定 → 真的流水線：基站座標與 .pos 的 % ref pos 一致 ⇒ 採用 PPK；填錯 2 cm ⇒ 整架不用 PPK。"""
    from tests.test_survey_pipeline import GRS80, build_sortie
    s = build_sortie(tmp_path / "h1", uav="HERO1", sid="S1", seed=1)
    dlg = spd.SurveyProcessDialog()
    dlg.add_sortie_row("", str(s["bin"]), str(s["sess"]), pos_path=str(s["pos"]))
    _pos_mode(dlg)
    for n, v in BASE_FIELDS:
        getattr(dlg, n).setText(v)
    dlg.ed_geo_projection.setText(GRS80)
    dlg.ed_out_dir.setText(str(tmp_path / "ok"))
    inp, errors = dlg.collect_inputs()
    assert errors == []
    res = pipeline.process_campaign(inp["sorties"], inp["out_dir"], **inp["kwargs"])
    assert res.passed, res.qa["reasons"]
    assert {r["source"] for r in res.rows} == {"ppk"}
    assert res.sorties[0].ppk["trajectory"]["ref_pos_check"]["passed"] is True
    dlg.ed_base_h.setText("52.32")
    dlg.ed_out_dir.setText(str(tmp_path / "bad"))
    inp, errors = dlg.collect_inputs()
    assert errors == []
    res = pipeline.process_campaign(inp["sorties"], inp["out_dir"], **inp["kwargs"])
    assert not res.passed and {r["source"] for r in res.rows} == {"fc_interp"}
    assert "垂直 -20.0 mm" in res.sorties[0].ppk["error"]
