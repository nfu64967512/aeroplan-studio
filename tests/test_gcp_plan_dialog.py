"""GCP／檢核點規劃的 GUI：GcpPlanDialog（offscreen、假地圖）、SurveyProcessDialog 的 GCP／檢核點／風欄位、
地圖點圖層（3D JS 呼叫、2D folium、DualMapWidget 轉送、main.html 的 SURVEY_POINTS 段在無頭瀏覽器裡跑）、
main_window 的選單與區塊提供者（不建 MainWindow）。

瀏覽器那幾條要 SAFE_ALLOW_BROWSER=1（safe runner），否則 skip。
"""
from __future__ import annotations

import json
import os
import re
import threading
import types
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import PyQt6.QtWebEngineWidgets  # noqa: F401  先載入 QtWebEngine 再建 QApplication
    from PyQt6.QtWidgets import QApplication
    from ui.dialogs.gcp_plan_dialog import MAP_LAYER, GcpPlanDialog
    from ui.dialogs.survey_process_dialog import SurveyProcessDialog
    from ui.widgets import cesium_map_widget as cmw
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.widgets.map_widget import MapWidget
    from tests.test_survey_map_layers import _bare2d, _bare3d, _call, _launch, _render2d
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

from survey import gcp_plan as gp  # noqa: E402
from tests.test_survey_gcp_plan import AREA, BLOCKS, _fake_hv, rect  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
MAIN_HTML = Path(os.environ.get("SURVEY_POINTS_MAIN_HTML") or REPO / "ui" / "widgets" / "cesium_templates" / "main.html")


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


class FakeMap:
    def __init__(self, corners=()):
        self.corners = list(corners)
        self.calls = []

    def set_survey_points(self, layer_id, points):
        self.calls.append(("set", layer_id, list(points)))
        return {"3d": {"drawn": len(points)}, "2d": {"drawn": len(points)}}

    def clear_survey_points(self, layer_id):
        self.calls.append(("clear", layer_id))
        return True


# ─────────────────────────────────────────────────────────────────────────────
# GcpPlanDialog
# ─────────────────────────────────────────────────────────────────────────────

def test_generate_shows_counts_compares_presets_and_draws_on_the_map(qt_app):
    fm = FakeMap()
    dlg = GcpPlanDialog(map_widget=fm, polygons=lambda: [AREA], blocks=lambda: (BLOCKS, {"A1": {}}, "假 HARVEST"))
    assert dlg.lbl_blocks.text() == "假 HARVEST" and dlg.lbl_areas.text().endswith("1 塊")
    assert dlg.generate() is True
    assert dlg.plan.counts["GCP"] == 8 and dlg.plan.counts["CHK"] == 20
    assert dlg.points_table.rowCount() == 28 and dlg.points_table.item(0, 0).text() == "GCP01"
    assert dlg.compare_table.rowCount() == 2
    assert [dlg.compare_table.item(i, 1).text() for i in range(2)] == ["8", "14"]   # 交叉航帶 vs 加密 GCP 的 GCP 數
    assert "小專案聲明" in dlg.lbl_counts.text() and dlg.lbl_warnings.text() == ""
    assert "✔ GCP 總數 ≥ 5" in dlg.checks_view.toPlainText()
    assert dlg.plan.effective["blocks_source"] == "假 HARVEST"
    assert fm.calls[-1][0:2] == ("set", MAP_LAYER) and len(fm.calls[-1][2]) == 28
    assert {p["type"] for p in fm.calls[-1][2]} == {"GCP", "CHK"}
    assert dlg.cmb_projection.currentText() == "WGS84 UTM 51N"
    dlg.cmb_preset.setCurrentIndex(dlg.cmb_preset.findData("dense_gcp"))
    dlg.chk_asprs30.setChecked(True)
    assert dlg.spn_chk.value() == 30
    assert dlg.generate() is True and dlg.plan.counts["GCP"] == 14 and dlg.plan.counts["CHK"] == 30
    assert dlg.clear_map() is True and fm.calls[-1] == ("clear", MAP_LAYER)


def test_user_overrides_reach_the_planner_and_bad_text_is_refused(qt_app):
    dlg = GcpPlanDialog(polygons=[AREA])
    assert "沒有 HARVEST 規劃" in dlg.lbl_blocks.text()
    dlg.ed_edge_band.setText("abc")
    assert dlg.generate() is False and "邊緣帶寬 不是數字" in dlg.lbl_errors.text()
    dlg.ed_edge_band.setText("20")
    dlg.ed_gcp_spacing.setText("150")
    dlg.cmb_per_block.setCurrentText("否")
    assert dlg.generate() is True
    e = dlg.plan.effective
    assert (e["edge_band_m"], e["inset_m"], e["gcp_min_spacing_m"]) == (20.0, 30.0, 150.0)
    assert e["sources"]["edge_band_m"] == "使用者" and e["per_block"] is False
    assert dlg.plan.counts["GCP"] == 5 and dlg.plan.counts["seams"] == 0
    dlg.spn_chk.setValue(10)
    dlg.chk_asprs30.blockSignals(True)
    dlg.chk_asprs30.setChecked(True)
    dlg.chk_asprs30.blockSignals(False)
    assert dlg.generate() is False and "ASPRS" in dlg.lbl_errors.text()


def test_a_failing_block_provider_is_reported_not_silently_dropped(qt_app):
    def boom():
        raise RuntimeError("壞掉的規劃")
    dlg = GcpPlanDialog(polygons=[AREA], blocks=boom)
    assert "取 HARVEST 區塊失敗" in dlg.lbl_blocks.text()
    assert dlg.generate() is False and "壞掉的規劃" in dlg.lbl_errors.text() and dlg.plan is None
    dlg.chk_use_blocks.setChecked(False)                             # 使用者明確選擇不分區塊
    assert dlg.generate() is True and dlg.plan.effective["blocks"] == []
    assert dlg.plan.effective["blocks_source"] == "使用者取消勾選：不分 UAV 區塊"
    none = GcpPlanDialog(polygons=lambda: [])
    assert none.generate() is False and "沒有任何作業區" in none.lbl_errors.text()


def test_export_and_gcp_list_from_survey(qt_app, tmp_path):
    dlg = GcpPlanDialog(polygons=[AREA], blocks=BLOCKS)
    assert dlg.export(str(tmp_path)) is None and "還沒有產生" in dlg.lbl_errors.text()
    assert dlg.generate() is True
    paths = dlg.export(str(tmp_path / "out"))
    assert paths and paths["gcp_template"].read_text(encoding="utf-8").splitlines()[0] == "WGS84 UTM 51N"
    assert json.loads(paths["effective"].read_text(encoding="utf-8"))["export"]["vertical"] == "ellipsoidal"
    assert dlg.export(str(tmp_path / "out")) is None and "已有" in dlg.lbl_errors.text()
    # 外業填好實測值 → gcp_list.txt
    rows = paths["survey_csv"].read_text(encoding="utf-8-sig").splitlines()
    head = rows[0].split(",")
    i = {c: k for k, c in enumerate(head)}
    out = [rows[0]]
    for r in rows[1:]:
        r = r.split(",")
        r[i["lat"]], r[i["lon"]], r[i["h"]], r[i["frame"]] = r[i["planned_lat"]], r[i["planned_lon"]], "51.2", "EPSG:4326"
        out.append(",".join(r))
    sv = tmp_path / "filled.csv"
    sv.write_text("\n".join(out) + "\n", encoding="utf-8")
    dlg.ed_survey_csv.setText(str(sv))
    dst = dlg.gcp_list_from_survey(str(tmp_path / "out"))
    assert dst == tmp_path / "out" / "gcp_list.txt"
    data = [ln.split() for ln in dst.read_text(encoding="utf-8").splitlines()[1:] if not ln.startswith("#")]
    assert len(data) == 8 and all(d[2] == "51.200" for d in data)
    assert dlg.gcp_list_from_survey(str(tmp_path / "out")) is None and "已存在" in dlg.lbl_errors.text()


# ─────────────────────────────────────────────────────────────────────────────
# SurveyProcessDialog：GCP、檢核點、風
# ─────────────────────────────────────────────────────────────────────────────

def _result(tmp_path, *, height="ellipsoidal"):
    from tests.test_survey_dialog import fake_result
    res = fake_result(tmp_path)
    res.effective["height"] = height
    return res


def _gcp_file(tmp_path, *, image="HERO1_S1_000001.jpg", name="GCP01", n=1):
    p = tmp_path / "gcp_list.txt"
    p.write_text("EPSG:4326\n" + "".join(f"121.2 25.1 48.5 {10 + k} {20 + k} {image} {name}\n" for k in range(n)),
                 encoding="utf-8")
    return p


def test_survey_dialog_button_opens_the_gcp_planner_with_the_same_sources(qt_app):
    fm = FakeMap()
    prov = lambda: (BLOCKS, {}, "HARVEST")                           # noqa: E731
    dlg = SurveyProcessDialog(map_widget=fm, polygons=lambda: [AREA], gcp_blocks=prov)
    g = dlg.open_gcp_plan()
    assert isinstance(g, GcpPlanDialog) and g.blocks_source is prov and g.map_widget is fm
    assert dlg.open_gcp_plan() is g
    assert g.generate() is True and g.plan.counts["seams"] == 2
    g.close()


def test_wind_limit_is_an_explicit_kwarg(qt_app, tmp_path):
    from tests.test_survey_dialog import filled_dialog
    dlg = filled_dialog(tmp_path)
    dlg.rb_exposure_start.setChecked(True)
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["wind_limit_mps"] == 3.0
    dlg.spn_wind_limit.setValue(5.0)
    assert dlg.collect_inputs()[0]["kwargs"]["wind_limit_mps"] == 5.0


def test_gcp_file_needs_its_vertical_datum_and_is_validated_before_upload(qt_app, tmp_path):
    dlg = SurveyProcessDialog()
    dlg.result = _result(tmp_path)
    dlg.spn_res.setValue(2.0)
    dlg.ed_gcp_txt.setText(str(tmp_path / "missing.txt"))
    assert dlg.odm_block_reason().startswith("GCP 檔不存在")
    g = _gcp_file(tmp_path, n=5)
    dlg.ed_gcp_txt.setText(str(g))
    assert dlg.odm_block_reason() == "選了 GCP 檔，但還沒選它的 geo_z 高程基準" and not dlg.btn_odm.isEnabled()
    dlg.cmb_gcp_vertical.setCurrentIndex(dlg.cmb_gcp_vertical.findData("orthometric"))
    assert dlg.odm_block_reason() is None
    jobs = []
    dlg.odm_run_fn = lambda job, **k: (jobs.append(job), {"ortho_tiles": None})[1]
    assert dlg.submit_odm() is False and "混用" in dlg.lbl_errors.text() and jobs == []     # geo.txt 是橢球高
    dlg.cmb_gcp_vertical.setCurrentIndex(dlg.cmb_gcp_vertical.findData("ellipsoidal"))
    assert dlg.submit_odm() is True
    from tests.test_survey_dialog import wait_worker
    wait_worker(dlg, "_odm_worker")
    assert jobs[0]["gcp_txt"] == str(g) and jobs[0]["gcp"]["n_gcps"] == 1 and jobs[0]["gcp"]["n_lines"] == 5
    sub = dlg.result.effective["odm_submission"]
    assert sub["gcp_txt"] == str(g) and sub["gcp"]["gcp_vertical"] == "ellipsoidal"
    assert any("GCP 只有 1 個" in w for w in sub["gcp"]["warnings"]) and "GCP：1 個、5 行" in dlg.lbl_gcp.text()
    # 影像檔名不在上傳清單 ⇒ 不送
    dlg.ed_gcp_txt.setText(str(_gcp_file(tmp_path, image="OTHER.jpg")))
    assert dlg.submit_odm() is False and "OTHER.jpg 不在本次要上傳的影像裡" in dlg.lbl_errors.text()


def test_checkpoint_ids_in_the_gcp_file_block_the_upload(qt_app, tmp_path):
    dlg = SurveyProcessDialog()
    dlg.result = _result(tmp_path)
    dlg.spn_res.setValue(2.0)
    dlg.ed_gcp_txt.setText(str(_gcp_file(tmp_path, name="CHK03", n=5)))
    dlg.cmb_gcp_vertical.setCurrentIndex(dlg.cmb_gcp_vertical.findData("ellipsoidal"))
    sv = tmp_path / "survey.csv"
    sv.write_text("id,type,lat,lon,h,frame,vertical_datum,sigma_h_m,sigma_v_m\n"
                  "CHK03,CHK,25.1,121.2,48,EPSG:4326,ellipsoidal,0.01,0.01\n", encoding="utf-8")
    dlg.ed_chk_survey.setText(str(sv))
    assert dlg.submit_odm() is False and "檢核點 ['CHK03']" in dlg.lbl_errors.text()


def _write_dsm(path, z0=40.0):
    import numpy as np
    import rasterio
    from pyproj import Transformer
    from rasterio.transform import from_origin
    t = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)
    x, y = t.transform(121.2, 25.1)
    z = np.full((200, 200), z0, "float32")
    with rasterio.open(path, "w", driver="GTiff", width=200, height=200, count=1, dtype="float32", crs="EPSG:32651",
                       transform=from_origin(x - 50, y + 50, 0.5, 0.5), nodata=-9999.0) as ds:
        ds.write(z, 1)


def test_checkpoint_report_after_odm_uses_the_dsm_and_the_geo_txt_datum(qt_app, tmp_path):
    from survey.pipeline import find_odm_dsm
    dlg = SurveyProcessDialog()
    res = _result(tmp_path)
    dlg._on_finished(res)
    assert dlg.cmb_dsm_vertical.currentData() == "ellipsoidal" and "預填" in dlg.lbl_dsm_vertical.text()
    sv = tmp_path / "survey.csv"
    sv.write_text("id,type,lat,lon,h,frame,vertical_datum,sigma_h_m,sigma_v_m\n"
                  "C1,CHK,25.1,121.2,39.95,EPSG:4326,ellipsoidal,0.01,0.02\n"
                  "C2,CHK,25.10005,121.20005,40.03,EPSG:4326,ellipsoidal,0.01,0.02\n", encoding="utf-8")
    dlg.ed_chk_survey.setText(str(sv))
    ext = tmp_path / "result_t"
    (ext / "odm_dem").mkdir(parents=True)
    _write_dsm(ext / "odm_dem" / "dsm.tif")
    assert find_odm_dsm(ext) == ext / "odm_dem" / "dsm.tif"
    dlg.ed_out_dir.setText(str(tmp_path / "out"))
    dlg._on_odm_finished({"uuid": "t", "extract_dir": str(ext), "ortho_tiles": None})
    rep = dlg.checkpoint_report
    assert rep is not None and dlg.ed_dsm.text() == str(ext / "odm_dem" / "dsm.tif")
    assert [round(r["dZ"], 4) for r in rep["points"]] == [0.05, -0.03]
    assert rep["effective"]["dsm_vertical_source"] == "geo.txt"
    assert "小專案聲明" in dlg.chk_report_view.toPlainText() and dlg.tabs.currentIndex() == 4
    assert (tmp_path / "out" / "checkpoints" / "checkpoint_report.json").is_file()
    # 操作員自己改成 EGM96 ⇒ 不再被預填蓋掉；沒給 N 就拒絕
    dlg.cmb_dsm_vertical.setCurrentIndex(dlg.cmb_dsm_vertical.findData("EGM96"))
    dlg._on_dsm_vertical_picked()
    dlg._on_finished(res)
    assert dlg.cmb_dsm_vertical.currentData() == "EGM96"
    assert dlg.compute_checkpoint_report() is None and "沒有明確給大地起伏" in dlg.chk_report_view.toPlainText()
    dlg.ed_chk_undulation.setText("18.54")
    rep2 = dlg.compute_checkpoint_report()
    assert [round(r["dZ"], 2) for r in rep2["points"]] == [18.59, 18.51]      # 40 + 18.54 − h（DSM 當正高換回橢球高）


def test_checkpoint_report_refuses_missing_inputs(qt_app, tmp_path):
    dlg = SurveyProcessDialog()
    assert dlg.compute_checkpoint_report() is None
    txt = dlg.chk_report_view.toPlainText()
    assert "實測檢核點 CSV 不存在" in txt and "至少要給正射量測 CSV 或 DSM" in txt
    dlg.ed_dsm.setText(str(tmp_path / "x.tif"))
    dlg.compute_checkpoint_report()
    assert "DSM 不存在" in dlg.chk_report_view.toPlainText()


# ─────────────────────────────────────────────────────────────────────────────
# 地圖點圖層
# ─────────────────────────────────────────────────────────────────────────────

PTS = [{"id": "GCP01", "type": "GCP", "lat": 25.07, "lon": 121.21, "label": "GCP01（A1）"},
       {"id": "CHK01", "type": "CHK", "lat": 25.071, "lon": 121.211, "label": "CHK01"},
       {"id": "BAD", "type": "CHK", "lat": None, "lon": 121.2, "label": "BAD"}]


def test_3d_sends_compact_rows_and_python_colours_match_the_page():
    w = _bare3d()
    st = CesiumMapWidget.set_survey_points(w, "g", PTS)
    lid, data = _call(w._sent[-1], "setSurveyPoints")
    assert lid == "g" and data == {"rows": [[25.07, 121.21, "GCP", "GCP01（A1）"], [25.071, 121.211, "CHK", "CHK01"]]}
    assert st == {"rows": 3, "drawn": 2, "skipped": 1, "by_type": {"GCP": 1, "CHK": 1}}
    assert CesiumMapWidget.clear_survey_points(w, "g") is True and w._sent[-1] == 'clearSurveyPoints("g")'
    assert CesiumMapWidget.clear_survey_points(w, "g") is False
    with pytest.raises(ValueError):
        CesiumMapWidget.set_survey_points(w, "", PTS)
    with pytest.raises(ValueError, match="上限"):
        CesiumMapWidget.set_survey_points(w, "big", [PTS[0]] * (cmw.SURVEY_POINTS_MAX + 1))
    sec = _points_section()
    js = dict(re.findall(r"(\w+):\s*'(#[0-9A-Fa-f]{6})'", re.search(r"var _SURVEY_POINT_COLORS = (\{[^}]*\});",
                                                                     sec).group(1)))
    assert js == cmw.SURVEY_POINT_COLORS
    assert re.search(r"var _SURVEY_POINT_OTHER_COLOR = '(#[0-9A-Fa-f]{6})'", sec).group(1) == cmw.SURVEY_POINT_OTHER_COLOR


def test_2d_draws_coloured_points_with_permanent_labels_and_clear_survey_layers_keeps_them():
    w = _bare2d()
    st = MapWidget.set_survey_points(w, "g", PTS)
    assert st["drawn"] == 2 and w._renders == [1]
    html = _render2d(w)
    markers = re.findall(r'L\.circleMarker\(\s*\[([\d.]+), ([\d.]+)\],\s*(\{.*?\})\s*\)', html, flags=re.S)
    got = {(float(a), float(b)): json.loads(re.sub(r",(\s*[\]}])", r"\1", o))["fillColor"] for a, b, o in markers}
    assert got == {(25.07, 121.21): "#FF3D00", (25.071, 121.211): "#2979FF"}
    assert html.count('"permanent": true') >= 2
    assert "GCP01（A1）" in html or json.dumps("GCP01（A1）")[1:-1] in html
    MapWidget.clear_survey_layers(w)                                  # 正射／位姿的清除不碰佈設
    assert "g" in w._survey_point_layers
    assert MapWidget.clear_survey_points(w, "g") is True and not w._survey_point_layers


def test_dual_map_forwards_survey_points_to_both_maps():
    assert "set_survey_points" in DualMapWidget.__dict__ and "clear_survey_points" in DualMapWidget.__dict__
    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = _bare2d(), _bare3d()
    eff = DualMapWidget.set_survey_points(d, "g", iter(PTS))
    assert eff["3d"]["drawn"] == 2 and eff["2d"]["drawn"] == 2
    assert _call(d.map_3d._sent[-1], "setSurveyPoints")[0] == "g" and "g" in d.map_2d._survey_point_layers
    assert DualMapWidget.clear_survey_points(d, "g") is True


def _points_section() -> str:
    src = MAIN_HTML.read_text(encoding="utf-8")
    a = src.index("SURVEY_POINTS_BEGIN")
    a = src.rindex("\n", 0, a) + 1
    return src[a:src.index("// SURVEY_POINTS_END")]


def _layers_section() -> str:
    from tests.test_survey_map_layers import _section
    return _section()


_STUB = r"""
window.__ents = [];
window.Cesium = {
  Color: { fromCssColorString: (s) => ({ css: s }), BLACK: { css: '#000000' } },
  Cartesian3: { fromDegrees: (lon, lat, h) => [lon, lat, h] },
  Cartesian2: function (x, y) { this.x = x; this.y = y; },
  HeightReference: { CLAMP_TO_GROUND: 'CLAMP' },
  LabelStyle: { FILL_AND_OUTLINE: 'FILL_AND_OUTLINE' },
  Rectangle: { fromDegrees: () => ({}) },
};
window.viewer = {
  entities: { add(o) { __ents.push(o); return o; }, remove(o) { const i = __ents.indexOf(o); if (i >= 0) __ents.splice(i, 1); return i >= 0; } },
  imageryLayers: { add() {}, remove() {}, contains() { return false; } },
  scene: { primitives: { add(c) { return c; }, remove() { return true; } } },
};
"""


@pytest.fixture(scope="module")
def stub_page(qt_app, tmp_path_factory):
    sync_api = pytest.importorskip("playwright.sync_api")
    try:
        pw = sync_api.sync_playwright().start()
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"無法啟動 Playwright（safe runner 要設 SAFE_ALLOW_BROWSER=1）：{e}")
    d = tmp_path_factory.mktemp("pts")
    f = d / "stub.html"
    f.write_text("<!doctype html><html><head><meta charset='utf-8'></head><body></body></html>", encoding="utf-8")
    try:
        b = _launch(pw, [])
        pg = b.new_page()
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.goto(f.as_uri())
        pg.add_script_tag(content=_STUB)
        pg.add_script_tag(content=_layers_section())
        pg.add_script_tag(content=_points_section())
    except Exception as e:                                   # pragma: no cover
        pw.stop()
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    pg._errs = errs
    yield pg
    b.close()
    pw.stop()


def test_page_draws_one_labelled_clamped_entity_per_valid_point(stub_page):
    rows = [[25.07, 121.21, "GCP", "GCP01（A1）"], [25.071, 121.211, "CHK", "CHK01"], [None, 121.2, "CHK", "X"],
            ["", 121.2, "CHK", "Y"], [25.0, 200.0, "GCP", "Z"], [25.072, 121.212, "OTHER", "O"]]
    st = stub_page.evaluate("(rows) => setSurveyPoints('g', {rows})", rows)
    assert st == {"drawn": 3, "skipped": 3, "byType": {"GCP": 1, "CHK": 1, "OTHER": 1}}
    ents = stub_page.evaluate("""() => __ents.map(e => ({pos: e.position, col: e.point.color.css, px: e.point.pixelSize,
        hr: e.point.heightReference, text: e.label.text, lcol: e.label.fillColor.css, dd: e.point.disableDepthTestDistance}))""")
    assert [e["pos"] for e in ents] == [[121.21, 25.07, 0], [121.211, 25.071, 0], [121.212, 25.072, 0]]
    assert [e["col"] for e in ents] == ["#FF3D00", "#2979FF", "#FFFFFF"] and [e["lcol"] for e in ents] == \
        ["#FF3D00", "#2979FF", "#FFFFFF"]
    assert [e["text"] for e in ents] == ["GCP01（A1）", "CHK01", "O"] and ents[0]["px"] > ents[1]["px"]
    assert {e["hr"] for e in ents} == {"CLAMP"} and all(e["dd"] is None or e["dd"] > 1e300 for e in ents)
    # 同代號重畫 = 換掉；clearSurveyLayers（正射／位姿）不碰；clearSurveyPoints 才清
    stub_page.evaluate("(rows) => setSurveyPoints('g', rows)", rows[:1])
    assert stub_page.evaluate("__ents.length") == 1
    stub_page.evaluate("clearSurveyLayers()")
    assert stub_page.evaluate("surveyPointsInfo()") == {"g": {"drawn": 1, "skipped": 0, "byType": {"GCP": 1},
                                                               "entities": 1}}
    assert stub_page.evaluate("clearSurveyPoints('g')") is True and stub_page.evaluate("__ents.length") == 0
    assert stub_page.evaluate("clearSurveyPoints('g')") is False
    assert not stub_page._errs, stub_page._errs


# ─────────────────────────────────────────────────────────────────────────────
# main_window（不建 MainWindow）
# ─────────────────────────────────────────────────────────────────────────────

def test_menu_action_is_wired_to_the_handler():
    src = (REPO / "ui" / "main_window.py").read_text(encoding="utf-8")
    m = re.search(r'action = tools_menu\.addAction\("GCP／檢核點規劃…"\)\s*\n(?:.*\n){0,2}?\s*action\.triggered\.connect\('
                  r'self\.on_gcp_plan\)', src)
    assert m, "工具選單要有「GCP／檢核點規劃…」並接到 on_gcp_plan"


def test_blocks_provider_reads_the_latest_harvest_plan_lazily():
    from ui import main_window as mw
    win = types.SimpleNamespace(_hv_coordinator=None)
    prov = mw.gcp_blocks_provider(win)
    blocks, info, src = prov()
    assert blocks is None and "沒有 HARVEST 規劃" in src
    win._hv_coordinator = _fake_hv()                                  # 規劃在對話框開著時才出現
    blocks, info, src = prov()
    assert sorted(blocks) == ["A1", "B2"] and info["B2"]["drone_id"] == 2 and "A1（#1）" in src


def test_on_gcp_plan_and_on_survey_process_hand_over_areas_and_blocks(qt_app, monkeypatch):
    from ui import main_window as mw
    made = {}

    class G:
        def __init__(self, parent=None, map_widget=None, polygons=None, blocks=None):
            made["gcp"] = dict(parent=parent, map_widget=map_widget, polygons=polygons, blocks=blocks)

        def show(self):
            made["shown"] = True

    class S:
        def __init__(self, parent=None, map_widget=None, polygons=None, gcp_blocks=None):
            made["survey"] = dict(polygons=polygons, gcp_blocks=gcp_blocks)

        def show(self):
            pass

    monkeypatch.setitem(__import__("sys").modules, "ui.dialogs.gcp_plan_dialog",
                        types.SimpleNamespace(GcpPlanDialog=G))
    monkeypatch.setitem(__import__("sys").modules, "ui.dialogs.survey_process_dialog",
                        types.SimpleNamespace(SurveyProcessDialog=S))
    win = types.SimpleNamespace(survey_areas=[AREA], corners=[], map_widget=object(), _hv_coordinator=_fake_hv())
    win.survey_area_polygons = types.MethodType(mw.MainWindow.survey_area_polygons, win)
    mw.MainWindow.on_gcp_plan(win)
    g = made["gcp"]
    assert g["parent"] is win and g["map_widget"] is win.map_widget and made["shown"]
    assert g["polygons"]() == [AREA]                                   # 呼叫時才取（非模態）
    assert sorted(g["blocks"]()[0]) == ["A1", "B2"]
    mw.MainWindow.on_survey_process(win)
    assert made["survey"]["polygons"] == [AREA] and sorted(made["survey"]["gcp_blocks"]()[0]) == ["A1", "B2"]


def test_real_gcp_dialog_from_a_fake_window_draws_on_a_dual_map(qt_app):
    """main_window → GcpPlanDialog（真的）→ DualMapWidget（裸）：兩張圖都拿到點。"""
    from ui import main_window as mw
    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = _bare2d(), _bare3d()
    win = types.SimpleNamespace(survey_areas=[AREA], corners=[], map_widget=d, _hv_coordinator=None)
    win.survey_area_polygons = types.MethodType(mw.MainWindow.survey_area_polygons, win)
    dlg = GcpPlanDialog(map_widget=d, polygons=win.survey_area_polygons, blocks=mw.gcp_blocks_provider(win))
    assert dlg.generate() is True and dlg.plan.counts["GCP"] == 5
    lid, data = _call(d.map_3d._sent[-1], "setSurveyPoints")
    assert lid == MAP_LAYER and len(data["rows"]) == 25
    assert len(d.map_2d._survey_point_layers[MAP_LAYER]["rows"]) == 25
    win._hv_coordinator = _fake_hv()
    assert dlg.generate() is True and dlg.plan.effective["blocks"] == ["A1", "B2"]
    assert any("只蓋到作業區的" in w for w in dlg.plan.warnings)      # 假規劃只蓋一部分 ⇒ 說出來
    dlg.close()
