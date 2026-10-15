"""航測後處理 GUI：SurveyProcessDialog（offscreen）、背景 worker 接線、ODM 按鈕把關、地圖呼叫，
以及 DualMapWidget 的航測圖層轉送（2D＋3D 都要到）。

不建 MainWindow；對話框用假的流水線函式（process_fn／odm_run_fn）與假的地圖物件。
DualMapWidget 用「Python 殼、C++ 端沒建」的裸實例（同 test_clear_all_wipes_map 的寫法），
2D／3D 是 test_survey_map_layers 的裸 MapWidget／CesiumMapWidget —— 跑的是真的下游方法。
"""
from __future__ import annotations

import os
import threading
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import PyQt6.QtWebEngineWidgets  # noqa: F401  先載入 QtWebEngine 再建 QApplication
    from PyQt6.QtWidgets import QApplication
    from ui.dialogs.survey_process_dialog import ORTHO_LAYER, PHOTO_POSE_LAYER, SurveyProcessDialog
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.workers.survey_worker import OdmSubmitWorker, SurveyProcessWorker
    from tests.test_survey_map_layers import _bare2d, _bare3d, _call, _make_tiles
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

from survey import datum, deliverable, synth  # noqa: E402
from survey.pipeline import CampaignResult, PipelineCanceled, PipelineError, PpkSettings  # noqa: E402
from survey.ppk import rtklib  # noqa: E402


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def no_rtklib(monkeypatch):
    def nf(name, explicit=None):
        raise rtklib.RtklibNotFound(f"找不到 RTKLIB 的 {name}（PATH 上沒有）")
    monkeypatch.setattr(rtklib, "find_executable", nf)


def make_session(d: Path, uav="HERO1", *, rover=True, readout=20.0):
    cam = {"sensor": "imx477", "lens_mm": 6.0, "pixel_um": 1.55, "width": 64, "height": 48, "readout_ms": readout,
           "shutter": "rolling", "kernel": "6.6.y", "dtoverlay": "imx477"}
    synth.synth_session_from_fc(d / "sess", [1_000_000, 2_000_000, 3_000_000], uav=uav, session_id="S1", camera=cam)
    if rover:
        (d / "sess" / "gnss").mkdir()
        (d / "sess" / "gnss" / "rover.ubx").write_bytes(b"\xb5\x62")
    (d / "fc.BIN").write_bytes(b"\xa3\x95")
    (d / "rover.pos").write_text("% x\n", encoding="utf-8")
    return d / "fc.BIN", d / "sess", d / "rover.pos"


def filled_dialog(tmp_path, map_widget=None, **sess_kw):
    dlg = SurveyProcessDialog(map_widget=map_widget)
    b, s, _ = make_session(tmp_path, **sess_kw)
    dlg.add_sortie_row(bin_path=str(b), session_dir=str(s))
    dlg.ed_out_dir.setText(str(tmp_path / "out"))
    return dlg


class FakeMap:
    def __init__(self, corners=()):
        self.corners = list(corners)
        self.calls = []

    def load_photo_poses_file(self, layer_id, path, color=None):
        self.calls.append(("load_photo_poses_file", layer_id, path))
        return {"3d": {"id": layer_id}, "2d": {"drawn": 2}}

    def add_ortho_layer(self, layer_id, path, scheme="tms", bounds=None, alpha=1.0):
        self.calls.append(("add_ortho_layer", layer_id, path, scheme))
        return {"3d": {"bounds_source": "tilemapresource.xml"}, "2d": {}}


def fake_result(tmp_path, *, passed=True, gsd=2.7, images=True):
    out = tmp_path / "res"
    out.mkdir(exist_ok=True)
    row = {c: None for c in deliverable.F2_COLUMNS}
    row.update(image_id="HERO1_S1_000001", file="images/000001.jpg", uav="HERO1", seq=1, lat_deg=25.1,
               lon_deg=121.2, h_ell_m=150.0, yaw_deg=95.0, source="fc_interp", crs_h="EPSG:4326",
               crs_v="ellipsoidal", flags="")
    csv_p = deliverable.write_photos_csv(out / "photos.csv", [row])
    geo = out / "odm" / "geo.txt"
    geo.parent.mkdir(exist_ok=True)
    geo.write_text("EPSG:4326\n", encoding="utf-8")
    img = out / "odm" / "images" / "HERO1_S1_000001.jpg"
    img.parent.mkdir(exist_ok=True)
    img.write_bytes(b"jpg")
    reasons = [] if passed else ["HERO1：第 1 帶張數不守恆（假的）"]
    return CampaignResult(out_dir=out, sorties=[], rows=[row], effective={"sensor_ts_semantics": "exposure_start"},
                          paths={"photos_csv": csv_p, "geo_txt": geo if images else None, "images_dir": img.parent},
                          images=[img] if images else [],
                          qa={"passed": passed, "reasons": reasons,
                              "exposure": {"computed": True, "scope": "測區多邊形內", "frac_ge": {"5": 0.97}}},
                          odm={"rolling_shutter_readout_ms": 20.0, "rolling_shutter_error": None, "gsd_cm": gsd,
                               "n_images": 1})


def wait_worker(dlg, attr="_worker"):
    w = getattr(dlg, attr)
    assert w is not None
    assert w.wait(10000)
    for _ in range(20):
        QApplication.processEvents()
        if getattr(dlg, attr) is None:
            break
    assert getattr(dlg, attr) is None


# ─────────────────────────────────────────────────────────────────────────────
# 對話框：建立與必填欄位
# ─────────────────────────────────────────────────────────────────────────────

def test_dialog_builds_offscreen_with_explicit_defaults(qt_app, no_rtklib):
    dlg = SurveyProcessDialog()
    assert dlg.ed_odm_url.text() == "http://127.0.0.1:3000"
    assert dlg.sensor_ts_semantics() is None                         # 沒有預設語意
    assert not dlg.rb_exposure_start.isChecked() and not dlg.rb_readout_start.isChecked()
    assert dlg.grp_ppk.isChecked() is False and dlg.spn_res.value() == 0.0
    assert dlg.lbl_rtklib.text().startswith("未安裝 RTKLIB")
    assert not dlg.btn_odm.isEnabled() and dlg.odm_block_reason() == "還沒有處理結果"
    assert dlg.btn_run.isEnabled() and not dlg.btn_cancel.isEnabled()
    assert not dlg.chk_rolling.isEnabled()                           # 還沒有會期 ⇒ 讀出時間未知


def test_rtklib_detected_state(qt_app, monkeypatch, tmp_path):
    monkeypatch.setattr(rtklib, "find_executable", lambda name, explicit=None: f"C:/rtk/{name}.exe")
    dlg = SurveyProcessDialog()
    assert dlg.lbl_rtklib.text() == "RTKLIB：convbin = C:/rtk/convbin.exe；rnx2rtkp = C:/rtk/rnx2rtkp.exe"
    dlg.detect_rtklib()
    assert dlg.ed_convbin.text() == "C:/rtk/convbin.exe" and dlg.ed_rnx2rtkp.text() == "C:/rtk/rnx2rtkp.exe"


def test_semantics_must_be_chosen(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    inp, errors = dlg.collect_inputs()
    assert inp is None and errors == ["請選擇 SensorTimestamp 語意（台架 B3 未定案，沒有預設值）"]
    assert dlg.start_processing() is False and dlg._worker is None
    assert "SensorTimestamp" in dlg.lbl_errors.text()
    dlg.rb_readout_start.setChecked(True)
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["sensor_ts_semantics"] == "readout_start"
    assert inp["out_dir"] == str(tmp_path / "out") and inp["kwargs"]["ppk"] is None


def test_rows_autofill_callsign_and_rover_ubx(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    assert dlg._cell(0, 0) == "HERO1"                                 # 取自 session.json
    assert dlg._cell(0, 3) == str(tmp_path / "sess" / "gnss" / "rover.ubx")
    assert dlg.lbl_readout.text().startswith("20.00 ms") and dlg.chk_rolling.isChecked()
    dlg.rb_exposure_start.setChecked(True)
    inp, _ = dlg.collect_inputs()
    s = inp["sorties"][0]
    assert (s.callsign, Path(s.bin_path), Path(s.session_dir)) == ("HERO1", tmp_path / "fc.BIN", tmp_path / "sess")
    assert Path(s.rover_ubx) == tmp_path / "sess" / "gnss" / "rover.ubx" and s.pos_path is None
    # 會期沒有實測讀出時間 ⇒ 不能開滾動快門校正
    d2 = tmp_path / "noro"
    d2.mkdir()
    dlg2 = filled_dialog(d2, readout=None)
    assert dlg2.lbl_readout.text().startswith("不能開") and not dlg2.chk_rolling.isEnabled()
    assert not dlg2.chk_rolling.isChecked()


def test_geoid_blank_is_unknown_never_zero(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    dlg.rb_exposure_start.setChecked(True)
    inp, _ = dlg.collect_inputs()
    assert inp["kwargs"]["geoid"] is None and inp["kwargs"]["height"] == "ellipsoidal"
    dlg.cmb_height.setCurrentIndex(1)                                 # 正高
    inp, errors = dlg.collect_inputs()
    assert inp is None and errors == ["選了正高但大地起伏 N 空白：無法算正高（不會用 N=0）"]
    dlg.ed_geoid_n.setText("18.54")
    dlg.cmb_geoid_datum.setCurrentText("TWVD2001")
    inp, errors = dlg.collect_inputs()
    g = inp["kwargs"]["geoid"]
    assert isinstance(g, datum.GeoidModel) and (g.datum, g.undulation_m) == ("TWVD2001", 18.54)
    assert inp["kwargs"]["height"] == "orthometric"
    dlg.ed_geoid_n.setText("abc")
    _, errors = dlg.collect_inputs()
    assert errors == ["大地起伏 N 不是數字：'abc'"]
    dlg.ed_geoid_n.setText("")
    dlg.cmb_height.setCurrentIndex(0)
    dlg.ed_ground_h.setText("48.5")
    dlg.chk_use_polygon.setChecked(True)                              # 沒有地圖 ⇒ 不能以測區算
    _, errors = dlg.collect_inputs()
    assert errors == ["地圖上的測區角點不到 3 個，無法以測區計算視數"]


def test_polygon_comes_from_map_corners(qt_app, no_rtklib, tmp_path):
    fm = FakeMap(corners=[(25.1, 121.2), (25.1, 121.21), (25.11, 121.21)])
    dlg = filled_dialog(tmp_path, map_widget=fm)
    dlg.rb_exposure_start.setChecked(True)
    dlg.chk_use_polygon.setChecked(True)
    dlg.ed_ground_h.setText("48.5")
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["polygons"] == [fm.corners] and inp["kwargs"]["ground_h_m"] == 48.5
    assert "polygon" not in inp["kwargs"]
    assert inp["kwargs"]["input_sources"]["polygons"] == "地圖上正在畫的測區角點（1 塊）"
    assert dlg.lbl_polygon_source.text() == "地圖上正在畫的測區角點（1 塊）：3 個角點"


# ─────────────────────────────────────────────────────────────────────────────
# R2：多區／已完成的區（MainWindow.survey_areas；完成本區後地圖角點被清空）
# ─────────────────────────────────────────────────────────────────────────────
RING_A = [(25.1, 121.2), (25.1, 121.21), (25.11, 121.21)]
RING_B = [(25.12, 121.2), (25.12, 121.21), (25.13, 121.21), (25.13, 121.2)]
MAIN_SRC = "主視窗的作業區（已完成的區＋正在畫且 ≥3 角點的區）"


def _mainwindow_areas(survey_areas, corners):
    """真的 MainWindow.survey_area_polygons（不建 MainWindow；用假的 self）。"""
    import types
    from ui.main_window import MainWindow
    win = types.SimpleNamespace(survey_areas=survey_areas, corners=corners)
    return win, (lambda: MainWindow.survey_area_polygons(win))


def test_committed_area_is_used_when_map_corners_are_empty(qt_app, no_rtklib, tmp_path):
    """R2 stage C：區已「完成本區」⇒ survey_areas 有 1 塊、地圖角點 [] ⇒ 舊版拒絕「角點不到 3 個」。"""
    win, provider = _mainwindow_areas([list(RING_A)], [])
    polys = provider()
    assert polys == [RING_A]                                           # 確認 MainWindow 回傳的形狀
    dlg = SurveyProcessDialog(map_widget=FakeMap(corners=[]), polygons=polys)
    b, s, _ = make_session(tmp_path)
    dlg.add_sortie_row(bin_path=str(b), session_dir=str(s))
    dlg.ed_out_dir.setText(str(tmp_path / "out"))
    dlg.rb_exposure_start.setChecked(True)
    dlg.ed_ground_h.setText("48.5")
    dlg.chk_use_polygon.setChecked(True)
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["polygons"] == [RING_A]
    assert inp["kwargs"]["input_sources"]["polygons"] == f"{MAIN_SRC}，1 塊"
    assert dlg.lbl_polygon_source.text() == f"{MAIN_SRC}，1 塊：3 個角點"


def test_mainwindow_polygons_take_precedence_and_are_read_at_collect_time(qt_app, no_rtklib, tmp_path):
    fm = FakeMap(corners=[(24.0, 120.0), (24.0, 120.01), (24.01, 120.01)])     # 地圖上另有角點：不用
    win, provider = _mainwindow_areas([list(RING_A)], [])
    dlg = filled_dialog(tmp_path, map_widget=fm)
    dlg.polygons_source = provider                                    # 傳入可呼叫物件 ⇒ 每次收集時取最新值
    dlg.rb_exposure_start.setChecked(True)
    dlg.chk_use_polygon.setChecked(True)
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["polygons"] == [RING_A]
    win.survey_areas.append(list(RING_B))                             # 對話框開著時又完成一塊、還在畫第三塊
    win.corners = [(25.14, 121.2), (25.14, 121.21), (25.15, 121.21)]
    dlg.show()                                                        # 重新顯示 ⇒ 來源標籤跟著更新
    assert dlg.lbl_polygon_source.text() == f"{MAIN_SRC}，3 塊：10 個角點"
    dlg.hide()
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["polygons"] == [RING_A, RING_B, win.corners]
    assert inp["kwargs"]["input_sources"]["polygons"] == f"{MAIN_SRC}，3 塊"
    win.survey_areas.clear()                                          # 全清 ⇒ 不退回地圖角點，明確拒絕
    win.corners = []
    inp, errors = dlg.collect_inputs()
    assert inp is None and errors == ["主視窗沒有任何作業區（已完成的區、或正在畫且 ≥3 角點的區），無法以測區計算視數"]
    dlg.chk_use_polygon.setChecked(False)                             # 不勾 ⇒ 不送測區
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["polygons"] is None and "polygons" not in inp["kwargs"]["input_sources"]


def test_bad_polygon_provider_values_are_refused(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    dlg.rb_exposure_start.setChecked(True)
    dlg.chk_use_polygon.setChecked(True)
    for value, msg in (([RING_A, RING_A[:2]], "第 2 塊作業區只有 2 個角點（至少 3 個）"),
                       ([[("x", 1), (2, 3), (4, 5)]], "第 1 塊作業區的角點不是（緯度, 經度）數字")):
        dlg.polygons_source = value
        assert dlg.collect_inputs() == (None, [msg])

    def boom():
        raise RuntimeError("壞了")
    dlg.polygons_source = boom
    assert dlg.collect_inputs() == (None, ["讀主視窗作業區失敗：RuntimeError: 壞了"])


# ─────────────────────────────────────────────────────────────────────────────
# R2：GPS 接收延遲（FC-only 的 GPST／EXIF 時間）
# ─────────────────────────────────────────────────────────────────────────────

def test_gps_lag_blank_is_zero_and_shown_as_effective(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    dlg.rb_exposure_start.setChecked(True)
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["gps_lag_s"] == 0.0
    assert inp["kwargs"]["input_sources"]["gps_lag_s"] == "空白＝0（不補償接收機延遲）"
    assert dlg.lbl_gps_lag.text() == "有效值：0 ms（空白＝0（不補償接收機延遲））"
    dlg.ed_gps_lag.setText("80")
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["gps_lag_s"] == pytest.approx(0.08)
    assert inp["kwargs"]["input_sources"]["gps_lag_s"] == "操作員輸入"
    assert dlg.lbl_gps_lag.text() == "有效值：80 ms（操作員輸入）"
    for text, msg in (("abc", "GPS 接收延遲不是數字：'abc'"), ("nan", "GPS 接收延遲不是有限數值：'nan'"),
                      ("-5", "GPS 接收延遲 -5 ms 超出合理範圍 0–1000 ms"),
                      ("1500", "GPS 接收延遲 1500 ms 超出合理範圍 0–1000 ms")):
        dlg.ed_gps_lag.setText(text)
        assert dlg.collect_inputs() == (None, [msg])
        assert dlg.lbl_gps_lag.text() == f"✘ {msg}"


def _two_rows(tmp_path):
    dlg = SurveyProcessDialog()
    for uav in ("HERO1", "HERO2"):
        d = tmp_path / uav
        d.mkdir()
        b, s, _ = make_session(d, uav=uav)
        dlg.add_sortie_row(bin_path=str(b), session_dir=str(s))
    dlg.ed_out_dir.setText(str(tmp_path / "out"))
    dlg.rb_exposure_start.setChecked(True)
    return dlg


def test_gps_lag_prefill_from_log_parameter_is_labelled(qt_app, no_rtklib, tmp_path):
    dlg = _two_rows(tmp_path)
    logs = {}
    dlg.param_reader = lambda p: logs[Path(p).parent.name]
    logs.update(HERO1=({"GPS_DELAY_MS": 120.0}, set()), HERO2=({"GPS_DELAY_MS": 120.0}, set()))
    assert dlg.fill_gps_lag_from_logs() == 120.0
    src = "取自飛控參數 GPS_DELAY_MS（HERO1、HERO2 皆 120 ms；EKF 的延遲模型值，不是實測）"
    assert dlg.ed_gps_lag.text() == "120" and dlg.lbl_gps_lag.text() == f"有效值：120 ms（{src}）"
    inp, errors = dlg.collect_inputs()
    assert errors == [] and inp["kwargs"]["gps_lag_s"] == pytest.approx(0.12)
    assert inp["kwargs"]["input_sources"]["gps_lag_s"] == src
    dlg.ed_gps_lag.textEdited.emit("100")                             # 操作員改過 ⇒ 來源變操作員輸入
    dlg.ed_gps_lag.setText("100")
    assert dlg.collect_inputs()[0]["kwargs"]["input_sources"]["gps_lag_s"] == "操作員輸入"
    # 不帶入的情況：欄位不動、寫明原因
    dlg.ed_gps_lag.setText("")
    for h1, h2, msg in (
            (({"GPS_DELAY_MS": 120.0}, set()), ({"GPS_DELAY_MS": 200.0}, set()),
             "各架不同（HERO1 120 ms、HERO2 200 ms）：流水線只吃單一值，不帶入"),
            (({"GPS_DELAY_MS": 0.0}, set()), ({"GPS_DELAY_MS": 0.0}, set()),
             "GPS_DELAY_MS = 0：飛控用 GPS 驅動依接收機型號的預設延遲，數值不在日誌裡 ⇒ 不帶入，請自行填"),
            (({}, set()), ({"GPS_DELAY_MS": 120.0}, set()), "HERO1：日誌沒有 GPS_DELAY_MS／GPS1_DELAY_MS"),
            (({"GPS_DELAY_MS": 120.0}, {"GPS_DELAY_MS"}), ({"GPS_DELAY_MS": 120.0}, set()),
             "HERO1：GPS_DELAY_MS 在飛行中改過")):
        logs.update(HERO1=h1, HERO2=h2)
        assert dlg.fill_gps_lag_from_logs() is None
        assert dlg.ed_gps_lag.text() == "" and dlg.lbl_gps_lag_log.text() == f"不帶入：{msg}"
    # 4.6 起改名 GPS1_DELAY_MS
    logs.update(HERO1=({"GPS1_DELAY_MS": 80.0}, set()), HERO2=({"GPS1_DELAY_MS": 80.0}, set()))
    assert dlg.fill_gps_lag_from_logs() == 80.0 and "GPS1_DELAY_MS" in dlg.lbl_gps_lag.text()


def test_gps_lag_prefill_reads_a_real_dataflash_log(qt_app, no_rtklib, tmp_path):
    from tests.test_survey_synthlog import DFWriter
    from ui.dialogs.survey_process_dialog import read_log_params
    w = DFWriter()
    w.define("PARM", "QNff", ["TimeUS", "Name", "Value", "Default"])
    w.write("PARM", TimeUS=1000, Name="GPS1_DELAY_MS", Value=110.0, Default=0.0)
    w.write("PARM", TimeUS=2000, Name="CAM1_TYPE", Value=2.0, Default=0.0)
    w.write("PARM", TimeUS=9_000_000, Name="CAM1_TYPE", Value=1.0, Default=0.0)
    p = w.save(tmp_path / "p.BIN")
    params, changed = read_log_params(p)
    assert params["GPS1_DELAY_MS"] == 110.0 and changed == {"CAM1_TYPE"}
    dlg = _two_rows(tmp_path)
    for r in range(2):
        dlg._set_cell(r, 1, str(p))
    assert dlg.fill_gps_lag_from_logs() == 110.0 and dlg.ed_gps_lag.text() == "110"
    dlg._set_cell(1, 1, str(tmp_path / "HERO2" / "fc.BIN"))           # 2 位元組的假檔 ⇒ 沒有參數
    assert dlg.fill_gps_lag_from_logs() is None
    assert dlg.lbl_gps_lag_log.text() == "不帶入：HERO2：日誌沒有 GPS_DELAY_MS／GPS1_DELAY_MS"
    dlg._set_cell(1, 1, str(tmp_path / "nope.BIN"))                   # 不存在 ⇒ 讀檔例外照實寫出
    assert dlg.fill_gps_lag_from_logs() is None
    assert dlg.lbl_gps_lag_log.text().startswith("不帶入：HERO2：讀不到日誌參數（")
    assert dlg.ed_gps_lag.text() == "110"                             # 不帶入時欄位不動


def test_ppk_frame_name_and_base_inputs_required(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    dlg.rb_exposure_start.setChecked(True)
    dlg.grp_ppk.setChecked(True)
    dlg.rb_ppk_pos.setChecked(True)
    _, errors = dlg.collect_inputs()
    assert "PPK：基站座標框架名稱必填（例如 e-GNSS[2025]）" in errors
    assert "PPK：HERO1 沒有填 .pos（或改選 RTKLIB 解算）" in errors
    dlg._set_cell(0, 4, str(tmp_path / "rover.pos"))
    dlg.ed_base_frame.setText("e-GNSS[2025]")
    inp, errors = dlg.collect_inputs()
    assert errors == [] and isinstance(inp["kwargs"]["ppk"], PpkSettings)
    assert inp["kwargs"]["ppk"].frame == "e-GNSS[2025]" and inp["kwargs"]["ppk"].run_rtklib is False
    # RTKLIB 模式：基站座標、天線高、觀測檔都必填；沒裝 RTKLIB 也擋下
    dlg.rb_ppk_rtklib.setChecked(True)
    _, errors = dlg.collect_inputs()
    for e in ("PPK：基站緯度必填", "PPK：基站經度必填", "PPK：基站橢球高必填", "PPK：天線高必填",
              "PPK：基站觀測檔要給 RINEX 或 UBX 其中一個"):
        assert e in errors
    assert any(e.startswith("未安裝 RTKLIB") for e in errors)
    dlg.ed_base_frame.setText("")
    _, errors = dlg.collect_inputs()
    assert "PPK：基站座標框架名稱必填（例如 e-GNSS[2025]）" in errors


def test_ppk_rtklib_mode_builds_base_station(qt_app, monkeypatch, tmp_path):
    monkeypatch.setattr(rtklib, "find_executable", lambda name, explicit=None: f"C:/rtk/{name}.exe")
    dlg = filled_dialog(tmp_path)
    dlg.rb_exposure_start.setChecked(True)
    dlg.grp_ppk.setChecked(True)
    obs = tmp_path / "base.obs"
    obs.write_text("x", encoding="utf-8")
    for ed, v in ((dlg.ed_base_lat, "25.0995"), (dlg.ed_base_lon, "121.1995"), (dlg.ed_base_h, "52.3"),
                  (dlg.ed_base_ant, "1.234"), (dlg.ed_base_frame, "e-GNSS[2025]"), (dlg.ed_base_rinex, str(obs))):
        ed.setText(v)
    dlg.ed_geo_projection.setText("EPSG:3826")
    _, errors = dlg.collect_inputs()
    assert any("不是經緯度座標系" in e for e in errors)
    dlg.ed_geo_projection.setText("+proj=longlat +ellps=GRS80 +no_defs")
    inp, errors = dlg.collect_inputs()
    assert errors == []
    p = inp["kwargs"]["ppk"]
    assert p.run_rtklib
    assert p.base == rtklib.BaseStation(25.0995, 121.1995, 52.3, "e-GNSS[2025]", antenna_height_m=1.234)
    assert p.base_obs == str(obs) and p.base_ubx is None
    assert inp["kwargs"]["geo_projection"] == "+proj=longlat +ellps=GRS80 +no_defs"


# ─────────────────────────────────────────────────────────────────────────────
# 背景 worker、結果、地圖、ODM
# ─────────────────────────────────────────────────────────────────────────────

def test_worker_runs_fake_pipeline_off_the_gui_thread_and_fills_results(qt_app, no_rtklib, tmp_path):
    fm = FakeMap()
    dlg = filled_dialog(tmp_path, map_widget=fm)
    dlg.rb_exposure_start.setChecked(True)
    got = {}
    res = fake_result(tmp_path)

    def fake(sorties, out_dir, *, progress, should_cancel, **kw):
        got.update(sorties=sorties, out_dir=out_dir, kw=kw, thread=threading.current_thread())
        progress("第一步", 0.5)
        return res

    dlg.process_fn = fake
    assert dlg.start_processing() is True
    assert dlg.is_busy() and not dlg.btn_run.isEnabled() and dlg.btn_cancel.isEnabled()
    wait_worker(dlg)
    assert got["thread"] is not threading.main_thread()
    assert got["kw"]["sensor_ts_semantics"] == "exposure_start" and got["out_dir"] == str(tmp_path / "out")
    assert (got["kw"]["gps_lag_s"], got["kw"]["polygons"]) == (0.0, None)
    assert got["kw"]["input_sources"] == {"gps_lag_s": "空白＝0（不補償接收機延遲）"}
    assert dlg.result is res and "第一步" in dlg.log_view.toPlainText()
    assert dlg.qa_table.rowCount() == 1 and dlg.qa_table.item(0, 2).text() == "97.0%"
    assert dlg.lbl_summary.text().startswith("QA 通過")
    assert fm.calls == [("load_photo_poses_file", PHOTO_POSE_LAYER, str(res.paths["photos_csv"]))]
    assert dlg.spn_res.value() == pytest.approx(2.7)                 # 正射解析度預設 ≈ GSD
    assert dlg.btn_odm.isEnabled() and not dlg.is_busy()


def test_summary_shows_effective_gps_lag_and_survey_areas(qt_app, no_rtklib, tmp_path):
    dlg = SurveyProcessDialog()
    res = fake_result(tmp_path)
    res.effective.update(gps_lag_s=0.12, polygons=[[[25.1, 121.2]] * 3] * 2,
                         input_sources={"gps_lag_s": "取自飛控參數 GPS_DELAY_MS（…）"})
    dlg._on_finished(res)
    lines = dlg.lbl_summary.text().splitlines()
    assert lines[0].startswith("QA 通過")
    assert "GPS 接收延遲補償 120 ms（取自飛控參數 GPS_DELAY_MS（…））" in lines
    assert "視數測區：2 塊" in lines
    dlg._on_finished(fake_result(tmp_path))                           # 舊結果沒有這兩個鍵 ⇒ 不顯示、不出錯
    assert not any(ln.startswith(("GPS 接收延遲補償", "視數測區")) for ln in dlg.lbl_summary.text().splitlines())


def test_dialog_runs_the_real_pipeline_with_committed_areas_and_gps_lag(qt_app, no_rtklib, tmp_path):
    """R2 stage C 的兩個發現，跑真的 survey.pipeline：兩塊已完成的區（地圖角點空白）＋ GPS 延遲 80 ms
    ⇒ 與直接呼叫 process_campaign(polygons=…, gps_lag_s=0.08) 的 photos.csv 位元組相同，GPST 無偏差。"""
    import numpy as np
    from survey import pipeline
    from tests.test_survey_pipeline import EAST, GROUND_ELL, WEST, build_sortie, truth_err
    s = build_sortie(tmp_path / "h1", uav="HERO1", sid="S1", seed=1)
    win, provider = _mainwindow_areas([list(WEST), list(EAST)], [])
    dlg = SurveyProcessDialog(map_widget=FakeMap(corners=[]), polygons=provider)
    dlg.add_sortie_row("", str(s["bin"]), str(s["sess"]))
    dlg.rb_exposure_start.setChecked(True)
    dlg.ed_ground_h.setText(str(GROUND_ELL))
    dlg.chk_use_polygon.setChecked(True)
    dlg.ed_gps_lag.setText("80")
    dlg.ed_out_dir.setText(str(tmp_path / "gui"))
    assert dlg.start_processing() is True
    wait_worker(dlg)
    res = dlg.result
    assert res is not None and res.passed, dlg.lbl_summary.text()
    direct = pipeline.process_campaign([pipeline.SortieInput("HERO1", s["bin"], s["sess"])], tmp_path / "direct",
                                       sensor_ts_semantics="exposure_start", ground_h_m=GROUND_ELL,
                                       polygons=[WEST, EAST], gps_lag_s=0.08)
    assert (tmp_path / "gui" / "photos.csv").read_bytes() == (tmp_path / "direct" / "photos.csv").read_bytes()
    assert res.effective["gps_lag_s"] == 0.08 and res.qa["exposure"]["n_polygons"] == 2
    assert res.qa["exposure"]["frac_ge"] == direct.qa["exposure"]["frac_ge"]
    assert res.effective["input_sources"] == {"gps_lag_s": "操作員輸入", "polygons": f"{MAIN_SRC}，2 塊"}
    assert abs(float(np.median([truth_err(s, r)[2] for r in res.rows]))) < 2e-3      # 0 ms 時是 −0.080 s
    assert "GPS 接收延遲補償 80 ms（操作員輸入）" in dlg.lbl_summary.text().splitlines()


def test_worker_failure_and_cancel(qt_app, no_rtklib, tmp_path):
    dlg = filled_dialog(tmp_path)
    dlg.rb_exposure_start.setChecked(True)

    def boom(*a, **k):
        raise PipelineError("HERO1：會期的 uav 與呼號不符")
    dlg.process_fn = boom
    dlg.start_processing()
    wait_worker(dlg)
    assert dlg.result is None and "處理失敗：HERO1：會期的 uav 與呼號不符" in dlg.lbl_summary.text()

    gate = threading.Event()

    def slow(sorties, out_dir, *, progress, should_cancel, **kw):
        assert gate.wait(5)
        if should_cancel():
            raise PipelineCanceled("使用者取消")
        return fake_result(tmp_path)
    dlg.process_fn = slow
    dlg.start_processing()
    dlg.reject()                                                      # 處理中不能關：改成要求取消
    assert dlg._worker is not None and dlg._worker.is_canceled()
    assert "處理中" in dlg.lbl_errors.text()
    gate.set()
    wait_worker(dlg)
    assert dlg.result is None and dlg.lbl_summary.text().startswith("已取消")


def test_worker_class_signals_directly(qt_app):
    w = SurveyProcessWorker([], "out", {"sensor_ts_semantics": "x"},
                            process_fn=lambda s, o, *, progress, should_cancel, **kw: (progress("p", None), "ok")[1])
    seen = {"p": [], "ok": [], "fail": []}
    w.progress.connect(lambda m, f: seen["p"].append((m, f)))
    w.finished_ok.connect(seen["ok"].append)
    w.failed.connect(seen["fail"].append)
    w.run()                                                           # 同步呼叫 run()：測執行緒裡那段邏輯
    assert seen == {"p": [("p", -1.0)], "ok": ["ok"], "fail": []}
    w2 = SurveyProcessWorker([], "out", {}, process_fn=lambda *a, **k: 1 / 0)
    w2.failed.connect(seen["fail"].append)
    w2.run()
    assert seen["fail"] == ["ZeroDivisionError: division by zero"]


def test_odm_button_gating_and_override_is_recorded(qt_app, no_rtklib, tmp_path):
    fm = FakeMap()
    dlg = SurveyProcessDialog(map_widget=fm)
    dlg.result = fake_result(tmp_path, passed=False, gsd=None)
    dlg._update_buttons()
    assert not dlg.btn_odm.isEnabled() and dlg.odm_block_reason().startswith("QA 未通過")
    assert dlg.submit_odm() is False
    dlg.chk_qa_override.setChecked(True)
    assert not dlg.btn_odm.isEnabled() and dlg.odm_block_reason() == "正射解析度未設定"
    dlg.spn_res.setValue(3.0)
    assert dlg.btn_odm.isEnabled()
    dlg.chk_qa_override.setChecked(False)
    assert not dlg.btn_odm.isEnabled()
    dlg.chk_qa_override.setChecked(True)
    tiles = tmp_path / "result" / "orthophoto_tiles"
    jobs = []

    def fake_odm(job, *, client, progress, should_cancel, poll_s):
        jobs.append(job)
        progress("上傳", 0.0)
        return {"uuid": "t-1", "zip": "z", "extract_dir": str(tiles.parent), "ortho_tiles": tiles}

    dlg.odm_run_fn = fake_odm
    assert dlg.submit_odm() is True
    wait_worker(dlg, "_odm_worker")
    job = jobs[0]
    assert job["qa_override"] is True and job["qa_passed"] is False
    assert job["qa_reasons"] == ["HERO1：第 1 帶張數不守恆（假的）"]
    assert job["orthophoto_resolution_source"] == "user" and job["node_url"] == "http://127.0.0.1:3000"
    assert {"name": "orthophoto-resolution", "value": 3.0} in job["options"]
    assert {"name": "rolling-shutter-readout", "value": 20.0} not in job["options"]   # 沒有會期 ⇒ 沒勾
    assert dlg.result.effective["odm_submission"]["qa_override"] is True
    assert fm.calls == [("add_ortho_layer", ORTHO_LAYER, str(tiles), "tms")]
    assert dlg.odm_result["uuid"] == "t-1"


def test_odm_disabled_without_geo_txt_and_auto_submit_only_when_passed(qt_app, no_rtklib, tmp_path):
    dlg = SurveyProcessDialog()
    dlg.result = fake_result(tmp_path, images=False)
    dlg.spn_res.setValue(2.0)
    dlg.chk_qa_override.setChecked(True)
    assert dlg.odm_block_reason() == "沒有 geo.txt 或加標籤影像（見處理錯誤）"
    calls = []
    dlg.odm_run_fn = lambda job, **k: (calls.append(job), {"ortho_tiles": None})[1]
    dlg.chk_auto_submit.setChecked(True)
    dlg.chk_qa_override.setChecked(False)
    dlg._on_finished(fake_result(tmp_path, passed=False))           # QA 未過 ⇒ 不自動送
    assert dlg._odm_worker is None and calls == []
    dlg._on_finished(fake_result(tmp_path, passed=True))
    wait_worker(dlg, "_odm_worker")
    assert len(calls) == 1 and calls[0]["qa_override"] is False
    assert "沒有 orthophoto_tiles" in dlg.log_view.toPlainText()


def test_odm_worker_class(qt_app):
    seen = {"ok": [], "cancel": 0}
    w = OdmSubmitWorker({"x": 1}, run_fn=lambda job, **k: {"uuid": "u", **job})
    w.finished_ok.connect(seen["ok"].append)
    w.run()
    assert seen["ok"] == [{"uuid": "u", "x": 1}]

    def canceled(job, **k):
        raise PipelineCanceled("使用者取消")
    w2 = OdmSubmitWorker({}, run_fn=canceled)
    w2.canceled.connect(lambda: seen.__setitem__("cancel", seen["cancel"] + 1))
    w2.run()
    assert seen["cancel"] == 1


# ─────────────────────────────────────────────────────────────────────────────
# DualMapWidget：航測圖層要同時到 2D 與 3D
# ─────────────────────────────────────────────────────────────────────────────

def _dual():
    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = _bare2d(), _bare3d()
    return d


SURVEY_METHODS = ("add_ortho_layer", "remove_ortho_layer", "set_photo_poses", "load_photo_poses_file",
                  "clear_photo_poses", "clear_survey_layers")


def test_survey_methods_are_explicit_not_getattr_fallback():
    """__getattr__ 只代理 2D：沒有明確方法時 add_ortho_layer 會 AttributeError、位姿只畫 2D。"""
    for name in SURVEY_METHODS:
        assert name in DualMapWidget.__dict__, name


def test_add_and_remove_ortho_reach_both_maps(qt_app, tmp_path):
    a, b = tmp_path / "a" / "orthophoto_tiles", tmp_path / "b" / "orthophoto_tiles"
    _make_tiles(a, tms=True, xml=True)
    _make_tiles(b, tms=False, xml=False)
    d = _dual()
    eff = DualMapWidget.add_ortho_layer(d, "o1", a, scheme="tms", alpha=0.6)
    lid, url, opts = _call(d.map_3d._sent[-1], "addOrthoLayer")
    assert (lid, opts["scheme"], opts["alpha"]) == ("o1", "tms", 0.6)
    assert eff["3d"]["bounds_source"] == "tilemapresource.xml" and eff["3d"]["min_level"] == 10
    assert eff["2d"]["tms"] is True and eff["2d"]["alpha"] is None and eff["2d"]["max_level"] == 16
    assert [t["tms"] for t in d.map_2d._ortho_tiles] == [True]
    DualMapWidget.add_ortho_layer(d, "o2", b, scheme="xyz")
    assert [t["tms"] for t in d.map_2d._ortho_tiles] == [True, False]
    assert DualMapWidget.remove_ortho_layer(d, "o1") is True
    assert _call(d.map_3d._sent[-1], "removeOrthoLayer") == ["o1"]
    assert [(t["tms"], t["url"]) for t in d.map_2d._ortho_tiles] == [(False, eff2_url(b))]
    assert set(d.map_3d._ortho_layers) == {"o2"}
    assert DualMapWidget.remove_ortho_layer(d, "nope") is False
    DualMapWidget.clear_survey_layers(d)
    assert d.map_2d._ortho_tiles == [] and d.map_3d._ortho_layers == {} and d.map_3d._sent[-1] == "clearSurveyLayers()"
    assert d._survey_ortho == {}


def eff2_url(p):
    from ui.widgets.cesium_map_widget import local_path_to_file_url
    return local_path_to_file_url(p)


def _rows(n):
    return [{"image_id": f"H_S_{i:06d}", "lat_deg": 25.1 + i * 1e-6, "lon_deg": 121.2, "h_ell_m": 150.0,
             "yaw_deg": 95.0, "cog_deg": 90.0, "source": "ppk"} for i in range(n)]


def test_set_photo_poses_draws_on_both_maps(qt_app, tmp_path):
    d = _dual()
    eff = DualMapWidget.set_photo_poses(d, "p", _rows(3))
    lid, payload, opts = _call(d.map_3d._sent[-1], "setPhotoPoses")
    assert lid == "p" and len(payload["rows"]) == 3 and payload["rows"][0][3] == 95.0
    assert eff["3d"]["drawn"] == 3 and eff["2d"]["drawn"] == 3 and len(d.map_2d._photo_pose_layers["p"]["rows"]) == 3
    big = _rows(5001)
    n_sent = len(d.map_3d._sent)
    with pytest.raises(ValueError, match="photos.csv"):
        DualMapWidget.set_photo_poses(d, "big", big)
    assert len(d.map_3d._sent) == n_sent and "big" not in d.map_2d._photo_pose_layers
    csv_p = tmp_path / "photos.csv"
    csv_p.write_text("lat_deg,lon_deg\n", encoding="utf-8")
    eff = DualMapWidget.set_photo_poses(d, "big", big, path=csv_p)
    assert d.map_3d._sent[-1].startswith("loadPhotoPosesUrl(") and eff["2d"] == {"rows": 5001, "drawn": 0,
                                                                                 "over_2d_cap": True}
    assert DualMapWidget.clear_photo_poses(d, "p") is True
    assert d.map_3d._sent[-1] == 'clearPhotoPoses("p")' and "p" not in d.map_2d._photo_pose_layers


def test_load_photo_poses_file_reads_f2_for_2d_and_lets_3d_read_the_file(qt_app, tmp_path):
    rows = []
    for i in range(1, 4):
        r = {c: None for c in deliverable.F2_COLUMNS}
        r.update(image_id=f"HERO1_S1_{i:06d}", file=f"images/{i:06d}.jpg", uav="HERO1", strip=1, seq=i,
                 t_gpst_s=1.444e9 + i, lat_deg=25.1 + i * 1e-4, lon_deg=121.2, h_ell_m=150.0, yaw_deg=95.0 + i,
                 cog_deg=90.0, source="fc_interp", crs_h="EPSG:4326", crs_v="ellipsoidal", flags="")
        rows.append(r)
    p = deliverable.write_photos_csv(tmp_path / "photos.csv", rows)
    d = _dual()
    eff = DualMapWidget.load_photo_poses_file(d, "survey", p)
    lid, url, opts = _call(d.map_3d._sent[-1], "loadPhotoPosesUrl")
    assert lid == "survey" and url == eff2_url(p) and eff["3d"]["format"] == "csv"
    drawn = d.map_2d._photo_pose_layers["survey"]["rows"]
    assert eff["2d"]["skipped"] == 0 and [x[3] for x in drawn] == [96.0, 97.0, 98.0]     # 航向 = yaw_deg
