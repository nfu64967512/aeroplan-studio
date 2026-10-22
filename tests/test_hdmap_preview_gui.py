"""高精地圖「預計成果」GUI（C 部分）：偵察頁按鈕／勾選、預覽對話框、背景 worker、MainWindow 生命週期。

不建 MainWindow（QtWebEngine 在此環境建不起來）：用假 self／裸實例跑**真的處理函式**（house style，見
tests/test_hdmap_gui_mainwindow.py、tests/test_clear_all_wipes_map.py）。

預覽物件一律是**真的** ``survey.preview.HdmapPreview``（``make_preview`` 用合成的契約欄位值建；ok=False 用
``HdmapPreview.failed``）。要控制時序／失敗的測試只換掉 ``survey.preview`` 的 ``compute_hdmap_preview`` /
``render_preview_map_png`` 兩個函式（monkeypatch，不寫檔）；``to_js_payload`` 是真的，只包一層計次。
真的規劃 → 真的 compute → 頁面上畫出來的端到端在 tests/test_hdmap_preview_e2e.py。

鎖住的事：
* 偵察頁：鈕只在 'ready' 可按；勾選預設打開；set_plan_result 一律回 'none'；信號真的發得出去；
* 對話框：非模態、標題與橫幅標明「規劃預測，非實際成果」、統計卡列出每個 stats 鍵（假設、文獻精度附條件、
  看得見的地物、基準）；匯出 PNG＋JSON（JSON 嚴格、不含影像位元組）；World Terrain 陰影拒絕落地；
* worker：ok=False／例外一律走 failed；放棄後不發信號；真的跨執行緒送回 GUI 執行緒；
* MainWindow：計算中→完成畫圖層（勾選關著不畫、之後勾才畫，payload 只算一次）、計畫換掉就丟、
  舊 worker 的排隊信號不採用、重設時還在跑的 worker 被寄放（不是直接丟參考）、各生命週期掛勾真的清掉圖層。
"""
from __future__ import annotations

import ast
import dataclasses
import io
import json
import math
import os
import threading
import time
import types
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine 再建 QApplication
    import ui.main_window as mw
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from ui.workers.hdmap_preview_worker import HdmapPreviewWorker
    from ui.dialogs import hdmap_preview_dialog as hpd
    from PyQt6 import QtWidgets as _QtW
    from PyQt6.QtWidgets import QApplication, QWidget
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

from mission.swarm_coordinator import MAPPING_EXPOSURE_SEAM_CAVEAT  # noqa: E402
import survey.preview as sp  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SRC = (ROOT / "ui" / "main_window.py").read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


# ═══════════════════════════ 合成資料 ═══════════════════════════
def _png(w: int = 40, h: int = 30, color=(229, 57, 53, 255)) -> bytes:
    from PIL import Image
    buf = io.BytesIO()
    Image.new("RGBA", (w, h), color).save(buf, "PNG")
    return buf.getvalue()


ASSUMPTIONS = ["規劃風 5 m/s 來自 270°（面板）—— 不是現場實測風", MAPPING_EXPOSURE_SEAM_CAVEAT,
               "CAM_MAX_ROLL 未建模：轉彎改出時的觸發可能被延後"]
ACCURACY = [
    {"case": "相對精度", "horizontal": "1–3×GSD", "vertical": "1–3×GSD", "condition": "同一架次內部一致性"},
    {"case": "PPK＋1 個控制點＋交叉航帶", "horizontal": "2–5 cm", "vertical": "2–10 cm",
     "condition": "文獻值；我們的硬體未驗證"},
]
ACC_NOTE = "我們自己的精度目前未知，要等實飛加獨立檢核點才能宣告。"


def _stats(**over):
    s = {
        "uav_labels": ["A1", "B1"], "drone_ids": [1, 2],
        "camera": {"preset": "RPi Camera Module 3（IMX708）4.74 mm", "focal_mm": 4.74, "pixel_um": 1.4,
                   "width_px": 4608, "height_px": 2592},
        "agl_m": {"min": 96.0, "median": 100.0, "max": 104.0},
        "gsd_cm": {"min": 2.84, "median": 2.95, "max": 3.07},
        "ortho_resolution_cm": 2.95, "dsm_resolution_cm_est": 5.9,
        "photos_per_uav": {"A1": 700, "B1": 534}, "photos_total": 1234,
        "data_gb_range": [9.1, 15.2], "odm_ram_tier": "32 GB（1,000–2,500 張）",
        "area_km2": {"survey": 0.95, "product": 0.812, "high_quality": 0.61},
        "frac_ge": {"interior": {"5": 0.9876, "9": 0.7}, "edge_band": {"5": 0.61, "9": 0.2}},
        "forward_overlap_p05": 0.78, "side_overlap_p05": 0.66, "seam_overlap_min": 0.31,
        "interior_hole_count": 2, "interior_hole_area_m2": 508.0, "edge_band_m": 37.5,
        "edge_fringe_note": "測區邊緣半刈幅內視數自然遞減", "cell_size_m": 2.0, "k_product": 5,
        "assumptions": list(ASSUMPTIONS), "accuracy_literature": [dict(a) for a in ACCURACY],
        "accuracy_note": ACC_NOTE,
        "visible_features": [{"feature": "車道線", "size_cm": 15, "pixels": 5.08},
                             {"feature": "人孔蓋", "size_cm": 60, "pixels": 20.34}],
        "datum": "正高（MSL）", "elapsed_s": 3.2,
    }
    s.update(over)
    return s


def _legend():
    return {"title": "預計成果（規劃預測）",
            "classes": [{"label": "只有 1 張（不會成圖）", "color": "#e53935"},
                        {"label": "≥9 張（高品質）", "color": "#43a047"}],
            "notes": ["顏色＝每格預計被幾張照片拍到"]}


RING = [[25.051, 121.201], [25.051, 121.209], [25.059, 121.209], [25.059, 121.201]]


#: 契約（固定）列出的 HdmapPreview 欄位 —— 真的 dataclass 要剛好是這些
CONTRACT_FIELDS = ("ok", "reason", "bounds_wsen", "image_png", "image_px", "cell_size_m", "extent_rings",
                   "extent_holes", "interior_holes", "edge_short", "survey_rings", "photos", "stats", "legend",
                   "hillshade_png", "hillshade_bounds_wsen")


def make_preview(**over) -> "sp.HdmapPreview":
    """真的 survey.preview.HdmapPreview，欄位值是合成的（照契約）。"""
    kw = dict(
        ok=True, reason=None, bounds_wsen=(121.20, 25.05, 121.21, 25.06), image_png=_png(40, 30), image_px=(40, 30),
        cell_size_m=2.0, extent_rings=[list(RING)], extent_holes=[],
        interior_holes=[
            {"ring": [[25.055, 121.205], [25.055, 121.2052], [25.0552, 121.2052]], "area_m2": 412.0, "min_views": 3},
            {"ring": [[25.056, 121.206], [25.056, 121.2061], [25.0561, 121.2061]], "area_m2": 96.0, "min_views": 1}],
        edge_short=[
            {"ring": [[25.0511, 121.2011], [25.0511, 121.2013], [25.0513, 121.2013]], "area_m2": 30.0, "min_views": 2}],
        survey_rings=[list(RING)], photos=[[25.052, 121.202, 0], [25.053, 121.203, 1]],
        stats=_stats(), legend=_legend(), hillshade_png=None, hillshade_bounds_wsen=None)
    kw.update(over)
    return sp.HdmapPreview(**kw)


def failed_preview(reason: str) -> "sp.HdmapPreview":
    return sp.HdmapPreview.failed(reason)


def _hv(result=None):
    return types.SimpleNamespace(last_result=result if result is not None else {"plan": 1},
                                 mapping_mode=types.SimpleNamespace(lead_in_m=40.0))


# ═══════════════════════════ survey.preview：真模組，只換 compute／render 兩個函式 ═══════════════════════════
@pytest.fixture
def preview_mod(monkeypatch):
    """回傳 SimpleNamespace(mod, calls, state)：state['compute'] / state['render'] 是實際行為。"""
    calls = {"compute": [], "render": []}
    state = {"compute": lambda hv, sw, **kw: make_preview(), "render": lambda pv, **kw: _png(64, 48)}

    def compute(hv, swarm, **kw):
        calls["compute"].append((hv, swarm, kw))
        return state["compute"](hv, swarm, **kw)

    def render(preview, **kw):
        calls["render"].append((preview, kw))
        return state["render"](preview, **kw)
    monkeypatch.setattr(sp, "compute_hdmap_preview", compute)
    monkeypatch.setattr(sp, "render_preview_map_png", render)
    return types.SimpleNamespace(mod=sp, calls=calls, state=state)


@pytest.fixture
def payload_calls(monkeypatch):
    """真的 HdmapPreview.to_js_payload 包一層計次：回傳呼叫過的 preview 串列。"""
    calls = []
    real = sp.HdmapPreview.to_js_payload

    def spy(self, **kw):
        calls.append(self)
        return real(self, **kw)
    monkeypatch.setattr(sp.HdmapPreview, "to_js_payload", spy)
    return calls


def _pump(cond, timeout: float = 10.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    QApplication.processEvents()
    return bool(cond())


# ═══════════════════════════ 假物件 ═══════════════════════════
class RecMap:
    """記錄預覽圖層 API 的假地圖（DualMapWidget 的回傳形狀）。"""

    def __init__(self, result=None, log=None):
        self.calls = []
        self.result = result if result is not None else {"2d": {"ok": True}, "3d": {"ok": True}}
        self.log = log

    def set_hdmap_preview(self, layer_id, payload):
        self.calls.append(("set", layer_id, payload))
        return self.result

    def clear_hdmap_preview(self, layer_id=None):
        self.calls.append(("clear", layer_id))
        return True

    def set_hdmap_preview_visible(self, layer_id, visible):
        self.calls.append(("visible", layer_id, bool(visible)))

    # on_clear_paths / on_clear_all_silent / _on_dccpp_coverage_requested 會碰的
    def clear_paths(self):
        self.calls.append(("clear_paths",))

    def clear_all_display_layers_js(self):
        self.calls.append(("clear_js",))
        if self.log is not None:
            self.log.append(("clear_js",))

    def names(self):
        return [c[0] for c in self.calls]


class FakeStatus:
    def __init__(self):
        self.msgs = []

    def showMessage(self, m, *a):
        self.msgs.append(m)

    def clearMessage(self):
        self.msgs.append("")

    def currentMessage(self):
        return self.msgs[-1] if self.msgs else ""


_BOUND = ("_start_hdmap_preview", "_hdmap_preview_signal_worker", "_on_hdmap_preview_done",
          "_on_hdmap_preview_failed", "_hdmap_preview_retire_worker", "_hdmap_preview_show_failure",
          "_hdmap_preview_draw", "_on_hdmap_preview_layer_toggled", "_on_hdmap_preview_open")


class Win(QWidget):
    """MainWindow 的假 self：QWidget（對話框要一個真的 parent），只綁預覽相關的真方法。"""

    def __init__(self, hv, panel, map_widget):
        super().__init__()
        self._hv_coordinator = hv
        self._dccpp_result = hv.last_result
        self._hv_swarm_coordinator = types.SimpleNamespace(name="swarm")
        self.vtol_recon_panel = panel
        self.map_widget = map_widget
        self._status = FakeStatus()
        for n in _BOUND:
            setattr(self, n, types.MethodType(getattr(MainWindow, n), self))

    def statusBar(self):
        return self._status


@pytest.fixture
def panel(qt_app):
    return VTOLReconPanel()


@pytest.fixture
def win(qt_app, panel):
    w = Win(_hv(), panel, RecMap())
    yield w
    pw = w.__dict__.get("_hdmap_preview_worker")
    if isinstance(pw, HdmapPreviewWorker) and pw.isRunning():
        pw.stop()
        pw.wait(10000)


@pytest.fixture
def qmb(monkeypatch):
    QMB = _QtW.QMessageBox
    calls = {"information": [], "warning": [], "critical": [], "question": []}

    def mk(name):
        def _f(*a, **k):
            calls[name].append((a, k))
            return QMB.StandardButton.Ok
        return staticmethod(_f)
    for name in calls:
        monkeypatch.setattr(QMB, name, mk(name))
    return calls


# ═══════════════════════════ 偵察頁 ═══════════════════════════
def test_panel_controls_sit_under_the_mapping_status_and_start_disabled(panel):
    btn, chk = panel.btn_hdmap_preview, panel.chk_hdmap_preview_layer
    assert btn.text() == "預覽高精地圖成果…"
    assert chk.text() == "地圖顯示預計成果" and chk.isChecked() is True     # 預設打開
    assert panel.hdmap_preview_state() == "none" and not btn.isEnabled()
    assert panel.hdmap_preview_layer_enabled() is True
    group = panel.hv_map_status_label.parentWidget()
    assert group.title() == "航測建圖"
    assert btn.parentWidget() is group and chk.parentWidget() is group
    lay = group.layout()
    i_status, i_pv, i_live = (lay.indexOf(panel.hv_map_status_label), lay.indexOf(panel.hdmap_preview_status_label),
                              lay.indexOf(panel.hv_map_live_label))
    assert -1 < i_status < i_pv < i_live
    # 按鈕列（子 layout）夾在狀態字與預覽狀態字之間
    row = [i for i in range(lay.count()) if lay.itemAt(i).layout() is not None
           and lay.itemAt(i).layout().indexOf(btn) >= 0]
    assert row and i_status < row[0] < i_pv


@pytest.mark.parametrize("state, enabled", [("none", False), ("computing", False), ("ready", True), ("failed", False)])
def test_panel_button_enabled_only_when_ready(panel, state, enabled):
    panel.set_hdmap_preview_state("ready")
    panel.set_hdmap_preview_state(state)
    assert panel.hdmap_preview_state() == state
    assert panel.btn_hdmap_preview.isEnabled() is enabled
    want = {"none": "", "computing": "預計成果計算中…（規劃預測）",
            "ready": "預計成果已算好（規劃預測，非實際成果）", "failed": "預計成果無法產生"}[state]
    assert panel.hdmap_preview_status_label.text() == want


def test_panel_rejects_unknown_state_and_status_text_reaches_tooltip(panel):
    with pytest.raises(ValueError):
        panel.set_hdmap_preview_state("done")
    assert panel.hdmap_preview_state() == "none"
    panel.set_hdmap_preview_state("failed")
    panel.set_hdmap_preview_status("預計成果無法產生：這份計畫沒有相機像素大小")
    assert panel.hdmap_preview_status_label.text() == "預計成果無法產生：這份計畫沒有相機像素大小"
    assert "沒有相機像素大小" in panel.btn_hdmap_preview.toolTip()
    assert "沒有相機像素大小" in panel.hdmap_preview_status_label.toolTip()


def test_panel_signals_fire(panel):
    got, toggles = [], []
    panel.hdmap_preview_requested.connect(lambda: got.append(1))
    panel.hdmap_preview_layer_toggled.connect(toggles.append)
    panel.btn_hdmap_preview.click()                     # 'none'：鈕是暗的 ⇒ 不發
    assert got == []
    panel.set_hdmap_preview_state("ready")
    panel.btn_hdmap_preview.click()
    assert got == [1]
    panel.chk_hdmap_preview_layer.setChecked(False)
    panel.chk_hdmap_preview_layer.setChecked(True)
    assert toggles == [False, True]


def test_set_plan_result_resets_preview_state(panel):
    panel.set_hdmap_preview_state("ready")
    panel.set_hdmap_preview_status("x")
    panel.set_plan_result([(1, 10, 1.0, 1.0)], "replanned")
    assert panel.hdmap_preview_state() == "none" and not panel.btn_hdmap_preview.isEnabled()
    assert panel.hdmap_preview_status_label.text() == ""
    panel.set_hdmap_preview_state("computing")
    panel.set_plan_result([], "規劃未產生結果")
    assert panel.hdmap_preview_state() == "none"


# ═══════════════════════════ 對話框（純函式） ═══════════════════════════
def test_stats_card_lists_every_contract_key(qt_app):
    text = _plain(hpd.preview_html(make_preview()))
    for s in ("規劃預測，非實際成果", "RPi Camera Module 3（IMX708）4.74 mm", "焦距 4.74 mm", "4608×2592 px",
              "最少 96 m／中位 100 m／最多 104 m", "最少 2.84 cm／中位 2.95 cm／最多 3.07 cm",
              "正射解析度（預估）", "2.95 cm", "5.9 cm", "約 2×GSD，ODM 預設值推估",
              "機 1（A1） 700 張、機 2（B1） 534 張", "1234 張", "9.1–15.2 GB", "假設每張 7.4–12.3 MB",
              "32 GB（1,000–2,500 張）", "測區 0.950 km²", "可成圖（視數 ≥5）0.812 km²", "高品質（≥9）0.610 km²",
              "內部（離邊界 > 37.5 m）", "視數 ≥5：98.76%", "≥9：70.00%", "邊緣帶（≤ 37.5 m）", "視數 ≥5：61.00%",
              "前向 0.78", "側向 0.66", "0.31 刈幅", "2 處、508 m²（視數 < 5；能不能上傳以上傳閘門為準）",
              "測區邊緣半刈幅內視數自然遞減",
              "2 m（成圖門檻 k = 5", "正高（MSL）", "3.2 s",
              "只有 1 張（不會成圖）", "≥9 張（高品質）", "顏色＝每格預計被幾張照片拍到",
              "車道線", "15 cm", "5.1 px", "人孔蓋", "20.3 px", "內部破洞（地圖紅框）2 處", "412 m²，最少 3 視",
              "邊緣帶不足 1 處"):
        assert s in text, s
    for a in ASSUMPTIONS:
        assert a in text
    for a in ACCURACY:
        for v in a.values():
            assert v in text, v
    assert ACC_NOTE in text


def test_stats_card_unknowns_show_dash_and_fallback_accuracy_sentence(qt_app):
    pv = make_preview(stats=_stats(gsd_cm={"min": None, "median": float("nan"), "max": None}, accuracy_note="",
                                  seam_overlap_min=None, data_gb_range=None, frac_ge={
                                      "interior": {5: 0.5, 9: 0.25}, "edge_band": None}))
    text = _plain(hpd.preview_html(pv))
    assert "最少 —／中位 —／最多 —" in text
    assert hpd.ACCURACY_UNKNOWN_FALLBACK in text            # 精度說明一定在
    assert "—（單機或區塊不相接）" in text
    assert "—–— GB（假設每張 7.4–12.3 MB" in text           # 範圍不知道也照樣寫出假設
    assert "視數 ≥5：50.00%" in text and "≥9：25.00%" in text   # 整數鍵也讀得到


def test_stats_card_shows_optional_keys_from_the_real_module(qt_app):
    """survey.preview 另外帶的鍵（全測區比例、規劃風、每張 MB、地形底圖、閘門格、不進度量的照片）要顯示；
    文獻表裡沒有值的格子顯示 —（不是空白）。"""
    st = _stats(frac_ge={"interior": {"5": 0.99, "9": 0.8}, "edge_band": {"5": 0.6, "9": 0.1},
                         "all": {"5": 0.93, "9": 0.71}},
                wind=[5.0, 270.0], wind_source="面板規劃風", data_mb_per_photo=[7.4, 12.3],
                terrain_context="規劃 DEM 山影（粗）", gate_cell_size_m=2.0, edge_short_count=1,
                edge_short_area_m2=30.0, photos_excluded={"agl_le_0": 3},
                dsm_resolution_note="估計值（約 2×GSD），實際以 ODM 輸出為準",
                accuracy_literature=[{"case": "絕對精度，只用飛控 GPS", "horizontal": "2–6 m", "vertical": None,
                                      "condition": "沒有地面控制點"}])
    st["area_km2"] = {"survey": 0.95, "product": 1.1, "high_quality": 0.9,
                      "product_in_survey": 0.94, "high_quality_in_survey": 0.85}
    text = _plain(hpd.preview_html(make_preview(stats=st)))
    for want in ("全測區", "視數 ≥5：93.00%", "≥9：71.00%", "5 m/s 來自 270°", "面板規劃風", "不是現場實測風",
                 "規劃 DEM 山影（粗）", "閘門格 2 m", "邊緣帶視數不足", "1 處、30 m²", "AGL ≤ 0 3 張",
                 "5.9 cm（估計值（約 2×GSD），實際以 ODM 輸出為準）", "2–6 m\n—\n沒有地面控制點",
                 # product 含伸出測區的足跡（比測區大）⇒ 測區內的值一起寫
                 "可成圖（視數 ≥5）1.100 km²（測區內 0.940 km²）", "高品質（≥9）0.900 km²（測區內 0.850 km²）",
                 "含足跡伸出測區外的部分"):
        assert want in text, want
    assert "ODM 預設值推估" not in text                     # 有 survey.preview 的說明就不再疊一句同義的


def test_world_terrain_flag_alone_blocks_export(tmp_path):
    pv = make_preview(stats=_stats(terrain_is_world_terrain=True, datum="橢球高（大地起伏 N 未知 ⇒ 不是海拔）"),
                     hillshade_png=_png(4, 4))
    assert hpd.preview_uses_world_terrain(pv)
    with pytest.raises(ValueError, match="World Terrain"):
        hpd.export_preview_files(pv, _png(), tmp_path / "x.png")
    assert not hpd.preview_uses_world_terrain(make_preview())


def test_export_json_is_strict_and_has_no_image_bytes(tmp_path):
    import numpy as np
    pv = make_preview(stats=_stats(seam_overlap_min=float("inf"), forward_overlap_p05=np.float64(0.5),
                                  interior_hole_count=np.int64(3)),
                     image_png=_png(), hillshade_png=_png(8, 8), hillshade_bounds_wsen=(121.2, 25.05, 121.21, 25.06))
    png_bytes = _png(50, 20)
    png, js = hpd.export_preview_files(pv, png_bytes, tmp_path / "out" / "pv")      # 沒副檔名 ⇒ 補 .png
    assert png == tmp_path / "out" / "pv.png" and js == tmp_path / "out" / "pv.json"
    assert png.read_bytes() == png_bytes

    def _no_const(c):
        raise AssertionError(f"JSON 不嚴格：{c}")
    data = json.loads(js.read_text(encoding="utf-8"), parse_constant=_no_const)
    assert data["kind"] == "hdmap_preview" and "規劃預測，非實際成果" in data["label"]
    assert any(ACC_NOTE in d for d in data["disclaimer"]) and any("7.4–12.3 MB" in d for d in data["disclaimer"])
    assert data["stats"]["seam_overlap_min"] is None                  # inf → null
    assert data["stats"]["forward_overlap_p05"] == 0.5 and data["stats"]["interior_hole_count"] == 3
    assert data["stats"]["assumptions"] == ASSUMPTIONS
    assert data["legend"] == _legend()
    assert data["extent_rings"] == [RING] and data["survey_rings"] == [RING]
    assert [h["area_m2"] for h in data["interior_holes"]] == [412.0, 96.0]
    # 分級格網的範圍／像素（不是旁邊那張 PNG 的）：欄位名稱寫明是 raster，另附 PNG 不是地理參照影像的說明
    assert data["raster_bounds_wsen"] == [121.20, 25.05, 121.21, 25.06] and data["raster_px"] == [40, 30]
    assert data["raster_cell_m"] == 2.0 and "bounds_wsen" not in data and "image_px" not in data
    assert "不是地理參照影像" in data["png_note"] and data["stale"] is False and data["stale_reason"] is None
    assert data["photos_count"] == 2 and data["terrain_shading_in_png"] is True
    raw = js.read_text(encoding="utf-8")
    assert "image_png" not in raw and "hillshade_png" not in raw and "base64" not in raw


def test_export_refuses_world_terrain_shading(tmp_path):
    pv = make_preview(stats=_stats(datum="Cesium World Terrain 橢球高（大地起伏 N 未知）"), hillshade_png=_png(8, 8))
    with pytest.raises(ValueError, match="World Terrain"):
        hpd.export_preview_files(pv, _png(), tmp_path / "wt.png")
    assert list(tmp_path.iterdir()) == []                              # 什麼都沒寫
    # 契約下 World Terrain 時 hillshade 是 None ⇒ 可以存（PNG 裡沒有 WT 衍生的陰影）
    pv.hillshade_png = None
    png, js = hpd.export_preview_files(pv, _png(), tmp_path / "wt.png")
    assert png.exists() and json.loads(js.read_text(encoding="utf-8"))["terrain_shading_in_png"] is False
    with pytest.raises(ValueError):
        hpd.export_preview_files(pv, b"", tmp_path / "empty.png")


# ═══════════════════════════ 對話框（widget） ═══════════════════════════
def test_dialog_is_modeless_labelled_and_shows_the_png(qt_app):
    dlg = hpd.HdmapPreviewDialog(make_preview(), map_png=_png(40, 30))
    try:
        assert not dlg.isModal()
        assert "規劃預測" in dlg.windowTitle()
        assert "規劃預測，非實際成果" in dlg.banner.text() and "CAM_MAX_ROLL" in dlg.banner.text()
        assert dlg.map_label.source_size() == (40, 30)
        assert dlg.btn_export.isEnabled()
        assert ACC_NOTE in dlg.stats_browser.toPlainText()
        assert not dlg.stale_label.isVisibleTo(dlg)
        dlg.mark_stale("計畫已變更")
        assert dlg.stale_label.isVisibleTo(dlg) and "計畫已變更" in dlg.stale_label.text()
    finally:
        dlg.close()


def test_dialog_renders_through_survey_preview(qt_app, preview_mod):
    pv = make_preview()
    dlg = hpd.HdmapPreviewDialog(pv)
    try:
        assert len(preview_mod.calls["render"]) == 1 and preview_mod.calls["render"][0][0] is pv
        assert dlg.map_label.source_size() == (64, 48) and dlg.map_png == _png(64, 48)
    finally:
        dlg.close()


def test_dialog_survives_render_failure(qt_app, preview_mod):
    def boom(pv, **kw):
        raise RuntimeError("matplotlib 壞了")
    preview_mod.state["render"] = boom
    dlg = hpd.HdmapPreviewDialog(make_preview())
    try:
        assert "matplotlib 壞了" in dlg.map_label.text()
        assert not dlg.btn_export.isEnabled()                    # 沒圖就不給匯出
        assert "9.1–15.2 GB" in dlg.stats_browser.toPlainText()  # 統計照樣看得到
    finally:
        dlg.close()


def test_dialog_export_button_writes_both_files(qt_app, tmp_path, qmb):
    dlg = hpd.HdmapPreviewDialog(make_preview(), map_png=_png(40, 30))
    try:
        dlg._ask_export_path = lambda: str(tmp_path / "preview.png")
        dlg.btn_export.click()
        assert (tmp_path / "preview.png").read_bytes() == _png(40, 30)
        assert json.loads((tmp_path / "preview.json").read_text(encoding="utf-8"))["kind"] == "hdmap_preview"
        assert len(qmb["information"]) == 1 and qmb["warning"] == []
        dlg._ask_export_path = lambda: ""                     # 取消 ⇒ 什麼都不做
        dlg.btn_export.click()
        assert len(qmb["information"]) == 1
        bad = hpd.HdmapPreviewDialog(make_preview(stats=_stats(datum="Cesium World Terrain"),
                                                 hillshade_png=_png(4, 4)), map_png=_png())
        bad._ask_export_path = lambda: str(tmp_path / "bad.png")
        bad.btn_export.click()
        assert len(qmb["warning"]) == 1 and "World Terrain" in qmb["warning"][0][0][2]
        assert not (tmp_path / "bad.png").exists()
        bad.close()
    finally:
        dlg.close()


# ═══════════════════════════ worker ═══════════════════════════
def _capture(w):
    ok, bad = [], []
    w.finished_ok.connect(ok.append)
    w.failed.connect(bad.append)
    return ok, bad


def test_worker_run_success_passes_inputs_and_stamp(qt_app, preview_mod):
    hv, sw = _hv(), object()
    w = HdmapPreviewWorker(hv, sw, cell_size_m=3.0)
    assert w.stamp == (hv, hv.last_result)
    ok, bad = _capture(w)
    w.run()                                              # 同步跑 run()
    assert preview_mod.calls["compute"] == [(hv, sw, {"cell_size_m": 3.0})]
    assert len(ok) == 1 and ok[0] is w.result and bad == [] and w.error_text is None
    w2 = HdmapPreviewWorker(hv, sw)
    w2.run()
    assert preview_mod.calls["compute"][-1][2] == {}     # 沒指定格網 ⇒ 不帶（用 compute 的預設）


def test_worker_not_ok_and_exceptions_go_to_failed(qt_app, preview_mod):
    preview_mod.state["compute"] = lambda hv, sw, **kw: failed_preview("這份計畫不是建圖模式")
    w = HdmapPreviewWorker(_hv(), None)
    ok, bad = _capture(w)
    w.run()
    assert ok == [] and bad == ["這份計畫不是建圖模式"] and w.error_text == bad[0] and w.result is None

    def boom(hv, sw, **kw):
        raise KeyError("camera")
    preview_mod.state["compute"] = boom
    w = HdmapPreviewWorker(_hv(), None)
    ok, bad = _capture(w)
    w.run()
    assert ok == [] and bad == ["KeyError: 'camera'"]

    preview_mod.state["compute"] = lambda hv, sw, **kw: None
    w = HdmapPreviewWorker(_hv(), None)
    ok, bad = _capture(w)
    w.run()
    assert ok == [] and len(bad) == 1


def test_worker_abandoned_emits_nothing(qt_app, preview_mod):
    w = HdmapPreviewWorker(_hv(), None)
    ok, bad = _capture(w)
    w.stop()
    w.run()
    assert ok == [] and bad == [] and w.abandoned
    preview_mod.state["compute"] = lambda hv, sw, **kw: failed_preview("x")
    w = HdmapPreviewWorker(_hv(), None)
    ok, bad = _capture(w)
    w.stop()
    w.run()
    assert ok == [] and bad == []


def test_worker_thread_delivers_on_the_gui_thread(qt_app, preview_mod):
    seen = []
    preview_mod.state["compute"] = lambda hv, sw, **kw: (seen.append(threading.get_ident()), make_preview())[1]
    w = HdmapPreviewWorker(_hv(), None, parent=None)
    got = []
    w.finished_ok.connect(lambda pv: got.append((pv, threading.get_ident())))
    w.start()
    assert w.wait(10000)
    assert _pump(lambda: got)
    assert seen and seen[0] != threading.get_ident()         # 計算在背景執行緒
    assert got[0][0] is w.result and got[0][1] == threading.get_ident()   # 結果送回 GUI 執行緒


# ═══════════════════════════ MainWindow：計算 → 圖層 → 按鈕 ═══════════════════════════
def test_start_then_done_draws_layer_and_enables_button(win, panel, preview_mod, payload_calls):
    win._start_hdmap_preview(win._hv_coordinator)
    w = win._hdmap_preview_worker
    assert isinstance(w, HdmapPreviewWorker) and w.stamp == (win._hv_coordinator, win._dccpp_result)
    assert panel.hdmap_preview_state() == "computing"           # 結果要回到 GUI 執行緒才會變 ready
    assert w.wait(10000)
    assert _pump(lambda: panel.hdmap_preview_state() == "ready")
    assert preview_mod.calls["compute"][0][0] is win._hv_coordinator
    assert preview_mod.calls["compute"][0][1] is win._hv_swarm_coordinator
    pv = win._hdmap_preview
    assert isinstance(pv, sp.HdmapPreview) and win._hdmap_preview_worker is None
    sets = [c for c in win.map_widget.calls if c[0] == "set"]
    assert len(sets) == 1 and sets[0][1] == mw.HDMAP_PREVIEW_LAYER_ID == "hdmap_plan"
    assert sets[0][2] is win._hdmap_preview_payload and [c for c in payload_calls if c is pv] == [pv]
    assert sets[0][2] == pv.to_js_payload()                      # 送給地圖的就是真的 to_js_payload（預設值）
    assert ("visible", "hdmap_plan", True) in win.map_widget.calls
    assert win._hdmap_preview_drawn is True
    assert panel.btn_hdmap_preview.isEnabled()
    summary = mw.hdmap_preview_summary(pv)
    assert panel.hdmap_preview_status_label.text() == summary
    assert win._status.msgs and win._status.msgs[-1].startswith(summary)


def test_summary_line_values():
    s = mw.hdmap_preview_summary(make_preview())
    assert s == ("預計成果（規劃預測，非實際成果）：可成圖 0.812 km²（視數 ≥5）／測區 0.950 km²、內部破洞 2 處、"
                 "GSD 中位 2.95 cm、照片 1,234 張（約 9.1–15.2 GB）")
    s = mw.hdmap_preview_summary(make_preview(stats={}))
    assert "可成圖 — km²" in s and "約 —–— GB" in s
    # survey.preview 的 product 含伸出測區外的足跡（會比測區大）⇒ 有「測區內」的值就用它跟測區並列
    st = _stats(area_km2={"survey": 1.196, "product": 1.383, "high_quality": 1.269,
                          "product_in_survey": 1.195, "high_quality_in_survey": 1.16})
    s = mw.hdmap_preview_summary(make_preview(stats=st))
    assert "：測區內可成圖 1.195 km²（視數 ≥5）／測區 1.196 km²" in s and "1.383" not in s


def test_layer_checkbox_off_defers_drawing_until_ticked(win, panel, preview_mod, payload_calls):
    panel.chk_hdmap_preview_layer.setChecked(False)
    panel.hdmap_preview_layer_toggled.connect(win._on_hdmap_preview_layer_toggled)
    win._start_hdmap_preview(win._hv_coordinator)
    assert win._hdmap_preview_worker.wait(10000)
    assert _pump(lambda: panel.hdmap_preview_state() == "ready")
    assert win.map_widget.calls == []                              # 勾選關著 ⇒ 不畫
    panel.chk_hdmap_preview_layer.setChecked(True)                 # 真的信號路徑
    assert win.map_widget.names() == ["set", "visible"]
    panel.chk_hdmap_preview_layer.setChecked(False)
    panel.chk_hdmap_preview_layer.setChecked(True)
    assert win.map_widget.calls[2:] == [("visible", "hdmap_plan", False), ("visible", "hdmap_plan", True)]
    assert [c for c in payload_calls if c is win._hdmap_preview] == [win._hdmap_preview]   # payload 只算一次


def test_draw_reports_partial_and_missing_map_api(win, preview_mod):
    win._hdmap_preview = make_preview()
    win.map_widget = RecMap(result={"2d": {"ok": True}, "3d": {"ok": False, "error": "page not ready"}})
    assert win._hdmap_preview_draw() is True                        # 一邊有畫上就算有
    win._hdmap_preview_payload = None
    win.map_widget = RecMap(result={"2d": {"ok": False}, "3d": {"ok": False}})
    assert win._hdmap_preview_draw() is False
    win.map_widget = RecMap(result={"ok": True})                    # 單一 widget 的回傳形狀
    assert win._hdmap_preview_draw() is True
    win.map_widget = object()                                       # B 部分還沒落地：不炸
    assert win._hdmap_preview_draw() is False


def test_failure_shows_reason_and_keeps_button_dark(win, panel, preview_mod):
    preview_mod.state["compute"] = lambda hv, sw, **kw: failed_preview("這份計畫沒有相機像素大小")
    win._start_hdmap_preview(win._hv_coordinator)
    assert win._hdmap_preview_worker.wait(10000)
    assert _pump(lambda: panel.hdmap_preview_state() == "failed")
    assert "這份計畫沒有相機像素大小" in panel.hdmap_preview_status_label.text()
    assert not panel.btn_hdmap_preview.isEnabled()
    assert win._hdmap_preview is None and win._hdmap_preview_worker is None
    assert win.map_widget.calls == []
    assert "這份計畫沒有相機像素大小" in win._status.msgs[-1]


def test_result_for_a_replaced_plan_is_dropped(win, panel, preview_mod):
    gate = threading.Event()
    preview_mod.state["compute"] = lambda hv, sw, **kw: (gate.wait(10), make_preview())[1]
    win._start_hdmap_preview(win._hv_coordinator)
    w = win._hdmap_preview_worker
    # 計算中計畫被換掉（沒經過重設的路徑）
    win._hv_coordinator.last_result = {"plan": 2}
    win._dccpp_result = win._hv_coordinator.last_result
    gate.set()
    assert w.wait(10000)
    assert _pump(lambda: win.__dict__.get("_hdmap_preview_worker") is None)
    assert win._hdmap_preview is None and win.map_widget.calls == []
    assert panel.hdmap_preview_state() == "none"


@pytest.mark.parametrize("mutate, why", [
    (lambda w: setattr(w, "_hv_coordinator", _hv()), "規劃器物件已換"),
    (lambda w: setattr(w._hv_coordinator, "last_result", {"x": 9}), "計畫結果已換"),
    (lambda w: setattr(w, "_dccpp_result", None), "目前畫面上的結果已不是這份建圖計畫"),
    (lambda w: setattr(w._hv_coordinator, "mapping_mode", None), "不是建圖模式"),
])
def test_stale_reason_values(win, mutate, why):
    stamp = (win._hv_coordinator, win._hv_coordinator.last_result)
    assert mw.hdmap_preview_stale_reason(win, stamp) is None
    mutate(win)
    assert why in mw.hdmap_preview_stale_reason(win, stamp)
    assert mw.hdmap_preview_stale_reason(win, None)


def test_queued_signal_from_an_old_worker_is_ignored(win, panel):
    hv = win._hv_coordinator
    pv_old = make_preview()
    win._hdmap_preview_worker = types.SimpleNamespace(result=None, error_text=None, stamp=(hv, hv.last_result))
    win._on_hdmap_preview_done(pv_old)
    win._on_hdmap_preview_failed("舊的那趟失敗")
    assert win.__dict__.get("_hdmap_preview") is None
    assert win.map_widget.calls == [] and panel.hdmap_preview_state() == "none"
    assert win._hdmap_preview_worker is not None                    # 目前這一趟沒被動到


def test_reset_while_computing_parks_the_running_worker(win, panel, preview_mod):
    gate = threading.Event()
    preview_mod.state["compute"] = lambda hv, sw, **kw: (gate.wait(10), make_preview())[1]
    win._start_hdmap_preview(win._hv_coordinator)
    w = win._hdmap_preview_worker
    assert _pump(lambda: w.isRunning(), 5)
    mw.hdmap_preview_reset(win, "新的規劃開始")
    assert win._hdmap_preview_worker is None and w.abandoned
    assert w in mw._HV_ORPHAN_WORKERS                               # 還在跑 ⇒ 寄放，不是直接丟參考
    assert panel.hdmap_preview_state() == "none"
    gate.set()
    assert w.wait(10000)
    assert _pump(lambda: w not in mw._HV_ORPHAN_WORKERS, 5)
    assert win._hdmap_preview is None and win.map_widget.calls == []


def test_close_abandon_waits_then_parks(qt_app, preview_mod):
    # 3 s 內結束 ⇒ 等到、不寄放
    preview_mod.state["compute"] = lambda hv, sw, **kw: (time.sleep(0.3), make_preview())[1]
    w = HdmapPreviewWorker(_hv(), None)
    got = []
    w.finished_ok.connect(got.append)
    w.start()
    mw._hdmap_preview_abandon(w, closing=True)
    assert not w.isRunning() and w not in mw._HV_ORPHAN_WORKERS
    QApplication.processEvents()
    assert got == []                                                # 放棄 ⇒ 結果不送
    # 3 s 還沒完 ⇒ 寄放到它自己結束
    gate = threading.Event()
    preview_mod.state["compute"] = lambda hv, sw, **kw: (gate.wait(20), make_preview())[1]
    w2 = HdmapPreviewWorker(_hv(), None)
    w2.start()
    t0 = time.monotonic()
    mw._hdmap_preview_abandon(w2, closing=True)
    assert time.monotonic() - t0 >= 2.5 and w2 in mw._HV_ORPHAN_WORKERS
    gate.set()
    assert w2.wait(10000)
    assert _pump(lambda: w2 not in mw._HV_ORPHAN_WORKERS, 5)


def test_close_event_hands_the_preview_worker_to_abandon():
    fn = next(n for n in ast.walk(ast.parse(SRC)) if isinstance(n, ast.FunctionDef) and n.name == "closeEvent")
    calls = [c for c in ast.walk(fn) if isinstance(c, ast.Call) and getattr(c.func, "id", None) == "_hdmap_preview_abandon"]
    assert calls and any(k.arg == "closing" and getattr(k.value, "value", None) is True for k in calls[0].keywords)


# ═══════════════════════════ 重設 ═══════════════════════════
def test_reset_clears_drawn_layer_panel_and_marks_dialog_stale(win, panel, qt_app):
    win._hdmap_preview = make_preview()
    win._hdmap_preview_payload = {"x": 1}
    win._hdmap_preview_drawn = True
    panel.set_hdmap_preview_state("ready")
    dlg = hpd.HdmapPreviewDialog(win._hdmap_preview, map_png=_png())
    win._hdmap_preview_dialog = dlg
    mw.hdmap_preview_reset(win, "清除全部")
    assert win.map_widget.calls == [("clear", "hdmap_plan")]
    assert win._hdmap_preview is None and win._hdmap_preview_payload is None and win._hdmap_preview_drawn is False
    assert panel.hdmap_preview_state() == "none"
    assert dlg.stale_label.isVisibleTo(dlg) and "清除全部" in dlg.stale_label.text()
    mw.hdmap_preview_reset(win, "再一次")                            # 沒畫過 ⇒ 不再叫 clear
    assert win.map_widget.calls == [("clear", "hdmap_plan")]
    dlg.close()


def test_reset_works_on_a_bare_mainwindow_instance():
    bare = MainWindow.__new__(MainWindow)                           # C++ 端沒建：getattr 會丟 RuntimeError
    mw.hdmap_preview_reset(bare, "x")
    assert bare.__dict__["_hdmap_preview"] is None and bare.__dict__["_hdmap_preview_drawn"] is False
    bare.map_widget = RecMap()
    bare._hdmap_preview_drawn = True
    mw.hdmap_preview_reset(bare, "y")
    assert bare.map_widget.calls == [("clear", "hdmap_plan")]


# ═══════════════════════════ 生命週期掛勾（真的方法、假 self） ═══════════════════════════
@pytest.fixture
def reset_log(monkeypatch):
    log = []
    real = mw.hdmap_preview_reset

    def rec(w, why=""):
        log.append(("reset", why))
        return real(w, why)
    monkeypatch.setattr(mw, "hdmap_preview_reset", rec)
    return log


def _drawn(obj):
    obj._hdmap_preview = make_preview()
    obj._hdmap_preview_drawn = True
    return obj


def test_clear_paths_and_clear_all_drop_the_layer(reset_log):
    label = types.SimpleNamespace(setText=lambda *_: None)

    class M(RecMap):
        def clear_corners(self): self.calls.append(("clear_corners",))
        def clear_circle_overlay(self): self.calls.append(("clear_circle",))
        def clear_home_point_overlay(self): self.calls.append(("clear_home",))
        def clear_all(self): self.calls.append(("clear_all",))
    bare = MainWindow.__new__(MainWindow)
    for k, v in dict(map_widget=M(), parameter_panel=types.SimpleNamespace(update_corner_count=lambda n: None,
                                                                         clear_home_point_display=lambda: None),
                     corners=[(25.07, 121.23)], waypoints=[(25.07, 121.23, 50.0)], spiral_waypoint_altitudes=[50.0],
                     waypoint_label=label, distance_label=label, circle_center=None, picking_circle_center=False,
                     home_point=None, picking_home_point=False, obstacles=[], nfz_zones=[],
                     _dccpp_result={"x": 1}).items():
        setattr(bare, k, v)
    _drawn(bare)
    MainWindow.on_clear_paths(bare)
    assert ("reset", "清除路徑") in reset_log and ("clear", "hdmap_plan") in bare.map_widget.calls
    assert bare.__dict__["_hdmap_preview"] is None
    _drawn(bare)
    reset_log.clear()
    MainWindow.on_clear_all_silent(bare)
    assert [r for r in reset_log] == [("reset", "清除路徑"), ("reset", "清除全部")]
    assert bare.map_widget.calls.count(("clear", "hdmap_plan")) == 2   # 第一次 clear_paths、第二次 clear_all 各清一次已畫的
    assert bare.__dict__["_hdmap_preview"] is None


class _Plain:
    """不是 QObject 的假 self（statusBar 用假的）。"""

    def __init__(self):
        self._status = FakeStatus()
        self.map_widget = RecMap()
        self.done = []

    def statusBar(self):
        return self._status

    def _hv_notify_done(self, ok):
        self.done.append(ok)


def test_hv_failed_and_cancelled_drop_the_preview(reset_log, qmb):
    f = _drawn(_Plain())
    f._hv_last_report = {}
    MainWindow._on_hv_failed(f, "NO_GO")
    assert ("reset", "規劃失敗") in reset_log and ("clear", "hdmap_plan") in f.map_widget.calls
    assert f._hdmap_preview is None and f.done == [False]
    g = _drawn(_Plain())
    MainWindow._on_hv_cancelled(g)
    assert ("reset", "規劃取消") in reset_log and g._hdmap_preview is None and g.done == [False]


def test_vehicle_type_change_drops_the_preview(reset_log):
    f = _drawn(_Plain())
    f.current_vehicle_type, f._dccpp_result, f.flight_params, f.circle_center = "vtol", None, {}, None
    MainWindow.on_parameters_changed(f, {"vehicle_type": "vtol"})          # 沒換 ⇒ 不動
    assert reset_log == [] and f._hdmap_preview is not None
    MainWindow.on_parameters_changed(f, {"vehicle_type": "multirotor"})
    assert reset_log == [("reset", "載具改為 multirotor")] and f._hdmap_preview is None


def test_recon_planned_without_result_drops_and_mapping_result_starts_after_gate_status(reset_log, panel):
    f = _drawn(_Plain())
    f.vtol_recon_panel = panel
    f._dccpp_result = None
    MainWindow._on_vtol_recon_planned(f, {}, False)
    assert ("reset", "規劃未產生結果") in reset_log and f._hdmap_preview is None
    # 建圖計畫：閘門狀態先顯示，再開始算預計成果
    order = []
    hv = _hv({"assembled_paths": {}, "total_makespan": 0.0})
    g = _Plain()
    g.vtol_recon_panel, g._hv_coordinator, g._dccpp_result = panel, hv, hv.last_result
    g._get_cesium_widget = lambda: None
    g._polyline_length_m = MainWindow._polyline_length_m
    g._show_mapping_plan_status = lambda h: order.append(("gate", h))
    g._start_hdmap_preview = lambda h: order.append(("preview", h))
    panel.set_hdmap_preview_state("ready")
    MainWindow._on_vtol_recon_planned(g, {"num_drones": 0}, True)
    assert order == [("gate", hv), ("preview", hv)]
    assert panel.hdmap_preview_state() == "none"                         # set_plan_result 先回 'none'
    # 不是建圖計畫 ⇒ 不算
    order.clear()
    hv.mapping_mode = None
    MainWindow._on_vtol_recon_planned(g, {"num_drones": 0}, True)
    assert order == []
    # 開始計算本身丟例外 ⇒ 面板 'failed'，規劃結果照常
    hv.mapping_mode = types.SimpleNamespace(lead_in_m=40.0)
    g._start_hdmap_preview = lambda h: (_ for _ in ()).throw(ImportError("no survey.preview"))
    MainWindow._on_vtol_recon_planned(g, {"num_drones": 0}, True)
    assert panel.hdmap_preview_state() == "failed" and "no survey.preview" in panel.hdmap_preview_status_label.text()


def test_new_plan_start_resets_before_clearing_display_layers(monkeypatch, reset_log, qmb):
    log = reset_log

    class _Coord:
        use_harvest = False

        def plan_coverage_dccpp(self, **kw):
            raise RuntimeError("到這裡就夠了")
    monkeypatch.setattr(mw, "SwarmCoordinator", _Coord)
    f = _drawn(_Plain())
    f.map_widget = RecMap(log=log)
    f.corners, f.home_point = [], None
    f.survey_area_polygons = lambda: [[(25.0, 121.0), (25.0, 121.01), (25.01, 121.01)]]
    f._hv_planning_in_flight = lambda: False
    f._configure_hv_dccpp = lambda c, p, v: False
    MainWindow._on_dccpp_coverage_requested(f, {"num_drones": 1}, run_async=False)
    assert log[:2] == [("reset", "新的規劃開始"), ("clear_js",)]
    assert f._hdmap_preview is None and ("clear", "hdmap_plan") in f.map_widget.calls
    assert qmb["warning"]                                                  # 規劃失敗照常報
    # 背景規劃還在跑 ⇒ 被閘門擋下，不動預計成果
    log.clear()
    h = _drawn(_Plain())
    h.survey_area_polygons = f.survey_area_polygons
    h._hv_planning_in_flight = lambda: True
    MainWindow._on_dccpp_coverage_requested(h, {"num_drones": 1})
    assert log == [] and h._hdmap_preview is not None


# ═══════════════════════════ 開對話框 ═══════════════════════════
def test_open_without_preview_explains(win, qmb):
    win._on_hdmap_preview_open()
    assert len(qmb["information"]) == 1 and "航測建圖" in qmb["information"][0][0][2]
    win._hdmap_preview_worker = types.SimpleNamespace()
    win._on_hdmap_preview_open()
    assert "還在背景計算" in qmb["information"][1][0][2]
    win._hdmap_preview_worker = None


def test_open_shows_one_modeless_dialog_per_preview(win, preview_mod, qmb):
    win._hdmap_preview = make_preview()
    win._on_hdmap_preview_open()
    dlg = win._hdmap_preview_dialog
    assert isinstance(dlg, hpd.HdmapPreviewDialog) and dlg.isVisible() and not dlg.isModal()
    assert dlg.parent() is win and dlg.preview is win._hdmap_preview
    assert len(preview_mod.calls["render"]) == 1
    win._on_hdmap_preview_open()                                     # 同一份 ⇒ 同一個視窗拉到前面
    assert win._hdmap_preview_dialog is dlg and len(preview_mod.calls["render"]) == 1
    win._hdmap_preview = make_preview()                               # 新的一份 ⇒ 新視窗、舊的關掉
    win._on_hdmap_preview_open()
    assert win._hdmap_preview_dialog is not dlg and not dlg.isVisible()
    win._hdmap_preview_dialog.close()
    assert qmb["warning"] == []


# ═══════════════════════════ 接線（原始碼） ═══════════════════════════
def test_menu_and_panel_wiring():
    tree = ast.parse(SRC)
    texts = []
    for c in ast.walk(tree):
        if (isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute) and c.func.attr == "addAction"
                and c.args and isinstance(c.args[0], ast.Constant)):
            texts.append(c.args[0].value)
    i = texts.index("航測處理…")
    assert texts[i + 1] == "預覽高精地圖成果…"                       # 緊接在「航測處理…」後面
    assert 'action = tools_menu.addAction("預覽高精地圖成果…")' in SRC
    blk = SRC.split('tools_menu.addAction("預覽高精地圖成果…")', 1)[1].split("tools_menu.addAction", 1)[0]
    assert "action.triggered.connect(self._on_hdmap_preview_open)" in blk
    assert "self.vtol_recon_panel.hdmap_preview_requested.connect(self._on_hdmap_preview_open)" in SRC
    assert ("self.vtol_recon_panel.hdmap_preview_layer_toggled.connect(self._on_hdmap_preview_layer_toggled)"
            in SRC)


def test_worker_has_no_parent_and_mainwindow_does_not_touch_widgets_in_run():
    src = (ROOT / "ui" / "workers" / "hdmap_preview_worker.py").read_text(encoding="utf-8")
    assert "QWidget" not in src and "PyQt6.QtWidgets" not in src
    assert "HdmapPreviewWorker(hv, sc, stamp=stamp, parent=None)" in SRC


# ═══════════════════════════ 契約：真的 dataclass 欄位 ═══════════════════════════
def test_real_hdmap_preview_has_exactly_the_contract_fields_and_feeds_dialog_and_summary(qt_app, tmp_path):
    assert tuple(f.name for f in dataclasses.fields(sp.HdmapPreview)) == CONTRACT_FIELDS
    pv = make_preview()
    assert type(pv) is sp.HdmapPreview and callable(pv.to_js_payload)
    assert "可成圖 0.812 km²" in mw.hdmap_preview_summary(pv)
    assert ACC_NOTE in _plain(hpd.preview_html(pv))
    bad = failed_preview("x")
    assert bad.ok is False and bad.reason == "x" and bad.stats["k_product"] == 5
    dlg = hpd.HdmapPreviewDialog(pv, map_png=_png())
    try:
        png, js = hpd.export_preview_files(pv, dlg.map_png, tmp_path / "real.png")
        assert json.loads(js.read_text(encoding="utf-8"))["stats"]["k_product"] == 5
    finally:
        dlg.close()


# ═══════════════════════════ 端到端：真的規劃 → 真的 compute → MainWindow → 對話框 ═══════════════════════════
@pytest.fixture(scope="module")
def real_plan():
    pytest.importorskip("hv_dccpp")
    from tests.test_hdmap_gui_coordinator import mapping_params, plan
    return plan(mapping_params())                       # 約 20 s：真的 HARVEST 建圖規劃


def test_end_to_end_real_plan_through_mainwindow_and_dialog(qt_app, panel, real_plan, tmp_path, qmb):
    hv = real_plan.hv_coordinator
    win = Win(hv, panel, RecMap())
    win._hv_swarm_coordinator = real_plan               # 規劃用的 SwarmCoordinator（有效輸入在它身上）
    try:
        win._start_hdmap_preview(hv)
        w = win._hdmap_preview_worker
        assert panel.hdmap_preview_state() == "computing"
        assert w.wait(180000)
        assert _pump(lambda: panel.hdmap_preview_state() in ("ready", "failed"), 30)
        assert panel.hdmap_preview_state() == "ready", panel.hdmap_preview_status_label.text()
        pv = win._hdmap_preview
        assert isinstance(pv, sp.HdmapPreview) and pv.ok and pv.stats["k_product"] == 5
        assert panel.btn_hdmap_preview.isEnabled()
        assert panel.hdmap_preview_status_label.text() == mw.hdmap_preview_summary(pv)
        sets = [c for c in win.map_widget.calls if c[0] == "set"]
        assert len(sets) == 1 and sets[0][1] == "hdmap_plan"
        payload = sets[0][2]
        json.dumps(payload, allow_nan=False)                    # 送給 JS 的一定是嚴格 JSON
        assert "規劃預測" in payload["label"] and payload["image"].startswith("data:image/png;base64,")
        from ui.widgets import hdmap_preview_layer as hl        # B 部分的圖層要收得下這份 payload（A→C→B 同一份）
        hl.check_payload(payload, "hdmap_plan")
        # 對話框：真的 render_preview_map_png
        win._on_hdmap_preview_open()
        dlg = win._hdmap_preview_dialog
        assert qmb["warning"] == [] and dlg.map_label.source_size()[0] > 0
        text = dlg.stats_browser.toPlainText()
        assert "規劃預測，非實際成果" in text and sp.ACCURACY_NOTE in text
        for a in pv.stats["assumptions"]:
            assert a in text
        assert MAPPING_EXPOSURE_SEAM_CAVEAT in text
        png, js = hpd.export_preview_files(pv, dlg.map_png, tmp_path / "e2e.png")
        data = json.loads(js.read_text(encoding="utf-8"))
        assert data["stats"]["photos_total"] == pv.stats["photos_total"] and png.read_bytes() == dlg.map_png
        dlg.close()
        # 清除路徑那類重設 ⇒ 圖層清掉、面板回 'none'
        mw.hdmap_preview_reset(win, "清除路徑")
        assert win.map_widget.calls[-1] == ("clear", "hdmap_plan") and panel.hdmap_preview_state() == "none"
    finally:
        pw = win.__dict__.get("_hdmap_preview_worker")
        if isinstance(pw, HdmapPreviewWorker):
            pw.wait(60000)


def _plain(html_text: str) -> str:
    """HTML → 純文字（用 QTextDocument，跟 QTextBrowser 顯示的一樣）。"""
    from PyQt6.QtGui import QTextDocument
    d = QTextDocument()
    d.setHtml(html_text)
    return d.toPlainText()


# ═══════════════════════════ 審查修正（2026-10-19） ═══════════════════════════
class StatusWin(Win):
    """真的 QStatusBar（訊息有沒有被蓋掉要看真的行為）＋ HARVEST 橫幅用的方法。"""

    def __init__(self, hv, panel, map_widget):
        super().__init__(hv, panel, map_widget)
        self._status = _QtW.QStatusBar()
        self._hv_show_report_banner = types.MethodType(MainWindow._hv_show_report_banner, self)


def test_preview_does_not_overwrite_the_harvest_loud_note_or_uncovered_warning(qt_app, panel, preview_mod, qmb):
    win = StatusWin(_hv(), panel, RecMap())
    win._hv_last_report = {"loud_notes": ["coverage_altitude_clamped(100.0->120.0)"], "uncovered": []}
    win._hv_show_report_banner()
    loud = "HARVEST：coverage_altitude_clamped(100.0->120.0)"
    assert win.statusBar().currentMessage() == loud
    win._start_hdmap_preview(win._hv_coordinator)
    assert win._hdmap_preview_worker.wait(10000)
    assert _pump(lambda: panel.hdmap_preview_state() == "ready")
    assert win.statusBar().currentMessage() == loud, "預計成果的摘要蓋掉了 HARVEST 的重點"
    assert panel.hdmap_preview_status_label.text() == mw.hdmap_preview_summary(win._hdmap_preview)
    # 未覆蓋區域（常駐）也一樣；失敗訊息也不蓋
    win._hv_last_report = {"loud_notes": [], "uncovered": [["A1", 0.3]]}
    win._hv_show_report_banner()
    unc = win.statusBar().currentMessage()
    assert unc.startswith("⚠ HARVEST 有未覆蓋區域")
    win._hdmap_preview_show_failure("相機沒有像元大小")
    assert win.statusBar().currentMessage() == unc
    assert "相機沒有像元大小" in panel.hdmap_preview_status_label.text()
    # 狀態列空著（或是規劃後處理的「完成」）⇒ 照樣顯示
    win.statusBar().showMessage("DCCPP 最佳化完成: 3 台 VTOL", 0)
    assert mw.hdmap_preview_status_message(win, "預計成果（規劃預測，非實際成果）：x", 1000) is True
    assert win.statusBar().currentMessage().startswith("預計成果")
    win.statusBar().clearMessage()
    assert mw.hdmap_preview_status_message(win, "預計成果：y", 1000) is True


def test_mapping_status_keeps_the_harvest_loud_notes(panel, monkeypatch):
    monkeypatch.setattr(mw, "hv_upload_gate", lambda hv, sc: {"uploadable": True, "reasons": []})
    monkeypatch.setattr(mw, "harvest_takeoff_offsets", lambda hv: ({}, ""))
    monkeypatch.setattr(mw, "mapping_exposure_lines", lambda xs: ["曝光層級（視數）：內部破洞 0 處"])
    f = _Plain()
    f.vtol_recon_panel = panel
    f._hv_swarm_coordinator = None
    f._hv_last_report = {"loud_notes": ["coverage_altitude_clamped(100.0->120.0)", "mapping_setup(相機…)"],
                         "uncovered": [["A1", 0.3]]}
    hv = types.SimpleNamespace(camera_cfg=None, mapping_effective={}, mapping_mode=types.SimpleNamespace(lead_in_m=40.0))
    MainWindow._show_mapping_plan_status(f, hv)
    text = panel.hv_map_status_label.text()
    assert "HARVEST：coverage_altitude_clamped(100.0->120.0)" in text
    assert "⚠ HARVEST 有未覆蓋區域：['A1', 0.3]" in text
    assert "mapping_setup" not in text, "建圖狀態字已另外寫出的不重複"
    assert text.index("曝光層級") < text.index("HARVEST：") < text.index("建圖閘門：可上傳")
    assert mw.harvest_report_lines(None) == [] and mw.harvest_report_lines({}) == []


class _RaisingMap(RecMap):
    def set_hdmap_preview(self, layer_id, payload):
        self.calls.append(("set", layer_id, payload))
        raise RuntimeError("3D page not ready")


def test_draw_failure_is_reported_after_the_summary_and_retick_redraws(win, panel, preview_mod):
    panel.hdmap_preview_layer_toggled.connect(win._on_hdmap_preview_layer_toggled)
    for bad, frag in ((RecMap(result={"2d": {"ok": False, "error": "page gone"}, "3d": {"ok": False}}),
                       "2D：page gone；3D：頁面回報失敗"),
                      (_RaisingMap(), "RuntimeError: 3D page not ready")):
        win.map_widget = bad
        win._status = FakeStatus()
        win._start_hdmap_preview(win._hv_coordinator)
        assert win._hdmap_preview_worker.wait(10000)
        assert _pump(lambda: panel.hdmap_preview_state() == "ready")
        summary = mw.hdmap_preview_summary(win._hdmap_preview)
        txt = panel.hdmap_preview_status_label.text()
        assert txt.startswith(summary) and "地圖圖層畫不出來" in txt and frag in txt, txt
        assert "取消再勾「地圖顯示預計成果」會重畫" in txt
        assert win._status.msgs[-1].startswith(summary) and frag in win._status.msgs[-1], "失敗不能被成功的摘要蓋掉"
        assert win._hdmap_preview_drawn is True and win._hdmap_preview_draw_error
        # 修好之後取消再勾 ⇒ 整份重送一次，錯誤字拿掉
        good = RecMap()
        win.map_widget = good
        panel.chk_hdmap_preview_layer.setChecked(False)
        panel.chk_hdmap_preview_layer.setChecked(True)
        assert [c[0] for c in good.calls].count("set") == 1, good.calls
        assert win._hdmap_preview_draw_error is None
        assert panel.hdmap_preview_status_label.text() == summary
        panel.chk_hdmap_preview_layer.setChecked(False)               # 畫好了 ⇒ 之後只切顯示
        panel.chk_hdmap_preview_layer.setChecked(True)
        assert [c[0] for c in good.calls].count("set") == 1
        mw.hdmap_preview_reset(win, "下一輪")
        assert win._hdmap_preview_draw_error is None


def test_open_without_preview_says_why_after_failure_and_while_planning(win, qmb):
    win._hdmap_preview_show_failure("相機沒有像元大小")
    win._on_hdmap_preview_open()
    assert qmb["information"][-1][0][2] == "上一份建圖計畫的預計成果無法產生：相機沒有像元大小"
    mw.hdmap_preview_reset(win, "新的規劃開始")                       # 重設會清掉上一趟的原因
    win._hv_planning_in_flight = lambda: True
    win._on_hdmap_preview_open()
    assert "規劃還在背景執行" in qmb["information"][-1][0][2]
    win._hv_planning_in_flight = lambda: False
    win._on_hdmap_preview_open()
    assert "請在 VTOL 偵察頁用「航測建圖」模式" in qmb["information"][-1][0][2]


def test_summary_line_names_the_hole_basis_and_preview_only_holes():
    st = _stats(interior_hole_count=0, interior_hole_basis="gate", gate_cell_size_m=4.0, cell_size_m=2.0,
                preview_only_hole_count=1)
    s = mw.hdmap_preview_summary(make_preview(stats=st))
    assert "內部破洞 0 處（上傳閘門 4 m 格），預覽 2 m 格另見 1 處（閘門沒擋，建議檢查）" in s
    st = _stats(interior_hole_count=3, interior_hole_basis="preview", cell_size_m=2.0, gate_cell_size_m=4.0)
    assert "內部視數不足 3 處（預覽 2 m 格；上傳閘門結果未知，以閘門為準）" in mw.hdmap_preview_summary(make_preview(stats=st))


def test_stats_card_states_the_gate_result_and_the_preview_grid_separately(qt_app):
    st = _stats(interior_hole_count=0, interior_hole_area_m2=0.0, interior_hole_basis="gate", gate_cell_size_m=4.0,
                cell_size_m=2.0, preview_grid_hole_count=1, preview_grid_hole_area_m2=1596.0,
                preview_only_hole_count=1, gate_disagrees=True,
                gate={"cell_size_m": 4.0, "interior_hole_count": 0, "undercovered_interior_area_m2": 0.0,
                      "uploadable": True},
                data_mb_per_photo=[0.95, 1.58], data_mb_basis="本相機 1.58 MP × 0.6–1.0 B/px（未經實測）")
    holes = [{"ring": RING, "area_m2": 1596.0, "min_views": 0, "source": "preview_only"}]
    text = _plain(hpd.preview_html(make_preview(stats=st, interior_holes=holes)))
    for want in ("內部破洞（上傳閘門，格 4 m）\n0 處、0 m²（視數 < 5；不擋上傳；閘門格另算一次，與上傳閘門同一判定）"
                 "；建圖閘門目前：可上傳",
                 "預覽格 2 m 的內部視數不足\n1 處、1,596 m²；其中只在預覽格看到 1 處（閘門沒擋，仍建議檢查）"
                 "（能不能上傳以上一列的閘門為準）",
                 "⚠ 兩個格距不一致", "紅框＝閘門判定、紅點框＝只在預覽格看到",
                 "1,596 m²，最少 0 視（只在預覽細格看到，閘門沒擋，建議檢查）",
                 "內部（離邊界 > 37.5 m）（預覽格 2 m）",
                 "（假設每張 0.95–1.58 MB＝本相機 1.58 MP × 0.6–1.0 B/px（未經實測），依壓縮與場景而變）"):
        assert want in text, want
    assert "有就擋上傳" not in text


def test_export_paths_never_double_suffix_and_never_overwrite_silently(tmp_path):
    assert hpd.export_target_paths(tmp_path / "plan.json") == (tmp_path / "plan.png", tmp_path / "plan.json")
    assert hpd.export_target_paths(tmp_path / "site.PNG") == (tmp_path / "site.PNG", tmp_path / "site.json")
    assert hpd.export_target_paths(tmp_path / "pv") == (tmp_path / "pv.png", tmp_path / "pv.json")
    keep = tmp_path / "site.json"
    keep.write_text("使用者自己的資料", encoding="utf-8")
    with pytest.raises(FileExistsError, match="site.json"):
        hpd.export_preview_files(make_preview(), _png(), tmp_path / "site.png")
    assert keep.read_text(encoding="utf-8") == "使用者自己的資料" and not (tmp_path / "site.png").exists()
    png, js = hpd.export_preview_files(make_preview(), _png(), tmp_path / "plan.json")
    assert (png.name, js.name) == ("plan.png", "plan.json") and sorted(p.name for p in tmp_path.iterdir()) == [
        "plan.json", "plan.png", "site.json"]


def test_dialog_asks_before_overwriting_the_matching_json(qt_app, tmp_path, qmb, monkeypatch):
    answers = []
    monkeypatch.setattr(_QtW.QMessageBox, "question",
                        staticmethod(lambda *a, **k: (qmb["question"].append((a, k)), answers.pop(0))[1]))
    dlg = hpd.HdmapPreviewDialog(make_preview(), map_png=_png(40, 30))
    try:
        (tmp_path / "site.json").write_text("舊的", encoding="utf-8")
        dlg._ask_export_path = lambda: str(tmp_path / "site.png")
        answers.append(_QtW.QMessageBox.StandardButton.No)
        dlg._on_export()
        assert len(qmb["question"]) == 1 and "site.json" in qmb["question"][0][0][2]
        assert (tmp_path / "site.json").read_text(encoding="utf-8") == "舊的" and not (tmp_path / "site.png").exists()
        answers.append(_QtW.QMessageBox.StandardButton.Yes)
        dlg._on_export()
        assert json.loads((tmp_path / "site.json").read_text(encoding="utf-8"))["kind"] == "hdmap_preview"
        assert (tmp_path / "site.png").read_bytes() == _png(40, 30) and len(qmb["information"]) == 1
        # 沒有同名 JSON ⇒ 不問（PNG 的覆寫由存檔對話框自己問）
        n = len(qmb["question"])
        dlg._ask_export_path = lambda: str(tmp_path / "fresh.png")
        dlg._on_export()
        assert len(qmb["question"]) == n and (tmp_path / "fresh.json").exists()
    finally:
        dlg.close()


def test_stale_dialog_export_asks_and_marks_the_json(qt_app, tmp_path, qmb, monkeypatch):
    answers = []
    monkeypatch.setattr(_QtW.QMessageBox, "question",
                        staticmethod(lambda *a, **k: (qmb["question"].append((a, k)), answers.pop(0))[1]))
    dlg = hpd.HdmapPreviewDialog(make_preview(), map_png=_png(40, 30))
    try:
        assert dlg.stale_reason is None and "_stale" not in dlg._default_export_name()
        dlg.mark_stale("計畫已變更（清除全部）：這份預覽不再對應目前的計畫")
        assert dlg.stale_reason.startswith("計畫已變更") and dlg._default_export_name().endswith("_stale.png")
        dlg._ask_export_path = lambda: str(tmp_path / "old.png")
        answers.append(_QtW.QMessageBox.StandardButton.No)
        dlg._on_export()
        assert "不對應目前的計畫" in qmb["question"][-1][0][2] and not (tmp_path / "old.png").exists()
        answers.append(_QtW.QMessageBox.StandardButton.Yes)
        dlg._on_export()
        data = json.loads((tmp_path / "old.json").read_text(encoding="utf-8"))
        assert data["stale"] is True and data["stale_reason"].startswith("計畫已變更（清除全部）")
        assert any("已不對應目前的計畫" in d for d in data["disclaimer"])
        assert "已標明過期" in qmb["information"][-1][0][2]
    finally:
        dlg.close()


def test_export_json_states_the_per_camera_mb_assumption(tmp_path):
    st = _stats(data_mb_per_photo=[0.95, 1.58], data_mb_basis="本相機 1.58 MP × 0.6–1.0 B/px（未經實測）")
    _png_p, js = hpd.export_preview_files(make_preview(stats=st), _png(), tmp_path / "gs.png")
    data = json.loads(js.read_text(encoding="utf-8"))
    assert "資料量為範圍：假設每張 0.95–1.58 MB（本相機 1.58 MP × 0.6–1.0 B/px（未經實測））。" in data["disclaimer"]


def test_stats_card_has_one_hole_row_when_the_preview_grid_is_the_gate_grid(qt_app):
    st = _stats(interior_hole_count=1, interior_hole_area_m2=356533.0, interior_hole_basis="gate",
                gate_cell_size_m=4.0, cell_size_m=4.0, preview_grid_hole_count=1, preview_only_hole_count=0,
                gate_disagrees=False, gate={"uploadable": False})
    text = _plain(hpd.preview_html(make_preview(stats=st)))
    assert "1 處、356,533 m²（視數 < 5；會擋上傳；預覽格就是閘門格，與上傳閘門同一判定）；建圖閘門目前：不可上傳" in text
    assert "預覽格 4 m 的內部視數不足" not in text and "兩個格距不一致" not in text
