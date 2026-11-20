"""高精地圖「預計成果」端到端整合（A＋B＋C 接在一起）：真的 HARVEST 建圖規劃 → survey.preview → payload →
MainWindow 的處理函式（假 self）→ DualMapWidget → 真的 2D Leaflet／3D Cesium 頁面 → 靜態圖 → 對話框。

不建 MainWindow（QtWebEngine 在這裡建不起來）：處理函式用假 self 跑，地圖 widget 是裸實例，
它們送出去的 JS 原樣丟進無頭 Chrome 裡由真範本產生的頁面（2D：MapWidget._build_map＋_inject_javascript；
3D：_fill_cesium_template 填好的 main.html），網路一律擋掉。

鎖住的事（值層級）：
* 規劃 → 背景 worker → MainWindow._on_hdmap_preview_done 那條路算出來的預覽，跟直接呼叫 compute_hdmap_preview 的
  **同一份**（影像位元組、拍照點一樣）⇒ GUI 用的是規劃協調器同一組有效輸入；
* 頁面上畫出來的：影像範圍＝payload bounds（1e-9 度內）、拍照點數、內部破洞數、邊緣不足數、外框數，2D 與 3D 都一樣；
  2D 整頁重建後由 Python 端狀態套回，數字不變；
* 真資料的方向：從預覽 PNG 挑「上下翻轉／左右翻轉後顏色會不同」的均勻格，截圖讀那個經緯度的像素 ⇒ 等於分級色；
* 重設（清除路徑）與圖層勾選真的把兩張圖上的東西拿掉／藏起來；
* render_preview_map_png：真預覽的長寬比依 cos(緯度) 校正；合成案例把破洞放北邊 ⇒ 紅色像素都在圖的上半；
* 對話框用真預覽建得起來，統計卡有 GSD／張數／規劃預測。

截圖：設 ``HDMAP_E2E_SHOTS=<資料夾>`` 就把 2D／3D／對話框截圖存在那裡（對話框另開 ``QT_QPA_PLATFORM=windows`` 的子行程，
WA_DontShowOnScreen，因為 offscreen 平台沒有中文字型）；沒設就存在 pytest 的 tmp_path、對話框只做 offscreen grab。

要開瀏覽器：用 safe runner 時設 SAFE_ALLOW_BROWSER=1。真的規劃約 20 s（兩份：一般、加一塊沒人能飛的 LiDAR 區 ⇒ 真破洞）。
"""
from __future__ import annotations

import io
import json
import math
import os
import pickle
import subprocess
import sys
import time
import types
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")

pytest.importorskip("hv_dccpp")

try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine 再建 QApplication
    import ui.main_window as mw
    from ui.widgets.map_widget import MapWidget
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from ui.dialogs import hdmap_preview_dialog as hpd
    from PyQt6.QtWidgets import QApplication
    from tests.test_survey_map_layers import _pw, _launch, _REAL_ARGS, CESIUM_DIR, MAIN_HTML  # noqa: F401
    from tests.test_wind_map_pages import _bare2d, _folium_page_html
    from tests.test_hdmap_preview_layer import _route_with_plugin_stubs, _template_values

    class _NoDel3D(CesiumMapWidget):
        """裸實例（沒跑 QWidget.__init__）回收時 __del__ 會摸 Qt 屬性；測試用這個子類。"""
        def __del__(self):
            pass
    _QT_OK = True
except Exception as _e:                              # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

import survey.preview as sp                          # noqa: E402
from ui.widgets import hdmap_preview_layer as hl     # noqa: E402

REPO = Path(__file__).resolve().parent.parent
LID = mw.HDMAP_PREVIEW_LAYER_ID if _QT_OK else "hdmap_plan"


# ═══════════════════════════ 真的規劃（模組共用） ═══════════════════════════
@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(scope="module")
def plans():
    """{'main': 一般建圖計畫（可上傳）, 'hole': 多一塊單光相機下沒人能飛的 LiDAR 區（真的內部破洞）}。"""
    from tests.test_hdmap_gui_coordinator import mapping_params, plan
    return {"main": plan(mapping_params()), "hole": plan(mapping_params(), lidar_area=True)}


@pytest.fixture(scope="module")
def previews(plans):
    out = {}
    for k, sc in plans.items():
        pv = sp.compute_hdmap_preview(sc.hv_coordinator, sc)
        assert pv.ok, pv.reason
        out[k] = pv
    return out


@pytest.fixture(scope="module")
def shots_dir(tmp_path_factory):
    d = os.environ.get("HDMAP_E2E_SHOTS")
    p = Path(d) if d else tmp_path_factory.mktemp("hdmap_shots")
    p.mkdir(parents=True, exist_ok=True)
    return p


def _pump(cond, timeout: float = 60.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    QApplication.processEvents()
    return bool(cond())


# ═══════════════════════════ 假 self（MainWindow 的真處理函式）＋ 裸地圖 widget ═══════════════════════════
class _Page:
    """假 QWebEnginePage：記下 2D widget 送出的 JS（之後原樣丟進真頁面）。"""

    def __init__(self):
        self.js = []

    def runJavaScript(self, js, cb=None):
        self.js.append(js)


_BOUND = ("_start_hdmap_preview", "_hdmap_preview_signal_worker", "_on_hdmap_preview_done",
          "_on_hdmap_preview_failed", "_hdmap_preview_retire_worker", "_hdmap_preview_show_failure",
          "_hdmap_preview_draw", "_on_hdmap_preview_layer_toggled")


class _Win:
    """MainWindow 的假 self：只綁預覽相關的真方法；map_widget 是真的 DualMapWidget（裸實例，兩張圖都是裸實例）。"""

    def __init__(self, sc, panel, mwid):
        self._hv_coordinator = sc.hv_coordinator
        self._dccpp_result = sc.hv_coordinator.last_result
        self._hv_swarm_coordinator = sc
        self.vtol_recon_panel = panel
        self.map_widget = mwid
        self.msgs = []
        for n in _BOUND:
            setattr(self, n, types.MethodType(getattr(MainWindow, n), self))

    def statusBar(self):
        return types.SimpleNamespace(showMessage=lambda m, *a: self.msgs.append(m))


def _maps(center):
    w2 = _bare2d()
    w2.corners = []                                  # 不畫角點多邊形（不染色）
    w2._view_state = {"lat": center[0], "lon": center[1], "zoom": 16}
    w2.custom_page = _Page()
    w2._render_map = lambda: None
    w3 = _NoDel3D.__new__(_NoDel3D)
    w3._sent = []
    w3._js = w3._sent.append
    d = DualMapWidget.__new__(DualMapWidget)
    d.__dict__.update(map_2d=w2, map_3d=w3)
    return d, w2, w3


def _center(pv):
    W, S, E, N = pv.bounds_wsen
    return ((S + N) / 2.0, (W + E) / 2.0)


# ═══════════════════════════ 真頁面 ═══════════════════════════
@pytest.fixture(scope="module")
def browser(_pw):
    try:
        b = _launch(_pw, _REAL_ARGS)
    except Exception as e:                           # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    yield b
    b.close()


def _open_2d(browser, html_path: Path, errs: list):
    pg = browser.new_page(viewport={"width": 1100, "height": 900})
    pg.on("pageerror", lambda e: errs.append(str(e)))
    _route_with_plugin_stubs(pg)
    pg.goto(html_path.as_uri())
    pg.wait_for_function("window.currentMap && typeof setHdmapPreview === 'function'", timeout=30000)
    return pg


def _open_3d(browser, tmp_dir: Path, center, errs: list):
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    w = _NoDel3D.__new__(_NoDel3D)
    w._sent = []
    w._js = w._sent.append
    with pytest.MonkeyPatch.context() as mp:
        vals = _template_values(mp, w)
    vals.update({"__PLANE_MODEL_URI__": "", "__COPTER_MODEL_URI__": "", "__VTOL_MODEL_URI__": "",
                 "__INIT_LAT__": str(center[0]), "__INIT_LON__": str(center[1])})
    html = w._fill_cesium_template(MAIN_HTML.read_text(encoding="utf-8"), vals)
    assert "/*__HDMAP_GLUE__*/" not in html
    f = tmp_dir / "main.html"
    f.write_text(html, encoding="utf-8")
    pg = browser.new_page(viewport={"width": 1100, "height": 900})
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())
    pg.goto(f.as_uri())
    try:
        pg.wait_for_function("typeof viewer !== 'undefined' && typeof hdmapPreviewInfo === 'function'", timeout=60000)
    except Exception as e:                           # pragma: no cover - 沒有 WebGL 等
        pg.close()
        pytest.skip(f"真的 Cesium 起不來：{e}")
    return pg


def _flush(pg, sent: list, start: int) -> list:
    """把 widget 從 start 之後送出的 JS 依序丟進頁面；回傳每一段的結果。"""
    return [pg.evaluate(js) for js in sent[start:]]


def _rgb(pg):
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(pg.screenshot())).convert("RGB")).astype(int)


def _patch_median(img, xy, r: int = 4):
    x, y = int(round(xy[0])), int(round(xy[1]))
    p = img[max(y - r, 0):y + r + 1, max(x - r, 0):x + r + 1].reshape(-1, 3)
    return tuple(int(v) for v in np.median(p, axis=0))


# ═══════════════════════════ 真資料的方向：挑「翻轉後顏色不同」的均勻格 ═══════════════════════════
def _class_map(png: bytes) -> np.ndarray:
    from PIL import Image
    a = np.asarray(Image.open(io.BytesIO(png)).convert("RGBA")).astype(int)
    cls = np.zeros(a.shape[:2], dtype=int)
    for i, (_lo, _hi, col, _lab) in enumerate(sp.QUALITY_CLASSES):
        if col:
            rgb = [int(col[k:k + 2], 16) for k in (1, 3, 5)]
            cls[(a[..., 3] == 255) & (a[..., 0] == rgb[0]) & (a[..., 1] == rgb[1]) & (a[..., 2] == rgb[2])] = i
    return cls


def _class_rgb(i: int):
    col = sp.QUALITY_CLASSES[i][2]
    return tuple(int(col[k:k + 2], 16) for k in (1, 3, 5))


def _orientation_spots(pv, n: int = 2, r: int = 4, survey_clear_m: float = 25.0) -> list:
    """預覽 PNG（北在上）裡挑 n 個格：(2r+1)² 鄰域同一級、上下與左右翻轉後的位置也是均勻的**另一級**（或透明），
    離測區外框（藍虛線）> survey_clear_m。回傳 [(lat, lon, 該級 RGB, 翻轉後的級), …]，北半、南半各挑一個。"""
    from numpy.lib.stride_tricks import sliding_window_view
    cls = _class_map(pv.image_png)
    h, w = cls.shape
    # 影像邊上 r 格內不算均勻（外框白線常沿著影像邊走；邊外是底圖）
    win = sliding_window_view(np.pad(cls, r, mode="constant", constant_values=-1), (2 * r + 1, 2 * r + 1))
    uni = win.max(axis=(2, 3)) == win.min(axis=(2, 3))
    cand = (uni & uni[::-1, :] & uni[:, ::-1] & (cls > 0)
            & (cls != cls[::-1, :]) & (cls != cls[:, ::-1]))
    W, S, E, N = pv.bounds_wsen
    kx, ky = sp._m_per_deg((S + N) / 2.0)
    segs = []
    for ring in pv.survey_rings:
        q = np.asarray(ring, dtype=float)
        q = np.vstack([q, q[:1]])
        segs += [((a[1] * kx, a[0] * ky), (b[1] * kx, b[0] * ky)) for a, b in zip(q[:-1], q[1:])]

    def far_from_survey(lat, lon):
        px, py = lon * kx, lat * ky
        for (x1, y1), (x2, y2) in segs:
            dx, dy = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / max(dx * dx + dy * dy, 1e-12)))
            if math.hypot(px - (x1 + t * dx), py - (y1 + t * dy)) < survey_clear_m:
                return False
        return True

    spots = []
    for half in (range(0, h // 2), range(h // 2, h)):
        ys, xs = np.nonzero(cand[half.start:half.stop])
        order = np.lexsort((xs, ys))
        for j in order:
            row, col = int(ys[j]) + half.start, int(xs[j])
            lat = N - (row + 0.5) / h * (N - S)
            lon = W + (col + 0.5) / w * (E - W)
            exp, flip_v, flip_h = _class_rgb(cls[row, col]), int(cls[h - 1 - row, col]), int(cls[row, w - 1 - col])
            # 翻轉後那一級的顏色要跟預期差得夠遠（透明 = 背景，一定差很遠）⇒ 這個點量得出方向
            if any(f > 0 and max(abs(a - b) for a, b in zip(exp, _class_rgb(f))) < 60 for f in (flip_v, flip_h)):
                continue
            if far_from_survey(lat, lon):
                spots.append((lat, lon, exp, (flip_v, flip_h)))
                break
    assert len(spots) == n, f"挑不到量得出方向的格（候選 {int(cand.sum())}）"
    return spots


# ═══════════════════════════ 1. 預覽與上傳閘門、payload 合約 ═══════════════════════════
def test_real_previews_follow_the_gate_and_the_payload_contract(plans, previews):
    for key, pv in previews.items():
        sc = plans[key]
        st = pv.stats
        assert st["k_product"] == sp.K_PRODUCT == 5
        gs = sc.hv_exposure_summary
        assert gs and gs.get("ok") is True
        # 預覽格 2 m、閘門格 4 m：同一份輸入 ⇒ 閘門的破洞數與預覽記下的閘門結果一致
        assert st["gate"]["interior_hole_count"] == gs["interior_hole_count"]
        assert st["gate"]["uploadable"] is sc.hv_mapping_gate["uploadable"]
        assert st["cell_size_m"] == 2.0 and st["gate_cell_size_m"] == 4.0
        assert pv.hillshade_png is None and st["terrain_is_world_terrain"] is False   # 平地 DEM：沒有山影
        p = pv.to_js_payload()
        json.dumps(p, allow_nan=False)
        q = hl.check_payload(p, LID)
        s = hl.summary(q)
        assert (s["points"], s["holes"], s["edge_short"], s["extent_rings"], s["survey_rings"]) == (
            len(pv.photos), len(pv.interior_holes), len(pv.edge_short), len(pv.extent_rings), len(pv.survey_rings))
        assert q["bounds"] == list(pv.bounds_wsen)
    main, hole = previews["main"], previews["hole"]
    assert main.stats["interior_hole_count"] == 0 and plans["main"].hv_mapping_gate["uploadable"] is True
    assert hole.stats["interior_hole_count"] == len(hole.interior_holes) == 1
    assert plans["hole"].hv_mapping_gate["uploadable"] is False
    assert hole.interior_holes[0]["area_m2"] > 3.0e5, "整塊 LiDAR 區（600×600 m 扣半刈幅邊緣帶）都沒拍到"


# ═══════════════════════════ 2. 規劃 → worker → MainWindow → Dual → 真頁面 ═══════════════════════════
def test_mainwindow_path_draws_the_same_preview_on_real_2d_and_3d_pages(qt_app, plans, previews, browser,
                                                                      tmp_path, shots_dir):
    sc, pv0 = plans["main"], previews["main"]
    center = _center(pv0)
    dual, w2, w3 = _maps(center)
    panel = VTOLReconPanel()
    win = _Win(sc, panel, dual)

    # 2D／3D 頁面先開（空的），之後 widget 送的 JS 原樣丟進去 —— 跟 QWebEngine 收到的是同一串
    errs2, errs3 = [], []
    f2 = tmp_path / "folium.html"
    f2.write_text(_folium_page_html(w2), encoding="utf-8")
    pg2 = _open_2d(browser, f2, errs2)
    pg3 = _open_3d(browser, tmp_path, center, errs3)
    try:
        n2, n3 = len(w2.custom_page.js), len(w3._sent)
        # ── 真的處理函式：開始（背景執行緒、真的 compute）→ 完成 → 畫圖層 ──
        win._start_hdmap_preview(sc.hv_coordinator)
        assert panel.hdmap_preview_state() == "computing"
        assert _pump(lambda: panel.hdmap_preview_state() in ("ready", "failed"), 120)
        assert panel.hdmap_preview_state() == "ready", panel.hdmap_preview_status_label.text()
        assert panel.btn_hdmap_preview.isEnabled()
        pv = win._hdmap_preview
        assert pv is not pv0 and pv.ok
        # GUI 這條路跟直接呼叫是同一份輸入 ⇒ 同一份結果
        assert pv.image_png == pv0.image_png and pv.photos == pv0.photos and pv.bounds_wsen == pv0.bounds_wsen
        assert win._hdmap_preview_drawn is True and win.msgs and "規劃預測" in win.msgs[-1]
        payload = win._hdmap_preview_payload
        assert payload == pv0.to_js_payload()
        assert w2._hdmap_previews[LID]["bounds"] == payload["bounds"]

        out2, out3 = _flush(pg2, w2.custom_page.js, n2), _flush(pg3, w3._sent, n3)
        assert out2 and out2[0]["ok"] is True and out3 and out3[0]["ok"] is True
        pg3.wait_for_function(f"hdmapPreviewInfo().layers[{json.dumps(LID)}].state === 'ready'", timeout=60000)

        want = (len(payload["points"]), len(payload["holes"]), len(payload["edge_short"]),
                len(payload["extent_rings"]), len(payload["survey_rings"]))
        assert want == (len(pv.photos), 0, len(pv.edge_short), 1, 1) and want[0] > 1000
        L2 = pg2.evaluate("hdmapPreviewInfo()")["layers"][LID]
        assert L2["onMap"] and L2["image"] and L2["mask"] and L2["legendShown"]
        assert L2["imageBounds"] == pytest.approx(payload["bounds"], abs=1e-9)
        assert (L2["points"], L2["holes"], L2["edgeShort"], L2["extent"], L2["survey"]) == want
        assert L2["hits"] == want[1] + want[2]
        L3 = pg3.evaluate("hdmapPreviewInfo()")["layers"][LID]
        assert L3["shown"] and L3["imagery"] and L3["imageryShow"] and L3["nearest"] and L3["legendShown"]
        assert L3["rect"] == pytest.approx(payload["bounds"], abs=1e-9)
        assert (L3["points"], L3["holes"], L3["edgeShort"], L3["extent"], L3["survey"]) == want
        assert L3["pointsInScene"] and L3["mask"]
        # 圖例：固定的橫幅＋survey.preview 的標題（說顏色是什麼）＋每架點色（標籤＝uav_labels）
        leg2 = pg2.evaluate("document.querySelector('.hdmap-preview-legend').textContent")
        leg3 = pg3.evaluate("document.querySelector('#hdmapLegendBox .hdmap-preview-legend').textContent")
        for leg in (leg2, leg3):
            assert hl.PREDICTION_BANNER in leg and pv.legend["title"] in leg and "每格預計被幾張照片拍到" in leg
            assert all(lab in leg for lab in pv.stats["uav_labels"])
            assert leg.count(f"{sp.PREVIEW_LABEL}——") == 0, "圖例標題不要再把上面兩行重複一次"

        # ── 截圖（給人看：預設透明度、範圍外變暗）──
        pg2.evaluate("(b) => { window.currentMap.fitBounds([[b[1], b[0]], [b[3], b[2]]], {animate: false, padding: [30, 30]}); }",
                     payload["bounds"])
        pg2.wait_for_timeout(400)
        pg2.screenshot(path=str(shots_dir / "hdmap_preview_2d.png"))
        _shot_3d_overview(pg3, payload["bounds"], shots_dir / "hdmap_preview_3d.png")

        # ── 圖層勾選（偵察頁）：取消 ⇒ 兩邊都藏起來、資料留著；再勾 ⇒ 回來 ──
        n2, n3 = len(w2.custom_page.js), len(w3._sent)
        panel.hdmap_preview_layer_toggled.connect(win._on_hdmap_preview_layer_toggled)
        panel.chk_hdmap_preview_layer.setChecked(False)
        _flush(pg2, w2.custom_page.js, n2), _flush(pg3, w3._sent, n3)
        L2 = pg2.evaluate("hdmapPreviewInfo()")["layers"][LID]
        L3 = pg3.evaluate("hdmapPreviewInfo()")["layers"][LID]
        assert (L2["onMap"], L2["points"], L2["legendShown"]) == (False, 0, False)
        assert (L3["shown"], L3["imageryShow"], L3["pointsShown"], L3["legendShown"]) == (False, False, False, False)
        n2, n3 = len(w2.custom_page.js), len(w3._sent)
        panel.chk_hdmap_preview_layer.setChecked(True)
        _flush(pg2, w2.custom_page.js, n2), _flush(pg3, w3._sent, n3)
        assert pg2.evaluate("hdmapPreviewInfo()")["layers"][LID]["points"] == want[0]
        assert pg3.evaluate("hdmapPreviewInfo()")["layers"][LID]["shown"] is True

        # ── 2D 整頁重建（例如角點變動）：由 Python 端狀態套回，數字不變 ──
        f2b = tmp_path / "folium_rebuilt.html"
        f2b.write_text(_folium_page_html(w2), encoding="utf-8")
        errs2b = []
        pg2b = _open_2d(browser, f2b, errs2b)
        try:
            Lb = pg2b.evaluate("hdmapPreviewInfo()")["layers"][LID]
            assert Lb["imageBounds"] == pytest.approx(payload["bounds"], abs=1e-9)
            assert (Lb["points"], Lb["holes"], Lb["edgeShort"], Lb["extent"], Lb["survey"]) == want
            assert not errs2b, errs2b
        finally:
            pg2b.close()

        # ── 重設（清除路徑）：兩張圖都清掉、面板回 'none' ──
        n2, n3 = len(w2.custom_page.js), len(w3._sent)
        mw.hdmap_preview_reset(win, "清除路徑")
        assert panel.hdmap_preview_state() == "none" and win._hdmap_preview is None
        assert _flush(pg2, w2.custom_page.js, n2) == [True] and _flush(pg3, w3._sent, n3) == [True]
        assert pg2.evaluate("hdmapPreviewInfo().ids") == [] and pg3.evaluate("hdmapPreviewInfo().ids") == []
        assert pg2.evaluate("document.querySelectorAll('.hdmap-preview-legend').length") == 0
        assert pg3.evaluate("viewer.entities.values.filter(e => e._hdmapPreview).length") == 0
        assert w2._hdmap_previews == {} and w3._hdmap_previews == {}
        assert not errs2, errs2
        assert not errs3, errs3
    finally:
        # 結果收到時 run() 可能還沒 return ⇒ worker 被寄放；等它真的結束（在還在跑的 QThread 上解構是 qFatal）
        for pw_ in [win.__dict__.get("_hdmap_preview_worker"), *list(mw._HV_ORPHAN_WORKERS)]:
            if pw_ is not None:
                pw_.wait(60000)
        pg2.close()
        pg3.close()


_OVERVIEW_3D = """async (b) => {
  const pad = 0.15 * Math.max(b[2] - b[0], b[3] - b[1]);
  viewer.camera.setView({destination: Cesium.Rectangle.fromDegrees(b[0] - pad, b[1] - pad, b[2] + pad, b[3] + pad)});
  const frame = () => new Promise(r => requestAnimationFrame(() => r()));
  let ok = false;
  for (let i = 0; i < 600; i++) {
    viewer.scene.requestRender();
    await frame();
    const dsReady = viewer.dataSourceDisplay.update(viewer.clock.currentTime);
    if (i > 40 && dsReady && viewer.scene.globe.tilesLoaded) { ok = true; break; }
  }
  for (let i = 0; i < 10; i++) { viewer.scene.requestRender(); await frame(); }
  return ok;
}"""


def _shot_3d_overview(pg, bounds, path: Path):
    pg.evaluate(_OVERVIEW_3D, bounds)
    pg.screenshot(path=str(path))


# ═══════════════════════════ 3. 真資料的方向（截圖讀像素）＋ 真破洞 ═══════════════════════════
_SPOT_3D = """async (ll) => {
  viewer.camera.setView({destination: Cesium.Cartesian3.fromDegrees(ll[1], ll[0], 250),
                         orientation: {heading: 0, pitch: -Math.PI / 2, roll: 0}});
  const frame = () => new Promise(r => requestAnimationFrame(() => r()));
  let ok = false;
  for (let i = 0; i < 600; i++) {
    viewer.scene.requestRender();
    await frame();
    const dsReady = viewer.dataSourceDisplay.update(viewer.clock.currentTime);
    if (i > 40 && dsReady && viewer.scene.globe.tilesLoaded) { ok = true; break; }
  }
  for (let i = 0; i < 10; i++) { viewer.scene.requestRender(); await frame(); }
  const ST = Cesium.SceneTransforms, toWin = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
  const w = toWin.call(ST, viewer.scene, Cesium.Cartesian3.fromDegrees(ll[1], ll[0], 0));
  return {ok, px: [w.x, w.y]};
}"""
_HIDE_3D = ("['layerPanel', 'toolbar', 'hdmapLegendBox', 'legend', 'modeLabel'].forEach(id => {"
            " const e = document.getElementById(id); if (e) e.style.visibility = 'hidden'; })")
_HIDE_2D = ("document.querySelectorAll('.usage-hint, #corner-counter, .leaflet-control-container')"
            ".forEach(e => { e.style.visibility = 'hidden'; })")


def test_real_preview_is_north_up_on_both_pages_and_the_real_hole_is_drawn(qt_app, previews, browser, tmp_path):
    pv = previews["main"]
    spots = _orientation_spots(pv)
    center = _center(pv)
    _dual, w2, w3 = _maps(center)
    # 讀像素用：不透明、範圍外不變暗（只量方向與位置；畫法本身 B 的測試已鎖）
    r2 = MapWidget.set_hdmap_preview(w2, LID, pv.to_js_payload(opacity=1.0, mask_alpha=0.0))
    r3 = CesiumMapWidget.set_hdmap_preview(w3, LID, pv.to_js_payload(opacity=1.0, mask_alpha=0.0))
    assert r2["ok"] and r3["ok"]
    errs2, errs3 = [], []
    f2 = tmp_path / "folium.html"
    f2.write_text(_folium_page_html(w2), encoding="utf-8")           # 2D：整頁重建套回這一層
    pg2 = _open_2d(browser, f2, errs2)
    pg3 = _open_3d(browser, tmp_path, center, errs3)
    try:
        assert pg3.evaluate(w3._sent[-1])["ok"] is True
        pg3.wait_for_function(f"hdmapPreviewInfo().layers[{json.dumps(LID)}].state === 'ready'", timeout=60000)
        pg2.evaluate(_HIDE_2D)
        pg3.evaluate(_HIDE_3D)
        for lat, lon, want, flips in spots:
            xy = pg2.evaluate("""(ll) => { const m = window.currentMap; m.setView(ll, 18, {animate: false});
              const p = m.latLngToContainerPoint(ll), c = m.getContainer().getBoundingClientRect();
              return [p.x + c.left, p.y + c.top]; }""", [lat, lon])
            pg2.wait_for_timeout(250)
            got = _patch_median(_rgb(pg2), xy)
            assert got == pytest.approx(want, abs=12), f"2D ({lat:.6f}, {lon:.6f})：{got} ≠ {want}（翻轉後的級 {flips}）"
            r = pg3.evaluate(_SPOT_3D, [lat, lon])
            assert r["ok"], "3D 圖磚一直沒載好"
            got = _patch_median(_rgb(pg3), r["px"])
            if max(got) < 8:                                         # pragma: no cover - 沒有 WebGL 輸出
                pytest.skip(f"3D 畫面讀不到像素（WebGL 不可用？）：{got}")
            assert got == pytest.approx(want, abs=25), f"3D ({lat:.6f}, {lon:.6f})：{got} ≠ {want}（翻轉後的級 {flips}）"

        # ── 真破洞：多一塊單光相機下沒人能飛的 LiDAR 區 ⇒ 兩邊都畫 1 個紅框、2D 掛得上說明 ──
        hp = previews["hole"].to_js_payload()
        n2, n3 = len(w2.custom_page.js), len(w3._sent)
        assert MapWidget.set_hdmap_preview(w2, LID, hp)["ok"] and CesiumMapWidget.set_hdmap_preview(w3, LID, hp)["ok"]
        _flush(pg2, w2.custom_page.js, n2)
        _flush(pg3, w3._sent, n3)
        pg3.wait_for_function(f"hdmapPreviewInfo().layers[{json.dumps(LID)}].state === 'ready'", timeout=60000)
        want = (len(hp["points"]), 1, len(hp["edge_short"]), len(hp["survey_rings"]))
        assert want[3] == 2
        L2 = pg2.evaluate("hdmapPreviewInfo()")["layers"][LID]
        L3 = pg3.evaluate("hdmapPreviewInfo()")["layers"][LID]
        assert (L2["points"], L2["holes"], L2["edgeShort"], L2["survey"]) == want
        assert (L3["points"], L3["holes"], L3["edgeShort"], L3["survey"]) == want
        assert L2["imageBounds"] == pytest.approx(hp["bounds"], abs=1e-9)
        assert L3["rect"] == pytest.approx(hp["bounds"], abs=1e-9)
        names = pg3.evaluate("viewer.entities.values.filter(e => e._hdmapPreview).map(e => e.name)")
        assert hp["holes"][0]["label"] in names and "會擋上傳" in hp["holes"][0]["label"]
        assert not errs2, errs2
        assert not errs3, errs3
    finally:
        pg2.close()
        pg3.close()


# ═══════════════════════════ 4. 靜態圖（matplotlib） ═══════════════════════════
def _png_rgb(png: bytes) -> np.ndarray:
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(png)).convert("RGB")).astype(int)


def test_static_map_of_the_real_preview_keeps_the_ground_aspect(previews, shots_dir):
    pv = previews["main"]
    png = sp.render_preview_map_png(pv, width_px=1100)
    img = _png_rgb(png)
    h, w = img.shape[:2]
    W, S, E, N = pv.bounds_wsen
    kx, ky = sp._m_per_deg((S + N) / 2.0)
    ratio = ((N - S) * ky) / ((E - W) * kx)                        # 地面上的高寬比（cos 緯度校正後）
    assert w == 1100 and abs(h - 1100 * ratio) <= 2, (w, h, ratio)
    assert len(np.unique(img.reshape(-1, 3), axis=0)) > 20             # 不是一張空白圖
    (shots_dir / "hdmap_preview_static_map.png").write_bytes(png)


def _synthetic_preview(with_hole: bool):
    from PIL import Image
    W, S, E, N = 121.20, 25.050, 121.21, 25.058
    buf = io.BytesIO()
    Image.new("RGBA", (20, 16), (67, 160, 71, 255)).save(buf, "PNG")  # 全部 ≥9 張（綠）
    box = [[S, W], [S, E], [N, E], [N, W]]
    hole = [[25.0565, 121.2045], [25.0565, 121.2055], [25.0575, 121.2055], [25.0575, 121.2045]]   # 北邊、東西置中
    return sp.HdmapPreview(ok=True, bounds_wsen=(W, S, E, N), image_png=buf.getvalue(), image_px=(20, 16),
                           cell_size_m=50.0, extent_rings=[box], survey_rings=[box],
                           interior_holes=([{"ring": hole, "area_m2": 1e4, "min_views": 0}] if with_hole else []))


def test_static_map_is_north_up_a_hole_in_the_north_is_drawn_in_the_top_half():
    a = _png_rgb(sp.render_preview_map_png(_synthetic_preview(True), width_px=800))
    b = _png_rgb(sp.render_preview_map_png(_synthetic_preview(False), width_px=800))
    assert a.shape == b.shape
    red = (a[..., 0] > 150) & (a[..., 1] < 110) & (a[..., 2] < 110)
    new_red = red & ~((b[..., 0] > 150) & (b[..., 1] < 110) & (b[..., 2] < 110))   # 圖例裡本來就有紅色方塊
    ys, xs = np.nonzero(new_red)
    assert len(ys) > 50, "破洞沒畫出來"
    h, w = red.shape
    assert ys.max() < h / 2, f"破洞在北邊卻畫到下半部（y 最大 {ys.max()} / 高 {h}）⇒ 南北翻了"
    assert w * 0.35 < xs.mean() < w * 0.65, f"破洞東西置中卻在 x 平均 {xs.mean():.0f} / 寬 {w}"
    # 位置量化：破洞中心緯度 25.057 在 [S−3%, N+3%] 裡的比例 ⇒ 圖上 y 的比例（軸佔整張圖 98%，上下各 1%）
    S, N = 25.050, 25.058
    pad = 0.03 * (N - S)
    frac_from_top = ((N + pad) - 25.057) / ((N + pad) - (S - pad))
    y_expect = 0.01 * h + frac_from_top * 0.98 * h
    assert abs(ys.mean() - y_expect) < 0.04 * h, (ys.mean(), y_expect)


# ═══════════════════════════ 5. 對話框（真預覽） ═══════════════════════════
def _plain(html_text: str) -> str:
    from PyQt6.QtGui import QTextDocument
    d = QTextDocument()
    d.setHtml(html_text)
    return d.toPlainText()


def test_dialog_builds_from_the_real_preview_and_states_it_is_a_prediction(qt_app, previews, tmp_path, shots_dir):
    pv = previews["main"]
    dlg = hpd.HdmapPreviewDialog(pv)
    try:
        assert not dlg.isModal() and "規劃預測" in dlg.windowTitle()
        text = dlg.stats_browser.toPlainText()
        html_text = _plain(hpd.preview_html(pv))
        assert text == html_text
        for s in ("GSD", "張", "規劃預測，非實際成果", sp.ACCURACY_NOTE, "7.4–12.3 MB",
                  f"{pv.stats['photos_total']} 張", f"{pv.stats['gsd_cm']['median']:.2f} cm"):
            assert s in text, s
        for a in pv.stats["assumptions"]:
            assert a in text
        png = sp.render_preview_map_png(pv)
        from PIL import Image
        with Image.open(io.BytesIO(png)) as im:
            assert dlg.map_label.source_size() == im.size
        assert dlg.btn_export.isEnabled()
        png_p, js_p = hpd.export_preview_files(pv, dlg.map_png, tmp_path / "e2e.png")
        data = json.loads(js_p.read_text(encoding="utf-8"))
        assert data["stats"]["photos_total"] == pv.stats["photos_total"] and data["terrain_shading_in_png"] is False
        assert "base64" not in js_p.read_text(encoding="utf-8")
        # offscreen grab：至少畫得出來（中文字型在 offscreen 平台可能是方塊；要看字用下面的 windows 平台截圖）
        dlg.resize(1400, 860)
        assert not dlg.grab().isNull()
    finally:
        dlg.close()

    if not os.environ.get("HDMAP_E2E_SHOTS") or sys.platform != "win32":
        return
    # 給人看的對話框截圖：windows 平台＋WA_DontShowOnScreen（不出現在螢幕上），子行程跑（本行程是 offscreen）
    pk = tmp_path / "pv.pkl"
    pk.write_bytes(pickle.dumps((pv, png)))
    out = shots_dir / "hdmap_preview_dialog.png"
    code = ("import sys, pickle\n"
            "from PyQt6.QtWidgets import QApplication\n"
            "from PyQt6.QtCore import Qt\n"
            "from ui.dialogs.hdmap_preview_dialog import HdmapPreviewDialog\n"   # 先 import（會帶到 QtWebEngine）
            "app = QApplication(sys.argv[:1])\n"
            "pv, png = pickle.loads(open(sys.argv[1], 'rb').read())\n"
            "dlg = HdmapPreviewDialog(pv, map_png=png)\n"
            "dlg.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)\n"
            "dlg.resize(1400, 860)\n"
            "dlg.show()\n"
            "for _ in range(30): app.processEvents()\n"
            "ok = dlg.grab().save(sys.argv[2], 'PNG')\n"
            "dlg.close()\n"
            "print('saved' if ok else 'failed')\n")
    env = dict(os.environ, QT_QPA_PLATFORM="windows", AEROPLAN_NO_FILE_LOG="1", PYTHONIOENCODING="utf-8")
    r = subprocess.run([sys.executable, "-c", code, str(pk), str(out)], cwd=str(REPO), env=env,
                       capture_output=True, text=True, timeout=120)
    assert "saved" in r.stdout, (r.stdout, r.stderr[-2000:])
    assert out.stat().st_size > 10_000
