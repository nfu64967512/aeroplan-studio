"""分層風（10／80／120 m）端到端整合（A＋B＋C 接在一起；使用者 2026-10-19：「那就先 10 80 120 這幾層」）。

真的一條路走到底，不建 MainWindow（QtWebEngine 在這裡建不起來）：

    真的 VTOLReconPanel（選「分層」、按「抓氣象網格」）
      → MainWindow._on_wind_grid_requested（假 self，綁真的處理函式）
      → 真的 WindProfileWorker.run()（只把 opener 換成回放錄製的 Open-Meteo 回應、時鐘凍在錄製當下）
      → core.wind_field.fetch_open_meteo_profile／parse_open_meteo_profile
      → MainWindow._on_wind_profile_done → payload（profile_column 在作業區中心）＋寫入規劃風
      → 真的 DualMapWidget（兩張圖是裸實例，送出去的 JS 記下來）
      → 原樣丟進無頭 Chrome 裡**由真範本產生的頁面**：2D＝MapWidget._build_map＋_inject_javascript（本機 Leaflet）、
        3D＝_fill_cesium_template 填好的 main.html（本機 Cesium 1.115）；網路一律擋掉。

錄製資料：tests/fixtures/open_meteo_profile_icon_global_2x3_20261019T06.json（2026-10-19 06:34 UTC 實打 api.open-meteo.com，
無金鑰）是 2×3 格點的多點回應（JSON list），請求範圍 23.40–23.60°N、120.30–120.60°E。這裡把作業區放在「外擴 500 m 後剛好是
那個範圍」的位置，worker 的格點數設成錄製時的 3×2（正式程式是 6×6），⇒ GUI 這條路送出去的請求**逐字等於**錄製時的請求。

鎖住的事（值層級）：
* 請求＝錄製的請求；三層同一個時間步、同一個模式（icon_global）；
* payload：三層高度、剖面柱在作業區中心且 levels ＝ profile_column、2D 預設＝最接近偵察航高 100 m 的層（80／120 等距取低 ⇒ 80）；
* 寫入規劃風＝那一點 80 m 的值，狀態照實寫「規劃高度 100 m 與 80／120 m 一樣近，依規則取較低的 80 m」；
* 2D 頁：只畫一層（prof:80）、剖面表三列（高的在上、畫的那層反白、數字與顏色＝剖面柱的值／speedColor）；換「2D 顯示高度」
  ⇒ 只換那一層；整頁重繪後由 Python 狀態套回；
* 3D 頁：三個引擎風場各在自己的高度、粒子＝依可見面積的自動數 ÷ 3（上限 800）、80 m 實線其餘淡化、圖例只一個區塊
  （模式／非實測／時間＋實線層）、剖面柱 5 個 entity（垂直線、柱頂標題、三層：高度、順風方向、√ 長度、點與箭頭的
  風速顏色、淺色文字「來自」）；換 2D 顯示高度只換實線層；重載後套回；
* 清除：分層風拿掉、單一流動層回來（而且是**寫入後的**規劃風）、飛控即時風箭頭一直都在。

截圖：設 ``WIND_E2E_SHOTS=<資料夾>`` 就存 3D 斜視（看得到疊起來的三層與剖面柱）與 2D 截圖。
要開瀏覽器：設 SAFE_ALLOW_BROWSER=1（沒設時只跑不開瀏覽器的那一段）。
"""
from __future__ import annotations

import datetime
import json
import math
import os
import re
import time
import types
import urllib.parse
import urllib.request
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")

from core import wind_field as wf                                   # noqa: E402
from ui.widgets import wind_layer as wl                             # noqa: E402

try:
    from ui.main_window import MainWindow          # noqa: F401  先載入 QtWebEngine 再建 QApplication
    import ui.workers.wind_grid_worker as wgw
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from PyQt6.QtWidgets import QApplication
    from tests.test_survey_map_layers import _pw, _launch, _REAL_ARGS, CESIUM_DIR, MAIN_HTML  # noqa: F401
    from tests.test_wind_map_pages import _bare2d, _folium_page_html
    from tests.test_hdmap_preview_layer import _route_with_plugin_stubs, _template_values
    from tests.test_wind_profile_maps import _ENTS, _azimuth, _haversine, _ang

    class _NoDel3D(CesiumMapWidget):
        """裸實例（沒跑 QWidget.__init__）回收時 __del__ 會摸 Qt 屬性；測試用這個子類。"""
        def __del__(self):
            pass
    _QT_OK = True
except Exception as _e:                              # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")
needs_browser = pytest.mark.skipif(os.environ.get("SAFE_ALLOW_BROWSER") != "1",
                                   reason="真瀏覽器測試要設 SAFE_ALLOW_BROWSER=1")

FIX = Path(__file__).resolve().parent / "fixtures"
GRID_2X3 = FIX / "open_meteo_profile_icon_global_2x3_20261019T06.json"
#: 錄製 GRID_2X3 時實際送出的查詢參數（與 tests/test_wind_profile.py 的 RECORDED_URL_QUERY 同一份）
RECORDED_URL_QUERY = (
    "latitude=23.40000%2C23.40000%2C23.40000%2C23.60000%2C23.60000%2C23.60000"
    "&longitude=120.30000%2C120.45000%2C120.60000%2C120.30000%2C120.45000%2C120.60000"
    "&hourly=wind_speed_10m%2Cwind_direction_10m%2Cwind_speed_80m%2Cwind_direction_80m"
    "%2Cwind_speed_120m%2Cwind_direction_120m"
    "&wind_speed_unit=ms&timezone=UTC&models=icon_global&forecast_hours=1")
REC_BBOX = (23.40, 120.30, 23.60, 120.60)            # 錄製請求的格點範圍（grid_points 含邊界）
REC_NX, REC_NY = 3, 2
RECORDED_AT = (2026, 10, 19, 6, 34)                  # 錄製當下（UTC）
TIME = "2026-10-19T06:00"
MARGIN_M = 500.0                                     # MainWindow.WIND_GRID_MARGIN_M（下面會核對）
_DLAT = MARGIN_M / 111_320.0
_DLON = MARGIN_M / (111_320.0 * math.cos(math.radians(0.5 * (REC_BBOX[0] + REC_BBOX[2]))))
#: 作業區：外擴 500 m 後剛好是錄製的範圍
AREA = [(REC_BBOX[0] + _DLAT, REC_BBOX[1] + _DLON), (REC_BBOX[2] - _DLAT, REC_BBOX[1] + _DLON),
        (REC_BBOX[2] - _DLAT, REC_BBOX[3] - _DLON), (REC_BBOX[0] + _DLAT, REC_BBOX[3] - _DLON)]
CENTRE = (0.5 * (REC_BBOX[0] + REC_BBOX[2]), 0.5 * (REC_BBOX[1] + REC_BBOX[3]))
ALT_M = 100.0                                        # 偵察航高：與 80／120 等距 ⇒ 取較低的 80 m
LIMIT_MPS = 4.0                                      # 上色門檻：0.7×4 = 2.8 ⇒ 剖面點三層分屬綠／琥珀
PLAN0 = (1.5, 90.0)                                  # 抓之前的規劃風（面板假設）
LIVE = {"dir_from_deg": 272.0, "speed_mps": 3.2, "speed_z_mps": 0.0, "lat": CENTRE[0] + 0.01,
        "lon": CENTRE[1] - 0.01, "alt_rel": 90.0}
GREEN, AMBER, RED = "#2fbf71", "#f5a623", "#d03b3b"
LEGEND = f"風場分層 10／80／120 m（icon_global 預報，非實測，{TIME} UTC）"


def _speed_color(s: float, lim: float) -> str:
    """引擎 AeroWind.speedColor 的獨立寫法：綠 < 0.7·limit、琥珀 < limit、紅 ≥ limit。"""
    return GREEN if s < 0.7 * lim else (AMBER if s < lim else RED)


def _js_round(x: float) -> int:
    """JS Math.round（.5 往上），不是 Python 的銀行家捨入。"""
    return int(math.floor(x + 0.5))


def _calls(js_list, fn: str) -> list:
    """從記下來的 JS 挑出某個頁面函式的呼叫，回傳每一次的引數串列（JSON 解回來）。"""
    out = []
    for js in js_list:
        m = re.fullmatch(re.escape(fn) + r"\((.*)\)", js, flags=re.S)
        if m:
            out.append(json.loads("[" + m.group(1) + "]") if m.group(1) else [])
    return out


# ═══════════════════════════ 假 self（MainWindow 的真處理函式）＋ 裸地圖 widget ═══════════════════════════
_BOUND = ("_wind_state", "_wind_display", "_wind_limit_mps", "_wind_area_bbox", "_on_wind", "_wind_arrow_rows",
          "_push_wind_arrows", "_refresh_wind_status", "_refresh_wind_field", "_apply_wind_options",
          "_on_wind_display_changed", "_on_wind_grid_requested", "_on_wind_grid_done", "_on_wind_grid_failed",
          "_on_wind_grid_clear", "survey_area_polygons", "_wind_loaded", "_wind_near_point", "_apply_wind_to_plan",
          "_wind_note", "_start_wind_profile_fetch", "_wind_profile_heights_of", "_wind_planned_alt_m",
          "_wind_profile_h2d", "_wind_profile_centre", "_wind_profile_payload", "_wind_profile_status",
          "_push_wind_profile", "_on_wind_profile_done", "_on_wind_profile_failed", "_drop_wind_profile",
          "_on_wind_profile_view_changed", "_apply_profile_wind_to_plan", "_profile_plan_not_written",
          "_on_planned_altitude_changed")


class _Page:
    """假 QWebEnginePage：記下 2D widget 送出的 JS（之後原樣丟進真頁面）。"""

    def __init__(self):
        self.js = []

    def runJavaScript(self, js, cb=None):
        self.js.append(js)


class _Win:
    """MainWindow 的假 self：只綁風場相關的真方法；面板是真的；map_widget 是真的 DualMapWidget（兩張圖都是裸實例）。"""

    def __init__(self, panel, dual):
        self.survey_areas, self.corners = [list(AREA)], []
        self.vtol_recon_panel = panel
        self.map_widget = dual
        self.msgs = []
        for name in _BOUND:
            setattr(self, name, types.MethodType(getattr(MainWindow, name), self))
        for k in ("WIND_LIVE_PUSH_INTERVAL_S", "WIND_LIVE_STALE_S", "WIND_PLAN_SYSID", "WIND_GRID_MARGIN_M",
                  "WIND_CWA_SYSID", "WIND_PROFILE_PARTICLES_TOTAL"):
            setattr(self, k, getattr(MainWindow, k))

    def statusBar(self):
        return types.SimpleNamespace(showMessage=lambda m, *a: self.msgs.append(m))


def _base_2d():
    """2D 裸實例：作業區四角、視角在作業區中心（30 km 寬的錄製範圍 ⇒ zoom 12）。"""
    w = _bare2d()
    w.corners = list(AREA)
    w._view_state = {"lat": CENTRE[0], "lon": CENTRE[1], "zoom": 12}
    return w


def _bare3d():
    w = _NoDel3D.__new__(_NoDel3D)
    w._sent = []
    w._js = w._sent.append
    return w


def _main_html(w) -> str:
    """真範本（不讀設定檔、不寫 data/cache、不載 3D 模型），初始視角在作業區中心。"""
    with pytest.MonkeyPatch.context() as mp:
        vals = _template_values(mp, w)
    vals.update({"__PLANE_MODEL_URI__": "", "__COPTER_MODEL_URI__": "", "__VTOL_MODEL_URI__": "",
                 "__INIT_LAT__": str(CENTRE[0]), "__INIT_LON__": str(CENTRE[1])})
    html = w._fill_cesium_template(MAIN_HTML.read_text(encoding="utf-8"), vals)
    assert "/*__WIND_GLUE__*/" not in html and "/*__WIND_RESTORE__*/" not in html
    return html


class _Resp:
    def __init__(self, b):
        self._b = b

    def read(self):
        return self._b

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class _FrozenDT(datetime.datetime):
    """core.wind_field 的時鐘凍在錄製當下（挑「這個小時」那一筆用的）；其餘行為同 datetime。"""

    @classmethod
    def now(cls, tz=None):
        t = cls(*RECORDED_AT, tzinfo=datetime.timezone.utc)
        return t.astimezone(tz) if tz is not None else t.replace(tzinfo=None)


def _replay_worker_class(urls: list):
    """真的 WindProfileWorker：格點數設成錄製時的 3×2、opener 回放錄製回應、start() 同步跑 run()（不開執行緒）。"""

    class _ReplayWorker(wgw.WindProfileWorker):
        def __init__(self, bbox, *, parent=None, **kw):
            assert not kw, f"MainWindow 不該給其他參數：{kw}"

            def opener(url, timeout):
                urls.append((url, timeout))
                return _Resp(GRID_2X3.read_bytes())
            super().__init__(bbox, nx=REC_NX, ny=REC_NY, opener=opener, parent=parent)

        def start(self):
            self.run()

    return _ReplayWorker


# ═══════════════════════════ 整條 GUI 流程（不開瀏覽器）：記下各階段送出的 JS 與重建用的 HTML ═══════════════════════════
@pytest.fixture(scope="module")
def flow():
    app = QApplication.instance() or QApplication([])          # 要留著參照：被回收後再建 widget 會直接 qFatal
    assert MainWindow.WIND_GRID_MARGIN_M == MARGIN_M, "作業區的擺法依賴外擴 500 m"
    panel = VTOLReconPanel()
    panel.rb_by_gsd.setChecked(False)
    panel.altitude_spin.setValue(ALT_M)
    panel.hv_map_max_wind_spin.setValue(LIMIT_MPS)
    panel.hv_wind_speed_spin.setValue(PLAN0[0])
    panel.hv_wind_dir_spin.setValue(PLAN0[1])
    panel.wind_show_plan_check.setChecked(True)
    panel.wind_show_live_check.setChecked(True)
    panel.wind_apply_plan_check.setChecked(True)
    panel.set_corner_count(4)

    w2 = _base_2d()
    w2.custom_page = _Page()
    w2._render_map = lambda: None
    w3 = _bare3d()
    dual = DualMapWidget.__new__(DualMapWidget)
    dual.__dict__.update(map_2d=w2, map_3d=w3)
    win = _Win(panel, dual)
    # 與 MainWindow 建構時同樣的接線（tests/test_wind_profile_gui.py 鎖原始碼）
    panel.wind_display_changed.connect(win._on_wind_display_changed)
    panel.wind_grid_requested.connect(win._on_wind_grid_requested)
    panel.wind_grid_clear_requested.connect(win._on_wind_grid_clear)
    panel.wind_profile_view_changed.connect(win._on_wind_profile_view_changed)
    panel.planned_altitude_changed.connect(win._on_planned_altitude_changed)

    ns = types.SimpleNamespace(app=app, panel=panel, win=win, w2=w2, w3=w3, urls=[], cp={}, html2d={}, html3d={},
                               status={})

    def checkpoint(name):
        ns.cp[name] = (len(w2.custom_page.js), len(w3._sent))
        ns.status[name] = panel.wind_status_label.text()

    # A：規劃風（均勻流動層＋中心箭頭）＋一架飛控即時風
    win._on_wind_display_changed({})
    win._on_wind(1, dict(LIVE, t=time.monotonic()))
    checkpoint("A")

    # B：選「分層」、按「抓氣象網格」⇒ 真 worker 回放錄製回應 ⇒ 處理函式 ⇒ 兩張圖
    panel.wind_grid_height_combo.setCurrentIndex(panel.wind_grid_height_combo.findData("profile"))
    shim = types.SimpleNamespace(datetime=_FrozenDT, timezone=datetime.timezone, timedelta=datetime.timedelta,
                                 date=datetime.date)
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(wgw, "WindProfileWorker", _replay_worker_class(ns.urls))
        mp.setattr(wgw, "WindGridWorker", lambda *a, **k: pytest.fail("分層不該起單層網格 worker"))
        mp.setattr(wf, "datetime", shim)
        mp.setattr(urllib.request, "urlopen", lambda *a, **k: pytest.fail("測試不准連網"))
        assert panel.btn_wind_grid.isEnabled()
        panel.btn_wind_grid.click()
    ns.profile = win._wind_state()["profile"]
    ns.plan_after = (panel.hv_wind_speed_spin.value(), panel.hv_wind_dir_spin.value())
    checkpoint("B")
    ns.html2d["B"] = _folium_page_html(w2)
    ns.html3d["B"] = _main_html(w3)

    # C：面板「2D 顯示高度」換成 120 m ⇒ 只換 2D 那一層
    h2d = panel.wind_profile_h2d_combo
    assert [h2d.itemData(i) for i in range(h2d.count())] == [10.0, 80.0, 120.0]
    assert h2d.currentData() == 80.0, "偵察航高 100 m：與 80／120 等距 ⇒ 預設較低的 80 m"
    h2d.setCurrentIndex(2)
    checkpoint("C")
    ns.html2d["C"] = _folium_page_html(w2)

    # D：清除 ⇒ 分層拿掉、單一流動層（寫入後的規劃風）回來
    assert panel.btn_wind_grid_clear.isEnabled()
    panel.btn_wind_grid_clear.click()
    checkpoint("D")
    ns.msgs = list(win.msgs)
    yield ns


def _seg(ns, which: int, a: str, b: str | None) -> list:
    """第 which 張圖（0＝2D、1＝3D）在 a 之後到 b 為止送出的 JS（a=None ⇒ 從頭）。"""
    lst = ns.w2.custom_page.js if which == 0 else ns.w3._sent
    i = 0 if a is None else ns.cp[a][which]
    j = len(lst) if b is None else ns.cp[b][which]
    return lst[i:j]


# ═══════════════════════════ 不開瀏覽器：請求、payload、規劃風、各階段送出的 JS ═══════════════════════════
def test_the_gui_sends_exactly_the_recorded_request_and_parses_one_model_one_hour(flow):
    assert len(flow.urls) == 1, "按一次只抓一次"
    url, timeout = flow.urls[0]
    assert url.startswith(wf.OPEN_METEO_URL + "?") and timeout == 20.0
    assert urllib.parse.parse_qs(urllib.parse.urlparse(url).query) == urllib.parse.parse_qs(RECORDED_URL_QUERY), \
        "GUI 這條路送出的請求要逐字等於錄製時的請求（否則回放的值對不上座標）"
    p = flow.profile
    assert isinstance(p, wf.WindProfile) and p.heights() == [10.0, 80.0, 120.0]
    assert p.model == "icon_global" and p.time_utc == TIME and {f.time_utc for f in p.layers} == {TIME}
    assert p.bbox == pytest.approx(REC_BBOX, abs=1e-12)
    assert not [n for n in p.notes if "時鐘差" in n], "時鐘凍在錄製當下 ⇒ 取的就是這個小時，沒有退回第一筆"
    assert flow.win._wind_state()["profile"] is None, "最後已清除"


def test_the_payload_both_maps_received_is_the_profile_at_the_area_centre(flow):
    p = flow.profile
    sent3 = _calls(_seg(flow, 1, "A", "B"), "setWindProfile")
    sent2 = _calls(_seg(flow, 0, "A", "B"), "setWindProfile")
    assert len(sent3) == len(sent2) == 1, "整份分層風只送一次（寫入規劃風觸發的重推不重送）"
    q3, q2 = sent3[0][0], sent2[0][0]
    assert q3 == q2 == wl.check_wind_profile(dict(
        layers=[f.to_js() for f in p.layers], label=p.label, height_2d_m=80.0, particles_total=2400,
        limit_mps=LIMIT_MPS, column={"lat": CENTRE[0], "lon": CENTRE[1],
                                     "levels": wf.profile_column(p, CENTRE[0], CENTRE[1])})), \
        "頁面拿到的＝check_wind_profile(三層 to_js＋中心剖面柱)；2D 與 3D 同一份"
    assert q3["heights"] == [10.0, 80.0, 120.0] and q3["height_2d_m"] == 80.0 and q3["height_2d_requested_m"] == 80.0
    assert q3["legend"] == LEGEND and q3["particles_total"] == 2400 and q3["limit_mps"] == LIMIT_MPS
    col = q3["column"]
    assert (col["lat"], col["lon"]) == pytest.approx(CENTRE, abs=1e-12)
    want = wf.profile_column(p, *CENTRE)
    assert [lv["h_m"] for lv in col["levels"]] == [10.0, 80.0, 120.0]
    for got, exp in zip(col["levels"], want):
        assert got["speed_mps"] == exp["speed_mps"] and got["dir_from_deg"] == exp["dir_from_deg"]
    # 每一層的 u／v 就是錄製回應換算的（NaN 不會變 0）
    rec = json.loads(GRID_2X3.read_text(encoding="utf-8"))
    for lay in q3["layers"]:
        h = int(lay["height_m"])
        for k, it in enumerate(rec):
            i, j = divmod(k, REC_NX)
            u, v = wf.dir_to_uv(it["hourly"][f"wind_direction_{h}m"][0], it["hourly"][f"wind_speed_{h}m"][0])
            assert (lay["u"][i][j], lay["v"][i][j]) == (pytest.approx(u, abs=1e-12), pytest.approx(v, abs=1e-12))
    # 送分層前先把上色門檻送過去（粒子、剖面柱、剖面表同一個上限）
    opts = _calls(_seg(flow, 1, "A", "B"), "setWindOptions")
    assert opts and opts[-1][0]["limitMps"] == LIMIT_MPS


def test_the_planner_gets_the_80m_layer_and_the_status_names_model_time_and_layer(flow):
    s, d, used = wf.profile_wind_at(flow.profile, CENTRE[0], CENTRE[1], ALT_M)
    assert used == 80.0, "100 m 與 80／120 等距 ⇒ 較低的 80 m（不內插）"
    assert flow.plan_after == (pytest.approx(round(s, 1)), pytest.approx(round(d))), "面板欄位：風速 1 位小數、風向整數"
    st = flow.status["B"]
    assert st.startswith(f"分層風：Open-Meteo icon_global 預報（模式資料，非實測），{TIME} UTC；"
                         "可用 10／80／120 m（只有這幾層，不內插、不外推）；2D 顯示 80 m")
    assert "已寫入 80 m 的預報風（規劃高度 100 m 與 80／120 m 一樣近，依規則取較低的 80 m）" in st
    assert "⚠" not in st.split("規劃風已寫入：")[1], "錄製資料在中心 80 m 比 120 m 強：等距取低層不用警告"
    assert any("已寫入 80 m 的預報風" in m for m in flow.msgs)
    assert "2D 顯示 120 m" in flow.status["C"]
    assert "分層風：" not in flow.status["D"] and "已寫入 80 m 的預報風" in flow.status["D"], \
        "清掉分層風後規劃風的來源說明留著（規劃器還在用那個值）"


def test_each_stage_sends_only_what_it_should(flow):
    # 2D 高度換了：兩張圖都只收到 setWindProfileView（不重送整份）
    for which in (0, 1):
        seg = _seg(flow, which, "B", "C")
        assert [s.split("(")[0] for s in seg] == ["setWindProfileView"], seg
        (v,) = _calls(seg, "setWindProfileView")[0]
        assert v["height_2d_m"] == 120.0 and v["layers_3d"] is None
    # 清除：先清分層、再推寫入後的規劃風（單一流動層）
    plan = wf.uniform_field(flow.win._wind_area_bbox(), *flow.plan_after)
    for which in (0, 1):
        seg = _seg(flow, which, "C", "D")
        names = [s.split("(")[0] for s in seg]
        assert names[0] == "clearWindProfile" and "setWindField" in names, names
        assert _calls(seg, "setWindField")[-1][0]["label"] == plan.label
    # 飛控即時風箭頭：收到 UAV-1 之後的每一次推送都還有它（分層風不碰它）
    rows = _calls(_seg(flow, 1, None, None), "setWindArrows")
    rows = rows[next(i for i, call in enumerate(rows) if any(r[0] == 1 for r in call[0])):]
    assert len(rows) >= 3, "A 的即時風、B 寫入規劃風後的重推、D 清除後的重推"
    assert rows and all(any(r[0] == 1 for r in call[0]) for call in rows)


# ═══════════════════════════ 真頁面 ═══════════════════════════
@pytest.fixture(scope="module")
def browser(_pw):
    try:
        b = _launch(_pw, _REAL_ARGS)
    except Exception as e:                           # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    yield b
    b.close()


@pytest.fixture(scope="module")
def shots_dir():
    d = os.environ.get("WIND_E2E_SHOTS")
    if not d:
        return None
    p = Path(d)
    p.mkdir(parents=True, exist_ok=True)
    return p


def _open_2d(browser, html: str, path: Path, size=(1100, 800)):
    path.write_text(html, encoding="utf-8")
    pg = browser.new_page(viewport={"width": size[0], "height": size[1]})
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    _route_with_plugin_stubs(pg)
    pg.goto(path.as_uri())
    pg.wait_for_function("window.currentMap && typeof windInfo === 'function' && windInfo().engine", timeout=30000)
    pg._errs = errs
    return pg


def _open_3d(browser, html: str, path: Path, ready: str, size=(1280, 800)):
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    path.write_text(html, encoding="utf-8")
    pg = browser.new_page(viewport={"width": size[0], "height": size[1]})
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())
    pg.goto(path.as_uri())
    try:
        pg.wait_for_function(f"typeof viewer !== 'undefined' && typeof windInfo === 'function' && ({ready})",
                             timeout=60000)
    except Exception as e:                           # pragma: no cover - 沒有 WebGL 等
        pg.close()
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._errs = errs
    return pg


#: 2D：folium 從 CDN 載 jQuery／bootstrap（測試擋網路 ⇒ 沒有 $）；這是頁面本來就有的、與風場無關的錯誤，其餘一律不准
_CDN_ONLY_ERRS = ("$ is not defined",)


def _page_errors(pg) -> list:
    return [e for e in pg._errs if e not in _CDN_ONLY_ERRS]


def _flush(pg, js_list) -> list:
    return [pg.evaluate(js) for js in js_list]


def _check_2d_profile(info, levels, drawn_h: float):
    prof, st = info["profile"], info["state"]
    assert prof["drawn"] == [drawn_h] and prof["fieldIds"] == [wl.profile_field_id(drawn_h)]
    assert set(st["fields"]) == {wl.profile_field_id(drawn_h)}, "2D 俯視只畫一層；單一風場（規劃風）被藏起來"
    assert info["field"] is True, "單一風場仍記著（清掉分層後要畫回來）"
    assert st["arrows"] == 2 and st["arrowLayers"] == {"default": 2}, "飛控即時風＋規劃風中心箭頭不受影響"
    d = prof["decor"]
    assert d["kind"] == "table" and d["present"] is True
    assert [r["h"] for r in d["rows"]] == ["120", "80", "10"], "高的在上面"
    assert [r["drawn"] for r in d["rows"]] == [h == drawn_h for h in (120.0, 80.0, 10.0)]
    by_h = {lv["h_m"]: lv for lv in levels}
    for r, h in zip(d["rows"], (120.0, 80.0, 10.0)):
        lv = by_h[h]
        assert r["color"] == _speed_color(lv["speed_mps"], LIMIT_MPS)
        assert f"{h:g} m" in r["text"] and f"{lv['speed_mps']:.1f} m/s" in r["text"]
        assert f"來自 {_js_round(lv['dir_from_deg']) % 360}°" in r["text"], "數字是來向：要寫「來自」"
    assert f"icon_global 預報，非實測，{TIME} UTC" in d["text"] and "不內插" in d["text"]
    assert f"2D 只畫 {drawn_h:g} m" in d["text"]


@needs_browser
def test_2d_page_live_one_layer_table_view_change_and_clear(browser, flow, tmp_path, shots_dir):
    pg = _open_2d(browser, _folium_page_html(_base_2d()), tmp_path / "before.html")
    try:
        assert all(r is not False for r in _flush(pg, _seg(flow, 0, None, "B")))
        info = pg.evaluate("windInfo()")
        levels = _calls(_seg(flow, 0, "A", "B"), "setWindProfile")[0][0]["column"]["levels"]
        _check_2d_profile(info, levels, 80.0)
        colors = {lv["h_m"]: _speed_color(lv["speed_mps"], LIMIT_MPS) for lv in levels}
        assert colors == {10.0: GREEN, 80.0: AMBER, 120.0: GREEN}, "錄製資料在剖面點：80 m 最快（2.83 m/s ≥ 2.8）"
        f80 = info["state"]["fields"]["prof:80"]
        assert f80["height_m"] == 80.0 and f80["max_speed"] == pytest.approx(flow.profile.layer(80).max_speed())
        pg.wait_for_function("windInfo().state.frames > 40 && windInfo().state.fields['prof:80'].n_alive > 0",
                             timeout=20000)
        if shots_dir is not None:
            pg.screenshot(path=str(shots_dir / "wind_profile_2d.png"))
        # C：換 2D 顯示高度 ⇒ 只換那一層
        _flush(pg, _seg(flow, 0, "B", "C"))
        _check_2d_profile(pg.evaluate("windInfo()"), levels, 120.0)
        # D：清除 ⇒ 表格不見、單一流動層＝寫入後的規劃風；箭頭照舊
        _flush(pg, _seg(flow, 0, "C", "D"))
        info = pg.evaluate("windInfo()")
        plan = wf.uniform_field(flow.win._wind_area_bbox(), *flow.plan_after)
        assert info["profile"] is None and set(info["state"]["fields"]) == {"default"}
        assert info["state"]["label"] == plan.label and info["state"]["arrows"] == 2
        assert pg.evaluate("document.querySelector('.aerowind-profile-table')") is None
        assert not _page_errors(pg), pg._errs
    finally:
        pg.close()


@needs_browser
def test_2d_full_rebuild_replays_the_python_state(browser, flow, tmp_path):
    levels = _calls(_seg(flow, 0, "A", "B"), "setWindProfile")[0][0]["column"]["levels"]
    for stage, h in (("B", 80.0), ("C", 120.0)):
        pg = _open_2d(browser, flow.html2d[stage], tmp_path / f"rebuilt_{stage}.html")
        try:
            pg.wait_for_function("windInfo().profile !== null", timeout=20000)
            _check_2d_profile(pg.evaluate("windInfo()"), levels, h)
            assert not _page_errors(pg), pg._errs
        finally:
            pg.close()


_OBLIQUE = """(c) => {
  const C = Cesium;
  viewer.camera.cancelFlight();
  viewer.camera.lookAt(C.Cartesian3.fromDegrees(c.lon, c.lat, 60),
                       new C.HeadingPitchRange(C.Math.toRadians(c.heading), C.Math.toRadians(c.pitch), c.range));
  viewer.camera.lookAtTransform(C.Matrix4.IDENTITY);
  _requestRender();
  const ST = C.SceneTransforms, f = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
  return [0, 10, 80, 120].map(h => { const w = f.call(ST, viewer.scene, C.Cartesian3.fromDegrees(c.lon, c.lat, h));
                                      return w ? [w.x, w.y] : null; });
}"""


def _check_3d_profile(info, ents, levels, emph=80.0):
    st, prof = info["state"], info["profile"]
    assert set(st["fields"]) == {"prof:10", "prof:80", "prof:120"}, "三層各一個引擎風場；單一風場被藏"
    for h in (10.0, 80.0, 120.0):
        f = st["fields"][wl.profile_field_id(h)]
        assert f["height_m"] == h and f["group"] == "profile" and f["n_requested"] == 800, "每層上限 max(80, 2400 // 3)"
        assert f["share"] == pytest.approx(1 / 3) and f["faint"] is (h != emph), f"{emph:g} m 實線、其餘淡化"
    assert prof["drawn"] == [10.0, 80.0, 120.0] and prof["particles_per_layer"] == 800
    assert prof["emphasis_3d_m"] == emph
    assert info["field"] is True and info["arrows"] == 2
    # 剖面柱：1 條垂直線＋柱頂標題＋每層一個（順風箭頭＋點＋文字）
    assert len(ents) == 5, [e["name"] for e in ents]
    col = next(e for e in ents if "剖面柱" in e["name"])
    assert [round(p[2], 3) for p in col["line"]] == [0.0, 120.0]
    assert all(abs(p[0] - CENTRE[0]) < 1e-9 and abs(p[1] - CENTRE[1]) < 1e-9 for p in col["line"])
    head = next(e for e in ents if "標題" in e["name"])
    assert head["text"] == f"風場剖面（icon_global 預報，非實測，{TIME} UTC）"
    lv_ents = {round(e["pos"][2]): e for e in ents if "pos" in e and "標題" not in e["name"]}
    assert sorted(lv_ents) == [10, 80, 120]
    for lv in levels:
        h, s, d = lv["h_m"], lv["speed_mps"], lv["dir_from_deg"]
        e = lv_ents[int(h)]
        base, tip = e["line"]
        assert e["arrow"] is True and abs(base[2] - h) < 1e-3 and abs(tip[2] - h) < 1e-3, "箭頭水平地畫在自己的高度"
        assert _ang(_azimuth(base, tip), (d + 180.0) % 360.0) < 0.5, f"{h:g} m 箭頭要指向下風處"
        assert _haversine(base, tip) == pytest.approx(min(150.0, max(10.0, 30.0 * math.sqrt(s))), rel=0.01)
        c = _speed_color(s, LIMIT_MPS)
        assert e["pointColor"] == e["lineColor"] == c and e["labelColor"] == "#e8eef5"
        assert e["text"] == f"{h:g} m  {s:.1f} m/s  來自 {_js_round(d) % 360}°"


@needs_browser
def test_3d_page_live_three_heights_column_oblique_view_and_clear(browser, flow, tmp_path, shots_dir):
    pg = _open_3d(browser, _main_html(_bare3d()), tmp_path / "main_before.html", "true")
    try:
        _flush(pg, _seg(flow, 1, None, "B"))
        levels = _calls(_seg(flow, 1, "A", "B"), "setWindProfile")[0][0]["column"]["levels"]
        pg.wait_for_function("windInfo().state.legend.length > 1", timeout=20000)
        info, ents = pg.evaluate("windInfo()"), pg.evaluate(_ENTS)
        _check_3d_profile(info, ents, levels)
        vmax = max(f.max_speed() for f in flow.profile.layers)
        leg = info["state"]["legend"]
        assert leg[0] == f"風場分層 10／80／120 m　最大 {vmax:.1f} m/s"
        assert leg[1] == f"（icon_global 預報，非實測，{TIME} UTC）", "模式、非實測、時間都在，不被截斷"
        assert leg[2] == "實線＝80 m 層，其餘層淡化（各層數值見剖面柱）"
        assert not [ln for ln in leg if "個模式格點" in ln], "不是一層一行"
        # 斜視：三層疊在一起看得出來（剖面柱上的 10／80／120 m 由下往上排、都在畫面裡），每層都有活的粒子
        xy = pg.evaluate(_OBLIQUE, {"lat": CENTRE[0], "lon": CENTRE[1], "heading": 40.0, "pitch": -14.0,
                                    "range": 520.0})
        assert all(p is not None and 0 < p[0] < 1280 and 0 < p[1] < 800 for p in xy), xy
        assert xy[0][1] > xy[1][1] > xy[2][1] > xy[3][1], f"高度越高在畫面越上面：{xy}"
        f0 = pg.evaluate("windInfo().state.frames")
        pg.wait_for_function("(f0) => { const s = windInfo().state, f = s.fields; return s.frames > f0 + 90 && "
                             "['prof:10','prof:80','prof:120'].every(k => f[k] && f[k].n_alive > 0); }",
                             arg=f0, timeout=30000)
        pg.evaluate("_requestRender()")
        if shots_dir is not None:
            pg.wait_for_timeout(600)
            pg.screenshot(path=str(shots_dir / "wind_profile_3d_oblique.png"))
        # C：2D 顯示高度換了 ⇒ 3D 照樣三層，只換哪一層實線：粒子陣列與剖面柱 entity 都沿用（不重生、不重畫）
        pg.evaluate("""() => { window.__pts = ['prof:10','prof:80','prof:120'].map(k => _windLayer.fields.get(k).pt);
                               window.__ents = _windColEnts.slice(); }""")
        _flush(pg, _seg(flow, 1, "B", "C"))
        _check_3d_profile(pg.evaluate("windInfo()"), pg.evaluate(_ENTS), levels, emph=120.0)
        assert pg.evaluate("""() => ['prof:10','prof:80','prof:120'].every((k, i) => _windLayer.fields.get(k).pt === window.__pts[i])
                                    && _windColEnts.every((e, i) => e === window.__ents[i])""")
        # D：清除 ⇒ 剖面柱與三層都拿掉、單一流動層＝寫入後的規劃風、箭頭照舊
        _flush(pg, _seg(flow, 1, "C", "D"))
        info = pg.evaluate("windInfo()")
        plan = wf.uniform_field(flow.win._wind_area_bbox(), *flow.plan_after)
        assert info["profile"] is None and set(info["state"]["fields"]) == {"default"}
        assert info["state"]["label"] == plan.label and info["arrows"] == 2
        assert pg.evaluate(_ENTS) == []
        assert not _page_errors(pg), pg._errs
    finally:
        pg.close()


@needs_browser
def test_3d_reload_replays_the_profile_from_the_template(browser, flow, tmp_path):
    levels = _calls(_seg(flow, 1, "A", "B"), "setWindProfile")[0][0]["column"]["levels"]
    pg = _open_3d(browser, flow.html3d["B"], tmp_path / "main_B.html", "windInfo().profile !== null")
    try:
        _check_3d_profile(pg.evaluate("windInfo()"), pg.evaluate(_ENTS), levels)
        assert pg.evaluate("windInfo().profile.legend") == LEGEND
        assert not _page_errors(pg), pg._errs
    finally:
        pg.close()
