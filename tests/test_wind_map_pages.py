"""風場圖層整合（真的瀏覽器）：2D folium 頁面（本機 Leaflet）與 3D Cesium 頁面（本機 Cesium 1.115）
各自載入引擎 ui/widgets/web_shared/wind_flow.js ＋ ui.widgets.wind_layer 的膠水，畫出風場後讀 windInfo()
與引擎 canvas 的**像素**（不是只看函式有沒有被呼叫）。

引擎本身的行為（粒子平流、箭頭幾何、圖例、選項驗證）由 tests/test_wind_flow_js.py 鎖；這裡鎖的是兩個頁面的
接法：引擎載得進來、膠水的 adapter 投影／viewKey 對、Python 端記住的狀態在頁面重建時真的套回去。
要開瀏覽器：用 safe runner 時要設 SAFE_ALLOW_BROWSER=1，否則整檔 skip。
"""
from __future__ import annotations

import io
import json
import os
import pathlib
import re

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from core import wind_field as wf
from ui.widgets import wind_layer as wl
from tests.test_survey_map_layers import _pw, _launch, _REAL_ARGS, CESIUM_DIR, MAIN_HTML, LAT0, LON0  # noqa: F401

try:
    from ui.widgets.map_widget import MapWidget
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = [pytest.mark.skipif(not wl.WIND_FLOW_JS.is_file(), reason="引擎 wind_flow.js 還沒有"),
              pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")]

AREA = [(LAT0, LON0), (LAT0 + 0.005, LON0), (LAT0 + 0.005, LON0 + 0.008), (LAT0, LON0 + 0.008)]
BBOX = wf.area_bbox([AREA], margin_m=300.0)
FIELD = wf.uniform_field(BBOX, 3.0, 270.0)
ARROW = [1, LAT0 + 0.0025, LON0 + 0.004, 90.0, 270.0, 3.2, "UAV-1 即時風"]

_PIXELS = """() => {
  const host = %s;
  const cvs = Array.from(host.children).filter(e => e.tagName === 'CANVAS');
  const cv = cvs[cvs.length - 1];
  const g = cv.getContext('2d');
  const d = g.getImageData(0, 0, cv.width, cv.height).data;
  let n = 0, nc = 0;
  const x0 = cv.width * 0.25, x1 = cv.width * 0.75, y0 = cv.height * 0.25, y1 = cv.height * 0.75;
  for (let y = 0; y < cv.height; y++) for (let x = 0; x < cv.width; x++) {
    if (d[(y * cv.width + x) * 4 + 3] > 0) { n++; if (x > x0 && x < x1 && y > y0 && y < y1) nc++; }
  }
  const cs = getComputedStyle(cv);
  return {n, nc, w: cv.width, h: cv.height, canvases: cvs.length, z: cs.zIndex, pe: cs.pointerEvents, pos: cs.position};
}"""


def _route_local_only(pg):
    pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())


# ═══════════════════════════════════════════════════════════════════════
#  2D：folium 產生的整頁（本機 Leaflet）＋ MapWidget 注入的引擎與膠水
# ═══════════════════════════════════════════════════════════════════════
def _bare2d():
    w = MapWidget.__new__(MapWidget)
    for k, v in dict(corners=list(AREA), markers=[], paths=[], path_colors=[], _path_tooltips=[],
                     transit_paths=[], _circle_center=None, _circle_radius_m=0.0, _home_point=None,
                     _nfz_zones=[], _geofence=None, _swarm_data=None, _view_state=None,
                     _ortho_tiles=[], _photo_pose_layers={}, _survey_point_layers={},
                     _wind_field=None, _wind_arrows={}, _wind_options={}).items():
        setattr(w, k, v)
    return w


def _folium_page_html(w) -> str:
    """與 MapWidget._render_map 同一條路：save() → 本機 Leaflet → 注入互動 JS（含風場區塊）。"""
    m = w._build_map()
    buf = io.BytesIO()
    m.save(buf, close_file=False)
    html = buf.getvalue().decode("utf-8")
    html = MapWidget._rewrite_leaflet_to_local(w, html)
    var = re.search(r"var\s+(map_[a-f0-9]+)\s*=\s*L\.map", html).group(1)
    return MapWidget._inject_javascript(w, html, var)


@pytest.fixture(scope="module")
def page2d(_pw, tmp_path_factory):
    w = _bare2d()
    MapWidget.set_wind_field(w, FIELD)                        # 建頁前就有狀態 ⇒ 頁面載入時要自己套回去
    MapWidget.set_wind_options(w, limitMps=3.0)
    f = tmp_path_factory.mktemp("f2d") / "folium.html"
    f.write_text(_folium_page_html(w), encoding="utf-8")
    try:
        b = _launch(_pw, ["--allow-file-access-from-files"])
        pg = b.new_page(viewport={"width": 800, "height": 600})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        _route_local_only(pg)
        pg.goto(f.as_uri())
        pg.wait_for_function("window.currentMap && typeof windInfo === 'function'", timeout=30000)
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    pg._errs = errs
    yield pg
    b.close()


def test_2d_page_loads_the_engine_and_replays_the_field_saved_in_python(page2d):
    info = page2d.evaluate("windInfo()")
    assert info["engine"] is True and info["layer"] is True and info["field"] is True
    assert info["options"]["limitMps"] == 3.0 and info["state"]["field"] is True
    assert info["state"]["label"] == FIELD.label and info["state"]["maxSpeed"] == pytest.approx(3.0)
    # 本頁的錯誤只允許是被擋掉的 CDN（folium 的外掛）；風場相關不可以有
    assert not [e for e in page2d._errs if "Wind" in e or "AeroWind" in e], page2d._errs


def test_2d_particles_are_drawn_in_the_middle_of_the_map_on_a_non_interactive_canvas(page2d):
    page2d.wait_for_function("windInfo().state.frames > 40 && windInfo().state.particles > 0", timeout=20000)
    px = page2d.evaluate(_PIXELS % "window.currentMap._container")
    assert px["canvases"] == 1, "引擎在 host 裡只能加一個 canvas"
    assert px["pe"] == "none" and px["pos"] == "absolute" and px["z"] == "450"
    assert px["nc"] > 50, f"地圖中央沒有粒子軌跡像素：{px}"


def test_2d_view_key_changes_when_the_map_pans_and_arrows_follow_the_api(page2d):
    k0 = page2d.evaluate("windInfo().state.viewKey")
    page2d.evaluate("window.currentMap.panBy([80, 0], {animate: false})")
    page2d.wait_for_function(f"windInfo().state.viewKey !== {json.dumps(k0)}", timeout=5000)
    assert page2d.evaluate("(r) => setWindArrows([r])", ARROW) is True
    page2d.wait_for_function("windInfo().state.arrows === 1", timeout=5000)
    st = page2d.evaluate("windInfo().state")
    assert st["arrows"] == 1 and st["arrowLayers"] == {"default": 1}
    assert page2d.evaluate("clearWindArrows()") is True and page2d.evaluate("windInfo().state.arrows") == 0
    assert page2d.evaluate("clearWindField()") is True
    assert page2d.evaluate("windInfo()")["field"] is False and page2d.evaluate("windInfo().state.field") is False
    assert page2d.evaluate("(f) => setWindField(f)", FIELD.to_js()) is True


def test_2d_without_the_engine_the_page_functions_say_so_instead_of_throwing(_pw, tmp_path, monkeypatch):
    """引擎檔不在 ⇒ Python 不注入；頁面上沒有 windInfo（MapWidget 端只會呼叫、不會炸）。"""
    monkeypatch.setattr(wl, "WIND_FLOW_JS", tmp_path / "missing.js")
    w = _bare2d()
    html = _folium_page_html(w)
    assert "wind_flow" not in html and "function setWindField" not in html


# ═══════════════════════════════════════════════════════════════════════
#  3D：真的 Cesium 1.115 ＋ 引擎 ＋ cesium 膠水
# ═══════════════════════════════════════════════════════════════════════
@pytest.fixture(scope="module")
def page3d(_pw, tmp_path_factory):
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    d = tmp_path_factory.mktemp("w3d")
    page_file = d / "w3d.html"
    page_file.write_text(f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<script src="{(CESIUM_DIR / 'Cesium.js').as_uri()}"></script>
<script src="{wl.wind_flow_js_uri()}"></script>
<style>html,body,#c{{width:100%;height:100%;margin:0;padding:0;overflow:hidden}}#c{{position:relative}}</style>
</head><body><div id="c"></div><script>
Cesium.Ion.defaultAccessToken = '';
window.viewer = new Cesium.Viewer('c', {{ terrainProvider: new Cesium.EllipsoidTerrainProvider(),
  baseLayer: false, baseLayerPicker: false, navigationHelpButton: false, homeButton: false,
  sceneModePicker: false, geocoder: false, timeline: false, animation: false, fullscreenButton: false,
  infoBox: false, selectionIndicator: false, skyBox: false, skyAtmosphere: false }});
viewer.scene.globe.baseColor = Cesium.Color.BLACK; viewer.scene.backgroundColor = Cesium.Color.BLACK;
viewer.camera.setView({{destination: Cesium.Cartesian3.fromDegrees({LON0 + 0.004}, {LAT0 + 0.0025}, 1500),
                        orientation: {{heading: 0, pitch: -Math.PI / 2, roll: 0}}}});
{wl.glue_js('cesium')}
</script></body></html>""", encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
        pg = b.new_page(viewport={"width": 800, "height": 600})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        _route_local_only(pg)
        pg.goto(page_file.as_uri())
        pg.wait_for_function("window.viewer !== undefined && typeof windInfo === 'function' && windInfo().engine",
                             timeout=60000)
    except Exception as e:                                   # pragma: no cover - 沒有 WebGL 等
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._errs = errs
    yield pg
    b.close()


def test_3d_adapter_projects_the_area_centre_near_the_screen_centre_and_hides_the_far_side(page3d):
    r = page3d.evaluate(f"""() => {{ const a = _windAdapter();
      return {{ c: a.project({LAT0 + 0.0025}, {LON0 + 0.004}, 0), far: a.project({-LAT0}, {LON0 - 180}, 0),
                key: a.viewKey(), w: viewer.canvas.clientWidth, h: viewer.canvas.clientHeight }}; }}""")
    assert r["c"] is not None and abs(r["c"]["x"] - r["w"] / 2) < 40 and abs(r["c"]["y"] - r["h"] / 2) < 40, r
    assert r["far"] is None, "地球背面的點要回 null"
    assert "|" in r["key"] and "x" in r["key"].split("|")[-1]


def test_3d_field_draws_particles_over_the_area_and_the_view_key_follows_the_camera(page3d):
    assert page3d.evaluate("(f) => setWindField(f)", FIELD.to_js()) is True
    page3d.wait_for_function("windInfo().state.field && windInfo().state.frames > 40 && windInfo().state.particles > 0",
                             timeout=30000)
    px = page3d.evaluate(_PIXELS % "viewer.container")
    assert px["canvases"] >= 1 and px["pe"] == "none" and px["pos"] == "absolute"
    assert px["nc"] > 50, f"3D 畫面中央沒有粒子軌跡像素：{px}"
    k0 = page3d.evaluate("windInfo().state.viewKey")
    page3d.evaluate("viewer.camera.moveRight(200)")
    page3d.wait_for_function(f"windInfo().state.viewKey !== {json.dumps(k0)}", timeout=5000)
    assert page3d.evaluate("(r) => setWindArrows([r])", ARROW) is True
    page3d.wait_for_function("windInfo().state.arrows === 1", timeout=5000)
    assert page3d.evaluate("clearWindField() && clearWindArrows()") is True
    assert page3d.evaluate("windInfo().state.field") is False and page3d.evaluate("windInfo().state.arrows") == 0
    assert not page3d._errs, page3d._errs


def test_the_whole_main_html_template_replays_python_state_through_the_placeholders(_pw, tmp_path):
    """整份 main.html 照 _load_cesium 的方式填（含三個風場佔位）：引擎從 head 載入、膠水與狀態在頁面初始化時
    套回去，不需要 Python 再呼叫一次。"""
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    rows = [list(ARROW)]
    restore = wl.restore_js(wl.check_wind_field(FIELD), rows, {"limitMps": 2.0})
    html = MAIN_HTML.read_text(encoding="utf-8")
    for k, v in {"__WIND_FLOW_SCRIPT__": f'<script src="{wl.wind_flow_js_uri()}"></script>',
                 "/*__WIND_GLUE__*/": wl.glue_js("cesium"), "/*__WIND_RESTORE__*/": restore,
                 "__CESIUM_JS__": (CESIUM_DIR / "Cesium.js").as_uri(),
                 "__CESIUM_CSS__": (CESIUM_DIR / "Widgets" / "widgets.css").as_uri(),
                 "__CESIUM_TOKEN__": "", "__INIT_LAT__": str(LAT0), "__INIT_LON__": str(LON0),
                 "__INIT_HEIGHT__": "2000", "__PLANE_MODEL_URI__": "", "__COPTER_MODEL_URI__": "",
                 "__VTOL_MODEL_URI__": "", "__VTOL_MODEL_SCALE__": "0.01", "__VTOL_HEADING_OFFSET__": "-90",
                 "__VTOL_PITCH_OFFSET__": "90", "__VTOL_ROLL_OFFSET__": "180"}.items():
        html = html.replace(k, v)
    assert not [k for k in ("__WIND_FLOW_SCRIPT__", "/*__WIND_GLUE__*/", "/*__WIND_RESTORE__*/") if k in html]
    f = tmp_path / "main.html"
    f.write_text(html, encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    try:
        pg = b.new_page(viewport={"width": 640, "height": 480})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        _route_local_only(pg)
        pg.goto(f.as_uri())
        pg.wait_for_function("typeof viewer !== 'undefined' && typeof windInfo === 'function' && windInfo().layer",
                             timeout=60000)
        info = pg.evaluate("windInfo()")
        survey = pg.evaluate("surveyPointsInfo()")
    finally:
        b.close()
    assert not errs, errs
    assert info["engine"] is True and info["field"] is True and info["arrows"] == 1
    assert info["options"]["limitMps"] == 2.0 and info["state"]["label"] == FIELD.label
    assert info["state"]["arrows"] == 1
    assert survey == {}, "其餘區段照常初始化"
