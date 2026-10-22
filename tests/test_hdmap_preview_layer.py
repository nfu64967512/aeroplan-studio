"""預計成果圖層（高精地圖「規劃預測」）—— ui.widgets.hdmap_preview_layer ＋ 2D／3D／Dual 的 set／clear／visible：值層級的鎖。

三層：
  A. 純 Python：payload 檢查（整份拒絕、不偷偷截斷）、可放進行內 <script> 的 JSON、膠水不含佔位字與 ``</``、
     兩個 widget 記住的狀態與送出去的 JS、3D 範本佔位最後換、Dual 兩張圖都畫、2D 圖層控制最後才加。
  B. 真的 Leaflet（本機 assets/leaflet）跑 MapWidget._build_map／_inject_javascript 產生的整頁：畫上去的東西、
     影像位置與方向（截圖讀像素）、範圍外變暗（奇偶填色的洞）、窗格 z-index／不吃滑鼠、圖例在角點計數框之上、
     漏洞懸停說明（標籤是文字不是 HTML）、clear_all_display_layers_js 不會掃掉、顯示切換、整頁重建後套回；
     加上 folium 圖層控制的回歸（照片位姿一加整張 2D 圖空白）。
  C. 真的 Cesium 1.115（assets/cesium，gitignored；沒有就 skip）跑 _fill_cesium_template 填好的整份 main.html：
     單張影像 ImageryLayer（NEAREST、範圍＝bounds）、範圍外變暗（巢狀 PolygonHierarchy）、截圖讀像素、
     圖層面板「地圖」開關不關它／「預計成果」勾選框會、顯示切換、清除、頁面重載時由範本套回。

B、C 要開瀏覽器：用 safe runner 時要設 SAFE_ALLOW_BROWSER=1，否則這兩層 skip。網路一律擋掉；folium 從 CDN 載的
三個外掛（全螢幕／滑鼠座標／量測）在 B 層用空殼頂替（擋掉時 folium 的整段 script 會在第一個外掛中止，
後面的圖層一個都不會畫 —— 那就量不到任何東西）。
"""
from __future__ import annotations

import base64
import io
import json
import os
import re

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")

from ui.widgets import hdmap_preview_layer as hl  # noqa: E402

try:
    import folium
    from ui.widgets.map_widget import MapWidget
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from tests.test_survey_map_layers import _pw, _launch, _REAL_ARGS, CESIUM_DIR, MAIN_HTML, LAT0, LON0  # noqa: F401
    from tests.test_wind_map_pages import _bare2d, _folium_page_html

    class _NoDel3D(CesiumMapWidget):
        """裸實例（沒跑 QWidget.__init__）回收時 __del__ 會摸 Qt 屬性；測試用這個子類。"""
        def __del__(self):
            pass
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e
    LAT0, LON0 = 25.07, 121.21

    @pytest.fixture(scope="module")
    def _pw():
        pytest.skip(f"Qt 匯入失敗：{_QT_ERR}")

needs_qt = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

# ═══════════════════════════════════════════════════════════════════════
#  手做的 payload：2×2 的品質分級 PNG（四個象限四種顏色 ⇒ 讀像素就知道方向對不對）
# ═══════════════════════════════════════════════════════════════════════
B = (LON0, LAT0, LON0 + 0.004, LAT0 + 0.0036)            # 西, 南, 東, 北（約 400 m × 400 m）
QUAD = {"NW": (229, 57, 53), "NE": (67, 160, 71), "SW": (30, 136, 229), "SE": (253, 216, 53)}
HOLE_LABEL = "內部漏洞 <b>#1</b>：約 120 m²（最少 1 張）"
EDGE_LABEL = "邊緣不足：南側 <i>x</i>"


def _f(fx, fy):
    """bounds 內的比例位置 → [lat, lon]（fx 由西往東、fy 由南往北）。"""
    w, s, e, n = B
    return [s + fy * (n - s), w + fx * (e - w)]


def _box(x0, y0, x1, y1):
    return [_f(x0, y0), _f(x1, y0), _f(x1, y1), _f(x0, y1)]


def _png_b64() -> str:
    from PIL import Image
    im = Image.new("RGBA", (2, 2))
    im.putpixel((0, 0), QUAD["NW"] + (255,))                 # 第 0 列 = 北（north-up）
    im.putpixel((1, 0), QUAD["NE"] + (255,))
    im.putpixel((0, 1), QUAD["SW"] + (255,))
    im.putpixel((1, 1), QUAD["SE"] + (255,))
    buf = io.BytesIO()
    im.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


POINT_FX = (0.15, 0.3, 0.42, 0.58, 0.7, 0.85, 0.95)


def _payload(**over) -> dict:
    p = {
        "id": "hdmap_plan", "label": "預計成果（規劃預測）", "bounds": list(B),
        "image": "data:image/png;base64," + _png_b64(), "image_px": [2, 2],
        "opacity": 0.45, "mask_alpha": 0.55,
        "extent_rings": [_box(0.1, 0.1, 0.9, 0.9)],
        "extent_holes": [_box(0.45, 0.45, 0.55, 0.55)],
        "holes": [{"ring": _box(0.46, 0.46, 0.54, 0.54), "label": HOLE_LABEL}],
        "edge_short": [{"ring": _box(0.40, 0.0, 0.60, 0.08), "label": EDGE_LABEL}],
        "survey_rings": [_box(0.0, 0.0, 1.0, 1.0)],
        "points": [[*_f(x, 0.5), i % 2] for i, x in enumerate(POINT_FX)],
        "point_colors": ["#00E5FF", "#FF4081"],
        "legend": {"title": "每點被拍到的張數（規劃預測）",
                   "classes": [{"label": "未拍到", "color": None},
                               {"label": "只有 1 張（不會成圖）", "color": "#e53935"},
                               {"label": "≥9 張（高品質）", "color": "#43a047"}],
                   "notes": ["CAM_MAX_ROLL 與轉彎動態未模擬（樂觀）"],
                   "points": [{"label": "UAV-1", "color": "#00E5FF"}, {"label": "UAV-2 <i>", "color": "#FF4081"}]},
        "stats_line": "預計成圖 0.10 km²；拍照 7 張",
    }
    p.update(over)
    return p


class _Page:
    """假 QWebEnginePage：只記下送出去的 JS。"""
    def __init__(self):
        self.js = []

    def runJavaScript(self, js, cb=None):
        self.js.append(js)


def _call(js: str, fn: str) -> list:
    m = re.fullmatch(re.escape(fn) + r"\((.*)\)", js, flags=re.S)
    assert m, f"不是 {fn}(...)：{js[:120]}"
    return json.loads("[" + m.group(1) + "]")


# ═══════════════════════════════════════════════════════════════════════
#  A. 純 Python
# ═══════════════════════════════════════════════════════════════════════
def test_check_payload_normalizes_overrides_the_id_and_keeps_every_field():
    p = hl.check_payload(_payload(), "plan_x")
    assert p["id"] == "plan_x" and p["bounds"] == list(B) and p["image_px"] == [2, 2]
    assert (p["opacity"], p["mask_alpha"]) == (0.45, 0.55)
    assert len(p["extent_rings"]) == 1 and len(p["extent_rings"][0]) == 4
    assert p["holes"][0]["label"] == HOLE_LABEL, "標籤原樣保留（轉義是頁面的事：用 textContent）"
    assert [q[2] for q in p["points"]] == [0, 1, 0, 1, 0, 1, 0]
    assert p["point_colors"] == ["#00E5FF", "#FF4081"]
    assert p["legend"]["classes"][0] == {"label": "未拍到", "color": None}
    assert p["legend"]["classes"][1]["color"] == "#E53935"
    assert p["legend"]["points"] == [{"label": "UAV-1", "color": "#00E5FF"}, {"label": "UAV-2 <i>", "color": "#FF4081"}]
    json.dumps(p, allow_nan=False)
    s = hl.summary(p)
    assert (s["extent_rings"], s["extent_holes"], s["holes"], s["edge_short"], s["survey_rings"], s["points"]) == \
        (1, 1, 1, 1, 1, 7)
    # 省略的欄位有明確預設；label 空字串也補回「規劃預測」
    q = hl.check_payload({k: v for k, v in _payload(label="").items()
                          if k not in ("extent_holes", "edge_short", "stats_line")})
    assert q["label"] == hl.LABEL_DEFAULT and q["extent_holes"] == [] and q["edge_short"] == [] and q["stats_line"] == ""


def _mut(path, value):
    p = _payload()
    if path == "points[0]":
        p["points"][0] = value
    elif path == "ring":
        p["extent_rings"][0][1] = value
    elif path == "legend.color":
        p["legend"]["classes"][1]["color"] = value
    else:
        p[path] = value
    return p


@pytest.mark.parametrize("payload,what", [
    (_mut("ring", [float("nan"), 121.2]), "extent_rings"),
    (_mut("ring", [95.0, 121.2]), "extent_rings"),
    (_mut("extent_rings", [[[25.0, 121.0], [25.0, 121.0], [25.1, 121.1]]]), "少於 3 個"),
    (_mut("points[0]", [25.07, 121.21, 2]), "color_index"),
    (_mut("points[0]", [25.07, 121.21, True]), "color_index"),
    (_mut("points", [[25.07, 121.21, 0]] * (hl.PREVIEW_MAX_POINTS + 1)), "超過上限"),
    (_mut("point_colors", ["red", "#00FF00"]), "point_colors"),
    (_mut("legend.color", "#12345"), "legend.classes"),
    (_mut("image", "data:image/jpeg;base64,AAAA"), "image"),
    (_mut("image", "data:image/png;base64,"), "base64"),
    (_mut("image", "data:image/png;base64,<script>"), "base64"),
    (_mut("bounds", [121.214, 25.07, 121.21, 25.0736]), "bounds"),
    (_mut("bounds", [121.21, 25.07, 121.214]), "bounds"),
    (_mut("opacity", 1.5), "opacity"),
    (_mut("mask_alpha", float("inf")), "mask_alpha"),
    (_mut("holes", [{"ring": "x", "label": "a"}]), "holes[0].ring"),
    (_mut("image_px", [2.5, 2]), "image_px"),
])
def test_check_payload_rejects_the_whole_payload_and_says_which_field(payload, what):
    with pytest.raises(ValueError, match=re.escape(what)):
        hl.check_payload(payload)


def test_check_payload_takes_numpy_scalars_and_hands_out_plain_python_numbers():
    np = pytest.importorskip("numpy")
    p = _payload()
    p["bounds"] = [np.float64(v) for v in B]
    p["points"] = [[np.float32(q[0]), np.float64(q[1]), np.int64(q[2])] for q in p["points"]]
    p["opacity"] = np.float32(0.5)
    p["image_px"] = [np.int64(2), np.int32(2)]
    out = hl.check_payload(p)
    assert type(out["points"][0][2]) is int and type(out["points"][0][0]) is float and type(out["opacity"]) is float
    assert out["image_px"] == [2, 2] and type(out["image_px"][0]) is int
    json.dumps(out, allow_nan=False)


def test_the_real_survey_preview_payload_is_accepted_as_is():
    """合約對接：survey.preview.HdmapPreview.to_js_payload() 的輸出原樣丟進來要過、而且什麼都不掉。"""
    sp = pytest.importorskip("survey.preview")
    pv = sp.HdmapPreview(ok=True, reason=None, bounds_wsen=tuple(B), image_png=base64.b64decode(_png_b64()),
                         image_px=(2, 2), cell_size_m=2.0,
                         extent_rings=[_box(0.1, 0.1, 0.9, 0.9)], extent_holes=[_box(0.45, 0.45, 0.55, 0.55)],
                         interior_holes=[{"ring": _box(0.46, 0.46, 0.54, 0.54), "area_m2": 120.0, "min_views": 1}],
                         edge_short=[{"ring": _box(0.40, 0.0, 0.60, 0.08), "area_m2": 40.0, "min_views": 3}],
                         survey_rings=[_box(0.0, 0.0, 1.0, 1.0)],
                         photos=[[*_f(x, 0.5), i % 2] for i, x in enumerate(POINT_FX)])
    pv.stats["uav_labels"] = ["UAV-1", "UAV-2"]
    raw = pv.to_js_payload()
    p = hl.check_payload(raw, "hdmap_plan")
    s = hl.summary(p)
    assert (s["extent_rings"], s["extent_holes"], s["holes"], s["edge_short"], s["survey_rings"], s["points"]) ==         (1, 1, 1, 1, 1, len(POINT_FX))
    assert p["bounds"] == list(B) and p["image"] == raw["image"] and p["image_px"] == [2, 2]
    assert all(h["label"] for h in p["holes"] + p["edge_short"]), "漏洞／邊緣不足要帶說明文字"
    assert [c["label"] for c in p["legend"]["points"]] == ["UAV-1", "UAV-2"]
    assert len(p["legend"]["classes"]) == len(raw["legend"]["classes"]) >= 1
    assert "規劃預測" in p["label"]


def test_check_payload_rejects_non_dicts_and_empty_ids():
    for bad in (None, [], "x"):
        with pytest.raises(ValueError):
            hl.check_payload(bad)
    with pytest.raises(ValueError, match="圖層代號"):
        hl.check_payload(_payload(), "  ")


def test_js_json_cannot_close_the_script_tag_and_round_trips():
    evil = "</script><script>alert(1)</script> &  "
    s = hl.js_call("setHdmapPreview", "x", {"label": evil})
    assert "</" not in s and "<" not in s and ">" not in s and " " not in s
    args = _call(s, "setHdmapPreview")                       # \\u003c 在 JSON 裡就是 '<'：值不變
    assert args == ["x", {"label": evil}]
    with pytest.raises(ValueError):
        hl.js_call("setHdmapPreview", "x", {"v": float("nan")})
    with pytest.raises(ValueError):
        hl.js_call("eval", "x")


@pytest.mark.parametrize("kind", ["leaflet", "cesium"])
def test_glue_defines_the_page_functions_without_placeholders_or_closing_tags(kind):
    g = hl.glue_js(kind)
    for fn in hl.PAGE_FUNCTIONS:
        assert re.search(r"^function\s+%s\s*\(" % fn, g, flags=re.M), fn
    assert "</" not in g, "膠水會被放進行內 <script>：不可以有 </"
    assert re.findall(r"__[A-Z][A-Z0-9_]*__", g) == [], "3D 範本的佔位字檢查會把它當成沒換掉的佔位"
    assert "%STYLE" not in g and "%BANNER%" not in g
    assert hl.PREDICTION_BANNER in g.encode("utf-8").decode("utf-8") or json.dumps(hl.PREDICTION_BANNER) in g
    assert "innerHTML" not in g, "標籤是資料：一律 textContent，不可以當 HTML"
    with pytest.raises(ValueError):
        hl.glue_js("svg")


def test_restore_js_replays_every_layer_and_the_hidden_ones_stay_hidden():
    a, b = hl.check_payload(_payload(), "a"), hl.check_payload(_payload(), "b")
    js = hl.restore_js({"a": a, "b": b}, {"b"})
    lines = js.split("\n")
    assert len(lines) == 3
    assert _call(lines[0].rstrip(";"), "setHdmapPreview")[0] == "a"
    assert _call(lines[1].rstrip(";"), "setHdmapPreview") == ["b", b]
    assert _call(lines[2].rstrip(";"), "setHdmapPreviewVisible") == ["b", False]
    assert hl.restore_js({}, ()) == "" and hl.restore_js(None) == ""


# ── 2D widget（裸實例；不建 QWebEngineView）──────────────────────────────
def _bare2d_fake():
    w = _bare2d()
    w.custom_page = _Page()
    w._render_map = lambda: None
    return w


@needs_qt
def test_2d_set_clear_visible_keep_python_state_and_send_one_call_each():
    w = _bare2d_fake()
    r = MapWidget.set_hdmap_preview(w, "hdmap_plan", _payload())
    assert r["ok"] is True and r["points"] == 7 and r["holes"] == 1 and r["bounds"] == list(B)
    lid, p = _call(w.custom_page.js[-1], "setHdmapPreview")
    assert lid == "hdmap_plan" and p == w._hdmap_previews["hdmap_plan"] and p["id"] == "hdmap_plan"
    MapWidget.set_hdmap_preview_visible(w, "hdmap_plan", False)
    assert w._hdmap_hidden == {"hdmap_plan"}
    assert _call(w.custom_page.js[-1], "setHdmapPreviewVisible") == ["hdmap_plan", False]
    n = len(w.custom_page.js)
    MapWidget.set_hdmap_preview_visible(w, "nope", True)       # 沒有這一層：不送、不記
    assert len(w.custom_page.js) == n and "nope" not in w._hdmap_hidden
    MapWidget.set_hdmap_preview(w, "hdmap_plan", _payload())  # 重畫 = 顯示
    assert w._hdmap_hidden == set()
    MapWidget.set_hdmap_preview(w, "other", _payload())
    assert MapWidget.clear_hdmap_preview(w, "other") is True and set(w._hdmap_previews) == {"hdmap_plan"}
    assert _call(w.custom_page.js[-1], "clearHdmapPreview") == ["other"]
    assert MapWidget.clear_hdmap_preview(w, "other") is False
    assert MapWidget.clear_hdmap_preview(w) is True and w._hdmap_previews == {}
    assert _call(w.custom_page.js[-1], "clearHdmapPreview") == [None]
    assert MapWidget.clear_hdmap_preview(w) is False


@needs_qt
def test_2d_bad_payload_reports_and_removes_the_stale_layer_of_that_id():
    w = _bare2d_fake()
    MapWidget.set_hdmap_preview(w, "hdmap_plan", _payload())
    r = MapWidget.set_hdmap_preview(w, "hdmap_plan", _payload(opacity=2.0))
    assert r["ok"] is False and "opacity" in r["error"] and r["removed_old"] is True
    assert "hdmap_plan" not in w._hdmap_previews, "舊的是上一份規劃的預測，不能留著被當成這一份的"
    assert _call(w.custom_page.js[-1], "clearHdmapPreview") == ["hdmap_plan"]
    n = len(w.custom_page.js)
    r = MapWidget.set_hdmap_preview(w, None, _payload())       # 代號不合法：不可以送 clearHdmapPreview(null) 清光全部
    assert r["ok"] is False and len(w.custom_page.js) == n


@needs_qt
def test_2d_page_always_has_the_glue_and_replays_stored_layers_with_hidden_state():
    w = _bare2d_fake()
    html0 = _folium_page_html(w)
    assert "function setHdmapPreview" in html0 and "setHdmapPreview(\"" not in html0
    MapWidget.set_hdmap_preview(w, "a", _payload(label="</script><b>x</b>"))
    MapWidget.set_hdmap_preview(w, "b", _payload())
    MapWidget.set_hdmap_preview_visible(w, "b", False)
    html = _folium_page_html(w)
    block = html[html.index("function setHdmapPreview"):]
    block = block[:block.index("</script>")]
    assert 'setHdmapPreview("a",' in block and 'setHdmapPreview("b",' in block
    assert 'setHdmapPreviewVisible("b", false)' in block
    assert "</script><b>" not in html, "標籤裡的 </script> 不可以原樣進頁面"


@needs_qt
def test_2d_a_change_made_while_the_page_reloads_is_resent_once_the_load_finishes():
    """整頁重建時頁面帶的是「產生 HTML 那一刻」的狀態；載入期間才 set 的會送到舊頁面上丟失 ⇒ loadFinished 補送。"""
    w = _bare2d_fake()
    _folium_page_html(w)                                       # 頁面帶版本 0
    assert MapWidget._hdmap_resync_after_load(w, True) is False, "沒變動就不重送"
    MapWidget.set_hdmap_preview(w, "a", _payload())            # 載入期間：送到舊頁面（丟失）
    assert MapWidget._hdmap_resync_after_load(w, False) is False, "載入失敗不送"
    assert MapWidget._hdmap_resync_after_load(w, True) is True
    js = w.custom_page.js[-1]
    assert js.startswith("clearHdmapPreview(null);\n") and 'setHdmapPreview("a",' in js
    assert MapWidget._hdmap_resync_after_load(w, True) is False, "只補送一次"
    MapWidget.set_hdmap_preview_visible(w, "a", False)
    _folium_page_html(w)                                       # 新頁面已帶最新狀態 ⇒ 不必補
    assert MapWidget._hdmap_resync_after_load(w, True) is False
    MapWidget.clear_hdmap_preview(w, "a")
    assert MapWidget._hdmap_resync_after_load(w, True) is True
    assert w.custom_page.js[-1].strip() == "clearHdmapPreview(null);"


@needs_qt
def test_2d_clear_all_drops_the_preview_and_the_planning_wipe_skips_flagged_layers():
    w = _bare2d_fake()
    w.clear_swarm_display = lambda: None
    w.clear_fc_mission = lambda: None
    MapWidget.set_hdmap_preview(w, "hdmap_plan", _payload())
    MapWidget.clear_all(w)
    assert w._hdmap_previews == {} and _call(w.custom_page.js[-1], "clearHdmapPreview") == [None]
    MapWidget.clear_all_display_layers_js(w)
    assert "layer.options.hdmapPreview" in w.custom_page.js[-1]


@needs_qt
def test_2d_layer_control_is_added_after_every_overlay():
    """folium.LayerControl 在 render 時收集圖層，但 JS 寫在它被加入的位置：要排在所有疊圖之後。"""
    w = _bare2d_fake()
    rows = [{"lat_deg": LAT0 + i * 1e-4, "lon_deg": LON0, "h_ell_m": 100, "yaw_deg": 90, "source": "ppk",
             "image_id": f"I{i}"} for i in range(5)]
    MapWidget.set_photo_poses(w, "p1", rows)
    MapWidget.set_survey_points(w, "g1", [{"id": "G1", "type": "GCP", "lat": LAT0, "lon": LON0, "label": "G1"}])
    m = w._build_map()
    controls = [c for c in m._children.values() if isinstance(c, folium.LayerControl)]
    assert len(controls) == 1
    assert list(m._children)[-1] == controls[0].get_name(), "圖層控制要是地圖的最後一個子元素"
    html = m.get_root().render()
    i_ctl = html.index("L.control.layers(")
    for var in re.findall(r"var (feature_group_[0-9a-f]+) = L\.featureGroup", html):
        assert html.index(f"var {var} =") < i_ctl, f"{var} 在圖層控制之後才宣告 ⇒ 控制拿到 undefined"


# ── 3D widget（裸實例）──────────────────────────────────────────────────
def _bare3d():
    w = _NoDel3D.__new__(_NoDel3D)
    w._sent = []
    w._js = w._sent.append
    return w


def _no_settings():
    raise RuntimeError("測試不讀 config/settings.json")


def _template_values(mp, w) -> dict:
    import config.settings as cs
    import ui.widgets.glb_merge as gm
    mp.setattr(cs, "get_settings", _no_settings)
    mp.setattr(CesiumMapWidget, "_resolve_token", lambda self: "")
    mp.setattr(gm, "optimized_model_path", lambda p, cache: p)      # 不寫 data/cache
    mp.delenv(CesiumMapWidget.RENDER_DT_ENV, raising=False)
    return w._cesium_template_values()


@needs_qt
def test_3d_set_clear_visible_and_the_template_replays_last(monkeypatch):
    w = _bare3d()
    vals0 = _template_values(monkeypatch, w)
    assert list(vals0)[-1] == "/*__HDMAP_GLUE__*/", "帶資料的置換要排最後（不被別的佔位誤觸）"
    assert "function setHdmapPreview" in vals0["/*__HDMAP_GLUE__*/"] and 'setHdmapPreview("' not in vals0["/*__HDMAP_GLUE__*/"]
    r = CesiumMapWidget.set_hdmap_preview(w, "hdmap_plan", _payload())
    assert r["ok"] is True and r["edge_short"] == 1
    assert _call(w._sent[-1], "setHdmapPreview")[1] == w._hdmap_previews["hdmap_plan"]
    CesiumMapWidget.set_hdmap_preview_visible(w, "hdmap_plan", False)
    vals = _template_values(monkeypatch, w)
    glue = vals["/*__HDMAP_GLUE__*/"]
    assert 'setHdmapPreview("hdmap_plan",' in glue and 'setHdmapPreviewVisible("hdmap_plan", false)' in glue
    r = CesiumMapWidget.set_hdmap_preview(w, "hdmap_plan", _payload(points=[[1, 2]]))
    assert r["ok"] is False and r["removed_old"] is True and w._hdmap_previews == {}
    CesiumMapWidget.set_hdmap_preview(w, "hdmap_plan", _payload())
    w.__dict__.update(corners=[], paths=[], path_colors=[], _path_tooltips=[], transit_paths=[], _nfz_zones=[])
    CesiumMapWidget.clear_all(w)
    assert w._hdmap_previews == {} and any("clearHdmapPreview(null)" in s for s in w._sent)


@needs_qt
def test_3d_a_change_made_while_the_page_loads_is_resent_once_the_load_finishes(monkeypatch):
    w = _bare3d()
    _template_values(monkeypatch, w)                           # 範本帶版本 0
    assert CesiumMapWidget._hdmap_resync_after_load(w, True) is False
    CesiumMapWidget.set_hdmap_preview(w, "a", _payload())
    assert CesiumMapWidget._hdmap_resync_after_load(w, True) is True
    assert w._sent[-1].startswith("clearHdmapPreview(null);\n") and 'setHdmapPreview("a",' in w._sent[-1]
    assert CesiumMapWidget._hdmap_resync_after_load(w, True) is False
    _template_values(monkeypatch, w)                           # 重新填值 ⇒ 範本已帶最新狀態
    CesiumMapWidget.set_hdmap_preview_visible(w, "a", False)
    assert CesiumMapWidget._hdmap_resync_after_load(w, True) is True
    assert 'setHdmapPreviewVisible("a", false)' in w._sent[-1]


@needs_qt
def test_dual_draws_both_maps_and_returns_both_results():
    class M:
        def __init__(self, tag):
            self.tag, self.calls = tag, []

        def set_hdmap_preview(self, lid, p):
            self.calls.append(("set", lid))
            return {"ok": True, "map": self.tag}

        def clear_hdmap_preview(self, lid=None):
            self.calls.append(("clear", lid))
            return self.tag == "3d"

        def set_hdmap_preview_visible(self, lid, v):
            self.calls.append(("vis", lid, v))

    d = DualMapWidget.__new__(DualMapWidget)
    d.__dict__.update(map_2d=M("2d"), map_3d=M("3d"))
    assert DualMapWidget.set_hdmap_preview(d, "x", {}) == {"2d": {"ok": True, "map": "2d"},
                                                          "3d": {"ok": True, "map": "3d"}}
    assert DualMapWidget.clear_hdmap_preview(d) is True
    assert DualMapWidget.set_hdmap_preview_visible(d, "x", False) is None
    for m in (d.map_2d, d.map_3d):
        assert m.calls == [("set", "x"), ("clear", None), ("vis", "x", False)]


# ═══════════════════════════════════════════════════════════════════════
#  B. 真的 Leaflet：MapWidget 產生的整頁
# ═══════════════════════════════════════════════════════════════════════
_PLUGIN_STUBS = {      # folium 從 CDN 載的外掛 → 空殼（測試不連網）
    "leaflet.fullscreen": "L.Control.FullScreen = L.Control.extend({onAdd: function () { return L.DomUtil.create('div'); }});"
                          "L.control.fullscreen = function (o) { return new L.Control.FullScreen(o); };",
    "Leaflet.MousePosition": "L.Control.MousePosition = L.Control.extend({onAdd: function () { return L.DomUtil.create('div'); }});",
    "leaflet-measure": "L.Control.Measure = L.Control.extend({onAdd: function () { return L.DomUtil.create('div'); }});",
    "leaflet.awesome-markers": "L.AwesomeMarkers = {icon: function () { return L.divIcon({className: 'stub-awesome'}); }};",
}


def _route_with_plugin_stubs(pg):
    def handler(route):
        url = route.request.url
        if url.startswith(("file:", "data:", "blob:")):
            return route.continue_()
        if url.split("?")[0].endswith(".js"):
            for key, body in _PLUGIN_STUBS.items():
                if key in url:
                    return route.fulfill(status=200, content_type="application/javascript", body=body)
        return route.abort()
    pg.route("**/*", handler)


def _bare2d_empty():
    """沒有角點（不會有藍色邊界多邊形蓋在上面染色）；初始視角給定（不讀設定）。"""
    w = _bare2d_fake()
    w.corners = []
    w._view_state = {"lat": (B[1] + B[3]) / 2, "lon": (B[0] + B[2]) / 2, "zoom": 17}
    return w


def _open_2d(browser, html_path, errs):
    pg = browser.new_page(viewport={"width": 1000, "height": 800})
    pg.on("pageerror", lambda e: errs.append(str(e)))
    _route_with_plugin_stubs(pg)
    pg.goto(html_path.as_uri())
    pg.wait_for_function("window.currentMap && typeof setHdmapPreview === 'function'", timeout=30000)
    return pg


@pytest.fixture(scope="module")
def browser2d(_pw):
    try:
        b = _launch(_pw, ["--allow-file-access-from-files"])
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    yield b
    b.close()


@pytest.fixture(scope="module")
def page2d(browser2d, tmp_path_factory):
    if not _QT_OK:
        pytest.skip("Qt 匯入失敗")
    f = tmp_path_factory.mktemp("hd2d") / "folium.html"
    f.write_text(_folium_page_html(_bare2d_empty()), encoding="utf-8")
    errs = []
    pg = _open_2d(browser2d, f, errs)
    pg._errs = errs
    yield pg
    pg.close()


@pytest.fixture
def page(page2d):
    yield page2d
    page2d.evaluate("clearHdmapPreview(null)")
    assert not page2d._errs, page2d._errs


def _set_2d(pg, lid="hdmap_plan", **over):
    w = _bare2d_fake()
    r = MapWidget.set_hdmap_preview(w, lid, _payload(**over))
    assert r["ok"] is True, r
    out = pg.evaluate(w.custom_page.js[-1])
    assert out["ok"] is True and out["id"] == lid
    return w


def _info2d(pg, lid="hdmap_plan"):
    return pg.evaluate("hdmapPreviewInfo()")["layers"][lid]


@needs_qt
def test_2d_draws_every_part_in_its_own_non_interactive_pane_below_the_paths(page):
    _set_2d(page)
    info = page.evaluate("hdmapPreviewInfo()")
    assert info["ids"] == ["hdmap_plan"]
    L = info["layers"]["hdmap_plan"]
    assert L["onMap"] and L["image"] and L["imageBounds"] == pytest.approx(list(B), abs=1e-12)
    assert L["imageOpacity"] == 0.45 and L["imageRendering"] == "pixelated" and L["imagePane"] == "hdmapPreviewImg"
    assert L["mask"] and L["maskRings"] == 3 and L["maskAlpha"] == 0.55, "世界外框＋範圍外框（洞）＋範圍內的洞（島）"
    assert (L["extent"], L["extentHoles"], L["holes"], L["edgeShort"], L["survey"], L["points"]) == (1, 1, 1, 1, 1, 7)
    assert L["hits"] == 2, "漏洞與邊緣不足各一個懸停說明目標"
    assert L["interactive"] == 0, "預覽圖層裡不可以有吃滑鼠的東西（會搶走規劃路徑／禁航區的懸停與點擊）"
    assert page.evaluate("!window.currentMap.getPane('hdmapPreviewTips')"), "z 450 的隱形命中多邊形窗格不能再出現"
    assert L["legend"] and L["legendShown"]
    panes = page.evaluate("""() => { const m = window.currentMap, cs = (n) => getComputedStyle(m.getPane(n));
      return { root: [cs('hdmapPreview').zIndex, cs('hdmapPreview').pointerEvents],
               img: [cs('hdmapPreviewImg').zIndex, m.getPane('hdmapPreviewImg').parentNode === m.getPane('hdmapPreview')],
               vec: [cs('hdmapPreviewVec').zIndex, m.getPane('hdmapPreviewVec').parentNode === m.getPane('hdmapPreview')],
               overlay: cs('overlayPane').zIndex, tile: cs('tilePane').zIndex }; }""")
    assert panes["root"] == ["350", "none"] and panes["img"] == ["1", True] and panes["vec"] == ["2", True]
    assert int(panes["tile"]) < 350 < int(panes["overlay"]), "要在底圖之上、規劃路徑之下"
    # 影像四角就在 bounds 四角（north-up、不翻轉）
    geo = page.evaluate("""(b) => { const m = window.currentMap, el = m.getPane('hdmapPreviewImg').querySelector('img');
      const r = el.getBoundingClientRect(), c = m.getContainer().getBoundingClientRect();
      const nw = m.latLngToContainerPoint([b[3], b[0]]), se = m.latLngToContainerPoint([b[1], b[2]]);
      return { r: [r.left - c.left, r.top - c.top, r.right - c.left, r.bottom - c.top], p: [nw.x, nw.y, se.x, se.y] }; }""",
                        list(B))
    assert geo["r"] == pytest.approx(geo["p"], abs=1.5), geo


@needs_qt
def test_2d_legend_says_prediction_uses_text_not_html_and_sits_above_the_corner_counter(page):
    _set_2d(page)
    leg = page.evaluate("""() => { const el = document.querySelector('.hdmap-preview-legend');
      const c = document.getElementById('corner-counter');
      const a = el.getBoundingClientRect(), b = c.getBoundingClientRect();
      return { text: el.textContent, bold: el.querySelectorAll('b, i').length, a: [a.left, a.top, a.right, a.bottom],
               b: [b.left, b.top, b.right, b.bottom], pe: getComputedStyle(el).pointerEvents,
               corner: el.parentNode.className }; }""")
    assert hl.PREDICTION_BANNER in leg["text"] and "預計成果（規劃預測）" in leg["text"]
    assert "每點被拍到的張數（規劃預測）" in leg["text"] and "≥9 張（高品質）" in leg["text"]
    assert "預計成圖 0.10 km²；拍照 7 張" in leg["text"] and "CAM_MAX_ROLL 與轉彎動態未模擬（樂觀）" in leg["text"]
    assert hl.STYLE_LABELS["hole"] in leg["text"] and hl.STYLE_LABELS["points"] in leg["text"]
    assert "UAV-1" in leg["text"] and "UAV-2 <i>" in leg["text"], "每架拍照點的顏色圖例（標籤當文字）"
    assert leg["bold"] == 0 and leg["pe"] == "none" and "leaflet-bottom" in leg["corner"] and "leaflet-left" in leg["corner"]
    assert leg["a"][3] <= leg["b"][1], f"圖例蓋到角點計數框：圖例 {leg['a']}、計數框 {leg['b']}"


@needs_qt
def test_2d_hole_tooltip_shows_the_label_as_text_and_clicks_elsewhere_reach_the_map(page):
    _set_2d(page)
    # folium 用 preferCanvas：規劃路徑的 canvas 會蓋滿 overlayPane（z 400）。先放一條這種路徑，說明照樣要出得來
    page.evaluate("window.__cv = L.polyline([[%f, %f], [%f, %f]], {renderer: L.canvas()}).addTo(window.currentMap)"
                  % (*_f(0.0, 0.2), *_f(1.0, 0.2)))
    assert page.evaluate("window.currentMap.getPane('overlayPane').querySelector('canvas') !== null")
    pt = page.evaluate("""(ll) => { const p = window.currentMap.latLngToContainerPoint(ll),
      c = window.currentMap.getContainer().getBoundingClientRect(); return [p.x + c.left, p.y + c.top]; }""", _f(0.5, 0.5))
    page.mouse.move(pt[0] - 3, pt[1] - 3)
    page.mouse.move(pt[0], pt[1])
    page.wait_for_function("document.querySelector('.leaflet-tooltip') !== null", timeout=5000)
    tip = page.evaluate("() => { const t = document.querySelector('.leaflet-tooltip'); return [t.textContent, t.querySelectorAll('b').length]; }")
    assert tip == [HOLE_LABEL, 0], "漏洞說明要原樣顯示文字，<b> 不可以變成標籤"
    q = page.evaluate("""(spots) => spots.map(ll => { const m = window.currentMap, p = m.latLngToContainerPoint(ll),
      c = m.getContainer().getBoundingClientRect(); const el = document.elementFromPoint(p.x + c.left, p.y + c.top);
      return { preview: !!el.closest('.leaflet-hdmapPreview-pane'), tips: !!el.closest('.leaflet-hdmapPreviewTips-pane'),
               inMap: m.getContainer().contains(el) }; })""", [_f(0.25, 0.75), _f(0.5, 0.5)])
    assert q[0] == {"preview": False, "tips": False, "inMap": True}, "範圍內一般位置的點擊要穿過預計成果（還能加角點）"
    assert q[1] == {"preview": False, "tips": False, "inMap": True}, "漏洞上方也不能有吃滑鼠的東西（說明靠 mousemove）"
    clicks = page.evaluate("""(ll) => { const m = window.currentMap; window.__clicks = 0;
      const f = () => { window.__clicks++; }; m.on('click', f);
      const p = m.latLngToContainerPoint(ll), c = m.getContainer().getBoundingClientRect();
      const el = document.elementFromPoint(p.x + c.left, p.y + c.top);
      el.dispatchEvent(new MouseEvent('click', {bubbles: true, clientX: p.x + c.left, clientY: p.y + c.top}));
      m.off('click', f); return window.__clicks; }""", _f(0.5, 0.5))
    assert clicks == 1, "在漏洞上點擊也要傳到地圖（加角點）"
    page.evaluate("window.currentMap.removeLayer(window.__cv)")
    page.mouse.move(5, 5)


def _screenshot_rgb(pg):
    from PIL import Image
    return Image.open(io.BytesIO(pg.screenshot())).convert("RGB")


@needs_qt
def test_2d_pixels_quadrants_are_north_up_and_outside_the_extent_is_dimmed(page):
    _set_2d(page, opacity=1.0)
    pts = page.evaluate("""([b, spots]) => {
      const m = window.currentMap;
      m.fitBounds([[b[1], b[0]], [b[3], b[2]]], {animate: false, padding: [200, 200]});
      document.querySelectorAll('.usage-hint, #corner-counter, .leaflet-control-container')
        .forEach(e => { e.style.visibility = 'hidden'; });
      const c = m.getContainer().getBoundingClientRect();
      return spots.map(ll => { const p = m.latLngToContainerPoint(ll); return [Math.round(p.x + c.left), Math.round(p.y + c.top)]; });
    }""", [list(B), [_f(0.25, 0.75), _f(0.75, 0.75), _f(0.25, 0.25), _f(0.75, 0.25), _f(0.04, 0.96)]])
    page.wait_for_timeout(300)
    img = _screenshot_rgb(page)
    page.evaluate("""() => document.querySelectorAll('.usage-hint, #corner-counter, .leaflet-control-container')
      .forEach(e => { e.style.visibility = ''; })""")
    got = [img.getpixel(tuple(p)) for p in pts]
    for name, px in zip(("NW", "NE", "SW", "SE"), got[:4]):
        assert px == pytest.approx(QUAD[name], abs=12), f"{name} 象限的顏色不對（方向翻了？）：{px} vs {QUAD[name]}"
    dim = got[4]
    want = tuple(round(v * (1 - 0.55)) for v in QUAD["NW"])
    assert dim == pytest.approx(want, abs=14), f"範圍外沒有變暗：{dim}，應約 {want}"


@needs_qt
def test_2d_the_planning_wipe_keeps_the_preview_but_still_removes_ordinary_paths(page):
    _set_2d(page)
    before = _info2d(page)
    page.evaluate("window.__plainLine = L.polyline([[%f, %f], [%f, %f]]).addTo(window.currentMap)" % (*_f(0, 0), *_f(1, 1)))
    w = _bare2d_fake()
    MapWidget.clear_all_display_layers_js(w)
    page.evaluate(w.custom_page.js[-1])
    assert page.evaluate("window.currentMap.hasLayer(window.__plainLine)") is False, "對照組：一般折線要被清掉"
    after = _info2d(page)
    assert after == before, "clear_all_display_layers_js 掃掉了預計成果（Python 端狀態還在 ⇒ 畫面與狀態不一致）"


@needs_qt
def test_2d_visibility_toggle_and_clear_remove_everything(page):
    w = _set_2d(page)
    MapWidget.set_hdmap_preview_visible(w, "hdmap_plan", False)
    assert page.evaluate(w.custom_page.js[-1]) is True
    L = _info2d(page)
    assert (L["onMap"], L["image"], L["mask"], L["points"], L["holes"], L["legendShown"]) == (False, False, False, 0, 0, False)
    MapWidget.set_hdmap_preview_visible(w, "hdmap_plan", True)
    page.evaluate(w.custom_page.js[-1])
    L = _info2d(page)
    assert (L["onMap"], L["image"], L["mask"], L["points"], L["holes"], L["legendShown"]) == (True, True, True, 7, 1, True)
    MapWidget.clear_hdmap_preview(w, "hdmap_plan")
    assert page.evaluate(w.custom_page.js[-1]) is True
    left = page.evaluate("""() => { let n = 0; window.currentMap.eachLayer(l => { if (l.options && l.options.hdmapPreview) n++; });
      return { n, legends: document.querySelectorAll('.hdmap-preview-legend').length,
               img: document.querySelectorAll('.leaflet-hdmapPreviewImg-pane img').length, ids: hdmapPreviewInfo().ids }; }""")
    assert left == {"n": 0, "legends": 0, "img": 0, "ids": []}
    assert page.evaluate("clearHdmapPreview('hdmap_plan')") is False
    assert page.evaluate("setHdmapPreviewVisible('hdmap_plan', true)") is False


@needs_qt
def test_2d_load_finished_resend_replaces_whatever_the_page_has_with_the_python_state(page):
    _set_2d(page, lid="stale")                                 # 頁面上多一層 Python 端已經沒有的
    w = _bare2d_fake()
    _folium_page_html(w)
    MapWidget.set_hdmap_preview(w, "a", _payload())
    MapWidget.set_hdmap_preview(w, "b", _payload(points=[]))
    MapWidget.set_hdmap_preview_visible(w, "b", False)
    assert MapWidget._hdmap_resync_after_load(w, True) is True
    page.evaluate(w.custom_page.js[-1])
    info = page.evaluate("hdmapPreviewInfo()")
    assert info["ids"] == ["a", "b"]
    assert info["layers"]["a"]["onMap"] and info["layers"]["a"]["points"] == 7
    assert info["layers"]["b"]["onMap"] is False and info["layers"]["b"]["legendShown"] is False
    assert page.evaluate("document.querySelectorAll('.hdmap-preview-legend').length") == 2


@needs_qt
def test_2d_full_rebuild_replays_the_stored_layers_with_their_visibility(browser2d, tmp_path):
    w = _bare2d_empty()
    MapWidget.set_hdmap_preview(w, "hdmap_plan", _payload())
    MapWidget.set_hdmap_preview(w, "second", _payload(points=[]))
    MapWidget.set_hdmap_preview_visible(w, "second", False)
    f = tmp_path / "rebuilt.html"
    f.write_text(_folium_page_html(w), encoding="utf-8")
    errs = []
    pg = _open_2d(browser2d, f, errs)
    try:
        info = pg.evaluate("hdmapPreviewInfo()")
    finally:
        pg.close()
    assert not errs, errs
    assert info["ids"] == ["hdmap_plan", "second"]
    a, b = info["layers"]["hdmap_plan"], info["layers"]["second"]
    assert a["onMap"] and a["points"] == 7 and a["holes"] == 1 and a["legendShown"]
    assert b["visible"] is False and b["onMap"] is False and b["legendShown"] is False and b["counts"]["points"] == 0


@needs_qt
def test_2d_photo_poses_page_draws_and_has_no_errors_with_the_layer_control_last(browser2d, tmp_path, monkeypatch):
    """回歸：folium.LayerControl 加在疊圖之前 ⇒ 'setZIndex' 例外、整段 folium script 中止、位姿與路徑都沒畫。"""
    rows = [{"lat_deg": LAT0 + i * 1e-4, "lon_deg": LON0, "h_ell_m": 100, "yaw_deg": 90, "source": "ppk",
             "image_id": f"I{i}"} for i in range(5)]

    def run(html_name):
        w = _bare2d_empty()
        MapWidget.set_photo_poses(w, "p1", rows)
        f = tmp_path / html_name
        f.write_text(_folium_page_html(w), encoding="utf-8")
        errs = []
        pg = _open_2d(browser2d, f, errs)
        try:
            pg.wait_for_timeout(200)
            got = pg.evaluate("""() => { let cm = 0, pl = 0; window.currentMap.eachLayer(l => {
                if (l instanceof L.CircleMarker) cm++; else if (l instanceof L.Polyline) pl++; });
              const ov = Array.from(document.querySelectorAll('.leaflet-control-layers-overlays label')).map(e => e.textContent.trim());
              return { cm, pl, ov }; }""")
        finally:
            pg.close()
        return errs, got

    errs, got = run("fixed.html")
    assert errs == [], errs
    assert got["cm"] == 5 and got["pl"] >= 1, got
    assert any("照片位姿" in t for t in got["ov"]), got["ov"]

    # 負對照：把圖層控制放回疊圖之前（舊行為）⇒ 這條測試必須抓得到
    orig = MapWidget._create_base_map

    def old_base(self):
        m = orig(self)
        folium.LayerControl().add_to(m)
        return m
    monkeypatch.setattr(MapWidget, "_create_base_map", old_base)
    errs, got = run("old.html")
    assert any("setZIndex" in e for e in errs), errs
    assert got["cm"] == 0, "舊順序下位姿應該完全畫不出來（否則這個負對照量的是空氣）"


# ═══════════════════════════════════════════════════════════════════════
#  C. 真的 Cesium 1.115：_fill_cesium_template 填好的整份 main.html
# ═══════════════════════════════════════════════════════════════════════
FAR = 0.05      # 範本套回的兩層放在東邊 0.05°（約 5 km），不干擾像素測試


@pytest.fixture(scope="module")
def page3d(_pw, tmp_path_factory):
    if not _QT_OK:
        pytest.skip("Qt 匯入失敗")
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    far = lambda ring: [[la, lo + FAR] for la, lo in ring]          # noqa: E731
    shifted = dict(bounds=[B[0] + FAR, B[1], B[2] + FAR, B[3]], extent_rings=[far(_box(0.1, 0.1, 0.9, 0.9))],
                   extent_holes=[], holes=[], edge_short=[], survey_rings=[], points=[])
    w = _bare3d()
    CesiumMapWidget.set_hdmap_preview(w, "restored", _payload(**shifted))
    CesiumMapWidget.set_hdmap_preview(w, "restored_hidden", _payload(**shifted))
    CesiumMapWidget.set_hdmap_preview_visible(w, "restored_hidden", False)
    with pytest.MonkeyPatch.context() as mp:
        vals = _template_values(mp, w)
    vals.update({"__PLANE_MODEL_URI__": "", "__COPTER_MODEL_URI__": "", "__VTOL_MODEL_URI__": "",
                 "__INIT_LAT__": str(LAT0), "__INIT_LON__": str(LON0)})
    html = w._fill_cesium_template(MAIN_HTML.read_text(encoding="utf-8"), vals)
    assert "/*__HDMAP_GLUE__*/" not in html and 'setHdmapPreview("restored",' in html
    f = tmp_path_factory.mktemp("hd3d") / "main.html"
    f.write_text(html, encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
        pg = b.new_page(viewport={"width": 1000, "height": 800})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())
        pg.goto(f.as_uri())
        pg.wait_for_function("typeof viewer !== 'undefined' && typeof hdmapPreviewInfo === 'function'", timeout=60000)
    except Exception as e:                                   # pragma: no cover - 沒有 WebGL 等
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._errs = errs
    yield pg
    b.close()


@pytest.fixture
def page3(page3d):
    yield page3d
    page3d.evaluate("clearHdmapPreview(null); document.getElementById('lyr_hdmap').checked = true;"
                    "document.getElementById('lyr_imagery').checked = true; toggleLayer('imagery'); toggleLayer('hdmap');")
    assert not page3d._errs, page3d._errs


def _set_3d(pg, lid="hdmap_plan", **over):
    w = _bare3d()
    r = CesiumMapWidget.set_hdmap_preview(w, lid, _payload(**over))
    assert r["ok"] is True, r
    out = pg.evaluate(w._sent[-1])
    assert out["ok"] is True and out["id"] == lid
    pg.wait_for_function(f"hdmapPreviewInfo().layers[{json.dumps(lid)}].state === 'ready'", timeout=30000)
    return w


def _info3d(pg, lid="hdmap_plan"):
    return pg.evaluate("hdmapPreviewInfo()")["layers"][lid]


@needs_qt
def test_3d_the_template_replays_the_layers_python_remembered(page3):
    page3.wait_for_function("hdmapPreviewInfo().ids.length === 2 && hdmapPreviewInfo().layers.restored.state === 'ready'",
                            timeout=30000)
    info = page3.evaluate("hdmapPreviewInfo()")
    assert info["ids"] == ["restored", "restored_hidden"] and info["panel"] is True
    a, b = info["layers"]["restored"], info["layers"]["restored_hidden"]
    assert a["shown"] and a["imagery"] and a["imageryShow"] and a["legendShown"]
    assert a["rect"] == pytest.approx([B[0] + FAR, B[1], B[2] + FAR, B[3]], abs=1e-9)
    assert b["visible"] is False and b["shown"] is False and b["imageryShow"] is False and b["legendShown"] is False


@needs_qt
def test_3d_set_builds_a_nearest_single_tile_layer_a_nested_mask_outlines_points_and_a_legend(page3):
    _set_3d(page3)
    L = _info3d(page3)
    assert L["imagery"] and L["imageryFlag"] and L["imageryShow"] and L["alpha"] == 0.45 and L["nearest"]
    assert L["rect"] == pytest.approx(list(B), abs=1e-9), "影像範圍要正好是 bounds（格子邊緣）"
    assert L["mask"] and L["maskTopHoles"] == 1, "範圍內的洞要巢狀在範圍外框裡（島），不是另一個並列的洞"
    W, S, E, N = L["maskBox"]
    assert (B[1] - S) * 111320 == pytest.approx(20000, rel=1e-6) and (N - B[3]) * 111320 == pytest.approx(20000, rel=1e-6)
    import math
    k = 111320 * math.cos(math.radians((B[1] + B[3]) / 2))
    assert (B[0] - W) * k == pytest.approx(20000, rel=1e-6) and (E - B[2]) * k == pytest.approx(20000, rel=1e-6)
    assert (L["extent"], L["extentHoles"], L["holes"], L["edgeShort"], L["survey"]) == (1, 1, 1, 1, 1)
    assert L["points"] == 7 and L["pointsShown"] and L["pointsInScene"]
    assert L["legend"] and L["legendShown"]
    leg = page3.evaluate("""() => { const el = document.querySelector('#hdmapLegendBox .hdmap-preview-legend[data-hdmap-id="hdmap_plan"]');
      return [el.textContent, el.querySelectorAll('b, i').length, getComputedStyle(document.getElementById('hdmapLegendBox')).pointerEvents]; }""")
    assert hl.PREDICTION_BANNER in leg[0] and EDGE_LABEL not in leg[0] and leg[1] == 0 and leg[2] == "none"
    assert "UAV-2 <i>" in leg[0] and "預計成圖 0.10 km²；拍照 7 張" in leg[0]
    names = page3.evaluate("viewer.entities.values.filter(e => e._hdmapPreview).map(e => e.name)")
    assert HOLE_LABEL in names and EDGE_LABEL in names, "漏洞說明放在 entity 名稱（點選看資訊框）"


_SHOT_JS = """async ([b, spots]) => {
  const cx = (b[0] + b[2]) / 2, cy = (b[1] + b[3]) / 2;
  viewer.camera.setView({destination: Cesium.Cartesian3.fromDegrees(cx, cy, 900),
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
  ['layerPanel', 'toolbar', 'hdmapLegendBox', 'legend', 'modeLabel'].forEach(id => {
    const e = document.getElementById(id); if (e) e.style.visibility = 'hidden'; });
  const ST = Cesium.SceneTransforms, toWin = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
  const px = spots.map(ll => { const w = toWin.call(ST, viewer.scene, Cesium.Cartesian3.fromDegrees(ll[1], ll[0], 0));
                               return [Math.round(w.x), Math.round(w.y)]; });
  return {ok, px};
}"""


@needs_qt
def test_3d_pixels_quadrants_are_north_up_and_outside_the_extent_is_dimmed(page3):
    _set_3d(page3, opacity=1.0)
    spots = [_f(0.25, 0.75), _f(0.75, 0.75), _f(0.25, 0.25), _f(0.75, 0.25), _f(0.04, 0.96)]
    r = page3.evaluate(_SHOT_JS, [list(B), spots])
    img = _screenshot_rgb(page3)
    page3.evaluate("['layerPanel', 'toolbar', 'hdmapLegendBox', 'legend', 'modeLabel'].forEach(id => {"
                   " const e = document.getElementById(id); if (e) e.style.visibility = ''; })")
    assert r["ok"], "圖磚／地面多邊形一直沒載好"
    got = [img.getpixel(tuple(p)) for p in r["px"]]
    if all(max(px) < 8 for px in got):                         # pragma: no cover - 沒有 WebGL 輸出
        pytest.skip(f"畫面讀不到像素（WebGL 不可用？）：{got}")
    for name, px in zip(("NW", "NE", "SW", "SE"), got[:4]):
        assert px == pytest.approx(QUAD[name], abs=25), f"{name} 象限的顏色不對（方向翻了？）：{px} vs {QUAD[name]}"
    dim, red = got[4], got[0]
    assert dim[0] < 0.7 * red[0] and dim[0] > 0.2 * red[0], f"範圍外沒有變暗（或整片黑）：{dim} vs 範圍內 {red}"


@needs_qt
def test_3d_map_toggle_leaves_it_alone_and_the_preview_checkbox_and_visibility_hide_it(page3):
    w = _set_3d(page3)
    base = page3.evaluate("""() => { document.getElementById('lyr_imagery').checked = false; toggleLayer('imagery');
      const out = []; for (let i = 0; i < viewer.imageryLayers.length; i++) { const l = viewer.imageryLayers.get(i);
        if (!l._hdmapPreview) out.push(l.show); } return out; }""")
    assert base and not any(base), "對照組：一般底圖要被「地圖」開關關掉"
    L = _info3d(page3)
    assert L["imageryShow"] and L["shown"], "「地圖」開關不可以把預計成果一起關掉"
    page3.evaluate("document.getElementById('lyr_imagery').checked = true; toggleLayer('imagery');"
                   "document.getElementById('lyr_hdmap').checked = false; toggleLayer('hdmap');")
    L = _info3d(page3)
    assert (L["shown"], L["imageryShow"], L["extent"], L["holes"], L["pointsShown"], L["legendShown"]) == \
        (False, False, 0, 0, False, False)
    assert L["visible"] is True, "面板勾選框不改圖層自己的顯示狀態"
    page3.evaluate("document.getElementById('lyr_hdmap').checked = true; toggleLayer('hdmap');")
    assert _info3d(page3)["shown"] is True
    CesiumMapWidget.set_hdmap_preview_visible(w, "hdmap_plan", False)
    assert page3.evaluate(w._sent[-1]) is True
    L = _info3d(page3)
    assert (L["visible"], L["shown"], L["imageryShow"], L["survey"], L["pointsShown"], L["legendShown"]) == \
        (False, False, False, 0, False, False)
    CesiumMapWidget.set_hdmap_preview_visible(w, "hdmap_plan", True)
    page3.evaluate(w._sent[-1])
    L = _info3d(page3)
    assert (L["shown"], L["imageryShow"], L["survey"], L["pointsShown"], L["legendShown"]) == (True, True, 1, True, True)


@needs_qt
def test_3d_clear_removes_the_layer_entities_points_and_legend(page3):
    w = _set_3d(page3)
    CesiumMapWidget.clear_hdmap_preview(w, "hdmap_plan")
    assert page3.evaluate(w._sent[-1]) is True
    left = page3.evaluate("""() => { let il = 0; for (let i = 0; i < viewer.imageryLayers.length; i++)
        if (viewer.imageryLayers.get(i)._hdmapPreview) il++;
      let pp = 0; for (let i = 0; i < viewer.scene.primitives.length; i++) if (viewer.scene.primitives.get(i)._hdmapPreview) pp++;
      const ids = hdmapPreviewInfo().ids;
      return { il, pp, ents: viewer.entities.values.filter(e => e._hdmapPreview).length,
               legends: document.querySelectorAll('.hdmap-preview-legend[data-hdmap-id="hdmap_plan"]').length,
               has: ids.indexOf('hdmap_plan') >= 0 }; }""")
    restored_left = len([k for k in page3.evaluate("hdmapPreviewInfo().ids") if k.startswith("restored")])
    # 範本套回的兩層可能還在（若此測試先跑）：它們各有 1 個 imagery、沒有點
    assert left["has"] is False and left["legends"] == 0 and left["pp"] == 0
    assert left["il"] == restored_left
    assert page3.evaluate("clearHdmapPreview('hdmap_plan')") is False
    page3.evaluate("clearHdmapPreview(null)")
    assert page3.evaluate("""() => [viewer.entities.values.filter(e => e._hdmapPreview).length,
      document.querySelectorAll('.hdmap-preview-legend').length, hdmapPreviewInfo().ids.length]""") == [0, 0, 0]


# ═══════════════════════════════════════════════════════════════════════
#  審查修正（2026-10-19）：漏洞種類、2D 不搶滑鼠、3D 遮罩不可拾取、拍照點不擋位姿、圖例不蓋控制項
# ═══════════════════════════════════════════════════════════════════════
def test_check_payload_keeps_the_hole_kind_and_rejects_unknown_kinds():
    holes = [{"ring": _box(0.46, 0.46, 0.54, 0.54), "label": "a"},
             {"ring": _box(0.2, 0.2, 0.25, 0.25), "label": "b", "kind": "preview_only"},
             {"ring": _box(0.6, 0.6, 0.65, 0.65), "label": "c", "kind": "marker"}]
    p = hl.check_payload(_payload(holes=holes), "x")
    assert [h["kind"] for h in p["holes"]] == ["hole", "preview_only", "marker"], "沒給 kind ＝ 'hole'"
    s = hl.summary(p)
    assert (s["holes"], s["holes_preview_only"], s["hole_markers"]) == (3, 1, 1)
    with pytest.raises(ValueError, match=r"holes\[0\]\.kind"):
        hl.check_payload(_payload(holes=[dict(holes[0], kind="bogus")]), "x")
    assert "kind" not in hl.check_payload(_payload(), "x")["edge_short"][0], "邊緣不足沒有種類"


LONG_NOTES = ["規劃預測，非實際成果：由模擬拍照位置推算每格被幾張照片看到（視數），沒有影像內容",
              "未上色＝未拍到；白框＝預計可成圖範圍（≥5 張，含足跡伸出測區外的部分）",
              "紅框＝內部破洞（<5 張；上傳閘門 4 m 格判定，會擋上傳）", "橘框＝邊緣帶視數不足（不擋上傳）",
              "樂觀模型：未建模 CAM_MAX_ROLL 側傾門檻與轉彎動態，有風時實際可能較差", "格 2 m（視數以格心計，量化約 ±1 格）"]


def _long_legend_payload(**over):
    """真實規劃的圖例長度：4 級、3 架點色、6 條說明、兩行長的摘要。"""
    leg = {"title": "顏色＝每格預計被幾張照片拍到（視數）——規劃預測，非實際成果",
           "classes": [{"label": "只有 1 張（不會成圖）", "color": "#e53935"},
                       {"label": "2–4 張（邊緣，可能缺漏或扭曲）", "color": "#fb8c00"},
                       {"label": "5–8 張（可成圖）", "color": "#c0ca33"}, {"label": "≥9 張（高品質）", "color": "#43a047"}],
           "notes": list(LONG_NOTES),
           "points": [{"label": u, "color": c} for u, c in (("A1", "#1E88E5"), ("H1", "#8E24AA"), ("H2", "#00ACC1"))]}
    line = ("預計成果（規劃預測）：測區內可成圖 1.20 km²（測區 1.20 km²；白框含測區外足跡 1.38 km²），≥9 張 1.16 km²，"
            "照片 1,942 張（約 14.4–23.9 GB，每張 7.4–12.3 MB 的假設），GSD 中位 3.1 cm，內部破洞 0 處（上傳閘門 4 m 格）"
            "（規劃預測，非實際成果）")
    return _payload(legend=leg, stats_line=line, **over)


def _hover_tips(pg, ll):
    pg.mouse.move(5, 5)
    pg.wait_for_timeout(80)
    x, y = pg.evaluate("""(ll) => { const m = window.currentMap, p = m.latLngToContainerPoint(ll),
      c = m.getContainer().getBoundingClientRect(); return [p.x + c.left, p.y + c.top]; }""", ll)
    pg.mouse.move(x - 2, y - 2)
    pg.mouse.move(x, y)
    pg.wait_for_timeout(250)
    return pg.evaluate("Array.from(document.querySelectorAll('.leaflet-tooltip')).map(t => t.textContent)")


@needs_qt
def test_2d_planning_layers_inside_holes_keep_their_own_hover_and_click(page):
    """回歸：以前漏洞／邊緣不足上有 z 450 的隱形命中多邊形 ⇒ 規劃路徑 tooltip、衝突點 tooltip 變成漏洞說明、
    禁航區點了不開 popup。規劃圖層照 map_widget 的寫法（預設 canvas 繪圖器）放在漏洞與邊緣不足裡面。"""
    _set_2d(page)
    page.evaluate("""([a, c, hole, nfz]) => { const map = window.currentMap;
      window.__op = L.polyline([a, c], {color: '#ff00ff', weight: 5, opacity: 0.95}); __op.bindTooltip('Drone 1 作業段'); __op.addTo(map);
      window.__cf = L.circleMarker(hole, {radius: 14, color: '#FF1744', weight: 3, fillColor: '#FF1744', fillOpacity: 0.25});
      __cf.bindTooltip('⚠ 衝突: UAV 1 ↔ UAV 2'); __cf.addTo(map);
      window.__nfz = L.polygon(nfz, {color: '#B71C1C'}); __nfz.bindPopup('🚫 禁航區：N1'); __nfz.addTo(map);
    }""", [_f(0.0, 0.5), _f(1.0, 0.5), _f(0.52, 0.53), _box(0.47, 0.03, 0.53, 0.06)])
    try:
        assert _hover_tips(page, _f(0.48, 0.5)) == ["Drone 1 作業段"], "漏洞裡的規劃路徑要顯示它自己的說明"
        assert _hover_tips(page, _f(0.52, 0.53)) == ["⚠ 衝突: UAV 1 ↔ UAV 2"], "漏洞裡的衝突點要顯示它自己的說明"
        assert _hover_tips(page, _f(0.53, 0.47)) == [HOLE_LABEL], "漏洞裡沒有別的東西的地方才顯示漏洞說明"
        assert _hover_tips(page, _f(0.42, 0.02)) == [EDGE_LABEL]
        assert _hover_tips(page, _f(0.25, 0.75)) == [], "漏洞外不顯示"
        page.mouse.move(5, 5)
        x, y = page.evaluate("""(ll) => { const m = window.currentMap, p = m.latLngToContainerPoint(ll),
          c = m.getContainer().getBoundingClientRect(); return [p.x + c.left, p.y + c.top]; }""", _f(0.5, 0.045))
        page.mouse.click(x, y)
        page.wait_for_timeout(300)
        pops = page.evaluate("Array.from(document.querySelectorAll('.leaflet-popup-content')).map(t => t.textContent)")
        assert pops == ["🚫 禁航區：N1"], "邊緣不足裡的禁航區點了要開它的 popup"
        # 預覽關掉 ⇒ 漏洞說明也不出來
        page.evaluate("setHdmapPreviewVisible('hdmap_plan', false)")
        assert _hover_tips(page, _f(0.53, 0.47)) == []
    finally:
        page.evaluate("window.currentMap.closePopup(); ['__op', '__cf', '__nfz'].forEach(k => window.currentMap.removeLayer(window[k]));")
        page.mouse.move(5, 5)


@needs_qt
def test_2d_legend_notes_collapse_and_the_legend_never_covers_the_zoom_buttons(page):
    page.set_viewport_size({"width": 1000, "height": 560})
    try:
        page.wait_for_timeout(200)
        w = _bare2d_fake()
        MapWidget.set_hdmap_preview(w, "hdmap_plan", _long_legend_payload())
        page.evaluate(w.custom_page.js[-1])

        def rects():
            return page.evaluate("""() => { const el = document.querySelector('.hdmap-preview-legend'),
                z = document.querySelector('.leaflet-control-zoom'), n = el.querySelector('.hdmap-legend-notes');
              const a = el.getBoundingClientRect(), b = z.getBoundingClientRect();
              return { leg: [a.left, a.top, a.right, a.bottom], zoom: [b.left, b.top, b.right, b.bottom],
                       notes: getComputedStyle(n).display, text: el.innerText,
                       head: el.querySelector('.hdmap-legend-head').textContent, pe: getComputedStyle(el).pointerEvents,
                       scroll: el.scrollHeight > el.clientHeight + 1 }; }""")
        r = rects()
        assert r["notes"] == "none" and LONG_NOTES[0] not in r["text"], "說明預設收起（對話框裡都有）"
        assert "說明 6 條" in r["head"] and hl.PREDICTION_BANNER in r["text"], "收起時橫幅照樣看得到"
        assert r["leg"][1] >= r["zoom"][3], f"圖例蓋到縮放鈕：{r}"
        # 點標題列 ⇒ 展開；點擊不可以穿到地圖（不會加角點）
        page.evaluate("window.__clicks = 0; window.__cf = () => { window.__clicks++; }; window.currentMap.on('click', window.__cf);")
        hb = page.evaluate("(() => { const b = document.querySelector('.hdmap-legend-head').getBoundingClientRect(); return [b.left + 20, b.top + b.height / 2]; })()")
        page.mouse.click(*hb)
        r = rects()
        assert r["notes"] == "block" and all(n in r["text"] for n in LONG_NOTES) and "收起說明" in r["head"]
        assert page.evaluate("window.__clicks") == 0, "點圖例標題列不可以傳到地圖"
        assert r["leg"][1] >= r["zoom"][3] - 0.5, f"展開後也不能蓋到縮放鈕（放不下就捲動）：{r}"
        assert r["scroll"] and r["pe"] == "auto", "放不下 ⇒ 圖例可捲動（這時才吃滑鼠）"
        assert page.evaluate("hdmapPreviewInfo().layers.hdmap_plan.notesOpen") is True
        hb = page.evaluate("(() => { const b = document.querySelector('.hdmap-legend-head').getBoundingClientRect(); return [b.left + 20, b.top + b.height / 2]; })()")
        page.mouse.click(*hb)                                        # 展開後圖例往上長：標題列位置要重量
        assert rects()["notes"] == "none" and page.evaluate("window.__clicks") == 0
    finally:
        page.evaluate("window.currentMap.off('click', window.__cf)")
        page.set_viewport_size({"width": 1000, "height": 800})
        page.wait_for_timeout(200)


_SETTLE_3D = """async ([cx, cy, h]) => {
  viewer.camera.setView({destination: Cesium.Cartesian3.fromDegrees(cx, cy, h), orientation: {heading: 0, pitch: -Math.PI / 2, roll: 0}});
  const frame = () => new Promise(r => requestAnimationFrame(() => r()));
  for (let i = 0; i < 400; i++) { viewer.scene.requestRender(); await frame();
    const ok = viewer.dataSourceDisplay.update(viewer.clock.currentTime);
    const L = (hdmapPreviewInfo().layers || {}).hdmap_plan;
    if (i > 40 && ok && viewer.scene.globe.tilesLoaded && (!L || !L.mask || L.maskReady)) break; }
  for (let i = 0; i < 10; i++) { viewer.scene.requestRender(); await frame(); }
  return viewer.camera.positionCartographic.height; }"""
_TOWIN_3D = """(ll) => { const ST = Cesium.SceneTransforms, f = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
  const w = f.call(ST, viewer.scene, Cesium.Cartesian3.fromDegrees(ll[1], ll[0], ll[2] || 0)); return [w.x, w.y]; }"""


@needs_qt
def test_3d_dim_mask_is_not_pickable_so_viewer_click_and_double_click_do_nothing(page3):
    """回歸：遮罩以前是 entity ⇒ Viewer 預設的左鍵選取（開資訊框）、雙擊 zoomTo（飛到約 49 km）在變暗區每點都中。"""
    _set_3d(page3)
    cx, cy = (B[0] + B[2]) / 2, (B[1] + B[3]) / 2
    h0 = page3.evaluate(_SETTLE_3D, [cx, cy, 3000])
    L = _info3d(page3)
    assert L["mask"] and L["maskReady"] and L["maskShown"] and L["maskPickable"] is False
    assert page3.evaluate("viewer.entities.values.filter(e => e._hdmapPreview && e.polygon).length") == 0
    spot = [cy, B[2] + 0.006]                                         # 影像東邊 0.6 km：範圍外、在變暗框裡
    px = page3.evaluate(_TOWIN_3D, spot)
    # 對照：同一點放一個一般的點 entity ⇒ 拾取得到（證明這個位置的拾取是有效的）
    page3.evaluate("(ll) => { window.__ctl = viewer.entities.add({position: Cesium.Cartesian3.fromDegrees(ll[1], ll[0], 0),"
                   " point: {pixelSize: 24, color: Cesium.Color.YELLOW, disableDepthTestDistance: Number.POSITIVE_INFINITY}}); }",
                   spot)
    page3.evaluate(_SETTLE_3D, [cx, cy, 3000])
    ctl = page3.evaluate("(p) => { const o = viewer.scene.pick(new Cesium.Cartesian2(p[0], p[1])); return !!(o && o.id === window.__ctl); }", px)
    page3.evaluate("viewer.entities.remove(window.__ctl); viewer.scene.requestRender();")
    assert ctl, "對照組拾取不到 ⇒ 這個測試量不到東西"
    page3.evaluate(_SETTLE_3D, [cx, cy, 3000])
    picked = page3.evaluate("(p) => { const o = viewer.scene.pick(new Cesium.Cartesian2(p[0], p[1])); return o ? (o.id && o.id.name || 'primitive') : null; }", px)
    assert picked is None, f"變暗遮罩被拾取到：{picked}"
    # Viewer 預設的左鍵／雙擊動作（不經頁面自己的加角點 handler，不會導到 pyqt://）
    out = page3.evaluate("""async (p) => {
      const h = viewer.screenSpaceEventHandler, pos = new Cesium.Cartesian2(p[0], p[1]);
      h.getInputAction(Cesium.ScreenSpaceEventType.LEFT_CLICK)({position: pos});
      const sel = viewer.selectedEntity ? viewer.selectedEntity.name : null, info = viewer.infoBox.viewModel.showInfo;
      h.getInputAction(Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK)({position: pos});
      await new Promise(r => setTimeout(r, 1500));
      for (let i = 0; i < 5; i++) { viewer.scene.requestRender(); await new Promise(r => requestAnimationFrame(() => r())); }
      return {sel, info, tracked: viewer.trackedEntity ? viewer.trackedEntity.name : null,
              h: viewer.camera.positionCartographic.height}; }""", px)
    assert out["sel"] is None and out["info"] is False and out["tracked"] is None, out
    assert out["h"] == pytest.approx(h0, abs=1.0), f"雙擊後相機高度變了：{h0} → {out['h']}"


@needs_qt
def test_3d_photo_pose_hover_still_finds_poses_under_the_planned_photo_points(page3):
    """回歸：預計拍照點 disableDepthTestDistance=∞ 永遠排在拾取最前面 ⇒ 位姿的懸停說明（image_id／航向）拿不到。"""
    poses = [[*_f(x, 0.5), 0.0] for x in POINT_FX]                     # 實拍位置＝規劃的拍照點（最壞情況）
    cx, cy = (B[0] + B[2]) / 2, (B[1] + B[3]) / 2
    page3.evaluate("(rows) => setPhotoPoses('real', rows.map((r, i) => [r[0], r[1], r[2], 90, 'ppk', 'IMG_' + i]), {})", poses)
    try:
        page3.evaluate(_SETTLE_3D, [cx, cy, 600])

        def hits():
            out = []
            for q in poses:
                px = page3.evaluate(_TOWIN_3D, q)
                out.append(page3.evaluate("(p) => { const h = photoPoseAt(new Cesium.Cartesian2(p[0], p[1])); return h ? h.image_id : null; }", px))
            return out
        want = [f"IMG_{i}" for i in range(len(poses))]
        assert hits() == want, "對照組（沒有預計成果）"
        _set_3d(page3)
        page3.evaluate(_SETTLE_3D, [cx, cy, 600])
        top = page3.evaluate("(p) => hdmapPreviewOwnsPick(viewer.scene.pick(new Cesium.Cartesian2(p[0], p[1])))",
                             page3.evaluate(_TOWIN_3D, poses[0]))
        assert top is True, "前提：最上面拾取到的是預計拍照點（否則這條量不到東西）"
        assert hits() == want
    finally:
        page3.evaluate("clearPhotoPoses('real')")


@needs_qt
def test_3d_legend_never_covers_the_layer_panel_and_notes_start_collapsed(page3):
    page3.set_viewport_size({"width": 1000, "height": 600})
    try:
        page3.wait_for_timeout(200)
        w = _bare3d()
        CesiumMapWidget.set_hdmap_preview(w, "hdmap_plan", _long_legend_payload())
        page3.evaluate(w._sent[-1])
        page3.wait_for_function("hdmapPreviewInfo().layers.hdmap_plan.state === 'ready'", timeout=30000)

        def rects():
            return page3.evaluate("""() => { const host = document.getElementById('hdmapLegendBox'),
                lp = document.getElementById('layerPanel'), cb = document.getElementById('lyr_hdmap'),
                el = host.querySelector('.hdmap-preview-legend[data-hdmap-id="hdmap_plan"]');
              const a = host.getBoundingClientRect(), b = lp.getBoundingClientRect(), c = cb.getBoundingClientRect();
              return { host: [a.left, a.top, a.right, a.bottom], panel: [b.left, b.top, b.right, b.bottom],
                       cbTop: document.elementFromPoint(c.left + c.width / 2, c.top + c.height / 2) === cb,
                       notes: getComputedStyle(el.querySelector('.hdmap-legend-notes')).display, text: el.innerText }; }""")
        r = rects()
        assert r["notes"] == "none" and LONG_NOTES[0] not in r["text"] and hl.PREDICTION_BANNER in r["text"]
        assert r["host"][1] >= r["panel"][3], f"圖例蓋到圖層面板：{r}"
        assert r["cbTop"], "「預計成果」勾選框要看得到（最上面就是它）"
        page3.evaluate("document.querySelector('#hdmapLegendBox .hdmap-legend-head').click()")
        r = rects()
        assert r["notes"] == "block" and r["host"][1] >= r["panel"][3] - 0.5 and r["cbTop"], r
        page3.evaluate("document.querySelector('#hdmapLegendBox .hdmap-legend-head').click()")
    finally:
        page3.set_viewport_size({"width": 1000, "height": 800})
        page3.wait_for_timeout(200)
