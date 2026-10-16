"""風場圖層的 Python 端（ui.widgets.wind_layer）與兩個地圖 widget 發出去的 JS —— 值層級的鎖。

不開瀏覽器、不建 QWebEngineView：3D/2D widget 用裸實例（__new__），只收集送出的 JS 字串。
引擎檔 wind_flow.js 的行為由 tests/test_wind_flow_js.py 在無頭瀏覽器裡驗；這裡只鎖「頁面拿到什麼」。
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import re

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from core import wind_field as wf
from ui.widgets import wind_layer as wl

REPO = pathlib.Path(__file__).resolve().parents[1]
MAIN_HTML = REPO / "ui" / "widgets" / "cesium_templates" / "main.html"

try:
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.widgets.map_widget import MapWidget
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

BBOX = (24.0, 120.0, 24.02, 120.03)


def _field(**kw):
    return wf.uniform_field(BBOX, 3.0, 270.0, **kw)


def _call(js: str, fn: str):
    m = re.fullmatch(re.escape(fn) + r"\((.*)\)", js, flags=re.S)
    assert m, f"不是 {fn}(...)：{js[:120]}"
    return json.loads("[" + m.group(1) + "]") if m.group(1) else []


# ═══════════════════════════════════════════════════════════════════════
#  check_wind_field
# ═══════════════════════════════════════════════════════════════════════
def test_check_accepts_a_windfield_or_its_dict_and_normalises_values():
    f = _field()
    d = wl.check_wind_field(f)
    assert d == wl.check_wind_field(f.to_js())
    assert d["bbox"] == list(BBOX) and d["mode"] == "flow" and d["height_m"] == 120.0
    assert d["u"] == [[3.0]] and abs(d["v"][0][0]) < 1e-12 and "非實測" in d["label"]
    json.dumps(d, allow_nan=False)


def test_check_keeps_nulls_for_missing_cells_but_rejects_nan():
    d = {"lats": [24.0, 24.1], "lons": [120.0, 120.1], "u": [[1.0, None], [0.0, 0.0]], "v": [[0.0, 0.0], [None, 1.0]],
         "height_m": 10, "label": "x"}
    out = wl.check_wind_field(d)
    assert out["u"][0][1] is None and out["v"][1][0] is None and "bbox" not in out
    bad = dict(d, u=[[math.nan, 0.0], [0.0, 0.0]])
    with pytest.raises(ValueError, match="非有限"):
        wl.check_wind_field(bad)


@pytest.mark.parametrize("patch, msg", [
    ({"lats": [24.1, 24.0]}, "遞增"),
    ({"u": [[1.0]]}, "形狀"),
    ({"lons": [200.0, 201.0]}, "超出"),
    ({"label": "  "}, "label"),
    ({"mode": "wavy"}, "mode"),
    ({"bbox": [24.0, 120.0, 23.0, 121.0]}, "bbox"),
])
def test_check_rejects_bad_fields(patch, msg):
    d = {"lats": [24.0, 24.1], "lons": [120.0, 120.1], "u": [[1.0, 1.0], [1.0, 1.0]], "v": [[0.0, 0.0], [0.0, 0.0]],
         "height_m": 10, "label": "x"}
    d.update(patch)
    with pytest.raises(ValueError, match=msg):
        wl.check_wind_field(d)


def test_single_point_needs_a_bbox_and_non_dict_is_rejected():
    with pytest.raises(ValueError, match="bbox"):
        wl.check_wind_field({"lats": [24.0], "lons": [120.0], "u": [[1.0]], "v": [[0.0]], "height_m": 10, "label": "x"})
    with pytest.raises(ValueError):
        wl.check_wind_field([1, 2, 3])


# ═══════════════════════════════════════════════════════════════════════
#  wind_arrows_to_compact / js_call / glue / restore
# ═══════════════════════════════════════════════════════════════════════
def test_arrows_compact_skips_unplaced_or_invalid_and_counts_them():
    arrows = [{"sysid": 1, "lat": 24.1, "lon": 120.2, "alt_rel": 100.0, "dir_from_deg": 270.0, "speed_mps": 3.2},
              {"sysid": 2, "lat": None, "lon": None, "alt_rel": None, "dir_from_deg": 10.0, "speed_mps": 1.0},
              {"sysid": 3, "lat": 24.1, "lon": 120.2, "alt_rel": None, "dir_from_deg": 370.0, "speed_mps": 1.0,
               "label": "Hero"},
              {"sysid": 4, "lat": 24.1, "lon": 120.2, "alt_rel": 0.0, "dir_from_deg": math.nan, "speed_mps": 1.0},
              {"sysid": 5, "lat": 24.1, "lon": 120.2, "alt_rel": 0.0, "dir_from_deg": 0.0, "speed_mps": -1.0},
              {"sysid": "x"}]
    rows, st = wl.wind_arrows_to_compact(arrows)
    assert rows == [[1, 24.1, 120.2, 100.0, 270.0, 3.2, "UAV-1"], [3, 24.1, 120.2, 0.0, 10.0, 1.0, "Hero"]]
    assert st == {"rows": 6, "drawn": 2, "skipped": 4}
    with pytest.raises(ValueError, match="上限"):
        wl.wind_arrows_to_compact([dict(arrows[0], sysid=i) for i in range(wl.WIND_ARROWS_MAX + 1)])


def test_js_call_only_builds_page_functions_and_never_writes_nan():
    assert wl.js_call("clearWindField") == "clearWindField()"
    assert wl.js_call("setWindOptions", {"limitMps": 3.0, "label": "風"}) == 'setWindOptions({"limitMps": 3.0, "label": "風"})'
    with pytest.raises(ValueError):
        wl.js_call("alert", 1)
    with pytest.raises(ValueError):
        wl.js_call("setWindOptions", {"x": math.nan})


def test_glue_defines_the_six_page_functions_with_the_limit_and_the_right_adapter():
    for kind, marker in (("leaflet", "latLngToContainerPoint"), ("cesium", "wgs84ToWindowCoordinates")):
        js = wl.glue_js(kind, limit_mps=2.5)
        for fn in wl.PAGE_FUNCTIONS:
            assert f"function {fn}(" in js, f"{kind} 膠水缺 {fn}"
        assert marker in js and "limitMps: 2.5" in js and "AeroWind.create(" in js
    assert "viewer.container" in wl.glue_js("cesium") and "window.currentMap._container" in wl.glue_js("leaflet")
    with pytest.raises(ValueError):
        wl.glue_js("openlayers")
    with pytest.raises(ValueError):
        wl.glue_js("leaflet", limit_mps=0.0)


def test_restore_js_replays_options_then_field_then_arrows_and_nothing_when_empty():
    assert wl.restore_js(None, [], {}) == ""
    f = wl.check_wind_field(_field())
    js = wl.restore_js(f, [[1, 24.1, 120.2, 0.0, 270.0, 3.0, "UAV-1"]], {"limitMps": 3.0})
    lines = js.split("\n")
    assert lines[0].startswith("setWindOptions(") and lines[1].startswith("setWindField(") \
        and lines[2].startswith("setWindArrows(")
    assert json.loads(lines[1][len("setWindField("):-2]) == f


def test_engine_uri_is_loud_when_the_file_is_missing(monkeypatch, tmp_path):
    monkeypatch.setattr(wl, "WIND_FLOW_JS", tmp_path / "nope.js")
    with pytest.raises(FileNotFoundError, match="風場引擎"):
        wl.wind_flow_js_uri()
    eng = tmp_path / "wind_flow.js"
    eng.write_text("window.AeroWind = {};", encoding="utf-8")
    monkeypatch.setattr(wl, "WIND_FLOW_JS", eng)
    assert wl.wind_flow_js_uri() == eng.as_uri()


def test_main_html_carries_the_three_placeholders_as_harmless_comments():
    src = MAIN_HTML.read_text(encoding="utf-8")
    # 佔位只能出現一次、而且在 <head>：出現在 JS 註解裡會被換成 </script> 把整段 script 截斷
    assert src.count("__WIND_FLOW_SCRIPT__") == 1 and src.index("__WIND_FLOW_SCRIPT__") < src.index("</head>")
    assert "__WIND_FLOW_SCRIPT__" not in src[src.index("WIND_BEGIN"):src.index("// WIND_END")]
    assert "/*__WIND_GLUE__*/" in src and "/*__WIND_RESTORE__*/" in src
    assert src.index("WIND_BEGIN") < src.index("/*__WIND_GLUE__*/") < src.index("/*__WIND_RESTORE__*/") < src.index("// WIND_END")


# ═══════════════════════════════════════════════════════════════════════
#  widgets（裸實例）
# ═══════════════════════════════════════════════════════════════════════
pytestmark_qt = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")


class _NoDelCesium(CesiumMapWidget if _QT_OK else object):
    def __del__(self):
        pass


def _bare3d():
    w = _NoDelCesium.__new__(_NoDelCesium)
    w._sent = []
    w._js = w._sent.append
    return w


class _FakePage:
    def __init__(self):
        self.sent = []

    def runJavaScript(self, js, cb=None):
        self.sent.append(js)
        if cb:
            cb({"fake": True})


def _bare2d():
    w = MapWidget.__new__(MapWidget)
    w.custom_page = _FakePage()
    return w


@pytestmark_qt
@pytest.mark.parametrize("make, sent", [(_bare3d, lambda w: w._sent), (_bare2d, lambda w: w.custom_page.sent)],
                         ids=["3d", "2d"])
def test_widget_sends_checked_field_and_rows_and_keeps_state_for_replay(make, sent):
    w = make()
    cls = type(w) if make is _bare2d else CesiumMapWidget
    eff = cls.set_wind_field(w, _field())
    assert eff["cells"] == 1 and "非實測" in eff["label"]
    (f,) = _call(sent(w)[-1], "setWindField")
    assert f == wl.check_wind_field(_field()) and w._wind_field == f
    st = cls.set_wind_arrows(w, [{"sysid": 1, "lat": 24.1, "lon": 120.2, "alt_rel": 90.0, "dir_from_deg": 270.0,
                                  "speed_mps": 3.5}, {"sysid": 2, "lat": None, "lon": None, "dir_from_deg": 0,
                                                      "speed_mps": 1}])
    assert st == {"rows": 2, "drawn": 1, "skipped": 1}
    (rows,) = _call(sent(w)[-1], "setWindArrows")
    assert rows == [[1, 24.1, 120.2, 90.0, 270.0, 3.5, "UAV-1"]] and w._wind_arrows == {1: rows[0]}
    assert cls.set_wind_options(w, limitMps=2.0, particles=800) == {"limitMps": 2.0, "particles": 800}
    assert cls.set_wind_options(w, opacity=0.5) == {"limitMps": 2.0, "particles": 800, "opacity": 0.5}
    assert _call(sent(w)[-1], "setWindOptions") == [{"limitMps": 2.0, "particles": 800, "opacity": 0.5}]
    with pytest.raises(ValueError):
        cls.set_wind_options(w, limitMps=-1)
    assert cls.clear_wind_field(w) is True and sent(w)[-1] == "clearWindField()" and w._wind_field is None
    assert cls.clear_wind_field(w) is False
    assert cls.clear_wind_arrows(w) is True and sent(w)[-1] == "clearWindArrows()" and w._wind_arrows == {}
    assert cls.clear_wind_arrows(w) is False
    with pytest.raises(ValueError):
        cls.set_wind_field(w, {"lats": [1], "lons": [2], "u": [[0]], "v": [[0]], "label": "x"})
    assert w._wind_field is None, "不合法的風場不可以留下狀態"


@pytestmark_qt
def test_2d_block_loads_the_engine_then_the_glue_then_replays_state(monkeypatch, tmp_path):
    eng = tmp_path / "wind_flow.js"
    eng.write_text("window.AeroWind = {};", encoding="utf-8")
    monkeypatch.setattr(wl, "WIND_FLOW_JS", eng)
    w = _bare2d()
    assert MapWidget._wind_js_block(w).count("<script") == 2 and "setWindField({" not in MapWidget._wind_js_block(w)
    MapWidget.set_wind_field(w, _field())
    MapWidget.set_wind_options(w, limitMps=3.0)
    block = MapWidget._wind_js_block(w)
    i_src, i_glue, i_opt, i_field = (block.index(eng.as_uri()), block.index("function setWindField("),
                                     block.index("setWindOptions({"), block.index("setWindField({"))
    assert i_src < i_glue < i_opt < i_field
    # 引擎缺檔：不注入、只記一次 error、其餘照常
    monkeypatch.setattr(wl, "WIND_FLOW_JS", tmp_path / "missing.js")
    assert MapWidget._wind_js_block(w) == "" and MapWidget._wind_js_block(w) == ""
    assert w._wind_missing_logged is True


@pytestmark_qt
def test_2d_query_and_no_page_are_safe():
    w = _bare2d()
    got = []
    MapWidget.query_wind(w, got.append)
    assert w.custom_page.sent[-1] == "windInfo()" and got == [{"fake": True}]
    w2 = MapWidget.__new__(MapWidget)                         # 連 page 都還沒有：set/clear 只記狀態
    MapWidget.set_wind_field(w2, _field())
    assert w2._wind_field is not None and MapWidget.clear_wind_field(w2) is True


@pytestmark_qt
def test_dual_widget_forwards_to_both_maps():
    calls = []

    class _M:
        def __init__(self, tag):
            self.tag = tag

        def __getattr__(self, name):
            def f(*a, **k):
                calls.append((self.tag, name, a, k))
                return {"tag": self.tag} if name.startswith("set_") else True
            return f

    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = _M("2d"), _M("3d")
    assert DualMapWidget.set_wind_field(d, "F") == {"3d": {"tag": "3d"}, "2d": {"tag": "2d"}}
    assert DualMapWidget.set_wind_arrows(d, iter([1, 2])) == {"3d": {"tag": "3d"}, "2d": {"tag": "2d"}}
    assert DualMapWidget.set_wind_options(d, limitMps=3.0)["2d"] == {"tag": "2d"}
    assert DualMapWidget.clear_wind_field(d) is True and DualMapWidget.clear_wind_arrows(d) is True
    names = [(t, n) for t, n, _a, _k in calls]
    for n in ("set_wind_field", "set_wind_arrows", "set_wind_options", "clear_wind_field", "clear_wind_arrows"):
        assert ("2d", n) in names and ("3d", n) in names
    assert [a for t, n, a, k in calls if n == "set_wind_arrows"] == [([1, 2],), ([1, 2],)]
