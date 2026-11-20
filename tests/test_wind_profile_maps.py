"""分層風場（10／80／120 m）在 2D／3D 地圖上的畫法 —— 值層級的鎖（B 部分：wind_layer＋兩個頁面＋三個 widget）。

使用者 2026-10-19 定調：3D 地圖要看得出不同高度的風速，先只做 10、80、120 m 三層；不內插、不外推。

A. 純 Python（不開瀏覽器）：check_wind_profile 的正規化與拒絕、最接近層（一樣近取低的）、粒子預算、圖例字、
   js_call 的跳脫（'</'、'__'）、restore_js 順序、三個 widget（裸實例）送出的 JS 與實際生效的回傳值、雙圖轉送。
B. 真的瀏覽器（要 SAFE_ALLOW_BROWSER=1；本機 Leaflet、本機 Cesium 1.115；頁內一律擋網路）：
   * 2D folium 整頁：只畫 height_2d_m 那一層（引擎裡只有 prof:80、單一風場被藏）、左上剖面表三列、畫的那層反白、
     顏色＝speedColor、不壓到角點計數框／風場圖例／群飛圖例／預計成果圖例；Python 狀態在整頁重繪後套回；清掉後單一風場回來；
     飛控即時風箭頭不受影響。
   * 3D（Cesium＋引擎＋膠水；頁面載入 Cesium 的 widgets.css，canvas 才是真的視窗大小、螢幕量測才有意義）：
     三個引擎風場、各自的高度、粒子＝依可見面積的自動數 ÷ 層數（上限 max(80, 總數 // 顯示層數)，拉遠時全組不比單一
     風場密）、實線層／淡化層、換實線層不重生粒子也不重畫剖面柱、圖例只一個區塊、剖面柱 entity 的數量／高度／箭頭方向
     （獨立用大圓方位角算）／√ 長度／顏色／淺色文字對比、文字不蓋住箭頭（四個風向 × 俯視／斜視，用 Cesium 的 drillPick
     量）、文字底下粒子挖空、柱頂標題（3D 一層都不顯示時也有模式與時間）、每次變動都要求算圖、清掉後單一風場回來。
   * 整份 main.html（真的 _cesium_template_values，不讀設定檔）：分層風場與 3D 子集在頁面重載時套回。

payload 一律手做（core.wind_field 的分層 API 由另一部分平行撰寫；這裡不 import 它的新名字）。
"""
from __future__ import annotations

import io
import json
import math
import os
import re

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from core import wind_field as wf
from ui.widgets import wind_layer as wl
from tests.test_survey_map_layers import _pw, _launch, _REAL_ARGS, CESIUM_DIR, MAIN_HTML, LAT0, LON0  # noqa: F401

try:
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.dual_map_widget import DualMapWidget
    from ui.widgets.map_widget import MapWidget

    class _NoDel3D(CesiumMapWidget):
        """裸實例（沒跑 QWidget.__init__）回收時 __del__ 會摸 Qt 屬性；測試用這個子類。"""
        def __del__(self):
            pass
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

needs_qt = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

# ═══════════════════════════════════════════════════════════════════════
#  手做的分層風場：三層各自均勻（3×3 格點），風速落在三個顏色級距（上限 3.0：綠 <2.1 ≤ 琥珀 <3.0 ≤ 紅）
# ═══════════════════════════════════════════════════════════════════════
AREA = [(LAT0, LON0), (LAT0 + 0.005, LON0), (LAT0 + 0.005, LON0 + 0.008), (LAT0, LON0 + 0.008)]
BBOX = wf.area_bbox([AREA], margin_m=300.0)
LATS = [BBOX[0], 0.5 * (BBOX[0] + BBOX[2]), BBOX[2]]
LONS = [BBOX[1], 0.5 * (BBOX[1] + BBOX[3]), BBOX[3]]
CENTER = (LATS[1], LONS[1])
SPEC = {10.0: (2.0, 270.0), 80.0: (2.8, 250.0), 120.0: (4.5, 225.0)}     # 高度 → (風速, 來向)
GREEN, AMBER, RED = "#2fbf71", "#f5a623", "#d03b3b"
COLOR = {10.0: GREEN, 80.0: AMBER, 120.0: RED}
TEXT = "#e8eef5"                                                          # 剖面表／剖面柱的文字色（淺色）
TIME = "2026-10-19T08:00"
LEGEND = f"風場分層 10／80／120 m（icon_global 預報，非實測，{TIME} UTC）"
SINGLE = wf.uniform_field(BBOX, 3.0, 270.0)                                # 原本的單一風場（規劃風）
ARROW = {"sysid": 1, "lat": LAT0 + 0.0025, "lon": LON0 + 0.004, "alt_rel": 90.0, "dir_from_deg": 270.0,
         "speed_mps": 3.2, "label": "UAV-1 即時風"}


def _layer_label(h: float) -> str:
    return f"Open-Meteo 預報（icon_global，非實測）{h:g} m 風，{TIME} UTC；9 個模式格點"


def _layer(h: float) -> dict:
    s, d = SPEC[h]
    u, v = wf.dir_to_uv(d, s)
    return {"lats": list(LATS), "lons": list(LONS), "u": [[u] * 3 for _ in range(3)], "v": [[v] * 3 for _ in range(3)],
            "height_m": h, "label": _layer_label(h), "mode": "flow"}


def _payload(**over) -> dict:
    p = {"layers": [_layer(h) for h in (120.0, 10.0, 80.0)],                  # 故意亂序：要被排成遞增
         "column": {"lat": CENTER[0], "lon": CENTER[1],
                    "levels": [{"h_m": h, "speed_mps": SPEC[h][0], "dir_from_deg": SPEC[h][1]} for h in (80.0, 10.0, 120.0)]},
         "label": "Open-Meteo icon_global 預報（非實測）</script><b>x", "height_2d_m": 80.0,
         "particles_total": 2400, "limit_mps": 3.0}
    p.update(over)
    return p


def _call(js: str, fn: str):
    m = re.fullmatch(re.escape(fn) + r"\((.*)\)", js, flags=re.S)
    assert m, f"不是 {fn}(...)：{js[:120]}"
    return json.loads("[" + m.group(1) + "]") if m.group(1) else []


def _haversine(a, b):
    R = 6371008.8
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp, dl = p2 - p1, math.radians(b[1] - a[1])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))


def _azimuth(a, b):
    p1, p2, dl = math.radians(a[0]), math.radians(b[0]), math.radians(b[1] - a[1])
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def _ang(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _js_round(x: float) -> int:
    return int(math.floor(x + 0.5))


def _area_count(area: float) -> int:
    """引擎 areaCount 的獨立寫法：每 1200 px² 一顆（400–1500），且不密於每 25 px² 一顆（至少 20）。"""
    n = min(max(_js_round(area / 1200.0), 400), 1500)
    return min(n, max(20, _js_round(area / 25.0)))


def _share_count(area: float, n_layers: int, *, n_min: int = 80, cap: int = 800) -> int:
    """分層風場每層的粒子數（獨立寫法）：自動數 ÷ 層數；下限 min(80, 密度上限 ÷ 層數)；上限 cap。"""
    share = 1.0 / n_layers
    n = _js_round(_area_count(area) * share)
    floor = min(n_min, max(1, _js_round(max(20, _js_round(area / 25.0)) * share)))
    return min(max(n, floor), cap)


def _lum(rgb) -> float:
    def ch(c):
        c = c / 255.0
        return c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4
    r, g, b = (ch(float(x)) for x in rgb[:3])
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def _contrast(fg, bg) -> float:
    a, b = sorted((_lum(fg), _lum(bg)), reverse=True)
    return (a + 0.05) / (b + 0.05)


def _over(rgba, under) -> tuple:
    """半透明 rgba 疊在不透明 under 上（各通道線性混合）。"""
    a = rgba[3]
    return tuple(a * rgba[i] + (1.0 - a) * under[i] for i in range(3))


def _css_rgba(s: str) -> tuple:
    nums = [float(x) for x in re.findall(r"[\d.]+", s)]
    return tuple(nums[:3]) + ((nums[3],) if len(nums) > 3 else (1.0,))


def _hex_rgb(h: str) -> tuple:
    return tuple(int(h[i:i + 2], 16) for i in (1, 3, 5))


# ═══════════════════════════════════════════════════════════════════════
#  A1. check_wind_profile／最接近層／粒子預算／圖例字
# ═══════════════════════════════════════════════════════════════════════
def test_check_sorts_the_layers_resolves_the_2d_height_and_is_json_safe():
    p = wl.check_wind_profile(_payload())
    assert p["heights"] == [10.0, 80.0, 120.0] and [f["height_m"] for f in p["layers"]] == [10.0, 80.0, 120.0]
    assert p["layers"][2] == wl.check_wind_field(_layer(120.0)), "各層照單一風場的檢查（值不改）"
    assert [lv["h_m"] for lv in p["column"]["levels"]] == [10.0, 80.0, 120.0]
    assert p["height_2d_m"] == 80.0 and p["height_2d_requested_m"] == 80.0 and p["layers_3d"] is None
    assert p["particles_total"] == 2400 and p["limit_mps"] == 3.0
    assert p["legend"] == LEGEND
    assert (p["legend_head"], p["legend_desc"]) == ("風場分層 10／80／120 m", f"（icon_global 預報，非實測，{TIME} UTC）")
    json.dumps(p, allow_nan=False)


@pytest.mark.parametrize("req, eff", [(100.0, 80.0), (100.0001, 120.0), (45.0, 10.0), (45.0001, 80.0), (0.0, 10.0),
                                      (500.0, 120.0), (80.0, 80.0)])
def test_2d_height_is_the_nearest_existing_layer_ties_go_down_and_nothing_is_interpolated(req, eff):
    p = wl.check_wind_profile(_payload(height_2d_m=req))
    assert p["height_2d_m"] == eff and p["height_2d_requested_m"] == req
    assert wl.nearest_height([10, 80, 120], req) == eff


def test_missing_2d_height_falls_back_to_the_lowest_layer_and_says_so():
    p = wl.check_wind_profile(_payload(height_2d_m=None))
    assert p["height_2d_m"] == 10.0 and p["height_2d_requested_m"] is None


def test_a_dropped_layer_is_simply_absent_no_gap_filling():
    pl = _payload(layers=[_layer(10.0), _layer(80.0)], height_2d_m=120.0,
                  column={"lat": CENTER[0], "lon": CENTER[1],
                          "levels": [{"h_m": 10.0, "speed_mps": 2.0, "dir_from_deg": 270.0}]})
    p = wl.check_wind_profile(pl)
    assert p["heights"] == [10.0, 80.0] and p["height_2d_m"] == 80.0 and p["height_2d_requested_m"] == 120.0
    assert p["legend_head"] == "風場分層 10／80 m"
    assert [lv["h_m"] for lv in p["column"]["levels"]] == [10.0]
    eff = wl.profile_summary(p, "3d")
    assert eff["drawn"] == [10.0, 80.0] and eff["particles_per_layer"] == 1200 and eff["column_levels"] == 1


@pytest.mark.parametrize("over, msg", [
    ({"layers": []}, "至少要一層"),
    ({"layers": [_layer(10.0), _layer(10.0)]}, "重複"),
    ({"layers": [dict(_layer(10.0), height_m=None)]}, "height_m"),
    ({"layers": [dict(_layer(10.0), u=[[math.nan] * 3] * 3)]}, "第 1 層"),
    ({"layers": [_layer(10.0)] * 9}, "最多"),
    ({"label": "  "}, "label"),
    ({"column": {"lat": 25.0, "lon": 121.0, "levels": [{"h_m": 95.0, "speed_mps": 1.0, "dir_from_deg": 0.0}]}}, "不內插"),
    ({"column": {"lat": 25.0, "lon": 121.0, "levels": [{"h_m": 10.0, "speed_mps": math.nan, "dir_from_deg": 0.0}]}},
     "缺值"),
    ({"column": {"lat": 25.0, "lon": 121.0, "levels": [{"h_m": 10.0, "speed_mps": None, "dir_from_deg": 0.0}]}}, "缺值"),
    ({"column": {"lat": 95.0, "lon": 121.0, "levels": []}}, "位置"),
    ({"particles_total": 0}, "particles_total"),
    ({"particles_total": True}, "particles_total"),
    ({"particles_total": 1.5}, "particles_total"),
    ({"particles_total": 20001}, "particles_total"),
    ({"limit_mps": -1.0}, "limit_mps"),
    ({"height_2d_m": math.inf}, "2D 顯示高度"),
    ({"layers_3d": "80"}, "layers_3d"),
])
def test_bad_profiles_are_rejected_loudly(over, msg):
    with pytest.raises(ValueError, match=msg):
        wl.check_wind_profile(_payload(**over))
    with pytest.raises(ValueError):
        wl.check_wind_profile([_layer(10.0)])


def test_layers_3d_keeps_only_existing_layers_and_reports_the_rest():
    p = wl.check_wind_profile(_payload(layers_3d=[120, 95, 10.0000001, 120]))
    assert p["layers_3d"] == [10.0, 120.0] and p["layers_3d_ignored"] == [95.0]
    v = wl.resolve_profile_view(p, layers_3d=None)
    assert v["layers_3d"] is None and v["height_2d_m"] == 80.0, "height_2d_m=None ⇒ 不變；layers_3d=None ⇒ 全部"
    assert wl.resolve_profile_view(p, layers_3d=[])["layers_3d"] == []


@pytest.mark.parametrize("total, n, per", [(2400, 3, 800), (2400, 2, 1200), (2400, 1, 2400), (100, 3, 80),
                                           (2400, 0, 2400), (241, 3, 80)])
def test_particle_budget_per_layer(total, n, per):
    assert wl.profile_particles_per_layer(total, n) == per


def test_legend_falls_back_to_the_payload_label_and_never_drops_non_measured():
    other = [dict(_layer(h), label=f"某來源 {h:g} m") for h in (10.0, 80.0)]
    p = wl.check_wind_profile(_payload(layers=other, column=None, label="ECMWF IFS 2026-10-19 06Z"))
    assert p["legend"] == "風場分層 10／80 m（ECMWF IFS 2026-10-19 06Z；預報，非實測）"
    p = wl.check_wind_profile(_payload(layers=other, column=None, label="X 預報（非實測）"))
    assert p["legend"] == "風場分層 10／80 m（X 預報（非實測））"
    p = wl.check_wind_profile(_payload(model="gfs_seamless", time_utc="2026-10-19T09:00Z UTC"))
    assert p["legend"] == "風場分層 10／80／120 m（gfs_seamless 預報，非實測，2026-10-19T09:00Z UTC）"
    mixed = [_layer(10.0), dict(_layer(80.0), label=_layer_label(80.0).replace(TIME, "2026-10-19T09:00"))]
    p = wl.check_wind_profile(_payload(layers=mixed, column=None, label="混合"))
    assert "混合" in p["legend"], "各層時間不一致 ⇒ 不可以挑一個當全部的時間"


def test_field_ids_and_height_strings_match_the_page():
    assert [wl.profile_field_id(h) for h in (10.0, 80.0, 120.0, 12.5)] == ["prof:10", "prof:80", "prof:120", "prof:12.5"]
    assert wl.fmt_height(120.0) == "120"


# ═══════════════════════════════════════════════════════════════════════
#  A2. js_call 跳脫、restore 順序、膠水
# ═══════════════════════════════════════════════════════════════════════
def test_js_call_escapes_script_end_and_placeholder_like_text_without_changing_values():
    val = {"label": "a</script><b>__INIT_LAT__ ___x", "n": 1.5}
    js = wl.js_call("setWindOptions", val)
    assert "</" not in js and "__" not in js
    assert _call(js, "setWindOptions") == [val]


def test_restore_replays_the_profile_last():
    p = wl.check_wind_profile(_payload())
    js = wl.restore_js(wl.check_wind_field(SINGLE), [[1, 25.0, 121.0, 0.0, 270.0, 3.0, "UAV-1"]], {"limitMps": 3.0}, p)
    lines = js.split("\n")
    assert [ln.split("(")[0] for ln in lines] == ["setWindOptions", "setWindField", "setWindArrows", "setWindProfile"]
    assert _call(lines[3][:-1], "setWindProfile") == [p]
    assert wl.restore_js(None, None, None, None) == "" and wl.restore_js(None, [], {}) == ""


def test_glue_defines_the_profile_functions_and_has_no_leftover_placeholders():
    for kind in ("leaflet", "cesium"):
        js = wl.glue_js(kind)
        for fn in ("setWindProfile", "clearWindProfile", "setWindProfileView", "_windProfileDecor", "_windProfileDecorInfo"):
            assert f"function {fn}(" in js, f"{kind} 膠水缺 {fn}"
        assert not re.findall(r"__[A-Z][A-Z_]*__", js), "main.html 是逐一 .replace 佔位字：膠水不能留任何 __XXX__"
        assert "</" not in js, "2D 是內嵌 <script>"
        assert "clearField()" not in js, "單一風場的清除只能清 'default'：無 id 的 clearField 會連分層一起清掉"
    assert "PolylineArrowMaterialProperty" in wl.glue_js("cesium") and "_requestRender" in wl.glue_js("cesium")
    assert "L.control({position: 'topleft'})" in wl.glue_js("leaflet") and "_hdmapFitLegends" in wl.glue_js("leaflet")


# ═══════════════════════════════════════════════════════════════════════
#  A3. widget（裸實例：只收集送出的 JS）
# ═══════════════════════════════════════════════════════════════════════
class _FakePage:
    def __init__(self):
        self.sent = []

    def runJavaScript(self, js, cb=None):
        self.sent.append(js)


def _bare3d():
    w = _NoDel3D.__new__(_NoDel3D)
    w._sent = []
    w._js = w._sent.append
    return w


def _bare2d_fake():
    w = MapWidget.__new__(MapWidget)
    w.custom_page = _FakePage()
    return w


@needs_qt
def test_3d_widget_sends_options_then_profile_and_returns_the_effective_values():
    w = _bare3d()
    eff = CesiumMapWidget.set_wind_profile(w, _payload())
    assert [s.split("(")[0] for s in w._sent] == ["setWindOptions", "setWindProfile"]
    assert _call(w._sent[0], "setWindOptions") == [{"limitMps": 3.0}]
    (p,) = _call(w._sent[1], "setWindProfile")
    assert p == w._wind_profile and p["label"].endswith("</script><b>x")
    assert eff["drawn"] == [10.0, 80.0, 120.0] and eff["field_ids"] == ["prof:10", "prof:80", "prof:120"]
    assert eff["particles_per_layer"] == 800 and eff["column_levels"] == 3 and eff["legend"] == LEGEND
    assert eff["particles_cap_per_layer"] == 800 and eff["particles_rule"] == "每層＝依可見面積的自動粒子數 ÷ 3（至少 80），上限 800"
    assert eff["emphasis_3d_m"] == 80.0, "2D 顯示高度那層在 3D 畫實線"
    # 只換 2D 要求的高度、落在同一層（100 ⇒ 80）：3D 顯示的層與實線層都沒變 ⇒ 不送 JS（不讓頁面重設粒子）
    n = len(w._sent)
    CesiumMapWidget.set_wind_profile_view(w, height_2d_m=100.0)
    assert len(w._sent) == n and w._wind_profile["height_2d_requested_m"] == 100.0, "Python 狀態照記（重載時套回）"
    CesiumMapWidget.set_wind_profile_view(w, height_2d_m=120.0)        # 實線層換了 ⇒ 送（頁面只換樣式）
    (v,) = _call(w._sent[-1], "setWindProfileView")
    assert v["height_2d_m"] == 120.0 and len(w._sent) == n + 1
    CesiumMapWidget.set_wind_profile_view(w, height_2d_m=80.0)
    CesiumMapWidget.set_wind_profile_view(w, layers_3d=[120, 80, 95])
    (v,) = _call(w._sent[-1], "setWindProfileView")
    assert v == {"height_2d_m": 80.0, "height_2d_requested_m": 80.0, "layers_3d": [80.0, 120.0], "layers_3d_ignored": [95.0]}
    assert w._wind_profile["layers_3d"] == [80.0, 120.0], "重載時要套回同一個子集"
    n = len(w._sent)
    with pytest.raises(ValueError):
        CesiumMapWidget.set_wind_profile(w, _payload(label=""))
    assert len(w._sent) == n and w._wind_profile["layers_3d"] == [80.0, 120.0], "不合法 ⇒ 不送、舊的不動"
    assert CesiumMapWidget.clear_wind_profile(w) is True and w._sent[-1] == "clearWindProfile()"
    assert CesiumMapWidget.clear_wind_profile(w) is False and w._wind_profile is None
    n = len(w._sent)
    CesiumMapWidget.set_wind_profile_view(w, height_2d_m=10)           # 沒有分層風場 ⇒ 什麼都不做
    assert len(w._sent) == n


@needs_qt
def test_2d_widget_draws_one_layer_and_the_view_change_is_resolved_in_python():
    w = _bare2d_fake()
    eff = MapWidget.set_wind_profile(w, _payload(height_2d_m=100.0))
    assert eff["drawn"] == [80.0] and eff["height_2d_requested_m"] == 100.0 and eff["particles_per_layer"] is None
    MapWidget.set_wind_profile_view(w, height_2d_m=120)
    (v,) = _call(w.custom_page.sent[-1], "setWindProfileView")
    assert v["height_2d_m"] == 120.0 and v["layers_3d"] is None
    with pytest.raises(ValueError):
        MapWidget.set_wind_profile_view(w, height_2d_m="高")
    assert w._wind_profile["height_2d_m"] == 120.0
    w2 = MapWidget.__new__(MapWidget)                                  # 連 page 都還沒有：只記狀態
    MapWidget.set_wind_profile(w2, _payload())
    assert w2._wind_profile is not None and MapWidget.clear_wind_profile(w2) is True


@needs_qt
def test_2d_block_replays_field_arrows_then_profile_after_a_full_rebuild():
    w = _bare2d_fake()
    MapWidget.set_wind_field(w, SINGLE)
    MapWidget.set_wind_profile(w, _payload())
    MapWidget.set_wind_profile_view(w, height_2d_m=10.0)
    block = MapWidget._wind_js_block(w)
    i_glue, i_field, i_prof = (block.index("function setWindProfile("), block.index("setWindField({"),
                               block.index("setWindProfile({"))
    assert i_glue < i_field < i_prof
    line = next(ln for ln in block.split("\n") if ln.startswith("setWindProfile("))
    (p,) = _call(line[:-1], "setWindProfile")
    assert p["height_2d_m"] == 10.0, "整頁重繪後要畫的是最後選的那層"
    assert "</script><b>" not in block, "內嵌 <script> 裡的 '</' 要跳脫"


@needs_qt
def test_a_profile_changed_while_the_page_was_loading_is_resent_once_after_load():
    """整頁重繪／3D 重載期間才 set／clear／換高度的分層風場會送到舊頁面上 ⇒ 載入完成時依版本補送一次。"""
    w = _bare2d_fake()
    MapWidget._wind_js_block(w)                                         # 產生 HTML（那一刻沒有分層風場）
    MapWidget.set_wind_profile(w, _payload())                           # 載入期間才設
    w.custom_page.sent.clear()
    assert MapWidget._wind_profile_resync_after_load(w, False) is False, "載入失敗不補送"
    assert MapWidget._wind_profile_resync_after_load(w, True) is True
    assert _call(w.custom_page.sent[-1], "setWindProfile") == [w._wind_profile]
    assert MapWidget._wind_profile_resync_after_load(w, True) is False, "同一版只補一次"
    MapWidget._wind_js_block(w)                                         # 新 HTML 已帶著目前的狀態 ⇒ 不用補
    assert MapWidget._wind_profile_resync_after_load(w, True) is False
    MapWidget.set_wind_profile_view(w, height_2d_m=10.0)
    assert MapWidget._wind_profile_resync_after_load(w, True) is True
    assert _call(w.custom_page.sent[-1], "setWindProfile")[0]["height_2d_m"] == 10.0
    MapWidget.clear_wind_profile(w)
    assert MapWidget._wind_profile_resync_after_load(w, True) is True and w.custom_page.sent[-1] == "clearWindProfile()"
    w3 = _bare3d()
    CesiumMapWidget.set_wind_profile(w3, _payload())
    assert CesiumMapWidget._wind_profile_resync_after_load(w3, True) is True
    assert _call(w3._sent[-1], "setWindProfile") == [w3._wind_profile]
    assert CesiumMapWidget._wind_profile_resync_after_load(w3, True) is False


@needs_qt
def test_dual_forwards_the_three_profile_methods_to_both_maps():
    calls = []

    class _M:
        def __init__(self, tag):
            self.tag = tag

        def __getattr__(self, name):
            def f(*a, **k):
                calls.append((self.tag, name, a, k))
                return {"tag": self.tag} if name == "set_wind_profile" else (self.tag == "2d")
            return f

    d = DualMapWidget.__new__(DualMapWidget)
    d.map_2d, d.map_3d = _M("2d"), _M("3d")
    assert DualMapWidget.set_wind_profile(d, {"x": 1}) == {"2d": {"tag": "2d"}, "3d": {"tag": "3d"}}
    assert DualMapWidget.clear_wind_profile(d) is True
    assert DualMapWidget.set_wind_profile_view(d, height_2d_m=80.0) is None
    got = {(t, n): k for t, n, _a, k in calls}
    for t in ("2d", "3d"):
        assert (t, "set_wind_profile") in got and (t, "clear_wind_profile") in got
        assert got[(t, "set_wind_profile_view")] == {"height_2d_m": 80.0, "layers_3d": None}


# ═══════════════════════════════════════════════════════════════════════
#  B. 真的瀏覽器
# ═══════════════════════════════════════════════════════════════════════
def _route_local_only(pg):
    pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())


@pytest.fixture(scope="module")
def browser(_pw):
    if not _QT_OK:
        pytest.skip(f"Qt 匯入失敗：{_QT_ERR}")
    try:
        b = _launch(_pw, _REAL_ARGS)
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    yield b
    b.close()


def _open(browser, path, ready, size=(800, 600)):
    pg = browser.new_page(viewport={"width": size[0], "height": size[1]})
    errs, cons = [], []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.on("console", lambda m: cons.append(m.text) if m.type in ("error", "warning") else None)
    _route_local_only(pg)
    pg.goto(path.as_uri())
    pg.wait_for_function(ready, timeout=60000)
    pg._errs, pg._cons = errs, cons
    return pg


def _wind_noise(pg):
    """本頁跟風場有關的錯誤／警告（被擋掉的 CDN 外掛不算）。"""
    return [e for e in pg._errs + pg._cons if "Wind" in e or "風" in e or "AeroWind" in e]


# ── 2D：folium 整頁（與 MapWidget._render_map 同一條路）──
def _bare2d_page(**state):
    w = MapWidget.__new__(MapWidget)
    for k, v in dict(corners=list(AREA), markers=[], paths=[], path_colors=[], _path_tooltips=[],
                     transit_paths=[], _circle_center=None, _circle_radius_m=0.0, _home_point=None,
                     _nfz_zones=[], _geofence=None, _swarm_data=None, _view_state=None,
                     _ortho_tiles=[], _photo_pose_layers={}, _survey_point_layers={},
                     _wind_field=None, _wind_arrows={}, _wind_options={}).items():
        setattr(w, k, v)
    for k, v in state.items():
        setattr(w, k, v)
    return w


def _folium_page_html(w) -> str:
    m = w._build_map()
    buf = io.BytesIO()
    m.save(buf, close_file=False)
    html = buf.getvalue().decode("utf-8")
    html = MapWidget._rewrite_leaflet_to_local(w, html)
    var = re.search(r"var\s+(map_[a-f0-9]+)\s*=\s*L\.map", html).group(1)
    return MapWidget._inject_javascript(w, html, var)


def _hdmap_payload():
    try:
        from tests.test_hdmap_preview_layer import _payload as hp
    except Exception as e:                                   # pragma: no cover - 環境相依
        pytest.skip(f"預計成果測試的 payload 取不到：{e}")
    return hp()


@pytest.fixture(scope="module")
def page2d(browser, tmp_path_factory):
    w = _bare2d_page()
    MapWidget.set_wind_field(w, SINGLE)                        # 建頁前就有狀態 ⇒ 頁面載入時要自己套回去
    MapWidget.set_wind_arrows(w, [ARROW])
    MapWidget.set_wind_profile(w, _payload())
    hd = MapWidget.set_hdmap_preview(w, "hdmap_plan", _hdmap_payload())
    assert hd["ok"], hd
    f = tmp_path_factory.mktemp("p2d") / "folium.html"
    f.write_text(_folium_page_html(w), encoding="utf-8")
    pg = _open(browser, f, "window.currentMap && typeof windInfo === 'function' && windInfo().profile")
    pg._w = w
    yield pg
    pg.close()


def test_2d_draws_only_the_selected_layer_and_hides_the_single_field(page2d):
    info = page2d.evaluate("windInfo()")
    prof, st = info["profile"], info["state"]
    assert info["options"]["trailsFollowView"] is False and st["options"]["trailsFollowView"] is False, \
        "2D 維持舊行為：拖曳時清掉、放開後重新累積（trailsFollowView 只給 3D 追機）"
    assert prof["drawn"] == [80.0] and prof["fieldIds"] == ["prof:80"] and prof["particles_per_layer"] is None
    assert set(st["fields"]) == {"prof:80"}, "2D 俯視只畫一層；單一風場被藏起來"
    f80 = st["fields"]["prof:80"]
    assert f80["height_m"] == 80.0 and f80["n_requested"] == 0 and f80["share"] == 0, "2D 粒子用單一風場的自動值"
    assert f80["faint"] is False and f80["group"] == "profile"
    assert f80["max_speed"] == pytest.approx(2.8)
    assert info["field"] is True, "單一風場仍記著（清掉分層後要畫回來）"
    assert st["arrows"] == 1 and st["arrowLayers"] == {"default": 1}, "飛控即時風箭頭不受影響"
    assert prof["label"] == "Open-Meteo icon_global 預報（非實測）</script><b>x", "內嵌 script 的 '</' 跳脫後值不變"
    page2d.wait_for_function("windInfo().state.frames > 40 && windInfo().state.fields['prof:80'].n_alive > 0",
                             timeout=20000)
    page2d.wait_for_function("windInfo().state.legend.length > 1", timeout=10000)
    leg = page2d.evaluate("windInfo().state.legend")
    # 短的兩行：層＋最大風速 ／ 模式、非實測、有效時間（舊的長 label 一定被引擎 360 px 截斷，時間與最大值都看不到）
    assert leg[0] == "風場分層 80 m（2D 只畫這層）　最大 2.8 m/s"
    assert leg[1] == f"（icon_global 預報，非實測，{TIME} UTC）"
    assert not [ln for ln in leg if ln.endswith("…")], f"圖例行不可以被截斷：{leg}"


def test_2d_profile_table_lists_the_three_layers_and_highlights_the_drawn_one(page2d):
    d = page2d.evaluate("windInfo().profile.decor")
    assert d["kind"] == "table" and d["present"] is True
    assert [r["h"] for r in d["rows"]] == ["120", "80", "10"], "高的在上面"
    assert [r["drawn"] for r in d["rows"]] == [False, True, False]
    assert [r["color"] for r in d["rows"]] == [RED, AMBER, GREEN], "色塊顏色＝引擎 speedColor（上限 3.0）"
    for r, h in zip(d["rows"], (120.0, 80.0, 10.0)):
        assert f"{h:g} m" in r["text"] and f"{SPEC[h][0]:.1f} m/s" in r["text"]
        assert f"來自 {SPEC[h][1]:.0f}°" in r["text"], "數字是來向：要寫「來自」（旁邊的箭頭指的是順風方向）"
    assert f"icon_global 預報，非實測，{TIME} UTC" in d["text"] and "剖面點" in d["text"] and "不內插" in d["text"]
    assert "2D 只畫 80 m" in d["text"] and "方向＝風的來向" in d["text"]


def test_2d_table_text_is_readable_on_any_basemap_and_the_drawn_row_too(page2d):
    """文字一律淺色；疊在最亮的底圖（白）上、連反白的那一列都要 ≥ 4.5:1（紅字在深底上只有 ~3:1、反白列 1.6:1）。"""
    d = page2d.evaluate("windInfo().profile.decor")
    tbl = _css_rgba(d["bg"])
    for r in d["rows"]:
        assert _css_rgba(r["text_color"])[:3] == _hex_rgb(TEXT), r
        bg = _over(tbl, (255.0, 255.0, 255.0))
        row = _css_rgba(r["row_bg"])
        if row[3] > 0:
            bg = _over(row, bg)
        assert _contrast(_hex_rgb(TEXT), bg) >= 4.5, (r, bg)


def test_2d_table_stays_clear_of_the_counter_legends_and_the_zoom_buttons(page2d):
    page2d.evaluate("""(html) => document.body.insertAdjacentHTML('beforeend', html)""",
                    MapWidget._build_swarm_legend(page2d._w, {"drones": [{"drone_id": 1}, {"drone_id": 2}],
                                                             "stats": {"total_distance": 1000.0}}))
    page2d.wait_for_function("windInfo().state.legendBox !== null", timeout=10000)
    r = page2d.evaluate("""() => {
      const rect = (e) => { if (!e) return null; const b = e.getBoundingClientRect(); return [b.left, b.top, b.right, b.bottom]; };
      const tbl = document.querySelector('.aerowind-profile-table');
      const mc = window.currentMap._container.getBoundingClientRect();
      const lb = windInfo().state.legendBox;
      const swarm = [...document.body.children].find(e => e.textContent.includes('群飛路徑圖例'));
      const hd = document.querySelector('.hdmap-preview-legend');
      return { tbl: rect(tbl), parent: tbl.parentNode.className, counter: rect(document.getElementById('corner-counter')),
               wind: [mc.left + lb[0], mc.top + lb[1], mc.left + lb[0] + lb[2], mc.top + lb[1] + lb[3]],
               swarm: rect(swarm), hdmap: rect(hd), zoom: rect(document.querySelector('.leaflet-control-zoom')),
               W: innerWidth, H: innerHeight };
    }""")
    assert "leaflet-top" in r["parent"] and "leaflet-left" in r["parent"], "放在左上控制項的堆疊裡（縮放鈕下面）"
    t = r["tbl"]
    assert t[1] >= r["zoom"][3] and t[2] - t[0] <= 260 and t[3] - t[1] < r["H"] * 0.5, r

    def overlap(a, b):
        return a and b and a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]
    for name in ("counter", "wind", "swarm", "hdmap"):
        assert r[name] is not None, f"{name} 沒畫出來，量不到"
        assert not overlap(t, r[name]), f"剖面表壓到 {name}：{t} vs {r[name]}"


def _overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


@pytest.mark.parametrize("width", [800, 646, 600, 520])
def test_2d_wind_legend_never_covers_the_table_on_narrow_maps(browser, tmp_path, width):
    """地圖窄（半個螢幕、拖動分隔線）時右上的風場圖例會伸到左上的剖面表 ⇒ 圖例移到表格下方；夠寬時不移。
    圖例兩行都完整（最大風速、有效時間不被截斷）。"""
    w = _bare2d_page()
    MapWidget.set_wind_field(w, SINGLE)
    MapWidget.set_wind_arrows(w, [ARROW])
    MapWidget.set_wind_profile(w, _payload())
    f = tmp_path / f"narrow_{width}.html"
    f.write_text(_folium_page_html(w), encoding="utf-8")
    pg = _open(browser, f, "window.currentMap && typeof windInfo === 'function' && windInfo().profile", size=(width, 600))
    js = """() => { const mc = window.currentMap._container.getBoundingClientRect(), lb = windInfo().state.legendBox;
      const b = (s) => { const r = document.querySelector(s).getBoundingClientRect(); return [r.left, r.top, r.right, r.bottom]; };
      return { tbl: b('.aerowind-profile-table'), zoom: b('.leaflet-control-zoom'),
               leg: [mc.left + lb[0], mc.top + lb[1], mc.left + lb[0] + lb[2], mc.top + lb[1] + lb[3]],
               legend: windInfo().state.legend, shift: windInfo().profile.decor.legend_shift, W: innerWidth, H: innerHeight }; }"""
    try:
        pg.wait_for_function("windInfo().state.legendBox !== null && windInfo().state.legend.length > 2", timeout=20000)
        pg.wait_for_timeout(400)                                   # 兩個 rAF 之後才量、移；再等引擎重畫圖例
        r = pg.evaluate(js)
        # 視窗之後被拉窄（分隔線）：Leaflet resize ⇒ 重量
        pg.set_viewport_size({"width": max(480, width - 120), "height": 600})
        pg.evaluate("window.currentMap.invalidateSize()")
        pg.wait_for_timeout(500)
        r2 = pg.evaluate(js)
    finally:
        pg.close()
    for rr in (r, r2):
        assert not _overlap(rr["tbl"], rr["leg"]), f"風場圖例壓到剖面表：{rr}"
        assert not _overlap(rr["zoom"], rr["leg"]), rr
        assert rr["leg"][2] <= rr["W"] + 0.5 and rr["leg"][3] <= rr["H"] + 0.5, "移下去之後整個圖例還在畫面裡"
        assert rr["legend"][0] == "風場分層 80 m（2D 只畫這層）　最大 2.8 m/s"
        assert rr["legend"][1] == f"（icon_global 預報，非實測，{TIME} UTC）"
    if width >= 800:
        assert r["shift"] is None and r["leg"][1] < r["tbl"][3], "夠寬：圖例留在原位（右上）"
    if width <= 520:
        assert r["shift"] is not None and r["leg"][1] >= r["tbl"][3], f"窄：圖例移到表格下方：{r}"
    assert not _wind_noise(pg), _wind_noise(pg)


def test_2d_table_added_live_makes_the_hdmap_legend_refit_on_a_short_map(browser, tmp_path):
    """矮的地圖（420 px）：預計成果圖例自然高度會伸到左上角。表格是之後才（runJavaScript）加上去的 ⇒
    要請預計成果圖例重新量上限，否則兩個疊在一起。"""
    w = _bare2d_page()
    MapWidget.set_wind_profile(w, _payload())
    assert MapWidget.set_hdmap_preview(w, "hdmap_plan", _hdmap_payload())["ok"]
    f = tmp_path / "short.html"
    f.write_text(_folium_page_html(w), encoding="utf-8")
    pg = _open(browser, f, "window.currentMap && typeof windInfo === 'function' && windInfo().profile", size=(800, 420))
    js = """() => { const b = (s) => { const r = document.querySelector(s).getBoundingClientRect(); return [r.left, r.top, r.right, r.bottom]; };
                   return { t: b('.aerowind-profile-table'), h: b('.hdmap-preview-legend') }; }"""
    try:
        at_load = pg.evaluate(js)
        pg.evaluate("clearWindProfile(); _hdmapFitLegends();")                 # 沒有表格 ⇒ 預計成果圖例長回自然高度
        grown = pg.evaluate("document.querySelector('.hdmap-preview-legend').getBoundingClientRect().top")
        assert pg.evaluate("(p) => setWindProfile(p)", w._wind_profile) is True
        live = pg.evaluate(js)
    finally:
        pg.close()

    def overlap(a, b):
        return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]
    assert grown < live["t"][3], f"這個尺寸下預計成果圖例要真的伸得到表格的高度，測試才有意義：{grown} vs {live['t']}"
    assert not overlap(at_load["t"], at_load["h"]), at_load
    assert not overlap(live["t"], live["h"]), f"即時加上的剖面表壓到預計成果圖例：{live}"
    assert not _wind_noise(pg), _wind_noise(pg)


def test_2d_view_change_clear_and_redraw_keep_arrows_and_restore_the_single_field(page2d):
    p = page2d._w._wind_profile
    view = wl.resolve_profile_view(p, height_2d_m=120.0)
    assert page2d.evaluate("(v) => setWindProfileView(v)", view) is True
    info = page2d.evaluate("windInfo()")
    assert set(info["state"]["fields"]) == {"prof:120"} and info["profile"]["drawn"] == [120.0]
    assert [r["drawn"] for r in info["profile"]["decor"]["rows"]] == [True, False, False]
    assert page2d.evaluate("clearWindProfile()") is True
    info = page2d.evaluate("windInfo()")
    assert info["profile"] is None and set(info["state"]["fields"]) == {"default"}
    assert info["state"]["label"] == SINGLE.label, "清掉分層 ⇒ 原本的單一風場畫回來"
    assert page2d.evaluate("document.querySelector('.aerowind-profile-table')") is None
    assert info["state"]["arrows"] == 1
    # 分層設著時再換單一風場：只記著、不畫；清掉分層後畫的是新的
    assert page2d.evaluate("(p) => setWindProfile(p)", p) is True
    new_single = wl.check_wind_field(wf.uniform_field(BBOX, 1.0, 90.0, label="新的規劃風（非實測）"))
    assert page2d.evaluate("(f) => setWindField(f)", new_single) is True
    assert set(page2d.evaluate("windInfo().state.fields")) == {"prof:80"}, "Python 端記的是 80 m"
    assert page2d.evaluate("clearWindProfile() && windInfo().state.label") == "新的規劃風（非實測）"
    assert page2d.evaluate("clearWindProfile()") is False
    assert not _wind_noise(page2d), _wind_noise(page2d)


def test_2d_python_state_is_replayed_after_a_full_rebuild(browser, tmp_path):
    w = _bare2d_page()
    MapWidget.set_wind_field(w, SINGLE)
    MapWidget.set_wind_profile(w, _payload())
    MapWidget.set_wind_profile_view(w, height_2d_m=10.0)
    f = tmp_path / "rebuilt.html"
    f.write_text(_folium_page_html(w), encoding="utf-8")
    pg = _open(browser, f, "window.currentMap && typeof windInfo === 'function' && windInfo().profile")
    try:
        info = pg.evaluate("windInfo()")
    finally:
        pg.close()
    assert info["profile"]["drawn"] == [10.0] and set(info["state"]["fields"]) == {"prof:10"}
    assert [r["drawn"] for r in info["profile"]["decor"]["rows"]] == [False, False, True]
    assert not _wind_noise(pg), _wind_noise(pg)


# ── 3D：真的 Cesium 1.115 ＋ 引擎 ＋ cesium 膠水 ──
_ENTS = """() => {
  const now = Cesium.JulianDate.now(), D = Cesium.Math.toDegrees;
  const hex = (c) => '#' + [c.red, c.green, c.blue].map(x => Math.round(x * 255).toString(16).padStart(2, '0')).join('');
  const geo = (p) => { const c = Cesium.Cartographic.fromCartesian(p); return [D(c.latitude), D(c.longitude), c.height]; };
  return viewer.entities.values.filter(e => (e.name || '').indexOf('風場剖面') === 0).map(e => {
    const o = { name: e.name };
    if (e.position) o.pos = geo(e.position.getValue(now));
    if (e.polyline) {
      o.line = e.polyline.positions.getValue(now).map(geo);
      o.arrow = e.polyline.material instanceof Cesium.PolylineArrowMaterialProperty;
      o.lineColor = hex(e.polyline.material.color.getValue(now));
      o.width = e.polyline.width.getValue(now);
    }
    if (e.point) o.pointColor = hex(e.point.color.getValue(now));
    if (e.label) { o.text = e.label.text.getValue(now); o.labelColor = hex(e.label.fillColor.getValue(now)); }
    return o;
  });
}"""


@pytest.fixture(scope="module")
def page3d(browser, tmp_path_factory):
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    d = tmp_path_factory.mktemp("p3d")
    page_file = d / "p3d.html"
    page_file.write_text(f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<link rel="stylesheet" href="{(CESIUM_DIR / 'Widgets' / 'widgets.css').as_uri()}">
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
viewer.camera.setView({{destination: Cesium.Cartesian3.fromDegrees({CENTER[1]}, {CENTER[0]}, 1500),
                        orientation: {{heading: 0, pitch: -Math.PI / 2, roll: 0}}}});
window.__rr = 0;
window._requestRender = function () {{ window.__rr++; viewer.scene.requestRender(); }};
{wl.glue_js('cesium')}
</script></body></html>""", encoding="utf-8")
    pg = _open(browser, page_file, "window.viewer !== undefined && typeof windInfo === 'function' && windInfo().engine")
    # 沒有 widgets.css 時 Cesium canvas 是 300×150、引擎覆蓋層卻是 800×600：螢幕上的量測（面積、文字與箭頭）全不準
    assert pg.evaluate("[viewer.scene.canvas.clientWidth, viewer.scene.canvas.clientHeight]") == [800, 600]
    pg.evaluate("(f) => setWindField(f)", wl.check_wind_field(SINGLE))
    pg.evaluate("(r) => setWindArrows(r)", wl.wind_arrows_to_compact([ARROW])[0])
    pg._p = wl.check_wind_profile(_payload())
    rr0 = pg.evaluate("window.__rr")
    assert pg.evaluate("(p) => setWindProfile(p)", pg._p) is True
    assert pg.evaluate("window.__rr") > rr0, "加剖面柱要要求算圖（requestRenderMode）"
    yield pg
    pg.close()


_ALL_ALIVE = ("(() => { const f = windInfo().state.fields; return ['prof:10','prof:80','prof:120']"
              ".every(k => f[k] && f[k].n_alive > 0 && f[k].n_particles > 0); })()")


def _level_ents(ents):
    """剖面柱上每一層的 entity（依高度）；柱頂標題也有位置（在最高層），要排除。"""
    return {round(e["pos"][2]): e for e in ents if "pos" in e and "標題" not in e["name"]}


def _set_cam(pg, *, height=None, heading=0.0, pitch=-90.0, rng=None):
    pg.evaluate("""(a) => { const C = Cesium; viewer.camera.cancelFlight();
      if (a.rng === null) {
        viewer.camera.setView({destination: C.Cartesian3.fromDegrees(a.lon, a.lat, a.height),
                               orientation: {heading: C.Math.toRadians(a.heading), pitch: C.Math.toRadians(a.pitch), roll: 0}});
      } else {
        viewer.camera.lookAt(C.Cartesian3.fromDegrees(a.lon, a.lat, 60),
                             new C.HeadingPitchRange(C.Math.toRadians(a.heading), C.Math.toRadians(a.pitch), a.rng));
        viewer.camera.lookAtTransform(C.Matrix4.IDENTITY);
      }
      _requestRender(); }""", {"lat": CENTER[0], "lon": CENTER[1], "height": height, "heading": heading,
                               "pitch": pitch, "rng": rng})


def _settle(pg, ms=500):
    """entity 的屬性在下一個 clock tick 才套到圖元上；preRender 換邊後還要再算一次圖。"""
    for _ in range(3):
        pg.evaluate("_requestRender()")
        pg.wait_for_timeout(ms // 3)


def test_3d_each_layer_is_its_own_field_at_its_own_height_with_an_area_shared_budget(page3d):
    page3d.wait_for_function("windInfo().state.frames > 40 && " + _ALL_ALIVE, timeout=30000)
    info = page3d.evaluate("windInfo()")
    st, prof = info["state"], info["profile"]
    assert set(st["fields"]) == {"prof:10", "prof:80", "prof:120"}, "單一風場被藏；三層各一個引擎風場"
    total = 0
    for h in (10.0, 80.0, 120.0):
        f = st["fields"][wl.profile_field_id(h)]
        assert f["height_m"] == h and f["group"] == "profile" and f["max_speed"] == pytest.approx(SPEC[h][0])
        assert f["n_requested"] == 800 and f["share"] == pytest.approx(1 / 3) and f["n_min"] == 80, "800 是上限"
        assert f["visible_area_px"] > 50000, "canvas 是真的 800×600（widgets.css）：可見面積才是真的"
        assert f["n_particles"] == _share_count(f["visible_area_px"], 3), (h, f["visible_area_px"], f["n_particles"])
        assert f["n_particles"] < 800, "依面積平分：一般視角遠低於上限（舊的固定 3×800 是單一風場的 5–6 倍）"
        assert f["faint"] is (h != 80.0), "2D 顯示高度（80 m）那層實線，其餘淡化"
        total += f["n_particles"]
    assert total <= 1500, "三層加起來不超過單一風場的自動上限"
    assert prof["drawn"] == [10.0, 80.0, 120.0] and prof["particles_per_layer"] == 800
    assert prof["particles_actual"] == total and prof["emphasis_3d_m"] == 80.0
    assert info["field"] is True and st["arrows"] == 1 and st["arrowLayers"] == {"default": 1}


def test_3d_profile_budget_matches_the_single_field_and_zoomed_out_is_not_denser(page3d):
    p = page3d._p
    try:
        assert page3d.evaluate("clearWindProfile()") is True       # 同一個視角：單一風場（規劃風）畫回來
        page3d.wait_for_function("(() => { const f = windInfo().state.fields.default; return f && f.n_particles > 0; })()",
                                 timeout=20000)
        single = page3d.evaluate("windInfo().state.fields.default")
        assert single["n_particles"] == _area_count(single["visible_area_px"])
        assert page3d.evaluate("(p) => setWindProfile(p)", p) is True
        page3d.wait_for_function(_ALL_ALIVE, timeout=20000)
        f = page3d.evaluate("windInfo().state.fields")
        total = sum(f[wl.profile_field_id(h)]["n_particles"] for h in (10.0, 80.0, 120.0))
        assert total <= single["n_particles"] + 3, f"分層三層合計不比它取代的單一風場多：{total} vs {single['n_particles']}"
        # 拉到 20 km 高：整區只剩幾千 px²。舊的固定 3×800 會擠成一團；現在每層守 面積密度上限 ÷ 3
        _set_cam(page3d, height=20000.0)
        page3d.evaluate("(p) => { clearWindProfile(); setWindProfile(p); }", p)
        page3d.wait_for_function(_ALL_ALIVE, timeout=20000)
        f = page3d.evaluate("windInfo().state.fields")
        total = 0
        for h in (10.0, 80.0, 120.0):
            g = f[wl.profile_field_id(h)]
            area = g["visible_area_px"]
            assert 0 < area < 20000, area
            assert g["n_particles"] == _share_count(area, 3), (h, area, g["n_particles"])
            total += g["n_particles"]
        cap = max(_js_round(f[wl.profile_field_id(h)]["visible_area_px"] / 25.0) for h in (10.0, 80.0, 120.0))
        assert total <= max(20, cap) + 3, f"拉遠時全組加起來不比單一風場的密度上限密：{total} vs {cap}"
    finally:
        _set_cam(page3d, height=1500.0)
        page3d.evaluate("(p) => { clearWindProfile(); setWindProfile(p); }", p)
        page3d.wait_for_function(_ALL_ALIVE, timeout=20000)


def test_3d_legend_is_one_compact_block_not_one_line_per_layer(page3d):
    page3d.wait_for_function("windInfo().state.legend.length > 0", timeout=10000)
    leg = page3d.evaluate("windInfo().state.legend")
    assert leg[0] == "風場分層 10／80／120 m　最大 4.5 m/s"
    assert leg[1] == f"（icon_global 預報，非實測，{TIME} UTC）", "第二行不可以被截斷（模式、非實測、時間都要在）"
    assert leg[2] == "實線＝80 m 層，其餘層淡化（各層數值見剖面柱）", "三層疊在一起：說明哪一層是實線"
    assert leg[0][:len("風場分層 10／80／120 m")] + leg[1] == LEGEND
    assert not [ln for ln in leg if "個模式格點" in ln], "各層自己的 label 不再一層一行"
    assert any(ln.startswith("UAV-1 即時風") or "1 點" in ln for ln in leg), "飛控即時風那一行照舊"
    assert not [ln for ln in leg if ln.endswith("…")], leg


def test_3d_column_entities_heights_downwind_arrows_and_colors(page3d):
    ents = page3d.evaluate(_ENTS)
    assert len(ents) == 5, [e["name"] for e in ents]
    col = next(e for e in ents if "剖面柱" in e["name"])
    assert [round(p[2], 3) for p in col["line"]] == [0.0, 120.0], "垂直線從 0 m 到最高層"
    assert all(abs(p[0] - CENTER[0]) < 1e-9 and abs(p[1] - CENTER[1]) < 1e-9 for p in col["line"])
    assert col["name"] == f"風場剖面柱（icon_global 預報，非實測，{TIME} UTC）"
    head = next(e for e in ents if "標題" in e["name"])
    assert head["text"] == f"風場剖面（icon_global 預報，非實測，{TIME} UTC）" and round(head["pos"][2], 3) == 120.0
    lv = _level_ents(ents)
    assert sorted(lv) == [10, 80, 120]
    lengths = []
    for h, e in lv.items():
        s, d = SPEC[float(h)]
        assert e["arrow"] is True and e["width"] >= 8
        base, tip = e["line"]
        assert abs(base[2] - h) < 1e-3 and abs(tip[2] - h) < 1e-3, "箭頭水平地畫在自己的高度"
        assert _ang(_azimuth(base, tip), (d + 180.0) % 360.0) < 0.5, f"{h} m 箭頭要指向下風處"
        exp_len = min(150.0, max(10.0, 30.0 * math.sqrt(s)))
        assert _haversine(base, tip) == pytest.approx(exp_len, rel=0.01)
        assert wl.profile_arrow_length_m(s) == pytest.approx(exp_len)
        lengths.append(_haversine(base, tip))
        assert e["pointColor"] == e["lineColor"] == COLOR[float(h)], "風速顏色在點與箭頭上"
        assert e["labelColor"] == TEXT, "文字一律淺色（紅字在深底上讀不清楚）"
        assert e["text"] == f"{h} m  {s:.1f} m/s  來自 {d:.0f}°", "數字是來向：寫「來自」，箭頭才是順風方向"
        assert e["name"] == f"風場剖面 {h} m（icon_global 預報，非實測，{TIME} UTC）"
    assert lengths == sorted(lengths) and len(set(round(x, 3) for x in lengths)) == 3
    # 西風（從 270° 來）⇒ 箭頭往東：經度變大、緯度幾乎不動
    b10, t10 = lv[10]["line"]
    assert t10[1] > b10[1] and abs(t10[0] - b10[0]) < 1e-7
    info = page3d.evaluate("windInfo().profile.decor")
    assert info["entities"] == 5 and info["column"]["top_m"] == 120.0
    assert [x["bearing_deg"] for x in info["column"]["levels"]] == [90.0, 70.0, 45.0]
    assert [x["label_dy_px"] for x in info["column"]["levels"]] == [20, 0, -20], "正上方俯視時三行文字不可以疊在一起"
    assert info["column"]["header_dy_px"] == -40
    # 淺色字在深色底上（底是半透明：疊在最亮的白色地表上也要 ≥ 4.5:1）
    bg = info["column"]["label_bg"]
    under = _over((bg[0] * 255, bg[1] * 255, bg[2] * 255, bg[3]), (255.0, 255.0, 255.0))
    assert info["column"]["text_color"] == TEXT and _contrast(_hex_rgb(TEXT), under) >= 4.5


@pytest.mark.parametrize("speed", [8.0, 12.0, 16.0, 24.0])
def test_column_arrow_length_keeps_growing_past_8_mps(speed):
    """舊的線性 15 m/(m/s) 夾 120 m：8、12、16 m/s 全是 120 m。√ 刻度到 25 m/s 才到上限。"""
    assert wl.profile_arrow_length_m(speed) < wl.profile_arrow_length_m(speed + 1.0) <= 150.0
    assert wl.profile_arrow_length_m(0.0) == 0.0 and wl.profile_arrow_length_m(0.1) == 10.0


_PICK = """(fr) => {
  const C = Cesium, sc = viewer.scene, ST = C.SceneTransforms, f = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
  const I = windInfo().profile.decor.column, out = [];
  I.levels.forEach(lv => {
    if (!lv.tip) return;
    fr.forEach(t => {
      const lat = I.lat + (lv.tip[0] - I.lat) * t, lon = I.lon + (lv.tip[1] - I.lon) * t;
      const w = f.call(ST, sc, C.Cartesian3.fromDegrees(lon, lat, lv.h_m));
      if (!w || w.x < 2 || w.y < 2 || w.x > sc.canvas.clientWidth - 2 || w.y > sc.canvas.clientHeight - 2) {
        out.push({ h: lv.h_m, t: t, off: true }); return;
      }
      const picks = sc.drillPick(w, 6);
      out.push({ h: lv.h_m, t: t, x: w.x, y: w.y, n: picks.length,
                 labels: picks.filter(p => p.primitive instanceof C.Label).map(p => (p.id && p.id.name) || '?'),
                 // entity 的 polyline 是一般 Primitive（不是 PolylineCollection 的 Polyline）：用「不是文字、不是點、id 是這一層」認
                 arrow: picks.some(p => !(p.primitive instanceof C.Label) && !(p.primitive instanceof C.PointPrimitive)
                                        && p.id && p.id.polyline && (p.id.name || '').indexOf('風場剖面 ' + lv.h_m + ' m') === 0) });
    });
  });
  return { picks: out, side: I.label_side, extent: I.arrow_extent_px, mask: I.mask };
}"""


def _uniform_profile(dir_from: float) -> dict:
    """三層同一個風向（最難的情形：三支箭頭都伸向同一邊）。"""
    lay = []
    for h in (10.0, 80.0, 120.0):
        u, v = wf.dir_to_uv(dir_from, SPEC[h][0])
        lay.append(dict(_layer(h), u=[[u] * 3 for _ in range(3)], v=[[v] * 3 for _ in range(3)]))
    return wl.check_wind_profile(_payload(layers=lay, column={"lat": CENTER[0], "lon": CENTER[1], "levels": [
        {"h_m": h, "speed_mps": SPEC[h][0], "dir_from_deg": dir_from} for h in (10.0, 80.0, 120.0)]}))


@pytest.mark.parametrize("cam", ["top", "oblique"])
@pytest.mark.parametrize("dir_from", [0.0, 90.0, 180.0, 270.0])
def test_3d_column_labels_never_cover_the_downwind_arrows(page3d, dir_from, cam):
    """文字放在箭頭的另一側：每支箭頭的頭（75–97%）在螢幕上不可以被任何一個文字框蓋住（用 Cesium 自己的 drillPick 量）。
    舊版文字一律往右、80 m 那行剛好在箭頭的起點上 ⇒ 西風（箭頭往右）時箭頭整支被字擋住。"""
    try:
        if cam == "top":
            _set_cam(page3d, height=600.0)
        else:
            _set_cam(page3d, heading=0.0, pitch=-35.0, rng=900.0)
        assert page3d.evaluate("(p) => setWindProfile(p)", _uniform_profile(dir_from)) is True
        _settle(page3d)
        r = page3d.evaluate(_PICK, [0.75, 0.85, 0.92, 0.97])
        on = [x for x in r["picks"] if not x.get("off")]
        assert len(on) >= 9, r
        hidden = [x for x in on if x["labels"]]
        assert not hidden, f"箭頭頭部被文字蓋住（風從 {dir_from}°、{cam}，文字在 {r['side']}）：{hidden}"
        assert all(x["arrow"] for x in on), f"量的點上要真的有箭頭（測試才有意義）：{on}"
        downwind_right = dir_from == 270.0                         # 箭頭往東 ＝ 螢幕右（北朝上）
        downwind_left = dir_from == 90.0
        if downwind_right:
            assert r["side"] == "left", r
        if downwind_left:
            assert r["side"] == "right", r
        assert len(r["mask"]) == 4, "柱頂標題＋三層文字底下都挖空粒子"
    finally:
        _set_cam(page3d, height=1500.0)
        assert page3d.evaluate("(p) => setWindProfile(p)", page3d._p) is True
        _settle(page3d, 300)


def test_3d_particles_are_cut_out_under_the_column_text(page3d):
    """粒子 canvas 疊在 Cesium 之上：文字框底下的軌跡要被挖空（引擎 setMask），旁邊照樣有軌跡（量得到才算數）。"""
    _set_cam(page3d, height=1500.0)
    _settle(page3d, 300)
    page3d.wait_for_function("windInfo().state.frames > 60 && " + _ALL_ALIVE, timeout=20000)
    r = page3d.evaluate("""() => {
      const m = windInfo().state.mask, cv = _windLayer._trailCanvas, ctx = cv.getContext('2d'), d = window.devicePixelRatio || 1;
      // 讀的範圍先裁到 canvas 裡（Chrome 對負的起點讀回來的畫面外像素不一定是透明的）
      const alpha = (x0, y0, w, h) => { const X0 = Math.max(0, Math.ceil(x0 * d)), Y0 = Math.max(0, Math.ceil(y0 * d));
                                         const X1 = Math.min(cv.width, Math.floor((x0 + w) * d)), Y1 = Math.min(cv.height, Math.floor((y0 + h) * d));
                                         if (X1 <= X0 || Y1 <= Y0) return 0;
                                         const im = ctx.getImageData(X0, Y0, X1 - X0, Y1 - Y0).data;
                                         let n = 0; for (let i = 3; i < im.length; i += 4) if (im[i] > 0) n++; return n; };
      const inside = m.map(r => alpha(r[0] + 1, r[1] + 1, r[2] - 2, r[3] - 2));
      const allX0 = Math.min(...m.map(r => r[0])), allX1 = Math.max(...m.map(r => r[0] + r[2]));
      const allY0 = Math.min(...m.map(r => r[1])), allY1 = Math.max(...m.map(r => r[1] + r[3]));
      const around = alpha(allX0 - 120, allY1 + 30, (allX1 - allX0) + 240, 120);
      return { mask: m, inside: inside, around: around };
    }""")
    assert len(r["mask"]) == 4 and all(w > 40 and h > 10 for _x, _y, w, h in r["mask"]), r["mask"]
    assert r["inside"] == [0, 0, 0, 0], f"文字框底下還有粒子軌跡：{r}"
    assert r["around"] > 0, "文字框旁邊要有軌跡（不然上面那條驗不到東西）"


def test_3d_switching_the_solid_layer_keeps_particles_and_the_column(page3d):
    """換 2D 顯示高度 ⇒ 3D 只換哪一層實線：粒子陣列、軌跡、剖面柱 entity 都沿用（不重生、不重畫）。"""
    p = page3d._p
    page3d.wait_for_function(_ALL_ALIVE, timeout=20000)
    before = page3d.evaluate("""() => { window.__pts = {}; ['prof:10','prof:80','prof:120'].forEach(k => {
        window.__pts[k] = _windLayer.fields.get(k).pt; });
      window.__ents = _windColEnts.slice();
      return ['prof:10','prof:80','prof:120'].map(k => _windLayer.fields.get(k).faint); }""")
    assert before == [True, False, True]
    rr0 = page3d.evaluate("window.__rr")
    assert page3d.evaluate("(v) => setWindProfileView(v)", wl.resolve_profile_view(p, height_2d_m=10.0)) is True
    after = page3d.evaluate("""() => ({
      same_pt: ['prof:10','prof:80','prof:120'].map(k => _windLayer.fields.get(k).pt === window.__pts[k]),
      same_ents: _windColEnts.length === window.__ents.length && _windColEnts.every((e, i) => e === window.__ents[i]),
      faint: ['prof:10','prof:80','prof:120'].map(k => _windLayer.fields.get(k).faint),
      emph: windInfo().profile.emphasis_3d_m })""")
    assert after == {"same_pt": [True, True, True], "same_ents": True, "faint": [False, True, True], "emph": 10.0}
    page3d.wait_for_function("windInfo().state.legend.indexOf('實線＝10 m 層，其餘層淡化（各層數值見剖面柱）') >= 0",
                             timeout=10000)
    # 要求值落到同一層（10 ⇒ 10）：什麼都不動
    assert page3d.evaluate("(v) => setWindProfileView(v)", wl.resolve_profile_view(p, height_2d_m=10.0)) is True
    assert page3d.evaluate("['prof:10','prof:80','prof:120'].every(k => _windLayer.fields.get(k).pt === window.__pts[k])")
    assert page3d.evaluate("(v) => setWindProfileView(v)", wl.resolve_profile_view(p, height_2d_m=80.0)) is True
    assert page3d.evaluate("window.__rr") >= rr0


def test_3d_column_alone_still_says_forecast_model_and_time(page3d):
    """3D 一層都不顯示（layers_3d=[]）：沒有引擎風場就沒有圖例區塊 ⇒ 剖面柱自己的柱頂標題要寫模式、預報非實測、時間。"""
    p = page3d._p
    try:
        assert page3d.evaluate("(v) => setWindProfileView(v)", wl.resolve_profile_view(p, layers_3d=[])) is True
        # 圖例在引擎下一幀才重畫：等分層那一塊消失（前提：這時畫面上只剩剖面柱自己的字）
        page3d.wait_for_function("windInfo().state.legend.every(l => l.indexOf('非實測') < 0)", timeout=10000)
        info = page3d.evaluate("windInfo()")
        assert info["state"]["fields"] == {} and info["profile"]["drawn"] == []
        texts = [e.get("text") or "" for e in page3d.evaluate(_ENTS)]
        assert f"風場剖面（icon_global 預報，非實測，{TIME} UTC）" in texts, texts
    finally:
        assert page3d.evaluate("(p) => setWindProfile(p)", p) is True


def test_3d_view_subset_rebalances_particles_and_limit_change_recolors(page3d):
    p = page3d._p
    rr0 = page3d.evaluate("window.__rr")
    assert page3d.evaluate("(v) => setWindProfileView(v)", wl.resolve_profile_view(p, layers_3d=[10, 120])) is True
    page3d.wait_for_function("windInfo().state.legend.length > 2 && windInfo().state.legend[2].indexOf('只顯示') >= 0",
                             timeout=10000)
    info = page3d.evaluate("windInfo()")
    assert set(info["state"]["fields"]) == {"prof:10", "prof:120"}
    assert {k: v["n_requested"] for k, v in info["state"]["fields"].items()} == {"prof:10": 1200, "prof:120": 1200}
    assert info["state"]["legend"][:3] == ["風場分層 10／80／120 m　最大 4.5 m/s", f"（icon_global 預報，非實測，{TIME} UTC）",
                                           "3D 只顯示 10／120 m（其餘層隱藏）"]
    assert info["profile"]["decor"]["entities"] == 5, "剖面柱照樣畫三層＋標題（它是剖面，不跟著 3D 子集）"
    assert page3d.evaluate("window.__rr") > rr0
    assert page3d.evaluate("(v) => setWindProfileView(v)", wl.resolve_profile_view(p, layers_3d=None)) is True
    assert len(page3d.evaluate("windInfo().state.fields")) == 3
    # 粒子總數很小 ⇒ 每層至少 80
    small = wl.check_wind_profile(_payload(particles_total=100))
    assert page3d.evaluate("(p) => setWindProfile(p)", small) is True
    assert {v["n_requested"] for v in page3d.evaluate("windInfo().state.fields").values()} == {80}
    # 上色門檻改成 5 ⇒ 4.5 m/s 從紅變琥珀（剖面柱跟引擎同一個上限）
    rr0 = page3d.evaluate("window.__rr")
    page3d.evaluate("setWindOptions({limitMps: 5.0})")
    top = _level_ents(page3d.evaluate(_ENTS))[120]
    assert top["pointColor"] == AMBER and page3d.evaluate("window.__rr") > rr0
    page3d.evaluate("setWindOptions({limitMps: 3.0})")
    assert page3d.evaluate("(p) => setWindProfile(p)", p) is True


def test_3d_clear_restores_the_single_field_removes_the_column_and_keeps_the_arrows(page3d):
    rr0 = page3d.evaluate("window.__rr")
    assert page3d.evaluate("clearWindProfile()") is True
    info = page3d.evaluate("windInfo()")
    assert info["profile"] is None and set(info["state"]["fields"]) == {"default"}
    assert info["state"]["label"] == SINGLE.label and info["state"]["arrows"] == 1
    assert page3d.evaluate(_ENTS) == [] and page3d.evaluate("window.__rr") > rr0
    assert page3d.evaluate("(p) => setWindProfile(p)", page3d._p) is True        # 給下一個人留同樣的狀態
    assert not page3d._errs and not _wind_noise(page3d), (page3d._errs, _wind_noise(page3d))


# 追機（main.html setChaseCamera 的做法）：每幀 lookAtTransform(機體座標系, 後方 80 m、上方 14 m)，飛機以 18 m/s
# 往東北飛、機頭慢慢轉；每幀接著跑引擎一幀（同一次 evaluate 裡，中間不插引擎自己的 rAF）。
# 回傳每幀的軌跡畫法、三層各畫了幾段、活粒子數；最後幾幀量軌跡層 alpha（整張 getImageData 很慢，只量最後 3 幀）。
_CHASE = """([lat, lon, n, follow]) => {
  const C = Cesium, cam = viewer.camera, L_ = _windLayer, ids = ['prof:10', 'prof:80', 'prof:120'], out = [];
  L_.setOptions({trailsFollowView: follow});
  const alpha = () => { const cv = L_._trailCanvas, d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data;
                        let s = 0; for (let i = 3; i < d.length; i += 4) s += d[i]; return s; };
  for (let i = 0; i < n; i++) {
    const s = 0.3 * i, hdg = C.Math.toRadians(45 + 0.2 * i);
    const tgt = C.Cartesian3.fromDegrees(lon + s * Math.sin(hdg) / (111320 * Math.cos(lat * Math.PI / 180)),
                                         lat + s * Math.cos(hdg) / 111320, 100);
    cam.lookAtTransform(C.Transforms.headingPitchRollToFixedFrame(tgt, new C.HeadingPitchRoll(hdg, 0, 0)),
                        new C.Cartesian3(0, -80, 14));
    L_._step(1);
    const st = L_.state();
    out.push({mode: st.trail.mode, segs: ids.map(k => st.fields[k] ? st.fields[k].trail_segments : -1),
              alive: ids.map(k => st.fields[k] ? st.fields[k].n_alive : -1), a: i >= n - 3 ? alpha() : null});
  }
  cam.lookAtTransform(C.Matrix4.IDENTITY);
  return out;
}"""


def test_3d_chase_camera_keeps_flow_trails_on_all_three_layers(page3d):
    """2026-10-21：追機時鏡頭每幀都動，引擎原本每幀清掉軌跡（只剩點）。3D 頁預設 trailsFollowView：
    每幀用粒子的位置歷史重畫 ⇒ 三層都有連續的線段；同一段追機把選項關掉（舊行為）就每幀全空。"""
    page3d.wait_for_function(_ALL_ALIVE, timeout=20000)
    assert page3d.evaluate("windInfo().options.trailsFollowView") is True, "3D 膠水預設要打開"
    try:
        off = page3d.evaluate(_CHASE, [CENTER[0], CENTER[1], 30, False])
        on = page3d.evaluate(_CHASE, [CENTER[0], CENTER[1], 40, True])
    finally:
        page3d.evaluate("_windLayer.setOptions({trailsFollowView: windInfo().options.trailsFollowView})")
        _set_cam(page3d, height=1500.0)
    assert all(f["mode"] == "cleared" for f in off) and [f["a"] for f in off[-3:]] == [0, 0, 0], \
        "前提：舊行為在追機下每幀全清（這就是看不到流動的原因）"
    assert all(f["mode"] == "follow" for f in on), [f["mode"] for f in on]
    for f in on[-10:]:                                          # 開始追機 ~30 幀後：歷史已累積
        assert min(f["alive"]) > 20, f
        assert all(seg > 3 * alive for seg, alive in zip(f["segs"], f["alive"])), \
            f"每一層（10／80／120 m）的粒子都要有多段拖尾：{f}"
    assert min(f["a"] for f in on[-3:]) > 255 * 3000, [f["a"] for f in on[-3:]]
    assert not page3d._errs and not _wind_noise(page3d), (page3d._errs, _wind_noise(page3d))


# ── 整份 main.html：真的 _cesium_template_values（不讀設定檔、不寫 data/cache）──
def _no_settings():
    raise RuntimeError("測試不讀 config/settings.json")


@needs_qt
def test_the_whole_main_html_replays_the_profile_and_the_3d_subset_after_a_reload(browser, tmp_path, monkeypatch):
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    import config.settings as cs
    import ui.widgets.glb_merge as gm
    monkeypatch.setattr(cs, "get_settings", _no_settings)
    monkeypatch.setattr(CesiumMapWidget, "_resolve_token", lambda self: "")
    monkeypatch.setattr(gm, "optimized_model_path", lambda p, cache: p)
    monkeypatch.delenv(CesiumMapWidget.RENDER_DT_ENV, raising=False)
    w = _bare3d()
    CesiumMapWidget.set_wind_field(w, SINGLE)
    CesiumMapWidget.set_wind_arrows(w, [ARROW])
    CesiumMapWidget.set_wind_profile(w, _payload(label="Open-Meteo __INIT_LAT__ 預報（非實測）"))
    CesiumMapWidget.set_wind_profile_view(w, layers_3d=[80.0, 120.0])
    html = w._fill_cesium_template(MAIN_HTML.read_text(encoding="utf-8"))
    f = tmp_path / "main.html"
    f.write_text(html, encoding="utf-8")
    pg = _open(browser, f, "typeof viewer !== 'undefined' && typeof windInfo === 'function' && windInfo().profile",
               size=(640, 480))
    try:
        info = pg.evaluate("windInfo()")
        ents = pg.evaluate(_ENTS)
        rr = pg.evaluate("typeof _requestRender === 'function'")
    finally:
        pg.close()
    assert not pg._errs, pg._errs
    assert info["profile"]["drawn"] == [80.0, 120.0] and set(info["state"]["fields"]) == {"prof:80", "prof:120"}
    assert info["profile"]["label"] == "Open-Meteo __INIT_LAT__ 預報（非實測）", "佔位字樣的字串不可以被範本置換改掉"
    assert info["field"] is True and info["arrows"] == 1 and info["options"]["limitMps"] == 3.0
    assert info["options"]["trailsFollowView"] is True, "整份 main.html：追機時軌跡跟著場景（引擎選項）"
    assert info["state"]["options"]["trailsFollowView"] is True, "選項要真的送到引擎"
    assert len(ents) == 5 and rr is True
    assert not _wind_noise(pg), _wind_noise(pg)
