"""航測成果圖層：正射瓦片（TMS／XYZ）＋照片位姿（BillboardCollection）—— 值層級的鎖。

背景（docs/評估_2026-10-14_群飛航測高精地圖.md §2/§6/§9）：
  * 正射：ODM ``--tiles`` 呼叫自帶 gdal2tiles.py（預設 mercator、沒有 ``--xyz``）⇒ **TMS**
    （y 由南往北）＋ tilemapresource.xml。XYZ（y 由北往南）另走 UrlTemplateImageryProvider。
  * 位姿：每點一個 Entity 在 10 萬點只剩 13 fps ⇒ BillboardCollection＋共用一張箭頭圖，
    rotation = −航向（Cesium billboard 的 rotation 在螢幕上逆時針為正，航向自真北順時針）。

三層：
  A. 純 Python（不開瀏覽器）：路徑 → file:// URL、瓦片資料夾推範圍、位姿列轉換、
     兩個 widget 發出去的 JS 呼叫／算出來的 folium HTML。
  B. 假 Cesium（只記錄呼叫）在無頭 Chrome 裡跑 main.html 的 SURVEY_LAYERS 段：
     billboard 數量、rotation、alignedAxis、上色、正射 provider 種類／URL、移除、CSV 解析、
     file:// XHR 讀檔、拾取。
  C. 真的本機 Cesium 1.115（assets/cesium，gitignored；沒有就 skip）：讀**畫出來的像素**驗
     航向 0/90/225° 的箭頭尖端方位，以及 TMS 瓦片從含空白＋中文的路徑真的畫得出來。

B、C 要開瀏覽器：用 safe runner 時要設 SAFE_ALLOW_BROWSER=1，否則這兩層會 skip。
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import re

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = pathlib.Path(__file__).resolve().parents[1]
# 突變測試用：可指向 main.html 的突變副本（不改 repo 檔）
MAIN_HTML = pathlib.Path(os.environ.get("SURVEY_LAYERS_MAIN_HTML")
                         or REPO / "ui" / "widgets" / "cesium_templates" / "main.html")
CESIUM_DIR = REPO / "assets" / "cesium" / "Build" / "Cesium"

try:
    from ui.widgets import cesium_map_widget as cmw
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.widgets.map_widget import MapWidget
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")


def _section() -> str:
    src = MAIN_HTML.read_text(encoding="utf-8")
    a = src.index("SURVEY_LAYERS_BEGIN")
    a = src.rindex("\n", 0, a) + 1
    b = src.index("// SURVEY_LAYERS_END")
    return src[a:b]


# ═══════════════════════════════════════════════════════════════════════
#  共用：合成瓦片資料夾
# ═══════════════════════════════════════════════════════════════════════
def _tile_x(lon, z):
    return int((lon + 180.0) / 360.0 * 2 ** z)


def _tile_y_xyz(lat, z):
    r = math.radians(lat)
    return int((1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * 2 ** z)


def _lon_of(x, z):
    return x / 2 ** z * 360.0 - 180.0


def _lat_of(y_xyz, z):
    n = math.pi - 2.0 * math.pi * y_xyz / 2 ** z
    return math.degrees(math.atan(0.5 * (math.exp(n) - math.exp(-n))))


LAT0, LON0 = 25.07, 121.21


def _make_tiles(root: pathlib.Path, *, tms: bool, xml: bool, levels=(10, 11, 12, 13, 14, 15, 16),
                cols=2, rows=3, ext="png", payload=None):
    """在 (LAT0, LON0) 西北角起、每層 cols×rows 張瓦片；回傳 {z: (x0, x1, y0_xyz, y1_xyz)}。"""
    spans = {}
    for z in levels:
        x0, y0 = _tile_x(LON0, z), _tile_y_xyz(LAT0, z)
        spans[z] = (x0, x0 + cols - 1, y0, y0 + rows - 1)
        for x in range(x0, x0 + cols):
            for y in range(y0, y0 + rows):
                yy = (2 ** z - 1 - y) if tms else y
                f = root / str(z) / str(x) / f"{yy}.{ext}"
                f.parent.mkdir(parents=True, exist_ok=True)
                f.write_bytes(payload if payload is not None else b"x")
    # gdal2tiles 會放的雜物：非數字資料夾與 html，必須被略過
    (root / "leaflet.html").write_text("<html></html>", encoding="utf-8")
    (root / "notalevel").mkdir(exist_ok=True)
    if xml:
        (root / "tilemapresource.xml").write_text("<TileMap/>", encoding="utf-8")
    return spans


def _expected_bounds(spans, z):
    x0, x1, y0, y1 = spans[z]
    return (_lon_of(x0, z), _lat_of(y1 + 1, z), _lon_of(x1 + 1, z), _lat_of(y0, z))


# ═══════════════════════════════════════════════════════════════════════
#  A1. 路徑 → file:// URL
# ═══════════════════════════════════════════════════════════════════════
def test_windows_path_with_space_and_cjk_becomes_a_percent_encoded_file_url():
    assert cmw.local_path_to_file_url(r"C:\a b\地圖\tiles") == \
        "file:///C:/a%20b/%E5%9C%B0%E5%9C%96/tiles"


def test_hash_percent_and_braces_are_encoded_so_they_cannot_act_as_anchor_escape_or_template():
    assert cmw.local_path_to_file_url(r"D:\x#y\50%\{z}") == "file:///D:/x%23y/50%25/%7Bz%7D"


def test_urls_pass_through_and_relative_paths_become_absolute(tmp_path, monkeypatch):
    for u in ("https://h/tiles", "http://h/t/{z}/{x}/{y}.png", "FILE:///C:/x"):
        assert cmw.local_path_to_file_url(u) == u
    monkeypatch.chdir(tmp_path)
    assert cmw.local_path_to_file_url("sub dir") == (tmp_path / "sub dir").as_uri()
    assert cmw.local_path_to_file_url("sub dir").startswith("file:///")


# ═══════════════════════════════════════════════════════════════════════
#  A2. 瓦片資料夾：層級、副檔名、範圍
# ═══════════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("tms", [True, False], ids=["tms", "xyz"])
def test_tile_dir_levels_ext_and_bounds_from_the_level_four_above_the_finest(tmp_path, tms):
    spans = _make_tiles(tmp_path, tms=tms, xml=tms)
    info = cmw.inspect_tile_dir(tmp_path, "tms" if tms else "xyz")
    assert (info["min_level"], info["max_level"], info["ext"]) == (10, 16, "png")
    assert info["has_tilemapresource"] is tms
    exp = _expected_bounds(spans, 12)                 # max(10, 16-4) = 12
    assert info["bounds"] == pytest.approx(exp, abs=1e-9)
    w, s, e, n = info["bounds"]
    assert w <= LON0 < e and s < LAT0 <= n, "範圍要包住瓦片起點"


def test_tms_and_xyz_readings_of_the_same_folder_are_mirrored_not_equal(tmp_path):
    """同一個 TMS 資料夾當成 XYZ 讀 ⇒ 範圍跑到南半球（這就是指定錯 scheme 的後果）。"""
    _make_tiles(tmp_path, tms=True, xml=True)
    as_tms = cmw.inspect_tile_dir(tmp_path, "tms")["bounds"]
    as_xyz = cmw.inspect_tile_dir(tmp_path, "xyz")["bounds"]
    assert as_tms[1] > 0 and as_xyz[3] < 0
    assert as_xyz[1] == pytest.approx(-as_tms[3], abs=1e-9)


def test_a_single_level_folder_uses_that_level_and_jpg_is_detected(tmp_path):
    spans = _make_tiles(tmp_path, tms=False, xml=False, levels=(17,), ext="jpg")
    info = cmw.inspect_tile_dir(tmp_path, "xyz")
    assert (info["min_level"], info["max_level"], info["ext"]) == (17, 17, "jpg")
    assert info["bounds"] == pytest.approx(_expected_bounds(spans, 17), abs=1e-9)


def test_tile_dir_errors_are_loud(tmp_path):
    with pytest.raises(FileNotFoundError):
        cmw.inspect_tile_dir(tmp_path / "nope", "tms")
    (tmp_path / "empty").mkdir()
    with pytest.raises(ValueError, match="不是瓦片資料夾"):
        cmw.inspect_tile_dir(tmp_path / "empty", "tms")
    (tmp_path / "lvl" / "5" / "3").mkdir(parents=True)
    with pytest.raises(ValueError, match="沒有瓦片檔"):
        cmw.inspect_tile_dir(tmp_path / "lvl", "tms")
    with pytest.raises(ValueError, match="scheme"):
        cmw.inspect_tile_dir(tmp_path, "wmts")


@pytest.mark.parametrize("bad", [(121.3, 25.0, 121.2, 25.1), (121.2, 25.1, 121.3, 25.0),
                                 (-181, 0, 0, 1), (0, -91, 1, 0), (1, 2, 3), "abcd"])
def test_bad_bounds_are_rejected_not_fixed(bad):
    with pytest.raises(ValueError):
        cmw.check_bounds(bad)


# ═══════════════════════════════════════════════════════════════════════
#  A3. 位姿列（photos.csv F2）→ 精簡列
# ═══════════════════════════════════════════════════════════════════════
def _row(i, lat=LAT0, lon=LON0, h="150.5", yaw="90", src="ppk"):
    return {"image_id": f"IMG_{i:04d}", "lat_deg": lat, "lon_deg": lon, "h_ell_m": h,
            "yaw_deg": yaw, "source": src, "cog_deg": "999"}


def test_compact_rows_keep_values_and_count_every_gap():
    rows = [_row(1), _row(2, yaw=""), _row(3, h=None), _row(4, lat=""), _row(5, lon="abc"),
            _row(6, lat="91"), _row(7, src="rtk"), _row(8, yaw="nan"), _row(9, yaw=225.25, src="sfm")]
    out, st = cmw.photo_pose_rows_to_compact(rows)
    assert out[0] == [LAT0, LON0, 150.5, 90.0, "ppk", "IMG_0001"]
    assert out[1][3] is None and out[2][2] is None
    assert [r[5] for r in out] == ["IMG_0001", "IMG_0002", "IMG_0003", "IMG_0007", "IMG_0008", "IMG_0009"]
    assert out[-1][3] == 225.25 and out[-1][4] == "sfm"
    assert st["rows"] == 9 and st["drawn"] == 6 and st["skipped"] == 3
    assert st["skipped_ids"] == ["IMG_0004", "IMG_0005", "IMG_0006"]
    assert st["no_heading"] == 2 and st["no_height"] == 1
    assert st["unknown_source"] == {"rtk": 1}


def test_yaw_is_taken_from_yaw_deg_not_from_course_over_ground():
    out, _ = cmw.photo_pose_rows_to_compact([_row(1, yaw="12.5")])
    assert out[0][3] == 12.5                          # cog_deg=999 不可被拿去用


class _Warnings:
    def __init__(self):
        self.msgs = []

    def warning(self, m, *a, **k):
        self.msgs.append(m)

    def info(self, *a, **k):
        pass


def test_compact_rows_log_the_gaps_loudly(monkeypatch):
    _, st = cmw.photo_pose_rows_to_compact([_row(1, lat=None), _row(2, yaw=None), _row(3, src="x"),
                                            _row(4, h="")])
    fake = _Warnings()
    monkeypatch.setattr(cmw, "logger", fake)
    cmw.log_photo_pose_stats("L1", st, "3D")
    assert len(fake.msgs) == 4
    joined = "\n".join(fake.msgs)
    assert "1 列經緯度" in joined and "IMG_0001" in joined
    assert "1 點沒有 yaw_deg" in joined and "{'x': 1}" in joined and "1 點沒有 h_ell_m" in joined
    fake.msgs.clear()
    cmw.log_photo_pose_stats("L1", cmw.photo_pose_rows_to_compact([_row(1)])[1], "3D")
    assert fake.msgs == [], "沒有缺漏就不該警告"


# ═══════════════════════════════════════════════════════════════════════
#  A4. 3D widget 發出去的 JS（裸實例：不建 QWebEngineView）
# ═══════════════════════════════════════════════════════════════════════
class _NoDelCesium(CesiumMapWidget if _QT_OK else object):
    def __del__(self):
        pass


def _bare3d():
    w = _NoDelCesium.__new__(_NoDelCesium)
    w._sent = []
    w._js = w._sent.append
    w._ortho_layers = {}
    w._photo_pose_layers = {}
    return w


def _call(js: str, fn: str):
    m = re.fullmatch(re.escape(fn) + r"\((.*)\)", js, flags=re.S)
    assert m, f"不是 {fn}(...)：{js[:120]}"
    return json.loads("[" + m.group(1) + "]")


def test_add_ortho_layer_tms_with_xml_lets_cesium_read_the_bounds(tmp_path):
    root = tmp_path / "a b" / "地圖" / "orthophoto_tiles"
    _make_tiles(root, tms=True, xml=True)
    w = _bare3d()
    info = CesiumMapWidget.add_ortho_layer(w, "ortho1", root, alpha=0.7)
    lid, url, opts = _call(w._sent[-1], "addOrthoLayer")
    assert lid == "ortho1"
    assert url == root.as_uri() and "%E5%9C%B0%E5%9C%96" in url and "%20" in url
    assert opts == {"scheme": "tms", "alpha": 0.7, "rectangle": None, "minimumLevel": 10,
                    "maximumLevel": 16, "fileExtension": "png"}
    assert info["bounds_source"] == "tilemapresource.xml" and info["bounds"] is None
    assert w._ortho_layers["ortho1"]["url"] == url


def test_add_ortho_layer_without_xml_or_as_xyz_sends_the_bounds_from_the_tiles(tmp_path):
    spans = _make_tiles(tmp_path / "t", tms=True, xml=False)
    w = _bare3d()
    info = CesiumMapWidget.add_ortho_layer(w, "o", tmp_path / "t", scheme="tms")
    _, _, opts = _call(w._sent[-1], "addOrthoLayer")
    assert opts["rectangle"] == pytest.approx(list(_expected_bounds(spans, 12)), abs=1e-9)
    assert info["bounds_source"] == "tiles_on_disk"

    spans = _make_tiles(tmp_path / "x", tms=False, xml=False)
    CesiumMapWidget.add_ortho_layer(w, "x", tmp_path / "x", scheme="xyz")
    _, _, opts = _call(w._sent[-1], "addOrthoLayer")
    assert opts["scheme"] == "xyz"
    assert opts["rectangle"] == pytest.approx(list(_expected_bounds(spans, 12)), abs=1e-9)


def test_user_bounds_win_and_urls_are_not_inspected(tmp_path):
    _make_tiles(tmp_path, tms=True, xml=True)
    w = _bare3d()
    info = CesiumMapWidget.add_ortho_layer(w, "o", tmp_path, bounds=(121.2, 25.0, 121.3, 25.1))
    assert _call(w._sent[-1], "addOrthoLayer")[2]["rectangle"] == [121.2, 25.0, 121.3, 25.1]
    assert info["bounds_source"] == "user"
    info = CesiumMapWidget.add_ortho_layer(w, "r", "http://h/tiles/{z}/{x}/{y}.png", scheme="xyz")
    lid, url, opts = _call(w._sent[-1], "addOrthoLayer")
    assert url == "http://h/tiles/{z}/{x}/{y}.png" and opts["minimumLevel"] is None
    assert info["bounds_source"] == "none"


def test_add_ortho_layer_rejects_bad_input_without_sending_anything(tmp_path):
    _make_tiles(tmp_path, tms=True, xml=True)
    w = _bare3d()
    for kw in ({"scheme": "wmts"}, {"alpha": 1.5}, {"alpha": -0.1}, {"bounds": (1, 2, 0, 3)}):
        with pytest.raises(ValueError):
            CesiumMapWidget.add_ortho_layer(w, "o", tmp_path, **kw)
    with pytest.raises(FileNotFoundError):
        CesiumMapWidget.add_ortho_layer(w, "o", tmp_path / "missing")
    with pytest.raises(ValueError):
        CesiumMapWidget.add_ortho_layer(w, "", tmp_path)
    assert w._sent == [] and w._ortho_layers == {}


def test_remove_ortho_layer_reports_whether_it_existed(tmp_path):
    _make_tiles(tmp_path, tms=True, xml=True)
    w = _bare3d()
    CesiumMapWidget.add_ortho_layer(w, "o", tmp_path)
    assert CesiumMapWidget.remove_ortho_layer(w, "o") is True
    assert w._sent[-1] == 'removeOrthoLayer("o")' and "o" not in w._ortho_layers
    assert CesiumMapWidget.remove_ortho_layer(w, "o") is False


def test_set_photo_poses_sends_compact_rows_and_returns_the_counts():
    w = _bare3d()
    rows = [_row(1), _row(2, yaw=""), _row(3, lat=None)]
    st = CesiumMapWidget.set_photo_poses(w, "fc", rows)
    lid, data, opts = _call(w._sent[-1], "setPhotoPoses")
    assert lid == "fc" and opts == {"color": None}
    assert data == {"rows": [[LAT0, LON0, 150.5, 90.0, "ppk", "IMG_0001"],
                             [LAT0, LON0, 150.5, None, "ppk", "IMG_0002"]]}
    assert (st["drawn"], st["skipped"], st["no_heading"]) == (2, 1, 1)
    assert w._photo_pose_layers["fc"]["drawn"] == 2


def test_set_photo_poses_caps_inline_json_and_validates_colour():
    w = _bare3d()
    cap = cmw.PHOTO_POSES_INLINE_MAX
    assert cap == 5000
    CesiumMapWidget.set_photo_poses(w, "ok", [_row(i) for i in range(cap)])
    assert len(_call(w._sent[-1], "setPhotoPoses")[1]["rows"]) == cap
    n = len(w._sent)
    with pytest.raises(ValueError, match="load_photo_poses_file"):
        CesiumMapWidget.set_photo_poses(w, "big", [_row(i) for i in range(cap + 1)])
    for bad in ("red", "#12345", "#GGGGGG", 0xFF0000):
        with pytest.raises(ValueError):
            CesiumMapWidget.set_photo_poses(w, "c", [_row(1)], color=bad)
    assert len(w._sent) == n
    CesiumMapWidget.set_photo_poses(w, "c", [_row(1)], color="#FF00aa")
    assert _call(w._sent[-1], "setPhotoPoses")[2] == {"color": "#FF00aa"}


def test_load_photo_poses_file_checks_the_header_and_sends_an_encoded_url(tmp_path):
    w = _bare3d()
    d = tmp_path / "成果 1"
    d.mkdir()
    good = d / "photos.csv"
    good.write_text("\ufeffimage_id,lat_deg,lon_deg,yaw_deg\nA,25,121,10\n", encoding="utf-8")
    info = CesiumMapWidget.load_photo_poses_file(w, "big", good)
    lid, url, opts = _call(w._sent[-1], "loadPhotoPosesUrl")
    assert lid == "big" and url == good.as_uri() and "%20" in url and opts == {"color": None}
    assert info["format"] == "csv" and w._photo_pose_layers["big"]["url"] == url
    bad = d / "bad.csv"
    bad.write_text("image_id,lat,lon\n", encoding="utf-8")
    with pytest.raises(ValueError, match="lat_deg"):
        CesiumMapWidget.load_photo_poses_file(w, "x", bad)
    with pytest.raises(FileNotFoundError):
        CesiumMapWidget.load_photo_poses_file(w, "x", d / "none.csv")
    (d / "p.txt").write_text("x", encoding="utf-8")
    with pytest.raises(ValueError, match=".csv"):
        CesiumMapWidget.load_photo_poses_file(w, "x", d / "p.txt")
    (d / "p.json").write_text('{"rows": []}', encoding="utf-8")
    CesiumMapWidget.load_photo_poses_file(w, "j", d / "p.json")
    assert _call(w._sent[-1], "loadPhotoPosesUrl")[1].endswith("/p.json")


def test_clear_survey_layers_empties_python_state_and_calls_the_page(tmp_path):
    _make_tiles(tmp_path, tms=True, xml=True)
    w = _bare3d()
    CesiumMapWidget.add_ortho_layer(w, "o", tmp_path)
    CesiumMapWidget.set_photo_poses(w, "p", [_row(1)])
    assert CesiumMapWidget.clear_photo_poses(w, "p") is True and w._sent[-1] == 'clearPhotoPoses("p")'
    CesiumMapWidget.clear_survey_layers(w)
    assert w._sent[-1] == "clearSurveyLayers()" and w._ortho_layers == {} == w._photo_pose_layers


def test_python_and_page_source_colours_are_the_same_table():
    m = re.search(r"var _PHOTO_POSE_COLORS = (\{[^}]*\});", _section())
    js = dict(re.findall(r"(\w+):\s*'(#[0-9A-Fa-f]{6})'", m.group(1)))
    assert js == cmw.PHOTO_POSE_SOURCE_COLORS
    other = re.search(r"var _PHOTO_POSE_OTHER_COLOR = '(#[0-9A-Fa-f]{6})'", _section()).group(1)
    assert other == cmw.PHOTO_POSE_OTHER_COLOR


def test_clear_all_still_does_not_touch_survey_layers():
    """刻意的決定：成果圖層有自己的移除方法；clear_all（清規劃疊圖）不碰它們。"""
    w = _bare3d()
    for k, v in dict(corners=[], paths=[], path_colors=[], _path_tooltips=[], transit_paths=[],
                     _circle_center=None, _circle_radius_m=0.0, _home_point=None, _nfz_zones=[],
                     _swarm_data=None, _fw_result=None, _geofence=None).items():
        setattr(w, k, v)
    w._ortho_layers = {"o": {}}
    CesiumMapWidget.clear_all(w)
    js = "\n".join(w._sent)
    assert "clearSurveyLayers" not in js and "removeOrthoLayer" not in js
    assert w._ortho_layers == {"o": {}}


# ═══════════════════════════════════════════════════════════════════════
#  A5. 2D folium
# ═══════════════════════════════════════════════════════════════════════
def _bare2d():
    w = MapWidget.__new__(MapWidget)
    for k, v in dict(corners=[], markers=[], paths=[], path_colors=[], _path_tooltips=[],
                     transit_paths=[], _circle_center=None, _circle_radius_m=0.0, _home_point=None,
                     _nfz_zones=[], _geofence=None, _swarm_data=None, _view_state=None,
                     _ortho_tiles=[], _photo_pose_layers={}).items():
        setattr(w, k, v)
    w._renders = []
    w._render_map = lambda: w._renders.append(1)
    return w


def _render2d(w) -> str:
    return w._build_map().get_root().render()


def _tile_layer_calls(html: str):
    """回傳每個 L.tileLayer(url, {options}) 的 (url, options dict)。"""
    out = []
    for m in re.finditer(r'L\.tileLayer\(\s*"([^"]*)",\s*(\{.*?\})\s*\)', html, flags=re.S):
        out.append((m.group(1), json.loads(re.sub(r",(\s*[\]}])", r"\1", m.group(2)))))   # folium 會留尾逗號
    return out


def _js_str(s: str) -> str:
    """folium 把字串寫成 JSON（非 ASCII 與 <> 都變 \\uXXXX）；用同樣的寫法去 HTML 裡找。"""
    return json.dumps(s)[1:-1].replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")


def _features(html: str):
    m = re.search(r'_add\((\{"features": \[.*?\], "type": "FeatureCollection"\})\);', html, flags=re.S)
    return json.loads(m.group(1))["features"]


def test_2d_ortho_is_a_tms_tile_layer_with_the_file_url_bounds_and_native_zooms(tmp_path):
    root = tmp_path / "a b" / "地圖" / "tiles"
    spans = _make_tiles(root, tms=True, xml=True)
    w = _bare2d()
    info = MapWidget.add_ortho_tiles(w, root)
    assert w._renders == [1], "加正射要重繪一次（folium 是整頁重建）"
    calls = [c for c in _tile_layer_calls(_render2d(w)) if c[0].startswith("file:///")]
    assert len(calls) == 1
    url, o = calls[0]
    assert url == root.as_uri() + "/{z}/{x}/{y}.png" == info["template"]
    assert o["tms"] is True
    assert (o["minNativeZoom"], o["maxNativeZoom"], o["maxZoom"]) == (10, 16, 22)
    w_, s_, e_, n_ = _expected_bounds(spans, 12)
    assert o["bounds"][0] == pytest.approx([s_, w_], abs=1e-9)      # Leaflet：[[南, 西], [北, 東]]
    assert o["bounds"][1] == pytest.approx([n_, e_], abs=1e-9)
    assert "正射影像" in o["attribution"]


def test_2d_xyz_tiles_are_not_flipped_and_re_adding_the_same_folder_replaces_it(tmp_path):
    _make_tiles(tmp_path, tms=False, xml=False)
    w = _bare2d()
    MapWidget.add_ortho_tiles(w, tmp_path, tms=False)
    MapWidget.add_ortho_tiles(w, tmp_path, tms=False)
    calls = [c for c in _tile_layer_calls(_render2d(w)) if c[0].startswith("file:///")]
    assert len(calls) == 1 and calls[0][1]["tms"] is False
    MapWidget.clear_ortho_tiles(w)
    assert not [c for c in _tile_layer_calls(_render2d(w)) if c[0].startswith("file:///")]


def test_2d_photo_poses_draw_every_point_with_heading_ticks(tmp_path):
    w = _bare2d()
    rows = [_row(1, yaw="0"), _row(2, lat=LAT0 + 0.001, yaw="90", src="fc_interp"),
            _row(3, lat=LAT0 + 0.002, yaw="", src="sfm"), _row(4, lat=None)]
    st = MapWidget.set_photo_poses(w, "fc", rows)
    assert (st["drawn"], st["skipped"]) == (3, 1)
    html = _render2d(w)
    feats = _features(html)
    assert [f["properties"]["image_id"] for f in feats] == ["IMG_0001", "IMG_0002", "IMG_0003"]
    assert [f["properties"]["heading"] for f in feats] == ["0.0°", "90.0°", "未知"]
    assert [f["properties"]["color"] for f in feats] == [cmw.PHOTO_POSE_SOURCE_COLORS[s]
                                                         for s in ("ppk", "fc_interp", "sfm")]
    assert feats[1]["geometry"]["coordinates"] == [LON0, LAT0 + 0.001]
    # 航向短線：0° → 正北 8 m；90° → 正東 8 m；沒有航向的不畫
    lines = [json.loads(m.group(1)) for m in re.finditer(r"L\.polyline\(\s*(\[\[\[.*?\]\]\])", html, flags=re.S)]
    segs = [s for line in lines for s in line]
    assert len(segs) == 2
    north = next(s for s in segs if s[0] == [LAT0, LON0])
    east = next(s for s in segs if s[0] == [LAT0 + 0.001, LON0])
    tick = MapWidget.PHOTO_POSE_TICK_M
    assert north[1] == pytest.approx([LAT0 + tick / 111320.0, LON0], abs=1e-12)
    assert east[1][0] == pytest.approx(LAT0 + 0.001, abs=1e-12)
    assert east[1][1] == pytest.approx(LON0 + tick / (111320.0 * math.cos(math.radians(LAT0 + 0.001))), abs=1e-12)
    assert _js_str("照片位姿：fc（3 張）") in html, "圖層選單要列出這一層與張數"


def test_2d_over_the_cap_draws_nothing_and_shows_a_non_blocking_notice():
    w = _bare2d()
    cap = MapWidget.PHOTO_POSES_2D_MAX
    assert cap == 5000
    st = MapWidget.set_photo_poses(w, "big", [_row(i) for i in range(cap + 1)])
    assert st == {"rows": cap + 1, "drawn": 0, "over_2d_cap": True}
    html = _render2d(w)
    assert '"FeatureCollection"' not in html
    notice = re.search(r'<div class="aeroplan-pose-cap" style="([^"]*)">([^<]*)</div>', html)
    assert notice and f"{cap + 1} 點" in notice.group(2) and "3D" in notice.group(2)
    assert "pointer-events:none" in notice.group(1)
    # 上限內照畫
    MapWidget.set_photo_poses(w, "big", [_row(i) for i in range(cap)])
    html = _render2d(w)
    assert 'aeroplan-pose-cap' not in html and html.count('"image_id": "IMG_') == cap


def test_2d_html_escapes_ids_and_clear_survey_layers_renders_once():
    w = _bare2d()
    MapWidget.set_photo_poses(w, "<b>x</b>", [dict(_row(1), image_id="<img src=x>", source="<i>")])
    html = _render2d(w)
    props = _features(html)[0]["properties"]
    assert props["image_id"] == "&lt;img src=x&gt;" and props["source"] == "&lt;i&gt;"
    assert _js_str("照片位姿：&lt;b&gt;x&lt;/b&gt;（1 張）") in html, "圖層選單用 innerHTML，代號要跳脫"
    assert _js_str("<b>x</b>") not in html
    w._renders.clear()
    MapWidget.clear_survey_layers(w)
    assert w._renders == [1] and w._photo_pose_layers == {} and w._ortho_tiles == []


def test_2d_bare_instances_without_the_new_fields_still_render():
    """既有測試用沒跑 __init__ 的裸實例呼叫 _build_map；新欄位不存在時不能炸。"""
    w = _bare2d()
    del w._ortho_tiles, w._photo_pose_layers
    assert "L.map(" in _render2d(w)


# ═══════════════════════════════════════════════════════════════════════
#  B. 假 Cesium 跑 main.html 的 SURVEY_LAYERS 段
# ═══════════════════════════════════════════════════════════════════════
_STUB = r"""
window.__log = [];
function _evt() { const l = []; return { addEventListener(f) { l.push(f); }, raise(x) { l.forEach(f => f(x)); }, n: () => l.length }; }
class _BC { constructor() { this._b = []; window.__colls.push(this); }
  add(o) { this._b.push(o); return o; } get length() { return this._b.length; } get(i) { return this._b[i]; } }
window.__colls = [];
window.Cesium = {
  Color: { fromCssColorString: (s) => ({ css: s }) },
  Cartesian3: { fromDegrees: (lon, lat, h) => [lon, lat, h], UNIT_Z: { axis: 'UNIT_Z' }, ZERO: { axis: 'ZERO' } },
  Cartesian2: { clone: (p) => ({ x: p.x, y: p.y }) },
  Math: { toRadians: (d) => d * Math.PI / 180 },
  Rectangle: { fromDegrees: (w, s, e, n) => ({ rect: [w, s, e, n] }) },
  BillboardCollection: _BC,
  TileMapServiceImageryProvider: { fromUrl: (url, o) => { __log.push(['TMS.fromUrl', url, o]);
      return window.__tmsReject ? Promise.reject(new Error('boom'))
                                : Promise.resolve({ kind: 'TMS', url, o, errorEvent: _evt() }); } },
  UrlTemplateImageryProvider: function (o) { __log.push(['UrlTemplate', o]); this.kind = 'XYZ'; this.o = o; this.errorEvent = _evt(); },
  ImageryLayer: { fromProviderAsync: (p, o) => { const L = { alpha: o.alpha, readyEvent: _evt(), errorEvent: _evt(), provider: null };
      Promise.resolve(p).then(pr => { L.provider = pr; L.readyEvent.raise(pr); }, e => L.errorEvent.raise(e)); return L; } },
  ScreenSpaceEventHandler: function () { window.__handlers = (window.__handlers || 0) + 1;
      this.setInputAction = (f, t) => { (window.__inputTypes = window.__inputTypes || []).push(t); window.__hover = f; }; },
  ScreenSpaceEventType: { MOUSE_MOVE: 'MOVE', LEFT_CLICK: 'LEFT_CLICK', LEFT_DOWN: 'LEFT_DOWN', LEFT_UP: 'LEFT_UP' },
};
const _layers = [];
window.viewer = {
  imageryLayers: { add: (l) => _layers.push(l), remove: (l, d) => { __log.push(['remove', !!d]); const i = _layers.indexOf(l); if (i >= 0) _layers.splice(i, 1); return i >= 0; },
                   contains: (l) => _layers.includes(l), get length() { return _layers.length; } },
  scene: { primitives: { _p: [], add(c) { this._p.push(c); return c; }, remove(c) { const i = this._p.indexOf(c); if (i >= 0) this._p.splice(i, 1); return i >= 0; } },
           pick: (pos) => window.__pick, canvas: { getBoundingClientRect: () => ({ left: 0, top: 0 }) } },
};
"""


@pytest.fixture(scope="module")
def _pw():
    """整個模組共用一個 Playwright（同一執行緒不能同時開兩個 sync_playwright）。"""
    sync_api = pytest.importorskip("playwright.sync_api")
    try:
        p = sync_api.sync_playwright().start()
    except Exception as e:                                   # pragma: no cover - 沒有可用的瀏覽器
        pytest.skip(f"無法啟動 Playwright（safe runner 要設 SAFE_ALLOW_BROWSER=1）：{e}")
    yield p
    p.stop()


@pytest.fixture(scope="module")
def stub_page(_pw, tmp_path_factory):
    d = tmp_path_factory.mktemp("stub")
    page_file = d / "stub.html"
    page_file.write_text("<!doctype html><html><head><meta charset='utf-8'></head><body></body></html>",
                         encoding="utf-8")
    try:
        b = _launch(_pw, ["--allow-file-access-from-files"])
        pg = b.new_page()
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.goto(page_file.as_uri())
        pg.add_script_tag(content=_STUB)
        pg.add_script_tag(content=_section())
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    pg._dir = d
    pg._errs = errs
    yield pg
    b.close()


def _launch(p, args):
    try:
        return p.chromium.launch(channel="chrome", headless=True, args=args)
    except Exception:
        return p.chromium.launch(headless=True, args=args)


@pytest.fixture
def page(stub_page):
    stub_page.evaluate("clearSurveyLayers(); __log.length = 0; window.__pick = undefined; window.__tmsReject = false;")
    yield stub_page
    assert not stub_page._errs, stub_page._errs


def _billboards(pg, lid):
    return pg.evaluate("""(id) => _photoPoseLayers[id].coll._b.map(b => ({
        pos: b.position, img: b.image, rot: b.rotation, axis: b.alignedAxis.axis, color: b.color.css,
        w: b.width, h: b.height, id: b.id }))""", lid)


ROWS = [[25.07, 121.21, 150.0, 90, "ppk", "A"],
        [25.08, 121.22, 151.0, 0, "fc_interp", "B"],
        [25.09, 121.23, 152.0, 225, "sfm", "C"],
        [25.10, 121.24, 153.0, None, "other", "D"],
        [None, 121.25, 1.0, 10, "ppk", "E"],
        ["", 121.25, 1.0, 10, "ppk", "F"],
        [91.0, 121.25, 1.0, 10, "ppk", "G"],
        [25.11, 121.26, None, -45, "ppk", "H"]]


def test_billboard_count_equals_the_valid_rows_and_they_live_in_one_collection(page):
    st = page.evaluate("(rows) => setPhotoPoses('p', {rows})", ROWS)
    assert st == {"drawn": 5, "skipped": 3, "noHeading": 1, "noHeight": 1}
    bb = _billboards(page, "p")
    assert len(bb) == 5
    assert page.evaluate("viewer.scene.primitives._p.length") == 1
    assert page.evaluate("__colls.length") == 1, "一層只能有一個 BillboardCollection（不是每點一個）"
    assert [b["id"] for b in bb] == [0, 1, 2, 3, 4]
    assert bb[0]["pos"] == [121.21, 25.07, 150.0] and bb[4]["pos"] == [121.26, 25.11, 0]


def test_rotation_is_minus_heading_with_the_axis_on_true_north(page):
    page.evaluate("(rows) => setPhotoPoses('p', rows)", ROWS)
    bb = _billboards(page, "p")
    assert bb[0]["rot"] == pytest.approx(-math.pi / 2)        # 90° → −π/2（箭頭指東）
    assert bb[1]["rot"] == 0
    assert bb[2]["rot"] == pytest.approx(-math.radians(225))
    assert bb[4]["rot"] == pytest.approx(math.radians(45))     # −45° → +π/4
    assert [b["axis"] for b in bb] == ["UNIT_Z", "UNIT_Z", "UNIT_Z", "ZERO", "UNIT_Z"]
    assert bb[3]["rot"] == 0, "沒有航向的點不可以有旋轉"


def test_one_shared_arrow_image_and_a_distinct_dot_for_missing_heading(page):
    page.evaluate("(rows) => setPhotoPoses('p', rows)", ROWS)
    bb = _billboards(page, "p")
    arrows = {b["img"] for i, b in enumerate(bb) if i != 3}
    assert len(arrows) == 1, "所有箭頭必須共用同一個圖（同一個字串 ⇒ 材質只放一份）"
    arrow = arrows.pop()
    assert isinstance(arrow, str) and arrow.startswith("data:image/png")
    assert bb[3]["img"] != arrow and bb[3]["img"].startswith("data:image/png")
    page.evaluate("(rows) => setPhotoPoses('q', rows)", ROWS[:1])
    assert _billboards(page, "q")[0]["img"] == arrow, "第二層也用同一張圖"
    assert {b["w"] for b in bb} == {20} == {b["h"] for b in bb}


def test_colours_follow_the_source_or_a_fixed_layer_colour(page):
    page.evaluate("(rows) => setPhotoPoses('p', rows)", ROWS)
    assert [b["color"] for b in _billboards(page, "p")] == \
        ["#00E676", "#FFB300", "#29B6F6", "#FFFFFF", "#00E676"]
    page.evaluate("(rows) => setPhotoPoses('p', rows, {color: '#123456'})", ROWS)
    assert {b["color"] for b in _billboards(page, "p")} == {"#123456"}


def test_resetting_or_clearing_a_layer_removes_its_collection(page):
    page.evaluate("(rows) => setPhotoPoses('p', rows)", ROWS)
    page.evaluate("(rows) => setPhotoPoses('p', rows.slice(0, 2))", ROWS)
    assert page.evaluate("viewer.scene.primitives._p.length") == 1
    assert len(_billboards(page, "p")) == 2
    assert page.evaluate("clearPhotoPoses('p')") is True
    assert page.evaluate("viewer.scene.primitives._p.length") == 0
    assert page.evaluate("clearPhotoPoses('p')") is False
    assert page.evaluate("surveyLayersInfo().poses") == {}


def test_hover_handler_is_created_once_and_does_not_take_the_left_click(page):
    page.evaluate("(rows) => { setPhotoPoses('p', rows); setPhotoPoses('q', rows); }", ROWS)
    assert page.evaluate("window.__handlers") == 1
    # 左鍵在 3D 圖上 = 加角點（既有語意）；位姿拾取只能聽滑鼠移動
    assert page.evaluate("window.__inputTypes") == ["MOVE"]


def test_pick_returns_the_image_id_and_heading_of_the_billboard_under_the_cursor(page):
    page.evaluate("(rows) => setPhotoPoses('p', rows)", ROWS)
    hit = page.evaluate("window.__pick = {collection: _photoPoseLayers.p.coll, id: 2}; photoPoseAt({x: 1, y: 1})")
    assert hit == {"layer": "p", "index": 2, "image_id": "C", "yaw": 225, "source": "sfm"}
    assert page.evaluate("(h) => photoPoseLabel(h)", hit) == "C｜航向 225.0°（sfm）"
    hit = page.evaluate("window.__pick = {collection: _photoPoseLayers.p.coll, id: 3}; photoPoseAt({x: 1, y: 1})")
    assert hit["yaw"] is None and page.evaluate("(h) => photoPoseLabel(h)", hit) == "D｜航向未知（other）"
    assert page.evaluate("window.__pick = {collection: {}, id: 1}; photoPoseAt({x: 1, y: 1})") is None
    assert page.evaluate("window.__pick = undefined; photoPoseAt({x: 1, y: 1})") is None


def test_hover_shows_the_tip_for_a_hit_and_hides_it_otherwise(page):
    page.evaluate("(rows) => setPhotoPoses('p', rows)", ROWS)
    page.evaluate("window.__pick = {collection: _photoPoseLayers.p.coll, id: 0}; __hover({endPosition: {x: 30, y: 40}})")
    page.wait_for_function("_photoPoseTip.style.display === 'block'", timeout=3000)
    assert page.evaluate("_photoPoseTip.textContent") == "A｜航向 90.0°（ppk）"
    assert page.evaluate("_photoPoseTip.style.left") == "44px"
    page.evaluate("window.__pick = undefined; __hover({endPosition: {x: 31, y: 41}})")
    page.wait_for_function("_photoPoseTip.style.display === 'none'", timeout=3000)


def test_tms_ortho_goes_through_fromUrl_and_is_removed_cleanly(page):
    ok = page.evaluate("""() => addOrthoLayer('o', 'file:///C:/a%20b/tiles/', {scheme: 'tms', alpha: 0.6,
                          rectangle: [121.2, 25.0, 121.3, 25.1], minimumLevel: 5, maximumLevel: 21})""")
    assert ok is True
    log = page.evaluate("__log")
    assert log == [["TMS.fromUrl", "file:///C:/a%20b/tiles",
                    {"rectangle": {"rect": [121.2, 25.0, 121.3, 25.1]}, "minimumLevel": 5,
                     "maximumLevel": 21, "fileExtension": "png"}]]
    page.wait_for_function("_orthoLayers.o.state === 'ready'", timeout=3000)
    info = page.evaluate("surveyLayersInfo().ortho.o")
    assert info == {"scheme": "tms", "url": "file:///C:/a%20b/tiles", "alpha": 0.6,
                    "rectangle": [121.2, 25.0, 121.3, 25.1], "state": "ready", "tileErrors": 0,
                    "inCollection": True}
    assert page.evaluate("_orthoLayers.o.layer.provider.kind") == "TMS"
    assert page.evaluate("viewer.imageryLayers.length") == 1
    assert page.evaluate("removeOrthoLayer('o')") is True
    assert page.evaluate("viewer.imageryLayers.length") == 0
    assert page.evaluate("__log[__log.length - 1]") == ["remove", True], "移除時要 destroy"
    assert page.evaluate("removeOrthoLayer('o')") is False


def test_xyz_ortho_uses_a_url_template_and_re_adding_replaces(page):
    page.evaluate("() => addOrthoLayer('x', 'file:///C:/t', {scheme: 'xyz', fileExtension: 'jpg'})")
    page.evaluate("() => addOrthoLayer('x', 'file:///C:/t', {scheme: 'xyz', fileExtension: 'jpg'})")
    assert page.evaluate("viewer.imageryLayers.length") == 1
    log = page.evaluate("__log")
    assert log[0] == ["UrlTemplate", {"url": "file:///C:/t/{z}/{x}/{y}.jpg"}]
    page.evaluate("() => addOrthoLayer('y', 'https://h/{z}/{x}/{y}.png?k=1', {scheme: 'xyz'})")
    assert page.evaluate("__log[__log.length - 1]")[1]["url"] == "https://h/{z}/{x}/{y}.png?k=1"
    page.wait_for_function("_orthoLayers.y.state === 'ready'", timeout=3000)
    assert page.evaluate("_orthoLayers.y.layer.alpha") == 1


def test_bad_scheme_adds_nothing_and_a_failed_provider_is_reported(page):
    assert page.evaluate("() => addOrthoLayer('b', 'file:///C:/t', {scheme: 'wmts'})") is False
    assert page.evaluate("viewer.imageryLayers.length") == 0 and page.evaluate("__log") == []
    page.evaluate("window.__tmsReject = true; addOrthoLayer('e', 'file:///C:/t', {scheme: 'tms'})")
    page.wait_for_function("_orthoLayers.e.state === 'error'", timeout=3000)


def test_tile_errors_are_counted_after_the_provider_is_ready(page):
    page.evaluate("() => addOrthoLayer('o', 'file:///C:/t', {scheme: 'tms'})")
    page.wait_for_function("_orthoLayers.o.state === 'ready'", timeout=3000)
    page.evaluate("const e = _orthoLayers.o.layer.provider.errorEvent; e.raise(1); e.raise(2); e.raise(3);")
    assert page.evaluate("surveyLayersInfo().ortho.o.tileErrors") == 3


def test_csv_parsing_handles_bom_crlf_quotes_and_missing_columns(page):
    text = ('\ufeffimage_id,file,lat_deg,lon_deg,h_ell_m,yaw_deg,source\r\n'
            'A,"img/a,1.jpg",25.07,121.21,150.5,90,ppk\r\n'
            'B,"say ""hi""",25.08,121.22,,,"sfm"\r\n\r\n')
    rows = page.evaluate("(t) => parsePhotoPosesText(t)", text)
    assert rows == [["25.07", "121.21", "150.5", "90", "ppk", "A"],
                    ["25.08", "121.22", "", "", "sfm", "B"]]
    st = page.evaluate("(t) => setPhotoPoses('c', parsePhotoPosesText(t))", text)
    assert st == {"drawn": 2, "skipped": 0, "noHeading": 1, "noHeight": 1}
    assert _billboards(page, "c")[0]["rot"] == pytest.approx(-math.pi / 2)
    rows = page.evaluate("(t) => parsePhotoPosesText(t)", "lat_deg,lon_deg\n1,2\n")
    assert rows == [["1", "2", None, None, None, None]]
    err = page.evaluate("(t) => { try { parsePhotoPosesText(t); return null; } catch (e) { return e.message; } }",
                        "lat,lon\n1,2\n")
    assert err and "lat_deg" in err
    assert page.evaluate("(t) => parsePhotoPosesText(t)", ' {"rows": [[1, 2, 3, 4, "ppk", "x"]]}') == \
        [[1, 2, 3, 4, "ppk", "x"]]
    assert page.evaluate("(t) => parsePhotoPosesText(t)", '[[1, 2, null, null, "", ""]]') == \
        [[1, 2, None, None, "", ""]]


def test_load_photo_poses_reads_a_local_file_by_xhr_and_reports_failures(page):
    d = page._dir / "地圖 資料"
    d.mkdir(exist_ok=True)
    f = d / "photos.csv"
    f.write_text("image_id,lat_deg,lon_deg,h_ell_m,yaw_deg,source\n"
                 + "".join(f"I{i},{25 + i * 1e-4},121.2,150,{i * 10},ppk\n" for i in range(37)),
                 encoding="utf-8")
    st = page.evaluate("(u) => loadPhotoPosesUrl('f', u)", f.as_uri())
    assert st == {"drawn": 37, "skipped": 0, "noHeading": 0, "noHeight": 0}
    assert page.evaluate("surveyLayersInfo().poses.f") == {
        "drawn": 37, "skipped": 0, "noHeading": 0, "noHeight": 0, "length": 37, "url": f.as_uri()}
    assert _billboards(page, "f")[3]["rot"] == pytest.approx(-math.radians(30))
    res = page.evaluate("(u) => loadPhotoPosesUrl('m', u)", (d / "none.csv").as_uri())
    assert "error" in res and "m" not in page.evaluate("surveyLayersInfo().poses")
    bad = d / "bad.csv"
    bad.write_text("a,b\n1,2\n", encoding="utf-8")
    assert "lat_deg" in page.evaluate("(u) => loadPhotoPosesUrl('b', u)", bad.as_uri())["error"]


def test_an_older_load_or_a_clear_supersedes_an_in_flight_load(page):
    d = page._dir
    big, small = d / "big.csv", d / "small.csv"
    big.write_text("lat_deg,lon_deg,yaw_deg\n" + "25,121,0\n" * 20000, encoding="utf-8")
    small.write_text("lat_deg,lon_deg,yaw_deg\n25,121,0\n25,121,1\n", encoding="utf-8")
    res = page.evaluate("""async ([a, b]) => { const p1 = loadPhotoPosesUrl('s', a); const p2 = loadPhotoPosesUrl('s', b);
                           return [await p1, await p2]; }""", [big.as_uri(), small.as_uri()])
    assert res[0] == {"superseded": True} and res[1]["drawn"] == 2
    assert len(_billboards(page, "s")) == 2
    res = page.evaluate("async (a) => { const p = loadPhotoPosesUrl('t', a); clearPhotoPoses('t'); return await p; }",
                        big.as_uri())
    assert res == {"superseded": True} and "t" not in page.evaluate("surveyLayersInfo().poses")


def test_clear_survey_layers_clears_both_kinds(page):
    page.evaluate("(rows) => { setPhotoPoses('p', rows); addOrthoLayer('o', 'file:///C:/t', {scheme: 'tms'}); }", ROWS)
    page.evaluate("clearSurveyLayers()")
    assert page.evaluate("surveyLayersInfo()") == {"ortho": {}, "poses": {}}
    assert page.evaluate("viewer.imageryLayers.length") == 0
    assert page.evaluate("viewer.scene.primitives._p.length") == 0


# ═══════════════════════════════════════════════════════════════════════
#  C. 真的本機 Cesium 1.115：讀像素驗航向方向、TMS 真的畫得出來
# ═══════════════════════════════════════════════════════════════════════
_REAL_ARGS = ["--allow-file-access-from-files", "--ignore-gpu-blocklist", "--enable-gpu",
              "--use-angle=d3d11", "--enable-unsafe-swiftshader"]


@pytest.fixture(scope="module")
def real_page(_pw, tmp_path_factory):
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    d = tmp_path_factory.mktemp("real")
    page_file = d / "real.html"
    page_file.write_text(f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<script src="{(CESIUM_DIR / 'Cesium.js').as_uri()}"></script>
<style>html,body,#c{{width:100%;height:100%;margin:0;padding:0;overflow:hidden}}</style>
</head><body><div id="c"></div><script>
Cesium.Ion.defaultAccessToken = '';
window.viewer = new Cesium.Viewer('c', {{ terrainProvider: new Cesium.EllipsoidTerrainProvider(),
  baseLayer: false, baseLayerPicker: false, navigationHelpButton: false, homeButton: false,
  sceneModePicker: false, geocoder: false, timeline: false, animation: false, fullscreenButton: false,
  infoBox: false, selectionIndicator: false, skyBox: false, skyAtmosphere: false,
  contextOptions: {{ webgl: {{ preserveDrawingBuffer: true, antialias: false }} }} }});
viewer.scene.globe.baseColor = Cesium.Color.BLACK; viewer.scene.backgroundColor = Cesium.Color.BLACK;
viewer.scene.fog.enabled = false; viewer.scene.globe.showGroundAtmosphere = false;
window.frames_ = (k) => new Promise(r => {{ let n = 0; const f = () => (++n >= k ? r() : requestAnimationFrame(f)); requestAnimationFrame(f); }});
window.pixels_ = () => {{ const c = viewer.scene.canvas, t = document.createElement('canvas');
  t.width = c.width; t.height = c.height; const g = t.getContext('2d'); g.drawImage(c, 0, 0);
  return {{ d: g.getImageData(0, 0, c.width, c.height).data, w: c.width, h: c.height,
           sx: c.width / c.clientWidth, sy: c.height / c.clientHeight }}; }};
</script></body></html>""", encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
        pg = b.new_page(viewport={"width": 640, "height": 480})
        pg.goto(page_file.as_uri())
        pg.wait_for_function("window.viewer !== undefined", timeout=60000)
        pg.add_script_tag(content=_section())
    except Exception as e:                                   # pragma: no cover - 沒有 WebGL 等
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._dir = d
    yield pg
    b.close()


_TIP_JS = """async ([lat, lon, hdg, cam]) => {
  viewer.camera.setView({destination: Cesium.Cartesian3.fromDegrees(lon, lat, 400),
                         orientation: {heading: Cesium.Math.toRadians(cam), pitch: -Math.PI / 2, roll: 0}});
  setPhotoPoses('r', [[lat, lon, 0, hdg, 'x', 'A']], {color: '#FF0000', size: 64});
  await frames_(20);
  const ctr = Cesium.SceneTransforms.wgs84ToWindowCoordinates(viewer.scene, Cesium.Cartesian3.fromDegrees(lon, lat, 0));
  const P = pixels_();
  let best = -1, bx = 0, by = 0, n = 0;
  for (let y = 0; y < P.h; y++) for (let x = 0; x < P.w; x++) {
    const k = (y * P.w + x) * 4;
    if (P.d[k] > 150 && P.d[k + 1] < 90 && P.d[k + 2] < 90) {
      n++; const dx = x / P.sx - ctr.x, dy = y / P.sy - ctr.y, r = dx * dx + dy * dy;
      if (r > best) { best = r; bx = dx; by = dy; }
    }
  }
  clearPhotoPoses('r');
  return {n, angle: (Math.atan2(bx, -by) * 180 / Math.PI + 360) % 360};
}"""


@pytest.mark.parametrize("heading,cam", [(0.0, 0.0), (90.0, 0.0), (225.0, 0.0), (0.0, 90.0), (90.0, 90.0)])
def test_real_cesium_arrow_tip_points_along_the_true_north_heading(real_page, heading, cam):
    """俯視：箭頭尖端（離中心最遠的紅色像素）在螢幕上的方位角 = 航向 − 相機航向（螢幕上方 = 相機朝向）。

    相機朝北時螢幕上方就是北；相機轉向東（cam=90）時，朝北的箭頭要指向螢幕左邊 ——
    這一組只有 alignedAxis = UNIT_Z（把「上」綁在真北）才會對。
    """
    r = real_page.evaluate(_TIP_JS, [LAT0, LON0, heading, cam])
    if r["n"] == 0:                                           # pragma: no cover - 沒有 WebGL 輸出
        pytest.skip("畫面讀不到像素（WebGL 不可用？）")
    want = (heading - cam) % 360
    diff = (r["angle"] - want + 540) % 360 - 180
    assert abs(diff) < 10, f"航向 {heading}°、相機 {cam}°：尖端在螢幕 {r['angle']:.1f}°，應為 {want}°（差 {diff:.1f}°）"


def test_real_cesium_draws_tms_tiles_from_a_path_with_space_and_cjk(real_page):
    png = pytest.importorskip("PIL.Image")
    import io
    buf = io.BytesIO()
    png.new("RGBA", (256, 256), (0, 200, 0, 255)).save(buf, format="PNG")
    root = real_page._dir / "a b" / "地圖" / "orthophoto_tiles"
    _make_tiles(root, tms=True, xml=False, levels=(13, 14, 15, 16, 17), cols=3, rows=3, payload=buf.getvalue())
    info = cmw.inspect_tile_dir(root, "tms")
    url = cmw.local_path_to_file_url(root)
    res = real_page.evaluate("""async ([url, rect, lat, lon]) => {
      addOrthoLayer('o', url, {scheme: 'tms', rectangle: rect, minimumLevel: 13, maximumLevel: 17, alpha: 1});
      viewer.camera.setView({destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1500),
                             orientation: {heading: 0, pitch: -Math.PI / 2, roll: 0}});
      for (let i = 0; i < 300 && !(_orthoLayers.o.state === 'ready' && viewer.scene.globe.tilesLoaded && i > 30); i++) await frames_(1);
      const P = pixels_();
      const k = ((P.h >> 1) * P.w + (P.w >> 1)) * 4;
      const out = {px: [P.d[k], P.d[k + 1], P.d[k + 2]], state: _orthoLayers.o.state,
                   tms: _orthoLayers.o.layer.imageryProvider instanceof Cesium.TileMapServiceImageryProvider};
      out.removed = removeOrthoLayer('o');
      out.left = viewer.imageryLayers.length;
      return out;
    }""", [url, list(info["bounds"]), (info["bounds"][1] + info["bounds"][3]) / 2,
           (info["bounds"][0] + info["bounds"][2]) / 2])
    assert res["state"] == "ready" and res["tms"] is True
    assert res["px"][1] > 150 and res["px"][0] < 60, f"畫面中心不是瓦片的綠色：{res['px']}"
    assert res["removed"] is True and res["left"] == 0


def test_the_whole_main_html_template_loads_and_exposes_the_survey_functions(_pw, tmp_path):
    """整份 main.html（佔位字串照 _load_cesium 的方式填）在真的 Cesium 下沒有腳本錯誤，
    新函式是全域可呼叫的。網路請求全部擋掉（Google 圖磚讀不到是預期的）。"""
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    html = MAIN_HTML.read_text(encoding="utf-8")
    for k, v in {"__CESIUM_JS__": (CESIUM_DIR / "Cesium.js").as_uri(),
                 "__CESIUM_CSS__": (CESIUM_DIR / "Widgets" / "widgets.css").as_uri(),
                 "__CESIUM_TOKEN__": "", "__INIT_LAT__": str(LAT0), "__INIT_LON__": str(LON0),
                 "__INIT_HEIGHT__": "2000", "__PLANE_MODEL_URI__": "", "__COPTER_MODEL_URI__": "",
                 "__VTOL_MODEL_URI__": "", "__VTOL_MODEL_SCALE__": "0.01", "__VTOL_HEADING_OFFSET__": "-90",
                 "__VTOL_PITCH_OFFSET__": "90", "__VTOL_ROLL_OFFSET__": "180"}.items():
        html = html.replace(k, v)
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
        pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:"))
                 else r.abort())
        pg.goto(f.as_uri())
        pg.wait_for_function("typeof viewer !== 'undefined' && typeof surveyLayersInfo === 'function'",
                             timeout=60000)
        st = pg.evaluate("setPhotoPoses('p', [[25.07, 121.21, 150, 90, 'ppk', 'A'], [25.07, 121.21, 150, null, 'sfm', 'B']])")
        info = pg.evaluate("surveyLayersInfo()")
    finally:
        b.close()
    assert not errs, errs
    assert st == {"drawn": 2, "skipped": 0, "noHeading": 1, "noHeight": 0}
    assert info["poses"]["p"]["length"] == 2 and info["ortho"] == {}
