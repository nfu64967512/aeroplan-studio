"""風場視覺化引擎 ui/widgets/web_shared/wind_flow.js —— 值層級的鎖（無頭 Chrome＋線性測試轉接器）。

引擎同時給 2D Leaflet 頁與 3D Cesium 頁用；這裡不載 Leaflet／Cesium，只用一個線性投影的假轉接器：
    x = ox + (lon − LON0)·K·cos(LAT0)、y = oy − (lat − LAT0)·K     （K = 每度緯度的像素）
所以「畫面上的位移」可以從粒子的經緯度用同一條公式獨立算回來，不必相信引擎自己回報的步長。

鎖住的值：
  * 氣象慣例 dirToUV / uvToDir（風「從」哪裡來；270° ⇒ u = +s）
  * interp 的雙線性值、範圍外／四角有 null 或 NaN ⇒ null、遞減座標自動翻轉、1×1＋bbox 均勻場
  * speedColor 綠 < 0.7·limit ≤ 琥珀 < limit ≤ 紅
  * 西風 5 m/s：粒子往東、每幀螢幕位移 = 5 × pxPerMps（與縮放無關），60 幀存活者位移 180 px
  * 視角不變時 0 次 adapter.project（只在 viewKey 變時重投影）；viewKey 一變軌跡清成 0（預設；2D 頁）
  * trailsFollowView（3D 頁）：視角每幀都動（追機）時軌跡不清、用位置歷史在新視角重畫在「歷史投影到的位置」；
    縮放一跳就丟歷史；定點鏡頭逐像素與預設相同
  * 箭頭指向下風處（從 90° 來 ⇒ 指西）、長度夾 18–70 px、文字＝label＋風速
  * 圖例文字、animate=false 停止 frames、destroy 移除 canvas、壞輸入不丟例外
  * 轉接器對畫面外回 null（leader 的 2D／3D 轉接器）時仍找得到可見區、邊緣沒有空白帶

要開瀏覽器：用 safe runner 時要設 SAFE_ALLOW_BROWSER=1，否則整個檔 skip。
"""
from __future__ import annotations

import math
import os
import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
# 突變測試用：可指向 wind_flow.js 的突變副本（不改 repo 檔）
WIND_JS = pathlib.Path(os.environ.get("WIND_FLOW_JS") or REPO / "ui" / "widgets" / "web_shared" / "wind_flow.js")

W_PX, H_PX = 800, 600
LAT0, LON0, K = 25.0, 121.0, 10000.0
KX = K * math.cos(math.radians(LAT0))
LAT1, LON1 = LAT0 + H_PX / K, LON0 + W_PX / KX
LATC, LONC = (LAT0 + LAT1) / 2, (LON0 + LON1) / 2


# ═══════════════════════════════════════════════════════════════════════
#  A. 原始碼：能被內嵌／被 Jinja／被佔位字串替換而不壞
# ═══════════════════════════════════════════════════════════════════════
def test_source_is_utf8_lf_and_safe_to_inline_into_both_pages():
    raw = WIND_JS.read_bytes()
    src = raw.decode("utf-8")
    assert b"\r" not in raw, "要 LF 行尾"
    assert "</script" not in src.lower(), "內嵌進 <script> 時會提早結束"
    for tok in ("{{", "{%", "{#"):
        assert tok not in src, f"folium/Jinja 模板會吃掉 {tok!r}"
    assert not re.search(r"__[A-Z][A-Z_]*__", src), "main.html 用 __XXX__ 佔位字串替換"
    assert not re.search(r"^\s*(import|export)\s", src, flags=re.M), "不可用 ES module"
    assert "root.AeroWind = {" in src


# ═══════════════════════════════════════════════════════════════════════
#  B. 無頭 Chrome
# ═══════════════════════════════════════════════════════════════════════
_PAGE_JS = r"""
window.LAT0 = %(LAT0)r; window.LON0 = %(LON0)r; window.K = %(K)r; window.KX = %(KX)r;
window.LAT1 = %(LAT1)r; window.LON1 = %(LON1)r; window.LATC = %(LATC)r; window.LONC = %(LONC)r;
window.__view = {ox: 0, oy: %(H)d, key: 'v0', nullOff: false, s: 1};
window.__calls = 0;
window.screenOf = function (lat, lon) {          // s = 縮放倍數（拉近拉遠）；其他測試都是 1
  return {x: __view.ox + (lon - LON0) * KX * __view.s, y: __view.oy - (lat - LAT0) * K * __view.s};
};
window.__adapter = {
  project: function (lat, lon, h) {
    __calls++;
    var p = screenOf(lat, lon);
    if (__view.nullOff && (p.x < 0 || p.x > %(W)d || p.y < 0 || p.y > %(H)d)) return null;
    return p;
  },
  viewKey: function () { return __view.key; }
};
window.newWind = function (opts) {
  if (window.W) W.destroy();
  var host = document.getElementById('host');
  host.style.width = '%(W)dpx'; host.style.height = '%(H)dpx';
  __view.ox = 0; __view.oy = %(H)d; __view.key = 'v0'; __view.nullOff = false; __view.s = 1;
  window.W = AeroWind.create(host, __adapter);
  W.setOptions(Object.assign({seed: 12345}, opts || {}));
  return true;
};
window.uniform = function (dir, s, extra) {
  var uv = AeroWind.dirToUV(dir, s);
  return Object.assign({lats: [LATC], lons: [LONC], u: [[uv[0]]], v: [[uv[1]]],
                        bbox: [LAT0 - 0.01, LON0 - 0.01, LAT1 + 0.01, LON1 + 0.01], height_m: 100,
                        label: '規劃風'}, extra || {});
};
// n 幀之間「沒有重生」的粒子（年齡正好 +n），用測試自己的投影公式算螢幕位移
window.measure = function (id, n) {
  var a = W._debug(id); W._step(n); var b = W._debug(id);
  var dx = [], dy = [];
  for (var k = 0; k < Math.min(a.n, b.n); k++) {
    if (a.alive[k] && b.alive[k] && b.age[k] - a.age[k] === n) {
      var p = screenOf(a.lat[k], a.lon[k]), q = screenOf(b.lat[k], b.lon[k]);
      dx.push(q.x - p.x); dy.push(q.y - p.y);
    }
  }
  var mean = function (v) { var s = 0; for (var i = 0; i < v.length; i++) s += v[i]; return v.length ? s / v.length : NaN; };
  var len = dx.map(function (x, i) { return Math.sqrt(x * x + dy[i] * dy[i]); });
  return {n: dx.length, total: a.n, mdx: mean(dx), mdy: mean(dy),
          minLen: Math.min.apply(null, len), maxLen: Math.max.apply(null, len),
          minDx: Math.min.apply(null, dx)};
};
window.pixHash = function (cv) {                 // 整張畫布的 RGBA 指紋（逐像素相同才相等）
  var d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data, h = 2166136261;
  for (var i = 0; i < d.length; i++) { h = Math.imul(h ^ d[i], 16777619); }
  return h >>> 0;
};
window.alphaAt = function (cv, x, y) {            // CSS 像素 (x, y) 附近 3×3 的最大 alpha（線寬 1.5 px、反鋸齒）
  var r = W.state().size[2], X = Math.round(x * r), Y = Math.round(y * r), best = 0;
  if (X < 1 || Y < 1 || X >= cv.width - 1 || Y >= cv.height - 1) return -1;
  var d = cv.getContext('2d').getImageData(X - 1, Y - 1, 3, 3).data;
  for (var i = 3; i < d.length; i += 4) if (d[i] > best) best = d[i];
  return best;
};
window.alphaSum = function (cv) {
  var d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data, s = 0;
  for (var i = 3; i < d.length; i += 4) s += d[i];
  return s;
};
window.colorStats = function (cv, x0, y0, x1, y1, hex, tol) {
  var w = x1 - x0, d = cv.getContext('2d').getImageData(x0, y0, w, y1 - y0).data;
  var r = parseInt(hex.slice(1, 3), 16), g = parseInt(hex.slice(3, 5), 16), b = parseInt(hex.slice(5, 7), 16);
  var n = 0, sx = 0, sy = 0, far = 0;
  for (var i = 0; i < d.length; i += 4) {
    if (d[i + 3] > 200 && Math.abs(d[i] - r) <= tol && Math.abs(d[i + 1] - g) <= tol && Math.abs(d[i + 2] - b) <= tol) {
      var p = i / 4, x = x0 + p %% w, y = y0 + Math.floor(p / w);
      n++; sx += x; sy += y;
      if (Math.abs(y - (y0 + y1) / 2) >= 3) far++;
    }
  }
  return {n: n, cx: n ? sx / n : null, cy: n ? sy / n : null, far: far};
};
// 軌跡層每個像素歸到最近的等級色（只算 alpha 夠高的）
window.trailClasses = function () {
  var cv = W._trailCanvas, d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data;
  var C = AeroWind.COLORS, names = ['ok', 'caution', 'warn'], out = {ok: 0, caution: 0, warn: 0};
  var rgb = names.map(function (k) { var h = C[k]; return [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)]; });
  for (var i = 0; i < d.length; i += 4) {
    if (d[i + 3] < 120) continue;
    var best = 0, bd = 1e9;
    for (var c = 0; c < 3; c++) {
      var e = Math.abs(d[i] - rgb[c][0]) + Math.abs(d[i + 1] - rgb[c][1]) + Math.abs(d[i + 2] - rgb[c][2]);
      if (e < bd) { bd = e; best = c; }
    }
    out[names[best]]++;
  }
  return out;
};
""" % dict(LAT0=LAT0, LON0=LON0, K=K, KX=KX, LAT1=LAT1, LON1=LON1, LATC=LATC, LONC=LONC, W=W_PX, H=H_PX)


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


def _launch(p):
    try:
        return p.chromium.launch(channel="chrome", headless=True)
    except Exception:
        return p.chromium.launch(headless=True)


@pytest.fixture(scope="module")
def browser(_pw):
    try:
        b = _launch(_pw)
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    yield b
    b.close()


@pytest.fixture(scope="module")
def wind_page(browser, tmp_path_factory):
    """跟兩個地圖頁一樣用 <script src="file:///…/wind_flow.js"> 載入（中文＋空白路徑也要能讀）。"""
    d = tmp_path_factory.mktemp("wind") / "風場 頁"
    d.mkdir()
    f = d / "wind.html"
    f.write_text(f"""<!doctype html><html><head><meta charset="utf-8">
<style>html,body{{margin:0;padding:0;background:#fff}}#host{{position:relative;width:{W_PX}px;height:{H_PX}px;overflow:hidden}}</style>
</head><body><div id="host"></div>
<script src="{WIND_JS.as_uri()}"></script>
<script>{_PAGE_JS}</script></body></html>""", encoding="utf-8")
    pg = browser.new_page(viewport={"width": 1000, "height": 700})
    errs, cons = [], []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.on("console", lambda m: cons.append((m.type, m.text)))
    pg.goto(f.as_uri())
    pg.wait_for_function("typeof AeroWind === 'object' && typeof newWind === 'function'", timeout=10000)
    pg._errs, pg._cons = errs, cons
    yield pg
    pg.close()


@pytest.fixture
def page(wind_page):
    wind_page.evaluate("newWind()")
    wind_page._cons.clear()
    yield wind_page
    assert not wind_page._errs, wind_page._errs
    bad = [t for k, t in wind_page._cons if k == "error"]
    assert not bad, bad


def _console(page, kind):
    return [t for k, t in page._cons if k == kind]


# ── B1. 純函式 ──────────────────────────────────────────────────────────
@pytest.mark.parametrize("d,exp", [(0, (0, -5)), (90, (-5, 0)), (180, (0, 5)), (270, (5, 0)),
                                   (45, (-5 / math.sqrt(2), -5 / math.sqrt(2))), (-90, (5, 0)), (630, (5, 0))])
def test_dir_to_uv_uses_the_meteorological_from_convention(page, d, exp):
    u, v = page.evaluate("([d]) => AeroWind.dirToUV(d, 5)", [d])
    assert (u, v) == pytest.approx(exp, abs=1e-12)
    assert page.evaluate("([d]) => { const r = AeroWind.dirToUV(d, 5); return [r.u, r.v]; }", [d]) == [u, v]
    back = page.evaluate("([u, v]) => AeroWind.uvToDir(u, v)", [u, v])
    assert back[1] == pytest.approx(5.0, abs=1e-12)
    assert back[0] == pytest.approx(d % 360, abs=1e-9)


def test_dir_to_uv_of_garbage_is_nan_not_zero(page):
    r = page.evaluate("[AeroWind.dirToUV(null, 5), AeroWind.dirToUV(90, undefined), AeroWind.dirToUV('abc', 1)]")
    assert all(math.isnan(x) for pair in r for x in pair)
    assert page.evaluate("AeroWind.dirToUV('270', '4')") == [4, 0]


GRID = {"lats": [25, 26], "lons": [121, 122, 123],
        "u": [[0, 10, 20], [100, 110, 120]], "v": [[1, 2, 3], [4, 5, 6]]}


def _interp(page, field, lat, lon):
    return page.evaluate("([f, a, b]) => AeroWind.interp(f, a, b)", [field, lat, lon])


@pytest.mark.parametrize("lat,lon,exp", [
    (25, 121, (0, 1)), (26, 123, (120, 6)), (25, 123, (20, 3)), (26, 121, (100, 4)),
    (25.5, 121.5, (55, 3)), (25.25, 122.75, (42.5, 3.5)), (25.0, 122.5, (15, 2.5))])
def test_interp_is_bilinear_at_corners_centres_and_edges(page, lat, lon, exp):
    r = _interp(page, GRID, lat, lon)
    assert (r["u"], r["v"]) == pytest.approx(exp, abs=1e-9)


@pytest.mark.parametrize("lat,lon", [(24.999, 121.5), (26.001, 121.5), (25.5, 120.999), (25.5, 123.001)])
def test_interp_outside_the_grid_is_null(page, lat, lon):
    assert _interp(page, GRID, lat, lon) is None


def test_interp_null_or_nan_corner_kills_only_the_cells_that_touch_it(page):
    g = {**GRID, "u": [[None, 10, 20], [100, 110, 120]]}
    assert _interp(page, g, 25.5, 121.5) is None
    assert _interp(page, g, 25, 121) is None
    assert _interp(page, g, 25.5, 122.5)["u"] == pytest.approx(65)
    g = {**GRID, "v": [[1, 2, 3], [4, 5, float("nan")]]}
    assert _interp(page, g, 25.5, 122.5) is None
    assert _interp(page, g, 25.5, 121.5)["v"] == pytest.approx(3)


def test_descending_coordinates_are_flipped_not_rejected(page):
    g = {"lats": [26, 25], "lons": [123, 122, 121],
         "u": [[120, 110, 100], [20, 10, 0]], "v": [[6, 5, 4], [3, 2, 1]]}
    r = _interp(page, g, 25.25, 122.75)
    assert (r["u"], r["v"]) == pytest.approx((42.5, 3.5), abs=1e-9)


def test_uniform_field_covers_its_bbox_or_a_small_default_box(page):
    f = {"lats": [25.5], "lons": [121.5], "u": [[3]], "v": [[-4]], "bbox": [25, 121, 26, 122]}
    assert _interp(page, f, 25.1, 121.9) == {"u": 3, "v": -4}
    assert _interp(page, f, 26.1, 121.5) is None
    f.pop("bbox")
    assert _interp(page, f, 25.505, 121.5) == {"u": 3, "v": -4}
    assert _interp(page, f, 25.52, 121.5) is None, "沒有 bbox 時只蓋 ±0.01°"
    assert _interp(page, {"lats": [], "lons": [], "u": [], "v": []}, 25, 121) is None
    assert _interp(page, None, 25, 121) is None


def test_speed_color_is_green_amber_red_with_red_at_the_limit(page):
    C = page.evaluate("AeroWind.COLORS")
    got = page.evaluate("""() => [0, 2.09, 2.1, 2.99, 3.0, 3.01, 50, -2.5].map(s => AeroWind.speedColor(s, 3))""")
    assert got == [C["ok"], C["ok"], C["caution"], C["caution"], C["warn"], C["warn"], C["warn"], C["caution"]]
    assert page.evaluate("AeroWind.speedColor(2.9, 'x')") == C["caution"], "無效上限 ⇒ 3.0"
    assert page.evaluate("AeroWind.speedColor(9, 10)") == C["caution"]
    assert page.evaluate("AeroWind.speedColor(NaN, 3)") == C["unknown"]
    assert len({C["ok"], C["caution"], C["warn"]}) == 3


# ── B2. 粒子 ────────────────────────────────────────────────────────────
def test_canvas_is_one_absolute_click_through_layer_sized_to_the_host(page):
    st = page.evaluate("""() => { const c = document.querySelectorAll('#host canvas'); const s = getComputedStyle(c[0]);
        return {n: c.length, pos: s.position, pe: s.pointerEvents, z: s.zIndex, w: c[0].width, h: c[0].height,
                cw: c[0].clientWidth, dpr: devicePixelRatio}; }""")
    assert st["n"] == 1 and st["pos"] == "absolute" and st["pe"] == "none" and st["z"] == "450"
    assert (st["w"], st["h"]) == (round(W_PX * st["dpr"]), round(H_PX * st["dpr"])) and st["cw"] == W_PX
    page.evaluate("W.setOptions({zIndex: 650, opacity: 0.5}); document.getElementById('host').style.width = '640px';"
                  "W.setField(uniform(270, 5)); W._step(1)")
    assert page.evaluate("[W.canvas.style.zIndex, W.canvas.style.opacity, W.canvas.width, W._trailCanvas.width]") == \
        ["650", "0.5", 640, 640]


def test_west_wind_moves_particles_east_at_speed_times_px_per_mps(page):
    page.evaluate("W.setField(uniform(270, 5))")
    st = page.evaluate("W._step(5)")
    f = st["fields"]["default"]
    assert f["n_particles"] == 400, "整個畫面 480000 px² / 1200 = 400"
    assert st["particles"] == 400 and st["field"] is True and st["cells"] == 1
    ages = page.evaluate("W._debug().age")
    assert len(set(ages)) >= 60, "初始年齡要隨機（0–89），否則 90 幀後全部同時重生"
    one = page.evaluate("measure('default', 1)")
    assert one["n"] > 350
    assert one["mdx"] == pytest.approx(3.0, abs=1e-6) and abs(one["mdy"]) < 1e-6
    assert one["minLen"] == pytest.approx(3.0, abs=1e-6) and one["maxLen"] == pytest.approx(3.0, abs=1e-6)
    sixty = page.evaluate("measure('default', 60)")
    assert sixty["n"] > 40, sixty
    assert sixty["mdx"] == pytest.approx(180.0, abs=1e-4) and abs(sixty["mdy"]) < 1e-4
    ls = page.evaluate("W.state().fields.default.last_step")
    assert (ls["dx"], ls["dy"], ls["len"]) == pytest.approx((3.0, 0.0, 3.0), abs=1e-6)


def test_on_screen_speed_scales_with_px_per_mps_and_wind_speed(page):
    page.evaluate("W.setOptions({pxPerMps: 1.2}); W.setField(uniform(270, 5)); W._step(3)")
    assert page.evaluate("measure('default', 1)")["mdx"] == pytest.approx(6.0, abs=1e-6)
    page.evaluate("W.setOptions({px_per_mps: 0.6}); W.setField(uniform(270, 2)); W._step(3)")
    assert page.evaluate("measure('default', 1)")["mdx"] == pytest.approx(1.2, abs=1e-6)


@pytest.mark.parametrize("d", [0, 45, 90, 180, 225])
def test_particles_move_downwind_on_screen(page, d):
    page.evaluate("([d]) => { W.setField(uniform(d, 5)); W._step(5); }", [d])
    m = page.evaluate("measure('default', 1)")
    # 下風：東分量 −sin、北分量 −cos；螢幕 y 向下 ⇒ dy = +3·cos(d)
    assert m["mdx"] == pytest.approx(-3 * math.sin(math.radians(d)), abs=2e-3)
    assert m["mdy"] == pytest.approx(3 * math.cos(math.radians(d)), abs=2e-3)


def test_particles_respawn_inside_the_extent_and_never_sit_on_null_cells(page):
    lats = [LAT0 + i * (LAT1 - LAT0) / 4 for i in range(5)]
    lons = [LON0 + j * (LON1 - LON0) / 4 for j in range(5)]
    u = [[3.0] * 5 for _ in range(5)]
    v = [[1.0] * 5 for _ in range(5)]
    u[2][2] = None                                     # 中央節點 ⇒ 周圍四格都不能有粒子
    page.evaluate("([f]) => W.setField('g', f)", [{"lats": lats, "lons": lons, "u": u, "v": v, "label": "格點"}])
    page.evaluate("W._step(120)")
    # 檢查發生在每一步的開頭 ⇒ 這一步剛跨進 null 區／出界的粒子最多深入一步（√10·0.6 ≈ 1.9 px），
    # 下一步就會重生；「深」入 null 區（離邊界 > 3 px）或出界 > 3 px 的必須是 0。
    res = page.evaluate("""() => { const d = W._debug('g'); let deep = 0, shallow = 0, out = 0, n = 0;
        for (let k = 0; k < d.n; k++) { if (!d.alive[k]) continue; n++;
          const p = screenOf(d.lat[k], d.lon[k]);
          if (W.interpAt('g', d.lat[k], d.lon[k]) === null) {
            if (p.x > 203 && p.x < 597 && p.y > 153 && p.y < 447) deep++; else shallow++; }
          if (p.x < -13 || p.x > 813 || p.y < -13 || p.y > 613) out++; }
        return {n, deep, shallow, out}; }""")
    assert res["n"] > 300 and res["deep"] == 0 and res["out"] == 0, res
    assert res["shallow"] < 0.03 * res["n"], res
    assert page.evaluate("W.state().cells") == 24


def test_a_static_view_costs_zero_projections_per_frame(page):
    """契約：只有 viewKey 變（或資料變）才重新投影；畫面不動時每幀用快取的投影格網。"""
    page.evaluate("W.setField(uniform(270, 5)); W._step(3)")
    assert page.evaluate("() => { const c = __calls; W._step(20); return __calls - c; }") == 0
    n = page.evaluate("() => { const c = __calls; __view.key = 'pan'; __view.ox -= 10; W._step(1); return __calls - c; }")
    assert 0 < n < 5000


def test_view_key_change_clears_the_trails_completely(page):
    page.evaluate("W.setField(uniform(270, 5))")
    a1, a2, a3 = page.evaluate("""() => { W._step(40); const a1 = alphaSum(W._trailCanvas);
        __view.key = 'v1'; __view.ox -= 50; W._step(1); const a2 = alphaSum(W._trailCanvas);
        W._step(10); return [a1, a2, alphaSum(W._trailCanvas)]; }""")
    assert a1 > 255 * 2000, "40 幀後要有明顯的軌跡"
    assert a2 == 0, "viewKey 一變那一幀軌跡全清、不畫新段"
    assert a3 > 255 * 500
    assert page.evaluate("W.state().viewKey") == "v1"


# ── B2b. 視角每幀都動（3D 追機）：trailsFollowView ─────────────────────────
# 每幀平移 dx px、換 viewKey、跑一幀；回傳每幀軌跡層的 alpha 總和與引擎回報的畫法。
# ⚠ 統計要在同一次 evaluate 裡讀：兩次 evaluate 之間引擎自己的 rAF 會用「沒變的視角」再跑幾幀（那幾幀是累積畫布）。
_PAN_EACH_FRAME = """([n, dx, dy]) => { const out = [];
  for (let i = 0; i < n; i++) { __view.ox += dx; __view.oy += dy; __view.key = 'cam' + Math.random(); W._step(1);
    const st = W.state(), f = st.fields['default'] || {}, d = W._debug();
    out.push({a: alphaSum(W._trailCanvas), t: st.trail, alive: f.n_alive, field_segments: f.trail_segments,
              full: d && d.hist_cnt ? d.hist_cnt.filter((c, k) => d.alive[k] && c === 25).length : 0}); }
  return out; }"""


@pytest.fixture(params=["webgl", "canvas2d"])
def renderer(request, page):
    """重畫拖尾的兩條路：WebGL（優先）與 Canvas 2D（拿不到 WebGL 時）。兩條都要對。"""
    page.evaluate("([g]) => { W._noGL = g; }", [request.param == "canvas2d"])
    if request.param == "webgl" and not page.evaluate(
            "(() => { try { return !!document.createElement('canvas').getContext('webgl'); } catch (e) { return false; } })()"):
        pytest.skip("這個瀏覽器拿不到 WebGL")
    return request.param


def test_follow_view_keeps_the_trails_while_the_view_moves_every_frame(page, renderer):
    """追機：鏡頭每幀都動。預設（2D）每幀清空 ⇒ 什麼都看不到；trailsFollowView 用位置歷史重畫 ⇒ 跟靜止時差不多亮。"""
    page.evaluate("W.setField(uniform(270, 5)); W._step(40)")
    off = page.evaluate(_PAN_EACH_FRAME, [10, -3, 1])
    assert all(f["a"] == 0 and f["t"]["mode"] == "cleared" for f in off), "前提：預設行為每幀清空（追機只剩點的原因）"
    page.evaluate("W.setOptions({trailsFollowView: true}); W._step(40)")
    still = page.evaluate("alphaSum(W._trailCanvas)")
    assert page.evaluate("W.state().trail.mode") == "accumulate"
    on = page.evaluate(_PAN_EACH_FRAME, [30, -3, 1])
    assert all(f["t"]["mode"] == "follow" and f["t"]["renderer"] == renderer for f in on)
    assert min(f["a"] for f in on) > 0.5 * still, (still, [f["a"] for f in on])
    last = on[-1]
    assert last["alive"] > 300 and last["t"]["segments"] == last["field_segments"], last
    assert last["t"]["segments"] > 4 * last["alive"], f"每顆粒子的拖尾要有多段（歷史 25 點、每 4 點一段 ⇒ 最多 6 段）：{last}"
    assert all(f["t"]["jumps"] == 0 for f in on), "平移不改縮放：不能丟歷史"
    assert last["full"] > 0.6 * last["alive"], f"歷史沒有在每幀被清掉：{last}"


def test_follow_view_trails_are_drawn_where_the_history_projects_in_the_new_view(page, renderer):
    """軌跡錨在世界座標：一跳平移 (−37, +23) px 之後，每顆粒子的歷史用「測試自己的投影公式＋新視角」算出的位置上要有線；
    舊畫面原地（沒跟著場景）的位置就只是偶爾碰到別的粒子。"""
    page.evaluate("W.setOptions({trailsFollowView: true}); W.setField(uniform(270, 5)); W._step(40)")
    r = page.evaluate("""() => { __view.ox -= 37; __view.oy += 23; __view.key = 'jump'; W._step(1);
      const d = W._debug(), cv = W._trailCanvas, hit = [0, 0, 0], stale = [0, 0, 0]; let n = 0;
      for (let k = 0; k < d.n && n < 150; k++) {
        if (!d.alive[k] || d.hist_cnt[k] !== 25) continue;
        const h = W._history('default', k), ok = [0, 10, 20].map(a => {
          const p = screenOf(h[a][0], h[a][1]), q = screenOf(h[a + 2][0], h[a + 2][1]);
          return [(p.x + q.x) / 2, (p.y + q.y) / 2]; });
        if (ok.some(m => m[0] < 3 || m[0] > 797 || m[1] < 3 || m[1] > 597 || m[0] + 37 > 797 || m[1] - 23 < 3)) continue;
        n++;
        ok.forEach((m, i) => { if (alphaAt(cv, m[0], m[1]) > 0) hit[i]++;
                               if (alphaAt(cv, m[0] + 37, m[1] - 23) > 0) stale[i]++; });
      }
      return {n: n, hit: hit, stale: stale, trail: W.state().trail}; }""")
    n = r["n"]
    assert n >= 100, r
    assert r["trail"]["mode"] == "follow" and r["trail"]["jumps"] == 0 and r["trail"]["renderer"] == renderer, r
    assert all(h == n for h in r["hit"]), f"年齡 0–2、10–12、20–22 那段都要畫在歷史投影的位置：{r}"
    assert all(x < 0.5 * n for x in r["stale"]), f"舊畫面原地不該有成排的線（那是沒重投影）：{r}"


def test_follow_view_drops_the_history_when_the_scale_jumps(page, renderer):
    """拉近 2 倍（Leaflet 換級、Cesium 瞬間換視角）：舊歷史在新視角裡會被拉長 2 倍 ⇒ 那顆粒子只留現在這一步。"""
    page.evaluate("W.setOptions({trailsFollowView: true}); W.setField(uniform(270, 5)); W._step(40)")
    pan = page.evaluate(_PAN_EACH_FRAME, [1, -5, 0])[0]
    assert pan["t"]["jumps"] == 0
    r = page.evaluate("""() => { __view.s = 2; __view.ox -= 800; __view.oy += 600; __view.key = 'zoom'; W._step(1);
      const d = W._debug(); let alive = 0, longest = 0;
      for (let k = 0; k < d.n; k++) if (d.alive[k]) { alive++; longest = Math.max(longest, d.hist_cnt[k]); }
      return {alive: alive, longest: longest, trail: W.state().trail}; }""")
    assert r["trail"]["mode"] == "follow" and r["trail"]["jumps"] > 50, r
    assert r["longest"] <= 2, f"縮放一跳之後不能還留著舊歷史：{r}"
    nxt = page.evaluate(_PAN_EACH_FRAME, [5, -2, 0])
    assert all(f["t"]["jumps"] == 0 for f in nxt), "跳完之後的連續小移動照常保留"


def test_follow_view_leaves_a_still_camera_pixel_identical(page):
    """定點鏡頭的外觀不變：同一個亂數種子，選項開或關，60 幀後累積畫布逐像素相同。"""
    out = []
    for follow in (False, True):
        # 一次 evaluate 做完：中間不能插進引擎自己的 rAF 幀（幀數不同畫面就不同）
        out.append(page.evaluate("([f]) => { newWind({trailsFollowView: f}); W.setField(uniform(250, 4)); "
                                 "W.setField('b', uniform(200, 2.5, {height_m: 30, faint: true})); W._step(60); "
                                 "return [pixHash(W._trailCanvas), alphaSum(W._trailCanvas), W.state().trail.mode]; }",
                                 [follow]))
    assert out[0][1] > 255 * 2000, out
    assert out[0][:2] == out[1][:2], out
    assert out[1][2] == "accumulate"


def test_follow_view_option_is_validated_and_has_a_snake_case_alias(page):
    assert page.evaluate("W.options().trailsFollowView") is False, "預設關（2D 頁）"
    assert page.evaluate("W.setOptions({trails_follow_view: true}).trailsFollowView") is True
    assert page.evaluate("W.setOptions({trailsFollowView: 'yes'}).trailsFollowView") is True
    assert any("trailsFollowView" in t for t in _console(page, "warning")), "無效值要警告、保留原值"
    page.evaluate("W.setField(uniform(270, 5)); W._step(30); W.setOptions({trailsFollowView: false}); W._step(1)")
    assert page.evaluate("W._debug().hist_cnt") is None, "關掉後不再保留歷史（不付成本）"


@pytest.mark.parametrize("limit,cls", [(3, "warn"), (6, "caution"), (10, "ok")])
def test_particle_colour_follows_speed_color_of_the_field_speed(page, limit, cls):
    page.evaluate("([L]) => { W.setOptions({limitMps: L}); W.setField(uniform(270, 5)); W._step(30); }", [limit])
    c = page.evaluate("trailClasses()")
    assert c[cls] > 1000 and sum(c.values()) - c[cls] < 0.02 * c[cls], c


def test_particle_count_auto_scales_and_honours_explicit_counts(page):
    small = {"lats": [LATC], "lons": [LONC], "u": [[5]], "v": [[0]],
             "bbox": [LATC - 20 / K, LONC - 20 / KX, LATC + 20 / K, LONC + 20 / KX]}
    page.evaluate("([f]) => { W.setField('s', f); W._step(2); }", [small])
    st = page.evaluate("W.state().fields.s")
    assert st["visible_area_px"] == pytest.approx(1600, rel=1e-6)
    assert st["n_particles"] == 64, "40×40 px ⇒ 每 25 px² 一顆"
    page.evaluate("W.setField('s', uniform(270, 5, {particles: 777})); W._step(2)")
    assert page.evaluate("W.state().fields.s.n_particles") == 777
    page.evaluate("W.setOptions({particles: 123}); W.setField('s', uniform(270, 5)); W._step(2)")
    assert page.evaluate("W.state().fields.s.n_particles") == 123


@pytest.mark.parametrize("null_off", [False, True], ids=["coords-off-screen", "null-off-screen"])
def test_a_field_much_larger_than_the_view_still_fills_the_whole_screen(page, null_off):
    """放大到風場的 1/30：取樣格比畫面還大。轉接器對畫面外回 null（leader 的寫法）也要找得到、邊緣不能空。"""
    page.evaluate("([n]) => { __view.nullOff = n; }", [null_off])
    # 這個 bbox 的 13×13 取樣點沒有一個落在畫面內（49×49 才有 6 個）⇒ null 版本一定走加密搜尋
    bbox = [24.2, 120.119, 25.7, 121.719]
    # 無風 ⇒ 粒子不動，分布 = 重生分布（有風時上風邊緣本來就稀疏，那是流場性質，不是找區塊失敗）
    calm = {"lats": [25.1], "lons": [121.1], "u": [[0]], "v": [[0]], "bbox": bbox}
    strips = """() => { const d = W._debug(), s = {n: 0, left: 0, right: 0, top: 0, bottom: 0, out: 0};
        for (let k = 0; k < d.n; k++) if (d.alive[k]) { const p = screenOf(d.lat[k], d.lon[k]); s.n++;
          if (p.x < 40) s.left++; if (p.x > 760) s.right++; if (p.y < 40) s.top++; if (p.y > 560) s.bottom++;
          if (p.x < -10 || p.x > 810 || p.y < -10 || p.y > 610) s.out++; }
        return s; }"""
    page.evaluate("([f]) => { W.setField(f); W._step(100); }", [calm])
    for _ in range(2):
        res = page.evaluate(strips)
        # 400 顆均勻撒 ⇒ 40 px 寬的邊條期望 ~20／~27 顆
        assert res["n"] == 400 and res["out"] == 0, res
        assert min(res["left"], res["right"], res["top"], res["bottom"]) >= 6, res
        # 平移後（null 版本會先用上一次的可見區當提示，不必每次加密搜尋）也要立刻找得到、再撒滿；
        # 3D 相機連續移動時每幀都會走這裡 ⇒ 鎖住每次重投影的成本（沒有提示時 null 版本約 4300 次）
        cost = page.evaluate("() => { const c = __calls; __view.ox -= 20; "
                             "__view.key = 'nudge' + Math.random(); W._step(1); return __calls - c; }")
        assert 0 < cost < 3000, cost
        page.evaluate("__view.ox -= 210; __view.oy += 170; __view.key = 'panned' + Math.random(); W._step(100)")
    page.evaluate("([f]) => { W.setField(f); W._step(5); }", [{**calm, "u": [[5]]}])
    m = page.evaluate("measure('default', 1)")
    assert m["n"] > 350 and m["mdx"] == pytest.approx(3.0, abs=1e-6) and abs(m["mdy"]) < 1e-6


# ── B3. 箭頭 ────────────────────────────────────────────────────────────
def _geom(page, kind="arrow"):
    return [g for g in page.evaluate("W.state().arrowGeom") if g["kind"] == kind]


def test_arrow_from_the_east_points_west_and_the_pixels_agree(page):
    n = page.evaluate("([a, b]) => W.setArrows([[1, a, b, 50, 90, 2.0, 'UAV1']])", [LATC, LONC])
    assert n == 1
    page.evaluate("W._step(1)")
    g = _geom(page)[0]
    assert g["label"] == "UAV1 2.0 m/s" and g["sysid"] == 1
    assert (g["x"], g["y"]) == pytest.approx((W_PX / 2, H_PX / 2), abs=1e-6)
    assert g["tip"][0] == pytest.approx(W_PX / 2 - 12, abs=1e-6) and g["tail"][0] == pytest.approx(W_PX / 2 + 12, abs=1e-6)
    assert g["tip"][1] == pytest.approx(H_PX / 2, abs=1e-6) and g["length"] == 24
    col = page.evaluate("AeroWind.speedColor(2, 3)")
    assert g["color"] == col == page.evaluate("AeroWind.COLORS.ok")
    # 真的畫出來的像素：箭頭（三角形，|dy|≥3 的那一段）只在西邊
    west = page.evaluate("([c]) => colorStats(W.canvas, 380, 290, 400, 311, c, 40)", [col])
    east = page.evaluate("([c]) => colorStats(W.canvas, 401, 290, 421, 311, c, 40)", [col])
    assert west["far"] >= 6 and east["far"] == 0, (west, east)
    assert west["n"] > east["n"]
    whole = page.evaluate("([c]) => colorStats(W.canvas, 370, 290, 431, 311, c, 40)", [col])
    assert whole["cx"] < W_PX / 2 - 1


@pytest.mark.parametrize("d,sx,sy", [(0, 0, 1), (180, 0, -1), (270, 1, 0), (45, -1, 1)])
def test_arrows_point_downwind(page, d, sx, sy):
    page.evaluate("([a, b, d]) => { W.setArrows('t', [{lat: a, lon: b, h_m: 0, dir_from_deg: d, speed_mps: 3.5}]); W._step(1); }",
                  [LATC, LONC, d])
    g = _geom(page)[0]
    vx, vy = g["tip"][0] - g["tail"][0], g["tip"][1] - g["tail"][1]
    n = math.hypot(sx, sy)
    assert (vx / g["length"], vy / g["length"]) == pytest.approx((sx / n, sy / n), abs=2e-3)
    assert g["label"] == f"3.5 m/s {d}°" and g["color"] == page.evaluate("AeroWind.COLORS.warn")


def test_arrow_length_is_clamped_and_calm_is_a_circle(page):
    rows = [[i, LATC + i * 0.005, LONC, 0, 90, s, f"a{i}"] for i, s in enumerate([0.5, 4.0, 20.0, 0.0])]
    assert page.evaluate("([r]) => W.setArrows(r)", [rows]) == 4
    page.evaluate("W._step(1)")
    lens = {g["label"]: g["length"] for g in _geom(page)}
    assert lens == {"a0 0.5 m/s": 18, "a1 4.0 m/s": 48, "a2 20.0 m/s": 70, "a3 0.0 m/s": 0}


def test_bad_arrow_rows_are_skipped_and_counted(page):
    rows = [[1, LATC, LONC, 0, 90, 2, "ok"], [2, None, LONC, 0, 90, 2, "x"], [3, LATC, LONC, 0, float("nan"), 2, "x"],
            [4, 95, LONC, 0, 90, 2, "x"], [5, LATC, LONC, 0, 90, -1, "x"], "junk", None]
    assert page.evaluate("([r]) => W.setArrows(r)", [rows]) == 1
    assert page.evaluate("W.state().arrows") == 1
    assert any("6 列資料無效" in t for t in _console(page, "warning"))
    assert page.evaluate("W.setArrows([])") == 0 and page.evaluate("W.state().arrows") == 0


def test_arrows_mode_draws_grid_nodes_at_least_60px_apart(page):
    lats = [LAT0 + i * 0.015 for i in range(5)]                     # 150 px 一格
    lons = [LON0 + j * (LON1 - LON0) / 4 for j in range(5)]         # 200 px 一格
    u = [[1.0 + j for j in range(5)] for _ in range(5)]
    v = [[0.5 * i for _ in range(5)] for i in range(5)]
    u[2][2] = None
    page.evaluate("([f]) => { W.setField('g', f); W._step(1); }",
                  [{"lats": lats, "lons": lons, "u": u, "v": v, "mode": "arrows"}])
    g = _geom(page, "field")
    # 內部節點 3×3（邊上的在 10 px 內縮外），扣掉 null 那一個
    assert len(g) == 8
    for a in g:
        i = round((a["lat"] - LAT0) / 0.015)
        j = round((a["lon"] - LON0) / ((LON1 - LON0) / 4))
        assert a["speed"] == pytest.approx(math.hypot(1.0 + j, 0.5 * i), abs=1e-9)
        assert a["color"] == page.evaluate("([s]) => AeroWind.speedColor(s, 3)", [a["speed"]])
    pts = [(a["x"], a["y"]) for a in g]
    assert min(math.dist(p, q) for k, p in enumerate(pts) for q in pts[k + 1:]) >= 60
    assert page.evaluate("W.state().particles") == 0, "arrows 模式不放粒子"


# ── B4. 圖例、靜態、生命週期、強健性 ─────────────────────────────────────
def test_legend_lists_each_layer_and_the_limit(page):
    page.evaluate("([a, b]) => { W.setField(uniform(270, 5, {label: 'GFS 10/16 06Z'}));"
                  " W.setArrows('obs', [[7, a, b, 0, 90, 2, 'UAV7']], {label: '實測'}); W._step(1); }", [LATC, LONC])
    st = page.evaluate("W.state()")
    assert st["legend"] == ["GFS 10/16 06Z　最大 5.0 m/s", "實測　1 點　最大 2.0 m/s", "風速 m/s（上限 3.0）",
                            "0", "3.0 上限", "5.5"]
    assert st["label"] == "GFS 10/16 06Z"
    x0, y0, bw, bh = st["legendBox"]
    assert x0 + bw == pytest.approx(W_PX - 10) and y0 == 10, "預設右上角、邊距 10"
    box_alpha = page.evaluate("""([x, y, w, h]) => { const d = W.canvas.getContext('2d').getImageData(x + 2, y + 2, w - 4, h - 4).data;
        let s = 0, n = 0; for (let i = 3; i < d.length; i += 4) { s += d[i]; n++; } return s / n; }""",
                              [round(x0), round(y0), round(bw), round(bh)])
    assert box_alpha > 150, "圖例底框（深色半透明）要真的畫在可見 canvas 上"
    page.evaluate("W.setOptions({showLegend: false}); W._step(1)")
    assert page.evaluate("W.state().legend") == []


def test_clearing_everything_stops_the_loop_and_blanks_the_canvas(page):
    page.evaluate("W.setField(uniform(270, 5)); W.setArrows([[1, LATC, LONC, 0, 90, 2, 'a']]); W._step(10)")
    assert page.evaluate("alphaSum(W.canvas)") > 0
    assert page.evaluate("W.clearField()") is True
    assert page.evaluate("W.state().animating") is True, "還有箭頭 ⇒ 迴圈繼續"
    assert page.evaluate("W.clearArrows()") is True
    st = page.evaluate("W.state()")
    assert st["animating"] is False and st["field"] is False and st["arrows"] == 0 and st["legend"] == []
    assert page.evaluate("alphaSum(W.canvas)") == 0
    f = st["frames"]
    page.wait_for_timeout(200)
    assert page.evaluate("W.state().frames") == f


def test_animate_false_stops_frames_hides_particles_and_shows_arrows(page):
    page.evaluate("W.setField(uniform(270, 5))")
    page.wait_for_function("W.state().frames > 5", timeout=5000)        # 真的 rAF 迴圈在跑
    assert page.evaluate("W.state().animating") is True
    page.evaluate("W.setOptions({animate: false})")
    page.wait_for_timeout(50)
    f1 = page.evaluate("W.state().frames")
    page.wait_for_timeout(300)
    st = page.evaluate("W.state()")
    assert st["frames"] == f1 and st["animating"] is False and st["particles"] == 0
    assert page.evaluate("alphaSum(W._trailCanvas)") == 0
    g = _geom(page, "field")
    assert len(g) >= 20
    assert all(a["tip"][0] > a["tail"][0] and abs(a["tip"][1] - a["tail"][1]) < 1e-6 for a in g)
    pts = [(a["x"], a["y"]) for a in g]
    assert min(math.dist(p, q) for k, p in enumerate(pts) for q in pts[k + 1:]) >= 60
    # 靜態時 viewKey 一變就重畫；箭頭錨在風場範圍上 ⇒ 平移 37 px 時同一批經緯度正好平移 37 px
    old = {(round(a["lat"], 9), round(a["lon"], 9)): a["x"] for a in g}
    page.evaluate("__view.ox += 37; __view.key = 'static-pan'")
    page.wait_for_function("W.state().viewKey === 'static-pan'", timeout=3000)
    new = {(round(a["lat"], 9), round(a["lon"], 9)): a["x"] for a in _geom(page, "field")}
    common = set(old) & set(new)
    assert len(common) >= 10
    assert all(new[k] - old[k] == pytest.approx(37, abs=1e-6) for k in common)
    assert page.evaluate("W.state().frames") == f1
    page.evaluate("W.setOptions({animate: true})")
    page.wait_for_function(f"W.state().frames > {f1 + 3}", timeout=5000)


def test_destroy_removes_the_canvas_and_stops(page):
    page.evaluate("W.setField(uniform(270, 5)); W._step(2)")
    assert page.evaluate("document.querySelectorAll('#host canvas').length") == 1
    page.evaluate("W.destroy()")
    assert page.evaluate("document.querySelectorAll('#host canvas').length") == 0
    st = page.evaluate("W.state()")
    assert st["destroyed"] is True and st["animating"] is False
    assert page.evaluate("W.setField(uniform(270, 5))") is False
    page.wait_for_timeout(100)
    assert page.evaluate("W.state().frames") == st["frames"]


def test_bad_inputs_are_no_ops_with_a_warning_not_exceptions(page):
    r = page.evaluate("""() => [
        W.setField('e', {lats: [], lons: [], u: [], v: []}),
        W.setField(null),
        W.setField('n', {lats: [25, 26], lons: [121, 122], u: [[null, null], [null, null]], v: [[1, 1], [1, 1]]}),
        W.setField('m', {lats: [25, 24, 26], lons: [121], u: [[1], [1], [1]], v: [[0], [0], [0]]}),
        W.setField('x', {lats: [25], lons: [121], u: [[1]], v: [[0]], mode: 'sideways'})]""")
    assert r == [False, False, False, False, True]
    st = page.evaluate("W._step(3)")
    assert list(st["fields"]) == ["x"] and st["fields"]["x"]["mode"] == "flow"
    o = page.evaluate("W.setOptions({bogus: 1, limitMps: -1, maxAge: 'abc', legendCorner: 'middle'})")
    assert o["limitMps"] == 3.0 and o["maxAge"] == 90 and o["legendCorner"] == "top-right"
    w = _console(page, "warning")
    assert any("bogus" in t for t in w) and any("limitMps" in t for t in w) and any("sideways" in t for t in w)


def test_an_error_inside_the_animation_loop_is_logged_once_and_the_loop_survives(page):
    page.evaluate("W.setField(uniform(270, 5)); W._drawSegs = function () { throw new Error('boom'); }; true")
    page.wait_for_timeout(300)
    errs = _console(page, "error")
    assert len(errs) == 1 and "AeroWind" in errs[0]
    assert page.evaluate("W._raf !== 0")
    page._cons.clear()
    page.evaluate("delete W._drawSegs")
    f = page.evaluate("W.state().frames")
    page.wait_for_function(f"W.state().frames > {f + 3}", timeout=5000)


def test_adapter_that_throws_or_returns_garbage_is_treated_as_invisible(page):
    page.evaluate("""() => { const real = __adapter.project;
        W.adapter = {project: (a, b, h) => { if (a > LATC) throw new Error('x'); if (b > LONC) return {x: NaN, y: 1}; return real(a, b, h); },
                     viewKey: () => { throw new Error('y'); }};
        W.setField(uniform(270, 5)); W._step(30); }""")
    d = page.evaluate("""() => { const d = W._debug(); let bad = 0, n = 0;
        for (let k = 0; k < d.n; k++) if (d.alive[k]) { n++; if (d.lat[k] > LATC || d.lon[k] > LONC + 0.001) bad++; } return {n, bad}; }""")
    assert d["n"] > 50 and d["bad"] == 0


def test_inlined_copy_works_too(browser):
    """原始規格是由 Python 內嵌；整檔塞進 <script> 也要能跑（並且不會重複定義）。"""
    src = WIND_JS.read_text(encoding="utf-8")
    pg = browser.new_page()
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.set_content("<!doctype html><html><head><meta charset='utf-8'></head><body><div id='h' "
                   "style='width:300px;height:200px'></div><script>" + src + "</script><script>" + src +
                   "</script></body></html>")
    r = pg.evaluate("""() => { const w = AeroWind.create(document.getElementById('h'), {project: () => null, viewKey: () => ''});
        const s = w._step(2); const pos = getComputedStyle(document.getElementById('h')).position; w.destroy();
        return [AeroWind.VERSION, s.frames, pos]; }""")
    pg.close()
    assert not errs, errs
    assert r == ["1.0.0", 0, "relative"], "沒有內容時不算幀；static 的 host 會被改成 relative"
