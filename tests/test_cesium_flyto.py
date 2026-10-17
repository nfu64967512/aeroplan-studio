"""main.html 的 flyToPos（場地預設、找平坦位置、雷達都用它）：目標要落在畫面正中。

以前把相機放在目標正上方再俯 50°，目標掉到畫面下緣外、畫面中心跑到目標北方約 1.3 km（2026-10-17 審查抓到）。
用真的本機 Cesium 1.115（assets/cesium，gitignored；沒有就 skip）跑補間後，用 cartesianToCanvasCoordinates 量目標在畫布的位置。
要開瀏覽器：safe runner 要設 SAFE_ALLOW_BROWSER=1。
"""
from __future__ import annotations

import pathlib
import re

import pytest

from tests.test_survey_map_layers import _pw, _launch, _REAL_ARGS, CESIUM_DIR, MAIN_HTML  # noqa: F401

LAT, LON = 23.451785, 120.286324          # 嘉義亞創中心跑道北端


def _fly_to_pos_js() -> str:
    src = MAIN_HTML.read_text(encoding="utf-8")
    m = re.search(r"function flyToPos\(lat, lon, alt, range\) \{.*?\n\}\n", src, flags=re.S)
    assert m, "main.html 裡找不到 flyToPos"
    return m.group(0)


def test_fly_to_pos_source_centres_on_the_target():
    js = _fly_to_pos_js()
    assert "flyToBoundingSphere" in js and "HeadingPitchRange" in js and "lookAtTransform(Cesium.Matrix4.IDENTITY)" in js


@pytest.fixture(scope="module")
def page(_pw, tmp_path_factory):
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    d = tmp_path_factory.mktemp("flyto")
    f = d / "flyto.html"
    f.write_text(f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<script src="{(CESIUM_DIR / 'Cesium.js').as_uri()}"></script>
<style>html,body,#c{{width:100%;height:100%;margin:0;padding:0;overflow:hidden}}</style>
</head><body><div id="c"></div><script>
Cesium.Ion.defaultAccessToken = '';
window.viewer = new Cesium.Viewer('c', {{ terrainProvider: new Cesium.EllipsoidTerrainProvider(),
  baseLayer: false, baseLayerPicker: false, navigationHelpButton: false, homeButton: false,
  sceneModePicker: false, geocoder: false, timeline: false, animation: false, fullscreenButton: false,
  infoBox: false, selectionIndicator: false, skyBox: false, skyAtmosphere: false }});
{_fly_to_pos_js()}
</script></body></html>""", encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
        pg = b.new_page(viewport={"width": 1280, "height": 720})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())
        pg.goto(f.as_uri())
        pg.wait_for_function("window.viewer !== undefined && typeof flyToPos === 'function'", timeout=60000)
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._errs = errs
    yield pg
    b.close()


@pytest.mark.parametrize("range_m", [600.0, 1500.0])
def test_after_fly_to_pos_the_target_is_at_the_canvas_centre(page, range_m):
    r = page.evaluate(f"""async () => {{
      flyToPos({LAT}, {LON}, 0, {range_m});
      await new Promise(res => setTimeout(res, 2500));          // 補間 1.5 s
      for (let i = 0; i < 5; i++) await new Promise(res => requestAnimationFrame(res));
      const tgt = Cesium.Cartesian3.fromDegrees({LON}, {LAT}, 0);
      const p = viewer.scene.cartesianToCanvasCoordinates(tgt);
      const cv = viewer.scene.canvas;
      const dist = Cesium.Cartesian3.distance(viewer.camera.positionWC, tgt);
      return {{x: p ? p.x : null, y: p ? p.y : null, w: cv.clientWidth, h: cv.clientHeight, dist,
               pitch: Cesium.Math.toDegrees(viewer.camera.pitch), heading: Cesium.Math.toDegrees(viewer.camera.heading)}};
    }}""")
    assert r["x"] is not None, "目標投影不到畫布（在相機後方？）"
    assert abs(r["x"] - r["w"] / 2) < r["w"] * 0.05 and abs(r["y"] - r["h"] / 2) < r["h"] * 0.05, \
        f"目標不在畫面中央：{r}"
    assert r["dist"] == pytest.approx(range_m, rel=0.05) and r["pitch"] == pytest.approx(-50.0, abs=2.0)
    assert abs(((r["heading"] + 180) % 360) - 180) < 2.0, "朝北"
    assert not page._errs, page._errs
