"""3D 風場引擎的快速投影（ui/widgets/wind_layer.py 的 Cesium adapter）—— 值層級的鎖。

2026-10-19 量測：追機視角＋三層分層風 44 FPS（單層 60）；CDP CPU profile 58.6% 在 adapter.project 逐點呼叫
SceneTransforms.wgs84ToWindowCoordinates（鏡頭每幀都動 ⇒ 引擎每幀把每層格網重投影）。改成：3D＋透視鏡頭時，
鏡頭沒變就沿用同一個「投影×視圖」矩陣、每點一次 4×4 乘法。這裡在真的 Cesium 頁面上逐點比對快速路徑與
Cesium 原函式（各種鏡頭姿態、追機的 lookAtTransform、相機後方、地球背面），並確認鏡頭一動矩陣就更新。
（真 Cesium＝本機 assets/cesium；沒有就 skip；safe runner 要設 SAFE_ALLOW_BROWSER=1。）
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from tests.test_survey_map_layers import _pw  # noqa: E402,F401  （module 級 fixture）
from tests.test_cesium_uav_batch import page3d, LAT0, LON0  # noqa: E402,F401

_CMP_JS = """([lat0, lon0, poses]) => {
  const ST = Cesium.SceneTransforms, toWin = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
  const scene = viewer.scene, cam = scene.camera, ad = _windAdapter();
  const out = [];
  let seed = 7; const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
  for (const pose of poses) {
    try { cam.lookAtTransform(Cesium.Matrix4.IDENTITY); } catch (e) {}
    if (pose.lookAt) {
      const tgt = Cesium.Cartesian3.fromDegrees(lon0, lat0, 100);
      const T = Cesium.Transforms.headingPitchRollToFixedFrame(tgt, new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(pose.h), 0, 0));
      cam.lookAtTransform(T, new Cesium.Cartesian3(0, -80, 14));
    } else {
      cam.setView({destination: Cesium.Cartesian3.fromDegrees(lon0 + pose.dlon, lat0 + pose.dlat, pose.alt),
                   orientation: {heading: Cesium.Math.toRadians(pose.h), pitch: Cesium.Math.toRadians(pose.p), roll: 0}});
    }
    scene.render();                                     // 讓 frustum.aspectRatio／viewMatrix 是這一幀的值
    let n = 0, maxErr = 0, nullMismatch = 0, nullBoth = 0;
    for (let k = 0; k < 400; k++) {
      const lat = lat0 + (rnd() - 0.5) * pose.span, lon = lon0 + (rnd() - 0.5) * pose.span, h = [10, 80, 120, 500][k % 4];
      const fast = ad.project(lat, lon, h);
      const p = Cesium.Cartesian3.fromDegrees(lon, lat, h);
      const occ = new Cesium.EllipsoidalOccluder(Cesium.Ellipsoid.WGS84, cam.positionWC);
      const w = occ.isPointVisible(p) ? toWin.call(ST, scene, p) : undefined;
      const ref = (w && isFinite(w.x) && isFinite(w.y)) ? {x: w.x, y: w.y} : null;
      if (!fast && !ref) { nullBoth++; continue; }
      if (!fast || !ref) { nullMismatch++; continue; }
      n++; maxErr = Math.max(maxErr, Math.hypot(fast.x - ref.x, fast.y - ref.y));
    }
    out.push({pose, n, maxErr, nullMismatch, nullBoth});
  }
  try { cam.lookAtTransform(Cesium.Matrix4.IDENTITY); } catch (e) {}
  return out;
}"""


@pytest.fixture
def page(page3d):
    yield page3d
    assert not page3d._errs, page3d._errs


def test_fast_projection_matches_cesium_across_camera_poses_including_chase(page):
    poses = [
        {"dlat": -0.012, "dlon": 0.0, "alt": 900, "h": 0, "p": -35, "span": 0.03},      # 探針的預設俯視
        {"dlat": -0.004, "dlon": 0.002, "alt": 160, "h": 40, "p": -12, "span": 0.02},   # 低空斜視（點在相機後方也有）
        {"dlat": 0.0, "dlon": 0.0, "alt": 3000, "h": 0, "p": -90, "span": 0.08},        # 正上方往下
        {"dlat": -0.2, "dlon": -0.2, "alt": 20000, "h": 45, "p": -20, "span": 0.6},     # 拉遠
        {"lookAt": True, "h": 135, "span": 0.01},                                        # 追機：lookAtTransform 機體座標系
        {"lookAt": True, "h": 290, "span": 0.01},
    ]
    res = page.evaluate(_CMP_JS, [LAT0, LON0, poses])
    for r in res:
        assert r["n"] >= 50, r
        assert r["nullMismatch"] == 0, f"可見性（相機後方／地球背面）要和 Cesium 一致：{r}"
        assert r["maxErr"] < 0.5, f"快速投影與 Cesium 差超過 0.5 px：{r}"
    assert any(r["nullBoth"] > 0 for r in res), "前提：至少有一個姿態測到相機後方的點"


def test_the_cached_matrix_follows_every_camera_move(page):
    r = page.evaluate("""([lat0, lon0]) => {
      const cam = viewer.scene.camera, ad = _windAdapter(), out = [];
      try { cam.lookAtTransform(Cesium.Matrix4.IDENTITY); } catch (e) {}
      for (let i = 0; i < 5; i++) {
        cam.setView({destination: Cesium.Cartesian3.fromDegrees(lon0, lat0 - 0.01 + i * 0.001, 600),
                     orientation: {heading: Cesium.Math.toRadians(i * 10), pitch: Cesium.Math.toRadians(-30), roll: 0}});
        const a = ad.project(lat0, lon0, 100);          // 不先 render：快取一定要看鏡頭本身，不能看幀號
        const ST = Cesium.SceneTransforms, toWin = ST.wgs84ToWindowCoordinates || ST.worldToWindowCoordinates;
        const w = toWin.call(ST, viewer.scene, Cesium.Cartesian3.fromDegrees(lon0, lat0, 100));
        out.push(Math.hypot(a.x - w.x, a.y - w.y));
      }
      return out; }""", [LAT0, LON0])
    assert max(r) < 0.5, r


# 追機姿態下，引擎的可見區格網（_mapLL）對照 adapter 精確投影：格網在齊次座標內插（adapter 回傳 w）之後
# 靠近鏡頭的點也準。2026-10-21 改之前（只內插 x、y）追機 p90 43–56 px、p99 90–130 px；粒子位置每幀隨格網重建跳動，
# 追機時重畫的拖尾會抖。
_LATTICE_JS = """([lat0, lon0, h, hdgDeg]) => {
  const C = Cesium, cam = viewer.camera, ad = _windAdapter();
  const W = AeroWind.create(viewer.container, ad);
  try {
    const uv = AeroWind.dirToUV(250, 3), d = 0.05;
    W.setOptions({seed: 3});
    W.setField('t', {lats: [lat0 - d, lat0 + d], lons: [lon0 - d, lon0 + d], u: [[uv[0], uv[0]], [uv[0], uv[0]]],
                     v: [[uv[1], uv[1]], [uv[1], uv[1]]], height_m: h});
    const tgt = C.Cartesian3.fromDegrees(lon0, lat0, 120);
    cam.lookAtTransform(C.Transforms.headingPitchRollToFixedFrame(tgt, new C.HeadingPitchRoll(C.Math.toRadians(hdgDeg), 0, 0)),
                        new C.Cartesian3(0, -80, 14));
    W._step(3);
    const F = W.fields.get('t'), pt = F.pt, errs = [];
    let withW = !!(F.lat && F.lat.ws);
    for (let k = 0; k < pt.n; k++) {
      if (!pt.alive[k]) continue;
      for (const [dl, dn] of [[0, 0], [2e-4, 0], [0, -3e-4]]) {      // 粒子本身＋附近（可能落在別的格）
        if (!W._mapLL(F, pt.lat[k] + dl, pt.lon[k] + dn)) continue;
        const a = [W._px, W._py], b = ad.project(pt.lat[k] + dl, pt.lon[k] + dn, h);
        if (b) errs.push(Math.hypot(a[0] - b.x, a[1] - b.y));
      }
    }
    errs.sort((x, y) => x - y);
    return {n: errs.length, p50: errs[errs.length >> 1], p99: errs[Math.floor(0.99 * (errs.length - 1))],
            max: errs[errs.length - 1], withW: withW, lattice: F.lat ? F.lat.N : 0};
  } finally { W.destroy(); cam.lookAtTransform(C.Matrix4.IDENTITY); }
}"""


@pytest.mark.parametrize("h,hdg", [(10.0, 45.0), (80.0, 160.0), (120.0, 300.0)])
def test_lattice_is_exact_near_the_chase_camera_by_interpolating_in_homogeneous_coordinates(page, h, hdg):
    r = page.evaluate(_LATTICE_JS, [LAT0, LON0, h, hdg])
    assert r["withW"] is True, f"3D 快速路徑要回傳 w，格網才會在齊次座標內插：{r}"
    assert r["n"] >= 100, r
    assert r["max"] < 0.5, f"格網位置與精確投影差超過 0.5 px：{r}"


def test_non_perspective_or_non_3d_falls_back_to_cesium():
    import inspect
    from ui.widgets import wind_layer as wl
    src = wl._GLUE_CESIUM
    assert "if (!(f instanceof Cesium.PerspectiveFrustum)) return false;" in src
    assert "if (scene.mode === Cesium.SceneMode.SCENE3D)" in src and "toWin.call(ST, scene, p, win)" in src
    assert inspect.getsource(wl.glue_js)
