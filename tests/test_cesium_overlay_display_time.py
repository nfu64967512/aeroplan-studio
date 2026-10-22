"""跟著飛機走的 3D 疊圖和模型用**同一個顯示時間**（2026-10-21）—— 真的 Cesium 逐幀量測＋結構鎖。

背景：3D 飛機以「現在 − 每架顯示延遲」（20 Hz 串流時約 0.13 s）做純內插（_uavDisplayTime／_uavSmoothPos／
_uavSmoothHpr，見 tests/test_cesium_uav_batch.py）。但光錐、SAR 熱力圖掃描、RCS 仍用 Python 送來的**最新**遙測
⇒ 18 m/s 時光錐頂點領先模型約 2 m、轉彎時光錐先轉；GPU 填色（Primitive）更只在遙測進來時才移動，而且
GeometryInstance 烘了 ENU、之後又設 primitive.modelMatrix ⇒ 第二次更新起套兩次變換（E9，離飛機 ~76.5 萬 km）。

修法（main.html）：
  * ``_uavShownPose(st)``：模型此刻畫出來的位置／機鼻；``_fovDisplayState(sid)``：光錐用的顯示狀態（視場參數仍取
    updateFOVCone 傳進來的；沒有該架 UAV 才退回最新值）；``_fovApex``：頂點 = 模型位置本身。
  * 光錐邊線／地面框／舊式圓錐的 CallbackProperty 都讀顯示狀態；GPU 填色由 preRender 的 ``_fovFillFollow`` 每幀
    對齊，幾何留在機體座標、只靠 primitive.modelMatrix 擺放（E9 一起修掉）。
  * 熱力圖：消除用顯示狀態的足跡；「足跡沒動不累積」的判別量仍是最新遙測那份的 key（否則累積速率 = 呼叫頻率）。
  * RCS：威脅用模型畫出來的位置／機鼻算。碰撞警告線本來就讀 entity.position（= 顯示時間的內插位置），這裡鎖住。

量測方式（不是讀碼推論）：每一幀 postRender 時比較**這一幀 Cesium 實際拿到的值** ——
  模型 = BillboardVisualizer 這一幀設給飛機圖示的 position；姿態 = 模型 orientation 回呼這一幀回傳的四元數；
  光錐 = 邊線／地面框回呼這一幀回傳的頂點與角點、GPU 填色這一幀的 modelMatrix；碰撞線 = 它的回呼這一幀回傳的兩端。
負對照：同一幀的「最新遙測」離模型要 > 1.5 m（轉彎時方向差 > 2°），證明這個量測分得出舊行為。
"""
from __future__ import annotations

import os
import re

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from tests.test_cesium_uav_batch import (  # noqa: E402,F401  (_pw 是 fixture，匯入即註冊)
    HTML, LAT0, LON0, _QT_OK, _code, _fn, _pw, _strip_comments, _top_statements,
)

if _QT_OK:
    from tests.test_cesium_uav_batch import CESIUM_DIR, _NoDel, _REAL_ARGS, _launch, _template_values


# ═══════════════════════════════════════════════════════════════════════
#  A. 結構鎖（不開瀏覽器）
# ═══════════════════════════════════════════════════════════════════════
def test_shown_pose_is_the_model_display_time_position_and_nose():
    body = _code(_fn('_uavShownPose'))
    assert 'const pos = _uavSmoothPos(st);' in body and '_uavSmoothHpr(st).heading' in body
    assert 'st.lat' in body and 'lastPos' not in body, '經緯度只在 Cartographic 失敗時才退回 st（最新遙測）'


def test_fov_display_state_reads_the_aircraft_and_keeps_the_view_parameters():
    body = _code(_fn('_fovDisplayState'))
    assert 'const st = _UAV_STATE[sid];' in body and 'if (!st || !st.props) return s;' in body, '沒有該架 ⇒ 舊行為'
    assert 'const p = _uavShownPose(st);' in body and 'd.apex = p.pos;' in body
    assert 'd.heading = p.heading;' in body and 'd.alt = Math.max(0.01, p.alt);' in body
    for k in ('radius', 'pitch', 'roll', 'hfov', 'vfov', 'mountAngle'):
        assert f'd.{k} = s.{k};' in body, f'視場參數 {k} 要取 updateFOVCone 傳進來的'
    assert 'if (d._ms !== nowMs || d._st !== st) {' in body, '同一毫秒共用一次姿態；換了一架（clearUAV 後重建）要重算'
    assert 'return s.apex || Cesium.Cartesian3.fromDegrees(s.lon, s.lat, s.alt);' in _code(_fn('_fovApex'))


def test_every_fov_position_callback_reads_the_display_state_not_the_latest_telemetry():
    body = _strip_comments(_fn('updateFOVCone'))
    cbs = re.findall(r'new Cesium\.CallbackProperty\(function \(\) \{(.*?)\}, false\)', body, flags=re.S)
    assert len(cbs) == 9, f'4 條邊（一個回呼）＋地面框＋圓錐位置／方向／長度／底半徑＋地面圈位置／長短軸 = 9：{len(cbs)}'
    pos_cbs = [c for c in cbs if 's.lat' in c or 's.lon' in c or 's.alt' in c or 's.heading' in c or 'Corners' in c]
    assert len(pos_cbs) == 6, f'邊線、地面框、圓錐位置／方向／長度、地面圈位置：{pos_cbs}'
    for c in pos_cbs:
        assert 'var s = _fovDisplayState(sid);' in c and '_fovStates[sid]' not in c, c
    assert 'return [_fovApex(s), corners[idx]];' in body
    assert 'Cesium.Cartesian3.fromDegrees(s.lon, s.lat, s.alt)' not in body, '頂點不可再用傳進來的經緯高'
    assert _top_statements(_fn('updateFOVCone'))[-1] == '_requestRender()'


def test_fill_primitive_follows_every_frame_and_is_placed_only_by_its_model_matrix():
    follow = _code(_fn('_fovFillFollow'))
    assert 'for (const k in fovFillPrimitives) {' in follow
    assert 'if (prim && prim._bakeInv && d) prim.modelMatrix = _fovFillMatrix(prim, d);' in follow
    assert '_requestRender' not in follow and 'viewer.clock' not in follow, 'preRender 裡要求算圖 = 永久 60 fps'
    assert re.search(r'^viewer\.scene\.preRender\.addEventListener\(_fovFillFollow\);', HTML, re.M)
    build = _strip_comments(_fn('_buildFrustumPrimitive'))
    inst = build[build.index('new Cesium.GeometryInstance({'):build.index('new Cesium.Primitive({')]
    # 幾何烘在世界座標（不是 scene3DOnly：留在機體座標會在 projectTo2D 丟例外、3D 靜默停住）；
    # 之後只放相對變換 M(現在) × M0⁻¹（E9：直接放 M(現在) ⇒ 套兩次、填色飛到 ~76.5 萬 km）
    assert 'var bake = _computeFrustumModelMatrix(s);' in build and 'modelMatrix: bake,' in inst
    assert 'modelMatrix' not in build[build.index('new Cesium.Primitive({'):build.index('prim._bakeInv')]
    assert 'prim._bakeInv = Cesium.Matrix4.inverse(bake, new Cesium.Matrix4());' in build
    assert 'return Cesium.Matrix4.multiply(_computeFrustumModelMatrix(s), prim._bakeInv, new Cesium.Matrix4());' \
        in _code(_fn('_fovFillMatrix'))
    ens = _code(_fn('_ensureFrustumFillPrimitive'))
    assert 'prim.modelMatrix = _fovFillMatrix(prim, s);' in ens
    assert len(re.findall(r'\.modelMatrix = ', _code(HTML[HTML.index('function _buildFrustumPrimitive('):
                                                         HTML.index('function _pointInConvexQuad(')]))) == 1
    assert 'var s = _fovDisplayState(sysid);' in build and 'var s = _fovDisplayState(sysid);' in ens
    assert 'var origin = _fovApex(s);' in _code(_fn('_computeFrustumModelMatrix'))


def test_heatmap_clears_under_the_drawn_cone_but_counts_sweeps_on_the_latest_footprint():
    body = _code(_fn('updateHeatmap'))
    i_raw = body.index('_computeFrustumGroundCorners(_fovStates[sysid]);')
    i_fd = body.index('const _fd = _fovDisplayState(sysid);')
    assert i_raw < i_fd < body.index('_computeFrustumGroundCorners(_fd);') < body.index('llCorners = _fd._cornersLL;')
    assert '_fs._lastSweptKey === _fs._cornersKey' in body and 'var _fs = _fovStates[sysid];' in body, \
        '判別量是最新遙測那份 —— 顯示狀態每毫秒都在動，用它的話每次呼叫都算新足跡'
    assert '_fd._lastSweptKey' not in body and '_fd._cornersKey' not in body


def test_rcs_uses_the_shown_pose_and_collision_lines_read_the_entity_position():
    rcs = _code(_fn('updateRCSSensitivity'))
    i = rcs.index('const _p = _uavShownPose(_st);')
    assert 'uavLat = _p.lat; uavLon = _p.lon; uavAlt = _p.alt; uavHeading = _p.heading;' in rcs
    assert i < rcs.index('const dLatM = (uavLat - radar.lat) * mPerDegLat;')
    col = _code(_fn('_checkCollisions'))
    assert col.count('a.position.getValue(Cesium.JulianDate.now())') == 2 and \
        col.count('b.position.getValue(Cesium.JulianDate.now())') == 2, '線的兩端／標籤中點讀 entity.position（顯示時間）'
    assert '.lat' not in col and '.lon' not in col and '_UAV_STATE' not in col, '碰撞線不可讀最新遙測的經緯度'


# ═══════════════════════════════════════════════════════════════════════
#  B. 真的 Cesium（本機 assets/cesium；沒有就 skip；safe runner 要設 SAFE_ALLOW_BROWSER=1）
# ═══════════════════════════════════════════════════════════════════════
@pytest.fixture(scope="module")
def page3d(_pw, tmp_path_factory):
    if not _QT_OK:
        pytest.skip('Qt 匯入失敗')
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    with pytest.MonkeyPatch.context() as mp:
        vals = _template_values(mp)
    # GLB 留空：固定翼／VTOL 走 box＋圖示（位置／姿態和 GLB 模型同一個 entity.position／orientation）
    vals.update({'__PLANE_MODEL_URI__': '', '__COPTER_MODEL_URI__': '', '__VTOL_MODEL_URI__': '',
                 '__INIT_LAT__': str(LAT0), '__INIT_LON__': str(LON0)})
    f = tmp_path_factory.mktemp("ovdisp") / "main.html"
    f.write_text(_NoDel.__new__(_NoDel)._fill_cesium_template(HTML, vals), encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
        pg = b.new_page(viewport={"width": 640, "height": 480})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())
        pg.goto(f.as_uri())
        pg.wait_for_function("typeof viewer !== 'undefined' && typeof updateFOVCone === 'function'", timeout=60000)
        pg.add_script_tag(content=_OV_HELPERS)
    except Exception as e:                                   # pragma: no cover - 沒有 WebGL 等
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._errs = errs
    yield pg
    b.close()


@pytest.fixture
def page(page3d):
    yield page3d
    st = page3d.evaluate("({errs: OV_.renderErrs.slice(), loop: viewer.useDefaultRenderLoop})")
    page3d.evaluate("OV_.cleanup()")
    assert not page3d._errs, page3d._errs
    assert st == {'errs': [], 'loop': True}, f'算圖出錯 ⇒ Cesium 關掉算圖迴圈（3D 靜默停住）：{st}'


_OV_HELPERS = f"""
window.OV_ = {{
  sleep: (ms) => new Promise(r => setTimeout(r, ms)),
  deg: (rad) => ((Cesium.Math.toDegrees(rad) % 360) + 360) % 360,
  angDiff: (a, b) => Math.abs((((a - b) % 360) + 540) % 360 - 180),
  enu0: Cesium.Transforms.eastNorthUpToFixedFrame(Cesium.Cartesian3.fromDegrees({LON0}, {LAT0}, 0)),
  // 真的 ENU 偏移（公尺）→ 經緯度；north(lat, lon) 反過來取北向分量
  ll(e, n) {{
    const w = Cesium.Matrix4.multiplyByPoint(OV_.enu0, new Cesium.Cartesian3(e, n, 0), new Cesium.Cartesian3());
    const c = Cesium.Cartographic.fromCartesian(w);
    return {{lat: Cesium.Math.toDegrees(c.latitude), lon: Cesium.Math.toDegrees(c.longitude)}};
  }},
  north(lat, lon) {{
    const inv = Cesium.Matrix4.inverseTransformation(OV_.enu0, new Cesium.Matrix4());
    return Cesium.Matrix4.multiplyByPoint(inv, Cesium.Cartesian3.fromDegrees(lon, lat, 0), new Cesium.Cartesian3()).y;
  }},
  latest: {{}},
  // 這一幀 BillboardVisualizer 設給飛機圖示的位置（和 box／GLB 模型同一個 entity.position、同一次更新）
  shownBillboard(sid) {{
    const vis = viewer.dataSourceDisplay.defaultDataSource._visualizers.find(v => v instanceof Cesium.BillboardVisualizer);
    const it = vis && vis._items.get(uavEntities[sid].id);
    return (it && it.billboard) ? Cesium.Cartesian3.clone(it.billboard.position) : null;
  }},
  hdgOfQuat(pos, q) {{
    const m = Cesium.Matrix4.fromTranslationQuaternionRotationScale(pos, q, new Cesium.Cartesian3(1, 1, 1));
    return OV_.deg(Cesium.Transforms.fixedFrameToHeadingPitchRoll(m).heading);
  }},
  // GPU 填色實際的「機體 → 世界」變換 = primitive.modelMatrix × 烘進頂點的 M0（= _bakeInv 的反矩陣）
  fillWorld(prim) {{
    return Cesium.Matrix4.multiply(prim.modelMatrix, Cesium.Matrix4.inverse(prim._bakeInv, new Cesium.Matrix4()),
                                   new Cesium.Matrix4());
  }},
  // 上面那個 = ENU(apex) × RotZ(−h) × S ⇒ 機體 +y 在 ENU 的方位就是 h
  hdgOfFill(m, apex) {{
    const y = Cesium.Matrix4.getColumn(m, 1, new Cesium.Cartesian4());
    const enu = Cesium.Transforms.eastNorthUpToFixedFrame(apex);
    const e = Cesium.Matrix4.getColumn(enu, 0, new Cesium.Cartesian4()), n = Cesium.Matrix4.getColumn(enu, 1, new Cesium.Cartesian4());
    return OV_.deg(Math.atan2(y.x * e.x + y.y * e.y + y.z * e.z, y.x * n.x + y.y * n.y + y.z * n.z));
  }},
  // 以實時 20 Hz 串流：pose(sid, t秒) → {{e, n, hdg}}；每步像 Python 合幀一樣先批次推位置、再推光錐（最新遙測）
  // o.cont：接續上一次 fly 的軌跡時間（不然第二段會從 t = 0 重來 ⇒ 飛機瞬移回起點、內插拉出假的高速）
  async fly(sids, pose, durMs, o) {{
    o = o || {{}};
    const start = Date.now();
    if (!o.cont || !OV_.t0) OV_.t0 = start;
    while (Date.now() - start < durMs) {{
      OV_.send(sids, pose, (Date.now() - OV_.t0) / 1000, o);
      await OV_.sleep(50);
    }}
  }},
  // 送一筆（樣本時間 = 現在）：先批次推位置，再推光錐／熱力圖（最新遙測）
  send(sids, pose, t, o) {{
    o = o || {{}};
    const now = Date.now(), alt = o.alt || 60, items = [];
    for (const sid of sids) {{
      const p = pose(sid, t), ll = OV_.ll(p.e, p.n);
      OV_.latest[sid] = {{lat: ll.lat, lon: ll.lon, alt: alt, hdg: p.hdg,
                          cart: Cesium.Cartesian3.fromDegrees(ll.lon, ll.lat, alt)}};
      items.push({{lat: ll.lat, lon: ll.lon, alt: alt, hdg: p.hdg, crs: p.hdg, spd: 18, sysid: sid, mode: 'AUTO',
                   armed: true, vt: 'VTOL', pitch: 0, roll: 0, pt: now, at: now}});
    }}
    updateUAVPositions(items);
    for (const sid of sids) {{
      const L = OV_.latest[sid];
      if (o.fov) updateFOVCone(sid, L.lat, L.lon, L.alt, 50, Math.round(L.hdg), 0, 0, 60, 45, o.mount || 0);
      if (o.heat) updateHeatmap(L.lat, L.lon, 50, sid);
      if (o.onStep) o.onStep(sid);
    }}
  }},
  // 包住回呼：記下這一幀 Cesium 實際拿到的值（postRender 時讀完就清空）
  wrap(prop, rec, key, pick) {{
    const orig = prop._callback;
    prop.setCallback(function (t, r) {{ const v = orig(t, r); rec[key] = pick(v); return v; }}, false);
    rec._restore.push(() => prop.setCallback(orig, false));
  }},
  frameListener: null,
  renderErrs: [],
  onFrame(fn) {{
    OV_.frameListener = fn;
    viewer.scene.postRender.addEventListener(fn);
  }},
  cleanup() {{
    if (OV_.frameListener) {{ viewer.scene.postRender.removeEventListener(OV_.frameListener); OV_.frameListener = null; }}
    clearFOVCone(); clearSARHeatmap(); clearRadarDomes(); clearUAV(); OV_.latest = {{}};
  }},
  stats(xs) {{
    const s = xs.slice().sort((a, b) => a - b);
    return {{n: s.length, max: s.length ? s[s.length - 1] : null, med: s.length ? s[s.length >> 1] : null}};
  }},
}};
// 算圖例外 Cesium 自己吞掉、再把 useDefaultRenderLoop 關掉（整個 3D 靜默停住）⇒ 每條測試結束都要查
viewer.scene.renderError.addEventListener((s, e) => OV_.renderErrs.push(String((e && e.stack) || e).slice(0, 400)));
"""

# 光錐逐幀量測：先飛 0.7 s 暖身（建光錐、顯示延遲收斂），再錄 2.0 s
_FOV_PROBE = """async (args) => {
  const sid = args.sid, pose = new Function('sid', 't', args.pose);
  await OV_.fly([sid], pose, 700, {fov: true, mount: 20});
  const rec = {_restore: []}, fr = {edge: [], fill: [], hdg: [], corner: [], lead: [], hdgLead: [], spin: [], bs: []};
  OV_.wrap(uavEntities[sid].orientation, rec, 'ori', v => v && Cesium.Quaternion.clone(v));
  OV_.wrap(fovEdgeEntities[sid][0].polyline.positions, rec, 'edge', v => v && v.length ? Cesium.Cartesian3.clone(v[0]) : null);
  OV_.wrap(fovGroundPolyEntities[sid].polyline.positions, rec, 'loop', v => v && v.length ? v.slice(0, 4).map(c => Cesium.Cartesian3.clone(c)) : null);
  let lastModel = null;
  OV_.onFrame(function () {
    const model = OV_.shownBillboard(sid), prim = fovFillPrimitives[sid], L = OV_.latest[sid];
    if (model && rec.edge && rec.ori && rec.loop && prim && L) {
      const W = OV_.fillWorld(prim), fillT = Cesium.Matrix4.getTranslation(W, new Cesium.Cartesian3());
      // 獨立粗檢（不經 _bakeInv）：Cesium 自己算的填色包圍球要在飛機附近（E9 那種會在 ~76.5 萬 km 外）
      const bsw = prim._boundingSphereWC && prim._boundingSphereWC[0];
      fr.bs.push(bsw ? Cesium.Cartesian3.distance(bsw.center, model) : 1e12);
      const mh = OV_.hdgOfQuat(model, rec.ori), c = Cesium.Cartographic.fromCartesian(model), s = _fovStates[sid];
      // 期望的地面足跡：用「這一幀模型的位置＋機鼻」配同一組視場參數重算
      const exp = _computeFrustumGroundCorners({lat: Cesium.Math.toDegrees(c.latitude), lon: Cesium.Math.toDegrees(c.longitude),
                                                alt: c.height, heading: mh, hfov: s.hfov, vfov: s.vfov, mountAngle: s.mountAngle});
      fr.edge.push(Cesium.Cartesian3.distance(model, rec.edge));
      fr.fill.push(Cesium.Cartesian3.distance(model, fillT));
      fr.hdg.push(OV_.angDiff(OV_.hdgOfFill(W, fillT), mh));
      fr.corner.push(Math.max(...exp.map((p, i) => Cesium.Cartesian3.distance(p, rec.loop[i]))));
      fr.lead.push(Cesium.Cartesian3.distance(model, L.cart));
      fr.hdgLead.push(OV_.angDiff(L.hdg, mh));
      if (lastModel) fr.spin.push(Cesium.Cartesian3.distance(model, lastModel));
      lastModel = model;
    }
    rec.edge = rec.loop = rec.ori = null;
  });
  await OV_.fly([sid], pose, 2000, {fov: true, mount: 20, cont: true});
  rec._restore.forEach(f => f());
  const out = {};
  for (const k in fr) out[k] = OV_.stats(fr[k]);
  out.delay = _UAV_STATE[sid].delay;
  out.alt = _fovStates[sid]._disp.alt; out.hfov = _fovStates[sid]._disp.hfov; out.mount = _fovStates[sid]._disp.mountAngle;
  return out;
}"""


def _check_fov(r: dict, turning: bool) -> None:
    msg = f'逐幀量測：{r}'
    assert r['edge']['n'] >= 30, '至少 30 幀（rAF 沒在跑 ⇒ 這條量不到東西）' + msg
    assert r['spin']['med'] > 0.1, '模型逐幀在動（不然距離為 0 是假的）' + msg
    assert r['edge']['max'] < 0.3, '光錐邊線頂點 vs 同一幀的模型' + msg
    assert r['fill']['max'] < 0.3, 'GPU 填色頂點 vs 同一幀的模型（舊碼 ~76.5 萬 km）' + msg
    assert r['bs']['max'] < 150, 'Cesium 算的填色包圍球中心離模型 < 150 m（60 m 高、前傾 20° 的角錐）' + msg
    assert r['hdg']['max'] < 0.5, '填色方向 vs 同一幀的模型機鼻（度）' + msg
    assert r['corner']['max'] < 0.5, '地面足跡四角 vs 用模型位置／機鼻重算的足跡' + msg
    # 負對照：同一幀「最新遙測」離模型 ≈ 延遲 × 18 m/s ⇒ 舊行為（光錐用最新遙測）一定量得出來
    assert r['lead']['med'] > 1.5 and 0.08 <= r['delay'] <= 0.2, msg
    if turning:
        assert r['hdgLead']['med'] > 2.0, '急轉時最新遙測的方向領先模型 > 2°（30°/s × 延遲）' + msg
    assert r['alt'] == pytest.approx(60, abs=0.01) and r['hfov'] == 60 and r['mount'] == 20, '視場參數取傳進來的'


def test_real_cesium_cone_apex_stays_on_the_model_every_frame_flying_straight_at_18ms(page):
    # 航向 30°、18 m/s 直飛
    r = page.evaluate(_FOV_PROBE, {'sid': 51, 'pose': "return {e: 18 * t * Math.sin(Math.PI / 6), "
                                                      "n: 18 * t * Math.cos(Math.PI / 6), hdg: 30};"})
    _check_fov(r, turning=False)


def test_real_cesium_cone_apex_and_heading_stay_on_the_model_every_frame_in_a_hard_turn(page):
    # 右轉 30°/s、18 m/s（半徑 34.4 m，約 45° 坡度）：β = 方位（自圓心），航向 = β + 90°
    r = page.evaluate(_FOV_PROBE, {'sid': 52, 'pose': "const w = 30, R = 18 / (w * Math.PI / 180), "
                                                      "b = (-90 + w * t) * Math.PI / 180; "
                                                      "return {e: R * Math.sin(b), n: R * Math.cos(b), "
                                                      "hdg: (((-90 + w * t + 90) % 360) + 360) % 360};"})
    _check_fov(r, turning=True)


def test_real_cesium_collision_line_ends_are_on_the_models_every_frame(page):
    r = page.evaluate("""async () => {
      const pose = (sid, t) => ({e: sid === 41 ? -20 : 20, n: 18 * t, hdg: 0});    // 並排 40 m（< 80 m 預警）
      await OV_.fly([41, 42], pose, 700);
      const key = '41-42', w = collisionWarnings[key];
      if (!w) return {missing: Object.keys(collisionWarnings)};
      const rec = {_restore: []}, d = [], lead = [];
      OV_.wrap(w.polyline.positions, rec, 'ends', v => v && v.length === 2 ? v.map(c => Cesium.Cartesian3.clone(c)) : null);
      OV_.onFrame(function () {
        const a = OV_.shownBillboard(41), b = OV_.shownBillboard(42);
        if (a && b && rec.ends) {
          d.push(Math.max(Cesium.Cartesian3.distance(a, rec.ends[0]), Cesium.Cartesian3.distance(b, rec.ends[1])));
          lead.push(Cesium.Cartesian3.distance(a, OV_.latest[41].cart));
        }
        rec.ends = null;
      });
      await OV_.fly([41, 42], pose, 1500, {cont: true});
      rec._restore.forEach(f => f());
      return {d: OV_.stats(d), lead: OV_.stats(lead), show: w.show, text: w.label.text.getValue(Cesium.JulianDate.now())};
    }""")
    assert 'missing' not in r, r
    assert r['d']['n'] >= 20 and r['d']['max'] < 0.3, f'碰撞線兩端 vs 同一幀的兩架模型：{r}'
    assert r['lead']['med'] > 1.5, f'負對照：最新遙測離模型 ≈ 延遲 × 速度：{r}'
    assert r['show'] is True and r['text'] == '⚠ 40.0m'


def test_real_cesium_heatmap_is_cleared_by_the_drawn_cone_not_the_latest_footprint(page):
    r = page.evaluate("""async () => {
      const sid = 61, alt = 20, a = OV_.ll(-1, -10), b = OV_.ll(1, 60);
      initSARHeatmap(a.lat, b.lat, a.lon, b.lon, 140, 2, 50, 0.8);                // 0.5 m 一列、橫向 ±1 m
      const out = {};
      await OV_.fly([sid], (s, t) => ({e: 0, n: 18 * t, hdg: 0}), 2000, {fov: true, heat: true, alt: alt,
        onStep: () => {                                                          // 每次 updateHeatmap 剛跑完（同一個 task）
          const fs = _fovStates[sid];
          if (!fs || !fs._disp || !fs._disp._cornersLL || !fs._cornersLL) return;
          out.dispFront = Math.max(...fs._disp._cornersLL.map(c => OV_.north(c.lat, c.lon)));
          out.rawFront = Math.max(...fs._cornersLL.map(c => OV_.north(c.lat, c.lon)));
          const p = _uavShownPose(_UAV_STATE[sid]);
          out.modelN = OV_.north(p.lat, p.lon); out.latestN = OV_.north(OV_.latest[sid].lat, OV_.latest[sid].lon);
        }});
      const cells = heatmapGrid.cells, swept = cells.filter(c => c.sweepCount > 0);
      out.nSwept = swept.length;
      out.sweptMax = Math.max(...swept.map(c => OV_.north(c.latCenter, c.lonCenter)));
      out.sum = cells.reduce((s, c) => s + c.sweepCount, 0);
      // 同一筆最新遙測再呼叫一次（顯示狀態已經又往前了）：判別量是最新那份 ⇒ 不再累積
      const L = OV_.latest[sid];
      updateHeatmap(L.lat, L.lon, 50, sid);
      out.sumAgain = cells.reduce((s, c) => s + c.sweepCount, 0);
      return out;
    }""")
    msg = f'{r}'
    lead = r['rawFront'] - r['dispFront']
    assert 1.5 < lead < 4.0 and abs(lead - (r['latestN'] - r['modelN'])) < 0.1, '最新足跡領先畫出來的光錐 = 模型的延遲' + msg
    assert r['dispFront'] - r['modelN'] == pytest.approx(20 * 0.41421356, abs=0.05), '前緣 = 模型 + alt·tan(22.5°)' + msg
    assert r['nSwept'] > 50, msg
    assert r['sweptMax'] <= r['dispFront'] + 0.3, '掃到的最北格不超過畫出來的光錐前緣（格心 0.25 m）' + msg
    assert r['sweptMax'] >= r['dispFront'] - 1.5, '一路掃到畫出來的光錐前緣附近（一步 0.9 m）' + msg
    assert r['rawFront'] - r['sweptMax'] > 1.5, '最新足跡前緣那 ~2 m 還沒畫到 ⇒ 不可以先消除' + msg
    assert r['sumAgain'] == r['sum'], '最新遙測沒變 ⇒ 不累積（即使顯示狀態在動）' + msg


def test_real_cesium_rcs_color_follows_the_drawn_pose(page):
    r = page.evaluate("""async () => {
      const sid = 71, alt = 2, pose = (s, t) => ({e: 0, n: 18 * t, hdg: 0});
      await OV_.fly([sid], pose, 1200, {alt: alt});
      // 以下同一個 task（不讓出、不算圖）：先送一筆新的（顯示時間還落後一整個延遲），
      // 雷達放在最新位置東邊 2 m、半徑 10（δ = 3 m）⇒ 2 m 的領先會改變顏色
      OV_.send([sid], pose, (Date.now() - OV_.t0) / 1000, {alt: alt});
      const L = OV_.latest[sid], east = OV_.ll(2, OV_.north(L.lat, L.lon)), st = _UAV_STATE[sid], e = uavEntities[sid];
      addRadarDome(east.lat, east.lon, 0, 10, 'T');
      const col = () => { const c = e.billboard.color.getValue(Cesium.JulianDate.now()); return [c.red, c.green, c.blue]; };
      updateRCSSensitivity(L.lat, L.lon, L.alt, L.hdg, sid);                     // Python 送最新遙測
      const shown = col(), p = _uavShownPose(st);
      delete _UAV_STATE[sid];                                                    // 沒有取樣 ⇒ 用傳進來的（舊行為）
      updateRCSSensitivity(p.lat, p.lon, p.alt, p.heading, sid); const expect = col();
      updateRCSSensitivity(L.lat, L.lon, L.alt, L.hdg, sid); const old = col();
      _UAV_STATE[sid] = st;
      clearRCSSensitivity(sid);
      return {shown, expect, old, lead: Cesium.Cartesian3.distance(_uavSmoothPos(st), L.cart)};
    }""")
    assert r['lead'] > 1.5, r
    assert max(abs(a - b) for a, b in zip(r['shown'], r['expect'])) < 0.01, f'顏色 = 畫出來那個位置／機鼻算的：{r}'
    assert max(abs(a - b) for a, b in zip(r['shown'], r['old'])) > 0.1, f'負對照：用最新遙測算的顏色明顯不同：{r}'


def test_real_cesium_cone_without_an_aircraft_keeps_the_old_latest_value_behaviour(page):
    r = page.evaluate("""() => {
      const ll = OV_.ll(10, 20);
      updateFOVCone(81, ll.lat, ll.lon, 80, 50, 45, 0, 0, 60, 45, 10);           // 只有光錐、沒有 UAV（外部腳本）
      const now = Cesium.JulianDate.now(), want = Cesium.Cartesian3.fromDegrees(ll.lon, ll.lat, 80);
      const apex = fovEdgeEntities[81][0].polyline.positions.getValue(now)[0];
      _fovFillFollow();
      const m = OV_.fillWorld(fovFillPrimitives[81]), t = Cesium.Matrix4.getTranslation(m, new Cesium.Cartesian3());
      return {same: _fovDisplayState(81) === _fovStates[81], apex: Cesium.Cartesian3.distance(apex, want),
              fill: Cesium.Cartesian3.distance(t, want), hdg: OV_.hdgOfFill(m, t)};
    }""")
    assert r['same'] is True and r['apex'] < 1e-6 and r['fill'] < 1e-6 and r['hdg'] == pytest.approx(45, abs=1e-6), r
