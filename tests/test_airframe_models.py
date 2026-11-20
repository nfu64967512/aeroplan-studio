# -*- coding: utf-8 -*-
"""機體專用 3D 模型（2026-10-23）：Hero（MFE 逆行者）、Pioneer（MFE 开拓者）—— 建模程式、成品指紋、3D 圖上「哪幾架畫哪個機體」的接線。

  A. tools/glb_kit.py／build_hero_glb.py／build_pioneer_glb.py 的純函式（只要 numpy）：CAD → glTF 座標轉換是真旋轉、
     機鼻／上／右翼方向、GLB 格式、槳的扭向。
  B. ui/resources/hero.glb、pioneer.glb 指紋（gitignored；沒建就 skip）：翼展／機長／機高對手冊、方向、primitive 數、沒有貼圖／擴充。
  C. 設定檔 model3d.vtol_model／uav_airframes → 範本那一行 JS；執行期 set_uav_airframes 的優先序；不認得的值要警告。
  D. HARVEST 上傳：構型取自計畫本身（UAVPlan.cls），A＝pioneer、B＝hero。
  真 Cesium 裡模型檔與校正配對：tests/test_cesium_uav_batch.py K 節。
"""
from __future__ import annotations

import importlib.util
import json
import os
import pathlib
import re
import struct
import types

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = pathlib.Path(__file__).resolve().parents[1]
HERO_GLB = REPO / "ui" / "resources" / "hero.glb"
PIONEER_GLB = REPO / "ui" / "resources" / "pioneer.glb"


def _load_builder(name):
    spec = importlib.util.spec_from_file_location(name, REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


bh = _load_builder('build_hero_glb')
bp = _load_builder('build_pioneer_glb')


def _read_glb(data: bytes):
    magic, ver, total = struct.unpack_from('<III', data, 0)
    assert magic == 0x46546C67 and ver == 2 and total == len(data)
    jlen, jtype = struct.unpack_from('<II', data, 12)
    assert jtype == 0x4E4F534A
    gltf = json.loads(data[20:20 + jlen])
    blen, btype = struct.unpack_from('<II', data, 20 + jlen)
    assert btype == 0x004E4942
    return gltf, data[28 + jlen:28 + jlen + blen]


def _positions(gltf, binb, prim):
    acc = gltf['accessors'][prim['attributes']['POSITION']]
    view = gltf['bufferViews'][acc['bufferView']]
    return np.frombuffer(binb, np.float32, acc['count'] * 3, view['byteOffset']).reshape(-1, 3)


# ══════════════════════════════════════════════════════════════════
#  A. 建模程式
# ══════════════════════════════════════════════════════════════════
def test_cad_to_gltf_is_a_proper_rotation_with_nose_up_right_where_cesium_expects_them():
    """CAD（x 後、y 右翼、z 上，mm）→ glTF（+X 機鼻、+Y 上、+Z 右翼，m；Barracuda 慣例 ⇒ 頁面校正 0/0/0）。
    行列式 +1 ⇒ 不會鏡像（鏡像的話左右翼對調、而且三角形繞行反過來）。"""
    cg = np.asarray(bh.CG)
    o = bh.to_gltf(cg[None])[0]
    assert np.allclose(o, 0), '原點在重心（Cesium 繞這點轉）'
    d = lambda v: bh.to_gltf((cg + np.asarray(v, float) * 1000.0)[None])[0]   # noqa: E731
    assert np.allclose(d((-1, 0, 0)), (1, 0, 0)), '往前（CAD -x）＝glTF +X 機鼻'
    assert np.allclose(d((0, 0, 1)), (0, 1, 0)), '上＝glTF +Y'
    assert np.allclose(d((0, 1, 0)), (0, 0, 1)), '右翼（CAD +y）＝glTF +Z'
    M = np.c_[d((1, 0, 0)), d((0, 1, 0)), d((0, 0, 1))]
    assert np.linalg.det(M) == pytest.approx(1.0)


def test_cg_is_the_manual_cg_mark():
    """手冊第 5 頁：重心在翼根塑料件下凸起處 ＝ 前緣後 74 mm（弦長 28.6%）。"""
    assert bh.CG[0] == pytest.approx(74.0) and bh.CG[1] == 0.0


def test_naca_and_resampled_profile_start_at_the_trailing_edge_upper_surface_first():
    p = bh.naca4('0009')
    assert (p[:, 1].max() - p[:, 1].min()) == pytest.approx(0.09, abs=2e-3)
    r = bh.resample_profile(p[::-1], 40)                    # 反向給也要排回「後緣 → 上表面 → 前緣 → 下表面」
    assert len(r) == 40 and r[0, 0] == pytest.approx(r[:, 0].max())
    i_le = int(np.argmin(r[:, 0]))
    assert r[1:i_le, 1].mean() > 0 > r[i_le + 1:, 1].mean()


def test_loft_and_primitives_are_closed_and_outward():
    for V, F in (bh.cylinder((0, 0, 0), (0, 0, 10), 2.0), bh.rounded_box((0, 0, 0), (10, 6, 4), 1.0),
                 bh.loft([np.c_[bh.resample_profile(bh.naca4('2412'), 30) * 100, np.full(30, y)][:, [0, 2, 1]]
                          for y in (0.0, 50.0)])):
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        assert np.einsum('ij,ij->i', a, np.cross(b, c)).sum() > 0, '有號體積 > 0 ＝ 法向朝外'


@pytest.mark.parametrize('handed', [1, -1])
def test_both_blades_of_a_prop_are_pitched_the_same_way_and_handed_flips_them(handed):
    """2026-10-23 審查：以前第二片葉的扭角跟著 side 反號 ⇒ 每支槳一片正、一片反，正反槳設定沒有作用。
    逆時針（從 +軸看）轉時葉片往 +切向走；推力朝 +軸 ⇒ 前緣（運動方向那一側）要比較高。"""
    D = 381.0
    V, _ = bh.propeller((0, 0, 0), (0, 0, 1), (1, 0, 0), D, handed=handed)
    slopes = []
    for s in (1, -1):
        sel = (np.sign(V[:, 0]) == s) & (np.abs(np.abs(V[:, 0]) - 0.5 * D / 2) < 0.06 * D / 2)
        slopes.append(np.polyfit(s * V[sel, 1], V[sel, 2], 1)[0])
    assert all(np.sign(k) == handed for k in slopes), slopes
    hub_r = D * 0.06 / 2
    blades = V[:-42]                                  # 槳轂是最後加的圓柱（20 段 × 2 圈 ＋ 2 個端面中心）
    assert np.abs(blades[:, 0]).min() < hub_r, '葉根伸進槳轂（審查：以前葉片離槳轂 7.6 mm 懸空）'


def test_mirror_keeps_the_winding_outward():
    V, F = bh.cylinder((0, 5, 0), (0, 5, 10), 2.0)
    Vm, Fm = bh.mirror_y(V, F)
    a, b, c = Vm[Fm[:, 0]], Vm[Fm[:, 1]], Vm[Fm[:, 2]]
    assert np.einsum('ij,ij->i', a, np.cross(b, c)).sum() > 0


def test_write_glb_is_plain_gltf_one_primitive_per_material_and_deterministic(tmp_path):
    V, F = bh.cylinder((0, 0, 0), (0, 0, 10), 2.0)
    parts = [bh.Part('a', 'epo_white', V, F, False), bh.Part('b', 'black', V + 20, F, False)]   # smooth 要 trimesh
    groups = bh.build_groups(parts)
    d1 = bh.write_glb(groups, tmp_path / 'x.glb', {'k': 1})
    d2 = bh.write_glb(bh.build_groups(parts), tmp_path / 'y.glb', {'k': 1})
    assert d1 == d2, '同樣輸入要逐位元組相同'
    gltf, binb = _read_glb(d1)
    assert len(gltf['meshes']) == 1 and len(gltf['meshes'][0]['primitives']) == 2
    assert [m['name'] for m in gltf['materials']] == ['epo_white', 'black']
    for k in ('textures', 'images', 'extensionsUsed', 'extensionsRequired', 'animations', 'skins'):
        assert k not in gltf
    for prim in gltf['meshes'][0]['primitives']:
        assert set(prim['attributes']) == {'POSITION', 'NORMAL'} and prim['mode'] == 4
        pos = _positions(gltf, binb, prim)
        acc = gltf['accessors'][prim['attributes']['POSITION']]
        assert np.allclose(pos.min(axis=0), acc['min']) and np.allclose(pos.max(axis=0), acc['max'])


def test_sources_are_pinned_by_git_blob_sha():
    assert len(bh.SOURCES) == 11 and all(len(sha) == 40 for _n, sha in bh.SOURCES.values())
    assert bh.git_blob_sha1(b'') == 'e69de29bb2d1d6434b8b29ae775ad8c2e48c5391', 'git 空 blob 的 sha1'


# ══════════════════════════════════════════════════════════════════
#  B. 成品指紋（ui/resources/hero.glb 是 gitignored ⇒ 沒建就 skip）
# ══════════════════════════════════════════════════════════════════
needs_glb = pytest.mark.skipif(not HERO_GLB.exists(), reason='沒有 ui/resources/hero.glb（python -I tools/build_hero_glb.py build）')


@pytest.fixture(scope='module')
def hero():
    return _read_glb(HERO_GLB.read_bytes())


@needs_glb
def test_hero_glb_dimensions_match_the_manual(hero):
    """手冊：翼展 2180、機長 1140（航測機頭）、機高 255（含腳架）。CAD 翼尖到 1092.9 ⇒ 2.186。"""
    gltf, binb = hero
    P = np.vstack([_positions(gltf, binb, p) for p in gltf['meshes'][0]['primitives']])
    lo, hi = P.min(axis=0), P.max(axis=0)
    assert hi[2] - lo[2] == pytest.approx(2.186, abs=0.006), '翼展（glTF Z）'
    assert hi[2] == pytest.approx(-lo[2], abs=1e-3), '左右對稱'
    white = _positions(gltf, binb, gltf['meshes'][0]['primitives'][0])   # EPO 機體（不含槳、整流罩）
    assert white[:, 0].max() == pytest.approx((350.0 + bh.CG[0]) / 1000, abs=0.002), '機頭尖：航測機頭 CAD x=-350'
    assert lo[0] == pytest.approx(-(880.7 - bh.CG[0]) / 1000, abs=0.003), '整流罩尖：手冊俯視圖 x=880.7'
    assert hi[1] - lo[1] == pytest.approx(0.42, abs=0.02), '機高（V 尾尖 → 腳架底）'
    gear_bottom = (lo[1] * 1000 + bh.CG[2])                                  # 回到 CAD z（mm）
    assert 57.9 - gear_bottom == pytest.approx(255.0, abs=4.0), '機身高 255（含腳架）：艙蓋頂 57.9 → 腳架底'


@needs_glb
def test_hero_glb_axes_nose_forward_tail_up_gear_down(hero):
    gltf, binb = hero
    prims = {gltf['materials'][p['material']]['name']: _positions(gltf, binb, p)
             for p in gltf['meshes'][0]['primitives']}
    white = prims['epo_white']
    nose = white[white[:, 0] > white[:, 0].max() - 0.02]
    assert abs(nose[:, 2]).max() < 0.08, '+X 最前端是機頭（在中線上）'
    assert prims['prop_grey'][:, 0].mean() < -0.7, '尾推槳在 -X（機尾）'
    top = white[white[:, 1] > white[:, 1].max() - 0.01]
    assert abs(top[:, 2]).min() > 0.25, '+Y 最高點是兩片 V 尾尖（在兩側）'
    assert prims['black'][:, 1].min() < -0.19, '腳架在 -Y'


@needs_glb
def test_hero_glb_needs_no_merge_cache_and_has_nothing_cesium_or_the_merger_rejects(hero):
    from ui.widgets import glb_merge as gm
    gltf, _ = hero
    assert gm.count_primitives(HERO_GLB) == len(gltf['meshes'][0]['primitives']) < gm.MIN_PRIMITIVES_TO_MERGE
    assert {m['name'] for m in gltf['materials']} == {m[0] for m in bh.MATERIALS}
    for k in ('textures', 'images', 'extensionsUsed', 'extensionsRequired', 'animations', 'skins'):
        assert k not in gltf
    assert gltf['asset']['extras']['frame'].startswith('metres; +X nose, +Y up, +Z right wing')
    assert HERO_GLB.stat().st_size < 10_000_000


needs_pioneer = pytest.mark.skipif(not PIONEER_GLB.exists(),
                                   reason='沒有 ui/resources/pioneer.glb（python -I tools/build_pioneer_glb.py build）')


@pytest.fixture(scope='module')
def pioneer():
    return _read_glb(PIONEER_GLB.read_bytes())


def test_pioneer_builder_uses_the_same_export_convention_and_the_manual_cg():
    """兩個機體共用 glb_kit 的匯出慣例 ⇒ 頁面校正同樣 0/0/0；重心：手冊第 5 頁重心掛環（弦長 28.2%）。"""
    assert bp.CG[0] == pytest.approx(93.3) and bp.CG[1] == 0.0
    assert {m[0] for m in bp.MATERIALS} == {m[0] for m in bh.MATERIALS}
    assert len(bp.SOURCES) == 5 and all(len(sha) == 40 for _n, sha in bp.SOURCES.values())
    assert bp.wing_le_x(800.0) == 0.0 and bp.wing_te_x(300.0) == pytest.approx(330.9)
    assert bp.wing_te_x(1600.0) == pytest.approx(288.8, abs=0.5), '示意圖翼尖後緣 288.6'


@needs_pioneer
def test_pioneer_glb_dimensions_match_the_manual_and_the_mfe_drawing(pioneer):
    """翼展 3200、機身長 1680（機頭蓋尖 -450 → 尾端面 1230）、機高 416（地面 -269 → 背鰭頂 147.4；手冊 410）。"""
    gltf, binb = pioneer
    prims = {gltf['materials'][p['material']]['name']: _positions(gltf, binb, p)
             for p in gltf['meshes'][0]['primitives']}
    P = np.vstack(list(prims.values()))
    lo, hi = P.min(axis=0), P.max(axis=0)
    assert hi[2] - lo[2] == pytest.approx(3.20, abs=0.006), '翼展（glTF Z）'
    assert hi[2] == pytest.approx(-lo[2], abs=1e-3), '左右對稱'
    white = prims['epo_white']
    nose_x, tail_x = white[:, 0].max(), white[np.abs(white[:, 2]) < 0.05][:, 0].min()
    assert nose_x == pytest.approx((450.0 + bp.CG[0]) / 1000, abs=0.003), '機頭蓋尖 x=-450'
    assert nose_x - tail_x == pytest.approx(1.68, abs=0.006), '機身長 1680（不含尾推）'
    assert hi[1] - lo[1] == pytest.approx(0.4165, abs=0.006), '機高：地面 -269 → 背鰭頂 147.4'
    assert lo[1] * 1000 + bp.CG[2] == pytest.approx(-269.5, abs=2.0), '腳（片厚 6 mm）與腹鰭加強片同一條地面線'


@needs_pioneer
def test_pioneer_glb_axes_and_layout(pioneer):
    gltf, binb = pioneer
    prims = {gltf['materials'][p['material']]['name']: _positions(gltf, binb, p)
             for p in gltf['meshes'][0]['primitives']}
    grey = prims['prop_grey']
    assert np.abs(grey[:, 2]).max() < 0.3, '淺灰只有尾推槳（航燈外殼是白的，官方渲染）'
    assert grey[:, 0].mean() < -1.1, '尾推槳在 -X（機尾）'
    white = prims['epo_white']
    for s in (1, -1):                                   # 航燈外殼：在翼尖前緣、不凸出前緣（審查：+89 時凸出 11–15 mm）
        tip = white[(s * white[:, 2] > 1.51) & (s * white[:, 2] < 1.59)]
        assert len(tip) > 6000, '翼尖有航燈外殼（STEP 8 千多面；只有翼的話幾百點）'
        x_cad, y_cad = bp.CG[0] - 1000.0 * tip[:, 0], 1000.0 * np.abs(tip[:, 2])
        for y0 in np.arange(1528.0, 1582.0, 2.0):
            sel = (y_cad >= y0) & (y_cad < y0 + 2.0)
            if sel.any():                               # 和這一格前緣最前的位置比（翼尖前緣每 mm 展長後退到 1.6 mm）
                assert x_cad[sel].min() >= bp.wing_le_x(y0) - 3.0, (s, y0, x_cad[sel].min(), bp.wing_le_x(y0))
    motors = prims['motor']
    lift = motors[np.abs(motors[:, 2]) > 0.4]
    assert np.median(np.abs(lift[:, 2])) == pytest.approx(0.4993, abs=0.005), '升力馬達在機臂 ±499.3 上'
    assert sorted({round(float(x), 2) for x in (-(np.array(bp.LIFT_HUB_X) - bp.CG[0]) / 1000)}) == \
        sorted({round(float(x), 2) for x in (lift[:, 0].min() + 0.0364, lift[:, 0].max() - 0.0364)}), '前後馬達軸心'
    black = prims['black']
    assert black[:, 1].min() * 1000 + bp.CG[2] < -265, '起落架／腹鰭加強片在 -Y（地面）'


@needs_pioneer
def test_pioneer_glb_needs_no_merge_cache(pioneer):
    from ui.widgets import glb_merge as gm
    gltf, _ = pioneer
    assert gm.count_primitives(PIONEER_GLB) == len(gltf['meshes'][0]['primitives']) < gm.MIN_PRIMITIVES_TO_MERGE
    for k in ('textures', 'images', 'extensionsUsed', 'extensionsRequired', 'animations', 'skins'):
        assert k not in gltf
    assert gltf['asset']['extras']['frame'].startswith('metres; +X nose, +Y up, +Z right wing')
    assert PIONEER_GLB.stat().st_size < 10_000_000


# ══════════════════════════════════════════════════════════════════
#  C. 設定 → 範本、執行期指定
# ══════════════════════════════════════════════════════════════════
def test_model3d_is_its_own_section_old_map_keys_migrate_and_unknown_map_keys_only_warn(tmp_path, capsys):
    """2026-10-23 審查：第一版把 vtol_model／uav_airframes 放在 map 底下 ⇒ 舊版 MapSettings(**...) 丟 TypeError、
    map 以後的區塊全部回預設。現在：自己的頂層區塊（舊版本會跳過）；讀到 map 底下的舊鍵就搬過來；map 的未知鍵只警告。"""
    from config.settings import GlobalSettings, Model3DSettings
    assert Model3DSettings() == Model3DSettings(vtol_model='', uav_airframes={}), '程式預設＝通用 VTOL'
    f = tmp_path / 'settings.json'
    f.write_text(json.dumps({'map': {'default_lat': 23.5, 'default_lon': 120.5, 'vtol_model': 'hero',
                                     'uav_airframes': {'2': 'hero'}, 'brand_new_key': 1},
                             'hil': {'instances': [{'port': 'COM9'}]}}), encoding='utf-8')
    g = GlobalSettings(config_file=str(f))
    assert g.load()
    assert (g.model3d.vtol_model, g.model3d.uav_airframes) == ('hero', {'2': 'hero'}), '舊位置的鍵搬到 model3d'
    assert g.map.default_lat == 23.5 and g.hil.instances == [{'port': 'COM9'}], 'map 的未知鍵不再讓後面的區塊歸零'
    assert 'brand_new_key' in g.map_load_warning and '1' not in g.map_load_warning.split('brand_new_key')[1][:3]
    assert g.save()
    saved = json.loads(f.read_text(encoding='utf-8'))
    assert saved['model3d'] == {'vtol_model': 'hero', 'uav_airframes': {'2': 'hero'}}
    assert 'vtol_model' not in saved['map'] and 'uav_airframes' not in saved['map']
    # model3d 區塊本身：未知鍵、型別不對 ⇒ 警告，不中斷
    f.write_text(json.dumps({'model3d': {'vtol_model': 'hero', 'uav_airframes': ['x'], 'zz': 1}}), encoding='utf-8')
    g = GlobalSettings(config_file=str(f))
    assert g.load() and g.model3d.vtol_model == 'hero' and g.model3d.uav_airframes == {}
    assert 'zz' in g.model3d_load_warning and 'uav_airframes' in g.model3d_load_warning


try:
    from ui.widgets import cesium_map_widget as cmw
    from ui.widgets.cesium_map_widget import CesiumMapWidget

    class _NoDel(CesiumMapWidget):
        def __del__(self):
            pass
    _QT_OK = True
except Exception as _e:                                    # pragma: no cover
    _QT_OK, _QT_ERR = False, str(_e)
needs_qt = pytest.mark.skipif(not _QT_OK, reason='Qt 匯入失敗')


@pytest.fixture
def widget(monkeypatch):
    warnings, sent = [], []
    monkeypatch.setattr(cmw.logger, 'warning', lambda msg, *a, **k: warnings.append(str(msg)))
    w = _NoDel.__new__(_NoDel)
    w._js = sent.append
    w._warnings, w._sent = warnings, sent
    return w


def _settings(monkeypatch, **kw):
    import config.settings as cs
    fake = types.SimpleNamespace(map=types.SimpleNamespace(), model3d=types.SimpleNamespace(**kw))
    monkeypatch.setattr(cs, 'get_settings', lambda: fake)


def _parse(js: str):
    """'_HERO_MODEL_URI = "…"; _PIONEER_MODEL_URI = "…"; _UAV_AIRFRAME_DEFAULT = "…"; Object.assign(_UAV_AIRFRAME, {…});'
    → (hero uri, dflt, map)；pioneer uri 放在 _parse.pioneer。"""
    head, rest = js.split('_UAV_AIRFRAME_DEFAULT = ')
    uris = dict(re.findall(r'(_\w+_MODEL_URI) = ("[^"]*");', head))
    assert set(uris) == {'_HERO_MODEL_URI', '_PIONEER_MODEL_URI'}, head
    dflt, rest = rest.split('; Object.assign(_UAV_AIRFRAME, ')
    assert rest.endswith(');')
    _parse.pioneer = json.loads(uris['_PIONEER_MODEL_URI'])
    return json.loads(uris['_HERO_MODEL_URI']), json.loads(dflt), json.loads(rest[:-2])


@needs_qt
def test_settings_become_the_page_defaults_and_bad_values_warn(widget, monkeypatch):
    _settings(monkeypatch, vtol_model='hero', uav_airframes={'1': 'pioneer', '2': 'HERO', '3': 'bogus', 'x': 'hero'})
    uri, dflt, per = _parse(widget._airframe_models_js({'hero': 'file:///h.glb'}))
    assert (uri, dflt, per) == ('file:///h.glb', 'hero', {'1': 'pioneer', '2': 'hero', '3': ''})
    assert any("'bogus'" in w for w in widget._warnings), '不認得的機體要警告（不靜默當成通用）'
    assert any("'x'" in w for w in widget._warnings), '不是 sysid 的鍵要警告'


@needs_qt
def test_no_settings_means_the_old_behaviour(widget, monkeypatch):
    import config.settings as cs
    monkeypatch.setattr(cs, 'get_settings', lambda: (_ for _ in ()).throw(RuntimeError('不讀設定檔')))
    assert _parse(widget._airframe_models_js({'hero': ''})) == ('', '', {})
    assert widget._warnings == []


@needs_qt
def test_hero_requested_without_the_file_warns(widget, monkeypatch):
    _settings(monkeypatch, vtol_model='hero', uav_airframes={})
    assert _parse(widget._airframe_models_js({'hero': ''}))[0] == ''
    assert any('hero.glb' in w for w in widget._warnings)


@needs_qt
def test_bad_uav_airframes_type_warns(widget, monkeypatch):
    _settings(monkeypatch, vtol_model='', uav_airframes=['hero'])
    assert _parse(widget._airframe_models_js({'hero': 'file:///h.glb'}))[2] == {}
    assert any('uav_airframes' in w for w in widget._warnings)


@needs_qt
def test_runtime_hero_on_a_page_without_hero_glb_warns_but_a_bare_widget_does_not(widget, monkeypatch):
    _settings(monkeypatch, vtol_model='', uav_airframes={})
    widget.set_uav_airframes({1: 'hero'})
    assert widget._warnings == [], '頁面還沒建（不知道有沒有 hero.glb）⇒ 不亂報'
    widget._airframe_models_js({'hero': ''})
    widget.set_uav_airframes({1: 'hero'})
    assert any('hero.glb' in w for w in widget._warnings)
    widget._warnings.clear()
    widget.set_uav_airframes({2: 'pioneer'})
    assert any('pioneer.glb' in w for w in widget._warnings), 'Pioneer 一樣'
    widget._warnings.clear()
    widget._airframe_models_js({'hero': 'file:///h.glb', 'pioneer': 'file:///p.glb'})
    widget.set_uav_airframes({3: 'pioneer'}, default='hero')
    assert widget._warnings == [], '兩個都有 ⇒ 不報'


@needs_qt
def test_runtime_assignment_wins_survives_a_page_reload_and_can_be_undone(widget, monkeypatch):
    _settings(monkeypatch, vtol_model='hero', uav_airframes={'1': 'pioneer'})
    assert widget.set_uav_airframes({1: 'hero', '2': 'pioneer', 'zz': 'hero'}) == {1: 'hero', 2: 'pioneer'}
    assert widget._sent[-1] == 'setUavAirframes({"1": "hero", "2": "pioneer"})'
    assert _parse(widget._airframe_models_js({'hero': 'file:///h.glb'}))[2] == {'1': 'hero', '2': 'pioneer'}, '重載後套回去'
    widget.set_uav_airframes({1: None, 2: None}, default='')
    assert widget._sent[-1] == 'setUavAirframes({"1": "pioneer", "2": null}, "")', \
        '取消執行期指定 ⇒ 回到設定檔那一架（1）或預設（2）'
    assert _parse(widget._airframe_models_js({'hero': 'file:///h.glb'}))[1:] == ('', {'1': 'pioneer'})
    widget.set_uav_airframes({}, default='nope')
    assert widget._sent[-1].endswith(', "")') and any("'nope'" in w for w in widget._warnings)


@needs_qt
def test_page_has_the_hero_channel_and_the_dual_map_forwards_it():
    html = (REPO / 'ui' / 'widgets' / 'cesium_templates' / 'main.html').read_text(encoding='utf-8')
    assert html.count('/*__AIRFRAME_MODELS__*/') == 1, '佔位只能出現一次（註解裡寫出來也會被換掉）'
    assert 'function setUavAirframes(map, dflt)' in html and 'function _uavAirframeOf(sysid)' in html
    from ui.widgets.dual_map_widget import DualMapWidget
    calls = []
    fake = types.SimpleNamespace(map_3d=types.SimpleNamespace(
        set_uav_airframes=lambda m, d: calls.append((m, d)) or {'ok': 1}))
    assert DualMapWidget.set_uav_airframes(fake, {3: 'hero'}) == {'ok': 1} and calls == [({3: 'hero'}, None)]


# ══════════════════════════════════════════════════════════════════
#  D. HARVEST 上傳 → 構型 → 機體模型
# ══════════════════════════════════════════════════════════════════
def _hv(classes):
    plans = {f'U{d}': types.SimpleNamespace(cls=c) for d, c in classes.items()}
    return types.SimpleNamespace(last_plan=types.SimpleNamespace(plans=plans),
                                 id_map={d: f'U{d}' for d in classes})


@pytest.fixture(scope='module')
def mwmod():
    try:
        from ui.main_window import MainWindow  # noqa: F401  先載入 QtWebEngine
        import ui.main_window as mw
    except Exception as e:                               # pragma: no cover
        pytest.skip(f'main_window 匯入失敗：{e}')
    return mw


def test_harvest_airframes_come_from_the_uploaded_plan(mwmod):
    hv = _hv({1: 'A', 2: 'B', 3: 'b'})
    assert mwmod.harvest_airframes(hv, [(5, 1), (6, 2), (7, 3), (8, 9)]) == {5: 'pioneer', 6: 'hero', 7: 'hero'}
    assert mwmod.harvest_airframes(types.SimpleNamespace(), [(1, 1)]) == {}, '沒有計畫 ⇒ 不指定'


def test_upload_pushes_the_airframes_to_the_map_and_never_breaks_the_upload(mwmod, monkeypatch):
    calls = []
    win = types.SimpleNamespace(map_widget=types.SimpleNamespace(set_uav_airframes=calls.append))
    mwmod._show_harvest_airframes(win, _hv({1: 'A', 2: 'B'}), [(3, 1), (4, 2)])
    assert calls == [{3: 'pioneer', 4: 'hero'}] and win._hv_airframe_sysids == {3, 4}
    mwmod._show_harvest_airframes(win, _hv({}), [(3, 1)])
    assert len(calls) == 1, '沒有構型、也不是新計畫 ⇒ 不呼叫'

    def boom(_m):
        raise RuntimeError('頁面還沒好')
    warned = []
    monkeypatch.setattr(mwmod.logger, 'warning', lambda m, *a, **k: warned.append(str(m)))
    mwmod._show_harvest_airframes(types.SimpleNamespace(map_widget=types.SimpleNamespace(set_uav_airframes=boom)),
                                  _hv({1: 'B'}), [(3, 1)])
    assert warned and '頁面還沒好' in warned[0]
    mwmod._show_harvest_airframes(types.SimpleNamespace(), _hv({1: 'B'}), [(3, 1)])   # 沒有地圖 ⇒ 什麼都不做


def test_harvest_assignments_are_dropped_for_a_new_plan_and_when_sitl_restarts(mwmod):
    """2026-10-23 審查：HARVEST 指定以前永遠不會清 ⇒ 換一份計畫、重開 SITL 後同一個 sysid 還畫成舊構型。"""
    calls = []
    win = types.SimpleNamespace(map_widget=types.SimpleNamespace(set_uav_airframes=calls.append))
    mwmod._show_harvest_airframes(win, _hv({1: 'A', 2: 'A', 3: 'B'}), [(1, 1), (2, 2), (3, 3)], new_result=True)
    mwmod._show_harvest_airframes(win, _hv({1: 'B'}), [(4, 1)], new_result=False)          # 同一份計畫分批上傳 ⇒ 累加
    assert win._hv_airframe_sysids == {1, 2, 3, 4}
    mwmod._show_harvest_airframes(win, _hv({1: 'B', 2: 'B'}), [(1, 1), (2, 2)], new_result=True)
    assert calls[-1] == {3: None, 4: None, 1: 'hero', 2: 'hero'}, '新計畫沒有的那幾架取消（回到設定檔）'
    assert win._hv_airframe_sysids == {1, 2}
    mwmod.clear_harvest_airframes(win, '停止 SITL')
    assert calls[-1] == {1: None, 2: None} and win._hv_airframe_sysids == set()
    n = len(calls)
    mwmod.clear_harvest_airframes(win, '重新啟動 SITL')
    assert len(calls) == n, '沒有指定過 ⇒ 不呼叫'


def test_both_upload_paths_and_sitl_restart_go_through_the_hooks(mwmod):
    """record_release_schedule（一般 HARVEST 與建圖上傳都經過）真的把構型送到地圖；SITL 啟動／停止會清。"""
    plans = {f'U{d}': types.SimpleNamespace(cls=c, t_takeoff_s=t) for d, (c, t) in {1: ('A', 0.0), 2: ('B', 5.0)}.items()}
    hv = types.SimpleNamespace(last_plan=types.SimpleNamespace(plans=plans), id_map={1: 'U1', 2: 'U2'},
                               last_result=object())
    calls = []
    win = types.SimpleNamespace(map_widget=types.SimpleNamespace(set_uav_airframes=calls.append),
                                _release=None, _hv_release_schedule=None)
    mwmod.record_release_schedule(win, hv, [(3, 1), (4, 2)], mapping=False)
    assert calls == [{3: 'pioneer', 4: 'hero'}]
    import inspect
    assert "clear_harvest_airframes(self, '停止 SITL')" in inspect.getsource(mwmod.MainWindow.on_sitl_stop_local)


def test_cancelling_the_sitl_launch_keeps_the_harvest_airframes_and_a_real_launch_clears_them(mwmod, monkeypatch):
    """審查 2026-10-23：以前在 on_sitl_launch 一開頭就清 ⇒ 對話框按取消（什麼都沒啟動）也把 HARVEST 指定的機體洗掉。"""
    import mission.sitl_launcher as sl
    import ui.dialogs.sitl_launch_dialog as dlg
    calls, connected = [], []

    class Launcher:
        def start_multi(self, **kw):
            return [(0, 'tcp:127.0.0.1:5760', 0)]

    def make_win():
        return types.SimpleNamespace(
            _sitl_launcher=Launcher(), _diamond_plans=None, _dccpp_result=None, flight_params={},
            _hv_airframe_sysids={1}, map_widget=types.SimpleNamespace(set_uav_airframes=calls.append),
            sitl_hud=types.SimpleNamespace(on_sitl_stopped=lambda: None, on_sitl_launched=lambda *a: None),
            statusBar=lambda: types.SimpleNamespace(showMessage=lambda *a: None),
            on_sitl_connect=lambda conn, **kw: connected.append(conn))
    monkeypatch.setattr(sl.SITLLauncher, 'is_available', staticmethod(lambda: True))
    monkeypatch.setattr(mwmod, 'sitl_plan_wind_offer', lambda win: None)
    monkeypatch.setattr(mwmod.QApplication, 'processEvents', staticmethod(lambda *a: None))
    import time as _time
    monkeypatch.setattr(_time, 'sleep', lambda s: None)

    monkeypatch.setattr(dlg.SITLLaunchDialog, 'get_config', staticmethod(lambda **kw: None))
    win = make_win()
    mwmod.MainWindow.on_sitl_launch(win, 'VTOL', 1)
    assert calls == [] and win._hv_airframe_sysids == {1}, '取消 ⇒ 什麼都沒啟動、HARVEST 指定不動'

    cfg = {'enable_fanout': False, 'instances': [], 'auto_firewall': False, 'home_override': None, 'sim_wind': None}
    monkeypatch.setattr(dlg.SITLLaunchDialog, 'get_config', staticmethod(lambda **kw: cfg))
    win = make_win()
    mwmod.MainWindow.on_sitl_launch(win, 'VTOL', 1)
    assert calls == [{1: None}] and connected == ['tcp:127.0.0.1:5760'], '真的啟動 ⇒ 先取消、再連線'
