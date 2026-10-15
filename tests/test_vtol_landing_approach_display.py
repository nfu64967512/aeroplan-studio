"""VTOL 3D：降落進場腿要和起飛垂直段看得出差別（2026-10-08 使用者回報）。

使用者看到 3D 圖上從起降坪往南拉出兩條粗黃線，問「轉換後的直線為什麼還在 UAV2 的航線上、
方向完全不同」。那其實是**降落進場腿**：全隊降同一個起降場（= UAV-2 的坪），最後一條進場腿
從正南接進坪；實機起飛後是往**北**。舊畫法把進場腿和起飛垂直段畫成同樣的黃色粗線、沒有方向，
所以讀起來像「起飛後往南飛的線」。

這裡把 main.html 的 ``drawVTOLSwarmPaths`` 原樣抽出來，放進假的 Cesium（只記下每個 entity 的
名稱 / 座標 / 材質 / 顏色）在無頭瀏覽器裡執行，斷言**畫出來的值**：
  * 進場腿 = 獨立 entity，座標恰為 [進場起點, 落地點上空]，材質是箭頭、顏色是機色（不是黃）
  * 垂直落地段仍是黃色粗線，座標恰為 [落地點上空, 地面]
  * 起飛垂直段不變（黃色粗線）
  * 沒有進場腿的資料（只有 2 點）不會多畫箭頭
不需要 Cesium CDN、不需要 GPU。
"""
from __future__ import annotations

import json
import pathlib

import pytest

MAIN_HTML = pathlib.Path(__file__).resolve().parents[1] / "ui" / "widgets" / "cesium_templates" / "main.html"

# 取出 drawVTOLSwarmPaths 函式本體（括號配對，不靠行號）
_SRC = MAIN_HTML.read_text(encoding="utf-8")


def _function_source(name: str) -> str:
    i = _SRC.index(f"function {name}(")
    j = _SRC.index("{", i)
    depth = 0
    for k in range(j, len(_SRC)):
        c = _SRC[k]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return _SRC[i:k + 1]
    raise AssertionError(f"{name} 的大括號沒有配對")


_STUB = r"""
window.__ents = [];
const _mk = (type) => function (...args) { return { __type: type, args }; };
window.Cesium = {
  Color: { fromCssColorString: (s) => ({ css: s, withAlpha() { return this; } }),
           YELLOW: { css: 'YELLOW' }, WHITE: { css: 'WHITE' }, BLACK: { css: 'BLACK' } },
  Cartesian3: { fromDegreesArrayHeights: (a) => a, fromDegrees: (lon, lat, h) => [lon, lat, h] },
  Cartesian2: function (x, y) { this.x = x; this.y = y; },
  PolylineGlowMaterialProperty: _mk('Glow'),
  PolylineDashMaterialProperty: _mk('Dash'),
  PolylineArrowMaterialProperty: _mk('Arrow'),
  VerticalOrigin: { BOTTOM: 'BOTTOM' }, LabelStyle: { FILL_AND_OUTLINE: 'FILL' },
  Math: { toRadians: (d) => d * Math.PI / 180 },
};
window.viewer = { entities: { add: (o) => { window.__ents.push(o); return o; } },
                  camera: { flyTo() {} } };
window.vtolClearAll = () => {};
window._vtolEntities = [];
window._vtolArrowSvg = () => 'svg';
"""


def _run(uav_list):
    sync_api = pytest.importorskip("playwright.sync_api")
    code = _function_source("drawVTOLSwarmPaths")
    try:
        with sync_api.sync_playwright() as p:
            try:
                b = p.chromium.launch(channel="chrome", headless=True)
            except Exception:
                b = p.chromium.launch(headless=True)
            pg = b.new_page()
            pg.set_content("<html><body></body></html>")
            pg.add_script_tag(content=_STUB)
            pg.add_script_tag(content=code + "\nwindow.drawVTOLSwarmPaths = drawVTOLSwarmPaths;")
            ents = pg.evaluate(
                """(data) => { window.__ents.length = 0; window.drawVTOLSwarmPaths(data);
                    return window.__ents.map(e => ({
                        name: e.name || null,
                        positions: e.polyline ? e.polyline.positions : (e.position || null),
                        width: e.polyline ? e.polyline.width : null,
                        mat: e.polyline && e.polyline.material ? e.polyline.material.__type : null,
                        matArg: e.polyline && e.polyline.material
                                ? JSON.stringify(e.polyline.material.args) : null,
                        label: e.label ? e.label.text : null,
                        point: !!e.point,
                        registered: window._vtolEntities.includes(e) })); }""",
                uav_list)
            b.close()
    except pytest.skip.Exception:
        raise
    except Exception as e:                                    # pragma: no cover - 沒有可用的瀏覽器
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    return ents


# 真實形狀：HARVEST 1A+2B，降落場 H = 使用者選的起降場；進場腿從 H 正南 ~520 m 接入
H = {"lat": 25.071956, "lon": 121.211472}


def _uav(uid, color, pad_lon, appr):
    return {
        "uav_id": uid, "color": color, "home": {"lat": H["lat"], "lon": pad_lon},
        "transition_alt": 40.0,
        "takeoff_segment": [{"lat": H["lat"], "lon": pad_lon, "alt": 0},
                            {"lat": H["lat"], "lon": pad_lon, "alt": 40.0},
                            {"lat": H["lat"], "lon": pad_lon, "alt": 120.0}],
        "cruise_segment": [{"lat": 25.0754, "lon": pad_lon, "alt": 90.0},
                           dict(appr or {"lat": 25.0680, "lon": pad_lon, "alt": 91.0})],
        "landing_segment": ([appr] if appr else []) + [
            {"lat": H["lat"], "lon": H["lon"], "alt": 50.0},
            {"lat": H["lat"], "lon": H["lon"], "alt": 0}],
        "fw_transition_pt": {"lat": H["lat"], "lon": pad_lon, "alt": 120.0},
        "mc_transition_pt": {"lat": H["lat"], "lon": H["lon"], "alt": 50.0},
    }


APPR = {"lat": 25.067339, "lon": 121.211472, "alt": 91.0}
COLOR = "#1c7ed6"


@pytest.fixture(scope="module")
def drawn():
    return _run([_uav(2, COLOR, 121.211472, APPR)])


def _named(ents, suffix):
    return [e for e in ents if e["name"] and e["name"].endswith(suffix)]


def test_approach_leg_is_its_own_arrow_in_the_uav_colour(drawn):
    leg = _named(drawn, "降落進場")
    assert len(leg) == 1, [e["name"] for e in drawn]
    leg = leg[0]
    assert leg["mat"] == "Arrow", f"進場腿材質應為箭頭，實際 {leg['mat']}"
    assert COLOR in leg["matArg"], f"進場腿應為機色 {COLOR}，實際 {leg['matArg']}"
    assert "YELLOW" not in leg["matArg"], "進場腿不能再是起飛垂直段那種黃線"
    # 座標：恰為 [進場起點, 落地點上空]（lon, lat, alt）；箭頭頭端 = 落地點，方向才對
    assert leg["positions"] == [APPR["lon"], APPR["lat"], APPR["alt"], H["lon"], H["lat"], 50.0]


def test_vertical_landing_leg_stays_yellow_and_excludes_the_approach(drawn):
    leg = _named(drawn, "降落段")
    assert len(leg) == 1
    leg = leg[0]
    assert leg["mat"] == "Glow" and "YELLOW" in leg["matArg"]
    assert leg["positions"] == [H["lon"], H["lat"], 50.0, H["lon"], H["lat"], 0.0], \
        "垂直落地段只能是 [落地點上空, 地面]，不可再包含進場腿"


def test_takeoff_pillar_is_unchanged(drawn):
    tk = _named(drawn, "起飛段")
    assert len(tk) == 1
    assert tk[0]["mat"] == "Glow" and "YELLOW" in tk[0]["matArg"]
    assert tk[0]["positions"] == [121.211472, H["lat"], 0, 121.211472, H["lat"], 40.0,
                                  121.211472, H["lat"], 120.0]


def test_approach_start_is_labelled_with_the_uav(drawn):
    lab = _named(drawn, "降落進場起點")
    assert len(lab) == 1
    assert lab[0]["label"] == "VTOL-2 降落進場" and lab[0]["point"]
    assert lab[0]["positions"] == [APPR["lon"], APPR["lat"], APPR["alt"]]


def test_every_drawn_entity_is_registered_so_clearing_removes_it(drawn):
    """vtolClearAll 只清 _vtolEntities 裡的東西：漏登記 ⇒ 重新規劃後舊的進場線／標籤殘留在 3D 圖上。"""
    assert drawn, "前提：有畫東西"
    missing = [e["name"] or e["label"] for e in drawn if not e["registered"]]
    assert not missing, f"這些 entity 沒登記進 _vtolEntities，清除時會殘留：{missing}"


def test_approach_and_takeoff_are_visually_distinguishable(drawn):
    """使用者的原始問題：兩者要讀得出差別（材質或顏色至少一項不同，且進場腿帶方向）。"""
    tk = _named(drawn, "起飛段")[0]
    ap = _named(drawn, "降落進場")[0]
    assert (tk["mat"], tk["matArg"]) != (ap["mat"], ap["matArg"])
    assert ap["mat"] == "Arrow"                       # 箭頭 = 有方向；起飛段是無方向的 Glow


def test_no_approach_leg_means_no_arrow():
    """沒有進場腿的資料（只有 [落地點上空, 地面]）：不畫箭頭、垂直段照畫。"""
    ents = _run([_uav(2, COLOR, 121.211472, None)])
    assert not _named(ents, "降落進場") and not _named(ents, "降落進場起點")
    land = _named(ents, "降落段")
    assert len(land) == 1 and land[0]["positions"] == [H["lon"], H["lat"], 50.0, H["lon"], H["lat"], 0.0]


def test_three_uavs_share_the_pad_but_each_gets_its_own_coloured_arrow():
    """全隊降同一個場（設計如此）⇒ 三條進場腿終點相同，但各自機色、各自標籤。"""
    uavs = [_uav(1, "#ff6b35", 121.210877, dict(APPR, lat=25.066216, alt=101.7)),
            _uav(2, COLOR, 121.211472, APPR),
            _uav(3, "#22b8cf", 121.212067, dict(APPR, lat=25.067318, alt=91.3))]
    ents = _run(uavs)
    arrows = _named(ents, "降落進場")
    assert len(arrows) == 3
    assert {a["name"] for a in arrows} == {"VTOL-1 降落進場", "VTOL-2 降落進場", "VTOL-3 降落進場"}
    assert len({a["matArg"] for a in arrows}) == 3, "三架的進場箭頭應各自機色"
    assert {tuple(a["positions"][3:]) for a in arrows} == {(H["lon"], H["lat"], 50.0)}
    assert len(_named(ents, "降落進場起點")) == 3
