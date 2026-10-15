"""2D 地圖規劃後「整片變暗、無法操作」（2026-10-08 使用者回報，「還是會有此問題」）。

成因：`MapWidget._inject_javascript` 注入的樣式裡有一條

    body > div { width:100% !important; height:100% !important; }

它把 body 底下**每一個** div 撐成全螢幕。重繪（`_render_map`，例如規劃後建電子圍籬的
`set_geofence`）時群飛圖例是以 `folium.Element` 直接放進 body 的
（position:fixed、z-index:9999、背景 rgba(30,30,30,.88)）⇒ 變成蓋滿地圖的深色層、
接走所有點擊。右鍵選單（.ctx-menu）與 toast 也掛在 body 下，一出現同樣整片蓋住。

實測（使用者當時那份頁面，1200×900）：圖例框 1200×900 → 拿掉後 135×188；
地圖中心最上層元素：圖例 div → Leaflet canvas。

這裡用與正式流程相同的方式組頁面（folium + `_build_swarm_legend` + `_inject_javascript`），
在真的瀏覽器裡量圖例框的大小。不建 MapWidget（QtWebEngine 在此環境建不起來）。
"""
from __future__ import annotations

import re
import types

import pytest

folium = pytest.importorskip("folium")

try:
    from ui.widgets.map_widget import MapWidget
    _OK = True
except Exception as _e:                              # pragma: no cover - 環境相依
    _OK = False
    _ERR = _e

pytestmark = pytest.mark.skipif(not _OK, reason=f"map_widget 匯入失敗：{globals().get('_ERR', '')}")

SWARM = {
    'drones': [{'drone_id': i} for i in (1, 2, 3)],
    'stats': {'total_distance': 33075.0, 'estimated_time': 18.1 * 60},
}


def _page() -> str:
    """與 MapWidget._build_map / _render_map 相同的組法：圖例以 folium.Element 進 body，再注入互動 JS。"""
    fake = types.SimpleNamespace(_DRONE_COLORS=MapWidget._DRONE_COLORS,
                                 corners=[(25.07, 121.22), (25.06, 121.23), (25.08, 121.24)])
    m = folium.Map(location=[25.072, 121.231], zoom_start=15, prefer_canvas=True)
    m.get_root().html.add_child(folium.Element(MapWidget._build_swarm_legend(fake, SWARM)))
    html = m.get_root().render()
    return MapWidget._inject_javascript(fake, html, m.get_name())


def test_no_unqualified_body_child_rule():
    """注入的樣式不可有「body > 任何 div／*」這種不帶 class/id 的規則（會撐大所有 body 子元素）。"""
    css = "\n".join(re.findall(r"<style[^>]*>(.*?)</style>", _page(), flags=re.S))
    css = re.sub(r"/\*.*?\*/", "", css, flags=re.S)           # 註解裡提到它不算
    bad = []
    for sel_block in re.findall(r"([^{}]+)\{[^{}]*\}", css):
        for sel in sel_block.split(","):
            s = sel.strip()
            if re.fullmatch(r"(html\s+)?body\s*>\s*(div|\*)", s):
                bad.append(s)
    assert not bad, f"注入的樣式含有會撐大所有 body 子元素的規則：{bad}"


def test_legend_stays_a_small_box_and_does_not_cover_the_map():
    """真的瀏覽器量：圖例框不能蓋滿視窗，地圖中心最上層的元素不能是圖例。"""
    sync_api = pytest.importorskip("playwright.sync_api")
    html = _page()
    try:
        with sync_api.sync_playwright() as p:
            try:
                b = p.chromium.launch(channel="chrome", headless=True)
            except Exception:
                b = p.chromium.launch(headless=True)
            pg = b.new_page(viewport={"width": 1200, "height": 900})
            pg.set_content(html, wait_until="domcontentloaded")
            res = pg.evaluate("""() => {
                const W = innerWidth, H = innerHeight;
                const leg = [...document.body.children].find(e => e.textContent.includes('群飛路徑圖例'));
                const r = leg.getBoundingClientRect();
                const top = document.elementFromPoint(W / 2, H / 2);
                return {W, H, w: r.width, h: r.height, centerIsLegend: leg.contains(top)};
            }""")
            b.close()
    except pytest.skip.Exception:
        raise
    except Exception as e:                            # pragma: no cover - 沒有可用的瀏覽器
        pytest.skip(f"無法啟動無頭瀏覽器：{e}")
    assert res["w"] < res["W"] * 0.5 and res["h"] < res["H"] * 0.5, \
        f"圖例框被撐成 {res['w']:.0f}×{res['h']:.0f}（視窗 {res['W']}×{res['H']}）"
    assert not res["centerIsLegend"], "地圖中心最上層是圖例 ⇒ 點擊會被它接走"
