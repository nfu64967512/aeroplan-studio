# -*- coding: utf-8 -*-
"""熱力圖只能被**真的掃過的光錐足跡**消除（2026-08-26）。

使用者規格:「我要真的 FOV 光錐掃過的地方才有消除熱力圖的效果」。

修正前有兩個違反這條規格的行為（都已在原始碼確認）:

1. **沒有光錐也會消除**。`updateHeatmap` 在 `_fovStates[sysid]` 不存在時
   退回「以 UAV 為圓心、半徑 fovRadius 的圓」。那個圓**畫面上根本不存在**。
   而 `_fovStates` 正好在 `alt <= 0` 時被刪掉（main.html 對 `!(alt > 0)` 直接
   `_removeFOVConeForSysid`）⇒ 飛機還在起降場時,周圍就先被清乾淨了。

2. **沒移動也會累積**。`updateHeatmap` 被**每一種**遙測訊息觸發
   （HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE / VFR_HUD / SERVO_OUTPUT_RAW
   各自 `emit = True`，見 mission/sitl_link.py:640-694），其中只有位置類會
   改變足跡。⇒ COS 的成長速度取決於**飛控送了幾種訊息**，不是掃過多少地面。

⚠️⚠️ **本檔的限制,先講清楚**:這段邏輯是 `main.html` 裡的 JavaScript,
而這台機器上**沒有任何 JS runtime**（node/deno/bun 皆不存在）,QtWebEngine 要
完整 Cesium 才跑得起來。所以下面是**結構層級**守衛,不是值層級 —— 這是本專案
一向不信任的那一類。補償措施:
  * 每條都有**前提自檢**（被檢查的函式必須存在），避免函式改名後守衛靜靜長綠;
  * 每條都用刪除實驗確認會轉紅（scratchpad/bite_heatmap_cone.py）;
  * 斷言挑的是**會改變行為**的字串,不是註解或格式。
參見 [[verifying-the-verifier]]、[[promised-guards-that-do-not-exist]]。
"""

import pathlib
import re

import pytest


def _html() -> str:
    return (pathlib.Path('ui') / 'widgets' / 'cesium_templates'
            / 'main.html').read_text(encoding='utf-8')


def _update_heatmap_body() -> str:
    """只取 updateHeatmap 函式體，**而且把行註解剝掉**。

    ⚠⚠ 剝註解不是潔癖，是必要的：我在程式碼裡寫了一句註解說明
    「上面 if (!useFrustum) return; 已經擋掉」——那行**字面文字**讓正則
    在真正的敘述被刪掉之後依然匹配得到，守衛因此完全失效
    （突變臂 M1 把守衛整行拿掉，測試照樣全綠）。
    斷言要打在**會執行的程式碼**上，不能打在說明文字上。
    """
    html = _html()
    i = html.find('function updateHeatmap(')
    assert i != -1, "找不到 updateHeatmap ⇒ 本檔所有守衛都失效（可能被改名）"
    # 抓到下一個 top-level function 為止
    j = html.find('\nfunction ', i + 1)
    assert j != -1
    body = html[i:j]
    return '\n'.join(re.sub(r'//.*$', '', ln) for ln in body.splitlines())


def test_comment_stripping_actually_works():
    """前提自檢：剝註解真的有效，否則上面幾條又會被說明文字餵假綠燈。"""
    body = _update_heatmap_body()
    assert '⚠' not in body, "註解沒被剝掉 ⇒ 守衛可能匹配到說明文字"


# ══════════════════════════════════════════════════════════════
# 前提自檢：被守衛的東西真的存在
# ══════════════════════════════════════════════════════════════

def test_the_functions_under_guard_exist():
    """⚠ 沒有這條,下面每一條在函式改名後都會靜靜變成永遠通過。"""
    html = _html()
    for fn in ('function updateHeatmap(',
               'function _computeFrustumGroundCorners(',
               'function _pointInConvexQuad('):
        assert fn in html, f"{fn} 不存在 ⇒ 守衛是空的"


# ══════════════════════════════════════════════════════════════
# 規格一：沒有光錐 ⇒ 什麼都不消除
# ══════════════════════════════════════════════════════════════

def test_no_cone_means_no_clearing():
    """`useFrustum` 為假時必須直接 return，不得往下走到消除迴圈。"""
    body = _update_heatmap_body()
    assert re.search(r'if\s*\(\s*!\s*useFrustum\s*\)\s*return\s*;', body), (
        "updateHeatmap 沒有『沒有光錐就 return』的守衛 ⇒ "
        "畫面上不存在的圓還是會清掉熱力圖")


def test_the_circular_fallback_cannot_cover_anything():
    """舊的圓形 fallback 必須不能再把任何格子判成 covered。

    ⚠ 斷言的是「圓形距離判定不再產生 covered」,不是「那段程式碼消失了」——
    保留一個 `covered = false` 的分支是刻意的（萬一上面的守衛被改壞,
    它會安靜地不覆蓋,而不是安靜地全覆蓋）。
    """
    body = _update_heatmap_body()
    assert not re.search(r'covered\s*=\s*\(\s*dist\s*<=\s*R\s*\)', body), (
        "圓形半徑 fallback 仍會把格子判成 covered ⇒ 沒有光錐的地方也會被清")


def test_footprint_corners_are_the_only_coverage_source():
    """覆蓋判定只能來自光錐地面角點（_pointInConvexQuad）。"""
    body = _update_heatmap_body()
    assigns = re.findall(r'covered\s*=\s*([^;\n]+)', body)
    assert assigns, "找不到任何 covered 指派 ⇒ 守衛是空的"
    for a in assigns:
        a = a.strip()
        assert ('_pointInConvexQuad' in a or a == 'false'), (
            f"covered 來自非光錐來源：{a!r}")


# ══════════════════════════════════════════════════════════════
# 規格二：足跡沒移動 ⇒ 不累積
# ══════════════════════════════════════════════════════════════

def test_unmoved_footprint_does_not_accumulate():
    """同一個足跡不得重複累積 COS。

    判別量是 `_cornersKey` —— 它已經是足跡的完整識別
    （lat|lon|alt|heading|mountAngle|hfov|vfov，見 _computeFrustumGroundCorners）。
    """
    body = _update_heatmap_body()
    assert '_lastSweptKey' in body, (
        "沒有『足跡沒動就不累積』的守衛 ⇒ COS 成長速度取決於遙測訊息種類數")
    assert re.search(r'_lastSweptKey\s*===\s*\w+\._cornersKey', body), (
        "有 _lastSweptKey 但沒有拿它和 _cornersKey 比 ⇒ 守衛不會生效")
    assert re.search(r'_lastSweptKey\s*=\s*\w+\._cornersKey', body), (
        "比對後沒有更新 _lastSweptKey ⇒ 第二拍起會全部被擋掉")


def test_the_key_covers_everything_that_moves_the_footprint():
    """⚠ `_cornersKey` 必須包含每一個會改變地面足跡的量。

    漏掉任何一個（例如 heading）,飛機原地轉向時足跡明明轉了,
    卻會被判成「沒移動」而不累積 —— 那就變成漏掃。
    """
    html = _html()
    i = html.find('function _computeFrustumGroundCorners(')
    key_src = html[i:i + 900]
    m = re.search(r"var key = ([^;]+);", key_src)
    assert m, "找不到 _cornersKey 的組成 ⇒ 上一條守衛的判別量無法驗證"
    expr = m.group(1)
    for field in ('lat', 'lon', 'alt', 'heading', 'mountAngle', 'hfov', 'vfov'):
        assert field in expr, (
            f"_cornersKey 沒有包含 {field} ⇒ 只改這個量時足跡會變、"
            f"卻被判成『沒移動』而漏掃")


# ══════════════════════════════════════════════════════════════
# 不可回歸：光錐停用時本來就不會走到這裡
# ══════════════════════════════════════════════════════════════

def test_python_side_only_sweeps_while_the_cone_is_enabled():
    """Python 端必須只有在光錐啟用時才呼叫 update_heatmap。"""
    import ast
    src = pathlib.Path('ui/main_window.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == 'update_heatmap']
    assert calls, "找不到 update_heatmap 呼叫 ⇒ 守衛是空的"
    # 每個呼叫都必須在某個 `if self._fov_cone_enabled:` 之下
    guarded_lines = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.If)
                and isinstance(node.test, ast.Attribute)
                and node.test.attr == '_fov_cone_enabled'):
            for sub in ast.walk(node):
                if hasattr(sub, 'lineno'):
                    guarded_lines.add(sub.lineno)
    for c in calls:
        assert c.lineno in guarded_lines, (
            f"main_window.py:{c.lineno} 的 update_heatmap 不在 "
            f"_fov_cone_enabled 守衛之下 ⇒ 光錐關掉了還在清熱力圖")


# ══════════════════════════════════════════════════════════════
# 規格三：消除的位置必須就是**畫出來**的那個光錐
# ══════════════════════════════════════════════════════════════

def _corner_fn_body() -> str:
    html = _html()
    i = html.find('function _computeFrustumGroundCorners(')
    assert i != -1, "找不到 _computeFrustumGroundCorners ⇒ 守衛失效"
    j = html.find('\nfunction ', i + 1)
    assert j != -1
    body = html[i:j]
    return '\n'.join(re.sub(r'//.*$', '', ln) for ln in body.splitlines())


def test_corners_use_the_same_enu_frame_as_the_drawn_cone():
    """⚠⚠ 消除用的角點必須用**和畫面填充錐同一套** ENU 變換擺放。

    以前用等距長方近似 `offN / 111320.0`。111320 是赤道附近的值,
    在本專案作業緯度(~25N)實測 1/111320 度 = 0.9951 m ⇒ 南北短 0.49%,
    而畫面上的光錐是用 Cesium 真實 ENU 框架擺的 ⇒ 兩者系統性錯開
    (300 m 高實測每角差 1.17 m,且隨高度線性放大)。
    修正後水平差 0.000 m。

    ⇒ 規格是「真的被光錐掃過才消除」,消除的四邊形就必須**是**畫出來的那個。
    """
    body = _corner_fn_body()
    assert 'eastNorthUpToFixedFrame' in body, (
        "角點沒有用 ENU 變換 ⇒ 消除的位置和畫面上的光錐會系統性錯開")
    assert not re.search(r'/\s*111320\.0', body), (
        "仍在用等距長方近似 111320 換算角點 ⇒ 南北方向會短 ~0.5%")


def test_the_approximation_constant_is_not_silently_reintroduced():
    """前提自檢:確認這個函式真的在做角點換算（否則上一條是空的）。"""
    body = _corner_fn_body()
    assert 'cornersLL.push' in body, "找不到 cornersLL 產生點 ⇒ 守衛是空的"
    assert 'Cartographic.fromCartesian' in body, (
        "沒有從 ENU 世界座標反解回經緯度 ⇒ cornersLL 可能又是自己近似算的")
