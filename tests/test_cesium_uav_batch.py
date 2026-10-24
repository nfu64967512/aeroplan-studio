"""3D 地圖多機更新：批次化、以鏈路時間戳取樣的平滑插值、機鼻方向、自適應算圖（2026-10-17）—— 值層級的鎖。

量到／審查抓到的問題：
  1. Python 每架各送一次 ``updateUAVPosition(...)``（6 架 × 25 Hz = 150 次 JS 呼叫／秒），每次都跑 O(N²) 碰撞檢查。
  2. 尾跡 polyline 的 CallbackProperty 每個算圖幀對每架把整條（≤1500 點）``fromDegreesArrayHeights`` 重算一遍。
  3. 位置／姿態每筆換 ConstantProperty ⇒ 以遙測率跳格；使用者看到「機頭沒跟著路徑走、轉彎慢半拍」。
  4. 第一版取樣插值（審查推翻）：樣本在 JS 執行當下 now() 加入 ⇒ 30 Hz 合幀重送 20 Hz 的同一位置變成新樣本
     （停 33 ms 再跳）、Hermite 外推讓模型倒退、兩個批次連著跑出現相隔 ~1 ms 的兩點、任何 primitive 在建時
     clock.canAnimate=false 全部凍住、斷訊 > 1 s 外推到期彈回、SYSTEM_CLOCK 吃掉末端俯衝自己設的 currentTime、
     機鼻 2 m/s 硬切而且連多旋翼也套用。

修法與這裡鎖住的東西：
  A. main.html：``_updateUAVOne``（本體）＋ ``updateUAVPosition``（單架薄包裝）＋ ``updateUAVPositions(arr)``（批次：
     每架一次本體、碰撞檢查整批一次、最後只要求一次算圖）。
  B. 取樣：時間 = Python 鏈路收到 GLOBAL_POSITION_INT／ATTITUDE 的牆鐘 ms（批次 pt／at），不可信就用 Date.now()；
     同一時間戳重送不加；< 10 ms 取代上一筆；全部線性、兩端 HOLD、不外推；顯示時間 = now − 每架顯示延遲
     （延遲 = clamp(2×間隔EMA+0.03, 0.08, 1.0)，每秒最多改 0.2 s）；不靠 viewer.clock。
  C. 時鐘：allowDataSourcesToSuspendAnimation=false；SYSTEM_CLOCK_MULTIPLIER ×1、UNBOUNDED、不碰 currentTime、
     打擊／俯衝動畫播放中不碰。
  D. 機鼻（2026-10-22 使用者：「模型要忠實反映 SITL 的姿態」）：預設 = 飛控 ATTITUDE yaw（模型姿態 = 飛控姿態）；
     明確 setUavNoseFollowsTrack(true)（顯示輔助）才在固定翼／VTOL、非 Q 模式 3→6 m/s 由 hdg 沿最短角差過渡到 crs；
     模型、相機用的姿態四元數、平面圖示、疊圖一致；機型中途改變 ⇒ 模型與它的 GLB 校正一起換；
     Python 端 yaw 0.0（正北）在收過 ATTITUDE 時不退回 VFR_HUD heading。
  E. 尾跡：Cartesian3 快取；點等這一架的顯示時間走到才進尾跡（不會凸出在延遲顯示的模型前面）。
  F. 自適應算圖：有遙測 ⇒ 連續；閒置 ⇒ requestRenderMode＋maximumRenderTimeChange；每個改場景的公開函式的
     **最後一個頂層敘述**要求算圖；rAF／onTick 迴圈每輪要求；自計時特效用 ``_renderPumpFor``。
  G. Python：一個 JS 呼叫、可選 crs／pt／at、``_fill_cesium_template``（純字串置換）、行內刪 entity 的腳本要求算圖。
  H. 真的 Cesium（本機 assets/cesium；沒有就 skip；safe runner 要設 SAFE_ALLOW_BROWSER=1）：上面 B～F 的實際行為。
  I. 模型姿態（2026-10-22）：滾轉繞機鼻、俯仰繞翼展，組合姿態對照獨立的 3-2-1 DCM。
  J. FPV 相機（2026-10-22）：視線／上方向 = 機體姿態 × 雲台俯角，對照 3-2-1 DCM × 雲台（含舊寫法要失敗）。
"""
from __future__ import annotations

import ast
import inspect
import json
import math
import os
import pathlib
import re

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = pathlib.Path(__file__).resolve().parents[1]
# 突變測試用：可指向 main.html 的突變副本（不改 repo 檔）
MAIN_HTML = pathlib.Path(os.environ.get("CESIUM_UAV_BATCH_MAIN_HTML")
                         or REPO / "ui" / "widgets" / "cesium_templates" / "main.html")
WIDGET_PY = REPO / "ui" / "widgets" / "cesium_map_widget.py"
HTML = MAIN_HTML.read_text(encoding="utf-8")

try:
    from ui.widgets import cesium_map_widget as cmw
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from tests.test_survey_map_layers import _pw, _launch, _REAL_ARGS, CESIUM_DIR, LAT0, LON0  # noqa: F401

    class _NoDel(CesiumMapWidget):
        """裸實例（__new__、沒跑 QWidget.__init__）在回收時 __del__ 會摸 Qt 屬性而丟 RuntimeError；測試用這個子類。"""
        def __del__(self):
            pass
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e
    LAT0, LON0 = 25.07, 121.21

    @pytest.fixture(scope="module")
    def _pw():
        pytest.skip(f"Qt 匯入失敗：{_QT_ERR}")

needs_qt = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")


# ═══════════════════════════════════════════════════════════════════════
#  工具：從 main.html 取出函式、去註解、切出「頂層敘述」
# ═══════════════════════════════════════════════════════════════════════
def _skip_string(src: str, i: int) -> int:
    """``src[i]`` 是引號：回傳字串結束後的位置（處理反斜線逸出）。"""
    q, j, n = src[i], i + 1, len(src)
    while j < n and src[j] != q:
        j += 2 if src[j] == '\\' else 1
    return j + 1


def _scan_body(src: str, start: int) -> str:
    """從 ``start``（第一個 ``{``）配對到對應的 ``}``，回傳含兩端大括號的片段（跳過字串與註解）。"""
    i, depth, n = start, 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith('//', i):
            i = src.find('\n', i)
            if i < 0:
                break
            continue
        if src.startswith('/*', i):
            i = src.find('*/', i) + 2
            continue
        if c in '"\'`':
            i = _skip_string(src, i)
            continue
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return src[start:i + 1]
        i += 1
    raise AssertionError('大括號不配對')


def _fn(name: str, *, top_level: bool = True, src: str = HTML) -> str:
    """``function NAME(...) {...}`` 的完整文字。top_level=False 時也找縮排的內層函式（動畫迴圈）。"""
    pat = (r'^function\s+%s\s*\([^)]*\)\s*\{' if top_level else r'function\s+%s\s*\([^)]*\)\s*\{') % re.escape(name)
    m = re.search(pat, src, flags=re.M)
    assert m, f'main.html 找不到 function {name}'
    return src[m.start():m.end() - 1] + _scan_body(src, m.end() - 1)


def _strip_comments(src: str) -> str:
    """去掉 // 與 /* */ 註解（字串原樣保留）。"""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith('//', i):
            j = src.find('\n', i)
            i = n if j < 0 else j
            continue
        if src.startswith('/*', i):
            i = src.find('*/', i) + 2
            continue
        if c in '"\'`':
            j = _skip_string(src, i)
            out.append(src[i:j])
            i = j
            continue
        out.append(c)
        i += 1
    return ''.join(out)


def _code(text: str) -> str:
    """去掉整行註解（區段／整檔掃描用：逐行處理，HTML 文字裡落單的撇號不會讓它失控）。"""
    return '\n'.join(ln for ln in text.splitlines() if not ln.strip().startswith('//'))


def _top_statements(fn_text: str) -> list[str]:
    """函式本體的頂層敘述（去註解；巢狀區塊／函式本體一律收成 ``{}``，所以 setTimeout 回呼裡的東西不算）。"""
    head = re.match(r'function\s+\w+\s*\([^)]*\)\s*\{', fn_text)
    assert head, fn_text[:80]
    code = _strip_comments(fn_text[head.end():fn_text.rindex('}')])
    stmts: list[str] = []
    cur: list[str] = []
    depth = par = 0
    i, n = 0, len(code)

    def flush():
        s = ''.join(cur).strip().rstrip(';').strip()
        if s:
            stmts.append(' '.join(s.split()))
        cur.clear()

    while i < n:
        c = code[i]
        if c in '"\'`':
            j = _skip_string(code, i)
            if depth == 0:
                cur.append(code[i:j])
            i = j
            continue
        if c == '{':
            if depth == 0:
                cur.append('{')
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                cur.append('}')
                if par == 0:
                    flush()
        elif depth == 0:
            cur.append(c)
            if c in '([':
                par += 1
            elif c in ')]':
                par -= 1
            elif c == ';' and par == 0:
                flush()
        i += 1
    flush()
    return stmts


_RENDER_STMT = re.compile(r"^(?:_requestRender\(\)|_surveyRender\(\)|_renderPumpFor\(.+\)|"
                          r"if \(typeof _requestRender === 'function'\) _requestRender\(\))$")


def _requests_render_at_tail(fn_text: str) -> bool:
    """最後一個頂層敘述是要求算圖；或最後是 return、它前一個是要求算圖。"""
    st = _top_statements(fn_text)
    if not st:
        return False
    if st[-1] == 'return' or st[-1].startswith('return '):
        return len(st) >= 2 and bool(_RENDER_STMT.match(st[-2]))
    return bool(_RENDER_STMT.match(st[-1]))


def _top_level_functions() -> list[str]:
    return re.findall(r'^function\s+(\w+)\s*\(', HTML, flags=re.M)


def test_the_extractors_are_discriminating():
    """正對照：本體以 ``}`` 收尾、只含一個函式；頂層敘述把巢狀回呼收成 ``{}``。"""
    body = _fn('hexColor')
    assert body.endswith('}') and body.count('function ') == 1 and '_requestRender' not in body
    names = _top_level_functions()
    assert len(names) > 120 and len(set(names)) == len(names), '頂層函式名不可重複（重複 = 後者靜默蓋掉前者）'
    st = _top_statements(_fn('explodeAt'))
    assert st[-2] == 'setTimeout(() => {}, DUR * 1000 + 200)' and st[-1] == '_renderPumpFor(DUR * 1000 + 300)'
    assert _top_statements(_fn('strikeVizEnd'))[-1] == 'return JSON.stringify({})'


def test_tail_render_check_has_teeth():
    """負對照：把一個尾端呼叫在記憶體裡拿掉／只放在巢狀回呼／後面還有別的敘述／只在註解裡 ⇒ 都要判失敗。"""
    body = _fn('clearCorners')
    assert _requests_render_at_tail(body)
    cut = body[:body.rindex('_requestRender();')] + body[body.rindex('_requestRender();') + len('_requestRender();'):]
    assert not _requests_render_at_tail(cut), '拿掉尾端的 _requestRender() 後還判通過 ⇒ 這個鎖沒有鑑別力'
    for src, ok in [
        ('function f() { a(); setTimeout(() => { _requestRender(); }, 10); }', False),
        ('function f() { _requestRender(); foo(); }', False),
        ('function f() { foo();\n  // _requestRender();\n}', False),
        ('function f() { if (x) { _requestRender(); } }', False),
        ("function f() { foo(); var s = '_requestRender();'; }", False),
        ('function f() { foo(); _requestRender(); return 1; }', True),
        ('function f() { foo(); _requestRender(); return; }', True),
        ("function f() { foo(); if (typeof _requestRender === 'function') _requestRender(); }", True),
        ('function f() { foo(); _renderPumpFor(1000); }', True),
    ]:
        assert _requests_render_at_tail(src) is ok, src


# ═══════════════════════════════════════════════════════════════════════
#  A. 單架本體／薄包裝／批次
# ═══════════════════════════════════════════════════════════════════════
_ARGS = 'lat, lon, alt, headingDeg, speedMs, sysid, mode, armed, vehicleType, pitchDeg, rollDeg'


def test_update_one_wrapper_and_batch_all_exist_with_the_old_signature():
    assert f'function _updateUAVOne({_ARGS}, crsDeg, posT, attT, attSamples)' in HTML, \
        'crs／pt／at／atts 是第 12～15 個可省略的參數'
    assert f'function updateUAVPosition({_ARGS})' in HTML, '單架舊 API 的簽名不可變（外部腳本／docs 的 smoke.js 直接呼叫）'
    assert 'function updateUAVPositions(arr)' in HTML
    assert 'function _requestRender()' in HTML and 'function _renderPumpFor(ms)' in HTML


def test_batch_runs_every_aircraft_then_one_collision_check_then_one_render_request():
    body = _fn('updateUAVPositions')
    assert body.count('_checkCollisions()') == 1, '碰撞檢查整批只做一次'
    assert body.count('_updateUAVOne(') == 1 and 'for (let i = 0; i < arr.length; i++)' in body
    assert ('u.lat, u.lon, u.alt, u.hdg, u.spd, u.sysid, u.mode, u.armed, u.vt, u.pitch, u.roll, u.crs, u.pt, u.at, u.atts'
            in body), '批次鍵名要和 Python UAV_BATCH_KEYS（＋crs／pt／at／atts）／main_window._uav_batch_item 一致'
    assert "typeof arr === 'string'" in body and 'JSON.parse' in body, '也要接受 JSON 字串'
    assert _top_statements(body)[-1] == '_requestRender()', '算圖要求要在最後（所有架都更新完才算一幀）'
    assert body.index('_noteUavActivity();') < body.index('_ensureClockRunning();') < body.index('for (') \
        < body.index('_checkCollisions()') < body.index('_requestRender();')


def test_single_update_is_a_thin_wrapper_with_the_old_behaviour():
    body = _fn('updateUAVPosition')
    assert _top_statements(body) == ['_noteUavActivity()', '_ensureClockRunning()',
                                     f'_updateUAVOne({_ARGS})', '_checkCollisions()', '_requestRender()'], \
        '單架不傳 crs／pt／at ⇒ 機鼻用 hdg、樣本時間用 Date.now()'


def test_update_one_never_checks_collisions_or_requests_render_and_keeps_the_sysid1_status_bar():
    body = _code(_fn('_updateUAVOne'))
    assert '_checkCollisions' not in body, '每架都檢查就回到 O(N²) × N 次／批'
    assert '_requestRender' not in body and '_noteUavActivity' not in body, '留給批次尾端'
    assert 'if (sysid === 1) {' in body and "getElementById('sitlStatus')" in body
    assert 'uavEntities[sysid] = entity;' in body and 'if (sysid === 1) uavEntity = entity;' in body, '舊 API 相容'


def test_collision_check_is_only_called_from_the_two_entry_points():
    callers = [n for n in _top_level_functions() if '_checkCollisions()' in _code(_fn(n)) and n != '_checkCollisions']
    assert sorted(callers) == ['updateUAVPosition', 'updateUAVPositions'], callers


# ═══════════════════════════════════════════════════════════════════════
#  B. 取樣：時間戳、去重、取代、線性＋HOLD、顯示延遲
# ═══════════════════════════════════════════════════════════════════════
def test_sampled_properties_are_linear_hold_and_never_extrapolate():
    props = _code(_fn('_uavSampledProps'))
    assert 'interpolationDegree: 1, interpolationAlgorithm: Cesium.LinearApproximation' in props
    assert 'p.forwardExtrapolationType = Cesium.ExtrapolationType.HOLD;' in props
    assert 'p.backwardExtrapolationType = Cesium.ExtrapolationType.HOLD;' in props
    assert ('pos: lin(new Cesium.SampledPositionProperty())' in props
            and 'ori: lin(new Cesium.SampledProperty(Cesium.Quaternion))' in props
            and 'att: lin(new Cesium.SampledProperty(Cesium.Quaternion))' in props), '三個屬性都套同一個設定'
    uav = _code(HTML[HTML.index('const _UAV_STATE = {};'):HTML.index('function clearUAV(')])
    for bad in ('EXTRAPOLATE', 'HermitePolynomialApproximation', 'forwardExtrapolationDuration', '_UAV_EXTRAP_S'):
        assert bad not in uav, f'即時 UAV 程式碼裡不可以再有 {bad}（外推＝倒退、到期回彈）'


def test_sample_time_comes_from_the_link_timestamp_with_a_sanity_window():
    f = _code(_fn('_uavSampleMs'))
    assert "typeof t === 'number' && isFinite(t) && t >= nowMs - _UAV_TS_PAST_MS && t <= nowMs + _UAV_TS_FUTURE_MS" in f
    assert '? t : nowMs' in f
    assert 'const _UAV_TS_PAST_MS = 5000, _UAV_TS_FUTURE_MS = 1000;' in HTML
    one = _code(_fn('_updateUAVOne'))
    assert 'const pMs = _uavSampleMs(posT, nowMs), aMs = _uavSampleMs(attT, nowMs);' in one


def test_push_sample_skips_resends_and_replaces_samples_closer_than_ten_ms():
    push = _code(_fn('_uavPushSample'))
    assert 'if (!(ms > lastMs)) return null;' in push, '同一個（或更舊的）時間戳 ⇒ 什麼都不加'
    assert 'Cesium.JulianDate.fromDate(new Date(ms))' in push
    assert 'if (lastJD && ms - lastMs < _UAV_MIN_SAMPLE_GAP_MS)' in push and 'props[i].removeSample(lastJD)' in push
    assert push.index('removeSample(lastJD)') < push.index('addSample(jd, values[i])'), '先移除上一筆再加'
    assert 'const _UAV_MIN_SAMPLE_GAP_MS = 10;' in HTML
    one = _code(_fn('_updateUAVOne'))
    assert '_uavPushSample([st.props.pos], [pos], pMs, st.lastPosMs, st.lastPosJD)' in one
    assert '_uavPushSample([st.props.ori, st.props.att], [orientation, attitude], aMs, st.lastAttMs, st.lastAttJD)' in one
    assert 'addSeconds(now, -1.0' not in one and 'addSample(t0' not in one, '不再補 1 s 前的假種子樣本（HOLD 撐住單一樣本）'
    assert 'addSample(' not in one, '樣本只能經 _uavPushSample 加（才有去重／取代）'


def test_older_attitude_samples_go_through_the_same_push_rule_before_the_main_sample():
    """批次的 atts（2026-10-22）：逐筆經 _uavPushSample（去重／< 10 ms 取代）加進同兩個姿態屬性，排在主樣本之前；
    時間不可信或不比主樣本舊的那一筆略過（不退回現在：退回現在會讓主樣本變成「重送」被丟掉）；陣列順序 [ms, roll, pitch, 機頭]。"""
    one = _code(_fn('_updateUAVOne'))
    i_loop = one.index('if (Array.isArray(attSamples)) {')
    i_main = one.index('_uavPushSample([st.props.ori, st.props.att], [orientation, attitude], aMs, st.lastAttMs, st.lastAttJD)')
    assert one.index('if (st.kind !== undefined && st.kind !== modelKind) {') < i_loop < i_main, \
        '機型變了先換掉模型姿態屬性；舊樣本先加、主樣本最後'
    loop = one[i_loop:i_main]
    assert 'if (_uavSampleMs(sMs, nowMs) !== sMs || !(sMs < aMs)) continue;' in loop
    assert 'if (!Array.isArray(s) || s.length !== 4) continue;' in loop
    assert '_uavNoseHeading(s[3], crsDeg, speedMs, vehicleType, mode)' in loop, '機頭規則和主樣本相同'
    assert '_uavModelOrientation(pos, sNose, s[2], s[1], headingOffset, pitchOffset, rollOffset)' in loop, \
        '俯仰是 s[2]、滾轉是 s[1]；校正與主樣本同一組'
    assert ('Cesium.Math.toRadians(sNose), Cesium.Math.toRadians(s[2] || 0), Cesium.Math.toRadians(s[1] || 0)'
            in loop), '相機用的原始姿態'
    assert '_uavPushSample([st.props.ori, st.props.att], [sOri, sAtt], sMs, st.lastAttMs, st.lastAttJD)' in loop
    assert 'if (sj) { st.lastAttMs = sMs; st.lastAttJD = sj; st.nAtt += 1; }' in loop
    assert loop.count('addSample(') == 0


def test_display_delay_tracks_twice_the_sample_interval_with_a_slew_limit():
    assert 'const _UAV_DELAY_INIT = 0.12, _UAV_DELAY_MIN = 0.08, _UAV_DELAY_MAX = 1.0;' in HTML
    assert 'const _UAV_DELAY_SLEW = 0.2, _UAV_DT_EMA_ALPHA = 0.1;' in HTML
    assert 'return Math.max(_UAV_DELAY_MIN, Math.min(_UAV_DELAY_MAX, 2 * dtEma + 0.03));' in _fn('_uavDelayTarget')
    note = _code(_fn('_uavNoteInterval'))
    assert 'if (gapMs >= _UAV_MIN_SAMPLE_GAP_MS) {' in note
    assert 'st.dtEma = (st.dtEma === null) ? g : st.dtEma + _UAV_DT_EMA_ALPHA * (g - st.dtEma);' in note
    assert 'const maxStep = _UAV_DELAY_SLEW * Math.max(0, (nowMs - st.delayUpdMs) / 1000);' in note
    assert 'st.delay += Math.max(-maxStep, Math.min(maxStep, want));' in note and 'st.delayUpdMs = nowMs;' in note
    one = _code(_fn('_updateUAVOne'))
    i_pj = one.index('if (pj) {')
    block = one[i_pj:one.index('}', i_pj)]
    assert '_uavNoteInterval(st, prevPosMs > 0 ? pMs - prevPosMs : -1, nowMs);' in block, '只在加了位置樣本時更新延遲'
    assert '_appendTrail(sysid, lat, lon, alt, color, pMs);' in block, '重送的同一點不進尾跡'
    assert 'delay: _UAV_DELAY_INIT' in one


def test_entities_and_cameras_read_the_per_aircraft_display_time_not_the_viewer_clock():
    disp = _code(_fn('_uavDisplayTime'))
    assert 'Cesium.JulianDate.addSeconds(Cesium.JulianDate.now(result), -st.delay, result)' in disp
    assert 'return Date.now() - st.delay * 1000;' in _fn('_uavDisplayMs')
    one = _code(_fn('_updateUAVOne'))
    assert 'return st.props.pos.getValue(_uavDisplayTime(st, _uavScratchJD), r) || st.lastPos;' in one
    assert 'return st.props.ori.getValue(_uavDisplayTime(st, _uavScratchJD), r) || st.lastOri;' in one
    assert 'position: posProp,' in one and 'orientation: oriProp,' in one
    assert 'ConstantPositionProperty' not in one and 'ConstantProperty' not in one
    for n in ('_uavSmoothPos', '_uavSmoothHpr'):
        assert re.search(r'function %s\(st\)' % n, HTML) and '_uavDisplayTime(st, _uavScratchJD)' in _fn(n)
    for a, b in (('_chasePreRender = function() {', 'viewer.scene.preRender.addEventListener(_chasePreRender);'),
                 ('_fpvPreRender = function() {', 'viewer.scene.preRender.addEventListener(_fpvPreRender);')):
        seg = _code(HTML[HTML.index(a):HTML.index(b)])
        assert '_uavSmoothPos(st)' in seg and '_uavSmoothHpr(st)' in seg
        assert 'viewer.clock.currentTime' not in seg and '_requestRender' not in seg, \
            'preRender 回呼裡要求算圖 = 永久 60 fps；讀 clock 時間 = 跟模型不同步'


def test_pruning_keeps_thirty_seconds():
    assert 'const _UAV_SAMPLE_KEEP_S = 30, _UAV_PRUNE_EVERY_MS = 5000;' in HTML
    prune = _code(_fn('_uavPruneSamples'))
    assert 'new Date(nowMs - _UAV_SAMPLE_KEEP_S * 1000)' in prune and "st.props[k].removeSamples(iv)" in prune


# ═══════════════════════════════════════════════════════════════════════
#  C. 時鐘
# ═══════════════════════════════════════════════════════════════════════
def test_clock_runs_with_system_clock_multiplier_never_touches_current_time_and_never_suspends():
    i_viewer_end = HTML.index('});', HTML.index("const viewer = new Cesium.Viewer('cesiumContainer', {"))
    i_allow = HTML.index('viewer.allowDataSourcesToSuspendAnimation = false;')
    assert i_viewer_end < i_allow < HTML.index('viewer.scene.maximumRenderTimeChange = __RENDER_MAX_DT__;'), \
        'Viewer 建好就要關（primitive 在建時 canAnimate=false 會讓時鐘停）'
    body = _code(_fn('_ensureClockRunning'))
    assert 'c.clockStep = Cesium.ClockStep.SYSTEM_CLOCK_MULTIPLIER' in body
    assert 'ClockStep.SYSTEM_CLOCK)' not in body and 'ClockStep.SYSTEM_CLOCK;' not in body, \
        'SYSTEM_CLOCK 每個 tick 都把 currentTime 蓋成現在 ⇒ 末端俯衝自己設的起點被吃掉'
    for k in ('c.clockRange = Cesium.ClockRange.UNBOUNDED', 'c.multiplier = 1', 'c.shouldAnimate = true'):
        assert k in body, k
    assert 'currentTime =' not in body and 'currentTime=' not in body, '從不改 currentTime'
    assert 'Cesium.JulianDate.lessThan(c.currentTime, c.stopTime)' in body and 'if (animating) return;' in body
    assert re.search(r'^_ensureClockRunning\(\);', HTML, re.M), 'init 要呼叫一次（widget 關了沒人按播放）'
    assert 'function _ensureClockRunning()' in HTML and '_ensureClockRunning(true)' not in HTML


# ═══════════════════════════════════════════════════════════════════════
#  D. 機鼻
# ═══════════════════════════════════════════════════════════════════════
def test_nose_is_the_fc_yaw_by_default_and_the_track_blend_is_an_explicit_opt_in():
    nose = _code(_fn('_uavNoseHeading'))
    assert 'function _uavNoseHeading(hdgDeg, crsDeg, spdMs, vehicleType, mode)' in nose
    assert "if (!_UAV_NOSE_FOLLOWS_TRACK || typeof crsDeg !== 'number' || !isFinite(crsDeg)) return hdg;" in nose
    assert "if (vt.indexOf('PLANE') < 0 && vt.indexOf('FIXED') < 0 && vt.indexOf('VTOL') < 0) return hdg;" in nose
    assert "if (String(mode || '').toUpperCase().charAt(0) === 'Q') return hdg;" in nose
    assert 'const w = Math.max(0, Math.min(1, ((spdMs || 0) - _UAV_NOSE_BLEND_LO) / (_UAV_NOSE_BLEND_HI - _UAV_NOSE_BLEND_LO)));' in nose
    assert 'const d = ((crsDeg - hdg) % 360 + 540) % 360 - 180;' in nose, '最短角差'
    assert 'const _UAV_NOSE_BLEND_LO = 3.0, _UAV_NOSE_BLEND_HI = 6.0;' in HTML
    # 2026-10-22：預設關 ⇒ 模型機鼻 = 飛控 yaw；過渡（機鼻改指航跡）只能明確打開
    assert 'let _UAV_NOSE_FOLLOWS_TRACK = false;' in HTML and 'let _UAV_NOSE_FOLLOWS_TRACK = true;' not in HTML
    setter = _code(_fn('setUavNoseFollowsTrack'))
    assert 'function setUavNoseFollowsTrack(on)' in setter
    assert '_UAV_NOSE_FOLLOWS_TRACK = !!on;' in setter and 'return _UAV_NOSE_FOLLOWS_TRACK;' in setter
    assert _requests_render_at_tail(_fn('setUavNoseFollowsTrack'))
    writes = re.findall(r'_UAV_NOSE_FOLLOWS_TRACK\s*=(?!=)', _code(HTML))
    assert len(writes) == 2, f'只有宣告與 setUavNoseFollowsTrack 可以改這個旗標（頁面裡不可以偷偷打開）：{writes}'
    assert '_UAV_TRACK_MIN_SPEED' not in HTML, '2 m/s 硬切已拿掉'


def test_model_attitude_camera_attitude_and_billboard_all_use_the_same_nose():
    one = _strip_comments(_fn('_updateUAVOne'))       # 連行內註解也去掉（四元數那行後面掛著註解）
    assert 'const noseDeg = _uavNoseHeading(headingDeg, crsDeg, speedMs, vehicleType, mode);' in one
    # 模型（含 GLB 校正偏移）：2026-10-22 起校正是另外乘上去的固定旋轉（見 test_real_cesium_model_rolls_about_the_nose…）
    assert '_uavModelOrientation(pos, noseDeg, pitchDeg, rollDeg, headingOffset, pitchOffset, rollOffset)' in one
    assert re.search(r'Cesium\.Quaternion\.fromHeadingPitchRoll\(new Cesium\.HeadingPitchRoll\(\s*'
                     r'Cesium\.Math\.toRadians\(noseDeg\),', one), '相機用的原始姿態四元數也要用 noseDeg'
    assert one.count('Cesium.Math.toRadians(-noseDeg)') == 2, '平面圖示（建立＋更新）'
    assert 'headingDeg || 0) + headingOffset' not in one and 'toRadians(-(headingDeg' not in one
    assert 'st.heading = noseDeg; st.yaw = headingDeg || 0;' in one


def test_model_and_its_glb_calibration_come_from_one_model_kind_and_change_together():
    """機型中途變了（第一批還沒 HEARTBEAT ⇒ '---' 當多旋翼；Q_ENABLE 讀回後 PLANE → VTOL）：以前模型建立時定死、
    校正卻每筆照新機型算 ⇒ 多旋翼 GLB 套固定翼校正。行為見 test_real_cesium_vehicle_type_change_rebuilds_the_model…"""
    one = _strip_comments(_fn('_updateUAVOne'))
    k = one.index("const modelKind = isHeroGlb ? 'hero' : isPioneerGlb ? 'pioneer' : isVtolGlb ? 'vtol' : isPlaneGlb ? 'plane' "
                  ": isCopterGlb ? 'copter'\n                    : (isPlane ? 'planeBox' : 'copterBall');")
    assert one.index('if (isCopterGlb) { headingOffset = _COPTER_HEADING_OFFSET;') < k, '校正與模型種類出自同一組旗標'
    assert one.index('if (isHeroGlb)   { headingOffset = _HERO_HEADING_OFFSET;') < k
    assert one.index('if (isPioneerGlb) { headingOffset = _PIONEER_HEADING_OFFSET;') < k
    # 旗標互斥（校正是依序 if 指定的；isPlane 包含 VTOL）：機體專用模型優先，通用 VTOL、固定翼都要排除它們
    assert "const airframe = isVtol ? _uavAirframeOf(sysid) : '';" in one, '機體只對 VTOL 有效'
    assert "const isHeroGlb = airframe === 'hero' && !!_HERO_MODEL_URI;" in one
    assert "const isPioneerGlb = airframe === 'pioneer' && !!_PIONEER_MODEL_URI;" in one
    assert 'const isAirframeGlb = isHeroGlb || isPioneerGlb;' in one
    assert 'const isVtolGlb = !isAirframeGlb && isVtol &&' in one
    assert 'const isPlaneGlb = !isAirframeGlb && !isVtolGlb && isPlane &&' in one
    assert 'if (st.kind !== undefined && st.kind !== modelKind) {' in one
    assert 'st.props.ori = _uavSampledProps().ori;' in one, '舊校正的模型姿態樣本要丟掉'
    assert one.index('st.kind = modelKind;') < one.index('_uavPushSample([st.props.ori, st.props.att]')
    assert one.index('if (entity && entity._uavKind !== modelKind) {') < one.index('if (!entity) {')
    assert 'entity._uavKind = modelKind;' in one


@needs_qt
def test_batch_item_keeps_a_true_north_yaw_once_attitude_was_received():
    """以前 ``yaw if yaw != 0 or heading == 0 else heading``：真的朝正北（ATTITUDE yaw = 0.0）會被換成 VFR_HUD 的整數 heading。"""
    from mission.sitl_link import TelemetryFrame
    from ui.main_window import MainWindow
    base = dict(sysid=1, lat=23.45, lon=120.28, gps_fix=3, vel_n=0.0, vel_e=20.0, ground_speed=20.0)
    north = MainWindow._uav_batch_item(TelemetryFrame(yaw=0.0, heading=359.0, att_t=1_700_000_000.2, **base))
    assert north['hdg'] == 0.0, '收過 ATTITUDE（att_t > 0）時 yaw 0.0 就是正北，不可以換成 VFR_HUD 的 359'
    assert north['at'] == 1_700_000_000_200 and north['crs'] == pytest.approx(90.0)
    legacy = MainWindow._uav_batch_item(TelemetryFrame(yaw=0.0, heading=359.0, **base))
    assert legacy['hdg'] == 359.0 and 'at' not in legacy, '沒收過 ATTITUDE（沒有 att_t）才退回 heading'
    bad = MainWindow._uav_batch_item(TelemetryFrame(yaw=float('nan'), heading=12.0, att_t=1_700_000_000.2, **base))
    assert bad['hdg'] == 12.0, '壞掉的 yaw 退回 heading（不把 NaN 送進 3D）'
    west = MainWindow._uav_batch_item(TelemetryFrame(yaw=-90.0, heading=271.0, att_t=1_700_000_000.2, **base))
    assert west['hdg'] == 270.0, '-180～180 的 yaw 正規化成 0～360，而且用 yaw 不用整數 heading'


# ═══════════════════════════════════════════════════════════════════════
#  E. 尾跡
# ═══════════════════════════════════════════════════════════════════════
def test_trail_caches_cartesians_and_only_takes_points_the_display_time_has_reached():
    app = _code(_fn('_appendTrail'))
    assert 'function _appendTrail(sysid, lat, lon, alt, colorHex, tMs)' in app
    assert 'fromDegreesArrayHeights' not in _code(HTML[HTML.index('function _trailPushPoint('):HTML.index('const COLLISION_WARN_DIST')])
    assert "pend.push({ c: Cesium.Cartesian3.fromDegrees(lon, lat, alt || 0), ms: (typeof tMs === 'number') ? tMs : Date.now() });" in app
    assert 'while (pend.length > 1500) _trailPushPoint(sysid, pend.shift().c);' in app, '不算圖時待進點不能無限長'
    push = _code(_fn('_trailPushPoint'))
    assert 'pts.push(c);' in push and 'if (pts.length > 1500) pts.shift();' in push and 'uavTrailDirty[sysid] = true;' in push
    adv = _code(_fn('_trailAdvance'))
    assert 'const tdMs = st ? _uavDisplayMs(st) : Infinity;' in adv
    assert 'while (pend.length && pend[0].ms <= tdMs) _trailPushPoint(sysid, pend.shift().c);' in adv
    cb = app[app.index('new Cesium.CallbackProperty('):]
    cb = cb[:cb.index('}, false)')]
    assert cb.index('_trailAdvance(sysid);') < cb.index('if (uavTrailDirty[sysid])')
    assert '.slice()' in cb and 'uavTrailDirty[sysid] = false' in cb and 'fromDegrees' not in cb
    assert 'uavTrailPts' not in _code(HTML), '舊的平面陣列整個拿掉（只准在註解裡提到）'


def test_clear_uav_wipes_state_and_all_trail_caches():
    body = _code(_fn('clearUAV'))
    for k in ('delete uavTrailCart[sid]', 'delete uavTrailDirty[sid]', 'delete uavTrailCache[sid]',
              'delete uavTrailPend[sid]', 'delete _UAV_STATE[sysid];', 'Object.keys(uavTrailCart).forEach(_rmTrail)'):
        assert k in body, k
    assert _top_statements(_fn('clearUAV'))[-1] == '_requestRender()'


# ═══════════════════════════════════════════════════════════════════════
#  F. 自適應算圖、widget、每個改場景的入口
# ═══════════════════════════════════════════════════════════════════════
def test_viewer_takes_both_placeholders_and_the_template_still_parses_without_replacement():
    assert 'requestRenderMode: __REQUEST_RENDER_MODE__,' in HTML
    assert 'viewer.scene.maximumRenderTimeChange = __RENDER_MAX_DT__;' in HTML
    assert 'requestRenderMode: false' not in HTML and 'requestRenderMode: true' not in HTML
    g1 = HTML.index("if (typeof __REQUEST_RENDER_MODE__ === 'undefined') { window['__REQUEST_RENDER_MODE__'] = true;")
    g2 = HTML.index("if (typeof __RENDER_MAX_DT__ === 'undefined') { window['__RENDER_MAX_DT__'] = 0.05;")
    assert max(g1, g2) < HTML.index("const viewer = new Cesium.Viewer('cesiumContainer', {")
    assert HTML.index("const viewer = new Cesium.Viewer(") < HTML.index('function _requestRender()') < HTML.index('function _appendTrail(')
    stale = HTML[HTML.index('viewer.allowDataSourcesToSuspendAnimation = false;'):HTML.index('viewer.scene.maximumRenderTimeChange = __RENDER_MAX_DT__;')]
    assert 'shouldAnimate=false' not in stale and '1/__RENDER_MAX_DT__ fps' in stale, '註解要跟現況一致（時鐘會走）'


def test_render_mode_is_adaptive_continuous_while_telemetry_arrives_and_on_demand_when_idle():
    assert 'const _RENDER_ADAPTIVE = (__REQUEST_RENDER_MODE__ === true);' in HTML
    assert 'let _RENDER_IDLE_MS = 2000;' in HTML
    note = _code(_fn('_noteUavActivity'))
    assert '_lastUavUpdateMs = Date.now();' in note
    assert 'if (_RENDER_ADAPTIVE && viewer.scene.requestRenderMode) viewer.scene.requestRenderMode = false;' in note
    idle = _code(_fn('_renderIdleCheck'))
    assert 'Date.now() - _lastUavUpdateMs > _RENDER_IDLE_MS' in idle
    assert 'viewer.scene.requestRenderMode = true;' in idle and 'viewer.scene.maximumRenderTimeChange = __RENDER_MAX_DT__;' in idle
    assert 'if (!_RENDER_ADAPTIVE || viewer.scene.requestRenderMode) return;' in idle
    assert 'setInterval(_renderIdleCheck, 500);' in HTML
    assert len(re.findall(r'^\s*_noteUavActivity\(\);', HTML, re.M)) == 2, '單架與批次兩個入口'


def test_timeline_and_animation_widgets_are_off_and_the_only_timeline_reference_is_guarded():
    assert 'timeline: false,' in HTML and 'animation: false,' in HTML
    assert 'timeline: true' not in HTML and 'animation: true' not in HTML
    refs = [ln.strip() for ln in _code(HTML).splitlines() if 'viewer.timeline' in ln or 'viewer.animation' in ln]
    assert refs and all(ln.startswith(('if (viewer.timeline)', 'if (viewer.animation)')) for ln in refs), refs


# 每一個會改場景的公開函式（Python 端或工具列會直接呼叫）——**最後一個頂層敘述**要要求算圖（SURVEY_LAYERS 段用段內的
# _surveyRender()；flyToPos 用 typeof 守門形式，因為 tests/test_cesium_flyto.py 只抽它一個進頁面）。
_SCENE_PUBLIC = [
    'updateUAVPosition', 'updateUAVPositions', 'clearUAV', 'flyToPos', 'flyToScene', 'setViewPos',
    'setChaseCamera', 'setChaseCameraParams', 'resetChaseView', 'setFpvCamera', 'setFpvCameraParams',
    'toggleLayer', 'clearPaths', 'addPath', 'clearCorners', 'setCorners',
    'clearTacticalOverlay', 'setDispersionRing', 'addLoiterCircle', 'addDiveLine',
    'clearMissionOverlay', 'clearMissionLayer', 'clearMissionLayerFamily', 'addMissionRoute', 'addMissionHome', 'addMissionLoiter',
    'clearGeofence', 'setGeofence', 'clearTotGate', 'setTotGate', 'clearExplosions', 'explodeAt',
    'clearStores', 'dropStore', 'clearImpacts', 'addFenceZone', 'removeFenceZone', 'clearAllFenceZones',
    'setCircleOverlay', 'setHomePoint', 'clearNFZ', 'addNFZPolygon', 'addNFZCircle',
    'clearDemOverlay', 'showDemOverlay', 'clearDemTerrain', 'applyDemTerrainProvider', 'showDemTerrainHeightmap',
    'showDemTerrain3D', 'setInitialView', 'updateElevationSlicer', 'clearElevationSlicer',
    'updateFOVCone', 'clearFOVCone', 'initSARHeatmap', 'updateHeatmap', 'renderCoverageHeatmap', 'clearSARHeatmap',
    'resetSARHeatmap', 'addRadarDome', 'clearRadarDomes', 'updateRCSSensitivity', 'clearRCSSensitivity',
    'strikeAddTarget', 'strikeClearAll', 'strikeExecuteAnimation', 'strikeVizBegin', 'strikeVizUpdateUav', 'strikeVizEnd',
    'strikeRenderPath', 'drawVTOLSwarmPaths', 'vtolClearAll', 'drawCoverageGaps', 'clearCoverageGaps',
    'renderTerminalDive', 'clearTerminalDive', 'clearAllTerminalDives',
    'addOrthoLayer', 'removeOrthoLayer', 'setPhotoPoses', 'clearPhotoPoses', 'setSurveyPoints', 'clearSurveyPoints',
]
_RENDER_CALLS = ('_requestRender()', '_renderPumpFor(', '_surveyRender()')


@pytest.mark.parametrize('name', _SCENE_PUBLIC)
def test_scene_changing_public_function_ends_with_a_render_request(name):
    assert _requests_render_at_tail(_fn(name)), f'{name}：最後一個頂層敘述不是要求算圖 —— {_top_statements(_fn(name))[-2:]}'


def test_survey_section_uses_its_own_safe_wrapper_because_it_is_loaded_alone_in_a_stub_page():
    sec = _code(HTML[HTML.index('SURVEY_LAYERS_BEGIN'):HTML.index('// SURVEY_LAYERS_END')])
    assert "function _surveyRender() { if (typeof _requestRender === 'function') _requestRender(); }" in sec
    assert sec.replace("if (typeof _requestRender === 'function') _requestRender();", '').count('_requestRender') == 0, \
        'SURVEY_LAYERS 段只能透過 _surveyRender()（tests/test_survey_map_layers.py 的假 Cesium 頁沒有 _requestRender）'
    assert sec.count('_surveyRender();') == 4, 'addOrthoLayer / removeOrthoLayer / setPhotoPoses / clearPhotoPoses'


_SCENE_CALL = re.compile(r'viewer\.entities\.(?:add|remove|removeAll)\(|scene\.primitives\.(?:add|remove)\(|'
                         r'imageryLayers\.(?:add|remove)\(|viewer\.terrainProvider\s*=|globe\.material\s*=')


def test_no_top_level_function_changes_the_scene_without_requesting_a_render_unless_it_is_a_private_helper():
    """抓新加的函式：頂層函式只要增刪 entity／primitive／imagery 或換地形、材質，就得自己要求算圖，
    除非是底線開頭的私有 helper（由公開的呼叫端負責）。"""
    offenders = []
    for n in _top_level_functions():
        body = _code(_fn(n))
        if not _SCENE_CALL.search(body) or any(k in body for k in _RENDER_CALLS):
            continue
        if n.startswith('_') or n == 'grabCanvas':      # grabCanvas 自己 viewer.render()
            continue
        offenders.append(n)
    assert offenders == [], offenders


def test_every_raf_or_tick_driven_animation_loop_requests_a_render():
    loops = set(re.findall(r'requestAnimationFrame\((_\w+)\)', HTML))
    assert {'_pulse', '_expand', '_shockExpand', '_scorchFade', '_pump'} <= loops, loops
    for n in loops:
        assert '_requestRender()' in _code(_fn(n, top_level=False)), n
    assert '_requestRender()' in _code(_fn('_onClockTick', top_level=False)), '末端俯衝的 clock tick'
    tick = _code(HTML[HTML.index('const tickListener = function(clock) {'):])
    assert '_requestRender()' in tick[:300], '打擊動畫的 clock tick'
    for n in ('explodeAt', 'dropStore', '_impactMark'):
        assert '_renderPumpFor(' in _fn(n), n
    assert '_renderPumpFor(SHOCKWAVE_DURATION' in HTML, '末端俯衝震波'
    pump = _code(_fn('_renderPumpFor'))
    assert 'requestAnimationFrame(_pump)' in pump and '_renderPumpOn = false' in pump, '到期要停，不能變成永久 60 fps'


# ═══════════════════════════════════════════════════════════════════════
#  G. Python 端
# ═══════════════════════════════════════════════════════════════════════
def _bare():
    w = _NoDel.__new__(_NoDel)
    sent, hits = [], []
    w.__dict__['_js'] = lambda s: sent.append(s)
    w.__dict__['check_bullseye_hit'] = lambda *a: hits.append(a)
    return w, sent, hits


def _item(sid: int) -> dict:
    return {'sysid': sid, 'lat': 25.07 + sid * 0.000123456789, 'lon': 121.21 + sid * 0.0001, 'alt': 100.0 + sid / 3.0,
            'hdg': 90.0 + sid, 'spd': 18.123456, 'mode': 'AUTO', 'armed': True, 'vt': 'VTOL',
            'pitch': 1.23456, 'roll': -2.34567, 'crs': 135.456, 'pt': 1_760_000_000_000 + sid, 'at': 1_760_000_000_010.6}


def _payload(js: str) -> list:
    assert js.startswith('updateUAVPositions([') and js.endswith('])'), js[:80]
    return json.loads(js[len('updateUAVPositions('):-1])


@needs_qt
def test_six_aircraft_become_exactly_one_js_call_with_all_six_sysids():
    w, sent, hits = _bare()
    w.update_uav_positions([_item(i) for i in range(1, 7)])
    assert len(sent) == 1, sent
    items = _payload(sent[0])
    assert [it['sysid'] for it in items] == [1, 2, 3, 4, 5, 6]
    assert all(set(it) == set(CesiumMapWidget.UAV_BATCH_KEYS) | {'crs', 'pt', 'at'} for it in items)
    it = items[1]
    assert it['lat'] == round(_item(2)['lat'], 7) and it['lon'] == round(_item(2)['lon'], 7)
    assert it['spd'] == 18.12 and it['pitch'] == 1.23 and it['roll'] == -2.35 and it['alt'] == round(100 + 2 / 3, 2)
    assert it['crs'] == 135.46 and it['mode'] == 'AUTO' and it['armed'] is True and it['vt'] == 'VTOL'
    assert it['pt'] == 1_760_000_000_002 and it['at'] == 1_760_000_000_011
    assert '"pt":1760000000002,' in sent[0] and '"at":1760000000011' in sent[0], 'pt／at 是整數（JSON 沒有小數點）'
    assert ' ' not in sent[0], '緊湊 JSON（separators）'
    assert hits == [(it['sysid'], it['lat'], it['lon'], it['alt']) for it in items]


@needs_qt
def test_optional_keys_are_sent_only_when_valid():
    w, sent, _ = _bare()
    base = {'lat': 25.0, 'lon': 121.0}
    w.update_uav_positions([dict(base, sysid=1),                                            # 都沒有
                            dict(base, sysid=2, crs=None, pt=None, at=None),               # None = 沒有
                            dict(base, sysid=3, crs='x', pt='x', at=True),                 # 不是數字／bool
                            dict(base, sysid=4, crs=float('nan'), pt=float('inf'), at=float('nan')),
                            dict(base, sysid=5, crs=0, pt=0, at=-5),                       # crs 0 合法；pt／at 要 > 0
                            dict(base, sysid=6, crs='270.126', pt=1.7e12 + 0.4, at='1700000000123')])
    items = _payload(sent[0])
    assert [sorted(set(it) - set(CesiumMapWidget.UAV_BATCH_KEYS)) for it in items] == \
        [[], [], [], [], ['crs'], ['at', 'crs', 'pt']]
    assert items[4]['crs'] == 0 and items[5]['crs'] == 270.13
    assert items[5]['pt'] == 1_700_000_000_000 and items[5]['at'] == 1_700_000_000_123
    assert CesiumMapWidget.UAV_BATCH_OPTIONAL_KEYS == ('crs', 'pt', 'at', 'atts')
    assert not set(CesiumMapWidget.UAV_BATCH_OPTIONAL_KEYS) & set(CesiumMapWidget.UAV_BATCH_KEYS)


@needs_qt
def test_older_attitude_samples_ride_along_validated_rounded_and_capped():
    """atts（2026-10-22：50 Hz 姿態、30 Hz 合幀，舊樣本跟著最新一筆走）：每筆 [ms 整數, roll, pitch, 機頭] 2 位小數；
    壞的那一筆略過（其他照送）；一筆都不剩或不是串列 ⇒ 不帶這個鍵；只留最新的 UAV_BATCH_MAX_ATTS 筆。"""
    w, sent, _ = _bare()
    base = {'lat': 25.0, 'lon': 121.0, 'at': 1_700_000_000_100}
    good = [[1_700_000_000_060.4, 1.234, -2.345, 359.999], [1_700_000_000_080.6, 3, 4, 5]]
    bad = [None, 'x', [1, 2, 3], [1, 2, 3, 4, 5], [float('nan'), 1, 2, 3], [0, 1, 2, 3], [-5, 1, 2, 3],
           [1.7e12, float('inf'), 2, 3], [1.7e12, 1, 2, 'y'], [True, 1, 2, 3], [1.7e12, 1, False, 3], 'abcd']
    many = [[1_700_000_000_000 + k, k, 0, 0] for k in range(100)]
    w.update_uav_positions([dict(base, sysid=1, atts=good + bad), dict(base, sysid=2, atts=bad),
                            dict(base, sysid=3, atts='not a list'), dict(base, sysid=4, atts=[]),
                            dict(base, sysid=5, atts=many), dict(base, sysid=6, atts=(tuple(good[1]),))])
    items = _payload(sent[0])
    assert items[0]['atts'] == [[1_700_000_000_060, 1.23, -2.35, 360.0], [1_700_000_000_081, 3.0, 4.0, 5.0]]
    assert all('atts' not in it for it in items[1:4]), '全部壞／不是串列／空 ⇒ 不帶'
    assert CesiumMapWidget.UAV_BATCH_MAX_ATTS == 64 and len(items[4]['atts']) == 64
    assert items[4]['atts'][0][0] == 1_700_000_000_036 and items[4]['atts'][-1][0] == 1_700_000_000_099, '留最新的'
    assert items[5]['atts'] == [[1_700_000_000_081, 3.0, 4.0, 5.0]], 'tuple 也收'
    assert '"atts":[[1700000000060,1.23,-2.35,360.0],' in sent[0], 'ms 是整數'


@needs_qt
def test_single_update_goes_through_the_same_batch_path_without_optional_keys():
    w, sent, hits = _bare()
    w.update_uav_position(25.07, 121.21, 120.0, 45.0, 17.0, sysid=3, mode='GUIDED', armed=False,
                          vehicle_type='PLANE', pitch_deg=2.0, roll_deg=-1.0)
    assert len(sent) == 1
    assert _payload(sent[0]) == [{'lat': 25.07, 'lon': 121.21, 'alt': 120.0, 'hdg': 45.0, 'spd': 17.0, 'sysid': 3,
                                  'mode': 'GUIDED', 'armed': False, 'vt': 'PLANE', 'pitch': 2.0, 'roll': -1.0}], \
        '單架沒有 crs／pt／at ⇒ JS 端機鼻用 hdg、樣本時間用 Date.now()'
    assert hits == [(3, 25.07, 121.21, 120.0)]


@needs_qt
def test_missing_keys_get_defaults_a_bad_item_is_skipped_and_an_empty_batch_sends_nothing():
    w, sent, hits = _bare()
    w.update_uav_positions([{'sysid': 2, 'lat': 25.0, 'lon': 121.0}, {'sysid': 9, 'lon': 121.0},
                            {'sysid': 4, 'lat': 'x', 'lon': 121.0}, None])
    assert len(sent) == 1
    assert _payload(sent[0]) == [{'lat': 25.0, 'lon': 121.0, 'alt': 0.0, 'hdg': 0.0, 'spd': 0.0, 'sysid': 2,
                                  'mode': '', 'armed': False, 'vt': '', 'pitch': 0.0, 'roll': 0.0}]
    assert hits == [(2, 25.0, 121.0, 0.0)]
    w.update_uav_positions([])
    w.update_uav_positions(None)
    w.update_uav_positions([{'sysid': 1}])
    assert len(sent) == 1, '沒有可用的筆就不要送空批次'


@needs_qt
def test_non_ascii_and_quotes_in_mode_survive_the_json_round_trip_as_ascii_only_javascript():
    w, sent, _ = _bare()
    mode = "自動'\" x"
    w.update_uav_positions([{'sysid': 1, 'lat': 25.0, 'lon': 121.0, 'mode': mode, 'vt': 'VTOL\\'}])
    js = sent[0]
    assert js.isascii() and '\\u2028' in js and '\\u81ea' in js
    assert _payload(js)[0]['mode'] == mode and _payload(js)[0]['vt'] == 'VTOL\\'


@needs_qt
def test_render_placeholders_default_env_override_zero_and_garbage(monkeypatch):
    w = _NoDel.__new__(_NoDel)
    warnings: list[str] = []
    monkeypatch.setattr(cmw.logger, 'warning', lambda msg, *a, **k: warnings.append(str(msg)))   # 專案 logger 不進 caplog
    monkeypatch.delenv(CesiumMapWidget.RENDER_DT_ENV, raising=False)
    assert CesiumMapWidget.RENDER_MAX_DT == 0.05
    assert w._render_mode_placeholders() == {'__REQUEST_RENDER_MODE__': 'true', '__RENDER_MAX_DT__': '0.05'}
    monkeypatch.setenv(CesiumMapWidget.RENDER_DT_ENV, '0.2')
    assert w._render_mode_placeholders() == {'__REQUEST_RENDER_MODE__': 'true', '__RENDER_MAX_DT__': '0.2'}
    monkeypatch.setenv(CesiumMapWidget.RENDER_DT_ENV, '0')
    assert w._render_mode_placeholders() == {'__REQUEST_RENDER_MODE__': 'false', '__RENDER_MAX_DT__': '0'}
    assert warnings == []
    for bad in ('abc', 'inf', 'nan', '-1'):
        monkeypatch.setenv(CesiumMapWidget.RENDER_DT_ENV, bad)
        del warnings[:]
        assert w._render_mode_placeholders() == {'__REQUEST_RENDER_MODE__': 'true', '__RENDER_MAX_DT__': '0.05'}, bad
        assert len(warnings) == 1 and CesiumMapWidget.RENDER_DT_ENV in warnings[0] and repr(bad) in warnings[0], (bad, warnings)
    monkeypatch.setenv(CesiumMapWidget.RENDER_DT_ENV, '  ')
    del warnings[:]
    assert w._render_mode_placeholders()['__RENDER_MAX_DT__'] == '0.05' and warnings == [], '空白 = 沒設，不是壞值'
    monkeypatch.delenv(CesiumMapWidget.RENDER_DT_ENV, raising=False)
    monkeypatch.setattr(CesiumMapWidget, 'RENDER_MAX_DT', 0)
    assert w._render_mode_placeholders() == {'__REQUEST_RENDER_MODE__': 'false', '__RENDER_MAX_DT__': '0'}
    monkeypatch.setattr(CesiumMapWidget, 'RENDER_MAX_DT', 1e-05)
    assert w._render_mode_placeholders()['__RENDER_MAX_DT__'] == '1e-05', '要是合法的 JS 數字字面量'


def _no_settings():
    raise RuntimeError('測試不讀 config/settings.json')


def _template_values(monkeypatch) -> dict:
    """真的 _cesium_template_values()，但不讀設定檔（預設座標走 fallback）、不帶 token。"""
    import config.settings as cs
    monkeypatch.setattr(cs, 'get_settings', _no_settings)
    monkeypatch.setattr(CesiumMapWidget, '_resolve_token', lambda self: '')
    monkeypatch.delenv(CesiumMapWidget.RENDER_DT_ENV, raising=False)
    return _NoDel.__new__(_NoDel)._cesium_template_values()


@needs_qt
def test_template_values_cover_exactly_the_placeholders_in_the_template(monkeypatch):
    vals = _template_values(monkeypatch)
    in_template = set(re.findall(r'__[A-Z][A-Z0-9_]*__', HTML))
    keys = {k.strip('/*') for k in vals}
    assert keys == in_template, (keys ^ in_template)
    assert list(vals)[:3] == ['__WIND_FLOW_SCRIPT__', '/*__WIND_GLUE__*/', '/*__WIND_RESTORE__*/'], '風場三個先換（舊順序）'
    assert vals['__INIT_LAT__'] == '25.0777' and vals['__CESIUM_TOKEN__'] == ''
    assert vals['__REQUEST_RENDER_MODE__'] == 'true' and vals['__RENDER_MAX_DT__'] == '0.05'


@needs_qt
def test_fill_template_on_the_real_text_leaves_no_placeholder_and_load_cesium_uses_it(monkeypatch):
    vals = _template_values(monkeypatch)
    w = _NoDel.__new__(_NoDel)
    monkeypatch.setattr(CesiumMapWidget, '_cesium_template_values', lambda self: (_ for _ in ()).throw(
        AssertionError('給了 values 就不可以再去讀設定／檔案')))
    html = w._fill_cesium_template(HTML, vals)
    left = sorted(set(re.findall(r'__[A-Z][A-Z0-9_]*__', _code(html))))   # 註解裡提到佔位字的名字不算
    assert left == [], left
    assert 'requestRenderMode: true,' in html and 'viewer.scene.maximumRenderTimeChange = 0.05;' in html
    assert 'const _RENDER_ADAPTIVE = (true === true);' in html
    assert "if (typeof true === 'undefined') { window['true'] = true;" in html, '守門那兩行退化成無害敘述'
    assert "Cesium.Ion.defaultAccessToken = '';" in html and 'setInitialView(25.0777, 121.2325, 2000.0);' in html
    assert 'function setWindField' in html, '風場膠水換進去了'
    zero = dict(vals, __REQUEST_RENDER_MODE__='false', __RENDER_MAX_DT__='0')
    html0 = w._fill_cesium_template(HTML, zero)
    assert 'requestRenderMode: false,' in html0 and 'const _RENDER_ADAPTIVE = (false === true);' in html0
    src = inspect.getsource(CesiumMapWidget._load_cesium)
    assert 'html = self._fill_cesium_template(_CESIUM_HTML)' in src and '.replace(' not in src
    fill = inspect.getsource(CesiumMapWidget._fill_cesium_template)
    assert 'open(' not in fill and 'web_view' not in fill and 'setUrl' not in fill, '純字串處理'


def _js_literal_scene_offenders(source: str) -> list[int]:
    """AST：送進 _js(...) 的字串字面量（含 f-string）若直接動 viewer.*，就得自己要求算圖。"""
    bad = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == '_js' and node.args:
            seg = ast.get_source_segment(source, node.args[0]) or ''
            if isinstance(node.args[0], (ast.Constant, ast.JoinedStr)) and 'viewer.' in seg \
                    and '_requestRender' not in seg:
                bad.append(node.lineno)
    return bad


@needs_qt
def test_inline_entity_removal_scripts_request_a_render():
    assert _js_literal_scene_offenders("self._js('if(x){viewer.entities.remove(x);x=null;}')") == [1], '負對照'
    assert _js_literal_scene_offenders(WIDGET_PY.read_text(encoding='utf-8')) == []
    assert CesiumMapWidget._JS_RENDER == "if(typeof _requestRender==='function')_requestRender();"
    for k in ('_JS_CLEAR_CIRCLE', '_JS_CLEAR_HOME'):
        assert getattr(CesiumMapWidget, k).endswith(CesiumMapWidget._JS_RENDER), k
    w, sent, _ = _bare()
    w.__dict__.update(_circle_center=(25.0, 121.0), _circle_radius_m=100.0, _home_point=(25.0, 121.0))
    CesiumMapWidget.clear_circle_overlay(w)
    CesiumMapWidget.clear_home_point_overlay(w)
    assert sent == [CesiumMapWidget._JS_CLEAR_CIRCLE, CesiumMapWidget._JS_CLEAR_HOME]
    assert 'viewer.entities.remove(circleEntity)' in sent[0] and 'viewer.entities.remove(homeEntity)' in sent[1]
    clear_all = inspect.getsource(CesiumMapWidget.clear_all)
    assert 'self._js(self._JS_CLEAR_CIRCLE)' in clear_all and 'self._js(self._JS_CLEAR_HOME)' in clear_all


# ═══════════════════════════════════════════════════════════════════════
#  H. 真的 Cesium（本機 assets/cesium；沒有就 skip）
# ═══════════════════════════════════════════════════════════════════════
@pytest.fixture(scope="module")
def page3d(_pw, tmp_path_factory):
    if not _QT_OK:
        pytest.skip('Qt 匯入失敗')
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    with pytest.MonkeyPatch.context() as mp:
        vals = _template_values(mp)
    # GLB 留空（只測取樣與姿態；模型載入非同步、和這裡無關）；初始視角放在測試點
    vals.update({'__PLANE_MODEL_URI__': '', '__COPTER_MODEL_URI__': '', '__VTOL_MODEL_URI__': '',
                 '__INIT_LAT__': str(LAT0), '__INIT_LON__': str(LON0)})
    f = tmp_path_factory.mktemp("uavbatch") / "main.html"
    f.write_text(_NoDel.__new__(_NoDel)._fill_cesium_template(HTML, vals), encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
        pg = b.new_page(viewport={"width": 640, "height": 480})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())
        pg.goto(f.as_uri())
        pg.wait_for_function("typeof viewer !== 'undefined' && typeof updateUAVPositions === 'function'", timeout=60000)
        pg.add_script_tag(content=_PAGE_HELPERS)
    except Exception as e:                                   # pragma: no cover - 沒有 WebGL 等
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._errs = errs
    yield pg
    b.close()


@pytest.fixture
def page(page3d):
    # 機鼻旗標回到**頁面宣告的**預設（不是寫死 false：寫死會把「預設被改回 true」的突變在第一個測試之後蓋掉）
    if not hasattr(page3d, '_nose0'):
        page3d._nose0 = page3d.evaluate("_UAV_NOSE_FOLLOWS_TRACK")
    yield page3d
    page3d.evaluate("(v) => { clearUAV(); _RENDER_IDLE_MS = 2000; _UAV_NOSE_FOLLOWS_TRACK = v; }", page3d._nose0)
    assert not page3d._errs, page3d._errs


_PAGE_HELPERS = f"""
window.T_ = {{
  sleep: (ms) => new Promise(r => setTimeout(r, ms)),
  // JulianDate → epoch ms：toDate() 可能把 249.9999 ms 截成 249；改用和近期基準（2023 年後沒有閏秒）的秒差
  E: Cesium.JulianDate.fromDate(new Date(1700000000000)),
  ms: (jd) => 1700000000000 + Math.round(Cesium.JulianDate.secondsDifference(jd, T_.E) * 1000),
  // lat 每步往北 0.0001°（≈ 11 m）的一筆
  item: (sid, k, extra) => Object.assign({{lat: {LAT0} + 0.0001 * k, lon: {LON0}, alt: 100, hdg: 90, spd: 18, sysid: sid,
                                           mode: 'AUTO', armed: true, vt: 'VTOL', pitch: 0, roll: 0, crs: 135}}, extra || {{}}),
  posTimes: (sid) => _UAV_STATE[sid].props.pos._property._times.map(T_.ms),
  attTimes: (sid) => _UAV_STATE[sid].props.att._times.map(T_.ms),
  oriTimes: (sid) => _UAV_STATE[sid].props.ori._times.map(T_.ms),
  posAt: (sid, ms) => _UAV_STATE[sid].props.pos.getValue(Cesium.JulianDate.fromDate(new Date(ms))),
  shown: (sid) => uavEntities[sid].position.getValue(Cesium.JulianDate.now()),
  dist: (a, b) => Cesium.Cartesian3.distance(a, b),
  deg: (rad) => ((Cesium.Math.toDegrees(rad) % 360) + 360) % 360,
}};
"""


def test_real_cesium_resends_add_nothing_close_samples_replace_and_times_are_the_link_timestamps(page):
    r = page.evaluate("""async () => {
      // pt = base, base+50, …, base+300（= 開始時 − 50 ms：顯示時間 ≈ now − 120 ms 有 ~70 ms 餘裕落在最新一筆之前；
      // 之後 +3 ms 的取代樣本也還在過去）
      const sid = 11, out = {}, base = Date.now() - 350;
      for (let k = 0; k <= 6; k++) {
        updateUAVPositions([T_.item(sid, k, {pt: base + 50 * k, at: base + 50 * k + 5})]);
        const t = T_.posTimes(sid);
        if (t[t.length - 1] !== base + 50 * k) out.badNewest = [k, t[t.length - 1], base + 50 * k];
      }
      out.pos0 = T_.posTimes(sid); out.att0 = T_.attTimes(sid); out.ori0 = T_.oriTimes(sid);
      out.sent = [0, 1, 2, 3, 4, 5, 6].map(k => base + 50 * k);
      // 落在兩筆之間：顯示時間 ≈ now − 0.12 s
      // ⚠ td（Date.now）與 shown（JulianDate.now）都是毫秒粒度：兩次呼叫跨過毫秒邊界 ⇒ frac 差 1/50 = 0.02，
      //   正好壓在容差上（2026-10-21 改動前的 main.html 也 1/6 失敗、值 0.647998 vs 0.668）⇒ 重取到同一毫秒為止。
      const st = _UAV_STATE[sid];
      let td, p, t0;
      do { t0 = Date.now(); td = _uavDisplayMs(st); p = T_.shown(sid); } while (Date.now() !== t0);
      const ts = out.pos0, j = ts.findIndex(t => t > td);
      out.delay = st.delay; out.j = j;
      if (j > 0) {
        const A = T_.posAt(sid, ts[j - 1]), B = T_.posAt(sid, ts[j]);
        out.dA = T_.dist(A, p); out.dB = T_.dist(p, B); out.dAB = T_.dist(A, B);
        out.frac = (td - ts[j - 1]) / (ts[j] - ts[j - 1]);
      }
      // 同一個 pt 重送（GUI 合幀重送同一則位置）：位置換了也不加樣本、不動最後一筆
      const before = T_.posAt(sid, base + 300);
      updateUAVPositions([T_.item(sid, 99, {pt: base + 300, at: base + 305})]);
      out.resendPos = T_.posTimes(sid).length; out.resendAtt = T_.attTimes(sid).length;
      out.resendMoved = T_.dist(before, T_.posAt(sid, base + 300));
      out.lastPosMs = st.lastPosMs;
      // 3 ms 後的一筆：取代上一筆（不會出現相隔 3 ms 的兩點）
      updateUAVPositions([T_.item(sid, 7, {pt: base + 303, at: base + 308})]);
      out.rep = T_.posTimes(sid); out.repAtt = T_.attTimes(sid);
      // 時間戳不可信（太舊／太新／非數字）⇒ 用現在
      const now1 = Date.now();
      updateUAVPositions([T_.item(sid, 8, {pt: now1 - 60000, at: 'x'})]);
      out.staleUsedNow = T_.posTimes(sid).slice(-1)[0] - now1;
      out.staleAttUsedNow = T_.attTimes(sid).slice(-1)[0] - now1;
      await T_.sleep(5);                     // 同一毫秒的兩次「用現在」⇒ 時間戳不比上一筆新 ⇒ 不加（正確行為）
      const prevLast = T_.posTimes(sid).slice(-1)[0];
      updateUAVPositions([T_.item(sid, 9, {pt: Date.now() + 60000})]);   // 太新 ⇒ 也用現在
      out.futureAdded = T_.posTimes(sid).slice(-1)[0] > prevLast;
      out.futureUsedNow = T_.posTimes(sid).slice(-1)[0] - Date.now();
      out.nPos = st.nPos; out.nAtt = st.nAtt;
      return out;
    }""")
    assert 'badNewest' not in r, f"最新樣本時間要等於送進去的 pt：{r.get('badNewest')}"
    assert r["pos0"] == r["sent"] and len(r["att0"]) == 7 and len(r["ori0"]) == 7, '沒有種子樣本；時間就是 pt／at'
    assert r["att0"] == [t + 5 for t in r["sent"]]
    assert 0.11 < r["delay"] < 0.135
    assert r["j"] > 0 and r["dA"] > 0.5 and r["dB"] > 0.5 and abs(r["dA"] + r["dB"] - r["dAB"]) < 0.01, \
        f"顯示位置要嚴格落在兩筆之間（線性）：{r}"
    assert r["frac"] == pytest.approx(r["dA"] / r["dAB"], abs=0.02)
    assert r["resendPos"] == 7 and r["resendAtt"] == 7 and r["resendMoved"] == 0 and r["lastPosMs"] == r["sent"][-1]
    assert r["rep"] == r["sent"][:-1] + [r["sent"][-1] + 3], '3 ms 後的樣本取代上一筆'
    assert r["repAtt"][-1] == r["sent"][-1] + 8 and len(r["repAtt"]) == 7
    assert 0 <= r["staleUsedNow"] < 50 and 0 <= r["staleAttUsedNow"] < 50, '太舊／非數字的時間戳 ⇒ 用現在'
    assert r["futureAdded"] is True and -50 < r["futureUsedNow"] <= 0, '比現在晚 1 s 以上的時間戳 ⇒ 用現在'
    assert r["nPos"] == 10 and r["nAtt"] == 10, '7 筆＋取代 1＋太舊 1＋太新 1；重送那次不算'


def test_real_cesium_stall_holds_the_last_sample_and_never_moves_backwards(page):
    r = page.evaluate("""async () => {
      const sid = 12, base = Date.now() - 300;
      for (let k = 0; k <= 6; k++) updateUAVPositions([T_.item(sid, k, {pt: base + 50 * k, at: base + 50 * k})]);
      const first = T_.posAt(sid, base), last = T_.posAt(sid, base + 300), prog = [];
      for (let i = 0; i < 12; i++) { prog.push(T_.dist(first, T_.shown(sid))); await T_.sleep(125); }   // 1.5 s 不送
      const end = T_.shown(sid);
      return {prog, endToLast: T_.dist(end, last), total: T_.dist(first, last)};
    }""")
    prog = r["prog"]
    assert all(b >= a - 1e-6 for a, b in zip(prog, prog[1:])), f'斷訊時不可以往回走：{prog}'
    assert prog[0] < r["total"] - 1.0, '一開始還在兩筆之間（顯示延遲）'
    assert r["endToLast"] < 1e-6, '斷訊 1.5 s 後停在最後一筆（HOLD，不外推、不回彈）'


def test_real_cesium_display_delay_converges_to_twice_the_interval_and_display_time_never_goes_back(page):
    r = page.evaluate("""async () => {
      const sid = 13, disp = [], st0 = {};
      for (let k = 0; k < 30; k++) {                                   // 實時 100 ms 一筆、持續 3 s
        updateUAVPositions([T_.item(sid, k, {pt: Date.now(), at: Date.now()})]);
        const st = _UAV_STATE[sid];
        disp.push(_uavDisplayMs(st));
        await T_.sleep(100);
      }
      const st = _UAV_STATE[sid];
      return {disp, delay: st.delay, dtEma: st.dtEma, target: _uavDelayTarget(st.dtEma),
              t001: _uavDelayTarget(0.01), t005: _uavDelayTarget(0.05), t06: _uavDelayTarget(0.6)};
    }""")
    assert 0.09 < r["dtEma"] < 0.13
    assert r["delay"] == pytest.approx(r["target"], abs=0.01) and 0.08 <= r["delay"] <= 1.0, r
    assert r["target"] == pytest.approx(2 * r["dtEma"] + 0.03)
    d = r["disp"]
    assert all(b > a for a, b in zip(d, d[1:])), '顯示時間只能往前（延遲每秒最多改 0.2 s）'
    assert r["t001"] == 0.08 and r["t005"] == pytest.approx(0.13) and r["t06"] == 1.0


def test_real_cesium_track_blend_only_when_opted_in_and_the_decoded_attitudes_follow_the_nose(page):
    r = page.evaluate("""async () => {
      const f = (h, c, s, vt, m) => _uavNoseHeading(h, c, s, vt, m);
      // 預設（2026-10-22 起）：不看航跡，機鼻 = 飛控 yaw
      const out = {flag0: _UAV_NOSE_FOLLOWS_TRACK, def8: f(90, 135, 8, 'VTOL', 'AUTO'), defPlane: f(90, 135, 20, 'PLANE', 'CRUISE')};
      updateUAVPositions([T_.item(23, 0, {pt: Date.now(), at: Date.now()})]);
      out.defStHeading = _UAV_STATE[23].heading;
      // 明確打開顯示輔助
      out.on = setUavNoseFollowsTrack(true);
      Object.assign(out, {
        copter: f(90, 135, 8, 'COPTER', 'AUTO'), qmode: f(90, 135, 8, 'VTOL', 'QLOITER'),
        v2: f(90, 135, 2, 'VTOL', 'AUTO'), v45: f(90, 135, 4.5, 'VTOL', 'AUTO'), v8: f(90, 135, 8, 'VTOL', 'AUTO'),
        plane8: f(90, 135, 8, 'PLANE', 'CRUISE'), wrap: f(350, 10, 4.5, 'VTOL', 'AUTO'), wrapFull: f(350, 10, 9, 'VTOL', 'AUTO'),
        noCrs: f(90, undefined, 8, 'VTOL', 'AUTO'), nanCrs: f(90, NaN, 8, 'VTOL', 'AUTO'),
      });
      // 經過整條更新路徑：單一樣本（HOLD）⇒ 解出來的姿態就是那一筆
      updateUAVPositions([T_.item(21, 0, {pt: Date.now(), at: Date.now()})]);
      const st = _UAV_STATE[21], e = uavEntities[21], now = Cesium.JulianDate.now();
      out.stHeading = st.heading;
      out.attHeading = T_.deg(_uavSmoothHpr(st).heading);
      const pos = e.position.getValue(now), q = e.orientation.getValue(now);
      const m = Cesium.Matrix4.fromTranslationQuaternionRotationScale(pos, q, new Cesium.Cartesian3(1, 1, 1));
      out.oriHeading = T_.deg(Cesium.Transforms.fixedFrameToHeadingPitchRoll(m).heading);
      out.billboard = T_.deg(-e.billboard.rotation.getValue(now));
      updateUAVPositions([T_.item(22, 0, {pt: Date.now(), at: Date.now(), vt: 'COPTER'})]);
      out.copterHeading = _UAV_STATE[22].heading;
      out.off = setUavNoseFollowsTrack(false);
      out.flagOff = f(90, 135, 8, 'VTOL', 'AUTO');
      return out;
    }""")
    assert r["flag0"] is False and r["def8"] == 90 and r["defPlane"] == 90, '預設：機鼻 = 飛控 yaw，不看航跡'
    assert r["defStHeading"] == 90, '預設經過整條更新路徑：VTOL、AUTO、18 m/s、crs 135 ⇒ 機鼻仍是 hdg 90'
    assert r["on"] is True and r["off"] is False and r["flagOff"] == 90
    assert r["copter"] == 90 and r["qmode"] == 90 and r["v2"] == 90 and r["noCrs"] == 90 and r["nanCrs"] == 90
    assert r["v45"] == pytest.approx(112.5) and r["v8"] == pytest.approx(135) and r["plane8"] == pytest.approx(135)
    assert min(r["wrap"], 360 - r["wrap"]) < 1e-9, f'350°→10° 的中點要是 0°，不是 180°：{r["wrap"]}'
    assert r["wrapFull"] == pytest.approx(10)
    assert r["stHeading"] == pytest.approx(135)
    assert r["attHeading"] == pytest.approx(135, abs=0.01), '相機用的姿態四元數解回來要是機鼻方向'
    assert r["oriHeading"] == pytest.approx(135, abs=0.01), '模型姿態（沒有 GLB ⇒ 偏移 0）解回來要是機鼻方向'
    assert r["billboard"] == pytest.approx(135, abs=1e-6)
    assert r["copterHeading"] == 90, '多旋翼機鼻用 hdg'


# 模型四元數 → 3-2-1（航空）heading／pitch／roll：校正後機體座標（機鼻 +y、右翼 +x、機背 +z）的三軸換到 ENU 再解。
# 測試頁沒有 GLB ⇒ 校正 0 ⇒ entity 的方向就是校正後機體座標。與 Cesium 自己的 HPR 慣例（x 前）無關。
_DECODE_321_JS = """
  const _dec321 = (q, pos) => {
    const M = Cesium.Matrix3.fromQuaternion(q);
    const enuInv = Cesium.Matrix4.inverseTransformation(Cesium.Transforms.eastNorthUpToFixedFrame(pos), new Cesium.Matrix4());
    const ax = (x, y, z) => Cesium.Matrix4.multiplyByPointAsVector(enuInv,
      Cesium.Matrix3.multiplyByVector(M, new Cesium.Cartesian3(x, y, z), new Cesium.Cartesian3()), new Cesium.Cartesian3());
    const n = ax(0, 1, 0), r = ax(1, 0, 0), u = ax(0, 0, 1), D = Cesium.Math.toDegrees;
    return {hdg: ((D(Math.atan2(n.x, n.y)) % 360) + 360) % 360, pitch: D(Math.asin(n.z)), roll: D(Math.atan2(-r.z, u.z))};
  };
"""


@needs_qt
def test_real_cesium_vtol_model_attitude_is_the_fc_attitude_through_the_whole_chain(page):
    """使用者 2026-10-22：「模型要忠實反映 SITL 的姿態」。AUTO 中的 VTOL、20 m/s、航跡與 yaw 差 30°（側風蟹行）：
    TelemetryFrame → MainWindow._uav_batch_item → CesiumMapWidget.update_uav_positions（四捨五入＋JSON）→ 真的頁面
    ⇒ 模型以 3-2-1 解回來 heading = yaw（不是航跡）、pitch／roll = 飛控送的；相機姿態、疊圖、平面圖示同一個 yaw。
    舊預設（機鼻 3→6 m/s 過渡到航跡）這裡會解出 240°。"""
    import math
    import time
    from mission.sitl_link import TelemetryFrame
    from ui.main_window import MainWindow
    yaw, crs, spd = -150.0, 240.0, 20.0                     # ATTITUDE yaw −150° = 210°；航跡 240°
    t = time.time()
    f = TelemetryFrame(sysid=1, lat=LAT0, lon=LON0, alt_rel=120.0, gps_fix=3, mode='AUTO', armed=True,
                       vehicle_type='VTOL', yaw=yaw, heading=210.0, pitch=8.5, roll=-32.25, ground_speed=spd,
                       vel_n=spd * math.cos(math.radians(crs)), vel_e=spd * math.sin(math.radians(crs)), pos_t=t, att_t=t)
    w, sent, _ = _bare()
    w.update_uav_positions([MainWindow._uav_batch_item(f)])
    payload = _payload(sent[0])
    assert payload[0]['hdg'] == 210.0 and payload[0]['crs'] == pytest.approx(240.0)
    assert payload[0]['pitch'] == 8.5 and payload[0]['roll'] == -32.25 and payload[0]['at'] == int(round(t * 1000))
    r = page.evaluate("(payload) => {" + _DECODE_321_JS + """
      updateUAVPositions(payload);
      const sid = payload[0].sysid, st = _UAV_STATE[sid], e = uavEntities[sid], now = Cesium.JulianDate.now();
      const out = {model: _dec321(e.orientation.getValue(now), e.position.getValue(now)), nOri: T_.oriTimes(sid).length};
      const hpr = _uavSmoothHpr(st);
      out.cam = {hdg: T_.deg(hpr.heading), pitch: Cesium.Math.toDegrees(hpr.pitch), roll: Cesium.Math.toDegrees(hpr.roll)};
      out.shown = _uavShownPose(st).heading; out.stHeading = st.heading;
      out.billboard = T_.deg(-e.billboard.rotation.getValue(now));
      out.status = document.getElementById('sitlStatus').textContent;
      // 對照：明確打開顯示輔助 ⇒ 同一筆的機鼻改指航跡（證明上面不是因為 crs 沒送到）
      setUavNoseFollowsTrack(true);
      updateUAVPositions([Object.assign({}, payload[0], {sysid: sid + 1})]);
      const e2 = uavEntities[sid + 1];
      out.optIn = _dec321(e2.orientation.getValue(now), e2.position.getValue(now));
      setUavNoseFollowsTrack(false);
      return out;
    }""", payload)
    m = r["model"]
    assert r["nOri"] == 1, '單一樣本（HOLD）⇒ 畫的就是這一筆'
    assert m["hdg"] == pytest.approx(210.0, abs=1e-6), f'模型機鼻要是 ATTITUDE yaw 210°，不是航跡 240°：{m}'
    assert m["pitch"] == pytest.approx(8.5, abs=1e-6) and m["roll"] == pytest.approx(-32.25, abs=1e-6), m
    assert r["cam"] == pytest.approx({"hdg": 210.0, "pitch": 8.5, "roll": -32.25}, abs=1e-6), '追機／FPV 用的姿態'
    assert r["shown"] == pytest.approx(210.0, abs=1e-6) and r["stHeading"] == 210.0, '疊圖（光錐／RCS）用的機鼻'
    assert r["billboard"] == pytest.approx(210.0, abs=1e-6)
    assert 'hdg:210°' in r["status"] and 'crs:240°' in r["status"] and 'roll:-32.3°' in r["status"], r["status"]
    o = r["optIn"]
    assert o["hdg"] == pytest.approx(240.0, abs=1e-6) and o["pitch"] == pytest.approx(8.5, abs=1e-6) \
        and o["roll"] == pytest.approx(-32.25, abs=1e-6), f'明確打開才跟航跡：{o}'


def test_real_cesium_attitude_interpolation_wraps_through_north(page):
    """兩筆 yaw 350° → 10°（相隔 100 ms）：模型與相機的姿態插值走最短的 20°（中點 0°），不是繞 340°（中點 180°）。"""
    r = page.evaluate("() => {" + _DECODE_321_JS + """
      const sid = 24, base = Date.now() - 400, out = {};
      updateUAVPositions([T_.item(sid, 0, {hdg: 350, pt: base, at: base})]);
      updateUAVPositions([T_.item(sid, 0, {hdg: 10, pt: base + 100, at: base + 100})]);
      const st = _UAV_STATE[sid], pos = T_.posAt(sid, base);
      for (const k of [25, 50, 75]) {
        const jd = Cesium.JulianDate.fromDate(new Date(base + k));
        out['m' + k] = _dec321(st.props.ori.getValue(jd), pos).hdg;
        out['c' + k] = T_.deg(Cesium.HeadingPitchRoll.fromQuaternion(st.props.att.getValue(jd)).heading);
      }
      return out;
    }""")
    for k, want in ((25, 355.0), (50, 0.0), (75, 5.0)):
        for kind in ('m', 'c'):
            got = r[f'{kind}{k}']
            assert min(abs(got - want), 360 - abs(got - want)) < 1e-3, (kind, k, got, want)


def _att50(n: int) -> list:
    """50 Hz 的姿態（相對 ms、roll、pitch、機頭）：一直在變、跨北。"""
    return [[20 * k, round(10 * math.sin(0.7 * k), 2), round(-5 + 0.25 * k, 2), round((350 + 1.5 * k) % 360, 2)]
            for k in range(n)]


def test_real_cesium_every_attitude_sample_of_a_50hz_stream_reaches_the_model_through_30hz_batches(page):
    """2026-10-22：50 Hz 姿態、33 ms 一批。以前每批只送最新一筆 ⇒ 約四成樣本沒進頁面；現在比主樣本舊的跟著 atts 一起送：
    每一筆都以**自己的時間**、自己的姿態進模型與相機的姿態屬性（逐筆 3-2-1 解回來比），一筆不少、一筆不多。"""
    att = _att50(40)
    batches, last = [], -1                       # 每 33 ms 一批：主樣本 = 當下最新一筆、atts = 上一批之後更舊的
    for tb in range(33, 20 * 40, 33):
        idx = [i for i, a in enumerate(att) if a[0] <= tb and i > last]
        if idx:
            batches.append({'main': idx[-1], 'older': idx[:-1]})
            last = idx[-1]
    assert sum(len(b['older']) for b in batches) > 10, '這個節奏下真的有被合掉的樣本'
    r = page.evaluate("([att, batches]) => {" + _DECODE_321_JS + """
      const sid = 31, base = Date.now() - 1200, out = {dec: [], cam: []};
      for (const b of batches) {
        const m = att[b.main];
        updateUAVPositions([T_.item(sid, 0, {pt: base + m[0], at: base + m[0], roll: m[1], pitch: m[2], hdg: m[3],
                                            atts: b.older.map(i => [base + att[i][0], att[i][1], att[i][2], att[i][3]])})]);
      }
      const st = _UAV_STATE[sid], pos = T_.posAt(sid, base + att[batches[0].main][0]);
      out.att = T_.attTimes(sid).map(t => t - base); out.ori = T_.oriTimes(sid).map(t => t - base); out.nAtt = st.nAtt;
      for (const a of att) {
        const jd = Cesium.JulianDate.fromDate(new Date(base + a[0]));
        const q = st.props.ori.getValue(jd);
        out.dec.push(q ? _dec321(q, pos) : null);
        const h = Cesium.HeadingPitchRoll.fromQuaternion(st.props.att.getValue(jd));
        out.cam.push({hdg: T_.deg(h.heading), pitch: Cesium.Math.toDegrees(h.pitch), roll: Cesium.Math.toDegrees(h.roll)});
      }
      // 整批重送（同一個 at、同一串 atts）⇒ 一筆都不加
      const lb = batches[batches.length - 1], m = att[lb.main];
      updateUAVPositions([T_.item(sid, 0, {pt: base + m[0], at: base + m[0], roll: m[1], pitch: m[2], hdg: m[3],
                                          atts: lb.older.map(i => [base + att[i][0], att[i][1], att[i][2], att[i][3]])})]);
      out.nAttAfterResend = st.nAtt; out.lenAfterResend = T_.attTimes(sid).length;
      return out;
    }""", [att, batches])
    first = batches[0]['older'][0] if batches[0]['older'] else batches[0]['main']
    want_t = [a[0] for a in att[first:batches[-1]['main'] + 1]]
    assert r['att'] == want_t and r['ori'] == want_t, '每一筆都在、時間就是它自己的取樣時間'
    assert r['nAtt'] == len(want_t) and r['nAttAfterResend'] == r['nAtt'] and r['lenAfterResend'] == len(want_t)
    for a, d, c in zip(att[first:], r['dec'][first:], r['cam'][first:]):
        if a[0] > want_t[-1]:
            break
        want = {'hdg': a[3], 'pitch': a[2], 'roll': a[1]}
        for k in ('pitch', 'roll'):
            assert d[k] == pytest.approx(want[k], abs=1e-6) and c[k] == pytest.approx(want[k], abs=1e-6), (a, d, c)
        for got in (d['hdg'], c['hdg']):
            assert min(abs(got - a[3]), 360 - abs(got - a[3])) < 1e-6, (a, d, c)


def test_real_cesium_older_samples_skip_bad_times_never_steal_now_and_still_replace_within_ten_ms(page):
    r = page.evaluate("""() => {
      const sid = 32, base = Date.now() - 600, out = {};
      updateUAVPositions([T_.item(sid, 0, {pt: base, at: base})]);
      updateUAVPositions([T_.item(sid, 0, {pt: base + 100, at: base + 100, atts: [
        [base + 50, 1, 2, 3],            // 加
        ['x', 1, 2, 3],                  // 時間不是數字 ⇒ 略過（不退回現在）
        [Date.now() + 60000, 1, 2, 3],   // 太新、不可信 ⇒ 略過
        [base + 55, 4, 5, 6],            // 離上一筆 5 ms ⇒ 取代 base+50
        [base + 100, 7, 8, 9],           // 不比主樣本舊 ⇒ 略過（主樣本自己加）
        [base + 150, 7, 8, 9],           // 比主樣本新 ⇒ 略過
        [base + 30, 7, 8, 9],            // 比已加的舊 ⇒ 不加
        null, [base + 70], 'abcd']}),    // 形狀不對 ⇒ 略過
      ]);
      out.t1 = T_.attTimes(sid).map(t => t - base); out.n1 = _UAV_STATE[sid].nAtt;
      // 主樣本時間不可信 ⇒ 舊樣本照加、主樣本退回現在（舊樣本不能佔掉「現在」）
      const now0 = Date.now();
      updateUAVPositions([T_.item(sid, 0, {pt: base + 200, at: 'bad',
                                          atts: [[base + 160, 1, 1, 1], ['x', 9, 9, 9], [base + 180, 2, 2, 2]]})]);
      const t2 = T_.attTimes(sid);
      out.t2 = t2.slice(0, -1).map(t => t - base); out.lastIsNow = t2[t2.length - 1] - now0; out.n2 = _UAV_STATE[sid].nAtt;
      return out;
    }""")
    assert r['t1'] == [0, 55, 100], r
    assert r['n1'] == 4, '0、50、55（取代也算一次加）、100'
    assert r['t2'] == [0, 55, 100, 160, 180] and 0 <= r['lastIsNow'] < 50 and r['n2'] == 7, r


def test_real_cesium_vehicle_type_change_rebuilds_the_model_with_its_own_calibration(page):
    """第一批還沒收到 HEARTBEAT（vt '---' ⇒ 多旋翼）、之後變 PLANE／VTOL：模型要跟著換，模型姿態樣本裡的舊校正要丟掉。
    以前模型在建 entity 時定死、校正每筆照新機型算 ⇒ GLB 頁上是多旋翼模型套固定翼校正（0/0/0）＝整架躺著。
    （測試頁沒有 GLB ⇒ 這裡看的是退回的方塊／橢球；校正與模型種類出自同一組旗標，見 test_model_and_its_glb_calibration…）"""
    r = page.evaluate("""() => {
      const sid = 61, now = Date.now(), out = {};
      updateUAVPositions([T_.item(sid, 0, {vt: '---', pt: now - 300, at: now - 300, roll: 10})]);
      const e0 = uavEntities[sid];
      out.k0 = e0._uavKind; out.ell0 = !!e0.ellipsoid; out.box0 = !!e0.box;
      updateUAVPositions([T_.item(sid, 1, {vt: 'PLANE', pt: now - 250, at: now - 250, roll: 20})]);
      const e1 = uavEntities[sid];
      out.k1 = e1._uavKind; out.ell1 = !!e1.ellipsoid; out.box1 = !!e1.box; out.wing1 = !!e1._wingChild;
      out.rebuilt = e1 !== e0; out.oldGone = !viewer.entities.contains(e0); out.stKind = _UAV_STATE[sid].kind;
      out.nOri = T_.oriTimes(sid).length; out.nAtt = T_.attTimes(sid).length;
      updateUAVPositions([T_.item(sid, 2, {vt: 'PLANE', pt: now - 200, at: now - 200})]);
      out.same = uavEntities[sid] === e1; out.nOri2 = T_.oriTimes(sid).length;
      updateUAVPositions([T_.item(sid, 3, {vt: 'COPTER', pt: now - 150, at: now - 150})]);
      const e2 = uavEntities[sid];
      out.k2 = e2._uavKind; out.wingGone = !viewer.entities.contains(e1._wingChild) && !viewer.entities.contains(e1);
      out.legacySysid1 = (uavEntity === null || uavEntity === undefined || uavEntity === uavEntities[1]);
      return out;
    }""")
    assert r["k0"] == 'copterBall' and r["ell0"] and not r["box0"]
    assert r["k1"] == 'planeBox' and r["box1"] and not r["ell1"] and r["wing1"], f'機型變了模型要跟著換：{r}'
    assert r["rebuilt"] and r["oldGone"] and r["stKind"] == 'planeBox'
    assert r["nOri"] == 1 and r["nAtt"] == 2, '舊校正的模型姿態樣本丟掉；相機用的純姿態樣本保留'
    assert r["same"] and r["nOri2"] == 2, '同一機型不重建'
    assert r["k2"] == 'copterBall' and r["wingGone"], '舊的機翼子 entity 也要移掉'
    assert r["legacySysid1"]


def test_real_cesium_hidden_aircraft_layer_also_hides_new_and_rebuilt_models(page):
    """圖層面板「✈ 飛機」關著：toggleLayer 只改**已經存在**的 entity ⇒ 第一次出現的、機型變了重建的模型
    （含固定翼退回方塊的機翼子 entity）以前都會冒出來（2026-10-22 審查）。"""
    r = page.evaluate("""() => {
      const box = document.getElementById('lyr_aircraft'), out = {};
      try {
        box.checked = false; toggleLayer('aircraft');
        const sid = 71, now = Date.now();
        updateUAVPositions([T_.item(sid, 0, {vt: 'COPTER', pt: now - 300, at: now - 300})]);
        out.first = uavEntities[sid].show;
        updateUAVPositions([T_.item(sid, 1, {vt: 'PLANE', pt: now - 250, at: now - 250})]);
        const e = uavEntities[sid];
        out.rebuilt = [e.show, !!e._wingChild && e._wingChild.show, e._uavKind];
        box.checked = true; toggleLayer('aircraft');
        out.back = [e.show, e._wingChild.show];
        updateUAVPositions([T_.item(sid + 1, 0, {vt: 'PLANE', pt: now - 200, at: now - 200})]);
        out.shownNew = [uavEntities[sid + 1].show, uavEntities[sid + 1]._wingChild.show];
      } finally { box.checked = true; toggleLayer('aircraft'); }
      return out;
    }""")
    assert r["first"] is False, '圖層關著時第一次出現的飛機也要藏著'
    assert r["rebuilt"] == [False, False, 'planeBox'], f'機型變了重建的模型（含機翼）也要藏著：{r}'
    assert r["back"] == [True, True] and r["shownNew"] == [True, True], '圖層打開 ⇒ 照常顯示'


def test_real_cesium_clock_render_mode_and_trail(page):
    r = page.evaluate("""async () => {
      const out = {};
      out.suspend = viewer.allowDataSourcesToSuspendAnimation;
      out.step = viewer.clock.clockStep === Cesium.ClockStep.SYSTEM_CLOCK_MULTIPLIER;
      out.noTimeline = viewer.timeline === undefined; out.noAnimation = viewer.animation === undefined;
      _RENDER_IDLE_MS = 300;
      await T_.sleep(1200);
      out.rrmIdle0 = viewer.scene.requestRenderMode;
      const base = Date.now() - 300;
      for (let k = 0; k <= 6; k++) updateUAVPositions([T_.item(31, k, {pt: base + 50 * k}), T_.item(32, k + 20, {pt: base + 50 * k})]);
      out.rrmBusy = viewer.scene.requestRenderMode;
      out.clock = {animate: viewer.clock.shouldAnimate, unbounded: viewer.clock.clockRange === Cesium.ClockRange.UNBOUNDED,
                   mult: viewer.clock.multiplier};
      out.n = Object.keys(uavEntities).length; out.trails = Object.keys(uavTrails).length;
      // 尾跡：顯示時間還沒走到的點留在待進區；斷訊後全部進來；沒新點時回同一個快取
      const cb = uavTrails[31].polyline.positions;
      out.trailNow = cb.getValue(Cesium.JulianDate.now()).length; out.pendNow = uavTrailPend[31].length;
      await T_.sleep(1200);
      out.rrmIdle = viewer.scene.requestRenderMode; out.mdtIdle = viewer.scene.maximumRenderTimeChange;
      const a = cb.getValue(Cesium.JulianDate.now()), b = cb.getValue(Cesium.JulianDate.now());
      out.pendAfter = uavTrailPend[31].length; out.trailAfter = a.length; out.same = (a === b);
      out.cart = a[0] instanceof Cesium.Cartesian3;
      // 打擊／俯衝動畫佔用時鐘（CLAMPED、播放中）⇒ 批次不碰；播完 ⇒ 收回，但**不改** currentTime
      const c = viewer.clock, past = Cesium.JulianDate.addHours(Cesium.JulianDate.now(), -3, new Cesium.JulianDate());
      c.currentTime = past.clone(); c.stopTime = Cesium.JulianDate.addHours(past, 1, new Cesium.JulianDate());
      c.clockRange = Cesium.ClockRange.CLAMPED; c.shouldAnimate = true;
      updateUAVPositions([T_.item(31, 7)]);
      out.animKept = c.clockRange === Cesium.ClockRange.CLAMPED;
      c.stopTime = Cesium.JulianDate.addSeconds(past, 1, new Cesium.JulianDate()); c.currentTime = c.stopTime.clone();
      updateUAVPositions([T_.item(31, 8)]);
      out.reclaimed = c.clockRange === Cesium.ClockRange.UNBOUNDED && c.clockStep === Cesium.ClockStep.SYSTEM_CLOCK_MULTIPLIER;
      out.hoursBehind = Cesium.JulianDate.secondsDifference(Cesium.JulianDate.now(), c.currentTime) / 3600;
      out.rrmAgain = viewer.scene.requestRenderMode;
      clearUAV();
      out.afterClear = [Object.keys(uavEntities).length, Object.keys(uavTrails).length, Object.keys(uavTrailCart).length,
                        Object.keys(uavTrailPend).length, Object.keys(_UAV_STATE).length];
      return out;
    }""")
    assert r["suspend"] is False and r["step"] is True and r["noTimeline"] and r["noAnimation"]
    assert r["rrmIdle0"] is True and r["rrmBusy"] is False, '有遙測 ⇒ 連續算圖'
    assert r["clock"] == {"animate": True, "unbounded": True, "mult": 1}
    assert r["n"] == 2 and r["trails"] == 2
    assert r["pendNow"] >= 2 and r["trailNow"] + r["pendNow"] == 7, f'顯示時間還沒到的點不可以先進尾跡：{r}'
    assert r["rrmIdle"] is True and r["mdtIdle"] == 0.05, '閒置 ⇒ 切回 requestRenderMode'
    assert r["pendAfter"] == 0 and r["trailAfter"] == 7 and r["same"] is True and r["cart"] is True
    assert r["animKept"] is True, '動畫播放中批次不碰時鐘'
    assert r["reclaimed"] is True and r["hoursBehind"] > 2.9, '收回時鐘但不改 currentTime（SYSTEM_CLOCK 會把它蓋成現在）'
    assert r["rrmAgain"] is False
    assert r["afterClear"] == [0, 0, 0, 0, 0]


# ══════════════════════════════════════════════════════════════════
#  I. 模型姿態：滾轉繞機鼻、俯仰繞翼展（2026-10-22 使用者：「SITL 做 roll 的時候模型只有偏航沒有滾轉」）
# ══════════════════════════════════════════════════════════════════
#   舊寫法 HPR(機鼻 + hOff, pitch + pOff, roll + rOff)：VTOL／多旋翼 GLB 校正 pOff = 90° ⇒ roll 繞垂直軸轉（看起來是偏航）；
#   偏移 0 的固定翼則是 Cesium 的 roll 繞「航向 0 時朝東」的 x 軸 ⇒ 滾轉變俯仰。校正都是「航向 0 時機鼻朝北」。
#   這裡只靠那個已經目視驗證過的事實（水平時機鼻＝航向）：相對於水平姿態的旋轉，軸與方向必須對。

_ATT_JS = """([h, p, r, lat, lon, useOld]) => {
  const pos = Cesium.Cartesian3.fromDegrees(lon, lat, 100);
  const enu = Cesium.Transforms.eastNorthUpToFixedFrame(pos);
  const vec = (x, y, z) => Cesium.Cartesian3.normalize(Cesium.Matrix4.multiplyByPointAsVector(
    enu, new Cesium.Cartesian3(x, y, z), new Cesium.Cartesian3()), new Cesium.Cartesian3());
  const E = vec(1, 0, 0), N = vec(0, 1, 0), U = vec(0, 0, 1), dot = Cesium.Cartesian3.dot;
  const old = (n, pp, rr) => Cesium.Transforms.headingPitchRollQuaternion(pos, new Cesium.HeadingPitchRoll(
    Cesium.Math.toRadians(n + h), Cesium.Math.toRadians(pp + p), Cesium.Math.toRadians(rr + r)));
  const O = useOld ? old : (n, pp, rr) => _uavModelOrientation(pos, n, pp, rr, h, p, r);
  // q = R·q0 的世界座標旋轉 R：角度、軸（對 E/N/U 的分量）、把向量轉到哪
  const rel = (q, q0) => {
    const d = Cesium.Quaternion.multiply(q, Cesium.Quaternion.inverse(q0, new Cesium.Quaternion()), new Cesium.Quaternion());
    const R = Cesium.Matrix3.fromQuaternion(d);
    const ax = Cesium.Quaternion.computeAxis(d, new Cesium.Cartesian3());
    const app = (v) => Cesium.Matrix3.multiplyByVector(R, v, new Cesium.Cartesian3());
    return {ang: Cesium.Math.toDegrees(Cesium.Quaternion.computeAngle(d)),
            axE: Math.abs(dot(ax, E)), axN: Math.abs(dot(ax, N)), axU: Math.abs(dot(ax, U)),
            eastUp: dot(app(E), U), northUp: dot(app(N), U), northE: dot(app(N), E)};
  };
  return {roll: rel(O(0, 0, 30), O(0, 0, 0)), pitch: rel(O(0, 20, 0), O(0, 0, 0)), yaw: rel(O(40, 0, 0), O(0, 0, 0)),
          roll90: rel(O(90, 0, 30), O(90, 0, 0)), pitch90: rel(O(90, 20, 0), O(90, 0, 0)),
          levelSame: Math.abs(Cesium.Quaternion.dot(O(0, 0, 0), old(0, 0, 0))),
          level137Same: Math.abs(Cesium.Quaternion.dot(O(137, 0, 0), old(137, 0, 0)))};
}"""


@pytest.mark.parametrize("offs", [(0, 0, 0), (-90, 90, 180), (-90, 0, 0)],
                         ids=["plane-glb-no-offset", "vtol-and-copter-glb", "heading-offset-only"])
def test_real_cesium_model_rolls_about_the_nose_and_pitches_about_the_wing(page, offs):
    r = page.evaluate(_ATT_JS, [*offs, LAT0, LON0, False])
    assert r["levelSame"] > 1 - 1e-9 and r["level137Same"] > 1 - 1e-9, '水平時必須和舊寫法完全相同（保住目視校正）'
    roll, pitch, yaw = r["roll"], r["pitch"], r["yaw"]
    assert roll["ang"] == pytest.approx(30, abs=1e-6) and roll["axN"] > 1 - 1e-9, f'航向 0：滾轉軸要是北（機鼻）{roll}'
    assert roll["eastUp"] == pytest.approx(-0.5, abs=1e-9), '正滾轉＝右翼（東）下沉'
    assert pitch["ang"] == pytest.approx(20, abs=1e-6) and pitch["axE"] > 1 - 1e-9, f'俯仰軸要是翼展（東）{pitch}'
    assert pitch["northUp"] == pytest.approx(0.3420201, abs=1e-6), '正俯仰＝機鼻（北）上仰'
    assert yaw["ang"] == pytest.approx(40, abs=1e-6) and yaw["axU"] > 1 - 1e-9 and yaw["northE"] > 0.64, '航向繞垂直軸、順時針'
    r90, p90 = r["roll90"], r["pitch90"]
    assert r90["axE"] > 1 - 1e-9 and r90["northUp"] == pytest.approx(0.5, abs=1e-9), '航向 90：滾轉軸是東（機鼻），右翼（南）下沉'
    assert p90["axN"] > 1 - 1e-9 and p90["eastUp"] == pytest.approx(0.3420201, abs=1e-6), '航向 90：俯仰時機鼻（東）上仰'


@pytest.mark.parametrize("offs", [(0, 0, 0), (-90, 90, 180)], ids=["plane-glb-no-offset", "vtol-and-copter-glb"])
def test_the_attitude_check_catches_the_old_angle_addition(page, offs):
    """同一套檢查拿去量舊寫法必須不過（牙齒）：VTOL 校正下滾轉軸是垂直的、偏移 0 時滾轉軸是東。"""
    r = page.evaluate(_ATT_JS, [*offs, LAT0, LON0, True])
    assert r["roll"]["axN"] < 0.5, r["roll"]
    if offs[1] == 90:
        assert r["roll"]["axU"] > 1 - 1e-9, '舊寫法在 VTOL 校正下：滾轉＝繞垂直軸＝偏航（使用者看到的現象）'
    else:
        assert r["roll"]["axE"] > 1 - 1e-9, '舊寫法在偏移 0 時：滾轉繞東軸＝俯仰'


def _ref_axes_enu(hdg, pitch, roll):
    """航空 3-2-1（ArduPilot ZYX 歐拉角）的獨立參考：機體 FRD→NED 的 DCM C = Rz(ψ)·Ry(θ)·Rx(φ)，再換成 ENU。
    回傳 ENU 下的 (機鼻, 右翼, 機背)。"""
    import math
    ps, th, ph = (math.radians(v) for v in (hdg, pitch, roll))
    cps, sps, cth, sth, cph, sph = math.cos(ps), math.sin(ps), math.cos(th), math.sin(th), math.cos(ph), math.sin(ph)
    c = [[cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
         [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
         [-sth, sph * cth, cph * cth]]
    col = lambda j: (c[1][j], c[0][j], -c[2][j])              # NED → ENU
    nose, right, down = col(0), col(1), col(2)
    return nose, right, tuple(-v for v in down)


@pytest.mark.parametrize("offs", [(0, 0, 0), (-90, 90, 180)], ids=["plane-glb-no-offset", "vtol-and-copter-glb"])
@pytest.mark.parametrize("att", [(137, 15, 30), (225, -10, -45), (300, 40, 60)])
def test_real_cesium_combined_pitch_and_roll_match_the_3_2_1_reference(page, offs, att):
    """單軸測試分不出旋轉順序（審查實測：把 pitch、roll 對調的突變單軸全過，組合姿態卻差 7.7°～39°）⇒ 對照獨立的 3-2-1 DCM。"""
    got = page.evaluate("""([h, p, r, H, P, R, lat, lon]) => {
      const pos = Cesium.Cartesian3.fromDegrees(lon, lat, 100);
      const q = _uavModelOrientation(pos, H, P, R, h, p, r);
      const fix = Cesium.Quaternion.fromHeadingPitchRoll(new Cesium.HeadingPitchRoll(
        Cesium.Math.toRadians(h), Cesium.Math.toRadians(p), Cesium.Math.toRadians(r)));
      const body = Cesium.Quaternion.multiply(q, Cesium.Quaternion.inverse(fix, new Cesium.Quaternion()), new Cesium.Quaternion());
      const M = Cesium.Matrix3.fromQuaternion(body);
      const enuInv = Cesium.Matrix4.inverseTransformation(Cesium.Transforms.eastNorthUpToFixedFrame(pos), new Cesium.Matrix4());
      const ax = (x, y, z) => {        // 校正後機體座標：機鼻 +y、右翼 +x、機背 +z
        const w = Cesium.Matrix3.multiplyByVector(M, new Cesium.Cartesian3(x, y, z), new Cesium.Cartesian3());
        const e = Cesium.Matrix4.multiplyByPointAsVector(enuInv, w, new Cesium.Cartesian3());
        return [e.x, e.y, e.z];
      };
      return {nose: ax(0, 1, 0), right: ax(1, 0, 0), up: ax(0, 0, 1)};
    }""", [*offs, *att, LAT0, LON0])
    for name, ref in zip(("nose", "right", "up"), _ref_axes_enu(*att)):
        assert got[name] == pytest.approx(list(ref), abs=1e-9), (name, got[name], ref)


def test_update_uses_the_composed_model_orientation_and_never_adds_offsets_to_attitude_angles():
    body = _fn("_updateUAVOne")
    assert "_uavModelOrientation(pos, noseDeg, pitchDeg, rollDeg, headingOffset, pitchOffset, rollOffset)" in body
    for bad in ("(rollDeg || 0) + rollOffset", "(pitchDeg || 0) + pitchOffset", "noseDeg + headingOffset"):
        assert bad not in body, bad


# ══════════════════════════════════════════════════════════════════
#  J. FPV 相機：視線／上方向 = 機體姿態 × 雲台安裝旋轉（2026-10-22 審查：雲台俯角直接加進歐拉角）
# ══════════════════════════════════════════════════════════════════
#   舊寫法 setView(HPR(機鼻, 雲台 + pitch, roll))：Cesium 相機的 HPR 先繞視線滾轉 ⇒ 雲台 0° 時剛好對；
#   下視 −90° 時機體滾轉只讓影像旋轉、視線還是鉛垂（審查實測差 30.0°）；−45° 時差 21.1°。
#   參考 = 獨立的航空 3-2-1 DCM（_ref_axes_enu）× 雲台繞右翼俯仰 g（負＝往下）：
#     視線 = cos g·機鼻 + sin g·機背、上方向 = −sin g·機鼻 + cos g·機背、右 = 右翼（繞右翼轉 ⇒ 右不變）。
#   rollFollow=false（雲台穩定）⇒ 機體 pitch、roll 當 0（只跟航向＋雲台俯角）。
#   量的是相機軸在**飛機位置的 ENU** 下的分量（和模型同一個座標系；鏡頭在機頭前 2 m，那裡的 ENU 差 ≈ 3e-7 rad）。

_FPV_JS = """([H, P, R, g, follow, mode, ref]) => {
  const sid = 41, C = Cesium;
  clearUAV();                      // 只留這一筆樣本（HOLD）：連續呼叫時顯示時間會落在上一筆和這一筆之間、姿態被插值
  updateUAVPositions([T_.item(sid, 0, {hdg: H, crs: H, vt: 'COPTER', pitch: P, roll: R, pt: Date.now(), at: Date.now()})]);
  setFpvCamera(sid, g, 75, follow);
  const st = _UAV_STATE[sid], pos = _uavSmoothPos(st), hpr = _uavSmoothHpr(st);
  const enu = C.Transforms.eastNorthUpToFixedFrame(pos);
  const inv = C.Matrix4.inverseTransformation(enu, new C.Matrix4());
  const fwd = _fpvConfig.forwardOffset;
  const camPos = C.Cartesian3.add(pos, C.Matrix4.multiplyByPointAsVector(enu, new C.Cartesian3(
    Math.sin(hpr.heading) * fwd, Math.cos(hpr.heading) * fwd, _fpvConfig.upOffset), new C.Cartesian3()), new C.Cartesian3());
  const wc = (v) => C.Matrix4.multiplyByPointAsVector(enu, new C.Cartesian3(v[0], v[1], v[2]), new C.Cartesian3());
  if (mode === 'old') {            // 2026-10-22 以前的寫法（照抄）
    viewer.camera.setView({destination: camPos, orientation: {heading: hpr.heading,
      pitch: C.Math.toRadians(g) + (follow ? hpr.pitch : 0), roll: follow ? hpr.roll : 0}});
  } else if (mode === 'dirup') {   // 拿正確的視線／上方向走 setView 的 {direction, up} 形式
    viewer.camera.setView({destination: camPos, orientation: {direction: wc(ref[0]), up: wc(ref[1])}});
  } else {
    _fpvPreRender();
  }
  const c = viewer.camera;
  const loc = (w) => { const e = C.Matrix4.multiplyByPointAsVector(inv, w, new C.Cartesian3()); return [e.x, e.y, e.z]; };
  const out = {dir: loc(c.directionWC), up: loc(c.upWC), right: loc(c.rightWC),
               camOff: C.Cartesian3.distance(c.positionWC, camPos), heading: hpr.heading};
  setFpvCamera(0); setFpvCameraParams(0, 75, true, 2.0);
  return out;
}"""


def _ref_fpv_enu(hdg, pitch, roll, gimbal, follow):
    """(視線, 上方向, 右)：3-2-1 參考 × 雲台。follow=False ⇒ 雲台穩定（pitch、roll 不跟）。"""
    import math
    nose, right, up = _ref_axes_enu(hdg, pitch if follow else 0, roll if follow else 0)
    cg, sg = math.cos(math.radians(gimbal)), math.sin(math.radians(gimbal))
    return ([cg * n + sg * u for n, u in zip(nose, up)], [-sg * n + cg * u for n, u in zip(nose, up)], list(right))


def _ang_deg(a, b):
    """兩個單位向量的夾角（度）；atan2(|a×b|, a·b)，小角度不失真。"""
    import math
    cx = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    return math.degrees(math.atan2(math.sqrt(sum(v * v for v in cx)), sum(x * y for x, y in zip(a, b))))


# (航向, pitch, roll)：一般組合、負角、大角度；下視時離鉛垂 1°／2°（Cesium direction/up 換算會失真的範圍）、剛好鉛垂
_FPV_ATT = [(137, 15, 30), (225, -10, -45), (300, 40, 60), (40, 1, 0), (40, 0, 2), (0, 0, 0)]


@pytest.mark.parametrize("follow", [True, False], ids=["roll-follow", "stabilised"])
@pytest.mark.parametrize("gimbal", [0, -45, -90])
@pytest.mark.parametrize("att", _FPV_ATT)
def test_real_cesium_fpv_view_is_body_attitude_times_gimbal(page, att, gimbal, follow):
    d, u, r = _ref_fpv_enu(*att, gimbal, follow)
    got = page.evaluate(_FPV_JS, [*att, gimbal, follow, 'new', None])
    for name, ref in (("dir", d), ("up", u), ("right", r)):
        assert got[name] == pytest.approx(ref, abs=1e-9), (name, got[name], ref)
    assert got["camOff"] < 1e-6, '鏡頭位置不變（機頭前 2 m、上 0.3 m，只跟航向）'


@pytest.mark.parametrize("gimbal, err_deg", [(-45, 21.1), (-90, 30.0)])
def test_the_fpv_check_catches_the_old_gimbal_plus_pitch_angle_addition(page, gimbal, err_deg):
    """同一套參考量舊寫法：有雲台俯角＋機體滾轉時視線偏掉（審查的兩個數字）；舊寫法本來就對的地方
    （雲台 0°、雲台穩定）在 1e-6 內相同 ⇒ 這個檢查抓的正是那個錯，不是隨便都會失敗。"""
    d, u, _ = _ref_fpv_enu(137, 15, 30, gimbal, True)
    old = page.evaluate(_FPV_JS, [137, 15, 30, gimbal, True, 'old', None])
    assert _ang_deg(old["dir"], d) == pytest.approx(err_deg, abs=0.1), old
    for g, follow in ((0, True), (gimbal, False)):
        d, u, r = _ref_fpv_enu(137, 15, 30, g, follow)
        old = page.evaluate(_FPV_JS, [137, 15, 30, g, follow, 'old', None])
        for name, ref in (("dir", d), ("up", u), ("right", r)):
            assert old[name] == pytest.approx(ref, abs=1e-6), (g, follow, name, old[name], ref)


def test_setview_direction_up_form_is_lossy_near_nadir_so_fpv_converts_to_hpr_itself(page):
    """Cesium 的 setView({direction, up}) 自己換 HPR：視線離鉛垂 2.6° 以內就把 roll 當 0、航向取上方向
    ⇒ 下視時機體滾轉 2°（視線往側邊偏 2°）被畫成往前偏 2°（兩者差 ≈ 2.83°）。FPV 自己換 HPR 才對。"""
    d, u, _ = _ref_fpv_enu(40, 0, 2, -90, True)
    lossy = page.evaluate(_FPV_JS, [40, 0, 2, -90, True, 'dirup', [d, u]])
    assert _ang_deg(lossy["dir"], d) > 2.5, lossy
    got = page.evaluate(_FPV_JS, [40, 0, 2, -90, True, 'new', None])
    assert _ang_deg(got["dir"], d) < 1e-6 and _ang_deg(got["up"], u) < 1e-6


# ══════════════════════════════════════════════════════════════════
#  K. 有 GLB 的頁面：每種模型配它自己的校正（2026-10-22 審查）
# ══════════════════════════════════════════════════════════════════
#   上面 H／I 的頁面三個 GLB 都留空 ⇒ 永遠是退回的方塊／橢球、校正 0 ⇒「VTOL 用了固定翼的校正」或「VTOL 載了固定翼的檔」
#   這兩種配對錯誤整個檔案照樣全過（審查實測）。這裡的頁面三個 URI 各填一個**不同的**小 GLB（一個三角形，真的載得起來），
#   校正用頁面真的那幾組（VTOL 的由 widget 常數填、固定翼／多旋翼是頁面常數）。
#   期望值**寫死在這裡**（使用者目視確認過、GLB 幾何稽核證實的那幾組），不從頁面讀 —— 從頁面讀就是拿錯的值去驗錯的值。
#   HERO（2026-10-23）：tools/build_hero_glb.py 匯出就是 Barracuda 慣例（glTF +X 機鼻、+Y 上、+Z 右翼）⇒ 0/0/0。
#   'HERO' 不是飛控的機型字串：飛控回報 VTOL、setUavAirframes 指定 'hero'（見 _hero_step）。
_GLB_EXPECT = {'VTOL': ('vtol', (-90, 90, 180)), 'PLANE': ('plane', (0, 0, 0)), 'COPTER': ('copter', (-90, 90, 180)),
               'HERO': ('hero', (0, 0, 0)), 'PIONEER': ('pioneer', (0, 0, 0))}
# 模型 scale（實際尺寸）：P25-ID 與多旋翼 GLB 是公分 ⇒ 0.01；Barracuda、Hero、Pioneer 匯出就是公尺 ⇒ 1
_GLB_SCALE = {'vtol': 0.01, 'plane': 1.0, 'copter': 0.01, 'hero': 1.0, 'pioneer': 1.0}


def _tiny_glb() -> bytes:
    """最小的合法 GLB（一個三角形）：Cesium 載得起來、不會在頁面丟錯。"""
    import struct
    pos = struct.pack('<9f', 0, 0, 0, 1, 0, 0, 0, 1, 0)
    gltf = {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
            "accessors": [{"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3",
                           "min": [0, 0, 0], "max": [1, 1, 0]}],
            "bufferViews": [{"buffer": 0, "byteLength": len(pos)}], "buffers": [{"byteLength": len(pos)}]}
    js = json.dumps(gltf, separators=(',', ':')).encode('utf-8')
    js += b' ' * ((4 - len(js) % 4) % 4)
    return (struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(pos))
            + struct.pack('<II', len(js), 0x4E4F534A) + js + struct.pack('<II', len(pos), 0x004E4942) + pos)


@pytest.fixture(scope="module")
def page_glb3d(_pw, tmp_path_factory):
    if not _QT_OK:
        pytest.skip('Qt 匯入失敗')
    if not (CESIUM_DIR / "Cesium.js").is_file():
        pytest.skip("沒有本機 Cesium（assets/cesium 是 gitignored）")
    d = tmp_path_factory.mktemp("uavglb")
    uris = {}
    for kind in ('vtol', 'plane', 'copter', 'hero', 'pioneer'):
        p = d / f'{kind}_dummy.glb'
        p.write_bytes(_tiny_glb())
        uris[kind] = p.as_uri()
    with pytest.MonkeyPatch.context() as mp:
        vals = _template_values(mp)
    vals.update({'__PLANE_MODEL_URI__': uris['plane'], '__COPTER_MODEL_URI__': uris['copter'],
                 '__VTOL_MODEL_URI__': uris['vtol'], '__INIT_LAT__': str(LAT0), '__INIT_LON__': str(LON0),
                 '/*__AIRFRAME_MODELS__*/': (f'_HERO_MODEL_URI = {json.dumps(uris["hero"])}; '
                                             f'_PIONEER_MODEL_URI = {json.dumps(uris["pioneer"])};')})
    f = d / "main.html"
    f.write_text(_NoDel.__new__(_NoDel)._fill_cesium_template(HTML, vals), encoding="utf-8")
    try:
        b = _launch(_pw, _REAL_ARGS)
        pg = b.new_page(viewport={"width": 640, "height": 480})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.route("**/*", lambda r: r.continue_() if r.request.url.startswith(("file:", "data:", "blob:")) else r.abort())
        pg.goto(f.as_uri())
        pg.wait_for_function("typeof viewer !== 'undefined' && typeof updateUAVPositions === 'function'", timeout=60000)
        pg.add_script_tag(content=_PAGE_HELPERS)
        pg.evaluate("() => { T_.heroUri = _HERO_MODEL_URI; T_.pioneerUri = _PIONEER_MODEL_URI; }")
    except Exception as e:                                   # pragma: no cover - 沒有 WebGL 等
        pytest.skip(f"真的 Cesium 起不來：{e}")
    pg._errs = errs
    pg._uris = uris
    yield pg
    b.close()


@pytest.fixture
def page_glb(page_glb3d):
    yield page_glb3d
    page_glb3d.evaluate("() => { clearUAV(); _RENDER_IDLE_MS = 2000; _HERO_MODEL_URI = T_.heroUri; "
                        "_PIONEER_MODEL_URI = T_.pioneerUri; "
                        "_UAV_AIRFRAME_DEFAULT = ''; for (const k of Object.keys(_UAV_AIRFRAME)) delete _UAV_AIRFRAME[k]; }")
    assert not page_glb3d._errs, page_glb3d._errs


# 畫面上這一架：載入的模型 URI、entity 的方向乘上「期望那一種模型」校正的反矩陣 ⇒ 校正後機體座標（機鼻 +y、右翼 +x、
# 機背 +z）的三軸在 ENU 下的分量。單一樣本（HOLD）⇒ 就是送進去那一筆。位置每步都一樣（k 只改時間）：位置樣本不隨換模型
# 丟掉，位置一動，顯示時間落在兩筆之間 ⇒ ENU 基準差 ~1e-6，會混進 1e-9 的比較。
_GLB_STEP_JS = """([sid, k, vt, h, p, r, offs]) => {
  const now = Date.now();
  updateUAVPositions([T_.item(sid, 0, {vt: vt, hdg: h, pitch: p, roll: r, pt: now - 300 + 50 * k, at: now - 300 + 50 * k})]);
  const e = uavEntities[sid], jd = Cesium.JulianDate.now();
  const u = e.model ? e.model.uri.getValue(jd) : null;
  const q = e.orientation.getValue(jd), pos = e.position.getValue(jd);
  const fix = Cesium.Quaternion.fromHeadingPitchRoll(new Cesium.HeadingPitchRoll(
    Cesium.Math.toRadians(offs[0]), Cesium.Math.toRadians(offs[1]), Cesium.Math.toRadians(offs[2])));
  const body = Cesium.Quaternion.multiply(q, Cesium.Quaternion.inverse(fix, new Cesium.Quaternion()), new Cesium.Quaternion());
  const M = Cesium.Matrix3.fromQuaternion(body);
  const enuInv = Cesium.Matrix4.inverseTransformation(Cesium.Transforms.eastNorthUpToFixedFrame(pos), new Cesium.Matrix4());
  const ax = (x, y, z) => {
    const w = Cesium.Matrix3.multiplyByVector(M, new Cesium.Cartesian3(x, y, z), new Cesium.Cartesian3());
    const v = Cesium.Matrix4.multiplyByPointAsVector(enuInv, w, new Cesium.Cartesian3());
    return [v.x, v.y, v.z];
  };
  const cs = e.model && e.model.customShader ? e.model.customShader.getValue(jd) : null;
  return {uri: u ? String(u.url || u) : null, kind: e._uavKind, nOri: T_.oriTimes(sid).length,
          scale: e.model ? e.model.scale.getValue(jd) : null,
          fixedLight: !!cs && cs.lightingModel === Cesium.LightingModel.UNLIT,
          nose: ax(0, 1, 0), right: ax(1, 0, 0), up: ax(0, 0, 1)};
}"""


def _glb_check(pg, got, vt, att):
    kind, _offs = _GLB_EXPECT[vt]
    assert got["uri"] == pg._uris[kind], f'{vt} 要載入 {kind} 的 GLB：{got["uri"]}'
    assert got["kind"] == kind
    assert got["scale"] == pytest.approx(_GLB_SCALE[kind], rel=1e-12), f'{kind} 的實際尺寸（scale）：{got["scale"]}'
    # Hero／Pioneer 的白色不隨太陽（晚上也是白的）＝不看太陽的打光；其他機型照舊跟太陽（不能被連帶改掉）
    assert got["fixedLight"] == (kind in ('hero', 'pioneer')), f'{kind} 的打光（fixedLight）：{got["fixedLight"]}'
    for name, ref in zip(("nose", "right", "up"), _ref_axes_enu(*att)):
        assert got[name] == pytest.approx(list(ref), abs=1e-9), (vt, name, got[name], ref)


def _hero_step(pg, sid, k, airframe, att, expect_vt, dflt=None):
    """飛控回報 VTOL、這一架指定成 airframe（None ⇒ 不動逐架指定、看預設）；dflt 給了就換預設。
    畫出來的模型與校正要是 expect_vt 那一種（回傳 _GLB_STEP_JS 的結果）。"""
    pg.evaluate("([sid, a, d]) => setUavAirframes(a === null ? null : {[sid]: a}, d === null ? undefined : d)",
                [sid, airframe, dflt])
    return pg.evaluate(_GLB_STEP_JS, [sid, k, 'VTOL', *att, list(_GLB_EXPECT[expect_vt][1])])


@pytest.mark.parametrize("vt", ["VTOL", "PLANE", "COPTER", "HERO", "PIONEER"])
@pytest.mark.parametrize("att", [(137, 15, 30), (300, -20, -60)])
def test_real_cesium_each_glb_is_drawn_with_its_own_calibration(page_glb, vt, att):
    if vt in ('HERO', 'PIONEER'):
        got = _hero_step(page_glb, 81, 0, vt.lower(), att, vt)
    else:
        got = page_glb.evaluate(_GLB_STEP_JS, [81, 0, vt, *att, list(_GLB_EXPECT[vt][1])])
    _glb_check(page_glb, got, vt, att)


def test_real_cesium_hero_is_only_for_the_vtols_it_is_assigned_to(page_glb):
    """逐架指定 ＞ 預設；'' ＝通用 vtol.glb；指定 hero／pioneer 但沒有那個 GLB ⇒ 通用 vtol.glb（不會拿機體模型的校正
    去畫別的檔）；固定翼／多旋翼不管預設是什麼都不受影響。每一步模型檔、校正、姿態三者一致。"""
    att = (45, 6, -25)
    _glb_check(page_glb, _hero_step(page_glb, 83, 0, 'pioneer', att, 'PIONEER'), 'PIONEER', att)
    _glb_check(page_glb, _hero_step(page_glb, 84, 0, None, att, 'VTOL'), 'VTOL', att)               # 預設 ''
    _glb_check(page_glb, _hero_step(page_glb, 84, 1, None, att, 'HERO', dflt='hero'), 'HERO', att)  # 預設換成 hero
    _glb_check(page_glb, _hero_step(page_glb, 83, 1, None, att, 'PIONEER'), 'PIONEER', att)         # 逐架 pioneer 優先
    for sid, vt in ((85, 'PLANE'), (86, 'COPTER')):                                                 # 預設 hero 也不影響
        got = page_glb.evaluate(_GLB_STEP_JS, [sid, 0, vt, *att, list(_GLB_EXPECT[vt][1])])
        _glb_check(page_glb, got, vt, att)
    st = page_glb.evaluate("() => setUavAirframes({83: null})")                                    # 取消逐架 ⇒ 用預設
    assert st == {'dflt': 'hero', 'map': {}}, st
    _glb_check(page_glb, _hero_step(page_glb, 83, 2, None, att, 'HERO'), 'HERO', att)
    page_glb.evaluate("() => { _HERO_MODEL_URI = ''; _PIONEER_MODEL_URI = ''; }")                 # 沒有那兩個 GLB
    _glb_check(page_glb, _hero_step(page_glb, 87, 0, 'hero', att, 'VTOL'), 'VTOL', att)
    _glb_check(page_glb, _hero_step(page_glb, 89, 0, 'pioneer', att, 'VTOL'), 'VTOL', att)


def test_real_cesium_switching_a_vtol_to_hero_mid_stream_rebuilds_model_and_calibration_together(page_glb):
    """HARVEST 上傳後才知道是 Hero：同一架 VTOL 從通用 vtol.glb 換成 hero.glb（再換回去），模型、校正、姿態一起換，
    舊校正的模型姿態樣本丟掉（和機型改變同一條路徑）。"""
    steps = [(None, 'VTOL', (20, 4, 10)), ('hero', 'HERO', (110, -6, 30)), ('', 'VTOL', (250, 9, -35)),
             ('pioneer', 'PIONEER', (300, 3, 12)), ('hero', 'HERO', (340, 2, 5))]
    for k, (airframe, as_vt, att) in enumerate(steps):
        got = _hero_step(page_glb, 88, k, airframe, att, as_vt)
        if k:
            assert got["nOri"] == 1, f'換了模型 ⇒ 舊校正的模型姿態樣本丟掉（{airframe}）'
        _glb_check(page_glb, got, as_vt, att)


def test_real_cesium_glb_and_calibration_change_together_when_the_vehicle_type_changes(page_glb):
    """'---'（還沒 HEARTBEAT ⇒ 多旋翼）→ PLANE → VTOL（Q_ENABLE 讀回）→ COPTER：每一步模型檔、校正、姿態三者一致。"""
    steps = [('---', 'COPTER', (10, 5, 20)), ('PLANE', 'PLANE', (95, -8, 35)), ('VTOL', 'VTOL', (200, 12, -40)),
             ('COPTER', 'COPTER', (330, -3, 15))]
    for k, (vt, as_vt, att) in enumerate(steps):
        got = page_glb.evaluate(_GLB_STEP_JS, [82, k, vt, *att, list(_GLB_EXPECT[as_vt][1])])
        assert got["nOri"] == 1, f'換了模型 ⇒ 舊校正的模型姿態樣本丟掉（{vt}）'
        _glb_check(page_glb, got, as_vt, att)


# ── Hero／Pioneer 的白色不隨太陽（2026-10-24 使用者：晚上規劃也要白色）──────────────────────────────────
# 真的 hero.glb／pioneer.glb（gitignored ⇒ 沒建過就 skip）、真的 Cesium：時鐘釘在凌晨 1 點、傍晚、正午各畫一張，
# 機身要一樣白（逐像素相同）。沒有這個機制時晚上整架是深灰（太陽在地平線下）。
_NIGHT_TIMES = {'night': '2026-10-23T17:00:00Z', 'dusk': '2026-10-23T10:30:00Z', 'noon': '2026-10-24T04:00:00Z'}   # UTC；台灣 UTC+8

_NIGHT_SETUP_JS = """(iso) => {
  const s = viewer.scene;
  window.__nw = {globe: s.globe.show, sky: s.skyAtmosphere ? s.skyAtmosphere.show : null, bg: s.backgroundColor.clone()};
  s.globe.show = false; if (s.skyAtmosphere) s.skyAtmosphere.show = false;
  s.backgroundColor = Cesium.Color.fromCssColorString('#2b3340'); s.fog.enabled = false;
  window.__fixedTime = Cesium.JulianDate.fromIso8601(iso);
  const orig = viewer.clock.tick.bind(viewer.clock);
  viewer.clock.tick = function () { orig(); this.currentTime = window.__fixedTime; return this.currentTime; };
  viewer.clock.currentTime = window.__fixedTime;
}"""
_NIGHT_TEARDOWN_JS = """() => {
  const s = viewer.scene, w = window.__nw;
  delete viewer.clock.tick;                                   // 回到 prototype 的 tick
  if (w) { s.globe.show = w.globe; if (s.skyAtmosphere && w.sky !== null) s.skyAtmosphere.show = w.sky; s.backgroundColor = w.bg; }
}"""
_NIGHT_PLACE_JS = """([sid, af, lat, lon, alt, iso]) => {
  clearUAV(); window.__fixedTime = Cesium.JulianDate.fromIso8601(iso); viewer.clock.currentTime = window.__fixedTime;
  const now = Date.now();
  setUavAirframes({[sid]: af});
  updateUAVPositions([{sysid: sid, lat: lat, lon: lon, alt: alt, hdg: 0, pitch: 5, roll: 30, vt: 'VTOL',
                       spd: 0, mode: '', armed: false, pt: now, at: now}]);
  const e = uavEntities[sid];
  if (e.label) e.label.show = false;
  if (e.billboard) e.billboard.show = false;
  viewer.camera.lookAt(Cesium.Cartesian3.fromDegrees(lon, lat, alt),
                       new Cesium.HeadingPitchRange(Cesium.Math.toRadians(35), Cesium.Math.toRadians(-28), 4.0));
}"""
_NIGHT_READY_JS = """(sid) => {
  const e = uavEntities[sid];
  const walk = (p, d) => {
    if (!p || d > 6) return null;
    if (p.id === e && p.modelMatrix) return p;
    if (typeof p.length === 'number' && typeof p.get === 'function')
      for (let i = 0; i < p.length; i++) { const r = walk(p.get(i), d + 1); if (r) return r; }
    return null;
  };
  viewer.clock.tick(); viewer.render();
  const m = walk(viewer.scene.primitives, 0);
  return !!(m && m.ready);
}"""


@pytest.mark.parametrize("kind", ["hero", "pioneer"])
def test_real_cesium_airframe_is_equally_white_at_night_dusk_and_noon(page_glb, kind):
    import base64
    import io
    import time
    np = pytest.importorskip("numpy")
    Image = pytest.importorskip("PIL.Image")
    glb = REPO / 'ui' / 'resources' / f'{kind}.glb'
    if not glb.is_file():
        pytest.skip(f'{glb.name} 沒建過（gitignored；tools/build_{kind}_glb.py）')
    page_glb.evaluate("([h, p]) => { _HERO_MODEL_URI = h; _PIONEER_MODEL_URI = p; }",
                      [(REPO / 'ui' / 'resources' / 'hero.glb').as_uri(), (REPO / 'ui' / 'resources' / 'pioneer.glb').as_uri()])
    shots = {}
    page_glb.evaluate(_NIGHT_SETUP_JS, _NIGHT_TIMES['night'])
    try:
        for tname, iso in _NIGHT_TIMES.items():
            page_glb.evaluate(_NIGHT_PLACE_JS, [95, kind, LAT0, LON0, 120.0, iso])
            t0 = time.time()
            while not page_glb.evaluate(_NIGHT_READY_JS, 95):
                assert time.time() - t0 < 90, f'{kind} 模型 90 秒內沒載好'
                time.sleep(0.2)
            for _ in range(4):
                page_glb.evaluate(_NIGHT_READY_JS, 95)
            assert page_glb.evaluate("() => Cesium.JulianDate.toIso8601(viewer.scene.frameState.time)") == iso
            png = page_glb.evaluate('grabCanvas()')
            shots[tname] = np.asarray(Image.open(io.BytesIO(base64.b64decode(png.split(',', 1)[1]))).convert('RGB')).astype(int)
    finally:
        page_glb.evaluate(_NIGHT_TEARDOWN_JS)
    bg = shots['noon'][5, 5]
    plane = np.abs(shots['noon'] - bg).sum(axis=2) > 30           # 機身像素
    assert plane.sum() > 2000, f'{kind} 畫面上幾乎沒有機身（{int(plane.sum())} px）'
    for tname in ('night', 'dusk'):
        assert np.abs(shots[tname] - shots['noon']).max() <= 1, f'{kind} {tname} 和正午畫的不一樣（打光又跟太陽了）'
    white = shots['night'][plane].max(axis=1)
    assert np.percentile(white, 95) >= 235, f'{kind} 晚上機身不夠白：p95 亮度 {np.percentile(white, 95)}'
    assert shots['night'][plane].mean() >= 180, f'{kind} 晚上機身平均亮度 {shots["night"][plane].mean():.0f}（深灰＝跟著太陽）'
