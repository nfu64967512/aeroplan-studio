# -*- coding: utf-8 -*-
"""cesium_fps_probe.py — 3D 地圖（真的 main.html、真的 QtWebEngine）幀率與每幀耗時探針。

為什麼要有這支：tools/swarm_perf_profiler.py 量到的 FPS 不管幾架都 ~22，但它 (1) 沒套用 app 啟動時的 Chromium GPU 旗標、
(2) 視窗可能被其他視窗遮住（Chromium 會對被遮住的視窗降頻）、(3) 只量「算了幾幀」，分不出是 GPU、JS 還是合成節奏卡住。
這支把那三件事拆開量：
  rafHz       瀏覽器 requestAnimationFrame 的實際節奏（合成器給的上限）
  fps         Cesium 真的算圖的幀數（scene.postRender）
  jsMs        每幀 JS 時間（我們的 clock.onTick → postRender：場景更新＋繪製命令；不含 GPU 等待。
              ⚠ Viewer 自己的 onTick（DataSourceDisplay 更新 entity）先跑，不在這段裡）
  intervalMs  相鄰兩次 postRender 的間隔 p50 / p95 / max（抖動／卡頓）
  dpr、canvas 實際 drawing buffer 尺寸（Windows 縮放 150% 時畫素量是 2.25 倍）

Qt 呈現設定預設跟 app 一樣（ui/render_setup.py：Windows swapInterval 0）；--qt-swap-interval 1 量舊行為（~22 FPS）。
3D 模型預設跟 app 一樣走合併快取（ui/widgets/glb_merge.py）；AEROPLAN_GLB_MERGE=0 量原檔。

用法（一次一組，GPU 量測不要並行）：
  python tools/cesium_fps_probe.py --n 6                       # app 的 GPU 旗標、6 架、連續算圖
  python tools/cesium_fps_probe.py --blank                      # 不載 Cesium，只量 rAF 節奏（合成器上限）
  python tools/cesium_fps_probe.py --n 6 --flags "--disable-features=CalculateNativeWinOcclusion"
  python tools/cesium_fps_probe.py --n 6 --set msaa=1 --set rs=0.67 --set atmo=0 --set fxaa=0 --set lighting=0
  python tools/cesium_fps_probe.py --n 6 --json out.json --label 某組

--set 可用：msaa=<1|2|4|8> rs=<resolutionScale> brr=<0|1 useBrowserRecommendedResolution> atmo=<0|1> fxaa=<0|1>
            lighting=<0|1> fog=<0|1> sse=<globe.maximumScreenSpaceError> shadows=<0|1>
不讀 config/settings.json（ion token 留空）、不開 SITL；只開一個 QtWebEngine 視窗量完就關。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

APP_GPU_FLAGS = ('--ignore-gpu-blocklist --enable-gpu-rasterization --enable-zero-copy '
                 '--enable-accelerated-video-decode')        # 與 main.py::_apply_gpu_acceleration 相同


def _parse():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=6, help='合成飛機數')
    ap.add_argument('--hz', type=float, default=30.0, help='批次更新頻率（模擬主視窗合幀）')
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--warmup', type=float, default=4.0)
    ap.add_argument('--size', default='1400x900')
    ap.add_argument('--maximized', action='store_true', help='視窗最大化（量真實 app 的畫布大小）')
    ap.add_argument('--no-app-flags', action='store_true', help='不套 app 的 GPU 旗標')
    ap.add_argument('--flags', default='', help='額外的 Chromium 旗標（QTWEBENGINE_CHROMIUM_FLAGS）')
    ap.add_argument('--blank', action='store_true', help='不載 Cesium，只量 rAF 節奏')
    ap.add_argument('--synthetic', default='', choices=['', 'anim', 'webgl', 'webgl-aa', 'canvas2d'],
                    help='不載 Cesium，換成每幀都改畫面的最小頁：anim=DOM 方塊移動、webgl=每幀清色、canvas2d=每幀畫')
    ap.add_argument('--set', action='append', default=[], help='Cesium 畫質開關 key=value，可多次')
    ap.add_argument('--render-dt', default='0', help='__RENDER_MAX_DT__；0 = 連續算圖（預設，量上限用）')
    ap.add_argument('--chase', type=int, default=0, help='跟隨相機鎖定的 sysid（0 = 不跟隨，固定俯視）')
    ap.add_argument('--json', default='')
    ap.add_argument('--label', default='')
    ap.add_argument('--hires-timer', action='store_true', help='timeBeginPeriod(1)：Windows 計時器解析度 15.6 ms → 1 ms')
    ap.add_argument('--js', default='', help='就緒後額外執行的 JS（例如 setChaseCameraParams(15, 4)）')
    ap.add_argument('--shot', default='', help='量完把螢幕上的視窗存成 PNG（驗證畫面不是黑的）')
    ap.add_argument('--qt-swap-interval', type=int, default=None,
                    help='QSurfaceFormat 預設 swapInterval；省略 = 跟 app 一樣（ui/render_setup.py，Windows 0）；'
                         '1 = 舊行為（Qt 端等 vsync）；-1 = 不動 Qt 預設')
    ap.add_argument('--target-fps', type=float, default=0, help='viewer.targetFrameRate（0 = 不設）')
    ap.add_argument('--template', default='', help='main.html 的替代檔（量改動前的頁面當基準；預設 = repo 的 main.html）')
    ap.add_argument('--att-hz', type=float, default=0.0,
                    help='姿態樣本頻率（Hz）：比 --hz 高時，兩批之間比主樣本舊的那幾筆放進 atts（和主視窗合幀一樣，'
                         '2026-10-22）；0 = 每批只帶一筆姿態（舊行為）')
    return ap.parse_args()


ARGS = _parse()
if ARGS.hires_timer and sys.platform == 'win32':
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)
flags = ' '.join(x for x in ((APP_GPU_FLAGS if not ARGS.no_app_flags else ''), ARGS.flags,
                             os.environ.get('QTWEBENGINE_CHROMIUM_FLAGS', '')) if x).strip()
if flags:
    os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = flags
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

import logging  # noqa: E402
logging.disable(logging.INFO)

from PyQt6.QtCore import QTimer, QUrl, Qt  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMainWindow  # noqa: E402
QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
if ARGS.qt_swap_interval is None:
    from ui.render_setup import configure_qt_presentation
    _applied = configure_qt_presentation()
    ARGS.qt_swap_interval = -1 if _applied is None else _applied
elif ARGS.qt_swap_interval >= 0:
    from PyQt6.QtGui import QSurfaceFormat
    _fmt = QSurfaceFormat.defaultFormat()
    _fmt.setSwapInterval(ARGS.qt_swap_interval)
    QSurfaceFormat.setDefaultFormat(_fmt)
from PyQt6.QtWebEngineWidgets import QWebEngineView  # noqa: E402
from PyQt6.QtWebEngineCore import QWebEnginePage  # noqa: E402

APP = QApplication.instance() or QApplication(sys.argv)
LAT0, LON0 = 23.4518, 120.2863

BLANK_HTML = """<!doctype html><html><body style="margin:0;background:#123"><canvas id=c></canvas><script>
window.__m = {rafs: 0}; (function r(){ window.__m.rafs++; requestAnimationFrame(r); })();
window.__ready = true;</script></body></html>"""


SYNTH_HTML = """<!doctype html><html><body style="margin:0;background:#123;overflow:hidden">
<canvas id=c style="width:100vw;height:100vh;display:block"></canvas><div id=b style="position:absolute;left:0;top:40px;width:80px;height:80px;background:#f80"></div>
<script>
const KIND = '__KIND__'; window.__m = {rafs: 0, frames: 0, iv: [], js: [], lastPost: 0};
const c = document.getElementById('c'), b = document.getElementById('b');
c.width = c.clientWidth * devicePixelRatio; c.height = c.clientHeight * devicePixelRatio;
let gl = null, g2 = null;
if (KIND.startsWith('webgl')) gl = c.getContext('webgl2', {antialias: KIND === 'webgl-aa'});
if (KIND === 'canvas2d') g2 = c.getContext('2d');
(function r(t){ const m = window.__m; m.rafs++; m.frames++;
  const now = performance.now(); if (m.lastPost) m.iv.push(now - m.lastPost); m.lastPost = now;
  const x = (t / 4) % (innerWidth - 80);
  if (KIND === 'anim') b.style.transform = 'translateX(' + x + 'px)';
  if (gl) { gl.clearColor((t / 1000) % 1, 0.2, 0.4, 1); gl.clear(gl.COLOR_BUFFER_BIT); }
  if (g2) { g2.fillStyle = '#123'; g2.fillRect(0, 0, c.width, c.height); g2.fillStyle = '#0f8'; g2.fillRect(x * devicePixelRatio, 200, 100, 100); }
  requestAnimationFrame(r); })(0);
window.__ready = true;</script></body></html>"""


MODEL_INFO: dict = {}


def build_page() -> pathlib.Path:
    out = ROOT / 'data' / 'tmp_maps'
    out.mkdir(parents=True, exist_ok=True)
    path = out / 'cesium_fps_probe.html'
    if ARGS.synthetic:
        path.write_text(SYNTH_HTML.replace('__KIND__', ARGS.synthetic), encoding='utf-8')
        return path
    if ARGS.blank:
        path.write_text(BLANK_HTML, encoding='utf-8')
        return path
    from ui.widgets.cesium_map_widget import CesiumMapWidget, _CESIUM_HTML
    from ui.widgets import wind_layer as wl
    cz = ROOT / 'assets' / 'cesium' / 'Build' / 'Cesium'
    res = ROOT / 'ui' / 'resources'
    from ui.widgets.glb_merge import optimized_model_path       # 與 app 相同（AEROPLAN_GLB_MERGE=0 ⇒ 原檔）
    cache = ROOT / 'data' / 'cache' / 'models'
    def uri(p):
        if not p.exists():
            return ''
        q = optimized_model_path(p, cache)
        MODEL_INFO[p.name] = q.name                       # 實際生效：merged-*.glb 或原檔名
        return q.as_uri()
    try:
        wind_script = f'<script src="{wl.wind_flow_js_uri()}"></script>'
    except FileNotFoundError:
        wind_script = ''
    rr_true = float(ARGS.render_dt) > 0
    values = {
        '__WIND_FLOW_SCRIPT__': wind_script, '/*__WIND_GLUE__*/': wl.glue_js('cesium'), '/*__WIND_RESTORE__*/': '',
        '__CESIUM_JS__': (cz / 'Cesium.js').as_uri(), '__CESIUM_CSS__': (cz / 'Widgets' / 'widgets.css').as_uri(),
        '__CESIUM_TOKEN__': '', '__INIT_LAT__': str(LAT0), '__INIT_LON__': str(LON0), '__INIT_HEIGHT__': '1500',
        '__PLANE_MODEL_URI__': uri(res / 'barracuda500.glb'), '__COPTER_MODEL_URI__': uri(res / 'copter.glb'),
        '__VTOL_MODEL_URI__': uri(res / 'vtol.glb'),
        '__VTOL_MODEL_SCALE__': str(CesiumMapWidget.VTOL_MODEL_SCALE),
        '__VTOL_HEADING_OFFSET__': str(CesiumMapWidget.VTOL_HEADING_OFFSET),
        '__VTOL_PITCH_OFFSET__': str(CesiumMapWidget.VTOL_PITCH_OFFSET),
        '__VTOL_ROLL_OFFSET__': str(CesiumMapWidget.VTOL_ROLL_OFFSET),
        '__REQUEST_RENDER_MODE__': 'true' if rr_true else 'false',
        '__RENDER_MAX_DT__': ARGS.render_dt if rr_true else '0',
    }
    html = pathlib.Path(ARGS.template).read_text(encoding='utf-8') if ARGS.template else _CESIUM_HTML
    for k, v in values.items():
        html = html.replace(k, v)
    path.write_text(html, encoding='utf-8')
    return path


MEASURE_JS = r"""
(function(){
  window.__m = {frames: 0, rafs: 0, js: [], iv: [], lastPost: 0, tick: 0, batch: []};
  (function r(){ window.__m.rafs++; requestAnimationFrame(r); })();
  // 每次批次呼叫（updateUAVPositions）本身花多久：取樣、姿態四元數、尾跡、碰撞檢查（不在每幀的 jsMs 裡）
  if (typeof updateUAVPositions === 'function' && !window.__batchWrapped) {
    const orig = updateUAVPositions;
    window.updateUAVPositions = function (a) {
      const t = performance.now(); const r = orig(a); window.__m.batch.push(performance.now() - t); return r;
    };
    window.__batchWrapped = true;
  }
  if (typeof viewer !== 'undefined') {
    viewer.clock.onTick.addEventListener(function(){ window.__m.tick = performance.now(); });
    viewer.scene.postRender.addEventListener(function(){
      const now = performance.now(), m = window.__m;
      m.frames++;
      if (m.tick) m.js.push(now - m.tick);
      if (m.lastPost) m.iv.push(now - m.lastPost);
      m.lastPost = now;
    });
  }
  return 'ok';
})();
"""

READ_JS = r"""
(function(){
  const m = window.__m || {};
  const q = (a, p) => { if (!a || !a.length) return null; const s = a.slice().sort((x, y) => x - y);
                        return +s[Math.min(s.length - 1, Math.floor(p * (s.length - 1)))].toFixed(2); };
  let cv = null; try { const c = viewer.scene.canvas; cv = [c.width, c.height, c.clientWidth, c.clientHeight]; } catch (_) {}
  let gpu = null; try { const gl = viewer.scene.context._gl; const d = gl.getExtension('WEBGL_debug_renderer_info');
                        gpu = d ? gl.getParameter(d.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER); } catch (_) {}
  let msaa = null, rs = null, rrm = null; try { msaa = viewer.scene.msaaSamples; rs = viewer.resolutionScale;
                                               rrm = viewer.scene.requestRenderMode; } catch (_) {}
  return JSON.stringify({frames: m.frames || 0, rafs: m.rafs || 0, jsP50: q(m.js, 0.5), jsP95: q(m.js, 0.95),
          batchN: (m.batch || []).length, batchP50: q(m.batch, 0.5), batchP95: q(m.batch, 0.95), batchMax: q(m.batch, 1.0),
          ivP50: q(m.iv, 0.5), ivP95: q(m.iv, 0.95), ivMax: q(m.iv, 1.0), dpr: window.devicePixelRatio, canvas: cv,
          gpu: gpu, msaa: msaa, rs: rs, requestRenderMode: rrm, visibility: document.visibilityState});
})();
"""


def apply_settings_js() -> str:
    parts = []
    for kv in ARGS.set:
        k, _, v = kv.partition('=')
        if k == 'msaa':
            parts.append(f'viewer.scene.msaaSamples = {int(v)};')
        elif k == 'rs':
            parts.append(f'viewer.resolutionScale = {float(v)};')
        elif k == 'brr':
            parts.append(f'viewer.useBrowserRecommendedResolution = {"true" if v == "1" else "false"};')
        elif k == 'atmo':
            parts.append(f'if (viewer.scene.skyAtmosphere) viewer.scene.skyAtmosphere.show = {"true" if v == "1" else "false"};'
                         f'viewer.scene.globe.showGroundAtmosphere = {"true" if v == "1" else "false"};')
        elif k == 'fxaa':
            parts.append(f'viewer.scene.postProcessStages.fxaa.enabled = {"true" if v == "1" else "false"};')
        elif k == 'lighting':
            parts.append(f'viewer.scene.globe.enableLighting = {"true" if v == "1" else "false"};')
        elif k == 'fog':
            parts.append(f'viewer.scene.fog.enabled = {"true" if v == "1" else "false"};')
        elif k == 'sse':
            parts.append(f'viewer.scene.globe.maximumScreenSpaceError = {float(v)};')
        elif k == 'shadows':
            parts.append(f'viewer.shadows = {"true" if v == "1" else "false"};')
        else:
            raise SystemExit(f'未知的 --set {kv}')
    return '\n'.join(parts)


def _effective_swap():
    from PyQt6.QtGui import QSurfaceFormat
    return QSurfaceFormat.defaultFormat().swapInterval()


def main() -> int:
    w, h = (int(x) for x in ARGS.size.lower().split('x'))
    win = QMainWindow()
    view = QWebEngineView()
    page = QWebEnginePage(view)
    view.setPage(page)
    win.setCentralWidget(view)
    win.resize(w, h)
    win.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, True)    # 不讓別的視窗擋住（擋住會被降頻）
    if ARGS.maximized:
        win.showMaximized()
    else:
        win.show()
    win.raise_(); win.activateWindow()
    page.load(QUrl.fromLocalFile(str(build_page())))

    box = {}
    qt = {'frames': 0, 'hooked': [], 'ts': []}

    def _qt_frame():
        qt['frames'] += 1
        qt['ts'].append(time.perf_counter())

    def hook_qt_frames():
        try:
            from PyQt6.QtQuickWidgets import QQuickWidget
        except Exception:                                   # noqa: BLE001
            return
        for qw in view.findChildren(QQuickWidget):
            if id(qw) in qt['hooked']:
                continue
            qt['hooked'].append(id(qw))
            qw.quickWindow().afterRendering.connect(_qt_frame, Qt.ConnectionType.DirectConnection)

    def js(code, timeout=5.0):
        box.pop('v', None)
        page.runJavaScript(code, lambda v: box.__setitem__('v', v))
        from PyQt6.QtCore import QEventLoop
        loop = QEventLoop()
        page.runJavaScript('0;', lambda _v: None)
        t0 = time.monotonic()
        while 'v' not in box and time.monotonic() - t0 < timeout:
            QTimer.singleShot(5, loop.quit)
            loop.exec()
        return box.get('v')

    def pump(sec):
        # 真的事件迴圈（QEventLoop.exec），跟 app 一樣；手動 processEvents()+sleep 會改變 Chromium 任務的排程
        from PyQt6.QtCore import QEventLoop
        loop = QEventLoop()
        QTimer.singleShot(int(sec * 1000), Qt.TimerType.PreciseTimer, loop.quit)
        loop.exec()

    ARGS.blank = ARGS.blank or bool(ARGS.synthetic)
    ready_expr = 'window.__ready === true' if ARGS.blank else \
        "typeof viewer !== 'undefined' && typeof updateUAVPositions === 'function'"
    t0 = time.monotonic()
    while not js(ready_expr, 2.0):
        pump(0.2)
        if time.monotonic() - t0 > 60:
            print(json.dumps({'error': '頁面 60 s 內沒就緒'})); return 1
    if not ARGS.blank:
        s = apply_settings_js()
        if s:
            js(s + " 'ok';")
        if ARGS.target_fps:
            js(f'viewer.targetFrameRate = {float(ARGS.target_fps)}; "ok";')
        if ARGS.chase:
            js(f'setChaseCamera({int(ARGS.chase)}); "ok";')
        else:
            js(f'viewer.camera.setView({{destination: Cesium.Cartesian3.fromDegrees({LON0}, {LAT0 - 0.012}, 900),'
               f' orientation: {{heading: 0, pitch: Cesium.Math.toRadians(-35), roll: 0}}}}); "ok";')
        if ARGS.js:                       # 在鏡頭擺好之後才跑（以前插在 if/else 中間，給了 --js 就跳過擺鏡頭）
            js(ARGS.js + '; "ok";')

    # 合成飛機：繞圈，帶 pt/at（同 app 的批次項目）；--att-hz 時姿態是自己的節奏，舊的放 atts
    t_start = time.time()
    feed_st = {'last_att': {}, 'n_atts': 0, 'n_items': 0}

    def att_of(i, t):
        r, v = 150.0 + 30 * i, 18.0
        th = v * (t - t_start) / r + i
        return (-17.0 + 3.0 * math.sin(40.0 * t), 2.0 + 2.0 * math.sin(37.0 * t), (math.degrees(th) + 90.0) % 360.0)

    def feed():
        if ARGS.blank or ARGS.n <= 0:
            return
        now = time.time()
        items = []
        for i in range(ARGS.n):
            r, v = 150.0 + 30 * i, 18.0
            th = v * (now - t_start) / r + i
            e, n = r * math.sin(th), r * math.cos(th)
            hdg = (math.degrees(th) + 90.0) % 360.0
            it = {'sysid': i + 1, 'lat': LAT0 + n / 111319.49,
                  'lon': LON0 + e / (111319.49 * math.cos(math.radians(LAT0))), 'alt': 120.0 + 10 * i,
                  'hdg': hdg, 'crs': hdg, 'spd': v, 'mode': 'AUTO', 'armed': True, 'vt': 'VTOL',
                  'pitch': 2.0, 'roll': -17.0, 'pt': int(now * 1000), 'at': int(now * 1000)}
            if ARGS.att_hz > 0:
                dt = 1.0 / ARGS.att_hz
                k_now = math.floor((now - t_start) / dt)            # 這一批時最新的那一則姿態
                k_last = feed_st['last_att'].get(i, k_now - 1)
                ks = list(range(k_last + 1, k_now + 1))[-64:]
                feed_st['last_att'][i] = k_now
                samples = []
                for k in ks:
                    tk = t_start + k * dt
                    rr, pp, yy = att_of(i, tk)
                    samples.append([int(round(tk * 1000)), round(rr, 2), round(pp, 2), round(yy, 2)])
                if samples:
                    ms, rr, pp, yy = samples[-1]
                    it.update(at=ms, roll=rr, pitch=pp, hdg=yy)
                    if len(samples) > 1:
                        it['atts'] = samples[:-1]
                        feed_st['n_atts'] += len(samples) - 1
            feed_st['n_items'] += 1
            items.append(it)
        page.runJavaScript('updateUAVPositions(' + json.dumps(items, separators=(',', ':')) + ');')

    timer = QTimer()
    timer.setTimerType(Qt.TimerType.PreciseTimer)
    timer.timeout.connect(feed)
    timer.start(int(1000 / ARGS.hz))
    pump(ARGS.warmup)
    hook_qt_frames()
    if ARGS.synthetic or ARGS.blank:
        # 頁面自己已有 rAF 迴圈 ⇒ 只歸零；再注入 MEASURE_JS 會多一個迴圈、rafHz 變兩倍（2026-10-17 審查：120 Hz 其實是 60）
        js('window.__m.rafs = 0; window.__m.frames = 0; window.__m.iv = []; "ok";')
    else:
        js(MEASURE_JS)
    qt['frames'] = 0
    qt['ts'] = []
    t0 = time.monotonic()
    pump(ARGS.seconds)
    wall = time.monotonic() - t0
    qt_frames = qt['frames']
    qiv = sorted((b - a) * 1000 for a, b in zip(qt['ts'], qt['ts'][1:]))
    hist = {}
    for v in qiv:
        k = int(round(v))
        hist[k] = hist.get(k, 0) + 1
    qt_iv = {'p10': round(qiv[len(qiv) // 10], 2), 'p50': round(qiv[len(qiv) // 2], 2),
             'p90': round(qiv[len(qiv) * 9 // 10], 2), 'max': round(qiv[-1], 2),
             'hist_ms': dict(sorted(hist.items(), key=lambda kv: -kv[1])[:8])} if qiv else None
    raw = js(READ_JS)
    if ARGS.shot:
        g = win.frameGeometry()                       # 抓桌面那塊（GL/D3D 視窗直接 grabWindow(winId) 會是白的）
        win.screen().grabWindow(0, g.x(), g.y(), g.width(), g.height()).save(ARGS.shot)
    timer.stop()
    r = json.loads(raw) if raw else {}
    res = {'label': ARGS.label, 'n': ARGS.n, 'blank': ARGS.blank, 'size': ARGS.size, 'flags': flags, 'set': ARGS.set,
           'chase': ARGS.chase, 'renderDt': ARGS.render_dt, 'hiresTimer': ARGS.hires_timer,
           'targetFps': ARGS.target_fps, 'rhi': os.environ.get('QSG_RHI_BACKEND', ''),
           'glbMerge': os.environ.get('AEROPLAN_GLB_MERGE', '1'), 'models': dict(MODEL_INFO),
           'swapEffective': _effective_swap(), 'maximized': ARGS.maximized,
           'qtSwapInterval': ARGS.qt_swap_interval, 'template': ARGS.template, 'attHz': ARGS.att_hz,
           'feedItems': feed_st['n_items'], 'feedExtraAtts': feed_st['n_atts'],
           'env': {k: v for k, v in os.environ.items() if k.startswith(('QT_', 'QSG_')) and 'LOGGING' not in k},
           'qtHz': round(qt_frames / wall, 1) if qt['hooked'] else None, 'qtIntervalMs': qt_iv,
           'qtDelegates': [type(c).__name__ for c in view.findChildren(object)][:12],
           'fps': round(r.get('frames', 0) / wall, 1), 'rafHz': round(r.get('rafs', 0) / wall, 1), **r}
    print(json.dumps(res, ensure_ascii=False))
    if ARGS.json:
        p = pathlib.Path(ARGS.json); p.parent.mkdir(parents=True, exist_ok=True)
        old = json.loads(p.read_text(encoding='utf-8')) if p.exists() else []
        old.append(res)
        p.write_text(json.dumps(old, ensure_ascii=False, indent=1), encoding='utf-8')
    win.close()
    pump(0.3)
    return 0


if __name__ == '__main__':
    sys.exit(main())
