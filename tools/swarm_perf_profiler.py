"""swarm_perf_profiler.py — AeroPlan 3D 地圖多機效能 profiler(遞增 1→12 台)。

目的:在「你的機器」上實測「開 3D 地圖、N 台同時串流」時的 **FPS / CPU / 記憶體**,
並提供 baseline vs optimized 前後對比,作為是否把最佳化(requestRenderMode + 遙測節流)
寫進核心碼的依據。

量什麼:
  - FPS  : 注入 JS 掛 Cesium `scene.postRender`,數「真實渲染幀」,每段視窗算平均。
  - CPU  : psutil 涵蓋本行程 + 所有子行程(含 QtWebEngineProcess GPU/render)。
  - RSS  : 同上行程樹的常駐記憶體。

怎麼測:
  用「真實的」DualMapWidget(切 3D)+ FleetRegistry + SITLLink,接 6/12 個會飛編隊
  軌跡的 MAVLink 樁(真 TCP)。台數從 1 遞增到 12 = 兩群 ×6(一長機+一備長+四僚機),
  每個台數階段穩定後量一段視窗。

用法:
  python tools/swarm_perf_profiler.py                 # 預設:both(先 baseline 後 optimized),GUI
  python tools/swarm_perf_profiler.py --mode baseline  # 只測現況
  python tools/swarm_perf_profiler.py --mode optimized # 只測最佳化後
  python tools/swarm_perf_profiler.py --steps 1,3,6,9,12 --dwell 8
  python tools/swarm_perf_profiler.py --no-gui         # 不開地圖,只驗機制 + 量 CPU(無頭可跑)

兩種模式(2026-10-17 起;核心碼已內建合幀、批次與自適應算圖,這支不再在執行時覆寫頁面設定):
  baseline : 每筆遙測直接呼叫 DualMapWidget.update_uav_position(單架舊 API;頁面仍是新核心)。
  optimized: 走 app 實際的路徑 —— 以 ui.main_window.TELEMETRY_FLUSH_HZ 合幀、每批用
             MainWindow._uav_batch_item(含 pt/at 時間戳)組批次,一次 DualMapWidget.update_uav_positions。
  ⚠ 以前的 optimized 會注入 `scene.requestRenderMode=true`;頁面現在有自己的自適應算圖(有遙測就連續算圖),
    注入的值會被下一批蓋掉、量到的不是它宣稱的東西(審查 2026-10-17)⇒ 拿掉了。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

# ── 專案根路徑 ───────────────────────────────────────────────────────────
_THIS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(_THIS)
if REPO not in sys.path:
    sys.path.insert(0, REPO)

import logging
logging.disable(logging.INFO)

try:
    import psutil
except ImportError:
    psutil = None

from pymavlink import mavutil
from PyQt6.QtCore import QCoreApplication, QObject, Qt, QTimer

_MAV = mavutil.mavlink

# ── 編隊 / 群組設定 ───────────────────────────────────────────────────────
GROUP_SIZE = 6                       # 一群 6 台 = 1 長機 + 1 備長 + 4 僚機
BASE_PORT = 47600
GROUP_CENTERS = [(25.000, 121.500), (25.050, 121.560)]   # 兩群的繞行中心
ORBIT_R_M = 300.0                    # 繞行半徑
ORBIT_PERIOD_S = 80.0                # 繞一圈秒數(讓位置持續變化、尾跡成長)


def role_of(idx_in_group: int) -> str:
    return {0: 'LEADER', 1: 'BACKUP'}.get(idx_in_group, 'WINGMAN')


def vehicle_plan(n_total: int):
    """回傳 n_total 台的 (sysid, port, group, role, center, phase, radius)。"""
    out = []
    for i in range(n_total):
        g = i // GROUP_SIZE
        j = i % GROUP_SIZE
        clat, clon = GROUP_CENTERS[g % len(GROUP_CENTERS)]
        phase = 2 * math.pi * j / GROUP_SIZE          # 群內均分相位
        radius = ORBIT_R_M * (0.6 + 0.1 * j)          # 略不同半徑 → 散開
        out.append(dict(sysid=i + 1, port=BASE_PORT + 10 * i, group=g,
                        role=role_of(j), clat=clat, clon=clon,
                        phase=phase, radius=radius))
    return out


# ═══════════════════════════════════════════════════════════════════════════
#  會飛的 MAVLink 樁(tcpin 伺服端,送移動中的遙測)
# ═══════════════════════════════════════════════════════════════════════════

# 樁機的 MAV_TYPE（--vehicle 改）：FIXED_WING ⇒ Barracuda（3 個 primitive，很輕）；VTOL_QUADROTOR ⇒ vtol.glb（CAD 匯出，最重）
STUB_MAV_TYPE = 'MAV_TYPE_FIXED_WING'

class MovingMavStub:
    def __init__(self, spec: dict, rate_hz: float):
        self.spec = spec
        self.dt = 1.0 / rate_hz
        self._stop = False
        self._thread = None
        self._conn = None
        self._ready = threading.Event()
        self.bind_error = None

    def start(self) -> bool:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._ready.wait(3.0)
        return self.bind_error is None

    def _pos(self, t: float):
        s = self.spec
        ang = s['phase'] + 2 * math.pi * (t / ORBIT_PERIOD_S)
        dnorth = s['radius'] * math.cos(ang)
        deast = s['radius'] * math.sin(ang)
        lat = s['clat'] + dnorth / 111_320.0
        lon = s['clon'] + deast / (111_320.0 * math.cos(math.radians(s['clat'])))
        hdg = (math.degrees(ang) + 90.0) % 360.0
        return lat, lon, hdg

    def _run(self):
        s = self.spec
        try:
            self._conn = mavutil.mavlink_connection(
                f"tcpin:127.0.0.1:{s['port']}", source_system=s['sysid'],
                dialect="ardupilotmega")
        except Exception as e:
            self.bind_error = repr(e)
            self._ready.set()
            return
        self._ready.set()
        m = self._conn.mav
        t0 = time.monotonic()
        i = 0
        while not self._stop:
            t = time.monotonic() - t0
            lat, lon, hdg = self._pos(t)
            lat_e7, lon_e7 = int(lat * 1e7), int(lon * 1e7)
            try:
                self._conn.recv_match(blocking=False)
            except Exception:
                pass
            try:
                m.heartbeat_send(getattr(_MAV, STUB_MAV_TYPE),
                                 _MAV.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                 _MAV.MAV_MODE_FLAG_SAFETY_ARMED, 10, 4)
                m.global_position_int_send(int(t * 1000) & 0xFFFFFFFF, lat_e7, lon_e7,
                                           120000, 80000, 0, 0, 0, int(hdg * 100))
                m.attitude_send(int(t * 1000) & 0xFFFFFFFF, 0.05 * math.sin(t),
                                0.02, math.radians(hdg), 0.0, 0.0, 0.0)
                m.vfr_hud_send(18.0, 17.0, int(hdg), 55, 120.0, 1.0)
                m.sys_status_send(0, 0, 0, 500, 11000, -1, 87, 0, 0, 0, 0, 0, 0)
                m.gps_raw_int_send(int(t * 1e6) & 0xFFFFFFFF, 3, lat_e7, lon_e7,
                                   120000, 100, 100, 1700, int(hdg * 100), 12)
            except Exception:
                pass
            i += 1
            time.sleep(self.dt)

    def stop(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            if self._conn:
                self._conn.close()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════
#  行程樹 CPU / 記憶體 監測(psutil)
# ═══════════════════════════════════════════════════════════════════════════
class ProcMonitor:
    def __init__(self):
        self.ok = psutil is not None
        self.ncpu = psutil.cpu_count() if self.ok else (os.cpu_count() or 1)
        self._procs = []

    def _tree(self):
        p = psutil.Process(os.getpid())
        procs = [p]
        try:
            procs += p.children(recursive=True)
        except Exception:
            pass
        return procs

    def prime(self):
        if not self.ok:
            return
        self._procs = self._tree()
        for p in self._procs:
            try:
                p.cpu_percent(None)
            except Exception:
                pass

    def read(self):
        """回傳 (cpu_sum_pct, machine_pct, rss_mb)。cpu_sum 為跨核總和。"""
        if not self.ok:
            return (float('nan'), float('nan'), float('nan'))
        cpu = 0.0
        rss = 0
        for p in self._procs:
            try:
                cpu += p.cpu_percent(None)
                rss += p.memory_info().rss
            except Exception:
                pass
        return (cpu, cpu / self.ncpu, rss / 1e6)


# ═══════════════════════════════════════════════════════════════════════════
#  遙測 → 地圖 推送(baseline 每幀直推 / optimized 12Hz 合併)
# ═══════════════════════════════════════════════════════════════════════════
class MapFeeder(QObject):
    def __init__(self, dualmap, optimized: bool):
        super().__init__()
        self.map = dualmap
        self.optimized = optimized
        self._latest = {}                # sysid -> frame(optimized 用)
        self._timer = None
        if optimized:
            # 與 app 同一套:合幀頻率與批次項目都直接取 ui.main_window 的(不在這裡另抄一份)
            from ui.main_window import MainWindow, TELEMETRY_FLUSH_HZ
            self._item = MainWindow._uav_batch_item
            hz = TELEMETRY_FLUSH_HZ if TELEMETRY_FLUSH_HZ > 0 else 30.0
            self._timer = QTimer(self)
            self._timer.setTimerType(Qt.TimerType.PreciseTimer)
            self._timer.timeout.connect(self._flush)
            self._timer.start(int(1000 / hz))

    def on_telemetry(self, frame):
        if not getattr(frame, 'is_valid_gps', lambda: True)():
            return
        if self.optimized:
            self._latest[frame.sysid] = frame      # 只存最新,等 timer flush
        else:
            self._push(frame)                       # baseline:每幀直推(單架舊 API)

    def _flush(self):
        if not self._latest:
            return
        frames = list(self._latest.values())
        self._latest.clear()
        if hasattr(self.map, 'update_uav_positions'):
            self.map.update_uav_positions([self._item(f) for f in frames])
        else:
            for frame in frames:
                self._push(frame)

    def _push(self, frame):
        if not hasattr(self.map, 'update_uav_position'):
            return
        self.map.update_uav_position(
            frame.lat, frame.lon, frame.alt_rel,
            frame.heading, frame.ground_speed,
            sysid=frame.sysid, mode=frame.mode, armed=frame.armed,
            vehicle_type=getattr(frame, 'vehicle_type', ''),
            pitch_deg=getattr(frame, 'pitch', 0.0),
            roll_deg=getattr(frame, 'roll', 0.0),
        )


# ── Cesium page 取得 helper ──────────────────────────────────────────────
def _cesium_page(dualmap):
    m3d = getattr(dualmap, 'map_3d', None)
    return getattr(m3d, '_page', None) if m3d is not None else None


def eval_js(app, page, js, timeout=2.0):
    """同步讀 runJavaScript 結果(spin event loop)。"""
    box = {'v': None, 'done': False}
    page.runJavaScript(js, lambda v: box.__setitem__('v', v) or box.__setitem__('done', True))
    t0 = time.monotonic()
    while not box['done'] and time.monotonic() - t0 < timeout:
        app.processEvents()
        time.sleep(0.004)
    return box['v']


def pump(app, seconds):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        app.processEvents()
        time.sleep(0.003)


_IN_VIEW_JS = r"""
(function(){
  const now = Cesium.JulianDate.now(), c = viewer.scene.canvas, models = new Set();
  const toWin = Cesium.SceneTransforms.worldToWindowCoordinates || Cesium.SceneTransforms.wgs84ToWindowCoordinates;
  let n = 0, inview = 0;
  for (const k in uavEntities) {
    const e = uavEntities[k]; n++;
    const p = e.position && e.position.getValue(now);
    if (p) { const w = toWin(viewer.scene, p);
             if (w && w.x >= 0 && w.y >= 0 && w.x <= c.clientWidth && w.y <= c.clientHeight) inview++; }
    if (e.model && e.model.uri) { const u = e.model.uri.getValue ? e.model.uri.getValue(now) : e.model.uri;
                                  const s = String(u && (u.url || u)); models.add(s.split('/').pop()); }
  }
  return JSON.stringify({n: n, inview: inview, models: Array.from(models)});
})()
"""


def _in_view(app, page) -> dict:
    """畫面內幾架、實際用的模型檔（合併快取 merged-*.glb 或原檔）。"""
    try:
        raw = eval_js(app, page, _IN_VIEW_JS)
        return json.loads(raw) if raw else {}
    except Exception:                                               # noqa: BLE001
        return {}


# ═══════════════════════════════════════════════════════════════════════════
#  一次 ramp(指定 mode)
# ═══════════════════════════════════════════════════════════════════════════
AIRFRAME = ''          # --airframe（main 設定）：VTOL 畫哪個機體模型


def run_ramp(app, mode, steps, dwell, rate, gui, settle, mon):
    from mission.fleet_registry import FleetRegistry
    from mission.sitl_link import SITLLink

    optimized = (mode == 'optimized')
    dualmap = None
    feeder = None
    window = None

    if gui:
        from PyQt6.QtWidgets import QMainWindow
        from ui.widgets.dual_map_widget import DualMapWidget, _MODE_3D
        window = QMainWindow()
        dualmap = DualMapWidget()
        window.setCentralWidget(dualmap)
        window.resize(1400, 900)
        window.show()
        dualmap._switch_mode(_MODE_3D)
        # 等 Cesium viewer 就緒
        page = _cesium_page(dualmap)
        print(f"  [{mode}] 等待 Cesium viewer 就緒 ...")
        t0 = time.monotonic()
        ready = False
        while time.monotonic() - t0 < 40.0:
            pump(app, 0.2)
            if page is not None and eval_js(app, page, "typeof viewer!=='undefined' && !!viewer"):
                ready = True
                break
        if not ready:
            print(f"  [{mode}] [X] Cesium 未就緒(40s);可能缺 assets/cesium 或 WebGL 不可用")
            return []
        # 注入 FPS hook +(optimized)requestRenderMode
        eval_js(app, page, """
            window.__viewer = (typeof viewer!=='undefined')?viewer:null;
            window.__fc = 0;
            if (window.__viewer && !window.__fcHook){
                window.__fcHook = true;
                window.__viewer.scene.postRender.addEventListener(function(){ window.__fc++; });
            }
            'ok';
        """)
        # VTOL 畫哪個機體模型由這裡明確決定（--airframe，預設 '' ＝ 通用 vtol.glb ＝ 歷來基準），不隨 settings.json
        # 的 model3d 走（2026-10-23 審查：本機設了 hero ⇒ --vehicle vtol 會默默量成 hero.glb）
        dualmap.set_uav_airframes({}, default=AIRFRAME)
        # (以前 optimized 會在這裡注入 requestRenderMode=true;頁面現在自己做自適應算圖,不再覆寫)
        # 等 3D 安定窗口結束(DualMapWidget 切 3D 後會丟棄更新一段時間)
        t0 = time.monotonic()
        while getattr(dualmap, '_map3d_settling', False) and time.monotonic() - t0 < 8.0:
            pump(app, 0.2)
        feeder = MapFeeder(dualmap, optimized)
        # 鏡頭對準第一群（2026-10-17 審查：以前鏡頭停在預設起始點，樁機全在 28 km 外、全被視錐剔除，
        # 量到的 FPS 跟模型重量無關）。setViewPos：俯 40°、距離 1400 m ⇒ 300 m 半徑的繞圈整個在畫面裡。
        clat, clon = GROUP_CENTERS[0]
        eval_js(app, page, f"setViewPos({clat}, {clon}, 120, 1400); 'ok';")
        print(f"  [{mode}] Cesium 就緒,鏡頭對準第一群,開始遞增。")

    reg = FleetRegistry.instance()
    stubs, links = [], []
    rows = []
    page = _cesium_page(dualmap) if gui else None

    def add_vehicle(spec):
        st = MovingMavStub(spec, rate)
        st.start()
        stubs.append(st)
        lk = SITLLink(conn_str=f"tcp:127.0.0.1:{spec['port']}",
                      sysid_label=spec['sysid'], vehicle_hint='PLANE')
        if feeder is not None:
            lk.telemetry.connect(feeder.on_telemetry)
        reg.register(f"UAV-{spec['sysid']}", lk, spec['sysid'])
        lk.start()
        links.append(lk)

    plan = vehicle_plan(max(steps))
    for target in steps:
        while len(links) < target:
            add_vehicle(plan[len(links)])
        # 連線 + 編隊軌跡穩定
        pump(app, settle)
        # 量測視窗
        mon.prime()
        fc0 = eval_js(app, page, "window.__fc||0") if page else 0
        t0 = time.monotonic()
        pump(app, dwell)
        wall = time.monotonic() - t0
        fc1 = eval_js(app, page, "window.__fc||0") if page else 0
        cpu, machine, rss = mon.read()
        fps = (fc1 - fc0) / wall if page else float('nan')
        groups = (target + GROUP_SIZE - 1) // GROUP_SIZE
        seen = _in_view(app, page) if page else {}
        rows.append(dict(n=target, groups=groups, fps=fps, cpu=cpu,
                         machine=machine, rss=rss, inview=seen.get('inview', ''),
                         models='|'.join(seen.get('models', []))))
        fps_s = f"{fps:5.1f}" if page else "  n/a"
        print(f"  [{mode}] N={target:>2} ({groups}群) | FPS {fps_s} | "
              f"CPU {cpu:6.1f}% (整機 {machine:4.1f}%) | RSS {rss:6.0f} MB | "
              f"畫面內 {seen.get('inview', '?')} 架 | 模型 {seen.get('models', [])}")

    # 拆除
    for lk in links:
        lk.stop()
    for lk in links:
        lk.wait(3000)
    for st in stubs:
        st.stop()
    for spec in plan[:len(links)]:
        reg.unregister(f"UAV-{spec['sysid']}")
    if window is not None:
        window.close()
    pump(app, 0.3)
    return rows


# ═══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="AeroPlan 3D 地圖多機效能 profiler")
    ap.add_argument('--mode', choices=['baseline', 'optimized', 'both'], default='both')
    ap.add_argument('--steps', default='1,2,4,6,8,10,12')
    ap.add_argument('--dwell', type=float, default=6.0, help='每階段量測視窗秒數')
    ap.add_argument('--settle', type=float, default=3.0, help='加台後穩定秒數')
    ap.add_argument('--rate', type=float, default=10.0, help='每台遙測 Hz')
    ap.add_argument('--no-gui', action='store_true', help='不開地圖,只量 CPU(無頭驗證)')
    ap.add_argument('--vehicle', choices=['plane', 'vtol', 'copter'], default='plane',
                    help='樁機回報的機型（決定 3D 用哪個模型）')
    ap.add_argument('--airframe', choices=['', 'hero', 'pioneer'], default='',
                    help="--vehicle vtol 時 3D 畫哪個機體模型：''＝通用 vtol.glb（預設、歷來基準）／hero／pioneer")
    ap.add_argument('--csv', default='', help='CSV 輸出路徑(預設 tools/swarm_perf_result.csv,那是被追蹤的檔)')
    args = ap.parse_args()
    global AIRFRAME
    AIRFRAME = args.airframe

    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

    global STUB_MAV_TYPE
    STUB_MAV_TYPE = {'plane': 'MAV_TYPE_FIXED_WING', 'vtol': 'MAV_TYPE_VTOL_QUADROTOR',
                     'copter': 'MAV_TYPE_QUADROTOR'}[args.vehicle]
    steps = [int(x) for x in args.steps.split(',') if x.strip()]
    gui = not args.no_gui
    modes = ['baseline', 'optimized'] if args.mode == 'both' else [args.mode]

    if gui:
        # QtWebEngine 規定:WebEngine 必須在 QApplication 建立「之前」初始化,
        # 否則 import 時拋 AA_ShareOpenGLContexts 錯誤。
        from PyQt6.QtCore import Qt
        from PyQt6.QtWidgets import QApplication
        QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
        # 與 main.py 相同的 Qt 呈現設定(Windows 預設 swapInterval 0;AEROPLAN_QT_SWAP_INTERVAL=1 量舊行為)
        from ui.render_setup import configure_qt_presentation
        print(f"Qt swapInterval = {configure_qt_presentation()}(None = Qt 預設)")
        import PyQt6.QtWebEngineWidgets  # noqa: F401  先載入 WebEngine
        app = QApplication.instance() or QApplication(sys.argv)
    else:
        app = QCoreApplication.instance() or QCoreApplication(sys.argv)

    mon = ProcMonitor()
    print("=" * 78)
    print(f"AeroPlan 蜂群效能 profiler | GUI={'on' if gui else 'off'} | "
          f"steps={steps} | dwell={args.dwell}s | rate={args.rate}Hz | "
          f"CPU={'psutil' if mon.ok else 'N/A(請 pip install psutil)'} | cores={mon.ncpu}")
    if not gui:
        print("!! --no-gui:不開 3D 地圖,FPS=n/a,僅驗證遙測/監測機制 + Python/連線 CPU。")
    print("=" * 78)

    results = {}
    for mode in modes:
        print(f"\n── ramp: {mode} ──")
        results[mode] = run_ramp(app, mode, steps, args.dwell, args.rate, gui,
                                 args.settle, mon)

    # 對比表
    print("\n" + "=" * 78)
    print("結果總表")
    print("=" * 78)
    if gui and 'baseline' in results and 'optimized' in results \
            and results['baseline'] and results['optimized']:
        print(f"{'N台':>4} {'群':>2} | {'base FPS':>9} {'opt FPS':>9} {'dFPS':>7} | "
              f"{'base CPU%':>9} {'opt CPU%':>9}")
        print("-" * 78)
        bmap = {r['n']: r for r in results['baseline']}
        omap = {r['n']: r for r in results['optimized']}
        for n in steps:
            b, o = bmap.get(n), omap.get(n)
            if not b or not o:
                continue
            dfps = o['fps'] - b['fps']
            print(f"{n:>4} {b['groups']:>2} | {b['fps']:>9.1f} {o['fps']:>9.1f} "
                  f"{dfps:>+7.1f} | {b['cpu']:>9.1f} {o['cpu']:>9.1f}")
    else:
        for mode, rows in results.items():
            print(f"\n[{mode}]")
            print(f"{'N台':>4} {'群':>2} | {'FPS':>7} | {'CPU%(跨核)':>11} | "
                  f"{'整機%':>6} | {'RSS MB':>7}")
            print("-" * 60)
            for r in rows:
                fps_s = f"{r['fps']:7.1f}" if not math.isnan(r['fps']) else "    n/a"
                print(f"{r['n']:>4} {r['groups']:>2} | {fps_s} | {r['cpu']:>11.1f} | "
                      f"{r['machine']:>6.1f} | {r['rss']:>7.0f}")

    # CSV
    out_csv = args.csv or os.path.join(_THIS, 'swarm_perf_result.csv')
    try:
        with open(out_csv, 'w', encoding='utf-8') as fh:
            # 條件欄（2026-10-17 審查：以前 CSV 看不出量的是哪種設定）
            from PyQt6.QtGui import QSurfaceFormat
            swap = QSurfaceFormat.defaultFormat().swapInterval()
            merge = os.environ.get('AEROPLAN_GLB_MERGE', '1')
            fh.write("mode,n_vehicles,groups,fps,cpu_sum_pct,machine_pct,rss_mb,"
                     "vehicle,airframe,swap_interval,glb_merge_env,in_view,models\n")
            for mode, rows in results.items():
                for r in rows:
                    fh.write(f"{mode},{r['n']},{r['groups']},{r['fps']:.2f},"
                             f"{r['cpu']:.2f},{r['machine']:.2f},{r['rss']:.1f},"
                             f"{args.vehicle},{args.airframe},{swap},{merge},{r.get('inview', '')},{r.get('models', '')}\n")
        print(f"\nCSV 已輸出: {out_csv}")
    except Exception as e:
        print(f"CSV 輸出失敗: {e}")


if __name__ == '__main__':
    main()
