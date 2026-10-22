# -*- coding: utf-8 -*-
"""telemetry_bench.py — 多機遙測在 GUI 執行緒上的成本（無頭、不開 SITL、不建 MainWindow）。

量的是「一幀遙測進到主執行緒之後」真正會做的事，分三段各自計時：
  hud   ：真的 SITLHud（offscreen）吃 N 架 × F Hz 的 TelemetryFrame，含機卡重繪（WA_DontShowOnScreen + show）。
  map2d ：裸 MapWidget.update_uav_position（記錄 runJavaScript 的次數與位元組，不開 QtWebEngine）。
  map3d ：裸 CesiumMapWidget.update_uav_position（同上，記 _js）。
  window：MainWindow._on_sitl_telemetry 綁到假 self（真 SITLHud ＋ 記錄用假 map_widget）。

用法：
  python tools/telemetry_bench.py                       # N=1,3,6；F=25 Hz（每條鏈路每秒 emit 的量級）；每段 4 s
  python tools/telemetry_bench.py --n 6 --hz 40 --seconds 6 --json runs/artifacts/PERF/bench.json

輸出每段：每幀主執行緒毫秒、每秒主執行緒佔用百分比、JS 呼叫數與位元組／秒。
⚠ 這是**相對**量尺（同一台機器前後對照用），不是 FPS；真 FPS 用 tools/swarm_perf_profiler.py（要開 3D 地圖）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import pathlib

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import logging
logging.disable(logging.INFO)

import ui.main_window  # noqa: E402,F401  QtWebEngine 匯入順序
from PyQt6.QtCore import Qt  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from mission.sitl_link import TelemetryFrame  # noqa: E402


def make_frame(sysid: int, t: float) -> TelemetryFrame:
    """繞圈飛的合成遙測（每幀都不同，避免被「沒變就不畫」優化騙過）。"""
    w = 0.3 + 0.05 * sysid
    return TelemetryFrame(
        sysid=sysid, lat=23.45 + 0.002 * math.sin(w * t + sysid), lon=120.28 + 0.002 * math.cos(w * t + sysid),
        alt_msl=108.0 + 5 * math.sin(t), alt_rel=100.0 + 5 * math.sin(t), heading=(w * t * 57.3 + sysid * 40) % 360,
        ground_speed=18.0 + math.sin(t), vel_n=1.0, vel_e=2.0, vel_d=0.1, air_speed=19.0, climb=0.2 * math.cos(t),
        roll=15 * math.sin(2 * t), pitch=3 * math.cos(t), yaw=0.0, throttle=55, battery_v=23.1, battery_pct=78,
        battery_a=12.0, gps_fix=3, gps_sats=14, hdop=0.8, mode='AUTO', armed=True, vehicle_type='VTOL',
    )


class JsRecorder:
    def __init__(self):
        self.calls = 0
        self.bytes = 0

    def runJavaScript(self, js, *a, **k):
        self.calls += 1
        self.bytes += len(js)

    def reset(self):
        self.calls = 0
        self.bytes = 0


def _bare(cls, **attrs):
    w = cls.__new__(cls)
    for k, v in attrs.items():
        setattr(w, k, v)
    return w


def bench_stage(name: str, n: int, hz: float, seconds: float, feed, app, recorder=None, flush_counter=None,
                checks=None) -> dict:
    """feed(frame) 餵一幀；以 1/hz 的節奏輪流餵 N 架；量 feed 的累積主執行緒時間與 processEvents 時間。

    事件迴圈每 ≤2 ms 抽一次（與餵幀節奏脫鉤）：合幀的 QTimer 才會以真實的 TELEMETRY_FLUSH_HZ 觸發，
    不然 flush 會被量化到餵幀的格子上、成本被低估一半（審查 2026-10-17）。
    flush_counter：可呼叫 → 回目前 flush 次數（window 段用來印 flushes/s 並核對 JS 呼叫數）。
    checks：段結束後呼叫，回 dict（例如卡片數）併進結果；不符預期就 raise，不讓「掉幀」被當成變快。
    """
    period = 1.0 / hz
    frames = 0
    t_feed = 0.0
    t_events = 0.0
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    if recorder:
        recorder.reset()
    f0 = flush_counter() if flush_counter else 0
    while time.perf_counter() < t_end:
        now = time.perf_counter()
        if now >= next_t:
            for sid in range(1, n + 1):
                f = make_frame(sid, now)
                t0 = time.perf_counter()
                feed(f)
                t_feed += time.perf_counter() - t0
                frames += 1
            next_t += period
        t0 = time.perf_counter()
        app.processEvents()
        t_events += time.perf_counter() - t0
        time.sleep(0.002)
    busy = t_feed + t_events
    out = {'stage': name, 'n': n, 'hz': hz, 'seconds': seconds, 'frames': frames,
           'ms_per_frame_feed': 1000 * t_feed / max(frames, 1),
           'ms_per_frame_total': 1000 * busy / max(frames, 1),
           'gui_thread_busy_pct': 100 * busy / seconds}
    if recorder:
        out.update({'js_calls_per_s': recorder.calls / seconds, 'js_kb_per_s': recorder.bytes / 1024 / seconds})
    if flush_counter:
        flushes = flush_counter() - f0
        out['flushes_per_s'] = flushes / seconds
        if recorder and flushes and abs(recorder.calls - flushes) > max(2, 0.1 * flushes):
            raise RuntimeError(f'{name}: JS 呼叫數 {recorder.calls} 與 flush 數 {flushes} 不符 —— 合幀路徑有幀被丟掉或多畫')
    if checks:
        out.update(checks())
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=str, default='1,3,6')
    ap.add_argument('--hz', type=float, default=25.0, help='每架每秒幀數（鏈路 emit 的量級）')
    ap.add_argument('--seconds', type=float, default=4.0)
    ap.add_argument('--json', default='')
    ap.add_argument('--stages', default='hud,map2d,map3d,window')
    args = ap.parse_args()
    app = QApplication.instance() or QApplication([])
    from ui.widgets.sitl_hud import SITLHud
    from ui.widgets.map_widget import MapWidget
    from ui.widgets.cesium_map_widget import CesiumMapWidget
    from ui.main_window import MainWindow
    import types

    results = []
    stages = args.stages.split(',')
    for n in [int(x) for x in args.n.split(',')]:
        if 'hud' in stages:
            hud = SITLHud()
            hud.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
            hud.resize(440, 1100)
            hud.show()
            app.processEvents()
            results.append(bench_stage('hud', n, args.hz, args.seconds, hud.on_telemetry, app))
            hud.hide(); hud.deleteLater(); app.processEvents()
        if 'map2d' in stages:
            rec = JsRecorder()
            m2 = _bare(MapWidget, custom_page=rec)
            results.append(bench_stage('map2d', n, args.hz, args.seconds,
                                       lambda f: m2.update_uav_position(f.lat, f.lon, f.alt_rel, f.heading, f.ground_speed,
                                                                        sysid=f.sysid, mode=f.mode, armed=f.armed,
                                                                        vehicle_type=f.vehicle_type),
                                       app, rec))
        if 'map3d' in stages:
            rec3 = JsRecorder()
            m3 = _bare(CesiumMapWidget)
            m3._js = rec3.runJavaScript
            m3.check_bullseye_hit = lambda *a, **k: None
            results.append(bench_stage('map3d', n, args.hz, args.seconds,
                                       lambda f: m3.update_uav_position(f.lat, f.lon, f.alt_rel, f.heading, f.ground_speed,
                                                                        sysid=f.sysid, mode=f.mode, armed=f.armed,
                                                                        vehicle_type=f.vehicle_type, pitch_deg=f.pitch,
                                                                        roll_deg=f.roll),
                                       app, rec3))
        if 'window' in stages:
            hud = SITLHud()
            hud.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
            hud.resize(440, 1100); hud.show(); app.processEvents()
            rec = JsRecorder()
            m2 = _bare(MapWidget, custom_page=rec)
            fake_map = types.SimpleNamespace(
                update_uav_position=lambda *a, **k: m2.update_uav_position(*a, **{kk: vv for kk, vv in k.items()
                                                                                 if kk not in ('pitch_deg', 'roll_deg')}))
            win = types.SimpleNamespace(sitl_hud=hud, map_widget=fake_map, _fov_cone_enabled=False, _rcs_enabled=False,
                                        _sitl_links=[], statusBar=lambda: types.SimpleNamespace(showMessage=lambda *a: None))
            for name in ('_on_sitl_telemetry', '_note_armed_transition', '_armed_seen', '_hud_log_msg', '_log_dl_state',
                         '_update_tactical_on_uav_move', '_tele_state', '_flush_telemetry', 'LOG_DOWNLOAD_AUTO_DELAY_S'):
                attr = getattr(MainWindow, name)
                setattr(win, name, types.MethodType(attr, win) if callable(attr) else attr)
            win._uav_batch_item = MainWindow._uav_batch_item      # staticmethod
            # 新版 map_widget 有批次 API：假 map_widget 也提供，否則會退回逐架舊路徑而量不到批次效果
            fake_map.update_uav_positions = lambda batch: m2.update_uav_positions(batch)
            # flush 只畫「還連著」的機：給假 self 每架一條假鏈路
            win._sitl_links = [types.SimpleNamespace(sysid_label=s) for s in range(1, n + 1)]
            flushes = [0]
            real_flush = win._flush_telemetry

            def counted_flush():
                n_done = real_flush()
                if n_done:                      # 只數真的處理到幀的 flush（連鎖續排的空 flush 不算）
                    flushes[0] += 1
                return n_done
            win._flush_telemetry = counted_flush

            def checks():
                cards = len(hud._uav_cards)
                if cards != n:
                    raise RuntimeError(f'window: 機卡 {cards} 張 ≠ {n} 架 —— HUD 更新沒發生，數字不可信')
                return {'cards': cards}
            results.append(bench_stage('window', n, args.hz, args.seconds, win._on_sitl_telemetry, app, rec,
                                       flush_counter=lambda: flushes[0], checks=checks))
            hud.hide(); hud.deleteLater(); app.processEvents()

    print(f'{"stage":7s} {"n":>2s} {"hz":>4s} {"frames":>6s} {"ms/f feed":>9s} {"ms/f tot":>8s} {"busy%":>6s} '
          f'{"js/s":>7s} {"kB/s":>8s} {"flush/s":>8s}')
    for r in results:
        print(f'{r["stage"]:7s} {r["n"]:2d} {r["hz"]:4.0f} {r["frames"]:6d} {r["ms_per_frame_feed"]:9.3f} '
              f'{r["ms_per_frame_total"]:8.3f} {r["gui_thread_busy_pct"]:6.1f} '
              f'{r.get("js_calls_per_s", 0):7.1f} {r.get("js_kb_per_s", 0):8.1f} {r.get("flushes_per_s", 0):8.1f}')
    if args.json:
        p = pathlib.Path(args.json); p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps({'args': vars(args), 'results': results}, ensure_ascii=False, indent=1), encoding='utf-8')
        print('saved', p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
