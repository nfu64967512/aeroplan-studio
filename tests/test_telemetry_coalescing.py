# -*- coding: utf-8 -*-
"""tests/test_telemetry_coalescing.py — 多機遙測合幀（2026-10-17 效能）。

使用者：「多機跑起來不順」「畫面要流暢、SITL 的 Hz 要高、3D 機頭要跟著航線」。量到的事實（tools/telemetry_bench.py
基準）：6 架 × 25 Hz 時主執行緒每秒 150 幀，每幀重繪一張卡片＋兩段 ~2 KB 的 2D JS（連 SVG 一起送、每幀重建 L.divIcon）
＋一段 3D JS ⇒ 2D 每秒 300 次 runJavaScript、465 kB。現在 `_on_sitl_telemetry` 只記每架最新一幀，`_flush_telemetry`
以 TELEMETRY_FLUSH_HZ（30）合幀推地圖、HUD 機卡另以 HUD_MAX_HZ（15）限頻。

鎖的是：
① 一個窗口內多幀 ⇒ 只排一次 flush；flush 時 HUD 每架一次（最新那幀）、地圖一個批次一次。
② 便宜的狀態觀察（解鎖→上鎖）每幀都做，不會因為合幀漏掉轉換。
③ GPS 無效的幀不進地圖批次，但仍進 HUD。
④ 只畫還連著的機：逐機斷線（真的跑 _on_sitl_link_disconnected）與全域斷線（真的跑 on_sitl_disconnect）之後，
   已排的 flush 與斷線後才送到的幀都不會畫出幽靈卡片／地圖標記（審查 2026-10-17 抓到）。
⑤ 機頭用 ATTITUDE yaw、航跡用 NED 速度；HUD 限頻但離散狀態不限。
⑥ 沒有批次 API 的地圖替身退回逐架 update_uav_position；Hz=0 同步；環境變數接線（AST）。
"""
from __future__ import annotations

import ast
import os
import pathlib
import types

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

import ui.main_window as mw  # noqa: E402
from ui.main_window import MainWindow  # noqa: E402
from mission.sitl_link import TelemetryFrame  # noqa: E402


class _Hud:
    def __init__(self):
        self.frames = []
        self.msgs = []
        self.removed = []
        self.disconnected = []
        self.lbl_status = types.SimpleNamespace(setText=lambda t: self.msgs.append(('status', t)))

    def on_telemetry(self, f):
        self.frames.append(f)

    def on_log_message(self, *a):
        self.msgs.append(a)

    def auto_log_download_enabled(self):
        return False

    def remove_card(self, sysid):
        self.removed.append(sysid)

    def on_disconnected(self, reason=''):
        self.disconnected.append(reason)


class _BatchMap:
    def __init__(self):
        self.batches = []
        self.cleared = []

    def update_uav_positions(self, batch):
        self.batches.append(list(batch))

    def clear_uav(self, sysid=None):
        self.cleared.append(sysid)


class _OldMap:
    def __init__(self):
        self.calls = []

    def update_uav_position(self, *a, **k):
        self.calls.append((a, k))


class _Sig:
    def disconnect(self, *a):
        pass


class _Link:
    def __init__(self, sysid):
        self.sysid_label = sysid
        self.conn_str = f'tcp:127.0.0.1:{5760 + 10 * sysid}'
        self.disconnected = _Sig()
        self.stopped = False

    def stop(self):
        self.stopped = True

    def wait(self, ms):
        pass


_BOUND = ('_on_sitl_telemetry', '_tele_state', '_flush_telemetry', '_note_armed_transition', '_armed_seen',
          '_hud_log_msg', '_log_dl_state', '_update_tactical_on_uav_move', '_on_sitl_link_disconnected',
          'on_sitl_disconnect', '_update_write_wp_enabled')


def _win(map_widget, sysids=(1, 2, 3)):
    w = types.SimpleNamespace()
    w.sitl_hud = _Hud()
    w.map_widget = map_widget
    w._fov_cone_enabled = False
    w._rcs_enabled = False
    w._sitl_links = [_Link(s) for s in sysids]
    w._sitl_link = w._sitl_links[0] if w._sitl_links else None
    w._hil_links = []
    w.hil_panel = types.SimpleNamespace(on_disconnected=lambda: None)
    w._write_pending_sysids = set()
    w._status = []
    w.statusBar = lambda: types.SimpleNamespace(showMessage=lambda m, *a: w._status.append(m))
    w.LOG_DOWNLOAD_AUTO_DELAY_S = MainWindow.LOG_DOWNLOAD_AUTO_DELAY_S
    for name in _BOUND:
        setattr(w, name, types.MethodType(getattr(MainWindow, name), w))
    w._uav_batch_item = MainWindow._uav_batch_item
    return w


def _frame(sysid, lat=23.45, armed=True, gps=3, mode='AUTO'):
    return TelemetryFrame(sysid=sysid, lat=lat, lon=120.28, alt_rel=100.0, heading=90.0, yaw=90.0, ground_speed=18.0,
                          vel_n=0.0, vel_e=18.0, gps_fix=gps, gps_sats=12, mode=mode, armed=armed, vehicle_type='VTOL',
                          roll=1.0, pitch=2.0)


@pytest.fixture
def timer(monkeypatch):
    fired = []
    monkeypatch.setattr(mw.QTimer, 'singleShot', staticmethod(lambda ms, *a: fired.append((ms, a[-1]))))   # (ms, [type,] fn)
    monkeypatch.setattr(mw, 'TELEMETRY_FLUSH_HZ', 30.0)
    monkeypatch.setattr(mw, 'HUD_MAX_HZ', 0.0)      # 這裡測合幀本身；HUD 限頻另測
    return fired


def test_many_frames_one_flush_latest_wins(timer):
    m = _BatchMap()
    w = _win(m)
    for i in range(10):
        for sid in (1, 2, 3):
            w._on_sitl_telemetry(_frame(sid, lat=23.45 + 0.001 * i))
    assert len(timer) == 1 and timer[0][0] == int(1000 / 30), '一個窗口內只能排一次 flush'
    assert w.sitl_hud.frames == [] and m.batches == [], 'flush 之前不畫'
    n = timer[0][1]()
    assert n == 3
    assert [f.sysid for f in w.sitl_hud.frames] == [1, 2, 3]
    assert all(abs(f.lat - (23.45 + 0.009)) < 1e-9 for f in w.sitl_hud.frames), '要畫最新那幀'
    assert len(m.batches) == 1 and [b['sysid'] for b in m.batches[0]] == [1, 2, 3]
    item = m.batches[0][0]
    assert set(item) == {'sysid', 'lat', 'lon', 'alt', 'hdg', 'crs', 'spd', 'mode', 'armed', 'vt', 'pitch', 'roll'}
    w._on_sitl_telemetry(_frame(1))
    assert len(timer) == 2, 'flush 後再來一幀 ⇒ 再排一次'


def test_armed_transition_is_seen_every_frame_not_only_at_flush(timer):
    w = _win(_BatchMap())
    w._on_sitl_telemetry(_frame(1, armed=True))
    w._on_sitl_telemetry(_frame(1, armed=False))     # 同一個窗口內解鎖→上鎖
    w._on_sitl_telemetry(_frame(1, armed=True))
    assert any('已上鎖（落地）' in m for m in w._status), '合幀不可以吃掉落地轉換'


def test_invalid_gps_goes_to_hud_but_not_to_the_map(timer):
    m = _BatchMap()
    w = _win(m)
    w._on_sitl_telemetry(_frame(1, gps=0))
    w._on_sitl_telemetry(_frame(2, gps=3))
    timer[0][1]()
    assert [f.sysid for f in w.sitl_hud.frames] == [1, 2]
    assert [b['sysid'] for b in m.batches[0]] == [2]


def test_old_style_map_without_batch_api_still_gets_per_uav_calls(timer):
    m = _OldMap()
    w = _win(m)
    w._on_sitl_telemetry(_frame(1)); w._on_sitl_telemetry(_frame(2))
    timer[0][1]()
    assert [c[1]['sysid'] for c in m.calls] == [1, 2]
    assert m.calls[0][1]['pitch_deg'] == 2.0 and m.calls[0][1]['vehicle_type'] == 'VTOL'


def test_hz_zero_means_synchronous_old_behaviour(monkeypatch):
    monkeypatch.setattr(mw, 'TELEMETRY_FLUSH_HZ', 0.0)
    monkeypatch.setattr(mw, 'HUD_MAX_HZ', 0.0)
    fired = []
    monkeypatch.setattr(mw.QTimer, 'singleShot', staticmethod(lambda ms, *a: fired.append(a[-1])))
    m = _BatchMap()
    w = _win(m)
    w._on_sitl_telemetry(_frame(1))
    assert fired == [] and len(w.sitl_hud.frames) == 1 and len(m.batches) == 1


# ── ④ 斷線後不畫幽靈 ───────────────────────────────────────────────────
def test_per_link_disconnect_really_drops_that_aircraft(timer):
    """真的跑 _on_sitl_link_disconnected（不是手動清快取）：flush 只剩還連著的機。"""
    m = _BatchMap()
    w = _win(m, sysids=(1, 2))
    link2 = w._sitl_links[1]
    w._on_sitl_telemetry(_frame(1)); w._on_sitl_telemetry(_frame(2))
    w._on_sitl_link_disconnected(link2, '測試')
    assert link2 not in w._sitl_links and w.sitl_hud.removed == [2]
    n = timer[0][1]()
    assert n == 1 and [f.sysid for f in w.sitl_hud.frames] == [1]
    assert [b['sysid'] for b in m.batches[0]] == [1]


def test_global_disconnect_leaves_nothing_for_the_pending_flush(timer):
    """全域斷線（真的跑 on_sitl_disconnect）後：已排的 flush 畫 0 架，斷線後才送到的幀也不畫（沒鏈路了）。"""
    m = _BatchMap()
    w = _win(m, sysids=(1, 2, 3))
    for sid in (1, 2, 3):
        w._on_sitl_telemetry(_frame(sid))
    pending = timer[0][1]
    w.on_sitl_disconnect()
    assert w._sitl_links == [] and w.sitl_hud.disconnected and m.cleared == [None]
    assert pending() == 0 and w.sitl_hud.frames == [] and m.batches == [], '幽靈卡片'
    # 鏈路執行緒排隊中的最後一幀這時才送到
    w._on_sitl_telemetry(_frame(2))
    assert timer[-1][1]() == 0 and w.sitl_hud.frames == [], '斷線後的尾幀也不能畫'


def test_every_disconnect_path_forgets_the_telemetry_cache():
    """AST：四條路（逐機、全域、HIL 斷開全部、關窗）都呼叫 _forget_tele_state。"""
    src = pathlib.Path(mw.__file__).read_text(encoding='utf-8', errors='replace')
    tree = ast.parse(src)
    for name, arg in (('_on_sitl_link_disconnected', 'sysid'), ('on_sitl_disconnect', 'None'),
                      ('_on_hil_disconnect_all', None), ('closeEvent', 'None')):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == '_forget_tele_state']
        assert len(calls) == 1, f'{name} 沒有（或多次）呼叫 _forget_tele_state'
        if arg is not None:
            assert ast.unparse(calls[0].args[1]) == arg, (name, ast.unparse(calls[0]))


def test_forget_tele_state_helper():
    w = types.SimpleNamespace()
    w._tele = {'latest': {1: 'a', 2: 'b'}, 'dirty': {1, 2}, 'pending': True, 'hud_t': {1: 0.0}, 'hud_state': {1: ()}}
    mw._forget_tele_state(w, 2)
    assert w._tele['latest'] == {1: 'a'} and w._tele['dirty'] == {1}
    mw._forget_tele_state(w, None)
    assert w._tele['latest'] == {} and w._tele['dirty'] == set() and w._tele['hud_t'] == {}
    mw._forget_tele_state(types.SimpleNamespace(), 1)      # 沒有快取也不炸


# ── ⑤ 機頭／航跡與 HUD 限頻 ─────────────────────────────────────────────
def test_batch_item_heading_comes_from_attitude_yaw_and_course_from_velocity():
    f = TelemetryFrame(sysid=1, lat=23.45, lon=120.28, heading=91.0, yaw=-89.5, vel_n=10.0, vel_e=10.0,
                       ground_speed=14.1, gps_fix=3)
    it = MainWindow._uav_batch_item(f)
    assert abs(it['hdg'] - 270.5) < 1e-9, 'yaw -89.5° 要正規化成 270.5°'
    assert abs(it['crs'] - 45.0) < 1e-9, '航跡 = atan2(東, 北)'
    slow = TelemetryFrame(sysid=1, heading=120.0, yaw=0.0, vel_n=0.1, vel_e=0.0, gps_fix=3)
    it2 = MainWindow._uav_batch_item(slow)
    assert it2['hdg'] == 120.0, '沒收過 ATTITUDE（yaw 0 而 heading 非 0）就退回 heading'
    assert it2['crs'] == it2['hdg'], '慢速（< 0.5 m/s）沒有可信的航跡 ⇒ 退回機頭'


def test_hud_repaint_is_capped_but_discrete_changes_pass(timer, monkeypatch):
    monkeypatch.setattr(mw, 'HUD_MAX_HZ', 10.0)
    t = {'now': 100.0}
    monkeypatch.setattr(mw.time, 'monotonic', lambda: t['now'])
    m = _BatchMap()
    w = _win(m)
    w._on_sitl_telemetry(_frame(1, lat=23.45)); w._flush_telemetry()
    t['now'] += 0.02
    w._on_sitl_telemetry(_frame(1, lat=23.46)); w._flush_telemetry()     # 20 ms 後：地圖推、HUD 不畫
    assert len(m.batches) == 2 and len(w.sitl_hud.frames) == 1
    t['now'] += 0.02
    w._on_sitl_telemetry(_frame(1, lat=23.47, mode='RTL')); w._flush_telemetry()   # 模式變了 ⇒ 立刻畫
    assert len(w.sitl_hud.frames) == 2 and w.sitl_hud.frames[-1].mode == 'RTL'
    t['now'] += 0.2
    w._on_sitl_telemetry(_frame(1, lat=23.48, mode='RTL')); w._flush_telemetry()   # 過了 1/10 s ⇒ 畫
    assert len(w.sitl_hud.frames) == 3


# ── ⑥ 環境變數接線（AST，不是只測 helper）────────────────────────────────
def test_rate_constants_are_wired_to_their_env_vars():
    src = pathlib.Path(mw.__file__).read_text(encoding='utf-8', errors='replace')
    tree = ast.parse(src)
    found = {}
    for n in tree.body:
        if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name) and n.value is not None:
            found[n.target.id] = ast.unparse(n.value)
    assert found.get('TELEMETRY_FLUSH_HZ', '').startswith("_env_float('AEROPLAN_TELEMETRY_HZ', ")
    assert found.get('HUD_MAX_HZ', '').startswith("_env_float('AEROPLAN_HUD_HZ', ")
    assert mw._env_float('AEROPLAN_TELEMETRY_HZ_NOPE', 7.5) == 7.5


# ── ⑦ 每一則姿態都送進 3D（2026-10-22：50 Hz 姿態、30 Hz 合幀）──────────────────────────
def _att_frame(sysid, att_t, att_ts=0.0, roll=0.0, pitch=0.0, yaw=0.0, gps=3, lat=23.45):
    return TelemetryFrame(sysid=sysid, lat=lat, lon=120.28, alt_rel=100.0, gps_fix=gps, mode='QLOITER', armed=True,
                          vehicle_type='VTOL', roll=roll, pitch=pitch, yaw=yaw, att_t=att_t, att_ts=att_ts,
                          pos_t=1_700_000_000.0)


def test_every_attitude_between_two_flushes_rides_along_older_than_the_latest(timer):
    """鏈路每一則訊息都送一幀快照（位置／VFR／心跳的幀帶著同一個 att_t）：只有 att_t 變了才是一則新姿態。flush 時
    最新一則照舊放在項目本身（at = 飛控時間換到牆鐘的 att_ts），更舊的依時間放進 atts；同一則不重複、不重送。"""
    m = _BatchMap()
    w = _win(m, sysids=(1,))
    t0 = 1_700_000_000.0
    for k in range(4):                                       # 4 則姿態（20 ms 一則），每則後面跟一幀位置／VFR
        f = _att_frame(1, t0 + 0.02 * k + 0.013, t0 + 0.02 * k, roll=k, pitch=-k, yaw=-170.0 + k)
        w._on_sitl_telemetry(f)
        w._on_sitl_telemetry(_att_frame(1, f.att_t, f.att_ts, roll=k, pitch=-k, yaw=-170.0 + k, lat=23.46))
    timer[0][1]()
    item = m.batches[0][0]
    assert item['at'] == int(round((t0 + 0.06) * 1000)), '最新一則；樣本時間 = att_ts（不是收到時刻 att_t）'
    assert item['atts'] == [[pytest.approx((t0 + 0.02 * k) * 1000), float(k), float(-k), (-170.0 + k) % 360.0]
                            for k in range(3)], item['atts']
    assert all(a[0] < item['at'] for a in item['atts'])
    w._on_sitl_telemetry(_att_frame(1, t0 + 0.073, t0 + 0.06, roll=3, pitch=-3, yaw=-167.0, lat=23.47))   # 沒有新姿態
    w._flush_telemetry()
    assert 'atts' not in m.batches[1][0] and m.batches[1][0]['at'] == item['at'], '同一則姿態不會再送一次'
    w._on_sitl_telemetry(_att_frame(1, t0 + 0.093, t0 + 0.08, roll=4, yaw=10.0))
    w._flush_telemetry()
    assert 'atts' not in m.batches[2][0] and m.batches[2][0]['at'] == int(round((t0 + 0.08) * 1000)), \
        '只有一則新姿態 ⇒ 它就是主樣本'


def test_attitude_sample_time_prefers_the_fc_clock_and_falls_back_to_the_receipt_time():
    t = 1_700_000_000.5
    assert MainWindow._uav_batch_item(_att_frame(1, t, t - 0.017))['at'] == int(round((t - 0.017) * 1000))
    assert MainWindow._uav_batch_item(_att_frame(1, t, 0.0))['at'] == int(round(t * 1000)), '沒有 att_ts ⇒ 收到時刻'
    assert MainWindow._uav_batch_item(_att_frame(1, t, float('nan')))['at'] == int(round(t * 1000))
    assert 'at' not in MainWindow._uav_batch_item(_att_frame(1, 0.0, 0.0)), '沒收過姿態 ⇒ 不帶'


def test_attitude_samples_never_pile_up_invalid_gps_old_map_cap_and_forget(timer, monkeypatch):
    m = _BatchMap()
    w = _win(m, sysids=(1, 2))
    t0 = 1_700_000_000.0
    for k in range(5):
        w._on_sitl_telemetry(_att_frame(1, t0 + 0.02 * k, gps=0))              # GPS 無效：進 HUD、不進地圖
    timer[0][1]()
    assert m.batches == [] and w._tele.get('atts', {}).get(1) is None, 'GPS 無效那架的樣本 flush 時一起丟掉'
    monkeypatch.setattr(mw, 'ATT_SAMPLES_MAX', 8)
    for k in range(30):                                                        # 主執行緒卡住：一直沒 flush
        w._on_sitl_telemetry(_att_frame(2, t0 + 1 + 0.02 * k, roll=k))
    q = w._tele['atts'][2]
    assert len(q) == 8 and q[-1][1] == 29.0 and q[0][1] == 22.0, '只留最新的 ATT_SAMPLES_MAX 筆'
    mw._forget_tele_state(w, 2)
    assert 2 not in w._tele['atts'] and 2 not in w._tele['att_seen']
    w._on_sitl_telemetry(_att_frame(2, t0 + 2.0)); w._on_sitl_telemetry(_att_frame(2, t0 + 2.02))
    mw._forget_tele_state(w, None)
    assert w._tele['atts'] == {} and w._tele['att_seen'] == {}
    old = _OldMap()                                                            # 沒有批次 API 的地圖：不帶 atts，也不累積
    w2 = _win(old, sysids=(1,))
    for k in range(3):
        w2._on_sitl_telemetry(_att_frame(1, t0 + 0.02 * k))
    w2._flush_telemetry()
    assert len(old.calls) == 1 and not w2._tele.get('atts', {}).get(1)
