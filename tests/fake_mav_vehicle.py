# -*- coding: utf-8 -*-
"""tests/fake_mav_vehicle.py — 假的 ArduPlane QuadPlane（pymavlink、TCP 伺服端），給 tools/attitude_fidelity_probe.py 驗證用。

**不是飛控模型**：姿態是寫死的劇本（或 --fly 時的簡化運動學），用來證明「量測工具本身」量得準、也抓得到錯。
跟真的 SITL 一樣當 TCP 伺服端（一個埠一個客戶端；可以開好幾個埠 ⇒ 主鏈路＋獨立的真值鏈路），每個客戶端各自的串流頻率。

支援（只做 SITLLink 與探針會用到的）：
  * HEARTBEAT 1 Hz：MAV_TYPE_FIXED_WING＋ARDUPILOTMEGA（QuadPlane 韌體的回報方式），custom_mode 用 ArduPlane 模式表
  * REQUEST_DATA_STREAM（0=全部、2=EXTENDED_STATUS、6=POSITION、10=EXTRA1、11=EXTRA2）與
    SET_MESSAGE_INTERVAL（只認 ATTITUDE）；沒要求前的預設頻率是 2／2／4／4 Hz
  * ATTITUDE（EXTRA1）、GLOBAL_POSITION_INT（POSITION）、VFR_HUD（EXTRA2）、SYS_STATUS＋GPS_RAW_INT（EXTENDED_STATUS）
    SYS_STATUS 的 MAV_SYS_STATUS_PREARM_CHECK：present／enabled 一直有，health 在開機 ready_after_s 秒後才有
  * PARAM_REQUEST_READ（Q_ENABLE=1；其他名字不回，同真飛控「查無此參數」）、PARAM_SET
  * 任務寫入（MISSION_COUNT → REQUEST_INT × n → ACK）與讀回（REQUEST_LIST → COUNT → ITEM_INT）
  * COMMAND_LONG：ARM_DISARM（未就緒或 refuse_normal_arm 時正常解鎖被拒＋PreArm STATUSTEXT；21196 = force 一定過）、DO_SET_MODE、
    MISSION_START、SET_MESSAGE_INTERVAL；SET_MODE 訊息

劇本（script='attitude'，預設）：12.5 s 一個循環、不停重播（時間 0 = 物件建立）：
  偏航 340°→20°（40°/s，跨過 359→1）、滾轉 0→+45→−45→0（60°/s）、俯仰 0→+20→−20→0（40°/s）、
  最後一段左轉：滾轉 −30、俯仰 5、偏航 20°→340°（−20°/s，再跨一次北）。
  水平速度 speed_ms 沿「航跡 = 偏航 + crab_deg」（crab ≠ 0 模擬側風偏流：機頭與航跡差 crab_deg），高度固定。
  模式 mode_name（預設 AUTO）、已解鎖。
  ⚠ 分段線性 ⇒ 兩筆樣本之間線性內插幾乎沒有誤差（只有轉角）：證明不了插值。要驗插值用 script='sine'。

script='sine'（2026-10-22 審查：分段線性、相位對齊、本機沒有抖動的串流讓時間與插值的錯誤全部看不到）：
  滾轉 45·sin(2π·0.5 t)、俯仰 20·sin(2π·0.5 t + 1)、偏航 350 + 40·sin(2π·0.2 t)（跨北）——一直在彎 ⇒ 20 Hz 線性內插
  的弦誤差 ≈ 0.14°、4 Hz ≈ 3.5°；不插值（畫前一筆）≈ 角速度 141°/s × 樣本間隔。
其他讓串流變難的選項：
  att_phase_s      ATTITUDE 比 GLOBAL_POSITION_INT 晚這麼多秒送（真飛控兩條串流不同相）
  burst_every_s／burst_hold_s／burst_start_s／burst_ports
                   每 burst_every_s 秒把那幾個埠（索引）的輸出扣住 burst_hold_s 秒再一口氣送出（成批到：收到時刻不再是
                   取樣時刻；time_boot_ms 照取樣當下蓋）
  hb_period_s      心跳週期。vehicle='quadplane' 而 SITLLink 沒有 vehicle_hint 時，第一筆位置在 Q_ENABLE 讀回後、下一個心跳
                   之前是 'PLANE'，之後才變 'VTOL' ⇒ 3D 頁面在飛行中換模型（拉長心跳週期 ⇒ PLANE 那一段更長）
  vehicle          'quadplane'（FIXED_WING 心跳、Q_ENABLE=1）／'plane'（Q_ENABLE=0）／'copter'（QUADROTOR 心跳、ArduCopter
                   模式表、沒有 Q_ENABLE）
  ⚠ 「心跳之前先送位置」走不到 3D：SITLLink 連線時先 wait_heartbeat（其他訊息丟掉）才進主迴圈，第一個來源的 frame
    用那個心跳的機型預先建立 ⇒ 這條路上第一筆一定有機型；'---' → 機型那條只在別的來源（同鏈路多機）出現。

--fly（script='fly'）：地面、未解鎖、MANUAL；解鎖＋AUTO 後照上傳的任務飛：VTOL_TAKEOFF(84)／TAKEOFF(22) 垂直爬升 →
  WAYPOINT(16) 等速協調轉彎（偏航率 ≤ 20°/s，滾轉 = atan(V·r/g)）→ VTOL_LAND(85)／LAND(21) 飛到點上垂直下降 → 觸地自動上鎖。

用法（手動試探針，不需要 SITL）：
    python tests/fake_mav_vehicle.py --port 6760 --truth-port 6762 --crab 10
    python tools/attitude_fidelity_probe.py --conn tcp:127.0.0.1:6760 --truth tcp:127.0.0.1:6762 --seconds 15
"""
from __future__ import annotations

import argparse
import math
import os
import select
import socket
import threading
import time

os.environ.setdefault('MAVLINK20', '1')       # ArduPilot 走 MAVLink 2；mission_type 等擴充欄位要 v2

from pymavlink import mavutil  # noqa: E402

mav = mavutil.mavlink

PLANE_MODES = {'MANUAL': 0, 'CIRCLE': 1, 'STABILIZE': 2, 'FBWA': 5, 'FBWB': 6, 'CRUISE': 7, 'AUTO': 10, 'RTL': 11,
               'LOITER': 12, 'TAKEOFF': 13, 'GUIDED': 15, 'QSTABILIZE': 17, 'QHOVER': 18, 'QLOITER': 19, 'QLAND': 20,
               'QRTL': 21}
COPTER_MODES = {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'LAND': 9}
G = 9.80665
EARTH_R = 6378137.0

# 劇本關鍵格：(t 秒, 滾轉, 俯仰, 偏航（展開值，可超過 360，內插後才取模）) —— 分段線性
KEYFRAMES = (
    (0.0, 0.0, 0.0, 340.0),
    (1.0, 0.0, 0.0, 340.0),
    (2.0, 0.0, 0.0, 380.0),      # 偏航 340 → 20（40°/s，跨過 359→1）
    (2.5, 0.0, 0.0, 380.0),
    (3.25, 45.0, 0.0, 380.0),    # 右滾 +45（60°/s）
    (3.75, 45.0, 0.0, 380.0),
    (5.25, -45.0, 0.0, 380.0),   # 一口氣滾到 −45（60°/s）
    (5.75, -45.0, 0.0, 380.0),
    (6.5, 0.0, 0.0, 380.0),
    (7.0, 0.0, 20.0, 380.0),     # 抬頭 +20（40°/s）
    (7.5, 0.0, 20.0, 380.0),
    (8.5, 0.0, -20.0, 380.0),    # 低頭到 −20
    (9.0, 0.0, -20.0, 380.0),
    (9.5, 0.0, 0.0, 380.0),
    (10.0, -30.0, 5.0, 380.0),   # 左坡度 30、俯仰 5
    (12.0, -30.0, 5.0, 340.0),   # 左轉 20 → 340（−20°/s，再跨一次北）
    (12.5, 0.0, 0.0, 340.0),
)
PERIOD_S = KEYFRAMES[-1][0]


def scripted_attitude(t: float) -> tuple[float, float, float]:
    """劇本在時間 t（秒）的 (滾轉, 俯仰, 偏航 0–360)；超過一個週期就重播。"""
    t = t % PERIOD_S
    for (t0, r0, p0, y0), (t1, r1, p1, y1) in zip(KEYFRAMES, KEYFRAMES[1:]):
        if t0 <= t <= t1:
            u = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return r0 + u * (r1 - r0), p0 + u * (p1 - p0), (y0 + u * (y1 - y0)) % 360.0
    r, p, y = KEYFRAMES[-1][1:]
    return r, p, y % 360.0


def sine_attitude(t: float) -> tuple[float, float, float]:
    """script='sine' 在時間 t 的 (滾轉, 俯仰, 偏航 0–360)：一直在彎（二階導數不為零），偏航 310～30 跨北。"""
    w = 2.0 * math.pi * 0.5
    return (45.0 * math.sin(w * t), 20.0 * math.sin(w * t + 1.0),
            (350.0 + 40.0 * math.sin(2.0 * math.pi * 0.2 * t)) % 360.0)


def _offset(lat, lon, dn, de):
    lat2 = lat + math.degrees(dn / EARTH_R)
    lon2 = lon + math.degrees(de / (EARTH_R * math.cos(math.radians(lat))))
    return lat2, lon2


def _ne(lat0, lon0, lat, lon):
    return (math.radians(lat - lat0) * EARTH_R,
            math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)))


def _wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


class _Client:
    """一條 TCP 連線：自己的 MAVLink 編碼器（序號）、解析器與串流頻率。"""
    STREAMS = (2, 6, 10, 11)

    def __init__(self, sock: socket.socket, sysid: int, idx: int):
        self.sock = sock
        self.idx = idx
        self.alive = True
        self.mav = mav.MAVLink(self, srcSystem=sysid, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
        self.mav.robust_parsing = True
        self.rates = {2: 2.0, 6: 2.0, 10: 4.0, 11: 4.0}      # 沒要求前的預設
        self.att_interval_s = None                            # SET_MESSAGE_INTERVAL(ATTITUDE)；None = 跟 EXTRA1
        self.next_due = {s: 0.0 for s in self.STREAMS}
        self.next_hb = 0.0
        self.t_connect = time.time()
        self.hold_until = 0.0                                 # 成批：這個時刻之前的輸出先扣住
        self.next_burst = None
        self.held = bytearray()
        self.n_bursts = 0

    def write(self, buf):                                     # pymavlink 編碼器呼叫
        if not self.alive:
            return
        if time.time() < self.hold_until:
            self.held += buf
            return
        self.flush(buf)

    def flush(self, buf=b''):
        """扣住的輸出（成批）連同這一包一口氣送出。"""
        if not self.alive:
            return
        data = bytes(self.held) + bytes(buf)
        self.held = bytearray()
        if not data:
            return
        try:
            self.sock.sendall(data)
        except OSError:
            self.alive = False

    def period(self, stream: int):
        if stream == 10 and self.att_interval_s is not None:
            return self.att_interval_s if self.att_interval_s > 0 else None
        hz = self.rates.get(stream, 0.0)
        return (1.0 / hz) if hz > 0 else None


class FakeVehicle:
    """假的 QuadPlane。``ports`` 給 0 ⇒ 系統挑空埠（看 ``self.ports``）。``start()`` 開背景執行緒、``stop()`` 收掉。"""

    def __init__(self, ports=(0,), sysid: int = 1, home=(23.4518, 120.2863, 30.0), script: str = 'attitude',
                 crab_deg: float = 0.0, speed_ms: float = 18.0, alt_rel_m: float = 100.0, mode_name: str = 'AUTO',
                 ready_after_s: float = 1.0, climb_ms: float = 4.0, sink_ms: float = 3.0, turn_rate_dps: float = 20.0,
                 refuse_normal_arm: bool = False, verbose: bool = False, vehicle: str = 'quadplane',
                 att_phase_s: float = 0.0, burst_every_s: float = 0.0, burst_hold_s: float = 0.0,
                 burst_start_s: float = 2.0, burst_ports=(0,), hb_period_s: float = 1.0):
        self.sysid = int(sysid)
        self.home = tuple(float(v) for v in home)            # lat, lon, alt_msl
        self.script = script
        self.vehicle = str(vehicle).lower()
        if self.vehicle not in ('quadplane', 'plane', 'copter'):
            raise ValueError(f'vehicle 要是 quadplane／plane／copter：{vehicle}')
        if script == 'fly' and self.vehicle == 'copter':
            raise ValueError('script=fly 只做固定翼／QuadPlane')
        self.modes = COPTER_MODES if self.vehicle == 'copter' else PLANE_MODES
        self.crab_deg = float(crab_deg)
        self.speed_ms = float(speed_ms)
        self.ready_after_s = float(ready_after_s)
        self.refuse_normal_arm = bool(refuse_normal_arm)    # PreArm 位元說好、但正常解鎖照樣拒絕（逼 SITLLink 走 FORCE）
        self.climb_ms, self.sink_ms, self.turn_rate_dps = float(climb_ms), float(sink_ms), float(turn_rate_dps)
        self.att_phase_s = float(att_phase_s)
        self.burst_every_s, self.burst_hold_s = float(burst_every_s), float(burst_hold_s)
        self.burst_start_s, self.burst_ports = float(burst_start_s), tuple(int(i) for i in burst_ports)
        self.hb_period_s = float(hb_period_s)
        self.verbose = verbose
        self.t0 = time.time()
        # Q_ENABLE：QuadPlane 1、純固定翼 0；多旋翼沒有這個參數（查無此參數 ⇒ 不回）
        self.params = {'quadplane': {'Q_ENABLE': 1.0}, 'plane': {'Q_ENABLE': 0.0}, 'copter': {}}[self.vehicle]
        self.mission: list = []
        self._pending = None                                   # [count, next_seq, items]
        self._servers = []
        for p in ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('127.0.0.1', int(p)))
            s.listen(1)
            s.setblocking(False)
            self._servers.append(s)
        self.ports = [s.getsockname()[1] for s in self._servers]
        self._clients: list[_Client | None] = [None] * len(self._servers)
        self._stop = threading.Event()
        self._thread = None
        self.att_log: list = []        # (port 序號, 送出時刻 epoch s, time_boot_ms, roll, pitch, yaw) —— 真的送出去的
        self.pos_log: list = []        # (port 序號, 送出時刻 epoch s, time_boot_ms) —— GLOBAL_POSITION_INT
        self.events: list = []         # (t, 文字)：解鎖、模式、任務…
        # 狀態
        if script == 'fly':
            self.armed, self.mode = False, PLANE_MODES['MANUAL']
            self.lat, self.lon, self.alt_rel = self.home[0], self.home[1], 0.0
            self.roll = self.pitch = 0.0
            self.yaw = 0.0
            self.gs, self.track, self.vz = 0.0, 0.0, 0.0
            self.phase, self.wp_idx = 'ground', 0
        else:
            self.armed, self.mode = True, self.modes.get(mode_name.upper(), self.modes['AUTO'])
            self.lat, self.lon, self.alt_rel = self.home[0], self.home[1], float(alt_rel_m)
            self.roll, self.pitch, self.yaw = self._script_attitude(0.0)
            self.gs, self.track, self.vz = self.speed_ms, (self.yaw + self.crab_deg) % 360.0, 0.0
            self.phase = 'script'
        self._last_step = time.time()

    # ── 公開 ──────────────────────────────────────────────────────────
    def start(self):
        self._thread = threading.Thread(target=self._run, name='fake-mav-vehicle', daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        for c in self._clients:
            if c is not None:
                try:
                    c.sock.close()
                except OSError:
                    pass
        for s in self._servers:
            try:
                s.close()
            except OSError:
                pass

    def log(self, text):
        self.events.append((time.time(), text))
        if self.verbose:
            print('[fake]', text, flush=True)

    def _script_attitude(self, t):
        return sine_attitude(t) if self.script == 'sine' else scripted_attitude(t)

    def bursts(self) -> int:
        """各埠到目前為止扣住再一口氣送出的次數總和。"""
        return sum(c.n_bursts for c in self._clients if c is not None)

    # ── 主迴圈 ────────────────────────────────────────────────────────
    def _run(self):
        while not self._stop.is_set():
            self._accept()
            self._read()
            self._step(time.time())
            self._send_due(time.time())
            time.sleep(0.002)

    def _accept(self):
        for i, s in enumerate(self._servers):
            c = self._clients[i]
            if c is not None and c.alive:
                continue
            try:
                sock, _addr = s.accept()
            except (BlockingIOError, OSError):
                continue
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(1.0)
            self._clients[i] = _Client(sock, self.sysid, i)
            self.log(f'客戶端連上埠 {self.ports[i]}')

    def _read(self):
        socks = [c.sock for c in self._clients if c is not None and c.alive]
        if not socks:
            return
        try:
            rd, _, _ = select.select(socks, [], [], 0)
        except (OSError, ValueError):
            return
        for c in self._clients:
            if c is None or not c.alive or c.sock not in rd:
                continue
            try:
                data = c.sock.recv(65536)
            except OSError:
                data = b''
            if not data:
                c.alive = False
                self.log(f'埠 {self.ports[c.idx]} 的客戶端斷線')
                continue
            try:
                msgs = c.mav.parse_buffer(data) or []
            except Exception:                                   # noqa: BLE001 — 壞封包不讓假飛控掛掉
                msgs = []
            for m in msgs:
                try:
                    self._handle(c, m)
                except Exception as e:                          # noqa: BLE001
                    self.log(f'處理 {m.get_type()} 失敗：{e}')

    # ── 運動 ──────────────────────────────────────────────────────────
    def _step(self, now: float):
        dt = max(0.0, min(0.1, now - self._last_step))
        self._last_step = now
        if self.script != 'fly':
            self.roll, self.pitch, self.yaw = self._script_attitude(now - self.t0)
            self.track = (self.yaw + self.crab_deg) % 360.0
            self.gs = self.speed_ms
            self._move(dt)
            return
        self._step_fly(dt)

    def _move(self, dt):
        dn = self.gs * math.cos(math.radians(self.track)) * dt
        de = self.gs * math.sin(math.radians(self.track)) * dt
        self.lat, self.lon = _offset(self.lat, self.lon, dn, de)
        self.alt_rel += self.vz * dt

    def _nav_items(self):
        return [it for it in self.mission[1:] if it['command'] in (16, 21, 22, 84, 85)]

    def _step_fly(self, dt):
        if not self.armed or self.mode != PLANE_MODES['AUTO']:
            if self.phase != 'ground' and not self.armed:
                self.phase = 'ground'
            self.gs = 0.0
            self.vz = 0.0
            return
        nav = self._nav_items()
        if self.phase == 'ground':
            self.phase, self.wp_idx = 'nav', 0
            self.log('AUTO：開始任務')
        if self.wp_idx >= len(nav):
            self.vz, self.gs = 0.0, 0.0
            return
        it = nav[self.wp_idx]
        cmd = it['command']
        if cmd in (22, 84):                                     # 垂直爬升到 alt
            self.gs, self.vz, self.roll, self.pitch = 0.0, self.climb_ms, 0.0, 0.0
            self._move(dt)
            if self.alt_rel >= it['alt']:
                self.alt_rel = it['alt']
                self.vz = 0.0
                self.wp_idx += 1
                self.log(f'起飛完成 {self.alt_rel:.0f} m')
            return
        dn, de = _ne(self.lat, self.lon, it['lat'], it['lon'])
        dist = math.hypot(dn, de)
        if cmd in (21, 85) and (dist < 5.0 or self.phase == 'descend'):   # 垂直下降、觸地上鎖
            self.phase = 'descend'
            self.gs, self.vz, self.roll, self.pitch = 0.0, -self.sink_ms, 0.0, 0.0
            self._move(dt)
            if self.alt_rel <= 0.0:
                self.alt_rel, self.vz = 0.0, 0.0
                self.armed = False
                self.wp_idx = len(nav)
                self.log('觸地、自動上鎖')
            return
        # 平飛到點：偏航率限制的協調轉彎
        want = math.degrees(math.atan2(de, dn)) % 360.0
        err = _wrap180(want - self.track)
        rate = max(-self.turn_rate_dps, min(self.turn_rate_dps, 1.5 * err))     # 比例轉彎、限偏航率
        self.track = (self.track + rate * dt) % 360.0
        self.yaw = (self.track - self.crab_deg) % 360.0
        self.gs = self.speed_ms
        self.roll = max(-45.0, min(45.0, math.degrees(math.atan(self.gs * math.radians(rate) / G))))
        self.pitch = 2.0
        self.vz = max(-2.0, min(2.0, (it['alt'] - self.alt_rel) * 0.5))
        self._move(dt)
        if dist < max(8.0, self.gs * 0.6) and cmd == 16:
            self.wp_idx += 1
            self.log(f'到達航點 {self.wp_idx}')

    # ── 送出 ──────────────────────────────────────────────────────────
    def _boot_ms(self, now):
        return int((now - self.t0) * 1000) & 0xFFFFFFFF

    def _send_due(self, now):
        for c in self._clients:
            if c is None or not c.alive:
                continue
            if self.burst_every_s > 0 and self.burst_hold_s > 0 and c.idx in self.burst_ports:
                if c.next_burst is None:
                    c.next_burst = c.t_connect + self.burst_start_s
                if now >= c.next_burst:                     # 開始扣住
                    c.hold_until, c.next_burst = now + self.burst_hold_s, now + self.burst_every_s
                    c.n_bursts += 1
                elif c.held and now >= c.hold_until:        # 放開：一口氣送出
                    c.flush()
            if now >= c.next_hb:
                self._send_heartbeat(c)
                c.next_hb = now + self.hb_period_s
            for s in c.STREAMS:
                per = c.period(s)
                if per is None or now < c.next_due[s]:
                    continue
                # 下一次 = 上一次 + 週期（不累積抖動），落後太多就從現在重來
                c.next_due[s] = c.next_due[s] + per if now - c.next_due[s] < per else now + per
                self._send_stream(c, s, now)

    def _send_heartbeat(self, c):
        base = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
        typ = mav.MAV_TYPE_QUADROTOR if self.vehicle == 'copter' else mav.MAV_TYPE_FIXED_WING
        c.mav.heartbeat_send(typ, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, base, self.mode,
                             mav.MAV_STATE_ACTIVE if self.armed else mav.MAV_STATE_STANDBY)

    def _send_stream(self, c, s, now):
        if s == 10:
            boot = self._boot_ms(now)
            # 劇本：姿態取在 time_boot_ms 那一毫秒上（不是上一次 _step 的時刻、也不被毫秒截斷差掉 ≤ 1 ms）
            # ⇒ (time_boot_ms, 姿態) 一對一精確；快速轉動時兩條鏈路在開機時間軸上的比較才只剩內插誤差
            r, p, y = (self._script_attitude(boot / 1000.0) if self.script != 'fly'
                       else (self.roll, self.pitch, self.yaw))
            yr = math.radians(_wrap180(y))
            c.mav.attitude_send(boot, math.radians(r), math.radians(p), yr, 0.0, 0.0, 0.0)
            self.att_log.append((c.idx, now, boot, r, p, _wrap180(y) % 360.0))
        elif s == 6:
            self.pos_log.append((c.idx, now, self._boot_ms(now)))
            vn = self.gs * math.cos(math.radians(self.track))
            ve = self.gs * math.sin(math.radians(self.track))
            c.mav.global_position_int_send(self._boot_ms(now), int(round(self.lat * 1e7)), int(round(self.lon * 1e7)),
                                           int(round((self.home[2] + self.alt_rel) * 1000)),
                                           int(round(self.alt_rel * 1000)), int(round(vn * 100)), int(round(ve * 100)),
                                           int(round(-self.vz * 100)), int(round((self.yaw % 360.0) * 100)) % 36000)
        elif s == 11:
            c.mav.vfr_hud_send(self.gs, self.gs, int(round(self.yaw)) % 360, 40 if self.armed else 0,
                               self.home[2] + self.alt_rel, self.vz)
        elif s == 2:
            pre = mav.MAV_SYS_STATUS_PREARM_CHECK
            ready = (now - self.t0) >= self.ready_after_s
            c.mav.sys_status_send(pre, pre, pre if ready else 0, 300, 16000, 500, 90, 0, 0, 0, 0, 0, 0)
            c.mav.gps_raw_int_send(int(now * 1e6), 3, int(round(self.lat * 1e7)), int(round(self.lon * 1e7)),
                                   int(round((self.home[2] + self.alt_rel) * 1000)), 80, 120,
                                   int(round(self.gs * 100)), int(round((self.track % 360.0) * 100)) % 36000, 12)

    def _statustext(self, c, sev, text):
        c.mav.statustext_send(sev, text.encode('utf-8')[:50])

    # ── 收到 ──────────────────────────────────────────────────────────
    def _handle(self, c, m):
        t = m.get_type()
        if t in ('BAD_DATA', 'HEARTBEAT'):
            return
        src, comp = m.get_srcSystem(), m.get_srcComponent()
        if t == 'REQUEST_DATA_STREAM':
            rate = float(m.req_message_rate) if m.start_stop else 0.0
            ids = c.STREAMS if m.req_stream_id == 0 else ((m.req_stream_id,) if m.req_stream_id in c.STREAMS else ())
            for s in ids:
                c.rates[s] = rate
                # ATTITUDE（EXTRA1）可以和其他串流錯開 att_phase_s（真飛控兩條串流不同相）
                c.next_due[s] = (time.time() + self.att_phase_s) if (s == 10 and self.att_phase_s) else 0.0
            return
        if t == 'PARAM_REQUEST_READ':
            name = m.param_id if isinstance(m.param_id, str) else m.param_id.decode('utf-8', 'ignore')
            name = name.split('\x00', 1)[0]
            if name in self.params:
                c.mav.param_value_send(name.encode(), self.params[name], mav.MAV_PARAM_TYPE_REAL32,
                                       len(self.params), list(self.params).index(name))
            return
        if t == 'PARAM_SET':
            name = m.param_id if isinstance(m.param_id, str) else m.param_id.decode('utf-8', 'ignore')
            name = name.split('\x00', 1)[0]
            self.params[name] = float(m.param_value)
            c.mav.param_value_send(name.encode(), self.params[name], mav.MAV_PARAM_TYPE_REAL32,
                                   len(self.params), list(self.params).index(name))
            return
        if t == 'MISSION_COUNT':
            if getattr(m, 'mission_type', 0) != 0:
                c.mav.mission_ack_send(src, comp, mav.MAV_MISSION_UNSUPPORTED, m.mission_type)
                return
            self._pending = [int(m.count), 0, []]
            if m.count == 0:
                self.mission, self._pending = [], None
                c.mav.mission_ack_send(src, comp, mav.MAV_MISSION_ACCEPTED, 0)
            else:
                c.mav.mission_request_int_send(src, comp, 0, 0)
            return
        if t in ('MISSION_ITEM_INT', 'MISSION_ITEM'):
            if self._pending is None:
                c.mav.mission_ack_send(src, comp, mav.MAV_MISSION_ERROR, 0)
                return
            count, nxt, items = self._pending
            if m.seq != nxt:
                c.mav.mission_request_int_send(src, comp, nxt, 0)
                return
            k = 1e7 if t == 'MISSION_ITEM_INT' else 1.0
            items.append({'seq': m.seq, 'frame': m.frame, 'command': m.command, 'autocontinue': m.autocontinue,
                          'param1': m.param1, 'param2': m.param2, 'param3': m.param3, 'param4': m.param4,
                          'lat': m.x / k, 'lon': m.y / k, 'alt': m.z})
            nxt += 1
            self._pending[1] = nxt
            if nxt < count:
                c.mav.mission_request_int_send(src, comp, nxt, 0)
            else:
                self.mission, self._pending = items, None
                c.mav.mission_ack_send(src, comp, mav.MAV_MISSION_ACCEPTED, 0)
                self.log(f'任務寫入 {count} 項：' + ','.join(str(i['command']) for i in items))
            return
        if t == 'MISSION_REQUEST_LIST':
            c.mav.mission_count_send(src, comp, len(self.mission), 0)
            return
        if t in ('MISSION_REQUEST_INT', 'MISSION_REQUEST'):
            if 0 <= m.seq < len(self.mission):
                it = self.mission[m.seq]
                c.mav.mission_item_int_send(src, comp, it['seq'], it['frame'], it['command'], 0, it['autocontinue'],
                                            it['param1'], it['param2'], it['param3'], it['param4'],
                                            int(round(it['lat'] * 1e7)), int(round(it['lon'] * 1e7)), it['alt'], 0)
            return
        if t == 'SET_MODE':
            self._set_mode(c, int(m.custom_mode))
            return
        if t == 'COMMAND_LONG':
            res = mav.MAV_RESULT_ACCEPTED
            if m.command == mav.MAV_CMD_COMPONENT_ARM_DISARM:
                if m.param1 >= 0.5:
                    forced = int(round(m.param2)) == 21196
                    ready = (time.time() - self.t0) >= self.ready_after_s
                    if forced or (ready and not self.refuse_normal_arm):
                        self.armed = True
                        self.log('解鎖' + ('（FORCE）' if forced else ''))
                    else:
                        res = mav.MAV_RESULT_FAILED
                        self._statustext(c, mav.MAV_SEVERITY_CRITICAL, 'PreArm: fake vehicle not ready')
                else:
                    self.armed = False
                    self.log('上鎖')
                self._send_heartbeat(c)
            elif m.command == mav.MAV_CMD_DO_SET_MODE:
                self._set_mode(c, int(round(m.param2)))
            elif m.command == mav.MAV_CMD_MISSION_START:
                self._set_mode(c, PLANE_MODES['AUTO'])
            elif m.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL:
                if int(round(m.param1)) == mav.MAVLINK_MSG_ID_ATTITUDE:
                    us = float(m.param2)
                    c.att_interval_s = None if us == 0 else (-1.0 if us < 0 else us / 1e6)
                    c.next_due[10] = (time.time() + self.att_phase_s) if self.att_phase_s else 0.0
            else:
                res = mav.MAV_RESULT_UNSUPPORTED
            c.mav.command_ack_send(m.command, res)

    def _set_mode(self, c, mode: int):
        if mode != self.mode:
            self.mode = mode
            name = next((k for k, v in self.modes.items() if v == mode), str(mode))
            self.log(f'模式 → {name}')
        self._send_heartbeat(c)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='假的 ArduPlane QuadPlane（給姿態探針驗證用）')
    ap.add_argument('--port', type=int, default=6760, help='主鏈路 TCP 埠（探針的 --conn）')
    ap.add_argument('--truth-port', type=int, default=0, help='另一個 TCP 埠（探針的 --truth）；0 = 不開')
    ap.add_argument('--sysid', type=int, default=1)
    ap.add_argument('--crab', type=float, default=0.0, help='航跡 − 機頭（度）：模擬側風偏流')
    ap.add_argument('--speed', type=float, default=18.0)
    ap.add_argument('--fly', action='store_true', help='地面待命，照上傳的任務飛（給探針的 --fly 用）')
    ap.add_argument('--script', default='attitude', choices=['attitude', 'sine'],
                    help='姿態劇本：attitude = 分段線性；sine = 一直在彎（驗插值）')
    ap.add_argument('--vehicle', default='quadplane', choices=['quadplane', 'plane', 'copter'])
    ap.add_argument('--att-phase', type=float, default=0.0, help='ATTITUDE 比 GLOBAL_POSITION_INT 晚幾秒送')
    ap.add_argument('--burst-every', type=float, default=0.0, help='每幾秒把主鏈路的輸出扣住一次（成批到）')
    ap.add_argument('--burst-hold', type=float, default=0.0, help='每次扣住幾秒')
    ap.add_argument('--hb-period', type=float, default=1.0, help='心跳週期（秒）')
    ap.add_argument('--ready-after', type=float, default=1.0, help='開機幾秒後 PreArm 才過')
    ap.add_argument('--refuse-arm', action='store_true', help='正常解鎖一律拒絕（只有 FORCE 會過）')
    ap.add_argument('--seconds', type=float, default=0.0, help='跑多久（0 = 直到 Ctrl+C）')
    a = ap.parse_args(argv)
    ports = [a.port] + ([a.truth_port] if a.truth_port else [])
    v = FakeVehicle(ports=ports, sysid=a.sysid, script='fly' if a.fly else a.script, crab_deg=a.crab,
                    speed_ms=a.speed, ready_after_s=a.ready_after, refuse_normal_arm=a.refuse_arm, verbose=True,
                    vehicle=a.vehicle, att_phase_s=a.att_phase, burst_every_s=a.burst_every,
                    burst_hold_s=a.burst_hold, hb_period_s=a.hb_period).start()
    print(f'假飛控在 {v.ports}（sysid {a.sysid}，{a.vehicle}，{"任務飛行" if a.fly else "姿態劇本 " + a.script}）',
          flush=True)
    t_end = time.time() + a.seconds if a.seconds > 0 else None
    try:
        while t_end is None or time.time() < t_end:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    v.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
