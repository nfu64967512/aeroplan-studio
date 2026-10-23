# -*- coding: utf-8 -*-
"""attitude_fidelity_probe.py — 3D 模型姿態忠實度量測：SITL 的姿態，3D 模型到底畫成什麼樣子。

使用者 2026-10-22：「模型要忠實反映 SITL 的姿態」。這支走**真的那一條路**，量「畫出來的」對「飛控送出來的」：

    MAVLink（SITL）
      → mission.sitl_link.SITLLink（QThread，真的；ATTITUDE → roll/pitch/yaw 度＋att_t）
      → MainWindow._on_sitl_telemetry／_flush_telemetry（真的方法，綁在一個替身物件上：30 Hz 合幀、只送最新一幀）
      → MainWindow._uav_batch_item（真的 staticmethod：hdg＝ATTITUDE yaw、crs＝atan2(ve, vn)、pt／at）
      → ui.widgets.cesium_map_widget.CesiumMapWidget（**真的 widget 實例**、真的 main.html、真的 GLB，真的 QtWebEngine 視窗，
        預設 WA_DontShowOnScreen：不出現在螢幕上但照常算圖）.update_uav_positions
      → main.html：_uavNoseHeading（機鼻混航跡）→ _uavModelOrientation → SampledProperty 插值 → entity.orientation

真值（時間軸兩種 × 來源兩份）：
  * link：同一個 SITLLink 的 telemetry 信號（DirectConnection，在鏈路執行緒當下記）—— (att_t, roll, pitch, yaw, att_ts)，
    外加那一則 ATTITUDE 的 time_boot_ms（pymavlink 的最後一則訊息快取；值對得上才記）
  * raw（--truth CONN，可省；要閘門就必須有）：另一條 pymavlink 連線直接收 ATTITUDE（不經 SITLLink）；預設把它的取樣
    相位對齊主鏈路（--truth-align-phase，理由見 RawTruth）
  * 頁面樣本時間軸（2026-10-22 起 = SITLLink 的 att_ts：飛控時間經 mission/boot_clock.py 換到牆鐘；舊資料是收到時刻
    att_t）：就是頁面拿來當樣本時間的那個數 ⇒ 只驗畫法＋插值＋合幀，看不到時間戳本身的錯
  * 開機時間軸（boot clock，2026-10-22 審查後加）：飛控的 time_boot_ms 用「下包絡」線性擬合換成牆鐘（最快那幾則的收到
    時刻；斜率吃得下 --speedup）⇒ 真的「飛控在那一刻的姿態」。成批到、排隊、收到時刻當樣本時間的誤差全部看得到；
    另外報收到時刻比下包絡晚多少（抖動）、被壓扁的間隔／成批次數

讀回兩種（2026-10-22 審查：以前固定 50 ms 讀一次 = 20 Hz 樣本的週期 ⇒ 每一趟只量到樣本間隔裡的一個相位）：
  * 每一個算圖幀（頁面 clock.onTick 的環形緩衝，每次讀回時整批取走）：顯示時間、entity.orientation、模型 primitive 的
    modelMatrix ⇒ errors_per_frame_deg、errors_drawn_frame_deg、errors_vs_boot_clock_deg
  * 讀回當下（週期 --period-ms × (1 ± --period-jitter) 隨機，不和樣本鎖相）：另外帶頁面狀態（yaw／nose／crs…）
    ⇒ errors_deg（舊的主指標）、errors_vs_wallclock_now_deg（「現在」= JS 執行當下的 Date.now()，不是 Python 收到回呼的時刻）
兩者都乘上該機型 GLB 校正的反矩陣 ⇒ ENU 機體軸（機鼻 +Y、右翼 +X、機背 +Z）⇒ 航空 3-2-1 偏航／俯仰／滾轉（偏航由北順時針），
真值在**同一個顯示時間**內插。校正用**探針自己寫死的那組**（PINNED_OFFSETS：使用者目視確認、GLB 幾何稽核證實），不用頁面
常數 —— 從頁面讀就是拿錯的值去驗錯的值；用哪一組看 entity 實際載入的模型 URI（不看 vt）。頁面常數和寫死的不同 ⇒
summary.json 的 calibration.mismatch 列出來、閘門不過。
「drawn」= entity／模型 primitive 的 modelMatrix（Cesium 再乘上 glTF Y-up 軸修正與節點變換才真的畫；那兩個是固定的，
GLB 幾何稽核已證實），不是「螢幕上的像素」。俯仰 |θ| > 85° 時偏航／滾轉的分軸統計不算（萬向鎖），總旋轉角照算。

--fail-over DEG（要 --truth）：閘門是下面全部同時成立（summary.json 的 gate.checks 逐項列出）：
  每一種誤差（讀回、每幀、drawn、開機時間軸、獨立真值的開機時間軸＋逐階段（懸停／轉態／巡航，列數 ≥ --min-rows 的）、
  SITLLink 換算 vs 獨立真值）p95 ≤ DEG（獨立真值放在它自己的收到時刻上那一項只報告，見 evaluate_gate）；
  覆蓋率（ok 列數 ≥ --min-rows、暖機後 ok 比例 ≥ --min-ok-frac）；模型種類認得出來；頁面校正 = 寫死的；
  （可選）顯示延遲 p95 ≤ --max-delay-s。任一不過 ⇒ 結束碼 4。

輸出（--out 目錄）：
  samples.csv     每次讀回／每個算圖幀 × 每架一列（row_kind = read／frame）：真值／畫面／誤差／顯示延遲／模式／速度…
  summary.json    p50／p95／最大絕對誤差（偏航、俯仰、滾轉、總旋轉角）：全部、依模式、依速度段、依模型；延遲；樣本數；
                  相位直方圖（顯示時間落在兩筆樣本之間的哪裡）；開機時間軸擬合與抖動；校正對照；閘門；設定
  truth_link.csv  鏈路真值（每則 ATTITUDE 一列，含 att_boot_ms）；truth_raw.csv（有 --truth 時）；batches.csv（送進 3D 的每一筆）
  attitude_timeseries.png  真值 vs 畫面（偏航／俯仰／滾轉＋總誤差）時間序列（有 matplotlib 才畫）
  status.log      SITLLink 的 STATUSTEXT／錯誤；console.log 頁面 console；shots/*.png（--shot-every）

用法（**一次只跑一個**：會開真的 QtWebEngine／GPU；QT_QPA_PLATFORM=offscreen 會被改成 windows，否則沒有 WebGL）：
  # 已經在飛的 SITL（只看、不下指令）
  python tools/attitude_fidelity_probe.py --conn tcp:127.0.0.1:5760 --seconds 120
  # 加獨立真值鏈路（SITL 的 SERIAL1 = 基準埠 + 2）
  python tools/attitude_fidelity_probe.py --conn tcp:127.0.0.1:6760 --truth tcp:127.0.0.1:6762 --seconds 120
  # 舊行為（機鼻跟航跡）vs 新行為對照：同一段飛行各跑一次
  python tools/attitude_fidelity_probe.py --conn ... --nose-follows-track off
  # 自己飛一趟：上傳任務（VTOL 起飛到 60 m、600 m 正方形三個角、回 Home 垂直降落）→ 真解鎖 → AUTO → 錄到落地上鎖
  python tools/attitude_fidelity_probe.py --conn tcp:127.0.0.1:6760 --truth tcp:127.0.0.1:6762 --fly --seconds 900
  # 加閘門（結束碼 4 = 不過；summary.json 的 gate.checks 逐項列出哪一項）
  python tools/attitude_fidelity_probe.py --conn tcp:127.0.0.1:6760 --truth tcp:127.0.0.1:6762 --fly --fail-over 0.5
  # 牙齒檢查：把頁面的模型姿態換回舊的角度相加寫法（滾轉誤差應該很大）／不插值（畫前一筆）
  python tools/attitude_fidelity_probe.py --conn ... --mutate old-angle-addition
  python tools/attitude_fidelity_probe.py --conn ... --mutate hold-previous
  # 不用 SITL 先試：假飛控（tests/fake_mav_vehicle.py；--script sine 一直在彎、--att-phase 錯相、--burst-every 成批）
  python tests/fake_mav_vehicle.py --port 6760 --truth-port 6762 --crab 10
  python tests/fake_mav_vehicle.py --port 6760 --truth-port 6762 --script sine --att-phase 0.023

SITL 的埠（ArduPlane --instance N）：SERIAL0 = tcp 5760+10N（GUI 預設連這個）、SERIAL1 = tcp 5762+10N、SERIAL2 = tcp 5763+10N；
每個 TCP 埠只收一個客戶端 ⇒ GUI 已經連著 SERIAL0 時，探針要走 SERIAL1／2（或 --serial1 udpclient:… ＋ udpin:…）。
--sysid 要等於飛控的 MAV_SYSID（舊韌體叫 SYSID_THISMAV；ArduPlane.exe --sysid N）—— GUI 只畫 sysid == 鏈路標示的那一架。
SITL 開了 --speedup > 1 時姿態變化也快 N 倍、串流頻率（模擬時間）也變 N 倍（量測照樣成立，但要看「使用者看到的樣子」請用 1 倍速）。
不讀 config/settings.json（任何 ui／mission 模組 import 之前就把全域設定換成「指向不存在的檔」的預設值、ion token 留空）、
頁面暫存檔寫在 --out 底下、預設擋掉所有非 file:／data:／blob: 請求（衛星圖磚也擋 ⇒ 地球是底色；--allow-network 才放行）。
GLB 合併快取預設跟 app 一樣（data/cache/models/；AEROPLAN_GLB_MERGE=0 ⇒ 原檔），--model-cache 可改到別處。不開 SITL。
⚠ --fly 用 SITLLink.arm()：它在 PreArm 擋下時會改走 FORCE ARM。這支一看到 FORCE 就立刻上鎖並中止（不在 force 下飛）。
⚠ ArduPilot SITL 的 TCP 客戶端一斷線 SITL 自己會結束：探針結束時鏈路關閉 ⇒ 若 SITL 沒有其他客戶端撐著就會跟著結束
  （同一個 SITL 要量好幾趟：另外留一個客戶端撐著 SERIAL0，探針走 SERIAL1／2）。--fly 錄到時間到還沒落地，飛機會繼續照任務飛。
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import pathlib
import random
import sys
import threading
import time
import types

ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

APP_GPU_FLAGS = ('--ignore-gpu-blocklist --enable-gpu-rasterization --enable-zero-copy '
                 '--enable-accelerated-video-decode')        # 與 main.py::_apply_gpu_acceleration 相同


# ═══════════════════════════════════════════════════════════════════════
#  姿態數學（純 Python；測試直接呼叫）
#  DCM C：機體 FRD → NED，C = Rz(偏航)·Ry(俯仰)·Rx(滾轉)（航空 3-2-1、ArduPilot 的歐拉角）
#  四元數 (w, x, y, z)，表示同一個旋轉
# ═══════════════════════════════════════════════════════════════════════
def wrap180(x: float) -> float:
    return (float(x) + 180.0) % 360.0 - 180.0


def dcm_from_euler(yaw: float, pitch: float, roll: float) -> list:
    ps, th, ph = math.radians(yaw), math.radians(pitch), math.radians(roll)
    cps, sps, cth, sth, cph, sph = math.cos(ps), math.sin(ps), math.cos(th), math.sin(th), math.cos(ph), math.sin(ph)
    return [[cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
            [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
            [-sth, sph * cth, cph * cth]]


def euler_from_dcm(c) -> tuple:
    """(偏航 0–360, 俯仰, 滾轉)，度。"""
    yaw = math.degrees(math.atan2(c[1][0], c[0][0])) % 360.0
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -c[2][0]))))
    roll = math.degrees(math.atan2(c[2][1], c[2][2]))
    return yaw, pitch, roll


def quat_from_dcm(c) -> tuple:
    tr = c[0][0] + c[1][1] + c[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (c[2][1] - c[1][2]) / s, (c[0][2] - c[2][0]) / s, (c[1][0] - c[0][1]) / s)
    elif c[0][0] > c[1][1] and c[0][0] > c[2][2]:
        s = math.sqrt(1.0 + c[0][0] - c[1][1] - c[2][2]) * 2
        q = ((c[2][1] - c[1][2]) / s, 0.25 * s, (c[0][1] + c[1][0]) / s, (c[0][2] + c[2][0]) / s)
    elif c[1][1] > c[2][2]:
        s = math.sqrt(1.0 + c[1][1] - c[0][0] - c[2][2]) * 2
        q = ((c[0][2] - c[2][0]) / s, (c[0][1] + c[1][0]) / s, 0.25 * s, (c[1][2] + c[2][1]) / s)
    else:
        s = math.sqrt(1.0 + c[2][2] - c[0][0] - c[1][1]) * 2
        q = ((c[1][0] - c[0][1]) / s, (c[0][2] + c[2][0]) / s, (c[1][2] + c[2][1]) / s, 0.25 * s)
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def dcm_from_quat(q) -> list:
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def slerp(q0, q1, u: float) -> tuple:
    d = sum(a * b for a, b in zip(q0, q1))
    if d < 0:                                    # 走短的那一邊
        q1, d = tuple(-v for v in q1), -d
    if d > 0.9995:
        q = tuple(a + u * (b - a) for a, b in zip(q0, q1))
    else:
        th = math.acos(min(1.0, d))
        s0, s1 = math.sin((1 - u) * th) / math.sin(th), math.sin(u * th) / math.sin(th)
        q = tuple(s0 * a + s1 * b for a, b in zip(q0, q1))
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def dcm_from_enu_axes(nose, right, up) -> list:
    """ENU 下的機體三軸（機鼻、右翼、機背）→ C（機體 FRD → NED）。ENU→NED：(n, e, d) = (y, x, −z)。"""
    def ned(v):
        return (v[1], v[0], -v[2])
    c0, c1, c2 = ned(nose), ned(right), ned((-up[0], -up[1], -up[2]))
    return [[c0[i], c1[i], c2[i]] for i in range(3)]


def rotation_angle_deg(ca, cb) -> float:
    """兩個姿態之間的總旋轉角（測地距離，度）：與歐拉角的分解方式無關。"""
    tr = sum(ca[k][i] * cb[k][i] for i in range(3) for k in range(3))       # trace(Caᵀ·Cb)
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


class TruthSeries:
    """一架的真值序列：(t 秒, roll, pitch, yaw) → 依時間 slerp。同一個時間戳重複的只留一筆。"""

    def __init__(self, samples):
        d = {}
        for t, r, p, y in samples:
            t = float(t)
            if t > 0 and t not in d:
                d[t] = (float(r), float(p), float(y))
        items = sorted(d.items())
        self.t = [k for k, _ in items]
        self.rpy = [v for _, v in items]
        self.q = [quat_from_dcm(dcm_from_euler(y, p, r)) for (r, p, y) in self.rpy]

    def __len__(self):
        return len(self.t)

    def at(self, t: float):
        """回 (四元數 | None, 狀態, 夾住它的兩筆間隔 s | None)。狀態：ok（落在範圍內）／before／hold（晚於最後一筆）／none。"""
        n = len(self.t)
        if n == 0:
            return None, 'none', None
        i = bisect.bisect_right(self.t, t) - 1
        if i < 0:
            return self.q[0], 'before', None
        if i >= n - 1:
            return self.q[-1], ('ok' if t == self.t[-1] else 'hold'), None
        t0, t1 = self.t[i], self.t[i + 1]
        return slerp(self.q[i], self.q[i + 1], (t - t0) / (t1 - t0)), 'ok', t1 - t0

    def phase(self, t: float):
        """t 落在夾住它的兩筆之間的哪裡（0 = 前一筆、→1 = 後一筆）；不在範圍內回 None。"""
        i = bisect.bisect_right(self.t, t) - 1
        if i < 0 or i >= len(self.t) - 1:
            return None
        return (t - self.t[i]) / (self.t[i + 1] - self.t[i])


def percentile(vals, p: float):
    v = sorted(x for x in vals if x is not None and math.isfinite(x))
    if not v:
        return None
    k = (len(v) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] if lo == hi else v[lo] + (v[hi] - v[lo]) * (k - lo)


def stats(vals) -> dict:
    v = [x for x in vals if x is not None and math.isfinite(x)]
    if not v:
        return {'n': 0, 'p50': None, 'p95': None, 'max': None, 'mean': None}
    r = lambda x: round(x, 4)   # noqa: E731
    return {'n': len(v), 'p50': r(percentile(v, 0.5)), 'p95': r(percentile(v, 0.95)), 'max': r(max(v)),
            'mean': r(sum(v) / len(v))}


# ═══════════════════════════════════════════════════════════════════════
#  讀回 → 每列誤差
# ═══════════════════════════════════════════════════════════════════════
#: frames（鏈路真值）每筆的欄位（_TruthTap.on_frame 依這個順序記）。att_boot_ms 只在「這一幀是剛收到的 ATTITUDE」時有值
#: （探針自己從 pymavlink 快取讀、和 SITLLink 無關）；att_ts = SITLLink 給這則姿態的取樣時刻（飛控時間換到牆鐘，
#: 2026-10-22 起頁面的樣本時間就是它；舊資料沒有這欄 ⇒ 退回 att_t）
FRAME_FIELDS = ('t', 'sysid', 'mode', 'armed', 'speed', 'vt', 'att_t', 'roll', 'pitch', 'yaw', 'lat', 'lon', 'alt',
                'gps_fix', 'vel_n', 'vel_e', 'att_boot_ms', 'att_ts')


def page_sample_t(f: dict) -> float:
    """頁面拿來當這則姿態樣本時間的那個數（秒）：SITLLink 的 att_ts（> 0 才算），沒有就是收到時刻 att_t。"""
    ts = f.get('att_ts')
    return float(ts) if isinstance(ts, (int, float)) and ts > 0 else float(f.get('att_t') or 0.0)
#: samples.csv 的欄位（row_kind：read = 讀回當下、frame = 頁面每一個算圖幀）
COLUMNS = ('row_kind', 't_read', 't_disp', 'sysid', 'status', 'mode', 'armed', 'speed', 'vt', 'model', 'nose_flag',
           'delay_s', 'truth_gap_s', 'truth_phase', 'page_att_age_s',
           'truth_yaw', 'truth_pitch', 'truth_roll', 'rend_yaw', 'rend_pitch', 'rend_roll',
           'err_yaw', 'err_pitch', 'err_roll', 'err_total',
           'page_yaw', 'page_nose', 'page_crs', 'crs_minus_yaw', 'page_pitch', 'page_roll', 'page_nAtt',
           'raw_yaw', 'raw_pitch', 'raw_roll', 'err_raw_yaw', 'err_raw_pitch', 'err_raw_roll', 'err_raw_total',
           'boot_yaw', 'boot_pitch', 'boot_roll', 'err_boot_yaw', 'err_boot_pitch', 'err_boot_roll', 'err_boot_total',
           'err_rawboot_yaw', 'err_rawboot_pitch', 'err_rawboot_roll', 'err_rawboot_total',
           'drawn_t_disp', 'drawn_tick', 'drawn_err_yaw', 'drawn_err_pitch', 'drawn_err_roll', 'drawn_err_total',
           'lag_err_yaw', 'lag_err_pitch', 'lag_err_roll', 'lag_err_total')
SPEED_BANDS = ((3.0, 'lt3'), (6.0, '3to6'), (float('inf'), 'ge6'))    # 與 main.html 的機鼻混合門檻一致（3→6 m/s）
#: |俯仰| 超過這個度數 ⇒ 偏航／滾轉的分軸統計不算（萬向鎖：分軸沒有意義），總旋轉角照算
GIMBAL_PITCH_DEG = 85.0
#: 有 GLB 的模型種類（探針用 entity 載入的 URI 認）
GLB_KINDS = ('hero', 'pioneer', 'vtol', 'plane', 'copter')
#: 校正（航向／俯仰／滾轉偏移，度）**寫死在這裡**：使用者目視確認過，2026-10-22 GLB 幾何稽核（Cesium 1.115 的 glTF 軸修正
#: × HPR(−90, 90, 180) 剛好是單位矩陣；vtol／copter 原始座標 x 右、y 機鼻、z 上；Barracuda 的 glTF +X 是機鼻）證實。
#: 探針用這一組去掉校正、不用頁面常數 ⇒ 頁面常數被改錯會直接變成姿態誤差（也列在 summary.json 的 calibration）。
#: hero／pioneer（2026-10-23）：tools/glb_kit.py 匯出時就是 Barracuda 慣例（glTF +X 機鼻、+Y 上、+Z 右翼）⇒ 0/0/0，
#: 由匯出程式的座標轉換決定（不是目視），經真 Cesium 的軸向截圖確認。
PINNED_OFFSETS = {'hero': (0.0, 0.0, 0.0), 'pioneer': (0.0, 0.0, 0.0), 'vtol': (-90.0, 90.0, 180.0),
                  'plane': (0.0, 0.0, 0.0), 'copter': (-90.0, 90.0, 180.0)}


def speed_band(v) -> str:
    v = float(v or 0.0)
    for hi, name in SPEED_BANDS:
        if v < hi:
            return name
    return SPEED_BANDS[-1][1]


def _errors(c_truth, c_rend) -> dict:
    ty, tp, tr = euler_from_dcm(c_truth)
    ry, rp, rr = euler_from_dcm(c_rend)
    return {'truth': (ty, tp, tr), 'rend': (ry, rp, rr),
            'err': (wrap180(ry - ty), rp - tp, wrap180(rr - tr), rotation_angle_deg(c_truth, c_rend))}


def _axes_dcm(ax):
    return dcm_from_enu_axes(ax['nose'], ax['right'], ax['up'])


# ── 開機時間軸的真值（2026-10-22 審查：收到時刻 att_t 就是頁面的樣本時間 ⇒ 拿它當真值看不到成批到、排隊、時間戳接錯）──
def fit_lower_envelope(xs, ys):
    """(a, b)：對每一點都 y ≥ a + b·x、而且在 x 的平均處最高的那條線（= 下凸包在 x̄ 的支撐線，也就是「殘差全 ≥ 0 時
    殘差和最小」的線性規劃解）。x = 飛控開機時間（s）、y = 收到時刻（牆鐘 s）⇒ 線 = 最快那幾則的收到時刻；殘差 = 比最快
    多等了多久（排隊、成批到）—— 成批只會把點往上推，不會拉歪這條線。斜率 b 吃得下 --speedup（b ≈ 1/speedup）。
    點少於 2 個（或 x 全相同）回 None。"""
    best: dict = {}
    for x, y in zip(xs, ys):
        x, y = float(x), float(y)
        if x not in best or y < best[x]:
            best[x] = y
    pts = sorted(best.items())
    if len(pts) < 2:
        return None
    hull: list = []
    for p in pts:                                    # Andrew 單調鏈：下凸包
        while len(hull) >= 2 and ((hull[-1][0] - hull[-2][0]) * (p[1] - hull[-2][1])
                                  - (hull[-1][1] - hull[-2][1]) * (p[0] - hull[-2][0])) <= 0:
            hull.pop()
        hull.append(p)
    xs = [float(x) for x in xs]
    xbar = sum(xs) / len(xs)
    edge = hull[0], hull[1]
    for e in zip(hull, hull[1:]):
        if e[0][0] <= xbar <= e[1][0]:
            edge = e
            break
    (x0, y0), (x1, y1) = edge
    b = (y1 - y0) / (x1 - x0)
    return y0 - b * x0, b


def _boot_segments(samples):
    """依收到時刻排序；開機時間倒退 > 1 s（飛控重開機）就切一段。samples：[(t_recv, boot_s, r, p, y)]。"""
    out, cur, last = [], [], None
    for s in sorted(samples):
        if last is not None and s[1] < last - 1.0:
            out.append(cur)
            cur = []
        cur.append(s)
        last = s[1]
    if cur:
        out.append(cur)
    return out


def boot_clock_series(samples, skip_first_s: float = 0.0):
    """[(t_recv, boot_s, r, p, y)] → (TruthSeries：時間 = 開機時間經下包絡換成的牆鐘, 統計)。

    統計：每段的擬合（a、b、筆數）；latency_above_min_s = 收到時刻 − 下包絡（抖動、排隊）；gaps = 相鄰兩筆的間隔數
    （開機時間差 ≥ 10 ms 的）；compressed_gaps = 收到間隔 < 0.25 × 開機時間間隔（被壓扁＝成批到）；bursts = 連著 ≥ 2 個
    被壓扁的間隔（≥ 3 則一起到）的次數。這三項只算第一筆之後 skip_first_s 秒起的（連線那一下 SITLLink 會先同步讀
    Q_ENABLE、再逐一要 80 個 SERVOn_* 參數（每個睡 20 ms ≈ 1.6 s）才進主迴圈 ⇒ 這段的訊息一定成批到，是連線時的事，
    另外記在 latency_above_min_s_incl_start）；擬合與真值用全部。"""
    mapped, lat, lat_all, fits = [], [], [], []
    gaps = comp = bursts = 0
    t_first = min((s[0] for s in samples), default=0.0)
    for seg in _boot_segments(samples):
        fit = fit_lower_envelope([s[1] for s in seg], [s[0] for s in seg])
        if fit is None:
            continue
        a, b = fit
        fits.append({'a': round(a, 6), 'b': round(b, 9), 'n': len(seg),
                     'boot_s': [round(seg[0][1], 3), round(seg[-1][1], 3)]})
        prev, run = None, 0
        for t, bs, r, p, y in seg:
            tw = a + b * bs
            mapped.append((tw, r, p, y))
            lat_all.append(t - tw)
            if t < t_first + skip_first_s:
                prev = (t, bs)
                continue
            lat.append(t - tw)
            if prev is not None:
                dboot, drecv = b * (bs - prev[1]), t - prev[0]
                if dboot >= 0.01:
                    gaps += 1
                    if drecv < 0.25 * dboot:
                        comp += 1
                        run += 1
                        if run == 2:
                            bursts += 1
                    else:
                        run = 0
            prev = (t, bs)
    st = {'segments': fits, 'n': len(mapped), 'skipped_first_s': skip_first_s, 'latency_above_min_s': stats(lat),
          'latency_above_min_s_incl_start': stats(lat_all), 'gaps': gaps, 'compressed_gaps': comp, 'bursts': bursts}
    return TruthSeries(mapped), st


def build_boot_truth(frames, raw_samples=None, skip_first_s: float = 0.0) -> dict:
    """每架兩份開機時間軸的真值：link（SITLLink 那一則 ATTITUDE 的 time_boot_ms）、raw（獨立鏈路）。各自擬合（各自的最小延遲）。"""
    link: dict = {}
    raw: dict = {}
    for fr in frames:
        f = dict(zip(FRAME_FIELDS, fr))
        bm = f.get('att_boot_ms')
        if bm is None or not f.get('att_t'):
            continue
        link.setdefault(int(f['sysid']), []).append((float(f['att_t']), bm / 1000.0, f['roll'], f['pitch'], f['yaw']))
    for t, sid, bm, r, p, y in raw_samples or []:
        raw.setdefault(int(sid), []).append((float(t), bm / 1000.0, r, p, y))
    out = {'link': {}, 'raw': {}, 'stats': {'link': {}, 'raw': {}}}
    for name, src in (('link', link), ('raw', raw)):
        for sid, ss in src.items():
            ts, st = boot_clock_series(ss, skip_first_s)
            out[name][sid] = ts
            out['stats'][name][str(sid)] = st
    return out


def link_vs_raw_boot(frames, raw_samples) -> dict:
    """SITLLink 解出來的每一則姿態 vs 獨立鏈路**同一個飛控時刻**（開機時間軸上內插）的總旋轉角：沒有任何時鐘、排隊的影響，
    只剩 SITLLink 的換算（單位、正負號、軸序）錯誤。"""
    raw: dict = {}
    for _t, sid, bm, r, p, y in raw_samples or []:
        raw.setdefault(int(sid), []).append((bm / 1000.0, r, p, y))
    series = {s: TruthSeries(v) for s, v in raw.items()}
    vals = []
    for fr in frames:
        f = dict(zip(FRAME_FIELDS, fr))
        bm, ts = f.get('att_boot_ms'), series.get(int(f['sysid']))
        if bm is None or ts is None:
            continue
        q, st, _ = ts.at(bm / 1000.0)
        if st == 'ok':
            vals.append(rotation_angle_deg(dcm_from_quat(q), dcm_from_euler(f['yaw'], f['pitch'], f['roll'])))
    return stats(vals)


def common_boot_fraction(frames, raw_samples, skip_first_s: float = 0.0) -> dict:
    """每架：獨立真值的樣本裡，有多少和主鏈路是**同一個飛控取樣時刻**（time_boot_ms 相同）。只算主鏈路開機時間範圍內、
    第一筆之後 skip_first_s 秒起的。50 Hz 對 50 Hz、相位對齊 ⇒ ≈ 1；錯半個間隔 ⇒ 0（見 RawTruth）。"""
    link: dict = {}
    for fr in frames:
        f = dict(zip(FRAME_FIELDS, fr))
        if f.get('att_boot_ms') is not None:
            link.setdefault(int(f['sysid']), set()).add(int(f['att_boot_ms']))
    raw: dict = {}
    for t, sid, bm, *_ in raw_samples or []:
        raw.setdefault(int(sid), []).append((float(t), int(bm)))
    out = {}
    for sid, rs in raw.items():
        ls = link.get(sid)
        if not ls or not rs:
            continue
        t0, lo, hi = rs[0][0] + skip_first_s, min(ls), max(ls)
        inr = [bm for t, bm in rs if t >= t0 and lo <= bm <= hi]
        k = sum(1 for bm in inr if bm in ls)
        out[str(sid)] = {'raw_in_range': len(inr), 'common': k, 'frac': round(k / len(inr), 4) if inr else None,
                         'link_n': len(ls)}
    return out


def _read_time(t_read, data) -> float:
    """讀回的「現在」：JS 執行當下的 Date.now()（data['ms']）；沒有才用 Python 收到回呼的時刻（多了 runJavaScript 來回）。"""
    ms = (data or {}).get('ms')
    return float(ms) / 1000.0 if isinstance(ms, (int, float)) and ms > 0 else float(t_read)


def _fill_errors(row, prefix, series, t, c_r, truth_cols=None):
    """series 在 t 內插 vs 畫面 c_r ⇒ row[prefix + yaw/pitch/roll/total]（只有 ok 才填）。"""
    if series is None or not len(series):
        return
    q, st, _ = series.at(t)
    if st != 'ok':
        return
    e = _errors(dcm_from_quat(q), c_r)
    if truth_cols:
        for c, v in zip(truth_cols, e['truth']):
            row[c] = round(v, 4)
    for ax, v in zip(('yaw', 'pitch', 'roll', 'total'), e['err']):
        row[prefix + ax] = round(v, 4)


def compute_rows(reads, frames, raw_samples=None, *, warmup_s: float = 1.0, boot=None) -> list:
    """reads：[(t_read, 頁面讀回 dict)]；frames：FRAME_FIELDS 元組清單；raw_samples：[(t, sysid, boot_ms, r, p, y)]；
    boot：build_boot_truth() 的結果（省略就現算）。

    每次讀回 × 每架一列（row_kind = read），加上這次讀回帶回來的每一個算圖幀一列（row_kind = frame；頁面 u['frames']）。
    dict，鍵 = COLUMNS。狀態：
      ok      顯示時間落在真值範圍內（統計只用這些）
      warmup  該架第一次出現後 warmup_s 秒內（顯示延遲還在收斂；時間 = JS 的 Date.now()）
      before／hold  顯示時間早於第一筆／晚於最後一筆真值
      no_render     頁面還沒有姿態（entity 沒有 orientation）
    """
    by_sid: dict = {}
    for fr in frames:
        f = dict(zip(FRAME_FIELDS, fr))
        by_sid.setdefault(int(f['sysid']), []).append(f)
    # 鏈路真值的時間 = 頁面拿到的樣本時間（page_sample_t）：這一組只驗畫法＋插值＋合幀（每一筆有沒有送到、送到時對不對）
    truth = {s: TruthSeries((page_sample_t(f), f['roll'], f['pitch'], f['yaw']) for f in fs) for s, fs in by_sid.items()}
    timeline = {s: sorted(fs, key=lambda f: f['t']) for s, fs in by_sid.items()}
    tl_t = {s: [f['t'] for f in fs] for s, fs in timeline.items()}
    raw = {}
    for t, sid, _boot, r, p, y in (raw_samples or []):
        raw.setdefault(int(sid), []).append((t, r, p, y))
    raw_truth = {s: TruthSeries(v) for s, v in raw.items()}
    if boot is None:
        boot = build_boot_truth(frames, raw_samples)

    first_seen: dict = {}
    last_drawn_tick: dict = {}

    def make(sid, row_kind, t_now, u, flag):
        t_disp = float(u['disp']) / 1000.0
        fs, ts = timeline.get(sid, []), tl_t.get(sid, [])
        j = bisect.bisect_right(ts, t_disp) - 1
        f = fs[max(0, j)] if fs else {}
        crs, yaw = u.get('crs'), u.get('yaw')
        row = {c: None for c in COLUMNS}
        row.update(row_kind=row_kind, t_read=round(t_now, 4), t_disp=round(t_disp, 4), sysid=sid,
                   mode=f.get('mode', ''), armed=f.get('armed'), speed=f.get('speed'), vt=f.get('vt', ''),
                   model=u.get('kind'), nose_flag=flag, delay_s=u.get('delay'),
                   page_yaw=yaw, page_nose=u.get('nose'), page_crs=crs,
                   crs_minus_yaw=(round(wrap180(crs - yaw), 3) if isinstance(crs, (int, float))
                                  and isinstance(yaw, (int, float)) else None),
                   page_pitch=u.get('pitch'), page_roll=u.get('roll'), page_nAtt=u.get('nAtt'),
                   page_att_age_s=(round(t_disp - u['lastAttMs'] / 1000.0, 4) if u.get('lastAttMs') else None))
        ts_obj = truth.get(sid)
        if not u.get('ax'):
            row['status'] = 'no_render'
            return row
        if ts_obj is None or not len(ts_obj):
            row['status'] = 'none'
            return row
        q, st, gap = ts_obj.at(t_disp)
        row['truth_gap_s'] = round(gap, 4) if gap is not None else None
        ph = ts_obj.phase(t_disp)
        row['truth_phase'] = round(ph, 4) if ph is not None else None
        if st == 'ok' and t_now - first_seen[sid] < warmup_s:
            st = 'warmup'
        row['status'] = st
        c_r = _axes_dcm(u['ax'])
        e = _errors(dcm_from_quat(q), c_r)
        for name, vals in (('truth', e['truth']), ('rend', e['rend'])):
            row[f'{name}_yaw'], row[f'{name}_pitch'], row[f'{name}_roll'] = (round(v, 4) for v in vals)
        row['err_yaw'], row['err_pitch'], row['err_roll'], row['err_total'] = (round(v, 4) for v in e['err'])
        if row_kind == 'read':
            # 牆鐘「現在」（JS 執行當下）的真值：含顯示延遲（使用者實際看到的落後）
            _fill_errors(row, 'lag_err_', ts_obj, t_now, c_r)
        _fill_errors(row, 'err_raw_', raw_truth.get(sid), t_disp, c_r, ('raw_yaw', 'raw_pitch', 'raw_roll'))
        _fill_errors(row, 'err_boot_', boot['link'].get(sid), t_disp, c_r, ('boot_yaw', 'boot_pitch', 'boot_roll'))
        _fill_errors(row, 'err_rawboot_', boot['raw'].get(sid), t_disp, c_r)
        if row_kind == 'frame' and u.get('dax'):
            # 同一幀：模型 primitive 的 modelMatrix（Viewer 的 onTick 剛更新、這一幀就畫它）在同一個顯示時間比真值
            ed = _errors(dcm_from_quat(q), _axes_dcm(u['dax']))
            row['drawn_t_disp'], row['drawn_tick'] = round(t_disp, 4), u.get('tick')
            (row['drawn_err_yaw'], row['drawn_err_pitch'], row['drawn_err_roll'],
             row['drawn_err_total']) = (round(v, 4) for v in ed['err'])
        dr = u.get('drawn')                                 # 舊格式（讀回時帶上一幀的矩陣）：同一幀只算一次
        if row_kind == 'read' and dr and dr.get('ax') and dr.get('tick') != last_drawn_tick.get(sid):
            last_drawn_tick[sid] = dr.get('tick')
            td = float(dr['disp']) / 1000.0
            qd, std, _ = ts_obj.at(td)
            if std == 'ok':
                ed = _errors(dcm_from_quat(qd), _axes_dcm(dr['ax']))
                row['drawn_t_disp'], row['drawn_tick'] = round(td, 4), dr.get('tick')
                (row['drawn_err_yaw'], row['drawn_err_pitch'], row['drawn_err_roll'],
                 row['drawn_err_total']) = (round(v, 4) for v in ed['err'])
        return row

    rows = []
    for t_read, data in reads:
        t_now = _read_time(t_read, data)
        uav = (data or {}).get('uav') or {}
        for k, u in uav.items():
            sid = int(k)
            # 這一架真的取值的那一毫秒（READ_JS 每架各自的 ms）；沒有就用整次讀回的時間
            t_u = float(u['ms']) / 1000.0 if isinstance(u.get('ms'), (int, float)) and u['ms'] > 0 else t_now
            frs = [fr for fr in (u.get('frames') or []) if fr.get('ms') and fr.get('disp') is not None]
            first_seen.setdefault(sid, min([t_u] + [float(fr['ms']) / 1000.0 for fr in frs]))
            for fr in frs:                       # 這次讀回之前的那幾幀（時間在讀回之前）
                rows.append(make(sid, 'frame', float(fr['ms']) / 1000.0, fr, data.get('flag')))
            rows.append(make(sid, 'read', t_u, u, data.get('flag')))
    return rows


def _err_block(rows, prefix: str = 'err_') -> dict:
    out = {'n': len(rows)}
    for ax in ('yaw', 'pitch', 'roll', 'total'):
        rs = [r for r in rows if r.get(f'{prefix}{ax}') is not None]
        if ax in ('yaw', 'roll'):                  # 萬向鎖附近分軸沒有意義（總旋轉角照算）
            keep = [r for r in rs if r.get('truth_pitch') is None or abs(r['truth_pitch']) <= GIMBAL_PITCH_DEG]
            if len(keep) != len(rs):
                out['gimbal_rows_skipped'] = len(rs) - len(keep)
            rs = keep
        vals = [r[f'{prefix}{ax}'] for r in rs]
        out[ax] = stats([abs(v) for v in vals])
        if ax != 'total':
            out[ax]['bias'] = round(sum(vals) / len(vals), 4) if vals else None      # 有號平均：系統性偏差
    return out


def _is_frame(r) -> bool:
    return r.get('row_kind') == 'frame'


def _error_groups(ok) -> dict:
    modes = sorted({str(r['mode']) for r in ok})
    bands = [b for _, b in SPEED_BANDS if any(speed_band(r['speed']) == b for r in ok)]
    models = sorted({str(r['model']) for r in ok})
    return {'all': _err_block(ok),
            # 頁面這一刻還有更新的姿態樣本（沒有在 HOLD）：只剩畫法＋插值；HOLD 的列是主執行緒卡住、
            # 批次晚到（例如 GLB 載入那一下），誤差 ≈ 姿態角速度 × page_att_age_s
            'page_not_holding': _err_block([r for r in ok if (r['page_att_age_s'] or 0) <= 0]),
            'by_mode': {m: _err_block([r for r in ok if str(r['mode']) == m]) for m in modes},
            'by_speed_band': {b: _err_block([r for r in ok if speed_band(r['speed']) == b]) for b in bands},
            'by_model': {m: _err_block([r for r in ok if str(r['model']) == m]) for m in models}}


def phase_histogram(rows, bins: int = 10) -> list:
    """顯示時間落在兩筆真值之間的哪裡（0–1 分 bins 格）的次數。全擠在一格 ＝ 讀回和樣本鎖相（只量到一個相位）。"""
    h = [0] * bins
    for r in rows:
        ph = r.get('truth_phase')
        if ph is not None:
            h[min(bins - 1, max(0, int(ph * bins)))] += 1
    return h


def calibration_check(page_info: dict) -> dict:
    """頁面上的校正常數 vs 探針寫死的那組（PINNED_OFFSETS）。"""
    page = (page_info or {}).get('offsets') or {}
    mismatch = []
    for kind, pinned in PINNED_OFFSETS.items():
        got = page.get(kind)
        if got is None or len(got) != 3 or any(abs(float(a) - b) > 1e-9 for a, b in zip(got, pinned)):
            mismatch.append({'kind': kind, 'page': got, 'pinned': list(pinned)})
    return {'pinned': {k: list(v) for k, v in PINNED_OFFSETS.items()}, 'page': page, 'mismatch': mismatch}


#: 飛行階段（依地速，與 SPEED_BANDS 同界線）：懸停 < 3 m/s、轉態／低速 3–6 m/s、巡航 ≥ 6 m/s
PHASE_OF_BAND = {'lt3': 'hover', '3to6': 'transition', 'ge6': 'cruise'}


def _phase_blocks(rows, prefix: str) -> dict:
    """依飛行階段分組的 _err_block（只放有列的階段）。"""
    out = {}
    for band, name in PHASE_OF_BAND.items():
        rs = [r for r in rows if r.get(f'{prefix}total') is not None and speed_band(r['speed']) == band]
        if rs:
            out[name] = _err_block(rs, prefix)
    return out


def _mode_blocks(rows, prefix: str) -> dict:
    rs = [r for r in rows if r.get(f'{prefix}total') is not None]
    return {m: _err_block([r for r in rs if str(r['mode']) == m], prefix) for m in sorted({str(r['mode']) for r in rs})}


def summarize(rows, meta: dict | None = None) -> dict:
    reads = [r for r in rows if not _is_frame(r)]
    frs = [r for r in rows if _is_frame(r)]
    ok = [r for r in reads if r['status'] == 'ok']
    okf = [r for r in frs if r['status'] == 'ok']
    best = okf or ok                                     # 有每幀的就用每幀（每一個畫出來的幀都算）
    by_status: dict = {}
    for r in reads:
        by_status[r['status']] = by_status.get(r['status'], 0) + 1
    fr_status: dict = {}
    for r in frs:
        fr_status[r['status']] = fr_status.get(r['status'], 0) + 1
    s = {
        'counts': {'rows': len(reads), 'by_status': by_status, 'frame_rows': len(frs), 'frames_by_status': fr_status,
                   'sysids': sorted({r['sysid'] for r in rows})},
        'errors_deg': _error_groups(ok),
        'errors_per_frame_deg': _error_groups(okf),
        'errors_vs_boot_clock_deg': _err_block([r for r in best if r['err_boot_total'] is not None], 'err_boot_'),
        'errors_vs_raw_truth_deg': _err_block([r for r in ok if r['err_raw_total'] is not None], 'err_raw_'),
        'errors_vs_raw_boot_clock_deg': _err_block([r for r in best if r['err_rawboot_total'] is not None],
                                                   'err_rawboot_'),
        'errors_vs_raw_boot_clock_by_phase': _phase_blocks(best, 'err_rawboot_'),
        'errors_vs_raw_boot_clock_by_mode': _mode_blocks(best, 'err_rawboot_'),
        'errors_vs_boot_clock_by_phase': _phase_blocks(best, 'err_boot_'),
        'errors_drawn_frame_deg': _err_block([r for r in ok + okf if r['drawn_err_total'] is not None], 'drawn_err_'),
        'errors_vs_wallclock_now_deg': _err_block([r for r in ok if r['lag_err_total'] is not None], 'lag_err_'),
        'phase_histogram': {'reads': phase_histogram(ok), 'frames': phase_histogram(okf)},
        'display_delay_s': stats([r['delay_s'] for r in best]),
        'page_hold_rows': sum(1 for r in ok if (r['page_att_age_s'] or 0) > 0),
        'page_hold_frames': sum(1 for r in okf if (r['page_att_age_s'] or 0) > 0),
        'page_att_age_s': stats([r['page_att_age_s'] for r in best]),
        'truth_gap_s': stats([r['truth_gap_s'] for r in best]),
        'nose_follows_track_observed': sorted({str(r['nose_flag']) for r in rows}),
        'models': sorted({str(r['model']) for r in rows}),
        'crs_minus_yaw_deg': stats([abs(r['crs_minus_yaw']) for r in ok if r['crs_minus_yaw'] is not None]),
    }
    if meta:
        s.update(meta)
    return s


def evaluate_gate(s: dict, limit_deg: float, *, need_raw: bool = True, min_rows: int = 50,
                  min_ok_frac: float = 0.9, max_delay_s=None) -> dict:
    """--fail-over 的閘門：全部同時成立才算過（2026-10-22 審查：以前只看 errors_deg.all 的 p95 ⇒ SITLLink 換算錯、
    姿態串流斷掉（之後的列全變 hold、被排除）、每一幀的畫法、成批到，全都過得了）。"""
    checks = []

    def add(name, value, limit, ok):
        checks.append({'name': name, 'value': value, 'limit': limit, 'ok': bool(ok)})

    def p95(name, block, required=True):
        n = (block or {}).get('n', 0)
        v = ((block or {}).get('total') or {}).get('p95')
        if not n or v is None:
            add(f'{name}.p95', None, f'≤ {limit_deg}（必要：{required}）', not required)
        else:
            add(f'{name}.p95', v, f'≤ {limit_deg}', v <= limit_deg)

    glb = any(m in GLB_KINDS for m in s.get('models', []))
    p95('errors_deg.all', s['errors_deg']['all'])
    p95('errors_per_frame_deg.all', s['errors_per_frame_deg']['all'])
    p95('errors_vs_boot_clock_deg', s['errors_vs_boot_clock_deg'])
    p95('errors_drawn_frame_deg', s['errors_drawn_frame_deg'], required=glb)
    # ⚠ errors_vs_raw_truth_deg（獨立真值放在**它自己的收到時刻**上）不進閘門（2026-10-22）：頁面的姿態樣本時間改成飛控時間
    #   換到牆鐘之後，收到時刻軸本身就是一把有 0～32 ms 抖動的尺 —— 頁面畫得完全對，在懸停振盪（~130°/s）時拿這把尺量也是
    #   幾度。獨立真值的判準是同一份資料放在飛控時間軸上（errors_vs_raw_boot_clock_deg，含逐階段）。
    p95('errors_vs_raw_boot_clock_deg', s['errors_vs_raw_boot_clock_deg'], required=need_raw)
    for phase, blk in (s.get('errors_vs_raw_boot_clock_by_phase') or {}).items():
        if (blk or {}).get('n', 0) >= min_rows:       # 列太少的階段（例如轉態只過一下）p95 沒有意義，只報告
            p95(f'errors_vs_raw_boot_clock_deg.{phase}', blk)
    lvr = ((s.get('raw_truth') or {}).get('link_vs_raw_boot_deg')) or {}
    if need_raw or lvr.get('n'):
        v = lvr.get('p95')
        add('link_vs_raw_boot_deg.p95', v, f'≤ {limit_deg}', v is not None and v <= limit_deg)
    for key, label in (('by_status', 'reads'), ('frames_by_status', 'frames')):
        bs = (s.get('counts') or {}).get(key) or {}
        n_ok = bs.get('ok', 0)
        n_eval = sum(v for k, v in bs.items() if k not in ('warmup', 'before'))   # before 只會出現在剛出現的那一下
        frac = (n_ok / n_eval) if n_eval else 0.0
        add(f'coverage.{label}.n_ok', n_ok, f'≥ {min_rows}', n_ok >= min_rows)
        add(f'coverage.{label}.ok_frac', round(frac, 4), f'≥ {min_ok_frac}', frac >= min_ok_frac)
    page = s.get('page') or {}
    uris = dict(page.get('uris') or {})
    af = (page.get('final') or {}).get('airframe') or page.get('airframe') or {}
    for name in ('hero', 'pioneer'):     # 機體模型只畫給被指定的 VTOL：沒人被指定 ⇒ 它的 URI 不算「頁面有 GLB」（審查）
        if uris.get(name) and not (af.get('dflt') == name or name in (af.get('map') or {}).values()):
            uris.pop(name)
    have_glb = any(uris.values())
    bad = [m for m in s.get('models', []) if m == 'unknown' or (m == 'none' and have_glb)]
    add('models', s.get('models'), '沒有 unknown；頁面有 GLB 時也沒有 none', not bad)
    mis = (s.get('calibration') or {}).get('mismatch')
    add('calibration.mismatch', mis, '[]（頁面校正 = PINNED_OFFSETS）', mis == [])
    if max_delay_s is not None:
        v = (s.get('display_delay_s') or {}).get('p95')
        add('display_delay_s.p95', v, f'≤ {max_delay_s}', v is not None and v <= max_delay_s)
    return {'limit_deg': limit_deg, 'ok': all(c['ok'] for c in checks), 'checks': checks}


def link_vs_raw(frames, raw_samples) -> dict:
    """兩份真值互相比：raw 每一筆的時刻上內插 link 真值 ⇒ 總旋轉角（兩條鏈路的時間差＋SITLLink 換算有沒有錯）。"""
    by_sid: dict = {}
    for fr in frames:
        f = dict(zip(FRAME_FIELDS, fr))
        by_sid.setdefault(int(f['sysid']), []).append((f['att_t'], f['roll'], f['pitch'], f['yaw']))
    series = {s: TruthSeries(v) for s, v in by_sid.items()}
    vals = []
    for t, sid, _b, r, p, y in raw_samples or []:
        ts = series.get(int(sid))
        if ts is None:
            continue
        q, st, _ = ts.at(t)
        if st == 'ok':
            vals.append(rotation_angle_deg(dcm_from_quat(q), dcm_from_euler(y, p, r)))
    return stats(vals)


def write_plot(rows, path, title: str = '') -> bool:
    """真值 vs 畫面的時間序列（偏航／俯仰／滾轉＋總誤差，共用時間軸），只畫 status=ok、第一架；有每幀的列就畫每幀
    （每一個畫出來的幀），否則畫讀回的列。沒有 matplotlib 就跳過。
    配色取 dataviz 參考調色盤的類別色 1、2（藍＝真值實線、橘＝畫面虛線：顏色以外還有線型可分）。"""
    ok = [r for r in rows if r['status'] == 'ok' and _is_frame(r)] or [r for r in rows if r['status'] == 'ok']
    if not ok:
        return False
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:                                           # noqa: BLE001
        return False
    sid = ok[0]['sysid']
    ok = sorted((r for r in ok if r['sysid'] == sid), key=lambda r: r['t_disp'])
    t0 = ok[0]['t_disp']
    t = [r['t_disp'] - t0 for r in ok]

    def broken(vals, wrap):                                     # 偏航跨 0/360 時斷線，不畫一條垂直線
        out, prev = [], None
        for v in vals:
            out.append(float('nan') if (wrap and prev is not None and abs(v - prev) > 180.0) else v)
            prev = v
        return out
    # 偏航用 0–360 或 ±180：挑真值斷線比較少的那一種（繞北飛時 ±180 連續、繞南飛時 0–360 連續）
    ty = [r['truth_yaw'] for r in ok]
    jumps = lambda vs: sum(abs(b - a) > 180.0 for a, b in zip(vs, vs[1:]))   # noqa: E731
    y180 = jumps([wrap180(v) for v in ty]) < jumps(ty)
    yconv = (lambda v: wrap180(v)) if y180 else (lambda v: v % 360.0)
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}       # 只列有的字型（沒有的每個字都會警告一次）
    plt.rcParams['font.family'] = [f for f in ('Microsoft JhengHei', 'Noto Sans CJK TC', 'PingFang TC')
                                   if f in have] + ['DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    ink, muted, grid = '#0b0b0b', '#52514e', '#e4e3df'
    c_truth, c_rend = '#2a78d6', '#eb6834'
    fig, axs = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True, facecolor='#fcfcfb')
    for ax, (key, label, wrap) in zip(axs, (('yaw', '偏航（度，北起順時針' + ('，±180）' if y180 else '）'), True),
                                            ('pitch', '俯仰（度）', False), ('roll', '滾轉（度，右翼下為正）', False))):
        conv = yconv if wrap else (lambda v: v)
        ax.plot(t, broken([conv(r[f'truth_{key}']) for r in ok], wrap), color=c_truth, lw=2, label='SITL 真值（鏈路）')
        ax.plot(t, broken([conv(r[f'rend_{key}']) for r in ok], wrap), color=c_rend, lw=2, ls=(0, (4, 2)),
                label='3D 模型畫出來的')
        ax.set_ylabel(label, color=muted, fontsize=9)
    axs[3].plot(t, [r['err_total'] for r in ok], color=muted, lw=2, label='對收到時刻真值')
    if any(r.get('err_boot_total') is not None for r in ok):
        axs[3].plot(t, [r.get('err_boot_total') if r.get('err_boot_total') is not None else float('nan') for r in ok],
                    color=c_rend, lw=1.5, ls=(0, (4, 2)), label='對開機時間軸真值')
        axs[3].legend(loc='upper right', frameon=False, fontsize=8, labelcolor=ink)
    axs[3].set_ylabel('總旋轉角誤差（度）', color=muted, fontsize=9)
    axs[3].set_xlabel('顯示時間（秒）', color=muted, fontsize=9)
    for ax in axs:
        ax.set_facecolor('#fcfcfb')
        ax.grid(True, color=grid, lw=0.8)
        ax.tick_params(colors=muted, labelsize=8)
        for sp in ax.spines.values():
            sp.set_visible(False)
    h, lab = axs[0].get_legend_handles_labels()
    fig.legend(h, lab, loc='upper right', frameon=False, fontsize=9, labelcolor=ink, ncol=2)
    fig.suptitle(title or f'UAV{sid} 姿態：SITL 真值 vs 3D 模型（同一個顯示時間）', color=ink, fontsize=11, x=0.02,
                 ha='left')
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(path, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)
    return True


def write_csv(path, header, rows):
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for r in rows:
            w.writerow([r.get(h) for h in header] if isinstance(r, dict) else list(r))


# ═══════════════════════════════════════════════════════════════════════
#  --fly 的任務：VTOL 起飛 → 正方形三個角 → 回 Home 垂直降落
# ═══════════════════════════════════════════════════════════════════════
def _offset_ll(lat, lon, dn, de):
    R = 6378137.0
    return lat + math.degrees(dn / R), lon + math.degrees(de / (R * math.cos(math.radians(lat))))


def build_square_mission(home_lat: float, home_lon: float, alt_m: float = 60.0, side_m: float = 600.0,
                         vehicle: str = 'VTOL') -> list:
    """SITLLink.upload_mission_items 的格式（seq 0 = Home 槽）。三個角 (side,0)→(side,side)→(0,side)（北、東為正）
    ⇒ 每個轉角約 90°，飛機要壓坡度轉彎；最後一段 side_m 公尺回 Home 降落（⚠ VTOL 降落前的最後航點離降落點太近會
    PreArm「VTOL land too short」，side_m 不要小於 ~150 m）。"""
    vtol = str(vehicle).upper() != 'PLANE'

    def item(seq, cmd, frame, lat, lon, alt, p1=0.0):
        return {'seq': seq, 'command': cmd, 'frame': frame, 'lat': float(lat), 'lon': float(lon), 'alt': float(alt),
                'param1': float(p1), 'param2': 0.0, 'param3': 0.0, 'param4': 0.0, 'current': 0, 'autocontinue': 1}
    REL = 3                                          # MAV_FRAME_GLOBAL_RELATIVE_ALT
    items = [item(0, 16, 0, home_lat, home_lon, 0.0)]
    items.append(item(1, 84, REL, home_lat, home_lon, alt_m) if vtol else
                 item(1, 22, REL, home_lat, home_lon, alt_m, p1=15.0))
    for dn, de in ((side_m, 0.0), (side_m, side_m), (0.0, side_m)):
        lat, lon = _offset_ll(home_lat, home_lon, dn, de)
        items.append(item(len(items), 16, REL, lat, lon, alt_m))
    items.append(item(len(items), 85 if vtol else 21, REL, home_lat, home_lon, 0.0))
    return items


# ═══════════════════════════════════════════════════════════════════════
#  頁面端 JS
# ═══════════════════════════════════════════════════════════════════════
READY_JS = "typeof viewer !== 'undefined' && typeof updateUAVPositions === 'function' && typeof _UAV_STATE !== 'undefined'"

# 一次性安裝：輔助函式（模型種類→探針寫死的校正、去校正→ENU 機體軸、找模型 primitive）＋ clock.onTick 每一幀記一筆
# （環形緩衝；加在 Viewer 自己的 onTick 之後 ⇒ DataSourceDisplay 剛更新完模型矩陣、這一幀就畫它，差 < 1 ms）：
# 顯示時間、entity.orientation（CallbackProperty 在這一架的顯示時間取插值）、primitive 的 modelMatrix。讀回時整批取走。
# 2026-10-22 審查：以前只在讀回當下取樣、讀回週期 = 樣本週期 ⇒ 每一趟只量到樣本間隔裡的一個相位。
# 校正偏移用 PINNED（Python 的 PINNED_OFFSETS 換進來），不用頁面常數；頁面常數另外回報（pageOffs）。
INSTALL_JS_TEMPLATE = r"""
(function () {
  if (window.__afp) return 'already';
  const A = window.__afp = {tick: null, ticks: 0, models: {}, lastSearch: {}, ring: {}, ringMax: 4000, dropped: 0,
                            pinned: /*PINNED*/null};
  A.kind = function (e) {
    let uri = '';
    try {
      if (e.model && e.model.uri) {
        const u = e.model.uri.getValue(Cesium.JulianDate.now());
        uri = (u && u.url) ? String(u.url) : String(u || '');
      }
    } catch (_) {}
    const K = [['hero', _HERO_MODEL_URI, [_HERO_HEADING_OFFSET, _HERO_PITCH_OFFSET, _HERO_ROLL_OFFSET]],
               ['pioneer', _PIONEER_MODEL_URI, [_PIONEER_HEADING_OFFSET, _PIONEER_PITCH_OFFSET, _PIONEER_ROLL_OFFSET]],
               ['vtol', _VTOL_MODEL_URI, [_VTOL_HEADING_OFFSET, _VTOL_PITCH_OFFSET, _VTOL_ROLL_OFFSET]],
               ['plane', _PLANE_MODEL_URI, [_PLANE_HEADING_OFFSET, _PLANE_PITCH_OFFSET, _PLANE_ROLL_OFFSET]],
               ['copter', _COPTER_MODEL_URI, [_COPTER_HEADING_OFFSET, _COPTER_PITCH_OFFSET, _COPTER_ROLL_OFFSET]]];
    for (let i = 0; i < K.length; i++) {
      if (uri && K[i][1] && uri === K[i][1]) return {kind: K[i][0], offs: A.pinned[K[i][0]], pageOffs: K[i][2], uri: uri};
    }
    return {kind: uri ? 'unknown' : 'none', offs: [0, 0, 0], pageOffs: [0, 0, 0], uri: uri};
  };
  A.axes = function (q, p, offs) {
    const fix = Cesium.Quaternion.fromHeadingPitchRoll(new Cesium.HeadingPitchRoll(
      Cesium.Math.toRadians(offs[0]), Cesium.Math.toRadians(offs[1]), Cesium.Math.toRadians(offs[2])));
    const body = Cesium.Quaternion.multiply(q, Cesium.Quaternion.inverse(fix, new Cesium.Quaternion()), new Cesium.Quaternion());
    const M = Cesium.Matrix3.fromQuaternion(body);
    const inv = Cesium.Matrix4.inverseTransformation(Cesium.Transforms.eastNorthUpToFixedFrame(p), new Cesium.Matrix4());
    const ax = function (x, y, z) {
      const w = Cesium.Matrix3.multiplyByVector(M, new Cesium.Cartesian3(x, y, z), new Cesium.Cartesian3());
      const v = Cesium.Matrix4.multiplyByPointAsVector(inv, w, new Cesium.Cartesian3());
      return [v.x, v.y, v.z];
    };
    return {nose: ax(0, 1, 0), right: ax(1, 0, 0), up: ax(0, 0, 1)};
  };
  A.model = function (k, e) {
    const c = A.models[k];
    if (c && c.id === e && !(c.isDestroyed && c.isDestroyed())) return c;
    const now = Date.now();
    if (now - (A.lastSearch[k] || 0) < 500) return null;
    A.lastSearch[k] = now;
    const walk = function (p, depth) {
      if (!p || depth > 6) return null;
      if (p.id === e && p.modelMatrix) return p;
      if (typeof p.length === 'number' && typeof p.get === 'function') {
        for (let i = 0; i < p.length; i++) { const r = walk(p.get(i), depth + 1); if (r) return r; }
      }
      return null;
    };
    const m = walk(viewer.scene.primitives, 0);
    if (m) A.models[k] = m;
    return m;
  };
  // 模型 primitive 的 modelMatrix → 去校正的 ENU 機體軸（欄向量正規化去掉 scale）；還沒放到地球上回 null
  A.drawnAxes = function (m, offs) {
    const pd = Cesium.Matrix4.getTranslation(m.modelMatrix, new Cesium.Cartesian3());
    if (Cesium.Cartesian3.magnitude(pd) <= 1e6) return null;
    const M3 = Cesium.Matrix4.getMatrix3(m.modelMatrix, new Cesium.Matrix3());
    for (let c = 0; c < 3; c++) {
      const col = Cesium.Matrix3.getColumn(M3, c, new Cesium.Cartesian3());
      Cesium.Cartesian3.normalize(col, col);
      Cesium.Matrix3.setColumn(M3, c, col, M3);
    }
    return A.axes(Cesium.Quaternion.fromRotationMatrix(M3, new Cesium.Quaternion()), pd, offs);
  };
  viewer.clock.onTick.addEventListener(function () {
    A.ticks++;
    const d = {};
    for (const k in _UAV_STATE) {
      const st = _UAV_STATE[k], e = uavEntities[k];
      let t0, q = null, p = null, disp, n = 0;
      do {      // 同一毫秒內取完（Date.now／JulianDate.now 都是毫秒粒度）
        t0 = Date.now();
        const jd = Cesium.JulianDate.now();
        disp = _uavDisplayMs(st);
        if (e) { q = e.orientation.getValue(jd); p = e.position.getValue(jd); }
        n++;
      } while (Date.now() !== t0 && n < 6);
      d[k] = disp;
      if (!e || !q || !p) continue;
      const kd = A.kind(e);
      const rec = {ms: t0, disp: disp, tick: A.ticks, delay: st.delay, lastAttMs: st.lastAttMs, kind: kd.kind,
                   ax: A.axes(q, p, kd.offs)};
      const m = A.model(k, e);
      if (m) { const dax = A.drawnAxes(m, kd.offs); if (dax) rec.dax = dax; }
      const ring = A.ring[k] || (A.ring[k] = []);
      ring.push(rec);
      if (ring.length > A.ringMax) { ring.shift(); A.dropped++; }
    }
    A.tick = {ms: Date.now(), disp: d, n: A.ticks};
  });
  return 'ok';
})();
"""


def install_js(pinned=None) -> str:
    """INSTALL_JS_TEMPLATE 換進校正偏移（預設 PINNED_OFFSETS）。"""
    pinned = PINNED_OFFSETS if pinned is None else pinned
    return INSTALL_JS_TEMPLATE.replace('/*PINNED*/null', json.dumps({k: list(v) for k, v in pinned.items()}))


INSTALL_JS = install_js()

# 每次讀回：每架的顯示時間、模型姿態（entity.orientation 就是 CallbackProperty → 取樣屬性在顯示時間的插值）、
# 去校正後的 ENU 機體軸、頁面狀態（JS 執行當下的 Date.now() 一起回：「現在」用它，不用 Python 收到回呼的時刻）；
# 加上從上次讀回到現在每一個算圖幀的紀錄（frames，環形緩衝整批取走）
READ_JS = r"""
(function () {
  const A = window.__afp;
  const out = {ms: Date.now(), ticks: A ? A.ticks : null,
               flag: (typeof _UAV_NOSE_FOLLOWS_TRACK === 'undefined') ? null : _UAV_NOSE_FOLLOWS_TRACK, uav: {}};
  if (!A || typeof uavEntities === 'undefined') return JSON.stringify(out);
  const ks = Object.keys(uavEntities);
  for (let i = 0; i < ks.length; i++) {
    const k = ks[i], e = uavEntities[k], st = _UAV_STATE[k];
    if (!e || !st) continue;
    let t0, q, p, disp, n = 0;
    do {      // 同一毫秒內取完（Date.now／JulianDate.now 都是毫秒粒度）
      t0 = Date.now();
      const jd = Cesium.JulianDate.now();
      q = e.orientation.getValue(jd); p = e.position.getValue(jd); disp = _uavDisplayMs(st); n++;
    } while (Date.now() !== t0 && n < 6);
    const kd = A.kind(e);
    const r = {disp: disp, ms: t0, sameMsTries: n, delay: st.delay, kind: kd.kind, offs: kd.offs, pageOffs: kd.pageOffs,
               nAtt: st.nAtt, nPos: st.nPos, lastAttMs: st.lastAttMs, lastPosMs: st.lastPosMs, yaw: st.yaw,
               nose: st.heading, crs: st.crs, pitch: st.pitch, roll: st.roll, speed: st.speed,
               frames: A.ring[k] ? A.ring[k].splice(0) : []};
    if (q && p) r.ax = A.axes(q, p, kd.offs);
    out.uav[k] = r;
  }
  out.ms = Date.now();
  return JSON.stringify(out);
})();
"""

PAGE_INFO_JS = r"""
JSON.stringify({cesium: Cesium.VERSION,
  flag: (typeof _UAV_NOSE_FOLLOWS_TRACK === 'undefined') ? null : _UAV_NOSE_FOLLOWS_TRACK,
  offsets: {hero: [_HERO_HEADING_OFFSET, _HERO_PITCH_OFFSET, _HERO_ROLL_OFFSET],
            pioneer: [_PIONEER_HEADING_OFFSET, _PIONEER_PITCH_OFFSET, _PIONEER_ROLL_OFFSET],
            vtol: [_VTOL_HEADING_OFFSET, _VTOL_PITCH_OFFSET, _VTOL_ROLL_OFFSET],
            plane: [_PLANE_HEADING_OFFSET, _PLANE_PITCH_OFFSET, _PLANE_ROLL_OFFSET],
            copter: [_COPTER_HEADING_OFFSET, _COPTER_PITCH_OFFSET, _COPTER_ROLL_OFFSET]},
  uris: {hero: _HERO_MODEL_URI, pioneer: _PIONEER_MODEL_URI, vtol: _VTOL_MODEL_URI, plane: _PLANE_MODEL_URI,
         copter: _COPTER_MODEL_URI},
  airframe: {dflt: _UAV_AIRFRAME_DEFAULT, map: _UAV_AIRFRAME},
  blend: [_UAV_NOSE_BLEND_LO, _UAV_NOSE_BLEND_HI],
  nAtt: Object.fromEntries(Object.keys(_UAV_STATE).map(function (k) { return [k, _UAV_STATE[k].nAtt]; })),
  nPos: Object.fromEntries(Object.keys(_UAV_STATE).map(function (k) { return [k, _UAV_STATE[k].nPos]; })),
  ticks: window.__afp ? window.__afp.ticks : null,
  framesDropped: window.__afp ? window.__afp.dropped : null});
"""

# 牙齒檢查用的突變（只改這個頁面實例，不動 repo 檔）
MUTATIONS = {
    # 2026-10-22 之前的寫法：校正角度直接加進姿態角 ⇒ VTOL／多旋翼校正 pOff = 90° 時滾轉變偏航
    'old-angle-addition': r"""
_uavModelOrientation = function (pos, noseDeg, pitchDeg, rollDeg, hOff, pOff, rOff, result) {
  return Cesium.Transforms.headingPitchRollQuaternion(pos, new Cesium.HeadingPitchRoll(
    Cesium.Math.toRadians((noseDeg || 0) + (hOff || 0)), Cesium.Math.toRadians((pitchDeg || 0) + (pOff || 0)),
    Cesium.Math.toRadians((rollDeg || 0) + (rOff || 0))), undefined, undefined, result);
}; 'mutated';
""",
    # 模型姿態不插值：顯示時間落在兩筆之間時一律畫前一筆（階梯）。讀回和樣本鎖相時，相位剛好在樣本上就量不到它
    'hold-previous': r"""
_uavSampledProps = (function (orig) {
  return function () {
    const p = orig(), ori = p.ori, get = ori.getValue.bind(ori);
    ori.getValue = function (t, r) {
      const ts = ori._times;
      if (t && ts && ts.length > 1) {
        let i = ts.length - 1;
        while (i > 0 && Cesium.JulianDate.greaterThan(ts[i], t)) i--;
        if (!Cesium.JulianDate.greaterThan(ts[i], t)) return get(ts[i], r);
      }
      return get(t, r);
    };
    return p;
  };
})(_uavSampledProps); 'mutated';
""",
}

NOSE_JS = {'on': '_UAV_NOSE_FOLLOWS_TRACK = true; "ok";', 'off': '_UAV_NOSE_FOLLOWS_TRACK = false; "ok";'}
NO_IMAGERY_JS = ("viewer.imageryLayers.removeAll(); viewer.scene.globe.baseColor = "
                 "Cesium.Color.fromCssColorString('#33475b'); if (typeof _requestRender === 'function') _requestRender();"
                 " 'ok';")


# ═══════════════════════════════════════════════════════════════════════
#  命令列
# ═══════════════════════════════════════════════════════════════════════
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description='3D 模型姿態忠實度量測（真 SITLLink → 真合幀 → 真 CesiumMapWidget）')
    ap.add_argument('--conn', required=True, help='SITLLink 連線字串，例如 tcp:127.0.0.1:5760、udpin:0.0.0.0:14550')
    ap.add_argument('--truth', default='', help='獨立真值鏈路（另一條 pymavlink 連線，例如 SITL 的 SERIAL1 tcp:127.0.0.1:5762）')
    ap.add_argument('--truth-hz', type=float, default=50.0, help='真值鏈路要求的 ATTITUDE 頻率')
    ap.add_argument('--truth-align-phase', default='on', choices=['on', 'off'],
                    help='真值鏈路的取樣相位對齊主鏈路（同間隔時；理由見 RawTruth）；off = 照舊（相位隨機）')
    ap.add_argument('--truth-phase-offset-ms', type=float, default=0.0,
                    help='對齊到主鏈路相位 + 這麼多 ms（例如 10 = 50 Hz 的半個間隔：量兩組錯相樣本彼此差多少）')
    ap.add_argument('--sysid', type=int, default=1, help='SITLLink 的 sysid_label')
    ap.add_argument('--vehicle', default='VTOL', choices=['VTOL', 'PLANE', 'COPTER', ''],
                    help='SITLLink 的 vehicle_hint（GUI 啟動 SITL 時傳的那個；QuadPlane 心跳分不出 VTOL）')
    ap.add_argument('--seconds', type=float, default=None,
                    help='錄多久（從第一次讀到這架開始算）；預設 60，--fly 時預設 900（落地上鎖就提早結束）')
    ap.add_argument('--period-ms', type=float, default=50.0, help='讀回頁面的平均週期（每一個算圖幀另外都記）')
    ap.add_argument('--period-jitter', type=float, default=0.4,
                    help='讀回週期在 period × (1 ± jitter) 之間均勻亂數（不和 20 Hz 樣本鎖相；0 = 固定週期）')
    ap.add_argument('--warmup', type=float, default=3.0,
                    help='每架第一次出現後幾秒不算（顯示延遲收斂；GLB 第一次載入會卡主執行緒 ~0.15 s，約在出現後 1–2 s）')
    ap.add_argument('--out', default='', help='輸出目錄（預設 runs/artifacts/ATTITUDE_FIDELITY_<時間>）')
    ap.add_argument('--shot-every', type=int, default=0, help='每 N 次讀回存一張畫面（0 = 不存）')
    ap.add_argument('--nose-follows-track', default='page-default', choices=['on', 'off', 'page-default'],
                    help='開始時設定頁面的 _UAV_NOSE_FOLLOWS_TRACK（量舊／新行為不用改程式）')
    ap.add_argument('--mutate', default='none', choices=['none', *MUTATIONS],
                    help='牙齒檢查：把頁面換成已知錯的寫法')
    ap.add_argument('--js', default='', help='就緒後額外在頁面執行的 JS')
    ap.add_argument('--telemetry-hz', type=float, default=None,
                    help='GUI 合幀頻率（AEROPLAN_TELEMETRY_HZ）；省略 = 跟 app 一樣')
    ap.add_argument('--window', default='offscreen', choices=['offscreen', 'visible'],
                    help='offscreen = WA_DontShowOnScreen（照常算圖、不出現在螢幕上）')
    ap.add_argument('--size', default='960x640')
    ap.add_argument('--no-chase', action='store_true', help='不開跟隨相機（截圖會看不到飛機）')
    ap.add_argument('--chase-distance', type=float, default=14.0)
    ap.add_argument('--allow-network', action='store_true', help='放行網路請求（衛星圖磚）；預設全擋')
    ap.add_argument('--connect-timeout', type=float, default=60.0, help='多久內沒有任何一架出現在頁面上就放棄')
    ap.add_argument('--fail-over', type=float, default=None,
                    help='閘門（要 --truth）：任何一種總旋轉角誤差 p95 超過這個度數、覆蓋率不夠、模型認不出來、'
                         '頁面校正 ≠ 寫死的 ⇒ 結束碼 4（逐項見 summary.json 的 gate）')
    ap.add_argument('--min-rows', type=int, default=50, help='閘門：讀回與每幀各至少幾列 ok')
    ap.add_argument('--min-ok-frac', type=float, default=0.9, help='閘門：暖機後 ok 列的比例下限（hold／none 算不過）')
    ap.add_argument('--max-delay-s', type=float, default=None, help='閘門（可選）：顯示延遲 p95 上限（秒）')
    ap.add_argument('--model-cache', default='',
                    help='GLB 合併快取目錄（預設跟 app 一樣 data/cache/models/）；AEROPLAN_GLB_MERGE=0 ⇒ 用原檔')
    # --fly
    ap.add_argument('--fly', action='store_true', help='自己飛一趟：上傳任務 → 真解鎖 → AUTO → 錄到落地上鎖')
    ap.add_argument('--fly-alt', type=float, default=60.0)
    ap.add_argument('--fly-side', type=float, default=600.0)
    ap.add_argument('--fly-ready-timeout', type=float, default=240.0, help='等 PreArm 就緒最多幾秒')
    ap.add_argument('--fly-min-boot-s', type=float, default=50.0,
                    help='飛控沒有回報 PreArm 狀態位元時，開機至少幾秒才解鎖（記憶：SITL 開機 ~50 s 才解得了鎖）')
    a = ap.parse_args(argv)
    if a.seconds is None:
        a.seconds = 900.0 if a.fly else 60.0
    if a.fly and a.vehicle == 'COPTER':
        ap.error('--fly 只支援 VTOL／PLANE')
    if a.fail_over is not None and not a.truth:
        ap.error('--fail-over 需要 --truth（閘門要有獨立真值：SITLLink 自己換算錯了，鏈路真值會跟著錯）')
    if not 0.0 <= a.period_jitter < 1.0:
        ap.error('--period-jitter 要在 0～1 之間')
    return a


def _prepare_env(args) -> list:
    """Qt／Chromium／pymavlink 的環境變數；必須在 import PyQt6／pymavlink 之前。回傳要印的提醒。"""
    notes = []
    os.environ['AEROPLAN_NO_FILE_LOG'] = '1'
    os.environ.setdefault('MAVLINK20', '1')
    if os.environ.get('QT_QPA_PLATFORM', '').lower() == 'offscreen':
        if sys.platform == 'win32':
            os.environ['QT_QPA_PLATFORM'] = 'windows'
            notes.append('QT_QPA_PLATFORM=offscreen 沒有 WebGL ⇒ 改成 windows（視窗仍以 WA_DontShowOnScreen 不顯示）')
        else:
            notes.append('⚠ QT_QPA_PLATFORM=offscreen：Cesium 可能沒有 WebGL')
    flags = os.environ.get('QTWEBENGINE_CHROMIUM_FLAGS', '')
    if APP_GPU_FLAGS not in flags:
        os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = (flags + ' ' + APP_GPU_FLAGS).strip()
    if args.telemetry_hz is not None:
        os.environ['AEROPLAN_TELEMETRY_HZ'] = repr(float(args.telemetry_hz))
    return notes


# ═══════════════════════════════════════════════════════════════════════
#  鏈路兩端的記錄器
# ═══════════════════════════════════════════════════════════════════════
class _TruthTap:
    """SITLLink.telemetry 的 DirectConnection 接收端（鏈路執行緒當下呼叫）：每幀快照照記（FRAME_FIELDS）。

    有 link 時，att_t 變了的那一幀（= 剛收到一則 ATTITUDE）另外記那一則的 time_boot_ms：SITLLink 在處理那則訊息的同一個
    呼叫裡送出 telemetry，pymavlink 的 messages['ATTITUDE'] 就是它（收訊時 post_message 先記快取才回傳）。保險起見
    sysid 與 roll／pitch／yaw 要逐值相同才用，對不上記 None（例如同一條鏈路有別架的 ATTITUDE 插進來）。"""

    def __init__(self, link=None):
        self.link = link
        self.frames: list = []
        self.latest: dict = {}
        self._att_t: dict = {}
        self.boot_ms: dict = {}                                 # sysid → [time_boot_ms]（有配對到的才記）

    def _boot_ms(self, f):
        try:
            m = self.link._mav.messages.get('ATTITUDE')
            if m is None or int(m.get_srcSystem()) != int(f.sysid):
                return None
            if max(abs(math.degrees(m.roll) - f.roll), abs(math.degrees(m.pitch) - f.pitch),
                   abs(math.degrees(m.yaw) - f.yaw)) > 1e-9:
                return None
            return int(m.time_boot_ms)
        except Exception:                                       # noqa: BLE001
            return None

    def on_frame(self, f):
        try:
            sid, att_t = int(f.sysid), float(f.att_t or 0.0)
            boot = None
            if att_t > 0 and att_t != self._att_t.get(sid) and self.link is not None:
                boot = self._boot_ms(f)
            self._att_t[sid] = att_t
            row = (time.time(), sid, str(f.mode), bool(f.armed), float(f.ground_speed),
                   str(getattr(f, 'vehicle_type', '') or ''), att_t, float(f.roll), float(f.pitch),
                   float(f.yaw), float(f.lat), float(f.lon), float(f.alt_rel), int(f.gps_fix),
                   float(f.vel_n), float(f.vel_e), boot, float(getattr(f, 'att_ts', 0.0) or 0.0))
        except Exception:                                       # noqa: BLE001 — 記錄器不能讓鏈路掛掉
            return
        self.frames.append(row)
        self.latest[row[1]] = f
        if boot is not None:
            self.boot_ms.setdefault(sid, []).append(boot)

    def recent_boot_ms(self, sid: int, n: int = 40) -> list:
        """這架最近 n 則 ATTITUDE 的 time_boot_ms（真值鏈路對齊取樣相位用；別的執行緒讀：只切片、不改）。"""
        return list(self.boot_ms.get(int(sid), [])[-n:])


def phase_offset_ms(own: list, ref: list):
    """兩串 time_boot_ms（同一個間隔）的取樣相位差（ms，0 ≤ d < 間隔）：(間隔, 相位差)；間隔對不上（差 > 1 ms）、
    樣本不夠回 None。間隔 = 相鄰差的中位數。"""
    def interval(v):
        d = sorted(b - a for a, b in zip(v, v[1:]) if b > a)
        return d[len(d) // 2] if d else None
    if len(own) < 5 or len(ref) < 5:
        return None
    io, ir = interval(own), interval(ref)
    if io is None or ir is None or abs(io - ir) > 1 or io < 2:
        return None
    return io, (own[-1] - ref[-1]) % io


class RawTruth(threading.Thread):
    """獨立真值：另一條 pymavlink 連線直接收 ATTITUDE（不經 SITLLink）。samples = [(t, sysid, boot_ms, r, p, y)]。

    align（可省）：回傳主鏈路最近幾則 ATTITUDE time_boot_ms 的函式 ⇒ 把真值的取樣**相位**對齊到主鏈路（2026-10-22）。
    為什麼：飛控每條串流的取樣時刻是要求那一刻定的（ArduPilot 的訊息桶 last_sent_ms = 收到要求時的 millis），兩條 50 Hz
    串流可能差半個間隔（SITL 迴圈 100 Hz ⇒ 不是同一刻就是差 10 ms）。通用 VTOL 懸停有 ~17 Hz、±3° 的俯仰振盪：
    50 Hz 取樣畫不出它，兩組錯開 10 ms 的樣本各自內插、彼此就差 ~1°（2026-10-22 真 SITL：懸停 p95 0.98°、最大 1.2°）——
    那是取樣率的極限，不是顯示的錯。對齊之後
    兩邊是同一組取樣時刻（值仍然各自從自己的連線收）⇒ 量的才是「畫面是不是飛控在那一刻回報的姿態」。
    做法：相位不同就 SET_MESSAGE_INTERVAL(ATTITUDE, −1) 再設回原間隔（ArduPilot 會開新的訊息桶、相位重抽），直到對上；
    對齊模式下**不**用 REQUEST_DATA_STREAM 拉 EXTRA1（同間隔的桶已存在時 ATTITUDE 會被放回那個桶、相位不會變）。
    align_log：[(t, 相位差 ms, 動作)]；aligned：最後一次檢查是否對齊。"""

    ALIGN_CHECK_S = 0.25        # 多久檢查一次相位
    ALIGN_RETRY_S = 0.6         # 兩次重抽至少隔多久（等新相位的樣本進來）
    ALIGN_MAX_TRIES = 30

    def __init__(self, conn: str, hz: float = 50.0, align=None, phase_offset_ms: float = 0.0):
        super().__init__(name='attitude-raw-truth', daemon=True)
        self.conn, self.hz = conn, float(hz)
        self.align = align
        self.phase_offset_ms = float(phase_offset_ms)     # 對齊到「主鏈路相位 + 這麼多 ms」（量錯相的取樣極限用）
        self.samples: list = []
        self.error = ''
        self.connected = False
        self.align_log: list = []
        self.align_tries = 0
        self.aligned = None
        self._stop_ev = threading.Event()

    def stop(self):
        self._stop_ev.set()

    def run(self):
        from pymavlink import mavutil
        try:
            m = mavutil.mavlink_connection(self.conn, source_system=255, autoreconnect=False, dialect='ardupilotmega')
        except Exception as e:                                  # noqa: BLE001
            self.error = f'真值鏈路連不上：{e}'
            return
        gcs = (mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        try:
            m.mav.heartbeat_send(*gcs)
            t_end = time.time() + 30.0
            hb = None
            while hb is None and not self._stop_ev.is_set() and time.time() < t_end:
                hb = m.wait_heartbeat(timeout=1.0)
                if hb is not None and hb.get_srcSystem() in (0, 255):
                    hb = None
            if hb is None:
                self.error = self.error or '真值鏈路 30 秒內沒有飛控心跳'
                return
            self.connected = True
            interval_us = 1e6 / max(1.0, self.hz)

            def set_interval(us):
                m.mav.command_long_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, us, 0, 0, 0, 0, 0)
            if self.align is None:
                m.mav.request_data_stream_send(m.target_system, m.target_component, 10, max(1, int(self.hz)), 1)
            set_interval(interval_us)
            last_hb = time.time()
            last_check = last_try = 0.0
            own: list = []
            while not self._stop_ev.is_set():
                now = time.time()
                if now - last_hb >= 1.0:
                    m.mav.heartbeat_send(*gcs)
                    last_hb = now
                if self.align is not None and now - last_check >= self.ALIGN_CHECK_S:
                    last_check = now
                    ph = phase_offset_ms(own[-40:], self.align())
                    if ph is not None:
                        iv, d = ph
                        e = (d - self.phase_offset_ms) % iv
                        self.aligned = min(e, iv - e) <= 1
                        if (not self.aligned and now - last_try >= self.ALIGN_RETRY_S
                                and self.align_tries < self.ALIGN_MAX_TRIES):
                            set_interval(-1)            # 停掉（訊息桶清空）再設回 ⇒ 新桶、相位重抽
                            set_interval(interval_us)
                            self.align_tries += 1
                            last_try = now
                            own.clear()                 # 舊相位的樣本不算
                            self.align_log.append((round(now, 3), d, 'retry'))
                        elif self.aligned and (not self.align_log or self.align_log[-1][2] != 'aligned'):
                            self.align_log.append((round(now, 3), d, 'aligned'))
                msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=0.3)
                if msg is None:
                    continue
                own.append(int(msg.time_boot_ms))
                if len(own) > 200:
                    del own[:100]
                self.samples.append((time.time(), int(msg.get_srcSystem()), int(msg.time_boot_ms),
                                     math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)))
        except Exception as e:                                  # noqa: BLE001
            self.error = f'真值鏈路錯誤：{e}'
        finally:
            try:
                m.close()
            except Exception:                                   # noqa: BLE001
                pass


#: MainWindow 遙測合幀用到的方法：全部是**真的**（未綁定函式綁到替身上），見 tests/test_telemetry_coalescing.py 同一做法
_BOUND_METHODS = ('_on_sitl_telemetry', '_tele_state', '_flush_telemetry', '_note_armed_transition', '_armed_seen',
                  '_hud_log_msg', '_log_dl_state', '_update_tactical_on_uav_move')


class _HudStub:
    def __init__(self, log):
        self._log = log

    def on_telemetry(self, f):
        pass

    def on_log_message(self, sysid, text, severity=6):
        self._log(f'[HUD] UAV-{sysid}: {text}')

    def auto_log_download_enabled(self):
        return False


class _MapRecorder:
    """map_widget 的位置：記下每一批再交給真的 CesiumMapWidget.update_uav_positions。"""

    def __init__(self, widget, sink: list):
        self.widget, self.sink = widget, sink

    def update_uav_positions(self, batch):
        now = time.time()
        for it in batch:
            self.sink.append((now, dict(it)))
        self.widget.update_uav_positions(batch)


def make_gui_stub(MainWindow, map_widget, links, log):
    """MainWindow 的替身：遙測合幀那幾個方法是真的，其他是樁（不碰 FOV／RCS／日誌下載）。"""
    class _GuiStub:          # 一般類別（不是 SimpleNamespace）⇒ PyQt 拿得到 weakref
        pass
    w = _GuiStub()
    w.sitl_hud = _HudStub(log)
    w.map_widget = map_widget
    w._fov_cone_enabled = False
    w._rcs_enabled = False
    w._sitl_links = list(links)
    w._map_photo_stats = None
    w._pending_releases = None
    w.LOG_DOWNLOAD_AUTO_DELAY_S = getattr(MainWindow, 'LOG_DOWNLOAD_AUTO_DELAY_S', 8.0)
    w.statusBar = lambda: types.SimpleNamespace(showMessage=lambda m, *a: log(f'[狀態列] {m}'))
    for name in _BOUND_METHODS:
        setattr(w, name, types.MethodType(getattr(MainWindow, name), w))
    w._uav_batch_item = MainWindow._uav_batch_item
    return w


def prearm_state(link):
    """飛控 SYS_STATUS 的 PreArm 位元：True／False；沒回報這個位元 ⇒ None。只讀 pymavlink 的最後一則訊息快取。"""
    try:
        from pymavlink import mavutil
        bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        ss = getattr(getattr(link, '_mav', None), 'messages', {}).get('SYS_STATUS')
        if ss is None or not (int(ss.onboard_control_sensors_present) & bit):
            return None
        return bool(int(ss.onboard_control_sensors_health) & bit)
    except Exception:                                           # noqa: BLE001
        return None


def boot_seconds(link):
    try:
        att = getattr(getattr(link, '_mav', None), 'messages', {}).get('ATTITUDE')
        return None if att is None else att.time_boot_ms / 1000.0
    except Exception:                                           # noqa: BLE001
        return None


# ═══════════════════════════════════════════════════════════════════════
#  主程式（Qt）
# ═══════════════════════════════════════════════════════════════════════
def bootstrap_modules(out: pathlib.Path, model_cache: str = ''):
    """先把全域設定換掉，**才** import 任何 ui／mission 模組；回傳用到的模組。

    ⚠ 必須是 main() 裡第一個碰專案模組的動作（2026-10-22 審查：以前先 `from ui.render_setup import …`，那會先跑
    ui/__init__ → ui.main_window，它在模組層就呼叫 get_settings ⇒ 已經讀了 config/settings.json，而且
    ui.main_window.settings 一直是那一份）。設定換成「指向 out 裡不存在的檔」的預設值；頁面暫存檔寫在 out/_page；
    不帶 ion token。model_cache 給了 ⇒ GLB 合併快取放那裡（app 是 data/cache/models/）。
    測試在子行程裡用稽核掛鉤擋 settings.json 的開檔、直接呼叫這個函式。"""
    import config.settings as cs
    cs._global_settings = cs.GlobalSettings(config_file=str(out / '_no_settings.json'))
    import utils.temp_cache as tc
    tc._TEMP_HTML_DIR = out / '_page'
    from ui.render_setup import configure_qt_presentation
    from ui.widgets import cesium_map_widget as cmw
    from ui.widgets import glb_merge
    cmw.CesiumMapWidget._resolve_token = lambda self: ''            # 不帶 ion token（也不讀環境變數）
    if model_cache:
        cache, orig = pathlib.Path(model_cache), glb_merge.optimized_model_path
        glb_merge.optimized_model_path = lambda src, _cache_dir, _orig=orig: _orig(src, cache)
    import ui.main_window as uimw
    from mission.sitl_link import SITLLink
    return types.SimpleNamespace(cs=cs, tc=tc, configure_qt_presentation=configure_qt_presentation, cmw=cmw,
                                 glb_merge=glb_merge, uimw=uimw, MainWindow=uimw.MainWindow, SITLLink=SITLLink)


def main(argv=None) -> int:
    args = parse_args(argv)
    out = (pathlib.Path(args.out) if args.out else ROOT / 'runs' / 'artifacts' / (
        'ATTITUDE_FIDELITY_' + time.strftime('%Y%m%d_%H%M%S'))).resolve()   # 相對路徑 ⇒ 頁面暫存檔的 file: URL 會壞
    out.mkdir(parents=True, exist_ok=True)
    notes = _prepare_env(args)

    import logging
    logging.disable(logging.INFO)
    mods = bootstrap_modules(out, args.model_cache)      # ← 第一個碰專案模組的動作（設定先換掉）
    from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSlot
    from PyQt6.QtWebEngineWidgets import QWebEngineView  # noqa: F401 — 必須在 QApplication 之前
    from PyQt6.QtWebEngineCore import QWebEngineProfile, QWebEngineUrlRequestInterceptor
    from PyQt6.QtWidgets import QApplication, QMainWindow
    QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
    mods.configure_qt_presentation()
    app = QApplication.instance() or QApplication([sys.argv[0]])
    cmw = mods.cmw
    console_fh = open(out / 'console.log', 'w', encoding='utf-8')

    def _console(self, level, message, line, source):               # 頁面 console：全部寫檔，只印錯誤（圖磚失敗除外）
        try:
            if not console_fh.closed:
                console_fh.write(f'{getattr(level, "name", level)}\t{message}\t{source}:{line}\n')
            if int(getattr(level, 'value', level)) >= 2 and 'image tile' not in str(message):
                print(f'[頁面] {message}（{pathlib.Path(str(source)).name}:{line}）', flush=True)
        except Exception:                                           # noqa: BLE001 — 虛擬函式裡不能丟例外
            pass
    cmw.CesiumPage.javaScriptConsoleMessage = _console
    uimw, MainWindow, SITLLink = mods.uimw, mods.MainWindow, mods.SITLLink

    status_lines: list = []

    def log(text):
        line = f'{time.strftime("%H:%M:%S")} {text}'
        status_lines.append(line)
        print(line, flush=True)

    for n in notes:
        log(n)

    class _Block(QWebEngineUrlRequestInterceptor):
        def __init__(self):
            super().__init__()
            self.blocked = 0

        def interceptRequest(self, info):
            if info.requestUrl().scheme() not in ('file', 'data', 'blob', 'qrc', 'about'):
                self.blocked += 1
                info.block(True)
    blocker = None
    if not args.allow_network:
        blocker = _Block()
        QWebEngineProfile.defaultProfile().setUrlRequestInterceptor(blocker)

    win = QMainWindow()
    widget = cmw.CesiumMapWidget()
    win.setCentralWidget(widget)
    w_, h_ = (int(x) for x in args.size.lower().split('x'))
    win.resize(w_, h_)
    win.setWindowTitle('attitude_fidelity_probe')
    if args.window == 'offscreen':
        win.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    win.show()

    batches: list = []
    reads: list = []
    page_info: dict = {}
    shots: list = []
    st = {'t0': time.time(), 'ready': False, 't_first': None, 'finishing': False, 'exit': 0, 'reason': '',
          'n_read': 0, 'pending': 0, 'chase': False, 'link_error': '', 'connected': False}
    link = SITLLink(args.conn, sysid_label=args.sysid, vehicle_hint=args.vehicle)
    tap = _TruthTap(link)                     # 帶 link ⇒ 每則 ATTITUDE 另外記 time_boot_ms（開機時間軸的真值）
    rng = random.Random()
    gui = make_gui_stub(MainWindow, _MapRecorder(widget, batches), [link], log)
    raw = (RawTruth(args.truth, args.truth_hz,
                    align=(lambda: tap.recent_boot_ms(args.sysid)) if args.truth_align_phase == 'on' else None,
                    phase_offset_ms=args.truth_phase_offset_ms)
           if args.truth else None)
    fly_log: list = []
    fly = {'state': 'off' if not args.fly else 'wait_link', 't_state': time.time(), 'attempts': 0, 'force': False,
           'uploaded': None, 'items': None, 'seen_auto_armed': False, 'landed_at': None}

    def js(code, cb=None):
        if cb is None:
            widget._page.runJavaScript(code)
        else:
            widget._page.runJavaScript(code, cb)

    class _Bridge(QObject):
        """鏈路信號 → 主執行緒（跟 GUI 一樣是 queued）。"""
        @pyqtSlot(object)
        def on_frame(self, f):
            gui._on_sitl_telemetry(f)

        @pyqtSlot(int, str)
        def on_status(self, sev, text):
            log(f'[SITL {sev}] {text}')
            if fly['state'] == 'arm' and ('FORCE ARM' in text or 'PreArm 擋住解鎖' in text):
                fly['force'] = True

        @pyqtSlot(str)
        def on_error(self, text):
            if st['finishing']:                       # 收尾時關 socket，接收執行緒會報一次錯：正常
                log(f'[SITL] 收尾中關閉連線：{text}')
                return
            st['link_error'] = text
            log(f'[SITL 錯誤] {text}')

        @pyqtSlot(str)
        def on_connected(self, conn):
            st['connected'] = True
            log(f'[SITL] 已連線 {conn}')

        @pyqtSlot(str)
        def on_disconnected(self, reason):
            log(f'[SITL] 斷線：{reason}')
            if not st['finishing']:
                finish(f'鏈路斷線：{reason}', 2 if not reads else 0)

        @pyqtSlot(int, bool, str)
        def on_uploaded(self, sysid, ok, msg):
            fly['uploaded'] = (bool(ok), msg)
            log(f'[任務寫入] UAV-{sysid} ok={ok}：{msg}')
    bridge = _Bridge()
    link.telemetry.connect(tap.on_frame, Qt.ConnectionType.DirectConnection)
    link.telemetry.connect(bridge.on_frame)
    link.status_text.connect(bridge.on_status)
    link.error.connect(bridge.on_error)
    link.connected.connect(bridge.on_connected)
    link.disconnected.connect(bridge.on_disconnected)
    link.mission_uploaded.connect(bridge.on_uploaded)

    timers = []

    def every(ms, fn):
        t = QTimer()
        t.setTimerType(Qt.TimerType.PreciseTimer)
        t.timeout.connect(fn)
        t.start(int(ms))
        timers.append(t)
        return t

    # ── 頁面就緒 ────────────────────────────────────────────────────
    def poll_ready():
        if st['ready'] or st['finishing']:
            return
        if time.time() - st['t0'] > 120:
            finish('頁面 120 s 內沒有就緒', 2)
            return
        js(READY_JS, lambda ok: on_ready() if ok and not st['ready'] else None)

    def on_ready():
        st['ready'] = True
        log(f'頁面就緒（{time.time() - st["t0"]:.1f} s）')
        js(INSTALL_JS)
        js(PAGE_INFO_JS, lambda v: page_info.update(json.loads(v)) if v else None)   # 先讀：頁面預設的旗標
        if not args.allow_network:
            js(NO_IMAGERY_JS)
        if args.nose_follows_track in NOSE_JS:
            js(NOSE_JS[args.nose_follows_track])
        if args.mutate != 'none':
            js(MUTATIONS[args.mutate], lambda v: log(f'突變 {args.mutate}：{v}'))
        if args.js:
            js(args.js + '\n;"ok";')
            # --js 可能改了機體（setUavAirframes）⇒ 重讀，閘門與 summary 看的是這一趟實際用的（2026-10-23 審查）
            js('JSON.stringify({dflt: _UAV_AIRFRAME_DEFAULT, map: _UAV_AIRFRAME})',
               lambda v: page_info.update(airframe_initial=page_info.get('airframe'), airframe=json.loads(v)) if v else None)
        link.start()
        if raw is not None:
            raw.start()
        schedule_read()
        every(250, check_end)
        if args.fly:
            every(250, fly_tick)

    # ── 讀回 ────────────────────────────────────────────────────────
    # 週期亂數（period × (1 ± jitter)）：固定 50 ms 會和 20 Hz 的樣本鎖相，一整趟只量到樣本間隔裡的同一個相位
    # （2026-10-22 審查）。每一個算圖幀另外由頁面的環形緩衝記，讀回時整批取走。
    def schedule_read():
        if st['finishing']:
            return
        lo, hi = args.period_ms * (1.0 - args.period_jitter), args.period_ms * (1.0 + args.period_jitter)
        QTimer.singleShot(max(1, int(round(rng.uniform(lo, hi)))), Qt.TimerType.PreciseTimer, sample)

    def sample():
        if st['finishing']:
            return
        schedule_read()
        if st['pending'] > 20:
            return
        st['pending'] += 1
        js(READ_JS, on_read)

    def on_read(v):
        st['pending'] -= 1
        t_read = time.time()
        if not v or st['finishing']:
            return
        try:
            data = json.loads(v)
        except ValueError:
            return
        if not data.get('uav'):
            return
        reads.append((t_read, data))
        st['n_read'] += 1
        if st['t_first'] is None:
            st['t_first'] = t_read
            log(f'第一次讀到飛機：sysid {sorted(data["uav"])}')
        if not st['chase'] and not args.no_chase:
            st['chase'] = True
            sid = int(sorted(data['uav'], key=int)[0])
            widget.set_chase_camera_params(distance=args.chase_distance, pitch_deg=-10.0)
            widget.set_chase_camera(sid)
        if args.shot_every > 0 and st['n_read'] % args.shot_every == 0:
            p = out / 'shots' / f'shot_{st["n_read"]:05d}.png'
            p.parent.mkdir(exist_ok=True)
            widget.capture_png(str(p), lambda ok, d, p=p, t=t_read: shots.append(
                {'t': round(t, 3), 'path': str(p) if ok else None, 'detail': d}))

    def check_end():
        if st['finishing']:
            return
        now = time.time()
        if fly['state'] == 'abort':
            finish('飛行中止：' + (fly_log[-1][2] if fly_log else ''), 3)
            return
        if st['t_first'] is None:
            if st['link_error'] and not st['connected']:
                finish(f'鏈路失敗：{st["link_error"]}', 2)
            elif now - st['t0'] > args.connect_timeout + 120:
                seen = sorted(tap.latest)
                gps = {s: int(getattr(tap.latest[s], 'gps_fix', 0)) for s in seen}
                finish(f'{args.connect_timeout:.0f} s 內頁面上沒有任何一架：鏈路看到的 sysid {seen}、GPS fix {gps}'
                       f'（GUI 只畫 sysid == --sysid {args.sysid} 且 GPS fix ≥ 3 的機）', 2)
            return
        if now - st['t_first'] >= args.seconds:
            finish(f'錄滿 {args.seconds:.0f} s')
        elif fly['state'] == 'done' and fly['landed_at'] and now - fly['landed_at'] > 3.0:
            finish('落地上鎖')

    # ── --fly：上傳 → 真解鎖 → AUTO → 等落地 ─────────────────────────
    def fly_to(state, note=''):
        fly['state'], fly['t_state'] = state, time.time()
        fly_log.append((round(time.time(), 3), state, note))
        log(f'[飛行] {state} {note}')

    def fly_tick():
        stt = fly['state']
        if stt in ('off', 'done', 'abort') or st['finishing']:
            return
        f = tap.latest.get(args.sysid)
        el = time.time() - fly['t_state']
        if stt == 'wait_link':
            if f is not None and int(f.gps_fix) >= 3 and abs(f.lat) > 1e-6:
                fly_to('wait_ready', f'GPS {f.gps_fix}，Home {f.lat:.7f},{f.lon:.7f}')
            elif el > args.connect_timeout:
                fly_to('abort', '沒有定位')
            return
        if f is None:
            return
        if stt == 'wait_ready':
            pa, boot = prearm_state(link), boot_seconds(link)
            ok = pa is True or (pa is None and boot is not None and boot >= args.fly_min_boot_s)
            if ok:
                fly['items'] = build_square_mission(f.lat, f.lon, args.fly_alt, args.fly_side, args.vehicle or 'VTOL')
                fly_to('upload', f'PreArm 位元 {pa}、開機 {boot}s；任務 {len(fly["items"])} 項')
                fly['uploaded'] = None
                link.upload_mission_items(fly['items'], verify=True)
            elif el > args.fly_ready_timeout:
                fly_to('abort', f'{args.fly_ready_timeout:.0f} s 內 PreArm 沒有就緒（位元 {pa}、開機 {boot}s）')
            return
        if stt == 'upload':
            if fly['uploaded'] is not None:
                ok, msg = fly['uploaded']
                if not ok:
                    fly_to('abort', f'任務寫入失敗：{msg}')
                elif (args.vehicle or 'VTOL') == 'VTOL':
                    link.set_mode('QLOITER')
                    fly_to('qmode')
                else:
                    fly_to('arm_start')
            elif el > 90:
                fly_to('abort', '任務寫入 90 s 沒有結果')
            return
        if stt == 'qmode':
            if f is not None and f.mode == 'QLOITER':
                fly_to('arm_start')
            elif el > 10:
                fly_to('abort', f'切不進 QLOITER（現在 {getattr(f, "mode", "?")}）')
            return
        if stt == 'arm_start':
            fly['attempts'] += 1
            fly['force'] = False
            link.arm()
            fly_to('arm', f'第 {fly["attempts"]} 次')
            return
        if stt == 'arm':
            if fly['force']:
                link.disarm()
                fly_to('disarming', 'PreArm 擋下、SITLLink 走了 FORCE ARM ⇒ 立刻上鎖（不在 force 下飛）')
            elif f is not None and f.armed and el > 1.0:
                link.set_mode('AUTO')
                fly_to('auto')
            elif el > 12:
                if fly['attempts'] >= 3:
                    fly_to('abort', '解鎖三次都沒成功')
                else:
                    fly_to('arm_wait', '解鎖逾時，5 s 後重試')
            return
        if stt == 'arm_wait':
            if el > 5:
                fly_to('arm_start')
            return
        if stt == 'disarming':                       # 等飛控真的回報上鎖才收（不然鏈路一關，上鎖指令可能還沒送出）
            fly['seen_armed'] = fly.get('seen_armed') or bool(f.armed)
            if not f.armed and (fly['seen_armed'] or el > 4.0):
                fly_to('abort', 'PreArm 擋下、SITLLink 走了 FORCE ARM ⇒ 已上鎖並中止（不在 force 下飛）')
            elif el > 12:
                fly_to('abort', '⚠ PreArm 擋下、SITLLink 走了 FORCE ARM；送了上鎖但 12 s 內沒看到上鎖 —— 請手動確認')
            elif int(el) in (4, 8) and el - int(el) < 0.25:
                link.disarm()
            return
        if stt == 'auto':
            if f is not None and f.mode == 'AUTO' and f.armed:
                fly['seen_auto_armed'] = True
                fly_to('flying')
            elif el > 10:
                # 已解鎖、還在地上（QLOITER 油門 0）⇒ 上鎖再中止；離地了就不碰（SITLLink 的上鎖是 force，空中上鎖＝摔機）
                on_ground = abs(float(getattr(f, 'alt_rel', 0.0) or 0.0)) < 1.0
                if f.armed and on_ground:
                    link.disarm()
                fly_to('abort', f'切不進 AUTO（現在 {getattr(f, "mode", "?")}）'
                       + ('；已送上鎖（還在地上）' if f.armed and on_ground else
                          ('；⚠ 已離地、沒有上鎖 —— 請接手' if f.armed else '')))
            return
        if stt == 'flying':
            if f is not None and not f.armed:
                fly['landed_at'] = time.time()
                fly_to('done', f'上鎖（{f.mode}）')

    # ── 收尾 ────────────────────────────────────────────────────────
    def finish(reason, code=0):
        if st['finishing']:
            return
        st['finishing'] = True
        st['reason'], st['exit'] = reason, code
        log(f'結束：{reason}')
        for t in timers:
            t.stop()
        js(PAGE_INFO_JS, lambda v: (page_info.update(final=json.loads(v)) if v else None,
                                    QTimer.singleShot(200, shutdown)))
        QTimer.singleShot(5000, shutdown)          # 頁面沒回也要收

    def shutdown():
        if st.get('shut'):
            return
        st['shut'] = True
        link.stop()
        link.wait(4000)
        if raw is not None:
            raw.stop()
            raw.join(timeout=3.0)
        try:
            write_results()
        except Exception as e:                                  # noqa: BLE001
            log(f'寫結果失敗：{e}')
            st['exit'] = st['exit'] or 5
        console_fh.close()
        win.close()
        QTimer.singleShot(300, app.quit)

    def write_results():
        frames = list(tap.frames)
        raw_s = list(raw.samples) if raw is not None else []
        boot = build_boot_truth(frames, raw_s, skip_first_s=args.warmup)
        rows = compute_rows(reads, frames, raw_s, warmup_s=args.warmup, boot=boot)
        write_csv(out / 'samples.csv', COLUMNS, rows)
        try:
            plotted = write_plot(rows, out / 'attitude_timeseries.png',
                                 f'姿態忠實度：機鼻跟航跡={args.nose_follows_track}'
                                 + (f'、突變={args.mutate}' if args.mutate != 'none' else ''))
        except Exception as e:                                  # noqa: BLE001 — 圖壞了不影響數據
            plotted = False
            log(f'畫圖失敗：{e}')
        write_csv(out / 'truth_link.csv', FRAME_FIELDS, frames)
        if raw is not None:
            write_csv(out / 'truth_raw.csv', ('t', 'sysid', 'time_boot_ms', 'roll', 'pitch', 'yaw'), raw_s)
        bkeys = ('t', 'sysid', 'hdg', 'crs', 'spd', 'pitch', 'roll', 'mode', 'vt', 'pt', 'at', 'n_atts', 'atts_ms',
                 'lat', 'lon', 'alt')
        write_csv(out / 'batches.csv', bkeys,
                  [{'t': round(t, 4), **it, 'n_atts': len(it.get('atts') or ()),
                    'atts_ms': ' '.join(str(int(round(a[0]))) for a in it.get('atts') or ())} for t, it in batches])
        (out / 'status.log').write_text('\n'.join(status_lines) + '\n', encoding='utf-8')
        n_att_truth = {}
        for fr in frames:
            n_att_truth.setdefault(fr[1], set()).add(fr[6])
        meta = {
            'tool': 'tools/attitude_fidelity_probe.py',
            'config': {k: v for k, v in vars(args).items()},
            'end_reason': st['reason'],
            'exit_code': st['exit'],
            'page': page_info,
            'calibration': calibration_check(page_info),
            'telemetry_flush_hz': uimw.TELEMETRY_FLUSH_HZ,
            'samples_in': {'truth_link_frames': len(frames),
                           'truth_link_attitude_samples': {str(k): len(v - {0.0}) for k, v in n_att_truth.items()},
                           'truth_link_with_boot_ms': sum(1 for fr in frames if fr[16] is not None),
                           'truth_raw_samples': len(raw_s), 'batches_items': len(batches), 'page_reads': len(reads),
                           'batches_extra_attitudes': sum(len(it.get('atts') or ()) for _t, it in batches),
                           'truth_link_with_att_ts': sum(1 for fr in frames if len(fr) > 17 and fr[17]),
                           'page_frames': sum(1 for r in rows if _is_frame(r))},
            'boot_clock': boot['stats'],
            'raw_truth': None if raw is None else {'conn': args.truth, 'connected': raw.connected, 'error': raw.error,
                                                   'link_vs_raw_total_deg': link_vs_raw(frames, raw_s),
                                                   'link_vs_raw_boot_deg': link_vs_raw_boot(frames, raw_s),
                                                   'align_phase': {'enabled': raw.align is not None,
                                                                   'offset_ms': raw.phase_offset_ms,
                                                                   'aligned_at_end': raw.aligned,
                                                                   'tries': raw.align_tries, 'log': raw.align_log,
                                                                   'common_boot': common_boot_fraction(
                                                                       frames, raw_s, args.warmup)}},
            'fly': None if not args.fly else {'final_state': fly['state'], 'log': fly_log,
                                              'mission': fly['items']},
            'network_blocked_requests': blocker.blocked if blocker is not None else None,
            'shots': shots,
            'plot': 'attitude_timeseries.png' if plotted else None,
            'notes': ['誤差 = 畫面 − 真值（度）；偏航由北順時針、俯仰機頭上為正、滾轉右翼下為正（ArduPilot 3-2-1）；'
                      'total = 兩個姿態間的總旋轉角（與歐拉分解無關）；|俯仰| > 85° 的列不進偏航／滾轉分軸統計',
                      'status=ok 的列才進統計；warmup／before／hold／no_render 只留在 CSV；row_kind：read = 讀回當下、'
                      'frame = 頁面每一個算圖幀（errors_per_frame_deg：每一個畫出來的幀都算，和讀回的相位無關）',
                      'page_att_age_s > 0 ＝ 頁面在 HOLD（顯示時間已超過它最新的姿態樣本：主執行緒卡住、批次晚到）；'
                      'page_not_holding 排除這些列 ⇒ 只剩畫法與插值誤差',
                      'drawn = 模型 primitive 那一幀的 modelMatrix（entity／模型變換；Cesium 再乘的 glTF 軸修正與'
                      '節點變換是固定的、GLB 幾何稽核已證實），在同一個顯示時間比真值',
                      '真值時間軸：errors_deg／errors_per_frame_deg 用頁面拿到的樣本時間（SITLLink 的 att_ts：飛控時間換到'
                      '牆鐘；沒有才是收到時刻）⇒ 只驗畫法＋插值＋合幀；errors_vs_boot_clock_deg／errors_vs_raw_boot_clock_deg'
                      '（含 _by_phase：hover < 3 m/s、transition 3–6、cruise ≥ 6）用飛控 time_boot_ms 經探針自己的下包絡換成'
                      '牆鐘（見 boot_clock）⇒ 成批到、排隊、時間戳接錯、對時估錯都會變成誤差；errors_vs_raw_truth_deg 用獨立'
                      '鏈路自己的收到時刻（0～32 ms 抖動，只報告、不進閘門）',
                      'raw_truth.align_phase：獨立真值的取樣相位對齊主鏈路（common_boot.frac ≈ 1 ⇒ 兩邊同一組飛控取樣時刻；'
                      '理由見 RawTruth：50 Hz 畫不出懸停的 ~17 Hz 振盪，錯相的兩組樣本彼此就差 ~1°）',
                      'raw_truth.link_vs_raw_boot_deg：SITLLink 解出來的姿態 vs 獨立鏈路同一個飛控時刻 ⇒ 只剩換算錯誤',
                      '校正：探針用 calibration.pinned（寫死）去掉校正，不用頁面常數；calibration.mismatch 非空 ⇒ 頁面常數被改了',
                      'errors_vs_wallclock_now_deg 拿 JS 執行當下（Date.now）的真值比（多了顯示延遲 display_delay_s 造成的'
                      '落後，快速滾轉時最明顯）'],
        }
        s = summarize(rows, meta)
        gate = evaluate_gate(s, args.fail_over if args.fail_over is not None else 0.5, need_raw=raw is not None,
                             min_rows=args.min_rows, min_ok_frac=args.min_ok_frac, max_delay_s=args.max_delay_s)
        gate['enforced'] = args.fail_over is not None
        s['gate'] = gate
        if gate['enforced'] and not gate['ok']:
            st['exit'] = st['exit'] or 4
            s['exit_code'] = st['exit']
        (out / 'summary.json').write_text(json.dumps(s, ensure_ascii=False, indent=1), encoding='utf-8')
        a = s['errors_deg']['all']
        fmt = lambda b: f"p50 {b['p50']} / p95 {b['p95']} / max {b['max']}"   # noqa: E731
        log(f'有效列 {a["n"]}（全部 {s["counts"]["rows"]}；每幀 {s["errors_per_frame_deg"]["all"]["n"]}）  '
            f'延遲 p50 {s["display_delay_s"]["p50"]} s')
        for ax in ('yaw', 'pitch', 'roll', 'total'):
            log(f'  {ax:<5} |誤差| 度：{fmt(a[ax])}' + (f"  偏差 {a[ax]['bias']}" if ax != 'total' else ''))
        nh = s['errors_deg']['page_not_holding']['total']
        log(f'  頁面 HOLD（批次晚到）的列 {s["page_hold_rows"]}；不含這些：total {fmt(nh)}')
        log(f'  每一個算圖幀：total {fmt(s["errors_per_frame_deg"]["all"]["total"])}；'
            f'drawn：total {fmt(s["errors_drawn_frame_deg"]["total"])}')
        log(f'  開機時間軸真值：total {fmt(s["errors_vs_boot_clock_deg"]["total"])}；'
            f'相位直方圖 讀回 {s["phase_histogram"]["reads"]} 每幀 {s["phase_histogram"]["frames"]}')
        log(f'  獨立真值（開機時間軸）：total {fmt(s["errors_vs_raw_boot_clock_deg"]["total"])}')
        for ph, blk in (s.get('errors_vs_raw_boot_clock_by_phase') or {}).items():
            log(f'    {ph:<10} n {blk["n"]}：total {fmt(blk["total"])}')
        if raw is not None:
            al = s['raw_truth']['align_phase']
            log(f'  真值相位對齊：{"開" if al["enabled"] else "關"}、重抽 {al["tries"]} 次、結束時對齊 {al["aligned_at_end"]}、'
                f'同一取樣時刻 {al["common_boot"]}')
        for sid, bst in (boot['stats']['link'] or {}).items():
            la = bst['latency_above_min_s']
            log(f'  UAV{sid} 收到時刻比下包絡晚：p95 {la["p95"]} s、max {la["max"]} s；壓扁的間隔 '
                f'{bst["compressed_gaps"]}/{bst["gaps"]}、成批 {bst["bursts"]} 次')
        lag = s['errors_vs_wallclock_now_deg']['total']
        log(f'  對牆鐘「現在」（含顯示延遲）：total {fmt(lag)}')
        bad = [c['name'] for c in gate['checks'] if not c['ok']]
        log(f'閘門（{gate["limit_deg"]}°{"" if gate["enforced"] else "，僅報告"}）：'
            + ('全過' if gate['ok'] else '不過 ' + '、'.join(bad)))
        log(f'輸出：{out}')

    every(250, poll_ready)
    app.exec()
    return int(st['exit'])


if __name__ == '__main__':
    sys.exit(main())
