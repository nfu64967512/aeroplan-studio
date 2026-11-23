# -*- coding: utf-8 -*-
"""boot_clock.py — 飛控開機時間（MAVLink time_boot_ms）→ 地面站牆鐘（epoch 秒）的線上對時。

為什麼（2026-10-22 姿態忠實度實測）：3D 模型的姿態樣本時間以前是「鏈路收到那則 ATTITUDE 的時刻」，
而 SITL（Windows）的訊息是成批送出的，收到時刻比飛控真正取樣的時刻晚 0～32 ms 不等（均勻分布、p50 16 ms）。
懸停時通用 VTOL 有 ~17 Hz、±3° 的俯仰振盪（角速度 p50 130°/s），16 ms 的時間誤差就是 2°。
飛控在每一則 ATTITUDE 都帶了自己的開機時間 ⇒ 換成牆鐘就沒有收到時刻的抖動。

方法：收到時刻 t 與開機時間 b 的**下包絡**（最快到的那幾則）就是「飛控送出 → 地面站收到」的最小延遲線，
排隊、成批只會把點往上推、不會拉低這條線。線上、因果地估：
  * 開機時間每 BLOCK_S 秒一個區塊，記區塊裡最快的一筆（b, t）；只留最近 WINDOW_S 秒的區塊。
  * 斜率：區塊最小值的下凸包在平均開機時間處的支撐線（與 tools/attitude_fidelity_probe.py 的事後擬合同一個定義，
    程式各寫各的）；跨度不到 MIN_FIT_SPAN_S 時假設 1。晶振漂移（真飛控 ±100 ppm）、SITL --speedup N（斜率 ≈ 1/N）
    都靠它吃掉。
  * 截距：給定斜率後，窗口內（含目前區塊）所有點的最小值 ⇒ 線壓在最快那一筆上。
  * 窗口要長：成批送出時「最快的那一筆」本身在幾十秒的尺度上會飄 ~1 ms，窗口愈長愈接近全程的下包絡。
    2026-10-22 用上一趟真 SITL 的 50 Hz 姿態離線比較（同一組樣本、線上 vs 全程事後擬合）：20 s 窗口＋斜率鎖 1 ⇒
    時間差 p95 1.3 ms、懸停姿態差 p95 0.25°；300 s＋擬合斜率 ⇒ 0.45 ms、0.07°；斜率鎖 1 在 −100 ppm 漂移、
    120 s 窗口時差到 12 ms。
  * 新的一筆比線還早到（更快）⇒ 線立刻往下移到它（之後的樣本跟著用新的下包絡）。
  * 開機時間倒退 > REBOOT_BACK_S（飛控重開機）⇒ 重新開始一段；比線晚到 > MAX_LATE_S（牆鐘往後跳、或鏈路塞了
    好幾秒）⇒ 也重新開始（之後先用收到時刻，幾個區塊後回到下包絡）。
回傳的時間一定 ≤ 收到時刻（線壓在包含這一筆在內的最小值上）。

純 Python、無 Qt；mission/sitl_link.py 每個來源 system 一個。
"""
from __future__ import annotations

import math
from collections import deque

#: 區塊長度（開機時間，秒）：每塊只留最快的一筆
BLOCK_S = 0.5
#: 窗口（開機時間，秒）：下包絡只看最近這麼久（見模組說明「窗口要長」；600 個區塊，重擬合 2 次／秒）
WINDOW_S = 300.0
#: 斜率擬合要的最短跨度（開機時間，秒）；不到就假設 1
MIN_FIT_SPAN_S = 10.0
#: 開機時間倒退超過這麼多秒 ⇒ 飛控重開機、重新開始一段
REBOOT_BACK_S = 1.0
#: 比下包絡晚到超過這麼多秒 ⇒ 重新開始（牆鐘跳動／長時間塞住）
MAX_LATE_S = 3.0


def lower_support_line(points):
    """點 [(x, y)] 的下凸包在 x 平均處的支撐線 (a, b)：y ≥ a + b·x 對每一點成立。點少於 2 個或 x 全相同回 None。"""
    best: dict = {}
    for x, y in points:
        if x not in best or y < best[x]:
            best[x] = y
    pts = sorted(best.items())
    if len(pts) < 2:
        return None
    hull: list = []
    for p in pts:                                        # Andrew 單調鏈的下半部
        while len(hull) >= 2 and ((hull[-1][0] - hull[-2][0]) * (p[1] - hull[-2][1])
                                  - (hull[-1][1] - hull[-2][1]) * (p[0] - hull[-2][0])) <= 0:
            hull.pop()
        hull.append(p)
    xbar = sum(x for x, _ in points) / len(points)
    edge = (hull[0], hull[1])
    for e in zip(hull, hull[1:]):
        if e[0][0] <= xbar <= e[1][0]:
            edge = e
            break
    (x0, y0), (x1, y1) = edge
    b = (y1 - y0) / (x1 - x0)
    return y0 - b * x0, b


class BootClockSync:
    """一個來源 system 的開機時間 → 牆鐘對時。``map(boot_ms, t_recv)`` 回這一筆在牆鐘上的取樣時刻（epoch 秒）。"""

    def __init__(self, *, block_s: float = BLOCK_S, window_s: float = WINDOW_S,
                 min_fit_span_s: float = MIN_FIT_SPAN_S, reboot_back_s: float = REBOOT_BACK_S, max_late_s: float = MAX_LATE_S):
        self.block_s, self.window_s = float(block_s), float(window_s)
        self.min_fit_span_s = float(min_fit_span_s)
        self.reboot_back_s, self.max_late_s = float(reboot_back_s), float(max_late_s)
        self.segments = 0            # 開過幾段（第一筆算 1；重開機／跳動各 +1）
        self.reboots = 0
        self.late_resets = 0
        self._reset()

    def _reset(self) -> None:
        self._blocks: deque = deque()     # [(boot_s, t)]：已結束的區塊各自最快的一筆
        self._cur = None                  # 目前區塊 [起點 boot_s, boot_s, t]
        self._a = None                    # 線 t = a + b·boot
        self._b = 1.0
        self._last_boot = None
        self.segments += 1

    @property
    def slope(self) -> float:
        return self._b

    def _refit(self) -> None:
        pts = list(self._blocks)
        if self._cur is not None:
            pts.append((self._cur[1], self._cur[2]))
        if not pts:
            return
        b = 1.0
        if len(pts) >= 2 and pts[-1][0] - pts[0][0] >= self.min_fit_span_s:
            fit = lower_support_line(pts)
            if fit is not None and math.isfinite(fit[1]) and fit[1] > 0:
                b = fit[1]
        self._b = b
        self._a = min(t - b * x for x, t in pts)

    def map(self, boot_ms, t_recv: float) -> float:
        boot = float(boot_ms) / 1000.0
        t = float(t_recv)
        if not (math.isfinite(boot) and math.isfinite(t)):
            return t
        if self._last_boot is not None and boot < self._last_boot - self.reboot_back_s:
            self.reboots += 1
            self._reset()
        self._last_boot = boot
        if self._a is None:
            self._a, self._b = t - boot, 1.0
        pred = self._a + self._b * boot
        if t - pred > self.max_late_s:
            self.late_resets += 1
            self._reset()
            self._last_boot = boot
            self._a, self._b = t - boot, 1.0
            pred = t
        if t < pred:                                     # 比下包絡還早到 ⇒ 線往下移
            self._a -= pred - t
            pred = t
        # 區塊：目前這塊最快的一筆；跨到下一塊 ⇒ 收進窗口、丟掉太舊的、重擬合
        if self._cur is None:
            self._cur = [boot, boot, t]
        elif boot >= self._cur[0] + self.block_s:
            self._blocks.append((self._cur[1], self._cur[2]))
            while self._blocks and self._blocks[0][0] < boot - self.window_s:
                self._blocks.popleft()
            self._cur = [boot, boot, t]
            self._refit()
            pred = min(t, self._a + self._b * boot)
        elif t - self._b * boot < self._cur[2] - self._b * self._cur[1]:
            self._cur[1], self._cur[2] = boot, t
        return pred
