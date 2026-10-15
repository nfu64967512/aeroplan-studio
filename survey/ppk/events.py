"""survey/ppk/events.py — TIM-TM2 時間標記 → GPST，並與樹莓派記到的同一個觸發邊緣逐一配對。

時間換算（F3）：GPST 秒 = wn·604800 + towMs/1e3 + towSubMs/1e9（無閏秒）；UTC = GPST − 18 s。
TIM-TM2 的 timeBase（flags bits4..3，UBX-22008968 §3.19.1 p.194–195）：
- 1 = GNSS 時間，**是哪一個 GNSS 由 CFG-TP-TIMEGRID_TP1 決定**，訊息本身看不出來
  ⇒ 呼叫端必須宣告（只支援 "GPS"；Galileo 週數差 1024、BDS 差 14 s 與 1356 週，不在這裡猜）。
- 2 = UTC。⚠ F9P 出廠預設 CFG-TP-TIMEGRID_TP1 = 0 (UTC)（同文件 Configuration defaults，
  Table 105 p.284），所以沒改設定時 TM2 報的是 UTC ⇒ 加 GPS_UTC_LEAP_S 換回 GPST，並標 ``tm2_utc_base``。
  RTKLIB demo5 ublox.c decode_timtm2 也是同樣處理（timeBase==2 ⇒ utc2gpst）。
- 0 = 接收機時間 ⇒ 無法換算，該邊緣列為無效。
另外檢查 bit6 time（Valid GNSS fix），UTC 基準時再要求 bit5 utc available。

兩道數值鎖（防止整批照片靜默偏 18 s ＝ 20 m/s 下 360 m）：
1. 有 RXM-RAWX 且 recStat.leapSec=1 時，接收機回報的 leapS 必須等於 ``leap_s``，否則丟錯；
2. 有 NAV-PVT 時，每個邊緣的 GPST 週內秒必須落在串流位置最近的 NAV-PVT iTOW（GPS 時間）
   ± 2.5 個量測週期內（10 Hz ⇒ 0.25 s），否則該邊緣列為無效（pvt_mismatch）。

遺失邊緣：整合手冊 UBX-18010802 §3.13.2（p.67–68）「Only the last rising and falling edge detected
between two epochs is reported」⇒ 兩個邊緣落在同一個 epoch 時前一個沒有時間。TM2 的 count 是上升邊緣
累計數（U2，65536 繞回），相鄰兩則有新邊緣的訊息 count 差 d > 1 ⇒ 遺失 d−1 個，逐段列出。

配對（never pair blindly by order）：
1. 樹莓派 seq 與 TM2 count 數的是同一批實體邊緣，所以真配對滿足 count − seq = k（常數，直到某一側漏掉
   一個邊緣）。對每個可能的 k 取出配對，計算「相鄰配對的時間差 GPST − Pi 時鐘」是否只隨時鐘頻差緩慢
   變化（|Δr| ≤ pair_tol + 500 ppm·Δt）；**最長一致鏈**的 k 勝出。分數相近（差 ≤2 步）的另一個 k
   若落在不同的時鐘線上即報錯（觸發間隔幾乎完全週期性時，錯位一張也對得上 ⇒ 等於按順序配對）。
2. 用勝出 k 的鏈做最小平方 t_gpst = a·t_pi + b，再對**所有**邊緣依模型找最近的 Pi 邊緣，
   殘差 ≤ pair_tol 才接受（容差遠小於最短拍照間隔，所以唯一）。某一側中途漏邊緣造成的 k 位移
   在這一步以時間接回來，並標 ``counter_shift``。配對與擬合交替三輪後輸出殘差統計。
   實測：5100 張、30 條帶、20 ms 量化觸發、Pi 與 TM2 各漏數十個邊緣，配對 0.74 s。
3. 已配對的 Pi 邊緣直接用 TM2 的 GPST（ns 級），不用擬合值；沒配到 TM2 的 Pi 邊緣才用擬合值並標
   ``tm2_missing``（及超出配對範圍時的 ``fit_extrapolated``）。

曝光中心：t_exp = t_edge_gpst + a·offset，offset 由呼叫端（ingest）依 Pi 會期算好傳入：
(sensor_ts − edge) + exposure/2 + readout/2，單位秒、Pi 時鐘。沒有傳 ⇒ 用邊緣時刻並標 ``edge_time_only``。

⚠ 邊緣極性：Pi 記的是哪一個邊緣、TM2 就要取同一個（``edge="rising"|"falling"``）。取錯會讓
**所有**時間差一個脈衝寬（CAM1_DURATION 0.1 s ⇒ 2 m），而且擬合看不出來（只是常數偏移）。
"""

from __future__ import annotations

import bisect
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from survey.ppk.ubx import NavPvt, RawxHeader, TimTm2, UbxLog

__all__ = [
    "GPS_WEEK_S",
    "GPS_UTC_LEAP_S",
    "TM2_COUNT_MOD",
    "MAX_CLOCK_RATE_PPM",
    "PairingError",
    "Tm2Edge",
    "LostEdges",
    "EdgeExtraction",
    "PairedEvent",
    "ClockFit",
    "PairingResult",
    "tm2_time_gpst",
    "extract_edges",
    "read_frames_jsonl",
    "pair_with_frames",
]

_log = logging.getLogger(__name__)

GPS_WEEK_S = 604800
GPS_UTC_LEAP_S = 18          # GPST − UTC，2017-01-01 起（IERS Bulletin C）；之後未再插閏秒
TM2_COUNT_MOD = 65536        # TIM-TM2 count 是 U2
MAX_CLOCK_RATE_PPM = 500.0   # Pi 晶振對 GPS 的頻差上限（實際約 ±50 ppm，留 10 倍餘裕）


class PairingError(RuntimeError):
    """TM2 與 Pi 邊緣無法可靠配對（不是可以靜默略過的狀況）。"""


@dataclass(frozen=True)
class Tm2Edge:
    count: int                 # 解開繞回後的上升邊緣累計數
    t_gpst_s: float
    acc_est_ns: int
    time_base: int
    offset: int                # 在 .ubx 串流中的位置
    flags: tuple[str, ...] = ()


@dataclass(frozen=True)
class LostEdges:
    count_before: int
    count_after: int
    n_lost: int
    t_before_gpst_s: float | None
    t_after_gpst_s: float | None


@dataclass
class EdgeExtraction:
    edges: list[Tm2Edge]
    lost: list[LostEdges]
    invalid: list[tuple[TimTm2, str]]
    duplicates: list[TimTm2]
    other_channel: int
    edge: str
    gnss_time_base_system: str
    leap_s: int
    pvt_checked: bool
    pvt_tol_s: float | None
    rawx_leap_s: int | None    # RAWX 回報（且 leapSec 有效）的閏秒；沒有 RAWX 時 None

    @property
    def n_lost(self) -> int:
        return sum(x.n_lost for x in self.lost)


def tm2_time_gpst(msg: TimTm2, edge: str = "rising", *, gnss_time_base_system: str = "GPS",
                  leap_s: int = GPS_UTC_LEAP_S) -> float:
    """單一 TIM-TM2 的指定邊緣 → GPST 秒。timeBase 為接收機時間時丟 ValueError。"""
    if edge == "rising":
        wn, ms, sub = msg.wn_r, msg.tow_ms_r, msg.tow_sub_ms_r_ns
    elif edge == "falling":
        wn, ms, sub = msg.wn_f, msg.tow_ms_f, msg.tow_sub_ms_f_ns
    else:
        raise ValueError(f"edge 只能是 'rising' 或 'falling'，收到 {edge!r}")
    t = wn * GPS_WEEK_S + ms / 1e3 + sub / 1e9
    tb = msg.time_base
    if tb == 1:
        if gnss_time_base_system != "GPS":
            raise ValueError(
                f"TIM-TM2 timeBase=GNSS，宣告的系統是 {gnss_time_base_system!r} —— 只支援 GPS"
                "（請把 CFG-TP-TIMEGRID_TP1 設為 GPS 或 UTC）")
        return t
    if tb == 2:
        return t + leap_s
    raise ValueError(f"TIM-TM2 timeBase={tb}（接收機時間或保留值）無法換成 GPST")


def _tow_diff(t_gpst_s: float, itow_ms: int) -> float:
    """邊緣的週內秒 − NAV-PVT iTOW，繞回到 [−302400, 302400)。"""
    return ((t_gpst_s % GPS_WEEK_S) - itow_ms / 1e3 + GPS_WEEK_S / 2) % GPS_WEEK_S - GPS_WEEK_S / 2


def _pvt_tol(pvt: Sequence[NavPvt]) -> float | None:
    if len(pvt) < 2:
        return None
    d = np.diff(np.array([p.itow_ms for p in pvt], dtype=np.int64)) % (GPS_WEEK_S * 1000)
    d = d[d > 0]
    if d.size == 0:
        return None
    return 2.5 * float(np.median(d)) / 1e3


def extract_edges(
    source: UbxLog | Sequence[TimTm2],
    *,
    pvt: Sequence[NavPvt] | None = None,
    rawx: Sequence[RawxHeader] | None = None,
    edge: str = "rising",
    channel: int = 0,
    gnss_time_base_system: str = "GPS",
    leap_s: int = GPS_UTC_LEAP_S,
    pvt_tol_s: float | None = None,
) -> EdgeExtraction:
    """把 TIM-TM2 訊息整理成有效邊緣、遺失段、無效邊緣（附原因）。

    ``source`` 給 UbxLog 時，pvt/rawx 沒指定就用 log 裡的（自動做兩道數值鎖）。
    """
    if edge not in ("rising", "falling"):
        raise ValueError(f"edge 只能是 'rising' 或 'falling'，收到 {edge!r}")
    if isinstance(source, UbxLog):
        msgs = list(source.tm2)
        pvt = source.pvt if pvt is None else pvt
        rawx = source.rawx if rawx is None else rawx
    else:
        msgs = list(source)
    pvt = list(pvt or [])
    rawx = list(rawx or [])

    msgs.sort(key=lambda m: m.offset)
    on_ch = [m for m in msgs if m.ch == channel]
    other = len(msgs) - len(on_ch)

    leaps = sorted({r.leap_s for r in rawx if r.leap_valid})
    rawx_leap = leaps[0] if len(leaps) == 1 else None
    # 閏秒只影響 UTC 基準的 TM2；GPS 基準時不擋（但仍回報 rawx_leap_s）
    if leaps and leaps != [leap_s] and any(m.time_base == 2 for m in on_ch):
        raise ValueError(
            f"RXM-RAWX 回報 GPS−UTC 閏秒 {leaps}，與設定 leap_s={leap_s} 不符 —— "
            "UTC 基準的 TIM-TM2 會整批偏移，請確認閏秒常數")

    # count 依串流順序解開 65536 繞回
    unwrapped: list[int] = []
    for m in on_ch:
        if not unwrapped:
            unwrapped.append(m.count)
        else:
            prev = unwrapped[-1]
            unwrapped.append(prev + ((m.count - prev) % TM2_COUNT_MOD))

    tol = pvt_tol_s if pvt_tol_s is not None else _pvt_tol(pvt)
    pvt_off = [p.offset for p in pvt]

    edges: list[Tm2Edge] = []
    invalid: list[tuple[TimTm2, str]] = []
    dups: list[TimTm2] = []
    lost: list[LostEdges] = []
    prev_c: int | None = None
    prev_t: float | None = None
    for m, c in zip(on_ch, unwrapped):
        if not (m.new_rising_edge if edge == "rising" else m.new_falling_edge):
            continue
        if prev_c is not None and c == prev_c:
            dups.append(m)
            continue
        t: float | None = None
        reason = None
        if not m.time_valid:
            reason = "time_not_valid"
        elif m.time_base == 2 and not m.utc_available:
            reason = "utc_not_available"
        elif m.time_base not in (1, 2):
            reason = f"time_base_{m.time_base}"
        else:
            t = tm2_time_gpst(m, edge, gnss_time_base_system=gnss_time_base_system, leap_s=leap_s)
            if tol is not None and pvt:
                # 串流中緊鄰這則 TM2 的前、後兩則 NAV-PVT（同一 epoch 內的輸出順序不固定）
                k = bisect.bisect_left(pvt_off, m.offset)
                d = min((_tow_diff(t, pvt[j].itow_ms) for j in (k - 1, k) if 0 <= j < len(pvt)),
                        key=abs)
                if abs(d) > tol:
                    reason = f"pvt_mismatch({d:+.3f}s)"
                    t = None
        if prev_c is not None and c - prev_c > 1:
            lost.append(LostEdges(prev_c, c, c - prev_c - 1, prev_t, t))
        prev_c, prev_t = c, t
        if reason is not None:
            invalid.append((m, reason))
            continue
        f = ("tm2_utc_base",) if m.time_base == 2 else ()
        edges.append(Tm2Edge(c, float(t), m.acc_est_ns, m.time_base, m.offset, f))

    if not pvt:
        _log.warning("TIM-TM2 沒有 NAV-PVT 可交叉核對時間基準（閏秒／GNSS 系統錯誤將無法偵測）")
    return EdgeExtraction(
        edges=edges, lost=lost, invalid=invalid, duplicates=dups, other_channel=other,
        edge=edge, gnss_time_base_system=gnss_time_base_system, leap_s=leap_s,
        pvt_checked=bool(pvt) and tol is not None, pvt_tol_s=tol, rawx_leap_s=rawx_leap,
    )


# ---------------------------------------------------------------------------
# Pi 端
# ---------------------------------------------------------------------------
def read_frames_jsonl(path: str | Path) -> tuple[list[dict], list[int]]:
    """讀 F1 的 frames.jsonl。斷電時最後一行可能寫一半 ⇒ 只容許**最後一行**壞掉，並回報行號。"""
    rows: list[dict] = []
    bad: list[int] = []
    # 斷電可能把尾端切在多位元組字元中間（Pi 端 json.dumps(ensure_ascii=False)）：用 replace 解碼，
    # 壞掉的那一行交給下面的 JSON 判斷（與 survey.pi_session 相同；嚴格解碼會整檔 UnicodeDecodeError）
    lines = Path(path).read_bytes().decode("utf-8", errors="replace").splitlines()
    for ln, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            if ln == len(lines):
                bad.append(ln)
                continue
            raise ValueError(f"{path} 第 {ln} 行不是合法 JSON（不是結尾截斷）") from None
    return rows, bad


@dataclass(frozen=True)
class PairedEvent:
    """每個 Pi 觸發邊緣一筆（依 seq 排序）。"""
    seq: int
    edge_ns: int
    tm2_count: int | None          # None ⇒ 沒有對應的 TM2，時間來自擬合
    t_edge_gpst_s: float
    t_exposure_gpst_s: float | None
    residual_s: float | None       # TM2 時間 − 擬合預測（只有配對成功的才有）
    acc_est_ns: int | None
    flags: tuple[str, ...]
    file: str | None = None
    status: str | None = None


@dataclass(frozen=True)
class ClockFit:
    """t_gpst = slope·(edge_ns − x0_ns)·1e-9 + offset_s"""
    slope: float
    offset_s: float
    x0_ns: int
    counter_offset: int            # 主要的 count − seq
    n_pairs: int
    rms_s: float
    max_abs_s: float

    def predict(self, edge_ns) -> np.ndarray:
        x = (np.asarray(edge_ns, dtype=np.int64) - self.x0_ns) * 1e-9
        return self.slope * x + self.offset_s

    @property
    def rate_ppm(self) -> float:
        return (self.slope - 1.0) * 1e6


@dataclass
class PairingResult:
    events: list[PairedEvent]
    fit: ClockFit
    unmatched_tm2: list[Tm2Edge]
    unmatched_pi_seq: list[int]
    counter_shifts: list[tuple[int, int]] = field(default_factory=list)  # (自哪個 seq 起, 該段的 count−seq)
    pair_tol_s: float = 0.0
    lost: list[LostEdges] = field(default_factory=list)

    @property
    def n_paired(self) -> int:
        return sum(1 for e in self.events if e.tm2_count is not None)


def _lsq(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    xm, ym = x.mean(), y.mean()
    dx = x - xm
    den = float(np.dot(dx, dx))
    a = float(np.dot(dx, y - ym)) / den if den > 0 else 1.0
    return a, float(ym - a * xm)


def pair_with_frames(
    edges: EdgeExtraction | Sequence[Tm2Edge],
    frames: Sequence[Mapping],
    *,
    exposure_offset_s: Sequence[float | None] | None = None,
    pair_tol_s: float = 0.002,
    min_pairs: int = 3,
) -> PairingResult:
    """把 TM2 邊緣與 Pi frames（F1 frames.jsonl 的列：至少有 seq、edge_ns）配對。

    exposure_offset_s 與 ``frames`` 同順序、同長度（秒，Pi 時鐘；None/NaN ⇒ 該張沒有曝光時刻）。
    """
    lost: list[LostEdges] = []
    if isinstance(edges, EdgeExtraction):
        lost = list(edges.lost)
        edges = edges.edges
    tm2 = sorted(edges, key=lambda e: e.count)
    if not (pair_tol_s > 0):
        raise ValueError(f"pair_tol_s 必須 > 0，收到 {pair_tol_s!r}")
    if exposure_offset_s is not None and len(exposure_offset_s) != len(frames):
        raise ValueError(f"exposure_offset_s 長度 {len(exposure_offset_s)} 與 frames {len(frames)} 不符")

    order = sorted(range(len(frames)), key=lambda i: int(frames[i]["seq"]))
    fr = [frames[i] for i in order]
    offs = None if exposure_offset_s is None else [exposure_offset_s[i] for i in order]
    S = np.array([int(f["seq"]) for f in fr], dtype=np.int64)
    if S.size and np.any(np.diff(S) == 0):
        raise ValueError("frames 的 seq 有重複")
    E = np.array([int(f["edge_ns"]) for f in fr], dtype=np.int64)
    if E.size > 1 and np.any(np.diff(E) <= 0):
        raise ValueError("frames 的 edge_ns 沒有隨 seq 嚴格遞增（時鐘不是 CLOCK_MONOTONIC？）")
    C = np.array([e.count for e in tm2], dtype=np.int64)
    if C.size > 1 and np.any(np.diff(C) == 0):
        raise ValueError("TM2 邊緣的 count 有重複")
    if len(tm2) < min_pairs or len(fr) < min_pairs:
        raise PairingError(f"邊緣太少：TM2 {len(tm2)}、Pi {len(fr)}（至少要 {min_pairs}）")

    x0 = int(E[0])
    X = (E - x0) * 1e-9
    y0 = tm2[0].t_gpst_s
    Y = np.array([e.t_gpst_s for e in tm2]) - y0
    rate = MAX_CLOCK_RATE_PPM * 1e-6

    # ---- 1. 依 count − seq 的假設 k 打分 ----
    cmin = int(C.min())
    lut = np.full(int(C.max()) - cmin + 1, -1, dtype=np.int64)
    lut[C - cmin] = np.arange(C.size)

    def chain_for(k: int) -> tuple[np.ndarray, np.ndarray, int]:
        """假設 count − seq = k，回傳最長一串「相鄰配對時間一致」的配對 (tm2 索引, Pi 索引, 步數)。

        用最長連續鏈而不是一致步數總和：錯位的 k 在 20 ms 量化的觸發間隔下也會有大量零星一致的步
        （實測約 6 成），但它們在每次間隔改變、每次轉彎處斷開；真 k 的鏈橫跨整段（漏掉的 TM2 只是
        跳過，不會斷）。鏈內的點才拿來做初始擬合，避免另一段錯位的點把直線拉歪。
        """
        idx = S + k - cmin
        j = np.nonzero((idx >= 0) & (idx < lut.size))[0]
        i = lut[idx[j]]
        sel = i >= 0
        i, j = i[sel], j[sel]
        if i.size < 2:
            return i[:0], j[:0], 0
        step_ok = np.abs(np.diff(Y[i] - X[j])) <= pair_tol_s + rate * np.diff(X[j])
        d = np.diff(np.concatenate(([0], step_ok.astype(np.int8), [0])))
        starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]   # 一致步 [start, end)
        if starts.size == 0:
            return i[:0], j[:0], 0
        m = int(np.argmax(ends - starts))
        s0, e0 = int(starts[m]), int(ends[m])
        return i[s0:e0 + 1], j[s0:e0 + 1], e0 - s0       # 步 [s0, e0) 連起配對 s0..e0

    scored: list[tuple[int, int]] = []
    for k in range(cmin - int(S.max()), int(C.max()) - int(S.min()) + 1):
        n_ok = chain_for(k)[2]
        if n_ok:
            scored.append((n_ok, k))
    scored.sort(key=lambda sk: (-sk[0], sk[1]))
    if not scored or scored[0][0] < min_pairs - 1:
        raise PairingError(
            "找不到一致的 count−seq 對應（TM2 與 Pi 邊緣的時間間隔對不起來）—— "
            "檢查邊緣極性、TM2 時間基準、Pi 是否用 CLOCK_MONOTONIC")
    k_best = scored[0][1]
    i, j, _ = chain_for(k_best)
    a, b = _lsq(X[j], Y[i])
    # 分數接近的其他 k：若落在同一條時鐘線上（某一側中途漏一個邊緣，前後兩段各成一個 k）就沒有歧義；
    # 若是另一條線（觸發間隔幾乎完全週期性，錯位一張也對得上）就拒絕猜測 —— 那等於按順序配對
    for sc, k2 in scored[1:]:
        if sc < scored[0][0] - 2:
            break
        i2, j2, _ = chain_for(k2)
        if np.max(np.abs(Y[i2] - (a * X[j2] + b))) > pair_tol_s:
            raise PairingError(
                f"count−seq 的對應有歧義：k={k_best}（{scored[0][0]} 步一致）與 k={k2}（{sc} 步）"
                "落在不同的時鐘線上（觸發間隔過於週期性），拒絕猜測")

    # ---- 2. 依模型為每個 TM2 邊緣找最近的 Pi 邊緣；配對與擬合交替三輪 ----
    pair_i = pair_j = np.zeros(0, dtype=np.int64)
    for _ in range(3):
        xh = (Y - b) / a
        pos = np.searchsorted(X, xh)
        lo = np.clip(pos - 1, 0, X.size - 1)
        hi = np.clip(pos, 0, X.size - 1)
        cand = np.where(np.abs(X[lo] - xh) <= np.abs(X[hi] - xh), lo, hi)
        res = Y - (a * X[cand] + b)
        ok = np.abs(res) <= pair_tol_s
        ti = np.nonzero(ok)[0]
        tj = cand[ok]
        # 一個 Pi 邊緣只能配一個 TM2（容差遠小於拍照間隔，理論上不會發生；發生就留殘差小的）
        keep = {}
        for a_i, a_j in zip(ti, tj):
            if a_j not in keep or abs(res[a_i]) < abs(res[keep[a_j]]):
                keep[a_j] = a_i
        pair_j = np.array(sorted(keep), dtype=np.int64)
        pair_i = np.array([keep[q] for q in pair_j], dtype=np.int64)
        if pair_i.size < 2:
            break
        a, b = _lsq(X[pair_j], Y[pair_i])

    if pair_i.size < min_pairs:
        raise PairingError(f"只配到 {pair_i.size} 對（至少要 {min_pairs}）")
    if abs(a - 1.0) > rate:
        raise PairingError(f"擬合的時鐘頻差 {(a - 1) * 1e6:+.1f} ppm 超過 ±{MAX_CLOCK_RATE_PPM:.0f} ppm，不可信")
    res_all = Y[pair_i] - (a * X[pair_j] + b)
    fit = ClockFit(slope=a, offset_s=b + y0, x0_ns=x0, counter_offset=k_best,
                   n_pairs=int(pair_i.size), rms_s=float(np.sqrt(np.mean(res_all ** 2))),
                   max_abs_s=float(np.max(np.abs(res_all))))

    # ---- 3. 逐 Pi 邊緣輸出 ----
    by_j = {int(q): (int(p), float(rr)) for q, p, rr in zip(pair_j, pair_i, res_all)}
    x_lo, x_hi = X[pair_j].min(), X[pair_j].max()
    events: list[PairedEvent] = []
    shifts: list[tuple[int, int]] = []
    cur_k = None
    for jj, f in enumerate(fr):
        flags: list[str] = []
        if jj in by_j:
            ii, rr = by_j[jj]
            e = tm2[ii]
            t_edge = e.t_gpst_s
            count, acc = e.count, e.acc_est_ns
            flags += list(e.flags)
            kk = e.count - int(S[jj])
            if kk != k_best:
                flags.append("counter_shift")
            if kk != cur_k:
                shifts.append((int(S[jj]), kk))
                cur_k = kk
        else:
            t_edge = float(fit.predict(E[jj]))
            count, acc, rr = None, None, None
            flags.append("tm2_missing")
            if X[jj] < x_lo or X[jj] > x_hi:
                flags.append("fit_extrapolated")
        if offs is None:
            t_exp = t_edge
            flags.append("edge_time_only")
        else:
            o = offs[jj]
            if o is None or not np.isfinite(o):
                t_exp = None
                flags.append("no_exposure_offset")
            else:
                t_exp = t_edge + a * float(o)
        events.append(PairedEvent(
            seq=int(S[jj]), edge_ns=int(E[jj]), tm2_count=count, t_edge_gpst_s=t_edge,
            t_exposure_gpst_s=t_exp, residual_s=rr, acc_est_ns=acc, flags=tuple(flags),
            file=f.get("file"), status=f.get("status"),
        ))
    paired_i = set(int(p) for p in pair_i)
    unmatched_tm2 = [e for n_, e in enumerate(tm2) if n_ not in paired_i]
    unmatched_pi = [ev.seq for ev in events if ev.tm2_count is None]
    if unmatched_tm2 or unmatched_pi:
        _log.warning("TM2↔Pi 配對：%d 對；未配到 TM2 %d 個、Pi %d 個",
                     fit.n_pairs, len(unmatched_tm2), len(unmatched_pi))
    return PairingResult(events=events, fit=fit, unmatched_tm2=unmatched_tm2,
                         unmatched_pi_seq=unmatched_pi, counter_shifts=shifts,
                         pair_tol_s=pair_tol_s, lost=lost)
