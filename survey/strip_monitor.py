"""建圖條帶的即時張數監測（討論文件 A5）：哪一條帶「拍了 0 張」、張數不足、被中斷後「沒倒帶就接續」、或根本沒飛。

純 Python（不用 Qt、不用 numpy）。GUI 把遙測事件餵進 :class:`StripPhotoMonitor`，自己決定怎麼顯示警報。

條帶段表（:func:`camera_segments`）只看**任務項目**：hv 匯出器每條帶開一次相機（206 d>0）、關一次（206 0）；
最後一條帶由 ``帶終點 NAV → DO_LAND_START(189) → 206(0)`` 收尾；地形重觸發是相機開著時再送一次 206(d') ——
**不是**新條帶（與 ``core.recon.exposure_verifier.simulate_mission_photos`` 的 strip_no 同一個規則，所以段的
``index`` ＝ 飛前模擬照片的 ``strip`` 鍵）。

ArduPilot 語意（master 原始碼，2026-10-15 核對；4.5／4.6 板子未逐版本核對）：

* ``AP_Mission::start_command`` 對**每一個**指令（DO 也算）送 STATUSTEXT ``"Mission: <seq> <type>"`` ⇒ 206 開／關
  真正執行的瞬間看得到 ⇒ **主要來源** :meth:`StripPhotoMonitor.item_started`。
* MISSION_CURRENT.seq ＝ **正飛向**的 NAV（``get_current_nav_index``）；4 Hz 取樣、同步交易期間會斷 ⇒ 只當備援
  （:meth:`StripPhotoMonitor.mission_current`），用區間判斷：相機開著的整段 MISSION_CURRENT ∈ (arm_nav, end_nav]。
* MIS_RESTART=0（預設，resume）：中斷後回到 AUTO 從目前 NAV 接續 ⇒ 中斷點到帶終點之間是斜飛回接、該段會漏拍或拍到
  斜照 ⇒ 監測器在「離開 AUTO 時相機開著、回 AUTO 時 MISSION_CURRENT 還在同一段而且中間沒倒帶」時發
  ``resumed_without_rewind``（SOP：先把 MISSION_CURRENT 設回 ``rejoin_nav_seq`` 再切 AUTO）。
* CAMERA_FEEDBACK 只代表**飛控觸發了**；沒接熱靴回授腳時，相機端（樹莓派）漏拍地面站看不到 ⇒ 0 張警報只涵蓋
  飛控端故障（206 沒執行、CAM1_TYPE 錯、CAM_AUTO_ONLY 擋住、側傾一直超過 CAM_MAX_ROLL…）。

時間 ``t``（秒，單調遞增）一律由呼叫端給（GUI 用 ``time.monotonic()``），模組本身不讀時鐘 ⇒ 測試可重播。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Optional, Sequence

MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
MAV_CMD_DO_LAND_START = 189
#: ArduPilot ``AP_Mission::is_nav_cmd``：``cmd.id <= MAV_CMD_NAV_LAST``（95）
MAV_CMD_NAV_LAST = 95

#: 預設門檻（有效值寫在 :attr:`StripPhotoMonitor.effective`）
DEFAULT_MIN_RATIO = 0.8      # 張數 < floor(0.8·預期) ⇒ under_count（有風時 CAM_MAX_ROLL 會延後觸發 ⇒ 保守）
DEFAULT_GRACE_S = 2.0        # 段關閉後等這麼久才評估（CAMERA_FEEDBACK 與 STATUSTEXT 先後不定）
DEFAULT_GAP_S = 5.0          # 一次嘗試中任兩個事件相隔超過這個秒數 ⇒ 遙測中斷，張數不可信

ALARM_KINDS = ("zero_photos", "under_count", "unverified_count", "interrupted", "resumed_without_rewind",
               "jumped", "not_flown", "off_strip_photos")


def _is_nav(cmd: int) -> bool:
    return 0 < int(cmd) <= MAV_CMD_NAV_LAST


def _fields(obj: Any) -> tuple[int, int, float]:
    """任務項目 → (seq, command, param1)。吃 dict（seq/command/param1 或 p1）、MissionItem（屬性）、或 10-tuple。"""
    if isinstance(obj, Mapping):
        p1 = obj.get("param1", obj.get("p1", 0.0))
        return int(obj["seq"]), int(obj["command"]), float(p1 or 0.0)
    if hasattr(obj, "command") and hasattr(obj, "seq"):
        return int(obj.seq), int(obj.command), float(getattr(obj, "param1", 0.0) or 0.0)
    if isinstance(obj, (tuple, list)) and len(obj) == 10:
        return int(obj[0]), int(obj[1]), float(obj[3])
    raise TypeError(f"任務項目要是 dict、MissionItem 或 10-tuple，收到 {obj!r}")


@dataclass(frozen=True)
class CamSegment:
    """一段「相機開著」（＝一條帶的一次 206 開窗）。seq 都是任務項目序號（seq 0 ＝ home 槽）。"""
    index: int                         # 1 起；＝ 飛前模擬照片的 strip 鍵
    on_seq: int                        # 206(d>0)
    off_seq: Optional[int]             # 206(0)；None ＝ 任務結束都沒關（匯出器不會產生）
    arm_nav_seq: Optional[int]         # 抵達後執行 on_seq 的 NAV（引入 NAV 或帶起點 NAV）
    end_nav_seq: Optional[int]         # 抵達後執行 off_seq 的 NAV（帶終點）
    next_nav_seq: Optional[int]        # off_seq 之後第一個 NAV（正常完成後 MISSION_CURRENT 會是它）
    rejoin_nav_seq: Optional[int]      # arm_nav 前一個 NAV：倒帶重飛時 MISSION_SET_CURRENT 設這個（含進場彎與引入段）
    trigg_dist_m: float
    retrigger_seqs: tuple = ()         # 相機開著時再送的 206(d')（地形重觸發）
    closed_by_land_start: bool = False  # off 之前隔著 DO_LAND_START（最後一條帶）
    strip_id: Optional[str] = None
    expected: Optional[int] = None     # 預期張數（飛前模擬）；None ＝ 不判張數不足

    def contains_current(self, seq: int) -> bool:
        """MISSION_CURRENT（正飛向的 NAV）落在這段相機開著的區間 (arm_nav, end_nav]。"""
        if self.arm_nav_seq is None or self.end_nav_seq is None:
            return False
        return self.arm_nav_seq < int(seq) <= self.end_nav_seq


def camera_segments(items: Sequence[Any], *, strip_ids: Optional[Sequence[str]] = None,
                    expected: Optional[Sequence[int]] = None) -> list[CamSegment]:
    """任務項目 → 條帶段表。``strip_ids``／``expected`` 依段的順序給（長度必須等於段數，否則 ValueError —— 不靜默錯位）。"""
    rows = sorted((_fields(it) for it in items), key=lambda r: r[0])
    navs = [s for s, c, _p in rows if _is_nav(c) and s > 0]
    land_starts = [s for s, c, _p in rows if c == MAV_CMD_DO_LAND_START]

    def prev_nav(seq: int) -> Optional[int]:
        cand = [n for n in navs if n < seq]
        return cand[-1] if cand else None

    def next_nav(seq: int) -> Optional[int]:
        cand = [n for n in navs if n > seq]
        return cand[0] if cand else None

    raw: list[dict] = []
    cur: Optional[dict] = None
    for seq, cmd, p1 in rows:
        if cmd != MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            continue
        if p1 > 0:
            if cur is None:
                cur = {"on": seq, "d": p1, "re": []}
            else:
                cur["re"].append(seq)                      # 地形重觸發：不是新條帶
        elif cur is not None:
            cur["off"] = seq
            raw.append(cur)
            cur = None
        # p1 <= 0 而相機本來就關（DO_LAND_START 後多出來的那個）⇒ 忽略
    if cur is not None:
        raw.append(cur)
    n = len(raw)
    if strip_ids is not None and len(strip_ids) != n:
        raise ValueError(f"strip_ids 有 {len(strip_ids)} 個，但任務裡有 {n} 段 206 開窗（不靜默錯位）")
    if expected is not None and len(expected) != n:
        raise ValueError(f"expected 有 {len(expected)} 個，但任務裡有 {n} 段 206 開窗（不靜默錯位）")
    out: list[CamSegment] = []
    for i, r in enumerate(raw):
        off = r.get("off")
        arm = prev_nav(r["on"])
        end = prev_nav(off) if off is not None else None
        out.append(CamSegment(
            index=i + 1, on_seq=r["on"], off_seq=off, arm_nav_seq=arm, end_nav_seq=end,
            next_nav_seq=next_nav(off) if off is not None else None,
            rejoin_nav_seq=prev_nav(arm) if arm is not None else None,
            trigg_dist_m=float(r["d"]), retrigger_seqs=tuple(r["re"]),
            closed_by_land_start=bool(off is not None and end is not None
                                      and any(end < s < off for s in land_starts)),
            strip_id=(None if strip_ids is None else str(strip_ids[i])),
            expected=(None if expected is None else int(expected[i]))))
    return out


@dataclass
class Alarm:
    kind: str                          # ALARM_KINDS 之一
    segment: Optional[int]             # 段 index（off_strip_photos 為 None）
    strip_id: Optional[str]
    attempt: int
    photos: int
    expected: Optional[int]
    t: float
    severity: str                      # "critical" | "warning" | "info"
    message: str
    rejoin_nav_seq: Optional[int] = None


@dataclass
class _Attempt:
    n: int
    t_start: float
    photos: int = 0
    t_end: Optional[float] = None
    end: str = ""                      # "closed" | "jumped" | "interrupted" | "rewound" | "finished"
    paused: bool = False               # 相機開著時離開 AUTO
    resumed_without_rewind: bool = False
    gap: bool = False                  # 期間遙測中斷
    evaluated: bool = False


@dataclass
class _SegState:
    seg: CamSegment
    attempts: list = field(default_factory=list)

    @property
    def last(self) -> Optional[_Attempt]:
        return self.attempts[-1] if self.attempts else None


class StripPhotoMonitor:
    """一架的條帶張數監測器。事件方法都只更新狀態；警報從 :meth:`poll` 與 :meth:`finish` 回傳（每件事只發一次）。"""

    def __init__(self, segments: Sequence[CamSegment], *, min_ratio: float = DEFAULT_MIN_RATIO,
                 grace_s: float = DEFAULT_GRACE_S, gap_s: float = DEFAULT_GAP_S) -> None:
        if not (0.0 < float(min_ratio) <= 1.0):
            raise ValueError(f"min_ratio 須 ∈ (0, 1]，得 {min_ratio!r}")
        for name, v in (("grace_s", grace_s), ("gap_s", gap_s)):
            if not (math.isfinite(float(v)) and float(v) >= 0.0):
                raise ValueError(f"{name} 須為 ≥ 0 的有限數，得 {v!r}")
        self.segments = list(segments)
        self.min_ratio, self.grace_s, self.gap_s = float(min_ratio), float(grace_s), float(gap_s)
        self.effective = {"min_ratio": self.min_ratio, "grace_s": self.grace_s, "gap_s": self.gap_s,
                          "n_segments": len(self.segments),
                          "expected_known": all(s.expected is not None for s in self.segments)}
        self._st = [_SegState(s) for s in self.segments]
        self._by_on = {s.on_seq: i for i, s in enumerate(self.segments)}
        self._active: Optional[int] = None         # 目前相機開著的段（索引）
        self._auto: Optional[bool] = None
        self._mc: Optional[int] = None             # 最近一次 MISSION_CURRENT
        self._last_event_t: Optional[float] = None
        self._recent_closed: Optional[tuple[int, float]] = None   # (段索引, 關閉時刻)：grace 內的照片歸它
        self._jump_pending: Optional[tuple[int, float]] = None   # (段索引, 時刻)：開著時看到段外的項目先開始
        self._held: list[tuple[float, int]] = []    # 沒有段可歸的照片（等 grace_s 看有沒有段開）
        self._alarms: list[Alarm] = []
        self.off_strip_photos = 0
        self._finished = False

    # ── 內部 ─────────────────────────────────────────────────────────────
    def _tick(self, t: float) -> None:
        t = float(t)
        if self._last_event_t is not None and self._active is not None and t - self._last_event_t > self.gap_s:
            a = self._st[self._active].last
            if a is not None:
                a.gap = True
        if self._last_event_t is None or t > self._last_event_t:
            self._last_event_t = t

    def _alarm(self, kind: str, i: Optional[int], a: Optional[_Attempt], t: float, severity: str, msg: str) -> None:
        seg = self.segments[i] if i is not None else None
        self._alarms.append(Alarm(kind=kind, segment=(seg.index if seg else None),
                                  strip_id=(seg.strip_id if seg else None), attempt=(a.n if a else 0),
                                  photos=(a.photos if a else 0), expected=(seg.expected if seg else None),
                                  t=float(t), severity=severity, message=msg,
                                  rejoin_nav_seq=(seg.rejoin_nav_seq if seg else None)))

    def _label(self, i: int) -> str:
        s = self.segments[i]
        return f"條帶 {s.index}" + (f"（{s.strip_id}）" if s.strip_id else "")

    def _open(self, i: int, t: float) -> None:
        if self._active is not None and self._active != i:
            self._close(self._active, t, "jumped")
        st = self._st[i]
        if self._active == i and st.last is not None and st.last.t_end is None:
            return                                  # 重複的開（STATUSTEXT 與 MISSION_CURRENT 都報）
        st.attempts.append(_Attempt(n=len(st.attempts) + 1, t_start=float(t)))
        self._active = i
        # grace 內先到、沒段可歸的照片（第一張在 206 開的同一個 tick 拍）
        keep = []
        for th, n in self._held:
            if float(t) - th <= self.grace_s:
                st.last.photos += n
            else:
                keep.append((th, n))
        self._held = keep

    def _close(self, i: int, t: float, how: str) -> None:
        if self._jump_pending is not None and self._jump_pending[0] == i:
            self._jump_pending = None
        a = self._st[i].last
        if a is not None and a.t_end is None:
            a.t_end, a.end = float(t), how
        if self._active == i:
            self._active = None
        self._recent_closed = (i, float(t))

    # ── 事件 ─────────────────────────────────────────────────────────────
    def item_started(self, seq: int, t: float) -> None:
        """STATUSTEXT「Mission: <seq> …」（飛控開始執行第 seq 項）。主要來源。"""
        self._tick(t)
        seq = int(seq)
        if seq in self._by_on:
            self._open(self._by_on[seq], t)
            return
        if self._active is not None:
            s = self.segments[self._active]
            if s.off_seq is not None and seq == s.off_seq:
                self._jump_pending = None
                self._close(self._active, t, "closed")
            elif s.off_seq is not None and seq > s.off_seq:
                # ⚠ 實測（SITL S1_HDMAP_20261015_PW3 .BIN）：最後一條帶 ``帶終點 NAV → DO_LAND_START → 206(0)``，
                # 飛控先開始**下一個 NAV**、0.1 s 後才執行那個 206(0)（DO 在下一個 NAV 飛行中逐週期執行）⇒ 這裡只先記下；
                # grace 內收到本段的 206(0) ＝ 正常關閉，沒收到才算跳號（相機實際上到 206(0) 才關，照片照常歸本段）
                if self._jump_pending is None:
                    self._jump_pending = (self._active, float(t))
            elif s.arm_nav_seq is not None and seq <= s.arm_nav_seq:
                self._jump_pending = None
                self._close(self._active, t, "rewound")

    def mission_current(self, seq: int, t: float) -> None:
        """MISSION_CURRENT.seq（正飛向的 NAV）。備援：STATUSTEXT 掉了也能推出段的開關與倒帶。"""
        self._tick(t)
        seq = int(seq)
        prev, self._mc = self._mc, seq
        if self._active is not None:
            s = self.segments[self._active]
            a = self._st[self._active].last
            if s.end_nav_seq is not None and seq > s.end_nav_seq:
                self._close(self._active, t, "closed" if seq == s.next_nav_seq else "jumped")
            elif s.arm_nav_seq is not None and seq <= s.arm_nav_seq:
                if a is not None and a.paused:
                    a.paused = False                    # 照 SOP 倒帶（暫停中把 MISSION_CURRENT 設回段前）
                self._close(self._active, t, "rewound")
            return
        if self._auto is False:
            return
        for i, s in enumerate(self.segments):
            if s.contains_current(seq):
                last = self._st[i].last
                fresh = last is None or (last.t_end is not None and prev is not None and prev <= (s.arm_nav_seq or 0))
                if fresh:
                    self._open(i, t)                    # STATUSTEXT 沒收到 206 開，但已經在帶上
                return

    def mode(self, mode: str, armed: bool, t: float) -> None:
        """HEARTBEAT 的模式與解鎖狀態（MISSION_CURRENT.mission_mode 在非 AUTO 時送 0＝不知道，所以模式要看這裡）。"""
        self._tick(t)
        auto = str(mode or "").upper() == "AUTO" and bool(armed)
        was = self._auto
        self._auto = auto
        if self._active is None:
            return
        a = self._st[self._active].last
        if a is None:
            return
        if was is not False and not auto and not a.paused:
            a.paused = True
            self._alarm("interrupted", self._active, a, t, "warning",
                        f"{self._label(self._active)} 拍攝中離開 AUTO（{mode}）：回 AUTO 前先把目前航點設回 seq "
                        f"{self.segments[self._active].rejoin_nav_seq}（重飛進場與引入段），不要直接切回 AUTO")
        elif was is False and auto and a.paused:
            s = self.segments[self._active]
            if self._mc is not None and s.contains_current(self._mc):
                a.resumed_without_rewind = True
                self._alarm("resumed_without_rewind", self._active, a, t, "critical",
                            f"{self._label(self._active)} 中斷後沒倒帶就接續（MIS_RESTART=0 從目前航點 seq {self._mc} 接續）"
                            f"⇒ 中斷點到帶上的回接段會漏拍或拍到斜照；落地後補飛，或依 SOP 設回 seq {s.rejoin_nav_seq} 重飛")
            a.paused = False

    def photos(self, n_new: int, t: float, *, reset: bool = False) -> None:
        """CAMERA_FEEDBACK 的新增張數（SITLLink 已處理 img_idx 回捲與合併）；``reset``＝飛控重開機（只記錄）。"""
        self._tick(t)
        n = int(n_new)
        if n <= 0:
            return
        if self._active is not None:
            self._st[self._active].last.photos += n
            return
        if self._recent_closed is not None and float(t) - self._recent_closed[1] <= self.grace_s:
            a = self._st[self._recent_closed[0]].last
            if a is not None and not a.evaluated:
                a.photos += n
                return
        self._held.append((float(t), n))

    def poll(self, t: float) -> list[Alarm]:
        """評估 grace 已過的段；回傳**新的**警報（之前回傳過的不再回傳）。"""
        t = float(t)
        if self._jump_pending is not None and t - self._jump_pending[1] > self.grace_s:
            i, tj = self._jump_pending
            self._jump_pending = None
            if self._active == i:
                self._close(i, tj, "jumped")
        keep, expired = [], 0
        for th, n in self._held:
            if t - th > self.grace_s:
                expired += n
            else:
                keep.append((th, n))
        self._held = keep
        if expired:
            self.off_strip_photos += expired
            if self._auto:                              # 一次 poll 合併成一則（不逐張洗版）
                self._alarm("off_strip_photos", None, None, t, "warning",
                            f"AUTO 中有 {expired} 張不在任何條帶內（相機在帶外觸發；累計 {self.off_strip_photos} 張）")
        for i, st in enumerate(self._st):
            for a in st.attempts:
                if a.evaluated or a.t_end is None or t - a.t_end < self.grace_s:
                    continue
                a.evaluated = True
                self._evaluate(i, a, t)
        out, self._alarms = self._alarms, []
        return out

    def _evaluate(self, i: int, a: _Attempt, t: float) -> None:
        s = self.segments[i]
        lab = self._label(i)
        if a.end == "jumped":
            self._alarm("jumped", i, a, t, "warning",
                        f"{lab} 沒有正常關相機就跳走（失效保護跳降落序列或手動跳號）：拍到 {a.photos} 張 ⇒ 落地後補飛")
        if a.end in ("rewound",) and a.photos == 0:
            return                                      # 照 SOP 倒帶：這次嘗試本來就會在帶前結束
        if a.photos == 0:
            if a.gap:
                self._alarm("unverified_count", i, a, t, "info",
                            f"{lab} 張數 0，但期間遙測中斷超過 {self.gap_s:g} s ⇒ 不確定，飛後用 .BIN 對帳")
            else:
                self._alarm("zero_photos", i, a, t, "critical",
                            f"{lab} 第 {a.n} 次飛完拍了 0 張（飛控沒觸發）⇒ 先看相機／CAM1_TYPE／CAM_AUTO_ONLY；"
                            f"重飛請把目前航點設回 seq {s.rejoin_nav_seq}")
            return
        if s.expected is not None and a.end == "closed":
            need = max(1, math.floor(self.min_ratio * s.expected))
            if a.photos < need:
                self._alarm("under_count" if not a.gap else "unverified_count", i, a, t,
                            "warning" if not a.gap else "info",
                            f"{lab} 拍了 {a.photos} 張 < {need}（預期 {s.expected} 的 {self.min_ratio:g}）"
                            + ("；期間遙測中斷，張數不確定" if a.gap else "；有風時 CAM_MAX_ROLL 會延後觸發"))

    def finish(self, t: float) -> list[Alarm]:
        """落地／上鎖：還開著的段算中斷，從沒開過的段算沒飛；回傳新的警報（含 poll 剩下的）。"""
        if self._finished:
            return []
        self._finished = True
        t = float(t)
        if self._active is not None:
            self._close(self._active, t, "finished")
        for i, st in enumerate(self._st):
            if not st.attempts:
                self._alarm("not_flown", i, None, t, "warning", f"{self._label(i)} 整條沒飛到 ⇒ 落地後補飛")
        return self.poll(t + self.grace_s + 1e-9)

    # ── 顯示 ─────────────────────────────────────────────────────────────
    def summary(self) -> list[dict]:
        """逐段：state（pending／active／done／interrupted／jumped）、attempts、photos（全部嘗試合計）、expected。"""
        out = []
        for i, st in enumerate(self._st):
            a = st.last
            if a is None:
                state = "pending"
            elif self._active == i:
                state = "paused" if a.paused else "active"
            else:
                state = {"closed": "done", "jumped": "jumped", "rewound": "rewound",
                         "finished": "interrupted"}.get(a.end, a.end or "done")
            out.append({"index": st.seg.index, "strip_id": st.seg.strip_id, "state": state,
                        "attempts": len(st.attempts), "photos": sum(x.photos for x in st.attempts),
                        "photos_last": (a.photos if a else 0), "expected": st.seg.expected,
                        "rejoin_nav_seq": st.seg.rejoin_nav_seq,
                        "resumed_without_rewind": any(x.resumed_without_rewind for x in st.attempts)})
        return out

    def progress_text(self) -> str:
        """面板一行：條帶完成數、警報中的段。"""
        sm = self.summary()
        done = sum(1 for r in sm if r["state"] == "done")
        bad = [r for r in sm if r["state"] in ("jumped", "interrupted") or r["resumed_without_rewind"]
               or (r["state"] == "done" and r["photos_last"] == 0)]
        return (f"條帶 {done}/{len(sm)} 完成"
                + ("" if not bad else "；⚠ " + "、".join(f"#{r['index']}" for r in bad)))


def expected_by_segment(photo_strips: Iterable[Optional[str]], n_segments: int) -> list[int]:
    """飛前模擬照片的 strip 鍵（"1"、"2"…；None ＝ 帶外）→ 每段預期張數（長度 = n_segments）。"""
    out = [0] * int(n_segments)
    for k in photo_strips:
        if k is None:
            continue
        j = int(k) - 1
        if not (0 <= j < n_segments):
            raise ValueError(f"照片的條帶鍵 {k!r} 超出段數 {n_segments}（段表與模擬對不上）")
        out[j] += 1
    return out
