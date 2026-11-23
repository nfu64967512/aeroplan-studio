"""hv_dccpp.dubins — Dubins 曲線（6 word）、離散化、風中時間積分、閉式對照。

對應規格 §8.1（狀態與時間積分）、§8.3（轉場幾何閉式；D5）、§8.5（外切多邊形離散；D27）、
D14（弧段時間一律沿弧以 :math:`V_g(\\theta)` 數值積分）；論文 Eq.(27)–(29)。

**座標與方位約定（契約 §0）**

* 本地 ENU（m）：``x`` = East、``y`` = North。
* 航跡方位 ``theta`` 自北順時針（rad）：切向 :math:`\\hat t=(\\sin\\theta,\\cos\\theta)`、
  右法向 :math:`\\hat n=(\\cos\\theta,-\\sin\\theta)`。
* ``L`` = 左轉 = ``theta`` 遞減（``dtheta_rad < 0``）；``R`` = 右轉 = ``theta`` 遞增（``dtheta_rad > 0``）。
  左轉圓心 :math:`c=p-R\\hat n`、右轉圓心 :math:`c=p+R\\hat n`。
* 內部求解時以數學方位 :math:`\\varphi=\\pi/2-\\theta`（自 East 逆時針）套用
  Shkel & Lumelsky (2001) 的 6-word 閉式；左轉在兩種約定下皆為左轉，word 不需對調。

本模組為第 0 層：只依賴 numpy / shapely。``path_time`` / ``path_time_and_energy`` 對
``wind.WindField`` 與 ``platform.EnergyModel`` 採鴨子型別（duck typing），不在 import 期依賴。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal, Sequence

import numpy as np
import shapely

__all__ = [
    "Pose",
    "DubinsSegment",
    "DubinsPath",
    "WORDS",
    "dubins_all",
    "dubins_all_words",
    "dubins_solve_all",
    "dubins_all_times",
    "dubins_path",
    "discretize",
    "discretize_step_rad",
    "path_time",
    "path_time_and_energy",
    "ground_speed_uniform",
    "omega_length",
    "loop_length",
    "uturn_length",
    "skip_k",
    "parallel_transfer_pose_pair",
]

TWO_PI: float = 2.0 * math.pi
_SNAP_RAD: float = 1e-9          # 弧角落在 [2π−ε, 2π) 視為 0（整圈不可能最短）
_ZERO_LEN: float = 1e-9          # 視為零長度之段（m）

WORDS: tuple[str, ...] = ("LSL", "RSR", "LSR", "RSL", "RLR", "LRL")
_WORD_INDEX: dict[str, int] = {w: i for i, w in enumerate(WORDS)}
# 各 word 三段之轉向符號（L=−1、R=+1、S=0）
_SIGN1 = np.array([-1.0, +1.0, -1.0, +1.0, +1.0, -1.0])
_SIGN2 = np.array([0.0, 0.0, 0.0, 0.0, -1.0, +1.0])
_SIGN3 = np.array([-1.0, +1.0, +1.0, -1.0, +1.0, -1.0])


# ---------------------------------------------------------------------------
# 資料型別
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class Pose:
    """平面狀態 (x, y, theta)；theta 自北順時針（rad）。"""

    x_m: float
    y_m: float
    theta_rad: float

    def as_array(self) -> np.ndarray:
        """回傳 (3,) ``[x_m, y_m, theta_rad]``。"""
        return np.array([self.x_m, self.y_m, self.theta_rad], dtype=float)


@dataclass(frozen=True, slots=True)
class DubinsSegment:
    """Dubins 單段。

    * ``kind`` ``"S"``：``center=None``、``dtheta_rad=0``、``length_m`` 為直線長。
    * ``kind`` ``"L"``：``dtheta_rad<0``（方位減小）；``"R"``：``dtheta_rad>0``。
      ``length_m = R·|dtheta_rad|``；弧上任一點（方位 θ）位置 = ``center − sign·R·n̂(θ)``，
      sign = −1 (L) / +1 (R)。
    """

    kind: Literal["L", "R", "S"]
    length_m: float
    start: Pose
    end: Pose
    center: tuple[float, float] | None
    dtheta_rad: float


@dataclass(frozen=True, slots=True)
class DubinsPath:
    """三段 Dubins 路徑（word ∈ WORDS）。零長度之段仍保留，使 ``segments`` 恆為 3 段。"""

    word: str
    R_m: float
    segments: tuple[DubinsSegment, ...]
    length_m: float

    @property
    def start(self) -> Pose:
        return self.segments[0].start

    @property
    def end(self) -> Pose:
        return self.segments[-1].end

    def sample(self, ds_m: float) -> np.ndarray:
        """沿路徑每 ≤ ``ds_m`` 取樣，回傳 (M,3) ``[x, y, theta]``，含首尾端點。

        首列 = ``start``、末列 = ``end``（與 ``DubinsPath.start/end`` 完全一致）；
        中間各列 theta 為自 ``start.theta_rad`` 連續累積之方位（未取模）。
        """
        if not ds_m > 0:
            raise ValueError("ds_m must be > 0")
        st = self.start
        rows: list[np.ndarray] = [np.array([[st.x_m, st.y_m, st.theta_rad]])]
        for seg in self.segments:
            if seg.length_m <= _ZERO_LEN:
                continue
            n = max(1, int(math.ceil(seg.length_m / ds_m - 1e-12)))
            tau = np.linspace(0.0, 1.0, n + 1)[1:]
            x0, y0, th0 = seg.start.x_m, seg.start.y_m, seg.start.theta_rad
            if seg.kind == "S":
                xs = x0 + tau * seg.length_m * math.sin(th0)
                ys = y0 + tau * seg.length_m * math.cos(th0)
                ths = np.full(n, th0)
            else:
                assert seg.center is not None
                sign = 1.0 if seg.kind == "R" else -1.0
                ths = th0 + seg.dtheta_rad * tau
                cx, cy = seg.center
                xs = cx - sign * self.R_m * np.cos(ths)
                ys = cy + sign * self.R_m * np.sin(ths)
            rows.append(np.column_stack([xs, ys, ths]))
        out = np.vstack(rows)
        en = self.end
        if out.shape[0] == 1:
            out = np.vstack([out, [[en.x_m, en.y_m, en.theta_rad]]])
        else:
            out[-1] = (en.x_m, en.y_m, en.theta_rad)
        return out

    def polyline(self, ds_m: float) -> shapely.LineString:
        """``sample(ds_m)`` 之 xy 折線（shapely LineString）。"""
        return shapely.LineString(self.sample(ds_m)[:, :2])


# ---------------------------------------------------------------------------
# 基本工具
# ---------------------------------------------------------------------------
def _mod2pi(x: np.ndarray) -> np.ndarray:
    """取模至 [0, 2π)，並將 [2π−ε, 2π) 貼齊為 0。"""
    y = np.mod(x, TWO_PI)
    return np.where(y > TWO_PI - _SNAP_RAD, 0.0, y)


def _to_array(poses: Any) -> np.ndarray:
    """Pose / (3,) / (N,3) / 序列 → (N,3) float64。"""
    if isinstance(poses, Pose):
        return poses.as_array()[None, :]
    if isinstance(poses, np.ndarray):
        return np.asarray(poses, dtype=float).reshape(-1, 3)
    rows = [p.as_array() if isinstance(p, Pose) else np.asarray(p, dtype=float) for p in poses]
    if not rows:
        return np.zeros((0, 3))
    return np.asarray(rows, dtype=float).reshape(-1, 3)


def _wind_xy_of(wind: Any, z_m: float) -> np.ndarray:
    """由 ``WindField``（``wind_vec(z)``）、(2,) 陣列或 None 取得單一高度之風向量 (wx, wy)。"""
    if wind is None:
        return np.zeros(2)
    if hasattr(wind, "wind_vec"):
        w = np.asarray(wind.wind_vec(np.atleast_1d(float(z_m))), dtype=float).reshape(-1, 2)
        return w[0]
    w = np.asarray(wind, dtype=float).reshape(-1)
    if w.size != 2:
        raise TypeError("wind must be a WindField-like object (wind_vec) or a (2,) wind vector")
    return w


def ground_speed_uniform(theta_rad: Any, v_air_mps: float, wind_xy: Any) -> np.ndarray:
    """風三角地速（規格 §3.2）：:math:`V_g=\\sqrt{V_a^2-W_\\perp^2}+W_\\parallel`。

    ``wind_xy`` 為吹向向量 (wx, wy)，(2,) 廣播或與 ``theta`` 同形逐點；
    :math:`|W_\\perp|\\ge V_a` 回 ``nan``（呼叫端視為不可行）。
    """
    th = np.asarray(theta_rad, dtype=float)
    w = np.asarray(wind_xy, dtype=float)
    wx, wy = w[..., 0], w[..., 1]
    st, ct = np.sin(th), np.cos(th)
    w_par = wx * st + wy * ct
    w_perp = wx * ct - wy * st
    rad = v_air_mps * v_air_mps - w_perp * w_perp
    with np.errstate(invalid="ignore"):
        return np.where(rad > 0.0, np.sqrt(np.where(rad > 0.0, rad, 0.0)) + w_par, np.nan)


def _arc_time_vec(theta0: np.ndarray, dtheta: np.ndarray, R_m: float, v_air_mps: float,
                  wind_xy: np.ndarray, step_rad: float) -> np.ndarray:
    """弧段時間（D14）：中點 Riemann 和，步長 ≤ ``step_rad``，向量化於 N 條弧。

    :math:`t=\\sum_j R\\,\\delta/V_g(\\theta_j)`，:math:`\\theta_j=\\theta_0+\\mathrm{sign}(\\Delta\\theta)(j+\\tfrac12)\\delta`，
    :math:`\\delta=|\\Delta\\theta|/n`，:math:`n=\\lceil|\\Delta\\theta|/\\text{step}\\rceil`。任一步 :math:`V_g\\le0` → inf。
    """
    if not step_rad > 0:
        raise ValueError("step_rad must be > 0")
    theta0 = np.asarray(theta0, dtype=float).reshape(-1)
    dtheta = np.asarray(dtheta, dtype=float).reshape(-1)
    mag = np.abs(dtheta)
    n = np.ceil(mag / step_rad - 1e-12).astype(int)
    n = np.where(mag > 0.0, np.maximum(n, 1), 0)
    out = np.zeros(mag.shape)
    if mag.size == 0:
        return out
    n_max = int(n.max())
    if n_max == 0:
        return out
    delta = np.where(n > 0, mag / np.maximum(n, 1), 0.0)
    j = np.arange(n_max, dtype=float)[None, :]
    th = theta0[:, None] + np.sign(dtheta)[:, None] * delta[:, None] * (j + 0.5)
    vg = ground_speed_uniform(th, v_air_mps, wind_xy)
    mask = j < n[:, None]
    ok = vg > 0.0
    contrib = np.where(mask & ok, R_m * delta[:, None] / np.where(ok, vg, 1.0), 0.0)
    out = contrib.sum(axis=1)
    out[(mask & ~ok).any(axis=1)] = np.inf
    return out


def _line_time_vec(theta: np.ndarray, length_m: np.ndarray, v_air_mps: float,
                   wind_xy: np.ndarray) -> np.ndarray:
    """直線段時間 :math:`L/V_g(\\theta)`；:math:`V_g\\le0` 或 nan → inf；L=0 → 0。"""
    theta = np.asarray(theta, dtype=float).reshape(-1)
    length_m = np.asarray(length_m, dtype=float).reshape(-1)
    vg = ground_speed_uniform(theta, v_air_mps, wind_xy)
    ok = vg > 0.0
    out = np.where(ok, length_m / np.where(ok, vg, 1.0), np.inf)
    out = np.where(length_m <= 0.0, 0.0, out)
    return out


# ---------------------------------------------------------------------------
# 6-word 求解（向量化）
# ---------------------------------------------------------------------------
def dubins_solve_all(starts: Any, ends: Any, R_m: float) -> tuple[np.ndarray, np.ndarray]:
    """對 N 對狀態求 6 個 word 之 (t, p, q) 與長度（Shkel & Lumelsky 閉式，向量化）。

    回傳 ``(lengths (N,6), params (N,6,3))``；``params[:, w] = (t, p, q)``：
    t、q 為首末弧角（rad，≥0），p 為中段：CSC 為直線長 / R、CCC 為中弧角（rad，∈[π,2π)）。
    不可行 word 之長度為 inf（params 為 nan）。
    """
    if not R_m > 0:
        raise ValueError("R_m must be > 0")
    S = _to_array(starts)
    E = _to_array(ends)
    if S.shape != E.shape:
        raise ValueError("starts and ends must have the same shape (N,3)")
    N = S.shape[0]
    lengths = np.full((N, 6), np.inf)
    params = np.full((N, 6, 3), np.nan)
    if N == 0:
        return lengths, params

    dx = E[:, 0] - S[:, 0]
    dy = E[:, 1] - S[:, 1]
    d = np.hypot(dx, dy) / R_m
    psi = np.arctan2(dy, dx)                       # 數學方位（自 East 逆時針）
    phi_s = 0.5 * math.pi - S[:, 2]
    phi_e = 0.5 * math.pi - E[:, 2]
    a = np.mod(phi_s - psi, TWO_PI)
    b = np.mod(phi_e - psi, TWO_PI)
    sa, ca = np.sin(a), np.cos(a)
    sb, cb = np.sin(b), np.cos(b)
    cab = np.cos(a - b)
    d2 = d * d

    def _put(w: int, ok: np.ndarray, t: np.ndarray, p: np.ndarray, q: np.ndarray) -> None:
        t = np.where(ok, t, np.nan)
        p = np.where(ok, p, np.nan)
        q = np.where(ok, q, np.nan)
        params[:, w, 0] = t
        params[:, w, 1] = p
        params[:, w, 2] = q
        lengths[:, w] = np.where(ok, (t + p + q) * R_m, np.inf)

    # 起訖圓重合（p≈0，切線方向未定義）時整條路徑即單一弧：t = 方位差、q = 0
    coincide_tol = 1e-9

    with np.errstate(invalid="ignore", divide="ignore"):
        # LSL
        tmp0 = d + sa - sb
        p2 = 2.0 + d2 - 2.0 * cab + 2.0 * d * (sa - sb)
        p = np.sqrt(np.maximum(p2, 0.0))
        tmp1 = np.arctan2(cb - ca, tmp0)
        t = _mod2pi(-a + tmp1)
        q = _mod2pi(b - tmp1)
        same = p < coincide_tol
        t = np.where(same, _mod2pi(b - a), t)
        q = np.where(same, 0.0, q)
        p = np.where(same, 0.0, p)
        _put(0, np.ones(N, bool), t, p, q)

        # RSR
        tmp0 = d - sa + sb
        p2 = 2.0 + d2 - 2.0 * cab + 2.0 * d * (sb - sa)
        p = np.sqrt(np.maximum(p2, 0.0))
        tmp1 = np.arctan2(ca - cb, tmp0)
        t = _mod2pi(a - tmp1)
        q = _mod2pi(-b + tmp1)
        same = p < coincide_tol
        t = np.where(same, _mod2pi(a - b), t)
        q = np.where(same, 0.0, q)
        p = np.where(same, 0.0, p)
        _put(1, np.ones(N, bool), t, p, q)

        # LSR
        p2 = -2.0 + d2 + 2.0 * cab + 2.0 * d * (sa + sb)
        ok = p2 >= 0.0
        p = np.sqrt(np.maximum(p2, 0.0))
        tmp2 = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
        _put(2, ok, _mod2pi(-a + tmp2), p, _mod2pi(-b + tmp2))

        # RSL
        p2 = d2 - 2.0 + 2.0 * cab - 2.0 * d * (sa + sb)
        ok = p2 >= 0.0
        p = np.sqrt(np.maximum(p2, 0.0))
        tmp2 = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
        _put(3, ok, _mod2pi(a - tmp2), p, _mod2pi(b - tmp2))

        # RLR（中弧 p = 2π − acos(tmp) ∈ [π, 2π]：Dubins CCC 最佳解之中弧必 > π）
        tmp = (6.0 - d2 + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0
        ok = np.abs(tmp) <= 1.0
        p = TWO_PI - np.arccos(np.clip(tmp, -1.0, 1.0))
        t = _mod2pi(a - np.arctan2(ca - cb, d - sa + sb) + 0.5 * p)
        q = _mod2pi(a - b - t + p)
        _put(4, ok, t, p, q)

        # LRL
        tmp = (6.0 - d2 + 2.0 * cab + 2.0 * d * (-sa + sb)) / 8.0
        ok = np.abs(tmp) <= 1.0
        p = TWO_PI - np.arccos(np.clip(tmp, -1.0, 1.0))
        t = _mod2pi(-a - np.arctan2(ca - cb, d + sa - sb) + 0.5 * p)
        q = _mod2pi(b - a - t + p)
        _put(5, ok, t, p, q)

    return lengths, params


def dubins_all_words(starts: Any, ends: Any, R_m: float) -> np.ndarray:
    """6 個 word 之長度 (N,6)（不可行為 inf）。"""
    return dubins_solve_all(starts, ends, R_m)[0]


def dubins_all(starts: Any, ends: Any, R_m: float) -> tuple[np.ndarray, np.ndarray]:
    """N 對狀態之最短 Dubins 長度與 word 索引（規格 §8.1；向量化）。

    參數 ``starts``/``ends``：(N,3) ``[x, y, theta]`` 或 ``Pose`` 序列。
    回傳 ``(lengths (N,), word_idx (N,) int8)``，``word_idx`` 對應 ``WORDS``。
    """
    lengths, _ = dubins_solve_all(starts, ends, R_m)
    if lengths.shape[0] == 0:
        return np.zeros(0), np.zeros(0, dtype=np.int8)
    idx = np.argmin(lengths, axis=1)
    best = lengths[np.arange(lengths.shape[0]), idx]
    return best, idx.astype(np.int8)


def dubins_all_times(starts: Any, ends: Any, R_m: float, v_air_mps: float, wind_xy: Any,
                     step_rad: float = 0.0873) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """N 對狀態之最短 word 於均勻風 ``wind_xy`` (2,) 下之時間（規格 §8.1；D14），向量化。

    等價於對每對呼叫 ``path_time(dubins_path(a, b, R), v_air, wind, z)``（同一風向量），
    供 ``costs.CostCache`` 建表時避免逐對 Python 迴圈。
    回傳 ``(times (N,), lengths (N,), word_idx (N,) int8)``；不可飛（:math:`V_g\\le0`）為 inf。
    """
    lengths_all, params = dubins_solve_all(starts, ends, R_m)
    N = lengths_all.shape[0]
    if N == 0:
        return np.zeros(0), np.zeros(0), np.zeros(0, dtype=np.int8)
    w = np.asarray(wind_xy, dtype=float).reshape(2)
    idx = np.argmin(lengths_all, axis=1)
    ar = np.arange(N)
    lengths = lengths_all[ar, idx]
    prm = params[ar, idx]                     # (N,3)
    t, p, q = prm[:, 0], prm[:, 1], prm[:, 2]
    s1, s2, s3 = _SIGN1[idx], _SIGN2[idx], _SIGN3[idx]
    th0 = _to_array(starts)[:, 2]
    th1 = th0 + s1 * t
    th2 = th1 + s2 * p
    time1 = _arc_time_vec(th0, s1 * t, R_m, v_air_mps, w, step_rad)
    time3 = _arc_time_vec(th2, s3 * q, R_m, v_air_mps, w, step_rad)
    time2 = np.zeros(N)
    is_arc = idx >= 4
    if (~is_arc).any():
        time2[~is_arc] = _line_time_vec(th1[~is_arc], p[~is_arc] * R_m, v_air_mps, w)
    if is_arc.any():
        time2[is_arc] = _arc_time_vec(th1[is_arc], (s2 * p)[is_arc], R_m, v_air_mps, w, step_rad)
    times = time1 + time2 + time3
    times = np.where(np.isfinite(lengths), times, np.inf)
    return times, lengths, idx.astype(np.int8)


# ---------------------------------------------------------------------------
# 單條路徑建構
# ---------------------------------------------------------------------------
def _build_segments(start: Pose, end: Pose, word: str, t: float, p: float, q: float,
                    R_m: float) -> tuple[DubinsSegment, ...]:
    """由 (t, p, q) 自 ``start`` 前向積分建立三段；末段終點貼齊為 ``end``（要求值）。"""
    segs: list[DubinsSegment] = []
    x, y, th = start.x_m, start.y_m, start.theta_rad
    vals = (t, p, q)
    for k, kind in enumerate(word):
        val = float(vals[k])
        sx, sy, sth = x, y, th
        if kind == "S":
            L = val * R_m
            x = sx + L * math.sin(sth)
            y = sy + L * math.cos(sth)
            center = None
            dth = 0.0
        else:
            sign = 1.0 if kind == "R" else -1.0
            dth = sign * val
            L = R_m * val
            cx = sx + sign * R_m * math.cos(sth)
            cy = sy - sign * R_m * math.sin(sth)
            th = sth + dth
            x = cx - sign * R_m * math.cos(th)
            y = cy + sign * R_m * math.sin(th)
            center = (cx, cy)
        segs.append(DubinsSegment(kind=kind, length_m=L, start=Pose(sx, sy, sth),  # type: ignore[arg-type]
                                  end=Pose(x, y, th), center=center, dtheta_rad=dth))
    # 閉合檢查與貼齊（浮點雜訊 ≪ 1e-6·R）
    tol = 1e-6 * max(R_m, 1.0)
    if math.hypot(x - end.x_m, y - end.y_m) > tol:
        raise RuntimeError(f"dubins word {word} does not close on the goal (err={math.hypot(x - end.x_m, y - end.y_m):.3e} m)")
    dth_err = (th - end.theta_rad + math.pi) % TWO_PI - math.pi
    if abs(dth_err) > 1e-6:
        raise RuntimeError(f"dubins word {word} heading mismatch at goal ({dth_err:.3e} rad)")
    last = segs[-1]
    segs[-1] = DubinsSegment(kind=last.kind, length_m=last.length_m, start=last.start,
                             end=Pose(end.x_m, end.y_m, end.theta_rad), center=last.center,
                             dtheta_rad=last.dtheta_rad)
    return tuple(segs)


def dubins_path(start: Pose, end: Pose, R_m: float, word: str | None = None) -> DubinsPath:
    """單對狀態之 Dubins 路徑（規格 §8.1）；``word=None`` → 6 word 最短。

    指定 ``word`` 不可行時丟 ``ValueError``。回傳路徑之 ``end`` 恰等於傳入之 ``end``
    （x, y, theta 皆為要求值）；``start`` 同。
    """
    if not isinstance(start, Pose):
        start = Pose(*np.asarray(start, dtype=float).reshape(3).tolist())
    if not isinstance(end, Pose):
        end = Pose(*np.asarray(end, dtype=float).reshape(3).tolist())
    lengths, params = dubins_solve_all(start.as_array()[None, :], end.as_array()[None, :], R_m)
    if word is None:
        w = int(np.argmin(lengths[0]))
        if not np.isfinite(lengths[0, w]):
            raise ValueError("no feasible Dubins word")
    else:
        if word not in _WORD_INDEX:
            raise ValueError(f"unknown Dubins word {word!r}; expected one of {WORDS}")
        w = _WORD_INDEX[word]
        if not np.isfinite(lengths[0, w]):
            raise ValueError(f"Dubins word {word} infeasible for this pair")
    t, p, q = (float(v) for v in params[0, w])
    segs = _build_segments(start, end, WORDS[w], t, p, q, R_m)
    return DubinsPath(word=WORDS[w], R_m=float(R_m), segments=segs, length_m=float(lengths[0, w]))


# ---------------------------------------------------------------------------
# 離散化（規格 §8.5；D27；論文 Eq.28/29）
# ---------------------------------------------------------------------------
def discretize_step_rad(R_m: float, e_max_m: float, psi_max_rad: float) -> float:
    """外切多邊形角步 :math:`\\Delta\\psi=\\min\\{2\\arccos(R/(R+e_{\\max})),\\ \\psi_{\\max}\\}`（D27）。

    A（R=200，e=3）→ 19.8°、B（R=120）→ 25.6°。
    """
    if not e_max_m > 0:
        raise ValueError("e_max_m must be > 0")
    if not psi_max_rad > 0:
        raise ValueError("psi_max_rad must be > 0")
    return min(2.0 * math.acos(R_m / (R_m + e_max_m)), psi_max_rad)


def discretize(path: DubinsPath, e_max_m: float, psi_max_rad: float) -> list[tuple[float, float, float]]:
    """外切多邊形航點 ``[(x_m, y_m, theta_rad)]``（論文 Eq.28 切點、Eq.29 相鄰切線交點）。

    每段弧以 :math:`n=\\lceil|\\Delta\\theta|/\\Delta\\psi\\rceil` 等分（實際角步 :math:`\\delta\\le\\Delta\\psi`）；
    航點為相鄰切線之交點，位於圓心徑向距離 :math:`R/\\cos(\\delta/2)`，故對弧之最大橫向偏差
    :math:`R(\\sec(\\delta/2)-1)\\le e_{\\max}`，且多邊形全程在圓外側（外切）。
    航點附帶之 ``theta`` 為該點出發之航段方位（弧上下一切點之切線方位）；首點為 ``path.start``、
    末點為 ``path.end``；直線段只輸出終點。零長度段略過。
    """
    R = path.R_m
    dpsi = discretize_step_rad(R, e_max_m, psi_max_rad)
    st = path.start
    pts: list[tuple[float, float, float]] = [(st.x_m, st.y_m, st.theta_rad)]
    for seg in path.segments:
        if seg.length_m <= _ZERO_LEN:
            continue
        if seg.kind == "S":
            pts.append((seg.end.x_m, seg.end.y_m, seg.end.theta_rad))
            continue
        assert seg.center is not None
        sign = 1.0 if seg.kind == "R" else -1.0
        mag = abs(seg.dtheta_rad)
        n = max(1, int(math.ceil(mag / dpsi - 1e-12)))
        delta = mag / n
        rho = R / math.cos(0.5 * delta)
        cx, cy = seg.center
        th0 = seg.start.theta_rad
        for j in range(n):
            th_mid = th0 + sign * (j + 0.5) * delta          # 切線交點之徑向方位
            th_wp = th0 + sign * (j + 1.0) * delta           # 出發航段方位（下一切點之切線）
            vx = cx - sign * rho * math.cos(th_mid)
            vy = cy + sign * rho * math.sin(th_mid)
            pts.append((vx, vy, th_wp))
        pts.append((seg.end.x_m, seg.end.y_m, seg.end.theta_rad))
    return pts


# ---------------------------------------------------------------------------
# 風中時間 / 能量（規格 §8.1；D14）
# ---------------------------------------------------------------------------
def path_time(path: DubinsPath, v_air_mps: float, wind: Any, z_m: float,
              step_rad: float = 0.0873) -> float:
    """路徑時間（規格 §8.1 式；D14）：直線 :math:`L/V_g(\\theta)`、弧 :math:`\\sum_j R\\delta/V_g(\\theta_j)`。

    ``wind``：``WindField``（鴨子型別，需 ``wind_vec(z) -> (N,2)``）、(2,) 風向量或 None（無風）；
    ``z_m`` 為 ENU z（路徑視為等高）。弧段以中點 Riemann 和、步長 ≤ ``step_rad``（預設 5°）。
    任一處 :math:`V_g\\le0`（含 :math:`|W_\\perp|\\ge V_a`）→ inf。
    """
    if not v_air_mps > 0:
        raise ValueError("v_air_mps must be > 0")
    if not step_rad > 0:
        raise ValueError("step_rad must be > 0")
    w = _wind_xy_of(wind, z_m)
    total = 0.0
    for seg in path.segments:
        if seg.length_m <= 0.0:
            continue
        if seg.kind == "S":
            tt = float(_line_time_vec(np.array([seg.start.theta_rad]), np.array([seg.length_m]),
                                      v_air_mps, w)[0])
        else:
            tt = float(_arc_time_vec(np.array([seg.start.theta_rad]), np.array([seg.dtheta_rad]),
                                     path.R_m, v_air_mps, w, step_rad)[0])
        if not np.isfinite(tt):
            return math.inf
        total += tt
    return float(total)


def _cruise_mode() -> Any:
    """取得 ``mission.Mode.CRUISE``（延遲 import；失敗時回傳字串 ``"CRUISE"``）。"""
    try:
        from .mission import Mode  # type: ignore[import-not-found]
        return Mode.CRUISE
    except Exception:  # pragma: no cover - mission 尚未存在時
        return "CRUISE"


def path_time_and_energy(path: DubinsPath, v_air_mps: float, wind: Any, z_m: float, energy: Any,
                         gamma_rad: float = 0.0) -> tuple[float, float]:
    """``(t_s, e_wh)``：時間同 ``path_time``；能量 = ``energy.segment_wh(Mode.CRUISE, t, gamma_rad)``（規格 §5.2）。

    ``energy`` 為 ``platform.EnergyModel``（鴨子型別）。時間為 inf 時能量亦 inf。
    """
    t = path_time(path, v_air_mps, wind, z_m)
    if not math.isfinite(t):
        return math.inf, math.inf
    e = float(energy.segment_wh(_cruise_mode(), t, gamma_rad))
    return t, e


# ---------------------------------------------------------------------------
# 閉式（規格 §8.3；D5）— 僅測試與 §7.3 代理
# ---------------------------------------------------------------------------
def omega_length(s_m: float, R_m: float) -> float:
    """Ω／燈泡（RLR/LRL）反向轉場長度 :math:`R(\\pi+4\\arccos\\frac{s+2R}{4R})`，:math:`s<2R`（D5）。

    :math:`s=0` → 7.33R；:math:`s=2R` → πR。:math:`s>2R` 時 arccos 引數截至 1（回 πR，供代理取 min 用）。
    """
    c = (s_m + 2.0 * R_m) / (4.0 * R_m)
    c = min(1.0, max(-1.0, c))
    return R_m * (math.pi + 4.0 * math.acos(c))


def uturn_length(s_m: float, R_m: float) -> float:
    """U 型（LSL/RSR）反向轉場長度 :math:`\\pi R+(s-2R)`，:math:`s\\ge2R`（D5）。

    :math:`s<2R` 時 LSL/RSR 退化為環型，回傳 ``loop_length``（同組態之真實 LSL/RSR 長度）。
    """
    if s_m >= 2.0 * R_m:
        return math.pi * R_m + (s_m - 2.0 * R_m)
    return 3.0 * math.pi * R_m + (2.0 * R_m - s_m)


def loop_length(s_m: float, R_m: float) -> float:
    """環型（LSL/RSR 於 :math:`s<2R`）長度 :math:`3\\pi R+(2R-s)`（D5）；:math:`s=0` → 11.42R。

    :math:`s\\ge2R` 時回傳 U 型 :math:`\\pi R+(s-2R)`（同組態之真實 LSL/RSR 長度）。
    """
    if s_m < 2.0 * R_m:
        return 3.0 * math.pi * R_m + (2.0 * R_m - s_m)
    return math.pi * R_m + (s_m - 2.0 * R_m)


def skip_k(s_m: float, R_m: float) -> int:
    """skip-k：:math:`k=\\lceil 2R/s\\rceil`（規格 §8.3）。A（s=168.05, R=200）→ 3。"""
    if not s_m > 0:
        raise ValueError("s_m must be > 0")
    return max(1, int(math.ceil(2.0 * R_m / s_m - 1e-9)))


def parallel_transfer_pose_pair(p_end: np.ndarray, theta_out: float, s_m: float,
                                same_direction: bool = False, strip_len_m: float = 0.0) -> tuple[Pose, Pose]:
    """平行條帶轉場之起訖狀態（測試用；規格 §8.2 四狀態）。

    起點 = 掃完條帶之端點狀態 ``(p_end, theta_out)``（如 :math:`S^{LR}_l`）；目標條帶沿右法向
    :math:`\\hat n(\\theta_{out})` 側移 ``s_m``（負值 = 左側）。

    * ``same_direction=False``（反向，論文原型）：終點 = ``(p_end + s·n̂, theta_out + π)``（:math:`S^R_{l'}`）。
    * ``same_direction=True``（同向 racetrack，D34）：終點在對側端點
      ``(p_end + s·n̂ − strip_len·t̂, theta_out)``（:math:`S^L_{l'}`）。
    """
    p = np.asarray(p_end, dtype=float).reshape(2)
    n_hat = np.array([math.cos(theta_out), -math.sin(theta_out)])
    t_hat = np.array([math.sin(theta_out), math.cos(theta_out)])
    start = Pose(float(p[0]), float(p[1]), float(theta_out))
    if same_direction:
        q = p + s_m * n_hat - strip_len_m * t_hat
        end = Pose(float(q[0]), float(q[1]), float(theta_out))
    else:
        q = p + s_m * n_hat
        end = Pose(float(q[0]), float(q[1]), float(theta_out) + math.pi)
    return start, end
