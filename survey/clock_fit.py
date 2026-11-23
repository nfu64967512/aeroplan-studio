"""飛控觸發事件 ↔ 樹莓派 GPIO 邊緣：逐張配對與時鐘擬合 t_fc = a·t_pi + b。

為什麼不能依序配對
==================
同一個觸發邊緣，飛控記一筆 CAM（TimeUS），樹莓派記一筆 edge_ns。只要任一邊少一筆或多一筆
（Pi 漏邊緣、雜訊多一個邊緣、LOG_DISARMED=0 時地面試拍飛控不記…），依序配對就會從那裡起
全部錯一張 —— 在 21.5 m/s、18.85 m 間距下就是每張位姿錯 19 m，而且看起來一切正常。
Mission Planner 遇到張數不符直接中止（評估 §7）；我們要的是**指出到底哪幾筆沒配上**。

做法
====
1. 間隔特徵：每個事件取「前一個間隔、後一個間隔」。兩台時鐘的漂移只有數十 ppm，1 s 的間隔
   只差 0.1 ms，所以真正對應的事件特徵幾乎相同；轉彎的長間隔更是獨一無二。特徵相符的
   (i, j) 才當候選。
2. 斜率網格 × 偏移投票（Hough）：漂移在 ±max_drift_ppm 內掃斜率，每個斜率對候選的
   t_fc − s·t_pi 做直方圖，票數最多者為初值。距離觸發的帶內間隔被 50 Hz 量化成 20 ms 的
   倍數，「錯一張」的對齊也會拿到不少票 ⇒ 取前幾名分別精修，比配對數與殘差；
   最佳對齊多配的對數少於 ``ambiguity_margin`` 且殘差相當時直接拒絕（``ClockFitError``），不猜。
3. 精修：用模型預測，對每個 Pi 事件找最近的飛控事件，**雙向最近**且殘差 ≤ match_tol 才配對
   （一對一）；穩健最小平方更新 a、b，反覆到配對不再變。殘差超過 max(outlier_sigma·σ_MAD,
   min_outlier_s) 的配對仍保留在配對表，但不進擬合（``inlier`` 為假）。

輸出的 ``unmatched_fc``＝飛控有觸發、Pi 沒看到邊緣（少拍）；``unmatched_pi``＝Pi 有邊緣、
飛控沒有對應事件（多出的邊緣或飛控沒記）。索引都是呼叫端傳入陣列的原始索引。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


class ClockFitError(ValueError):
    """事件太少、對不齊、或對齊有歧義。"""


@dataclass
class ClockFit:
    a: float                      # 斜率（無單位，1 + 漂移）
    b: float                      # 偏移（s），相對於兩邊參考時刻
    t_fc_ref_s: float
    t_pi_ref_s: float
    pairs: np.ndarray             # (n, 2) int：(fc 索引, pi 索引)，依 fc 時間排序
    residuals_s: np.ndarray       # 每一對的 t_fc − 模型（s）
    inlier: np.ndarray            # bool，進擬合的配對
    unmatched_fc: np.ndarray      # int，原始索引
    unmatched_pi: np.ndarray
    ambiguity: float              # 次佳對齊的配對數／最佳配對數（越接近 1 越危險）
    params: dict = field(default_factory=dict)

    def pi_to_fc(self, t_pi_s):
        """Pi 時鐘（s，BOOTTIME）→ 飛控時鐘（s，自開機）。"""
        return self.t_fc_ref_s + self.a * (np.asarray(t_pi_s, float) - self.t_pi_ref_s) + self.b

    @property
    def drift_ppm(self) -> float:
        return (self.a - 1.0) * 1e6

    @property
    def n_pairs(self) -> int:
        return int(len(self.pairs))

    @property
    def n_inliers(self) -> int:
        return int(self.inlier.sum())

    @property
    def rms_s(self) -> float:
        r = self.residuals_s[self.inlier]
        return float(np.sqrt(np.mean(r ** 2))) if r.size else float("nan")

    @property
    def max_abs_s(self) -> float:
        r = self.residuals_s[self.inlier]
        return float(np.max(np.abs(r))) if r.size else float("nan")

    def summary(self) -> dict:
        return {
            "a": self.a, "drift_ppm": self.drift_ppm, "n_pairs": self.n_pairs, "n_inliers": self.n_inliers,
            "rms_ms": self.rms_s * 1e3, "max_abs_ms": self.max_abs_s * 1e3,
            "n_unmatched_fc": int(len(self.unmatched_fc)), "n_unmatched_pi": int(len(self.unmatched_pi)),
            "ambiguity": self.ambiguity,
        }


def _prepare(t, name):
    t = np.asarray(t, float)
    if t.ndim != 1:
        raise ClockFitError(f"{name} 必須是一維陣列")
    if not np.all(np.isfinite(t)):
        raise ClockFitError(f"{name} 含非有限值")
    order = np.argsort(t, kind="stable")
    ts = t[order]
    if ts.size > 1 and np.any(np.diff(ts) <= 0):
        raise ClockFitError(f"{name} 有重複時刻（同一事件記兩次？）")
    return ts, order


def _signature_candidates(f, p, sig_tol_s, drift):
    """前後間隔都對得上的 (i, j)。端點只有單邊間隔時，只比那一邊。"""
    nan = np.array([np.nan])
    fp, fn = np.concatenate([nan, np.diff(f)]), np.concatenate([np.diff(f), nan])
    pp, pn = np.concatenate([nan, np.diff(p)]), np.concatenate([np.diff(p), nan])
    ii, jj = [], []
    chunk = max(1, 4_000_000 // max(len(p), 1))
    with np.errstate(invalid="ignore"):
        for s in range(0, len(f), chunk):
            a_p, a_n = fp[s:s + chunk, None], fn[s:s + chunk, None]
            m_p = np.abs(a_p - pp[None, :]) <= sig_tol_s + drift * pp[None, :]
            m_n = np.abs(a_n - pn[None, :]) <= sig_tol_s + drift * pn[None, :]
            u_p = np.isnan(a_p) | np.isnan(pp)[None, :]
            u_n = np.isnan(a_n) | np.isnan(pn)[None, :]
            ok = (m_p | u_p) & (m_n | u_n) & (m_p | m_n)
            i, j = np.nonzero(ok)
            ii.append(i + s)
            jj.append(j)
    return np.concatenate(ii), np.concatenate(jj)


def _hough(f, p, ii, jj, vote_tol_s, max_drift, top_k, min_sep_s):
    """回傳前 top_k 個 (票數, 斜率, 偏移)，彼此偏移至少相隔 min_sep_s。"""
    span = max(float(f[-1]), float(p[-1]), 1.0)
    step = vote_tol_s / span
    n = int(np.ceil(max_drift / step))
    slopes = 1.0 + np.arange(-n, n + 1) * step
    best = []                                   # (score, slope, offset)
    for s in slopes:
        off = f[ii] - s * p[jj]
        lo = off.min()
        k = np.floor((off - lo) / vote_tol_s).astype(np.int64)
        cnt = np.bincount(k)
        win = cnt.copy()
        win[:-1] += cnt[1:]                     # 相鄰兩格合計，避免真值剛好落在格界
        for kb in np.argsort(win)[::-1][:top_k * 3]:
            best.append((int(win[kb]), float(s), float(lo + (kb + 1) * vote_tol_s)))
    best.sort(key=lambda x: -x[0])
    picked = []
    for sc, s, b in best:
        if all(abs(b - pb) >= min_sep_s for _, _, pb in picked):
            picked.append((sc, s, b))
        if len(picked) >= top_k:
            break
    return picked


def _nearest(sorted_x, q):
    """q 中每個值在 sorted_x 的最近索引。"""
    k = np.searchsorted(sorted_x, q)
    k0 = np.clip(k - 1, 0, len(sorted_x) - 1)
    k1 = np.clip(k, 0, len(sorted_x) - 1)
    return np.where(np.abs(sorted_x[k1] - q) < np.abs(sorted_x[k0] - q), k1, k0)


def _robust_line(x, y, outlier_sigma, min_outlier_s, x0):
    """最小平方擬合 y = a·x + b，反覆剔除殘差 > max(k·σ_MAD, 下限) 的點。
    全部都被剔除時（錯誤的對齊候選）回傳全假的 keep，a、b 保留上一輪的有限值。"""
    keep = np.ones(x.size, bool)
    a, b = 1.0, float(np.median(y - x))
    for _ in range(10):
        if keep.sum() >= 2:
            a, b0 = np.polyfit(x[keep] - x0, y[keep], 1)
            b = b0 - a * x0
        else:
            a, b = 1.0, float(np.median(y[keep] - x[keep]))
        r = y - (a * x + b)
        mad = np.median(np.abs(r[keep] - np.median(r[keep])))
        new = np.abs(r) <= max(outlier_sigma * 1.4826 * mad, min_outlier_s)
        if not new.any():
            keep = new
            break
        if (new == keep).all():
            break
        keep = new
    return float(a), float(b), keep


def _refine(f, p, a, b, match_tol_s, outlier_sigma, min_outlier_s, max_iter=20):
    pairs = None
    keep = None
    x0 = float(np.median(p))
    for _ in range(max_iter):
        pred = a * p + b
        nn_f = _nearest(f, pred)                    # 每個 Pi 事件最近的飛控事件
        nn_p = _nearest(a * p + b, f)               # 每個飛控事件最近的（預測後）Pi 事件
        j = np.arange(len(p))
        mutual = nn_p[nn_f] == j
        r = f[nn_f] - pred
        ok = mutual & (np.abs(r) <= match_tol_s)
        new_pairs = np.stack([nn_f[ok], j[ok]], axis=1)
        if new_pairs.shape[0] < 2:
            return new_pairs, a, b, np.zeros(len(new_pairs), bool)
        a, b, keep = _robust_line(p[new_pairs[:, 1]], f[new_pairs[:, 0]], outlier_sigma, min_outlier_s, x0)
        if pairs is not None and pairs.shape == new_pairs.shape and (pairs == new_pairs).all():
            break
        pairs = new_pairs
    return pairs, a, b, keep


def fit_clock(t_fc_s, t_pi_s, *, match_tol_s: float = 0.02, max_drift_ppm: float = 500.0,
              sig_tol_s: float = 0.003, vote_tol_s: float = 0.02, outlier_sigma: float = 5.0,
              min_outlier_s: float = 0.001, top_k: int = 3, ambiguity_margin: int = 1) -> ClockFit:
    """配對並擬合。``t_fc_s``：飛控事件（s，自開機）；``t_pi_s``：Pi 邊緣（s，BOOTTIME）。

    ``match_tol_s`` 必須小於中位拍照間隔的一半（預設 20 ms，拍照間隔 ≥0.5 s）。
    ``ambiguity_margin``：最佳對齊至少要比另一種（殘差相當的）對齊多配幾對，否則拒絕。
    """
    f_abs, f_order = _prepare(t_fc_s, "t_fc_s")
    p_abs, p_order = _prepare(t_pi_s, "t_pi_s")
    if len(f_abs) < 3 or len(p_abs) < 3:
        raise ClockFitError(f"事件太少（飛控 {len(f_abs)}、Pi {len(p_abs)}），至少各 3 筆")
    # 用中位間隔判斷（雜訊造成的近距離多餘邊緣交給雙向最近處理，不在這裡擋）
    med_int = float(min(np.median(np.diff(f_abs)), np.median(np.diff(p_abs))))
    if match_tol_s >= med_int / 2:
        raise ClockFitError(f"match_tol_s={match_tol_s} s 不小於中位拍照間隔的一半（{med_int / 2:.4f} s），配對會混淆")
    f0, p0 = float(f_abs[0]), float(p_abs[0])
    f, p = f_abs - f0, p_abs - p0
    drift = max_drift_ppm * 1e-6

    ii, jj = _signature_candidates(f, p, sig_tol_s, drift)
    if ii.size == 0:
        raise ClockFitError("兩邊事件的間隔序列完全對不上（不是同一架次？）")
    sep = max(0.25 * float(np.median(np.diff(f))), 4 * vote_tol_s)
    cands = _hough(f, p, ii, jj, vote_tol_s, drift, top_k, sep)

    results = []
    for _, s, b in cands:
        # Hough 初值只準到 vote_tol；先用該格內的候選對做一次穩健擬合，再全域精修
        near = np.abs(f[ii] - (s * p[jj] + b)) <= 2 * vote_tol_s
        if near.sum() >= 2:
            s, b, _ = _robust_line(p[jj[near]], f[ii[near]], outlier_sigma, min_outlier_s, float(np.median(p)))
        pairs, a1, b1, keep = _refine(f, p, s, b, match_tol_s, outlier_sigma, min_outlier_s)
        if len(pairs) < 3:
            continue
        r = f[pairs[:, 0]] - (a1 * p[pairs[:, 1]] + b1)
        rms = float(np.sqrt(np.mean(r[keep] ** 2))) if keep.any() else float("inf")
        results.append((int(keep.sum()), -rms, pairs, a1, b1, keep, r))
    if not results:
        raise ClockFitError("找不到至少 3 對一致的事件")
    results.sort(key=lambda x: (x[0], x[1]), reverse=True)
    n_best = results[0][0]
    # 只算真正不同的對齊（同一組配對被兩個初值各找到一次不算歧義）
    others = [x for x in results[1:] if not (x[2].shape == results[0][2].shape and (x[2] == results[0][2]).all())]
    ambiguity = (others[0][0] / n_best) if others else 0.0
    # 用「多配了幾對」判斷，不用比例：長帶少轉彎的架次比例會很接近 1。兩邊張數固定時，少配 1 對
    # 等於多出 2 個未配對事件（Pi 少一個＋多一個）⇒ 預設只在兩種對齊配對數相同（真正的平手）時拒絕
    if others and n_best - others[0][0] < ambiguity_margin and others[0][1] >= results[0][1] - 1e-4:
        raise ClockFitError(f"對齊有歧義：兩種對齊各有 {n_best} 與 {others[0][0]} 對且殘差相當"
                            "（事件間隔過於規律？請檢查轉彎間隔是否被記錄）")
    _, _, pairs, a, b, keep, r = results[0]

    order = np.argsort(pairs[:, 0])
    pairs, keep, r = pairs[order], keep[order], r[order]
    orig = np.stack([f_order[pairs[:, 0]], p_order[pairs[:, 1]]], axis=1)
    un_f = np.setdiff1d(np.arange(len(f)), pairs[:, 0])
    un_p = np.setdiff1d(np.arange(len(p)), pairs[:, 1])
    return ClockFit(a=a, b=b, t_fc_ref_s=f0, t_pi_ref_s=p0, pairs=orig, residuals_s=r, inlier=keep,
                    unmatched_fc=np.sort(f_order[un_f]), unmatched_pi=np.sort(p_order[un_p]),
                    ambiguity=float(ambiguity),
                    params=dict(match_tol_s=match_tol_s, max_drift_ppm=max_drift_ppm, sig_tol_s=sig_tol_s,
                                vote_tol_s=vote_tol_s, outlier_sigma=outlier_sigma, min_outlier_s=min_outlier_s,
                                ambiguity_margin=ambiguity_margin))
