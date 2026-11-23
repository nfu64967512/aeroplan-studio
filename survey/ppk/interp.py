"""survey/ppk/interp.py — 在曝光時刻內插 PPK 軌跡，再加槓桿臂得到相機中心。

內插方法：預設三次 Hermite（ECEF 座標），切線用 .pos 的速度欄位（out-outvel=on），
沒有速度欄位時用相鄰兩個 epoch 的中央差分（非等距 Catmull-Rom）。理由與量化：
- 10 Hz、20 m/s 時相鄰 epoch 相距 2 m。直線等速段線性內插誤差為 0；
  沿航跡加速度 a 的誤差上限 a·dt²/8（1 m/s² ⇒ 1.25 mm）；
  轉彎半徑 R 的弦高誤差 v²dt²/(8R)：30° 坡度 R≈70.6 m ⇒ 7.1 mm，15° R≈152 m ⇒ 3.3 mm。
  每帶前幾張常落在轉彎改出（CAM_MAX_ROLL 只擋 >10–15°），這個量級與 F9P RTK 規格
  （1 cm + 1 ppm）同級，不能忽略。測試 tests/test_survey_ppk_interp.py 實測兩種方法的誤差。
- Hermite 在同一條軌跡上把誤差壓到 0.1 mm 級（實測：中央差分切線全程最大 0.10 mm；用速度欄位
  0.085 mm）。例外是直線接轉彎這種曲率不連續點，中央差分切線在兩側約 1.8 mm（仍小於線性的 7 mm；
  真實飛機滾轉進入約 1 s，曲率連續）。代價是位置雜訊經差分切線放大，1 cm 的 epoch 雜訊在 u=1/3
  處約多 1 mm（h10 最大 4/27 × dt × σ·√2/(2dt)）。
- 相鄰 epoch 間隔超過 ``max_gap_s``（預設 0.25 s ＝ 10 Hz 下漏了 ≥2 個 epoch）時一律改線性並標
  ``ppk_gap``；Hermite 拿遠處的點估切線會在缺口裡亂彎。

σ 傳播：相鄰 epoch 的 PPK 誤差高度相關（同一組整數模糊度），所以 σ 用線性內插（完全相關的上界）
而不是 √((1−u)²σ0²+u²σ1²)（獨立假設，會在中點低估 29%）。另可加入時間不確定度（速度×σ_t）與
姿態不確定度（|槓桿臂|×σ_att），都以平方和加上。

槓桿臂：相機中心 = 天線 + R_nb(yaw,pitch,roll)·lever_FRD（geodesy.body_to_ned），在天線點的 NED
展開後轉 ECEF 相加。姿態由呼叫端提供（FC ATT 經 FC↔Pi↔GPST 時鐘鏈內插到同一個曝光時刻）。
**有槓桿臂卻沒有姿態就丟錯**，不會靜默只輸出天線位置；槓桿臂未知（None）則輸出天線位置並標
``lever_arm_unknown``。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from survey.ppk.geodesy import (body_to_ned, ecef_to_llh, ecef_to_ned_delta, llh_to_ecef,
                                ned_to_ecef_delta)
from survey.ppk.pos import Q_FIX, Q_FLOAT, PosTrajectory

__all__ = ["MAX_GAP_S", "InterpResult", "interpolate_at"]

MAX_GAP_S = 0.25


@dataclass
class InterpResult:
    """每個事件一列；位置無法求得時為 NaN 並帶對應旗標。"""
    t_gpst_s: np.ndarray
    lat_deg: np.ndarray           # 相機中心（已加槓桿臂；槓桿臂未知時＝天線）
    lon_deg: np.ndarray
    h_ell_m: np.ndarray
    ant_lat_deg: np.ndarray       # 天線（內插結果，未加槓桿臂）
    ant_lon_deg: np.ndarray
    ant_h_ell_m: np.ndarray
    vel_ned_mps: np.ndarray       # (M,3) 內插處的速度
    sigma_h_m: np.ndarray
    sigma_v_m: np.ndarray
    q_before: np.ndarray          # 前後兩個 epoch 的 Q（0 = 範圍外）
    q_after: np.ndarray
    bracket_s: np.ndarray         # 前後 epoch 的間隔
    flags: list[tuple[str, ...]]
    method: str
    max_gap_s: float
    lever_arm_frd_m: tuple[float, float, float] | None

    def __len__(self) -> int:
        return int(self.t_gpst_s.size)

    @property
    def cog_deg(self) -> np.ndarray:
        """對地航跡角（真北順時針，0–360）。"""
        return np.degrees(np.arctan2(self.vel_ned_mps[:, 1], self.vel_ned_mps[:, 0])) % 360.0


def _tangents(t: np.ndarray, p: np.ndarray, max_gap_s: float) -> np.ndarray:
    """切線：兩側都在 max_gap 內用非等距中央差分；只剩一側時用同側三點（二次多項式）的單側差分，
    同側只有一點才退成兩點差分（一階，轉彎中可達數 mm，見測試）；一點都沒有為 NaN。"""
    n = len(t)
    m = np.full_like(p, np.nan)
    if n < 2:
        return m
    dt = np.diff(t)
    ok = dt <= max_gap_s + 1e-9
    left_ok = np.zeros(n, bool)
    right_ok = np.zeros(n, bool)
    left_ok[1:] = ok
    right_ok[:-1] = ok
    left2 = np.zeros(n, bool)
    right2 = np.zeros(n, bool)
    left2[2:] = ok[1:] & ok[:-1]
    right2[:-2] = ok[:-1] & ok[1:]
    both = left_ok & right_ok
    i = np.nonzero(both)[0]
    m[i] = (p[i + 1] - p[i - 1]) / (t[i + 1] - t[i - 1])[:, None]

    def three(i, a, b):
        # 經過 (t_i, t_a, t_b) 的二次 Lagrange 多項式在 t_i 的導數
        t0, t1, t2 = t[i][:, None], t[a][:, None], t[b][:, None]
        return (p[i] * (2 * t0 - t1 - t2) / ((t0 - t1) * (t0 - t2))
                + p[a] * (t0 - t2) / ((t1 - t0) * (t1 - t2))
                + p[b] * (t0 - t1) / ((t2 - t0) * (t2 - t1)))

    i = np.nonzero(~both & right2)[0]
    m[i] = three(i, i + 1, i + 2)
    i = np.nonzero(~both & ~right2 & left2)[0]
    m[i] = three(i, i - 1, i - 2)
    i = np.nonzero(~both & ~right2 & ~left2 & right_ok)[0]
    m[i] = (p[i + 1] - p[i]) / (t[i + 1] - t[i])[:, None]
    i = np.nonzero(~both & ~right2 & ~left2 & left_ok)[0]
    m[i] = (p[i] - p[i - 1]) / (t[i] - t[i - 1])[:, None]
    return m


def interpolate_at(
    traj: PosTrajectory,
    t_events,
    *,
    method: str = "hermite",
    max_gap_s: float = MAX_GAP_S,
    lever_arm_frd_m=None,
    attitude_deg=None,
    attitude_sigma_deg: float = 0.0,
    time_sigma_s=None,
) -> InterpResult:
    """在 ``t_events``（GPST 秒）內插軌跡。

    attitude_deg：(M,3) 的 [yaw, pitch, roll]（度，yaw 自真北順時針），與 t_events 對齊；
    time_sigma_s：純量或長度 M 的陣列（曝光時刻本身的不確定度）。
    """
    if method not in ("hermite", "linear"):
        raise ValueError(f"method 只能是 'hermite' 或 'linear'，收到 {method!r}")
    if not (max_gap_s > 0):
        raise ValueError(f"max_gap_s 必須 > 0，收到 {max_gap_s!r}")
    n = len(traj)
    if n < 2:
        raise ValueError("軌跡少於兩個 epoch，無法內插")
    te = np.atleast_1d(np.asarray(t_events, dtype=float))
    m_ev = te.size
    T = traj.t_gpst_s
    P = llh_to_ecef(traj.lat_deg, traj.lon_deg, traj.h_ell_m)

    lever = None
    if lever_arm_frd_m is not None:
        lever = np.asarray(lever_arm_frd_m, dtype=float).reshape(3)
        if not np.all(np.isfinite(lever)):
            raise ValueError(f"槓桿臂含非有限值：{lever_arm_frd_m!r}")
    att = None
    if lever is not None and np.any(lever != 0.0):
        if attitude_deg is None:
            raise ValueError("給了非零槓桿臂卻沒有姿態 —— 無法把機身座標的槓桿臂轉到地面座標")
        att = np.asarray(attitude_deg, dtype=float).reshape(-1, 3)
        if att.shape[0] != m_ev:
            raise ValueError(f"姿態筆數 {att.shape[0]} 與事件數 {m_ev} 不符")
    if time_sigma_s is None:
        ts = np.zeros(m_ev)
    else:
        ts = np.broadcast_to(np.asarray(time_sigma_s, dtype=float), (m_ev,)).astype(float)

    k = np.searchsorted(T, te, side="right") - 1
    k = np.clip(k, 0, n - 2)
    inside = np.isfinite(te) & (te >= T[0]) & (te <= T[-1])
    t0, t1 = T[k], T[k + 1]
    dt = t1 - t0
    u = np.where(inside, (te - t0) / dt, np.nan)
    gap = dt > max_gap_s + 1e-9

    p0, p1 = P[k], P[k + 1]
    pos = (1.0 - u)[:, None] * p0 + u[:, None] * p1
    vel = (p1 - p0) / dt[:, None]
    used_linear = np.ones(m_ev, bool)
    if method == "hermite":
        if traj.vel_ned_mps is not None:
            M = ned_to_ecef_delta(traj.lat_deg, traj.lon_deg, traj.vel_ned_mps)
        else:
            M = _tangents(T, P, max_gap_s)
        m0, m1 = M[k], M[k + 1]
        herm = inside & ~gap & np.all(np.isfinite(m0), axis=1) & np.all(np.isfinite(m1), axis=1)
        if np.any(herm):
            uu = u[herm]
            d = dt[herm][:, None]
            u2, u3 = uu * uu, uu * uu * uu
            h00, h10 = 2 * u3 - 3 * u2 + 1, u3 - 2 * u2 + uu
            h01, h11 = -2 * u3 + 3 * u2, u3 - u2
            a0, a1, b0, b1 = p0[herm], p1[herm], m0[herm], m1[herm]
            pos[herm] = (h00[:, None] * a0 + h10[:, None] * d * b0
                         + h01[:, None] * a1 + h11[:, None] * d * b1)
            g00, g10 = 6 * u2 - 6 * uu, 3 * u2 - 4 * uu + 1
            g01, g11 = -6 * u2 + 6 * uu, 3 * u2 - 2 * uu
            vel[herm] = (g00[:, None] * a0 / d + g10[:, None] * b0
                         + g01[:, None] * a1 / d + g11[:, None] * b1)
            used_linear[herm] = False

    pos[~inside] = np.nan
    vel[~inside] = np.nan
    a_lat, a_lon, a_h = ecef_to_llh(pos)
    vel_ned = ecef_to_ned_delta(a_lat, a_lon, vel)

    sh = traj.sigma_h_m
    sv = traj.sigma_v_m
    sig_h = (1.0 - u) * sh[k] + u * sh[k + 1]
    sig_v = (1.0 - u) * sv[k] + u * sv[k + 1]
    sig_h = np.sqrt(sig_h ** 2 + (np.hypot(vel_ned[:, 0], vel_ned[:, 1]) * ts) ** 2)
    sig_v = np.sqrt(sig_v ** 2 + (vel_ned[:, 2] * ts) ** 2)

    cam = pos.copy()
    no_att = np.zeros(m_ev, bool)
    if att is not None:
        d_ned = body_to_ned(att[:, 0], att[:, 1], att[:, 2], np.broadcast_to(lever, (m_ev, 3)))
        no_att = ~np.all(np.isfinite(att), axis=1)
        cam = pos + ned_to_ecef_delta(a_lat, a_lon, d_ned)
        cam[no_att] = np.nan
        s_att = float(np.linalg.norm(lever)) * np.radians(float(attitude_sigma_deg))
        sig_h = np.sqrt(sig_h ** 2 + s_att ** 2)
        sig_v = np.sqrt(sig_v ** 2 + s_att ** 2)
    c_lat, c_lon, c_h = ecef_to_llh(cam)

    q0 = np.where(inside, traj.q[k], 0)
    q1 = np.where(inside, traj.q[k + 1], 0)
    flags: list[tuple[str, ...]] = []
    for i in range(m_ev):
        f: list[str] = []
        if not inside[i]:
            f.append("ppk_out_of_range")
        else:
            if gap[i]:
                f.append("ppk_gap")
            if q0[i] != Q_FIX or q1[i] != Q_FIX:
                f.append("ppk_float")
                for q in sorted({int(q0[i]), int(q1[i])} - {Q_FIX, Q_FLOAT}):
                    f.append(f"ppk_q{q}")
            if method == "hermite" and used_linear[i]:
                f.append("interp_linear")
        if lever is None:
            f.append("lever_arm_unknown")
        if no_att[i]:
            f.append("no_attitude")
        flags.append(tuple(f))

    return InterpResult(
        t_gpst_s=te, lat_deg=c_lat, lon_deg=c_lon, h_ell_m=c_h,
        ant_lat_deg=a_lat, ant_lon_deg=a_lon, ant_h_ell_m=a_h,
        vel_ned_mps=vel_ned, sigma_h_m=sig_h, sigma_v_m=sig_v,
        q_before=q0, q_after=q1, bracket_s=np.where(inside, dt, np.nan), flags=flags,
        method=method, max_gap_s=float(max_gap_s),
        lever_arm_frd_m=None if lever is None else (float(lever[0]), float(lever[1]), float(lever[2])),
    )
