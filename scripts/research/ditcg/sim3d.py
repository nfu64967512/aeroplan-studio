#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim3d.py —— 六態(含側向)點質量模擬器,用來建 Guo (2020) DD-ITCG 的 5 維 FSV 表。

## 為什麼要有這一支(不是重造 predictor.py)

`scripts/wsl/sar_swarm_patches/predictor.py` 第 29 行自述:
「縱向平面點質量,**不含側向動態**。橫向由 PN 自己管」。
Guo 的表是 5 維 `(r, z, V, gamma, eps)`,其中 **eps 是側向狀態** ⇒ 缺了側向通道就
建不出表。這一支補上側向,並且**逐項複刻我們真正在飛的那條律**
(`tot_node._pn_cmd` 2597-2707),而不是 Guo 的理想 PNG。

⚠️ 本檔**不修改** `predictor.py` / `scripts/research/predictor/sim.py`。
   `tests/test_predictor_module.py` 會逐點比對那兩支,改了就紅。
   氣動原語(air_density / thrust 律 / 機體常數)一律 **import 復用**,
   確保跟已校準的縱向模型同源(CD0=0.01500,n=1617、sd 0.00058)。

## 狀態(6 個 + 2 個遲滯態)

    r      水平距紅心(m)            —— Guo Eq.(18) 的 r 是**水平**投影,不是斜距
    z      離紅心地表高度(m)        —— 海上紅心 target_amsl = 0
    V      3D 總速(m/s)             —— ⚠️ CSV 的 `gs` 是水平地速 = V cos(gamma)
    gamma  航跡角(rad,**下降為負**)—— 與 tot_node `atan2(hdot, vh)` 同號
    eps    **帶號**水平方位誤差(rad)—— eps>0 = 速度在 LOS 右側
    t      俯衝經過時間(s)

    遲滯態:gdot_act(gamma 角速率的一階遲滯)、phi_act(傾角的一階遲滯)

⚠️⚠️ **eps 的定義採 Guo 的運動學式 Eq.(27)-(29),不採 §II.A 那句口語**。
   口語版「速度向量與 LOS 鉛垂平面的夾角」會得到 eps' = arcsin(cos(gamma)·sin(eps)),
   在 gamma=-49°、eps=45° 時差 **-17.4°**。而 Eq.(27) `eps = psi + theta ± pi/2`
   (兩個**方位角**的線性組合)、Eq.(29) `epsdot = psidot + V cos(gamma) sin(eps)/r`
   只有在「eps = 水平方位差」時才自洽。兩者不一致是原文自己的問題,我們一律取運動學版。
   ⇒ 本檔的 eps **就是** `tot_node._lead_angle_deg` 的量(但**帶號**,不取 abs)。

## 運動方程(在我們自己的座標慣例下重導,不轉抄 Guo 的正負號)

    rdot     = -V cos(gamma) cos(eps)
    zdot     =  V sin(gamma)                        (gamma<0 ⇒ 下降)
    Vdot     = (T - D)/m - g sin(gamma)             (gamma<0 ⇒ 重力**加速**)
    gammadot =  g (n_z - cos(gamma)) / V            (n_z = L cos(phi)/(m g))
    psidot   =  g n_z tan(phi) /(V cos(gamma))      (= L sin(phi)/(m V cos(gamma)))
    epsdot   =  psidot + V cos(gamma) sin(eps) / r  (Guo Eq.29)

⭐ `thetadot_LOS = -V cos(gamma) sin(eps)/r` 與 tot_node 的
   `lam_dot = (de*vn - dn*ve)/d**2` 是**同一個量**(代數已驗:
   de*vn - dn*ve = -cross,而 cross = r * V cos(gamma) * sin(eps))。

## 誘導阻力:唯一相對 predictor.py 改動的一行

    predictor:  c_L = W cos(gamma) /(q S)               ← 準平衡,看不見機動
    本檔:      c_L = m g n_z /(cos(phi) * q S)         ← 由**實際指令升力**驅動

⚠️ **但要誠實**:實算在我們的速度帶幾乎不重要 ——
   V=180、z=2000:n=1 → 誘導阻力佔 c_D0 的 0.26%;n=3 → 2.34%
   V=150、z=3100:n=1 → 0.67%;n=3 → 6.06%(最大)
⇒ **側向對 t_go 的影響 99% 是幾何(航程被 cos(eps) 加長),不是能量。**
   以中位俯衝 41.2 s 為基準,單看 rdot = -V cos(gamma) cos(eps) 這一項:
   eps=10° → +0.64 s;20° → +2.64 s;27° → +5.04 s;45° → +17.07 s。
   `use_load_factor_cl=False` 可以退回 predictor 的準平衡式做對照。

## ⚠️⚠️ 建表的「PNG 基準」:v_req 必須釘死(這是規格選擇,不是實作細節)

現行油門律 `v_req = clip(45, 210, R/t_rem)`,`t_rem = dive_T - now`
**引用絕對時鐘** ⇒ 同一個 `(r,z,V,gamma,eps)` 會因 t_rem 不同而有不同 t_go,
Guo §III.B 的 5 維不變性**直接破功**。而且不是邊角案例:RRE7 放行瞬間
R≈6900、t_rem≈43 ⇒ v_req=161 < V=202 ⇒ thr = 0.36+0.030*(-41) < 0 ⇒ **油門全關**。

本檔提供三個政策(`SimParams.throttle_law`),建表**只准用 "vmax"**:

    "vmax"  v_req == pn_vmax = 210        ← ⭐ 表的 PNG 定義。thr 只是 V 的函數
                                            ⇒ 不變性恢復。物理意義 =「最快可達時間」
                                            ⇒ e_tgo = t_go^req - t_go^PNG **恆 >= 0**,
                                            控制方向只有「拖慢/繞遠」一種,與
                                            `_itcg_bias`「只能延後不能提前」一致,
                                            也與使用者 2026-08-15「降速度保散布」同向。
    "clock" v_req = clip(45,210, R/t_rem),t_rem = t_rem0 - t   ← **重現實測用**
                                            (t_rem0 從 CSV 的
                                             early_by_s + secs_to_target 取)
    "const" v_req = 使用者給的常數        ← 靈敏度掃描用

⚠️ 這個選擇會寫進表的中繼資料。**換了它整張表作廢。**

## 遲滯 tau:必須用實測擬,不可沿用 predictor.PN_TAU_S = 2.0

`predictor.py:206` 自述那個 2.0「**沒有受控實測,是估的**」。
gamma 一旦升為狀態,這個遲滯就直接寫進 t_go^PNG ⇒ 閘門會變成在測這個估值。
`fit_lag.py` 用 RRD/RRE 32 架次的 gamma(t) 重擬,結果回填到本檔的 `TAU_GAMMA_S`。
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, asdict

import numpy as np

# ── 復用已校準的縱向原語(不複製常數,避免兩邊漂移)──────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_PATCHES = os.path.abspath(os.path.join(_HERE, "..", "..", "wsl", "sar_swarm_patches"))
if _PATCHES not in sys.path:
    sys.path.insert(0, _PATCHES)
import predictor as _P                                              # noqa: E402

G = _P.G
MASS_KG = _P.MASS_KG
S_REF_M2 = _P.S_REF_M2
WEIGHT_N = _P.WEIGHT_N
THRUST_FULL_N = _P.THRUST_FULL_N
CD0 = _P.CD0
PI_E_AR = _P._PI_E_AR

# ⚠️ 這裡刻意不拿 `predictor.air_density` / `drag_n` 的**純量**版本去迴圈,
#    而是用 numpy 重寫**同一條式子**(逐項相同,由 selfcheck 對拍驗證)。
#    理由:建表要跑 1e8 級的步數,純 Python 迴圈會慢 2-3 個數量級。


def air_density_np(h_amsl):
    """ISA 密度(kg/m^3)。與 `predictor.air_density` 同式。⚠️ h 必須是 AMSL。"""
    return 1.225 * (1.0 - 2.25577e-5 * np.maximum(h_amsl, 0.0)) ** 4.2586


def thrust_np(v, thr_max_pct, v_req):
    """`tot_node._pn_cmd` 的油門律,再夾 THR_MAX。與 `predictor.thrust_n` 同式。"""
    cmd = np.clip(0.36 + 0.030 * (v_req - v), 0.0, 1.0)
    return np.minimum(cmd, thr_max_pct / 100.0) * THRUST_FULL_N


#: ⚠️ gamma 通道的等效一階時間常數(s)。**由 `fit_lag.py` 用 32 架次實測擬出**,
#:    不是 `predictor.PN_TAU_S = 2.0`(那個自述「沒有受控實測,是估的」)。
#:    tau = pn_lead = 1.0 時 gdot_act 恆等於 gdot_cmd,也就是「完全複刻節點指令、
#:    零額外遲滯」;> 1.0 才代表機體跟不上。實測擬合結果見 fit_lag.py 的輸出。
TAU_GAMMA_S = 1.00
#: 傾角通道的一階時間常數(s)。ArduPilot roll 迴路比 pitch 快,取 0.5 當先驗。
#: 靈敏度已掃(validate32.py 的 tau 掃描),對 t_go 的影響 < 0.2 s。
TAU_PHI_S = 0.50


@dataclass
class PNParams:
    """RRE7 契約快照的生效值。⚠️ 改任何一項,表就要重建。"""
    n_lat: float = 4.0
    n_vert: float = 5.0
    pn_lead: float = 1.0            # s
    pn_los_floor: float = 1.0
    pn_nz_min: float = -1.0
    pn_nz_max: float = 3.0
    pn_vmax: float = 210.0
    pn_thr_trim: float = 0.36
    pn_thr_gain: float = 0.030
    thr_max_pct: float = 100.0      # THR_MAX=100(JET_THRMAX 刻意不設)
    roll_limit_deg: float = 80.0
    dive_pitch_min_deg: float = -80.0
    dive_pitch_max_deg: float = 45.0
    fz_tgo: float = 1.5             # s,終端凍結
    fz_r: float = 100.0             # m,終端凍結
    tick_dt: float = 0.05           # 導引週期(實際 20 Hz,不是宣告的 50)
    #: ⚠️ `Plane.cpp:179` 的 `pitch_limit_min *= |cos(roll)|` —— tot_node **沒有複製**
    #:    這一條(它的 pitch 夾制是固定 [-80,+45])⇒ 橫向機動會偷走縱向權限而節點不知道。
    #:    以 uav2 中位 roll 36.6° 算,實得俯仰下限只有 -64.2°。
    pitch_min_roll_coupling: bool = True


@dataclass
class SimParams:
    tau_gamma_s: float = TAU_GAMMA_S
    tau_phi_s: float = TAU_PHI_S
    dt: float = 0.01                # 積分子步長(導引仍是 tick_dt 零階保持)
    t_max: float = 200.0
    home_amsl: float = 0.0          # 紅心地表 AMSL(海上 = 0)
    v_floor: float = 20.0           # 與 predictor 同
    r_overshoot_m: float = 5000.0   # 打過頭這麼多就判不可判
    use_load_factor_cl: bool = True
    throttle_law: str = "vmax"      # "vmax" | "clock" | "const"
    v_req_const: float = 210.0      # throttle_law == "const" 時用
    record: bool = False            # 是否回傳逐拍軌跡(只在單樣本除錯用)


def _as_arr(x, n):
    a = np.asarray(x, dtype=np.float64)
    if a.ndim == 0:
        a = np.full(n, float(a))
    return np.array(a, dtype=np.float64)


def fly(r0, z0, V0, gamma0_rad, eps0_rad, *,
        pn: PNParams | None = None, sp: SimParams | None = None,
        t_rem0=None):
    """向量化正向積分到觸地(z <= 0)。純量或 numpy 陣列都吃。

    參數
    ----
    r0, z0, V0   : 水平距(m)、離紅心地表高度(m)、3D 總速(m/s)
    gamma0_rad   : 航跡角(rad),**下降為負**
    eps0_rad     : **帶號**水平方位誤差(rad),eps>0 = 速度在 LOS 右側
    t_rem0       : throttle_law == "clock" 時的初始 `dive_T - now`(s)

    回傳(全部同形狀的 ndarray;純量輸入回純量)
    ----
    t_go        觸地耗時(s)。⚠️ 不可判的樣本是 **NaN**,不是 0 也不是 t_max
    valid       bool。False = 模擬失敗
    fail_code   0=OK 1=逾時 2=打過頭 3=速度掉到地板 4=NaN/Inf
    V_end / gs_end / gamma_end_deg / r_end / eps_end_deg
    z_at_r0     ⭐ r 首次 <= 0 時的殘餘高度(與 `predictor.h_at_crossing` 同語意,
                也是 KPI③ 的直接上游:err = 1.1668*h - 2.62,R^2=0.998)
    nz_sat_frac / phi_sat_frac / gamma_min_deg / n_total_max
    """
    pn = pn or PNParams()
    sp = sp or SimParams()

    scalar_in = np.ndim(r0) == 0
    n = int(np.size(r0))
    r = _as_arr(r0, n)
    z = _as_arr(z0, n)
    V = _as_arr(V0, n)
    gam = _as_arr(gamma0_rad, n)
    eps = _as_arr(eps0_rad, n)
    t = np.zeros(n)
    t_rem = (_as_arr(t_rem0, n) if t_rem0 is not None else np.full(n, np.nan))

    gdot_act = np.zeros(n)
    phi_act = np.zeros(n)
    # 終端凍結:凍結的是 (phi_cmd, gdot_want),油門**不凍**(見 tot_node._pn_step
    # 的註解:凍結是為了 lam_dot 在 d->0 的奇異點,油門通道沒有這個問題,
    # 連油門一起凍會變成「滑行進紅心」而驗收第一優先量的正是觸地速度)
    frozen = np.zeros(n, dtype=bool)
    phi_frz = np.zeros(n)
    gdot_frz = np.zeros(n)

    alive = np.ones(n, dtype=bool)
    fail = np.zeros(n, dtype=np.int8)
    t_go = np.full(n, np.nan)
    V_end = np.full(n, np.nan)
    gam_end = np.full(n, np.nan)
    r_end = np.full(n, np.nan)
    eps_end = np.full(n, np.nan)
    z_at_r0 = np.full(n, np.nan)
    t_at_r0 = np.full(n, np.nan)
    nz_sat = np.zeros(n)
    phi_sat = np.zeros(n)
    n_ticks = np.zeros(n)
    gam_min = np.zeros(n)
    n_tot_max = np.zeros(n)

    sub = max(1, int(round(pn.tick_dt / sp.dt)))
    dt = pn.tick_dt / sub
    roll_lim0 = math.radians(pn.roll_limit_deg)
    pitch_min0 = math.radians(pn.dive_pitch_min_deg)
    pitch_max0 = math.radians(pn.dive_pitch_max_deg)
    n_guid = int(math.ceil(sp.t_max / pn.tick_dt))

    traj = [] if sp.record else None

    for _tick in range(n_guid):
        idx = np.flatnonzero(alive)
        if idx.size == 0:
            break

        # ── 導引(零階保持,每 tick_dt 一次)────────────────────────────
        rr, zz, vv = r[idx], z[idx], V[idx]
        gg, ee = gam[idx], eps[idx]
        cg = np.cos(gg)
        vh = vv * cg                                # 水平速度分量
        R = np.maximum(np.hypot(rr, zz), 1e-3)

        # 橫向:真 PN。thetadot_LOS = -V cos(gamma) sin(eps)/r(= tot_node lam_dot)
        lam_dot = -vh * np.sin(ee) / np.maximum(rr, 1e-3)
        a_h = pn.n_lat * vh * lam_dot
        # ⚠️ 分母是 g*cos(gamma) 不是 g(tot_node 2609:維持俯角要 n*cos(phi)=cos(gamma))
        phi_cmd = np.arctan2(a_h, G * np.maximum(cg, 0.25))

        # 縱向:同一條律在垂直面,用角速率形式(角度形式會要出 -90°)
        rdot = -vh * np.cos(ee)                     # ⚠️ 含 eps ⇒ 這才是真的 ddot
        zdot = vv * np.sin(gg)
        sig_dot = (zdot * rr - zz * rdot) / (R * R)
        gdot_pn = -pn.n_vert * sig_dot
        nz_raw = cg + vv * gdot_pn / G
        nz = np.clip(nz_raw, pn.pn_nz_min, pn.pn_nz_max)
        nz_sat[idx] += (nz_raw != nz)
        gdot_cmd = (nz - cg) * G / vv
        gam_cmd = gg + gdot_cmd * pn.pn_lead

        # LOS 地板:指令俯角不准比「直指紅心的視線」更淺
        if pn.pn_los_floor > 0.0:
            gam_los = -np.arctan2(zz, np.maximum(rr, 1e-3))
            gam_cmd = np.minimum(gam_cmd, gam_los * pn.pn_los_floor)

        # 姿態夾制。⚠️ 點質量沒有 alpha ⇒ 用 gamma 當 pitch 的代理(實測 alpha 幾度)
        pmin = pitch_min0
        if pn.pitch_min_roll_coupling:
            # `Plane.cpp:179` pitch_limit_min *= |cos(roll)| —— tot_node 沒複製這條
            pmin = pitch_min0 * np.abs(np.cos(phi_act[idx]))
        gam_cmd = np.clip(gam_cmd, pmin, pitch_max0)

        # 韌體耦合:roll 上限 = 80°*|cos(pitch)|(tot_node 2699 已自己複製)
        r_lim = roll_lim0 * np.abs(np.cos(gg))
        phi_c = np.clip(phi_cmd, -r_lim, r_lim)
        phi_sat[idx] += (phi_c != phi_cmd)

        gdot_want = (gam_cmd - gg) / pn.pn_lead

        # 終端凍結:t_go = R/V <= fz_tgo 或 R <= fz_r ⇒ 鎖 (phi, gdot);油門不鎖
        tgo_now = R / np.maximum(vv, 1e-3)
        newly = (~frozen[idx]) & ((tgo_now <= pn.fz_tgo) | (R <= pn.fz_r))
        if newly.any():
            j = idx[newly]
            phi_frz[j] = phi_c[newly]
            gdot_frz[j] = gdot_want[newly]
            frozen[j] = True
        fz = frozen[idx]
        phi_c = np.where(fz, phi_frz[idx], phi_c)
        gdot_want = np.where(fz, gdot_frz[idx], gdot_want)

        # 油門:唯一的時間通道
        if sp.throttle_law == "vmax":
            v_req = np.full(idx.size, pn.pn_vmax)
        elif sp.throttle_law == "const":
            v_req = np.full(idx.size, sp.v_req_const)
        elif sp.throttle_law == "clock":
            tl = t_rem[idx] - t[idx]
            late = tl <= 1.0
            trm = np.where(late, np.maximum(1.0, R / np.maximum(vv, 1e-3)), tl)
            v_req = np.clip(R / np.maximum(trm, 1e-3), 45.0, pn.pn_vmax)
            v_req = np.where(late, pn.pn_vmax, v_req)   # pn_late_full
        else:
            raise ValueError(f"unknown throttle_law {sp.throttle_law!r}")

        n_ticks[idx] += 1

        # ── 積分(sub 個子步,導引指令零階保持)──────────────────────────
        for _s in range(sub):
            if idx.size == 0:
                break
            gg = gam[idx]
            cg = np.cos(gg)
            sg = np.sin(gg)
            vv = V[idx]

            # gammadot 一階遲滯 → 再夾一次可達 n_z(plant 端的能力限制)
            gdot_act[idx] += (gdot_want - gdot_act[idx]) * dt / sp.tau_gamma_s
            phi_act[idx] += (phi_c - phi_act[idx]) * dt / sp.tau_phi_s
            nz_ach = np.clip(cg + vv * gdot_act[idx] / G,
                             pn.pn_nz_min, pn.pn_nz_max)
            gdot = (nz_ach - cg) * G / vv

            # 誘導阻力:由**實際指令升力**驅動(相對 predictor 唯一的物理改動)
            cphi = np.maximum(np.abs(np.cos(phi_act[idx])), 0.10)
            n_total = nz_ach / cphi
            n_tot_max[idx] = np.maximum(n_tot_max[idx], np.abs(n_total))
            q_s = 0.5 * air_density_np(z[idx] + sp.home_amsl) * S_REF_M2 * vv * vv
            if sp.use_load_factor_cl:
                c_l = MASS_KG * G * n_total / np.maximum(q_s, 1e-6)
            else:
                c_l = WEIGHT_N * cg / np.maximum(q_s, 1e-6)
            D = q_s * (CD0 + c_l * c_l / PI_E_AR)
            T = thrust_np(vv, pn.thr_max_pct, v_req)

            # psidot = g n_z tan(phi)/(V cos(gamma)) ⇒ 側向真實轉向率(Guo Eq.6)
            psidot = G * nz_ach * np.tan(phi_act[idx]) / np.maximum(vv * cg, 1e-3)
            epsdot = (psidot
                      + vv * cg * np.sin(eps[idx]) / np.maximum(r[idx], 1e-3))

            r_prev = r[idx].copy()
            z_prev = z[idx].copy()

            v_new = np.maximum(vv + ((T - D) / MASS_KG - G * sg) * dt, sp.v_floor)
            V[idx] = v_new
            gam[idx] = gg + gdot * dt
            eps[idx] = eps[idx] + epsdot * dt
            g_new = gam[idx]
            r[idx] = r_prev - v_new * np.cos(g_new) * np.cos(eps[idx]) * dt
            z[idx] = z_prev + v_new * np.sin(g_new) * dt
            t[idx] = t[idx] + dt
            gam_min[idx] = np.minimum(gam_min[idx], g_new)

            # ⭐ r 首次 <= 0 的殘餘高度 —— 與 predictor.h_at_crossing 同語意
            crossed = (r_prev > 0.0) & (r[idx] <= 0.0) & np.isnan(z_at_r0[idx])
            if crossed.any():
                j = idx[crossed]
                f = r_prev[crossed] / np.maximum(
                    r_prev[crossed] - r[idx][crossed], 1e-9)
                z_at_r0[j] = z_prev[crossed] + (z[idx][crossed]
                                                - z_prev[crossed]) * f
                t_at_r0[j] = t[idx][crossed] - dt * (1.0 - f)

            # ── 終止判定 ────────────────────────────────────────────────
            zn = z[idx]
            done_land = zn <= 0.0
            done_over = r[idx] < -sp.r_overshoot_m
            done_slow = v_new <= sp.v_floor + 1e-9
            done_nan = ~(np.isfinite(zn) & np.isfinite(v_new)
                         & np.isfinite(gam[idx]) & np.isfinite(eps[idx]))
            done = done_land | done_over | done_slow | done_nan
            if done.any():
                j = idx[done]
                # 線性內插到 z=0(觸地),讓 t_go 不受步長量化影響
                zp, zc = z_prev[done], zn[done]
                fr = np.where(zp > zc, zp / np.maximum(zp - zc, 1e-9), 1.0)
                t_go[j] = np.where(done_land[done],
                                   t[idx][done] - dt * (1.0 - fr), t[idx][done])
                V_end[j] = v_new[done]
                gam_end[j] = gam[idx][done]
                r_end[j] = r[idx][done]
                eps_end[j] = eps[idx][done]
                fail[j] = np.where(done_nan[done], 4,
                                   np.where(done_slow[done], 3,
                                            np.where(done_over[done], 2, 0)
                                            )).astype(np.int8)
                alive[j] = False
                keep = ~done
                idx = idx[keep]
                phi_c = phi_c[keep]
                gdot_want = gdot_want[keep]
                v_req = v_req[keep]
            if sp.record and n == 1 and alive[0]:
                traj.append((float(t[0]), float(r[0]), float(z[0]), float(V[0]),
                             math.degrees(float(gam[0])),
                             math.degrees(float(eps[0])),
                             math.degrees(float(phi_act[0]))))

    # 還活著的 = 逾時
    if alive.any():
        j = np.flatnonzero(alive)
        fail[j] = 1
        V_end[j] = V[j]
        gam_end[j] = gam[j]
        r_end[j] = r[j]
        eps_end[j] = eps[j]

    valid = fail == 0
    # ⚠️⚠️ 不可判的樣本 t_go 一律 NaN —— **不可以**填 0 / t_max / 內插值。
    #    論文對「模擬失敗怎麼辦」是 NOT STATED,而我們的洞正好落在陡俯衝區,
    #    讓內插靜靜蓋過去 = 在最需要誠實的地方說謊。
    t_go = np.where(valid, t_go, np.nan)

    tot = np.maximum(n_ticks, 1.0)
    out = {
        "t_go": t_go, "valid": valid, "fail_code": fail,
        "V_end": V_end, "gs_end": V_end * np.cos(gam_end),
        "gamma_end_deg": np.degrees(gam_end),
        "r_end": r_end, "eps_end_deg": np.degrees(eps_end),
        "z_at_r0": z_at_r0, "t_at_r0": t_at_r0,
        "nz_sat_frac": nz_sat / tot, "phi_sat_frac": phi_sat / tot,
        "gamma_min_deg": np.degrees(gam_min), "n_total_max": n_tot_max,
        # 由殘餘高度推的觸地誤差(實測迴歸 err = 1.1668*h - 2.62,R^2=0.998)
        "miss_pred_m": np.where(np.isnan(z_at_r0), np.nan,
                                np.maximum(0.0, 1.1668 * z_at_r0 - 2.62)),
    }
    if scalar_in:
        out = {k: (v[0] if isinstance(v, np.ndarray) else v)
               for k, v in out.items()}
    if sp.record:
        out["traj"] = traj
    return out


def meta(pn: PNParams, sp: SimParams) -> dict:
    """建表中繼資料。⚠️ 這份東西**必須**跟表一起存 —— 換了任何一項表就作廢。"""
    return {
        "pn": asdict(pn), "sim": asdict(sp),
        "body": {"MASS_KG": MASS_KG, "S_REF_M2": S_REF_M2, "CD0": CD0,
                 "THRUST_FULL_N": THRUST_FULL_N, "PI_E_AR": PI_E_AR},
        "eps_definition": ("Guo Eq.(27)-(29) 運動學版:eps = 帶號**水平**方位誤差 "
                           "(= tot_node._lead_angle_deg 但不取 abs)。"
                           "⚠️ 不是 §II.A 口語版『速度向量與 LOS 鉛垂平面夾角』"
                           "—— 兩者在 gamma=-49°、eps=45° 差 -17.4°"),
        "r_definition": "Guo Eq.(18):**水平**距,不是斜距",
        "terminate": "z <= 0(觸地)。不是 R <= 0 —— 見 hit-criterion-ignores-altitude",
    }


def selfcheck() -> int:
    """與 `predictor.py` 的原語對拍 + 數值收斂檢查。回傳失敗數。"""
    bad = 0
    for h in (0.0, 1000.0, 3100.0, 8000.0):
        a, b = air_density_np(h), _P.air_density(h)
        if abs(a - b) > 1e-12:
            print(f"  FAIL air_density h={h}: {a} vs {b}")
            bad += 1
    for v in (100.0, 150.0, 210.0, 250.0):
        for vr in (150.0, 210.0):
            a = float(thrust_np(v, 100.0, vr))
            b = _P.thrust_n(v, 100.0, vr)
            if abs(a - b) > 1e-9:
                print(f"  FAIL thrust v={v} v_req={vr}: {a} vs {b}")
                bad += 1
    print(f"  predictor 原語對拍:{'OK' if bad == 0 else f'{bad} 項不合'}")
    # 數值收斂:同一組 FSV 用三個步長,t_go 全距必須遠小於 3.0 s 的閘門
    r0, z0, v0 = 6142.0, 3071.0, 186.0
    g0, e0 = math.radians(-3.0), math.radians(15.4)
    ts = []
    for d in (0.05, 0.02, 0.005):
        o = fly(r0, z0, v0, g0, e0, sp=SimParams(dt=d))
        ts.append(float(o["t_go"]))
    span = max(ts) - min(ts)
    print(f"  dt 收斂 0.05/0.02/0.005 → t_go "
          f"{ts[0]:.4f}/{ts[1]:.4f}/{ts[2]:.4f} s,全距 {span:.4f} s "
          f"({'OK' if span < 0.1 else 'FAIL: 數值雜訊 >= 0.1 s'})")
    if span >= 0.1:
        bad += 1
    return bad


if __name__ == "__main__":
    print("== sim3d selfcheck ==")
    nbad = selfcheck()
    print()
    print("== 冒煙:RRE7-U1 放行狀態(vmax 政策)==")
    o = fly(6145.0, 3130.0, 202.2, math.radians(-2.87), math.radians(8.13),
            sp=SimParams(throttle_law="vmax"))
    print(f"  t_go={o['t_go']:.2f}s  V_end={o['V_end']:.1f}  "
          f"gs_end={o['gs_end']:.1f}  gam_end={o['gamma_end_deg']:.1f}deg")
    print(f"  r_end={o['r_end']:.1f}m  valid={o['valid']}  "
          f"gam_min={o['gamma_min_deg']:.1f}deg  n_total_max={o['n_total_max']:.2f}")
    sys.exit(1 if nbad else 0)
