"""predictor.py — 線上終端狀態預測器(Hybrid Trajectory Shaping 的地基)。

⚠️⚠️ 這支的物理**逐項對齊** `scripts/research/predictor/sim.py`,
   而那一支是用 RPK/RPL 十二架次實測校準並驗證過的:

       c_D0 = 0.01500     n=1617、sd 0.00058
       PN 段重放 γ(t):速度 RMS 5.4 m/s、終點偏差 +0.2 ± 1.0 m/s(121→219 m/s)

   `tests/test_predictor_module.py` 逐點比對兩邊的輸出。**兩者分歧 = 有一邊改壞了。**

## 為什麼要有節點端的版本

Leblebici 的 Hybrid Trajectory Shaping 用「線上正向積分 + 可達性觸發」取代
固定距離的交棒:每個導引週期把當前狀態往前推到 R≤0,拿到**預測的**終端速度與
航跡角,兩者同時落在容差內才觸發俯衝;需求角度越陡就**延後**觸發。

我們現在是固定 `gate_r`,而 RPQ 三趟證明那條路無解:
交接高度釘在 perch 3100 時,最小可能 γ = atan(3100/4784) = 32.9°,
而實測全中帶是 ≤29.1°。**沒有任何 gate_r 值能解**,必須先知道「還降得下來嗎」。

## ⚠️ 假設(用之前必讀)

* **無風**:V 取 3D 地速當 TAS。`scripts/wsl` 下沒有任何 `SIM_WIND` 設定,
  SITL 目前無風、兩者相等。加風之後 c_D0 的校準值會跟著偏。
* **推力模型**是 `tot_node._pn_cmd` 的油門律 `thr = clip(0,1, 0.36+0.030·(v_req−V))`,
  上限 `THR_MAX`。⚠️ 設計推力 25 與實跑 100 差 4 倍,換組態要重新校準。
* **單一 c_D0** 取代論文的 (Mach, α, δe) 三維查表。次音速定構型下誤差已量化
  (RMS 5.4 m/s),不可外推到其他構型。
* 縱向平面點質量,不含側向動態。橫向由 PN 自己管(實測 12/12 架 ≤8.53 m,不是瓶頸)。
"""
from __future__ import annotations

import math

G = 9.80665

#: ⚠️ 機體常數。與 `scripts/research/predictor/sim.py` 逐項相同;
#: `jet_design.py` 是它們的來源。改任何一個都必須重跑校準。
MASS_KG = 22.0
S_REF_M2 = 0.45
SPAN_M = 1.88
OSWALD_E = 0.9
CD0 = 0.01500                       # 實測反解,n=1617
#: 伺服 1.0 時的推力(N)。`jet_design.py` 的 T/W 定義:TFULL = m·g/0.7。
THRUST_FULL_N = MASS_KG * G / 0.7

WEIGHT_N = MASS_KG * G
_AR = SPAN_M * SPAN_M / S_REF_M2
_PI_E_AR = math.pi * OSWALD_E * _AR


def air_density(h_m: float) -> float:
    """ISA 密度(kg/m³)。⚠️ h 必須是 **AMSL**,不是相對 HOME。"""
    return 1.225 * (1.0 - 2.25577e-5 * max(h_m, 0.0)) ** 4.2586


def drag_n(v: float, h_m: float, gamma_rad: float) -> float:
    """阻力(N)。誘導阻力用「升力平衡 W·cosγ」估 C_L —— 準平衡假設。"""
    q_s = 0.5 * air_density(h_m) * S_REF_M2 * v * v
    c_l = WEIGHT_N * math.cos(gamma_rad) / max(q_s, 1e-6)
    return q_s * (CD0 + c_l * c_l / _PI_E_AR)


def thrust_n(v: float, thr_max_pct: float, v_req: float = 210.0) -> float:
    """`tot_node._pn_cmd` 的油門律,再夾 THR_MAX。

    ⚠️ 這個律是 PN 段的。若哪天 `dive_full_thr` 打開(thr 鎖 1.0),
       速度伺服就消失、俯衝變成純能量積分,**靈敏度會差 6–9 倍** ——
       屆時這個函式要跟著換,否則預測會系統性偏。
    """
    cmd = max(0.0, min(1.0, 0.36 + 0.030 * (v_req - v)))
    return min(cmd, thr_max_pct / 100.0) * THRUST_FULL_N


def terminal_speed_on_ray(gamma_deg: float, h_m: float, thr_max_pct: float,
                          v_req: float = 210.0) -> float:
    """定俯角路徑上的準平衡速度(m/s)。用來畫可行包絡,不用來積分。"""
    gam = math.radians(gamma_deg)
    v = 150.0
    for _ in range(400):
        f = (thrust_n(v, thr_max_pct, v_req) + WEIGHT_N * math.sin(gam)
             - drag_n(v, h_m, gam))
        v = max(20.0, min(400.0, v + f / MASS_KG * 0.25))
    return v


#: ⚠️⚠️ PN 實際交得出的航跡角上限(度)。**這不是設計值,是實測分界。**
#:
#: ⚠️⚠️ 2026-08-14 更新 33.0 → 36.0。舊值是**兩個陡俯角開關關閉時**量的:
#:     (pn_los_floor=0、pn_nz_min=0.05,互動路徑 24 架次全在這個狀態)
#:     舊分界 γ ≤ 29.1° 全中 / γ ≥ 33.8° 全敗
#: RPT 三趟把開關對齊批次路徑(1.0 / −1.0)之後,分界整個上移:
#:     γ ≤ 35.6°  四架全中(殘餘 −0.30 / 2.34 / 2.88 / 4.61 m)
#:     γ ≥ 37.4°  兩架全敗(殘餘 1461 / 3048 m)
#: 取 36.0 = 兩者中間。
#: ⚠️ 用舊值 33 的代價是**拒絕本來可行的交棒** —— 可達性判據會把 34–35° 判成
#:    不可達而延後俯衝,而那個區間實測是全中的。誤拒與誤放一樣傷。
#:
#: ⚠️ 不夾這個上限的後果**已經量到**:把 RPQ 的交接狀態
#:    (h=3100、d=4700、V=160 ⇒ 需求 γ=33.4°)代進純視線追蹤,
#:    預測器回報 d_end = −0.17 m,也就是「會命中」;
#:    而 RPQ 六架實際殘餘 416–3084 m,**全部沒落地**。
#:    差別不在積分器(它用實測 γ(t) 重放時 RMS 只有 5.4 m/s),
#:    而在「指令的 γ」與「做得到的 γ」是兩回事 —— 這正是本專案的核心病灶。
#: ⇒ 預測器若不夾,它會用一個飛機做不到的剖面去宣告可達,那比沒有預測更糟。
PN_GAMMA_CAP_DEG = 36.0


def predict_terminal(h0: float, d0: float, v0: float, *,
                     thr_max_pct: float = 100.0, v_req: float = 210.0,
                     home_amsl: float = 0.0, dt: float = 0.05,
                     t_max: float = 400.0, gamma_policy=None,
                     gamma_cap_deg: float = PN_GAMMA_CAP_DEG) -> dict:
    """把當前狀態正向積分到觸地,回傳預測的終端量。

    ⚠️⚠️ **終止條件是 h <= 0(觸地),不是 R <= 0。**
       論文用 R<=0 是因為飛彈打的是空中/地面點目標且工作在斜距;我們的驗收
       是**觸地**基準(`stot_score.py` 的觸地欄),而且誤差 = 1.167×殘餘高度
       —— 用 R<=0 會把「從上空掠過」算成命中,那正是 hit-criterion-ignores-altitude
       記過的坑。

    參數
    ----
    h0        : 離**紅心地表**的高度(m)。⚠️ 不是相對 HOME,不是 AMSL。
    d0        : 距紅心的水平距(m)。
    v0        : 當下總速(m/s)。
    home_amsl : 紅心地表的 AMSL(m),用來算空氣密度。海上目標給 0。
    gamma_policy : f(d, h) -> 俯角(度)。None = 純視線追蹤 atan(h/d),
                   也就是「一直對著紅心飛」。
    gamma_cap_deg : ⚠️⚠️ **飛機做得到的**俯角上限,不是想要的。預設取實測
                   `PN_GAMMA_CAP_DEG`。傳 None 才是理想追蹤(只用於對照,
                   實際判斷可達性時**絕不可以**傳 None —— 見該常數的說明)。

    回傳
    ----
    dict(v_end, gs_end, gamma_end_deg, t_go, d_end, reached)
      v_end    : 觸地總速
      gs_end   : 觸地**地速** = v·cosγ ⚠️ KPI② 用的是這個量
      d_end    : 觸地時距紅心的水平距(> 0 = 打短,< 0 = 打過頭)
      reached  : 是否在 t_max 內觸地
    """
    if gamma_policy is None:
        def gamma_policy(d, h):                       # noqa: E306
            return math.degrees(math.atan2(max(h, 0.0), max(d, 1.0)))

    cap = 85.0 if gamma_cap_deg is None else float(gamma_cap_deg)
    h, d, v, t = float(h0), float(d0), max(1.0, float(v0)), 0.0
    gam = 0.0
    gam_req_max = 0.0        # 這條剖面**要求**過的最陡角(未夾)
    sat_n = tot_n = 0        # 夾到上限的取樣比例 = 「要求超過能力」的程度
    h_cross = None           # ⭐ 水平掠過紅心(d 由正轉負)當下的高度
    t_cross = None
    d_prev = float(d0)
    while h > 0.0 and t < t_max:
        want = max(0.0, gamma_policy(d, h))
        gam_req_max = max(gam_req_max, want)
        tot_n += 1
        if want > cap:
            sat_n += 1
        gam = math.radians(min(want, cap))
        h_amsl = h + home_amsl
        v += (thrust_n(v, thr_max_pct, v_req) + WEIGHT_N * math.sin(gam)
              - drag_n(v, h_amsl, gam)) / MASS_KG * dt
        v = max(20.0, v)
        h -= v * math.sin(gam) * dt
        d -= v * math.cos(gam) * dt
        t += dt
        # ⭐⭐ 驗收量的是**水平最近通過時的殘餘高度**,不是觸地時的水平距。
        #    誤差 = 1.167 × 殘餘高度(R²=0.998),所以這個量才是 KPI③ 的直接上游。
        #    ⚠️ 少了它,預測器會把「從紅心上空 3 km 掠過、然後在 74 m 外落地」
        #       回報成 reached=True —— 那正是 hit-criterion-ignores-altitude 的坑。
        if h_cross is None and d_prev > 0.0 >= d:
            f = d_prev / max(d_prev - d, 1e-9)      # 線性內插到 d=0
            h_cross = h + v * math.sin(gam) * dt * (1.0 - f)
            t_cross = t - dt * (1.0 - f)
        d_prev = d
        if d < -5000.0:                               # 飛過頭太多,不必再積
            break
    return {
        "v_end": v,
        "gs_end": v * math.cos(gam),
        "gamma_end_deg": math.degrees(gam),
        "t_go": t,
        "d_end": d,
        "reached": h <= 0.0,
        # ⚠️ 下面兩個是**判讀用的誠實指標**,不要丟掉:
        #    gamma_req_max_deg 高於 cap ⇒ 這條剖面要求飛機做它做不到的事;
        #    gamma_sat_frac 就是「要求超過能力」的時間比例。
        #    兩者都 > 0 時,`reached` 這個布林值不可信。
        "gamma_req_max_deg": gam_req_max,
        "gamma_sat_frac": (sat_n / tot_n) if tot_n else 0.0,
        # ⭐ 與 stot_score 的「最低AGL / 殘餘高度」同一個量,可直接對照實測。
        "h_at_crossing": h_cross,
        "t_at_crossing": t_cross,
        # 由殘餘高度推的觸地誤差(實測迴歸 err = 1.1668·h − 2.62,R²=0.998)
        "miss_pred_m": (None if h_cross is None
                        else max(0.0, 1.1668 * h_cross - 2.62)),
    }


#: ⚠️ 終端導引段的**等效系統時間常數**(s)。
#: 由 u1-descent-authority 的兩條獨立估計取下緣:時域擬合 γ(t) 給 20–58 s、
#: 從終端脫靶量反解給 15.7–34.5 s —— 但那兩個是**含 TECS 的航點段**。
#: PN 段繞過 TECS,實測 α 濾波 τ = 1.75 s、姿態迴路快得多,取 2.0 s。
#: ⚠️ 這個值**沒有受控實測**,是估的。改動它會改變下面的最小交接高度。
PN_TAU_S = 2.0

#: Zarchan《Tactical and Strategic Missile Guidance》第 3/6 章的伴隨法結論:
#: 單一遲滯歸向迴路裡,若導引飛行時間 `t_F < 3~5τ`,**終端脫靶是物理必然**。
#: 取 5 當保守值。
ZARCHAN_TF_OVER_TAU = 5.0


def min_homing_time_s(tau_s: float = PN_TAU_S) -> float:
    """終端導引至少需要的時間(s)。低於它,殘餘高度清不掉是必然而非調參問題。"""
    return ZARCHAN_TF_OVER_TAU * tau_s


def min_handover_altitude_m(gamma_deg: float, v_ms: float,
                            tau_s: float = PN_TAU_S) -> float:
    """交接高度的**硬下界**(m):要有 `5τ` 的時間可用,就要有這麼多高度可降。

        h_min = V·sin(γ) · 5τ

    ⚠️⚠️ 這是**下界不是目標**。它回答的是「太低會必然脫靶」,
       不回答「多高才好」—— 太高會讓需求俯角超過 PN 能力(見 PN_GAMMA_CAP_DEG),
       兩者夾出可用窗口。
    ⚠️ 我們實測交接在 2247–3510 m,而本式在 γ=30°、V=180 給 900 m
       ⇒ **這一條目前沒有被違反**,不是現在的瓶頸。寫進來是為了將來降低交接高度時
       有東西擋住 —— 那是「提早下降」這個方向的必然終點。
    """
    return abs(v_ms) * math.sin(math.radians(gamma_deg)) * min_homing_time_s(tau_s)


def required_gamma_deg(h: float, d: float) -> float:
    """直指紅心所需的航跡角(度)。交棒可行性的第一判據。"""
    return math.degrees(math.atan2(max(h, 0.0), max(d, 1.0)))


def feasible_time_window(h0: float, d0: float, v0: float, *,
                         thr_max_pct: float = 100.0, home_amsl: float = 0.0,
                         gamma_lo: float = 10.0, gamma_hi: float = 45.0) -> dict:
    """本機**自己飛**能達成的抵達時間窗 [t_min, t_max](s)。

    ⭐ 這是 PINN-PCG 時間協同的輸入:協同時刻取
       `t_c = ½(max_i t_min_i + min_i t_max_i)`,
       而 `t_min_i` 在現行程式碼裡**完全不存在** —— 現行共識只取最慢的那架
       (`now + max_j(d_j/pace_j) + 2.0`),**從不檢查最快的那架能不能等那麼久**。

    ⚠️ 這裡用「定俯角族」掃出上下界,不是真正的最佳控制解 ——
       足以當協商用的粗窗,不足以當保證。窗太窄或交集為空時要退回舊行為。
    """
    ts = []
    for gd in (gamma_lo, gamma_hi):
        r = predict_terminal(h0, d0, v0, thr_max_pct=thr_max_pct,
                             home_amsl=home_amsl,
                             gamma_policy=lambda d, h, _g=gd: _g)
        if r["reached"]:
            ts.append(r["t_go"])
    # 純視線追蹤是「最直接」的一條,通常最快
    r = predict_terminal(h0, d0, v0, thr_max_pct=thr_max_pct,
                         home_amsl=home_amsl)
    if r["reached"]:
        ts.append(r["t_go"])
    if not ts:
        return {"t_min": None, "t_max": None, "n": 0}
    return {"t_min": min(ts), "t_max": max(ts), "n": len(ts)}
