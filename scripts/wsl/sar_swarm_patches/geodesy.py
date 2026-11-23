"""共用幾何工具：避免 _dist_m / EARTH 常數在各節點重複定義（DRY）。"""
import math

EARTH_M_PER_DEG = 111320.0


def dist_m(a, b):
    """兩 (lat, lon) 點間近似距離（m），平面近似。"""
    dn = (b[0] - a[0]) * EARTH_M_PER_DEG
    de = (b[1] - a[1]) * EARTH_M_PER_DEG * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def polar_slot(leader_lat, leader_lon, leader_heading_deg, distance_m,
               bearing_offset_deg, leader_alt=0.0, alt_offset=0.0):
    """以長機航向為基準的極座標 slot（隨長機轉向旋轉、永遠相對長機固定方位）。

    參考多旋翼 track.follower_target：slot 的羅盤方位 = 長機航向 + bearing_offset，
    所以長機轉向時整個編隊跟著轉、僚機永遠保持在長機的相對位置。
      bearing_offset_deg: 相對長機機鼻的方位（0=正前、180=正後、180+θ=後左、180-θ=後右）。
    回傳 (lat, lon, alt)。
    """
    brg = math.radians(leader_heading_deg + bearing_offset_deg)
    dn = distance_m * math.cos(brg)                  # 北分量（m）
    de = distance_m * math.sin(brg)                  # 東分量（m）
    lat = leader_lat + dn / EARTH_M_PER_DEG
    lon = leader_lon + de / (EARTH_M_PER_DEG * math.cos(math.radians(leader_lat)))
    return lat, lon, leader_alt + alt_offset


def advance_along_heading(lat, lon, heading_deg, dist):
    """沿羅盤航向 heading_deg 前進 dist 公尺後的 (lat, lon)。

    slot 前饋用：高速編隊時 slot 每秒隨長機移動 v 公尺，而導引 WP 有 ~數秒的
    有效滯後(註冊+迴轉動力學) → 僚機永遠慢一拍、誤差存在結構性地板 ≈ v×lag。
    把 slot 沿長機航向外推 lead_time×leader_speed 就能抵消大部分滯後。"""
    hd = math.radians(heading_deg)
    nlat = lat + dist * math.cos(hd) / EARTH_M_PER_DEG
    nlon = lon + dist * math.sin(hd) / (EARTH_M_PER_DEG * math.cos(math.radians(lat)))
    return nlat, nlon


def speed_for_error(err, formed_r, catch_r, nominal, catch_speed):
    """編隊速度控制：離 slot 越遠（err 越大）→ 越快追上；err is None → nominal。

    斜坡從 err=0 起算（不是 formed_r）。若從 formed_r 起算，err 掉到門檻附近時
    閉合速率趨近 0，err 會漸近卡在 formed_r 上緣、永遠過不了門檻（實測：gate 60
    → floor 64-66，長機永不釋放巡航）。formed_r 仍是長機成形/等待的判準（master
    只拿它比較 err，不經過這裡）；這裡只保證僚機在門檻附近仍有正閉合率
    （例 catch_r=300、err=60 → f=0.2 → 等待時 14+0.2×17≈17.4，閉合 ~3.4 m/s）。"""
    if err is None:
        return nominal
    if err >= catch_r:
        return catch_speed
    f = err / max(catch_r, 1e-6)
    return nominal + f * (catch_speed - nominal)

def speed_for_along_error(along, catch_r, nominal, catch_speed, min_speed):
    """**有號**的縱向速度控制。along = slot 相對本機的沿航向分量(長機座標系)。

        along > 0  ⇒ slot 在前方,本機落後 ⇒ 加速,上限 catch_speed
        along < 0  ⇒ 本機超前 slot        ⇒ 減速,下限 min_speed
        |along| ≥ catch_r ⇒ 飽和

    ⚠️⚠️ 為什麼一定要有號(實測 S31):
       舊的 `speed_for_error(err, ...)` 吃的是**無號距離**,只有
       `err >= catch_r → catch_speed` 這一支。僚機超前 slot 10–28 km 時
       err 一樣是 10–28 km ⇒ 仍然命令 catch_speed 157 ⇒ **加速、超前更多**。
       S31 實測:四架僚機 along **全部為正** +10383 ~ +28014 m,
       佔總誤差 98%,而且單調發散。
    ⚠️ 這個 bug 先前**不會發作**:機體推力不足時僚機一律落後,踩不到超前那一側;
       而且實驗 A 之前速度指令零權限(TECS 走 no-airspeed 路徑),
       連加速那一側也沒有執行器。**修好速度通道等於同時解鎖了這個 bug。**

    ⚠️⚠️ `min_speed` 必須**低於長機當下速度**,否則誤差會凍結而不是收斂:
       長機被 max-consensus 壓在 wait_speed 等人,若僚機超前時也降到同一個
       wait_speed,兩者同速 ⇒ 閉合率 0 ⇒ 誤差卡在 10 km 不動。
       呼叫端應傳 nominal×0.67(本專案已確立的 wait 比例)並夾住 AIRSPEED_MIN。
    ⚠️ 減速權限(−45 EAS)是加速權限(+21)的 2 倍 —— 超前這一側本來就是
       比較強的通道,先前完全沒有用到。
    """
    if along is None:
        return nominal
    f = max(-1.0, min(1.0, float(along) / max(catch_r, 1e-6)))
    if f >= 0.0:
        return nominal + f * (catch_speed - nominal)
    return nominal + (-f) * (min_speed - nominal)


# ══════════════════════════════════════════════════════════════════════════
#  空速 / 地速轉換(EAS → TAS → GS)
#
#  ⚠️⚠️ 本專案反覆踩過的錯:**速度指令是空速,幾何是地速**,兩者不可混用。
#     DO_CHANGE_SPEED / nominal_speed / catch_speed 吃的都是 EAS(等效空速),
#     而編隊間距、追趕距離、驗收的「觸地地速」全是地面量。
#     3100 m 差 1.167 倍、5000 m 差 1.30 倍 —— 混用會系統性低估閉合率。
#     實例:長機 90 EAS vs 僚機 157 EAS,空速差 67 但**地速差 78.2** m/s。
# ══════════════════════════════════════════════════════════════════════════

ISA_LAPSE_K = 2.25577e-5      # 1/m;ISA 對流層溫度遞減率的無因次化係數
ISA_RHO_EXP = 4.25588         # ρ/ρ0 = (1 − k·h)^此指數
ISA_H_MAX = 11000.0           # 對流層頂;超過此高度本模型不成立


def density_ratio(altitude_amsl):
    """ISA 標準大氣的空氣密度比 σ = ρ/ρ0。

    σ = (1 − 2.25577e-5·h)^4.25588,h 為**絕對海拔**(m AMSL)。
    ⚠️ 用 AMSL 不是 AGL —— 密度只看絕對高度,和地表無關。本專案的
       HOME 在山區(1839–2673 m AMSL),用相對高度算會差一大截。
    ⚠️ 夾在對流層內並保證 σ > 0:負高度(死海、氣壓異常)回 σ ≥ 1 的合理值,
       超過 11 km 回對流層頂的值,不外插。
    """
    h = min(max(float(altitude_amsl), -500.0), ISA_H_MAX)
    return max(1e-3, (1.0 - ISA_LAPSE_K * h) ** ISA_RHO_EXP)


def eas_to_tas(eas, altitude_amsl):
    """等效空速 → 真實空速:TAS = EAS / √σ。"""
    return float(eas) / math.sqrt(density_ratio(altitude_amsl))


def tas_to_eas(tas, altitude_amsl):
    """真實空速 → 等效空速(反向,設定指令時用)。"""
    return float(tas) * math.sqrt(density_ratio(altitude_amsl))


def flight_path_angle_deg(pitch_deg, alpha_deg):
    """航跡角 γ = θ − α(度)。爬升為正、俯衝為負。"""
    return float(pitch_deg) - float(alpha_deg)


def calculate_projected_ground_speed(eas, altitude_amsl, pitch_deg=0.0,
                                     alpha_deg=0.0):
    """指令 EAS 在當前高度與姿態下的**水平地速投影** V_G。

        σ  = (1 − 2.25577e-5·h)^4.25588
        TAS = EAS / √σ
        γ  = θ − α
        V_G = TAS · cos γ

    ⚠️ 這就是「爬升時前進地速急劇下降」的量化式子:γ 越大,cos γ 把 TAS
       投影掉越多。長機爬升 / 僚機平飛的非對稱幾何,閉合率就是由這裡來的。
    ⚠️ 無除以零風險:density_ratio 有下限 1e-3,cos γ 在 ±90° 內恆 ≥ 0;
       γ 超過 ±90°(倒飛/垂直)時回 0 而不是負值 —— 水平投影的量值不應為負。
    回傳 (V_G, TAS, γ_deg)。
    """
    tas = eas_to_tas(eas, altitude_amsl)
    gam = flight_path_angle_deg(pitch_deg, alpha_deg)
    if abs(gam) >= 90.0:
        return 0.0, tas, gam
    return tas * math.cos(math.radians(gam)), tas, gam


def climb_rate_from_eas(eas, altitude_amsl, pitch_deg=0.0, alpha_deg=0.0):
    """同一組狀態下的垂直分量(爬升率,m/s;上為正)= TAS · sin γ。"""
    tas = eas_to_tas(eas, altitude_amsl)
    return tas * math.sin(math.radians(flight_path_angle_deg(pitch_deg, alpha_deg)))


def gamma_for_climb_rate(roc, eas, altitude_amsl):
    """要達到指定爬升率所需的航跡角(度)。用來把 TECS 的爬升率上限
    換算成可以直接夾的 γ 上限(見 wingman_node._climb_authority)。"""
    tas = eas_to_tas(eas, altitude_amsl)
    if tas <= 1e-6:
        return 0.0
    return math.degrees(math.asin(max(-1.0, min(1.0, float(roc) / tas))))
