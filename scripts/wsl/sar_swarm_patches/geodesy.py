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
