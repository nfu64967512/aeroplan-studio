"""survey/ppk/geodesy.py — PPK 鏈用的最小大地計算（numpy 向量化）。

橢球一律用 WGS84 常數，與 RTKLIB 自己輸出經緯高時用的一樣（rtklib.h：RE_WGS84、FE_WGS84）。
基站框架若是 e-GNSS【2025】（GRS80 橢球），WGS84 與 GRS80 只差扁率 1/298.257223563 vs
1/298.257222101，對「經緯高 → ECEF → 加 0.3 m 槓桿臂 → 經緯高」這種來回換算的影響 < 1e-6 m；
重點是**來回用同一組常數**，座標本身仍屬基站的框架（見 outputs.py 的 crs_h）。

姿態慣例：yaw/pitch/roll 為 ZYX（航太）歐拉角，度；機身座標 FRD（前、右、下），
R_nb = Rz(yaw)·Ry(pitch)·Rx(roll) 把機身向量轉到 NED。yaw 從**真北**順時針量。
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "WGS84_A",
    "WGS84_F",
    "llh_to_ecef",
    "ecef_to_llh",
    "ned_basis",
    "ned_to_ecef_delta",
    "ecef_to_ned_delta",
    "body_to_ned",
]

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
_E2 = WGS84_F * (2.0 - WGS84_F)


def llh_to_ecef(lat_deg, lon_deg, h_m) -> np.ndarray:
    """經緯度（度）＋橢球高 → ECEF（m），回傳 (..., 3)。"""
    lat = np.radians(np.asarray(lat_deg, dtype=float))
    lon = np.radians(np.asarray(lon_deg, dtype=float))
    h = np.asarray(h_m, dtype=float)
    s, c = np.sin(lat), np.cos(lat)
    n = WGS84_A / np.sqrt(1.0 - _E2 * s * s)
    return np.stack([(n + h) * c * np.cos(lon), (n + h) * c * np.sin(lon),
                     (n * (1.0 - _E2) + h) * s], axis=-1)


def ecef_to_llh(xyz) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ECEF（m）→ 經緯度（度）＋橢球高（m）。

    固定點疊代（RTKLIB ecef2pos 同法），收斂到 1e-12 rad 以內；地表附近 4–5 次即收斂。
    """
    p = np.asarray(xyz, dtype=float)
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    r2 = x * x + y * y
    v = np.full_like(z, WGS84_A)
    zk = np.zeros_like(z)
    zz = z.copy()
    for _ in range(20):
        zk = zz
        sinp = zz / np.sqrt(r2 + zz * zz)
        v = WGS84_A / np.sqrt(1.0 - _E2 * sinp * sinp)
        zz = z + v * _E2 * sinp
        if np.all(np.abs(zz - zk) < 1e-6):
            break
    lat = np.where(r2 > 1e-24, np.arctan(zz / np.sqrt(np.maximum(r2, 1e-300))),
                   np.where(z > 0, np.pi / 2, -np.pi / 2))
    lon = np.where(r2 > 1e-24, np.arctan2(y, x), 0.0)
    h = np.sqrt(r2 + zz * zz) - v
    bad = ~np.all(np.isfinite(p), axis=-1)   # NaN 輸入要維持 NaN，不能掉進極點分支變成 ±90°
    return (np.where(bad, np.nan, np.degrees(lat)), np.where(bad, np.nan, np.degrees(lon)),
            np.where(bad, np.nan, h))


def ned_basis(lat_deg, lon_deg) -> np.ndarray:
    """回傳 (..., 3, 3)：列（row）依序為 N、E、D 單位向量在 ECEF 的分量。"""
    lat = np.radians(np.asarray(lat_deg, dtype=float))
    lon = np.radians(np.asarray(lon_deg, dtype=float))
    sp, cp, sl, cl = np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon)
    zero = np.zeros_like(sp)
    n = np.stack([-sp * cl, -sp * sl, cp], axis=-1)
    e = np.stack([-sl, cl, zero], axis=-1)
    d = np.stack([-cp * cl, -cp * sl, -sp], axis=-1)
    return np.stack([n, e, d], axis=-2)


def ned_to_ecef_delta(lat_deg, lon_deg, ned) -> np.ndarray:
    """局部 NED 位移／速度 → ECEF 分量。"""
    b = ned_basis(lat_deg, lon_deg)
    return np.einsum("...ij,...i->...j", b, np.asarray(ned, dtype=float))


def ecef_to_ned_delta(lat_deg, lon_deg, dxyz) -> np.ndarray:
    """ECEF 位移／速度 → 局部 NED 分量。"""
    b = ned_basis(lat_deg, lon_deg)
    return np.einsum("...ij,...j->...i", b, np.asarray(dxyz, dtype=float))


def body_to_ned(yaw_deg, pitch_deg, roll_deg, v_frd) -> np.ndarray:
    """機身 FRD 向量 → NED：R_nb = Rz(yaw)·Ry(pitch)·Rx(roll)。

    例：yaw 90°（機頭朝東）時 (0.3, 0, 0) → (0, 0.3, 0)，即往東 0.3 m。
    """
    y = np.radians(np.asarray(yaw_deg, dtype=float))
    p = np.radians(np.asarray(pitch_deg, dtype=float))
    r = np.radians(np.asarray(roll_deg, dtype=float))
    cy, sy, cp, sp, cr, sr = np.cos(y), np.sin(y), np.cos(p), np.sin(p), np.cos(r), np.sin(r)
    v = np.asarray(v_frd, dtype=float)
    vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
    n = (cy * cp) * vx + (cy * sp * sr - sy * cr) * vy + (cy * sp * cr + sy * sr) * vz
    e = (sy * cp) * vx + (sy * sp * sr + cy * cr) * vy + (sy * sp * cr - cy * sr) * vz
    d = (-sp) * vx + (cp * sr) * vy + (cp * cr) * vz
    return np.stack([n, e, d], axis=-1)
