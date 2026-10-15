"""survey/ppk/interp.py ＋ geodesy.py —— 內插精度（10 Hz、20 m/s 直線＋轉彎）、旗標、σ、槓桿臂。

真值軌跡（局部 ENU，原點 25.1N 121.2E 150 m）：0–10 s 向東 20 m/s 直飛；10–20 s 以半徑 R=70.6 m
（20 m/s、30° 坡度）右轉；全程 0.5 m/s 爬升。ENU→ECEF→經緯高全部用 pyproj 與測試內自寫的旋轉，
不經過被測的 geodesy 模組。.pos 用 RTKLIB 的格式寫出（經緯 9 位小數 ≈ 0.1 mm 量化）。

量化結果（本測試斷言的數字；2026-10-14 實測值在括號內）：
- 線性內插：直線段 ≤0.3 mm（0.03 mm，只剩格式量化）；轉彎段在 epoch 中點誤差 = 弦高
  v²dt²/(8R) = 7.08 mm（7.05 mm；全程最大 7.13 mm、轉彎段平均 4.7 mm）。
- Hermite（中央差分切線）：直線與轉彎段 ≤0.3 mm（全程最大 0.10 mm、轉彎段平均 0.05 mm）；
  直線接轉彎的**曲率不連續點**兩側 ≤2.5 mm（1.8 mm；真實飛機滾轉進入需約 1 s，曲率連續）。
- Hermite（.pos 速度欄位當切線）：全程 ≤0.3 mm（0.085 mm），曲率不連續點也是 0.03 mm。
"""
import numpy as np
import pytest
from pyproj import Transformer

from survey.ppk import geodesy
from survey.ppk.interp import interpolate_at
from survey.ppk.pos import parse_pos
from tests.test_survey_ppk_pos import pos_text

_TO_ECEF = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
_TO_LLH = Transformer.from_crs("EPSG:4978", "EPSG:4979", always_xy=True)

LAT0, LON0, H0 = 25.1, 121.2, 150.0
V, R = 20.0, 70.6
OMEGA = V / R
CLIMB = 0.5
T0 = 2389 * 604800 + 180000.0


def _enu_basis(lat, lon):
    p, lo_ = np.radians(lat), np.radians(lon)
    e = np.array([-np.sin(lo_), np.cos(lo_), 0.0])
    n = np.array([-np.sin(p) * np.cos(lo_), -np.sin(p) * np.sin(lo_), np.cos(p)])
    u = np.array([np.cos(p) * np.cos(lo_), np.cos(p) * np.sin(lo_), np.sin(p)])
    return e, n, u


_O = np.array(_TO_ECEF.transform(LON0, LAT0, H0))
_E, _N, _U = _enu_basis(LAT0, LON0)


def truth_enu(tau):
    tau = np.asarray(tau, float)
    e = np.where(tau < 10, V * tau, 200.0 + R * np.sin(OMEGA * (tau - 10)))
    n = np.where(tau < 10, 0.0, -R + R * np.cos(OMEGA * (tau - 10)))
    u = CLIMB * tau
    ve = np.where(tau < 10, V, V * np.cos(OMEGA * (tau - 10)))
    vn = np.where(tau < 10, 0.0, -V * np.sin(OMEGA * (tau - 10)))
    return np.stack([e, n, u], -1), np.stack([ve, vn, np.full_like(tau, CLIMB)], -1)


def truth_ecef(tau):
    enu, _ = truth_enu(tau)
    return _O + enu[:, :1] * _E + enu[:, 1:2] * _N + enu[:, 2:3] * _U


def truth_llh(tau):
    x = truth_ecef(tau)
    lon, lat, h = _TO_LLH.transform(x[:, 0], x[:, 1], x[:, 2])
    return np.asarray(lat), np.asarray(lon), np.asarray(h)


def _traj(with_vel=False, q=None, sd=None, drop=()):
    tau = 0.1 * np.arange(201)
    keep = np.setdiff1d(np.arange(tau.size), drop)
    tau = tau[keep]
    lat, lon, h = truth_llh(tau)
    vel = None
    if with_vel:
        _, venu = truth_enu(tau)
        # 用原點的 ENU 軸代替各點的 NED 軸：300 m 內軸向差 <5e-5 rad ⇒ 速度差 ~1e-3 m/s，位置影響 <0.02 mm
        vel = np.stack([venu[:, 1], venu[:, 0], -venu[:, 2]], -1)
    txt = pos_text(T0 + tau, lat, lon, h, vel_ned=vel, q=None if q is None else np.asarray(q)[keep],
                   sd=None if sd is None else np.asarray(sd)[keep])
    return parse_pos(txt)


def _err_m(res, tau):
    x, y, z = map(np.asarray, _TO_ECEF.transform(list(res.lon_deg), list(res.lat_deg), list(res.h_ell_m)))
    return np.linalg.norm(np.stack([x, y, z], -1) - truth_ecef(tau), axis=-1)


STRAIGHT_MID = 0.05 + 0.1 * np.arange(5, 95)
TURN_MID = 10.05 + 0.1 * np.arange(2, 95)


# ---------------------------------------------------------------------------
# 精度
# ---------------------------------------------------------------------------
def test_linear_error_quantified_straight_vs_turn():
    tr = _traj()
    s = interpolate_at(tr, T0 + STRAIGHT_MID, method="linear")
    t = interpolate_at(tr, T0 + TURN_MID, method="linear")
    es, et = _err_m(s, STRAIGHT_MID), _err_m(t, TURN_MID)
    assert es.max() < 3e-4
    sagitta = V ** 2 * 0.1 ** 2 / (8 * R)                    # 7.08 mm
    assert sagitta == pytest.approx(7.08e-3, abs=1e-5)
    assert et.min() > sagitta - 3e-4 and et.max() < sagitta + 3e-4
    assert s.method == "linear" and all(f == ("lever_arm_unknown",) for f in s.flags)


@pytest.mark.parametrize("with_vel", [False, True])
def test_hermite_error_in_turn_and_straight(with_vel):
    tr = _traj(with_vel=with_vel)
    assert (tr.vel_ned_mps is not None) == with_vel
    rng = np.random.default_rng(1)
    rnd = rng.uniform(0.0, 20.0, 400)
    rnd = rnd[np.abs(rnd - 10.0) > 0.2][:200]
    tau = np.concatenate([STRAIGHT_MID, TURN_MID, rnd])
    r = interpolate_at(tr, T0 + tau)
    err = _err_m(r, tau)
    assert err.max() < 3e-4
    # 曲率不連續點（10 s）兩側的 epoch 中點：線性在轉彎側 7 mm；中央差分 Hermite 約 1.8 mm；給速度則 <0.3 mm
    j = np.array([9.95, 10.05])
    ej = _err_m(interpolate_at(tr, T0 + j), j)
    assert ej.max() < (3e-4 if with_vel else 2.5e-3)
    assert _err_m(interpolate_at(tr, T0 + j, method="linear"), j)[1] > 7e-3
    assert all("interp_linear" not in f for f in r.flags[:-200])
    # 速度：直線段 N=0、E=20、D=−0.5，航跡 90°
    k = slice(0, STRAIGHT_MID.size)
    assert np.allclose(r.vel_ned_mps[k], [0.0, V, -CLIMB], atol=2e-3)
    assert np.allclose(r.cog_deg[k], 90.0, atol=0.01)
    # 轉彎段：速度方向跟著真值轉
    _, venu = truth_enu(TURN_MID)
    vt = r.vel_ned_mps[STRAIGHT_MID.size:STRAIGHT_MID.size + TURN_MID.size]
    assert np.allclose(vt[:, 0], venu[:, 1], atol=0.02) and np.allclose(vt[:, 1], venu[:, 0], atol=0.02)


def test_event_exactly_on_epochs_and_ends():
    tr = _traj()
    tau = np.array([0.0, 0.1, 10.0, 20.0])
    r = interpolate_at(tr, T0 + tau)
    assert _err_m(r, tau).max() < 3e-4
    assert r.q_before.tolist() == [1, 1, 1, 1]


# ---------------------------------------------------------------------------
# 旗標與 σ
# ---------------------------------------------------------------------------
def test_flags_gap_float_single_out_of_range():
    q = np.ones(201, int)
    q[50] = 2          # 5.0 s float
    q[80] = 5          # 8.0 s single
    tr = _traj(q=q, drop=list(range(121, 125)))      # 12.0→12.5 s 缺 4 個 epoch
    tau = np.array([4.95, 5.05, 7.95, 12.25, 11.95, -1.0, 25.0])
    r = interpolate_at(tr, T0 + tau)
    assert r.flags[0] == ("ppk_float", "lever_arm_unknown")
    assert r.flags[1] == ("ppk_float", "lever_arm_unknown")
    assert r.flags[2] == ("ppk_float", "ppk_q5", "lever_arm_unknown")
    assert r.flags[3] == ("ppk_gap", "interp_linear", "lever_arm_unknown")
    assert r.flags[4] == ("lever_arm_unknown",)         # 缺口旁：同側三點切線，仍是 Hermite
    assert r.flags[5] == r.flags[6] == ("ppk_out_of_range", "lever_arm_unknown")
    assert (r.q_before[2], r.q_after[2]) == (1, 5)
    assert r.bracket_s[3] == pytest.approx(0.5)
    assert np.isnan(r.lat_deg[5]) and np.isnan(r.h_ell_m[6]) and np.isnan(r.sigma_h_m[5])
    # 缺口內線性內插：轉彎段 0.5 s 弦高 ≈ v²dt²/(8R) = 0.177 m，確實被標出來
    assert _err_m(r, tau)[3] == pytest.approx(V ** 2 * 0.25 / (8 * R), rel=0.05)
    # 缺口兩側相鄰區間（單側三點切線）仍 <0.3 mm
    side = np.array([11.95, 12.55, 12.65])
    assert _err_m(interpolate_at(tr, T0 + side), side).max() < 3e-4


def test_sigma_interpolation_and_extra_terms():
    sd = np.tile([0.006, 0.008, 0.010, 0.0, 0.0, 0.0], (201, 1))
    sd[31] = [0.012, 0.016, 0.030, 0.0, 0.0, 0.0]       # σh 0.01 → 0.02、σv 0.01 → 0.03
    tr = _traj(sd=sd)
    tau = np.array([3.025, 3.025, 3.025])
    r = interpolate_at(tr, T0 + tau, time_sigma_s=[0.0, 0.001, 0.0],
                       lever_arm_frd_m=(0.3, 0.0, 0.0), attitude_deg=[[90, 0, 0]] * 3,
                       attitude_sigma_deg=1.0)
    s_att = 0.3 * np.radians(1.0)                         # 5.24 mm
    assert r.sigma_h_m[0] == pytest.approx(np.hypot(0.0125, s_att), abs=1e-6)
    assert r.sigma_v_m[0] == pytest.approx(np.hypot(0.015, s_att), abs=1e-6)
    # 時間 1 ms × 水平 20 m/s = 2 cm，垂直 0.5 m/s × 1 ms = 0.5 mm
    assert r.sigma_h_m[1] == pytest.approx(np.sqrt(0.0125 ** 2 + 0.02 ** 2 + s_att ** 2), abs=2e-5)
    assert r.sigma_v_m[1] == pytest.approx(np.sqrt(0.015 ** 2 + 0.0005 ** 2 + s_att ** 2), abs=2e-6)
    r0 = interpolate_at(tr, T0 + tau[:1])
    assert r0.sigma_h_m[0] == pytest.approx(0.0125, abs=1e-6)


# ---------------------------------------------------------------------------
# 槓桿臂
# ---------------------------------------------------------------------------
def _lever_enu(res):
    # 傳 list：pyproj 對長度 1 的 ndarray 會走純量路徑並發 DeprecationWarning
    xa, ya, za = map(np.asarray, _TO_ECEF.transform(list(res.ant_lon_deg), list(res.ant_lat_deg),
                                                    list(res.ant_h_ell_m)))
    xc, yc, zc = map(np.asarray, _TO_ECEF.transform(list(res.lon_deg), list(res.lat_deg),
                                                    list(res.h_ell_m)))
    d = np.stack([xc - xa, yc - ya, zc - za], -1)
    out = []
    for k in range(len(res)):
        e, n, u = _enu_basis(res.ant_lat_deg[k], res.ant_lon_deg[k])
        out.append([d[k] @ e, d[k] @ n, d[k] @ u])
    return np.array(out)


def _rot(yaw, pitch, roll):
    y, p, r = np.radians([yaw, pitch, roll])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    return rz @ ry @ rx


def test_lever_arm_known_rotations():
    tr = _traj()
    tau = np.full(1, 5.05)
    att = [[90, 0, 0], [0, 90, 0], [0, 0, 90], [30, 5, -8]]
    levers = [(0.3, 0, 0), (0.3, 0, 0), (0, 0, 0.3), (0.3, 0.1, -0.2)]
    got = []
    for k in range(4):
        r = interpolate_at(tr, T0 + tau[:1], lever_arm_frd_m=levers[k], attitude_deg=[att[k]])
        got.append(_lever_enu(r)[0])
        assert r.lever_arm_frd_m == tuple(float(v) for v in levers[k])
        assert "lever_arm_unknown" not in r.flags[0]
    # yaw 90°：前方 0.3 m ⇒ 東 +0.3
    assert got[0] == pytest.approx([0.3, 0.0, 0.0], abs=1e-6)
    # pitch +90°（機頭朝上）：前方 ⇒ 上 +0.3
    assert got[1] == pytest.approx([0.0, 0.0, 0.3], abs=1e-6)
    # roll +90°（右翼朝下）：機腹方向（D）⇒ 西，east −0.3
    assert got[2] == pytest.approx([-0.3, 0.0, 0.0], abs=1e-6)
    # 一般姿態：與獨立建構的 Rz·Ry·Rx 一致
    ned = _rot(30, 5, -8) @ np.array([0.3, 0.1, -0.2])
    assert got[3] == pytest.approx([ned[1], ned[0], -ned[2]], abs=1e-6)
    # 天線位置本身不受槓桿臂影響
    r0 = interpolate_at(tr, T0 + tau[:1])
    r1 = interpolate_at(tr, T0 + tau[:1], lever_arm_frd_m=(0.3, 0, 0), attitude_deg=[[90, 0, 0]])
    assert r1.ant_lat_deg[0] == r0.lat_deg[0] and r1.ant_h_ell_m[0] == r0.h_ell_m[0]


def test_lever_arm_requires_attitude_and_handles_nan():
    tr = _traj()
    with pytest.raises(ValueError, match="姿態"):
        interpolate_at(tr, [T0 + 1.0], lever_arm_frd_m=(0.3, 0, 0))
    with pytest.raises(ValueError, match="筆數"):
        interpolate_at(tr, [T0 + 1.0, T0 + 2.0], lever_arm_frd_m=(0.3, 0, 0), attitude_deg=[[0, 0, 0]])
    with pytest.raises(ValueError, match="非有限"):
        interpolate_at(tr, [T0 + 1.0], lever_arm_frd_m=(np.nan, 0, 0), attitude_deg=[[0, 0, 0]])
    r = interpolate_at(tr, [T0 + 1.0, T0 + 2.0], lever_arm_frd_m=(0.3, 0, 0),
                       attitude_deg=[[0, 0, 0], [np.nan, 0, 0]])
    assert np.isfinite(r.lat_deg[0]) and np.isnan(r.lat_deg[1]) and np.isfinite(r.ant_lat_deg[1])
    assert r.flags[1] == ("no_attitude",)
    # 零槓桿臂不需要姿態；位置等於天線
    z = interpolate_at(tr, [T0 + 1.0], lever_arm_frd_m=(0, 0, 0))
    assert z.lat_deg[0] == z.ant_lat_deg[0] and z.flags[0] == ()


def test_argument_validation():
    tr = _traj()
    with pytest.raises(ValueError, match="method"):
        interpolate_at(tr, [T0], method="spline")
    with pytest.raises(ValueError, match="max_gap_s"):
        interpolate_at(tr, [T0], max_gap_s=0)


# ---------------------------------------------------------------------------
# geodesy 對 pyproj
# ---------------------------------------------------------------------------
def test_geodesy_matches_pyproj():
    rng = np.random.default_rng(2)
    lat = rng.uniform(-89, 89, 50)
    lon = rng.uniform(-180, 180, 50)
    h = rng.uniform(-100, 5000, 50)
    x = geodesy.llh_to_ecef(lat, lon, h)
    xr = np.stack(_TO_ECEF.transform(lon, lat, h), -1)
    assert np.max(np.abs(x - xr)) < 1e-6
    la, lo, hh = geodesy.ecef_to_llh(xr)
    assert np.max(np.abs(la - lat)) < 1e-10 and np.max(np.abs(lo - lon)) < 1e-10
    assert np.max(np.abs(hh - h)) < 1e-6
    nan = geodesy.ecef_to_llh(np.array([[np.nan, 1.0, 2.0]]))
    assert all(np.isnan(v[0]) for v in nan)


def test_body_to_ned_matches_rotation_matrix():
    rng = np.random.default_rng(3)
    for _ in range(20):
        y, p, r = rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180)
        v = rng.normal(size=3)
        assert geodesy.body_to_ned(y, p, r, v) == pytest.approx(_rot(y, p, r) @ v, abs=1e-12)
