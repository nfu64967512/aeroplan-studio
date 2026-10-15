"""survey/ppk/pos.py —— RTKLIB .pos 讀取：兩種時間格式、llh／xyz、速度欄位、拒絕清單。

pos_text() 依 RTKLIB demo5 src/solution.c 的格式字串產生 .pos（tow：``%4d %10.3f``；
hms：time2str；經緯 ``%14.9f``、高 ``%10.4f``、Q/ns ``%3d``、sd ``%8.4f``、age ``%6.2f``、
ratio ``%6.1f``、速度 ``%10.5f``；ECEF ``%14.4f``）。
"""
from datetime import datetime, timedelta

import numpy as np
import pytest
from pyproj import Transformer

from survey.ppk.pos import (PosFormatError, gpst_calendar_to_seconds, parse_pos, read_pos)

_TO_ECEF = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
_GPS_EPOCH = datetime(1980, 1, 6)
WEEK = 2389


def _hms(t: float) -> str:
    whole = int(np.floor(t))
    ms = int(round((t - whole) * 1000))
    if ms == 1000:
        whole, ms = whole + 1, 0
    d = _GPS_EPOCH + timedelta(seconds=whole)
    return d.strftime("%Y/%m/%d %H:%M:%S") + f".{ms:03d}"


def _tow(t: float) -> str:
    wk = int(t // 604800)
    return f"{wk:4d} {t - wk * 604800:10.3f}"


_HEAD = [
    "% program   : RTKPOST ver.demo5 b34k",
    "% inp file  : rover.obs",
    "% inp file  : base.obs",
    "% obs start : 2026/10/20 02:00:00.0 GPST (week2389 180000.0s)",
    "% pos mode  : Kinematic",
    "% solution  : Combined",
    "%",
]


def pos_text(t, lat, lon, h, *, q=None, ns=None, sd=None, vel_ned=None, timeform="tow",
             fmt="llh", height="ellipsoidal", label="GPST", datum="WGS84", ecef_sd=None) -> str:
    n = len(t)
    q = np.ones(n, int) if q is None else np.asarray(q)
    ns = np.full(n, 18) if ns is None else np.asarray(ns)
    sd = np.tile([0.006, 0.008, 0.015, 0.002, -0.003, 0.004], (n, 1)) if sd is None else np.asarray(sd)
    tf = _tow if timeform == "tow" else _hms
    tlab = f"%  {label:<{9 if timeform == 'tow' else 17}}"
    lines = list(_HEAD)
    if fmt == "llh":
        lines.append(f"% (lat/lon/height={datum}/{height},Q=1:fix,2:float,3:sbas,4:dgps,5:single,"
                     "6:ppp,ns=# of satellites)")
        hdr = (tlab + "  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)"
               "  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio")
        if vel_ned is not None:
            hdr += "    vn(m/s)    ve(m/s)    vu(m/s)      sdvn     sdve     sdvu    sdvne    sdveu    sdvun"
        lines.append(hdr)
        for k in range(n):
            s = (f"{tf(t[k])} {lat[k]:14.9f} {lon[k]:14.9f} {h[k]:10.4f} {q[k]:3d} {ns[k]:3d}"
                 + "".join(f" {v:8.4f}" for v in sd[k]) + f" {0.0:6.2f} {3.5:6.1f}")
            if vel_ned is not None:
                vn, ve, vd = vel_ned[k]
                s += f" {vn:10.5f} {ve:10.5f} {-vd:10.5f}" + " 0.00100" * 6
            lines.append(s)
    else:
        lines.append(f"% (x/y/z-ecef={datum},Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=# of satellites)")
        hdr = (tlab + "      x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns   sdx(m)   sdy(m)   sdz(m)"
               "  sdxy(m)  sdyz(m)  sdzx(m) age(s)  ratio")
        if vel_ned is not None:
            hdr += "    vx(m/s)    vy(m/s)    vz(m/s)      sdvx     sdvy     sdvz    sdvxy    sdvyz    sdvzx"
        lines.append(hdr)
        x, y, z = _TO_ECEF.transform(lon, lat, h)
        for k in range(n):
            s = (f"{tf(t[k])} {x[k]:14.4f} {y[k]:14.4f} {z[k]:14.4f} {q[k]:3d} {ns[k]:3d}"
                 + "".join(f" {v:8.4f}" for v in ecef_sd[k]) + f" {0.0:6.2f} {3.5:6.1f}")
            if vel_ned is not None:
                s += "".join(f" {v:10.5f}" for v in vel_ned[k]) + " 0.00100" * 6
            lines.append(s)
    return "\n".join(lines) + "\n"


def _small_track(n=20):
    t0 = WEEK * 604800 + 180000.0
    t = t0 + 0.1 * np.arange(n)
    lat = 25.1 + 1e-6 * np.arange(n)
    lon = 121.2 + 2e-5 * np.arange(n)
    h = 150.0 + 0.01 * np.arange(n)
    return t, lat, lon, h


# ---------------------------------------------------------------------------
def test_gpst_calendar_conversion():
    # GPST 曆法時間沒有閏秒 ⇒ 等於與 1980-01-06 的日曆差
    assert gpst_calendar_to_seconds("2025/10/19", "02:00:00.000") == \
        (datetime(2025, 10, 19, 2) - _GPS_EPOCH).total_seconds()
    assert gpst_calendar_to_seconds("1980/01/06", "00:00:01.250") == pytest.approx(1.25)
    assert gpst_calendar_to_seconds("2025/10/19", "02:00:00.007") - \
        gpst_calendar_to_seconds("2025/10/19", "02:00:00") == pytest.approx(0.007, abs=1e-6)


def test_tow_and_hms_formats_give_same_trajectory():
    t, lat, lon, h = _small_track()
    a = parse_pos(pos_text(t, lat, lon, h, timeform="tow"))
    b = parse_pos(pos_text(t, lat, lon, h, timeform="hms"))
    assert (a.time_format, b.time_format) == ("tow", "hms")
    assert np.max(np.abs(a.t_gpst_s - t)) < 1e-6
    assert np.max(np.abs(b.t_gpst_s - t)) < 1e-6
    for x in (a, b):
        assert len(x) == 20 and x.solution_format == "llh" and x.datum_label == "WGS84"
        assert np.allclose(x.lat_deg, lat, atol=1e-9) and np.allclose(x.lon_deg, lon, atol=1e-9)
        assert np.allclose(x.h_ell_m, h, atol=1e-4)
        assert x.q.tolist() == [1] * 20 and x.ns.tolist() == [18] * 20
        assert x.sdn_m[0] == pytest.approx(0.006) and x.sde_m[0] == pytest.approx(0.008)
        assert x.sdu_m[0] == pytest.approx(0.015) and x.sdeu_m[0] == pytest.approx(-0.003)
        assert x.sigma_h_m[0] == pytest.approx(0.01) and x.sigma_v_m[0] == pytest.approx(0.015)
        assert x.age_s[0] == 0.0 and x.ratio[0] == pytest.approx(3.5)
        assert x.vel_ned_mps is None and x.skipped_lines == []


def test_velocity_columns_llh():
    t, lat, lon, h = _small_track(5)
    v = np.tile([19.5, -2.25, 0.31], (5, 1))
    p = parse_pos(pos_text(t, lat, lon, h, vel_ned=v))
    assert p.vel_ned_mps.shape == (5, 3)
    assert np.allclose(p.vel_ned_mps, v, atol=1e-5)          # vu → −D


def test_ecef_format_converts_to_llh_and_ned_sigma():
    t, lat, lon, h = _small_track(6)
    # 已知 NED 共變異數：sdn 0.01、sde 0.02、sdu 0.03、cov(n,e)=+1e-4、cov(e,u)=−2e-4
    c_ned = np.array([[1e-4, 1e-4, 0.0], [1e-4, 4e-4, 2e-4], [0.0, 2e-4, 9e-4]])   # D 軸：cov(e,d)=−cov(e,u)
    ecef_sd = []
    for la, lo in zip(lat, lon):
        p, lo_ = np.radians(la), np.radians(lo)
        b = np.array([[-np.sin(p) * np.cos(lo_), -np.sin(p) * np.sin(lo_), np.cos(p)],
                      [-np.sin(lo_), np.cos(lo_), 0.0],
                      [-np.cos(p) * np.cos(lo_), -np.cos(p) * np.sin(lo_), -np.sin(p)]])
        c = b.T @ c_ned @ b

        def sq(v):
            return np.sign(v) * np.sqrt(abs(v))
        ecef_sd.append([np.sqrt(c[0, 0]), np.sqrt(c[1, 1]), np.sqrt(c[2, 2]),
                        sq(c[0, 1]), sq(c[1, 2]), sq(c[2, 0])])
    vel = []
    for la, lo in zip(lat, lon):
        p, lo_ = np.radians(la), np.radians(lo)
        b = np.array([[-np.sin(p) * np.cos(lo_), -np.sin(p) * np.sin(lo_), np.cos(p)],
                      [-np.sin(lo_), np.cos(lo_), 0.0],
                      [-np.cos(p) * np.cos(lo_), -np.cos(p) * np.sin(lo_), -np.sin(p)]])
        vel.append(b.T @ np.array([19.5, -2.25, 0.31]))
    p = parse_pos(pos_text(t, lat, lon, h, fmt="xyz", ecef_sd=np.array(ecef_sd), vel_ned=np.array(vel),
                           timeform="hms"))
    assert p.solution_format == "xyz"
    assert np.allclose(p.lat_deg, lat, atol=2e-9) and np.allclose(p.lon_deg, lon, atol=2e-9)
    assert np.allclose(p.h_ell_m, h, atol=2e-4)
    assert np.allclose(p.sdn_m, 0.01, atol=5e-4) and np.allclose(p.sde_m, 0.02, atol=5e-4)
    assert np.allclose(p.sdu_m, 0.03, atol=5e-4)
    assert np.allclose(p.sdne_m, 0.01, atol=1e-3)            # sqrt(1e-4)
    assert np.allclose(p.sdeu_m, -np.sqrt(2e-4), atol=1e-3)  # cov(e,u) = −cov(e,d) < 0
    assert np.allclose(p.sdun_m, 0.0, atol=2e-3)
    assert np.allclose(p.vel_ned_mps, [19.5, -2.25, 0.31], atol=1e-4)


@pytest.mark.parametrize("kw, msg", [
    (dict(label="UTC"), "GPST"),
    (dict(label="JST"), "GPST"),
    (dict(height="geodetic"), "ellipsoidal"),
    (dict(datum="Tokyo"), "datum"),
])
def test_rejected_settings(kw, msg):
    t, lat, lon, h = _small_track(3)
    with pytest.raises(PosFormatError, match=msg):
        parse_pos(pos_text(t, lat, lon, h, **kw))


def test_rejects_missing_header_dms_enu():
    t, lat, lon, h = _small_track(3)
    txt = pos_text(t, lat, lon, h)
    body = "\n".join(ln for ln in txt.splitlines() if not ln.startswith("%"))
    with pytest.raises(PosFormatError, match="out-outhead"):
        parse_pos(body)
    no_hline = "\n".join(ln for ln in txt.splitlines() if "lat/lon/height=" not in ln)
    with pytest.raises(PosFormatError, match="高度"):
        parse_pos(no_hline)
    dms = txt.replace("latitude(deg) longitude(deg)", "latitude(d'\") longitude(d'\")")
    with pytest.raises(PosFormatError, match="out-degform"):
        parse_pos(dms)
    enu = txt.replace("latitude(deg) longitude(deg)  height(m)", "e-baseline(m) n-baseline(m) u-baseline(m)")
    with pytest.raises(PosFormatError, match="無法辨識"):
        parse_pos(enu)
    nosd = txt.replace("sdun(m)", "foo(m)")
    with pytest.raises(PosFormatError, match="sdun"):
        parse_pos(nosd)


def test_truncated_last_line_reported_middle_line_rejected():
    t, lat, lon, h = _small_track(5)
    txt = pos_text(t, lat, lon, h)
    lines = txt.splitlines()
    cut = "\n".join(lines[:-1] + [lines[-1][:30]])
    p = parse_pos(cut)
    assert len(p) == 4 and p.skipped_lines == [len(lines)]
    bad = "\n".join(lines[:-3] + [lines[-3][:30]] + lines[-2:]) + "\n"
    with pytest.raises(PosFormatError, match=f"第 {len(lines) - 2} 行"):
        parse_pos(bad)
    # 截斷行有換行、欄位數不對 ⇒ 只容許最後一行
    p2 = parse_pos("\n".join(lines[:-1] + [lines[-1][:30]]) + "\n")
    assert len(p2) == 4 and p2.skipped_lines == [len(lines)]
    # 檔尾沒有換行：最後一行即使欄位數剛好對（截在最後一個數字中間）也不採用，並記下行號
    p3 = parse_pos(txt[:-3])
    assert len(p3) == 4 and p3.skipped_lines == [len(lines)]
    assert p3.ratio[-1] == pytest.approx(3.5)


def test_non_increasing_time_rejected():
    t, lat, lon, h = _small_track(5)
    t2 = t.copy()
    t2[3] = t2[2]
    with pytest.raises(PosFormatError, match="遞增"):
        parse_pos(pos_text(t2, lat, lon, h))


def test_mixed_time_format_rejected():
    t, lat, lon, h = _small_track(4)
    a = pos_text(t, lat, lon, h, timeform="tow").splitlines()
    b = pos_text(t, lat, lon, h, timeform="hms").splitlines()
    with pytest.raises(PosFormatError, match="時間格式"):
        parse_pos("\n".join(a[:-1] + [b[-1]]) + "\n")


def test_comma_separator_and_read_pos(tmp_path):
    t, lat, lon, h = _small_track(4)
    txt = pos_text(t, lat, lon, h)
    lines = [ln if ln.startswith("%") else ",".join(ln.split()) for ln in txt.splitlines()]
    f = tmp_path / "rover.pos"
    f.write_text("\n".join(lines) + "\n", encoding="utf-8")
    p = read_pos(f)
    assert len(p) == 4 and p.source == str(f)
    assert np.allclose(p.lat_deg, lat, atol=1e-9)
