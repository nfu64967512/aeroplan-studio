"""survey 測試共用：寫 DataFlash .BIN 的小工具＋已知真值的合成建圖航線。

其他 test_survey_*.py 從這裡 import。真值的座標換算刻意用 pyproj 與 scipy 的旋轉（不是
survey.datum／survey.pose 的函式），比對才不會自己驗自己。
"""

import math
import struct

import numpy as np
import pytest
from pyproj import Transformer
from scipy.spatial.transform import Rotation

from pymavlink import DFReader

_STRUCT = {"Q": "Q", "B": "B", "H": "H", "I": "I", "L": "i", "e": "i", "E": "I", "c": "h", "C": "H",
           "f": "f", "d": "d", "Z": "64s", "N": "16s", "n": "4s", "b": "b", "h": "h", "i": "i", "M": "B", "q": "q"}
_SCALE = {"L": 1e7, "e": 100.0, "E": 100.0, "c": 100.0, "C": 100.0}

CAM_FMT_FLOAT = "QBHIHLLeeefff"      # ≥4.7
CAM_FMT_CENTIDEG = "QBHIHLLeeeccC"   # ≤4.6
CAM_COLS = ["TimeUS", "I", "Img", "GPSTime", "GPSWeek", "Lat", "Lng", "Alt", "RelAlt", "GPSAlt", "R", "P", "Y"]


class DFWriter:
    """最小的 DataFlash 二進位寫入器（FMT + 訊息；可選 UNIT/FMTU）。"""

    def __init__(self):
        self.buf = bytearray()
        self.fmts = {}
        self.next_id = 130

    def define(self, name, fmt, cols):
        mid = self.next_id
        self.next_id += 1
        length = 3 + struct.calcsize("<" + "".join(_STRUCT[c] for c in fmt))
        self.buf += bytes([0xA3, 0x95, 0x80]) + struct.pack(
            "<BB4s16s64s", mid, length, name.encode(), fmt.encode(), ",".join(cols).encode())
        self.fmts[name] = (mid, fmt, list(cols))

    def write(self, name, **vals):
        mid, fmt, cols = self.fmts[name]
        packed = []
        for c, ch in zip(cols, fmt):
            v = vals.get(c, 0)
            if ch in _SCALE:
                v = int(round(v * _SCALE[ch]))
            elif ch in "ZNn":
                v = v.encode() if isinstance(v, str) else v
            elif ch in "BHIQbhiqM":
                v = int(v)
            packed.append(v)
        self.buf += bytes([0xA3, 0x95, mid]) + struct.pack("<" + "".join(_STRUCT[c] for c in fmt), *packed)

    def save(self, path):
        with open(path, "wb") as fh:
            fh.write(bytes(self.buf))
        return path


def define_standard(w, cam_fmt=CAM_FMT_FLOAT, gpa_field="AEl"):
    w.define("PARM", "QNff", ["TimeUS", "Name", "Value", "Default"])
    w.define("MSG", "QBBZ", ["TimeUS", "ID", "Seq", "Message"])
    w.define("CAM", cam_fmt, CAM_COLS)
    w.define("GPS", "QBBIHBcLLeffffB", ["TimeUS", "I", "Status", "GMS", "GWk", "NSats", "HDop", "Lat", "Lng",
                                        "Alt", "Spd", "GCrs", "VZ", "Yaw", "U"])
    if gpa_field == "AEl":
        w.define("GPA", "QBCCCCfBIHeHH", ["TimeUS", "I", "VDop", "HAcc", "VAcc", "SAcc", "YAcc", "VV", "SMS",
                                          "Delta", "AEl", "RTCMFU", "RTCMFD"])
    else:
        w.define("GPA", "QBCCCCfBIHfHH", ["TimeUS", "I", "VDop", "HAcc", "VAcc", "SAcc", "YAcc", "VV", "SMS",
                                          "Delta", "Und", "RTCMFU", "RTCMFD"])
    w.define("ATT", "QffffffB", ["TimeUS", "DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw", "AEKF"])
    w.define("POS", "QLLfff", ["TimeUS", "Lat", "Lng", "Alt", "RelHomeAlt", "RelOriginAlt"])
    w.define("XKF1", "QBccCfffffffccce", ["TimeUS", "C", "Roll", "Pitch", "Yaw", "VN", "VE", "VD", "dPD", "PN",
                                          "PE", "PD", "GX", "GY", "GZ", "OH"])
    w.define("ORGN", "QBLLe", ["TimeUS", "Type", "Lat", "Lng", "Alt"])
    w.define("MODE", "QMBB", ["TimeUS", "Mode", "ModeNum", "Rsn"])
    w.define("CMD", "QHHHffffLLfB", ["TimeUS", "CTot", "CNum", "CId", "Prm1", "Prm2", "Prm3", "Prm4", "Lat",
                                     "Lng", "Alt", "Frame"])
    w.define("MISE", "QHHHffffLLfB", ["TimeUS", "CTot", "CNum", "CId", "Prm1", "Prm2", "Prm3", "Prm4", "Lat",
                                      "Lng", "Alt", "Frame"])


# ── 合成建圖航線（真值） ────────────────────────────────────────────────
LAT0, LON0 = 25.10, 121.20
_AP_SCALE = 0.011131884502145034 * 1e7       # ArduPilot Location 的 m/deg


class SurveyTruth:
    """東西向往返作業帶＋半圓轉彎，等速；位置以 IMU 原點為準（ENU，m）。"""

    def __init__(self, *, n_strips=2, strip_len=500.0, spacing=120.0, speed=21.5, lead=150.0,
                 h_amsl=150.0, crab_deg=5.0, pitch_deg=2.0, t_start=200.0):
        self.v, self.h, self.crab, self.pitch, self.t0 = speed, h_amsl, crab_deg, pitch_deg, t_start
        self.R = spacing / 2.0
        segs = [("line", (-lead, 0.0), (0.0, 0.0), None)]
        y = 0.0
        for k in range(n_strips):
            x0, x1 = (0.0, strip_len) if k % 2 == 0 else (strip_len, 0.0)
            segs.append(("strip", (x0, y), (x1, y), k + 1))
            if k < n_strips - 1:
                segs.append(("turn", (x1, y), +1 if k % 2 == 0 else -1, None))
                y += spacing
        xe = strip_len if (n_strips - 1) % 2 == 0 else 0.0
        segs.append(("line", (xe, y), (xe + (lead if (n_strips - 1) % 2 == 0 else -lead), y), None))
        self.segs = []
        s = 0.0
        for sg in segs:
            L = math.pi * self.R if sg[0] == "turn" else math.dist(sg[1], sg[2])
            self.segs.append((sg, s, s + L))
            s += L
        self.total_s = s
        self.t_end = t_start + s / speed
        self.tm = Transformer.from_crs(f"+proj=tmerc +lat_0={LAT0} +lon_0={LON0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",
                                       "EPSG:4326", always_xy=True)

    def _at(self, t):
        s = min(max((t - self.t0) * self.v, 0.0), self.total_s - 1e-9)
        for sg, a, b in self.segs:
            if a <= s < b:
                u = s - a
                if sg[0] in ("line", "strip"):
                    (x0, y0), (x1, y1) = sg[1], sg[2]
                    L = b - a
                    dx, dy = (x1 - x0) / L, (y1 - y0) / L
                    return (x0 + dx * u, y0 + dy * u), (dx, dy), 0.0, sg[3] if sg[0] == "strip" else None
                (xs, ys), sense = sg[1], sg[2]
                th = u / self.R
                # 東端：由東向轉西向（左轉，CCW）；西端：由西向轉東向（右轉，CW），圓心都在北邊 R 處
                if sense > 0:
                    cx, cy, ang = xs, ys + self.R, -math.pi / 2 + th
                    pos = (cx + self.R * math.cos(ang), cy + self.R * math.sin(ang))
                    d = (-math.sin(ang), math.cos(ang))
                    bank = -math.degrees(math.atan(self.v ** 2 / (9.80665 * self.R)))
                else:
                    cx, cy, ang = xs, ys + self.R, -math.pi / 2 - th
                    pos = (cx + self.R * math.cos(ang), cy + self.R * math.sin(ang))
                    d = (math.sin(ang), -math.cos(ang))
                    bank = math.degrees(math.atan(self.v ** 2 / (9.80665 * self.R)))
                return pos, d, bank, None
        raise AssertionError(s)

    def state(self, t):
        """回傳 dict：e, n（IMU，m）、u（AMSL）、vE, vN, vU、roll, pitch, yaw（度）、strip。"""
        (e, n), (dx, dy), bank, strip = self._at(t)
        track = math.degrees(math.atan2(dx, dy)) % 360.0
        return dict(e=e, n=n, u=self.h, vE=self.v * dx, vN=self.v * dy, vU=0.0, roll=bank, pitch=self.pitch,
                    yaw=(track + self.crab) % 360.0, track=track, strip=strip)

    def to_llh(self, e, n, u):
        lon, lat = self.tm.transform(e, n)
        return lat, lon, u

    def point_llh(self, t, body_offset_frd=(0.0, 0.0, 0.0)):
        """IMU（或其上機體偏移處）在 t 的 (lat, lon, AMSL)。"""
        st = self.state(t)
        rot = Rotation.from_euler("ZYX", [st["yaw"], st["pitch"], st["roll"]], degrees=True)
        dn, de, dd = rot.apply(np.asarray(body_offset_frd, float))
        return self.to_llh(st["e"] + de, st["n"] + dn, st["u"] - dd)


def strip_intervals(truth):
    out = []
    for sg, a, b in truth.segs:
        if sg[0] == "strip":
            out.append((sg[3], truth.t0 + a / truth.v, truth.t0 + b / truth.v))
    return out


def write_survey_log(path, truth, *, d=18.85, rate_hz=50, cam_fmt=CAM_FMT_FLOAT, gpa_field="AEl",
                     undulation_n=18.54, gps_pos=(0.0, 0.0, 0.0), gps_lag_s=0.08, gpst_at_boot=1_444_000_000.0,
                     firmware="ArduPlane V4.5.7 (0123abcd)", extra_cam_times=(), new_param_names=True,
                     drop_mise=True):
    """把真值航線寫成 .BIN；回傳 {'cam_t_us': [...], 'windows': [(strip, t_on_us, t_off_us)]}。"""
    w = DFWriter()
    define_standard(w, cam_fmt=cam_fmt, gpa_field=gpa_field)
    arm_us = int((truth.t0 - 5.0) * 1e6)
    w.write("MSG", TimeUS=arm_us - 1000, Message=firmware)
    names = ("GPS1_POS_X", "GPS1_POS_Y", "GPS1_POS_Z") if new_param_names else ("GPS_POS1_X", "GPS_POS1_Y", "GPS_POS1_Z")
    if gps_pos is None:                       # 模擬日誌沒有天線位置參數
        gps_pos = (0.0, 0.0, 0.0)
    else:
        for nm, v in zip(names, gps_pos):
            w.write("PARM", TimeUS=arm_us - 900, Name=nm, Value=v, Default=0.0)
    w.write("PARM", TimeUS=arm_us - 900, Name="CAM1_TYPE", Value=2.0, Default=0.0)
    w.write("PARM", TimeUS=arm_us - 900, Name="CAM1_DURATION", Value=0.1, Default=0.1)
    olat, olon = LAT0 - 0.001, LON0 - 0.001
    w.write("ORGN", TimeUS=arm_us - 800, Type=0, Lat=olat, Lng=olon, Alt=40.0)
    w.write("MODE", TimeUS=arm_us, Mode=10, ModeNum=10, Rsn=2)
    # 任務：每條帶 NAV(起) → 206 ON → NAV(終) → 206 OFF
    strips = strip_intervals(truth)
    items = [(1, 16, 0.0)]
    cn = 2
    plan = []
    for k, t_on, t_off in strips:
        items += [(cn, 206, d), (cn + 1, 16, 0.0), (cn + 2, 206, 0.0)]
        plan.append((k, cn, cn + 2, t_on, t_off))
        cn += 3
    for c, cid, p1 in items:
        w.write("CMD", TimeUS=arm_us, CTot=len(items) + 1, CNum=c, CId=cid, Prm1=p1, Prm3=1.0 if p1 > 0 else 0.0)
    # 時間軸：50 Hz tick
    dt = 1.0 / rate_hz
    ticks = np.arange(truth.t0 - 2.0, truth.t_end - 0.05, dt)
    events = []                                   # (t, kind, payload)
    for k, con, coff, t_on, t_off in plan:
        events.append((t_on, "mission", (con, 206, d)))
        events.append((t_off, "mission", (coff, 206, 0.0)))
    cam_t = []
    last = None
    img = 0
    win = []
    for k, con, coff, t_on, t_off in plan:
        t_on_tick = ticks[np.searchsorted(ticks, t_on)]
        t_off_tick = ticks[np.searchsorted(ticks, t_off)]
        win.append((k, int(round(t_on_tick * 1e6)), int(round(t_off_tick * 1e6))))
    for t in ticks:
        tu = int(round(t * 1e6))
        st = truth.state(t)
        lat, lon, alt = truth.point_llh(t)
        # 任務事件（206 開關）在 tick 上
        for k, ton, toff in win:
            if tu == ton:
                if not drop_mise:
                    w.write("MISE", TimeUS=tu, CTot=len(items) + 1, CNum=plan[k - 1][1], CId=206, Prm1=d, Prm3=1.0)
                w.write("MSG", TimeUS=tu, Message=f"Mission: {plan[k - 1][1]} SetCamTrigDst")
                img += 1
                cam_t.append(tu)
                last = (st["e"], st["n"])
                _write_cam(w, tu, img, lat, lon, alt, st)
            elif tu == toff:
                w.write("MISE", TimeUS=tu, CTot=len(items) + 1, CNum=plan[k - 1][2], CId=206, Prm1=0.0)
                w.write("MSG", TimeUS=tu, Message=f"Mission: {plan[k - 1][2]} SetCamTrigDst")
                last = None
            elif ton < tu < toff and last is not None and math.dist(last, (st["e"], st["n"])) >= d:
                img += 1
                cam_t.append(tu)
                last = (st["e"], st["n"])
                _write_cam(w, tu, img, lat, lon, alt, st)
        for te in extra_cam_times:
            if abs(t - te) < dt / 2:
                img += 1
                cam_t.append(tu)
                _write_cam(w, tu, img, lat, lon, alt, st)
        w.write("ATT", TimeUS=tu, Roll=st["roll"], Pitch=st["pitch"], Yaw=st["yaw"], AEKF=3)
        w.write("POS", TimeUS=tu, Lat=lat, Lng=lon, Alt=alt, RelHomeAlt=alt - 40.0, RelOriginAlt=alt - 40.0)
        dlat, dlon = lat - olat, lon - olon
        pn = dlat * _AP_SCALE
        pe = dlon * _AP_SCALE * max(math.cos(math.radians(olat + dlat / 2)), 0.01)
        w.write("XKF1", TimeUS=tu, C=0, Roll=st["roll"], Pitch=st["pitch"], Yaw=st["yaw"], VN=st["vN"],
                VE=st["vE"], VD=-st["vU"], PN=pn, PE=pe, PD=-(alt - 40.0))
        # GPS 5 Hz：飛控在 epoch + 延遲後才處理並寫入 ⇒ TimeUS = 現在、GMS = 較早的量測 epoch
        te = t - gps_lag_s
        if int(round(te * rate_hz)) % (rate_hz // 5) == 0:
            ste = truth.state(te)
            alat, alon, aalt = truth.point_llh(te, gps_pos)
            gpst = gpst_at_boot + te
            wk = int(gpst // 604800)
            gms = int(round((gpst - wk * 604800) * 1000))
            gu = tu
            w.write("GPS", TimeUS=gu, I=0, Status=3, GMS=gms, GWk=wk, NSats=14, HDop=0.7, Lat=alat, Lng=alon,
                    Alt=aalt, Spd=math.hypot(ste["vE"], ste["vN"]), GCrs=ste["track"], VZ=-ste["vU"], U=1)
            if gpa_field == "AEl":
                w.write("GPA", TimeUS=gu, I=0, VDop=1.0, HAcc=0.5, VAcc=0.8, SAcc=0.1, AEl=aalt + undulation_n)
            else:
                # ArduPilot 慣例：Und = AMSL − h_ell = −N
                w.write("GPA", TimeUS=gu, I=0, VDop=1.0, HAcc=0.5, VAcc=0.8, SAcc=0.1, Und=-undulation_n)
    w.save(path)
    return {"cam_t_us": cam_t, "windows": win, "gpst_at_boot": gpst_at_boot}


def _write_cam(w, tu, img, lat, lon, alt, st):
    w.write("CAM", TimeUS=tu, I=0, Img=img, GPSTime=0, GPSWeek=0, Lat=lat, Lng=lon, Alt=alt, RelAlt=alt - 40.0,
            GPSAlt=alt, R=st["roll"], P=st["pitch"], Y=st["yaw"])


# ── 工具本身的測試 ──────────────────────────────────────────────────────

def test_dfwriter_roundtrip_through_dfreader(tmp_path):
    """寫入器產生的檔案 DFReader 讀得回來，且數值（含 'c' 百分度、'L' 1e-7 度）正確。"""
    w = DFWriter()
    w.define("CAM", CAM_FMT_CENTIDEG, CAM_COLS)
    w.write("CAM", TimeUS=1_000_000, I=0, Img=7, Lat=25.1234567, Lng=121.7654321, Alt=123.45, R=-12.34, P=5.67,
            Y=350.25)
    p = w.save(tmp_path / "x.BIN")
    r = DFReader.DFReader_binary(str(p))
    m = r.recv_match(type="CAM")
    assert m.Img == 7 and m.TimeUS == 1_000_000
    assert m.Lat == pytest.approx(25.1234567, abs=1e-9) and m.Lng == pytest.approx(121.7654321, abs=1e-9)
    assert m.Alt == pytest.approx(123.45) and m.R == pytest.approx(-12.34) and m.Y == pytest.approx(350.25)


def test_survey_truth_geometry():
    """真值航線本身：帶長、等速、轉彎半徑與偏航＝航跡＋偏流。"""
    tr = SurveyTruth(n_strips=2, strip_len=500.0, spacing=120.0)
    st = strip_intervals(tr)
    assert [k for k, _, _ in st] == [1, 2]
    assert (st[0][2] - st[0][1]) * tr.v == pytest.approx(500.0)
    mid = tr.state((st[0][1] + st[0][2]) / 2)
    assert mid["track"] == pytest.approx(90.0) and mid["yaw"] == pytest.approx(95.0) and mid["roll"] == 0.0
    mid2 = tr.state((st[1][1] + st[1][2]) / 2)
    assert mid2["track"] == pytest.approx(270.0) and mid2["n"] == pytest.approx(120.0)
    turn_mid = tr.state(st[0][2] + math.pi * tr.R / 2 / tr.v)
    assert turn_mid["e"] == pytest.approx(500.0 + tr.R) and turn_mid["track"] == pytest.approx(0.0, abs=1e-6)
