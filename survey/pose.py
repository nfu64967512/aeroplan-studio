"""每張照片在曝光中心時刻的位姿（相機中心經緯高＋真北航向＋姿態＋速度）→ F2 photos.csv 的列。

時間鏈
======
曝光中心（BOOTTIME）＝ SensorTimestamp ± 曝光/2 ＋ 讀出/2：

* ``sensor_ts_semantics='exposure_start'``：SensorTimestamp 是第一列**開始曝光**的時刻
  （libcamera control_ids：“The time when the first row of the image sensor active array is
  exposed”）⇒ 中心 = ts + E/2 + R/2。
* ``'readout_start'``：SensorTimestamp 是第一個像素**讀出**的時刻（picamera2 手冊的說法）
  ⇒ 第一列曝光結束 ⇒ 中心 = ts − E/2 + R/2。

兩份文件互相矛盾，台架 B3 定案前沒有預設值 —— 呼叫端必須明講（評估 §7、phase2 time_sync）。
讀出時間 R 取 session.json 的實測值；是 null 時用 0 並加旗標 ``readout_unknown``（滾動快門
中心修正等於沒做，旗標讓它看得到）。

再用時鐘擬合換到飛控時鐘，對日誌內插：位置（POS 或 XKF1，取頻率較高者；線性）、姿態
（ATT；偏航取最短角）、速度（XKF1，沒有就用 GPS）。**不直接用 CAM 的經緯度**：Pi 挑幀
有 0–100 ms 延遲，21.5 m/s 下最多 2 m（評估 §3、§14 K12）。

槓桿臂：session.json 的 ``lever_arm_m`` 是「相機中心 − 飛控 GNSS 天線」（機體 FRD）。
EKF 輸出的是 IMU 原點的位置（GPS_POSn 偏移已在 EKF 內扣掉），所以位置來源是 POS／XKF1 時
有效槓桿臂 = lever_arm_m + 天線偏移（參數 GPS1_POS_X/Y/Z，舊名 GPS_POS1_X/Y/Z）。
來源是原始 GPS（天線本身）時只用 lever_arm_m。有效值寫進 ``effective``。
"""

from __future__ import annotations

import logging
import math
import re
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from survey import datum, dflog

_log = logging.getLogger(__name__)

SENSOR_TS_SEMANTICS = ("exposure_start", "readout_start")
POSITION_SOURCES = ("auto", "POS", "XKF1", "GPS")
VELOCITY_SOURCES = ("auto", "XKF1", "GPS")
SOURCE_FC_INTERP = "fc_interp"
CRS_H_WGS84 = "EPSG:4326"
_SAFE_ID = re.compile(r"[^A-Za-z0-9_-]+")


@dataclass
class PoseResult:
    rows: list                         # F2 欄位的 dict（None = 空欄）
    skipped: list                      # [{'seq', 'reason'}]
    effective: dict                    # 實際採用的來源、槓桿臂、讀出時間…（有效值契約）
    flags: list = field(default_factory=list)


def image_id(uav: str, session_id: str, seq: int) -> str:
    """跨機、跨架次唯一的影像代號（也用來命名匯出的影像檔）。"""
    return f"{_SAFE_ID.sub('-', str(uav))}_{_SAFE_ID.sub('-', str(session_id))}_{int(seq):06d}"


def exposure_center_ns(sensor_ts_ns: int, exposure_us: Optional[int], readout_ms: Optional[float],
                       semantics: str):
    """曝光中心（與 sensor_ts 同一時鐘，ns）與旗標。"""
    if semantics not in SENSOR_TS_SEMANTICS:
        raise ValueError(f"sensor_ts_semantics 必須是 {SENSOR_TS_SEMANTICS} 之一，收到 {semantics!r}")
    flags = []
    if exposure_us is None:
        e_ns = 0
        flags.append("exposure_unknown")
    else:
        e_ns = int(exposure_us) * 1000
    if readout_ms is None:
        r_ns = 0
        flags.append("readout_unknown")
    else:
        r_ns = int(round(float(readout_ms) * 1e6))
    half_e = e_ns // 2 if semantics == "exposure_start" else -(e_ns // 2)
    return int(sensor_ts_ns) + half_e + r_ns // 2, flags


def _rate(t):
    return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0


def _position_series(log, source, ekf_core, gps_instance):
    if source == "POS":
        t = log.table("POS")
        if not t:
            return None
        return dict(name="POS", t=t["TimeUS"] * 1e-6, lat=np.asarray(t["Lat"], float),
                    lon=np.asarray(t["Lng"], float), alt=np.asarray(t["Alt"], float))
    if source == "XKF1":
        x, o = log.table("XKF1"), log.table("ORGN")
        if not x or not o:
            return None
        sel = np.asarray(x["C"]) == ekf_core if "C" in x else np.ones(len(x["TimeUS"]), bool)
        osel = np.asarray(o["Type"]) == 0 if "Type" in o else np.ones(len(o["TimeUS"]), bool)
        if not sel.any() or not osel.any():
            return None
        ot = np.asarray(o["TimeUS"])[osel]
        tx = np.asarray(x["TimeUS"])[sel]
        k = np.clip(np.searchsorted(ot, tx, side="right") - 1, 0, len(ot) - 1)   # 當時有效的 EKF 原點
        lat, lon, alt = datum.ap_ned_to_llh(np.asarray(o["Lat"], float)[osel][k], np.asarray(o["Lng"], float)[osel][k],
                                            np.asarray(o["Alt"], float)[osel][k], np.asarray(x["PN"], float)[sel],
                                            np.asarray(x["PE"], float)[sel], np.asarray(x["PD"], float)[sel])
        return dict(name=f"XKF1[C={ekf_core}]", t=tx * 1e-6, lat=lat, lon=lon, alt=alt)
    if source == "GPS":
        g = log.table("GPS")
        if not g:
            return None
        sel = np.asarray(g["Status"]) >= 3
        if "I" in g:
            sel &= np.asarray(g["I"]) == gps_instance
        if not sel.any():
            return None
        return dict(name="GPS", t=np.asarray(g["TimeUS"])[sel] * 1e-6, lat=np.asarray(g["Lat"], float)[sel],
                    lon=np.asarray(g["Lng"], float)[sel], alt=np.asarray(g["Alt"], float)[sel])
    raise ValueError(f"position_source 必須是 {POSITION_SOURCES} 之一，收到 {source!r}")


def _velocity_series(log, source, ekf_core, gps_instance):
    if source == "XKF1":
        x = log.table("XKF1")
        if not x:
            return None
        sel = np.asarray(x["C"]) == ekf_core if "C" in x else np.ones(len(x["TimeUS"]), bool)
        if not sel.any():
            return None
        return dict(name=f"XKF1[C={ekf_core}]", t=np.asarray(x["TimeUS"])[sel] * 1e-6,
                    vn=np.asarray(x["VN"], float)[sel], ve=np.asarray(x["VE"], float)[sel],
                    vd=np.asarray(x["VD"], float)[sel])
    if source == "GPS":
        g = log.table("GPS")
        if not g:
            return None
        sel = np.asarray(g["Status"]) >= 3
        if "I" in g:
            sel &= np.asarray(g["I"]) == gps_instance
        if not sel.any():
            return None
        spd = np.asarray(g["Spd"], float)[sel]
        crs = np.radians(np.asarray(g["GCrs"], float)[sel])
        # GPS.VZ = velocity(i).z，NED 向下為正（AP_GPS.cpp Write_GPS）
        return dict(name="GPS", t=np.asarray(g["TimeUS"])[sel] * 1e-6, vn=spd * np.cos(crs),
                    ve=spd * np.sin(crs), vd=np.asarray(g["VZ"], float)[sel])
    raise ValueError(f"velocity_source 必須是 {VELOCITY_SOURCES} 之一，收到 {source!r}")


def _interp(ts, ys, t, max_gap_s):
    """線性內插；回傳 (值, 是否在範圍內, 前後樣本間隔是否 ≤ max_gap_s)。"""
    ts = np.asarray(ts, float)
    k = np.clip(np.searchsorted(ts, t), 1, len(ts) - 1)
    inside = (t >= ts[0]) & (t <= ts[-1])
    gap_ok = (ts[k] - ts[k - 1]) <= max_gap_s
    return np.interp(t, ts, ys), inside, gap_ok


def _gps_antenna_offset(log, gps_instance):
    n = gps_instance + 1
    for names in ((f"GPS{n}_POS_X", f"GPS{n}_POS_Y", f"GPS{n}_POS_Z"),
                  (f"GPS_POS{n}_X", f"GPS_POS{n}_Y", f"GPS_POS{n}_Z")):
        if all(k in log.params for k in names):
            return np.asarray([log.params[k] for k in names], float), names[0][:-2]
    return None, None


def body_to_ned(roll_deg, pitch_deg, yaw_deg, v_frd):
    """機體 FRD 向量 → NED（ZYX 歐拉角，ArduPilot ATT 的慣例）。可對多個姿態向量化。"""
    r, p, y = (np.radians(np.asarray(a, float)) for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    x, yb, z = v_frd
    n = cp * cy * x + (sr * sp * cy - cr * sy) * yb + (cr * sp * cy + sr * sy) * z
    e = cp * sy * x + (sr * sp * sy + cr * cy) * yb + (cr * sp * sy - sr * cy) * z
    d = -sp * x + sr * cp * yb + cr * cp * z
    return n, e, d


def compute_photo_poses(log, session, fit, fc_events, *, sensor_ts_semantics: str,
                        geoid: Optional[datum.GeoidModel] = None, fc_amsl_datum: Optional[str] = None,
                        readout_ms: Optional[float] = None, position_source: str = "auto",
                        velocity_source: str = "auto", ekf_core: int = 0, gps_instance: int = 0,
                        max_gap_s: float = 0.5, roll_flag_deg: float = 10.0, gps_lag_s: float = 0.0,
                        windows=None) -> PoseResult:
    """``fit``／``fc_events`` 必須是同一次 :func:`survey.ingest.fit_session` 的結果
    （配對表的索引指向 ``fc_events`` 與 ``session.frames``）。"""
    if sensor_ts_semantics not in SENSOR_TS_SEMANTICS:
        raise ValueError(f"sensor_ts_semantics 必須是 {SENSOR_TS_SEMANTICS} 之一，收到 {sensor_ts_semantics!r}")
    run_flags = []

    # ── 讀出時間：session.json 為準；呼叫端覆寫要大聲講 ──
    sess_ro = session.readout_ms
    if readout_ms is not None:
        if sess_ro is not None and abs(float(readout_ms) - sess_ro) > 1e-9:
            _log.warning("會期 %s：讀出時間以呼叫端 %.3f ms 覆寫 session.json 的 %.3f ms",
                         session.session_id, readout_ms, sess_ro)
        eff_ro, ro_src = float(readout_ms), "override"
    else:
        eff_ro, ro_src = sess_ro, ("session" if sess_ro is not None else "unknown")
    if eff_ro is None:
        run_flags.append("readout_unknown")

    # ── 位置來源 ──
    if position_source == "auto":
        cands = [s for s in (_position_series(log, "POS", ekf_core, gps_instance),
                             _position_series(log, "XKF1", ekf_core, gps_instance)) if s is not None]
        pos = max(cands, key=lambda s: _rate(s["t"])) if cands else _position_series(log, "GPS", ekf_core, gps_instance)
    else:
        pos = _position_series(log, position_source, ekf_core, gps_instance)
    if pos is None:
        raise ValueError(f"日誌裡沒有可用的位置來源（要求 {position_source}）")
    if pos["name"] == "GPS":
        run_flags.append("pos_from_gps")
    if velocity_source == "auto":
        vel = _velocity_series(log, "XKF1", ekf_core, gps_instance) or _velocity_series(log, "GPS", ekf_core, gps_instance)
    else:
        vel = _velocity_series(log, velocity_source, ekf_core, gps_instance)
    att = log.table("ATT")
    if not att:
        raise ValueError("日誌沒有 ATT 紀錄，無法內插姿態")
    att_t = np.asarray(att["TimeUS"]) * 1e-6
    yaw_unwrapped = np.degrees(np.unwrap(np.radians(np.asarray(att["Yaw"], float))))

    # ── 槓桿臂 ──
    lever = session.lever_arm_m
    ant_off, ant_name = (None, None)
    if pos["name"] != "GPS":
        ant_off, ant_name = _gps_antenna_offset(log, gps_instance)
        if ant_off is None:
            run_flags.append("gps_pos_param_missing")
            _log.warning("日誌沒有 GPS 天線位置參數：假設天線在 IMU 原點（槓桿臂只用 session.json 的值）")
    eff_lever = lever + (ant_off if ant_off is not None else 0.0)

    rx = datum.receiver_undulation_from_log(log, gps_instance)
    try:
        gfit = dflog.gps_time_fit(log, instance=gps_instance, lag_s=gps_lag_s)
    except ValueError as e:
        gfit = None
        run_flags.append("gpst_unknown")
        _log.warning("無法換算 GPS 時間：%s", e)
    gpa = log.table("GPA")
    if gpa and "I" in gpa:
        gsel = np.asarray(gpa["I"]) == gps_instance
        gpa = {k: np.asarray(v)[gsel] for k, v in gpa.items()}
    if windows is None:
        windows = dflog.strip_windows(log)

    paired = {int(pi): (int(fc), bool(inl)) for (fc, pi), inl in zip(fit.pairs, fit.inlier)}
    fc_t_us = np.asarray(fc_events["TimeUS"]) if fc_events else np.zeros(0)

    rows, skipped, todo = [], [], []
    for k, fr in enumerate(session.frames):
        if fr.status != "ok":
            skipped.append({"seq": fr.seq, "reason": f"status:{fr.status}"})
        elif fr.sensor_ts_ns is None:
            skipped.append({"seq": fr.seq, "reason": "sensor_ts_missing"})
        elif fr.seq in session.missing_files:
            skipped.append({"seq": fr.seq, "reason": "file_missing"})
        elif fr.seq in session.sha256_mismatch:
            skipped.append({"seq": fr.seq, "reason": "sha256_mismatch"})
        else:
            todo.append(k)

    if todo:
        centers, fflags = [], []
        for k in todo:
            fr = session.frames[k]
            c, fl = exposure_center_ns(fr.sensor_ts_ns, fr.exposure_us, eff_ro, sensor_ts_semantics)
            centers.append(c)
            fflags.append(fl)
        t_fc = fit.pi_to_fc(np.asarray(centers, np.float64) * 1e-9)
        lat, in_p, gap_p = _interp(pos["t"], pos["lat"], t_fc, max_gap_s)
        lon, _, _ = _interp(pos["t"], pos["lon"], t_fc, max_gap_s)
        alt, _, _ = _interp(pos["t"], pos["alt"], t_fc, max_gap_s)
        roll, in_a, gap_a = _interp(att_t, np.asarray(att["Roll"], float), t_fc, max_gap_s)
        pitch, _, _ = _interp(att_t, np.asarray(att["Pitch"], float), t_fc, max_gap_s)
        yaw = np.mod(_interp(att_t, yaw_unwrapped, t_fc, max_gap_s)[0], 360.0)
        if vel is not None:
            vn, in_v, gap_v = _interp(vel["t"], vel["vn"], t_fc, max_gap_s)
            ve, _, _ = _interp(vel["t"], vel["ve"], t_fc, max_gap_s)
            vd, _, _ = _interp(vel["t"], vel["vd"], t_fc, max_gap_s)
        dn, de, dd = body_to_ned(roll, pitch, yaw, eff_lever)
        clat, clon, calt = datum.ned_offset_to_llh(lat, lon, alt, dn, de, dd)

        for n, k in enumerate(todo):
            fr = session.frames[k]
            flags = list(fflags[n])
            if not (in_p[n] and in_a[n]):
                skipped.append({"seq": fr.seq, "reason": "pose_out_of_log_range"})
                continue
            pair = paired.get(k)
            t_trig_us = None
            if pair is None:
                flags.append("unpaired_edge")
            else:
                t_trig_us = int(fc_t_us[pair[0]])
                if not pair[1]:
                    flags.append("fit_outlier")
            if not gap_p[n]:
                flags.append("pos_gap")
            if not gap_a[n]:
                flags.append("att_gap")
            t_us = t_fc[n] * 1e6
            strip = dflog.strip_of(windows, t_trig_us if t_trig_us is not None else t_us)
            if strip is None:
                flags.append("off_strip")
            if abs(roll[n]) > roll_flag_deg:
                flags.append(f"roll_gt_{roll_flag_deg:g}")
            if vel is not None and in_v[n]:
                if not gap_v[n]:
                    flags.append("vel_gap")
                v = (float(vn[n]), float(ve[n]), float(vd[n]))
                gs = math.hypot(v[0], v[1])
                cog = math.degrees(math.atan2(v[1], v[0])) % 360.0 if gs >= 0.5 else None
                if cog is None:
                    flags.append("cog_undefined")
            else:
                v, cog = (None, None, None), None
                flags.append("velocity_unknown")
            h_ell, h_orth, crs_v, hfl = datum.fc_heights(calt[n], clat[n], clon[n], receiver=rx,
                                                         geoid=geoid, fc_amsl_datum=fc_amsl_datum)
            flags += hfl
            sig_h = sig_v = None
            if gpa and "HAcc" in gpa:
                g = int(np.searchsorted(gpa["TimeUS"], t_us, side="right")) - 1
                if g >= 0:
                    sig_h, sig_v = float(gpa["HAcc"][g]), float(gpa["VAcc"][g])
            if sig_h is None:
                flags.append("sigma_unknown")
            rows.append({
                "image_id": image_id(session.uav, session.session_id, fr.seq),
                "file": fr.file, "uav": session.uav, "strip": strip, "seq": fr.seq,
                "t_fc_us": int(round(t_us)),
                "t_gpst_s": float(gfit.gpst_s(t_us)) if gfit is not None else None,
                "lat_deg": float(clat[n]), "lon_deg": float(clon[n]),
                "h_ell_m": h_ell, "h_orth_m": h_orth,
                "yaw_deg": float(yaw[n]), "pitch_deg": float(pitch[n]), "roll_deg": float(roll[n]),
                "cog_deg": cog, "speed_n_mps": v[0], "speed_e_mps": v[1], "speed_d_mps": v[2],
                "sigma_h_m": sig_h, "sigma_v_m": sig_v,
                "source": SOURCE_FC_INTERP, "crs_h": CRS_H_WGS84, "crs_v": crs_v,
                "flags": ";".join(dict.fromkeys(flags)),
            })

    skipped.sort(key=lambda s: s["seq"])
    effective = {
        "position_source": pos["name"], "position_rate_hz": _rate(pos["t"]),
        "velocity_source": vel["name"] if vel is not None else None,
        "attitude_source": "ATT", "ekf_core": ekf_core, "gps_instance": gps_instance,
        "lever_arm_session_m": lever.tolist(),
        "gps_antenna_offset_m": ant_off.tolist() if ant_off is not None else None,
        "gps_antenna_param": ant_name,
        "lever_arm_effective_m": np.asarray(eff_lever, float).tolist(),
        "sensor_ts_semantics": sensor_ts_semantics,
        "readout_ms": eff_ro, "readout_source": ro_src,
        "receiver_undulation_m": rx.n_m, "receiver_undulation_source": rx.source,
        "geoid": None if geoid is None else {"datum": geoid.datum, "source": geoid.source},
        "fc_amsl_datum": fc_amsl_datum,
        "gps_time_fit": None if gfit is None else {"a": gfit.a, "c": gfit.c, "lag_s": gfit.lag_s,
                                                    "rms_s": gfit.rms_s, "n": gfit.n},
        "clock_fit": fit.summary(),
        "firmware": log.firmware,
    }
    return PoseResult(rows=rows, skipped=skipped, effective=effective, flags=run_flags)
