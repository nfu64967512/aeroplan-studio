"""落地後現場 QA：張數守恆、間距、側傾、時鐘殘差、帶外照片、足跡視數格網。

門檻（評估 §8 G4、§11 S1；phase2_verify V1 #3 的更正）：
* 間距**在飛控 CAM 紀錄上量**：中位數在 d 的 ±2% 內、p99 ≤ 1.2 d。Pi 影像因挑幀延遲
  抖動 ±11–14%，拿影像量一定不過。
* |roll| > 10° 的比例，**排除每條帶第一張**（206 的 p3=1 立即拍不經 CAM_MAX_ROLL 閘門）。
* 時鐘殘差 RMS ≤ 0.5 ms。
* 張數守恆：每條帶 飛控 CAM（＋TRIG）＝ Pi 邊緣 ＝ 成功存檔的 JPEG，任何不等都列出。

風（2026-10-15 使用者決定：首次建圖飛行只在風速 ≤ 3 m/s）：:func:`wind_report` 讀飛控 EKF 的風估計
（EKF3 = XKF2、EKF2 = NKF2 的 VWN／VWE；主要核心由 XKF4／NKF4 的 PI 逐時刻決定），逐帶報風速、風向
（「風從哪裡來」，氣象慣例）與超過上限的照片比例。這是**旗標**（寫進 QA 摘要的警告），不改變 ``passed``；
EKF 的風是估計值，不是實測。

足跡視數格網刻意保持簡單：相機固定在機身、正下視，影像上緣朝機頭（與 ODM compute_opk 的
假設相同），足跡是依姿態平移、依航向旋轉的矩形；不做地形起伏與透視梯形。
飛前的曝光層級驗證器是另一支（core/recon/exposure_verifier.py），這裡不相依。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from survey import datum, dflog
from survey.pose import body_to_ned

FIT_RMS_LIMIT_S = 0.5e-3
WIND_LIMIT_MPS = 3.0           # 使用者的建圖風速上限（2026-10-15：首飛 ≤ 3 m/s）
WIND_MAX_GAP_S = 2.0           # 風估計樣本之間超過這麼久就不內插（該張照片的風記為未知）


def _short_dist_m(lat1, lon1, lat2, lon2):
    """相鄰拍照點的水平距離（WGS84 中點曲率半徑；數十 m 內誤差 < 1 mm）。
    不用球面半徑：25°N 的子午圈半徑比 6371 km 小 0.44%，±2% 的門檻會被吃掉五分之一。"""
    lat1, lat2 = np.asarray(lat1, float), np.asarray(lat2, float)
    mid = (lat1 + lat2) / 2.0
    m, n = datum.wgs84_radii(mid)
    dn = np.radians(lat2 - lat1) * m
    de = np.radians(np.asarray(lon2, float) - np.asarray(lon1, float)) * n * np.cos(np.radians(mid))
    return np.hypot(dn, de)


def _strip_index(windows, t_us):
    """每個時刻所屬作業帶編號（不在任何帶內 = 0）。"""
    out = np.zeros(len(t_us), int)
    for w in windows:
        out[(t_us >= w.t_on_us) & (t_us < w.t_off_us)] = w.strip
    return out


def pi_edge_fc_times_us(session, fit, fc_events) -> np.ndarray:
    """每個 Pi 邊緣在飛控時鐘上的時刻：配上的用飛控事件本身的時刻（帶界上不會因抖動跑出去），
    沒配上的用擬合換算。"""
    t = fit.pi_to_fc(session.edge_boottime_ns().astype(np.float64) * 1e-9) * 1e6
    fc_t = np.asarray(fc_events["TimeUS"], float)
    for fc_i, pi_i in fit.pairs:
        t[int(pi_i)] = fc_t[int(fc_i)]
    return t


def conservation_table(log, session, fit, fc_events, windows) -> list:
    """每條帶一列（最後一列 strip=None 是帶外）：飛控 CAM／TRIG、Pi 邊緣、成功 JPEG 張數，
    以及該帶沒配上的事件數。張數相等不夠：同一條帶少一個邊緣又多一個雜訊邊緣時張數會剛好抵銷，
    所以 ``conserved`` 還要求該帶兩邊都沒有未配對事件。"""
    fc_strip = _strip_index(windows, np.asarray(fc_events["TimeUS"]))
    trig = log.table("TRIG")
    trig_strip = _strip_index(windows, np.asarray(trig["TimeUS"])) if trig else None
    pi_strip = _strip_index(windows, pi_edge_fc_times_us(session, fit, fc_events))
    ok = np.asarray([f.status == "ok" and f.seq not in session.missing_files
                     and f.seq not in session.sha256_mismatch for f in session.frames], bool)
    un_fc = fc_strip[np.asarray(fit.unmatched_fc, int)]
    un_pi = pi_strip[np.asarray(fit.unmatched_pi, int)]
    rows = []
    for s in [w.strip for w in windows] + [0]:
        r = {"strip": s if s else None,
             "fc_cam": int((fc_strip == s).sum()),
             "fc_trig": None if trig_strip is None else int((trig_strip == s).sum()),
             "pi_edges": int((pi_strip == s).sum()),
             "jpeg_ok": int(((pi_strip == s) & ok).sum()),
             "unmatched_fc": int((un_fc == s).sum()),
             "unmatched_pi": int((un_pi == s).sum())}
        counts = [r["fc_cam"], r["pi_edges"], r["jpeg_ok"]] + ([r["fc_trig"]] if r["fc_trig"] is not None else [])
        r["conserved"] = len(set(counts)) == 1 and r["unmatched_fc"] == 0 and r["unmatched_pi"] == 0
        rows.append(r)
    return rows


def spacing_stats(fc_events, windows, *, rel_tol: float = 0.02, p99_factor: float = 1.2) -> list:
    """每條帶相鄰 CAM 的水平間距（m），與該帶 206 的 d 比較。"""
    t = np.asarray(fc_events["TimeUS"])
    lat, lon = np.asarray(fc_events["Lat"], float), np.asarray(fc_events["Lng"], float)
    out = []
    for w in windows:
        sel = (t >= w.t_on_us) & (t < w.t_off_us)
        d = _short_dist_m(lat[sel][:-1], lon[sel][:-1], lat[sel][1:], lon[sel][1:]) if sel.sum() > 1 else np.zeros(0)
        r = {"strip": w.strip, "d_m": w.trigg_dist_m, "n_shots": int(sel.sum()), "n_intervals": int(d.size)}
        if d.size:
            med, p99 = float(np.median(d)), float(np.percentile(d, 99))
            r.update(median_m=med, p99_m=p99, min_m=float(d.min()), max_m=float(d.max()),
                     median_ok=abs(med / w.trigg_dist_m - 1.0) <= rel_tol,
                     p99_ok=p99 <= p99_factor * w.trigg_dist_m)
        else:
            r.update(median_m=None, p99_m=None, min_m=None, max_m=None, median_ok=False, p99_ok=False)
        out.append(r)
    return out


def roll_share(fc_events, windows, *, limit_deg: float = 10.0) -> dict:
    """帶內 |roll| > limit 的比例，每條帶第一張不計（CAM 的 R 欄）。"""
    t = np.asarray(fc_events["TimeUS"])
    roll = np.asarray(fc_events["R"], float)
    n = n_gt = 0
    for w in windows:
        idx = np.nonzero((t >= w.t_on_us) & (t < w.t_off_us))[0]
        idx = idx[1:]
        n += idx.size
        n_gt += int((np.abs(roll[idx]) > limit_deg).sum())
    return {"limit_deg": limit_deg, "n": n, "n_gt": n_gt, "share": (n_gt / n) if n else None,
            "excluded_first_per_strip": len(windows)}


def fit_stats(fit) -> dict:
    s = fit.summary()
    s["rms_ok"] = bool(fit.rms_s <= FIT_RMS_LIMIT_S)
    return s


@dataclass
class MultiplicityGrid:
    counts: np.ndarray          # (ny, nx) 每格被幾張照片看到
    lat0: float                 # 格網原點（西南角）
    lon0: float
    cell_m: float
    n_footprints: int
    n_skipped: int              # 沒有高度或 AGL ≤ 0 的照片

    def fraction_at_least(self, k: int, mask: Optional[np.ndarray] = None) -> float:
        """在 mask（預設：至少被看到一次的格子）內，被 ≥k 張看到的比例。"""
        m = (self.counts > 0) if mask is None else np.asarray(mask, bool)
        return float((self.counts[m] >= k).mean()) if m.any() else 0.0


def footprint_multiplicity(rows, *, focal_mm: float, pixel_um: float, width_px: int, height_px: int,
                           ground_h_m: float, height: str, cell_m: float = 5.0,
                           origin: Optional[tuple] = None) -> MultiplicityGrid:
    """每格視數。``height`` 選 'ellipsoidal'（h_ell_m）或 'orthometric'（h_orth_m），
    ``ground_h_m`` 必須是同一個基準的地面高 —— 不替呼叫端猜。"""
    key = {"ellipsoidal": "h_ell_m", "orthometric": "h_orth_m"}.get(height)
    if key is None:
        raise ValueError(f"height 必須是 'ellipsoidal' 或 'orthometric'，收到 {height!r}")
    across = width_px * pixel_um * 1e-3 / focal_mm        # 每 1 m AGL 的足跡寬（影像寬 ⟂ 機頭）
    along = height_px * pixel_um * 1e-3 / focal_mm        # 影像高 ∥ 機頭
    use, skipped = [], 0
    for r in rows:
        h = r.get(key)
        if h is None or r.get("lat_deg") is None or (h - ground_h_m) <= 0:
            skipped += 1
            continue
        use.append(r)
    if not use:
        return MultiplicityGrid(np.zeros((0, 0), int), 0.0, 0.0, cell_m, 0, skipped)
    lat = np.asarray([r["lat_deg"] for r in use])
    lon = np.asarray([r["lon_deg"] for r in use])
    agl = np.asarray([r[key] for r in use]) - ground_h_m
    roll = np.asarray([r.get("roll_deg") or 0.0 for r in use])
    pitch = np.asarray([r.get("pitch_deg") or 0.0 for r in use])
    yaw = np.asarray([r["yaw_deg"] for r in use])
    # 光軸（機體 +Z）與地面交點：依姿態把足跡中心平移
    zn, ze, zd = body_to_ned(roll, pitch, yaw, (0.0, 0.0, 1.0))
    if origin is None:
        lat0, lon0 = float(lat.min()), float(lon.min())
    else:
        lat0, lon0 = map(float, origin)
    e, n = datum.local_en_m(lat0, lon0, lat, lon)
    ce, cn = e + agl * ze / zd, n + agl * zn / zd
    half_l, half_w = along * agl / 2.0, across * agl / 2.0
    rad = np.hypot(half_l, half_w)
    x_min, y_min = float((ce - rad).min()), float((cn - rad).min())
    nx = int(math.ceil((float((ce + rad).max()) - x_min) / cell_m)) + 1
    ny = int(math.ceil((float((cn + rad).max()) - y_min) / cell_m)) + 1
    counts = np.zeros((ny, nx), int)
    ys = np.radians(yaw)
    fwd = np.stack([np.sin(ys), np.cos(ys)], 1)          # (E, N)
    rgt = np.stack([np.cos(ys), -np.sin(ys)], 1)
    for k in range(len(use)):
        i0 = max(int((cn[k] - rad[k] - y_min) / cell_m), 0)
        i1 = min(int((cn[k] + rad[k] - y_min) / cell_m) + 1, ny)
        j0 = max(int((ce[k] - rad[k] - x_min) / cell_m), 0)
        j1 = min(int((ce[k] + rad[k] - x_min) / cell_m) + 1, nx)
        gy, gx = np.mgrid[i0:i1, j0:j1]
        de = x_min + (gx + 0.5) * cell_m - ce[k]
        dn = y_min + (gy + 0.5) * cell_m - cn[k]
        inside = (np.abs(de * fwd[k, 0] + dn * fwd[k, 1]) <= half_l[k]) & \
                 (np.abs(de * rgt[k, 0] + dn * rgt[k, 1]) <= half_w[k])
        counts[i0:i1, j0:j1] += inside
    # 格網原點換回經緯度（西南角）
    m, nr = datum.wgs84_radii(lat0)
    glat0 = lat0 + math.degrees(y_min / m)
    glon0 = lon0 + math.degrees(x_min / (nr * math.cos(math.radians(lat0))))
    return MultiplicityGrid(counts, glat0, glon0, cell_m, len(use), skipped)


def sortie_report(log, session, fit, fc_events, rows, *, windows=None, rel_tol: float = 0.02,
                  p99_factor: float = 1.2, roll_limit_deg: float = 10.0) -> dict:
    """一個架次的 QA 摘要；``passed`` 只在所有硬性條件都過時為真。"""
    if windows is None:
        windows = dflog.strip_windows(log)
    cons = conservation_table(log, session, fit, fc_events, windows)
    spacing = spacing_stats(fc_events, windows, rel_tol=rel_tol, p99_factor=p99_factor)
    fs = fit_stats(fit)
    report = {
        "uav": session.uav, "session_id": session.session_id, "firmware": log.firmware,
        "n_strips": len(windows),
        "conservation": cons,
        "spacing": spacing,
        "roll": roll_share(fc_events, windows, limit_deg=roll_limit_deg),
        "fit": fs,
        "unmatched_fc": [{"index": int(i), "TimeUS": int(fc_events["TimeUS"][i]),
                          "Img": int(fc_events["Img"][i]) if "Img" in fc_events else None}
                         for i in fit.unmatched_fc],
        "unmatched_pi": [{"index": int(i), "seq": session.frames[int(i)].seq} for i in fit.unmatched_pi],
        "off_strip": [r["image_id"] for r in rows if "off_strip" in (r.get("flags") or "").split(";")],
        "session_flags": list(session.flags),
        "seq_gaps": list(session.seq_gaps),
    }
    report["passed"] = bool(all(c["conserved"] for c in cons)
                            and all(s["median_ok"] and s["p99_ok"] for s in spacing)
                            and fs["rms_ok"] and not report["unmatched_fc"] and not report["unmatched_pi"])
    return report


# ── 風（EKF 估計）────────────────────────────────────────────────────────

def _wind_unit_problem(log, name) -> Optional[str]:
    """VWN／VWE 的解碼方式不是「m/s」就回報原因（不靠版本號猜）。"""
    spec = getattr(log, "column_specs", {}).get(name) or {}
    for c in ("VWN", "VWE"):
        if c not in spec:
            return f"{name} 沒有 {c} 欄"
        ch, mult, unit = spec[c]
        if ch in "cC":
            if mult is None or not math.isclose(mult, 0.01):
                return f"{name}.{c} 格式 {ch!r} 的乘數是 {mult}，預期 0.01"
        elif ch in "fd":
            if mult is not None:
                return f"{name}.{c} 格式 {ch!r} 不該有乘數，DFReader 給了 {mult}"
        else:
            return f"{name}.{c} 格式字元 {ch!r} 無法判讀單位"
        if unit and unit != "m/s":
            return f"{name}.{c} 的 FMTU 單位是 {unit!r}，不是 m/s"
    return None


def _wind_source(log):
    """(訊息名, 不可用的原因, 要用的紀錄)。EKF2 的 NKF2／NKF4 預設不讀（dflog.DEFAULT_TYPES）⇒ 需要時另讀一次。"""
    ekf = log.params.get("AHRS_EKF_TYPE")
    if ekf is None:
        cands = ("XKF2", "NKF2")
    elif int(ekf) == 3:
        cands = ("XKF2",)
    elif int(ekf) == 2:
        cands = ("NKF2",)
    else:
        return None, f"AHRS_EKF_TYPE = {ekf:g}：不是 EKF2／EKF3，日誌沒有 EKF 風估計", log
    for name in cands:
        src = log
        if name == "NKF2" and not log.count("NKF2") and "NKF2" not in getattr(log, "formats", {}):
            path = getattr(log, "path", None)
            if path and Path(path).is_file():
                src = dflog.read_dflog(path, types=list(dflog.EKF2_WIND_TYPES))
        if src.count(name):
            return name, "", src
    return None, f"日誌沒有 {'／'.join(cands)}（EKF 風估計；LOG_BITMASK 沒開 NKF/XKF？）", log


def wind_dir_from_deg(vn, ve):
    """空氣速度向量（往北、往東）→ 風「從哪裡來」的方位（度，真北順時針；氣象慣例）。"""
    return (np.degrees(np.arctan2(-np.asarray(ve, float), -np.asarray(vn, float))) + 360.0) % 360.0


def wind_report(log, fc_events, windows, *, limit_mps: float = WIND_LIMIT_MPS,
                max_gap_s: float = WIND_MAX_GAP_S) -> dict:
    """逐帶風速／風向與超過 ``limit_mps`` 的照片比例（EKF 估計）。``flagged`` = 有任何一張照片超過。"""
    limit_mps = float(limit_mps)
    if not (math.isfinite(limit_mps) and limit_mps > 0):
        raise ValueError(f"風速上限要是 > 0 的有限數值，收到 {limit_mps!r}")
    base = {"limit_mps": limit_mps, "available": False, "flagged": False,
            "note": "EKF 估計的風（不是實測）；方向是風從哪裡來（真北順時針）"}
    name, why, wlog = _wind_source(log)
    if name is None:
        return {**base, "reason": why}
    bad = _wind_unit_problem(wlog, name)
    if bad:
        return {**base, "source": name, "reason": bad}
    tab = wlog.table(name)
    t = np.asarray(tab["TimeUS"], np.int64)
    core = np.asarray(tab["C"], int) if "C" in tab else np.zeros(t.size, int)
    vn_all, ve_all = np.asarray(tab["VWN"], float), np.asarray(tab["VWE"], float)
    kf4 = wlog.table("XKF4" if name == "XKF2" else "NKF4")
    if kf4 and "PI" in kf4 and len(kf4["TimeUS"]):
        o = np.argsort(np.asarray(kf4["TimeUS"]), kind="stable")
        pi_t = np.asarray(kf4["TimeUS"], np.int64)[o]
        pi_v = np.asarray(kf4["PI"], int)[o]
        selection = f"{'XKF4' if name == 'XKF2' else 'NKF4'}.PI（主要核心，逐時刻）"
    else:
        pi_t, pi_v = np.asarray([0], np.int64), np.asarray([int(core.min()) if core.size else 0])
        selection = f"核心 {int(pi_v[0])}（日誌沒有 PI 欄）"

    def primary(tq):
        k = np.searchsorted(pi_t, np.asarray(tq, np.int64), side="right") - 1
        return pi_v[np.clip(k, 0, len(pi_v) - 1)]

    series = {}
    for c in sorted(set(core.tolist())):
        m = core == c
        o = np.argsort(t[m], kind="stable")
        series[c] = (t[m][o], vn_all[m][o], ve_all[m][o])
    # 每個樣本只在它屬於當下主要核心時採用
    use = primary(t) == core
    ts, vns, ves = t[use], vn_all[use], ve_all[use]
    o = np.argsort(ts, kind="stable")
    ts, vns, ves = ts[o], vns[o], ves[o]

    def at(tq):
        c = int(primary(tq))
        if c not in series:
            return None
        st, sn, se = series[c]
        k = int(np.searchsorted(st, tq))
        if k == 0 or k >= st.size:
            return None
        t0, t1 = st[k - 1], st[k]
        if (t1 - t0) * 1e-6 > max_gap_s:
            return None
        f = (tq - t0) / (t1 - t0) if t1 > t0 else 0.0
        return sn[k - 1] + f * (sn[k] - sn[k - 1]), se[k - 1] + f * (se[k] - se[k - 1])

    ct = np.asarray(fc_events.get("TimeUS", []), np.int64) if fc_events else np.zeros(0, np.int64)
    pw = [at(int(x)) for x in ct]
    spd = np.asarray([math.hypot(*w) if w is not None else np.nan for w in pw], float)
    known = np.isfinite(spd)
    above = known & (spd > limit_mps)
    strip_idx = _strip_index(windows, ct) if len(windows) else np.zeros(ct.size, int)
    per = []
    in_strips = np.zeros(ts.size, bool)
    for w in windows:
        m = (ts >= w.t_on_us) & (ts < w.t_off_us)
        in_strips |= m
        r = {"strip": w.strip, "n_samples": int(m.sum()), "n_photos": int((strip_idx == w.strip).sum()),
             "n_photos_above": int((above & (strip_idx == w.strip)).sum())}
        if m.any():
            s_ = np.hypot(vns[m], ves[m])
            mvn, mve = float(vns[m].mean()), float(ves[m].mean())
            r.update(mean_speed_mps=float(s_.mean()), max_speed_mps=float(s_.max()),
                     mean_vn_mps=mvn, mean_ve_mps=mve, mean_vector_speed_mps=float(math.hypot(mvn, mve)),
                     dir_from_deg=float(wind_dir_from_deg(mvn, mve)))
        else:
            r.update(mean_speed_mps=None, max_speed_mps=None, mean_vn_mps=None, mean_ve_mps=None,
                     mean_vector_speed_mps=None, dir_from_deg=None)
        per.append(r)
    sel = in_strips if windows else np.ones(ts.size, bool)
    if sel.any() and np.all(vns[sel] == 0.0) and np.all(ves[sel] == 0.0):
        return {**base, "source": name, "reason": f"{name} 的 VWN／VWE 在作業帶內全為 0：EKF 沒有在估風"
                "（沒有空速計或側滑融合？）⇒ 無法確認風速"}
    switches = int(np.count_nonzero(np.diff(pi_v))) if pi_v.size > 1 else 0
    n_known = int(known.sum())
    return {**base, "available": True, "source": name, "ekf_type": log.params.get("AHRS_EKF_TYPE"),
            "core_selection": selection, "lane_switches": switches,
            "n_photos": int(ct.size), "n_photos_with_wind": n_known, "n_photos_above": int(above.sum()),
            "share_above": (float(above.sum()) / n_known) if n_known else None,
            "photo_speed_median_mps": float(np.median(spd[known])) if n_known else None,
            "photo_speed_max_mps": float(np.max(spd[known])) if n_known else None,
            "per_strip": per, "flagged": bool(above.any()),
            "max_gap_s": float(max_gap_s)}

