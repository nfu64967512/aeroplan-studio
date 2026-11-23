"""SITL S1：航測建圖的 SITL 驗證分析（飛完之後跑；本程式不起 SITL、不開任何連線）。

輸入是一趟 ``scripts/hv_sitl_run.py`` 的產物目錄，或明確給的飛控 .BIN。逐架做三件事：

1. **只看飛控日誌**（survey.dflog）算評估 §11 S1 的出口條件，每項都有數值、門檻、通過與否：
   * CAM 張數：每條帶在 CMD/MSG 的 206 開／關**實際時刻**之間，沿飛控自己的 POS（AHRS 位置）航跡**重播
     AP_Camera 的 50 Hz 距離觸發**（206 p3=1 立即拍、CAM1_INTRVAL_MIN 的 pending、CAM1_DURATION 脈衝、
     有 ATT 時的 CAM_MAX_ROLL 閘門），tick 相位未知 ⇒ 掃 5 個相位取張數範圍，實際張數離範圍 ±1 以內。
     ``floor(L/d)+1`` 只列為參考（``expected_naive``）：它忽略 tick 量化 —— 飛控在「第一個距離 ≥ d 的 tick」
     才拍，實際間距是 d 加上最多 v·0.02 m（HERO 19 m/s、d 18.848 m ⇒ 19.0 m），長帶會多算（R2：1152 m 帶
     61 vs 62，4.8 km 帶 253 vs 255 ⇒ 正確的飛行被判未通過）；
   * 帶內間距（**在飛控 CAM 上量**）：中位數在 d 的 ±2% 內、p99 ≤ 1.2 d；
   * 206 時間窗外 0 張、轉換／降落／非 AUTO 期間 0 張；
   * 距離觸發中 |roll| > 10° 的比例（每條帶第一張不計：206 p3=1 立即拍不經 CAM_MAX_ROLL）；
   * 相鄰兩張最小間隔 ≥ CAM1_INTRVAL_MIN（飛控日誌裡的有效值）；
   * 任務裡每個 DO_LAND_START 後面緊接 206(0)（飛控 CMD 傾印＝真正上傳的任務）；
   * 飛控相機參數符合 S1 設定（CAM1_DURATION 0.1 s、CAM_AUTO_ONLY 1、CAM_MAX_ROLL > 0、
     CAM1_INTRVAL_MIN > 0；有 survey_knobs.json 時再逐值比對 CameraTriggerConfig）；
   * 有匯出任務檔時：飛控 CMD 傾印與匯出檔逐項一致，206 的 p1 以 float32 位元一致。
2. 用飛控 CAM 時刻**合成樹莓派會期**（survey.synth：注入時鐘偏移／漂移／抖動、挑幀延遲、漏邊緣、
   沒存到影像、雜訊邊緣），跑 survey.ingest.ingest_sortie 與 survey.qa.sortie_report，檢查注入的故障
   被逐一找回、時鐘擬合殘差 ≤ 0.5 ms、曝光中心時刻誤差 ≤ 1 ms。
3. 曝光層級驗證（core.recon.exposure_verifier）：飛前吃匯出任務、飛後吃 ingest 出來的 photos.csv（F2）。
   這兩項只出數字（視數、重疊、接縫、視數不足面積），不列入通過與否。

輸出只寫到 ``--out``（預設 ``runs/artifacts/SITL_S1/<名稱>``），**絕不寫 data/**。

用法
----
    # 旋鈕開著飛的（hv_sitl_run.py 會在 survey_knobs.json 記下每架 .BIN）
    python scripts/survey_sitl_s1.py --run-dir runs/artifacts/S1_xxx
    # 舊產物沒有日誌編號 ⇒ 明確給 .BIN（不猜）
    python scripts/survey_sitl_s1.py --run-dir runs/artifacts/HV4W55S \\
        --bin A1=sitl/vtol/i100/logs/00000019.BIN --bin B2=sitl/vtol/i101/logs/00000019.BIN \\
        --bin B3=sitl/vtol/i102/logs/00000019.BIN
    # 只有日誌（沒有測區與匯出任務 ⇒ 不跑曝光驗證、不比對任務檔）
    python scripts/survey_sitl_s1.py --bin A1=path/to/00000019.BIN --out <dir>
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")     # 分析工具不寫 data/logs
os.environ.setdefault("MAVLINK20", "1")                # 只讀檔；避免 pymavlink 鎖成 v1

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

CMD_NAV_LAND = 21
CMD_NAV_VTOL_LAND = 85
CMD_DO_LAND_START = 189
CMD_DO_SET_CAM_TRIGG_DIST = 206
PLANE_MODE_AUTO = 10                    # ArduPlane MODE.ModeNum
_LANDING_MSG = re.compile(r"^(Mission: \d+ (VTOLLand|Land)\b|VTOL approach|Land descend)")
# AP_Camera::update 的排程頻率（ArduPlane.cpp SCHED_TASK_CLASS(AP_Camera, …, update, 50, …)；
# 與 core.recon.exposure_verifier.CAMERA_UPDATE_HZ 同一個數）。Relay／Servo 的脈衝計數也以這個 tick 計。
CAMERA_UPDATE_HZ = 50
CAMERA_TICK_US = 1_000_000 // CAMERA_UPDATE_HZ
# 206 執行時刻與 50 Hz tick 的相位日誌裡看不到 ⇒ 掃這幾個相位，取張數的範圍
TICK_PHASES_US = (0, 4_000, 8_000, 12_000, 16_000)
# 等速直線、d 剛好是 tick 位移整數倍時（只會出現在理想化的合成資料），距離比較是浮點擲硬幣；
# 給 1 µm 讓重播結果確定。飛控自己的 Location 是 1e-7°（約 1 cm）量化，這個容差遠小於它。
_DIST_EPS_M = 1e-6


@dataclass(frozen=True)
class Thresholds:
    """出口門檻（評估 §11 S1；roll 比例上限是本專案訂的，報告裡會寫出來）。"""
    count_tol_per_strip: int = 1
    spacing_rel_tol: float = 0.02
    spacing_p99_factor: float = 1.2
    roll_limit_deg: float = 10.0
    roll_share_max: float = 0.01
    interval_tol_s: float = 0.001        # 日誌 µs 對飛控 ms 截斷的容差
    s1_cam_duration_s: float = 0.1
    fit_rms_max_s: float = 0.5e-3
    drift_tol_ppm: float = 1.0
    center_err_max_s: float = 1e-3


@dataclass(frozen=True)
class SynthParams:
    """合成樹莓派會期的注入量（survey.synth.synth_session_from_fc）。"""
    drift_ppm: float = 37.0
    clock_offset_s: float = -1234.5      # t_fc = (1 + drift)·t_boot + offset
    boottime_minus_monotonic_s: float = 7.0
    jitter_s: float = 1e-4
    frame_period_s: float = 0.1          # 挑幀延遲 ∈ [0, T)
    exposure_us: int = 1000
    drop_edges: int = 2                  # Pi 沒看到的邊緣
    no_frame: int = 1                    # 看到邊緣但沒存到影像
    extra_edges: int = 1                 # 雜訊邊緣（飛控沒拍）
    seed: int = 1
    sensor_ts_semantics: str = "exposure_start"


def _crit(key: str, name: str, value, threshold, passed, **detail) -> dict:
    """一項出口條件。``passed=None`` ＝ 未評估（detail 寫理由）。"""
    out = {"key": key, "name": name, "value": value, "threshold": threshold,
           "passed": None if passed is None else bool(passed)}
    if detail:
        out["detail"] = detail
    return out


def _param(log, name: str):
    """(日誌結束時的有效值, 飛行中有沒有改過)。沒有這個參數 ⇒ (None, False)。"""
    v = log.params.get(name)
    changed = False
    for _t, n, val in log.param_changes:
        if n == name:
            v, changed = val, True
    return v, changed


def fc_mission(log) -> list[dict]:
    """飛控 CMD 傾印＝真正在飛控上的任務。多次傾印時取第一個 206 開啟之前的最後一次。"""
    from survey import dflog
    cmd = log.table("CMD")
    if not cmd:
        return []
    t = np.asarray(cmd["TimeUS"])
    first_on = next((e.t_us for e in dflog.mission_executions(log)
                     if e.cid == CMD_DO_SET_CAM_TRIGG_DIST and (e.prm1 or 0) > 0), None)
    stamps = sorted(set(t.tolist()))
    use = [s for s in stamps if first_on is None or s <= first_on] or stamps
    sel = np.nonzero(t == use[-1])[0]
    items = {}
    for i in sel:
        items[int(cmd["CNum"][i])] = {
            "seq": int(cmd["CNum"][i]), "command": int(cmd["CId"][i]),
            "p1": float(cmd["Prm1"][i]), "p2": float(cmd["Prm2"][i]), "p3": float(cmd["Prm3"][i]),
            "p4": float(cmd["Prm4"][i]), "lat": float(cmd["Lat"][i]), "lon": float(cmd["Lng"][i]),
            "alt": float(cmd["Alt"][i]), "ctot": int(cmd["CTot"][i])}
    return [items[k] for k in sorted(items)]


def land_start_offenders(items) -> list[int] | None:
    """DO_LAND_START 後面不是 206(0) 的 seq；任務沒用相機 ⇒ None（不適用）。"""
    cmds = [(int(it["seq"]), int(it["command"]), float(it["p1"])) for it in items]
    if not any(c == CMD_DO_SET_CAM_TRIGG_DIST for _s, c, _p in cmds):
        return None
    bad = []
    for k, (seq, c, _p) in enumerate(cmds):
        if c == CMD_DO_LAND_START:
            nxt = cmds[k + 1] if k + 1 < len(cmds) else None
            if nxt is None or nxt[1] != CMD_DO_SET_CAM_TRIGG_DIST or nxt[2] != 0.0:
                bad.append(seq)
    return bad


def phase_intervals(log) -> list[tuple[int, int, str]]:
    """不該拍照的時段 [t0, t1)：起飛到第一次「Transition done」、之後每次轉換、降落起到日誌結束、非 AUTO。"""
    from survey import dflog
    t_lo, t_hi = log.time_range_us
    t_hi = int(t_hi) + 1
    out = []
    msgs = sorted(log.messages)
    done = [t for t, m in msgs if m.startswith("Transition done")]
    if done:
        out.append((int(t_lo), done[0], "takeoff_transition"))
        for t, m in msgs:
            if m.startswith("Transition started") and t > done[0]:
                end = next((d for d in done if d > t), t_hi)
                out.append((t, end, "transition"))
    else:
        out.append((int(t_lo), t_hi, "no_transition_done_msg"))
    land = [e.t_us for e in dflog.mission_executions(log) if e.cid in (CMD_NAV_VTOL_LAND, CMD_NAV_LAND)]
    land += [t for t, m in msgs if _LANDING_MSG.match(m)]
    if land:
        out.append((min(land), t_hi, "landing"))
    mode = log.table("MODE")
    if mode:
        mt, mn = np.asarray(mode["TimeUS"]), np.asarray(mode["ModeNum"])
        for k in range(len(mt)):
            if int(mn[k]) != PLANE_MODE_AUTO:
                out.append((int(mt[k]), int(mt[k + 1]) if k + 1 < len(mt) else t_hi, f"mode_{int(mn[k])}"))
    return out


def track_length_m(pos, t0_us: int, t1_us: int) -> float:
    """[t0, t1] 之間的水平航跡長（POS 內插端點，WGS84 局部平面）。"""
    from survey import datum
    t = np.asarray(pos["TimeUS"], float)
    lat, lon = np.asarray(pos["Lat"], float), np.asarray(pos["Lng"], float)
    sel = (t > t0_us) & (t < t1_us)
    la = np.concatenate([[np.interp(t0_us, t, lat)], lat[sel], [np.interp(t1_us, t, lat)]])
    lo = np.concatenate([[np.interp(t0_us, t, lon)], lon[sel], [np.interp(t1_us, t, lon)]])
    e, n = datum.local_en_m(float(la[0]), float(lo[0]), la, lo)
    return float(np.hypot(np.diff(e), np.diff(n)).sum())


def _param_at(log, name: str, t_us: int):
    """t_us 當下飛控上的參數值（之前最後一次改動；沒有這個參數 ⇒ None）。"""
    v = log.params.get(name)
    for t, n, val in log.param_changes:
        if n == name and t <= t_us:
            v = val
    return v


def replay_distance_trigger(pos, t_on_us: int, t_off_us: int, d_m: float, *, immediate: bool,
                            intrval_min_s: float = 0.0, duration_s: float = 0.0, max_roll_deg: float = 0.0,
                            att=None, phase_us: int = 0) -> list[tuple[int, float, float]]:
    """沿飛控實際航跡（POS = AHRS 位置）重播 AP_Camera 的距離觸發，回傳 [(t_us, 東 m, 北 m), …]。

    語意照 Plane-4.5.7／master 的 AP_Camera_Backend::update／take_picture 與 Relay／Servo 的 trigger_pic：

    * ``immediate``（206 p3=1）：206 執行當下拍一張（不經 CAM_MAX_ROLL），last_location = 當下位置；
    * 之後每 20 ms 一個 tick（第一個 tick 在 t_on + ``phase_us``）：先倒數脈衝計數（CAM1_DURATION×50 個 tick
      內 trigger_pic 失敗）、有 pending 就補拍；|roll| > CAM_MAX_ROLL（> 0 才有效，要給 ``att``）就跳過；
      last_location 未設 ⇒ 設成當下位置、不拍；沒移動 ⇒ 不拍；與 last_location 的水平距離 ≥ d ⇒ take_picture
      （距上一張 < CAM1_INTRVAL_MIN ⇒ 改成 pending，之後每個 tick 補拍）。

    假設：206 立即拍那一張成功（前一條帶的最後一張早已過了 INTRVAL_MIN 與脈衝時間）；全程 AUTO
    （CAM_AUTO_ONLY 的模式閘門不建模，非 AUTO 期間的照片由 transition_landing 這一項抓）。
    """
    from survey import datum
    d = float(d_m)
    if not (math.isfinite(d) and d > 0):
        raise ValueError(f"d_m 要是正數，收到 {d_m!r}")
    if not (0 <= int(phase_us) < CAMERA_TICK_US):
        raise ValueError(f"phase_us 要在 [0, {CAMERA_TICK_US}) µs，收到 {phase_us!r}")
    t = np.asarray(pos["TimeUS"], float)
    ticks = np.arange(int(t_on_us) + int(phase_us), int(t_off_us), CAMERA_TICK_US, dtype=np.int64)
    tt = np.r_[float(t_on_us), ticks.astype(float)]
    lat = np.interp(tt, t, np.asarray(pos["Lat"], float))
    lon = np.interp(tt, t, np.asarray(pos["Lng"], float))
    e, n = datum.local_en_m(float(lat[0]), float(lon[0]), lat, lon)
    e, n = np.asarray(e, float).tolist(), np.asarray(n, float).tolist()
    gate = None
    if max_roll_deg and max_roll_deg > 0 and att:
        gate = (np.abs(np.interp(ticks.astype(float), np.asarray(att["TimeUS"], float),
                                 np.asarray(att["Roll"], float))) > float(max_roll_deg)).tolist()
    # 飛控以 float32 相乘再截斷成整數（Backend.cpp take_picture、Relay.cpp trigger_pic）
    tmin_ms = int(np.float32(intrval_min_s or 0.0) * np.float32(1000.0))
    dur_ticks = int(min(np.float32(duration_s or 0.0) * np.float32(CAMERA_UPDATE_HZ), 65535.0))
    shots: list[tuple[int, float, float]] = []
    st = {"last": None, "last_ms": None, "counter": 0, "pending": False}

    def take(i: int, t_us: int) -> None:
        now_ms = t_us // 1000
        if st["last_ms"] is not None and now_ms - st["last_ms"] < tmin_ms:
            st["pending"] = True
            return
        st["pending"] = False
        if st["counter"] > 0:
            return
        st["counter"] = dur_ticks
        st["last_ms"] = now_ms
        st["last"] = (e[i], n[i])
        shots.append((int(t_us), e[i], n[i]))

    if immediate:
        take(0, int(t_on_us))
    for k, t_k in enumerate(ticks.tolist()):
        i = k + 1
        if st["counter"] > 0:
            st["counter"] -= 1
        if st["pending"]:
            take(i, t_k)
        if gate is not None and gate[k]:
            continue
        last = st["last"]
        if last is None:
            st["last"] = (e[i], n[i])
            continue
        if e[i] == last[0] and n[i] == last[1]:
            continue
        if math.hypot(e[i] - last[0], n[i] - last[1]) >= d - _DIST_EPS_M:
            take(i, t_k)
    return shots


def expected_cam_count(log, w, pos, att, mission_by_seq: dict) -> dict:
    """一條帶的張數期望值：在飛控航跡上重播 50 Hz 距離觸發（``replay_distance_trigger``），掃 tick 相位。

    參數取 206 開啟當下飛控上的值（日誌 PARM）；206 的 p3 取飛控 CMD 傾印，傾印裡沒有這一項 ⇒ 立即拍與
    不立即拍兩種都算（不替飛控猜），範圍取聯集。回傳 ``{"range": [lo, hi], "model": {...}}``。
    """
    it = mission_by_seq.get(w.cnum_on)
    immediate = None if it is None or it["command"] != CMD_DO_SET_CAM_TRIGG_DIST else int(round(it["p3"])) == 1
    tmin = _param_at(log, "CAM1_INTRVAL_MIN", w.t_on_us)
    dur = _param_at(log, "CAM1_DURATION", w.t_on_us)
    max_roll = _param_at(log, "CAM_MAX_ROLL", w.t_on_us)
    gate_on = bool(max_roll and max_roll > 0 and att)
    counts, spacing0 = [], None
    for imm in ([immediate] if immediate is not None else [True, False]):
        for ph in TICK_PHASES_US:
            shots = replay_distance_trigger(pos, w.t_on_us, w.t_off_us, w.trigg_dist_m, immediate=imm,
                                            intrval_min_s=tmin or 0.0, duration_s=dur or 0.0,
                                            max_roll_deg=max_roll or 0.0, att=att if gate_on else None,
                                            phase_us=ph)
            counts.append(len(shots))
            if spacing0 is None and len(shots) >= 2:
                xy = np.asarray([(s[1], s[2]) for s in shots], float)
                spacing0 = float(np.median(np.hypot(*np.diff(xy, axis=0).T)))
    notes = []
    if immediate is None:
        notes.append("飛控 CMD 傾印裡沒有這個 206 ⇒ 不知道 p3（立即拍），兩種都算")
    if max_roll and max_roll > 0 and not att:
        notes.append(f"CAM_MAX_ROLL {max_roll:g}° 但日誌沒有 ATT ⇒ 側傾閘門沒有建模（期望值可能偏多）")
    for name, v in (("CAM1_INTRVAL_MIN", tmin), ("CAM1_DURATION", dur)):
        if v is None:
            notes.append(f"日誌沒有 {name} ⇒ 以 0 重播（不建模）")
    return {"range": [min(counts), max(counts)],
            "model": {"immediate": immediate, "intrval_min_s": tmin, "duration_s": dur,
                      "cam_max_roll_deg": max_roll, "roll_gate_modelled": gate_on,
                      "tick_phases_us": list(TICK_PHASES_US), "counts": counts,
                      "spacing_median_m": spacing0, "notes": notes}}


def analyze_fc(log, *, th: Thresholds = Thresholds(), exported_items=None, expect_cam: dict | None = None) -> dict:
    """只用飛控日誌的出口條件。``exported_items``：匯出任務檔（MissionItem 串列），給了才比對任務。
    ``expect_cam``：survey_knobs.json 的 camera_trigger_config，給了才逐值比對相機參數。"""
    from survey import dflog, qa
    cam = dflog.cam_events(log)
    windows = dflog.strip_windows(log)
    mission = fc_mission(log)
    crit = []
    t = np.asarray(cam["TimeUS"]) if cam else np.zeros(0, np.int64)
    n_on_mission = sum(1 for it in mission if it["command"] == CMD_DO_SET_CAM_TRIGG_DIST and it["p1"] > 0)

    # 1. 張數：期望值＝在飛控航跡上重播 50 Hz 距離觸發；floor(L/d)+1 只是參考
    pos = log.table("POS")
    att = log.table("ATT")
    by_seq = {it["seq"]: it for it in mission}
    strips = []
    for w in windows:
        n_cam = int(((t >= w.t_on_us) & (t < w.t_off_us)).sum())
        L = track_length_m(pos, w.t_on_us, w.t_off_us) if pos else None
        naive = (math.floor(L / w.trigg_dist_m) + 1) if L is not None and w.trigg_dist_m > 0 else None
        exp = rng = model = None
        if pos and w.trigg_dist_m > 0:
            m = expected_cam_count(log, w, pos, att, by_seq)
            rng, model = m["range"], m["model"]
            exp = min(max(n_cam, rng[0]), rng[1])          # 範圍內任何張數都算吻合；差值量到範圍邊緣
        strips.append({"strip": w.strip, "d_m": w.trigg_dist_m, "t_on_us": w.t_on_us, "t_off_us": w.t_off_us,
                       "track_m": L, "expected": exp, "expected_range": rng, "expected_naive": naive,
                       "n_cam": n_cam, "diff": None if exp is None else n_cam - exp, "flags": list(w.flags),
                       "model": model})
    diffs = [abs(s["diff"]) for s in strips if s["diff"] is not None]
    ok = (bool(strips) and len(diffs) == len(strips) and max(diffs) <= th.count_tol_per_strip
          and len(windows) == n_on_mission and not any(s["flags"] for s in strips))
    crit.append(_crit("cam_count", "每條帶 CAM 張數 = 飛控航跡上重播 50 Hz 距離觸發的張數（206 開關實際時刻之間）",
                      max(diffs) if diffs else None, f"每帶 ±{th.count_tol_per_strip}；時間窗數 = 任務 206 開啟數",
                      ok, n_cam=int(t.size), expected_total=sum(s["expected"] or 0 for s in strips),
                      expected_total_naive=sum(s["expected_naive"] or 0 for s in strips),
                      n_windows=len(windows), n_on_in_mission=n_on_mission, strips=strips,
                      pos_missing=not pos,
                      note=("期望值：沿 POS 航跡重播 AP_Camera（206 立即拍、50 Hz 距離觸發、INTRVAL_MIN pending、"
                            "DURATION 脈衝、有 ATT 時的 CAM_MAX_ROLL），tick 相位掃 "
                            f"{'/'.join(str(p // 1000) for p in TICK_PHASES_US)} ms 取範圍；"
                            "expected_naive = floor(L/d)+1 忽略 tick 量化（實際間距 d + 最多 v·0.02 m），只供參考")))

    # 2. 間距（飛控 CAM）
    sp = qa.spacing_stats(cam, windows, rel_tol=th.spacing_rel_tol, p99_factor=th.spacing_p99_factor) if cam else []
    rel = [abs(s["median_m"] / s["d_m"] - 1.0) for s in sp if s["median_m"] is not None]
    p99 = [s["p99_m"] / s["d_m"] for s in sp if s["p99_m"] is not None]
    tick_sp = {str(s["strip"]): s["model"]["spacing_median_m"] for s in strips
               if s["model"] is not None and s["model"]["spacing_median_m"] is not None}
    # 對照基準：同一段航跡上 50 Hz 重播的間距中位數（有就用它；沒有才退回 d）。
    # ⚠ 不能直接對 d 比：距離觸發只在 50 Hz tick 上檢查 ⇒ 實際間距 = ceil(d/(v·0.02))·v·0.02，最多比 d 多一格
    #   v·0.02。S1 3 m/s 那趟 HERO 順風 21.9 m/s：d/(v·0.02) = 43.03 ⇒ 44 格 ⇒ 19.27 m（+2.3%），而重播模型
    #   逐帶 19.27–19.28 m，實測與模型差 ≤ 1 cm（runs/artifacts/S1_HDMAP_20261015_W3）。
    rel_model = []
    for s in sp:
        if s["median_m"] is None:
            continue
        ref = tick_sp.get(str(s["strip"]))
        base = ref if ref is not None else s["d_m"]
        dev = abs(s["median_m"] - base) / s["d_m"]
        s["median_ref_m"] = base
        s["median_ref"] = "tick_model" if ref is not None else "d"
        s["median_ok"] = bool(dev <= th.spacing_rel_tol)
        rel_model.append(dev)
    crit.append(_crit("spacing_median", "帶內間距中位數（飛控 CAM）與 50 Hz 重播模型的相對差（以 d 為分母）",
                      max(rel_model) if rel_model else None,
                      th.spacing_rel_tol, bool(sp) and all(s["median_ok"] for s in sp), strips=sp,
                      tick_model_spacing_m=tick_sp, rel_to_d_max=(max(rel) if rel else None),
                      note=("基準是同一段航跡上 50 Hz 重播的間距中位數（沒有模型的帶才對 d 比）；"
                            "rel_to_d_max 是對 d 的偏差，只供判讀：tick 量化本身可讓間距比 d 多到 v·0.02/d")))
    crit.append(_crit("spacing_p99", "帶內間距 p99 / d", max(p99) if p99 else None, th.spacing_p99_factor,
                      bool(sp) and all(s["p99_ok"] for s in sp)))

    # 3. 帶外、轉換／降落／非 AUTO
    outside = [int(x) for x in t if dflog.strip_of(windows, int(x)) is None]
    crit.append(_crit("outside_windows", "206 開關時間窗外的 CAM 張數", len(outside), 0, not outside,
                      t_us=outside[:20]))
    phases = phase_intervals(log)
    in_phase: dict[str, int] = {}
    for x in t:
        for a, b, label in phases:
            if a <= int(x) < b:
                in_phase[label] = in_phase.get(label, 0) + 1
    n_bad = sum(1 for x in t if any(a <= int(x) < b for a, b, _l in phases))
    crit.append(_crit("transition_landing", "轉換／降落／非 AUTO 期間的 CAM 張數", n_bad, 0, n_bad == 0,
                      by_phase=in_phase, phases=[[a, b, lab] for a, b, lab in phases]))

    # 4. 側傾：比例門檻；飛控開了 CAM_MAX_ROLL 時，距離觸發不可以有任何一張超過它（閘門真的有作用）
    rs = qa.roll_share(cam, windows, limit_deg=th.roll_limit_deg) if cam else None
    max_roll, _ = _param(log, "CAM_MAX_ROLL")
    over_gate = None
    if cam and max_roll:
        roll = np.abs(np.asarray(cam["R"], float))
        over_gate = 0
        for w in windows:
            idx = np.nonzero((t >= w.t_on_us) & (t < w.t_off_us))[0][1:]
            over_gate += int((roll[idx] > max_roll + 0.01).sum())
    ok = None if rs is None or rs["share"] is None else (rs["share"] <= th.roll_share_max and not over_gate)
    crit.append(_crit("roll_share", f"距離觸發中 |roll| > {th.roll_limit_deg:g}° 的比例（每帶第一張不計）",
                      None if rs is None else rs["share"], th.roll_share_max, ok,
                      n=None if rs is None else rs["n"], n_gt=None if rs is None else rs["n_gt"],
                      fc_cam_max_roll_deg=max_roll, n_over_cam_max_roll=over_gate,
                      note="門檻 1% 是本專案訂的（評估 §4 只說要加引入段＋CAM_MAX_ROLL）；"
                           "CAM_MAX_ROLL > 0 時另要求超過它的距離觸發 0 張"))

    # 5. 最小間隔
    intrval, intrval_changed = _param(log, "CAM1_INTRVAL_MIN")
    dt = np.diff(np.sort(t)) * 1e-6
    min_dt = float(dt.min()) if dt.size else None
    if intrval is None or min_dt is None:
        crit.append(_crit("min_interval", "相鄰兩張最小間隔（s）≥ CAM1_INTRVAL_MIN", min_dt, intrval, None,
                          reason="日誌沒有 CAM1_INTRVAL_MIN 或不足兩張"))
    else:
        crit.append(_crit("min_interval", "相鄰兩張最小間隔（s）≥ CAM1_INTRVAL_MIN", min_dt,
                          intrval - th.interval_tol_s, min_dt >= intrval - th.interval_tol_s,
                          fc_intrval_min_s=intrval, changed_in_flight=intrval_changed,
                          note=("CAM1_INTRVAL_MIN=0 ⇒ 這一項沒有約束力" if intrval == 0 else "")))

    # 6. DO_LAND_START → 206(0)
    off = land_start_offenders(mission)
    crit.append(_crit("land_start_then_off", "每個 DO_LAND_START 後面緊接 206(0)（飛控 CMD 傾印）",
                      off, [], None if off is None else not off,
                      **({"reason": "任務沒有 206 或日誌沒有 CMD 傾印"} if off is None else {})))

    # 7. 飛控相機參數
    names = ("CAM1_TYPE", "CAM1_DURATION", "CAM1_INTRVAL_MIN", "CAM_AUTO_ONLY", "CAM_MAX_ROLL")
    got = {n: _param(log, n)[0] for n in names}
    bad = []
    f32 = np.float32
    if got["CAM1_DURATION"] is None or f32(got["CAM1_DURATION"]) != f32(th.s1_cam_duration_s):
        bad.append("CAM1_DURATION")
    if got["CAM_AUTO_ONLY"] != 1.0:
        bad.append("CAM_AUTO_ONLY")
    if not (got["CAM_MAX_ROLL"] or 0) > 0:
        bad.append("CAM_MAX_ROLL")
    if not (got["CAM1_INTRVAL_MIN"] or 0) > 0:
        bad.append("CAM1_INTRVAL_MIN")
    want = {"CAM1_DURATION": th.s1_cam_duration_s, "CAM_AUTO_ONLY": 1.0, "CAM_MAX_ROLL": "> 0",
            "CAM1_INTRVAL_MIN": "> 0"}
    if expect_cam is not None:
        exact = {"CAM1_TYPE": 1.0 if expect_cam.get("backend") == "servo" else None,
                 "CAM1_DURATION": expect_cam.get("duration_s"),
                 "CAM1_INTRVAL_MIN": expect_cam.get("interval_min_s"),
                 "CAM_AUTO_ONLY": 1.0 if expect_cam.get("auto_only") else 0.0,
                 "CAM_MAX_ROLL": expect_cam.get("max_roll_deg")}
        for n, v in exact.items():
            if v is None or got[n] is None or f32(got[n]) != f32(v):
                bad.append(n)
        want = exact
    crit.append(_crit("cam_params", "飛控相機參數符合 S1 設定", got, want, not bad,
                      mismatched=sorted(set(bad)), source=("survey_knobs.json" if expect_cam else "S1 通則")))

    # 8. 任務讀回（有匯出檔才比）
    if exported_items is None:
        crit.append(_crit("mission_readback", "飛控 CMD 傾印 = 匯出任務檔（206 p1 float32 位元一致）",
                          None, 0, None, reason="沒有匯出任務檔"))
    else:
        exp_rows = [(int(it.seq), int(it.command), float(it.param1)) for it in exported_items]
        fc_rows = [(it["seq"], it["command"], it["p1"]) for it in mission]
        mism = []
        if len(exp_rows) != len(fc_rows):
            mism.append(f"項數 匯出 {len(exp_rows)} ≠ 飛控 {len(fc_rows)}")
        for a, b in zip(exp_rows, fc_rows):
            if a[0] != b[0] or a[1] != b[1]:
                mism.append(f"seq {a[0]}: 匯出 cmd {a[1]} ≠ 飛控 seq {b[0]} cmd {b[1]}")
            elif a[1] == CMD_DO_SET_CAM_TRIGG_DIST and f32(a[2]).tobytes() != f32(b[2]).tobytes():
                mism.append(f"seq {a[0]}: 206 p1 匯出 {a[2]!r} ≠ 飛控 {b[2]!r}")
        n206 = sum(1 for r in exp_rows if r[1] == CMD_DO_SET_CAM_TRIGG_DIST)
        crit.append(_crit("mission_readback", "飛控 CMD 傾印 = 匯出任務檔（206 p1 float32 位元一致）",
                          len(mism), 0, not mism, mismatches=mism[:20], n_items=len(exp_rows), n_206=n206))

    d_vals = sorted({round(w.trigg_dist_m, 6) for w in windows})
    summary = {"n_cam": int(t.size), "n_trig": log.count("TRIG"), "firmware": log.firmware,
               "trigg_dist_m": d_vals, "n_windows": len(windows), "n_mission_items": len(mission),
               "fc_params": {n: _param(log, n)[0] for n in names + ("LOG_DISARMED", "AIRSPEED_CRUISE",
                                                                     "AIRSPEED_MIN", "AIRSPEED_MAX",
                                                                     "SIM_WIND_SPD", "SIM_WIND_DIR")}}
    return {"summary": summary, "criteria": crit, "mission": mission, "windows": windows}


def _fault_plan(t_fc_us, sp: SynthParams):
    """依 seed 選要注入故障的索引：漏邊緣、沒存影像、雜訊邊緣（放在相鄰兩張的正中間，離兩邊都 ≥ 0.5 s）。"""
    n = len(t_fc_us)
    if n < sp.drop_edges + sp.no_frame + 3:
        raise ValueError(f"CAM 只有 {n} 張，不夠注入 {sp.drop_edges}+{sp.no_frame} 個故障")
    rng = np.random.default_rng(sp.seed)
    idx = rng.permutation(n)
    drop = sorted(int(i) for i in idx[:sp.drop_edges])
    no_frame = sorted(int(i) for i in idx[sp.drop_edges:sp.drop_edges + sp.no_frame])
    t = np.asarray(t_fc_us, np.int64)
    gaps = np.nonzero(np.diff(t) >= 1_000_000)[0]
    if gaps.size < sp.extra_edges:
        raise ValueError(f"相鄰間隔 ≥ 1 s 的只有 {gaps.size} 處，放不下 {sp.extra_edges} 個雜訊邊緣")
    pick = sorted(int(k) for k in rng.choice(gaps, size=sp.extra_edges, replace=False))
    extra_s = [float(t[k] + t[k + 1]) * 0.5e-6 for k in pick]
    return drop, no_frame, extra_s


def run_ingest(bin_path, uav: str, sess_dir: Path, sp: SynthParams, th: Thresholds, log, windows) -> dict:
    """合成會期 → ingest_sortie → 檢查注入的故障被逐一找回。回傳 (摘要 dict)；照片列放在 ``rows``。"""
    from survey import dflog, ingest, synth
    cam = dflog.cam_events(log)
    t_fc_us = np.asarray(cam["TimeUS"], np.int64)
    drop, no_frame, extra_s = _fault_plan(t_fc_us, sp)
    if sess_dir.exists():
        shutil.rmtree(sess_dir)
    truth = synth.synth_session_from_fc(
        sess_dir, t_fc_us, uav=uav, session_id="S1", clock_a=1.0 + sp.drift_ppm * 1e-6,
        clock_b_s=sp.clock_offset_s,
        boottime_minus_monotonic_ns=int(round(sp.boottime_minus_monotonic_s * 1e9)), jitter_s=sp.jitter_s,
        frame_period_s=sp.frame_period_s, exposure_us=sp.exposure_us,
        sensor_ts_semantics=sp.sensor_ts_semantics, drop_index=drop, extra_edges_fc_s=extra_s,
        no_frame_index=no_frame, seed=sp.seed)
    res = ingest.ingest_sortie(bin_path, sess_dir, sensor_ts_semantics=sp.sensor_ts_semantics)
    fit, sess = res.fit, res.session
    tf = truth["frames"]
    extra_seqs = sorted(f["seq"] for f in tf if f["fc_index"] is None)
    got_extra = sorted(int(sess.frames[int(i)].seq) for i in fit.unmatched_pi)
    wrong_pairs = sum(1 for fc_i, pi_j in fit.pairs if tf[int(pi_j)]["fc_index"] != int(fc_i))
    center = {f["seq"]: f["center_fc_s"] for f in tf}
    paired_seq = {int(sess.frames[int(pi_j)].seq) for _fc, pi_j in fit.pairs}
    errs = [abs(r["t_fc_us"] * 1e-6 - center[r["seq"]]) for r in res.poses.rows if r["seq"] in paired_seq]
    max_center_err = max(errs) if errs else None
    n_no_frame_skipped = sum(1 for s in res.poses.skipped if s["reason"] == "status:no_frame")
    # 守恆表：沒守恆的條帶必須剛好是注入故障的那幾條（None ＝ 帶外那一列）
    fault_strips = {dflog.strip_of(windows, int(t_fc_us[i])) for i in drop + no_frame}
    fault_strips |= {dflog.strip_of(windows, int(round(s * 1e6))) for s in extra_s}
    cons = res.qa["conservation"]
    unconserved = {r["strip"] for r in cons if not r["conserved"]}
    checks = {
        "unmatched_fc_equals_dropped": [int(i) for i in fit.unmatched_fc] == drop,
        "unmatched_pi_equals_extra": got_extra == extra_seqs,
        "all_pairs_correct": wrong_pairs == 0,
        "fit_rms_ok": fit.rms_s <= th.fit_rms_max_s,
        "drift_ok": abs(fit.drift_ppm - sp.drift_ppm) <= th.drift_tol_ppm,
        "exposure_center_ok": max_center_err is not None and max_center_err <= th.center_err_max_s,
        "no_frame_skipped_ok": n_no_frame_skipped == sp.no_frame,
        "unconserved_strips_are_fault_strips": unconserved == fault_strips,
    }
    return {
        "criterion": _crit("ingest_recovery", "合成會期：注入故障逐一找回、擬合殘差、曝光中心時刻",
                           {"fit_rms_ms": fit.rms_s * 1e3, "drift_ppm": fit.drift_ppm,
                            "max_center_err_ms": None if max_center_err is None else max_center_err * 1e3,
                            "wrong_pairs": wrong_pairs},
                           {"fit_rms_ms": th.fit_rms_max_s * 1e3, "drift_tol_ppm": th.drift_tol_ppm,
                            "max_center_err_ms": th.center_err_max_s * 1e3, "wrong_pairs": 0},
                           all(checks.values()), checks=checks),
        "injected": {"drop_index": drop, "no_frame_index": no_frame, "extra_edges_fc_s": extra_s,
                     "extra_seq": extra_seqs, "fault_strips": sorted(fault_strips, key=lambda s: (s is None, s))},
        "recovered": {"unmatched_fc": [int(i) for i in fit.unmatched_fc], "unmatched_pi_seq": got_extra,
                      "unconserved_strips": sorted(unconserved, key=lambda s: (s is None, s))},
        "fit": fit.summary(),
        "n_poses": len(res.poses.rows), "skipped": res.poses.skipped,
        "pose_effective": res.poses.effective,
        "qa": {k: res.qa[k] for k in ("conservation", "spacing", "roll", "fit", "passed")},
        "qa_passed_note": "注入了故障時 qa.passed 為 False 是預期的（守恆表要抓的就是它們）",
        "rows": res.poses.rows,
    }


class FovCamera:
    """由 HFOV／VFOV 定義的正下視相機（與規劃器 / _uniform_camera 同一個足跡）。"""

    def __init__(self, hfov_deg: float, vfov_deg: float):
        if not (0.0 < hfov_deg < 180.0 and 0.0 < vfov_deg < 180.0):
            raise ValueError(f"FOV 無效：{hfov_deg}, {vfov_deg}")
        self.hfov_deg, self.vfov_deg = float(hfov_deg), float(vfov_deg)

    def footprint_m(self, agl_m: float) -> tuple[float, float]:
        return (2.0 * agl_m * math.tan(math.radians(self.hfov_deg) / 2.0),
                2.0 * agl_m * math.tan(math.radians(self.vfov_deg) / 2.0))


def _exposure_summary(rep) -> dict:
    o = lambda s: None if s is None else asdict(s)      # noqa: E731
    return {"frac_ge": {str(k): v for k, v in rep.frac_ge.items()}, "forward_overlap": o(rep.forward_overlap),
            "side_overlap": o(rep.side_overlap), "seam_overlap_min": rep.seam_overlap_min,
            "seams": [{k: v for k, v in asdict(s).items()} for s in rep.seams],
            "photos_per_uav": rep.photos_per_uav, "photos_excluded": rep.photos_excluded,
            "undercovered_area_m2": rep.undercovered_area_m2, "area_m2": rep.area_m2,
            "notes": list(rep.notes), "effective": rep.effective, "elapsed_s": rep.elapsed_s}


def _json_default(o):
    """numpy 純量照值寫（numpy.bool_ 不可以變成字串 "True"）；其他物件寫成字串。"""
    return o.item() if isinstance(o, np.generic) else str(o)


def _check_out(out: Path) -> Path:
    out = Path(out).resolve()
    data = (ROOT / "data").resolve()
    if out == data or data in out.parents:
        raise SystemExit(f"輸出目錄 {out} 在 data/ 底下：這支分析工具絕不寫使用者資料目錄")
    return out


def resolve_inputs(run_dir: Path | None, bins: dict[str, str]) -> dict:
    """決定每架的 .BIN、匯出任務、測區、相機 FOV、旋鈕紀錄。BIN 一律明確（--bin 或 survey_knobs.json）。"""
    knobs = scen = None
    plan_uavs: list[str] = []
    if run_dir is not None:
        run_dir = Path(run_dir)
        if not run_dir.is_dir():
            raise SystemExit(f"找不到產物目錄 {run_dir}")
        if (run_dir / "survey_knobs.json").exists():
            knobs = json.loads((run_dir / "survey_knobs.json").read_text(encoding="utf-8"))
        if (run_dir / "scenario.json").exists():
            scen = json.loads((run_dir / "scenario.json").read_text(encoding="utf-8"))
        plan_uavs = sorted(p.stem for p in (run_dir / "plan").glob("*.waypoints"))
    logs = {u: r.get("bin") for u, r in ((knobs or {}).get("sitl_logs") or {}).items()}
    use = {u: (bins.get(u) or logs.get(u)) for u in sorted(set(plan_uavs) | set(bins))}
    missing = sorted(u for u, b in use.items() if not b)
    use = {u: Path(b) for u, b in use.items() if b}
    for u, b in use.items():
        if not b.is_file():
            raise SystemExit(f"{u} 的日誌 {b} 不存在")
    if not use:
        raise SystemExit("沒有任何 .BIN：請用 --bin UAV=路徑 指定（舊產物沒有記錄日誌編號，不猜）")
    return {"run_dir": run_dir, "bins": use, "missing_bin": missing, "knobs": knobs, "scenario": scen}


def analyze(*, run_dir=None, bins: dict[str, str] | None = None, out, th: Thresholds = Thresholds(),
            sp: SynthParams = SynthParams(), hfov_deg: float | None = None, vfov_deg: float | None = None,
            cell_size_m: float = 2.0, run_verifier: bool = True) -> dict:
    """分析一趟 S1，寫 ``s1_report.json``／``s1_report.md``／``photos.csv`` 到 ``out``，回傳報告 dict。"""
    from survey import datum, deliverable, dflog
    out = _check_out(Path(out))
    out.mkdir(parents=True, exist_ok=True)
    inp = resolve_inputs(Path(run_dir) if run_dir else None, dict(bins or {}))
    scen, knobs = inp["scenario"], inp["knobs"]
    expect_cam = (knobs or {}).get("camera_trigger_config")
    report: dict = {"format": "aeroplan-sitl-s1/1", "run_dir": str(inp["run_dir"]) if inp["run_dir"] else None,
                    "thresholds": asdict(th), "synth": asdict(sp), "uavs": {},
                    "missing_bin": inp["missing_bin"], "knobs": knobs}
    logs, rows_all, missions, homes, und = {}, [], {}, {}, {}
    for k, (uav, bin_path) in enumerate(sorted(inp["bins"].items())):
        log = dflog.read_dflog(bin_path)
        logs[uav] = log
        wp = inp["run_dir"] / "plan" / f"{uav}.waypoints" if inp["run_dir"] else None
        exported = None
        if wp is not None and wp.exists():
            from hv_dccpp.ardupilot import read_qgc_wpl
            exported = read_qgc_wpl(wp)
            missions[uav] = exported
        fc = analyze_fc(log, th=th, exported_items=exported, expect_cam=expect_cam)
        st = os.stat(bin_path)
        u = {"bin": str(bin_path), "bin_size": st.st_size, "waypoints": str(wp) if exported else None,
             "summary": fc["summary"], "criteria": fc["criteria"]}
        sp_u = SynthParams(**{**asdict(sp), "seed": sp.seed + k})      # 每架不同 seed，記在報告裡
        try:
            ing = run_ingest(bin_path, uav, out / "sessions" / uav, sp_u, th, log, fc["windows"])
            rows_all += ing.pop("rows")
            ing["synth"] = asdict(sp_u)
            u["ingest"] = ing
            u["criteria"].append(ing["criterion"])
        except Exception as e:      # noqa: BLE001 —— ingest 失敗本身就是 S1 的失敗，寫進報告
            u["criteria"].append(_crit("ingest_recovery", "合成會期：注入故障逐一找回、擬合殘差、曝光中心時刻",
                                       None, None, False, error=f"{type(e).__name__}: {e}"))
        home = next((it for it in fc["mission"] if it["seq"] == 0), None)
        homes[uav] = None if home is None else home["alt"]
        und[uav] = datum.receiver_undulation_from_log(log).n_m
        evaluated = [c for c in u["criteria"] if c["passed"] is not None]
        u["passed"] = bool(evaluated) and all(c["passed"] for c in evaluated)
        u["not_evaluated"] = [c["key"] for c in u["criteria"] if c["passed"] is None]
        report["uavs"][uav] = u
    if rows_all:
        deliverable.write_photos_csv(out / "photos.csv", rows_all)

    report["exposure"] = _run_verifiers(out, scen, logs, missions, homes, und, rows_all,
                                        hfov_deg, vfov_deg, cell_size_m) if run_verifier else {
        "skipped": "run_verifier=False"}
    report["passed"] = (not inp["missing_bin"]) and bool(report["uavs"]) and all(
        u["passed"] for u in report["uavs"].values())
    (out / "s1_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=_json_default),
                                        encoding="utf-8", newline="\n")
    (out / "s1_report.md").write_text(render_markdown(report), encoding="utf-8", newline="\n")
    return report


def _run_verifiers(out, scen, logs, missions, homes, und, rows_all, hfov_deg, vfov_deg, cell_size_m) -> dict:
    """曝光層級驗證（只出數字）。缺測區、相機或高度一致性時寫明不跑的理由。"""
    from core.recon import exposure_verifier as ev
    polys = [a["polygon"] for a in (scen or {}).get("areas", [])]
    hf = hfov_deg if hfov_deg is not None else (scen or {}).get("hfov_deg")
    vf = vfov_deg if vfov_deg is not None else (scen or {}).get("vfov_deg")
    if not polys or hf is None or vf is None:
        return {"skipped": "沒有測區或相機 FOV（需要 --run-dir 的 scenario.json，或 --hfov-deg/--vfov-deg）"}
    hv = [h for h in homes.values() if h is not None]
    if not hv or max(hv) - min(hv) > 0.5:
        return {"skipped": f"各架 home 高度 {homes} 缺漏或相差 > 0.5 m，平地假設不成立"}
    ground = float(np.mean(hv))
    cam = FovCamera(float(hf), float(vf))
    par = {u: {n: _param(logs[u], n)[0] for n in ("AIRSPEED_CRUISE", "AIRSPEED_MIN", "AIRSPEED_MAX",
                                                 "CAM1_INTRVAL_MIN", "CAM1_DURATION", "SIM_WIND_SPD",
                                                 "SIM_WIND_DIR")} for u in missions}
    res: dict = {"inputs": {"hfov_deg": hf, "vfov_deg": vf, "polygons": len(polys), "cell_size_m": cell_size_m,
                            "ground_msl_m": ground, "ground_source": "飛控 CMD 傾印 seq 0（home）高度的平均",
                            "fc_params": par}}
    no_cruise = sorted(u for u in par if par[u]["AIRSPEED_CRUISE"] is None)
    if missions and no_cruise:
        res["preflight"] = {"skipped": f"日誌沒有 AIRSPEED_CRUISE：{no_cruise}"}
    elif missions:
        p0 = par[sorted(par)[0]]
        if p0["SIM_WIND_SPD"] is not None:
            wind, wsrc = (float(p0["SIM_WIND_SPD"]), float(p0["SIM_WIND_DIR"] or 0.0)), "飛控 SIM_WIND_*（實際模擬風）"
        else:
            wind, wsrc = tuple((scen or {}).get("wind") or (0.0, 0.0)), "scenario.json 規劃風"
        try:
            pre = ev.verify_preflight(
                missions, polys, cam, airspeed_mps={u: par[u]["AIRSPEED_CRUISE"] for u in missions}, wind=wind,
                terrain=ground, home_alt_msl={u: homes[u] for u in missions},
                intrval_min_s={u: par[u]["CAM1_INTRVAL_MIN"] or 0.0 for u in missions},
                trigger_duration_s={u: par[u]["CAM1_DURATION"] or 0.1 for u in missions},
                airspeed_limits_mps={u: ((par[u]["AIRSPEED_MIN"], par[u]["AIRSPEED_MAX"])
                                         if None not in (par[u]["AIRSPEED_MIN"], par[u]["AIRSPEED_MAX"]) else None)
                                     for u in missions},
                cell_size_m=cell_size_m)
            res["preflight"] = _exposure_summary(pre)
            res["preflight"]["wind"], res["preflight"]["wind_source"] = list(wind), wsrc
            res["preflight"]["fc_cam_vs_simulated"] = {u: {"fc_cam": logs[u].count("CAM"),
                                                           "simulated": pre.photos_per_uav.get(u)} for u in missions}
        except Exception as e:      # noqa: BLE001
            res["preflight"] = {"error": f"{type(e).__name__}: {e}"}
    else:
        res["preflight"] = {"skipped": "沒有匯出任務檔"}
    nv = set(round(v, 3) for v in und.values() if v is not None)
    if not rows_all:
        res["postflight"] = {"skipped": "沒有 ingest 出來的照片"}
    elif len(nv) != 1:
        res["postflight"] = {"skipped": f"各架接收機大地起伏不一致或未知 {und}"}
    else:
        n_und = nv.pop()
        try:
            post = ev.verify_postflight(out / "photos.csv", polys, cam, terrain=ground + n_und,
                                        height_field="h_ell_m", cell_size_m=cell_size_m)
            res["postflight"] = _exposure_summary(post)
            res["postflight"]["terrain_ell_m"] = ground + n_und
            res["postflight"]["undulation_m"] = n_und
        except Exception as e:      # noqa: BLE001
            res["postflight"] = {"error": f"{type(e).__name__}: {e}"}
    return res


def _fmt(v) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, dict):
        return "; ".join(f"{k}={_fmt(x)}" for k, x in v.items())
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    return str(v)


def render_markdown(rep: dict) -> str:
    mark = {True: "通過", False: "**未通過**", None: "未評估"}
    lines = ["# SITL S1 航測驗證報告", "",
             f"- 產物目錄：{rep.get('run_dir') or '（只給日誌）'}",
             f"- 整體：{mark[rep['passed']]}"
             + (f"（缺日誌：{', '.join(rep['missing_bin'])}）" if rep.get("missing_bin") else ""), ""]
    for uav, u in rep["uavs"].items():
        s = u["summary"]
        lines += [f"## {uav}", "", f"- 日誌：`{u['bin']}`（{s['firmware']}）",
                  f"- CAM {s['n_cam']} 張、TRIG {s['n_trig']}、條帶時間窗 {s['n_windows']}、d = {_fmt(s['trigg_dist_m'])} m",
                  f"- 結果：{mark[u['passed']]}", "",
                  "| 條件 | 數值 | 門檻 | 結果 |", "|---|---|---|---|"]
        for c in u["criteria"]:
            cells = (c["name"], _fmt(c["value"]), _fmt(c["threshold"]), mark[c["passed"]])
            lines.append("| " + " | ".join(x.replace("|", r"\|") for x in cells) + " |")
        lines.append("")
    ex = rep.get("exposure") or {}
    lines += ["## 曝光層級驗證（只出數字，不列入通過與否）", ""]
    for k in ("preflight", "postflight"):
        e = ex.get(k)
        if not isinstance(e, dict) or "frac_ge" not in e:
            lines.append(f"- {k}：{_fmt(e) if e is not None else _fmt(ex.get('skipped'))}")
            continue
        lines.append(f"- {k}：視數比例 {_fmt(e['frac_ge'])}；前向重疊中位 "
                     f"{_fmt((e['forward_overlap'] or {}).get('median'))}；側向重疊中位 "
                     f"{_fmt((e['side_overlap'] or {}).get('median'))}；接縫最窄比例 {_fmt(e['seam_overlap_min'])}；"
                     f"視數不足面積 {_fmt(e['undercovered_area_m2'])} m²；張數 {_fmt(e['photos_per_uav'])}")
    lines += ["", f"門檻：{_fmt(rep['thresholds'])}", f"合成會期：{_fmt(rep['synth'])}", ""]
    return "\n".join(lines)


def _parse_bins(items) -> dict[str, str]:
    out = {}
    for s in items or ():
        uav, sep, path = s.partition("=")
        if not sep or not uav.strip() or not path.strip():
            raise SystemExit(f"--bin 要寫成 UAV=路徑，收到 {s!r}")
        out[uav.strip()] = path.strip()
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SITL S1 航測驗證分析（不起 SITL、不開連線）")
    ap.add_argument("--run-dir", help="hv_sitl_run.py 的產物目錄（runs/artifacts/<trial>）")
    ap.add_argument("--bin", action="append", default=[], metavar="UAV=PATH", help="明確指定某架的 .BIN（可重複）")
    ap.add_argument("--out", help="輸出目錄（預設 runs/artifacts/SITL_S1/<名稱>；不可在 data/ 底下）")
    ap.add_argument("--hfov-deg", type=float)
    ap.add_argument("--vfov-deg", type=float)
    ap.add_argument("--cell-size-m", type=float, default=2.0)
    ap.add_argument("--no-verifier", action="store_true", help="不跑曝光層級驗證")
    th0, sp0 = Thresholds(), SynthParams()
    for f, v in asdict(th0).items():
        ap.add_argument("--" + f.replace("_", "-"), type=type(v), default=v, dest="th_" + f)
    for f, v in asdict(sp0).items():
        ap.add_argument("--synth-" + f.replace("_", "-"), type=type(v), default=v, dest="sp_" + f)
    a = ap.parse_args(argv)
    if not a.run_dir and not a.bin:
        ap.error("至少要給 --run-dir 或 --bin")
    name = Path(a.run_dir).name if a.run_dir else "bins"
    out = Path(a.out) if a.out else ROOT / "runs" / "artifacts" / "SITL_S1" / name
    th = Thresholds(**{f: getattr(a, "th_" + f) for f in asdict(th0)})
    sp = SynthParams(**{f: getattr(a, "sp_" + f) for f in asdict(sp0)})
    rep = analyze(run_dir=a.run_dir, bins=_parse_bins(a.bin), out=out, th=th, sp=sp, hfov_deg=a.hfov_deg,
                  vfov_deg=a.vfov_deg, cell_size_m=a.cell_size_m, run_verifier=not a.no_verifier)
    for uav, u in rep["uavs"].items():
        bad = [c["key"] for c in u["criteria"] if c["passed"] is False]
        print(f"{uav}: {'通過' if u['passed'] else '未通過'}" + (f"（{', '.join(bad)}）" if bad else ""))
    print(f"S1 {'通過' if rep['passed'] else '未通過'}；報告 {out / 's1_report.md'}")
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
