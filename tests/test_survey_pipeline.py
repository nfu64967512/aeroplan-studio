"""survey.pipeline 端到端：合成航線 → .BIN ＋ Pi 會期 ＋ 相機模組 rover.ubx（TIM-TM2）＋ PPK .pos → 交付物。

真值與 test_survey_e2e 同一套（tests/test_survey_synthlog 的 SurveyTruth＋DataFlash 寫入器，survey.synth 的會期），
另外產生：
* rover.ubx：每個實體觸發邊緣（= 飛控 CAM 的時刻，換成 GPST）依 F9P「每 epoch 只報最後一個上升緣」寫 TIM-TM2，
  每 epoch 一則 NAV-PVT 與 RAWX（tests/test_survey_ppk_ubx 的獨立編碼器）。
* rover.pos：相機模組 F9P 天線（機身 FRD 偏移 F9P_ANT）在 10 Hz GPST 的橢球高軌跡（tests/test_survey_ppk_pos 的格式器）。
  表頭 ``% obs start／end`` = rover.ubx 第一／最後一個 RAWX epoch（RTKLIB 的定義），``% ref pos`` = 基站 BASE 的標石座標
  （2026-10-21 起 PPK 數值鎖會核對這兩項，見 survey.pipeline 的 M2a／M2b）。
* session.json 的 gnss_lever_arm_m = 相機中心 − F9P 天線。

真值的相機中心：SurveyTruth.point_llh(曝光中心, GPS_POS + LEVER)，pyproj／scipy 計算，與 survey 的換算互相獨立。
"""
from __future__ import annotations

import io
import json
import math
import zipfile
from dataclasses import replace
from collections import Counter
from pathlib import Path

import numpy as np
import pytest
from pyproj import Geod

from survey import datum, pipeline, synth
from survey.pipeline import PipelineCanceled, PipelineError, PpkSettings, SortieInput
from survey.ppk import rtklib
from tests.test_survey_ppk_pos import pos_text
from tests.test_survey_ppk_ubx import pvt_frame, rawx_frame, tm2_frame
from tests.test_survey_synthlog import SurveyTruth, strip_intervals, write_survey_log

GEOD = Geod(ellps="WGS84")
N_SITE = 18.54                                 # 本場址 EGM96 大地起伏 —— 只用在測試
GPS_POS = np.array((0.30, 0.0, -0.20))         # 飛控 GNSS 天線（自 IMU，FRD）
LEVER = np.array((0.15, -0.05, 0.10))          # 相機中心 − 飛控 GNSS 天線
F9P_ANT = np.array((0.10, 0.02, -0.25))        # 相機模組 F9P 天線（自 IMU）
GNSS_LEVER = GPS_POS + LEVER - F9P_ANT         # 相機中心 − F9P 天線（session.json gnss_lever_arm_m）
GPST0 = 1_444_000_000.0
GPS_LAG = 0.08
# 64×48 的小影像；像素間距放大成 97.66 µm，讓 120 m 足跡 ≈ 125 × 94 m（與 HQ6 相同），視數才有意義
CAMERA = {"sensor": "imx477", "lens_mm": 6.0, "pixel_um": 97.66, "width": 64, "height": 48, "readout_ms": 20.0,
          "shutter": "rolling", "kernel": "6.6.y", "dtoverlay": "imx477"}
GROUND_ELL = 30.0 + N_SITE                     # 地面 AMSL 30 m（航高 150 m ⇒ AGL 120 m）的橢球高
POLY = [(25.1003, 121.2005), (25.1003, 121.2045), (25.1019, 121.2045), (25.1019, 121.2005)]
GRS80 = "+proj=longlat +ellps=GRS80 +no_defs"
W_NS = 604800 * 10**9
FRAME = "e-GNSS[2025]"
BASE = rtklib.BaseStation(25.0995, 121.1995, 52.3, FRAME, antenna_height_m=1.234)   # 合成的基站（標石）座標
EPOCH_S = 0.1


def ppk_settings(**kw):
    kw.setdefault("base", BASE)
    return PpkSettings(frame=FRAME, **kw)


def rawx_span(tr, *, lead_s=2.0, tail_s=0.5):
    """移動站 RAWX（10 Hz 格點）涵蓋整條 .pos（第一個 epoch = 第一筆解，如 RTKLIB／ES）：回傳 (第一個, 最後一個) epoch 的 GPST 秒。"""
    n0 = math.ceil(round((GPST0 + tr.t0 - lead_s) / EPOCH_S, 6))
    n1 = math.floor(round((GPST0 + tr.t_end + tail_s) / EPOCH_S, 6))
    return n0 * EPOCH_S, n1 * EPOCH_S


def _fields(t_ns):
    wn, rem = divmod(int(t_ns), W_NS)
    ms, sub = divmod(rem, 1_000_000)
    return wn, ms, sub


def rover_ubx_bytes(edges_ns, *, epoch_ns=100_000_000, c0=100, span_s=None, week0_lead=0):
    """F9P 10 Hz：每個 epoch RAWX＋NAV-PVT，有新上升緣就加一則 TIM-TM2（只報該 epoch 最後一個）。

    ``span_s`` = (第一個, 最後一個) RAWX epoch 的 GPST 秒（預設只涵蓋第一個邊緣前 0.3 s 到最後一個後 0.3 s）；
    ``week0_lead``：最前面先放幾個 week=0 的 RAWX（Reach 檔開頭就有，接收機還不知道週數）。
    """
    edges = np.asarray(sorted(edges_ns), np.int64)
    out = [rawx_frame(1.0 + 0.1 * k, 0, leap=18, nmeas=1, rec_stat=0) for k in range(week0_lead)]
    lo, hi = int(edges[0]) // epoch_ns - 3, int(edges[-1]) // epoch_ns + 4
    if span_s is not None:
        lo, hi = round(span_s[0] * 1e9 / epoch_ns), round(span_s[1] * 1e9 / epoch_ns) + 1
    for n in range(lo, hi):
        tau = n * epoch_ns
        wn, ms, _ = _fields(tau)
        rm = (edges > tau - epoch_ns) & (edges <= tau)
        msg = b""
        if rm.any():
            wr, msr, sr = _fields(edges[int(np.nonzero(rm)[0][-1])])
            msg = tm2_frame(ch=0, flags=0x01 | 0x40 | (1 << 3) | 0x80, count=c0 + int((edges <= tau).sum()),
                            wn_r=wr, tow_ms_r=msr, sub_r=sr, acc=25)
        out.append(rawx_frame(ms / 1000.0, wn, leap=18, nmeas=1) + pvt_frame(itow_ms=ms) + msg)
    return b"".join(out)


def write_pos(path, tr, *, dlat_deg=0.0, dh_m=0.0, dt_s=0.0, q_of_t=None, obs="rawx", noise=None, **pos_kw):
    """F9P 天線軌跡（10 Hz、橢球高）；q_of_t(t_fc) → Q 值（預設全 fix）。

    ``dt_s``：時間欄整體平移（−18 ⇒ UTC 卻標 GPST），表頭 obs start／end 不動；``obs="rawx"`` ⇒ 表頭寫
    rover.ubx 的 RAWX 跨度，其他值照傳給 pos_text；``ref_pos`` 預設 = BASE 的標石座標；
    ``noise`` = (水平 σ m, 垂直 σ m, 亂數種子)；其他關鍵字照傳給 pos_text。
    """
    t = np.arange(tr.t0 - 2.0, tr.t_end - 0.1, 0.1)
    llh = np.asarray([tr.point_llh(x, tuple(F9P_ANT)) for x in t])
    q = np.ones(t.size, int) if q_of_t is None else np.asarray([q_of_t(x) for x in t], int)
    lat, lon, h = llh[:, 0] + dlat_deg, llh[:, 1], llh[:, 2] + N_SITE + dh_m
    if noise is not None:
        rng = np.random.default_rng(noise[2])
        lat = lat + rng.normal(0.0, noise[0], t.size) / 111_000.0
        h = h + rng.normal(0.0, noise[1], t.size)
    pos_kw.setdefault("ref_pos", (BASE.lat_deg, BASE.lon_deg, BASE.h_ell_m))
    if pos_kw.get("fmt") == "xyz":
        pos_kw.setdefault("ecef_sd", np.tile([0.006, 0.008, 0.015, 0.0, 0.0, 0.0], (t.size, 1)))
    Path(path).write_text(pos_text(GPST0 + t + dt_s, lat, lon, h, q=q, obs=rawx_span(tr) if obs == "rawx" else obs,
                                   **pos_kw), encoding="utf-8")
    return Path(path)


def build_sortie(d: Path, *, uav, sid, seed, drops=(), gnss_lever=tuple(GNSS_LEVER)):
    d.mkdir(parents=True, exist_ok=True)
    tr = SurveyTruth(n_strips=3, strip_len=500.0, spacing=120.0)
    info = write_survey_log(d / "fc.BIN", tr, undulation_n=N_SITE, gps_pos=tuple(GPS_POS), gps_lag_s=GPS_LAG,
                            gpst_at_boot=GPST0)
    cam_t = info["cam_t_us"]
    truth = synth.synth_session_from_fc(d / "sess", cam_t, uav=uav, session_id=sid, unit_model=f"RPiHQ6-{uav}",
                                        camera=CAMERA, lever_arm_m=tuple(LEVER), clock_a=1 + 42e-6,
                                        clock_b_s=-523.25, boottime_minus_monotonic_ns=1_500_000_000,
                                        jitter_s=2e-5, exposure_us=800, drop_index=drops, seed=seed)
    sj = d / "sess" / "session.json"
    meta = json.loads(sj.read_text(encoding="utf-8"))
    meta["gnss_lever_arm_m"] = None if gnss_lever is None else [float(x) for x in gnss_lever]
    sj.write_text(json.dumps(meta, indent=1), encoding="utf-8")
    (d / "sess" / "gnss").mkdir(exist_ok=True)
    edges_ns = [round((GPST0 + t * 1e-6) * 1e9) for t in cam_t]
    (d / "sess" / "gnss" / "rover.ubx").write_bytes(
        rover_ubx_bytes(edges_ns, span_s=rawx_span(tr)))                  # TM2 看得到每一個實體邊緣
    pos = write_pos(d / "rover.pos", tr)
    return {"dir": d, "uav": uav, "truth": tr, "info": info, "pi": truth, "pos": pos, "edges_ns": edges_ns,
            "bin": d / "fc.BIN", "sess": d / "sess",
            "center_fc": {f["seq"]: f["center_fc_s"] for f in truth["frames"]}}


@pytest.fixture(scope="module")
def fleet(tmp_path_factory):
    root = tmp_path_factory.mktemp("pipeline")
    return {"root": root,
            "h1": build_sortie(root / "h1", uav="HERO1", sid="S1", seed=1),
            "h2": build_sortie(root / "h2", uav="HERO2", sid="S2", seed=2),
            "bad": build_sortie(root / "bad", uav="HERO3", sid="S3", seed=3, drops=(5, 40)),
            "nolever": build_sortie(root / "nolever", uav="HERO4", sid="S4", seed=4, gnss_lever=None)}


def sortie(s, *, pos=True, pos_path=None, rover_ubx=None):
    return SortieInput(s["uav"], s["bin"], s["sess"], pos_path=(pos_path or s["pos"]) if pos else None,
                       rover_ubx=rover_ubx)


def run(out, sorties, **kw):
    kw.setdefault("sensor_ts_semantics", "exposure_start")
    kw.setdefault("gps_lag_s", GPS_LAG)
    return pipeline.process_campaign(sorties, out, **kw)


def truth_err(s, row):
    """(水平 m, 橢球高 m, GPST s) 誤差。"""
    t = s["center_fc"][row["seq"]]
    lat, lon, h = s["truth"].point_llh(t, tuple(GPS_POS + LEVER))
    _, _, dist = GEOD.inv(lon, lat, row["lon_deg"], row["lat_deg"])
    return dist, row["h_ell_m"] - (h + N_SITE), row["t_gpst_s"] - (GPST0 + t)


def strip_first_seqs(rows):
    first = {}
    for r in sorted(rows, key=lambda r: r["seq"]):
        first.setdefault(r["strip"], r["seq"])
    return set(first.values())


# ─────────────────────────────────────────────────────────────────────────────
# 只有飛控（fc_interp）
# ─────────────────────────────────────────────────────────────────────────────

def test_fc_only_campaign_writes_every_product_and_passes_qa(fleet, tmp_path):
    h1, h2 = fleet["h1"], fleet["h2"]
    res = run(tmp_path / "out", [sortie(h1, pos=False), sortie(h2, pos=False)], ground_h_m=GROUND_ELL)
    assert res.passed and res.qa["reasons"] == [] and res.errors == []
    n1, n2 = len(h1["pi"]["frames"]), len(h2["pi"]["frames"])
    assert len(res.rows) == n1 + n2 and Counter(r["source"] for r in res.rows) == {"fc_interp": n1 + n2}
    for s in (h1, h2):
        for r in (x for x in res.rows if x["uav"] == s["uav"]):
            dh, dv, dt = truth_err(s, r)
            assert dh < 0.01 and abs(dv) < 0.01 and abs(dt) < 2e-3
    # 交付物：同一份 F2、geo.txt 依 WGS84、一張影像一列
    out = res.out_dir
    from survey import deliverable
    assert len(deliverable.read_photos_csv(out / "photos.csv")) == n1 + n2
    geo = (out / "odm" / "geo.txt").read_text(encoding="utf-8").splitlines()
    assert geo[0] == "EPSG:4326" and len(geo) == n1 + n2 + 1
    assert sorted(p.name for p in res.images) == sorted(ln.split()[0] for ln in geo[1:])
    assert sorted(p.name for p in (out / "odm" / "images").iterdir()) == sorted(p.name for p in res.images)
    # qa.json：有效設定（含 SensorTimestamp 語意）與每架 QA
    q = json.loads((out / "qa.json").read_text(encoding="utf-8"))
    assert q["effective"]["sensor_ts_semantics"] == "exposure_start" and q["effective"]["ppk"] is None
    assert q["effective"]["sorties"]["HERO2"]["pose"]["lever_arm_effective_m"] == pytest.approx(list(GPS_POS + LEVER))
    assert q["qa"]["passed"] is True and set(q["qa"]["per_sortie"]) == {"HERO1", "HERO2"}
    assert q["n_photos"] == n1 + n2 and q["sources"] == {"fc_interp": n1 + n2}
    # 視數（沒有多邊形 ⇒ 在被拍到的範圍內）、GSD（中位 AGL × 像素 ÷ 焦距）、讀出時間取自會期
    assert res.qa["exposure"]["computed"] and res.qa["exposure"]["source"] == "survey.qa.footprint_multiplicity"
    agl = np.median([r["h_ell_m"] - GROUND_ELL for r in res.rows if r["uav"] == "HERO1"])
    assert res.odm["gsd_cm"] == pytest.approx(round(agl * 97.66e-6 / 6e-3 * 100, 1))
    assert res.odm["rolling_shutter_readout_ms"] == 20.0 and res.odm["n_images"] == n1 + n2


def test_fc_gpst_depends_on_gps_lag_but_is_recorded(fleet, tmp_path):
    """飛控 GPST 帶接收機延遲：lag 給 0 就整體早 0.08 s —— 生效值寫進 effective（不靜默）。"""
    h1 = fleet["h1"]
    res = run(tmp_path / "out", [sortie(h1, pos=False)], gps_lag_s=0.0)
    dts = [truth_err(h1, r)[2] for r in res.rows]
    assert np.median(dts) == pytest.approx(-GPS_LAG, abs=2e-3)
    assert res.effective["gps_lag_s"] == 0.0


def test_geoid_gives_orthometric_heights_and_blank_n_is_not_zero(fleet, tmp_path):
    h1 = fleet["h1"]
    res = run(tmp_path / "a", [sortie(h1, pos=False)])
    assert all(r["h_orth_m"] is None and r["crs_v"] == "ellipsoidal" for r in res.rows)
    with pytest.raises(PipelineError, match="N=0"):
        run(tmp_path / "b", [sortie(h1, pos=False)], height="orthometric")
    g = datum.GeoidModel("EGM96", N_SITE, "test")
    res = run(tmp_path / "c", [sortie(h1, pos=False)], geoid=g, height="orthometric", ground_h_m=30.0)
    for r in res.rows:
        assert r["crs_v"] == "EGM96" and r["h_orth_m"] == pytest.approx(r["h_ell_m"] - N_SITE, abs=1e-9)
    geo = (res.out_dir / "odm" / "geo.txt").read_text(encoding="utf-8").splitlines()
    first = next(r for r in res.rows if f"{r['image_id']}.jpg" == geo[1].split()[0])
    assert float(geo[1].split()[3]) == pytest.approx(first["h_orth_m"], abs=1e-3)      # geo.txt 用正高
    assert res.effective["geoid"] == {"datum": "EGM96", "source": "test", "undulation_m": N_SITE}


# ─────────────────────────────────────────────────────────────────────────────
# PPK
# ─────────────────────────────────────────────────────────────────────────────

def test_ppk_camera_centre_accuracy_and_tm2_time(fleet, tmp_path):
    h1 = fleet["h1"]
    # gps_lag_s=0：飛控 GPST 會早 0.08 s；PPK 列的時間來自 TIM-TM2，不受影響
    res = run(tmp_path / "out", [sortie(h1)], ppk=ppk_settings(), geo_projection=GRS80,
              gps_lag_s=0.0)
    assert res.passed, res.qa["reasons"]
    assert Counter(r["source"] for r in res.rows) == {"ppk": len(h1["pi"]["frames"])}
    assert {r["crs_h"] for r in res.rows} == {"e-GNSS[2025]"} and {r["crs_v"] for r in res.rows} == {"ellipsoidal"}
    errs = {r["seq"]: truth_err(h1, r) for r in res.rows}
    e = np.asarray(list(errs.values()))
    firsts = strip_first_seqs(res.rows)
    inner = np.asarray([v for k, v in errs.items() if k not in firsts])
    # 帶內：天線內插（直線）＋ 0.4 m 槓桿臂 ＋ 20 µs 邊緣抖動 ⇒ 遠小於 1 cm
    assert inner[:, 0].max() < 2e-3 and np.abs(inner[:, 1]).max() < 5e-3
    # 轉彎改出後第一張：合成航線的曲率瞬間從 1/60 m 跳成 0，10 Hz Hermite 在這個 epoch 約 1 cm（實機滾轉約 1 s，不會這樣）
    assert e[:, 0].max() < 0.02 and np.sqrt(np.mean(e[:, 0] ** 2)) < 5e-3
    assert np.abs(e[:, 2]).max() < 2e-4                               # GPST 來自 TM2，不帶 0.08 s 延遲
    p = res.sorties[0].ppk
    assert p["used"] == len(res.rows) and p["rejected"] == {}
    assert p["pairing"]["n_paired"] == len(res.rows) and p["edges"]["pvt_checked"] is True
    assert p["pairing"]["rate_ppm"] == pytest.approx(42.0, abs=0.5)     # Pi 時鐘 +42 ppm
    assert p["lever_arm_frd_m"] == pytest.approx(list(GNSS_LEVER))
    assert all(not w["below_threshold"] for w in p["fix_windows"]) and len(p["fix_windows"]) == 3


def test_ppk_exposure_time_follows_the_chosen_semantics(fleet, tmp_path):
    """合成會期的真值是 exposure_start；選 readout_start ⇒ 曝光中心早一個曝光時間 E = 0.8 ms。
    PPK 列與 FC 列必須用同一個曝光中心定義（同樣早 E），不能 PPK 自己固定用 +E/2。"""
    h1 = fleet["h1"]
    res = run(tmp_path / "out", [sortie(h1)], ppk=ppk_settings(), geo_projection=GRS80,
              sensor_ts_semantics="readout_start")
    dts = np.asarray([truth_err(h1, r)[2] for r in res.rows])
    assert np.all(np.abs(dts + 0.8e-3) < 1e-4)
    fc = run(tmp_path / "fc", [sortie(h1, pos=False)], sensor_ts_semantics="readout_start")
    t_fc = {r["seq"]: r["t_fc_us"] for r in fc.rows}
    assert all(r["t_fc_us"] == t_fc[r["seq"]] for r in res.rows)


def test_ppk_position_comes_from_the_pos_file(fleet, tmp_path):
    h1 = fleet["h1"]
    base = run(tmp_path / "a", [sortie(h1)], ppk=ppk_settings(), geo_projection=GRS80)
    shifted = write_pos(tmp_path / "shift.pos", h1["truth"], dlat_deg=1e-5)
    moved = run(tmp_path / "b", [sortie(h1, pos_path=shifted)], ppk=ppk_settings(),
                geo_projection=GRS80)
    for a, b in zip(base.rows, moved.rows):
        assert b["lat_deg"] - a["lat_deg"] == pytest.approx(1e-5, abs=2e-9)
        assert b["lon_deg"] == pytest.approx(a["lon_deg"], abs=2e-9)
        # 位置以外的 FC 欄位原樣保留
        for k in ("image_id", "file", "uav", "strip", "seq", "t_fc_us", "yaw_deg", "pitch_deg", "roll_deg"):
            assert a[k] == b[k]


def test_ppk_merge_rules_float_strip_keeps_fc_with_flags(fleet, tmp_path):
    h1 = fleet["h1"]
    _, t_on, t_off = strip_intervals(h1["truth"])[1]                    # 第 2 帶整條 float
    flo = write_pos(tmp_path / "float.pos", h1["truth"], q_of_t=lambda t: 2 if t_on - 0.5 <= t <= t_off + 0.5 else 1)
    res = run(tmp_path / "out", [sortie(h1, pos_path=flo)], ppk=ppk_settings(),
              geo_projection=GRS80)
    s2 = [r for r in res.rows if r["strip"] == 2]
    others = [r for r in res.rows if r["strip"] != 2]
    assert s2 and others
    for r in s2:
        fl = r["flags"].split(";")
        assert r["source"] == "fc_interp" and r["crs_h"] == "EPSG:4326"
        assert "ppk_rejected" in fl and "ppk_float" in fl
        dh, dv, _ = truth_err(h1, r)
        assert dh < 0.01                                                 # FC 位姿本身沒被動到
    assert all(r["source"] == "ppk" and r["crs_h"] == "e-GNSS[2025]" for r in others)
    p = res.sorties[0].ppk
    assert p["rejected"] == {"ppk_float": len(s2)} and p["used"] == len(others)
    win = {w["label"]: w for w in p["fix_windows"]}
    assert win["2"]["below_threshold"] and win["2"]["fix_ratio"] == 0.0
    assert not win["1"]["below_threshold"] and not win["3"]["below_threshold"]
    assert not res.passed and any("PPK 第 2 帶 fix 比例 0.0%" in x for x in res.qa["reasons"])
    # 框架不混用：geo.txt 以明確的座標系寫出，原本兩種框架各幾張寫進警告
    assert (res.out_dir / "odm" / "geo.txt").read_text(encoding="utf-8").splitlines()[0] == GRS80
    assert any(f"EPSG:4326：{len(s2)} 張" in w for w in res.warnings)
    assert any(f"e-GNSS[2025]：{len(others)} 張" in w for w in res.warnings)


def test_crs_refusal_without_explicit_projection(fleet, tmp_path):
    h1 = fleet["h1"]
    res = run(tmp_path / "a", [sortie(h1)], ppk=ppk_settings())
    assert res.paths["geo_txt"] is None and res.images == [] and not (res.out_dir / "odm" / "geo.txt").exists()
    assert any("不全是 EPSG:4326" in e for e in res.errors)
    assert not res.passed and any(x.startswith("交付物：geo.txt 未寫") for x in res.qa["reasons"])
    # photos.csv 照寫，各列帶自己的框架（不改寫成 WGS84）
    from survey import deliverable
    assert {r["crs_h"] for r in deliverable.read_photos_csv(res.paths["photos_csv"])} == {"e-GNSS[2025]"}
    # 投影座標系（公尺）不能當 geo.txt 標頭：經緯度會被當成公尺
    with pytest.raises(PipelineError, match="不是經緯度"):
        run(tmp_path / "b", [sortie(h1)], ppk=ppk_settings(), geo_projection="EPSG:3826")
    with pytest.raises(PipelineError, match="PROJ"):
        run(tmp_path / "c", [sortie(h1)], ppk=ppk_settings(), geo_projection="e-GNSS[2025]")
    with pytest.raises(PipelineError, match="框架名稱必填"):
        PpkSettings(frame="  ")


def test_unknown_gnss_lever_arm_keeps_every_photo_on_fc(fleet, tmp_path):
    s = fleet["nolever"]
    res = run(tmp_path / "out", [sortie(s)], ppk=ppk_settings(), geo_projection=GRS80)
    assert {r["source"] for r in res.rows} == {"fc_interp"}
    assert all({"ppk_rejected", "lever_arm_unknown"} <= set(r["flags"].split(";")) for r in res.rows)
    assert any("gnss_lever_arm_m 是 null" in w for w in res.warnings)
    assert not res.passed and any("沒有任何一張照片採用 PPK" in x for x in res.qa["reasons"])


def test_ppk_input_validation(fleet, tmp_path):
    h1 = fleet["h1"]
    with pytest.raises(PipelineError, match="沒有 .pos，也沒有開啟 RTKLIB"):
        run(tmp_path / "a", [sortie(h1, pos=False)], ppk=ppk_settings())
    with pytest.raises(PipelineError, match="RINEX 或 UBX"):
        PpkSettings(frame="e-GNSS[2025]", base=rtklib.BaseStation(25.1, 121.2, 50.0, "e-GNSS[2025]"),
                    run_rtklib=True)
    with pytest.raises(PipelineError, match="基站框架"):
        PpkSettings(frame="e-GNSS[2025]", base=rtklib.BaseStation(25.1, 121.2, 50.0, "TWD97"))
    # M2b（2026-10-21）：既有 .pos 路線也必須給基站座標（要核對 .pos 表頭的 % ref pos）
    with pytest.raises(PipelineError, match="用既有 .pos 也必須給基站座標"):
        PpkSettings(frame="e-GNSS[2025]")


def test_rtklib_branch_goes_through_survey_ppk_rtklib(fleet, tmp_path, monkeypatch):
    """開 RTKLIB 時：基站 UBX 先 convbin、再 run_ppk（都走 survey.ppk.rtklib；這裡換成假的，不開子行程）。"""
    h1 = fleet["h1"]
    base_ubx = tmp_path / "base.ubx"
    base_ubx.write_bytes(b"\xb5\x62")
    calls = {"run": [], "ppk": []}
    monkeypatch.setattr(rtklib, "find_executable", lambda name, explicit=None: f"/fake/{name}")

    def fake_run(cmd, timeout_s):
        calls["run"].append(list(map(str, cmd)))
        Path(cmd[cmd.index("-o") + 1]).write_text("obs", encoding="utf-8")
        Path(cmd[cmd.index("-n") + 1]).write_text("nav", encoding="utf-8")
        return ""

    def fake_run_ppk(rover_ubx, base_obs, base, workdir, **kw):
        calls["ppk"].append((Path(rover_ubx), Path(base_obs), base, Path(workdir), kw))
        Path(workdir).mkdir(parents=True, exist_ok=True)
        pos = Path(workdir) / "rover.pos"
        pos.write_bytes(h1["pos"].read_bytes())
        return rtklib.PpkRun(pos_path=pos, conf_path=Path(workdir) / "c.conf", rover_obs=Path(workdir) / "r.obs",
                             rover_nav=Path(workdir) / "r.nav", options={"pos1-posmode": "kinematic"},
                             commands=[["/fake/convbin"], ["/fake/rnx2rtkp"]])

    monkeypatch.setattr(rtklib, "_run", fake_run)
    monkeypatch.setattr(rtklib, "run_ppk", fake_run_ppk)
    base = rtklib.BaseStation(25.0995, 121.1995, 52.3, "e-GNSS[2025]", antenna_height_m=1.234)
    ppk = PpkSettings(frame="e-GNSS[2025]", base=base, base_ubx=base_ubx, run_rtklib=True)
    res = run(tmp_path / "out", [sortie(h1, pos=False)], ppk=ppk, geo_projection=GRS80)
    assert res.passed, res.qa["reasons"]
    assert len(calls["run"]) == 1 and calls["run"][0][0] == "/fake/convbin" and calls["run"][0][-1] == str(base_ubx)
    (rover, obs, b, wd, kw) = calls["ppk"][0]
    assert rover == h1["sess"] / "gnss" / "rover.ubx" and b == base
    assert obs == res.out_dir / "ppk" / "base" / "base.obs" and wd == res.out_dir / "ppk" / "HERO1"
    assert [Path(p) for p in kw["extra_nav"]] == [res.out_dir / "ppk" / "base" / "base.nav"]
    assert {r["source"] for r in res.rows} == {"ppk"}
    q = json.loads((res.out_dir / "qa.json").read_text(encoding="utf-8"))
    assert q["effective"]["ppk"]["base"]["antenna_height_m"] == 1.234
    assert q["effective"]["ppk"]["base_conversion"]["obs"] == "ppk/base/base.obs"
    assert q["qa"]["per_sortie"]["HERO1"]["ppk"]["trajectory"]["source"] == "rtklib"
    # RTKLIB 路線的 .pos 一樣核對 % ref pos 與 RAWX 跨度（rnx2rtkp 的 rb 就是 ant2-pos 給的標石座標）
    assert q["qa"]["per_sortie"]["HERO1"]["ppk"]["trajectory"]["ref_pos_check"]["passed"] is True
    assert q["qa"]["per_sortie"]["HERO1"]["ppk"]["trajectory"]["rover_span"]["n_rawx"] > 0


def test_rtklib_not_installed_is_refused_before_processing(fleet, tmp_path, monkeypatch):
    def nf(name, explicit=None):
        raise rtklib.RtklibNotFound(f"找不到 {name}")
    monkeypatch.setattr(rtklib, "find_executable", nf)
    obs = tmp_path / "base.obs"
    obs.write_text("x", encoding="utf-8")
    ppk = PpkSettings(frame="e-GNSS[2025]", base=rtklib.BaseStation(25.1, 121.2, 50.0, "e-GNSS[2025]"),
                      base_obs=obs, run_rtklib=True)
    with pytest.raises(PipelineError, match="未安裝 RTKLIB"):
        run(tmp_path / "out", [sortie(fleet["h1"], pos=False)], ppk=ppk)
    assert not (tmp_path / "out").exists()


# ─────────────────────────────────────────────────────────────────────────────
# PPK 數值鎖（docs/評估_2026-10-21_EmlidStudio_PPK接入.md §4 M1／M2a／M2b／M3／events.pos）
# 全部用合成數值；不放論壇下載的 Emlid Studio 檔或其座標。
# ─────────────────────────────────────────────────────────────────────────────

def _ppk_fail(res):
    """整架 PPK 被鎖擋下：照片全部保留 fc_interp、旗標 ppk_unavailable、QA 未通過；回傳錯誤訊息。"""
    p = res.sorties[0].ppk
    assert p.get("error"), p
    assert not res.passed and p["error"] in res.qa["reasons"]
    assert {r["source"] for r in res.rows} == {"fc_interp"}
    assert all("ppk_unavailable" in r["flags"].split(";") for r in res.rows)
    return p["error"]


def test_ppk_locks_pass_and_are_recorded_in_qa_json(fleet, tmp_path):
    h1 = fleet["h1"]
    res = run(tmp_path / "out", [sortie(h1)], ppk=ppk_settings(), geo_projection=GRS80)
    assert res.passed, res.qa["reasons"]
    q = json.loads((res.out_dir / "qa.json").read_text(encoding="utf-8"))
    p = q["qa"]["per_sortie"]["HERO1"]["ppk"]
    tr = p["trajectory"]
    first, last = rawx_span(h1["truth"])
    # M1：表頭中繼資料
    assert tr["meta"]["program"] == "RTKPOST ver.demo5 b34k" and tr["meta"]["inp_files"] == ["rover.obs", "base.obs"]
    assert tr["meta"]["obs_start"]["gpst_s"] == pytest.approx(first, abs=1e-6)
    assert tr["meta"]["obs_end"]["gpst_s"] == pytest.approx(last, abs=1e-6)
    assert tr["meta"]["ref_pos"]["llh"] == pytest.approx([BASE.lat_deg, BASE.lon_deg, BASE.h_ell_m])
    # M2a：RAWX 跨度與 obs start／end
    sp = tr["rover_span"]
    assert sp["rawx_first_gpst_s"] == pytest.approx(first, abs=1e-6)
    assert sp["rawx_last_gpst_s"] == pytest.approx(last, abs=1e-6)
    assert sp["n_rawx_week0"] == 0 and sp["first_solution_gpst_s"] == pytest.approx(first, abs=1e-6)
    assert tr["age_s"]["median_s"] == 0.0 and tr["age_s"]["n"] == tr["n_epochs"]
    # M2b：ref pos 與基站座標
    rc = tr["ref_pos_check"]
    assert rc["passed"] and rc["d_h_m"] < 1e-3 and abs(rc["d_v_m"]) < 1e-3 and rc["base_frame"] == FRAME
    # M3：PPK 對飛控（合成資料同一個真值 ⇒ mm 級）
    fc = p["fc_check"]
    assert fc["passed"] and fc["n"] == len(res.rows) == fc["n_v"]
    assert fc["median_h_m"] < 0.01 and abs(fc["median_v_m"]) < 0.01
    assert q["effective"]["ppk"]["fc_gross_h_m"] == pipeline.PPK_FC_MAX_MEDIAN_H_M == 15.0
    assert q["effective"]["ppk"]["fc_gross_v_m"] == pipeline.PPK_FC_MAX_MEDIAN_V_M == 15.0
    rows = {r[1]: r for r in pipeline.qa_table_rows(res)}
    assert rows["PPK 對飛控粗差（中位數）"][3] is True and rows["PPK 基站座標（% ref pos）"][3] is True
    assert rows["PPK 觀測跨度（obs start／end 對 rover.ubx）"][2].startswith("開始 +0.000 s")


def test_ppk_emlid_studio_shaped_pos_goes_through(fleet, tmp_path):
    """ES 1.5 的表頭形狀：program「ES 1.5」、沒有處理選項行、hh:mm:ss GPST、沒有速度欄（合成數值）。"""
    h1 = fleet["h1"]
    es = write_pos(tmp_path / "rover_raw_20261020.pos", h1["truth"], program="ES 1.5", outopt=False,
                   timeform="hms", inp=("C:/data/rover_raw_20261020.26O", "C:/data/rover_raw_20261020.26P"))
    txt = es.read_text(encoding="utf-8")
    assert "% pos mode" not in txt and "vn(m/s)" not in txt
    res = run(tmp_path / "out", [sortie(h1, pos_path=es)], ppk=ppk_settings(), geo_projection=GRS80)
    assert res.passed, res.qa["reasons"]
    tr = res.sorties[0].ppk["trajectory"]
    assert tr["meta"]["program"] == "ES 1.5" and tr["has_velocity"] is False
    assert {r["source"] for r in res.rows} == {"ppk"}


def test_m2a_time_column_18s_early_is_refused(fleet, tmp_path):
    """標 GPST、實際是 UTC（時間欄整體 −18 s，表頭 obs start 仍是 GPST）：pos.py 照收，第一筆解早於
    obs start 18.000 s ⇒ 擋；擋下時已讀到的跨度資料也寫進摘要。"""
    from survey.ppk.pos import read_pos
    h1 = fleet["h1"]
    utc = write_pos(tmp_path / "utc.pos", h1["truth"], dt_s=-18.0)
    assert len(read_pos(utc)) > 0                                      # 格式本身讀得進來
    res = run(tmp_path / "out", [sortie(h1, pos_path=utc)], ppk=ppk_settings(), geo_projection=GRS80)
    err = _ppk_fail(res)
    assert "PpkLockError" in err and "比表頭 obs start" in err and "早 18.000 s" in err and "UTC 卻標成 GPST" in err
    sp = res.sorties[0].ppk["trajectory"]["rover_span"]
    assert sp["obs_start_gpst_s"] - sp["first_solution_gpst_s"] == pytest.approx(18.0, abs=1e-6)
    q = json.loads((res.out_dir / "qa.json").read_text(encoding="utf-8"))
    assert q["qa"]["per_sortie"]["HERO1"]["ppk"]["trajectory"]["meta"]["obs_start"] is not None


def test_m2a_rover_ubx_of_another_recording_is_refused(fleet, tmp_path):
    """rover.ubx 換成別次（晚 1 小時）的觀測：.pos 的 obs start 早於它第一個 RAWX ⇒ 擋（start 鎖）。"""
    h1 = fleet["h1"]
    first, last = rawx_span(h1["truth"])
    other = tmp_path / "other.ubx"
    other.write_bytes(rover_ubx_bytes([e + 3600 * 10**9 for e in h1["edges_ns"]], span_s=(first + 3600, last + 3600)))
    res = run(tmp_path / "out", [sortie(h1, rover_ubx=other)], ppk=ppk_settings(), geo_projection=GRS80)
    err = _ppk_fail(res)
    assert "obs start" in err and "第一個 RAWX epoch" in err and "早 3600.000 s" in err


def test_m2a_truncated_rover_ubx_is_refused(fleet, tmp_path):
    """rover.ubx 比 .pos 的 obs end 早 5 s 結束（截斷或別次）⇒ 擋（end 鎖）。"""
    h1 = fleet["h1"]
    first, last = rawx_span(h1["truth"])
    cut = tmp_path / "cut.ubx"
    cut.write_bytes(rover_ubx_bytes(h1["edges_ns"], span_s=(first, last - 5.0)))
    res = run(tmp_path / "out", [sortie(h1, rover_ubx=cut)], ppk=ppk_settings(), geo_projection=GRS80)
    err = _ppk_fail(res)
    assert "obs end" in err and "最後一個 RAWX epoch" in err and "晚 5.000 s" in err


def test_m2a_missing_obs_lines_are_refused(fleet, tmp_path):
    h1 = fleet["h1"]
    first, last = rawx_span(h1["truth"])
    for name, obs, key in (("nostart", (None, last), "obs start"), ("noend", (first, None), "obs end"),
                           ("none", None, "obs start")):
        p = write_pos(tmp_path / f"{name}.pos", h1["truth"], obs=obs)
        res = run(tmp_path / name, [sortie(h1, pos_path=p)], ppk=ppk_settings(), geo_projection=GRS80)
        assert f"沒有可用的 % {key}" in _ppk_fail(res), name


def test_m2a_week0_rawx_epochs_are_ignored(fleet, tmp_path):
    """Reach 檔開頭有 week=0 的 RAWX：不誤擋；也不能把它當成「第一個 epoch」讓 start 鎖失效。"""
    h1 = fleet["h1"]
    first, last = rawx_span(h1["truth"])
    w0 = tmp_path / "w0.ubx"
    w0.write_bytes(rover_ubx_bytes(h1["edges_ns"], span_s=(first, last), week0_lead=7))
    ok = run(tmp_path / "ok", [sortie(h1, rover_ubx=w0)], ppk=ppk_settings(), geo_projection=GRS80)
    assert ok.passed, ok.qa["reasons"]
    assert ok.sorties[0].ppk["trajectory"]["rover_span"]["n_rawx_week0"] == 7
    early = write_pos(tmp_path / "early.pos", h1["truth"], obs=(first - 1.0, last))
    res = run(tmp_path / "early", [sortie(h1, pos_path=early, rover_ubx=w0)], ppk=ppk_settings(), geo_projection=GRS80)
    assert "第一個 RAWX epoch" in (err := _ppk_fail(res)) and "早 1.000 s" in err


def test_age_over_one_second_warns_and_missing_age_is_said(fleet, tmp_path):
    h1 = fleet["h1"]
    old = write_pos(tmp_path / "age.pos", h1["truth"], age=2.5)
    res = run(tmp_path / "a", [sortie(h1, pos_path=old)], ppk=ppk_settings(), geo_projection=GRS80)
    assert res.passed, res.qa["reasons"]                               # 警告，不擋
    assert any("age(s) 中位數 2.50 s > 1 s" in w for w in res.warnings)
    assert res.sorties[0].ppk["trajectory"]["age_s"]["median_s"] == 2.5
    noage = tmp_path / "noage.pos"
    noage.write_text(h1["pos"].read_text(encoding="utf-8").replace(" age(s)", " agex(s)"), encoding="utf-8")
    res = run(tmp_path / "b", [sortie(h1, pos_path=noage)], ppk=ppk_settings(), geo_projection=GRS80)
    assert any("沒有 age(s) 欄" in w for w in res.warnings) and res.sorties[0].ppk["trajectory"]["age_s"] is None
    res = run(tmp_path / "c", [sortie(h1)], ppk=ppk_settings(), geo_projection=GRS80)
    assert not any("age(s)" in w for w in res.warnings)


def test_m2b_base_must_match_ref_pos_within_5mm(fleet, tmp_path):
    h1 = fleet["h1"]
    dlat_1cm = 0.01 / 110_790.0                                        # 緯度 25.1° 的 1 cm
    for name, kw, want in (("h10", dict(lat_deg=BASE.lat_deg + dlat_1cm), "水平 10.0 mm"),
                           ("v10", dict(h_ell_m=BASE.h_ell_m + 0.010), "垂直 -10.0 mm"),
                           ("h4", dict(lat_deg=BASE.lat_deg + 0.4 * dlat_1cm), None),
                           ("v4", dict(h_ell_m=BASE.h_ell_m - 0.004), None)):
        res = run(tmp_path / name, [sortie(h1)], ppk=ppk_settings(base=replace(BASE, **kw)), geo_projection=GRS80)
        if want is None:
            assert res.passed, (name, res.qa["reasons"])
        else:
            err = _ppk_fail(res)
            assert "% ref pos" in err and want in err and "容許各 5 mm" in err, (name, err)


def test_m2b_missing_or_unreadable_ref_pos_is_refused(fleet, tmp_path):
    h1 = fleet["h1"]
    nor = write_pos(tmp_path / "noref.pos", h1["truth"], ref_pos=None)
    err = _ppk_fail(run(tmp_path / "a", [sortie(h1, pos_path=nor)], ppk=ppk_settings(), geo_projection=GRS80))
    assert "沒有 % ref pos" in err and "不是相對定位" in err
    dms = write_pos(tmp_path / "dms.pos", h1["truth"], ref_pos_text="  25 05 58.20000  121 11 58.20000    52.3000")
    err = _ppk_fail(run(tmp_path / "b", [sortie(h1, pos_path=dms)], ppk=ppk_settings(), geo_projection=GRS80))
    assert "% ref pos 解不出來" in err


def test_m2b_ecef_ref_pos_is_converted_before_comparing(fleet, tmp_path):
    h1 = fleet["h1"]
    xyz = write_pos(tmp_path / "xyz.pos", h1["truth"], fmt="xyz")
    assert "x-ecef(m)" in xyz.read_text(encoding="utf-8")
    res = run(tmp_path / "ok", [sortie(h1, pos_path=xyz)], ppk=ppk_settings(), geo_projection=GRS80)
    assert res.passed, res.qa["reasons"]
    rc = res.sorties[0].ppk["trajectory"]["ref_pos_check"]
    assert rc["d_h_m"] < 1e-3 and abs(rc["d_v_m"]) < 1e-3
    off = run(tmp_path / "off", [sortie(h1, pos_path=xyz)], ppk=ppk_settings(base=replace(BASE, h_ell_m=52.32)),
              geo_projection=GRS80)
    assert "垂直 -20.0 mm" in _ppk_fail(off)


def test_m2b_ref_pos_at_antenna_height_gets_a_hint(fleet, tmp_path):
    """解算軟體把天線高加進基站高（ref pos 是 ARP）⇒ 仍擋，但訊息指出差值就是天線高。"""
    h1 = fleet["h1"]
    arp = write_pos(tmp_path / "arp.pos", h1["truth"],
                    ref_pos=(BASE.lat_deg, BASE.lon_deg, BASE.h_ell_m + BASE.antenna_height_m))
    err = _ppk_fail(run(tmp_path / "out", [sortie(h1, pos_path=arp)], ppk=ppk_settings(), geo_projection=GRS80))
    assert "恰好等於天線高 1.2340 m" in err


def test_m3_orthometric_height_entered_as_ellipsoidal_fails_qa(fleet, tmp_path):
    """基站高填成正高，而且 ES 與 AeroPlan 兩邊填同一個錯的值（% ref pos 一致 ⇒ M2b 抓不到）：
    整條軌跡低一個 N ⇒ M3 垂直差 −18.5 m ⇒ QA 不通過。門檻可調（寫進 effective）。"""
    h1 = fleet["h1"]
    low = replace(BASE, h_ell_m=BASE.h_ell_m - N_SITE)
    ortho = write_pos(tmp_path / "ortho.pos", h1["truth"], dh_m=-N_SITE,
                      ref_pos=(low.lat_deg, low.lon_deg, low.h_ell_m))
    res = run(tmp_path / "out", [sortie(h1, pos_path=ortho)], ppk=ppk_settings(base=low), geo_projection=GRS80)
    assert not res.passed
    msg = [x for x in res.qa["reasons"] if "PPK 與飛控位置差" in x]
    assert len(msg) == 1 and "垂直 -18.5 m" in msg[0], res.qa["reasons"]
    fc = res.sorties[0].ppk["fc_check"]
    assert not fc["passed"] and fc["median_v_m"] == pytest.approx(-N_SITE, abs=0.01) and fc["median_h_m"] < 0.01
    assert {r["source"] for r in res.rows} == {"ppk"}                  # 只讓 QA 不通過，不改合併規則
    assert {r[1]: r for r in pipeline.qa_table_rows(res)}["PPK 對飛控粗差（中位數）"][3] is False
    loose = run(tmp_path / "loose", [sortie(h1, pos_path=ortho)], ppk=ppk_settings(base=low, fc_gross_v_m=20.0),
                geo_projection=GRS80)
    assert loose.passed, loose.qa["reasons"]
    assert loose.effective["ppk"]["fc_gross_v_m"] == 20.0


def test_m3_time_shift_with_a_consistent_header_fails_qa(fleet, tmp_path):
    """時間欄與表頭一起早 18 s、rover.ubx 的 RAWX 前後多錄 30 s ⇒ M2a 抓不到；M3 水平差 ≈ 18 s × 21.5 m/s。"""
    h1 = fleet["h1"]
    first, last = rawx_span(h1["truth"])
    long_ubx = tmp_path / "long.ubx"
    long_ubx.write_bytes(rover_ubx_bytes(h1["edges_ns"], span_s=(first - 30.0, last + 30.0)))
    shifted = write_pos(tmp_path / "shift.pos", h1["truth"], dt_s=-18.0, obs=(first - 18.0, last - 18.0))
    res = run(tmp_path / "out", [sortie(h1, pos_path=shifted, rover_ubx=long_ubx)], ppk=ppk_settings(),
              geo_projection=GRS80)
    assert not res.passed and res.sorties[0].ppk.get("error") is None
    fc = res.sorties[0].ppk["fc_check"]
    assert not fc["passed"] and fc["median_h_m"] > 300.0, fc
    assert any("PPK 與飛控位置差 中位數 水平" in x for x in res.qa["reasons"])


def test_m3_pos_of_another_aircraft_fails_qa(fleet, tmp_path):
    """同一時段、北邊 33 m 平行飛的另一架的 .pos（時間跨度相同 ⇒ M2a 抓不到）⇒ 水平中位數差 > 15 m。"""
    h1 = fleet["h1"]
    other = write_pos(tmp_path / "other.pos", h1["truth"], dlat_deg=3e-4)
    res = run(tmp_path / "out", [sortie(h1, pos_path=other)], ppk=ppk_settings(), geo_projection=GRS80)
    assert not res.passed
    assert any("PPK 與飛控位置差 中位數 水平 33.2 m" in x for x in res.qa["reasons"]), res.qa["reasons"]


def test_m3_ordinary_differences_do_not_fail(fleet, tmp_path):
    """框架差＋飛控 GNSS 雜訊這類幾公尺的差不擋：北偏 3 m、高 +2.5 m、σ 水平 2 m／垂直 4 m。"""
    h1 = fleet["h1"]
    noisy = write_pos(tmp_path / "noisy.pos", h1["truth"], dlat_deg=3.0 / 110_790.0, dh_m=2.5, noise=(2.0, 4.0, 7))
    res = run(tmp_path / "out", [sortie(h1, pos_path=noisy)], ppk=ppk_settings(), geo_projection=GRS80)
    assert res.passed, res.qa["reasons"]
    fc = res.sorties[0].ppk["fc_check"]
    assert fc["passed"] and 1.0 < fc["median_h_m"] < 6.0 and 0.5 < fc["median_v_m"] < 4.5, fc


def test_m3_thresholds_and_missing_fc_height():
    """門檻要是正的有限公尺數；飛控沒有橢球高 ⇒ 垂直檢查沒做要說出來（不靜默當通過）。"""
    for bad in (0.0, -1.0, float("nan"), float("inf"), True, "15"):
        with pytest.raises(PipelineError, match="粗差門檻"):
            ppk_settings(fc_gross_h_m=bad)
        with pytest.raises(PipelineError, match="粗差門檻"):
            ppk_settings(fc_gross_v_m=bad)
    fc = {"lat_deg": 25.1, "lon_deg": 121.2, "h_ell_m": None}
    p = {"lat_deg": 25.1, "lon_deg": 121.2, "h_ell_m": 168.5}
    rec, reasons, warnings = pipeline._fc_gross_check("HERO1", [(fc, p)] * 3, ppk_settings())
    assert rec["n_v"] == 0 and rec["median_v_m"] is None and rec["passed"] and reasons == []
    assert any("垂直粗差檢查沒做" in w for w in warnings)
    rec, reasons, _ = pipeline._fc_gross_check("HERO1", [(dict(fc, h_ell_m=150.0), p)] * 3, ppk_settings())
    assert rec["median_v_m"] == pytest.approx(18.5) and not rec["passed"] and "垂直 +18.5 m" in reasons[0]


def test_events_pos_is_refused_before_anything_is_written(fleet, tmp_path):
    h1 = fleet["h1"]
    ev = tmp_path / "rover_raw_20261020_events.pos"
    ev.write_bytes(h1["pos"].read_bytes())
    with pytest.raises(PipelineError, match="_events.pos"):
        run(tmp_path / "out", [sortie(h1, pos_path=ev)], ppk=ppk_settings(), geo_projection=GRS80)
    assert not (tmp_path / "out").exists()
    assert pipeline.is_events_pos("A_EVENTS.POS") and not pipeline.is_events_pos("events.pos")
    assert not pipeline.is_events_pos(h1["pos"])


# ─────────────────────────────────────────────────────────────────────────────
# QA、視數、輸入檢查、可重現、取消
# ─────────────────────────────────────────────────────────────────────────────

def test_qa_failure_lists_unmatched_and_gates_odm(fleet, tmp_path):
    bad = fleet["bad"]
    res = run(tmp_path / "out", [sortie(bad, pos=False)])
    assert not res.passed
    assert any("2 個飛控拍照事件沒有對應的 Pi 邊緣（index 5, 40）" in x for x in res.qa["reasons"])
    assert any("張數不守恆" in x for x in res.qa["reasons"])
    with pytest.raises(PipelineError, match="QA 未通過"):
        pipeline.odm_job(res, node_url="http://127.0.0.1:3000", orthophoto_resolution_cm=3.0, rolling_shutter=True)
    job = pipeline.odm_job(res, node_url="http://127.0.0.1:3000", orthophoto_resolution_cm=3.0,
                           rolling_shutter=True, qa_override=True)
    assert job["qa_override"] is True and job["qa_passed"] is False and job["qa_reasons"] == res.qa["reasons"]
    assert {"name": "rolling-shutter-readout", "value": 20.0} in job["options"]
    assert {"name": "orthophoto-resolution", "value": 3.0} in job["options"]
    assert len(job["images"]) == len(res.images) and job["geo_txt"].endswith("geo.txt")
    rows = pipeline.qa_table_rows(res)
    items = {(r[1], r[3]) for r in rows}
    assert ("未配對飛控事件", False) in items and ("未配對 Pi 邊緣", True) in items
    assert any(r[1].startswith("張數守恆 第") and r[3] is False for r in rows)
    assert any(r[1].startswith("間距 第 1 帶") for r in rows) and any(r[1] == "時鐘擬合殘差" for r in rows)
    assert any(r[1].startswith("側傾") and r[3] is None for r in rows)


def test_views_over_polygon_use_exposure_verifier(fleet, tmp_path):
    res = run(tmp_path / "a", [sortie(fleet["h1"], pos=False)], ground_h_m=GROUND_ELL, polygon=POLY)
    ex = res.qa["exposure"]
    assert ex["computed"] and ex["source"] == "core.recon.exposure_verifier" and ex["scope"] == "測區多邊形內"
    assert set(ex["frac_ge"]) == {"5", "9"} and 0.9 < ex["frac_ge"]["5"] <= 1.0
    # 前向重疊（面積）：足跡 125 × 93.75 m 隨偏流 5° 旋轉、沿航跡位移 d ⇒ (1 − d·cosψ/L)(1 − d·sinψ/W)
    d, psi = 18.92, math.radians(5.0)
    assert ex["forward_overlap"]["median"] == pytest.approx((1 - d * math.cos(psi) / 93.75)
                                                            * (1 - d * math.sin(psi) / 125.0), abs=0.01)
    rows = pipeline.qa_table_rows(res)
    assert any(r[1] == "視數 ≥ 5 比例（測區多邊形內）" for r in rows)
    res2 = run(tmp_path / "b", [sortie(fleet["h1"], pos=False)])
    assert res2.qa["exposure"] == {"computed": False, "reason": "沒有給地面高（與所選高度同一基準）⇒ 不算視數"}
    assert res2.odm["gsd_cm"] is None


def test_inputs_are_checked_before_anything_is_written(fleet, tmp_path):
    h1 = fleet["h1"]
    with pytest.raises(TypeError):
        pipeline.process_campaign([sortie(h1, pos=False)], tmp_path / "x")          # 語意沒有預設值
    with pytest.raises(PipelineError, match="SensorTimestamp"):
        run(tmp_path / "x", [sortie(h1, pos=False)], sensor_ts_semantics="")
    with pytest.raises(PipelineError, match="與呼號不符"):
        run(tmp_path / "x", [SortieInput("HERO9", h1["bin"], h1["sess"])])
    with pytest.raises(PipelineError, match="呼號重複"):
        run(tmp_path / "x", [sortie(h1, pos=False), sortie(h1, pos=False)])
    with pytest.raises(PipelineError, match="至少要 3 點"):
        run(tmp_path / "x", [sortie(h1, pos=False)], polygon=POLY[:2])
    assert not (tmp_path / "x").exists()


# ─────────────────────────────────────────────────────────────────────────────
# 多區測區（R2：完成本區後地圖角點清空，作業區在 MainWindow.survey_areas）與 GPS 延遲
# ─────────────────────────────────────────────────────────────────────────────
WEST = [(25.1003, 121.2005), (25.1003, 121.2025), (25.1019, 121.2025), (25.1019, 121.2005)]
EAST = [(25.1003, 121.2025), (25.1003, 121.2045), (25.1019, 121.2045), (25.1019, 121.2025)]


def test_views_over_several_rings_equal_their_union(fleet, tmp_path):
    """POLY 切成東西兩半當兩個區（MainWindow.survey_area_polygons() 的形狀）⇒ 視數與單一 POLY 相同。"""
    one = run(tmp_path / "one", [sortie(fleet["h1"], pos=False)], ground_h_m=GROUND_ELL, polygon=POLY)
    two = run(tmp_path / "two", [sortie(fleet["h1"], pos=False)], ground_h_m=GROUND_ELL, polygons=[WEST, EAST])
    a, b = one.qa["exposure"], two.qa["exposure"]
    assert b["computed"] and b["source"] == "core.recon.exposure_verifier" and b["scope"] == "測區多邊形內"
    assert (a["n_polygons"], b["n_polygons"]) == (1, 2)
    assert b["frac_ge"] == pytest.approx(a["frac_ge"], abs=1e-12) and 0.9 < b["frac_ge"]["5"] <= 1.0
    assert b["area_m2"] == pytest.approx(a["area_m2"], rel=1e-9)
    assert b["forward_overlap"] == a["forward_overlap"]
    assert two.effective["polygons"] == [[list(p) for p in WEST], [list(p) for p in EAST]]
    assert one.effective["polygons"] == [[list(p) for p in POLY]]
    q = json.loads((two.out_dir / "qa.json").read_text(encoding="utf-8"))
    assert q["effective"]["polygons"] == [[list(p) for p in WEST], [list(p) for p in EAST]]
    # 一個區時用 polygons=[ring] 與舊的 polygon=ring 結果相同
    three = run(tmp_path / "three", [sortie(fleet["h1"], pos=False)], ground_h_m=GROUND_ELL, polygons=[POLY])
    assert three.qa["exposure"] == a and three.effective["polygons"] == one.effective["polygons"]


def test_polygon_inputs_are_refused_clearly(fleet, tmp_path):
    s = [sortie(fleet["h1"], pos=False)]
    nan_ring = [(float("nan"), 121.2), (25.1, 121.21), (25.11, 121.21)]
    for kw, msg in ((dict(polygon=POLY, polygons=[POLY]), "只能給一個"),
                    (dict(polygons=[]), "polygons 是空的"),
                    (dict(polygons=[POLY, POLY[:2]]), "第 2 塊至少要 3 點"),
                    (dict(polygon=[WEST, EAST]), "多區請用 polygons="),
                    (dict(polygons=[[(95.0, 121.2), (25.1, 121.21), (25.11, 121.21)]]), "超出範圍"),
                    (dict(polygons=[nan_ring]), "不是有限數值"),
                    (dict(polygons=[[("a", 121.2), (25.1, 121.21), (25.11, 121.21)]]), "（緯度, 經度）")):
        with pytest.raises(PipelineError, match=msg):
            run(tmp_path / "x", s, **kw)
    assert not (tmp_path / "x").exists()


@pytest.mark.parametrize("lag", [float("nan"), float("inf"), -0.01, 1.5, "0.08", True])
def test_gps_lag_must_be_a_plausible_number_of_seconds(fleet, tmp_path, lag):
    """接收機延遲以秒計、0 ≤ lag ≤ 1：NaN 會讓每張照片的 GPST 變成 NaN；120（毫秒誤當秒）要擋下。"""
    with pytest.raises(PipelineError, match="GPS 接收延遲"):
        run(tmp_path / "x", [sortie(fleet["h1"], pos=False)], gps_lag_s=lag)
    assert not (tmp_path / "x").exists()


def test_input_sources_are_recorded_only_when_given(fleet, tmp_path):
    src = {"gps_lag_s": "操作員輸入", "polygons": "主視窗的作業區（2 塊）"}
    res = run(tmp_path / "a", [sortie(fleet["h1"], pos=False)], input_sources=src)
    assert res.effective["input_sources"] == src
    assert json.loads((tmp_path / "a" / "qa.json").read_text(encoding="utf-8"))["effective"]["input_sources"] == src
    res = run(tmp_path / "b", [sortie(fleet["h1"], pos=False)])
    assert "input_sources" not in res.effective                       # 沒給就不多一個鍵（輸出不變）
    with pytest.raises(PipelineError, match="input_sources"):
        run(tmp_path / "c", [sortie(fleet["h1"], pos=False)], input_sources={"gps_lag_s": 0.08})


def test_outputs_are_deterministic_and_never_silently_overwritten(fleet, tmp_path):
    ss = [sortie(fleet["h1"]), sortie(fleet["h2"])]
    kw = dict(ppk=ppk_settings(), geo_projection=GRS80, ground_h_m=GROUND_ELL)
    a = run(tmp_path / "a", ss, **kw)
    b = run(tmp_path / "b", ss, **kw)
    files = sorted(p.relative_to(a.out_dir) for p in a.out_dir.rglob("*") if p.is_file())
    assert files == sorted(p.relative_to(b.out_dir) for p in b.out_dir.rglob("*") if p.is_file())
    assert len(files) == 4 + 1 + len(a.images)
    for f in files:
        assert (a.out_dir / f).read_bytes() == (b.out_dir / f).read_bytes(), f
    assert b"\r\n" not in (a.out_dir / "qa.json").read_bytes()
    with pytest.raises(PipelineError, match="已有成果"):
        run(tmp_path / "a", ss, **kw)
    c = run(tmp_path / "a", ss, overwrite=True, **kw)
    assert (c.out_dir / "photos.csv").read_bytes() == (b.out_dir / "photos.csv").read_bytes()


def test_cancel_between_steps(fleet, tmp_path):
    seen = []
    with pytest.raises(PipelineCanceled):
        run(tmp_path / "out", [sortie(fleet["h1"], pos=False), sortie(fleet["h2"], pos=False)],
            progress=lambda m, f: seen.append(m), should_cancel=lambda: len(seen) >= 1)
    assert len(seen) == 1 and not (tmp_path / "out" / "photos.csv").exists()


# ─────────────────────────────────────────────────────────────────────────────
# NodeODM（假的 HTTP 端）
# ─────────────────────────────────────────────────────────────────────────────

class _Resp:
    def __init__(self, data=None, *, status=200, body=None, ctype="application/json"):
        self._data, self.status_code, self._body, self.headers = data, status, body, {"Content-Type": ctype}

    def json(self):
        if self._data is None:
            raise ValueError("not json")
        return self._data

    def iter_content(self, chunk_size=1):
        for i in range(0, len(self._body), chunk_size):
            yield self._body[i:i + chunk_size]


def _odm_zip(evil=False) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr("odm_orthophoto/odm_orthophoto.tif", b"tif")
        zf.writestr("orthophoto_tiles/tilemapresource.xml", "<TileMap/>")
        zf.writestr("orthophoto_tiles/18/219834/146802.png", b"png")
        if evil:
            zf.writestr("../escape.txt", b"x")
    return buf.getvalue()


class FakeNodeOdm:
    """session 層的假 NodeODX：/info、/options、init → upload（串流 multipart）→ commit、info、output、all.zip。

    2026-10-20 起客戶端走分段上傳（舊版是一個 POST /task/new）；完整的 HTTP 行為（故障、權杖、接回）在
    tests/test_survey_odm_fake_node.py 用真的本機 HTTP 假節點測。
    """

    def __init__(self, *, codes=(20, 40), evil=False):
        self.posted, self.codes, self.evil = [], list(codes), evil
        self.uuid, self.init_fields, self.uploaded = None, None, []

    def post(self, url, params=None, data=None, headers=None, timeout=None, **kw):
        from urllib.parse import urlsplit

        from tests.fake_nodeodx import parse_multipart
        path = urlsplit(url).path
        if path == "/task/new/init":
            self.uuid = headers["set-uuid"]
            self.init_fields, _ = parse_multipart(data, headers["Content-Type"])
            self.posted.append({"url": url, "names": []})
            return _Resp({"uuid": self.uuid})
        if path.startswith("/task/new/upload/"):
            _, files = parse_multipart(data, headers["Content-Type"])
            names = [n for field, n, _ in files if field == "images"]
            self.uploaded += names
            self.posted.append({"url": url, "names": names})
            return _Resp({"success": True})
        if path.startswith("/task/new/commit/"):
            self.posted.append({"url": url, "names": []})
            return _Resp({"uuid": self.uuid})
        return _Resp({"success": True})                                  # /task/cancel、/task/remove

    def get(self, url, params=None, timeout=None, stream=False):
        from urllib.parse import urlsplit
        path = urlsplit(url).path
        if path == "/info":
            return _Resp({"version": "2.4.0", "engineVersion": "3.8.4", "engine": "odx", "maxImages": None})
        if path == "/options":
            return _Resp([{"name": n, "type": "bool", "value": "false"} for n in (
                "orthophoto-resolution", "dsm", "tiles", "rolling-shutter", "rolling-shutter-readout", "force-gps",
                "max-concurrency", "gps-accuracy")])
        if path.endswith("/info"):
            code = self.codes.pop(0) if len(self.codes) > 1 else self.codes[0]
            n_img = len([n for n in self.uploaded if not n.endswith(".txt")])
            return _Resp({"status": {"code": code}, "processingTime": 1000, "progress": 100.0 if code == 40 else 50.0,
                          "imagesCount": n_img, "options": json.loads(self.init_fields["options"])})
        if path.endswith("/output"):
            return _Resp([])
        return _Resp(body=_odm_zip(self.evil), ctype="application/zip")


def test_odm_job_upload_wait_download_and_find_tiles(fleet, tmp_path):
    from survey.odm_client import NodeOdmClient
    res = run(tmp_path / "out", [sortie(fleet["h1"], pos=False)])
    job = pipeline.odm_job(res, node_url="http://127.0.0.1:3000", orthophoto_resolution_cm=2.5,
                           rolling_shutter=False)
    fake = FakeNodeOdm()
    sleeps, prog = [], []
    out = pipeline.run_odm_job(job, client=NodeOdmClient(job["node_url"], session=fake), poll_s=1.0,
                               sleep=sleeps.append, progress=lambda m, f: prog.append(m))
    assert fake.posted[0]["url"] == "http://127.0.0.1:3000/task/new/init"
    assert fake.posted[-1]["url"] == f"http://127.0.0.1:3000/task/new/commit/{fake.uuid}"
    assert fake.posted[1]["names"] == ["geo.txt"]                                # 附檔自己一批，再傳影像
    assert sorted(fake.uploaded) == sorted([p.name for p in res.images] + ["geo.txt"])
    assert json.loads(fake.init_fields["options"]) == [{"name": "orthophoto-resolution", "value": 2.5},
                                                       {"name": "dsm", "value": True}, {"name": "tiles", "value": True}]
    assert out["uuid"] == fake.uuid and sum(sleeps) == pytest.approx(1.0)
    assert out["ortho_tiles"] == res.out_dir / "odm" / f"result_{fake.uuid}" / "orthophoto_tiles"
    rec = json.loads((res.out_dir / "odm" / "submission.json").read_text(encoding="utf-8"))
    assert rec["uuid"] == fake.uuid and rec["qa_override"] is False and rec["n_images"] == len(res.images)
    assert rec["ortho_tiles"].endswith("orthophoto_tiles") and rec["phase"] == "downloaded"
    assert rec["images_check"]["ok"] is True and rec["force_gps"]["decision"] == "not_applicable_no_gcp"
    with pytest.raises(PipelineError, match="跳出目錄"):
        pipeline.run_odm_job(job, client=NodeOdmClient(job["node_url"], session=FakeNodeOdm(evil=True)),
                             poll_s=1.0, sleep=lambda s: None)
    assert not (res.out_dir / "odm" / "escape.txt").exists()
    assert json.loads((res.out_dir / "odm" / "submission.json").read_text(encoding="utf-8"))["phase"] == \
        "postprocess_failed"
    # 取消：等待中檢查旗標（只停本機等待；節點上的任務不受影響，紀錄留在 running 可接回）
    prog.clear()
    with pytest.raises(PipelineCanceled, match="接回任務"):
        pipeline.run_odm_job(job, client=NodeOdmClient(job["node_url"], session=FakeNodeOdm(codes=(20, 20, 40))),
                             poll_s=1.0, sleep=lambda s: None, should_cancel=lambda: any("節點" in m for m in prog),
                             progress=lambda m, f: prog.append(m))
    assert json.loads((res.out_dir / "odm" / "submission.json").read_text(encoding="utf-8"))["phase"] == "running"
    assert len(list((res.out_dir / "odm" / "history").glob("submission_*.json"))) == 2   # 前兩次的紀錄都留著
