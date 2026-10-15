"""survey.pipeline 端到端：合成航線 → .BIN ＋ Pi 會期 ＋ 相機模組 rover.ubx（TIM-TM2）＋ PPK .pos → 交付物。

真值與 test_survey_e2e 同一套（tests/test_survey_synthlog 的 SurveyTruth＋DataFlash 寫入器，survey.synth 的會期），
另外產生：
* rover.ubx：每個實體觸發邊緣（= 飛控 CAM 的時刻，換成 GPST）依 F9P「每 epoch 只報最後一個上升緣」寫 TIM-TM2，
  每 epoch 一則 NAV-PVT 與 RAWX（tests/test_survey_ppk_ubx 的獨立編碼器）。
* rover.pos：相機模組 F9P 天線（機身 FRD 偏移 F9P_ANT）在 10 Hz GPST 的橢球高軌跡（tests/test_survey_ppk_pos 的格式器）。
* session.json 的 gnss_lever_arm_m = 相機中心 − F9P 天線。

真值的相機中心：SurveyTruth.point_llh(曝光中心, GPS_POS + LEVER)，pyproj／scipy 計算，與 survey 的換算互相獨立。
"""
from __future__ import annotations

import io
import json
import math
import zipfile
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


def _fields(t_ns):
    wn, rem = divmod(int(t_ns), W_NS)
    ms, sub = divmod(rem, 1_000_000)
    return wn, ms, sub


def rover_ubx_bytes(edges_ns, *, epoch_ns=100_000_000, c0=100):
    """F9P 10 Hz：每個 epoch RAWX＋NAV-PVT，有新上升緣就加一則 TIM-TM2（只報該 epoch 最後一個）。"""
    edges = np.asarray(sorted(edges_ns), np.int64)
    out = []
    for n in range(int(edges[0]) // epoch_ns - 3, int(edges[-1]) // epoch_ns + 4):
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


def write_pos(path, tr, *, dlat_deg=0.0, q_of_t=None):
    """F9P 天線軌跡（10 Hz、橢球高）；q_of_t(t_fc) → Q 值（預設全 fix）。"""
    t = np.arange(tr.t0 - 2.0, tr.t_end - 0.1, 0.1)
    llh = np.asarray([tr.point_llh(x, tuple(F9P_ANT)) for x in t])
    q = np.ones(t.size, int) if q_of_t is None else np.asarray([q_of_t(x) for x in t], int)
    Path(path).write_text(pos_text(GPST0 + t, llh[:, 0] + dlat_deg, llh[:, 1], llh[:, 2] + N_SITE, q=q),
                          encoding="utf-8")
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
    (d / "sess" / "gnss" / "rover.ubx").write_bytes(
        rover_ubx_bytes([round((GPST0 + t * 1e-6) * 1e9) for t in cam_t]))   # TM2 看得到每一個實體邊緣
    pos = write_pos(d / "rover.pos", tr)
    return {"dir": d, "uav": uav, "truth": tr, "info": info, "pi": truth, "pos": pos,
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


def sortie(s, *, pos=True, pos_path=None):
    return SortieInput(s["uav"], s["bin"], s["sess"], pos_path=(pos_path or s["pos"]) if pos else None)


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
    res = run(tmp_path / "out", [sortie(h1)], ppk=PpkSettings(frame="e-GNSS[2025]"), geo_projection=GRS80,
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
    res = run(tmp_path / "out", [sortie(h1)], ppk=PpkSettings(frame="e-GNSS[2025]"), geo_projection=GRS80,
              sensor_ts_semantics="readout_start")
    dts = np.asarray([truth_err(h1, r)[2] for r in res.rows])
    assert np.all(np.abs(dts + 0.8e-3) < 1e-4)
    fc = run(tmp_path / "fc", [sortie(h1, pos=False)], sensor_ts_semantics="readout_start")
    t_fc = {r["seq"]: r["t_fc_us"] for r in fc.rows}
    assert all(r["t_fc_us"] == t_fc[r["seq"]] for r in res.rows)


def test_ppk_position_comes_from_the_pos_file(fleet, tmp_path):
    h1 = fleet["h1"]
    base = run(tmp_path / "a", [sortie(h1)], ppk=PpkSettings(frame="e-GNSS[2025]"), geo_projection=GRS80)
    shifted = write_pos(tmp_path / "shift.pos", h1["truth"], dlat_deg=1e-5)
    moved = run(tmp_path / "b", [sortie(h1, pos_path=shifted)], ppk=PpkSettings(frame="e-GNSS[2025]"),
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
    res = run(tmp_path / "out", [sortie(h1, pos_path=flo)], ppk=PpkSettings(frame="e-GNSS[2025]"),
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
    res = run(tmp_path / "a", [sortie(h1)], ppk=PpkSettings(frame="e-GNSS[2025]"))
    assert res.paths["geo_txt"] is None and res.images == [] and not (res.out_dir / "odm" / "geo.txt").exists()
    assert any("不全是 EPSG:4326" in e for e in res.errors)
    assert not res.passed and any(x.startswith("交付物：geo.txt 未寫") for x in res.qa["reasons"])
    # photos.csv 照寫，各列帶自己的框架（不改寫成 WGS84）
    from survey import deliverable
    assert {r["crs_h"] for r in deliverable.read_photos_csv(res.paths["photos_csv"])} == {"e-GNSS[2025]"}
    # 投影座標系（公尺）不能當 geo.txt 標頭：經緯度會被當成公尺
    with pytest.raises(PipelineError, match="不是經緯度"):
        run(tmp_path / "b", [sortie(h1)], ppk=PpkSettings(frame="e-GNSS[2025]"), geo_projection="EPSG:3826")
    with pytest.raises(PipelineError, match="PROJ"):
        run(tmp_path / "c", [sortie(h1)], ppk=PpkSettings(frame="e-GNSS[2025]"), geo_projection="e-GNSS[2025]")
    with pytest.raises(PipelineError, match="框架名稱必填"):
        PpkSettings(frame="  ")


def test_unknown_gnss_lever_arm_keeps_every_photo_on_fc(fleet, tmp_path):
    s = fleet["nolever"]
    res = run(tmp_path / "out", [sortie(s)], ppk=PpkSettings(frame="e-GNSS[2025]"), geo_projection=GRS80)
    assert {r["source"] for r in res.rows} == {"fc_interp"}
    assert all({"ppk_rejected", "lever_arm_unknown"} <= set(r["flags"].split(";")) for r in res.rows)
    assert any("gnss_lever_arm_m 是 null" in w for w in res.warnings)
    assert not res.passed and any("沒有任何一張照片採用 PPK" in x for x in res.qa["reasons"])


def test_ppk_input_validation(fleet, tmp_path):
    h1 = fleet["h1"]
    with pytest.raises(PipelineError, match="沒有 .pos，也沒有開啟 RTKLIB"):
        run(tmp_path / "a", [sortie(h1, pos=False)], ppk=PpkSettings(frame="e-GNSS[2025]"))
    with pytest.raises(PipelineError, match="RINEX 或 UBX"):
        PpkSettings(frame="e-GNSS[2025]", base=rtklib.BaseStation(25.1, 121.2, 50.0, "e-GNSS[2025]"),
                    run_rtklib=True)
    with pytest.raises(PipelineError, match="基站框架"):
        PpkSettings(frame="e-GNSS[2025]", base=rtklib.BaseStation(25.1, 121.2, 50.0, "TWD97"))


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
    kw = dict(ppk=PpkSettings(frame="e-GNSS[2025]"), geo_projection=GRS80, ground_h_m=GROUND_ELL)
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
    def __init__(self, *, codes=(20, 40), evil=False):
        self.posted, self.codes, self.evil = [], list(codes), evil

    def post(self, url, params=None, data=None, files=None, timeout=None):
        self.posted.append({"url": url, "data": data, "names": [f[1][0] for f in files]})
        return _Resp({"uuid": "task-1"})

    def get(self, url, params=None, timeout=None, stream=False):
        if url.endswith("/info"):
            return _Resp({"status": {"code": self.codes.pop(0) if len(self.codes) > 1 else self.codes[0]}})
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
    post = fake.posted[0]
    assert post["url"] == "http://127.0.0.1:3000/task/new"
    assert sorted(post["names"]) == sorted([p.name for p in res.images] + ["geo.txt"])
    assert json.loads(post["data"]["options"]) == [{"name": "orthophoto-resolution", "value": 2.5},
                                                   {"name": "dsm", "value": True}, {"name": "tiles", "value": True}]
    assert out["uuid"] == "task-1" and sum(sleeps) == pytest.approx(1.0)
    assert out["ortho_tiles"] == res.out_dir / "odm" / "result_task-1" / "orthophoto_tiles"
    rec = json.loads((res.out_dir / "odm" / "submission.json").read_text(encoding="utf-8"))
    assert rec["uuid"] == "task-1" and rec["qa_override"] is False and rec["n_images"] == len(res.images)
    assert rec["ortho_tiles"].endswith("orthophoto_tiles")
    with pytest.raises(PipelineError, match="跳出目錄"):
        pipeline.run_odm_job(job, client=NodeOdmClient(job["node_url"], session=FakeNodeOdm(evil=True)),
                             poll_s=1.0, sleep=lambda s: None)
    assert not (res.out_dir / "odm" / "escape.txt").exists()
    # 取消：等待中檢查旗標（NodeODM 上的任務不受影響）
    with pytest.raises(PipelineCanceled):
        pipeline.run_odm_job(job, client=NodeOdmClient(job["node_url"], session=FakeNodeOdm(codes=(20, 20, 40))),
                             poll_s=1.0, sleep=lambda s: None, should_cancel=lambda: len(prog) > 0)
