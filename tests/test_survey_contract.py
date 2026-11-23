"""跨套件契約：相機模組（payload.pi_capture）寫的東西，地面端（survey、survey.ppk、地圖）讀得對。

每條都用「寫出端的真實程式」產生資料、再用「讀取端的真實程式」讀回比對值，不手寫中間格式：

1. CaptureService（假後端）寫出含 ok／no_frame／write_error／dropped 四種狀態、結尾半行（斷電）
   的會期 → survey.pi_session.read_pi_session 與 survey.ppk.events.read_frames_jsonl 都讀得出同樣的影格。
2. F2 欄位只有一份：survey.deliverable、survey.ppk.outputs、core.recon.exposure_verifier 三邊相同。
3. survey.deliverable 寫的 photos.csv → 地圖的 photo_pose_rows_to_compact：零略過、航向取 yaw_deg。
   survey.ppk.outputs.ppk_photo_rows 的列可直接交給 survey.deliverable 寫出。
4. payload f9p_config 打開的訊息與時間基準，正是 survey.ppk.events 需要的（TIM-TM2＋NAV-PVT＋RAWX，GPS 時間基準）。

這次在這裡找到並修掉的不一致（最小修正）：
* survey.ppk.events.read_frames_jsonl 以嚴格 UTF-8 解碼：Pi 端 json.dumps(ensure_ascii=False)，斷電把最後一行切在
  多位元組字元中間時整檔 UnicodeDecodeError（survey.pi_session 用 replace 解碼則正常）⇒ 改成與 pi_session 相同。
* survey.ppk.outputs.ppk_photo_rows 以 "" 表示空的 strip，survey.deliverable 寫 CSV 時 int("") 丟 ValueError
  ⇒ deliverable 把 "" 視為空欄。
"""
from __future__ import annotations

import csv
import threading
import time

import numpy as np
import pytest

from payload.pi_capture import capture as PC
from payload.pi_capture import f9p_config as F
from payload.pi_capture import imu as PI_IMU
from payload.pi_capture import session as PS
from survey import deliverable, pi_session
from survey.ppk import events as ppk_events
from survey.ppk import outputs as ppk_outputs
from survey.ppk import ubx as ppk_ubx
from tests.test_pi_capture import (BOOT_OFF, MS, SEC, T0, FakeCamera, FakeClock, FakeEdges, FakeFrame,
                                   FakeHealth, make_meta)

TORN_TAIL = b'{"seq":6,"edge_ns":10000000\xe5\x9c'     # 斷電：切在 3 位元組 UTF-8 字元中間


def _wait_idle(svc, timeout_s=5.0):
    t_end = time.monotonic() + timeout_s
    while svc.stats()["inflight"] and time.monotonic() < t_end:
        time.sleep(0.005)
    assert svc.stats()["inflight"] == 0


def _capture_all_statuses(root, *, power_cut: bool):
    """真的 CaptureService（非同步寫入、佇列 1）跑出 ok／dropped／write_error／no_frame／ok。"""
    clock = FakeClock()
    st = PS.LocalStorage(root / "sess")
    svc = PC.CaptureService(st, make_meta(), FakeCamera(buffer_count=3), FakeEdges(), clock,
                            health=FakeHealth(), sync_writes=False, max_inflight_writes=1)
    svc.start()
    gate = threading.Event()
    svc.handle_edge(T0)                                            # seq 1：ok（寫入卡在編碼）
    svc.handle_frame(FakeFrame(T0 + 5 * MS, block=gate))
    svc.handle_edge(T0 + SEC)                                      # seq 2：佇列滿 → dropped
    svc.handle_frame(FakeFrame(T0 + SEC + 5 * MS, exposure_us=900))
    gate.set()
    _wait_idle(svc)
    svc.handle_edge(T0 + 2 * SEC)                                  # seq 3：編碼失敗 → write_error
    svc.handle_frame(FakeFrame(T0 + 2 * SEC + 5 * MS, fail_encode=True))
    _wait_idle(svc)
    svc.handle_edge(T0 + 3 * SEC)                                  # seq 4：沒有影格 → no_frame
    clock.set(T0 + 3 * SEC + 600 * MS)
    svc.tick()
    svc.handle_edge(T0 + 4 * SEC)                                  # seq 5：ok
    svc.handle_frame(FakeFrame(T0 + 4 * SEC + 7 * MS))
    _wait_idle(svc)
    if power_cut:
        svc.abort()
    else:
        svc.stop()
    with open(st.path("frames.jsonl"), "ab") as fh:              # 斷電寫到一半的最後一行
        fh.write(TORN_TAIL)
    return st


EXPECTED = {1: ("ok", T0 + 5 * MS), 2: ("dropped", T0 + SEC + 5 * MS), 3: ("write_error", T0 + 2 * SEC + 5 * MS),
            4: ("no_frame", None), 5: ("ok", T0 + 4 * SEC + 7 * MS)}


@pytest.mark.parametrize("power_cut", [False, True], ids=["clean_stop", "power_cut"])
def test_capture_session_is_read_by_pi_session(tmp_path, power_cut):
    st = _capture_all_statuses(tmp_path, power_cut=power_cut)
    s = pi_session.read_pi_session(st.root, verify_sha256=True)
    assert [f.seq for f in s.frames] == [1, 2, 3, 4, 5]
    assert {f.seq: f.status for f in s.frames} == {k: v[0] for k, v in EXPECTED.items()}
    # 影格的 SensorTimestamp 是 BOOTTIME：= 單調時鐘 + 休眠偏移；no_frame 沒有
    assert {f.seq: f.sensor_ts_ns for f in s.frames} == {k: (None if v[1] is None else v[1] + BOOT_OFF)
                                                         for k, v in EXPECTED.items()}
    assert [f.edge_ns for f in s.frames] == [T0, T0 + SEC, T0 + 2 * SEC, T0 + 3 * SEC, T0 + 4 * SEC]
    assert s.clock_offset_ns == BOOT_OFF and s.clock_offset_spread_ns == 0
    assert s.edge_boottime_ns().tolist() == [e + BOOT_OFF for e in (T0, T0 + SEC, T0 + 2 * SEC, T0 + 3 * SEC,
                                                                    T0 + 4 * SEC)]
    assert [f.file for f in s.frames] == ["images/000001.jpg", None, None, None, "images/000005.jpg"]
    assert s.missing_files == [] and s.sha256_mismatch == [] and s.seq_gaps == []
    assert "frames_truncated_tail" in s.flags
    # seq 1 的寫入卡著時 seq 2 已判 dropped，但紀錄走同一個寫入佇列 ⇒ 檔案仍依 seq 排序
    assert "frames_out_of_order" not in s.flags
    if power_cut:
        assert s.summary is None and "summary_missing" in s.flags
    else:
        assert s.summary["edges"] == 5 and s.summary["ok"] == 2 and "summary_mismatch" not in s.flags
    assert s.uav == "HERO2" and s.unit_model == "RPiHQ6-HERO2" and s.readout_ms is None
    assert s.lever_arm_m.tolist() == [0.12, -0.03, 0.08]


def test_capture_session_is_read_by_ppk_frames_reader(tmp_path):
    st = _capture_all_statuses(tmp_path, power_cut=True)
    rows, bad = ppk_events.read_frames_jsonl(st.path("frames.jsonl"))
    assert bad == [6]                                               # 只有最後那半行
    by_seq = {r["seq"]: r for r in rows}
    assert sorted(by_seq) == [1, 2, 3, 4, 5]
    s = pi_session.read_pi_session(st.root)
    for f in s.frames:                                              # 兩個讀取器讀到同一批影格
        r = by_seq[f.seq]
        got = (r["edge_ns"], r["status"], r["file"], r["sensor_ts_ns"])
        assert got == (f.edge_ns, f.status, f.file, f.sensor_ts_ns)
    # 中間行壞掉仍然拒絕（不是斷電）
    raw = st.path("frames.jsonl").read_bytes().split(b"\n")
    raw.insert(1, b'{"seq": broken')
    st.path("frames.jsonl").write_bytes(b"\n".join(raw))
    with pytest.raises(ValueError, match="第 2 行"):
        ppk_events.read_frames_jsonl(st.path("frames.jsonl"))


def test_session_json_has_every_field_the_ground_side_reads():
    meta = make_meta(gnss_lever_arm_m=[0.1, 0.0, -0.2]).to_json()
    assert meta["format"] == pi_session.SESSION_FORMAT == PS.FORMAT
    assert set(pi_session._REQUIRED_META) <= set(meta)
    assert set(pi_session._REQUIRED_CAMERA) <= set(meta["camera"])
    assert meta["gnss_lever_arm_m"] == [0.1, 0.0, -0.2]                # survey.pipeline 的 PPK 槓桿臂來源
    assert pi_session.FRAME_STATUSES == PS.FRAME_STATUSES
    # 選配檔的路徑：Pi 端寫的位置就是地面端找的位置
    assert F.ROVER_UBX == "gnss/rover.ubx" and PI_IMU.IMU_CSV == "imu.csv"


def test_optional_files_are_found_where_pi_writes_them(tmp_path):
    st = _capture_all_statuses(tmp_path, power_cut=False)
    (st.root / F.ROVER_UBX).parent.mkdir(parents=True, exist_ok=True)
    (st.root / F.ROVER_UBX).write_bytes(b"")
    (st.root / PI_IMU.IMU_CSV).write_text(PI_IMU.IMU_HEADER + "\n", encoding="utf-8")
    s = pi_session.read_pi_session(st.root)
    assert s.gnss_ubx_path == st.root / F.ROVER_UBX and s.imu_path == st.root / PI_IMU.IMU_CSV


# ── F2 ────────────────────────────────────────────────────────────────────────

def test_f2_columns_are_one_definition():
    from core.recon.exposure_verifier import F2_COLUMNS as VERIFIER_F2
    assert deliverable.F2_COLUMNS == ppk_outputs.PHOTOS_CSV_FIELDS == tuple(VERIFIER_F2)
    assert len(deliverable.F2_COLUMNS) == 24


def _row(seq, *, yaw, cog, source="fc_interp", crs_h="EPSG:4326", strip=1, h_ell=150.0):
    r = {c: None for c in deliverable.F2_COLUMNS}
    r.update(image_id=f"HERO1_S1_{seq:06d}", file=f"images/{seq:06d}.jpg", uav="HERO1", strip=strip, seq=seq,
             t_fc_us=400_000_000 + seq * 900_000, t_gpst_s=1_444_000_400.0 + seq * 0.9,
             lat_deg=25.1 + seq * 1e-4, lon_deg=121.2 + seq * 2e-4, h_ell_m=h_ell, yaw_deg=yaw, pitch_deg=2.0,
             roll_deg=-1.0, cog_deg=cog, speed_n_mps=0.0, speed_e_mps=21.5, speed_d_mps=0.0, sigma_h_m=0.5,
             sigma_v_m=0.8, source=source, crs_h=crs_h, crs_v="ellipsoidal", flags="")
    return r


def test_photos_csv_reaches_the_map_with_zero_skips_and_yaw_heading(tmp_path):
    from ui.widgets.cesium_map_widget import photo_pose_rows_to_compact
    rows = [_row(1, yaw=95.25, cog=90.0),
            _row(2, yaw=274.75, cog=270.0, source="ppk", crs_h="e-GNSS[2025]", strip=None),
            _row(3, yaw=0.5, cog=359.0, h_ell=None)]
    p = deliverable.write_photos_csv(tmp_path / "photos.csv", rows)
    typed = deliverable.read_photos_csv(p)
    with open(p, encoding="utf-8-sig", newline="") as fh:
        as_text = list(csv.DictReader(fh))                         # DualMapWidget.load_photo_poses_file 的讀法
    for got in (typed, as_text):
        compact, stats = photo_pose_rows_to_compact(got)
        assert stats["skipped"] == 0 and stats["drawn"] == 3 and stats["no_heading"] == 0
        assert stats["unknown_source"] == {} and stats["no_height"] == 1
        by_id = {c[5]: c for c in compact}
        for r in rows:
            lat, lon, h, heading, src, _ = by_id[r["image_id"]]
            assert heading == pytest.approx(r["yaw_deg"], abs=1e-3)      # 航向 = 機頭（yaw），不是航跡（cog）
            assert abs(heading - r["cog_deg"]) > 0.4
            assert (lat, lon) == (pytest.approx(r["lat_deg"], abs=1e-9), pytest.approx(r["lon_deg"], abs=1e-9))
            assert (h is None) == (r["h_ell_m"] is None) and src == r["source"]


def test_ppk_rows_can_be_written_by_the_deliverable(tmp_path):
    """outputs.ppk_photo_rows 的列（空 strip 是 ""）直接交給 deliverable 寫 F2 並讀回。"""
    from survey.ppk.interp import interpolate_at
    from survey.ppk.pos import parse_pos
    from tests.test_survey_ppk_pos import pos_text
    t0 = 2389 * 604800 + 180000.0
    t = t0 + 0.1 * np.arange(50)
    tr = parse_pos(pos_text(t, np.full(50, 25.1), 121.2 + 1e-5 * np.arange(50), np.full(50, 150.0)))
    evs = [ppk_events.PairedEvent(seq=k, edge_ns=k * SEC, tm2_count=100 + k, t_edge_gpst_s=t0 + k,
                                  t_exposure_gpst_s=t0 + k + 0.05, residual_s=0.0, acc_est_ns=20, flags=(),
                                  file=f"images/{k:06d}.jpg", status="ok") for k in (1, 2)]
    ip = interpolate_at(tr, [e.t_exposure_gpst_s for e in evs], lever_arm_frd_m=(0.1, 0.0, 0.0),
                        attitude_deg=[[90.0, 0.0, 0.0]] * 2)
    rows = ppk_outputs.ppk_photo_rows(evs, ip, crs_h="e-GNSS[2025]", uav="HERO1", attitude_deg=[[90.0, 0.0, 0.0]] * 2,
                                      strip_of_seq={1: 3}, image_id_of_seq={1: "HERO1_S_000001", 2: "HERO1_S_000002"})
    assert rows[1]["strip"] == ""                                   # 輸出端的空值寫法
    p = deliverable.write_photos_csv(tmp_path / "photos.csv", rows)
    back = {r["seq"]: r for r in deliverable.read_photos_csv(p)}
    assert back[1]["strip"] == 3 and back[2]["strip"] is None
    assert back[1]["source"] == "ppk" and back[1]["crs_h"] == "e-GNSS[2025]" and back[1]["t_fc_us"] is None
    assert back[2]["lat_deg"] == pytest.approx(rows[1]["lat_deg"], abs=1e-9)
    # 曝光驗證器也讀得懂（空 strip → 不進前向重疊）
    from core.recon.exposure_verifier import read_photos_csv as verifier_read
    poses, meta = verifier_read(p, height_field="h_ell_m")
    assert [ph.strip for ph in sorted(poses, key=lambda x: x.seq)] == ["3", None]
    assert meta["source"] == {"ppk": 2}


# ── F9P 設定 ↔ PPK 事件 ───────────────────────────────────────────────────────

_REQUIRED_BY_EVENTS = {          # survey.ppk.events 需要的訊息（名稱 → survey.ppk.ubx 的 class/id）
    "UBX_TIM_TM2": ppk_ubx.UBX_TIM_TM2,      # 曝光邊緣
    "UBX_NAV_PVT": ppk_ubx.UBX_NAV_PVT,      # 時間基準數值鎖 2
    "UBX_RXM_RAWX": ppk_ubx.UBX_RXM_RAWX,    # 閏秒數值鎖 1 ＋ RTKLIB 觀測量
}


def _valset_items(frame: bytes) -> dict:
    payload = frame[6:-2]
    out, i = {}, 4
    while i < len(payload):
        kid = int.from_bytes(payload[i:i + 4], "little")
        n = F.key_size_bytes(kid)
        out[kid] = int.from_bytes(payload[i + 4:i + 4 + n], "little")
        i += 4 + n
    return out


@pytest.mark.parametrize("port,baud", [("USB", None), ("UART1", 460800), ("UART2", 921600)])
def test_f9p_config_enables_what_ppk_events_needs(port, baud):
    msgs = F.build_rover_config(port, baudrate=baud)
    kv = {}
    for m in msgs:
        kv.update(_valset_items(m.frame))
    for name, (cls, mid) in _REQUIRED_BY_EVENTS.items():
        key = F.KEYS[f"CFG-MSGOUT-{name}_{port}"]
        assert kv[key.key_id] == 1, f"{name} 在 {port} 沒有每 epoch 輸出"
        assert (cls, mid) in ((0x0D, 0x03), (0x01, 0x07), (0x02, 0x15))
    assert kv[F.KEYS["CFG-MSGOUT-UBX_RXM_SFRBX_" + port].key_id] == 1    # RTKLIB 需要導航電文
    # TIM-TM2 的時間基準 = GPS（timeBase=1）⇒ events 以 gnss_time_base_system='GPS'（預設）直接當 GPST
    assert kv[F.KEYS["CFG-TP-TIMEGRID_TP1"].key_id] == F.TIMEGRID_GPS == 1
    # 量測週期 100 ms ⇒ NAV-PVT 每 epoch 一則，events 的 PVT 容差 = 2.5 × 0.1 s
    assert kv[F.KEYS["CFG-RATE-MEAS"].key_id] == 100 and kv[F.KEYS["CFG-RATE-NAV"].key_id] == 1


def test_timegrid_gps_time_base_is_read_as_gpst_by_events():
    """F9P 設成 TIMEGRID=GPS 時 TIM-TM2 的 timeBase=1；events 不加閏秒、直接就是 GPST。UTC 基準（出廠）才加 18 s。"""
    from tests.test_survey_ppk_ubx import tm2_payload
    wn, tow_ms, sub_ns = 2389, 180_000_123, 456_789
    gps = ppk_ubx.decode_tim_tm2(tm2_payload(flags=0x01 | 0x40 | (F.TIMEGRID_GPS << 3) | 0x80, count=7,
                                             wn_r=wn, tow_ms_r=tow_ms, sub_r=sub_ns))
    utc = ppk_ubx.decode_tim_tm2(tm2_payload(flags=0x01 | 0x40 | 0x20 | (2 << 3) | 0x80, count=7,
                                             wn_r=wn, tow_ms_r=tow_ms, sub_r=sub_ns))
    t = wn * 604800 + tow_ms / 1e3 + sub_ns / 1e9
    assert gps.time_base == 1 and ppk_events.tm2_time_gpst(gps) == pytest.approx(t, abs=1e-6)
    assert ppk_events.tm2_time_gpst(utc) == pytest.approx(t + 18, abs=1e-6)


def test_ubx_framing_is_the_same_on_both_sides():
    rng = np.random.default_rng(3)
    for n in (0, 1, 7, 92, 300):
        body = rng.integers(0, 256, n, dtype=np.uint8).tobytes()
        assert F.ubx_checksum(body) == ppk_ubx.ubx_checksum(body)
    # Pi 端組的 VALSET 框，地面端切框器全部認得（0 個檢查碼錯誤）
    frames = b"".join(m.frame for m in F.build_rover_config("UART1", baudrate=460800))
    log = ppk_ubx.parse_ubx(frames)
    assert log.stats.frames == 2 and log.stats.checksum_errors == 0 and log.stats.garbage_bytes == 0
    # 地面端組的 TIM-TM2，Pi 端的 UbxReader 也切得出來
    from tests.test_survey_ppk_ubx import tm2_payload
    fr = ppk_ubx.build_frame(0x0D, 0x03, tm2_payload(count=5))
    got = F.UbxReader().feed(b"$GPGGA,noise*00\r\n" + fr)
    assert [(c, i) for c, i, _ in got] == [(0x0D, 0x03)]
    assert ppk_ubx.decode_tim_tm2(got[0][2]).count == 5
