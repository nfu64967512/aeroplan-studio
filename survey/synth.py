"""由飛控拍照事件合成樹莓派會期（F1）—— SITL S1「用 SITL 日誌合成 Pi 會期」與測試用。

真實相機模組還沒上機之前，ingest 的整條鏈（配對、擬合、位姿、QA、geo.txt、EXIF）只能用合成會期
驗證。這裡依明確給的「真值」產生：

* 時鐘：t_fc = a·t_boot + b（漂移 a、偏移 b），邊緣再加高斯抖動；MONOTONIC = BOOTTIME − 固定偏移。
* 相機連續出幀（週期 T、隨機相位），每個邊緣保留第一張 SensorTimestamp ≥ 邊緣的影格
  ⇒ 挑幀延遲落在 [0, T)，與相機模組的設計相同。
* 可指定：漏掉的邊緣（Pi 沒看到）、多出的邊緣（雜訊）、有邊緣但沒存到影像（no_frame）。

回傳的真值（每個寫出的影格）讓測試能逐張比對配對與曝光時刻。
"""

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from survey.pi_session import SESSION_FORMAT


def tiny_jpeg(width: int, height: int, seed: int = 0) -> bytes:
    from PIL import Image
    rng = np.random.default_rng(seed)
    arr = rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)
    buf = io.BytesIO()
    Image.fromarray(arr, "RGB").save(buf, "JPEG", quality=90)
    return buf.getvalue()


def synth_session_from_fc(out_dir, t_fc_us: Sequence[int], *, uav: str = "uav1", session_id: str = "S1",
                          unit_model: str = "RPiHQ6-SYN1", camera: Optional[dict] = None,
                          lever_arm_m=(0.0, 0.0, 0.0), clock_a: float = 1.0, clock_b_s: float = 0.0,
                          boottime_minus_monotonic_ns: int = 0, jitter_s: float = 0.0,
                          frame_period_s: float = 0.1, exposure_us: int = 1000,
                          sensor_ts_semantics: str = "exposure_start",
                          drop_index: Sequence[int] = (), extra_edges_fc_s: Sequence[float] = (),
                          no_frame_index: Sequence[int] = (), seed: int = 0, write_summary: bool = True,
                          image_bytes: Optional[bytes] = None) -> dict:
    """寫出會期目錄；``t_fc_us`` 是飛控事件（CAM.TimeUS）。回傳真值 dict。"""
    rng = np.random.default_rng(seed)
    out = Path(out_dir)
    (out / "images").mkdir(parents=True, exist_ok=True)
    camera = dict(camera or {"sensor": "imx477", "lens_mm": 6.0, "pixel_um": 1.55, "width": 64, "height": 48,
                             "readout_ms": 86.9, "shutter": "rolling", "kernel": "6.6.y", "dtoverlay": "imx477"})
    if image_bytes is None:
        image_bytes = tiny_jpeg(int(camera["width"]), int(camera["height"]), seed)
    sha = hashlib.sha256(image_bytes).hexdigest()

    # 每個邊緣在 BOOTTIME 的真值：t_boot = (t_fc − b) / a
    edges = []                           # (t_boot_s 真值, 來源 fc 索引或 None)
    drop = set(int(i) for i in drop_index)
    for i, t in enumerate(np.asarray(t_fc_us, np.float64) * 1e-6):
        if i not in drop:
            edges.append(((t - clock_b_s) / clock_a, i))
    for t in extra_edges_fc_s:
        edges.append(((t - clock_b_s) / clock_a, None))
    edges.sort(key=lambda e: e[0])
    phase = float(rng.uniform(0, frame_period_s))
    readout_s = (camera.get("readout_ms") or 0.0) * 1e-3
    no_frame = set(int(i) for i in no_frame_index)
    frames, truth = [], []
    for seq, (tb, src) in enumerate(edges, start=1):
        tb_meas = tb + float(rng.normal(0.0, jitter_s)) if jitter_s > 0 else tb
        edge_boot_ns = int(round(tb_meas * 1e9))
        edge_mono_ns = edge_boot_ns - int(boottime_minus_monotonic_ns)
        k = np.ceil((tb - phase) / frame_period_s)
        ts = phase + k * frame_period_s                        # 第一張 SensorTimestamp ≥ 真實邊緣
        ts_ns = int(round(ts * 1e9))
        e_s = exposure_us * 1e-6
        center = ts + (e_s / 2 if sensor_ts_semantics == "exposure_start" else -e_s / 2) + readout_s / 2
        rec = {"seq": seq, "edge_ns": edge_mono_ns, "sensor_ts_ns": ts_ns, "exposure_us": exposure_us,
               "frame_duration_us": int(round(frame_period_s * 1e6)), "analogue_gain": 1.0,
               "file": None, "bytes": None, "sha256": None, "encode_ms": 120.0, "temp_c": 55.0,
               "throttled": 0, "status": "ok"}
        if src is not None and src in no_frame:
            rec.update(status="no_frame", sensor_ts_ns=None)
        else:
            rel = f"images/{seq:06d}.jpg"
            (out / rel).write_bytes(image_bytes)
            rec.update(file=rel, bytes=len(image_bytes), sha256=sha)
        frames.append(rec)
        truth.append({"seq": seq, "fc_index": src, "edge_boot_s": tb, "sensor_ts_s": ts,
                      "center_boot_s": center, "center_fc_s": clock_a * center + clock_b_s,
                      "status": rec["status"]})

    meta = {"format": SESSION_FORMAT, "session_id": session_id, "uav": uav, "unit_model": unit_model,
            "camera": camera, "lever_arm_m": [float(x) for x in lever_arm_m], "gnss_lever_arm_m": None,
            "software": "survey.synth", "started_utc_approx": "2026-10-14T00:00:00Z"}
    (out / "session.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    with (out / "frames.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        for rec in frames:
            fh.write(json.dumps(rec) + "\n")
    t_lo = (edges[0][0] if edges else 0.0) - 5.0
    t_hi = (edges[-1][0] if edges else 0.0) + 5.0
    with (out / "clocks.jsonl").open("w", encoding="utf-8", newline="\n") as fh:
        for tb in np.append(np.arange(t_lo, t_hi, 10.0), t_hi):
            bt = int(round(tb * 1e9))
            fh.write(json.dumps({"monotonic_ns": bt - int(boottime_minus_monotonic_ns), "boottime_ns": bt,
                                 "realtime_ns": 1_760_000_000_000_000_000 + bt}) + "\n")
    if write_summary:
        (out / "summary.json").write_text(json.dumps({
            "edges": len(frames), "ok": sum(1 for f in frames if f["status"] == "ok"),
            "no_frame": sum(1 for f in frames if f["status"] == "no_frame"), "write_error": 0,
            "stopped_utc_approx": "2026-10-14T01:00:00Z"}), encoding="utf-8")
    return {"frames": truth, "phase_s": phase, "image_sha256": sha}
