"""survey.pi_session：F1 會期的讀取與容錯（斷電、截斷、時鐘、雜湊）。"""

import json

import numpy as np
import pytest

from survey import pi_session, synth
from survey.pi_session import read_pi_session


@pytest.fixture
def sess_dir(tmp_path):
    t_fc_us = [10_000_000 + k * 900_000 for k in range(8)]
    truth = synth.synth_session_from_fc(tmp_path / "s", t_fc_us, clock_a=1.0, clock_b_s=-5.0,
                                        boottime_minus_monotonic_ns=2_500_000_000, no_frame_index=[3], seed=2)
    return tmp_path / "s", truth


def test_reads_session_and_converts_edges_to_boottime(sess_dir):
    d, truth = sess_dir
    s = read_pi_session(d, verify_sha256=True)
    assert s.uav == "uav1" and s.unit_model == "RPiHQ6-SYN1" and s.readout_ms == 86.9
    assert s.clock_offset_ns == 2_500_000_000 and s.clock_offset_spread_ns == 0
    assert [f.seq for f in s.frames] == list(range(1, 9))
    assert s.frames[3].status == "no_frame" and s.frames[3].file is None
    # MONOTONIC + (BOOTTIME − MONOTONIC) = 真值的 BOOTTIME（t_fc − b）
    np.testing.assert_allclose(s.edge_boottime_ns() * 1e-9, [f["edge_boot_s"] for f in truth["frames"]], atol=1e-9)
    assert s.flags == [] and s.sha256_mismatch == [] and s.missing_files == [] and s.seq_gaps == []
    assert s.lever_arm_m.tolist() == [0.0, 0.0, 0.0]
    assert s.gnss_ubx_path is None and s.imu_path is None


def test_truncated_tail_is_tolerated_and_flagged(sess_dir):
    d, _ = sess_dir
    p = d / "frames.jsonl"
    p.write_bytes(p.read_bytes() + b'{"seq": 9, "edge_ns": 123, "file": "\xe5\xbd')   # 斷電：連 UTF-8 字元都切一半
    (d / "summary.json").unlink()
    s = read_pi_session(d)
    assert len(s.frames) == 8
    assert "frames_truncated_tail" in s.flags and "summary_missing" in s.flags


def test_corrupt_line_in_the_middle_is_rejected(sess_dir):
    d, _ = sess_dir
    lines = (d / "frames.jsonl").read_text(encoding="utf-8").splitlines()
    lines[2] = lines[2][:20]
    (d / "frames.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="第 3 列"):
        read_pi_session(d)


def test_summary_mismatch_seq_gap_and_duplicate(sess_dir):
    d, _ = sess_dir
    lines = (d / "frames.jsonl").read_text(encoding="utf-8").splitlines()
    (d / "frames.jsonl").write_text("\n".join(lines[:4] + lines[5:]) + "\n", encoding="utf-8")   # 拿掉 seq 5
    s = read_pi_session(d)
    assert s.seq_gaps == [5] and "seq_gap" in s.flags and "summary_mismatch" in s.flags
    (d / "frames.jsonl").write_text("\n".join(lines + [lines[0]]) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="重複的 seq"):
        read_pi_session(d)


def test_sha256_and_missing_file_detection(sess_dir):
    d, _ = sess_dir
    (d / "images" / "000002.jpg").write_bytes(b"\xff\xd8 tampered")
    (d / "images" / "000005.jpg").unlink()
    s = read_pi_session(d, verify_sha256=True)
    assert s.sha256_mismatch == [2] and s.missing_files == [5]
    assert "sha256_mismatch" in s.flags and "missing_files" in s.flags
    s2 = read_pi_session(d, verify_sha256=False)
    assert s2.sha256_mismatch == [] and s2.missing_files == [5]


def test_clock_offset_unstable_and_missing(sess_dir):
    d, _ = sess_dir
    lines = (d / "clocks.jsonl").read_text(encoding="utf-8").splitlines()
    last = json.loads(lines[-1])
    last["boottime_ns"] += 1_500_000                    # 會期中休眠 1.5 ms
    lines[-1] = json.dumps(last)
    (d / "clocks.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    s = read_pi_session(d)
    assert "clock_offset_unstable" in s.flags and s.clock_offset_spread_ns == 1_500_000
    (d / "clocks.jsonl").unlink()
    with pytest.raises(ValueError, match="clocks.jsonl"):
        read_pi_session(d)
    s3 = read_pi_session(d, allow_missing_clocks=True)
    assert s3.clock_offset_ns == 0 and "clocks_missing_assumed_zero" in s3.flags


def test_bad_session_json_is_rejected(sess_dir):
    d, _ = sess_dir
    meta = json.loads((d / "session.json").read_text(encoding="utf-8"))
    bad = dict(meta, format="aeroplan-pi-session/2")
    (d / "session.json").write_text(json.dumps(bad), encoding="utf-8")
    with pytest.raises(ValueError, match="只接受"):
        read_pi_session(d)
    bad = dict(meta)
    bad["camera"] = {k: v for k, v in meta["camera"].items() if k != "readout_ms"}
    del bad["lever_arm_m"]
    (d / "session.json").write_text(json.dumps(bad), encoding="utf-8")
    with pytest.raises(ValueError, match=r"lever_arm_m.*camera.readout_ms"):
        read_pi_session(d)
    with pytest.raises(FileNotFoundError):
        read_pi_session(d / "images")


def test_invalid_status_is_rejected(sess_dir):
    d, _ = sess_dir
    lines = (d / "frames.jsonl").read_text(encoding="utf-8").splitlines()
    rec = json.loads(lines[0])
    rec["status"] = "maybe"
    lines[0] = json.dumps(rec)
    (d / "frames.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="status"):
        read_pi_session(d)
    assert pi_session.FRAME_STATUSES == ("ok", "no_frame", "write_error", "dropped")
