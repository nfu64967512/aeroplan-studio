"""NodeODX 回報「完成」但成果是空的：必須當成失敗，不能記成已下載。

2026-10-20 本機實測：NodeODX 找不到 ODX 的 winrun.bat（執行環境設了 NoDefaultCurrentDirectoryInExePath）時，
任務 65 ms 就 status 40、progress 100，all.zip 裡只有 task_output.txt。
"""
from __future__ import annotations

import pytest

from survey import pipeline
from survey.pipeline import PipelineError
from tests.test_survey_odm_fake_node import NOOP, assert_no_token, client, job_for, make_node, make_result, record


def test_completed_task_without_an_orthophoto_is_a_failure_not_a_download(tmp_path):
    n = make_node(tmp_path, "node", empty_result=True)
    try:
        res = make_result(tmp_path)
        with pytest.raises(PipelineError, match="沒有正射") as ei:
            pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        assert "winrun.bat" in str(ei.value)
        rec = record(res)
        assert rec["phase"] == "failed" and rec["failure"]["kind"] == "empty_result"
        assert "也沒有品質報告" in rec["failure"]["message"]
        assert any("winrun.bat" in line for line in rec["failure"]["console_tail"])
        st = pipeline.odm_submission_status(rec)
        assert st["can_reattach"] is False
        assert_no_token(res.out_dir)
    finally:
        n.stop()


def test_a_normal_result_is_still_a_download(tmp_path):
    n = make_node(tmp_path, "node")
    try:
        res = make_result(tmp_path)
        out = pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        assert out["phase"] == "downloaded" and out["ortho_tiles"] is not None
        assert record(res)["phase"] == "downloaded"
    finally:
        n.stop()
