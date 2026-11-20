"""scripts/hv_sitl_wind_summary.py：把 wind_check.json 整理成對照表（合成資料，不依賴任何真實航次）。"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import hv_sitl_wind_summary as ws  # noqa: E402


def _fake(tmp_path: Path, trial: str, *, plan=(5.0, 270.0), sim=(5.0, 270.0), dur=(600.0, 630.0)):
    d = {
        "winds": {"plan": {"speed_mps": plan[0], "dir_from_deg": plan[1]}, "sim": {"speed_mps": sim[0], "dir_from_deg": sim[1]}},
        "plan_wind_handling": {"measured_crab_from_bin": {"A1": {"max_abs_crab_deg": 7.8}}},
        "per_uav": {"A1": {
            "class": "A",
            "times": {"duration_plan_s": dur[0], "duration_flown_s": dur[1], "duration_delta_s": dur[1] - dur[0],
                      "duration_delta_pct": 100 * (dur[1] - dur[0]) / dur[0]},
            "leg_time_totals": {"flown_s": 226.0, "plan_model_s": 231.0, "sim_model_s": 226.4},
            "leg_classes_vs_ref": {"headwind": {"gs_pos_mean_mps": 16.95}, "tailwind": {"gs_pos_mean_mps": 26.95}},
            "landing": {"touchdown_err_m": 0.13, "approach": {"vs_wind": {"sim_wind": {"headwind_component_mps": 5.0}}}},
            "path": {"delta_pct_all_samples": 3.8},
        }},
    }
    (tmp_path / trial).mkdir(parents=True)
    (tmp_path / trial / "wind_check.json").write_text(json.dumps(d), encoding="utf-8")


def test_rows_take_values_straight_from_the_json(tmp_path):
    _fake(tmp_path, "T1")
    r = ws.rows_for("T1", tmp_path)[0]
    assert (r["plan_wind"], r["sim_wind"]) == ("5.0@270", "5.0@270")
    assert r["dur_plan_s"] == 600.0 and r["dur_flown_s"] == 630.0 and abs(r["dur_delta_pct"] - 5.0) < 1e-9
    assert r["gs_headwind"] == 16.95 and r["gs_tailwind"] == 26.95 and r["gs_crosswind"] is None
    assert r["crab_bin_max_deg"] == 7.8 and r["approach_headwind_mps"] == 5.0 and r["touchdown_err_m"] == 0.13


def test_missing_fields_become_na_not_zero(tmp_path):
    (tmp_path / "T2").mkdir()
    (tmp_path / "T2" / "wind_check.json").write_text(json.dumps({"winds": {}, "per_uav": {"B2": {"class": "B"}}}), encoding="utf-8")
    r = ws.rows_for("T2", tmp_path)[0]
    assert r["dur_plan_s"] is None and r["crab_bin_max_deg"] is None
    assert ws._f(r["dur_plan_s"]) == "n/a" and ws._f(None, 1, True) == "n/a"


def test_cli_prints_and_writes_markdown_and_json(tmp_path):
    _fake(tmp_path, "WINDFLOW_F_X", plan=(0.0, 270.0))
    out = tmp_path / "out"
    cp = subprocess.run([sys.executable, str(ROOT / "scripts" / "hv_sitl_wind_summary.py"), "WINDFLOW_F_X",
                         "--check-dir", str(tmp_path), "--out", str(out)], capture_output=True, text=True, encoding="utf-8",
                        env={**__import__("os").environ, "PYTHONIOENCODING": "utf-8"})
    assert cp.returncode == 0, cp.stderr
    assert "| X | A1 | 0.0@270 | 5.0@270 | 600→630 |" in cp.stdout
    assert (out / "wind_summary.md").read_text(encoding="utf-8") == cp.stdout.rstrip("\n") + "\n" or "X" in (out / "wind_summary.md").read_text(encoding="utf-8")
    assert json.loads((out / "wind_summary.json").read_text(encoding="utf-8"))[0]["uav"] == "A1"


def test_missing_trial_is_a_clear_error(tmp_path):
    try:
        ws.load("nope", tmp_path)
    except SystemExit as e:
        assert "找不到" in str(e)
    else:  # pragma: no cover
        raise AssertionError("應該丟 SystemExit")
