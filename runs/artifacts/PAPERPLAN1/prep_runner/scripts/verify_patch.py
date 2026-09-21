# -*- coding: utf-8 -*-
"""Step 2 verification: with NO HV_SIM_WIND_* / HV_PLAN_* set, does the patched
runner still produce the same `--defaults` chain and the same parameter gate?

Method: import BOTH versions of scripts/hv_sitl_run.py side by side (the
`.before` copy and the patched file), then call `launch()` on each with
`_instance_dir` redirected into a scratch tree and `subprocess.Popen` stubbed
out.  NOTHING is executed: no ArduPlane.exe, no port is opened, and the real
sitl/vtol/i1NN directories are never touched.

The aircraft/plan inputs are a stub built from the ARCHIVED HV4SITL15 artifacts
(mission_summary.json + sitl_cmdlines.txt), so the chain is generated from the
same values the flown sortie used.
"""
import importlib.machinery
import importlib.util
import json
import os
import re
import subprocess
import sys
import types
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs/artifacts/PAPERPLAN1/prep_runner"
SCRATCH = OUT / "verify"
REF = ROOT / "runs/artifacts/HV4SITL15"          # archived, read-only here
os.chdir(ROOT)
sys.path.insert(0, str(ROOT))

# The gate must be exercised with a clean environment.
for _k in [k for k in os.environ if k.startswith("HV_")]:
    os.environ.pop(_k)


def load(path: Path, name: str):
    # ⚠ the `.before` copy has no .py suffix ⇒ the loader cannot be inferred
    loader = importlib.machinery.SourceFileLoader(name, str(path))
    spec = importlib.util.spec_from_file_location(name, path, loader=loader)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


def stub_hv():
    ms = json.loads((REF / "plan/mission_summary.json").read_text(encoding="utf-8"))
    homes = {}
    for ln in (REF / "sitl_cmdlines.txt").read_text(encoding="utf-8").splitlines():
        u = ln[1:ln.index("]")]
        lat, lon = re.search(r"--home ([0-9.\-]+),([0-9.\-]+),", ln).groups()
        homes[u] = (float(lat), float(lon))
    sites, plans = {}, {}
    for u, p in sorted(ms["uavs"].items()):
        sid = p["takeoff_site"]
        sites[sid] = types.SimpleNamespace(lat_deg=homes[u][0], lon_deg=homes[u][1])
        plans[u] = types.SimpleNamespace(takeoff_site=sid)
    inputs = types.SimpleNamespace(sites=sites, home=types.SimpleNamespace(
        lat_deg=ms["home"]["lat_deg"], lon_deg=ms["home"]["lon_deg"]))
    return types.SimpleNamespace(
        id_map={1: "A1", 2: "B2", 3: "B3"},
        last_plan=types.SimpleNamespace(plans=plans),
        last_ctx=types.SimpleNamespace(inputs=inputs))


def chain_of(mod_path: Path, tag: str, trial: str):
    """Generate the cmdlines `launch()` WOULD use, starting nothing."""
    work = SCRATCH / tag
    work.mkdir(parents=True, exist_ok=True)
    os.environ["HV_TRIAL"] = trial
    m = load(mod_path, "hvrun_" + tag)
    # ⚠ the module computes ROOT from __file__; the `.before` copy lives under
    #   the evidence folder, so out_dir() would resolve there. Pin it to the repo.
    m.ROOT = ROOT
    import scripts.hv_sitl_flight as F

    def fake_instance_dir(i):
        d = work / ("i" + str(i))
        d.mkdir(parents=True, exist_ok=True)
        return d

    started = []

    class FakePopen:
        def __init__(self, cmd, **kw):
            started.append(cmd)
            self.pid = 900000 + len(started)

    m._instance_dir = fake_instance_dir
    F._instance_dir = fake_instance_dir
    m.subprocess = types.SimpleNamespace(Popen=FakePopen, DEVNULL=None,
                                         run=subprocess.run)
    m.time = types.SimpleNamespace(sleep=lambda s: None, time=lambda: 0.0,
                                   perf_counter=lambda: 0.0)
    sc = m.SCENARIOS["gui_legal_120_1a2b"]
    m.launch(sc, stub_hv())
    lines = (m.out_dir(sc) / "sitl_cmdlines.txt").read_text(encoding="utf-8").splitlines()
    assert started and all(str(c[0]).endswith("ArduPlane.exe") for c in started)
    assert len(started) == 3, started
    return m, lines, work


def parse(lines):
    out = {}
    for ln in lines:
        u = ln[1:ln.index("]")]
        d = re.search(r"--defaults (\S+)", ln).group(1).split(",")
        home = re.search(r"--home (\S+)", ln).group(1)
        out[u] = {"home": home, "defaults": [Path(x).name for x in d],
                  "defaults_full": d}
    return out


def sim_wind_effective(files):
    """Last-wins over the whole chain, restricted to SIM_WIND* / MIN_GROUNDSPEED."""
    eff = {}
    for f in files:
        p = Path(f)
        if not p.exists():
            eff.setdefault("_MISSING", []).append(str(p))
            continue
        for ln in p.read_text(encoding="utf-8").splitlines():
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = re.split(r"[,\s]+", ln, maxsplit=1)
            if len(parts) != 2:
                continue
            k, v = parts[0].strip(), parts[1].strip()
            if k.startswith("SIM_WIND") or k == "MIN_GROUNDSPEED":
                try:
                    eff[k] = float(v)
                except ValueError:
                    pass
    return eff


def gate(m):
    sc = m.SCENARIOS["gui_legal_120_1a2b"]
    g = {}
    for u in ("A1", "B2", "B3"):
        ff = m._param_file_values(sc, u)
        g[u] = {k: tuple(v) for k, v in m.expected_params(ff).items()}
    return g


def main():
    res = {}
    TRIAL = "PAPERPLAN1/prep_runner/verify/trial"
    tdir = ROOT / "runs/artifacts" / TRIAL
    tdir.mkdir(parents=True, exist_ok=True)
    if not (tdir / "plan").exists():
        import shutil
        shutil.copytree(REF / "plan", tdir / "plan")

    old_m, old_lines, _ = chain_of(OUT / "src/hv_sitl_run.py.before", "old", TRIAL)
    new_m, new_lines, new_work = chain_of(ROOT / "scripts/hv_sitl_run.py", "new", TRIAL)
    old, new = parse(old_lines), parse(new_lines)
    arch = parse((REF / "sitl_cmdlines.txt").read_text(encoding="utf-8").splitlines())

    res["defaults_chain"] = {
        "old": {u: old[u]["defaults"] for u in old},
        "new": {u: new[u]["defaults"] for u in new},
        "added": {u: [x for x in new[u]["defaults"] if x not in old[u]["defaults"]]
                  for u in new},
        "removed": {u: [x for x in old[u]["defaults"] if x not in new[u]["defaults"]]
                    for u in old},
        "order_prefix_unchanged": all(
            new[u]["defaults"][:len(old[u]["defaults"])] == old[u]["defaults"] for u in new),
        "home_unchanged": {u: old[u]["home"] == new[u]["home"] for u in new},
        "archived_HV4SITL15": {u: arch[u]["defaults"] for u in arch},
        "old_matches_archived": all(old[u]["defaults"] == arch[u]["defaults"] for u in arch),
        "archived_home_matches": {u: old[u]["home"] == arch[u]["home"] for u in arch},
    }
    res["sim_wind_on_chain"] = {
        "old_A1": sim_wind_effective(old["A1"]["defaults_full"]),
        "new_A1": sim_wind_effective(new["A1"]["defaults_full"]),
        "firmware_defaults_for_reference": {
            "source": "sitl/models/plane.parm (ArduPilot's own model default list)",
            "SIM_WIND_SPD": 0.0, "SIM_WIND_DIR": 180.0, "SIM_WIND_T": 0.0,
            "SIM_WIND_TURB": 0.0, "SIM_WIND_DIR_Z": 0.0},
    }
    wp = new_work / "i100" / "wind.parm"
    res["wind_parm_generated_with_no_env"] = {
        "path": str(wp), "exists": wp.exists(),
        "content": wp.read_text(encoding="utf-8") if wp.exists() else None}

    og, ng = gate(old_m), gate(new_m)
    res["param_gate"] = {
        "identical": og == ng,
        "n_params_per_uav": {u: len(ng[u]) for u in ng},
        "n_rows_total": sum(len(v) for v in ng.values()),
        "n_exported_in_file": {u: len(new_m._param_file_values(
            new_m.SCENARIOS["gui_legal_120_1a2b"], u)) for u in ng},
        "diff": None if og == ng else {
            u: {k: (og[u].get(k), ng[u].get(k)) for k in set(og[u]) | set(ng[u])
                if og[u].get(k) != ng[u].get(k)} for u in ng},
        "param_names": sorted(ng["A1"]),
    }

    src_old = (OUT / "src/hv_sitl_run.py.before").read_text(encoding="utf-8")
    src_new = (ROOT / "scripts/hv_sitl_run.py").read_text(encoding="utf-8")
    res["flight_csv_header"] = {
        "old": re.search(r'w\.writerow\(\["t_s".*?\]\)', src_old, re.S).group(0),
        "new": re.search(r'w\.writerow\(\["t_s".*?\]\)', src_new, re.S).group(0)}
    res["env_knobs_new"] = sorted(set(re.findall(r'"(HV_[A-Z_0-9]+)"', src_new)))
    res["env_knobs_old"] = sorted(set(re.findall(r'"(HV_[A-Z_0-9]+)"', src_old)))

    ok = (res["defaults_chain"]["order_prefix_unchanged"]
          and all(res["defaults_chain"]["home_unchanged"].values())
          and res["defaults_chain"]["old_matches_archived"]
          and res["param_gate"]["identical"]
          and res["param_gate"]["n_rows_total"] == 102
          and all(v == 34 for v in res["param_gate"]["n_params_per_uav"].values()))
    res["VERDICT_no_env_behaviour_unchanged"] = bool(ok)

    (OUT / "verify_patch.json").write_text(
        json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({k: v for k, v in res.items() if k != "param_gate"},
                     ensure_ascii=False, indent=1))
    print("param gate identical:", res["param_gate"]["identical"],
          " params/uav:", res["param_gate"]["n_params_per_uav"],
          " rows:", res["param_gate"]["n_rows_total"])
    print("VERDICT:", ok)


if __name__ == "__main__":
    main()
