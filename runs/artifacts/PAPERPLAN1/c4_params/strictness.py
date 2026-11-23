# -*- coding: utf-8 -*-
"""「相符」的三種定義，各自算一次分母/分子；並輸出逐項對照表 (markdown + csv)。

定義 D1 原始規劃值嚴格比對：只看匯出檔 32x3=96 項，容差 max(0.01, 0.1%|exp|)，
                             不套 FIRMWARE_OVERRIDE、不套整數救援。
定義 D2 韌體正規化後之期望值（＝程式現行判準）：比對集合 34x3=102 項，
                             FIRMWARE_OVERRIDE 改期望值、整數型別救援視為相符。
定義 D3 同 D2 但不含整數救援（看整數救援救了幾筆）。
"""
from __future__ import annotations
import csv, json
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
ART = ROOT / "runs" / "artifacts"
HERE = Path(__file__).resolve().parent
TRIAL = "HV4SITL16"


def param_file(p: Path):
    d = {}
    for ln in p.read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if ln and not ln.startswith("#") and "," in ln:
            k, v = ln.split(",", 1)
            d[k.strip()] = float(v)
    return d


def tol_ok(got, exp):
    return got is not None and abs(got - exp) <= max(0.01, abs(exp) * 0.001)


def int_rescue(got, exp):
    return got is not None and got in (float(int(exp)), float(round(exp)))


def main():
    files = {f.stem: param_file(f) for f in sorted((ART / TRIAL / "plan").glob("*.param"))}
    rows = list(csv.DictReader((ART / TRIAL / "params_readback.csv").open(encoding="utf-8")))
    actual = {(r["uav"], r["param"]): (float(r["actual"]) if r["actual"] not in ("", "None") else None)
              for r in rows}

    d1 = {"n": 0, "ok": 0, "fail": []}
    for uav, fv in files.items():
        for name, raw in fv.items():
            got = actual.get((uav, name))
            d1["n"] += 1
            if tol_ok(got, raw):
                d1["ok"] += 1
            else:
                d1["fail"].append(f"{uav}.{name} planned={raw:g} read={got}")

    d2 = {"n": len(rows), "ok": sum(r["ok"].lower() == "true" for r in rows)}
    d3 = {"n": len(rows), "ok": 0, "rescued": []}
    for r in rows:
        exp = float(r["expected"])
        got = float(r["actual"]) if r["actual"] not in ("", "None") else None
        if tol_ok(got, exp):
            d3["ok"] += 1
        elif int_rescue(got, exp):
            d3["rescued"].append(f'{r["uav"]}.{r["param"]} expect={exp:g} read={got:g} '
                                 f'rel_err={abs(got - exp) / abs(exp) * 100:.2f}%')

    # 逐項對照表
    table = []
    for uav in sorted(files):
        for name in sorted(set(files[uav]) | {"ARMING_REQUIRE", "SCHED_LOOP_RATE"}):
            raw = files[uav].get(name)
            got = actual.get((uav, name))
            crow = next((r for r in rows if r["uav"] == uav and r["param"] == name), None)
            exp = float(crow["expected"]) if crow else None
            table.append({"uav": uav, "param": name,
                          "exported": "" if raw is None else f"{raw:g}",
                          "expected_after_normalisation": "" if exp is None else f"{exp:g}",
                          "read_back": "" if got is None else f"{got:g}",
                          "source": crow["source"] if crow else "",
                          "class": ("firmware_override" if crow and crow["source"] == "FIRMWARE_OVERRIDE"
                                    else "fallback_const" if crow and crow["source"] == "EXPECT_FALLBACK"
                                    else "int_truncated" if crow and "整數型別" in (crow["note"] or "")
                                    else "exact")})
    with (HERE / "c4_param_table.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader()
        w.writerows(table)

    res = {"trial": TRIAL,
           "D1_raw_planned_strict": d1,
           "D2_firmware_normalised_current_gate": d2,
           "D3_normalised_without_int_rescue": d3,
           "tolerance_formula": "abs(got-exp) <= max(0.01, abs(exp)*0.001)  [scripts/hv_sitl_run.py:527-528]",
           "int_rescue_formula": "got in (float(int(exp)), float(round(exp)))  [scripts/hv_sitl_run.py:530-532]"}
    (HERE / "c4_strictness.json").write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
