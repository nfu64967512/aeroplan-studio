# -*- coding: utf-8 -*-
"""label update_paper / stage 7 -- corrected copies of the earlier labels' artifacts.

RULE: never overwrite someone else's evidence.  For every file that still
carries one of the K1..K9 claims, this writes

  corrected/<label>__<file>.CORRECTION.{md,json}

which records the original's sha256 at the time of correction, the exact wrong
text, the recomputed value, and the wording that replaces it.  It also drops a
small, clearly-named POINTER file next to the original (a NEW file, never an
overwrite) so a reader who opens the original finds the correction.

Re-runnable and deterministic: it rewrites only files under this label plus the
pointer cards, and the pointer cards' content depends only on this script.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from up_lib import OUT, ROOT, jdump, jload

PP = ROOT / "runs" / "artifacts" / "PAPERPLAN1"
HERE = Path(__file__).resolve().parent.parent
CORR = HERE / "corrected"
C = jload(OUT / "corrections.json")
LABEL = "update_paper"
POINTER_NAME = "READ_ME_FIRST__corrections_in_update_paper.md"

# (key, original path relative to PAPERPLAN1, [K-ids], summary)
TARGETS = [
    ("paper_text__TABLES_ZH", "paper_text/TABLES_ZH.md",
     ["K1", "K2", "K3", "K4", "K5", "K9"],
     "表 2b 表註 (e) 與 (b)、表 2 參數列、表 X 之 BEAMIDP1 腳註、表 Z 標題與圖說首段"),
    ("paper_text__PAPER_TEXT_ZH", "paper_text/PAPER_TEXT_ZH.md",
     ["K2", "K6", "K7"],
     "C4-b/C4-e/C4-f 的 91/96、:51 與 :74 的 +0.02 s、C2-f 的附錄 B :276 區間"),
    ("paper_text__README", "paper_text/README.md",
     ["K2", "K5", "K6"],
     "一.1 的 +0.02 s、一.5 未揭露 BEAMIDP1 的無風模擬、一.7 的 91/96"),
    ("paper_text__out_NUMBERS", "paper_text/out/NUMBERS.json",
     ["K2"], "upload_and_params.*.strict_* 欄位之措辭"),
    ("prep_text__C1_C2_placeholders", "prep_text/C1_C2_placeholders.md",
     ["K6"], ":20 與 :33 的 makespan +0.02 s"),
    ("prep_text__C4_params", "prep_text/C4_params.md",
     ["K2"], "91/96 的「嚴格比對」措辭"),
    ("prep_text__Table2_skeleton", "prep_text/Table2_skeleton.md",
     ["K2"], "表 2 骨架中的 91/96"),
    ("sorties__sortie_results", "sorties/sortie_results.json",
     ["K3", "K4"], "status_reason 與 spawn_diagnosis 的順風說法與「逐位元重現」"),
    ("workflow__sortie_workflow_result", "sortie_workflow_result.json",
     ["K8"], "ana/fly 敘述中的「dataflash 沒有電池遙測」"),
]


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def body(kid: str) -> dict:
    k1 = C["K1_separation_note_e"]
    k2 = C["K2_param_strict_comparison"]
    k3 = C["K3_K4_spawn"]
    k5 = C["K5_beamidp_makespan_confound"]
    k6 = C["K6_ceiling_cost_s"]
    k7 = C["K7_appendixB_276"]
    k8 = C["K8_battery_telemetry"]
    k9 = C["K9_table_z_rulers"]
    return {
        "K1": {
            "wrong": "「最小水平間距…與既有報表程式之值差 3.4–5.3 m」",
            "why_wrong": "不可重現。3.4–5.3 m 既不是對 2 s 網格報表的差，也不是對原生取樣平面框的差。",
            "recomputed": {
                "vs_separation_from_tracks_json_m": k1["range_vs_reporter_m"],
                "vs_binsep_json_m": k1["range_vs_binsep_m"],
                "of_which_flat_earth_frame_m": k1["range_frame_component_m"],
                "of_which_at_sample_vs_continuous_m": k1["range_sampling_component_m"],
                "per_trial": {t: {kk: v[kk] for kk in
                                  ("exact_ellipsoid_m", "reporter_2s_grid_m", "binsep_native_flat_m",
                                   "delta_vs_reporter_m", "delta_vs_binsep_m",
                                   "frame_component_m", "sampling_component_m")}
                              for t, v in k1["per_trial"].items()},
                "the_quantity_3.4_to_5.3_resembles": {
                    "name": "planned minus flown (NOT a ruler difference)",
                    "values_m": k1["what_3.4_to_5.3_probably_was"]},
            },
            "replacement": "見 update_paper/TABLES_ZH.md 表 2b 表註 (e) 與 (f)。",
        },
        "K2": {
            "wrong": "「原始值嚴格比對 91／96」",
            "why_wrong": "「嚴格」若照字面做逐位元比較會得到 85/96；91/96 只在 float32 精度下成立。",
            "recomputed": {"strict_exact": k2["HV4W55"]["strict_exact"],
                           "strict_float32": k2["HV4W55"]["strict_float32"],
                           "float32_roundtrip_rows": k2["float32_roundtrip_rows_HV4W55"],
                           "genuinely_different_rows": k2["genuinely_different_rows_HV4W55"],
                           "identical_on_all_eight_trials": True},
            "replacement": "「**到 float32 精度**之原始值比對為 91／96；逐位元為 85／96。」",
        },
        "K3": {
            "wrong": "「180° 對機首朝北是純順風，把停放機體往北推 ~5 m」",
            "why_wrong": "位移方位角是 43.60°–44.38°（東北）。純順風只能解釋北向分量。",
            "recomputed": {"per_uav": k3["per_uav"],
                           "bearing_range_deg": k3["bearing_range_deg"],
                           "arc": jload(OUT / "spawn_verification.json")["B_body_axis_arc"],
                           "truth_vs_estimate": jload(OUT / "spawn_verification.json")["A_truth_vs_estimate"],
                           "ground_weathervane": jload(OUT / "spawn_verification.json")["C_ground_weathervane"]},
            "replacement": "「機體在地面自由偏航轉向來風，位移鎖在機體縱軸；~5 m 是該圓弧的弦。"
                           "位移的量值未能自原始碼導出，照實聲明。」見 PAPER_TEXT_ZH.md C2-i。",
        },
        "K4": {
            "wrong": "「兩次獨立執行逐位元重現」",
            "why_wrong": "檔案不同：spawn_readback.json 的 settle_s 與 report.json 的 timing_ms。",
            "recomputed": {"file_bit_identity": k3["file_bit_identity"],
                           "spawn_positions_identical": {u: (v["got_lat_identical"] and v["got_lon_identical"])
                                                         for u, v in k3["per_uav"].items()},
                           "rigid_translation_max_spacing_delta_m": k3["max_abs_spacing_delta_m"]},
            "replacement": "「**生成位置**逐位元重現；檔案本身不同。」",
        },
        "K5": {
            "wrong": "BEAMIDP1 腳註「完工時間規劃 2 348.0 → 實飛 2 272.3 s（−3.22 %）」（未揭露）",
            "why_wrong": "該架次為『規劃風 5 m/s/270°、模擬風 0』，是本文指摘 HV4SITL15/16 的同一缺陷。",
            "recomputed": k5,
            "replacement": "腳註刪除 makespan 一項，或原句揭露。見 TABLES_ZH.md 表 X 腳註。",
        },
        "K6": {
            "wrong": "「代價只有 +0.02 s 完工時間」",
            "why_wrong": "四捨五入假影。",
            "recomputed": k6,
            "replacement": "寫 +0.0279 s，或兩個四位小數值 1 260.4261 → 1 260.4540。",
        },
        "K7": {
            "wrong": "附錄 B :276 的替換文字引用 HV4SITL15 的 −0.001~−0.019 m/s",
            "why_wrong": ":276 那一列的出處是 HV4SITL16。",
            "recomputed": k7,
            "replacement": "改成 HV4SITL16 的 +0.005／−0.002／−0.019 m/s；"
                           "HV4SITL15 的值放括號。結論 |W| ≤ 0.02 m/s 不變。",
        },
        "K8": {
            "wrong": "「這個機體模型的 dataflash 沒有電池遙測」",
            "why_wrong": "BAT 訊息存在於每趟每機（BEAMIDP1 的 B2 有 5 039 筆）。",
            "recomputed": {t: v for t, v in k8.items() if t not in ("verdict",)},
            "replacement": "「SITL 記錄的是通用 12.6 V 模擬電池，不是規劃器模型之 HERO 電池，"
                           "故能量餘裕為規劃模型值，飛行紀錄無法佐證。」",
        },
        "K9": {
            "wrong": "表 Z 標題「（同一把尺）」與圖說「兩把檢查尺是同一把」",
            "why_wrong": "兩臂受檢的物件不同：基準走重抬升航跡，HARVEST 走原生 MissionPlan.plans。",
            "recomputed": k9,
            "replacement": "標題刪去「（同一把尺）」；圖說首段改為「HARVEST 的 0 來自其原生四維排程，"
                           "基準沒有這個排程，代價見第 7 列（起飛佇列 330–6 460 s）」，"
                           "並揭露 HARVEST 重抬升後為 959／688。",
        },
    }[kid]


def main() -> None:
    CORR.mkdir(parents=True, exist_ok=True)
    bodies = {k: body(k) for k in ("K1", "K2", "K3", "K4", "K5", "K6", "K7", "K8", "K9")}
    index = {"label": LABEL,
             "rule": "originals are never modified; each entry is a corrected COPY plus a "
                     "pointer card written beside the original",
             "authority": "update_paper/TABLES_ZH.md and update_paper/PAPER_TEXT_ZH.md are the "
                          "final text; the entries below say what changed and why",
             "entries": []}
    lines = ["# `corrected/` 索引 —— 前面各 label 產物的更正副本（label: update_paper）", "",
             "> **原檔一律未被修改。** 每一列給：原檔、更正當下的 sha256、涉及哪幾項 K、更正副本路徑。",
             "> 定稿文字在 `update_paper/TABLES_ZH.md` 與 `update_paper/PAPER_TEXT_ZH.md`；",
             "> 逐項重算與出處在 `update_paper/CORRECTIONS_K1_K9.md`；機器可讀在 `update_paper/out/corrections.json`。",
             "", "| 原檔 | 更正當下 sha256（前 16） | 涉及 | 更正副本 |", "|---|---|---|---|"]
    for key, rel, kids, summary in TARGETS:
        src = PP / rel
        if not src.exists():
            print("MISSING (skipped):", rel)
            continue
        h = sha(src)
        outp = CORR / f"{key}.CORRECTION.json"
        jdump(outp, {
            "corrected_by_label": LABEL,
            "original": f"runs/artifacts/PAPERPLAN1/{rel}",
            "original_sha256_at_correction_time": h,
            "original_bytes": src.stat().st_size,
            "original_is_unmodified": True,
            "what_in_this_file_is_wrong": summary,
            "corrections": {k: bodies[k] for k in kids},
            "final_text_lives_in": [
                "runs/artifacts/PAPERPLAN1/update_paper/TABLES_ZH.md",
                "runs/artifacts/PAPERPLAN1/update_paper/PAPER_TEXT_ZH.md"],
            "recomputation_scripts": "runs/artifacts/PAPERPLAN1/update_paper/scripts/up_*.py",
        })
        index["entries"].append({"original": f"runs/artifacts/PAPERPLAN1/{rel}",
                                 "sha256": h, "k_ids": kids,
                                 "correction_copy": f"runs/artifacts/PAPERPLAN1/update_paper/corrected/{outp.name}",
                                 "summary": summary})
        lines.append(f"| `{rel}` | `{h[:16]}` | {'、'.join(kids)} | `corrected/{outp.name}` |")
        print(f"wrote corrected/{outp.name}  <- {rel}")

    # reconciliation (not a correction): two valid methods, different numbers
    rec = CORR / "sorties2__HV4W55S_windaxis.RECONCILIATION.json"
    w = jload(OUT / "wind.json")
    jdump(rec, {
        "written_by_label": LABEL,
        "not_a_correction": "both methods are valid; this records why the numbers differ",
        "other_label_file": "runs/artifacts/PAPERPLAN1/sorties2/HV4W55S_windaxis.json",
        "other_label_method": "~1 Hz flight.csv; track from consecutive lat/lon with a flat "
                              "111132 m/deg frame; rel-alt floor 30 m; steps >= 5 m",
        "this_label_method": "native-rate dataflash GPS.Spd minus interpolated ARSP.Airspeed; "
                             "heading from GPS.GCrs; rel-alt floor swept (100 m and 30 m); "
                             "no median quoted from fewer than 10 samples",
        "north_minus_south_mps": {
            "sorties2_flightcsv_30m": {"A1": 9.9, "B2": 9.68, "B3": 9.61},
            "update_paper_dataflash_30m": {u: w["floors"]["30m"]["HV4W55S"][u]["north_minus_south"]
                                           for u in ("A1", "B2", "B3")},
            "update_paper_dataflash_100m": {u: w["floors"]["100m"]["HV4W55S"][u]["north_minus_south"]
                                            for u in ("A1", "B2", "B3")}},
        "sample_counts_dataflash": {
            fl: {u: {k: w["floors"][fl]["HV4W55S"][u]["bins"][k]["n"] for k in ("E", "W", "N", "S")}
                 for u in ("A1", "B2", "B3")} for fl in ("100m", "30m")},
        "the_point": "at a 100 m cruise floor B2's SOUTHBOUND bin keeps exactly 1 sample, so its "
                     "north-minus-south is not quotable there; the paper therefore quotes the "
                     "30 m floor for the 180 deg cell and says so in the table note. Both "
                     "methods agree on the verdict (+9.5..+9.9 vs an expected +10.0) and on the "
                     "null axis (~0).",
        "paper_uses": "update_paper 30 m floor values",
    })
    lines += ["", "## 非更正、但必須說明的一則",
              "", f"| `sorties2/HV4W55S_windaxis.json` | — | （方法對照） | `corrected/{rec.name}` |",
              "",
              "兩者都對：該 label 用 ~1 Hz `flight.csv` 並自連續經緯度推算航跡，本 label 用原生 dataflash 的 "
              "`GPS.Spd`／`GPS.GCrs`。180° 那一趟在 100 m 巡航門檻下 B2 的南向箱只剩 1 個樣本，"
              "故本文改引 30 m 門檻並在表註寫明。結論（+9.5~+9.9 對期望 +10.0、歸零軸 ≈ 0）兩法一致。"]
    print(f"wrote corrected/{rec.name}")

    jdump(CORR / "INDEX.json", index)
    (CORR / "INDEX.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # pointer cards beside the originals (new files, never overwriting evidence)
    card_dirs = sorted({(PP / rel).parent for _, rel, _, _ in TARGETS if (PP / rel).exists()})
    for d in card_dirs:
        rels = [rel for _, rel, _, _ in TARGETS if (PP / rel).parent == d and (PP / rel).exists()]
        txt = [f"# ⚠ 本資料夾的部分檔案已由 label `{LABEL}` 更正（原檔未被修改）", "",
               "受影響的檔案：", ""]
        txt += [f"- `{Path(r).name}`" for r in rels]
        txt += ["", "更正內容、重算數字與定稿措辭在：", "",
                "- `runs/artifacts/PAPERPLAN1/update_paper/CORRECTIONS_K1_K9.md`（逐項說明）",
                "- `runs/artifacts/PAPERPLAN1/update_paper/corrected/INDEX.md`（索引與 sha256）",
                "- `runs/artifacts/PAPERPLAN1/update_paper/TABLES_ZH.md`、`PAPER_TEXT_ZH.md`（定稿文字）",
                "",
                "這張卡片是**新增**檔案，本資料夾原有的任何檔案都沒有被覆寫。", ""]
        (d / POINTER_NAME).write_text("\n".join(txt), encoding="utf-8")
        print("pointer card ->", (d / POINTER_NAME).relative_to(ROOT))


if __name__ == "__main__":
    main()
