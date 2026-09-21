"""Build the C2 wind patch for scripts/hv_sitl_run.py WITHOUT touching the repo.

Writes `hv_sitl_run.py.new` next to this file; a unified diff against the real
file is then produced with `diff -u` and checked with `git apply --check`.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
HERE = Path(__file__).resolve().parent
src = (ROOT / "scripts" / "hv_sitl_run.py").read_text(encoding="utf-8")
out = src


def rep(old: str, new: str) -> None:
    global out
    assert out.count(old) == 1, (out.count(old), old[:70])
    out = out.replace(old, new)


# (a) wind config constants -------------------------------------------------
rep(
    '_POLL_S = float(os.environ.get("HV_POLL_S", "0.05"))\n',
    '_POLL_S = float(os.environ.get("HV_POLL_S", "0.05"))\n'
    "# ── SITL 風場（C2）──────────────────────────────────────────────────\n"
    "#   ⚠⚠⚠ 規劃層的風在 `Scenario.wind`（預設 5 m/s／270°），而 SITL 端到 2026-09-14\n"
    "#   為止**從來沒有設過 SIM_WIND_***（見 runs/artifacts/*/sitl_cmdlines.txt）⇒\n"
    "#   「含風規劃」是被「無風模擬」驗證的。這兩個量必須分開給、分開讀回。\n"
    "#   方向約定：SIM_WIND_DIR 與 hv_dccpp 的 `dir_from_deg` 同一個約定，都是氣象\n"
    '#   「來向」（ArduPilot SITL.cpp: "Direction simulated wind is coming from"）\n'
    "#   ⇒ 270 = 西風，吹向東。\n"
    "#   SIM_WIND_T=1（none）：風不隨高度衰減，與規劃的單一 0–3000 m AGL 風層一致；\n"
    "#   預設 0（square law）會讓 AGL < SIM_WIND_T_ALT(60 m) 的風乘上 sqrt(agl/60)。\n"
    "#   SIM_WIND_TURB 一律 0：湍流走 rand() ⇒ 會打掉 SITL 的決定性。\n"
    'SIM_WIND_SPD = float(os.environ.get("HV_SIM_WIND_SPD", "0"))\n'
    'SIM_WIND_DIR = float(os.environ.get("HV_SIM_WIND_DIR", "270"))\n'
    'SIM_WIND_TURB = float(os.environ.get("HV_SIM_WIND_TURB", "0"))\n'
    'SIM_WIND_T = float(os.environ.get("HV_SIM_WIND_T", "1"))\n',
)

# (b) _write_wind -----------------------------------------------------------
rep(
    "def launch(sc: Scenario, hv) -> list[dict]:\n",
    "def _write_wind(i: int) -> Path:\n"
    '    """SITL 風場參數檔，排在 ``--defaults`` 最後（後蓋前）。\n'
    "\n"
    "    ⚠ 這一份是**模擬器**的風，跟規劃層的 `Scenario.wind` 是兩件事：前者決定飛機\n"
    "      真的遭遇什麼風，後者決定規劃器**以為**會遭遇什麼風。刻意不綁在一起，\n"
    "      因為「規劃 0 飛 5」與「規劃 5 飛 5」的對照才是論文要的那個證據。\n"
    '    """\n'
    '    p = _instance_dir(i) / "wind.parm"\n'
    "    p.write_text(\n"
    '        "# 由 scripts/hv_sitl_run.py 產生；SIM_WIND_DIR 是氣象「來向」（270 = 西風）\\n"\n'
    '        f"SIM_WIND_SPD  {SIM_WIND_SPD:g}\\n"\n'
    '        f"SIM_WIND_DIR  {SIM_WIND_DIR:g}\\n"\n'
    '        f"SIM_WIND_TURB {SIM_WIND_TURB:g}\\n"\n'
    '        f"SIM_WIND_T    {SIM_WIND_T:g}\\n"\n'
    '        "SIM_WIND_DIR_Z 0\\n", encoding="utf-8")\n'
    "    return p\n"
    "\n"
    "\n"
    "def launch(sc: Scenario, hv) -> list[dict]:\n",
)

# (c) defaults chain --------------------------------------------------------
rep(
    "            _write_firmware_override(i),\n        ]\n",
    "            _write_firmware_override(i),\n"
    "            _write_wind(i),          # ★ 最後蓋：SITL 風場（C2）\n        ]\n",
)

# (d) flight.csv header -----------------------------------------------------
rep(
    '            w.writerow(["t_s", "lat", "lon", "rel_alt_m", "airspeed",\n'
    '                        "groundspeed", "wp_seq", "mode", "armed"])\n',
    '            w.writerow(["t_s", "lat", "lon", "rel_alt_m", "airspeed",\n'
    '                        "groundspeed", "wp_seq", "mode", "armed",\n'
    '                        "wind_spd_est", "wind_dir_from_est"])\n',
)

# (e) row writer ------------------------------------------------------------
rep(
    '                    getattr(tel, "mission_seq", None), getattr(tel, "mode", ""),\n'
    '                    getattr(tel, "armed", None)])\n',
    '                    getattr(tel, "mission_seq", None), getattr(tel, "mode", ""),\n'
    '                    getattr(tel, "armed", None),\n'
    "                    # ★ 飛控自己的風估計（MAVLink WIND）—— 風真的有作用的獨立證據。\n"
    "                    #   跟「groundspeed − airspeed」互為對照：兩者都指向同一個 W 才算數。\n"
    '                    getattr(tel, "wind_speed_mps", None),\n'
    '                    getattr(tel, "wind_dir_from_deg", None)])\n',
)

# (f) wind readback ---------------------------------------------------------
rep(
    '            print("  ⚠ 有機體沒吃到速率設定 ⇒ 該機的間距量測仍是 5 s 解析度，"\n'
    '                  "低於數百公尺的結論不可信")\n',
    '            print("  ⚠ 有機體沒吃到速率設定 ⇒ 該機的間距量測仍是 5 s 解析度，"\n'
    '                  "低於數百公尺的結論不可信")\n'
    "\n"
    "        # ── 風場讀回（C2）─────────────────────────────────────────\n"
    "        #   ⚠⚠ 與 params_readback.csv **分開寫**：SIM_* 不屬於「匯出參數」那一組，\n"
    "        #   混進去會把論文的 34 × 3 = 102 變成 38 × 3，把 C4 的數字攪亂。\n"
    "        #   ⚠ 讀回 None ＝ 韌體沒有這個參數名（非 SITL build）⇒ 當成失敗，不可放行。\n"
    "        #   ⚠⚠⚠ MIN_GROUNDSPEED 也在這一組：它不是 SIM_ 參數，但只要 > 0，TECS 會為了\n"
    "        #   維持地速而在逆風腿**自己加空速** ⇒ 風的影響被飛控吃掉，整個對照作廢，\n"
    "        #   而且沒有任何徵狀。匯出檔沒有設它（韌體預設 0），所以這裡是讀回確認。\n"
    '        want_wind = {"SIM_WIND_SPD": SIM_WIND_SPD, "SIM_WIND_DIR": SIM_WIND_DIR,\n'
    '                     "SIM_WIND_TURB": SIM_WIND_TURB, "SIM_WIND_T": SIM_WIND_T,\n'
    '                     "MIN_GROUNDSPEED": 0.0}\n'
    "        wind_rows, wind_bad = [], []\n"
    "        for uav, lk in links.items():\n"
    "            for name, exp in want_wind.items():\n"
    "                got = lk.get_param(name, timeout_s=10.0)\n"
    "                ok = got is not None and abs(float(got) - exp) <= 0.01\n"
    '                wind_rows.append({"uav": uav, "param": name, "expected": exp,\n'
    '                                  "actual": got, "ok": bool(ok)})\n'
    "                if not ok:\n"
    '                    wind_bad.append(f"{uav}.{name} 期望 {exp} 實得 {got}")\n'
    '        (out / "wind_readback.json").write_text(json.dumps(\n'
    '            {"sim_wind": want_wind, "plan_wind": list(sc.wind), "rows": wind_rows,\n'
    '             "dir_convention": "氣象「來向」（270 = 西風，吹向東），與 hv_dccpp '
    'dir_from_deg 同約定"},\n'
    "            ensure_ascii=False, indent=1), encoding=\"utf-8\")\n"
    "        n_ok = sum(r[\"ok\"] for r in wind_rows)\n"
    '        print(f"\\nSITL 風場讀回：{n_ok}/{len(wind_rows)} 相符"\n'
    '              f"（SIM_WIND_SPD={SIM_WIND_SPD:g} DIR={SIM_WIND_DIR:g}）"\n'
    '              f"；規劃假設風 {sc.wind}")\n'
    "        if wind_bad:\n"
    "            for b in wind_bad:\n"
    '                print("   ✗", b)\n'
    '            raise SystemExit("風場參數沒有生效，停止起飛（見 wind_readback.json）")\n',
)

# (g) scenario.json record --------------------------------------------------
rep(
    '         "wind": list(sc.wind), "areas": sc.areas, "nfz": sc.nfz, "note": sc.note,\n',
    '         "wind": list(sc.wind), "areas": sc.areas, "nfz": sc.nfz, "note": sc.note,\n'
    "         # ★ 規劃風（上一行）與**模擬風**是兩件事，產物上必須分得出來\n"
    '         "sim_wind": {"spd_mps": SIM_WIND_SPD, "dir_from_deg": SIM_WIND_DIR,\n'
    '                      "turb": SIM_WIND_TURB, "profile_type": SIM_WIND_T},\n',
)

# (h) planner wind override in main() ---------------------------------------
rep(
    "    sc = SCENARIOS[key]\n",
    "    sc = SCENARIOS[key]\n"
    "    # ★ 規劃風的覆寫：做「規劃 0 飛 5」這種不匹配格子用，不用新增情境。\n"
    '    if os.environ.get("HV_PLAN_WIND_SPD"):\n'
    "        import dataclasses\n"
    '        sc = dataclasses.replace(sc, wind=(float(os.environ["HV_PLAN_WIND_SPD"]),\n'
    '                                           float(os.environ.get("HV_PLAN_WIND_DIR", "270"))))\n'
    '        print(f"⚠ HV_PLAN_WIND_SPD：規劃假設風改為 {sc.wind}（SITL 風是另一個旋鈕）")\n',
)

(HERE / "hv_sitl_run.py.new").write_text(out, encoding="utf-8")
print("ok", len(src), "->", len(out))
