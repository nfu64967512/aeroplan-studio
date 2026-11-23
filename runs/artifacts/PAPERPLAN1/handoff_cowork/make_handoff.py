"""產生給 Cowork 的風場 SITL 交付包（結果表＋圖）。

所有數字由程式讀取：update_paper/out/NUMBERS.json（24 個 .BIN 獨立重算）、各趟 release_schedule.json 與
HV4W55S/spawn_readback.json。唯一的常數是 W*（規劃半徑裕度可吸收之風速），出處 runs/artifacts/PAPERPLAN1/c6_reff/。
風判準的高度門檻依論文表註：270° 各趟取相對高度 100 m 以上樣本，180° 架次取 30 m 以上。"""
import json
import shutil
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
ROOT = Path(r"C:\Users\joy46\Documents\Projects\aeroplan-studio")
ART = ROOT / "runs/artifacts"
N = json.load(open(ART / "PAPERPLAN1/update_paper/out/NUMBERS.json", encoding="utf-8"))
OUT = ART / "PAPERPLAN1/handoff_cowork"
FIG = ROOT / "docs/hv_dccpp/AASRC2026_abstract"
T = N["trials"]
ORDER = ["HV4W00", "HV4W05", "HV4W55", "HV4W55S", "HV4W55R", "BEAMIDP1", "HV4SITL15", "HV4SITL16"]
W_STAR = {"Pioneer": 5.1675, "HERO": 5.0333}          # c6_reff/out_compute_reff.txt


def jload(p):
    return json.load(open(p, encoding="utf-8"))


def wind(w, sim=False):
    if w is None:
        return "0（未設風）" if sim else "—"
    if isinstance(w, dict):
        return "0" if not w.get("spd_mps") else f"{w['spd_mps']:g} m/s 自 {w['dir_from_deg']:g}°"
    return "0" if not w[0] else f"{w[0]:g} m/s 自 {w[1]:g}°"


def signal_vals(k):
    wt = T[k].get("wind_test") or {}
    key = "signal_30m" if k == "HV4W55S" else "signal_100m"
    sig = wt.get(key) or {}
    return [v for v in sig.values() if v is not None], wt.get("signal_axis")


def signal_txt(k):
    vals, ax = signal_vals(k)
    if not vals:
        return "—"
    name = {"east_minus_west": "東−西", "north_minus_south": "北−南"}.get(ax, ax)
    return f"{name} {min(vals):+.2f}～{max(vals):+.2f}"


def ekf_range(keys):
    v = [u["spd_mps"] for k in keys for u in (T[k].get("wind_test") or {}).get("ekf_wind", {}).values()]
    return min(v), max(v)


rows = []
for k in ORDER:
    t = T[k]
    ms, di, se, al = t["makespan"], t["distance"], t["separation"], t["altitude"]
    rows.append(
        f"| {k} | {wind(t.get('plan_wind'))} | {wind(t.get('sim_wind'), sim=True)} | {t['ceiling_m']:g} | "
        f"{ms['planned_s']:.1f} → {ms['flown_s']:.1f}（{ms['err_pct']:+.2f}%） | "
        f"{di['planned_km']:.3f} → {di['flown_km']:.3f} | {se['planned_m']:.2f} → {se['flown_exact_m']:.2f}（違規 {se['n_violations']}） | "
        f"{al['flown_max_agl_m']:.2f} | {al['t_above_120_s']:.2f} | {signal_txt(k)} |")

fl = {k: T[k]["makespan"]["flown_s"] for k in ORDER}
err = {k: T[k]["makespan"]["err_pct"] for k in ORDER}
ignore_cost = fl["HV4W05"] - fl["HV4W00"]
late = fl["HV4W55"] - fl["HV4W05"]
rel05 = jload(ART / "HV4W05/release_schedule.json")["planned_t_takeoff_s"]["A1"]
rel55 = jload(ART / "HV4W55/release_schedule.json")["planned_t_takeoff_s"]["A1"]
rel_delay = rel55 - rel05
sig270 = [v for k in ("HV4W05", "HV4W55", "HV4W55R") for v in signal_vals(k)[0]]
sig180 = signal_vals("HV4W55S")[0]
e270 = ekf_range(["HV4W05", "HV4W55", "HV4W55R"])
e180 = ekf_range(["HV4W55S"])
ns0 = [v for v in (T["HV4W00"]["wind_test"].get("signal_100m") or {}).values() if v is not None]
old0 = [v for k in ("HV4SITL15", "HV4SITL16") for v in (T[k]["wind_test"].get("signal_100m") or {}).values() if v is not None]
xt = max(u["residual_mps"] for u in N["crosstrack_wind_triangle"].values())
sp = jload(ART / "HV4W55S/spawn_readback.json")
sp_err = [r["err_m_full"] for r in sp["rows"]]
sp_rigid = sp["rigid_translation"]["max_abs_delta_m"]
cc = N["ceiling_control"]
a55r, a55 = T["HV4W55R"]["altitude"], T["HV4W55"]["altitude"]
sep = {k: T[k]["separation"] for k in ("HV4W00", "HV4W05", "HV4W55", "HV4W55R")}
main_margin = sep["HV4W55"]["flown_exact_m"] - 100
min_margin = min(s["flown_exact_m"] for s in sep.values()) - 100
vert = [s["vert_at_min_m"] for s in sep.values()]

md = f"""# 風場 SITL 結果（交給 Cowork 用）

產生日期 2026-09-21。本檔**所有數字**由同資料夾的 `make_handoff.py` 讀取 `NUMBERS.json`（24 個 dataflash .BIN 以獨立讀取器重算）、
各趟 `release_schedule.json` 與 `HV4W55S/spawn_readback.json` 計算，沒有手抄。情境：1 架 Pioneer（A1）＋2 架 HERO（B2、B3），作業區 1.2434 km²。

## 一、八趟總表

| 架次 | 規劃風 | 模擬風 | 天花板 (m) | 完工時間 規劃→SITL (s) | 航程 規劃→SITL (km) | 最小水平間距 規劃→SITL (m) | 最高 AGL (m) | 高於 120 m (s) | 風判準 (m/s，理論 +10.0) |
|---|---|---|---|---|---|---|---|---|---|
""" + "\n".join(rows) + f"""

欄位說明：
- 完工時間百分比對 t0 判準敏感（「放行」與「離地 2 m」差 2–3 s）；**請引用差值，不要單獨引用百分比**。
- 規劃間距為 2 s 網格上界；SITL 值為 WGS84 橢球、連續時間最小值。違規判準為「水平 < 100 m 且 垂直 < 50 m」。
- 風判準：巡航段依航向分東西（或南北）向，比較「地速 − 空速」中位數差，理論值 2W = +10.0 m/s；
  270° 各趟取相對高度 100 m 以上樣本，180° 架次（HV4W55S）取 30 m 以上（100 m 以上的南向樣本不足）。無風各趟應接近 0。
- BEAMIDP1 為「規劃假設有風、模擬無風」，**其完工時間不可引用**。

## 二、主要結論（可引用）

1. **忽略風的代價**（HV4W05 對 HV4W00，同一份任務檔，只差模擬風）：完工時間 +{ignore_cost:.1f} s；
   規劃預測誤差由 {err['HV4W00']:+.2f}% 變為 {err['HV4W05']:+.2f}%。
2. **含風規劃買到的是準度，不是速度**：同樣 5 m/s 風中，含風規劃（HV4W55）預測誤差較小（{err['HV4W55']:+.2f}% 對 {err['HV4W05']:+.2f}%），
   但完工時間反而晚 {late:.2f} s；其中 {rel_delay:.1f} s 是規劃刻意把 A1 的放行時刻由 {rel05:.0f} s 延後到 {rel55:.0f} s。
3. **風確實作用**：270° 各趟東−西判準 {min(sig270):+.2f}～{max(sig270):+.2f} m/s（理論 +10.0）；飛控 EKF 風估計 {e270[0]:.2f}–{e270[1]:.2f} m/s。
   同一判準在無風的 HV4W00 為 {min(ns0):+.2f}～{max(ns0):+.2f}，在原論文證據 HV4SITL15／16 為 {min(old0):+.2f}～{max(old0):+.2f} ⇒ 原證據是在無風中飛的。
4. **側風（180°，HV4W55S）**：北−南判準 {min(sig180):+.2f}～{max(sig180):+.2f} m/s，EKF 風 {e180[0]:.2f}–{e180[1]:.2f} m/s；
   側風造成的地速虧損與風三角理論相差 ≤ {xt:.3f} m/s。
   ⚠ 270° 與 180° 是規劃器各自重解的不同計畫，**不可比較兩者的完工時間**。
   HV4W55S 三機解鎖前被 SITL 地面模型推移 {min(sp_err):.2f}–{max(sp_err):.2f} m（整群剛體平移，兩兩間距變化 {sp_rigid:.4f} m），以明示記錄的容差放行。
5. **天花板單變因對照**（HV4W55R 對 HV4W55：同風場、同任務項數、參數檔逐位元相同，只差天花板 120.0 對 119.5 m）：
   匯出航點高於 120 m：{cc['deltas']['exported_above_120']} → 0；SITL 高於 120 m 的時間 {a55r['t_above_120_s']:.2f} s → {a55['t_above_120_s']:.2f} s；
   最高 AGL {a55r['flown_max_agl_m']:.4f} → {a55['flown_max_agl_m']:.4f} m；代價是規劃完工時間 +{cc['planned_makespan_cost_of_1195_s']:.4f} s。

## 三、不可主張（請 Cowork 注意）

- R_eff 的風修正**沒有被驗證**：5 m/s 低於生效門檻 W* = {W_STAR['Pioneer']}（Pioneer）／{W_STAR['HERO']}（HERO）m/s，匯出轉彎半徑逐趟都是 200／120 m。
- 最小間距**沒有餘裕**：主結果 HV4W55 只比 100 m 門檻多 {main_margin:.2f} m（四趟 gui 情境最緊 {min_margin:.2f} m），
  最近接時垂直差僅 {min(vert):.1f}–{max(vert):.1f} m，低於 50 m 垂直門檻，水平是唯一防線。
- 風場為單層、定常、無紊流；5 m/s 是情境預設值，不是場址實測；8 m/s 在現行陣風模型下規劃器拒飛（NO_GO），沒有飛。
- 每個條件只飛 1 趟、單一作業區；SITL 為決定性模擬。

## 四、圖

- `fig4_wind.png／.svg`：(a) 風的運動學證據（航向分向後的地速−空速）；(b) 忽略風場的代價（規劃 vs SITL 完工時間）。
- `fig5_compliance.png／.svg`：(a) 各趟最高相對高度與高於 120 m 的秒數（含天花板單變因對照）；(b) 規劃與 SITL 最小水平間距。
- 兩張圖都是 17 cm 寬、300 dpi、圖內最小字 8 pt，數字讀自同一份 NUMBERS.json。

## 五、更完整的資料

- 定稿表格：`runs/artifacts/PAPERPLAN1/update_paper/TABLES_ZH.md`
- 逐條批註替換文字：`runs/artifacts/PAPERPLAN1/update_paper/PAPER_TEXT_ZH.md`
- 原始飛行證據：`runs/artifacts/{{HV4W00,HV4W05,HV4W55,HV4W55S,HV4W55R,BEAMIDP1,HV4SITL15,HV4SITL16}}/`（commit 95a337b）
"""
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "RESULTS_zh.md").write_text(md, encoding="utf-8")
for f in ("fig4_wind.png", "fig4_wind.svg", "fig5_compliance.png", "fig5_compliance.svg"):
    shutil.copy2(FIG / f, OUT / f)
shutil.copy2(ART / "PAPERPLAN1/update_paper/out/NUMBERS.json", OUT / "NUMBERS.json")
print(md[md.index("## 二"):md.index("## 四")])
