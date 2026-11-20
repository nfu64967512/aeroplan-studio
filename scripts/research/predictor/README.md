# 點質量預測器與機體阻力校準

2026-08-13/14 產出。**這是目前唯一經過交叉驗證的機體性能模型**,
Hybrid Trajectory Shaping 的線上預測器要建在它上面。

⚠️ 這些檔原本只存在於 session 的 scratchpad,搬進 repo 是因為它們是資產不是草稿。

## 核心數字(可複現)

    c_D0(有效寄生阻力) = 0.01500     n=1617、10 架次、sd 0.00058、p10–p90 0.01446–0.01566
    交叉驗證:jet_design.py 的未校正解 0.01642/1.087 = 0.01511,差 0.7%(兩條獨立路徑)

    工作速度帶的實測升阻比
      V=157 → L/D 2.11(平衡滑翔角 25.4°)
      V=204 → L/D 1.25(38.6°)
      V=216 → L/D 1.12(41.8°)
    ⇒ 這具機體在任務速度帶**不是滑翔機,是磚頭**。

    預測器驗證(PN 段:交棒 → 120 m AGL,重放實測 γ(t))
      12/12 架次,速度 RMS 5.4 m/s、終點偏差 +0.2 ± 1.0 m/s,涵蓋 121→219 m/s

⚠️ 同一個積分器套在**交棒之前**的航點段會高估 60–110 m/s ——
那不是模型錯,是已知的 TECS 下降權限缺口(h_in→h_dem 只放行 1–33%)。

## 檔案

| 檔 | 用途 |
|---|---|
| `sim.py` | 點質量縱向積分器(本體)。其餘檔都 import 它 |
| `extract.py` | 從 `runs/artifacts/RPK*/RPL*` 抽 h–V 軌跡,產生 `traj.pkl` |
| `calib.py` | 由終端準平衡段反解 `c_D0`,並列出各速度的 L/D |
| `validate2.py` | 在 PN 段重放實測 γ(t) 驗證積分器 |
| `authority.py` | 各機自飛極值與可行時間窗 |
| `window.py` | 「交棒 γ ≤ X」與「觸地速度 ≥ Y」的可行窗口二分搜尋 |
| `gate_calc*.py` | 交棒幾何試算 |
| `stats.py` / `profile.py` / `study.py` | 統計、剖面、參數掃描 |
| `bench.py` | 執行時間量測(線上預測器的預算) |
| `gamma_lag.py` / `fitdrag.py` / `pred_audit*.py` | 各種診斷 |

## 怎麼跑

```bash
cd scripts/research/predictor && python extract.py && python calib.py && python validate2.py
```

`traj.pkl` 是 `extract.py` 的產物,**不入版控**;刪掉重跑即可。

## ⚠️ 假設(用之前必讀)

* **無風**:`V` 取自 GLOBAL_POSITION_INT 的 3D 地速,不是 TAS。
  `scripts/wsl` 下 grep 不到任何 `SIM_WIND` 設定,所以 SITL 目前無風、兩者相等。
  一旦加風,`c_D0` 的校準值會跟著偏。
* **推力模型**:`thr = clip(0,1, 0.36 + 0.030·(210 − V))`(`tot_node._pn_cmd` 的油門律),
  `THR_MAX = 100`(`rep_interactive.sh` 明確 `unset JET_THRMAX`)。
  ⚠️ 設計推力是 `THR_MAX = 25`,兩者差 4 倍 —— **換組態時這個模型要重新校準**。
* 單一 `c_D0` 取代論文的 `(Mach, α, δe)` 三維查表。在我們的次音速定構型任務裡誤差
  已由 `validate2.py` 量化(RMS 5.4 m/s),但不可外推到其他構型。

迴歸鎖:`tests/test_predictor_calibration.py`
