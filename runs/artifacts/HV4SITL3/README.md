# HV4SITL3 — 單光相機 × AeroPlan GUI 路徑，1 Pioneer + 3 HERO

2026-09-07 · ArduPlane 4.8.0-dev QuadPlane SITL ×4 · speedup 5 · 分支 `quad_swarm`
情境 `gui_uniform_1a3b`（`scripts/hv_sitl_scenarios.py`）

## 這一趟在測什麼

HV4SITL2 走的是 `HvDccppCoordinator` 直接呼叫 + 異構酬載（A 掛 LiDAR、B 掛相機）。
這一趟走的是**介面按下去真正會走的那一條**：`SwarmCoordinator.plan_coverage_dccpp`
＋ 2026-09-07 新加的單光相機（`hv_uniform_sensor=True`，兩型共用同一台相機）。
所以航線、條帶數、作業高度全部不一樣，**不是** HV4SITL2 的重跑。

## 結論

4/4 起飛、飛完、降落。航程比規劃多 **+1.4%**，落地時刻誤差 **-7.7 ~ +13.4 s**。
錯開起飛照規劃時刻發生，落地順序與規劃一致。

| 機 | 構型 | 規劃 t_off | 放行(牆鐘) 規劃/實際 | 落地 規劃/實飛 | 航程 km 規劃/實飛 | 最高 m |
|---|---|---|---|---|---|---|
| B3 | HERO | 0 s | 0.0 / 0.0 | 668 / 665 (-3.2) | 10.43 / 10.56 (+1.3%) | 140.3 |
| B4 | HERO | 270 s | 54.0 / 54.6 | 943 / 938 (-4.9) | 10.50 / 10.64 (+1.4%) | 140.2 |
| B2 | HERO | 360 s | 72.0 / 72.3 | 1197 / 1189 (-7.7) | 13.51 / 13.73 (+1.6%) | 140.2 |
| A1 | Pioneer | 1323 s | 264.6 / 265.0 | 2200 / 2213 (+13.4) | 15.96 / 16.16 (+1.3%) | 169.3 |

合計航程：規劃 50.40 km、實飛 51.09 km。放行誤差最大 0.6 s（B4）。

⚠⚠ **實飛時刻都扣掉了 45.0 s 的時基偏移。** `flight.csv` 的 `t_s` 零點不是放行
時刻：每機第一列都在 `t_s≈43-46`，而那時 A1/B2 還停在地面 MANUAL。不扣掉會把它
讀成「每一架都晚 37-58 秒」——本檔第一版就是這樣寫的，已訂正。偏移量由
`sim_t(切AUTO) = offset + 牆鐘 × speedup` 反解，八個架次量到 +43 ~ +46 s。
校正碼在 `scripts/hv_sitl_report.py::_clock_offset`。

## 規劃

`SwarmCoordinator` → HV 分支 → makespan 2199.6 s、`validate.passed=True`、
衝突 0、未覆蓋 0、四架都有條帶（5/7/4/4）。最小水平間距 **283.9 m**（規劃值，非實飛量測）。

作業區是四塊 520×600 m 的矩形，東西向排開、中心間距 1500 m。
**幾何是掃出來的**（`scripts/hv_sitl_scout.py`）：用到幾台是胞的數目決定的，
不是機隊大小 —— 換成「一塊大區」同樣四架只有兩架有工作。

感測器：`{'A': 'rgb', 'B': 'rgb'}`、酬載需求只有一個 `default`。
刈幅由面板 HFOV 60°／VFOV 45° 反推，兩型的條帶間距因此相同。
`sensor_spec_default_example` 警告消失了（HV4SITL2 有），因為相機標成
`is_default_example=False`。

## 量到的三件事

1. **Pioneer 在返場段爬到 169.7 m，比作業高度 140 m 高 30 m。**
   是**規劃如此**不是飛偏：`DO_LAND_START`(189) 之後 seq 98–105 從 150 一路爬到
   169.7，接著降到 106 → 102 → `VTOL_LAND`。實飛 169.3 m，差 0.4 m。
   三架 HERO 全程不超過 140.3 m，沒有這一段。
2. **時刻表跟得很準，但兩型的符號相反。** 扣掉時基偏移後：
   離地誤差 0 ~ +6 s，落地誤差 Pioneer **+13.4 s**、三架 HERO **-3.2 / -4.9 / -7.7 s**。
   併 HV4SITL4 一起看（n=8）：Pioneer 中位 +15.1 s、HERO 中位 -4.9 s，
   差約 20 s（約飛行時長的 2%）。目前只有現象，沒有機制。
3. **收尾時 `stop_all()` 回報「已終止 0 個」而四個行程確實已經不在。**
   代表清理路徑沒有真的被走到；下一趟起,`scripts/hv_sitl_run.py` 已改成
   只殺自己起的 PID（`stop_ours`），順帶不再誤殺別的 session 的 SITL。

## SITL 參數對齊

`--defaults` 鏈（後蓋前）：
```
plane.parm → vtol_default.parm → identity.parm → vtol_enforced.parm → <UAV>.param → firmware_override.parm
```
**參數讀回 68/68 相符**（`params_readback.csv`）。唯一的差異是型別截斷：
HERO 的 `AIRSPEED_MAX` 檔案寫 26.6、韌體 INT16 ⇒ 生效 26.0（三架都是）。
任務上傳 109/121/75/70 項，下載回來項數全部一致。

## 實飛量到的最小間距（規劃值不是上界）

| | 規劃 `min_sep_h_m` | 實飛量到 | 差 |
|---|---|---|---|
| HV4SITL3 | 283.9 m | **143.8 m**（B2-B3，t≈573 s） | 規劃值高估 2.0 倍 |
| HV4SITL4 | 528.7 m | 541.3 m（A2-B4，t≈1264 s） | 大致相符 |

HV4SITL3 那一刻 B2 在 140.2 m、B3 在 96.7 m，垂直還隔著 43.5 m，
而面板的最小間距門檻是 50 m，所以**這一趟沒有違規**。但要記住：
`validation.json` 的 `min_sep_h_m` 是規劃值，**不保證是實飛的下界**，
這一趟就差了兩倍。`min_sep_v_m` 是 `Infinity`，代表垂直方向根本沒被評估。
量法見 `scripts/hv_sitl_report.py::_min_separation`（只在兩架都 >2 m AGL 時計算）。

## 這一趟**沒有**證明的事

* 覆蓋率是**上界**（航跡緩衝法），不是「合格影像涵蓋率」。
* 機體物理仍是 SITL 內建 `quadplane` 模型：對齊的是**飛控參數**，不是質量與氣動。
* 只跑 speedup 5，沒有做倍率 A/B。
* 這一趟與 HV4SITL2 **不是**受控對照（幾何與高度都換了）。受控對照見下。

## 附帶的受控對照（規劃層，沒有飛）

同一份幾何（HV4SITL2 的兩塊區）、同一組機隊，只換規劃路徑：

| | 分配 | 未覆蓋 | status |
|---|---|---|---|
| 異構酬載（`hv_uniform_sensor=False`） | A1:3 B2:2 B3:1 | 0 | GO_WITH_WARNINGS |
| 單光相機（True） | A1:4 | **1** | GO_WITH_WARNINGS |

原因：那份幾何的 Z1 自帶 `sensor_req='lidar'`，單光相機之下沒有構型掛 LiDAR
⇒ `no_feasible_class`，整塊被丟掉。**errors 是空的、status 不是 NO_GO、
`raise_on_no_go=True` 也不會 raise**，只留在 `plan.uncovered` 與一則 note。
已在 `mission/swarm_coordinator.py` 補一道點名警告，守衛與現象都鎖在
`tests/test_hv_uniform_sensor.py`（突變實驗驗證會咬）。

## 重現

```bash
python scripts/hv_sitl_run.py fly gui_uniform_1a3b && python scripts/hv_sitl_report.py gui_uniform_1a3b
```

## 檔案

```
plan/                     export_all 的完整輸出（每機 .waypoints/.param/rally/fence/.plan）
scenario.json             這一趟的情境（機隊、作業區、高度、FOV、風、speedup）
A1/ B2/ B3/ B4/flight.csv t_s(模擬時鐘),lat,lon,rel_alt_m,airspeed,groundspeed,wp_seq,mode,armed
params_readback.csv       每架 17 個參數的 期望/實得/是否相符/備註
mission_upload.json       上傳項數 vs 下載回來的項數
release_schedule.json     規劃起飛時刻 → 牆鐘放行時刻 → 實際放行時刻
flight_summary.json       上表的機器可讀版（落地時刻用真判準，不是 max(t_s)）
validation.json report.json   規劃側的驗證與 preflight 報告
compare.png               左：規劃(虛線) vs 實飛(實線) 航跡；右：高度 vs 模擬時間
sitl_cmdlines.txt         四個 ArduPlane.exe 的完整命令列
hv_sitl_*.py              當時用的驅動腳本
```
