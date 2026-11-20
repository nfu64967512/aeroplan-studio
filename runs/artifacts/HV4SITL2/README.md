# HV4SITL2 — HARVEST 1 Pioneer + 3 HERO 真 SITL 實跑

2026-09-07 · ArduPlane 4.8.0-dev QuadPlane SITL ×4 · speedup 5 · 分支 `quad_swarm`

## 結論

四架全部起飛、飛完規劃航線、降落。**4/4 落地**，航程與規劃相差 **+2.0%**，
兩個構型各自飛在自己的作業高度（Pioneer 169.6 m、三架 HERO 100.2–100.4 m）。
起飛依規劃時刻錯開，落地順序與規劃一致。

| 機 | 構型 | 規劃 t_off | 放行（牆鐘） | 落地 規劃/實飛 | 航程 km 規劃/實飛 | 最高 m | 誤差 |
|---|---|---|---|---|---|---|---|
| B4 | HERO | 21 s | 0.0 s | 419 / 434 s | 5.54 / 5.65 | 100.2 | +2.0% |
| A1 | Pioneer | 92 s | 14.7 s | 612 / 642 s | 8.65 / 8.88 | 169.6 | +2.6% |
| B2 | HERO | 229 s | 43.0 s | 772 / 790 s | 8.10 / 8.23 | 100.4 | +1.6% |
| B3 | HERO | 813 s | 158.7 s | 1210 / 1231 s | 5.52 / 5.62 | 100.2 | +1.8% |

合計航程：規劃 27.82 km、實飛 28.38 km。落地順序 B4 → A1 → B2 → B3（與規劃相同）。
時間一律取模擬時鐘 `GLOBAL_POSITION_INT.time_boot_ms`，不是牆鐘。

## 規劃

`HvDccppCoordinator`（AeroPlan 接入用的同一個轉接層）→ makespan 1210.2 s、
`validate.passed=True`、衝突 0、未覆蓋 0、四架都有條帶（3/3/2/1）。
兩個作業區：Z1 400×400 m 給 LiDAR（h_target 160 m AGL），Z2 800×560 m 給 RGB（100 m AGL）。

## SITL 參數怎麼對齊實機

`--defaults` 鏈（後蓋前）：
```
plane.parm → vtol_default.parm → identity.parm → vtol_enforced.parm → <UAV>.param → firmware_override.parm
```
`<UAV>.param` 由 `hv_dccpp.export_all` 直接從**平台參數表**推導，每行都附出處：

| | Pioneer PNP | HERO PNP |
|---|---|---|
| AIRSPEED_CRUISE / MIN / STALL | 21.5 / 18 / 14 | 19 / 16 / 12 |
| ROLL_LIMIT_DEG | 20 | 25 |
| TECS_CLMB_MAX / SINK_MAX | 1.13 / 1.5 | 1.0 / 1.33 |
| WP_LOITER_RAD / RTL_RADIUS | 200 | 120 |
| Q_ASSIST_SPEED / Q_RTL_ALT | 17 / 120 | 15 / 100 |

滾轉限制與轉彎半徑是自洽的：`R = v²/(g·tan φ)` 用表上的 bank 算出
Pioneer 129.5 m、HERO 78.9 m，對上表列理論值 130 / 75 m（差 0.4% / 5.2%）。

**參數讀回 68/68 相符**（`params_readback.csv`）。只有一項是型別截斷：
HERO 的 `AIRSPEED_MAX` 檔案寫 26.6，韌體是 INT16 ⇒ 實際生效 26.0。

## 三個必須知道的坑（都是實測出來的）

1. **`plane.parm` 的 `ARMING_REQUIRE 0` 會讓四軸馬達停在 PWM 1000。**
   工作樹裡的 `mission/sitl_launcher.py` 不會掛 `vtol_enforced.parm`，所以用它起 VTOL
   會「看起來解鎖、實際離地 0.15 m」。本次自己組參數鏈並強制 `ARMING_REQUIRE 1`。
2. **`RTL_AUTOLAND` 與 `Q_RTL_MODE` 在 4.8 互斥。** hv 匯出的 `.param` 兩個都設，
   PreArm 直接擋：「unset one of RTL_AUTOLAND or Q_RTL_MODE」。實測：
   `Q_RTL_MODE=1 + RTL_AUTOLAND=0` 仍被拒（任務含 `DO_LAND_START`）；
   `RTL_AUTOLAND=2 + Q_RTL_MODE=0` 解鎖成功。故加 `firmware_override.parm` 關掉 Q_RTL_MODE。
3. **⚠⚠ ArduPlane 不執行任務裡的 `NAV_DELAY`。** 這是最重要的一條：
   `export_all(takeoff_delay='nav_delay')` 會把錯開起飛寫成每架任務第 1 列的
   `NAV_DELAY(93)`，而**它不會生效**。前一趟（HV4SITL1）B3 的延遲是 792 s，
   結果它在開機後 53 s 就已經在 21.9 m 高、任務推進到 seq 2；四架起飛時間差 3 s。
   **4-D 去衝突的執行機制等於沒有，而且沒有任何錯誤訊息。**
   本趟改成 `takeoff_delay='operator'`：任務檔不帶時間，由地面站在各自的
   `t_takeoff_s` 才把該機切進 AUTO（切 AUTO 就等於放行）。錯開因此真的發生。

   ⚠ 附帶：`MISSION_START` 在這版一律被拒，但 AUTO + armed 本來就會跑任務，
   所以判準要看「任務有沒有推進」而不是那道指令的回覆。
   另外**先解鎖再切 AUTO**：在 AUTO 但未解鎖時任務狀態機已經在跑。

## 這一趟**沒有**證明的事

* 沒有證明 4-D 去衝突的空間分離。錯開起飛做到了，但飛行中的最小間距沒有量測。
* 沒有驗證覆蓋率。規劃器說覆蓋 1.000，但沒有從實飛航跡回算。
* 機體物理仍是 SITL 內建的 `quadplane` 模型：對齊的是**飛控參數**，
  不是質量、氣動與推力（24.6 kg / 6.65 kg 的真實機體模型不在這裡）。
* 只跑了 speedup 5（本 repo 唯一有 VTOL AUTO 成功紀錄的倍率），沒有做倍率 A/B。

## 檔案

```
plan/                     export_all 的完整輸出（每機 .waypoints/.param/rally/fence/.plan）
  mission_summary.json    makespan、每機 t_off/t_land/能量
A1/ B2/ B3/ B4/flight.csv t_s(模擬時鐘),lat,lon,rel_alt_m,airspeed,groundspeed,wp_seq,mode,armed
params_readback.csv       每架 17 個參數的 期望/實得/是否相符/備註
mission_upload.json       上傳項數 vs 下載回來的項數（98/59/42/34，全部一致）
release_schedule.json     規劃起飛時刻 → 牆鐘放行時刻 → 實際放行時刻
flight_summary.json       上表的機器可讀版
validation.json  report.json   規劃側的驗證與 preflight 報告
compare.png               左：規劃(虛線) vs 實飛(實線) 航跡；右：高度 vs 模擬時間
sitl_cmdlines.txt         四個 ArduPlane.exe 的完整命令列（證明 --speedup 與參數鏈）
hv_sitl_flight.py hv_sitl_fly.py hv_sitl_artifacts.py   當時用的驅動腳本
git_state.txt             HEAD 與工作樹狀態
```

重現：`python scripts/hv_sitl_fly.py`（會自己規劃、起 SITL、上傳、飛、收資料），
再 `python scripts/hv_sitl_artifacts.py`。
