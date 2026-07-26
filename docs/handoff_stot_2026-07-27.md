# 交接文件 — 蜂群 STOT 紅心精準命中(2026-07-27)

給接手的 AI / 工程師。這份文件是自足的:看完應該能直接接著跑,不需要回頭讀對話。

---

## 1. 這個專案在做什麼

**AeroPlan Studio**(Windows PyQt6 地面站,`python main.py`)+ **WSL ArduPilot SITL**
+ **ROS2 `sar_swarm`**,做一場六機固定翼蜂群表演:

> 彈射起飛 → 雁型編隊長程巡航 → 目標前四散進扇形 → STOT 時間對齊
> → 對正紅心徑向 → 終端俯衝 → **六機同時命中紅心**

**兩管道架構(不可違反)**
- ① **MAVLink / ArduPilot** = 單機機體控制(Mission Planner 做得到的:模式、航點、速度、參數)
- ② **ROS2 / sar_swarm** = 多機協同(MP 做不到的:編隊、時間對齊、扇形指派、俯衝時序)
- ② 只能經 ① 下達,不可繞過。詳見 `docs/swarm_architecture.md`。

---

## 2. 驗收標準(使用者訂的,目前**尚未達成**)

| 項目 | 門檻 | 目前最佳實測 | 狀態 |
|---|---|---|---|
| 命中高度 | ≈ 0(離紅心地表) | **0, 0, 0, 0, 0, 0** | ✅ 已達成 |
| 水平誤差(六機**全部**) | **≤ 5 m** | 5.8 – 29.1 m | ❌ |
| 命中時間散布 | **≤ 3 s** | 3.30 s(最佳單趟) | ⚠️ 差一點 |
| 成功率 | **≥ 99%** | **尚未量到** | ❌ |

> ⚠️ **沒有任何一趟同時滿足 ≤5m 與 ≤3s。** 不要在數字出來前宣稱達標。

**使用者要求**:持續測試 → 自己診斷沒過的原因 → 查資料/論文 → 修 → 再測,循環到達標才交付。
使用者同意「先取 10 次測試」作為第一批樣本。

---

## 3. 目前狀態(2026-07-27)

- 分支:`fix/dccpp-geofence-2d`(**不要 checkout main,main 是舊的**)
- 最後 commit:`9bcd985`
- `tot_node.py` 806 行,裝在 `~/fixwing_ws/src/sar_swarm/sar_swarm/`
  (**egg-link 安裝,改完不用 colcon build,下次 `ros2 run` 就生效**)
- 鏡像在 repo:`scripts/wsl/sar_swarm_patches/tot_node.py`(改完記得同步兩邊)
- `pn_enable` 目前 **False**(PN 終端導引已實作但尚未驗證成功,見 §5)

---

## 4. 已驗證有效的機制(不要回頭改壞)

| 機制 | 為什麼需要 |
|---|---|
| **每機各自的地表高度** | 每個 SITL instance 有**自己的地形快取**,對同一個紅心座標回報的地表高度差到 **12.4 m**(實測 −42.91 / −48.63 / −55.31 / −52.12 / −44.94 / −38.72)。用單一常數套全隊,以 **1.8 m 誤差/1 m 地表誤差**放大 → 光這一項就 13 m。**正確算法**:對紅心座標送 `TERRAIN_CHECK`,用 `terrain_height − home_amsl`。**不要**用 `rel_alt − current_height`(那是飛機正下方的地形,會隨飛行一直變)。 |
| **俯衝航點鎖存** | ArduPlane GUIDED 高度走 `prev_WP → next_WP` 線性斜坡;每 tick 重發 `DO_REPOSITION` 會把 `prev_WP` 重設成當前位置,下降需求永遠累積不起來。整段俯衝只發一次航點。 |
| **`DO_REPOSITION` param3 要給小半徑** | param3=0 → 用 `WP_LOITER_RAD`(180 m),飛機被捕獲進大圈永遠收不了。給 5–30 m:誤差從 124–140 m 降到 18–54 m。 |
| **航點高度要放在下滑線上** | 用「飛機當下距離」算高度 = 命令「保持現在高度飛到目標」= 幾乎平飛。要算**瞄準點自己位置**的線上高度。 |
| **追蹤率補償 `track_frac`** | 機體只做到指令俯角的 ~0.5–0.65 倍(實測 45°→28°、25°→15.6°、15°→10°)。指令要更陡才會真的落到地面。 |
| **STOT 前先對正徑向 + 鎖存** | 對正閘門要**鎖存**,不能每 tick 重判 —— 否則形成極限環(飛出去對正→內切→飄掉→又被趕出去),實測 15 分鐘只有 2/6 過閘門。 |
| **錯開放行(staggered release)** | 靠減速等不到同時命中:近機需要的速度會低於噴射可持續下限 45 m/s;把 vmin 壓到 38 又會 mush。改成近機在環上多等 `(dmax−d)/pace` 秒,之後全隊同速飛。 |
| **`dive_pace` 要用實際可達地速** | 配速 42→散布 3.30 s,48→4.62 s,62→11.22 s。配太樂觀 `t_rem` 立刻歸零、時間對齊失效。實際俯衝地速約 40 m/s。 |
| **雁型間距要大於機體站位精度** | 噴射在 52–65 m/s 迴轉半徑 ~275 m,對定點只能繞不能停,slot 誤差有 ~205 m 硬地板。間距用 **600 m**(不是 300),誤差/間距才會落在 0.34。 |

---

## 5. PN 終端導引 — 已實作、尚未驗證成功

研究(文獻 + ArduPilot 原始碼 + 600 組蒙地卡羅)結論:

- **航點導引本質到不了個位數公尺**:`GUIDED_CHANGE_ALTITUDE` 實際只給到需求下沉率的 1/30
  (`TECS_HDEM_TCONST=3.0` × 10 Hz 節流);L1 時間常數 `ζ·T/π ≈ 4.06 s` 終端來不及。
- **PN 導引殘差 p99 = 0.60 m**(用實測機體參數模擬)
- **但 EKF 定位誤差 p99 = 2.15 m**(對比 SITL 真值,n=25419)← **不可消除的地板**
- **合計 RSS p99 ≈ 3.0–3.2 m** ⇒ ≤5 m 可達但不寬裕,99% 只能算 **90–97%**
- ⚠️ **「miss 用什麼量」差 2 m**:用遙測量 EKF 誤差會抵消(0.5–1.5 m),用 SITL 真值量則疊加。
  **先跟使用者講好用哪個**。

**實作位置**:`tot_node.py` 的 `_pn_cmd` / `_pn_step` / `_att_target` / `_geom`,DIVE 內
`d <= gate_r` 時交接。已驗證會 engage(6/6 直連、6/6 交接 @ d≈1197 m)。

**實測結果(12 趟,`pn_enable=True`):0 通過,三軸全面比航點基準差** ——
誤差 21–24 m、命中高度 37–53 m、散布 14–24.6 s、9/12 逾時。

**事後查出兩個真因(都已修,但修完還沒重測)**:
1. `_pump` 只讀分流線(6960),但 50 Hz 串流是下到**直連飛控線**(5762)→ 攻角估計拿到舊值/0
   → 俯仰指令全錯。(commit `6d7ce14`)
2. **PERCH 死結**:「退回扇形 slot 重新對正/爬升」那條分支用 `vmin`(45 m/s)飛,
   22 kg 噴射撐不住高度 → 五機停在 4052–4172 m 外、高度掉到 216–306 m,
   永遠達不到 0.9×perch_alt=360 m 的閘門,卡死 20 分鐘。
   **那 9 次「逾時」其實是這個,不是導引失敗。**(commit `9bcd985`)

👉 **下一步就是:把 `pn_enable` 打開重跑,看修完後 PN 是否終於贏過航點基準。**

---

## 6. 怎麼跑(照抄即可)

```bash
# 0) 環境:WSL Ubuntu-22.04。腳本都在 ~/fixwing_ws/sitl_run/
#    ⚠️ 一律 JET=1(plane-jet)。不要換回泡棉機,除非使用者明講。

# 1) SITL(彈射 + 噴射)。用前景常駐方式跑,不要 nohup &(會被回收)
cd ~/fixwing_ws/sitl_run
JET=1 LAUNCH=catapult AIRSPEED_MAX_CAP=150 LOC=TAOYUAN bash swarm_sitl_up.sh
# 等 ~100 s,確認 arduplane=6;bridge 還要 ~45 s 做完 param-FTP 才接受 client

# 2) 彈射起飛
python3 jet_catapult_launch.py 5 90

# 3) 四散到扇形
python3 jet_rescatter.py 300

# 4) STOT + 俯衝(參數見 §7)
ros2 run sar_swarm tot_node --ros-args -p tlat:=25.1406 -p tlon:=121.2325 ... 

# 5) 自動重複試驗 + KPI 計分(每趟含重啟 SITL,約 30 分鐘/趟)
PN=True bash stot_kpi_loop.sh 10
# 結果:/tmp/stot_kpi.csv(每趟 spread / max_miss / max_alt / pass)
```

**一趟為什麼要重啟 SITL**:命中即消耗(`expend_on_impact`),機體留在地面拉不起來,
第二趟湊不齊六機。

---

## 7. 目前最佳參數組(航點基準線)

```
-p tlat:=25.1406 -p tlon:=121.2325 -p talt:=400.0 -p approach_from:=180.0
-p r_fan:=4000.0 -p half_angle:=25.0 -p vmin:=45.0 -p vmax:=100.0
-p plan_frac:=0.9 -p margin:=40.0
-p perch_alt:=400.0 -p ground_alt:=-48.0 -p dive_alt:=0.0 -p dive_layer:=12.0
-p dive_angle:=10.0 -p auto_ring:=True -p perch_alt_gate:=0.9 -p align_tol:=40.0
-p track_frac:=0.50 -p dive_radius:=5.0 -p dive_pace:=40.0 -p dive_spdweight:=0.0
-p overshoot:=300.0 -p switch_r:=300.0 -p dive_speed:=140.0
-p reach:=260.0 -p layer_m:=25.0 -p perch_reach:=260.0 -p impact_reach:=25.0
-p link_base:=6960 -p link_stride:=10
```
PN 要開再加 `-p pn_enable:=True -p gate_r:=1200.0`。

---

## 8. ⚠️ 踩過的坑(不要再踩一次)

| 坑 | 症狀 | 解 |
|---|---|---|
| `set -u` + ROS2 | `AMENT_TRACE_SETUP_FILES: unbound variable`,腳本第一趟就中止 | 腳本**不要**用 `set -u` |
| ROS2 double 參數 | 送 `dive_pace:=40` 被判 INTEGER **直接拒收**,節點起不來 | 一律補小數點 `40.0` |
| `SET_MESSAGE_INTERVAL` | 填 20000 µs 回 `MAV_RESULT_DENIED`(cap = loop/800 = 25 ms),然後你默默用 5 Hz 跑 50 Hz 的律 | 填 **25000** |
| 分流搶速率 | 同一道指令,6960 分流上 uav3 只有 5 Hz、uav1 有 50 Hz | 高速狀態走**直連 5762+10i** |
| 分流「先連者得」 | AeroPlan 佔 6760 後,tot_node 連同一埠只拿得到心跳、沒有位置 → 永遠停在 INIT | 三方分家:5860 腳本 / 6760 AeroPlan / 6960 tot_node |
| `pkill -f` 自殺 | 自己的 cmdline 含同樣字串,把自己的 shell 殺掉(exit 15) | 用 `pkill -x`,或 `[j]et_...` 括號技巧,且**別和啟動指令寫在同一行** |
| `nohup ... &` 被回收 | 背景任務印一行就消失 | 用 Bash 工具的 `run_in_background` 跑**前景**指令,或 `setsid` + 尾端 `sleep` |
| JET=1 蓋掉彈射 | `LAUNCH=catapult` 選的模型被 JET 覆寫,彈射器靜默消失 | 合成 `plane-jet-catapult`(SIM_Plane 用獨立 strstr) |
| JET 缺加速度校正 | `PreArm: 3D Accel calibration needed`,六機全解不了鎖 | `--defaults` 要先疊 `models/plane.parm`(只有它有 12 行 `INS_ACC*`) |
| 彈射觸發 | RC override 沒用(實測 servo7 一直是 1000) | `SERVO7_FUNCTION=0` + `MAV_CMD_DO_SET_SERVO` ch7=2000 |
| `/tmp` 被清空 | WSL 重啟後所有飛行記錄和 SITL 消失 | 重要資料別只放 `/tmp` |
| computer-use 搶前景 | 每次點擊前 TextInputHost/desktop 搶前景 | 點擊前先 `Stop-Process TextInputHost` + Win32 `SetForegroundWindow` |

---

## 9. AeroPlan 前端(已完成)

- **紅心靶**:`main.html` 的 `setTotGate()` 畫同心紅環 + 靶心 + `❤ 紅心 (STOT)` 標籤;
  Cesium 地圖**右鍵**即畫(實測落點誤差 ~5–30 m)。預設幾何已對齊噴射尺度(扇形 4 km / ±25°)。
- **命中爆炸動畫**:`explodeAt()` — 火球(白→黃→橙→紅,邊擴邊淡)+ 地面震波環 + 命中標籤,
  用 `CallbackProperty` 逐幀動畫、結束自動移除。
  **會自動觸發**:`update_uav_position()` → `check_bullseye_hit()`,UAV 進紅心 60 m 內放一發,每機一次。
- **整場動畫**:`scripts/wsl/_show_anim.py`,吃 `/tmp/show_track.csv`(巡航)+ `/tmp/tot_node.csv`(STOT/俯衝),
  輸出俯視 + 高度剖面的 GIF。

---

## 10. 建議的下一步(依序)

1. **開 `pn_enable=True` 重跑** —— 兩個擋路 bug(`6d7ce14`、`9bcd985`)都修了但**還沒重測**。
   先跑 2–3 趟確認 PN 有沒有贏過航點基準(基準:誤差 5.8–29.1 m、高度 0、散布 3.6 s)。
2. **若 PN 仍不如基準** → 逐項比對飛行資料驗證 PN 的每個中間量(λ̇、σ̇、α 估計、nz 夾制、凍結),
   不要整條律一起猜。研究的完整規格在 workflow 輸出(§11)。
3. **若 PN 贏了** → 跑 10 趟量成功率,再依失敗類型調 `N_lat` / `N_vert` / `pn_lead`。
4. **先跟使用者確認 miss 的量測基準**(遙測 vs SITL 真值),兩者差 2 m,會直接決定能不能宣稱 ≤5 m。
5. **統計誠實**:10 趟全過只能支持約 74% 的信心下界;要撐 99% 需要約 300 趟(≈65 小時)。
   回報累計數字,不要外推。

---

## 11. 參考資料位置

- 研究工作流完整輸出(含 PN 方程式、增益掃描、誤差預算、ArduPilot 原始碼證據):
  `C:\Users\joy46\AppData\Local\Temp\claude\...\tasks\wb45q4cyu.output`(約 163 KB)
- 前一輪研究(俯衝幾何 / 雁型):`...\tasks\wccia4xqt.output`
- 架構文件:`docs/swarm_architecture.md`
- 編隊規格:`docs/formation_v6_tight_coupled_spec.md`
- 前一份交接:`docs/handoff_swarm_2026-07-24.md`
