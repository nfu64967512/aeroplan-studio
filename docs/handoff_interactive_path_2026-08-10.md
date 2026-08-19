# 交接:AeroPlan 互動打擊路徑(右鍵 → STOT)從「完全跑不起來」到「跑完整趟」

2026-08-07 ~ 08-10。適合直接交給新的 session,不需要前文。

---

## 一句話

AeroPlan 地圖右鍵「以此點執行 STOT 打擊」這條**產品路徑**在此之前從來沒有成功過。
查下去是**五個各自獨立、每一個都單獨致命**的缺陷,加上**一整組沒有被鏡像過來的噴射機組態**。

---

## 一、五個缺陷(依發現順序)

### 1. 分流埠被搶 —— 靜默到底

MAVProxy 的 `--out=tcpin` **只餵第一個連上的 client**。
`tot_node` 與編隊層共用 `6960+10i`,而 `master_node`(→6960)、`backup_node`(→6970)
在 launch 檔裡比 tot_node 早起 ⇒ **tot_node 收到 0 bytes**。

    ss -tnpi 實測:
      master_node → 6960   bytes_received:847965      ← 有餵
      tot_node    → 6960   (ss 省略該欄 = 0)          ← 餓死

⇒ `ready_for_target` 恆 false,右鍵按下去毫無反應。
**而且完全靜默**:「等機體定位中」告警只在 `INIT` 印,STANDBY 走的是它前面那條 early return。

* 修:新增 tot_node **專屬埠 `6860+10i`**(現行四路:5860 MAVROS / 6760 AeroPlan / 6860 tot_node / 6960 編隊層)
* 補 `_standby_warn_pos()`(30 s 一次,含查證指令)
* bring-up 改成讀回 `ready_for_target: true`,不再只看「STANDBY 待命就緒」那行
* ⭐ 批次路徑為什麼沒踩到:`stot_trial.sh:425` 交棒前會 `swarm_ros2_down.sh` 把編隊層整個收掉

### 2. payload 沒有 `amsl` —— 海上紅心用垃圾高程

`build_target_payload` 只送 `lat`/`lon`/`approach_from`。海上目標因此回頭查 SRTM,
而水域**沒有資料**(實測散布 17 m)。批次路徑早就用 `TARGET_AMSL=0.0` 釘死。

* 修:payload 加 **Optional** 的 `amsl`;確認框改成**兩顆並列動作鈕**
  (`🌊 海上目標(amsl = 0)` / `⛰ 陸上目標(查地形庫)`),預設鈕仍是取消
* `_selfcheck_roundtrip` 連 amsl 一起對帳;`nan`/`inf` 一律擋(`json.dumps(nan)` 產生 `NaN`,不是合法 JSON)
* ⚠️ 這個模組在此之前**完全沒有測試**

### 3. 觸水終結誤殺 150 m/s 巡航中的飛機

    agl = relative_alt − z_g

`relative_alt` 每筆跟著飛控當下的 HOME 走;`z_g` 在 `i not in self.zg` 保護下**只算一次就凍結**。
飛控重設 HOME(GPS 海拔 +32.0 → −0.31)⇒ agl 瞬間變 −0.2 ⇒ 強制 disarm。
`kill_confirm_n=2` 在 20 Hz 只有 0.1 秒,而基準漂移是**持續偏移**、不是單筆跳點。

* 修:釘死 AMSL 的目標改在 AMSL 空間算 `agl = alt_amsl − target_amsl`,
  兩者取自**同一則** GLOBAL_POSITION_INT ⇒ 基準跳動自我抵消(不是啟發式否決)
* 基準穩定時新舊兩式**逐位元相同** ⇒ 既有海上資料不受影響
* 終結訊息加印**地速**(只印不擋 —— 真觸水發生在 ~140 m/s 的俯衝末端)

### 4. 兩層同時下令 —— 飛機被壓在 50–80 m

互動路徑沒有「交棒」這一步,編隊層與 tot_node 同時活著:

    tot_node   : ALIGN,飛 48 km 向西、爬到 perch
    master_node: `master -> leg 2 (…) @ 80 m` + `leader slowing to 10 m/s`
    backup_node: `UAV2 -> V-tail slot -> (…) @ 50 m`(1 Hz 重發)

編隊層靠**重發頻率**贏 ⇒ 高度 122→74 m,飛機在起飛點繞圈。

* 修:**軟交接**(使用者定調)。新增 `sar_swarm_patches/strike_authority.py`,
  三個節點共用。訂閱 `/swarm/tot_status`,phase ∈ {ALIGN,PERCH,DIVE} ⇒ **停止下令**,
  但**保持存活**繼續發遙測/心跳/formation_status;回 IDLE 或**逾時 5 s** ⇒ 自動收回
* ⚠️ 失效方向刻意選「恢復下令」—— 逾時繼續讓權會讓飛機沒有任何控制器
* 備長打擊期間**暫停接管職責**(使用者定調);收回時要**重置 `last_hb`**,否則當場誤判心跳消失
* 文獻:NASA MOMU TP-2018-219875 的 handover 協定;arXiv 2606.26315 的巡航/終端權限分離

### 5. 雙路送 cmd 178 殺死 MAVROS

`mavros_interface.change_speed_async` 同時走直連與 MAVROS 送**同一個** `DO_CHANGE_SPEED`
⇒ 飛控回兩個 COMMAND_ACK ⇒ MAVROS 對同一個 promise `set_value` 兩次 ⇒
`std::future_error: Promise already satisfied` ⇒ SIGABRT(exit −6)。

後果不只是少一個節點:`master_role` 會抱著最後一筆定位以 10 Hz 繼續廣播,
僚機飛向永遠不動的 slot,**全程 WARN/ERROR 計數 = 0**。

* 修:**一次只准走一條路**。直連是權威(T1 實測),MAVROS 那發只是觀測探針
  ⇒ 改由 `_speed_report` 記錄。`SPEED_DIRECT=0` 仍可切純 MAVROS 做 A/B

---

## 二、一整組沒有鏡像過來的噴射機組態

`swarm_ros2_up.sh` 的 tot_node 參數**逐條抄自 `stot_trial.sh` 的預設**,
而**批次航次實際跑的是 `run_west.sh` 覆蓋後的值**。
⇒ 互動路徑忠實複製了一組**噴射機改裝後沒有任何一趟航次用過**的泡棉機基準。

⚠️ `test_tot_standby.py` 的「兩邊一致」迴歸鎖**擋不住這個**——它比的是 stot_trial 的預設。
已改成「run_west 有覆寫的比 run_west,其餘才比 stot_trial」。

| 參數 | 泡棉 | 噴射(run_west / TWO1) |
|---|---|---|
| VMIN | 45 | 95 |
| VMAX | 100 | **150**(見下) |
| PERCH_CLIMB_V | 0(⇒65) | 100 |
| WEAVE_PERIOD | 22 s | 120 s |
| PN_VMAX / DIVE_SPEED | 135 / 140 | 210 / 210 |
| PERCH_ALT / TALT | 400 | 3100 |
| DIVE_ANGLE | 10° | 27° |
| R_FAN / REACH | 4000 / 260 | 35000 / 2400 |
| GATE_R | 1200 | 4800 |
| AIRSPEED_MAX(FC) | 165 | 230 |

### ⚠️⚠️ 「157 m/s」是地速,不是 EAS(我踩過一次)

`run_west.sh:68`:「157 m/s 地速 @3100 m = **EAS 135**」;`:72` 就在警告
「餵 157 會設計成 3100 m 的 183」。機體照 `--cruise 135` 反推。

    v_plan = plan_frac × vmax = 0.9 × vmax
      0.9 × 175 = 157.5 EAS  ✗ 高於天花板 17%
      0.9 × 150 = 135.0 EAS  ✓

把 157 當 EAS 的後果**不是飛慢一點**:全隊卡在 `mode="max"` 飽和點 ⇒
T_arr 永遠達不到 ⇒ **共同抵達的同步迴路變成開迴路**,散布凍結。

    3100 m:EAS 135 ⇒ 地速 ≈ 157
     400 m:EAS 135 ⇒ 地速 ≈ 138

### 幾何是耦合的,不可以只改一項

    環半徑 = (perch − dive)/tan(角)
      3100/tan(27°) =  6084 m   ← TWO1 這一組
      3100/tan(10°) = 17575 m   ← 只改高度的下場
       400/tan(10°) =  2269 m   ← 泡棉尺度

`gate_r/環 = 4800/6084 = 0.789`(u1-descent-authority 的判別比)。
⚠️ `HALF_ANGLE` 留 25.0:TWO1 的 5.0 是 n=2 特例,n=6 收太小 ALIGN 收不完。

### ⚠️ AIRSPEED_MAX:兩個檔案互相矛盾

* `swarm_sitl_up.sh` 註解:W1 實測拉到 220 之後 UAV6 climb 逾時 ⇒ 取 165
* `run_west.sh:86` 匯出 **230**,而 TWO1 走的就是 run_west

而且耦合:`DIVE_SPEED=210` 在 165 之下會被**整發拒收**。
⇒ 採「與唯一達標組態一致」= 230,**W1 的反證原文保留**(測試鎖著不准刪)。

---

## 三、機體身分守門(讀回式)

`swarm_sitl_up.sh` **從不自己設 JET=1**(`:64` 是 `${JET:-0}`),
只有 `stot_repeat.sh` / `stot_kpi_loop.sh` 會設。
照文件的 CMD1 裸跑 ⇒ 原廠泡棉機 ⇒ 上面每一發速度指令被 `MAV_RESULT=4` 拒收,**完全靜默**。
`JET=1` 但 `JET_CDP` 空 ⇒ 噴射模型 + 原廠阻力,天花板只有 EAS 98.5,**只會慢不會錯**。

* 修:`swarm_ros2_up.sh` 開跑前讀 `pgrep -a arduplane` 的 `--model` 比對
  (`*-jet*` 且 `*:*json`),不符即中止並印出可貼的重跑指令
* **不建立任何 MAVLink 連線** —— 不搶分流埠
* 只在 `TOT_ENABLE=1` 時檢查(PRESET=tight/max 本來就要泡棉機)
* 逃生門 `AIRFRAME_FORCE=1`,而且**訊息裡會印出來**

---

## 三·五、重複航次(2026-08-10 下午)又挖出三件事

### ⑥ 觸水終結缺少「曾經飛起來過」這個物理前提

REP1:起飛後 25 s 下目標,兩機在 **6 秒內**全被終結,而當下地速 95–106 m/s:

    t=0.0 uav=1 alt=-30.252 gs=100.4 agl= 1.8
    t=0.0 uav=2 alt=-36.089 gs=105.8 agl=-4.0

⚠️ 真因**不是**基準不同步(缺陷 3 已修那條),而是**飛控的高度估計本身是壞的** ——
剛 arm/起飛的幾十秒 EKF 原點還在沉澱,`relative_alt` 回報 −30 m。
`agl` 忠實反映了一個錯的輸入,再多的基準對齊都救不了。

⇒ 補上閂鎖:量到 `agl ≥ kill_arm_agl`(50 m)之後才為該機解除保險。
**是閂鎖不是計時器** —— 不用「起飛後 N 秒」這種會漂的常數。
不會擋掉正確的終結:AP4 那架飛了 10 分鐘才掉下去,閂鎖早就解除。

### ⑦ 編隊層對噴射機**既無速度權限、也爬不上去**

實測(起飛後逐 5 s 量 180 s,桃園跑道 +32 m AMSL):

    t(s)   UAV1_rel   UAV2_rel   UAV1_amsl   UAV2_amsl
       0     65.787     -2.003      105.27       47.52
      60    -17.658    -22.113       32.83       29.46
     180     -8.265    -14.664       42.22       36.33

三分鐘下來兩機一直在跑道高度浮沉。兩個獨立成因:

* **(a)** 編隊層下 `NOMINAL 14 / WAIT 10 / CATCH 20` m/s,而噴射機
  `AIRSPEED_MIN = 25` ⇒ 每一發 `DO_CHANGE_SPEED` 被 `MAV_RESULT=4` **整發拒收**。
  這是 catch-speed-rejected-by-fc 的**另一端**(那次太快 157>150,這次太慢)。
* **(b)** backup/wingman 以 1 Hz 重發 `DO_REPOSITION`,每發都
  `set_target_altitude_current()` 重錨高度 ⇒ 高度需求累積不起來。
  tot_node 的 `_goto` 用航點鎖存修過同一個病,**編隊層沒跟上**。

⚠️⚠️ **這推翻了我先前對 AP5「②編隊 ✅」的記錄。**
交接(軟讓權)成立是真的,**編隊**在互動路徑從未被真正證明。
打擊本身不受影響 —— tot_node 接手後自己爬(AP5 到 3100 m)。
⚠️ 尚未修:牽涉編隊層的速度組態與航點鎖存,要單變數對照。

### ⑧ 命中高度與 CSV 的 agl 分屬兩套基準

`hit_alt = pos − _zg`(HOME 相對)vs `agl = _agl()`(AMSL,8/7 起)。
HOME 重設實測平移 **32.3 m**,而 `impact_alt_ok` 門檻只有 **30 m** ⇒
真落地會被寫成「高空掠過」,反向跳變會把 40 m 掠過寫成「全員命中」。
而且 `_metrics_frozen` 之後**永不更新**。

⚠️ AP3/AP5 沒被污染是**運氣**:兩趟的 HOME 重設都在命中之後約 1 秒(撞海觸發)。
⇒ `hit_alt` 改走 `_agl(i)`。HOME 穩定時兩式逐位元相同,既有資料不受影響。
順帶更正一句我 8/7 造成的錯誤註解(「agl 和命中高度同基準」當時已不成立)。

## 四、工程教訓(跨專案可移轉)

1. **任何「A − B」的判定,A 與 B 必須來自同一個取樣。**
   本專案已在同一形狀上摔過三次(環半徑漏算地形、命中只看水平距、agl 基準)。
2. **同時活著的兩個消費者,永遠不可以共用一個 `--out=tcpin` 埠。**
3. **「起了」不等於「生效」。** 待命宣告 ≠ 接得了目標;要讀回真正的就緒旗標。
4. **Windows 上用 Python `write_text` 改 `.sh` 會寫成 CRLF** ⇒ WSL bash 整層起不來,
   而錯誤訊息(`pipefail: invalid option name`)指向一個語法完全正確的行。
   改 `.sh` 一律用會保留換行的編輯方式;事後用 **WSL 的** `bash -n` 驗。
5. **`pgrep -f <name>` 會打中任何「命令列提到該名字」的行程** ——
   包含你自己的 log 監看指令。要收緊到只有真行程才有的形狀(這裡是 `--ros-args`)。
6. **單趟散布的雜訊底線是 ±10 s**(B0 同組態三趟:17.5 / 30.7 / 10.9)。
   差距小於 ~20 s 的單次比較**沒有診斷力** —— 不要拿一趟的散布去歸因。
7. **⭐ 不要拿「系統做不到的事」當門檻。** 這一輪我連犯三次:
   固定 25 s(EKF 還沒收斂)、`rel_alt ≥ 120`(編隊層只到 80)、
   `rel_alt ≥ 60`(編隊層根本爬不上去)。專案裡同形狀的前科:
   `reach=260` 對 35 km 扇形、`perch_alt_gate` 要 360 m 而飛機卡在 30–80 m。
   **設門檻之前先量一次上限**;而且要**開跑前就擋**,不要等逾時才發現。
8. **`bash -n` 只驗語法** —— 未定義的函式、空變數它都看不到。
   我把函式改名卻忘了改呼叫點,浪費了三趟 × 180 s,而語法檢查全綠。
   ⇒ 腳本自己在開跑前 `declare -F` 驗函式、驗要印的變數非空;
   而且**要實測這兩個檢查真的擋得住**(故意製造錯誤跑一次),不是寫了就算。
9. **同一個症狀可以有多個獨立成因。** 「飛行中被判觸水」查到第一個
   (基準不同步)就收手,結果第二個(輸入本身是壞的)在下一輪原地復發。
   修完一個之後要問:**還有什麼會讓這個輸入是錯的?**

---

## 五、迴歸鎖

    tests/test_link_port_exclusivity.py    22   分流埠獨佔 + 批次先收編隊層的順序
    tests/test_rclick_strike_datum.py      34   amsl 三層引號往返 + 海/陸兩顆鈕 + N 感知
    tests/test_touchdown_kill.py           37   含「舊式重現誤殺」與「基準穩定時逐位元相同」
    tests/test_no_duplicate_command.py     10   一次只准走一條路
    tests/test_strike_authority.py         46   軟交接(含逾時收回、備長心跳重置)
    tests/test_jet_config_parity.py        44   噴射組態 + 環半徑算式 + 機體守門
    ────────────────────────────────────────
    全套                                 1182   全綠

## 六、實測進展

| 航次 | 結果 |
|---|---|
| AP1 | UAV2 於 t+9 s 被**誤殺**(agl 基準漂移)⇒ 作廢,但定位出缺陷 3 |
| AP2 | 兩層同時下令,飛機被壓在 50–80 m ⇒ 作廢,但定位出缺陷 4 |
| AP3 | **首次跑完整趟**:2/2 命中、誤差 23.1/20.9 m、散布 7.20 s、抵扇形 0.1 s |
| AP4 | 我在相機上耗太久,UAV2 在 80 m 巡邏時掉下去(**觸水判定是正確的**)⇒ 作廢 |
| AP5 | 3100 m 噴射剖面:抵扇形散布 **2.8 s**、ALIGN 最近接近 **3363 m**(舊 348 m) |

⚠️ AP3 的四項門檻:抵達散布 ✅、編隊 ✅、命中 23.1 m ✗(門檻 10)、散布 7.20 s ✗(門檻 3)。
但**單趟散布沒有診斷力**(見教訓 6),要判定改動有效必須同組態跑 ≥3 趟。
