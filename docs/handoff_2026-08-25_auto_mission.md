# 交接:AUTO 預規劃航線已實作(2026-08-25,續前一份交接)

前一份交接(`docs/handoff_2026-08-25.md`)定調的架構**已經實作、已接線、
已測**,但**一趟都還沒飛過**。這份文件講:落在哪裡、怎麼開、以及第一趟
要量什麼。

---

## 一句話

`auto_mission=True` 之下,ALIGN 段從「GUIDED 逐點下令」改成
**事前規劃的 AUTO 任務航線 → 攻擊發起點盤旋 → `_gate_all` 全員到齊 →
交回 GUIDED 做實時 PN**。預設 **False = 逐位元舊行為**。

    起飛 → 巡航 → 進場      AUTO    飛控自己飛任務航點
    攻擊發起點盤旋           AUTO    NAV_LOITER_UNLIM
    全員到齊閘門             ——      tot_node `_gate_all(self.arr)`
    pushover → 俯衝 → 命中   GUIDED  tot_node 實時 PN(**完全沒動**)

---

## 動了哪些檔

| 檔 | 改了什麼 |
|---|---|
| `sar_swarm_patches/mission_plan.py` | **新檔**。任務項建構 + 上傳協商狀態機(純邏輯,不 import pymavlink/rclpy) |
| `sar_swarm_patches/tot_node.py` | 9 個參數、ALIGN 的 auto 分支、上傳泵、模式切換與讀回、`_LINK_TYPES` + 三個分派分支、CSV 兩欄、逐發重置三格 |
| `sar_swarm_patches/mavros_interface.py` | 新增 `in_auto_mode()`;`_may_change_mode()` 加**第四道閘門** |
| `sar_swarm_patches/master_node.py` | AUTO 進兩處模式白名單 + AUTO 期間整段停手 |
| `sar_swarm_patches/backup_node.py` | RTL/AUTO 判別**移到 `change_speed_async` 之前** |
| `sar_swarm_patches/contract_table.py` | 9 列契約 + ENV_FOR + BOOL_PARAMS + JET_REFERENCE + UNPROVEN + **兩條不變式** |
| `contract_verify.py` | 9 個讀回名 |
| `swarm_ros2_up.sh` / `stot_trial.sh` | 兩條 launcher 都餵(不進批次落差帳) |
| `rep_interactive.sh` | env 明確傳遞 + 航次證據抓 `tot_node` 的 environ、regex 加 `AUTO_` |
| `deploy_build.sh` | `mission_plan.py` 進 NODES 清單與 import 冒煙測試 |
| `tests/test_auto_mission.py` | **新檔,132 條** |

測試:**3730 passed / 0 failed**(基準 3590 + 8 紅)。CSV 現在 **68 欄**。

---

## 怎麼開第一趟

```bash
AUTO_MISSION=True N=2 TRIALS=1 PREFIX=AUTO1 bash scripts/wsl/rep_interactive.sh
```

出貨預設(`swarm_ros2_up.sh`):

    AUTO_IP_R=0          攻擊發起點 = 本機自己的 pushover 環(≈6084 m @ 27°)
    AUTO_IP_ALT=0        盤旋高度 = perch_of()(3100 m)
    AUTO_IP_REACH=5000   抵達判定半徑
    AUTO_LOITER_R=3300   任務盤旋半徑
    AUTO_TRANSIT_R=0     不放進場航點,直接飛 IP
    AUTO_FALLBACK_GUIDED=False   上傳失敗 = **大聲中止**,不靜默退回

⚠️ **先換架構,不要同時換幾何。** `AUTO_IP_R=0` 讓第一趟落在既有環半徑上
(R0 分箱的 6000–7000 格:殘餘中位 23.0 m、3.8% 超標),確認架構會飛之後
再顯式設 `AUTO_IP_R=7500` 去買「0% 超標」那一格。單變數對照。

---

## 第一趟要量什麼

1. **`mission_state` 欄**(CSV 新欄)—— `done:3/3` 才算上傳成功。
   卡在 `count:0/3` = 協商訊息沒回來(先查 MAVProxy 分流是不是被搶)。
2. **`fc_mode_n` 欄**(CSV 新欄)—— AUTO=10、GUIDED=15。
   應該看到 `15 → 10 → 15` 三段。停在 15 = 模式沒切成功(log 會有 ERROR)。
3. **放行距離 R0** —— 這是整個架構要買的東西。應該收斂到 IP 半徑附近,
   而不是像 GMODE1 那樣抽到 3447/3797 m(全母體第 2.0/5.1 百分位)。
4. ⭐ **高度振盪**(`climb_fc` 的零交越週期)—— 前一份交接明寫
   「值得量,不要假設」。AUTO 用**同一套 TECS**,但高度處理走
   mission item + `ALT_SLOPE` 而不是反覆重錨 `prev_WP`。
   比對基準:巡航段指令值域 12.5 m vs 實得 1776/3404 m、週期 15–24 s。
5. **`ack_bad_own`** —— 速度指令那一半的拒收應該消失(AUTO 通過韌體閘門)。

---

## ⚠️ 還沒解決 / 刻意沒做的

* **一趟都沒飛過。** 契約表的 `UNPROVEN` 已經把這一組七顆全部列進去。
  建議值有推導依據(迴轉半徑、R0 分箱),但**推導不是實測**。
* **MISSION 協商經過 MAVProxy 分流鏈路,沒有在真硬體上驗證過。**
  失敗會逾時 + 大聲中止(不會靜默),但第一趟要盯 `mission_state`。
* **weave 的去留**:ALIGN 段蛇形在 `auto_mission` 之下**整段不執行**
  (`_weave_aim_m` / `weave_aim_m` 欄 / `align_early_s` / `align_vreq_mps`
  / `_proximity_check` 的 weave 部分)。**沒有刪除** —— `auto_mission=False`
  仍是預設與退路。節點啟動時會印一行點名哪些欄位這一趟會是空的,
  免得恆空欄被下游讀成「量到零」。
  ⇒ 等 AUTO 架構真的飛順了再決定要不要刪。
* **PERCH 之後完全沒動。** 交接時預設好 `runup_t0` / `perch_T` / `ready`,
  所以 PERCH 會直接走 hold_ring → 環上收斂 → DIVE,陡俯角參數的重送視窗
  (`ring_t0 且 phase=="PERCH"`)保持不變。
* **批次路徑(`stot_trial.sh`)有 `-p` 通路但沒驗過** —— 它會先
  `swarm_ros2_down.sh` 收掉編隊層,AUTO 期間的讓權情境與互動路徑不同。

---

## 這次踩到並修掉的坑(給下一個人)

1. **CRLF 又來了一次。** 突變掃描腳本用 `pathlib.write_text` 還原源碼,
   Windows 預設把 `\n` 翻成 `\r\n` ⇒ **七個檔全變 CRLF**,其中
   `swarm_ros2_up.sh` 會讓整層 WSL 起不來。內容逐字相同、python 照樣
   import 得起來 ⇒ **完全靜默**。抓到它的是腳本結尾那道 md5 比對,
   不是任何一個測試。⇒ 突變腳本一律用 `open(..., newline="")`,
   而且**還原之後一定要比 md5**。
2. **`--timeout=300` 沒裝 pytest-timeout ⇒ pytest rc=4(用法錯誤)。**
   若沒有先跑對照組,26 個突變會**全部**看起來被咬到 —— 26 個假陽性。
   ⇒ 突變掃描的第一步永遠是「未突變時必須全綠」。
3. **註解裡的字串害守衛誤判(第五次)。** ALIGN auto 分支上面的註解寫著
   「用 `_gate_all` 而不是 `len(self.arr) >= self.n`」,而我的鎖斷言
   「`len(self.arr)` 不可出現」⇒ 對正確的程式碼開火。
   ⇒ 負向字串斷言前一律先 `_code_only()` 剝註解。
4. **`_may_change_mode` 的裸 `except Exception: return False` 會把
   「樁少了一個方法」吞成一個看起來合理的 False。** 加第四道閘門時
   四條既有的鎖同時轉紅,而紅的訊息指向閘門邏輯、不指向樁。
   ⇒ 已補一條 `test_the_shim_covers_every_method_the_gate_calls`,
     直接比對「原始碼呼叫了哪些 `self.x()`」與「樁有哪些」。
5. **一行 log 差點有能力打斷交接。** `_auto_handoff` 的散布計算在
   `arr` 為空時 `max()` 會拋例外 —— 生產路徑上 `_gate_all` 保證非空,
   但那是在 timer callback 裡,炸掉的話飛機留在 AUTO、相位卡在 ALIGN。
   ⇒ 已改成 `nan`。backup_node 早就有這條規矩(「記錄永遠不可以拖垮飛行迴圈」)。

---

## 突變驗證(不是「有寫測試」,是「測試咬得住」)

`scratchpad/bite_auto_mission.py`,**26/26 全部被咬到**,還原後七個檔
md5 逐位元相同。涵蓋:兩種閘門退化、第四道閘門被拿掉、CSV 欄位對調、
IP 從設計量退回湧現量、交接沒預設 runup、對殘骸下令、白名單/分派分支
不同步、早到 ACK 被採信、越界請求被忽略、重送無上限、盤旋半徑 0 給預設值、
終態不黏著、靜默退回舊架構、上傳沒完就切 AUTO、不變式沒掛上、
守衛放寬成 `>=`、launcher 漏餵、bool 傳成浮點、master/backup 的 AUTO 守衛、
逐發重置漏格。

---

## 相關記憶

`residual-is-release-range`、`kill-deadlocks-the-sortie`、
`hold-ring-is-unflyable`、`two-launcher-paths-differ`、
`crlf-breaks-wsl-scripts`、`bare-else-in-dispatch`、
`verifying-the-verifier`、`promised-guards-that-do-not-exist`、
`instrument-that-cannot-fire`、`oscillation-is-downstream`、
`sortie-evidence-is-volatile`、`next-arch-auto-then-guided`
