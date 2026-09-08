# HV4SITL6 — speedup 10：**失敗的航次**（四台 SITL 全部死掉）

2026-09-07 · ArduPlane 4.8.0-dev QuadPlane SITL ×4 · **speedup 10** · 分支 `quad_swarm`
情境 `gui_uniform_1a3b`（與 HV4SITL3 完全相同，**只換 `HV_SPEEDUP=10`**）

## 結論：這個倍率在這台機器上跑不起來，而且失敗的樣子會騙人

規劃、參數讀回、任務上傳全部正常（與 HV4SITL3 逐項相同：68/68、109/121/75/70）。
但一進監看迴圈就開始壞：

| 徵狀 | 細節 |
|---|---|
| 兩架**根本沒起飛** | B2、A1 的 `set_mode("AUTO")` 被**拒絕**（log 印「切換被拒」），高度全程 0.0 m |
| 一架卡住 | B4 爬到 53.5 m 之後高度就再也不變 |
| 一架凍結 | B3 爬到 140.3 m 之後凍結 |
| 連線 | 大量 `Connection reset or closed by peer on TCP socket` |
| 行程 | 四個 ArduPlane 全部自己死掉（dataflash 都停在 03:19，各約 5 MB） |
| 放行時刻漂移 | B2 規劃牆鐘 36.0 s，實際 76.2 s（監看迴圈被餓到） |

`flight.csv` 四個檔都是 0 列（緩衝還沒 flush 就被收掉），所以**證據在 console log**，
不在 CSV。log 在 `runs/artifacts/HV4SITL6_console.log`。

## 為什麼這一趟值得留著

1. **失敗不會自己說話。** 兩架沒起飛、兩架凍住，但腳本沒有丟例外，
   如果不看高度欄，這一趟看起來就只是「跑得比較久」。
   `set_mode` 被拒有印出來是因為腳本有檢查回傳值 —— 那一行是必要的。
2. 這給了「speedup 5 是本 repo 唯一有 VTOL AUTO 成功紀錄的倍率」一個**具體的失效樣貌**，
   而不只是一句傳聞。

## ⚠ 這一趟**沒有**證明「speedup 10 一定不行」

跑這一趟的時候，機器上同時有六個代理人在跑 Python 規劃掃描，CPU 是滿載的。
所以正確的說法是「**在這個負載下**，speedup 10 會讓四台 SITL 死掉」，
不是「speedup 10 本身不可行」。要分辨的話得在閒置機器上重跑一次。

## 重現

```bash
HV_SPEEDUP=10 HV_TRIAL=HV4SITL6 python scripts/hv_sitl_run.py fly gui_uniform_1a3b
```

## 附帶驗證到的一件事

收尾用的是新的 `stop_ours()`（只殺自己起的 PID）。它正確地回報
「已終止 0 個本次的 ArduPlane 行程（機器上另有 1 個不是我們起的，沒有動）」——
四個都已自己死掉，而別的 session 的那一個**沒有被誤殺**。
舊的 `stop_all()` 會把它一起 taskkill 掉。
