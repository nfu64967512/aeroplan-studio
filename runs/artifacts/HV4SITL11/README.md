# HV4SITL11 — 埠段改到 6760 之後的驗證航次

2026-09-08。**與 HV4SITL10 同一個情境、同一份規劃**，唯一目的是驗證
`scripts/hv_sitl_run.py` 的預設埠段從 5760 改成 **6760** 之後整條流程仍然跑得通。

## 結果

* SITL 實際綁在 **6760 / 6770 / 6780 / 6790**（log 逐行確認），
  四機心跳都帶對 sysid（`A1 心跳 OK (sysid 1, port 6760)` …）。
* **4/4 起飛、飛完、降落**；最高高度 120.1–120.2 m。
* 參數讀回 **68/68**；任務上傳逐項讀回相同（122/140/103/87）。

## 為什麼要改埠

HV4SITL10 第一次 `fly` 在 5760 等 90 s 拿不到心跳。追出來是
**`wslrelay.exe` 同時持有 5760/5770/5780/5790** —— 另一個 session 的 WSL SITL 群
佔著標準 ArduPilot 埠段，我們的 client 連到的是別人的載具。
詳細診斷過程見 `runs/artifacts/HV4SITL10/README.md`。

`--instance N` 是「所有埠加 10·N」⇒ 6760 = instance 100。
現在由 `HV_BASE_PORT`（預設 6760）換算，要回舊行為設 `HV_BASE_PORT=5760`。

⚠ `scripts/hv_sitl_flight.py`（HV4SITL1/2 的基準線腳本）**刻意保持 5760 起**，
它是可重現的基準線，不動。新航次一律走 `hv_sitl_run.py`。

⚠ 這一趟的飛行數據與 HV4SITL10 等價，**不要當成獨立樣本**。
