# 差分測試的凍結原版（不要改）

`deconflict_pre_speedup.py` 是 vendored 修改 #13（去衝突加速，2026-10-21）**之前**的 `hv_dccpp/deconflict.py`，
逐位元複製（含 #11 的殘留時收尾重排）。`tests/hv_dccpp/test_deconflict_speedup_identity.py` 把它以
`hv_dccpp._deconflict_pre_speedup` 的名字載入（相對匯入仍指向 `hv_dccpp`），在同一個程序裡拿它跟現行版
比 `check_separation`／`repair`／`_global_reschedule`／`shift_plan`／`_escape` 的輸出，逐欄位、逐位元。

為什麼需要它：驗證器（`hv_dccpp.sim.validate`）也呼叫加速後的 `check_separation`，只跑驗證等於自己驗自己。

- 測試以「換行正規化成 LF 後的 md5」鎖住這個檔案，被改到就會紅。
- **以後要改 `deconflict.py` 的行為**（不是加速、會改結果的修改）：同樣的改動也要套到這份檔案上，
  並更新測試裡的 md5 與 `docs/hv_dccpp/VENDOR.md` 的說明；否則差分測試會（正確地）報不一致。
- 純加速的修改不要動這份檔案：差分測試就是用來證明「結果沒變」的。
