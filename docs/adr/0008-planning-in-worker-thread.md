# ADR-0008: 覆蓋規劃移入背景執行緒（先只搬 HV-DCCPP 這條）

**狀態**: Accepted
**日期**: 2026-09-06
**決策者**: @james zheng

## Context / 背景

`ui/main_window.py` 的 `_on_dccpp_coverage_requested` 是一個 435 行的
`try:` 區塊：驗證邊界點 → 組 areas/drones → **同步**呼叫
`SwarmCoordinator.plan_coverage_dccpp` → 300 行後處理（NFZ 重繞、避撞、
轉 swarm_raw、畫地圖、統計面板）。全程跑在 GUI 執行緒上，唯一的讓步是呼叫**前**
一次 `QApplication.processEvents()`。

`core/base/planner_base.py` 有 `progress_callback` 與 `cancel()`，但無人提供也無人檢查；
`config/settings.py` 的 `enable_multiprocessing` / `max_workers` 是死設定。
全樹只有 `mission/sitl_link.py` 等三個 QThread。

規劃耗時實測：2 機 7 條帶 **1.2 s**（可忍受）；8 機 107 條帶的山區情境
**數十秒** —— Windows 會把標題列改成「沒有回應」。

## Decision / 決策

1. 把 `_on_dccpp_coverage_requested` 的後處理（原 5246-5562 行）原封不動切成
   `_display_dccpp_result(dccpp_result, *, params, corners_snapshot, ...)`，
   只改兩處：兩次 `list(self.corners)` 改讀規劃前拍的 `corners_snapshot`，
   以及自己帶一個 try/except（讓「規劃失敗」與「顯示失敗」不再互相掩蓋）。
2. 新增 `ui/workers/planning_worker.py` 的 `HvPlanningWorker(QThread)`，
   寫法比照 `mission/sitl_link.py:169 SITLLink`：對外只走 pyqtSignal、
   失敗有自己的信號、worker 內不碰任何 widget。
3. **只有 HV-DCCPP 分支走背景執行緒**；既有 DCCPP 維持同步，行為完全不變。
4. 呼叫端若需要同步讀回 `self._dccpp_result`，必須明寫 `run_async=False`。
   目前 `_on_vtol_recon_plan` 就是這種呼叫端。

## Alternatives Considered / 考慮過的替代方案

1. **兩條路徑一起改成非同步**。風險不成比例：`_on_vtol_recon_plan` 呼叫完
   立刻讀 `self._dccpp_result`，改成非同步會**靜默**讀到上一次的結果；
   而既有 DCCPP 的耗時本來就在 1 秒等級。
2. **丟到子行程（multiprocessing）以支援「按下就停」**。`plan_mission` 沒有取消點，
   子行程 `terminate()` 是唯一乾淨的即時取消。但 `terrain_func` 是 bound method
   不可 pickle，得先在主行程取樣成 `DemGrid`；成本大於目前的收益，留待日後。
3. **`QApplication.processEvents()` 灑在規劃迴圈裡**。要改規劃器本身，而且
   在 GUI 執行緒重入事件迴圈是既有的一整類 bug 來源。

## Consequences / 後果

### Positive
- 山區大情境不再凍結視窗；狀態列有六段進度回報。
- 後處理成為一支可獨立測試的方法：`tests/test_hv_dccpp_adapter.py` 用假 self
  餵真實規劃結果跑完整條顯示路徑（本機 QtWebEngine 一 `setHtml` 就 abort，
  建不出真的 MainWindow，這個切點讓後處理仍可被測）。
- 邊界點改用快照 ⇒ 使用者在規劃期間改邊界，不會畫出與實飛不符的多邊形。

### Negative
- **取消只在階段邊界生效**：`plan_mission` 沒有取消點，0.40 → 0.90 之間整段不可中斷。
  絕對不可用 `QThread.terminate()`（會在 numpy/scipy 的 C 層留下不一致狀態）。
- 0.40 → 0.90 沒有中間進度 ⇒ UI 應該用不定量 `QProgressBar`，不要用百分比條。
- 例外處理分成兩段：solver 的例外由 worker 轉成 `failed` 信號，顯示的例外由
  `_display_dccpp_result` 自己接。原本一個 `except` 涵蓋兩者。
- 新增了一道重入閘門（規劃中再按一次會被擋下）。以前是靠 UI 凍結擋住的。

### Neutral
- `HvPlanningWorker` 接受**已設定好旗標的 coordinator**，而不是自己組
  `HvDccppCoordinator`：GUI 與 headless 共用 `mission/swarm_coordinator.py`
  的同一個接縫，避免出現「兩條啟動路徑參數覆蓋不一致」這類靜默失效。
