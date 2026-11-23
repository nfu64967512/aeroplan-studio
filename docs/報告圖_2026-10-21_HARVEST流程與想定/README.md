# 報告圖：HARVEST 流程與想定（2026-10-21）

| 檔案 | 內容 |
|---|---|
| `fig1_flow.png/.svg` | 圖1 端到端流程（任務設定 → 協調前處理 → HARVEST → 飛前檢核 → 匯出放行 → 執行與飛後）＋驗證現況 |
| `fig2_harvest.png/.svg` | 圖2 `plan_mission` 八步驟，含 NO_GO／未覆蓋出口、能量修復與去衝突修復回圈 |
| `fig3_scenario.png/.svg` | 圖3 想定圖（真實比例），航線取自 HV4W55 上傳的 `.waypoints` |
| `fig4_timeline.png/.svg` | 圖4 規劃 vs. SITL 起飛時程、相對高度剖面、117–121 m 放大 |

- PNG 為 2×（寬 2560 px），白底；SVG 用屬性上色（不靠 CSS 變數），Word／PowerPoint 可直接插入。
- 字型：SVG 指定 Noto Sans TC → 微軟正黑體 → 系統無襯線；沒裝 Noto 的電腦會用微軟正黑體。
- 數據出處：SITL 架次 HV4W55（證據 commit 95a337b，數字以 `runs/artifacts/PAPERPLAN1/update_paper/out/NUMBERS.json` 為準）。
- ⚠ 圖3／圖4 是**已飛驗的 09-21 那一版**計畫。以 10-21 程式重規劃同一想定會得到不同計畫（B3 改降備降點 ALT1、規劃 makespan 898.5 s，未飛驗）；10-09 起預設每架降回自己的起飛坪。

重產（在 repo 根目錄；輸出到任一資料夾 `<out>`）：

```bash
python -X utf8 "docs/報告圖_2026-10-21_HARVEST流程與想定/src/extract.py" . "docs/報告圖_2026-10-21_HARVEST流程與想定/src/data.json"
python -X utf8 "docs/報告圖_2026-10-21_HARVEST流程與想定/src/gen.py" <out>
python -X utf8 "docs/報告圖_2026-10-21_HARVEST流程與想定/src/shot.py" <out> 2
```

## ppt/：今天規劃結果的 16:9 投影片圖（建議用這組）

| 檔案 | 內容 |
|---|---|
| `ppt/slide1_birdseye.png/.svg` | 想定圖鳥瞰（3D 斜視，高度放大 1.8 倍）＋ VTOL 八階段分鏡：垂直起飛 → 轉換 → 爬升進場 → 條帶作業 → 迴轉 → 返場 → 回轉換 → 垂直降落 |
| `ppt/slide2_timeline.png/.svg` | 三機依航段上色的時程（甘特）＋ 規劃高度剖面，標號對應分鏡 ①–⑧ |
| `ppt/slide3_plan.png/.svg` | 真實比例平面圖＋含 ALT1 的位置圖＋機隊分工表 |

- 3840×2160（16:9），可直接當整張投影片；SVG 用屬性上色，PowerPoint 插入後可「轉換成圖形」再改字。
- 資料：以 2026-10-21 工作樹重規劃 `gui_legal_120_1a2b`（天花板 119.5 m），快照在 `src/today.json`（由 `src/dump_today.py` 產生）。**規劃值、尚未飛驗**：makespan 898.5 s、validate 通過、四維衝突 0；B3 由回收站自動指派改降 ALT1（H 北方 2.5 km）。
- 高度一律用 AGL（`agl_m`）；`z_m` 是局部 ENU，條帶兩端會因地球曲率低 0.1 m。

重產投影片圖：`python -X utf8 src/dump_today.py <repo根> src/today.json` → `python -X utf8 src/gen2.py <out>` → `python -X utf8 src/shot.py <out> 2 "slide*.svg"`（`gen2.py` 需與 `gen.py`、`data.json` 同資料夾）。
