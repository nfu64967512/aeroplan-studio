# 單一真相來源（single source of truth）— 確認與建議

## 我自己查到的事實（不是轉述）

| 檢查 | 結果 | 如何重現 |
|---|---|---|
| `docs/hv_dccpp/AASRC2026_abstract/AASRC2026-投稿格式_加長型摘要_.doc` 是什麼 | OLE2 複合文件，54 272 bytes；OLE 摘要資訊：**2 頁、287 字**、Title `2019\|NQ\|`、Author `Chien-Chun Hung`、Last Saved By `admin`、Revision 2、建立與最後儲存皆 **2026-05-12 03:07**、Code page 950 | `file "….doc"` |
| 內容 | 只有大會範本樣板字串：「作者一ᵃ、作者二ᵃ、作者三ᵇ」「中文摘要內容，中文：標楷體11點」「English Title / Times New Roman 14-Point」，參考文獻是範本自帶的 Becker/Carey/Oden 與 Chassaing/Minh 兩筆，頁首「2026中華民國航太學會學術研討會／虎尾，中華民國115年11月14日」 | UTF-16LE 字串萃取，102 段 |
| 是否含本研究任何字串 | `HARVEST` / `DCCPP` / `VTOL` / `Pioneer` / `HERO` / `ArduPilot` / `SITL` / `HV-` / `102` / `makespan` 在 ASCII 與 UTF-16LE 下**各 0 次命中** | 二進位 `count()` |
| 版控狀態 | `git log -- "…/…doc"` **無任何 commit**；`git status --porcelain -- docs/hv_dccpp/AASRC2026_abstract/` 回 `?? docs/hv_dccpp/AASRC2026_abstract/` ⇒ **整個資料夾（含 cowork_brief.md、三張圖）都未納管** | `git log` / `git status` |

**結論（與先前調查一致，且更嚴重）**：repo 裡那份 `.doc` 是大會空白範本，指導老師引述的句子（例如「迎風」）**不可能**出自它。
而且不只 `.doc` 未納管——`cowork_brief.md` 本身也未納管。目前這篇論文**沒有任何一份文字在版本控制裡**。

`cowork_brief.md` 全文 grep「迎風」**0 筆**（:81 摘要寫「含風時間成本」、:131 正文寫「含風地速」）。
⇒ 老師看到的那份 Word 稿與 `cowork_brief.md` 已經分岔，而分岔的那一份沒有留存。
這就是 C1／C4／C7 這類「前後不一致」會反覆出現的根因：**沒有人能指著同一份檔案講同一句話。**

## 建議（成本最低、槓桿最高，投稿前必做）

1. **以 `cowork_brief.md` 為唯一真相來源（single source of truth）**，Word 稿一律由它排版產生，不在 Word 裡改內容。
   理由：數字有 附錄 B 的逐項出處、有 附錄 A 的禁用措辭，Word 檔沒有這兩樣東西。
2. **立刻把整個 `docs/hv_dccpp/AASRC2026_abstract/` 納入 git**（含真正投稿的 `.docx`／`.doc`，不是範本），
   並在 `cowork_brief.md` 開頭加一行版本戳（例如「v2026-09-21，對應 commit <sha>」）。
   ⚠ 依規則我沒有代為 `git add`／`commit`。
3. **Word 稿檔名帶日期**（例如 `AASRC2026_HARVEST_2026-09-21.docx`），且每次交給指導老師都附上對應的 `cowork_brief.md` commit sha。
   往後老師引述任何一句，都能在 30 秒內定位。
4. **建立回寫紀律**：老師在 Word 上的批註 → 改 `cowork_brief.md` → 重新產生 Word。
   反方向（直接改 Word）一次都不要做，否則真相來源立刻再次分岔。
5. 同場加映（`cowork_brief.md` 附錄 D 第 2 點已自述）：`runs/artifacts/HV4SITL15`、`HV4SITL16` 也未納管，
   而 `scripts/hv_sitl_scenarios.py:281` 把 `gui_legal_120_1a2b` 的 trial 寫死成 `"HV4SITL15"`。
   **在 C1 重飛與 C2 風場重跑開始前先納管這兩個目錄**，否則不帶 `HV_TRIAL` 跑一次就會蓋掉論文唯一的證據。
