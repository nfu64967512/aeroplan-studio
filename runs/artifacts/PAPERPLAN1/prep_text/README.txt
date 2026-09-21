prep_text — AASRC2026 論文文字修訂包（label: prep_text，2026-09-21）
唯讀調查 + 純 Python 規劃器重跑；**未動 repo 任何原始碼**、未起 SITL / MAVProxy / ArduPilot、
未碰 WSL 或 COM、未 git add/commit、未覆寫 HV4SITL15/16。

檔案
  00_SOURCE_OF_TRUTH.md      .doc 是空白範本的實證 + 單一真相來源建議（任務 1）
  C4_params.md               C4 逐句替換稿：「相符」定義、32 vs 34（任務 2）
  C6_reff.md                 C6 逐句替換稿：R_eff 定義式、表 1 只需加一列、W=5 失效揭露（任務 2）
  C7_costfn.md               C7 逐句替換稿：用詞統一、k=5/3 是無風值（任務 2）
  C1_C2_placeholders.md      C1 高度 / C2 風場的誠實揭露句，⟪…⟫ 為待填佔位符（任務 3）
  Table2_skeleton.md         修訂後表 2 + 新增表 2b，每格標明數字來源（任務 4）
  Unbacked_claims.md         無證據支撐之主張，依審稿人查驗機率排序（任務 5）
  CITATIONS_VERIFIED.txt     [5] / Techy&Woolsey / McGee&Hedrick 的 Crossref 與全文查證

本次新產生的量測
  probe_strip_heading.py         規劃器實跑三種風，量作業段航跡方位（純 Python，約 31 s）
  strip_heading_vs_wind_dir.json 結果：5/270、5/180、0/0 三者作業段方位**全為 90°（東西向）**
                                 ⇒ 條帶方位不隨風轉；5/180 的 makespan 966.2 s 與 A1 13.603 km
                                   來自配置與排程改變，不是風成本
重現：python runs/artifacts/PAPERPLAN1/prep_text/probe_strip_heading.py
