# ⚠ 本資料夾有一則「方法對照」說明（原檔未被修改）

`HV4W55S_windaxis.json` 的北−南值（+9.90／+9.68／+9.61 m/s）取自 ~1 Hz 的 `flight.csv`；
label `update_paper` 以原生取樣率的 dataflash（`GPS.Spd`／`GPS.GCrs`）重算得 +9.853／+9.512／+9.598 m/s。
**兩者都對，判定也一致**（期望 +10.0；歸零軸 ≈ 0）；差異來自資料來源與航跡推算方式。
論文引用的是 `update_paper` 的值，並在表註寫明 30 m 高度門檻的理由
（100 m 門檻下 B2 的南向箱只剩 1 個樣本）。

對照說明：`runs/artifacts/PAPERPLAN1/update_paper/corrected/sorties2__HV4W55S_windaxis.RECONCILIATION.json`

另外：本資料夾的 `ACCEPTANCE.md:125` 與 `COMMIT_PLAN.md:113` 對 +0.0279 s（K6）**已經寫對**，無須更正。
`SPAWN_ANNOTATION.md` 的機制分析與本 label 的獨立重算一致（見 `update_paper/CORRECTIONS_K1_K9.md` K3），
唯本 label 另外指出：**弧的證據只來自 A1**（B2／B3 在 3.0／3.1 s 內收斂，其 trace 沒有任何 > 0.10 m 的步進）。

這張卡片是**新增**檔案，本資料夾原有的任何檔案都沒有被覆寫。
