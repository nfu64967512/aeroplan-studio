# `corrected/` 索引 —— 前面各 label 產物的更正副本（label: update_paper）

> **原檔一律未被修改。** 每一列給：原檔、更正當下的 sha256、涉及哪幾項 K、更正副本路徑。
> 定稿文字在 `update_paper/TABLES_ZH.md` 與 `update_paper/PAPER_TEXT_ZH.md`；
> 逐項重算與出處在 `update_paper/CORRECTIONS_K1_K9.md`；機器可讀在 `update_paper/out/corrections.json`。

| 原檔 | 更正當下 sha256（前 16） | 涉及 | 更正副本 |
|---|---|---|---|
| `paper_text/TABLES_ZH.md` | `c37ba4fc82921e23` | K1、K2、K3、K4、K5、K9 | `corrected/paper_text__TABLES_ZH.CORRECTION.json` |
| `paper_text/PAPER_TEXT_ZH.md` | `6a73a5015e2675e2` | K2、K6、K7 | `corrected/paper_text__PAPER_TEXT_ZH.CORRECTION.json` |
| `paper_text/README.md` | `b18a53769092a0b8` | K2、K5、K6 | `corrected/paper_text__README.CORRECTION.json` |
| `paper_text/out/NUMBERS.json` | `6e55f5c97f727963` | K2 | `corrected/paper_text__out_NUMBERS.CORRECTION.json` |
| `prep_text/C1_C2_placeholders.md` | `0e393deef7e66c15` | K6 | `corrected/prep_text__C1_C2_placeholders.CORRECTION.json` |
| `prep_text/C4_params.md` | `c1d8b23c98758d51` | K2 | `corrected/prep_text__C4_params.CORRECTION.json` |
| `prep_text/Table2_skeleton.md` | `3a773fd8a2610aeb` | K2 | `corrected/prep_text__Table2_skeleton.CORRECTION.json` |
| `sorties/sortie_results.json` | `227c6db214c02efc` | K3、K4 | `corrected/sorties__sortie_results.CORRECTION.json` |
| `sortie_workflow_result.json` | `747ebcfb2f0464ff` | K8 | `corrected/workflow__sortie_workflow_result.CORRECTION.json` |

## 非更正、但必須說明的一則

| `sorties2/HV4W55S_windaxis.json` | — | （方法對照） | `corrected/sorties2__HV4W55S_windaxis.RECONCILIATION.json` |

兩者都對：該 label 用 ~1 Hz `flight.csv` 並自連續經緯度推算航跡，本 label 用原生 dataflash 的 `GPS.Spd`／`GPS.GCrs`。180° 那一趟在 100 m 巡航門檻下 B2 的南向箱只剩 1 個樣本，故本文改引 30 m 門檻並在表註寫明。結論（+9.5~+9.9 對期望 +10.0、歸零軸 ≈ 0）兩法一致。
