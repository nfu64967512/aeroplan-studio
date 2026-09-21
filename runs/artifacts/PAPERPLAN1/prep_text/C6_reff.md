# C6 有效轉彎半徑 R_eff — 逐句替換稿（不需重飛）

## 先更正一件事：表 1 已經有三列了

先前規劃建議「表 1 增列四列（R_theo／R_plan／φ_max／W*）」。我核對 `cowork_brief.md:166`：

> `| 轉彎半徑（理論／規劃，最大坡度） | 130／200 m（20°） | 75／120 m（25°） |`

**R_theo、R_plan、φ_max 三者都已經在表 1 了**，只是沒有掛上符號。
在 4 頁限制下，正確做法是**改寫這一列的表頭（掛符號）＋只新增一列 W\***，而不是加四列。省 3 行版面。

## 我今天親自驗過的數字

| 量 | Pioneer（A） | HERO（B） | 出處 |
|---|---|---|---|
| $V_a$ | 21.5 m/s | 19.0 m/s | `hv_dccpp/platform.py:435-443, 464-472`（我讀了原始碼常數） |
| $\phi_{\max}$ | 20° | 25° | 同上 |
| $R_{theo}$（表值） | 130 m | 75 m | 同上 |
| $R_{plan}$ | 200 m | 120 m | 同上 |
| $V_a^2/(g\tan\phi)$（g = 9.81） | 129.5 m（−0.4 %） | 78.9 m（**+5.2 %**） | 我以 Python 重算；與 `docs/hv_dccpp/reference/design_drafts/design_geometry.md:82` 同值 |
| $W^*=V_a(\sqrt{R_{plan}/R_{theo}}-1)$ | **5.1675 m/s** | **5.0333 m/s** | `platform.py:302-304` `w_star()` 實呼叫 |
| $R_{eff}(5)$ | 200.00 m（$R_{plan}$ 分支） | 120.00 m（$R_{plan}$ 分支） | `platform.py:296-300` 實呼叫 |
| $R_{theo}(1+W/V_a)^2$ at W=5 | 197.50 m | 119.67 m | 同上 |
| $R_{eff}(6)$／$R_{eff}(8)$ | 212.68／244.74 m | 129.85／151.45 m | 同上 |
| $k=\lceil 2R_{eff}/s\rceil$，s = 96.99 | 5 | 3 | `hv_dccpp/dubins.py:619-623` 實呼叫 |
| 產物讀回 | `RTL_RADIUS,200`／`WP_LOITER_RAD,200` | `RTL_RADIUS,120`／`WP_LOITER_RAD,120` | `HV4SITL16/plan/{A1,B2}.param`（我直接 grep 過）；15 趟全同 |

⇒ **在本文的 W = 5 m/s，風修正項恰好為 0 m。** 這不是推論，是產物讀回。

## 參考文獻查證（我用 Crossref 與出版社全文實查，記錄在 `CITATIONS_VERIFIED.txt`）

- **[5] Coombes et al., Sensors 18(7) 2132, 2018** — 書目與草稿逐字相符。全文確實**以擺線（trochoid）建模風中轉彎的地面軌跡**，並以兩條擺線的公切線構造 Dubins 式路徑。
- **但 [5] 全文沒有把轉彎半徑放大成 $(1+W/V_a)^2$ 倍**，它用的是 $R=V/\dot\psi$ 的精確擺線參數式。
  ⇒ **不可以把 $R_{eff}$ 這個純量代理寫成出自 [5]**，也**不可以主張為本文提出**。正確定位：擺線曲率的一行推論。
- [5] 自己把轉彎時間式歸給 Techy & Woolsey；repo 內第一手閱讀筆記 `docs/hv_dccpp/論文_三篇全文閱讀_2026-09-10.md:104` 記載同一件事。
- **[15]（選配新增）Techy, L., and Woolsey, C. A., "Minimum-Time Path Planning for Unmanned Aerial Vehicles in Steady Uniform Winds," Journal of Guidance, Control, and Dynamics, Vol. 32, No. 6, pp 1736-1746, November 2009.** — 經 Crossref (10.2514/1.44580) 逐欄核對**完全相符**，可直接使用。

---

## 新增 A／2.3 節，插在 `cowork_brief.md:131` 該句之後、式 (3) 之前

新增編號式（建議編為 **(3)**，原 (3)(4)(5) 順延為 (4)(5)(6)；或編為 (4) 置於式 (3) 之後，視排版順手為準）：

$$R_{\mathrm{theo},c}=\frac{V_{a,c}^{2}}{g\tan\phi_{\max,c}},\qquad
R_{\mathrm{eff},c}(W)=\max\Big\{R_{\mathrm{plan},c},\ R_{\mathrm{theo},c}\big(1+\tfrac{W}{V_{a,c}}\big)^{2}\Big\},\qquad
W^{*}_{c}=V_{a,c}\Big(\sqrt{\tfrac{R_{\mathrm{plan},c}}{R_{\mathrm{theo},c}}}-1\Big)$$

Word 線性格式（貼入後按 Enter 轉專業格式）：
```
R_theo,c = V_a,c^2/(g tan φ_max,c),  R_eff,c(W) = max{R_plan,c, R_theo,c (1+W/V_a,c)^2},  W*_c = V_a,c(√(R_plan,c/R_theo,c) − 1)
```

## 新增 B／緊接其後的說明段（約 150 字）

> 以固定傾角 $\phi_{\max}$ 飛出之等速轉彎，於空速座標為圓，於地面座標則為擺線（trochoid）[5]；該擺線之曲率半徑於順風相位最大，其值為 $R_{\mathrm{theo}}(1+W/V_a)^{2}$，其中 $R_{\mathrm{theo}}$ 為無風理論最小轉彎半徑、$W$ 為風速。本文不於空速座標生成路徑再轉換至地面（端點固定於地面時將成為移動目標問題），而是直接以單一半徑 $R_{\mathrm{eff}}$ 於地面座標生成 Dubins 路徑[6]，取上式之最大值為該半徑。$R_{\mathrm{plan}}$ 為各構型之規劃轉彎半徑，其內建裕度可吸收 $W\le W^{*}$ 之風；式中 $W$ 取「起降場高度加轉換完成高度」至「最高作業高度」區間內之最大層風速。上式非本文新提出，係擺線曲率之直接推論[5]；本文之選擇在於以其最壞值作為地面座標之**單一** Dubins 半徑，使條帶排序與成本表得以全程留在地面座標。

## 新增 C／誠實註記（**必寫**；置於 2.3 節末或第四節「限制」）

> 本文情境之規劃風速 $W = 5$ m/s 低於兩型之 $W^{*}$（Pioneer 5.17 m/s、HERO 5.03 m/s），故 $R_{\mathrm{eff}}$ 於本次驗證中退化為 $R_{\mathrm{plan}}$（200 m／120 m），風修正項未生效；匯出至飛控之 `RTL_RADIUS` 與 `WP_LOITER_RAD` 亦逐趟讀回為 200 m／120 m。其作用需 $W\gtrsim 5.2$ m/s 之情境方顯現（$W = 6$ m/s 時為 213 m／130 m，$W = 8$ m/s 時為 245 m／151 m，跳帶數 $k$ 亦由 5／3 增為 6／4）。

## 替換 D／表 1 第二列（`cowork_brief.md:166`），並**新增一列**

**原列**：`| 轉彎半徑（理論／規劃，最大坡度） | 130／200 m（20°） | 75／120 m（25°） |`

**改為（掛上符號，讓式子與表綁定）**：
> `| 轉彎半徑 $R_{theo}$／$R_{plan}$（最大傾角 $\phi_{\max}$） | 130／200 m（20°） | 75／120 m（25°） |`

**新增一列**：
> `| 規劃裕度可吸收之風速 $W^{*}$ | 5.17 m/s | 5.03 m/s |`

## 替換 E／前言貢獻 (i)（`cowork_brief.md:93`）

**原句**：「(i) 以各構型含風有效轉彎半徑計算作業時間，並以決策級機數配額取代僅作為搜尋種子之機數界；」

**改為**：
> 「(i) 以各構型之**風修正有效轉彎半徑 $R_{\mathrm{eff}}$ 與考慮風場之地速**計算作業時間，並以決策級機數配額取代僅作為搜尋種子之機數界；」

（此句同時是 C7 用詞統一的一部分；見 `C7_costfn.md`。）

## 替換 F／參考文獻（選配，僅在版面允許時）

於 `cowork_brief.md:240` 之後新增：
> `[15] Techy, L., and Woolsey, C. A., "Minimum-Time Path Planning for Unmanned Aerial Vehicles in Steady Uniform Winds", Journal of Guidance, Control, and Dynamics, Vol. 32, No. 6, pp 1736-1746, November 2009.`

省版面方案：**不新增**，新增 B 段只引 [5]（已查證 [5] 確實用擺線建模，此引用正確）。

---

## ⚠ 絕對不要寫的句子

> 「$R_{\mathrm{eff}}$ 保證任何相位所需傾角不超過 $\phi_{\max}$」

依表 1 現值，HERO 在 $W = 5$ m/s 飛 $R_{\mathrm{eff}} = 120$ m 之地面圓，**需求傾角 26.08°**（我以 g = 9.80665 實算），高於其 $\phi_{\max} = 25°$。
成因：表值 $R_{theo} = 75$ m 反推傾角為 26.13°，幾何自洽值應為 78.9 m（偏差 +5.2 %，恰好通過 `platform.py:249-253` 的 ±10 % 建構期檢查而未被擋下）。
Pioneer 則成立（19.7° ≤ 20°，餘裕 0.3°）。
⇒ 上文新增 B 段已改成描述性寫法（「取上式之最大值為該半徑」），不作保證性宣稱。若一定要寫保證，只能針對 Pioneer。
⇒ **投稿前不要改 `platform.py` 的 75 m**：會連鎖改變 $W^{*}$、$d_{turn}$、圍籬外擴與匯出參數，使既有 15 趟產物與程式碼脫節。此事宜投稿後另案處理。
