# 已取得的論文（2026-09-08）

使用者提供 8 篇 PDF。7 篇已用 `pdftotext -layout` 抽成同目錄的 `.txt`；
每篇由一個代理逐字讀完，再由另一個對抗性代理**逐欄開檔覆核**。

⚠ 第 8 篇（Dubins 農噴）是**掃描影像，沒有文字層**（pypdf 抽出 0 字元），
本機也缺 `pdftoppm` 無法轉圖 ⇒ **尚未讀**。需要有文字層的版本。

---

## 書目（逐字取自 PDF，未逐字者標明）

| # | 引用 | DOI |
|---|---|---|
| 1 | Kapoutsis, A.Ch.; Chatzichristofis, S.A.; Kosmatopoulos, E.B. **DARP: Divide Areas Algorithm for Optimal Multi-Robot Coverage Path Planning.** *J Intell Robot Syst*. Received 6 Dec 2016 / Accepted 15 Dec 2016. © Springer 2017. | `10.1007/s10846-016-0461-x` |
| 2 | Choset, H. **Coverage of Known Spaces: The Boustrophedon Cellular Decomposition.** *Autonomous Robots* 9, 247–253, 2000. Kluwer. | 不在 PDF 內 |
| 3 | Coombes, M.; Fletcher, T.; Chen, W.-H.; Liu, C. **Optimal Polygon Decomposition for UAV Survey Coverage Path Planning in Wind.** *Sensors* 2018. Loughborough University. | 見基準線 ref [11]：18(7):2132 |
| 4 | Bähnemann, R.; Lawrance, N.; Chung, J.J.; Pantic, M.; Siegwart, R.; Nieto, J. **Revisiting Boustrophedon Coverage Path Planning as a Generalized Traveling Salesman Problem.** arXiv:1907.09224v1 [cs.RO] 22 Jul 2019. | 見基準線 ref [33]：FSR 2021, 277–90 |
| 5 | Bochtis, D.D.; Vougioukas, S.G. **Minimising the non-working distance travelled by machines operating in a headland field pattern.** *Biosystems Engineering* 101 (2008) 1–12. | 不在 PDF 內 |
| 6 | Karakaya, S.; Konyar, M.Z. **Hybrid Boustrophedon and Direction-Biased Region Transitions for Mobile Robot Coverage Path Planning: A Region-Based Multi-Cost Framework.** *Appl. Sci.* 2025, 15, 12666. | `10.3390/app152312666` |
| 7 | Adoni, W.Y.H.; Fareedh, J.S.; Lorenz, S.; Gloaguen, R.; Madriz, Y.; Singh, A.; Kühne, T.D. **Intelligent Swarm: Formalism, Design and Validation of Self-Organized UAVs Based on Leader–Followers Paradigm for Autonomous Mission Planning.** *Drones* 2024, 8, 575. | `10.3390/drones8100575` |

⚠⚠ **第 1 篇的卷/期/頁不在 PDF 裡**（online-first 版面），投稿前必須從出版商紀錄補上。
我不會替它編號碼。

⚠ **第 7 篇的標題**：PDF 的 citation 欄印的是「**Formalism**, Design and Validation」，
與檔名的「Concept, Design and Validation」不同。以 PDF 內文為準。

---

## ⚠⚠⚠ 對我們主張的衝擊（這才是重點）

### A2（風修正時間成本）—— **已被 [3] 先做，必須放棄**

Coombes et al. 2018 的自述貢獻逐字（extraction :210-211）：

> • Novel cost function for calculating flight time in the presence of wind.
> • Mathematical proof of minimum flight time with the survey direction perpendicular to the wind.

他們**已經**把成本定成含風的飛行時間，而且**證明了**最短時間出現在掃描方向垂直於風。
:174 還明講轉彎時間取決於空速與風向。

⇒ **A2 不是我們的貢獻。** 論文裡只能寫成「沿用 [3] 的含風時間成本」。

### A3 / A4（s<2R 區間、轉場型態）—— **被 [5] 嚴重削弱**

Bochtis & Vougioukas 2008 已經做了：

* 逐字 :157-158 —— 「the turning distance between any two tracks **depends on the turning
  radius of the vehicle as well as on the distance between these tracks**」。
  **這就是我們說的 s 與 R 同量級。** 而且是 2008 年的農機文獻。
* 轉彎型態分類：**P-turn（double round corner）、U-turn（loop/forward-turn）、
  T-turn（reverse/switch-back）**（:195-196、Fig. 2）。逐字 :231
  「For shake of presentation simplicity only the P-turn and the U-turn are considered
  in this work」（原文就是 `shake`，不是 `sake`，引用時要標 [sic]）。
* **manoeuvre degree** `d_i = |π⁻¹(i+1) − π⁻¹(i)|` —— **就是我們的 skip-k**，
  並用 `J_turn(s) = Σ L_min(d_i)` 對整個 track 覆蓋順序做最佳化（Eq. 3）。
* Fig. 1 明確區分 adjacent 與 **non-adjacent** traversal。

⇒ **A3「沒人在 R 與 s 同量級的區間工作」是錯的**，農機領域做了十幾年。
⇒ **A4 的轉場型態分類也不是新的。**

### A1（續航驅動的機數下限）—— **目前存活**

* [1] DARP：機數 `n_r` 是**外生輸入**，全文沒有 radius / Dubins / 速度 / 風，
  單位全是格子。§6.5 有一段「依能力給不同配額 p_i」的延伸（Eq. 22），
  但**從未實作也未評估**，且 `battery` 全文只出現那一次、`endurance` 零次。
* [7] 觀察到單機在大面積下電量不夠，但那是**事後**從 k 掃描看出來的，
  沒有下限公式。⚠ 而且它的比較基準 [99] LMAT 是 WSN 定位軌跡、
  論文自己說它「consumes quite a lot of energy due to the frequent acceleration
  and deceleration maneuvers」—— 拿它當「單機不可行」的證據站不住。

⇒ **A1 是目前唯一還算完整的演算法主張。**

### B（法規上限下的閉迴路）—— 存活，且 [4] 反而支持 A3 的一半

[4] Bähnemann 逐字 :321：「Since MAVs can **hover and turn on the spot**, we can plan
rest-to-rest segments」⇒ 它是多旋翼，**R = 0**。加上基準線的 R = 4000 m，
文獻確實在兩個極端，中間帶是空的 —— 但 [5] 證明那個中間帶在**農機**裡是滿的。

---

## 重新定位的建議

不要再主張「我們發現了 s<2R 這個區間」。改成：

> 農機文獻（[5]）在**地面車輛**上處理了轉彎半徑與作業寬度同量級的排序問題；
> UAV 覆蓋文獻在 **R≈0 的多旋翼**（[4]）或 **R≫s 的大型定翼**（基準線）兩端。
> 把農機的轉彎幾何搬到**空中**之後，多出來的是農機沒有的耦合：
> 續航決定機數（A1）、法規高度上限決定機隊組成、含風時間成本（[3]）、
> 以及規劃到飛控的閉迴路驗證（B）。

這個講法**每一句都有證據**，而且不需要宣稱任何被 [3][5] 先做過的東西。

---

## 還缺的

* Dubins 原始論文（1957）—— [5] 用的是 `r_min` 但不是 Dubins 曲線。
* 那篇農噴 Dubins 論文的**文字版**。
* 前面清單裡的能量受限 CPP、min-max 多機、UTM/4-D 去衝突、VTOL 任務規劃。
