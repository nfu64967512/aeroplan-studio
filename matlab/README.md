# MATLAB 使用手冊(aeroplan-studio)

> 對象:接手這個專案、要用 MATLAB 端做分析或模擬的人。
> 最後更新 2026-08-26。

---

## 0. 三十秒上手

```matlab
cd C:\Users\joy46\Documents\Projects\aeroplan-studio\matlab
addpath(fullfile(pwd,'mgs'))        % ⚠️ 這一行不可以省,見 §1

stot_selftest                        % 自檢:7 項,10 s
gsurvey.runAll('n', 8)               % 10 條導引律 + 比較圖
gsurvey.missionSim(gsurvey.missionParams('n',8,'releaseAtRing',true,'ringPace',true))
```

---

## 1. ⚠️ 環境:一定要先做的一件事

`+mgs` 套件放在 `matlab/mgs/` **底下**,不是 `matlab/` 根目錄。所以:

```matlab
addpath(fullfile(pwd,'mgs'))
```

沒加的話 `which mgs.selftest` 回**空字串** —— 而空看起來像「沒裝」,
會讓人去找不存在的安裝問題。這是本專案實際踩過的。

驗證:
```matlab
which mgs.selftest      % 應該回一個路徑
which gsurvey.params    % +gsurvey 在根目錄,不必 addpath
which cfg.params        % +cfg 同上
```

**環境現況(2026-08-26 實測,R2025b)**

| 自檢 | 結果 | 時間 |
|---|---|---|
| `stot_selftest` | **7/7 PASS** | 10 s |
| `mgs.selftest` | 15 項 **14 PASS / 1 FAIL** | 90 s |
| `mgs.zarchan.benchSuite` | **1546 scored / 1542 PASS / 4 FAIL** | 27 s |

那 4 個 FAIL 全在**戰略/彈道**章節(Ch.15/29/32),與我們的定翼終端導引無關。
⭐ 我們真正在飛的那條 `engage3d`(3D PPN 核心)**PASS**。

⭐ `stot_selftest` 的 t3 量到 MATLAB 移植與 `predictor.py` 差 **2.842e-14**
—— 機器精度級的跨語言驗證,代表兩邊跑的是同一份數學。

---

## 2. 四個套件,問的是四個不同的問題

| 套件 | 問的問題 | 含飛控? |
|---|---|---|
| `+gsurvey` | **這條律本身**在理想執行器上能做什麼 | ❌ 沒有 |
| `mgs` | 對得上 **SITL / 真機**嗎 | ✅ 四個時鐘 + 夾制鏈 |
| `mgs.zarchan` | 我們的核心對得上**教科書**嗎 | ❌(英尺!) |
| `+cfg` / `stot_*` | 逐行移植的 `tot_node` 導引與 plant | ✅ 夾制鏈 |

⚠️⚠️ **不要互相取代。** `gsurvey` 把飛控整個拿掉是**刻意**的:
真機資料上「律沒效」與「律的指令沒被執行」分不開
(實測 γ̇ 追蹤比只有 0.10–0.31),所以要一個乾淨的對照組。

⚠️⚠️ **單位邊界**:`mgs.zarchan.*` 一律用**英尺**(書上的單位),
其餘一律**公尺**。脫靶量「18.3」在英尺和公尺都是合理的數字,
靜默換算不會當掉,只會產出一個差 3.28 倍的答案。跨界要顯式轉。

---

## 3. `+gsurvey` —— 純運動學導引律測試台

### 3.1 掃所有律

```matlab
R = gsurvey.runAll('n', 8);                       % 表 + 圖
R = gsurvey.runAll('n', 6, 'aMaxG', 2.0);         % 收緊過載上限看誰先垮
gsurvey.animate(R);                                % 錄 MP4(⚠️ 約 29 分鐘)
```

輸出:`matlab/out/gsurvey/`(PNG)、`~/Videos/aeroplan_guidance_survey/`(MP4)。

**已實作的 10 條**(挑過的,不是全部 25 條):
`pursuit / ppn / tpn / apn / zem / bpng / tpg / itcg / mpng / dgcg`

挑掉的與理由:RPN・TCPN(尾追,我們目標靜止)、Beam Rider(沒波束)、
Weave(打蛇形目標)、WTG・NOG・PINN-PCG・DD-ITCG(需要我們沒有的基礎設施)。

**實測結果摘要**(n=8,實測工作點,理想執行器):

| 律 | 散布 s | 水平誤差 m | 落角 | 中位 g | 飽和% | 航跡比 |
|---|---|---|---|---|---|---|
| PPN(現行) | 3.12 | 0.89 | −26.8 | 0.10 | 0 | 1.005 |
| BPNG / TPG | 3.18 | 0.8–1.5 | **−39.9 / −40.0** | 0.23 | 0 | 1.01 |
| ITCG | **0.82** | **48.2** | −22.2 | 0.33 | 3.4 | **1.049** |
| DGCG | 3.09 | 0.89 | −26.8 | 0.10 | 0 | 1.005 |

### 3.2 加一條新的律

在 `+gsurvey/+laws/` 放一個檔,簽章固定:

```matlab
function [a, s] = mylaw(o, p, s)
%   o  觀測(見 gsurvey/Contents.m 的完整欄位表)
%   p  參數  s  跨拍狀態
a = p.N * o.V * cross(o.omega, o.v / o.V);   % 回傳 3x1,單位 m/s^2
end
```

⚠️ **三條硬規矩**:
1. 回傳**必須**是 3x1。二維律把第三分量設 0 並在檔頭說明。
2. **不可以**用 `persistent` 或 `global`。跨拍狀態一律走 `s`,
   否則 n=8 的八架會互相污染,**而且是靜默的**。
3. 回傳的是**導引指令**,不含重力 —— 重力由 `simulate` 加。

然後在 `runAll.m` 的 `LAWS` 表加一列。

### 3.3 ⚠️ 建這個測試台踩過的五個坑(都會靜默)

| # | 坑 | 表徵 |
|---|---|---|
| 1 | 沒有散度 ⇒ 散布**恆 0** | 每條時間律都假達標([[instrument-that-cannot-fire]]) |
| 2 | 沒有地面約束 | 軌跡穿到地面下 117 m 再拉回,落角量到 **+17°(爬升)** |
| 3 | 重力沒配平 | 所有不含補償項的律都下沉 —— 真機是配平的(θ=γ+α) |
| 4 | 終端 V²/R 奇異點 | TPG 峰值 **703 g**、DGCG **1966 g**;修法照搬真機的終端凍結 |
| 5 | 落角律第二項符號 | 要 `(λ − γ_f)` 不是 `(γ_f − γ)`;寫反會提早插地,`track < 1` 是指紋 |

⚠️ 另外:MATLAB `inputParser` **不分大小寫**,`'N'` 會與 `'n'` 撞名。

---

## 4. `gsurvey.missionSim` —— AUTO 航線全任務

```matlab
p = gsurvey.missionParams('n',8,'releaseAtRing',true,'ringPace',true);
o = gsurvey.missionSim(p);
gsurvey.plotMission(o);
```

相位機與 `tot_node` **同名**:
`TAKEOFF → CLIMB → TRANSIT → LOITER →` 全員到齊閘門 `→ DIVE → DONE`

**三組態實測(n=8,彈射間隔 3 s)**

| 組態 | 散布 s | R0 全距 m | 誤差中位 m | 落角全距 | 盤旋 s |
|---|---|---|---|---|---|
| 基線(閘門即放行) | 23.75 | **1987** | 5.1 | 20.3° | 14.4 |
| + `releaseAtRing` | **139.55** | 5 | 2.8 | 0.8° | 137.7 |
| **+ `ringPace`** | **5.90** | **4** | **2.2** | 0.8° | **2.9** |

⭐⭐⭐ 三個對設計有直接影響的結論:

1. **閘門開啟 ≠ 放行。** 各機在盤旋圈上的相位隨機 ⇒
   R0 全距 **1987 m ≈ 盤旋直徑**,而 R0 是殘餘高度的唯一自變數。
2. **`releaseAtRing` 單獨開會炸掉散布(139 s),而那是 modulo 假影** ——
   七架剛錯過穿越點要再繞一整圈。縮小盤旋半徑救不了(已被最小迴轉半徑
   2666 m 頂住)。
3. ⚠️⚠️ **`ringPace` 必須在進場段,不可以在盤旋圈裡調速。**
   要全員同時穿越,速度比必須覆蓋弧長比,而 `Vmax/Vmin = 1.58`,
   弧長比接近整圈 ⇒ **權限差一個數量級**。盤旋一旦開始就補不回來。

剩下的 5.90 s **全部**來自俯衝耗時差(抵環時刻只差 0.5 s)⇒
下一步要鎖的是**放行航向**,不是距離。

---

## 5. `mgs` —— 對 SITL 的高保真層

```matlab
addpath(fullfile(pwd,'mgs'))
mgs.selftest                                  % 15 項
mgs.selftest('slx', true)                      % 加跑 Simulink 對照
c   = mgs.params();
scn = mgs.scenario(...);
out = mgs.simSwarm(scn, c);                    % N 機 + 四個時鐘 + 夾制鏈
out.kpi                                         % 散布 / 命中率 / CEP / 終端速度
```

四個時鐘(**不是裝飾**):遙測 40 Hz、導引 20 Hz、飛控 50 Hz、plant 連續 RK4。
把它們摺成同一個速率,觸地誤差會退化成純步長殘差。

其他常用:
```matlab
mgs.contractCheck(...)   % 對 contract_table.py
mgs.shadowDiff(...)      % 分層歸因:guidance / clamp / plant
mgs.kpi(out)             % KPI
mgs.flightMovie(out)     % 動畫
```

⚠️ `out.calibrated` 為 false 時**看 `out.notes`** —— 它會說出哪一條腿沒有
真值背書。不要引用一個 `calibrated=false` 的數字。

---

## 6. `mgs.zarchan` —— 98 個 Zarchan 移植

```matlab
mgs.zarchan.benchSuite()                       % 全部,27 s
mgs.zarchan.benchSuite('only', {'engage3d'})   % 只跑一個
mgs.zarchan.engage3d('benchmark', false, 'qpn', 1)
```

⚠️⚠️ **英尺**,`g = 32.2`。⚠️ 綠板**不代表**模型得了我們這台機體:
沒有重力、沒有阻力、沒有推力、沒有不對稱 nz 夾制、沒有 LOS 地板,
最關鍵的是**沒有會飽和的遲滯**。我們實測 γ 追蹤增益 0.063 對模型的 1.385。

---

## 7. `stot_*` —— `tot_node` 的逐行移植

```matlab
stot_selftest        % 7 項
stot_guidance(m, gs, tnow, pn)     % = tot_node._pn_cmd + _pn_step
stot_plant(...)                     % 3-DOF 質點 + 一階角度迴路
stot_run32                          % 32 架次重放
```

⚠️⚠️ **順序是規格的一部分**(`stot_guidance.m` 檔頭明寫):

    nz 夾制 → pn_lead → **LOS 地板 min()** → alpha → pitch 夾 → roll 夾
            → 油門 → 終端凍結

把 LOS 地板放到 nz 夾制**之前**,會得到一個「`pn_nz_min` 有效」的模型 ——
而真機上它幾乎惰性(實測 35,590 拍裡正側夾制只決定 **4 拍**)。
`stot_selftest` 的第 1 項就在鎖這件事。

---

## 8. 常見問題

**Q:`which mgs.xxx` 回空**
A:忘了 `addpath(fullfile(pwd,'mgs'))`。見 §1。

**Q:改了 `.m` 但行為沒變**
A:`clear functions; rehash`。套件函式會被快取。

**Q:MATLAB 主控台印出亂碼**
A:那是 Windows 的 cp950。從 Python 呼叫時加 `PYTHONIOENCODING=utf-8`;
MATLAB 端寫檔用 `fopen(..., 'n', 'UTF-8')`。

**Q:錄影多久?**
A:`gsurvey.animate` 11 支約 **29 分鐘**(每支 ~440 幀)。要改參數重錄請預留。

**Q:模擬結果可以拿去當航次預測嗎?**
A:**不可以。** `gsurvey` 是理想執行器;`mgs` 才對 SITL,而 SITL 也不是真機。
實測 γ̇ 追蹤比 0.10–0.31,所以模擬裡收斂得漂亮的落角在真機上一定打折。

---

## 9. 檔案地圖

```
matlab/
  README.md                 ← 本檔
  +gsurvey/                 純運動學導引律測試台
    Contents.m              ⭐ 介面契約(律的簽章、觀測欄位)
    params.m  scenario.m  simulate.m  metrics.m
    runAll.m  plotRuns.m  animate.m
    missionParams.m  missionSim.m  plotMission.m   ← AUTO 全任務
    +laws/                  一律一檔
  mgs/+mgs/                 ⚠️ 要 addpath(matlab/mgs)
    selftest.m  simSwarm.m  params.m  kpi.m  shadowDiff.m  contractTable.m
    +zarchan/               98 個書本移植
  +cfg/params.m             tot_node 參數的 MATLAB 鏡像
  stot_*.m                  tot_node 的逐行移植 + 自檢
  gammadiag/                γ 追蹤缺陷的離線分析
  out/gsurvey/              圖
```

## 10. 相關記憶

`gsurvey-testbed`、`auto-mission-matlab-run`、`inner-loop-is-not-the-bottleneck`、
`zarchan-port-suite`、`model-is-5dof-not-6dof`、`law-port-verified-in-flight`
