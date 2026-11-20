# STOT 交接文檔 — 2026-08-04

給下一個 session 用。**這份是自足的**,不需要讀前一份就能接手。
前一份是 `docs/handoff_stot_2026-08-01.md`(內容多數已被本輪的實測推翻,見第 7 節)。

---

## 0. 三十秒摘要

六架固定翼從桃園機場向西彈射起飛,飛到台灣海峽海面同時打擊一個定點。
四項驗收門檻:①彈射間隔 3 s ②巡航編隊成形 ③命中精度 ≤10 m ④STOT 散布 ≤3 s。

**現況**:門檻 1 已穩定達標。門檻 3 只有 **P91** 那一趟真正達成過(6/6 觸地、
誤差 max 3.0 m、真的落到海面)。門檻 2、4 都還沒達標。

**最新一趟 SW1(2026-08-04)是混合結果**:ALIGN 階段大幅改善(抵達散布 18.3 s,
歷來最佳),但終端**實質 0/6** —— 六架在最近點都還離海面 291–1678 m。
已做過 26 個代理的對抗式徹查(20 條發現只有 2 條通過驗證),
**終端精度的根因目前仍是空的**。見第 5 節。

⚠️ **接手第一件事**:讀第 6 節「已被實測推翻的說法」。這個專案已經被好幾個
「看起來很合理但錯」的診斷浪費過很多趟。

---

## 1. 環境與開跑

```bash
# 全部在 WSL 裡跑。Windows 端用 wsl.exe -e bash -lc '...'
cd ~/fixwing_ws/sitl_run

# 開一趟(PREFIX 決定航次代號,TRIALS=1 跑一趟)
PREFIX=XX TRIALS=1 bash run_west.sh

# 改了 sar_swarm_patches/ 下的節點之後,一定要重新部署
bash deploy_build.sh      # 刪 build/+install/ → 重建 → md5 雙向驗證 → 冒煙測試
```

**專案原始碼在 Windows 端**:`C:\Users\joy46\Documents\Projects\aeroplan-studio\`
(WSL 路徑 `/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio/`)。
`deploy_build.sh` 負責 repo → `~/fixwing_ws/src/` → build/install 的同步。

⚠️ **不要在航次進行中跑 `deploy_build.sh`** —— 它會 `rm -rf build/`,
而 `tot_node` 正從那裡執行。

**分支**:`fix/dccpp-geofence-2d`(**不是 main**;main 是舊的)。
本輪所有改動都還**沒有 commit**,`git status` 會看到一大片。

---

## 2. 架構:一趟航次的四個階段

```
彈射起飛 ──→ 巡航編隊 ──→ ALIGN(散開到扇形) ──→ PERCH(對準內切) ──→ DIVE(俯衝)
 catapult      wingman_node          ←────────── tot_node ──────────→
 3 s 間隔      V 型編隊              扇形 R=35 km,6 slot,間隔 6109 m
```

- **編隊層**(`wingman_node` / `backup_node` / `master_node`)在交棒給 `tot_node`
  之前會被**刻意收掉**(`stot_trial.sh:308`),避免搶控制。
  所以在 log 裡看到編隊節點 `ExternalShutdownException` 死掉是**正常的**,不是崩潰。
- **tot_node** 是主角,`scripts/wsl/sar_swarm_patches/tot_node.py`,約 1600 行。
  ALIGN/PERCH/DIVE 三個相位都在它的 `_tick()` 裡。

---

## 3. 本輪(2026-08-04)做了什麼

### 3.1 修掉的量測錯誤(比修飛行行為更重要)

| 項目 | 問題 | 修法 |
|---|---|---|
| **命中判定不看高度** | `tot_node.py` 的衝擊判定只看水平距離,飛過目標上空 1400 m 也算「命中紅心」。RD1 因此自報「★★ 全員命中」,實際六架全在 598–1460 m 高空 | 加 `impact_alt_ok`(預設 30 m),摘要行會點名「⚠️ 名義命中,但 N/6 架在高空掠過」。SW1 已驗證生效 |
| **海面基準錯誤** | 目標在開闊海面,SRTM 無資料回外插值,六架對同一點各報一個高度(散布 14–17 m),均值把紅心算成 −3.2 ~ −15.1 m AMSL,**瞄到海平面以下** | `target_amsl` 參數,`run_west.sh` 設 0.0 釘死。SW1 實測跨機散布降到 **0.16 m** |
| **`_kpi_parse.py` 靜默污染** | 它 `open(CSV,"a")` 無條件附加;把軌跡 CSV 餵進去會在 12 MB 的航次資料尾巴悄悄多一列 | 加防呆:表頭像軌跡 CSV 就 exit 1 |
| **素材事後補會張冠李戴** | `/tmp/trial_launch.log` 不分航次只留最後一趟,回頭補素材會把最新一趟的起飛紀錄安到舊航次頭上 | `make_artifacts.sh` 比對時間戳,不屬於本航次就寫 `launch.log.MISSING` |

⚠️ **驗收一律用 `stot_score.py --json` 的 `alt_agl_m`,不要信 tot_node 的「全員命中」那行。**

### 3.2 改了飛行參數(全在 `run_west.sh`)

| 參數 | 舊 | 新 | 依據 |
|---|---|---|---|
| `VMIN` | 45 | **95** | `sep_envelope.py` 量到 3100 m 的 SEP 零交越在 EAS 72–73;45 在包絡外。**同時是 weave 的啟動門檻** |
| `WEAVE_PERIOD` | 22 | **120** | 噴射機轉彎率 3.58 °/s,θ=45° 換向要 25.1 s,舊的半週期只有 11 s ⇒ θ>19.7° 轉不完 |
| `PERCH_CLIMB_V` | 65 | **100** | 同一個包絡。EL1 的 UAV3 用 65 m/s「爬升」卻從 2603 m 掉到 519 m,整趟卡死 |
| `TARGET_AMSL` | (無) | **0.0** | 海面 |
| `DIVE_ANGLE` 預設 | 90 | **45** | 90° 讓 tan(γ) 奇異,每趟都被開跑前驗證擋下 |

**核心邏輯:用轉彎耗時間,不要用慢飛耗時間。** 舊設定要求飛機用飛不動的速度等,
結果是往下掉;而 weave 因為門檻設在 45 幾乎不啟動。

### 3.3 新增的工具

| 工具 | 用途 |
|---|---|
| `scripts/wsl/impact_feasibility.py` | **同時命中的物理可行區間**。用 Dubins「先轉後直」算各機最早可命中時刻 t_min,跨機差換算成「需繞幾圈盤旋」 |
| `scripts/research/paper_search.py` | 學術檢索(OpenAlex + arXiv;Semantic Scholar 需 `S2_API_KEY`) |
| `scripts/wsl/make_artifacts.sh` | 航次結束**立刻**產 GIF/圖表到 `runs/artifacts/<TRIAL>/`,已接進 `stot_trial.sh` 收尾 |
| `scripts/wsl/_formation_from_show.py` | 從 `show_*.csv` 事後重建編隊俯視圖(原本只能飛行中即時錄) |
| `scripts/wsl/sep_envelope.py` | 從 dataflash 反推 SEP 包絡(回答「這速度飛不飛得住」) |
| `scripts/wsl/preflight_check.py` | 開跑前從飛控讀回參數做**關係檢查**,不過就中止 |

**測試**:`tests/test_impact_feasibility.py`(13)、`test_sea_level_datum.py`(10)、
`test_phase_gate.py`、`test_lookahead_geometry.py`(30)、`test_signed_speed.py`(17)
—— 合計 **89 passed**。

```bash
wsl.exe -e bash -lc 'cd /mnt/c/Users/joy46/Documents/Projects/aeroplan-studio && \
  python3 -m pytest tests/test_impact_feasibility.py tests/test_sea_level_datum.py \
  tests/test_phase_gate.py tests/test_lookahead_geometry.py tests/test_signed_speed.py -q'
```
⚠️ 不要跑 `pytest tests`(全部)—— WSL 缺 pyproj/PyQt6/shapely,7 個模組會 collection error,
與本專案無關。

---

## 4. 關鍵發現:3 秒散布做得到,但架構短了一個數量級

`impact_feasibility.py` 跨三趟實測(傾斜上限 45°):

| 航次 | 跨機 t_min 差 | 需吸收路徑 | 環幾何**實際提供** | 換算盤旋圈數 |
|---|---|---|---|---|
| P91 (27°) | 172.5 s | 29.7 km | 0.35 km | 1.57 圈 |
| RD1 (45°) | 70.8 s | 8.2 km | 0.88 km | 0.95 圈 |
| EL1 (45°) | 66.4 s | 8.7 km | 2.22 km | 0.80 圈 |

**結論**:
1. 3 秒散布**物理上做得到** —— 飛機可以盤旋等,t_max 不受幾何限制。
2. **但純幾何排程結構性不足**。現行架構靠「各機環半徑不同」製造路徑差,
   環半徑 = 高度/tanγ,而高度差只有 `layer_m×(n−1)` ⇒ 只能生 0.35–2.2 km,
   需要 8–30 km。**短了約 10 倍。**
3. 這解釋了為什麼「縮小 hold_radius 壓散布」永遠失敗(需 37 m 半徑 vs 物理最小 206 m)。

**S 型機動(weave)就是那個延遲吸收器**,而且早就實作在 ALIGN 裡
(`tot_node.py` 搜 `mode = "weave"`),控制律 `cos θ = v_req/vmin` 是對的。
問題只是門檻(`vmin`)太低導致幾乎不啟動、週期(22 s)是泡棉機時代的值。

**weave 佔 ALIGN 的比例是關鍵指標**:P91 = 68.5%(成功)、RD1 = 12.3%(失敗)、
EL1 = 16.8%(失敗)、**SW1 = 98.2%**(改動後)。

---

## 5. 最新航次 SW1 的結果(2026-08-04)

設定:`VMIN=95 WEAVE_PERIOD=120 PERCH_CLIMB_V=100 TARGET_AMSL=0 DIVE_ANGLE=45`

### 成功的部分
- ✅ **海面基準生效**:z_g = −31.89 ~ −32.05(跨機散布 **0.16 m**,舊值 14–17 m)
- ✅ **weave 佔 ALIGN 98.2%**(舊值 12–17%)
- ✅ **抵達散布 18.3 s** —— 歷來最佳(RD1 36.9 s、EL1 118.8 s)
- ✅ **零接近告警**,ALIGN 全程最近水平接近 1601 m(P91 舊設定是 348 m)。
  擺幅放大 2.8× 沒有造成防撞問題,`layer_m` 暫時不必動
- ✅ **新的命中判定生效**:正確印出「⚠️ 名義命中,但 4/6 架在高空掠過(未落地)」

### 失敗的部分
- ❌ 對準內切時高度 `{1:3182, 2:3166, 3:3214, 4:2923, 5:3345, 6:3205}`,
  收斂 **245 m** vs 門檻 40 m,**等了 25 s 就放行**
- ❌ **實質 0/6**。摘要那組「命中高度 {1:448, 2:−16, 3:432, 4:573, 5:564, 6:2}」
  **不能照字面讀**(見下方逃逸鏈)。真正攻擊航次的逐機表現是:

  | UAV | 首航次最近水平距 | 離海面高度 |
  |---|---|---|
  | 3 | 22.6 m | 291 m |
  | 4 | 29.8 m | 639 m |
  | 1 | 35.2 m | 516 m |
  | 5 | 67.1 m | 404 m |
  | 6 | **468.9 m** | **1214 m** |
  | 2 | **638.1 m** | **1678 m** |

- ❌ **逃逸鏈(已驗證,是本趟真正的結構性缺陷)**:`impact` 集合同時承擔
  「命中」與「這架已處置完畢」兩個角色,**沒有 miss / timeout 出口**。
  U2/U6 首航次脫靶 638/469 m(> `switch_r`=300)⇒ `commit` 從未成立
  ⇒ `passed` 恆為 False ⇒ impact 不鎖存 ⇒ 節點持續發俯衝指令,
  `v_req = d/max(t_rem,0.05)` 飽和成滿速 ⇒ **U2 飛到離目標 207 km、U6 飛到 412 km**,
  數小時後隨機重新穿越紅心才鎖存,而鎖存瞬間恰在海面附近。
  航次因此跑 3.6 小時、CSV 1,480,764 列 / ~130 MB。

### 診斷結果(wf_48ae0976-831,已完成)

**20 條候選發現進入對抗式驗證,只有 2 條通過、18 條被推翻。**
被推翻的 18 條**涵蓋了所有「終端為什麼打不準」的候選根因**
(gate_r 1200 vs 4800、DIVE_ANGLE 45 vs 27、DIVE_PACE 115→50、航點段不下降、
off-boresight、hold_ring 半徑物理、PERCH 權限參數提前推送、weave 四條)。

⇒ **唯一站得住的兩條都在「記帳/判定層」,不在「導引/飛行層」。
終端精度的根因目前是空的。**

**四項改動逐項判定**:

| 改動 | 判定 |
|---|---|
| `TARGET_AMSL=0` | **有效**(限量測基準)。z_g 散布 0.16 m 屬實,但**對命中鏈無可歸因效果** |
| `WEAVE_PERIOD 22→120` | **證據不足(偏無害)**。支持它的三個數字未經驗證;反對它的四條候選也全被推翻 |
| `VMIN 45→95` | **證據不足,頭號待測**。同時影響 weave 觸發 / hold_ring 速度 / 俯衝配速三條路徑,**必須單變數 A/B** |
| `PERCH_CLIMB_V 65→100` | **證據不足**,零通過驗證的發現觸及 |

⚠️ 四項同時改、無 A/B,且與 P91 之間還差 GATE_R / DIVE_ANGLE / LINE_TOL 至少三項;
更嚴重的是 **P91 的 log 沒有「生效值」行而 SW1 有 ⇒ 兩趟之間 tot_node 改過版**。
在這種對照下任何單一參數歸因都不成立。

---

## 6. ⚠️ 已被實測推翻的說法(不要重複)

### 6.1 「45° 俯衝打不到,27° 才行」— 錯
RD1 的 PN 交接 h/d = **0.99–1.16**(45° 需要 1.00,幾何幾乎完美),實際俯衝角
做到 39–48°、降下 2514–3267 m。角度打得出來。真正的現象是 **PN 在目標上方
數百公尺就拉平**,不是角度做不到。

### 6.2 「RD1 的 h/d 是 2.41–2.75」— 這個數字是錯的
它來自更早的交接摘要,重算是 0.99–1.16。不要拿它當實驗判據。

### 6.3 「交接幾何 h/d 不對是回歸元凶」— 我自己提的,被自己的數據推翻
教訓:提出機制之前先算,不要用「合理」代替「量過」。

### 6.4 「地形資料庫的 z_g 散布 40 m,取平均就好」— 錯
平均掉的是誤差的一半,不是誤差本身。海上目標要用 `target_amsl` 釘死。

### 6.5 SITL 的「地面」不是海面
SITL 用同一份地形資料做碰撞,模擬世界裡目標處的地面在 −13 ~ −15 m AMSL,
飛機會掉到 `relative_alt ≈ −45 m` 才停。程式碼註解裡那個「紅心地表約 −45 m」
講的是這個,**不是驗收基準**。驗收一律以 **0 m AMSL** 為準。

### 6.6 更早的 8 項推翻紀錄
見 memory `stot-refuted-changes`,以及 `docs/stot_dive_authority_2026-07-28.md`。

---

## 7. 靜默失敗清單(這個專案最大的敵人)

使用者明確定調:**「之後不能再發生靜默失敗的問題了」**。已知的坑:

| 坑 | 症狀 |
|---|---|
| `-p param:=nan` | YAML 把 `nan` 解析成**字串**,節點型別檢查掛掉。NaN 要寫 `.nan` |
| `_kpi_parse.py` 的 CSV | 無條件 append,餵錯檔案靜默污染(已加防呆) |
| `/tmp/trial_launch.log` | 不分航次,只留最後一趟 |
| `pkill -x tot_node` | 平行跑兩趟時後啟動的會殺掉前一趟,且靜默(已改成記 PID) |
| `pgrep -f 'xxx.sh'` 當等待條件 | 會匹配到等待器自己 |
| 航向取不到前一點就退回「朝目標」 | 航向誤差恆為 0,四個傾斜角算出一模一樣的 t_min,看起來完全正常 |
| 座標慣例混用 | 數學慣例 vs 航空慣例,「正前方目標」會算成要轉 180° |
| `JET_CDP`/`JET_THRMAX` 空字串 | 幾十趟跑原廠阻力,該線之前的數據全作廢 |
| `catch_speed > AIRSPEED_MAX` | 速度指令**整發拒收**(MAV_RESULT=4) |
| 地形快取 / `head -9` 截斷 | 誤判「第六架沒就緒」 |

完整清單在 memory `sitl-scripting-traps`。

---

## 8. 文獻基礎(2026-08-04 新增)

我們的問題在文獻裡叫 **Impact Time Control Guidance (ITCG)** / cooperative salvo guidance。
綜整在 `docs/lit/synthesis_2026-08-04.md`,原始檢索結果 `docs/lit/*.md`(254 篇)。

**關鍵落差**:我們用開迴路幾何排程,文獻用**回授共識**。三個流派:

| 流派 | 代表 | 共識變數 | 需要 t_go 估計? |
|---|---|---|---|
| t_go 共識 | [Free-Will Arbitrary Time Consensus 2024](https://arxiv.org/abs/2407.15324) | t_go 估計值 | 需要 |
| range/range-rate 共識 | [MPC-Based Consensus, TAES 2018](https://doi.org/10.1109/taes.2018.2816880) | 距離與距離率 | **不需要** |
| 偏差追蹤 + 閉式 t_go | [Deviated Pursuit 2024](https://arxiv.org/abs/2402.05918) | t_go(精確解) | 不需要估計 |

三者都指出 **t_go 估計誤差是散布的主因**。

另外 [TAES 2020 (被引 164)](https://doi.org/10.1109/taes.2020.3037958) 把導引律
**沿 LOS 控時間、垂直 LOS 控角度**分開設計 —— 直接對應我們「調角度就壞時間」的困境,
是下一個值得實作的方向。

⚠️ 多數 Elsevier 論文只讀到摘要(ScienceDirect 對爬取回 403)。
`docs/lit/*.md` 裡標「(無摘要)」的那些**沒有讀過內容**。
Semantic Scholar 需要 `S2_API_KEY`(免費申請),Consensus 需付費帳號 —— 兩者本輪未納入。

---

## 9. 建議的下一步(依證據強度 × 風險 排序)

> 依據:多代理對抗式徹查 `wf_48ae0976-831`(26 個代理、20 條候選、2 條通過)。
> transcript:`.claude/projects/.../subagents/workflows/wf_48ae0976-831/journal.jsonl`

### M1 — 定義「首航次窗」,tot_node 與 stot_score 兩邊都套用
以 `dive_go` 為起點,窗外的接近不進 impact 統計。

⚠️ **實作陷阱(已驗證)**:`self.dive_T` 是**絕對牆鐘時戳**(`now + ... + 2.0`),
不是時長。條件要寫 `now > self.dive_T + 60`;寫成 `now - self.dive_go > self.dive_T + 60`
是**死碼**(拿 ≤12007 的時長比 ≈1.79e9 的 epoch,恆假)。
`dive_T` 初始為 None、「首架抵環」還會重排一次 ⇒ **使用前必須 None 檢查**,
否則會在摘要那行丟 TypeError 把整段結算炸掉。

**驗收**:同一份 `tot_SW1.csv` 重放 → U2/U6 印成 MISS;散布只由 U1/U3/U4/U5 算出,
應落在 **19 s 上下**(而非 11950 s)。

### M2 — 新增 `self.missed` 出口,**且必須計入 done**
⚠️ **最容易改壞的一步**:`done` 只在 `i in self.impact` 遞增,而摘要與 `phase="DONE"`
都被 `done == self.n` 擋著。**若只是把逃逸機排除出 impact,結果是永遠不印摘要、
永遠不進 DONE、航次不會自己結束 —— 比現在更糟。**

正確做法:`if i in self.impact or i in self.missed: done += 1`;
算 spread 時只取 `self.impact`;摘要把 missed 印成 MISS 而不是一個高度數字。

觸發條件(分離度極大,不會誤判):離目標水平距曾 **> 20 km**
(正常機全程最遠 4.1–5.3 km,逃逸機 207 / 412 km),
或 gs < 5 m/s 持續 > 60 s。

**驗收**:SW1 重放應在 `dive_go`+~250 s 內結案(而非 3.6 小時);CSV 從 ~130 MB 降到個位數 MB。
⚠️ 旁證(未二次驗證):`stot_trial.sh` 的 break 條件疑似 grep「全員命中」,
新的 MISS 摘要不匹配 ⇒ harness 的 break 條件要一起改。**改之前先確認**。

### M3 — 凍結 `dmin` / `hit_alt` 的時刻一致性
`dmin` 的更新沒有 `i not in self.impact` 保護,鎖存後仍持續變小
(U5 鎖存 217.7 → 最終 67.1;U6 20.9 → 6.1)。
⇒ 摘要裡的「命中高度」與「最近水平距」**來自不同時刻,不能配對解讀**。

### M4 — `stot_score.py` 套同一個窗
⚠️ **`stot_score.py` 不是獨立驗證** —— 它讀的是 tot_node 自己用 `self.pos` 寫出的
同一份 CSV,只是獨立計算。11950.42 vs 11950.65 只差 0.23 s 是**巧合**
(U3 差 26.0 s、U5 差 20.3 s 對不上)。**不能拿它交叉驗證 tot_node。**

### M5 — CSV 加 `agl` 欄或明確標註高度基準
徹查中至少一條候選發現因為混用 CSV `alt`(相對 HOME)與 log 命中高度
(相對紅心地表,z_g = −32.0)而把六個高度全部少算 32 m。純防呆。

### ❌ 明確**不要**做的

1. **不要只在 spread 計算後加「太大就警告」** —— 擋不住同一行的 2509636.9 m,
   而且把「兩架飛出 200–400 km」包裝成「判定失效」,是本專案最怕的
   「看似合理但錯」的下一步。
2. **不要調 `impact_reach` / `switch_r` / `passed` 門檻讓 U2/U6「算命中」** ——
   重放反證:放寬 `passed` 到 `dmin<800` 會讓 U2 在 **1498 m**、U6 在 **1069 m**
   高空被判命中,製造幽靈命中。
3. **不要在沒有 A/B 的情況下同時改回多個參數。** VMIN 是頭號待測,單變數跑。

---

## 9b. ⚠️ 還不知道的事(最重要的一節)

| # | 項目 | 狀態 |
|---|---|---|
| U1 | **終端殘餘高度 291–639 m 的根因** | **完全未定。**所有候選(GATE_R、DIVE_ANGLE、DIVE_PACE、航點段不下降、off-boresight)**全部被推翻**。這是任務失敗的第一因,而它現在沒有解釋。 |
| U2 | U2/U6 首航次為什麼脫靶 469 / 638 m | 未定。它是逃逸鏈的**觸發條件**,但觸發條件本身沒有解釋。 |
| U3 | 逃逸是否可重現 | 未知,n=1。 |
| U4 | VMIN / WEAVE_PERIOD / PERCH_CLIMB_V 各自的貢獻 | 未知,四項同時改且無 A/B。 |
| U5 | P91 vs SW1 的對照有效性 | **已知有問題** —— P91 的 log 沒有「生效值」行而 SW1 有 ⇒ 兩趟之間 tot_node 改過版,不只是參數差異。 |

**接手時請從 U1 開始,而不是從調參開始。**

## 10. 每趟航次的例行事項

- **航次結束立刻產素材**(`make_artifacts.sh` 已接進 `stot_trial.sh` 收尾)。
  資料是覆寫式的,不當場產就沒了。產物在 `~/fixwing_ws/runs/artifacts/<TRIAL>/`。
- **用 `SendUserFile` 把 GIF/PNG 主動送給使用者**,不要只給路徑 —— 他要做進度報告。
- **統計結論禁止基於單次航次下定論**,除非明確標注「初步觀察,待多次驗證」。
- **每個修復要附可驗證的判斷標準**(改完看什麼數字、達到多少算成功)。

---

## 11. 相關 memory(會自動載入索引)

`stot-acceptance-criteria`、`hit-criterion-ignores-altitude`、`sea-level-target-datum`、
`stot-spread-needs-loiter-orbits`、`stot-refuted-changes`、`sitl-scripting-traps`、
`jet-sitl-standard`、`jet-conversion-never-applied`、`progress-artifacts-required`、
`real-working-branch`、`dive-angle-ceiling`
