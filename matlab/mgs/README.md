# `matlab/mgs/` — 多機導引模擬環境 + WSL ROS 2 接口

依 MathWorks 官方教學
[*Designing a Guidance System in MATLAB and Simulink*](https://www.mathworks.com/help/simulink/slref/designing-a-guidance-system-in-matlab-and-simulink.html)
的四方塊分解建置,擴成 **N 機**,並接到 WSL 裡現行的 ROS 2 導引堆疊。

```
mgs_setup                    把 matlab/ 與 matlab/mgs/ 掛上路徑
mgs.selftest                 11 項檢查(逐位元對齊、契約新鮮度、閘門、shadow 歸因)
mgs.preflight                7 項檢查(WSL、ROS2、網路模式、橋接握手)
```

---

## 一、它跟官方教學的對應與差異

| MathWorks 教學 | 這裡 | 差異(**刻意的,不是漏做**) |
|---|---|---|
| Seeker/Tracker(0.05 s 追蹤迴路) | `mgs.seekerTracker` | 教學**估測**視線角速率;`tot_node.py:2824` 是從遙測**解析微分**,沒有 tracker 遲滯與雜訊。預設 `analytic`(照飛的那個);`tracker` 才是教學結構 |
| Guidance(PNG + Stateflow 模式邏輯) | `mgs.guidance` → `stot_guidance` | PN 律**不在**這裡,在 `matlab/stot_guidance.m`(`_pn_cmd` 的逐行移植)。這一層只決定相位 |
| Autopilot(三迴路加速度自駕儀) | `mgs.autopilot` → `stot_fcs_clamp` | ⚠️ **我們沒有加速度外迴路**。tot_node 下的是**姿態**指令,ArduPlane 收角度迴路。硬要對成三迴路會誤導每一個比較 |
| Airframe(fin actuator + 2-D 氣動表,Mach 2–4) | `mgs.airframe` → `stot_plant` | 次音速噴射 UAV、解析阻力極線(CD0=0.01500,由 1617 個飛行樣本反解)。沒有 fin actuator,因為舵機在角度迴路的集總遲滯裡 |
| 單發飛彈 | N 機共時鐘 | `mgs.simSwarm`,並產出散布/命中/終端速度 |

四個時鐘照抄 `stot_sim`,因為它們不是裝飾:

```
遙測  40 Hz (tel_us=25000)   節點被允許看到的東西
導引  20 Hz (TICK_PERIOD_S)  _pn_cmd
飛控  50 Hz (SCHED_LOOP_RATE) 韌體夾制鏈,ZOH
機體  連續,RK4,子步落在每個時鐘邊界上
```

### 逐位元對齊(這是整個環境能用的前提)

`mgs.simSwarm` 在 n=1 時與已驗證的 `stot_sim` **完全一致**:

```
stot_sim : t 39.819737619  err 0.081196716  V 178.452744
mgs      : t 39.819737619  err 0.081196716  V 178.452744
delta    : dt 0  derr 0  dV 0
```

`mgs.selftest` 第 1 項每次都重跑。**這條一斷,「多機環境繼承 32 架次驗證」這句話就作廢**,而且會安靜地斷 —— 兩邊都還是會畫出像樣的軌跡。

---

## 二、接口(這是重點)

### 一套 schema,兩種傳輸

```
c.io.transport = 'tcp'    ← 預設
    MATLAB(Windows)  ──tcpclient──►  NDJSON  ──►  mgs_bridge_node.py(WSL)  ──► ROS 2
c.io.transport = 'ros2'
    ROS Toolbox 直接講 DDS
```

**為什麼預設是 TCP 而不是 DDS。** 本機 WSL2 跑在**預設 NAT 模式**(沒有 `.wslconfig`,WSL IP 172.17.155.121)。DDS 的 multicast discovery **不會**跨過 NAT,所以 ROS Toolbox 開起來看得到自己、看不到 WSL 的節點 —— 而且**不報錯**,只是永遠收不到東西。那正是本專案最貴的那種失效形狀,所以 `mgs.Bridge` 的 `ros2` 路徑會**主動偵測並中止**,而不是讓你等。

TCP 反過來今天就通:WSL2 會把 Windows 的 `127.0.0.1:PORT` 轉發進 WSL,**不用改任何網路設定**。實測 RTT 64 ms。

要走原生 DDS,`%USERPROFILE%\.wslconfig`:

```
[wsl2]
networkingMode=mirrored
```

再 `wsl --shutdown`。⚠️ 會影響**所有** WSL 網路行為(含 SITL 的埠)。`mgs.preflight` 會告訴你現在是哪一種。

### 三種模式

| `c.io.mode` | 做什麼 | 會不會動到飛機 |
|---|---|---|
| `shadow` | MATLAB 訂閱現場、平行跑同一套導引,比對分歧 | **不會**。`sendCmd` 會直接報錯 |
| `sil` | MATLAB 算導引指令,發到 `/mgs/cmd` | 由 WSL 端決定要不要吃 |
| `offline` | 完全不連,`mgs.simSwarm` 單機跑 | 不會 |

### 線上訊息(NDJSON,一行一個 JSON)

**MATLAB → WSL**

| type | 落到哪 | 說明 |
|---|---|---|
| `hello` | — | schema 握手,版本不合**直接斷線** |
| `shadow` | `/mgs/shadow` | 模型的 N 機狀態,導引拍頻 |
| `cmd` | `/mgs/cmd` | SIL 指令 |
| `target` | `/swarm/tot_target` | ⚠️⚠️ **會讓真飛機動**。預設關閉,要 `--allow-target` |
| `contract` | — | 觸發現場生效值讀回 |
| `ping` | — | |

**WSL → MATLAB**

| type | 來源 |
|---|---|
| `hello_ack` | schema / ROS distro / 訂到的 topic 清單 / `have_cmd_msgs` |
| `status` | `/swarm/tot_status` 原樣轉發 |
| `gate` | `/swarm/tot_gate` 原樣轉發(latched QoS 有對上) |
| `state` | `/uavI/mavros/*` 逐機狀態,**20 Hz**(= 節點 tick,見四之二),每筆帶 `age_s` |
| `contract` | `contract_verify.py --snapshot` 的完整輸出 |
| `error` / `pong` | |

`state` 的 `age_s` 不是裝飾:一筆看起來正常但其實 40 秒前的狀態,和一筆新鮮的長得一模一樣。長機位姿凍結那次就是這樣過去的。

### 啟動

WSL:

```bash
source /opt/ros/humble/setup.bash
python3 matlab/mgs/bridge/mgs_bridge_node.py --port 5799 --n 6
```

指令側(`setpoint_raw/target_attitude`、`nav_controller_output`)是**分開一個
try 匯入**的:拿不到 `mavros_msgs` 時狀態側照樣跑,`hello_ack.have_cmd_msgs`
會是 false,而 `mgs.shadowDiff` 會拒絕替那一層評分 —— 不會把「沒錄到」報成
「沒有分歧」。

MATLAB:

```matlab
mgs_setup
mgs.preflight          % 先看七項檢查
b = mgs.Bridge(mgs.params()); b.connect();
```

---

## 三、「如果這樣設定,MATLAB 會怎麼跑」

這是 `mgs.fromContract` 的用途。**同一份 JSON**,`contract_verify.py` 拿它擋航次,模型拿它當參數:

```matlab
c = mgs.fromContract('RRE8');                 % 某趟的讀回值
c = mgs.fromContract(b.contract());           % 現場即時讀回
c = mgs.fromContract(struct('gate_r', 5200)); % 假設性設定
out = mgs.simSwarm(mgs.scenario(c), c);
```

三件事會依序發生,一件都不能少:

1. **對契約表評分**(`mgs.contractCheck`)。泡棉禁區值、超出區間、非數值 → 預設**中止**。
2. 有對應的名字寫進參數。
3. **沒有對應的名字被逐一唱名**。RRE8 有 47 項,模型吃 17 項、**明確報出被丟掉的 30 項**:

```
Warning: 30 of 47 contract values are NOT consumed by this model and were DROPPED:
    formation_alt, patrol_alt, dive_pace, reach, ... , READY_AGL
If one of them is the knob you are studying, this model cannot answer the
question and you must not read its output as if it could.
```

「我設了 dive_pace 而模型沒理它」和「我設了 dive_pace 而節點沒理它」是同一種失效,必須一樣大聲。

### 契約表只有一份

`contract_table.json` 是**產生物**,由 `contract_table.py` 匯出:

```bash
python matlab/mgs/bridge/export_contract_table.py
```

JSON 帶著來源 `.py` 的 sha1,MATLAB 端每次載入都**重新雜湊比對**,不合就中止。手抄一份區間或禁區值 = 兩份會漂移,而漂移過的表照樣會跑、照樣出數字、結論是錯的。

⚠️ 46 條單值契約在 MATLAB 端檢查;**17 條跨參數不變式沒有**(它們是 Python callable)。`mgs.contractCheck` 會明講「N 條未評估」並指向 `contract_verify.py`,不會假裝清過了。

---

## 四、⚠️ 這個模型現在**打不出失效**

沒開缺口時它 8/8 全中、誤差 0.02 m。真實不是這樣。`c.gam.*` 是注入那個落差的掛勾。

### ⚠️ 那組常被引用的數字已經被推翻,而且訂正影響這裡

「γ 追蹤增益 0.063 / 遲滯 8 s / 26 架觸頂」**大部分是記帳假象**:

- LOS 地板排在 nz 夾制**之後**取 `min()`,它的輸出是**角度**指令,**整個繞過**夾制;
- 把那個角度階躍除以 `pn_lead = 1.0 s`,就被記成高達 **−61 °/s** 的角速率需求;
- dataflash 說內迴路好得很:姿態誤差 1.5–4.8°、**角速率環誤差 0.000**、沒有任何限幅在作用。

改用**角度迴路**重擬(它本來就是角度迴路):

| 取樣 | k | τ |
|---|---|---|
| 全部拍 | 0.0661 | **15.1 s** ← 這就是遲滯擬合 26/32 卡在 8 s 上限的原因 |
| 可行拍 | **0.2916** | **3.43 s**(corr +0.552) |

—— 一個很普通的一階角度追蹤迴路。`c.gam` 的預設值是**可行拍那組**,不是假象那組。

建置期間獨立撞到同一件事:把假象那組直接餵進去給出 **0/8、脫靶 800–960 m**;連增益開到 1.0(等於不衰減)都還差 205 m —— 主導的是那個 8 s 遲滯,而那個 8 s 就是上面那個記帳問題。RRE 實際飛出 15/16。

### ⚠️ 而重擬那組**也還是對不上**,這件事本身有資訊

把重擬值(0.2916 / 3.43 s)當成俯仰指令路徑上的一階遲滯注入,n=8 得到:

```
0/8 命中,脫靶 522–600 m        (假象那組是 0/8、800–960 m)
```

實際 RRE 飛出 15/16。⇒ **這個注入結構本身是錯的,而且錯在太嚴苛。**

這跟 dataflash 的結論是一致的:內迴路**追得到**(角速率環誤差 0.000、無限幅),所以把落差建模成「俯仰路徑追不上」在方向上就不對。落差比較可能在**需求那一側**(地板繞過夾制、需求本身不可行),不在追蹤那一側。

⇒ 這是**下一個要做的事**,不是已經做完的事。

### 它預設拒絕執行

```
c.gam.deficit_on = true
  → mgs:autopilot:gam_uncalibrated
```

兩條路:拿 `matlab/gammadiag/chain32.csv` + `matlab/data/truth32.csv` 擬到模擬脫靶母體對得上實測;或設 `gam.acknowledge_uncalibrated = true` 當成下界跑,並知道輸出不是預測。

**在那之前,這個環境的命中率數字沒有意義,散布與夾制佔比才有。**

---

## 四之二、Shadow 逐拍比對(`mgs.shadowDiff`)

回答的是「模型在**哪一層**跟真飛機分家」,不是「有沒有分家」—— 後者從來不可行動。

```matlab
mgs.recordStream(c, 'runs/shadow/RRX.ndjson', 'seconds', 600);   % 錄
D = mgs.shadowDiff('runs/shadow/RRX.ndjson', c);                 % 比
```

### ⚠️ 自由前推**無法歸因**,這是量出來的不是猜的

第一版是「從放行狀態播種、讓模型自己飛、比兩條軌跡」。實測:把 `pn.los_floor` 從 1.0 改成 0.5,四層的 rms 是 **0.518 / 0.518 / 0.513 / 0.510** —— 四個一樣。改阻力係數也一樣。兩台飛機一旦處在不同**狀態**,每一層量到的都是同一個狀態差。

所以改成**逐拍、每一層都餵現場輸入**(一步預測誤差):

| 層 | 模型端 | 現場端 | 分歧代表 |
|---|---|---|---|
| **L1 導引** | `mgs.guidance(現場量測)` | `setpoint_raw/target_attitude` | **律**不一樣 |
| **L2 夾制** | `mgs.autopilot(現場指令, 現場狀態)` | `nav_controller_output` | 我們的 ArduPlane 夾制模型錯 |
| **L3 機體** | `mgs.airframe(現場狀態, 現場致動)` | 現場狀態的中央差分 | **機體**錯 ← γ 掛勾屬於這一層 |

⚠️ L1 **不是完全乾淨的隔離**:律會讀**量測俯仰**做攻角估計(`tot_node.py:2871`),所以下游造成的姿態差會回饋進 L1。實測:純夾制層注入仍讓 L1 動 rms 4.34°,但**晚 0.1 s**。所以歸因靠的是首次超標的**順序**,不是某一層完全安靜。

### 三個把它做對的細節

**(1) 錄製頻率要對上節點的 20 Hz。** α 低通的增益是 `tick_dt/alpha_tau`,重放頻率一偏就是**另一個濾波器**。實測 L1 空測殘差:20 Hz **0.013°**、10 Hz 0.037°、50 Hz 0.023°。20 Hz 最低,也是唯一有原理依據的;但**對 50 Hz 的差距不大**,所以頻率不符時該懷疑的是「小的 L1 讀數」,不是把大的也一併否定。橋接預設已改成 `--state-hz 20`,不一致時 `shadowDiff` 會出聲。

**(2) 絕對門檻與相對門檻**都要超過才算分歧。只用相對的會在空測產生假陽性(俯仰指令本身很平滑,scale 只有 0.18°,0.03° 的殘差就變 17%);只用絕對的會偏袒擺幅大的那一層。絕對底線取自實測空測噪聲的約 15 倍。

**(3) 跳過前 0.5 s。** 放行後第一拍兩邊都是啟動瞬態,那**單一個樣本**曾把 L2 的 rms 從 0.04 拉到 0.97。

### 工具本身怎麼驗

`mgs.synthStream` 產生**已知答案**的假現場流,`mgs.selftest` 第 9a/9b 每次重跑:

```
[PASS] shadowDiff null case shows no divergence  attribution "none"; rms [0.013 0.011 0.003 0.026]
[PASS] shadowDiff attributes a known injection   guidance->L1, clamp->L2, plant->L3
```

**空測必須是 "none"** —— 一個在空測就報分歧的歸因工具,之後每一個判定都不值錢。

---

## 四之三、現行導引律**有**時間收斂,但被飽和框住(`mgs.timeAuthority`)

先前(含本文件先前版本)的說法是「俯衝段幾乎原樣傳遞上游時間差,沒有修正能力」。
用逐位元對齊的模型量過之後,**那個說法要限定**。

標稱放行狀態(r0 6760 m、z0 3100 m、V0 157 m/s、γ0 −27°),掃 `dive_T`:

| `dive_T` | `t_impact` | `d(t_impact)/d(dive_T)` |
|---|---|---|
| ≤ 34 s | **釘死 35.816 s** | **0.000** |
| 36–46 s | 35.95 → 45.22 | **0.84–0.97** ← 迴路在這裡是有效的 |
| ≥ 52 s | **釘死 47.221 s** | **0.000** |

⇒ **可達抵達窗 = [35.82, 47.22] s,寬 11.41 s。**

窗外的 0.000 不是「增益太小」,是**飽和** —— 機體到不了,調參救不了。早端
`thr=100%` 只佔 6%(不是油門飽和,是 `v_req` 被 `pn_vmax` 夾住);晚端 `thr=0%`
佔 100%(重力主導,慢不下來)。

### 三個決定尺度的關係

1. **窗以 `R^1.70` 收縮** —— r0 3000→**3.35 s**、6760→11.41 s、12000→35.39 s。
   交接當下有的權限,等到發現需要時已經沒了。
2. **早端由 `pn_vmax` 決定** —— 150 ⇒ 窗只剩 **0.72 s**;210 ⇒ 11.41 s;320 ⇒ 19.24 s。
   晚端不動。⚠️ 拉高會把終端速度推出阻力極線校準過的 121–219 m/s 帶,寬窗那幾個
   數字是**外插不是預測**。
3. **實際收斂率**(兩架同抵達時刻、放行差 dt,觸地保留的比例):

   | 放行差 | 0.5 s | 1 s | 2 s | 3 s | 5 s | 8 s |
   |---|---|---|---|---|---|---|
   | 保留 | 17% | 15% | 10% | **8%** | 68% | **147%** |

   小誤差移除約九成;大誤差原樣通過甚至**放大**。
   ⚠️ dt ≥ 5 的兩欄被**未校準的守環相位**污染(第二架要先 hold dt 秒),只當指示。

### 這訂正了什麼

`tot_node.py:630` 專案自己的迴歸是「散布 = 0.76 × 交接時間差 + 1.62(R²=0.779,
n=5)」。斜率 0.76 跟上表 dt=5 的 68% 對得上 ⇒ **那 5 趟是坐在飽和區**,而不是
「律沒有時間迴路」。把交接差壓進 ~3 s,現有的油門迴路就會移除九成 —— 這跟專案
自訂的「交接時間差 ≤1.8 s」是同一個要求。

### 通道盤點(獨立 grep 確認)

- `tot_node.py:3168` 自述:「油門:**唯一的**命中時刻通道,與兩個轉向通道解耦」
- RRE8 契約 `itcg_on = 0.0` ⇒ 橫向時間項是關的
- 完全不下抵達時刻時,放行差在觸地被**放大成 176–219%** ⇒ PN 通道不只是「不修正」

✅ **「Guo 2020 Eq. 33」的來源已於 2026-08-22 找到並逐字核對。**

Guo Yuheng、Li Xiang、Zhang Houjun、Cai Ming、He Feng,
*Data-Driven Method for Impact Time Control Based on Proportional Navigation
Guidance*,北京理工大學。推導鏈:Eq.(31) `t_f = t + t_go^PNG` → Eq.(32)
`ṫ_go^PNG = −1` → **Eq.(33) `ė_tgo = 0`**。原文逐字:

> "if only PNG is adopted, the etgo keeps unchanged; that is, the etgo at the
> beginning of PNG trajectory is the same as the etgo at the end of the PNG
> trajectory; etgo cannot be eliminated."

⚠️ 我先前把它判成「無來源」是**查錯架子**:PDF 不在 repo,但在 STOT計劃筆記本裡
(source `c3644548-7fb0-4d9e-a336-59e7b10cdb51`,`guo2020.pdf`,55 813 字元 ——
遠高於交接文件 §六.4 的 3000 字偽造判準)。結論三行可推,所以推導留著,
**但出處要掛回去,那不是我們的結果。**

### ⭐⭐ 而且同一篇的 Eq.(34)–(37) 直接打到我們的天花板

Eq.(34) 明列**三條**產生非零 `ė_tgo` 的路:縱向加 `Δγ̇`、橫向加 `Δψ̇`、或兩者。
論文選了哪一條,以及**為什麼**,是逐字的:

> "To avoid complex coordination of the two channels, the extra guidance command
> is added in **lateral channel** in this study while the vertical channel keeps
> the same"

⇒ Eq.(36) `γ̇ = k₁λ̇`(縱向 = 純 PN,原封不動)、Eq.(37) `ψ̇ = k₂θ̇ + Δψ̇`。

**他把全部時間權限放在橫向,而那正是我們量到已經飽和的通道**
(契約 ≤1.0 g、坡度夾制 1.49 g 且與 PN 共用,n=8 需要 1.26–1.34 g)。
關鍵在於他選橫向的理由是原文寫的 *"to avoid complex coordination"* ——
**是為了省事,不是因為橫向權限比較大。** 所以「照抄 DD-ITCG」對我們是死路,
但那是**他們的取捨**碰巧撞上我們的天花板,不是 DD-ITCG 這個方法的限制。

---

## 五、Zarchan《Tactical and Strategic Missile Guidance》

`matlab/zarchan/` 已經有 7 份逐行移植與 21 PASS 的重現套件。`matlab/mgs/+mgs/+zarchan/` 是把它接成**活的參考基準**,不是第三份副本。

書的全文現在放在 `matlab/zarchan/_source/zarchan.txt`(15,120 行,已 gitignore —— 版權書籍不進 git)。先前 README 引用的行號指向**上一個 session 的暫存目錄**,會被回收;四個抽查行號(409 / 650 / 10887 / 10930)在新位置全部對得上。

| 工具 | 做什麼 |
|---|---|
| `mgs.zarchan.crosscheck(out)` | 同一場交戰同時跑 `purepn2d`(Listing 30.1)與我們的全非線性模型,**並附上夾制佔比來解釋落差** |
| `mgs.zarchan.sizing(out)` | Ch.2 需求/可用加速度比,**氣動與夾制兩個天花板分開報** |
| `mgs.zarchan.additiveLaw` | 他的**加法式**終端角項,取代我們的單邊 `min()` |

### 那個 `min()`

`tot_node.py:2866` 用單邊最小值合併 PN 需求與終端角需求:

```
gam_cmd = min(gam_cmd_pn, gam_los * los_floor)
```

Zarchan **四個地方都用加法**(Ch.24 `8109`、Ch.28、Ch.30 `10887`、附錄 A `14446`)。Ch.30 那一行:

```
nc = 4*Vc*lamdot + 2*Vc*(lam - gamF)/tgo      ← 加起來
```

差別不是形式,有**兩個**:

**(1) `min()` 在生效那一側梯度為零。** 地板一旦是較小的那個,移動視線對指令的影響是**恰好 0**。`mgs.selftest` 第 4 項量了:地板生效時把 `n_vert` 加倍,`gam_cmd` 移動 **0 rad**。而地板在標稱情境佔了 82–87% 的俯衝,實測全集 43.1%、四架脫靶的 89.8–99.7%。

**(2) ⭐ `min()` 排在夾制之後,所以終端角需求整個繞過 `[nz_min, nz_max]`。** 這正是第四節那個「0.063」記帳假象的來源,也是 `cmd_nz` 會出現 −18.28 g 的原因。加法式把整形項放在**夾制之前**,總需求因此**受夾制管轄** —— 而那也正是 Zarchan 的做法:他一律夾制**總指令**,從不只夾 PN 那一段。所以這個導入同時解掉旁路問題,不只是換個組合器。

用法:

```matlab
c = mgs.params('pn.law', 'zarchan_additive', 'pn.shape_gain', 2.0);
```

⚠️ **這條律從來沒飛過**,32 架次的驗證一項都不轉移過去。他的 2.0 是跟 N′=4 從同一個最佳控制問題出來的,而我們飛 n_vert=5 —— 那兩個增益怎麼配,書上沒處理。

⚠️ **不要用 `adjoint_miss.m` 裁決任何事。** 伴隨法要求線性、單一一階遲滯、處處不飽和,我們三項全違反,而它會為一個不像我們的系統畫出一條乾淨又有說服力的曲線。細節在 `matlab/zarchan/README.md`。

### 兩份導引律副本怎麼不漂移

`additiveLaw` 是 `stot_guidance` 的衍生副本。`mgs.selftest` 第 2 項釘住它們:`shape_gain = 0` 且 `los_floor = 0` 時,兩者輸出必須**逐位元相同**。有人改了 `stot_guidance` 卻忘了這裡,測試會紅,而不是兩條律安靜地分家。第 3 項反向釘:項打開時指令**必須**真的動(接好線卻毫無作用的旋鈕,跟能用的旋鈕長得一樣 —— `itcg_accel` 80,760 個樣本全零就是這樣過去的)。

---

## 六、Simulink 模型

```matlab
mgs.slx.build(c, scn, 'open', true)     % 產生 models/mgs_multi.slx
```

四個 MATLAB System 方塊(**Interpreted execution**,因為底層那些 `error()` 正是重點,codegen 會把它們拿掉)+ 連續積分器。所有訊號都是 N×k。

⚠️ **數值保真度**:Simulink 用它自己的解算器,`mgs.simSwarm` 用抄自 `stot_sim` 的定步長 RK4 + 割線觸地搜尋。**要和 32 架次真值對齊的數字,用 `mgs.simSwarm`。** `mgs.selftest` 第 9 項比對兩者(實測觸地時間差 < 0.1 ms),是**聲明容差**,不是假裝相同。

---

## 七、驗收指標(`mgs.kpi`)

四項門檻:彈射間隔 3 s、編隊成形、命中 ≤10 m、散布 ≤3 s —— 而且散布是 **8 架**打完,不是 16。

以及一條凌駕其他的排序:**逐機真觸地率先於散布**。`k.spread_valid` 在命中率沒過 90% 時是 `false`,因為在一半會脫靶的母體上算散布,不是一個關於散布的數字。

散布會拆成兩塊:

```
spread_s = spread_release_s + spread_dive_s
```

**這是唯一能判斷該修上游還是下游的工具。** 但要分開兩件事(先前這裡混在一起、結論是反的):

- **PN 通道本身不帶時間資訊。** 純 PN 對靜止目標,`d(t_go)/dt = −1` 精確到 O(σ²) ⇒ 放行時刻差原樣傳過去。本模型實測:**完全不下抵達時刻**時,1–5 s 的放行差在觸地被**放大成 176–219%**。
  出處:Guo 等,*Data-Driven Method for Impact Time Control Based on PNG*,**Eq.(33) `ė_tgo = 0`**,2026-08-22 逐字核對過(詳見第四節)。⚠️ 我先前把它判成無來源是查錯架子 —— PDF 不在 repo,但在 STOT計劃筆記本裡。
- **我們的律不是純 PN**,它有油門時間通道(`tot_node.py:3168` 自述「唯一的命中時刻通道」),而且**那個通道是有效的** —— 只是被飽和框住。見第四之三節的實測數字。

命中同時看水平誤差**與**殘餘高度。只看水平的「全員命中」曾經其實是六次高空掠過。

終端速度**總速與地速分開兩個欄位**。現場管線曾經在 16 架次裡有 12 架把總速悄悄退化成地速(dt=0 讓 vz 讀成 0),157–215 變成 140–189。兩個具名欄位不花什麼成本,而且讓那種替換在這裡不可能發生。

---

## 八、還沒做 / 已知限制

- **γ 追蹤缺口未校準** —— 第四節。這是最重要的一項。
- **上游只有 `'ic'` 這條路是驗證過的。** `'ring'` 未校準,而且方向已知是錯的:指令的守環(95 m/s / 1300 m)**從來沒被飛過**,實飛 124–186 m/s / ~5867 m 直穿。`c.up.ring_honest = true` 改成模型化實飛的那個環。任一種都會把 `out.calibrated` 打成 false。
- **PN 接手前的爬升沒有模型。** 真值集只從放行看起,而 `mode=="dive"` 比 PN 開口早 8.1 s,那段是 L1/TECS 在飛 —— 兩組全部 6 個失效都是「PN 接手前已在爬升」的機體。這裡爬升是**輸入**(設 `ic.gam0_deg > 0`),不是湧現行為。
- **17 條跨參數不變式在 MATLAB 端沒檢查。**
- **n > 2 的蜂群行為未驗證。** 逐機俯衝鏈仍然是驗證過的那一條;蜂群層不是。真值集是 n=2,而 n=2 是「一長一備長」。

---

## 九、檔案

```
mgs_setup.m                掛路徑 + 檢查已驗證鏈在不在
contract_table.json        產生物,帶來源 sha1
+mgs/
  params.m                 生效值快照(cfg.params + swarm/trk/up/io/gam/scen)
  fromContract.m           讀回值 → 參數,含唱名未消費項
  contractCheck.m          泡棉禁區 / 區間 / 缺項
  contractTable.m          載入 + 新鮮度閘門
  scenario.m               N 機放行狀態
  seekerTracker.m          ┐
  guidance.m               │ 官方教學的四方塊
  autopilot.m              │
  airframe.m               ┘
  simSwarm.m               N 機主迴圈(四時鐘 + RK4 + 割線觸地)
  kpi.m                    驗收指標
  Bridge.m                 接口(tcp / ros2)
  preflight.m              7 項連通性檢查
  selftest.m               9 項正確性檢查
  +slx/                    Simulink 方塊 + build.m
  +zarchan/                crosscheck / sizing / additiveLaw
  +util/sha1file.m
bridge/
  mgs_bridge_node.py       WSL 端 rclpy ↔ TCP(⚠️ 必須 LF 換行)
  export_contract_table.py 契約表匯出器
models/mgs_multi.slx       產生物
demo/
  demo_offline.m           「如果這樣設定會怎麼跑」
  demo_live_shadow.m       接現場、串影子
  demo_zarchan.m           書當參考基準 + 加法式終端角
```


---

## 六、6-DOF 位姿視覺化 `mgs.viz6dof`(2026-08-22)

```matlab
mgs.viz6dof(out)                                   % mgs.simSwarm 的輸出
mgs.viz6dof('runs/artifacts/BASE21/tot_BASE21.csv')  % 真實航次
mgs.viz6dof(p, at=595, glyph=1.2, trail_s=45)      % 單張定格(報告用)
mgs.viz6dof(p, export="mp4", file="runs/artifacts/viz/engage.mp4")
```

畫面:左側 3D 交戰場景(機身圖示 + 機體三軸 + 航跡 + 視線 + 海平面紅心),
右側三格同步時序(γ/θ、n_z req/cmd 與夾制帶、地速/總速與 157 / 204 兩條驗收線),
下方**自由度來源帳**。

### ⚠️⚠️ 放進報告之前一定要看這一段

**這是六個自由度的「位姿」,不是 6-DOF 剛體模擬。** 底下跑的是
`mgs.airframe` 自己的檔頭寫的 *"3-DOF point mass + ArduPilot angle loop"*:

| 量 | 狀態 | 說明 |
|---|---|---|
| `n, e, z` | **DYN** | 平移運動方程,`CD0 = 0.01500`(1617 個飛行樣本反解) |
| `θ` | **DYN** | 一階角度迴路,`tconst = fc.PTCH2SRV_TCONST` |
| `φ` | **DYN** | 一階角度迴路,`tconst = fc.RLL2SRV_TCONST` |
| `ψ` | KIN | 協調轉彎;不是獨立狀態 |
| `p, q, r` | **無** | 沒有機體角速率、沒有力矩方程、沒有慣量張量 |
| `β` | **無** | 沒有側滑;傾斜轉彎是假設 |

⇒ **標準分類是 5-DOF。** 圖上那張帳會自己印出來,所以截圖自帶但書 ——
那是刻意的,不是裝飾。

要做真 6-DOF 需要這個機體的慣量張量與舵面導數,而
[[barracuda500-model-facts]] 記著 CAD 與 SITL 實際飛的**不是同一個機體**;
硬套發明的數字還會破壞 `mgs.simSwarm == stot_sim` 的逐位元 parity。

### ⭐⭐ 這個模組最重要的設計:**它會否決自己**

視覺化是「捏造的數字不像數字、像圖」的唯一地方,而**沒有人會回頭重推一張圖**。
所以 φ 這一路有一道會自我否決的閘門:

* 2026-08-22 之前的航次**從來沒有記錄過 roll**,只能用協調轉彎反解
  `tan φ = V·ψ̇ / (g·cos γ)`;
* 但 ψ 本身已經是**位置微分**來的,再微分一次是二階差分 ⇒ 20 Hz 的
  lat/lon 量化雜訊被放大到不可信;
* 實測 BASE2 俯衝段反解出**中位 56°**、4% 打到 80° 限幅,而節點自己送出的
  `|roll|` 中位是 **0.93°**(32 架次);
* ⇒ 這時**不畫**:φ 歸零、帳上標成 `[NONE]` 並附上那兩個數字。
  一條平滑但錯誤的姿態曲線比沒有姿態更貴。

`mgs.selftest` 第 9 項就是在鎖這個行為,不是鎖「跑得動」。

⚠️ 判斷刻意做在 `focus` 裁切**之後**:整趟含 ~60 km 巡航(坡度小)會把俯衝段的
雜訊稀釋掉,在整趟上判「合理」然後畫一段其實不合理的。

### 其他幾個不是裝飾的細節

* **`focus`**:飛行資料預設只留 `mode=="dive"`,否則 60 km 轉場會把終端交戰
  壓成一個點。裁掉的區間會寫進標題與帳上(`[CUT]`),不靜默。
* **兩條驗收線都畫在地速上**(157 / 204)。畫到總速上會憑空好看 8–25 m/s
  ——[[eas-vs-groundspeed-157]]、[[terminal-204-is-groundspeed]]。
* **`n_z` 面板用穩健分位數定範圍**並標明裁掉多少:末端 `cmd_nz` 由 γ 指令反解,
  凍結前會噴到 ±20 g 以上(BASE2 實測最低 −20.96 g),讓 autoscale 跟著它跑
  會把整段有訊息的區間壓成一條線。
* **目標座標找不到就報錯**,不猜:每個位置都是相對它算的,datum 錯了整張圖
  仍然看起來完全正常。會自動找同目錄的 `*_geom.csv`。
* 三格時序**共用 x 軸**:`n_z` 只在俯衝段有值,各自 autoscale 會讓人以為那是
  同一段時間。

### 兩種樣式、兩種機身

```matlab
mgs.viz6dof(p, style="dash")                    % 完整儀表板(預設)
mgs.viz6dof(p, style="simple")                  % 素色主視覺,只有 3D + 一行但書
mgs.viz6dof(p, model="cad")                     % 真的 Barracuda-500 CAD
mgs.viz6dof(p, model="cad", style="simple", glyph=4.0)   % 報告主圖
```

`model="cad"` 走 `mgs.loadGlb` —— **純 MATLAB 的 glTF 2.0 binary 解碼器**,
零工具箱相依(`mgs.viz6dof` 整支也是,全部解析到 `toolbox/matlab`)。
338,282 → 1,500 三角形,載入 1.79 s。

⚠️⚠️ **CAD 不是飛出這些資料的機體。** 所有航次都是 ArduPilot SIM_Plane 的通用
`-jet`(m 22.0 kg、b 1.88 m、S 0.45 m²),而 CAD 展長 2.325 m、全長 3.786 m。
畫 CAD 等於畫一架**氣動從沒被模擬過**的飛機 —— 帳上會寫明,當插圖沒問題,
當「已驗證模型」就不行。

#### ⭐ 軸向是量出來的,而且有一半不可觀測

glTF 2.0 是 +Y 上、右手系,但這個模型的**機身在 X 上**(非 glTF 的 forward 慣例),
所以映射只能從網格自己建立。沿 X 每 0.25 m 量剖面:

| X 區間 | 機身 r95 | |
|---|---|---|
| +1.85..+2.10 | **0.076** | 收成一個點 |
| +0.10..+0.60 | 0.344(\|Z\|max **1.162**) | 主翼 |
| −1.90..−1.65 | **0.451** | 鈍端,最粗 |

⇒ **+X 是機鼻。** 記憶 `barracuda500-model-facts` 原本記載這一項「必須在 Gazebo
裡目視確認」,現在用幾何解掉了。佐證:X≈+1.17 有一片薄圓盤、Y 偏負(下方)
= 腹部進氣口。

    [bx by bz] = [+gx +gz -gy]      body = x前 / y右 / z下

數值驗證 `dot(機鼻向量, 速度向量) = 1.0000`。
⚠️ 但 **`+gz` 對到哪一邊機翼無法觀測** —— 機體左右對稱,沒有任何量測分得出來。
右手系只固定了算術。對稱機體畫起來一樣,**不要搬到在乎手性的地方**。

⚠️ `mgs.loadGlb` 對 Draco/meshopt 壓縮、sparse accessor、byteStride 交錯、
節點變換、非 TRIANGLES **一律報錯而不是盡力而為** —— 解錯會畫出一個看起來
合理的錯形狀,而那比讀不進來貴得多。

### 產物

    runs/artifacts/viz/viz6dof_simple_cad.png / .mp4   ⭐ 報告主視覺(CAD + 素色)
    runs/artifacts/viz/viz6dof_dash_cad.png            完整儀表板 + CAD
    runs/artifacts/viz/viz6dof_sim_cad.png             模型 n=6 + CAD
    runs/artifacts/viz/viz6dof_flight.png / .mp4       BASE2 真實航次(內建示意體)
    runs/artifacts/viz/viz6dof_sim.png    / .mp4       模型 n=6(內建示意體)
