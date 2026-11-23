# JSBSim 後端 —— 安裝、接線、與目前的狀態(2026-08-26)

## 為什麼要接 JSBSim

我們現在的動力學是 **3-DOF 質點 + ArduPilot 一階角度迴路 = 標準分類的 5-DOF**
(見記憶 `model-is-5dof-not-6dof`)。`matlab/mgs/README.md:501-509` 明文記著
我們**沒有** `p, q, r`、沒有力矩方程、沒有慣量張量、沒有舵面導數。

那正是 2026-08-26 那次「內迴路到底是不是瓶頸」查不下去的地方:
實測 γ̇ 追蹤比只有 **0.10–0.31**、`PIDP.Tar` p95 頂到 RMAX 90 °/s,
但升降舵飽和 **0/32 趟** —— 要判斷是「速率迴路增益不足」還是「機體慣量
就是這樣」,需要一個真的有力矩方程的模型。JSBSim 提供那個。

⚠️⚠️ **接 JSBSim = 換物理後端。** `jet_model.json` / `JET_CDP` / `JET_THRMAX`
**完全不會被讀**,既有 KPI 基線不可延用 —— 與記憶 `gazebo-backend-facts`
記載的是同一件事。

---

## 安裝(已完成)

| 項目 | 值 |
|---|---|
| 版本 | JSBSim **v1.2.2**(釘 tag,非 master) |
| 原始碼 | `~/jsbsim`(WSL Ubuntu 22.04) |
| 執行檔 | `~/.local/bin/JSBSim` |
| 建置 | `cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_MODULE=OFF -DBUILD_SHARED_LIBS=OFF` |

apt 沒有 jsbsim 套件;pip 的 `jsbsim` 是 **Python 模組**,不是 ArduPilot 要的
獨立執行檔(`SIM_JSBSim.cpp` 是 `execlp("JSBSim", ...)`),所以只能自己建。

⚠️ `~/.local/bin` 已加進 `~/.bashrc` 的 PATH。非互動 shell 要自己
`export PATH="$HOME/.local/bin:$PATH"`。

---

## 埠位(**已確認不與現行航次衝突**)

JSBSim 每個 instance 用 `5504+10i`(FDM)與 `5505+10i`(control)。
我們現有的:`5760/5860/6760/6860/6960 + 10i`,以及 `5762+10i`(契約讀回)。
⇒ i=0..7 時 JSBSim 落在 **5504–5575**,與上述**全部不重疊**。

---

## ⚠️⚠️⚠️ ArduPilot 必要的一行修改(已套用並重建)

`libraries/SITL/SIM_JSBSim.cpp` 產生的啟動腳本原本只寫:

    <set name="propulsion/engine[0]/set-running" value="1"/>

那一個綁到 `FGEngine::SetRunning()`,**只設 `Running` 旗標**。
`turbine_engine` 有自己的相位機,不吃它 ⇒ 相位停在 Off。

**實測(JSBSim 單獨跑,同一個機體檔):**

| 屬性 | thrust | N1 |
|---|---|---|
| `propulsion/engine[0]/set-running` | **0.00 lbf** | **0.003** |
| `propulsion/set-running`(值 −1 = 全部引擎) | **32.89 lbf** | **100.0** |

後者綁到 `FGPropulsion::SetEngineRunning()`,那一支才會推相位。

⚠️ 失效**完全靜默**:ArduPilot 側油門照顯示 100%、零錯誤訊息。
表徵是「跑道上加速到 61 m/s 後一路減速回 1.8 m/s、永遠不離地」。

修法是在樣板加一行 `<set name="propulsion/set-running" value="-1"/>`,
**已套用**並 `./waf plane` 重建(4.7 s 增量)。
備份在 `/tmp/SIM_JSBSim.cpp.bak`。
✅ 已驗證這個修改**不影響** Rascal(打 patch 後仍正常出遙測)。

### ⚠️ 試過並被推翻的修法(不要重試)

在機體檔的 `<flight_control>` 裡加一條 `pure_gain` 持續把 0 寫進
`propulsion/set-running`。那會讓 JSBSim **每一幀重新初始化引擎狀態機**
⇒ 整個模擬卡死(實測跑超過 120 s 沒有任何輸出)。

---

## 目前狀態(2026-08-27 更新)

| 項目 | 狀態 |
|---|---|
| JSBSim 安裝 | ✅ v1.2.2,PATH 上可執行 |
| ArduPilot ↔ JSBSim 管線 | ✅ Rascal 與**我們自己的 Barracuda500** 都通 |
| 我們的機體檔載入 | ✅ 解析成功,ATTITUDE 帶真正的 p/q/r |
| 引擎推力 | ✅ 32.99 lbf / N1 100(單獨跑實測) |
| 配平 | ✅ **三軸全部 Passed**(先前記載的「發散」是錯的,見下) |
| 起飛 / 爬升 | ✅ 會抬頭、會離地,單趟最高爬升 **688.9 m** |
| **持續可控飛行** | ❌ 橫向發散振盪,而且**不可重現** |

---

## ⚠️⚠️⚠️ 訂正:先前記載的「配平發散」是**錯的診斷**

本檔舊版寫著「配平解算器在我們這組氣動/地面反作用力數值下發散」,並建議去
查 `do_simple_trim` 的收斂條件、調 `<ground_reactions>` 的彈簧阻尼與
Cmalpha/Cmde。**那會浪費時間在一個不存在的問題上。**

2026-08-27 受控實測:同一個 arduplane 二進位,**只換機體檔**,就得到

    Ground Trim / Trim successful
      Altitude AGL  0.58   wdot -1.00e-04   Passed
      Pitch Angle  -0.02   qdot -9.72e-07   Passed
      Roll  Angle   0.00   pdot -2.28e-07   Passed

配平**根本沒跑到**就死了 —— 舊 log 裡沒有任何一行 `(Event 0) executed`、
沒有 `Ground Trim`、沒有 `Trim successful`。

⚠️ 順帶訂正一個術語錯誤:`do_simple_trim = 2` **不是**空中配平,mode 2 是
`tGround`(地面配平)。

⇒ 這正是 [[verifying-the-verifier]] 記的那一類:**一個合理的假說沒有做受控
   A/B 就被寫成「發現」**,然後後面每個人都照著它去查錯方向。

### 三個真正的原因

**① 部署副本是舊版(這是主因)**

`~/ardupilot/Tools/autotest/aircraft/Barracuda500/` 底下那份,還帶著本檔
自己在「試過並被推翻的修法」段落標記為**不要重試**的那條 `pure_gain`
(持續寫 `propulsion/set-running`)。repo 的正確版本是 3 分鐘後寫的,
**但從來沒有重新部署**。

    部署 md5 a1394a607b3206b88380d5c34575b0cb
    repo md5 96c3632a0516d18fbc50940956b81060

單單換這個檔,就從「JSBSim 在第 0 幀就死」變成「配平成功、完整遙測」。
⚠️ 這是 [[two-launcher-paths-differ]] 的同一個形狀:**兩份副本靜默分歧,
   而沒有任何地方檢查它們相同**。修法見下面的部署守衛。

**② FGTurbine 的待機油耗是寫死的下限,3 kg 油 110 秒燒光**

`FGTurbine.cpp:528` 把 `IdleFF = pow(MilThrust, 0.2) * 107.0` [pph] 寫死
(原始碼註解就寫 `just an estimate`),而 `:226` 把它當**下限**套用 ⇒
機體檔的 `<tsfc>` 降不下來。我們 milthrust=33.72 ⇒ 216.25 pph = 0.06007 pps。

    6.6139 lb / 0.06007 pps = 110.1 秒

實測:t=109.99 推力 32.05、油量 0.006;t=119.99 推力 **0**、N1 7.44、油量 0。
**完全靜默** —— 油門照樣顯示 100%。

⚠️⚠️ 這正好是本檔先前歸因給 `<milthrust>` 單位 bug 的那個表徵
   「加速到 61 m/s 後一路減速、永遠不離地」。**兩個不同的原因,同一個症狀**;
   單位那個已經修好了,這個是另一個。

算術上不可能靠加大油箱解決:撐 1200 秒要 72.1 lb = 32.7 kg 的油,整機才 22 kg。
⇒ 修法:每幀把 `propulsion/refuel` 拉成 true(已寫進機體檔的 `InfiniteFuel`
通道)。它是一個單純的 bool,不像 `propulsion/set-running` 會驅動狀態機,
所以每幀寫是安全的(400 秒實測:推力恆 32.058、油量恆 6.6139、不卡死)。

**③ 三個起落架接觸點同高 ⇒ 地面姿態 0 度 ⇒ 滑跑產生負升力**

原本 NOSE / LEFT_MAIN / RIGHT_MAIN 的 z 全是 -0.20。而我們的 CL 表在
α=0 時 CL=0(無彎度)⇒ 滑跑整段是**負升力**,實測滑跑俯仰角 -1.9 到 -5.4 度,
永遠抬不起頭。
會飛的 Rascal 是 **14.3 度**朝上,而 ArduPilot 產生的 `reset.xml` 裡
**寫死 `theta=13.0`**,正是照它的姿態校的。
⇒ 修法:把 `name="NOSE"`(其實是尾輪 —— x=+0.55 在 CG 後方,JSBSim 結構
座標 +X 朝後)的 z 由 -0.20 抬到 -0.05。已套用。

---

## ❌ 未解:橫向控制

修好上面三個之後 **會起飛、會爬升**(單趟 688.9 m),但離地後進入**發散的
滾轉振盪**,而且**不可重現** —— 同一組數值、同一支腳本,有時爬 688 m、
有時 5 m 就摔。

實測形態(見 `runs/artifacts/JSB1/`):滾轉包絡 ±40° → ±100° → ±180° 逐次放大,
p 峰值 190 到 560 度/秒,油門同時在 0/100 之間 bang-bang(TECS 也在打架)。

### ⚠️ 試過且被受控實測推翻的修法(不要重試)

| 改動 | 動機 | 結果 |
|---|---|---|
| `Clda` 0.18 → 0.06 | pb/2V=0.15 是戰鬥機的 1.7 倍,看起來明顯過強 | **更糟**,連離地都做不到(爬升 3.5 m) |
| 再加 `Clb` -0.08 → -0.04、`Cnr` -0.10 → -0.15 | 側滑經 Clb 放大成滾轉 | **更糟**(爬升 4.0 m) |
| `RLL_RATE_P` 0.03 / 0.08 兩種 | 增益太高 | 兩邊都發散 |
| `TKOFF_ROTATE_SPD`=30 + `TKOFF_THR_MAX`=65 | 推重比 0.7 讓它以 2.3 倍失速速度才離地 | 留空變長(22→50 s)但仍爬不出去 |

⇒ **降低控制權限讓情況更糟**,因為自駕儀失去救回擾動的能力。

### 下一步(先量,再調)

⚠️ 這一組橫向導數(Clb/Clp/Clr/Clda)與**整個慣量張量**都是估計值。
在量到之前,任何照理論值調整都只是換一組猜測。順序:

1. 飛 FBWA / MANUAL,**開迴路階躍副翼**,量實得滾轉率 vs 指令 → 據以校 `Clda`
2. 配平後脈衝方向舵,看荷蘭滾 / 螺旋模態
3. **只有在 1 和 2 之後**才去碰 `RLL_RATE_*` / `PTCH_RATE_*`,而且用 AUTOTUNE
4. 慣量張量要真的量(或至少從 CAD 算),否則 p/q/r 的**數值**不可引用 ——
   這個模型現在給得出 6-DOF 的**形狀**,給不出 6-DOF 的**數值**

---

## ⚠️ 部署守衛(必要)

整個多小時的死路來自「部署副本和 repo 靜默分歧」。部署一定要驗 md5:

```bash
cp sitl/jsbsim/Barracuda500/Barracuda500.xml    ~/ardupilot/Tools/autotest/aircraft/Barracuda500/Barracuda500.xml
md5sum 兩份 && 不同就大聲失敗
```

`scratchpad/deploy_and_fly.sh` 已經內建這一步(md5 不符就 exit 2),
並額外檢查 `InfiniteFuel` 在、`engine-autostart` 不在。

---

## 3D 畫面:FlightGear(2026-08-27 接好接線,尚未安裝)

⭐ **不需要改 ArduPilot 的程式碼。**

ArduPilot 產生的 `jsbsim_fgout_<i>.xml` 本來就是

    <output name="127.0.0.1" type="FLIGHTGEAR" port="5504+10i" protocol="UDP" rate="..."/>

也就是說 **JSBSim 早就在講 FlightGear 的原生 FDM 協定**,只是收的那一端是
ArduPilot 自己假扮的(`SIM_JSBSim.cpp` 裡的 `FGNetFDM` 類別就是幹這個)。
要有 3D 畫面,只要**再加一個 `<output>`** 指到真正的 fgfs。

### 已驗證(不需要 FlightGear 就驗得了)

在部署副本注入第二個 output 指到 127.0.0.1:5550,自己寫一個假 FG 去聽:

    packets   : 3375        穩定約 60 Hz
    sizes     : [408]       FGNetFDM 正確長度
    version   : 24          FG 原生 FDM 協定版本
    lat/lon   : 25.077700  121.232500     正是 home
    altitude  : 1.30 m      對上 reset.xml
    RESULT: PACKETS_OK

⚠️ **先驗送出端再裝 FG 是刻意的**:裝完之後若畫面不動,
「封包沒送出來」和「FG 沒在收」表徵一模一樣(都是一架不動的飛機),
而查錯方向相反。先把一端釘死,就只剩一個變數。

### ⚠️⚠️⚠️ WSL 上 fgfs 起不來:是 **D-Bus**,不是顯示卡

實測症狀:`fgfs` 拿得到 pid,但視窗不出現,log 只有 6 行,停在

    0.35 [WARN]:general  Enabling ATI/AMD viewport hack
    dbus[...]: dbus_connection_set_exit_on_disconnect() ...
               assertion "connection != NULL" failed

**真因**:WSL 沒有桌面 session ⇒ 沒有 D-Bus **session bus**。FlightGear 去連
session bus 拿到 `NULL`,然後**沒檢查回傳值**就把 NULL 傳進
`dbus_connection_set_exit_on_disconnect()` ⇒ assertion ⇒ abort。

⚠️ 那行 `Enabling ATI/AMD viewport hack` 是**誤導**:WSLg 的 GL vendor 字串是
`Microsoft Corporation`,被 FG 誤判成 AMD。它看起來很像元凶,但不是。
受控隔離(一次只動一個變因)四個變體**死法完全一樣**:

| 變體 | 結果 |
|---|---|
| 空手啟動 | DIED,同一個 dbus assertion |
| `--prop:/sim/rendering/ati-viewport-hack=false` | DIED,一模一樣 |
| 強制 X11(`env -u WAYLAND_DISPLAY`) | DIED,一模一樣 |
| 軟體算圖(`LIBGL_ALWAYS_SOFTWARE=1`) | DIED,一模一樣 |

**連軟體算圖都死** ⇒ 整個顯示路徑被排除。

**修法**:`dbus-run-session -- fgfs ...`

⚠️⚠️ **這個坑最陰的地方**:`DBUS_SESSION_BUS_ADDRESS` **本來就有值**
(`unix:path=/run/user/1000/bus`)—— 變數設好了、環境檢查完全正常,
**但那個位址後面沒有 bus 在聽**。「變數有設」和「服務有跑」在
`echo $VAR` 底下長得一模一樣,所以照環境變數查會查不出東西。
同 [[verifying-the-verifier]] 那一類:檢查本身看得到的東西不是要檢查的東西。

⭐ 順帶確認 WSLg 的 GL 其實很好:
`D3D12 (NVIDIA GeForce RTX 4070)`、`direct rendering: Yes`、
`Accelerated: yes`、`Max core profile version: 4.2`。

### 安裝(需要 sudo,只能人工跑)

    sudo apt-get update
    sudo apt-get install -y flightgear flightgear-data-base mesa-utils

WSLg 已確認可用(`/mnt/wslg` 存在、DISPLAY=:0、WAYLAND_DISPLAY=wayland-0),
所以 fgfs 的視窗會直接出現在 Windows 桌面上。磁碟 564 GB 可用。

### 啟動

    bash scratchpad/fg_deploy_fly.sh 180

它會:部署並注入 output → 起 fgfs(`--fdm=external`,只當顯示器)→ 起 SITL
→ 飛並錄 CSV。

⚠️ 兩個裝完才知道的變數:
* **視覺機體**:我們沒有 Barracuda500 的 FG 3D 模型,先用 `--aircraft=ufo`
  之類的內建機體當替身。外形不對,但姿態是對的。
* **地景**:`flightgear-data-base` 只帶最小地景,台灣一帶會是預設海面。
  要真地形得開 TerraSync(需要網路,而且會下載很久)。

### ⚠️ repo 曾經缺一個檔(已補)

`sitl/jsbsim/Barracuda500/Engines/direct.xml` **原本不在 repo 裡**,
只存在於部署目錄。機體檔寫著 `<thruster file="direct">`,少了它就是

    FATAL ERROR: No thruster definition supplied with engine definition.

先前沒被發現,是因為一直重用**已經有那個檔**的 `Barracuda500` 部署目錄;
一建新目錄就爆。這是同一類的部署漂移 —— 而且和主因那個一樣,
是「repo 與部署副本不一致」而不是模型本身有問題。

---

## ⚠️ 操作陷阱(再補三個)

6. ⚠️ `*CRASH DETECTED*` 會**無界洗版**:JSBSim 對每個接觸點每一幀印一次,
   然後丟 SIGFPE,arduplane 跟著 `Floating point exception (core dumped)`。
   一次實測產生 **304 MB / 1690 萬行** 的 console log。
   ⇒ 測試 harness 要把這個字串當終止條件,並且截斷 log。
7. ⚠️ **XML 註解裡不可以出現連續兩個減號**。JSBSim 只會報一個含糊的
   `XML parse error: not well-formed (invalid token)`,完全不會提到註解。
8. ⚠️ JSBSim 的 CSV 是寫在 `--root` 底下,**不是 cwd**。

## 機體模型的數字分三級 —— **不可混為一談**

| 級別 | 項目 | 來源 |
|---|---|---|
| **量到的** | S_REF 0.45 m²、翼展 1.88 m、質量 22 kg、CD0 0.01642 | `predictor.py:39-41`、`jet_model.json` |
| **推導的** | 弦長 0.239 m、AR 7.85、CL_alpha 4.88 /rad(Helmbold) | 幾何 |
| **假設的** | ⚠️ **慣量張量整組是估的**(細長體近似) | 本專案從未量過 |

⚠️⚠️ **所有與 p/q/r 有關的結果在慣量校準之前都不可引用。**
這個模型現在能給你 6-DOF 的**形狀**,不能給你 6-DOF 的**數值**。

### ⚠️ CL_alpha 為什麼用 4.88 而不是「辨識值」6.9503

6.9503 高於無限展弦比的理論上限 2π = 6.283,對 AR=7.85 的有限翼**物理上
不可能**(有限展弦比只會**降低**斜率)。那個辨識值是穿過 ArduPilot 自己的
模型擬合出來的,不可直接搬進另一個氣動框架。CL_max 同理用 1.4 不用 3.2503。

### ⚠️⚠️ 推力:我們自己的資料差 5 倍,尚未解決

| 來源 | T/W | 推力 |
|---|---|---|
| `predictor.py:45`(`m*g/0.7`) | 1.43 | 308 N |
| 分析文件 | 0.274 | 59 N |

用飛行資料反算(3100 m、ρ≈0.9、CD0 0.01642、S 0.45):
巡航 157 m/s 需 **~82 N**、193 m/s 需 **~124 N**
⇒ 0.274 飛不到巡航速、1.43 過大。模型暫取海平面靜推力 **150 N(33.72 lbf)**。

**校準方法**:JSBSim 上重跑水平加速,把最大平飛速度對到實測的 193 m/s。

---

## 操作陷阱(踩到才記下來的)

1. ⚠️⚠️ **孤兒 JSBSim 會佔住埠。** 它被 `setsid()` 分到獨立 session,
   arduplane 死掉它還活著,下一次啟動會 `Could not bind to TCP2 input
   socket, error = 798` —— 而那個訊息不會告訴你是誰佔著。
   清理要用 `pkill -9 -x JSBSim`(**`-x` 精確比對**)。
2. ⚠️ `pkill -f JSBSim` 會 **match 到自己的命令列**(同記憶 `sitl-scripting-traps`
   的「pkill 自殺」)。一律用 `-x`。
3. ⚠️ `<milthrust>` **不可以帶 `unit` 屬性** —— `turbine_engine` 只吃裸磅力。
   寫 `unit="N"` 不會報錯,只會讓推力恆 0。
4. ⚠️ 密度高度表要涵蓋**負值**(海平面可能略小於 0)。JSBSim 自帶的 J85
   從 −10000 起,照做。
5. ⚠️ SITL 要等 GCS 連上 SERIAL0 才會 fork JSBSim。沒接 client 之前
   看不到任何 JSBSim 程序,那**不是**安裝失敗。

---

## 檔案

    sitl/jsbsim/Barracuda500/Barracuda500.xml           機體
    sitl/jsbsim/Barracuda500/Engines/barracuda_jet.xml  引擎
    部署到 ~/ardupilot/Tools/autotest/aircraft/Barracuda500/

## 怎麼跑

```bash
export PATH="$HOME/.local/bin:$PATH"
cd ~/ardupilot
./build/sitl/bin/arduplane -w --model jsbsim:Barracuda500 \
    --home 25.0777,121.2325,32,52 -I5 --sysid 55 \
    --defaults Tools/autotest/models/plane.parm
```
⚠️ `-I5` 是刻意的:避開現行常駐航次的 I0/I1。
