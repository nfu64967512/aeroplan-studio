"""contract_table.py — 生效值契約表(單一事實來源)+ 純函式評估器。

⚠️⚠️ 存在的理由(使用者 2026-08-11 定調:「這些問題一定不能再發生」):

單日七次靜默失敗的**共同結構是同一個**——

    生效值與意圖值不同,而沒有任何東西在檢查。

這一類**不會噴例外**:照跑、照出數字、結論卻是錯的。代價最高的一次是
RPF/RPG/RPH/RPI 一整批報廢,還有一次把 0.86 m 的誤差讀成 26–32 m
並據此排錯了優先序。

⇒ 所以這裡放的是**表**不是補丁。每個關鍵參數一列:
   `{名稱, 值來源, 噴射組態期望區間, 泡棉禁區值, 理由, 反證來源}`。

## 為什麼「泡棉禁區」要單獨一欄

`tot_node.py` 的 `declare_parameter` 預設值**全部是泡棉時代的**
(`vmin=14`、`vmax=30`、`dive_speed=42`、`perch_alt=280`、`r_fan=2000`)。
它們合法、型別正確、不會報錯 —— 一旦某條餵值路徑斷掉就會**靜靜退回**去。
只寫「新值應該是多少」抓不到這件事,因為區間檢查可能剛好放得夠寬。
**禁區是獨立判定**:命中泡棉值一律視為「沒有人設過它」。

## 三個值來源(缺一不可)

* `node`  —— ROS2 節點參數。從**節點自己印出的「生效值」行**讀回,
              不是從 launch 指令抄。兩者不同正是失效#3 的形狀。
* `fc`    —— 飛控 PARM。從 pymavlink **讀回**,不是相信「我有送」。
* `shell` —— 批次腳本環境變數。

## ⚠️ 不變式比單值更重要

壞掉的從來不是單一參數,是**參數之間的關係**。本檔的 `INVARIANTS` 收的是
已被實測釘死的關係;單值區間只是第一道網。

本模組**不做任何 I/O**,不 import ROS2 / pymavlink,可在 Windows 上直接測。
取值與中止流程在 `scripts/wsl/contract_verify.py`。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

# ── 值來源 ────────────────────────────────────────────────────────────
SOURCE_NODE = "node"
SOURCE_FC = "fc"
SOURCE_SHELL = "shell"

G = 9.80665
#: 守環/盤旋的設計傾角(度)。tot_node 自己的迴轉半徑告警用的就是 45°,
#: 這裡沿用同一個值,兩邊才會給出同一個數字。
BANK_DEG = 45.0


@dataclass(frozen=True)
class Contract:
    """一個關鍵參數的契約。

    lo/hi 為 None 表示該側不設限(只記錄)。foam 為 None 表示這個參數
    沒有泡棉時代的對應值(例如飛控 PARM)。
    """

    name: str
    source: str
    lo: Optional[float]
    hi: Optional[float]
    foam: Optional[float]
    why: str
    evidence: str


@dataclass(frozen=True)
class Violation:
    #: "missing" 讀不到 / "foam" 命中泡棉禁區 / "range" 落在區間外 / "invariant" 關係不成立
    kind: str
    name: str
    got: object
    want: str
    why: str
    evidence: str

    def line(self) -> str:
        tag = {"missing": "讀不到", "foam": "泡棉禁區",
               "range": "區間外", "invariant": "不變式"}[self.kind]
        return (f"  ✗ [{tag}] {self.name} = {self.got}(需要 {self.want})\n"
                f"      理由:{self.why}\n"
                f"      反證:{self.evidence}")


# ══ 契約表 ═══════════════════════════════════════════════════════════
#
# ⚠️ 區間**全部來自實測或已驗證組態**,不是估的。噴射組態的真值來源是
#    `swarm_ros2_up.sh` 的 TOT_* 預設(互動/量測路徑的實際生效值);
#    泡棉值來源是 `tot_node.py` 的 declare_parameter 預設。
#    區間在真值兩側留調參餘裕,但**必須窄到把泡棉值排除在外**——
#    否則這張表對它要抓的那一類失敗是瞎的。

TABLE: tuple[Contract, ...] = (
    # ── ⭐ t_go 估計器與影子預測(RPY3 2026-08-14 之後納管)────────────
    # ⚠️⚠️ 這三個在 RPY3 之前**完全不在契約表內**,所以閘門對它們是瞎的。
    #    RPY3 開著 tgo_from_predictor=1 跑,而 predict_hz=0(從沒接線)——
    #    「拿預測器當致動來源、卻沒開影子觀測」這個組合閘門一聲都沒吭。
    Contract(
        "tgo_from_predictor", SOURCE_NODE, 0.0, 1.0, None,
        "0 = 靜態幾何配速 d/pace;1 = 點質量積分器正向積分。"
        "它決定放行時刻表、v_req 與 miss 出口的共同上游。",
        "RPY3:開著它跑,而 _tgo_of 的快取被牆鐘/開機時鐘混寫 ⇒ 共同命中"
        "時刻被排到 56 年後 ⇒ 時刻表放行永不成立、俯衝段零時間權限;"
        "散布 78.5 s,同組態另一趟 12.1 s(重現性全失)",
    ),
    Contract(
        # ⚠️ 下界 0.5 ⇒ 宣告預設 False(0.0)被拒:餵值路徑斷掉時契約層才抓得到。
        "tgo_ring_transit", SOURCE_NODE, 0.5, 1.0, None,
        "把「還要多久才進得了 pushover 環」併入 t_go。"
        "放行是**先空間閘門、後時間時刻表**,而 t_go 若只算直飛紅心的時間,"
        "共同命中時刻就會低估受空間限制那一架真正需要的時間。",
        "RPZ1(2026-08-15):UAV2 在環外 851 m,時刻表早已輪到卻不能放行、"
        "等了 37 s ⇒ 放行時刻差 +37.2 s ⇒ 散布 36.2 s,而同趟俯衝耗時差只有 −1.0 s"
        "(下游已修好,問題整個搬到上游)。文獻依據:Bin et al. 式 49 —— "
        "可行命中時刻下界由**當下可飛的最短幾何航程**決定,不是直線距離",
    ),
    # ── ⭐⭐ S1:抵環即放行(2026-08-18)────────────────────────────
    Contract(
        # ⚠️ 0/1 都放行:這是**對照用**的開關,關閉時逐位元同舊行為。
        #    列進契約表是為了讓生效值 log 印得出來 —— 前三批白跑有兩批
        #    (RRA / RRB)的症狀都是「開關看起來開著,但沒有任何一行證明它生效」。
        "release_at_ring", SOURCE_NODE, 0.0, 1.0, None,
        "1 = 飛機 d ≤ ring_of(i) 當拍就放行,完全不讀 dive_T;0 = 舊的時刻表放行。",
        "RRB(2026-08-16,7 趟 14 架次):抵環到放行隔了中位 32.1 s,期間往內漂到"
        "中位 3417 m、掉高度中位 970 m。放行前置角 σ 中位 130.2°、最小 73.7°,"
        "而抵環當拍 σ 是 0.1°(14/14 落在 0.0–0.3°)⇒ **σ≈0 的放行從未被取樣過**。"
        "純 PN 幾何:σ=0 時 2 s 耗時預算容許 σ 擺動 ±125°,σ=125° 只剩 ±1.1°",
    ),
    Contract(
        "dive_from_runup", SOURCE_NODE, 0.0, 1.0, None,
        "1 = 內切(runup)一開始就開始 DIVE 倒數,不在環上等 ⇒ hold_ring 整段消失。",
        "RRB:hold_ring 段指令 vmin=95 但實飛地速中位 142(EAS 換算後仍差 +23,"
        "是 straight 段的 10 倍);段長只有 25 s,而該機體從 EAS 138 減到 95 需要"
        "約 100–140 s(THR_MIN=0、無擾流板,實測減速率 −0.12 m/s²)⇒ "
        "**守環在物理上不可能完成它的任務**,只是把飛機往環內送 3.1 km",
    ),
    Contract(
        # ⚠️ 上界 200 而非更緊:這一列的目的是**讓它可見**,不是收緊它。
        "perch_settle", SOURCE_NODE, 1.0, 200.0, None,
        "抵環後等高度收斂的逾時秒數。dive_from_runup 開著時它同時是"
        "**陡俯角參數的重送視窗長度**(視窗 × 20 Hz = 重送次數)。",
        "這一列以前不在契約表、也不在生效值 log(對過 RRB1 的 tot_standby.log)。"
        "而 PTCH_LIM_MIN_DEG=-80 / TECS_SPDWEIGHT / THR_MAX 就跑在這個視窗裡,"
        "原始碼註解自己寫著「送出去了沒吃進去是已知會發生的」⇒ 視窗一塌就是"
        "靜默失效,而那組開關是 0/6 變 4/6 的關鍵",
    ),
    Contract(
        "perch_band", SOURCE_NODE, 0.0, 500.0, None,
        "高度收斂帶(m)。⚠️ 它量的是 |z − perch_of(i)|,但飛機被指到的是 "
        "hold_alt_of(i) —— HOLD_ALT>0 時這個閘門**永遠不成立且完全靜默**。",
        "RRB 的 hold_alt=0 所以現在看不出來;登記進契約表是為了讓下一個人"
        "改 HOLD_ALT 時至少看得到這一列",
    ),
    Contract(
        "wait_goto_radius", SOURCE_NODE, 0.0, 1.0, None,
        "1 = wait 段也用 goto_radius(飛過去);0 = 用 hold_radius(繞圈)。",
        "RRD(2026-08-18,16 架次)的真根因是**指令從未送達**:runin→wait 切換時"
        "`_goto` 的 lat/lon/alt 逐位元相同、只有 radius 變,而變更偵測器不看 radius "
        "⇒ moved=False ⇒ DO_REPOSITION 沒送出。hold_radius 要等 30 s keepalive 才生效,"
        "⇒ 起不起彎由相位隨機決定。實測兩個母體:沒吃到補發 11 架直線、命中 10/11;"
        "吃到補發 5 架硬彎、命中 0/5",
    ),
    # ── ⭐⭐ ITCG(2026-08-15)────────────────────────────────────────
    Contract(
        # ⚠️ 0/1 都放行:ITCG 目前**刻意關閉**(RQA/RQB 的確認還沒跑完,
        #    不要同時動兩個變數)。真正的守門是下面那條不變式。
        "itcg_on", SOURCE_NODE, 0.0, 1.0, None,
        "命中時間控制的總開關。純 PN 的式子裡沒有時間這個量 ⇒ 保證撞到、"
        "不保證何時撞到。",
        "RQA2:兩架放行幾何幾乎相同(5120m/31° 與 5590m/29°、放行差 −2.5s),"
        "PN 仍自己散開成 γ −27.1° vs −32.4°、耗時 55s vs 33s(差 21.2s)",
    ),
    Contract(
        "itcg_gain", SOURCE_NODE, 0.2, 5.0, 0.0,
        # ⚠️ 訂正(2026-08-19):這裡原本寫 `a = K·V²/R·(e_t/R)`,那是 2026-08-15
        #    就被換掉的舊式子 —— 契約文件與程式碼脫節了四天。現行是**反解前置角**:
        #    1/cos σ_need = 1 + surplus/R,偏置正好抵銷 PN 對 σ 的回復加速度
        #    a_PN = N·vh·V·sin σ / R ⇒ K 變成**純修整係數**,1.0 = 正好抵銷。
        "ITCG 橫向偏置增益 K(無因次,純修整係數):反解 σ_need 後抵銷 "
        "a_PN = N·vh·V·sin σ_need / R;1.0 = 正好抵銷。",
        "⚠️ 泡棉禁區 0.0 = 「開了但沒效果」—— 這是最難查的一種失效:"
        "契約顯示 itcg_on=1、log 一切正常、飛出來只是沒改善",
    ),
    Contract(
        "itcg_a_max_g", SOURCE_NODE, 0.1, 1.0, 0.0,
        # ⚠️⚠️ 上限 1.0 **不是待查**,就是契約上限。要跑 2.0 必須先由使用者
        #    裁決改這一列 —— contract_verify 會把超界的整批中止,不可偷渡。
        "ITCG 橫向偏置上限(g)。避免時間項壓過 PN 的命中通道。⭐⭐ 這是"
        "時間權限的**唯一**硬上界:n=32 幾何重放給出 0.4 ⇒ +1.45 s、"
        "1.0 ⇒ +10.59 s,而 n=8 的需求是 7.3–9.0 s。",
        "同上:0.0 等於把偏置整個夾成 0,開關開著也不會有任何效果",
    ),
    Contract(
        # ⚠️ 0/1 都放行:MPNG 預設**刻意關閉**(逐位元同現行 surplus 路徑)。
        #    真正的守門是下面的 _inv_mpng_needs_itcg_on。
        "mpng_on", SOURCE_NODE, 0.0, 1.0, None,
        "1 = 時間項改走 Jiang 2021 MPNG 的 Eq.(41)(a_ε + a_s);"
        "0 = 現行的 surplus/σ_need 反解。⚠️ 只換**橫向時間項**,"
        "a_PNG 與整個縱向通道一行都沒動。",
        "⚠️⚠️ ITCG 那條通道 80,760 個樣本 itcg_accel **全零** —— 從來沒承載過"
        "訊號。列進表是為了讓「這一輪到底跑的是哪一條律」在事後看得見;"
        "擋值的是不變式,不是這個區間",
    ),
    Contract(
        # ⚠️⚠️ 這兩列存在的理由與 mpng_on 不同:開關有不變式守著,**增益沒有**。
        #    `MPNG_K_EPS` 打錯一個字母 ⇒ 節點吃到 0.0 ⇒ 自動走論文建議式,
        #    而 log、契約行、CSV **全部看起來正常** —— 掃增益的那一批會得到
        #    「增益怎麼調都一樣」的結論,然後那個結論其實是對的(因為根本
        #    沒調到),但理由完全錯。這正是 effective-value-contract 那一整類。
        # ⚠️ 下界 0.0 是**合法值**(= 走論文建議式 40/(r₀·tgo₀) 自動推),
        #    所以這一列沒有泡棉禁區 —— 它擋的是量級打錯,不是擋 0。
        # 上界 1e-2:代進我們的工作點(N=4、V=161.7)⇒ a = 1e-2·4·V² ≈ 1046 m/s²,
        # 比 1 g 夾制大兩個數量級 ⇒ 超過它的一定是單位或量級寫錯,不是調參。
        "mpng_k_eps", SOURCE_NODE, 0.0, 1e-2, None,
        "MPNG Eq.(41) 的 K_ε(1/(m·s)):a_ε = K_ε·N·V²·sin(σ)·ε_T。"
        "0 = 走論文建議式 40/(r₀·tgo₀)(放行當拍鎖存)。",
        "⚠️ 論文的建議值是他們 330 m/s **等速**、r₀=10 km、tgo₀≈30 s 的算例值;"
        "代進我們的工作點 K≈1.41e−4 ⇒ ε_T 只要 0.27 s 就撞上 0.4 g 夾制 ⇒ "
        "這條律整段貼在夾制上、退化成 bang-bang。真正的自變數是 itcg_a_max_g,"
        "不是這兩個增益 —— 但那個結論只有在增益**真的餵進去**時才有意義。",
    ),
    Contract(
        "mpng_k_s", SOURCE_NODE, 0.0, 1e-2, None,
        "MPNG Eq.(41) 的 K_s(1/(m·s)):a_s = K_s·N·V²·sign(σ)·ε_T。"
        "0 = 同上走建議式。",
        "⚠️ a_s 是**不連續**項,大小與 σ 無關 —— 原文行 843-844 讓它存在的唯一"
        "理由是解 σ₀=0 的啟動問題。我們俯衝段 σ 中位只有 0.2° ⇒ 幾乎整段貼在"
        "零附近,正是它抖振最兇的區(側向鎖存 itcg_side 就是對策)。"
        "設成 0 以外的值等於改抖振量級,必須看得見。",
    ),
    Contract(
        "tgo_predict_dt", SOURCE_NODE, 0.05, 2.0, None,
        "積分節流(秒)。太大 ⇒ t_go 走階梯;太小 ⇒ 每 tick 跑 500 步積分。",
        "節流靠快取,而快取正是 RPY3 時基污染的載體 —— 這一欄必須看得見",
    ),
    Contract(
        "predict_hz", SOURCE_NODE, 1.0, 20.0, None,
        "影子預測頻率。⚠️ 下界刻意訂在 1.0 而不是 0.0:節點端 0.0 是合法的"
        "「關閉」語意,但**量測航次不准關**——RPY3 證明了「拿預測器致動卻不"
        "觀測它」會讓時刻表壞掉整整一批而無人察覺。要真的關就得顯式改契約。",
        "S9 做好後從沒接線 ⇒ 一路吃預設 0.0 ⇒ _predict_tick 第一行就 return、"
        "pred_* 五欄整批空白,而 early-return **不發任何告警**;"
        "校準過的預測器到 RPY3 為止一趟都沒真的跑過",
    ),
    Contract(
        "dive_angle_max", SOURCE_NODE, None, 38.0, None,
        "放行的幾何守門員:所需俯角 atan(h/d) 一到這個上限就立刻放行,"
        "不等時刻表。⚠️ 0.0 = 關閉,**目前刻意維持關閉**:它掛在 phase==DIVE "
        "分支裡、不在 PERCH 段跑,而 RPY3 的 UAV1 在 phase 翻 DIVE 時 γ_req 已經 "
        "41.5°、接下來在 wait 模式外飛到 33.5° 才放行 ⇒ 設 34 會在 41.5° 立刻觸發,"
        "把它放進比等待更差的幾何。正確修法是把 γ_req 檢查前移到 PERCH。"
        "上界 38 擋的是「設得比機體天花板還高」= 等於沒設。",
        "RPY3:預設 0.0(關閉)⇒ 放行只剩 perch_settle 25 s 逾時、完全不看幾何 ⇒ "
        "UAV1 在所需俯角 41.5°(> 實測天花板 37–40°)時被放行 ⇒ 外飛 7024 m、"
        "洩掉 1900 m 高度、淺滑翔 90 s ⇒ 俯衝 137.8 s vs 同趟另一架 59.4 s;"
        "四架次 (γ_req, 耗時) = (23.0,59.4)(23.2,49.4)(28.4,61.5)(41.5,137.8)",
    ),
    # ── ⚠️ 兩個先前完全隱形的參數(2026-08-15 納管)──────────────────
    Contract(
        # ⚠️ 下界 150 ⇒ 節點宣告預設 80 被拒。80 低於俯衝實測地速 69–211,
        #    餵值路徑一斷就會把 v_req 夾在 80 ⇒ 俯衝段油門通道整段飽和。
        "pn_vmax", SOURCE_NODE, 150.0, 260.0, 80.0,
        "PN 油門律的速度上限。`v_req = max(45, min(pn_vmax, R/t_rem))`。",
        "swarm_ros2_up.sh 餵 210,但**先前不在契約表**⇒ 餵值路徑斷掉會靜默退回"
        "宣告預設 80,而俯衝實測地速 69–211 ⇒ v_req 恆夾在 80、油門通道整段飽和,"
        "且完全無告警(與 predict_hz 同一形狀)",
    ),
    Contract(
        # ⚠️ 目前刻意維持 ArduPlane 原廠 15(= 現行行為),見白名單理由。
        "SCALING_SPEED", SOURCE_FC, None, 200.0, None,
        "舵面速度縮放的基準速度。Attitude.cpp::calc_speed_scaler():"
        "`speed_scaler = SCALING_SPEED / aspeed` ⇒ 所有俯仰/滾轉 PID 的"
        "**有效增益 ∝ SCALING_SPEED/空速**。",
        "⚠️ 它是**純乘子**,不改變 1/V 的形狀;15→135 等價於增益 ×9。"
        "實測 dataflash:高速段升降舵只用 1400–1619 µs(±12% 行程、0% 打到底)、"
        "Pitch 追隨誤差中位 1.9–3.8° ⇒ 現有增益**不缺權限**。"
        "上界 200 擋的是「設得比巡航空速還高」= 增益暴衝",
    ),
    # ── 速度包絡 ──────────────────────────────────────────────────────
    Contract(
        "vmin", SOURCE_NODE, 80.0, 120.0, 14.0,
        "同步迴路的速度下限。噴射組態實測 95。",
        "泡棉值 14 遠低於飛控 AIRSPEED_MIN=25 ⇒ 每一發 DO_CHANGE_SPEED "
        "被 MAV_RESULT=4 整發拒收(catch-speed-rejected-by-fc)",
    ),
    Contract(
        "vmax", SOURCE_NODE, 130.0, 230.0, 30.0,
        "同步迴路的速度上限。噴射組態實測 150。",
        "必須 ≤ 飛控 AIRSPEED_MAX,超出即整發拒收;"
        "泡棉值 30 會讓全機隊被夾在巡航速度的 1/5",
    ),
    Contract(
        "dive_speed", SOURCE_NODE, 180.0, 240.0, 42.0,
        "俯衝指令空速。使用者定調終端 KPI 204 m/s,組態值 210。",
        "dive-pace-saturates-vmin:dive_speed=210 因為配速錯誤而從未生效",
    ),
    Contract(
        # 0 = 交給 tot_node 自動取 0.8×dive_speed,是**合法且目前採用**的設定。
        # 真正的判定在 INVARIANTS 的 dive_pace 兩側餘裕,這裡只擋泡棉常數。
        "dive_pace", SOURCE_NODE, None, None, 50.0,
        "俯衝配速。0 = 自動取 0.8×dive_speed(目前組態);非 0 時受不變式管。",
        "handoff 缺陷#5:泡棉常數 50.0 ≪ vmin 95 ⇒ 同步迴路全程飽和,"
        "散布 25.00 → 0.78/2.40 s(dive-pace-saturates-vmin)",
    ),
    # ── 幾何 ──────────────────────────────────────────────────────────
    Contract(
        "gate_r", SOURCE_NODE, 2500.0, 6000.0, 1200.0,
        "PN 交接水平距。噴射組態 4800,且受 gate_r_min/gate_r_max 夾。",
        "u1-descent-authority:成敗看 gate_r/環半徑 比(n=17 零重疊),"
        "≥0.566 殘餘 44–343 m、≤0.446 殘餘 506–2499 m",
    ),
    Contract(
        "perch_alt", SOURCE_NODE, 2500.0, 3600.0, 280.0,
        "高點佔位高度。噴射組態 3100(= EAS 135 / 地速 157 的設計點)。",
        "eas-vs-groundspeed-157:157 是 3100 m 的地速,高度改了整個速度契約就失效",
    ),
    Contract(
        "r_fan", SOURCE_NODE, 20000.0, 45000.0, 2000.0,
        "扇形半徑。噴射組態 35000。",
        "泡棉值 2000 對 100 m/s 級機體只有 20 s 航程,扇形收不完",
    ),
    Contract(
        "dive_angle", SOURCE_NODE, 20.0, 60.0, 25.0,
        "設計俯衝角。噴射組態 27。",
        "dive-angle-ceiling:硬上限 ~60°(驗收用地速),90° 不可行;"
        "⚠️ 泡棉值 25 與噴射值 27 只差 2,單值檢查抓不到,靠禁區欄位",
    ),
    Contract(
        "reach", SOURCE_NODE, 1500.0, 3500.0, 120.0,
        "扇形抵達判定半徑。噴射組態 2400。",
        "preflight_check 的 ALIGN 死結檢查:WP_LOITER_RAD×eas2tas² 必須 < reach",
    ),
    Contract(
        "perch_reach", SOURCE_NODE, 150.0, 400.0, 70.0,
        "認定已就位 pushover 環的半徑。噴射組態 260。",
        "泡棉值 70 小於噴射機一個取樣步的位移,環永遠判不到",
    ),
    Contract(
        "impact_reach", SOURCE_NODE, 10.0, 60.0, 40.0,
        "認定已穿越紅心的水平半徑。噴射組態 25。",
        "⚠️ 泡棉值 40 落在噴射區間內 —— 單值檢查抓不到,只有禁區欄位擋得住",
    ),
    Contract(
        "half_angle", SOURCE_NODE, 15.0, 40.0, 30.0,
        "扇形半頂角。噴射組態 25。",
        "b500-wide-spacing-status:扇形收太小 ALIGN 收不完(UAV4 卡死)",
    ),
    # ── 編隊層高度(⚠️ 目前互動路徑實際違規中)────────────────────────
    Contract(
        "formation_alt", SOURCE_NODE, 2500.0, 3600.0, 50.0,
        "編隊層高度。必須落在噴射巡航帶,否則編隊層會把飛機往地面錨。",
        "formation-layer-cannot-fly-the-jet:編隊層速度低於 AIRSPEED_MIN 全被拒收 "
        "+ 1Hz 重錨高度;n3-wingman-and-sea-datum:僚機爬升被拖回地面(288s→59s)。"
        "⚠️ swarm_ros2_up.sh:545 目前預設 50.0 —— **現行組態就是違規的**",
    ),
    Contract(
        "patrol_alt", SOURCE_NODE, 2500.0, 3600.0, 80.0,
        "長機巡邏高度。同上,必須在噴射巡航帶。",
        "⚠️ swarm_ros2_up.sh:546 目前預設 80.0(退回 FORM_ALT)—— **現行組態違規**",
    ),
    Contract(
        "cruise_alt_threshold", SOURCE_NODE, 2500.0, 3600.0, None,
        "編隊層判定「已進巡航」的高度。與彈射端交棒高度有死區關係。",
        "preflight_check 的高度閘門死區檢查:必須 ≤ CLIMB_TO_ALT − CLIMB_TO_TOL",
    ),
    # ── ⭐⭐ 陡俯角的兩個開關(2026-08-14 補;兩條路徑不一致是本專案最貴的一類)──
    Contract(
        # ⚠️ 0.0 = 關閉幾何前饋 = 純 PN。純 PN 的縱向通道**完全不引用 dive_angle**,
        #    終端航跡角只是「對視線角速率積分」的副產品,不是被要求的量。
        #    ⇒ 關著的時候,DIVE_ANGLE 設幾度都不會影響實際俯衝角。
        "pn_los_floor", SOURCE_NODE, 0.8, 1.2, 0.0,
        "幾何前饋:指令俯角不准比「直指紅心的視線」更淺。1.0 = 完全對著紅心。",
        "tot_node.py 自述『這是陡俯角**唯一有效的開關**』。"
        "⚠️⚠️ 路徑不一致:run_west.sh:238 給 1.0(TWO1 唯一達標航次走這條),"
        "swarm_ros2_up.sh:316 給 **0.0** ⇒ RPK/RPL/RPO/RPQ 二十四架次全部在關閉狀態下飛。"
        "因此『TWO1 與 RPK/RPL 同組態卻結果不同』的比較是無效的",
    ),
    Contract(
        # ⚠️ 0.05 = 不准負 g 推頭。俯角上限因此被**法向過載下限**鎖死,不是被機體鎖死。
        "pn_nz_min", SOURCE_NODE, -2.0, 0.0, 0.05,
        "PN 法向過載下限。要陡俯角必須放到負值(允許小負 g 推頭)。",
        "tot_node.py:464『0.05 ⇒ 俯角上限被它鎖死:推到 −80° 需要 9259 m 高度,"
        "我們只有 3148』。⚠️ 路徑不一致:run_west.sh:240 給 −1.0,"
        "swarm_ros2_up.sh:317 給 **0.05**",
    ),
    # ── 就緒閘門 ──────────────────────────────────────────────────────
    Contract(
        "READY_AGL", SOURCE_SHELL, 20.0, 300.0, None,
        "批次就緒閘門的離地高門檻。",
        "handoff 缺陷#4:舊閘門只看 AMSL 穩定 ⇒ 飛機停在跑道上永遠成立、4 秒放行。"
        "⚠️ 門檻不可達同樣致命(RPB1/RPC1 整趟逾時)—— 見不變式",
    ),
    # ── 飛控 PARM(讀回,不是相信送出)────────────────────────────────
    Contract(
        "AIRSPEED_MIN", SOURCE_FC, 20.0, 30.0, None,
        "飛控空速下限。噴射機 25。",
        "任何低於它的速度指令被**整發拒收**,不是夾住",
    ),
    Contract(
        "AIRSPEED_MAX", SOURCE_FC, 150.0, 240.0, None,
        "飛控空速上限。噴射組態 230。",
        "catch-speed-rejected-by-fc:catch_speed 157 > 150 ⇒ 追趕通道從未生效",
    ),
    Contract(
        "THR_MAX", SOURCE_FC, 20.0, 100.0, None,
        "飛控推力上限。⚠️ 互動路徑實際跑 100,設計值 25。",
        "thrmax-needs-climb-boost:JET_THRMAX=25 是巡航值,互動路徑沒有爬升提升就飛不起來;"
        "AP3/AP5 其實跑 100。**所有互動路徑 KPI 都帶著這個前提**",
    ),
)

BY_NAME = {c.name: c for c in TABLE}

#: 契約參數 → 設定它的環境變數(`swarm_ros2_up.sh` / `rep_interactive.sh` 的入口)。
#: ⚠️ 用途是讓違規訊息能印出**可以直接貼的重跑指令**。
#:    「印了錯誤但不告訴你怎麼修」等於逼人去翻腳本,而翻錯就是下一次靜默失敗。
ENV_FOR = {
    "vmin": "VMIN", "vmax": "VMAX", "dive_speed": "DIVE_SPEED",
    "dive_pace": "DIVE_PACE", "gate_r": "GATE_R", "perch_alt": "PERCH_ALT",
    "r_fan": "R_FAN", "dive_angle": "DIVE_ANGLE", "reach": "REACH",
    "perch_reach": "PERCH_REACH", "impact_reach": "IMPACT_REACH",
    "half_angle": "HALF_ANGLE", "formation_alt": "FORM_ALT",
    "pn_los_floor": "PN_LOS_FLOOR", "pn_nz_min": "PN_NZ_MIN",
    "patrol_alt": "PATROL_ALT", "cruise_alt_threshold": "CRUISE_ALT_THRESHOLD",
    "READY_AGL": "READY_AGL", "THR_MAX": "JET_THRMAX",
    "AIRSPEED_MAX": "AIRSPEED_MAX_CAP", "pn_vmax": "PN_VMAX",
    "SCALING_SPEED": "SCALING_SPEED",
    "tgo_from_predictor": "TGO_FROM_PREDICTOR",
    "tgo_ring_transit": "TGO_RING_TRANSIT",
    "itcg_on": "ITCG_ON", "itcg_gain": "ITCG_GAIN",
    "itcg_a_max_g": "ITCG_A_MAX_G",
    "mpng_on": "MPNG_ON",
    # ⚠️ 增益也要有 env 對應:違規訊息印出的重跑指令少一個變數,
    #    使用者照貼就會跑到「自動推導」而不是他想掃的那個值。
    "mpng_k_eps": "MPNG_K_EPS", "mpng_k_s": "MPNG_K_S",
    "tgo_predict_dt": "TGO_PREDICT_DT", "predict_hz": "PREDICT_HZ",
    "dive_angle_max": "DIVE_ANGLE_MAX",
    "release_at_ring": "RELEASE_AT_RING",
    "dive_from_runup": "DIVE_FROM_RUNUP",
    "wait_goto_radius": "WAIT_GOTO_RADIUS",
    "perch_settle": "PERCH_SETTLE", "perch_band": "PERCH_BAND",
}

#: 已驗證的噴射組態值(`swarm_ros2_up.sh` 的 TOT_* 預設 = 互動路徑實際生效值)。
#: ⚠️ 這**不是**契約本身(契約是區間),只是違規時要建議的具體數字。
#: ⚠️ formation_alt / patrol_alt 沒有「已驗證值」—— 現行組態就是違規的,
#:    建議值取巡航高度 perch_alt,但**尚未經過實測**,標記在下方 UNPROVEN。
JET_REFERENCE = {
    "vmin": 95.0, "vmax": 150.0, "dive_speed": 210.0, "dive_pace": 0.0,
    # ⚠️ 2026-08-13:曾改成 5600(為壓低需求俯角),**當日被 RPO 三趟實測回退**——
    #    5600 與 hold_radius=1300 相乘會讓 gate 落進守環擺盪帶,交棒距從 ±11 m
    #    炸成 ±1075 m、真落地 5/12→1/6。見不變式 `gate_r < 環半徑 − hold_radius`。
    "gate_r": 4700.0, "perch_alt": 3100.0, "r_fan": 35000.0, "dive_angle": 27.0,
    "reach": 2400.0, "perch_reach": 260.0, "impact_reach": 25.0,
    "half_angle": 25.0, "cruise_alt_threshold": 3000.0, "READY_AGL": 30.0,
    # TWO1(唯一達標航次)的值,來源 run_west.sh:238,240
    "pn_los_floor": 1.0, "pn_nz_min": -1.0,
    "formation_alt": 3100.0, "patrol_alt": 3100.0,
    # ⭐ t_go 估計器與影子預測。predict_hz=2.0 是**硬性**的:RPY3 之後,
    #    「拿預測器決定放行時刻表、卻沒有任何一欄記錄它算了什麼」不再被允許。
    "tgo_from_predictor": 1.0, "tgo_predict_dt": 0.5, "predict_hz": 2.0,
    "tgo_ring_transit": 1.0,
    # ⚠️ ITCG 目前刻意關閉;增益仍餵值,才不會在真要開的時候吃到 0。
    "itcg_on": 0.0, "itcg_gain": 1.0, "itcg_a_max_g": 0.4,
    # ⚠️ B1(2026-08-19):MPNG 預設 **關閉** = 逐位元同現行 surplus 路徑。
    #    要跑 MPNG 才顯式打開,而且必須連 itcg_on 一起開(見不變式)。
    "mpng_on": 0.0,
    # ⚠️ 建議值 0.0 = 走論文建議式 40/(r₀·tgo₀) 自動推(**合法值**,
    #    不是「沒設」)。與 itcg_gain 不同:那一列 0.0 是泡棉禁區。
    "mpng_k_eps": 0.0, "mpng_k_s": 0.0,
    "pn_vmax": 210.0,
    # ⚠️ 15 = ArduPlane 原廠值 = 現行實際行為。列出來只為了可見。
    "SCALING_SPEED": 15.0,
    "dive_angle_max": 0.0,
    # ⚠️ S1 的兩顆開關預設 **關閉** = 逐位元舊行為。要跑 S1 才顯式打開。
    "release_at_ring": 0.0, "dive_from_runup": 0.0, "wait_goto_radius": 0.0,
    "perch_settle": 25.0, "perch_band": 40.0,
}

#: ⚠️⚠️ 這些建議值**沒有實測背書**,照著設等於開一趟新實驗,不是「修好了」。
#: ⚠️ itcg_gain / itcg_a_max_g 是**紙上推算**的起始值(由 V²/R 尺度與
#: 實測 e_t 量級反推),一趟都沒飛過。照著設等於開一趟新實驗。
UNPROVEN = {"formation_alt", "patrol_alt", "itcg_gain", "itcg_a_max_g"}


#: 宣告為整數的節點參數。⚠️ 這些**不可以**印小數點(反向的型別坑)。
INT_PARAMS = frozenset({"param_verify", "stall_prevention_glide", "gate_hold_n",
                        "n", "tel_us", "kill_confirm_n"})


#: ⚠️⚠️ **bool** 型別的節點參數。契約表內部一律用 0.0/1.0 表示,但重跑提示
#: 必須印回 `True`/`False` —— 印成 `1.0` 貼上去會變成
#: `-p tgo_from_predictor:=1.0` 餵給宣告為 BOOL 的參數 ⇒
#: InvalidParameterTypeException ⇒ 節點整個起不來。
#: 這與上面 INT_PARAMS 是**同一類坑的第三次**(:g 印整數、整數參數被 _f() 加小數點、
#: 現在是 bool 被印成浮點)。三次都是「照著建議貼,然後整批報廢」。
BOOL_PARAMS = frozenset({"tgo_from_predictor", "tgo_ring_transit", "itcg_on",
                         "weave_time_sync", "ring_pace",
                         "release_at_ring", "dive_from_runup", "wait_goto_radius",
                         "mpng_on",
                         "dive_ring_gate", "att_require_guided"})


def _is_int_param(name: str) -> bool:
    return name in INT_PARAMS


def _is_bool_param(name: str) -> bool:
    return name in BOOL_PARAMS


def rerun_hint(violations) -> str:
    """把違規清單變成一段可以直接貼的重跑指令。"""
    env = []
    unproven = []
    for v in violations:
        name = v.name
        if name not in ENV_FOR or name not in JET_REFERENCE:
            continue
        # ⚠️⚠️ **不可以用 :g** —— 它把 1.0 印成 `1`,而貼上去之後
        #    rclpy 會把 `1` 當 INTEGER 餵給宣告為 DOUBLE 的參數 ⇒
        #    InvalidParameterTypeException ⇒ 節點整個起不來。
        #    也就是「照著建議貼,然後整批報廢」—— 比不給建議更糟。
        #    浮點一律保留小數點;整數參數(靠 int 判定)才印整數。
        _v = float(JET_REFERENCE[name])
        # 整數參數印整數;其餘**一律保留小數點**(repr 對 float 必帶 `.`)。
        if _is_bool_param(name):
            _s = "True" if _v > 0.5 else "False"
        elif _is_int_param(name):
            _s = f"{int(_v)}"
        else:
            _s = repr(_v)
        env.append(f"{ENV_FOR[name]}={_s}")
        if name in UNPROVEN:
            unproven.append(name)
    if not env:
        return ""
    out = ["建議重跑(已驗證值;逐項對照上面的反證再決定):",
           "  " + " ".join(sorted(set(env))) + " \\",
           "    bash rep_interactive.sh"]
    if unproven:
        out.append(f"  ⚠️ {', '.join(sorted(set(unproven)))} 的建議值**沒有實測背書** ——"
                   f"照著設是開一趟新實驗,不是修好了。")
    return "\n".join(out)


# ══ 不變式 ═══════════════════════════════════════════════════════════
#
# ⚠️ 每一條都對應一次實測踩過的坑。回傳 None = 通過,回傳 Violation = 違規。
# ⚠️ 取不到值時一律回傳 None(不判定)—— 缺值由 evaluate() 的 missing 檢查
#    負責,不變式不該因為缺值而給出誤導的結論。

def turn_radius_m(eas: float, alt_m: float, bank_deg: float = BANK_DEG) -> float:
    """給定 EAS 與高度,算物理最小迴轉半徑(m)。

    與 tot_node 自己印的告警同式:EAS → TAS 用標準大氣密度比,傾角 45°。
    """
    sigma = max(1e-3, (1.0 - 2.25577e-5 * alt_m) ** 4.25588)
    tas = eas / math.sqrt(sigma)
    return tas * tas / (G * math.tan(math.radians(bank_deg)))


def _inv_dive_pace_above_vmin(v):
    pace, vmin, dspd = v("dive_pace"), v("vmin"), v("dive_speed")
    if None in (pace, vmin, dspd):
        return None
    eff = pace if pace > 0 else 0.8 * dspd     # 0 = 自動取 0.8×dive_speed
    if eff >= vmin:
        return None
    return Violation(
        "invariant", "dive_pace ≥ vmin", f"配速實效 {eff:.0f} < vmin {vmin:.0f}",
        "配速實效值不得低於 vmin",
        "配速低於 vmin 時同步迴路整段飽和在下限,**同時**毀掉散布與終端速度",
        "dive-pace-saturates-vmin:配速 46 ≪ vmin 95 ⇒ dive_speed=210 從未生效",
    )


def _inv_dive_pace_has_headroom(v):
    """⚠️ 兩側都要有餘裕。貼上限與貼下限一樣是失去權限。"""
    pace, dspd = v("dive_pace"), v("dive_speed")
    if None in (pace, dspd):
        return None
    eff = pace if pace > 0 else 0.8 * dspd
    if eff <= dspd * 0.95:
        return None
    return Violation(
        "invariant", "dive_pace ≤ dive_speed(留餘裕)",
        f"配速實效 {eff:.0f} vs dive_speed {dspd:.0f}",
        "配速實效值不得超過 dive_speed 的 95%",
        "貼上限時飛機「想更快但追不上」,同步迴路同樣沒有權限可用",
        "handoff §4.5:修配速後全部貼上限 210;散布與飽和的**不對稱程度**相關"
        "(RPK1 兩機都 82/83% → 0.78 s;RPL3 70% vs 0% → 4.40 s)",
    )


def _inv_vmin_above_airspeed_min(v):
    vmin, amin = v("vmin"), v("AIRSPEED_MIN")
    if None in (vmin, amin) or vmin >= amin:
        return None
    return Violation(
        "invariant", "vmin ≥ AIRSPEED_MIN", f"vmin {vmin:.0f} < AIRSPEED_MIN {amin:.0f}",
        "節點速度下限不得低於飛控空速下限",
        "低於飛控下限的速度指令被 MAV_RESULT=4 **整發拒收**(不是夾住),且零告警",
        "catch-speed-rejected-by-fc;formation-layer-cannot-fly-the-jet",
    )


def _inv_vmax_below_airspeed_max(v):
    vmax, amax = v("vmax"), v("AIRSPEED_MAX")
    if None in (vmax, amax) or vmax <= amax:
        return None
    return Violation(
        "invariant", "vmax ≤ AIRSPEED_MAX", f"vmax {vmax:.0f} > AIRSPEED_MAX {amax:.0f}",
        "節點速度上限不得高於飛控空速上限",
        "超出上限的速度指令整發拒收,追趕通道等於關閉",
        "catch-speed-rejected-by-fc:catch_speed 157 > AIRSPEED_MAX 150 ⇒ MAV_RESULT=4",
    )


def _inv_hold_radius_flyable(v):
    """⚠️⚠️ RPL3 的 log 印了這條的 ERROR **然後照跑**——正是契約層要治的病。"""
    hold, vmin, perch = v("hold_radius"), v("vmin"), v("perch_alt")
    if None in (hold, vmin, perch):
        return None
    r_min = turn_radius_m(vmin, perch)
    if hold >= r_min:
        return None
    return Violation(
        "invariant", "hold_radius ≥ 物理迴轉半徑",
        f"hold_radius {hold:.0f} m < 迴轉半徑 {r_min:.0f} m"
        f"(vmin {vmin:.0f} EAS @ {perch:.0f} m、{BANK_DEG:.0f}° 傾角)",
        f"守環半徑不得小於 {r_min:.0f} m",
        "飛機飛不出這個圈,守環退化成繞自己的最小圈 ⇒ **放行時距目標的距離會在 "
        "環半徑 ±迴轉半徑 之間隨機**,放行 h/d 與需求俯角因此逐機不同",
        "SW1:環 3120 m、放行 d 散到 1806–3449 m、需求俯角 38.1°→59.3°。"
        "⚠️ RPL3 實測 hold_radius=400 vs 迴轉 1252 —— tot_node 印了 ERROR 但**沒有中止**",
    )


def _inv_ready_agl_reachable(v):
    """⚠️ 兩側都會殺:太低 ⇒ 停在跑道上就放行;太高 ⇒ 永遠到不了、整趟逾時。"""
    ready, form = v("READY_AGL"), v("formation_alt")
    if None in (ready, form):
        return None
    if ready < form:
        return None
    return Violation(
        "invariant", "READY_AGL < formation_alt",
        f"READY_AGL {ready:.0f} ≥ formation_alt {form:.0f}",
        "就緒門檻必須低於編隊層會把飛機帶到的高度",
        "門檻不可達 ⇒ 閘門永遠不成立 ⇒ 整趟逾時,而且沒有任何訊息說明為什麼",
        "RPB1/RPC1 兩趟因此報廢;handoff 缺陷#4 的反向失效",
    )


def _inv_gate_r_within_bounds(v):
    gate, lo, hi = v("gate_r"), v("gate_r_min"), v("gate_r_max")
    if gate is None:
        return None
    if lo is not None and gate < lo:
        return Violation(
            "invariant", "gate_r ≥ gate_r_min", f"gate_r {gate:.0f} < gate_r_min {lo:.0f}",
            f"交棒距不得低於 {lo:.0f}",
            "交棒太晚 ⇒ PN 擁有的航程比例不足,幾何上卸不掉殘餘高度",
            "u1-descent-authority:gate_r/環半徑 ≤0.446 的十趟殘餘 506–2499 m",
        )
    if hi is not None and gate > hi:
        return Violation(
            "invariant", "gate_r ≤ gate_r_max", f"gate_r {gate:.0f} > gate_r_max {hi:.0f}",
            f"交棒距不得高於 {hi:.0f}",
            "交棒太早 ⇒ PN 在放行瞬間就接手而退化",
            "u1-descent-authority:小環上 gate > 放行距離會讓 PN 退化(AD1 反證)",
        )
    return None


#: PN 可靠追蹤的需求俯角上限(度)。
#: ⚠️ 這是**實測分界**不是設計值:
#:   * P91 需求 γ 29–33° ⇒ PN 追到 0.1–0.6° 內、六發全中
#:   * SW1 需求 γ 64–69° ⇒ PN 只做到 53–66°、全脫靶
#:   * RPK/RPL 十二架次:γ ≤ 29.1° 三架全中(殘餘 0.75/1.23/4.06 m);
#:     γ ≥ 33.8° 四架全敗(殘餘 412/623/860/1513 m);30.6–33.4° 混合
#: 取 33.0 作為硬上限(失敗端最低值 33.8 再留一點餘裕)。
PN_GAMMA_MAX_DEG = 33.0


#: ⚠️⚠️ 交接高度**會超過** perch_alt,不可以拿理想值算。
#: RPK/RPL 十二架次實測交接高度 2286–3510 m(perch_alt 名目 3100):
#: 有五架高於 3100,最高 3510 = perch × 1.13。用 perch_alt 當最壞情況
#: 會讓契約**放過現行組態**(算出 32.9° < 33°,而實際最壞是 36.3°)。
PERCH_OVERSHOOT = 1.13


def required_gamma_deg(perch_alt: float, gate_r: float,
                       overshoot: float = PERCH_OVERSHOOT) -> float:
    """交接瞬間**最壞情況**的需求俯角(度)。

    高度有上界(perch_alt × 超調)而交接距離沒有 ⇒ 拉大 gate_r 會把最壞的 γ
    壓下來。這正是「讓 PN 早一點接手」真正的作用機制:
    **不是多給時間,是降低陡度。**

    ⚠️ 拉大 gate_r **單獨做不到**把 γ 壓進實測全中的 ≤29.1° 帶:
       gate_r 被環半徑(= perch_alt/tan(dive_angle))夾住,而超調 13%
       會把 γ 頂回 30–32°。要進全中帶必須**同時**收斂交接高度的散布
       (實測 2286–3510,±600 m)。
    """
    h = max(0.0, perch_alt) * max(1.0, overshoot)
    return math.degrees(math.atan2(h, max(1e-6, gate_r)))


# ⚠️⚠️ 為什麼「需求俯角」**不是**硬性契約檢查(2026-08-13,想清楚後撤掉的)
#
# 一度想把 `γ需求 = atan(perch_alt/gate_r) ≤ 上限` 做成不變式。**不能做。**
#
#     TWO1      gate 4800 / perch 3100 ⇒ γ_nominal 32.9°  ⇒ 2/2 達標(殘餘 0.72/2.59 m)
#     RPK/RPL   gate 4800 / perch 3100 ⇒ γ_nominal 32.9°  ⇒ 2/6(殘餘 0.75–2423 m)
#
# **同一組設計幾何,結果天差地遠** ⇒ γ_nominal 對成敗零鑑別力。
# 門檻設 29.1° 會誤擋 TWO1(實測成功過的組態);設 33° 則兩者都過、什麼也沒擋到。
# 真正的決定量是**交接時的實際高度**(RPK/RPL 實測 2286–3510,±600 m 散布),
# 而那是起飛前讀不到的量。
#
# ⚠️ 這一層最不能有的就是誤報 —— 擋掉已知可行的組態,人就會把它關掉。
# ⇒ `required_gamma_deg()` 保留為**報告量**(寫進生效值快照供事後判讀),
#   不進 INVARIANTS。真正的把關留給下面那條幾何硬約束。


def _inv_gate_r_below_ring(v):
    """⚠️ gate_r 不可以碰到放行距離(環半徑)。

    已知失效模式:gate ≥ 放行距離 ⇒ PN 在放行**瞬間**就接手而退化
    (u1-descent-authority 用 AD1 佐證「小環救不回」)。
    ⇒ 拉大 gate_r 有上界,不是越大越好。
    """
    gate, perch, ang = v("gate_r"), v("perch_alt"), v("dive_angle")
    if None in (gate, perch, ang) or not (0.0 < ang < 89.9):
        return None
    ring = perch / math.tan(math.radians(ang))     # auto_ring 的反推式
    if gate <= ring * 0.95:
        return None
    return Violation(
        "invariant", "gate_r < 環半徑(留餘裕)",
        f"gate_r {gate:.0f} vs 環半徑 {ring:.0f}(= perch_alt/tan(dive_angle))",
        f"gate_r ≤ {ring * 0.95:.0f} m",
        "交接距離貼上放行距離 ⇒ PN 在放行瞬間就接手,退化成沒有交接",
        "u1-descent-authority:小環上 gate > 放行距離會讓 PN 退化(AD1 反證)",
    )


def _inv_hold_alt_leaves_enough_homing_time(v):
    """⭐ Zarchan 伴隨法的下界:交接高度太低 ⇒ 終端脫靶是**物理必然**。

    《Tactical and Strategic Missile Guidance》第 3/6 章:單一遲滯歸向迴路裡
    導引飛行時間 `t_F < 3~5τ` 時,終端脫靶無法靠調參解決。取 5τ 當保守值:

        h_min = V·sin(γ) · 5τ

    ⚠️ 目前**不是瓶頸**:實測交接在 2247–3510 m,而 γ=30°、V=180 的下界只有 900 m。
       寫進來是因為「提早下降」這個方向的終點必然會撞到它 —— 一路壓低 hold_alt
       總會越過這條線,而越過之後的症狀(殘餘高度清不掉)與**現在**的症狀
       長得一模一樣,不會有人發現基準已經換了。
    ⚠️ τ = 2.0 s 是估的、沒有受控實測。這條不變式的門檻因此帶著那個不確定性,
       所以取 5τ(保守端)而不是 3τ。
    """
    ha, ang = v("hold_alt"), v("dive_angle")
    if ha is None or ha <= 0.0 or ang is None:
        return None                      # 未啟用 hold_alt 時不判定
    try:
        from sar_swarm.predictor import min_handover_altitude_m
    except ImportError:                  # pragma: no cover
        from predictor import min_handover_altitude_m
    # 用保守的速度估:速度越低,下界越低 ⇒ 取實測俯衝段的低端 170 m/s
    need = min_handover_altitude_m(ang, 170.0)
    if ha >= need:
        return None
    return Violation(
        "invariant", "hold_alt 需留足終端導引時間",
        f"hold_alt {ha:.0f} < 下界 {need:.0f} m(γ={ang:.0f}°、V=170、5τ)",
        f"hold_alt >= {need:.0f} m",
        "終端導引時間不足 5 倍系統時間常數 ⇒ 殘餘高度清不掉是**物理必然**,"
        "不是調參問題;而症狀與現在的失敗長得一樣,不會有人發現基準換了",
        "Zarchan《Tactical and Strategic Missile Guidance》第 3/6 章伴隨法:"
        "t_F < 3~5τ 時終端脫靶不可避免",
    )


def _inv_weave_period_can_actually_absorb_time(v):
    """⭐⭐ 開了時間同步蛇形卻用泡棉時代的週期 = **看起來有做、實際吸不到時間**。

    換向率 ω = g·tanφ/V。把航向從 +θ 甩到 −θ 需要 2θ/ω,而它必須塞得進
    半個週期的一半,否則飛機還沒轉到就被要求反向 ⇒ 實得 cosθ 遠大於指令值。
    可用上限:θ_max = ω·P/8。路徑拉長倍率 1/cosθ_max 決定能吸收多少 t_go:

        V=157、P= 22 s ⇒ ω=3.58°/s ⇒ θ ≤  9.8° ⇒ 吸收  1.5%(t_go 30 s ⇒ 0.45 s)
        V=157、P=120 s ⇒ ω=3.58°/s ⇒ θ ≤ 53.7° ⇒ 吸收 41.4%(≈12 s)

    我們需要吸收 1–14 s(實測交接時間差),而散布要 ≤3 s ⇒ 時間差 ≤1.8 s。
    ⚠️ 22 s 是泡棉機時代的值(14 m/s 時 ω=40.1°/s,週期短也夠轉)。
       這一條擋的正是「開關打開了、參數卻還是舊機體的」那一類 ——
       它不會報錯、不會有告警,只會讓一整批航次的結論變成「蛇形沒用」。
    """
    on, per = v("weave_time_sync"), v("weave_period")
    if on is None or on <= 0.0 or per is None:
        return None                      # 未啟用不判定
    vmax = v("vmax") or 150.0
    omega = math.degrees(G * math.tan(math.radians(BANK_DEG)) / max(20.0, vmax))
    th_max = omega * per / 8.0
    absorb = (1.0 / math.cos(math.radians(min(60.0, th_max)))) - 1.0
    if absorb >= 0.10:                   # 至少要吸收得到 10% 的 t_go 才有意義
        return None
    need = 8.0 * math.degrees(math.acos(1.0 / 1.10)) / omega
    return Violation(
        "invariant", "weave_period 要大到吸收得到時間",
        f"weave_period {per:.0f}s ⇒ 換向率夾限 θ ≤ {th_max:.1f}° ⇒ "
        f"只吸收得到 t_go 的 {absorb:.1%}",
        f"weave_period >= {need:.0f}s(至少吸收 10%)",
        "週期太短 ⇒ 飛機還沒轉到指令角度就被要求反向 ⇒ 實得 cosθ 遠大於指令值 "
        "⇒ 蛇形形同虛設,而且**不會有任何告警**",
        "tot_node.py weave 段自述:157 m/s 的換向率只有 3.58°/s,"
        "22 s 週期之下任何 θ>19.7° 都轉不完;120 s 之下 θ=45° 才吸收得到 105 s",
    )


def _inv_gate_clears_the_loiter_swing(v):
    """⚠️⚠️ RPO 實測(2026-08-13)當場燒出來的一條:**gate 必須在守環擺盪之外**。

    守環是 loiter,飛機在環點周圍以 hold_radius 繞圈 ⇒ 距紅心的水平距在
        [環半徑 − hold_radius, 環半徑 + hold_radius]
    之間擺盪。若 gate 落在這個帶子**裡面**,放行當下飛機可能已經在 gate 內側,
    於是「當場交棒」而不是在進場途中乾淨穿越 ⇒ **交棒距離變成隨機量**。

    RPO 實測代價(gate 5600、hold_radius 1300、環 6147 ⇒ 擺盪 4847–7447,gate 在帶內):
        交棒距離 4765–4787 m(±11)  →  3430–5579 m(±1075)
        真落地   5/12               →  1/6
    對照舊組態(gate 4800、hold_radius 400 ⇒ 擺盪 5747–6547,gate 在帶外)每架都乾淨穿越。

    ⚠️ 這條與 `gate_r < 環半徑` 不同,而且更嚴:當初只驗 0.95×環=5780,
       5600 順利通過 —— 但真正的邊界是 6147−1300=4847。
       **兩個單獨看都合法的參數,相乘出災難** —— 契約層存在的理由。
    """
    gate, perch, ang, hold = (v("gate_r"), v("perch_alt"), v("dive_angle"),
                              v("hold_radius"))
    if None in (gate, perch, ang, hold) or not (0.0 < ang < 89.9):
        return None
    ring = perch / math.tan(math.radians(ang))
    inner = ring - hold                      # 擺盪內緣
    if gate < inner:
        return None
    return Violation(
        "invariant", "gate_r < 環半徑 − hold_radius",
        f"gate_r {gate:.0f} ≥ 擺盪內緣 {inner:.0f}"
        f"(環 {ring:.0f} − hold_radius {hold:.0f})",
        f"gate_r < {inner:.0f} m",
        "gate 落在守環擺盪帶內 ⇒ 放行時飛機可能已在 gate 內側 ⇒ 當場交棒 ⇒ "
        "交棒距離變成隨機量,而交棒幾何是命中與否的唯一自變數",
        "RPO(gate 5600 / hold 1300 / 環 6147)實測:交棒距離 ±11 m → ±1075 m、"
        "真落地 5/12 → 1/6;RPK/RPL(gate 4800 / hold 400)每架乾淨穿越",
    )




def _inv_agl_floor_above_kill(v):
    """⚠️⚠️ AGL 地板必須高於觸水終結線,否則它永遠來不及作用。

    `kill_agl_m` 是**驗屍**不是保護(agl-datum-false-kill:它甚至誤殺過巡航中的
    飛機)。`att_agl_floor` 才是保護。地板設在終結線之下 = 飛機先被終結、
    地板從未觸發 ⇒ 一個看起來有開、實際上永遠不生效的安全機制。
    """
    floor, kill = v("att_agl_floor"), v("kill_agl_m")
    if floor is None or kill is None or floor <= 0.0:
        return None                      # 0 = 停用,不判定
    if floor > kill:
        return None
    return Violation(
        "invariant", "att_agl_floor > kill_agl_m",
        f"att_agl_floor {floor:.1f} <= kill_agl_m {kill:.1f}",
        f"att_agl_floor > {kill:.1f} m",
        "AGL 地板低於觸水終結線 ⇒ 飛機先被終結,地板永遠不會觸發 ⇒ "
        "一個看起來有開、實際上不存在的保護",
        "agl-datum-false-kill:kill 是驗屍不是保護,它甚至誤殺過 150 m/s 巡航中的飛機",
    )


def _inv_eas_floor_above_airspeed_min(v):
    """⚠️ EAS 地板低於飛控空速下限 = 沒有保護。

    ⚠️⚠️ 而且這個量**必須是 EAS 不是地速**:157 是 3100 m 的地速(= EAS 135),
    拿 157 當 EAS 門檻會超天花板 17%(eas-vs-groundspeed-157)。
    """
    floor, amin = v("att_eas_floor"), v("AIRSPEED_MIN")
    if floor is None or floor <= 0.0:
        return None
    if amin is not None and floor < amin:
        return Violation(
            "invariant", "att_eas_floor >= AIRSPEED_MIN",
            f"att_eas_floor {floor:.1f} < AIRSPEED_MIN {amin:.1f}",
            f"att_eas_floor >= {amin:.1f}",
            "EAS 地板低於飛控空速下限 ⇒ 飛機已經在被拒收指令的區間裡了,地板才作用",
            "catch-speed-rejected-by-fc:低於下限的速度指令被 MAV_RESULT=4 整發拒收",
        )
    # ⚠️ 上界綁 vmin 而不是一個魔術數字:地板 >= 導引自己的速度下限 ⇒
    #    飛機在正常守環(v_cmd = vmin)時就會一直低於地板 ⇒ 姿態通道全程被擋。
    #    誤把**地速** 157 當 EAS 填進來正好落在這裡(vmin 95 < 157)。
    vmin = v("vmin")
    if vmin is not None and floor >= vmin:
        return Violation(
            "invariant", "att_eas_floor 是 EAS 不是地速",
            f"att_eas_floor {floor:.1f} >= vmin {vmin:.1f}",
            f"att_eas_floor < vmin({vmin:.0f});EAS 量綱",
            "把地速當 EAS 填進來 ⇒ 地板永遠成立 ⇒ 姿態通道全程被擋、飛機交還 TECS",
            "eas-vs-groundspeed-157:157 是 3100 m 的**地速**,對應 EAS 只有 135",
        )
    return None


def _inv_descending_wait_needs_throttle_off_the_floor(v):
    """⭐⭐ 開了下降式等待就必須同時把守環速度提起來,否則它做不到。

    ⚠️⚠️ 機制:節點在守環時同時下令「降到目標高度 + v_cmd = vmin」,
    而實測 12/12 架次地速比指令高 +1.6~+61 m/s ⇒ 速度誤差恆為負
    ⇒ 油門需求打到 THR_MIN=0 ⇒ 觸發 `AP_TECS.cpp:1245-1252` 的抗失控閂鎖
    (油門觸底就把降高需求扣在上一拍,而且**不設任何旗標**)
    ⇒ **每一次都自己把下降權限關掉**。

    只設 hold_alt 不設 hold_v = 下令下降然後自己擋住它,而且完全靜默 ——
    那會產出一趟「改了但沒差別」的資料,然後被誤讀成「下降式等待沒用」。
    """
    ha, hv, vmin = v("hold_alt"), v("hold_v"), v("vmin")
    if ha is None or ha <= 0.0:
        return None                      # 未啟用
    if hv is not None and vmin is not None and hv > vmin:
        return None
    return Violation(
        "invariant", "hold_alt 啟用時必須同時設 hold_v",
        f"hold_alt {ha:.0f} 已啟用,但 hold_v = {hv} (vmin = {vmin})",
        "hold_v > vmin(建議取計畫巡航 EAS)",
        "守環同時下令『降高 + vmin』⇒ 油門觸底 ⇒ TECS 抗失控閂鎖扣住降高需求 ⇒ "
        "下降指令自己被自己擋掉,而且不設旗標、dataflash 看不到",
        "AP_TECS.cpp:1245-1252;實測 12/12 架次守環地速比 v_cmd 高 +1.6~+61 m/s",
    )


#: 不變式清單。每條簽名都是 `f(getter) -> Violation | None`。
def _inv_predictor_actuation_needs_shadow_observation(v):
    """拿預測器當**致動**來源(tgo_from_predictor=1),就必須同時開影子觀測。

    ⚠️⚠️ 這一條擋的是本專案自己的方法論被繞過:S9 的影子模式存在的理由,
       就是「先證明它在飛行中準不準,再談拿它做決策」。RPY3 直接跳過了
       第一步 —— tgo_from_predictor=1(致動)但 predict_hz=0(不觀測)。
       結果:預測器算出來的 t_go 決定了整個放行時刻表,而**沒有任何一欄**
       記錄它算了什麼,事後連「它到底準不準」都無從回答。
       這不是效能問題,是可觀測性問題:壞掉的時候你不會知道。
    """
    on, hz = v("tgo_from_predictor"), v("predict_hz")
    if on is None or on <= 0.0:
        return None                      # 沒用預測器致動 ⇒ 不強制觀測
    if hz is not None and hz > 0.0:
        return None
    return Violation(
        "invariant", "預測器致動必須配影子觀測",
        f"tgo_from_predictor={on:.0f}(致動)但 predict_hz={0.0 if hz is None else hz}",
        "predict_hz > 0(建議 2.0)",
        "預測器的輸出決定放行時刻表,卻沒有任何一欄記錄它算了什麼",
        "RPY3:pred_h_cross/pred_gs_end/pred_tgo/pred_sat 四欄整批空白,"
        "而同一趟的 t_go 已經在決定每一架什麼時候放行",
    )


def _inv_itcg_enabled_needs_real_gains(v):
    """ITCG 開著就必須有**非零**的增益與上限。

    ⚠️⚠️ 「開了但沒效果」是本專案最貴的一類失效:契約顯示 itcg_on=1、
       log 一切正常、飛出來只是「沒改善」,於是結論變成「ITCG 沒用」——
       而真相是它從來沒出過力。同一形狀已經發生過三次:
       JET_CDP 空字串、predict_hz=0、pn_los_floor=0。
    """
    on = v("itcg_on")
    if on is None or on <= 0.0:
        return None                      # 沒開 ⇒ 不判定
    k, cap = v("itcg_gain"), v("itcg_a_max_g")
    bad = [n for n, x in (("itcg_gain", k), ("itcg_a_max_g", cap))
           if x is None or x <= 0.0]
    if not bad:
        return None
    return Violation(
        "invariant", "ITCG 開著卻沒有增益",
        f"itcg_on=1 但 {', '.join(bad)} 為 0/讀不到",
        "itcg_gain > 0 且 itcg_a_max_g > 0",
        "時間項被夾成 0 ⇒ 導引律逐位元退回純 PN,而契約與 log 都顯示已啟用",
        "RQA2:純 PN 在放行差 −2.5s 的情況下仍散開 21.2s —— "
        "若 ITCG 名義上開著卻沒出力,會被誤讀成「ITCG 也解決不了」",
    )


def _inv_mpng_needs_itcg_on(v):
    """MPNG 開著就必須 `itcg_on=1`,否則整條律**一次都不會被呼叫到**。

    ⚠️⚠️ `_itcg_bias` 的第一行是 `if not self.itcg_on ... return 0.0, None` ——
       `mpng_on=1` 而 `itcg_on=0` 會讓契約快照印出「MPNG 已啟用」、log 一切
       正常、CSV 六欄全空,而那一批的結論會變成「MPNG 也沒用」。
       這是本專案的頭號失效類別(「開關開著但那一段從來沒真的發生」,
       已經第五次:JET_CDP 空字串、predict_hz=0、pn_los_floor=0、
       catch_speed 被 FC 整發拒收、itcg_accel 80,760 樣本全零)。

    ⭐ 值層級的鎖:不看程式碼長什麼樣,只看送上線的那組值。
       (源碼字串比對的 38 條鎖被 ≥25 種等價改寫繞過過。)
    """
    on = v("mpng_on")
    if on is None or on <= 0.0:
        return None                      # 沒開 ⇒ 不判定
    itcg = v("itcg_on")
    if itcg is not None and itcg > 0.0:
        return None
    return Violation(
        "invariant", "MPNG 開著但 ITCG 總開關是關的",
        f"mpng_on=1 但 itcg_on={0.0 if itcg is None else itcg}",
        "itcg_on=1(MPNG 掛在 _itcg_bias 裡,總開關關著就整段不執行)",
        "契約與 log 都會顯示 MPNG 已啟用,但 _itcg_bias 第一行就 return ⇒ "
        "六欄儀表全空、a_h 完全沒有時間項",
        "itcg_accel 80,760 個樣本全零 —— 這條通道從來沒承載過訊號,"
        "而先前沒有任何一層會告訴你",
    )


INVARIANTS: tuple[Callable, ...] = (
    _inv_itcg_enabled_needs_real_gains,
    _inv_mpng_needs_itcg_on,
    _inv_predictor_actuation_needs_shadow_observation,
    _inv_gate_r_below_ring,
    _inv_gate_clears_the_loiter_swing,
    _inv_weave_period_can_actually_absorb_time,
    _inv_hold_alt_leaves_enough_homing_time,
    _inv_agl_floor_above_kill,
    _inv_eas_floor_above_airspeed_min,
    _inv_descending_wait_needs_throttle_off_the_floor,
    _inv_dive_pace_above_vmin,
    _inv_dive_pace_has_headroom,
    _inv_vmin_above_airspeed_min,
    _inv_vmax_below_airspeed_max,
    _inv_hold_radius_flyable,
    _inv_ready_agl_reachable,
    _inv_gate_r_within_bounds,
)


# ══ 評估器 ═══════════════════════════════════════════════════════════

@dataclass
class Report:
    violations: list
    checked: list          # [(name, source, got, 判定)]
    missing: list          # 讀不到的契約參數名
    skipped: list = None   # 擁有者不在此組態 ⇒ N/A(必須印出來)

    @property
    def ok(self) -> bool:
        return not self.violations

    def text(self) -> str:
        out = ["══ 生效值契約 ══",
               f"{'參數':<22}{'來源':<7}{'生效值':>12}  判定"]
        for name, src, got, verdict in self.checked:
            out.append(f"{name:<22}{src:<7}{str(got):>12}  {verdict}")
        # ⚠️ N/A **一定要印**。不印就是靜默跳過,而靜默跳過正是這一層要治的病。
        if self.skipped:
            out.append(f"\n⚠️ 以下 {len(self.skipped)} 項本次不適用"
                       f"(擁有它的節點不在這個組態裡,已從 `ros2 node list` 查證):")
            for n in self.skipped:
                out.append(f"    • {n}")
        if self.violations:
            out.append("")
            out.extend(v.line() for v in self.violations)
            out.append(f"\n✗ {len(self.violations)} 項違反契約 —— **不要開跑**")
        else:
            out.append("\n✅ 契約全數通過")
        return "\n".join(out)


def _as_float(x):
    try:
        f = float(x)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def evaluate(values: dict, *, require: Optional[Iterable[str]] = None,
             not_applicable: Optional[Iterable[str]] = None) -> Report:
    """對照契約表評估一組**生效值**。

    `values` 的鍵是參數名,值是從節點生效值行 / 飛控 PARM / shell 讀回來的量。
    `require` 給定時只檢查這些參數(其餘視為本次不適用);None = 全表。

    ⚠️ 讀不到 = 違規,不是跳過。「沒讀到所以算過」正是這一整類 bug 的形狀。

    `not_applicable` 是**唯一**的例外,而且語意很窄:
    **擁有這個參數的節點在這個組態下根本不存在。**
    實例:`cruise_alt_threshold` 只在 `wingman_node` 宣告,而 n=2 是
    「一長一備長」沒有僚機 ⇒ 該參數在 n=2 下不存在,不是「讀失敗」。

    ⚠️⚠️ 兩者必須嚴格分開:
        擁有者在跑、參數讀不到      ⇒ **違規**(可能是名字打錯、節點崩了)
        擁有者根本不在這個組態裡    ⇒ N/A,但**一定要印出來**
    呼叫端必須從 `ros2 node list` **實際查證**擁有者不在,
    不可以因為「讀不到」就自己歸到 N/A —— 那就退化成「沒讀到所以算過」。
    """
    na = set(not_applicable or ())
    wanted = [c for c in TABLE if require is None or c.name in set(require)]
    violations, checked, missing, skipped = [], [], [], []

    for c in wanted:
        if c.name in na:
            skipped.append(c.name)
            checked.append((c.name, c.source, "—", "N/A 擁有者不在此組態"))
            continue
        raw = values.get(c.name)
        got = _as_float(raw)
        if got is None:
            missing.append(c.name)
            checked.append((c.name, c.source, "—", "✗ 讀不到"))
            violations.append(Violation(
                "missing", c.name, raw, "必須能從實際生效值讀回",
                c.why,
                "讀不到就無法比對 ⇒ 等於沒有檢查。" + c.evidence))
            continue

        # 1) 泡棉禁區:獨立於區間判定。命中即視為「沒有人設過它」。
        if c.foam is not None and abs(got - c.foam) < 1e-9:
            checked.append((c.name, c.source, got, "✗ 泡棉禁區"))
            violations.append(Violation(
                "foam", c.name, got, f"≠ {c.foam}(泡棉時代預設)",
                c.why,
                f"{got} 正是 declare_parameter 的泡棉預設值 ⇒ "
                f"餵值路徑斷了、靜靜退回原廠常數。" + c.evidence))
            continue

        # 2) 區間
        bad = (c.lo is not None and got < c.lo) or (c.hi is not None and got > c.hi)
        if bad:
            want = f"[{c.lo if c.lo is not None else '−∞'}, " \
                   f"{c.hi if c.hi is not None else '+∞'}]"
            checked.append((c.name, c.source, got, "✗ 區間外"))
            violations.append(Violation(
                "range", c.name, got, want, c.why, c.evidence))
            continue

        checked.append((c.name, c.source, got, "OK"))

    # 3) 不變式:用完整的 values(含表外的輔助量,如 hold_radius/gate_r_min)
    def getter(name):
        return _as_float(values.get(name))

    for inv in INVARIANTS:
        v = inv(getter)
        if v is not None:
            violations.append(v)

    return Report(violations=violations, checked=checked, missing=missing,
                  skipped=skipped)
