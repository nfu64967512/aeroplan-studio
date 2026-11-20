#!/usr/bin/env python3
"""stot_score.py — 從 tot_node 的飛行記錄重算 STOT 命中 KPI(取代「最近取樣點」)。

為什麼需要這支:
  tot_node 回報的 `最近水平距` 是**所有取樣點裡最近的那一個**。俯衝地速約
  40-100 m/s,而分流線的 GLOBAL_POSITION_INT 實測只有 5-20 Hz ⇒ 相鄰取樣點
  相距 5-20 m。真正的最近通過點幾乎一定落在**兩個取樣點之間**,所以
  「最近取樣點距離」會系統性地**高估** miss —— 量測誤差和驗收門檻(5 m)同級。

  本工具改用「點到線段」的距離:機體在相鄰兩取樣點之間走直線(俯衝段本來就是
  直線),所以線段距離就是該區間真正的最近通過距離,量測量化誤差直接消掉。
  命中高度與命中時刻同樣在該點上做線性內插。

驗收基準(2026-07-28 使用者定調的優先序):
  ① 終端速度 —— 六機**平均**觸地**總速** ≥ --speed-min m/s
     ⚠️ 2026-08-14 由地速改為總速(使用者定調)。地速 = 總速×cosγ,
        在陡俯衝時退化(γ→90° 時 →0),且設計推力下地速上界只有 182。
  ② 觸地時間散布 ≤ 3 s
  ③ 觸地水平誤差 ≤ 5 m(明確列為其次)
  另加:六機都要真的落地、命中高度 ≤ --alt-max

  ⚠️ 三項一律以**觸地基準**判定,不用「最近通過點」。以前本工具的判定用最近通過
  點、而 stot_repeat.sh 用觸地基準,同一趟兩支會給出不同結論。
  ⚠️ 速度取「觸地那一點」而不是俯衝峰值:峰值出現在俯衝中段,實測比觸地速度高
  10–25 m/s(J2 的 UAV4:峰值 67.9、觸地 52.1)。峰值仍照印,只是不拿來判定。

首航次窗(2026-08-04 加,預設啟用;--no-window 回到舊行為):
  ⚠️ 舊路徑取的是**全程全域**最近通過點。SW1 證明這會被「逃逸鏈」污染到失去意義:
     impact 集合沒有 miss/timeout 出口 ⇒ 首航次脫靶的機不鎖存 ⇒ 持續滿速追 ⇒
     UAV2 飛到 207 km、UAV6 飛到 412 km 外,數小時後隨機重新穿過紅心才被鎖存。
     全域最小值就取到那個「隨機重穿」,於是散布算出 11950 s,而且那兩架看起來
     像是打中了(95 m / 6 m)。這是**假的**。

     所以評分只看「首航次」:從進入俯衝那一刻起,第一次接近再遠離的那一段。
     首航次沒打中就是 MISS —— 不准用後面幾小時的隨機重穿補救。
     ⚠️ 不要為了讓 UAV2/UAV6「算命中」去放寬 miss_r:重放已證明放寬到 800 m 會
        讓它們在 1498 m / 1069 m 高空被判命中,製造幽靈命中。

  ⚠️⚠️ 「幾小時後的重穿」不是唯一的污染源,**同一分鐘內的第二趟**才是最陰險的:
     SITL 不會因為機體觸海就終結它。SW1 的 UAV3 在 t=1012.6 觸海、掉到海面下
     95.7 m,然後**爬回 +331 m**,24.1 s 後再穿一次紅心(22.58 m);UAV5 同型
     (17.9 s 後 67.11 m)。這兩個數字一度被當成首航次寫進交接文檔,而真值是
     32.62 / 217.90 m。回升量只有 555 / 637 m,遲滯 1000 m 收不掉它 ——
     調小遲滯也不行:UAV3 放行時的外衝就有 923 m,兩邊夾死。
     所以窗改成「**首次 AGL ≤ 0(觸地/觸海)當下無條件收掉**」:機體落水之後
     不可能再有合法的攻擊航次,這是物理,不是門檻。

用法:
  python3 stot_score.py /tmp/tot_B1.csv [--log /tmp/trial_B1.log] [--json]
  python3 stot_score.py /tmp/tot_B1.csv --no-window     # 舊的全域最近點(對照用)
"""
import argparse
import csv
import json
import math
import os
import re
import sys

EARTH = 111320.0

# ── 收尾摘要的起點哨兵(2026-08-05 blocker)────────────────────────────────
# stot_trial.sh 以前是用 `stot_score.py … | tail -12` 收尾的,那是一個**固定
# 行數預算**:degraded 區塊固定 8 行 + 每架 MISS/ESCAPE 一則警告 + 判定 1 行。
# SW1 形狀(2 架 ESCAPE)剛好 12 行、**零餘裕**;3 架 ESCAPE 就是 13 行,而
# tail -12 砍掉的第一行正好是 ①「只算 N 架 HIT」—— 收尾畫面於是從
# 「⚠️ 全員值 180.0 m/s」開頭,正是本輪要降級的灌水數字,而降級後的真值被砍掉。
# 再多一則警告(UNKNOWN、partial-agl、命中沒鎖存、mode 詞彙脫節)都會踩同一條線。
#
# 把 12 改成 20 只是把線往後挪 —— 下次加行還是會踩到。改成哨兵:文字模式在
# 收尾摘要之前印這一行,harness 從它的**下一行**取到結尾,行數自動伸縮。
# ⚠️ --json 模式**不印**:stdout 必須是純 JSON,下游 jq / stot_repeat.sh 會噎到。
# ⚠️ 這個字串是 stot_score.py 與 stot_trial.sh 之間的契約,兩邊改要一起改
#    (tests/test_score_window.py 有一條測試把兩邊釘在一起)。
SUMMARY_SENTINEL = "===STOT_SUMMARY_BEGIN==="


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def seg_closest(px, py, ax, ay, bx, by):
    """點 P 到線段 AB 的最近距離,以及最近點在 AB 上的參數 t∈[0,1]。"""
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.hypot(px - ax, py - ay), 0.0
    t = ((px - ax) * dx + (py - ay) * dy) / den
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy), t


def load_zg(log_path):
    """從 tot_node log 撈每機自量的紅心地表高度(相對 HOME)。"""
    zg = {}
    if not log_path or not os.path.exists(log_path):
        return zg
    pat = re.compile(r"UAV(\d+) 紅心地表 z_g = ([+-]?[\d.]+)")
    with open(log_path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = pat.search(ln)
            if m:
                zg[int(m.group(1))] = float(m.group(2))
    return zg


# ── 首航次窗 ──────────────────────────────────────────────────────────────
# 軌跡點的欄位順序(load 出來的 tuple):
P_T, P_LAT, P_LON, P_ALT, P_GS, P_MODE, P_AGL = range(7)


def horiz_d(lat, lon, T, ml):
    """對紅心的水平距(m)。"""
    return math.hypot((lat - T[0]) * EARTH, (lon - T[1]) * ml)


# ── 放行點的 mode 判準 ─────────────────────────────────────────────────────
# DIVE 相底下 tot_node 寫得出來的 mode 一共六個(repo 版 tot_node.py):
#   :2007 missed        :2022 runin / wait
#   :2037 expended / impact                     :2060 dive
# 兩堆**都要列出來**:只列一堆的話,節點端新增 mode 時會被靜默歸進另一堆。
#
# 放行**之前**可能出現的 mode:
#   wait   已在環上繞圈等錯開放行
#   runin  還在 30 km 外轉場,指令速度 vmax
#   missed 逾時/脫靶出口(M1/M2)。
# ⚠️⚠️ missed **不專屬於放行前**(2026-08-05 blocker)。tot_node:2004 刻意把
#    _miss_check 放在 `if not released` **之前**(註解:「從未放行、卡在環上
#    不動的機一樣走得到出口」),所以同一個 mode 兩種列序都會出現:
#      從未放行  wait/runin → missed …          (整段都在環上,沒有攻擊航次)
#      放行後脫靶 wait/runin → dive → … → missed(有攻擊航次,放行點在 dive)
#    ⇒ missed 既不能當「已放行」的證據,也不能當「還沒放行」的證據。
#    所以它列在這裡只是為了「不算放行證據」,判準本身**不看**這一堆。
PRE_RELEASE_MODES = ("wait", "runin", "missed")

# 放行**之後**才可能出現的 mode —— 放行點的判準用的是**這一堆**。
# ⚠️⚠️ 方向是刻意選的(2026-08-05 blocker)。兩種寫法在節點端改名/新增 mode
#    時的失效方向剛好相反:
#      排除式(mode not in PRE_RELEASE_MODES)
#        ⇒ 新增的**放行前** mode 被當成放行 ⇒ 窗從環上起算、d1 是繞圈的局部
#          最小,而 start 照樣標 "release" ⇒ **靜默**。這正是 "missed" 這次
#          踩到的形狀:整段 wait→missed 的機被判成 MISS(甚至 HIT),
#          no-release-found / UNKNOWN 機制完全不觸發。
#      納入式(mode in POST_RELEASE_MODES)
#        ⇒ 新增的**放行後** mode 找不到 ⇒ 退回 no-release-found ⇒ UNKNOWN
#          + ⚠️⚠️ 警告 ⇒ **出聲**。
#    靜默失敗是這專案的頭號死因,所以選出聲的那一邊。
POST_RELEASE_MODES = ("dive", "impact", "expended")

# 兩堆的聯集 = 節點端 DIVE 相的完整 mode 詞彙。落在這之外的值一定是節點端
# 改了名或新增了狀態,main() 會為它印一條專屬警告(見 unknown_modes)——
# 判準只會因此變保守(找不到放行點 ⇒ UNKNOWN),但沉默是不允許的。
KNOWN_DIVE_MODES = frozenset(PRE_RELEASE_MODES) | frozenset(POST_RELEASE_MODES)

# window_start 的回傳標記。放行點找不到時**不可以**只回一個索引就算了 ——
# 見 window_start 的 docstring(2026-08-04 blocker)。
START_RELEASE = "release"
START_NO_RELEASE = "no-release-found"


def unknown_modes(pts):
    """軌跡裡出現、但不在 KNOWN_DIVE_MODES 的 mode 值(排序後的 list)。

    放行點是靠 mode 字串反推的代理量。節點端改名時,判準會退到保守側
    (找不到 POST_RELEASE_MODES ⇒ no-release-found ⇒ UNKNOWN),但那個
    UNKNOWN 看起來會像「這一趟沒放行」而不是「工具跟節點脫節了」——
    兩者的處置完全不同,所以要單獨吼一句,把陌生的 mode 名字印出來。
    """
    return sorted({p[P_MODE] for p in pts
                   if p[P_MODE] not in KNOWN_DIVE_MODES})


def window_start(pts):
    """首航次窗的起點 —— 回傳 **(索引, 標記)**,索引 = 第一筆放行後的取樣。

    ⚠️ 不能直接用「第一筆 phase == DIVE」。tot_node 在 phase 翻成 DIVE 之後,
       還沒輪到放行的機會以 mode == "wait"/"runin" 繼續轉場或在環上盤旋 ——
       那是繞圈與轉場,不是俯衝。環的周長讓 d(t) 一圈上下擺盪 2000–4000 m,
       首航次偵測會咬到環上的某個局部最小值就收工。
    ⚠️⚠️ 只跳 wait 不夠(這是 2026-08-04 的 blocker):SW1 的 UAV5 在 DIVE 的
       頭 284 筆是 **runin**(t=939.9–955.2),窗因此從 t=939.9 / d=4706 m
       起算而不是放行的 t=964.5。量化過的後果:窗從第一筆 DIVE 起算時,各機
       「放行前 d 減跑動最小值」的最大值是 U1 0 / U2 73 / U3 1142 / U4 6 /
       U5 0 / U6 39 m —— UAV3 的 1142 m 已經超過 1000 m 遲滯,窗會在環上收掉,
       d1 從真值 32.62 m 變成 2098 m,一架真的打中的機被判成 MISS。

    ⚠️⚠️ 判準是「第一筆 mode ∈ POST_RELEASE_MODES」,**不是**「第一筆
       mode ∉ PRE_RELEASE_MODES」(2026-08-05 blocker)。差別在 "missed":
       tot_node:2004 把 _miss_check 放在 `if not released` 之前,所以從未
       放行、卡在環上的機逾時後 mode 會從 wait/runin 變成 missed。用排除式
       寫的話,那一筆 missed 會被當成放行點回 START_RELEASE ⇒ 整個
       no-release-found / UNKNOWN 機制在它唯一要救的形狀上失效:
         wait 盤旋 → missed 盤旋 ⇒ 判 MISS、d1=2400 m(守點盤旋的局部最小)
                                    被印成「攻擊航次脫靶 2.4 km」、零警告
         同形狀但 miss_pt 落在紅心附近 ⇒ 判 **HIT**、all_hit **True**
                                    ⇒ first_pass_ok 在 stot_score /
                                       stot_rescore_all.verdict /
                                       stot_repeat.sh 三處同時變 True = 幽靈達標
       而 missed 也可能出現在**放行之後**(真的俯衝、脫靶,才被 _miss_check
       標記),那種機的列序是 wait/runin → dive → … → missed,放行點必須照樣
       在 dive 那一筆被找到 —— 所以也不能單純把 missed 當成「還沒放行」。
       納入式同時滿足兩邊:看 dive/impact/expended 有沒有出現過就好。

    ⚠️ 這條規則是「放行那一刻」的**代理**,不是它本身:CSV 沒有記放行時戳,
       這裡只能靠 mode 反推。mode 的取值若在節點端改名,判準會退到保守側
       (找不到 ⇒ no-release-found ⇒ UNKNOWN + 警告),而不是靜默放行;
       另外 unknown_modes() 會把陌生的 mode 名字單獨吼出來。
       PRE/POST_RELEASE_MODES 兩堆都必須跟著 tot_node 的 mode 賦值一起改。

    ⚠️⚠️ 找不到放行點時退回 0,但**一定要把標記帶出去**(2026-08-04 的 blocker)。
       整段 DIVE 都是 wait/runin/missed 代表這架從頭到尾沒被放行(逾時航次最
       常見的形狀)。窗退回整段之後,d(t) 的最小值是**環上盤旋 / miss_pt 守點
       盤旋擺盪出來的局部最小**,
       不是攻擊航次的距離 —— 合成 CSV 實測會印出
         「UAV1 MISS 首航次d1=2100.00 t1=1.00 收窗 hysteresis」
       把繞圈的最近點當成首航次脫靶距離報出來,零警告,連 win[i] 都看不出來。
       可達路徑:逾時航次被 stot_rescore_all.py:88 掃進總表重算,而總表正是
       拿來排序、挑「最好的一趟」的依據。
       退回本身是對的(不要整架丟掉),錯的是**靜默** —— 呼叫端拿到標記後
       要把這架標成「不可信」而不是 MISS/HIT。
    """
    for k, p in enumerate(pts):
        # ⚠️ 納入式,不是排除式 —— 見上面的 blocker 說明。改成
        #    `not in PRE_RELEASE_MODES` 會讓 wait→missed 的機被判成已放行。
        if p[P_MODE] in POST_RELEASE_MODES:
            return k, START_RELEASE
    # 一筆 dive/impact/expended 都沒有(整段 wait/runin/missed,或 mode 欄
    # 改了名):退回原點,不要整架丟掉,但要出聲 —— 窗內的最小值來自繞圈
    # (環上等待,或 missed 之後在 miss_pt 守點盤旋),不是攻擊航次。
    return 0, START_NO_RELEASE


def first_pass_window(pts, T, ml, hyst=1000.0, escape_r=20000.0, agl_of=None):
    """切出「首航次」= 第一次接近紅心再遠離的那一段。

    回傳 (i0, i1, info)。pts[i0:i1+1] 含端點。info 有:
      d1/t1/idx1  窗內最近的**取樣點**(細到取樣間的內插由呼叫端用 seg_closest 做)
      closed_by   "touchdown" | "hysteresis" | "escape" | "end-of-track"
      start       "release" | "no-release-found" —— 窗起點是不是真的找到放行點。
                  "no-release-found" 表示整段都在放行前,窗內的 d1 來自環上
                  盤旋,**不可信**(見 window_start)。呼叫端必須據此降級判定。
      escaped     窗起點之後**整段**軌跡是否曾經 > escape_r(逃逸的判準)
      max_d       窗起點之後整段的最遠距離(m)

    `agl_of(p)` 回傳該取樣點離紅心地表的高度(m),沒有高度基準時傳 None。

    收窗條件(先到先收):
      ⓪ AGL ≤ 0(觸地/觸海)⇒ **無條件**收窗,連 hyst 都不看
      ① d 從目前最小值回升超過 hyst(遲滯,預設 1000 m)⇒ 首航次結束
      ② d > escape_r(預設 20 km)⇒ 直接判逃逸,提前結束

    ⚠️⚠️ ⓪ 是 2026-08-04 的 blocker 修正,不是保險絲。SITL 不會因為機體觸海就
       終結它:SW1 的 UAV3 於 t=1012.6 觸海掉到 −95.7 m,爬回 +331 m 之後在
       24.1 s 後**再穿一次**紅心(22.58 m);UAV5 同型(17.9 s 後 67.11 m)。
       兩趟之間的外衝峰值只有 555.5 / 637.0 m,遲滯 1000 m 永遠收不掉,於是
       工具回報的是第二趟。而且「把遲滯調小」不是解:調到 < 555 會撞上 UAV3
       放行時 923 m 的外衝(見 ① 的守衛),兩邊夾死。
       物理依據明確 —— 機體落水之後不可能再有合法的攻擊航次。
    ⚠️ 收窗的索引取「第一筆 AGL ≤ 0 的取樣」而不是它前一筆:觸地點的內插要
       用 (AGL>0, AGL≤0) 這一對相鄰取樣,少了後者觸地掃描就找不到穿越點。
    ⚠️ 沒有高度基準(agl_of 回 None)時這條規則整條失效,窗退回 ①②。
       那不是「沒觸地」,是**量不出來** —— 呼叫端的 touch_datum == "none"
       會把這件事說出來。

    ⚠️ ① 額外要求「已經真的接近過」(最小值曾低於窗起點的距離)才准收窗。
       放行瞬間機頭多半還沒轉正,d 會先往外衝一段再回頭:實測 SW1 的外衝量
       UAV3 923 m、UAV2 696 m、UAV5 640 m —— UAV3 距離 1000 m 遲滯只剩 77 m
       餘裕。少了這個條件,遲滯值稍微調小、或下一趟外衝再大一點,窗就會在
       出發點收掉,而且收得「看起來很合理」(印出一個 3000 m 的 d1)。
    """
    n = len(pts)
    i0, start_flag = window_start(pts)
    if n == 0:
        return 0, -1, {"d1": None, "t1": None, "idx1": None,
                       "closed_by": "empty", "start": start_flag,
                       "escaped": False, "max_d": None}
    d0 = horiz_d(pts[i0][P_LAT], pts[i0][P_LON], T, ml)
    best_d, best_k = d0, i0
    approached = False            # 最小值是否已經低於出發距離
    i1, closed_by = n - 1, "end-of-track"
    max_d = d0
    for k in range(i0, n):
        d = horiz_d(pts[k][P_LAT], pts[k][P_LON], T, ml)
        if d > max_d:
            max_d = d
        # ⚠️ 最小值先更新再判收窗:觸地那一筆本身可能就是最近點(俯衝角越陡
        #    越可能),先 break 會把它漏掉。d < best_d 時 ① 必然不成立,
        #    所以三條收窗條件仍然互斥。
        if d < best_d:
            best_d, best_k = d, k
            if d < d0:
                approached = True
        ag = agl_of(pts[k]) if agl_of is not None else None
        if ag is not None and ag <= 0.0:
            i1, closed_by = k, "touchdown"
            break
        if d > escape_r:
            i1, closed_by = k, "escape"
            break
        if approached and d > best_d + hyst:
            i1, closed_by = k, "hysteresis"
            break
    # 逃逸旗標看的是**窗之後整段**軌跡,不是只看窗內。SW1 的 UAV2/UAV6 首航次
    # 在 638/469 m 處就收窗了,飛到 207/412 km 外是後面幾小時的事 —— 那才是要
    # 抓的病灶(impact 沒有 miss 出口),所以掃到底。
    # ⚠️ 不要一碰到 escape_r 就 break:那樣 max_d 永遠只會印「20.0 km」,把
    #    412 km 的離譜程度壓成剛好過門檻,警告行就失去了它唯一的作用。
    for k in range(i1 + 1, n):
        d = horiz_d(pts[k][P_LAT], pts[k][P_LON], T, ml)
        if d > max_d:
            max_d = d
    return i0, i1, {"d1": best_d, "t1": pts[best_k][P_T], "idx1": best_k,
                    "closed_by": closed_by, "start": start_flag,
                    "escaped": max_d > escape_r, "max_d": max_d}


def _datum_summary(res, key):
    """把逐機的高度基準彙總成一個標籤;不一致時標 mixed:…(不隱藏)。"""
    vals = sorted({v[key] for v in res.values()})
    if not vals:
        return "none"
    return vals[0] if len(vals) == 1 else "mixed:" + "+".join(vals)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--log", default=None, help="tot_node log(讀 z_g)")
    ap.add_argument("--geom", default=None, help="*_geom.csv(讀目標座標)")
    ap.add_argument("--ground", type=float, default=None,
                    help="無 log 時的統一地表高度 fallback")
    ap.add_argument("--phase", default="DIVE", help="只算這個階段(預設 DIVE)")
    ap.add_argument("--alt-max", type=float, default=30.0,
                    help="命中高度門檻(m);沿用 _kpi_parse.py 的 ALT_MAX 預設 30")
    ap.add_argument("--miss-max", type=float, default=5.0,
                    help="觸地水平誤差門檻(m)。⚠️ 使用者 2026-07-28 放寬到 10;"
                         "預設仍留 5,重算歷史航次時才不會把過去的判定改寫")
    ap.add_argument("--speed-min", type=float, default=100.0,
                    help="俯衝地速門檻(m/s):六機**平均**觸地地速要達到這個值。"
                         "驗收第一優先(使用者 2026-07-28 定調)")
    ap.add_argument("--no-window", dest="window", action="store_false",
                    default=True,
                    help="關掉首航次窗,回到舊的『全程全域最近點』評分。"
                         "⚠️ 舊路徑會被逃逸鏈污染(SW1 散布 11950 s),只供對照 —— "
                         "本模式**不做首航次判定,因此永遠印 ❌**(未判定 ≠ 通過)")
    ap.add_argument("--miss-r", type=float, default=300.0,
                    help="首航次命中半徑(m):d1 <= 這個值判 HIT。預設 300 = "
                         "tot_node 的 switch_r。⚠️ 不要為了讓脫靶機『算命中』"
                         "而放寬它:重放證明放到 800 會在 1000-1500 m 高空判命中")
    ap.add_argument("--window-hyst", type=float, default=1000.0,
                    help="首航次收窗遲滯(m):d 從最近點回升超過這個值即結束")
    ap.add_argument("--escape-r", type=float, default=20000.0,
                    help="逃逸半徑(m):俯衝後曾離目標超過這個距離標 ESCAPE")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)

    geom = a.geom or a.csv.replace(".csv", "_geom.csv")
    T = None
    if os.path.exists(geom):
        with open(geom) as f:
            for r in csv.DictReader(f):
                if r["kind"] == "target":
                    T = (float(r["lat"]), float(r["lon"]))
    if T is None:
        print("找不到目標座標(%s)" % geom, file=sys.stderr)
        return 2

    zg = load_zg(a.log)
    tracks = {}
    has_agl = False               # CSV 尾端有沒有 tot_node 直接寫的 agl 欄
    with open(a.csv) as f:
        rd = csv.DictReader(f)
        # ⚠️ 按**欄名**判斷,不要按欄位數 —— 另一支節點正在加 agl 欄,新舊兩種
        #    CSV 都要吃得下。有 agl 就直接用(基準跟著 tot_node 走),沒有就照舊
        #    用 alt - z_g 換算。
        has_agl = bool(rd.fieldnames) and "agl" in rd.fieldnames
        for r in rd:
            if r["phase"] != a.phase:
                continue
            i = int(r["sysid"])
            agl = None
            if has_agl:
                v = r.get("agl")
                if v not in (None, ""):
                    try:
                        agl = float(v)
                    except ValueError:
                        agl = None
            tracks.setdefault(i, []).append(
                (float(r["t"]), float(r["lat"]), float(r["lon"]),
                 float(r["alt"]), float(r["gs"] or 0), r["mode"], agl))

    ml = mlon(T[0])
    res = {}
    win = {}
    # [(sysid, 範圍內有 agl 的筆數, 範圍內總筆數, 實際退到的基準)] —— 半有半無要出聲
    partial_agl = []
    # [(sysid, 評分範圍**外**缺的筆數, 整段筆數)] —— 基準沒變,但缺值仍要出聲
    outside_agl = []
    unk_modes = {}            # {sysid: [不認得的 mode 值]} —— 節點端改名要出聲
    for i in sorted(tracks):
        full = tracks[i]
        # ⚠️ 用**整段**(full)而不是窗內:mode 詞彙脫節會同時打壞放行點判準
        #    與 gs 峰值的取樣集合(mode == "dive"),兩者都在窗切出來之前就
        #    被決定了。窗內掃描看不到「放行前那一段用了什麼名字」。
        unk = unknown_modes(full)
        if unk:
            unk_modes[i] = unk
        g = zg.get(i, a.ground)
        # ── 高度基準:**一架一個、一次決定,四個使用點共用同一個** ────────
        # 使用點有四個,少一個共用就會出現「兩套基準,而且標錯」:
        #   ① 收窗規則 ⓪(AGL ≤ 0 無條件收窗)   ② 觸地掃描(穿越 AGL=0)
        #   ③ 殘餘高度 alt_agl_m                 ④ 報表標籤 alt_datum/touch_datum
        #
        # ⚠️⚠️ 2026-08-05 blocker:上一版有**三個**決定點 —— ① 用整段 full 判、
        #    ②③④ 用窗內 pts 判、partial-agl 警告也只看 pts。於是 agl 欄只要在
        #    **窗外**缺值(kill -KILL 截掉末列、新舊 CSV 併檔、float() 解析失敗
        #    走 except ValueError),① 就退回 alt−z_g,而 ②③④ 仍算 csv:agl:
        #    合成 CSV 實測(放行前 5 列 agl 空、窗內完整、--ground 0):
        #      收窗用 alt−0 在 t=10 判「觸地」收窗,agl 欄要到 t=22 才歸零
        #      ⇒ 窗提早 10 個取樣收掉 ⇒ 一架**正對紅心通過**的機被判
        #        d1=1000.00 m / MISS / impact_miss_m=None / n_touch=0,
        #        報表標 csv:agl,warnings 裡連一句基準相關的話都沒有。
        #    方向不保證保守(這一次是把 HIT 打成 MISS,反過來也做得到)。
        #
        # 決定範圍 = **這一架實際會被評分的所有取樣**:
        #   窗開著   full[i_rel:] —— i_rel 是放行點,只看 mode 反推(window_start
        #            不需要任何 AGL)⇒ 不循環。窗必定是 full[i_rel:] 的前綴切片,
        #            所以這個範圍**涵蓋**所有使用點。
        #            (涵蓋而非等於:窗的**收尾**點要有基準才決定得出來,所以
        #             窗尾之後的缺值也會逼出退路 —— 那是保守方向,而且會出聲。)
        #   --no-window  full —— 這時 pts 就是 full,範圍剛好相等。
        # ⚠️ 判準仍是「範圍內每一筆都有 agl」才算 csv:agl,否則整架退回 alt−z_g:
        #    在同一條軌跡上混兩套基準(這一筆用 agl、下一筆用 alt−z_g)比退回舊
        #    基準更危險 —— 觸地內插正好跨在兩套基準之間就會歸零在假的位置。
        # ⚠️ 兩者都沒有就回 None,收窗規則 ⓪ 整條失效(而不是靜默把 AGL 當 0),
        #    datum 標 "none",呼叫端會說「算不出來」而不是「沒落地」。
        # ⚠️ SW1 的 CSV 沒有 agl 欄(那是 2026-08-04 之後才加的),所以退路
        #    不是備案而是主要路徑:舊航次全靠 alt − z_g 才收得掉窗。
        i_rel, _rel_flag = window_start(full)
        scope = full[i_rel:] if a.window else full
        n_agl_scope = sum(1 for p in scope if p[P_AGL] is not None)
        n_agl_full = sum(1 for p in full if p[P_AGL] is not None)
        use_csv_agl = bool(scope) and n_agl_scope == len(scope)
        if use_csv_agl:
            datum = "csv:agl"
        elif g is not None:
            datum = "log:z_g" if i in zg else "arg:--ground"
        else:
            datum = "none"
        # ⚠️ 缺值一律出聲,兩個方向都要 —— 靜默退回正是上一版的病灶。
        #    範圍內缺 ⇒ 退回別的基準(退路是什麼也一起印,不要寫死 alt−z_g:
        #    沒有 z_g 時退無可退,那是 "none")。
        #    只有範圍外缺 ⇒ 基準沒變,但仍要講一句,免得下一個人以為 agl 欄是滿的。
        if has_agl and not use_csv_agl:
            partial_agl.append((i, n_agl_scope, len(scope), datum))
        elif has_agl and n_agl_full != len(full):
            outside_agl.append((i, len(full) - n_agl_full, len(full)))

        def _agl_of(p, _use=use_csv_agl, _g=g):
            """該取樣點的 AGL(離紅心地表);沒有基準就回 None。

            ⚠️ ①②③ 全部走這一個取值器。要改基準就只有這裡一個地方可以改。
            """
            if _use:
                return p[P_AGL]
            return None if _g is None else p[P_ALT] - _g

        # 收窗規則 ⓪ 用的就是**同一個**取值器,不再另判一次。
        win_agl = _agl_of if datum != "none" else None
        # ── 首航次窗 ────────────────────────────────────────────────────
        # 窗一開,底下所有量(最近通過點、觸地點、峰值地速)都只在窗內找。
        # --no-window 時窗 = 整段,走的就是原本那條路徑,一個字都沒改。
        if a.window:
            i0, i1, wi = first_pass_window(full, T, ml, a.window_hyst,
                                           a.escape_r, agl_of=win_agl)
            pts = full[i0:i1 + 1]
        else:
            i0, i1 = 0, len(full) - 1
            wi = {"d1": None, "t1": None, "idx1": None, "closed_by": "off",
                  "start": "off", "escaped": None, "max_d": None}
            pts = full
        win[i] = {
            "enabled": bool(a.window),
            "t_start": round(full[i0][P_T], 2) if full else None,
            "t_end": round(full[i1][P_T], 2) if i1 >= 0 and full else None,
            "n_samples": len(pts),
            "n_samples_full": len(full),
            "closed_by": wi["closed_by"],
            # ⚠️ 窗起點是不是真的找到放行點。"no-release-found" ⇒ 這一列的
            #    d1/t1 來自環上盤旋,不是攻擊航次(見 window_start)。
            "start": wi["start"],
            "max_d_m": round(wi["max_d"], 1) if wi["max_d"] is not None else None,
            "escaped": wi["escaped"],
        }
        best = None
        for k in range(len(pts) - 1):
            t0, la0, lo0, z0, g0, _m0, ag0 = pts[k]
            t1, la1, lo1, z1, g1, _m1, ag1 = pts[k + 1]
            ax, ay = (la0 - T[0]) * EARTH, (lo0 - T[1]) * ml
            bx, by = (la1 - T[0]) * EARTH, (lo1 - T[1]) * ml
            # 取樣間跳太遠(> 400 m)代表中間斷訊,直線假設不成立,跳過
            if math.hypot(bx - ax, by - ay) > 400.0:
                continue
            dmin, tt = seg_closest(0.0, 0.0, ax, ay, bx, by)
            if best is None or dmin < best[0]:
                z = z0 + tt * (z1 - z0)
                tm = t0 + tt * (t1 - t0)
                ag = (ag0 + tt * (ag1 - ag0)
                      if (ag0 is not None and ag1 is not None) else None)
                best = (dmin, z, tm, g0 + tt * (g1 - g0), ag)
        if best is None:
            # 窗太短(< 2 個取樣點)或整段都被 400 m 跳點濾掉:退回窗內最近的
            # 那個取樣點,不要靜默把這架丟掉。
            if not pts:
                continue
            k = min(range(len(pts)),
                    key=lambda j: horiz_d(pts[j][P_LAT], pts[j][P_LON], T, ml))
            best = (horiz_d(pts[k][P_LAT], pts[k][P_LON], T, ml),
                    pts[k][P_ALT], pts[k][P_T], pts[k][P_GS], pts[k][P_AGL])
        # ── 真正的命中點:軌跡與地面的交點 ────────────────────────────────
        # ⚠️⚠️ R1 blocker(2026-08-04):上一版 agl 欄只餵給 alt_agl_m,觸地掃描
        #    仍寫死 `pts[k][3] - g`(alt − log 的 z_g)。後果有兩個,都會給出
        #    錯誤結論:
        #      A. log 不在了(輪替 / 只留 CSV / 跨機分析;stot_rescore_all.py
        #         是 `if os.path.exists(logp)` 才加 --log)⇒ g=None ⇒ 六機
        #         touch 全 None ⇒ 印「六機都沒有真的落地」、判「未達標」。
        #         資料明明就在 agl 欄裡,一趟完美航次被判成全員沒落地。
        #      B. 陸地目標時 tot_node 的 z_g 每收到一次 TERRAIN_REPORT 就更新,
        #         但只有第一次進 log ⇒ agl 欄用時變 z_g、touch 用初始 z_g,
        #         而報表卻宣稱 alt_datum = "csv:agl"。基準混用且靜默。
        #    ⇒ 掃描一律走上面那個 _agl_of(datum 的唯一使用介面),這裡**不准**
        #      再出現第二次基準判斷。2026-08-05 的 blocker 就是「第二次判斷」
        #      搬到收窗層重演的。
        # ⚠️ 「最近通過距離」會美化結果:機體若在目標前 200 m 觸地再貼地滑進紅心,
        #    最近距離看起來很小,但那不是命中紅心 —— 它是在 200 m 外墜地的。
        #    物理上正確的命中點 = 軌跡穿過 AGL=0 的那一點。沒穿過就是從空中飛過。
        touch = None
        touch_datum = datum          # 觸地掃描**實際**用的基準(獨立欄位,
        #                              不與 alt_agl_m 的 alt_datum 共用)
        if datum != "none":
            # ⚠️ 要取「離紅心最近的那個」穿越點,不是第一個。AGL 是相對**紅心地表**
            #    算的,所以機體在遠處飛過較高地形時可能先出現一次 AGL<=0 的穿越,
            #    那不是命中。實測 B21 的 UAV2:最近通過 0.72 m、最低 AGL 0.42 m
            #    (其實很準),卻因為取到 2554 m 外那個早期穿越,被判成誤差 2554 m、
            #    散布 37.9 s —— 一趟好的航次被誤判成失敗。
            for k in range(len(pts) - 1):
                a0 = _agl_of(pts[k])
                a1 = _agl_of(pts[k + 1])
                if a0 is None or a1 is None:
                    continue
                if a0 > 0.0 >= a1:
                    f = a0 / (a0 - a1) if (a0 - a1) != 0 else 0.0
                    la = pts[k][1] + f * (pts[k + 1][1] - pts[k][1])
                    lo = pts[k][2] + f * (pts[k + 1][2] - pts[k][2])
                    tm = pts[k][0] + f * (pts[k + 1][0] - pts[k][0])
                    dd = math.hypot((la - T[0]) * EARTH, (lo - T[1]) * ml)
                    # ⚠️ 速度也要在**觸地那一點**內插,不能沿用「最近通過點」的 gs。
                    #    淺俯衝時兩者不是同一點:飛機幾乎平飛過紅心、再往前觸地,
                    #    最近通過點的速度會比真正砸到地面時高。驗收第一優先是
                    #    「打中紅心時多快」,所以要用這個。
                    gx = pts[k][4] + f * (pts[k + 1][4] - pts[k][4])
                    # ⭐⭐ 2026-08-14(使用者定調):驗收改用**沿航跡總速**,
                    #    不再用地速。三個理由,前兩個是文獻通例、第三個是我們自己的量測:
                    #    ① 殺傷/侵徹取決於總動能 ½mV²;控制權限取決於動壓 ½ρV²。
                    #       兩者都是總速,不是水平分量。
                    #    ② 地速 = V·cosγ 在陡俯衝時**退化**:γ→90° 時 cosγ→0,
                    #       地速趨近零而與實際速度無關。以地速下約束等於在陡角時
                    #       要求無限大的總速。
                    #    ③ 實測:RPT1 兩架觸地地速 182.4/182.7,精準命中設計推力的
                    #       理論地速天花板 182.2–182.7 ⇒ 地速 204 是**物理上界之外**,
                    #       而同一趟的總速有 +11% 的餘裕。
                    #    ⚠️ 垂直率由相鄰兩筆的高度差推(CSV 沒有直接的 vz 欄);
                    #       用**觸地區間**那一對,不是整段平均。
                    _dt = pts[k + 1][0] - pts[k][0]
                    _vz = ((pts[k][3] - pts[k + 1][3]) / _dt) if _dt > 1e-6 else 0.0
                    vx = math.hypot(gx, _vz)
                    if touch is None or dd < touch[0]:
                        touch = (dd, tm, gx, vx)
        # ── 俯衝地速峰值:取樣集合 = **只有 mode == "dive"** ─────────────────
        # ⚠️ 只排除 wait 不夠(2026-08-04 的 blocker)。同一個 DIVE phase 底下
        #    躺著四種 mode,其中三種都不是「俯衝中的地速」:
        #      wait     在環上繞圈等錯開放行(實測 F3 的 UAV2 105.8 m/s)
        #      runin    還在 30 km 外轉場,指令速度是 vmax(SW1 U5 156.4 m/s)
        #      expended 命中鎖存後**彈道自由墜落**(SW1 U5 204.1、U2 296.7 m/s)
        #    而同一架真正在 dive 的峰值只有 139.2 m/s。報表印 204.1 的時候,
        #    那個數字既不是導引也不是飛控做出來的,是重力 —— 名實不符,而且會
        #    讓「俯衝地速已經逼近 200」這種結論看起來有憑有據。
        #    (飛控的 AIRSPEED_MAX 是 150;超過它的取樣一律該先懷疑取樣集合。)
        gs_all = [p[P_GS] for p in pts if p[P_GS] > 0 and p[P_MODE] == "dive"]
        # ── 命中高度基準 ────────────────────────────────────────────────
        # ⚠️ 高度有兩套基準,混用會系統性差一個 z_g(SW1 是 32 m):
        #      CSV 的 alt = 相對 HOME;要的是相對**紅心地表** = alt - z_g。
        #    CSV 若已經有 agl 欄(tot_node 直接寫),就用它,基準跟著節點走。
        # ⚠️ datum 已在上面**一次決定**,這裡只負責取值 —— 不可以再自己判一次,
        #    那正是觸地與最近點分岔成兩套基準的原因。
        if use_csv_agl:
            agl_m = best[4]
        elif g is not None:
            agl_m = best[1] - g
        else:
            agl_m = None
        res[i] = {
            # ⚠️ 欄名保留(下游 _kpi_parse / stot_repeat 還在讀),但取樣集合
            #    另開一欄講明白 —— 「峰值」兩個字本身講不出它是從哪一堆取樣
            #    挑出來的,而那正是 204.1 m/s 混進來的縫。
            "gs_peak_ms": round(max(gs_all), 1) if gs_all else None,
            "gs_peak_from": 'mode=="dive"',
            "gs_peak_n": len(gs_all),
            "miss_m": round(best[0], 2),
            # ⚠️ 窗開著時這是**首航次最近點**的高度,不再是全程全域最小 ——
            #    全域最小會取到逃逸機幾小時後的隨機重穿(SW1 的 UAV6:首航次
            #    殘餘 1214 m,全域卻是 0.6 m,看起來像完美命中)。
            "alt_agl_m": round(agl_m, 2) if agl_m is not None else None,
            "alt_datum": datum,
            # ⚠️ 觸地判定用的基準單獨記一欄。兩者現在**一定**相同(同一個 datum),
            #    但要看得出來是「同一個」,而不是靠讀者相信它們同一個 ——
            #    上一版就是共用一個標籤、實際跑兩套基準。
            "touch_datum": touch_datum,
            "alt_rel_m": round(best[1], 2),
            "t_s": round(best[2], 2),
            "gs_ms": round(best[3], 1),
            "impact_miss_m": round(touch[0], 2) if touch else None,
            "impact_t_s": round(touch[1], 2) if touch else None,
            "impact_gs_ms": round(touch[2], 1) if touch else None,
            # ⭐ 沿航跡總速(= hypot(地速, 下沉率))。2026-08-14 起這是驗收量。
            "impact_v_ms": round(touch[3], 1) if touch else None,
            "n_samples": len(pts),
            "z_g": g,
        }
        # ── 首航次判定 ──────────────────────────────────────────────────
        if a.window:
            d1, t1 = best[0], best[2]      # 用線段內插的最近點(比取樣點準)
            escaped = bool(win[i]["escaped"])
            # ⚠️⚠️ 窗起點沒找到放行點時,d1/t1 量的是**環上盤旋**的局部最小,
            #    不是攻擊航次 —— 兩個方向都會騙人:小的被印成 HIT(幽靈命中)、
            #    大的被印成 MISS(其實是「還沒出發」)。所以不判 HIT/MISS/ESCAPE,
            #    標成 UNKNOWN,並且不進 hits ⇒ all_hit 必然 False ⇒ 下游
            #    (stot_rescore_all / stot_repeat.sh)的 first_pass_ok 也跟著 False,
            #    這一趟不會被排進「最好的一趟」。
            if win[i]["start"] == START_NO_RELEASE:
                status = "UNKNOWN"
            elif d1 <= a.miss_r:
                status = "HIT"
            else:
                status = "ESCAPE" if escaped else "MISS"
            res[i].update({"status": status, "d1_m": round(d1, 2),
                           "t1_s": round(t1, 2), "escaped": escaped,
                           "window_start": win[i]["start"]})
        else:
            res[i].update({"status": None, "d1_m": None, "t1_s": None,
                           "escaped": None, "window_start": "off"})

    if not res:
        print("沒有 %s 階段的資料" % a.phase, file=sys.stderr)
        return 2

    # ── 首航次彙總:散布**只由 HIT 的機**算 ──────────────────────────────
    # ⚠️ 把 MISS/ESCAPE 混進散布就是 SW1 那個 11950 s 的來源。它們不是「散布很
    #    大」,它們是**沒打中** —— 兩件事,不能用同一個數字表示。
    hits = [i for i in sorted(res) if res[i].get("status") == "HIT"]
    missed = [i for i in sorted(res) if res[i].get("status") == "MISS"]
    escapes = [i for i in sorted(res) if res[i].get("status") == "ESCAPE"]
    unknown = [i for i in sorted(res) if res[i].get("status") == "UNKNOWN"]
    hit_t = [res[i]["t1_s"] for i in hits]
    hit_d = [res[i]["d1_m"] for i in hits]
    hit_alt = [abs(res[i]["alt_agl_m"]) for i in hits
               if res[i]["alt_agl_m"] is not None]
    hit_touch_t = [res[i]["impact_t_s"] for i in hits
                   if res[i]["impact_t_s"] is not None]
    # ── HIT-only 的觸地量(2026-08-04 blocker)────────────────────────────
    # ⚠️⚠️ ①(地速)與 ③(水平誤差)以前只有「窗內全員」一種版本,而 MISS/ESCAPE
    #    的觸地是它衝過紅心之後在幾百公尺外砸海 —— 那一下的速度是重力給的,
    #    誤差是脫靶量。SW1 實測:全員平均 151.2 m/s(門檻 100 ⇒ 看起來達標),
    #    但裡面躺著本工具自己判成 ESCAPE、觸地誤差 1060.85 m、飛出 412.4 km 的
    #    UAV6,它的 impact_gs = 279.9。只算 4 架 HIT 是 119.0 m/s —— 被墊高
    #    32.2 m/s(27%),而且方向永遠是**樂觀**的:脫靶機的自由墜落比導引俯衝快。
    #    ②(散布)早就補了 hit_touch_spread_s 並寫明「拿它去湊散布等於用失敗
    #    掩蓋失敗」,①③ 同一個道理,不能只修一項。
    hit_touch_gs = [res[i]["impact_gs_ms"] for i in hits
                    if res[i]["impact_gs_ms"] is not None]
    hit_touch_miss = [res[i]["impact_miss_m"] for i in hits
                      if res[i]["impact_miss_m"] is not None]
    verdict = None
    if a.window:
        verdict = {
            "n": len(res),
            "n_hit": len(hits), "n_miss": len(missed), "n_escape": len(escapes),
            "n_unknown": len(unknown),
            "hit_ids": hits, "miss_ids": missed, "escape_ids": escapes,
            # 窗起點沒找到放行點的機:d1/t1 來自環上盤旋,判定不可信
            "unknown_ids": unknown,
            "miss_r_m": a.miss_r,
            # 只有 HIT 進統計;不足 2 架時散布沒有意義,標 None 而不是 0.0
            "hit_spread_s": (round(max(hit_t) - min(hit_t), 2)
                             if len(hit_t) > 1 else None),
            "hit_max_d1_m": round(max(hit_d), 2) if hit_d else None,
            "hit_max_abs_alt_m": round(max(hit_alt), 2) if hit_alt else None,
            # 觸地基準的散布也給一份只算 HIT 的 —— summary 頂層那個
            # touch_spread_s 是窗內全員(含 MISS/ESCAPE)算的舊欄位,兩者
            # 不同時要看得出來是哪一個。
            "hit_touch_spread_s": (round(max(hit_touch_t) - min(hit_touch_t), 2)
                                   if len(hit_touch_t) > 1 else None),
            "n_hit_touch": len(hit_touch_t),
            # ①③ 的 HIT-only 版本。頂層 summary 的 touch_gs_mean_ms /
            # touch_max_miss_m 是窗內**全員**(含 MISS/ESCAPE)算的舊欄位,
            # 兩者不同時要看得出來是哪一個 —— SW1:151.2 vs 119.0。
            "hit_touch_gs_mean_ms": (round(sum(hit_touch_gs) / len(hit_touch_gs), 1)
                                     if hit_touch_gs else None),
            "hit_touch_gs_min_ms": (round(min(hit_touch_gs), 1)
                                    if hit_touch_gs else None),
            "hit_touch_gs_max_ms": (round(max(hit_touch_gs), 1)
                                    if hit_touch_gs else None),
            "hit_touch_max_miss_m": (round(max(hit_touch_miss), 2)
                                     if hit_touch_miss else None),
            "all_hit": len(hits) == len(res),
        }
    warns = []
    # ── 高度基準的健康度:沒有基準 ≠ 沒落地 ─────────────────────────────
    # ⚠️⚠️ 這一條是 R1 blocker 的失效情境 A:沒有 log、沒有 --ground、CSV 也沒有
    #    agl 欄時,觸地全部算不出來,而報表底下那句「六機都沒有真的落地(全部
    #    從空中飛過)」在字面上把「量不出來」講成「飛過去了」。必須分得開。
    no_datum = [i for i in sorted(res) if res[i]["touch_datum"] == "none"]
    if no_datum:
        warns.append("⚠️⚠️ UAV%s 沒有任何高度基準(--log 沒有 z_g、沒給 --ground、"
                     "CSV 也沒有 agl 欄)⇒ 觸地判定**算不出來**,不是「沒落地」。"
                     "補 --log 或 --ground,或改用有 agl 欄的 CSV"
                     % "、".join(str(i) for i in no_datum))
    # ── agl 欄缺值:退回別的基準,而且**一定要出聲** ────────────────────────
    # ⚠️⚠️ 2026-08-05 blocker:上一版這條警告只看窗內的筆數,而收窗用的基準是
    #    用整段判的 ⇒ 缺值全在窗外時,收窗已經退回 alt−z_g、報表卻標 csv:agl、
    #    警告一句都沒有。現在基準的決定範圍與警告的統計範圍是**同一個**。
    for i, n_have, n_all, dtm in partial_agl:
        warns.append("⚠️ UAV%d 的 agl 欄只有 %d/%d 筆有值(評分範圍:放行點之後)"
                     "⇒ 整架退回 %s(收窗、觸地、殘餘高度共用這一個;"
                     "不在同一條軌跡上混兩套基準)"
                     % (i, n_have, n_all,
                        "**退無可退**:沒有 z_g 也沒有 --ground,觸地判定算不出來"
                        if dtm == "none" else "%s 基準" % dtm))
    # 缺值全在評分範圍**外**(放行前的列):基準沒退,但缺值本身仍要講一句 ——
    # 「哪些筆沒有值」是判斷 CSV 有沒有被截斷 / 被併檔的唯一線索。
    for i, n_missing, n_all in outside_agl:
        warns.append("⚠️ UAV%d 的 agl 欄有 %d/%d 筆缺值,但全部落在評分範圍外"
                     "(放行前)⇒ 基準仍是 csv:agl。CSV 可能被截斷或新舊併檔"
                     % (i, n_missing, n_all))
    # ── 窗起點找不到放行點:整段 DIVE 都還在 wait/runin ────────────────────
    # ⚠️⚠️ 2026-08-04 blocker:以前這裡是靜默的 —— window_start 找不到放行點就
    #    退回索引 0,零警告、win[i] 也沒有欄位標示,於是「還沒出發就結束的航次」
    #    會被印成一個看起來很正常的 MISS(合成 CSV 實測:UAV1 MISS
    #    首航次d1=2100.00 t1=1.00 收窗 hysteresis),而那個 2100 m 是環上盤旋
    #    的局部最小。d1 若剛好落進 miss_r 還會被印成 HIT —— 幽靈命中。
    #    可達路徑:逾時航次被 stot_rescore_all.py:88 掃進總表重算,而總表是
    #    拿來排序、挑「最好的一趟」的依據。
    if unknown:
        warns.append("⚠️⚠️ UAV%s 整段 %s 一筆 %s 都沒有(mode 全是 %s)⇒ 從來沒有"
                     "被放行,窗只能退回整段。這些機的 d1/t1 量的是**盤旋的局部"
                     "最小**(環上等待,或判 MISS 之後在 miss_pt 守點繞圈),"
                     "不是攻擊航次 —— 判定標成 UNKNOWN(不可信),不是 MISS。"
                     "要嘛這一趟逾時/放行閘門沒開,要嘛 tot_node 的 mode 取值"
                     "改了名(對照 POST_RELEASE_MODES)"
                     % ("、".join(str(i) for i in unknown), a.phase,
                        "/".join(POST_RELEASE_MODES),
                        "/".join(PRE_RELEASE_MODES)))
    # ── mode 詞彙脫節:節點端改名或新增狀態 ────────────────────────────────
    # ⚠️⚠️ 放行點與 gs 峰值都是靠 mode 字串反推的代理量。認不得的 mode 只會讓
    #    判準退到保守側(UNKNOWN),但那個 UNKNOWN 讀起來像「這一趟沒放行」,
    #    而真正的原因是「工具跟節點脫節」—— 兩者的處置完全不同,要分得開。
    if unk_modes:
        warns.append("⚠️⚠️ CSV 出現本工具不認得的 mode:%s ⇒ tot_node 的 mode 取值"
                     "已經改名或新增狀態,而放行點判準(POST_RELEASE_MODES=%s)"
                     "與 gs 峰值取樣集合(mode==\"dive\")都是靠這些字串反推的。"
                     "請同步 stot_score.py 的 PRE/POST_RELEASE_MODES"
                     % ("；".join("UAV%d %s" % (i, "/".join(m))
                                  for i, m in sorted(unk_modes.items())),
                        "/".join(POST_RELEASE_MODES)))
    if a.window and (missed or escapes):
        warns.append("⚠️⚠️ 首航次未命中 %d/%d 架:%s —— 散布只由 %d 架 HIT 算,"
                     "不是全隊指標"
                     % (len(missed) + len(escapes), len(res),
                        "、".join("UAV%d(%s d1=%.1fm 殘餘%s)"
                                  % (i, res[i]["status"], res[i]["d1_m"],
                                     "n/a" if res[i]["alt_agl_m"] is None
                                     else "%.0fm" % res[i]["alt_agl_m"])
                                  for i in missed + escapes),
                        len(hits)))
    for i in escapes:
        # ⚠️ 措辭不能寫死成「impact 集合沒有 miss 出口」—— 出口在 2026-08-04 之後
        #    已經補上了,對新航次那句話是錯的。這裡只陳述**觀測到的事實**
        #    (飛到幾百公里外),原因分兩種列出來讓人自己去 log 對:
        #    舊航次 = 沒有出口;新航次 = 出口沒生效(門檻/放行閘門看 trial log 的
        #    「★ miss 出口生效值」那一行)。
        warns.append("⚠️⚠️ UAV%d 俯衝後飛到 %.1f km 外 —— 脫靶機沒被鎖存就一直"
                     "滿速追。舊航次(無 miss 出口)本來就會這樣;新航次請對照 "
                     "trial log 的「★ miss 出口生效值」看出口為什麼沒收掉它"
                     % (i, win[i]["max_d_m"] / 1000.0))
    for i in hits:
        if res[i].get("escaped"):
            warns.append("⚠️ UAV%d 首航次 d1=%.1f m 判 HIT,但之後仍飛到 %.1f km 外"
                         " —— 命中沒有鎖存"
                         % (i, res[i]["d1_m"], win[i]["max_d_m"] / 1000.0))
    if a.window and len(hits) < 2:
        warns.append("⚠️ 只有 %d 架 HIT,散布沒有統計意義" % len(hits))

    misses = [v["miss_m"] for v in res.values()]
    times = [v["t_s"] for v in res.values()]
    alts = [abs(v["alt_agl_m"]) for v in res.values()
            if v["alt_agl_m"] is not None]
    # ── 觸地基準的彙總(驗收就是看這一組)────────────────────────────────
    tv = [v for v in res.values() if v["impact_miss_m"] is not None]
    t_miss = [v["impact_miss_m"] for v in tv]
    t_time = [v["impact_t_s"] for v in tv]
    t_gs = [v["impact_gs_ms"] for v in tv if v["impact_gs_ms"] is not None]
    t_v = [v["impact_v_ms"] for v in tv if v.get("impact_v_ms") is not None]
    summary = {
        "n": len(res),
        "n_touch": len(tv),
        "max_miss_m": round(max(misses), 2),
        "spread_s": round(max(times) - min(times), 2),
        "max_abs_alt_m": round(max(alts), 2) if alts else None,
        # 觸地基準
        "touch_max_miss_m": round(max(t_miss), 2) if t_miss else None,
        "touch_spread_s": round(max(t_time) - min(t_time), 2) if len(t_time) > 1 else 0.0,
        # ⭐ 驗收第一優先(使用者 2026-07-28 定調):六機**平均**觸地地速。
        #    選平均而不是最慢那架,是使用者明確的取捨。最慢/最快照報,不要拿掉。
        "touch_gs_mean_ms": round(sum(t_gs) / len(t_gs), 1) if t_gs else None,
        "touch_gs_min_ms": round(min(t_gs), 1) if t_gs else None,
        "touch_gs_max_ms": round(max(t_gs), 1) if t_gs else None,
        # ⭐ 總速(驗收量)。地速三欄保留供對照 —— 兩者差一個 cosγ,
        #    事後判讀時同時看得到才不會又搞混量綱。
        "touch_v_mean_ms": round(sum(t_v) / len(t_v), 1) if t_v else None,
        "touch_v_min_ms": round(min(t_v), 1) if t_v else None,
        "touch_v_max_ms": round(max(t_v), 1) if t_v else None,
        # ── 首航次窗(2026-08-04 加)────────────────────────────────────
        "verdict": verdict,
        "window": {
            "enabled": bool(a.window),
            "hysteresis_m": a.window_hyst,
            "escape_r_m": a.escape_r,
            "miss_r_m": a.miss_r,
            # ⚠️ 找不到放行點時窗退回整段,per_uav[i]["start"] 會標
            #    "no-release-found",該機判定降級成 UNKNOWN —— 不是靜默退回。
            # ⚠️ 納入式:看的是「有沒有出現 dive/impact/expended」,不是「不是
            #    wait/runin」。missed 兩種列序都會出現(從未放行、放行後脫靶),
            #    排除式會把前者當成放行 ⇒ 幽靈命中(見 window_start)。
            "start_rule": "第一筆 mode ∈ {%s} 的 %s 取樣(= 放行);"
                          "整段都是 {%s} 則標 %s 並把該機判定降級成 UNKNOWN"
                          % ("/".join(POST_RELEASE_MODES), a.phase,
                             "/".join(PRE_RELEASE_MODES), START_NO_RELEASE),
            # ⚠️ 收窗規則要印出來:同一個 d1 在「遲滯收窗」與「觸地收窗」兩種
            #    規則下可以差 3 倍(SW1 U5:217.90 vs 67.11),不寫清楚是哪一種
            #    收的,下一個人沒辦法判斷手上的數字能不能跟別的航次比。
            "close_rules": "AGL<=0(觸地)> escape_r > 遲滯",
            # ⚠️ 從**逐機實際用到的**基準彙總,不是從「CSV 有沒有 agl 欄」猜。
            #    猜出來的標籤正是 R1 blocker 的核心:報表寫 csv:agl,觸地卻在
            #    跑 log:z_g。不同機不同基準時標成 mixed:…,不准只印好看的那個。
            "alt_datum": _datum_summary(res, "alt_datum"),
            "touch_datum": _datum_summary(res, "touch_datum"),
            # ⚠️ 基準的決定**範圍**要跟著印:2026-08-05 的 blocker 就是收窗用
            #    整段判、報表用窗內判 ⇒ 缺值全在窗外時兩者分岔而且標錯。
            "datum_rule": "放行點之後(--no-window 時為整段)每一筆都有 agl 才算 "
                          "csv:agl,否則整架退回 alt−z_g;收窗/觸地/殘餘高度"
                          "共用同一個,缺值一律出警告",
            "per_uav": win,
        },
        "warnings": warns,
        "per_uav": res,
    }
    if a.json:
        # ⚠️ 警告走 stderr,stdout 只留純 JSON(下游 jq 才不會噎到);但**一定要
        #    印**,不能因為是 --json 就靜默 —— 靜默失敗是這專案的頭號死因。
        for w in warns:
            print(w, file=sys.stderr)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0

    print("目標 %.6f/%.6f    階段=%s" % (T[0], T[1], a.phase))
    if a.window:
        print("首航次窗:AGL≤0(觸地)無條件收窗;否則遲滯 %.0f m、逃逸半徑 "
              "%.0f m。命中半徑 %.0f m(--no-window 可回到舊的全域最近點)"
              % (a.window_hyst, a.escape_r, a.miss_r))
        # ⚠️ 「起窗」欄是 2026-08-04 補的:窗起點找不到放行點時,以前 win[i] 連
        #    一個欄位都沒有標示,讀者無從得知這一列的 d1 是繞圈繞出來的。
        print("%-5s %7s %10s %10s %11s %14s %16s %10s %10s"
              % ("UAV", "判定", "首航次d1(m)", "首航次t1(s)",
                 "t1高度AGL(m)", "窗(s)", "起窗", "收窗", "最遠(km)"))
        for i in sorted(res):
            v, w = res[i], win[i]
            print("%-5d %7s %10.2f %10.2f %11s %14s %16s %10s %10.1f"
                  % (i, v["status"], v["d1_m"], v["t1_s"],
                     "n/a" if v["alt_agl_m"] is None
                     else "%.1f" % v["alt_agl_m"],
                     "%.1f→%.1f" % (w["t_start"], w["t_end"]),
                     w["start"],
                     w["closed_by"],
                     w["max_d_m"] / 1000.0))
        print("★ 首航次:%d/%d HIT(門檻 d1 ≤ %.0f m)%s  高度基準 %s"
              % (verdict["n_hit"], verdict["n"], a.miss_r,
                 "" if not unknown
                 else "  ⚠️⚠️ 另有 %d 架 UNKNOWN(從未放行,d1 不可信)"
                      % len(unknown),
                 summary["window"]["alt_datum"]))
        print("   HIT 散布 = %s   HIT 最大 d1 = %s   HIT 最大殘餘高度 = %s"
              % ("n/a(HIT < 2 架)" if verdict["hit_spread_s"] is None
                 else "%.2f s" % verdict["hit_spread_s"],
                 "n/a" if verdict["hit_max_d1_m"] is None
                 else "%.2f m" % verdict["hit_max_d1_m"],
                 "n/a" if verdict["hit_max_abs_alt_m"] is None
                 else "%.1f m" % verdict["hit_max_abs_alt_m"]))
        print("─" * 78)
    # ⚠️ 「gs觸地」以前印的是 v["gs_ms"](最近通過點的速度),名不副實。
    #    現在印真正在 AGL=0 那一點內插出來的 impact_gs_ms,最近通過點的速度
    #    另開一欄叫「gs最近」,兩個都留著才對得起前面幾輪的記錄。
    print("%-5s %8s %8s %8s %10s %8s | %11s %8s"
          % ("UAV", "gs峰值dive", "gs觸地", "gs最近", "最低AGL(m)", "最近(m)",
             "觸地誤差(m)", "觸地t(s)"))
    for i in sorted(res):
        v = res[i]
        print("%-5d %8s %8s %8.1f %10s %8.2f | %11s %8s"
              % (i, "n/a" if v["gs_peak_ms"] is None else "%.1f" % v["gs_peak_ms"],
                 "n/a" if v["impact_gs_ms"] is None else "%.1f" % v["impact_gs_ms"],
                 v["gs_ms"],
                 "n/a" if v["alt_agl_m"] is None else "%.2f" % v["alt_agl_m"],
                 v["miss_m"],
                 "未觸地" if v["impact_miss_m"] is None
                 else "%.2f" % v["impact_miss_m"],
                 "-" if v["impact_t_s"] is None else "%.2f" % v["impact_t_s"]))
    print("─" * 78)
    # ⚠️ 判定一律用**觸地基準**(軌跡穿過 AGL=0 的那一點)。
    #    以前這裡的判定用的是 max_miss_m/spread_s(最近通過點),而 stot_repeat.sh
    #    用的是觸地基準 —— 同一趟兩支工具會給出不同結論。統一成觸地基準,
    #    最近通過點的數字仍然照印,只是不再拿來判定。
    #    「最近通過」會美化結果:機體若在目標前 200 m 觸地再貼地滑進紅心,
    #    最近距離看起來很小,但它是在 200 m 外墜地的。
    if summary["n_touch"] == 0 and no_datum:
        # ⚠️ 「量不出來」和「沒落地」是兩件事,不可以印成同一句(見 warns)。
        print("★ 觸地基準:**算不出來** —— %d 架沒有高度基準(%s)"
              % (len(no_datum), summary["window"]["touch_datum"]))
    elif summary["n_touch"] == 0:
        print("★ 觸地基準:六機都沒有真的落地(全部從空中飛過)  基準 %s"
              % summary["window"]["touch_datum"])
    else:
        print("★ 觸地基準(%d/%d 架真的落地)  基準 %s"
              % (summary["n_touch"], summary["n"],
                 summary["window"]["touch_datum"]))
    print("   參考:最近通過點 誤差 max = %.2f m、時間散布 = %.2f s"
          % (summary["max_miss_m"], summary["spread_s"]))
    # ⚠️⚠️ 哨兵:以下到檔尾是「收尾摘要」——①②③ + 警告 + 判定,stot_trial.sh
    #    從這一行的下一行取到結尾。以前那裡是 tail -12 的固定行數預算,3 架
    #    ESCAPE 就會把 ① 砍掉(見 SUMMARY_SENTINEL 的說明)。**加行前不必再
    #    重算預算,但這一行不可以被移到 ① 底下**。
    print(SUMMARY_SENTINEL)
    print("─" * 78)
    # ⭐ 使用者 2026-07-28 定調的優先序:
    #      ① 俯衝地速(六機**平均**觸地地速 ≥ speed_min)
    #      ② 觸地時間散布 ≤ 3 s
    #      ③ 水平命中誤差 ≤ 5 m(明確列為其次)
    #    統計方式取平均而非最慢那架,是使用者明確的取捨;最慢/最快照報不隱藏。
    gsp = [v["gs_peak_ms"] for v in res.values() if v["gs_peak_ms"] is not None]
    # ── ①③ 的污染源:窗內全員平均含 MISS/ESCAPE/UNKNOWN ─────────────────────
    # ⚠️⚠️ 2026-08-04 blocker。脫靶機也有「觸地」—— 那是它衝過紅心之後在幾百
    #    公尺外砸海,速度由重力給、誤差就是脫靶量。把它算進 ①③ 的方向永遠是
    #    **樂觀**的:SW1 的 UAV6 被本工具自己判成 ESCAPE(觸地誤差 1060.85 m、
    #    飛出 412.4 km),impact_gs 卻是 279.9 m/s,把全員平均從 119.0 墊高到
    #    151.2(+27%)⇒ 越過門檻 100 ⇒ 讀者認定第一優先項目已達標。
    #    ②(散布)早就補了 hit_touch_spread_s 並寫明「拿它去湊散布等於用失敗
    #    掩蓋失敗」;①③ 是同一個病,這裡一起治:判定看 HIT-only,全員值降級成
    #    參考值並在同一行講明它含了哪幾架。
    degraded = bool(a.window and (missed or escapes or unknown))
    # 只列**實際出現**的類別 —— 印「含 2 架 MISS/ESCAPE/UNKNOWN」而其實一架
    # UNKNOWN 都沒有,讀者會去找那架不存在的機。
    bad_tag = "含 %d 架 %s" % (
        len(missed) + len(escapes) + len(unknown),
        "/".join(k for k, ids in (("MISS", missed), ("ESCAPE", escapes),
                                  ("UNKNOWN", unknown)) if ids))
    if summary["touch_gs_mean_ms"] is None:
        print("① 觸地地速 平均 = n/a（沒有任何一架落地）")
    elif degraded:
        hgm = verdict["hit_touch_gs_mean_ms"]
        print("① 觸地地速 平均(只算 %d 架 HIT)= %s   (門檻 %.0f)   最慢 %s / 最快 %s"
              % (verdict["n_hit_touch"],
                 "n/a(沒有 HIT 的機落地)" if hgm is None else "%.1f m/s" % hgm,
                 a.speed_min,
                 "n/a" if verdict["hit_touch_gs_min_ms"] is None
                 else "%.1f" % verdict["hit_touch_gs_min_ms"],
                 "n/a" if verdict["hit_touch_gs_max_ms"] is None
                 else "%.1f" % verdict["hit_touch_gs_max_ms"]))
        print("   ⚠️ 全員值 %.1f m/s(%s,最慢 %.1f / 最快 %.1f)—— 參考用,"
              "**不是達標依據**:脫靶機的「觸地」是衝過紅心後在幾百公尺外的"
              "自由墜落,比導引俯衝快"
              % (summary["touch_gs_mean_ms"], bad_tag,
                 summary["touch_gs_min_ms"], summary["touch_gs_max_ms"]))
    else:
        # ⭐⭐ 2026-08-14 起驗收量 = **沿航跡總速**(使用者定調),地速改列參考。
        #    ⚠️ 兩者差一個 cosγ:27° 是 11%、40° 是 23%。混用會系統性偏。
        _vm = summary.get("touch_v_mean_ms")
        if _vm is None:
            print("① 觸地總速 平均 = n/a(舊產物沒有總速欄,請重跑計分)")
        else:
            print("① 觸地**總速** 平均 = %.1f m/s   (門檻 %.0f)   最慢 %.1f / 最快 %.1f"
                  % (_vm, a.speed_min,
                     summary["touch_v_min_ms"], summary["touch_v_max_ms"]))
        print("   參考:觸地**地速** 平均 = %.1f(= 總速 × cosγ_終端)。"
              "⚠️ 地速在陡俯衝時退化(γ→90° 時 cosγ→0),不適合當速度約束;"
              "設計推力下地速的理論天花板只有 182 m/s。"
              % summary["touch_gs_mean_ms"])
    if gsp:
        print("   參考:**mode==dive 取樣**的峰值地速 %.1f – %.1f m/s"
              "(峰值出現在俯衝中段,通常高於觸地速度 10–25 m/s;"
              "wait/runin/expended 不列入 —— expended 是命中後的彈道墜落)"
              % (min(gsp), max(gsp)))
    print("② 觸地時間散布 = %s   (門檻 3.0)"
          % ("n/a" if summary["touch_max_miss_m"] is None
             else "%.2f s" % summary["touch_spread_s"]))
    # ⚠️ 上面那個散布是窗內**全員**算的,脫靶機也在裡面。脫靶機的「觸地」是它
    #    衝過紅心之後在幾百公尺外砸海,拿它去湊散布等於用失敗掩蓋失敗。
    if degraded:
        print("   ⚠️ 上行%s。只算 %d 架 HIT:觸地散布 = %s、"
              "首航次 t1 散布 = %s"
              % (bad_tag, verdict["n_hit"],
                 "n/a" if verdict["hit_touch_spread_s"] is None
                 else "%.2f s" % verdict["hit_touch_spread_s"],
                 "n/a" if verdict["hit_spread_s"] is None
                 else "%.2f s" % verdict["hit_spread_s"]))
    # ③ 同 ①:全員 max 會被脫靶機的「觸地」(= 它自己的脫靶量)主導。
    #    SW1 全員 max = 1060.85 m,那整個就是 ESCAPE 的 UAV6 砸海的位置;
    #    只算 4 架 HIT 是 436.34 m。方向與 ① 相反(這裡是悲觀),但同樣不是
    #    「這一趟打得多準」的答案 —— 兩個值都要印,而且要講清楚誰是判定依據。
    if degraded:
        hmm = verdict["hit_touch_max_miss_m"]
        print("③ 觸地水平誤差 max(只算 %d 架 HIT)= %s   (門檻 %.0f,優先序最低)"
              % (verdict["n_hit_touch"],
                 "n/a" if hmm is None else "%.2f m" % hmm, a.miss_max))
        print("   ⚠️ 全員值 %s(%s)—— 參考用,**不是達標依據**"
              % ("n/a" if summary["touch_max_miss_m"] is None
                 else "%.2f m" % summary["touch_max_miss_m"], bad_tag))
    else:
        print("③ 觸地水平誤差 = %s   (門檻 %.0f,優先序最低)"
              % ("n/a" if summary["touch_max_miss_m"] is None
                 else "%.2f m" % summary["touch_max_miss_m"], a.miss_max))
    if summary["max_abs_alt_m"] is not None:
        print("   全員最大命中高度 = %.2f m   (門檻 %.0f)"
              % (summary["max_abs_alt_m"], a.alt_max))
    # ⚠️ 判定必須含命中高度。先前漏掉這一項,結果水平誤差和散布達標時就印「通過」,
    #    而那一趟六機其實是從紅心上空 35-175 m 飛過 —— 全部要過才算過。
    # ⚠️⚠️ 2026-08-06:分母必須是**本趟的機數**,不是寫死的 6。
    #    寫死的後果不是誤判通過,而是**兩機航次永遠不可能過** —— 就算兩架都
    #    完美觸地(TWO1 實測 觸地誤差 1.95/4.04 m、殘餘高度 0.72/2.59 m),
    #    n_touch 也只會是 2,條件恆假,判定行還會印「六機到齊」誤導歸因。
    #    機數取自 summary["n"](計分器自己從 CSV 的 sysid 數出來的),
    #    不靠環境變數 —— 離線重算舊航次時沒有那個環境。
    _n_expect = int(summary.get("n") or 6)
    all_touch = summary["n_touch"] == _n_expect
    # ⭐⭐ 2026-08-14:判定改吃**總速**(使用者定調)。地速留在 summary 供對照。
    # ⚠️ 舊產物沒有 touch_v_mean_ms ⇒ 退回地速並在輸出標明,**不可以靜默退回**:
    #    靜默退回會讓新舊產物用不同基準判定,而報表長得一模一樣。
    speed_ok = (summary.get("touch_v_mean_ms") is not None
                and summary["touch_v_mean_ms"] >= a.speed_min)
    spread_ok = all_touch and summary["touch_spread_s"] <= 3.0
    miss_ok = (summary["touch_max_miss_m"] is not None
               and summary["touch_max_miss_m"] <= a.miss_max)
    # ⚠️⚠️ 有非 HIT 的機時,①③ 的判定改看 HIT-only 值。以前 speed_ok 吃的是被
    #    脫靶機墊高的全員平均 ⇒ SW1 的「俯衝地速」永遠不會出現在 fails 清單裡,
    #    而 ① 那一行照印「151.2(門檻 100)」—— 讀者結論是「第一優先已達標」。
    #    ⓘ 這個覆寫**不會**改變整趟的 ok:只要有一架不是 HIT,verdict["all_hit"]
    #      就是 False ⇒ first_pass_ok False ⇒ ok 早就 False。它只讓 fails 清單
    #      說實話。所以 stot_rescore_all.verdict() 與 stot_repeat.sh 內嵌的判定
    #      不必跟著改,三處對同一份 JSON 仍然給出同一個結論。
    if degraded:
        # ⚠️ HIT-only 覆寫目前仍以地速計(hit_touch_* 只有地速版)。
        #    這是已知的不一致:degraded 路徑的門檻語意與主路徑不同,
        #    要改必須連同 hit_touch_* 一起加總速版,屬於另一個步驟。
        _hg = verdict["hit_touch_gs_mean_ms"]
        _hm = verdict["hit_touch_max_miss_m"]
        speed_ok = _hg is not None and _hg >= a.speed_min
        miss_ok = _hm is not None and _hm <= a.miss_max
    alt_ok = (summary["max_abs_alt_m"] is not None
              and summary["max_abs_alt_m"] <= a.alt_max)
    # ⚠️ 首航次全員命中是**前提**,不是加分項。少了這條,一趟「兩架脫靶飛到
    #    200 km 外、幾小時後隨機重穿紅心」的航次會在觸地基準上看起來很正常。
    # ⚠️⚠️ 2026-08-05 blocker C:舊寫法 `(not a.window) or verdict["all_hit"]`
    #    在 --no-window 之下**恆 True** —— 一個「刻意不做首航次判定」的診斷模式,
    #    於是有資格印「✅ 全項通過」。方向是樂觀的 fail-open。
    #    為什麼 --no-window 也要 fail-closed(而不是「刻意不判、視同通過」):
    #      ① 首航次全員命中是**前提**。前提「沒有量」不等於「成立」——
    #         把未量測當成通過,就是本專案反覆踩的那一類靜默失敗。
    #      ② --no-window 的每一個數字都已知被逃逸鏈污染(SW1:散布 11950 s、
    #         U3 32.6→22.6 從失敗側跨到通過側、U5 217.9→67.1),它自己的
    #         --help 就寫「只供對照」。一個對照模式印出 ✅,被貼進進度報告
    #         之後和驗收結論長得一模一樣 = 幽靈達標的第三條路。
    #      ③ 沒有任何合法消費者需要 --no-window 印通過:harness(stot_trial.sh)
    #         與總表(stot_repeat.sh / stot_rescore_all.py)都不帶這個旗標。
    #    但歸因必須分得開:「沒有全員命中」(飛行結果)≠「沒判定」(量測模式),
    #    所以 fails 的標籤不同,而且底下另外印一句話講清楚。
    if a.window:
        first_pass_ok = bool(verdict["all_hit"])
        first_pass_label = "首航次全員命中"
    else:
        first_pass_ok = False
        first_pass_label = "首航次全員命中(--no-window ⇒ 未判定,不予通過)"
    ok = (speed_ok and spread_ok and miss_ok and alt_ok and all_touch
          and first_pass_ok)
    fails = [n for n, good in (("終端總速", speed_ok), ("散布", spread_ok),
                               ("水平誤差", miss_ok), ("命中高度", alt_ok),
                               ("全員到齊", all_touch),
                               (first_pass_label, first_pass_ok))
             if not good]
    # ⚠️ 警告印在**判定的正上方**,不是報表開頭 —— 印在開頭的警告會落在
    #    SUMMARY_SENTINEL 之前,被 stot_trial.sh 的收尾擷取整段砍掉,航次結束時
    #    只看到一行判定,那就又變成一個靜默失敗。
    #    (以前這裡是 `| tail -12` 的固定行數預算;2026-08-05 改成哨兵擷取,
    #     警告可以隨便加幾則都不會把 ① 擠掉,但位置仍然必須在哨兵**之後**。)
    for w in warns:
        print(w)
    # ⚠️⚠️ --no-window 是對照/診斷模式,**永遠不會印通過**。這句話要印在哨兵
    #    之後(才進得了 stot_trial.sh 的收尾擷取)、判定的正上方,否則讀者只
    #    看到一個沒有理由的 ❌ 就會去改門檻。
    #    ⓘ 不放進 warns:warns 會進 --json 的 "warnings" 欄,而那是**資料品質**
    #      警告(逐機、可據以重跑);這一條是模式說明,兩者混在一起會讓下游
    #      「有沒有警告」的判斷失去意義。
    if not a.window:
        print("⚠️⚠️ --no-window:首航次窗關掉 ⇒ **沒有首航次判定**,而全域最近點"
              "會被逃逸後的隨機重穿污染(SW1:散布 11950 s、U5 217.90→67.11)。"
              "本模式只供對照,不予通過 —— 要驗收請拿掉 --no-window。")
    print("判定:%s" % ("✅ 全項通過" if ok else "❌ 未達標(%s)" % "、".join(fails)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
