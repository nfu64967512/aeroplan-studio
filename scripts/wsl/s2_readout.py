#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s2_readout.py — S2(`wait_goto_radius`)批次判讀。

S2 改的是什麼:`tot_node.py` 的 `_goto()` 變更偵測器存的是 `wp_pt=(lat,lon,alt)`,
**radius 不在裡面**。runin→wait 切換那一發 `_goto` 的 lat/lon/alt 逐位元相同
(`ring_point_of()` 不變;hold_alt=0 ⇒ `hold_alt_of()` 就是 `perch_of()`),
只有 radius 由 goto_radius=30 變成 hold_radius=1300 ⇒ moved=False
⇒ **那發 DO_REPOSITION 根本沒送出去**,飛機掛著舊的 param3=30 繼續直飛;
hold_radius 只有在下一個 30 s keepalive 補發時才生效
⇒ 一架會不會起彎,取決於它進 wait 時距 30 s 格點還有多久 —— **逐趟隨機**。

⇒ RRD(8 趟 × 2 架)其實是**兩個母體**,而不是一個有雜訊的母體:
     沒吃到補發 → 直進 → σ_pre 低 → 命中率高
     吃到補發   → 硬彎 → σ_pre 高 → 命中率 0
   兩群之間**零樣本** —— 離散失效模式的指紋。

本腳本做兩件事:
  ① 把任一批次(RRD / RRE / …)重新分成這兩個母體,並逐組報命中率。
     ⚠️ 分群門檻**不是**從答案倒推的:σ_pre 兩群之間有一段空帶,門檻放在空帶
        裡的任何一個值都給同一個分群(見 `SIG_PRE_SPLIT_DEG` 的註解)。
        腳本會把**實測的間隙**印出來;間隙塌掉就代表離散指紋消失、分群不可信。
  ② 對 S2 之後的批次(CSV 有 `req_radius_m`)驗證「機制有沒有生效」。

⚠️⚠️ **成功的指紋是「指令的缺席」。** 開關打開後 wait 與 runin 要求同一個半徑
   ⇒ 那一拍**不會**送出任何 DO_REPOSITION。在 log 裡找確認指令是找不到的。
   ⇒ 判機制有沒有生效**只能**看 `req_radius_m`(本拍**要求**的半徑,每拍都寫);
     **不能**看 `cmd_radius_m`(真正送出去的,hold-last-value)——
     後者在開關開與關時同樣是 30,會給**假綠燈**。腳本會主動印這條警語。

════ 量測紀律(違反任一條,結論一定錯)════════════════════════════════
1. 誤差**一律**用 `stot_score.py` 的觸地誤差(內插到觸地那一刻),不自己重寫。
   節點結案訊息的「最近水平距」是鎖存**上界**,可差兩個數量級
   (RRC1 鎖存 21.3 m,真觸地 0.18 m)。
2. `stot_score.py` 印的「最低AGL」那一欄(`alt_agl_m`)**不是**最低 AGL,
   是「最近水平通過點當下的**殘餘高度**」。RRD 16 架次那一欄是 −3.6…+42.8,
   而真正的最低 AGL 是 −8.3…−0.5(**16/16 全部落水**)。
   ⇒ 拿它判「有沒有落地」會憑空造出一個不存在的「高空掠過」族群。
   ⇒ 本腳本的最低 AGL 自己從 CSV 掃(見 `_scan_min_agl`)。
3. **不可用節點的 `expended`**:它只看水平距而且是鎖存,會犯兩個對稱的錯 ——
   把高空掠過記成命中、把真觸地誤判成沒觸地(收窗收在 expended 上)。
   ⇒ 掃描窗掃到 AGL ≤ 0,用逃逸半徑 20000 m 擋第二趟
   ([[second-pass-contamination]]:SITL 不終結落海機體,沉水再爬起會做第二趟)。
4. **落地 ≠ 命中**:RRD3/U2 最低 AGL −3.6 m(真落地)但觸地誤差 **647.88 m** ——
   它以 24.5 m 從紅心上空掠過,再飛過頭 648 m 才落地。兩個量分開印。
5. 單趟沒有診斷力(同組態散布雜訊底線 ±10 s)⇒ 這支腳本吃的是**批次**。

用法:
  PYTHONIOENCODING=utf-8 python scripts/wsl/s2_readout.py RRD        # 前綴 ⇒ RRD1..RRD8
  PYTHONIOENCODING=utf-8 python scripts/wsl/s2_readout.py RRE1 RRE2  # 指定趟次
  PYTHONIOENCODING=utf-8 python scripts/wsl/s2_readout.py --glob 'RRE?'
⚠️ 主控台是 cp950 ⇒ **一定**要帶 PYTHONIOENCODING=utf-8,否則整支假紅。
"""
import argparse
import csv
import json
import math
import os
import pathlib
import re
import statistics as st
import subprocess
import sys
import unicodedata

HERE = pathlib.Path(__file__).resolve()
ROOT = HERE.parents[2]
ART = ROOT / "runs" / "artifacts"
STOT_SCORE = HERE.parent / "stot_score.py"
R_EARTH = 6378137.0

# ── 判準常數。全部要嘛有反證數字,要嘛落在實測的空帶裡 ────────────────────
#: σ_pre 的窗長(秒)。**必須用時間、不可用列數。**
#: ⚠️ RRD 這批 CSV 每機 10864 列裡有 5431 對相鄰列**時戳相同**,先前被寫成
#:    「每一拍逐位元寫兩次」⇒ 用「最後 200 列」當窗口實際是 10.0–13.6 s 的
#:    浮動窗長,換一批 CSV 就不可移植。⇒ 這裡一律用**秒**。
#:    (時戳相同的真正原因見 `load_csv`,那不是重複寫,不可以去重。)
#: ⚠️ 窗長有一條有效帶:5–13 s 都給同一組分群;≥15 s 就垮
#:    (RRD4/U1 從 29.00° 掉到 5.50°,11/5 變成 12/4),因為它的硬彎只在放行前
#:    5.8 s 才開始,窗一長就被前面的直飛段稀釋。10 s 在帶子中央。
SIG_PRE_WIN_S = 10.0

#: 分群門檻(度)。⚠️⚠️ **這不是從答案倒推的**:RRD 實測 σ_pre 是
#:    0.00 0.20 1.70 3.30 3.40 3.50 3.60 4.50 4.70 5.90 5.90 ‖ 27.85 60.55 70.30 70.50 79.10
#:    —— (5.90°, 27.85°) 之間**零樣本**,門檻放在這 21.95° 空帶裡的任何值都給
#:    同一個 11/5 分群。15.0 大約在空帶中央,離兩側各 9.1 / 12.9°。
#: ⇒ 真正的判準不是這個數字,是**空帶存不存在**。腳本會把跨門檻的實測間隙印出來,
#:   間隙 < GAP_MIN_DEG 就代表離散指紋消失 ⇒ 分群不可信,要印警語。
SIG_PRE_SPLIT_DEG = 15.0
GAP_MIN_DEG = 10.0

#: 交叉判準(判準 C):wait 段內 |lead| 首次 ≥ TURN_THR_DEG 起算到放行的秒數。
#: ⚠️ 這比中位數判準**更穩健**,因為它量的是「硬彎持續了多久」,不會被窗長稀釋:
#:    RRD 直進群 0.00–0.40 s、硬彎群 5.80–31.20 s(間隙 5.4 s,倍率 14.5×),
#:    而且門檻從 10° 掃到 40° 指認出的**完全同一組 5 架**。
#: ⇒ 兩個判準指認的機體不一致時,腳本要吵(那代表資料變了,不是可以挑一個用)。
TURN_THR_DEG = 20.0
TURN_DUR_SPLIT_S = 2.0     # 落在 0.40 與 5.80 的空帶裡;也大於滾轉建立時間 1.05–2.50 s

#: 命中判準 = `stot_score.py` 的觸地誤差 ≤ 10 m。
#: ⚠️ RRD 沒有任何一架的誤差落在 5–10 m 之間 ⇒ 門檻取 5 或 10 都是 10/16,
#:    這個判準對門檻不敏感,不是湊出來的。
HIT_M = 10.0

#: 逃逸半徑:飛出這個距離就收窗。與 `stot_score.py --escape-r` 預設同一個數,
#: 兩支才可以互相對照。⚠️ RRD 最遠只到 ~6.1 km ⇒ 這個半徑**從未觸發**,
#:    真正收窗的是「首筆 agl ≤ 0」。兩者缺一不可:只靠逃逸半徑會一路掃到
#:    CSV 結尾,最低 AGL 變成沉沒深度 −102.9…−21.3 m,而不是 −8.3…−0.5 m。
ESCAPE_R_M = 20000.0

#: 契約半徑讀不到時的退路(RRD 實測值)。⚠️ 讀不到要**印警語**,不可靜默套用。
#: ⚠️⚠️ 「讀不到」包含**契約檔存在但缺 key**:`con.get("goto_radius", FALLBACK)`
#:    那種寫法會靜默套一組來自別批次的常數,等於用錯尺規量新資料。
#:    ⇒ 缺 key 必須走 None 那條路,警語才會真的觸發。
FALLBACK_GOTO_R = 30.0
FALLBACK_HOLD_R = 1300.0

#: `req_radius_m` 的最低覆蓋率。低於這個比例就**不可以**宣稱「全段 = goto_radius」。
#: ⚠️⚠️ 「全段」是本腳本對 S2 生效的核心斷言,而它可能只有一個樣本在撐:
#:    wait 段 1013 拍裡只要有 1 拍寫了 30.0、其餘全空,舊版照樣印「✅ 全段」,
#:    而且細節字串 `req=[30.0]` 與 100% 覆蓋時**一模一樣**,看不出差別。
#:    儀表若中途才開始寫、或恰好在半徑跳成 hold 的那幾拍是空的,就是高信心假綠燈。
REQ_COV_MIN = 0.90

#: 放行 = mode 第一次進入這個集合。
#: ⚠️ 用納入式(不是只認 "dive")才不會被 wait→missed 這種序列騙過去;
#:    實際落到 impact/expended 而不是 dive 的架次會另外標出來。
RELEASE_MODES = ("dive", "impact", "expended")


# ══════════════════════════════════════════════════════════════════════════
#  基礎工具
# ══════════════════════════════════════════════════════════════════════════
def _enc_guard():
    """cp950 主控台會讓整支腳本假紅。**不可以靜默 replace** —— 那是本專案的頭號禁忌。

    ⇒ 偵測到印不出 emoji 就改用 utf-8 並印一條 ASCII 警語,讓人知道發生了什麼。
    """
    try:
        "⚠️".encode(sys.stdout.encoding or "ascii")
        return
    except (UnicodeEncodeError, LookupError):
        pass
    sys.stderr.write(
        "[WARN] stdout encoding = %r cannot represent this script's output.\n"
        "       Forcing utf-8. Re-run with PYTHONIOENCODING=utf-8 for clean output.\n"
        % (sys.stdout.encoding,))
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                        # noqa: BLE001
        pass


def dist_m(a, b):
    """水平距(m)。與 `s1_readout.py` 的 `dist_m()` 同式(平均緯度縮放經度)。

    ⚠️ `_rrd_rows.json` 那邊用的是 cos(紅心緯度) ⇒ 6.1 km 上差 0.47–0.57 m
       (相對 9e-5)。那是慣例差不是誰算錯,但對帳時要知道用的是哪一個。

    ⚠️⚠️ **座標缺值一律回 NaN,不可以丟 TypeError。** `_f()` 依設計對空字串回
       `None`(見它的 docstring),而 `_row()` 對 lat/lon 以外的欄位本來就會寫空
       字串 —— 空字串是這份 CSV 的正常公民。一格 lat 是空字串就讓整批以未捕捉的
       TypeError 中止,會連已經算好的其他趟次(每趟數十秒的 stot_score 子行程)
       一起賠掉。NaN 會一路傳到航跡比,印成「—」= 不可判,那是誠實的答案;
       呼叫端另外數缺值列數並印警語,不讓它靜默。
    """
    if a is None or b is None or None in a or None in b:
        return float("nan")
    la = math.radians((a[0] + b[0]) / 2.0)
    return math.hypot(math.radians(b[0] - a[0]) * R_EARTH,
                      math.radians(b[1] - a[1]) * R_EARTH * math.cos(la))


def _dw(s):
    """顯示寬度(CJK 全形算 2)。表格對齊用。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in str(s))


def pad(s, w, right=False):
    s = str(s)
    f = " " * max(0, w - _dw(s))
    return (f + s) if right else (s + f)


def fnum(v, fmt="%.2f", dash="—"):
    if v is None or (isinstance(v, float) and v != v):
        return dash
    return fmt % v


# ══════════════════════════════════════════════════════════════════════════
#  載入
# ══════════════════════════════════════════════════════════════════════════
def expand_trials(tokens, globpat):
    """把 `RRD` 這種**批次前綴**展開成 RRD1..RRD8;完整趟次名則原樣採用。

    ⚠️ 展不出東西要**報錯**,不可以靜默回空清單然後印一張空表 ——
       那正是「靜默失敗」,本專案的頭號禁忌。
    """
    if not ART.is_dir():
        raise SystemExit(f"❌ 找不到 artifacts 目錄 {ART} ⇒ 沒有任何趟次可讀"
                         f"(是不是在別的 repo / 還沒跑過任何航次?)")
    out = []
    for tk in tokens:
        if (ART / tk).is_dir():
            out.append(tk)
            continue
        pat = re.compile(r"^" + re.escape(tk) + r"(\d+)$")
        hits = [(int(pat.match(p.name).group(1)), p.name)
                for p in ART.iterdir() if p.is_dir() and pat.match(p.name)]
        if not hits:
            raise SystemExit(f"❌ 找不到趟次或批次前綴 {tk!r}(看過 {ART})")
        out += [n for _, n in sorted(hits)]
    if globpat:
        out += sorted(p.name for p in ART.glob(globpat) if p.is_dir())
    seen, uniq = set(), []
    for t in out:
        if t not in seen:
            seen.add(t)
            uniq.append(t)
    if not uniq:
        raise SystemExit("❌ 沒有指定任何趟次")
    return uniq


def load_geom(trial):
    g = ART / trial / f"tot_{trial}_geom.csv"
    if not g.exists():
        return {}
    out = {}
    for r in csv.DictReader(open(g, encoding="utf-8", errors="replace")):
        try:
            out[r["kind"]] = (float(r["lat"]), float(r["lon"]))
        except Exception:                                    # noqa: BLE001
            pass
    return out


def load_contract(trial):
    """契約表(生效值)。讀得到就用它的 goto_radius/hold_radius,不要寫死。"""
    f = ART / trial / f"contract_{trial}.json"
    if not f.exists():
        return None
    try:
        return json.load(open(f, encoding="utf-8")).get("values", {})
    except Exception:                                        # noqa: BLE001
        return None


def load_csv(trial):
    """回傳 (欄位名, {sysid: [rows]}, 時戳碰撞對數, 逐位元重複對數)。

    ⚠️⚠️ **「時戳相同」不等於「重複列」,而且這兩件事的處置完全相反。**
       RRD1/U1 的 10864 列裡有 **5431** 對相鄰列時戳相同,但其中只有 **2134** 對
       是**內容也相同**的。剩下 3297 對是**真實而不同**的取樣 ——
           540.7 wait 25.0982592 120.6683096 agl 3054.0
           540.7 dive 25.0982858 120.6681888 agl 3053.7   ← 放行就在這一列
       真因是 `t` 欄被 `round(now - t0, 1)` **量化到 0.1 s**,而 `_row()` 跑
       ~18-20 Hz ⇒ 每個 0.1 s 桶裡本來就有兩列。
       (逐位元相同的那 2134 對則是另一回事:位置由 MAVLink ~10 Hz 更新,
        寫入 20 Hz ⇒ 有一半的列拿到和上一列一樣的 pos。)

    ⚠️⚠️ ⇒ **絕對不可以「同時戳只留第一列」去重。** 實測後果(本腳本第一版就是
       這樣寫的,而且看起來完全正常):
         · 放行點被推遲一拍 —— RRD1/U1 t_rel 540.7 → 540.8、R_rel 6138 → 6130、
           σ_rel 16.8 → 17.1(因為放行那一列和前一列 wait 同時戳,被當成重複丟掉);
         · 最低 AGL 掃過頭一格 —— RRD1/U1 −5.7 → −7.4(首筆 agl≤0 被丟掉,
           收窗晚一列)。
       兩個偏差都很小、都不會報錯,但方向一致 ⇒ 這正是本專案最怕的靜默偏移。
       ⇒ 本腳本**一律用原始列序**,所有窗口都用**秒**定義,不用列數。

    ⚠️ 還是要知道:`stot_score.py` 回報的 `n_samples` 是含碰撞的列數,
       任何用列數推取樣率的程式都會算出兩倍的假頻率(~18 Hz vs 真值 ~9 Hz 位置更新)。
    """
    f = ART / trial / f"tot_{trial}.csv"
    if not f.exists():
        return None, None, 0, 0
    seq, same_t, same_all = {}, 0, 0
    with open(f, encoding="utf-8", errors="replace") as fh:
        rd = csv.DictReader(fh)
        fields = list(rd.fieldnames or [])
        for r in rd:
            try:
                r["_t"] = float(r["t"])
            except (TypeError, ValueError):
                continue
            seq.setdefault(r["sysid"], []).append(r)
    for _s, Q in seq.items():
        for a, b in zip(Q, Q[1:]):
            if a["_t"] != b["_t"]:
                continue
            same_t += 1
            if all(a.get(c) == b.get(c) for c in (fields or [])):
                same_all += 1
    return fields, seq, same_t, same_all


def _f(r, k):
    """欄位取浮點。缺欄/空字串一律回 None。

    ⚠️ 空字串**不可以**當 0:節點寫空字串正是為了區分「沒啟用」與「值就是 0」
       (0 是合法的半徑、合法的誤差、合法的前置角)。
    """
    v = r.get(k)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


# ══════════════════════════════════════════════════════════════════════════
#  精度:一律外包給 stot_score.py
# ══════════════════════════════════════════════════════════════════════════
def score_trial(trial):
    """呼叫 `stot_score.py --json`,回傳 ({sysid:int -> per_uav dict}, 錯誤字串)。

    ⚠️ 精度計算**不重寫**。這支腳本只負責分群與機制證明;誤差的唯一真相是
       `stot_score.py` 的 `impact_miss_m`(內插到觸地那一刻)。
    ⚠️ 失敗要**回報**不可吞掉:回 (None, 原因),呼叫端把該趟標成不可判。
    """
    c = ART / trial / f"tot_{trial}.csv"
    g = ART / trial / f"tot_{trial}_geom.csv"
    if not c.exists():
        return None, "沒有 CSV"
    cmd = [sys.executable, str(STOT_SCORE), str(c), "--json"]
    if g.exists():
        cmd += ["--geom", str(g)]
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    try:
        p = subprocess.run(cmd, capture_output=True, env=env, timeout=600)
    except Exception as e:                                   # noqa: BLE001
        return None, f"呼叫失敗:{e}"
    if p.returncode != 0:
        return None, (f"stot_score.py exit={p.returncode}: "
                      f"{p.stderr.decode('utf-8', 'replace').strip()[:200]}")
    try:
        j = json.loads(p.stdout.decode("utf-8", "replace"))
    except Exception as e:                                   # noqa: BLE001
        return None, f"JSON 解析失敗:{e}"
    return {int(k): v for k, v in (j.get("per_uav") or {}).items()}, None


# ══════════════════════════════════════════════════════════════════════════
#  逐機判讀
# ══════════════════════════════════════════════════════════════════════════
def _scan_min_agl(D, k, tgt):
    """真最低 AGL —— **自己掃**,不用 `stot_score.py` 印的「最低AGL」欄。

    收窗規則(兩條缺一不可,見模組 docstring 紀律 3):
      ① 首筆 agl ≤ 0 就收(= 真的落地/落水那一刻);
      ② 距紅心 > ESCAPE_R_M 就收(擋第二趟污染)。
    ⚠️ 只有 ② 的話,RRD 會一路掃到 CSV 結尾,最低 AGL 變成沉沒深度
       −102.9…−21.3 m,而不是 −8.3…−0.5 m —— 兩個都是「16/16 落水」,
       但數字差一個量級,對帳時會以為誰算錯了。
    """
    lo = None
    for r in D[k:]:
        a = _f(r, "agl")
        if a is not None:
            lo = a if lo is None else min(lo, a)
            if a <= 0.0:
                break
        la, ln = _f(r, "lat"), _f(r, "lon")
        if tgt and la is not None and ln is not None \
                and dist_m((la, ln), tgt) > ESCAPE_R_M:
            break
    return lo


def read_aircraft(Q, tgt):
    """單機判讀。輸入是**原始**列序,而且**故意不去重**(見 `load_csv` 的說明:
    同時戳的兩列是不同取樣,去重會把放行點推遲一拍、把最低 AGL 掃過頭一格)。"""
    D = Q
    a = {"n_rows": len(Q), "released": False,
         "hold_ring_rows": sum(1 for r in Q if r.get("mode") == "hold_ring"),
         "mode_flips": 0, "max_gap_s": 0.0, "sig_pre": float("nan"),
         "turn_dur": float("nan")}
    if len(D) >= 2:
        # ⚠️ 空檔要連**兩端的位移**一起記下來。「取樣空檔」有兩個成因完全相反的
        #    解釋:節點/SITL 一起凍結(位移 ≈ 0)vs 真的掉樣本(飛機照飛,位移
        #    ≈ 空檔 × 速度)。只記秒數的話,報表就只能拿**別批**的舊結論去猜是
        #    哪一種 —— 那正是「把 RRD 的答案套到 RRE」這類硬編結論的來源。
        _g = max(zip(D, D[1:]), key=lambda z: z[1]["_t"] - z[0]["_t"])
        a["max_gap_s"] = _g[1]["_t"] - _g[0]["_t"]
        a["max_gap_move_m"] = dist_m((_f(_g[0], "lat"), _f(_g[0], "lon")),
                                     (_f(_g[1], "lat"), _f(_g[1], "lon")))
        # 「若真的在飛」應該走多遠 = 空檔秒數 × 地速。有了這個分母,報表才能
        # **用本批自己的數字**判斷是凍結還是掉樣本,不必引用別批的舊結論。
        _v = [v for r in (_g[0], _g[1]) for v in [_f(r, "gs")] if v is not None]
        a["max_gap_fly_m"] = (a["max_gap_s"] * max(_v)) if _v else float("nan")
    k = next((i for i, r in enumerate(D) if r.get("mode") in RELEASE_MODES), None)
    if k is None:
        a["closest_d"] = min((dist_m((_f(r, "lat"), _f(r, "lon")), tgt) for r in D
                              if _f(r, "lat") is not None and _f(r, "lon") is not None),
                             default=float("nan"))
        return a
    a["released"] = True
    a["release_mode"] = D[k]["mode"]
    r0 = D[k]
    sr = _f(r0, "lead_angle_deg")
    a.update(t_rel=r0["_t"],
             R_rel=dist_m((_f(r0, "lat"), _f(r0, "lon")), tgt),
             sig_rel=abs(sr) if sr is not None else float("nan"),
             agl_rel=_f(r0, "agl"), k=k)

    # ── σ_pre:放行**之前** SIG_PRE_WIN_S 秒的 median|lead_angle_deg| ──────
    # ⚠️ 不含放行拍本身:放行那一拍 PN 就開始拉桿,含進去會把直進群墊高。
    # ⚠️⚠️ 放行**那一拍**的 σ_rel 對分群**完全沒有鑑別力**(RRD 實測直進群
    #    1.3–23.9° vs 硬彎群 6.8–81.0°,兩者重疊;RRD3/U2 以 σ_rel=6.8° 放行
    #    卻脫靶 647.88 m)⇒ 分群訊息只存在於**放行前的歷史**。
    #    若有人拿「放行 σ 降下來了」當機制生效的證據,那是假綠燈。
    lo = a["t_rel"] - SIG_PRE_WIN_S
    w = [abs(v) for r in D[:k]
         for v in [_f(r, "lead_angle_deg")]
         if v is not None and lo <= r["_t"] < a["t_rel"]]
    a["sig_pre"] = st.median(w) if w else float("nan")
    a["sig_pre_n"] = len(w)

    # ── wait 段 ────────────────────────────────────────────────────────────
    wi = next((i for i, r in enumerate(D[:k]) if r.get("mode") == "wait"), None)
    a["wait_i"] = wi
    if wi is not None:
        seg = D[wi:k]
        a["t_wait"] = seg[0]["_t"]
        a["wait_len_s"] = a["t_rel"] - a["t_wait"]
        # far 旗標若來回翻面,「進 wait 的時刻」就沒有唯一定義 ⇒ 要吵
        a["mode_flips"] = sum(1 for x, b in zip(seg, seg[1:])
                              if x.get("mode") != b.get("mode"))
        # 判準 C:硬彎持續秒數
        tt = next((r["_t"] for r in seg
                   for v in [_f(r, "lead_angle_deg")]
                   if v is not None and abs(v) >= TURN_THR_DEG), None)
        a["turn_dur"] = 0.0 if tt is None else a["t_rel"] - tt
        # 航跡長(wait 段)。環點綁定在外面做,這裡先存端點。
        # ⚠️ lat/lon 有缺值時 dist_m 回 NaN ⇒ 航跡比整個變 NaN(印「—」= 不可判)。
        #    那是對的方向(寧可不可判也不要少算),但**不可靜默** ⇒ 數出來給報表印警語。
        p = 0.0
        a["bad_pos_rows"] = sum(1 for r in seg
                                if _f(r, "lat") is None or _f(r, "lon") is None)
        for x, b in zip(seg, seg[1:]):
            p += dist_m((_f(x, "lat"), _f(x, "lon")), (_f(b, "lat"), _f(b, "lon")))
        a["wait_path"] = p
        a["wait_p0"] = (_f(seg[0], "lat"), _f(seg[0], "lon"))
        a["wait_rows"] = seg
    a["min_agl"] = _scan_min_agl(D, k, tgt)
    return a


def bind_ring_points(acs, pps):
    """把各機綁到自己的環點(pp)。

    優先用 CSV 的 `wp_lat`/`wp_lon`(本拍真正瞄的航點,S2 儀表才有);沒有就用
    「wait 進入時**最近的 pp**」反推,並做**唯一指派**(貪婪),因為一個環點
    只會派給一架。
    ⚠️⚠️ 綁錯的後果是**會翻轉結論**,不是精度問題:RRD1/U2 綁到 pp0 時航跡比
       2.02(硬彎),綁到 pp1 變 0.90(看起來比直線還短)—— 一架硬彎機會被
       判成直進,而它正是受測對象。⇒ 用反推法一定要印警語 + 列出另一個綁法的值。
    """
    cand = []
    for key, a in acs.items():
        if a.get("wait_p0") is None or None in a["wait_p0"]:
            continue
        wp = None
        for r in a.get("wait_rows") or []:
            la, lo = _f(r, "wp_lat"), _f(r, "wp_lon")
            if la is not None and lo is not None:
                wp = (la, lo)
                break
        src, ref = ("wp", wp) if wp is not None else ("near", a["wait_p0"])
        for name, p in pps.items():
            cand.append((dist_m(ref, p), key, name, src))
    used_ac, used_pp = set(), set()
    for _d, key, name, src in sorted(cand, key=lambda x: (x[0], x[1], x[2])):
        if key in used_ac or name in used_pp:
            continue
        used_ac.add(key)
        used_pp.add(name)
        a = acs[key]
        a["pp_name"] = name
        a["pp_src"] = src
        a["ratio"] = a["wait_path"] / max(dist_m(a["wait_p0"], pps[name]), 1.0)
        alt = sorted((dist_m(a["wait_p0"], q), n2)
                     for n2, q in pps.items() if n2 != name)
        if alt:
            n2 = alt[0][1]
            a["ratio_alt"] = (n2, a["wait_path"]
                              / max(dist_m(a["wait_p0"], pps[n2]), 1.0))


# ══════════════════════════════════════════════════════════════════════════
#  S2 儀表(req_radius_m)
# ══════════════════════════════════════════════════════════════════════════
def s2_probe(a, gr, hr):
    """wait 段的半徑要求。回傳 (判定字串, 細節字串)。

    ⚠️⚠️ 決定量是 `req_radius_m`(本拍**要求**的半徑,每拍都寫)。
       `cmd_radius_m` 是 hold-last-value 的「真正送出去的半徑」,開關開與關時
       **同樣都是 goto_radius** —— 因為開關關閉時那發 hold_radius 根本沒送出去。
       拿它當證明是**假綠燈**。
    """
    if gr is None or hr is None:
        return "⚠️ 不可判", "這一趟讀不到契約半徑 ⇒ 沒有尺規,不判"
    seg = a.get("wait_rows") or []
    req = [v for r in seg for v in [_f(r, "req_radius_m")] if v is not None]
    cmd = [v for r in seg for v in [_f(r, "cmd_radius_m")] if v is not None]
    if not req:
        return "⚠️ 不可判", f"wait 段 {len(seg)} 列沒有任何 req_radius_m 值(欄位存在但全空)"
    uq = sorted({round(v, 1) for v in req})
    uc = sorted({round(v, 1) for v in cmd})
    cov = len(req) / max(len(seg), 1)
    det = f"req={uq} cmd={uc} 覆蓋 {len(req)}/{len(seg)}({100.0 * cov:.1f}%)"
    if cov < REQ_COV_MIN:
        return ("⚠️ 覆蓋不足,不可判",
                det + f" ⇒ 低於 {100.0 * REQ_COV_MIN:.0f}%,「全段」無從斷言")
    near_hold = [v for v in uq if abs(v - hr) <= max(1.0, 0.02 * hr)]
    near_goto = all(abs(v - gr) <= max(1.0, 0.02 * max(gr, 1.0)) for v in uq)
    if near_hold:
        return "❌ 要求過 hold_radius", det
    if near_goto:
        return "✅ 全段 = goto_radius", det
    return "⚠️ 半徑既非 goto 也非 hold", det


# ══════════════════════════════════════════════════════════════════════════
#  報表
# ══════════════════════════════════════════════════════════════════════════
def _collect(trials, warn):
    """逐趟載入 + 逐機判讀。回傳 (rows, have_s2, rad_of, contract_wgr, 統計)。

    ⚠️⚠️ `rad_of` 是**逐趟**的 (goto_radius, hold_radius) 尺規,不是整批共用一組。
       舊版把 gr/hr 宣告在迴圈外、每趟有契約就覆寫 ⇒ 最後只剩**最後一趟**的值,
       拿去判整批所有架次;某一趟沒有契約檔時還會沿用前一趟的值,全程無聲。
       hold_radius 遲早要重調([[hold-ring-is-unflyable]]),屆時把舊批次與新批次
       併起來對照(這正是 S2 對照實驗要做的事)就會用錯尺規翻轉判定。
    """
    rows, have_s2, contract_wgr, rad_of = [], {}, {}, {}
    tot_same_t = tot_same_all = tot_rows = 0
    for tr in trials:
        fields, seq, same_t, same_all = load_csv(tr)
        if seq is None:
            print(f"⚠️ {tr}:沒有 CSV —— 這一趟**不存在**,不可以算進統計")
            warn.append(f"{tr} 沒有 CSV")
            continue
        tot_same_t += same_t
        tot_same_all += same_all
        tot_rows += sum(len(v) for v in seq.values())
        geom = load_geom(tr)
        tgt = geom.get("target")
        if tgt is None:
            print(f"⚠️ {tr}:geom.csv 沒有 target 列 ⇒ 這一趟的距離量全部不可信,跳過")
            warn.append(f"{tr} 沒有 target")
            continue
        pps = {k: v for k, v in geom.items() if k.startswith("pp")}
        con = load_contract(tr)
        g_t = h_t = None
        if con:
            miss_k = [k for k in ("goto_radius", "hold_radius") if k not in con]
            if miss_k:
                # ⚠️ 缺 key **不可以**靜默套 FALLBACK:那是拿別批次的常數當尺規。
                print(f"⚠️⚠️ {tr}:契約表缺 {'/'.join(miss_k)} ⇒ 這一趟沒有可信的判定尺規")
                warn.append(f"{tr} 契約缺 {'/'.join(miss_k)}")
            else:
                try:
                    g_t = float(con["goto_radius"])
                    h_t = float(con["hold_radius"])
                except (TypeError, ValueError):
                    g_t = h_t = None
            if "wait_goto_radius" in con:
                contract_wgr[tr] = con["wait_goto_radius"]
        rad_of[tr] = (g_t, h_t)
        have_s2[tr] = all(c in (fields or []) for c in ("req_radius_m", "cmd_radius_m"))

        per, err = score_trial(tr)
        if per is None:
            print(f"⚠️ {tr}:stot_score.py 失敗({err})⇒ 這一趟的誤差全部標為不可判")
            warn.append(f"{tr} 計分失敗:{err}")
            per = {}

        acs = {}
        for s, Q in sorted(seq.items(), key=lambda kv: int(kv[0])):
            a = read_aircraft(Q, tgt)
            a["trial"], a["sysid"] = tr, int(s)
            sc = per.get(int(s)) or {}
            a["miss"] = sc.get("impact_miss_m")
            a["touch_t"] = sc.get("impact_t_s")
            a["score_ok"] = bool(sc)
            a["has_s2"] = have_s2[tr]
            acs[(tr, int(s))] = a
        bind_ring_points(acs, pps)
        rows += [acs[key] for key in sorted(acs, key=lambda x: x[1])]
    return rows, have_s2, rad_of, contract_wgr, tot_same_t, tot_same_all, tot_rows


def report(trials, split_deg):
    warn = []
    (rows, have_s2, rad_of, contract_wgr,
     tot_same_t, tot_same_all, tot_rows) = _collect(trials, warn)
    if not rows:
        raise SystemExit("❌ 沒有任何可判讀的架次")

    # ── 判定尺規:**逐趟**。缺的那一趟才退回,不牽連別趟 ────────────────────
    seen_tr = sorted({a["trial"] for a in rows})
    for tr in seen_tr:
        g_t, h_t = rad_of.get(tr, (None, None))
        if g_t is None or h_t is None:
            rad_of[tr] = (FALLBACK_GOTO_R, FALLBACK_HOLD_R)
            print(f"⚠️ {tr}:讀不到契約半徑 ⇒ 退回 goto_radius={FALLBACK_GOTO_R:.0f} / "
                  f"hold_radius={FALLBACK_HOLD_R:.0f}(RRD 實測值);"
                  f"若這一趟不是這組值,它的 S2 判定會錯")
            warn.append(f"{tr} 契約半徑讀不到")
    uniq_rad = sorted({rad_of[tr] for tr in seen_tr})
    if len(uniq_rad) > 1:
        print("⚠️⚠️ 各趟的 goto/hold 半徑**不一致**:"
              + " / ".join(f"{tr}=({rad_of[tr][0]:.0f},{rad_of[tr][1]:.0f})"
                           for tr in seen_tr))
        print("     ⇒ 尺規已逐趟套用(不共用),但**併批對照時要自己確認這是刻意的**:"
              "半徑不同的兩批本來就不是同一個實驗。")
        warn.append("各趟契約半徑不一致")
    gr, hr = uniq_rad[0]        # 只用於整批敘述文字;判定一律走 rad_of[trial]

    # ── 逐機表 ────────────────────────────────────────────────────────────
    print("═" * 100)
    print(f"【逐機表】{len(rows)} 架次 / {len(trials)} 趟   "
          f"σ_pre = 放行前 {SIG_PRE_WIN_S:.0f} s 的 median|lead_angle_deg|")
    hdr = [("趟/機", 11), ("t_rel", 8), ("R_rel", 8), ("σ_rel", 7), ("σ_pre", 8),
           ("硬彎s", 8), ("航跡比", 8), ("觸地誤差", 11), ("最低AGL", 10),
           ("分群", 8), ("命中", 6)]
    print("".join(pad(h, w, right=(i > 0)) for i, (h, w) in enumerate(hdr)))
    for a in rows:
        name = f"{a['trial']}/U{a['sysid']}"
        if not a["released"]:
            print(pad(name, 11)
                  + "❌ 從未放行(最近 %s m)" % fnum(a.get("closest_d"), "%.0f"))
            continue
        sp = a["sig_pre"]
        grp = "?" if sp != sp else ("硬彎" if sp > split_deg else "直進")
        a["group"] = grp
        a["hit"] = None if a["miss"] is None else (a["miss"] <= HIT_M)
        print(pad(name, 11)
              + pad(fnum(a["t_rel"], "%.1f"), 8, True)
              + pad(fnum(a["R_rel"], "%.0f"), 8, True)
              + pad(fnum(a["sig_rel"], "%.1f"), 7, True)
              + pad(fnum(sp, "%.2f"), 8, True)
              + pad(fnum(a.get("turn_dur"), "%.1f"), 8, True)
              + pad(fnum(a.get("ratio"), "%.2f"), 8, True)
              + pad(fnum(a["miss"], "%.2f", "不可判"), 11, True)
              + pad(fnum(a.get("min_agl"), "%.1f"), 10, True)
              + pad(grp, 8, True)
              + pad("✅" if a["hit"] else ("—" if a["hit"] is None else "❌"), 6, True))

    odd = [a for a in rows if a["released"] and a.get("release_mode") != "dive"]
    if odd:
        print("⚠️ %d 架的放行拍不是 mode==\"dive\" 而是 %s ⇒ 放行點定義退化,單獨看"
              % (len(odd), sorted({a["release_mode"] for a in odd})))
        warn.append("有架次的放行拍不是 dive")
    nrel = sum(1 for a in rows if not a["released"])
    if nrel:
        print(f"⚠️ {nrel} 架從未放行 ⇒ 不進分群統計,但**要算進整批分母**")

    ok = [a for a in rows if a["released"] and a["sig_pre"] == a["sig_pre"]]

    # ⚠️⚠️ σ_pre 算不出來的架次會被**靜默剔除**,而成因包含「整個 lead_angle_deg
    #    欄不存在」。那時【★決定性讀數★】會印「硬彎組 n=0 命中 0/0」——
    #    標題掛著★決定性讀數★卻在說「這批根本沒量到」,正是本專案定義的靜默失敗。
    #    ⇒ 剔除幾架就吵幾架;一架都不剩就**直接中止**,不印 0/0 這種假數字。
    nsig = [a for a in rows if a["released"] and a["sig_pre"] != a["sig_pre"]]
    if nsig:
        print("⚠️⚠️ %d 架放行了但 σ_pre **算不出來**(lead_angle_deg 欄缺或放行前 %.0f s 全空)"
              % (len(nsig), SIG_PRE_WIN_S))
        print("     ⇒ 這些架次**不進分群**(分群母體因此變小),但仍在整批分母內:"
              + ", ".join(f"{a['trial']}/U{a['sysid']}" for a in nsig))
        warn.append("%d 架 σ_pre 算不出來(未分群)" % len(nsig))
    if not ok:
        raise SystemExit(
            "❌ σ_pre **全部**算不出來(放行 %d 架,無一架有可用的 lead_angle_deg)\n"
            "   ⇒ 分群不可判。硬要往下印會得到「硬彎 0/0 / 直進 0/0」——\n"
            "     那不是「命中 0」,是「這批沒量到」,兩者長得一模一樣 ⇒ 中止,不印。\n"
            "   本次判讀的保留:%s"
            % (sum(1 for a in rows if a["released"]), " / ".join(warn) if warn else "無"))

    # ── 分群 ──────────────────────────────────────────────────────────────
    hard = [a for a in ok if a["group"] == "硬彎"]
    strt = [a for a in ok if a["group"] == "直進"]
    sortd = sorted(a["sig_pre"] for a in ok)
    print("\n" + "═" * 100)
    print("【分群】門檻 σ_pre = %.1f°(常數;落在實測空帶裡,見原始碼註解)" % split_deg)
    print("  排序後的 σ_pre:" + "  ".join("%.2f" % v for v in sortd))

    lo_max = max([a["sig_pre"] for a in strt], default=float("nan"))
    hi_min = min([a["sig_pre"] for a in hard], default=float("nan"))
    gap = (hi_min - lo_max) if (hard and strt) else float("nan")
    gaps = sorted(((b - x, x, b) for x, b in zip(sortd, sortd[1:])), reverse=True)
    print("  跨門檻的實測間隙 = %s°(低群 max %s° ‖ 高群 min %s°)"
          % (fnum(gap, "%.2f"), fnum(lo_max, "%.2f"), fnum(hi_min, "%.2f")))
    if gap == gap and gap < GAP_MIN_DEG:
        print("  ⚠️⚠️ 間隙 < %.0f° ⇒ **離散指紋消失**,這批不再是兩個母體。"
              "分群數字不可信,先查資料再談結論。" % GAP_MIN_DEG)
        warn.append("σ_pre 間隙塌掉(%.2f°)" % gap)
    if gap != gap:
        # ⚠️ 單一母體時 gap 退化成 NaN,而 `gap == gap` 的護欄會把上面那條警語擋掉
        #    ⇒ 「跨門檻的實測間隙 = —°」不痛不癢地印過去。要明講它**沒被檢驗**。
        why = ("高群是空的(這批全部判為直進)" if strt and not hard else
               "低群是空的(這批全部判為硬彎)" if hard and not strt else
               "兩群都是空的")
        print("  ⚠️⚠️ 跨門檻間隙**算不出來**(%s)⇒ 「離散指紋」在此批**無從檢驗**。" % why)
        print("     單一母體本身不是壞事(S2 修好之後就該長這樣),但它**不能**由這一段")
        print("     背書 —— 要由「開關真的開著 + req_radius_m 全段 = goto_radius」來證明。")
        warn.append("單一母體:跨門檻間隙算不出來(離散指紋未檢驗)")
    if len(sortd) < 4:
        # ⚠️ n 太小的時候「最大間隙」永遠存在而且永遠不等於跨門檻間隙 ⇒ 那段敘述
        #    會對 2–3 架的批次印出一段煞有介事的群內結構分析,實際上零資訊。
        print("  ⚠️ 只有 %d 架量得到 σ_pre ⇒ 間隙結構無從談起(n 太小),"
              "不要從排序清單讀出分群結構。" % len(sortd))
        warn.append("σ_pre 樣本只有 %d 架(間隙結構不可讀)" % len(sortd))
    elif gaps and gaps[0][0] < GAP_MIN_DEG:
        # ⚠️⚠️ 這一段舊版是**只在無鑑別力時才開火**的偽分群:單一母體時 `gap` 是
        #    NaN ⇒ `gap != gap` 恆真 ⇒ 就在「跨門檻間隙算不出來」的警語正下方,
        #    拿本批**雜訊等級**的最大間隙(實測 1.60°,遠低於 GAP_MIN_DEG=10°)
        #    印出一整段「把 11 架切成 3 / 8、落在低群內部」的兩群敘述。
        #    反證很尖銳:**真的**有兩群時(RRD gap=65.80/21.95°)這段反而不印。
        #    ⇒ 最大間隙沒有超過 GAP_MIN_DEG 就代表「沒有群內結構」,要明講,
        #      不可以照著雜訊講故事。
        top, glo, ghi = gaps[0]
        print("  本批**最大**的間隙只有 %.2f°(在 %.2f 與 %.2f 之間)< %.0f° ⇒ "
              "排序清單裡**沒有群內結構**," % (top, glo, ghi, GAP_MIN_DEG))
        print("     這是一段**連續分布**,不是幾個小群。⇒ 不要從它讀出任何"
              "「切成 N 架 / M 架」的敘述,那是雜訊。")
    elif gaps and (gap != gap or abs(gaps[0][0] - gap) > 1e-9):
        top, glo, ghi = gaps[0]
        if gap != gap:
            # 單一母體,但**確實**存在一個 ≥ GAP_MIN_DEG 的間隙 ⇒ 值得講,
            # 但它是母體**內部**的結構,不是「兩個母體」的證據,措辭要分清楚。
            print("  ⚠️ 本批只有一個母體(跨門檻間隙算不出來),但排序清單裡仍有一個 "
                  "%.2f° 的間隙(在 %.2f 與 %.2f 之間)≥ %.0f°"
                  % (top, glo, ghi, GAP_MIN_DEG))
            print("     ⇒ 它是**單一母體內部**的結構,**不是**分群門檻兩側的間隙"
                  " —— 不可以拿它當「這批其實有兩個母體」的證據。")
            warn.append("單一母體內部仍有 %.2f° 的間隙(需另外解釋)" % top)
        else:
            print("  ⚠️ 注意:最大的間隙不是跨門檻那一個,而是 %.2f°(在 %.2f 與 %.2f 之間)"
                  % (top, glo, ghi))
        n_lo = sum(1 for v in sortd if v <= glo)
        n_hi = len(sortd) - n_lo
        print("     ⇒ 群內本身還有結構;不要把任何一群敘述成一段連續分布。")
        print("        這個間隙把 %d 架切成 %d 架(≤ %.2f°)/ %d 架(≥ %.2f°);"
              % (len(sortd), n_lo, glo, n_hi, ghi))
        if hi_min == hi_min and glo >= hi_min:
            iso = [a["sig_pre"] for a in hard if a["sig_pre"] <= glo]
            print("        而且它落在**高群內部**(高群下緣 %.2f°):高群的下緣 %d 架"
                  "(%s)與上緣被這個間隙隔開"
                  % (hi_min, len(iso), ", ".join("%.2f" % v for v in sorted(iso))))
            print("        ⇒ 下緣若只有 1–2 架,那是孤點,不屬於任何一群,"
                  "不要拿它當高群的代表。")
        elif lo_max == lo_max and ghi <= lo_max:
            iso = [a["sig_pre"] for a in strt if a["sig_pre"] >= ghi]
            print("        而且它落在**低群內部**(低群上緣 %.2f°):低群的上緣 %d 架"
                  "(%s)與下緣被這個間隙隔開"
                  % (lo_max, len(iso), ", ".join("%.2f" % v for v in sorted(iso))))

    # 判準 C 交叉檢查
    cset = {(a["trial"], a["sysid"]) for a in ok
            if a.get("turn_dur") == a.get("turn_dur")
            and a.get("turn_dur") >= TURN_DUR_SPLIT_S}
    sset = {(a["trial"], a["sysid"]) for a in hard}
    print("  交叉判準 C(wait 段內 |lead|≥%.0f° 持續到放行的秒數 ≥ %.1f s)指認 %d 架"
          % (TURN_THR_DEG, TURN_DUR_SPLIT_S, len(cset)))
    if not cset and not sset:
        # ⚠️⚠️ 空集合 == 空集合 也成立 ⇒ 舊版會對著兩個空集合蓋「互相佐證 ✅」的章。
        #    而**單一母體正是 S2 修好之後應該看到的樣子**:S2 有效 ⇒ RRE 不該再有
        #    硬彎組 ⇒ 兩個判準都指認 0 架 ⇒ 決定性讀數正上方蓋一個零證據力的 ✅。
        #    這是交接文件警告過的假綠燈形狀,只是換成了判準一致性。
        print("     ⚠️⚠️ 兩個判準**都沒有指認任何硬彎機** ⇒ 一致性檢查在此批"
              "**沒有鑑別力**(空集合對空集合,不是互相佐證)。")
        print("        單一母體是 S2 生效時**預期**的形狀,但這條檢查對它零證據力:")
        print("        要證明 S2 生效,看【S2 機制生效證明】那一段的 req_radius_m,"
              "不是看這裡。")
        warn.append("單一母體(兩判準都指認 0 架)⇒ 一致性檢查無鑑別力")
    elif cset == sset:
        print("     ✅ 與 σ_pre 判準指認**完全同一組**(兩個獨立判準互相佐證)")
    else:
        only_a = sorted(sset - cset)
        only_c = sorted(cset - sset)
        print("     ⚠️⚠️ 兩個判準**不一致** —— 代表資料的結構變了,"
              "不可以挑一個順眼的用:")
        if only_a:
            print("        只有 σ_pre 判為硬彎:" + ", ".join(f"{t}/U{s}" for t, s in only_a))
        if only_c:
            print("        只有 持續時間 判為硬彎:" + ", ".join(f"{t}/U{s}" for t, s in only_c))
        warn.append("兩個分群判準不一致")

    # ── 決定性讀數 ────────────────────────────────────────────────────────
    def rate(g):
        n = len(g)
        h = sum(1 for a in g if a.get("hit"))
        u = sum(1 for a in g if a["miss"] is None)
        return n, h, f"{h}/{n}" + (f"(其中 {u} 架不可判)" if u else "")

    nh, _hh, sh = rate(hard)
    ns, _hs, ss = rate(strt)
    # ⚠️⚠️ 空的一群**絕對不可以**印裸的 `0/0`。本腳本自己在第 668-673 行(σ_pre
    #    全部算不出來時)逐字寫著:「那不是『命中 0』,是『這批沒量到』,兩者長得
    #    一模一樣 ⇒ 中止,不印」—— 兩群都空就中止、單群空就照印 `0/0`,是同一條
    #    規則自相矛盾。而且兩種空的**意義完全相反**:
    #      · 硬彎組空 = S2 生效時**預期**的形狀(受測的失效模式消失了);
    #      · 直進組空 = 全面回歸(或分群門檻/儀表壞了),是最壞的消息。
    #    印成同一個 `0/0` 會讓最好的結果與最壞的結果長得一模一樣。
    n_ok = len(ok)
    empty_hard = ("(本批**沒有這一群**:一架都沒有 ⇒ 這不是「命中 0」。"
                  "%d 架全判直進 —— 這正是 S2 生效時**預期**的形狀,"
                  "硬彎母體本來就該消失)" % n_ok)
    empty_strt = ("(本批**沒有這一群**:一架都沒有 ⇒ 這不是「命中 0」。"
                  "%d 架全判硬彎 —— 這是**全面回歸**(或分群門檻/儀表壞了),"
                  "先查為什麼一架直進都沒有)" % n_ok)
    print("\n" + "═" * 100)
    print("【★決定性讀數★】")
    print(f"  硬彎組  n = {nh:2d}   "
          + (empty_hard if nh == 0
             else f"命中(觸地誤差 ≤ {HIT_M:.0f} m)= **{sh}**"))
    print(f"  直進組  n = {ns:2d}   "
          + (empty_strt if ns == 0
             else f"命中(觸地誤差 ≤ {HIT_M:.0f} m)= **{ss}**"))
    if nh == 0:
        warn.append("硬彎組是空的(S2 生效時的預期形狀;命中率無從計算,不是 0/0)")
    if ns == 0:
        warn.append("直進組是空的(全面回歸?);命中率無從計算,不是 0/0")
    # ⚠️ 整批那一行也要掛「其中 N 架不可判」的註記:stot_score 拿不到誤差時
    #    舊版印成 0/N(0.0%),讀起來像「一架都沒打中」的事實陳述 —— 把「量不到」
    #    印成「量到了而且是 0」是本專案最忌諱的方向。
    _na, allh, _sall = rate(rows)
    u_all = sum(1 for a in rows if a["miss"] is None)
    print(f"  整批    n = {len(rows):2d}   命中 = **{allh}/{len(rows)}**"
          f"({100.0 * allh / max(len(rows), 1):.1f}%)"
          + (f"  ⚠️ 其中 {u_all} 架**不可判**(誤差量不到)⇒ 這不是「沒打中」"
             if u_all else ""))
    mh = [a["miss"] for a in hard if a["miss"] is not None]
    ms = [a["miss"] for a in strt if a["miss"] is not None]
    # ⚠️⚠️ 舊版是 `if mh and ms:` ⇒ **任何**單邊母體(S2 生效 ⇒ 硬彎組空;
    #    全面回歸 ⇒ 直進組空)都會讓「誤差中位 + 全距」整條**靜默消失**:
    #    沒有替代輸出、沒有警語、也不進保留欄。缺的那一群要**明說缺**,
    #    而且要分清楚「這一群沒有飛機」與「有飛機但誤差全部量不到」。
    def _mtxt(v, g):
        if v:
            return "%.2f m" % st.median(v)
        return ("(本批沒有這一群)" if not g
                else "(這一群 %d 架的誤差**全部不可判**)" % len(g))

    def _rtxt(v):
        return ("%.2f–%.2f" % (min(v), max(v))) if v else "—"

    print(f"  誤差中位:硬彎 {_mtxt(mh, hard)}  vs  直進 {_mtxt(ms, strt)}"
          f"(全距 硬彎 {_rtxt(mh)} / 直進 {_rtxt(ms)})")
    if not mh or not ms:
        warn.append("誤差中位只有單邊(%s那一群沒有可用的誤差)"
                    % ("硬彎" if not mh else "直進"))
    # 落地 ≠ 命中(紀律 4)
    # ⚠️ 分母只算**掃得到 AGL** 的架次。CSV 缺 agl 欄時舊版印成 0/N,那是
    #    「一架都沒落地」的語氣,實際意思是「這批量不到落地」。
    agl_ok = [a for a in rows if a.get("min_agl") is not None]
    land = [a for a in agl_ok if a["min_agl"] < 0.0]
    print(f"  逐機真觸地率 = {len(land)}/{len(agl_ok)}(自掃 AGL≤0,不用 stot_score 的"
          f"「最低AGL」欄)⚠️ 落地 ≠ 命中")
    n_noagl = len(rows) - len(agl_ok)
    if n_noagl:
        print(f"     ⚠️⚠️ 另有 {n_noagl} 架**掃不到 AGL**(agl 欄缺或全空)⇒ 分子分母"
              f"都已排除。這**不是**「沒落地」,是量不到。")
        warn.append(f"{n_noagl} 架掃不到 AGL(已排除在真觸地率之外)")
    bad = [a for a in land if a.get("hit") is False]
    if bad:
        print("     其中 %d 架真落地但未命中,誤差 %s m —— 它們是**飛過頭**,不是沒落地"
              % (len(bad), ", ".join("%.1f" % a["miss"] for a in bad)))

    # ── S2 機制生效證明 ────────────────────────────────────────────────────
    print("\n" + "═" * 100)
    print("【S2 機制生效證明】")
    s2_verdict = None          # True=生效已證明 / False=未獲證明 / None=無從判定
    # ⚠️⚠️ 「契約說開關已開、CSV 卻沒有 S2 儀表欄」的對帳**必須逐趟**做,而且
    #    必須在**任何**批次形狀下都跑到。舊版把它關在 `if not any(have_s2.values()):`
    #    (= **整批**都沒有儀表)的分支裡 ⇒ **混合批次完全不觸發**。
    #    而 RRE 是 8 趟 = 正是混合批次,「其中一趟跑到舊 binary / 參數沒吃到」
    #    是本專案反覆踩到的形狀([[jet-conversion-never-applied]]、
    #    [[catch-speed-rejected-by-fc]]、[[live-config-violates-contract]])
    #    ⇒ 那正是 RRE **最可能**的部分失效,舊版會被 else 分支裡一句良性的
    #    「這一趟沒有任何可判的架次」放過去。⇒ 判定提到分支**外面**,逐趟檢查。
    on_claim, off_claim, bad_claim = [], [], []
    for t, v in sorted(contract_wgr.items()):
        try:
            (on_claim if float(v) != 0.0 else off_claim).append((t, v))
        except (TypeError, ValueError):
            bad_claim.append((t, v))
    ghost = [(t, v) for t, v in on_claim if not have_s2.get(t)]
    if ghost:
        print("  ⚠️⚠️ 契約說開關**已開**,但這些趟次的 CSV **沒有 S2 儀表欄**:%s"
              % ", ".join(f"{t}(wait_goto_radius={v})" for t, v in ghost))
        print("     ⇒ 參數宣告與節點實際行為**不一致**(舊 binary?參數沒吃到?"
              "跑到別的 launch?)")
        print("     ⇒ 這**不是**「S2 部署前的正常批次」,先查為什麼契約說開,"
              "再談這批資料。")
        print("     ⇒ 這幾趟**不可以**與其餘趟次合併統計:它們既不是開關組"
              "(沒有證據)也不是對照組(契約說開)。")
        warn.append("⚠️⚠️ 契約說開關已開但 CSV 沒有 S2 儀表欄:"
                    + ",".join(t for t, _ in ghost))
    if bad_claim:
        print("  ⚠️ 契約的 wait_goto_radius 值看不懂(%s)⇒ 開/關無從判定"
              % ", ".join(f"{t}={v}" for t, v in bad_claim))
        warn.append("契約 wait_goto_radius 值無法解讀")
    if not any(have_s2.values()):
        print("  ⚠️⚠️ **這批沒有 S2 儀表** —— CSV 沒有 req_radius_m / cmd_radius_m 欄。")
        print("     ⇒ 無法從資料證明 `wait_goto_radius` 有沒有生效。這是 S2 部署**之前**")
        print("       的批次(RRD 就是)的正常結果,但**不可以**當成「機制沒問題」:")
        print("       它只代表這個問題在這批資料上量不出來。")
        print("     ⇒ 要驗 S2,必須跑部署後的批次(RRE),CSV 才會有那四個欄位。")
        warn.append("這批沒有 S2 儀表(req_radius_m 欄不存在)")
        if contract_wgr:
            print("     契約表在這批**有**宣告 wait_goto_radius:"
                  + ", ".join(f"{t}={v}" for t, v in sorted(contract_wgr.items())))
    else:
        miss_col = [t for t, v in have_s2.items() if not v]
        if miss_col:
            print(f"  ⚠️⚠️ 這批**混了**沒有 S2 儀表的趟次:{miss_col} ⇒ 不可以合併統計")
            warn.append("批次混了有/沒有 S2 儀表的趟次")
        print("  ⚠️⚠️ 判定量是 **req_radius_m**(本拍**要求**的半徑,每拍都寫)。")
        print("     **不可以**看 cmd_radius_m —— 那是真正送出去的(hold-last-value),")
        if len(uniq_rad) == 1:
            print("     開關**開與關同樣都是 %.0f**(關閉時那發 %.0f 根本沒送出去)⇒ 假綠燈。"
                  % (gr, hr))
        else:
            print("     開關**開與關同樣都是各趟自己的 goto_radius**"
                  "(關閉時那發 hold_radius 根本沒送出去)⇒ 假綠燈。")
            print("     ⚠️ 本批各趟的 (goto, hold) 不一致 ⇒ 判定尺規逐趟套用:"
                  + " / ".join(f"{tr}=({rad_of[tr][0]:.0f},{rad_of[tr][1]:.0f})"
                               for tr in seen_tr))
        print("     ⚠️ 成功的指紋是「指令的缺席」:開關打開後 runin→wait 那一拍**不會**")
        print("        送出任何 DO_REPOSITION。在 log 裡找確認指令是找不到的,那是對的。")
        print()
        print(pad("趟/機", 11) + pad("S2 判定", 26) + "  細節")
        # ⚠️⚠️ 計數**必須逐趟**。舊版用整批累加的 n_on/n_off 去對逐趟的契約,
        #    兩個方向都致命:
        #      · 假綠燈 —— 一個「契約說關、資料卻顯示開關生效」的**被污染對照組**,
        #        單獨跑會報警,一旦與別趟併批就完全靜音(連保留欄都乾淨)。
        #        而對照組是「直進 10/11 vs 硬彎 0/5」整個宣稱的地基。
        #      · 假紅 —— 最自然的 A/B 批次(開關組 + 關閉組一起跑)必定對每個
        #        開關組趟次誤報「契約與實際致動不一致」,把人送去查不存在的問題。
        #    而本腳本 docstring 紀律 5 自己寫「單趟沒有診斷力 ⇒ 這支腳本吃的是批次」
        #    ⇒ 失效的正是**唯一被授權的用法**。
        per_tr = {}                       # trial -> [n_on, n_off, n_unk]
        for a in rows:
            if not a.get("has_s2") or not a["released"]:
                continue
            g_t, h_t = rad_of.get(a["trial"], (None, None))
            v, det = s2_probe(a, g_t, h_t)
            print(pad(f"{a['trial']}/U{a['sysid']}", 11) + pad(v, 26) + "  " + det)
            c = per_tr.setdefault(a["trial"], [0, 0, 0])
            c[0] += v.startswith("✅")
            c[1] += v.startswith("❌")
            c[2] += v.startswith("⚠️")
        t_on = sum(c[0] for c in per_tr.values())
        t_off = sum(c[1] for c in per_tr.values())
        t_unk = sum(c[2] for c in per_tr.values())
        print(f"  ⇒ 【整批總計】wait 段全程只要求 goto_radius 的:**{t_on}** 架;"
              f"要求過 hold_radius 的:{t_off} 架;不可判 {t_unk} 架")
        print("     ⚠️ 整批總計**只是總計**,不可拿來對任何一趟的契約 —— "
              "下面的契約交叉檢查一律逐趟做。")
        for t in sorted(per_tr):
            c = per_tr[t]
            print(f"       {t}: goto {c[0]} / hold {c[1]} / 不可判 {c[2]}")
        no_con = [t for t in sorted(per_tr) if t not in contract_wgr]
        if no_con:
            print("  ⚠️⚠️ 這些趟次有 S2 儀表、卻**沒有契約值**可對帳:%s"
                  % ", ".join(no_con))
            print("     ⇒ contract_*.json 的 values 裡沒有 wait_goto_radius ⇒ "
                  "無法判斷它們是開關組還是對照組,不可以拿來做 A/B。")
            warn.append("有 S2 儀表但契約缺 wait_goto_radius:" + ",".join(no_con))
        for t, v in sorted(contract_wgr.items()):
            try:
                on = float(v) != 0.0
            except (TypeError, ValueError):
                on = None
            n_on, n_off, n_unk = per_tr.get(t, (0, 0, 0))
            print(f"  契約 {t}: wait_goto_radius = {v}"
                  + ("" if on is None else ("(開)" if on else "(關)"))
                  + f"  ‖ 本趟實測:goto {n_on} / hold {n_off} / 不可判 {n_unk}")
            if t not in per_tr:
                # ⚠️⚠️ 舊版把「沒有 S2 儀表欄」與「有儀表但沒放行」併成同一句
                #    良性措辭。兩者成因天差地遠:前者 = 節點根本沒寫出儀表
                #    (舊 binary / 參數沒吃到),是**部署失敗**;後者 = 這一趟
                #    飛得不順(沒放行),是**航次問題**。併成一句 ⇒ 前者被後者
                #    的語氣稀釋掉,而前者正是 RRE 最可能的部分失效。
                if not have_s2.get(t):
                    print("     ⚠️⚠️ 這一趟**沒有 S2 儀表欄**(CSV 裡沒有 req_radius_m /"
                          " cmd_radius_m)—— 契約宣告了,節點卻沒寫出來。")
                    print("        ⇒ 這是**部署沒生效**的形狀(舊 binary?參數沒吃到?"
                          "跑到別的 launch?),不是「這一趟飛得不好」。先查部署。")
                    warn.append(f"⚠️⚠️ {t} 契約有宣告但沒有 S2 儀表欄(部署可能沒生效)")
                else:
                    print("     ⚠️ 這一趟**有** S2 儀表欄,但沒有任何架次放行(或放行的架次"
                          "wait 段一個 req_radius_m 值都沒有)⇒ 契約無從對帳,")
                    print("        它既不算開關組也不算對照組。這是**航次**問題,"
                          "不是部署問題。")
                    warn.append(f"{t} 有 S2 儀表但沒有可判架次(沒放行)")
                continue
            if on is True and n_off:
                print("     ⚠️⚠️ 契約說**開**,但**本趟**有 %d 架要求過 hold_radius ⇒ "
                      "契約與實際致動**不一致**,先查參數有沒有真的吃到。" % n_off)
                warn.append(f"{t} 契約說開但資料顯示未生效")
            if on is False and n_on and not n_off:
                print("     ⚠️⚠️ 契約說**關**,但**本趟**沒有任何機體要求過 hold_radius"
                      "(%d 架全段 = goto_radius)⇒ 這一趟的對照組沒有成立,"
                      "不能當 baseline。" % n_on)
                warn.append(f"{t} 契約說關但資料看不到 hold_radius")
            if n_unk:
                print("     ⚠️ 本趟另有 %d 架不可判 ⇒ 上面的開/關對帳只涵蓋其餘架次。"
                      % n_unk)

        # ── ⭐ 正面讀數:讓成功長得像成功 ──────────────────────────────────
        # ⚠️⚠️ S2 生效的指紋是「指令的**缺席**」+ 單一母體 ⇒ 成功在報表上天生會
        #    長成「⚠️ 跨門檻間隙算不出來」+「⚠️ 一致性檢查無鑑別力」+「硬彎組
        #    沒有這一群」三條警語疊在一起。若整份報表沒有任何一句**正面**斷言,
        #    下一個人會把 S2 成功讀成「判讀壞了」,然後去修一個沒有壞的東西。
        #    ⇒ 這裡把兩個條件(契約宣告 + 實測致動)合起來明文下結論。
        good, bad = [], []
        for t, v in on_claim:
            n_on, n_off, n_unk = per_tr.get(t, (0, 0, 0))
            if not have_s2.get(t):
                bad.append(f"{t}(沒有 S2 儀表欄)")
            elif n_on and not n_off and not n_unk:
                good.append((t, n_on))
            else:
                bad.append(f"{t}(goto {n_on} / hold {n_off} / 不可判 {n_unk})")
        print()
        if not on_claim:
            print("  ⚠️ 沒有任何一趟的契約宣告 wait_goto_radius 為**開** ⇒ 這批"
                  "**不能**用來證明 S2 生效(它最多是對照組)。")
            warn.append("沒有任何一趟契約宣告 wait_goto_radius 為開(無從證明生效)")
        elif good and not bad:
            s2_verdict = True
            tot = sum(n for _, n in good)
            print("  ✅✅ 【S2 生效】契約宣告 **開** ✔(%s) ‖ 實測致動 wait 段全程"
                  "只要求 goto_radius **%d/%d 架** ✔"
                  % (", ".join(t for t, _ in good), tot, tot))
            print("     ⇒ **S2 在這批生效**。兩個條件同時成立才算數:契約真的開了,"
                  "而且資料真的看到「wait 段從頭到尾只要求 goto_radius」。")
            if not nh:
                print("     ⇒ 因此上面的「硬彎組沒有這一群 / 跨門檻間隙算不出來 /"
                      " 一致性檢查無鑑別力」**不是判讀壞掉**,")
                print("        而是 S2 生效**應有**的形狀:受測的失效模式消失了,"
                      "母體本來就該只剩一個。")
            else:
                print("     ⚠️ 但本批仍有 %d 架被判為硬彎 ⇒ 那**不是** wait_goto_radius"
                      " 造成的(致動已證明全程 goto),要另外找成因。" % nh)
                warn.append("S2 致動已證明生效,但仍有 %d 架硬彎(另有成因)" % nh)
        else:
            s2_verdict = False
            print("  ❌ 【S2 生效未獲證明】契約宣告開的趟次裡,這些**沒有**通過致動"
                  "驗證:" + " / ".join(bad))
            if good:
                print("     (通過的只有:%s)⇒ **不可以**拿通過的那幾趟代表整批。"
                      % ", ".join(t for t, _ in good))
            warn.append("S2 生效未獲證明(契約說開但致動沒通過):"
                        + ",".join(b.split("(")[0] for b in bad))

    # ── 環點綁定 ──────────────────────────────────────────────────────────
    print("\n" + "═" * 100)
    print("【環點綁定】")
    srcs = {a.get("pp_src") for a in rows if a.get("pp_src")}
    if not srcs:
        print("  ⚠️ 沒有任何一架綁得到環點(geom.csv 沒有 pp 列?)⇒ 航跡比整欄不可信")
        warn.append("環點綁不到")
    elif srcs == {"wp"}:
        print("  ✅ 全部用 CSV 的 wp_lat/wp_lon 綁定(本拍真正瞄的航點,不是猜的)")
    else:
        n_near = sum(1 for a in rows if a.get("pp_src") == "near")
        print(f"  ⚠️ {n_near} 架**沒有 wp_lat/wp_lon**,改用「wait 進入時最近的 pp」反推。")
        # ⚠️ 敘述用**本批自己**算出來的敏感架次,不寫死別批的數字。
        sens = [a for a in rows
                if a.get("ratio_alt") and a.get("ratio") is not None
                and a["ratio"] == a["ratio"]
                and (a["ratio"] - 1.0) * (a["ratio_alt"][1] - 1.0) < 0]
        if sens:
            e = sens[0]
            print("     這是**猜的**:綁錯會翻轉結論,不是精度問題 —— 本批有 %d 架跨過 1.0,"
                  % len(sens))
            print("     例如 %s/U%s 綁 %s 時航跡比 %.2f,綁 %s 變 %.2f ⇒ 硬彎/直進直接對調。"
                  % (e["trial"], e["sysid"], e["pp_name"], e["ratio"],
                     e["ratio_alt"][0], e["ratio_alt"][1]))
            warn.append("%d 架的航跡比對環點綁定敏感(跨過 1.0)" % len(sens))
        else:
            print("     這是**猜的**:綁錯會翻轉結論,不是精度問題。本批沒有任何一架的")
            print("     兩種綁法跨過 1.0 ⇒ 這一次的結論對綁定不敏感(但仍然是猜的)。")
        print("     ⇒ 下面同時列出「綁到另一個 pp」會得到的比值,自己看差多少:")
        for a in rows:
            if a.get("ratio_alt") and a.get("pp_name"):
                n2, r2 = a["ratio_alt"]
                flag = ("   ⚠️ 兩者跨過 1.0 ⇒ 這一架對綁定敏感"
                        if (a["ratio"] - 1.0) * (r2 - 1.0) < 0 else "")
                print("       %-12s %s→%s   %s→%s%s"
                      % (f"{a['trial']}/U{a['sysid']}", a["pp_name"],
                         fnum(a["ratio"]), n2, fnum(r2), flag))
        warn.append("環點靠「最近的 pp」反推(非 wp_lat/wp_lon)")

    # ── 資料品質 ──────────────────────────────────────────────────────────
    print("\n【資料品質】")
    print(f"  相鄰列**時戳相同**:{tot_same_t} / {tot_rows} 對("
          f"{100.0 * tot_same_t / max(tot_rows, 1):.1f}%);"
          f"其中**內容也相同**的只有 {tot_same_all} 對"
          f"({100.0 * tot_same_all / max(tot_same_t, 1):.1f}%)")
    print("     ⚠️⚠️ 這兩個數字**不是同一件事**:t 欄量化到 0.1 s 而 _row() 跑 ~18-20 Hz")
    print("        ⇒ 每個 0.1 s 桶裡本來就有兩列**不同**的取樣。「同時戳只留第一列」會把")
    print("        放行點推遲一拍(RRD1/U1 t_rel 540.7→540.8、R_rel 6138→6130)、把最低")
    print("        AGL 掃過頭一格(−5.7→−7.4)—— 小、無聲、方向一致。本腳本**不去重**。")
    print("     ⚠️ 但列數仍不可當取樣率:`stot_score.py` 的 n_samples 是含碰撞的列數。")
    mg = max((a.get("max_gap_s", 0.0) for a in rows), default=0.0)
    print(f"  最大取樣空檔 = {mg:.1f} s")
    if mg > 1.0:
        # ⚠️⚠️ 舊版在這裡直接印 RRD 的結論與分母(「這不是掉樣本 —— 實測位移只有
        #    0–11.6 m … 會錯(11/16)」),對**任何** mg > 1.0 的批次都印。
        #    RRE 上線後會對 RRE 印 RRD 的 11/16 —— 這正是本專案一再吃虧的
        #    「把別批的答案冒充成本批的量測」。⇒ 只描述**本批實測**;RRD 那組
        #    數字降級成明確標示的「參考」,不冒充本批數字。
        gp = [(a["max_gap_move_m"], a.get("max_gap_fly_m", float("nan")))
              for a in rows
              if a.get("max_gap_s", 0.0) > 1.0
              and a.get("max_gap_move_m") is not None
              and a["max_gap_move_m"] == a["max_gap_move_m"]]
        mv = sorted(m for m, _fm in gp)
        fly = sorted(f for _m, f in gp if f == f)
        frac = [m / f for m, f in gp if f == f and f > 1.0]
        if mv:
            print("     本批有 %d 架的最大空檔 > 1.0 s:**空檔兩端的實際位移**"
                  " = %.1f–%.1f m(中位 %.1f m)"
                  % (len(mv), mv[0], mv[-1], st.median(mv)))
        if mv and fly and frac:
            print("        對照「若真的在飛」應走的距離(空檔秒數 × 該拍地速)"
                  "= %.0f–%.0f m ⇒ 實際/應走 = %.1f%%–%.1f%%"
                  % (fly[0], fly[-1], 100.0 * min(frac), 100.0 * max(frac)))
        if frac and max(frac) < 0.2:
            print("     ⇒ 本批**不是掉樣本**:實際位移最多只有應走距離的 %.1f%% "
                  "⇒ SITL/MAVProxy/節點**一起凍結**,但 time.time() 照走。"
                  % (100.0 * max(frac)))
        elif frac and min(frac) > 0.8:
            print("     ⇒ 本批**是真的掉樣本**:實際位移至少已達應走距離的 %.1f%% "
                  "⇒ 飛機照飛,只是那一段沒寫進 CSV —— 與凍結是兩回事,處置也不同。"
                  % (100.0 * min(frac)))
        elif frac:
            print("     ⚠️ 本批**兩種成因混在一起**(實際/應走 = %.1f%%–%.1f%%)⇒ "
                  "不可以整批套同一個解釋,要逐機看。"
                  % (100.0 * min(frac), 100.0 * max(frac)))
            warn.append("取樣空檔的成因逐機不一致(凍結 / 掉樣本混合)")
        else:
            print("     ⚠️ 本批算不出空檔兩端的位移或地速(lat/lon/gs 缺值)⇒ **無法**"
                  "判斷這是凍結還是掉樣本,不要預設任何一種。")
            warn.append("最大空檔 > 1 s 但算不出位移/地速(凍結、掉樣本無從分辨)")
        print("     ⚠️ 不論是哪一種,都**不可以**用理想的 30 s keepalive 格點去推"
              "補發時刻,要用 CSV 裡的**實際 tick**。")
        print("        (參考,這是 **RRD 的舊量測、不是本批的數字**:在 RRD 上用"
              "理想格點推,16 架次裡會錯 11 架。)")
    # ⚠️ lat/lon 缺值:dist_m 回 NaN(不再丟 TypeError 吃掉整批),但**不可靜默**。
    bp = [a for a in rows if a.get("bad_pos_rows", 0) > 0]
    if bp:
        print("  ⚠️⚠️ %d 架的 wait 段有 lat/lon **缺值**(合計 %d 列)⇒ 航跡長會變 NaN、"
              "航跡比印成「—」= 不可判" % (len(bp), sum(a["bad_pos_rows"] for a in bp)))
        print("     " + ", ".join("%s/U%s(%d 列)" % (a["trial"], a["sysid"],
                                                     a["bad_pos_rows"]) for a in bp))
        warn.append("%d 架 wait 段有 lat/lon 缺值(航跡比不可判)" % len(bp))
    fl = [a for a in rows if a.get("mode_flips", 0) > 1]
    if fl:
        print(f"  ⚠️ {len(fl)} 架的 wait 段 mode 來回翻面 ⇒「進 wait 的時刻」沒有唯一定義")
        warn.append("wait 段 mode 翻面")
    hrr = sum(a.get("hold_ring_rows", 0) for a in rows)
    print(f"  hold_ring 列數合計 = {hrr}(抵環即放行 ⇒ 必須 0;"
          f"非 0 表示 dive_from_runup 沒生效)")
    nosc = [a for a in rows if not a.get("score_ok")]
    if nosc:
        print(f"  ⚠️ {len(nosc)} 架沒有 stot_score 結果 ⇒ 誤差標為不可判,"
              f"已排除在命中分子外(但**仍在分母內**)")

    print("\n" + "═" * 100)
    print("【本次判讀的保留】" + (" / ".join(warn) if warn else "無"))
    # ⚠️⚠️ 保留欄不可以是報表的最後一句話。S2 生效的形狀天生就是「單一母體 +
    #    一致性檢查無鑑別力 + 硬彎組沒有這一群」⇒ 保留欄一定是滿的,讀者掃到
    #    最後一行只會看到一排警語,把**成功**讀成「判讀壞了」。⇒ 結論殿後。
    if s2_verdict is True:
        print("── 結論:**S2 生效**(契約宣告開 ✔ ‖ 實測致動 wait 段全程 = "
              "goto_radius ✔)。上面的保留是**單一母體應有**的形狀,不是判讀故障。")
    elif s2_verdict is False:
        print("── 結論:**S2 生效未獲證明** —— 契約與實測對不上,詳見"
              "【S2 機制生效證明】那一段,不要拿這批當 S2 已生效的證據。")


def main(argv=None):
    _enc_guard()
    ap = argparse.ArgumentParser(
        description="S2(wait_goto_radius)批次判讀:兩個母體分群 + 機制生效證明")
    ap.add_argument("trials", nargs="*",
                    help="趟次名(RRE1)或批次前綴(RRD ⇒ RRD1..RRD8)")
    ap.add_argument("--glob", default=None, help="另外用 glob 挑趟次")
    ap.add_argument("--split", type=float, default=SIG_PRE_SPLIT_DEG,
                    help="σ_pre 分群門檻(度);預設 %(default)s,落在實測空帶裡")
    a = ap.parse_args(argv)
    if not a.trials and not a.glob:
        ap.error("要給趟次名稱、批次前綴或 --glob")
    report(expand_trials(a.trials, a.glob), a.split)


if __name__ == "__main__":
    main()
