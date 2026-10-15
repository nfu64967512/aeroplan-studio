# -*- coding: utf-8 -*-
"""B2:路徑 A(縱向閉在 n_z 上)的**值層級**迴歸鎖(L1–L8,2026-08-20)。

═══════════════════════════════════════════════════════════════════════════
 這一組鎖在守什麼
═══════════════════════════════════════════════════════════════════════════
dataflash(RRE1–7 × U1/U2、俯衝段 10155 拍)證實的機制:

  · 姿態環**追得好**(|Pitch−DesPitch| 中位 1.54–4.83°、PIDP.Err 中位 0.000、
    Dmod 全 1.000 無限幅),升降舵 max 只用到 26.8% 行程、碰軌 0.0000%;
  · 但後段實得 n_z 中位只有 **0.94**、|γ̇| 中位只有 **0.56 °/s**
    —— 3 g 只需要 6.37 °/s,差 **10.6 倍**;
  · 俯衝段 AOA 中位 **−3.2°(負攻角)** ⇒ n_z 自然落在 0.6–0.9。

⇒ 結論不是「機體做不到」,是**整條鏈路沒有任何一段在閉 γ̇ 或 n_z 的迴路**:
   舊路 `pitch = γ_cmd + α_est` 把「**當下的** α」當前饋加上去,而不是
   「**產生所要 n_z 需要的** α」。它是角度追蹤迴路 + 落後的 α 前饋,
   飛機照角度飛、不多不少,那 2 g 沒人去要。

路徑 A 換掉的就是最後那一步:

    α_cmd = ( n_z·W/(q·S) − C_L0 ) / C_Lα
    pitch = γ + α_cmd / max(cos roll, 0.5)

⚠️⚠️ 交辦的式子寫成 `α_cmd = n_z·W/(C_Lα·q·S)`(**漏掉 C_L0**)。辨識結果
   C_L0 = 0.56381 是一個**常數偏移** C_L0/C_Lα = 4.650°,與 q 無關、不會自己
   消掉;換算成指令 n_z 是 **+20.7 g**。L2 有一條專門的反向鎖盯這一項
   (`test_dropping_c_l_zero_would_be_a_20g_error`)。

⚠️ 交辦的反解式 `α = pitch·cos(roll) − γ` 的代數也是錯的。正解是
   `α = (pitch − γ)·max(cos roll, 0.5)`(把前向式反過來,含 0.5 的地板)。
   本檔一律用正解。

═══════════════════════════════════════════════════════════════════════════
 ⭐⭐ 為什麼一條源碼字串比對都沒有
═══════════════════════════════════════════════════════════════════════════
本專案實證:源碼層的 38 條鎖被 **≥25 種等價改寫**繞過(改上游、改下游、
換容器、換命名、把式子搬進 helper —— 全部免疫);改成值層級之後 26 條
**一次關掉 8 個**([[value-locks-beat-source-locks]])。

所以本檔一律:**餵一組數字進去 → 對送出來的數字下斷言**。
唯一走 AST 的地方是「把受測方法從檔案原始碼綁成 mixin」與「讀
`declare_parameter` 的**預設值**」—— 讀的是**值**,不是那一行長什麼樣;
換行、改排版、加註解、把 `g()` 換成別的 helper 都不影響。

⚠️ 用 `exec` 原始碼而不是 `import tot_node`:ROS2 在 Windows 開發機上不可用,
   而這些鎖必須**每次都跑得到**(跑不到的鎖等於沒有鎖)。走原始碼也保證
   測的是「檔案裡真正在飛的那一份」,不是另外維護的複本。
   (與 `tests/test_mpng_value_locks.py` 同一套手法。)

═══════════════════════════════════════════════════════════════════════════
 ⭐⭐ L1 的黃金值是**改動前**跑出來的
═══════════════════════════════════════════════════════════════════════════
`_GOLD_BEFORE` 那 11 組數字是在 B1 動 `_pn_cmd` **之前**,從當時的
`tot_node.py` 逐位元抓下來的(mtime 2026-08-19 22:22 的那一版)。
⇒ 它們是真正的「修改前」參照,不是事後從新碼倒推的自我證明。
   事後倒推的黃金值只能鎖住「今天以後不要再變」,鎖不住「這一輪沒有動到」——
   而這一輪要保的正是**對照組不被污染**:現況 15/16 命中、觸地誤差中位
   0.80 m,精度不在授權範圍內([[spread-outranks-terminal-speed]])。

11 組工作點刻意覆蓋:一般俯衝 / LOS 地板咬到 / 地板把 n_z 打出band /
淺角滿油門 / 陡角高動壓 / α≠0 的橫縱耦合 / 大前置角 / roll 貼夾制 /
負 n_z 下限 / 同幾何但高度不同的兩點(後者在舊碼**必須**同值 —— 舊碼
根本沒有 q 這個量,這是 L3 的對照組)。

═══════════════════════════════════════════════════════════════════════════
 L1 開關關閉 ⇒ 逐位元同現行        L5 兩顆開關獨立、四種組合各自可達
 L2 送出的 pitch 真的產生所要 n_z   L6 終端凍結不受影響
 L3 n_z 上限吃機體值(隨 q 變)     L7 橫向零影響
 L4 順序 bug(地板繞過夾制)修好了  L8 CSV 六個新欄的位置與空字串紀律
═══════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════
 ⭐⭐ 突變測試紀錄(2026-08-20,18 個變異全部轉紅)
═══════════════════════════════════════════════════════════════════════════
「鎖亮綠燈」不等於「鎖有用」。本檔逐一把 `tot_node.py` 的**副本**改壞
(repo 的那一份一個字都沒動),確認每一種壞法都真的被抓到:

  變異                                   轉紅的那一組
  ── 縱向換算 ────────────────────────────────────────────────────────────
  M1  漏掉 C_L0(= 交辦式子的原樣)        L2
  M1b 用夾制前的 nz 而不是 cmd_nz          L2 / L4 / L5
  M1c α_cmd 沒除 max(cos roll, 0.5)        L2
  M2  基準用 γ_cmd 而不是量測的 γ          L2
  M8  α_cmd 算了但沒接上致動鏈             L1 / L2 / L3 / L5
  ── 上限 ────────────────────────────────────────────────────────────────
  M3  閉迴路不套機體上限                   L3
  M4  機體上限用固定動壓(不隨 q 變)      L3
  M7  關閉時也套機體上限                   L1 / L3
  M9  上緣寫死回 3.0                       L3 / L4
  M14 動壓退回教科書 q(少一個 σ)         L0 / L3
  ── 順序 bug ────────────────────────────────────────────────────────────
  M5  夾完只記錄、不回寫 γ 指令            L4 / L5
  M16 把夾制搬到地板之前                   L4 / L5
  ── 對照組與旁通道 ──────────────────────────────────────────────────────
  M6  開關預設打開                         L0 / 前提鎖
  M10 閉迴路順手動到 roll(×1.000001)     L7
  M15 終端凍結連油門一起凍                 L6
  ── 記錄層 ──────────────────────────────────────────────────────────────
  M11 未設值寫 0 不寫空字串                L8
  M12 六欄順序調換                         L8
  M13 alpha_cmd 關閉時寫 0                 L8

⚠️ 重跑腳本在 scratchpad(`mut_nz.py`),不在版本庫裡 —— 它會複製一份 repo
   到 temp 再改,**不會**碰到工作目錄。
═══════════════════════════════════════════════════════════════════════════
"""
import ast
import csv
import itertools
import math
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCHES = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
TOT = PATCHES / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))

sys.path.insert(0, str(PATCHES))
import predictor as _pred                                    # noqa: E402

GRAV = 9.80665


# ══════════════════════════════════════════════════════════════════════════
#  把受測方法從**檔案原始碼**綁成 mixin(不 import,不複製邏輯)
# ══════════════════════════════════════════════════════════════════════════
def _func(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError(f"tot_node.py 找不到 {name}()")


def _seg(node):
    return ast.get_source_segment(SRC, node)


def _block(name):
    """把一個方法縮排成可以塞進 `class _M:` 的形狀。"""
    return "\n".join("    " + line for line in _seg(_func(name)).splitlines())


def _declared_default(name):
    """讀 `declare_parameter` 的**預設值**(讀值,不是讀那一行長什麼樣)。"""
    for n in ast.walk(TREE):
        if (isinstance(n, ast.Call) and len(n.args) == 2
                and isinstance(n.args[0], ast.Constant)
                and n.args[0].value == name):
            return ast.literal_eval(n.args[1])
    raise AssertionError(f"tot_node.py 沒有宣告參數 {name}")


def _class_default(name):
    """讀節點類別上的**類別層級**預設值(樁與真節點共用的那一份)。"""
    for cls in (n for n in ast.walk(TREE) if isinstance(n, ast.ClassDef)):
        for st in cls.body:
            if isinstance(st, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id == name
                    for t in st.targets):
                return ast.literal_eval(st.value)
    raise AssertionError(f"tot_node.py 的類別上沒有 {name}")


def _module_ns():
    """重建 `_pn_cmd` / `_qbar` / `_row` 會碰到的模組層命名空間。

    ⚠️ 模組層的常數與純函式**一律自動搬過來**,不逐個列名。理由是實務的:
       這一輪 `tot_node.py` 在本檔動筆期間被 B1 改了三次(`_air_density`
       這個 helper 出現過又被收掉、`AERO_WEIGHT_N` 換成行內乘法)。
       逐個列名的寫法會在每一次重構時丟一個看起來像「測試壞了」的
       NameError —— 而那正是讓人把鎖停掉的原因。
    ⚠️ 兩趟:先 literal_eval(純字面值),再在已累積的命名空間裡 eval
       (吃得下 `A * B`、`getattr(_pred, ...)` 這類)。吃不下的
       (QoSProfile 等需要 rclpy)直接跳過 —— 受測方法碰不到它們。
    """
    ns = {"math": math, "csv": csv, "time": None, "_pred": _pred,
          "EARTH": 111320.0}
    todo = []
    for n in TREE.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(
                n.targets[0], ast.Name):
            try:
                ns[n.targets[0].id] = ast.literal_eval(n.value)
            except Exception:                                # noqa: BLE001
                todo.append((n.targets[0].id, _seg(n.value)))
    for name, src in todo:
        try:
            ns[name] = eval(src, ns)                         # noqa: S307
        except Exception:                                    # noqa: BLE001
            pass
    for n in TREE.body:
        if isinstance(n, ast.FunctionDef):
            try:
                exec(_seg(n), ns)
            except Exception:                                # noqa: BLE001
                pass
    return ns


_NS = _module_ns()
S_REF = _NS["AERO_S_REF_M2"]
#: ⚠️ 重量取模組自己的 `AERO_WEIGHT_N`(有的話),沒有就用 m·g 現算 ——
#: 這一輪 `tot_node.py` 在 `AERO_WEIGHT_N` 與行內 `AERO_MASS_KG * G0` 之間
#: 來回過三次,寫死其中一種會在下一次重構時丟一個假的紅燈。
WEIGHT = _NS.get("AERO_WEIGHT_N") or _NS["AERO_MASS_KG"] * _NS["G0"]
RHO0 = _NS["RHO0"]
#: ISA 密度比的指數。⚠️ 本檔**獨立**寫一份,刻意不從 `tot_node` 讀:
#: 從被測碼讀等於「用它自己驗它自己」,σ 整個被拿掉時反而測不出來。
#:
#: ⚠️ 已知的第五位差異(2026-08-20 實測,不是猜的):導引律(`_qbar` 與
#:   `_eas_of`,兩者同源)走 **4.25588**,`predictor.air_density` 走 4.2586。
#:   在 3400 m 它讓 σ 差 0.022%、q 差 0.044%、指令 n_z 差 **0.0004 g**
#:   —— 量級上與「忘了乘 σ」(3400 m 差 0.86 g,差 2000 倍)完全不同類。
#:   本檔跟導引律那一個(它才是飛出去的那一個)。
#: ⇒ 本檔對 q 的獨立比對用 rel=2e-4:抓得到少乘一個 σ / σ 次方用錯,
#:   放得過第五位的指數分歧。真正需要**逐位元**的 L2 改用節點自己記的 q,
#:   理由寫在 TestL2 的 docstring。
_ISA_EXP = 4.25588
Q_REL = 2e-4


def _mixin(names):
    ns = dict(_NS)
    exec("class _M:\n" + "\n\n".join(_block(n) for n in names), ns)
    return ns["_M"]


_GUID = _mixin(("_geom", "_qbar", "_eas_of", "_itcg_bias", "_pn_cmd", "_pn_step"))
_LOG = _mixin(("_open_csv", "_row"))


class _Log:
    """⚠️ 樁的 logger 只是不印,不吞任何別的東西。錯誤要數得出來。"""

    def __init__(self):
        self.errors = []

    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def debug(self, *a, **k):
        pass

    def error(self, m, *a, **k):
        self.errors.append(str(m))


# ══════════════════════════════════════════════════════════════════════════
#  導引樁:只提供 `_pn_cmd` / `_qbar` / `_pn_step` 真的會碰到的東西
# ══════════════════════════════════════════════════════════════════════════
class Guid(_GUID):
    """⚠️ 刻意**不提供**多餘屬性:多給一個,就會把「忘了初始化某個 dict」
    這一整類失效在測試裡蓋掉,而那正是本專案的頭號失效類別。

    ⚠️⚠️ 類別層的預設值必須與 `_pn_cmd` 改動**之前**的生效值一致,否則
       `_GOLD_BEFORE` 那 11 組黃金值就對不上了。新加的五顆(nz_closed_loop /
       nz_clamp_after_floor / nz_max_g / aero_* )全部取「等同舊行為」的值。
    """

    # ── ITCG / MPNG(本組鎖全程關閉:縱向的事不牽動時間項)──
    itcg_on = False
    itcg_gain = 1.0
    itcg_a_max_g = 0.4
    mpng_on = False
    mpng_k_eps = 0.0
    mpng_k_s = 0.0
    # ── PN 三通道(RRE 契約快照值)──
    n_lat = 4.0
    n_vert = 5.0
    pn_lead = 1.0
    pn_vmax = 210.0
    pn_los_floor = 0.0
    pn_nz_min = 0.05
    pn_alpha_tau = 1.75
    tick_dt = 0.05
    dive_pitch_max = 45.0
    dive_pitch_min = -80.0
    pn_thr_trim = 0.36
    pn_thr_gain = 0.02
    pn_late_full = True
    dive_full_thr = False
    fz_r = 100.0
    fz_tgo = 1.5
    # ── ⭐ 路徑 A 的五顆旋鈕。預設 = 逐位元同現行 ──
    nz_closed_loop = False
    nz_clamp_after_floor = False
    nz_max_g = 3.0
    aero_cl_alpha = 6.9503
    aero_cl_zero = 0.56381
    aero_cl_max = 3.2503
    aero_q_sigma_exp = 2.0
    _nz_lead_warned = False

    def __init__(self, d=6142.0, h=3100.0, vh=145.0, sigma_deg=15.3,
                 gam_deg=-30.0, alpha_deg=0.0, alt=None, tgo=41.3):
        """⚠️ `h` 是**相對紅心**的高度,`alt` 是 AMSL(動壓要它)。兩者刻意
        分開:L3 要在**幾何完全不動**的前提下只改 q,否則「上限隨 q 變」與
        「幾何變了所以需求變了」分不開。`_zg` 負責把兩者接起來。
        """
        self.itcg_e, self.itcg_s, self.itcg_side = {}, {}, {}
        self.mpng_k0, self.itcg_dbg, self.itcg_a = {}, {}, {}
        self.alpha, self.nz_dbg, self.alt_amsl, self.frozen = {}, {}, {}, {}
        self.sent = []
        self.log = _Log()
        # ⭐ `alpha_deg = 0` 讓現場估計的攻角 α **恰好**是 0:α_raw = θ_meas − γ,
        #   而 α[i] 的初值就是 α_raw ⇒ θ_meas = γ 時 α ≡ 0。
        #   ⚠️ 這不是為了讓測試好過:α ≠ 0 時 pitch 透過 cos(roll) 與橫向耦合,
        #     `(a+b)−b == a` 在浮點下不成立。需要耦合的鎖另外用 alpha_deg != 0。
        self.att = {1: (0.0, math.radians(gam_deg + alpha_deg), 0.0)}
        self.dive_T = 1000.0
        self.tgo = tgo
        self.T = (24.0, 121.0)
        self._h = h
        self.pos = {1: (24.0 - d / 111320.0, 121.0, h if alt is None else alt)}
        a = math.radians(sigma_deg)
        hdot = -vh * math.tan(math.radians(-gam_deg))
        self.vel = {1: (vh * math.cos(a), vh * math.sin(a), -hdot)}

    # ── 受測碼會呼叫、但不屬於本組鎖的東西 ──
    def _zg(self, i):
        """地面高程 = AMSL − 相對紅心高度 ⇒ `_geom` 的 h 恆等於建構時的 h。"""
        return self.pos[i][2] - self._h

    def _tgo_of(self, i, d):
        return self.tgo

    def _att_target(self, i, roll, pitch, thr):
        self.sent.append((i, roll, pitch, thr))

    def get_logger(self):
        return self.log

    # ── 測試用取值器(不是被測碼的一部分)──
    def now_for(self, early_s=3.7):
        return self.dive_T - (self.tgo + early_s)


#: 建構子吃的參數(其餘一律當成「跑之前先 setattr 的旋鈕」)。
_CTOR = ("d", "h", "vh", "sigma_deg", "gam_deg", "alpha_deg", "alt", "tgo")


def _guid(**kw):
    """⚠️ 每一次都給**全新**的樁:`alpha` / `frozen` / `nz_dbg` 都是跨拍鎖存的
    狀態,重用會讓後一次呼叫吃到前一次的鎖存值。
    """
    ctor = {k: kw.pop(k) for k in _CTOR if k in kw}
    s = Guid(**ctor)
    for k, v in kw.items():
        assert hasattr(Guid, k) or hasattr(s, k), f"樁沒有 {k},打錯字了"
        setattr(s, k, v)
    return s


def _run(**kw):
    """跑一拍 `_pn_cmd`,回傳 (樁, (roll, pitch, thr, R, V))。"""
    s = _guid(**kw)
    return s, s._pn_cmd(1, s.now_for())


# ══════════════════════════════════════════════════════════════════════════
#  ⭐⭐ 反解:從**送出去的** pitch 還原 α 與 n_z
# ══════════════════════════════════════════════════════════════════════════
def _q_eff(alt_amsl, v_tas, sigma_exp=2.0):
    """本拍的實效動壓(Pa)。**測試側獨立算一次**,不從被測碼拿。

    q = ½·ρ0·σ^k·V²,σ = ρ(h_AMSL)/ρ0。

    ⚠️⚠️ k = 2 不是筆誤,也不是教科書動壓。SITL 本體
       (`SIM_Aircraft.cpp:1417` 把 `airspeed` 存成 **EAS**,
        `SIM_Plane.cpp:384` 再用 `0.5*ρ(h)*airspeed²`)實得的是
       q_eff = q_true × σ。⇒ 我們飛的那個本體就是 σ² 這一支。
       用教科書 q(k=1)在 3400 m 會少指令 **0.86 g**、在地面歸零 ——
       一條隨高度的靜默梯度,剛好吃掉要買的 2 g 的 43%。
       接 Gazebo / 上實機時 k 要改,所以它是參數不是常數。
    """
    sigma = max(1e-3, (1.0 - 2.25577e-5 * max(0.0, alt_amsl)) ** _ISA_EXP)
    return 0.5 * RHO0 * (sigma ** sigma_exp) * v_tas * v_tas


def _nz_of_alpha(alpha_rad, q_pa, cl_alpha=6.9503, cl_zero=0.56381):
    """α → n_z。⚠️ **C_L0 不可省**,見檔頭。"""
    return (cl_zero + cl_alpha * alpha_rad) * q_pa * S_REF / WEIGHT


def _invert(s, roll, pitch):
    """從送出去的 (roll, pitch) 反解 (γ, α)。

    前向式 `pitch = γ + α/max(cos roll, 0.5)` ⇒ `α = (pitch − γ)·max(cos roll, 0.5)`。

    ⚠️⚠️ 反解只在**兩個夾制都沒有咬到**時成立:
        · pitch 貼 [dive_pitch_min, dive_pitch_max] ⇒ 它已經是常數,反解是垃圾;
        · roll 貼 r_lim ⇒ 前向式用的是**夾制前**的 roll(受測碼先算 pitch
          再夾 roll),回傳值反解不回去。
      垃圾在兩組之間「剛好相同」會給假綠燈 ⇒ 每一條用到反解的鎖都先驗
      `clamped is False`。
    """
    dn, de, d, h, R, vn, ve, vh, hdot, V = s._geom(1)
    gam = math.atan2(hdot, max(vh, 1e-3))
    return gam, (pitch - gam) * max(math.cos(roll), 0.5)


def _clamped(s, roll, pitch):
    th_meas = s.att[1][1]
    lo = math.radians(s.dive_pitch_min if s.dive_pitch_min < 0.0 else -80.0)
    hi = math.radians(s.dive_pitch_max)
    r_lim = math.radians(80.0) * abs(math.cos(th_meas))
    return (not (lo + 1e-9 < pitch < hi - 1e-9)) or abs(roll) >= r_lim - 1e-9


# ══════════════════════════════════════════════════════════════════════════
#  ⭐⭐ 改動**之前**的逐位元黃金值
# ══════════════════════════════════════════════════════════════════════════
#: 11 組 (名字, 旋鈕) → 改動前的 (roll, pitch, thr, R, V)。
#: ⚠️ 這些數字是 2026-08-19 22:22 那一版 `tot_node.py` 跑出來的,**不是**從
#:    現在的碼倒推的。倒推的黃金值鎖不住「這一輪有沒有動到對照組」。
_POINTS = {
    "nominal":          {},
    "floor_bites":      dict(pn_los_floor=1.0, gam_deg=-5.0, d=3100.0,
                             h=3100.0, vh=150.0),
    "floor_blows_band": dict(pn_los_floor=1.0, gam_deg=0.0, d=2000.0,
                             h=5000.0, vh=150.0),
    "shallow":          dict(gam_deg=-5.0, d=9000.0, h=3100.0),
    "steep_hi_q":       dict(gam_deg=-45.0, d=2000.0, h=2000.0, vh=170.0),
    "alpha_nonzero":    dict(alpha_deg=3.0),
    "big_sigma":        dict(sigma_deg=60.0),
    "near_roll_clamp":  dict(d=400.0, h=200.0, vh=150.0),
    "nzmin_neg":        dict(pn_nz_min=-1.0, pn_los_floor=1.0, gam_deg=-10.0),
    "low_alt_dense":    dict(gam_deg=-20.0, d=1500.0, h=500.0, vh=160.0,
                             alt=500.0),
    "high_alt_thin":    dict(gam_deg=-20.0, d=1500.0, h=500.0, vh=160.0,
                             alt=6000.0),
}

_GOLD_BEFORE = {
    "nominal": (-0.4022362992446651, -0.5150833810261228,
                0.06913859372948761, 6879.982848815963, 167.43157806499147),
    "floor_bites": (-0.6650145076050575, -0.785398163397453,
                    0.0, 4384.062043356574, 150.57297563150212),
    "floor_blows_band": (-0.8804772168635331, -1.190289949682557,
                         0.0, 5385.16480713445, 150.0),
    "shallow": (-0.2472330778922845, -0.10489122000482846,
                1.0, 9518.928511129936, 145.55387644378538),
    "steep_hi_q": (-0.987307319590748, -0.7778666148197454,
                   0.0, 2828.4271247460865, 240.41630560342614),
    "alpha_nonzero": (-0.4022362992446651, -0.4581820881259669,
                      0.06913859372948761, 6879.982848815963,
                      167.43157806499147),
    "big_sigma": (-0.9492802295659345, -0.49302546957798016,
                  0.06913859372948761, 6879.982848815963, 167.43157806499147),
    "near_roll_clamp": (-1.2091995761561452, -0.4027758650778455,
                        0.0, 447.2135954999317, 173.20508075688772),
    "nzmin_neg": (-0.3579975501196966, -0.4674177419439599,
                  0.4730329805606567, 6879.982848815963, 147.23685872343302),
    "low_alt_dense": (-1.0979377751459247, -0.3286893249043284,
                      0.0, 1581.1388300842732, 170.26844359614594),
    "high_alt_thin": (-1.0979377751459247, -0.3286893249043284,
                      0.0, 1581.1388300842732, 170.26844359614594),
}

#: LOS 地板把 γ 指令打到 −68.2° 的幾何 ⇒ 反解出來的 n_z = −17.2,
#: 遠遠在 [pn_nz_min, nz_max_g] = [0.05, 3.0] 之外。L4/L5 的主場。
_FLOOR_BLOWS = _POINTS["floor_blows_band"]


# ══════════════════════════════════════════════════════════════════════════
#  L0 介面契約:兩顆開關**預設關**
# ══════════════════════════════════════════════════════════════════════════
class TestL0TheSwitchesDefaultOff:
    """⚠️ 新開關預設關 = 逐位元同現行,是 S2 定下來的既有模式。
    預設值若哪天被翻成 True,L1 那 11 條會一起紅 —— 但先在這裡說清楚是**哪一個
    決定**變了,而不是讓人去猜 11 個數字為什麼同時動。"""

    @pytest.mark.parametrize("name", ["nz_closed_loop", "nz_clamp_after_floor"])
    def test_the_declared_default_is_false(self, name):
        assert _declared_default(name) is False, (
            f"⚠️⚠️ {name} 的 declare_parameter 預設不是 False ⇒ 對照組被污染,"
            f"從此沒有任何一趟航次是乾淨的基準線")

    @pytest.mark.parametrize("name", ["nz_closed_loop", "nz_clamp_after_floor"])
    def test_the_class_level_default_is_false(self, name):
        """⚠️ 類別層的預設是**樁節點與失敗路徑**吃的那一份。兩份必須同值,
        否則「參數沒宣告成功」時會靜靜地走另一條路。"""
        assert _class_default(name) is False, f"⚠️⚠️ 類別層 {name} 預設不是 False"

    def test_the_authority_ceiling_still_defaults_to_the_old_hardcoded_value(self):
        """`3.0` 從式子裡的字面常數變成參數 —— **預設值必須還是 3.0**。
        單變數紀律:這一輪動的是「閉迴路」,不是「抬權限」。"""
        assert _declared_default("nz_max_g") == 3.0
        assert _class_default("nz_max_g") == 3.0

    def test_the_body_constants_agree_with_the_calibrated_source(self):
        """⚠️⚠️ 質量/參考面積在 `tot_node.py` 與 `predictor.py` 兩邊各有一份
        (刻意的:`_pred` import 失敗時導引律不可以吃到另一組機體常數)。
        ⇒ 「改一邊沒改另一邊」的老坑改由**這一條**盯。它比對的是兩個模組的
           **數字**,不是哪一行長什麼樣。改任何一個都要重跑校準(n=1617)。"""
        assert _NS["AERO_MASS_KG"] == _pred.MASS_KG == 22.0
        assert _NS["AERO_S_REF_M2"] == _pred.S_REF_M2 == 0.45
        assert _NS["G0"] == GRAV

    def test_the_guidance_atmosphere_is_self_consistent(self):
        """⭐⭐ `_qbar` 的 σ 與 `_eas_of` 的 σ 必須是**同一個數**。

        ⚠️ 不一致的話,n_z↔α 的換算會與速度量測互相矛盾 —— 而兩邊都「看起來
           對」,只有把兩個 σ 反解出來並排才看得見。這一條就是那個並排。
        ⚠️ 導引律用 4.25588、`predictor.air_density` 用 4.2586,兩者本來就不同;
           所以這裡驗的是**導引律內部**一致,不是與 predictor 一致。
        """
        for alt in (0.0, 1500.0, 3400.0, 8000.0):
            s = _guid(alt=alt)
            _, _, _, _, V = s._pn_cmd(1, s.now_for())
            q = s._qbar(1, V)
            eas = s._eas_of(1)
            sig_q = (2.0 * q / (RHO0 * V * V)) ** (1.0 / s.aero_q_sigma_exp)
            sig_e = (eas / V) ** 2
            assert sig_q == pytest.approx(sig_e, rel=1e-3), (
                f"⚠️⚠️ alt={alt}:_qbar 的 σ={sig_q:.6f} 與 _eas_of 的 "
                f"σ={sig_e:.6f} 不同 ⇒ 動壓與速度量測用了兩套大氣")


# ══════════════════════════════════════════════════════════════════════════
#  L1 ⭐⭐ 開關關閉 ⇒ 逐位元同**改動前**
# ══════════════════════════════════════════════════════════════════════════
class TestL1SwitchesOffIsBitIdenticalToBefore:
    """⚠️⚠️ 這是對照組不被污染的唯一保證。

    現況 15/16 命中、觸地誤差中位 0.80 m,而**精度不在授權範圍內**
    ([[spread-outranks-terminal-speed]])。開關關著時只要有一個 bit 動了,
    往後所有「新舊比較」的結論都不成立 —— 而那種污染是靜默的:
    飛出來只會長得像「效果不如預期」。
    """

    @pytest.mark.parametrize("name", sorted(_POINTS))
    def test_every_operating_point_reproduces_the_pre_change_numbers(self, name):
        _, got = _run(**dict(_POINTS[name]))
        assert tuple(got) == _GOLD_BEFORE[name], (
            f"⚠️⚠️ 工作點 {name} 的輸出與**改動前**不同:\n"
            f"    now    = {tuple(got)}\n"
            f"    before = {_GOLD_BEFORE[name]}\n"
            f"這些黃金值是 B1 動 _pn_cmd 之前抓的。差一個 bit 就代表對照組"
            f"被污染,往後所有新舊比較都不成立。")

    def test_the_new_path_is_inert_when_off_even_with_absurd_body_constants(self):
        """⭐⭐ 反向鎖:開關關著時,把三個氣動常數換成荒謬值也**不准**動到輸出。

        ⚠️ 只驗「預設值下相同」是不夠的:那放行了「新路照算、但常數剛好讓
           它算出同一個數」以及「新舊路混在一起」兩類寫法。把常數整個打壞、
           輸出還是逐位元不變,才證明新路**真的沒有接在致動鏈上**。
        """
        junk = dict(aero_cl_alpha=1e-6, aero_cl_zero=1.0e9, aero_cl_max=1e-9,
                    aero_q_sigma_exp=0.0)
        for name, kw in _POINTS.items():
            _, got = _run(**dict(kw, **junk))
            assert tuple(got) == _GOLD_BEFORE[name], (
                f"⚠️⚠️ {name}:開關關著,但改氣動常數竟然動到了輸出 ⇒ "
                f"新路沒有真的被關掉")

    def test_the_altitude_pair_is_identical_before_and_differs_after(self):
        """⭐ `low_alt_dense` / `high_alt_thin` 是**同一組幾何、只差 AMSL**。

        舊碼裡沒有 q 這個量 ⇒ 兩點必須逐位元相同(這一半在上面已經鎖住);
        這裡鎖的是**另一半**:閉迴路開啟後它們必須不同 —— 否則 q 根本沒有
        進到指令鏈,而那正是本專案的頭號失效類別
        (「開關開著但那一段從來沒真的發生」,已第五次)。
        """
        lo, hi = _POINTS["low_alt_dense"], _POINTS["high_alt_thin"]
        assert _GOLD_BEFORE["low_alt_dense"] == _GOLD_BEFORE["high_alt_thin"]
        _, a = _run(nz_closed_loop=True, **dict(lo))
        _, b = _run(nz_closed_loop=True, **dict(hi))
        assert a[1] != b[1], (
            "⚠️⚠️ 閉迴路開啟後,500 m 與 6000 m 送出同一個 pitch ⇒ 動壓沒有"
            "進到指令鏈,這顆開關是空的")


# ══════════════════════════════════════════════════════════════════════════
#  L2 ⭐⭐ 送出去的 pitch 真的會產生所要的 n_z
# ══════════════════════════════════════════════════════════════════════════
#: 反解在浮點下的容差。α_cmd → pitch → α_cmd → n_z 是四則運算的往返,
#: 殘差量級 1e-13 g;訂 1e-9 是留兩個數量級的餘裕,而不是掩護任何近似。
NZ_TOL = 1e-9

#: L2 的工作點:刻意讓 roll 與 pitch **都離夾制很遠**(反解才成立),
#: 且覆蓋「地板咬到 / 沒咬到」「α_est=0 / α_est≠0」「n_z 下限放到負值」。
_L2_POINTS = {
    "nominal":   {},
    "shallow":   dict(gam_deg=-5.0, d=9000.0, h=3100.0, alt=3100.0),
    "alpha3":    dict(alpha_deg=3.0),
    "floor_on":  dict(pn_los_floor=1.0, gam_deg=-5.0, d=3100.0, h=3100.0,
                      vh=150.0, alt=3100.0),
    "nzmin_neg": dict(pn_nz_min=-1.0, gam_deg=-10.0),
    "sea_level": dict(gam_deg=-20.0, d=4000.0, h=1200.0, vh=150.0, alt=0.0),
}


class TestL2ThePitchProducesTheDemandedNz:
    """⭐⭐ 這一條是路徑 A 的**行為定義**。

        給定 n_z_des、q、γ、roll:
            α  = (pitch − γ)·max(cos roll, 0.5)          ← 從送出去的 pitch 反解
            n_z = (C_L0 + C_Lα·α)·q·S/W
        必須 ≈ n_z_des(容差 1e-9 g,見 NZ_TOL)。

    ⚠️ n_z_des 取的是**夾制後、地板後**的那個需求(受測碼裡的 `cmd_nz`),
       因為那才是「這一拍決定要買多少 g」。夾制前的 `req_nz` 是判定量、
       不是被追蹤量,拿它當被追蹤量會在 4.45% 的拍上假紅燈。

    ⚠️⚠️ **q 用節點自己記的那一個,不是本檔獨立算的。** 這是刻意的分解:
       · 這一組(L2)問的是「**在你用的那個 q 下**,你送的角會不會產生你要的
         n_z」—— 那是路徑 A 的行為定義,必須**逐位元**成立(1e-9 g),
         不可以被大氣模型第五位的分歧稀釋掉。
       · 「你用的 q 是不是對的 q」是**另一件事**,由 L3 的
         `test_the_ceiling_tracks_the_dynamic_pressure` 拿獨立算的 q 去釘。
       兩條合起來才是密的:單用節點的 q 會漏掉「q 全錯但自洽」,
       單用獨立的 q 會讓 L2 的容差被大氣模型綁架。
    """

    @pytest.mark.parametrize("name", sorted(_L2_POINTS))
    def test_the_round_trip_is_exact(self, name):
        s, (roll, pitch, thr, R, V) = _run(nz_closed_loop=True,
                                           **dict(_L2_POINTS[name]))
        assert not _clamped(s, roll, pitch), (
            f"⚠️ 工作點 {name} 貼到了夾制 ⇒ 反解是垃圾,這條鎖形同虛設")
        gam, alpha = _invert(s, roll, pitch)
        q = s.nz_dbg[1]["q_pa"]              # ← 節點自己用的 q,見 docstring
        nz_back = _nz_of_alpha(alpha, q, s.aero_cl_alpha, s.aero_cl_zero)
        want = s.nz_dbg[1]["cmd_nz"]
        assert nz_back == pytest.approx(want, abs=NZ_TOL), (
            f"⚠️⚠️ {name}:送出去的 pitch 反推得到 n_z = {nz_back:.6f},"
            f"但這一拍要的是 {want:.6f} ⇒ 送出去的**不是**會產生所要 n_z 的角。")

    def test_the_quiescent_demand_is_about_one_g(self):
        """量綱與組成的體檢:平穩俯衝時要的就是 ~1 g(維持 γ 而已)。
        ⚠️ 這不是精度斷言,是攔「反解湊出來剛好自洽但物理荒謬」的那一類。"""
        s, (roll, pitch, thr, R, V) = _run(nz_closed_loop=True)
        gam, alpha = _invert(s, roll, pitch)
        q = s.nz_dbg[1]["q_pa"]
        assert 0.8 < _nz_of_alpha(alpha, q, s.aero_cl_alpha,
                                  s.aero_cl_zero) < 1.3
        assert -8.0 < math.degrees(alpha) < 2.0, (
            f"⚠️ α 指令 {math.degrees(alpha):.2f}° 離實測箱(min −11.6、"
            f"p99 −1.5、max +0.4)太遠")

    @pytest.mark.parametrize("alt,floor_g", [(0.0, 18.0), (3100.0, 8.0)])
    def test_dropping_c_l_zero_would_be_a_double_digit_g_error(self, alt, floor_g):
        """⚠️⚠️ 交辦的式子 `α = n_z·W/(C_Lα·q·S)` **漏掉 C_L0**。

        C_L0/C_Lα = 4.650° 是一個與 q 無關的**常數 α 偏移**;換算成 n_z 之後
        是 `C_L0·q·S/W` —— 那個**隨 q 變**:海平面 **20.7 g**、3100 m 10.9 g。
        (交辦文字裡的「+20.7 g」是海平面那一個,不是全高度通用的數。)
        ⇒ 這條鎖不比對一個魔術數字,而是比對那個**機制式**,並確認送出去的
          α 不是漏 C_L0 的那一個。
        """
        s, (roll, pitch, thr, R, V) = _run(nz_closed_loop=True, alt=alt,
                                           d=4000.0, h=1200.0, vh=150.0,
                                           gam_deg=-20.0)
        assert not _clamped(s, roll, pitch)
        gam, alpha = _invert(s, roll, pitch)
        q = s.nz_dbg[1]["q_pa"]
        want = s.nz_dbg[1]["cmd_nz"]
        alpha_bad = want * WEIGHT / (q * S_REF) / s.aero_cl_alpha
        nz_if_bad = _nz_of_alpha(alpha_bad, q, s.aero_cl_alpha, s.aero_cl_zero)
        offset = s.aero_cl_zero * q * S_REF / WEIGHT
        assert nz_if_bad - want == pytest.approx(offset, rel=1e-9), (
            "⚠️ 漏掉 C_L0 的代價就是 C_L0·q·S/W,不多也不少")
        assert offset > floor_g, (
            f"⚠️ 這個工作點的偏移只有 {offset:.2f} g ⇒ 前提壞了,換工作點")
        assert abs(alpha - alpha_bad) > 1e-3, (
            f"⚠️⚠️ 送出去的 α 就是漏掉 C_L0 的那一個 ⇒ 指令 n_z 高估 "
            f"{offset:.1f} g。飛出去不是撕了就是整發被拒。")

    def test_zeroing_c_l_zero_moves_the_command(self):
        """⭐ 正向反向各一條:把 C_L0 設成 0,送出去的角**必須**跟著動。
        (只驗「不等於錯的那個」會放行「兩個都不對」。)"""
        sa, ga = _run(nz_closed_loop=True)
        sb, gb = _run(nz_closed_loop=True, aero_cl_zero=0.0)
        assert ga[1] != gb[1], "⚠️⚠️ C_L0 改成 0 竟然不動指令 ⇒ 它根本沒進式子"
        # ⚠️ 比的是 **α**,不是 pitch:pitch = γ + α/max(cos roll, 0.5),
        #    那個除數在 roll = −23° 時是 0.92 ⇒ 直接比 pitch 會多出 8.6%,
        #    看起來像「係數不對」,其實是忘了除回去。
        da = _invert(sa, ga[0], ga[1])[1] - _invert(sb, gb[0], gb[1])[1]
        assert math.degrees(abs(da)) == pytest.approx(
            math.degrees(sa.aero_cl_zero / sa.aero_cl_alpha), rel=1e-9), (
            "⚠️ C_L0 的貢獻應該恰好是 C_L0/C_Lα = 4.650° 的常數 α 偏移")

    def test_the_closed_loop_no_longer_feeds_forward_the_lagging_alpha(self):
        """⭐⭐ 機制鎖:新路的 pitch **不准**依賴 α 的低通估計。

        舊路 `pitch = γ_cmd + α_est`,α_est 是**當下的** α(τ=1.75 s 的落後量)
        —— 那正是「角度追蹤迴路 + 落後前饋」而不是 n_z 迴路的本體。
        ⇒ 把 α_est 的來源(θ_meas)整個換掉,新路必須逐位元不動、舊路必須動。
        """
        on = [_run(nz_closed_loop=True, alpha_deg=a)[1][1] for a in (0.0, 3.0)]
        off = [_run(nz_closed_loop=False, alpha_deg=a)[1][1] for a in (0.0, 3.0)]
        assert on[0] == on[1], (
            "⚠️⚠️ 閉迴路的 pitch 還在吃 α 的低通估計 ⇒ 它仍然是角度追蹤迴路,"
            "不是 n_z 迴路")
        assert off[0] != off[1], (
            "⚠️ 反向鎖:舊路**應該**吃 α_est,兩邊都不動代表這條鎖沒換到東西")

    def test_the_recorded_alpha_command_is_the_one_actually_sent(self):
        """⚠️ 儀表與指令脫鉤是 S2 踩過的坑(送出去的半徑與記錄下來的半徑不同)。
        `alpha_cmd_deg` 必須就是反解得到的那個 α。"""
        s, (roll, pitch, thr, R, V) = _run(nz_closed_loop=True)
        gam, alpha = _invert(s, roll, pitch)
        assert s.nz_dbg[1]["alpha_cmd"] == pytest.approx(alpha, abs=1e-12)

    def test_a_zero_lift_slope_does_not_blow_up_the_law(self):
        """⚠️ C_Lα = 0 是組態錯誤,但導引律不可以因此丟例外 —— tick 被打斷
        是**靜默的**,飛機會默默掉回 L1。"""
        s, got = _run(nz_closed_loop=True, aero_cl_alpha=0.0)
        assert all(isinstance(x, float) and math.isfinite(x) for x in got)


# ══════════════════════════════════════════════════════════════════════════
#  L3 ⭐ n_z 上限吃**機體值**(隨動壓變),不是寫死的 3.0
# ══════════════════════════════════════════════════════════════════════════
class TestL3TheCeilingComesFromTheBodyNotAConstant:
    """`n_z_max = C_Lmax·q·S/W`。俯衝段實測 min 40.0 / 中位 **120.4** /
    p95 163.0 g ⇒ 寫死的 3.0 只是氣動天花板的 **2.49%**。

    ⚠️ 設計上的分工(單變數紀律):`nz_max_g` 是**權限**上界(這一輪不動,
       預設仍 3.0),`nz_max_true` 是**物理**硬停(失速是機體事實)。
       閉迴路開啟時取兩者的 min ⇒ 抬 `nz_max_g` 時,咬住的就換成機體值。
    """

    _GEO = dict(d=1500.0, h=500.0, vh=160.0, gam_deg=-20.0)

    @pytest.mark.parametrize("alt", [0.0, 3100.0, 6000.0, 15000.0])
    def test_the_ceiling_tracks_the_dynamic_pressure(self, alt):
        s, (roll, pitch, thr, R, V) = _run(nz_closed_loop=True, alt=alt,
                                           **dict(self._GEO))
        q = _q_eff(alt, V, s.aero_q_sigma_exp)      # ← 本檔**獨立**算的 q
        want = s.aero_cl_max * q * S_REF / WEIGHT
        assert s.nz_dbg[1]["q_pa"] == pytest.approx(q, rel=Q_REL), (
            f"⚠️⚠️ alt={alt}:節點算的動壓 {s.nz_dbg[1]['q_pa']:.2f} Pa 與獨立"
            f"算的 {q:.2f} Pa 不同 ⇒ 大氣約定不一致。少乘一個 σ 在 3400 m "
            f"是 0.86 g 的靜默梯度,而它從指令上完全看不出來。")
        assert s.nz_dbg[1]["nz_max_true"] == pytest.approx(want, rel=Q_REL)

    def test_the_ceiling_is_not_the_hardcoded_three(self):
        """⚠️ 反向鎖:機體上限在實際工作點上必須是**兩位數以上**的 g。
        如果它剛好等於 3.0,那 L3 整組就只是在鎖一個常數。"""
        s, _ = _run(nz_closed_loop=True, alt=3100.0, **dict(self._GEO))
        assert s.nz_dbg[1]["nz_max_true"] > 30.0

    def test_halving_the_stall_coefficient_halves_the_ceiling(self):
        """⭐ 比例鎖:上限對 C_Lmax 與 q 都是**線性**的。
        改一個常數、上限跟著等比例動 —— 這排除了「算出來但用了別的數」。"""
        a = _run(nz_closed_loop=True, alt=3100.0,
                 **dict(self._GEO))[0].nz_dbg[1]["nz_max_true"]
        b = _run(nz_closed_loop=True, alt=3100.0, aero_cl_max=3.2503 / 2.0,
                 **dict(self._GEO))[0].nz_dbg[1]["nz_max_true"]
        assert b == pytest.approx(a / 2.0, rel=1e-12)

    def test_the_body_ceiling_actually_binds_the_command(self):
        """⭐⭐ 「算出來」與「咬得到」是兩件事(本專案頭號失效類別)。

        把 `nz_max_g` 抬到 500(讓權限上界不再是咬住的那一個)、把 C_Lmax
        壓到 0.05(讓機體上限落在需求附近),然後:
          · 海平面(q 大)⇒ 上限 1.85 > 需求 1.29 ⇒ **不**咬 ⇒ cmd == req;
          · 15 km(q 小)⇒ 上限 0.055 < 需求 1.29 ⇒ 咬 ⇒ cmd == 上限。
        同一個需求、同一個幾何,只有 q 不同 ⇒ 分岔只能來自 q。
        """
        kw = dict(nz_closed_loop=True, nz_max_g=500.0, aero_cl_max=0.05,
                  **self._GEO)
        lo = _run(alt=0.0, **kw)[0].nz_dbg[1]
        hi = _run(alt=15000.0, **kw)[0].nz_dbg[1]
        assert lo["req_nz"] == pytest.approx(hi["req_nz"], rel=1e-12), (
            "⚠️ 前提壞了:兩點的需求必須一模一樣,分岔才只能來自 q")
        assert lo["nz_max_true"] > lo["req_nz"], "⚠️ 低空這一點應該不咬"
        assert lo["cmd_nz"] == pytest.approx(lo["req_nz"], rel=1e-12)
        assert hi["nz_max_true"] < hi["req_nz"], "⚠️ 高空這一點應該要咬"
        assert hi["cmd_nz"] == pytest.approx(hi["nz_max_true"], rel=1e-12), (
            f"⚠️⚠️ 高空需求 {hi['req_nz']:.3f} 超過機體上限 "
            f"{hi['nz_max_true']:.3f},卻沒有被它咬住 ⇒ 機體上限只是一欄擺設")

    def test_the_body_ceiling_is_not_applied_while_the_switch_is_off(self):
        """⚠️ 關閉時**不套用**機體上限 —— 低速近地時 nz_max_true 可能 < 3.0,
        套了就不是逐位元同現行了。這是 L1 的一部分,在這裡正面寫出理由。"""
        kw = dict(nz_max_g=3.0, aero_cl_max=1e-6, **self._GEO)
        s, got = _run(nz_closed_loop=False, alt=15000.0, **kw)
        assert s.nz_dbg[1]["nz_max_true"] < 0.01, "⚠️ 前提壞了,上限沒被壓低"
        assert s.nz_dbg[1]["cmd_nz"] == pytest.approx(
            s.nz_dbg[1]["req_nz"], rel=1e-12), (
            "⚠️⚠️ 開關關著,機體上限卻咬到了指令 ⇒ 對照組被污染")

    def test_the_density_exponent_is_a_parameter_not_a_constant(self):
        """⚠️⚠️ SITL 本體實得 q_eff = q_true × σ(`SIM_Aircraft.cpp:1417` 把
        airspeed 存成 EAS)。用教科書 q(σ^1)在 3400 m 會少指令 **0.86 g**、
        在地面歸零 —— 一條隨高度的靜默梯度。接 Gazebo/實機時它要改,
        所以必須是參數。"""
        a = _run(nz_closed_loop=True, alt=3400.0, aero_q_sigma_exp=2.0,
                 **dict(self._GEO))
        b = _run(nz_closed_loop=True, alt=3400.0, aero_q_sigma_exp=1.0,
                 **dict(self._GEO))
        assert a[0].nz_dbg[1]["q_pa"] != b[0].nz_dbg[1]["q_pa"]
        assert a[1][1] != b[1][1], (
            "⚠️⚠️ 改 aero_q_sigma_exp 不動指令 ⇒ 這個參數是擺設,而它正是"
            "接 Gazebo/實機時唯一能事後抓到用錯 q 的地方")
        # 地面 σ=1 ⇒ 兩種約定必須重合(反向鎖:證明差異真的來自 σ 的次方)
        c = _run(nz_closed_loop=True, alt=0.0, aero_q_sigma_exp=2.0,
                 **dict(self._GEO))
        d = _run(nz_closed_loop=True, alt=0.0, aero_q_sigma_exp=1.0,
                 **dict(self._GEO))
        assert c[1][1] == d[1][1]


# ══════════════════════════════════════════════════════════════════════════
#  L4 ⭐⭐ 順序 bug:LOS 地板的輸出繞過了 [nz_min, nz_max] 夾制
# ══════════════════════════════════════════════════════════════════════════
class TestL4TheFloorNoLongerBypassesTheClamp:
    """現行順序是「先夾 n_z、再取 LOS 地板的 min」⇒ 地板的輸出**完全繞過**夾制:
    地板是一個**角度**否決,夾制卻是**過載**預算,中間沒有換算。

    後果:
      (a) 事後把 (γ_cmd − γ)/pn_lead 當角速率需求會讀到 −61 °/s 的假值
          (那是 −61.13° 的**角度階躍**除以 1 s)⇒「追蹤增益 0.063」這個
          結論是**記帳假象**,不是物理;
      (b) `pn_nz_min` 名義上存在卻管不到真正送出去的指令:實得 n_z 在
          **14/14 架次**都跌破 −1.0(合計 35.15 s、最低 −3.12 g),
          而 +3.0 那側只有 1/14 越界 —— min() 只會更負,不對稱完全吻合。

    ── 兩種修法選哪一個,以及為什麼 ─────────────────────────────────────
    可選 (i) 把地板搬到夾制**之前**,或 (ii) 地板之後**再夾一次**。
    採用 (ii)。理由是**可逆性與可觀測性**:

      · 地板是唯一「不看能量預算、只看幾何」的否決條款,它的存在理由是
        「比視線淺 = 會從目標上方飛過去」。把它搬到夾制前,等於讓夾制有權
        把它否決掉的東西再放回來 —— 地板就不再是硬否決,語意變了。
      · 而 (ii) 只是在同一條 γ 指令上再走一次「反解 n_z → 夾 → 正解回 γ」,
        地板的否決仍然成立,只是被限制在能量預算內。
      · 更要緊的是:(ii) 讓 `cmd_nz` 這一欄在**兩種模式下都由最終的 γ 指令
        反解**,於是開關關著時它也是誠實的 —— 上面那組「14/14 跌破 −1.0」
        的數字正是這樣量出來的。搬順序的寫法會讓那一欄在開關兩側量的不是
        同一個量,新舊就沒得比。

    ⚠️ (ii) 需要 `pn_lead > 0`(γ ↔ n_z 的映射靠它才可逆)。生產值是 1.0;
       pn_lead = 0 時這顆開關不生效,而且**必須吼**(見最後一條鎖)。
    """

    _BAND = (0.05, 3.0)          # [pn_nz_min, nz_max_g] 的生效值

    def test_the_bug_is_reproducible_with_the_switch_off(self):
        """⚠️ 先證明**病在**:關著時反解出來的 n_z 必須真的在 band 外。
        病不在的話,下面那條「修好了」是在鎖一個不存在的東西。"""
        s, _ = _run(nz_clamp_after_floor=False, **dict(_FLOOR_BLOWS))
        cmd = s.nz_dbg[1]["cmd_nz"]
        assert cmd < self._BAND[0], (
            f"⚠️ 這個幾何沒有把 n_z 打出 band(cmd_nz = {cmd:.3f})⇒ 換幾何,"
            f"別放寬斷言")
        assert cmd < -10.0, f"⚠️ 預期是 −17 量級的荒謬值,實得 {cmd:.3f}"

    @pytest.mark.parametrize("closed", [False, True])
    def test_the_clamp_bites_when_the_switch_is_on(self, closed):
        s, _ = _run(nz_clamp_after_floor=True, nz_closed_loop=closed,
                    **dict(_FLOOR_BLOWS))
        cmd = s.nz_dbg[1]["cmd_nz"]
        assert self._BAND[0] - 1e-12 <= cmd <= self._BAND[1] + 1e-12, (
            f"⚠️⚠️ 開關開著,地板的輸出還是跑出 band:cmd_nz = {cmd}")
        assert cmd == pytest.approx(self._BAND[0], abs=1e-12), (
            "⚠️ 這個幾何是往下越界,應該剛好被 pn_nz_min 接住")

    @pytest.mark.parametrize("closed", [False, True])
    def test_turning_the_switch_on_actually_moves_the_command(self, closed):
        """⚠️ 「夾住了」與「動到指令了」是兩件事。夾完之後 γ 指令必須被
        **正解回去**,否則夾制只改了那一欄記錄,飛機照舊。"""
        off = _run(nz_clamp_after_floor=False, nz_closed_loop=closed,
                   **dict(_FLOOR_BLOWS))[1]
        on = _run(nz_clamp_after_floor=True, nz_closed_loop=closed,
                  **dict(_FLOOR_BLOWS))[1]
        assert off[1] != on[1], (
            "⚠️⚠️ 夾制只寫進了儀表,沒有回寫到 γ 指令 ⇒ 送出去的角沒變")
        assert on[1] > off[1], "夾住負向越界 ⇒ 指令俯角必須變**淺**"

    def test_the_switch_is_inert_where_the_floor_never_bites(self):
        """⭐ 反向鎖:地板沒咬到的工作點上,這顆開關必須**逐位元**不動輸出。
        (會動的話,它就不只是在修地板,而是在改整條律。)"""
        for name in ("nominal", "shallow", "steep_hi_q", "big_sigma"):
            a = _run(**dict(_POINTS[name]))[1]
            b = _run(nz_clamp_after_floor=True, **dict(_POINTS[name]))[1]
            assert tuple(a) == tuple(b), f"⚠️⚠️ {name} 沒有地板卻被動到了"

    def test_the_upper_edge_is_enforced_upstream_and_tracks_the_parameter(self):
        """⚠️ 地板是 `min()` ⇒ 它**只會讓 γ 指令更負**,post-floor 的 n_z 只能
        往**下**越界。實測完全吻合:14/14 架次跌破 −1.0(合計 35.15 s、
        最低 −3.12 g),而 +3.0 那側只有 1/14 —— 不對稱不是巧合,是 min() 的
        結構後果。⇒ post-floor 夾制的**上緣在結構上碰不到**,不要為它寫一條
        永遠不會失敗的鎖(那種鎖比沒有更糟:它會讓人以為上緣被守著)。

        上緣真正生效的位置是**地板之前**的那一次夾制。這裡驗那一個,
        並且順便釘死「3.0 不再是寫死的字面常數」。
        """
        hi = _run(n_vert=200.0)[0].nz_dbg[1]           # 需求推到 6.68 g
        assert hi["req_nz"] > 3.0, "⚠️ 前提壞了:需求沒有超過上緣"
        assert hi["cmd_nz"] == pytest.approx(3.0, abs=1e-12)
        for cap in (1.5, 2.0, 5.0):
            got = _run(n_vert=200.0, nz_max_g=cap)[0].nz_dbg[1]
            assert got["cmd_nz"] == pytest.approx(cap, abs=1e-12), (
                f"⚠️⚠️ nz_max_g={cap} 卻夾在 {got['cmd_nz']} ⇒ 上緣還是寫死的")
            assert got["req_nz"] == pytest.approx(hi["req_nz"], rel=1e-12), (
                "⚠️ 判定量 req_nz 不可以被夾制參數動到")

    def test_a_zero_lead_is_loud_instead_of_silently_doing_nothing(self):
        """⚠️⚠️ pn_lead = 0 ⇒ γ ↔ n_z 沒有可逆映射 ⇒ 這顆開關**不會生效**。
        本專案的招牌失效是「開關開著但那一段從來沒真的發生」(已第五次)⇒
        踩到就要吼,而且不可以丟例外把 tick 打斷。"""
        s, got = _run(nz_clamp_after_floor=True, pn_lead=0.0,
                      **dict(_FLOOR_BLOWS))
        assert all(math.isfinite(x) for x in got), "⚠️ 導引律不可以因此爆掉"
        assert any("pn_lead" in m for m in s.log.errors), (
            "⚠️⚠️ pn_lead=0 讓開關失效卻沒有任何一行告警 ⇒ 靜默失敗")
        # 只吼一次(每拍吼會把 log 淹掉,反而看不見別的事)
        n1 = len(s.log.errors)
        s._pn_cmd(1, s.now_for())
        assert len(s.log.errors) == n1, "⚠️ 這個告警必須只吼一次"


# ══════════════════════════════════════════════════════════════════════════
#  L5 兩顆開關獨立:四種組合各自可達、行為各自不同
# ══════════════════════════════════════════════════════════════════════════
class TestL5TheTwoSwitchesAreIndependent:
    """⚠️ 兩顆開關若在實作裡互相隱含(例如閉迴路自動打開夾制),往後任何
    A/B 都變成雙變數實驗 —— 本專案有一半被推翻的診斷是這樣壞掉的
    ([[stot-refuted-changes]])。
    """

    _COMBOS = list(itertools.product((False, True), (False, True)))

    def test_all_four_combinations_give_four_different_commands(self):
        out = {}
        for cl, caf in self._COMBOS:
            _, got = _run(nz_closed_loop=cl, nz_clamp_after_floor=caf,
                          **dict(_FLOOR_BLOWS))
            out[(cl, caf)] = got[1]
        assert len(set(out.values())) == 4, (
            f"⚠️⚠️ 四種組合沒有四個不同的 pitch ⇒ 兩顆開關不獨立:{out}")

    def test_the_off_off_corner_is_still_the_pre_change_behaviour(self):
        _, got = _run(nz_closed_loop=False, nz_clamp_after_floor=False,
                      **dict(_FLOOR_BLOWS))
        assert tuple(got) == _GOLD_BEFORE["floor_blows_band"]

    def test_each_switch_moves_the_command_on_its_own(self):
        """⭐ 單獨開任一顆都必須動;兩顆的效果也不可以互相抵銷成原樣。"""
        base = _run(**dict(_FLOOR_BLOWS))[1][1]
        only_cl = _run(nz_closed_loop=True, **dict(_FLOOR_BLOWS))[1][1]
        only_caf = _run(nz_clamp_after_floor=True, **dict(_FLOOR_BLOWS))[1][1]
        both = _run(nz_closed_loop=True, nz_clamp_after_floor=True,
                    **dict(_FLOOR_BLOWS))[1][1]
        assert only_cl != base and only_caf != base and both != base
        assert both != only_cl and both != only_caf

    def test_the_clamp_switch_does_not_change_the_pre_clamp_demand(self):
        """⚠️ `req_nz` 是**判定量**(夾制前的需求)。它必須在四種組合下
        逐位元相同 —— 會動的話,那一欄就不再是「權限有沒有被要求」的證據。"""
        vals = set()
        for cl, caf in self._COMBOS:
            s, _ = _run(nz_closed_loop=cl, nz_clamp_after_floor=caf,
                        **dict(_FLOOR_BLOWS))
            vals.add(s.nz_dbg[1]["req_nz"])
        assert len(vals) == 1, f"⚠️⚠️ req_nz 被開關動到了:{vals}"


# ══════════════════════════════════════════════════════════════════════════
#  L6 終端凍結不受影響(精度的保險絲,不准動)
# ══════════════════════════════════════════════════════════════════════════
class TestL6TheTerminalFreezeStillHolds:
    """⚠️⚠️ t_go ≤ fz_tgo 或 R ≤ fz_r 時鎖住 roll/pitch,是命中精度的保險絲
    (PN 的 λ̇ 在 d→0 時分母爆掉)。這一輪**不准**動它。

    ⚠️ 凍結只鎖 roll/pitch,**不鎖油門** —— 驗收第一優先量的是觸地速度,
       連油門一起凍住等於滑行進紅心。這一條也一起鎖住。
    """

    #: t_go = R/V ≈ 0.35 s < fz_tgo=0.5 ⇒ 一進來就凍。
    _TERM = dict(d=50.0, h=20.0, vh=150.0, gam_deg=-20.0)

    @pytest.mark.parametrize("cl,caf", list(itertools.product((False, True),
                                                             (False, True))))
    def test_the_command_is_frozen_after_the_first_terminal_tick(self, cl, caf):
        s = _guid(nz_closed_loop=cl, nz_clamp_after_floor=caf, **self._TERM)
        assert s._pn_step(1, s.now_for()) is True
        first = s.sent[-1]
        # 幾何整個換掉(而且是會讓未凍結的指令大幅改變的方向)
        s.pos[1] = (s.T[0] - 30.0 / 111320.0, s.T[1] + 20.0 / 111320.0, 12.0)
        s.vel[1] = (100.0, 90.0, 60.0)
        assert s._pn_step(1, s.now_for()) is True
        second = s.sent[-1]
        assert second[1] == first[1] and second[2] == first[2], (
            f"⚠️⚠️ 終端凍結失效:roll/pitch 在凍結後還在動 "
            f"{first[1:3]} → {second[1:3]}")

    @pytest.mark.parametrize("cl", [False, True])
    def test_the_throttle_is_not_frozen(self, cl):
        s = _guid(nz_closed_loop=cl, **self._TERM)
        s._pn_step(1, s.now_for())
        s.vel[1] = (40.0, 5.0, 12.0)            # 掉速 ⇒ 油門必須跟著催
        s._pn_step(1, s.now_for())
        assert s.sent[-1][3] != s.sent[0][3], (
            "⚠️⚠️ 油門被一起凍住了 ⇒ 最後 fz_r 公尺用「凍結當下剛好是多少」"
            "的油門飛完,而驗收第一優先量的正是觸地速度")

    @pytest.mark.parametrize("cl", [False, True])
    def test_the_frozen_value_is_the_first_terminal_command(self, cl):
        """⚠️ 凍的必須是**進入凍結那一拍**的指令,不是別的什麼值。"""
        s = _guid(nz_closed_loop=cl, **self._TERM)
        direct = s._pn_cmd(1, s.now_for())
        s2 = _guid(nz_closed_loop=cl, **self._TERM)
        s2._pn_step(1, s2.now_for())
        assert s2.sent[0][1] == direct[0] and s2.sent[0][2] == direct[1]

    def test_the_freeze_thresholds_are_still_the_production_values(self):
        """⚠️ 保險絲的門檻本身也是被保護的量:改了它等於改了精度。
        ⚠️ 樁用的就是這兩個生效值(1.5 s / 100 m),不是別的什麼數。"""
        assert _declared_default("pn_freeze_tgo") == 1.5 == Guid.fz_tgo
        assert _declared_default("pn_freeze_r") == 100.0 == Guid.fz_r


# ══════════════════════════════════════════════════════════════════════════
#  L7 橫向零影響(保護已達標的 15/16 命中)
# ══════════════════════════════════════════════════════════════════════════
class TestL7TheLateralChannelIsUntouched:
    """⚠️⚠️ 縱向管角度、側向管時間 —— 這是兩篇獨立文獻的同一結論,也是本專案
    的分工。這一輪動的是縱向;roll 只要動一個 bit,已達標的命中誤差
    (中位 0.80 m)就進了實驗變因裡。
    """

    @pytest.mark.parametrize("name", sorted(_POINTS))
    @pytest.mark.parametrize("cl,caf", list(itertools.product((False, True),
                                                             (False, True))))
    def test_roll_is_bit_identical_across_all_four_combinations(
            self, name, cl, caf):
        _, got = _run(nz_closed_loop=cl, nz_clamp_after_floor=caf,
                      **dict(_POINTS[name]))
        assert got[0] == _GOLD_BEFORE[name][0], (
            f"⚠️⚠️ {name} / (closed={cl}, clamp={caf}):roll 變了 "
            f"{_GOLD_BEFORE[name][0]} → {got[0]}")

    @pytest.mark.parametrize("name", sorted(_POINTS))
    def test_the_lateral_itcg_bias_is_untouched(self, name):
        """⚠️ 連 ITCG 疊在橫向的偏置也不准動 —— 它與縱向共用 `_pn_cmd`,
        是最容易被順手改到的東西。"""
        a = _run(**dict(_POINTS[name]))[0].itcg_a[1]
        b = _run(nz_closed_loop=True, nz_clamp_after_floor=True,
                 **dict(_POINTS[name]))[0].itcg_a[1]
        assert a == b

    @pytest.mark.parametrize("name", sorted(_POINTS))
    def test_the_throttle_channel_is_untouched(self, name):
        """⚠️ 油門是唯一的命中時刻通道。縱向改法不可以順手改到它,
        否則散布(現況 2.47 s,門檻 3.0)會跟著漂。"""
        for cl, caf in itertools.product((False, True), (False, True)):
            _, got = _run(nz_closed_loop=cl, nz_clamp_after_floor=caf,
                          **dict(_POINTS[name]))
            assert got[2] == _GOLD_BEFORE[name][2], (
                f"⚠️⚠️ {name} / (closed={cl}, clamp={caf}):油門變了")


# ══════════════════════════════════════════════════════════════════════════
#  L8 CSV 六個新欄:位置、順序、空字串紀律
# ══════════════════════════════════════════════════════════════════════════
class _Cap:
    """把 writerow 錄下來(不落地,免得測試互相踩檔)。"""

    def __init__(self):
        self.rows = []

    def writerow(self, row):
        self.rows.append(list(row))


class Rec(_LOG):
    """記錄樁:只提供 `_open_csv` / `_row` 真的會碰到的東西。"""

    def __init__(self, tmpdir):
        self._dir = tmpdir
        self._csv, self._cw = None, _Cap()
        self.t0, self.phase = 0.0, "DIVE"
        self.ground_alt, self.target_amsl = 0.0, 0.0
        self._agl_datum_logged = True
        self.pos = {1: (24.0, 121.0, 3100.0)}
        self.vel = {1: (140.0, 10.0, 80.0)}
        self.gs, self.last_v = {1: 150.0}, {1: 160.0}
        self.pred, self.e_t, self.weave_theta, self.tgo_cache = {}, {}, {}, {}
        self.itcg_e, self.itcg_a, self.itcg_s, self.itcg_dbg = {}, {}, {}, {}
        self.req_radius, self.sent_radius = {}, {}
        # ⭐ 2026-08-25 的三組儀表(_row 尾端的能量五欄會讀)。
        # ⚠️ Rec 不是 TotNode 的子類 ⇒ 吃不到那邊的類別層級後備,
        #    必須自己給。空 dict 正好是「這一拍沒量到」的正確表示。
        self.vfr, self.ack_bad, self._wv_aim = {}, {}, {}
        # ⭐ 拒收來源分流(2026-08-25)。
        self.ack_bad_own = {}
        # ⭐ 指令高度(2026-08-25)。Rec 不是 TotNode 的子類,
        #    吃不到類別層級後備,必須自己給。
        self.alt_cmd = {}
        self.wp_pt, self.tgo_obs = {}, {}
        self.nz_dbg = {}

    # ── 與本組鎖無關的副作用一律做成 no-op ──
    def _predict_tick(self, i, now):
        pass

    def _observe_timing(self, i, now):
        pass

    def _kill_arm_update(self):
        pass

    def _kill_airframe(self, i, agl, now):
        pass

    def _agl(self, i):
        return 100.0

    def _zg(self, i):
        return 0.0

    def _lead_angle_deg(self, i):
        return 12.3

    def _csv_base(self):
        return str(pathlib.Path(self._dir) / "trial")

    def get_logger(self):
        return _Log()


class TestL8TheCsvColumnContract:
    """⚠️⚠️ **只能尾端追加。** `_tot_plot.py` / `_dive_plot.py` 用 `r[0..8]`
    位置解析,插在中間會把每一欄靜靜地平移一格 —— 圖照樣畫得出來,只是全錯。

    ⚠️⚠️ **未設值寫空字串不寫 0。** 0 是**合法的**過載/攻角值,拿它當「沒跑」
       會把「PN 這一拍根本沒跑」誤讀成「剛好不需要過載」——
       `itcg_accel` 80,760 樣本全零那一批就是這樣被看了很久才發現的。
    """

    #: 六欄,順序即契約。
    NEW = ["req_nz", "cmd_nz", "nz_max_true",
           "alpha_cmd_deg", "alpha_est_deg", "q_pa"]
    #: 上一輪(MPNG)追加的六欄 —— 新欄必須排在它們**後面**。
    PREV6 = ["itcg_req_a", "itcg_cmd_a", "itcg_eps_t",
             "itcg_tgo_ref", "itcg_tgo_jiang", "itcg_sat"]
    #: ⚠️⚠️ 2026-08-22:「2-lite」橫向六欄**再追加在 NEW 之後** ⇒ 這個
    #:    class 的所有尾端定位一律後移六格(NEW 從 [-6:] 變成 [-12:-6])。
    #:    鎖的語意沒有變:只准尾端追加、組內順序固定、未設值寫空字串。
    LAT6 = ["a_h_cmd_g", "roll_cmd_deg", "roll_out_deg",
            "n_req_tot", "n_cmd_tot", "lat_share"]
    #: 尾端額外欄數,之後再追加時只改這一個數。
    #: ⚠️ 2026-08-23:T3b 的 weave_early_rel_s 再追加一欄 ⇒ 6 -> 7。
    #: ⚠️ 2026-08-24:①②③ 的橫向實得側/蛇形窗口/配速殘差九欄 ⇒ 7 -> 16。
    #:    (roll_meas_deg chi_deg chi_dot_meas_dps chi_dot_kin_dps chi_dt_s
    #:     weave_window_s weave_rev_n pace_eta_resid_s r_to_target_m)
    #:    ⭐ 這一個數字就是這個 class 的擴充點,而它擋得住的正是最危險的
    #:      那件事:有人把新欄**插進中間**而不是追加。插中間的話這裡的每一個
    #:      切片都會對到別欄,而每一欄都還是個合理的數字。
    #: ⚠️ 2026-08-24(同日第二次):ALIGN 段的 align_early_s / align_vreq_mps
    #:    再追加兩欄 ⇒ 16 -> 18。那一段以前**整段沒有儀表** —— 12,618 個
    #:    mode=weave 拍裡六個相關欄位全部 0%。
    #: ⚠️ 2026-08-25:能量 + 指令 ACK + 蛇形瞄準點五欄再追加 ⇒ 18 -> 23。
    #:    (thr_fc eas_fc climb_fc ack_bad_n weave_aim_m)
    #:    ⭐ 依本 class 的設計「之後再追加時只改這一個數」——
    #:      它擋得住的正是最危險的那件事:有人把新欄**插進中間**。
    #: ⚠️ 2026-08-25:指令高度 alt_cmd 再追加一欄 ⇒ 23 -> 24。
    #:    ⭐ 依本 class 的設計「之後再追加時只改這一個數」。
    #: ⚠️ 2026-08-25:ack_bad_own 再追加一欄 ⇒ 24 -> 25。
    #: ⚠️ 2026-08-25:AUTO 預規劃航線的兩欄再追加 ⇒ 25 -> 27。
    #:    (fc_mode_n mission_state)
    #:    ⭐ fc_mode_n = HEARTBEAT.custom_mode。本檔先前收了 HEARTBEAT 卻
    #:      只有一個選配閘門在讀它 ⇒ 「飛機到底在哪個模式」從來沒進過資料,
    #:      而本專案至少四次事故的根因都是模式不對(非 GUIDED 之下
    #:      DO_REPOSITION **整發被丟且零告警**)。
    #:    ⭐ mission_state = "<state>:<已送>/<總數>",分得出「航線沒上傳成功」
    #:      與「上傳成功但飛控沒照飛」—— 兩者的下一步完全相反。
    TAIL_AFTER_NEW = 27
    #: 位置解析的消費者吃的前九欄 —— 這九欄的**位置**是硬契約。
    HEAD9 = ["t", "sysid", "lat", "lon", "alt", "gs", "d_slot", "mode", "v_cmd"]

    @staticmethod
    def _header(tmp_path):
        s = Rec(tmp_path)
        s._open_csv()
        s._csv.close()
        with open(s._csv_base() + ".csv", newline="") as fh:
            return next(csv.reader(fh))

    def test_the_six_new_columns_are_the_last_six_in_order(self, tmp_path):
        hdr = self._header(tmp_path)
        _n, _t = len(self.NEW), self.TAIL_AFTER_NEW
        assert hdr[-(_n + _t):-_t] == self.NEW, (
            f"⚠️⚠️ 六個新欄不在該在的位置或順序不對:{hdr[-14:]}")
        # ⚠️ 2026-08-23:T3b 追加一欄 ⇒ LAT 也不再貼著尾巴,改用位置切片。
        _l = len(self.LAT6)
        assert hdr[-_t:-(_t - _l)] == self.LAT6, (
            f"⚠️⚠️ LAT 六欄不在該在的位置或順序不對:{hdr[-_t:-(_t - _l)]}")

    def test_they_come_after_the_previous_tail_block(self, tmp_path):
        hdr = self._header(tmp_path)
        _n, _t = len(self.NEW), self.TAIL_AFTER_NEW
        _p = len(self.PREV6)
        assert hdr[-(_p + _n + _t):-(_n + _t)] == self.PREV6, (
            "⚠️⚠️ 新欄插到了 MPNG 那六欄的**前面** ⇒ 那一批的判讀腳本會整批平移")

    def test_the_positional_prefix_is_untouched(self, tmp_path):
        assert self._header(tmp_path)[:9] == self.HEAD9, (
            "⚠️⚠️ 位置解析的前九欄被改動 ⇒ _tot_plot.py / _dive_plot.py "
            "會靜靜地畫出全錯的圖")

    def test_no_column_name_is_duplicated(self, tmp_path):
        hdr = self._header(tmp_path)
        dup = sorted(c for c in set(hdr) if hdr.count(c) > 1)
        assert not dup, f"⚠️ 重複欄名 {dup} ⇒ DictReader 只留最後一個"

    def test_the_row_width_matches_the_header(self, tmp_path):
        hdr = self._header(tmp_path)
        s = Rec(tmp_path)
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        assert len(s._cw.rows[0]) == len(hdr), (
            f"⚠️⚠️ 表頭 {len(hdr)} 欄、資料列 {len(s._cw.rows[0])} 欄 ⇒ "
            f"DictReader 會靜靜地把尾巴丟掉或塞 None")

    #: ⭐ 尾端**豁免**「未跑寫空字串」規矩的欄位,以及豁免的理由。
    #:
    #: ⚠️⚠️ 這份名單只收**累計計數器** —— 從第一拍就有明確定義的值、
    #:    永遠不會處於「沒量到」的狀態。逐拍儀表**一律不准**進來:
    #:    那條規矩守的正是「這一拍沒跑」與「跑了但值是零」的差別,
    #:    而那兩者的下一步完全相反。
    #:
    #: ack_bad_n:被飛控整發拒收的指令累計數。0 = 到目前為止沒被拒過。
    #:    寫 "" 會讓「乾淨的航次」與「舊版根本沒有這一欄」長得一模一樣,
    #:    而這一欄存在的理由就是要能一眼看出「這一趟有沒有被拒過」。
    #: ⚠️ 2026-08-25:ack_bad_own 同類 —— 只算**本節點**(source 233/234)的
    #:    拒收。0 = 我們的指令一次都沒被拒,是有意義的值。
    #:    (ack_bad_n 是整條分流鏈路的;兩欄之差 = 別層送的。)
    COUNTER_COLUMNS = {"ack_bad_n", "ack_bad_own"}

    def test_an_unrun_channel_writes_empty_strings_not_zeros(self, tmp_path):
        """⭐⭐ PN 這一拍沒跑(ALIGN/PERCH 段就是這樣)⇒ 尾端必須全是 `''`。

        ⚠️ 2026-08-25:改成**指名**豁免累計計數器,不再是位置式全稱斷言。
           這比原本強:說得出誰被豁免、為什麼,而且新加的欄忘了寫空字串
           照樣會紅。
        """
        hdr = self._header(tmp_path)
        s = Rec(tmp_path)
        s._row(10.0, 1, 500.0, "align", (24.0, 121.0))
        _w = len(self.NEW) + self.TAIL_AFTER_NEW
        tail = s._cw.rows[0][-_w:]
        names = hdr[-_w:]
        assert len(names) == len(tail), "表頭與資料列尾端寬度不一致"
        bad = [(c, v) for c, v in zip(names, tail)
               if c not in self.COUNTER_COLUMNS and v != ""]
        assert not bad, (
            f"⚠️⚠️ 通道沒跑卻寫了 {bad} —— 只要其中一個是 0,事後判讀就會把"
            f"「PN 根本沒跑」讀成「剛好不需要過載」")
        # ⭐ 反向:豁免名單裡的欄**必須真的有值**,否則豁免就是掩護一個 bug。
        for c, v in zip(names, tail):
            if c in self.COUNTER_COLUMNS:
                assert v != "", (
                    f"{c} 在豁免名單裡卻寫了空字串 —— 那它就不該被豁免")

    def test_alpha_cmd_is_empty_while_the_closed_loop_is_off(self, tmp_path):
        """⚠️ `alpha_cmd_deg` 只在閉迴路開啟時存在。關著時是**空字串**不是 0
        —— 0 是完全合法的攻角。其餘五欄照常寫數字(它們兩種模式都算)。"""
        s = Rec(tmp_path)
        s.nz_dbg = {1: {"req_nz": 0.761, "cmd_nz": -3.1234,
                        "nz_max_true": 120.44, "q_pa": 9260.16,
                        "alpha_cmd": None, "alpha_est": math.radians(-3.2)}}
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        _n, _t = len(self.NEW), self.TAIL_AFTER_NEW
        req, cmd, nzmax, a_cmd, a_est, q = s._cw.rows[0][-(_n + _t):-_t]
        assert a_cmd == "", "⚠️⚠️ 閉迴路關著,alpha_cmd_deg 必須是空字串"
        assert req == pytest.approx(0.761, abs=5e-4)
        assert cmd == pytest.approx(-3.123, abs=5e-4)
        assert nzmax == pytest.approx(120.4, abs=5e-2)
        assert a_est == pytest.approx(-3.2, abs=5e-4)
        assert q == pytest.approx(9260.2, abs=5e-2)
        assert req != cmd, (
            "⚠️ req 與 cmd 寫成同一個值 ⇒ 那兩欄的**全部價值**就是它們不同")

    def test_a_live_closed_loop_writes_all_six(self, tmp_path):
        s = Rec(tmp_path)
        s.nz_dbg = {1: {"req_nz": 2.5, "cmd_nz": 2.5, "nz_max_true": 64.925,
                        "q_pa": 9576.8, "alpha_cmd": math.radians(-3.9876),
                        "alpha_est": math.radians(-3.2)}}
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        _n, _t = len(self.NEW), self.TAIL_AFTER_NEW
        tail = s._cw.rows[0][-(_n + _t):-_t]
        assert "" not in tail
        assert tail[3] == pytest.approx(-3.988, abs=5e-4)
        assert tail[3] != tail[4], (
            "⚠️ 「所要的 α」與「當下的 α」寫成同一個值 ⇒ 並列的意義整個消失")

    def test_the_recorded_angles_are_degrees_not_radians(self, tmp_path):
        """⚠️ 欄名帶單位就要真的是那個單位。本專案吃過「描述對但方向/單位沒說」
        的虧不只一次([[confirm-direction-not-just-topic]])。"""
        s = Rec(tmp_path)
        s.nz_dbg = {1: {"req_nz": 1.0, "cmd_nz": 1.0, "nz_max_true": 100.0,
                        "q_pa": 9000.0, "alpha_cmd": math.radians(-45.0),
                        "alpha_est": math.radians(30.0)}}
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        # ⚠️ 2026-08-23:原本寫死 [-9]/[-8] ⇒ 尾端追加一欄就整批平移一格,
        #    而症狀是「角度變成別欄的值」——看起來像單位錯,其實是位置錯。
        #    改成從 NEW 這一組的**組內位置**算(alpha_cmd_deg / alpha_est_deg
        #    是 NEW 的第 4、5 項)。
        _t = self.TAIL_AFTER_NEW
        _base = -(len(self.NEW) + _t)
        row = s._cw.rows[0]
        assert row[_base + self.NEW.index("alpha_cmd_deg")] == pytest.approx(
            -45.0, abs=1e-3)
        assert row[_base + self.NEW.index("alpha_est_deg")] == pytest.approx(
            30.0, abs=1e-3)

    def test_the_logging_layer_survives_a_stub_node_without_the_dict(
            self, tmp_path):
        """⚠️ 記錄層絕不可以因為缺屬性就讓整列消失 —— 少了那一列,計分器的
        觸地內插會少掉 (AGL>0, AGL≤0) 那一對,整架被判 n_touch=0。"""
        s = Rec(tmp_path)
        del s.nz_dbg
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        _t = self.TAIL_AFTER_NEW
        assert s._cw.rows[0][-_t:-(_t - 6)] == ["", "", "", "", "", ""]


# ══════════════════════════════════════════════════════════════════════════
#  反向鎖:確認本檔的手法本身沒有腐化
# ══════════════════════════════════════════════════════════════════════════
class TestTheHarnessItself:
    """⚠️ 鎖住的前提如果自己壞了,整組會亮綠燈放行。這幾條盯的是前提。"""

    @pytest.mark.parametrize("alpha_deg", [-11.6, -4.65, 0.0, 0.4])
    @pytest.mark.parametrize("roll_deg", [0.0, -23.0, -69.0, 75.0])
    def test_invert_really_inverts_the_forward_map(self, alpha_deg, roll_deg):
        """`_invert` 必須是 `pitch = γ + α/max(cos roll, 0.5)` 的**逆**。

        ⚠️ 這條不碰受測碼:它驗的是本檔自己的反解helper。反解若寫錯(例如
           照抄交辦的 `α = pitch·cos(roll) − γ`),L2 會拿一個錯的參照去比,
           而錯的參照與錯的實作「剛好互相抵銷」就是假綠燈。
        ⚠️ roll = ±69°/75° 刻意跨過 `max(cos roll, 0.5)` 的地板轉折點。
        """
        gam, alpha = math.radians(-30.0), math.radians(alpha_deg)
        roll = math.radians(roll_deg)
        k = max(math.cos(roll), 0.5)
        pitch = gam + alpha / k
        assert (pitch - gam) * k == pytest.approx(alpha, abs=1e-15)

    def test_the_clamp_detector_flags_the_known_clamped_point(self):
        """⚠️ `_clamped` 是所有反解鎖的守門員。它若恆回 False,L2 會在
        「pitch 已經是常數」的點上比出假綠燈。這裡驗它抓得到已知的貼邊點。"""
        s, (roll, pitch, thr, R, V) = _run(**dict(_POINTS["near_roll_clamp"]))
        assert _clamped(s, roll, pitch), "⚠️⚠️ 已知 roll 貼夾制的點卻沒被抓到"
        s2, got2 = _run()
        assert not _clamped(s2, got2[0], got2[1]), "⚠️ 一般工作點被誤判成貼邊"

    def test_the_golden_table_covers_every_point(self):
        assert set(_POINTS) == set(_GOLD_BEFORE)
        assert len(_POINTS) == 11

    def test_the_stub_uses_the_real_class_level_defaults_for_the_new_knobs(self):
        """⭐⭐ 黃金值是在樁的預設值上跑的 ⇒ 樁的五顆新旋鈕必須與真節點的
        類別層預設**同值**,否則 L1 鎖住的是一組沒有人會飛的組態。"""
        for k in ("nz_closed_loop", "nz_clamp_after_floor", "nz_max_g",
                  "aero_cl_alpha", "aero_cl_zero", "aero_cl_max",
                  "aero_q_sigma_exp"):
            assert getattr(Guid, k) == _class_default(k), (
                f"⚠️⚠️ 樁的 {k} 與真節點類別層預設不同 ⇒ 黃金值測的是別的組態")
            assert getattr(Guid, k) == _declared_default(k), (
                f"⚠️⚠️ {k} 的類別層預設與 declare_parameter 預設不一致 ⇒ "
                f"參數宣告失敗時會靜靜地走另一條路")

    def test_the_guidance_methods_really_came_from_the_file(self):
        """⚠️ mixin 若哪天綁錯方法(名字改了、搬到別的類別),整組會靜靜地
        測到空殼。這裡確認 `_pn_cmd` 真的跑得出有限的數。"""
        s, got = _run()
        assert len(got) == 5 and all(math.isfinite(x) for x in got)
        assert s.nz_dbg[1].keys() >= {"req_nz", "cmd_nz", "nz_max_true",
                                      "q_pa", "alpha_cmd", "alpha_est"}


# ══════════════════════════════════════════════════════════════════════
#  ⭐⭐⭐ L9(2026-08-22「2-lite」):橫向與總負載儀表的**值層級**鎖
#
#  ⚠️⚠️ 為什麼不只鎖欄名:`a_h` 與 `roll` 在此之前**從來沒有被任何一趟
#     記錄過**,所以先前所有橫向結論(總指令 29.3% 超 3 g、a_h 中位
#     2.87 g)都是重放重建。這一組欄位存在的唯一價值就是把它變成量測 ——
#     而只鎖欄名的話,取樣點取錯(例如在 ITCG 疊加**之前**、或在韌體
#     夾制**之前**)欄位照樣有數字、照樣通過欄名鎖,而且數字看起來合理。
#  ⚠️ 這一組**只記錄不改指令**:每條測試都同時確認回傳的 roll 沒被動到。
# ══════════════════════════════════════════════════════════════════════
class TestL9TheLateralInstrumentIsSampledAtTheRightPoint:

    GRAV = 9.80665
    LAT6 = ["a_h_cmd_g", "roll_cmd", "roll_out",
            "n_req_tot", "n_cmd_tot", "lat_share"]

    @staticmethod
    def _gam(s):
        """γ 由樁自己的幾何重算一次,不從被測碼拿。"""
        _dn, _de, _d, _h, _R, _vn, _ve, vh, hdot, _V = s._geom(1)
        return math.atan2(hdot, max(vh, 1e-3))

    def test_every_field_is_populated_by_a_real_tick(self):
        s, _out = _run()
        d = s.nz_dbg.get(1)
        assert d is not None, "跑完一拍卻沒有儀表 dict"
        for k in self.LAT6:
            assert k in d, f"儀表缺 {k}"
            assert d[k] is not None and math.isfinite(float(d[k])),                 f"{k} 不是有限數:{d[k]}"

    def test_n_cmd_tot_is_the_hypot_of_the_two_channels(self):
        """⭐ 這條恆等式就是這一欄的**全部價值**:全鏈路沒有任何一處夾
        hypot(nz, a_h/g),nz_max_g 只夾縱向。算錯就沒有人看得出耦合。

        ⚠️⚠️ **一定要在 req_nz != cmd_nz 的組態下測。** 預設樁裡夾制沒咬
           ⇒ 兩者相同 ⇒ 把 cmd 寫成 req(本專案已經記過三次的 req/cmd
           互換)完全測不出來。實測:突變 `hypot(nz_req, ...)` 在預設組態
           下 7 條全過。這裡用 nz_max_g=0.5 逼夾制咬住。
        """
        s, _ = _run(nz_max_g=0.5)
        d = s.nz_dbg[1]
        assert d["req_nz"] != d["cmd_nz"], (
            "這一組參數沒讓夾制咬到 => 前提沒了,req/cmd 互換將測不出來")
        assert d["n_cmd_tot"] == pytest.approx(
            math.hypot(d["cmd_nz"], d["a_h_cmd_g"]), rel=1e-9)
        assert d["n_req_tot"] == pytest.approx(
            math.hypot(d["req_nz"], d["a_h_cmd_g"]), rel=1e-9)
        assert d["n_cmd_tot"] != d["n_req_tot"], (
            "夾制咬住了但兩欄總量相同 => 其中一欄用錯了來源")

    def test_lat_share_says_which_channel_owns_the_budget(self):
        s, _ = _run()
        d = s.nz_dbg[1]
        assert 0.0 <= d["lat_share"] <= 1.0
        assert d["lat_share"] == pytest.approx(
            abs(d["a_h_cmd_g"]) / d["n_cmd_tot"], rel=1e-9)

    def test_a_h_is_sampled_after_the_itcg_term_not_before(self):
        """⭐⭐ 取樣點鎖。`roll = atan2(a_h, g·max(cosγ, 0.25))` 用的是
        **疊加 ITCG 之後**的 a_h;若儀表在疊加之前取樣,這條恆等式就破。

        ⚠️⚠️ **一定要把 ITCG 打開來測。** 本輪生產組態 itcg_on=0 ⇒ a_t=0
           ⇒ 兩個取樣點數值相同 ⇒ 取樣點取錯完全測不出來。實測:突變
           「在疊加前取樣」在 itcg_on=0 下 7 條全過。這裡打開它,
           a_h 由 -0.368 g 變 +0.032 g,取樣點一錯就對不上。
        """
        s, out = _run(itcg_on=True, itcg_gain=1.0, itcg_a_max_g=0.4)
        assert abs(s.itcg_a.get(1, 0.0)) > 1e-6, (
            "ITCG 沒有真的產生偏置 => 前提沒了,取樣點鎖等於沒鎖")
        d = s.nz_dbg[1]
        gam = self._gam(s)
        want = math.atan2(d["a_h_cmd_g"] * self.GRAV,
                          self.GRAV * max(math.cos(gam), 0.25))
        assert d["roll_cmd"] == pytest.approx(want, abs=1e-12), (
            "a_h_cmd_g 與 roll_cmd 不自洽 => 兩者取樣點不同")
        assert out[0] == pytest.approx(d["roll_out"], abs=1e-12), (
            "回傳的 roll 不等於 roll_out => 儀表寫在夾制之前")

    def test_roll_out_is_the_post_clamp_value(self):
        """⭐⭐ 讓韌體耦合夾制**真的咬**,再確認兩欄分得開。

        r_lim = 80°·|cos θ|,θ 是量測俯仰。俯仰壓到 -85° ⇒ r_lim ≈ 6.97°,
        而預設橫向需求約 23° ⇒ 必咬。
        ⚠️ 夾制最兇的時刻正是俯衝最陡的時刻 —— 只鎖不咬的情況等於沒鎖。
        """
        s, out = _run(att={1: (0.0, math.radians(-85.0), 0.0)})
        d = s.nz_dbg[1]
        r_lim = math.radians(80.0) * abs(math.cos(math.radians(-85.0)))
        assert abs(d["roll_out"]) <= r_lim + 1e-12, "roll_out 超出 r_lim"
        assert abs(d["roll_cmd"]) > r_lim, (
            "這一組參數沒讓夾制咬到 => 這條測試的前提沒了,請調 att/幾何")
        assert d["roll_cmd"] != d["roll_out"], (
            "夾制咬住了但兩欄相同 => roll_out 寫在夾制之前,"
            "「橫向權限被吃掉多少」永遠看不見")
        assert out[0] == pytest.approx(d["roll_out"], abs=1e-12)

    def test_the_csv_writes_degrees_not_radians(self, tmp_path):
        """⚠️ 欄名帶單位就要真的是那個單位(同 L8 的理由)。
        這裡刻意把**真跑過一拍**的儀表餵進記錄樁,而不是手寫假 dict ——
        單位錯在轉換那一行,手寫 dict 測不到。"""
        s, _ = _run()
        r = Rec(tmp_path)
        r.nz_dbg = {1: s.nz_dbg[1]}
        r._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        row = r._cw.rows[0]
        # ⚠️ 同一個坑:原本寫死 [-5]/[-4]。改成從 LAT 組內位置算。
        _L = TestL8TheCsvColumnContract
        _base = -_L.TAIL_AFTER_NEW
        cmd_deg = row[_base + _L.LAT6.index("roll_cmd_deg")]
        out_deg = row[_base + _L.LAT6.index("roll_out_deg")]
        assert cmd_deg == pytest.approx(math.degrees(s.nz_dbg[1]["roll_cmd"]),
                                        abs=5e-3)
        assert out_deg == pytest.approx(math.degrees(s.nz_dbg[1]["roll_out"]),
                                        abs=5e-3)
        assert abs(cmd_deg) > 1.0, "看起來仍是弧度"

    def test_a_partial_instrument_dict_degrades_to_blanks_not_a_lost_row(
            self, tmp_path):
        """⚠️⚠️ 缺鍵絕不可以讓**整列消失** —— 少一列會讓計分器的觸地內插
        少掉 (AGL>0, AGL≤0) 那一對,整架被判 n_touch=0。"""
        s = Rec(tmp_path)
        s.nz_dbg = {1: {"req_nz": 1.0, "cmd_nz": 1.0, "nz_max_true": 100.0,
                        "q_pa": 9000.0, "alpha_cmd": None,
                        "alpha_est": 0.0}}          # 舊六鍵,沒有 LAT 六鍵
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        assert len(s._cw.rows) == 1, "缺鍵讓整列消失了"
        _t = TestL8TheCsvColumnContract.TAIL_AFTER_NEW
        assert s._cw.rows[0][-_t:-(_t - 6)] == ["", "", "", "", "", ""],             "缺鍵要寫空字串,不可以寫 0(0 是合法的橫向指令)"

