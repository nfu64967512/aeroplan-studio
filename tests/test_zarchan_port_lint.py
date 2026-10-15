# -*- coding: utf-8 -*-
"""Zarchan 移植品的共通守衛(2026-08-23)。

═══════════════════════════════════════════════════════════════════════════
 這一組鎖在守什麼
═══════════════════════════════════════════════════════════════════════════
`matlab/mgs/+mgs/+zarchan/` 下的移植是 `mgs.selftest` 裡**唯一有外部真值**的
一組檢查 —— 其餘每一項都是「mgs 比 mgs」或「mgs 比我們自己產的飛行 log」。
所以它們壞掉的代價特別高:壞了之後我們就沒有任何獨立的方法知道導引核心對不對。

本檔把三波移植裡**各咬過一次**的教訓變成跨檔守衛。三個都有共同的形狀:
**靜態看起來完全正常,一跑就炸或靜靜給錯答案。**

  ① `if nargin == 0` 當模式開關
     只宣告 name-value 的 `arguments` 區塊裡,MATLAB 的 nargin **恆為 0**,
     不論傳幾個參數(R2025b 實測:ntest() / ntest('A',5) / ntest('A',5,'B',1)
     全部 0)。⇒ 條件恆真。第一波三個檔踩到:一個無限遞迴到
     StackOverflow,另兩個**靜靜把呼叫者的參數換成 benchmark**。

  ② local function 讀主函式的區域變數
     MATLAB 的 local function **看不到**主函式工作區。第三波 `engage3d` 把
     g = 32.2 放在主函式、在 local function 裡讀 ⇒ 第一次執行就
     `Unrecognized function or variable`,而完整的靜態驗證**沒抓到**
     (它是個合法的識別字)。⚠️ 同一類在 `tot_node._weave_theta_of` 也有一個,
     而那一行從沒執行過所以從沒炸過 —— 兩個 bug 互相遮掩。

  ③ 自檢無條件報錯
     第三波 `airframeSim` 的結構自檢失敗時無條件 error,而 `assert=false`
     對它沒有作用 ⇒ 一紅燈就連結果都拿不到,**沒辦法診斷是哪一側錯**。
     「做不到就報錯」是對的,但它不該同時把證據一起帶走。

⚠️ 本檔是**靜態掃描**,不是值層級鎖 —— 因為它守的正是「執行前看不出來」的
   那一類,而唯一能在執行前看出來的手段就是掃形狀。真正的答案由
   `mgs.zarchan.benchSuite()` 在 MATLAB 裡驗。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PKG = ROOT / "matlab" / "mgs" / "+mgs" / "+zarchan"

pytestmark = pytest.mark.skipif(
    not PKG.is_dir(), reason="matlab/ 不在這個 checkout 裡")


def ports():
    """所有移植檔。benchSuite 是彙總器不是移植品,分開處理。"""
    return sorted(p for p in PKG.glob("*.m") if p.stem != "benchSuite")


def strip_comments(src: str) -> str:
    """去掉 MATLAB 註解。

    ⚠️ 必須去註解:本檔的每一條規則都在講「程式碼裡不可以有 X」,而 X 幾乎
       一定會出現在解釋它為什麼不可以的註解裡 —— 那會讓每一條規則都對
       自己的說明文字誤報,然後被關掉。
    ⚠️ 只處理行首與行尾的 %,不處理字串裡的 % (sprintf 的格式)。
       這是刻意的近似:誤留一點程式碼不會漏報,誤刪字串才會。
    """
    out = []
    for line in src.splitlines():
        # 行內的 % 若在引號裡就不是註解。用「引號計數為偶數」近似。
        cut = None
        for i, ch in enumerate(line):
            if ch == "%" and line[:i].count("'") % 2 == 0:
                cut = i
                break
        out.append(line if cut is None else line[:cut])
    code = "\n".join(out)
    # ⚠️⚠️ 字串字面值也要剝掉(sprintf 的格式裡有一堆識別字),
    #    但**必須在剝完註解之後**,而且**不可以跨行**。
    #    第一版先剝字串、而且用 [^'](吃得下換行):英文註解裡的所有格
    #    (the book's DEG)會開啟一個假字串,一路吃到下一個引號為止,
    #    **把中間的真程式碼吞掉**。症狀是 lint 對兩個函式報「讀了 DEG」,
    #    而那兩個函式的碼裡根本沒有 DEG —— 誤報的成因不是規則太寬,
    #    是前處理把碼吃掉了,而那種更難查。
    return re.sub(r"'(?:[^'\n]|'')*'", "''", code)


ID = pytest.mark.parametrize("path", ports(), ids=lambda p: p.stem)


@ID
def test_nargin_is_never_a_mode_switch(path):
    """⚠️⚠️ ① nargin 恆為 0(只有 name-value 的 arguments 區塊)。

    第一波實測:`engage2d()` 無限遞迴到 StackOverflow;`engageLinear` 與
    `weaveDesignCurve` **靜靜把呼叫者的參數換成 benchmark**,而那更糟 ——
    它不會炸,只會讓你以為自己在掃參數。
    """
    # ⚠️ 只看**主函式**。nargin 只有在「arguments 區塊只宣告 name-value」的
    #    函式裡才退化成 0;有位置參數的 local function 用 nargin 是正常且正確的
    #    (flightControlDelay 的 `if nargin < 8` 就是),把它們一起罵是誤報。
    code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
    fns = list(re.finditer(r"^function\b", code, re.M))
    main = code[:fns[1].start()] if len(fns) > 1 else code
    hits = re.findall(r"\bnargin\s*[=<>~]=?\s*\d", main)
    assert not hits, (
        f"⚠️⚠️ {path.name} 拿 nargin 當條件:{hits}\n"
        "    只宣告 name-value 的 arguments 區塊裡,MATLAB 的 nargin **恆為 0**\n"
        "    (R2025b 實測,傳幾個參數都一樣)⇒ 條件恆真。\n"
        "    用一個明確的旗標當開關。")


@ID
def test_no_toolbox_calls(path):
    """基底 MATLAB only。⚠️ 有 toolbox 的開發機上一切正常,沒有的機器上
    整個套件消失,而症狀是「函式不存在」看起來像安裝壞了。"""
    code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
    # ⚠️ 清單刻意**不含** tf / ss / step / margin / place。
    #    它們在這個領域是再普通不過的變數名:Zarchan 全書用 `tf` 當飛行時間、
    #    `STEP` 當 RK2 的旗標。把它們列進去 ⇒ 誤報,而誤報會讓整條 lint 被關掉。
    #    只留在 MATLAB 之外幾乎不會當變數名的那些。
    banned = ("bode", "nyquist", "lsim", "c2d", "d2c",
              "lqr", "care", "dare", "normpdf", "normcdf")
    hits = sorted({b for b in banned
                   if re.search(rf"(?<![\w.]){b}\s*\(", code)})
    assert not hits, (
        f"⚠️ {path.name} 呼叫了 {hits} —— 這些是 Control System / Statistics "
        "Toolbox。本套件必須只用基底 MATLAB。")


#: ⚠️ 這三個**不是**書上 listing 的移植品,是把 Zarchan 的判準套到我們自己
#: (SI 的)mgs 執行結果上的橋接工具 ⇒ 它們沒有「書的單位」可宣告。
#: ⚠️ 但正因為它們跨在兩套單位之間,反而最該講清楚 —— 列在這裡是**帳**,
#: 不是豁免:哪天有人回頭補表頭,就把名字從這裡拿掉(反向鎖會提醒)。
NO_BOOK_UNITS = frozenset({"additiveLaw", "crosscheck", "sizing"})


@ID
def test_the_units_are_declared_in_the_header(path):
    """⚠️⚠️ 書大部分是 ft / g=32.2,而 **Listing 30.1 是公制**。
    混單位會產出一個看起來合理的錯數字,那是本專案最怕的形狀。
    ⇒ 每個檔都要在**表頭**講清楚自己是哪一種。"""
    head = path.read_text(encoding="utf-8", errors="replace")[:6000].lower()
    got = bool(re.search(r"\bunits?\b|\bft/s\b|\bmetric\b|\b32\.2\b", head))
    if path.stem in NO_BOOK_UNITS:
        assert not got, (
            f"⭐ {path.name} 已經宣告單位了 —— 請從 NO_BOOK_UNITS 移除,"
            "否則帳會停在過去。")
        pytest.skip(f"橋接工具,沒有書的單位({path.stem});見清單上的說明")
    assert got, (
        f"⚠️ {path.name} 的表頭沒有宣告單位。書大部分是 ft、g=32.2,"
        "而 Listing 30.1 是公制 —— 讀的人必須不用翻程式碼就知道是哪一種。")


#: ⚠️ 目前**無條件**在自檢失敗時報錯的移植。這不是豁免,是**帳**。
#:
#: 每一項的意思都是:自檢一旦紅燈,呼叫端連結果都拿不到,沒辦法診斷是哪一側錯。
#: 它們現在不痛,是因為自檢都通過 —— 那是運氣,不是設計。
#: `airframeSim` 已改掉,而它會被改掉正是因為它的自檢**真的**紅了一次,
#: 而拿到能診斷的數字花了好幾輪。
#:
#: ⇒ 這份清單可以**變小**,不可以變大。
SELFCHECK_ERRORS_UNCONDITIONALLY = frozenset({
    "accelMissAdjoint", "adjointBinomial5", "adjointOptimal",
    "adjointWithAutopilot", "covarianceHoming", "threeLoopAutopilot",
})


@ID
def test_a_selfcheck_error_is_gated_on_the_assert_flag(path):
    """⚠️ ③ 自檢無條件報錯 ⇒ 一紅燈就連結果都拿不到,沒辦法診斷。

    「做不到就報錯」是對的**預設**,但它不該同時把證據一起帶走。
    ⇒ 看 `error(...:selfcheck...)` 之前最近的 `if`,條件裡有沒有 assert。
    """
    raw = path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"error\(\s*'[^']*:selfcheck'", raw)
    if not m:
        pytest.skip("這個檔沒有 selfcheck 的 error")
    ifs = list(re.finditer(r"^\s*if\s+([^\n]*)$", raw[:m.start()], re.M))
    cond = ifs[-1].group(1).strip() if ifs else ""
    gated = "assert" in cond
    if path.stem in SELFCHECK_ERRORS_UNCONDITIONALLY:
        assert not gated, (
            f"⭐ {path.name} 已經改好了 —— 請從 "
            "SELFCHECK_ERRORS_UNCONDITIONALLY 移除,否則帳會停在過去。")
        pytest.skip(f"已知未改({path.stem});見清單上的說明")
    assert gated, (
        f"⚠️ {path.name} 的 selfcheck error 條件是 `if {cond}` ——\n"
        "    看不到 assert 旗標 ⇒ 無條件報錯,呼叫端連結果都拿不到,\n"
        "    而那正是最需要看資料的時候。\n"
        "    預設仍該報錯,但 assert=false 要能拿回結果(warning 即可)。\n"
        "    ⚠️ 既有的檔請加進 SELFCHECK_ERRORS_UNCONDITIONALLY 並寫明理由;\n"
        "    新檔一律不准。")


class TestNoLocalFunctionReadsTheMainWorkspace:
    """⚠️⚠️ ② MATLAB 的 local function **看不到**主函式的工作區。

    第三波 `engage3d` 把 g = 32.2 放在主函式、在 local function 裡讀它 ——
    第一次執行就 `Unrecognized function or variable`,而完整的靜態驗證沒抓到
    (那是個合法的識別字,只有執行到才知道)。

    ⭐ 這裡只掃**全大寫的常數命名**(GRAV_FT、G0、TAU0 之類):那是這一類 bug
       實際出現的形狀,而且誤報率低。小寫的區域變數同名太常見,掃了會吵。
    """

    #: 這些是 MATLAB 內建或本套件的模組層名字,出現在 local function 裡不算違規。
    ALLOWED = frozenset({"NaN", "Inf", "MATLAB", "R", "S", "P", "C", "T", "N",
                         "M", "L", "A", "B", "D", "E", "F", "G", "H", "I", "J",
                         "K", "O", "Q", "U", "V", "W", "X", "Y", "Z"})

    @staticmethod
    def _split(code):
        """回傳 (主函式碼, [(名字, 碼), ...])。"""
        # ⚠️⚠️⚠️ 簽名**會跨行**。只吃第一個實體行 ⇒ 兩件事同時錯:
        #    (a) 續行上的參數沒進 sig ⇒ 它們被判成「讀主函式的常數」
        #    (b) 續行被算進 body ⇒ 更確定「有讀到」
        #    兩個錯互相加強,所以看起來非常像真的。實例:radomeEngagement 的
        #    rhs(Y, XNL, ELAMDH, <續行> X4, ..., VC, ..., XNP, ...) 被判洩漏
        #    VC/XNP,而它們就明明白白在第二行的參數列上。
        #    ⚠️ 這是這條 lint 第三次因為「詞法假設太窄」誤報(前兩次:沒剝
        #    字串、在 body 裡找下一個函式的簽名)。誤報率高的 lint 會被關掉,
        #    而被關掉的 lint 抓不到它真正要抓的東西。
        _SIG = (r"^function\s"
                r"(?:[^\n]*\.\.\.[ \t]*\n)*"
                r"[^\n]*$")
        fn = list(re.finditer(_SIG, code, re.M))
        if not fn:
            return code, []
        main = code[fn[0].end():(fn[1].start() if len(fn) > 1 else len(code))]
        locals_ = []
        for i in range(1, len(fn)):
            end = fn[i + 1].start() if i + 1 < len(fn) else len(code)
            name = re.search(r"function\s+(?:\[?[^\]=]*\]?\s*=\s*)?(\w+)",
                             fn[i].group(0))
            # ⚠️⚠️ 簽名要**單獨帶出來**。第一版在 body 裡找 `function...`,
            #    而 body 是從 `function` 那一行**之後**開始的 ⇒ 找到的是
            #    **下一個**函式的簽名 ⇒ 每一個「其實是參數」的常數都被判成洩漏。
            #    實測 12 個命中**全部**是這個 bug,而 12/15 的誤報率會讓
            #    整條規則被關掉,連帶漏掉它真正要抓的那一個。
            locals_.append((name.group(1) if name else f"#{i}",
                            code[fn[i].end():end], fn[i].group(0)))
        return main, locals_

    @ID
    def test_no_uppercase_constant_leaks_across_the_boundary(self, path):
        code = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        main, locs = self._split(code)
        # 主函式裡被賦值的全大寫常數
        assigned = {m.group(1) for m in
                    re.finditer(r"^\s*([A-Z][A-Z0-9_]{1,})\s*=", main, re.M)}
        assigned -= self.ALLOWED
        if not assigned:
            pytest.skip("主函式沒有全大寫常數")
        bad = []
        for name, body, sig in locs:
            for c in assigned:
                if not re.search(rf"(?<![\w.]){c}(?![\w])", body):
                    continue
                # ⚠️ MATLAB 一行可以擠多個賦值,例如 optimalGainsCubic 的
                #    benchmarks() 第一行:  t = R.t;  NP = R.NP;  NPsl = R.NP_singlelag;
                #    只認行首的 ^\s*NP\s*= 就會把「這個函式自己有定義 NP」
                #    看成「它讀了主函式的 NP」。
                #    ⚠️⚠️ 這是這條 lint 第**五**次因為詞法假設太窄而誤報:
                #      ① 沒剝字串字面值 ② 在 body 裡找到下一個函式的簽名
                #      ③ 簽名跨 ... 續行 ④ 工具名與領域變數撞名 ⑤ 這個。
                #    每一次的形狀都一樣:規則對「我寫這條時看到的那個案例」
                #    成立,對真實語法不成立。改完一定要重跑那 20 個植入洩漏。
                #    分號或逗號後面的賦值一樣算數;`==` 不算,那是比較。
                own = re.search(rf"(?:^|[;,])\s*{c}\s*=(?!=)", body, re.M)
                # ⚠️⚠️⚠️ 這裡原本是 `c in sig` —— **純子字串比對**。
                #    `DEG` 會被 `GAMTDEG` 遮住:把 DEG 從
                #    `ballisticRun(RT2IC, VTIC, GAMTDEG, ...)` 拿掉之後,
                #    函式本體用了 DEG 三次而 lint **完全沒反應**,因為
                #    `'DEG' in sig` 仍然為真。
                #    ⚠️ 這是這條 lint 第六個詞法錯誤,而前五個都是**誤報**,
                #    這一個是**漏報** —— 它讓守衛安靜地放過它存在的理由。
                #    實測影響:48 個植入洩漏裡有 4 個因此沒被抓到
                #    (DEG⊂GAMTDEG、TS、XMEAN、DG⊂HEDEG 這一類)。
                param = bool(re.search(rf"(?<![\w.]){c}(?![\w])", sig))
                if not own and not param:
                    bad.append(f"{name}() 讀 {c}")
        assert not bad, (
            f"⚠️⚠️ {path.name}:local function 讀主函式的常數 —— {bad}\n"
            "    MATLAB 的 local function **看不到**主函式工作區 ⇒ 執行時\n"
            "    `Unrecognized function or variable`,而靜態檢查看不出來\n"
            "    (那是個合法的識別字)。把常數**當引數傳進去**。")


class TestTheSuiteItselfStaysHonest:
    """benchSuite 是彙總器;它的行為契約與移植品不同。"""

    SUITE = PKG / "benchSuite.m"

    def test_it_exists(self):
        assert self.SUITE.exists()

    def test_a_port_that_errors_becomes_a_failure_not_a_gap(self):
        """⭐⭐ 實測過:第三波兩個 port 一跑就炸,而板子**照樣印完**,
        把它們各記成一列 FAIL。那是設計行為,要鎖住 ——
        改成讓第一個錯誤中斷整塊板,就再也看不到全貌。"""
        code = self.SUITE.read_text(encoding="utf-8", errors="replace")
        assert re.search(r"catch\b", code), "彙總器必須接住 port 的例外"
        assert "PORT RAISED AN ERROR" in code, (
            "⚠️ 出錯的 port 要留下一列可見的 FAIL,不是消失。"
            "先前實測 airframeSim / engage3d 就是靠這一列被發現的。")

    def test_nfail_counts_scored_failures_only(self):
        """INFO(誠實拒評)不可以把板子染紅,否則沒有人敢拒評。"""
        code = self.SUITE.read_text(encoding="utf-8", errors="replace")
        assert re.search(r"scored", code), (
            "⚠️ 看不到 scored 的概念 —— INFO / SKIP / FAIL 三者必須分得開。")
