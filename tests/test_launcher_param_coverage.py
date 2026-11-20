# -*- coding: utf-8 -*-
"""啟動路徑的參數覆蓋率鎖(2026-08-23)。

═══════════════════════════════════════════════════════════════════════════
 這一組鎖在守什麼
═══════════════════════════════════════════════════════════════════════════
tot_node 有**兩條**啟動路徑,而它們的參數覆蓋率不一樣:

    互動/待命路徑  rep_interactive.sh → swarm_ros2_up.sh → tot_node(由 topic 收目標)
    批次路徑      run_west.sh → stot_repeat.sh → stot_trial.sh → tot_node

`stot_trial.sh` 用一份**手動維護的 `-p` 清單**啟動節點。清單裡沒有的參數
**不會**被傳,節點退回 `declare_parameter` 的宣告預設 —— 而那是**完全靜默**的:
沒有警告、契約行印出的是退回後的值(在區間內 ⇒ 判定 OK)、
開跑前閘門也過,因為那個值本身合法。

⚠️⚠️ 這一條是實際咬過人的,不是假想:

  2026-08-23,WV1-3 三趟(約 2.5 小時 SITL)整批白跑。啟動腳本
  `export WEAVE_TIME_SYNC=True WEAVE_RELATIVE=True`,節點跑的是 **0 / 0**。
  同一次還順帶吃到 `vmax=100`(stot_trial.sh 自己的預設,不是 run_west 的 150)
  與 `n=6`(`-p n:="${SWARM_COUNT:-6}"`,而啟動腳本設的是 `N`)。
  三趟的結論本來會被寫成「蛇形通道無效」。

  真正的錯誤是**換了 launcher 而不自知**:基準系列(RA1/RC1/RD1/RE21/SB1-3)
  全部走互動路徑,我改用了批次路徑。兩條路徑看起來都「跑得起來」。

⇒ 本檔把「哪條路徑設得到哪些參數」變成一份**被審過的清單**。
   缺口可以存在(批次路徑本來就不打算支援全部),但不可以**沒有人知道**。

═══════════════════════════════════════════════════════════════════════════
 ⚠️ 為什麼不是「把 31 個補進 stot_trial.sh 就好」
═══════════════════════════════════════════════════════════════════════════
補進去只解決今天這 31 個。下一個加到契約表的參數會**再一次**只被加進一條路徑,
而症狀一樣是靜默的。所以這裡鎖的是**差集本身**:差集變大就紅燈,
逼人在加參數的當下就決定「這條路徑要不要支援它」。
"""
import pathlib
import re
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
WSL = ROOT / "scripts" / "wsl"
sys.path.insert(0, str(WSL / "sar_swarm_patches"))

import contract_table as ct                                    # noqa: E402

#: 由**別的**節點擁有的契約參數 —— tot_node 的啟動路徑本來就不該傳它們。
#: (contract_verify.py 的 NODE_PARAMS 把它們掛在 wingman/master/backup 底下。)
OWNED_BY_OTHER_NODES = frozenset({
    "formation_alt", "patrol_alt", "cruise_alt_threshold", "climbout_hold_alt",
})

TOT_CONTRACTS = frozenset(
    c.name for c in ct.TABLE if c.source == ct.SOURCE_NODE) - OWNED_BY_OTHER_NODES


def _passed(path: pathlib.Path, anchor: str | None = None) -> frozenset:
    """腳本裡 `-p <name>:=` 實際傳出去的參數名。

    ⚠️ 讀的是**值層級的事實**(這個名字有沒有出現在命令列上),不是
       「那一行長什麼樣」。改排版、換變數名、把命令拆行都不影響。
    """
    s = path.read_text(encoding="utf-8", errors="replace")
    if anchor is not None:
        i = s.find(anchor)
        assert i != -1, f"⚠️ {path.name} 找不到錨點 {anchor!r} —— 本測試的定位失效了"
        s = s[i:]
        j = s.find("\n\n")           # 到命令結束(第一個空行)
        if j > 0:
            s = s[:j]
    return frozenset(re.findall(r"-p ([a-z_0-9]+):=", s))


UP = WSL / "swarm_ros2_up.sh"
TRIAL = WSL / "stot_trial.sh"


class TestTheInteractivePathIsComplete:
    """⭐⭐ 互動路徑是基準系列在用的那一條 ⇒ 它必須 100% 覆蓋。

    這一條沒有豁免清單:互動路徑漏一個參數,代表整個基準系列的某個設定
    其實從來沒生效過,而先前所有結論都建立在它之上。
    """

    def test_it_can_set_every_tot_node_contract(self):
        gap = sorted(TOT_CONTRACTS - _passed(UP))
        assert not gap, (
            f"⚠️⚠️ 互動路徑設不到這些 tot_node 契約參數:{gap}\n"
            "    ⇒ 不論怎麼 export,節點都會退回宣告預設,**完全靜默**:\n"
            "      沒有警告、契約行印出的是退回後的值、開跑前閘門也過。\n"
            f"    修法:在 {UP.name} 加 -p <name>:=\"$TOT_<NAME>\" 與對應的預設。")


class TestTheBatchPathGapIsReviewedNotAccidental:
    """⚠️⚠️ 批次路徑的缺口是**已知技術債**,不是設計。

    這份清單記錄 2026-08-23 當下的實況。它可以**變小**(把參數補進
    stot_trial.sh),但不可以變大 —— 變大代表又有人加了參數卻只加一條路徑,
    而下一個用批次路徑的人會再撞一次 WV1-3。
    """

    #: ⚠️ 這不是豁免,是**帳**。每一項都是「透過批次路徑設它會被靜靜忽略」。
    #:    真要用這些參數,請走互動路徑(rep_interactive.sh),或先補 stot_trial.sh。
    KNOWN_BATCH_GAPS = frozenset({
        # ── 放行排程 / 時間同步。WV1-3 就是死在這裡。
        "ring_pace", "ring_pace_far", "ring_pace_settle", "ring_pace_tol",
        "ring_pace_hold_n", "release_at_ring", "wait_goto_radius",
        "dive_from_runup", "weave_time_sync", "weave_relative",
        # ── 協同歸向的放行閘門(2026-08-23)。現階段只觀測(gate_arm=False),
        #    所以批次路徑設不到它們**暫時**無害。⚠️ 等 gate_arm 要真的致動時,
        #    這三個必須先補進 stot_trial.sh —— 否則批次跑出來的閘門行為會與
        #    互動路徑不同,而那個差異是靜默的(WV1-3 就是這樣白跑三趟)。
        "gate_arm", "gate_spread_s", "gate_alt_band_m", "gate_range_err_m",
        # ── 終端律的實驗開關(ITCG / MPNG)。都預設關閉。
        "itcg_on", "itcg_gain", "itcg_a_max_g", "mpng_on", "mpng_k_eps", "mpng_k_s",
        # ── n_z 閉迴路與氣動常數。⚠️ 常數的餵值路徑斷掉是靜默的
        #    (aero_q_sigma_exp 用錯只表現成「高空少 0.86 g」)。
        "nz_closed_loop", "nz_clamp_after_floor", "nz_max_g",
        "aero_cl_alpha", "aero_cl_zero", "aero_cl_max", "aero_q_sigma_exp",
        # ── 預測器與時刻表來源。
        "tgo_from_predictor", "tgo_ring_transit", "tgo_predict_dt", "predict_hz",
        # ── 其餘。
        "pn_alpha_tau", "state_max_age", "att_eas_floor",
        # ⚠️⚠️ expend_on_impact 缺席特別危險:不終結落海機體會讓它沉水再爬起
        #    做第二趟(second-pass-contamination),而節點宣告預設剛好是 True
        #    ⇒ 目前**靠運氣**是對的。
        "expend_on_impact",
    })

    def test_no_new_gap_appeared(self):
        gap = TOT_CONTRACTS - _passed(TRIAL, "ros2 run sar_swarm tot_node")
        new = sorted(gap - self.KNOWN_BATCH_GAPS)
        assert not new, (
            f"⚠️⚠️ 批次路徑**新增**了設不到的契約參數:{new}\n"
            "    ⇒ 透過 stot_trial.sh 設它們會被靜靜忽略,節點退回宣告預設。\n"
            "    2026-08-23 的 WV1-3 就是這樣白跑三趟(約 2.5 小時 SITL):\n"
            "      export WEAVE_TIME_SYNC=True,節點跑的是 0,而每一道閘門都說 OK。\n"
            "    兩個修法(擇一,不要只加註解):\n"
            "      ① 在 stot_trial.sh 的 `ros2 run sar_swarm tot_node` 加 -p;\n"
            "      ② 確定批次路徑不需要它 ⇒ 加進 KNOWN_BATCH_GAPS 並寫明理由。")

    def test_the_ledger_has_no_stale_entries(self):
        """⭐ 反向鎖:清單裡列了但其實**已經**傳得到的項目要移除。

        留著會讓人以為「這個參數在批次路徑沒用」而繞路,或在補完之後
        永遠不知道帳已經清了。
        """
        gap = TOT_CONTRACTS - _passed(TRIAL, "ros2 run sar_swarm tot_node")
        stale = sorted(self.KNOWN_BATCH_GAPS - gap)
        assert not stale, (
            f"⚠️ 這些已經傳得到了,請從 KNOWN_BATCH_GAPS 移除:{stale}")

    def test_the_ledger_only_names_real_contracts(self):
        bogus = sorted(self.KNOWN_BATCH_GAPS - TOT_CONTRACTS)
        assert not bogus, (
            f"⚠️ KNOWN_BATCH_GAPS 列了契約表上沒有的名字:{bogus}\n"
            "    ⇒ 打錯字的項目會永遠留在帳上,而它擋不到任何東西。")


class TestTheAircraftCountVariableIsUnambiguous:
    """⚠️⚠️ 兩條路徑讀**不同**的變數名決定機數,而給錯的那個是靜默的。

    WV1-3 的啟動腳本設了 `N=2`,批次路徑讀的是 `SWARM_COUNT`(預設 6)
    ⇒ 實際飛了 6 架,而我拿它去比 n=2 的基準系列。
    """

    def test_the_batch_path_reads_swarm_count(self):
        s = TRIAL.read_text(encoding="utf-8", errors="replace")
        m = re.search(r'-p n:="\$\{([A-Z_]+):-(\d+)\}"', s)
        assert m, "⚠️ stot_trial.sh 的 `-p n:=` 形式變了,本測試的定位失效"
        var, dflt = m.group(1), int(m.group(2))
        assert var == "SWARM_COUNT", (
            f"⚠️ 批次路徑改讀 {var} 了 —— 請同步更新 rep_interactive.sh 與本測試,"
            "否則兩條路徑會再一次用不同的變數名決定機數。")
        assert dflt == 6, (
            f"⚠️ 批次路徑的機數預設變成 {dflt} 了。這個數字**不是**中性的:"
            "設錯變數名時它就是實際飛的架數,而沒有任何警告。")

    def test_the_interactive_path_forwards_both_names(self):
        """⭐ 互動路徑同時傳 N 與 SWARM_COUNT ⇒ 兩個名字都對。

        這是目前唯一「給哪個都不會錯」的路徑,值得鎖住。
        """
        s = (WSL / "rep_interactive.sh").read_text(encoding="utf-8", errors="replace")
        # ⚠️ 必須錨在**啟動 ROS2 層那一行**,不能只找 `N="$N" SWARM_COUNT="$N"`:
        #    這個片語在檔案裡出現三次(swarm_sitl_up / swarm_ros2_up / 就緒等待),
        #    只找片語的話,拿掉 ROS2 那一行的轉發仍然會通過。
        #    2026-08-23 的刪除實驗實測:那個寫法**沒有咬**。
        # ⚠️ 錨點要允許**行接續**(2026-08-24):編隊生效值接進來之後,
        #    起 swarm_ros2_up.sh 那一句已經是多行的。原本的 `[^\n]*?`
        #    因此對不上 —— 而它**正確地大聲報錯**了,沒有靜默通過。
        #    這裡只放寬「可以跨行」,**不**放寬「兩個名字都要轉發」那個斷言。
        for launcher in ("swarm_ros2_up.sh", "swarm_sitl_up.sh"):
            m = re.search(
                r'setsid env ((?:[^\n]|\\\n)*?)bash "\$SR/' + re.escape(launcher),
                s)
            assert m, f"⚠️ rep_interactive.sh 不再用 setsid env 起 {launcher} —— 錨點失效"
            envs = m.group(1)
            for var in ("N", "SWARM_COUNT"):
                assert re.search(rf'\b{var}="\$N"', envs), (
                    f"⚠️ 起 {launcher} 時沒轉發 {var} ⇒ 設 N 的人可能拿到"
                    f"預設機數,而那是靜默的(WV1-3 實際飛了 6 架而不是 2 架)。\n"
                    f"    實得 env 段:{envs.strip()}")


class TestTheGapIsBigEnoughToDeserveTheWarning:
    """⭐ 讓數字本身留在測試裡,不然「缺很多」會隨時間變成一句沒有份量的話。"""

    def test_the_batch_path_covers_well_under_half(self):
        got = TOT_CONTRACTS & _passed(TRIAL, "ros2 run sar_swarm tot_node")
        assert len(TOT_CONTRACTS) >= 50, "契約表縮水了?本測試的量級假設要重看"
        # 2026-08-23 實況:24 / 55。
        assert len(got) < len(TOT_CONTRACTS) * 0.6, (
            f"⚠️ 批次路徑的覆蓋率變成 {len(got)}/{len(TOT_CONTRACTS)} —— "
            "如果是補齊了,請把 KNOWN_BATCH_GAPS 一起縮小並改掉這一條的門檻。")


class TestNoShellAssignmentHasSpacesAroundTheEquals:
    """⚠️⚠️ `export FOO   =1.0` 在 bash **不是賦值**。

    實測(2026-08-23):

        export FOO_A         =60.0
        rc=1   FOO_A='[]'   ${FOO_A:-DEFAULT} -> DEFAULT

    bash 把它讀成「`export FOO_A`」加上一個非法識別字 `=60.0`:變數被匯出成
    **空字串**,而 `${FOO:-預設}` 對空字串也會取預設 ⇒ 整組設定靜靜退回預設值,
    唯一的痕跡是 stderr 上一行 `not a valid identifier`,而它會被淹沒在
    ROS2 的輸出裡。

    這一條是實際咬過的:重跑 WX 時為了讓 55 行 export 對齊,用了 `%-22s`
    格式化 —— **55 行全部失效**,而航次照常起飛。

    ⚠️ 純賦值 `FOO =1` 更糟:那是「執行名為 FOO 的指令,參數 =1」。
    """

    #: 只掃**我們自己**的 shell 腳本。
    ROOTS = ("scripts/wsl", "scratchpad")

    @staticmethod
    def _offenders(path: pathlib.Path):
        bad = []
        for k, line in enumerate(
                path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            s = line.split("#", 1)[0]
            # ⚠️ **只抓 `export` 開頭。** 第一版連裸賦值 `FOO   =…` 一起抓,
            #    結果命中的全是腳本裡**內嵌的 Python / awk**
            #    (stot_repeat.sh 的 `rows = [...]`、swarm_column_up.sh 的
            #     `bearing = (...)`)—— 那些語言的 `=` 兩側本來就可以有空格。
            #    誤報會讓人把整條 lint 關掉,而它要擋的那一類就再也沒人看。
            #    `export` 是 shell 關鍵字,不會出現在那些內嵌片段裡。
            if re.search(r"^\s*export\s+[A-Za-z_]\w*\s+=", s):
                bad.append((k, line.strip()[:90]))
        return bad

    def test_no_script_has_a_spaced_assignment(self):
        hits = []
        for root in self.ROOTS:
            d = ROOT / root
            if not d.is_dir():
                continue
            for f in sorted(d.rglob("*.sh")):
                for k, line in self._offenders(f):
                    hits.append(f"{f.relative_to(ROOT)}:{k}  {line}")
        assert not hits, (
            "⚠️⚠️ 這些行的 `=` 前面有空格 ⇒ **不是賦值**,變數會是空字串,\n"
            "    而 ${VAR:-預設} 會靜靜取預設值(航次照常起飛,結論全錯):\n      "
            + "\n      ".join(hits))

    def test_the_lint_actually_detects_the_shape(self):
        """⭐ 反向鎖:這條 lint 抓得到已知的壞形狀,而不是永遠回空清單。"""
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "x.sh"
            p.write_text(
                "export FOO_A         =60.0\n"       # 1  壞的 —— 必須抓到
                "export FOO_B=60.0\n"                # 2  好的
                "rows = [r for r in x]\n"            # 3  內嵌 Python,不可誤報
                "bearing = (cb=='') ? 1 : 2;\n"      # 4  內嵌 awk,不可誤報
                "# export FOO_C   =1\n",             # 5  註解,不算
                encoding="utf-8")
            got = [k for k, _ in self._offenders(p)]
            assert got == [1], (
                f"lint 抓到的行號是 {got},應為 [1]。\n"
                "    3/4 行是腳本裡內嵌的 Python 與 awk —— 那些語言的 `=` 兩側\n"
                "    本來就可以有空格。第一版連裸賦值一起抓,命中的**全是**\n"
                "    這一類,而誤報會讓人把整條 lint 關掉。")
