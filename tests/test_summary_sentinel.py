"""結案哨兵:四種結案形狀 × 三個消費者的匹配矩陣。

背景 —— BLOCKER 3(2026-08-05):
  上一版的摘要行把「結案哨兵」和「達標描述」寫在同一個字串上:

      lines.append(f"★★ 俯衝命中紅心:{verdict}。{tail}")

  verdict 在「零 miss 但有機高空掠過」時會變成「⚠️ 名義命中,但 N/M 架在
  高空掠過(未落地){…}」,於是印出來的是

      ★★ 俯衝命中紅心:⚠️ 名義命中,但 6/6 架在高空掠過…

  —— 既不含 "俯衝命中紅心:全員命中" 也不含 "俯衝結案:",三個消費者
  (stot_trial.sh、stot_kpi_loop.sh、_kpi_parse.py)全部落空:
    * stot_trial.sh 的結案等待迴圈跑滿 WAIT_TICKS(預設 120×12 s = 1440 s)
      後記成 timeout、exit 2 ⇒ 連帶跳過 make_artifacts.sh(專案規定「航次一
      結束就立刻產素材」,資料是覆寫式的)與觸地基準摘要
    * stot_kpi_loop.sh 同樣卡死
    * _kpi_parse.py 寫出 `TRIAL,,,,0,0,no_summary`
  ⚠️ **這正是 RD1 的形狀**(6/6 命中、命中高度 598–1460 m、零 miss)⇒ 歸因
     會從「全員高空掠過」變成看起來像飛行失敗的「逾時」。

修法:哨兵與達標描述**分離**。tot_node.CLOSEOUT_SENTINEL 是給機器 grep 的
固定字串,無條件出現在每一種結案形狀的行首;達標描述只給人看,再怎麼變都
蓋不掉哨兵。

⚠️⚠️ 這個檔案的「消費者匹配式」**直接從 .sh / .py 原始碼裡讀出來**,不是抄
   一份常數進來。抄一份的話,.sh 改了測試不會紅 —— 又是一個假迴歸鎖(本 repo
   已經被 _dmin_frozen 那種死述詞咬過一次)。萃取失敗一律當測試失敗。
"""
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WSL = REPO / "scripts" / "wsl"
PATCHES = WSL / "sar_swarm_patches"
sys.path.insert(0, str(PATCHES))

# 重用 test_miss_exit 的 ROS 樁與 node() 工廠 —— 測的是**真的那份** tot_node.py。
# 複製一份邏輯來測只會證明複製品是對的。
from tests.test_miss_exit import node  # noqa: E402

import tot_node as TN  # noqa: E402


# ═══════════════════════════════════════════════════════════════════════
#  四種結案形狀
# ═══════════════════════════════════════════════════════════════════════

def shape_a():
    """(a) 零 miss + 全部低空觸地 —— 真達標。基準線 P91 的形狀。"""
    s = node()
    for i in range(1, 7):
        s.impact[i] = s.dive_go + 40.0 + 0.1 * i
        s.dmin[i] = 1.0 + 0.1 * i
        s.hit_d[i] = 20.0 + i
        s.hit_alt[i] = 2.0
    return s


def shape_b():
    """(b) 零 miss + 全部高空掠過 —— **RD1 形狀**,上一版就是在這裡壞掉。

    命中高度取 RD1 的真值 598/775/927/746/1460/1291 m(一架都沒落地,
    而摘要行照樣被記成「全員命中」——見 tot_node 的 too_high 說明)。
    """
    s = node()
    alts = {1: 598.0, 2: 775.0, 3: 927.0, 4: 746.0, 5: 1460.0, 6: 1291.0}
    for i in range(1, 7):
        s.impact[i] = s.dive_go + 40.0 + 0.1 * i
        s.dmin[i] = 1.0 + 0.1 * i
        s.hit_d[i] = 20.0 + i
        s.hit_alt[i] = alts[i]
    return s


def shape_b_partial():
    """(b') 零 miss + **部分**高空掠過 —— 分母是命中數,不是全隊。"""
    s = node()
    for i in range(1, 7):
        s.impact[i] = s.dive_go + 40.0 + 0.1 * i
        s.dmin[i] = 1.0 + 0.1 * i
        s.hit_d[i] = 20.0 + i
        s.hit_alt[i] = 900.0 if i in (5, 6) else 2.0
    return s


def shape_c():
    """(c) 有 miss + 有命中 —— SW1 的逃逸鏈形狀。"""
    s = node()
    for i in (1, 3, 4, 5):
        s.impact[i] = s.dive_go + 40.0 + 0.1 * i
        s.dmin[i] = 5.0
        s.hit_d[i] = 22.0
        s.hit_alt[i] = 2.0
    for i in (2, 6):
        s.missed[i] = s.dive_go + 60.0
        s.miss_why[i] = "逃逸:離目標 412.0 km > miss_far_m 20.0 km"
        s.dmin[i] = 638.1
    return s


def shape_d():
    """(d) 全部 miss —— 一架都沒命中,航次仍必須結得了案。"""
    s = node()
    for i in range(1, 7):
        s.missed[i] = s.dive_go + 60.0
        s.miss_why[i] = "逾時"
        s.dmin[i] = 900.0
    return s


SHAPES = {
    "a_all_low_touch": shape_a,
    "b_all_high_pass": shape_b,
    "b_partial_high_pass": shape_b_partial,
    "c_some_miss": shape_c,
    "d_all_miss": shape_d,
}
SHAPE_IDS = list(SHAPES)


def summary_of(name):
    return SHAPES[name]()._impact_summary()[0]


# ═══════════════════════════════════════════════════════════════════════
#  消費者的匹配式 —— 從原始碼萃取,不抄常數
# ═══════════════════════════════════════════════════════════════════════

def _grep_pattern(rel):
    """從 .sh 讀出結案等待迴圈的 `grep -qE "…"` 匹配式,連同行號。

    ⚠️ 只准有一條。多出一條代表有人另外加了偵測點而沒有納入這個矩陣,
       那條就會變成下一個「沒人測到」的落空路徑。
    """
    src = (WSL / rel).read_text(encoding="utf-8")
    hits = [(no, m.group(1))
            for no, ln in enumerate(src.splitlines(), 1)
            for m in [re.search(r'grep -qE "([^"]+)"', ln)] if m]
    assert len(hits) == 1, \
        f"{rel} 的 `grep -qE` 匹配式不是剛好一條:{hits}(萃取失敗 = 這個測試" \
        f"就變成假迴歸鎖,所以直接當失敗)"
    return hits[0]                       # (lineno, pattern)


def _kpi_consts():
    """從 _kpi_parse.py 讀出 CLOSEOUT / LEGACY 兩個判定字串,連同行號。"""
    src = (WSL / "_kpi_parse.py").read_text(encoding="utf-8")
    out = {}
    for no, ln in enumerate(src.splitlines(), 1):
        m = re.match(r'(CLOSEOUT|LEGACY) = "([^"]+)"', ln)
        if m:
            out[m.group(1)] = (no, m.group(2))
    assert set(out) == {"CLOSEOUT", "LEGACY"}, \
        f"_kpi_parse.py 的判定常數萃取失敗:{out}"
    # 判定式本身也要對得上,否則常數改了但 if 沒用它 = 白測
    assert "if CLOSEOUT in ln or LEGACY in ln:" in src, \
        "_kpi_parse.py 的結案行判定式不是 `if CLOSEOUT in ln or LEGACY in ln:`"
    return out


TRIAL_LINE, TRIAL_PAT = _grep_pattern("stot_trial.sh")          # 來源:stot_trial.sh
LOOP_LINE, LOOP_PAT = _grep_pattern("stot_kpi_loop.sh")         # 來源:stot_kpi_loop.sh
_KPI = _kpi_consts()                                            # 來源:_kpi_parse.py


def _kpi_matches(line):
    """複刻 _kpi_parse.py 的結案行判定(常數從原始碼讀)。"""
    return _KPI["CLOSEOUT"][1] in line or _KPI["LEGACY"][1] in line


CONSUMERS = {
    f"stot_trial.sh:{TRIAL_LINE}": lambda ln: re.search(TRIAL_PAT, ln) is not None,
    f"stot_kpi_loop.sh:{LOOP_LINE}": lambda ln: re.search(LOOP_PAT, ln) is not None,
    f"_kpi_parse.py:{_KPI['CLOSEOUT'][0]}": _kpi_matches,
}


# ═══════════════════════════════════════════════════════════════════════
#  (i) 固定哨兵在四種形狀下都存在
# ═══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("shape", SHAPE_IDS)
def test_sentinel_present_in_every_closeout_shape(shape):
    line = summary_of(shape)
    assert line.startswith(TN.CLOSEOUT_SENTINEL), \
        f"形狀 {shape} 的摘要行沒有以固定哨兵開頭 ⇒ 三個消費者會落空:{line[:80]}"


@pytest.mark.parametrize("shape", SHAPE_IDS)
def test_hit_and_miss_counts_are_machine_readable(shape):
    """哨兵後面緊接的計數是固定格式,不受達標描述影響。"""
    line = summary_of(shape)
    m = re.match(re.escape(TN.CLOSEOUT_SENTINEL) + r"(\d+)/(\d+) 命中、(\d+) MISS",
                 line)
    assert m, f"形狀 {shape} 的計數段格式跑掉了:{line[:80]}"
    n_hit, n_all, n_miss = (int(x) for x in m.groups())
    assert n_all == 6
    assert n_hit + n_miss == n_all, f"命中 {n_hit} + MISS {n_miss} != {n_all}"


def test_the_sentinel_is_not_the_verdict():
    """⭐ 哨兵與達標描述**分離**:同一個哨兵下掛得出四種不同的達標描述。

    上一版就是因為兩者綁在一起,verdict 一變就把哨兵覆蓋掉。
    """
    verdicts = set()
    for shape in SHAPE_IDS:
        line = summary_of(shape)
        head, _, rest = line.partition(";")
        assert head.startswith(TN.CLOSEOUT_SENTINEL)
        verdicts.add(rest.split("。")[0])
    assert len(verdicts) == len(SHAPE_IDS), \
        f"每種形狀該有自己的達標描述,實得 {len(verdicts)} 種:{verdicts}"


# ═══════════════════════════════════════════════════════════════════════
#  (ii) 匹配矩陣:四(五)種形狀 × 三個消費者
# ═══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("shape", SHAPE_IDS)
@pytest.mark.parametrize("who", list(CONSUMERS))
def test_every_consumer_matches_every_shape(who, shape):
    line = summary_of(shape)
    assert CONSUMERS[who](line), \
        (f"{who} 的匹配式抓不到形狀 {shape} ⇒ 該消費者會空等到 timeout"
         f"(stot_trial.sh 預設 120×12 s = 1440 s)並跳過 make_artifacts.sh。\n"
         f"  匹配式:{TRIAL_PAT if 'trial' in who else LOOP_PAT}\n"
         f"  摘要行:{line[:120]}")


def test_matrix_is_complete():
    """⚠️ 少一個消費者或少一種形狀,上面的矩陣就縮水而且不會有人發現。"""
    assert len(CONSUMERS) == 3
    assert len(SHAPE_IDS) == 5          # 四種必測形狀 + 部分高空掠過


def _run_kpi(tmp_path, line):
    """跑**真的那支** _kpi_parse.py,回傳它 append 的那一列。"""
    log = tmp_path / "trial.log"
    log.write_text("[WARN] [tot_node]: " + line + "\n", encoding="utf-8")
    out = tmp_path / "kpi.csv"
    env = dict(os.environ, TOT_LOG=str(log), CSV=str(out), TRIAL="TST",
               MISS_MAX="30", ALT_MAX="30", SPREAD_MAX="3.0",
               PYTHONIOENCODING="utf-8")
    subprocess.run([sys.executable, str(WSL / "_kpi_parse.py")],
                   env=env, check=True, capture_output=True)
    return out.read_text(encoding="utf-8").strip()


@pytest.mark.parametrize("shape", SHAPE_IDS)
def test_the_real_kpi_parser_never_writes_no_summary(tmp_path, shape):
    """no_summary = 「跑完」被記成「沒跑完」,歸因整個錯掉。"""
    row = _run_kpi(tmp_path, summary_of(shape))
    assert "no_summary" not in row, f"形狀 {shape} 被記成沒有摘要:{row}"


@pytest.mark.parametrize(
    "shape,n_hit,ok",
    [("a_all_low_touch", "6", "1"),      # 真達標
     ("b_all_high_pass", "6", "0"),      # RD1:6/6 命中但高空掠過 ⇒ 不通過
     ("b_partial_high_pass", "6", "0"),
     ("c_some_miss", "4", "0"),
     ("d_all_miss", "0", "0")])
def test_kpi_verdict_per_shape(tmp_path, shape, n_hit, ok):
    """⚠️ 哨兵讓四種形狀都「讀得到」,但**只有**形狀 (a) 該判通過。

    高空掠過必須靠命中高度欄位被擋下來(RD1 的 598–1460 m 遠超 ALT_MAX),
    不是靠讀不到摘要行而變成 timeout —— 那是歸因錯誤,不是把關。
    """
    row = _run_kpi(tmp_path, summary_of(shape))
    cols = row.split(",")
    assert cols[4] == n_hit, f"n_hit 錯:{row}"
    assert cols[5] == ok, f"pass 判定錯:{row}"


# ═══════════════════════════════════════════════════════════════════════
#  (iii) 向後相容:形狀 (a) 逐字含舊字串
# ═══════════════════════════════════════════════════════════════════════

def test_shape_a_contains_the_legacy_string_verbatim():
    """既有基準與歷史 log 的比對靠這個字串,一個字都不能動。"""
    assert TN.LEGACY_ALL_HIT == "俯衝命中紅心:全員命中"
    assert TN.LEGACY_ALL_HIT in summary_of("a_all_low_touch")


@pytest.mark.parametrize("shape", [s for s in SHAPE_IDS if s != "a_all_low_touch"])
def test_only_shape_a_may_say_all_hit(shape):
    """⚠️ 其餘形狀出現「全員命中」= 舊 grep 把高空掠過/脫靶讀成成功。"""
    assert "全員命中" not in summary_of(shape)


def test_legacy_only_logs_still_parse():
    """2026-08-05 之前的 log 沒有固定哨兵,只有舊字串 —— 仍要讀得到。"""
    old = ("[WARN] [tot_node]: ★★ 俯衝命中紅心:全員命中。命中時間散布 = 0.50s "
           "(≈70.0m 縱向間隔);最近水平距 = {1: 3.0} m;命中高度 = {1: 2} m")
    for who, fn in CONSUMERS.items():
        assert fn(old), f"{who} 讀不到 2026-08-05 之前的舊 log"


# ═══════════════════════════════════════════════════════════════════════
#  消費者與節點的常數必須對得上
# ═══════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("rel,pat", [("stot_trial.sh", TRIAL_PAT),
                                     ("stot_kpi_loop.sh", LOOP_PAT)])
def test_shell_patterns_carry_the_node_sentinel(rel, pat):
    assert TN.CLOSEOUT_SENTINEL in pat, \
        f"{rel} 的匹配式沒有帶 tot_node.CLOSEOUT_SENTINEL({TN.CLOSEOUT_SENTINEL!r})"
    assert TN.LEGACY_ALL_HIT in pat, f"{rel} 丟掉了讀舊 log 的能力"


def test_kpi_constants_match_the_node():
    assert _KPI["CLOSEOUT"][1] == TN.CLOSEOUT_SENTINEL
    assert _KPI["LEGACY"][1] == TN.LEGACY_ALL_HIT


def test_make_artifacts_still_captures_the_closeout_line():
    """make_artifacts.sh 的門檻 3/4 摘錄也要抓得到結案行(四種形狀都要)。"""
    src = (REPO / "scripts/wsl/make_artifacts.sh").read_text(encoding="utf-8")
    pats = re.findall(r"grep -E '([^']+)'", src)
    assert pats, "make_artifacts.sh 的摘錄 grep 式萃取失敗"
    # 檔裡有好幾條 grep -E(門檻 1 抓彈射、門檻 3/4 抓結案),只要求**有一條**
    # 能抓到全部五種形狀 —— 那條就是門檻 3/4 的摘錄式。
    covering = [p for p in pats
                if all(re.search(p, summary_of(s)) for s in SHAPE_IDS)]
    assert covering, \
        ("make_artifacts.sh 沒有任何一條 grep -E 能抓到全部結案形狀 ⇒ "
         f"summary.md 的『門檻 3/4』會少掉結案行。現有的式子:{pats}")
