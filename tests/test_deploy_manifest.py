# -*- coding: utf-8 -*-
"""部署清單必須涵蓋鏈路真正會用到的每一個檔案。

═══ 為什麼要自動檢查,而不是靠人記得 ═══
整條驅動鏈跑的是 **`$SR = ~/fixwing_ws/sitl_run` 的部署版**,不是 repo:
    repro_two1.sh → run_west.sh → stot_repeat.sh → stot_trial.sh
全部用 `bash "$SR/…"`。而 `deploy_build.sh` 的 SCRIPTS 是**手維護的字串清單**,
漏一個就會靜默地跑舊版 —— 而且 `deploy_build.sh` 的一致性驗證只涵蓋 ROS2
package 那一層,漏檔照樣回報「✅ 部署一致」。

這件事已經發生兩次:
  1. `preflight_check.py` 漏列 ⇒ 「No such file」(deploy_build.sh 檔頭自己記著)
  2. 2026-08-06 `_kpi_parse.py` 漏列 ⇒ 部署版停在 8/4,**只認舊哨兵**
     「俯衝命中紅心:全員命中」。而 tot_node 只在「零 miss + 全部低空觸地」
     那一種結案形狀才寫這句 ⇒ 其餘三種形狀一律被記成 `no_summary`,
     一趟 5/6 命中的航次在遙測基準上長得跟 0 命中一樣。
     更糟:`stot_trial.sh` 呼叫它時 rc 是丟掉的,失敗還會讓 `tail -1 $CSV`
     印出**上一趟**的成績(而 stot_repeat.sh 的清場不碰 /tmp/stot_kpi.csv)。

⇒ 清單會再漏。這條測試把「鏈路引用到什麼」變成機器可查的事實。
"""
import pathlib
import re

import pytest

W = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "wsl"
DEPLOY = (W / "deploy_build.sh").read_text(encoding="utf-8")

# 驅動鏈上會去 $SR 拿東西的腳本
CHAIN = ["run_west.sh", "repro_two1.sh", "stot_repeat.sh", "stot_trial.sh",
         "swarm_ros2_up.sh", "swarm_sitl_up.sh", "make_artifacts.sh"]


def _manifest():
    scripts = re.search(r'SCRIPTS="(.*?)"', DEPLOY, re.S)
    nodes = re.search(r'NODES="(.*?)"', DEPLOY, re.S)
    launches = re.search(r'LAUNCHES="(.*?)"', DEPLOY, re.S)
    assert scripts and nodes, "deploy_build.sh 的 SCRIPTS / NODES 解析不出來"
    out = set(scripts.group(1).split()) | set(nodes.group(1).split())
    if launches:
        out |= set(launches.group(1).split())
    return out


def _referenced():
    """鏈路裡引用到的部署檔。

    ⚠️⚠️ 2026-08-07 修:第一版**只**比對 `$SR/<file>` 這個字面樣式,而
    `stot_trial.sh` 呼叫巡航腳本用的是

        ( cd "$SR" && timeout ... python3 jet_show_ros2.py )

    —— **沒有 `$SR/` 前綴**(前面先 `cd` 進去了)。於是這條測試掃不到它們,
    卻照樣全綠 ⇒ **假的安全網,比沒有更糟**。
    實際後果:`jet_show_cruise.py` / `jet_rescatter.py` / `stot_kpi_loop.sh` /
    `stot_rescore_all.py` 四支長期不在清單,WSL 跑的是舊版;其中
    `jet_show_cruise.py` 的「N 寫死 6」當天才修好,**但從來沒被部署過**。
    ⇒ 兩種樣式都要掃。
    """
    refs = {}
    pats = (
        re.compile(r"\$SR/([A-Za-z0-9_.\-]+)"),                 # "$SR/foo.py"
        # ⚠️ 這些呼叫常是**跨行**的(`\` 續行),所以不能用 [^\n]*? ——
        #    實測:單行樣式只抓到 jet_show_ros2.py,跨行樣式才抓得到
        #    jet_rescatter.py 與 jet_show_cruise.py(它們的 python3 在下一行)。
        re.compile(r'cd\s+"\$SR"\s*&&[\s\S]{0,200}?'            # cd "$SR" && … foo.py
                   r'(?:python3|bash|sh)\s+([A-Za-z0-9_.\-]+)'),
    )
    for c in CHAIN:
        p = W / c
        if not p.exists():
            continue
        src = p.read_text(encoding="utf-8")
        for pat in pats:
            for m in pat.finditer(src):
                name = m.group(1)
                if "." in name:                     # 過濾掉變數名之類的雜訊
                    refs.setdefault(name, set()).add(c)
    return refs


def test_every_referenced_script_is_in_the_manifest():
    listed = _manifest()
    missing = {f: sorted(w) for f, w in _referenced().items() if f not in listed}
    assert not missing, (
        "這些檔案被驅動鏈引用,但不在 deploy_build.sh 的部署清單 ⇒ "
        "WSL 會跑舊版而且沒有任何訊息:\n" +
        "\n".join("  %s  ← %s" % (f, "、".join(w)) for f, w in sorted(missing.items())))


def test_referenced_scripts_exist_in_repo():
    """引用了但 repo 裡根本沒有 = 部署後會 No such file。"""
    gone = {f: sorted(w) for f, w in _referenced().items() if not (W / f).exists()}
    assert not gone, "引用了不存在的檔案:%s" % gone


def test_deploy_script_lists_itself():
    """deploy_build.sh 不列自己 ⇒ 改了清單也送不過去(第一次踩的就是這個)。"""
    assert "deploy_build.sh" in _manifest()


@pytest.mark.parametrize("f", ["_kpi_parse.py", "swarm_ros2_down.sh",
                               "swarm_sitl_down.sh", "preflight_check.py"])
def test_previously_missed_files_stay_listed(f):
    """踩過的坑逐一釘死,避免下次重構時又被刪掉。"""
    assert f in _manifest(), f


class TestNodeStartupSmokeDoesNotDependOnSitl:
    """⚠️⚠️ 部署的步驟 5(節點啟動冒煙)**不可以**要求 SITL 先起來。

    部署跑在每趟 SITL **之前**(rep_interactive.sh 起 SITL 之前先 deploy),
    而 tot_node 建構到一半就會去連 MAVLink TCP ⇒ 沒有 SITL 必然 Errno 111。

    第一版寫成「沒印 `tot_node up` 就算失敗」,實際行為是:
        單獨手跑 deploy_build.sh → 剛好有殘留 SITL → **綠**
        走 rep_interactive.sh   → SITL 還沒起     → **紅** ⇒ 整批中止
    ⇒ 綠或紅取決於上一趟有沒有清乾淨。那不是閘門,是擲骰子 ——
      而且它擋掉的是**正確**的組態(2026-08-18 實際發生,S1 第一次開跑被自己擋下)。

    這一步要驗的是**參數層**(RRA 那顆地雷:`-p x:=` 空值解析失敗、
    哨兵字串被當 bool ⇒ InvalidParameterTypeException ⇒ 節點整個起不來)。
    參數在建構的最前面就處理完,所以「走到 MAVLink 連線」已經足夠涵蓋它。
    """

    SRC = (W / "deploy_build.sh").read_text(encoding="utf-8")

    def test_connection_refused_counts_as_pass(self):
        assert "ConnectionRefusedError" in self.SRC and "Errno 111" in self.SRC, (
            "⚠️⚠️ 步驟 5 沒有把「連不到 MAVLink」當成通過 ⇒ "
            "它會在正常流程裡永遠失敗,而單獨手跑時因為有殘留 SITL 而通過。")

    def test_it_still_catches_the_parameter_landmines(self):
        """⭐ 反向鎖:放寬不可以把它變成永遠通過。"""
        assert "InvalidParameterType" in self.SRC, (
            "⚠️⚠️ 步驟 5 不再檢查參數型別錯誤 —— 那是 RRA 整批 8 趟 / 5 小時報廢的成因")
        assert "Couldn't parse parameter" in self.SRC, (
            "⚠️⚠️ 步驟 5 不再檢查參數解析失敗(`-p name:=` 空值)")
        assert "fail=1" in self.SRC.split("5) 節點啟動冒煙")[1][:1800], (
            "⚠️ 步驟 5 已經沒有任何一條路會設 fail=1 ⇒ 它變成裝飾品")
