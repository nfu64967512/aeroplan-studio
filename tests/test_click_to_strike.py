"""地圖點擊 → WSL ROS2 STOT 全流程:橋接層的行為鎖。

⚠️ 這個檔案必須能在 **WSL 的 python3** 底下整份跑完(那邊沒有 PyQt6 /
   pyproj / shapely)。所以:
     * 只 import ``integrations.stot_ros2_bridge``,它是零相依的純邏輯層
     * Qt 那一層(ui/controllers/stot_ros2_controller.py)只做「有沒有語法錯」
       的編譯檢查,PyQt6 缺席時自動 skip

    python3 -m pytest tests/test_click_to_strike.py -q

⚠️ 不要跑 `pytest tests`(整包):WSL 缺 pyproj/PyQt6/shapely,會有 7 個無關
   模組 collection error。

────────────────────────────────────────────────────────────────────────
測什麼(對應四項驗收)
────────────────────────────────────────────────────────────────────────
  1. 指令組裝  —— 座標精度、PREFIX 不撞名、env 確實傳到 run_west.sh
  2. 前置檢查  —— 每一種失敗都擋得下來,而且 code 可辨識
  3. 相位解析  —— 各階段 log 片段 → 正確相位;**兩種結案哨兵都要認**
  4. 中止      —— 指令內容正確、殘留要讀回來驗證
"""
import re
import sys
from datetime import datetime
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from integrations import stot_ros2_bridge as B  # noqa: E402

WSL = REPO / "scripts" / "wsl"


# ═══════════════════════════════════════════════════════════════════════
#  0. 與上游腳本的耦合點 —— 從**原始碼**萃取,不抄常數
#     (抄一份的話,.sh 改了測試不會紅 —— 又是一個假迴歸鎖)
# ═══════════════════════════════════════════════════════════════════════

def test_closeout_sentinels_match_tot_node_source():
    """CLOSEOUT_SENTINEL / LEGACY_ALL_HIT 必須逐字等於 tot_node.py 裡的定義。"""
    src = (WSL / "sar_swarm_patches" / "tot_node.py").read_text(encoding="utf-8")
    m1 = re.search(r'^CLOSEOUT_SENTINEL\s*=\s*"([^"]+)"', src, re.M)
    m2 = re.search(r'^LEGACY_ALL_HIT\s*=\s*"([^"]+)"', src, re.M)
    assert m1 and m2, "萃取失敗 —— tot_node.py 的哨兵定義形狀變了,一律當測試失敗"
    assert B.CLOSEOUT_SENTINEL == m1.group(1)
    assert B.LEGACY_ALL_HIT == m2.group(1)


def test_home_and_rfan_match_wsl_scripts():
    """起飛點座標與扇形半徑必須跟 WSL 腳本一致 —— 幾何閘門是拿它們算的。"""
    up = (WSL / "swarm_sitl_up.sh").read_text(encoding="utf-8")
    assert "25.0777,121.2325" in up, "swarm_sitl_up.sh 的 TAOYUAN 座標變了"
    lat, lon = B.HOME_BY_LOC["TAOYUAN"]
    assert (lat, lon) == (25.0777, 121.2325)

    west = (WSL / "run_west.sh").read_text(encoding="utf-8")
    m = re.search(r'export R_FAN="\$\{R_FAN:-([\d.]+)\}"', west)
    assert m, "run_west.sh 的 R_FAN 預設形狀變了"
    assert B.R_FAN_DEFAULT == float(m.group(1))


def test_run_west_still_takes_the_env_we_override():
    """我們覆寫的四個變數必須真的是 run_west.sh 的 ${VAR:-預設} 旗標。"""
    west = (WSL / "run_west.sh").read_text(encoding="utf-8")
    for var in ("TGT_LAT", "TGT_LON", "TARGET_AMSL", "PREFIX", "TRIALS"):
        assert re.search(r'\b%s="\$\{%s:-' % (var, var), west), \
            "%s 不再是可覆寫旗標 —— 橋接會靜默失效" % var
    # APPROACH_FROM 刻意**不在** run_west.sh 裡:它只在 stot_trial.sh 有預設 180.0，
    # 我們靠環境變數穿透過去。這條斷言就是在鎖「穿透仍然成立」。
    assert "APPROACH_FROM" not in west
    trial = (WSL / "stot_trial.sh").read_text(encoding="utf-8")
    assert "${APPROACH_FROM:-180.0}" in trial


def test_run_west_execs_stot_repeat_which_rebuilds_sitl():
    """鎖住「按下按鈕 = SITL 會被拆掉重建」這個副作用。

    交接文件曾寫「run_west.sh 不會重啟 SITL」—— 對這個檔案本身成立,對整條鏈
    不成立。若哪天上游改掉了,UI 的警告文案就該一起改。
    """
    west = (WSL / "run_west.sh").read_text(encoding="utf-8")
    assert 'exec bash "$SR/stot_repeat.sh"' in west
    rep = (WSL / "stot_repeat.sh").read_text(encoding="utf-8")
    assert "swarm_sitl_down.sh" in rep and "swarm_sitl_up.sh" in rep


# ═══════════════════════════════════════════════════════════════════════
#  1. 指令組裝
# ═══════════════════════════════════════════════════════════════════════

TGT = (24.1234567, 119.9431500)


def _script_of(cmd):
    """從 argv 取出 `bash -lc` 的那一段。"""
    assert cmd[0] == "wsl.exe"
    assert cmd[-3:-1] == ["bash", "-lc"]
    return cmd[-1]


def test_run_command_shape():
    cmd = B.build_run_command(TGT[0], TGT[1], "AP0805A")
    assert cmd[:5] == ["wsl.exe", "-d", B.WSL_DISTRO_DEFAULT, "-e", "bash"]
    s = _script_of(cmd)
    assert "bash run_west.sh" in s
    assert "cd ~/fixwing_ws/sitl_run" in s
    assert "PREFIX=AP0805A" in s
    assert "TRIALS=1" in s
    # stdout 在 WSL 端就落檔 —— Windows 端不開 PIPE(sitl_launcher.py:658 的教訓)
    assert "> ~/fixwing_ws/runs/driver_AP0805A.log 2>&1" in s


def test_coords_are_seven_decimals_not_repr():
    """座標精度 ≥6 位,而且**永遠帶小數點**。

    stot_trial.sh:376 的 tlat/tlon 沒有套 _f() 小數點修補 ⇒ 送出「121」這種
    看起來像整數的字串,ROS 2 會判成 INTEGER 而參數宣告是 DOUBLE,節點一起來
    就 InvalidParameterTypeException(對外表現成 rc=3 節點提前死亡)。
    """
    s = _script_of(B.build_run_command(25.0, 121.0, "AP0805A"))
    assert "TGT_LAT=25.0000000" in s
    assert "TGT_LON=121.0000000" in s
    for tok in re.findall(r"TGT_(?:LAT|LON)=(\S+)", s):
        assert "." in tok, "座標沒有小數點 → ROS2 會判成 INTEGER"
        assert len(tok.split(".")[1]) >= 6


def test_clicked_point_is_carried_verbatim():
    """使用者點的那一點必須原樣出現在指令裡(誤差 < 1e-7 度)。"""
    s = _script_of(B.build_run_command(TGT[0], TGT[1], "AP0805A"))
    lat = float(re.search(r"TGT_LAT=(\S+)", s).group(1))
    lon = float(re.search(r"TGT_LON=(\S+)", s).group(1))
    assert abs(lat - TGT[0]) < 1e-7
    assert abs(lon - TGT[1]) < 1e-7


def test_env_is_exposed_for_ui_and_log():
    """UI 與 log 要印的就是這個 dict —— 順序固定,值與指令一致。"""
    env = B.build_launch_env(TGT[0], TGT[1], "AP0805A")
    assert list(env) == ["TGT_LAT", "TGT_LON", "APPROACH_FROM",
                         "TARGET_AMSL", "PREFIX", "TRIALS"]
    s = _script_of(B.build_run_command(TGT[0], TGT[1], "AP0805A"))
    for k, v in env.items():
        assert "%s=%s" % (k, v) in s


def test_target_amsl_switch():
    """海面 0.0 / 陸地 .nan。**必須是字面小數點的 .nan** —— 寫 nan 會被 YAML
    當字串,節點型別檢查當場掛掉(stot_trial.sh:394)。"""
    sea = B.build_launch_env(*TGT, prefix="AP0805A", sea_level_target=True)
    land = B.build_launch_env(*TGT, prefix="AP0805A", sea_level_target=False)
    assert sea["TARGET_AMSL"] == "0.0"
    assert land["TARGET_AMSL"] == ".nan"


def test_approach_from_points_back_at_home_not_the_hardcoded_180():
    """扇形要擺在「起飛點的方向」,不能沿用 stot_trial.sh 的 180.0。"""
    home = B.HOME_BY_LOC["TAOYUAN"]
    # 目標在起飛點正西 ⇒ 從目標看回起飛點是正東(90°)
    west_of_home = (home[0], home[1] - 1.3)
    assert abs(B.approach_from_for(west_of_home, home) - 90.0) < 1.5
    # 目標在起飛點正南 ⇒ 從目標看回起飛點是正北(0°/360°)
    south_of_home = (home[0] - 1.0, home[1])
    b = B.approach_from_for(south_of_home, home) % 360.0
    assert min(b, 360.0 - b) < 1.5
    # 而且它確實進了指令
    env = B.build_launch_env(west_of_home[0], west_of_home[1], "AP0805A")
    assert env["APPROACH_FROM"] != "180.0"


def test_no_hardcoded_target_in_bridge_source():
    """紅心座標絕不可以寫死在橋接層任何地方。

    run_west.sh 的預設目標是 (25.0777, 119.94315);橋接層若出現這組數字,
    代表有人把示範座標留在程式裡了。
    """
    src = (REPO / "integrations" / "stot_ros2_bridge.py").read_text(encoding="utf-8")
    code = "\n".join(l for l in src.splitlines()
                     if not l.lstrip().startswith("#"))
    assert "119.94315" not in code
    ctrl = (REPO / "ui" / "controllers" / "stot_ros2_controller.py").read_text(encoding="utf-8")
    assert "119.94315" not in ctrl


@pytest.mark.parametrize("bad", [
    (float("nan"), 121.0), (25.0, float("inf")),
    (95.0, 121.0), (25.0, 181.0),
])
def test_bad_coords_raise(bad):
    with pytest.raises(B.StotBridgeError):
        B.build_run_command(bad[0], bad[1], "AP0805A")


@pytest.mark.parametrize("bad", ["", "ap0805a", "AP 0805", "A;rm -rf /", "1AP",
                                 "AP$(whoami)", "A" * 20])
def test_bad_prefix_raises(bad):
    """PREFIX 是唯一有機會夾帶 shell metacharacter 的欄位 —— 白名單擋死。"""
    with pytest.raises(B.StotBridgeError):
        B.build_run_command(25.0, 121.0, bad)


# ═══════════════════════════════════════════════════════════════════════
#  2. PREFIX 不撞名(撞名 = 靜默刪掉舊航次的原始資料)
# ═══════════════════════════════════════════════════════════════════════

_NOW = datetime(2026, 8, 5, 14, 23)


def test_prefix_shape_and_trial_name():
    p = B.make_prefix([], now=_NOW)
    assert p == "AP0805A"
    assert B.trial_name(p) == "AP0805A1"


def test_prefix_skips_taken_trials():
    """taken 裡放的是**航次代號**(PREFIX+1),不是 PREFIX。"""
    p = B.make_prefix(["AP0805A1", "AP0805B1"], now=_NOW)
    assert p == "AP0805C"


def test_prefix_avoids_all_161_existing_runs():
    """既有 161 個航次代號都是 [A-Z]+[0-9]+ 形狀,AP 開頭的一個都沒有。"""
    existing = ["A21", "SW1", "P91", "RD1", "EL1", "D451", "CA1", "AM31",
                "R451", "RP1", "SG1", "DEMO1", "FULL1", "NEW1", "V901"]
    p = B.make_prefix(existing, now=_NOW)
    assert B.trial_name(p) not in existing


def test_prefix_matching_is_case_insensitive():
    assert B.make_prefix(["ap0805a1"], now=_NOW) == "AP0805B"


def test_prefix_exhausted_raises_loudly():
    taken = ["AP0805%s1" % c for c in "ABCDEFGHIJKLMNOPQRSTUVWXYZ"]
    with pytest.raises(B.StotBridgeError) as ei:
        B.make_prefix(taken, now=_NOW)
    assert "AP0805" in str(ei.value)


# ═══════════════════════════════════════════════════════════════════════
#  3. 前置檢查 —— 每一種失敗都要擋得下來,而且訊息可辨識
# ═══════════════════════════════════════════════════════════════════════

#: 一切正常的 WSL 前置檢查輸出(SITL 起著、ROS2 沒起、沒有上一趟)
CLEAN = """HOME=/home/windows-wsl
ARDUPLANE=6
MAVROS=0
SWARMNODE=0
TOTNODE=0
TRIALSH=0
REPEATSH=0
WESTSH=0
SHOWPY=0
""" + "".join("SRFILE=%s\n" % s for s in B.REQUIRED_SCRIPTS) + """TAKEN=P91
TAKEN=SW1
TAKEN=RD1
PREFLIGHT_OK=1
"""

#: 遠離桃園的海上紅心(約 130 km 西)—— 幾何閘門過得了
FAR_TARGET = (25.0777, 119.94315)


def _codes(rep, target=FAR_TARGET, **kw):
    return {i.code for i in B.blocking_issues(
        B.evaluate_preflight(rep, target=target, **kw))}


def test_preflight_parse():
    rep = B.parse_preflight(CLEAN)
    assert rep.ok is True
    assert rep.wsl_home == "/home/windows-wsl"
    assert rep.arduplane == 6 and rep.mavros == 0 and rep.tot_node == 0
    assert rep.taken == ["P91", "SW1", "RD1"]
    assert rep.missing == []


def test_clean_state_passes():
    assert _codes(B.parse_preflight(CLEAN)) == set()


def test_clean_state_still_warns_about_sitl_restart():
    """SITL 重建是**非阻擋**的告知 —— 但一定要講,不然使用者不知道連線為何斷。"""
    issues = B.evaluate_preflight(B.parse_preflight(CLEAN), target=FAR_TARGET)
    warn = [i for i in issues if i.code == "SITL_WILL_RESTART"]
    assert warn and warn[0].blocking is False
    assert "155" in warn[0].message


def test_no_target_blocks():
    assert "NO_TARGET" in _codes(B.parse_preflight(CLEAN), target=None)


def test_wsl_unreachable_blocks_and_stops_further_inference():
    """缺 PREFLIGHT_OK ⇒ 底下的數字全部不可信,只報這一條。"""
    rep = B.parse_preflight("HOME=/home/x\nARDUPLANE=0\n")
    issues = B.blocking_issues(B.evaluate_preflight(rep, target=FAR_TARGET))
    assert [i.code for i in issues] == ["WSL_UNREACHABLE"]


def test_missing_scripts_block_and_are_named():
    dropped = CLEAN.replace("SRFILE=stot_trial.sh\n", "") \
                   .replace("SRFILE=make_artifacts.sh\n", "")
    rep = B.parse_preflight(dropped)
    assert set(rep.missing) == {"stot_trial.sh", "make_artifacts.sh"}
    issues = B.evaluate_preflight(rep, target=FAR_TARGET)
    hit = [i for i in issues if i.code == "MISSING_SCRIPTS"]
    assert hit and hit[0].blocking
    assert "stot_trial.sh" in hit[0].message and "make_artifacts.sh" in hit[0].message
    assert "deploy_build.sh" in hit[0].message   # 要講處置,不能只講症狀


def test_empty_sitl_run_dir_reports_every_script_missing():
    """~/fixwing_ws/sitl_run 整個不在時,nullglob 讓迴圈是空的 —— 必須擋下來。"""
    rep = B.parse_preflight("HOME=/home/x\nARDUPLANE=6\nPREFLIGHT_OK=1\n")
    assert set(rep.missing) == set(B.REQUIRED_SCRIPTS)
    assert "MISSING_SCRIPTS" in _codes(rep)


@pytest.mark.parametrize("key", ["TOTNODE", "TRIALSH", "REPEATSH", "WESTSH", "SHOWPY"])
def test_previous_run_still_alive_blocks(key):
    """絕對不能平行跑兩趟 —— 第二趟的無差別 pkill 會靜默殺掉第一趟。"""
    rep = B.parse_preflight(CLEAN.replace(key + "=0", key + "=1"))
    assert "RUN_IN_PROGRESS" in _codes(rep)


def test_local_proc_alive_blocks_even_if_wsl_looks_clean():
    """AeroPlan 自己還握著驅動行程時也要擋(WSL 端可能剛好在 sleep 150)。"""
    rep = B.parse_preflight(CLEAN)
    assert "RUN_IN_PROGRESS" in _codes(rep, local_run_active=True)


@pytest.mark.parametrize("key", ["MAVROS", "SWARMNODE"])
def test_ros2_already_up_blocks(key):
    """⚠️ 最反直覺的一條:ROS2 層**不該**事先起好。

    stot_trial.sh:354 自己拉起、:358 自己收掉;預先起好的會在
    swarm_ros2_up.sh:141 被 pkill 重來,而且 MANUAL=0 起的節點會跟彈射搶控制。
    """
    rep = B.parse_preflight(CLEAN.replace(key + "=0", key + "=6"))
    issues = B.evaluate_preflight(rep, target=FAR_TARGET)
    hit = [i for i in issues if i.code == "ROS2_ALREADY_UP"]
    assert hit and hit[0].blocking
    assert "swarm_ros2_down.sh" in hit[0].message


def test_sitl_not_running_is_not_a_blocker():
    """SITL 沒起來**不是**失敗前置 —— stot_repeat.sh 自己會重建。"""
    rep = B.parse_preflight(CLEAN.replace("ARDUPLANE=6", "ARDUPLANE=0"))
    assert _codes(rep) == set()


def test_target_too_close_blocks():
    """紅心離起飛點 < 35 km × 1.15 ⇒ 起飛點落在扇形內,整條航路是反的。

    **沒有任何 WSL 腳本會擋這一關**(stot_repeat.sh:58-61 只寫在註解裡)。
    """
    home = B.HOME_BY_LOC["TAOYUAN"]
    near = (home[0], home[1] - 0.2)          # 約 20 km
    assert B.haversine_m(home, near) < B.R_FAN_DEFAULT
    issues = B.evaluate_preflight(B.parse_preflight(CLEAN), target=near)
    hit = [i for i in issues if i.code == "TARGET_TOO_CLOSE"]
    assert hit and hit[0].blocking
    assert "km" in hit[0].message            # 要把實際距離報出來


def test_target_just_outside_margin_passes():
    home = B.HOME_BY_LOC["TAOYUAN"]
    far = (home[0], home[1] - 0.50)          # 約 50 km > 35 km × 1.15
    assert B.haversine_m(home, far) > B.R_FAN_DEFAULT * B.GEOMETRY_MARGIN
    assert "TARGET_TOO_CLOSE" not in _codes(B.parse_preflight(CLEAN), target=far)


def test_every_issue_str_is_prefixed_for_the_dialog():
    rep = B.parse_preflight(CLEAN.replace("MAVROS=0", "MAVROS=6"))
    for i in B.evaluate_preflight(rep, target=None):
        assert str(i)[0] in "✗⚠"


def test_preflight_command_never_touches_mavlink_ports():
    """『順手測一下埠通不通』會把 AeroPlan 餓死 —— MAVProxy 的 tcpin 只 accept 一次。"""
    s = _script_of(B.build_preflight_command())
    for banned in ("6760", "6960", "5760", "5762", "5860", "nc ", "telnet",
                   "mavproxy", "/dev/tcp"):
        assert banned not in s, "前置檢查碰了 %r —— 會餓死 AeroPlan 的連線" % banned


def test_preflight_command_counts_with_wc_not_pgrep_dash_c():
    """`pgrep -c` 沒命中時會印 0 **且** exit 1,套 `|| echo 0` 會變成「0 0」。"""
    s = _script_of(B.build_preflight_command())
    assert "pgrep -c" not in s
    assert s.count("| wc -l") >= 8


def test_preflight_command_does_not_match_itself():
    """⚠️⚠️ 迴歸鎖(2026-08-05,實測抓到):

    這段指令自己的 cmdline 就是 ``pgrep -f`` 的掃描對象。第一版把
    REQUIRED_SCRIPTS 展開成 ``for f in run_west.sh stot_trial.sh …``,結果
    ``pgrep -f '[s]tot_trial.sh'`` 匹配到 bash 自己與它的命令替換子殼 ——
    實測 TRIALSH=2 / WESTSH=2,「上一趟還在跑」會在**每一次**啟動時誤報。

    規則:凡是拿來 ``pgrep -f`` 的名字,在整段腳本裡只准以中括號形式出現。
    """
    s = _script_of(B.build_preflight_command())
    patterns = re.findall(r"pgrep -f '([^']+)'", s)
    assert patterns, "萃取失敗 —— pgrep -f 的寫法變了"
    for pat in patterns:
        for alt in pat.split("|"):
            plain = alt.replace("[", "").replace("]", "")
            assert plain not in s, (
                "腳本裡出現了未加中括號的 %r,pgrep -f 會匹配到指令自己" % plain)


def test_preflight_lists_the_dir_and_python_computes_missing():
    """缺哪幾支在 Python 端算 —— 腳本裡不可以出現裸的腳本名(見上一條)。"""
    s = _script_of(B.build_preflight_command())
    assert 'echo "SRFILE=' in s
    for name in ("run_west.sh", "stot_trial.sh", "stot_repeat.sh"):
        assert name not in s
    assert set(B.REQUIRED_SCRIPTS) >= {"run_west.sh", "stot_trial.sh",
                                       "stot_repeat.sh", "preflight_check.py"}


# ═══════════════════════════════════════════════════════════════════════
#  4. 相位解析
# ═══════════════════════════════════════════════════════════════════════

def _track(*lines):
    t = B.PhaseTracker()
    for l in lines:
        t.feed_line(l)
    return t


@pytest.mark.parametrize("line,phase", [
    ("[west] 目標 (25.0777, 119.94315) ← 起飛 TAOYUAN", B.PHASE_STARTING),
    ("════════════════ REPEAT AP0805A1 ════════════════", B.PHASE_SITL_REBUILD),
    ("[1/3] 彈射起飛…", B.PHASE_CATAPULT),
    ("[2/3] ROS2 編隊層長程巡航(隊形 V)…", B.PHASE_CRUISE),
    ("[3/3] STOT + 俯衝 → /home/x/fixwing_ws/runs/trial_AP0805A1.log", B.PHASE_TOT_INIT),
    ("[INFO] [tot_node]: ALIGN：T_arr=+420s;各機距 slot {1: 7500}", B.PHASE_ALIGN),
    ("[WARN] [tot_node]: ★ 全員抵扇形。抵達散布 = 77.7s → PERCH(對準內切)", B.PHASE_PERCH),
    ("[WARN] [tot_node]: ★ 全員就緒(alt {1: 3100})→ 同步內切到環,預計 210s", B.PHASE_PERCH),
    ("[WARN] [tot_node]: ★ 全員對準內切(alt {1: 3100};高度收斂 40m)→ DIVE 直線俯衝穿越!",
     B.PHASE_DIVE),
])
def test_single_marker_gives_phase(line, phase):
    assert _track(line).phase == phase


def test_phases_advance_in_order_through_a_whole_run():
    t = B.PhaseTracker()
    seen = []
    for line in (
        "[west] 目標 (25.0777, 119.94315) ← 起飛 TAOYUAN",
        "════════════════ REPEAT AP0805A1 ════════════════",
        "[1/3] 彈射起飛…",
        "  UAV1 彈射 @+0s",
        "  全員 GUIDED",
        "[2/3] ROS2 編隊層長程巡航(隊形 V)…",
        "  t+120s [CRUISE] 長機距T 90000m alt 3100",
        "[3/3] STOT + 俯衝 → runs/trial_AP0805A1.log",
        "[INFO] [tot_node]: ALIGN：T_arr=+420s;各機距 slot {}",
        "[WARN] [tot_node]: ★ 全員抵扇形。抵達散布 = 77.7s → PERCH(對準內切)",
        "[WARN] [tot_node]: ★ 全員對準內切(alt {})→ DIVE 直線俯衝穿越!",
    ):
        t.feed_line(line)
        seen.append(t.phase)
    assert seen == [
        B.PHASE_STARTING, B.PHASE_SITL_REBUILD, B.PHASE_CATAPULT,
        B.PHASE_CATAPULT, B.PHASE_CATAPULT, B.PHASE_CRUISE, B.PHASE_CRUISE,
        B.PHASE_TOT_INIT, B.PHASE_ALIGN, B.PHASE_PERCH, B.PHASE_DIVE,
    ]


def test_phase_never_goes_backwards():
    """兩個 log 檔是交錯讀進來的,不保證時間有序 —— UI 不可以從 DIVE 跳回 CRUISE。"""
    t = _track("[WARN] [tot_node]: ★ 全員對準內切 → DIVE 直線俯衝穿越!")
    assert t.phase == B.PHASE_DIVE
    t.feed_line("[2/3] ROS2 編隊層長程巡航(隊形 V)…")
    assert t.phase == B.PHASE_DIVE


# ── 結案哨兵:兩種形狀都要認 ──────────────────────────────────────

def test_closeout_sentinel_new_shape():
    """新形狀 —— 哨兵與達標描述分離(RD1 那種「名義命中但高空掠過」也認得)。"""
    t = _track("[WARN] [tot_node]: ★★ 俯衝結案:6/6 命中、0 MISS。"
               "⚠️ 名義命中,但 6/6 架在高空掠過(未落地)")
    assert t.phase == B.PHASE_CLOSED
    assert t.finished and t.state.closed_out


def test_closeout_sentinel_legacy_shape():
    """舊形狀 —— 歷史 log 裡還有,不能漏掉。"""
    t = _track("[WARN] [tot_node]: ★★ 俯衝命中紅心:全員命中。散布 3.0s")
    assert t.phase == B.PHASE_CLOSED
    assert t.finished


def test_closeout_wins_over_phase_markers():
    t = _track("[3/3] STOT + 俯衝 → x.log", B.CLOSEOUT_SENTINEL + "6/6 命中")
    assert t.phase == B.PHASE_CLOSED


# ── 失敗標記 ────────────────────────────────────────────────────

@pytest.mark.parametrize("line", [
    "  ✗ SITL 起不來",
    "✗ SITL 不是 6 架(現在 4)—— 先跑 swarm_sitl_up.sh",
    "✗ 開跑前驗證未通過 —— 中止(設 STRICT=0 可強制略過,但不建議)",
    "✗ CRUISE_MODE=ros2 但 CLIMB_TO_ALT=100(<1000 m)。",
    "  ✗ 彈射失敗",
    "  ✗ tot_node(PID 1234)在結案前就不在了 —— **不是逾時**:",
    "  ✗ 已命中 6/6,但摘要行還沒印,預算就用完了 —— **不是飛不完**:",
    "  ✗ 逾時(只命中 2/6);最後階段:",
])
def test_failure_markers_end_the_run(line):
    t = _track(line)
    assert t.phase == B.PHASE_FAILED
    assert t.finished
    assert t.state.failed_reason


def test_failure_reason_is_shown_to_the_user():
    t = _track("✗ SITL 不是 6 架(現在 4)")
    assert "6 架" in t.status_text() or "SITL" in t.status_text()


# ── 抓值:PGID / 命中 / 目標回讀 ────────────────────────────────

def test_tot_pgid_is_captured_for_abort():
    """中止一定要拿到 PGID —— tot_node 是 setsid nohup,殺上游殺不到它。"""
    t = _track("[trial] ros2 run wrapper PID=12345 / 行程群組 PGID=12345 "
               "(收尾殺整個群組)")
    assert t.state.tot_pgid == 12345


def test_hits_and_misses_are_counted_once_per_uav():
    t = _track(
        "[INFO] [tot_node]: UAV1 命中紅心 @+41.20s (鎖存當下 3m…)",
        "[INFO] [tot_node]: UAV1 命中紅心 @+41.20s (重複行)",
        "[INFO] [tot_node]: UAV2 命中紅心 @+42.10s (鎖存當下 5m…)",
        "[WARN] [tot_node]: ✗ UAV3 判定 MISS @+60s(自 dive_go)",
    )
    assert t.state.hits == {1, 2}
    assert t.state.misses == {3}
    assert "命中 2 / MISS 1" in t.status_text()


def test_target_echo_from_driver_and_node():
    t = _track(
        "[west] 目標 (24.1234567, 119.94315) ← 起飛 TAOYUAN",
        "[INFO] [tot_node]: tot_node up：扇形 R=35000 ±25° 6 slot 間隔 3000m;"
        "pushover R=8600 perch=3100m → 俯衝 → 目標 (24.1234567, 119.94315)",
    )
    assert t.state.echoed_target_west == pytest.approx((24.1234567, 119.94315))
    assert t.state.echoed_target_node == pytest.approx((24.1234567, 119.94315))
    assert B.target_matches((24.1234567, 119.94315), t.state.echoed_target_node)


def test_target_mismatch_is_detected():
    """『不要相信自己送出的值』—— 回讀不符必須看得出來。"""
    assert not B.target_matches((24.1234567, 119.94315), (24.5, 119.94315))
    assert not B.target_matches((24.1234567, 119.94315), None)


def test_status_text_carries_trial_and_phase():
    t = _track("[3/3] STOT + 俯衝 → x.log")
    s = t.status_text("AP0805A1")
    assert "AP0805A1" in s and "[3/3]" in s


def test_feed_reports_change_only_when_something_moved():
    t = B.PhaseTracker()
    assert t.feed_line("[1/3] 彈射起飛…") is True
    assert t.feed_line("  一行無關的輸出") is False
    assert t.feed_line("") is False


def test_feed_handles_multiline_chunks():
    t = B.PhaseTracker()
    assert t.feed("[1/3] 彈射起飛…\n[2/3] ROS2 編隊層長程巡航(隊形 V)…\n")
    assert t.phase == B.PHASE_CRUISE


# ═══════════════════════════════════════════════════════════════════════
#  5. 中止
# ═══════════════════════════════════════════════════════════════════════

def test_abort_kills_the_tot_process_group_first():
    """tot_node 是 setsid nohup 起的(stot_trial.sh:375),而 :195 的 TERM trap
    跳過了 :515-529 的 PGID 收尾 ⇒ 光殺上游會留下一個還在指揮六架飛機的孤兒。"""
    s = _script_of(B.build_abort_command(4242))
    assert "kill -TERM -4242" in s
    assert "kill -KILL -4242" in s
    assert s.index("kill -TERM -4242") < s.index("swarm_ros2_down.sh")


def test_abort_without_pgid_still_tears_down():
    s = _script_of(B.build_abort_command(None))
    # 沒有 PGID 就不准發任何 group signal —— `kill -TERM -<空>` 會變成殺自己
    assert not re.search(r"(?<!p)kill\s+-(TERM|KILL)\s+-", s)
    assert 'pkill -f "[t]ot_node"' in s
    assert "swarm_ros2_down.sh" in s


def test_abort_uses_pkill_dash_f_not_dash_x():
    """`ros2 run` 是 Popen+communicate(無 exec/setproctitle)⇒ 節點的 comm 其實是
    python3,`pkill -x tot_node` 打不中。中括號是避免 pkill 匹配到自己這行。"""
    s = _script_of(B.build_abort_command(None))
    assert "pkill -x tot_node" not in s
    assert '[t]ot_node' in s


def test_abort_never_kills_the_users_sitl():
    """留著 arduplane / mavproxy,使用者才能立刻重連;殺掉只是多等 150 秒。"""
    s = _script_of(B.build_abort_command(None))
    assert "pkill -x arduplane" not in s
    assert "swarm_sitl_down" not in s
    assert "mavproxy" not in s


def test_abort_covers_the_whole_chain():
    s = _script_of(B.build_abort_command(None))
    for pat in ("[j]et_show_ros2", "[s]tot_trial.sh", "[s]tot_repeat.sh",
                "[r]un_west.sh"):
        assert pat in s


def test_abort_command_does_not_match_itself():
    """同一個坑的另一半:abort 的 pkill 不可以打到自己這段指令,否則會自殺 ——
    ``pkill`` 殺掉自己的 shell 之後,後面的 swarm_ros2_down.sh 與 RESID 讀回
    驗證都不會執行,而使用者只會看到「中止完成」。"""
    s = _script_of(B.build_abort_command(4242))
    pats = re.findall(r'pkill -f "([^"]+)"', s) + re.findall(r"pgrep -f '([^']+)'", s)
    assert pats
    for pat in pats:
        plain = pat.replace("[", "").replace("]", "")
        assert plain not in s, "abort 腳本會 pkill 掉自己:%r" % plain


def test_abort_reads_residuals_back():
    """拆完不驗證等於沒拆。"""
    s = _script_of(B.build_abort_command(None))
    assert "RESID" in s
    for k in ("tot=", "mavros=", "show=", "trial=", "west=", "arduplane="):
        assert k in s


def test_parse_resid_clean():
    r = B.parse_resid("RESID tot=0 mavros=0 show=0 trial=0 west=0 arduplane=6\n")
    assert r["tot"] == 0 and r["arduplane"] == 6
    assert B.resid_leftovers(r) == {}


def test_parse_resid_leftovers_exclude_arduplane():
    r = B.parse_resid("RESID tot=1 mavros=0 show=0 trial=0 west=0 arduplane=6")
    left = B.resid_leftovers(r)
    assert left == {"tot": 1}, "arduplane 是刻意留下的,不算殘留"


def test_parse_resid_missing_raises_loudly():
    """沒有 RESID 就代表『無法確認是否收乾淨』—— 絕不可以當成成功。"""
    with pytest.raises(B.StotBridgeError):
        B.parse_resid("something went wrong\n")


@pytest.mark.parametrize("bad", [0, 1, -5])
def test_abort_rejects_bogus_pgid(bad):
    """`kill -TERM -1` 會殺掉整台機器上能殺的一切。"""
    with pytest.raises(B.StotBridgeError):
        B.build_abort_command(bad)


# ═══════════════════════════════════════════════════════════════════════
#  6. 產物解析
# ═══════════════════════════════════════════════════════════════════════

def test_parse_geom_target():
    csv = "kind,lat,lon\ntarget,24.1234567,119.9431500\nslot0,24.2,119.8\n"
    assert B.parse_geom_target(csv) == pytest.approx((24.1234567, 119.94315))


def test_parse_geom_target_missing():
    assert B.parse_geom_target("kind,lat,lon\nslot0,24.2,119.8\n") is None
    assert B.parse_geom_target("") is None


def test_parse_summary_row():
    csv = ("trial,n_touch,gs_touch_mean,gs_touch_min,gs_touch_max,"
           "max_impact_miss_m,max_abs_alt_m,touch_spread_s,pass\n"
           "P91,6,180,175,190,3.0,0.4,3.0,1\n"
           "AP0805A1,6,182,178,188,4.2,0.6,2.8,1\n")
    row = B.parse_summary_row(csv, "AP0805A1")
    assert row["pass"] == "1" and row["max_impact_miss_m"] == "4.2"
    assert B.parse_summary_row(csv, "NOPE1") is None


def test_summary_header_matches_stot_repeat_source():
    rep = (WSL / "stot_repeat.sh").read_text(encoding="utf-8")
    assert ("trial,n_touch,gs_touch_mean,gs_touch_min,gs_touch_max,"
            "max_impact_miss_m,max_abs_alt_m,touch_spread_s,pass") in rep


def test_describe_rc_covers_the_documented_codes():
    for rc in (0, 1, 2, 3, 4, 130):
        assert B.describe_rc(rc).startswith("rc=%d(" % rc)
    assert B.describe_rc(None) == "尚未結束"


# ═══════════════════════════════════════════════════════════════════════
#  7. Qt 那一層 —— 沒有 PyQt6 就 skip,不要在 WSL 爆掉
# ═══════════════════════════════════════════════════════════════════════

def test_controller_module_compiles():
    """WSL 沒有 PyQt6,所以只做語法檢查(compile),不 import。"""
    import py_compile
    src = REPO / "ui" / "controllers" / "stot_ros2_controller.py"
    py_compile.compile(str(src), doraise=True, cfile=str(src) + "c")


def test_bridge_imports_stdlib_only():
    """Qt 層 → 純邏輯層 是單向相依,而且純邏輯層只准用 stdlib。

    多一個相依,這份測試就不再能在 WSL 的 python3 底下跑 —— 而那正是唯一
    能順手驗證 WSL 腳本耦合點的地方。
    """
    allowed = {"math", "re", "collections", "dataclasses", "datetime",
               "typing", "__future__"}
    src = (REPO / "integrations" / "stot_ros2_bridge.py").read_text(encoding="utf-8")
    for line in src.splitlines():
        m = re.match(r"^(?:from|import)\s+([A-Za-z_][\w.]*)", line.strip())
        if m:
            assert m.group(1).split(".")[0] in allowed, \
                "純邏輯層 import 了 %s —— 就不再能在 WSL 單測了" % m.group(1)


def test_logger_calls_pass_one_preformatted_string():
    """⚠️ 迴歸鎖(2026-08-05,實測抓到):

    本專案的 ``utils.logger.get_logger()`` 回傳的是自訂 Logger,簽章是
    ``info(message: str, exc_info: bool = False)`` —— **不吃 %-args**。
    寫成 stdlib 慣用的 ``logger.info("… %s", x)`` 會在執行期丟 TypeError,
    而那些呼叫全都在錯誤處理路徑上(前置檢查失敗、中止、殘留告警),
    正好是最不想再爆一次的地方。
    """
    import ast
    src = (REPO / "ui" / "controllers" / "stot_ros2_controller.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    bad = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "logger"):
            if len(node.args) != 1:
                bad.append((node.lineno, node.func.attr, len(node.args)))
    assert not bad, ("logger 只吃一個已格式化的字串,這些呼叫傳了多個位置參數:%s" % bad)


def test_controller_is_wired_into_main_window():
    """接線斷了會表現成「按鈕沒反應」—— 典型的靜默失敗,用測試鎖住。"""
    mw = (REPO / "ui" / "main_window.py").read_text(encoding="utf-8")
    assert "StotRos2ControllerMixin" in mw
    assert "stot_ros2_run_requested.connect(self._on_stot_ros2_run)" in mw
    assert "stot_ros2_abort_requested.connect(self._on_stot_ros2_abort)" in mw

    panel = (REPO / "ui" / "widgets" / "parameter_panel.py").read_text(encoding="utf-8")
    assert "stot_ros2_run_requested = pyqtSignal()" in panel
    assert "stot_ros2_abort_requested = pyqtSignal()" in panel
    assert "_stot_run_btn" in panel and "_stot_abort_btn" in panel
    assert "_stot_status_label" in panel


def test_existing_strike_path_is_untouched():
    """這次是**新增**一條路,不是取代 —— strike_controller.py 不准出現新東西。"""
    sc = (REPO / "ui" / "controllers" / "strike_controller.py").read_text(encoding="utf-8")
    for token in ("run_west", "wsl.exe", "stot_ros2", "_on_stot_ros2_run"):
        assert token not in sc, "既有打擊路徑被動到了:%r" % token


# ═══════════════════════════════════════════════════════════════════════
#  8. Controller 的實際行為 —— 需要 PyQt6,WSL 會整段 skip
#     subprocess 全部打樁:這裡**不會**碰到真的 WSL,也不會起任何東西
# ═══════════════════════════════════════════════════════════════════════

try:                                   # pragma: no cover - 環境相關
    import PyQt6.QtWidgets             # noqa: F401
    _HAS_QT = True
except Exception:                      # WSL 沒有 PyQt6
    _HAS_QT = False

#: ⚠️ 用 per-test skipif,**不要**用 module 層的 importorskip ——
#:    後者會把上面那 100 多條純邏輯測試一起跳掉,WSL 端就只剩「1 skipped」,
#:    看起來全綠但其實什麼都沒驗到。
qt_only = pytest.mark.skipif(not _HAS_QT, reason="WSL 沒有 PyQt6 —— Qt 層在 Windows 上測")


@pytest.fixture(scope="module")
def qt_app():
    from PyQt6.QtWidgets import QApplication
    yield QApplication.instance() or QApplication([])


@pytest.fixture(scope="module")
def ctrl_mod():
    """直接從檔案載入,繞過 ui/__init__.py(它會拉進整個 MainWindow)。"""
    import importlib.util
    path = REPO / "ui" / "controllers" / "stot_ros2_controller.py"
    spec = importlib.util.spec_from_file_location("_stot_ctrl_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class _Widget:
    def __init__(self):
        self.text = ""
        self.enabled = True

    def setText(self, t):
        self.text = t

    def setEnabled(self, v):
        self.enabled = bool(v)

    def isChecked(self):
        return True


class _Panel:
    def __init__(self):
        self._stot_status_label = _Widget()
        self._stot_run_btn = _Widget()
        self._stot_abort_btn = _Widget()
        self._stot_sea_target_chk = _Widget()


class _BoxBase:
    """QMessageBox 的替身 —— 記錄每一次彈窗,question 一律回 Yes。

    真的 QMessageBox 是 modal 的:不打樁的話測試會停在對話框上等人按。
    ``StandardButton`` 等列舉在 _make_box() 裡才綁真值(WSL 沒有 PyQt6)。
    """
    calls = []

    @classmethod
    def critical(cls, parent, title, body, *a, **kw):
        cls.calls.append(("critical", title, body))

    @classmethod
    def warning(cls, parent, title, body, *a, **kw):
        cls.calls.append(("warning", title, body))

    @classmethod
    def information(cls, parent, title, body, *a, **kw):
        cls.calls.append(("information", title, body))

    @classmethod
    def question(cls, parent, title, body, *a, **kw):
        cls.calls.append(("question", title, body))
        return cls.StandardButton.Yes


def _make_box():
    from PyQt6.QtWidgets import QMessageBox as Q
    _BoxBase.StandardButton = Q.StandardButton
    _BoxBase.Icon = Q.Icon
    _BoxBase.ButtonRole = Q.ButtonRole
    return _BoxBase


_Box = _BoxBase


class _Proc:
    def __init__(self, argv, **kw):
        self.argv = argv
        self.kw = kw
        self.terminated = False
        self._rc = None

    def poll(self):
        return self._rc

    def terminate(self):
        self.terminated = True
        self._rc = -15

    def wait(self, timeout=None):
        return self._rc

    def kill(self):
        self._rc = -9


@pytest.fixture
def host(qt_app, ctrl_mod, monkeypatch):
    from PyQt6.QtWidgets import QWidget

    class _Host(QWidget, ctrl_mod.StotRos2ControllerMixin):
        pass

    h = _Host()
    h.parameter_panel = _Panel()
    h._strike_targets = [FAR_TARGET]
    h._sitl_links = []
    box = _make_box()
    box.calls = []
    monkeypatch.setattr(ctrl_mod, "QMessageBox", box)
    h._ctrl = ctrl_mod
    return h


def _stub_wsl(ctrl_mod, monkeypatch, preflight_stdout, launched):
    def fake_run(argv, **kw):
        class R:
            stdout = preflight_stdout
            stderr = ""
            returncode = 0
        launched.setdefault("run", []).append(argv)
        return R()

    def fake_popen(argv, **kw):
        p = _Proc(argv, **kw)
        launched.setdefault("popen", []).append(p)
        return p

    monkeypatch.setattr(ctrl_mod.subprocess, "run", fake_run)
    monkeypatch.setattr(ctrl_mod.subprocess, "Popen", fake_popen)
    return launched


@qt_only
def test_ctrl_blocks_when_no_target(host, ctrl_mod, monkeypatch):
    host._strike_targets = []
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN, {})
    host._on_stot_ros2_run()
    assert "popen" not in got, "沒有紅心卻啟動了 —— 絕對不行"
    assert any(c[0] == "critical" and "前置檢查未通過" in c[1] for c in _Box.calls)
    assert any("NO_TARGET" in c[2] or "紅心" in c[2] for c in _Box.calls)


@qt_only
def test_ctrl_blocks_when_ros2_already_up(host, ctrl_mod, monkeypatch):
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN.replace("MAVROS=0", "MAVROS=6"), {})
    host._on_stot_ros2_run()
    assert "popen" not in got
    assert any("swarm_ros2_down.sh" in c[2] for c in _Box.calls if c[0] == "critical")


@qt_only
def test_ctrl_blocks_when_previous_run_alive(host, ctrl_mod, monkeypatch):
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN.replace("TOTNODE=0", "TOTNODE=1"), {})
    host._on_stot_ros2_run()
    assert "popen" not in got


@qt_only
def test_ctrl_blocks_when_wsl_is_dead(host, ctrl_mod, monkeypatch):
    got = _stub_wsl(ctrl_mod, monkeypatch, "", {})
    host._on_stot_ros2_run()
    assert "popen" not in got
    assert any(c[0] == "critical" for c in _Box.calls)


@qt_only
def test_ctrl_reports_missing_wsl_exe(host, ctrl_mod, monkeypatch):
    def boom(*a, **kw):
        raise FileNotFoundError("wsl.exe")
    monkeypatch.setattr(ctrl_mod.subprocess, "run", boom)
    host._on_stot_ros2_run()
    assert any("wsl.exe" in c[1] for c in _Box.calls if c[0] == "critical")


@qt_only
def test_ctrl_happy_path_launches_with_the_clicked_point(host, ctrl_mod, monkeypatch):
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN, {})
    host._on_stot_ros2_run()
    assert "popen" in got, "前置全過卻沒啟動"
    argv = got["popen"][0].argv
    assert argv[0] == "wsl.exe"
    script = argv[-1]
    assert "TGT_LAT=%s" % B.fmt_coord(FAR_TARGET[0]) in script
    assert "TGT_LON=%s" % B.fmt_coord(FAR_TARGET[1]) in script
    assert "bash run_west.sh" in script
    # 使用者要能一眼確認送出的值 —— 確認框與狀態列都必須帶著座標
    assert any(B.fmt_coord(FAR_TARGET[0]) in c[2] for c in _Box.calls if c[0] == "question")
    assert B.fmt_coord(FAR_TARGET[0]) in host.parameter_panel._stot_status_label.text
    # 互斥:執行鈕鎖住、ABORT 開啟
    assert host.parameter_panel._stot_run_btn.enabled is False
    assert host.parameter_panel._stot_abort_btn.enabled is True
    host._stot_stop_timer()


@qt_only
def test_ctrl_refuses_to_start_twice(host, ctrl_mod, monkeypatch):
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN, {})
    host._on_stot_ros2_run()
    n = len(got["popen"])
    _Box.calls = []
    host._on_stot_ros2_run()
    assert len(got["popen"]) == n, "平行跑兩趟 —— 第二趟會靜默殺掉第一趟"
    assert any("已有航次在跑" in c[1] for c in _Box.calls)
    host._stot_stop_timer()


@qt_only
def test_ctrl_abort_uses_pgid_from_log_and_verifies_residuals(host, ctrl_mod, monkeypatch):
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN, {})
    host._on_stot_ros2_run()
    host._stot()["tracker"].feed_line(
        "[trial] ros2 run wrapper PID=999 / 行程群組 PGID=777")

    def fake_run(argv, **kw):
        class R:
            stdout = ("RESID tot=0 mavros=0 show=0 trial=0 west=0 arduplane=6\n")
            stderr = ""
            returncode = 0
        got.setdefault("abort", []).append(argv)
        return R()
    monkeypatch.setattr(ctrl_mod.subprocess, "run", fake_run)

    _Box.calls = []
    host._on_stot_ros2_abort()
    assert got["popen"][0].terminated, "沒有先收掉 Windows 端的驅動行程"
    script = got["abort"][0][-1]
    assert "kill -TERM -777" in script, "沒有用 log 裡回讀到的 PGID"
    assert "swarm_ros2_down.sh" in script
    assert not [c for c in _Box.calls if c[0] == "critical"]
    assert host.parameter_panel._stot_abort_btn.enabled is False


@qt_only
def test_ctrl_abort_shouts_when_residuals_remain(host, ctrl_mod, monkeypatch):
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN, {})
    host._on_stot_ros2_run()

    def fake_run(argv, **kw):
        class R:
            stdout = "RESID tot=1 mavros=0 show=0 trial=0 west=0 arduplane=6\n"
            stderr = ""
            returncode = 0
        return R()
    monkeypatch.setattr(ctrl_mod.subprocess, "run", fake_run)

    _Box.calls = []
    host._on_stot_ros2_abort()
    crit = [c for c in _Box.calls if c[0] == "critical"]
    assert crit and "殘留" in crit[0][1]
    assert "tot=1" in crit[0][2]


@qt_only
def test_ctrl_abort_shouts_when_resid_line_is_absent(host, ctrl_mod, monkeypatch):
    """沒有 RESID = 無法確認收乾淨,絕不可以當成成功。"""
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN, {})
    host._on_stot_ros2_run()
    monkeypatch.setattr(ctrl_mod.subprocess, "run",
                        lambda argv, **kw: type("R", (), {"stdout": "", "stderr": "",
                                                          "returncode": 0})())
    _Box.calls = []
    host._on_stot_ros2_abort()
    assert [c for c in _Box.calls if c[0] == "critical"]


@qt_only
def test_ctrl_resets_every_per_run_field_on_relaunch(host, ctrl_mod, monkeypatch):
    """第二趟不可以沿用第一趟的殘值 —— 這個專案最貴的錯就是「用上一趟的資料
    判這一趟」(stot_repeat.sh:40-42 的 rm -f 註解記的正是同型事故)。"""
    got = _stub_wsl(ctrl_mod, monkeypatch, CLEAN, {})
    host._on_stot_ros2_run()
    st = host._stot()
    # 假裝第一趟已經結案、已驗過目標、已警告過逾時
    st["closed_at"] = 123.0
    st["geom_checked"] = True
    st["warned_timeout"] = True
    st["offsets"] = {"x": 999}
    first_trial = st["trial"]
    host._stot_stop_timer()
    st["proc"] = None

    host._on_stot_ros2_run()
    st = host._stot()
    assert st["closed_at"] == 0.0
    assert st["geom_checked"] is False
    assert st["warned_timeout"] is False
    assert st["offsets"] == {}
    assert st["trial"] != first_trial or len(got["popen"]) == 2
    host._stot_stop_timer()


@qt_only
def test_ctrl_incremental_read_holds_back_partial_lines(host, tmp_path):
    """log 是邊寫邊讀的 —— 半行餵進 parser 會漏掉相位標記。"""
    p = tmp_path / "driver.log"
    p.write_bytes("[1/3] 彈射起飛…\n[2/3] ROS".encode("utf-8"))
    first = host._stot_read_new(str(p))
    assert first == "[1/3] 彈射起飛…\n"
    with open(p, "ab") as fh:
        fh.write("2 編隊層長程巡航(隊形 V)…\n".encode("utf-8"))
    second = host._stot_read_new(str(p))
    assert second.startswith("[2/3] ROS2 編隊層長程巡航")
    assert host._stot_read_new(str(p)) == ""


@qt_only
def test_ctrl_runs_unc_path():
    from PyQt6.QtWidgets import QWidget  # noqa: F401
    import importlib.util
    path = REPO / "ui" / "controllers" / "stot_ros2_controller.py"
    spec = importlib.util.spec_from_file_location("_stot_ctrl_unc", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    unc = mod.StotRos2ControllerMixin._stot_runs_unc("/home/windows-wsl")
    assert unc == r"\\wsl.localhost\Ubuntu-22.04\home\windows-wsl\fixwing_ws\runs"


@qt_only
def test_ctrl_soft_timeout_never_auto_kills(ctrl_mod):
    """UI 端的硬上限只警示 —— 自動殺會製造不可歸因的失敗
    (stot_trial.sh:435-447 對 CA1/D451 的檢討)。"""
    src = (REPO / "ui" / "controllers" / "stot_ros2_controller.py").read_text(encoding="utf-8")
    tick = src.split("def _tick_stot_ros2")[1].split("def ")[0]
    assert "_SOFT_TIMEOUT_S" in tick
    for banned in ("terminate()", "kill()", "_on_stot_ros2_abort()"):
        assert banned not in tick, "輪詢器裡出現自動中止:%r" % banned
    assert ctrl_mod._SOFT_TIMEOUT_S >= 65 * 60   # 單趟最壞約 65 分鐘


if __name__ == "__main__":            # pragma: no cover
    raise SystemExit(pytest.main([__file__, "-q"]))
