"""AeroPlan 地圖點擊 → WSL ROS2 STOT 全流程 的**純邏輯層**(零 GUI、零 WSL 相依)。

這一層只做四件事,全部是可測的純函式/純資料:

  1. 組出要送給 ``wsl.exe`` 的指令(前置檢查 / 啟動 / 中止)
  2. 解析前置檢查的輸出,判斷哪些閘門過不了
  3. 產生不撞名的航次代號 PREFIX
  4. 把驅動 log 與 tot_node log 解析成「目前相位」

真正的 subprocess / QTimer / QMessageBox 都在 ``ui/controllers/stot_ros2_controller.py``。
分開的目的是讓 ``tests/test_click_to_strike.py`` 能在**沒有 GUI、沒有 WSL** 的
環境下(例如 WSL 裡的 python3)整份跑完。

────────────────────────────────────────────────────────────────────────
執行的到底是什麼
────────────────────────────────────────────────────────────────────────
    TGT_LAT=… TGT_LON=… APPROACH_FROM=… TARGET_AMSL=… PREFIX=… TRIALS=1 \\
        bash ~/fixwing_ws/sitl_run/run_west.sh > ~/fixwing_ws/runs/driver_<PREFIX>.log 2>&1

``run_west.sh`` 是整條管線的**唯一參數真相源**(40 多條 ⚠️ 註解是實測血淚反推
出來的)。這裡刻意只覆寫四個變數,其餘一律沿用它的 ``${VAR:-預設}``。
把幾十個 export 抄一份到 Python 端是這個專案明文禁止的模式(參數會兩邊漂移)。

⚠️⚠️ run_west.sh 的最後一行是 ``exec bash "$SR/stot_repeat.sh"``,而
``stot_repeat.sh:51/66/67`` 每一趟都是 swarm_sitl_down → up → sleep 150。
所以 **按下執行鈕之後幾秒內,WSL 上的 6 架 SITL 會被拆掉重建**,AeroPlan 在
6760+10i 的連線會全部斷開,約 155 s 後才會有新的 tcpin 可連。UI 必須事先講明
並主動斷線,否則會留下一堆對死埠重試的 SITLLink QThread。

────────────────────────────────────────────────────────────────────────
外部字串來源(將來上游改了,這裡要一起改)
────────────────────────────────────────────────────────────────────────
    CLOSEOUT_SENTINEL / LEGACY_ALL_HIT
        來源 scripts/wsl/sar_swarm_patches/tot_node.py:45 / :48
        ⚠️ 本檔只**讀**這兩個字串,不會在 .sh 裡新增第二條偵測式 ——
           tests/test_summary_sentinel.py 規定 stot_trial.sh 裡的結案偵測式
           只准存在一條,多一條就當萃取失敗。
    HOME_BY_LOC
        來源 scripts/wsl/swarm_sitl_up.sh:37(LOC=TAOYUAN → 25.0777,121.2325)
    R_FAN_DEFAULT
        來源 scripts/wsl/run_west.sh:112
    APPROACH_FROM 預設 180.0
        來源 scripts/wsl/stot_trial.sh:377 —— run_west.sh **從不設定它**,
        所以扇形永遠擺在目標正南 35 km。對任意點擊的紅心這是錯的,必須由
        AeroPlan 自己算 bearing(target → 起飛點) 傳進去。
"""

from __future__ import annotations

import math
import re
from collections import OrderedDict
from dataclasses import dataclass, field
from datetime import datetime
from typing import Iterable, List, Optional, Sequence, Tuple

# ═════════════════════════════════════════════════════════════════════
#  常數 — 全部標註來源檔案與行號
# ═════════════════════════════════════════════════════════════════════

WSL_DISTRO_DEFAULT = "Ubuntu-22.04"
WS_SUBPATH = "fixwing_ws"
SITL_RUN_SUBPATH = WS_SUBPATH + "/sitl_run"
RUNS_SUBPATH = WS_SUBPATH + "/runs"

#: 結案哨兵 — 來源 sar_swarm_patches/tot_node.py:45 / :48。**兩種都要認**:
#: 新形狀(哨兵與達標描述分離)與舊形狀(全員命中)在歷史 log 裡都存在。
CLOSEOUT_SENTINEL = "★★ 俯衝結案:"
LEGACY_ALL_HIT = "俯衝命中紅心:全員命中"

#: 起飛點對照表 — 來源 swarm_sitl_up.sh:37。run_west.sh:29 預設 LOC=TAOYUAN。
HOME_BY_LOC = {
    "TAOYUAN": (25.0777, 121.2325),
    "RCTP": (25.0777, 121.2325),
    "TPE": (25.0777, 121.2325),
}
DEFAULT_LOC = "TAOYUAN"

#: 扇形半徑 — 來源 run_west.sh:112。
R_FAN_DEFAULT = 35000.0

#: 幾何裕度。stot_repeat.sh:58-61 明載必要條件 D_home > r_fan > pushover_r,
#: 但**沒有任何腳本會檢查它** —— 紅心離起飛點太近時整條航路是反的,而且靜默。
#: 這一關只能在 AeroPlan 端擋。
GEOMETRY_MARGIN = 1.15

#: 每一趟 run_west.sh 只跑一趟(TRIALS=1);stot_repeat.sh:39 會把 trial 序號
#: 接在 PREFIX 後面 ⇒ 實際航次代號 = PREFIX + "1"。
TRIAL_SUFFIX = "1"

#: 走 stot_repeat.sh 的整條鏈需要的腳本。缺任何一支都會死在該行(靜默或 rc≠0)。
REQUIRED_SCRIPTS = (
    "run_west.sh",
    "stot_repeat.sh",
    "stot_trial.sh",
    "swarm_sitl_up.sh",
    "swarm_sitl_down.sh",
    "swarm_ros2_up.sh",
    "swarm_ros2_down.sh",
    "preflight_check.py",
    "make_artifacts.sh",
    "stot_score.py",
)

#: 目標高度基準。run_west.sh:185 為海面目標把 TARGET_AMSL 釘死在 0.0;
#: 註解明講「陸地目標請改回 nan(=查地形資料庫)」。
#: ⚠️ 必須送**字面小數點**的 ".nan" —— 寫 "nan" 會被 YAML 當字串,
#:    節點的型別檢查當場掛掉(stot_trial.sh:394)。
TARGET_AMSL_SEA = "0.0"
TARGET_AMSL_TERRAIN = ".nan"

WGS84_R = 6371008.8  # m,平均地球半徑


# ═════════════════════════════════════════════════════════════════════
#  例外
# ═════════════════════════════════════════════════════════════════════

class StotBridgeError(RuntimeError):
    """組指令/解析階段的錯誤。訊息一律直接給使用者看,不要吞掉。"""


# ═════════════════════════════════════════════════════════════════════
#  幾何 — 只用 stdlib(不能 import utils.math_utils,那會拉進整個重量級套件)
# ═════════════════════════════════════════════════════════════════════

def haversine_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """兩個 (lat, lon) 之間的大圓距離,公尺。"""
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * WGS84_R * math.asin(min(1.0, math.sqrt(h)))


def bearing_deg(frm: Tuple[float, float], to: Tuple[float, float]) -> float:
    """從 frm 看向 to 的初始方位角,0..360(正北 0、正東 90)。"""
    lat1, lat2 = math.radians(frm[0]), math.radians(to[0])
    dlon = math.radians(to[1] - frm[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def approach_from_for(target: Tuple[float, float],
                      home: Tuple[float, float]) -> float:
    """扇形要擺在目標的哪個方位 —— 就是「起飛點的方向」。

    tot_node.py:847/856 的 slot 是 ``dest(T, approach_from ± off, r_fan)``,
    也就是以**目標**為圓心、往 approach_from 方位量 r_fan。蜂群是從起飛點過來的,
    所以扇形要擺在 bearing(target → home) 上,蜂群才不必先繞過目標再折返。

    stot_trial.sh:377 的預設 180.0(正南)只對「目標在桃園正西」那組寫死的
    基準值成立,對任意點擊的紅心是錯的。
    """
    return bearing_deg(target, home)


# ═════════════════════════════════════════════════════════════════════
#  座標與代號的格式化 / 驗證
#  —— 唯一會被拼進 `bash -lc` 的東西,所以驗證放在這裡,不放呼叫端
# ═════════════════════════════════════════════════════════════════════

_PREFIX_RE = re.compile(r"^[A-Z][A-Z0-9]{0,15}$")
_DISTRO_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")


def fmt_coord(v: float) -> str:
    """座標一律 ``%.7f``(約 1 cm 解析度)。

    兩個理由,都不是美觀問題:
      * 使用者必須能一眼確認送出的就是他點的那一點(這個專案吃過「參數靜默
        沒生效」的虧)——輸出精度不足就沒得比對。
      * stot_trial.sh:92 的 ``_f()`` 小數點修補套用在十幾個 float 參數上,
        **唯獨 :376 的 tlat/tlon 沒套**。字串若長得像整數(例如 "121"),
        ROS 2 會判成 INTEGER 而參數宣告是 DOUBLE ⇒ 節點一起來就
        InvalidParameterTypeException,對外表現成 rc=3「節點提前死亡」。
        ``%.7f`` 保證永遠帶小數點。
    """
    f = float(v)
    if not math.isfinite(f):
        raise StotBridgeError(f"座標不是有限數:{v!r}")
    return "%.7f" % f


def validate_target(lat: float, lon: float) -> Tuple[float, float]:
    la, lo = float(lat), float(lon)
    if not (math.isfinite(la) and math.isfinite(lo)):
        raise StotBridgeError(f"目標座標不是有限數:({lat!r}, {lon!r})")
    if not (-90.0 <= la <= 90.0):
        raise StotBridgeError(f"緯度超出範圍:{la}")
    if not (-180.0 <= lo <= 180.0):
        raise StotBridgeError(f"經度超出範圍:{lo}")
    return la, lo


def validate_prefix(prefix: str) -> str:
    if not isinstance(prefix, str) or not _PREFIX_RE.match(prefix):
        raise StotBridgeError(
            f"PREFIX 不合法:{prefix!r}(必須是大寫字母開頭的英數字,長度 ≤16)")
    return prefix


def trial_name(prefix: str) -> str:
    """PREFIX → 實際航次代號。stot_repeat.sh:39 ``T="${PREFIX}${n}"``,TRIALS=1 ⇒ n=1。"""
    return validate_prefix(prefix) + TRIAL_SUFFIX


# ═════════════════════════════════════════════════════════════════════
#  PREFIX 產生 —— 撞名 = 靜默刪除舊航次的原始資料
# ═════════════════════════════════════════════════════════════════════

_PREFIX_LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


def make_prefix(taken: Iterable[str], now: Optional[datetime] = None) -> str:
    """產生 ``AP<MMDD><letter>``,保證 trial 代號不撞既有航次。

    stot_repeat.sh:44-46 在開跑前會 ``rm -f`` 掉同名的 tot_/trial_/df_/show_
    檔案,**不報錯**。更糟的是 ``runs/artifacts/<T>/`` 不在 rm 清單裡 ⇒ 撞名之後
    會變成「新 log + 舊素材」的混合體,比整批刪掉更難察覺。

    taken: 已被占用的**航次代號**集合(不是 PREFIX)。呼叫端要把
           runs/trial_*.log、runs/tot_*.csv、runs/artifacts/*/ 三處都掃進來。
    """
    stamp = (now or datetime.now()).strftime("%m%d")
    used = {str(t).strip().upper() for t in taken if str(t).strip()}
    for letter in _PREFIX_LETTERS:
        prefix = "AP%s%s" % (stamp, letter)
        if trial_name(prefix).upper() not in used:
            return prefix
    raise StotBridgeError(
        "今天的航次代號 AP%sA … AP%sZ 全被占用了 —— 26 趟已經是異常,"
        "請先清理 ~/%s/ 再重試。" % (stamp, stamp, RUNS_SUBPATH))


# ═════════════════════════════════════════════════════════════════════
#  wsl.exe 指令組裝
# ═════════════════════════════════════════════════════════════════════

def _wsl(distro: str, script: str) -> List[str]:
    if not _DISTRO_RE.match(distro or ""):
        raise StotBridgeError(f"WSL distro 名稱不合法:{distro!r}")
    return ["wsl.exe", "-d", distro, "-e", "bash", "-lc", script]


def build_preflight_command(distro: str = WSL_DISTRO_DEFAULT) -> List[str]:
    """一次 wsl.exe 呼叫查完所有前置狀態,輸出 ``KEY=VALUE`` 行。

    ⚠️ 三個刻意的限制:
      * **絕不建立任何 MAVLink 連線**。MAVProxy 的 ``--out tcpin`` 只 accept
        一次(pymavlink mavutil.mavtcpin),「順手測一下埠通不通」會把 AeroPlan
        自己餓死 —— 這個坑本專案已經踩過。這裡只用 pgrep / test -f / glob。
      * **一次呼叫查完**。每次 spawn wsl.exe 要 100–300 ms,分成六次就是半秒起跳
        的 GUI 凍結。
      * ``pgrep -c`` 在沒有命中時會印 0 **並且** exit 1,套 ``|| echo 0`` 會變成
        「0 0」。所以一律用 ``pgrep … | wc -l``,永遠只印一個數字且不會失敗。

    ⚠️⚠️ **這段指令自己的 cmdline 就是 pgrep -f 的掃描對象。**
       第一版把 REQUIRED_SCRIPTS 直接展開成 ``for f in run_west.sh stot_trial.sh …``,
       於是 ``pgrep -f '[s]tot_trial.sh'`` 一路匹配到 bash 自己與它 fork 出來的
       命令替換子殼 —— 實測回報 TRIALSH=2 / WESTSH=2,「上一趟還在跑」會在
       **每一次**啟動時誤報,功能等於永遠開不起來。
       ⇒ 這裡**不再出現任何未加中括號的行程名**:腳本只列出目錄內容,
         「缺哪幾支」交給 Python 端比對(見 parse_preflight)。
         中括號寫法(``[s]tot_trial.sh``)本身不會匹配到自己,那是刻意的。
       tests/test_click_to_strike.py 有一條迴歸鎖在守這件事。
    """
    body = r"""
set +e
shopt -s nullglob
SR="$HOME/{sitl_run}"
RUNS="$HOME/{runs}"
echo "HOME=$HOME"
echo "ARDUPLANE=$(pgrep -x arduplane 2>/dev/null | wc -l)"
echo "MAVROS=$(pgrep -x mavros_node 2>/dev/null | wc -l)"
echo "SWARMNODE=$(pgrep -f '[m]aster_node|[w]ingman_node|[b]ackup_node' 2>/dev/null | wc -l)"
echo "TOTNODE=$(pgrep -f '[t]ot_node' 2>/dev/null | wc -l)"
echo "TRIALSH=$(pgrep -f '[s]tot_trial.sh' 2>/dev/null | wc -l)"
echo "REPEATSH=$(pgrep -f '[s]tot_repeat.sh' 2>/dev/null | wc -l)"
echo "WESTSH=$(pgrep -f '[r]un_west.sh' 2>/dev/null | wc -l)"
echo "SHOWPY=$(pgrep -f '[j]et_show_ros2' 2>/dev/null | wc -l)"
for f in "$SR"/*; do [ -f "$f" ] && echo "SRFILE=${{f##*/}}"; done
for f in "$RUNS"/trial_*.log; do b="${{f##*/}}"; b="${{b#trial_}}"; echo "TAKEN=${{b%.log}}"; done
for f in "$RUNS"/tot_*.csv;   do b="${{f##*/}}"; b="${{b#tot_}}";   echo "TAKEN=${{b%.csv}}"; done
for d in "$RUNS"/artifacts/*/; do b="${{d%/}}"; echo "TAKEN=${{b##*/}}"; done
echo "PREFLIGHT_OK=1"
""".format(sitl_run=SITL_RUN_SUBPATH, runs=RUNS_SUBPATH)
    return _wsl(distro, body)


def build_launch_env(lat: float, lon: float, prefix: str, *,
                     home: Tuple[float, float] = HOME_BY_LOC[DEFAULT_LOC],
                     sea_level_target: bool = True) -> "OrderedDict[str, str]":
    """組出要覆寫的環境變數。**只有這四個 + 兩個代號**,其餘沿用 run_west.sh。

    回傳 OrderedDict 是為了讓 UI 與 log 能原樣印出「實際送出的值」——
    這是使用者明確要求的驗收點:必須一眼確認送出的就是他點的那一點。
    """
    la, lo = validate_target(lat, lon)
    prefix = validate_prefix(prefix)
    env: "OrderedDict[str, str]" = OrderedDict()
    env["TGT_LAT"] = fmt_coord(la)
    env["TGT_LON"] = fmt_coord(lo)
    # run_west.sh 從不設 APPROACH_FROM ⇒ 不傳的話 stot_trial.sh:377 會用 180.0
    env["APPROACH_FROM"] = "%.1f" % approach_from_for((la, lo), home)
    env["TARGET_AMSL"] = TARGET_AMSL_SEA if sea_level_target else TARGET_AMSL_TERRAIN
    env["PREFIX"] = prefix
    env["TRIALS"] = "1"
    return env


def driver_log_path(prefix: str) -> str:
    """驅動 stdout 的落點(WSL 端絕對路徑,~ 展開前)。"""
    return "~/%s/driver_%s.log" % (RUNS_SUBPATH, validate_prefix(prefix))


def build_run_command(lat: float, lon: float, prefix: str, *,
                      home: Tuple[float, float] = HOME_BY_LOC[DEFAULT_LOC],
                      sea_level_target: bool = True,
                      distro: str = WSL_DISTRO_DEFAULT) -> List[str]:
    """完整的 ``wsl.exe …`` argv。

    stdout/stderr **在 WSL 端就重導到檔案**,Windows 端不開 PIPE ——
    mission/sitl_launcher.py:658-660 已經記下教訓:PIPE 滿了會 deadlock。
    而且相位有一半在 ``runs/trial_<T>.log`` 裡,PIPE 本來就解決不了那半邊,
    兩邊都用「輪詢檔案」是一套機制而不是兩套。
    """
    env = build_launch_env(lat, lon, prefix, home=home,
                           sea_level_target=sea_level_target)
    assigns = " ".join("%s=%s" % (k, v) for k, v in env.items())
    script = "cd ~/%s && %s bash run_west.sh > %s 2>&1" % (
        SITL_RUN_SUBPATH, assigns, driver_log_path(prefix))
    return _wsl(distro, script)


def build_abort_command(tot_pgid: Optional[int] = None,
                        distro: str = WSL_DISTRO_DEFAULT) -> List[str]:
    """WSL 端的無條件拆除 + **讀回來驗證**。

    ⚠️ 為什麼不能只殺 Windows 端的 wsl.exe:這條鏈上有三個地方刻意脫離父行程 ——
       tot_node 是 ``setsid nohup``(stot_trial.sh:375)、swarm_sitl_up.sh 在背景
       子殼(stot_repeat.sh:66,註解明講要避開 PDEATHSIG)、MAVProxy 也是 setsid
       (swarm_sitl_up.sh:377)。而 stot_trial.sh:195 的 INT/TERM trap 只
       ``_artifacts_once; exit 130``,**跳過** :515-529 的 PGID 收尾。
       ⇒ 光殺上游會留下一個還在指揮六架飛機的 tot_node,而且完全靜默。

    ⚠️ 名稱後備用 ``pkill -f "[t]ot_node"`` 而不是 ``pkill -x tot_node``:
       ``ros2 run`` 是 Popen+communicate(無 exec、無 setproctitle),節點的 comm
       其實是 ``python3``,``-x`` 比對 comm ⇒ 打不中。中括號是為了避免 pkill
       匹配到自己這行指令。

    ⚠️ **不碰 arduplane / mavproxy**。走 run_west.sh 這條路,SITL 是
       stot_repeat.sh 重建的,中止後留著它反而讓使用者可以立刻重連 AeroPlan;
       殺掉只是把「可恢復」變成「還要再等 150 s」。arduplane 的殘量會在
       RESID 行裡回報,由使用者決定要不要自己收。
    """
    steps = []
    if tot_pgid is not None:
        pg = int(tot_pgid)
        if pg <= 1:
            raise StotBridgeError(f"tot_pgid 不合法:{tot_pgid!r}")
        # 先斷掉「正在指揮飛機的那一個」,再收上游
        steps.append('kill -TERM -%d 2>/dev/null' % pg)
        steps.append('for _k in 1 2 3; do kill -0 -%d 2>/dev/null || break; sleep 1; done' % pg)
        steps.append('kill -KILL -%d 2>/dev/null' % pg)
    steps.append('pkill -f "[t]ot_node" 2>/dev/null')
    steps.append('pkill -f "[j]et_show_ros2" 2>/dev/null')
    steps.append('pkill -f "[j]et_catapult_launch" 2>/dev/null')
    steps.append('pkill -f "[s]tot_trial.sh" 2>/dev/null')
    steps.append('pkill -f "[s]tot_repeat.sh" 2>/dev/null')
    steps.append('pkill -f "[r]un_west.sh" 2>/dev/null')
    steps.append('bash "$HOME/%s/swarm_ros2_down.sh" >/dev/null 2>&1' % SITL_RUN_SUBPATH)
    steps.append('sleep 2')
    steps.append(
        'echo "RESID tot=$(pgrep -f \'[t]ot_node\' 2>/dev/null | wc -l)'
        ' mavros=$(pgrep -x mavros_node 2>/dev/null | wc -l)'
        ' show=$(pgrep -f \'[j]et_show_ros2\' 2>/dev/null | wc -l)'
        ' trial=$(pgrep -f \'[s]tot_trial.sh\' 2>/dev/null | wc -l)'
        ' west=$(pgrep -f \'[r]un_west.sh\' 2>/dev/null | wc -l)'
        ' arduplane=$(pgrep -x arduplane 2>/dev/null | wc -l)"')
    return _wsl(distro, "set +e; " + "; ".join(steps))


_RESID_RE = re.compile(r"^RESID\s+(.*)$", re.M)


def parse_resid(stdout: str) -> "OrderedDict[str, int]":
    """把 abort 的 ``RESID …`` 行解析成計數。找不到就丟例外 —— 拆完沒驗證等於沒拆。"""
    m = _RESID_RE.search(stdout or "")
    if not m:
        raise StotBridgeError(
            "中止指令沒有回報 RESID 殘留計數 —— 無法確認 WSL 端是否收乾淨。"
            "請在自己的終端機執行:pgrep -af '[t]ot_node'")
    out: "OrderedDict[str, int]" = OrderedDict()
    for tok in m.group(1).split():
        if "=" in tok:
            k, _, v = tok.partition("=")
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = -1
    return out


#: 中止後仍在跑就代表「還有東西在指揮飛機」的行程。arduplane 不在內(見上)。
RESID_FATAL_KEYS = ("tot", "mavros", "show", "trial", "west")


def resid_leftovers(resid: "OrderedDict[str, int]") -> "OrderedDict[str, int]":
    return OrderedDict((k, v) for k, v in resid.items()
                       if k in RESID_FATAL_KEYS and v != 0)


# ═════════════════════════════════════════════════════════════════════
#  前置檢查結果
# ═════════════════════════════════════════════════════════════════════

@dataclass
class PreflightReport:
    """``build_preflight_command`` 輸出的結構化版本。"""
    wsl_home: str = ""
    arduplane: int = 0
    mavros: int = 0
    swarm_node: int = 0
    tot_node: int = 0
    trial_sh: int = 0
    repeat_sh: int = 0
    west_sh: int = 0
    show_py: int = 0
    sitl_run_files: List[str] = field(default_factory=list)
    missing: List[str] = field(default_factory=list)   # 由 sitl_run_files 反推
    taken: List[str] = field(default_factory=list)
    ok: bool = False           # PREFLIGHT_OK=1 有印出來 ⇒ 腳本整段跑完


@dataclass
class Issue:
    """一條前置檢查結論。blocking=True 就是硬擋。"""
    code: str
    message: str
    blocking: bool = True

    def __str__(self) -> str:          # 方便直接丟進 QMessageBox
        return ("✗ " if self.blocking else "⚠ ") + self.message


_KV_RE = re.compile(r"^([A-Z_]+)=(.*)$")
_FIELD_MAP = {
    "HOME": "wsl_home", "ARDUPLANE": "arduplane", "MAVROS": "mavros",
    "SWARMNODE": "swarm_node", "TOTNODE": "tot_node", "TRIALSH": "trial_sh",
    "REPEATSH": "repeat_sh", "WESTSH": "west_sh", "SHOWPY": "show_py",
}


def parse_preflight(stdout: str) -> PreflightReport:
    rep = PreflightReport()
    for raw in (stdout or "").splitlines():
        m = _KV_RE.match(raw.strip())
        if not m:
            continue
        key, val = m.group(1), m.group(2).strip()
        if key == "SRFILE":
            if val:
                rep.sitl_run_files.append(val)
        elif key == "TAKEN":
            if val:
                rep.taken.append(val)
        elif key == "PREFLIGHT_OK":
            rep.ok = (val == "1")
        elif key in _FIELD_MAP:
            attr = _FIELD_MAP[key]
            if attr == "wsl_home":
                rep.wsl_home = val
            else:
                try:
                    setattr(rep, attr, int(val))
                except ValueError:
                    setattr(rep, attr, -1)
    # 「缺哪幾支」刻意在 Python 端比對 —— 腳本裡不可以出現未加中括號的行程名,
    #  否則 pgrep -f 會匹配到指令自己(見 build_preflight_command 的 ⚠️⚠️)。
    have = set(rep.sitl_run_files)
    rep.missing = [s for s in REQUIRED_SCRIPTS if s not in have]
    return rep


def evaluate_preflight(rep: PreflightReport, *,
                       target: Optional[Tuple[float, float]],
                       home: Tuple[float, float] = HOME_BY_LOC[DEFAULT_LOC],
                       r_fan: float = R_FAN_DEFAULT,
                       local_run_active: bool = False) -> List[Issue]:
    """把 PreflightReport 翻成給使用者看的閘門清單。

    ⚠️ 這個專案最大的敵人是靜默失敗,所以每一條都**明講原因與處置**,
       而且寧可多擋也不要讓一趟 50 分鐘的航次白燒。
    """
    issues: List[Issue] = []

    if not rep.ok:
        issues.append(Issue(
            "WSL_UNREACHABLE",
            "WSL 前置檢查沒有跑完(缺 PREFLIGHT_OK)——"
            "確認 WSL 發行版可用:wsl.exe -d %s -e bash -lc 'echo ok'"
            % WSL_DISTRO_DEFAULT))
        return issues        # 底下的數字全部不可信,不要繼續推論

    if target is None:
        issues.append(Issue(
            "NO_TARGET",
            "還沒有紅心座標 —— 請先在地圖上標記打擊目標,或在 ⓿ 區手動輸入 lat/lon。"))
    else:
        try:
            la, lo = validate_target(*target)
        except StotBridgeError as exc:
            issues.append(Issue("BAD_TARGET", str(exc)))
        else:
            d = haversine_m(home, (la, lo))
            need = r_fan * GEOMETRY_MARGIN
            if d < need:
                issues.append(Issue(
                    "TARGET_TOO_CLOSE",
                    "紅心離起飛點只有 %.1f km,必須 > %.1f km(扇形半徑 %.0f m × %.2f)。\n"
                    "  起飛點會落在扇形/環的內側,蜂群得先飛離目標再折返 —— "
                    "整條航路是反的,而且**沒有任何 WSL 腳本會擋下來**"
                    "(stot_repeat.sh:58-61 只寫在註解裡)。"
                    % (d / 1000.0, need / 1000.0, r_fan, GEOMETRY_MARGIN)))

    if rep.missing:
        issues.append(Issue(
            "MISSING_SCRIPTS",
            "~/%s/ 缺少 %d 支腳本:%s\n"
            "  請在 WSL 自行執行 deploy_build.sh 同步 scripts/wsl/ 之後再試。"
            % (SITL_RUN_SUBPATH, len(rep.missing), " ".join(rep.missing))))

    busy = []
    if local_run_active:
        busy.append("AeroPlan 自己的驅動行程")
    for label, n in (("tot_node", rep.tot_node), ("stot_trial.sh", rep.trial_sh),
                     ("stot_repeat.sh", rep.repeat_sh), ("run_west.sh", rep.west_sh),
                     ("jet_show_ros2.py", rep.show_py)):
        if n > 0:
            busy.append("%s×%d" % (label, n))
    if busy:
        issues.append(Issue(
            "RUN_IN_PROGRESS",
            "上一趟還在跑:%s\n"
            "  ⚠️ 絕對不能平行跑兩趟 —— stot_repeat.sh:50 的 pkill 與 "
            "swarm_sitl_down.sh:4 的 pkill 都是無差別的,第二趟會**靜默殺掉**第一趟。\n"
            "  請先按 ABORT,或在 WSL 端確認殘留已清乾淨。" % "、".join(busy)))

    if rep.mavros > 0 or rep.swarm_node > 0:
        issues.append(Issue(
            "ROS2_ALREADY_UP",
            "ROS2 層已經在跑(mavros_node×%d、編隊節點×%d)——**請先收掉**。\n"
            "  這條流程會由 stot_trial.sh:354 自己把 ROS2 層拉起來、:358 自己收掉;\n"
            "  預先起好的那一層會在 swarm_ros2_up.sh:141 被 pkill 重來(純屬白工),\n"
            "  若當初是用 MANUAL=0 起的,節點還會自己 arm 起飛跟彈射搶控制。\n"
            "  處置:bash ~/%s/swarm_ros2_down.sh"
            % (rep.mavros, rep.swarm_node, SITL_RUN_SUBPATH)))

    # —— 以下是**非阻擋**的告知 ——————————————————————————————
    issues.append(Issue(
        "SITL_WILL_RESTART",
        "本流程會先**拆掉再重建** WSL 的 6 架 SITL(目前 arduplane×%d),約需 155 秒。\n"
        "  run_west.sh 末行 exec 到 stot_repeat.sh,而 :51 swarm_sitl_down.sh 會\n"
        "  pkill mavproxy ⇒ AeroPlan 在 6760+10i 的連線會全部斷開。"
        % rep.arduplane, blocking=False))

    return issues


def blocking_issues(issues: Sequence[Issue]) -> List[Issue]:
    return [i for i in issues if i.blocking]


# ═════════════════════════════════════════════════════════════════════
#  相位解析
# ═════════════════════════════════════════════════════════════════════

PHASE_IDLE = "IDLE"
PHASE_STARTING = "STARTING"
PHASE_SITL_REBUILD = "SITL_REBUILD"
PHASE_CATAPULT = "CATAPULT"
PHASE_CRUISE = "CRUISE"
PHASE_TOT_INIT = "TOT_INIT"
PHASE_ALIGN = "ALIGN"
PHASE_PERCH = "PERCH"
PHASE_DIVE = "DIVE"
PHASE_CLOSED = "CLOSED"
PHASE_FAILED = "FAILED"

#: 相位的推進順序。log 是兩個檔案交錯讀進來的,不保證時間有序 ——
#: 用 rank 擋住倒退,避免 UI 在 DIVE 之後又跳回 CRUISE。
PHASE_RANK = {
    PHASE_IDLE: 0, PHASE_STARTING: 1, PHASE_SITL_REBUILD: 2,
    PHASE_CATAPULT: 3, PHASE_CRUISE: 4, PHASE_TOT_INIT: 5,
    PHASE_ALIGN: 6, PHASE_PERCH: 7, PHASE_DIVE: 8,
}
PHASE_LABEL = {
    PHASE_IDLE: "待命",
    PHASE_STARTING: "驅動已啟動",
    PHASE_SITL_REBUILD: "重建 SITL(約 155 s)",
    PHASE_CATAPULT: "[1/3] 彈射起飛 / 爬升",
    PHASE_CRUISE: "[2/3] ROS2 編隊層長程巡航",
    PHASE_TOT_INIT: "[3/3] tot_node 起動",
    PHASE_ALIGN: "ALIGN 扇形集結",
    PHASE_PERCH: "PERCH 對準內切環",
    PHASE_DIVE: "DIVE 同步俯衝",
    PHASE_CLOSED: "結案",
    PHASE_FAILED: "失敗",
}

# ── 需要抓值的行 ──────────────────────────────────────────────────
#: run_west.sh:223 ``echo "[west] 目標 (${TGT_LAT}, ${TGT_LON}) ← 起飛 ${LOC}"``
_RE_WEST_TARGET = re.compile(r"\[west\]\s*目標\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)")
#: tot_node.py:563-565 ``tot_node up：… → 俯衝 → 目標 (lat, lon)``
_RE_NODE_TARGET = re.compile(r"tot_node up.*目標\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)")
#: stot_trial.sh:434 ``[trial] ros2 run wrapper PID=… / 行程群組 PGID=…``
_RE_PGID = re.compile(r"行程群組\s*PGID\s*=\s*(\d+)")
_RE_HIT = re.compile(r"UAV(\d+)\s*命中紅心\s*@")
_RE_MISS = re.compile(r"✗\s*UAV(\d+)\s*判定\s*MISS")

# ── 失敗標記(第一個命中就結案)────────────────────────────────────
_FAIL_RULES: Tuple[Tuple[str, str], ...] = (
    ("✗ SITL 起不來", "SITL 重建後不足 6 架 —— stot_repeat.sh:68 硬閘門"),
    ("✗ SITL 不是 6 架", "SITL 不是 6 架 —— stot_trial.sh:156"),
    ("✗ 開跑前驗證未通過", "preflight_check.py 未通過 —— 參數之間互相矛盾"),
    ("✗ CRUISE_MODE=ros2 但 CLIMB_TO_ALT=", "CLIMB_TO_ALT < 1000 m —— stot_trial.sh:210"),
    ("✗ 彈射失敗", "彈射起飛失敗 —— 見 /tmp/trial_launch.log"),
    ("在結案前就不在了", "tot_node 在結案前就死了(**不是逾時**)—— stot_trial.sh:551"),
    ("但摘要行還沒印", "已命中但摘要沒印完(impact_settle 窄縫)—— stot_trial.sh:557"),
    ("✗ 逾時(只命中", "逾時 —— 預算用完仍未全員命中"),
)

# ── 相位標記(由後往前比,最深的相位勝)────────────────────────────
_PHASE_RULES: Tuple[Tuple[str, str], ...] = (
    ("[west] 目標", PHASE_STARTING),
    ("REPEAT ", PHASE_SITL_REBUILD),
    ("[1/3] 彈射起飛", PHASE_CATAPULT),
    ("[2/3]", PHASE_CRUISE),
    ("[3/3] STOT", PHASE_TOT_INIT),
    ("tot_node up", PHASE_TOT_INIT),
    ("ALIGN：", PHASE_ALIGN),
    ("★ 全員抵扇形", PHASE_PERCH),
    ("★ 全員就緒", PHASE_PERCH),
    ("★ 全員對準內切", PHASE_DIVE),
    ("交接終端 PN", PHASE_DIVE),
)

#: 額外的里程碑 —— 不換相位,但值得顯示給使用者看
_DETAIL_RULES: Tuple[str, ...] = (
    "彈射間隔", "全員 GUIDED", "全員抵巡航高度", "抵達交棒距離",
    "★ 依實測地形重算幾何", "★ 首架抵環", "★★ 真值基準",
)


@dataclass
class PhaseState:
    phase: str = PHASE_IDLE
    detail: str = ""
    failed_reason: str = ""
    closed_out: bool = False
    hits: set = field(default_factory=set)
    misses: set = field(default_factory=set)
    tot_pgid: Optional[int] = None
    echoed_target_west: Optional[Tuple[float, float]] = None
    echoed_target_node: Optional[Tuple[float, float]] = None


class PhaseTracker:
    """把驅動 log 與 tot_node log 的每一行餵進來,推出目前相位。

    刻意做成「餵一行、回一個 bool(有沒有變化)」而不是「掃整份檔案」:
    輪詢器每秒只讀增量,掃整份在 1400 s 的航次尾端會愈來愈慢。
    """

    def __init__(self) -> None:
        self.state = PhaseState()

    # —— 查詢 ————————————————————————————————————————————
    @property
    def phase(self) -> str:
        return self.state.phase

    @property
    def finished(self) -> bool:
        return self.state.phase in (PHASE_CLOSED, PHASE_FAILED)

    def status_text(self, trial: str = "") -> str:
        s = self.state
        parts = [p for p in (trial, PHASE_LABEL.get(s.phase, s.phase)) if p]
        if s.hits or s.misses:
            parts.append("命中 %d / MISS %d" % (len(s.hits), len(s.misses)))
        if s.phase == PHASE_FAILED and s.failed_reason:
            parts.append(s.failed_reason)
        elif s.detail:
            parts.append(s.detail)
        return " · ".join(parts)

    # —— 餵資料 ——————————————————————————————————————————
    def feed_line(self, line: str) -> bool:
        """回傳 True 表示狀態有變(呼叫端據此決定要不要更新 UI)。"""
        if not line:
            return False
        line = line.strip()
        if not line:
            return False
        s = self.state
        before = (s.phase, s.detail, s.failed_reason, len(s.hits), len(s.misses),
                  s.tot_pgid, s.closed_out)

        m = _RE_WEST_TARGET.search(line)
        if m:
            s.echoed_target_west = (float(m.group(1)), float(m.group(2)))
        m = _RE_NODE_TARGET.search(line)
        if m:
            s.echoed_target_node = (float(m.group(1)), float(m.group(2)))
        m = _RE_PGID.search(line)
        if m:
            s.tot_pgid = int(m.group(1))

        m = _RE_HIT.search(line)
        if m:
            s.hits.add(int(m.group(1)))
        m = _RE_MISS.search(line)
        if m:
            s.misses.add(int(m.group(1)))

        # 結案哨兵 —— 兩種形狀都要認(來源 tot_node.py:45 / :48)
        if CLOSEOUT_SENTINEL in line or LEGACY_ALL_HIT in line:
            s.closed_out = True
            s.phase = PHASE_CLOSED
            s.detail = line[:200]
            return True

        for token, reason in _FAIL_RULES:
            if token in line:
                s.phase = PHASE_FAILED
                s.failed_reason = reason
                s.detail = line[:200]
                return True

        for token, phase in _PHASE_RULES:
            if token in line:
                if PHASE_RANK.get(phase, -1) > PHASE_RANK.get(s.phase, -1):
                    s.phase = phase
                    s.detail = ""
                break

        for token in _DETAIL_RULES:
            if token in line:
                s.detail = line[:120]
                break

        return (s.phase, s.detail, s.failed_reason, len(s.hits), len(s.misses),
                s.tot_pgid, s.closed_out) != before

    def feed(self, text: str) -> bool:
        changed = False
        for raw in (text or "").splitlines():
            if self.feed_line(raw):
                changed = True
        return changed


# ═════════════════════════════════════════════════════════════════════
#  產物解析 —— 「讀回來驗證」用
# ═════════════════════════════════════════════════════════════════════

def parse_geom_target(csv_text: str) -> Optional[Tuple[float, float]]:
    """從 ``runs/tot_<T>_geom.csv`` 取出第一列 ``target,<lat>,<lon>``。

    tot_node.py:548-555 保證第一列是 target。這是最終的回讀驗證點:
    節點**自己算完之後**寫下的目標,和 AeroPlan 送出去的值比對才算數。
    """
    for raw in (csv_text or "").splitlines():
        parts = [p.strip() for p in raw.split(",")]
        if len(parts) >= 3 and parts[0] == "target":
            try:
                return float(parts[1]), float(parts[2])
            except ValueError:
                return None
    return None


def target_matches(sent: Tuple[float, float],
                   echoed: Optional[Tuple[float, float]],
                   tol: float = 1e-6) -> bool:
    """送出值 vs 回讀值。tol 1e-6 度 ≈ 0.11 m,而我們送的是 %.7f ⇒ 只容忍捨入。"""
    if echoed is None:
        return False
    return abs(sent[0] - echoed[0]) <= tol and abs(sent[1] - echoed[1]) <= tol


def parse_summary_row(csv_text: str, trial: str) -> Optional[dict]:
    """從 ``runs/stot_repeat_v2.csv`` 取出某趟的那一列。

    表頭(stot_repeat.sh:36):
        trial,n_touch,gs_touch_mean,gs_touch_min,gs_touch_max,
        max_impact_miss_m,max_abs_alt_m,touch_spread_s,pass
    """
    lines = [l for l in (csv_text or "").splitlines() if l.strip()]
    if not lines:
        return None
    header = [h.strip() for h in lines[0].split(",")]
    if not header or header[0] != "trial":
        return None
    for raw in reversed(lines[1:]):
        cells = [c.strip() for c in raw.split(",")]
        if cells and cells[0] == trial:
            return dict(zip(header, cells))
    return None


#: stot_trial.sh 的退出碼對照(stot_repeat.sh:79-80 也引用同一份)。
#: ⚠️ 走 run_west.sh → stot_repeat.sh 這條路時,**驅動行程的 rc 幾乎恆為 0**
#:    (stot_repeat.sh 的 for 迴圈對失敗一律 continue)。成敗要看
#:    stot_repeat_v2.csv 的 pass 欄與 trial log 的結案哨兵,不要看 rc。
TRIAL_RC_MEANING = {
    0: "結案(是否達標看摘要行的數字,不是看 rc)",
    1: "前置/彈射失敗",
    2: "真逾時(飛不完)",
    3: "節點提前死亡",
    4: "impact_settle 窄縫(已命中 6 但摘要沒印完)",
    130: "被中止(收到 INT/TERM)",
}


def describe_rc(rc: Optional[int]) -> str:
    if rc is None:
        return "尚未結束"
    return "rc=%d(%s)" % (rc, TRIAL_RC_MEANING.get(rc, "未知"))
