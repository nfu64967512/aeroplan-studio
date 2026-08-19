"""地圖右鍵「到達 / 攻擊」→ **待命中的 tot_node** 的純邏輯層(零 GUI、零 WSL 相依)。

這一層只做可測的純函式:

  1. 右鍵語意判定(到達 / 攻擊;高度 0 = 攻擊快捷)
  2. 組出要送給 ``wsl.exe`` 的三條指令(前置檢查 / 送目標 / 監看+收監看)
  3. 解析前置檢查輸出 → 閘門清單
  4. 解析 ``ros2 topic echo`` 的輸出 → 目前相位、節點回讀的目標

Qt / subprocess / QMessageBox 都在 ``ui/controllers/rclick_strike_controller.py``。
分開是為了讓 ``tests/test_rclick_strike.py`` 能在**沒有 PyQt6** 的 WSL python3 跑完。

────────────────────────────────────────────────────────────────────────
這條路和「⑦ 蜂群打擊」那條**不是同一條**
────────────────────────────────────────────────────────────────────────
    integrations/stot_ros2_bridge.py   批次:整包 shell out 到 run_west.sh,
                                       會**拆掉重建 SITL**、跑完一趟才結束
    本檔                                互動:SITL 與 ROS2 層都是常駐的,
                                       只把「一個座標」丟給待命中的 tot_node

兩者共用的只有幾何/格式化函式(haversine / bearing / fmt_coord / Issue),
刻意 import 而不是抄一份 —— 抄一份的話兩邊會漂移,而漂移是靜默的。

────────────────────────────────────────────────────────────────────────
與 tot_node(任務 A)的契約 —— 這裡是**唯一**的真相源
────────────────────────────────────────────────────────────────────────
    topic     /swarm/tot_target
    型別      std_msgs/msg/String
    內容      JSON: {"lat": <float>, "lon": <float>, "approach_from": <float>}
              ``approach_from`` 是選填;節點沒實作就會忽略它(見下)。

為什麼是 String+JSON 而不是自訂 msg:
  * 自訂 msg 要改 sar_swarm 的 package.xml/CMakeLists 並 **colcon build**,
    而 build 產物要靠 deploy_build.sh 同步 —— 本輪明文禁止跑它。
    String 是 std_msgs,兩邊都已經在用(tot_node.py:532/533 的 gate/status
    也是 String+JSON),零建置成本。
  * 加欄位不必重編:未來要帶 target_amsl / r_fan 只是多一個 key。
  * ⚠️ 代價是 ``ros2 topic pub`` 的引號地獄(YAML 裡包 JSON 再包進 bash)。
    所以 build_publish_command() **自己把組好的指令解回來對帳**
    (見 _selfcheck_roundtrip),組錯會當場丟例外而不是送出一包爛資料。

為什麼傳輸用 ``wsl.exe -e bash -lc 'ros2 topic pub …'``:
  * 不新增傳輸層。Windows 端沒有 rclpy,自建 socket/HTTP 橋要在 WSL 端多跑一個
    常駐服務,而本輪不能改 scripts/wsl/ 也不能部署。
  * 走 MAVLink(6760+10i)是明確的禁區:MAVProxy 的 ``--out tcpin`` 只餵先連上
    的 client,AeroPlan 自己會被餓死 —— 本專案已踩過。
  * ``ros2 topic pub --once`` 預設 ``-w 1``(等到有 1 個匹配訂閱者才送),
    **沒有訂閱者就會一直卡住** ⇒ 一律包 ``timeout``,rc=124 就是「沒人在聽」。
    這是刻意的:寧可卡住被 timeout 抓到,也不要「送出成功但沒人收」。

────────────────────────────────────────────────────────────────────────
外部字串來源(上游改了這裡要一起改)
────────────────────────────────────────────────────────────────────────
    /swarm/tot_gate     tot_node.py:532  latched(TRANSIENT_LOCAL),內容含
                        target/r_fan/half_angle/approach_from/slots
                        ⇒ **回讀驗證**用:節點自己算完寫下的目標
    /swarm/tot_status   tot_node.py:533  20 Hz,內容 {"phase","T_arr","uavs"}
                        ⇒ 相位顯示用。phase 取值見 PHASE_* 常數
    setup.bash 兩行     swarm_ros2_up.sh:149-150 / stot_trial.sh:373-374
"""

from __future__ import annotations

import json
import math
import re
import shlex
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# ⚠️ 刻意共用批次橋接的純函式:argv 形狀、座標格式、Issue 呈現只准有一種。
#    這個 import 是單向的(本檔 → stot_ros2_bridge),不會反過來,也不會
#    動到那條批次路徑的任何行為。
from integrations.stot_ros2_bridge import (  # noqa: F401
    DEFAULT_LOC,
    GEOMETRY_MARGIN,
    HOME_BY_LOC,
    RUNS_SUBPATH,
    WSL_DISTRO_DEFAULT,
    Issue,
    StotBridgeError,
    approach_from_for,
    bearing_deg,
    fmt_coord,
    haversine_m,
    target_matches,
    validate_target,
)
from integrations.stot_ros2_bridge import _wsl as _wsl_argv

# ═════════════════════════════════════════════════════════════════════
#  契約常數
# ═════════════════════════════════════════════════════════════════════

TARGET_TOPIC = "/swarm/tot_target"
GATE_TOPIC = "/swarm/tot_gate"        # tot_node.py:532
STATUS_TOPIC = "/swarm/tot_status"    # tot_node.py:533
STRING_MSG = "std_msgs/msg/String"

#: payload 的鍵。lat/lon 必填,其餘選填(節點沒實作就忽略)。
PAYLOAD_REQUIRED_KEYS = ("lat", "lon")
PAYLOAD_OPTIONAL_KEYS = ("approach_from", "amsl")

#: 海上紅心的高度基準。**海面就是 0 m AMSL,不要去查地形庫** ——
#: SRTM 在水域沒有資料,回的是垃圾;實測散布 17 m,而驗收門檻是 3 s。
#: 對應批次路徑的 ``TARGET_AMSL=0.0``(見 repro_two1.sh)。
SEA_LEVEL_AMSL = 0.0

#: ROS2 環境。``bash -lc`` 進來時 ~/.bashrc 會在非互動時提早 return,
#: 所以**一定要自己 source**,不能假設 ros2 在 PATH 上。
#: 來源 swarm_ros2_up.sh:149-150(與 stot_trial.sh:373-374 同一組)。
ROS_SETUP_FILES = ("/opt/ros/humble/setup.bash", "$HOME/fixwing_ws/install/setup.bash")

#: 各段的硬時限(秒)。全部都要有 —— 這條鏈上每一個 ros2 CLI 都可能永久卡住。
PUB_TIMEOUT_S = 25        # ros2 topic pub --once(等訂閱者)
INFO_TIMEOUT_S = 10       # ros2 topic info
ECHO_TIMEOUT_S = 5        # ros2 topic echo --once(監看迴圈用)
#: 前置檢查裡的兩個 echo 探針。右鍵之後 UI 是卡住的,每個探針又要付一次
#: ros2 CLI 的 python 啟動成本(~1-2 s)⇒ 這裡壓得比監看短。
PROBE_TIMEOUT_S = 3
WATCH_PERIOD_S = 3        # 監看取樣週期
WATCH_MAX_S = 5400        # 監看自我到期(90 分鐘)—— 就算沒收乾淨也不會永遠留著

#: pub 逾時的 rc(coreutils timeout 的慣例)。
RC_TIMEOUT = 124

# ── 右鍵語意 ─────────────────────────────────────────────────────────
MODE_ARRIVE = "ARRIVE"
MODE_STRIKE = "STRIKE"

#: 高度框輸入 0 = 攻擊快捷。**只是快捷,不是預設** —— 呼叫端一定要跳確認框。
ALT_STRIKE_SHORTCUT = 0.0
#: 到達路徑的合法高度下限(來源:main_window.py 既有的 QInputDialog.getDouble)。
ALT_ARRIVE_MIN = 5.0
ALT_ARRIVE_MAX = 2000.0

# ── tot_node 的 phase 取值(tot_node.py:634/1691/1746/1864/2303)──────
PHASE_STANDBY = "STANDBY"    # 任務 A 新增的待命相位
PHASE_INIT = "INIT"
PHASE_ALIGN = "ALIGN"
PHASE_PERCH = "PERCH"
PHASE_DIVE = "DIVE"
PHASE_DONE = "DONE"

#: 這些相位代表「節點正在指揮飛機」⇒ 不可以再送第二個目標。
PHASE_BUSY = (PHASE_ALIGN, PHASE_PERCH, PHASE_DIVE)
#: 這些相位代表可以接目標。
PHASE_READY = (PHASE_STANDBY, PHASE_INIT, PHASE_DONE)

PHASE_LABEL = {
    PHASE_STANDBY: "STANDBY 待命",
    PHASE_INIT: "INIT 起動中",
    PHASE_ALIGN: "ALIGN 扇形集結",
    PHASE_PERCH: "PERCH 對準內切環",
    PHASE_DIVE: "DIVE 同步俯衝",
    PHASE_DONE: "DONE 結案",
}

#: 台灣周邊的合理框(南北、東西)。只拿來做 lat/lon 顛倒的提醒,不做硬擋 ——
#: 使用者可能真的在別的戰區點圖。
TAIWAN_BOX = (21.0, 26.5, 118.0, 123.5)   # lat_min, lat_max, lon_min, lon_max

#: 監看檔名前綴。跑在 ~/fixwing_ws/runs/ 底下,與批次航次的 trial_*.log 不撞名。
WATCH_PREFIX = "rclick_strike_"


# ═════════════════════════════════════════════════════════════════════
#  右鍵語意
# ═════════════════════════════════════════════════════════════════════

def mode_for_altitude(alt: float) -> str:
    """高度框的輸入值 → 語意。

    使用者原話:「右鍵如果輸入 0 公尺,或是拉一個選單可以選到達或是攻擊」。
    ⚠️ 0 只是**快捷**:呼叫端收到 MODE_STRIKE 之後**必須**跳確認框。
       默默把「飛到此點」改成「打擊此點」是這個專案最不能接受的那類行為。
    """
    a = float(alt)
    if not math.isfinite(a):
        raise StotBridgeError("高度不是有限數:%r" % (alt,))
    return MODE_STRIKE if abs(a - ALT_STRIKE_SHORTCUT) < 1e-9 else MODE_ARRIVE


def arrive_altitude_ok(alt: float) -> bool:
    """到達路徑的高度是否合法。

    既有 UI 的下限是 5.0(getDouble 的 min),為了讓 0 進得來才放寬到 0.0。
    0 已經被 mode_for_altitude 接走,剩下 0<alt<5 是舊 UI **產生不出來**的值 ⇒
    不要靜默夾到 5,直接擋下來並說明。
    """
    a = float(alt)
    return math.isfinite(a) and ALT_ARRIVE_MIN <= a <= ALT_ARRIVE_MAX


def looks_swapped(lat: float, lon: float) -> bool:
    """(lat, lon) 看起來是不是顛倒了。

    台灣附近 lat≈25 / lon≈121,顛倒之後 lat=121 會被 validate_target 硬擋;
    但 (24.9, 25.1) 這種「兩個都像緯度」的顛倒不會被範圍擋到,所以這裡再補一層:
    **原順序不在框內、交換後在框內** ⇒ 提醒(不硬擋,使用者可能真的在別處)。
    """
    la_min, la_max, lo_min, lo_max = TAIWAN_BOX

    def _in(a: float, o: float) -> bool:
        return la_min <= a <= la_max and lo_min <= o <= lo_max

    try:
        la, lo = float(lat), float(lon)
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(la) and math.isfinite(lo)):
        return False
    return (not _in(la, lo)) and _in(lo, la)


# ═════════════════════════════════════════════════════════════════════
#  payload 組裝 —— 引號地獄的唯一入口
# ═════════════════════════════════════════════════════════════════════

def build_target_payload(lat: float, lon: float,
                         approach_from: Optional[float] = None,
                         amsl: Optional[float] = None) -> str:
    """組出要放進 String.data 的 JSON。

    ⚠️ 座標**逐字**用 fmt_coord(``%.7f``)寫進 JSON,不走 json.dumps(float):
       json.dumps 會把 25.1406000 印回最短表示 25.1406,於是「確認框上看到的」
       和「線上送出去的」就不是同一串字。使用者的驗收點就是逐字對帳,
       不能在這裡自作聰明。
    ⚠️ ``%.7f`` 也保證數字**永遠帶小數點**(25 → 25.0000000)。同一個坑本專案
       吃過:stot_trial.sh:376 的 tlat/tlon 沒套 _f(),送出看起來像整數的
       "121",ROS2 判成 INTEGER 而宣告是 DOUBLE ⇒ 節點一起來就
       InvalidParameterTypeException(對外表現成 rc=3 節點提前死亡)。
    ⚠️ 手拼字串 ⇒ 最後一定要 json.loads 驗一次,不能只靠肉眼。

    ⚠️⚠️ ``amsl`` = 紅心所在地表的海拔基準,**海上目標非帶不可**。
       不帶的話 tot_node 的 target_amsl 是 nan ⇒ 回頭去查地形庫,
       而 SRTM 在水域**沒有資料、回的是垃圾**(實測散布 17 m)。
       台灣海峽的紅心一律 ``amsl=0.0``。
       —— 這是本專案 sea-level-target-datum 那條血淚:批次路徑早就用
       ``TARGET_AMSL=0.0`` 釘死,而右鍵這條產品路徑 2026-08-07 之前
       **根本沒有這個欄位**,等於每一發海上打擊都在用垃圾高程。
       刻意做成 Optional 而不是預設 0.0:陸上目標用地形高才是對的,
       預設任何一邊都會在另一邊靜默地錯。由呼叫端明示。
    """
    la, lo = validate_target(lat, lon)
    parts = ['"lat": %s' % fmt_coord(la), '"lon": %s' % fmt_coord(lo)]
    if approach_from is not None:
        af = float(approach_from)
        if not math.isfinite(af):
            raise StotBridgeError("approach_from 不是有限數:%r" % (approach_from,))
        parts.append('"approach_from": %.1f' % (af % 360.0))
    if amsl is not None:
        am = float(amsl)
        if not math.isfinite(am):
            # nan 在這裡**特別危險**:json.dumps(nan) 產生 `NaN`,不是合法 JSON,
            # 節點端 json.loads 會整包丟掉 ⇒ 整發打擊靜默消失。
            raise StotBridgeError("amsl 不是有限數:%r(要關掉請傳 None)" % (amsl,))
        parts.append('"amsl": %.1f' % am)
    payload = "{" + ", ".join(parts) + "}"
    try:
        json.loads(payload)          # 自我驗證:組出來的必須是合法 JSON
    except ValueError as exc:
        raise StotBridgeError("組出來的 payload 不是合法 JSON:%s(%r)" % (exc, payload))
    return payload


def ros2_string_arg(payload: str) -> str:
    """JSON → ``ros2 topic pub`` 吃的 YAML 引數,**外面已經包好 bash 雙引號**。

    三層剝殼(由外而內):
        bash 雙引號  "…"          → bash 把 \\" 還原成 "
        YAML 單引號  {data: '…'}  → YAML 把單引號內容原樣當字串
        JSON         {"lat": …}   → 節點 json.loads

    所以 payload 裡不能有單引號(會提前關掉 YAML 的字串)也不能有反斜線
    (bash 與 YAML 兩層都會吃掉)。JSON 數字不會產生這兩者,但還是硬擋 ——
    這種東西一旦漏掉就是「送出去的和看到的不一樣」。
    """
    if "'" in payload:
        raise StotBridgeError("payload 不能含單引號(YAML 單引號字串會提前結束):%r" % payload)
    if "\\" in payload:
        raise StotBridgeError("payload 不能含反斜線(bash/YAML 會各吃一層):%r" % payload)
    return '"{data: \'%s\'}"' % payload.replace('"', '\\"')


_YAML_DATA_RE = re.compile(r"^\{data:\s*'(.*)'\}$", re.S)


def decode_publish_arg(script_or_arg: str) -> Dict[str, Any]:
    """把組好的**指令字串**解回節點會收到的 dict —— 走與組裝完全不同的路徑。

    shlex(POSIX 模式)對雙引號/反斜線的處理和 bash 一致,所以這是真正的
    「照 bash 的規則再讀一次」,不是把組裝邏輯抄一遍。
    build_publish_command() 內部會呼叫它自我對帳(見 _selfcheck_roundtrip)。
    """
    for tok in shlex.split(script_or_arg, posix=True):
        m = _YAML_DATA_RE.match(tok.strip())
        if m:
            return json.loads(m.group(1))
    raise StotBridgeError("指令裡找不到 {data: '…'} 引數 —— 組裝形狀壞了")


def _selfcheck_roundtrip(script: str, lat: float, lon: float,
                         amsl: Optional[float] = None) -> None:
    """組完立刻解回來比對。引號組錯是靜默的,只有這一步會讓它出聲。"""
    got = decode_publish_arg(script)
    for key in PAYLOAD_REQUIRED_KEYS:
        if key not in got:
            raise StotBridgeError("payload 缺少必填鍵 %r:%r" % (key, got))
    if abs(float(got["lat"]) - float(lat)) > 1e-7 or \
       abs(float(got["lon"]) - float(lon)) > 1e-7:
        raise StotBridgeError(
            "指令自我對帳失敗:要送 (%s, %s),但指令解回來是 (%r, %r)"
            % (fmt_coord(lat), fmt_coord(lon), got.get("lat"), got.get("lon")))
    # ⚠️ amsl 也要對帳。它決定的是**紅心的高度基準**,錯了整發打在錯的平面上,
    #    而且錯得很安靜(高度差幾十公尺不會有任何一個檢查出聲)。
    if amsl is None:
        if "amsl" in got:
            raise StotBridgeError("沒有要送 amsl,但指令裡有:%r" % (got["amsl"],))
    else:
        if "amsl" not in got:
            raise StotBridgeError("要送 amsl=%.1f,但指令裡沒有" % float(amsl))
        if abs(float(got["amsl"]) - float(amsl)) > 1e-6:
            raise StotBridgeError(
                "指令自我對帳失敗:要送 amsl=%.1f,但指令解回來是 %r"
                % (float(amsl), got["amsl"]))


# ═════════════════════════════════════════════════════════════════════
#  wsl.exe 指令
# ═════════════════════════════════════════════════════════════════════

def _source_ros() -> str:
    """source 兩行 ROS2 環境。

    ⚠️ 不可以用 ``set -u``:setup.bash 會參照未綁定的 AMENT_TRACE_SETUP_FILES,
       整個腳本會當場中止(stot_trial.sh:13 / stot_repeat.sh:21 的血淚註解)。
    """
    return "\n".join('source %s >/dev/null 2>&1' % f for f in ROS_SETUP_FILES)


def build_preflight_command(distro: str = WSL_DISTRO_DEFAULT) -> List[str]:
    """一次 wsl.exe 呼叫查完所有前置狀態,輸出 ``KEY=VALUE`` 行。

    ⚠️ 三個刻意的限制(和批次橋接同一套理由):
      * **絕不建立 MAVLink 連線**去「順手測一下埠通不通」。MAVProxy 的
        ``--out tcpin`` 只 accept 一次,測一下就會把 AeroPlan 自己餓死。
        SITL 在不在只用 ``pgrep -x arduplane`` 判。
      * **一次呼叫查完**:每次 spawn wsl.exe 要 100–300 ms。
      * ``pgrep -c`` 沒命中會印 0 **且** exit 1 ⇒ 一律 ``pgrep … | wc -l``。

    ⚠️⚠️ 這段指令自己的 cmdline 就是 pgrep -f 的掃描對象 ⇒ **不准出現裸的
       行程名**。中括號寫法(``[t]ot_node``)不會匹配到自己,那是刻意的。
       (批次橋接第一版就是踩這個坑:TRIALSH 恆為 2,功能永遠開不起來。)

    ⚠️ TOTNODE 只判「有沒有」,**不判等於 1**:``ros2 run`` 是 Popen 不是 exec,
       wrapper 與節點兩個行程的 cmdline 都含 tot_node,單一節點也會數到 2。
    """
    body = "\n".join([
        "set +e",
        _source_ros(),
        'echo "HOME=$HOME"',
        'echo "ROS2CLI=$(command -v ros2 >/dev/null 2>&1 && echo 1 || echo 0)"',
        'echo "ARDUPLANE=$(pgrep -x arduplane 2>/dev/null | wc -l)"',
        'echo "MAVROS=$(pgrep -x mavros_node 2>/dev/null | wc -l)"',
        'echo "SWARMNODE=$(pgrep -f \'[m]aster_node|[w]ingman_node|[b]ackup_node\''
        ' 2>/dev/null | wc -l)"',
        'echo "TOTNODE=$(pgrep -f \'[t]ot_node\' 2>/dev/null | wc -l)"',
        'echo "TRIALSH=$(pgrep -f \'[s]tot_trial.sh\' 2>/dev/null | wc -l)"',
        'echo "REPEATSH=$(pgrep -f \'[s]tot_repeat.sh\' 2>/dev/null | wc -l)"',
        'echo "WESTSH=$(pgrep -f \'[r]un_west.sh\' 2>/dev/null | wc -l)"',
        # 訂閱者數 = 「有沒有待命中的 tot_node 在聽」。這是最關鍵的一關:
        # ros2 topic pub 對零訂閱者是靜默的(--once 會卡到 timeout),
        # 而「送出去石沉大海」正是本專案最貴的那類錯誤。
        '_ti="$(timeout %d ros2 topic info %s 2>/dev/null)"' % (INFO_TIMEOUT_S, TARGET_TOPIC),
        'echo "TOPICRC=$?"',
        'printf "%s\\n" "$_ti" | sed -n \'s/^Subscription count: *\\([0-9][0-9]*\\).*/SUBCOUNT=\\1/p\'',
        'printf "%s\\n" "$_ti" | sed -n \'s/^Type: *\\(.*\\)$/TOPICTYPE=\\1/p\'',
        # 目前相位。**拿不到不算失敗**(待命中的節點不一定在發 status),
        # 只有拿到且是 ALIGN/PERCH/DIVE 才代表「上一發還在進行」。
        'timeout %d ros2 topic echo --once --full-length %s %s 2>/dev/null'
        % (PROBE_TIMEOUT_S, STATUS_TOPIC, STRING_MSG),
        # gate 是 latched(TRANSIENT_LOCAL,tot_node.py:50-52)⇒ 只要節點發過
        # 一次就馬上拿得到。用途是拿**節點自己生效的 r_fan** 來擋幾何,
        # 比在 Python 端猜一個常數可靠(猜錯就是靜默放行一趟反向航路)。
        'timeout %d ros2 topic echo --once --full-length %s %s 2>/dev/null'
        % (PROBE_TIMEOUT_S, GATE_TOPIC, STRING_MSG),
        'echo "PREFLIGHT_OK=1"',
    ])
    return _wsl_argv(distro, body)


def build_publish_command(lat: float, lon: float, *,
                          approach_from: Optional[float] = None,
                          amsl: Optional[float] = None,
                          distro: str = WSL_DISTRO_DEFAULT) -> List[str]:
    """把右鍵那一點送給待命中的 tot_node。完整的 ``wsl.exe …`` argv。

    ⚠️ ``--once`` 預設會等 1 個匹配訂閱者(``-w 1``,這裡寫明以免將來預設改掉)。
       沒有訂閱者就會**永遠卡住** ⇒ 外面包 ``timeout``,rc=124 就是「沒人在聽」。
       這比「pub 成功但沒人收」好太多 —— 後者完全靜默。
    """
    la, lo = validate_target(lat, lon)
    payload = build_target_payload(la, lo, approach_from=approach_from, amsl=amsl)
    script = "\n".join([
        "set +e",
        _source_ros(),
        "timeout %d ros2 topic pub --once -w 1 %s %s %s"
        % (PUB_TIMEOUT_S, TARGET_TOPIC, STRING_MSG, ros2_string_arg(payload)),
        'echo "PUB_RC=$?"',
    ])
    _selfcheck_roundtrip(script, la, lo, amsl=amsl)
    return _wsl_argv(distro, script)


_PUB_RC_RE = re.compile(r"^PUB_RC=(-?\d+)\s*$", re.M)


def parse_pub_rc(stdout: str) -> Optional[int]:
    """取出 ``ros2 topic pub`` 的 rc。找不到 = 指令沒跑到底,呼叫端要當失敗。"""
    m = _PUB_RC_RE.search(stdout or "")
    return int(m.group(1)) if m else None


def describe_pub_rc(rc: Optional[int]) -> str:
    if rc is None:
        return ("指令沒有回報 PUB_RC —— 連 ros2 topic pub 都沒跑到。"
                "多半是 WSL 或 ROS2 環境有問題。")
    if rc == 0:
        return "送出成功(有訂閱者接走了)"
    if rc == RC_TIMEOUT:
        return ("等不到訂閱者,%d 秒後逾時 —— **沒有任何 tot_node 在聽 %s**。"
                "訊息沒有送出去。" % (PUB_TIMEOUT_S, TARGET_TOPIC))
    return "ros2 topic pub 失敗(rc=%d)" % rc


def watch_log_name(stamp: str) -> str:
    """監看檔名。stamp 由呼叫端給(通常是 %m%d_%H%M%S),不要在這裡拿時鐘。"""
    if not re.match(r"^[A-Za-z0-9_-]{1,32}$", stamp or ""):
        raise StotBridgeError("stamp 不合法:%r" % (stamp,))
    return "%s%s.log" % (WATCH_PREFIX, stamp)


def build_watch_command(stamp: str, *,
                        period_s: int = WATCH_PERIOD_S,
                        max_s: int = WATCH_MAX_S,
                        distro: str = WSL_DISTRO_DEFAULT) -> List[str]:
    """起一個常駐取樣器,把 status 與 gate 週期性寫進 runs/ 底下的檔案。

    為什麼是「WSL 端落檔 + Windows 端讀檔」而不是 PIPE:
      * mission/sitl_launcher.py:658-660 的教訓:PIPE 滿了會 deadlock
      * ``\\\\wsl.localhost\\<distro>\\…`` 從 Windows 直接可讀,輪詢不必每秒
        spawn 一次 wsl.exe(批次橋接已實測)

    為什麼是 ``--once`` 迴圈而不是連續 echo:
      tot_node 的 status 是 **20 Hz**(tot_node.py:558 的 0.05 s timer),
      連續 echo 一趟 30 分鐘會寫十幾 MB、還多一個 python 行程在燒 CPU。
      每 WATCH_PERIOD_S 秒取一個完整樣本就夠 UI 用。

    ⚠️ 一定要 ``--full-length``:``ros2 topic echo`` 預設把字串截到 128 字元,
       而 gate/status 的 JSON 都遠超過 ⇒ 不加就是解析不出來(而且靜默)。
    ⚠️ 監看行程會**自我到期**(WATCH_MAX_S 秒)。就算 UI 當掉沒收乾淨,
       它也不會永遠留著。
    """
    name = watch_log_name(stamp)
    body = "\n".join([
        "set +e",
        _source_ros(),
        'RUNS="$HOME/%s"' % RUNS_SUBPATH,
        'mkdir -p "$RUNS"',
        'LOG="$RUNS/%s"' % name,
        ': > "$LOG"',
        '_sample() {',
        "  trap '' HUP",                       # 沒有 tty,理論上不會來;還是擋著
        '  echo $BASHPID > "$LOG.pid"',        # ⚠️ 子殼要用 BASHPID,$$ 是父殼
        '  local t0=$SECONDS',
        '  while [ $((SECONDS-t0)) -lt %d ]; do' % int(max_s),
        '    timeout %d ros2 topic echo --once --full-length %s %s 2>/dev/null'
        % (ECHO_TIMEOUT_S, STATUS_TOPIC, STRING_MSG),
        '    timeout %d ros2 topic echo --once --full-length %s %s 2>/dev/null'
        % (ECHO_TIMEOUT_S, GATE_TOPIC, STRING_MSG),
        '    sleep %d' % int(period_s),
        '  done',
        '}',
        '_sample >> "$LOG" 2>&1 &',
        'disown 2>/dev/null',          # 非互動 shell 沒有 job control,失敗無所謂
        'sleep 0.5',
        'echo "WATCH_LOG=$LOG"',
        'echo "WATCH_PID=$(cat "$LOG.pid" 2>/dev/null)"',
    ])
    return _wsl_argv(distro, body)


def build_stop_watch_command(pid: Optional[int],
                             distro: str = WSL_DISTRO_DEFAULT) -> List[str]:
    """收掉監看行程,並**讀回來驗證**。

    ⚠️ 只用 PID,不用 ``pkill -f``:能唯一命中監看行程的字串是
       ``topic echo /swarm/tot_status``,而前置檢查的指令裡也有同一段
       —— 用 pattern 殺會誤傷正在跑的前置檢查。PID 拿不到就大聲說,
       別亂殺(監看本身有 WATCH_MAX_S 秒自我到期,漏掉不會永遠留著)。
    """
    if pid is None:
        return _wsl_argv(distro, 'echo "WATCH_STOP=nopid"')
    p = int(pid)
    if p <= 1:
        raise StotBridgeError("監看 PID 不合法:%r" % (pid,))
    body = "; ".join([
        "set +e",
        "pkill -P %d 2>/dev/null" % p,     # 先收它正在等的 ros2/timeout 子行程
        "kill -TERM %d 2>/dev/null" % p,
        "sleep 1",
        "kill -KILL %d 2>/dev/null" % p,
        'echo "WATCH_STOP=$(kill -0 %d 2>/dev/null && echo alive || echo gone)"' % p,
    ])
    return _wsl_argv(distro, body)


_WATCH_PID_RE = re.compile(r"^WATCH_PID=(\d+)\s*$", re.M)
_WATCH_LOG_RE = re.compile(r"^WATCH_LOG=(.+?)\s*$", re.M)


def parse_watch_start(stdout: str) -> Tuple[Optional[int], str]:
    """(pid, wsl 端 log 絕對路徑)。pid 拿不到就回 None —— 呼叫端要說出來。"""
    mp = _WATCH_PID_RE.search(stdout or "")
    ml = _WATCH_LOG_RE.search(stdout or "")
    return (int(mp.group(1)) if mp else None, ml.group(1) if ml else "")


def runs_unc(wsl_home: str, distro: str = WSL_DISTRO_DEFAULT) -> str:
    """WSL 的 $HOME → Windows 看得到的 UNC 路徑(runs 目錄)。

    ``\\\\wsl.localhost\\<distro>\\home\\<user>\\fixwing_ws\\runs``
    已實測從 Windows 直接可讀 ⇒ 相位輪詢不必 spawn wsl.exe。
    """
    home = (wsl_home or "").strip().rstrip("/")
    if not home.startswith("/"):
        home = "/home"
    return "\\\\wsl.localhost\\%s%s\\%s" % (
        distro, home.replace("/", "\\"), RUNS_SUBPATH.replace("/", "\\"))


# ═════════════════════════════════════════════════════════════════════
#  ros2 topic echo 的輸出解析
# ═════════════════════════════════════════════════════════════════════

_ECHO_DATA_RE = re.compile(r"^data:\s*(.*?)\s*$", re.M)


def _unyaml_scalar(raw: str) -> Optional[str]:
    """把 ``ros2 topic echo`` 印出來的 YAML 純量還原成原字串。

    rosidl_runtime_py 用 yaml.dump(width=sys.maxsize) ⇒ 不會折行;
    含 ``{`` / ``"`` 的字串會被包成單引號純量,內部單引號寫成兩個。
    """
    s = (raw or "").strip()
    if len(s) >= 2 and s[0] == "'" and s[-1] == "'":
        return s[1:-1].replace("''", "'")
    if len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        try:
            return json.loads(s)          # YAML 雙引號跳脫與 JSON 夠接近
        except ValueError:
            return s[1:-1]
    return s or None


def parse_echo_payloads(text: str) -> List[dict]:
    """一段 echo 輸出 → 所有解得開的 JSON payload(順序保留)。

    status 與 gate 混在同一個檔案裡,靠內容的鍵分辨(status 有 phase、
    gate 有 target/slots)—— 比在檔案裡插自訂分隔線可靠,echo 自己也會印 ``---``。
    """
    out: List[dict] = []
    for m in _ECHO_DATA_RE.finditer(text or ""):
        raw = _unyaml_scalar(m.group(1))
        if not raw:
            continue
        try:
            obj = json.loads(raw)
        except ValueError:
            continue
        if isinstance(obj, dict):
            out.append(obj)
    return out


def latest_status(payloads: List[dict]) -> Optional[dict]:
    """最後一個 status 樣本(有 phase 的那種)。"""
    for obj in reversed(payloads):
        if "phase" in obj:
            return obj
    return None


def latest_gate(payloads: List[dict]) -> Optional[dict]:
    """最後一個 gate 樣本(有 target 的那種)。"""
    for obj in reversed(payloads):
        if isinstance(obj.get("target"), dict):
            return obj
    return None


def gate_target(gate: Optional[dict]) -> Optional[Tuple[float, float]]:
    """gate payload → (lat, lon)。tot_node.py:640-645 的形狀。"""
    if not gate:
        return None
    t = gate.get("target")
    if not isinstance(t, dict):
        return None
    try:
        return float(t["lat"]), float(t["lon"])
    except (KeyError, TypeError, ValueError):
        return None


def status_phase(status: Optional[dict]) -> str:
    if not status:
        return ""
    ph = status.get("phase")
    return str(ph) if ph else ""


def status_text(status: Optional[dict]) -> str:
    """給狀態列用的一行字。"""
    if not status:
        return "尚未收到 tot_node 狀態"
    ph = status_phase(status)
    parts = [PHASE_LABEL.get(ph, ph or "?")]
    uavs = status.get("uavs")
    if isinstance(uavs, dict) and uavs:
        modes: "OrderedDict[str, int]" = OrderedDict()
        for v in uavs.values():
            if isinstance(v, dict):
                mode = str(v.get("mode", "?"))
                modes[mode] = modes.get(mode, 0) + 1
        if modes:
            parts.append(" ".join("%s×%d" % (k, n) for k, n in modes.items()))
    t_arr = status.get("T_arr")
    if isinstance(t_arr, (int, float)):
        parts.append("T_arr %.0fs" % t_arr)
    return " · ".join(parts)


def strike_finished(status: Optional[dict]) -> bool:
    """這一發打完了沒。DONE = tot_node.py:2303 的結案相位。"""
    return status_phase(status) == PHASE_DONE


# ═════════════════════════════════════════════════════════════════════
#  前置檢查
# ═════════════════════════════════════════════════════════════════════

@dataclass
class StandbyReport:
    """``build_preflight_command`` 輸出的結構化版本。"""
    wsl_home: str = ""
    ros2_cli: int = 0
    arduplane: int = 0
    mavros: int = 0
    swarm_node: int = 0
    tot_node: int = 0
    trial_sh: int = 0
    repeat_sh: int = 0
    west_sh: int = 0
    topic_rc: int = -1
    sub_count: Optional[int] = None      # None = topic 根本不存在
    topic_type: str = ""
    status: Optional[dict] = None        # /swarm/tot_status 的最後一個樣本
    gate: Optional[dict] = None          # /swarm/tot_gate(latched)的最後一個樣本
    ok: bool = False                     # PREFLIGHT_OK=1 ⇒ 腳本整段跑完

    @property
    def gate_r_fan(self) -> Optional[float]:
        """節點**實際生效**的扇形半徑。拿不到就回 None(不要拿常數硬猜)。"""
        try:
            return float(self.gate["r_fan"])          # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None

    @property
    def swarm_n(self) -> Optional[int]:
        """tot_node **實際生效**的機數。拿不到就回 None。

        ⚠️ 同一個理由:不要用常數 6。使用者定調「數量不要寫死」,而本檔
           2026-08-07 之前有五處寫死「六架」——n=2 的航次上,確認框會
           對著兩架飛機說「會立刻帶著六架進入 ALIGN」。
        """
        try:
            return int(self.status["n"])              # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None


#: ⚠️ 一定要含數字:第一版寫成 [A-Z_]+,``ROS2CLI=1`` 整行匹配不到,
#:    於是 ros2_cli 恆為 0 ⇒ 每一次都被「找不到 ros2 指令」擋下來。
_KV_RE = re.compile(r"^([A-Z0-9_]+)=(.*)$")
_FIELD_MAP = {
    "ROS2CLI": "ros2_cli", "ARDUPLANE": "arduplane", "MAVROS": "mavros",
    "SWARMNODE": "swarm_node", "TOTNODE": "tot_node", "TRIALSH": "trial_sh",
    "REPEATSH": "repeat_sh", "WESTSH": "west_sh", "TOPICRC": "topic_rc",
    "SUBCOUNT": "sub_count",
}


def parse_preflight(stdout: str) -> StandbyReport:
    rep = StandbyReport()
    for raw in (stdout or "").splitlines():
        m = _KV_RE.match(raw.strip())
        if not m:
            continue
        key, val = m.group(1), m.group(2).strip()
        if key == "HOME":
            rep.wsl_home = val
        elif key == "TOPICTYPE":
            rep.topic_type = val
        elif key == "PREFLIGHT_OK":
            rep.ok = (val == "1")
        elif key in _FIELD_MAP:
            try:
                setattr(rep, _FIELD_MAP[key], int(val))
            except ValueError:
                setattr(rep, _FIELD_MAP[key], -1)
    payloads = parse_echo_payloads(stdout)
    rep.status = latest_status(payloads)
    rep.gate = latest_gate(payloads)
    return rep


def evaluate_preflight(rep: StandbyReport, *,
                       target: Optional[Tuple[float, float]],
                       home: Tuple[float, float] = HOME_BY_LOC[DEFAULT_LOC],
                       expect_sitl: Optional[int] = None,
                       local_strike_active: bool = False) -> List[Issue]:
    """StandbyReport → 給使用者看的閘門清單。每一條都要**明講原因與處置**。

    blocking=True 的一律硬擋。這條路的每一次送出都會讓整個機隊真的開始俯衝,
    寧可多擋。
    """
    issues: List[Issue] = []

    if not rep.ok:
        issues.append(Issue(
            "WSL_UNREACHABLE",
            "WSL 前置檢查沒有跑完(缺 PREFLIGHT_OK)—— 後面的數字全部不可信。\n"
            "  處置:自己開一個終端機跑 wsl.exe -d %s -e bash -lc 'echo ok'"
            % WSL_DISTRO_DEFAULT))
        return issues          # 不要拿不可信的數字繼續推論

    if rep.ros2_cli != 1:
        issues.append(Issue(
            "NO_ROS2_CLI",
            "WSL 裡找不到 ros2 指令 —— %s 兩行 source 之後仍然沒有。\n"
            "  處置:確認 ROS2 有裝、workspace 有 build(deploy_build.sh 由你自己跑)。"
            % " / ".join(ROS_SETUP_FILES)))
        return issues          # 沒有 ros2 就沒有下面那些數字

    # ── 目標本身 ────────────────────────────────────────────────────
    if target is None:
        issues.append(Issue("NO_TARGET", "沒有座標 —— 右鍵事件沒有帶到點位。"))
    else:
        try:
            la, lo = validate_target(*target)
        except StotBridgeError as exc:
            issues.append(Issue(
                "BAD_TARGET",
                "%s\n  (lat/lon 顛倒最常見:台灣附近 lat≈25、lon≈121)" % exc))
        else:
            if looks_swapped(la, lo):
                issues.append(Issue(
                    "MAYBE_SWAPPED",
                    "座標 (%s, %s) 不在台灣周邊,但**對調之後就在** —— "
                    "確認一下 lat/lon 有沒有顛倒。" % (fmt_coord(la), fmt_coord(lo)),
                    blocking=False))
            d = haversine_m(home, (la, lo))
            issues.append(Issue(
                "GEOMETRY",
                "目標距起飛點 %.1f km、方位 %.0f°;扇形會擺在目標的 %.0f° 方位"
                "(= 蜂群來的方向)。"
                % (d / 1000.0, bearing_deg(home, (la, lo)),
                   approach_from_for((la, lo), home)),
                blocking=False))
            # 幾何裕度 D_home > r_fan(stot_repeat.sh:58-61 明載,但**沒有任何
            # 腳本會檢查**)。r_fan 取節點 latched gate 裡的生效值,拿不到就
            # 只能放行並說明 —— 不要拿常數硬猜,猜錯會靜默放掉一趟反向航路。
            r_fan = rep.gate_r_fan
            if r_fan is None:
                issues.append(Issue(
                    "NO_GEOMETRY_CHECK",
                    "拿不到 %s(節點還沒發過 gate)⇒ **沒辦法檢查目標離起飛點夠不夠遠**。\n"
                    "  太近的話扇形會蓋住起飛點,蜂群得先飛離目標再折返,"
                    "而且沒有任何腳本會擋。" % GATE_TOPIC, blocking=False))
            elif d < r_fan * GEOMETRY_MARGIN:
                issues.append(Issue(
                    "TARGET_TOO_CLOSE",
                    "目標離起飛點只有 %.1f km,必須 > %.1f km"
                    "(節點生效的扇形半徑 %.0f m × %.2f)。\n"
                    "  起飛點會落在扇形內側 ⇒ 整條航路是反的,"
                    "而且**沒有任何 WSL 腳本會擋下來**"
                    "(stot_repeat.sh:58-61 只寫在註解裡)。\n"
                    "  處置:點遠一點,或改用會重設 r_fan 的批次路徑。"
                    % (d / 1000.0, r_fan * GEOMETRY_MARGIN / 1000.0,
                       r_fan, GEOMETRY_MARGIN)))

    # ── 這條路自己的互斥 ─────────────────────────────────────────────
    if local_strike_active:
        issues.append(Issue(
            "LOCAL_STRIKE_ACTIVE",
            "AeroPlan 這邊還有一發沒結束 —— 等它跑完,或先按「中止監看」。\n"
            "  連送兩個目標會讓 tot_node 在半途換目標,全機隊的相位會炸開。"))

    # ── SITL 層(CMD1)────────────────────────────────────────────────
    if rep.arduplane <= 0:
        issues.append(Issue(
            "NO_SITL",
            "WSL 裡沒有任何 arduplane —— 第一個 CMD(SITL)沒起來。\n"
            "  處置:先跑 bash ~/fixwing_ws/sitl_run/swarm_sitl_up.sh,"
            "AeroPlan 要看得到全部機體再回來。"))
    else:
        # ⚠️ 期望機數以 **tot_node 生效的 n** 為準,呼叫端明講的優先。
        #    兩者都拿不到就**不報**這一條 —— 拿常數 6 去比,n=2 的航次
        #    每一次都會跳一條假警告,而真正該擋的(少了一台)反而被淹掉。
        want = expect_sitl if expect_sitl is not None else rep.swarm_n
        if want is not None and rep.arduplane != want:
            issues.append(Issue(
                "SITL_COUNT",
                "arduplane 有 %d 架,但 tot_node 生效的機數是 %d"
                "—— 對不上的那幾架不會參與這一發。"
                % (rep.arduplane, want), blocking=False))

    # ── ROS2 層(CMD2)────────────────────────────────────────────────
    if rep.tot_node <= 0:
        issues.append(Issue(
            "NO_TOT_NODE",
            "沒有 tot_node 在跑 —— 第二個 CMD(ROS2 層)沒起來,或它沒帶起 tot_node。\n"
            "  處置:先跑 bash ~/fixwing_ws/sitl_run/swarm_ros2_up.sh,"
            "等它印出 tot_node STANDBY 再回來。"))

    if rep.sub_count is None:
        issues.append(Issue(
            "NO_TOPIC",
            "ROS2 圖上找不到 %s(ros2 topic info rc=%d)。\n"
            "  代表沒有任何節點在聽這個 topic:送出去會**石沉大海**,"
            "而 ros2 topic pub 對這件事是靜默的。\n"
            "  最可能的原因:tot_node 是舊版(目標只吃啟動參數,沒有待命模式)。"
            % (TARGET_TOPIC, rep.topic_rc)))
    elif rep.sub_count <= 0:
        issues.append(Issue(
            "NO_SUBSCRIBER",
            "%s 存在但**訂閱者是 0** —— 沒有待命中的 tot_node 在聽。\n"
            "  送出去不會有人收(pub 會卡在等訂閱者,%d 秒後逾時)。\n"
            "  處置:確認 swarm_ros2_up.sh 起的 tot_node 真的進了 STANDBY。"
            % (TARGET_TOPIC, PUB_TIMEOUT_S)))
    elif rep.topic_type and rep.topic_type != STRING_MSG:
        issues.append(Issue(
            "TOPIC_TYPE",
            "%s 的型別是 %s,不是 %s —— 契約不符,送出去會被丟掉。"
            % (TARGET_TOPIC, rep.topic_type, STRING_MSG)))

    # ── 節點正在做什麼 ───────────────────────────────────────────────
    phase = status_phase(rep.status)
    if phase in PHASE_BUSY:
        issues.append(Issue(
            "NODE_BUSY",
            "tot_node 目前在 %s —— **上一發還在進行中**,不是待命狀態。\n"
            "  這時候送新目標會讓機隊在半途改打別的地方。等它到 DONE 再送。"
            % PHASE_LABEL.get(phase, phase)))
    elif not phase:
        issues.append(Issue(
            "NO_STATUS",
            "收不到 %s —— 無法確認 tot_node 是不是在待命"
            "(待命中的節點不一定會發 status,所以這不算失敗)。\n"
            "  想自己確認:wsl.exe -e bash -lc 'ros2 topic echo --once %s'"
            % (STATUS_TOPIC, STATUS_TOPIC), blocking=False))

    # ── 批次路徑正在跑 ⇒ 絕對不能插隊 ────────────────────────────────
    busy = []
    for label, n in (("stot_trial.sh", rep.trial_sh),
                     ("stot_repeat.sh", rep.repeat_sh),
                     ("run_west.sh", rep.west_sh)):
        if n > 0:
            busy.append("%s×%d" % (label, n))
    if busy:
        issues.append(Issue(
            "BATCH_RUNNING",
            "批次航次正在跑:%s\n"
            "  那條路會自己拆掉重建 SITL、自己起收 ROS2 層。"
            "現在插一個目標進去,兩邊會搶同一批飛機。\n"
            "  處置:等批次跑完,或在「蜂群打擊」分頁按 ABORT。" % "、".join(busy)))

    # ── 非阻擋的告知 ─────────────────────────────────────────────────
    if rep.mavros <= 0 or rep.swarm_node <= 0:
        issues.append(Issue(
            "NO_FORMATION_LAYER",
            "編隊層看起來沒起來(mavros_node×%d、長僚機節點×%d)。\n"
            "  tot_node 是直連飛控下指令的,還是打得出去;"
            "但飛機如果還停在地面,這一發不會有結果。"
            % (rep.mavros, rep.swarm_node), blocking=False))

    return issues


def blocking_issues(issues: List[Issue]) -> List[Issue]:
    return [i for i in issues if i.blocking]


# ═════════════════════════════════════════════════════════════════════
#  確認框文案 —— 逐字攤開要送出去的東西
# ═════════════════════════════════════════════════════════════════════

def confirm_text(lat: float, lon: float, *,
                 approach_from: Optional[float] = None,
                 amsl: Optional[float] = None,
                 n_aircraft: Optional[int] = None,
                 home: Tuple[float, float] = HOME_BY_LOC[DEFAULT_LOC],
                 via_alt_zero: bool = False) -> str:
    """送出前的確認框內容。

    使用者的驗收點:**能跟自己點的那一點對帳**。所以座標一律 %.7f 逐字列出,
    連要送出的 JSON 原文都攤開 —— 這個專案吃過太多次「參數靜默沒生效」。
    """
    la, lo = validate_target(lat, lon)
    payload = build_target_payload(la, lo, approach_from=approach_from, amsl=amsl)
    d = haversine_m(home, (la, lo))
    lines = []
    if via_alt_zero:
        lines += [
            "⚠️ 你在高度框輸入了 0 —— 這是「攻擊」的快捷,不是「飛到此點」。",
            "",
        ]
    lines += [
        "要對這一點執行 STOT 終端打擊:",
        "",
        "    緯度 lat = %s" % fmt_coord(la),
        "    經度 lon = %s" % fmt_coord(lo),
        "",
        "    距起飛點 %.1f km、方位 %.0f°" % (d / 1000.0, bearing_deg(home, (la, lo))),
    ]
    if approach_from is not None:
        lines.append("    扇形擺位 approach_from = %.1f°(蜂群來的方向)" % approach_from)
    # ⚠️ 高度基準一定要攤開,而且兩種情形都要說 —— 「沒帶 amsl」看起來像
    #    「沒這個欄位」,實際上是「回頭去查地形庫」,在海上那是垃圾值。
    if amsl is None:
        lines.append("    高度基準 = 由 tot_node 查地形庫"
                     "(⚠️ 水域無 SRTM 資料,海上目標不可用)")
    else:
        lines.append("    高度基準 amsl = %.1f m(釘死,不查地形庫)" % float(amsl))
    lines += [
        "",
        "實際會送出的訊息(逐字):",
        "    topic  %s" % TARGET_TOPIC,
        "    型別   %s" % STRING_MSG,
        "    data   %s" % payload,
        "",
        # ⚠️ 機數不可以寫死。n=2 的航次上,對著兩架飛機說「帶著六架」是**假話**,
        #    而使用者的驗收方式就是逐字對帳 —— 說錯數字會直接毀掉這個框的信用。
        #    拿不到就講「全部機體」,不要猜一個數字。
        "按下確定之後,待命中的 tot_node 會立刻帶著%s進入 ALIGN → PERCH → DIVE。"
        % ("全部機體" if n_aircraft is None else "%d 架" % int(n_aircraft)),
        "取消 = 什麼都不送。",
    ]
    return "\n".join(lines)


def sent_vs_gate_text(sent: Tuple[float, float],
                      echoed: Optional[Tuple[float, float]]) -> str:
    """回讀對帳的文字(送出值 vs 節點自己寫下的目標)。"""
    if echoed is None:
        return "節點還沒回報 gate,無法對帳"
    return ("送出 %s, %s / 節點 %s, %s"
            % (fmt_coord(sent[0]), fmt_coord(sent[1]),
               fmt_coord(echoed[0]), fmt_coord(echoed[1])))
