"""SITL 參數名值層級鎖 —— 擋掉「參數改名 ⇒ 靜默失效」這一整類。

背景
----
ArduPilot 對 `--defaults` 檔裡「名字合法但韌體不認識」的參數是**靜默略過**：
不報錯、不中止，只是那一行完全沒作用。2026-08-24 因此一次抓到 32 個死參數：

  * ``SYSID_THISMAV``  → 4.6+ 改名 ``MAV_SYSID``。整隊 SITL 都停在 sysid=1，
    HUD 三架併成一張卡、Cesium 一個圖示在三個起飛點之間瞬移。
  * ``ARMING_CHECK`` / ``GPS_HDOP_GOOD`` / ``ARSPD_PIN`` / ``ARSPD_BUS``
  * ``vtol_default.parm`` 的 28 個 —— 4.7 把 QuadPlane 那組改名**並改成 SI 單位**
    （cm/s → m/s、cdeg → deg），舊名整段等於沒寫。

順帶一提，未知但長度合法（<=16 字元）的參數名還會讓韌體把整份 defaults
反覆重載（實測 2 次 → 24 次），就是 SITL console 洗版的來源。

為什麼鎖在「值」而不是「源碼」
------------------------------
源碼層級的鎖（例如 grep 一行 ``MAV_SYSID``）只驗「那一行長什麼樣」，
改上游、改下游、換一條路徑寫檔都能繞過。這裡改成拿**實際會送進韌體的參數名**
去比對**韌體真實的參數表**，於是任何一條產生 defaults 的路徑寫錯名字都會紅。

韌體參數表快照
--------------
``sitl/default_params/firmware_param_names.json`` 是從實際開機的 SITL 用
``PARAM_REQUEST_LIST`` 讀回來的完整參數名清單（已驗證 收到數量 == param_count
且無缺號）。換 SITL binary 版本時要重新產生：

    for each vehicle: 啟動 SITL → mavutil PARAM_REQUEST_LIST → 收集 param_id
    → 確認 len(collected) == param_count → 存檔

注意 VTOL 那份必須在 ``Q_ENABLE=1`` 之下讀（否則 Q_* 子樹不會展開）。
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from mission.sitl_launcher import (
    _DEFAULT_PARAMS,
    _SCHED_LOOP_RATE,
    generate_vtol_params,
    identity_parm_text,
)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_SITL_DIR = _PROJECT_ROOT / "sitl"
_SNAPSHOT = _SITL_DIR / "default_params" / "firmware_param_names.json"

# 一行參數：NAME<sep>VALUE，'#' 之後為註解
_PARAM_LINE = re.compile(r"^([A-Z0-9_]+)\s*[=,\s]\s*(-?[\d.eE+]+)\s*$")


def _param_names(text: str, source: str = "<text>") -> list[str]:
    """從 .parm 內容抽出參數名（略過註解與空行）。

    ⚠ 解析不了的行會**丟錯**，不是默默跳過。
    若靜默跳過，一個 regex 不認得的格式會讓整份檔案解析出 0 個名字，
    「沒有死參數」就變成恆真的假綠燈 —— 這正是這組測試要防的東西。
    """
    names: list[str] = []
    unparsed: list[str] = []
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#")[0].strip()
        if not line:
            continue
        m = _PARAM_LINE.match(line)
        if m:
            names.append(m.group(1))
        else:
            unparsed.append(f"  {source}:{lineno}: {raw!r}")
    if unparsed:
        raise AssertionError(
            f"{source} 有 {len(unparsed)} 行無法解析成參數，"
            f"解析器與檔案格式不同步（靜默跳過會造成假綠燈）：\n"
            + "\n".join(unparsed[:10])
        )
    return names


@pytest.fixture(scope="module")
def firmware_names() -> dict[str, set[str]]:
    assert _SNAPSHOT.exists(), (
        f"找不到韌體參數表快照 {_SNAPSHOT}；"
        "換 SITL binary 後請依本檔檔頭說明重新產生。"
    )
    data = json.loads(_SNAPSHOT.read_text(encoding="utf-8"))
    return {k: set(v) for k, v in data.items() if not k.startswith("_")}


def test_snapshot_is_plausible(firmware_names):
    """快照本身要像一份真的參數表 —— 免得整組測試對著空集合空轉全綠。

    這是「儀表要能響」的自檢：若快照壞掉變成空的，下面每個測試都會假綠燈。
    """
    for fw in ("copter", "plane", "vtol"):
        assert fw in firmware_names, f"快照缺少 {fw}"
        assert len(firmware_names[fw]) > 1000, (
            f"{fw} 只有 {len(firmware_names[fw])} 個參數名，快照可能壞了"
        )
        # 幾個一定要存在的錨點
        for anchor in ("MAV_SYSID", "SCHED_LOOP_RATE", "SERIAL0_PROTOCOL"):
            assert anchor in firmware_names[fw], f"{fw} 快照缺少錨點 {anchor}"
    # VTOL 必須是在 Q_ENABLE=1 下讀的，否則 Q_* 子樹不會展開
    assert "Q_ENABLE" in firmware_names["vtol"]
    assert len([n for n in firmware_names["vtol"] if n.startswith("Q_")]) > 100, (
        "VTOL 快照的 Q_* 太少，可能不是在 Q_ENABLE=1 之下讀的"
    )


def test_old_names_are_really_gone(firmware_names):
    """確認這些舊名真的不在韌體裡 —— 這是上面那組測試有意義的前提。

    若哪天換了韌體、舊名又回來了，這個測試會紅，提醒我們重新評估。
    """
    for old in ("SYSID_THISMAV", "SYSID_MYGCS"):
        for fw in ("copter", "plane", "vtol"):
            assert old not in firmware_names[fw], (
                f"{old} 竟然存在於 {fw} 韌體 —— 快照或韌體版本變了，請重新檢視"
            )


@pytest.mark.parametrize("vehicle,fw", [("COPTER", "copter"),
                                        ("PLANE", "plane"),
                                        ("VTOL", "vtol")])
def test_identity_parm_names_exist_in_firmware(vehicle, fw, firmware_names):
    """launcher 寫出的 identity.parm，每個名字都要真的存在於該機型韌體。"""
    names = _param_names(identity_parm_text(vehicle, sysid=7), f"identity.parm[{vehicle}]")
    # identity.parm 模板固定 9 行；行數變了就要回來看，而不是讓下面的檢查空轉
    assert len(names) == 9, f"identity.parm 應有 9 個參數，實際 {len(names)}: {names}"
    dead = [n for n in names if n not in firmware_names[fw]]
    assert not dead, (
        f"{vehicle} 的 identity.parm 含有 {fw} 韌體不存在的參數名 {dead} —— "
        "這些會被 ArduPilot 靜默略過"
    )


@pytest.mark.parametrize("vehicle,fw", [("COPTER", "copter"),
                                        ("PLANE", "plane"),
                                        ("VTOL", "vtol")])
def test_default_params_file_names_exist_in_firmware(vehicle, fw, firmware_names):
    """copter.parm / plane.parm 裡的每個名字都要存在於對應韌體。"""
    parm = _SITL_DIR / _DEFAULT_PARAMS[vehicle]
    names = _param_names(parm.read_text(encoding="utf-8-sig", errors="replace"), parm.name)
    # 下限貼近實際值(copter 70 / plane 95),而不是寬鬆的 20 —— 檔案被清空一半也要紅
    assert len(names) >= 60, f"{parm} 只解析到 {len(names)} 個參數，格式可能變了"
    dead = [n for n in names if n not in firmware_names[fw]]
    assert not dead, f"{parm.name} 含有 {fw} 韌體不存在的參數名 {dead}"


def test_vtol_generated_params_exist_in_firmware(firmware_names, tmp_path):
    """generate_vtol_params() 產生的 QuadPlane 參數全部要存在於 VTOL 韌體。

    這份是**每次啟動都重新產生**的，所以必須鎖產生函式而不是磁碟上的檔案。
    """
    out = generate_vtol_params(tmp_path / "vtol_default.parm")
    names = _param_names(out.read_text(encoding="utf-8-sig", errors="replace"), out.name)
    assert len(names) >= 90, f"只解析到 {len(names)} 個參數，產生函式可能壞了"
    dead = [n for n in names if n not in firmware_names["vtol"]]
    assert not dead, (
        f"vtol_default.parm 含有 VTOL 韌體不存在的參數名 {dead} —— "
        "4.7 把 QuadPlane 那組改名並改成 SI 單位，請對照韌體參數表更新"
    )


def test_identity_pins_mav_sysid_by_value():
    """sysid 要真的寫進 MAV_SYSID，而且不是舊名。"""
    text = identity_parm_text("PLANE", sysid=23)
    assert "MAV_SYSID=23" in text
    assert "SYSID_THISMAV" not in text


@pytest.mark.parametrize("vehicle,expected", [("COPTER", 50), ("PLANE", 50), ("VTOL", 100)])
def test_sitl_loop_rate_is_the_lowest_the_firmware_accepts(vehicle, expected):
    """內建 SITL 的 SCHED_LOOP_RATE：能 50 就 50，VTOL 例外用韌體下限 100。

    使用者決定（2026-10-08）：「Hz 數高是 HIL 要做的，SITL 50 夠了」⇒ COPTER / PLANE 維持 50。
    **VTOL 不行**：ArduPlane 4.8 的 QuadPlane 解鎖檢查拒絕 < 100 Hz ——
        Arm: quadplane needs SCHED_LOOP_RATE >= 100
    （2026-10-08 在獨立 SITL 實測：50 Hz 時正常解鎖永遠被擋，只剩 force-arm 這條真機走不通的路。
     先前看起來 50 可行，是因為 ARMING_REQUIRE=0 讓飛機永遠「已解鎖」把這道檢查遮掉了。）
    要改這些值，先回頭問使用者，不要只改這條測試。
    """
    assert _SCHED_LOOP_RATE[vehicle] == expected
    assert f"SCHED_LOOP_RATE={expected}" in identity_parm_text(vehicle, sysid=1)


def test_vtol_loop_rate_meets_the_firmware_minimum():
    """對照組：VTOL 的值要真的 >= 韌體下限 100（寫回 50 就會紅，不是恆真）。"""
    assert _SCHED_LOOP_RATE["VTOL"] >= 100
    assert _SCHED_LOOP_RATE["COPTER"] < 100 and _SCHED_LOOP_RATE["PLANE"] < 100, \
        "COPTER/PLANE 是使用者要的低 Hz；若它們也被抬到 100，請確認使用者同意"
