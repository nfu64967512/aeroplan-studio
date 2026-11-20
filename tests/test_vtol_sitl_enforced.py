"""「只要是 VTOL 的 SITL，Q_ENABLE 一定是 1」的值層級鎖。

背景（2026-08-25，實跑後量到的）
--------------------------------
把 VTOL 偵察頁的任務上傳到 SITL、解鎖、進 AUTO 之後，飛機**停在地上不動**。
逐層量下去：四軸馬達輸出恆為 1000（= PWM 下限），而原廠 `--model quadplane`
開機時韌體會印 ``Q_ENABLE 0`` —— `--model` 只決定**物理模型**，QuadPlane 功能
要靠參數開。Q_ENABLE=0 時四軸馬達根本不存在，任務照跑、畫面一切正常，
飛機就是不會垂直起飛，而且**完全沒有錯誤訊息**。

實測對照（同一顆 binary、同一個 home）：

    只有 Q_ENABLE=1 + frame class/type + SERVO5~8 馬達對應 → 爬到 27 m ✅
    少了那組                                                 → 0.1 m ❌

所以這裡鎖兩件事：
  1. 產生出來的強制參數檔**真的含 Q_ENABLE=1** 與四軸馬達對應；
  2. 它在 ``--defaults`` 裡**排最後** —— 後面覆蓋前面，排最後才保證贏，
     前面任何一份檔（plane.parm / vtol_default.parm）都蓋不掉它。

⚠ 這是「注入」不是「檢查」：檢查只會事後告訴你壞了，注入直接讓它是對的。
"""

import json
from pathlib import Path

import pytest

from mission.sitl_launcher import (
    VTOL_ENFORCED_PARAMS,
    build_defaults_list,
    vtol_enforced_parm_text,
    write_vtol_enforced_parm,
)

_FW_JSON = Path(__file__).resolve().parents[1] / 'sitl' / 'default_params' / 'firmware_param_names.json'


def _parse(text):
    """把 .parm 內容解析成 {名稱: 值}（和韌體讀 --defaults 的方式一致）。"""
    out = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            out[parts[0]] = float(parts[1])
    return out


# ═════════════════════════════════════════════════════════════════════
#  一、強制參數檔的內容
# ═════════════════════════════════════════════════════════════════════
def test_q_enable_is_one():
    """最核心的一條：Q_ENABLE 必須是 1。"""
    assert _parse(vtol_enforced_parm_text()).get('Q_ENABLE') == 1.0


def test_quad_frame_is_declared():
    """只有 Q_ENABLE 而沒有 frame class ⇒ 等於啟用了一台沒定義的四軸。"""
    p = _parse(vtol_enforced_parm_text())
    assert p.get('Q_FRAME_CLASS') == 1.0
    assert p.get('Q_FRAME_TYPE') == 1.0


@pytest.mark.parametrize('ch,func', [(5, 33), (6, 34), (7, 35), (8, 36)])
def test_quad_motor_outputs_are_mapped(ch, func):
    """四軸馬達的 SERVO 功能對應 —— 少了這個，Q_ENABLE=1 也沒有馬達會轉。

    實測：原廠 SITL 沒有這組時，SERVO5~8 輸出是 0（未指派）而不是 1000。
    """
    assert _parse(vtol_enforced_parm_text()).get(f'SERVO{ch}_FUNCTION') == float(func)


def test_arming_require_is_one():
    """⭐ 二分搜尋出來的真兇：plane.parm 的 ARMING_REQUIRE=0 會讓四軸馬達
    永遠停在 PWM 下限，飛機解得了鎖、進得了 AUTO，就是不會離地，且無任何錯誤。

    實測（同一顆 binary、同一個 home）：
        plane.parm + Q_ENABLE 那組        → 0.15 m
        同上再加 ARMING_REQUIRE 1         → 21.4 m
    """
    assert _parse(vtol_enforced_parm_text()).get('ARMING_REQUIRE') == 1.0


def test_plane_parm_still_sets_arming_require_zero():
    """對照組：確認 plane.parm 真的還寫著 0 —— 所以「排最後覆蓋」不是多餘的。

    哪天有人直接改掉 plane.parm，這條會紅，提醒重新確認覆蓋還需不需要。
    """
    plane = Path(__file__).resolve().parents[1] / 'sitl' / 'default_params' / 'plane.parm'
    if not plane.exists():
        pytest.skip('找不到 plane.parm')
    assert _parse(plane.read_text(encoding='utf-8')).get('ARMING_REQUIRE') == 0.0


def test_every_declared_param_reaches_the_file():
    """宣告在 VTOL_ENFORCED_PARAMS 的每一項都要真的寫進檔案。"""
    p = _parse(vtol_enforced_parm_text())
    for name, val, _why in VTOL_ENFORCED_PARAMS:
        assert p.get(name) == float(val), f'{name} 沒寫進去或值不對'


def test_declared_params_are_real_firmware_names():
    """名字必須是韌體真的認得的 —— ArduPilot 對不認識的名字是**靜默略過**。"""
    if not _FW_JSON.exists():
        pytest.skip('沒有韌體參數表快照')
    data = json.loads(_FW_JSON.read_text(encoding='utf-8'))
    known = set(data.get('vtol') or []) | set(data.get('plane') or [])
    if not known:
        pytest.skip('快照裡沒有 vtol/plane 參數名')
    for name, _val, _why in VTOL_ENFORCED_PARAMS:
        assert name in known, f'{name} 不在韌體參數表裡 ⇒ 會被靜默略過'


def test_file_is_written_and_round_trips(tmp_path):
    path = write_vtol_enforced_parm(tmp_path)
    assert path.exists()
    assert _parse(path.read_text(encoding='utf-8')) == _parse(vtol_enforced_parm_text())


# ═════════════════════════════════════════════════════════════════════
#  二、順序：強制檔一定排最後
# ═════════════════════════════════════════════════════════════════════
def test_enforced_parm_is_last_for_vtol():
    """``--defaults`` 後面覆蓋前面 ⇒ 排最後才保證贏。"""
    got = build_defaults_list('VTOL', 'plane.parm', 'vtol.parm', 'id.parm', 'enf.parm')
    assert got[-1] == 'enf.parm', got


@pytest.mark.parametrize('base,vtol,ident', [
    (None, 'vtol.parm', 'id.parm'),
    ('plane.parm', None, 'id.parm'),
    ('plane.parm', 'vtol.parm', None),
    (None, None, None),
])
def test_enforced_stays_last_however_many_files_are_missing(base, vtol, ident):
    """任何一份前置檔缺席，強制檔仍必須是最後一個。"""
    got = build_defaults_list('VTOL', base, vtol, ident, 'enf.parm')
    assert got[-1] == 'enf.parm', got


def test_enforced_parm_appears_exactly_once():
    got = build_defaults_list('VTOL', 'plane.parm', 'vtol.parm', 'id.parm', 'enf.parm')
    assert got.count('enf.parm') == 1


def test_base_params_still_come_first():
    """對照組：順序不是隨便排的，plane.parm 仍要在最前面。"""
    got = build_defaults_list('VTOL', 'plane.parm', 'vtol.parm', 'id.parm', 'enf.parm')
    assert got == ['plane.parm', 'vtol.parm', 'id.parm', 'enf.parm']


# ═════════════════════════════════════════════════════════════════════
#  三、非 VTOL 不該被塞 QuadPlane 參數
# ═════════════════════════════════════════════════════════════════════
@pytest.mark.parametrize('vehicle', ['PLANE', 'COPTER'])
def test_non_vtol_gets_neither_vtol_nor_enforced(vehicle):
    got = build_defaults_list(vehicle, 'plane.parm', 'vtol.parm', 'id.parm', 'enf.parm')
    assert 'enf.parm' not in got
    assert 'vtol.parm' not in got
    assert got == ['plane.parm', 'id.parm']


# ═════════════════════════════════════════════════════════════════════
#  四、對照組：證明這些斷言真的測得到東西
# ═════════════════════════════════════════════════════════════════════
def test_plane_parm_alone_does_not_enable_quadplane():
    """基準：plane.parm 本身沒有 Q_ENABLE —— 所以「強制注入」不是多餘的。

    沒有這條，上面那些斷言可能只是在重述一件本來就成立的事。
    """
    plane = Path(__file__).resolve().parents[1] / 'sitl' / 'default_params' / 'plane.parm'
    if not plane.exists():
        pytest.skip('找不到 plane.parm')
    assert 'Q_ENABLE' not in _parse(plane.read_text(encoding='utf-8'))


def test_every_physical_line_fits_the_firmware_line_buffer():
    """韌體用 100-byte 緩衝讀 --defaults ⇒ 每個實體行（含 CRLF 前）要 <= 98 bytes，否則被切斷。

    2026-10-08 審查：說明那行曾是 112 bytes，被切後的尾巴 `在 1000 µs` 在韌體眼裡是一個合法長度的
    孤兒參數名 ⇒ defaults 反覆重載。_parse 會容忍任何東西，所以要直接量行長。
    """
    lines = vtol_enforced_parm_text().splitlines()
    assert lines, "強制檔是空的"
    longest = max(len(ln.encode("utf-8")) for ln in lines)
    assert longest <= 98, f"有實體行 {longest} bytes，超過韌體 100-byte 行緩衝：" + \
        next(ln for ln in lines if len(ln.encode("utf-8")) == longest)


def test_non_comment_lines_are_strictly_name_space_number():
    """非註解行只能是 `NAME 數字`：任何多餘的詞都可能被韌體當成別的參數。"""
    import re
    for ln in vtol_enforced_parm_text().splitlines():
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        assert re.fullmatch(r"[A-Z][A-Z0-9_]* -?\d+(\.\d+)?", ln), f"格式不嚴格：{ln!r}"


def test_parse_helper_actually_reads_values():
    """對照組：解析器本身要真的會讀值，否則上面全部恆真。"""
    assert _parse('Q_ENABLE 1\n# 註解\nFOO 2.5  # 尾註') == {'Q_ENABLE': 1.0, 'FOO': 2.5}
    assert _parse('# 全部都是註解\n') == {}
