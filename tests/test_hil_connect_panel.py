"""HIL 連線面板的值層級守衛（埠識別 + 連線失敗診斷）。

⚠⚠⚠ 2026-09-14 實際事件。一顆 Cube Orange+ 接上 micro-USB，Windows 正常列舉出
``COM26 Cube Orange+ Mavlink``，但 GCS 連不上。舊版面板對此只會說
「連線失敗（0/1）— 檢查埠名 / 鮑率 / 線材」——那是**四種完全不同處置**的集合名詞。

實際原因用原始序列埠讀 8 秒就找到了：埠開得起來、有 510 bytes 進來，
但 ``0xFD``／``0xFE`` **都是 0**，內容是 ASCII ``"Unable to init RAMTRON storage"``
以 2 Hz 重複 —— 板子卡在參數儲存初始化，根本沒跑到 MAVLink 階段。
埠名、鮑率、線材**全部都是對的**，照舊訊息去查會查錯方向。

兩個修正，這一檔鎖住它們的**行為**：

1. :func:`_scan_serial_ports` 回傳帶描述與 USB 序號的項目。
   六顆同型飛控在 Windows 上全叫 ``COMxx`` 而**編號會在重插後換**；
   只看編號認飛機，接錯沒有任何徵狀。實測同一顆 Cube 的兩個埠共用同一序號。
2. :func:`probe_serial_port` 把「連不上」分成 busy / silent / not_mavlink / ok，
   並把實際收到的位元組樣本帶進訊息。
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

pytest.importorskip("PyQt6")
panel = pytest.importorskip("ui.widgets.hil_connect_panel")

_port_of = panel._port_of
probe_serial_port = panel.probe_serial_port


# ---------------------------------------------------------------------------
# 埠識別
# ---------------------------------------------------------------------------

def test_port_is_extracted_from_a_labelled_entry() -> None:
    label = f"COM26{panel._PORT_SEP}Cube Orange+ Mavlink"
    assert _port_of(label) == "COM26"


@pytest.mark.parametrize("raw", ["udp:0.0.0.0:14550", "tcp:127.0.0.1:5760",
                                 "udpin:0.0.0.0:14551", "COM26", "/dev/ttyACM0"])
def test_hand_typed_endpoints_survive_unchanged(raw: str) -> None:
    """combo 是可編輯的，使用者會直接手打端點 —— 切壞它就連不上而且看不出為什麼。"""
    assert _port_of(raw) == raw


def test_scan_carries_description_and_serial(monkeypatch: pytest.MonkeyPatch) -> None:
    """實測資料：同一顆 Cube 列舉出兩個埠、共用同一個序號，其中 SLCAN 不是 MAVLink。"""
    class _P:
        def __init__(self, device, description, serial_number):
            self.device, self.description = device, description
            self.serial_number = serial_number

    fake = types.SimpleNamespace(comports=lambda: [
        _P("COM26", "Cube Orange+ Mavlink (COM26)", "25003A000E51323039383833"),
        _P("COM27", "Cube Orange+ SLCAN (COM27)", "25003A000E51323039383833"),
    ])
    monkeypatch.setitem(sys.modules, "serial.tools.list_ports", fake)
    monkeypatch.setitem(sys.modules, "serial.tools",
                        types.SimpleNamespace(list_ports=fake))

    got = panel._scan_serial_ports()
    assert [d for d, _l, _s in got] == ["COM26", "COM27"]
    for dev, label, sn in got:
        assert label.startswith(dev)
        assert "Cube Orange+" in label, "描述沒帶進來 ⇒ 六顆同型板無法分辨"
        assert f"({dev})" not in label, "pyserial 併進 description 的 (COMxx) 沒去掉"
        assert sn == "25003A000E51323039383833"
    assert _port_of(got[0][1]) == "COM26"


# ---------------------------------------------------------------------------
# 連線失敗診斷
# ---------------------------------------------------------------------------

def _fake_serial(monkeypatch: pytest.MonkeyPatch, payload: bytes | None = b"",
                 raise_on_open: Exception | None = None) -> None:
    class _S:
        def __init__(self, *a, **kw):
            if raise_on_open is not None:
                raise raise_on_open
            self._sent = False

        def read(self, _n):
            if self._sent:
                return b""
            self._sent = True
            return payload or b""

        def close(self):
            pass

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=_S))


def test_busy_port_is_reported_as_busy(monkeypatch: pytest.MonkeyPatch) -> None:
    _fake_serial(monkeypatch, raise_on_open=PermissionError("拒絕存取。"))
    verdict, msg = probe_serial_port("COM26", seconds=0.5)
    assert verdict == "busy"
    assert "COM26" in msg


def test_silent_port_is_distinguished_from_garbage(monkeypatch: pytest.MonkeyPatch) -> None:
    _fake_serial(monkeypatch, payload=b"")
    verdict, _msg = probe_serial_port("COM26", seconds=0.5)
    assert verdict == "silent"


def test_console_text_is_reported_as_not_mavlink(monkeypatch: pytest.MonkeyPatch) -> None:
    """實際遇到的形態：有資料、零個 MAVLink 框頭、內容是主控台錯誤字串。"""
    _fake_serial(monkeypatch, payload=b"Unable to init RAMTRON storage" * 4)
    verdict, msg = probe_serial_port("COM26", seconds=0.5)
    assert verdict == "not_mavlink"
    assert "RAMTRON" in msg, "沒把實際收到的內容帶出來 ⇒ 使用者還是不知道發生什麼事"
    assert "0xFD" in msg or "MAVLink" in msg


@pytest.mark.parametrize("magic", [b"\xfd", b"\xfe"])
def test_mavlink_framing_is_recognised(monkeypatch: pytest.MonkeyPatch, magic: bytes) -> None:
    """⚠ 非空洞性：若 probe 永遠回 not_mavlink，上面那條也會通過。

    這一條證明 'ok' 這個結果**真的產生得出來**，v1 與 v2 兩種框頭都要認。
    """
    _fake_serial(monkeypatch, payload=magic + b"\x09\x00" + bytes(20))
    verdict, msg = probe_serial_port("COM26", seconds=0.5)
    assert verdict == "ok", f"{magic!r} 框頭沒被認出來"
    assert "MAVLink" in msg


def test_the_four_verdicts_are_mutually_exclusive(monkeypatch: pytest.MonkeyPatch) -> None:
    """同一支函式對四種輸入必須給四個**不同**的答案，否則診斷沒有鑑別力。"""
    seen = set()
    _fake_serial(monkeypatch, raise_on_open=PermissionError("x"))
    seen.add(probe_serial_port("COM26", seconds=0.5)[0])
    _fake_serial(monkeypatch, payload=b"")
    seen.add(probe_serial_port("COM26", seconds=0.5)[0])
    _fake_serial(monkeypatch, payload=b"Unable to init RAMTRON storage")
    seen.add(probe_serial_port("COM26", seconds=0.5)[0])
    _fake_serial(monkeypatch, payload=b"\xfd" + bytes(20))
    seen.add(probe_serial_port("COM26", seconds=0.5)[0])
    assert seen == {"busy", "silent", "not_mavlink", "ok"}, f"實得 {seen}"
