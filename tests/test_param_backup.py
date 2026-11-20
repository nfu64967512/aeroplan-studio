"""``mission/param_backup.py`` 的值層級守衛。

背景（2026-09-14 查證）：AeroPlan 既有的 GUI 參數匯出／匯入是**結構性死的** ——
``ParamService`` 只在 ``link.param_value`` 訊號上收資料，而 ``SITLLink`` 沒有那個訊號，
``hasattr`` 檢查失敗後降級成一行 log。於是 ``ParametersBrowser._params`` 恆為空，
``_on_export`` 第一行 ``if not self._params: return`` **靜默返回**。
按鈕按下去什麼都不會發生，也不會報錯。

本檔鎖住取而代之的無頭實作的**行為**，三件事：完整、精確、認得出是哪一台。
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mission.param_backup import (                                  # noqa: E402
    VOLATILE_PARAMS,
    BoardId,
    IncompleteDump,
    Snapshot,
    as_f32,
    backup_before_write,
    board_id_for_port,
    diff_params,
    fetch_all,
    fmt_param,
    parse_param_file,
    render_param_file,
    restore,
    save_snapshot,
    write_params_with_backup,
)


# ---------------------------------------------------------------------------
# 假飛控連線
# ---------------------------------------------------------------------------
def _msg(name: str, value: float, index: int, count: int, ptype: int = 9):
    return SimpleNamespace(
        param_id=name.encode("ascii"), param_value=float(value),
        param_type=int(ptype), param_index=int(index), param_count=int(count),
    )


class _FakeConn:
    """最小的 pymavlink 連線替身。

    ``drop`` 的 index 不會出現在串流裡（模擬掉包）；``deaf`` 的 index 連補抓
    也不回應。``blind_index`` 讓補抓回應帶 65535（ArduPilot 以名稱回覆時的行為）。
    """

    target_system = 1
    target_component = 1

    def __init__(self, params: dict[str, float], *, drop: set[int] = frozenset(),
                 deaf: set[int] = frozenset(), blind_index: bool = False,
                 declared: int | None = None) -> None:
        self._names = sorted(params)
        self._params = params
        self._count = len(self._names) if declared is None else declared
        self._drop, self._deaf, self._blind = set(drop), set(deaf), blind_index
        self._queue: list = []
        self.list_requests = 0
        self.read_requests: list[int] = []
        self.mav = SimpleNamespace(
            param_request_list_send=self._list_send,
            param_request_read_send=self._read_send,
        )

    def _list_send(self, _ts, _tc) -> None:
        self.list_requests += 1
        for i, n in enumerate(self._names):
            if i in self._drop:
                continue
            self._queue.append(_msg(n, self._params[n], i, self._count))

    def _read_send(self, _ts, _tc, _name, index) -> None:
        idx = int(index)
        self.read_requests.append(idx)
        if idx in self._deaf or not (0 <= idx < len(self._names)):
            return
        n = self._names[idx]
        self._queue.append(
            _msg(n, self._params[n], 65535 if self._blind else idx, self._count)
        )

    def recv_match(self, type=None, blocking=False, timeout=None):   # noqa: A002
        return self._queue.pop(0) if self._queue else None


_REAL = {
    "ARMING_CHECK": 1.0,
    "COMPASS_OFS_X": 4e-07,                 # %.6f 會變成 0
    "INS_ACCOFFS_Y": 3.0517578125e-05,      # %.6f 會變成 0.000031（誤差 1.6%）
    "ATC_RAT_RLL_D": 0.0036,
    "Q_RTL_MODE": 0.0,
    "WP_RADIUS": 90.0,
    "AHRS_TRIM_X": -0.00123456,
}


# ---------------------------------------------------------------------------
# 精確：float32 往返
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("value", [
    0.0, 1.0, -1.0, 90.0, 1500.0, 0.0036, 0.283333343,
    4e-07, 1.2345678e-07, 3.0517578125e-05, -0.00123456,
    1234567.5, 3.4028235e38, 1.1754944e-38,
])
def test_formatting_round_trips_through_float32(value: float) -> None:
    assert as_f32(float(fmt_param(value))) == as_f32(value)


def test_six_decimal_formatting_would_destroy_real_parameters() -> None:
    """⚠ 地雷偵測器：有人把格式「簡化」回 ``%.6f`` 時這條會爆。

    這不是假設 —— ``hv_dccpp.mavlink_link._fmt`` 現在就是 ``%.6f``。
    """
    ruined = {
        name: v for name, v in _REAL.items()
        if as_f32(float(f"{as_f32(v):.6f}")) != as_f32(v)
    }
    assert "COMPASS_OFS_X" in ruined, "4e-07 應該被 %.6f 壓成 0"
    assert "INS_ACCOFFS_Y" in ruined
    for name in ruined:
        assert as_f32(float(fmt_param(_REAL[name]))) == as_f32(_REAL[name]), name


def test_integers_stay_integers() -> None:
    """``ARMING_CHECK,1`` 比 ``ARMING_CHECK,1.0`` 更像 ArduPilot 的檔案。"""
    assert fmt_param(1.0) == "1"
    assert fmt_param(-4.0) == "-4"
    assert fmt_param(0.0) == "0"


def test_non_finite_is_refused() -> None:
    for bad in (float("nan"), float("inf")):
        with pytest.raises(ValueError):
            fmt_param(bad)


# ---------------------------------------------------------------------------
# 決定性與往返
# ---------------------------------------------------------------------------
def _snap(params=None) -> Snapshot:
    return Snapshot(board=BoardId("25003A000E51323039383833", 1),
                    params=dict(_REAL if params is None else params),
                    taken_utc="2026-09-14T00:00:00Z")


def test_rendering_is_deterministic_regardless_of_insert_order() -> None:
    a = _snap()
    b = _snap(dict(reversed(list(_REAL.items()))))
    assert render_param_file(a) == render_param_file(b)


def test_render_parse_round_trip_is_exact() -> None:
    got, meta = parse_param_file(render_param_file(_snap()))
    assert {k: as_f32(v) for k, v in got.items()} == {k: as_f32(v) for k, v in _REAL.items()}
    assert meta["board_serial"] == "25003A000E51323039383833"
    assert meta["sysid"] == "1"


def test_content_hash_ignores_the_timestamp() -> None:
    """去重要看內容；時間戳不同不代表參數不同。"""
    a, b = _snap(), _snap()
    b.taken_utc = "2026-12-25T12:34:56Z"
    assert a.content_hash() == b.content_hash()
    c = _snap({**_REAL, "WP_RADIUS": 91.0})
    assert c.content_hash() != a.content_hash()


@pytest.mark.parametrize("line", [
    "WP_RADIUS,90", "WP_RADIUS\t90", "WP_RADIUS 90", "WP_RADIUS=90",
    "WP_RADIUS , 90", "wp_radius,90",
])
def test_parser_accepts_the_formats_gcs_tools_emit(line: str) -> None:
    params, _ = parse_param_file(line)
    assert params == {"WP_RADIUS": 90.0}


def test_parser_skips_comments_and_junk() -> None:
    params, meta = parse_param_file(
        "# AeroPlan\n# sysid: 7\n\nWP_RADIUS,90\nGARBAGE\n#WP_RADIUS,1\n"
    )
    assert params == {"WP_RADIUS": 90.0}
    assert meta["sysid"] == "7"


# ---------------------------------------------------------------------------
# 完整：抓取與補抓
# ---------------------------------------------------------------------------
def test_clean_stream_is_captured_whole() -> None:
    conn = _FakeConn(dict(_REAL))
    params, types = fetch_all(conn)
    assert {k: as_f32(v) for k, v in params.items()} == {k: as_f32(v) for k, v in _REAL.items()}
    assert set(types) == set(_REAL)
    assert conn.read_requests == [], "沒掉包就不該有補抓"


def test_dropped_indices_are_recovered_one_by_one() -> None:
    """⚠ 串流一定會掉包。沒有補抓的話會產生一份**看起來正常**的半份備份。"""
    conn = _FakeConn(dict(_REAL), drop={1, 4})
    params, _ = fetch_all(conn)
    assert set(params) == set(_REAL), "補抓沒把缺的補回來"
    assert sorted(conn.read_requests) == [1, 4]


def test_recovery_accepts_the_65535_index_ardupilot_sends() -> None:
    """以名稱回應 PARAM_REQUEST_READ 時 ArduPilot 回 65535；不能因此判成沒收到。"""
    conn = _FakeConn(dict(_REAL), drop={2}, blind_index=True)
    params, _ = fetch_all(conn)
    assert set(params) == set(_REAL)


def test_unrecoverable_gap_raises_instead_of_returning_a_partial_dump() -> None:
    """⚠⚠ 這是本檔最重要的一條：半份備份比沒有備份更危險。"""
    conn = _FakeConn(dict(_REAL), drop={3}, deaf={3})
    with pytest.raises(IncompleteDump) as ei:
        fetch_all(conn, recover_rounds=2)
    assert 3 in ei.value.missing
    assert ei.value.expected == len(_REAL)


def test_silent_autopilot_raises() -> None:
    conn = _FakeConn({}, declared=0)
    with pytest.raises(IncompleteDump):
        fetch_all(conn)


def test_a_shortened_stream_is_not_mistaken_for_completion() -> None:
    """飛控宣告 7 個卻只送 5 個就停 ⇒ 必須補抓，補不到就拋。"""
    conn = _FakeConn(dict(_REAL), drop={5, 6}, deaf={5, 6})
    with pytest.raises(IncompleteDump) as ei:
        fetch_all(conn, recover_rounds=1)
    assert ei.value.missing == [5, 6]


# ---------------------------------------------------------------------------
# 身分：序號而非 COM 編號
# ---------------------------------------------------------------------------
def test_identity_is_keyed_on_serial_not_port_number() -> None:
    """六顆同型板在 Windows 全叫 COMxx 而**編號會在重插後換**。

    同一顆板子換了埠仍須指向同一個目錄；兩顆不同板子佔到對方的埠不可混用。
    """
    a_before = BoardId("SERIAL_AAA", 1)
    a_after = BoardId("SERIAL_AAA", 1)       # 重插後 COM26 → COM31
    b = BoardId("SERIAL_BBB", 1)
    assert a_before.key() == a_after.key()
    assert a_before.key() != b.key()
    assert "SERIAL_AAA" in a_before.key()


def test_sysid_separates_boards_that_share_a_serial() -> None:
    assert BoardId("S", 1).key() != BoardId("S", 2).key()


def test_key_is_filesystem_safe() -> None:
    k = BoardId("udp:0.0.0.0:14550", 3).key()
    assert not set(k) & set(r'\/:*?"<>| ')


# ---------------------------------------------------------------------------
# 順序：先備份，再寫入
# ---------------------------------------------------------------------------
def test_backup_exists_on_disk_before_any_parameter_is_written(tmp_path: Path) -> None:
    """⚠⚠ 這是 ``write_params_with_backup`` 存在的全部理由。"""
    conn = _FakeConn(dict(_REAL))
    board = BoardId("SERIAL_AAA", 1)
    seen: dict[str, object] = {}

    def setter(params):
        d = tmp_path / board.key()
        files = list(d.glob("*.parm")) if d.is_dir() else []
        seen["files_at_write_time"] = files
        seen["content"] = files[0].read_text(encoding="utf-8") if files else ""
        return {k: True for k in params}

    path, _ = write_params_with_backup(
        conn, {"WP_RADIUS": 200.0}, board=board, root=tmp_path, setter=setter,
    )
    assert seen["files_at_write_time"], "setter 被呼叫時備份還不在磁碟上"
    assert path.exists()
    assert "WP_RADIUS,90" in str(seen["content"]), "備份裡存的必須是**寫入前**的值"


def test_a_failed_backup_blocks_the_write_entirely(tmp_path: Path) -> None:
    """寧可不寫，也不要在沒有退路的情況下蓋掉別人的設定。"""
    conn = _FakeConn(dict(_REAL), drop={0}, deaf={0})
    called = []
    with pytest.raises(IncompleteDump):
        write_params_with_backup(
            conn, {"WP_RADIUS": 200.0},
            board=BoardId("SERIAL_AAA", 1), root=tmp_path,
            setter=lambda p: called.append(p) or {}, recover_rounds=1,
        )
    assert called == [], "備份失敗了卻還是寫了參數"


def test_identical_snapshots_are_not_duplicated(tmp_path: Path) -> None:
    """⚠ 兩次的 ``taken_utc`` **必須不同**，否則這條是空洞的。

    檔名是「時間戳_內容雜湊」。同一秒內的兩次呼叫檔名相同，第二次會直接覆寫，
    於是「只有一個檔案」靠的是檔名碰撞而不是去重分支 —— 把去重整段拿掉也會過。
    （2026-09-14 突變實驗抓到的，原版就是這個樣子。）
    """
    board = BoardId("SERIAL_AAA", 1)
    a = Snapshot(board=board, params=dict(_REAL), taken_utc="2026-09-14T00:00:00Z")
    b = Snapshot(board=board, params=dict(_REAL), taken_utc="2026-09-14T09:30:00Z")
    first = save_snapshot(tmp_path, a)
    second = save_snapshot(tmp_path, b)
    assert second == first, "內容相同卻沒沿用既有檔案"
    assert len(list((tmp_path / board.key()).glob("*.parm"))) == 1


def test_repeated_backups_of_an_unchanged_board_reuse_one_file(tmp_path: Path) -> None:
    board = BoardId("SERIAL_AAA", 1)
    first = backup_before_write(_FakeConn(dict(_REAL)), board, tmp_path)
    second = backup_before_write(_FakeConn(dict(_REAL)), board, tmp_path)
    assert first == second
    assert len(list((tmp_path / board.key()).glob("*.parm"))) == 1


def test_a_changed_board_produces_a_second_snapshot(tmp_path: Path) -> None:
    board = BoardId("SERIAL_AAA", 1)
    backup_before_write(_FakeConn(dict(_REAL)), board, tmp_path)
    backup_before_write(_FakeConn({**_REAL, "WP_RADIUS": 200.0}), board, tmp_path)
    assert len(list((tmp_path / board.key()).glob("*.parm"))) == 2


# ---------------------------------------------------------------------------
# 比對與還原
# ---------------------------------------------------------------------------
def test_diff_reports_changed_added_and_removed() -> None:
    d = diff_params({"A": 1.0, "B": 2.0, "C": 3.0}, {"A": 1.0, "B": 9.0, "D": 4.0})
    assert d == {"B": (2.0, 9.0), "C": (3.0, None), "D": (None, 4.0)}


def test_diff_compares_on_the_float32_grid() -> None:
    """0.1 的 double 與 float32 不同；拿 double 比會把「相同」判成「不同」。"""
    assert diff_params({"A": 0.1}, {"A": as_f32(0.1)}) == {}


def test_restore_writes_only_what_actually_differs() -> None:
    board = BoardId("SERIAL_AAA", 1)
    on_board = {**_REAL, "WP_RADIUS": 200.0, "ARMING_CHECK": 0.0}
    conn = _FakeConn(on_board)
    wrote: dict[str, float] = {}

    def setter(params):
        wrote.update(params)
        return {k: True for k in params}

    rep = restore(conn, Snapshot(board=board, params=dict(_REAL)), setter=setter)
    assert set(wrote) == {"WP_RADIUS", "ARMING_CHECK"}
    assert wrote["WP_RADIUS"] == 90.0
    assert "ATC_RAT_RLL_D" in rep.unchanged
    assert rep.ok


def test_restore_refuses_parameters_the_board_does_not_have() -> None:
    """不同韌體／機型的備份硬寫回去會製造垃圾參數。"""
    on_board = {k: v for k, v in _REAL.items() if k != "Q_RTL_MODE"}
    conn = _FakeConn(on_board)
    wrote: dict[str, float] = {}
    rep = restore(conn, Snapshot(board=BoardId("S", 1), params=dict(_REAL)),
                  setter=lambda p: (wrote.update(p), {k: True for k in p})[1])
    assert rep.unknown == ["Q_RTL_MODE"]
    assert "Q_RTL_MODE" not in wrote


def test_restore_records_a_rejected_write_as_failed() -> None:
    conn = _FakeConn({**_REAL, "WP_RADIUS": 200.0})
    rep = restore(conn, Snapshot(board=BoardId("S", 1), params=dict(_REAL)),
                  setter=lambda p: {k: False for k in p})
    assert rep.failed and not rep.ok
    assert "WP_RADIUS" in rep.failed


def test_dry_run_writes_nothing_but_still_reports() -> None:
    conn = _FakeConn({**_REAL, "WP_RADIUS": 200.0})
    called = []
    rep = restore(conn, Snapshot(board=BoardId("S", 1), params=dict(_REAL)),
                  setter=lambda p: called.append(p) or {}, dry_run=True)
    assert called == []
    assert "WP_RADIUS" in rep.written


def test_saved_snapshot_reloads_identically(tmp_path: Path) -> None:
    from mission.param_backup import load_snapshot
    path = save_snapshot(tmp_path, _snap())
    back = load_snapshot(path)
    assert {k: as_f32(v) for k, v in back.params.items()} == \
           {k: as_f32(v) for k, v in _REAL.items()}
    assert back.board.serial == "25003A000E51323039383833"
    assert back.board.sysid == 1


# ---------------------------------------------------------------------------
# 飛控自己會改的參數
# ---------------------------------------------------------------------------
# ⚠⚠ 實測（2026-09-14，ArduPlane SITL quadplane，1620 個參數、8 次取樣、
# 跨約 367 秒模擬時間，scratchpad/param_volatility_probe.py）：六個參數在**沒有人
# 下 PARAM_SET** 的情況下自己變了 —— ARSPD_OFFSET（開機空速歸零 0→2014.45）、
# BARO1/2_GND_PRESS、COMPASS_DEC（由 GPS 自動設磁偏角）、MIS_TOTAL（任務航點數）、
# STAT_RUNTIME（單調遞增，永不穩定）。

def test_the_volatile_set_carries_what_was_actually_measured() -> None:
    for name in ("ARSPD_OFFSET", "BARO1_GND_PRESS", "BARO2_GND_PRESS",
                 "COMPASS_DEC", "MIS_TOTAL", "STAT_RUNTIME"):
        assert name in VOLATILE_PARAMS, f"{name} 是實測會自己變的，不可從集合移除"


def test_dedupe_survives_a_monotonic_counter(tmp_path: Path) -> None:
    """⚠⚠ 若雜湊涵蓋 STAT_RUNTIME，真板子上**任兩份快照都不同** ⇒ 去重永不命中。

    這一條用假連線模擬「只有 STAT_RUNTIME 在走」的真實情形。
    """
    board = BoardId("SERIAL_AAA", 1)
    base = {**_REAL, "STAT_RUNTIME": 30.0}
    a = Snapshot(board=board, params=dict(base), taken_utc="2026-09-14T00:00:00Z")
    b = Snapshot(board=board, params={**base, "STAT_RUNTIME": 278.0},
                 taken_utc="2026-09-14T00:05:00Z")
    assert a.content_hash() == b.content_hash(), "計數器讓去重失效"
    assert save_snapshot(tmp_path, b) == save_snapshot(tmp_path, a)
    assert len(list((tmp_path / board.key()).glob("*.parm"))) == 1


def test_a_real_configuration_change_still_produces_a_new_snapshot(tmp_path: Path) -> None:
    """⚠ 非空洞性：排除易變參數不可以連真的設定變更一起吃掉。"""
    board = BoardId("SERIAL_AAA", 1)
    a = Snapshot(board=board, params={**_REAL, "STAT_RUNTIME": 30.0},
                 taken_utc="2026-09-14T00:00:00Z")
    b = Snapshot(board=board, params={**_REAL, "STAT_RUNTIME": 30.0, "WP_RADIUS": 200.0},
                 taken_utc="2026-09-14T00:05:00Z")
    assert a.content_hash() != b.content_hash()
    save_snapshot(tmp_path, a)
    save_snapshot(tmp_path, b)
    assert len(list((tmp_path / board.key()).glob("*.parm"))) == 2


def test_the_backup_file_still_contains_the_volatile_values(tmp_path: Path) -> None:
    """排除只發生在「算不算同一份設定」的判斷上；**存進去的必須是完整的**。"""
    snap = Snapshot(board=BoardId("SERIAL_AAA", 1),
                    params={**_REAL, "STAT_RUNTIME": 278.0, "MIS_TOTAL": 41.0},
                    taken_utc="2026-09-14T00:00:00Z")
    got, _ = parse_param_file(save_snapshot(tmp_path, snap).read_text(encoding="utf-8"))
    assert got["STAT_RUNTIME"] == 278.0
    assert got["MIS_TOTAL"] == 41.0


def test_restore_does_not_rewrite_the_mission_item_count() -> None:
    """⚠⚠ ``MIS_TOTAL`` 是**任務航點數**；把舊值寫回去會動到飛控裡存的任務。"""
    on_board = {**_REAL, "MIS_TOTAL": 41.0, "STAT_RUNTIME": 999.0, "WP_RADIUS": 200.0}
    conn = _FakeConn(on_board)
    wrote: dict[str, float] = {}
    snap = Snapshot(board=BoardId("S", 1),
                    params={**_REAL, "MIS_TOTAL": 0.0, "STAT_RUNTIME": 30.0})
    rep = restore(conn, snap,
                  setter=lambda p: (wrote.update(p), {k: True for k in p})[1])
    assert "MIS_TOTAL" not in wrote, "還原竄改了任務航點數"
    assert "STAT_RUNTIME" not in wrote
    assert sorted(rep.volatile) == ["MIS_TOTAL", "STAT_RUNTIME"]
    assert set(wrote) == {"WP_RADIUS"}, "該寫的設定沒寫"


def test_volatile_can_be_written_when_explicitly_asked() -> None:
    """⚠ 非空洞性：證明那條路真的走得到，不是恆為空。"""
    conn = _FakeConn({**_REAL, "MIS_TOTAL": 41.0})
    wrote: dict[str, float] = {}
    rep = restore(conn, Snapshot(board=BoardId("S", 1),
                                 params={**_REAL, "MIS_TOTAL": 0.0}),
                  setter=lambda p: (wrote.update(p), {k: True for k in p})[1],
                  include_volatile=True)
    assert wrote == {"MIS_TOTAL": 0.0}
    assert rep.volatile == []


def test_diff_shows_volatile_by_default_and_hides_it_on_request() -> None:
    """問「有什麼不一樣」要看見全部；問「設定有沒有被改動」才過濾。"""
    a = {"WP_RADIUS": 90.0, "STAT_RUNTIME": 30.0}
    b = {"WP_RADIUS": 90.0, "STAT_RUNTIME": 278.0}
    assert "STAT_RUNTIME" in diff_params(a, b)
    assert diff_params(a, b, ignore=VOLATILE_PARAMS) == {}


# ---------------------------------------------------------------------------
# 由連線字串推識別
# ---------------------------------------------------------------------------
def _fake_ports(monkeypatch: pytest.MonkeyPatch, rows) -> None:
    class _P:
        def __init__(self, device, serial_number):
            self.device, self.serial_number = device, serial_number

    fake = types.SimpleNamespace(comports=lambda: [_P(d, s) for d, s in rows])
    monkeypatch.setitem(sys.modules, "serial.tools.list_ports", fake)
    monkeypatch.setitem(sys.modules, "serial.tools",
                        types.SimpleNamespace(list_ports=fake))


def test_serial_port_resolves_to_its_usb_serial(monkeypatch: pytest.MonkeyPatch) -> None:
    """實測資料：同一顆 Cube Orange+ 的 MAVLink 埠與 SLCAN 埠共用一個序號。"""
    _fake_ports(monkeypatch, [("COM26", "25003A000E51323039383833"),
                              ("COM27", "25003A000E51323039383833")])
    a = board_id_for_port("COM26", 1)
    b = board_id_for_port("COM27", 1)
    assert a.serial == "25003A000E51323039383833"
    assert a.key() == b.key(), "同一顆板子的兩個埠必須指向同一份備份"
    assert "COM26" not in a.key(), "識別不可以含 COM 編號 —— 重插會換"


def test_a_port_without_a_serial_is_marked_not_silently_accepted(
        monkeypatch: pytest.MonkeyPatch) -> None:
    """⚠ 查不到序號時識別是**弱的**，使用者必須看得出來。

    靜默退回埠名會讓「重插後換了編號」與「換了一顆板子」長得一模一樣。
    """
    _fake_ports(monkeypatch, [("COM3", None)])
    got = board_id_for_port("COM3", 1)
    assert got.serial.startswith("noserial-"), f"弱識別沒有標示：{got.serial!r}"
    assert "COM3" in got.serial


def test_an_unknown_port_does_not_borrow_another_boards_serial(
        monkeypatch: pytest.MonkeyPatch) -> None:
    """⚠ 非空洞性：只有**名稱相符**那一列的序號才算數。"""
    _fake_ports(monkeypatch, [("COM26", "25003A000E51323039383833")])
    got = board_id_for_port("COM99", 1)
    assert got.serial == "noserial-COM99"


@pytest.mark.parametrize("endpoint", ["tcp:127.0.0.1:5760", "udpin:0.0.0.0:14550",
                                      "udp:127.0.0.1:14560"])
def test_network_endpoints_keep_their_own_identity(
        monkeypatch: pytest.MonkeyPatch, endpoint: str) -> None:
    _fake_ports(monkeypatch, [("COM26", "AAA")])
    assert board_id_for_port(endpoint, 1).serial == endpoint


def test_a_broken_pyserial_does_not_crash_the_backup(
        monkeypatch: pytest.MonkeyPatch) -> None:
    """列舉失敗只該讓識別變弱，不該讓備份整個做不成。"""
    def _boom():
        raise OSError("列舉失敗")
    fake = types.SimpleNamespace(comports=_boom)
    monkeypatch.setitem(sys.modules, "serial.tools.list_ports", fake)
    monkeypatch.setitem(sys.modules, "serial.tools",
                        types.SimpleNamespace(list_ports=fake))
    assert board_id_for_port("COM26", 1).serial == "noserial-COM26"
