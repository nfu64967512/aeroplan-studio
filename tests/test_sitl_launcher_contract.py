"""`SITLLauncher.start_multi` 的回傳形狀契約。

⚠⚠⚠ 2026-09-09 實際發生：GUI 按「啟動 SITL」在四台**都已經成功起來之後**炸掉，
訊息是 ``not enough values to unpack (expected 3, got 2)``，然後把四台全部終止。

根因是兩條管線對不上，而且**在 HEAD 就已經對不上**：

* `ui/main_window.py` 的 ``for logical, conn, actual in results:`` 要三個值
  （port 被佔用而位移時，邏輯序號 ≠ 實際 instance，診斷訊息要印出來）
* `mission/sitl_launcher.py` 的 ``results.append((i, conn))`` 只給兩個

當時 `wslrelay` 佔住 5760–5790，四台被自動位移到 i4–i7 —— 也就是說**正是位移功能
要處理的那個情境觸發了這個 bug**。而且沒有任何測試鎖住這個介面。

本檔就是那道鎖。它不啟動任何 SITL，只驗**形狀與語意**。
"""

from __future__ import annotations

import inspect

import pytest

from mission.sitl_launcher import SITLLauncher

CONSUMERS = ("ui/main_window.py", "cli/demo_cmd.py", "cli/sitl_cmd.py")


class _FakeLauncher(SITLLauncher):
    """把 `start()` 換成不真的開行程的替身，其餘流程照走。"""

    def __init__(self, shift_map=None):
        super().__init__()
        self._shift = dict(shift_map or {})
        self.calls: list[int] = []

    def start(self, *, instance=0, **kw):                    # type: ignore[override]
        self.calls.append(instance)
        actual = self._shift.get(instance, instance)
        self._last_started_instance = actual
        return f"tcp:127.0.0.1:{5760 + 10 * actual}"


def test_start_multi_returns_triples():
    """⭐ 契約本身：每一列都是 (邏輯序號, 連線字串, 實際 instance)。"""
    rows = _FakeLauncher().start_multi("VTOL", 3, lat=23.95, lon=120.90)
    assert len(rows) == 3
    for i, row in enumerate(rows):
        assert len(row) == 3, f"第 {i} 列不是三元組：{row}"
        logical, conn, actual = row
        assert logical == i
        assert isinstance(conn, str) and conn.startswith("tcp:")
        assert isinstance(actual, int)


def test_actual_equals_logical_when_no_port_shift():
    """前置斷言：沒有 port 衝突時兩者相同 —— 否則下一條測不到位移。"""
    rows = _FakeLauncher().start_multi("VTOL", 4, lat=23.95, lon=120.90)
    assert [(lg, ac) for lg, _c, ac in rows] == [(0, 0), (1, 1), (2, 2), (3, 3)]


def test_actual_reports_the_shifted_instance():
    """⭐⭐ 重現 2026-09-09 的現場：wslrelay 佔住 5760–5790 ⇒ i0–i3 被推到 i4–i7。

    邏輯序號必須維持 0–3（sysid 標籤靠它），實際 instance 必須報 4–7（診斷靠它）。
    """
    lz = _FakeLauncher({0: 4, 1: 5, 2: 6, 3: 7})
    rows = lz.start_multi("VTOL", 4, lat=23.95, lon=120.90)
    assert [lg for lg, _c, _a in rows] == [0, 1, 2, 3]
    assert [ac for _l, _c, ac in rows] == [4, 5, 6, 7]
    assert [c for _l, c, _a in rows] == [f"tcp:127.0.0.1:{5760 + 10 * n}" for n in (4, 5, 6, 7)]


def test_every_consumer_unpacks_three_values():
    """⚠⚠ 這條鎖的是「**所有**消費端都跟得上」，不是只鎖 launcher 那一側。

    先前的缺陷正是單邊改動：UI 改成要三個值，launcher 沒跟上，而且沒人擋。
    """
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent
    bad = []
    for rel in CONSUMERS:
        src = (root / rel).read_text(encoding="utf-8")
        if "start_multi" not in src:
            bad.append(f"{rel} 沒有呼叫 start_multi —— 這條測試的前提失效了")
            continue
        for line in src.splitlines():
            # ⚠⚠ 先把行尾註解切掉再比。第一版用 `t.endswith(" in results:")`，
            #   突變實驗加一個 `# MUT` 就繞過去了 —— 一個註解就能關掉的守衛不算守衛。
            t = line.split("#", 1)[0].strip()
            # ⚠ 只看**直接迭代 results** 的迴圈。`results.items()` 之類是別的變數，
            #   掃太寬會抓到不相干的行（第一版就抓到 main_window 的
            #   `for area_id, mdtsp_result in results.items():`）。
            if t.startswith("for ") and t.endswith(" in results:") and t.count(",") != 2:
                bad.append(f"{rel}: {t}")
    assert not bad, bad


def test_docstring_states_the_three_way_shape():
    """介面文件要說得出第三個值是什麼，否則下一個人又會只改一邊。"""
    doc = inspect.getdoc(SITLLauncher.start_multi) or ""
    assert "實際 instance" in doc and "conn_str" in doc, doc


def test_last_started_instance_is_recorded_by_start():
    """`start()` 必須把位移後的值記下來，`start_multi` 才拿得到。"""
    lz = _FakeLauncher({0: 9})
    lz.start(instance=0)
    assert lz._last_started_instance == 9
