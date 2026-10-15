"""tests/test_solver_wallclock_independence.py — 求解器不得依賴牆鐘

背景
----
``IDP_Solver`` 的 joint 局部搜尋同時受兩個預算限制::

    budget = [self.max_joint_evals]              # 確定性：數迭代次數
    t_end = time.monotonic() + self.max_joint_seconds   # 牆鐘：數秒數
    if budget[0] <= 0 or time.monotonic() > t_end:
        break

機器忙的時候牆鐘會先到，搜尋跑到的輪數變少 ⇒ 同一份輸入不同答案 ⇒
golden 值間歇性紅燈，而且**紅的條數隨機器負載變動**。這種紅燈最貴的地方
不是煩，而是它會讓真正的紅燈被當成雜訊忽略掉。

修法是 ``conftest.py`` 把 ``DEFAULT_MAX_JOINT_SECONDS`` 設成 ``math.inf``，
整個測試 session 只留 evals 預算。本檔把那件事鎖成**值層級**契約：
不檢查源碼長什麼樣（源碼層的鎖只驗「那一行長什麼樣」，改上游／下游／旁路
全免疫），而是把時鐘換成兩個**互相矛盾**的假時鐘，要求答案一個位元都不變：

- ``frozen``    — 時間永遠不前進 ⇒ 任何時間預算「永遠用不完」
- ``exploding`` — 每次呼叫跳 1e6 秒 ⇒ 任何時間預算「第一次檢查就過期」

⚠️ 本檔的陽性對照**直接打在生產程式碼上**，不是打在合成假貨上：
   ``test_wallclock_budget_really_is_a_lever`` 把 ``max_joint_seconds``
   設回有限值，證明那兩個假時鐘**確實會**讓 joint 給出不同答案。
   若那條變綠，就表示假時鐘沒生效，本檔其餘的「結果沒變」全是假綠燈。
"""
from __future__ import annotations

import math
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, List, Tuple

import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from core.dccpp import idp_solver as _idp_solver
from core.dccpp.idp_solver import IDP_Solver

# 沿用既有測試的場景 —— 這支守衛要守的就是那兩批值
from tests.test_dccpp_pipeline import _run_pipeline
from tests.test_idp_load_balance import CASES, _setup


# ═══════════════════════════════════════════════════════════════════════
#  假時鐘
# ═══════════════════════════════════════════════════════════════════════

# 求解器預算會用到的介面，全部要換掉。
# 註：``time.time_ns`` 也在內 —— CPython 的 logging 用它建 LogRecord，所以
# 假時鐘的回傳值刻意維持在**合法時間戳範圍**內，換掉不會炸掉日誌格式化。
_PATCHED_NAMES: Tuple[str, ...] = (
    'time', 'time_ns',
    'monotonic', 'monotonic_ns',
    'perf_counter', 'perf_counter_ns',
    'process_time', 'process_time_ns',
)
_REAL = {n: getattr(time, n) for n in _PATCHED_NAMES}
_REAL_IDS = {id(f): n for n, f in _REAL.items()}

_EPOCH_S = 1_700_000_000.0     # 2023-11-14，合法時間戳
_MAX_S = 2.0e11                # 約公元 8300 年，仍在 datetime 有效範圍內
_EXPLODING_STEP_S = 1.0e6      # 每次呼叫跳 ~11.6 天，遠大於任何合理預算


class _FakeClock:
    """單調前進速率可調的假時鐘，同時統計被查詢了幾次。"""

    def __init__(self, step_s: float, label: str) -> None:
        self.step_s = float(step_s)
        self.label = label
        self.calls = 0
        self.capped = False

    def _now_s(self) -> float:
        self.calls += 1
        v = _EPOCH_S + self.step_s * (self.calls - 1)
        if v > _MAX_S:
            # 到頂之後就凍住 —— 但 capped 會讓斷言看得見，不許靜靜退化
            self.capped = True
            return _MAX_S
        return v

    def seconds(self) -> float:
        return self._now_s()

    def nanoseconds(self) -> int:
        return int(self._now_s() * 1e9)


# ``Path.resolve()`` 會碰檔案系統。全套跑起來 sys.modules 有上千個模組，
# 而每個測試要進出假時鐘兩次 —— 不快取的話光這一項就讓整套測試慢好幾倍。
# 只快取「這個檔案算不算專案內」（``__file__`` 不會變），**不**快取掃出來的
# 綁定清單：那份清單一旦過期，守衛就會漏掉綁定而且完全無聲。
_PROJECT_FILE_CACHE: Dict[str, bool] = {}


def _is_project_module(mod) -> bool:
    f = getattr(mod, '__file__', None)
    if not f:
        return False
    hit = _PROJECT_FILE_CACHE.get(f)
    if hit is None:
        try:
            p = Path(f).resolve()
            p.relative_to(_REPO_ROOT)
            hit = 'site-packages' not in p.parts
        except (ValueError, OSError, RuntimeError):
            hit = False
        _PROJECT_FILE_CACHE[f] = hit
    return hit


@contextmanager
def _adversarial_clock(step_s: float, label: str):
    """把整個專案的時鐘換成假的；離開時完全還原。

    除了 ``time`` 模組本身，還要掃過所有**專案內**已載入模組的屬性：
    ``from time import monotonic`` 這種寫法在 import 當下就把原函式抓走了，
    只 setattr(time, ...) 對它完全無效 —— 那正是守衛會啞掉的破口。
    """
    clock = _FakeClock(step_s, label)
    fakes = {
        n: (clock.nanoseconds if n.endswith('_ns') else clock.seconds)
        for n in _PATCHED_NAMES
    }

    saved_module = {n: getattr(time, n) for n in _PATCHED_NAMES}
    for n in _PATCHED_NAMES:
        setattr(time, n, fakes[n])

    rebound: List[Tuple[Any, str, Any]] = []
    for mod_name, mod in list(sys.modules.items()):
        if mod is None or mod_name == __name__ or not _is_project_module(mod):
            continue
        for attr, val in list(vars(mod).items()):
            n = _REAL_IDS.get(id(val))
            if n is not None:
                setattr(mod, attr, fakes[n])
                rebound.append((mod, attr, val))

    try:
        # ── 自檢：假時鐘真的裝上了嗎？──────────────────────────────
        # 裝不上 ⇒ 後面「結果沒變」全是假綠燈，必須大聲失敗。
        probe = time.monotonic()
        expected = _EPOCH_S + step_s * (clock.calls - 1)
        if probe != expected:
            raise RuntimeError(
                '假時鐘 {} 沒裝上：time.monotonic() 回 {!r}，預期 {!r} '
                '—— 本檔所有結論都不成立'.format(label, probe, expected)
            )
        clock.calls = 0        # 自檢用掉的那一次不計入普查
        yield clock
    finally:
        for n, v in saved_module.items():
            setattr(time, n, v)
        for m, a, v in rebound:
            setattr(m, a, v)


def _frozen():
    return _adversarial_clock(0.0, 'frozen')


def _exploding():
    return _adversarial_clock(_EXPLODING_STEP_S, 'exploding')


# ═══════════════════════════════════════════════════════════════════════
#  求解 + 摘要
# ═══════════════════════════════════════════════════════════════════════

# 只挑一個代表性 CASE 做 frozen/exploding 對照，其餘用 parametrize 掃。
# (1000.0, 138.6, 6) 是 test_idp_load_balance 用來驗「joint 真的產生
# 不連續分組」的那一個 —— 局部搜尋在它身上確實有事情可做。
# ⚠️⚠️ 2026-09-04 換組態。原本是 (1000.0, 138.6, 6)。Dubins 修好之後，
#   那個組態的 joint 局部搜尋**一步都不動**（連續分組已是局部最佳），
#   於是它連檢查時鐘的迴圈都沒進去 —— 實測把預算一路降到 1e-7 秒，
#   凍結時鐘與爆炸時鐘仍給出同一個答案，陽性對照整個失效。
#   ``(2000.0, 200.0, 4)`` 是四個 CASES 裡唯一還會用到「放棄連續」的，
#   在它身上預算從 8.0 一路到 1e-6 都是有效槓桿。
#   相關說明見 tests/test_idp_load_balance.py 的
#   ``test_joint_actually_produces_non_contiguous_groups``。
_LEVER_CASE = (2000.0, 200.0, 4)


def _solve_joint(case, max_joint_seconds=None) -> Dict[int, Any]:
    """跑 joint 指派並回傳可逐位元比對的摘要。"""
    half, w_i, m = case
    ar, tf, uavs = _setup(half, w_i, m)
    r = IDP_Solver(min_turn_radius=60.0, altitude=120.0, assignment='joint',
                   max_joint_seconds=max_joint_seconds).solve(uavs, ar, tf)
    return {
        u: (tuple(sl.index for sl in p.ordered_scan_lines),
            tuple(p.directions), p.total_length_m)
        for u, p in sorted(r.per_uav.items())
    }


def _pipeline_digest() -> Dict[str, Any]:
    res = _run_pipeline()
    ap = res['assembled_paths']
    return {
        'total_makespan': res['total_makespan'],
        'allocation': {k: list(v) for k, v in sorted(res['allocation'].items())},
        'lengths': {
            u: (ap[u].total_length_m, ap[u].entry_length_m,
                ap[u].operation_length_m, ap[u].transfer_length_m)
            for u in sorted(ap)
        },
        'waypoints': {
            u: [(w.lat, w.lon, getattr(w, 'alt', None)) for w in ap[u].waypoints]
            for u in sorted(ap)
        },
    }


# ═══════════════════════════════════════════════════════════════════════
#  1. 陽性對照 —— 這套儀器真的量得到牆鐘依賴嗎？
# ═══════════════════════════════════════════════════════════════════════

class TestInstrumentCanFire:
    """⚠️ 沒有這一節，下面每一條都可能是「儀器壞了」偽裝成的綠燈。"""

    def test_wallclock_budget_really_is_a_lever(self):
        """把牆鐘預算設回有限值，兩個假時鐘**必須**給出不同答案。

        這是打在生產程式碼上的陽性對照：它同時證明
        (a) 假時鐘確實影響得到 joint 局部搜尋，
        (b) ``max_joint_seconds`` 是真的槓桿，不是裝飾。
        這條若變綠，本檔其餘所有「結果沒變」都不能採信。
        """
        with _frozen():
            a = _solve_joint(_LEVER_CASE, max_joint_seconds=8.0)
        with _exploding():
            b = _solve_joint(_LEVER_CASE, max_joint_seconds=8.0)
        assert a != b, (
            '有限的牆鐘預算下，凍結時鐘與爆炸時鐘竟然給出同一個答案 —— '
            '假時鐘沒有生效，本檔的其他斷言全是假綠燈'
        )

    def test_call_census_actually_counts(self):
        """普查計數器本身要會加一，否則『0 次』只是計數器壞掉。"""
        with _exploding() as clock:
            _solve_joint(_LEVER_CASE, max_joint_seconds=8.0)
            observed = clock.calls
        assert observed > 0, '計數器沒動 —— 任何「查詢 0 次」的斷言都無意義'

    def test_clock_patch_reaches_from_time_import_bindings(self):
        """``from time import monotonic`` 式的綁定也要被換掉。"""
        import types
        victim = types.ModuleType('_wallclock_guard_victim')
        victim.__file__ = str(_REPO_ROOT / '_wallclock_guard_victim.py')
        victim.monotonic = time.monotonic          # 模擬 from time import monotonic
        sys.modules['_wallclock_guard_victim'] = victim
        try:
            captured = victim.monotonic
            with _frozen():
                assert victim.monotonic is not captured, (
                    'from-import 綁定沒被換掉 —— 求解器若這樣寫，守衛會啞掉'
                )
                assert victim.monotonic() == _EPOCH_S
            assert victim.monotonic is captured, '離開後沒還原'
        finally:
            sys.modules.pop('_wallclock_guard_victim', None)


# ═══════════════════════════════════════════════════════════════════════
#  2. 修法本身 —— conftest 真的把牆鐘關掉了嗎？
# ═══════════════════════════════════════════════════════════════════════

class TestDeterministicBudgetIsActive:

    def test_conftest_disabled_the_wallclock_budget(self):
        """整個測試 session 的預設必須是「不看牆鐘」。"""
        assert _idp_solver.DEFAULT_MAX_JOINT_SECONDS == math.inf, (
            'conftest.py 沒有把 DEFAULT_MAX_JOINT_SECONDS 設成 inf —— '
            'golden 值會重新開始隨機器負載飄動'
        )

    def test_default_constructed_solver_is_time_unlimited(self):
        """不傳參數建出來的求解器（含管線內部那顆）必須是無時限的。"""
        s = IDP_Solver(min_turn_radius=60.0)
        assert s.max_joint_seconds == math.inf

    def test_explicit_seconds_still_honoured(self):
        """明確傳值時仍要生效 —— 生產環境的延遲保險絲不能被這個修法弄壞。"""
        s = IDP_Solver(min_turn_radius=60.0, max_joint_seconds=2.5)
        assert s.max_joint_seconds == pytest.approx(2.5)

    def test_evaluation_budget_is_what_bounds_the_search(self):
        """關掉牆鐘之後，仍必須有一個確定性預算負責終止。"""
        s = IDP_Solver(min_turn_radius=60.0)
        assert s.max_joint_evals > 0
        assert s.max_joint_iters > 0


# ═══════════════════════════════════════════════════════════════════════
#  3. 契約 —— 測試組態下，結果不得隨牆鐘改變
# ═══════════════════════════════════════════════════════════════════════

class TestWallClockIndependence:

    @pytest.mark.parametrize("case", CASES)
    def test_joint_identical_under_adversarial_clocks(self, case):
        """每個 CASE 的 joint 解在兩個矛盾時鐘下必須逐位元相同。"""
        with _frozen():
            a = _solve_joint(case)
        with _exploding():
            b = _solve_joint(case)
        assert a == b, (
            'joint 指派隨牆鐘改變 —— test_idp_load_balance.py 的不變式與 '
            'test_dccpp_pipeline.py 的 golden 會隨機器負載間歇性紅燈'
        )

    def test_pipeline_identical_under_adversarial_clocks(self):
        """整條 DCCPP 管線（含 golden 航點）在兩個矛盾時鐘下必須逐位元相同。"""
        with _frozen():
            a = _pipeline_digest()
        with _exploding():
            b = _pipeline_digest()
        assert a == b, (
            'DCCPP 管線輸出隨牆鐘改變 —— test_dccpp_pipeline.py 的 golden '
            '會隨機器負載間歇性紅燈'
        )

    def test_pipeline_matches_unpatched_run(self):
        """假時鐘下的結果要跟真時鐘下一致（排除是假時鐘自己造成的偏移）。"""
        real = _pipeline_digest()
        with _exploding():
            faked = _pipeline_digest()
        assert real == faked

    def test_clock_never_capped_during_solve(self):
        """假時鐘不得跑到上限而退化成 frozen（否則 exploding 名存實亡）。"""
        with _exploding() as clock:
            _solve_joint(_LEVER_CASE)
            _pipeline_digest()
            assert not clock.capped, (
                'exploding 時鐘在 {} 次查詢後到頂並凍住，這一趟的對照已失效'
                .format(clock.calls)
            )


# ═══════════════════════════════════════════════════════════════════════
#  4. 更緊的鎖 —— 測試組態下，求解路徑根本不該查詢牆鐘
# ═══════════════════════════════════════════════════════════════════════

# 允許查詢牆鐘的合法用途（例如純粹記錄耗時的 log）。
# ⚠️ 往這裡加東西之前先想清楚：只要那個值不影響輸出，上面的
#    frozen/exploding 對照就會放行；真的需要放行才加。
_WALLCLOCK_ALLOWANCE = 0


class TestNoWallClockOnSolvePath:

    @pytest.mark.parametrize("case", CASES)
    def test_joint_never_consults_wallclock(self, case):
        with _exploding() as clock:
            _solve_joint(case)
            observed = clock.calls
        assert observed <= _WALLCLOCK_ALLOWANCE, (
            'joint 求解查詢了牆鐘 {} 次（允許 {}）。若只是記錄耗時、不影響輸出，'
            '上面的 frozen/exploding 對照才是真契約；但先確認它沒有變回搜尋預算。'
            .format(observed, _WALLCLOCK_ALLOWANCE)
        )

    def test_dccpp_pipeline_never_consults_wallclock(self):
        with _exploding() as clock:
            _pipeline_digest()
            observed = clock.calls
        assert observed <= _WALLCLOCK_ALLOWANCE, (
            'DCCPP 管線查詢了牆鐘 {} 次（允許 {}）'
            .format(observed, _WALLCLOCK_ALLOWANCE)
        )
