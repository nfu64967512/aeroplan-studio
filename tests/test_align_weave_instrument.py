# -*- coding: utf-8 -*-
"""ALIGN 段蛇形的儀表 —— 對**真實飛行資料**下斷言(2026-08-24)。

═══════════════════════════════════════════════════════════════════════════
 為什麼這個檔測的是 CSV 而不是函式
═══════════════════════════════════════════════════════════════════════════
實際發生的失效是這個形狀:LATOBS1 的 ALIGN 段有 **12,618 個 mode=weave 拍**,
而下列六欄**全部 0%**:

    weave_yaw_deg  weave_early_rel_s  early_by_s
    secs_to_target  itcg_eps_t  itcg_accel

也就是說:那一段的蛇形**不存在於任何資料裡**,而 `mode` 欄老實寫著 `weave`。
`scripts/wsl/weave_readout.py` 對那趟直接報「沒有 wait 段」——兩個工具對同一段
一起瞎。

⇒ 記錄在案的「0.21 個週期、零換向、只吸得到 2.85 s、達成率 24–33%」量的是
  **等待段**;ALIGN 段從來沒被分析過,而實測它換向 1–4 次、達成率 42.8%。

⚠️⚠️ 這種失效**函式層的測試抓不到**。每一支函式都對:`_weave_theta_of` 正確地
   寫 `self.weave_theta[i]`,`_row` 正確地讀它。錯的是 ALIGN 段用**另一條**
   內聯路徑算 theta,而它從不寫那個 dict —— 沒有任何一支函式壞掉。
   ⇒ 只有「拿真航次的 CSV 來看那幾欄是不是空的」抓得到。

⚠️ 本檔在沒有航次資料時會 **skip**(乾淨 checkout / CI)。skip 不是通過 ——
   它表示這道守衛這次沒有被執行。
"""
import csv
import io
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
ART = ROOT / "runs" / "artifacts"

#: ALIGN 段在擺動時,這些欄位**必須**有值。
#: ⚠️ 這份清單就是那次失效的清單 —— 它們當初全部是 0%。
REQUIRED_IN_ALIGN_WEAVE = ["weave_yaw_deg", "align_early_s", "align_vreq_mps"]

#: 這些欄位在**所有** ALIGN 拍都要有(不只擺動中的)。
#: ⚠️ 只記擺動中的會讓分母消失:沒在擺的機體的 align_early_s 正是
#:   「為什麼它不用擺」的答案,少了它,達成率會被算成 100%。
REQUIRED_IN_ALL_ALIGN = ["align_early_s", "align_vreq_mps"]


def _csvs():
    if not ART.is_dir():
        return []
    out = []
    for d in sorted(ART.iterdir()):
        if not d.is_dir():
            continue
        out += sorted(d.glob("tot_*.csv"))
    return out


def _rows(path, limit=400000):
    with io.open(path, encoding="utf-8", errors="replace", newline="") as fh:
        for k, r in enumerate(csv.DictReader(fh)):
            if k >= limit:
                return
            yield r


def _newest_with_align():
    """最新一份**有 ALIGN 段而且帶新欄位**的 CSV。

    ⚠️ 舊航次沒有這兩欄是**正常的**(欄位是後來加的),那種檔要跳過而不是
       判紅 —— 拿新守衛去判舊資料,得到的紅燈沒有任何意義。
    """
    for p in reversed(_csvs()):
        try:
            with io.open(p, encoding="utf-8", errors="replace", newline="") as fh:
                hdr = next(csv.reader(fh))
        except (StopIteration, OSError):
            continue
        if "align_early_s" not in hdr:
            continue
        for r in _rows(p, limit=200000):
            if r.get("phase") == "ALIGN":
                return p
    return None


@pytest.fixture(scope="module")
def flight():
    p = _newest_with_align()
    if p is None:
        pytest.skip("沒有帶 ALIGN 新欄位的航次資料 —— 這道守衛這次沒被執行")
    return p


def test_the_align_weave_is_not_invisible(flight):
    """⭐⭐⭐ 主鎖:ALIGN 段在擺動時,儀表不可以是空的。

    這一條就是那次失效本身。它紅的時候意思很明確:**又有一段蛇形跑了
    而沒有留下任何痕跡**,而下游的每一個分析都會回報「這一段沒有蛇形」。
    """
    n_weave = 0
    filled = {c: 0 for c in REQUIRED_IN_ALIGN_WEAVE}
    for r in _rows(flight):
        if r.get("phase") != "ALIGN" or r.get("mode") != "weave":
            continue
        n_weave += 1
        for c in REQUIRED_IN_ALIGN_WEAVE:
            if r.get(c) not in ("", None):
                filled[c] += 1
    if n_weave < 50:
        pytest.skip("這一趟的 ALIGN 段只有 %d 個 weave 拍,樣本不足" % n_weave)
    dead = [c for c in REQUIRED_IN_ALIGN_WEAVE if filled[c] == 0]
    assert not dead, (
        "⚠️⚠️⚠️ %s 在 %d 個 ALIGN/weave 拍裡**整欄空白**:%s\n"
        "    這正是 2026-08-24 抓到的那個失效:蛇形跑了 12,618 拍,而\n"
        "    weave_yaw_deg / weave_early_rel_s / early_by_s / secs_to_target /\n"
        "    itcg_eps_t / itcg_accel 六欄全部 0%%,mode 欄卻寫著 weave。\n"
        "    ⇒ 那一段不存在於任何資料裡,而記錄在案的「零換向、2.85 s」\n"
        "      量的是**另一段**。\n"
        "    ⚠️ 函式層的測試抓不到這個 —— 每一支函式都對,錯的是 ALIGN 段\n"
        "      走另一條內聯路徑而從不寫那個 dict。"
        % (flight.name, n_weave, dead))


def test_every_align_tick_has_the_driver_not_just_the_weaving_ones(flight):
    """⚠️ 沒在擺的機體也要記。

    它們的 align_early_s 正是「為什麼它不用擺」的答案。少了它,任何
    「達成率」的分母都會消失,而率就會被算成 100%。
    """
    n_all = 0
    filled = {c: 0 for c in REQUIRED_IN_ALL_ALIGN}
    for r in _rows(flight):
        if r.get("phase") != "ALIGN":
            continue
        n_all += 1
        for c in REQUIRED_IN_ALL_ALIGN:
            if r.get(c) not in ("", None):
                filled[c] += 1
    if n_all < 200:
        pytest.skip("ALIGN 拍太少(%d)" % n_all)
    for c in REQUIRED_IN_ALL_ALIGN:
        frac = filled[c] / n_all
        assert frac > 0.90, (
            "⚠️ %s 只在 %.1f%% 的 ALIGN 拍有值(%d/%d)。\n"
            "    ⚠️ 所有 ALIGN 機體都要記,不只在擺的那些。" %
            (c, 100 * frac, filled[c], n_all))


def test_the_two_drivers_are_consistent_with_each_other(flight):
    """⭐⭐ 兩欄加上既有的 d_slot 是**超定的**,可以互相驗證。

        align_vreq_mps = d / t_rem        ⇒  t_rem = d / align_vreq_mps
        align_early_s  = t_rem − d / vmin

    ⇒ 由 align_vreq_mps 與 d_slot 反推的 t_rem,代進第二式必須得到
      align_early_s。任何一式打錯(vmax 寫成 vmin、分子分母顛倒、
      減號寫成加號)都會在這裡露餡,而那些錯誤**每一個都會產出正數**,
      單看數值分佈完全正常。

    ⚠️ vmin 從契約快照讀。讀不到就 skip —— 用猜的 vmin 去驗算,
       驗的是我的猜測不是程式。
    """
    import json
    snap = flight.parent / ("contract_%s.json" % flight.parent.name)
    if not snap.is_file():
        pytest.skip("找不到 %s,沒有 vmin 就無法驗算" % snap.name)
    try:
        vmin = float(json.loads(snap.read_text(encoding="utf-8"))["values"]["vmin"])
    except (KeyError, ValueError, TypeError):
        pytest.skip("契約快照裡沒有 vmin")

    bad, n = [], 0
    for r in _rows(flight):
        if r.get("phase") != "ALIGN":
            continue
        try:
            vreq = float(r["align_vreq_mps"])
            early = float(r["align_early_s"])
            d = float(r["d_slot"])
        except (KeyError, ValueError, TypeError):
            continue
        if vreq <= 1e-6 or d <= 0:
            continue
        n += 1
        t_rem = d / vreq
        want = t_rem - d / vmin
        # ⚠️ 容差取記錄精度(兩位小數)的幾倍:d_slot 是**四捨五入到整數**的,
        #    所以 d 本身就帶 ±0.5 m 的量化,而它會被 1/vreq 放大。
        tol = max(0.5, 0.5 / max(vreq, 1e-6) + 0.5 / vmin) + 0.05
        if abs(early - want) > tol:
            bad.append((d, vreq, early, want))
    if n < 200:
        pytest.skip("可驗算的 ALIGN 拍太少(%d)" % n)
    frac = len(bad) / n
    assert frac < 0.02, (
        "⚠️⚠️ %.1f%% 的 ALIGN 拍(%d/%d)兩欄互不一致。\n"
        "    align_early_s 應該 = d/align_vreq_mps − d/vmin(vmin=%.1f)。\n"
        "    前 3 筆 (d, vreq, 實得 early, 應為):\n      %s\n"
        "    ⚠️ 這種錯每一個都會產出正數,單看數值分佈完全正常。" %
        (100 * frac, len(bad), n, vmin,
         "\n      ".join(repr(b) for b in bad[:3])))


def test_the_driver_does_not_leak_outside_align(flight):
    """⚠️⚠️ 相位守衛:離開 ALIGN 之後不可以還帶著舊值。

    align_dbg 是逐拍覆寫但**從不清空**的。少了 `phase == ALIGN` 那道守衛,
    俯衝段的每一列都會掛著最後一次 ALIGN 的數字 ——「早到 37.2 秒」出現在
    一列 mode=dive 上,沒有人會覺得不對勁,而它會被當成俯衝段的量測。
    """
    leak, n_out = 0, 0
    for r in _rows(flight):
        if r.get("phase") == "ALIGN":
            continue
        n_out += 1
        if any(r.get(c) not in ("", None) for c in REQUIRED_IN_ALL_ALIGN):
            leak += 1
    if n_out < 100:
        pytest.skip("ALIGN 以外的拍太少(%d)" % n_out)
    assert leak == 0, (
        "⚠️⚠️ %d/%d 個非 ALIGN 的拍帶著 ALIGN 的驅動量 ⇒ 舊值殘留。\n"
        "    這些數字看起來完全合理,而它們描述的是幾分鐘前的另一個相位。"
        % (leak, n_out))
