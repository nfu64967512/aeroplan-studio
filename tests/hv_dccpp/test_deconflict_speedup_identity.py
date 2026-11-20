"""vendored 修改 #13：去衝突加速必須**逐位元不變**——拿凍結的原版在同一個程序裡逐筆對照（差分測試）。

為什麼不能只看規劃結果：驗證器（``hv_dccpp.sim.validate``）也呼叫加速後的 ``check_separation``，
規劃與驗證同時錯的話兩邊會一致地錯，等於自己驗自己。原版凍結在 ``tests/hv_dccpp/reference/``
（說明見那裡的 README），以 ``hv_dccpp._deconflict_pre_speedup`` 載入，有自己的 ``Conflict``／``_Repair``，
與現行版不共用任何快取。比較用型別標記的正規形式：float 與 np.float64、−0.0 與 0.0、list 與 tuple 都算不同。

涵蓋：
* ``check_separation``、``repair``（含收尾全域重排）在擾動過的真實計畫上（mini 範例複製成 2／6／8 架、
  起飛平移、已起飛、幽靈、空間互斥對、兩種目標函數、小評估預算）；
* ``_global_reschedule`` 直接呼叫（resolve 兩種、評估預算 0／1／7／60 ⇒ ``evals`` 計數必須一樣）；
* 驗證器換成原版 ``check_separation`` 後報告逐欄位相同；
* **快取騙不過**：在同一個快取範圍內就地改航點欄位、換同長度航點清單、改階段／航跡角／起降站／時刻，
  每一步都要與原版重算的結果相同（而且每一步都真的改變了結果，否則測不到東西）；repair 期間回呼就地改計畫也一樣；
* ``shift_plan`` 位置參數快路徑仍在作用、與 ``dataclasses.replace`` 版逐欄位相同；``_escape`` 二分搜尋 = 線性掃描。
"""
from __future__ import annotations

import copy
import dataclasses
import enum
import hashlib
import importlib.util
import logging
import math
import random
import sys
import threading
import types
from pathlib import Path

import numpy as np
import pytest

shapely = pytest.importorskip("shapely")
pytest.importorskip("scipy")

import hv_dccpp                                                 # noqa: E402
from hv_dccpp import deconflict as D                            # noqa: E402
from hv_dccpp import sim as SIM                                 # noqa: E402
from hv_dccpp.context import build_context                      # noqa: E402
from hv_dccpp.mission import Mode, Phase, PlannerConfig, UAVPlan, Waypoint   # noqa: E402
from hv_dccpp.scenario import build_mini_example                # noqa: E402

REF_PATH = Path(__file__).parent / "reference" / "deconflict_pre_speedup.py"
REF_MD5_LF = "2ab8fca397ef12bf6ce8a2078b65174d"      # 換行正規化成 LF 後（git autocrlf 兩種 checkout 都一樣）


def _load_reference():
    name = "hv_dccpp._deconflict_pre_speedup"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, REF_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod                                     # dataclass(slots=True) 解析註解要找得到模組
    spec.loader.exec_module(mod)
    return mod


REF = _load_reference()


# ---------------------------------------------------------------------------
# 正規形式（型別標記；dict 保留插入順序——plans 的順序本身就是輸出的一部分）
# ---------------------------------------------------------------------------

_WALL_CLOCK = frozenset({"compute_ms", "timing_ms"})    # 牆鐘量測值：同一版本跑兩次也不同，不比

def canon(v):
    if v is None or isinstance(v, (bool, str)):
        return v
    if isinstance(v, enum.Enum):
        return ("E", type(v).__name__, v.name)
    if isinstance(v, np.generic):
        return ("np", str(v.dtype), repr(v.item()))
    if isinstance(v, int):
        return ("i", type(v).__name__, repr(v))
    if isinstance(v, float):
        return ("f", type(v).__name__, repr(v))
    if isinstance(v, np.ndarray):
        if v.dtype == object:
            return ("ndo", v.shape, [canon(x) for x in v.ravel().tolist()])
        return ("nd", str(v.dtype), v.shape, np.ascontiguousarray(v).tobytes())
    if isinstance(v, tuple):
        return ("T", [canon(x) for x in v])
    if isinstance(v, list):
        return ("L", [canon(x) for x in v])
    if isinstance(v, (set, frozenset)):
        return ("S", sorted(repr(canon(x)) for x in v))
    if isinstance(v, dict):
        return ("D", [(canon(k), canon(x)) for k, x in v.items()])
    if dataclasses.is_dataclass(v) and not isinstance(v, type):
        return ("C", type(v).__name__, [(f.name, canon(getattr(v, f.name))) for f in dataclasses.fields(v)
                                        if f.name not in _WALL_CLOCK])
    if isinstance(v, types.SimpleNamespace):
        return ("NS", canon(vars(v)))
    if type(v).__module__.startswith("shapely"):
        return ("G", shapely.to_wkb(v, hex=True))
    if hasattr(v, "__dict__"):
        return ("O", type(v).__name__, canon(dict(vars(v))))
    return ("R", type(v).__name__, repr(v))


def _sched_rows(sched):
    return [(r.site_id, r.kind, r.t_start_s, r.t_end_s, r.uav_id) for r in sched.reservations]


# ---------------------------------------------------------------------------
# 輸入：mini 範例的真實計畫，複製／擾動成多種情境
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def mini():
    logging.disable(logging.WARNING)
    try:
        inputs, cfg = build_mini_example()
        mp = hv_dccpp.plan_mission(inputs, cfg)
        ctx = build_context(inputs, cfg)
    finally:
        logging.disable(logging.NOTSET)
    return types.SimpleNamespace(inputs=inputs, cfg=cfg, plan=mp, ctx=ctx, base=dict(mp.plans))


def _decimate(p: UAVPlan, k: int) -> UAVPlan:
    """每 k 個航點留一個（保留首尾）：仍是合法計畫（t 非遞減），只為了讓原版在多機情境跑得完。"""
    wps = p.waypoints
    keep = sorted(set(range(0, len(wps), k)) | {len(wps) - 1})
    return dataclasses.replace(p, waypoints=[wps[i] for i in keep], warnings=list(p.warnings))


def _clone(p: UAVPlan, uav_id: str, dx: float, dy: float, dt: float, *, site: str | None = None) -> UAVPlan:
    wps = [dataclasses.replace(w, x_m=w.x_m + dx, y_m=w.y_m + dy, t_s=w.t_s + dt) for w in p.waypoints]
    return dataclasses.replace(p, uav_id=uav_id, waypoints=wps, t_takeoff_s=p.t_takeoff_s + dt,
                               t_arr_ptc_s=p.t_arr_ptc_s + dt, t_land_s=p.t_land_s + dt,
                               landing_site=site or p.landing_site, takeoff_site=site or p.takeoff_site,
                               warnings=list(p.warnings))


def _ghost(p: UAVPlan, uid: str, dt: float):
    s = REF.trajectory_samples(p, 5.0)
    return types.SimpleNamespace(uav_id=uid, samples=np.column_stack([s[:, 0] + 80.0, s[:, 1] - 60.0, s[:, 2],
                                                                      s[:, 3] + dt]),
                                 sigma_t=7.5, landing_site=p.landing_site, eta_s=float(s[-1, 3] + dt))


def _fleet(base: dict[str, UAVPlan], n: int, rng: random.Random, decimate: int) -> dict[str, UAVPlan]:
    src = [_decimate(p, decimate) for p in base.values()]
    out: dict[str, UAVPlan] = {}
    for i in range(n):
        p = src[i % len(src)]
        uid = f"{p.cls}{i + 1}"
        dx, dy = rng.choice([(0.0, 0.0), (60.0, 0.0), (0.0, 80.0), (250.0, -40.0), (-150.0, 120.0)])
        dt = 0.0 if i < len(src) else round(rng.uniform(-min(200.0, float(p.t_takeoff_s)), 600.0), 1)
        site = rng.choice([None, None, "H", f"P{i % 3}"])
        out[uid] = _clone(p, uid, dx, dy, dt, site=site) if i >= len(src) or dt or dx or dy or site else p
    return out


def _variants(m):
    """(名稱, plans, ghosts, skip_pairs, t_now, cfg)；固定亂數種子 ⇒ 每次相同。"""
    rng = random.Random(20261021)
    cfg0 = m.cfg
    out = []
    for k in range(8):                                          # 原兩架、起飛平移
        plans = {}
        for u, p in m.base.items():
            s = 0.0 if rng.random() < 0.3 else round(rng.uniform(-min(300.0, float(p.t_takeoff_s)), 700.0), 1)
            plans[u] = D.shift_plan(p, s) if s else p
        t_now = -math.inf if k % 2 == 0 else float(rng.choice([0.0, 30.0, 400.0]))
        ghosts = [_ghost(plans[rng.choice(sorted(plans))], "G9", rng.uniform(-200.0, 200.0))] if k % 4 >= 2 else None
        cfg = dataclasses.replace(cfg0, reschedule_objective="scan") if k == 5 else cfg0
        out.append((f"pair{k}", plans, ghosts, None, t_now, cfg))
    for k, n in enumerate((6, 6, 8, 8)):                        # 複製成 6／8 架（6 架全排列、8 架啟發序）
        plans = _fleet(m.base, n, rng, decimate=4)
        ids = sorted(plans)
        skip = {(ids[0], ids[2]), (ids[1], ids[3])} if k % 2 else None
        t_now = -math.inf if k < 2 else 50.0
        ghosts = [_ghost(plans[ids[-1]], "G9", 120.0)] if k == 3 else None
        cfg = dataclasses.replace(cfg0, reschedule_max_evals=60 if n == 6 else 200)   # 原版要跑得完
        out.append((f"fleet{n}_{k}", plans, ghosts, skip, t_now, cfg))
    return out


@pytest.fixture(scope="module")
def variants(mini):
    return _variants(mini)


def _run_repair(mod, plans, ghosts, skip, t_now, cfg, ctx, *, hook=None, max_rounds=3):
    plans = copy.deepcopy(plans)
    sched = mod.CorridorSchedule(cfg)
    to_site = {u: p.takeoff_site or ctx.inputs.home.site_id for u, p in plans.items()}
    for u, p in plans.items():
        sched.reserve_plan(p, to_site[u])
    confs = mod.check_separation(plans, cfg, ghosts, 20.0, skip)
    newp, resid = mod.repair(plans, list(confs), ctx, ctx.models, sched, cfg, max_rounds, ghosts=ghosts,
                             skip_pairs=skip, t_now_s=t_now, takeoff_site=to_site, resequence_hook=hook)
    return canon({"plans": newp, "resid": resid, "sched": _sched_rows(sched)}), plans


# ---------------------------------------------------------------------------
# 前置：對照組真的是凍結的原版、而且是另一個模組
# ---------------------------------------------------------------------------

def test_reference_is_the_frozen_pre_speedup_module():
    b = REF_PATH.read_bytes().replace(b"\r\n", b"\n")
    assert hashlib.md5(b).hexdigest() == REF_MD5_LF, "reference/deconflict_pre_speedup.py 被改過（見 reference/README.md）"
    assert REF is not D and REF.Conflict is not D.Conflict and REF._Repair is not D._Repair
    assert not hasattr(REF, "_SCOPE") and hasattr(D, "_SCOPE")
    assert REF.check_separation.__module__ == "hv_dccpp._deconflict_pre_speedup"


def test_variants_really_have_conflicts(variants):
    """前置斷言：多機情境都同時有 4D 與走廊衝突、兩架的情境至少一半有衝突（沒衝突的那幾個照樣有用：
    repair 沒有衝突時仍會跑工期壓縮與全域重排）。否則下面的比對測不到修復路徑。"""
    n_pair = 0
    for name, plans, ghosts, skip, _t_now, cfg in variants:
        sched = REF.CorridorSchedule(cfg)
        for u, p in plans.items():
            sched.reserve_plan(p, p.takeoff_site or "H")
        n4d, ncorr = len(REF.check_separation(plans, cfg, ghosts, 20.0, skip)), len(sched.conflicts())
        if name.startswith("fleet"):
            assert n4d >= 5 and ncorr >= 5, (name, n4d, ncorr)
        else:
            n_pair += bool(n4d or ncorr)
    assert n_pair >= 4


# ---------------------------------------------------------------------------
# 差分：check_separation／repair／_global_reschedule
# ---------------------------------------------------------------------------

def test_check_separation_matches_reference(variants):
    for name, plans, ghosts, skip, _t_now, cfg in variants:
        for only in (None, {sorted(plans)[0]}):
            a = D.check_separation(plans, cfg, ghosts, 20.0, skip, only_uavs=only)
            b = REF.check_separation(plans, cfg, ghosts, 20.0, skip, only_uavs=only)
            assert canon(a) == canon(b), name


@pytest.mark.parametrize("idx", range(12))
def test_repair_matches_reference(variants, mini, idx):
    name, plans, ghosts, skip, t_now, cfg = variants[idx]
    a, _ = _run_repair(D, plans, ghosts, skip, t_now, cfg, mini.ctx)
    b, _ = _run_repair(REF, plans, ghosts, skip, t_now, cfg, mini.ctx)
    assert a == b, name
    assert D._SCOPE.get() is None                               # 離開 repair 快取就丟掉


def _make_R(mod, plans, ctx, cfg, *, ghosts=None, skip=None, t_now=-math.inf):
    plans = copy.deepcopy(plans)
    to_site = {u: p.takeoff_site or "H" for u, p in plans.items()}
    sched = mod.CorridorSchedule(cfg)
    for u, p in plans.items():
        sched.reserve_plan(p, to_site[u])
    R = mod._Repair(plans, ctx, ctx.models, sched, cfg, to_site, float(t_now), 0.0, 20.0,
                    {str(g.uav_id): g for g in (ghosts or ())}, mod.default_energy_ok, None, mod._norm_pairs(skip))
    return R


@pytest.mark.parametrize("budget", [0, 1, 7, 60])
@pytest.mark.parametrize("resolve", [False, True])
def test_global_reschedule_matches_reference_including_eval_budget(variants, mini, budget, resolve):
    """同樣的順序、同樣的格點、同樣的 ``evals`` 計數：任何評估預算下結果都必須一樣。"""
    acted = 0
    for name, plans, ghosts, skip, t_now, cfg in (variants[1], variants[3], variants[8], variants[10]):
        Ra = _make_R(D, plans, mini.ctx, cfg, ghosts=ghosts, skip=skip, t_now=t_now)
        Rb = _make_R(REF, plans, mini.ctx, cfg, ghosts=ghosts, skip=skip, t_now=t_now)
        for objective in ("landing", "scan"):
            ma = D._global_reschedule(Ra, step_s=10.0, objective=objective, max_evals=budget, resolve=resolve)
            mb = REF._global_reschedule(Rb, step_s=10.0, objective=objective, max_evals=budget, resolve=resolve)
            assert ma == mb, (name, objective)
            assert canon(Ra.plans) == canon(Rb.plans), (name, objective)
            assert _sched_rows(Ra.sched) == _sched_rows(Rb.sched), (name, objective)
            acted += bool(ma)
    if budget >= 60:
        assert acted, "前置：預算夠時重排至少要出手一次，否則這條測不到擺放路徑"


def test_global_reschedule_respects_a_ghost_in_the_earliest_slot(mini):
    """幽靈正好飛在 B1 想提前到的時段（同航線、早 0–90 s）：重排只能把 B1 擺在幽靈過去之後。
    （情境刻意讓「不看幽靈」會選到最早時刻、再被套用前的全機驗證整個退回。）"""
    cfg = PlannerConfig()
    b1, b2 = _line("B1", 0.0, 600.0, phase=Phase.TRANSFER), _line("B2", 5000.0, 600.0, phase=Phase.TRANSFER)
    for p, site in ((b1, "S1"), (b2, "S2")):
        p.landing_site = p.takeoff_site = site
    s = REF.trajectory_samples(b1, 2.0)
    ghost = types.SimpleNamespace(uav_id="G1", samples=np.column_stack([s[:, 0], s[:, 1], s[:, 2], s[:, 3] - 600.0]),
                                  sigma_t=5.0, landing_site="GS", eta_s=float(s[-1, 3] - 600.0))
    plans = {"B1": b1, "B2": b2}
    Ra = _make_R(D, plans, mini.ctx, cfg, ghosts=[ghost])
    Rb = _make_R(REF, plans, mini.ctx, cfg, ghosts=[ghost])
    ma = D._global_reschedule(Ra, step_s=10.0)
    mb = REF._global_reschedule(Rb, step_s=10.0)
    assert ma == mb == {"B1", "B2"}
    assert canon(Ra.plans) == canon(Rb.plans)
    t_b1 = Ra.plans["B1"].t_takeoff_s
    assert 0.0 < t_b1 < 60.0 and Ra.plans["B2"].t_takeoff_s == 0.0      # B1 讓開幽靈、B2 提到最早
    assert not REF.check_separation(Rb.plans, cfg, [ghost])


def test_global_reschedule_checks_every_placed_aircraft_not_just_the_last(mini):
    """三架同航線（不同站）都可以提前：每一架都要與**所有**已擺放者錯開，不是只看最後擺的那一架。"""
    cfg = PlannerConfig()
    plans = {}
    for i, t0 in enumerate((600.0, 700.0, 800.0)):
        p = _line(f"B{i + 1}", 0.0, t0, phase=Phase.TRANSFER)
        p.landing_site = p.takeoff_site = f"S{i + 1}"
        plans[p.uav_id] = p
    Ra, Rb = _make_R(D, plans, mini.ctx, cfg), _make_R(REF, plans, mini.ctx, cfg)
    ma, mb = D._global_reschedule(Ra, step_s=10.0), REF._global_reschedule(Rb, step_s=10.0)
    assert ma == mb == {"B1", "B2", "B3"}
    assert canon(Ra.plans) == canon(Rb.plans)
    t = sorted(p.t_takeoff_s for p in Ra.plans.values())
    assert t[0] == 0.0 and t[1] > 0.0 and t[2] > t[1]           # 依序錯開
    assert not REF.check_separation(Rb.plans, cfg)


def test_shifted_track_equals_track_of_the_shifted_plan(mini):
    """全域重排的候選軌跡（不建 UAVPlan）＝原版「shift_plan 再建軌跡」，每個欄位逐位元（含起降區間決定的走廊遮罩）。"""
    cfg = mini.cfg
    plans = list(mini.base.values()) + [_lander("B7", (0.0, 0.0), 30.0), _line("B8", 0.0, 10.0)]
    f32 = dataclasses.replace(plans[-1], waypoints=[dataclasses.replace(w, t_s=np.float32(w.t_s))
                                                    for w in plans[-1].waypoints])   # 走退回路徑
    for p in plans + [f32]:
        sh = D._ShiftedTrack(p, cfg, 0.0)
        assert (sh.A is None) == (p is f32)
        for s in (-37.5, 0.0, 1e-7, 12.3, 600.0):
            got = sh.track(s, cacheable=False)
            ref = REF._track_from_plan(REF.shift_plan(p, s), cfg.dt_4d_s, 0.0, cfg)
            for f in ("uav_id", "samples", "heading", "oper", "is_ghost", "corr_pad", "corr_site"):
                assert canon(getattr(got, f)) == canon(getattr(ref, f)), (p.uav_id, s, f)
            st, sp = D._ShiftedTimes(p, s), REF.shift_plan(p, s)
            assert [canon(getattr(st, k)) for k in st.__slots__] == [canon(getattr(sp, k)) for k in st.__slots__]


def test_validator_with_reference_check_separation_gives_identical_report(mini, monkeypatch):
    """驗證器換成原版 check_separation，報告逐欄位相同（驗證器不能只靠加速版自己背書）。"""
    plan, ctx, cfg = mini.plan, mini.ctx, mini.cfg
    (ua, pa), (ub, pb) = list(plan.plans.items())
    for s in np.arange(-float(pb.t_takeoff_s), 1500.0, 25.0):  # 找一個讓兩機真的 4D 交會的平移（前置）
        moved = {ua: pa, ub: D.shift_plan(pb, float(s))}
        if any(c.kind == "4d" for c in REF.check_separation(moved, cfg)):
            break
    else:
        pytest.fail("前置：找不到會 4D 交會的平移")
    reps = []
    for mp in (plan, dataclasses.replace(plan, plans=moved)):
        rep_new = hv_dccpp.validate(mp, ctx, hv_dccpp.build_validation_cache(mp, ctx, budget_only=False), cfg)
        monkeypatch.setattr(SIM, "check_separation", REF.check_separation)
        rep_ref = hv_dccpp.validate(mp, ctx, hv_dccpp.build_validation_cache(mp, ctx, budget_only=False), cfg)
        monkeypatch.setattr(SIM, "check_separation", D.check_separation)
        assert canon(rep_new) == canon(rep_ref)
        reps.append(rep_new)
    assert any(c.kind == "4d" for c in reps[1].conflicts) and not reps[1].passed   # 第二份確實有 4D 衝突（比對咬得到衝突內容）


# ---------------------------------------------------------------------------
# 快取騙不過：同一個快取範圍內就地修改
# ---------------------------------------------------------------------------

def _wp(x, y, z, t, phase, track_deg):
    mode = Mode.HOVER if phase in (Phase.TAKEOFF, Phase.LAND) else (
        Mode.TRANSITION if phase in (Phase.TRANS_OUT, Phase.TRANS_IN) else Mode.CRUISE)
    return Waypoint(x, y, z, 0.0, 0.0, z + 100.0, z, t, mode, phase, phase is Phase.OPERATE,
                    None, None, None, 21.5, 20.0, track_deg, track_deg, 1000.0, "ALT1", True, "RTB")


def _plan(uav, wps, t_arr, t_land, site):
    return UAVPlan(uav, "B", wps, [], wps[0].t_s, t_arr, t_land, site, 100.0, 50.0, 2000.0, 2.0, 0, [],
                   takeoff_site=site)


def _line(uav, x0, t0, *, phase=Phase.OPERATE):
    """沿 +y 直飛 1.8 km（90 s，z = 200 m）；沒有起降段。"""
    wps = [_wp(x0, 20.0 * i, 200.0, t0 + float(i), phase, 0.0) for i in range(0, 91)]
    return _plan(uav, wps, t0 + 90.0, t0 + 90.0, "S")


def _lander(uav, start, t0, site="S"):
    """自 ``start``（南方 (0, 0) 或東方 (2000, 2000)）巡航 90 s 到坪 (0, 2000) 外 200 m，再 15 s 進場
    （TRANS_IN／LAND）觸地。兩個方向的巡航段彼此至少 280 m，只在坪邊相遇。"""
    sx, sy = start
    ex, ey = (0.0, 1800.0) if sx == 0.0 else (200.0, 2000.0)
    wps = [_wp(sx + (ex - sx) * f, sy + (ey - sy) * f, 200.0, t0 + 90.0 * f, Phase.TRANSFER, 0.0)
           for f in (i / 30.0 for i in range(31))]
    for k, ph in ((1, Phase.TRANS_IN), (2, Phase.TRANS_IN), (3, Phase.LAND)):
        f = k / 3.0
        wps.append(_wp(ex * (1.0 - f), ey + (2000.0 - ey) * f, 200.0 - 66.0 * k, t0 + 90.0 + 5.0 * k, ph, 0.0))
    return _plan(uav, wps, t0 + 90.0, t0 + 105.0, site)


def _same(plans, cfg):
    a = D.check_separation(plans, cfg)
    b = REF.check_separation(plans, cfg)
    assert canon(a) == canon(b)
    return canon(a)


def test_scope_cache_is_not_fooled_by_in_place_edits():
    cfg = PlannerConfig()
    plans = {"B1": _line("B1", 0.0, 0.0), "B2": _line("B2", 80.0, 0.0)}
    tok = D._SCOPE.set(D._Scope())
    try:
        seen = []
        seen.append(_same(plans, cfg))                          # 平行同向作業 80 m（> d_sep_lat 60）⇒ 無衝突
        p = plans["B2"]
        for w in p.waypoints:                                   # 就地改階段：不再是同向作業 ⇒ 門檻 100 m ⇒ 衝突
            w.phase = Phase.TRANSFER
        seen.append(_same(plans, cfg))
        for w in p.waypoints:                                   # 改回作業 ⇒ 又無衝突
            w.phase = Phase.OPERATE
        seen.append(_same(plans, cfg))
        for w in p.waypoints:                                   # 就地改航跡角（差 > 20° ⇒ 不算同向）⇒ 衝突
            w.track_deg = 45.0
        seen.append(_same(plans, cfg))
        for w in p.waypoints:                                   # 就地改 x（> d_sep 遠）⇒ 無衝突
            w.x_m += 400.0
        seen.append(_same(plans, cfg))
        p.waypoints = [dataclasses.replace(w, x_m=w.x_m - 400.0, t_s=w.t_s + 0.5) for w in p.waypoints]
        seen.append(_same(plans, cfg))                          # 換一份同長度的清單（同一個 UAVPlan 物件）⇒ 又衝突
        for w in p.waypoints:                                   # 就地改 t（錯開 300 s）⇒ 無衝突
            w.t_s += 300.0
        seen.append(_same(plans, cfg))
        for w in p.waypoints:
            w.t_s -= 300.0
        assert _same(plans, cfg) == seen[-2]
        assert all(seen[i] != seen[i + 1] for i in range(len(seen) - 1)), "每一步都要真的改變結果"
        assert len(D._SCOPE.get().tracks) > 0                   # 快取確實在作用
    finally:
        D._SCOPE.reset(tok)


def test_scope_cache_is_not_fooled_by_plan_level_edits():
    """走廊豁免看的起降站與時刻（UAVPlan 層級欄位）就地改也要重算：同站降落段在預約區間內 ⇒ 豁免；
    改站點 ⇒ 不再豁免；把 t_arr_ptc_s 往後推（區間不含那些樣本）⇒ 也不再豁免。"""
    cfg = PlannerConfig()
    plans = {"B1": _lander("B1", (0.0, 0.0), 0.0), "B2": _lander("B2", (2000.0, 2000.0), 5.0)}
    tok = D._SCOPE.set(D._Scope())
    try:
        a = _same(plans, cfg)
        plans["B2"].landing_site = "OTHER"
        b = _same(plans, cfg)
        plans["B2"].landing_site = "S"
        assert _same(plans, cfg) == a
        plans["B2"].t_arr_ptc_s += 200.0
        c = _same(plans, cfg)
        assert a != b and a != c
    finally:
        D._SCOPE.reset(tok)


def test_scope_pair_cache_does_not_leak_across_levels_or_skip_sets():
    """同一個快取範圍內，以不同細節層級（有無／SCORE／完整）與不同空間互斥集合交錯查同一對：
    每一次都要等於原版現算（repair 內 pair_clear 不帶互斥集合、involved_score 帶，兩者會查到同一對軌跡）。"""
    cfg = PlannerConfig()
    # 兩對各自同向作業 50 m < 60 ⇒ 衝突，而且兩對的事件**同一時刻**：(B1,B4) 與 (B2,B3) 依 (a, b) 與依 (b, a)
    # 排序結果不同 ⇒ 鎖住 SCORE 序列的順序（involved_score 會把它們依序加總，順序不同浮點和就可能不同）。
    plans = {"B1": _line("B1", 0.0, 0.0), "B4": _line("B4", 50.0, 0.0),
             "B2": _line("B2", 1000.0, 0.0), "B3": _line("B3", 1050.0, 0.0)}
    skip = {("B1", "B4")}                                                     # 互斥對 ⇒ 作業↔作業豁免
    full = {s: REF.check_separation(plans, cfg, None, 20.0, s) for s in (None, frozenset(skip))}
    assert {c.pair for c in full[None]} == {("B1", "B4"), ("B2", "B3")}      # 前置：兩種集合結果真的不同
    assert {c.pair for c in full[frozenset(skip)]} == {("B2", "B3")}
    assert len({c.t_s for c in full[None]}) < len(full[None])                 # 前置：確實有同時刻的事件
    expect = {}
    for s, cs in full.items():
        expect[(D._FULL, s)] = canon(cs)
        expect[(D._SCORE, s)] = canon([(c.t_s, c.uav_a, c.uav_b, c.dt_req_s) for c in cs])
        expect[(D._ANY, s)] = canon(bool(cs))
    calls = list(expect)
    rng = random.Random(3)
    for _ in range(25):
        rng.shuffle(calls)
        tok = D._SCOPE.set(D._Scope())
        try:
            for level, s in calls + calls[::-1]:
                got = D._check(plans, cfg, None, 20.0, None if s is None else set(s), 0.0, None, level)
                got = list(got) if level == D._SCORE else got
                assert canon(got) == expect[(level, s)], (level, s)
        finally:
            D._SCOPE.reset(tok)


def test_track_cache_keys_on_content_not_identity():
    cfg = PlannerConfig()
    p = _lander("B1", (0.0, 0.0), 0.0)
    tok = D._SCOPE.set(D._Scope())
    try:
        t1 = D._track_from_plan(p, cfg.dt_4d_s, 0.0, cfg)
        assert D._track_from_plan(p, cfg.dt_4d_s, 0.0, cfg) is t1           # 沒改 ⇒ 命中
        assert D._track_from_plan(copy.deepcopy(p), cfg.dt_4d_s, 0.0, cfg) is t1   # 內容相同的另一個物件也命中
        p.waypoints[10].z_m += 1.0
        t2 = D._track_from_plan(p, cfg.dt_4d_s, 0.0, cfg)
        assert t2 is not t1
        ref = REF._track_from_plan(p, cfg.dt_4d_s, 0.0, cfg)
        for f in ("samples", "heading", "oper", "corr_pad", "corr_site"):
            assert canon(getattr(t2, f)) == canon(getattr(ref, f)), f
        cfg2 = dataclasses.replace(cfg, k_t=0.04)
        assert D._track_from_plan(p, cfg2.dt_4d_s, 0.0, cfg2) is not t2      # cfg 值不同 ⇒ 另一個鍵
        assert D._track_from_plan(p, cfg.dt_4d_s, 5.0, cfg) is not t2        # t_sync 不同 ⇒ 另一個鍵
    finally:
        D._SCOPE.reset(tok)


@pytest.mark.parametrize("field", ["t_s", "x_m", "y_m", "z_m", "track_deg"])
def test_bad_waypoint_values_behave_like_reference(field):
    """快路徑用 ``np.fromiter`` 抽航點欄位，而它把 None 變成 nan（``float(None)`` 是 TypeError）：
    None 必須照原版丟同樣的例外，真的 nan 則與原版同結果（不能被當成 None、也不能被吞掉）。"""
    cfg = PlannerConfig()
    for bad, raises in ((None, TypeError), (math.nan, None)):
        outs = []
        for mod in (D, REF):
            plans = {"B1": _line("B1", 0.0, 0.0), "B2": _line("B2", 80.0, 0.0)}
            setattr(plans["B2"].waypoints[40], field, bad)
            for w in plans["B2"].waypoints:
                w.phase = Phase.TRANSFER                        # 有衝突 ⇒ nan 會影響輸出
            if raises is not None:
                with pytest.raises(raises) as ei:
                    mod.check_separation(plans, cfg)
                outs.append(str(ei.value))
            else:
                try:
                    outs.append(canon(mod.check_separation(plans, cfg)))
                except Exception as exc:                        # noqa: BLE001 — 例外也要一樣
                    outs.append((type(exc).__name__, str(exc)))
        assert outs[0] == outs[1], (field, bad)


def test_planner_config_is_frozen_because_pair_cache_keys_on_its_identity():
    with pytest.raises(dataclasses.FrozenInstanceError):
        PlannerConfig().d_sep_m = 5.0


def test_repair_with_hook_that_edits_plans_in_place_matches_reference(variants, mini, monkeypatch):
    """回呼（IDP 重排）在 repair 中途**就地**改計畫：加速版的快取不能拿到舊軌跡，結果必須與原版相同。"""
    name, plans, ghosts, skip, t_now, cfg = variants[9]
    outs, calls = [], []
    for mod in (D, REF):
        monkeypatch.setattr(mod, "_takeoff_actions", lambda c, R: [])    # 逼每筆衝突都走到回呼
        monkeypatch.setattr(mod, "_orbit_actions", lambda c, R: [])
        n = {"calls": 0, "edits": 0}

        def hook(req, n=n):
            n["calls"] += 1
            if n["edits"] < 3:
                p = req.plans[req.uav_ids[0]]
                for w in p.waypoints:
                    if w.phase not in D.SITE_CORRIDOR_PHASES:
                        w.x_m += 150.0                          # 就地改（同一個物件、同一份清單）
                n["edits"] += 1
            return None
        out, _ = _run_repair(mod, plans, ghosts, skip, t_now, cfg, mini.ctx, hook=hook)
        outs.append(out)
        calls.append(dict(n))
    assert calls[0] == calls[1] and calls[0]["edits"] >= 1, calls
    assert outs[0] == outs[1], name


def test_repair_does_not_edit_input_plans_in_place(variants, mini):
    """repair 自己不就地改任何輸入計畫（快取的前提之一；輸入只被換成新物件）。"""
    _name, plans, ghosts, skip, t_now, cfg = variants[10]
    before = canon(plans)
    plans_in = dict(plans)
    sched = D.CorridorSchedule(cfg)
    to_site = {u: p.takeoff_site or "H" for u, p in plans_in.items()}
    for u, p in plans_in.items():
        sched.reserve_plan(p, to_site[u])
    D.repair(plans_in, D.check_separation(plans_in, cfg, ghosts, 20.0, skip), mini.ctx, mini.ctx.models, sched, cfg,
             ghosts=ghosts, skip_pairs=skip, t_now_s=t_now, takeoff_site=to_site)
    assert canon(plans) == before and canon(plans_in) == before


def test_scope_is_per_call_and_per_thread(variants, mini):
    seen = {}

    def hook(req):
        seen.setdefault("inside", D._SCOPE.get())
        return None
    _name, plans, ghosts, skip, t_now, cfg = variants[0]
    _run_repair(D, plans, ghosts, skip, t_now, cfg, mini.ctx, hook=hook)
    assert D._SCOPE.get() is None
    other = []
    tok = D._SCOPE.set(D._Scope())
    try:
        th = threading.Thread(target=lambda: other.append(D._SCOPE.get()))
        th.start()
        th.join()
    finally:
        D._SCOPE.reset(tok)
    assert other == [None]                                     # 別的執行緒看不到這個範圍
    bad = {u: dataclasses.replace(p, waypoints=list(reversed(p.waypoints))) for u, p in plans.items()}
    with pytest.raises(ValueError):
        D.repair(bad, [], mini.ctx, mini.ctx.models, D.CorridorSchedule(cfg), cfg)
    assert D._SCOPE.get() is None                               # 例外也要把範圍收掉


# ---------------------------------------------------------------------------
# shift_plan、_escape
# ---------------------------------------------------------------------------

def test_shift_plan_positional_fast_path_is_active_and_identical(mini):
    assert D._WP_POSITIONAL, "Waypoint 欄位表變了：shift_plan 退回 dataclasses.replace（對但慢），請更新 _WP_FIELDS"
    p = next(iter(mini.base.values()))
    for d, k, e in ((12.5, 0, 0.0), (-3.25, 0, 0.0), (40.0, 17, 0.0), (7.0, 5, 1.5), (0.0, 0, 0.0)):
        assert canon(D.shift_plan(p, d, k, e)) == canon(REF.shift_plan(p, d, k, e))

    @dataclasses.dataclass(slots=True)
    class SubWaypoint(Waypoint):
        tag: str = "x"
    sub = dataclasses.replace(p, waypoints=[SubWaypoint(*[getattr(w, f.name) for f in dataclasses.fields(Waypoint)])
                                            for w in p.waypoints[:20]])
    got = D.shift_plan(sub, 3.0)
    assert all(type(w) is SubWaypoint for w in got.waypoints)    # 子類別走 replace，不會被降成 Waypoint
    assert canon(got) == canon(REF.shift_plan(sub, 3.0))


def test_escape_bisect_equals_linear_scan():
    rng = random.Random(7)
    k_t = 0.05
    cases = []
    for _ in range(300):
        n = rng.randint(1, 40)
        dt = np.array([rng.uniform(-900.0, 900.0) for _ in range(n)])
        win = np.array([rng.uniform(2.0, 95.0) for _ in range(n)])
        cases.append(REF._forbidden_intervals(dt, win, k_t))   # 結構化（二分搜尋會用）
    cases.append(tuple((float(i), float(i) + 0.5) for i in range(20)))
    cases.append(tuple((float(i), float(i)) for i in range(12)))                     # 退化區間
    cases.append(tuple((float(i), float(i + 2)) for i in range(12)))                 # 重疊（結構不成立 ⇒ 線性）
    cases.append(tuple((float(20 - i), float(21 - i)) for i in range(12)))           # 未排序
    cases.append(tuple((np.float64(i), np.float64(i) + 0.5) for i in range(12)))     # 非 float 型別
    cases.append(tuple((float(i), float(i) + 0.5) for i in range(5)) + ((math.nan, 9.0),) + ((10.0, 11.0),) * 3)
    cases.append([(float(i), float(i) + 0.5) for i in range(12)])                    # list（不快取）
    probes = [-1e9, -1.0, 0.0, 0.25, 0.5, 1.0, 3.0, 7.75, 11.5, 19.9, 1e9, math.inf, -math.inf, math.nan]
    tok = D._SCOPE.set(D._Scope())
    try:
        used = 0
        for iv in cases:
            lows = D._escape_lows(iv)
            used += lows is not None
            pts = probes + [x for a, b in list(iv)[:6] for x in (a, b, 0.5 * (a + b))]
            for s in pts:
                for f in ("_escape", "_escape_down"):
                    a, b = getattr(D, f)(s, iv), getattr(REF, f)(s, iv)
                    assert canon(a) == canon(b), (f, s, iv[:3])
        assert used >= 100                                      # 二分搜尋路徑確實被用到
    finally:
        D._SCOPE.reset(tok)
