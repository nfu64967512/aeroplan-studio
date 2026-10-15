"""實際機隊 **1 Pioneer + 2 HERO** 的作業容量包絡（值層級鎖）。

使用者 2026-09-10 定調：真正要飛的機隊是 **1 台 Pioneer（構型 A）+ 2 台 HERO（構型 B）**，
單趟任務面積 **12 km² 以內**。這一檔把「這個機隊在這個面積確實交得出乾淨計畫」鎖成值。

為什麼鎖的是**值**不是**源碼**
------------------------------
只斷言「某一行長什麼樣」的鎖，改上游、改下游、繞過去都免疫（補鎖是軍備競賽）。
這一檔一律斷言**規劃結果的數字**：幾台真的有航點、漏蓋幾個、衝突幾筆、每一區覆蓋率多少。

⚠⚠⚠ 為什麼一定要先斷言「3 台都真的在飛」
------------------------------------------
`mission/swarm_coordinator.py` 自己記著一個實測失效：

    單一大區配混編機隊會產出**空計畫**（殘帶對拍賣把母胞鎖成「A 一台 + B 一台」，
    胞太少餵不飽機隊，加機也沒用）。實測 20 km²、1 Pioneer + 3 HERO：整塊 0/4 台。

**空計畫的 `uncovered` 和 `conflicts` 都是 0** —— 只看那兩個數字，最嚴重的失效會顯示成滿分。
所以 :func:`test_operational_area_is_covered_by_the_real_fleet` 先鎖「3 台都有航點」，
再鎖覆蓋與衝突；少了前者，後面兩條是空語句。

實測基準（2026-09-10，產品路徑 `SwarmCoordinator`，本檔 `_rect_area` 的 4:3 矩形）
-----------------------------------------------------------------------------------
=========  ========  =========  =========  ======  ========  ==================================
面積       每機      passed     漏蓋       衝突    makespan  逐機航點
=========  ========  =========  =========  ======  ========  ==================================
12 km2     4.00 km2  **True**   0          0       3274.1 s  A1=914, B2=622, B3=771（3/3 在飛）
20 km2     6.67 km2  False      1（Z4_1）  0       3566.5 s  A1=1104, B2=620, B3=594（3/3 在飛）
=========  ========  =========  =========  ======  ========  ==================================

20 km2 那一列 `Z4` 的覆蓋率是 **0.0000**：機隊切區把整塊切成 Z2/Z3/Z4，
前兩個蓋滿，第三個完全拿不到任何條帶（`no_feasible_insertion`）。

容量天花板（二分，`scratchpad/hv_envelope_bisect.py`，同一條產品路徑）：
最大通過 **15.50 km2**（5.17 km2/機），最小失效 **16.00 km2**（5.33 km2/機）
⇒ 12 km2 的作業點有 **+29.2%** 餘裕。

⚠ 天花板**與形狀有關**，不是只看面積。上表用 4:3；基準情境 S1 是 5:4（5000x4000），
同樣 20 km2 在 S1 形狀下除了漏蓋還多一筆 HERO-HERO 4D 衝突
（B2-B3，水平 97.6 m、垂直 40.0 m，還差 +1.71 s），改成 4:3 就沒有那筆。
⇒「20 km2 會出現衝突」**不可**當普遍結論；普遍的只有「會失效」。

⚠ HERO–HERO 沒有垂直分層可用
-----------------------------
`mission.py` 的 `h_transit_offset_A_m = 50.0` 只加在構型 **A**，構型 B 是 0；
而垂直隔離門檻 `dh_sep_m` 也是 **50.0**。⇒ 兩台 HERO 依設計就同高，
只能靠水平與時間隔離。1 Pioneer + 2 HERO 裡唯一的同構型機對就是 HERO-HERO ⇒
**這個機隊的 4D 衝突主要就是這一型，不是邊角情況。**
2026-09-10 量到的兩筆 4D 衝突都是同構型機對：S1 形狀 20 km2 的 1P+2H（B2-B3），
以及封閉基準 `HARVEST_mix S3 n=4`（B3-B4，覆蓋率 1.0 卻 validate FAIL）。
"""

from __future__ import annotations

import logging
import math

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

from shapely.geometry import Polygon                                    # noqa: E402

from hv_dccpp.integration.mqtt_bridge import PLATFORM_KEY_BY_CLS        # noqa: E402
from hv_dccpp.mission import PlannerConfig                              # noqa: E402
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER           # noqa: E402

# 基準情境常數（與 research/cpp_benchmark/matrix.py 同源，複製而非匯入：
# tests/ 不該相依於 research/ 的執行環境）。
HOME_LL = (23.95, 120.90)
M_LAT = 111132.0
M_LON = 111412.0 * math.cos(math.radians(HOME_LL[0]))
ALT, HFOV, VFOV = 140.0, 60.0, 45.0
SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)
PITCH = SWATH * 0.7

# 作業包絡：使用者定調單趟 ≤ 12 km²。超過此值即為包絡外，見 test_..._checks_are_not_inert。
OPERATIONAL_AREA_KM2 = 12.0
BEYOND_ENVELOPE_KM2 = 20.0


def _ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME_LL[0] + north_m / M_LAT, HOME_LL[1] + east_m / M_LON)


def _rect_area(km2: float, aspect: float = 4.0 / 3.0) -> list[dict]:
    """以 HOME 為中心、面積 `km2` 的矩形作業區（長寬比同 S1/S4 的 4:3）。"""
    half_n = math.sqrt(km2 * 1e6 / aspect) / 2.0
    half_e = half_n * aspect
    pts = [(-half_e, -half_n), (half_e, -half_n), (half_e, half_n), (-half_e, half_n)]
    return [{"area_id": 1, "priority": 1.0, "polygon": [_ll(x, y) for x, y in pts]}]


def _fleet_1p2h() -> tuple[list[dict], dict[int, str]]:
    """實際機隊：1 台 Pioneer（A）+ 2 台 HERO（B）。

    速度與迴轉半徑取自 :data:`PLATFORM_PIONEER` / :data:`PLATFORM_HERO`，
    不寫死數字 —— 平台表被改動時本測試要跟著動，而不是繼續綠燈。
    """
    spec = [("A", PLATFORM_PIONEER), ("B", PLATFORM_HERO), ("B", PLATFORM_HERO)]
    drones = [{"drone_id": i + 1, "name": f"{c}{i + 1}", "position": HOME_LL,
               "speed": p.cruise_airspeed_mps, "heading": 0.0,
               "turn_radius": p.turn_radius_planning_m,
               "vehicle_type": "vtol", "fov_width": PITCH}
              for i, (c, p) in enumerate(spec)]
    return drones, {i + 1: c for i, (c, _p) in enumerate(spec)}


def _plan(km2: float, hv_cfg: dict | None = None):
    """走**產品路徑**（`SwarmCoordinator`）規劃一次，回傳 (validation, plan)。

    ⚠⚠ 不可改走 `HvDccppCoordinator` 直呼。轉接層預設 `sensor_a/sensor_b=None`
    時 `_hardware_for` 會**照構型字母**給 A→lidar、B→rgb，憑空生出一個酬載差異；
    產品路徑則明確把同一台單光相機掛給兩個構型，並且會先跑 `split_area_for_fleet`。
    實測（2026-09-10）：同樣 20 km² 的 1P+2H，轉接層直呼是 passed=True，
    產品路徑是 passed=False —— 兩條路徑量的**不是同一個問題**。
    """
    from mission.swarm_coordinator import SwarmCoordinator

    logging.disable(logging.CRITICAL)
    try:
        drones, cls = _fleet_1p2h()
        c = SwarmCoordinator()
        c.use_harvest = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME_LL
        c.hv_wind = (5.0, 270.0)
        c.hv_fence_zones = []
        c.harvest_params = {"hfov_deg": HFOV, "vfov_deg": VFOV, "hv_c2_range_km": 25.0,
                            "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
        if hv_cfg:
            # swarm_coordinator.py:1918 會把 hv_cfg 轉給 to_planner_config ⇒ 可覆寫任意欄位
            c.harvest_params["hv_cfg"] = dict(hv_cfg)
        c.plan_coverage_dccpp(areas=_rect_area(km2), drones=drones,
                              coverage_width_m=PITCH, overlap_rate=0.0,
                              coverage_altitude=ALT, mounting_angle_deg=0.0)
        return c.hv_validation, c.hv_coordinator.last_plan
    finally:
        logging.disable(logging.NOTSET)


# ---------------------------------------------------------------------------
# 機隊身分：Pioneer 是 A、HERO 是 B
# ---------------------------------------------------------------------------

def test_pioneer_is_class_a_and_hero_is_class_b() -> None:
    """構型字母 ↔ 平台鍵的對應被改掉時，上面所有面積數字都失去意義。"""
    assert PLATFORM_KEY_BY_CLS["A"] == "pioneer_pnp"
    assert PLATFORM_KEY_BY_CLS["B"] == "hero_pnp"
    assert PLATFORM_PIONEER.cls == "A"
    assert PLATFORM_HERO.cls == "B"


def test_the_two_platforms_really_are_heterogeneous() -> None:
    """異質性的驅動量是**迴轉半徑**（論文主張 1）。兩者相同時整個分配故事就沒了。"""
    assert PLATFORM_PIONEER.turn_radius_planning_m == pytest.approx(200.0)
    assert PLATFORM_HERO.turn_radius_planning_m == pytest.approx(120.0)
    assert PLATFORM_PIONEER.turn_radius_planning_m > PLATFORM_HERO.turn_radius_planning_m
    # HERO 腿短：續航與可用航程都比 Pioneer 小 —— 容量天花板由它決定。
    assert PLATFORM_HERO.reference_endurance_min < PLATFORM_PIONEER.reference_endurance_min
    assert PLATFORM_HERO.usable_distance_km[1] < PLATFORM_PIONEER.usable_distance_km[0]


def test_hero_pairs_have_no_vertical_separation_channel() -> None:
    """兩台 HERO 依設計同高：過境層偏移只加在 A，而它恰好等於垂直隔離門檻。

    ⚠ 這一條**不是**在鎖「現況是對的」，是在鎖「這個結構事實還在」。
    `h_transit_offset_A_m` 一旦不再等於 `dh_sep_m`，A–B 的垂直餘裕就從零變成有，
    上面那筆 HERO–HERO 衝突的成因敘述也要跟著重寫。
    """
    cfg = PlannerConfig()
    assert cfg.h_transit_offset_A_m == pytest.approx(cfg.dh_sep_m), (
        "A 的過境層偏移與垂直隔離門檻不再相等 —— A–B 的垂直餘裕變了，"
        "請重新量測 1P+2H 的衝突型態並更新本檔 docstring 的實測表。")
    # 構型 B 沒有任何偏移 ⇒ HERO–HERO 的垂直餘裕是 0，只能靠水平與時間。
    assert PLATFORM_HERO.min_fw_operation_agl_m < PLATFORM_PIONEER.min_fw_operation_agl_m


# ---------------------------------------------------------------------------
# 容量包絡
# ---------------------------------------------------------------------------

@pytest.mark.slow
def test_operational_area_is_covered_by_the_real_fleet() -> None:
    """12 km²（使用者定調的作業面積）：3 台都在飛、零漏蓋、零衝突、validate 通過。"""
    v, plan = _plan(OPERATIONAL_AREA_KM2)

    # (1) 先證明這份計畫**不是空的** —— 否則下面兩條是空語句（見檔頭 ⚠⚠⚠）。
    per_uav = {u: len(p.waypoints) for u, p in plan.plans.items()}
    assert len(per_uav) == 3, f"應有 3 台，實得 {len(per_uav)}：{per_uav}"
    assert all(n > 0 for n in per_uav.values()), (
        f"有機體拿到空計畫（殘帶對拍賣餵不飽機隊的已知失效）：{per_uav}")
    assert plan.makespan_s > 0.0

    # (2) 覆蓋：規劃器自己不得宣告任何漏蓋，且每一子區的 'all' 覆蓋率為 1。
    assert list(plan.uncovered or ()) == [], f"漏蓋：{list(plan.uncovered or ())}"
    assert v.coverage_ok is True
    for line in v.details.get("coverage", ()):
        assert "FAIL" not in line, f"子區覆蓋未過：{line}"

    # (3) 時空隔離與能量。
    assert list(v.conflicts) == [], f"4D 衝突：{[(c.uav_a, c.uav_b, c.dt_req_s) for c in v.conflicts]}"
    assert v.energy_ok is True
    assert v.passed is True


@pytest.mark.slow
def test_envelope_checks_are_not_inert() -> None:
    """包絡**外**（20 km²）同一套檢查必須真的響 —— 否則上一條測不到任何東西。

    ⚠ 這條紅了不代表壞掉：若哪天 20 km² 也能乾淨交付（例如接上多趟出動／換電），
    那是**進步**。此時要做的是把 `OPERATIONAL_AREA_KM2` / `BEYOND_ENVELOPE_KM2`
    往上調並重量一次上表，**不是**把這條測試刪掉。
    """
    v, plan = _plan(BEYOND_ENVELOPE_KM2)

    fired = {
        "uncovered": len(plan.uncovered or ()) > 0,
        "coverage_ok": v.coverage_ok is False,
        "conflicts": len(v.conflicts) > 0,
        "passed": v.passed is False,
    }
    assert any(fired.values()), (
        f"{BEYOND_ENVELOPE_KM2} km² 也全綠 ⇒ 這些檢查偵測不到任何失效，"
        f"test_operational_area_is_covered_by_the_real_fleet 因此無鑑別力。detail={fired}")
    # 已知形態（4:3、20 km2）：機隊切區的第三個子區完全拿不到條帶 ⇒ coverage_ok=False。
    # 衝突不是每個形狀都會出現（見檔頭），所以這裡只鎖 passed，不鎖 conflicts。
    assert fired["passed"], f"validate 應為 False，實得 passed=True（{fired}）"


# ---------------------------------------------------------------------------
# 不變式：蓋不完就一定要大聲說
# ---------------------------------------------------------------------------

def _strips_flown(plan) -> tuple[int, int]:
    """(實際被排進計畫的條帶數, 總條帶數)。"""
    flown: set[str] = set()
    for p in plan.plans.values():
        for w in p.waypoints:
            sid = getattr(w, "strip_id", None)
            if sid:
                flown.add(str(sid))
        for sid in (getattr(p, "strip_ids", None) or ()):
            flown.add(str(sid))
    return len(flown), len(plan.strips or {})


@pytest.mark.slow
@pytest.mark.parametrize("multi_sortie", [False, True])
def test_incomplete_coverage_is_never_silent(multi_sortie: bool) -> None:
    """**使用者硬性要求**：靜默失敗不可發生。蓋不完 ⇒ `uncovered` 非空且 `validate` 不通過。

    ⚠⚠⚠ 這條是**地雷偵測器**，鎖的不是現況而是不變式。

    現況（2026-09-10 實測）：`cfg.multi_sortie` 是**結構性死碼**，開或關逐位相同。
    `allocate.phase2_sorties` 第一道篩是

        pending = [(k, r) for (k, r) in self.uncoverable
                   if r == "no_feasible_insertion" and k in self.tasks]

    而 `uncoverable` 存的是**裸胞 id**（`'Z4_1'`），`self.tasks` 的鍵卻帶構型後綴
    （`'Z4_1#A'`、`'Z4_1#B'`）⇒ `k in self.tasks` 恆為 False ⇒ `pending` 恆空 ⇒
    立刻 return，**一台虛擬第二架次機都不會被建出來**。

    ⚠ 為什麼**不可以**只補這個鍵比對：補好之後虛擬 id（`A1#s1`）會拿到指派，
    `phase2_sorties` 隨即把那些 `uncoverable` 條目清掉（allocate.py:1252），
    但 `planner.py:846` 的 `u in init_states` 與 `planner.py:1489` 的
    `if not team_init: continue` 會把只含虛擬 id 的隊伍整支丟掉 ——
    結果就是 `uncovered` 空的、條帶卻少一半：**大聲的失敗變成安靜的失敗**。
    兩處要一起修，這條測試就是為了在只修一半時變紅。
    """
    v, plan = _plan(BEYOND_ENVELOPE_KM2, hv_cfg={"multi_sortie": multi_sortie})
    flown, total = _strips_flown(plan)

    # 先證明這個設定真的蓋不完 —— 否則下面的蘊含式是空語句。
    assert total > 0, "沒有條帶，這條測試量不到任何東西"
    assert flown < total, (
        f"{BEYOND_ENVELOPE_KM2} km² 竟然全蓋到了（{flown}/{total}）⇒ 包絡已改變，"
        f"請重新設定 BEYOND_ENVELOPE_KM2，本條測試目前無鑑別力")

    # 蓋不完 ⇒ 必須大聲。兩個管道都要響。
    assert list(plan.uncovered or ()), (
        f"靜默漏蓋：條帶只飛到 {flown}/{total}，而 MissionPlan.uncovered 是空的"
        f"（multi_sortie={multi_sortie}）")
    assert v.passed is False, (
        f"靜默漏蓋：條帶只飛到 {flown}/{total}，而 validate 回報 passed=True"
        f"（multi_sortie={multi_sortie}）")
