"""HV-DCCPP SITL 航次的「情境」定義。

一個情境 = 機隊 + 作業區 + 規劃路徑 + 高度。`scripts/hv_sitl_flight.py` 以
環境變數 ``HV_SCENARIO`` 選一個來跑，預設 ``lidar_rgb_1a3b``（HV4SITL1/2 那一組）。

兩條規劃路徑刻意都保留：

``path='gui'``
    走 ``mission.swarm_coordinator.SwarmCoordinator``，也就是 AeroPlan 介面按下去
    真正會走的那一條。它會用 ``hv_uniform_sensor``（預設 True）讓兩個構型
    共用同一台單光相機，酬載不再是分工依據。
``path='direct'``
    直接建 ``HvDccppCoordinator`` 並自己給 ``payload_reqs``，也就是
    HV4SITL1/2 用的那一條。可以指定多個需求（不同高度、不同 kind）。

⚠ 兩條路徑會產出**不同的航線**。同一份幾何下 GUI 路徑的刈幅由面板 FOV 決定，
  direct 路徑由 PayloadRequirement 決定，條帶數與 makespan 都會不一樣。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Literal

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))


def ll(east_m: float, north_m: float) -> tuple[float, float]:
    """以 HOME 為原點的 ENU 位移轉經緯度（平面近似，數公里內夠用）。"""
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def rect(cx: float, cy: float, half_e: float, half_n: float) -> list[tuple[float, float]]:
    return [ll(cx - half_e, cy - half_n), ll(cx + half_e, cy - half_n),
            ll(cx + half_e, cy + half_n), ll(cx - half_e, cy + half_n)]


# 每個構型的 (巡航 m/s, 規劃轉彎半徑 m)；值取自 hv_dccpp.platform 的 PlatformSpec。
_PERF = {"A": (21.5, 200.0), "B": (19.0, 120.0)}


def fleet_of(n_a: int, n_b: int) -> list[tuple[str, str, int, float, float]]:
    """(name, cls, sysid, cruise_mps, turn_radius_m)；sysid 依序 1..n。"""
    out = []
    for i, cls in enumerate(["A"] * n_a + ["B"] * n_b):
        v, r = _PERF[cls]
        out.append((f"{cls}{i + 1}", cls, i + 1, v, r))
    return out


@dataclass
class Scenario:
    key: str
    trial: str
    path: Literal["gui", "direct"]
    n_a: int
    n_b: int
    areas: list[dict[str, Any]]
    coverage_altitude_m: float
    coverage_width_m: float
    overlap_rate: float
    hfov_deg: float = 60.0
    vfov_deg: float = 45.0
    wind: tuple[float, float] = (5.0, 270.0)
    payload_reqs: dict[str, dict[str, Any]] = field(default_factory=dict)
    # 禁航區：[[(lat, lon), ...], ...]。走 gui 路徑時轉成 FenceZone 餵給規劃器。
    nfz: list[list[tuple[float, float]]] = field(default_factory=list)
    # ── 空域與平台的可調量（只有 gui 路徑會用）──────────────────────
    #   waiver_max_agl_m=None ⇒ **沒有豁免**，AGL 上限就是民航法規的 120 m。
    #   min_fw_agl_m ⇒ 逐構型覆寫最低固定翼作業高度（表 1 的 120/100 是規劃值，
    #     不是機體極限；下界是各自的轉換完成高度上緣 A 100 / B 90）。
    #   dh_band_min_m ⇒ 高度帶下限（規格預設 20）。
    #   target_gsd ⇒ 目標地面解析度；不給的話轉接層會合成 1.05×航高。
    waiver_max_agl_m: float | None = 200.0
    min_fw_agl_m: dict[str, float] | None = None
    dh_band_min_m: float | None = None
    target_gsd: float | None = None
    note: str = ""

    @property
    def fleet(self):
        return fleet_of(self.n_a, self.n_b)

    def drones(self) -> list[dict[str, Any]]:
        return [{"drone_id": i + 1, "name": n, "hv_class": c, "position": HOME,
                 "speed": v, "heading": 0.0, "turn_radius": r,
                 "vehicle_type": "vtol", "fov_width": self.coverage_width_m}
                for i, (n, c, _sysid, v, r) in enumerate(self.fleet)]


# ══════════════════════════════════════════════════════════════════════
#  情境
# ══════════════════════════════════════════════════════════════════════
SCENARIOS: dict[str, Scenario] = {}


def _add(s: Scenario) -> Scenario:
    SCENARIOS[s.key] = s
    return s


# HV4SITL1/2 飛過的那一組：LiDAR 區給 Pioneer、RGB 區給三架 HERO。
# 保留它是為了讓新情境有可比的基準線。
_add(Scenario(
    key="lidar_rgb_1a3b", trial="HV4SITL2", path="direct", n_a=1, n_b=3,
    coverage_altitude_m=160.0, coverage_width_m=224.0, overlap_rate=0.25,
    areas=[
        {"area_id": 1, "priority": 1.0, "sensor_req": "lidar", "payload_req_id": "lidar_req",
         "polygon": [(23.944131, 120.906876), (23.944131, 120.910806),
                     (23.947742, 120.910806), (23.947743, 120.906877)]},
        {"area_id": 2, "priority": 1.0, "sensor_req": "rgb", "payload_req_id": "rgb_req",
         "polygon": [(23.952256, 120.885264), (23.952257, 120.893123),
                     (23.957313, 120.893123), (23.957312, 120.885263)]},
    ],
    payload_reqs={
        "lidar_req": dict(kind="lidar", density_pt_m2=20.0, side_overlap=0.25,
                          h_target_agl_m=160.0),
        "rgb_req": dict(kind="rgb", gsd_required_m=0.05, side_overlap=0.30,
                        forward_overlap=0.70, h_target_agl_m=100.0),
    },
    note="基準線：異構酬載，兩型飛在不同高度（Pioneer 169.6 m / HERO 100 m）"))


def _grid(n: int, half_e: float, half_n: float, pitch: float, row: float = 0.0):
    """n 個並排的矩形作業區，中心間距 pitch。"""
    x0 = -pitch * (n - 1) / 2.0
    return [{"area_id": i + 1, "priority": 1.0,
             "polygon": rect(x0 + i * pitch, row, half_e, half_n)}
            for i in range(n)]


# 以下三組的幾何是 scripts/hv_sitl_scout.py 掃出來的：四架都有工作、未覆蓋 0、
# makespan ≤ 2600 s（speedup 5 ⇒ 牆鐘 ≤ 8.7 分鐘）。
# ⚠ 用到幾台是**胞的數目**決定的，不是機隊大小，所以幾何不能隨便改。

_add(Scenario(
    key="gui_uniform_1a3b", trial="HV4SITL3", path="gui", n_a=1, n_b=3,
    areas=_grid(4, 260, 300, 1500), coverage_altitude_m=140.0,
    coverage_width_m=150.0, overlap_rate=0.30,
    note="GUI 路徑 + 單光相機，機隊與 HV4SITL2 相同 ⇒ 可直接對照航線差異"))

_add(Scenario(
    key="gui_uniform_2a2b", trial="HV4SITL4", path="gui", n_a=2, n_b=2,
    areas=(_grid(2, 300, 350, 1600)
           + [{"area_id": 3, "priority": 1.0, "polygon": rect(-800, 2000, 300, 350)},
              {"area_id": 4, "priority": 1.0, "polygon": rect(800, 2000, 300, 350)}]),
    coverage_altitude_m=150.0, coverage_width_m=150.0, overlap_rate=0.30,
    note="兩台 Pioneer 兩台 HERO，作業區排成 2×2 ⇒ 沒飛過的機隊組成"))

_add(Scenario(
    key="gui_uniform_3a1b", trial="HV4SITL5", path="gui", n_a=3, n_b=1,
    areas=_grid(4, 320, 380, 1700), coverage_altitude_m=150.0,
    coverage_width_m=150.0, overlap_rate=0.30,
    note="三台 Pioneer 一台 HERO；規劃把 24.5 km 的長段丟給那台 HERO，"
         "能量餘裕只剩 133.9 Wh ⇒ 看真機飛不飛得完"))



_add(Scenario(
    key="gui_nfz_1a3b", trial="HV4SITL7", path="gui", n_a=1, n_b=3,
    areas=_grid(4, 260, 300, 1500), coverage_altitude_m=140.0,
    coverage_width_m=150.0, overlap_rate=0.30,
    # 禁航區釘在「沒有 NFZ 的版本**確實會飛過去**」的那條轉場腿上（實測穿越 368 m）。
    # ⚠ 選位置踩過三個坑，記著免得重來：
    #   1. 放在沒人飛過的地方 ⇒ 計畫與無 NFZ 版**逐位元相同**，等於沒測到東西。
    #   2. 壓在作業區上 ⇒ 蓋住該區 ≥42.9% 時 HARVEST 把**其他不相干的區也一起丟掉**
    #      （4 區全部 no_feasible_insertion、0 台上工）；≤28% 時只掉一台。
    #      作業區內的避繞要等胞分解接進來才有得測。
    #   3. 太靠近 home ⇒ `NO_GO: descent_orbit_blocked(H)`（正確行為，但測不到航線）。
    nfz=[rect(-1132, 174, 150, 150)],
    note="與 gui_uniform_1a3b 完全相同，只多一個轉場走廊上的禁航區 ⇒ 可對照 HV4SITL3"))


# ── 民航法規 120 m AGL 上限（無豁免）─────────────────────────────────
#   幾何與機隊刻意與 gui_uniform_1a3b（HV4SITL3）一模一樣，只改空域與平台的
#   四個量，這樣兩趟可以直接對照：
#       HV4SITL3: 航高 140、AGL 上限 200（假設有豁免）、min_fw 表值 120/100、帶 20
#       HV4SITL8: 航高 120、AGL 上限 **無豁免 ⇒ 法規 120**、min_fw 100/100、帶 15
#   ⚠ 表值那一組在 h_ceil=120 下是 NO_GO（Pioneer 高度帶 0、HERO 20−起伏）；
#     依據見 docs/adr/0012-min-fw-altitude-is-adjustable.md。
_add(Scenario(
    key="gui_legal_120", trial="HV4SITL8", path="gui", n_a=1, n_b=3,
    areas=_grid(4, 260, 300, 1500), coverage_altitude_m=120.0,
    coverage_width_m=150.0, overlap_rate=0.30,
    waiver_max_agl_m=None,
    min_fw_agl_m={"A": 100.0, "B": 100.0},
    dh_band_min_m=15.0,
    target_gsd=0.05,
    note="法規上限 120 m AGL、無豁免。驗證 Pioneer 在合法高度下真的飛得起來"))


# ── 模式一：使用者只畫**一大塊**，由 split_area_for_fleet 自動切 ────────
#   面積刻意與 gui_legal_120 的四小塊總和相同（1.248 km² = 1560 × 800 m），
#   空域與平台條件也完全相同 ⇒ 兩趟的差別只有「一大塊自動切」vs「四塊分開畫」。
#   ⚠ 單一大區配混編機隊會產出**空計畫**（殘帶對拍賣把每個母胞鎖成 A+B 各一台），
#     所以 SwarmCoordinator 會先切；這一趟就是要看那條路在法規上限下飛不飛得成。
_add(Scenario(
    key="gui_legal_120_split", trial="HV4SITL9", path="gui", n_a=1, n_b=3,
    areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 780, 400)}],
    coverage_altitude_m=120.0, coverage_width_m=150.0, overlap_rate=0.30,
    waiver_max_agl_m=None,
    min_fw_agl_m={"A": 100.0, "B": 100.0},
    dh_band_min_m=15.0,
    target_gsd=0.05,
    note="模式一：一大區（1.248 km²）自動切給 1A+3B，法規 120 m 上限無豁免。"
         "與 gui_legal_120 面積相同 ⇒ 可對照「自動切」vs「分開畫」"))


# ── 1 Pioneer + 3 HERO：禁航區 × 法規上限（兩件事第一次一起飛）────────
#   幾何、機隊、禁航區與 gui_nfz_1a3b（HV4SITL7）完全相同，只把空域換成
#   「無豁免 ⇒ 法規 120 m」那一組 ⇒ 可與 HV4SITL7（140 m、假設有 200 m 豁免）對照。
#   ⚠ 這一趟會同時走兩條切區路徑：先 split_area_for_fleet 再 split_areas_around_nfz。
#     禁航區子區的 id 從 "3_1" 這種字串改成整數（轉接層 int() 會把底線吃掉變 31 撞號）
#     之後**還沒有飛過**，這是第一次上天。
_add(Scenario(
    key="gui_legal_120_nfz", trial="HV4SITL12", path="gui", n_a=1, n_b=3,
    areas=_grid(4, 260, 300, 1500), coverage_altitude_m=120.0,
    coverage_width_m=150.0, overlap_rate=0.30,
    nfz=[rect(-1132, 174, 150, 150)],
    waiver_max_agl_m=None,
    min_fw_agl_m={"A": 100.0, "B": 100.0},
    dh_band_min_m=15.0,
    target_gsd=0.05,
    note="1 Pioneer + 3 HERO，禁航區 + 法規 120 m AGL 無豁免。"
         "與 HV4SITL7 同幾何同禁航區，只差空域 ⇒ 可直接對照"))
