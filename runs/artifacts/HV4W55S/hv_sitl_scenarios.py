"""HARVEST SITL 航次的「情境」定義。

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
    #: 生成點間距（m）。0 ⇒ 全隊共點（既有行為，逐位元不變）。
    #:
    #: ⚠⚠⚠ > 0 時每一台各有自己的起飛點，於是轉接層的 `per_uav_takeoff_sites`
    #:   會生出 n 個 `TO_<uav>` 站。**這是 vendored #8 那個缺陷的作用區間**：
    #:   走廊豁免原本比場坪座標 ≤ 50 m，而走廊排程以 site_id 為鍵 ⇒ 間距 < 50 m
    #:   時起飛與降落走廊兩層保護同時失效。修好之後 10 m 也安全（規劃層實測
    #:   最小水平間距 107.2 m），代價是 makespan +7～26%。
    #: ⚠⚠ SITL 的 `--home` 也**必須**跟著逐機分開，否則規劃說 A 從 TO_A1 起飛、
    #:   飛機卻生在 HOME —— 那是**靜默**的不一致（見 `hv_sitl_run.launch`）。
    spawn_spread_m: float = 0.0
    note: str = ""

    @property
    def fleet(self):
        return fleet_of(self.n_a, self.n_b)

    def spawn_points(self) -> list[tuple[float, float]]:
        """逐機生成點。`spawn_spread_m` = 0 時全部回 HOME（與舊版逐位元相同）。

        ⚠ 用 `utils.math_utils.spawn_positions`（GUI 的同一支），不要自己算：
          手刻 111320 m/deg 與 `EARTH_RADIUS_M` 差 0.51 m/deg，排 50 m 會量成
          49.999771 m 而掉到門檻底下。
        """
        n = len(self.fleet)
        if self.spawn_spread_m <= 0.0:
            return [HOME] * n
        from utils.math_utils import spawn_positions
        return spawn_positions(HOME[0], HOME[1], n, 0.0, float(self.spawn_spread_m))

    def drones(self) -> list[dict[str, Any]]:
        pts = self.spawn_points()
        return [{"drone_id": i + 1, "name": n, "hv_class": c, "position": pts[i],
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


# ── 同一份幾何，開全域起飛重排（vendored 修改 #3）────────────────────
#   與 gui_legal_120_split（HV4SITL9）**逐欄相同**，只換 trial ⇒ 這是一次乾淨的 A/B：
#   規劃器唯一的差別是 PlannerConfig.global_reschedule（預設已 True）。
#   ⚠ 分開 trial 是為了不覆寫 HV4SITL9 的基準產物（它是關掉重排的對照組）。
#   規劃值：makespan 1799.8 → 1450.8 s、掃描完成 1594.6 → 1216.4 s，
#   航程／條帶／航點逐位不變（只動時間戳）。這一趟要看它飛不飛得成。
_add(Scenario(
    key="gui_legal_120_split_resched", trial="HV4SITL13", path="gui", n_a=1, n_b=3,
    areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 780, 400)}],
    coverage_altitude_m=120.0, coverage_width_m=150.0, overlap_rate=0.30,
    waiver_max_agl_m=None,
    min_fw_agl_m={"A": 100.0, "B": 100.0},
    dh_band_min_m=15.0,
    target_gsd=0.05,
    note="與 HV4SITL9 同一份幾何與空域，開全域起飛重排 ⇒ 乾淨 A/B"))


# ── 四台四個起降點（間距 10 m）：vendored #8 的實飛驗證 ─────────────────
#   與 gui_legal_120_split_resched **完全相同**，只多 spawn_spread_m=10 ⇒ 乾淨 A/B。
#   ⚠⚠⚠ 為什麼是 10 m：使用者要「n 台 n 個起降點，方便快速起 SITL」，而 10 m 正好
#     把兩台 VTOL 的淨空圈（vtol_site_clear_radius_m 6–8 m）分開。10 m 也**遠低於**
#     SAME_PAD_TOL_M(50)，所以這一組落在 vendored #8 缺陷的正中央：
#     修好之前規劃層量到兩機水平 15.0 m／垂直 22.3 m 而 validate 回報通過。
#   ⚠⚠ 這是第一個 spawn_spread_m > 0 的情境 ⇒ 也是 `per_uav_takeoff_sites`
#     第一次在 SITL 真的生效（既有情境每一台的 position 都是同一個 HOME ⇒ no-op）。
_add(Scenario(
    key="gui_legal_120_split_spread10", trial="HV4SITL14", path="gui", n_a=1, n_b=3,
    areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 780, 400)}],
    coverage_altitude_m=120.0, coverage_width_m=150.0, overlap_rate=0.30,
    waiver_max_agl_m=None,
    min_fw_agl_m={"A": 100.0, "B": 100.0},
    dh_band_min_m=15.0,
    target_gsd=0.05,
    spawn_spread_m=10.0,
    note="四台四個起降點（間距 10 m）；與 HV4SITL13 同幾何 ⇒ 驗 vendored #8"))


# ── 實際機隊：1 Pioneer + 2 HERO（三台）────────────────────────────────
#   使用者 2026-09-10 定調實際要飛的就是這個機隊。與 gui_legal_120_split_spread10
#   （HV4SITL14）**逐欄相同，只把 n_b 從 3 改成 2** ⇒ 受控對照，唯一變數是少一台 HERO。
#   ⚠⚠ 為什麼不直接用作業面積 12 km²：那是另一個變數。先在已經飛過的幾何上看
#     「四台→三台」會怎麼變，壞掉才知道是機隊數造成的，不是面積造成的。
#     12 km² 的容量包絡另外量過（tests/hv_dccpp/test_fleet_1p2h_envelope.py，
#     天花板 15.50–16.00 km²），要飛那個再開一個情境。
#   ⚠ 三台是**奇數**：機隊切區會切成 3 個子區，而既有情境全是 4 的偶數切法。
_add(Scenario(
    key="gui_legal_120_1a2b", trial="HV4SITL15", path="gui", n_a=1, n_b=2,
    areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 780, 400)}],
    coverage_altitude_m=120.0, coverage_width_m=150.0, overlap_rate=0.30,
    waiver_max_agl_m=None,
    min_fw_agl_m={"A": 100.0, "B": 100.0},
    dh_band_min_m=15.0,
    target_gsd=0.05,
    spawn_spread_m=10.0,
    note="實際機隊 1 Pioneer + 2 HERO；與 HV4SITL14 同幾何同空域，只少一台 ⇒ 乾淨 A/B"))


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


# ── Beam-IDP 跳帶排序「真的觸發」的情境（論文 C3 用）───────────────────
#   ⚠ 為什麼要新開一個情境：跳帶序 `_skip_sequence(n,k)` 在 **k >= n** 時
#     逐位元等於順序來回掃描（decompose.py:689-693），所以觸發門檻是 **n > k**，
#     不是論文原本寫的「n >= 3」。gui_legal_120_1a2b 切完之後 A 胞 n=1、
#     B 胞 n=2，兩邊都 <= k（5 與 3）⇒ 那一趟**從來沒有觸發過**跳帶排序。
#   這一組把每個胞撐到 n > k：實測 A 胞 n=6/7（k=5）、B 胞 n=6/7/13（k=3）。
#
#   與 gui_legal_120_1a2b 的差別**只有幾何**（機隊、風、起降點間距、
#   min_fw、高度帶、GSD 全部相同），高度上限則依 C1 的結論降到 119.5 m
#   （coverage_altitude_m 與 waiver_max_agl_m 一起給 ⇒ h_ceil=119.5，
#    匯出的最高點 119.55 m、實飛預估 < 120 m）。
#   ⚠ 幾何必須是「三塊各 1600 x 1200 m」而不是一塊大區：
#     split_area_for_fleet 沿掃描步進軸把母區切成 n_uav 塊，子區再依構型切，
#     A 大約只拿到該子區的一半條帶 ⇒ 單一大區要 A 胞 n>=6 得 3000 x 3000 m
#     (9.00 km²、makespan 3842.6 s)；切成三塊只要 5.76 km²、makespan 2348.0 s。
#     實測淘汰過：三塊 1560x800（A 胞 n=3,3,4）、三塊 1200x1000（A 胞 n=5,5）。
#   ⚠ trial 故意不取 HV4SITL17，避免與 C1「降上限重飛」那一趟搶編號。
_add(Scenario(
    key="beamidp_1a2b_1195", trial="BEAMIDP1", path="gui", n_a=1, n_b=2,
    areas=[{"area_id": i + 1, "priority": 1.0,
            "polygon": rect(-2100.0 + i * 2100.0, 0.0, 800.0, 600.0)}
           for i in range(3)],
    coverage_altitude_m=119.5, coverage_width_m=150.0, overlap_rate=0.30,
    waiver_max_agl_m=119.5,
    min_fw_agl_m={"A": 100.0, "B": 100.0},
    dh_band_min_m=15.0,
    target_gsd=0.05,
    spawn_spread_m=10.0,
    note="Beam-IDP 跳帶排序觸發情境：三塊 1600x1200 m（5.76 km²）、1 Pioneer + 2 HERO、"
         "上限 119.5 m。每胞 n > k（A 6/7 > 5、B 6/7/13 > 3）⇒ 跳帶排序真的作用"))
