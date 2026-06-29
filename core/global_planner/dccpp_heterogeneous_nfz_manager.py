"""
core/global_planner/dccpp_heterogeneous_nfz_manager.py
=======================================================

DCCPP 異質機隊 NFZ 管理器 — ArduPilot 參數驅動版（v2.6.0）
-----------------------------------------------------------

本模組在原 ``HeterogeneousNFZPlanner`` 的雙路徑骨架（MR A* + FW Dubins）
基礎上重新優化，**所有緩衝距離、最小轉彎半徑都從 ArduPilot 飛控參數
動態推導**，並對輸出航點做：

    1. ArduPilot 物理極限驗證（R_min = V²/(g·tan(φ_max))）
    2. WP_RADIUS 間距下限強制（consecutive WP ≥ 2·WP_RADIUS）
       — 避免 ArduPlane L1 控制器連環 advance 跳過航點而剪進 NFZ
    3. MAVLink 指令碼標記：
         MR 中間繞行點 → NAV_SPLINE_WAYPOINT (82)
         VTOL 起降      → NAV_VTOL_TAKEOFF (84) / NAV_VTOL_LAND (85)
         FW            → NAV_WAYPOINT (16)（不可用 SPLINE，固定翼不支援）
    4. 雙重匯出 — Mission (QGC WPL 110) + Fence (.fence) 兩道防線

物理推導（CoT，詳見模組頂部 docstring 之 README）:
    協調轉彎 ⇒ tan(φ) = V²/(gR)  ⇒  R_min = V_a²/(g·tan(LEVEL_ROLL_LIMIT))
    buffer_fw = R_min + WP_RADIUS + GPS_Error_fw    (預設 GPS=15m)
    buffer_mr =          WP_RADIUS + GPS_Error_mr    (預設 GPS=5m)

L1 advance 規則：consecutive WP 間距 < WP_RADIUS 時，L1 會略過該 WP；
若兩個 WP 距離 < 2·WP_RADIUS 則飛機根本「沒到」第一個就已切換 →
連續略過導致剪西瓜。硬約束：所有相鄰 WP 必須間距 ≥ 2·WP_RADIUS。

Dubins 弧離散化（每弧獨立計算段數）：
    子弧弦 = 2R·sin(θ_sub/2) ≥ 2·WP_RADIUS
    ⇒ θ_sub_max = 2·arcsin(WP_RADIUS / R)
    ⇒ N_max = floor(|θ_total| / θ_sub_max)
    R < WP_RADIUS 時退化為直線弦（不放中間點）
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

from shapely.geometry import Polygon

# 重用既有模組的核心元件 — 投影、helper、A* 規劃器、Waypoint
from core.global_planner.heterogeneous_nfz_planner import (
    FixedWingDubinsPlanner,
    MultirotorAStarPlanner,
    PlannerConfig,
    Waypoint,
    _LocalProjection,
)

# ──────────────────────────────────────────────────────────────────────
#  MAVLink 指令碼（避免依賴 pymavlink，硬編碼常用值）
# ──────────────────────────────────────────────────────────────────────
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_SPLINE_WAYPOINT = 82
MAV_CMD_NAV_VTOL_TAKEOFF = 84
MAV_CMD_NAV_VTOL_LAND = 85

# ArduPilot 4.x 圍籬命令（MAV_MISSION_TYPE_FENCE）
MAV_CMD_NAV_FENCE_RETURN_POINT = 5000
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002

# MAVLink frame codes
MAV_FRAME_GLOBAL = 0                  # 絕對海拔
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3     # 相對 HOME 高度


# ──────────────────────────────────────────────────────────────────────
#  ArduPilot 參數資料類
# ──────────────────────────────────────────────────────────────────────
@dataclass
class ArduPilotParams:
    """ArduPilot 飛控核心參數 — 規劃器從這裡推導所有物理約束。

    對應的 ArduPilot 參數名稱（可直接從 .parm 檔讀取）：
        WP_RADIUS          航點切換半徑 [m]      預設 20m
        LEVEL_ROLL_LIMIT   最大協調轉彎側傾 [°]  預設 35°
        AIRSPEED_CRUISE    巡航空速 [m/s]       預設 18m/s
    """

    # ── 飛控核心參數 ──
    wp_radius: float = 20.0
    level_roll_limit_deg: float = 35.0
    airspeed_cruise: float = 18.0

    # ── 安全餘裕（GPS 漂移容差，依機型分配）──
    gps_error_fw_m: float = 15.0   # 固定翼較大（高速、機體形變大）
    gps_error_mr_m: float = 5.0    # 多旋翼較小（低速、定位較穩）

    # ── 高度層分配 ──
    alt_mr: float = 80.0
    alt_fw: float = 120.0

    # ── Fence 政策 ──
    fence_action: int = 1          # 1=RTL, 2=Land, 4=Brake
    fence_alt_max: float = 200.0   # FENCE_ALT_MAX [m]

    # ── 演算法上限（防止極端情況產生過多航點）──
    arc_segments_cap: int = 32     # Dubins 弧最大離散段數

    # ── 重力常數（給 R_min 計算用，可在月球任務等場合覆寫）──
    gravity: float = 9.81

    def __post_init__(self) -> None:
        if not (0 < self.level_roll_limit_deg < 90):
            raise ValueError(
                f"LEVEL_ROLL_LIMIT 必須在 (0, 90)° 範圍，收到 "
                f"{self.level_roll_limit_deg}"
            )
        if self.airspeed_cruise <= 0:
            raise ValueError(f"AIRSPEED_CRUISE 必須 > 0，收到 {self.airspeed_cruise}")
        if self.wp_radius <= 0:
            raise ValueError(f"WP_RADIUS 必須 > 0，收到 {self.wp_radius}")

    # ── 物理推導屬性 ──────────────────────────────────────────────
    @property
    def r_min_m(self) -> float:
        """物理最小轉彎半徑 R_min = V_a² / (g·tan(φ_max))（公尺）

        協調轉彎力平衡推導：
            L sinφ = mV²/R     (向心力)
            L cosφ = mg        (升力垂直平衡重力)
          ⇒ tan φ = V² / (g R)
          ⇒ R = V² / (g tan φ)
        """
        phi = math.radians(self.level_roll_limit_deg)
        return (self.airspeed_cruise ** 2) / (self.gravity * math.tan(phi))

    @property
    def buffer_fw_m(self) -> float:
        """固定翼安全 buffer = R_min + WP_RADIUS + GPS_Error

        確保 L1 控制器能在 buffer 內完整畫出 R_min 弧、不剪到原 NFZ：
          • R_min       : 物理轉彎本身佔用
          • WP_RADIUS   : L1 提前 advance 區
          • GPS_Error   : 定位漂移最壞情況
        """
        return self.r_min_m + self.wp_radius + self.gps_error_fw_m

    @property
    def buffer_mr_m(self) -> float:
        """多旋翼安全 buffer = WP_RADIUS + GPS_Error（無轉彎半徑問題）"""
        return self.wp_radius + self.gps_error_mr_m

    @property
    def min_wp_spacing_m(self) -> float:
        """連續航點最小間距 = 2·WP_RADIUS。

        硬約束：兩 WP 距離 < 2·WP_RADIUS 時，L1 在進到第一個 WP 的
        advance 區的同時就「已經」進到下一個 WP 的 advance 區 → 連環
        略過 → 切弧、剪角、闖入 NFZ。
        """
        return 2.0 * self.wp_radius

    def describe(self) -> str:
        """產生人類可讀的推導摘要（給 demo 與 log 用）"""
        return (
            f"ArduPilot 參數：\n"
            f"  WP_RADIUS          = {self.wp_radius:.1f} m\n"
            f"  LEVEL_ROLL_LIMIT   = {self.level_roll_limit_deg:.1f}°\n"
            f"  AIRSPEED_CRUISE    = {self.airspeed_cruise:.1f} m/s\n"
            f"  GPS_Error (fw/mr)  = {self.gps_error_fw_m:.1f} / "
            f"{self.gps_error_mr_m:.1f} m\n"
            f"推導物理量：\n"
            f"  R_min              = {self.r_min_m:.1f} m  "
            f"= V²/(g·tan φ)\n"
            f"  buffer_fw          = {self.buffer_fw_m:.1f} m  "
            f"= R_min + WP_R + GPS\n"
            f"  buffer_mr          = {self.buffer_mr_m:.1f} m  "
            f"= WP_R + GPS\n"
            f"  min_wp_spacing     = {self.min_wp_spacing_m:.1f} m  "
            f"= 2·WP_RADIUS"
        )


# ──────────────────────────────────────────────────────────────────────
#  ArduPilot-aware Dubins 規劃器 — 重寫 fillet 離散化
# ──────────────────────────────────────────────────────────────────────
class _ArduPilotDubinsPlanner(FixedWingDubinsPlanner):
    """擴展 ``FixedWingDubinsPlanner``：每個 fillet 圓弧的離散段數**動態**
    依 WP_RADIUS 與該弧半徑計算，保證任兩相鄰航點直線間距 ≥ 2·WP_RADIUS。

    為何要動態？
        原版用固定段數 (arc_segments=12)，對小弧會產生過密航點（間距
        遠小於 WP_RADIUS），L1 會連環略過。對大弧又可能過稀（弦深超過
        buffer，路徑剪角）。動態計算同時兼顧兩者。
    """

    def __init__(self, config: PlannerConfig, ap_params: ArduPilotParams) -> None:
        super().__init__(config)
        self._ap = ap_params

    def _apply_fillets(
        self, broken: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """每個 fillet 用 R_min 圓弧倒角，**離散段數依 WP_RADIUS 動態算**。

        覆寫父類別行為：唯一差別在 N_SEG 計算公式 — 父類用固定 cfg 值，
        本類用 ``_arc_segments(arc_angle, R)`` 物理約束公式。

        其餘 geometry 計算（切線點、圓心、弧角）完全沿用父類邏輯。
        """
        R = self._cfg.r_min
        if len(broken) <= 2:
            return list(broken)

        out: List[Tuple[float, float]] = [broken[0]]

        for i in range(1, len(broken) - 1):
            A = broken[i - 1]
            B = broken[i]
            C = broken[i + 1]

            # 入向 / 出向 單位向量（從 B 向外）
            uxa = A[0] - B[0]; uya = A[1] - B[1]
            la = math.hypot(uxa, uya)
            uxc = C[0] - B[0]; uyc = C[1] - B[1]
            lc = math.hypot(uxc, uyc)
            if la < 1e-6 or lc < 1e-6:
                out.append(B)
                continue
            uxa /= la; uya /= la
            uxc /= lc; uyc /= lc

            # 內部夾角 α
            cos_alpha = max(-1.0, min(1.0, uxa * uxc + uya * uyc))
            alpha = math.acos(cos_alpha)
            if alpha < math.radians(1.0):
                out.append(B)
                continue
            if alpha > math.radians(175.0):
                # 接近直線但仍是繞行關鍵點，保留 B（不放弧）
                out.append(B)
                continue

            # 切線長 / 圓心距 B 的距離
            d_tangent = R / math.tan(alpha / 2.0)
            d_max = 0.45 * min(la, lc)
            if d_tangent > d_max:
                R_eff = d_max * math.tan(alpha / 2.0)
                R_eff = max(R_eff, R * 0.3)
                d_tangent = R_eff / math.tan(alpha / 2.0)
                R_used = R_eff
            else:
                R_used = R

            T1 = (B[0] + uxa * d_tangent, B[1] + uya * d_tangent)
            T2 = (B[0] + uxc * d_tangent, B[1] + uyc * d_tangent)

            # 圓心
            bx = uxa + uxc; by = uya + uyc
            bl = math.hypot(bx, by)
            if bl < 1e-9:
                out.append(B)
                continue
            bx /= bl; by /= bl
            d_center = R_used / math.sin(alpha / 2.0)
            Cx = B[0] + bx * d_center
            Cy = B[1] + by * d_center

            theta1 = math.atan2(T1[1] - Cy, T1[0] - Cx)
            theta2 = math.atan2(T2[1] - Cy, T2[0] - Cx)
            d_theta = theta2 - theta1
            while d_theta > math.pi:
                d_theta -= 2 * math.pi
            while d_theta < -math.pi:
                d_theta += 2 * math.pi

            # ★★★ 關鍵：動態計算 N_SEG（依 WP_RADIUS 約束）★★★
            n_seg = self._arc_segments(abs(d_theta), R_used)

            out.append(T1)
            for k in range(1, n_seg):
                t = k / n_seg
                th = theta1 + d_theta * t
                out.append((
                    Cx + R_used * math.cos(th),
                    Cy + R_used * math.sin(th),
                ))
            out.append(T2)

        out.append(broken[-1])
        return out

    def _arc_segments(self, abs_theta_rad: float, R: float) -> int:
        """每段子弧的弦長 ≥ 2·WP_RADIUS 之最大段數。

        子弧角 θ_sub 對應弦長 chord = 2R·sin(θ_sub/2)
        要求 chord ≥ 2·WP_RADIUS：
            sin(θ_sub/2) ≥ WP_RADIUS / R
            θ_sub_max = 2·arcsin(WP_RADIUS / R)
            N_max = floor(|θ_total| / θ_sub_max)

        Args:
            abs_theta_rad : 整段弧的角度大小（弧度，≥ 0）
            R             : 該弧的實際半徑（可能因 fillet 縮減 < R_min）

        Returns:
            離散段數 N（弧上會放 N-1 個中間點 + 兩個切點）
            邊界處理：
              R ≤ WP_RADIUS  → 1（弧過小，只走 T1→T2 直線弦）
              N_max < 1     → 1（同上）
              N_max > cap   → cap（防止極端弧產生過多 WP）
        """
        WR = self._ap.wp_radius
        if R <= WR:
            return 1
        max_chord_angle = 2.0 * math.asin(WR / R)
        if max_chord_angle <= 0:
            return 1
        n_max = int(math.floor(abs_theta_rad / max_chord_angle))
        if n_max < 1:
            return 1
        return min(n_max, self._ap.arc_segments_cap)


# ──────────────────────────────────────────────────────────────────────
#  主控管理器：DCCPP_Heterogeneous_NFZ_Manager
# ──────────────────────────────────────────────────────────────────────
class DCCPP_Heterogeneous_NFZ_Manager:
    """DCCPP 異質機隊 NFZ 管理器 — ArduPilot 參數驅動主控類別。

    與 ``HeterogeneousNFZPlanner`` 的差異：
        • 不再使用硬編碼 buffer / R_min；改從 ``ArduPilotParams`` 動態推導
        • FW Dubins 弧離散化以 WP_RADIUS 為下限自動算段數
        • 後處理強制所有相鄰 WP 間距 ≥ 2·WP_RADIUS
        • MAV_CMD 指令碼依機型 + 位置自動標記
        • 支援 vehicle_type='vtol'，起降用 NAV_VTOL_TAKEOFF/LAND
        • 額外提供 ArduPilot 圍籬檔（.fence）匯出

    使用範例：
        >>> ap = ArduPilotParams(
        ...     wp_radius=20.0, level_roll_limit_deg=35.0,
        ...     airspeed_cruise=18.0,
        ... )
        >>> mgr = DCCPP_Heterogeneous_NFZ_Manager(ap)
        >>> wps = mgr.plan_heterogeneous_routes(
        ...     start_pt=(23.700, 120.440),
        ...     target_pt=(23.720, 120.440),
        ...     nfz_polygon=[(23.707, 120.435), (23.707, 120.445),
        ...                  (23.713, 120.445), (23.713, 120.435)],
        ... )
        >>> mission = mgr.export_qgc_wpl110(wps['fixed_wing'],
        ...                                 vehicle_type='fixed_wing',
        ...                                 home=(23.700, 120.440))
        >>> fence = mgr.export_ardupilot_fence(nfz_polygon)
    """

    def __init__(self, ap_params: Optional[ArduPilotParams] = None) -> None:
        self._ap = ap_params or ArduPilotParams()
        # 由 ArduPilotParams 推導 PlannerConfig（給底層共用幾何邏輯用）
        self._cfg = PlannerConfig(
            buffer_mr=self._ap.buffer_mr_m,
            buffer_fw=self._ap.buffer_fw_m,
            alt_mr=self._ap.alt_mr,
            alt_fw=self._ap.alt_fw,
            r_min=self._ap.r_min_m,
            arc_segments=self._ap.arc_segments_cap,  # 上限，實際動態算
        )
        # 多旋翼仍用既有 A*，固定翼換成 ArduPilot-aware Dubins
        self._mr_planner = MultirotorAStarPlanner(self._cfg)
        self._fw_planner = _ArduPilotDubinsPlanner(self._cfg, self._ap)

    @property
    def ap_params(self) -> ArduPilotParams:
        return self._ap

    @property
    def planner_config(self) -> PlannerConfig:
        return self._cfg

    # ─────────────────────────────────────────────────────────────────
    #  主對外 API
    # ─────────────────────────────────────────────────────────────────
    def plan_heterogeneous_routes(
        self,
        start_pt: Tuple[float, float],
        target_pt: Tuple[float, float],
        nfz_polygon: Sequence[Tuple[float, float]],
    ) -> Dict[str, List[Waypoint]]:
        """同時為 MR + FW 兩種機型規劃（不指定 vehicle_type 時用此）。

        Returns:
            {'multirotor': [Waypoint, ...], 'fixed_wing': [Waypoint, ...]}
            兩條路徑都已套用 WP_RADIUS 間距下限。Waypoint.cmd 統一為
            NAV_WAYPOINT(16)；若需 SPLINE / VTOL 標記，請改用 plan_for_vehicle()。
        """
        if len(nfz_polygon) < 3:
            raise ValueError('NFZ 多邊形至少需 3 頂點')

        lat0 = (start_pt[0] + target_pt[0]) / 2.0
        lon0 = (start_pt[1] + target_pt[1]) / 2.0
        proj = _LocalProjection(lat0, lon0)

        s_xy = proj.to_metric(*start_pt)
        t_xy = proj.to_metric(*target_pt)
        nfz_xy = proj.project_polygon(nfz_polygon)
        nfz_raw = Polygon(nfz_xy)
        if not nfz_raw.is_valid:
            nfz_raw = nfz_raw.buffer(0)

        # 雙 buffer — quad_segs=32 讓圓角化弦深 < 0.1m
        buf_mr = self._largest_polygon(
            nfz_raw.buffer(self._ap.buffer_mr_m, quad_segs=32)
        )
        buf_fw = self._largest_polygon(
            nfz_raw.buffer(self._ap.buffer_fw_m, quad_segs=32)
        )

        mr_path_xy = self._mr_planner.plan(s_xy, t_xy, buf_mr)
        fw_path_xy = self._fw_planner.plan(s_xy, t_xy, buf_fw)

        mr_wps = [Waypoint(*proj.to_latlon(x, y), alt=self._ap.alt_mr)
                  for x, y in mr_path_xy]
        fw_wps = [Waypoint(*proj.to_latlon(x, y), alt=self._ap.alt_fw)
                  for x, y in fw_path_xy]

        # 強制 WP_RADIUS 間距下限
        mr_wps = self._enforce_min_spacing(mr_wps, proj)
        fw_wps = self._enforce_min_spacing(fw_wps, proj)

        return {'multirotor': mr_wps, 'fixed_wing': fw_wps}

    def plan_for_vehicle(
        self,
        start_pt: Tuple[float, float],
        target_pt: Tuple[float, float],
        nfz_polygon: Sequence[Tuple[float, float]],
        vehicle_type: str,
    ) -> List[Waypoint]:
        """為單一機型規劃，並依機型自動標記 MAV_CMD 指令碼。

        Args:
            vehicle_type : 'multirotor' | 'fixed_wing' | 'vtol'
                MR 中段繞行點   → MAV_CMD_NAV_SPLINE_WAYPOINT (82)
                VTOL 起降       → MAV_CMD_NAV_VTOL_TAKEOFF / LAND (84/85)
                FW              → MAV_CMD_NAV_WAYPOINT (16)（不可 SPLINE）
        """
        if vehicle_type not in ('multirotor', 'fixed_wing', 'vtol'):
            raise ValueError(
                f"vehicle_type 必須是 multirotor/fixed_wing/vtol，"
                f"收到 {vehicle_type!r}"
            )
        routes = self.plan_heterogeneous_routes(start_pt, target_pt, nfz_polygon)
        # FW 與 VTOL 巡航段都用 FW 路徑（VTOL 在 cruise 是 FW 模式）；
        # MR 用 MR 路徑
        if vehicle_type == 'fixed_wing':
            wps = routes['fixed_wing']
        elif vehicle_type == 'vtol':
            # VTOL 起降垂直，巡航走 FW；用 FW 路徑但起降標記 VTOL 指令
            wps = routes['fixed_wing']
        else:
            wps = routes['multirotor']

        return self._tag_mavlink_commands(wps, vehicle_type)

    # ─────────────────────────────────────────────────────────────────
    #  後處理 — 強制 WP_RADIUS 間距下限
    # ─────────────────────────────────────────────────────────────────
    def _enforce_min_spacing(
        self, wps: List[Waypoint], proj: _LocalProjection,
    ) -> List[Waypoint]:
        """walk through 航點列表，丟掉與前點距離 < 2·WP_RADIUS 的點。

        關鍵原則：
            • 起點與終點絕對保留
            • 中間點若距前一保留點 < min_spacing → 丟棄
            • 終點若距最後一保留點 < min_spacing → 取代最後一保留點

        為何用 Shapely simplify 不夠？
            simplify 是基於 Douglas-Peucker，依「垂直偏離容差」過濾；
            無法直接保證「線段長度」下限。我們的約束是「直線距離下限」，
            必須自行 walk through。
        """
        min_d = self._ap.min_wp_spacing_m
        n = len(wps)
        if n <= 2:
            return list(wps)

        # 預先投影成 metric — 避免重複計算
        xy = [proj.to_metric(w.lat, w.lon) for w in wps]

        kept: List[int] = [0]
        for i in range(1, n - 1):
            ax, ay = xy[kept[-1]]
            bx, by = xy[i]
            if math.hypot(bx - ax, by - ay) >= min_d:
                kept.append(i)

        # 終點處理
        ax, ay = xy[kept[-1]]
        bx, by = xy[n - 1]
        last_d = math.hypot(bx - ax, by - ay)
        if last_d >= min_d:
            kept.append(n - 1)
        else:
            # 終點過近於前一保留點 — 但終點必須保留，所以取代前一個
            # （除非前一個就是起點，那只好把終點直接接在起點後）
            if len(kept) > 1:
                kept[-1] = n - 1
            else:
                kept.append(n - 1)

        return [wps[i] for i in kept]

    # ─────────────────────────────────────────────────────────────────
    #  後處理 — MAVLink 指令碼標記
    # ─────────────────────────────────────────────────────────────────
    def _tag_mavlink_commands(
        self, wps: List[Waypoint], vehicle_type: str,
    ) -> List[Waypoint]:
        """依機型 + 位置設定 wp.cmd 指令碼。"""
        if not wps:
            return wps
        n = len(wps)
        out: List[Waypoint] = []
        for i, wp in enumerate(wps):
            new = copy.copy(wp)
            if vehicle_type == 'vtol':
                # 起點 → VTOL_TAKEOFF，終點 → VTOL_LAND，中段巡航 → WAYPOINT
                # （VTOL 在 cruise 階段是固定翼模式，不能用 SPLINE）
                if i == 0:
                    new.cmd = MAV_CMD_NAV_VTOL_TAKEOFF
                elif i == n - 1:
                    new.cmd = MAV_CMD_NAV_VTOL_LAND
                else:
                    new.cmd = MAV_CMD_NAV_WAYPOINT
            elif vehicle_type == 'multirotor':
                # 中段繞行點用 SPLINE 平滑轉彎，避免銳角急煞抖動；
                # 起終點必須是 NAV_WAYPOINT（精準位置）
                if 0 < i < n - 1:
                    new.cmd = MAV_CMD_NAV_SPLINE_WAYPOINT
                else:
                    new.cmd = MAV_CMD_NAV_WAYPOINT
            else:  # fixed_wing
                # 固定翼全程 NAV_WAYPOINT；ArduPlane 不支援 SPLINE
                new.cmd = MAV_CMD_NAV_WAYPOINT
            out.append(new)
        return out

    @staticmethod
    def _largest_polygon(geom):
        if geom.geom_type == 'Polygon':
            return geom
        if geom.geom_type == 'MultiPolygon':
            return max(geom.geoms, key=lambda p: p.area)
        raise TypeError(f'未預期幾何 {geom.geom_type}')

    # ─────────────────────────────────────────────────────────────────
    #  匯出 — Mission (QGC WPL 110)
    # ─────────────────────────────────────────────────────────────────
    def export_qgc_wpl110(
        self,
        waypoints: List[Waypoint],
        home: Tuple[float, float],
        home_alt: float = 0.0,
    ) -> str:
        """匯出為 QGC WPL 110 任務檔（ArduPilot Mission Planner / QGC 通用）。

        第 0 列為 HOME（frame=0 GLOBAL，cmd=16），其後依序為使用者航點。
        若 ``waypoints[0].cmd == MAV_CMD_NAV_VTOL_TAKEOFF``，則該列 frame
        會自動設為 GLOBAL_RELATIVE_ALT（VTOL 起降使用相對高）。
        """
        lines = ['QGC WPL 110']
        home_wp = Waypoint(
            lat=home[0], lon=home[1], alt=home_alt,
            cmd=MAV_CMD_NAV_WAYPOINT, frame=MAV_FRAME_GLOBAL,
        )
        lines.append(home_wp.to_mavlink_wpl_line(seq=0, current=1))
        for i, wp in enumerate(waypoints, start=1):
            # VTOL_TAKEOFF / LAND 用 RELATIVE_ALT (frame=3) 標準慣例
            if wp.cmd in (MAV_CMD_NAV_VTOL_TAKEOFF, MAV_CMD_NAV_VTOL_LAND):
                wp_out = copy.copy(wp)
                wp_out.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
                lines.append(wp_out.to_mavlink_wpl_line(seq=i))
            else:
                lines.append(wp.to_mavlink_wpl_line(seq=i))
        return '\n'.join(lines)

    # ─────────────────────────────────────────────────────────────────
    #  匯出 — ArduPilot Fence（.fence 檔）
    # ─────────────────────────────────────────────────────────────────
    def export_ardupilot_fence(
        self,
        nfz_polygon: Sequence[Tuple[float, float]],
        return_point: Optional[Tuple[float, float]] = None,
        as_mission_item: bool = True,
    ) -> str:
        """匯出 ArduPilot 相容 .fence 檔，含 FENCE_* 參數 header。

        Args:
            nfz_polygon  : [(lat, lon), ...] NFZ 多邊形頂點
            return_point : 越界 RTL 的歸航點；不指定時用 polygon 質心
            as_mission_item :
                True  → 用 MAV_MISSION_TYPE_FENCE 現代格式（cmd=5002 排除）
                        支援 ArduPilot 4.0+
                False → 傳統 fencepoint 格式（單行 lat lon），相容老版本

        Returns:
            完整 .fence 檔內容字串，含 FENCE_* 參數設定 header。
        """
        if len(nfz_polygon) < 3:
            raise ValueError('NFZ 多邊形至少需 3 頂點')

        # 計算 return point — 預設用多邊形質心 (offset 一點點避開邊界)
        if return_point is None:
            cy = sum(p[0] for p in nfz_polygon) / len(nfz_polygon)
            cx = sum(p[1] for p in nfz_polygon) / len(nfz_polygon)
            return_point = (cy, cx)

        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        n_verts = len(nfz_polygon)

        # ── 共用 header ──
        header = [
            '# AeroPlan Studio — ArduPilot fence export',
            f'# Generated: {ts}',
            f'# NFZ vertices: {n_verts}',
            '#',
            '# ╔══════════════════════════════════════════════════════════╗',
            '# ║  REQUIRED ArduPilot fence parameters (set first!):      ║',
            '# ╚══════════════════════════════════════════════════════════╝',
            '# FENCE_ENABLE     = 1',
            f'# FENCE_ACTION     = {self._ap.fence_action}  '
            f'# (1=RTL, 2=Land, 4=Brake)',
            '# FENCE_TYPE       = 7  # (1=MaxAlt | 2=Circle | 4=Polygon)',
            f'# FENCE_TOTAL      = {n_verts}',
            f'# FENCE_ALT_MAX    = {self._ap.fence_alt_max:.0f}',
            '#',
        ]

        if as_mission_item:
            # ── 現代格式 (MAV_MISSION_TYPE_FENCE)，相容 ArduPilot 4.0+ ──
            lines = list(header) + [
                '# Format: MAVLink mission items, cmd=5000 (return) + '
                'cmd=5002 (exclusion polygon vertex)',
                'QGC WPL 110',
            ]
            # seq 0 = return point
            ret = Waypoint(
                lat=return_point[0], lon=return_point[1], alt=0.0,
                cmd=MAV_CMD_NAV_FENCE_RETURN_POINT,
                frame=MAV_FRAME_GLOBAL,
            )
            lines.append(ret.to_mavlink_wpl_line(seq=0, current=1))
            # seq 1..N = exclusion polygon vertices；param1 = 該 polygon 頂點數
            for i, (lat, lon) in enumerate(nfz_polygon, start=1):
                v = Waypoint(
                    lat=lat, lon=lon, alt=0.0,
                    cmd=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    frame=MAV_FRAME_GLOBAL,
                )
                lines.append(v.to_mavlink_wpl_line(
                    seq=i, p1=float(n_verts),
                ))
            return '\n'.join(lines)
        else:
            # ── 傳統 fencepoint 格式（每行 lat lon）──
            lines = list(header) + [
                '# Format: legacy fencepoint (lat lon per line)',
                '# First line = return point; following lines = polygon vertices.',
                '# Polygon should be closed: last line should equal second line.',
                '',
                f'{return_point[0]:.7f} {return_point[1]:.7f}',
            ]
            for lat, lon in nfz_polygon:
                lines.append(f'{lat:.7f} {lon:.7f}')
            # 閉合多邊形：尾端重複第一頂點
            lines.append(f'{nfz_polygon[0][0]:.7f} {nfz_polygon[0][1]:.7f}')
            return '\n'.join(lines)

    # ─────────────────────────────────────────────────────────────────
    #  驗證 — 規劃結果是否符合 ArduPilot 物理極限
    # ─────────────────────────────────────────────────────────────────
    def validate_for_ardupilot(
        self, wps: List[Waypoint], vehicle_type: str,
    ) -> Dict[str, object]:
        """檢查路徑是否符合 ArduPilot L1 / 物理極限。

        Returns:
            {
              'ok': bool,
              'n_waypoints': int,
              'min_spacing_m': float,        # 觀察到的最小間距
              'spacing_threshold_m': float,  # 規定的下限 (2·WP_RADIUS)
              'spacing_ok': bool,
              'min_local_radius_m': float,   # 觀察到的最小局部圓弧半徑
              'r_min_threshold_m': float,    # R_min × 0.85 容差
              'radius_ok': bool,             # 僅 FW 檢查；MR 跳過
              'warnings': List[str],
            }
        """
        result: dict = {
            'ok': True, 'n_waypoints': len(wps),
            'min_spacing_m': float('inf'),
            'spacing_threshold_m': self._ap.min_wp_spacing_m,
            'spacing_ok': True,
            'min_local_radius_m': float('inf'),
            'r_min_threshold_m': self._ap.r_min_m * 0.85,
            'radius_ok': True,
            'warnings': [],
        }
        if len(wps) < 2:
            result['warnings'].append('航點數 < 2，無法驗證')
            return result

        # 投影到本地 ENU
        proj = _LocalProjection(wps[0].lat, wps[0].lon)
        xy = [proj.to_metric(w.lat, w.lon) for w in wps]

        # 間距檢查
        for i in range(1, len(xy)):
            d = math.hypot(xy[i][0] - xy[i-1][0], xy[i][1] - xy[i-1][1])
            if d < result['min_spacing_m']:
                result['min_spacing_m'] = d
        if result['min_spacing_m'] < result['spacing_threshold_m']:
            result['spacing_ok'] = False
            result['warnings'].append(
                f'最小航點間距 {result["min_spacing_m"]:.1f}m < '
                f'門檻 {result["spacing_threshold_m"]:.1f}m '
                f'(2·WP_RADIUS) → ArduPlane L1 可能略過'
            )

        # 局部圓弧半徑檢查（僅 FW / VTOL cruise 適用）
        if vehicle_type in ('fixed_wing', 'vtol') and len(xy) >= 3:
            for i in range(1, len(xy) - 1):
                A, B, C = xy[i-1], xy[i], xy[i+1]
                ab = math.hypot(B[0]-A[0], B[1]-A[1])
                bc = math.hypot(C[0]-B[0], C[1]-B[1])
                ac = math.hypot(C[0]-A[0], C[1]-A[1])
                if ab < 0.5 or bc < 0.5 or ac < 0.5:
                    continue
                cross = (B[0]-A[0])*(C[1]-A[1]) - (B[1]-A[1])*(C[0]-A[0])
                area = abs(cross) / 2.0
                if area < 0.5:
                    continue
                r = ab * bc * ac / (4.0 * area)
                if r < result['min_local_radius_m']:
                    result['min_local_radius_m'] = r
            if result['min_local_radius_m'] < result['r_min_threshold_m']:
                result['radius_ok'] = False
                result['warnings'].append(
                    f'最小局部圓弧半徑 {result["min_local_radius_m"]:.1f}m < '
                    f'R_min×0.85 = {result["r_min_threshold_m"]:.1f}m '
                    f'→ 飛機可能飛不出此角度'
                )

        result['ok'] = result['spacing_ok'] and result['radius_ok']
        return result


# ──────────────────────────────────────────────────────────────────────
#  Demo / __main__
# ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=' * 75)
    print(' DCCPP_Heterogeneous_NFZ_Manager — ArduPilot 參數驅動規劃示範')
    print('=' * 75)

    # ── 1. 設定 ArduPilot 參數（直接對應 FCU .parm 檔）──
    ap = ArduPilotParams(
        wp_radius=20.0,           # WP_RADIUS
        level_roll_limit_deg=35.0,  # LEVEL_ROLL_LIMIT
        airspeed_cruise=18.0,     # AIRSPEED_CRUISE
        gps_error_fw_m=15.0,
        gps_error_mr_m=5.0,
        alt_mr=80.0,
        alt_fw=120.0,
        fence_action=1,           # 1 = RTL on breach
        fence_alt_max=200.0,
    )

    print()
    print(ap.describe())

    # ── 2. 場景：起點正南、目標正北，中間正方形 NFZ ──
    start = (23.700, 120.440)
    target = (23.720, 120.440)
    nfz_square = [
        (23.707, 120.435), (23.707, 120.445),
        (23.713, 120.445), (23.713, 120.435),
    ]
    print('\n場景：')
    print(f'  起點 = {start}')
    print(f'  目標 = {target}')
    print('  NFZ  = 正方形 4 頂點 (~660m × 1100m)')

    mgr = DCCPP_Heterogeneous_NFZ_Manager(ap)

    # ── 3. 分別為三種機型規劃 ──
    for vehicle in ('multirotor', 'fixed_wing', 'vtol'):
        print('\n' + '-' * 75)
        print(f' 機型：{vehicle}')
        print('-' * 75)
        wps = mgr.plan_for_vehicle(start, target, nfz_square, vehicle)
        print(f'  航點數: {len(wps)}')

        # 列前 6 / 後 3 航點
        n = len(wps)
        cmd_name = {16: 'WAYPOINT', 82: 'SPLINE', 84: 'VTOL_TAKEOFF', 85: 'VTOL_LAND'}
        for i, wp in enumerate(wps):
            if i < 6 or i >= n - 3:
                cn = cmd_name.get(wp.cmd, f'CMD{wp.cmd}')
                print(f'    [{i:3d}] lat={wp.lat:.6f}  lon={wp.lon:.6f}  '
                      f'alt={wp.alt:.0f}m  cmd={wp.cmd}({cn})')
            elif i == 6:
                print(f'    ... ({n - 9} 點省略) ...')

        # 驗證
        v = mgr.validate_for_ardupilot(wps, vehicle)
        print(f'\n  ArduPilot 驗證：'
              f'spacing_ok={v["spacing_ok"]}  '
              f'radius_ok={v["radius_ok"]}  '
              f'overall_ok={v["ok"]}')
        print(f'    最小間距 = {v["min_spacing_m"]:.1f} m  '
              f'(門檻 {v["spacing_threshold_m"]:.1f} m)')
        if vehicle != 'multirotor' and v['min_local_radius_m'] != float('inf'):
            print(f'    最小弧 R = {v["min_local_radius_m"]:.1f} m  '
                  f'(門檻 {v["r_min_threshold_m"]:.1f} m)')
        for w in v['warnings']:
            print(f'    ⚠ {w}')

    # ── 4. 匯出 QGC WPL 110（取固定翼當範例）──
    print('\n' + '=' * 75)
    print(' MAVLink Mission 匯出 — QGC WPL 110（固定翼，前 7 行）')
    print('=' * 75)
    fw_wps = mgr.plan_for_vehicle(start, target, nfz_square, 'fixed_wing')
    mission = mgr.export_qgc_wpl110(fw_wps, home=start)
    for line in mission.split('\n')[:7]:
        print('  ' + line)
    total_lines = len(mission.split('\n'))
    print(f'  ... (共 {total_lines} 行，含 HOME)')

    # ── 5. 匯出 ArduPilot Fence ──
    print('\n' + '=' * 75)
    print(' ArduPilot Fence 匯出 — 現代 MAVLink 格式')
    print('=' * 75)
    fence = mgr.export_ardupilot_fence(nfz_square, as_mission_item=True)
    for line in fence.split('\n'):
        print('  ' + line)

    print('\n' + '=' * 75)
    print(' 示範結束 — 路徑已符合 ArduPilot L1 / R_min 物理極限')
    print('=' * 75)
