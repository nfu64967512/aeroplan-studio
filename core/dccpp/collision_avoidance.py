"""
DCCPP 多機避撞模組
==================
在 DCCPP 路徑規劃完成後，對所有 UAV 的路徑做時間同步模擬，
偵測即將碰撞的衝突區段，並自動解衝突。

解衝突策略
----------
1. **高度錯開 (Altitude Deconfliction)**：
   衝突區段中，優先權較低的 UAV 暫時升高 / 降低 alt_offset_m，
   離開衝突區段後恢復原高度。適用於空域充裕的情境。
   `run()` 會迭代多輪（最多 max_rounds），每架的**累加**抬升不得超過
   max_alt_offset_m（None = 依機數自動）；撞到上限就停止抬升並 ERROR。

2. **時間延遲 (Temporal Deconfliction)**：⚠ 無效，已退回 altitude。
   插入的 loiter 航點與參考航點同經緯度同高度，而時間由「累積距離 / 速度」
   推出 ⇒ 零長度線段貢獻 0 秒延遲；delay_s 也沒有任何下游消費
   （見 tests/test_collision_avoidance_rounds.py）。

呼叫範例
--------
    avoidance = CollisionAvoidance(
        min_separation_m=100.0,
        alt_offset_m=100.0,
        time_step_s=1.0,
        cruise_speed_mps=40.0,
        max_alt_offset_m=None,      # None = 自動（依機數）
    )
    conflicts, actions = avoidance.run(all_built_paths)
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

from utils.math_utils import EARTH_RADIUS_M  # 0-2 標準化：WGS84 單一來源

if TYPE_CHECKING:
    # 僅供型別註解使用，避免與 dccpp_path_builder 的執行期循環依賴
    from core.dccpp.dccpp_path_builder import BuiltPath

logger = logging.getLogger(__name__)

#: 自動累加上限的層數係數：上限 = 係數 x (機數-1) x alt_offset_m。
#: 實測合法收斂需要 2(n-1) 層 —— 直覺的 (n-1) 不夠，因為每輪抬的是
#: 「配對中 ID 較大的那架」，中間那幾架會跟著一起爬（係數 1 會把 n=3
#: 本來解得開的情境擋成無解，見 tests/test_collision_alt_cap.py）。
_AUTO_CAP_LAYERS = 2


# ============================================================
# 資料結構
# ============================================================

@dataclass
class ConflictEvent:
    """一次衝突事件"""
    uav_a: int
    uav_b: int
    time_s: float                # 衝突發生時刻
    duration_s: float            # 衝突持續時間
    min_dist_m: float            # 最小距離
    pos_a: Tuple[float, float, float]  # (lat, lon, alt)
    pos_b: Tuple[float, float, float]
    wp_idx_a: int                # 衝突發生時 UAV A 正飛向的航點索引
    wp_idx_b: int


@dataclass
class AvoidanceAction:
    """避撞動作"""
    uav_id: int
    action_type: str             # 'altitude_offset' | 'loiter_delay'
    wp_start_idx: int            # 開始修改的航點索引
    wp_end_idx: int              # 結束修改的航點索引
    alt_offset_m: float = 0.0    # 高度偏移量
    delay_s: float = 0.0         # 延遲秒數
    conflict: Optional[ConflictEvent] = None


# ============================================================
# 工具函數
# ============================================================

def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """兩點間的大圓距離 (公尺)"""
    R = EARTH_RADIUS_M
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _dist_3d(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
    """3D 距離 (水平大圓 + 垂直差)"""
    h = _haversine(p1[0], p1[1], p2[0], p2[1])
    dz = abs(p1[2] - p2[2])
    return math.sqrt(h ** 2 + dz ** 2)


def _interpolate_path_at_time(
    waypoints: list,
    cum_dists: List[float],
    total_dist: float,
    speed_mps: float,
    t: float,
) -> Tuple[float, float, float, int]:
    """給定時刻 t，回傳 (lat, lon, alt, next_wp_idx)"""
    if not waypoints:
        return (0.0, 0.0, 0.0, 0)

    dist_at_t = speed_mps * t
    if dist_at_t >= total_dist:
        wp = waypoints[-1]
        return (wp.lat, wp.lon, wp.alt, len(waypoints) - 1)
    if dist_at_t <= 0:
        wp = waypoints[0]
        return (wp.lat, wp.lon, wp.alt, 0)

    # 二分搜尋目前在哪一段
    lo, hi = 0, len(cum_dists) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if cum_dists[mid] < dist_at_t:
            lo = mid + 1
        else:
            hi = mid
    seg_idx = max(lo - 1, 0)
    seg_start_dist = cum_dists[seg_idx]
    seg_end_dist = cum_dists[seg_idx + 1]
    seg_len = seg_end_dist - seg_start_dist
    if seg_len < 1e-6:
        frac = 0.0
    else:
        frac = (dist_at_t - seg_start_dist) / seg_len

    w0 = waypoints[seg_idx]
    w1 = waypoints[seg_idx + 1]
    lat = w0.lat + frac * (w1.lat - w0.lat)
    lon = w0.lon + frac * (w1.lon - w0.lon)
    alt = w0.alt + frac * (w1.alt - w0.alt)
    return (lat, lon, alt, seg_idx + 1)


# ============================================================
# 核心類別
# ============================================================

def _is_ground_phase(wp) -> bool:
    """這個航點屬於起飛或降落段嗎？（避撞的高度偏移一律豁免）"""
    return getattr(getattr(wp, 'segment_type', None), 'name', '') in (
        'TAKEOFF', 'LANDING')


class CollisionAvoidance:
    """多機避撞偵測與解衝突"""

    def __init__(
        self,
        min_separation_m: float = 100.0,
        alt_offset_m: float = 30.0,
        time_step_s: float = 1.0,
        cruise_speed_mps: float = 40.0,
        strategy: str = 'altitude',   # 'altitude' | 'temporal'
        max_alt_offset_m: Optional[float] = None,
        max_rounds: int = 8,
    ):
        """
        Args:
            max_alt_offset_m: 每架 UAV 的**累加**抬升上限（公尺）。
                None = 自動，依機數 `_AUTO_CAP_LAYERS x (機數-1) x alt_offset_m`
                （保證不擋住本來解得開的情境，但不保證高度低 —— 要壓住絕對
                高度就得自己填數字）。給數字 = 絕對天花板，自動值不得蓋過去。
                0 **不是**「自動」而是「完全不准抬」；面板的 0 在接縫
                （`build_avoidance_kwargs`）之前就已翻成 None。
            max_rounds: `run()` 最多解幾輪衝突。

        ⚠ 不合理的組合只**大聲警告**，不偷偷夾值 —— 顯示的值必須就是用的值。
        """
        self.min_sep = float(min_separation_m)
        self.alt_offset = float(alt_offset_m)
        self.dt = float(time_step_s)
        self.speed = float(cruise_speed_mps)
        self.strategy = strategy
        self.max_alt_offset = (None if max_alt_offset_m is None
                               else float(max_alt_offset_m))
        self.max_rounds = int(max_rounds)

        # NaN 比較一律為 False ⇒ 上限會**靜靜地**失效，所以直接拒絕
        if self.max_alt_offset is not None and math.isnan(self.max_alt_offset):
            raise ValueError("max_alt_offset_m 不可為 NaN（None 才是「自動」）")
        if self.max_rounds < 1:
            raise ValueError(f"max_rounds 必須 >= 1（收到 {max_rounds}）")

        # 每架 UAV 開始避撞前的基準高度；run() 開頭清空，resolve() 惰性填入
        self._base_alt: Dict[int, float] = {}
        #: 最近一次 run() 結束時**仍然存在**的衝突（空 = 全部解掉）。
        #: 給 GUI 用：解不掉不能只留在日誌裡，要讓使用者看得到。
        self.remaining_conflicts: List[ConflictEvent] = []

        if self.alt_offset < self.min_sep:
            logger.warning(
                f"避撞: 高度錯開量 {self.alt_offset:.0f} m < 最小間距 "
                f"{self.min_sep:.0f} m ⇒ 單次抬升拉不開，完全重疊的航跡要累加多輪，"
                f"且會受抬升上限限制")
        if self.max_alt_offset is not None and self.max_alt_offset < self.alt_offset:
            logger.error(
                f"避撞: 抬升上限 {self.max_alt_offset:.0f} m < 高度錯開量 "
                f"{self.alt_offset:.0f} m ⇒ 連抬一次都不被允許，高度錯開形同關閉")

    # ────────────────────────────────────────────────────────
    # 累加上限
    # ────────────────────────────────────────────────────────
    def _effective_cap(self, paths: Dict[int, "BuiltPath"]) -> float:
        """這一批路徑每架 UAV 的累加抬升上限（公尺）。"""
        if self.max_alt_offset is not None:
            return self.max_alt_offset
        n_uav = sum(1 for bp in paths.values() if bp.waypoints)
        return _AUTO_CAP_LAYERS * max(n_uav - 1, 1) * self.alt_offset

    # ────────────────────────────────────────────────────────
    # 路徑前處理：計算累積距離
    # ────────────────────────────────────────────────────────
    @staticmethod
    def _preprocess(waypoints) -> Tuple[List[float], float]:
        """計算累積距離 [0, d01, d01+d12, ...]"""
        cum = [0.0]
        for i in range(1, len(waypoints)):
            w0, w1 = waypoints[i - 1], waypoints[i]
            d = _haversine(w0.lat, w0.lon, w1.lat, w1.lon)
            dz = abs(w1.alt - w0.alt)
            cum.append(cum[-1] + math.sqrt(d ** 2 + dz ** 2))
        return cum, cum[-1] if cum else 0.0

    # ────────────────────────────────────────────────────────
    # 衝突偵測
    # ────────────────────────────────────────────────────────
    def detect(self, paths: Dict[int, "BuiltPath"]) -> List[ConflictEvent]:
        """
        對所有 UAV 路徑做時間同步模擬，回傳所有衝突事件。

        Args:
            paths: {uav_id: BuiltPath}
        Returns:
            衝突事件列表（已按時間排序）
        """
        if len(paths) < 2:
            return []

        # 前處理
        prep = {}
        for uid, bp in paths.items():
            if not bp.waypoints:
                continue
            cum, total = self._preprocess(bp.waypoints)
            prep[uid] = (bp.waypoints, cum, total)

        if len(prep) < 2:
            return []

        # 模擬最大時間
        max_time = max(total / max(self.speed, 0.1) for _, _, total in prep.values())

        uids = sorted(prep.keys())
        conflicts: List[ConflictEvent] = []
        active_conflicts: Dict[Tuple[int, int], dict] = {}

        t = 0.0
        while t <= max_time:
            # 計算每架 UAV 此刻的位置
            positions = {}
            for uid in uids:
                wps, cum, total = prep[uid]
                positions[uid] = _interpolate_path_at_time(wps, cum, total, self.speed, t)

            # 檢查所有配對
            for i in range(len(uids)):
                for j in range(i + 1, len(uids)):
                    ua, ub = uids[i], uids[j]
                    pa = positions[ua]  # (lat, lon, alt, wp_idx)
                    pb = positions[ub]
                    dist = _dist_3d(pa[:3], pb[:3])
                    pair = (ua, ub)

                    if dist < self.min_sep:
                        if pair not in active_conflicts:
                            active_conflicts[pair] = {
                                'start_t': t,
                                'min_dist': dist,
                                'pos_a': pa[:3],
                                'pos_b': pb[:3],
                                'wp_a': pa[3],
                                'wp_b': pb[3],
                            }
                        else:
                            ac = active_conflicts[pair]
                            if dist < ac['min_dist']:
                                ac['min_dist'] = dist
                                ac['pos_a'] = pa[:3]
                                ac['pos_b'] = pb[:3]
                                ac['wp_a'] = pa[3]
                                ac['wp_b'] = pb[3]
                    else:
                        if pair in active_conflicts:
                            ac = active_conflicts.pop(pair)
                            conflicts.append(ConflictEvent(
                                uav_a=ua, uav_b=ub,
                                time_s=ac['start_t'],
                                duration_s=t - ac['start_t'],
                                min_dist_m=ac['min_dist'],
                                pos_a=ac['pos_a'],
                                pos_b=ac['pos_b'],
                                wp_idx_a=ac['wp_a'],
                                wp_idx_b=ac['wp_b'],
                            ))
            t += self.dt

        # 結束時仍在衝突中的
        for pair, ac in active_conflicts.items():
            conflicts.append(ConflictEvent(
                uav_a=pair[0], uav_b=pair[1],
                time_s=ac['start_t'],
                duration_s=max_time - ac['start_t'],
                min_dist_m=ac['min_dist'],
                pos_a=ac['pos_a'],
                pos_b=ac['pos_b'],
                wp_idx_a=ac['wp_a'],
                wp_idx_b=ac['wp_b'],
            ))

        conflicts.sort(key=lambda c: c.time_s)
        logger.info(f"避撞偵測完成: {len(conflicts)} 個衝突事件")
        return conflicts

    # ────────────────────────────────────────────────────────
    # 衝突解決
    # ────────────────────────────────────────────────────────
    def resolve(
        self,
        paths: Dict[int, "BuiltPath"],
        conflicts: List[ConflictEvent],
    ) -> List[AvoidanceAction]:
        """
        根據衝突事件修改路徑。

        策略:
        - altitude: 在衝突區段升高/降低其中一架 UAV 的高度
        - temporal: 在衝突前插入 loiter 等待航點延遲通過

        Args:
            paths: {uav_id: BuiltPath}，會被 **就地修改**
            conflicts: detect() 的回傳
        Returns:
            所有避撞動作列表
        """
        if not conflicts:
            return []

        actions: List[AvoidanceAction] = []

        if self.strategy == 'altitude':
            actions = self._resolve_altitude(paths, conflicts)
        elif self.strategy == 'temporal':
            # ⚠ temporal 是 no-op（見模組 docstring）：不再呼叫 _resolve_temporal，
            #   大聲報錯並退回 altitude，而不是回報一堆沒有效果的動作
            logger.error("避撞策略 'temporal' 無效（插入的 loiter 航點貢獻 0 秒延遲），"
                         "改用 altitude")
            actions = self._resolve_altitude(paths, conflicts)
        else:
            logger.warning(f"未知避撞策略: {self.strategy}，使用 altitude")
            actions = self._resolve_altitude(paths, conflicts)

        logger.info(f"避撞解決完成: {len(actions)} 個動作")
        return actions

    def _resolve_altitude(
        self,
        paths: Dict[int, "BuiltPath"],
        conflicts: List[ConflictEvent],
    ) -> List[AvoidanceAction]:
        """高度錯開策略：衝突區段中，ID 較大的 UAV 升高 alt_offset_m"""
        actions = []
        # 收集每架 UAV 需要偏移的航點索引範圍
        uav_offsets: Dict[int, List[Tuple[int, int]]] = {}

        cap = self._effective_cap(paths)
        capped: set = set()       # 這一輪被累加上限擋下的 UAV
        n_unmovable = 0           # 這一輪衝突點落在起降段、抬不動的衝突數
        for uid, bp in paths.items():
            if uid not in self._base_alt and bp.waypoints:
                self._base_alt[uid] = max(w.alt for w in bp.waypoints)

        for cf in conflicts:
            # 選擇 ID 較大的 UAV 做高度偏移
            target_uid = max(cf.uav_a, cf.uav_b)
            target_wp_idx = cf.wp_idx_a if target_uid == cf.uav_a else cf.wp_idx_b
            bp = paths[target_uid]

            # 衝突點本身在起降段 ⇒ 抬不動（地面點被抬高 = 半空中起降）。
            # ⚠ 不可以先記動作再跳過：偏移窗會蓋到後面的作業航點，
            #   記了動作 ⇒ run() 的停滯保險絲永遠燒不斷 ⇒ 每輪都把
            #   +alt_offset 堆到作業航高上。
            if _is_ground_phase(bp.waypoints[target_wp_idx]):
                n_unmovable += 1
                continue

            # 擴展衝突區段：衝突航點 ± 緩衝（2 個航點），確保平滑過渡
            buffer = 2
            start_idx = max(0, target_wp_idx - buffer)
            end_idx = min(len(bp.waypoints) - 1, target_wp_idx + buffer)

            # 累加上限：這個區段目前已比基準高多少，再抬一次會不會超過
            # ⚠ 範圍要含前後各 2 點的**斜坡**，不能只看衝突窗：斜坡點在窗外，
            #   先前輪次抬過的高度會在這一輪被再疊一次（實測模糊檢查 300 組有
            #   1 組飛到 +105 m / 上限 100 m；見 test_collision_avoidance_loudness）
            # ⚠ 撞到就 continue，**不記動作** —— 否則停滯保險絲燒不斷
            _seg_peak = max(w.alt for w in bp.waypoints[
                max(0, start_idx - 2):min(len(bp.waypoints), end_idx + 3)])
            if (_seg_peak - self._base_alt[target_uid]) + self.alt_offset \
                    > cap + 1e-9:
                capped.add(target_uid)
                continue

            if target_uid not in uav_offsets:
                uav_offsets[target_uid] = []
            uav_offsets[target_uid].append((start_idx, end_idx))

            actions.append(AvoidanceAction(
                uav_id=target_uid,
                action_type='altitude_offset',
                wp_start_idx=start_idx,
                wp_end_idx=end_idx,
                alt_offset_m=self.alt_offset,
                conflict=cf,
            ))

        if n_unmovable:
            logger.warning(
                f"避撞: {n_unmovable} 個衝突落在起降段，高度錯開無能為力（已略過，"
                f"不記動作）")
        if capped:
            logger.error(
                f"避撞: UAV {sorted(capped)} 已達累加上限 {cap:.0f} m"
                f"（單次抬升 {self.alt_offset:.0f} m），停止抬升 ⇒ 這些衝突沒有解掉")

        # 合併重疊區段並套用高度偏移
        for uid, ranges in uav_offsets.items():
            bp = paths[uid]
            # 合併重疊區段
            merged = self._merge_ranges(ranges)

            for (s, e) in merged:
                n_wp = len(bp.waypoints)
                # 漸升段：s 前 2 點漸升
                ramp_before = max(0, s - 2)
                # 漸降段：e 後 2 點漸降
                ramp_after = min(n_wp - 1, e + 2)

                for i in range(ramp_before, ramp_after + 1):
                    if _is_ground_phase(bp.waypoints[i]):
                        # ⚠⚠ 起降段不可以被抬高：地面點被抬 60 m 就變成
                        #    「在半空中起飛／降落」，而其餘斷言（高度上限、
                        #    分隔距離）全都看不出來。VTOL 的垂直段更嚴重 ——
                        #    它靠「同經緯度、只差高度」定義，抬高會把整段剖面弄歪。
                        continue
                    if i < s:
                        # 漸升：線性插值
                        frac = (i - ramp_before) / max(s - ramp_before, 1)
                        bp.waypoints[i].alt += self.alt_offset * frac
                    elif i > e:
                        # 漸降
                        frac = 1.0 - (i - e) / max(ramp_after - e, 1)
                        bp.waypoints[i].alt += self.alt_offset * frac
                    else:
                        # 全偏移區段
                        bp.waypoints[i].alt += self.alt_offset

                    # 標記避撞段（使用 TRANSFER label，地圖顯示用）
                    # 保留原有 segment_type 不變，僅改高度

        return actions

    def _resolve_temporal(
        self,
        paths: Dict[int, "BuiltPath"],
        conflicts: List[ConflictEvent],
    ) -> List[AvoidanceAction]:
        """時間延遲策略：在衝突前插入 loiter 等待航點"""
        from core.dccpp.dccpp_path_builder import BuiltWaypoint, SegmentLabel

        actions = []
        already_delayed = set()

        for cf in conflicts:
            # 選擇 ID 較大的 UAV 做延遲
            target_uid = max(cf.uav_a, cf.uav_b)
            if target_uid in already_delayed:
                continue  # 同一架已延遲過，跳過

            target_wp_idx = cf.wp_idx_a if target_uid == cf.uav_a else cf.wp_idx_b
            bp = paths[target_uid]

            # 需要延遲的時間 = 衝突持續時間 + 安全餘量
            delay_s = cf.duration_s + self.min_sep / max(self.speed, 0.1)

            # 在衝突航點前方插入一個 loiter 點
            # loiter 位置 = 衝突前 2 個航點的位置
            insert_idx = max(0, target_wp_idx - 2)
            ref_wp = bp.waypoints[insert_idx]

            # 插入 loiter 航點（原地盤旋等待）
            loiter_wp = BuiltWaypoint(
                lat=ref_wp.lat,
                lon=ref_wp.lon,
                alt=ref_wp.alt,
                heading_compass_deg=ref_wp.heading_compass_deg,
                segment_type=SegmentLabel.TRANSFER,
            )
            bp.waypoints.insert(insert_idx, loiter_wp)
            already_delayed.add(target_uid)

            actions.append(AvoidanceAction(
                uav_id=target_uid,
                action_type='loiter_delay',
                wp_start_idx=insert_idx,
                wp_end_idx=insert_idx,
                delay_s=delay_s,
                conflict=cf,
            ))

        return actions

    @staticmethod
    def _merge_ranges(ranges: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """合併重疊的索引區段"""
        if not ranges:
            return []
        sorted_r = sorted(ranges, key=lambda r: r[0])
        merged = [sorted_r[0]]
        for s, e in sorted_r[1:]:
            if s <= merged[-1][1] + 1:
                merged[-1] = (merged[-1][0], max(merged[-1][1], e))
            else:
                merged.append((s, e))
        return merged

    # ────────────────────────────────────────────────────────
    # 一鍵執行：偵測 + 解衝突
    # ────────────────────────────────────────────────────────
    def run(
        self,
        paths: Dict[int, "BuiltPath"],
    ) -> Tuple[List[ConflictEvent], List[AvoidanceAction]]:
        """偵測衝突並自動解衝突，迭代到收斂為止。

        單輪不夠：每輪只抬「配對中 ID 較大的那架」，堆疊的機體要多輪才拉得開。
        三種收尾：收斂（無殘留）／本輪沒有任何可執行動作（停滯保險絲）／
        跑滿 max_rounds。後兩種都是**解不掉**，一律 ERROR —— 靜靜停下來和
        靜靜飛到 3 倍高度一樣糟。

        Args:
            paths: {uav_id: BuiltPath}，會被就地修改
        Returns:
            (conflicts, actions)：conflicts 是**第一輪**偵測到的衝突
            （殘留衝突看 `detect(paths)`），actions 是所有輪次累計。
        """
        self._base_alt = {}      # 基準高度只認這一次 run 開始時的路徑
        self.remaining_conflicts = []

        first_conflicts: Optional[List[ConflictEvent]] = None
        actions: List[AvoidanceAction] = []
        remaining: List[ConflictEvent] = []
        stalled = False

        for round_no in range(1, self.max_rounds + 1):
            remaining = self.detect(paths)
            if first_conflicts is None:
                first_conflicts = remaining
                if not remaining:
                    logger.info("無衝突事件，無需避撞")
                    return remaining, []
            if not remaining:
                break
            round_actions = self.resolve(paths, remaining)
            if not round_actions:
                # 保險絲：什麼都動不了就不要再空轉（也避免後面輪次把高度堆上去）
                stalled = True
                logger.error(
                    f"避撞停滯: 第 {round_no} 輪沒有任何可執行的動作，"
                    f"仍有 {len(remaining)} 個衝突沒有解掉")
                break
            actions.extend(round_actions)
        else:
            # 最後一輪解完還沒驗證過
            remaining = self.detect(paths)

        self.remaining_conflicts = list(remaining)
        if remaining and not stalled:
            logger.error(
                f"避撞跑滿 {self.max_rounds} 輪後仍有 {len(remaining)} 個衝突")
        elif not remaining:
            logger.info("所有衝突已成功解決 ✅")

        return first_conflicts or [], actions
