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

2. **時間延遲 (Temporal Deconfliction)**：
   在衝突區段前方插入一個 loiter / 盤旋等待航點，讓後起飛的 UAV 延遲通過，
   使兩機在同一位置的時間錯開。適用於高度受限的情境。

呼叫範例
--------
    avoidance = CollisionAvoidance(
        min_separation_m=100.0,
        alt_offset_m=30.0,
        time_step_s=1.0,
        cruise_speed_mps=40.0,
    )
    conflicts = avoidance.detect(all_built_paths)
    avoidance.resolve(all_built_paths, conflicts)
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


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
    R = 6371000.0
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

class CollisionAvoidance:
    """多機避撞偵測與解衝突"""

    def __init__(
        self,
        min_separation_m: float = 100.0,
        alt_offset_m: float = 30.0,
        time_step_s: float = 1.0,
        cruise_speed_mps: float = 40.0,
        strategy: str = 'altitude',   # 'altitude' | 'temporal'
    ):
        self.min_sep = float(min_separation_m)
        self.alt_offset = float(alt_offset_m)
        self.dt = float(time_step_s)
        self.speed = float(cruise_speed_mps)
        self.strategy = strategy

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
            actions = self._resolve_temporal(paths, conflicts)
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
        from core.dccpp.dccpp_path_builder import BuiltWaypoint, SegmentLabel

        actions = []
        # 收集每架 UAV 需要偏移的航點索引範圍
        uav_offsets: Dict[int, List[Tuple[int, int]]] = {}

        for cf in conflicts:
            # 選擇 ID 較大的 UAV 做高度偏移
            target_uid = max(cf.uav_a, cf.uav_b)
            target_wp_idx = cf.wp_idx_a if target_uid == cf.uav_a else cf.wp_idx_b
            bp = paths[target_uid]

            # 擴展衝突區段：衝突航點 ± 緩衝（2 個航點），確保平滑過渡
            buffer = 2
            start_idx = max(0, target_wp_idx - buffer)
            end_idx = min(len(bp.waypoints) - 1, target_wp_idx + buffer)

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
        """偵測衝突並自動解衝突。

        Args:
            paths: {uav_id: BuiltPath}，會被就地修改
        Returns:
            (conflicts, actions)
        """
        conflicts = self.detect(paths)
        if not conflicts:
            logger.info("無衝突事件，無需避撞")
            return conflicts, []
        actions = self.resolve(paths, conflicts)

        # 驗證：重新偵測確認衝突已解決
        remaining = self.detect(paths)
        if remaining:
            logger.warning(
                f"避撞後仍有 {len(remaining)} 個衝突（可能需要多輪迭代）"
            )
        else:
            logger.info("所有衝突已成功解決 ✅")

        return conflicts, actions
