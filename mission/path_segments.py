"""把一條 assembled path 依航段標籤（SegmentLabel）切成可繪製的連續段。

⚠⚠ 一條腿（wp[i] → wp[i+1]）歸哪個標籤，**兩個規劃器的慣例相反**：

  * HARVEST（hv_dccpp 轉接器，``HvBuiltPath``）：腿歸**出發點** wp[i] 的標籤
    （``HvBuiltPath`` docstring：「第 i 段計入 PHASE_TO_SEGMENT[wp[i].phase]」）。
  * 舊版 DCCPP（``core/dccpp/dccpp_path_builder.py``、``dccpp_path_assembler.py``）：
    ENTRY/TRANSFER 航點只是 Dubins 腿的**終點**，那條腿算進 entry/transfer 長度 ⇒
    腿歸**終點** wp[i+1] 的標籤。

所以標籤一換，**邊界那條腿**要依來源接到前一段（start）或後一段（end）。用錯慣例的後果：

  * 舊寫法兩邊都不接 ⇒ 1 點的段被繪圖端丟掉。HARVEST 縮點後每條條帶只剩 1 個 OPERATION 點
    ⇒ **18 條條帶一條都沒畫**（2026-10-07：畫出 24.20 km，計畫 78.42 km）；舊版 DCCPP 的
    U 型轉彎與進場腿（1 點 TRANSFER/ENTRY）同樣從來沒畫過。
  * 對舊版 DCCPP 套 start 慣例 ⇒ U 型轉彎與返場腿被畫成「作業」、進場腿被畫成「起飛」，
    TRANSFER 全變零長度（2026-10-07 審查實測：作業 9592 → 12632 m）。
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Tuple

SEGMENT_KEYS = {
    'TAKEOFF': 'takeoff_paths',
    'ENTRY': 'entry_paths',
    'OPERATION': 'operation_paths',
    'TRANSFER': 'transfer_paths',
    'LANDING': 'landing_paths',
}

Point = Tuple[float, float, float]


def leg_owner_for(apath: Any) -> str:
    """HARVEST 的路徑（帶構型 ``cls`` / ``hv_uav_id``）用 'start'，其餘（舊版 DCCPP）用 'end'。"""
    if getattr(apath, 'cls', None) or getattr(apath, 'hv_uav_id', None):
        return 'start'
    return 'end'


def group_waypoints_by_segment(waypoints: Iterable[Any], default_alt: float,
                               leg_owner: str = 'start') -> Dict[str, List[List[Point]]]:
    """回傳 ``{'takeoff_paths': [...], 'entry_paths': [...], ...}``，每個值是點序列的清單。

    leg_owner: 'start' = 腿歸出發點的標籤（HARVEST）；'end' = 腿歸終點的標籤（舊版 DCCPP）。
    點的格式是 ``(lat, lon, alt)``；航點沒有 ``alt`` 屬性時用 ``default_alt``。
    標籤不在 SEGMENT_KEYS 裡的段會被略過（與舊行為相同）。
    """
    if leg_owner not in ('start', 'end'):
        raise ValueError(f"leg_owner 必須是 'start' 或 'end'，收到 {leg_owner!r}")
    out: Dict[str, List[List[Point]]] = {k: [] for k in SEGMENT_KEYS.values()}
    current: List[Point] = []
    label = None

    def _flush():
        key = SEGMENT_KEYS.get(getattr(label, 'name', None))
        # 所有點完全重合（含高度）的段沒有任何可畫的長度，只會在圖例多一列空項
        # （例：舊版多旋翼的 ENTRY 點與起飛末點同座標）。垂直起降段高度不同，不受影響。
        if key is not None and current and (len(current) == 1 or len(set(current)) > 1):
            out[key].append(current)

    for wp in waypoints:
        pt = (wp.lat, wp.lon, getattr(wp, 'alt', default_alt))
        if wp.segment_type != label:
            prev = current[-1] if current else None
            if current and label is not None:
                if leg_owner == 'start':
                    current.append(pt)          # 邊界腿收進前一段
                _flush()
            # 'end'：邊界腿歸新的一段 ⇒ 新段從前一點開始
            current = [prev, pt] if (leg_owner == 'end' and prev is not None) else [pt]
            label = wp.segment_type
        else:
            current.append(pt)
    if current and label is not None:
        _flush()
    return out


def segments_for_path(apath: Any, default_alt: float) -> Dict[str, List[List[Point]]]:
    """依路徑來源自動選慣例的便利入口（GUI 用這個）。"""
    return group_waypoints_by_segment(apath.waypoints, default_alt, leg_owner_for(apath))


def imaging_path(apath: Any) -> Tuple[List[Optional[Tuple[float, float]]], str]:
    """覆蓋率驗算用：這條路徑上**相機真的在成像**的折線；不相連的成像段之間以 ``None`` 斷開。

    回傳 ``(path, 判準說明)``，路徑餵給 ``core.recon.verify_coverage``。

    * 有 ``sensor_on``（HARVEST）：**腿歸出發點**——航點 i 的 ``sensor_on`` 描述 wp[i] → wp[i+1]
      這一條腿。所以成像腿 = 每個 ``sensor_on`` 航點與它的**下一個**航點（條帶終點）。
      ⚠⚠ 只取 ``sensor_on`` 的**航點**不夠：縮點後每條條帶只剩起點帶旗標，終點是別的標籤 ⇒ 條帶
      被壓成一個點，點與點之間又被斜線連起來（2026-10-09：GUI 算 75.06%，實際路徑 100.00%）。
    * 沒有任何 ``sensor_on``（舊版 DCCPP）：取 OPERATION 標籤的航點，**行為與以前相同**
      （舊版標籤歸終點，那套本來就不是這個問題；要改要另外量過）。
    """
    wps = list(getattr(apath, 'waypoints', None) or [])
    if any(bool(getattr(w, 'sensor_on', False)) for w in wps):
        out: List[Optional[Tuple[float, float]]] = []
        in_run = False
        for i, w in enumerate(wps):
            if not bool(getattr(w, 'sensor_on', False)):
                in_run = False
                continue
            if not in_run:
                if out:
                    out.append(None)
                out.append((float(w.lat), float(w.lon)))
                in_run = True
            if i + 1 < len(wps):
                nxt = wps[i + 1]
                out.append((float(nxt.lat), float(nxt.lon)))
        return out, 'sensor_on（成像腿＝出發點 sensor_on 的那條腿）'
    pts = [(float(w.lat), float(w.lon)) for w in wps
           if getattr(getattr(w, 'segment_type', None), 'name', '') == 'OPERATION']
    return pts, 'segment_type==OPERATION（此規劃器不帶 sensor_on）'
