"""飛行中閉環時間到達修正（closed-loop Time-on-Target）。

開環規劃（S 機動 / 盤旋補時 / 起飛錯開）會把各機路徑長度大致拉齊，但實飛仍有
爬升、轉彎、空速控制延遲造成的殘差 → 命中時刻散開 10~40s。本控制器在終端段
持續量測各機到目標的即時 ETA，回算每機應有的速度（送 DO_CHANGE_SPEED），把全機
收斂到同一命中時刻，將實飛 spread 壓到秒級。

單一職責：只負責「由各機現況算出應發的目標速度」，不碰 MAVLink；GCS 端負責量測與下令。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from core.strike.geometry import haversine


@dataclass
class AircraftState:
    sysid: int
    lat: float
    lon: float
    rel_alt: float = 0.0


class TimeOnTargetController:
    def __init__(self, target_lat: float, target_lon: float,
                 v_min: float, v_max: float,
                 terminal_range_m: float = 4000.0,
                 impact_buffer_s: float = 2.0) -> None:
        if not (0 < v_min < v_max):
            raise ValueError(f"need 0 < v_min < v_max, got {v_min}, {v_max}")
        self.tlat, self.tlon = target_lat, target_lon
        self.v_min, self.v_max = v_min, v_max
        self.terminal_range = terminal_range_m
        self.impact_buffer = impact_buffer_s
        self._t_impact: Optional[float] = None   # 鎖定的共同命中時刻（絕對秒）

    def remaining_m(self, st: AircraftState) -> float:
        return haversine(st.lat, st.lon, self.tlat, self.tlon)

    @property
    def locked_impact_time(self) -> Optional[float]:
        return self._t_impact

    def compute_speeds(self, states: List[AircraftState], t_now: float) -> Dict[int, float]:
        """回傳 {sysid: 目標速度 m/s}。僅對「已進入終端段」(≤ terminal_range) 的機作用；
        終端機 < 2 架時回空（仍由開環飛）。"""
        rem = {s.sysid: self.remaining_m(s) for s in states
               if self.remaining_m(s) <= self.terminal_range}
        if len(rem) < 2:
            return {}
        # 共同命中時刻 = 各機「以 v_max 的最早可達時刻」之最大值（最遠機決定）+ buffer。
        # 鎖定後只允許往後推（feasibility：每機都能在不超過 v_max 下達到該時刻）。
        earliest_common = max(t_now + r / self.v_max for r in rem.values())
        if self._t_impact is None:
            self._t_impact = earliest_common + self.impact_buffer
        else:
            self._t_impact = max(self._t_impact, earliest_common)
        cmds: Dict[int, float] = {}
        for sid, r in rem.items():
            t_left = max(self._t_impact - t_now, 0.5)
            cmds[sid] = max(self.v_min, min(self.v_max, r / t_left))
        return cmds
