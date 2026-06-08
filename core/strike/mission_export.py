"""core.strike.mission_export — QGC WPL 110 任務匯出共用工具
==============================================================

集中 strike planner 共用的：
    * MissionItem  — MAVLink 任務項 dataclass（對應 QGC WPL 110 單行）
    * export_missions_qgc — 將 List[MissionItem] 寫成 .waypoints 檔

先前 swarm / advanced_swarm / vtol_swarm / recon_to_strike 四個 planner 各自
重複同一段「逐行寫 QGC WPL 110」迴圈（且都得繞道 import swarm_strike_planner
才能取得 MissionItem）。統一後：MissionItem 住在本中性模組，匯出迴圈只有一份。

相容性
------
為避免 break 既有 `from core.strike.swarm_strike_planner import MissionItem`，
swarm_strike_planner 仍 re-export MissionItem。新程式碼建議改用本模組。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from core.strike.geometry import MAV_FRAME_REL
from utils.file_io import create_waypoint_line, write_waypoints


@dataclass
class MissionItem:
    """MAVLink 任務項 (對應 QGC WPL 110 單行)"""
    cmd: int
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    comment: str = ''


# DO_SET_HOME 命令碼：seq=0 且為此指令時 current=1（QGC 慣例）
_DO_SET_HOME = 179


def export_missions_qgc(
    missions: Sequence[MissionItem],
    fpath: str,
    *,
    home_cmd: int = _DO_SET_HOME,
    frame: int = MAV_FRAME_REL,
) -> bool:
    """將 List[MissionItem] 寫成單一 QGC WPL 110 .waypoints 檔。

    收斂 swarm/advanced/vtol/recon 四個 planner 中逐行相同的匯出迴圈，與原迴圈
    逐位元等價：
      - 首行固定 'QGC WPL 110'
      - seq=0 且 cmd==home_cmd → current=1，其餘 current=0
      - frame 固定、autocontinue=1，param1..4 原樣傳入 create_waypoint_line

    檔名、目錄建立與 briefing 仍由各 planner 自理（本函式只負責單機單檔內容）。

    參數
    ----
    missions : Sequence[MissionItem]
        單架 UAV 的任務序列。
    fpath : str
        輸出檔完整路徑。
    home_cmd : int
        判定 seq=0 current 旗標用的 DO_SET_HOME 命令碼（預設 179）。
    frame : int
        MAV_FRAME（預設 MAV_FRAME_GLOBAL_RELATIVE_ALT = 3）。

    回傳
    ----
    bool：write_waypoints 的結果（成功為 True）。
    """
    lines = ['QGC WPL 110']
    for seq, item in enumerate(missions):
        current = 1 if seq == 0 and item.cmd == home_cmd else 0
        lines.append(create_waypoint_line(
            seq=seq, command=item.cmd,
            lat=item.lat, lon=item.lon, alt=item.alt,
            param1=item.param1, param2=item.param2,
            param3=item.param3, param4=item.param4,
            frame=frame,
            current=current, autocontinue=1,
        ))
    return write_waypoints(fpath, lines)
