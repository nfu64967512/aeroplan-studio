"""
VTOL (QuadPlane) 群飛任務匯出器

根據 DCCPP 組裝路徑 (AssembledPath) 為每架 UAV 產生完整的
QGC WPL 110 格式航點檔案，適用於 4+1 VTOL 混飛模式。

任務流程：
    Home → VTOL_TAKEOFF → DO_VTOL_TRANSITION(FW) → 巡航航點 →
    DO_VTOL_TRANSITION(MC) → VTOL_LAND

MAVLink 指令對照：
    MAV_CMD_DO_SET_HOME        (179) - Home 佔位
    MAV_CMD_DO_CHANGE_SPEED    (178) - 空速設定
    MAV_CMD_NAV_VTOL_TAKEOFF    (84) - VTOL 垂直起飛
    MAV_CMD_DO_VTOL_TRANSITION(3000) - 飛行模式轉換 (FW/MC)
    MAV_CMD_NAV_WAYPOINT        (16) - 巡航航點
    MAV_CMD_NAV_VTOL_LAND       (85) - VTOL 垂直降落
"""

from __future__ import annotations

import os
import math
from typing import List, Optional

from utils.file_io import create_waypoint_line, write_waypoints
from utils.logger import get_logger

logger = get_logger()

# MAVLink 座標系
_FRAME_GLOBAL_REL_ALT = 3   # MAV_FRAME_GLOBAL_RELATIVE_ALT
_FRAME_MISSION = 2          # MAV_FRAME_MISSION（DO 指令用）

# MAVLink 命令碼
_CMD_NAV_WAYPOINT       = 16
_CMD_NAV_VTOL_TAKEOFF   = 84     # 垂直起飛：param7(alt) = 目標高度
_CMD_NAV_VTOL_LAND      = 85     # 垂直降落：param5(lat), param6(lon)
_CMD_DO_CHANGE_SPEED    = 178    # param1=速度類型(0=AS,1=GS), param2=速度(m/s), param3=油門(-1=不變)
_CMD_DO_SET_HOME        = 179    # param5(lat), param6(lon), param7(alt)
_CMD_DO_VTOL_TRANSITION = 3000   # param1: 3=MC(多旋翼), 4=FW(固定翼)


class VTOLMissionExporter:
    """VTOL (QuadPlane) 群飛任務匯出器

    將 DCCPP 規劃結果中的每架 UAV 路徑轉換為 VTOL 專用
    QGC WPL 110 航點序列，支援垂直起降與固定翼巡航混合模式。

    Parameters
    ----------
    transition_alt_m : float
        VTOL 轉換安全高度（公尺）。起飛爬升到此高度後才觸發 FW 轉換，
        降落前先在此高度轉回 MC 再執行垂直降落。
    cruise_speed_ms : float
        固定翼巡航空速（m/s），插入 DO_CHANGE_SPEED 設定。
    turn_radius_m : float
        巡航航點 acceptance radius（公尺），寫入 param2。
    """

    def __init__(self,
                 transition_alt_m: float = 40.0,
                 cruise_speed_ms: float = 15.0,
                 turn_radius_m: float = 50.0,
                 explicit_transitions: bool = False):
        """
        Parameters
        ----------
        explicit_transitions : bool
            是否在任務裡明確插入兩筆 ``MAV_CMD_DO_VTOL_TRANSITION`` (3000)。

            預設 **False**，依 ArduPilot 官方 QuadPlane AUTO Missions 文件：

              * ``NAV_VTOL_TAKEOFF`` 到達高度後飛向下一航點時，飛控本來就會
                視需要自行轉固定翼 —— 明確插 param1=4 是多餘的。
              * ArduPlane 4.1+ 的 ``NAV_VTOL_LAND`` 預設保持固定翼進場、
                接近才 airbrake 轉多旋翼。提早插 param1=3 會強制轉 MC，
                抵銷掉那個省電設計。

            留這個開關是為了讓「預設不插」這件事可被對照驗證（沒有對照組的
            斷言可能只是永遠不可能觸發的假綠燈）。
        """
        self.transition_alt_m = transition_alt_m
        self.cruise_speed_ms = cruise_speed_ms
        self.turn_radius_m = turn_radius_m
        self.explicit_transitions = bool(explicit_transitions)

    # ─────────────────────────────────────────────────────────────
    #  核心：為單架 UAV 生成 VTOL 航點序列
    # ─────────────────────────────────────────────────────────────
    def generate_waypoint_lines(
        self,
        waypoints: list,
        altitude: float = 100.0,
        uav_id: int = 0,
        alt_offset_m: float = 0.0,
    ) -> List[str]:
        """為單架 UAV 生成 QGC WPL 110 航點序列。

        Parameters
        ----------
        waypoints : list[AssembledWaypoint]
            DCCPP 組裝路徑的航點列表（含 segment_type）。
        altitude : float
            預設巡航高度（公尺），航點自身高度為 0 時使用。
        uav_id : int
            UAV 編號（僅用於日誌）。
        alt_offset_m : float
            (飛控 home − 航點高度基準)。HARVEST 路徑的 alt 以規劃 home 為基準，飛控解鎖後 home 是這架的
            起飛坪 ⇒ 送出的相對高度要減掉這個差（路徑的 ``fc_alt_offset_m``；平地為 0）。

        Returns
        -------
        list[str]
            QGC WPL 110 格式的航點行列表。
        """
        lines: List[str] = ['QGC WPL 110']
        seq = 0

        if not waypoints:
            return lines

        # ── (0) Home 佔位 ───────────────────────────────────────
        #   seq=0 固定為 DO_SET_HOME
        #   param5=lat, param6=lon, param7=alt(地面)
        home_wp = waypoints[0]
        lines.append(create_waypoint_line(
            seq=seq, command=_CMD_DO_SET_HOME,
            lat=home_wp.lat, lon=home_wp.lon, alt=0.0,
            current=0, autocontinue=1,
        ))
        seq += 1

        # ── (1) DO_CHANGE_SPEED — 設定巡航空速 ─────────────────
        #   param1=1.0 (空速), param2=速度(m/s), param3=-1(油門不變)
        lines.append(create_waypoint_line(
            seq=seq, command=_CMD_DO_CHANGE_SPEED,
            frame=_FRAME_MISSION,
            param1=1.0, param2=self.cruise_speed_ms, param3=-1.0,
            current=0, autocontinue=1,
        ))
        seq += 1

        # ── (2) NAV_VTOL_TAKEOFF (84) — VTOL 垂直起飛 ──────────
        #   param7(alt) = 轉換安全高度
        #   使用 Home 座標作為起飛點
        #   飛控會在此座標垂直爬升至 alt 後繼續下一個 waypoint
        lines.append(create_waypoint_line(
            seq=seq, command=_CMD_NAV_VTOL_TAKEOFF,
            frame=_FRAME_GLOBAL_REL_ALT,
            lat=home_wp.lat, lon=home_wp.lon,
            alt=self.transition_alt_m,
            current=0, autocontinue=1,
        ))
        seq += 1

        # ── (3) DO_VTOL_TRANSITION → 固定翼 (FW, state=4) ──────
        #   ⚠ 預設**不發**：NAV_VTOL_TAKEOFF 之後飛向下一航點時飛控自己會轉。
        #   只有 explicit_transitions=True 才插回來（對照組用）。
        if self.explicit_transitions:
            lines.append(create_waypoint_line(
                seq=seq, command=_CMD_DO_VTOL_TRANSITION,
                frame=_FRAME_MISSION,
                param1=4.0,        # MAV_VTOL_STATE_FW
                current=0, autocontinue=1,
            ))
            seq += 1

        # ── (4) 巡航航點 — NAV_WAYPOINT (16) ───────────────────
        #   跳過第一個 TAKEOFF 地面點（已由 Home + VTOL_TAKEOFF 處理）
        #   param2 = acceptance_radius（轉彎半徑）
        land_wp = waypoints[-1]

        def _same_spot(a, b) -> bool:
            return abs(a.lat - b.lat) < 1e-9 and abs(a.lon - b.lon) < 1e-9

        for idx, wp in enumerate(waypoints):
            seg_name = getattr(getattr(wp, 'segment_type', None), 'name', '')

            # 起飛段第一點 (地面) 已作為 Home，跳過
            if seg_name == 'TAKEOFF' and idx == 0:
                continue

            # 降落段最後一點保留給 VTOL_LAND，跳過
            if seg_name == 'LANDING' and idx == len(waypoints) - 1:
                continue

            # ⚠⚠ 起降段的**純垂直**點（與 Home／降落點同經緯度、只差高度）
            #    不可以發成 NAV_WAYPOINT：垂直爬升由 NAV_VTOL_TAKEOFF 做、
            #    垂直下降由 NAV_VTOL_LAND 做。發出來的話 NAV_VTOL_LAND 的
            #    進場段會變成 0 m，真機 PreArm「VTOL land too short」直接
            #    擋住解鎖。規劃器仍保留這些點，3D 顯示的垂直段才畫得出來。
            if seg_name in ('TAKEOFF', 'LANDING') and (
                    _same_spot(wp, home_wp) or _same_spot(wp, land_wp)):
                continue

            wp_alt = (wp.alt - alt_offset_m) if wp.alt > 0 else altitude

            # 作業段密集航點：縮小 acceptance_radius 避免跳點
            accept_r = self.turn_radius_m
            if seg_name == 'OPERATION' and idx < len(waypoints) - 1:
                nxt = waypoints[idx + 1]
                dist_next = math.hypot(
                    (nxt.lat - wp.lat) * 111320,
                    (nxt.lon - wp.lon) * 111320 * math.cos(math.radians(wp.lat))
                )
                accept_r = max(
                    self.turn_radius_m / 10.0,
                    min(dist_next / 3.0, self.turn_radius_m)
                )

            lines.append(create_waypoint_line(
                seq=seq, command=_CMD_NAV_WAYPOINT,
                frame=_FRAME_GLOBAL_REL_ALT,
                lat=wp.lat, lon=wp.lon, alt=wp_alt,
                param2=accept_r,
                current=0, autocontinue=1,
            ))
            seq += 1

        # ── (5) DO_VTOL_TRANSITION → 多旋翼 (MC, state=3) ──────
        #   ⚠ 預設**不發**：4.1+ 的 NAV_VTOL_LAND 會自己固定翼進場再 airbrake。
        if self.explicit_transitions:
            lines.append(create_waypoint_line(
                seq=seq, command=_CMD_DO_VTOL_TRANSITION,
                frame=_FRAME_MISSION,
                param1=3.0,        # MAV_VTOL_STATE_MC
                current=0, autocontinue=1,
            ))
            seq += 1

        # ── (6) NAV_VTOL_LAND (85) — VTOL 垂直降落 ─────────────
        #   param5=lat, param6=lon, param7=alt(0 = 觸地)
        #   使用最後一個降落段航點座標；若無，回到 Home
        last_wp = waypoints[-1]
        land_lat = last_wp.lat
        land_lon = last_wp.lon

        lines.append(create_waypoint_line(
            seq=seq, command=_CMD_NAV_VTOL_LAND,
            frame=_FRAME_GLOBAL_REL_ALT,
            lat=land_lat, lon=land_lon, alt=0.0,
            current=0, autocontinue=1,
        ))
        seq += 1

        logger.info(
            f'[VTOLExporter] UAV-{uav_id}: {seq} 航點 '
            f'(轉換高度={self.transition_alt_m}m, '
            f'巡航速度={self.cruise_speed_ms}m/s)'
        )
        return lines

    # ─────────────────────────────────────────────────────────────
    #  批次匯出：群飛所有 UAV
    # ─────────────────────────────────────────────────────────────
    def export_all(
        self,
        assembled_paths: dict,
        export_dir: str,
        altitude: float = 100.0,
        speed: float = 15.0,
    ) -> List[str]:
        """將 DCCPP assembled_paths 批次匯出為 VTOL 航點檔案。

        Parameters
        ----------
        assembled_paths : dict[int, AssembledPath]
            DCCPP 組裝路徑，key=uav_id。
        export_dir : str
            匯出目錄。
        altitude : float
            預設巡航高度（公尺）。
        speed : float
            巡航速度（m/s），覆寫 self.cruise_speed_ms。

        Returns
        -------
        list[str]
            匯出的檔案名稱列表。
        """
        self.cruise_speed_ms = speed
        exported_files: List[str] = []

        for uav_id, apath in assembled_paths.items():
            if not apath.waypoints:
                continue

            wp_lines = self.generate_waypoint_lines(
                waypoints=apath.waypoints,
                altitude=altitude,
                uav_id=uav_id,
                alt_offset_m=float(getattr(apath, 'fc_alt_offset_m', 0.0) or 0.0),
            )

            filename = f"VTOL_UAV{uav_id}_高度{altitude:.0f}m.waypoints"
            filepath = os.path.join(export_dir, filename)
            write_waypoints(filepath, wp_lines)
            exported_files.append(filename)
            logger.info(f'[VTOLExporter] 匯出: {filepath}')

        # ── 匯出簡報 ──
        briefing_path = os.path.join(export_dir, "VTOL_briefing.txt")
        with open(briefing_path, 'w', encoding='utf-8') as f:
            f.write("VTOL (4+1 QuadPlane) 群飛任務簡報\n")
            f.write("=" * 45 + "\n\n")
            f.write("匯出模式：4+1 VTOL 混飛模式\n")
            f.write(f"無人機數量：{len(assembled_paths)}\n")
            f.write(f"巡航高度：{altitude:.0f}m\n")
            f.write(f"巡航速度：{speed:.1f}m/s\n")
            f.write(f"轉換安全高度：{self.transition_alt_m:.0f}m\n")
            f.write(f"轉彎半徑：{self.turn_radius_m:.0f}m\n\n")
            # ⚠ 摘要必須描述**實際發出去的**序列。宣稱有插轉換指令而檔案裡
            #   沒有（或反過來），讀簡報的人會照著錯的流程判讀飛行紀錄。
            f.write("任務流程：\n")
            f.write("  VTOL_TAKEOFF(84) → 垂直爬升至轉換高度\n")
            if self.explicit_transitions:
                f.write("  DO_VTOL_TRANSITION(3000,4) → 切換固定翼巡航\n")
            else:
                f.write("  （飛往下一航點時飛控自行轉固定翼）\n")
            f.write("  NAV_WAYPOINT(16) × N → 掃描/巡航航點\n")
            if self.explicit_transitions:
                f.write("  DO_VTOL_TRANSITION(3000,3) → 切換多旋翼減速\n")
            else:
                f.write("  （NAV_VTOL_LAND 自行固定翼進場後 airbrake 轉多旋翼）\n")
            f.write("  VTOL_LAND(85) → 垂直降落\n\n")

            for uav_id, apath in assembled_paths.items():
                total_len = apath.total_length_m
                f.write(
                    f"UAV-{uav_id}：{len(apath.waypoints)} 原始航點，"
                    f"總長 {total_len:.0f}m\n"
                    f"  起飛 {getattr(apath, 'takeoff_length_m', 0.0):.0f}m + "
                    f"進場 {getattr(apath, 'entry_length_m', 0.0):.0f}m + "
                    f"作業 {getattr(apath, 'operation_length_m', 0.0):.0f}m + "
                    f"轉移 {getattr(apath, 'transfer_length_m', 0.0):.0f}m + "
                    f"降落 {getattr(apath, 'landing_length_m', 0.0):.0f}m\n"
                )

        exported_files.append("VTOL_briefing.txt")
        return exported_files

    # ─────────────────────────────────────────────────────────────
    #  輔助：將匯出結果轉為 Cesium 視覺化所需的 JSON 資料
    # ─────────────────────────────────────────────────────────────
    def to_cesium_data(
        self,
        assembled_paths: dict,
        altitude: float = 100.0,
        colors: Optional[dict] = None,
    ) -> List[dict]:
        """將群飛路徑轉為 Cesium drawVTOLSwarmPaths 所需資料格式。

        colors: ``{uav_id: '#rrggbb'}``。GUI 要傳 3D 圖例層用的同一份
                （``cesium_map_widget.swarm_drone_colors``），否則同一架機在圖例與條帶層
                顏色不同；沒給時退回本函式自己的依機號色表（舊行為）。

        每架 UAV 回傳：
        {
            uav_id, color,
            home: {lat, lon},
            transition_alt: float,
            takeoff_segment:  [{lat, lon, alt}, ...],  (垂直起飛段)
            cruise_segment:   [{lat, lon, alt}, ...],  (固定翼巡航段)
            landing_segment:  [{lat, lon, alt}, ...],  (垂直降落段)
            fw_transition_pt: {lat, lon, alt},          (FW 轉換點)
            mc_transition_pt: {lat, lon, alt},          (MC 轉換點)
        }
        """
        # 每架 UAV 的顏色
        _COLORS = ['#e53935', '#1e88e5', '#43a047', '#fb8c00',
                    '#8e24aa', '#00acc1', '#d81b60', '#fdd835']

        result = []
        for uav_id, apath in assembled_paths.items():
            if not apath.waypoints:
                continue

            color = ((colors or {}).get(uav_id)
                     or _COLORS[uav_id % len(_COLORS)])
            wps = apath.waypoints
            home = wps[0]

            # 分離航段
            takeoff_pts = []    # 垂直起飛段（地面→轉換高度）
            cruise_pts = []     # 固定翼巡航段
            landing_pts = []    # 垂直降落段（轉換高度→地面）

            for wp in wps:
                seg = getattr(getattr(wp, 'segment_type', None), 'name', '')
                wp_alt = wp.alt if wp.alt > 0 else altitude
                pt = {'lat': wp.lat, 'lon': wp.lon, 'alt': wp_alt}

                if seg == 'TAKEOFF':
                    takeoff_pts.append(pt)
                elif seg == 'LANDING':
                    landing_pts.append(pt)
                else:
                    cruise_pts.append(pt)

            # 起飛垂直段：Home(地面) → Home(轉換高度)
            vtol_takeoff = [
                {'lat': home.lat, 'lon': home.lon, 'alt': 0},
                {'lat': home.lat, 'lon': home.lon, 'alt': self.transition_alt_m},
            ]
            # 如果有 TAKEOFF 爬升點，串接上去
            if takeoff_pts:
                vtol_takeoff.append(takeoff_pts[-1])

            # FW 轉換點 = 起飛段最後一點（或 Home 在轉換高度）
            fw_pt = takeoff_pts[-1] if takeoff_pts else {
                'lat': home.lat, 'lon': home.lon, 'alt': self.transition_alt_m
            }

            # 降落段：**最後一條進場腿**（巡航段的最後一點 → 降落點上空）→ 垂直落地。
            # ⚠ 不可從 landing_pts[0] 起畫：那是 LANDING 段的第一點（= 最後一條條帶的終點，
            #   HARVEST 實測離降落坪 0.6–1.7 km），直線拉到降落點上空會畫出一條實際不會飛的
            #   長弦（2026-10-07 截圖裡那三條長黃線）。真正的返場路徑已經在 cruise_segment 裡
            #   （它收了 landing_pts[:-1]），這裡只接上它的最後一點。
            land_wp = wps[-1]
            land_alt = land_wp.alt if land_wp.alt > 0 else self.transition_alt_m
            mc_pt = {'lat': land_wp.lat, 'lon': land_wp.lon, 'alt': land_alt}
            approach_end = (landing_pts[-2] if len(landing_pts) > 1
                            else (cruise_pts[-1] if cruise_pts else None))
            vtol_landing = [approach_end] if approach_end else []
            vtol_landing.append(mc_pt)
            vtol_landing.append({
                'lat': land_wp.lat, 'lon': land_wp.lon, 'alt': 0
            })

            result.append({
                'uav_id': uav_id,
                'color': color,
                'home': {'lat': home.lat, 'lon': home.lon},
                'transition_alt': self.transition_alt_m,
                'takeoff_segment': vtol_takeoff,
                'cruise_segment': (takeoff_pts[1:] if len(takeoff_pts) > 1 else []) + cruise_pts + (landing_pts[:-1] if len(landing_pts) > 1 else landing_pts),
                'landing_segment': vtol_landing,
                'fw_transition_pt': fw_pt,
                'mc_transition_pt': mc_pt,
            })

        return result
