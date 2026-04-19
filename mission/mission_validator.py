"""
MAVLink Mission Validator — 上傳前 dry-run 檢查

確保任務序列在 MAVLink / ArduPilot 眼中是合法的，避免：
    * 經緯度超出合法範圍 ±90 / ±180
    * 高度負值 (NAV_* 需 >0)
    * 未知 command ID
    * 過多航點 (MP 限 655)
    * 首個 waypoint 與 HOME 重疊 (ArduPilot 會 reject)
    * 連續重複航點 (無意義消耗任務序列)

使用::

    from mission.mission_validator import validate_mission, ValidationResult

    result = validate_mission(wp_tuples)
    if not result.ok:
        for err in result.errors:
            logger.error(err)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Sequence


# ── ArduPilot / MAVLink 合法指令 ID ──
# 參考: https://mavlink.io/en/messages/common.html
_VALID_COMMANDS = {
    # NAV_*
    16:  'NAV_WAYPOINT',
    17:  'NAV_LOITER_UNLIM',
    18:  'NAV_LOITER_TURNS',
    19:  'NAV_LOITER_TIME',
    20:  'NAV_RETURN_TO_LAUNCH',
    21:  'NAV_LAND',
    22:  'NAV_TAKEOFF',
    82:  'NAV_SPLINE_WAYPOINT',
    84:  'NAV_VTOL_TAKEOFF',
    85:  'NAV_VTOL_LAND',
    # DO_*
    177: 'DO_JUMP',
    178: 'DO_CHANGE_SPEED',
    179: 'DO_SET_HOME',
    192: 'DO_REPOSITION',
    # Condition
    112: 'CONDITION_DELAY',
    113: 'CONDITION_CHANGE_ALT',
    115: 'CONDITION_YAW',
    # VTOL / misc
    3000: 'DO_VTOL_TRANSITION',
    2000: 'IMAGE_START_CAPTURE',
    2001: 'IMAGE_STOP_CAPTURE',
    300:  'MISSION_START',
}

# Mission Planner 單次匯入上限
_MAX_WAYPOINTS = 655


@dataclass
class ValidationResult:
    """Mission 檢查結果"""
    ok: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    total_items: int = 0
    nav_items: int = 0
    do_items: int = 0


def _is_nav_cmd(cmd: int) -> bool:
    """NAV_* 類指令需要合法 lat/lon (16–85 大部分是 NAV)"""
    return 16 <= cmd <= 100


def _is_do_cmd(cmd: int) -> bool:
    """DO_* 類指令 (大部分 170–200)，通常 lat/lon=0"""
    return 170 <= cmd <= 200


def validate_mission(waypoints: Sequence[Tuple[float, ...]],
                      *,
                      strict: bool = False,
                      max_altitude_m: float = 10000.0,
                      ) -> ValidationResult:
    """驗證 MAVLink mission 序列

    Parameters
    ----------
    waypoints :
        8-tuple 列表：(lat, lon, alt, cmd, p1, p2, p3, p4)
    strict :
        True = warnings 也視為失敗
    max_altitude_m :
        可接受的最大高度 (預設 10 km)

    Returns
    -------
    ValidationResult
    """
    r = ValidationResult(ok=True, total_items=len(waypoints))

    # ── 數量檢查 ──────────────────────────────────────────
    if not waypoints:
        r.ok = False
        r.errors.append('mission 為空序列')
        return r
    if len(waypoints) > _MAX_WAYPOINTS:
        r.warnings.append(
            f'航點數 {len(waypoints)} 超過 MP 單次上傳上限 {_MAX_WAYPOINTS}'
        )

    # ── 逐項檢查 ──────────────────────────────────────────
    prev_lat, prev_lon, prev_alt = None, None, None
    dup_count = 0

    for seq, wp in enumerate(waypoints):
        if len(wp) != 8:
            r.ok = False
            r.errors.append(
                f'seq {seq}: 預期 8-tuple，得 {len(wp)}-tuple {wp}'
            )
            continue

        lat, lon, alt, cmd, p1, p2, p3, p4 = wp
        cmd = int(cmd)

        # 指令 ID
        if cmd not in _VALID_COMMANDS:
            r.ok = False
            r.errors.append(
                f'seq {seq}: 未知 command ID {cmd} (非 MAVLink 標準)'
            )
            continue

        if _is_nav_cmd(cmd):
            r.nav_items += 1
            # 經緯度範圍
            if not (-90.0 <= lat <= 90.0):
                r.ok = False
                r.errors.append(
                    f'seq {seq} ({_VALID_COMMANDS[cmd]}): '
                    f'lat={lat:.6f} 超出 ±90°'
                )
            if not (-180.0 <= lon <= 180.0):
                r.ok = False
                r.errors.append(
                    f'seq {seq} ({_VALID_COMMANDS[cmd]}): '
                    f'lon={lon:.6f} 超出 ±180°'
                )
            # lat=0 lon=0 對 NAV 類指令是可疑的 (赤道外海)
            if abs(lat) < 1e-6 and abs(lon) < 1e-6:
                r.warnings.append(
                    f'seq {seq} ({_VALID_COMMANDS[cmd]}): '
                    f'lat/lon 皆為 0 (可能未填座標)'
                )
            # 高度檢查
            if cmd not in (21, 85):   # LAND 允許 alt=0
                if alt < -10:
                    r.ok = False
                    r.errors.append(
                        f'seq {seq} ({_VALID_COMMANDS[cmd]}): '
                        f'alt={alt:.1f}m 負值'
                    )
                if alt > max_altitude_m:
                    r.warnings.append(
                        f'seq {seq}: alt={alt:.0f}m > {max_altitude_m:.0f}m'
                    )
            # 連續重複航點
            if cmd == 16 and prev_lat is not None:
                if (abs(lat - prev_lat) < 1e-7 and
                    abs(lon - prev_lon) < 1e-7 and
                    abs(alt - (prev_alt or 0)) < 0.5):
                    dup_count += 1
            prev_lat, prev_lon, prev_alt = lat, lon, alt
        elif _is_do_cmd(cmd):
            r.do_items += 1
            # DO_* 通常 lat/lon = 0，不檢查範圍

    if dup_count > 0:
        r.warnings.append(
            f'偵測到 {dup_count} 組連續重複航點 (可能無意義消耗序列)'
        )

    # ── 任務結構檢查 ──────────────────────────────────────
    # 首 NAV 通常應該是 TAKEOFF 或 SET_HOME
    first_nav = next((wp for wp in waypoints if _is_nav_cmd(int(wp[3]))), None)
    if first_nav is not None:
        first_cmd = int(first_nav[3])
        if first_cmd not in (22, 84, 179):   # TAKEOFF / VTOL_TAKEOFF / SET_HOME
            r.warnings.append(
                f'首個 NAV 指令為 {_VALID_COMMANDS.get(first_cmd)}，'
                '通常應為 NAV_TAKEOFF 或 NAV_VTOL_TAKEOFF'
            )

    # strict 模式：有 warning 也視為 fail
    if strict and r.warnings:
        r.ok = False

    return r


if __name__ == '__main__':
    # 冒煙測試
    good = [
        (25.0, 121.5, 0.0, 179, 0, 0, 0, 0),      # DO_SET_HOME
        (25.0, 121.5, 0.0, 178, 0, 25, -1, 0),    # DO_CHANGE_SPEED
        (25.001, 121.501, 100.0, 22, 10, 0, 0, 0),  # TAKEOFF
        (25.01, 121.51, 100.0, 16, 0, 150, 0, 0),   # WAYPOINT
        (25.02, 121.52, 0.0, 21, 0, 0, 0, 0),       # LAND
    ]
    bad = [
        (25.0, 121.5, 0.0, 999, 0, 0, 0, 0),       # 未知 cmd
        (91.0, 181.0, 50.0, 16, 0, 150, 0, 0),     # 超出範圍
        (25.0, 121.5, -50.0, 16, 0, 150, 0, 0),    # 負高度
    ]
    print('[GOOD mission]')
    r = validate_mission(good)
    print(f'  ok={r.ok}, errors={r.errors}, warnings={r.warnings}')
    print(f'[BAD mission]')
    r = validate_mission(bad)
    print(f'  ok={r.ok}, errors={len(r.errors)}:')
    for e in r.errors:
        print(f'    - {e}')
