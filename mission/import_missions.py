# -*- coding: utf-8 -*-
"""多檔航點匯入：每個 .waypoints／.plan **明確指定給一架**（sysid），再交給地圖預覽與寫入。

純 Python、不依賴 Qt。對話框（`ui/dialogs/import_waypoints_dialog.py`）與主視窗都只
透過這裡的四個函式工作，所以「誰拿到哪一份任務」的規則只有一份、而且測得到。

═══ 為什麼 sysid 不能用猜的 ═══
這個專案發生過「兩架飛機拿到同一份任務」（見 `ui/main_window.py::on_sitl_write_wp`
的註解）。所以：

* `load_many()` 回來的 `ImportedMission.sysid` **一律是 None** —— 載入不等於指派。
* `suggest_sysid()` 只從檔名的**明確模式**猜一個**預設值**（`UAV-3`、`uav3`、
  `_sys3`、`sysid3`、`mission_3.waypoints` 這種結尾數字）；猜不到、或檔名裡有兩個
  互相矛盾的數字就回 None。這個值放在 `ImportedMission.suggested_sysid`，UI 拿它當
  下拉的預設，**使用者按下確定才算指派**。
* `assignment_problems()` 列出所有擋住「確定」的理由：未指派、重複 sysid、不在
  1–255、檔案有錯誤、一個檔都沒有。UI 只要這個清單非空就不准按。

═══ 失敗策略 ═══
`load_many()` **不丟例外**：讀不到／認不出／語法錯的檔案會變成一份
`fmt='unreadable'`、items 空、issues 裡有一個 `load_failed` 錯誤的 LoadedMission，
留在清單裡讓使用者看到是哪個檔、為什麼；它同時也是 `assignment_problems()` 的一條
理由，所以不可能被靜靜略過或寫進飛控。
"""
from __future__ import annotations

import os
import pathlib
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

from mission import mission_view as _mv
from mission.waypoint_file import (
    CMD_NAV_LAND, CMD_NAV_TAKEOFF, CMD_NAV_VTOL_LAND, CMD_NAV_VTOL_TAKEOFF,
    LoadedMission, MissionFileError, MissionIssue, load_mission_file,
)

__all__ = [
    'SYSID_MIN', 'SYSID_MAX', 'ImportedMission',
    'load_many', 'suggest_sysid', 'assignment_problems', 'summaries', 'disambiguate_labels',
]

# MAVLink system id 是 uint8，0 是廣播、不是飛機。
SYSID_MIN = 1
SYSID_MAX = 255

_TAKEOFF_CMDS = frozenset({CMD_NAV_TAKEOFF, CMD_NAV_VTOL_TAKEOFF})
_LAND_CMDS = frozenset({CMD_NAV_LAND, CMD_NAV_VTOL_LAND})

# ── 檔名模式（只認這幾種；大小寫不拘）──────────────────────────────────
#   明確模式：UAV-3 / UAV_3 / uav3 / UAV 3、sys3 / _sys3 / sysid3 / sysid-3
#   弱模式：  以「分隔符＋數字」結尾（mission_3、area-B-12）；只有在沒有任何明確
#             模式時才採用。`flight01`（沒有分隔符）與 `mission_v2` 都不算。
#   數字前後都要是「非英數」邊界：`uav3x`、`nouav3` 都不算明確模式。
_PAT_EXPLICIT = (
    re.compile(r'(?<![a-z0-9])uav[-_ ]?(\d{1,3})(?![a-z0-9])', re.IGNORECASE),
    re.compile(r'(?<![a-z0-9])sys(?:id)?[-_ ]?(\d{1,3})(?![a-z0-9])', re.IGNORECASE),
)
_PAT_TRAILING = re.compile(r'[-_ ](\d{1,3})$')


@dataclass
class ImportedMission:
    """一個檔案 ＋ 它要給哪一架。

    sysid 是**指派結果**（None = 還沒指派）；suggested_sysid 只是檔名猜到的預設值，
    兩者分開放，才不會有人拿「猜的」當「指派的」送出去。
    """
    path: str
    sysid: Optional[int]
    loaded: LoadedMission
    label: str
    suggested_sysid: Optional[int] = None

    @property
    def has_errors(self) -> bool:
        return self.loaded.has_errors

    @property
    def items(self) -> list:
        return self.loaded.items


def _in_range(v) -> Optional[int]:
    try:
        s = int(v)
    except (TypeError, ValueError):
        return None
    return s if SYSID_MIN <= s <= SYSID_MAX else None


def suggest_sysid(path) -> Optional[int]:
    """只從檔名的明確模式猜一個**預設值**；猜不到或互相矛盾就 None。

    規則：
      1. 明確模式（UAV-n / sysn / sysidn）全部收集；恰好一個不同的值 ⇒ 回它，
         兩個以上不同的值（`uav1_uav2`）⇒ None。
      2. 沒有明確模式才看結尾的「分隔符＋數字」（`mission_3`）。
      3. 不在 1–255 的數字一律不算（`mission_2024` ⇒ None）。
    """
    stem = os.path.splitext(os.path.basename(os.fspath(path)))[0]
    explicit = set()
    for pat in _PAT_EXPLICIT:
        for m in pat.finditer(stem):
            v = _in_range(m.group(1))
            if v is not None:
                explicit.add(v)
    if len(explicit) == 1:
        return next(iter(explicit))
    if explicit:
        return None
    m = _PAT_TRAILING.search(stem)
    return _in_range(m.group(1)) if m else None


def load_many(paths: Iterable) -> List[ImportedMission]:
    """每個路徑 `load_mission_file`；讀不進來的檔**留在清單裡**（issues 帶 load_failed），不丟。

    回來的 `sysid` 一律 None（載入 ≠ 指派）；`suggested_sysid` 是檔名猜的預設值。
    """
    out: List[ImportedMission] = []
    for raw in paths:
        p = os.fspath(raw)
        try:
            lm = load_mission_file(p)
        except MissionFileError as e:
            lm = LoadedMission(p, 'unreadable', [],
                               [MissionIssue('error', None, 'load_failed', str(e))])
        except Exception as e:                         # noqa: BLE001 - 一個檔壞掉不能讓整批消失
            lm = LoadedMission(p, 'unreadable', [],
                               [MissionIssue('error', None, 'load_crashed',
                                             f'讀檔時發生未預期的錯誤：{type(e).__name__}: {e}')])
        out.append(ImportedMission(path=p, sysid=None, loaded=lm,
                                   label=os.path.basename(p) or p,
                                   suggested_sysid=suggest_sysid(p)))
    disambiguate_labels(out)
    return out


def _path_parts(path: str) -> tuple:
    try:
        return pathlib.PurePath(os.path.abspath(os.fspath(path))).parts
    except (TypeError, ValueError):
        return ()


def disambiguate_labels(missions: Iterable[ImportedMission]) -> List[ImportedMission]:
    """同名檔（不同資料夾）在表格／狀態列／確認框裡分不出來 ⇒ **就地**改 `label`。

    沒撞名的維持純檔名；撞名的改成 `<上層資料夾>/<檔名>`，還撞就再往上一層，直到分得
    開或已經用到完整路徑（兩個項目是同一個路徑時本來就分不開，維持原樣）。
    `load_many()` 回傳前會叫一次；對話框每次加入／移除檔案後也要再叫一次 —— 撞名的
    對象可能是**早就在清單裡**的那個，單看這一批新檔看不出來。回傳同一個 list 方便串接。
    """
    ms = missions if isinstance(missions, list) else list(missions)
    parts = [_path_parts(m.path) for m in ms]
    depth = [1] * len(ms)

    def _label(i: int) -> str:
        if not parts[i]:
            return os.path.basename(ms[i].path) or ms[i].path
        return '/'.join(parts[i][-depth[i]:])

    for _ in range(max((len(p) for p in parts), default=1)):
        groups: Dict[str, List[int]] = {}
        for i in range(len(ms)):
            groups.setdefault(_label(i), []).append(i)
        clash = [i for g in groups.values() if len(g) > 1 for i in g if depth[i] < len(parts[i])]
        if not clash:
            break
        for i in clash:
            depth[i] += 1
    for i, m in enumerate(ms):
        m.label = _label(i)
    return ms


def assignment_problems(missions: Iterable[ImportedMission]) -> List[str]:
    """擋住「確定」的理由清單；空清單 = 可以送。

    檢查：一個檔都沒有、未指派、sysid 不是 1–255 的整數、同一個 sysid 被多個檔指派、
    檔案本身有錯誤（含讀不進來）。訊息是繁體中文，直接給 UI 顯示。
    """
    missions = list(missions)
    probs: List[str] = []
    if not missions:
        return ['沒有任何檔案']
    by_sysid: Dict[int, List[str]] = {}
    for m in missions:
        name = m.label or os.path.basename(m.path) or m.path
        if m.sysid is None:
            probs.append(f'{name}：尚未指派 sysid')
        else:
            s = _in_range(m.sysid)
            if s is None or (isinstance(m.sysid, float) and m.sysid != int(m.sysid)):
                probs.append(f'{name}：sysid {m.sysid!r} 必須是 {SYSID_MIN}–{SYSID_MAX} 的整數')
            else:
                by_sysid.setdefault(s, []).append(name)
        if m.loaded.has_errors:
            errs = m.loaded.errors
            first = errs[0].message
            more = f'…另 {len(errs) - 1} 個' if len(errs) > 1 else ''
            probs.append(f'{name}：有 {len(errs)} 個錯誤（{first}{more}）')
    for s, names in sorted(by_sysid.items()):
        if len(names) > 1:
            probs.append(f'sysid {s} 被 {len(names)} 個檔同時指派：{"、".join(names)}')
    return probs


def summaries(missions: Iterable[ImportedMission]) -> List[dict]:
    """每檔一個 dict，給表格／狀態列用：

    `{'path','sysid','n_items','home','n_nav','has_takeoff','has_land',
      'issues_error','issues_warning','summary'}`

    n_items 是檔案裡的全部項目（含 seq 0 Home 槽）；n_nav 是 seq 0 以外的 NAV 指令
    數（MAV_CMD 16..95）；可繪製點數與盤旋圈由 `mission_view` 算，和地圖上畫的是同一套。
    """
    out: List[dict] = []
    for m in missions:
        items = list(m.loaded.items)
        body = [it for it in items if int(it.get('seq', -1)) != 0]
        cmds = [int(it.get('command', -1)) for it in body]
        n_nav = sum(1 for c in cmds if _mv.NAV_CMD_MIN <= c <= _mv.NAV_CMD_MAX)
        n_err = len(m.loaded.errors)
        n_warn = len(m.loaded.warnings)
        has_to = any(c in _TAKEOFF_CMDS for c in cmds)
        has_ld = any(c in _LAND_CMDS for c in cmds)
        if m.loaded.fmt == 'unreadable':
            text = '讀不進來：' + (m.loaded.errors[0].message if m.loaded.errors else '未知原因')
        else:
            drawable = len(_mv.mission_to_path_2d(items))
            rings = len(_mv.mission_loiters(items))
            parts = [f'{len(items)} 項', f'NAV {n_nav}', f'可繪製 {drawable} 點']
            if rings:
                parts.append(f'盤旋圈 {rings}')
            parts.append(f'起飛{"有" if has_to else "無"}')
            parts.append(f'降落{"有" if has_ld else "無"}')
            parts.append(f'錯誤 {n_err}、警告 {n_warn}')
            text = '、'.join(parts)
        out.append({
            'path': m.path,
            'sysid': m.sysid,
            'n_items': len(items),
            'home': _mv.mission_home(items),
            'n_nav': n_nav,
            'has_takeoff': has_to,
            'has_land': has_ld,
            'issues_error': n_err,
            'issues_warning': n_warn,
            'summary': text,
        })
    return out
