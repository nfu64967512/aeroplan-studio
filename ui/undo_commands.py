"""
AeroPlan Studio — Undo/Redo 命令集
====================================

以 QUndoCommand 實作可撤銷操作：
    * 標記邊界角點    AddCornerCommand
    * 標記打擊目標    AddStrikeTargetCommand
    * 設定 STOT 基地  SetStrikeLaunchBaseCommand
    * 清除所有角點    ClearCornersCommand

MainWindow 提供一個共用的 QUndoStack，menu/toolbar 可連接標準 Undo/Redo action。

使用範例::

    from PyQt6.QtGui import QUndoStack
    from ui.undo_commands import AddCornerCommand

    self.undo_stack = QUndoStack(self)
    cmd = AddCornerCommand(self, lat, lon)
    self.undo_stack.push(cmd)    # push 會自動呼叫 redo() 執行操作
    # 之後 Ctrl+Z 會觸發 undo()

參考：https://doc.qt.io/qt-6/qundocommand.html
"""
from __future__ import annotations

from typing import Any, Optional, Tuple

from PyQt6.QtGui import QUndoCommand


# ═══════════════════════════════════════════════════════════════════════
#  邊界角點
# ═══════════════════════════════════════════════════════════════════════

class AddCornerCommand(QUndoCommand):
    """新增一個邊界角點 — Ctrl+Z 會移除"""

    def __init__(self, main_window: Any, lat: float, lon: float):
        super().__init__(f'新增角點 ({lat:.4f}, {lon:.4f})')
        self.win = main_window
        self.lat = lat
        self.lon = lon
        self._added_index: Optional[int] = None

    def redo(self) -> None:
        self._added_index = len(self.win.corners)
        # 直接更新 state，不用 map.add_corner() 避免 signal 迴圈
        self.win.corners.append((self.lat, self.lon))
        if hasattr(self.win.map_widget, 'add_corner'):
            self.win.map_widget.add_corner(self.lat, self.lon)
        if hasattr(self.win, 'parameter_panel'):
            self.win.parameter_panel.update_corner_count(len(self.win.corners))

    def undo(self) -> None:
        if self._added_index is None:
            return
        if 0 <= self._added_index < len(self.win.corners):
            self.win.corners.pop(self._added_index)
        # 同步地圖（簡單做法：清空重繪）
        if hasattr(self.win.map_widget, 'clear_corners'):
            self.win.map_widget.clear_corners()
            for lat, lon in self.win.corners:
                self.win.map_widget.add_corner(lat, lon)
        if hasattr(self.win, 'parameter_panel'):
            self.win.parameter_panel.update_corner_count(len(self.win.corners))


class ClearCornersCommand(QUndoCommand):
    """清除所有邊界角點 — undo 會還原"""

    def __init__(self, main_window: Any):
        super().__init__('清除所有邊界角點')
        self.win = main_window
        self._snapshot: list = []

    def redo(self) -> None:
        self._snapshot = list(self.win.corners)
        self.win.corners.clear()
        if hasattr(self.win.map_widget, 'clear_corners'):
            self.win.map_widget.clear_corners()
        if hasattr(self.win, 'parameter_panel'):
            self.win.parameter_panel.update_corner_count(0)

    def undo(self) -> None:
        for lat, lon in self._snapshot:
            self.win.corners.append((lat, lon))
            if hasattr(self.win.map_widget, 'add_corner'):
                self.win.map_widget.add_corner(lat, lon)
        if hasattr(self.win, 'parameter_panel'):
            self.win.parameter_panel.update_corner_count(len(self.win.corners))


# ═══════════════════════════════════════════════════════════════════════
#  蜂群打擊目標
# ═══════════════════════════════════════════════════════════════════════

class AddStrikeTargetCommand(QUndoCommand):
    """新增一個打擊目標 — Ctrl+Z 會移除"""

    def __init__(self, main_window: Any, lat: float, lon: float):
        super().__init__(f'新增打擊目標 ({lat:.4f}, {lon:.4f})')
        self.win = main_window
        self.lat = lat
        self.lon = lon
        self._added_index: Optional[int] = None

    def redo(self) -> None:
        if not hasattr(self.win, '_strike_targets'):
            return
        self._added_index = len(self.win._strike_targets)
        self.win._strike_targets.append((self.lat, self.lon))
        cesium = self.win._get_cesium_widget() if hasattr(self.win, '_get_cesium_widget') else None
        if cesium:
            cesium.strike_add_target(self.lat, self.lon, self._added_index + 1)
        if hasattr(self.win, 'parameter_panel'):
            self.win.parameter_panel.update_strike_target_count(
                len(self.win._strike_targets)
            )

    def undo(self) -> None:
        if self._added_index is None:
            return
        if 0 <= self._added_index < len(self.win._strike_targets):
            self.win._strike_targets.pop(self._added_index)
        # 清空 3D 地圖上的目標並重繪（簡單做法）
        cesium = self.win._get_cesium_widget() if hasattr(self.win, '_get_cesium_widget') else None
        if cesium and hasattr(cesium, 'strike_clear_all'):
            cesium.strike_clear_all()
            for i, (lat, lon) in enumerate(self.win._strike_targets, 1):
                cesium.strike_add_target(lat, lon, i)
        if hasattr(self.win, 'parameter_panel'):
            self.win.parameter_panel.update_strike_target_count(
                len(self.win._strike_targets)
            )


class SetStrikeLaunchBaseCommand(QUndoCommand):
    """設定 STOT 共用發射基地 — undo 回復前一個值"""

    def __init__(self, main_window: Any, new_base: Optional[Tuple[float, float]]):
        super().__init__(
            f'設定 STOT 基地 ({new_base[0]:.4f}, {new_base[1]:.4f})'
            if new_base else '清除 STOT 基地'
        )
        self.win = main_window
        self._new = new_base
        self._old: Optional[Tuple[float, float]] = None

    def redo(self) -> None:
        self._old = getattr(self.win, '_strike_launch_base', None)
        self.win._strike_launch_base = self._new
        if hasattr(self.win, 'parameter_panel'):
            if self._new:
                self.win.parameter_panel.update_strike_base_label(*self._new)
            else:
                self.win.parameter_panel.update_strike_base_label(None, None)

    def undo(self) -> None:
        self.win._strike_launch_base = self._old
        if hasattr(self.win, 'parameter_panel'):
            if self._old:
                self.win.parameter_panel.update_strike_base_label(*self._old)
            else:
                self.win.parameter_panel.update_strike_base_label(None, None)


# ═══════════════════════════════════════════════════════════════════════
#  Undo Stack 輔助
# ═══════════════════════════════════════════════════════════════════════

def setup_undo_stack(main_window: Any) -> None:
    """在 MainWindow 建立共用的 QUndoStack 並掛選單 action

    呼叫端在 create_menus() 之後呼叫此函式即可。
    """
    from PyQt6.QtGui import QUndoStack, QKeySequence, QAction

    stack = QUndoStack(main_window)
    stack.setUndoLimit(100)   # 最多保留 100 步
    main_window.undo_stack = stack

    # 找編輯選單並加入 Undo/Redo
    menubar = main_window.menuBar()
    edit_menu = None
    for act in menubar.actions():
        if act.text().startswith('編輯'):
            edit_menu = act.menu()
            break

    if edit_menu is None:
        edit_menu = menubar.addMenu('編輯(&E)')

    undo_action = stack.createUndoAction(main_window, '復原')
    undo_action.setShortcut(QKeySequence.StandardKey.Undo)   # Ctrl+Z
    edit_menu.insertAction(edit_menu.actions()[0] if edit_menu.actions() else None,
                            undo_action)

    redo_action = stack.createRedoAction(main_window, '重做')
    redo_action.setShortcut(QKeySequence.StandardKey.Redo)   # Ctrl+Y
    edit_menu.insertAction(edit_menu.actions()[1] if len(edit_menu.actions()) > 1 else None,
                            redo_action)

    edit_menu.insertSeparator(edit_menu.actions()[2] if len(edit_menu.actions()) > 2 else None)
