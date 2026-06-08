"""ui/resources/aeroplan_theme/widgets.py

Drop-in widgets that replace emoji-prefixed QPushButton across the codebase.

Before:
    btn = QPushButton('🚀 一鍵起飛')
    btn.setStyleSheet('background:#D84315;color:#fff;...')

After:
    from ui.resources.aeroplan_theme.widgets import IconButton
    btn = IconButton('rocket', '一鍵起飛', tone='orange')
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import QSize, Qt
from PyQt6.QtWidgets import QPushButton

from .icons import get_icon


# QPushButton ::on / ::off don't apply — we set objectName so QSS hits
# the right tone selector.
_TONES = {
    "primary":  "btnPrimary",
    "success":  "btnSuccess",
    "danger":   "btnDanger",
    "warn":     "btnWarn",
    "purple":   "btnPurple",
    "orange":   "btnOrange",
    "neutral":  "btnNeutral",
    "ghost":    "btnGhost",
}

_TONE_ACTION = {
    "primary":  "info",
    "success":  "go",
    "danger":   "stop",
    "warn":     "warn",
    "purple":   "purple",
    "orange":   "target",
    "neutral":  "neutral",
    "ghost":    "neutral",
}


class IconButton(QPushButton):
    """QPushButton with an SVG glyph + label, styled by `tone`.

    Parameters
    ----------
    icon_name    From ui.resources.aeroplan_theme.icons.ICON_NAMES
    label        Visible text (no emoji)
    tone         primary | success | danger | warn | purple | orange |
                 neutral | ghost
    compact      If True, height becomes 26 px (toolbar density).
    """

    def __init__(self, icon_name: str, label: str = "",
                 tone: str = "ghost", compact: bool = False,
                 parent: Optional[object] = None) -> None:
        super().__init__(label, parent)
        self.setObjectName(_TONES.get(tone, "btnGhost"))
        action = _TONE_ACTION.get(tone, "neutral")
        # Use white-ish glyph on filled tone buttons, accent color on ghosts.
        glyph_color = "#FFFFFF" if tone in ("primary", "success", "danger",
                                            "purple", "orange") else None
        if tone == "warn":
            glyph_color = "#1c1208"
        if glyph_color:
            self.setIcon(get_icon(icon_name, color=glyph_color, size=16))
        else:
            self.setIcon(get_icon(icon_name, action_class=action, size=16))
        self.setIconSize(QSize(14, 14))
        self.setMinimumHeight(26 if compact else 30)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setProperty("compact", "true" if compact else "false")


__all__ = ["IconButton"]
