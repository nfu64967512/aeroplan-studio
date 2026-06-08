"""ui/resources/aeroplan_theme/icons.py

Loads SVG icons from ./svg/ and tints them per action-class color.
All icons share a 24×24 viewBox, 1.6 stroke, currentColor — single-weight
tactical glyph set replacing all emoji previously used in widget labels.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

from PyQt6.QtCore import QByteArray, Qt
from PyQt6.QtGui import QIcon, QPainter, QPixmap
from PyQt6.QtSvg import QSvgRenderer


_ICON_DIR = Path(__file__).parent / "svg"


# Canonical action-class colors (kept in sync with tokens.py)
_ACTION_COLORS: Dict[str, str] = {
    "go":      "#00E676",
    "stop":    "#FF003C",
    "warn":    "#FFB703",
    "info":    "#1565C0",
    "arm":     "#FFD700",
    "neutral": "#A8B2BD",
    "mission": "#00B4D8",
    "target":  "#FF3D00",
    "purple":  "#9C27B0",
}


ICON_NAMES = (
    "launch", "stop", "link", "unlink",
    "arm", "disarm", "takeoff", "land",
    "upload", "download", "import", "export",
    "map_2d", "map_3d", "sat", "fly_to",
    "pin", "polygon", "circle", "clear",
    "confirm", "warn", "error", "info",
    "read", "write", "battery", "tool",
    "pause", "play", "rtb", "settings",
    "rocket", "drone", "fpv", "fov_cone",
    "radar", "heart",
)


def icon_path(name: str) -> Path:
    """Return absolute path to an icon's source SVG."""
    return _ICON_DIR / f"{name}.svg"


def get_icon(name: str, action_class: Optional[str] = None,
             color: Optional[str] = None, size: int = 16) -> QIcon:
    """Load an SVG icon, tint by action_class or explicit color, return QIcon.

    Parameters
    ----------
    name           One of ICON_NAMES.
    action_class   Optional semantic class — looked up in _ACTION_COLORS.
                   If both action_class and color are None, the SVG's
                   currentColor falls back to a neutral foreground.
    color          Override hex color string.
    size           Rendered pixel size (square).
    """
    path = icon_path(name)
    if not path.exists():
        return QIcon()

    src = path.read_text(encoding="utf-8")
    # Tint: every stroke/fill that uses 'currentColor' becomes the chosen hue.
    tint = color or _ACTION_COLORS.get(action_class or "", "#E0E1DD")
    src = src.replace("currentColor", tint)

    renderer = QSvgRenderer(QByteArray(src.encode("utf-8")))
    pm = QPixmap(size, size)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    renderer.render(p)
    p.end()
    return QIcon(pm)


__all__ = ["ICON_NAMES", "icon_path", "get_icon"]
