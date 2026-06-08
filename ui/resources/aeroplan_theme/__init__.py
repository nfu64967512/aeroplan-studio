"""ui/resources/aeroplan_theme/__init__.py

Public API for the AeroPlan Studio design-system theme.

Usage in main.py:

    from ui.resources.aeroplan_theme import apply_theme
    app = QApplication(sys.argv)
    apply_theme(app)

Replaces ui.resources.tactical_theme.apply_tactical_theme (still imported by
some widgets; this module keeps a backwards-compatible alias).
"""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QFont, QFontDatabase, QPalette
from PyQt6.QtWidgets import QApplication

from . import tokens as T
from .icons import get_icon, icon_path, ICON_NAMES   # noqa: F401  (re-exported)


# ── QSS template substitution ─────────────────────────────────────
_QSS_PATH = Path(__file__).parent / "aeroplan_theme.qss"


def _render_qss() -> str:
    """渲染 QSS 模板，將 {{TOKEN}} 替換為實際值。"""
    template = _QSS_PATH.read_text(encoding="utf-8")
    subs = {
        "{{BG_PRIMARY}}":    T.BG_PRIMARY,
        "{{BG_SECONDARY}}":  T.BG_SECONDARY,
        "{{BG_ELEVATED}}":   T.BG_ELEVATED,
        "{{BG_SUNKEN}}":     T.BG_SUNKEN,
        "{{BORDER_SUBTLE}}": T.BORDER_SUBTLE,
        "{{BORDER}}":        T.BORDER,
        "{{BORDER_STRONG}}": T.BORDER_STRONG,
        "{{FG}}":            T.FG,
        "{{FG_SECONDARY}}":  T.FG_SECONDARY,
        "{{FG_MUTED}}":      T.FG_MUTED,
        "{{FG_EMPHASIS}}":   T.FG_EMPHASIS,
        # ADOS accent（chrome 用）
        "{{ACCENT}}":        T.ACCENT_PRIMARY,
        "{{ACCENT_HOVER}}":  T.ACCENT_PRIMARY_HOVER,
        "{{ACCENT_2}}":      T.ACCENT_SECONDARY,
        "{{HUD_GREEN}}":     T.HUD_GREEN,
        # 1472H 安全色
        "{{HOSTILE}}":       T.HOSTILE,
        "{{HOSTILE_DEEP}}":  T.HOSTILE_DEEP,
        "{{WARNING}}":       T.WARNING,
        "{{WARNING_DEEP}}":  T.WARNING_DEEP,
        "{{FRIENDLY}}":      T.FRIENDLY,
        "{{FRIENDLY_DEEP}}": T.FRIENDLY_DEEP,
        "{{NEUTRAL}}":       T.NEUTRAL,
        "{{INFO}}":          T.INFO,
        "{{INFO_HOVER}}":    T.INFO_HOVER,
        "{{PURPLE}}":        T.PURPLE,
        "{{ORANGE}}":        T.ORANGE,
        # 字型
        "{{FONT_MONO}}":     T.FONT_MONO_STACK,
        "{{FONT_CONDENSED}}":T.FONT_CONDENSED_STACK,
        "{{FONT_SANS}}":     T.FONT_SANS_STACK,
        "{{FONT_DISPLAY}}":  T.FONT_DISPLAY_STACK,
        # 字體尺寸
        "{{FS_BODY}}":       str(T.FS_BODY),
        "{{FS_CAUTION}}":    str(T.FS_CAUTION),
        "{{FS_WARN}}":       str(T.FS_WARN),
        "{{FS_MIN}}":        str(T.FS_MIN),
        "{{FS_LEGACY}}":     str(T.FS_LEGACY),
    }
    for k, v in subs.items():
        template = template.replace(k, v)
    return template


def apply_theme(app: QApplication) -> None:
    """Apply the AeroPlan Studio design system to a QApplication.

    Call once after QApplication() is constructed and before showing any
    widget. Sets palette, global stylesheet, and default font.
    """
    # 1. Palette — covers system-rendered surfaces (tooltips, dialogs,
    #    QMessageBox, scrollbar handle on some platforms).
    pal = QPalette()
    pal.setColor(QPalette.ColorRole.Window,           QColor(T.BG_PRIMARY))
    pal.setColor(QPalette.ColorRole.WindowText,       QColor(T.FG))
    pal.setColor(QPalette.ColorRole.Base,             QColor(T.BG_SUNKEN))
    pal.setColor(QPalette.ColorRole.AlternateBase,    QColor(T.BG_SECONDARY))
    pal.setColor(QPalette.ColorRole.Text,             QColor(T.FG))
    pal.setColor(QPalette.ColorRole.Button,           QColor(T.BG_SECONDARY))
    pal.setColor(QPalette.ColorRole.ButtonText,       QColor(T.FG))
    pal.setColor(QPalette.ColorRole.Highlight,        QColor(T.NEUTRAL))
    pal.setColor(QPalette.ColorRole.HighlightedText,  QColor(T.BG_PRIMARY))
    pal.setColor(QPalette.ColorRole.ToolTipBase,      QColor(T.BG_ELEVATED))
    pal.setColor(QPalette.ColorRole.ToolTipText,      QColor(T.FG_EMPHASIS))
    pal.setColor(QPalette.ColorRole.PlaceholderText,  QColor(T.FG_MUTED))
    app.setPalette(pal)

    # 2. Stylesheet
    app.setStyleSheet(_render_qss())

    # 3. Default app font
    f = QFont()
    # First-available font from sans stack
    for name in ("Inter", "Segoe UI", "PingFang TC", "Microsoft YaHei", "Arial"):
        if name in QFontDatabase.families():
            f.setFamily(name)
            break
    f.setPointSize(9)
    app.setFont(f)

    # 4. ADOS display 字型缺席提示（不阻擋啟動）
    if "Space Grotesk" not in QFontDatabase.families():
        import logging
        logging.getLogger(__name__).info(
            "Tip: install Space Grotesk from Google Fonts "
            "for closest match to ADOS visual."
        )


# ── Backwards-compatible alias ────────────────────────────────────
# ui/widgets/tactical_uav_card.py and others import the old name.
def apply_tactical_theme(app: QApplication) -> None:
    """Deprecated alias — calls apply_theme()."""
    apply_theme(app)


__all__ = ["apply_theme", "apply_tactical_theme", "tokens", "get_icon",
           "icon_path", "ICON_NAMES", "T"]
