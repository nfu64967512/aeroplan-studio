"""
AeroPlan Studio — 戰術 UI 主題系統

依循 MIL-STD-1472H 人機工程標準 + STANAG APP-6 戰術符號色彩語意。
單一事實來源（Single Source of Truth）：所有 QSS 與 Widget 共用此處定義的色彩常數與字型，
由 apply_tactical_theme() 在啟動時載入並注入至 QApplication。

色彩語意規範（嚴禁濫用）：
    HOSTILE   紅  — 敵方／失效／嚴重告警
    WARNING   黃  — 警告／未知／數值接近閾值
    FRIENDLY  綠  — 友軍／正常／就緒
    NEUTRAL   青  — 中立單位／一般資訊
    AMBER     琥珀 — 重點數字（非告警）
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QFont, QFontDatabase, QPalette
from PyQt6.QtWidgets import QApplication


# ══════════════════════════════════════════════════════════════════════
#  TacticalColors — 戰術色彩系統（暗視覺適應）
# ══════════════════════════════════════════════════════════════════════
class TacticalColors:
    """深色 HUD 色板。所有 hex 以 HTML 標準 '#RRGGBB' 表示。"""

    # ─── 中性底色（護眼暗視覺） ─────────────────────────────
    BG_PRIMARY: Final[str]   = "#0A0F14"  # 主視窗背景（深藏青，非純黑，保視桿細胞）
    BG_SECONDARY: Final[str] = "#0D1B2A"  # 面板／卡片背景
    BG_ELEVATED: Final[str]  = "#132236"  # 浮起層（Dialog / Popup）
    BG_SUNKEN: Final[str]    = "#060A0F"  # 凹陷層（輸入框）

    # ─── 邊線／分隔 ─────────────────────────────
    BORDER_SUBTLE: Final[str] = "#1F2D3D"  # 極低對比分隔線
    BORDER_DEFAULT: Final[str] = "#2A3D54"  # 一般邊框
    BORDER_STRONG: Final[str]  = "#415A77"  # 強調邊框／Hover

    # ─── 前景文字 ─────────────────────────────
    FG_PRIMARY: Final[str]   = "#E0E1DD"  # 主文字（非純白，降眩光）
    FG_SECONDARY: Final[str] = "#A8B2BD"  # 次要說明
    FG_MUTED: Final[str]     = "#6B7A8C"  # 停用／禁用
    FG_EMPHASIS: Final[str]  = "#FFB703"  # 琥珀重點數字（經典 HUD 色，570-590nm 對暗視覺最友善）

    # ─── 語意色（嚴守 MIL-STD-1472H） ─────────────────────────────
    HOSTILE: Final[str]   = "#FF003C"  # 紅 — 敵方／危險／失效
    WARNING: Final[str]   = "#FFB703"  # 黃 — 警告／未知
    FRIENDLY: Final[str]  = "#00E676"  # 綠 — 友軍／正常
    NEUTRAL: Final[str]   = "#00B4D8"  # 青 — 中立／資訊
    UNKNOWN: Final[str]   = "#FFD166"  # 淡黃 — 未識別

    # ─── Overlay（半透明 HUD 面板） ─────────────────────────────
    OVERLAY_BG: Final[str] = "rgba(13, 27, 42, 0.85)"   # 85% 不透明
    OVERLAY_BG_LIGHT: Final[str] = "rgba(13, 27, 42, 0.65)"

    @staticmethod
    def q(hex_or_rgba: str, alpha: int | None = None) -> QColor:
        """將 hex 字串轉為 QColor，可選覆寫 alpha (0-255)。"""
        c = QColor(hex_or_rgba)
        if alpha is not None:
            c.setAlpha(alpha)
        return c


# ══════════════════════════════════════════════════════════════════════
#  TacticalFonts — 字型堆疊（Fallback Chain）
# ══════════════════════════════════════════════════════════════════════
class TacticalFonts:
    """字型系統。以 fallback chain 確保跨平台優雅降級。"""

    # 等寬字型 — 用於數字、座標、遙測
    MONO_STACK: Final[tuple[str, ...]] = (
        "JetBrains Mono",
        "Cascadia Mono",
        "Consolas",
        "Courier New",
        "monospace",
    )

    # 窄體無襯線 — 用於狀態標籤、標題
    CONDENSED_STACK: Final[tuple[str, ...]] = (
        "Rajdhani",
        "Barlow Condensed",
        "Roboto Condensed",
        "Segoe UI Semibold",
        "Arial Narrow",
        "sans-serif",
    )

    # 一般 UI 文字
    SANS_STACK: Final[tuple[str, ...]] = (
        "Segoe UI",
        "Inter",
        "Microsoft YaHei",
        "微軟正黑體",
        "Arial",
        "sans-serif",
    )

    @classmethod
    def mono(cls, size: int = 10, bold: bool = False) -> QFont:
        """回傳等寬字型物件。"""
        f = cls._pick(cls.MONO_STACK, size)
        f.setBold(bold)
        f.setStyleHint(QFont.StyleHint.Monospace)
        return f

    @classmethod
    def condensed(cls, size: int = 10, bold: bool = True,
                  letter_spacing: float = 1.0) -> QFont:
        """回傳窄體字型物件，預設加 tracking。"""
        f = cls._pick(cls.CONDENSED_STACK, size)
        f.setBold(bold)
        f.setCapitalization(QFont.Capitalization.AllUppercase)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, letter_spacing)
        return f

    @classmethod
    def sans(cls, size: int = 9) -> QFont:
        """回傳一般無襯線字型。"""
        return cls._pick(cls.SANS_STACK, size)

    @staticmethod
    def _pick(stack: tuple[str, ...], size: int) -> QFont:
        """在 stack 中挑選第一個系統實際存在的字型。"""
        families = QFontDatabase.families()
        for name in stack:
            if name in families:
                return QFont(name, size)
        # fallback：讓 Qt 自行選擇 generic family
        return QFont(stack[-1], size)

    @classmethod
    def css_mono(cls) -> str:
        """QSS 用字型串列（CSV）。"""
        return ", ".join(f'"{n}"' for n in cls.MONO_STACK)

    @classmethod
    def css_condensed(cls) -> str:
        return ", ".join(f'"{n}"' for n in cls.CONDENSED_STACK)

    @classmethod
    def css_sans(cls) -> str:
        return ", ".join(f'"{n}"' for n in cls.SANS_STACK)


# ══════════════════════════════════════════════════════════════════════
#  Thresholds — 戰術閾值（跨元件共用）
# ══════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class TacticalThresholds:
    """UAV 健康/戰術閾值。越過此值即觸發告警顏色或閃爍。"""

    battery_critical: float = 20.0  # % — 紅色閃爍（強制 RTB）
    battery_low: float      = 40.0  # % — 黃色告警
    link_critical: float    = 30.0  # % — 紅色（可能失聯）
    link_low: float         = 60.0  # % — 黃色

    # 閃爍頻率（Hz → ms）：2 Hz 方波 = 250ms on / 250ms off
    # 符合 MIL-STD-411E 告警燈建議頻率
    blink_interval_ms: int  = 250


THRESHOLDS = TacticalThresholds()


# ══════════════════════════════════════════════════════════════════════
#  apply_tactical_theme — 啟動時注入主題
# ══════════════════════════════════════════════════════════════════════
_QSS_PATH = Path(__file__).parent / "styles" / "tactical_theme.qss"


def _render_qss() -> str:
    """讀取 QSS 模板並以 TacticalColors/Fonts 變數做字串替換。"""
    template = _QSS_PATH.read_text(encoding="utf-8")
    replacements: dict[str, str] = {
        # 色彩
        "{{BG_PRIMARY}}": TacticalColors.BG_PRIMARY,
        "{{BG_SECONDARY}}": TacticalColors.BG_SECONDARY,
        "{{BG_ELEVATED}}": TacticalColors.BG_ELEVATED,
        "{{BG_SUNKEN}}": TacticalColors.BG_SUNKEN,
        "{{BORDER_SUBTLE}}": TacticalColors.BORDER_SUBTLE,
        "{{BORDER_DEFAULT}}": TacticalColors.BORDER_DEFAULT,
        "{{BORDER_STRONG}}": TacticalColors.BORDER_STRONG,
        "{{FG_PRIMARY}}": TacticalColors.FG_PRIMARY,
        "{{FG_SECONDARY}}": TacticalColors.FG_SECONDARY,
        "{{FG_MUTED}}": TacticalColors.FG_MUTED,
        "{{FG_EMPHASIS}}": TacticalColors.FG_EMPHASIS,
        "{{HOSTILE}}": TacticalColors.HOSTILE,
        "{{WARNING}}": TacticalColors.WARNING,
        "{{FRIENDLY}}": TacticalColors.FRIENDLY,
        "{{NEUTRAL}}": TacticalColors.NEUTRAL,
        # 字型
        "{{FONT_MONO}}": TacticalFonts.css_mono(),
        "{{FONT_CONDENSED}}": TacticalFonts.css_condensed(),
        "{{FONT_SANS}}": TacticalFonts.css_sans(),
    }
    for k, v in replacements.items():
        template = template.replace(k, v)
    return template


def apply_tactical_theme(app: QApplication) -> None:
    """將戰術主題套用至 QApplication。

    使用方式：
        from ui.resources.tactical_theme import apply_tactical_theme
        app = QApplication(sys.argv)
        apply_tactical_theme(app)
    """
    # 1. 設 Qt Palette（系統對話框、Tooltip 等非 QSS 控制的元件）
    pal = QPalette()
    pal.setColor(QPalette.ColorRole.Window, TacticalColors.q(TacticalColors.BG_PRIMARY))
    pal.setColor(QPalette.ColorRole.WindowText, TacticalColors.q(TacticalColors.FG_PRIMARY))
    pal.setColor(QPalette.ColorRole.Base, TacticalColors.q(TacticalColors.BG_SUNKEN))
    pal.setColor(QPalette.ColorRole.AlternateBase, TacticalColors.q(TacticalColors.BG_SECONDARY))
    pal.setColor(QPalette.ColorRole.Text, TacticalColors.q(TacticalColors.FG_PRIMARY))
    pal.setColor(QPalette.ColorRole.Button, TacticalColors.q(TacticalColors.BG_SECONDARY))
    pal.setColor(QPalette.ColorRole.ButtonText, TacticalColors.q(TacticalColors.FG_PRIMARY))
    pal.setColor(QPalette.ColorRole.Highlight, TacticalColors.q(TacticalColors.NEUTRAL))
    pal.setColor(QPalette.ColorRole.HighlightedText, TacticalColors.q(TacticalColors.BG_PRIMARY))
    pal.setColor(QPalette.ColorRole.ToolTipBase, TacticalColors.q(TacticalColors.BG_ELEVATED))
    pal.setColor(QPalette.ColorRole.ToolTipText, TacticalColors.q(TacticalColors.FG_EMPHASIS))
    app.setPalette(pal)

    # 2. 套用全域 QSS
    app.setStyleSheet(_render_qss())

    # 3. 全域預設字型
    app.setFont(TacticalFonts.sans(9))


__all__ = [
    "TacticalColors",
    "TacticalFonts",
    "TacticalThresholds",
    "THRESHOLDS",
    "apply_tactical_theme",
]
