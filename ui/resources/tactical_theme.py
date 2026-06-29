"""
AeroPlan Studio — 戰術 UI 主題系統

依循 MIL-STD-1472H §5.17.25 + TABLE XL（Common color association meanings）
與 STANAG APP-6 戰術符號色彩語意。

單一事實來源（Single Source of Truth）：所有 QSS 與 Widget 共用此處定義的
色彩常數與字型，由 apply_tactical_theme() 在啟動時載入並注入至 QApplication。

色彩語意規範（依 TABLE XL — 嚴禁濫用）：

    HOSTILE   紅 (#FF003C) — Equipment: malfunction/critical/OFF/stop；
                              Tactical: hostile target identification
    WARNING   黃 (#FFB703) — Equipment: caution/check/abnormal/oil；
                              Tactical: unknown affiliation, CBRNE area
    FRIENDLY  綠 (#00E676) — Equipment: normal/in-tolerance/ready/ON/OPEN；
                              Tactical: NEUTRAL target affiliation（注意：戰術上
                              「friendly」其實對應 BLUE/CYAN — 命名僅延續舊版專案）
    NEUTRAL   青 (#00B4D8) — Equipment: advisory；
                              Tactical: friendly affiliation (cyan)
    AMBER/FG_EMPHASIS 琥珀 — 重點數字 (HUD boxed values, 經典 amber HUD 色)

    注意：本專案中所有 UAV 為己方無人機，因此 UAV_PALETTE 屬「discriminator
    color」(§5.17.25.6) — 用於區分多目標的辨識色板，並非 tactical affiliation 色。

字型規範（依 §5.17.18.7）：
    §5.17.18.7.1 標準字型 shall use Arial/Times/Courier/Verdana 等常見字型
    §5.17.18.7.2 不利條件下 shall use sans-serif (Arial/Verdana/Helvetica)
    本系統選用 Segoe UI / Inter / Roboto Condensed / Rajdhani / JetBrains Mono
    等於規範語意上同類，皆為合規字型。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

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

    # ─── 多機/多區域識別色板 (Discriminator Colors) ───────────────
    # MIL-STD-1472H 5.8.5 允許在「區分多個同類目標」時使用色相識別色板。
    # 必須通過 ISO/IEC 9241-3 色覺對比測試（避免紅綠色盲混淆）。
    # 此處挑選 8 色：明亮且高飽和，於 BG_PRIMARY (#0A0F14) 上對比度均 ≥ 4.5:1。
    UAV_PALETTE: Final[tuple[str, ...]] = (
        "#FFB703",  # 1. AMBER     ← 主機（與 FG_EMPHASIS 同）
        "#FF003C",  # 2. HOSTILE 紅
        "#00E676",  # 3. FRIENDLY 綠
        "#00B4D8",  # 4. NEUTRAL 青
        "#C77DFF",  # 5. 紫（電子戰）
        "#80FFDB",  # 6. 薄荷青
        "#FF8500",  # 7. 橘
        "#7CB9E8",  # 8. 淡藍
    )
    # 區域/任務區塊用色板（與 UAV 同階序但色相略偏移避免混淆）
    REGION_PALETTE: Final[tuple[str, ...]] = (
        "#08EC91",  # 1. 任務綠
        "#FFB703",  # 2. 琥珀
        "#00B4D8",  # 3. 中性青
        "#FF003C",  # 4. 危險紅
        "#C77DFF",  # 5. 紫
        "#FF8500",  # 6. 橘
    )

    @classmethod
    def uav_color(cls, sysid: int) -> str:
        """依 sysid (1-N) 回傳 UAV 識別色（循環取用）。"""
        if sysid < 1:
            sysid = 1
        return cls.UAV_PALETTE[(sysid - 1) % len(cls.UAV_PALETTE)]

    @classmethod
    def region_color(cls, idx: int) -> str:
        """依區域索引（0-based）回傳識別色（循環取用）。"""
        return cls.REGION_PALETTE[max(0, idx) % len(cls.REGION_PALETTE)]

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
# 2026-04 起，主 QSS 改為 Global_MIL_STD.qss（嚴格對應 MIL-STD-1472H 章節）。
# 若檔案不存在則回退至 tactical_theme.qss，以維持向後相容。
_STYLES_DIR = Path(__file__).parent / "styles"
_QSS_PRIMARY = _STYLES_DIR / "Global_MIL_STD.qss"
_QSS_LEGACY = _STYLES_DIR / "tactical_theme.qss"


def _resolve_qss_path() -> Path:
    """回傳實際使用的 QSS 檔案路徑（主檔優先，失敗則 legacy）。"""
    if _QSS_PRIMARY.exists():
        return _QSS_PRIMARY
    return _QSS_LEGACY


def _render_qss() -> str:
    """讀取 QSS 模板並以 TacticalColors/Fonts 變數做字串替換。"""
    template = _resolve_qss_path().read_text(encoding="utf-8")
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
