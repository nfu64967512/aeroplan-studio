"""ui/resources/aeroplan_theme/tokens.py — 設計系統 Token 單一來源。

由 main.py 透過 apply_theme(app) 載入。所有值經審核符合
MIL-STD-1472H §5.17.25（色）、§5.17.27（閃爍）、§5.7.3（警報），
§5.4.7 / Tables XVIII-XIX（0.5–1.0 m 操作距離字高）。

本檔同時對齊 ADOS Mission Control 的設計 token（取自其
`src/app/globals.css :root` 變數），但僅限「chrome」surface：
背景、邊框、accent 按鈕、卡片 hover。

**1472H 語意安全色（HOSTILE/WARNING/FRIENDLY/NEUTRAL/INFO）刻意維持原值**，
不被 ADOS 的較低飽和度替代，以保留遠距判讀色度。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Final


# ── Surfaces（採 ADOS 純黑/灰 chrome）─────────────────────────────
# 來源：ADOS globals.css :root --alt-bg-primary..tertiary
BG_PRIMARY:    Final[str] = "#000000"   # ADOS bg-primary（原 #0A0F14）
BG_SECONDARY:  Final[str] = "#0a0a0a"   # ADOS bg-secondary（原 #0D1B2A）
BG_ELEVATED:   Final[str] = "#141414"   # ADOS bg-tertiary（原 #132236）
BG_SUNKEN:     Final[str] = "#050505"   # 較 BG_PRIMARY 更暗（原 #060A0F）

BORDER_SUBTLE: Final[str] = "#1a1a1a"   # ADOS border-default（原 #1F2D3D）
BORDER:        Final[str] = "#2a2a2a"   # ADOS border-strong（原 #2A3D54）
BORDER_STRONG: Final[str] = "#3a3a3a"   # 略亮，hover 用（原 #415A77）

# ── Foreground（採 ADOS 純白/灰階）─────────────────────────────────
FG:            Final[str] = "#fafafa"   # ADOS text-primary（原 #E0E1DD）
FG_SECONDARY:  Final[str] = "#a0a0a0"   # ADOS text-secondary（原 #A8B2BD）
FG_MUTED:      Final[str] = "#666666"   # ADOS text-tertiary（原 #6B7A8C）
FG_EMPHASIS:   Final[str] = "#FFB703"   # 1472H 琥珀讀數（不動）

# ── ADOS Accent（chrome 專用，不可挪作安全色）────────────────────
ACCENT_PRIMARY:        Final[str] = "#3a82ff"   # ADOS accent-primary（主動作按鈕）
ACCENT_PRIMARY_HOVER:  Final[str] = "#5b9aff"   # ADOS accent-primary-hover
ACCENT_SECONDARY:      Final[str] = "#dff140"   # ADOS accent-secondary（lime / hero CTA）
HUD_GREEN:             Final[str] = "#00ff41"   # ADOS gcs-hud-green（HUD CRT 風讀數）

# ── Semantic（MIL-STD-1472H Table XL — 絕對不可被 ADOS 色票覆寫）──
HOSTILE:       Final[str] = "#FF003C"   # 紅 — 危急/失效/停止
HOSTILE_DEEP:  Final[str] = "#c62828"
WARNING:       Final[str] = "#FFB703"   # 琥珀 — 注意/異常
WARNING_DEEP:  Final[str] = "#FF8F00"
FRIENDLY:      Final[str] = "#00E676"   # 綠 — 正常/安全/通
FRIENDLY_DEEP: Final[str] = "#2e7d32"
NEUTRAL:       Final[str] = "#00B4D8"   # 青 — 諮詢/友軍
INFO:          Final[str] = "#1565C0"   # 藍 — 一般動作/非危急
INFO_HOVER:    Final[str] = "#1976D2"
PURPLE:        Final[str] = "#6A1B9A"   # 上傳/任務階
ORANGE:        Final[str] = "#D84315"   # one-click 既有 hero（保留向下相容）

# ── Type 字型堆疊 ─────────────────────────────────────────────────
FONT_MONO_STACK = (
    '"JetBrains Mono", "Cascadia Mono", "Consolas", "Courier New", monospace'
)
FONT_CONDENSED_STACK = (
    '"Rajdhani", "Barlow Condensed", "Roboto Condensed", '
    '"Segoe UI Semibold", "Arial Narrow", sans-serif'
)
FONT_SANS_STACK = (
    '"Inter", "Segoe UI", "Microsoft YaHei", "微軟正黑體", "Arial", sans-serif'
)
# ADOS display 字型（Space Grotesk）；Windows 預設無此字型，
# fallback Rajdhani → Segoe UI Semibold。apply_theme() log 會給安裝提示。
FONT_DISPLAY_STACK = (
    '"Space Grotesk", "Rajdhani", "Segoe UI Semibold", "Arial", sans-serif'
)

# Character-height ramp — 1472H Table XVIII at 0.5–1.0 m
FS_WARN:       Final[int] = 28       # §5.7.3.6 upper band
FS_CAUTION:    Final[int] = 22       # §5.7.3.6 lower band
FS_BODY:       Final[int] = 18       # Table XVIII floor
FS_MIN:        Final[int] = 15       # Table XIX critical fixed-position
FS_LEGACY:     Final[int] = 12       # tooltips / secondary 非任務

# ── Spacing / radius ──────────────────────────────────────────────
S1, S2, S3, S4, S5, S6 = 4, 6, 8, 10, 12, 16
R_XS, R_SM, R_MD, R_LG = 3, 4, 6, 8

# ── Alarm timing (1472H §5.7.3.5 / §5.17.27.3) ────────────────────
FLASH_WARNING_PERIOD_MS: Final[int] = 250    # 4 Hz, 50% duty
FLASH_CAUTION_PERIOD_MS: Final[int] = 600    # ~1.67 Hz, 70% on


@dataclass(frozen=True)
class Thresholds:
    """UAV 健康度門檻，供冗餘編碼使用。"""
    battery_critical: float = 20.0
    battery_low:      float = 40.0
    link_critical:    float = 30.0
    link_low:         float = 60.0
    blink_interval_ms: int = FLASH_CAUTION_PERIOD_MS


THRESHOLDS = Thresholds()


__all__ = [
    "BG_PRIMARY", "BG_SECONDARY", "BG_ELEVATED", "BG_SUNKEN",
    "BORDER_SUBTLE", "BORDER", "BORDER_STRONG",
    "FG", "FG_SECONDARY", "FG_MUTED", "FG_EMPHASIS",
    "ACCENT_PRIMARY", "ACCENT_PRIMARY_HOVER", "ACCENT_SECONDARY", "HUD_GREEN",
    "HOSTILE", "HOSTILE_DEEP", "WARNING", "WARNING_DEEP",
    "FRIENDLY", "FRIENDLY_DEEP", "NEUTRAL", "INFO", "INFO_HOVER",
    "PURPLE", "ORANGE",
    "FONT_MONO_STACK", "FONT_CONDENSED_STACK", "FONT_SANS_STACK", "FONT_DISPLAY_STACK",
    "FS_WARN", "FS_CAUTION", "FS_BODY", "FS_MIN", "FS_LEGACY",
    "S1", "S2", "S3", "S4", "S5", "S6",
    "R_XS", "R_SM", "R_MD", "R_LG",
    "FLASH_WARNING_PERIOD_MS", "FLASH_CAUTION_PERIOD_MS",
    "Thresholds", "THRESHOLDS",
]
