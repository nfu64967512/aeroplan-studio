"""ui/resources/aeroplan_theme/buttons.py — ADOS 風格按鈕工廠。

對齊 ADOS `src/components/ui/button.tsx` 的 variant × size 模型。

**Hero 變體使用警示**：`ButtonVariant.HERO`（lime `#dff140`）為「全畫面只應出現
1 個」的視覺重點。除以下情境外不得使用：
  1. SITL Launcher Dialog 的 Launch Demo Fleet (5) 按鈕。
  2. HUD 緊急 Disarm 動作。
PR review 必審。其餘需要強調的主動作改用 `ButtonVariant.PRIMARY`（藍 accent）。
"""
from __future__ import annotations

from enum import Enum
from typing import Optional

from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import QPushButton


class ButtonVariant(str, Enum):
    """按鈕語意變體，對應 QSS `[btnVariant="..."]` selector。"""
    PRIMARY = "primary"
    SECONDARY = "secondary"
    OUTLINE = "outline"
    GHOST = "ghost"
    DESTRUCTIVE = "destructive"   # 1472H 紅 — 不可逆動作
    SUCCESS = "success"           # 1472H 綠 — 正向確認
    HERO = "hero"                 # lime — 全畫面唯一視覺重點


class ButtonSize(str, Enum):
    """按鈕尺寸，對應 QSS `[btnSize="..."]` selector。"""
    XS = "xs"   # 24 px
    SM = "sm"   # 32 px
    MD = "md"   # 36 px（預設）
    LG = "lg"   # 44 px


# variant → icon 尺寸（hero 特別放大）
_ICON_PX = {
    ButtonSize.XS: 12,
    ButtonSize.SM: 14,
    ButtonSize.MD: 16,
    ButtonSize.LG: 18,
}


def make_button(
    text: str = "",
    *,
    variant: ButtonVariant = ButtonVariant.SECONDARY,
    size: ButtonSize = ButtonSize.MD,
    icon: Optional[QIcon] = None,
    object_name: Optional[str] = None,
    tooltip: Optional[str] = None,
    parent=None,
) -> QPushButton:
    """工廠：建立符合 ADOS 視覺體系的 QPushButton。

    Args:
        text: 按鈕文字，純 icon 按鈕可給空字串。
        variant: 語意變體（見 ButtonVariant 警示）。
        size: 尺寸。
        icon: 可選的 QIcon；hero variant 圖示自動放大。
        object_name: 覆寫 QPushButton objectName，方便 QSS 進一步個別樣式化。
        tooltip: 可選的滑鼠提示。
        parent: Qt parent。

    Returns:
        已設好 dynamic property 與 QSS 屬性的 QPushButton。
    """
    btn = QPushButton(text, parent)
    btn.setProperty("btnVariant", variant.value)
    btn.setProperty("btnSize", size.value)
    if object_name:
        btn.setObjectName(object_name)
    if icon is not None:
        btn.setIcon(icon)
        ipx = _ICON_PX[size]
        btn.setIconSize(QSize(ipx, ipx))
    if tooltip:
        btn.setToolTip(tooltip)
    btn.setCursor(Qt.CursorShape.PointingHandCursor)
    # 確保 QSS 屬性選擇器立即生效
    btn.style().unpolish(btn)
    btn.style().polish(btn)
    return btn


def set_loading(btn: QPushButton, loading: bool) -> None:
    """切換按鈕 loading 狀態（QSS `[loading="true"]` 取色，並禁互動）。

    注意：QSS `@keyframes` 不支援，spinner 動畫須以 QMovie 或 QPropertyAnimation
    自行繫於外掛 QLabel；本工具函式只負責屬性與啟用旗標。
    """
    btn.setProperty("loading", "true" if loading else "false")
    btn.setEnabled(not loading)
    btn.style().unpolish(btn)
    btn.style().polish(btn)


__all__ = ["ButtonVariant", "ButtonSize", "make_button", "set_loading"]
