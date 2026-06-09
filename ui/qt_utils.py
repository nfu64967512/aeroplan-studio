"""ui.qt_utils — 中性 Qt UI 小工具
================================

只依賴 PyQt6，不耦合任何主題（tactical_theme / aeroplan_theme），供所有 UI
模組共用。集中收斂全專案重複的 Qt 樣板。
"""
from __future__ import annotations


def repolish(widget) -> None:
    """重新套用 QSS 樣式。

    在 ``widget.setProperty(name, value)`` 改變動態 property 後呼叫，讓 QSS 的
    property selector（如 ``[role="..."]`` / ``[msgSeverity="..."]``）即時重新生效。

    (1-7 去重) 等同全 UI 多處重複的樣板
    ``widget.style().unpolish(widget); widget.style().polish(widget)``，行為逐位元相同。
    """
    widget.style().unpolish(widget)
    widget.style().polish(widget)
