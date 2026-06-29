"""
MilStdMessageBox — MIL-STD-1472H §5.17.10.7.3 合規錯誤訊息對話框

依據 MIL-STD-1472H 第 5.17.10.7.3 節，錯誤訊息必須：
    (a) **Constructive（建設性）** — 告知操作員如何排除，
        而非僅指出錯誤本身。
    (b) **Neutral tone（中立語氣）** — 不使用指責性字眼
        （如「你錯了」、「非法輸入」），改以系統視角陳述。
    (c) **Action-oriented（可操作）** — 至少提供一個可採取的下一步。
    (d) **Specific（具體）** — 指明哪個欄位、哪個參數、哪個閾值。

因此本模組提供 `MilStdMessageBox`，強制三欄結構：
    ┌──────────────────────────────────────────┐
    │  [ICON]  WHAT HAPPENED  （現象描述）      │
    │          ─────────────                   │
    │          DETAIL          （具體資訊）    │
    │          ─────────────                   │
    │          RECOMMENDED ACTION （建議操作） │
    └──────────────────────────────────────────┘

範例
----
錯誤示範（違反中立語氣）：
    「你輸入了非法的高度值！」

正確示範（MIL-STD-1472H 合規）：
    ┌── CAUTION ───────────────────────────────┐
    │ 現象：輸入之飛行高度超出允許範圍              │
    │ 資訊：已輸入 450 m，上限為 120 m             │
    │ 建議：請將「飛行高度」欄位改為 120 m 以下，    │
    │      或切換至「高空特許模式」並通過認證程序   │
    └──────────────────────────────────────────┘
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)

try:
    from ui.resources.tactical_theme import TacticalColors, TacticalFonts
except Exception:  # pragma: no cover
    class TacticalColors:  # type: ignore[no-redef]
        HOSTILE = "#FF003C"
        WARNING = "#FFB703"
        FRIENDLY = "#00E676"
        NEUTRAL = "#00B4D8"
        FG_PRIMARY = "#E0E1DD"
        FG_SECONDARY = "#A8B2BD"
        FG_EMPHASIS = "#FFB703"

    class TacticalFonts:  # type: ignore[no-redef]
        # Fallback：仍盡量遵循 MIL-STD-1472H 5.12.6.5.1 字型堆疊優先序
        # 1. Roboto Condensed / Rajdhani（窄體無襯線；軍用 HUD 標準字體之一）
        # 2. Segoe UI / Inter（系統主預設）
        # 3. sans-serif 通用 fallback
        _SANS_FALLBACK = ("Segoe UI", "Inter", "Microsoft YaHei", "微軟正黑體", "sans-serif")
        _CONDENSED_FALLBACK = ("Rajdhani", "Roboto Condensed", "Segoe UI Semibold",
                                "Arial Narrow", "sans-serif")

        @staticmethod
        def _pick(stack, size):
            from PyQt6.QtGui import QFontDatabase
            families = QFontDatabase.families()
            for n in stack:
                if n in families:
                    return QFont(n, size)
            return QFont(stack[-1], size)

        @classmethod
        def condensed(cls, size: int = 10, bold: bool = True,
                      letter_spacing: float = 1.0) -> QFont:
            f = cls._pick(cls._CONDENSED_FALLBACK, size)
            f.setBold(bold)
            return f

        @classmethod
        def sans(cls, size: int = 9) -> QFont:
            return cls._pick(cls._SANS_FALLBACK, size)


class Severity(Enum):
    """訊息嚴重度。"""
    INFO = "info"           # 青 — 資訊
    CAUTION = "caution"     # 黃 — 警告
    WARNING = "warning"     # 紅 — 警報（阻斷性）
    SUCCESS = "success"     # 綠 — 操作成功


# 色彩映射
_SEVERITY_COLOR: dict[Severity, str] = {
    Severity.INFO: TacticalColors.NEUTRAL,
    Severity.CAUTION: TacticalColors.WARNING,
    Severity.WARNING: TacticalColors.HOSTILE,
    Severity.SUCCESS: TacticalColors.FRIENDLY,
}

# 標題文字（中立語氣，英中雙語對齊 NATO 慣例）
_SEVERITY_TITLE: dict[Severity, str] = {
    Severity.INFO: "INFORMATION  /  資訊",
    Severity.CAUTION: "CAUTION  /  警告",
    Severity.WARNING: "WARNING  /  警報",
    Severity.SUCCESS: "OPERATION COMPLETE  /  操作完成",
}


class MilStdMessageBox(QDialog):
    """MIL-STD-1472H §5.17.10.7.3 合規訊息對話框。

    Parameters
    ----------
    parent : QWidget | None
        父視窗。
    severity : Severity
        嚴重度。
    phenomenon : str
        現象描述：發生什麼事（中立陳述，不指責操作員）。
    detail : str
        具體資訊：涉及的欄位、參數、閾值等細節。
    recommended_action : str
        建設性建議：操作員下一步該怎麼做。
    title : str | None
        視窗標題；None 時使用 severity 對應標題。
    allow_cancel : bool
        是否提供「取消」按鈕（預設 True）。

    範例
    ----
    >>> box = MilStdMessageBox(
    ...     severity=Severity.CAUTION,
    ...     phenomenon="輸入之飛行高度超出允許範圍。",
    ...     detail="已輸入 450 m；法規上限 120 m。",
    ...     recommended_action=(
    ...         "請修改『飛行高度』欄位至 120 m 以下，\\n"
    ...         "或啟用『高空特許模式』並上傳許可證明。"
    ...     ),
    ... )
    >>> box.exec()
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        severity: Severity = Severity.INFO,
        phenomenon: str = "",
        detail: str = "",
        recommended_action: str = "",
        title: Optional[str] = None,
        allow_cancel: bool = True,
    ) -> None:
        super().__init__(parent)

        self._severity = severity
        self.setWindowTitle(title or _SEVERITY_TITLE[severity])
        self.setModal(True)
        # 無圓角、視窗提示
        self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False)

        self._build_ui(phenomenon, detail, recommended_action, allow_cancel)

        self.setMinimumWidth(460)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def _build_ui(
        self,
        phenomenon: str,
        detail: str,
        recommended_action: str,
        allow_cancel: bool,
    ) -> None:
        """建立三欄式內容結構。"""
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        # ── 標題列（嚴重度色條） ──────────────────────────
        color = _SEVERITY_COLOR[self._severity]
        header = QFrame()
        header.setFixedHeight(28)
        header.setStyleSheet(
            f"background-color: {color}; border: none;"
        )
        hbox = QHBoxLayout(header)
        hbox.setContentsMargins(10, 0, 10, 0)
        title = QLabel(_SEVERITY_TITLE[self._severity])
        title.setFont(TacticalFonts.condensed(size=10, bold=True,
                                              letter_spacing=2.0))
        title.setStyleSheet(
            f"color: {TacticalColors.FG_PRIMARY}; background: transparent;"
        )
        hbox.addWidget(title, 1)
        root.addWidget(header)

        # ── 主內容：現象 / 資訊 / 建議 三區 ────────────────
        def _make_block(caption: str, body: str, caption_color: str) -> QWidget:
            """建構單一區塊：小標 + 本文。"""
            w = QWidget()
            v = QVBoxLayout(w)
            v.setContentsMargins(0, 0, 0, 0)
            v.setSpacing(2)

            cap = QLabel(caption)
            cap.setFont(TacticalFonts.condensed(size=8, bold=True,
                                                letter_spacing=1.5))
            cap.setStyleSheet(f"color: {caption_color};")
            v.addWidget(cap)

            bd = QLabel(body)
            bd.setFont(TacticalFonts.sans(size=10))
            bd.setWordWrap(True)
            bd.setTextInteractionFlags(
                Qt.TextInteractionFlag.TextSelectableByMouse
            )
            # QLabel 預設色已是 FG（透過全域 QSS），無需 inline 重設
            v.addWidget(bd)
            return w

        if phenomenon:
            root.addWidget(_make_block("WHAT HAPPENED  ／ 現象",
                                       phenomenon, color))
        if detail:
            root.addWidget(_make_block("DETAIL  ／ 資訊",
                                       detail, TacticalColors.FG_SECONDARY))
        # 建設性建議為核心：若未提供，自動附上中立 fallback
        action_body = recommended_action.strip() or (
            "請檢閱上述資訊，或聯絡系統操作員進行下一步確認。"
        )
        root.addWidget(_make_block("RECOMMENDED ACTION  ／ 建議操作",
                                   action_body, TacticalColors.FG_EMPHASIS))

        # ── 按鈕列 ───────────────────────────────────────
        btns = QDialogButtonBox(self)
        ok_btn = btns.addButton(
            "ACKNOWLEDGE  /  確認",
            QDialogButtonBox.ButtonRole.AcceptRole,
        )
        ok_btn.setObjectName("btnPrimary")  # 新版 QSS QPushButton#btnPrimary
        ok_btn.setDefault(True)
        if allow_cancel:
            cancel_btn = btns.addButton(
                "CANCEL  /  取消",
                QDialogButtonBox.ButtonRole.RejectRole,
            )
            cancel_btn.setObjectName("btnWarn")
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        root.addWidget(btns)

    # ------------------------------------------------------------------
    # 靜態便捷方法
    # ------------------------------------------------------------------
    @staticmethod
    def info(parent: Optional[QWidget], phenomenon: str,
             recommended_action: str = "", detail: str = "") -> int:
        """資訊對話框。"""
        return MilStdMessageBox(
            parent, severity=Severity.INFO,
            phenomenon=phenomenon, detail=detail,
            recommended_action=recommended_action,
        ).exec()

    @staticmethod
    def caution(parent: Optional[QWidget], phenomenon: str,
                recommended_action: str, detail: str = "") -> int:
        """警告對話框。"""
        return MilStdMessageBox(
            parent, severity=Severity.CAUTION,
            phenomenon=phenomenon, detail=detail,
            recommended_action=recommended_action,
        ).exec()

    @staticmethod
    def warning(parent: Optional[QWidget], phenomenon: str,
                recommended_action: str, detail: str = "") -> int:
        """警報對話框（紅，阻斷性）。"""
        return MilStdMessageBox(
            parent, severity=Severity.WARNING,
            phenomenon=phenomenon, detail=detail,
            recommended_action=recommended_action,
        ).exec()

    @staticmethod
    def success(parent: Optional[QWidget], phenomenon: str,
                recommended_action: str = "", detail: str = "") -> int:
        """成功回饋對話框。"""
        return MilStdMessageBox(
            parent, severity=Severity.SUCCESS,
            phenomenon=phenomenon, detail=detail,
            recommended_action=recommended_action,
            allow_cancel=False,
        ).exec()


__all__ = ["MilStdMessageBox", "Severity"]
