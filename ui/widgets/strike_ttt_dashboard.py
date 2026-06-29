"""
StrikeTTTDashboard — 蜂群末端飽和打擊「STOT 同步狀態卡」儀表板

戰術價值
========
規劃完成後，指揮官最關心的問題是：
    「4 架 UAV 真的會同秒命中目標嗎？演算法如何吸收時間差？」

本儀表板把 `DiamondSwarmStrikePlanner.plan()` 的時間平衡結果視覺化：

    ┌─────────────────────────────────────────────────────────────┐
    │  STOT TIME-ON-TARGET DASHBOARD            mode: STOT  N=4   │
    ├─────────────────────────────────────────────────────────────┤
    │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐│
    │  │ UAV1 LEAD  │ │ UAV2 REAR  │ │ UAV3 LEFT  │ │ UAV4 RIGHT ││
    │  │  ─04:30s   │ │  ─04:30s   │ │  ─04:30s   │ │  ─04:30s   ││
    │  │ +66s delay │ │ +9s delay  │ │   READY    │ │ +18s delay ││
    │  │ d=8.4 km   │ │ d=11.0 km  │ │ d=10.9 km  │ │ d=8.9 km   ││
    │  │ brg 222°   │ │ brg 042°   │ │ brg 270°   │ │ brg 090°   ││
    │  │ alt 500 m  │ │ alt 580 m  │ │ alt 660 m  │ │ alt 740 m  ││
    │  └────────────┘ └────────────┘ └────────────┘ └────────────┘│
    └─────────────────────────────────────────────────────────────┘

關鍵 UI 元素：
* TTT (Time To Target)        ─ 大字 mono，命中倒數計時。STOT 模式 4 機數字一致
* Loiter / Delay 補償         ─ 小字 amber，顯示「演算法用什麼方式吸收時間差」
* 距離 / 方位 / 高度          ─ 小字灰，給指揮官的態勢資訊
* 卡片邊框色                  ─ 依 sysid 對應 UAV_PALETTE，與 3D 地圖 polyline 同色

完全遵循 MIL-STD-1472H 設計規範：
* 全部直角邊框（border-radius:0）
* 色彩語意嚴守 TABLE XL（紅 HOSTILE、黃 WARNING、綠 FRIENDLY、青 NEUTRAL、琥珀 EMPHASIS）
* 字型 mono / condensed / sans 三組 fallback chain
* §5.17.22.1.1.1：精讀數值 ≤ 1Hz；§5.17.18.10.6：無前導零
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget,
)

from core.strike.diamond_swarm_planner import UAVStrikePlan
from core.strike.geometry import bearing_deg, haversine
from ui.resources.aeroplan_theme.widgets import IconButton
from ui.resources.tactical_theme import TacticalColors as TC
from ui.resources.tactical_theme import TacticalFonts as TF


# ══════════════════════════════════════════════════════════════════════
#  資料結構
# ══════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class _CardSnapshot:
    """單張 UAV 卡片在規劃完成時鎖定的靜態資訊。

    倒數計時時這些值不變，只有 TTT 與狀態文字會更新。
    """
    sysid: int
    role: str
    color: str                  # UAV palette 識別色
    impact_time_s: float        # 命中時刻 (s)，所有機 + delay + flight + loiter
    delay_s: float              # 地面 NAV_DELAY 秒數（GROUND 策略）
    loiter_s: float             # 空中 NAV_LOITER_TIME 秒數（AIR 策略）
    distance_m: float           # 從 slot 起飛點到目標的水平距離 (m)
    bearing_deg: float          # slot 起飛點朝目標的方位角 (deg)
    cruise_alt_m: float         # 該機個體巡航高度 (m)
    attack_bearing_deg: float   # 攻擊方位角 (從目標看 P_pre)


# ══════════════════════════════════════════════════════════════════════
#  單張 UAV 狀態卡
# ══════════════════════════════════════════════════════════════════════
class UAVStatusCard(QFrame):
    """蜂群打擊單機狀態卡 — 顯示 TTT、補償時間、態勢資訊。

    狀態演進：
        IDLE     ─ 尚未規劃（顯示 "─"）
        ARMED    ─ 規劃完成、未啟動倒數
        COUNTING ─ 倒數中（每秒更新 TTT）
        IMPACT   ─ 命中時刻已到 / 已過
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._snapshot: Optional[_CardSnapshot] = None
        self._countdown_start: Optional[float] = None  # epoch s 或 None
        self._build_ui()
        self._apply_idle_style()

    # ─────────────────────────────────────────────────────────────
    #  UI 建構
    # ─────────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        """建立卡片內所有 label，採用 MIL-STD-1472H 直角邊框。"""
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Fixed)
        self.setMinimumWidth(180)
        self.setMinimumHeight(132)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)

        # ── Header: UAV ID + role badge ───────────────────────
        header = QHBoxLayout()
        header.setSpacing(4)
        self._lbl_id = QLabel("UAV—")
        self._lbl_id.setFont(TF.condensed(11, bold=True))
        self._lbl_id.setStyleSheet(f"color:{TC.FG_PRIMARY};")
        header.addWidget(self._lbl_id)

        self._lbl_role = QLabel("")
        self._lbl_role.setFont(TF.condensed(9, bold=True, letter_spacing=1.2))
        self._lbl_role.setStyleSheet(f"color:{TC.FG_SECONDARY};")
        header.addWidget(self._lbl_role)
        header.addStretch(1)

        # 狀態指示燈（圓點）
        self._lbl_status = QLabel("●")
        self._lbl_status.setFont(TF.mono(11, bold=True))
        self._lbl_status.setStyleSheet(f"color:{TC.FG_MUTED};")
        self._lbl_status.setToolTip("IDLE")
        header.addWidget(self._lbl_status)
        lay.addLayout(header)

        # ── TTT 大字 ──────────────────────────────────────────
        # MIL-STD-1472H §5.17.18.10.6：時間數字無前導零（"-04:30s" 而非 "-04:30s"）
        # 但 mm:ss 仍補零（"-04:05s"），這是業界通例
        self._lbl_ttt = QLabel("─:──s")
        self._lbl_ttt.setFont(TF.mono(20, bold=True))
        self._lbl_ttt.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_ttt.setStyleSheet(f"color:{TC.FG_EMPHASIS};")
        self._lbl_ttt.setToolTip("Time To Target (倒數至命中時刻)")
        lay.addWidget(self._lbl_ttt)

        # ── 補償時間（黃字） ──────────────────────────────────
        self._lbl_compensation = QLabel("─")
        self._lbl_compensation.setFont(TF.mono(9, bold=False))
        self._lbl_compensation.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_compensation.setStyleSheet(f"color:{TC.WARNING};")
        self._lbl_compensation.setToolTip(
            "演算法分配給此機的時間差吸收量\n"
            "GROUND 策略 → 地面 NAV_DELAY 起飛前等待\n"
            "AIR    策略 → P_pre 預備點 NAV_LOITER_TIME 盤旋"
        )
        lay.addWidget(self._lbl_compensation)

        # ── 細節資訊（灰字 mono） ─────────────────────────────
        self._lbl_dist = QLabel("d= ─ km")
        self._lbl_dist.setFont(TF.mono(9))
        self._lbl_dist.setStyleSheet(f"color:{TC.FG_SECONDARY};")
        lay.addWidget(self._lbl_dist)

        self._lbl_bearing = QLabel("brg ─°")
        self._lbl_bearing.setFont(TF.mono(9))
        self._lbl_bearing.setStyleSheet(f"color:{TC.FG_SECONDARY};")
        lay.addWidget(self._lbl_bearing)

        self._lbl_alt = QLabel("alt ─ m")
        self._lbl_alt.setFont(TF.mono(9))
        self._lbl_alt.setStyleSheet(f"color:{TC.FG_SECONDARY};")
        lay.addWidget(self._lbl_alt)

    # ─────────────────────────────────────────────────────────────
    #  狀態切換樣式
    # ─────────────────────────────────────────────────────────────
    def _apply_idle_style(self) -> None:
        """IDLE：未規劃時的低對比樣式（灰邊）。"""
        self.setStyleSheet(
            f"QFrame{{background:{TC.BG_SECONDARY};"
            f"border:1px solid {TC.BORDER_SUBTLE};border-radius:0;}}"
        )

    def _apply_armed_style(self, color: str) -> None:
        """ARMED：規劃完成時，邊框改為該機識別色（與 3D polyline 一致）。"""
        self.setStyleSheet(
            f"QFrame{{background:{TC.BG_SECONDARY};"
            f"border:2px solid {color};border-radius:0;}}"
        )

    # ─────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────
    def set_snapshot(self, snap: _CardSnapshot) -> None:
        """規劃完成後一次性設定靜態資訊（不啟動倒數）。"""
        self._snapshot = snap
        self._countdown_start = None

        # Header
        self._lbl_id.setText(f"UAV{snap.sysid}")
        self._lbl_id.setStyleSheet(
            f"color:{snap.color};font-weight:bold;"
        )
        self._lbl_role.setText(snap.role.upper())

        # TTT 顯示為「規劃命中時刻」（尚未開始倒數，故無 minus 號）
        self._lbl_ttt.setText(self._fmt_seconds(snap.impact_time_s, lead='+'))
        self._lbl_ttt.setStyleSheet(f"color:{TC.FG_EMPHASIS};")

        # 補償時間：依策略顯示 delay 或 loiter
        if snap.delay_s > 0.5:
            self._lbl_compensation.setText(f"+{snap.delay_s:4.1f}s GROUND DELAY")
        elif snap.loiter_s > 0.5:
            self._lbl_compensation.setText(f"+{snap.loiter_s:4.1f}s AIR LOITER")
        else:
            self._lbl_compensation.setText("READY (基準機)")

        # 細節（單位採 km、角度補零、高度整數）
        self._lbl_dist.setText(f"d= {snap.distance_m / 1000.0:5.2f} km")
        self._lbl_bearing.setText(
            f"brg {int(round(snap.bearing_deg)) % 360:03d}° "
            f"atk {int(round(snap.attack_bearing_deg)) % 360:03d}°"
        )
        self._lbl_alt.setText(f"alt {snap.cruise_alt_m:4.0f} m")

        # 狀態指示
        self._lbl_status.setStyleSheet(f"color:{TC.FRIENDLY};")
        self._lbl_status.setToolTip("ARMED — 規劃完成，等待 AUTO 啟動")

        self._apply_armed_style(snap.color)

    def start_countdown(self, t0: Optional[float] = None) -> None:
        """啟動倒數計時（4 機 AUTO 同步切換時呼叫）。

        Parameters
        ----------
        t0 : 計時起點（time.monotonic() 值），None = 當下時刻
        """
        if self._snapshot is None:
            return
        self._countdown_start = t0 if t0 is not None else time.monotonic()
        self._lbl_status.setStyleSheet(f"color:{TC.WARNING};")
        self._lbl_status.setToolTip("COUNTING — 倒數中")

    def stop_countdown(self) -> None:
        """停止倒數，回到 ARMED 顯示。"""
        if self._snapshot is None:
            self._apply_idle_style()
            return
        self._countdown_start = None
        self._lbl_ttt.setText(self._fmt_seconds(self._snapshot.impact_time_s, lead='+'))
        self._lbl_ttt.setStyleSheet(f"color:{TC.FG_EMPHASIS};")
        self._lbl_status.setStyleSheet(f"color:{TC.FRIENDLY};")
        self._lbl_status.setToolTip("ARMED")

    def clear(self) -> None:
        """清除所有資訊回到 IDLE。"""
        self._snapshot = None
        self._countdown_start = None
        self._lbl_id.setText("UAV—")
        self._lbl_id.setStyleSheet(f"color:{TC.FG_PRIMARY};")
        self._lbl_role.setText("")
        self._lbl_ttt.setText("─:──s")
        self._lbl_ttt.setStyleSheet(f"color:{TC.FG_MUTED};")
        self._lbl_compensation.setText("─")
        self._lbl_dist.setText("d= ─ km")
        self._lbl_bearing.setText("brg ─°")
        self._lbl_alt.setText("alt ─ m")
        self._lbl_status.setStyleSheet(f"color:{TC.FG_MUTED};")
        self._lbl_status.setToolTip("IDLE")
        self._apply_idle_style()

    def tick(self) -> None:
        """由 Dashboard timer 每秒呼叫一次更新 TTT 倒數。"""
        if self._snapshot is None or self._countdown_start is None:
            return
        elapsed = time.monotonic() - self._countdown_start
        ttt = self._snapshot.impact_time_s - elapsed

        if ttt > 0.5:
            # 倒數中 — 黃色琥珀
            self._lbl_ttt.setText(self._fmt_seconds(ttt, lead='-'))
            # 進入最後 30 秒切紅色強調
            color = TC.HOSTILE if ttt < 30.0 else TC.FG_EMPHASIS
            self._lbl_ttt.setStyleSheet(f"color:{color};")
        else:
            # 命中時刻已到
            self._lbl_ttt.setText("IMPACT")
            self._lbl_ttt.setStyleSheet(f"color:{TC.HOSTILE};font-weight:bold;")
            self._lbl_status.setStyleSheet(f"color:{TC.HOSTILE};")
            self._lbl_status.setToolTip("IMPACT — 命中時刻已到")

    # ─────────────────────────────────────────────────────────────
    #  輔助
    # ─────────────────────────────────────────────────────────────
    @staticmethod
    def _fmt_seconds(total_s: float, lead: str = '') -> str:
        """秒數 → "mm:ss" 格式（含前導符號）。

        > 1 hour 顯示 "h:mm:ss"。
        """
        sign = lead if lead in ('-', '+') else ''
        s = abs(int(round(total_s)))
        if s >= 3600:
            return f"{sign}{s // 3600}:{(s % 3600) // 60:02d}:{s % 60:02d}s"
        return f"{sign}{s // 60:02d}:{s % 60:02d}s"


# ══════════════════════════════════════════════════════════════════════
#  Dashboard 容器（4 張卡片 + 全局 header）
# ══════════════════════════════════════════════════════════════════════
class StrikeTTTDashboard(QWidget):
    """蜂群打擊 STOT 同步狀態儀表板。

    Public API
    ----------
    set_plans(plans, target_lat, target_lon)  ── 規劃完成後設定 4-12 張卡片
    start_countdown()                          ── 啟動倒數（同步觸發 AUTO 時呼叫）
    stop_countdown()                           ── 停止倒數
    clear()                                    ── 清除所有卡片回到 IDLE

    Signals
    -------
    impact_reached(int sysid)  ── 某機命中時刻已到時 emit
    """

    impact_reached = pyqtSignal(int)
    launch_requested = pyqtSignal()    # 按下 KAMIKAZE LAUNCH 按鈕（開環：mission + 排程起飛）
    terminal_sync_requested = pyqtSignal()  # 按下 TERMINAL SYNC 按鈕（閉環：同步釋放 + ToT 修正）
    abort_requested = pyqtSignal()     # 按下 ABORT 按鈕（停止倒數）
    DEFAULT_MAX_CARDS: int = 12

    def __init__(self, parent: Optional[QWidget] = None,
                 max_cards: int = DEFAULT_MAX_CARDS) -> None:
        super().__init__(parent)
        self._cards: List[UAVStatusCard] = []
        self._timer = QTimer(self)
        self._timer.setInterval(1000)             # 1Hz；§5.17.22.1.1.1 精讀數值
        self._timer.timeout.connect(self._on_tick)
        self._countdown_start: Optional[float] = None
        self._build_ui(max_cards)

    def _build_ui(self, max_cards: int) -> None:
        """建立 dashboard 外框 + N 張卡片（橫向排列）。"""
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Fixed)
        self.setStyleSheet(
            f"StrikeTTTDashboard{{background:{TC.BG_PRIMARY};"
            f"border-top:1px solid {TC.BORDER_DEFAULT};"
            f"border-radius:0;}}"
        )

        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 4, 6, 4)
        outer.setSpacing(4)

        # ── Header bar：標題 + 模式 + 機數 ────────────────────
        header = QHBoxLayout()
        header.setSpacing(8)
        self._lbl_title = QLabel("STOT TIME-ON-TARGET DASHBOARD")
        self._lbl_title.setFont(TF.condensed(10, bold=True, letter_spacing=1.5))
        self._lbl_title.setStyleSheet(f"color:{TC.FG_EMPHASIS};")
        header.addWidget(self._lbl_title)
        header.addStretch(1)

        self._lbl_mode = QLabel("mode: ─    N: ─    sync: ─")
        self._lbl_mode.setFont(TF.mono(9))
        self._lbl_mode.setStyleSheet(f"color:{TC.FG_SECONDARY};")
        header.addWidget(self._lbl_mode)

        # ── KAMIKAZE LAUNCH 按鈕（同步啟動 + 倒數）──────────────
        # 蜂群打擊上傳 SITL 後，按此按鈕由 GCS 端依各機 takeoff_delay_s 排程送
        # AUTO+ARM 命令，達成「較近 UAV 較晚起飛」實現命中時刻同步。
        # MIL-STD-1472H Table XL：HOSTILE（紅色）+ 警告圖示，emoji 改為 SVG warn 圖示
        self._btn_launch = IconButton('warn', 'KAMIKAZE  LAUNCH', tone='danger')
        self._btn_launch.setFont(TF.condensed(10, bold=True, letter_spacing=2.0))
        self._btn_launch.setEnabled(False)
        self._btn_launch.setToolTip(
            "依各機 takeoff_delay_s 在 GCS 端排程送 AUTO+ARM 命令\n"
            "最遠的 UAV 立即起飛、最近的 UAV 延遲後起飛 → 同秒命中"
        )
        self._btn_launch.clicked.connect(self.launch_requested.emit)
        header.addWidget(self._btn_launch)

        # ── TERMINAL SYNC 按鈕（閉環同步打擊）──────────────────
        # 各機已在空中巡航後按此 → GCS 以 FleetRegistry 共享黑板讀全機態勢，
        # 等距 push point 同步釋放 + 飛行中 ToT 速度修正 → 命中散度壓到秒級。
        # 與 KAMIKAZE LAUNCH（開環起飛排程）互補：先 LAUNCH 起飛巡航，再 SYNC 收尾。
        self._btn_tsync = IconButton('fly_to', 'TERMINAL  SYNC', tone='primary')
        self._btn_tsync.setFont(TF.condensed(10, bold=True, letter_spacing=2.0))
        self._btn_tsync.setEnabled(False)
        self._btn_tsync.setToolTip(
            "閉環終端同步打擊：讀 FleetRegistry 全機態勢 →\n"
            "等距 push point 同步釋放 + 飛行中 ToT 速度修正 → 同時命中\n"
            "（各機指令皆為全機狀態函數，確保資訊互通）"
        )
        self._btn_tsync.clicked.connect(self.terminal_sync_requested.emit)
        header.addWidget(self._btn_tsync)

        # ── ABORT 按鈕（停止倒數，使用者可重新規劃） ──
        self._btn_abort = QPushButton("ABORT")
        self._btn_abort.setFont(TF.condensed(9, bold=True))
        self._btn_abort.setEnabled(False)
        self._btn_abort.setStyleSheet(
            f"QPushButton{{background:{TC.BG_ELEVATED};color:{TC.WARNING};"
            f"border:1px solid {TC.WARNING};padding:4px 8px;border-radius:0;}}"
            f"QPushButton:hover{{background:{TC.WARNING};color:{TC.BG_PRIMARY};}}"
            f"QPushButton:disabled{{color:{TC.FG_MUTED};"
            f"border:1px solid {TC.BORDER_SUBTLE};}}"
        )
        self._btn_abort.setToolTip("中止倒數計時（不會發送 RTL 給已起飛的 UAV）")
        self._btn_abort.clicked.connect(self.abort_requested.emit)
        header.addWidget(self._btn_abort)

        outer.addLayout(header)

        # ── 卡片橫列（最多 max_cards 張，預先建立全部，依需要顯示） ──
        cards_row = QHBoxLayout()
        cards_row.setSpacing(4)
        for _ in range(max_cards):
            card = UAVStatusCard(self)
            card.setVisible(False)
            self._cards.append(card)
            cards_row.addWidget(card, 1)
        outer.addLayout(cards_row)

    # ─────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────
    def set_plans(self,
                  plans: Sequence[UAVStrikePlan],
                  target_lat: float,
                  target_lon: float,
                  timing_mode: str = 'STOT',
                  delay_strategy: str = 'GROUND') -> None:
        """規劃完成後一次性更新所有卡片資訊。

        Parameters
        ----------
        plans          : DiamondSwarmStrikePlanner.plan() 的回傳結果
        target_lat/lon : 用於計算各機到目標的距離 / 方位
        timing_mode    : 'STOT' / 'DTOT'，僅顯示用
        delay_strategy : 'GROUND' / 'AIR'，僅顯示用
        """
        n = len(plans)
        if n == 0:
            self.clear()
            return

        # 計算各機命中時刻 (= delay + flight + loiter)
        impact_times: List[float] = []
        for p in plans:
            t = float(p.takeoff_delay_s) + float(p.final_leg_time_s) + float(p.loiter_time_s)
            impact_times.append(t)
        sync_err = (max(impact_times) - min(impact_times)) if impact_times else 0.0

        # 更新 header
        self._lbl_mode.setText(
            f"mode: {timing_mode}    "
            f"N: {n}    "
            f"strategy: {delay_strategy}    "
            f"sync_err: {sync_err:5.2f}s"
        )

        # 同步誤差大於 1s → header 變黃警告
        if sync_err > 1.0:
            self._lbl_mode.setStyleSheet(f"color:{TC.WARNING};")
        else:
            self._lbl_mode.setStyleSheet(f"color:{TC.FRIENDLY};")

        # 規劃完成 → 啟用 LAUNCH / TERMINAL SYNC 按鈕（ABORT 仍維持 disabled）
        self._btn_launch.setEnabled(True)
        self._btn_tsync.setEnabled(True)
        self._btn_abort.setEnabled(False)

        # 配給卡片
        for idx, card in enumerate(self._cards):
            if idx < n:
                p = plans[idx]
                # 從 slot 起飛點計算到目標的距離與方位
                slot_lat = p.slot_lat if abs(p.slot_lat) > 1e-9 else p.base_lat
                slot_lon = p.slot_lon if abs(p.slot_lon) > 1e-9 else p.base_lon
                dist_m = haversine(slot_lat, slot_lon, target_lat, target_lon)
                brg = bearing_deg(slot_lat, slot_lon, target_lat, target_lon)
                snap = _CardSnapshot(
                    sysid=p.sysid,
                    role=p.role,
                    color=TC.uav_color(p.sysid),
                    impact_time_s=impact_times[idx],
                    delay_s=float(p.takeoff_delay_s),
                    loiter_s=float(p.loiter_time_s),
                    distance_m=dist_m,
                    bearing_deg=brg,
                    cruise_alt_m=float(p.cruise_alt) if p.cruise_alt > 0 else 0.0,
                    attack_bearing_deg=float(p.attack_bearing_deg),
                )
                card.set_snapshot(snap)
                card.setVisible(True)
            else:
                card.clear()
                card.setVisible(False)

    def start_countdown(self) -> None:
        """啟動 1Hz 倒數計時（建議在「同步啟動 AUTO」時呼叫）。"""
        if not self._timer.isActive():
            self._countdown_start = time.monotonic()
            for c in self._cards:
                if c.isVisible():
                    c.start_countdown(self._countdown_start)
            self._timer.start()
        # 倒數中（開環打擊執行）：鎖住 LAUNCH + TERMINAL SYNC、開 ABORT（兩模式互斥）
        self.set_strike_running(True)

    def stop_countdown(self) -> None:
        """停止倒數計時。"""
        if self._timer.isActive():
            self._timer.stop()
        self._countdown_start = None
        for c in self._cards:
            c.stop_countdown()
        # 結束 → 還原（規劃尚在則重新啟用兩個啟動鈕；ABORT 禁用）
        self.set_strike_running(False)

    def set_strike_running(self, running: bool) -> None:
        """打擊執行中（開環倒數 或 閉環終端同步）→ 鎖住兩個啟動鈕、開 ABORT；結束 → 還原。

        確保 KAMIKAZE LAUNCH 與 TERMINAL SYNC 互斥：一個正在跑時不會誤觸另一個
        （否則 AUTO 任務與 GUIDED 指令會互相打架）。
        """
        if running:
            self._btn_launch.setEnabled(False)
            self._btn_tsync.setEnabled(False)
            self._btn_abort.setEnabled(True)
        else:
            any_visible = any(c.isVisible() for c in self._cards)
            self._btn_launch.setEnabled(any_visible)
            self._btn_tsync.setEnabled(any_visible)
            self._btn_abort.setEnabled(False)

    def clear(self) -> None:
        """清除全部卡片回到 IDLE 狀態。"""
        self.stop_countdown()
        for c in self._cards:
            c.clear()
            c.setVisible(False)
        self._lbl_mode.setText("mode: ─    N: ─    sync: ─")
        self._lbl_mode.setStyleSheet(f"color:{TC.FG_SECONDARY};")
        self._btn_launch.setEnabled(False)
        self._btn_tsync.setEnabled(False)
        self._btn_abort.setEnabled(False)

    # ─────────────────────────────────────────────────────────────
    #  Timer callback
    # ─────────────────────────────────────────────────────────────
    def _on_tick(self) -> None:
        """1Hz 更新 — 每張卡片自行計算 TTT。"""
        for c in self._cards:
            if c.isVisible():
                c.tick()
