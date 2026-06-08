"""ui/audio/alert_engine.py — 警示音引擎（對齊 ADOS audio-engine.ts）。

9 種音效，頻率/波形/長度 1:1 移植自 ADOS。每個 AlertId 預設 5 秒 cooldown，
`FAILSAFE` 例外不受 cooldown 與 Master Mute 限制（MIL-STD-1472H §5.7.3.6.5）。

依賴：PyQt6.QtMultimedia.QSoundEffect（低延遲、無 codec 依賴）。
無音效裝置或載入失敗時 play() 退化為 logger.debug，禁丟例外。

WAV 檔位置：`ui/resources/audio/<alert_id>.wav`。
若檔案不存在，AlertEngine 會於首次 play 時 lazy 生成（透過 numpy 合成）並落地。
亦可在開發階段執行 `python scripts/build_alert_wavs.py` 預先生成。
"""
from __future__ import annotations

import logging
import math
import struct
import time
import wave
from dataclasses import dataclass
from enum import IntEnum, auto
from pathlib import Path
from typing import Final, Optional

from PyQt6.QtCore import QObject, QUrl
from PyQt6.QtMultimedia import QSoundEffect


logger = logging.getLogger(__name__)


# ── 警示音 ID ─────────────────────────────────────────────────────
class AlertId(IntEnum):
    """警示音識別。對應 ADOS audio-engine.ts 的 9 種音。"""
    LOW_BATTERY = auto()
    GPS_LOST = auto()
    RC_LOST = auto()
    FAILSAFE = auto()
    ARM = auto()
    DISARM = auto()
    WAYPOINT_REACHED = auto()
    MISSION_COMPLETE = auto()
    ERROR = auto()


# ── 波形種類 ──────────────────────────────────────────────────────
class _Wave(IntEnum):
    SINE = 0
    SQUARE = 1
    SAWTOOTH = 2


# ── 單段音定義 ────────────────────────────────────────────────────
@dataclass(frozen=True)
class _Segment:
    """單段音：頻率、波形、毫秒。"""
    freq_hz: float
    wave: _Wave
    duration_ms: int


# ── ADOS 對照表（嚴格 1:1 移植）──────────────────────────────────
# 每筆 = list[_Segment]，依序播放，段間連續無 gap。
_ALERT_PATTERNS: Final[dict[AlertId, list[_Segment]]] = {
    AlertId.LOW_BATTERY: [
        _Segment(880, _Wave.SQUARE, 150),
        _Segment(660, _Wave.SQUARE, 150),
        _Segment(440, _Wave.SQUARE, 150),
    ],
    AlertId.GPS_LOST: [
        _Segment(1200, _Wave.SINE, 100),
        _Segment(1200, _Wave.SINE, 100),
        _Segment(800,  _Wave.SINE, 100),
    ],
    AlertId.RC_LOST: [
        _Segment(600,  _Wave.SAWTOOTH, 100),
        _Segment(900,  _Wave.SAWTOOTH, 100),
        _Segment(1200, _Wave.SAWTOOTH, 100),
    ],
    AlertId.FAILSAFE: [
        _Segment(1400, _Wave.SQUARE, 80),
        _Segment(800,  _Wave.SQUARE, 80),
        _Segment(1400, _Wave.SQUARE, 80),
        _Segment(800,  _Wave.SQUARE, 80),
        _Segment(1400, _Wave.SQUARE, 80),
    ],
    AlertId.ARM: [
        _Segment(523, _Wave.SINE, 80),
        _Segment(659, _Wave.SINE, 80),
        _Segment(784, _Wave.SINE, 80),
    ],
    AlertId.DISARM: [
        _Segment(784, _Wave.SINE, 80),
        _Segment(659, _Wave.SINE, 80),
        _Segment(523, _Wave.SINE, 80),
    ],
    AlertId.WAYPOINT_REACHED: [
        _Segment(1047, _Wave.SINE, 60),
    ],
    AlertId.MISSION_COMPLETE: [
        _Segment(523,  _Wave.SINE, 80),
        _Segment(659,  _Wave.SINE, 80),
        _Segment(784,  _Wave.SINE, 80),
        _Segment(1047, _Wave.SINE, 120),
    ],
    AlertId.ERROR: [
        _Segment(200, _Wave.SQUARE, 200),
        _Segment(200, _Wave.SQUARE, 200),
    ],
}


# ── WAV 合成參數 ──────────────────────────────────────────────────
_SAMPLE_RATE: Final[int] = 44100
_INT16_MAX: Final[int] = 32767
_ATTACK_MS: Final[int] = 3      # 進場毫秒（避免 click）
_RELEASE_MS: Final[int] = 5     # 離場毫秒

# ── Cooldown ──────────────────────────────────────────────────────
_COOLDOWN_SEC: Final[float] = 5.0


def _synth_segment(seg: _Segment, sample_rate: int = _SAMPLE_RATE) -> list[int]:
    """合成單段 PCM int16 樣本（含微小 attack/release，避免 click）。"""
    n = int(sample_rate * seg.duration_ms / 1000)
    if n <= 0:
        return []

    samples: list[int] = []
    period = sample_rate / seg.freq_hz if seg.freq_hz > 0 else 1.0
    attack_n = int(sample_rate * _ATTACK_MS / 1000)
    release_n = int(sample_rate * _RELEASE_MS / 1000)
    amplitude = 0.35  # 避免 clip，留 headroom

    for i in range(n):
        # 波形
        t = i / sample_rate
        phase = (i % period) / period
        if seg.wave == _Wave.SINE:
            v = math.sin(2 * math.pi * seg.freq_hz * t)
        elif seg.wave == _Wave.SQUARE:
            v = 1.0 if phase < 0.5 else -1.0
        else:  # SAWTOOTH
            v = 2.0 * phase - 1.0

        # 微小 attack / release envelope
        env = 1.0
        if i < attack_n:
            env = i / max(1, attack_n)
        elif i >= n - release_n:
            env = (n - i) / max(1, release_n)

        s = int(v * amplitude * env * _INT16_MAX)
        s = max(-_INT16_MAX, min(_INT16_MAX, s))
        samples.append(s)
    return samples


def _synth_pattern(alert: AlertId) -> bytes:
    """將整段 pattern 合成為 mono 16-bit PCM bytes。"""
    pattern = _ALERT_PATTERNS[alert]
    pcm: list[int] = []
    for seg in pattern:
        pcm.extend(_synth_segment(seg))
    return struct.pack(f"<{len(pcm)}h", *pcm)


def render_wav(alert: AlertId, out_path: Path) -> None:
    """將指定 alert 的 PCM 寫入 WAV 檔（mono / 16-bit / 44.1 kHz）。"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    pcm = _synth_pattern(alert)
    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(_SAMPLE_RATE)
        wf.writeframes(pcm)


# ── AlertEngine 單例 ──────────────────────────────────────────────
class AlertEngine(QObject):
    """警示音引擎。執行緒安全的單例介面。

    用法::

        AlertEngine.instance().play(AlertId.LOW_BATTERY)
    """

    _instance: Optional["AlertEngine"] = None

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._effects: dict[AlertId, QSoundEffect] = {}
        self._last_played: dict[AlertId, float] = {}
        self._muted: bool = False
        self._volume_linear: float = 1.0  # 0.0–1.0（QSoundEffect 介面）
        # WAV 落地路徑
        self._wav_dir: Path = (
            Path(__file__).resolve().parent.parent
            / "resources" / "audio"
        )

    @classmethod
    def instance(cls) -> "AlertEngine":
        """取得單例。若尚未建立則延遲建立。"""
        if cls._instance is None:
            cls._instance = AlertEngine()
        return cls._instance

    # ── 公開介面 ──────────────────────────────────────────────
    def play(self, alert: AlertId) -> None:
        """播放警示音。

        - FAILSAFE 忽略 Mute 與 cooldown（1472H §5.7.3.6.5）。
        - 其他音受 5 秒 cooldown 與 Master Mute 影響。
        - 無音效裝置或載入失敗時靜默 debug log。
        """
        now = time.monotonic()
        forced = (alert == AlertId.FAILSAFE)

        if not forced:
            # Mute 檢查
            if self._muted:
                logger.debug("AlertEngine: %s muted", alert.name)
                return
            # Cooldown 檢查
            last = self._last_played.get(alert, 0.0)
            if now - last < _COOLDOWN_SEC:
                logger.debug(
                    "AlertEngine: %s in cooldown (%.2fs left)",
                    alert.name, _COOLDOWN_SEC - (now - last),
                )
                return

        try:
            effect = self._get_effect(alert)
            if effect is None:
                return
            effect.setVolume(self._volume_linear)
            effect.play()
            self._last_played[alert] = now
        except Exception as exc:  # pragma: no cover — runtime guard
            logger.debug("AlertEngine play(%s) failed: %s", alert.name, exc)

    def set_muted(self, muted: bool) -> None:
        """設定 Master Mute；FAILSAFE 無視此旗標。"""
        self._muted = bool(muted)

    def is_muted(self) -> bool:
        return self._muted

    def set_volume_linear(self, v: float) -> None:
        """設定線性音量 0.0–1.0（與 QSoundEffect 介面一致）。"""
        self._volume_linear = max(0.0, min(1.0, float(v)))

    # ── 內部 ──────────────────────────────────────────────────
    def _get_effect(self, alert: AlertId) -> Optional[QSoundEffect]:
        """取（或建）QSoundEffect；首次取用時 lazy 合成 WAV。"""
        if alert in self._effects:
            return self._effects[alert]

        wav_path = self._wav_dir / f"{alert.name.lower()}.wav"
        if not wav_path.exists():
            try:
                render_wav(alert, wav_path)
                logger.info("AlertEngine: rendered %s", wav_path)
            except Exception as exc:
                logger.warning(
                    "AlertEngine: failed to render %s: %s", wav_path, exc
                )
                return None

        effect = QSoundEffect(self)
        effect.setSource(QUrl.fromLocalFile(str(wav_path)))
        effect.setLoopCount(1)
        # FAILSAFE 持續循環直到 ACK；外部呼叫 stop_alert 才停。
        if alert == AlertId.FAILSAFE:
            effect.setLoopCount(QSoundEffect.Loop.Infinite)
        self._effects[alert] = effect
        return effect

    def stop_alert(self, alert: AlertId) -> None:
        """停止指定 alert（主要用於 FAILSAFE 的 ACK）。"""
        eff = self._effects.get(alert)
        if eff is not None and eff.isPlaying():
            eff.stop()


__all__ = ["AlertEngine", "AlertId", "render_wav"]
