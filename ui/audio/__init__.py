"""ui/audio — AeroPlan Studio 警示音系統。

對齊 ADOS Mission Control `src/lib/audio-engine.ts` 的 9 種程序合成警示音。
公開介面：`AlertEngine`、`AlertId`。

Example::

    from ui.audio import AlertEngine, AlertId
    AlertEngine.instance().play(AlertId.LOW_BATTERY)
"""
from __future__ import annotations

from .alert_engine import AlertEngine, AlertId

__all__ = ["AlertEngine", "AlertId"]
