"""scripts/build_alert_wavs.py — 一次性開發工具：生成所有警示音 WAV。

執行方式（自 repo 根目錄）::

    python scripts/build_alert_wavs.py

輸出至 `ui/resources/audio/<alert_id>.wav`，共 9 檔。
若 WAV 已存在會覆寫。CI 不需執行；AlertEngine 首次播放時亦會 lazy 生成。
"""
from __future__ import annotations

import sys
from pathlib import Path

# 確保可從 repo 根目錄執行
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from ui.audio.alert_engine import AlertId, render_wav   # noqa: E402


def main() -> int:
    out_dir = _REPO_ROOT / "ui" / "resources" / "audio"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[build_alert_wavs] output dir: {out_dir}")
    for alert in AlertId:
        out_path = out_dir / f"{alert.name.lower()}.wav"
        render_wav(alert, out_path)
        print(f"  [OK] {out_path.name}  ({out_path.stat().st_size} bytes)")
    print(f"[build_alert_wavs] done — {len(list(AlertId))} files written.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
