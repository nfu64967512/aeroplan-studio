"""擷取可行性探針：Cesium（QWebEngineView）的內容抓不抓得到。

⚠️ 這是整個「錄影 + 截圖」需求的成敗點。QWidget.grab() 走 Qt 的繪圖路徑，
   而 QWebEngineView 的內容是另一個 GPU 行程合成的 —— 很多情況下 grab()
   會回傳一片空白，而且**不會報錯**。所以先量，不要假設。

用法：
    python scripts/_capture_probe.py
輸出：runs/artifacts/_probe/ 底下三張圖 + 每張的非空白統計。
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT))

OUT = _ROOT / "runs" / "artifacts" / "_probe"
OUT.mkdir(parents=True, exist_ok=True)


def stats(path: Path) -> str:
    """回報一張圖有多「不空白」——單一顏色佔比與獨立顏色數。"""
    from PIL import Image
    import numpy as np
    im = Image.open(path).convert("RGB")
    a = np.asarray(im).reshape(-1, 3)
    uniq, counts = np.unique(a, axis=0, return_counts=True)
    top = counts.max() / len(a)
    return ("%dx%d  獨立色 %d  最大單色佔比 %.1f%%  %s"
            % (im.width, im.height, len(uniq), top * 100,
               "⚠️ 幾乎空白" if top > 0.97 or len(uniq) < 12 else "有內容"))


def main() -> int:
    from PyQt6.QtCore import Qt, QTimer
    from PyQt6.QtWidgets import QApplication
    # ⚠️ WebEngine 必須在 QApplication 之前 import（PyQt6 的已知要求）
    from PyQt6.QtWebEngineWidgets import QWebEngineView  # noqa: F401

    QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
    app = QApplication(sys.argv)

    try:
        from ui.resources.aeroplan_theme import apply_theme
        apply_theme(app)
    except Exception as e:                       # pragma: no cover
        print("  主題套用失敗（不影響探針）:", e)

    from ui.main_window import MainWindow
    t0 = time.time()
    win = MainWindow()
    win.resize(1600, 950)
    win.show()
    print("  建構 + show 花了 %.1f s" % (time.time() - t0))

    # 讓 Cesium 有時間載入。⚠️ 不能只 sleep —— 事件迴圈不轉，WebEngine
    #    永遠不會畫任何東西。
    deadline = time.time() + 25.0
    while time.time() < deadline:
        app.processEvents()
        time.sleep(0.02)

    results = {}

    # ① QWidget.grab() —— Qt 自己的繪圖路徑
    p1 = OUT / "a_widget_grab.png"
    win.grab().save(str(p1))
    results["QWidget.grab()"] = stats(p1)

    # ② QScreen.grabWindow(winId) —— 走桌面合成，理論上含 GPU 內容
    p2 = OUT / "b_screen_grabwindow.png"
    scr = app.primaryScreen()
    scr.grabWindow(int(win.winId())).save(str(p2))
    results["QScreen.grabWindow(winId)"] = stats(p2)

    # ③ 整個螢幕（一定含所有內容，但會拍到桌面其他東西）
    p3 = OUT / "c_screen_full.png"
    scr.grabWindow(0).save(str(p3))
    results["QScreen.grabWindow(0)"] = stats(p3)

    print()
    print("=== 擷取方式比較 ===")
    for k, v in results.items():
        print("  %-28s %s" % (k, v))
    print()
    print("  檔案在", OUT)

    QTimer.singleShot(0, app.quit)
    app.exec()
    return 0


if __name__ == "__main__":
    sys.exit(main())
