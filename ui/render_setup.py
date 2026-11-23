# -*- coding: utf-8 -*-
"""render_setup.py — Qt 視窗呈現（present）設定：讓 QtWebEngine（3D／2D 地圖）在 Windows 上跑滿 60 FPS。

2026-10-17 量測（tools/cesium_fps_probe.py，Qt 6.11 / QtWebEngine 6.11 / Chromium 140，RTX 4070，60 Hz 螢幕）：
  任何「每幀都在變」的網頁 —— 連一個用 rAF 移動的 div 都算 —— 在 QWebEngineView 裡都只有 ~21.8 FPS
  （幀間隔集中在 47 ms），跟 Cesium、架數、視窗大小、Chromium 旗標（--disable-gpu-vsync 只讓頁面空轉到
  1200 FPS，Qt 端照樣 22）、Windows 計時器解析度、D3D 畫面延遲上限都無關。
  開關在 Qt 視窗端：QWebEngineView 內部是 QQuickWidget，整個頂層視窗改走 RHI(D3D11) 交換鏈。
  QSurfaceFormat 預設 swapInterval = 0（Qt 端 present 不等 vsync，出圖節奏交給 Chromium 自己的 60 Hz BeginFrame）
  ⇒ 同一頁 60.0 FPS（Qt 端實際合成次數也是 60.0）。
  ⚠ 確切機制沒有查明：幀間隔集中在 46.9 ms、次峰 31.25 ms ＝ Windows 預設計時器刻度 15.625 ms 的 3 倍／2 倍，
  **不是** vsync 的倍數（50.0／33.4 ms）⇒ 比較像「present 的等待＋某處用粗計時器輪詢」，不是單純等 3 個 vsync。
  只在這台 60 Hz 螢幕量過；高更新率螢幕上是否還是上限 60 也沒驗。
  ⚠ swapInterval 0 時 Qt 的 flip 交換鏈會帶 DXGI ALLOW_TEARING：一般視窗由 DWM 合成不會撕裂，被升格成獨立翻頁
  （全螢幕／MPO）時可能撕裂（未驗證）。會騙人的替代方案（FPS 60 但畫面全白，截圖才看得出來）：
  QSG_RHI_BACKEND=opengl、QT_D3D_NO_FLIP=1（舊式交換鏈）。

環境變數 AEROPLAN_QT_SWAP_INTERVAL：
  未設   ⇒ Windows 用 0，其他平台不動 Qt 預設
  整數   ⇒ 用這個值（1 = 回到舊行為，對照用）
  default／空字串以外的非整數 ⇒ 不動 Qt 預設
必須在 QApplication 建立「之前」呼叫。
"""
from __future__ import annotations

import os
import sys

ENV_SWAP_INTERVAL = 'AEROPLAN_QT_SWAP_INTERVAL'


def desired_swap_interval() -> int | None:
    """依平台與環境變數決定要設的 swapInterval；None = 不動 Qt 預設。"""
    raw = os.environ.get(ENV_SWAP_INTERVAL)
    if raw is None:
        return 0 if sys.platform == 'win32' else None
    raw = raw.strip()
    try:
        v = int(raw)
    except ValueError:
        return None
    return v if v >= 0 else None


def configure_qt_presentation(logger=None) -> int | None:
    """把 QSurfaceFormat 預設 swapInterval 設成 desired_swap_interval()。回傳實際設定值（None = 沒動）。"""
    v = desired_swap_interval()
    if v is None:
        if logger:
            logger.info('[Render] Qt swapInterval 維持 Qt 預設')
        return None
    try:
        from PyQt6.QtGui import QSurfaceFormat
        fmt = QSurfaceFormat.defaultFormat()
        fmt.setSwapInterval(v)
        QSurfaceFormat.setDefaultFormat(fmt)
    except Exception as e:                                      # noqa: BLE001
        if logger:
            logger.warning(f'[Render] 設定 Qt swapInterval 失敗：{e}')
        return None
    if logger:
        logger.info(f'[Render] Qt swapInterval = {v}'
                    + ('（Qt 端不等 vsync，3D/2D 地圖出圖節奏交給 Chromium 60 Hz）' if v == 0 else ''))
    return v
