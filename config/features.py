# -*- coding: utf-8 -*-
"""功能開關（單一來源）。

目前只有一個：蜂群攻擊這一系（投放／彈種／STOT 閘門／打擊 BDA／嵌入式蜂群／swarm_strike 橋接）。
使用者 2026-10-17 定調：**先從 UI 上拿掉、相關管線先斷開**，程式碼留著以後再接回來。

規則：
* 預設 `STRIKE_ENABLED = False`。要暫時打開只能靠環境變數 `AEROPLAN_STRIKE_UI=1`
  （不寫進 settings.json —— 這不是使用者設定，是開發期的開關）。
* 關閉時：HUD 不放投放列與「嵌入式蜂群」鈕、主視窗不接投放／STOT／打擊 BDA／嵌入式蜂群的信號、
  不建相關分頁與選單項目；地圖上的 TOT 閘門／落點動畫不會被觸發（沒人呼叫）。
* 相關 widget **仍然會被建構**但不放進版面（屬性名維持，既有測試照跑）；只有「接線」與「進版面」看這個開關。
"""
from __future__ import annotations

import os

__all__ = ['STRIKE_ENABLED', 'strike_enabled']


def _env_truthy(name: str) -> bool:
    v = os.environ.get(name, '')
    return v.strip().lower() in ('1', 'true', 'yes', 'on')


#: 蜂群攻擊 UI 與管線（投放／STOT／BDA／嵌入式蜂群）。預設關；`AEROPLAN_STRIKE_UI=1` 暫時開。
STRIKE_ENABLED: bool = _env_truthy('AEROPLAN_STRIKE_UI')


def strike_enabled() -> bool:
    """即時讀（測試可用 monkeypatch 改模組常數，或設環境變數後重新 import）。"""
    return bool(STRIKE_ENABLED)
