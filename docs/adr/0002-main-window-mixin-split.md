# ADR-0002: MainWindow 拆分策略：Mixin vs Composition

**狀態**: Accepted
**日期**: 2026-04

## Context

`ui/main_window.py` 達 **5,731 行**，包含：
- UI 組裝（menu / toolbar / status bar）
- DCCPP handlers (~800 行)
- SITL handlers (~1200 行)
- Swarm Strike handlers (~1200 行)
- VTOL handlers (~300 行)
- ReconToStrike handlers (~200 行)
- 基礎路徑規劃 (~800 行)

典型的 God Class。新增功能或 debug 時都要 scroll 過幾千行。

## Decision

**採用 Python 多重繼承 (Mixin Pattern)**：將邏輯依領域拆成 Mixin classes：

```python
# ui/controllers/strike_controller.py
class StrikeControllerMixin:
    """所有 _on_strike_* 方法"""
    def _on_strike_execute(self, params): ...
    # ... 16 methods ...

# ui/main_window.py
from ui.controllers import StrikeControllerMixin

class MainWindow(QMainWindow, StrikeControllerMixin):
    # 原本 5731 行 → 4555 行
    # _on_strike_* 透過 MRO 自動取得
    ...
```

Mixin 不持有狀態（不定義 `__init__`），所有 `self.*` 屬性由 MainWindow 提供。

## Alternatives Considered

### A. Composition (組合)
```python
class MainWindow:
    def __init__(self):
        self.strike_ctrl = StrikeController(self)  # 傳 self 進去
```
- ❌ 需要改寫所有 signal.connect(self._on_strike_X) 為 self.strike_ctrl._on_X
- ❌ self.* 屬性存取需要 proxy（strike_ctrl.main.xxx）
- ❌ 改動點爆炸（30+ 個 signal connection）

### B. 把 handlers 改成 free functions
- ❌ Qt signal/slot 要求 methods，重構成本高
- ❌ 失去 self 的便利

### C. 不拆，靠 region fold
- ❌ 規模繼續膨脹 → 最終不可維護

## Consequences

### Positive
- main_window.py -20% 行數
- Signal 連接點完全不動（向下相容 100%）
- 未來可再抽出 DCCPPControllerMixin / SITLControllerMixin
- Mixin 單檔可獨立 unit test（雖然需要 mock MainWindow）

### Negative
- Python MRO 需要一點學習：`class MainWindow(QMainWindow, MixinA, MixinB)`
  時 MRO 會把 Qt 體系放前面，Mixin 放後面
- Mixin 依賴 MainWindow 的 private 屬性（`self._strike_result`），
  耦合度沒完全解除

### Neutral
- 若未來需要嚴格 DI（依賴注入），可以再遷到 Composition 方案

## 後續計畫

依優先序逐步抽出：
1. ✅ StrikeControllerMixin（已完成）
2. ⏳ SITLControllerMixin
3. ⏳ DCCPPControllerMixin
