# Architecture Decision Records (ADR)

本資料夾記錄 AeroPlan Studio 的關鍵架構決策。

## 什麼是 ADR？

ADR (Architecture Decision Record) 是一份簡短文件，說明：
- **Context** — 為何需要做這個決策？
- **Decision** — 我們決定怎麼做？
- **Consequences** — 這個決策的影響（好的壞的都寫）

每當做出會影響長期架構的重大決定時，新增一份 ADR。
日後有人（包含未來的你）問「為什麼這樣做？」就有文件可查。

## 索引

| # | 標題 | 狀態 |
|---|---|---|
| [0001](0001-strike-planner-family.md) | 為何有 6 個 Strike Planner？ | Accepted |
| [0002](0002-main-window-mixin-split.md) | MainWindow 拆分策略：Mixin vs Composition | Accepted |
| [0003](0003-dubins-lru-cache.md) | Dubins 路徑計算的 LRO cache 策略 | Accepted |
| [0004](0004-stot-dtot-dual-definition.md) | STOT / DTOT 為何有兩組不同定義？ | Accepted |
| [0005](0005-owa-safety-lockout.md) | OWA-UAV 禁 RTL 參數的安全機制 | Accepted |
| [0006](0006-delay-aware-iapf.md) | 延遲感知改進型 APF 設計 | Accepted |

## 模板

建立新 ADR 時請拷貝 [template.md](template.md)。
