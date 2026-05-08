# ADR-0003: Dubins 路徑計算的 LRU cache 策略

**狀態**: Accepted
**日期**: 2026-04

## Context

`dubins_shortest_length()` 是 STOT/DTOT、IAPF、RTL 等多處共用的熱點函式。
實測 10 機群飛規劃需呼叫 **~500 次**，每次約 0.3-0.5 ms 純 Python 運算。

觀察：許多呼叫參數高度相似（例如同一 UAV 對多個候選 IP 重複試算），
但浮點參數直接當 cache key 會命中率極低（浮點誤差）。

## Decision

採用**量化 key + LRU cache** 兩層設計：

```python
@lru_cache(maxsize=2048)
def _dubins_cached(s_lat_q, s_lon_q, s_hdg_q,
                   e_lat_q, e_lon_q, e_hdg_q, r_q) -> float:
    return _dubins_impl(s_lat_q/1e6, ..., r_q/100.0)

def dubins_shortest_length(s_lat, s_lon, s_hdg, ...):
    # 量化：經緯度到 1e-6° (≈0.1m)，航向到 0.1°，半徑到 1cm
    return _dubins_cached(
        int(round(s_lat * 1e6)),
        int(round(s_hdg * 10.0)),
        ...
    )
```

### 量化粒度選擇
- **經緯度 1e-6°** ≈ 0.1m：小於 GPS 精度，無視覺差別
- **航向 0.1°**：遠小於 IMU 誤差（通常 ±1°）
- **半徑 1cm**：遠小於飛控可跟隨精度

## Alternatives Considered

### A. 不 cache
- ❌ 大規模群飛 (>20 機) 會明顯卡

### B. Redis / Memcached
- ❌ 跨進程過度設計，增加部署複雜度
- ❌ 網路延遲反而比 Python 運算慢

### C. Precompute matrix (N×M)
- ❌ 參數組合爆炸
- ❌ 多變參數（heading 連續值）無法窮舉

### D. 直接浮點 key LRU
- ❌ 命中率極低（實測 < 5%）

## Consequences

### Positive
- 實測 1000 次 random-order 呼叫：3.5ms、**63% hit rate**
- 自動適應熱點 pattern（使用者常用的 UAV/目標組合）
- 對正確性影響：量化誤差 < 0.1m（flight controller 容差內）

### Negative
- cache 最多 2048 entries，極端場景 (100+ UAV × 10+ targets) 可能每次 miss
- 若未來引入移動目標（每 tick 座標微變），cache 會失效

### Neutral
- `clear_dubins_cache()` API 已暴露，可在需要時手動清除
- `dubins_cache_info()` 提供統計，方便調優 maxsize
