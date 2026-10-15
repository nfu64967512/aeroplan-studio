# ADR-0009: HV-DCCPP 兩個構型共用同一台單光相機

> ⚠ 本文件寫於改名之前。文中的 **HV-DCCPP** 就是今日的 **HARVEST**（見 [ADR-0010](0010-harvest-naming.md)）。ADR 是有日期的紀錄，內文刻意不改寫。


**狀態**: Accepted
**日期**: 2026-09-07
**決策者**: @james zheng

## Context / 背景

HV-DCCPP 有兩個構型：Pioneer PNP（構型 A）與 HERO PNP（構型 B）。
上游函式庫的轉接層 `hv_dccpp/integration/aeroplan_adapter.py::_hardware_for`
在**呼叫端沒有明確給感測器**時，是**照構型字母**決定酬載種類：

```python
given = sensor_a if cls == "A" else sensor_b
if given: return tuple(given)
...
return (LidarHardware(),) if cls == "A" else (CameraHardware(),)
```

AeroPlan 的 `mission/swarm_coordinator.py` HV 分支原本沒有傳 `sensor_a` / `sensor_b`，
所以走的正是這條預設：**A 被當成 LiDAR、B 被當成相機**。實測 `ctx.sensors`：

```
{('A', 'default'): 'lidar', ('B', 'default_rgb'): 'rgb'}   需求 2 個
```

這帶來三個問題：

1. **分工變成酬載決定的。** 一塊作業區的 `sensor_req` 一旦是 lidar，
   HERO 就完全不可能拿到它，跟它的續航與速度沒有關係。
2. **兩台硬體都是 `is_default_example=True` 的範例規格**，不是這個專案量到的相機。
3. 手冊 §10.5 的坑：某構型找不到對應 kind 的 `PayloadRequirement` 時
   `context.build_sensors` 就不替它建 `Sensor`，該構型整型分不到工作，
   **而且完全不會報錯**。異構酬載讓這個坑一直開著。

「VTOL 偵查」這一頁本來就只設定**一台**相機（HFOV／VFOV／目標 GSD），
畫面上從來沒有第二種酬載可以選。

## Decision / 決策

兩個構型掛**同一台單光相機**，`kind` 一律 `'rgb'`。

1. `mission/swarm_coordinator.py` 新增 `_uniform_camera(hfov_deg, vfov_deg)`：
   由面板 FOV 反推針孔幾何 `sensor_width = 2·f·tan(HFOV/2)`，
   使規劃器的刈幅與面板算出來的足跡 `2·h·tan(HFOV/2)` **完全相同**；
   焦距與像素間距沿用 `CameraHardware` 預設（GSD 由它們決定）；
   `is_default_example=False`。
2. HV 分支把它同時傳給 `sensor_a` 與 `sensor_b`。
3. 開關是 `SwarmCoordinator.hv_uniform_sensor`，**預設 True**。
   要回到異構酬載就把它設成 False（面板沒有這個選項，是程式層的逃生口）。

於是分工只剩**平台性能**：

| | Pioneer PNP | HERO PNP |
|---|---|---|
| 巡航空速 | 21.5 m/s | 19.0 m/s |
| 規劃轉彎半徑 | 200 m | 120 m |
| 最低固定翼作業高度 | 120 m AGL | 100 m AGL |
| 續航 | 176 min | 136 min |
| 可用航程 | 133 km | 79 km |
| 酬載上限 | 5.0 kg | 1.0 kg |

這些量進到 `hv_dccpp/allocate.py::assign_config_and_count` 的評分
`score = S / T̃`、`T̃ = (t_in + T_single + t_RTB) / T_use`：續航是 `T_use`、
速度與轉彎半徑進 `T_single` 與 `t_in`／`t_RTB`。台數下限
`λ_floor = ⌈T_single / (κ·T_use − t_in − t_RTB)⌉` 也是續航直接決定的。

## Consequences / 後果

**量到的（`tests/test_hv_uniform_sensor.py`，四機兩區實跑規劃）**

```
uniform=True   ctx.sensors {('A','default'):'rgb', ('B','default'):'rgb'}   需求 1 個
uniform=False  ctx.sensors {('A','default'):'lidar', ('B','default_rgb'):'rgb'}  需求 2 個
```

* 兩型的條帶間距變成同一個數 ⇒ 幾何不再是分工依據。
* 每個構型都拿得到 `Sensor` 也都真的分到條帶，§10.5 的坑關上了。
* LiDAR 的寬刈幅消失，所有機都用相機刈幅 ⇒ 條帶變多、makespan 變長。
  四區情境實測 2161 s → 3198 s。**這不是退步，是把假的優勢拿掉。**

**⚠ 沒有處理的**

* 面板的 `mounting_angle_deg` **沒有**進到足跡模型。兩邊基準不同：
  面板是「離天底的傾角」（0 = 正下方），`TrapezoidFovSpec` 是「離水平的俯角」
  （90 = 正下方）。直接送過去會得到錯的足跡，所以改成以正下方建模，
  傾角非零時發 warning 而不是靜靜換算。
* 機隊用到幾台，是**胞的數目**決定的，不是機隊大小。實測同一組四機：
  一個作業區 → 3 胞 → 只有 2 台有工作；四個作業區 → 8 胞 → 四台都有。
  想讓全隊上工就得把作業區切開，或等 `_partition_rec` 改成看機隊大小。
