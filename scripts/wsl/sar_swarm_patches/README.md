# sar_swarm 節點修改參考副本

這些是 WSL `~/fixwing_ws/src/sar_swarm/` 內經本專案修改並實測驗證的節點檔
**參考副本**(部署地在 WSL,非本 repo 執行路徑)。放這裡是為了讓里程碑
自足、可追溯。部署方式:

```bash
cp master_role.py master_node.py wingman_node.py backup_node.py \
   geodesy.py mavros_interface.py  ~/fixwing_ws/src/sar_swarm/sar_swarm/
cp sar_failover_nodes.launch.py    ~/fixwing_ws/src/sar_swarm/launch/
cd ~/fixwing_ws && colcon build --packages-select sar_swarm --symlink-install
```

## 本專案在此驗證/實裝的內容(全部 WSL SITL 真飛)

| 檔 | 修改 |
|---|---|
| `mavros_interface.py` | 導引改 `MAV_CMD_DO_REPOSITION`(舊 MISSION_ITEM_INT 不吃高度,長機爬 1000m) |
| `geodesy.py` | `speed_for_error` 斜坡從 err=0 起算(舊版誤差漸近卡門檻);`advance_along_heading` slot 前饋 |
| `master_node.py` | 前饋(真 slot 誤差分離)、`formed_hysteresis`、`patrol_box_m`、自我降權(防腦裂)、**§4.5 弧轉程序**(轉前減速+定大半徑 fillet 弧)、`turn_rate` 廣播 |
| `master_role.py` | `suspend_who`(降權保心跳)、`DEEP_SUB_QOS`(多發布者 depth-8)、`omega` 廣播 |
| `backup_node.py` | 接管資格閘(啟動寬限+佐證閘)、讓權回收、代理長機手風琴、旋轉前饋 |
| `wingman_node.py` | slot 前饋(真 slot 誤差分離)、**旋轉前饋**(弧轉沿弧外推) |
| `sar_failover_nodes.launch.py` | 全部新參數 + `timeout` 2.5→6.0(騎過 master 週期性卡頓,待修) |

實測結論見 `docs/formation_v6_tight_coupled_spec.md` 附錄 A + 記憶檔。
耦合驗證(點雲):S=60 不耦合 → S=300+完全§4.5 RMS 0.67-0.83×slot距。
