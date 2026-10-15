"""樹莓派 5 相機模組擷取服務（aeroplan-pi-session/1）。

跑在每架機的相機模組上，**不在 App 的 import 路徑裡**；部署方式見同目錄 README.md。
子模組：
- session    會期格式 F1 的常數、session.json 欄位、fsync 寫檔、時鐘取樣、離線驗證
- capture    邊緣→挑幀→寫檔的擷取服務、起飛前自檢、picamera2／libgpiod 實機後端
- imu        原始 IMU（ICM-42688-P / SPI）紀錄 imu.csv
- f9p_config ZED-F9P 的 UBX-CFG-VALSET 產生器與 rover.ubx 原始紀錄器（第二階段 PPK）

本檔刻意不 re-export 任何名稱：測試的突變覆蓋層是以子模組為單位替換的。
"""

__version__ = "0.1.0"
