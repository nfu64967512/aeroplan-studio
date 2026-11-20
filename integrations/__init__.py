"""integrations — AeroPlan 與外部執行環境之間的橋接層。

⚠️ 這個套件的 ``__init__`` **刻意不 import 任何東西**。

理由:底下的模組(例如 ``stot_ros2_bridge``)必須能在 **WSL 的 python3** 裡
被單元測試 import,而那邊沒有 PyQt6 / pyproj / shapely。`core/__init__.py` 與
`mission/__init__.py` 都在套件層 import 了重量級相依,任何放進去的模組都會被
連坐。所以純邏輯層獨立成這個零相依套件,Qt 那一層(ui/controllers/)再去 import
它 —— 單向相依,不會反過來。
"""
