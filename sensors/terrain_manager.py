from abc import ABC, abstractmethod
import math
import logging

# 設定 Logger
logger = logging.getLogger(__name__)

class TerrainManagerBase(ABC):
    """地形管理抽象基類 (Interface)"""
    
    @abstractmethod
    def get_elevation(self, lat: float, lon: float) -> float:
        """獲取特定座標的海拔高度 (AMSL)"""
        pass

    @abstractmethod
    def check_collision(self, lat: float, lon: float, alt_amsl: float) -> bool:
        """檢查該點是否會撞地"""
        pass

class SimpleTerrainManager(TerrainManagerBase):
    """
    簡易地形管理器
    預設為平坦地面 (0m) 或從緩存讀取
    """
    def __init__(self, default_elevation=0.0):
        self.default_elevation = default_elevation
        self.cache = {} # 簡單的緩存機制
        logger.info(f"SimpleTerrainManager initialized with base alt: {default_elevation}")

    def get_elevation(self, lat: float, lon: float) -> float:
        """
        模擬 API 調用。在實際專案中，這裡會連接 SRTM 數據庫或 GDAL 庫。
        """
        # 為了模擬真實情況，這裡將座標 key 簡化以作為緩存鍵
        key = (round(lat, 5), round(lon, 5))
        if key in self.cache:
            return self.cache[key]
        
        # TODO: 替換為真實的 API 呼叫或 GeoTIFF 讀取
        # 這裡僅返回預設值
        return self.default_elevation

    def set_region_data(self, data_grid):
        """
        API 接口：用於預加載某個區域的地形數據
        """
        pass

    def check_collision(self, lat: float, lon: float, alt_amsl: float, buffer_m=5.0) -> bool:
        """
        檢查高度是否低於地形 + 安全緩衝區
        """
        terrain_alt = self.get_elevation(lat, lon)
        if alt_amsl < (terrain_alt + buffer_m):
            logger.warning(f"Collision risk at {lat}, {lon}: Alt {alt_amsl} < Terrain {terrain_alt} + Buffer {buffer_m}")
            return True
        return False