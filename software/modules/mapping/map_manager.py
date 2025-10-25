import numpy as np
import json
import time
from typing import Dict, List, Tuple, Optional, Any
from pathlib import Path
import logging

from .occupancy_grid import OccupancyGridMap

logger = logging.getLogger(__name__)


class MapManager:
    
    def __init__(self, map_dir: str = "maps"):
        self.map_dir = Path(map_dir)
        self.map_dir.mkdir(exist_ok=True)
        
        self.current_map: Optional[OccupancyGridMap] = None
        self.map_metadata: Dict[str, Any] = {}
        
        logger.info(f"地图管理器已初始化，地图目录: {self.map_dir}")
    
    def create_new_map(self, bounds: Tuple[float, float, float, float] = (-5, -5, 5, 5),
                      resolution: float = 0.03) -> OccupancyGridMap:
        self.current_map = OccupancyGridMap(*bounds, resolution=resolution)
        
        self.map_metadata = {
            'created_time': time.time(),
            'bounds': bounds,
            'resolution': resolution,
            'version': '1.0'
        }
        
        logger.info(f"创建新地图: 边界={bounds}, 分辨率={resolution}")
        return self.current_map
    
    def save_map(self, map_name: str, description: str = "") -> bool:
        if self.current_map is None:
            logger.error("没有当前地图可保存")
            return False
        
        try:
            map_file = self.map_dir / f"{map_name}.npy"
            self.current_map.save_map(str(map_file))
            
            metadata = self.map_metadata.copy()
            metadata.update({
                'map_name': map_name,
                'description': description,
                'saved_time': time.time(),
                'grid_shape': self.current_map.grid.shape
            })
            
            metadata_file = self.map_dir / f"{map_name}_metadata.json"
            with open(metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            logger.info(f"地图已保存: {map_name}")
            return True
            
        except Exception as e:
            logger.error(f"保存地图失败: {e}")
            return False
    
    def load_map(self, map_name: str) -> Optional[OccupancyGridMap]:
        try:
            map_file = self.map_dir / f"{map_name}.npy"
            metadata_file = self.map_dir / f"{map_name}_metadata.json"
            
            if not map_file.exists():
                logger.error(f"地图文件不存在: {map_file}")
                return None
            
            if metadata_file.exists():
                with open(metadata_file, 'r') as f:
                    self.map_metadata = json.load(f)
            else:
                logger.warning(f"元数据文件不存在: {metadata_file}")
                self.map_metadata = {}
            
            bounds = self.map_metadata.get('bounds', (-5, -5, 5, 5))
            resolution = self.map_metadata.get('resolution', 0.03)
            
            self.current_map = OccupancyGridMap(*bounds, resolution=resolution)
            self.current_map.load_map(str(map_file))
            
            logger.info(f"地图已加载: {map_name}")
            return self.current_map
            
        except Exception as e:
            logger.error(f"加载地图失败: {e}")
            return None
    
    def list_maps(self) -> List[Dict[str, Any]]:
        maps = []
        
        for map_file in self.map_dir.glob("*.npy"):
            map_name = map_file.stem
            metadata_file = self.map_dir / f"{map_name}_metadata.json"
            
            map_info = {
                'name': map_name,
                'file': str(map_file),
                'exists': True
            }
            
            if metadata_file.exists():
                try:
                    with open(metadata_file, 'r') as f:
                        metadata = json.load(f)
                        map_info.update(metadata)
                except Exception as e:
                    logger.warning(f"读取元数据失败: {e}")
            
            maps.append(map_info)
        
        return maps
    
    def delete_map(self, map_name: str) -> bool:
        try:
            map_file = self.map_dir / f"{map_name}.npy"
            metadata_file = self.map_dir / f"{map_name}_metadata.json"
            
            if map_file.exists():
                map_file.unlink()
            
            if metadata_file.exists():
                metadata_file.unlink()
            
            logger.info(f"地图已删除: {map_name}")
            return True
            
        except Exception as e:
            logger.error(f"删除地图失败: {e}")
            return False
    
    def merge_maps(self, map_names: List[str], merged_name: str) -> Optional[OccupancyGridMap]:
        if not map_names:
            logger.error("没有指定要合并的地图")
            return None
        
        try:
            base_map = self.load_map(map_names[0])
            if base_map is None:
                return None
            
            for map_name in map_names[1:]:
                other_map = self.load_map(map_name)
                if other_map is None:
                    continue
                
                self._expand_map_bounds(base_map, other_map)
                
                self._merge_grid_data(base_map, other_map)
            
            self.current_map = base_map
            self.save_map(merged_name, f"合并地图: {', '.join(map_names)}")
            
            logger.info(f"地图合并完成: {merged_name}")
            return base_map
            
        except Exception as e:
            logger.error(f"合并地图失败: {e}")
            return None
    
    def _expand_map_bounds(self, base_map: OccupancyGridMap, other_map: OccupancyGridMap):
        base_bounds = base_map.get_map_bounds()
        other_bounds = other_map.get_map_bounds()
        
        new_bounds = (
            min(base_bounds[0], other_bounds[0]),
            min(base_bounds[1], other_bounds[1]),
            max(base_bounds[2], other_bounds[2]),
            max(base_bounds[3], other_bounds[3])
        )
        
        if new_bounds != base_bounds:
            base_map.resize_map(*new_bounds)
    
    def _merge_grid_data(self, base_map: OccupancyGridMap, other_map: OccupancyGridMap):
        pass
    
    def get_map_statistics(self, map_name: str) -> Optional[Dict[str, Any]]:
        map_obj = self.load_map(map_name)
        if map_obj is None:
            return None
        
        prob_map = map_obj.get_probability_map()
        
        stats = {
            'grid_size': map_obj.grid.shape,
            'resolution': map_obj.resolution,
            'bounds': map_obj.get_map_bounds(),
            'total_cells': map_obj.grid.size,
            'free_cells': np.sum(prob_map <= 0.35),
            'occupied_cells': np.sum(prob_map >= 0.65),
            'unknown_cells': np.sum((prob_map > 0.35) & (prob_map < 0.65)),
            'coverage_ratio': np.sum(prob_map != 0.5) / map_obj.grid.size
        }
        
        return stats
    
    def export_map_image(self, map_name: str, output_file: str) -> bool:
        try:
            import matplotlib.pyplot as plt
            
            map_obj = self.load_map(map_name)
            if map_obj is None:
                return False
            
            occ_map = map_obj.get_occupancy_map()
            
            plt.figure(figsize=(10, 10))
            plt.imshow(occ_map, cmap='gray', origin='lower')
            plt.colorbar(label='占用概率')
            plt.title(f'地图: {map_name}')
            plt.xlabel('X (栅格)')
            plt.ylabel('Y (栅格)')
            
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info(f"地图图像已导出: {output_file}")
            return True
            
        except Exception as e:
            logger.error(f"导出地图图像失败: {e}")
            return False
