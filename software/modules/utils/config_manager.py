import json
import yaml
from pathlib import Path
from typing import Dict, Any, Optional, Union
from dataclasses import dataclass, field
import logging

logger = logging.getLogger(__name__)


@dataclass
class CommunicationConfig:
    port: str = "COM10"
    baudrate: int = 9600
    timeout: float = 1.0
    retry_count: int = 3


@dataclass
class SensorConfig:
    lidar_max_range: float = 3.0
    lidar_min_range: float = 0.1
    lidar_quality_threshold: int = 10
    imu_weight: float = 0.7
    encoder_weight: float = 0.3
    wheel_base: float = 0.165


@dataclass
class MappingConfig:
    map_bounds: tuple = (-5, -5, 5, 5)
    map_resolution: float = 0.03
    l_free: float = -0.4
    l_occ: float = 0.85
    l_min: float = -4.0
    l_max: float = 4.0
    hit_margin: float = 1e-3


@dataclass
class NavigationConfig:
    robot_radius: float = 0.15
    lookahead_distance: float = 0.3
    max_linear_vel: float = 0.3
    max_angular_vel: float = 2.0
    k_angular: float = 1.8
    k_avoid: float = 1.2
    avoid_radius: float = 0.35
    goal_tolerance: float = 0.1


@dataclass
class VisualizationConfig:
    figsize: tuple = (16, 12)
    layout: str = "2x2"
    update_interval: float = 0.1
    save_images: bool = False
    image_dir: str = "images"


@dataclass
class SystemConfig:
    communication: CommunicationConfig = field(default_factory=CommunicationConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    log_level: str = "INFO"
    data_logging: bool = True
    log_dir: str = "logs"
    max_log_size: int = 100


class ConfigManager:
    
    def __init__(self, config_file: Optional[str] = None):
        self.config_file = config_file or "config.json"
        self.config = SystemConfig()
        
        if Path(self.config_file).exists():
            self.load_config()
        else:
            self.save_config()
        
        logger.info(f"配置管理器已初始化: {self.config_file}")
    
    def load_config(self, config_file: Optional[str] = None) -> bool:
        file_path = config_file or self.config_file
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                if file_path.endswith('.yaml') or file_path.endswith('.yml'):
                    data = yaml.safe_load(f)
                else:
                    data = json.load(f)
            
            self._update_config_from_dict(data)
            
            logger.info(f"配置已加载: {file_path}")
            return True
            
        except Exception as e:
            logger.error(f"加载配置失败: {e}")
            return False
    
    def save_config(self, config_file: Optional[str] = None) -> bool:
        file_path = config_file or self.config_file
        
        try:
            data = self._config_to_dict()
            
            with open(file_path, 'w', encoding='utf-8') as f:
                if file_path.endswith('.yaml') or file_path.endswith('.yml'):
                    yaml.dump(data, f, default_flow_style=False, indent=2)
                else:
                    json.dump(data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"配置已保存: {file_path}")
            return True
            
        except Exception as e:
            logger.error(f"保存配置失败: {e}")
            return False
    
    def get_config(self) -> SystemConfig:
        return self.config
    
    def update_config(self, section: str, key: str, value: Any) -> bool:
        try:
            if hasattr(self.config, section):
                section_obj = getattr(self.config, section)
                if hasattr(section_obj, key):
                    setattr(section_obj, key, value)
                    logger.info(f"配置已更新: {section}.{key} = {value}")
                    return True
                else:
                    logger.error(f"配置键不存在: {section}.{key}")
                    return False
            else:
                logger.error(f"配置节不存在: {section}")
                return False
                
        except Exception as e:
            logger.error(f"更新配置失败: {e}")
            return False
    
    def get_config_value(self, section: str, key: str, default: Any = None) -> Any:
        try:
            if hasattr(self.config, section):
                section_obj = getattr(self.config, section)
                if hasattr(section_obj, key):
                    return getattr(section_obj, key)
            
            return default
            
        except Exception as e:
            logger.error(f"获取配置值失败: {e}")
            return default
    
    def reset_to_default(self):
        self.config = SystemConfig()
        logger.info("配置已重置为默认值")
    
    def _update_config_from_dict(self, data: Dict[str, Any]):
        if 'communication' in data:
            comm_data = data['communication']
            for key, value in comm_data.items():
                if hasattr(self.config.communication, key):
                    setattr(self.config.communication, key, value)
        
        if 'sensor' in data:
            sensor_data = data['sensor']
            for key, value in sensor_data.items():
                if hasattr(self.config.sensor, key):
                    setattr(self.config.sensor, key, value)
        
        if 'mapping' in data:
            mapping_data = data['mapping']
            for key, value in mapping_data.items():
                if hasattr(self.config.mapping, key):
                    setattr(self.config.mapping, key, value)
        
        if 'navigation' in data:
            nav_data = data['navigation']
            for key, value in nav_data.items():
                if hasattr(self.config.navigation, key):
                    setattr(self.config.navigation, key, value)
        
        if 'visualization' in data:
            viz_data = data['visualization']
            for key, value in viz_data.items():
                if hasattr(self.config.visualization, key):
                    setattr(self.config.visualization, key, value)
        
        for key, value in data.items():
            if key not in ['communication', 'sensor', 'mapping', 'navigation', 'visualization']:
                if hasattr(self.config, key):
                    setattr(self.config, key, value)
    
    def _config_to_dict(self) -> Dict[str, Any]:
        return {
            'communication': {
                'port': self.config.communication.port,
                'baudrate': self.config.communication.baudrate,
                'timeout': self.config.communication.timeout,
                'retry_count': self.config.communication.retry_count
            },
            'sensor': {
                'lidar_max_range': self.config.sensor.lidar_max_range,
                'lidar_min_range': self.config.sensor.lidar_min_range,
                'lidar_quality_threshold': self.config.sensor.lidar_quality_threshold,
                'imu_weight': self.config.sensor.imu_weight,
                'encoder_weight': self.config.sensor.encoder_weight,
                'wheel_base': self.config.sensor.wheel_base
            },
            'mapping': {
                'map_bounds': self.config.mapping.map_bounds,
                'map_resolution': self.config.mapping.map_resolution,
                'l_free': self.config.mapping.l_free,
                'l_occ': self.config.mapping.l_occ,
                'l_min': self.config.mapping.l_min,
                'l_max': self.config.mapping.l_max,
                'hit_margin': self.config.mapping.hit_margin
            },
            'navigation': {
                'robot_radius': self.config.navigation.robot_radius,
                'lookahead_distance': self.config.navigation.lookahead_distance,
                'max_linear_vel': self.config.navigation.max_linear_vel,
                'max_angular_vel': self.config.navigation.max_angular_vel,
                'k_angular': self.config.navigation.k_angular,
                'k_avoid': self.config.navigation.k_avoid,
                'avoid_radius': self.config.navigation.avoid_radius,
                'goal_tolerance': self.config.navigation.goal_tolerance
            },
            'visualization': {
                'figsize': self.config.visualization.figsize,
                'layout': self.config.visualization.layout,
                'update_interval': self.config.visualization.update_interval,
                'save_images': self.config.visualization.save_images,
                'image_dir': self.config.visualization.image_dir
            },
            'log_level': self.config.log_level,
            'data_logging': self.config.data_logging,
            'log_dir': self.config.log_dir,
            'max_log_size': self.config.max_log_size
        }
