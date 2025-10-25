from dataclasses import dataclass
from typing import List, Optional, Tuple
import time


@dataclass
class Pose:
    x: float
    y: float
    theta: float  
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)


@dataclass
class Scan:
    angles: List[float]  
    ranges: List[float]  
    timestamp: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()
    
    def get_points(self) -> List[Tuple[float, float]]:
        points = []
        for angle, range_val in zip(self.angles, self.ranges):
            if range_val > 0:
                x = range_val * __import__('math').cos(angle)
                y = range_val * __import__('math').sin(angle)
                points.append((x, y))
        return points


@dataclass
class SensorData:
    pose: Optional[Pose] = None
    scan: Optional[Scan] = None
    imu_data: Optional[dict] = None
    encoder_data: Optional[dict] = None
    timestamp: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()
    
    def is_valid(self) -> bool:
        return (self.pose is not None and 
                self.scan is not None and 
                len(self.scan.angles) > 0)
