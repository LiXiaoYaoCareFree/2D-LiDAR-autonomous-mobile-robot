import csv
import json
import time
from pathlib import Path
from typing import Dict, Any, List, Optional
import logging

from ..sensors.data_structures import Pose, Scan

logger = logging.getLogger(__name__)


class DataLogger:
    
    def __init__(self, log_dir: str = "logs", max_file_size: int = 100):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        
        self.max_file_size = max_file_size * 1024 * 1024
        
        self.pose_log_file = self.log_dir / "pose.csv"
        self.scan_log_file = self.log_dir / "scan.csv"
        self.system_log_file = self.log_dir / "system.json"
        
        self._init_log_files()
        
        logger.info(f"数据记录器已初始化: {self.log_dir}")
    
    def _init_log_files(self):
        if not self.pose_log_file.exists():
            with open(self.pose_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'theta', 'mode'])
        
        if not self.scan_log_file.exists():
            with open(self.scan_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'num_points', 'angles', 'ranges'])
        
        if not self.system_log_file.exists():
            with open(self.system_log_file, 'w') as f:
                json.dump([], f)
    
    def log_pose(self, pose: Pose, mode: str = "AUTO"):
        try:
            with open(self.pose_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(),
                    pose.x,
                    pose.y,
                    pose.theta,
                    mode
                ])
        except Exception as e:
            logger.error(f"记录位姿数据失败: {e}")
    
    def log_scan(self, scan: Scan):
        try:
            with open(self.scan_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(),
                    len(scan.angles),
                    json.dumps(scan.angles),
                    json.dumps(scan.ranges)
                ])
        except Exception as e:
            logger.error(f"记录扫描数据失败: {e}")
    
    def log_system_data(self, data: Dict[str, Any]):
        try:
            if self.system_log_file.exists():
                with open(self.system_log_file, 'r') as f:
                    system_data = json.load(f)
            else:
                system_data = []
            
            data['timestamp'] = time.time()
            system_data.append(data)
            
            with open(self.system_log_file, 'w') as f:
                json.dump(system_data, f, indent=2)
                
        except Exception as e:
            logger.error(f"记录系统数据失败: {e}")
    
    def log_navigation_data(self, pose: Pose, scan: Scan, 
                          control_output: Dict[str, Any],
                          navigation_state: Dict[str, Any]):
        self.log_pose(pose, navigation_state.get('mode', 'AUTO'))
        self.log_scan(scan)
        system_data = {
            'pose': {
                'x': pose.x,
                'y': pose.y,
                'theta': pose.theta
            },
            'control_output': control_output,
            'navigation_state': navigation_state,
            'scan_points': len(scan.angles) if scan.angles else 0
        }
        
        self.log_system_data(system_data)
    
    def get_pose_history(self, start_time: Optional[float] = None, 
                        end_time: Optional[float] = None) -> List[Dict[str, Any]]:
        try:
            poses = []
            with open(self.pose_log_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    timestamp = float(row['timestamp'])
                    
                    if start_time and timestamp < start_time:
                        continue
                    if end_time and timestamp > end_time:
                        continue
                    
                    poses.append({
                        'timestamp': timestamp,
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'theta': float(row['theta']),
                        'mode': row['mode']
                    })
            
            return poses
            
        except Exception as e:
            logger.error(f"获取位姿历史失败: {e}")
            return []
    
    def get_scan_history(self, start_time: Optional[float] = None,
                        end_time: Optional[float] = None) -> List[Dict[str, Any]]:
        try:
            scans = []
            with open(self.scan_log_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    timestamp = float(row['timestamp'])
                    
                    if start_time and timestamp < start_time:
                        continue
                    if end_time and timestamp > end_time:
                        continue
                    
                    scans.append({
                        'timestamp': timestamp,
                        'num_points': int(row['num_points']),
                        'angles': json.loads(row['angles']),
                        'ranges': json.loads(row['ranges'])
                    })
            
            return scans
            
        except Exception as e:
            logger.error(f"获取扫描历史失败: {e}")
            return []
    
    def get_system_data(self, start_time: Optional[float] = None,
                       end_time: Optional[float] = None) -> List[Dict[str, Any]]:
        try:
            with open(self.system_log_file, 'r') as f:
                system_data = json.load(f)
            
            if start_time or end_time:
                filtered_data = []
                for data in system_data:
                    timestamp = data.get('timestamp', 0)
                    
                    if start_time and timestamp < start_time:
                        continue
                    if end_time and timestamp > end_time:
                        continue
                    
                    filtered_data.append(data)
                
                return filtered_data
            
            return system_data
            
        except Exception as e:
            logger.error(f"获取系统数据失败: {e}")
            return []
    
    def clear_logs(self):
        try:
            with open(self.pose_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'theta', 'mode'])
            
            with open(self.scan_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'num_points', 'angles', 'ranges'])
            
            with open(self.system_log_file, 'w') as f:
                json.dump([], f)
            
            logger.info("日志文件已清空")
            
        except Exception as e:
            logger.error(f"清空日志失败: {e}")
    
    def export_data(self, output_file: str, start_time: Optional[float] = None,
                   end_time: Optional[float] = None):
        try:
            export_data = {
                'poses': self.get_pose_history(start_time, end_time),
                'scans': self.get_scan_history(start_time, end_time),
                'system_data': self.get_system_data(start_time, end_time),
                'export_time': time.time(),
                'time_range': {
                    'start': start_time,
                    'end': end_time
                }
            }
            
            with open(output_file, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            logger.info(f"数据已导出: {output_file}")
            
        except Exception as e:
            logger.error(f"导出数据失败: {e}")
    
    def get_log_statistics(self) -> Dict[str, Any]:
        try:
            stats = {
                'pose_records': 0,
                'scan_records': 0,
                'system_records': 0,
                'log_size': 0,
                'oldest_record': None,
                'newest_record': None
            }
            
            if self.pose_log_file.exists():
                with open(self.pose_log_file, 'r') as f:
                    reader = csv.DictReader(f)
                    poses = list(reader)
                    stats['pose_records'] = len(poses)
                    
                    if poses:
                        timestamps = [float(p['timestamp']) for p in poses]
                        stats['oldest_record'] = min(timestamps)
                        stats['newest_record'] = max(timestamps)
            
            if self.scan_log_file.exists():
                with open(self.scan_log_file, 'r') as f:
                    reader = csv.DictReader(f)
                    scans = list(reader)
                    stats['scan_records'] = len(scans)
            
            if self.system_log_file.exists():
                with open(self.system_log_file, 'r') as f:
                    system_data = json.load(f)
                    stats['system_records'] = len(system_data)
            
            for log_file in [self.pose_log_file, self.scan_log_file, self.system_log_file]:
                if log_file.exists():
                    stats['log_size'] += log_file.stat().st_size
            
            return stats
            
        except Exception as e:
            logger.error(f"获取日志统计失败: {e}")
            return {}
