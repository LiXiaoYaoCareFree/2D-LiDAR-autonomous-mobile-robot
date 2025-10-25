import math
import time
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import logging

from ..sensors.data_structures import Pose

logger = logging.getLogger(__name__)


@dataclass
class Goal:
    x: float
    y: float
    theta: Optional[float] = None
    tolerance: float = 0.1
    priority: int = 0
    created_time: float = 0.0
    
    def __post_init__(self):
        if self.created_time == 0.0:
            self.created_time = time.time()
    
    def distance_to(self, pose: Pose) -> float:
        return math.sqrt((self.x - pose.x)**2 + (self.y - pose.y)**2)
    
    def is_reached(self, pose: Pose) -> bool:
        distance = self.distance_to(pose)
        return distance <= self.tolerance


class GoalManager:
    
    def __init__(self, default_tolerance: float = 0.1, 
                 max_goals: int = 100,
                 timeout: float = 60.0):
        self.default_tolerance = default_tolerance
        self.max_goals = max_goals
        self.timeout = timeout
        
        self.goals: List[Goal] = []
        self.current_goal_index = 0
        self.navigation_state = "IDLE"  # IDLE, NAVIGATING, REACHED, FAILED
        self.start_time = None
        
        logger.info("目标管理器已初始化")
    
    def add_goal(self, x: float, y: float, theta: Optional[float] = None,
                 tolerance: Optional[float] = None, priority: int = 0) -> bool:
        
        if len(self.goals) >= self.max_goals:
            logger.warning("目标数量已达上限")
            return False
        
        goal = Goal(
            x=x, y=y, theta=theta,
            tolerance=tolerance or self.default_tolerance,
            priority=priority
        )
        
        inserted = False
        for i, existing_goal in enumerate(self.goals):
            if goal.priority > existing_goal.priority:
                self.goals.insert(i, goal)
                inserted = True
                break
        
        if not inserted:
            self.goals.append(goal)
        
        logger.info(f"添加目标: ({x:.2f}, {y:.2f}), 优先级: {priority}")
        return True
    
    def add_goals(self, goals: List[Tuple[float, float]], 
                  priorities: Optional[List[int]] = None) -> int:
        added_count = 0
        
        for i, (x, y) in enumerate(goals):
            priority = priorities[i] if priorities and i < len(priorities) else 0
            
            if self.add_goal(x, y, priority=priority):
                added_count += 1
        
        logger.info(f"批量添加目标: {added_count}/{len(goals)}")
        return added_count
    
    def get_current_goal(self) -> Optional[Goal]:
        if not self.goals or self.current_goal_index >= len(self.goals):
            return None
        return self.goals[self.current_goal_index]
    
    def update_navigation(self, current_pose: Pose) -> Dict[str, Any]:
        current_goal = self.get_current_goal()
        
        if current_goal is None:
            self.navigation_state = "IDLE"
            return {
                'state': self.navigation_state,
                'current_goal': None,
                'goals_remaining': 0,
                'progress': 0.0
            }

        if current_goal.is_reached(current_pose):
            self._complete_current_goal()
            return self.update_navigation(current_pose)
        if self._is_goal_timeout(current_goal):
            self._fail_current_goal()
            return self.update_navigation(current_pose)
        
        if self.navigation_state == "IDLE":
            self.navigation_state = "NAVIGATING"
            self.start_time = time.time()
        
        progress = self._calculate_progress(current_pose)
        
        return {
            'state': self.navigation_state,
            'current_goal': current_goal,
            'goals_remaining': len(self.goals) - self.current_goal_index,
            'progress': progress,
            'distance_to_goal': current_goal.distance_to(current_pose)
        }
    
    def _complete_current_goal(self):
        if self.current_goal_index < len(self.goals):
            completed_goal = self.goals[self.current_goal_index]
            logger.info(f"到达目标: ({completed_goal.x:.2f}, {completed_goal.y:.2f})")
            
            self.current_goal_index += 1
            
            if self.current_goal_index >= len(self.goals):
                self.navigation_state = "REACHED"
                logger.info("所有目标已完成")
            else:
                logger.info(f"开始导航到下一个目标: {self.current_goal_index + 1}/{len(self.goals)}")
    
    def _fail_current_goal(self):
        if self.current_goal_index < len(self.goals):
            failed_goal = self.goals[self.current_goal_index]
            logger.warning(f"目标超时失败: ({failed_goal.x:.2f}, {failed_goal.y:.2f})")
            
            self.current_goal_index += 1
            self.navigation_state = "FAILED"
    
    def _is_goal_timeout(self, goal: Goal) -> bool:
        return time.time() - goal.created_time > self.timeout
    
    def _calculate_progress(self, current_pose: Pose) -> float:
        if not self.goals:
            return 0.0
        
        current_goal = self.get_current_goal()
        if current_goal is None:
            return 1.0
        
        distance_to_goal = current_goal.distance_to(current_pose)
        
        max_distance = 10.0
        progress = max(0.0, 1.0 - distance_to_goal / max_distance)
        
        return min(1.0, progress)
    
    def clear_goals(self):
        self.goals.clear()
        self.current_goal_index = 0
        self.navigation_state = "IDLE"
        self.start_time = None
        logger.info("所有目标已清空")
    
    def remove_goal(self, index: int) -> bool:
        if 0 <= index < len(self.goals):
            removed_goal = self.goals.pop(index)
            logger.info(f"移除目标: ({removed_goal.x:.2f}, {removed_goal.y:.2f})")
            
            # 调整当前目标索引
            if index < self.current_goal_index:
                self.current_goal_index -= 1
            elif index == self.current_goal_index:
                # 如果移除的是当前目标，导航状态重置
                self.navigation_state = "IDLE"
            
            return True
        
        return False
    
    def get_goals(self) -> List[Goal]:
        return self.goals.copy()
    
    def get_remaining_goals(self) -> List[Goal]:
        return self.goals[self.current_goal_index:]
    
    def is_navigation_complete(self) -> bool:
        return self.navigation_state in ["REACHED", "FAILED"]
    
    def reset_navigation(self):
        self.current_goal_index = 0
        self.navigation_state = "IDLE"
        self.start_time = None
        logger.info("导航状态已重置")
    
    def get_navigation_summary(self) -> Dict[str, Any]:
        return {
            'total_goals': len(self.goals),
            'completed_goals': self.current_goal_index,
            'remaining_goals': len(self.goals) - self.current_goal_index,
            'state': self.navigation_state,
            'current_goal': self.get_current_goal(),
            'is_complete': self.is_navigation_complete()
        }
