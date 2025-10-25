from enum import IntEnum
from typing import Dict, Any
import logging

logger = logging.getLogger(__name__)


class RobotCommand(IntEnum):
    STOP = 0x00
    FORWARD = 0x01
    BACKWARD = 0x02
    LEFT = 0x03
    RIGHT = 0x04


class CommandProtocol:

    COMMAND_DESCRIPTIONS = {
        RobotCommand.STOP: "停止",
        RobotCommand.FORWARD: "前进",
        RobotCommand.BACKWARD: "后退", 
        RobotCommand.LEFT: "左转",
        RobotCommand.RIGHT: "右转"
    }
    
    @staticmethod
    def get_command_description(command: RobotCommand) -> str:
        return CommandProtocol.COMMAND_DESCRIPTIONS.get(command, "未知命令")
    
    @staticmethod
    def validate_command(command: int) -> bool:
        return command in [cmd.value for cmd in RobotCommand]
    
    @staticmethod
    def create_velocity_command(linear_vel: float, angular_vel: float, 
                              wheel_base: float = 0.165, 
                              max_linear: float = 0.3, 
                              max_angular: float = 2.0) -> RobotCommand:
        linear_vel = max(-max_linear, min(max_linear, linear_vel))
        angular_vel = max(-max_angular, min(max_angular, angular_vel))
        
        v_r = linear_vel + (angular_vel * wheel_base / 2.0)
        v_l = linear_vel - (angular_vel * wheel_base / 2.0)
        
        vel_threshold = 0.02
        
        if v_l > vel_threshold and v_r > vel_threshold:
            return RobotCommand.FORWARD
        elif v_l < -vel_threshold and v_r < -vel_threshold:
            return RobotCommand.BACKWARD
        elif v_l < -vel_threshold and v_r > vel_threshold:
            return RobotCommand.LEFT
        elif v_l > vel_threshold and v_r < -vel_threshold:
            return RobotCommand.RIGHT
        else:
            return RobotCommand.STOP
    
    @staticmethod
    def log_command(command: RobotCommand, details: Dict[str, Any] = None):
        desc = CommandProtocol.get_command_description(command)
        if details:
            logger.info(f"执行命令: {desc} (0x{command.value:02X}) - {details}")
        else:
            logger.info(f"执行命令: {desc} (0x{command.value:02X})")
