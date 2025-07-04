"""
硬件通信接口包
提供与各种硬件平台（Arduino、树莓派等）的通信接口
"""

from .base_interface import HardwareInterface
from .serial_interface import SerialInterface
from .network_interface import NetworkInterface
from .gpio_interface import GPIOInterface

__all__ = [
    'HardwareInterface',
    'SerialInterface', 
    'NetworkInterface',
    'GPIOInterface'
] 