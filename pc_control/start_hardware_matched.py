#!/usr/bin/env python3
"""
启动完全匹配硬件代码的蓝牙控制程序
基于hardware.c的数据格式和bluetooth2.py的显示方式
"""

import sys
import os

# 设置matplotlib使用英文
import matplotlib
matplotlib.rcParams['font.family'] = 'DejaVu Sans'

# 设置环境变量避免中文警告
os.environ['PYTHONIOENCODING'] = 'utf-8'

try:
    from hardware_matched_bluetooth import HardwareMatchedBluetooth
    print("Starting Hardware Matched Bluetooth Control System...")
    print("Features:")
    print("- 完全匹配hardware.c的数据格式")
    print("- 借鉴bluetooth2.py的雷达显示")
    print("- 支持所有hardware.c的控制命令")
    print("- 实时雷达扫描显示")
    print("- 三轴角度显示")
    print("- 电机和IMU数据显示")
    print("- 键盘控制：方向键 + 空格")
    print("-" * 60)
    
    app = HardwareMatchedBluetooth()
    app.run()
    
except ImportError as e:
    print(f"Import error: {e}")
    print("Please install required packages: pip install pyserial matplotlib numpy")
except Exception as e:
    print(f"Error: {e}")
    input("Press Enter to exit...")
