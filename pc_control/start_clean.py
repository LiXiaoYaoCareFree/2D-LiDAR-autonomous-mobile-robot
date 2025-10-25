#!/usr/bin/env python3
"""
启动清洁版蓝牙控制程序 - 完全避免中文字体问题
"""

import sys
import os

# 设置matplotlib使用英文
import matplotlib
matplotlib.rcParams['font.family'] = 'DejaVu Sans'

# 设置环境变量避免中文警告
os.environ['PYTHONIOENCODING'] = 'utf-8'

try:
    from clean_bluetooth_control import CleanBluetoothControl
    print("Starting Clean Bluetooth Control System...")
    print("Features:")
    print("- No Chinese font warnings")
    print("- Matches hardware.c data format exactly")
    print("- Real-time radar display")
    print("- Three-axis angle display")
    print("- Motor and IMU data display")
    print("- Keyboard control: Arrow Keys + Space")
    print("-" * 60)
    
    app = CleanBluetoothControl()
    app.run()
    
except ImportError as e:
    print(f"Import error: {e}")
    print("Please install required packages: pip install pyserial matplotlib numpy")
except Exception as e:
    print(f"Error: {e}")
    input("Press Enter to exit...")
