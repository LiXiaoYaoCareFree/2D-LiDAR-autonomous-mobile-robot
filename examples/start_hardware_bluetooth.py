#!/usr/bin/env python3
"""
硬件蓝牙连接模块 - 快速启动脚本
提供多种启动模式选择
"""

import sys
import os
import argparse
from pathlib import Path

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='硬件蓝牙连接模块')
    parser.add_argument('--mode', choices=['test', 'visual', 'simple'], 
                       default='test', help='启动模式')
    parser.add_argument('--port', default='COM3', help='串口端口')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--duration', type=int, default=30, help='测试持续时间(秒)')
    
    args = parser.parse_args()
    
    print("🚀 硬件蓝牙连接模块启动器")
    print("="*50)
    print(f"模式: {args.mode}")
    print(f"端口: {args.port}")
    print(f"波特率: {args.baudrate}")
    print("="*50)
    
    if args.mode == 'test':
        # 测试模式
        print("🧪 启动测试模式...")
        from hardware_bluetooth_test import SimpleBluetoothTest
        
        test = SimpleBluetoothTest(port=args.port, baudrate=args.baudrate)
        if test.connect():
            try:
                test.read_data_loop(duration=args.duration)
            except KeyboardInterrupt:
                print("\n📌 用户中断")
            finally:
                test.print_statistics()
                test.disconnect()
    
    elif args.mode == 'visual':
        # 可视化模式
        print("📊 启动可视化模式...")
        from hardware_bluetooth_module import HardwareBluetoothInterface, HardwareBluetoothVisualizer
        
        bt_interface = HardwareBluetoothInterface(port=args.port, baudrate=args.baudrate)
        if bt_interface.connect():
            try:
                visualizer = HardwareBluetoothVisualizer(bt_interface)
                visualizer.start_visualization()
            except KeyboardInterrupt:
                print("\n📌 用户中断")
            finally:
                bt_interface.disconnect()
    
    elif args.mode == 'simple':
        # 简单模式
        print("🔧 启动简单模式...")
        import serial
        import time
        
        try:
            ser = serial.Serial(args.port, args.baudrate, timeout=0.1)
            print(f"✅ 连接成功: {args.port}")
            
            print("📡 开始读取数据...")
            print("按 Ctrl+C 停止")
            
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"📥 {line}")
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n📌 用户中断")
        except Exception as e:
            print(f"❌ 异常: {e}")
        finally:
            if 'ser' in locals():
                ser.close()
                print("📌 连接已关闭")

if __name__ == "__main__":
    main()
