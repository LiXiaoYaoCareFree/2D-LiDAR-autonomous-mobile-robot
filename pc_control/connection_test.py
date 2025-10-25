#!/usr/bin/env python3
"""
连接测试工具
检查硬件连接和数据接收
"""

import serial
import time
import re

def test_connection(port, baudrate=115200):
    """测试串口连接"""
    print(f"Testing connection to {port} at {baudrate} baud...")
    
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"✅ Successfully connected to {port}")
        
        # 测试数据接收
        print("Waiting for data from hardware...")
        print("Please ensure:")
        print("1. Hardware is powered on")
        print("2. STM32 code is running")
        print("3. Sensors are connected")
        print("-" * 50)
        
        data_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 测试10秒
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    data_count += 1
                    print(f"[{data_count}] Received: {line[:80]}")
                    
                    # 检查数据类型
                    if "A:" in line and "D:" in line and "Q:" in line:
                        print("  → Laser radar data detected")
                    elif "MotorA:" in line and "RPMB:" in line:
                        print("  → Motor data detected")
                    elif "AccX:" in line and "GyroX:" in line:
                        print("  → IMU data detected")
                    elif line.startswith("Connected"):
                        print("  → Connection notification")
                    else:
                        print("  → Other data")
            
            time.sleep(0.1)
        
        ser.close()
        
        if data_count > 0:
            print(f"\n✅ Connection test successful!")
            print(f"Received {data_count} data packets")
            print("Hardware is working correctly")
        else:
            print(f"\n❌ No data received")
            print("Possible issues:")
            print("1. Hardware not powered on")
            print("2. STM32 code not running")
            print("3. Wrong port or baudrate")
            print("4. Hardware connection problems")
            
    except serial.SerialException as e:
        print(f"❌ Connection failed: {e}")
        print("Possible solutions:")
        print("1. Check if port exists")
        print("2. Try different port (COM3, COM4, etc.)")
        print("3. Check if port is used by other program")
        print("4. Run as Administrator")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    print("Hardware Connection Test Tool")
    print("=" * 50)
    
    # 常见串口列表
    common_ports = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']
    
    print("Testing common ports...")
    print("-" * 30)
    
    working_ports = []
    
    for port in common_ports:
        try:
            print(f"\nTesting {port}...")
            test_connection(port)
            working_ports.append(port)
        except:
            print(f"Port {port} not available")
    
    print("\n" + "=" * 50)
    if working_ports:
        print(f"Working ports found: {working_ports}")
        print(f"Recommended port: {working_ports[0]}")
    else:
        print("No working ports found")
        print("Please check hardware connections")

if __name__ == "__main__":
    main()
    input("\nPress Enter to exit...")
