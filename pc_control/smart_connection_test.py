#!/usr/bin/env python3
"""
智能连接测试工具
专门识别小车蓝牙端口并测试数据接收
"""

import serial
import time
import re
import threading
from collections import defaultdict

class SmartConnectionTester:
    def __init__(self):
        self.port_data = defaultdict(list)
        self.test_results = {}
        
    def test_port_data(self, port, baudrate=115200, test_duration=5):
        """测试端口数据接收"""
        print(f"🔍 Testing {port} for {test_duration} seconds...")
        
        try:
            ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f"✅ {port} opened successfully")
            
            data_count = 0
            start_time = time.time()
            last_data_time = start_time
            
            while time.time() - start_time < test_duration:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        data_count += 1
                        last_data_time = time.time()
                        
                        # 存储数据样本
                        if len(self.port_data[port]) < 10:  # 只保存前10条数据
                            self.port_data[port].append(line)
                        
                        # 实时显示数据
                        if data_count <= 5:  # 只显示前5条
                            print(f"  📡 [{data_count}] {line[:60]}")
                
                time.sleep(0.01)
            
            ser.close()
            
            # 分析结果
            time_since_last_data = time.time() - last_data_time
            is_active = data_count > 0 and time_since_last_data < 2
            
            self.test_results[port] = {
                'data_count': data_count,
                'is_active': is_active,
                'last_data_time': time_since_last_data,
                'sample_data': self.port_data[port][:3]  # 前3条数据样本
            }
            
            if is_active:
                print(f"✅ {port}: ACTIVE - {data_count} data packets received")
            else:
                print(f"❌ {port}: INACTIVE - {data_count} data packets received")
                
        except serial.SerialException as e:
            print(f"❌ {port}: Connection failed - {e}")
            self.test_results[port] = {
                'data_count': 0,
                'is_active': False,
                'error': str(e)
            }
        except Exception as e:
            print(f"❌ {port}: Error - {e}")
            self.test_results[port] = {
                'data_count': 0,
                'is_active': False,
                'error': str(e)
            }
    
    def analyze_data_patterns(self):
        """分析数据模式，识别小车数据"""
        print("\n🔍 Analyzing data patterns...")
        
        for port, result in self.test_results.items():
            if result['is_active'] and result['sample_data']:
                print(f"\n📊 {port} data analysis:")
                
                # 检查数据格式
                radar_data = 0
                motor_data = 0
                imu_data = 0
                other_data = 0
                
                for data in result['sample_data']:
                    if "A:" in data and "D:" in data and "Q:" in data:
                        radar_data += 1
                        print(f"  🎯 Radar data: {data[:50]}...")
                    elif "MotorA:" in data and "RPMB:" in data:
                        motor_data += 1
                        print(f"  ⚙️  Motor data: {data[:50]}...")
                    elif "AccX:" in data and "GyroX:" in data:
                        imu_data += 1
                        print(f"  📱 IMU data: {data[:50]}...")
                    else:
                        other_data += 1
                        print(f"  ❓ Other data: {data[:50]}...")
                
                # 判断是否为小车数据
                if radar_data > 0 or motor_data > 0 or imu_data > 0:
                    print(f"  ✅ {port} appears to be CAR DATA!")
                    result['is_car_data'] = True
                else:
                    print(f"  ❌ {port} does not appear to be car data")
                    result['is_car_data'] = False
            else:
                result['is_car_data'] = False
    
    def find_car_port(self):
        """找到小车端口"""
        print("\n🎯 Finding car port...")
        
        car_ports = []
        for port, result in self.test_results.items():
            if result.get('is_car_data', False):
                car_ports.append(port)
        
        if car_ports:
            print(f"✅ Car ports found: {car_ports}")
            return car_ports[0]  # 返回第一个找到的小车端口
        else:
            print("❌ No car ports found")
            return None
    
    def run_comprehensive_test(self):
        """运行综合测试"""
        print("🚗 Smart Car Connection Test")
        print("=" * 50)
        
        # 测试常见端口
        common_ports = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']
        
        print("Testing ports for car data...")
        print("-" * 30)
        
        for port in common_ports:
            self.test_port_data(port, test_duration=3)
            time.sleep(0.5)  # 端口间间隔
        
        # 分析数据模式
        self.analyze_data_patterns()
        
        # 找到小车端口
        car_port = self.find_car_port()
        
        # 总结报告
        print("\n" + "=" * 50)
        print("📋 TEST SUMMARY")
        print("=" * 50)
        
        for port, result in self.test_results.items():
            status = "✅ CAR" if result.get('is_car_data', False) else "❌ NOT CAR"
            data_count = result.get('data_count', 0)
            print(f"{port}: {status} ({data_count} packets)")
        
        if car_port:
            print(f"\n🎯 RECOMMENDED CAR PORT: {car_port}")
            print("Use this port in your control program!")
        else:
            print("\n❌ NO CAR PORT FOUND")
            print("Please check:")
            print("1. Hardware is powered on")
            print("2. STM32 code is running")
            print("3. Bluetooth is connected")
            print("4. Sensors are working")
        
        return car_port

def main():
    tester = SmartConnectionTester()
    car_port = tester.run_comprehensive_test()
    
    if car_port:
        print(f"\n🎉 SUCCESS! Use port {car_port} for car control")
    else:
        print(f"\n🔧 TROUBLESHOOTING NEEDED")
    
    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
