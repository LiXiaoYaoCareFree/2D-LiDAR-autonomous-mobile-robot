#!/usr/bin/env python3
"""
自动运动模式
让小车在没有雷达数据的情况下也能自动运动
"""

import serial
import time
import threading

class AutoMotion:
    def __init__(self, port='COM3', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        self.motion_thread = None
        
    def connect(self):
        """连接蓝牙"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✅ Connected to {self.port}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def send_command(self, command):
        """发送控制命令"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command.encode())
                print(f"📤 Sent: {command.strip()}")
                return True
            except Exception as e:
                print(f"❌ Send failed: {e}")
                return False
        return False
    
    def forward(self, duration=2):
        """前进"""
        print("🚗 Moving forward...")
        self.send_command('1')  # 前进命令
        time.sleep(duration)
        self.send_command('0')  # 停止命令
        print("⏹️  Stopped")
    
    def backward(self, duration=2):
        """后退"""
        print("🚗 Moving backward...")
        self.send_command('2')  # 后退命令
        time.sleep(duration)
        self.send_command('0')  # 停止命令
        print("⏹️  Stopped")
    
    def left_turn(self, duration=1):
        """左转"""
        print("🚗 Turning left...")
        self.send_command('3')  # 左转命令
        time.sleep(duration)
        self.send_command('0')  # 停止命令
        print("⏹️  Stopped")
    
    def right_turn(self, duration=1):
        """右转"""
        print("🚗 Turning right...")
        self.send_command('4')  # 右转命令
        time.sleep(duration)
        self.send_command('0')  # 停止命令
        print("⏹️  Stopped")
    
    def u_turn(self, duration=2):
        """掉头"""
        print("🚗 U-turn...")
        self.send_command('5')  # 掉头命令
        time.sleep(duration)
        self.send_command('0')  # 停止命令
        print("⏹️  Stopped")
    
    def stop(self):
        """停止"""
        print("⏹️  Stopping...")
        self.send_command('0')  # 停止命令
    
    def auto_patrol(self):
        """自动巡逻模式"""
        print("🤖 Starting auto patrol mode...")
        self.is_running = True
        
        while self.is_running:
            try:
                # 前进
                self.forward(3)
                time.sleep(0.5)
                
                # 右转
                self.right_turn(1)
                time.sleep(0.5)
                
                # 前进
                self.forward(3)
                time.sleep(0.5)
                
                # 左转
                self.left_turn(1)
                time.sleep(0.5)
                
                # 后退
                self.backward(2)
                time.sleep(0.5)
                
                # 掉头
                self.u_turn(2)
                time.sleep(1)
                
            except KeyboardInterrupt:
                print("\n🛑 Auto patrol stopped by user")
                break
        
        self.stop()
        print("🏁 Auto patrol finished")
    
    def start_auto_patrol(self):
        """启动自动巡逻"""
        if not self.connect():
            return False
        
        # 启动自动巡逻线程
        self.motion_thread = threading.Thread(target=self.auto_patrol)
        self.motion_thread.daemon = True
        self.motion_thread.start()
        
        return True
    
    def stop_auto_patrol(self):
        """停止自动巡逻"""
        self.is_running = False
        self.stop()
    
    def close(self):
        """关闭连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("📌 Connection closed")

def main():
    print("🚗 Auto Motion Control")
    print("=" * 40)
    
    # 创建自动运动对象
    auto_motion = AutoMotion()
    
    # 连接蓝牙
    if not auto_motion.connect():
        print("❌ Failed to connect. Please check:")
        print("1. Bluetooth module is connected")
        print("2. Port is correct (COM3, COM4, etc.)")
        print("3. Hardware is powered on")
        return
    
    try:
        while True:
            print("\n🎮 Control Options:")
            print("1. Forward (3s)")
            print("2. Backward (3s)")
            print("3. Left turn (1s)")
            print("4. Right turn (1s)")
            print("5. U-turn (2s)")
            print("6. Auto patrol")
            print("7. Stop")
            print("8. Exit")
            
            choice = input("\nEnter choice (1-8): ").strip()
            
            if choice == '1':
                auto_motion.forward(3)
            elif choice == '2':
                auto_motion.backward(3)
            elif choice == '3':
                auto_motion.left_turn(1)
            elif choice == '4':
                auto_motion.right_turn(1)
            elif choice == '5':
                auto_motion.u_turn(2)
            elif choice == '6':
                auto_motion.start_auto_patrol()
                input("Press Enter to stop auto patrol...")
                auto_motion.stop_auto_patrol()
            elif choice == '7':
                auto_motion.stop()
            elif choice == '8':
                break
            else:
                print("❌ Invalid choice")
    
    except KeyboardInterrupt:
        print("\n🛑 Program stopped by user")
    
    finally:
        auto_motion.close()
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()
