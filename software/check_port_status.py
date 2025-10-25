"""
Windows串口状态检查工具
用于诊断串口权限和占用问题
"""

import serial.tools.list_ports
import serial
import platform
import subprocess
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def check_port_permissions():
    """检查串口权限状态"""
    print("=== Windows串口权限检查 ===\n")
    
    # 1. 检查可用端口
    print("1. 可用串口设备:")
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("   未发现任何串口设备")
        return False
    
    for port in ports:
        print(f"   端口: {port.device}")
        print(f"   描述: {port.description}")
        print(f"   硬件ID: {port.hwid}")
        print()
    
    # 2. 测试端口访问权限
    print("2. 端口访问权限测试:")
    for port in ports:
        port_name = port.device
        print(f"   测试端口 {port_name}...")
        
        try:
            # 尝试打开端口
            ser = serial.Serial(
                port=port_name,
                baudrate=9600,
                timeout=1.0
            )
            ser.close()
            print(f"   ✅ {port_name} - 访问正常")
        except serial.SerialException as e:
            if "PermissionError" in str(e) or "拒绝访问" in str(e):
                print(f"   ❌ {port_name} - 权限被拒绝")
            else:
                print(f"   ⚠️  {port_name} - 其他错误: {e}")
        except Exception as e:
            print(f"   ❌ {port_name} - 错误: {e}")
    
    return True


def check_port_usage():
    """检查端口使用情况"""
    print("\n3. 端口使用情况检查:")
    
    try:
        # 使用WMIC检查串口使用情况
        result = subprocess.run([
            'wmic', 'path', 'win32_serialport', 'get', 
            'deviceid,description,name,status'
        ], capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("   串口设备状态:")
            lines = result.stdout.strip().split('\n')
            for line in lines[1:]:  # 跳过标题行
                if line.strip():
                    print(f"   {line.strip()}")
        else:
            print("   无法获取串口状态信息")
            
    except Exception as e:
        print(f"   检查失败: {e}")


def check_running_processes():
    """检查可能占用串口的进程"""
    print("\n4. 可能占用串口的进程:")
    
    try:
        # 检查常见的串口相关进程
        processes_to_check = [
            'putty.exe',
            'teraterm.exe', 
            'hyperterminal.exe',
            'arduino.exe',
            'platformio.exe',
            'python.exe'
        ]
        
        result = subprocess.run([
            'tasklist', '/FI', 'IMAGENAME eq python.exe'
        ], capture_output=True, text=True, timeout=5)
        
        if 'python.exe' in result.stdout:
            print("   发现Python进程正在运行，可能占用串口")
            print("   建议关闭其他Python程序后重试")
        else:
            print("   未发现明显的串口占用进程")
            
    except Exception as e:
        print(f"   进程检查失败: {e}")


def provide_solutions():
    """提供解决方案"""
    print("\n=== 解决方案 ===")
    print("1. 以管理员身份运行程序:")
    print("   - 右键点击命令提示符")
    print("   - 选择'以管理员身份运行'")
    print("   - 重新运行程序")
    print()
    print("2. 检查端口占用:")
    print("   - 关闭其他可能使用串口的程序")
    print("   - 如PuTTY、TeraTerm、Arduino IDE等")
    print()
    print("3. 重新插拔USB设备:")
    print("   - 拔掉USB转串口设备")
    print("   - 等待5秒")
    print("   - 重新插入设备")
    print()
    print("4. 更新驱动程序:")
    print("   - 检查设备管理器中的串口设备")
    print("   - 更新USB转串口驱动")
    print()
    print("5. 使用其他端口:")
    print("   - 尝试不同的USB端口")
    print("   - 检查设备管理器中的COM端口号")


def main():
    """主函数"""
    print("Windows串口权限诊断工具")
    print("=" * 50)
    
    # 检查系统信息
    print(f"操作系统: {platform.system()} {platform.release()}")
    print(f"Python版本: {platform.python_version()}")
    print()
    
    # 执行检查
    if check_port_permissions():
        check_port_usage()
        check_running_processes()
    
    # 提供解决方案
    provide_solutions()


if __name__ == "__main__":
    main()
