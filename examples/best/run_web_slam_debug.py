#!/usr/bin/env python3

"""
迷宫SLAM Web可视化系统调试启动脚本
"""

import sys
import os
import traceback

def main():
    print("🌐 迷宫SLAM Web可视化系统 (调试模式)")
    print("=" * 60)
    
    try:
        # 检查依赖包
        print("🔍 检查依赖包...")
        try:
            import flask
            print(f"✅ Flask: {flask.__version__}")
        except ImportError:
            print("❌ Flask未安装")
            
        try:
            import flask_socketio
            # Flask-SocketIO的版本检查方式不同
            try:
                version = flask_socketio.__version__
            except AttributeError:
                try:
                    import pkg_resources
                    version = pkg_resources.get_distribution('flask-socketio').version
                except:
                    version = "已安装(版本未知)"
            print(f"✅ Flask-SocketIO: {version}")
        except ImportError:
            print("❌ Flask-SocketIO未安装")
            
        try:
            import eventlet
            try:
                version = eventlet.__version__
            except AttributeError:
                try:
                    import pkg_resources
                    version = pkg_resources.get_distribution('eventlet').version
                except:
                    version = "已安装(版本未知)"
            print(f"✅ Eventlet: {version}")
        except ImportError:
            print("❌ Eventlet未安装")
            
        try:
            import numpy
            print(f"✅ NumPy: {numpy.__version__}")
        except ImportError:
            print("⚠️  NumPy未安装 (原SLAM系统需要)")
            
        # 检查原SLAM模块
        try:
            from maze_slam_visual_new2 import GlobalMazeSLAMSystem
            print(f"✅ 原SLAM系统模块导入成功")
        except ImportError as e:
            print(f"❌ 原SLAM系统模块导入失败: {e}")
            raise
        
        print("=" * 60)
        
        # 导入Web可视化系统
        from web_visualizer import WebMazeSLAMVisualizer
        
        # 创建系统实例
        web_viz = WebMazeSLAMVisualizer()
        
        # 启动服务器
        print("🚀 正在启动Web服务器 (调试模式)...")
        print("📱 请在浏览器中访问 http://localhost:5000")
        print("🔧 调试模式已启用，将显示详细日志")
        print("🎮 在界面中配置参数并点击'开始仿真'按钮")
        print("⏹️  按 Ctrl+C 停止服务器")
        print("🔍 如果出现问题，请查看控制台输出")
        print("=" * 60)
        
        # 运行服务器 (开启调试模式)
        web_viz.run(host='127.0.0.1', port=5000, debug=True)
        
    except ImportError as e:
        print(f"❌ 导入模块失败: {e}")
        print("💡 请确保已安装所需依赖包:")
        print("   pip install flask flask-socketio eventlet")
        traceback.print_exc()
        sys.exit(1)
        
    except KeyboardInterrupt:
        print("\n⏹️  服务器已停止")
        sys.exit(0)
        
    except Exception as e:
        print(f"❌ 系统启动失败: {e}")
        print("🔍 详细错误信息:")
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main() 