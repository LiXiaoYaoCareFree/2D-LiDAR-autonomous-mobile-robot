# 2D激光雷达自主移动机器人

本项目实现了一个基于2D激光雷达的自主移动机器人系统，包括SLAM地图构建、路径规划、导航控制等功能。

## 功能特点

- **SLAM地图构建**：使用BreezySLAM库处理激光雷达数据，构建环境地图
- **边界探索**：实现边界检测算法，探索未知区域
- **路径规划**：使用A*算法规划最短路径
- **导航控制**：实现机器人的运动控制
- **数据可视化**：使用PyRoboViz可视化地图和路径
- **蓝牙通信**：与机器人硬件进行数据交换
- **数据记录**：记录传感器数据和导航命令
- **模拟器**：提供模拟环境进行测试

## 高级功能

### 随机迷宫解决方案

- **信息增益边界探索**：基于信息增益的边界探索策略，更高效地探索未知区域
- **出口检测**：自动检测迷宫出口位置
- **最优路径规划**：使用优化算法规划最优路径，考虑障碍物距离和路径平滑度
- **完整探索策略**：包括探索、找到出口、前往出口、返回起点的完整流程
- **性能统计**：记录探索时间、导航时间等性能指标

### 交互界面升级

- **美观的GUI界面**：使用Tkinter和Matplotlib构建美观的用户界面
- **实时地图可视化**：显示实时更新的占用栅格地图
- **导航可视化**：显示机器人位置、路径、边界点等
- **状态监控**：显示当前状态、性能统计等信息
- **控制面板**：提供启动、停止、重置等控制功能
- **日志记录**：显示系统运行日志

## 安装说明

1. 克隆仓库：

```bash
git clone https://github.com/yourusername/2d-lidar-robot.git
cd 2d-lidar-robot
```

2. 安装依赖：

```bash
pip install -r requirements.txt
```

3. 安装BreezySLAM：

```bash
cd BreezySLAM
python setup.py install
cd ..
```

4. 安装PyRoboViz：

```bash
cd PyRoboViz
python setup.py install
cd ..
```

## 使用说明

### 运行演示程序

```bash
python demo_maze_solver.py --simulator
```

### 命令行参数

- `--simulator`：使用模拟器模式（默认）
- `--map-size-pixels`：设置地图大小（像素）
- `--map-size-meters`：设置地图大小（米）

### GUI界面操作

- **启动**：开始运行系统
- **停止**：停止系统运行
- **重置**：重置系统状态
- **保存地图**：保存当前地图
- **加载地图**：加载已有地图
- **设置**：打开设置面板

## 系统架构

- **主控模块**（robot_control_system.py）：协调各模块工作
- **SLAM模块**（slam_module.py）：处理激光雷达数据，构建地图
- **导航模块**（navigation.py）：实现边界检测和路径规划
- **高级导航模块**（advanced_navigation.py）：实现信息增益边界探索和最优路径规划
- **迷宫求解器**（maze_solver.py）：实现迷宫探索和解决策略
- **可视化模块**（visualization.py）：显示地图和路径
- **增强可视化模块**（enhanced_visualization.py）：提供美观的GUI界面
- **通信模块**（communication.py）：与机器人进行数据交换
- **数据记录模块**（data_logger.py）：记录传感器数据和导航命令
- **模拟器**（simulator.py）：生成模拟数据用于测试

## 开发环境

- Python 3.8+
- Windows 10/11 或 Linux
- 依赖库：NumPy, Matplotlib, SciPy, Tkinter, PySerial 等

## 许可证

MIT License
