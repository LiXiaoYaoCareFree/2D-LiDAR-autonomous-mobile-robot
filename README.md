# 2D激光雷达自主移动机器人系统

这是一个使用2D激光雷达的自主移动机器人系统，实现了SLAM（同步定位与地图构建）、自主导航和迷宫求解功能。

## 功能特性

1. **SLAM**：使用BreezySLAM库处理激光雷达数据，构建环境地图
2. **路径规划**：实现基于A*算法的全局路径规划和动态窗口法(DWA)的局部避障
3. **迷宫求解**：能够自主探索未知环境，找到出口并规划最优返回路径
4. **可视化界面**：美观的GUI界面，实时显示地图构建和导航过程
5. **数据记录**：记录传感器数据、地图和导航命令，支持离线分析

## 系统要求

- Python 3.8+
- numpy
- matplotlib
- pygame (可视化界面)
- PySerial (蓝牙通信)
- scikit-image (地图处理)

## 安装

1. 克隆仓库
```bash
git clone https://github.com/yourusername/2d-lidar-robot.git
cd 2d-lidar-robot
```

2. 安装依赖
```bash
pip install -r requirements.txt
```

## 使用方法

### 使用模拟器演示
```bash
python demo.py
```

### 使用真实设备
```bash
python demo.py --map-size 800 --map-scale 32.0 --log-dir ./logs
```

### 高级迷宫求解演示

1. 使用模拟器
```bash
python demo_maze_solver.py maze --simulator --map-size 800 --map-scale 32.0
```

2. 使用实际设备
```bash
python demo_maze_solver.py maze --port COM3 --map-size 800 --map-scale 32.0
```

### BreezySLAM示例演示

1. 使用exp1数据集(使用里程计)
```bash
python demo_maze_solver.py breezyslam --dataset exp1 --odometry
```

2. 使用exp2数据集(不使用里程计)
```bash
python demo_maze_solver.py breezyslam --dataset exp2 --no-odometry
```

3. 使用粒子滤波
```bash
python demo_maze_solver.py breezyslam --dataset exp1 --seed 9999
```

4. 使用增强可视化界面
```bash
python demo_maze_solver.py breezyslam --dataset exp1 --enhanced-viz
```

5. 也可以通过demo.py运行BreezySLAM示例
```bash
python demo.py --breezyslam --dataset exp1 --odometry 1 --seed 9999
```

## 模块说明

- **SLAM模块**：处理激光雷达和里程计数据，构建环境地图
- **导航模块**：实现边界探索和路径规划
- **可视化模块**：显示地图和机器人位置
- **通信模块**：通过蓝牙与机器人通信
- **数据记录模块**：记录传感器数据和导航命令
- **模拟器**：模拟激光雷达数据和随机迷宫环境

## 示例数据集

系统集成了BreezySLAM项目中的两个示例数据集：

- **exp1.dat**：包含激光雷达和里程计数据的示例1
- **exp2.dat**：包含激光雷达和里程计数据的示例2

## 项目结构

```
.
├── modules/                # 功能模块
│   ├── slam_module.py      # SLAM模块
│   ├── navigation.py       # 导航模块
│   ├── visualization.py    # 可视化模块
│   ├── communication.py    # 通信模块
│   ├── data_logger.py      # 数据记录模块
│   └── advanced_navigation.py # 高级导航模块
├── BreezySLAM/             # BreezySLAM库
├── demo.py                 # 基础演示脚本
├── demo_maze_solver.py     # 迷宫求解演示脚本
├── simulator.py            # 模拟器
└── README.md               # 说明文档
```

## 扩展开发

系统设计为模块化结构，可以方便地扩展和定制。

1. 添加新的传感器支持
2. 实现更先进的SLAM算法
3. 开发更高级的路径规划算法
4. 增强可视化界面功能

## 许可证

MIT
