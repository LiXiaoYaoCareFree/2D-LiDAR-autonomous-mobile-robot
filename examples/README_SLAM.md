# 迷宫探索SLAM可视化

本项目将迷宫探索与BreezySLAM风格的可视化结合起来，实现了一个完整的迷宫探索SLAM（同步定位与地图构建）系统。

## 功能特点

1. **实时SLAM可视化**：使用`maze_slam.py`可以实时显示机器人探索迷宫的过程，包括地图构建和路径规划。
2. **数据记录**：使用`maze_data_recorder.py`记录机器人探索过程中的数据，包括位置、激光扫描、地图状态等。
3. **离线回放**：使用`maze_visualizer.py`可以离线回放记录的探索过程。
4. **兼容BreezySLAM**：生成的数据文件与BreezySLAM兼容，可以使用BreezySLAM的工具进行处理。

## 安装依赖

确保已安装所需的依赖：

```bash
pip install numpy matplotlib
```

此外，还需要BreezySLAM库。本项目假设BreezySLAM目录位于`examples`目录下。如果没有，可以通过以下命令克隆：

```bash
cd examples
git clone https://github.com/simondlevy/BreezySLAM.git
```

## 使用方法

### 1. 实时SLAM可视化

运行以下命令启动实时SLAM可视化：

```bash
python maze_slam.py [选项]
```

选项：
- `--width WIDTH`：迷宫宽度，默认15
- `--height HEIGHT`：迷宫高度，默认15
- `--maze-id ID`：使用预定义的迷宫ID（1, 2, 或 3）
- `--json-file FILE`：使用JSON文件定义的迷宫
- `--no-odometry`：不使用里程计
- `--seed SEED`：随机种子，0表示使用确定性SLAM

示例：

```bash
# 使用预定义的迷宫1
python maze_slam.py --maze-id 1

# 使用20x20的随机迷宫，不使用里程计
python maze_slam.py --width 20 --height 20 --no-odometry

# 使用随机种子9999
python maze_slam.py --seed 9999
```

### 2. 离线回放

运行以下命令回放记录的探索过程：

```bash
python maze_visualizer.py 数据文件路径 [--speed 速度]
```

选项：
- `数据文件路径`：记录的数据文件路径（.pkl文件）
- `--speed SPEED`：回放速度倍数，默认1.0

示例：

```bash
# 正常速度回放
python maze_visualizer.py data/maze_slam_15x15_20230501_123456.pkl

# 2倍速回放
python maze_visualizer.py data/maze_slam_15x15_20230501_123456.pkl --speed 2.0
```

### 3. 使用BreezySLAM工具

本系统生成的数据文件与BreezySLAM兼容，可以使用BreezySLAM的工具进行处理。例如：

```bash
# 使用BreezySLAM的logmovie.py回放
python BreezySLAM/examples/logmovie.py data/maze_slam_15x15_20230501_123456 1 9999
```

## 文件说明

- `maze_slam.py`：实时SLAM可视化主程序
- `maze_data_recorder.py`：数据记录器
- `maze_visualizer.py`：离线回放工具
- `maze_robot.py`：机器人类，已添加激光扫描功能
- `maze_env.py`：迷宫环境类

## 工作原理

1. **数据记录**：
   - 记录机器人位置、方向
   - 记录激光扫描数据
   - 记录地图状态
   - 记录规划路径和目标点

2. **SLAM可视化**：
   - 使用BreezySLAM算法进行同步定位与地图构建
   - 实时显示机器人位置和构建的地图
   - 支持使用里程计数据提高精度

3. **离线回放**：
   - 加载记录的数据
   - 按照原始时间间隔回放
   - 支持调整回放速度

## 注意事项

1. 确保BreezySLAM目录位于正确位置，或者已正确安装BreezySLAM库。
2. 数据文件可能较大，特别是长时间的探索过程。
3. 在高分辨率地图上运行SLAM可能需要较高的计算资源。

## 故障排除

1. **无法导入BreezySLAM模块**：
   - 确保BreezySLAM目录位于正确位置
   - 尝试安装BreezySLAM：`cd BreezySLAM/python && python setup.py install`

2. **可视化窗口无响应**：
   - 检查matplotlib后端设置
   - 尝试使用不同的后端：`matplotlib.use('TkAgg')`或`matplotlib.use('Qt5Agg')`

3. **数据记录文件过大**：
   - 增加记录间隔：修改`record_interval`参数
   - 减少记录的数据类型：修改`MazeDataRecorder.record()`方法 