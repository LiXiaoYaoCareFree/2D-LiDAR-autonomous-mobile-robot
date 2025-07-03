# 迷宫探索与路径规划

本项目实现了一个迷宫探索与路径规划系统，支持从JSON文件加载预定义迷宫，进行探索、路径规划和导航。

## 功能特点

- 支持从JSON文件加载预定义迷宫地图
- 支持随机生成迷宫
- 实现了迷宫探索算法
- 实现了A*路径规划算法
- 支持机器人连续移动，避免跳格子现象
- 可视化迷宫探索和路径规划过程

## 文件说明

- `maze_env.py`: 迷宫环境类，支持从JSON文件加载迷宫
- `maze_robot.py`: 机器人类，实现了探索和导航功能
- `maze_exploration_main.py`: 主程序，控制迷宫探索和导航流程
- `maze_visualization.py`: 可视化模块，显示迷宫探索和导航过程
- `maze_json_loader.py`: JSON迷宫加载器，从JSON文件加载迷宫数据
- `maze_json_test.py`: 测试从JSON文件加载迷宫
- `maze_json_visualizer.py`: 可视化JSON文件中定义的迷宫

## 使用方法

### 运行迷宫探索

```bash
# 使用预定义的迷宫1
python maze_exploration_main.py --maze-id 1

# 使用预定义的迷宫2
python maze_exploration_main.py --maze-id 2

# 使用预定义的迷宫3
python maze_exploration_main.py --maze-id 3

# 使用自定义JSON迷宫文件
python maze_exploration_main.py --json path/to/your/maze.json

# 使用随机生成的迷宫
python maze_exploration_main.py
```

### 可视化JSON迷宫

```bash
# 可视化预定义的迷宫1
python maze_json_visualizer.py --maze-id 1

# 可视化自定义JSON迷宫文件
python maze_json_visualizer.py --json path/to/your/maze.json

# 保存迷宫图像
python maze_json_visualizer.py --maze-id 2 --save maze2.png
```

## JSON迷宫格式

JSON迷宫文件格式如下：

```json
{
  "segments": [
    {"start": [x1, y1], "end": [x2, y2]},
    ...
  ],
  "start_point": [x, y]
}
```

- `segments`: 迷宫墙壁线段列表，每个线段由起点和终点定义
- `start_point`: 机器人起始位置

## 示例

项目包含三个预定义的迷宫示例：

1. `json_data/1.json`: 简单迷宫，15x15大小
2. `json_data/2.json`: 中等复杂度迷宫，15x15大小
3. `json_data/3.json`: 复杂迷宫，21x21大小

## 算法说明

- **迷宫探索**: 使用深度优先搜索(DFS)策略探索未知迷宫
- **路径规划**: 使用A*算法规划从当前位置到目标位置的最短路径
- **连续移动**: 实现了插值移动算法，确保机器人移动连续，避免跳格子现象 