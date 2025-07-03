# JSON迷宫加载功能

本文档介绍如何使用JSON文件定义和加载迷宫地图。

## 功能概述

本项目新增了从JSON文件加载预定义迷宫地图的功能，使用户可以：

1. 使用预定义的迷宫进行探索和导航
2. 自定义迷宫地图并保存为JSON格式
3. 可视化JSON格式的迷宫地图

## JSON迷宫文件格式

JSON迷宫文件格式如下：

```json
{
  "segments": [
    {"start": [x1, y1], "end": [x2, y2]},
    {"start": [x3, y3], "end": [x4, y4]},
    ...
  ],
  "start_point": [x, y]
}
```

其中：
- `segments`: 迷宫墙壁线段列表，每个线段由起点和终点定义
- `start_point`: 机器人起始位置

## 相关文件

- `maze_json_loader.py`: 从JSON文件加载迷宫数据
- `maze_env.py`: 修改后支持从JSON文件加载迷宫
- `maze_json_test.py`: 测试从JSON文件加载迷宫
- `maze_json_visualizer.py`: 可视化JSON文件中定义的迷宫

## 使用方法

### 1. 运行迷宫探索（使用JSON迷宫）

```bash
# 使用预定义的迷宫1
python maze_exploration_main.py --maze-id 1

# 使用预定义的迷宫2
python maze_exploration_main.py --maze-id 2

# 使用预定义的迷宫3
python maze_exploration_main.py --maze-id 3

# 使用自定义JSON迷宫文件
python maze_exploration_main.py --json path/to/your/maze.json
```

### 2. 可视化JSON迷宫

```bash
# 可视化预定义的迷宫1
python maze_json_visualizer.py --maze-id 1

# 可视化预定义的迷宫2
python maze_json_visualizer.py --maze-id 2

# 可视化预定义的迷宫3
python maze_json_visualizer.py --maze-id 3

# 可视化自定义JSON迷宫文件
python maze_json_visualizer.py --json path/to/your/maze.json

# 保存迷宫图像
python maze_json_visualizer.py --maze-id 2 --save maze2.png
```

### 3. 创建自定义JSON迷宫

您可以参照`json_data`目录下的示例文件创建自己的迷宫：

1. 定义迷宫的墙壁线段，每个线段由起点和终点坐标表示
2. 定义机器人的起始位置
3. 将数据保存为JSON格式

## 示例迷宫

项目包含三个预定义的迷宫示例：

1. `json_data/1.json`: 简单迷宫，15x15大小
2. `json_data/2.json`: 中等复杂度迷宫，15x15大小
3. `json_data/3.json`: 复杂迷宫，21x21大小

## 技术实现

1. **JSON解析**: 使用Python的`json`模块解析迷宫文件
2. **障碍物转换**: 将线段转换为栅格地图中的障碍物点
3. **环境集成**: 将JSON迷宫无缝集成到现有的迷宫环境中
4. **命令行参数**: 通过命令行参数选择不同的迷宫文件

## 注意事项

1. 确保线段的坐标在迷宫范围内
2. 起始点不应位于障碍物上
3. 对于复杂迷宫，可能需要调整迷宫大小参数 