@echo off
echo 启动Python SLAM仿真...

cd /d "%~dp0"

echo 切换到PoseGraph_Slam-Simulation目录...
cd ..\..\PoseGraph_Slam-Simulation

echo 当前目录: %CD%
echo 检查文件是否存在...
if not exist "maze_slam_simulation.py" (
    echo 错误: maze_slam_simulation.py 不存在
    pause
    exit /b 1
)

if not exist "4.json" (
    echo 错误: 4.json 不存在
    pause
    exit /b 1
)

echo 启动Python SLAM仿真...
python maze_slam_simulation.py --map="4.json"

pause
