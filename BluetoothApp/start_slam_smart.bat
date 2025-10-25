@echo off
echo 启动Python SLAM仿真...

cd /d "%~dp0"

echo 智能查找PoseGraph_Slam-Simulation目录...

REM 尝试不同的路径
if exist "..\..\PoseGraph_Slam-Simulation\maze_slam_simulation.py" (
    echo 找到路径: ..\..\PoseGraph_Slam-Simulation
    cd ..\..\PoseGraph_Slam-Simulation
) else if exist "..\PoseGraph_Slam-Simulation\maze_slam_simulation.py" (
    echo 找到路径: ..\PoseGraph_Slam-Simulation
    cd ..\PoseGraph_Slam-Simulation
) else if exist "PoseGraph_Slam-Simulation\maze_slam_simulation.py" (
    echo 找到路径: PoseGraph_Slam-Simulation
    cd PoseGraph_Slam-Simulation
) else (
    echo 错误: 找不到PoseGraph_Slam-Simulation目录
    echo 当前目录: %CD%
    echo 请检查目录结构
    pause
    exit /b 1
)

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
