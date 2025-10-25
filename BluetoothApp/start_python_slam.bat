@echo off
echo 启动Python SLAM仿真...

cd /d "%~dp0"

echo 检查Python环境...
python --version
if %errorlevel% neq 0 (
    echo 错误: Python未安装或未配置环境变量
    pause
    exit /b 1
)

echo.
echo 检查依赖包...
python -c "import numpy, matplotlib, scipy" 2>nul
if %errorlevel% neq 0 (
    echo 依赖包缺失，正在安装...
    pip install numpy matplotlib scipy
    if %errorlevel% neq 0 (
        echo 依赖包安装失败，请手动安装: pip install numpy matplotlib scipy
        pause
        exit /b 1
    )
)

echo.
echo 启动Python SLAM仿真...
python "..\PoseGraph_Slam-Simulation\maze_slam_simulation.py" --map="..\PoseGraph_Slam-Simulation\4.json"

pause
