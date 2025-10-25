@echo off
echo 正在安装Python SLAM仿真所需的依赖包...
echo.

echo 检查Python环境...
python --version
if %errorlevel% neq 0 (
    echo 错误: Python未安装或未配置环境变量
    echo 请先安装Python 3.7+并配置环境变量
    pause
    exit /b 1
)

echo.
echo 安装numpy...
pip install numpy

echo.
echo 安装matplotlib...
pip install matplotlib

echo.
echo 安装scipy...
pip install scipy

echo.
echo 验证安装...
python -c "import numpy, matplotlib, scipy; print('All dependencies installed successfully!')"

if %errorlevel% equ 0 (
    echo.
    echo ✅ Python依赖包安装完成！
    echo 现在可以正常启动Python SLAM仿真了。
) else (
    echo.
    echo ❌ 依赖包安装失败，请检查网络连接或Python环境。
)

echo.
pause
