@echo off
chcp 65001 >nul
echo ========================================
echo 蓝牙小车控制App 编译脚本
echo ========================================
echo.

REM 检查是否存在解决方案文件
if not exist "BluetoothApp.sln" (
    echo 错误: 找不到 BluetoothApp.sln 文件
    echo 请确保在正确的目录中运行此脚本
    pause
    exit /b 1
)

echo 正在查找MSBuild...

REM 尝试不同的MSBuild路径
set MSBUILD_PATH=""
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" (
    set MSBUILD_PATH="C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe"
    echo 找到 Visual Studio 2022 Community
) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\MSBuild.exe" (
    set MSBUILD_PATH="C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\MSBuild.exe"
    echo 找到 Visual Studio 2022 Professional
) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe" (
    set MSBUILD_PATH="C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe"
    echo 找到 Visual Studio 2019 Community
) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\MSBuild\Current\Bin\MSBuild.exe" (
    set MSBUILD_PATH="C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\MSBuild\Current\Bin\MSBuild.exe"
    echo 找到 Visual Studio 2019 Professional
) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\MSBuild\15.0\Bin\MSBuild.exe" (
    set MSBUILD_PATH="C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\MSBuild\15.0\Bin\MSBuild.exe"
    echo 找到 Visual Studio 2017 Community
) else (
    echo 错误: 找不到MSBuild，请确保已安装Visual Studio
    echo 支持的版本: Visual Studio 2017/2019/2022
    pause
    exit /b 1
)

echo 使用MSBuild路径: %MSBUILD_PATH%
echo.

echo 开始清理旧文件...
if exist "bin\Release" rmdir /s /q "bin\Release"
if exist "bin\Debug" rmdir /s /q "bin\Debug"
echo 清理完成
echo.

echo 开始编译Release版本...
%MSBUILD_PATH% BluetoothApp.sln /p:Configuration=Release /p:Platform="Any CPU" /p:OutputPath=bin\Release\ /verbosity:minimal

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo 编译成功！
    echo ========================================
    echo.
    echo EXE文件位置: bin\Release\BluetoothApp.exe
    
    REM 检查EXE文件是否存在
    if exist "bin\Release\BluetoothApp.exe" (
        echo 文件大小: 
        dir "bin\Release\BluetoothApp.exe" | findstr "BluetoothApp.exe"
        echo.
        echo 依赖文件检查:
        if exist "bin\Release\InTheHand.Net.Personal.dll" (
            echo ✓ InTheHand.Net.Personal.dll 已复制
        ) else (
            echo ✗ InTheHand.Net.Personal.dll 缺失
        )
        echo.
        echo 是否要运行程序？(Y/N)
        set /p choice=
        if /i "%choice%"=="Y" (
            echo 启动程序...
            start "" "bin\Release\BluetoothApp.exe"
        )
    ) else (
        echo 警告: 未找到生成的EXE文件
    )
) else (
    echo.
    echo ========================================
    echo 编译失败！
    echo ========================================
    echo 请检查以下可能的问题:
    echo 1. 确保所有源文件都已正确添加到项目
    echo 2. 检查是否有编译错误
    echo 3. 确保.NET Framework 4.8已安装
    echo 4. 检查依赖库是否正确
)

echo.
echo 按任意键退出...
pause >nul
