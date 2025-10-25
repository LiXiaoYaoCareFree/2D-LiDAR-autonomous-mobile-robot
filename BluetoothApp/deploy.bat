@echo off
chcp 65001 >nul
echo ========================================
echo 蓝牙小车控制App 部署脚本
echo ========================================
echo.

REM 检查是否已编译
if not exist "bin\Release\BluetoothApp.exe" (
    echo 错误: 未找到编译后的EXE文件
    echo 请先运行 build.bat 进行编译
    pause
    exit /b 1
)

echo 创建部署目录...
set DEPLOY_DIR=deploy\BluetoothApp
if exist "deploy" rmdir /s /q "deploy"
mkdir "%DEPLOY_DIR%"

echo 复制程序文件...
copy "bin\Release\BluetoothApp.exe" "%DEPLOY_DIR%\"
copy "bin\Release\InTheHand.Net.Personal.dll" "%DEPLOY_DIR%\"
copy "20231020092828_b7e3bf620a1e4d99b1326f8a4ac35c77.ico" "%DEPLOY_DIR%\"
copy "README_小车控制.md" "%DEPLOY_DIR%\"

echo 创建启动脚本...
echo @echo off > "%DEPLOY_DIR%\启动蓝牙App.bat"
echo echo 正在启动蓝牙小车控制App... >> "%DEPLOY_DIR%\启动蓝牙App.bat"
echo start "" "BluetoothApp.exe" >> "%DEPLOY_DIR%\启动蓝牙App.bat"

echo 创建使用说明...
echo 蓝牙小车控制App > "%DEPLOY_DIR%\使用说明.txt"
echo. >> "%DEPLOY_DIR%\使用说明.txt"
echo 使用方法: >> "%DEPLOY_DIR%\使用说明.txt"
echo 1. 双击"启动蓝牙App.bat"或直接运行"BluetoothApp.exe" >> "%DEPLOY_DIR%\使用说明.txt"
echo 2. 点击"扫描"按钮搜索蓝牙设备 >> "%DEPLOY_DIR%\使用说明.txt"
echo 3. 选择小车蓝牙设备进行连接 >> "%DEPLOY_DIR%\使用说明.txt"
echo 4. 连接成功后点击"小车控制"按钮 >> "%DEPLOY_DIR%\使用说明.txt"
echo 5. 使用控制界面操作小车 >> "%DEPLOY_DIR%\使用说明.txt"
echo. >> "%DEPLOY_DIR%\使用说明.txt"
echo 系统要求: >> "%DEPLOY_DIR%\使用说明.txt"
echo - Windows 10 或更高版本 >> "%DEPLOY_DIR%\使用说明.txt"
echo - .NET Framework 4.5 或更高版本 >> "%DEPLOY_DIR%\使用说明.txt"
echo - 蓝牙适配器 >> "%DEPLOY_DIR%\使用说明.txt"
echo. >> "%DEPLOY_DIR%\使用说明.txt"
echo 故障排除: >> "%DEPLOY_DIR%\使用说明.txt"
echo - 如果无法连接蓝牙，请检查蓝牙是否开启 >> "%DEPLOY_DIR%\使用说明.txt"
echo - 如果程序无法启动，请以管理员身份运行 >> "%DEPLOY_DIR%\使用说明.txt"
echo - 确保.NET Framework已正确安装 >> "%DEPLOY_DIR%\使用说明.txt"

echo.
echo ========================================
echo 部署完成！
echo ========================================
echo.
echo 部署目录: %DEPLOY_DIR%
echo.
echo 文件列表:
dir "%DEPLOY_DIR%" /b
echo.
echo 总大小:
dir "%DEPLOY_DIR%" | findstr "个文件"
echo.
echo 您可以将整个 "%DEPLOY_DIR%" 文件夹复制到其他电脑使用
echo.
echo 是否要运行程序？(Y/N)
set /p choice=
if /i "%choice%"=="Y" (
    echo 启动程序...
    cd "%DEPLOY_DIR%"
    start "" "BluetoothApp.exe"
)

echo.
echo 按任意键退出...
pause >nul
