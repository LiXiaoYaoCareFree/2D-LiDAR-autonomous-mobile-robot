@echo off
echo Windows串口权限修复工具
echo ========================

echo 1. 检查当前用户权限...
whoami
echo.

echo 2. 检查串口设备...
wmic path win32_serialport get deviceid,description,name,status
echo.

echo 3. 尝试以管理员权限运行程序...
echo 请右键点击此批处理文件，选择"以管理员身份运行"
echo.

echo 4. 如果仍有问题，请尝试以下步骤：
echo    - 关闭所有可能使用串口的程序
echo    - 重新插拔USB设备
echo    - 更新USB转串口驱动
echo    - 尝试不同的USB端口
echo.

pause
