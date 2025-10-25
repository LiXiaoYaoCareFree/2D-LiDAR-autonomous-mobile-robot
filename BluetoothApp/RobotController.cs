using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using static BluetoothApp.BluetoothInfo;

namespace BluetoothApp
{
    /// <summary>
    /// 小车运动控制类
    /// 基于硬件代码中的控制协议实现
    /// </summary>
    public class RobotController
    {
        // 运动方向常量，对应硬件代码中的定义
        public const char DIR_STOP = '0';        // 停止
        public const char DIR_FORWARD = '1';     // 前进
        public const char DIR_BACKWARD = '2';    // 后退
        public const char DIR_LEFT = '3';        // 左转
        public const char DIR_RIGHT = '4';       // 右转
        public const char DIR_UTURN = '5';       // 掉头

        private BluetoothManage bluetoothManager;
        private char currentDirection = DIR_STOP;
        private bool isConnected = false;

        // 传感器数据存储
        public struct SensorData
        {
            public float MotorA_RPM;
            public float MotorB_RPM;
            public float AccelX, AccelY, AccelZ;
            public float GyroX, GyroY, GyroZ;
            public float LidarAngle, LidarDistance;
            public int LidarQuality;
            public float Voltage;
            public DateTime LastUpdate;
        }

        private SensorData _currentSensorData;
        public SensorData CurrentSensorData 
        { 
            get { return _currentSensorData; }
            private set { _currentSensorData = value; }
        }

        // 事件定义
        public event Action<SensorData> OnSensorDataReceived;
        public event Action<string> OnStatusChanged;
        public event Action<bool> OnConnectionChanged;

        public RobotController(BluetoothManage bluetoothManager)
        {
            this.bluetoothManager = bluetoothManager;
            this.bluetoothManager.SetListener(HandleBluetoothData);
        }

        /// <summary>
        /// 处理蓝牙数据回调
        /// </summary>
        private void HandleBluetoothData(MsgTypes type, string str, object data = null)
        {
            switch (type)
            {
                case MsgTypes.BluetoothData:
                    ProcessSensorData((byte[])data);
                    break;
                case MsgTypes.BluetoothStatus:
                    HandleConnectionStatus((BluetoothState)data, str);
                    break;
            }
        }

        /// <summary>
        /// 处理传感器数据
        /// </summary>
        private void ProcessSensorData(byte[] data)
        {
            string dataStr = Encoding.UTF8.GetString(data);
            ParseSensorData(dataStr);
        }

        /// <summary>
        /// 解析传感器数据
        /// </summary>
        private void ParseSensorData(string data)
        {
            try
            {
                // 解析电机RPM数据
                if (data.Contains("MotorA:") && data.Contains("RPMB:"))
                {
                    var parts = data.Split(',');
                    foreach (var part in parts)
                    {
                        if (part.Contains("MotorA:"))
                        {
                            float.TryParse(part.Replace("MotorA:", "").Trim(), out _currentSensorData.MotorA_RPM);
                        }
                        else if (part.Contains("RPMB:"))
                        {
                            float.TryParse(part.Replace("RPMB:", "").Trim(), out _currentSensorData.MotorB_RPM);
                        }
                    }
                }

                // 解析MPU6500数据
                if (data.Contains("AccX:") && data.Contains("GyroX:"))
                {
                    var parts = data.Split(',');
                    foreach (var part in parts)
                    {
                        var cleanPart = part.Trim();
                        if (cleanPart.StartsWith("AccX:"))
                            float.TryParse(cleanPart.Replace("AccX:", ""), out _currentSensorData.AccelX);
                        else if (cleanPart.StartsWith("AccY:"))
                            float.TryParse(cleanPart.Replace("AccY:", ""), out _currentSensorData.AccelY);
                        else if (cleanPart.StartsWith("AccZ:"))
                            float.TryParse(cleanPart.Replace("AccZ:", ""), out _currentSensorData.AccelZ);
                        else if (cleanPart.StartsWith("GyroX:"))
                            float.TryParse(cleanPart.Replace("GyroX:", ""), out _currentSensorData.GyroX);
                        else if (cleanPart.StartsWith("GyroY:"))
                            float.TryParse(cleanPart.Replace("GyroY:", ""), out _currentSensorData.GyroY);
                        else if (cleanPart.StartsWith("GyroZ:"))
                            float.TryParse(cleanPart.Replace("GyroZ:", ""), out _currentSensorData.GyroZ);
                    }
                }

                // 解析LiDAR数据
                if (data.Contains("A:") && data.Contains("D:") && data.Contains("Q:"))
                {
                    var parts = data.Split(',');
                    foreach (var part in parts)
                    {
                        var cleanPart = part.Trim();
                        if (cleanPart.StartsWith("A:"))
                            float.TryParse(cleanPart.Replace("A:", ""), out _currentSensorData.LidarAngle);
                        else if (cleanPart.StartsWith("D:"))
                            float.TryParse(cleanPart.Replace("D:", ""), out _currentSensorData.LidarDistance);
                        else if (cleanPart.StartsWith("Q:"))
                            int.TryParse(cleanPart.Replace("Q:", ""), out _currentSensorData.LidarQuality);
                    }
                }

                _currentSensorData.LastUpdate = DateTime.Now;
                OnSensorDataReceived?.Invoke(CurrentSensorData);
            }
            catch (Exception ex)
            {
                OnStatusChanged?.Invoke($"数据解析错误: {ex.Message}");
            }
        }

        /// <summary>
        /// 处理连接状态变化
        /// </summary>
        private void HandleConnectionStatus(BluetoothState state, string message)
        {
            bool wasConnected = isConnected;
            isConnected = (state == BluetoothState.Connected);
            
            if (wasConnected != isConnected)
            {
                OnConnectionChanged?.Invoke(isConnected);
                if (isConnected)
                {
                    OnStatusChanged?.Invoke("小车已连接");
                }
                else
                {
                    OnStatusChanged?.Invoke("小车已断开");
                    currentDirection = DIR_STOP;
                }
            }
        }

        /// <summary>
        /// 发送运动控制命令
        /// </summary>
        public void SendMovementCommand(char direction)
        {
            if (!isConnected)
            {
                OnStatusChanged?.Invoke("请先连接小车");
                return;
            }

            if (IsValidDirection(direction))
            {
                byte[] command = Encoding.UTF8.GetBytes(direction.ToString());
                bluetoothManager.Send(command);
                currentDirection = direction;
                OnStatusChanged?.Invoke($"发送命令: {GetDirectionName(direction)}");
            }
            else
            {
                OnStatusChanged?.Invoke("无效的控制命令");
            }
        }

        /// <summary>
        /// 停止小车
        /// </summary>
        public void Stop()
        {
            SendMovementCommand(DIR_STOP);
        }

        /// <summary>
        /// 前进
        /// </summary>
        public void MoveForward()
        {
            SendMovementCommand(DIR_FORWARD);
        }

        /// <summary>
        /// 后退
        /// </summary>
        public void MoveBackward()
        {
            SendMovementCommand(DIR_BACKWARD);
        }

        /// <summary>
        /// 左转
        /// </summary>
        public void TurnLeft()
        {
            SendMovementCommand(DIR_LEFT);
        }

        /// <summary>
        /// 右转
        /// </summary>
        public void TurnRight()
        {
            SendMovementCommand(DIR_RIGHT);
        }

        /// <summary>
        /// 掉头
        /// </summary>
        public void UTurn()
        {
            SendMovementCommand(DIR_UTURN);
        }

        /// <summary>
        /// 验证方向命令是否有效
        /// </summary>
        private bool IsValidDirection(char direction)
        {
            return direction >= '0' && direction <= '5';
        }

        /// <summary>
        /// 获取方向名称
        /// </summary>
        private string GetDirectionName(char direction)
        {
            switch (direction)
            {
                case DIR_STOP: return "停止";
                case DIR_FORWARD: return "前进";
                case DIR_BACKWARD: return "后退";
                case DIR_LEFT: return "左转";
                case DIR_RIGHT: return "右转";
                case DIR_UTURN: return "掉头";
                default: return "未知";
            }
        }

        /// <summary>
        /// 获取当前方向
        /// </summary>
        public char GetCurrentDirection()
        {
            return currentDirection;
        }

        /// <summary>
        /// 检查是否已连接
        /// </summary>
        public bool IsConnected()
        {
            return isConnected;
        }
    }
}
