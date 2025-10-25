using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Linq;

namespace BluetoothApp
{
    /// <summary>
    /// Python SLAM接口 - 跨代码调用PoseGraph_Slam-Simulation
    /// </summary>
    public class PythonSLAMInterface : IDisposable
    {
        private Process pythonProcess;
        private string pythonScriptPath;
        private string dataDirectory;
        private bool isInitialized = false;
        private readonly object lockObject = new object();

        public event EventHandler<SLAMResultEventArgs> SLAMResultReceived;
        public event EventHandler<string> ErrorOccurred;

        public PythonSLAMInterface()
        {
            // 设置Python脚本路径
            pythonScriptPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "PoseGraph_Slam-Simulation", "maze_slam_simulation.py");
            dataDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "SLAMData");
            
            // 确保数据目录存在
            if (!Directory.Exists(dataDirectory))
            {
                Directory.CreateDirectory(dataDirectory);
            }
        }

        /// <summary>
        /// 初始化Python SLAM环境
        /// </summary>
        public async Task<bool> InitializeAsync()
        {
            try
            {
                // 检查Python脚本是否存在
                if (!File.Exists(pythonScriptPath))
                {
                    OnErrorOccurred("Python SLAM脚本不存在: " + pythonScriptPath);
                    return false;
                }

                // 创建Python进程
                pythonProcess = new Process
                {
                    StartInfo = new ProcessStartInfo
                    {
                        FileName = "python",
                        Arguments = $"\"{pythonScriptPath}\" --mode=interface",
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        RedirectStandardInput = true,
                        CreateNoWindow = true,
                        WorkingDirectory = Path.GetDirectoryName(pythonScriptPath)
                    }
                };

                // 设置事件处理
                pythonProcess.OutputDataReceived += OnPythonOutputReceived;
                pythonProcess.ErrorDataReceived += OnPythonErrorReceived;
                pythonProcess.Exited += OnPythonProcessExited;

                // 启动进程
                pythonProcess.Start();
                pythonProcess.BeginOutputReadLine();
                pythonProcess.BeginErrorReadLine();

                // 等待初始化完成
                await Task.Delay(2000);
                
                isInitialized = true;
                return true;
            }
            catch (Exception ex)
            {
                OnErrorOccurred($"初始化Python SLAM失败: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// 发送机器人传感器数据到Python SLAM
        /// </summary>
        public async Task<bool> SendSensorDataAsync(RobotSensorData sensorData)
        {
            if (!isInitialized || pythonProcess?.HasExited == true)
            {
                return false;
            }

            try
            {
                // 创建数据包
                var dataPacket = new
                {
                    timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"),
                    robot_pose = new
                    {
                        x = sensorData.X,
                        y = sensorData.Y,
                        theta = sensorData.Theta
                    },
                    laser_data = sensorData.LaserData?.Select(p => new
                    {
                        angle = p.Angle,
                        distance = p.Distance,
                        x = p.X,
                        y = p.Y,
                        quality = p.Quality
                    }).ToArray(),
                    odometry = new
                    {
                        left_rpm = sensorData.LeftRPM,
                        right_rpm = sensorData.RightRPM,
                        velocity = sensorData.Velocity
                    }
                };

                // 发送JSON数据到Python
                string jsonData = Newtonsoft.Json.JsonConvert.SerializeObject(dataPacket);
                await pythonProcess.StandardInput.WriteLineAsync(jsonData);
                await pythonProcess.StandardInput.FlushAsync();

                return true;
            }
            catch (Exception ex)
            {
                OnErrorOccurred($"发送传感器数据失败: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// 请求SLAM结果
        /// </summary>
        public async Task<SLAMResult> RequestSLAMResultAsync()
        {
            if (!isInitialized || pythonProcess?.HasExited == true)
            {
                return null;
            }

            try
            {
                // 发送请求命令
                await pythonProcess.StandardInput.WriteLineAsync("GET_SLAM_RESULT");
                await pythonProcess.StandardInput.FlushAsync();

                // 等待结果（这里简化处理，实际应该通过事件机制）
                await Task.Delay(100);
                return new SLAMResult(); // 实际应该从Python返回的数据中解析
            }
            catch (Exception ex)
            {
                OnErrorOccurred($"请求SLAM结果失败: {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// 保存SLAM数据到文件
        /// </summary>
        public async Task<bool> SaveSLAMDataAsync(SLAMResult slamResult)
        {
            try
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string fileName = Path.Combine(dataDirectory, $"slam_result_{timestamp}.json");
                
                var dataToSave = new
                {
                    timestamp = DateTime.Now,
                    map_points = slamResult.MapPoints,
                    trajectory = slamResult.Trajectory,
                    optimized_poses = slamResult.OptimizedPoses,
                    statistics = slamResult.Statistics
                };

                string jsonData = Newtonsoft.Json.JsonConvert.SerializeObject(dataToSave, Newtonsoft.Json.Formatting.Indented);
                File.WriteAllText(fileName, jsonData);
                
                return true;
            }
            catch (Exception ex)
            {
                OnErrorOccurred($"保存SLAM数据失败: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// 处理Python输出
        /// </summary>
        private void OnPythonOutputReceived(object sender, DataReceivedEventArgs e)
        {
            if (string.IsNullOrEmpty(e.Data)) return;

            try
            {
                // 解析Python输出
                if (e.Data.StartsWith("SLAM_RESULT:"))
                {
                    // 解析SLAM结果
                    string jsonData = e.Data.Substring("SLAM_RESULT:".Length);
                    var slamResult = Newtonsoft.Json.JsonConvert.DeserializeObject<SLAMResult>(jsonData);
                    OnSLAMResultReceived(new SLAMResultEventArgs(slamResult));
                }
                else if (e.Data.StartsWith("ERROR:"))
                {
                    OnErrorOccurred(e.Data.Substring("ERROR:".Length));
                }
            }
            catch (Exception ex)
            {
                OnErrorOccurred($"解析Python输出失败: {ex.Message}");
            }
        }

        /// <summary>
        /// 处理Python错误输出
        /// </summary>
        private void OnPythonErrorReceived(object sender, DataReceivedEventArgs e)
        {
            if (!string.IsNullOrEmpty(e.Data))
            {
                OnErrorOccurred($"Python错误: {e.Data}");
            }
        }

        /// <summary>
        /// 处理Python进程退出
        /// </summary>
        private void OnPythonProcessExited(object sender, EventArgs e)
        {
            isInitialized = false;
            OnErrorOccurred("Python SLAM进程已退出");
        }

        /// <summary>
        /// 触发SLAM结果事件
        /// </summary>
        protected virtual void OnSLAMResultReceived(SLAMResultEventArgs e)
        {
            SLAMResultReceived?.Invoke(this, e);
        }

        /// <summary>
        /// 触发错误事件
        /// </summary>
        protected virtual void OnErrorOccurred(string error)
        {
            ErrorOccurred?.Invoke(this, error);
        }

        public void Dispose()
        {
            try
            {
                if (pythonProcess != null && !pythonProcess.HasExited)
                {
                    pythonProcess.Kill();
                }
                pythonProcess?.Dispose();
                isInitialized = false;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"释放Python SLAM接口失败: {ex.Message}");
            }
        }
    }

    /// <summary>
    /// 机器人传感器数据结构
    /// </summary>
    public class RobotSensorData
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float Theta { get; set; }
        public float LeftRPM { get; set; }
        public float RightRPM { get; set; }
        public float Velocity { get; set; }
        public List<LaserScanData> LaserData { get; set; } = new List<LaserScanData>();
        public DateTime Timestamp { get; set; } = DateTime.Now;
    }

    /// <summary>
    /// SLAM结果数据结构
    /// </summary>
    public class SLAMResult
    {
        public List<float[]> MapPoints { get; set; } = new List<float[]>();
        public List<float[]> Trajectory { get; set; } = new List<float[]>();
        public List<float[]> OptimizedPoses { get; set; } = new List<float[]>();
        public Dictionary<string, object> Statistics { get; set; } = new Dictionary<string, object>();
        public DateTime Timestamp { get; set; } = DateTime.Now;
    }

    /// <summary>
    /// SLAM结果事件参数
    /// </summary>
    public class SLAMResultEventArgs : EventArgs
    {
        public SLAMResult SLAMResult { get; }

        public SLAMResultEventArgs(SLAMResult slamResult)
        {
            SLAMResult = slamResult;
        }
    }
}
