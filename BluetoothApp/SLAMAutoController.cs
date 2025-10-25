using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;

namespace BluetoothApp
{
    /// <summary>
    /// SLAM自动控制器 - 集成SLAM建图和自动控制
    /// </summary>
    public class SLAMAutoController
    {
        #region 事件定义
        public event EventHandler<string> OnStatusChanged;
        public event EventHandler<SLAMMapData> OnMapUpdated;
        public event EventHandler<RobotPose> OnPoseUpdated;
        public event EventHandler<Exception> OnError;
        #endregion

        #region 私有字段
        private BluetoothWindow mainForm;
        private AutoRobotController autoController;
        private AdvancedMazeExplorationController slamController;
        private System.Timers.Timer slamUpdateTimer;
        private bool isRunning = false;
        private bool isPaused = false;
        private CancellationTokenSource cancellationTokenSource;
        private object lockObject = new object();
        
        // SLAM相关
        private RobotPose currentPose;
        private List<LaserScanPoint> laserData;
        private AdvancedSLAMMap slamMap;
        private PoseGraphSLAM poseGraph;
        #endregion

        #region 构造函数
        public SLAMAutoController(BluetoothWindow form, AutoRobotController autoCtrl, AdvancedMazeExplorationController slamCtrl)
        {
            mainForm = form;
            autoController = autoCtrl;
            slamController = slamCtrl;
            
            laserData = new List<LaserScanPoint>();
            currentPose = new RobotPose { X = 0, Y = 0, Theta = 0 };
            
            // 初始化SLAM地图
            InitializeSLAMMap();
            
            // 初始化定时器
            slamUpdateTimer = new System.Timers.Timer(100); // 100ms更新间隔
            slamUpdateTimer.Elapsed += SLAMUpdateTimer_Elapsed;
            slamUpdateTimer.AutoReset = true;
        }
        #endregion

        #region 公共方法
        /// <summary>
        /// 开始SLAM自动控制
        /// </summary>
        public void StartSLAMAutoControl()
        {
            if (isRunning)
            {
                OnStatusChanged?.Invoke(this, "SLAM自动控制已在运行中");
                return;
            }

            if (autoController == null || slamController == null)
            {
                OnStatusChanged?.Invoke(this, "错误：控制器未初始化");
                OnError?.Invoke(this, new Exception("控制器未初始化"));
                return;
            }

            isRunning = true;
            isPaused = false;
            cancellationTokenSource = new CancellationTokenSource();
            
            // 启动自动控制
            autoController.StartAutoControl();
            
            // 启动SLAM更新定时器
            slamUpdateTimer.Start();
            
            OnStatusChanged?.Invoke(this, "SLAM自动控制已启动");
        }

        /// <summary>
        /// 停止SLAM自动控制
        /// </summary>
        public void StopSLAMAutoControl()
        {
            if (!isRunning) return;

            isRunning = false;
            isPaused = false;
            
            // 停止自动控制
            autoController.StopAutoControl();
            
            // 停止SLAM更新定时器
            slamUpdateTimer.Stop();
            
            cancellationTokenSource?.Cancel();
            
            OnStatusChanged?.Invoke(this, "SLAM自动控制已停止");
        }

        /// <summary>
        /// 暂停/恢复SLAM自动控制
        /// </summary>
        public void TogglePause()
        {
            if (!isRunning) return;

            isPaused = !isPaused;
            autoController.TogglePause();
            
            string status = isPaused ? "SLAM自动控制已暂停" : "SLAM自动控制已恢复";
            OnStatusChanged?.Invoke(this, status);
        }

        /// <summary>
        /// 执行SLAM探索序列
        /// </summary>
        public void ExecuteSLAMExplorationSequence()
        {
            if (!isRunning) return;

            try
            {
                // 生成SLAM探索命令序列
                var slamCommands = GenerateSLAMExplorationCommands();
                autoController.AddCommandSequence(slamCommands);
                
                OnStatusChanged?.Invoke(this, "开始执行SLAM探索序列");
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        /// <summary>
        /// 执行迷宫探索序列
        /// </summary>
        public void ExecuteMazeExplorationSequence()
        {
            if (!isRunning) return;

            try
            {
                // 生成迷宫探索命令序列
                var mazeCommands = GenerateMazeExplorationCommands();
                autoController.AddCommandSequence(mazeCommands);
                
                OnStatusChanged?.Invoke(this, "开始执行迷宫探索序列");
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        /// <summary>
        /// 更新激光扫描数据
        /// </summary>
        public void UpdateLaserData(List<LaserScanPoint> newLaserData)
        {
            lock (lockObject)
            {
                laserData.Clear();
                laserData.AddRange(newLaserData);
            }
        }

        /// <summary>
        /// 更新机器人位姿
        /// </summary>
        public void UpdateRobotPose(RobotPose newPose)
        {
            lock (lockObject)
            {
                currentPose = newPose;
            }
            
            OnPoseUpdated?.Invoke(this, currentPose);
        }
        #endregion

        #region 私有方法
        private void InitializeSLAMMap()
        {
            try
            {
                // 初始化SLAM地图配置
                var mapConfig = new AdvancedSLAMMap.MapConfig
                {
                    Resolution = 0.05f,
                    Width = 20.0f,
                    Height = 20.0f,
                    Origin = new System.Drawing.PointF(10.0f, 10.0f),
                    MaxRange = 5.0f,
                    HitProbability = 0.7f,
                    MissProbability = 0.3f
                };

                slamMap = new AdvancedSLAMMap(mapConfig, AdvancedSLAMMap.MapType.Hybrid);
                poseGraph = slamMap.GetPoseGraphSLAM();
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        private void SLAMUpdateTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            if (!isRunning || isPaused) return;

            try
            {
                // 执行SLAM更新
                UpdateSLAM();
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        private void UpdateSLAM()
        {
            lock (lockObject)
            {
                // 更新SLAM地图
                if (laserData.Count > 0)
                {
                    // 转换激光数据格式
                    var lidarPoints = laserData.Select(p => new System.Drawing.PointF(p.X, p.Y)).ToList();
                    var robotPose = new System.Drawing.PointF(currentPose.X, currentPose.Y);
                    
                    slamMap.UpdateMap(robotPose, currentPose.Theta, lidarPoints);
                    
                    // 执行位姿图优化
                    poseGraph.OptimizeGraph();
                    
                    // 创建地图数据
                    var occupancyGrid = slamMap.GetOccupancyGrid();
                    var floatGrid = new float[occupancyGrid.GetLength(0), occupancyGrid.GetLength(1)];
                    for (int i = 0; i < occupancyGrid.GetLength(0); i++)
                    {
                        for (int j = 0; j < occupancyGrid.GetLength(1); j++)
                        {
                            floatGrid[i, j] = occupancyGrid[i, j];
                        }
                    }
                    
                    var mapData = new SLAMMapData
                    {
                        OccupancyGrid = floatGrid,
                        Trajectory = slamMap.GetRobotTrajectory().Select(p => new RobotPose { X = p.X, Y = p.Y, Theta = 0 }).ToList(),
                        ScanPoints = laserData,
                        Timestamp = DateTime.Now
                    };
                    OnMapUpdated?.Invoke(this, mapData);
                }
            }
        }

        private List<RobotCommand> GenerateSLAMExplorationCommands()
        {
            var commands = new List<RobotCommand>();
            
            // SLAM探索的系统化模式：螺旋式扫描
            for (int i = 0; i < 8; i++) // 8个方向扫描
            {
                // 前进一小段距离
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 800 });
                
                // 左转45度进行扫描
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_LEFT, Duration = 500 });
                
                // 短暂停止以收集激光数据
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_STOP, Duration = 200 });
                
                // 右转90度进行反向扫描
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_RIGHT, Duration = 1000 });
                
                // 再次停止收集数据
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_STOP, Duration = 200 });
                
                // 回到原方向
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_LEFT, Duration = 500 });
            }
            
            // 完成扫描后停止
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_STOP, Duration = 1000 });

            return commands;
        }

        private List<RobotCommand> GenerateMazeExplorationCommands()
        {
            var commands = new List<RobotCommand>();
            
            // 迷宫探索的墙跟随模式
            for (int i = 0; i < 12; i++) // 12个探索步骤
            {
                // 前进直到遇到障碍物
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 1500 });
                
                // 短暂停止检测
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_STOP, Duration = 300 });
                
                // 左转90度
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_LEFT, Duration = 1000 });
                
                // 前进一小段
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 500 });
                
                // 右转90度（回到原方向）
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_RIGHT, Duration = 1000 });
            }
            
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_STOP, Duration = 1000 });

            return commands;
        }
        #endregion

        #region 属性
        public bool IsRunning => isRunning;
        public bool IsPaused => isPaused;
        public RobotPose CurrentPose => currentPose;
        public AdvancedSLAMMap SLAMMap => slamMap;
        public PoseGraphSLAM PoseGraph => poseGraph;
        #endregion

        #region 资源清理
        public void Dispose()
        {
            StopSLAMAutoControl();
            slamUpdateTimer?.Dispose();
            cancellationTokenSource?.Dispose();
        }
        #endregion
    }

    /// <summary>
    /// 机器人位姿
    /// </summary>
    public class RobotPose
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float Theta { get; set; }
        
        public override string ToString()
        {
            return $"Pose: ({X:F2}, {Y:F2}, {Theta:F2}°)";
        }
    }

    /// <summary>
    /// 激光扫描点
    /// </summary>
    public class LaserScanPoint
    {
        public float Angle { get; set; }
        public float Distance { get; set; }
        public float X { get; set; }
        public float Y { get; set; }
        public int Quality { get; set; }
    }

    /// <summary>
    /// SLAM地图数据
    /// </summary>
    public class SLAMMapData
    {
        public float[,] OccupancyGrid { get; set; }
        public List<RobotPose> Trajectory { get; set; }
        public List<LaserScanPoint> ScanPoints { get; set; }
        public DateTime Timestamp { get; set; }
        
        public SLAMMapData()
        {
            Trajectory = new List<RobotPose>();
            ScanPoints = new List<LaserScanPoint>();
            Timestamp = DateTime.Now;
        }
    }
}
