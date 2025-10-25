using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using System.IO;

namespace BluetoothApp
{
    /// <summary>
    /// 集成SLAM控制器 - 结合PoseGraph_Slam-Simulation算法和JSON数据格式
    /// </summary>
    public class IntegratedSLAMController
    {
        #region 数据结构定义（基于4.json格式）
        public class Segment
        {
            public float[] Start { get; set; }
            public float[] End { get; set; }
        }

        public class MazeData
        {
            public List<Segment> Segments { get; set; }
            public float[] StartPoint { get; set; }
            public float[] GoalPoint { get; set; }
        }

        #endregion

        #region 事件定义
        public event EventHandler<string> OnStatusChanged;
        public event EventHandler<RobotState> OnRobotStateChanged;
        public event EventHandler<List<LaserScanData>> OnLaserDataReceived;
        public event EventHandler<SLAMMapResult> OnMapUpdated;
        public event EventHandler<Exception> OnError;
        #endregion

        #region 私有字段
        private BluetoothWindow mainForm;
        private AutoRobotController autoController;
        private PoseGraphSLAM poseGraphSLAM;
        private System.Timers.Timer slamUpdateTimer;
        private System.Timers.Timer controlTimer;
        
        private bool isRunning = false;
        private bool isPaused = false;
        private CancellationTokenSource cancellationTokenSource;
        private object lockObject = new object();
        
        // SLAM相关
        private RobotState currentRobotState;
        private List<LaserScanData> currentLaserData;
        private List<RobotState> robotTrajectory;
        private List<LaserScanData> allLaserData;
        private MazeData mazeData;
        
        // 控制参数
        private float explorationSpeed = 0.5f; // m/s
        private float rotationSpeed = 1.0f; // rad/s
        private float laserRange = 5.0f; // 激光雷达范围
        private int controlInterval = 100; // 控制间隔(ms)
        private int slamUpdateInterval = 200; // SLAM更新间隔(ms)
        
        // 探索状态
        private bool explorationComplete = false;
        private List<PointF> explorationPath;
        private int currentPathIndex = 0;
        #endregion

        #region 构造函数
        public IntegratedSLAMController(BluetoothWindow form, AutoRobotController autoCtrl)
        {
            mainForm = form;
            autoController = autoCtrl;
            poseGraphSLAM = new PoseGraphSLAM();
            
            currentRobotState = new RobotState { X = 0, Y = 0, Theta = 0, Velocity = 0 };
            currentLaserData = new List<LaserScanData>();
            robotTrajectory = new List<RobotState>();
            allLaserData = new List<LaserScanData>();
            
            // 初始化定时器
            controlTimer = new System.Timers.Timer(controlInterval);
            controlTimer.Elapsed += ControlTimer_Elapsed;
            controlTimer.AutoReset = true;
            
            slamUpdateTimer = new System.Timers.Timer(slamUpdateInterval);
            slamUpdateTimer.Elapsed += SLAMUpdateTimer_Elapsed;
            slamUpdateTimer.AutoReset = true;
        }
        #endregion

        #region 公共方法
        /// <summary>
        /// 加载迷宫数据
        /// </summary>
        public void LoadMazeData()
        {
            try
            {
                // 直接加载4.json数据
                mazeData = Load4JsonData();
                
                // 设置起始位置
                if (mazeData.StartPoint != null && mazeData.StartPoint.Length >= 2)
                {
                    currentRobotState.X = mazeData.StartPoint[0];
                    currentRobotState.Y = mazeData.StartPoint[1];
                }
                
                // 生成探索路径
                GenerateExplorationPath();
                
                OnStatusChanged?.Invoke(this, $"4.json迷宫数据已加载: {mazeData.Segments.Count} 个墙壁段");
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        /// <summary>
        /// 加载4.json数据
        /// </summary>
        private MazeData Load4JsonData()
        {
            var mazeData = new MazeData
            {
                Segments = new List<Segment>(),
                StartPoint = new float[2],
                GoalPoint = new float[2]
            };

            // 加载4.json的完整数据
            mazeData.StartPoint[0] = 2.0f;
            mazeData.StartPoint[1] = 14.0f;
            mazeData.GoalPoint[0] = 14.0f;
            mazeData.GoalPoint[1] = 10.0f;

            // 添加4.json中的所有墙壁段
            mazeData.Segments.Add(new Segment { Start = new float[] { 0, 0 }, End = new float[] { 0, 16 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 0, 16 }, End = new float[] { 16, 16 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 0, 0 }, End = new float[] { 16, 0 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 16, 0 }, End = new float[] { 16, 16 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 0, 12 }, End = new float[] { 4, 12 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 4, 4 }, End = new float[] { 4, 8 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 4, 8 }, End = new float[] { 8, 8 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 8, 0 }, End = new float[] { 8, 4 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 8, 8 }, End = new float[] { 8, 12 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 8, 12 }, End = new float[] { 16, 12 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 12, 4 }, End = new float[] { 12, 12 } });

            return mazeData;
        }

        /// <summary>
        /// 简单的JSON解析方法
        /// </summary>
        private MazeData ParseMazeData(string jsonContent)
        {
            var mazeData = new MazeData
            {
                Segments = new List<Segment>(),
                StartPoint = new float[2],
                GoalPoint = new float[2]
            };

            // 简单的JSON解析 - 这里使用硬编码的4.json数据
            // 在实际应用中，可以使用更复杂的JSON解析
            mazeData.StartPoint[0] = 2.0f;
            mazeData.StartPoint[1] = 14.0f;
            mazeData.GoalPoint[0] = 14.0f;
            mazeData.GoalPoint[1] = 10.0f;

            // 添加一些基本的墙壁段
            mazeData.Segments.Add(new Segment { Start = new float[] { 0, 0 }, End = new float[] { 0, 16 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 0, 16 }, End = new float[] { 16, 16 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 0, 0 }, End = new float[] { 16, 0 } });
            mazeData.Segments.Add(new Segment { Start = new float[] { 16, 0 }, End = new float[] { 16, 16 } });

            return mazeData;
        }

        /// <summary>
        /// 开始集成SLAM控制
        /// </summary>
        public void StartIntegratedSLAM()
        {
            if (isRunning)
            {
                OnStatusChanged?.Invoke(this, "集成SLAM已在运行中");
                return;
            }

            // 自动加载4.json数据
            if (mazeData == null)
            {
                LoadMazeData();
            }

            if (mazeData == null)
            {
                OnStatusChanged?.Invoke(this, "错误：无法加载4.json迷宫数据");
                OnError?.Invoke(this, new Exception("4.json迷宫数据加载失败"));
                return;
            }

            isRunning = true;
            isPaused = false;
            cancellationTokenSource = new CancellationTokenSource();
            
            // 启动自动控制
            autoController.StartAutoControl();
            
            // 启动控制定时器
            controlTimer.Start();
            slamUpdateTimer.Start();
            
            OnStatusChanged?.Invoke(this, "集成SLAM已启动，使用4.json迷宫数据");
        }

        /// <summary>
        /// 停止集成SLAM控制
        /// </summary>
        public void StopIntegratedSLAM()
        {
            if (!isRunning) return;

            isRunning = false;
            isPaused = false;
            
            controlTimer.Stop();
            slamUpdateTimer.Stop();
            autoController.StopAutoControl();
            
            cancellationTokenSource?.Cancel();
            
            OnStatusChanged?.Invoke(this, "集成SLAM已停止");
        }

        /// <summary>
        /// 暂停/恢复集成SLAM控制
        /// </summary>
        public void TogglePause()
        {
            if (!isRunning) return;

            isPaused = !isPaused;
            autoController.TogglePause();
            
            string status = isPaused ? "集成SLAM已暂停" : "集成SLAM已恢复";
            OnStatusChanged?.Invoke(this, status);
        }

        /// <summary>
        /// 更新激光扫描数据
        /// </summary>
        public void UpdateLaserData(List<LaserScanData> newLaserData)
        {
            lock (lockObject)
            {
                currentLaserData.Clear();
                currentLaserData.AddRange(newLaserData);
                allLaserData.AddRange(newLaserData);
            }
            
            OnLaserDataReceived?.Invoke(this, currentLaserData);
        }

        /// <summary>
        /// 更新机器人状态
        /// </summary>
        public void UpdateRobotState(RobotState newState)
        {
            lock (lockObject)
            {
                currentRobotState = newState;
                currentRobotState.Timestamp = DateTime.Now;
                robotTrajectory.Add(new RobotState
                {
                    X = newState.X,
                    Y = newState.Y,
                    Theta = newState.Theta,
                    Velocity = newState.Velocity,
                    Timestamp = newState.Timestamp
                });
            }
            
            OnRobotStateChanged?.Invoke(this, currentRobotState);
        }
        #endregion

        #region 私有方法
        private void ControlTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            if (!isRunning || isPaused) return;

            try
            {
                ExecuteExplorationControl();
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
                UpdateSLAM();
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        private void ExecuteExplorationControl()
        {
            if (explorationPath == null || explorationPath.Count == 0)
            {
                OnStatusChanged?.Invoke(this, "探索路径为空");
                return;
            }

            if (currentPathIndex >= explorationPath.Count)
            {
                explorationComplete = true;
                OnStatusChanged?.Invoke(this, "探索完成");
                return;
            }

            // 获取下一个目标点
            var targetPoint = explorationPath[currentPathIndex];
            var currentPoint = new PointF(currentRobotState.X, currentRobotState.Y);
            
            // 计算到目标点的距离和角度
            float distance = (float)Math.Sqrt(Math.Pow(targetPoint.X - currentPoint.X, 2) + 
                                            Math.Pow(targetPoint.Y - currentPoint.Y, 2));
            float targetAngle = (float)Math.Atan2(targetPoint.Y - currentPoint.Y, 
                                                targetPoint.X - currentPoint.X);
            
            // 真实小车运动控制 - 280cm x 280cm场景，每4个格子70cm
            ExecuteRealRobotControl(targetPoint, distance, targetAngle);
        }

        /// <summary>
        /// 执行真实小车运动控制
        /// </summary>
        private void ExecuteRealRobotControl(PointF targetPoint, float distance, float targetAngle)
        {
            // 角度差
            float angleDiff = targetAngle - currentRobotState.Theta;
            while (angleDiff > (float)Math.PI) angleDiff -= 2 * (float)Math.PI;
            while (angleDiff < -(float)Math.PI) angleDiff += 2 * (float)Math.PI;
            
            // 真实场景控制参数：280cm x 280cm，每4个格子70cm
            // 每个格子 = 70cm / 4 = 17.5cm
            // 缓慢移动，小步长
            float gridSize = 0.175f; // 17.5cm = 0.175m
            float minDistance = gridSize * 0.5f; // 半个格子
            float maxDistance = gridSize * 2.0f; // 两个格子
            
            // 执行控制
            if (Math.Abs(angleDiff) > 0.1f) // 需要转向
            {
                if (angleDiff > 0)
                {
                    // 左转 - 缓慢转向
                    autoController.AddCommand(AutoRobotController.DIR_LEFT, 200);
                }
                else
                {
                    // 右转 - 缓慢转向
                    autoController.AddCommand(AutoRobotController.DIR_RIGHT, 200);
                }
            }
            else if (distance > minDistance) // 需要前进
            {
                // 前进 - 限制最大距离，缓慢移动
                float moveDistance = Math.Min(distance, maxDistance);
                int duration = Math.Min(500, (int)(moveDistance * 2000)); // 缓慢移动
                autoController.AddCommand(AutoRobotController.DIR_FORWARD, duration);
                
                // 更新机器人位置（模拟）
                UpdateRobotPosition(moveDistance, targetAngle);
            }
            else
            {
                // 到达目标点，移动到下一个点
                currentPathIndex++;
                OnStatusChanged?.Invoke(this, $"到达路径点 {currentPathIndex}/{explorationPath.Count}");
            }
        }

        /// <summary>
        /// 更新机器人位置（模拟真实运动）
        /// </summary>
        private void UpdateRobotPosition(float distance, float angle)
        {
            // 更新机器人位置
            currentRobotState.X += (float)Math.Cos(angle) * distance;
            currentRobotState.Y += (float)Math.Sin(angle) * distance;
            currentRobotState.Theta = angle;
            
            // 添加到轨迹
            robotTrajectory.Add(new RobotState
            {
                X = currentRobotState.X,
                Y = currentRobotState.Y,
                Theta = currentRobotState.Theta,
                Velocity = 0.1f, // 缓慢速度
                Timestamp = DateTime.Now
            });
        }

        private void UpdateSLAM()
        {
            lock (lockObject)
            {
                if (currentLaserData.Count > 0)
                {
                    // 添加位姿到位姿图
                    var pose = new float[] { currentRobotState.X, currentRobotState.Y, currentRobotState.Theta };
                    var points = currentLaserData.Select(p => new PointF(p.X, p.Y)).ToList();
                    
                    int nodeId = poseGraphSLAM.AddPose(currentRobotState.X, currentRobotState.Y, currentRobotState.Theta, points);
                    
                    // 添加里程计边
                    if (robotTrajectory.Count > 1)
                    {
                        var prevPose = robotTrajectory[robotTrajectory.Count - 2];
                        var dx = currentRobotState.X - prevPose.X;
                        var dy = currentRobotState.Y - prevPose.Y;
                        var dtheta = currentRobotState.Theta - prevPose.Theta;
                        
                        poseGraphSLAM.AddEdge(nodeId - 1, nodeId, dx, dy, dtheta, 1.0f, PoseGraphSLAM.EdgeType.Odometry);
                    }
                    
                    // 执行位姿图优化
                    poseGraphSLAM.OptimizeGraph();
                    
                    // 创建地图结果
                    var optimizedTrajectory = poseGraphSLAM.GetOptimizedTrajectory();
                    var globalMapPoints = poseGraphSLAM.GetGlobalMapPoints();
                    
                    // 转换位姿数据
                    var robotPoses = new List<float[]>();
                    foreach (var point in optimizedTrajectory)
                    {
                        robotPoses.Add(new float[] { point.X, point.Y, 0.0f });
                    }
                    
                    // 转换地图点数据
                    var mapPoints = new List<float[]>();
                    foreach (var point in globalMapPoints)
                    {
                        mapPoints.Add(new float[] { point.X, point.Y });
                    }
                    
                    var mapResult = new SLAMMapResult
                    {
                        RobotPoses = robotPoses,
                        MapPoints = mapPoints,
                        Trajectory = robotTrajectory.ToList(),
                        LaserData = allLaserData.ToList(),
                        Timestamp = DateTime.Now
                    };
                    
                    OnMapUpdated?.Invoke(this, mapResult);
                }
            }
        }

        private void GenerateExplorationPath()
        {
            explorationPath = new List<PointF>();
            
            if (mazeData == null) return;
            
            // 基于迷宫数据生成探索路径
            // 这里使用简单的网格探索策略
            float gridSize = 1.0f;
            float startX = mazeData.StartPoint[0];
            float startY = mazeData.StartPoint[1];
            
            // 生成螺旋式探索路径
            for (int ring = 0; ring < 8; ring++)
            {
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < ring * 2 + 1; j++)
                    {
                        float x = startX + (ring + 1) * gridSize * (i == 0 ? 1 : (i == 2 ? -1 : 0));
                        float y = startY + (ring + 1) * gridSize * (i == 1 ? 1 : (i == 3 ? -1 : 0));
                        
                        if (x >= 0 && x < 20 && y >= 0 && y < 20) // 边界检查
                        {
                            explorationPath.Add(new PointF(x, y));
                        }
                    }
                }
            }
            
            OnStatusChanged?.Invoke(this, $"生成探索路径: {explorationPath.Count} 个点");
        }
        #endregion

        #region 属性
        public bool IsRunning => isRunning;
        public bool IsPaused => isPaused;
        public RobotState CurrentRobotState => currentRobotState;
        public List<RobotState> RobotTrajectory => robotTrajectory;
        public List<LaserScanData> AllLaserData => allLaserData;
        public bool ExplorationComplete => explorationComplete;
        #endregion

        #region 资源清理
        public void Dispose()
        {
            StopIntegratedSLAM();
            controlTimer?.Dispose();
            slamUpdateTimer?.Dispose();
            cancellationTokenSource?.Dispose();
        }
        #endregion
    }

    /// <summary>
    /// SLAM地图结果
    /// </summary>
    public class SLAMMapResult
    {
        public List<float[]> RobotPoses { get; set; }
        public List<float[]> MapPoints { get; set; }
        public List<RobotState> Trajectory { get; set; }
        public List<LaserScanData> LaserData { get; set; }
        public DateTime Timestamp { get; set; }
        
        public SLAMMapResult()
        {
            RobotPoses = new List<float[]>();
            MapPoints = new List<float[]>();
            Trajectory = new List<RobotState>();
            LaserData = new List<LaserScanData>();
            Timestamp = DateTime.Now;
        }
    }

    /// <summary>
    /// 机器人状态
    /// </summary>
    public class RobotState
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float Theta { get; set; }
        public float Velocity { get; set; }
        public DateTime Timestamp { get; set; }
    }

    /// <summary>
    /// 激光扫描数据
    /// </summary>
    public class LaserScanData
    {
        public float Angle { get; set; }
        public float Distance { get; set; }
        public float X { get; set; }
        public float Y { get; set; }
        public int Quality { get; set; }
        public DateTime Timestamp { get; set; }
    }
}
