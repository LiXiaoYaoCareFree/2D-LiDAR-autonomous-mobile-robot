using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace BluetoothApp
{
    /// <summary>
    /// 高级迷宫探索控制器
    /// 集成位姿图SLAM和高级路径规划
    /// </summary>
    public class AdvancedMazeExplorationController
    {
        /// <summary>
        /// 探索状态枚举
        /// </summary>
        public enum ExplorationState
        {
            Initialization,     // 初始化
            Exploration,        // 探索阶段
            SearchGoal,         // 搜索目标
            NavigateToGoal,     // 导航到目标
            PathPlanning,       // 路径规划
            Navigation,         // 导航阶段
            Completed          // 完成
        }

        /// <summary>
        /// 探索策略枚举
        /// </summary>
        public enum ExplorationStrategy
        {
            FrontierBased,      // 基于边界的探索
            RandomWalk,         // 随机游走
            WallFollowing,      // 沿墙行走
            Hybrid             // 混合策略
        }

        // 探索参数
        private PointF startPosition;
        private PointF goalPosition;
        private ExplorationState currentState;
        private ExplorationStrategy strategy;
        
        // SLAM组件
        private AdvancedSLAMMap slamMap;
        private PoseGraphSLAM poseGraphSLAM;
        
        // 机器人状态
        private PointF currentPosition;
        private float currentOrientation;
        private List<PointF> plannedPath;
        private int currentPathIndex;
        
        // 探索控制
        private bool isExplorationRunning;
        private bool isPaused;
        private DateTime explorationStartTime;
        private int stepCount;
        
        // 地图信息
        private List<PointF> frontiers;
        private List<PointF> visitedCells;
        private List<PointF> obstacleCells;
        
        // 路径规划
        private AStarPathPlanner pathPlanner;
        private List<PointF> goalPath;
        private List<PointF> returnPath;
        
        // 事件
        public event Action<ExplorationState> OnStateChanged;
        public event Action<string> OnStatusChanged;
        public event Action<float> OnExplorationProgressChanged;
        public event Action<AdvancedSLAMMap> OnMapUpdated;
        public event Action<PointF, float> OnRobotPoseChanged;

        public AdvancedMazeExplorationController(PointF startPos, PointF goalPos)
        {
            startPosition = startPos;
            goalPosition = goalPos;
            currentPosition = startPos;
            currentOrientation = 0.0f;
            
            // 初始化SLAM地图
            var mapConfig = new AdvancedSLAMMap.MapConfig
            {
                Resolution = 0.05f,  // 5cm分辨率
                Width = 20.0f,       // 20m宽度
                Height = 20.0f,      // 20m高度
                Origin = new PointF(10.0f, 10.0f), // 中心偏移
                MaxRange = 5.0f,     // 5m最大探测距离
                HitProbability = 0.7f,
                MissProbability = 0.3f
            };
            
            slamMap = new AdvancedSLAMMap(mapConfig, AdvancedSLAMMap.MapType.Hybrid);
            poseGraphSLAM = slamMap.GetPoseGraphSLAM();
            
            // 初始化路径规划器
            pathPlanner = new AStarPathPlanner();
            
            // 初始化状态
            currentState = ExplorationState.Initialization;
            strategy = ExplorationStrategy.Hybrid;
            isExplorationRunning = false;
            isPaused = false;
            stepCount = 0;
            
            // 初始化数据结构
            plannedPath = new List<PointF>();
            goalPath = new List<PointF>();
            returnPath = new List<PointF>();
            frontiers = new List<PointF>();
            visitedCells = new List<PointF>();
            obstacleCells = new List<PointF>();
        }

        /// <summary>
        /// 开始探索
        /// </summary>
        public void StartExploration()
        {
            isExplorationRunning = true;
            isPaused = false;
            explorationStartTime = DateTime.Now;
            stepCount = 0;
            currentState = ExplorationState.Exploration;
            
            OnStateChanged?.Invoke(currentState);
            OnStatusChanged?.Invoke("探索已开始");
        }

        /// <summary>
        /// 停止探索
        /// </summary>
        public void StopExploration()
        {
            isExplorationRunning = false;
            isPaused = false;
            currentState = ExplorationState.Completed;
            
            OnStateChanged?.Invoke(currentState);
            OnStatusChanged?.Invoke("探索已停止");
        }

        /// <summary>
        /// 暂停/继续探索
        /// </summary>
        public void TogglePause()
        {
            isPaused = !isPaused;
            OnStatusChanged?.Invoke(isPaused ? "探索已暂停" : "探索继续");
        }

        /// <summary>
        /// 重置探索
        /// </summary>
        public void Reset()
        {
            isExplorationRunning = false;
            isPaused = false;
            currentState = ExplorationState.Initialization;
            currentPosition = startPosition;
            currentOrientation = 0.0f;
            stepCount = 0;
            
            plannedPath.Clear();
            goalPath.Clear();
            returnPath.Clear();
            frontiers.Clear();
            visitedCells.Clear();
            obstacleCells.Clear();
            
            slamMap.Clear();
            
            OnStateChanged?.Invoke(currentState);
            OnStatusChanged?.Invoke("已重置");
        }

        /// <summary>
        /// 更新探索控制器
        /// </summary>
        public void Update(List<PointF> lidarData)
        {
            if (!isExplorationRunning || isPaused) return;
            
            stepCount++;
            
            // 更新SLAM地图
            slamMap.UpdateMap(currentPosition, currentOrientation, lidarData);
            
            // 更新机器人位姿
            UpdateRobotPose();
            
            // 根据当前状态执行相应操作
            switch (currentState)
            {
                case ExplorationState.Initialization:
                    HandleInitialization();
                    break;
                case ExplorationState.Exploration:
                    HandleExploration();
                    break;
                case ExplorationState.SearchGoal:
                    HandleSearchGoal();
                    break;
                case ExplorationState.NavigateToGoal:
                    HandleNavigateToGoal();
                    break;
                case ExplorationState.PathPlanning:
                    HandlePathPlanning();
                    break;
                case ExplorationState.Navigation:
                    HandleNavigation();
                    break;
                case ExplorationState.Completed:
                    HandleCompleted();
                    break;
            }
            
            // 更新进度
            float progress = CalculateExplorationProgress();
            OnExplorationProgressChanged?.Invoke(progress);
            
            // 触发地图更新事件
            OnMapUpdated?.Invoke(slamMap);
        }

        /// <summary>
        /// 处理初始化状态
        /// </summary>
        private void HandleInitialization()
        {
            OnStatusChanged?.Invoke("初始化完成，开始探索");
            currentState = ExplorationState.Exploration;
            OnStateChanged?.Invoke(currentState);
        }

        /// <summary>
        /// 处理探索状态
        /// </summary>
        private void HandleExploration()
        {
            // 更新边界
            UpdateFrontiers();
            
            // 检查是否找到目标
            if (IsGoalFound())
            {
                OnStatusChanged?.Invoke("发现目标，开始导航");
                currentState = ExplorationState.NavigateToGoal;
                OnStateChanged?.Invoke(currentState);
                return;
            }
            
            // 检查探索是否完成
            if (IsExplorationComplete())
            {
                OnStatusChanged?.Invoke("探索完成，搜索目标");
                currentState = ExplorationState.SearchGoal;
                OnStateChanged?.Invoke(currentState);
                return;
            }
            
            // 执行探索策略
            ExecuteExplorationStrategy();
        }

        /// <summary>
        /// 处理搜索目标状态
        /// </summary>
        private void HandleSearchGoal()
        {
            // 尝试规划到目标的路径
            if (pathPlanner.PlanPath(currentPosition, goalPosition, slamMap))
            {
                goalPath = pathPlanner.GetPath();
                OnStatusChanged?.Invoke("找到目标路径，开始导航");
                currentState = ExplorationState.NavigateToGoal;
                OnStateChanged?.Invoke(currentState);
            }
            else
            {
                OnStatusChanged?.Invoke("无法找到目标路径，继续探索");
                currentState = ExplorationState.Exploration;
                OnStateChanged?.Invoke(currentState);
            }
        }

        /// <summary>
        /// 处理导航到目标状态
        /// </summary>
        private void HandleNavigateToGoal()
        {
            if (IsGoalReached())
            {
                OnStatusChanged?.Invoke("已到达目标，开始路径规划");
                currentState = ExplorationState.PathPlanning;
                OnStateChanged?.Invoke(currentState);
                return;
            }
            
            // 沿着目标路径移动
            if (goalPath.Count > 0)
            {
                MoveAlongPath(goalPath);
            }
            else
            {
                OnStatusChanged?.Invoke("目标路径为空，重新规划");
                currentState = ExplorationState.SearchGoal;
                OnStateChanged?.Invoke(currentState);
            }
        }

        /// <summary>
        /// 处理路径规划状态
        /// </summary>
        private void HandlePathPlanning()
        {
            // 规划返回起点的路径
            if (pathPlanner.PlanPath(currentPosition, startPosition, slamMap))
            {
                returnPath = pathPlanner.GetPath();
                OnStatusChanged?.Invoke("返回路径规划完成，开始导航");
                currentState = ExplorationState.Navigation;
                OnStateChanged?.Invoke(currentState);
            }
            else
            {
                OnStatusChanged?.Invoke("无法规划返回路径，任务完成");
                currentState = ExplorationState.Completed;
                OnStateChanged?.Invoke(currentState);
            }
        }

        /// <summary>
        /// 处理导航状态
        /// </summary>
        private void HandleNavigation()
        {
            if (IsStartReached())
            {
                OnStatusChanged?.Invoke("已返回起点，任务完成");
                currentState = ExplorationState.Completed;
                OnStateChanged?.Invoke(currentState);
                return;
            }
            
            // 沿着返回路径移动
            if (returnPath.Count > 0)
            {
                MoveAlongPath(returnPath);
            }
            else
            {
                OnStatusChanged?.Invoke("返回路径为空，任务完成");
                currentState = ExplorationState.Completed;
                OnStateChanged?.Invoke(currentState);
            }
        }

        /// <summary>
        /// 处理完成状态
        /// </summary>
        private void HandleCompleted()
        {
            OnStatusChanged?.Invoke("任务完成");
            isExplorationRunning = false;
        }

        /// <summary>
        /// 更新机器人位姿
        /// </summary>
        private void UpdateRobotPose()
        {
            // 这里应该从实际的传感器数据更新位姿
            // 目前使用简化的更新方法
            
            OnRobotPoseChanged?.Invoke(currentPosition, currentOrientation);
        }

        /// <summary>
        /// 更新边界
        /// </summary>
        private void UpdateFrontiers()
        {
            frontiers.Clear();
            
            // 简化的边界检测算法
            var freeSpace = slamMap.GetFreeSpacePoints();
            var obstacles = slamMap.GetObstaclePoints();
            
            foreach (var freePoint in freeSpace)
            {
                if (IsFrontierPoint(freePoint, obstacles))
                {
                    frontiers.Add(freePoint);
                }
            }
        }

        /// <summary>
        /// 检查是否为边界点
        /// </summary>
        private bool IsFrontierPoint(PointF point, List<PointF> obstacles)
        {
            float threshold = 0.5f; // 50cm阈值
            
            foreach (var obstacle in obstacles)
            {
                float distance = (float)Math.Sqrt(
                    Math.Pow(point.X - obstacle.X, 2) + 
                    Math.Pow(point.Y - obstacle.Y, 2)
                );
                
                if (distance < threshold)
                {
                    return true;
                }
            }
            
            return false;
        }

        /// <summary>
        /// 执行探索策略
        /// </summary>
        private void ExecuteExplorationStrategy()
        {
            switch (strategy)
            {
                case ExplorationStrategy.FrontierBased:
                    ExecuteFrontierBasedExploration();
                    break;
                case ExplorationStrategy.RandomWalk:
                    ExecuteRandomWalkExploration();
                    break;
                case ExplorationStrategy.WallFollowing:
                    ExecuteWallFollowingExploration();
                    break;
                case ExplorationStrategy.Hybrid:
                    ExecuteHybridExploration();
                    break;
            }
        }

        /// <summary>
        /// 执行基于边界的探索
        /// </summary>
        private void ExecuteFrontierBasedExploration()
        {
            if (frontiers.Count == 0) return;
            
            // 选择最近的边界点
            PointF nearestFrontier = frontiers.OrderBy(f => 
                Math.Sqrt(Math.Pow(f.X - currentPosition.X, 2) + Math.Pow(f.Y - currentPosition.Y, 2))
            ).First();
            
            // 规划到边界点的路径
            if (pathPlanner.PlanPath(currentPosition, nearestFrontier, slamMap))
            {
                plannedPath = pathPlanner.GetPath();
                MoveAlongPath(plannedPath);
            }
        }

        /// <summary>
        /// 执行随机游走探索
        /// </summary>
        private void ExecuteRandomWalkExploration()
        {
            // 生成随机方向
            Random random = new Random();
            float angle = (float)(random.NextDouble() * 2 * Math.PI);
            
            // 计算新位置
            float distance = 0.5f; // 50cm步长
            float newX = currentPosition.X + distance * (float)Math.Cos(angle);
            float newY = currentPosition.Y + distance * (float)Math.Sin(angle);
            
            // 检查新位置是否安全
            if (IsPositionSafe(newX, newY))
            {
                currentPosition = new PointF(newX, newY);
                currentOrientation = angle;
            }
        }

        /// <summary>
        /// 执行沿墙行走探索
        /// </summary>
        private void ExecuteWallFollowingExploration()
        {
            // 简化的沿墙行走算法
            float wallDistance = 0.3f; // 30cm距离
            float stepSize = 0.2f; // 20cm步长
            
            // 检测右侧墙壁
            float rightDistance = GetDistanceInDirection(currentOrientation + (float)Math.PI / 2);
            
            if (rightDistance > wallDistance)
            {
                // 向右转
                currentOrientation += 0.1f;
            }
            else if (rightDistance < wallDistance / 2)
            {
                // 向左转
                currentOrientation -= 0.1f;
            }
            
            // 前进
            float newX = currentPosition.X + stepSize * (float)Math.Cos(currentOrientation);
            float newY = currentPosition.Y + stepSize * (float)Math.Sin(currentOrientation);
            
            if (IsPositionSafe(newX, newY))
            {
                currentPosition = new PointF(newX, newY);
            }
        }

        /// <summary>
        /// 执行混合探索策略
        /// </summary>
        private void ExecuteHybridExploration()
        {
            // 根据情况选择不同的探索策略
            if (frontiers.Count > 0)
            {
                ExecuteFrontierBasedExploration();
            }
            else
            {
                ExecuteWallFollowingExploration();
            }
        }

        /// <summary>
        /// 沿路径移动
        /// </summary>
        private void MoveAlongPath(List<PointF> path)
        {
            if (path.Count == 0) return;
            
            // 移动到路径上的下一个点
            PointF nextPoint = path[0];
            path.RemoveAt(0);
            
            // 计算朝向
            float dx = nextPoint.X - currentPosition.X;
            float dy = nextPoint.Y - currentPosition.Y;
            currentOrientation = (float)Math.Atan2(dy, dx);
            
            // 更新位置
            currentPosition = nextPoint;
        }

        /// <summary>
        /// 获取指定方向的距离
        /// </summary>
        private float GetDistanceInDirection(float direction)
        {
            // 简化的距离检测
            // 实际应用中应该使用激光雷达数据
            return 1.0f; // 默认返回1米
        }

        /// <summary>
        /// 检查位置是否安全
        /// </summary>
        private bool IsPositionSafe(float x, float y)
        {
            // 检查是否在障碍物附近
            var obstacles = slamMap.GetObstaclePoints();
            float safetyDistance = 0.2f; // 20cm安全距离
            
            foreach (var obstacle in obstacles)
            {
                float distance = (float)Math.Sqrt(
                    Math.Pow(x - obstacle.X, 2) + 
                    Math.Pow(y - obstacle.Y, 2)
                );
                
                if (distance < safetyDistance)
                {
                    return false;
                }
            }
            
            return true;
        }

        /// <summary>
        /// 检查是否找到目标
        /// </summary>
        private bool IsGoalFound()
        {
            float distance = (float)Math.Sqrt(
                Math.Pow(currentPosition.X - goalPosition.X, 2) + 
                Math.Pow(currentPosition.Y - goalPosition.Y, 2)
            );
            
            return distance < 1.0f; // 1米范围内认为找到目标
        }

        /// <summary>
        /// 检查是否到达目标
        /// </summary>
        private bool IsGoalReached()
        {
            return IsGoalFound();
        }

        /// <summary>
        /// 检查是否到达起点
        /// </summary>
        private bool IsStartReached()
        {
            float distance = (float)Math.Sqrt(
                Math.Pow(currentPosition.X - startPosition.X, 2) + 
                Math.Pow(currentPosition.Y - startPosition.Y, 2)
            );
            
            return distance < 1.0f; // 1米范围内认为到达起点
        }

        /// <summary>
        /// 检查探索是否完成
        /// </summary>
        private bool IsExplorationComplete()
        {
            // 简化的探索完成判断
            // 实际应用中应该基于地图覆盖率
            return stepCount > 1000; // 超过1000步认为探索完成
        }

        /// <summary>
        /// 计算探索进度
        /// </summary>
        private float CalculateExplorationProgress()
        {
            // 简化的进度计算
            // 实际应用中应该基于地图覆盖率
            float maxSteps = 2000.0f;
            return Math.Min(100.0f, (stepCount / maxSteps) * 100.0f);
        }

        // 公共属性
        public ExplorationState CurrentState => currentState;
        public PointF CurrentPosition => currentPosition;
        public float CurrentOrientation => currentOrientation;
        public AdvancedSLAMMap SLAMMap => slamMap;
        public PoseGraphSLAM PoseGraphSLAM => poseGraphSLAM;
        public bool IsRunning => isExplorationRunning;
        public bool IsPaused => isPaused;
        public int StepCount => stepCount;
        public List<PointF> Frontiers => new List<PointF>(frontiers);
        public List<PointF> PlannedPath => new List<PointF>(plannedPath);
        public List<PointF> GoalPath => new List<PointF>(goalPath);
        public List<PointF> ReturnPath => new List<PointF>(returnPath);
    }
}
