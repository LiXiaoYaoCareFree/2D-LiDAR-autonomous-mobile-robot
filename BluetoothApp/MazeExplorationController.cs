using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace BluetoothApp
{
    /// <summary>
    /// 迷宫探索控制器
    /// </summary>
    public class MazeExplorationController
    {
        // 探索状态枚举
        public enum ExplorationState
        {
            Exploration,        // 探索阶段
            SearchGoal,         // 搜索目标点
            NavigateToGoal,     // 导航到目标
            PathPlanning,       // 路径规划
            Navigation,         // 导航阶段
            Completed           // 完成
        }

        // 当前状态
        public ExplorationState CurrentState { get; private set; }
        
        // SLAM地图
        private SLAMMap slamMap;
        
        // 机器人位置和状态
        private PointF robotPosition;
        private float robotOrientation;
        
        // 目标位置
        private PointF startPosition;
        private PointF goalPosition;
        
        // 探索参数
        private float goalDetectionDistance = 3.0f;
        private float explorationThreshold = 0.99f;
        private int maxNavigationRetries = 5;
        private int navigationRetryCount = 0;
        
        // 探索完成标志
        private bool explorationComplete = false;
        private bool goalFound = false;
        private bool reachedGoal = false;
        private bool returnedToStart = false;
        
        // 路径规划
        private List<PointF> goalPath;
        private int currentPathIndex = 0;
        
        // 事件
        public event Action<ExplorationState> OnStateChanged;
        public event Action<string> OnStatusChanged;
        public event Action<float> OnExplorationProgressChanged;
        public event Action<SLAMMap> OnMapUpdated;

        public MazeExplorationController(PointF startPos, PointF goalPos, float mapSize = 20.0f, float resolution = 0.1f)
        {
            startPosition = startPos;
            goalPosition = goalPos;
            robotPosition = startPos;
            robotOrientation = 0.0f;
            
            slamMap = new SLAMMap(mapSize, resolution);
            CurrentState = ExplorationState.Exploration;
            goalPath = new List<PointF>();
        }

        /// <summary>
        /// 更新探索状态
        /// </summary>
        public void Update(List<PointF> lidarData)
        {
            // 更新SLAM地图
            slamMap.UpdateMap(robotPosition, lidarData);
            
            // 根据当前状态执行不同的操作
            switch (CurrentState)
            {
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
                    // 探索完成，不再更新
                    break;
            }
            
            // 触发地图更新事件
            OnMapUpdated?.Invoke(slamMap);
        }

        /// <summary>
        /// 处理探索阶段
        /// </summary>
        private void HandleExploration()
        {
            // 检查是否已经找到目标
            float distToGoal = DistanceTo(robotPosition, goalPosition);
            if (distToGoal <= goalDetectionDistance && !goalFound)
            {
                goalFound = true;
                OnStatusChanged?.Invoke($"发现目标点！({goalPosition.X:F1}, {goalPosition.Y:F1})");
            }
            
            // 检查探索完成度（这里简化处理）
            float explorationProgress = CalculateExplorationProgress();
            OnExplorationProgressChanged?.Invoke(explorationProgress);
            
            if (explorationProgress >= explorationThreshold * 100)
            {
                explorationComplete = true;
                OnStatusChanged?.Invoke($"探索完成，进度: {explorationProgress:F1}%");
                
                if (goalFound && !reachedGoal)
                {
                    ChangeState(ExplorationState.NavigateToGoal);
                }
                else
                {
                    ChangeState(ExplorationState.SearchGoal);
                }
            }
        }

        /// <summary>
        /// 处理搜索目标阶段
        /// </summary>
        private void HandleSearchGoal()
        {
            if (!goalFound)
            {
                // 尝试在地图中寻找目标点
                OnStatusChanged?.Invoke("在地图中搜索目标点...");
                
                if (FindPathToGoal(goalPosition))
                {
                    goalFound = true;
                    OnStatusChanged?.Invoke("找到目标点路径！");
                    ChangeState(ExplorationState.NavigateToGoal);
                }
                else
                {
                    OnStatusChanged?.Invoke("无法找到目标点，直接进入路径规划阶段");
                    ChangeState(ExplorationState.PathPlanning);
                }
            }
            else
            {
                ChangeState(ExplorationState.NavigateToGoal);
            }
        }

        /// <summary>
        /// 处理导航到目标阶段
        /// </summary>
        private void HandleNavigateToGoal()
        {
            // 检查是否已到达目标
            float distToGoal = DistanceTo(robotPosition, goalPosition);
            if (distToGoal <= 1.0f)
            {
                reachedGoal = true;
                OnStatusChanged?.Invoke($"已到达目标点！({goalPosition.X:F1}, {goalPosition.Y:F1})");
                ChangeState(ExplorationState.PathPlanning);
                navigationRetryCount = 0;
                return;
            }
            
            // 沿着规划的路径导航
            if (!NavigateToGoal())
            {
                OnStatusChanged?.Invoke("导航失败，重新规划路径...");
                navigationRetryCount++;
                
                if (navigationRetryCount >= maxNavigationRetries)
                {
                    OnStatusChanged?.Invoke($"导航失败次数达到最大值({maxNavigationRetries})，强制进入下一阶段");
                    reachedGoal = true;
                    ChangeState(ExplorationState.PathPlanning);
                    navigationRetryCount = 0;
                    return;
                }
                
                if (FindPathToGoal(goalPosition))
                {
                    OnStatusChanged?.Invoke("重新规划路径成功");
                }
                else
                {
                    OnStatusChanged?.Invoke("无法找到到目标的路径，强制进入下一阶段");
                    ChangeState(ExplorationState.PathPlanning);
                }
            }
        }

        /// <summary>
        /// 处理路径规划阶段
        /// </summary>
        private void HandlePathPlanning()
        {
            OnStatusChanged?.Invoke("规划从当前位置返回起点的路径");
            
            if (FindPathToGoal(startPosition))
            {
                OnStatusChanged?.Invoke($"找到返回起点的路径，长度: {goalPath.Count}");
                ChangeState(ExplorationState.Navigation);
            }
            else
            {
                OnStatusChanged?.Invoke("无法找到返回起点的路径，探索结束");
                ChangeState(ExplorationState.Completed);
            }
        }

        /// <summary>
        /// 处理导航阶段
        /// </summary>
        private void HandleNavigation()
        {
            // 检查是否已到达起点
            float distToStart = DistanceTo(robotPosition, startPosition);
            if (distToStart <= 1.0f)
            {
                returnedToStart = true;
                OnStatusChanged?.Invoke("任务完成：已返回起点！");
                ChangeState(ExplorationState.Completed);
                return;
            }
            
            // 检查是否有有效路径
            if (goalPath == null || goalPath.Count < 2)
            {
                OnStatusChanged?.Invoke("返回起点的路径无效或为空，尝试重新规划");
                if (FindPathToGoal(startPosition))
                {
                    OnStatusChanged?.Invoke($"重新规划成功，路径长度: {goalPath.Count}");
                }
                else
                {
                    OnStatusChanged?.Invoke("重新规划失败，放弃导航");
                    ChangeState(ExplorationState.Completed);
                    return;
                }
            }
            
            // 沿着规划的路径导航
            if (!NavigateToGoal())
            {
                OnStatusChanged?.Invoke("导航失败，重新规划路径...");
                navigationRetryCount++;
                
                if (navigationRetryCount >= maxNavigationRetries)
                {
                    OnStatusChanged?.Invoke($"导航失败次数达到最大值({maxNavigationRetries})，放弃导航");
                    ChangeState(ExplorationState.Completed);
                    return;
                }
                
                if (FindPathToGoal(startPosition))
                {
                    OnStatusChanged?.Invoke("重新规划路径成功");
                }
                else
                {
                    OnStatusChanged?.Invoke("无法找到到起点的路径，探索结束");
                    ChangeState(ExplorationState.Completed);
                }
            }
        }

        /// <summary>
        /// 寻找路径到目标
        /// </summary>
        private bool FindPathToGoal(PointF target)
        {
            // 这里应该实现A*算法或其他路径规划算法
            // 暂时简化处理，创建直接路径
            goalPath = new List<PointF>();
            
            // 创建从当前位置到目标的简单路径
            float dx = target.X - robotPosition.X;
            float dy = target.Y - robotPosition.Y;
            float distance = (float)Math.Sqrt(dx * dx + dy * dy);
            
            if (distance < 0.1f)
            {
                return true; // 已经到达目标
            }
            
            int steps = Math.Max(1, (int)(distance / 0.5f)); // 每0.5单位一个路径点
            for (int i = 1; i <= steps; i++)
            {
                float t = (float)i / steps;
                float x = robotPosition.X + dx * t;
                float y = robotPosition.Y + dy * t;
                goalPath.Add(new PointF(x, y));
            }
            
            currentPathIndex = 0;
            return goalPath.Count > 0;
        }

        /// <summary>
        /// 导航到目标
        /// </summary>
        private bool NavigateToGoal()
        {
            if (goalPath == null || goalPath.Count == 0)
            {
                return false;
            }
            
            if (currentPathIndex >= goalPath.Count)
            {
                return false; // 路径执行完毕
            }
            
            // 移动到下一个路径点
            PointF nextPoint = goalPath[currentPathIndex];
            float distToNext = DistanceTo(robotPosition, nextPoint);
            
            if (distToNext < 0.5f) // 接近路径点
            {
                currentPathIndex++;
                if (currentPathIndex >= goalPath.Count)
                {
                    return false; // 路径执行完毕
                }
            }
            
            // 计算移动方向
            PointF direction = new PointF(
                nextPoint.X - robotPosition.X,
                nextPoint.Y - robotPosition.Y
            );
            
            float distance = (float)Math.Sqrt(direction.X * direction.X + direction.Y * direction.Y);
            if (distance > 0.1f)
            {
                direction.X /= distance;
                direction.Y /= distance;
                
                // 更新机器人位置（这里应该通过实际控制实现）
                float moveDistance = 0.1f; // 每次移动的距离
                robotPosition.X += direction.X * moveDistance;
                robotPosition.Y += direction.Y * moveDistance;
                
                // 更新机器人朝向
                robotOrientation = (float)Math.Atan2(direction.Y, direction.X);
            }
            
            return true;
        }

        /// <summary>
        /// 计算探索进度
        /// </summary>
        private float CalculateExplorationProgress()
        {
            var map = slamMap.GetSLAMMap();
            int totalCells = map.GetLength(0) * map.GetLength(1);
            int exploredCells = 0;
            
            for (int x = 0; x < map.GetLength(0); x++)
            {
                for (int y = 0; y < map.GetLength(1); y++)
                {
                    if (map[x, y] != SLAMMap.MapCellState.Unknown)
                    {
                        exploredCells++;
                    }
                }
            }
            
            return (float)exploredCells / totalCells * 100.0f;
        }

        /// <summary>
        /// 计算两点间距离
        /// </summary>
        private float DistanceTo(PointF p1, PointF p2)
        {
            float dx = p2.X - p1.X;
            float dy = p2.Y - p1.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// 改变状态
        /// </summary>
        private void ChangeState(ExplorationState newState)
        {
            if (CurrentState != newState)
            {
                CurrentState = newState;
                OnStateChanged?.Invoke(newState);
            }
        }

        /// <summary>
        /// 获取当前机器人位置
        /// </summary>
        public PointF GetRobotPosition()
        {
            return robotPosition;
        }

        /// <summary>
        /// 获取当前机器人朝向
        /// </summary>
        public float GetRobotOrientation()
        {
            return robotOrientation;
        }

        /// <summary>
        /// 获取SLAM地图
        /// </summary>
        public SLAMMap GetSLAMMap()
        {
            return slamMap;
        }

        /// <summary>
        /// 重置探索状态
        /// </summary>
        public void Reset()
        {
            robotPosition = startPosition;
            robotOrientation = 0.0f;
            CurrentState = ExplorationState.Exploration;
            explorationComplete = false;
            goalFound = false;
            reachedGoal = false;
            returnedToStart = false;
            navigationRetryCount = 0;
            goalPath.Clear();
            currentPathIndex = 0;
        }
    }
}
