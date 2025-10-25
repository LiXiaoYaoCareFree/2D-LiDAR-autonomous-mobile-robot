using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;

namespace BluetoothApp
{
    /// <summary>
    /// 高级SLAM地图管理器
    /// 支持多种地图类型和实时更新
    /// </summary>
    public class AdvancedSLAMMap
    {
        /// <summary>
        /// 地图类型枚举
        /// </summary>
        public enum MapType
        {
            OccupancyGrid,  // 占用栅格地图
            PointCloud,     // 点云地图
            Hybrid         // 混合地图
        }

        /// <summary>
        /// 栅格状态
        /// </summary>
        public enum GridState
        {
            Unknown = 0,    // 未知
            Free = 1,       // 空闲
            Occupied = 2,   // 占用
            Obstacle = 3    // 障碍物
        }

        /// <summary>
        /// 地图配置
        /// </summary>
        public class MapConfig
        {
            public float Resolution { get; set; } = 0.05f;  // 分辨率 (m/pixel)
            public float Width { get; set; } = 20.0f;       // 地图宽度 (m)
            public float Height { get; set; } = 20.0f;      // 地图高度 (m)
            public PointF Origin { get; set; } = new PointF(10.0f, 10.0f); // 原点偏移
            public float MaxRange { get; set; } = 5.0f;     // 最大探测距离 (m)
            public float HitProbability { get; set; } = 0.7f; // 击中概率
            public float MissProbability { get; set; } = 0.3f; // 未击中概率
        }

        private MapConfig config;
        private MapType mapType;
        private int[,] occupancyGrid;
        private float[,] probabilityGrid;
        private List<PointF> pointCloud;
        private List<PointF> obstaclePoints;
        private List<PointF> freeSpacePoints;
        private List<PointF> robotTrajectory;
        
        // 地图尺寸
        private int gridWidth;
        private int gridHeight;
        
        // 位姿图SLAM
        private PoseGraphSLAM poseGraphSLAM;
        
        // 地图更新历史
        private List<MapUpdate> updateHistory;

        public AdvancedSLAMMap(MapConfig config, MapType mapType = MapType.Hybrid)
        {
            this.config = config;
            this.mapType = mapType;
            
            gridWidth = (int)(config.Width / config.Resolution);
            gridHeight = (int)(config.Height / config.Resolution);
            
            occupancyGrid = new int[gridWidth, gridHeight];
            probabilityGrid = new float[gridWidth, gridHeight];
            pointCloud = new List<PointF>();
            obstaclePoints = new List<PointF>();
            freeSpacePoints = new List<PointF>();
            robotTrajectory = new List<PointF>();
            
            poseGraphSLAM = new PoseGraphSLAM();
            updateHistory = new List<MapUpdate>();
            
            InitializeGrid();
        }

        /// <summary>
        /// 初始化栅格
        /// </summary>
        private void InitializeGrid()
        {
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    occupancyGrid[x, y] = (int)GridState.Unknown;
                    probabilityGrid[x, y] = 0.5f; // 初始概率为0.5（未知）
                }
            }
        }

        /// <summary>
        /// 更新地图
        /// </summary>
        public void UpdateMap(PointF robotPose, float robotTheta, List<PointF> lidarData)
        {
            // 添加位姿到位姿图
            int nodeId = poseGraphSLAM.AddPose(robotPose.X, robotPose.Y, robotTheta, lidarData);
            
            // 更新轨迹
            robotTrajectory.Add(robotPose);
            
            // 根据地图类型更新
            switch (mapType)
            {
                case MapType.OccupancyGrid:
                    UpdateOccupancyGrid(robotPose, robotTheta, lidarData);
                    break;
                case MapType.PointCloud:
                    UpdatePointCloud(robotPose, robotTheta, lidarData);
                    break;
                case MapType.Hybrid:
                    UpdateOccupancyGrid(robotPose, robotTheta, lidarData);
                    UpdatePointCloud(robotPose, robotTheta, lidarData);
                    break;
            }
            
            // 记录更新历史
            updateHistory.Add(new MapUpdate
            {
                Timestamp = DateTime.Now,
                RobotPose = robotPose,
                RobotTheta = robotTheta,
                LidarData = new List<PointF>(lidarData),
                NodeId = nodeId
            });
        }

        /// <summary>
        /// 更新占用栅格地图
        /// </summary>
        private void UpdateOccupancyGrid(PointF robotPose, float robotTheta, List<PointF> lidarData)
        {
            foreach (var point in lidarData)
            {
                // 转换激光点到全局坐标系
                var globalPoint = TransformToGlobal(robotPose, robotTheta, point);
                
                // 更新击中点
                UpdateGridCell(globalPoint, GridState.Occupied, config.HitProbability);
                
                // 更新从机器人到击中点的路径
                UpdateRay(robotPose, globalPoint);
            }
        }

        /// <summary>
        /// 更新点云地图
        /// </summary>
        private void UpdatePointCloud(PointF robotPose, float robotTheta, List<PointF> lidarData)
        {
            foreach (var point in lidarData)
            {
                var globalPoint = TransformToGlobal(robotPose, robotTheta, point);
                
                // 添加到点云
                if (!IsPointTooClose(globalPoint, pointCloud))
                {
                    pointCloud.Add(globalPoint);
                    obstaclePoints.Add(globalPoint);
                }
                
                // 更新自由空间
                UpdateFreeSpace(robotPose, globalPoint);
            }
        }

        /// <summary>
        /// 更新栅格单元
        /// </summary>
        private void UpdateGridCell(PointF point, GridState state, float probability)
        {
            var gridPos = WorldToGrid(point);
            if (IsValidGridPosition(gridPos))
            {
                // 更新占用栅格
                if (state == GridState.Occupied)
                {
                    occupancyGrid[gridPos.X, gridPos.Y] = (int)state;
                }
                
                // 更新概率栅格
                UpdateProbabilityGrid(gridPos, probability);
            }
        }

        /// <summary>
        /// 更新概率栅格
        /// </summary>
        private void UpdateProbabilityGrid(Point gridPos, float probability)
        {
            float currentProb = probabilityGrid[gridPos.X, gridPos.Y];
            float newProb = (currentProb * probability) / (currentProb * probability + (1 - currentProb) * (1 - probability));
            probabilityGrid[gridPos.X, gridPos.Y] = Math.Max(0.1f, Math.Min(0.9f, newProb));
        }

        /// <summary>
        /// 更新射线（从机器人到障碍物）
        /// </summary>
        private void UpdateRay(PointF start, PointF end)
        {
            var startGrid = WorldToGrid(start);
            var endGrid = WorldToGrid(end);
            
            if (!IsValidGridPosition(startGrid) || !IsValidGridPosition(endGrid))
                return;
            
            // 使用Bresenham算法更新射线路径
            var rayPoints = GetBresenhamLine(startGrid, endGrid);
            
            foreach (var point in rayPoints)
            {
                if (IsValidGridPosition(point))
                {
                    // 更新为自由空间
                    if (occupancyGrid[point.X, point.Y] == (int)GridState.Unknown)
                    {
                        occupancyGrid[point.X, point.Y] = (int)GridState.Free;
                        UpdateProbabilityGrid(point, config.MissProbability);
                    }
                }
            }
        }

        /// <summary>
        /// 更新自由空间
        /// </summary>
        private void UpdateFreeSpace(PointF robotPose, PointF obstaclePoint)
        {
            float distance = (float)Math.Sqrt(
                Math.Pow(obstaclePoint.X - robotPose.X, 2) + 
                Math.Pow(obstaclePoint.Y - robotPose.Y, 2)
            );
            
            if (distance > 0.1f)
            {
                int steps = (int)(distance / config.Resolution);
                for (int i = 1; i < steps; i++)
                {
                    float t = (float)i / steps;
                    float x = robotPose.X + (obstaclePoint.X - robotPose.X) * t;
                    float y = robotPose.Y + (obstaclePoint.Y - robotPose.Y) * t;
                    
                    var freePoint = new PointF(x, y);
                    if (!IsPointTooClose(freePoint, freeSpacePoints))
                    {
                        freeSpacePoints.Add(freePoint);
                    }
                }
            }
        }

        /// <summary>
        /// 坐标转换：世界坐标到栅格坐标
        /// </summary>
        private Point WorldToGrid(PointF worldPoint)
        {
            int x = (int)((worldPoint.X + config.Origin.X) / config.Resolution);
            int y = (int)((worldPoint.Y + config.Origin.Y) / config.Resolution);
            return new Point(x, y);
        }

        /// <summary>
        /// 坐标转换：栅格坐标到世界坐标
        /// </summary>
        private PointF GridToWorld(Point gridPoint)
        {
            float x = gridPoint.X * config.Resolution - config.Origin.X;
            float y = gridPoint.Y * config.Resolution - config.Origin.Y;
            return new PointF(x, y);
        }

        /// <summary>
        /// 坐标转换：局部坐标到全局坐标
        /// </summary>
        private PointF TransformToGlobal(PointF robotPose, float robotTheta, PointF localPoint)
        {
            float cosTheta = (float)Math.Cos(robotTheta);
            float sinTheta = (float)Math.Sin(robotTheta);
            
            float globalX = robotPose.X + localPoint.X * cosTheta - localPoint.Y * sinTheta;
            float globalY = robotPose.Y + localPoint.X * sinTheta + localPoint.Y * cosTheta;
            
            return new PointF(globalX, globalY);
        }

        /// <summary>
        /// 获取Bresenham直线算法路径点
        /// </summary>
        private List<Point> GetBresenhamLine(Point start, Point end)
        {
            var points = new List<Point>();
            
            int dx = Math.Abs(end.X - start.X);
            int dy = Math.Abs(end.Y - start.Y);
            int sx = start.X < end.X ? 1 : -1;
            int sy = start.Y < end.Y ? 1 : -1;
            int err = dx - dy;
            
            int x = start.X;
            int y = start.Y;
            
            while (true)
            {
                points.Add(new Point(x, y));
                
                if (x == end.X && y == end.Y) break;
                
                int e2 = 2 * err;
                if (e2 > -dy)
                {
                    err -= dy;
                    x += sx;
                }
                if (e2 < dx)
                {
                    err += dx;
                    y += sy;
                }
            }
            
            return points;
        }

        /// <summary>
        /// 检查栅格位置是否有效
        /// </summary>
        private bool IsValidGridPosition(Point gridPos)
        {
            return gridPos.X >= 0 && gridPos.X < gridWidth && 
                   gridPos.Y >= 0 && gridPos.Y < gridHeight;
        }

        /// <summary>
        /// 检查点是否太接近现有点
        /// </summary>
        private bool IsPointTooClose(PointF newPoint, List<PointF> existingPoints)
        {
            float minDistance = config.Resolution * 2; // 2个栅格单元的最小距离
            
            foreach (var point in existingPoints)
            {
                float distance = (float)Math.Sqrt(
                    Math.Pow(newPoint.X - point.X, 2) + 
                    Math.Pow(newPoint.Y - point.Y, 2)
                );
                
                if (distance < minDistance)
                {
                    return true;
                }
            }
            
            return false;
        }

        /// <summary>
        /// 获取占用栅格地图
        /// </summary>
        public int[,] GetOccupancyGrid()
        {
            return (int[,])occupancyGrid.Clone();
        }

        /// <summary>
        /// 获取概率栅格地图
        /// </summary>
        public float[,] GetProbabilityGrid()
        {
            return (float[,])probabilityGrid.Clone();
        }

        /// <summary>
        /// 获取点云数据
        /// </summary>
        public List<PointF> GetPointCloud()
        {
            return new List<PointF>(pointCloud);
        }

        /// <summary>
        /// 获取障碍物点
        /// </summary>
        public List<PointF> GetObstaclePoints()
        {
            return new List<PointF>(obstaclePoints);
        }

        /// <summary>
        /// 获取自由空间点
        /// </summary>
        public List<PointF> GetFreeSpacePoints()
        {
            return new List<PointF>(freeSpacePoints);
        }

        /// <summary>
        /// 获取机器人轨迹
        /// </summary>
        public List<PointF> GetRobotTrajectory()
        {
            return new List<PointF>(robotTrajectory);
        }

        /// <summary>
        /// 获取优化后的轨迹
        /// </summary>
        public List<PointF> GetOptimizedTrajectory()
        {
            return poseGraphSLAM.GetOptimizedTrajectory();
        }

        /// <summary>
        /// 获取位姿图SLAM对象
        /// </summary>
        public PoseGraphSLAM GetPoseGraphSLAM()
        {
            return poseGraphSLAM;
        }

        /// <summary>
        /// 获取地图配置
        /// </summary>
        public MapConfig GetConfig()
        {
            return config;
        }

        /// <summary>
        /// 获取地图统计信息
        /// </summary>
        public string GetMapStatistics()
        {
            int unknownCount = 0;
            int freeCount = 0;
            int occupiedCount = 0;
            
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    switch ((GridState)occupancyGrid[x, y])
                    {
                        case GridState.Unknown:
                            unknownCount++;
                            break;
                        case GridState.Free:
                            freeCount++;
                            break;
                        case GridState.Occupied:
                        case GridState.Obstacle:
                            occupiedCount++;
                            break;
                    }
                }
            }
            
            return $"地图统计:\n" +
                   $"栅格大小: {gridWidth}x{gridHeight}\n" +
                   $"分辨率: {config.Resolution}m/pixel\n" +
                   $"未知区域: {unknownCount}\n" +
                   $"自由空间: {freeCount}\n" +
                   $"障碍物: {occupiedCount}\n" +
                   $"点云数量: {pointCloud.Count}\n" +
                   $"轨迹长度: {robotTrajectory.Count}\n" +
                   $"位姿图: {poseGraphSLAM.GetMapStatistics()}";
        }

        /// <summary>
        /// 清空地图
        /// </summary>
        public void Clear()
        {
            InitializeGrid();
            pointCloud.Clear();
            obstaclePoints.Clear();
            freeSpacePoints.Clear();
            robotTrajectory.Clear();
            updateHistory.Clear();
            poseGraphSLAM.Clear();
        }

        /// <summary>
        /// 地图更新记录
        /// </summary>
        public class MapUpdate
        {
            public DateTime Timestamp { get; set; }
            public PointF RobotPose { get; set; }
            public float RobotTheta { get; set; }
            public List<PointF> LidarData { get; set; }
            public int NodeId { get; set; }
        }
    }
}
