using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace BluetoothApp
{
    /// <summary>
    /// SLAM地图管理器
    /// </summary>
    public class SLAMMap
    {
        // 地图状态枚举
        public enum MapCellState
        {
            Unknown = 0,    // 未知区域（灰色）
            Free = 1,       // 空闲区域（白色）
            Obstacle = 2    // 障碍物（黑色）
        }

        // 地图参数
        private int gridSize;
        private float resolution;
        private float displaySize;
        private MapCellState[,] slamMap;
        
        // 机器人路径记录
        private List<PointF> robotPath;
        
        // 激光扫描数据
        private List<PointF> scanPoints;
        private List<PointF> obstaclePoints;
        private List<Tuple<PointF, PointF>> radarRays;
        
        // 道路宽度
        private float roadWidth;
        
        // 雷达扫描参数
        private float radarRange;
        private float radarAngleMin;
        private float radarAngleMax;
        private float radarAngleStep;

        public SLAMMap(float displaySize = 20.0f, float resolution = 0.1f)
        {
            this.displaySize = displaySize;
            this.resolution = resolution;
            this.gridSize = (int)(displaySize / resolution);
            
            // 初始化地图
            slamMap = new MapCellState[gridSize, gridSize];
            
            // 初始化数据结构
            robotPath = new List<PointF>();
            scanPoints = new List<PointF>();
            obstaclePoints = new List<PointF>();
            radarRays = new List<Tuple<PointF, PointF>>();
            
            // 设置参数
            roadWidth = 0.5f;
            radarRange = 3.0f;
            radarAngleMin = -(float)Math.PI / 2;  // -90度
            radarAngleMax = (float)Math.PI / 2;    // 90度
            radarAngleStep = (float)Math.PI / 36; // 5度
        }

        /// <summary>
        /// 将世界坐标转换为网格坐标
        /// </summary>
        public Point WorldToGrid(PointF worldPos)
        {
            int x = (int)((worldPos.X + 2) / resolution);
            int y = (int)((worldPos.Y + 2) / resolution);
            
            x = Math.Max(0, Math.Min(x, gridSize - 1));
            y = Math.Max(0, Math.Min(y, gridSize - 1));
            
            return new Point(x, y);
        }

        /// <summary>
        /// 将网格坐标转换为世界坐标
        /// </summary>
        public PointF GridToWorld(Point gridPos)
        {
            float x = gridPos.X * resolution - 2;
            float y = gridPos.Y * resolution - 2;
            return new PointF(x, y);
        }

        /// <summary>
        /// 更新SLAM地图
        /// </summary>
        public void UpdateMap(PointF robotPos, List<PointF> lidarData)
        {
            // 添加机器人位置到路径
            robotPath.Add(robotPos);
            
            // 清空之前的数据
            scanPoints.Clear();
            obstaclePoints.Clear();
            radarRays.Clear();
            
            // 获取机器人位置的网格坐标
            Point robotGrid = WorldToGrid(robotPos);
            
            // 将机器人当前位置标记为已探索（空闲）
            int roadWidthGrid = (int)(roadWidth / resolution);
            for (int dx = -roadWidthGrid; dx <= roadWidthGrid; dx++)
            {
                for (int dy = -roadWidthGrid; dy <= roadWidthGrid; dy++)
                {
                    int nx = robotGrid.X + dx;
                    int ny = robotGrid.Y + dy;
                    
                    if (nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize)
                    {
                        // 只有未知区域才标记为空闲，避免覆盖已标记的障碍物
                        if (slamMap[nx, ny] == MapCellState.Unknown)
                        {
                            slamMap[nx, ny] = MapCellState.Free;
                        }
                    }
                }
            }
            
            // 执行雷达扫描
            PerformRadarScan(robotPos);
            
            // 处理激光扫描数据
            if (lidarData != null && lidarData.Count > 0)
            {
                foreach (var point in lidarData)
                {
                    scanPoints.Add(point);
                    
                    // 将激光点标记为障碍物
                    Point gridPoint = WorldToGrid(point);
                    
                    // 将障碍物点周围小范围标记为障碍物，使墙壁更明显
                    int obstacleWidth = 1;
                    for (int dx = -obstacleWidth; dx <= obstacleWidth; dx++)
                    {
                        for (int dy = -obstacleWidth; dy <= obstacleWidth; dy++)
                        {
                            int nx = gridPoint.X + dx;
                            int ny = gridPoint.Y + dy;
                            
                            if (nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize)
                            {
                                slamMap[nx, ny] = MapCellState.Obstacle;
                            }
                        }
                    }
                    
                    // 将机器人到激光点的路径标记为空闲
                    MarkLineAsFree(robotGrid, gridPoint);
                    
                    // 记录障碍物点
                    obstaclePoints.Add(point);
                }
            }
        }

        /// <summary>
        /// 执行雷达扫描
        /// </summary>
        private void PerformRadarScan(PointF robotPos)
        {
            float robotX = robotPos.X;
            float robotY = robotPos.Y;
            float robotTheta = 0; // 从机器人对象获取朝向，这里暂时设为0
            
            // 雷达扫描，从左到右扫描前方区域
            for (float angle = radarAngleMin; angle <= radarAngleMax; angle += radarAngleStep)
            {
                // 计算雷达射线的绝对角度（考虑机器人朝向）
                float absAngle = robotTheta + angle;
                
                // 计算雷达射线的方向向量
                float dx = (float)Math.Cos(absAngle);
                float dy = (float)Math.Sin(absAngle);
                
                // 执行射线投射，寻找障碍物
                var hitResult = CastRay(robotPos, dx, dy, radarRange);
                PointF hitPoint = hitResult.Item1;
                bool hitObstacle = hitResult.Item2;
                
                // 记录雷达射线
                radarRays.Add(new Tuple<PointF, PointF>(robotPos, hitPoint));
                
                // 如果射线击中障碍物，将其标记为障碍物
                if (hitObstacle)
                {
                    Point gridPoint = WorldToGrid(hitPoint);
                    
                    // 将障碍物点周围小范围标记为障碍物
                    int obstacleWidth = 1;
                    for (int dx_obs = -obstacleWidth; dx_obs <= obstacleWidth; dx_obs++)
                    {
                        for (int dy_obs = -obstacleWidth; dy_obs <= obstacleWidth; dy_obs++)
                        {
                            int nx = gridPoint.X + dx_obs;
                            int ny = gridPoint.Y + dy_obs;
                            
                            if (nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize)
                            {
                                slamMap[nx, ny] = MapCellState.Obstacle;
                            }
                        }
                    }
                    
                    // 将机器人到障碍物点的路径标记为空闲
                    Point robotGrid = WorldToGrid(robotPos);
                    MarkLineAsFree(robotGrid, gridPoint);
                    
                    // 记录障碍物点
                    obstaclePoints.Add(hitPoint);
                }
            }
        }

        /// <summary>
        /// 投射射线，检测障碍物
        /// </summary>
        private Tuple<PointF, bool> CastRay(PointF startPos, float dx, float dy, float maxRange)
        {
            float stepSize = 0.1f;
            float currentX = startPos.X;
            float currentY = startPos.Y;
            float distance = 0.0f;
            
            while (distance < maxRange)
            {
                currentX += dx * stepSize;
                currentY += dy * stepSize;
                distance += stepSize;
                
                // 检查是否超出地图边界
                if (currentX < 0 || currentX >= displaySize || currentY < 0 || currentY >= displaySize)
                {
                    return new Tuple<PointF, bool>(new PointF(currentX, currentY), true);
                }
                
                // 检查是否击中障碍物（这里需要根据实际环境判断）
                // 暂时简化处理
                if (IsObstacleAt(currentX, currentY))
                {
                    return new Tuple<PointF, bool>(new PointF(currentX, currentY), true);
                }
            }
            
            // 没有击中障碍物，返回最大范围的点
            return new Tuple<PointF, bool>(new PointF(startPos.X + dx * maxRange, startPos.Y + dy * maxRange), false);
        }

        /// <summary>
        /// 检查指定位置是否有障碍物
        /// </summary>
        private bool IsObstacleAt(float x, float y)
        {
            Point gridPos = WorldToGrid(new PointF(x, y));
            if (gridPos.X >= 0 && gridPos.X < gridSize && gridPos.Y >= 0 && gridPos.Y < gridSize)
            {
                return slamMap[gridPos.X, gridPos.Y] == MapCellState.Obstacle;
            }
            return true; // 超出边界视为障碍物
        }

        /// <summary>
        /// 使用Bresenham算法将线段标记为空闲
        /// </summary>
        private void MarkLineAsFree(Point start, Point end)
        {
            int dx = Math.Abs(end.X - start.X);
            int dy = Math.Abs(end.Y - start.Y);
            int sx = start.X < end.X ? 1 : -1;
            int sy = start.Y < end.Y ? 1 : -1;
            int err = dx - dy;
            
            int currentX = start.X;
            int currentY = start.Y;
            
            int roadWidthGrid = (int)(roadWidth / resolution);
            
            while (true)
            {
                // 检查是否到达终点前一个位置
                if ((sx > 0 && currentX >= end.X - 1) || (sx < 0 && currentX <= end.X + 1) ||
                    (sy > 0 && currentY >= end.Y - 1) || (sy < 0 && currentY <= end.Y + 1))
                {
                    break;
                }
                
                // 标记当前点及其周围为空闲
                for (int dx_road = -roadWidthGrid; dx_road <= roadWidthGrid; dx_road++)
                {
                    for (int dy_road = -roadWidthGrid; dy_road <= roadWidthGrid; dy_road++)
                    {
                        int nx = currentX + dx_road;
                        int ny = currentY + dy_road;
                        
                        if (nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize)
                        {
                            if (slamMap[nx, ny] == MapCellState.Unknown)
                            {
                                slamMap[nx, ny] = MapCellState.Free;
                            }
                        }
                    }
                }
                
                // 沿着Bresenham线移动
                int e2 = 2 * err;
                if (e2 > -dy)
                {
                    err -= dy;
                    currentX += sx;
                }
                if (e2 < dx)
                {
                    err += dx;
                    currentY += sy;
                }
            }
        }

        /// <summary>
        /// 获取SLAM地图数据
        /// </summary>
        public MapCellState[,] GetSLAMMap()
        {
            return slamMap;
        }

        /// <summary>
        /// 获取机器人路径
        /// </summary>
        public List<PointF> GetRobotPath()
        {
            return robotPath;
        }

        /// <summary>
        /// 获取扫描点
        /// </summary>
        public List<PointF> GetScanPoints()
        {
            return scanPoints;
        }

        /// <summary>
        /// 获取障碍物点
        /// </summary>
        public List<PointF> GetObstaclePoints()
        {
            return obstaclePoints;
        }

        /// <summary>
        /// 获取雷达射线
        /// </summary>
        public List<Tuple<PointF, PointF>> GetRadarRays()
        {
            return radarRays;
        }

        /// <summary>
        /// 获取地图尺寸
        /// </summary>
        public int GetGridSize()
        {
            return gridSize;
        }

        /// <summary>
        /// 获取分辨率
        /// </summary>
        public float GetResolution()
        {
            return resolution;
        }
    }
}
