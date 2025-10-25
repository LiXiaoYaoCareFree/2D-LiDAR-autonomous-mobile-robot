using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace BluetoothApp
{
    /// <summary>
    /// A*路径规划算法实现
    /// </summary>
    public class AStarPathPlanner
    {
        /// <summary>
        /// 节点类
        /// </summary>
        public class Node
        {
            public PointF Position { get; set; }
            public float G { get; set; }  // 从起点到当前节点的实际距离
            public float H { get; set; }  // 从当前节点到终点的启发式距离
            public float F => G + H;      // 总代价
            public Node Parent { get; set; }
            public bool IsObstacle { get; set; }
            public bool IsVisited { get; set; }

            public Node(PointF position)
            {
                Position = position;
                G = float.MaxValue;
                H = 0;
                Parent = null;
                IsObstacle = false;
                IsVisited = false;
            }
        }

        // 路径规划参数
        private float resolution = 0.1f;  // 栅格分辨率
        private float mapWidth = 20.0f;   // 地图宽度
        private float mapHeight = 20.0f;  // 地图高度
        private PointF mapOrigin = new PointF(10.0f, 10.0f); // 地图原点

        // 路径规划结果
        private List<PointF> path;
        private bool pathFound;

        public AStarPathPlanner()
        {
            path = new List<PointF>();
            pathFound = false;
        }

        /// <summary>
        /// 规划路径
        /// </summary>
        public bool PlanPath(PointF start, PointF goal, AdvancedSLAMMap slamMap)
        {
            path.Clear();
            pathFound = false;

            if (slamMap == null)
            {
                return false;
            }

            // 检查起点和终点是否有效
            if (!IsPositionValid(start, slamMap) || !IsPositionValid(goal, slamMap))
            {
                return false;
            }

            // 创建栅格地图
            var gridMap = CreateGridMap(slamMap);
            if (gridMap == null)
            {
                return false;
            }

            // 转换坐标到栅格坐标
            var startGrid = WorldToGrid(start);
            var goalGrid = WorldToGrid(goal);

            // 检查栅格坐标是否有效
            if (!IsValidGridPosition(startGrid, gridMap) || !IsValidGridPosition(goalGrid, gridMap))
            {
                return false;
            }

            // 执行A*算法
            var result = AStarSearch(gridMap, startGrid, goalGrid);
            
            if (result)
            {
                // 构建路径
                BuildPath(gridMap, startGrid, goalGrid);
            }

            return pathFound;
        }

        /// <summary>
        /// 创建栅格地图
        /// </summary>
        private Node[,] CreateGridMap(AdvancedSLAMMap slamMap)
        {
            int gridWidth = (int)(mapWidth / resolution);
            int gridHeight = (int)(mapHeight / resolution);
            
            var gridMap = new Node[gridWidth, gridHeight];
            
            // 初始化栅格
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    var worldPos = GridToWorld(new Point(x, y));
                    gridMap[x, y] = new Node(worldPos);
                }
            }

            // 标记障碍物
            var obstacles = slamMap.GetObstaclePoints();
            foreach (var obstacle in obstacles)
            {
                var gridPos = WorldToGrid(obstacle);
                if (IsValidGridPosition(gridPos, gridMap))
                {
                    gridMap[gridPos.X, gridPos.Y].IsObstacle = true;
                }
            }

            return gridMap;
        }

        /// <summary>
        /// A*搜索算法
        /// </summary>
        private bool AStarSearch(Node[,] gridMap, Point start, Point goal)
        {
            var openSet = new List<Node>();
            var closedSet = new HashSet<Node>();

            // 初始化起点
            var startNode = gridMap[start.X, start.Y];
            startNode.G = 0;
            startNode.H = CalculateHeuristic(start, goal);
            openSet.Add(startNode);

            while (openSet.Count > 0)
            {
                // 选择F值最小的节点
                var currentNode = openSet.OrderBy(n => n.F).First();
                openSet.Remove(currentNode);
                closedSet.Add(currentNode);

                // 检查是否到达目标
                if (currentNode.Position == gridMap[goal.X, goal.Y].Position)
                {
                    return true;
                }

                // 检查相邻节点
                var neighbors = GetNeighbors(currentNode, gridMap);
                foreach (var neighbor in neighbors)
                {
                    if (closedSet.Contains(neighbor) || neighbor.IsObstacle)
                    {
                        continue;
                    }

                    float tentativeG = currentNode.G + CalculateDistance(currentNode.Position, neighbor.Position);

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                    else if (tentativeG >= neighbor.G)
                    {
                        continue;
                    }

                    neighbor.Parent = currentNode;
                    neighbor.G = tentativeG;
                    neighbor.H = CalculateHeuristic(neighbor.Position, goal);
                }
            }

            return false;
        }

        /// <summary>
        /// 获取相邻节点
        /// </summary>
        private List<Node> GetNeighbors(Node node, Node[,] gridMap)
        {
            var neighbors = new List<Node>();
            var gridPos = WorldToGrid(node.Position);

            // 8方向相邻节点
            int[] dx = { -1, -1, -1, 0, 0, 1, 1, 1 };
            int[] dy = { -1, 0, 1, -1, 1, -1, 0, 1 };

            for (int i = 0; i < 8; i++)
            {
                int newX = gridPos.X + dx[i];
                int newY = gridPos.Y + dy[i];

                if (newX >= 0 && newX < gridMap.GetLength(0) && 
                    newY >= 0 && newY < gridMap.GetLength(1))
                {
                    neighbors.Add(gridMap[newX, newY]);
                }
            }

            return neighbors;
        }

        /// <summary>
        /// 计算启发式距离
        /// </summary>
        private float CalculateHeuristic(PointF current, PointF goal)
        {
            float dx = Math.Abs(current.X - goal.X);
            float dy = Math.Abs(current.Y - goal.Y);
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// 计算启发式距离（栅格坐标）
        /// </summary>
        private float CalculateHeuristic(Point current, Point goal)
        {
            int dx = Math.Abs(current.X - goal.X);
            int dy = Math.Abs(current.Y - goal.Y);
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// 计算两点间距离
        /// </summary>
        private float CalculateDistance(PointF a, PointF b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// 构建路径
        /// </summary>
        private void BuildPath(Node[,] gridMap, Point start, Point goal)
        {
            var goalNode = gridMap[goal.X, goal.Y];
            var currentNode = goalNode;

            while (currentNode != null)
            {
                path.Insert(0, currentNode.Position);
                currentNode = currentNode.Parent;
            }

            pathFound = true;
        }

        /// <summary>
        /// 世界坐标转栅格坐标
        /// </summary>
        private Point WorldToGrid(PointF worldPos)
        {
            int x = (int)((worldPos.X + mapOrigin.X) / resolution);
            int y = (int)((worldPos.Y + mapOrigin.Y) / resolution);
            return new Point(x, y);
        }

        /// <summary>
        /// 栅格坐标转世界坐标
        /// </summary>
        private PointF GridToWorld(Point gridPos)
        {
            float x = gridPos.X * resolution - mapOrigin.X;
            float y = gridPos.Y * resolution - mapOrigin.Y;
            return new PointF(x, y);
        }

        /// <summary>
        /// 检查栅格位置是否有效
        /// </summary>
        private bool IsValidGridPosition(Point gridPos, Node[,] gridMap)
        {
            return gridPos.X >= 0 && gridPos.X < gridMap.GetLength(0) &&
                   gridPos.Y >= 0 && gridPos.Y < gridMap.GetLength(1);
        }

        /// <summary>
        /// 检查位置是否有效
        /// </summary>
        private bool IsPositionValid(PointF position, AdvancedSLAMMap slamMap)
        {
            // 检查是否在地图范围内
            if (position.X < -mapOrigin.X || position.X > mapWidth - mapOrigin.X ||
                position.Y < -mapOrigin.Y || position.Y > mapHeight - mapOrigin.Y)
            {
                return false;
            }

            // 检查是否在障碍物附近
            var obstacles = slamMap.GetObstaclePoints();
            float safetyDistance = 0.2f; // 20cm安全距离

            foreach (var obstacle in obstacles)
            {
                float distance = (float)Math.Sqrt(
                    Math.Pow(position.X - obstacle.X, 2) + 
                    Math.Pow(position.Y - obstacle.Y, 2)
                );

                if (distance < safetyDistance)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// 获取规划路径
        /// </summary>
        public List<PointF> GetPath()
        {
            return new List<PointF>(path);
        }

        /// <summary>
        /// 获取路径长度
        /// </summary>
        public float GetPathLength()
        {
            if (path.Count < 2) return 0;

            float length = 0;
            for (int i = 0; i < path.Count - 1; i++)
            {
                length += CalculateDistance(path[i], path[i + 1]);
            }
            return length;
        }

        /// <summary>
        /// 简化路径
        /// </summary>
        public void SimplifyPath()
        {
            if (path.Count < 3) return;

            var simplifiedPath = new List<PointF>();
            simplifiedPath.Add(path[0]);

            for (int i = 1; i < path.Count - 1; i++)
            {
                // 检查是否可以跳过中间点
                if (!CanSkipPoint(simplifiedPath.Last(), path[i], path[i + 1]))
                {
                    simplifiedPath.Add(path[i]);
                }
            }

            simplifiedPath.Add(path.Last());
            path = simplifiedPath;
        }

        /// <summary>
        /// 检查是否可以跳过中间点
        /// </summary>
        private bool CanSkipPoint(PointF start, PointF middle, PointF end)
        {
            // 简化的直线检查
            // 实际应用中应该检查路径上是否有障碍物
            float distance1 = CalculateDistance(start, middle);
            float distance2 = CalculateDistance(middle, end);
            float directDistance = CalculateDistance(start, end);

            return Math.Abs(distance1 + distance2 - directDistance) < 0.1f;
        }

        /// <summary>
        /// 设置地图参数
        /// </summary>
        public void SetMapParameters(float resolution, float width, float height, PointF origin)
        {
            this.resolution = resolution;
            this.mapWidth = width;
            this.mapHeight = height;
            this.mapOrigin = origin;
        }

        /// <summary>
        /// 清空路径
        /// </summary>
        public void ClearPath()
        {
            path.Clear();
            pathFound = false;
        }

        /// <summary>
        /// 获取路径统计信息
        /// </summary>
        public string GetPathStatistics()
        {
            return $"路径点数: {path.Count}, 路径长度: {GetPathLength():F2}m, 路径找到: {pathFound}";
        }
    }
}
