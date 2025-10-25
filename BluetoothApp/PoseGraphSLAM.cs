using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace BluetoothApp
{
    /// <summary>
    /// 位姿图优化SLAM算法实现
    /// </summary>
    public class PoseGraphSLAM
    {
        /// <summary>
        /// 位姿节点
        /// </summary>
        public class PoseNode
        {
            public int Id { get; set; }
            public float X { get; set; }
            public float Y { get; set; }
            public float Theta { get; set; }
            public DateTime Timestamp { get; set; }
            public List<PointF> LidarPoints { get; set; }
            public bool IsKeyFrame { get; set; }

            public PoseNode(int id, float x, float y, float theta, List<PointF> lidarPoints = null)
            {
                Id = id;
                X = x;
                Y = y;
                Theta = theta;
                Timestamp = DateTime.Now;
                LidarPoints = lidarPoints ?? new List<PointF>();
                IsKeyFrame = false;
            }
        }

        /// <summary>
        /// 边约束
        /// </summary>
        public class Edge
        {
            public int FromId { get; set; }
            public int ToId { get; set; }
            public float DeltaX { get; set; }
            public float DeltaY { get; set; }
            public float DeltaTheta { get; set; }
            public float Information { get; set; }
            public EdgeType Type { get; set; }

            public Edge(int fromId, int toId, float deltaX, float deltaY, float deltaTheta, float information, EdgeType type)
            {
                FromId = fromId;
                ToId = toId;
                DeltaX = deltaX;
                DeltaY = deltaY;
                DeltaTheta = deltaTheta;
                Information = information;
                Type = type;
            }
        }

        public enum EdgeType
        {
            Odometry,    // 里程计约束
            LoopClosure, // 回环检测约束
            LidarMatch   // 激光匹配约束
        }

        // SLAM参数
        private float keyFrameDistance = 0.5f;      // 关键帧距离阈值
        private float keyFrameAngle = 0.3f;         // 关键帧角度阈值
        private float loopClosureDistance = 2.0f;   // 回环检测距离阈值
        private float loopClosureAngle = 0.5f;       // 回环检测角度阈值
        private int maxKeyFrames = 100;              // 最大关键帧数量
        private float lidarRange = 5.0f;             // 激光雷达范围
        private float lidarResolution = 0.1f;       // 激光雷达分辨率

        // 图结构
        private List<PoseNode> nodes;
        private List<Edge> edges;
        private int nextNodeId;
        private int lastKeyFrameId;

        // 优化后的位姿
        private List<PoseNode> optimizedNodes;

        // 地图数据
        private List<PointF> globalMapPoints;
        private List<PointF> obstaclePoints;
        private List<PointF> freeSpacePoints;

        // 回环检测
        private Dictionary<int, List<int>> loopCandidates;

        public PoseGraphSLAM()
        {
            nodes = new List<PoseNode>();
            edges = new List<Edge>();
            optimizedNodes = new List<PoseNode>();
            globalMapPoints = new List<PointF>();
            obstaclePoints = new List<PointF>();
            freeSpacePoints = new List<PointF>();
            loopCandidates = new Dictionary<int, List<int>>();
            nextNodeId = 0;
            lastKeyFrameId = -1;
        }

        /// <summary>
        /// 添加新的位姿节点
        /// </summary>
        public int AddPose(float x, float y, float theta, List<PointF> lidarData = null)
        {
            var node = new PoseNode(nextNodeId++, x, y, theta, lidarData);
            nodes.Add(node);

            // 检查是否应该创建关键帧
            if (ShouldCreateKeyFrame(node))
            {
                node.IsKeyFrame = true;
                lastKeyFrameId = node.Id;
                
                // 添加里程计约束
                if (nodes.Count > 1)
                {
                    var prevNode = nodes[nodes.Count - 2];
                    float deltaX = x - prevNode.X;
                    float deltaY = y - prevNode.Y;
                    float deltaTheta = NormalizeAngle(theta - prevNode.Theta);
                    
                    AddEdge(prevNode.Id, node.Id, deltaX, deltaY, deltaTheta, 1.0f, EdgeType.Odometry);
                }

                // 更新地图
                UpdateMap(node);
                
                // 回环检测
                DetectLoopClosure(node);
            }

            return node.Id;
        }

        /// <summary>
        /// 添加边约束
        /// </summary>
        public void AddEdge(int fromId, int toId, float deltaX, float deltaY, float deltaTheta, float information, EdgeType type)
        {
            var edge = new Edge(fromId, toId, deltaX, deltaY, deltaTheta, information, type);
            edges.Add(edge);
        }

        /// <summary>
        /// 检查是否应该创建关键帧
        /// </summary>
        private bool ShouldCreateKeyFrame(PoseNode node)
        {
            if (lastKeyFrameId == -1) return true;

            var lastKeyFrame = nodes.FirstOrDefault(n => n.Id == lastKeyFrameId);
            if (lastKeyFrame == null) return true;

            float distance = (float)Math.Sqrt(
                Math.Pow(node.X - lastKeyFrame.X, 2) + 
                Math.Pow(node.Y - lastKeyFrame.Y, 2)
            );
            float angleDiff = Math.Abs(NormalizeAngle(node.Theta - lastKeyFrame.Theta));

            return distance > keyFrameDistance || angleDiff > keyFrameAngle;
        }

        /// <summary>
        /// 更新地图
        /// </summary>
        private void UpdateMap(PoseNode node)
        {
            if (node.LidarPoints == null || node.LidarPoints.Count == 0) return;

            // 将激光点转换到全局坐标系
            foreach (var point in node.LidarPoints)
            {
                // 转换到全局坐标系
                float globalX = node.X + point.X * (float)Math.Cos(node.Theta) - point.Y * (float)Math.Sin(node.Theta);
                float globalY = node.Y + point.X * (float)Math.Sin(node.Theta) + point.Y * (float)Math.Cos(node.Theta);

                var globalPoint = new PointF(globalX, globalY);
                
                // 添加到全局地图
                if (!IsPointTooClose(globalPoint, globalMapPoints))
                {
                    globalMapPoints.Add(globalPoint);
                    obstaclePoints.Add(globalPoint);
                }
            }

            // 更新自由空间
            UpdateFreeSpace(node);
        }

        /// <summary>
        /// 更新自由空间
        /// </summary>
        private void UpdateFreeSpace(PoseNode node)
        {
            if (node.LidarPoints == null) return;

            // 在机器人和障碍物之间添加自由空间点
            foreach (var lidarPoint in node.LidarPoints)
            {
                float distance = (float)Math.Sqrt(lidarPoint.X * lidarPoint.X + lidarPoint.Y * lidarPoint.Y);
                if (distance > 0.1f) // 避免在机器人位置添加点
                {
                    int steps = (int)(distance / lidarResolution);
                    for (int i = 1; i < steps; i++)
                    {
                        float t = (float)i / steps;
                        float localX = lidarPoint.X * t;
                        float localY = lidarPoint.Y * t;

                        // 转换到全局坐标系
                        float globalX = node.X + localX * (float)Math.Cos(node.Theta) - localY * (float)Math.Sin(node.Theta);
                        float globalY = node.Y + localX * (float)Math.Sin(node.Theta) + localY * (float)Math.Cos(node.Theta);

                        var freePoint = new PointF(globalX, globalY);
                        if (!IsPointTooClose(freePoint, freeSpacePoints))
                        {
                            freeSpacePoints.Add(freePoint);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 回环检测
        /// </summary>
        private void DetectLoopClosure(PoseNode currentNode)
        {
            if (nodes.Count < 10) return; // 需要足够的历史数据

            // 寻找可能的回环候选
            var candidates = new List<int>();
            for (int i = 0; i < nodes.Count - 10; i++) // 避免与最近的关键帧匹配
            {
                var candidate = nodes[i];
                if (!candidate.IsKeyFrame) continue;

                float distance = (float)Math.Sqrt(
                    Math.Pow(currentNode.X - candidate.X, 2) + 
                    Math.Pow(currentNode.Y - candidate.Y, 2)
                );

                if (distance < loopClosureDistance)
                {
                    float angleDiff = Math.Abs(NormalizeAngle(currentNode.Theta - candidate.Theta));
                    if (angleDiff < loopClosureAngle)
                    {
                        candidates.Add(candidate.Id);
                    }
                }
            }

            // 对候选进行激光匹配验证
            foreach (var candidateId in candidates)
            {
                if (ValidateLoopClosure(currentNode, nodes.First(n => n.Id == candidateId)))
                {
                    // 添加回环约束
                    var candidate = nodes.First(n => n.Id == candidateId);
                    float deltaX = currentNode.X - candidate.X;
                    float deltaY = currentNode.Y - candidate.Y;
                    float deltaTheta = NormalizeAngle(currentNode.Theta - candidate.Theta);
                    
                    AddEdge(candidateId, currentNode.Id, deltaX, deltaY, deltaTheta, 10.0f, EdgeType.LoopClosure);
                    
                    // 触发图优化
                    OptimizeGraph();
                    break;
                }
            }
        }

        /// <summary>
        /// 验证回环检测
        /// </summary>
        private bool ValidateLoopClosure(PoseNode current, PoseNode candidate)
        {
            if (current.LidarPoints == null || candidate.LidarPoints == null) return false;
            if (current.LidarPoints.Count < 10 || candidate.LidarPoints.Count < 10) return false;

            // 简单的激光匹配验证
            int matchCount = 0;
            float matchThreshold = 0.2f;

            foreach (var currentPoint in current.LidarPoints)
            {
                foreach (var candidatePoint in candidate.LidarPoints)
                {
                    float distance = (float)Math.Sqrt(
                        Math.Pow(currentPoint.X - candidatePoint.X, 2) + 
                        Math.Pow(currentPoint.Y - candidatePoint.Y, 2)
                    );
                    
                    if (distance < matchThreshold)
                    {
                        matchCount++;
                        break;
                    }
                }
            }

            float matchRatio = (float)matchCount / current.LidarPoints.Count;
            return matchRatio > 0.3f; // 30%的匹配率
        }

        /// <summary>
        /// 图优化
        /// </summary>
        public void OptimizeGraph()
        {
            if (nodes.Count < 3) return;

            // 简化的图优化算法
            optimizedNodes.Clear();
            
            // 复制原始节点
            foreach (var node in nodes)
            {
                optimizedNodes.Add(new PoseNode(node.Id, node.X, node.Y, node.Theta, node.LidarPoints)
                {
                    Timestamp = node.Timestamp,
                    IsKeyFrame = node.IsKeyFrame
                });
            }

            // 简单的迭代优化
            for (int iteration = 0; iteration < 10; iteration++)
            {
                OptimizeIteration();
            }
        }

        /// <summary>
        /// 单次优化迭代
        /// </summary>
        private void OptimizeIteration()
        {
            // 简化的梯度下降优化
            float learningRate = 0.1f;
            
            foreach (var edge in edges)
            {
                var fromNode = optimizedNodes.FirstOrDefault(n => n.Id == edge.FromId);
                var toNode = optimizedNodes.FirstOrDefault(n => n.Id == edge.ToId);
                
                if (fromNode == null || toNode == null) continue;

                // 计算误差
                float errorX = (toNode.X - fromNode.X) - edge.DeltaX;
                float errorY = (toNode.Y - fromNode.Y) - edge.DeltaY;
                float errorTheta = NormalizeAngle(toNode.Theta - fromNode.Theta - edge.DeltaTheta);

                // 更新位姿
                float weight = edge.Information * learningRate;
                
                toNode.X -= errorX * weight;
                toNode.Y -= errorY * weight;
                toNode.Theta = NormalizeAngle(toNode.Theta - errorTheta * weight);
            }
        }

        /// <summary>
        /// 获取全局地图点
        /// </summary>
        public List<PointF> GetGlobalMapPoints()
        {
            return new List<PointF>(globalMapPoints);
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
        /// 获取优化后的轨迹
        /// </summary>
        public List<PointF> GetOptimizedTrajectory()
        {
            var trajectory = new List<PointF>();
            var nodesToUse = optimizedNodes.Count > 0 ? optimizedNodes : nodes;
            
            foreach (var node in nodesToUse)
            {
                trajectory.Add(new PointF(node.X, node.Y));
            }
            
            return trajectory;
        }

        /// <summary>
        /// 获取当前位姿
        /// </summary>
        public PoseNode GetCurrentPose()
        {
            return nodes.LastOrDefault();
        }

        /// <summary>
        /// 获取所有节点
        /// </summary>
        public List<PoseNode> GetAllNodes()
        {
            return new List<PoseNode>(nodes);
        }

        /// <summary>
        /// 获取所有边
        /// </summary>
        public List<Edge> GetAllEdges()
        {
            return new List<Edge>(edges);
        }

        /// <summary>
        /// 检查点是否太接近现有点
        /// </summary>
        private bool IsPointTooClose(PointF newPoint, List<PointF> existingPoints)
        {
            float minDistance = 0.05f; // 5cm最小距离
            
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
        /// 角度归一化到[-π, π]
        /// </summary>
        private float NormalizeAngle(float angle)
        {
            while (angle > Math.PI) angle -= 2 * (float)Math.PI;
            while (angle < -Math.PI) angle += 2 * (float)Math.PI;
            return angle;
        }

        /// <summary>
        /// 清空地图
        /// </summary>
        public void Clear()
        {
            nodes.Clear();
            edges.Clear();
            optimizedNodes.Clear();
            globalMapPoints.Clear();
            obstaclePoints.Clear();
            freeSpacePoints.Clear();
            loopCandidates.Clear();
            nextNodeId = 0;
            lastKeyFrameId = -1;
        }

        /// <summary>
        /// 获取地图统计信息
        /// </summary>
        public string GetMapStatistics()
        {
            return $"节点数: {nodes.Count}, 边数: {edges.Count}, 地图点数: {globalMapPoints.Count}, 障碍物点数: {obstaclePoints.Count}";
        }
    }
}
