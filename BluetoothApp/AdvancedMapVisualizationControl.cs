using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;

namespace BluetoothApp
{
    /// <summary>
    /// 高级地图可视化控件
    /// 支持多种地图类型和实时渲染
    /// </summary>
    public class AdvancedMapVisualizationControl : UserControl
    {
        // 地图数据
        private AdvancedSLAMMap slamMap;
        private PoseGraphSLAM poseGraphSLAM;
        
        // 显示设置
        private bool showOccupancyGrid = true;
        private bool showProbabilityGrid = false;
        private bool showPointCloud = true;
        private bool showTrajectory = true;
        private bool showOptimizedTrajectory = true;
        private bool showObstacles = true;
        private bool showFreeSpace = false;
        private bool showLoopClosures = true;
        private bool showPoseGraph = false;
        
        // 颜色设置
        private Color unknownColor = Color.Gray;
        private Color freeColor = Color.White;
        private Color occupiedColor = Color.Black;
        private Color obstacleColor = Color.Red;
        private Color trajectoryColor = Color.Blue;
        private Color optimizedTrajectoryColor = Color.Green;
        private Color robotColor = Color.Yellow;
        private Color loopClosureColor = Color.Magenta;
        private Color poseGraphColor = Color.Cyan;
        
        // 缩放和平移
        private float scale = 1.0f;
        private PointF offset = new PointF(0, 0);
        private bool isDragging = false;
        private PointF lastMousePos;
        
        // 机器人状态
        private PointF robotPosition = new PointF(0, 0);
        private float robotOrientation = 0.0f;
        
        // 渲染优化
        private Bitmap cachedMap;
        private bool mapNeedsUpdate = true;
        private Timer updateTimer;
        
        // 统计信息
        private string mapStatistics = "";

        public AdvancedMapVisualizationControl()
        {
            this.DoubleBuffered = true;
            this.BackColor = Color.LightGray;
            this.BorderStyle = BorderStyle.FixedSingle;
            
            // 初始化更新定时器
            updateTimer = new Timer();
            updateTimer.Interval = 100; // 100ms更新一次
            updateTimer.Tick += UpdateTimer_Tick;
            updateTimer.Start();
            
            // 启用鼠标事件
            this.MouseDown += AdvancedMapVisualizationControl_MouseDown;
            this.MouseMove += AdvancedMapVisualizationControl_MouseMove;
            this.MouseUp += AdvancedMapVisualizationControl_MouseUp;
            this.MouseWheel += AdvancedMapVisualizationControl_MouseWheel;
            this.Paint += AdvancedMapVisualizationControl_Paint;
        }

        /// <summary>
        /// 更新地图数据
        /// </summary>
        public void UpdateMap(AdvancedSLAMMap map, PointF robotPos, float robotOrient)
        {
            slamMap = map;
            robotPosition = robotPos;
            robotOrientation = robotOrient;
            
            if (slamMap != null)
            {
                poseGraphSLAM = slamMap.GetPoseGraphSLAM();
                mapStatistics = slamMap.GetMapStatistics();
            }
            
            mapNeedsUpdate = true;
        }

        /// <summary>
        /// 设置显示选项
        /// </summary>
        public void SetDisplayOptions(bool occupancy, bool probability, bool pointCloud, bool trajectory, 
                                    bool optimizedTrajectory, bool obstacles, bool freeSpace, bool loopClosures, bool poseGraph)
        {
            showOccupancyGrid = occupancy;
            showProbabilityGrid = probability;
            showPointCloud = pointCloud;
            showTrajectory = trajectory;
            showOptimizedTrajectory = optimizedTrajectory;
            showObstacles = obstacles;
            showFreeSpace = freeSpace;
            showLoopClosures = loopClosures;
            showPoseGraph = poseGraph;
            
            mapNeedsUpdate = true;
        }

        private void UpdateTimer_Tick(object sender, EventArgs e)
        {
            if (mapNeedsUpdate)
            {
                this.Invalidate();
                mapNeedsUpdate = false;
            }
        }

        private void AdvancedMapVisualizationControl_Paint(object sender, PaintEventArgs e)
        {
            if (slamMap == null) return;
            
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;
            
            // 应用变换
            g.TranslateTransform(offset.X, offset.Y);
            g.ScaleTransform(scale, scale);
            
            // 绘制背景
            g.Clear(Color.LightGray);
            
            // 绘制地图
            DrawMap(g);
            
            // 绘制机器人
            DrawRobot(g);
            
            // 绘制统计信息
            DrawStatistics(g);
        }

        /// <summary>
        /// 绘制地图
        /// </summary>
        private void DrawMap(Graphics g)
        {
            if (slamMap == null) return;
            
            var config = slamMap.GetConfig();
            
            // 绘制占用栅格地图
            if (showOccupancyGrid)
            {
                DrawOccupancyGrid(g, config);
            }
            
            // 绘制概率栅格地图
            if (showProbabilityGrid)
            {
                DrawProbabilityGrid(g, config);
            }
            
            // 绘制点云
            if (showPointCloud)
            {
                DrawPointCloud(g);
            }
            
            // 绘制障碍物点
            if (showObstacles)
            {
                DrawObstaclePoints(g);
            }
            
            // 绘制自由空间点
            if (showFreeSpace)
            {
                DrawFreeSpacePoints(g);
            }
            
            // 绘制轨迹
            if (showTrajectory)
            {
                DrawTrajectory(g);
            }
            
            // 绘制优化后的轨迹
            if (showOptimizedTrajectory)
            {
                DrawOptimizedTrajectory(g);
            }
            
            // 绘制回环检测
            if (showLoopClosures)
            {
                DrawLoopClosures(g);
            }
            
            // 绘制位姿图
            if (showPoseGraph)
            {
                DrawPoseGraph(g);
            }
        }

        /// <summary>
        /// 绘制占用栅格地图
        /// </summary>
        private void DrawOccupancyGrid(Graphics g, AdvancedSLAMMap.MapConfig config)
        {
            var occupancyGrid = slamMap.GetOccupancyGrid();
            int gridWidth = occupancyGrid.GetLength(0);
            int gridHeight = occupancyGrid.GetLength(1);
            
            float cellSize = config.Resolution * 10; // 放大显示
            
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    Color cellColor;
                    switch ((AdvancedSLAMMap.GridState)occupancyGrid[x, y])
                    {
                        case AdvancedSLAMMap.GridState.Unknown:
                            cellColor = unknownColor;
                            break;
                        case AdvancedSLAMMap.GridState.Free:
                            cellColor = freeColor;
                            break;
                        case AdvancedSLAMMap.GridState.Occupied:
                        case AdvancedSLAMMap.GridState.Obstacle:
                            cellColor = occupiedColor;
                            break;
                        default:
                            cellColor = unknownColor;
                            break;
                    }
                    
                    float worldX = x * config.Resolution - config.Origin.X;
                    float worldY = y * config.Resolution - config.Origin.Y;
                    
                    RectangleF cellRect = new RectangleF(worldX, worldY, cellSize, cellSize);
                    
                    using (Brush brush = new SolidBrush(cellColor))
                    {
                        g.FillRectangle(brush, cellRect);
                    }
                }
            }
        }

        /// <summary>
        /// 绘制概率栅格地图
        /// </summary>
        private void DrawProbabilityGrid(Graphics g, AdvancedSLAMMap.MapConfig config)
        {
            var probabilityGrid = slamMap.GetProbabilityGrid();
            int gridWidth = probabilityGrid.GetLength(0);
            int gridHeight = probabilityGrid.GetLength(1);
            
            float cellSize = config.Resolution * 10;
            
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    float probability = probabilityGrid[x, y];
                    
                    // 根据概率设置颜色
                    Color cellColor;
                    if (probability < 0.3f)
                    {
                        // 低概率 - 自由空间
                        int intensity = (int)(255 * (0.3f - probability) / 0.3f);
                        cellColor = Color.FromArgb(intensity, 255, intensity);
                    }
                    else if (probability > 0.7f)
                    {
                        // 高概率 - 占用
                        int intensity = (int)(255 * (probability - 0.7f) / 0.3f);
                        cellColor = Color.FromArgb(255, intensity, intensity);
                    }
                    else
                    {
                        // 中等概率 - 未知
                        cellColor = unknownColor;
                    }
                    
                    float worldX = x * config.Resolution - config.Origin.X;
                    float worldY = y * config.Resolution - config.Origin.Y;
                    
                    RectangleF cellRect = new RectangleF(worldX, worldY, cellSize, cellSize);
                    
                    using (Brush brush = new SolidBrush(cellColor))
                    {
                        g.FillRectangle(brush, cellRect);
                    }
                }
            }
        }

        /// <summary>
        /// 绘制点云
        /// </summary>
        private void DrawPointCloud(Graphics g)
        {
            var pointCloud = slamMap.GetPointCloud();
            if (pointCloud == null || pointCloud.Count == 0) return;
            
            float pointSize = 2.0f / scale;
            using (Brush brush = new SolidBrush(obstacleColor))
            {
                foreach (var point in pointCloud)
                {
                    RectangleF pointRect = new RectangleF(
                        point.X - pointSize / 2,
                        point.Y - pointSize / 2,
                        pointSize,
                        pointSize
                    );
                    g.FillEllipse(brush, pointRect);
                }
            }
        }

        /// <summary>
        /// 绘制障碍物点
        /// </summary>
        private void DrawObstaclePoints(Graphics g)
        {
            var obstacles = slamMap.GetObstaclePoints();
            if (obstacles == null || obstacles.Count == 0) return;
            
            float pointSize = 3.0f / scale;
            using (Brush brush = new SolidBrush(obstacleColor))
            {
                foreach (var point in obstacles)
                {
                    RectangleF pointRect = new RectangleF(
                        point.X - pointSize / 2,
                        point.Y - pointSize / 2,
                        pointSize,
                        pointSize
                    );
                    g.FillEllipse(brush, pointRect);
                }
            }
        }

        /// <summary>
        /// 绘制自由空间点
        /// </summary>
        private void DrawFreeSpacePoints(Graphics g)
        {
            var freeSpace = slamMap.GetFreeSpacePoints();
            if (freeSpace == null || freeSpace.Count == 0) return;
            
            float pointSize = 1.0f / scale;
            using (Brush brush = new SolidBrush(freeColor))
            {
                foreach (var point in freeSpace)
                {
                    RectangleF pointRect = new RectangleF(
                        point.X - pointSize / 2,
                        point.Y - pointSize / 2,
                        pointSize,
                        pointSize
                    );
                    g.FillEllipse(brush, pointRect);
                }
            }
        }

        /// <summary>
        /// 绘制轨迹
        /// </summary>
        private void DrawTrajectory(Graphics g)
        {
            var trajectory = slamMap.GetRobotTrajectory();
            if (trajectory == null || trajectory.Count < 2) return;
            
            using (Pen pen = new Pen(trajectoryColor, 2.0f / scale))
            {
                for (int i = 0; i < trajectory.Count - 1; i++)
                {
                    g.DrawLine(pen, trajectory[i], trajectory[i + 1]);
                }
            }
        }

        /// <summary>
        /// 绘制优化后的轨迹
        /// </summary>
        private void DrawOptimizedTrajectory(Graphics g)
        {
            var optimizedTrajectory = slamMap.GetOptimizedTrajectory();
            if (optimizedTrajectory == null || optimizedTrajectory.Count < 2) return;
            
            using (Pen pen = new Pen(optimizedTrajectoryColor, 3.0f / scale))
            {
                for (int i = 0; i < optimizedTrajectory.Count - 1; i++)
                {
                    g.DrawLine(pen, optimizedTrajectory[i], optimizedTrajectory[i + 1]);
                }
            }
        }

        /// <summary>
        /// 绘制回环检测
        /// </summary>
        private void DrawLoopClosures(Graphics g)
        {
            if (poseGraphSLAM == null) return;
            
            var edges = poseGraphSLAM.GetAllEdges();
            var nodes = poseGraphSLAM.GetAllNodes();
            
            using (Pen pen = new Pen(loopClosureColor, 2.0f / scale))
            {
                foreach (var edge in edges)
                {
                    if (edge.Type == PoseGraphSLAM.EdgeType.LoopClosure)
                    {
                        var fromNode = nodes.FirstOrDefault(n => n.Id == edge.FromId);
                        var toNode = nodes.FirstOrDefault(n => n.Id == edge.ToId);
                        
                        if (fromNode != null && toNode != null)
                        {
                            g.DrawLine(pen, new PointF(fromNode.X, fromNode.Y), new PointF(toNode.X, toNode.Y));
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 绘制位姿图
        /// </summary>
        private void DrawPoseGraph(Graphics g)
        {
            if (poseGraphSLAM == null) return;
            
            var nodes = poseGraphSLAM.GetAllNodes();
            var edges = poseGraphSLAM.GetAllEdges();
            
            // 绘制节点
            float nodeSize = 4.0f / scale;
            using (Brush brush = new SolidBrush(poseGraphColor))
            {
                foreach (var node in nodes)
                {
                    if (node.IsKeyFrame)
                    {
                        RectangleF nodeRect = new RectangleF(
                            node.X - nodeSize / 2,
                            node.Y - nodeSize / 2,
                            nodeSize,
                            nodeSize
                        );
                        g.FillEllipse(brush, nodeRect);
                    }
                }
            }
            
            // 绘制边
            using (Pen pen = new Pen(poseGraphColor, 1.0f / scale))
            {
                foreach (var edge in edges)
                {
                    var fromNode = nodes.FirstOrDefault(n => n.Id == edge.FromId);
                    var toNode = nodes.FirstOrDefault(n => n.Id == edge.ToId);
                    
                    if (fromNode != null && toNode != null)
                    {
                        g.DrawLine(pen, new PointF(fromNode.X, fromNode.Y), new PointF(toNode.X, toNode.Y));
                    }
                }
            }
        }

        /// <summary>
        /// 绘制机器人
        /// </summary>
        private void DrawRobot(Graphics g)
        {
            float robotSize = 0.3f;
            float robotRadius = robotSize / 2;
            
            // 绘制机器人主体
            using (Brush brush = new SolidBrush(robotColor))
            {
                RectangleF robotRect = new RectangleF(
                    robotPosition.X - robotRadius,
                    robotPosition.Y - robotRadius,
                    robotSize,
                    robotSize
                );
                g.FillEllipse(brush, robotRect);
            }
            
            // 绘制机器人朝向
            float arrowLength = robotSize;
            float arrowX = robotPosition.X + (float)Math.Cos(robotOrientation) * arrowLength;
            float arrowY = robotPosition.Y + (float)Math.Sin(robotOrientation) * arrowLength;
            
            using (Pen pen = new Pen(Color.DarkRed, 3.0f / scale))
            {
                g.DrawLine(pen, robotPosition, new PointF(arrowX, arrowY));
            }
        }

        /// <summary>
        /// 绘制统计信息
        /// </summary>
        private void DrawStatistics(Graphics g)
        {
            if (string.IsNullOrEmpty(mapStatistics)) return;
            
            // 重置变换
            g.ResetTransform();
            
            using (Brush brush = new SolidBrush(Color.Black))
            using (Font font = new Font("Arial", 8))
            {
                string[] lines = mapStatistics.Split('\n');
                float y = 10;
                foreach (string line in lines)
                {
                    g.DrawString(line, font, brush, 10, y);
                    y += 15;
                }
            }
        }

        /// <summary>
        /// 鼠标按下事件
        /// </summary>
        private void AdvancedMapVisualizationControl_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                isDragging = true;
                lastMousePos = e.Location;
            }
        }

        /// <summary>
        /// 鼠标移动事件
        /// </summary>
        private void AdvancedMapVisualizationControl_MouseMove(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                PointF delta = new PointF(e.Location.X - lastMousePos.X, e.Location.Y - lastMousePos.Y);
                offset.X += delta.X;
                offset.Y += delta.Y;
                lastMousePos = e.Location;
                this.Invalidate();
            }
        }

        /// <summary>
        /// 鼠标释放事件
        /// </summary>
        private void AdvancedMapVisualizationControl_MouseUp(object sender, MouseEventArgs e)
        {
            isDragging = false;
        }

        /// <summary>
        /// 鼠标滚轮事件
        /// </summary>
        private void AdvancedMapVisualizationControl_MouseWheel(object sender, MouseEventArgs e)
        {
            float zoomFactor = 1.1f;
            if (e.Delta > 0)
            {
                scale *= zoomFactor;
            }
            else
            {
                scale /= zoomFactor;
            }
            
            // 限制缩放范围
            scale = Math.Max(0.1f, Math.Min(10.0f, scale));
            
            this.Invalidate();
        }

        /// <summary>
        /// 重置视图
        /// </summary>
        public void ResetView()
        {
            scale = 1.0f;
            offset = new PointF(0, 0);
            this.Invalidate();
        }

        /// <summary>
        /// 适应窗口大小
        /// </summary>
        public void FitToWindow()
        {
            if (slamMap == null) return;
            
            var config = slamMap.GetConfig();
            float mapWidth = config.Width;
            float mapHeight = config.Height;
            
            float scaleX = (this.Width - 20) / mapWidth;
            float scaleY = (this.Height - 20) / mapHeight;
            scale = Math.Min(scaleX, scaleY);
            
            offset.X = (this.Width - mapWidth * scale) / 2;
            offset.Y = (this.Height - mapHeight * scale) / 2;
            
            this.Invalidate();
        }

        /// <summary>
        /// 设置颜色主题
        /// </summary>
        public void SetColorTheme(Color unknown, Color free, Color occupied, Color obstacle, 
                                 Color trajectory, Color robot, Color loopClosure)
        {
            unknownColor = unknown;
            freeColor = free;
            occupiedColor = occupied;
            obstacleColor = obstacle;
            trajectoryColor = trajectory;
            robotColor = robot;
            loopClosureColor = loopClosure;
            
            mapNeedsUpdate = true;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                updateTimer?.Stop();
                updateTimer?.Dispose();
                cachedMap?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
