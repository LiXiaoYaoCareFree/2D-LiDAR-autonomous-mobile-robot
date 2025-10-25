using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;

namespace BluetoothApp
{
    /// <summary>
    /// 地图可视化控件
    /// </summary>
    public class MapVisualizationControl : UserControl
    {
        // 地图数据
        private SLAMMap slamMap;
        private List<PointF> robotPath;
        private List<PointF> scanPoints;
        private List<PointF> obstaclePoints;
        private List<Tuple<PointF, PointF>> radarRays;
        
        // 显示设置
        private bool showOriginalMap = true;
        private bool showSLAMMap = true;
        private bool showExplorationPath = true;
        private bool showLidarData = true;
        private bool showRadarRays = false;
        
        // 颜色设置
        private Color unknownColor = Color.Gray;
        private Color freeColor = Color.White;
        private Color obstacleColor = Color.Black;
        private Color robotPathColor = Color.Blue;
        private Color scanPointColor = Color.Red;
        private Color radarRayColor = Color.Yellow;
        private Color robotColor = Color.Green;
        
        // 缩放和平移
        private float scale = 1.0f;
        private PointF offset = new PointF(0, 0);
        private bool isDragging = false;
        private PointF lastMousePos;
        
        // 机器人位置
        private PointF robotPosition = new PointF(0, 0);
        private float robotOrientation = 0.0f;

        public MapVisualizationControl()
        {
            this.DoubleBuffered = true;
            this.BackColor = Color.White;
            this.BorderStyle = BorderStyle.FixedSingle;
            
            // 启用鼠标事件
            this.MouseDown += MapVisualizationControl_MouseDown;
            this.MouseMove += MapVisualizationControl_MouseMove;
            this.MouseUp += MapVisualizationControl_MouseUp;
            this.MouseWheel += MapVisualizationControl_MouseWheel;
        }

        /// <summary>
        /// 更新地图数据
        /// </summary>
        public void UpdateMap(SLAMMap map, PointF robotPos, float robotOrient)
        {
            slamMap = map;
            robotPosition = robotPos;
            robotOrientation = robotOrient;
            
            if (slamMap != null)
            {
                robotPath = slamMap.GetRobotPath();
                scanPoints = slamMap.GetScanPoints();
                obstaclePoints = slamMap.GetObstaclePoints();
                radarRays = slamMap.GetRadarRays();
            }
            
            this.Invalidate();
        }

        /// <summary>
        /// 设置显示选项
        /// </summary>
        public void SetDisplayOptions(bool showOriginal, bool showSLAM, bool showPath, bool showLidar, bool showRadar)
        {
            showOriginalMap = showOriginal;
            showSLAMMap = showSLAM;
            showExplorationPath = showPath;
            showLidarData = showLidar;
            showRadarRays = showRadar;
            this.Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;
            
            // 应用变换
            g.TranslateTransform(offset.X, offset.Y);
            g.ScaleTransform(scale, scale);
            
            // 绘制背景
            g.Clear(Color.LightGray);
            
            if (slamMap != null)
            {
                // 绘制SLAM地图
                if (showSLAMMap)
                {
                    DrawSLAMMap(g);
                }
                
                // 绘制雷达射线
                if (showRadarRays && radarRays != null)
                {
                    DrawRadarRays(g);
                }
                
                // 绘制激光扫描点
                if (showLidarData && scanPoints != null)
                {
                    DrawScanPoints(g);
                }
                
                // 绘制障碍物点
                if (obstaclePoints != null)
                {
                    DrawObstaclePoints(g);
                }
                
                // 绘制机器人路径
                if (showExplorationPath && robotPath != null)
                {
                    DrawRobotPath(g);
                }
            }
            
            // 绘制机器人
            DrawRobot(g);
        }

        /// <summary>
        /// 绘制SLAM地图
        /// </summary>
        private void DrawSLAMMap(Graphics g)
        {
            if (slamMap == null) return;
            
            var map = slamMap.GetSLAMMap();
            int gridSize = slamMap.GetGridSize();
            float resolution = slamMap.GetResolution();
            
            for (int x = 0; x < gridSize; x++)
            {
                for (int y = 0; y < gridSize; y++)
                {
                    Color cellColor;
                    switch (map[x, y])
                    {
                        case SLAMMap.MapCellState.Unknown:
                            cellColor = unknownColor;
                            break;
                        case SLAMMap.MapCellState.Free:
                            cellColor = freeColor;
                            break;
                        case SLAMMap.MapCellState.Obstacle:
                            cellColor = obstacleColor;
                            break;
                        default:
                            cellColor = unknownColor;
                            break;
                    }
                    
                    // 绘制网格单元
                    float worldX = x * resolution - 2;
                    float worldY = y * resolution - 2;
                    RectangleF cellRect = new RectangleF(worldX, worldY, resolution, resolution);
                    
                    using (Brush brush = new SolidBrush(cellColor))
                    {
                        g.FillRectangle(brush, cellRect);
                    }
                }
            }
        }

        /// <summary>
        /// 绘制雷达射线
        /// </summary>
        private void DrawRadarRays(Graphics g)
        {
            if (radarRays == null) return;
            
            using (Pen pen = new Pen(radarRayColor, 1.0f / scale))
            {
                foreach (var ray in radarRays)
                {
                    g.DrawLine(pen, ray.Item1, ray.Item2);
                }
            }
        }

        /// <summary>
        /// 绘制扫描点
        /// </summary>
        private void DrawScanPoints(Graphics g)
        {
            if (scanPoints == null) return;
            
            float pointSize = 2.0f / scale;
            using (Brush brush = new SolidBrush(scanPointColor))
            {
                foreach (var point in scanPoints)
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
            if (obstaclePoints == null) return;
            
            float pointSize = 3.0f / scale;
            using (Brush brush = new SolidBrush(obstacleColor))
            {
                foreach (var point in obstaclePoints)
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
        /// 绘制机器人路径
        /// </summary>
        private void DrawRobotPath(Graphics g)
        {
            if (robotPath == null || robotPath.Count < 2) return;
            
            using (Pen pen = new Pen(robotPathColor, 2.0f / scale))
            {
                for (int i = 0; i < robotPath.Count - 1; i++)
                {
                    g.DrawLine(pen, robotPath[i], robotPath[i + 1]);
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
            
            using (Pen pen = new Pen(Color.DarkGreen, 3.0f / scale))
            {
                g.DrawLine(pen, robotPosition, new PointF(arrowX, arrowY));
            }
        }

        /// <summary>
        /// 鼠标按下事件
        /// </summary>
        private void MapVisualizationControl_MouseDown(object sender, MouseEventArgs e)
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
        private void MapVisualizationControl_MouseMove(object sender, MouseEventArgs e)
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
        private void MapVisualizationControl_MouseUp(object sender, MouseEventArgs e)
        {
            isDragging = false;
        }

        /// <summary>
        /// 鼠标滚轮事件
        /// </summary>
        private void MapVisualizationControl_MouseWheel(object sender, MouseEventArgs e)
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
            
            int gridSize = slamMap.GetGridSize();
            float resolution = slamMap.GetResolution();
            float mapWidth = gridSize * resolution;
            float mapHeight = gridSize * resolution;
            
            float scaleX = (this.Width - 20) / mapWidth;
            float scaleY = (this.Height - 20) / mapHeight;
            scale = Math.Min(scaleX, scaleY);
            
            offset.X = (this.Width - mapWidth * scale) / 2;
            offset.Y = (this.Height - mapHeight * scale) / 2;
            
            this.Invalidate();
        }
    }
}
