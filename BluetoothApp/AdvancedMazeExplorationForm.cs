using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace BluetoothApp
{
    /// <summary>
    /// 高级迷宫探索控制面板
    /// 集成位姿图SLAM和高级路径规划
    /// </summary>
    public partial class AdvancedMazeExplorationForm : Form
    {
        // 主窗体引用
        private BluetoothWindow parentForm;
        
        // 高级探索控制器
        private AdvancedMazeExplorationController explorationController;
        
        // 高级地图可视化控件
        private AdvancedMapVisualizationControl mapControl;
        
        // 控制按钮
        private Button btnStartExploration;
        private Button btnStopExploration;
        private Button btnReset;
        private Button btnPause;
        private Button btnOptimizeGraph;
        
        // 显示选项
        private CheckBox chkShowOccupancyGrid;
        private CheckBox chkShowProbabilityGrid;
        private CheckBox chkShowPointCloud;
        private CheckBox chkShowTrajectory;
        private CheckBox chkShowOptimizedTrajectory;
        private CheckBox chkShowObstacles;
        private CheckBox chkShowFreeSpace;
        private CheckBox chkShowLoopClosures;
        private CheckBox chkShowPoseGraph;
        
        // 状态显示
        private Label lblStatus;
        private Label lblProgress;
        private Label lblRobotPosition;
        private Label lblMapStatistics;
        private ProgressBar progressBar;
        
        // 地图选择
        private ComboBox cmbMapSelection;
        private Button btnLoadMap;
        
        // 探索策略选择
        private ComboBox cmbExplorationStrategy;
        
        // 参数设置
        private NumericUpDown numResolution;
        private NumericUpDown numMapWidth;
        private NumericUpDown numMapHeight;
        private NumericUpDown numMaxRange;
        
        // 探索状态
        private bool isExplorationRunning = false;
        private bool isPaused = false;
        private Timer explorationTimer;
        
        // 统计信息
        private Label lblStepCount;
        private Label lblPathLength;
        private Label lblLoopClosures;

        public AdvancedMazeExplorationForm(BluetoothWindow parent)
        {
            parentForm = parent;
            InitializeComponent();
            InitializeExploration();
        }

        private void InitializeComponent()
        {
            this.Text = "高级迷宫探索与SLAM建图";
            this.Size = new Size(1400, 900);
            this.StartPosition = FormStartPosition.CenterParent;
            this.FormBorderStyle = FormBorderStyle.Sizable;
            
            // 创建主面板
            TableLayoutPanel mainPanel = new TableLayoutPanel();
            mainPanel.Dock = DockStyle.Fill;
            mainPanel.ColumnCount = 2;
            mainPanel.RowCount = 1;
            mainPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70));
            mainPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
            this.Controls.Add(mainPanel);
            
            // 创建地图显示区域
            Panel mapPanel = new Panel();
            mapPanel.Dock = DockStyle.Fill;
            mapPanel.BorderStyle = BorderStyle.FixedSingle;
            mainPanel.Controls.Add(mapPanel, 0, 0);
            
            // 创建高级地图可视化控件
            mapControl = new AdvancedMapVisualizationControl();
            mapControl.Dock = DockStyle.Fill;
            mapPanel.Controls.Add(mapControl);
            
            // 创建控制面板
            Panel controlPanel = new Panel();
            controlPanel.Dock = DockStyle.Fill;
            controlPanel.AutoScroll = true;
            mainPanel.Controls.Add(controlPanel, 1, 0);
            
            // 创建控制面板内容
            CreateControlPanel(controlPanel);
        }

        private void CreateControlPanel(Panel parent)
        {
            int y = 10;
            int spacing = 35;
            
            // 地图选择
            Label lblMap = new Label();
            lblMap.Text = "地图选择:";
            lblMap.Location = new Point(10, y);
            lblMap.Size = new Size(100, 20);
            parent.Controls.Add(lblMap);
            y += 25;
            
            cmbMapSelection = new ComboBox();
            cmbMapSelection.Location = new Point(10, y);
            cmbMapSelection.Size = new Size(200, 25);
            cmbMapSelection.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbMapSelection.Items.AddRange(new string[] { "迷宫1 (15x15)", "迷宫2 (16x16)", "迷宫3 (21x21)", "自定义地图" });
            cmbMapSelection.SelectedIndex = 0;
            parent.Controls.Add(cmbMapSelection);
            y += 30;
            
            btnLoadMap = new Button();
            btnLoadMap.Text = "加载地图";
            btnLoadMap.Location = new Point(10, y);
            btnLoadMap.Size = new Size(100, 30);
            btnLoadMap.Click += BtnLoadMap_Click;
            parent.Controls.Add(btnLoadMap);
            y += 40;
            
            // 探索策略
            Label lblStrategy = new Label();
            lblStrategy.Text = "探索策略:";
            lblStrategy.Location = new Point(10, y);
            lblStrategy.Size = new Size(100, 20);
            parent.Controls.Add(lblStrategy);
            y += 25;
            
            cmbExplorationStrategy = new ComboBox();
            cmbExplorationStrategy.Location = new Point(10, y);
            cmbExplorationStrategy.Size = new Size(200, 25);
            cmbExplorationStrategy.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbExplorationStrategy.Items.AddRange(new string[] { "基于边界", "随机游走", "沿墙行走", "混合策略" });
            cmbExplorationStrategy.SelectedIndex = 3;
            parent.Controls.Add(cmbExplorationStrategy);
            y += 40;
            
            // 探索控制
            Label lblControl = new Label();
            lblControl.Text = "探索控制:";
            lblControl.Location = new Point(10, y);
            lblControl.Size = new Size(100, 20);
            parent.Controls.Add(lblControl);
            y += 25;
            
            btnStartExploration = new Button();
            btnStartExploration.Text = "开始探索";
            btnStartExploration.Location = new Point(10, y);
            btnStartExploration.Size = new Size(90, 30);
            btnStartExploration.Click += BtnStartExploration_Click;
            parent.Controls.Add(btnStartExploration);
            
            btnStopExploration = new Button();
            btnStopExploration.Text = "停止探索";
            btnStopExploration.Location = new Point(110, y);
            btnStopExploration.Size = new Size(90, 30);
            btnStopExploration.Enabled = false;
            btnStopExploration.Click += BtnStopExploration_Click;
            parent.Controls.Add(btnStopExploration);
            y += 40;
            
            btnPause = new Button();
            btnPause.Text = "暂停";
            btnPause.Location = new Point(10, y);
            btnPause.Size = new Size(90, 30);
            btnPause.Enabled = false;
            btnPause.Click += BtnPause_Click;
            parent.Controls.Add(btnPause);
            
            btnReset = new Button();
            btnReset.Text = "重置";
            btnReset.Location = new Point(110, y);
            btnReset.Size = new Size(90, 30);
            btnReset.Click += BtnReset_Click;
            parent.Controls.Add(btnReset);
            y += 40;
            
            btnOptimizeGraph = new Button();
            btnOptimizeGraph.Text = "优化图";
            btnOptimizeGraph.Location = new Point(10, y);
            btnOptimizeGraph.Size = new Size(90, 30);
            btnOptimizeGraph.Click += BtnOptimizeGraph_Click;
            parent.Controls.Add(btnOptimizeGraph);
            y += 50;
            
            // 显示选项
            Label lblDisplay = new Label();
            lblDisplay.Text = "显示选项:";
            lblDisplay.Location = new Point(10, y);
            lblDisplay.Size = new Size(100, 20);
            parent.Controls.Add(lblDisplay);
            y += 25;
            
            chkShowOccupancyGrid = new CheckBox();
            chkShowOccupancyGrid.Text = "占用栅格";
            chkShowOccupancyGrid.Location = new Point(10, y);
            chkShowOccupancyGrid.Size = new Size(100, 20);
            chkShowOccupancyGrid.Checked = true;
            chkShowOccupancyGrid.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowOccupancyGrid);
            y += 25;
            
            chkShowProbabilityGrid = new CheckBox();
            chkShowProbabilityGrid.Text = "概率栅格";
            chkShowProbabilityGrid.Location = new Point(10, y);
            chkShowProbabilityGrid.Size = new Size(100, 20);
            chkShowProbabilityGrid.Checked = false;
            chkShowProbabilityGrid.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowProbabilityGrid);
            y += 25;
            
            chkShowPointCloud = new CheckBox();
            chkShowPointCloud.Text = "点云地图";
            chkShowPointCloud.Location = new Point(10, y);
            chkShowPointCloud.Size = new Size(100, 20);
            chkShowPointCloud.Checked = true;
            chkShowPointCloud.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowPointCloud);
            y += 25;
            
            chkShowTrajectory = new CheckBox();
            chkShowTrajectory.Text = "原始轨迹";
            chkShowTrajectory.Location = new Point(10, y);
            chkShowTrajectory.Size = new Size(100, 20);
            chkShowTrajectory.Checked = true;
            chkShowTrajectory.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowTrajectory);
            y += 25;
            
            chkShowOptimizedTrajectory = new CheckBox();
            chkShowOptimizedTrajectory.Text = "优化轨迹";
            chkShowOptimizedTrajectory.Location = new Point(10, y);
            chkShowOptimizedTrajectory.Size = new Size(100, 20);
            chkShowOptimizedTrajectory.Checked = true;
            chkShowOptimizedTrajectory.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowOptimizedTrajectory);
            y += 25;
            
            chkShowObstacles = new CheckBox();
            chkShowObstacles.Text = "障碍物点";
            chkShowObstacles.Location = new Point(10, y);
            chkShowObstacles.Size = new Size(100, 20);
            chkShowObstacles.Checked = true;
            chkShowObstacles.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowObstacles);
            y += 25;
            
            chkShowFreeSpace = new CheckBox();
            chkShowFreeSpace.Text = "自由空间";
            chkShowFreeSpace.Location = new Point(10, y);
            chkShowFreeSpace.Size = new Size(100, 20);
            chkShowFreeSpace.Checked = false;
            chkShowFreeSpace.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowFreeSpace);
            y += 25;
            
            chkShowLoopClosures = new CheckBox();
            chkShowLoopClosures.Text = "回环检测";
            chkShowLoopClosures.Location = new Point(10, y);
            chkShowLoopClosures.Size = new Size(100, 20);
            chkShowLoopClosures.Checked = true;
            chkShowLoopClosures.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowLoopClosures);
            y += 25;
            
            chkShowPoseGraph = new CheckBox();
            chkShowPoseGraph.Text = "位姿图";
            chkShowPoseGraph.Location = new Point(10, y);
            chkShowPoseGraph.Size = new Size(100, 20);
            chkShowPoseGraph.Checked = false;
            chkShowPoseGraph.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowPoseGraph);
            y += 40;
            
            // 参数设置
            Label lblParams = new Label();
            lblParams.Text = "参数设置:";
            lblParams.Location = new Point(10, y);
            lblParams.Size = new Size(100, 20);
            parent.Controls.Add(lblParams);
            y += 25;
            
            // 分辨率
            Label lblResolution = new Label();
            lblResolution.Text = "分辨率(m):";
            lblResolution.Location = new Point(10, y);
            lblResolution.Size = new Size(80, 20);
            parent.Controls.Add(lblResolution);
            
            numResolution = new NumericUpDown();
            numResolution.Location = new Point(100, y);
            numResolution.Size = new Size(60, 20);
            numResolution.DecimalPlaces = 2;
            numResolution.Increment = 0.01m;
            numResolution.Minimum = 0.01m;
            numResolution.Maximum = 1.0m;
            numResolution.Value = 0.05m;
            parent.Controls.Add(numResolution);
            y += 30;
            
            // 地图宽度
            Label lblWidth = new Label();
            lblWidth.Text = "地图宽度(m):";
            lblWidth.Location = new Point(10, y);
            lblWidth.Size = new Size(80, 20);
            parent.Controls.Add(lblWidth);
            
            numMapWidth = new NumericUpDown();
            numMapWidth.Location = new Point(100, y);
            numMapWidth.Size = new Size(60, 20);
            numMapWidth.DecimalPlaces = 1;
            numMapWidth.Increment = 1.0m;
            numMapWidth.Minimum = 5.0m;
            numMapWidth.Maximum = 50.0m;
            numMapWidth.Value = 20.0m;
            parent.Controls.Add(numMapWidth);
            y += 30;
            
            // 地图高度
            Label lblHeight = new Label();
            lblHeight.Text = "地图高度(m):";
            lblHeight.Location = new Point(10, y);
            lblHeight.Size = new Size(80, 20);
            parent.Controls.Add(lblHeight);
            
            numMapHeight = new NumericUpDown();
            numMapHeight.Location = new Point(100, y);
            numMapHeight.Size = new Size(60, 20);
            numMapHeight.DecimalPlaces = 1;
            numMapHeight.Increment = 1.0m;
            numMapHeight.Minimum = 5.0m;
            numMapHeight.Maximum = 50.0m;
            numMapHeight.Value = 20.0m;
            parent.Controls.Add(numMapHeight);
            y += 30;
            
            // 最大探测距离
            Label lblMaxRange = new Label();
            lblMaxRange.Text = "探测距离(m):";
            lblMaxRange.Location = new Point(10, y);
            lblMaxRange.Size = new Size(80, 20);
            parent.Controls.Add(lblMaxRange);
            
            numMaxRange = new NumericUpDown();
            numMaxRange.Location = new Point(100, y);
            numMaxRange.Size = new Size(60, 20);
            numMaxRange.DecimalPlaces = 1;
            numMaxRange.Increment = 0.5m;
            numMaxRange.Minimum = 1.0m;
            numMaxRange.Maximum = 20.0m;
            numMaxRange.Value = 5.0m;
            parent.Controls.Add(numMaxRange);
            y += 50;
            
            // 状态显示
            Label lblStatusTitle = new Label();
            lblStatusTitle.Text = "状态信息:";
            lblStatusTitle.Location = new Point(10, y);
            lblStatusTitle.Size = new Size(100, 20);
            parent.Controls.Add(lblStatusTitle);
            y += 25;
            
            lblStatus = new Label();
            lblStatus.Text = "就绪";
            lblStatus.Location = new Point(10, y);
            lblStatus.Size = new Size(200, 20);
            lblStatus.ForeColor = Color.Blue;
            parent.Controls.Add(lblStatus);
            y += 25;
            
            lblProgress = new Label();
            lblProgress.Text = "探索进度: 0%";
            lblProgress.Location = new Point(10, y);
            lblProgress.Size = new Size(150, 20);
            parent.Controls.Add(lblProgress);
            y += 25;
            
            progressBar = new ProgressBar();
            progressBar.Location = new Point(10, y);
            progressBar.Size = new Size(200, 20);
            progressBar.Minimum = 0;
            progressBar.Maximum = 100;
            parent.Controls.Add(progressBar);
            y += 30;
            
            lblRobotPosition = new Label();
            lblRobotPosition.Text = "机器人位置: (0, 0)";
            lblRobotPosition.Location = new Point(10, y);
            lblRobotPosition.Size = new Size(200, 20);
            parent.Controls.Add(lblRobotPosition);
            y += 30;
            
            // 统计信息
            Label lblStats = new Label();
            lblStats.Text = "统计信息:";
            lblStats.Location = new Point(10, y);
            lblStats.Size = new Size(100, 20);
            parent.Controls.Add(lblStats);
            y += 25;
            
            lblStepCount = new Label();
            lblStepCount.Text = "步数: 0";
            lblStepCount.Location = new Point(10, y);
            lblStepCount.Size = new Size(150, 20);
            parent.Controls.Add(lblStepCount);
            y += 25;
            
            lblPathLength = new Label();
            lblPathLength.Text = "路径长度: 0m";
            lblPathLength.Location = new Point(10, y);
            lblPathLength.Size = new Size(150, 20);
            parent.Controls.Add(lblPathLength);
            y += 25;
            
            lblLoopClosures = new Label();
            lblLoopClosures.Text = "回环数: 0";
            lblLoopClosures.Location = new Point(10, y);
            lblLoopClosures.Size = new Size(150, 20);
            parent.Controls.Add(lblLoopClosures);
            y += 30;
            
            lblMapStatistics = new Label();
            lblMapStatistics.Text = "地图统计: 无数据";
            lblMapStatistics.Location = new Point(10, y);
            lblMapStatistics.Size = new Size(200, 60);
            lblMapStatistics.AutoSize = false;
            lblMapStatistics.Font = new Font("Arial", 8);
            parent.Controls.Add(lblMapStatistics);
            y += 70;
            
            // 视图控制
            Label lblView = new Label();
            lblView.Text = "视图控制:";
            lblView.Location = new Point(10, y);
            lblView.Size = new Size(100, 20);
            parent.Controls.Add(lblView);
            y += 25;
            
            Button btnFitToWindow = new Button();
            btnFitToWindow.Text = "适应窗口";
            btnFitToWindow.Location = new Point(10, y);
            btnFitToWindow.Size = new Size(90, 30);
            btnFitToWindow.Click += BtnFitToWindow_Click;
            parent.Controls.Add(btnFitToWindow);
            
            Button btnResetView = new Button();
            btnResetView.Text = "重置视图";
            btnResetView.Location = new Point(110, y);
            btnResetView.Size = new Size(90, 30);
            btnResetView.Click += BtnResetView_Click;
            parent.Controls.Add(btnResetView);
        }

        private void InitializeExploration()
        {
            // 初始化高级探索控制器
            PointF startPos = new PointF(1, 1);  // 起点
            PointF goalPos = new PointF(13, 13); // 目标点
            explorationController = new AdvancedMazeExplorationController(startPos, goalPos);
            
            // 订阅事件
            explorationController.OnStateChanged += OnExplorationStateChanged;
            explorationController.OnStatusChanged += OnStatusChanged;
            explorationController.OnExplorationProgressChanged += OnProgressChanged;
            explorationController.OnMapUpdated += OnMapUpdated;
            explorationController.OnRobotPoseChanged += OnRobotPoseChanged;
            
            // 初始化定时器
            explorationTimer = new Timer();
            explorationTimer.Interval = 100; // 100ms更新一次
            explorationTimer.Tick += ExplorationTimer_Tick;
        }

        private void BtnLoadMap_Click(object sender, EventArgs e)
        {
            // 根据选择的地图类型加载地图
            int mapType = cmbMapSelection.SelectedIndex;
            PointF startPos, goalPos;
            
            switch (mapType)
            {
                case 0: // 迷宫1
                    startPos = new PointF(1, 1);
                    goalPos = new PointF(13, 13);
                    break;
                case 1: // 迷宫2
                    startPos = new PointF(1, 1);
                    goalPos = new PointF(14, 14);
                    break;
                case 2: // 迷宫3
                    startPos = new PointF(1, 1);
                    goalPos = new PointF(19, 19);
                    break;
                default: // 自定义
                    startPos = new PointF(1, 1);
                    goalPos = new PointF(13, 13);
                    break;
            }
            
            // 重新初始化探索控制器
            explorationController = new AdvancedMazeExplorationController(startPos, goalPos);
            explorationController.OnStateChanged += OnExplorationStateChanged;
            explorationController.OnStatusChanged += OnStatusChanged;
            explorationController.OnExplorationProgressChanged += OnProgressChanged;
            explorationController.OnMapUpdated += OnMapUpdated;
            explorationController.OnRobotPoseChanged += OnRobotPoseChanged;
            
            lblStatus.Text = $"已加载地图 {mapType + 1}";
            mapControl.FitToWindow();
        }

        private void BtnStartExploration_Click(object sender, EventArgs e)
        {
            if (!isExplorationRunning)
            {
                isExplorationRunning = true;
                isPaused = false;
                
                btnStartExploration.Enabled = false;
                btnStopExploration.Enabled = true;
                btnPause.Enabled = true;
                
                explorationController.StartExploration();
                explorationTimer.Start();
            }
        }

        private void BtnStopExploration_Click(object sender, EventArgs e)
        {
            if (isExplorationRunning)
            {
                isExplorationRunning = false;
                isPaused = false;
                
                explorationTimer.Stop();
                explorationController.StopExploration();
                
                btnStartExploration.Enabled = true;
                btnStopExploration.Enabled = false;
                btnPause.Enabled = false;
            }
        }

        private void BtnPause_Click(object sender, EventArgs e)
        {
            if (isExplorationRunning)
            {
                isPaused = !isPaused;
                explorationController.TogglePause();
                btnPause.Text = isPaused ? "继续" : "暂停";
            }
        }

        private void BtnReset_Click(object sender, EventArgs e)
        {
            if (isExplorationRunning)
            {
                explorationTimer.Stop();
            }
            
            explorationController.Reset();
            isExplorationRunning = false;
            isPaused = false;
            
            btnStartExploration.Enabled = true;
            btnStopExploration.Enabled = false;
            btnPause.Enabled = false;
            btnPause.Text = "暂停";
            
            progressBar.Value = 0;
            lblStepCount.Text = "步数: 0";
            lblPathLength.Text = "路径长度: 0m";
            lblLoopClosures.Text = "回环数: 0";
            lblMapStatistics.Text = "地图统计: 无数据";
            
            mapControl.UpdateMap(null, new PointF(0, 0), 0);
        }

        private void BtnOptimizeGraph_Click(object sender, EventArgs e)
        {
            if (explorationController != null && explorationController.SLAMMap != null)
            {
                var poseGraphSLAM = explorationController.PoseGraphSLAM;
                if (poseGraphSLAM != null)
                {
                    poseGraphSLAM.OptimizeGraph();
                    lblStatus.Text = "图优化完成";
                    OnMapUpdated(explorationController.SLAMMap);
                }
            }
        }

        private void DisplayOption_Changed(object sender, EventArgs e)
        {
            if (mapControl != null)
            {
                mapControl.SetDisplayOptions(
                    chkShowOccupancyGrid.Checked,
                    chkShowProbabilityGrid.Checked,
                    chkShowPointCloud.Checked,
                    chkShowTrajectory.Checked,
                    chkShowOptimizedTrajectory.Checked,
                    chkShowObstacles.Checked,
                    chkShowFreeSpace.Checked,
                    chkShowLoopClosures.Checked,
                    chkShowPoseGraph.Checked
                );
            }
        }

        private void BtnFitToWindow_Click(object sender, EventArgs e)
        {
            mapControl.FitToWindow();
        }

        private void BtnResetView_Click(object sender, EventArgs e)
        {
            mapControl.ResetView();
        }

        private void ExplorationTimer_Tick(object sender, EventArgs e)
        {
            if (!isPaused && isExplorationRunning)
            {
                // 模拟激光数据（实际应用中应该从硬件获取）
                List<PointF> lidarData = SimulateLidarData();
                
                // 更新探索控制器
                explorationController.Update(lidarData);
            }
        }

        private List<PointF> SimulateLidarData()
        {
            // 这里应该从实际的激光雷达获取数据
            // 暂时返回模拟数据
            List<PointF> lidarData = new List<PointF>();
            
            // 模拟前方障碍物检测
            PointF robotPos = explorationController.CurrentPosition;
            float robotOrient = explorationController.CurrentOrientation;
            
            for (int i = 0; i < 36; i++) // 每10度一个点
            {
                float angle = robotOrient + (i - 18) * 10 * (float)Math.PI / 180;
                float distance = 2.0f + (float)Math.Sin(angle * 3) * 0.5f; // 模拟障碍物
                
                float x = robotPos.X + (float)Math.Cos(angle) * distance;
                float y = robotPos.Y + (float)Math.Sin(angle) * distance;
                
                lidarData.Add(new PointF(x, y));
            }
            
            return lidarData;
        }

        private void OnExplorationStateChanged(AdvancedMazeExplorationController.ExplorationState state)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<AdvancedMazeExplorationController.ExplorationState>(OnExplorationStateChanged), state);
                return;
            }
            
            string stateText = "";
            switch (state)
            {
                case AdvancedMazeExplorationController.ExplorationState.Initialization:
                    stateText = "初始化";
                    break;
                case AdvancedMazeExplorationController.ExplorationState.Exploration:
                    stateText = "探索阶段";
                    break;
                case AdvancedMazeExplorationController.ExplorationState.SearchGoal:
                    stateText = "搜索目标";
                    break;
                case AdvancedMazeExplorationController.ExplorationState.NavigateToGoal:
                    stateText = "导航到目标";
                    break;
                case AdvancedMazeExplorationController.ExplorationState.PathPlanning:
                    stateText = "路径规划";
                    break;
                case AdvancedMazeExplorationController.ExplorationState.Navigation:
                    stateText = "导航阶段";
                    break;
                case AdvancedMazeExplorationController.ExplorationState.Completed:
                    stateText = "探索完成";
                    break;
            }
            
            lblStatus.Text = $"状态: {stateText}";
        }

        private void OnStatusChanged(string status)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<string>(OnStatusChanged), status);
                return;
            }
            
            lblStatus.Text = status;
        }

        private void OnProgressChanged(float progress)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<float>(OnProgressChanged), progress);
                return;
            }
            
            progressBar.Value = Math.Min(100, Math.Max(0, (int)progress));
            lblProgress.Text = $"探索进度: {progress:F1}%";
        }

        private void OnMapUpdated(AdvancedSLAMMap map)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<AdvancedSLAMMap>(OnMapUpdated), map);
                return;
            }
            
            PointF robotPos = explorationController.CurrentPosition;
            float robotOrient = explorationController.CurrentOrientation;
            
            mapControl.UpdateMap(map, robotPos, robotOrient);
            
            // 更新统计信息
            lblStepCount.Text = $"步数: {explorationController.StepCount}";
            lblMapStatistics.Text = map.GetMapStatistics();
            
            // 更新路径长度
            var trajectory = map.GetRobotTrajectory();
            if (trajectory.Count > 1)
            {
                float pathLength = 0;
                for (int i = 0; i < trajectory.Count - 1; i++)
                {
                    float dx = trajectory[i + 1].X - trajectory[i].X;
                    float dy = trajectory[i + 1].Y - trajectory[i].Y;
                    pathLength += (float)Math.Sqrt(dx * dx + dy * dy);
                }
                lblPathLength.Text = $"路径长度: {pathLength:F2}m";
            }
            
            // 更新回环检测数量
            var poseGraphSLAM = map.GetPoseGraphSLAM();
            if (poseGraphSLAM != null)
            {
                var edges = poseGraphSLAM.GetAllEdges();
                int loopClosureCount = edges.Count(e => e.Type == PoseGraphSLAM.EdgeType.LoopClosure);
                lblLoopClosures.Text = $"回环数: {loopClosureCount}";
            }
        }

        private void OnRobotPoseChanged(PointF position, float orientation)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<PointF, float>(OnRobotPoseChanged), position, orientation);
                return;
            }
            
            lblRobotPosition.Text = $"机器人位置: ({position.X:F1}, {position.Y:F1})";
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            if (isExplorationRunning)
            {
                explorationTimer.Stop();
            }
            
            base.OnFormClosing(e);
        }
    }
}
