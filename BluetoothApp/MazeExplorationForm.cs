using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace BluetoothApp
{
    /// <summary>
    /// 迷宫探索控制面板
    /// </summary>
    public partial class MazeExplorationForm : Form
    {
        // 主窗体引用
        private BluetoothWindow parentForm;
        
        // 探索控制器
        private MazeExplorationController explorationController;
        
        // 地图可视化控件
        private MapVisualizationControl mapControl;
        
        // 控制按钮
        private Button btnStartExploration;
        private Button btnStopExploration;
        private Button btnReset;
        private Button btnPause;
        
        // 显示选项
        private CheckBox chkShowOriginalMap;
        private CheckBox chkShowSLAMMap;
        private CheckBox chkShowExplorationPath;
        private CheckBox chkShowLidarData;
        private CheckBox chkShowRadarRays;
        
        // 状态显示
        private Label lblStatus;
        private Label lblProgress;
        private Label lblRobotPosition;
        private ProgressBar progressBar;
        
        // 地图选择
        private ComboBox cmbMapSelection;
        private Button btnLoadMap;
        
        // 探索状态
        private bool isExplorationRunning = false;
        private bool isPaused = false;
        private Timer explorationTimer;

        public MazeExplorationForm(BluetoothWindow parent)
        {
            parentForm = parent;
            InitializeComponent();
            InitializeExploration();
        }

        private void InitializeComponent()
        {
            this.Text = "迷宫探索与SLAM建图";
            this.Size = new Size(1200, 800);
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
            
            // 创建地图可视化控件
            mapControl = new MapVisualizationControl();
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
            y += 50;
            
            // 显示选项
            Label lblDisplay = new Label();
            lblDisplay.Text = "显示选项:";
            lblDisplay.Location = new Point(10, y);
            lblDisplay.Size = new Size(100, 20);
            parent.Controls.Add(lblDisplay);
            y += 25;
            
            chkShowOriginalMap = new CheckBox();
            chkShowOriginalMap.Text = "原始地图";
            chkShowOriginalMap.Location = new Point(10, y);
            chkShowOriginalMap.Size = new Size(100, 20);
            chkShowOriginalMap.Checked = true;
            chkShowOriginalMap.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowOriginalMap);
            y += 25;
            
            chkShowSLAMMap = new CheckBox();
            chkShowSLAMMap.Text = "SLAM地图";
            chkShowSLAMMap.Location = new Point(10, y);
            chkShowSLAMMap.Size = new Size(100, 20);
            chkShowSLAMMap.Checked = true;
            chkShowSLAMMap.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowSLAMMap);
            y += 25;
            
            chkShowExplorationPath = new CheckBox();
            chkShowExplorationPath.Text = "探索路径";
            chkShowExplorationPath.Location = new Point(10, y);
            chkShowExplorationPath.Size = new Size(100, 20);
            chkShowExplorationPath.Checked = true;
            chkShowExplorationPath.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowExplorationPath);
            y += 25;
            
            chkShowLidarData = new CheckBox();
            chkShowLidarData.Text = "激光数据";
            chkShowLidarData.Location = new Point(10, y);
            chkShowLidarData.Size = new Size(100, 20);
            chkShowLidarData.Checked = true;
            chkShowLidarData.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowLidarData);
            y += 25;
            
            chkShowRadarRays = new CheckBox();
            chkShowRadarRays.Text = "雷达射线";
            chkShowRadarRays.Location = new Point(10, y);
            chkShowRadarRays.Size = new Size(100, 20);
            chkShowRadarRays.Checked = false;
            chkShowRadarRays.CheckedChanged += DisplayOption_Changed;
            parent.Controls.Add(chkShowRadarRays);
            y += 40;
            
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
            // 初始化探索控制器
            PointF startPos = new PointF(1, 1);  // 起点
            PointF goalPos = new PointF(13, 13); // 目标点
            explorationController = new MazeExplorationController(startPos, goalPos);
            
            // 订阅事件
            explorationController.OnStateChanged += OnExplorationStateChanged;
            explorationController.OnStatusChanged += OnStatusChanged;
            explorationController.OnExplorationProgressChanged += OnProgressChanged;
            explorationController.OnMapUpdated += OnMapUpdated;
            
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
            explorationController = new MazeExplorationController(startPos, goalPos);
            explorationController.OnStateChanged += OnExplorationStateChanged;
            explorationController.OnStatusChanged += OnStatusChanged;
            explorationController.OnExplorationProgressChanged += OnProgressChanged;
            explorationController.OnMapUpdated += OnMapUpdated;
            
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
                
                explorationTimer.Start();
                lblStatus.Text = "探索已开始";
            }
        }

        private void BtnStopExploration_Click(object sender, EventArgs e)
        {
            if (isExplorationRunning)
            {
                isExplorationRunning = false;
                isPaused = false;
                
                explorationTimer.Stop();
                
                btnStartExploration.Enabled = true;
                btnStopExploration.Enabled = false;
                btnPause.Enabled = false;
                
                lblStatus.Text = "探索已停止";
            }
        }

        private void BtnPause_Click(object sender, EventArgs e)
        {
            if (isExplorationRunning)
            {
                isPaused = !isPaused;
                btnPause.Text = isPaused ? "继续" : "暂停";
                lblStatus.Text = isPaused ? "探索已暂停" : "探索进行中";
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
            lblStatus.Text = "已重置";
            lblProgress.Text = "探索进度: 0%";
            lblRobotPosition.Text = "机器人位置: (0, 0)";
            
            mapControl.UpdateMap(null, new PointF(0, 0), 0);
        }

        private void DisplayOption_Changed(object sender, EventArgs e)
        {
            if (mapControl != null)
            {
                mapControl.SetDisplayOptions(
                    chkShowOriginalMap.Checked,
                    chkShowSLAMMap.Checked,
                    chkShowExplorationPath.Checked,
                    chkShowLidarData.Checked,
                    chkShowRadarRays.Checked
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
            PointF robotPos = explorationController.GetRobotPosition();
            float robotOrient = explorationController.GetRobotOrientation();
            
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

        private void OnExplorationStateChanged(MazeExplorationController.ExplorationState state)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<MazeExplorationController.ExplorationState>(OnExplorationStateChanged), state);
                return;
            }
            
            string stateText = "";
            switch (state)
            {
                case MazeExplorationController.ExplorationState.Exploration:
                    stateText = "探索阶段";
                    break;
                case MazeExplorationController.ExplorationState.SearchGoal:
                    stateText = "搜索目标";
                    break;
                case MazeExplorationController.ExplorationState.NavigateToGoal:
                    stateText = "导航到目标";
                    break;
                case MazeExplorationController.ExplorationState.PathPlanning:
                    stateText = "路径规划";
                    break;
                case MazeExplorationController.ExplorationState.Navigation:
                    stateText = "导航阶段";
                    break;
                case MazeExplorationController.ExplorationState.Completed:
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

        private void OnMapUpdated(SLAMMap map)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<SLAMMap>(OnMapUpdated), map);
                return;
            }
            
            PointF robotPos = explorationController.GetRobotPosition();
            float robotOrient = explorationController.GetRobotOrientation();
            
            mapControl.UpdateMap(map, robotPos, robotOrient);
            
            lblRobotPosition.Text = $"机器人位置: ({robotPos.X:F1}, {robotPos.Y:F1})";
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
