using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace BluetoothApp
{
    /// <summary>
    /// SLAM自动控制界面
    /// </summary>
    public partial class SLAMAutoControlForm : Form
    {
        #region 私有字段
        private BluetoothWindow mainForm;
        private SLAMAutoController slamAutoController;
        private AutoRobotController autoController;
        private AdvancedMazeExplorationController slamController;
        private System.Windows.Forms.Timer statusUpdateTimer;
        private bool isInitialized = false;
        #endregion

        #region 控件声明
        private GroupBox gbSLAMControl;
        private Button btnStartSLAM;
        private Button btnStopSLAM;
        private Button btnPauseResume;
        private Label lblSLAMStatus;
        private Label lblPoseInfo;
        private Label lblMapInfo;
        
        private GroupBox gbExplorationModes;
        private Button btnSLAMExploration;
        private Button btnMazeExploration;
        private Button btnCustomExploration;
        
        private GroupBox gbMapDisplay;
        private Panel pnlMapDisplay;
        private Label lblMapTitle;
        private CheckBox chkShowTrajectory;
        private CheckBox chkShowScanPoints;
        private CheckBox chkShowOccupancyGrid;
        
        private GroupBox gbDataLog;
        private ListBox lstDataLog;
        private Button btnClearLog;
        private Button btnSaveLog;
        #endregion

        #region 构造函数
        public SLAMAutoControlForm(BluetoothWindow parentForm)
        {
            mainForm = parentForm;
            autoController = new AutoRobotController(mainForm, mainForm.robotController);
            slamController = new AdvancedMazeExplorationController(new System.Drawing.PointF(0, 0), new System.Drawing.PointF(10, 10));
            slamAutoController = new SLAMAutoController(mainForm, autoController, slamController);
            
            InitializeComponent();
            SetupEventHandlers();
            InitializeStatusUpdateTimer();
            
            isInitialized = true;
        }
        #endregion

        #region 初始化方法
        private void InitializeComponent()
        {
            this.SuspendLayout();
            
            // 窗体设置
            this.Text = "SLAM自动控制";
            this.Size = new Size(800, 600);
            this.StartPosition = FormStartPosition.CenterParent;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;

            // SLAM控制组
            gbSLAMControl = new GroupBox
            {
                Text = "SLAM自动控制",
                Location = new Point(10, 10),
                Size = new Size(380, 150),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 开始/停止按钮
            btnStartSLAM = new Button
            {
                Text = "开始SLAM控制",
                Location = new Point(20, 30),
                Size = new Size(120, 35),
                BackColor = Color.LightGreen,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            btnStopSLAM = new Button
            {
                Text = "停止SLAM控制",
                Location = new Point(150, 30),
                Size = new Size(120, 35),
                BackColor = Color.LightCoral,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold),
                Enabled = false
            };

            // 暂停/恢复按钮
            btnPauseResume = new Button
            {
                Text = "暂停",
                Location = new Point(280, 30),
                Size = new Size(80, 35),
                BackColor = Color.Orange,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold),
                Enabled = false
            };

            // 状态标签
            lblSLAMStatus = new Label
            {
                Text = "状态: 未启动",
                Location = new Point(20, 75),
                Size = new Size(200, 20),
                Font = new Font("微软雅黑", 9F)
            };

            lblPoseInfo = new Label
            {
                Text = "位姿: (0.00, 0.00, 0.00°)",
                Location = new Point(20, 95),
                Size = new Size(200, 20),
                Font = new Font("微软雅黑", 9F)
            };

            lblMapInfo = new Label
            {
                Text = "地图: 未初始化",
                Location = new Point(20, 115),
                Size = new Size(200, 20),
                Font = new Font("微软雅黑", 9F)
            };

            // 探索模式组
            gbExplorationModes = new GroupBox
            {
                Text = "探索模式",
                Location = new Point(400, 10),
                Size = new Size(380, 150),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 探索模式按钮
            btnSLAMExploration = new Button
            {
                Text = "SLAM探索",
                Location = new Point(20, 30),
                Size = new Size(100, 35),
                BackColor = Color.LightBlue,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            btnMazeExploration = new Button
            {
                Text = "迷宫探索",
                Location = new Point(130, 30),
                Size = new Size(100, 35),
                BackColor = Color.LightGreen,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            btnCustomExploration = new Button
            {
                Text = "自定义探索",
                Location = new Point(240, 30),
                Size = new Size(100, 35),
                BackColor = Color.LightPink,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            // 地图显示组
            gbMapDisplay = new GroupBox
            {
                Text = "地图显示",
                Location = new Point(10, 170),
                Size = new Size(380, 200),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 地图显示面板
            pnlMapDisplay = new Panel
            {
                Location = new Point(20, 30),
                Size = new Size(340, 120),
                BackColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle
            };

            lblMapTitle = new Label
            {
                Text = "SLAM地图",
                Location = new Point(5, 5),
                Size = new Size(100, 20),
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            // 显示选项
            chkShowTrajectory = new CheckBox
            {
                Text = "显示轨迹",
                Location = new Point(20, 160),
                Size = new Size(80, 20),
                Checked = true,
                Font = new Font("微软雅黑", 8F)
            };

            chkShowScanPoints = new CheckBox
            {
                Text = "显示扫描点",
                Location = new Point(110, 160),
                Size = new Size(80, 20),
                Checked = true,
                Font = new Font("微软雅黑", 8F)
            };

            chkShowOccupancyGrid = new CheckBox
            {
                Text = "显示占用网格",
                Location = new Point(200, 160),
                Size = new Size(100, 20),
                Checked = true,
                Font = new Font("微软雅黑", 8F)
            };

            // 数据日志组
            gbDataLog = new GroupBox
            {
                Text = "数据日志",
                Location = new Point(400, 170),
                Size = new Size(380, 200),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 日志列表
            lstDataLog = new ListBox
            {
                Location = new Point(20, 30),
                Size = new Size(340, 120),
                Font = new Font("微软雅黑", 8F)
            };

            // 日志控制按钮
            btnClearLog = new Button
            {
                Text = "清空日志",
                Location = new Point(20, 160),
                Size = new Size(80, 25),
                BackColor = Color.LightGray,
                Font = new Font("微软雅黑", 8F)
            };

            btnSaveLog = new Button
            {
                Text = "保存日志",
                Location = new Point(110, 160),
                Size = new Size(80, 25),
                BackColor = Color.LightBlue,
                Font = new Font("微软雅黑", 8F)
            };

            // 添加控件到窗体
            gbSLAMControl.Controls.AddRange(new Control[] {
                btnStartSLAM, btnStopSLAM, btnPauseResume,
                lblSLAMStatus, lblPoseInfo, lblMapInfo
            });

            gbExplorationModes.Controls.AddRange(new Control[] {
                btnSLAMExploration, btnMazeExploration, btnCustomExploration
            });

            gbMapDisplay.Controls.AddRange(new Control[] {
                pnlMapDisplay, lblMapTitle, chkShowTrajectory,
                chkShowScanPoints, chkShowOccupancyGrid
            });

            gbDataLog.Controls.AddRange(new Control[] {
                lstDataLog, btnClearLog, btnSaveLog
            });

            this.Controls.AddRange(new Control[] {
                gbSLAMControl, gbExplorationModes, gbMapDisplay, gbDataLog
            });

            this.ResumeLayout(false);
        }

        private void SetupEventHandlers()
        {
            // SLAM控制事件
            btnStartSLAM.Click += BtnStartSLAM_Click;
            btnStopSLAM.Click += BtnStopSLAM_Click;
            btnPauseResume.Click += BtnPauseResume_Click;

            // 探索模式事件
            btnSLAMExploration.Click += BtnSLAMExploration_Click;
            btnMazeExploration.Click += BtnMazeExploration_Click;
            btnCustomExploration.Click += BtnCustomExploration_Click;

            // 日志控制事件
            btnClearLog.Click += BtnClearLog_Click;
            btnSaveLog.Click += BtnSaveLog_Click;

            // SLAM自动控制器事件
            slamAutoController.OnStatusChanged += SLAMAutoController_OnStatusChanged;
            slamAutoController.OnMapUpdated += SLAMAutoController_OnMapUpdated;
            slamAutoController.OnPoseUpdated += SLAMAutoController_OnPoseUpdated;
            slamAutoController.OnError += SLAMAutoController_OnError;
        }

        private void InitializeStatusUpdateTimer()
        {
            statusUpdateTimer = new System.Windows.Forms.Timer
            {
                Interval = 500 // 500ms更新一次
            };
            statusUpdateTimer.Tick += StatusUpdateTimer_Tick;
            statusUpdateTimer.Start();
        }
        #endregion

        #region 事件处理方法
        private void BtnStartSLAM_Click(object sender, EventArgs e)
        {
            try
            {
                slamAutoController.StartSLAMAutoControl();
                UpdateButtonStates();
                AddLog("SLAM自动控制已启动");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"启动SLAM自动控制失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnStopSLAM_Click(object sender, EventArgs e)
        {
            try
            {
                slamAutoController.StopSLAMAutoControl();
                UpdateButtonStates();
                AddLog("SLAM自动控制已停止");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"停止SLAM自动控制失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnPauseResume_Click(object sender, EventArgs e)
        {
            try
            {
                slamAutoController.TogglePause();
                UpdateButtonStates();
                string status = slamAutoController.IsPaused ? "已暂停" : "已恢复";
                AddLog($"SLAM自动控制{status}");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"暂停/恢复操作失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnSLAMExploration_Click(object sender, EventArgs e)
        {
            try
            {
                slamAutoController.ExecuteSLAMExplorationSequence();
                AddLog("开始执行SLAM探索序列");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"执行SLAM探索序列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnMazeExploration_Click(object sender, EventArgs e)
        {
            try
            {
                slamAutoController.ExecuteMazeExplorationSequence();
                AddLog("开始执行迷宫探索序列");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"执行迷宫探索序列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnCustomExploration_Click(object sender, EventArgs e)
        {
            try
            {
                // 显示自定义探索对话框
                var customForm = new CustomExplorationForm();
                if (customForm.ShowDialog() == DialogResult.OK)
                {
                    var commands = customForm.GetCommands();
                    autoController.AddCommandSequence(commands);
                    AddLog("添加自定义探索序列");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"自定义探索失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnClearLog_Click(object sender, EventArgs e)
        {
            lstDataLog.Items.Clear();
        }

        private void BtnSaveLog_Click(object sender, EventArgs e)
        {
            try
            {
                var saveDialog = new SaveFileDialog
                {
                    Filter = "文本文件|*.txt|所有文件|*.*",
                    Title = "保存日志文件"
                };

                if (saveDialog.ShowDialog() == DialogResult.OK)
                {
                    System.IO.File.WriteAllLines(saveDialog.FileName, 
                        lstDataLog.Items.Cast<string>().ToArray());
                    AddLog($"日志已保存到: {saveDialog.FileName}");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"保存日志失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void SLAMAutoController_OnStatusChanged(object sender, string status)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMAutoController_OnStatusChanged(sender, status)));
                return;
            }

            lblSLAMStatus.Text = $"状态: {status}";
            AddLog($"状态更新: {status}");
        }

        private void SLAMAutoController_OnMapUpdated(object sender, SLAMMapData mapData)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMAutoController_OnMapUpdated(sender, mapData)));
                return;
            }

            // 更新地图信息
            if (mapData.OccupancyGrid != null)
            {
                lblMapInfo.Text = $"地图: {mapData.OccupancyGrid.GetLength(0)}x{mapData.OccupancyGrid.GetLength(1)} 网格";
            }

            // 更新地图显示
            UpdateMapDisplay(mapData);
        }

        private void SLAMAutoController_OnPoseUpdated(object sender, RobotPose pose)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMAutoController_OnPoseUpdated(sender, pose)));
                return;
            }

            lblPoseInfo.Text = pose.ToString();
        }

        private void SLAMAutoController_OnError(object sender, Exception ex)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMAutoController_OnError(sender, ex)));
                return;
            }

            AddLog($"错误: {ex.Message}");
        }

        private void StatusUpdateTimer_Tick(object sender, EventArgs e)
        {
            if (!isInitialized) return;

            try
            {
                UpdateButtonStates();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"状态更新错误: {ex.Message}");
            }
        }
        #endregion

        #region 辅助方法
        private void UpdateButtonStates()
        {
            bool isRunning = slamAutoController.IsRunning;
            bool isPaused = slamAutoController.IsPaused;

            btnStartSLAM.Enabled = !isRunning;
            btnStopSLAM.Enabled = isRunning;
            btnPauseResume.Enabled = isRunning;
            btnPauseResume.Text = isPaused ? "恢复" : "暂停";
            btnPauseResume.BackColor = isPaused ? Color.LightGreen : Color.Orange;
        }

        private void UpdateMapDisplay(SLAMMapData mapData)
        {
            // 这里可以实现地图的图形显示
            // 由于这是一个简化的实现，我们只更新文本信息
            if (mapData.Trajectory != null)
            {
                lblMapTitle.Text = $"SLAM地图 (轨迹点: {mapData.Trajectory.Count})";
            }
        }

        private void AddLog(string message)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => AddLog(message)));
                return;
            }

            string timestamp = DateTime.Now.ToString("HH:mm:ss");
            lstDataLog.Items.Add($"[{timestamp}] {message}");
            
            // 保持最新的100条日志
            if (lstDataLog.Items.Count > 100)
            {
                lstDataLog.Items.RemoveAt(0);
            }
            
            // 自动滚动到底部
            lstDataLog.TopIndex = lstDataLog.Items.Count - 1;
        }
        #endregion

        #region 资源清理
        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            try
            {
                slamAutoController?.StopSLAMAutoControl();
                statusUpdateTimer?.Stop();
                statusUpdateTimer?.Dispose();
                slamAutoController?.Dispose();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"清理资源错误: {ex.Message}");
            }

            base.OnFormClosing(e);
        }
        #endregion
    }

    /// <summary>
    /// 自定义探索对话框
    /// </summary>
    public partial class CustomExplorationForm : Form
    {
        private List<RobotCommand> commands = new List<RobotCommand>();
        private ListBox lstCommands;
        private Button btnAddCommand;
        private Button btnRemoveCommand;
        private Button btnOK;
        private Button btnCancel;
        private ComboBox cmbDirection;
        private NumericUpDown nudDuration;

        public CustomExplorationForm()
        {
            InitializeComponent();
        }

        private void InitializeComponent()
        {
            this.Text = "自定义探索序列";
            this.Size = new Size(500, 400);
            this.StartPosition = FormStartPosition.CenterParent;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;

            // 命令列表
            lstCommands = new ListBox
            {
                Location = new Point(10, 10),
                Size = new Size(480, 200),
                Font = new Font("微软雅黑", 9F)
            };

            // 方向选择
            cmbDirection = new ComboBox
            {
                Location = new Point(10, 220),
                Size = new Size(100, 25),
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            cmbDirection.Items.AddRange(new string[] { "停止", "前进", "后退", "左转", "右转", "掉头" });
            cmbDirection.SelectedIndex = 1;

            // 持续时间
            nudDuration = new NumericUpDown
            {
                Location = new Point(120, 220),
                Size = new Size(80, 25),
                Minimum = 100,
                Maximum = 10000,
                Value = 1000,
                Increment = 100
            };

            // 按钮
            btnAddCommand = new Button
            {
                Text = "添加",
                Location = new Point(210, 220),
                Size = new Size(60, 25)
            };

            btnRemoveCommand = new Button
            {
                Text = "删除",
                Location = new Point(280, 220),
                Size = new Size(60, 25)
            };

            btnOK = new Button
            {
                Text = "确定",
                Location = new Point(320, 320),
                Size = new Size(80, 30),
                DialogResult = DialogResult.OK
            };

            btnCancel = new Button
            {
                Text = "取消",
                Location = new Point(410, 320),
                Size = new Size(80, 30),
                DialogResult = DialogResult.Cancel
            };

            // 事件处理
            btnAddCommand.Click += BtnAddCommand_Click;
            btnRemoveCommand.Click += BtnRemoveCommand_Click;

            // 添加控件
            this.Controls.AddRange(new Control[] {
                lstCommands, cmbDirection, nudDuration,
                btnAddCommand, btnRemoveCommand, btnOK, btnCancel
            });
        }

        private void BtnAddCommand_Click(object sender, EventArgs e)
        {
            byte direction = (byte)(cmbDirection.SelectedIndex);
            int duration = (int)nudDuration.Value;

            var command = new RobotCommand
            {
                Direction = direction,
                Duration = duration,
                Timestamp = DateTime.Now
            };

            commands.Add(command);
            UpdateCommandList();
        }

        private void BtnRemoveCommand_Click(object sender, EventArgs e)
        {
            if (lstCommands.SelectedIndex >= 0 && lstCommands.SelectedIndex < commands.Count)
            {
                commands.RemoveAt(lstCommands.SelectedIndex);
                UpdateCommandList();
            }
        }

        private void UpdateCommandList()
        {
            lstCommands.Items.Clear();
            foreach (var cmd in commands)
            {
                string directionName = GetDirectionName(cmd.Direction);
                lstCommands.Items.Add($"{directionName} ({cmd.Duration}ms)");
            }
        }

        private string GetDirectionName(byte direction)
        {
            string[] names = { "停止", "前进", "后退", "左转", "右转", "掉头" };
            return direction < names.Length ? names[direction] : "未知";
        }

        public List<RobotCommand> GetCommands()
        {
            return commands;
        }
    }
}
