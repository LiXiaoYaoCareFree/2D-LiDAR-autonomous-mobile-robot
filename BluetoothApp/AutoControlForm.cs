using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace BluetoothApp
{
    /// <summary>
    /// 自动控制界面
    /// </summary>
    public partial class AutoControlForm : Form
    {
        #region 私有字段
        private BluetoothWindow mainForm;
        private AutoRobotController autoController;
        private System.Windows.Forms.Timer statusUpdateTimer;
        private bool isInitialized = false;
        #endregion

        #region 控件声明
        private GroupBox gbAutoControl;
        private Button btnStartAuto;
        private Button btnStopAuto;
        private Button btnPauseResume;
        private Button btnClearQueue;
        private Button btnMazeExploration;
        private Button btnSLAMMapping;
        private Label lblStatus;
        private Label lblQueueCount;
        private ListBox lstCommandQueue;
        private GroupBox gbManualCommands;
        private Button btnAddForward;
        private Button btnAddBackward;
        private Button btnAddLeft;
        private Button btnAddRight;
        private Button btnAddUTurn;
        private Button btnAddStop;
        private NumericUpDown nudDuration;
        private Label lblDuration;
        private GroupBox gbPresetSequences;
        private Button btnAddExplorationSeq;
        private Button btnAddSLAMSeq;
        private Button btnAddCustomSeq;
        #endregion

        #region 构造函数
        public AutoControlForm(BluetoothWindow parentForm)
        {
            mainForm = parentForm;
            autoController = new AutoRobotController(mainForm, mainForm.robotController);
            
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
            this.Text = "自动机器人控制";
            this.Size = new Size(600, 500);
            this.StartPosition = FormStartPosition.CenterParent;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;

            // 自动控制组
            gbAutoControl = new GroupBox
            {
                Text = "自动控制",
                Location = new Point(10, 10),
                Size = new Size(280, 200),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 开始/停止按钮
            btnStartAuto = new Button
            {
                Text = "开始自动控制",
                Location = new Point(20, 30),
                Size = new Size(100, 35),
                BackColor = Color.LightGreen,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            btnStopAuto = new Button
            {
                Text = "停止自动控制",
                Location = new Point(130, 30),
                Size = new Size(100, 35),
                BackColor = Color.LightCoral,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold),
                Enabled = false
            };

            // 暂停/恢复按钮
            btnPauseResume = new Button
            {
                Text = "暂停",
                Location = new Point(20, 75),
                Size = new Size(100, 35),
                BackColor = Color.Orange,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold),
                Enabled = false
            };

            // 清空队列按钮
            btnClearQueue = new Button
            {
                Text = "清空队列",
                Location = new Point(130, 75),
                Size = new Size(100, 35),
                BackColor = Color.LightBlue,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            // 状态标签
            lblStatus = new Label
            {
                Text = "状态: 未启动",
                Location = new Point(20, 120),
                Size = new Size(200, 20),
                Font = new Font("微软雅黑", 9F)
            };

            lblQueueCount = new Label
            {
                Text = "队列: 0 个命令",
                Location = new Point(20, 145),
                Size = new Size(200, 20),
                Font = new Font("微软雅黑", 9F)
            };

            // 命令队列列表
            lstCommandQueue = new ListBox
            {
                Location = new Point(20, 170),
                Size = new Size(240, 20),
                Font = new Font("微软雅黑", 8F)
            };

            // 手动命令组
            gbManualCommands = new GroupBox
            {
                Text = "手动添加命令",
                Location = new Point(300, 10),
                Size = new Size(280, 200),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 方向按钮
            btnAddForward = new Button
            {
                Text = "前进",
                Location = new Point(20, 30),
                Size = new Size(60, 30),
                BackColor = Color.LightGreen,
                Font = new Font("微软雅黑", 8F)
            };

            btnAddBackward = new Button
            {
                Text = "后退",
                Location = new Point(90, 30),
                Size = new Size(60, 30),
                BackColor = Color.LightCoral,
                Font = new Font("微软雅黑", 8F)
            };

            btnAddLeft = new Button
            {
                Text = "左转",
                Location = new Point(160, 30),
                Size = new Size(60, 30),
                BackColor = Color.LightBlue,
                Font = new Font("微软雅黑", 8F)
            };

            btnAddRight = new Button
            {
                Text = "右转",
                Location = new Point(20, 70),
                Size = new Size(60, 30),
                BackColor = Color.LightBlue,
                Font = new Font("微软雅黑", 8F)
            };

            btnAddUTurn = new Button
            {
                Text = "掉头",
                Location = new Point(90, 70),
                Size = new Size(60, 30),
                BackColor = Color.Orange,
                Font = new Font("微软雅黑", 8F)
            };

            btnAddStop = new Button
            {
                Text = "停止",
                Location = new Point(160, 70),
                Size = new Size(60, 30),
                BackColor = Color.Gray,
                Font = new Font("微软雅黑", 8F)
            };

            // 持续时间设置
            lblDuration = new Label
            {
                Text = "持续时间(ms):",
                Location = new Point(20, 110),
                Size = new Size(100, 20),
                Font = new Font("微软雅黑", 9F)
            };

            nudDuration = new NumericUpDown
            {
                Location = new Point(120, 110),
                Size = new Size(80, 20),
                Minimum = 100,
                Maximum = 10000,
                Value = 1000,
                Increment = 100
            };

            // 预设序列组
            gbPresetSequences = new GroupBox
            {
                Text = "预设序列",
                Location = new Point(10, 220),
                Size = new Size(570, 100),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 迷宫探索按钮
            btnMazeExploration = new Button
            {
                Text = "迷宫探索序列",
                Location = new Point(20, 30),
                Size = new Size(120, 35),
                BackColor = Color.LightGreen,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            // SLAM建图按钮
            btnSLAMMapping = new Button
            {
                Text = "SLAM建图序列",
                Location = new Point(150, 30),
                Size = new Size(120, 35),
                BackColor = Color.LightBlue,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            // 添加自定义序列按钮
            btnAddExplorationSeq = new Button
            {
                Text = "添加探索序列",
                Location = new Point(280, 30),
                Size = new Size(100, 35),
                BackColor = Color.LightYellow,
                Font = new Font("微软雅黑", 8F)
            };

            btnAddSLAMSeq = new Button
            {
                Text = "添加SLAM序列",
                Location = new Point(390, 30),
                Size = new Size(100, 35),
                BackColor = Color.LightCyan,
                Font = new Font("微软雅黑", 8F)
            };

            btnAddCustomSeq = new Button
            {
                Text = "自定义序列",
                Location = new Point(500, 30),
                Size = new Size(60, 35),
                BackColor = Color.LightPink,
                Font = new Font("微软雅黑", 8F)
            };

            // 添加控件到窗体
            gbAutoControl.Controls.AddRange(new Control[] {
                btnStartAuto, btnStopAuto, btnPauseResume, btnClearQueue,
                lblStatus, lblQueueCount, lstCommandQueue
            });

            gbManualCommands.Controls.AddRange(new Control[] {
                btnAddForward, btnAddBackward, btnAddLeft, btnAddRight,
                btnAddUTurn, btnAddStop, lblDuration, nudDuration
            });

            gbPresetSequences.Controls.AddRange(new Control[] {
                btnMazeExploration, btnSLAMMapping, btnAddExplorationSeq,
                btnAddSLAMSeq, btnAddCustomSeq
            });

            this.Controls.AddRange(new Control[] {
                gbAutoControl, gbManualCommands, gbPresetSequences
            });

            this.ResumeLayout(false);
        }

        private void SetupEventHandlers()
        {
            // 自动控制事件
            btnStartAuto.Click += BtnStartAuto_Click;
            btnStopAuto.Click += BtnStopAuto_Click;
            btnPauseResume.Click += BtnPauseResume_Click;
            btnClearQueue.Click += BtnClearQueue_Click;

            // 手动命令事件
            btnAddForward.Click += (s, e) => AddCommand(AutoRobotController.DIR_FORWARD);
            btnAddBackward.Click += (s, e) => AddCommand(AutoRobotController.DIR_BACKWARD);
            btnAddLeft.Click += (s, e) => AddCommand(AutoRobotController.DIR_LEFT);
            btnAddRight.Click += (s, e) => AddCommand(AutoRobotController.DIR_RIGHT);
            btnAddUTurn.Click += (s, e) => AddCommand(AutoRobotController.DIR_UTURN);
            btnAddStop.Click += (s, e) => AddCommand(AutoRobotController.DIR_STOP);

            // 预设序列事件
            btnMazeExploration.Click += BtnMazeExploration_Click;
            btnSLAMMapping.Click += BtnSLAMMapping_Click;
            btnAddExplorationSeq.Click += BtnAddExplorationSeq_Click;
            btnAddSLAMSeq.Click += BtnAddSLAMSeq_Click;
            btnAddCustomSeq.Click += BtnAddCustomSeq_Click;

            // 自动控制器事件
            autoController.OnStatusChanged += AutoController_OnStatusChanged;
            autoController.OnCommandSent += AutoController_OnCommandSent;
            autoController.OnError += AutoController_OnError;
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
        private void BtnStartAuto_Click(object sender, EventArgs e)
        {
            try
            {
                autoController.StartAutoControl();
                UpdateButtonStates();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"启动自动控制失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnStopAuto_Click(object sender, EventArgs e)
        {
            try
            {
                autoController.StopAutoControl();
                UpdateButtonStates();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"停止自动控制失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnPauseResume_Click(object sender, EventArgs e)
        {
            try
            {
                autoController.TogglePause();
                UpdateButtonStates();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"暂停/恢复操作失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnClearQueue_Click(object sender, EventArgs e)
        {
            try
            {
                autoController.ClearCommandQueue();
                UpdateCommandQueueDisplay();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"清空队列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnMazeExploration_Click(object sender, EventArgs e)
        {
            try
            {
                autoController.ExecuteMazeExplorationSequence();
                UpdateCommandQueueDisplay();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"执行迷宫探索序列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnSLAMMapping_Click(object sender, EventArgs e)
        {
            try
            {
                autoController.ExecuteSLAMMappingSequence();
                UpdateCommandQueueDisplay();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"执行SLAM建图序列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnAddExplorationSeq_Click(object sender, EventArgs e)
        {
            try
            {
                var commands = GenerateMazeExplorationCommands();
                autoController.AddCommandSequence(commands);
                UpdateCommandQueueDisplay();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"添加探索序列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnAddSLAMSeq_Click(object sender, EventArgs e)
        {
            try
            {
                var commands = GenerateSLAMMappingCommands();
                autoController.AddCommandSequence(commands);
                UpdateCommandQueueDisplay();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"添加SLAM序列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnAddCustomSeq_Click(object sender, EventArgs e)
        {
            try
            {
                // 显示自定义序列对话框
                var customForm = new CustomSequenceForm();
                if (customForm.ShowDialog() == DialogResult.OK)
                {
                    autoController.AddCommandSequence(customForm.GetCommands());
                    UpdateCommandQueueDisplay();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"添加自定义序列失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void AutoController_OnStatusChanged(object sender, string status)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => AutoController_OnStatusChanged(sender, status)));
                return;
            }

            lblStatus.Text = $"状态: {status}";
        }

        private void AutoController_OnCommandSent(object sender, byte command)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => AutoController_OnCommandSent(sender, command)));
                return;
            }

            // 可以在这里添加命令发送的日志显示
        }

        private void AutoController_OnError(object sender, Exception ex)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => AutoController_OnError(sender, ex)));
                return;
            }

            MessageBox.Show($"自动控制错误: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }

        private void StatusUpdateTimer_Tick(object sender, EventArgs e)
        {
            if (!isInitialized) return;

            try
            {
                UpdateButtonStates();
                UpdateCommandQueueDisplay();
            }
            catch (Exception ex)
            {
                // 静默处理定时器错误
                System.Diagnostics.Debug.WriteLine($"状态更新错误: {ex.Message}");
            }
        }
        #endregion

        #region 辅助方法
        private void AddCommand(byte direction)
        {
            try
            {
                int duration = (int)nudDuration.Value;
                autoController.AddCommand(direction, duration);
                UpdateCommandQueueDisplay();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"添加命令失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void UpdateButtonStates()
        {
            bool isRunning = autoController.IsRunning;
            bool isPaused = autoController.IsPaused;

            btnStartAuto.Enabled = !isRunning;
            btnStopAuto.Enabled = isRunning;
            btnPauseResume.Enabled = isRunning;
            btnPauseResume.Text = isPaused ? "恢复" : "暂停";
            btnPauseResume.BackColor = isPaused ? Color.LightGreen : Color.Orange;
        }

        private void UpdateCommandQueueDisplay()
        {
            try
            {
                lblQueueCount.Text = $"队列: {autoController.QueueCount} 个命令";
                
                // 这里可以添加更详细的队列显示逻辑
                // 由于队列是私有的，我们只能显示数量
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"更新队列显示错误: {ex.Message}");
            }
        }

        private List<RobotCommand> GenerateMazeExplorationCommands()
        {
            var commands = new List<RobotCommand>();
            
            // 生成迷宫探索命令序列
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 2000 });
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_LEFT, Duration = 1000 });
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 1500 });
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_RIGHT, Duration = 1000 });
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 2000 });
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_UTURN, Duration = 2000 });
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 1000 });
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_STOP, Duration = 500 });

            return commands;
        }

        private List<RobotCommand> GenerateSLAMMappingCommands()
        {
            var commands = new List<RobotCommand>();
            
            // 生成SLAM建图命令序列
            for (int i = 0; i < 4; i++)
            {
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 1000 });
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_LEFT, Duration = 1000 });
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_FORWARD, Duration = 500 });
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_RIGHT, Duration = 2000 });
                commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_LEFT, Duration = 1000 });
            }
            
            commands.Add(new RobotCommand { Direction = AutoRobotController.DIR_STOP, Duration = 1000 });

            return commands;
        }
        #endregion

        #region 资源清理
        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            try
            {
                autoController?.StopAutoControl();
                statusUpdateTimer?.Stop();
                statusUpdateTimer?.Dispose();
                autoController?.Dispose();
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
    /// 自定义序列对话框
    /// </summary>
    public partial class CustomSequenceForm : Form
    {
        private List<RobotCommand> commands = new List<RobotCommand>();
        private ListBox lstCommands;
        private Button btnAddCommand;
        private Button btnRemoveCommand;
        private Button btnOK;
        private Button btnCancel;
        private ComboBox cmbDirection;
        private NumericUpDown nudDuration;

        public CustomSequenceForm()
        {
            InitializeComponent();
        }

        private void InitializeComponent()
        {
            this.Text = "自定义命令序列";
            this.Size = new Size(400, 300);
            this.StartPosition = FormStartPosition.CenterParent;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;

            // 命令列表
            lstCommands = new ListBox
            {
                Location = new Point(10, 10),
                Size = new Size(380, 150),
                Font = new Font("微软雅黑", 9F)
            };

            // 方向选择
            cmbDirection = new ComboBox
            {
                Location = new Point(10, 170),
                Size = new Size(100, 25),
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            cmbDirection.Items.AddRange(new string[] { "停止", "前进", "后退", "左转", "右转", "掉头" });
            cmbDirection.SelectedIndex = 1;

            // 持续时间
            nudDuration = new NumericUpDown
            {
                Location = new Point(120, 170),
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
                Location = new Point(210, 170),
                Size = new Size(60, 25)
            };

            btnRemoveCommand = new Button
            {
                Text = "删除",
                Location = new Point(280, 170),
                Size = new Size(60, 25)
            };

            btnOK = new Button
            {
                Text = "确定",
                Location = new Point(220, 220),
                Size = new Size(80, 30),
                DialogResult = DialogResult.OK
            };

            btnCancel = new Button
            {
                Text = "取消",
                Location = new Point(310, 220),
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
