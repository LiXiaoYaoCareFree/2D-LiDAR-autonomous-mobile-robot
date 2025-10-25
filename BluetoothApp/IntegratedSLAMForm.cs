using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using System.IO;

namespace BluetoothApp
{
    /// <summary>
    /// 集成SLAM控制界面 - 支持JSON数据加载和实时建图显示
    /// </summary>
    public partial class IntegratedSLAMForm : Form
    {
        #region 私有字段
        private BluetoothWindow mainForm;
        private IntegratedSLAMController slamController;
        private AutoRobotController autoController;
        private System.Windows.Forms.Timer statusUpdateTimer;
        private bool isInitialized = false;
        
        // 地图显示相关
        private Bitmap mapBitmap;
        private Graphics mapGraphics;
        private float mapScale = 20.0f; // 像素/米
        private PointF mapOffset = new PointF(200, 200);
        #endregion

        #region 控件声明
        private GroupBox gbSLAMControl;
        private Button btnStartSLAM;
        private Button btnStopSLAM;
        private Button btnPauseResume;
        private Label lblStatus;
        private Label lblRobotState;
        private Label lblMapInfo;
        
        private GroupBox gbMapDisplay;
        private Panel pnlMapDisplay;
        private Label lblMapTitle;
        private CheckBox chkShowTrajectory;
        private CheckBox chkShowLaserData;
        private CheckBox chkShowRobotPose;
        private CheckBox chkShowMapPoints;
        
        private GroupBox gbDataLog;
        private ListBox lstDataLog;
        private Button btnClearLog;
        private Button btnSaveLog;
        
        private GroupBox gbMazeInfo;
        private Label lblMazeInfo;
        private TextBox txtMazeData;
        #endregion

        #region 构造函数
        public IntegratedSLAMForm(BluetoothWindow parentForm)
        {
            mainForm = parentForm;
            autoController = new AutoRobotController(mainForm, mainForm.robotController);
            slamController = new IntegratedSLAMController(mainForm, autoController);
            
            InitializeComponent();
            SetupEventHandlers();
            InitializeStatusUpdateTimer();
            InitializeMapDisplay();
            
            isInitialized = true;
        }
        #endregion

        #region 初始化方法
        private void InitializeComponent()
        {
            this.SuspendLayout();
            
            // 窗体设置
            this.Text = "集成SLAM控制 - 基于PoseGraph算法";
            this.Size = new Size(1000, 700);
            this.StartPosition = FormStartPosition.CenterParent;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;

            // SLAM控制组
            gbSLAMControl = new GroupBox
            {
                Text = "SLAM控制",
                Location = new Point(10, 10),
                Size = new Size(300, 200),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 开始/停止按钮
            btnStartSLAM = new Button
            {
                Text = "开始SLAM ",
                Location = new Point(20, 30),
                Size = new Size(150, 35),
                BackColor = Color.LightGreen,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold)
            };

            btnStopSLAM = new Button
            {
                Text = "停止SLAM",
                Location = new Point(20, 75),
                Size = new Size(100, 35),
                BackColor = Color.LightCoral,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold),
                Enabled = false
            };

            // 暂停/恢复按钮
            btnPauseResume = new Button
            {
                Text = "暂停",
                Location = new Point(130, 75),
                Size = new Size(80, 35),
                BackColor = Color.Orange,
                Font = new Font("微软雅黑", 9F, FontStyle.Bold),
                Enabled = false
            };

            // 状态标签
            lblStatus = new Label
            {
                Text = "状态: 未启动",
                Location = new Point(20, 120),
                Size = new Size(250, 20),
                Font = new Font("微软雅黑", 9F)
            };

            lblRobotState = new Label
            {
                Text = "机器人: (0.00, 0.00, 0.00°)",
                Location = new Point(20, 140),
                Size = new Size(250, 20),
                Font = new Font("微软雅黑", 9F)
            };

            lblMapInfo = new Label
            {
                Text = "地图: 未初始化",
                Location = new Point(20, 160),
                Size = new Size(250, 20),
                Font = new Font("微软雅黑", 9F)
            };

            // 地图显示组
            gbMapDisplay = new GroupBox
            {
                Text = "实时地图显示",
                Location = new Point(320, 10),
                Size = new Size(400, 300),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 地图显示面板
            pnlMapDisplay = new Panel
            {
                Location = new Point(20, 30),
                Size = new Size(360, 200),
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
                Location = new Point(20, 240),
                Size = new Size(80, 20),
                Checked = true,
                Font = new Font("微软雅黑", 8F)
            };

            chkShowLaserData = new CheckBox
            {
                Text = "显示激光数据",
                Location = new Point(110, 240),
                Size = new Size(80, 20),
                Checked = true,
                Font = new Font("微软雅黑", 8F)
            };

            chkShowRobotPose = new CheckBox
            {
                Text = "显示机器人位姿",
                Location = new Point(200, 240),
                Size = new Size(100, 20),
                Checked = true,
                Font = new Font("微软雅黑", 8F)
            };

            chkShowMapPoints = new CheckBox
            {
                Text = "显示地图点",
                Location = new Point(300, 240),
                Size = new Size(80, 20),
                Checked = true,
                Font = new Font("微软雅黑", 8F)
            };

            // 数据日志组
            gbDataLog = new GroupBox
            {
                Text = "数据日志",
                Location = new Point(730, 10),
                Size = new Size(250, 300),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            // 日志列表
            lstDataLog = new ListBox
            {
                Location = new Point(20, 30),
                Size = new Size(210, 200),
                Font = new Font("微软雅黑", 8F)
            };

            // 日志控制按钮
            btnClearLog = new Button
            {
                Text = "清空日志",
                Location = new Point(20, 240),
                Size = new Size(80, 25),
                BackColor = Color.LightGray,
                Font = new Font("微软雅黑", 8F)
            };

            btnSaveLog = new Button
            {
                Text = "保存日志",
                Location = new Point(110, 240),
                Size = new Size(80, 25),
                BackColor = Color.LightBlue,
                Font = new Font("微软雅黑", 8F)
            };

            // 迷宫信息组
            gbMazeInfo = new GroupBox
            {
                Text = "迷宫信息",
                Location = new Point(10, 220),
                Size = new Size(300, 200),
                Font = new Font("微软雅黑", 10F, FontStyle.Bold)
            };

            lblMazeInfo = new Label
            {
                Text = "迷宫数据: 未加载",
                Location = new Point(20, 30),
                Size = new Size(250, 20),
                Font = new Font("微软雅黑", 9F)
            };

            txtMazeData = new TextBox
            {
                Location = new Point(20, 60),
                Size = new Size(250, 120),
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                Font = new Font("微软雅黑", 8F)
            };

            // 添加控件到窗体
            gbSLAMControl.Controls.AddRange(new Control[] {
                btnStartSLAM, btnStopSLAM, btnPauseResume,
                lblStatus, lblRobotState, lblMapInfo
            });

            gbMapDisplay.Controls.AddRange(new Control[] {
                pnlMapDisplay, lblMapTitle, chkShowTrajectory,
                chkShowLaserData, chkShowRobotPose, chkShowMapPoints
            });

            gbDataLog.Controls.AddRange(new Control[] {
                lstDataLog, btnClearLog, btnSaveLog
            });

            gbMazeInfo.Controls.AddRange(new Control[] {
                lblMazeInfo, txtMazeData
            });

            this.Controls.AddRange(new Control[] {
                gbSLAMControl, gbMapDisplay, gbDataLog, gbMazeInfo
            });

            this.ResumeLayout(false);
        }

        private void SetupEventHandlers()
        {
            // SLAM控制事件
            btnStartSLAM.Click += BtnStartSLAM_Click;
            btnStopSLAM.Click += BtnStopSLAM_Click;
            btnPauseResume.Click += BtnPauseResume_Click;

            // 日志控制事件
            btnClearLog.Click += BtnClearLog_Click;
            btnSaveLog.Click += BtnSaveLog_Click;

            // 地图显示事件
            chkShowTrajectory.CheckedChanged += DisplayOptions_Changed;
            chkShowLaserData.CheckedChanged += DisplayOptions_Changed;
            chkShowRobotPose.CheckedChanged += DisplayOptions_Changed;
            chkShowMapPoints.CheckedChanged += DisplayOptions_Changed;

            // 集成SLAM控制器事件
            slamController.OnStatusChanged += SLAMController_OnStatusChanged;
            slamController.OnRobotStateChanged += SLAMController_OnRobotStateChanged;
            slamController.OnLaserDataReceived += SLAMController_OnLaserDataReceived;
            slamController.OnMapUpdated += SLAMController_OnMapUpdated;
            slamController.OnError += SLAMController_OnError;
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

        private void InitializeMapDisplay()
        {
            // 初始化地图显示
            mapBitmap = new Bitmap(pnlMapDisplay.Width, pnlMapDisplay.Height);
            mapGraphics = Graphics.FromImage(mapBitmap);
            mapGraphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            
            // 添加地图面板绘制事件
            pnlMapDisplay.Paint += PnlMapDisplay_Paint;
        }

        /// <summary>
        /// 地图面板绘制事件
        /// </summary>
        private void PnlMapDisplay_Paint(object sender, PaintEventArgs e)
        {
            try
            {
                if (mapBitmap != null)
                {
                    // 绘制地图位图
                    e.Graphics.DrawImage(mapBitmap, 0, 0);
                }
                else
                {
                    // 绘制默认背景
                    e.Graphics.Clear(Color.White);
                    e.Graphics.DrawString("等待地图数据...", new Font("微软雅黑", 12), Brushes.Gray, 10, 10);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"地图绘制错误: {ex.Message}");
            }
        }
        #endregion

        #region 事件处理方法

        /// <summary>
        /// 启动Python SLAM仿真
        /// </summary>
        private void StartPythonSLAMSimulation()
        {
            try
            {
                // 智能检测项目根目录
                string currentDir = Application.StartupPath;
                string projectRoot = FindProjectRoot(currentDir);
                string scriptPath = Path.Combine(projectRoot, "PoseGraph_Slam-Simulation", "maze_slam_simulation.py");
                string jsonPath = Path.Combine(projectRoot, "PoseGraph_Slam-Simulation", "4.json");
                
                AddLog($"项目根目录: {projectRoot}");
                AddLog($"Python脚本路径: {scriptPath}");
                AddLog($"JSON文件路径: {jsonPath}");
                
                // 检查文件是否存在
                if (!File.Exists(scriptPath))
                {
                    AddLog($"Python脚本不存在: {scriptPath}");
                    return;
                }
                
                if (!File.Exists(jsonPath))
                {
                    AddLog($"4.json文件不存在: {jsonPath}");
                    return;
                }

                // 直接启动Python SLAM仿真
                var pythonProcess = new System.Diagnostics.Process
                {
                    StartInfo = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = "python",
                        Arguments = $"\"{scriptPath}\" --map=\"{jsonPath}\"",
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        CreateNoWindow = false,
                        WorkingDirectory = Path.GetDirectoryName(scriptPath)
                    }
                };

                AddLog("正在启动Python SLAM仿真...");
                pythonProcess.Start();
                AddLog("Python SLAM仿真已启动，使用4.json地图数据");
                
                // 异步读取输出
                pythonProcess.BeginOutputReadLine();
                pythonProcess.BeginErrorReadLine();
                
                // 启动地图显示更新
                StartMapDisplayUpdate();
            }
            catch (Exception ex)
            {
                AddLog($"启动Python SLAM仿真失败: {ex.Message}");
                AddLog($"详细错误: {ex.StackTrace}");
            }
        }

        /// <summary>
        /// 启动地图显示更新
        /// </summary>
        private void StartMapDisplayUpdate()
        {
            try
            {
                // 启动地图显示定时器
                if (statusUpdateTimer == null)
                {
                    statusUpdateTimer = new System.Windows.Forms.Timer();
                    statusUpdateTimer.Interval = 100; // 100ms更新一次
                    statusUpdateTimer.Tick += StatusUpdateTimer_Tick;
                }
                
                statusUpdateTimer.Start();
                AddLog("地图显示更新已启动");
                
                // 初始化地图显示
                InitializeMapDisplay();
            }
            catch (Exception ex)
            {
                AddLog($"启动地图显示更新失败: {ex.Message}");
            }
        }

        /// <summary>
        /// 检查Python环境
        /// </summary>
        private bool CheckPythonEnvironment()
        {
            try
            {
                var process = new System.Diagnostics.Process
                {
                    StartInfo = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = "python",
                        Arguments = "--version",
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        CreateNoWindow = true
                    }
                };
                
                process.Start();
                string output = process.StandardOutput.ReadToEnd();
                process.WaitForExit();
                
                AddLog($"Python版本: {output.Trim()}");
                return process.ExitCode == 0;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// 检查Python依赖包
        /// </summary>
        private bool CheckPythonDependencies()
        {
            try
            {
                var process = new System.Diagnostics.Process
                {
                    StartInfo = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = "python",
                        Arguments = "-c \"import numpy, matplotlib, scipy; print('所有依赖包已安装')\"",
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        CreateNoWindow = true
                    }
                };
                
                process.Start();
                string output = process.StandardOutput.ReadToEnd();
                string error = process.StandardError.ReadToEnd();
                process.WaitForExit();
                
                if (process.ExitCode == 0)
                {
                    AddLog(output.Trim());
                    return true;
                }
                else
                {
                    AddLog($"依赖包检查失败: {error}");
                    return false;
                }
            }
            catch (Exception ex)
            {
                AddLog($"依赖包检查异常: {ex.Message}");
                return false;
            }
        }

        private void BtnStartSLAM_Click(object sender, EventArgs e)
        {
            try
            {
                // 启动集成SLAM（自动加载4.json数据）
                slamController.StartIntegratedSLAM();
                
                // 启动Python SLAM仿真
                StartPythonSLAMSimulation();
                
                UpdateButtonStates();
                AddLog("集成SLAM已启动，使用4.json数据，Python SLAM仿真已启动");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"启动集成SLAM失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnStopSLAM_Click(object sender, EventArgs e)
        {
            try
            {
                slamController.StopIntegratedSLAM();
                UpdateButtonStates();
                AddLog("集成SLAM已停止");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"停止集成SLAM失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                AddLog($"错误: {ex.Message}");
            }
        }

        private void BtnPauseResume_Click(object sender, EventArgs e)
        {
            try
            {
                slamController.TogglePause();
                UpdateButtonStates();
                string status = slamController.IsPaused ? "已暂停" : "已恢复";
                AddLog($"集成SLAM{status}");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"暂停/恢复操作失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
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
                    File.WriteAllLines(saveDialog.FileName, 
                        lstDataLog.Items.Cast<string>().ToArray());
                    AddLog($"日志已保存到: {saveDialog.FileName}");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"保存日志失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void DisplayOptions_Changed(object sender, EventArgs e)
        {
            // 显示选项改变时重新绘制地图
            UpdateMapDisplay();
        }

        private void SLAMController_OnStatusChanged(object sender, string status)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMController_OnStatusChanged(sender, status)));
                return;
            }

            lblStatus.Text = $"状态: {status}";
            AddLog($"状态更新: {status}");
        }

        private void SLAMController_OnRobotStateChanged(object sender, RobotState robotState)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMController_OnRobotStateChanged(sender, robotState)));
                return;
            }

            lblRobotState.Text = $"机器人: ({robotState.X:F2}, {robotState.Y:F2}, {robotState.Theta:F2}°)";
        }

        private void SLAMController_OnLaserDataReceived(object sender, List<LaserScanData> laserData)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMController_OnLaserDataReceived(sender, laserData)));
                return;
            }

            AddLog($"接收到激光数据: {laserData.Count} 个点");
        }

        private void SLAMController_OnMapUpdated(object sender, SLAMMapResult mapResult)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMController_OnMapUpdated(sender, mapResult)));
                return;
            }

            lblMapInfo.Text = $"地图: {mapResult.RobotPoses.Count} 个位姿, {mapResult.MapPoints.Count} 个地图点";
            UpdateMapDisplay(mapResult);
        }

        private void SLAMController_OnError(object sender, Exception ex)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => SLAMController_OnError(sender, ex)));
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
                UpdateMapDisplay();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"状态更新错误: {ex.Message}");
            }
        }

        /// <summary>
        /// 更新地图显示
        /// </summary>
        private void UpdateMapDisplay()
        {
            try
            {
                if (mapBitmap == null || mapGraphics == null) return;

                // 清空地图
                mapGraphics.Clear(Color.White);
                
                // 重新绘制迷宫
                DrawMazeFrom4Json();
                
                // 绘制机器人位置（模拟）
                DrawRobotPosition();
                
                // 绘制SLAM轨迹（模拟）
                DrawSLAMTrajectory();
                
                // 刷新显示
                pnlMapDisplay.Invalidate();
            }
            catch (Exception ex)
            {
                // 只在出现错误时输出日志，避免频繁输出
                System.Diagnostics.Debug.WriteLine($"地图显示更新错误: {ex.Message}");
            }
        }

        /// <summary>
        /// 绘制4.json迷宫数据
        /// </summary>
        private void DrawMazeFrom4Json()
        {
            try
            {
                // 设置绘制参数
                Pen wallPen = new Pen(Color.Black, 2);
                Brush startBrush = new SolidBrush(Color.Green);
                Brush goalBrush = new SolidBrush(Color.Red);
                
                // 绘制边界墙
                mapGraphics.DrawLine(wallPen, 0, 0, 0, 16);
                mapGraphics.DrawLine(wallPen, 0, 16, 16, 16);
                mapGraphics.DrawLine(wallPen, 0, 0, 16, 0);
                mapGraphics.DrawLine(wallPen, 16, 0, 16, 16);
                
                // 绘制内部墙壁
                mapGraphics.DrawLine(wallPen, 0, 12, 4, 12);
                mapGraphics.DrawLine(wallPen, 4, 4, 4, 8);
                mapGraphics.DrawLine(wallPen, 4, 8, 8, 8);
                mapGraphics.DrawLine(wallPen, 8, 0, 8, 4);
                mapGraphics.DrawLine(wallPen, 8, 8, 8, 12);
                mapGraphics.DrawLine(wallPen, 8, 12, 16, 12);
                mapGraphics.DrawLine(wallPen, 12, 4, 12, 12);
                
                // 绘制起始点
                mapGraphics.FillEllipse(startBrush, 2 - 0.5f, 14 - 0.5f, 1, 1);
                
                // 绘制目标点
                mapGraphics.FillEllipse(goalBrush, 14 - 0.5f, 10 - 0.5f, 1, 1);
                
                // 释放资源
                wallPen.Dispose();
                startBrush.Dispose();
                goalBrush.Dispose();
            }
            catch (Exception ex)
            {
                AddLog($"绘制迷宫数据失败: {ex.Message}");
            }
        }

        /// <summary>
        /// 绘制机器人位置
        /// </summary>
        private void DrawRobotPosition()
        {
            try
            {
                // 模拟机器人位置（从SLAM控制器获取）
                if (slamController != null)
                {
                    var robotState = slamController.CurrentRobotState;
                    if (robotState != null)
                    {
                        // 绘制机器人位置
                        Brush robotBrush = new SolidBrush(Color.Blue);
                        float robotX = robotState.X * 20 + 10; // 缩放和偏移
                        float robotY = robotState.Y * 20 + 10;
                        
                        mapGraphics.FillEllipse(robotBrush, robotX - 2, robotY - 2, 4, 4);
                        
                        // 绘制机器人朝向
                        Pen directionPen = new Pen(Color.Blue, 2);
                        float endX = robotX + (float)Math.Cos(robotState.Theta) * 5;
                        float endY = robotY + (float)Math.Sin(robotState.Theta) * 5;
                        mapGraphics.DrawLine(directionPen, robotX, robotY, endX, endY);
                        
                        robotBrush.Dispose();
                        directionPen.Dispose();
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"绘制机器人位置错误: {ex.Message}");
            }
        }

        /// <summary>
        /// 绘制SLAM轨迹
        /// </summary>
        private void DrawSLAMTrajectory()
        {
            try
            {
                // 模拟SLAM轨迹
                if (slamController != null)
                {
                    var trajectory = slamController.RobotTrajectory;
                    if (trajectory != null && trajectory.Count > 1)
                    {
                        Pen trajectoryPen = new Pen(Color.Green, 1);
                        
                        for (int i = 1; i < trajectory.Count; i++)
                        {
                            float x1 = trajectory[i-1].X * 20 + 10;
                            float y1 = trajectory[i-1].Y * 20 + 10;
                            float x2 = trajectory[i].X * 20 + 10;
                            float y2 = trajectory[i].Y * 20 + 10;
                            
                            mapGraphics.DrawLine(trajectoryPen, x1, y1, x2, y2);
                        }
                        
                        trajectoryPen.Dispose();
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"绘制SLAM轨迹错误: {ex.Message}");
            }
        }

        /// <summary>
        /// 智能查找项目根目录
        /// </summary>
        private string FindProjectRoot(string currentDir)
        {
            try
            {
                // 从当前目录开始向上查找，直到找到包含PoseGraph_Slam-Simulation的目录
                string searchDir = currentDir;
                
                for (int i = 0; i < 5; i++) // 最多向上查找5级目录
                {
                    string testPath = Path.Combine(searchDir, "PoseGraph_Slam-Simulation", "maze_slam_simulation.py");
                    if (File.Exists(testPath))
                    {
                        AddLog($"找到项目根目录: {searchDir}");
                        return searchDir;
                    }
                    
                    // 向上一级目录
                    string parentDir = Path.GetDirectoryName(searchDir);
                    if (parentDir == null || parentDir == searchDir)
                        break;
                    searchDir = parentDir;
                }
                
                // 如果没找到，使用默认路径
                string defaultRoot = Path.GetFullPath(Path.Combine(currentDir, "..", ".."));
                AddLog($"使用默认项目根目录: {defaultRoot}");
                return defaultRoot;
            }
            catch (Exception ex)
            {
                AddLog($"查找项目根目录失败: {ex.Message}");
                return Path.GetFullPath(Path.Combine(currentDir, "..", ".."));
            }
        }
        #endregion

        #region 辅助方法
        private void UpdateButtonStates()
        {
            bool isRunning = slamController.IsRunning;
            bool isPaused = slamController.IsPaused;

            btnStartSLAM.Enabled = !isRunning;
            btnStopSLAM.Enabled = isRunning;
            btnPauseResume.Enabled = isRunning;
            btnPauseResume.Text = isPaused ? "恢复" : "暂停";
            btnPauseResume.BackColor = isPaused ? Color.LightGreen : Color.Orange;
        }

        private void DisplayMazeInfo()
        {
            if (slamController.AllLaserData.Count > 0)
            {
                txtMazeData.Text = $"迷宫数据已加载\n" +
                                 $"机器人轨迹: {slamController.RobotTrajectory.Count} 个点\n" +
                                 $"激光数据: {slamController.AllLaserData.Count} 个点\n" +
                                 $"探索状态: {(slamController.ExplorationComplete ? "完成" : "进行中")}";
            }
        }

        private void UpdateMapDisplay(SLAMMapResult mapResult = null)
        {
            try
            {
                // 清空地图
                mapGraphics.Clear(Color.White);
                
                // 绘制背景网格
                DrawBackgroundGrid();
                
                // 绘制坐标轴
                DrawCoordinateAxes();
                
                if (mapResult != null)
                {
                    // 绘制地图点（障碍物）- 优先绘制，作为背景
                    if (chkShowMapPoints.Checked && mapResult.MapPoints.Count > 0)
                    {
                        DrawMapPoints(mapResult.MapPoints);
                    }
                    
                    // 绘制激光数据
                    if (chkShowLaserData.Checked && mapResult.LaserData.Count > 0)
                    {
                        DrawLaserData(mapResult.LaserData);
                    }
                    
                    // 绘制机器人轨迹
                    if (chkShowTrajectory.Checked && mapResult.Trajectory.Count > 1)
                    {
                        DrawTrajectory(mapResult.Trajectory);
                    }
                    
                    // 绘制机器人位姿
                    if (chkShowRobotPose.Checked && mapResult.RobotPoses.Count > 0)
                    {
                        DrawRobotPoses(mapResult.RobotPoses);
                    }
                    
                    // 绘制当前机器人位置和朝向
                    DrawCurrentRobotPose(mapResult);
                    
                    // 绘制实时雷达扫描
                    DrawRealTimeLidarScan(mapResult);
                    
                    // 绘制地图统计信息
                    DrawMapStatistics(mapResult);
                }
                
                // 更新显示
                pnlMapDisplay.BackgroundImage = mapBitmap;
                pnlMapDisplay.Invalidate();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"地图显示更新错误: {ex.Message}");
            }
        }

        private void DrawBackgroundGrid()
        {
            var pen = new Pen(Color.LightGray, 1);
            pen.DashStyle = System.Drawing.Drawing2D.DashStyle.Dot;
            
            // 绘制网格线
            float gridSpacing = 20.0f; // 网格间距
            
            // 垂直线
            for (float x = mapOffset.X; x < pnlMapDisplay.Width; x += gridSpacing)
            {
                mapGraphics.DrawLine(pen, x, 0, x, pnlMapDisplay.Height);
            }
            for (float x = mapOffset.X; x >= 0; x -= gridSpacing)
            {
                mapGraphics.DrawLine(pen, x, 0, x, pnlMapDisplay.Height);
            }
            
            // 水平线
            for (float y = mapOffset.Y; y < pnlMapDisplay.Height; y += gridSpacing)
            {
                mapGraphics.DrawLine(pen, 0, y, pnlMapDisplay.Width, y);
            }
            for (float y = mapOffset.Y; y >= 0; y -= gridSpacing)
            {
                mapGraphics.DrawLine(pen, 0, y, pnlMapDisplay.Width, y);
            }
            
            pen.Dispose();
        }

        private void DrawCoordinateAxes()
        {
            var pen = new Pen(Color.Gray, 2);
            
            // X轴
            mapGraphics.DrawLine(pen, mapOffset.X, 0, mapOffset.X, pnlMapDisplay.Height);
            // Y轴
            mapGraphics.DrawLine(pen, 0, mapOffset.Y, pnlMapDisplay.Width, mapOffset.Y);
            
            // 绘制坐标轴标签
            var font = new Font("Arial", 8);
            var brush = new SolidBrush(Color.Gray);
            
            // X轴标签
            for (int i = -10; i <= 20; i += 5)
            {
                float x = mapOffset.X + i * mapScale;
                if (x >= 0 && x <= pnlMapDisplay.Width)
                {
                    mapGraphics.DrawLine(pen, x, mapOffset.Y - 5, x, mapOffset.Y + 5);
                    mapGraphics.DrawString(i.ToString(), font, brush, x - 10, mapOffset.Y + 5);
                }
            }
            
            // Y轴标签
            for (int i = -10; i <= 20; i += 5)
            {
                float y = mapOffset.Y - i * mapScale;
                if (y >= 0 && y <= pnlMapDisplay.Height)
                {
                    mapGraphics.DrawLine(pen, mapOffset.X - 5, y, mapOffset.X + 5, y);
                    mapGraphics.DrawString(i.ToString(), font, brush, mapOffset.X + 5, y - 5);
                }
            }
            
            pen.Dispose();
            font.Dispose();
            brush.Dispose();
        }

        private void DrawTrajectory(List<RobotState> trajectory)
        {
            if (trajectory.Count < 2) return;
            
            // 使用渐变色绘制轨迹，模拟PoseGraph_Slam-Simulation的效果
            var recentPen = new Pen(Color.Red, 3);
            var olderPen = new Pen(Color.Orange, 2);
            var oldestPen = new Pen(Color.Yellow, 1);
            
            // 将轨迹分为三段，用不同颜色绘制
            int segmentSize = trajectory.Count / 3;
            if (segmentSize < 1) segmentSize = 1;
            
            // 绘制最新段（红色）
            if (trajectory.Count > segmentSize * 2)
            {
                var recentPoints = new PointF[segmentSize];
                for (int i = 0; i < segmentSize; i++)
                {
                    int index = trajectory.Count - segmentSize + i;
                    recentPoints[i] = new PointF(
                        mapOffset.X + trajectory[index].X * mapScale,
                        mapOffset.Y - trajectory[index].Y * mapScale
                    );
                }
                mapGraphics.DrawLines(recentPen, recentPoints);
            }
            
            // 绘制中间段（橙色）
            if (trajectory.Count > segmentSize)
            {
                var olderPoints = new PointF[segmentSize];
                for (int i = 0; i < segmentSize; i++)
                {
                    int index = trajectory.Count - segmentSize * 2 + i;
                    olderPoints[i] = new PointF(
                        mapOffset.X + trajectory[index].X * mapScale,
                        mapOffset.Y - trajectory[index].Y * mapScale
                    );
                }
                mapGraphics.DrawLines(olderPen, olderPoints);
            }
            
            // 绘制最旧段（黄色）
            var oldestPoints = new PointF[Math.Min(segmentSize, trajectory.Count - segmentSize * 2)];
            for (int i = 0; i < oldestPoints.Length; i++)
            {
                oldestPoints[i] = new PointF(
                    mapOffset.X + trajectory[i].X * mapScale,
                    mapOffset.Y - trajectory[i].Y * mapScale
                );
            }
            if (oldestPoints.Length > 1)
            {
                mapGraphics.DrawLines(oldestPen, oldestPoints);
            }
            
            recentPen.Dispose();
            olderPen.Dispose();
            oldestPen.Dispose();
        }

        private void DrawLaserData(List<LaserScanData> laserData)
        {
            if (laserData.Count == 0) return;
            
            // 使用渐变色绘制激光数据，模拟PoseGraph_Slam-Simulation的效果
            var recentBrush = new SolidBrush(Color.Red);
            var olderBrush = new SolidBrush(Color.Orange);
            var oldestBrush = new SolidBrush(Color.Yellow);
            
            // 按时间戳排序，最新的数据用红色，较旧的用橙色，最旧的用黄色
            var sortedData = laserData.OrderByDescending(p => p.Timestamp).ToList();
            
            int totalCount = sortedData.Count;
            int recentCount = Math.Min(totalCount / 3, 100); // 最近1/3的数据
            int olderCount = Math.Min(totalCount / 3, 100); // 中间1/3的数据
            
            for (int i = 0; i < totalCount; i++)
            {
                var point = sortedData[i];
                var screenPoint = new PointF(
                    mapOffset.X + point.X * mapScale,
                    mapOffset.Y - point.Y * mapScale
                );
                
                // 根据数据的新旧程度选择颜色
                if (i < recentCount)
                {
                    // 最新数据 - 红色，较大
                    mapGraphics.FillEllipse(recentBrush, screenPoint.X - 1.5f, screenPoint.Y - 1.5f, 3, 3);
                }
                else if (i < recentCount + olderCount)
                {
                    // 较旧数据 - 橙色，中等
                    mapGraphics.FillEllipse(olderBrush, screenPoint.X - 1, screenPoint.Y - 1, 2, 2);
                }
                else
                {
                    // 最旧数据 - 黄色，较小
                    mapGraphics.FillEllipse(oldestBrush, screenPoint.X - 0.5f, screenPoint.Y - 0.5f, 1, 1);
                }
            }
            
            recentBrush.Dispose();
            olderBrush.Dispose();
            oldestBrush.Dispose();
        }

        private void DrawRobotPoses(List<float[]> poses)
        {
            var brush = new SolidBrush(Color.Green);
            var pen = new Pen(Color.Green, 2);
            
            foreach (var pose in poses)
            {
                if (pose.Length >= 3)
                {
                    var screenPoint = new PointF(
                        mapOffset.X + pose[0] * mapScale,
                        mapOffset.Y - pose[1] * mapScale
                    );
                    
                    // 绘制机器人位置
                    mapGraphics.FillEllipse(brush, screenPoint.X - 3, screenPoint.Y - 3, 6, 6);
                    
                    // 绘制机器人朝向
                    float angle = pose[2];
                    float endX = screenPoint.X + 10 * (float)Math.Cos(angle);
                    float endY = screenPoint.Y - 10 * (float)Math.Sin(angle);
                    mapGraphics.DrawLine(pen, screenPoint.X, screenPoint.Y, endX, endY);
                }
            }
            
            brush.Dispose();
            pen.Dispose();
        }

        private void DrawMapPoints(List<float[]> mapPoints)
        {
            if (mapPoints.Count == 0) return;
            
            // 使用不同的颜色和大小绘制不同类型的地图点
            var obstacleBrush = new SolidBrush(Color.Black);
            var freeSpaceBrush = new SolidBrush(Color.LightGray);
            var unknownBrush = new SolidBrush(Color.Blue);
            
            // 为了提高性能，批量绘制相同类型的点
            var obstaclePoints = new List<PointF>();
            var freeSpacePoints = new List<PointF>();
            var unknownPoints = new List<PointF>();
            
            foreach (var point in mapPoints)
            {
                if (point.Length >= 2)
                {
                    var screenPoint = new PointF(
                        mapOffset.X + point[0] * mapScale,
                        mapOffset.Y - point[1] * mapScale
                    );
                    
                    // 根据点的密度或重要性分类
                    if (point.Length > 2 && point[2] > 0.8f) // 高置信度的障碍物
                    {
                        obstaclePoints.Add(screenPoint);
                    }
                    else if (point.Length > 2 && point[2] < 0.2f) // 自由空间
                    {
                        freeSpacePoints.Add(screenPoint);
                    }
                    else // 未知区域
                    {
                        unknownPoints.Add(screenPoint);
                    }
                }
            }
            
            // 批量绘制障碍物点
            if (obstaclePoints.Count > 0)
            {
                foreach (var point in obstaclePoints)
                {
                    mapGraphics.FillEllipse(obstacleBrush, point.X - 1.5f, point.Y - 1.5f, 3, 3);
                }
            }
            
            // 批量绘制自由空间点
            if (freeSpacePoints.Count > 0)
            {
                foreach (var point in freeSpacePoints)
                {
                    mapGraphics.FillEllipse(freeSpaceBrush, point.X - 0.5f, point.Y - 0.5f, 1, 1);
                }
            }
            
            // 批量绘制未知区域点
            if (unknownPoints.Count > 0)
            {
                foreach (var point in unknownPoints)
                {
                    mapGraphics.FillEllipse(unknownBrush, point.X - 1, point.Y - 1, 2, 2);
                }
            }
            
            obstacleBrush.Dispose();
            freeSpaceBrush.Dispose();
            unknownBrush.Dispose();
        }

        private void DrawCurrentRobotPose(SLAMMapResult mapResult)
        {
            if (mapResult.Trajectory.Count == 0) return;
            
            var currentPose = mapResult.Trajectory[mapResult.Trajectory.Count - 1];
            float x = mapOffset.X + currentPose.X * mapScale;
            float y = mapOffset.Y - currentPose.Y * mapScale;
            
            // 绘制机器人当前位置
            var robotBrush = new SolidBrush(Color.Red);
            var robotPen = new Pen(Color.DarkRed, 2);
            
            // 绘制机器人主体（圆形）
            mapGraphics.FillEllipse(robotBrush, x - 4, y - 4, 8, 8);
            mapGraphics.DrawEllipse(robotPen, x - 4, y - 4, 8, 8);
            
            // 绘制机器人朝向（箭头）
            float arrowLength = 12.0f;
            float arrowX = x + (float)Math.Cos(currentPose.Theta) * arrowLength;
            float arrowY = y - (float)Math.Sin(currentPose.Theta) * arrowLength;
            
            var arrowPen = new Pen(Color.DarkRed, 3);
            mapGraphics.DrawLine(arrowPen, x, y, arrowX, arrowY);
            
            // 绘制箭头头部
            float headLength = 6.0f;
            float headAngle = (float)Math.PI / 6;
            
            float headX1 = arrowX - (float)Math.Cos(currentPose.Theta - headAngle) * headLength;
            float headY1 = arrowY + (float)Math.Sin(currentPose.Theta - headAngle) * headLength;
            float headX2 = arrowX - (float)Math.Cos(currentPose.Theta + headAngle) * headLength;
            float headY2 = arrowY + (float)Math.Sin(currentPose.Theta + headAngle) * headLength;
            
            mapGraphics.DrawLine(arrowPen, arrowX, arrowY, headX1, headY1);
            mapGraphics.DrawLine(arrowPen, arrowX, arrowY, headX2, headY2);
            
            robotBrush.Dispose();
            robotPen.Dispose();
            arrowPen.Dispose();
        }

        private void DrawMapStatistics(SLAMMapResult mapResult)
        {
            // 绘制地图统计信息
            var font = new Font("Arial", 9, FontStyle.Bold);
            var brush = new SolidBrush(Color.DarkBlue);
            var backgroundBrush = new SolidBrush(Color.FromArgb(200, 255, 255, 255));
            
            // 创建信息框背景
            var infoRect = new RectangleF(10, 10, 200, 120);
            mapGraphics.FillRectangle(backgroundBrush, infoRect);
            mapGraphics.DrawRectangle(new Pen(Color.Gray, 1), infoRect.X, infoRect.Y, infoRect.Width, infoRect.Height);
            
            // 绘制统计信息
            string[] info = {
                $"地图点数: {mapResult.MapPoints.Count}",
                $"轨迹点数: {mapResult.Trajectory.Count}",
                $"激光数据: {mapResult.LaserData.Count}",
                $"位姿数: {mapResult.RobotPoses.Count}",
                $"时间: {mapResult.Timestamp:HH:mm:ss}"
            };
            
            for (int i = 0; i < info.Length; i++)
            {
                mapGraphics.DrawString(info[i], font, brush, 15, 20 + i * 18);
            }
            
            font.Dispose();
            brush.Dispose();
            backgroundBrush.Dispose();
        }

        private void DrawRealTimeLidarScan(SLAMMapResult mapResult)
        {
            if (mapResult.LaserData.Count == 0) return;
            
            // 获取最新的激光扫描数据
            var latestScan = mapResult.LaserData
                .Where(l => l.Timestamp > DateTime.Now.AddSeconds(-1)) // 最近1秒的数据
                .OrderByDescending(l => l.Timestamp)
                .Take(50) // 最多显示50个点
                .ToList();
            
            if (latestScan.Count == 0) return;
            
            // 绘制雷达扫描范围圆
            var currentPose = mapResult.Trajectory.Count > 0 ? mapResult.Trajectory[mapResult.Trajectory.Count - 1] : null;
            if (currentPose != null)
            {
                float robotX = mapOffset.X + currentPose.X * mapScale;
                float robotY = mapOffset.Y - currentPose.Y * mapScale;
                
                // 绘制雷达范围圆
                var rangePen = new Pen(Color.LightBlue, 1);
                rangePen.DashStyle = System.Drawing.Drawing2D.DashStyle.Dot;
                
                for (int r = 1; r <= 4; r++)
                {
                    float radius = r * 20 * mapScale; // 1米、2米、3米、4米
                    mapGraphics.DrawEllipse(rangePen, robotX - radius, robotY - radius, radius * 2, radius * 2);
                }
                
                // 绘制激光扫描点
                var scanBrush = new SolidBrush(Color.Red);
                foreach (var point in latestScan)
                {
                    float x = mapOffset.X + point.X * mapScale;
                    float y = mapOffset.Y - point.Y * mapScale;
                    mapGraphics.FillEllipse(scanBrush, x - 2, y - 2, 4, 4);
                }
                
                rangePen.Dispose();
                scanBrush.Dispose();
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
                slamController?.StopIntegratedSLAM();
                statusUpdateTimer?.Stop();
                statusUpdateTimer?.Dispose();
                slamController?.Dispose();
                mapGraphics?.Dispose();
                mapBitmap?.Dispose();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"清理资源错误: {ex.Message}");
            }

            base.OnFormClosing(e);
        }
        #endregion
    }
}
