using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;

namespace BluetoothApp
{
    /// <summary>
    /// 硬件Python SLAM集成控制表单
    /// </summary>
    public partial class HardwarePythonSLAMForm : Form
    {
        private PythonSLAMInterface pythonSLAM;
        private RobotController robotController;
        private System.Windows.Forms.Timer dataUpdateTimer;
        private System.Windows.Forms.Timer slamUpdateTimer;
        private bool isSLAMRunning = false;
        private List<RobotSensorData> sensorDataHistory = new List<RobotSensorData>();
        private SLAMResult currentSLAMResult = null;
        private readonly object lockObject = new object();

        // UI控件
        private Panel pnlMapDisplay;
        private PictureBox picMap;
        private Label lblStatus;
        private ListBox lstDataLog;
        private Button btnStartSLAM;
        private Button btnStopSLAM;
        private Button btnSaveData;
        private CheckBox chkShowMapPoints;
        private CheckBox chkShowTrajectory;
        private CheckBox chkShowLaserData;
        private CheckBox chkShowRobotPose;

        // 绘图相关
        private Bitmap mapBitmap;
        private Graphics mapGraphics;
        private PointF mapOffset = new PointF(200, 200);
        private float mapScale = 20.0f;

        public HardwarePythonSLAMForm(RobotController robotController)
        {
            this.robotController = robotController;
            InitializeComponent();
            InitializeSLAM();
        }

        private void InitializeComponent()
        {
            this.SuspendLayout();

            // 设置表单属性
            this.Text = "硬件Python SLAM集成控制";
            this.Size = new Size(1200, 800);
            this.StartPosition = FormStartPosition.CenterScreen;

            // 创建地图显示面板
            pnlMapDisplay = new Panel
            {
                Location = new Point(10, 10),
                Size = new Size(800, 600),
                BorderStyle = BorderStyle.FixedSingle,
                BackColor = Color.White
            };
            this.Controls.Add(pnlMapDisplay);

            // 创建地图图片框
            picMap = new PictureBox
            {
                Dock = DockStyle.Fill,
                SizeMode = PictureBoxSizeMode.Zoom,
                BackColor = Color.White
            };
            pnlMapDisplay.Controls.Add(picMap);

            // 创建状态标签
            lblStatus = new Label
            {
                Location = new Point(820, 10),
                Size = new Size(360, 30),
                Text = "状态: 未连接",
                Font = new Font("Arial", 10, FontStyle.Bold)
            };
            this.Controls.Add(lblStatus);

            // 创建数据日志
            lstDataLog = new ListBox
            {
                Location = new Point(820, 50),
                Size = new Size(360, 200),
                Font = new Font("Consolas", 8)
            };
            this.Controls.Add(lstDataLog);

            // 创建控制按钮
            btnStartSLAM = new Button
            {
                Location = new Point(820, 260),
                Size = new Size(100, 30),
                Text = "开始SLAM",
                BackColor = Color.LightGreen
            };
            btnStartSLAM.Click += BtnStartSLAM_Click;
            this.Controls.Add(btnStartSLAM);

            btnStopSLAM = new Button
            {
                Location = new Point(930, 260),
                Size = new Size(100, 30),
                Text = "停止SLAM",
                BackColor = Color.LightCoral,
                Enabled = false
            };
            btnStopSLAM.Click += BtnStopSLAM_Click;
            this.Controls.Add(btnStopSLAM);

            btnSaveData = new Button
            {
                Location = new Point(820, 300),
                Size = new Size(100, 30),
                Text = "保存数据"
            };
            btnSaveData.Click += BtnSaveData_Click;
            this.Controls.Add(btnSaveData);

            // 创建显示选项
            chkShowMapPoints = new CheckBox
            {
                Location = new Point(820, 340),
                Size = new Size(120, 20),
                Text = "显示地图点",
                Checked = true
            };
            this.Controls.Add(chkShowMapPoints);

            chkShowTrajectory = new CheckBox
            {
                Location = new Point(820, 365),
                Size = new Size(120, 20),
                Text = "显示轨迹",
                Checked = true
            };
            this.Controls.Add(chkShowTrajectory);

            chkShowLaserData = new CheckBox
            {
                Location = new Point(820, 390),
                Size = new Size(120, 20),
                Text = "显示激光数据",
                Checked = true
            };
            this.Controls.Add(chkShowLaserData);

            chkShowRobotPose = new CheckBox
            {
                Location = new Point(820, 415),
                Size = new Size(120, 20),
                Text = "显示机器人位姿",
                Checked = true
            };
            this.Controls.Add(chkShowRobotPose);

            this.ResumeLayout(false);
        }

        private async void InitializeSLAM()
        {
            try
            {
                // 初始化Python SLAM接口
                pythonSLAM = new PythonSLAMInterface();
                pythonSLAM.SLAMResultReceived += OnSLAMResultReceived;
                pythonSLAM.ErrorOccurred += OnSLAMErrorOccurred;

                // 初始化Python SLAM
                bool initialized = await pythonSLAM.InitializeAsync();
                if (initialized)
                {
                    AddLog("Python SLAM接口初始化成功");
                    lblStatus.Text = "状态: Python SLAM已连接";
                    lblStatus.ForeColor = Color.Green;
                }
                else
                {
                    AddLog("Python SLAM接口初始化失败");
                    lblStatus.Text = "状态: Python SLAM连接失败";
                    lblStatus.ForeColor = Color.Red;
                }
            }
            catch (Exception ex)
            {
                AddLog($"初始化SLAM失败: {ex.Message}");
            }
        }

        private async void BtnStartSLAM_Click(object sender, EventArgs e)
        {
            try
            {
                if (robotController == null || !robotController.IsConnected())
                {
                    MessageBox.Show("请先连接机器人！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                // 开始SLAM
                isSLAMRunning = true;
                btnStartSLAM.Enabled = false;
                btnStopSLAM.Enabled = true;
                lblStatus.Text = "状态: SLAM运行中";
                lblStatus.ForeColor = Color.Blue;

                // 启动数据更新定时器
                dataUpdateTimer = new System.Windows.Forms.Timer();
                dataUpdateTimer.Interval = 100; // 100ms更新一次
                dataUpdateTimer.Tick += DataUpdateTimer_Tick;
                dataUpdateTimer.Start();

                // 启动SLAM更新定时器
                slamUpdateTimer = new System.Windows.Forms.Timer();
                slamUpdateTimer.Interval = 500; // 500ms更新一次
                slamUpdateTimer.Tick += SLAMUpdateTimer_Tick;
                slamUpdateTimer.Start();

                AddLog("开始硬件Python SLAM建图");
            }
            catch (Exception ex)
            {
                AddLog($"启动SLAM失败: {ex.Message}");
            }
        }

        private void BtnStopSLAM_Click(object sender, EventArgs e)
        {
            try
            {
                // 停止SLAM
                isSLAMRunning = false;
                btnStartSLAM.Enabled = true;
                btnStopSLAM.Enabled = false;
                lblStatus.Text = "状态: SLAM已停止";
                lblStatus.ForeColor = Color.Orange;

                // 停止定时器
                dataUpdateTimer?.Stop();
                slamUpdateTimer?.Stop();

                AddLog("停止硬件Python SLAM建图");
            }
            catch (Exception ex)
            {
                AddLog($"停止SLAM失败: {ex.Message}");
            }
        }

        private async void BtnSaveData_Click(object sender, EventArgs e)
        {
            try
            {
                if (currentSLAMResult != null)
                {
                    bool saved = await pythonSLAM.SaveSLAMDataAsync(currentSLAMResult);
                    if (saved)
                    {
                        AddLog("SLAM数据保存成功");
                        MessageBox.Show("SLAM数据保存成功！", "成功", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    }
                    else
                    {
                        AddLog("SLAM数据保存失败");
                        MessageBox.Show("SLAM数据保存失败！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
                else
                {
                    MessageBox.Show("没有可保存的SLAM数据！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            }
            catch (Exception ex)
            {
                AddLog($"保存数据失败: {ex.Message}");
            }
        }

        private async void DataUpdateTimer_Tick(object sender, EventArgs e)
        {
            if (!isSLAMRunning || robotController == null) return;

            try
            {
                // 获取机器人传感器数据
                var sensorData = new RobotSensorData
                {
                    X = 0.0f, // 模拟X坐标，实际应该从里程计计算
                    Y = 0.0f, // 模拟Y坐标，实际应该从里程计计算
                    Theta = 0.0f, // 模拟朝向角度，实际应该从陀螺仪计算
                    LeftRPM = robotController.CurrentSensorData.MotorA_RPM,
                    RightRPM = robotController.CurrentSensorData.MotorB_RPM,
                    Velocity = (robotController.CurrentSensorData.MotorA_RPM + robotController.CurrentSensorData.MotorB_RPM) / 2.0f,
                    LaserData = GetLaserDataFromRobot(),
                    Timestamp = DateTime.Now
                };

                // 发送到Python SLAM
                bool sent = await pythonSLAM.SendSensorDataAsync(sensorData);
                if (sent)
                {
                    lock (lockObject)
                    {
                        sensorDataHistory.Add(sensorData);
                        // 保持最近1000条数据
                        if (sensorDataHistory.Count > 1000)
                        {
                            sensorDataHistory.RemoveAt(0);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                AddLog($"数据更新错误: {ex.Message}");
            }
        }

        private void SLAMUpdateTimer_Tick(object sender, EventArgs e)
        {
            if (!isSLAMRunning) return;

            try
            {
                // 更新地图显示
                UpdateMapDisplay();
            }
            catch (Exception ex)
            {
                AddLog($"地图更新错误: {ex.Message}");
            }
        }

        private List<LaserScanData> GetLaserDataFromRobot()
        {
            // 这里应该从机器人控制器获取激光数据
            // 暂时返回模拟数据
            var laserData = new List<LaserScanData>();
            
            // 模拟激光扫描数据
            for (int i = 0; i < 360; i += 5)
            {
                float angle = i * (float)Math.PI / 180.0f;
                float distance = 2.0f + (float)Math.Sin(angle * 3) * 0.5f; // 模拟距离
                
                laserData.Add(new LaserScanData
                {
                    Angle = angle,
                    Distance = distance,
                    X = (float)Math.Cos(angle) * distance,
                    Y = (float)Math.Sin(angle) * distance,
                    Quality = 100,
                    Timestamp = DateTime.Now
                });
            }
            
            return laserData;
        }

        private void OnSLAMResultReceived(object sender, SLAMResultEventArgs e)
        {
            try
            {
                lock (lockObject)
                {
                    currentSLAMResult = e.SLAMResult;
                }
                
                // 更新UI
                if (InvokeRequired)
                {
                    Invoke(new Action(() => UpdateSLAMResultDisplay()));
                }
                else
                {
                    UpdateSLAMResultDisplay();
                }
            }
            catch (Exception ex)
            {
                AddLog($"处理SLAM结果错误: {ex.Message}");
            }
        }

        private void OnSLAMErrorOccurred(object sender, string error)
        {
            AddLog($"SLAM错误: {error}");
        }

        private void UpdateSLAMResultDisplay()
        {
            if (currentSLAMResult != null)
            {
                lblStatus.Text = $"状态: SLAM运行中 - 地图点: {currentSLAMResult.MapPoints.Count}, 轨迹: {currentSLAMResult.Trajectory.Count}";
            }
        }

        private void UpdateMapDisplay()
        {
            try
            {
                if (mapBitmap == null)
                {
                    mapBitmap = new Bitmap(pnlMapDisplay.Width, pnlMapDisplay.Height);
                    mapGraphics = Graphics.FromImage(mapBitmap);
                }

                // 清空地图
                mapGraphics.Clear(Color.White);

                // 绘制坐标轴
                DrawCoordinateAxes();

                // 绘制SLAM结果
                if (currentSLAMResult != null)
                {
                    if (chkShowMapPoints.Checked && currentSLAMResult.MapPoints.Count > 0)
                    {
                        DrawMapPoints(currentSLAMResult.MapPoints);
                    }

                    if (chkShowTrajectory.Checked && currentSLAMResult.Trajectory.Count > 0)
                    {
                        DrawTrajectory(currentSLAMResult.Trajectory);
                    }

                    if (chkShowRobotPose.Checked && currentSLAMResult.OptimizedPoses.Count > 0)
                    {
                        DrawRobotPoses(currentSLAMResult.OptimizedPoses);
                    }
                }

                // 绘制激光数据
                if (chkShowLaserData.Checked && sensorDataHistory.Count > 0)
                {
                    var latestSensorData = sensorDataHistory.Last();
                    if (latestSensorData.LaserData.Count > 0)
                    {
                        DrawLaserData(latestSensorData.LaserData);
                    }
                }

                // 更新显示
                picMap.Image = mapBitmap;
            }
            catch (Exception ex)
            {
                AddLog($"地图显示更新错误: {ex.Message}");
            }
        }

        private void DrawCoordinateAxes()
        {
            var pen = new Pen(Color.Gray, 1);
            
            // X轴
            mapGraphics.DrawLine(pen, mapOffset.X, 0, mapOffset.X, pnlMapDisplay.Height);
            // Y轴
            mapGraphics.DrawLine(pen, 0, mapOffset.Y, pnlMapDisplay.Width, mapOffset.Y);
            
            pen.Dispose();
        }

        private void DrawMapPoints(List<float[]> mapPoints)
        {
            var brush = new SolidBrush(Color.Black);
            
            foreach (var point in mapPoints)
            {
                if (point.Length >= 2)
                {
                    float x = mapOffset.X + point[0] * mapScale;
                    float y = mapOffset.Y - point[1] * mapScale;
                    
                    if (x >= 0 && x < pnlMapDisplay.Width && y >= 0 && y < pnlMapDisplay.Height)
                    {
                        mapGraphics.FillEllipse(brush, x - 1, y - 1, 2, 2);
                    }
                }
            }
            
            brush.Dispose();
        }

        private void DrawTrajectory(List<float[]> trajectory)
        {
            if (trajectory.Count < 2) return;
            
            var pen = new Pen(Color.Blue, 2);
            var points = new PointF[trajectory.Count];
            
            for (int i = 0; i < trajectory.Count; i++)
            {
                if (trajectory[i].Length >= 2)
                {
                    points[i] = new PointF(
                        mapOffset.X + trajectory[i][0] * mapScale,
                        mapOffset.Y - trajectory[i][1] * mapScale
                    );
                }
            }
            
            mapGraphics.DrawLines(pen, points);
            pen.Dispose();
        }

        private void DrawRobotPoses(List<float[]> poses)
        {
            var brush = new SolidBrush(Color.Red);
            
            foreach (var pose in poses)
            {
                if (pose.Length >= 2)
                {
                    float x = mapOffset.X + pose[0] * mapScale;
                    float y = mapOffset.Y - pose[1] * mapScale;
                    
                    if (x >= 0 && x < pnlMapDisplay.Width && y >= 0 && y < pnlMapDisplay.Height)
                    {
                        mapGraphics.FillEllipse(brush, x - 2, y - 2, 4, 4);
                    }
                }
            }
            
            brush.Dispose();
        }

        private void DrawLaserData(List<LaserScanData> laserData)
        {
            var brush = new SolidBrush(Color.Red);
            
            foreach (var point in laserData)
            {
                float x = mapOffset.X + point.X * mapScale;
                float y = mapOffset.Y - point.Y * mapScale;
                
                if (x >= 0 && x < pnlMapDisplay.Width && y >= 0 && y < pnlMapDisplay.Height)
                {
                    mapGraphics.FillEllipse(brush, x - 1, y - 1, 2, 2);
                }
            }
            
            brush.Dispose();
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

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            try
            {
                isSLAMRunning = false;
                dataUpdateTimer?.Stop();
                slamUpdateTimer?.Stop();
                pythonSLAM?.Dispose();
                mapGraphics?.Dispose();
                mapBitmap?.Dispose();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"关闭表单错误: {ex.Message}");
            }
            
            base.OnFormClosing(e);
        }
    }
}
