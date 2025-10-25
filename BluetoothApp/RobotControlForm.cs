using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace BluetoothApp
{
    /// <summary>
    /// 小车控制界面
    /// </summary>
    public partial class RobotControlForm : Form
    {
        private BluetoothWindow mainForm;
        private Timer sensorUpdateTimer;
        private Label lblMotorA, lblMotorB, lblAccel, lblGyro, lblLidar, lblVoltage;
        private TextBox txtMotorA, txtMotorB, txtAccelX, txtAccelY, txtAccelZ;
        private TextBox txtGyroX, txtGyroY, txtGyroZ, txtLidarAngle, txtLidarDistance, txtLidarQuality;
        private TextBox txtVoltage;
        private GroupBox groupControl, groupSensor;
        private Button btnStop, btnForward, btnBackward, btnLeft, btnRight, btnUTurn;
        private Button btnEmergencyStop;
        private Label lblStatus;

        public RobotControlForm(BluetoothWindow mainForm)
        {
            this.mainForm = mainForm;
            InitializeComponent();
            InitializeSensorTimer();
        }

        private void InitializeComponent()
        {
            this.SuspendLayout();
            
            // 窗体设置
            this.Text = "小车控制面板";
            this.Size = new Size(600, 500);
            this.StartPosition = FormStartPosition.CenterParent;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;

            // 创建控制面板
            CreateControlPanel();
            
            // 创建传感器显示面板
            CreateSensorPanel();

            this.ResumeLayout(false);
        }

        private void CreateControlPanel()
        {
            groupControl = new GroupBox();
            groupControl.Text = "运动控制";
            groupControl.Location = new Point(20, 20);
            groupControl.Size = new Size(250, 200);

            // 停止按钮
            btnStop = new Button();
            btnStop.Text = "停止";
            btnStop.Location = new Point(95, 30);
            btnStop.Size = new Size(60, 30);
            btnStop.BackColor = Color.Red;
            btnStop.ForeColor = Color.White;
            btnStop.Click += (s, e) => mainForm.StopRobot();

            // 前进按钮
            btnForward = new Button();
            btnForward.Text = "前进";
            btnForward.Location = new Point(95, 70);
            btnForward.Size = new Size(60, 30);
            btnForward.Click += (s, e) => mainForm.MoveForward();

            // 左转按钮
            btnLeft = new Button();
            btnLeft.Text = "左转";
            btnLeft.Location = new Point(30, 110);
            btnLeft.Size = new Size(60, 30);
            btnLeft.Click += (s, e) => mainForm.TurnLeft();

            // 右转按钮
            btnRight = new Button();
            btnRight.Text = "右转";
            btnRight.Location = new Point(160, 110);
            btnRight.Size = new Size(60, 30);
            btnRight.Click += (s, e) => mainForm.TurnRight();

            // 后退按钮
            btnBackward = new Button();
            btnBackward.Text = "后退";
            btnBackward.Location = new Point(95, 150);
            btnBackward.Size = new Size(60, 30);
            btnBackward.Click += (s, e) => mainForm.MoveBackward();

            // 掉头按钮
            btnUTurn = new Button();
            btnUTurn.Text = "掉头";
            btnUTurn.Location = new Point(95, 110);
            btnUTurn.Size = new Size(60, 30);
            btnUTurn.BackColor = Color.Orange;
            btnUTurn.Click += (s, e) => mainForm.UTurn();

            // 紧急停止按钮
            btnEmergencyStop = new Button();
            btnEmergencyStop.Text = "紧急停止";
            btnEmergencyStop.Location = new Point(30, 190);
            btnEmergencyStop.Size = new Size(190, 30);
            btnEmergencyStop.BackColor = Color.DarkRed;
            btnEmergencyStop.ForeColor = Color.White;
            btnEmergencyStop.Font = new Font("微软雅黑", 10, FontStyle.Bold);
            btnEmergencyStop.Click += (s, e) => {
                mainForm.StopRobot();
                MessageBox.Show("紧急停止已激活！", "紧急停止", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            };

            groupControl.Controls.AddRange(new Control[] {
                btnStop, btnForward, btnBackward, btnLeft, btnRight, btnUTurn, btnEmergencyStop
            });

            this.Controls.Add(groupControl);
        }

        private void CreateSensorPanel()
        {
            groupSensor = new GroupBox();
            groupSensor.Text = "传感器数据";
            groupSensor.Location = new Point(290, 20);
            groupSensor.Size = new Size(280, 400);

            int yPos = 30;
            int labelWidth = 80;
            int textWidth = 100;
            int spacing = 25;

            // 电机数据
            lblMotorA = new Label();
            lblMotorA.Text = "电机A RPM:";
            lblMotorA.Location = new Point(20, yPos);
            lblMotorA.Size = new Size(labelWidth, 20);
            txtMotorA = new TextBox();
            txtMotorA.Location = new Point(110, yPos);
            txtMotorA.Size = new Size(textWidth, 20);
            txtMotorA.ReadOnly = true;
            yPos += spacing;

            lblMotorB = new Label();
            lblMotorB.Text = "电机B RPM:";
            lblMotorB.Location = new Point(20, yPos);
            lblMotorB.Size = new Size(labelWidth, 20);
            txtMotorB = new TextBox();
            txtMotorB.Location = new Point(110, yPos);
            txtMotorB.Size = new Size(textWidth, 20);
            txtMotorB.ReadOnly = true;
            yPos += spacing + 10;

            // 加速度数据
            lblAccel = new Label();
            lblAccel.Text = "加速度 (m/s²):";
            lblAccel.Location = new Point(20, yPos);
            lblAccel.Size = new Size(labelWidth, 20);
            yPos += 20;

            lblAccel = new Label();
            lblAccel.Text = "X:";
            lblAccel.Location = new Point(30, yPos);
            lblAccel.Size = new Size(20, 20);
            txtAccelX = new TextBox();
            txtAccelX.Location = new Point(50, yPos);
            txtAccelX.Size = new Size(60, 20);
            txtAccelX.ReadOnly = true;

            lblAccel = new Label();
            lblAccel.Text = "Y:";
            lblAccel.Location = new Point(120, yPos);
            lblAccel.Size = new Size(20, 20);
            txtAccelY = new TextBox();
            txtAccelY.Location = new Point(140, yPos);
            txtAccelY.Size = new Size(60, 20);
            txtAccelY.ReadOnly = true;

            lblAccel = new Label();
            lblAccel.Text = "Z:";
            lblAccel.Location = new Point(210, yPos);
            lblAccel.Size = new Size(20, 20);
            txtAccelZ = new TextBox();
            txtAccelZ.Location = new Point(230, yPos);
            txtAccelZ.Size = new Size(60, 20);
            txtAccelZ.ReadOnly = true;
            yPos += spacing + 10;

            // 陀螺仪数据
            lblGyro = new Label();
            lblGyro.Text = "陀螺仪 (deg/s):";
            lblGyro.Location = new Point(20, yPos);
            lblGyro.Size = new Size(labelWidth, 20);
            yPos += 20;

            lblGyro = new Label();
            lblGyro.Text = "X:";
            lblGyro.Location = new Point(30, yPos);
            lblGyro.Size = new Size(20, 20);
            txtGyroX = new TextBox();
            txtGyroX.Location = new Point(50, yPos);
            txtGyroX.Size = new Size(60, 20);
            txtGyroX.ReadOnly = true;

            lblGyro = new Label();
            lblGyro.Text = "Y:";
            lblGyro.Location = new Point(120, yPos);
            lblGyro.Size = new Size(20, 20);
            txtGyroY = new TextBox();
            txtGyroY.Location = new Point(140, yPos);
            txtGyroY.Size = new Size(60, 20);
            txtGyroY.ReadOnly = true;

            lblGyro = new Label();
            lblGyro.Text = "Z:";
            lblGyro.Location = new Point(210, yPos);
            lblGyro.Size = new Size(20, 20);
            txtGyroZ = new TextBox();
            txtGyroZ.Location = new Point(230, yPos);
            txtGyroZ.Size = new Size(60, 20);
            txtGyroZ.ReadOnly = true;
            yPos += spacing + 10;

            // LiDAR数据
            lblLidar = new Label();
            lblLidar.Text = "LiDAR数据:";
            lblLidar.Location = new Point(20, yPos);
            lblLidar.Size = new Size(labelWidth, 20);
            yPos += 20;

            lblLidar = new Label();
            lblLidar.Text = "角度:";
            lblLidar.Location = new Point(30, yPos);
            lblLidar.Size = new Size(40, 20);
            txtLidarAngle = new TextBox();
            txtLidarAngle.Location = new Point(70, yPos);
            txtLidarAngle.Size = new Size(60, 20);
            txtLidarAngle.ReadOnly = true;

            lblLidar = new Label();
            lblLidar.Text = "距离:";
            lblLidar.Location = new Point(140, yPos);
            lblLidar.Size = new Size(40, 20);
            txtLidarDistance = new TextBox();
            txtLidarDistance.Location = new Point(180, yPos);
            txtLidarDistance.Size = new Size(60, 20);
            txtLidarDistance.ReadOnly = true;

            lblLidar = new Label();
            lblLidar.Text = "质量:";
            lblLidar.Location = new Point(250, yPos);
            lblLidar.Size = new Size(30, 20);
            txtLidarQuality = new TextBox();
            txtLidarQuality.Location = new Point(280, yPos);
            txtLidarQuality.Size = new Size(40, 20);
            txtLidarQuality.ReadOnly = true;
            yPos += spacing + 10;

            // 电压数据
            lblVoltage = new Label();
            lblVoltage.Text = "电池电压:";
            lblVoltage.Location = new Point(20, yPos);
            lblVoltage.Size = new Size(80, 20);
            txtVoltage = new TextBox();
            txtVoltage.Location = new Point(100, yPos);
            txtVoltage.Size = new Size(80, 20);
            txtVoltage.ReadOnly = true;

            // 状态标签
            lblStatus = new Label();
            lblStatus.Text = "状态: 未连接";
            lblStatus.Location = new Point(20, yPos + 30);
            lblStatus.Size = new Size(240, 20);
            lblStatus.ForeColor = Color.Red;

            groupSensor.Controls.AddRange(new Control[] {
                lblMotorA, txtMotorA, lblMotorB, txtMotorB,
                txtAccelX, txtAccelY, txtAccelZ, txtGyroX, txtGyroY, txtGyroZ,
                txtLidarAngle, txtLidarDistance, txtLidarQuality, txtVoltage,
                lblStatus
            });

            this.Controls.Add(groupSensor);
        }

        private void InitializeSensorTimer()
        {
            sensorUpdateTimer = new Timer();
            sensorUpdateTimer.Interval = 100; // 100ms更新一次
            sensorUpdateTimer.Tick += UpdateSensorDisplay;
            sensorUpdateTimer.Start();
        }

        private void UpdateSensorDisplay(object sender, EventArgs e)
        {
            if (mainForm?.robotController == null) return;

            var sensorData = mainForm.robotController.CurrentSensorData;
            
            // 更新电机数据
            txtMotorA.Text = sensorData.MotorA_RPM.ToString("F1");
            txtMotorB.Text = sensorData.MotorB_RPM.ToString("F1");

            // 更新加速度数据
            txtAccelX.Text = sensorData.AccelX.ToString("F2");
            txtAccelY.Text = sensorData.AccelY.ToString("F2");
            txtAccelZ.Text = sensorData.AccelZ.ToString("F2");

            // 更新陀螺仪数据
            txtGyroX.Text = sensorData.GyroX.ToString("F2");
            txtGyroY.Text = sensorData.GyroY.ToString("F2");
            txtGyroZ.Text = sensorData.GyroZ.ToString("F2");

            // 更新LiDAR数据
            txtLidarAngle.Text = sensorData.LidarAngle.ToString("F1");
            txtLidarDistance.Text = sensorData.LidarDistance.ToString("F0");
            txtLidarQuality.Text = sensorData.LidarQuality.ToString();

            // 更新电压数据
            txtVoltage.Text = sensorData.Voltage.ToString("F2") + "V";

            // 更新连接状态
            if (mainForm.robotController.IsConnected())
            {
                lblStatus.Text = "状态: 已连接";
                lblStatus.ForeColor = Color.Green;
            }
            else
            {
                lblStatus.Text = "状态: 未连接";
                lblStatus.ForeColor = Color.Red;
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            sensorUpdateTimer?.Stop();
            sensorUpdateTimer?.Dispose();
            base.OnFormClosing(e);
        }
    }
}
