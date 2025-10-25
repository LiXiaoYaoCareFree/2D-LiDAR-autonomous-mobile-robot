using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using static BluetoothApp.BluetoothInfo;

/*
  特别说明：HC-PC是广州汇承信息科技有限公司开发的PC软件，方便用户调试蓝牙模块。
  本软件提供代码和注释，免费给购买汇承蓝牙模块的用户学习和研究，但不能用于商业开发，
  最终解析权在广州汇承信息科技有限公司。
  :)
  **/

namespace BluetoothApp
{
    public partial class BluetoothWindow : Form
    {
        private int sendNum = 0;
        private int receiveNum = 0;
        private int checkLog = 0;//点击启动日志计数
        private MyTimer mTimer;//定时器
        private BluetoothManage manage;
        private int bluetoothMode;
        private Dictionary<string, string> hashMap;
        private bool isShowBox = true;//防止短时间内过多弹窗
        public RobotController robotController;
        private MazeExplorationForm mazeExplorationForm;
        private AdvancedMazeExplorationForm advancedMazeExplorationForm;
        private AutoControlForm autoControlForm;
        private SLAMAutoControlForm slamAutoControlForm;
        private IntegratedSLAMForm integratedSLAMForm;

        public BluetoothWindow()
        {
            // 监听载入资源
            AppDomain.CurrentDomain.AssemblyResolve += Utility.CurrentDomain_AssemblyResolve;
            InitializeComponent();
            InitImage();
            InitView();
            InitBleBluetooth(CheckbluetoothState());
        }

        private void InitBleBluetooth(bool bluetoothOpen)
        {
            if (bluetoothOpen)
            {
                bluetoothMode = 0;
                mTimer = new MyTimer();
                manage = new BluetoothManage(this);
                manage.SetListener(BluetoothCallback);
                
                // 初始化小车控制器
                robotController = new RobotController(manage);
                robotController.OnSensorDataReceived += OnSensorDataReceived;
                robotController.OnStatusChanged += OnRobotStatusChanged;
                robotController.OnConnectionChanged += OnRobotConnectionChanged;
            }
        }

        private void InitView()
        {
            this.modeSelection.SelectedIndex = 0;
            this.bluetoothListView.LargeImageList = this.bluetoothImageList;//Title模式必须要设置这个图片连接模式
            this.bluetoothListView.SmallImageList = this.bluetoothImageList;//将ListView的图标集与imageList绑定
            this.bluetoothListView.Columns.Add("蓝牙名称", 120, HorizontalAlignment.Left);
            this.bluetoothListView.Columns.Add("蓝牙地址", 120, HorizontalAlignment.Left);//必须要有两个组，才能显示
            this.bluetoothListView.Cursor = Cursors.Hand;//设置鼠标为手指型
            this.bluetoothListView.SelectedIndexChanged += new EventHandler(ItemClickListener);
            this.groupLog.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            if (!Utility.IsWindows8_OrHigher()) modeSelection.Enabled = false;
        }

        private void InitImage()
        {
            bluetoothImageList.Images.Add(BluetoothApp.Properties.Resources.item_src_2);
            bluetoothImageList.Images.Add(BluetoothApp.Properties.Resources.item_src_ble);
            bluetoothImageList.ImageSize = new Size(30, 30);
            hashMap = new Dictionary<string, string>();
        }


        private void ItemClickListener(object sender, EventArgs e)
        {
            ListView list = (ListView)sender;
            if (list.SelectedItems.Count > 0)
            {
                string mac = list.SelectedItems[0].SubItems[1].Text;
                manage?.Connect(mac);
            }
        }

        private bool CheckbluetoothState()
        {
            int status = BluetoothManage.GetBluetoothStatus();
            if (status == 1)
            {
                MessageBox.Show("蓝牙未开启,请打开蓝牙");
                state.Text = "状态:蓝牙未开启";
            }
            else if (status == 2)
            {
                MessageBox.Show("此设备不支持蓝牙,请更换设备");
                state.Text = "状态:不支持蓝牙";
            }else if (status == 0 && state.Text.Equals(""))
            {
                state.Text = "状态:等待连接";
            }
            return status == 0;
        }

        private void SetDevices(string name, string mac)
        {
            this.bluetoothListView.BeginUpdate();//数据更新，UI线程暂时挂起，直到EndUpdate绘制控件，可以有效避免闪烁并大大提高加载速度
            if(!hashMap.ContainsKey(mac)) hashMap.Add(mac, name);
            ListViewItem item = new ListViewItem();
            item.ImageIndex = bluetoothMode;//通过与imageList绑定，显示imageList中第i项图标
            if (name.Length > 21) name = name.Substring(0, 18)+"...";
            item.Text = name;
            item.SubItems.Add(mac);
            this.bluetoothListView.Items.Add(item);
            this.bluetoothListView.EndUpdate();//结束数据处理，UI界面一次性绘制
        }

        private void SetDeviceData(byte[] bytes)
        {
            receiveNum += bytes.Length;
            receptionNumber.Text = ""+receiveNum;
            receiveTextBox.AppendText(Utility.GetByteArrayToString(bytes, checkHexShowBox.Checked));
            if (checkClsBox.Checked && receiveTextBox.Text.Length >= 100000) receiveTextBox.Text = "";
        }

        private void SetLogData(string log)
        {
            logTextBox.AppendText(log + "\r\n");
        }

        private void SetSendNum(int num)
        {
            sendNum += num;
            sendNumber.Text = "" + sendNum;
        }

        private void SetBluetoothState(BluetoothState state,string msg)
        {
            string str = "状态:";
            Trace.WriteLine("回调状态值: " + state);
            
            switch (state)
            {
                case BluetoothState.Scanned:
                    str += "扫描结束";
                    break;
                case BluetoothState.Scanning:
                    str += "扫描中";
                    mTimer.startScanTimer(scanDynamicChange);
                    break;
                case BluetoothState.Connecting:
                    str += "连接中";
                    mTimer.startConnectTimer(connectingDynamicChange);
                    break;
                case BluetoothState.Connected:
                    str += "连接成功";
                    modeSelection.Enabled = false;
                    ShowConnectBluetooth(msg);
                    break;
                case BluetoothState.Disconnecting:
                    str += "正在断开";
                    break;
                case BluetoothState.Disconnected:
                    str += "断开连接";
                    if (Utility.IsWindows8_OrHigher()) modeSelection.Enabled = true;
                    break;
                case BluetoothState.Error:
                    str += "出现错误";
                    if (Utility.IsWindows8_OrHigher()) modeSelection.Enabled = true;
                    if (msg != null && !msg.Equals("") && isShowBox)
                    {
                        isShowBox = false;
                        this.state.Text = str;
                        MessageBox.Show(msg);
                        isShowBox = true;
                    }
                    break;
            }
            if (this.state.Text.Contains("状态:连接中") && str.Equals("状态:扫描结束")) return;
            if (!str.Contains("状态:连接中")) mTimer?.stopConnectTimer();
            if (!str.Contains("状态:扫描中")) mTimer?.stopScanTimer();
            this.state.Text = str;
        }

        private void ShowConnectBluetooth(string mac)
        {
            if (mac == null || mac.Equals("")) return;
            bluetoothListView.Items.Clear();
            SetDevices(hashMap[mac], mac);
        }

        private byte[] GetSendBoxByte()
        {
            string data = SendTextBox.Text;
            if (checkSendNewLineBox.Checked) data += "\r\n";
            return Utility.GetStringToByteArray(data, checkHexSendBox.Checked);
        }


        private void BluetoothCallback(MsgTypes type, string str, object data = null)
        {
            //Trace.WriteLine("type: " + type + "  str: " + str);
            switch (type)
            {
                case MsgTypes.BluetoothData://蓝牙收到的数据, data格式byte[]
                    SetDeviceData((byte[])data);
                    break;

                case MsgTypes.NotifyTxt://日志返回
                    SetLogData(str);
                    break;

                case MsgTypes.BluetoothDevice://扫描到的蓝牙数据，str为name data为mac，格式为string
                    SetDevices(str, (string)data);
                    break;

                case MsgTypes.BluetoothStatus://蓝牙状态 data格式是BluetoothInfo.BluetoothState
                    SetBluetoothState((BluetoothState)data,str);
                    break;

                case MsgTypes.SendNum://成功发送的数据 data格式是int
                    SetSendNum((int)data);
                    break;
            }
        }


        private void connectingDynamicChange(object sender, EventArgs e)
        {
            if (!state.Text.Contains("状态:连接中"))
            {
                SetLogData("connectingDynamicChange 出现问题");
                mTimer.stopConnectTimer();
                return;
            }
            string connecting;
            switch (state.Text)
            {
                case "状态:连接中":
                    connecting = "状态:连接中*";
                    break;
                case "状态:连接中*":
                    connecting = "状态:连接中**";
                    break;
                default:
                    connecting = "状态:连接中";
                    break;
            }

            state.Text = connecting;
        }

        private void scanDynamicChange(object sender, EventArgs e)
        {
            if (!state.Text.Contains("状态:扫描中"))
            {
                SetLogData("scanDynamicChange 出现问题");
                mTimer.stopScanTimer();
                return;
            }
            string scan;
            switch (state.Text)
            {
                case "状态:扫描中":
                    scan = "状态:扫描中*";
                    break;
                case "状态:扫描中*":
                    scan = "状态:扫描中**";
                    break;
                default:
                    scan = "状态:扫描中";
                    break;
            }

            state.Text = scan;
        }

        private void scan_Click(object sender, EventArgs e)
        {
            if (!CheckbluetoothState()) return;
            if (manage == null) InitBleBluetooth(true);
            if (manage.Scan())
            {
                hashMap.Clear();
                bluetoothListView.Items.Clear();
            }
        }

        private void disconnection_Click(object sender, EventArgs e)
        {
            mTimer?.stop();
            manage?.Dispose();
            this.bluetoothListView.Items.Clear();
            foreach (KeyValuePair<string, string> kvp in hashMap)
            {
                SetDevices(kvp.Value, kvp.Key);
            }
        }

        private void send_Click(object sender, EventArgs e)
        {
            manage?.Send(GetSendBoxByte());
        }

        private void clearLog_Click(object sender, EventArgs e)
        {
            logTextBox.Text = "";
        }

        private void BluetoothModeSelection(object sender, EventArgs e)
        {
            //蓝牙模式选择
            bluetoothMode = modeSelection.Text.ToString().Equals("2.0蓝牙") ? 0 : 1;
            manage?.BluetoothMode(bluetoothMode);
            hashMap?.Clear();
            bluetoothListView.Items.Clear();
        }

        private void onClickScaleListener(object sender, EventArgs e)
        {
            if (!checkHexSendBox.Checked)
            {
                string data = Utility.getHexToString(SendTextBox.Text);
                SendTextBox.Text = data;
            }
            else
            {
                string data = Utility.getHexString(SendTextBox.Text);
                SendTextBox.Text = data;
            }
        }

        private void SendBoxListener(object sender, KeyPressEventArgs e)
        {
            if (!checkHexSendBox.Checked || (int)e.KeyChar == 8 || (int)e.KeyChar == 32) return;

            bool isNumber = (int)e.KeyChar >= 48 && (int)e.KeyChar <= 57;
            bool isHexLatter = ((int)e.KeyChar >= 65 && (int)e.KeyChar <= 70) ||
                ((int)e.KeyChar >= 97 && (int)e.KeyChar <= 102);
            if (!(isNumber || isHexLatter))
            {
                e.Handled = true;
                return;
            }

            if ((int)e.KeyChar >= 97) e.KeyChar = (char)((int)e.KeyChar - 32);
        }

        private void LoopSendTimeListener(object sender, KeyPressEventArgs e)
        {
            if (((int)e.KeyChar < 48 && (int)e.KeyChar != 8) || ((int)e.KeyChar > 57))
            {
                e.Handled = true;
            }
        }

        private void LoopSendDataCheckChangeListener(object sender, EventArgs e)
        {
            
            if (checkLoopSendDataCheckBox.Checked)
            {
                if (manage == null || !manage.IsConnected())
                {
                    checkLoopSendDataCheckBox.Checked = false;
                    MessageBox.Show("请连接上蓝牙再选择连续发送");
                }
                else
                {
                    loopSendTime.Enabled = false;
                    send.Enabled = false;
                    int time = int.Parse(loopSendTime.Text);
                    mTimer.start(time, new EventHandler(send_Click));
                }
            }
            else
            {
                mTimer?.stop();
                loopSendTime.Enabled = true;
                send.Enabled = true;
            }
        }

        private void clearReceive_Click(object sender, EventArgs e)
        {
            //清除接收
            receiveTextBox.Text = "";
            receptionNumber.Text = "0";
            sendNumber.Text = "0";
            receiveNum = 0;
            sendNum = 0;

        }

        private void clearSend_Click(object sender, EventArgs e)
        {
            //清空发送
            SendTextBox.Text = "";
        }

        private void startWeb(object sender, LinkLabelLinkClickedEventArgs e)
        {
            System.Diagnostics.Process.Start("http://www.hc01.com/home");
        }

        private void startLog(object sender, EventArgs e)
        {
            checkLog++;
            if (checkLog == 3)
            {
                //打开日志
                receiveTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
                SendTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
                groupLog.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
                groupBluetoothHandle.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left)));
                groupDataHandle.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left)));
                groupSendHandle.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
                groupOther.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
                logTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
                clearLog.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
                MessageBox.Show("已显示运行日志");
            }
        }

        /// <summary>
        /// 打开小车控制界面
        /// </summary>
        private void OpenRobotControlForm()
        {
            try
            {
                RobotControlForm robotControlForm = new RobotControlForm(this);
                robotControlForm.Show();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"打开小车控制界面失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OpenMazeExplorationForm()
        {
            try
            {
                if (robotController == null)
                {
                    MessageBox.Show("请先连接蓝牙设备！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                if (mazeExplorationForm == null || mazeExplorationForm.IsDisposed)
                {
                    mazeExplorationForm = new MazeExplorationForm(this);
                }
                
                mazeExplorationForm.Show();
                mazeExplorationForm.BringToFront();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"打开迷宫探索界面失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OpenAdvancedMazeExplorationForm()
        {
            try
            {
                if (robotController == null)
                {
                    MessageBox.Show("请先连接蓝牙设备！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                if (advancedMazeExplorationForm == null || advancedMazeExplorationForm.IsDisposed)
                {
                    advancedMazeExplorationForm = new AdvancedMazeExplorationForm(this);
                }
                
                advancedMazeExplorationForm.Show();
                advancedMazeExplorationForm.BringToFront();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"打开高级迷宫探索界面失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        /// <summary>
        /// 小车控制按钮点击事件
        /// </summary>
        private void btnRobotControl_Click(object sender, EventArgs e)
        {
            OpenRobotControlForm();
        }

        private void btnMazeExploration_Click(object sender, EventArgs e)
        {
            OpenMazeExplorationForm();
        }

        private void btnAdvancedMazeExploration_Click(object sender, EventArgs e)
        {
            OpenAdvancedMazeExplorationForm();
        }

        #region 小车控制事件处理

        /// <summary>
        /// 处理传感器数据
        /// </summary>
        private void OnSensorDataReceived(RobotController.SensorData sensorData)
        {
            // 在UI线程中更新传感器数据显示
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<RobotController.SensorData>(OnSensorDataReceived), sensorData);
                return;
            }

            // 更新传感器数据显示（可以在这里添加传感器数据的UI更新）
            // 例如：更新文本框显示传感器数据
            string sensorInfo = $"电机A: {sensorData.MotorA_RPM:F1} RPM, 电机B: {sensorData.MotorB_RPM:F1} RPM\n" +
                              $"加速度: X:{sensorData.AccelX:F2} Y:{sensorData.AccelY:F2} Z:{sensorData.AccelZ:F2}\n" +
                              $"陀螺仪: X:{sensorData.GyroX:F2} Y:{sensorData.GyroY:F2} Z:{sensorData.GyroZ:F2}\n" +
                              $"LiDAR: 角度:{sensorData.LidarAngle:F2}° 距离:{sensorData.LidarDistance:F2}mm 质量:{sensorData.LidarQuality}";
            
            // 可以在这里添加传感器数据的显示逻辑
        }

        /// <summary>
        /// 处理小车状态变化
        /// </summary>
        private void OnRobotStatusChanged(string status)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<string>(OnRobotStatusChanged), status);
                return;
            }

            // 更新状态显示
            SetLogData($"[小车] {status}");
        }

        /// <summary>
        /// 处理小车连接状态变化
        /// </summary>
        private void OnRobotConnectionChanged(bool isConnected)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<bool>(OnRobotConnectionChanged), isConnected);
                return;
            }

            // 更新连接状态显示
            if (isConnected)
            {
                SetLogData("[小车] 已连接到小车");
            }
            else
            {
                SetLogData("[小车] 与小车断开连接");
            }
        }

        #endregion

        #region 小车控制方法

        /// <summary>
        /// 停止小车
        /// </summary>
        public void StopRobot()
        {
            robotController?.Stop();
        }

        /// <summary>
        /// 前进
        /// </summary>
        public void MoveForward()
        {
            robotController?.MoveForward();
        }

        /// <summary>
        /// 后退
        /// </summary>
        public void MoveBackward()
        {
            robotController?.MoveBackward();
        }

        /// <summary>
        /// 左转
        /// </summary>
        public void TurnLeft()
        {
            robotController?.TurnLeft();
        }

        /// <summary>
        /// 右转
        /// </summary>
        public void TurnRight()
        {
            robotController?.TurnRight();
        }

        /// <summary>
        /// 掉头
        /// </summary>
        public void UTurn()
        {
            robotController?.UTurn();
        }

        #endregion

        #region 自动控制方法

        /// <summary>
        /// 打开自动控制界面
        /// </summary>
        private void OpenAutoControlForm()
        {
            try
            {
                if (robotController == null)
                {
                    MessageBox.Show("请先连接蓝牙设备！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                if (autoControlForm == null || autoControlForm.IsDisposed)
                {
                    autoControlForm = new AutoControlForm(this);
                }
                
                autoControlForm.Show();
                autoControlForm.BringToFront();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"打开自动控制界面失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        /// <summary>
        /// 自动控制按钮点击事件
        /// </summary>
        private void btnAutoControl_Click(object sender, EventArgs e)
        {
            OpenAutoControlForm();
        }

        /// <summary>
        /// 打开SLAM自动控制界面
        /// </summary>
        private void OpenSLAMAutoControlForm()
        {
            try
            {
                if (robotController == null)
                {
                    MessageBox.Show("请先连接蓝牙设备！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                if (slamAutoControlForm == null || slamAutoControlForm.IsDisposed)
                {
                    slamAutoControlForm = new SLAMAutoControlForm(this);
                }
                
                slamAutoControlForm.Show();
                slamAutoControlForm.BringToFront();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"打开SLAM自动控制界面失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        /// <summary>
        /// SLAM自动控制按钮点击事件
        /// </summary>
        private void btnSLAMAutoControl_Click(object sender, EventArgs e)
        {
            OpenSLAMAutoControlForm();
        }

        /// <summary>
        /// 打开集成SLAM控制界面
        /// </summary>
        private void OpenIntegratedSLAMForm()
        {
            try
            {
                if (robotController == null)
                {
                    MessageBox.Show("请先连接蓝牙设备！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                if (integratedSLAMForm == null || integratedSLAMForm.IsDisposed)
                {
                    integratedSLAMForm = new IntegratedSLAMForm(this);
                }
                
                integratedSLAMForm.Show();
                integratedSLAMForm.BringToFront();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"打开集成SLAM控制界面失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        /// <summary>
        /// 集成SLAM控制按钮点击事件
        /// </summary>
        private void btnIntegratedSLAM_Click(object sender, EventArgs e)
        {
            OpenIntegratedSLAMForm();
        }

        private void btnHardwarePythonSLAM_Click(object sender, EventArgs e)
        {
            OpenHardwarePythonSLAMForm();
        }

        /// <summary>
        /// 打开硬件Python SLAM控制界面
        /// </summary>
        private void OpenHardwarePythonSLAMForm()
        {
            try
            {
                if (robotController == null)
                {
                    MessageBox.Show("请先连接蓝牙设备！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                var hardwarePythonSLAMForm = new HardwarePythonSLAMForm(robotController);
                hardwarePythonSLAMForm.Show();
                hardwarePythonSLAMForm.BringToFront();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"打开硬件Python SLAM控制界面失败: {ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        #endregion
    }
}
