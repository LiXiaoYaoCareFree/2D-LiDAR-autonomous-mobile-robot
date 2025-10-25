
namespace BluetoothApp
{
    partial class BluetoothWindow
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要修改
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(BluetoothWindow));
            this.bluetoothListView = new System.Windows.Forms.ListView();
            this.label1 = new System.Windows.Forms.Label();
            this.state = new System.Windows.Forms.Label();
            this.bluetoothImageList = new System.Windows.Forms.ImageList(this.components);
            this.receiveTextBox = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.scan = new System.Windows.Forms.Button();
            this.disconnection = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.SendTextBox = new System.Windows.Forms.TextBox();
            this.send = new System.Windows.Forms.Button();
            this.logTextBox = new System.Windows.Forms.TextBox();
            this.clearLog = new System.Windows.Forms.Button();
            this.groupBluetoothHandle = new System.Windows.Forms.GroupBox();
            this.label9 = new System.Windows.Forms.Label();
            this.modeSelection = new System.Windows.Forms.ComboBox();
            this.groupSendHandle = new System.Windows.Forms.GroupBox();
            this.checkLoopSendDataCheckBox = new System.Windows.Forms.CheckBox();
            this.loopSendTime = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.clearReceive = new System.Windows.Forms.Button();
            this.clearSend = new System.Windows.Forms.Button();
            this.label5 = new System.Windows.Forms.Label();
            this.linkLabel1 = new System.Windows.Forms.LinkLabel();
            this.label6 = new System.Windows.Forms.Label();
            this.receptionNumber = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.sendNumber = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.groupDataHandle = new System.Windows.Forms.GroupBox();
            this.checkClsBox = new System.Windows.Forms.CheckBox();
            this.checkHexSendBox = new System.Windows.Forms.CheckBox();
            this.checkHexShowBox = new System.Windows.Forms.CheckBox();
            this.checkSendNewLineBox = new System.Windows.Forms.CheckBox();
            this.groupLog = new System.Windows.Forms.GroupBox();
            this.groupOther = new System.Windows.Forms.GroupBox();
            this.btnRobotControl = new System.Windows.Forms.Button();
            this.btnMazeExploration = new System.Windows.Forms.Button();
            this.btnAdvancedMazeExploration = new System.Windows.Forms.Button();
            this.btnAutoControl = new System.Windows.Forms.Button();
            this.btnSLAMAutoControl = new System.Windows.Forms.Button();
            this.btnIntegratedSLAM = new System.Windows.Forms.Button();
            this.groupBluetoothHandle.SuspendLayout();
            this.groupSendHandle.SuspendLayout();
            this.groupDataHandle.SuspendLayout();
            this.groupLog.SuspendLayout();
            this.groupOther.SuspendLayout();
            this.SuspendLayout();
            // 
            // bluetoothListView
            // 
            this.bluetoothListView.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.bluetoothListView.Font = new System.Drawing.Font("宋体", 11F);
            this.bluetoothListView.HideSelection = false;
            this.bluetoothListView.Location = new System.Drawing.Point(12, 28);
            this.bluetoothListView.Name = "bluetoothListView";
            this.bluetoothListView.Size = new System.Drawing.Size(221, 399);
            this.bluetoothListView.TabIndex = 0;
            this.bluetoothListView.TileSize = new System.Drawing.Size(228, 38);
            this.bluetoothListView.UseCompatibleStateImageBehavior = false;
            this.bluetoothListView.View = System.Windows.Forms.View.Tile;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(12, 13);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(53, 12);
            this.label1.TabIndex = 1;
            this.label1.Text = "蓝牙列表";
            // 
            // state
            // 
            this.state.AutoSize = true;
            this.state.Location = new System.Drawing.Point(150, 13);
            this.state.Name = "state";
            this.state.Size = new System.Drawing.Size(83, 12);
            this.state.TabIndex = 2;
            this.state.Text = "状态:等待连接";
            // 
            // bluetoothImageList
            // 
            this.bluetoothImageList.ColorDepth = System.Windows.Forms.ColorDepth.Depth8Bit;
            this.bluetoothImageList.ImageSize = new System.Drawing.Size(16, 16);
            this.bluetoothImageList.TransparentColor = System.Drawing.Color.Transparent;
            // 
            // receiveTextBox
            // 
            this.receiveTextBox.AllowDrop = true;
            this.receiveTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.receiveTextBox.BackColor = System.Drawing.Color.White;
            this.receiveTextBox.Font = new System.Drawing.Font("宋体", 9.5F);
            this.receiveTextBox.Location = new System.Drawing.Point(250, 28);
            this.receiveTextBox.Multiline = true;
            this.receiveTextBox.Name = "receiveTextBox";
            this.receiveTextBox.ReadOnly = true;
            this.receiveTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.receiveTextBox.Size = new System.Drawing.Size(355, 269);
            this.receiveTextBox.TabIndex = 3;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(248, 13);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(53, 12);
            this.label2.TabIndex = 4;
            this.label2.Text = "数据接收";
            // 
            // scan
            // 
            this.scan.Location = new System.Drawing.Point(98, 49);
            this.scan.Name = "scan";
            this.scan.Size = new System.Drawing.Size(75, 32);
            this.scan.TabIndex = 5;
            this.scan.Text = "扫描蓝牙";
            this.scan.UseVisualStyleBackColor = true;
            this.scan.Click += new System.EventHandler(this.scan_Click);
            // 
            // disconnection
            // 
            this.disconnection.Location = new System.Drawing.Point(9, 49);
            this.disconnection.Name = "disconnection";
            this.disconnection.Size = new System.Drawing.Size(75, 32);
            this.disconnection.TabIndex = 6;
            this.disconnection.Text = "断开蓝牙";
            this.disconnection.UseVisualStyleBackColor = true;
            this.disconnection.Click += new System.EventHandler(this.disconnection_Click);
            // 
            // label3
            // 
            this.label3.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(248, 300);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(53, 12);
            this.label3.TabIndex = 7;
            this.label3.Text = "数据发送";
            // 
            // SendTextBox
            // 
            this.SendTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.SendTextBox.Font = new System.Drawing.Font("宋体", 9.5F);
            this.SendTextBox.Location = new System.Drawing.Point(250, 315);
            this.SendTextBox.Multiline = true;
            this.SendTextBox.Name = "SendTextBox";
            this.SendTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.SendTextBox.Size = new System.Drawing.Size(355, 112);
            this.SendTextBox.TabIndex = 8;
            this.SendTextBox.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.SendBoxListener);
            // 
            // send
            // 
            this.send.Location = new System.Drawing.Point(9, 126);
            this.send.Name = "send";
            this.send.Size = new System.Drawing.Size(161, 30);
            this.send.TabIndex = 9;
            this.send.Text = "发送数据";
            this.send.UseVisualStyleBackColor = true;
            this.send.Click += new System.EventHandler(this.send_Click);
            // 
            // logTextBox
            // 
            this.logTextBox.BackColor = System.Drawing.Color.White;
            this.logTextBox.Location = new System.Drawing.Point(7, 15);
            this.logTextBox.Multiline = true;
            this.logTextBox.Name = "logTextBox";
            this.logTextBox.ReadOnly = true;
            this.logTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.logTextBox.Size = new System.Drawing.Size(263, 370);
            this.logTextBox.TabIndex = 10;
            // 
            // clearLog
            // 
            this.clearLog.Location = new System.Drawing.Point(185, 391);
            this.clearLog.Name = "clearLog";
            this.clearLog.Size = new System.Drawing.Size(75, 23);
            this.clearLog.TabIndex = 12;
            this.clearLog.Text = "清空日志";
            this.clearLog.UseVisualStyleBackColor = true;
            this.clearLog.Click += new System.EventHandler(this.clearLog_Click);
            // 
            // groupBluetoothHandle
            // 
            this.groupBluetoothHandle.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.groupBluetoothHandle.Controls.Add(this.label9);
            this.groupBluetoothHandle.Controls.Add(this.modeSelection);
            this.groupBluetoothHandle.Controls.Add(this.scan);
            this.groupBluetoothHandle.Controls.Add(this.disconnection);
            this.groupBluetoothHandle.Location = new System.Drawing.Point(613, 13);
            this.groupBluetoothHandle.Name = "groupBluetoothHandle";
            this.groupBluetoothHandle.Size = new System.Drawing.Size(179, 89);
            this.groupBluetoothHandle.TabIndex = 13;
            this.groupBluetoothHandle.TabStop = false;
            this.groupBluetoothHandle.Text = "蓝牙操作";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("宋体", 9.5F);
            this.label9.Location = new System.Drawing.Point(6, 23);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(85, 13);
            this.label9.TabIndex = 8;
            this.label9.Text = "扫描蓝牙类型";
            // 
            // modeSelection
            // 
            this.modeSelection.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.modeSelection.Font = new System.Drawing.Font("宋体", 9.5F);
            this.modeSelection.FormattingEnabled = true;
            this.modeSelection.ItemHeight = 13;
            this.modeSelection.Items.AddRange(new object[] {
            "2.0蓝牙",
            "ble蓝牙"});
            this.modeSelection.Location = new System.Drawing.Point(98, 20);
            this.modeSelection.Name = "modeSelection";
            this.modeSelection.Size = new System.Drawing.Size(75, 21);
            this.modeSelection.TabIndex = 7;
            this.modeSelection.SelectedIndexChanged += new System.EventHandler(this.BluetoothModeSelection);
            // 
            // groupSendHandle
            // 
            this.groupSendHandle.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.groupSendHandle.Controls.Add(this.checkLoopSendDataCheckBox);
            this.groupSendHandle.Controls.Add(this.loopSendTime);
            this.groupSendHandle.Controls.Add(this.label10);
            this.groupSendHandle.Controls.Add(this.clearReceive);
            this.groupSendHandle.Controls.Add(this.clearSend);
            this.groupSendHandle.Controls.Add(this.send);
            this.groupSendHandle.Location = new System.Drawing.Point(613, 205);
            this.groupSendHandle.Name = "groupSendHandle";
            this.groupSendHandle.Size = new System.Drawing.Size(180, 162);
            this.groupSendHandle.TabIndex = 14;
            this.groupSendHandle.TabStop = false;
            this.groupSendHandle.Text = "发送操作";
            // 
            // checkLoopSendDataCheckBox
            // 
            this.checkLoopSendDataCheckBox.AutoSize = true;
            this.checkLoopSendDataCheckBox.Location = new System.Drawing.Point(89, 21);
            this.checkLoopSendDataCheckBox.Name = "checkLoopSendDataCheckBox";
            this.checkLoopSendDataCheckBox.Size = new System.Drawing.Size(72, 16);
            this.checkLoopSendDataCheckBox.TabIndex = 14;
            this.checkLoopSendDataCheckBox.Text = "循环发送";
            this.checkLoopSendDataCheckBox.UseVisualStyleBackColor = true;
            this.checkLoopSendDataCheckBox.CheckedChanged += new System.EventHandler(this.LoopSendDataCheckChangeListener);
            // 
            // loopSendTime
            // 
            this.loopSendTime.Font = new System.Drawing.Font("宋体", 9.5F);
            this.loopSendTime.Location = new System.Drawing.Point(40, 18);
            this.loopSendTime.Name = "loopSendTime";
            this.loopSendTime.Size = new System.Drawing.Size(43, 22);
            this.loopSendTime.TabIndex = 13;
            this.loopSendTime.Text = "500";
            this.loopSendTime.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.LoopSendTimeListener);
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(5, 21);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(35, 12);
            this.label10.TabIndex = 12;
            this.label10.Text = "周期:";
            // 
            // clearReceive
            // 
            this.clearReceive.Location = new System.Drawing.Point(9, 54);
            this.clearReceive.Name = "clearReceive";
            this.clearReceive.Size = new System.Drawing.Size(161, 30);
            this.clearReceive.TabIndex = 11;
            this.clearReceive.Text = "清除接收";
            this.clearReceive.UseVisualStyleBackColor = true;
            this.clearReceive.Click += new System.EventHandler(this.clearReceive_Click);
            // 
            // clearSend
            // 
            this.clearSend.Location = new System.Drawing.Point(9, 90);
            this.clearSend.Name = "clearSend";
            this.clearSend.Size = new System.Drawing.Size(161, 30);
            this.clearSend.TabIndex = 10;
            this.clearSend.Text = "清空发送";
            this.clearSend.UseVisualStyleBackColor = true;
            this.clearSend.Click += new System.EventHandler(this.clearSend_Click);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(1, 17);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(65, 12);
            this.label5.TabIndex = 15;
            this.label5.Text = "汇承官网：";
            // 
            // linkLabel1
            // 
            this.linkLabel1.AutoSize = true;
            this.linkLabel1.Font = new System.Drawing.Font("宋体", 11F);
            this.linkLabel1.Location = new System.Drawing.Point(72, 14);
            this.linkLabel1.Name = "linkLabel1";
            this.linkLabel1.Size = new System.Drawing.Size(103, 15);
            this.linkLabel1.TabIndex = 16;
            this.linkLabel1.TabStop = true;
            this.linkLabel1.Text = "www.hc01.com";
            this.linkLabel1.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.startWeb);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(1, 39);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(17, 12);
            this.label6.TabIndex = 17;
            this.label6.Text = "R:";
            // 
            // receptionNumber
            // 
            this.receptionNumber.AutoSize = true;
            this.receptionNumber.Location = new System.Drawing.Point(17, 39);
            this.receptionNumber.Name = "receptionNumber";
            this.receptionNumber.Size = new System.Drawing.Size(11, 12);
            this.receptionNumber.TabIndex = 18;
            this.receptionNumber.Text = "0";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(70, 39);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(17, 12);
            this.label7.TabIndex = 19;
            this.label7.Text = "S:";
            // 
            // sendNumber
            // 
            this.sendNumber.AutoSize = true;
            this.sendNumber.Location = new System.Drawing.Point(85, 39);
            this.sendNumber.Name = "sendNumber";
            this.sendNumber.Size = new System.Drawing.Size(11, 12);
            this.sendNumber.TabIndex = 20;
            this.sendNumber.Text = "0";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(131, 39);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(47, 12);
            this.label8.TabIndex = 21;
            this.label8.Text = "V:1.1.0";
            this.label8.Click += new System.EventHandler(this.startLog);
            // 
            // groupDataHandle
            // 
            this.groupDataHandle.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.groupDataHandle.Controls.Add(this.checkClsBox);
            this.groupDataHandle.Controls.Add(this.checkHexSendBox);
            this.groupDataHandle.Controls.Add(this.checkHexShowBox);
            this.groupDataHandle.Controls.Add(this.checkSendNewLineBox);
            this.groupDataHandle.Location = new System.Drawing.Point(612, 107);
            this.groupDataHandle.Name = "groupDataHandle";
            this.groupDataHandle.Size = new System.Drawing.Size(181, 90);
            this.groupDataHandle.TabIndex = 22;
            this.groupDataHandle.TabStop = false;
            this.groupDataHandle.Text = "数据处理";
            // 
            // checkClsBox
            // 
            this.checkClsBox.AutoSize = true;
            this.checkClsBox.Location = new System.Drawing.Point(10, 42);
            this.checkClsBox.Name = "checkClsBox";
            this.checkClsBox.Size = new System.Drawing.Size(96, 16);
            this.checkClsBox.TabIndex = 3;
            this.checkClsBox.Text = "接收200k清屏";
            this.checkClsBox.UseVisualStyleBackColor = true;
            // 
            // checkHexSendBox
            // 
            this.checkHexSendBox.AutoSize = true;
            this.checkHexSendBox.Location = new System.Drawing.Point(94, 64);
            this.checkHexSendBox.Name = "checkHexSendBox";
            this.checkHexSendBox.Size = new System.Drawing.Size(84, 16);
            this.checkHexSendBox.TabIndex = 2;
            this.checkHexSendBox.Text = "16进制发送";
            this.checkHexSendBox.UseVisualStyleBackColor = true;
            this.checkHexSendBox.CheckedChanged += new System.EventHandler(this.onClickScaleListener);
            // 
            // checkHexShowBox
            // 
            this.checkHexShowBox.AutoSize = true;
            this.checkHexShowBox.Location = new System.Drawing.Point(10, 64);
            this.checkHexShowBox.Name = "checkHexShowBox";
            this.checkHexShowBox.Size = new System.Drawing.Size(84, 16);
            this.checkHexShowBox.TabIndex = 1;
            this.checkHexShowBox.Text = "16进制显示";
            this.checkHexShowBox.UseVisualStyleBackColor = true;
            // 
            // checkSendNewLineBox
            // 
            this.checkSendNewLineBox.AutoSize = true;
            this.checkSendNewLineBox.Location = new System.Drawing.Point(10, 20);
            this.checkSendNewLineBox.Name = "checkSendNewLineBox";
            this.checkSendNewLineBox.Size = new System.Drawing.Size(72, 16);
            this.checkSendNewLineBox.TabIndex = 0;
            this.checkSendNewLineBox.Text = "发送新行";
            this.checkSendNewLineBox.UseVisualStyleBackColor = true;
            // 
            // groupLog
            // 
            this.groupLog.Controls.Add(this.logTextBox);
            this.groupLog.Controls.Add(this.clearLog);
            this.groupLog.Location = new System.Drawing.Point(823, 14);
            this.groupLog.Name = "groupLog";
            this.groupLog.Size = new System.Drawing.Size(276, 414);
            this.groupLog.TabIndex = 23;
            this.groupLog.TabStop = false;
            this.groupLog.Text = "运行日志";
            // 
            // groupOther
            // 
            this.groupOther.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.groupOther.Controls.Add(this.label5);
            this.groupOther.Controls.Add(this.linkLabel1);
            this.groupOther.Controls.Add(this.receptionNumber);
            this.groupOther.Controls.Add(this.label8);
            this.groupOther.Controls.Add(this.label7);
            this.groupOther.Controls.Add(this.sendNumber);
            this.groupOther.Controls.Add(this.label6);
            this.groupOther.Location = new System.Drawing.Point(613, 368);
            this.groupOther.Name = "groupOther";
            this.groupOther.Size = new System.Drawing.Size(182, 60);
            this.groupOther.TabIndex = 24;
            this.groupOther.TabStop = false;
            // 
            // btnRobotControl
            // 
            this.btnRobotControl.BackColor = System.Drawing.Color.LightBlue;
            this.btnRobotControl.Font = new System.Drawing.Font("微软雅黑", 10F, System.Drawing.FontStyle.Bold);
            this.btnRobotControl.Location = new System.Drawing.Point(613, 14);
            this.btnRobotControl.Name = "btnRobotControl";
            this.btnRobotControl.Size = new System.Drawing.Size(120, 40);
            this.btnRobotControl.TabIndex = 25;
            this.btnRobotControl.Text = "小车控制";
            this.btnRobotControl.UseVisualStyleBackColor = false;
            this.btnRobotControl.Click += new System.EventHandler(this.btnRobotControl_Click);
            // 
            // btnMazeExploration
            // 
            this.btnMazeExploration.BackColor = System.Drawing.Color.LightGreen;
            this.btnMazeExploration.Font = new System.Drawing.Font("微软雅黑", 10F, System.Drawing.FontStyle.Bold);
            this.btnMazeExploration.Location = new System.Drawing.Point(613, 60);
            this.btnMazeExploration.Name = "btnMazeExploration";
            this.btnMazeExploration.Size = new System.Drawing.Size(120, 40);
            this.btnMazeExploration.TabIndex = 26;
            this.btnMazeExploration.Text = "迷宫探索";
            this.btnMazeExploration.UseVisualStyleBackColor = false;
            this.btnMazeExploration.Click += new System.EventHandler(this.btnMazeExploration_Click);
            // 
            // btnAdvancedMazeExploration
            // 
            this.btnAdvancedMazeExploration.BackColor = System.Drawing.Color.Orange;
            this.btnAdvancedMazeExploration.Font = new System.Drawing.Font("微软雅黑", 10F, System.Drawing.FontStyle.Bold);
            this.btnAdvancedMazeExploration.Location = new System.Drawing.Point(613, 106);
            this.btnAdvancedMazeExploration.Name = "btnAdvancedMazeExploration";
            this.btnAdvancedMazeExploration.Size = new System.Drawing.Size(120, 40);
            this.btnAdvancedMazeExploration.TabIndex = 27;
            this.btnAdvancedMazeExploration.Text = "高级探索";
            this.btnAdvancedMazeExploration.UseVisualStyleBackColor = false;
            this.btnAdvancedMazeExploration.Click += new System.EventHandler(this.btnAdvancedMazeExploration_Click);
            // 
            // btnAutoControl
            // 
            this.btnAutoControl.BackColor = System.Drawing.Color.MediumPurple;
            this.btnAutoControl.Font = new System.Drawing.Font("微软雅黑", 10F, System.Drawing.FontStyle.Bold);
            this.btnAutoControl.Location = new System.Drawing.Point(613, 156);
            this.btnAutoControl.Name = "btnAutoControl";
            this.btnAutoControl.Size = new System.Drawing.Size(120, 40);
            this.btnAutoControl.TabIndex = 28;
            this.btnAutoControl.Text = "自动控制";
            this.btnAutoControl.UseVisualStyleBackColor = false;
            this.btnAutoControl.Click += new System.EventHandler(this.btnAutoControl_Click);
            // 
            // btnSLAMAutoControl
            // 
            this.btnSLAMAutoControl.BackColor = System.Drawing.Color.DarkMagenta;
            this.btnSLAMAutoControl.Font = new System.Drawing.Font("微软雅黑", 10F, System.Drawing.FontStyle.Bold);
            this.btnSLAMAutoControl.Location = new System.Drawing.Point(613, 206);
            this.btnSLAMAutoControl.Name = "btnSLAMAutoControl";
            this.btnSLAMAutoControl.Size = new System.Drawing.Size(120, 40);
            this.btnSLAMAutoControl.TabIndex = 29;
            this.btnSLAMAutoControl.Text = "SLAM控制";
            this.btnSLAMAutoControl.UseVisualStyleBackColor = false;
            this.btnSLAMAutoControl.Click += new System.EventHandler(this.btnSLAMAutoControl_Click);
            // 
            // btnIntegratedSLAM
            // 
            this.btnIntegratedSLAM.BackColor = System.Drawing.Color.DarkCyan;
            this.btnIntegratedSLAM.Font = new System.Drawing.Font("微软雅黑", 10F, System.Drawing.FontStyle.Bold);
            this.btnIntegratedSLAM.Location = new System.Drawing.Point(613, 256);
            this.btnIntegratedSLAM.Name = "btnIntegratedSLAM";
            this.btnIntegratedSLAM.Size = new System.Drawing.Size(120, 40);
            this.btnIntegratedSLAM.TabIndex = 30;
            this.btnIntegratedSLAM.Text = "集成SLAM";
            this.btnIntegratedSLAM.UseVisualStyleBackColor = false;
            this.btnIntegratedSLAM.Click += new System.EventHandler(this.btnIntegratedSLAM_Click);
            // 
            // BluetoothWindow
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.Control;
            this.ClientSize = new System.Drawing.Size(806, 440);
            this.Controls.Add(this.btnIntegratedSLAM);
            this.Controls.Add(this.btnSLAMAutoControl);
            this.Controls.Add(this.btnAutoControl);
            this.Controls.Add(this.btnAdvancedMazeExploration);
            this.Controls.Add(this.btnMazeExploration);
            this.Controls.Add(this.btnRobotControl);
            this.Controls.Add(this.groupOther);
            this.Controls.Add(this.groupLog);
            this.Controls.Add(this.groupDataHandle);
            this.Controls.Add(this.groupSendHandle);
            this.Controls.Add(this.groupBluetoothHandle);
            this.Controls.Add(this.SendTextBox);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.receiveTextBox);
            this.Controls.Add(this.state);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.bluetoothListView);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "BluetoothWindow";
            this.Text = "HC-PC";
            this.groupBluetoothHandle.ResumeLayout(false);
            this.groupBluetoothHandle.PerformLayout();
            this.groupSendHandle.ResumeLayout(false);
            this.groupSendHandle.PerformLayout();
            this.groupDataHandle.ResumeLayout(false);
            this.groupDataHandle.PerformLayout();
            this.groupLog.ResumeLayout(false);
            this.groupLog.PerformLayout();
            this.groupOther.ResumeLayout(false);
            this.groupOther.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ListView bluetoothListView;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label state;
        private System.Windows.Forms.ImageList bluetoothImageList;
        private System.Windows.Forms.TextBox receiveTextBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button scan;
        private System.Windows.Forms.Button disconnection;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox SendTextBox;
        private System.Windows.Forms.Button send;
        private System.Windows.Forms.TextBox logTextBox;
        private System.Windows.Forms.Button clearLog;
        private System.Windows.Forms.GroupBox groupBluetoothHandle;
        private System.Windows.Forms.GroupBox groupSendHandle;
        private System.Windows.Forms.Button clearReceive;
        private System.Windows.Forms.Button clearSend;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.LinkLabel linkLabel1;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label receptionNumber;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label sendNumber;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.ComboBox modeSelection;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.GroupBox groupDataHandle;
        private System.Windows.Forms.CheckBox checkClsBox;
        private System.Windows.Forms.CheckBox checkHexSendBox;
        private System.Windows.Forms.CheckBox checkHexShowBox;
        private System.Windows.Forms.CheckBox checkSendNewLineBox;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.CheckBox checkLoopSendDataCheckBox;
        private System.Windows.Forms.TextBox loopSendTime;
        private System.Windows.Forms.GroupBox groupLog;
        private System.Windows.Forms.GroupBox groupOther;
        private System.Windows.Forms.Button btnRobotControl;
        private System.Windows.Forms.Button btnMazeExploration;
        private System.Windows.Forms.Button btnAdvancedMazeExploration;
        private System.Windows.Forms.Button btnAutoControl;
        private System.Windows.Forms.Button btnSLAMAutoControl;
        private System.Windows.Forms.Button btnIntegratedSLAM;
    }
}

