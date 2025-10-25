
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Devices.Enumeration;
using Windows.Foundation;
using Windows.Security.Cryptography;
using static BluetoothApp.BluetoothInfo;

namespace BluetoothApp
{
    public class BleBluetoothInfo:BluetoothInterface
    {

        private BluetoothState bluetoothState = BluetoothState.Scanned;

        private Form form;

        //存储检测的设备MAC。
        public string CurrentDeviceMAC { get; set; }
        //存储检测到的设备。
        public BluetoothLEDevice CurrentDevice { get; set; }
        //存储检测到的主服务。
        public GattDeviceService CurrentService { get; set; }
        //存储检测到的写特征对象。
        public GattCharacteristic CurrentWriteCharacteristic { get; set; }
        //存储检测到的通知特征对象。
        public GattCharacteristic CurrentNotifyCharacteristic { get; set; }

        public string ServiceGuid { get; set; }

        public string ServiceGuid02 { get; set; }

        public string WriteCharacteristicGuid { get; set; }
        public string NotifyCharacteristicGuid { get; set; }


        private const int CHARACTERISTIC_INDEX = 0;
        //特性通知类型通知启用
        private const GattClientCharacteristicConfigurationDescriptorValue CHARACTERISTIC_NOTIFICATION_TYPE = GattClientCharacteristicConfigurationDescriptorValue.Notify;

        //定时器，用于结束蓝牙扫描
        System.Windows.Forms.Timer timer = new System.Windows.Forms.Timer();
        //蓝牙扫描
        private BluetoothLEAdvertisementWatcher Watcher = null;
        // 存储检测到的设备
        private List<BluetoothLEDevice> DeviceList;

        private Boolean asyncLock = false;

        //定义一个事件
        public event eventRun CallbackData;

        //将发送数据分包
        private List<byte[]> sendDataArray = new List<byte[]>();

        private int failNumber = 0;//连接一个蓝牙失败重连次数计数

        private bool isReconnection = true;//重新连接



        public BleBluetoothInfo(Form form,string serviceGuid, string serviceGuid02)
        {
            this.form = form;
            ServiceGuid = serviceGuid;
            ServiceGuid02 = serviceGuid02;
            DeviceList = new List<BluetoothLEDevice>();

            timer.Enabled = true;
            timer.Interval = 10000;//搜索10秒
            timer.Tick += new EventHandler(this.Timer1_Tick);
            timer.Stop();
        }

        /// <summary>
        /// 蓝牙的当前状态
        /// </summary>
        /// <returns></returns>
        public BluetoothState GetBluetoothState()
        {
            return bluetoothState;
        }

        /// <summary>
        /// 搜索蓝牙设备
        /// </summary>
        public void StartBleDeviceWatcher()
        {
            DeviceList.Clear();

            if (Watcher == null) Watcher = new BluetoothLEAdvertisementWatcher();

            Watcher.ScanningMode = BluetoothLEScanningMode.Active;//积极扫描

            // only activate the watcher when we're recieving values >= -80
            // Watcher.SignalStrengthFilter.InRangeThresholdInDBm = -80;

            // stop watching if the value drops below -90 (user walked away)
            //Watcher.SignalStrengthFilter.OutOfRangeThresholdInDBm = -90;

            // register callback for when we see an advertisements
            Watcher.Received += OnAdvertisementReceived;

            // wait 5 seconds to make sure the device is really out of range
            Watcher.SignalStrengthFilter.OutOfRangeTimeout = TimeSpan.FromMilliseconds(5000);
            Watcher.SignalStrengthFilter.SamplingInterval = TimeSpan.FromMilliseconds(2000);

            // starting watching for advertisements
            //设置定时器来限制蓝牙搜索时间

            timer.Start();
            Watcher.Start();

            bluetoothState = BluetoothState.Scanning;//扫描中
            //ValueChanged(MsgTypes.BluetoothStatus,"" ,bluetoothState);
            CallbackStatus(bluetoothState);

            Trace.WriteLine("自动发现设备中..");
        }


        /// <summary>
        /// 按蓝牙的mac连接蓝牙
        /// </summary>
        /// <param name="MAC"></param>
        /// <returns></returns>
        public void SelectDeviceFromIdAsync(string MAC)
        {
            failNumber = 0;//重置失败计数
            CurrentDeviceMAC = MAC;
            CurrentDevice = null;
            isReconnection = true;
            StopBleDeviceWatcher();
            bluetoothState = BluetoothState.Connecting;
            CallbackStatus(bluetoothState);
            BluetoothAdapter.GetDefaultAsync().Completed = (asyncInfo, asyncStatus) =>
            {
                if (asyncStatus == AsyncStatus.Completed)
                {
                    BluetoothAdapter mBluetoothAdapter = asyncInfo.GetResults();
                    byte[] _Bytes1 = BitConverter.GetBytes(mBluetoothAdapter.BluetoothAddress);//ulong转换为byte数组
                    Array.Reverse(_Bytes1);
                    string macAddress = BitConverter.ToString(_Bytes1, 2, 6).Replace('-', ':').ToLower();
                    string Id = "BluetoothLE#BluetoothLE" + macAddress + "-" + MAC;
                    MatchingBluetoothAsync(Id);
                }

            };

        }

        /// <summary>
        /// 发送数据接口
        /// </summary>
        /// <param name="characteristic"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public void Write(byte[] data)
        {
            if (CurrentWriteCharacteristic != null)
            {
                //CurrentWriteCharacteristic.WriteValueAsync(CryptographicBuffer.CreateFromByteArray(data), GattWriteOption.WriteWithResponse);
                SplitArray(data);
                handlerSendData();
            }
            else
            {
                CallbackLog("写入出错，CurrentWriteCharacteristic 为空");
                bluetoothState = BluetoothState.Error;
                CallbackStatus(bluetoothState, "找不到写入特征，请尝试重新连接蓝牙");
            }

        }

        /// <summary>
        /// 主动断开连接
        /// </summary>
        /// <returns></returns>
        public void Dispose()
        {
            CurrentDeviceMAC = null;
            CurrentService?.Dispose();
            CurrentDevice?.Dispose();

            CurrentDevice = null;
            CurrentService = null;
            CurrentWriteCharacteristic = null;
            CurrentNotifyCharacteristic = null;
            CallbackLog("主动断开连接");
            bluetoothState = BluetoothState.Disconnecting;
            CallbackStatus(bluetoothState);

        }

        private void Timer1_Tick(object sender, EventArgs e)
        {
            StopBleDeviceWatcher();//停止搜索蓝牙
            Trace.WriteLine("停止搜索");
            timer.Stop();
            //throw new NotImplementedException();
        }

        public void StopBleDeviceWatcher()
        {
            this.Watcher?.Stop();
            if (bluetoothState == BluetoothState.Scanning) {
                bluetoothState = BluetoothState.Scanned;//扫描结束
                CallbackStatus(bluetoothState);
            }
        }

        private void OnAdvertisementReceived(BluetoothLEAdvertisementWatcher watcher, BluetoothLEAdvertisementReceivedEventArgs eventArgs)
        {

            Int16 rssi = eventArgs.RawSignalStrengthInDBm;
            BluetoothLEDevice.FromBluetoothAddressAsync(eventArgs.BluetoothAddress).Completed = (asyncInfo, asyncStatus) =>
            {
                if (asyncStatus == AsyncStatus.Completed)
                {
                    if (asyncInfo.GetResults() == null)
                    {
                        //Console.WriteLine("没有得到结果集");
                        //Trace.WriteLine("没有结果集");
                    }
                    else
                    {
                        BluetoothLEDevice currentDevice = asyncInfo.GetResults();
                        if (DeviceList.FindIndex((x) => { return x.Name.Equals(currentDevice.Name); }) < 0)
                        {

                            this.DeviceList.Add(currentDevice);
                            // DeviceWatcherChanged?.Invoke(currentDevice);
                            DeviceWatcherChanged(currentDevice, rssi.ToString());
                        }
                        currentDevice.Dispose();

                    }

                }

            };
        }

        /*
         * 设备扫描到的回调
         */
        private void DeviceWatcherChanged(BluetoothLEDevice currentDevice, string signal)
        {
            if (bluetoothState == BluetoothState.Scanning)//只有在处于扫描状态下才把扫描到的数据回调
            {
                byte[] _Bytes1 = BitConverter.GetBytes(currentDevice.BluetoothAddress);
                Array.Reverse(_Bytes1);
                string address = BitConverter.ToString(_Bytes1, 2, 6).Replace('-', ':').ToLower();
                string name = currentDevice.Name.Contains(address) ? "N/A" : currentDevice.Name;
                CallbackDevice(name, address);
                Trace.WriteLine("发现设备：<" + name + ">  address:<" + address + ">" + "  信号值：<" + signal + ">");
            }
        }

        private void MatchingBluetoothAsync(string Id)
        {
            string log = "准备连接设备: " + Id;
            CallbackLog(log);
            try
            {
                BluetoothLEDevice.FromIdAsync(Id).Completed = (asyncInfo, asyncStatus) =>
                {
                    log = "连接结果: " + asyncStatus.ToString();
                    CallbackLog(log);
                    if (asyncStatus == AsyncStatus.Completed)
                    {
                        BluetoothLEDevice bleDevice = asyncInfo.GetResults();
                        //在当前设备变量中保存检测到的设备。
                        CurrentDevice = bleDevice;
                        Thread.Sleep(2000);//连接上蓝牙后等待两秒再寻找服务
                        Connect();

                    }
                    else
                    {
                        Dispose();
                        bluetoothState = BluetoothState.Error;
                        log = "连接失败: 可能设备超出连接范围，请调整距离再试试看";
                        CallbackStatus(bluetoothState, log);
                        CallbackLog(log);
                    }
                };
            }
            catch (Exception e)
            {
                string msg = "没有发现设备" + e.ToString();
                CallbackLog(msg);
            }

        }

        private void Connect()
        {
            string msg = "正在连接设备<" + CurrentDeviceMAC + ">..";
            CallbackLog(msg);
            CurrentDevice.ConnectionStatusChanged += CurrentDevice_ConnectionStatusChanged;
            SelectDeviceService();

        }



        static object lockObj = new object();//定义线程锁
        private void CurrentDevice_ConnectionStatusChanged(BluetoothLEDevice sender, object args)
        {
            
            lock(lockObj)//加个锁，防止短时间内频繁连接断开导致多个线程抢占这下面代码块
            {
                if (sender.ConnectionStatus == BluetoothConnectionStatus.Disconnected)
                {
                    if (!asyncLock) return;//防止短时间内重复触发下面代码
                    string msg = "设备断开";
                    Trace.WriteLine(msg);
                    CallbackLog(msg);
                    asyncLock = false;
                    CurrentDevice?.Dispose();
                    CurrentDevice = null;
                    CurrentService = null;
                    CurrentWriteCharacteristic = null;
                    CurrentNotifyCharacteristic = null;
                    //设备断开
                    if (CurrentDeviceMAC != null)
                    {
                        CallbackLog("设备意外断开");
                        bluetoothState = BluetoothState.Error;
                        CallbackStatus(bluetoothState, "设备意外断开");
                    }
                    bluetoothState = BluetoothState.Disconnected;
                    CallbackStatus(bluetoothState);
                    
                }
                //else if (sender.ConnectionStatus == BluetoothConnectionStatus.Connected)
                //{
                //    if(CurrentWriteCharacteristic == null)//没有发送句柄，重新连接。出现此情况是是信号弱
                //    {
                //        if (isReconnection)
                //        {
                //            //Reconnection();
                //            CallbackLog("原计划为重新连接..");
                //        }
                //        else
                //        {
                //            Dispose();
                //            bluetoothState = BluetoothState.Error;
                //            CallbackStatus(bluetoothState,"连接失败: 可能原因，信号过弱，请尝试缩短距离");
                //            CallbackLog("连接失败，可能信号较弱");
                //        }
                //        return;
                //    }
                //    if (!asyncLock)
                //    {
                //        CallbackLog("或许连接成功?被锁排斥!");
                //    }
                //    if (asyncLock) return;
                //    asyncLock = true;
                //    string msg = "设备已连接";
                //    Trace.WriteLine(msg);
                //    CallbackLog(msg);
                //    bluetoothState = BluetoothState.Connected;
                //    CallbackStatus(bluetoothState, CurrentDeviceMAC);
                //}
                //else 
                //{
                //    CallbackLog("Bluetooth Connect state is " + sender.ConnectionStatus);
                //}
            }
        }


        /// <summary>
        /// 按GUID 查找主服务
        /// </summary>
        /// <param name="characteristic">GUID 字符串</param>
        /// <returns></returns>
        private void SelectDeviceService()
        {
            if(failNumber > 4)
            {
                if (isReconnection)
                {
                    Reconnection();//尝试再连接一遍
                }
                else
                {
                    ConnectionFail("连接失败: 没有发现指定服务");//连接失败
                }
                return;
            }
            string serviceGuid = failNumber % 2 == 0 ? ServiceGuid : ServiceGuid02;
            CallbackLog(serviceGuid);
            Trace.WriteLine("尝试服务UUID: "+serviceGuid);
            Guid guid = new Guid(serviceGuid);
            if(CurrentDevice == null)
            {
                String msg = "CurrentDevice is null";
                Trace.WriteLine(msg);
                CallbackLog(msg);
                Reconnection();
                return;
            }
            CurrentDevice.GetGattServicesForUuidAsync(guid).Completed = (asyncInfo, asyncStatus) =>
            {
                if (asyncStatus == AsyncStatus.Completed)
                {
                    try
                    {
                        GattDeviceServicesResult result = asyncInfo.GetResults();
                        string msg = "主服务=" + CurrentDevice.ConnectionStatus.ToString();
                        CallbackLog(msg);
                        if (result.Services.Count > 0)
                        {
                            CurrentService = result.Services[CHARACTERISTIC_INDEX];
                            if (CurrentService != null)
                            {
                                SetCharacteristic(serviceGuid);
                                SetCurrentWriteCharacteristic();
                                SetCurrentNotifyCharacteristic();

                            }
                        }
                        else
                        {
                            failNumber++;
                            msg = "没有发现服务,自动重试中";
                            CallbackLog(msg);
                            SelectDeviceService();
                        }
                    }
                    catch (Exception)
                    {
                        failNumber++;
                        CallbackLog("没有发现服务,自动重试中");
                        SelectDeviceService();

                    }
                }
            };
        }
        
        /// <summary>
        /// 重新连接一次
        /// </summary>
        private void Reconnection()
        {
            Trace.WriteLine("尝试重连");
            CallbackLog("尝试重连");
            Dispose();
            bluetoothState = BluetoothState.Connecting;
            CallbackStatus(bluetoothState);
            Thread.Sleep(2000);
            SelectDeviceFromIdAsync(CurrentDeviceMAC);
            isReconnection = false;
        }

        /// <summary>
        /// 连接失败
        /// </summary>
        /// <param name="msg"></param>
        private void ConnectionFail(string msg)
        {
            Dispose();
            bluetoothState = BluetoothState.Error;
            CallbackStatus(bluetoothState, msg);
        }


        /// <summary>
        /// 设置写特征对象。
        /// </summary>
        /// <returns></returns>
        private void SetCurrentWriteCharacteristic()
        {
            if(failNumber > 3)
            {
                if (isReconnection)
                {
                    Reconnection();
                }
                else
                {
                    ConnectionFail("连接失败: 没有发现写入的特征值");
                }
                return;
            }

            string msg = "";
            Guid guid = new Guid(WriteCharacteristicGuid);
            if(CurrentService == null)
            {
                Reconnection();
                return;
            }
            CurrentService.GetCharacteristicsForUuidAsync(guid).Completed =  (asyncInfo, asyncStatus) =>
            {
                if (asyncStatus == AsyncStatus.Completed)
                {
                    GattCharacteristicsResult result = asyncInfo.GetResults();
                    msg = "特征对象=" + CurrentDevice.ConnectionStatus.ToString();
                    CallbackLog(msg);
                    if (result.Characteristics.Count > 0)
                    {
                        CurrentWriteCharacteristic = result.Characteristics[CHARACTERISTIC_INDEX];
                    }
                    else
                    {
                        failNumber++;
                        msg = "没有发现特征对象,自动重试中";
                        CallbackLog(msg);
                        SetCurrentWriteCharacteristic();
                    }
                }
            };
        }




        

        /// <summary>
        /// 设置通知特征对象。
        /// </summary>
        /// <returns></returns>
        private void SetCurrentNotifyCharacteristic()
        {

            if (failNumber > 3)
            {
                Dispose();
                bluetoothState = BluetoothState.Error;
                CallbackStatus(bluetoothState, "连接失败: 没有发现通知的特征值");
                return;
            }
            string msg = "";
            Guid guid = new Guid(NotifyCharacteristicGuid);
            if (CurrentService == null)
            {
                Reconnection();
                return;
            }
            CurrentService.GetCharacteristicsForUuidAsync(guid).Completed = (asyncInfo, asyncStatus) =>
            {
                if (asyncStatus == AsyncStatus.Completed)
                {
                    GattCharacteristicsResult result = asyncInfo.GetResults();
                    msg = "特征对象=" + CurrentDevice.ConnectionStatus.ToString();
                    CallbackLog(msg);
                    if (result.Characteristics.Count > 0)
                    {
                        CurrentNotifyCharacteristic = result.Characteristics[CHARACTERISTIC_INDEX];
                        CurrentNotifyCharacteristic.ProtectionLevel = GattProtectionLevel.Plain;
                        CurrentNotifyCharacteristic.ValueChanged += Characteristic_ValueChanged;
                        EnableNotifications(CurrentNotifyCharacteristic);

                    }
                    else
                    {
                        failNumber++;
                        msg = "没有发现特征对象,自动重试中";
                        CallbackLog(msg);
                        SetCurrentNotifyCharacteristic();
                    }
                }
            };
        }

        /// <summary>
        /// 设置特征对象为接收通知对象
        /// </summary>
        /// <param name="characteristic"></param>
        /// <returns></returns>
        private void EnableNotifications(GattCharacteristic characteristic)
        {
            string msg = "收通知对象=" + CurrentDevice.ConnectionStatus;
            CallbackLog(msg);

            characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(CHARACTERISTIC_NOTIFICATION_TYPE).Completed = (asyncInfo, asyncStatus) =>
            {
                if (asyncStatus == AsyncStatus.Completed)
                {
                    GattCommunicationStatus status = asyncInfo.GetResults();
                    
                    if (status == GattCommunicationStatus.Unreachable)
                    {
                        msg = "此设备连接出错:无法设置指定特征对象为接收通知对象";
                        CallbackLog(msg);
                        bluetoothState = BluetoothState.Error;
                        CallbackStatus(bluetoothState,msg);
                        Dispose();
                        //if (CurrentNotifyCharacteristic != null && !asyncLock)
                        //{
                        //    EnableNotifications(CurrentNotifyCharacteristic);
                        //}
                    } else if(status == GattCommunicationStatus.Success)
                    {
                        if (CurrentWriteCharacteristic == null)//没有发送句柄，重新连接。出现此情况是是信号弱
                        {
                            if (isReconnection)
                            {
                                Reconnection();
                                CallbackLog("写入句柄缺失，重新连接..");
                            }
                            else
                            {
                                Dispose();
                                bluetoothState = BluetoothState.Error;
                                CallbackStatus(bluetoothState, "连接失败: 可能原因，信号过弱，请尝试缩短距离");
                                CallbackLog("连接失败，可能信号较弱");
                            }
                            return;
                        }
                        CallbackLog("判定连接成功...");
                        asyncLock = true;
                        msg = "设备已连接";
                        Trace.WriteLine(msg);
                        CallbackLog(msg);
                        bluetoothState = BluetoothState.Connected;
                        CallbackStatus(bluetoothState, CurrentDeviceMAC);
                    }
                    msg = "设备连接状态:" + status.ToString();
                    CallbackLog(msg);
                    
                }
            };
        }

        private void Characteristic_ValueChanged(GattCharacteristic sender, GattValueChangedEventArgs args)
        {
            byte[] data;
            CryptographicBuffer.CopyToByteArray(args.CharacteristicValue, out data);
            //string str = BitConverter.ToString(data);
            CallbackBlutoothData(data);

        }

        /// <summary>
        /// 通过服务来设置写入与监听的特征值
        /// </summary>
        /// <param name="serviceGuid"></param>
        private void SetCharacteristic(string serviceGuid)
        {
            lock (lockObj)
            {
                if (ServiceGuid.Equals(serviceGuid))
                {
                    WriteCharacteristicGuid = "0000ffe1-0000-1000-8000-00805f9b34fb";
                    NotifyCharacteristicGuid = "0000ffe1-0000-1000-8000-00805f9b34fb";
                }
                else if (ServiceGuid02.Equals(serviceGuid))//02的服务
                {
                    WriteCharacteristicGuid = "49535343-8841-43F4-A8D4-ECBE34729BB3";
                    NotifyCharacteristicGuid = "49535343-1E4D-4BD9-BA61-23C647249616";
                }
                else
                {
                    bluetoothState = BluetoothState.Error;
                    CallbackStatus(bluetoothState, "连接出错:服务错误: " + serviceGuid);
                    Dispose();
                }
            }
        }


        private void handlerSendData()
        {
            if (sendDataArray.Count == 0) return;
            byte[] data = sendDataArray[0];
            sendDataArray.RemoveAt(0);
            if (CurrentWriteCharacteristic != null)
            {
                CurrentWriteCharacteristic.WriteValueAsync(CryptographicBuffer.CreateFromByteArray(data), GattWriteOption.WriteWithResponse).Completed = (asyncInfo, asyncStatus) =>
                {
                    if (asyncStatus == AsyncStatus.Completed)
                    {
                        CallbackSendNum(data.Length);
                        if (sendDataArray.Count > 0) handlerSendData();
                    }
                };
            }
            else
            {
                CallbackLog("写入出错，CurrentWriteCharacteristic 为空");
            }
        }

        private void SplitArray(byte[] bytes)
        {
            int BatchSize = 50;
            int groupNum = bytes.Length / BatchSize;

            for (int i = 0; i < groupNum; i++)
            {
                byte[] temp = new byte[BatchSize];
                Array.Copy(bytes, i * 10, temp, 0, BatchSize);
                //Trace.WriteLine("数组分包: "+ BitConverter.ToString(temp));
                sendDataArray.Add(temp);
            }
            int len = bytes.Length % BatchSize;
            if (len > 0)
            {
                byte[] temp = new byte[len];
                Array.Copy(bytes, groupNum * 10, temp, 0, len);
                sendDataArray.Add(temp);
            }
        }

        private void CallbackLog(string log)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.NotifyTxt, log,null);
        }

        private void CallbackDevice(string name,string mac)
        {
            if (bluetoothState == BluetoothState.Connecting || bluetoothState == BluetoothState.Connected) return;
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothDevice, name, mac);
        }

        private void CallbackBlutoothData(byte[] bytes)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothData, "", bytes);
        }

        private void CallbackStatus(BluetoothState state)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothStatus, "",state);
        }

        /// <summary>
        /// 传递一些必要信息
        /// </summary>
        /// <param name="state">目前只在Error,Connected状态下会有</param>
        /// <param name="msg">传递一些必要信息</param>
        private void CallbackStatus(BluetoothState state,string msg)
        {

            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothStatus, msg, state);
            
        }

        private void CallbackSendNum(int num)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.SendNum, "", num);
        }

    }

}
