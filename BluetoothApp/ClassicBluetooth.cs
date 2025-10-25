using InTheHand.Net;
using InTheHand.Net.Bluetooth;
using InTheHand.Net.Bluetooth.Factory;
using InTheHand.Net.Sockets;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using static BluetoothApp.BluetoothInfo;
using static InTheHand.Net.Sockets.BluetoothClient;

namespace BluetoothApp
{
    public class ClassicBluetooth:BluetoothInterface
    {
        private Form form;
        private BluetoothRadio radio;
        private NetworkStream stream;
        private BluetoothClient client;
        private byte[] readBytes = new byte[2048];
        private BluetoothState bluetoothState = BluetoothState.Scanned;
        private const string GUID = "00001101-0000-1000-8000-00805f9b34fb";
        private readonly Dictionary<string, BluetoothAddress> devicesInfo = new Dictionary<string, BluetoothAddress>();

        //定义一个事件
        public event eventRun CallbackData;

        public ClassicBluetooth(Form f)
        {
            this.form = f;
            InitBluetooth();
        }

        /// <summary>
        /// 蓝牙的当前状态
        /// </summary>
        /// <returns></returns>
        public BluetoothState GetBluetoothState()
        {
            return bluetoothState;
        }

        public void StartBleDeviceWatcher()
        {
            if (client == null) client = new BluetoothClient();

            BluetoothComponent component = new BluetoothComponent(client);

            component.DiscoverDevicesAsync(30, false, false, false, true, component);

            component.DiscoverDevicesProgress += BluetoothDivceCallback;

            component.DiscoverDevicesComplete += DevicesCompleteCallback;

            devicesInfo.Clear();//清除保存的蓝牙设备信息

            bluetoothState = BluetoothState.Scanning;
            CallbackStatus(bluetoothState);
        }

        public void SelectDeviceFromIdAsync(string mac)
        {
            if (client == null) client = new BluetoothClient();
            bluetoothState = BluetoothState.Connecting;
            CallbackStatus(bluetoothState);
            BluetoothEndPoint endPoint = new BluetoothEndPoint(devicesInfo[mac], Guid.Parse(GUID));
            CallbackLog("尝试连接蓝牙");
            Trace.WriteLine("尝试连接蓝牙");
            //client.SetPin("1234");//默认配对码
            client.BeginConnect(endPoint, AsyncCallback, mac);
        }

        public void Write(byte[] data)
        {
            stream.BeginWrite(data, 0, data.Length, SendDataAsyncCallback, data.Length);
        }

        public void Dispose()
        {
            client?.Dispose();
            client = null;
            Trace.WriteLine("断开连接");
            CallbackLog("断开连接");
            bluetoothState = BluetoothState.Disconnected;
            CallbackStatus(bluetoothState);     
        }

        private void AsyncCallback(IAsyncResult ar)
        {
            
            if (client.Connected)
            {
                Trace.WriteLine("Authenticate is " + client.Authenticate);
                Trace.WriteLine("Encrypt is " + client.Encrypt);
                Trace.WriteLine("2.0蓝牙连接成功");
                CallbackLog("2.0蓝牙连接成功");
                stream = client.GetStream();
                new Thread(ReceiveData).Start();
                bluetoothState = BluetoothState.Connected;
                CallbackStatus(bluetoothState,(string)ar.AsyncState);
            }
            else
            {
                Trace.WriteLine("连接失败了...");
                CallbackLog("连接失败了...");
                bluetoothState = BluetoothState.Error;
                CallbackStatus(bluetoothState, "连接失败..");
                client.Dispose();
                client = null;
            }
        }

        private void ReceiveData()
        {
            int num = 0;//记录缓存到readBytes字节数组里的数据长度
            while (client!= null && client.Connected)
            {
                try
                {
                    //某些蓝牙发送的数据会断成几节，接收时需要把断了的数据拼接起来
                    num += stream.Read(readBytes, num, readBytes.Length - num);
                    if (num >= readBytes.Length)//数据存满缓存数组，则直接返回
                    {
                        //CallbackLog("读取数据函数返回: " + num + " " + stream.DataAvailable);
                        CallbackBlutoothData(ReducedArray(num));
                        num = 0;
                    }
                    else
                    {
                        Thread.Sleep(50);//等待50ms
                        if (!stream.DataAvailable)//查看后边是否还有数据等待读取
                        {
                            //CallbackLog("读取数据函数返回: " + num);
                            CallbackBlutoothData(ReducedArray(num));
                            num = 0;
                        }
                    }
                }
                catch
                {
                    Trace.WriteLine("监听可能结束");
                    CallbackLog("监听可能结束");
                }
            }

            if (bluetoothState != BluetoothState.Disconnected)
            {
                Trace.WriteLine("意外断线");
                bluetoothState = BluetoothState.Error;
                CallbackStatus(bluetoothState,"意外断线");
            }
        }

        private void SendDataAsyncCallback(IAsyncResult ar)
        {
            //发送完成回调
            int num = (int)ar.AsyncState;
            CallbackSendNum(num);
        }

        private void BluetoothDivceCallback(object sender,DiscoverDevicesEventArgs e)
        {
            string name = e.Devices[0].DeviceName;
            string mac = e.Devices[0].DeviceAddress.ToString("C");
            if (name.Equals(mac)) name = "N/A";
            //保存搜索到的蓝牙设备地址信息
            devicesInfo.Add(mac, e.Devices[0].DeviceAddress);
            Trace.WriteLine("搜索到: " + name+" mac is "+mac);
            CallbackDevice(name, mac);
        }

        private void DevicesCompleteCallback(object sender,DiscoverDevicesEventArgs e)
        {
            if (bluetoothState == BluetoothState.Connecting) return;
            bluetoothState = BluetoothState.Scanned;
            CallbackStatus(bluetoothState);
        }


        private void InitBluetooth()
        {
            client = new BluetoothClient();
            //获取电脑蓝牙
            radio = CheckBluetoothStatus.GetRadio();
            //蓝牙设置可以被搜索到
            radio.Mode = RadioMode.Connectable;

        }

        /// <summary>
        /// 将数组一些无效数据剔除
        /// </summary>
        /// <returns></returns>
        private byte[] ReducedArray(int num)
        {
            //int length = readBytes.Length-1;
            //for (; length > 0; length--)
            //{
            //    if (readBytes[length] != 0) break;
            //}
            //length += 1;
            //byte[] bytes = new byte[length];
            //Array.Copy(readBytes, 0, bytes, 0, length);
            //for(length = 0; length < readBytes.Length; length++)
            //{
            //    readBytes[length] = 0;
            //}
            byte[] bytes = new byte[num];
            Array.Copy(readBytes, 0, bytes, 0, num);
            return bytes;
        }


        private void CallbackLog(string log)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.NotifyTxt, log, null);
        }

        private void CallbackDevice(string name, string mac)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothDevice, name, mac);
        }

        private void CallbackBlutoothData(byte[] bytes)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothData, "", bytes);
        }

        private void CallbackStatus(BluetoothState state)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothStatus, "", state);
        }

        /// <summary>
        /// 传递一些必要信息
        /// </summary>
        /// <param name="state">目前只在Error,Connected状态下会有</param>
        /// <param name="msg">传递一些必要信息</param>
        private void CallbackStatus(BluetoothState state, string msg)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.BluetoothStatus, msg, state);
        }

        private void CallbackSendNum(int num)
        {
            form.BeginInvoke(new eventRun(CallbackData), MsgTypes.SendNum, "", num);
        }

    }
}
