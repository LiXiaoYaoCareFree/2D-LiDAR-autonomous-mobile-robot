using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using static BluetoothApp.BluetoothInfo;

namespace BluetoothApp
{
    public class BluetoothManage
    {
        private const string _serviceGuid = "0000ffe0-0000-1000-8000-00805f9b34fb";
        private const string _serviceGuid02 = "49535343-FE7D-4AE5-8FA9-9FAFD205E455";
        private BleBluetoothInfo bleBluetoothInfo;
        private ClassicBluetooth classicBluetooth;
        private BluetoothInterface bluetoothInterface;

        public BluetoothManage(Form f)
        {
            InitBleBluetoothInfo(f);
            InitClassicBluetooth(f);
            bluetoothInterface = classicBluetooth;
        }

        /// <summary>
        /// 检查蓝牙信息，0：正常，1：蓝牙未开启，2：蓝牙不存在
        /// </summary>
        /// <returns></returns>
        public static int GetBluetoothStatus()
        {
            return CheckBluetoothStatus.GetBluetoothStatus();
        }

        public bool IsConnected()
        {
            if (bluetoothInterface == null) return false;
            return bluetoothInterface.GetBluetoothState() == BluetoothState.Connected;
        }

        public void BluetoothMode(int mode)
        {
            if (mode == 0) bluetoothInterface = classicBluetooth;
            if (mode == 1) bluetoothInterface = bleBluetoothInfo;
        }

        public void SetListener(eventRun callback)
        {
            bleBluetoothInfo.CallbackData += callback;
            classicBluetooth.CallbackData += callback;
        }

        public bool Scan()
        {
            if(isConnecting())
            {
                MessageBox.Show("处于连接中状态无法扫描蓝牙");
                return false;
            }
            bluetoothInterface.StartBleDeviceWatcher();
            return true;
        }

        public void Connect(String mac)
        {
            if(isConnecting())
            {
                MessageBox.Show("请先断开当前蓝牙再执行连接蓝牙操作");
                return;
            }
            
            bluetoothInterface.SelectDeviceFromIdAsync(mac);
        }

        public void Send(byte[] data)
        {
            if (bluetoothInterface.GetBluetoothState() == BluetoothState.Connected)
            {
                bluetoothInterface.Write(data);
            }
            else
            {
                MessageBox.Show("请先连接上蓝牙，再执行发送的操作");
            }
        }

        public void Dispose()
        {
            bluetoothInterface.Dispose();
        }

        /// <summary>
        /// 是否正处于连接中或是已连接
        /// </summary>
        /// <returns></returns>
        private bool isConnecting()
        {
            return bluetoothInterface.GetBluetoothState() == BluetoothState.Connected
                || bluetoothInterface.GetBluetoothState() == BluetoothState.Connecting;
        }

        private void InitBleBluetoothInfo(Form form)
        {
            bleBluetoothInfo = new BleBluetoothInfo(form,_serviceGuid,_serviceGuid02);
        }
        
        private void InitClassicBluetooth(Form form)
        {
            classicBluetooth = new ClassicBluetooth(form);
        }

    }
}
