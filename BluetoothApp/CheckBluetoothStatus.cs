using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Management;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using InTheHand.Net.Bluetooth;

namespace BluetoothApp
{
    public class CheckBluetoothStatus
    {
        private static BluetoothRadio radio;
        //检查蓝牙是否开启   0：正常，1：蓝牙未开启， 2: 蓝牙不存在不支持
        public static int GetBluetoothStatus()
        {

            if (!BluetoothExist())
            {
                return 2;
            }

            //获取电脑蓝牙
            if(radio == null) radio = BluetoothRadio.PrimaryRadio;

            if (radio != null)
            {
                Trace.WriteLine("蓝牙已经打开");
                return 0;
            }
            else
            {
                Trace.WriteLine("未找到本机蓝牙设备!");
                return 1;
            }
        }

        //是否支持蓝牙
        public static bool BluetoothExist()
        {
            return System.Runtime.InteropServices.Marshal.GetHRForLastWin32Error() != 0;
        }

        public static BluetoothRadio GetRadio()
        {
            if (radio == null) radio = BluetoothRadio.PrimaryRadio;
            return radio;
        }

        /**
        * 
        * 显示本地蓝牙设备信息
        * 
        * **/
        private static void localBluetoothInfo(BluetoothRadio bluetoothRadio)
        {
            Trace.WriteLine("ClassOfDevice: " + bluetoothRadio.ClassOfDevice);
            Trace.WriteLine("HardwareStatus: " + bluetoothRadio.HardwareStatus);
            Trace.WriteLine("HciRevision: " + bluetoothRadio.HciRevision);
            Trace.WriteLine("HciVersion: " + bluetoothRadio.HciVersion);
            Trace.WriteLine("LmpSubversion: " + bluetoothRadio.LmpSubversion);
            Trace.WriteLine("LmpVersion: " + bluetoothRadio.LmpVersion);
            Trace.WriteLine("LocalAddress: " + bluetoothRadio.LocalAddress);
            Trace.WriteLine("Manufacturer: " + bluetoothRadio.Manufacturer);
            Trace.WriteLine("Mode: " + bluetoothRadio.Mode);
            Trace.WriteLine("Name: " + bluetoothRadio.Name);
            Trace.WriteLine("Remote: " + bluetoothRadio.Remote);
            Trace.WriteLine("SoftwareManufacturer: " + bluetoothRadio.SoftwareManufacturer);
            Trace.WriteLine("StackFactory: " + bluetoothRadio.StackFactory);
            
        }



    }
}
