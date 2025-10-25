using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static BluetoothApp.BluetoothInfo;

namespace BluetoothApp
{
    public interface BluetoothInterface
    {
        /// <summary>
        /// 蓝牙的当前状态
        /// </summary>
        /// <returns></returns>
        BluetoothState GetBluetoothState();



        /// <summary>
        /// 搜索蓝牙设备
        /// </summary>
        void StartBleDeviceWatcher();


        /// <summary>
        /// 按蓝牙的mac连接蓝牙
        /// </summary>
        /// <param name="MAC"></param>
        /// <returns></returns>
        void SelectDeviceFromIdAsync(string mac);


        /// <summary>
        /// 发送数据接口
        /// </summary>
        /// <param name="characteristic"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        void Write(byte[] data);



        /// <summary>
        /// 主动断开连接
        /// </summary>
        /// <returns></returns>
        void Dispose();

    }
}
