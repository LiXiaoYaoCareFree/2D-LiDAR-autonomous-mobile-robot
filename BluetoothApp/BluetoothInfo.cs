using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BluetoothApp
{
    /// <summary>
    /// 蓝牙的各种交互信息
    /// </summary>
    public class BluetoothInfo
    {

        /// <summary>
        /// 将蓝牙的数据传递到主界面
        /// </summary>
        /// <param name="type">信息类型：日志，设备，状态，数据</param>
        /// <param name="str">string型数据</param>
        /// <param name="data">object：包含BluetoothState，string，byte[]</param>
        public delegate void eventRun(MsgTypes type, string str, object data = null);

        /// <summary>
        /// 蓝牙的各种状态值
        /// </summary>
        public enum BluetoothState
        {
            Scanning,//扫描中
            Scanned,//扫描结束
            Connecting,//连接中
            Connected,//连接完成
            Disconnecting,//正在断开
            Disconnected,//断开
            Error,//发生错误
        }

        /// <summary>
        /// 通信类型值
        /// </summary>
        public enum MsgTypes
        {
            BluetoothDevice,//蓝牙设备
            BluetoothStatus,//蓝牙状态
            BluetoothData,//蓝牙接收到的数据
            NotifyTxt,//日志
            SendNum,//成功发送的数据量
        }

    }
}
