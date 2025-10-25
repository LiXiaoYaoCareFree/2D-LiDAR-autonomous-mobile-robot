using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace BluetoothApp
{
    class Utility
    {
        /// <summary>
        /// 将byte数组转string
        /// </summary>
        /// <param name="bytes"></param>
        /// <param name="isHex"> 是否需要16进制格式</param>
        /// <returns></returns>
        public static string GetByteArrayToString(byte[] bytes,bool isHex)
        {
            if (isHex) return BytesToHexStr(bytes);

            return Encoding.Default.GetString(bytes);
        }

        public static string getHexToString(string data)
        {
            data = data.Replace("00", string.Empty).Replace(" ", string.Empty);
            StringBuilder stringBuilder = new StringBuilder();
            byte[] b = new byte[data.Length / 2];
            for (int i = 0; i < data.Length / 2; i++)
            {
                stringBuilder.Clear();
                stringBuilder.Append(data.Substring(i * 2, 2));
                b[i] = Convert.ToByte(stringBuilder.ToString(), 16);
            }
            return Encoding.Default.GetString(b);
        }

        /// <summary>
        /// 将string转byte数组
        /// </summary>
        /// <param name="data"></param>
        /// <param name="isHex">是否需要16进制格式</param>
        /// <returns></returns>
        public static byte[] GetStringToByteArray(string data, bool isHex)
        {
            if (isHex) return StringToHexBytes(data);
            return Encoding.Default.GetBytes(data);
        }


        // <summary>
        /// 动态加载资源
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="args"></param>
        /// <returns></returns>
        public static Assembly CurrentDomain_AssemblyResolve(object sender, ResolveEventArgs args)
        {
            //默认命名空间+文件夹名+.[注意]
            string resourceName = "BluetoothApp.lib." + new AssemblyName(args.Name).Name + ".dll";

            // LogHelper.Debug("在加载资源：" + resourceName);
            //Trace.WriteLine("加载资源: " + resourceName);
            using (var stream = Assembly.GetExecutingAssembly().GetManifestResourceStream(resourceName))
            {
                if (stream != null)
                {
                    byte[] assemblyData = new byte[stream.Length];
                    stream.Read(assemblyData, 0, assemblyData.Length);
                    return Assembly.Load(assemblyData);
                }

                return null;
            }
        }

        /// <summary>
        /// 判断当前系统版本是否大于8
        /// </summary>
        /// <returns></returns>
        public static bool IsWindows8_OrHigher()
        {
            Version currentVersion = Environment.OSVersion.Version;
            Version compareToVersion = new Version("6.2");
            if (currentVersion.CompareTo(compareToVersion) >= 0)
            {//win8及其以上版本的系统
                Console.WriteLine("当前系统是WIN8及以上版本系统。");
                return true;
            }
            else
            {
                Console.WriteLine("当前系统不是WIN8及以上版本系统。");
                return false;
            }
        }

        private static byte[] StringToHexBytes(string data)
        {
            data = data.Replace(" ", string.Empty);
            StringBuilder stringBuilder = new StringBuilder();
            byte[] b = new byte[data.Length / 2];
            for (int i = 0; i < data.Length / 2; i++)
            {
                stringBuilder.Clear();
                stringBuilder.Append(data.Substring(i * 2, 2));
                b[i] = Convert.ToByte(stringBuilder.ToString(), 16);
            }
            return b;
        }

        public static string getHexString(string data)
        {
            byte[] bytes = Encoding.Default.GetBytes(data);
            return BitConverter.ToString(bytes).Replace("-", " ");
        }

        //将byte数组转化为16进制字符串
        private static string BytesToHexStr(byte[] bytes)
        {
            string returnStr = "";
            if (bytes == null) return returnStr;


            for (int i = 0; i < bytes.Length; i++)
            {
                returnStr += " " + bytes[i].ToString("X2");
            }

            return returnStr;
        }
    }
}
