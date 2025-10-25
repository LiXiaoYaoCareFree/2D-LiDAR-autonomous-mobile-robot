using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BluetoothApp
{
    class MyTimer
    {
        private System.Windows.Forms.Timer mTimer;
        private System.Windows.Forms.Timer scanTimer;//扫描动画定时器
        private System.Windows.Forms.Timer connectTimer;//连接动画定时器

        public void start(int time, EventHandler handler)
        {
            if (mTimer == null)
            {
                mTimer = new System.Windows.Forms.Timer();

                mTimer.Tick += handler;
            }

            mTimer.Interval = time;

            mTimer.Enabled = true;
        }

        public void startScanTimer(EventHandler handler)
        {
            if (scanTimer == null)
            {
                scanTimer = new System.Windows.Forms.Timer();

                scanTimer.Tick += handler;

                scanTimer.Interval = 500;
            }

            scanTimer.Enabled = true;
        }

        public void startConnectTimer(EventHandler handler)
        {
            if (connectTimer == null)
            {
                connectTimer = new System.Windows.Forms.Timer();

                connectTimer.Tick += handler;

                connectTimer.Interval = 500;
            }

            connectTimer.Enabled = true;
        }

        public void stop()
        {
            if (mTimer != null)
            {
                Trace.WriteLine("关闭定时器1: "+mTimer.Enabled);
                mTimer.Stop();
                mTimer.Enabled = false;
                
            }
        }

        public void stopScanTimer()
        {
            if(scanTimer != null)
            {
                Trace.WriteLine("关闭定时器2: "+scanTimer.Enabled);
                scanTimer.Stop();
                scanTimer.Enabled = false;
                
            }
        }

        public void stopConnectTimer()
        {
            if(connectTimer != null)
            {
                Trace.WriteLine("关闭定时器3: " + connectTimer.Enabled);
                connectTimer.Stop();
                connectTimer.Enabled = false;
            }
        }

    }
}
