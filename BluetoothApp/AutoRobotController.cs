using System;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Timers;

namespace BluetoothApp
{
    /// <summary>
    /// 自动机器人控制器 - 程序自动发送控制命令
    /// </summary>
    public class AutoRobotController
    {
        #region 控制命令定义（与硬件代码对应）
        public const byte DIR_STOP = 0;      // 停止
        public const byte DIR_FORWARD = 1;   // 前进
        public const byte DIR_BACKWARD = 2;  // 后退
        public const byte DIR_LEFT = 3;      // 左转
        public const byte DIR_RIGHT = 4;     // 右转
        public const byte DIR_UTURN = 5;     // 掉头
        #endregion

        #region 事件定义
        public event EventHandler<string> OnStatusChanged;
        public event EventHandler<byte> OnCommandSent;
        public event EventHandler<Exception> OnError;
        #endregion

        #region 私有字段
        private BluetoothWindow mainForm;
        private RobotController robotController;
        private System.Timers.Timer commandTimer;
        private Queue<RobotCommand> commandQueue;
        private bool isRunning = false;
        private bool isPaused = false;
        private CancellationTokenSource cancellationTokenSource;
        private object lockObject = new object();
        #endregion

        #region 构造函数
        public AutoRobotController(BluetoothWindow form, RobotController controller)
        {
            mainForm = form;
            robotController = controller;
            commandQueue = new Queue<RobotCommand>();
            
            // 初始化命令定时器
            commandTimer = new System.Timers.Timer(100); // 100ms间隔
            commandTimer.Elapsed += CommandTimer_Elapsed;
            commandTimer.AutoReset = true;
        }
        #endregion

        #region 公共方法
        /// <summary>
        /// 开始自动控制
        /// </summary>
        public void StartAutoControl()
        {
            if (isRunning)
            {
                OnStatusChanged?.Invoke(this, "自动控制已在运行中");
                return;
            }

            if (robotController == null || !robotController.IsConnected())
            {
                OnStatusChanged?.Invoke(this, "错误：机器人未连接");
                OnError?.Invoke(this, new Exception("机器人未连接"));
                return;
            }

            isRunning = true;
            isPaused = false;
            cancellationTokenSource = new CancellationTokenSource();
            
            commandTimer.Start();
            OnStatusChanged?.Invoke(this, "自动控制已启动");
        }

        /// <summary>
        /// 停止自动控制
        /// </summary>
        public void StopAutoControl()
        {
            if (!isRunning) return;

            isRunning = false;
            isPaused = false;
            commandTimer.Stop();
            cancellationTokenSource?.Cancel();
            
            // 清空命令队列
            lock (lockObject)
            {
                commandQueue.Clear();
            }
            
            OnStatusChanged?.Invoke(this, "自动控制已停止");
        }

        /// <summary>
        /// 暂停/恢复自动控制
        /// </summary>
        public void TogglePause()
        {
            if (!isRunning) return;

            isPaused = !isPaused;
            string status = isPaused ? "自动控制已暂停" : "自动控制已恢复";
            OnStatusChanged?.Invoke(this, status);
        }

        /// <summary>
        /// 添加单个命令到队列
        /// </summary>
        public void AddCommand(byte direction, int duration = 1000)
        {
            if (!isRunning) return;

            var command = new RobotCommand
            {
                Direction = direction,
                Duration = duration,
                Timestamp = DateTime.Now
            };

            lock (lockObject)
            {
                commandQueue.Enqueue(command);
            }

            OnStatusChanged?.Invoke(this, $"添加命令: {GetDirectionName(direction)} ({duration}ms)");
        }

        /// <summary>
        /// 添加命令序列到队列
        /// </summary>
        public void AddCommandSequence(List<RobotCommand> commands)
        {
            if (!isRunning || commands == null) return;

            lock (lockObject)
            {
                foreach (var cmd in commands)
                {
                    commandQueue.Enqueue(cmd);
                }
            }

            OnStatusChanged?.Invoke(this, $"添加命令序列: {commands.Count} 个命令");
        }

        /// <summary>
        /// 执行迷宫探索序列
        /// </summary>
        public void ExecuteMazeExplorationSequence()
        {
            if (!isRunning) return;

            var explorationCommands = GenerateMazeExplorationCommands();
            AddCommandSequence(explorationCommands);
            OnStatusChanged?.Invoke(this, "开始执行迷宫探索序列");
        }

        /// <summary>
        /// 执行SLAM建图序列
        /// </summary>
        public void ExecuteSLAMMappingSequence()
        {
            if (!isRunning) return;

            var slamCommands = GenerateSLAMMappingCommands();
            AddCommandSequence(slamCommands);
            OnStatusChanged?.Invoke(this, "开始执行SLAM建图序列");
        }

        /// <summary>
        /// 清空命令队列
        /// </summary>
        public void ClearCommandQueue()
        {
            lock (lockObject)
            {
                commandQueue.Clear();
            }
            OnStatusChanged?.Invoke(this, "命令队列已清空");
        }
        #endregion

        #region 私有方法
        private void CommandTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            if (!isRunning || isPaused) return;

            try
            {
                RobotCommand currentCommand = null;
                
                lock (lockObject)
                {
                    if (commandQueue.Count > 0)
                    {
                        currentCommand = commandQueue.Peek();
                        
                        // 检查命令是否执行完成
                        if (DateTime.Now - currentCommand.Timestamp >= TimeSpan.FromMilliseconds(currentCommand.Duration))
                        {
                            commandQueue.Dequeue();
                            currentCommand = null;
                        }
                    }
                }

                // 执行当前命令
                if (currentCommand != null)
                {
                    ExecuteCommand(currentCommand.Direction);
                }
                else if (commandQueue.Count == 0)
                {
                    // 队列为空，发送停止命令
                    ExecuteCommand(DIR_STOP);
                }
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        private void ExecuteCommand(byte direction)
        {
            try
            {
                // 通过主窗体发送命令
                switch (direction)
                {
                    case DIR_STOP:
                        mainForm.StopRobot();
                        break;
                    case DIR_FORWARD:
                        mainForm.MoveForward();
                        break;
                    case DIR_BACKWARD:
                        mainForm.MoveBackward();
                        break;
                    case DIR_LEFT:
                        mainForm.TurnLeft();
                        break;
                    case DIR_RIGHT:
                        mainForm.TurnRight();
                        break;
                    case DIR_UTURN:
                        mainForm.UTurn();
                        break;
                }

                OnCommandSent?.Invoke(this, direction);
            }
            catch (Exception ex)
            {
                OnError?.Invoke(this, ex);
            }
        }

        private List<RobotCommand> GenerateMazeExplorationCommands()
        {
            var commands = new List<RobotCommand>();
            
            // 迷宫探索的基本模式：前进-左转-前进-右转-前进-掉头
            commands.Add(new RobotCommand { Direction = DIR_FORWARD, Duration = 2000 });
            commands.Add(new RobotCommand { Direction = DIR_LEFT, Duration = 1000 });
            commands.Add(new RobotCommand { Direction = DIR_FORWARD, Duration = 1500 });
            commands.Add(new RobotCommand { Direction = DIR_RIGHT, Duration = 1000 });
            commands.Add(new RobotCommand { Direction = DIR_FORWARD, Duration = 2000 });
            commands.Add(new RobotCommand { Direction = DIR_UTURN, Duration = 2000 });
            commands.Add(new RobotCommand { Direction = DIR_FORWARD, Duration = 1000 });
            commands.Add(new RobotCommand { Direction = DIR_STOP, Duration = 500 });

            return commands;
        }

        private List<RobotCommand> GenerateSLAMMappingCommands()
        {
            var commands = new List<RobotCommand>();
            
            // SLAM建图的系统化扫描模式
            for (int i = 0; i < 4; i++) // 4个方向扫描
            {
                commands.Add(new RobotCommand { Direction = DIR_FORWARD, Duration = 1000 });
                commands.Add(new RobotCommand { Direction = DIR_LEFT, Duration = 1000 });
                commands.Add(new RobotCommand { Direction = DIR_FORWARD, Duration = 500 });
                commands.Add(new RobotCommand { Direction = DIR_RIGHT, Duration = 2000 }); // 180度扫描
                commands.Add(new RobotCommand { Direction = DIR_LEFT, Duration = 1000 });
            }
            
            commands.Add(new RobotCommand { Direction = DIR_STOP, Duration = 1000 });

            return commands;
        }

        private string GetDirectionName(byte direction)
        {
            switch (direction)
            {
                case DIR_STOP: return "停止";
                case DIR_FORWARD: return "前进";
                case DIR_BACKWARD: return "后退";
                case DIR_LEFT: return "左转";
                case DIR_RIGHT: return "右转";
                case DIR_UTURN: return "掉头";
                default: return "未知";
            }
        }
        #endregion

        #region 属性
        public bool IsRunning => isRunning;
        public bool IsPaused => isPaused;
        public int QueueCount => commandQueue.Count;
        #endregion

        #region 资源释放
        public void Dispose()
        {
            StopAutoControl();
            commandTimer?.Dispose();
            cancellationTokenSource?.Dispose();
        }
        #endregion
    }

    /// <summary>
    /// 机器人命令结构
    /// </summary>
    public class RobotCommand
    {
        public byte Direction { get; set; }
        public int Duration { get; set; } // 持续时间（毫秒）
        public DateTime Timestamp { get; set; }
    }
}
